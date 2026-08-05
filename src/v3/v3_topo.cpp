// v3 topology: conversion BRep <-> v3::Solid, region queries, tessellation.
//
// Output-chart contract (to_brep): analytic surfaces are emitted as the
// project's quarter-arc NURBS charts with set_domain() applied, so the output
// chart UV == v3 analytic UV AFFINELY (u = angle in radians, v = height /
// latitude / minor angle). Pcurves transfer EXACTLY — nothing is re-fitted.
// Chart normals can differ in sign from Srf::normal; per-surface sign sigma is
// measured and folded into the emitted face `reversed` flags.

#include "v3_topo.h"
#include "../primitives.h"
#include <algorithm>
#include <cmath>
#include <map>

using session_cpp::BRep;
using session_cpp::NurbsCurve;
using session_cpp::NurbsSurface;
using session_cpp::Point;
using session_cpp::Primitives;
using session_cpp::Vector;
using session_cpp::Xform;

namespace v3 {

// ============================================================================
// Solid utils
// ============================================================================

void Solid::bbox(V3& lo, V3& hi) const {
    lo = {1e300, 1e300, 1e300};
    hi = {-1e300, -1e300, -1e300};
    auto grow = [&](const V3& p) {
        lo.x = std::min(lo.x, p.x); hi.x = std::max(hi.x, p.x);
        lo.y = std::min(lo.y, p.y); hi.y = std::max(hi.y, p.y);
        lo.z = std::min(lo.z, p.z); hi.z = std::max(hi.z, p.z);
    };
    for (auto& v : verts) grow(v.p);
    // edge curves (seam-vertex-only solids like spheres/tori need these)
    for (auto& e : edges) {
        if (e.degenerate) continue;
        for (int i = 0; i <= 16; i++) grow(e.c.eval(e.t0 + (e.t1 - e.t0) * i / 16));
    }
    // curved faces: their bulges are not covered by edges (sphere equator etc.)
    for (auto& f : faces) {
        const Srf& s = srfs[f.srf];
        if (s.k == Srf::PLANE) continue;
        double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
        for (const Loop& l : f.loops)
            for (int cei : l.ces)
                for (auto& q : coedges[cei].pc) {
                    u0 = std::min(u0, q.u); u1 = std::max(u1, q.u);
                    v0 = std::min(v0, q.v); v1 = std::max(v1, q.v);
                }
        if (u0 > u1) continue;
        for (int i = 0; i <= 6; i++)
            for (int j = 0; j <= 6; j++) {
                double u = u0 + (u1 - u0) * i / 6, v = v0 + (v1 - v0) * j / 6;
                if (!uv_in_face(*this, f, u, v)) continue;
                grow(s.eval(u, v));
            }
    }
    if (lo.x > hi.x) { lo = hi = {0, 0, 0}; }
}

bool Solid::is_closed(int* naked, int* nonmanifold) const {
    std::map<int, int> use;
    for (auto& ce : coedges) use[ce.edge]++;
    int nk = 0, nm = 0;
    for (auto& kv : use) {
        if (edges[kv.first].degenerate) continue; // pole edges: single-use is valid
        if (kv.second == 1) nk++;
        else if (kv.second > 2) nm++;
    }
    for (size_t e = 0; e < edges.size(); e++)
        if (!use.count((int)e) && !edges[e].degenerate) nk++;
    if (naked) *naked = nk;
    if (nonmanifold) *nonmanifold = nm;
    return nk == 0 && nm == 0;
}

double Solid::loop_uv_area(const Face& f, const Loop& l) const {
    double a = 0;
    for (int cei : l.ces) {
        const CoEdge& ce = coedges[cei];
        for (size_t i = 0; i + 1 < ce.pc.size(); i++) {
            const UV2& p = ce.pc[i];
            const UV2& q = ce.pc[i + 1];
            a += p.u * q.v - q.u * p.v;
        }
        if (ce.pc.size() > 2) {
            const UV2& p = ce.pc.back();
            const UV2& q = ce.pc.front();
            // close only if the loop actually closes on this coedge
        }
    }
    (void)f;
    return 0.5 * a;
}

// ============================================================================
// from_brep
// ============================================================================

namespace {

int curve_samples(const Cur& c, double t0, double t1) {
    switch (c.k) {
    case Cur::LINE: return 2;
    case Cur::CIRCLE:
    case Cur::ELLIPSE: {
        double span = std::abs(t1 - t0);
        return std::max(4, std::min(128, (int)std::ceil(span / 0.1)));
    }
    case Cur::NURBS: return 48;
    case Cur::POLY: return (int)c.poly.size();
    }
    return 16;
}

// Build a coedge pcurve by sampling the 3D edge curve along traversal and
// inverting through the face surface (SameParameter-exact by construction).
void build_coedge_pc(Solid& s, CoEdge& ce, const Srf& srf) {
    const Edge& e = s.edges[ce.edge];
    if (e.c.k == Cur::POLY) {
        // use the poly samples directly (sub-range by t)
        const Cur& c = e.c;
        for (int i = 0; i < (int)c.poly.size(); i++) {
            double t = i;
            if (t < e.t0 - 1e-9 || t > e.t1 + 1e-9) continue;
            ce.pt.push_back(t);
        }
        if (ce.pt.empty() || ce.pt.front() > e.t0 + 1e-9) ce.pt.insert(ce.pt.begin(), e.t0);
        if (ce.pt.back() < e.t1 - 1e-9) ce.pt.push_back(e.t1);
    } else {
        int n = curve_samples(e.c, e.t0, e.t1);
        for (int i = 0; i < n; i++)
            ce.pt.push_back(e.t0 + (e.t1 - e.t0) * i / (n - 1.0));
    }
    if (!ce.fwd) std::reverse(ce.pt.begin(), ce.pt.end());
    double pu = 0, pv = 0;
    bool first = true;
    for (double t : ce.pt) {
        V3 p = e.c.eval(t);
        double u, v;
        if (!srf.uv_of(p, u, v)) {
            // projection failure (NURBS): keep raw best effort
            u = pu; v = pv;
        }
        if (!first) {
            if (srf.periodic_u()) u = unwrap_near(u, pu);
            if (srf.periodic_v()) v = unwrap_near(v, pv);
        }
        first = false;
        pu = u; pv = v;
        ce.pc.push_back({u, v});
    }
}

// Degenerate (pole) edges: map the STORED 2D trim pcurve through the source
// surface into the v3 chart (3D has no curve to sample).
void build_degenerate_pc(Solid& s, CoEdge& ce, const Srf& srf,
                         const NurbsCurve& c2d, const NurbsSurface& src_srf,
                         bool reversed) {
    auto dom = c2d.domain();
    int n = 16;
    std::vector<UV2> pts;
    for (int i = 0; i < n; i++) {
        double t = dom.first + (dom.second - dom.first) * i / (n - 1.0);
        Point uv = c2d.point_at(t);
        V3 p3 = to_v3(src_srf.point_at(uv[0], uv[1]));
        double u, v;
        if (srf.uv_of(p3, u, v)) pts.push_back({u, v});
    }
    if (reversed) std::reverse(pts.begin(), pts.end());
    ce.pc = std::move(pts);
    ce.pt.assign(ce.pc.size(), 0.0);
}

// Loop chart alignment on periodic surfaces, four phases:
//  A. snap junction samples at singular vertices (cone apex, sphere poles) to
//     the coedge's own generator direction (uv_of is arbitrary there);
//  B. connect: shift each non-degenerate coedge by whole periods so it starts
//     where the previous non-degenerate coedge ended;
//  C. seam pairs: an edge appearing twice with opposite directions must sit
//     exactly one period apart in its seam coordinate (the full-period-band
//     rule that makes sphere lunes and cone sides cover 2pi);
//  D. degenerate coedges: rebuild as straight UV spans between neighbors.
void align_loop_charts(Solid& s, Face& f) {
    const Srf& srf = s.srfs[f.srf];
    if (!srf.periodic_u() && !srf.periodic_v()) return;
    auto singular_dir = [&](const V3& p, double& us, double& vs, int& dir) {
        double u, v;
        if (!srf.uv_of(p, u, v)) return false;
        V3 sp, du, dv;
        srf.d0d1(u, v, sp, du, dv);
        double sc = (srf.r > 0 ? srf.r : 1.0);
        if (du.cross(dv).norm() >= 1e-9 * sc * sc) return false;
        us = u; vs = v;
        dir = du.norm() < dv.norm() ? 0 : 1; // span the degenerate direction
        return true;
    };
    for (auto& l : f.loops) {
        int m = (int)l.ces.size();
        if (m == 0) continue;
        // ---- A. singular junction snaps (each side handled independently) ----
        for (int k = 0; k < m; k++) {
            CoEdge& prev = s.coedges[l.ces[k]];
            CoEdge& next = s.coedges[l.ces[(k + 1) % m]];
            if (prev.pc.empty() || next.pc.empty()) continue;
            const Edge& pe = s.edges[prev.edge];
            V3 junc = s.verts[prev.fwd ? pe.v1 : pe.v0].p;
            double us, vs;
            int dir;
            if (!singular_dir(junc, us, vs, dir)) continue;
            if (!pe.degenerate && prev.pc.size() >= 2) {
                if (dir == 0) { prev.pc.back().u = prev.pc[prev.pc.size() - 2].u; prev.pc.back().v = vs; }
                else { prev.pc.back().v = prev.pc[prev.pc.size() - 2].v; prev.pc.back().u = us; }
            }
            const Edge& ne = s.edges[next.edge];
            if (!ne.degenerate && next.pc.size() >= 2) {
                if (dir == 0) { next.pc.front().u = next.pc[1].u; next.pc.front().v = vs; }
                else { next.pc.front().v = next.pc[1].v; next.pc.front().u = us; }
            }
        }
        // ---- B. connect walk (skip degenerates) ----
        int prev_nd = -1;
        for (int k = 0; k < m; k++) {
            CoEdge& cur = s.coedges[l.ces[k]];
            if (s.edges[cur.edge].degenerate || cur.pc.empty()) continue;
            if (prev_nd >= 0) {
                UV2 pe = s.coedges[l.ces[prev_nd]].pc.back(), cs = cur.pc.front();
                double du = 0, dv = 0;
                if (srf.periodic_u())
                    du = std::round((pe.u - cs.u) / TWO_PI) * TWO_PI;
                if (srf.periodic_v())
                    dv = std::round((pe.v - cs.v) / TWO_PI) * TWO_PI;
                if (du != 0 || dv != 0)
                    for (auto& p : cur.pc) { p.u += du; p.v += dv; }
            }
            prev_nd = k;
        }
        // ---- C. seam-pair period bands ----
        for (int k = 0; k < m; k++) {
            for (int k2 = k + 1; k2 < m; k2++) {
                CoEdge& a = s.coedges[l.ces[k]];
                CoEdge& b = s.coedges[l.ces[k2]];
                if (a.edge != b.edge || a.fwd == b.fwd) continue;
                if (s.edges[a.edge].degenerate) continue;
                if (a.pc.size() < 2 || b.pc.size() < 2) continue;
                // seam coordinate = the one with smaller span on `a`
                double sau = std::abs(a.pc.back().u - a.pc.front().u);
                double sav = std::abs(a.pc.back().v - a.pc.front().v);
                bool u_seam = sau <= sav;
                if (u_seam && !srf.periodic_u()) continue;
                if (!u_seam && !srf.periodic_v()) continue;
                double ma = 0, mb = 0;
                for (auto& p : a.pc) ma += (u_seam ? p.u : p.v);
                for (auto& p : b.pc) mb += (u_seam ? p.u : p.v);
                ma /= a.pc.size(); mb /= b.pc.size();
                double d = ma - mb;
                if (std::abs(std::abs(d) - TWO_PI) < 0.5) continue; // already a period apart
                // copies must be exactly one period apart: whole-period shift
                double sep = d >= 0 ? TWO_PI : -TWO_PI;
                double shift = std::round((sep - d) / TWO_PI) * TWO_PI;
                if (std::getenv("V3DBG3"))
                    std::fprintf(stderr, "[alignC] edge=%d useam=%d ma=%.3f mb=%.3f d=%.3f shift=%.3f\n",
                                 a.edge, (int)u_seam, ma, mb, d, shift);
                for (auto& p : b.pc) { if (u_seam) p.u += shift; else p.v += shift; }
            }
        }
        // ---- D. degenerate rebuild ----
        for (int k = 0; k < m; k++) {
            CoEdge& cur = s.coedges[l.ces[k]];
            const Edge& ce_e = s.edges[cur.edge];
            if (!ce_e.degenerate || cur.pc.empty()) continue;
            V3 jp = s.verts[cur.fwd ? ce_e.v1 : ce_e.v0].p;
            double us, vs;
            int dir = 0;
            if (!singular_dir(jp, us, vs, dir)) continue;
            UV2 start = s.coedges[l.ces[(k + m - 1) % m]].pc.back();
            UV2 target = s.coedges[l.ces[(k + 1) % m]].pc.front();
            if (std::getenv("V3DBG3"))
                std::fprintf(stderr, "[alignD] k=%d start=(%.3f,%.3f) target=(%.3f,%.3f) dir=%d\n",
                             k, start.u, start.v, target.u, target.v, dir);
            if (dir == 0) { start.v = vs; target.v = vs; }
            else { start.u = us; target.u = us; }
            int n = std::max(2, (int)cur.pc.size());
            cur.pc.resize(n);
            for (int i = 0; i < n; i++) {
                double fr = (double)i / (n - 1);
                cur.pc[i].u = start.u + (target.u - start.u) * fr;
                cur.pc[i].v = start.v + (target.v - start.v) * fr;
            }
        }
    }
}

} // namespace

Solid from_brep(const BRep& b) {
    Solid s;
    s.verts.reserve(b.m_vertices.size());
    for (auto& p : b.m_vertices) s.verts.push_back({to_v3(p), 1e-7});
    s.srfs.reserve(b.m_surfaces.size());
    for (auto& sf : b.m_surfaces) s.srfs.push_back(recognize(sf));

    // edges
    for (auto& te : b.m_topology_edges) {
        Edge e;
        e.v0 = te.start_vertex;
        e.v1 = te.end_vertex;
        if (te.curve_3d_index < 0 || e.v0 == e.v1) {
            if (te.curve_3d_index < 0) {
                e.degenerate = true;
                s.edges.push_back(e);
                continue;
            }
        }
        Cur c = recognize_curve(b.m_curves_3d[te.curve_3d_index]);
        V3 p0 = s.verts[e.v0].p, p1 = s.verts[e.v1].p;
        double t0 = 0, t1 = 0;
        c.project(p0, t0);
        c.project(p1, t1);
        if ((c.k == Cur::CIRCLE || c.k == Cur::ELLIPSE) && e.v0 == e.v1 &&
            !std::getenv("V3NOFLIP")) {
            // Align the recognized frame winding with the SOURCE curve's
            // direction. recognize_curve picks the conic plane normal from a
            // PCA eigenvector (arbitrary sign); a flipped frame reverses the
            // parametrization, which flips every pcurve slope built from this
            // edge (cylinder/cone side bands then span two periods and
            // uv_in_face breaks). Compare tangents at the edge start.
            const NurbsCurve& src = b.m_curves_3d[te.curve_3d_index];
            auto dom = src.domain();
            double best = 1e300;
            int bi = 0;
            for (int i = 0; i <= 16; i++) {
                double tt = dom.first + (dom.second - dom.first) * i / 16.0;
                double d = to_v3(src.point_at(tt)).dist(p0);
                if (d < best) { best = d; bi = i; }
            }
            double ta = dom.first + (dom.second - dom.first) *
                                       std::max(0, bi - 1) / 16.0;
            double tb = dom.first + (dom.second - dom.first) *
                                       std::min(16, bi + 1) / 16.0;
            V3 src_t = to_v3(src.point_at(tb)) - to_v3(src.point_at(ta));
            if (std::getenv("V3FLIPDBG"))
                std::fprintf(stderr,
                             "[flip] edge=%d kind=%d v0v1=%d dot=%.6f best=%.3g\n",
                             (int)s.edges.size(), (int)c.k, (int)(e.v0 == e.v1),
                             src_t.dot(c.d1(t0)), best);
            if (best < 1e-6 * (1.0 + c.r) && src_t.dot(c.d1(t0)) < 0) {
                // reverse the parametrization: negate y,z (keeps right-handed)
                c.f.y = c.f.y * -1.0;
                c.f.z = c.f.z * -1.0;
                c.project(p0, t0);
                c.project(p1, t1);
            }
        }
        if (e.v0 == e.v1) {
            // full-period edge: entire curve domain
            if (c.k == Cur::NURBS) {
                auto dom = c.nrb.domain();
                t0 = dom.first; t1 = dom.second;
            } else if (c.k == Cur::POLY) {
                t0 = 0; t1 = (double)c.poly.size();
            } else if (c.is_closed()) {
                t1 = t0 + c.period();
            } else {
                // open curve whose endpoints coincide geometrically: full sweep
                if (t1 <= t0) { /* leave projection range */ }
            }
        } else {
            if (c.is_closed() && c.period() > 0) {
                t1 = unwrap_near(t1, t0);
                if (t1 <= t0) t1 += TWO_PI;
            } else if (t1 < t0) {
                // keep ascending range on open curves
                std::swap(t0, t1);
            }
        }
        e.c = c;
        e.t0 = t0;
        e.t1 = t1;
        s.edges.push_back(e);
    }

    // faces / loops / coedges
    for (size_t fi = 0; fi < b.m_faces.size(); fi++) {
        Face f;
        f.srf = b.m_faces[fi].surface_index;
        f.rev = b.m_faces[fi].reversed;
        f.src = (int)fi;
        const Srf& srf = s.srfs[f.srf];
        for (int li : b.m_faces[fi].loop_indices) {
            Loop l;
            l.outer = (b.m_loops[li].type == session_cpp::BRepLoopType::Outer);
            for (int ti : b.m_loops[li].trim_indices) {
                const auto& tr = b.m_trims[ti];
                if (std::getenv("V3FBDBG"))
                    std::fprintf(stderr,
                                 "[fb] f=%zu ti=%d ty=%d e=%d c2d=%d c3d?\n", fi,
                                 ti, (int)tr.type, tr.edge_index,
                                 tr.curve_2d_index);
                // Singular-typed trims mark degenerate (pole) edges even when a
                // placeholder 3D curve exists — authoritative typing.
                if (tr.type == session_cpp::BRepTrimType::Singular &&
                    tr.edge_index >= 0)
                    s.edges[tr.edge_index].degenerate = true;
                int ei = tr.edge_index;
                if (ei < 0) {
                    // singular trim without a backing edge (e.g. an imported
                    // cone apex): synthesize a degenerate point edge — a -1
                    // coedge edge index crashes downstream consumers.
                    V3 pp{0, 0, 0};
                    if (tr.curve_2d_index >= 0) {
                        Point uv =
                            b.m_curves_2d[tr.curve_2d_index].point_at_middle();
                        pp = to_v3(b.m_surfaces[f.srf].point_at(uv[0], uv[1]));
                    }
                    Edge de;
                    de.degenerate = true;
                    de.v0 = de.v1 = (int)s.verts.size();
                    s.verts.push_back({pp, 1e-7});
                    ei = (int)s.edges.size();
                    s.edges.push_back(de);
                }
                CoEdge ce;
                ce.edge = ei;
                ce.fwd = !tr.reversed;
                ce.face = (int)fi;
                if (s.edges[ce.edge].degenerate && tr.curve_2d_index >= 0)
                    build_degenerate_pc(s, ce, srf, b.m_curves_2d[tr.curve_2d_index],
                                        b.m_surfaces[f.srf], tr.reversed);
                else
                    build_coedge_pc(s, ce, srf);
                l.ces.push_back((int)s.coedges.size());
                s.coedges.push_back(std::move(ce));
            }
            f.loops.push_back(std::move(l));
        }
        s.faces.push_back(std::move(f));
    }

    // singular junction snaps + whole-period chart alignment per loop
    if (std::getenv("V3RAW")) {
        for (size_t f = 0; f < s.faces.size(); f++)
            for (auto& l : s.faces[f].loops)
                for (int cei : l.ces) {
                    auto& ce = s.coedges[cei];
                    std::fprintf(stderr, "[raw] ce%d e=%d fwd=%d:", cei, ce.edge,
                                 (int)ce.fwd);
                    for (auto& p : ce.pc)
                        std::fprintf(stderr, " (%.3f,%.3f)", p.u, p.v);
                    std::fprintf(stderr, "\n");
                }
    }
    // Junction snap: imported trimmed solids have no SameParameter guarantee
    // (chair STEP files carry no pcurves; consecutive edge endpoints disagree
    // by up to ~5e-4 in 3D/UV). Tie every loop junction to ONE chart point —
    // the projection of the shared vertex — so the UV arrangement's corner
    // merge (uvtol ~1e-6 scale) can chain the loop at all. The scatter is
    // enclosed in the vertex tolerance (OCCT vertex-tolerance model).
    for (auto& f : s.faces) {
        const Srf& srf = s.srfs[f.srf];
        for (auto& l : f.loops)
            for (size_t k = 0; k < l.ces.size(); k++) {
                CoEdge& c0 = s.coedges[l.ces[k]];
                CoEdge& c1 = s.coedges[l.ces[(k + 1) % l.ces.size()]];
                if (c0.pc.empty() || c1.pc.empty()) continue;
                const Edge& e0 = s.edges[c0.edge];
                const Edge& e1 = s.edges[c1.edge];
                // pole junctions: align_loop_charts phase A owns the snap
                if (e0.degenerate || e1.degenerate) continue;
                int v0 = c0.fwd ? e0.v1 : e0.v0;
                int v1 = c1.fwd ? e1.v0 : e1.v1;
                UV2 j;
                bool have = false;
                if (v0 == v1 && v0 >= 0) {
                    double uu, vv;
                    if (srf.uv_of(s.verts[v0].p, uu, vv)) {
                        j = {uu, vv};
                        have = true;
                    }
                    V3 q0 = e0.c.eval(c0.fwd ? e0.t1 : e0.t0);
                    V3 q1 = e1.c.eval(c1.fwd ? e1.t0 : e1.t1);
                    double g3 = std::max(q0.dist(s.verts[v0].p),
                                         q1.dist(s.verts[v0].p));
                    if (g3 > s.verts[v0].tol) s.verts[v0].tol = g3;
                }
                if (!have) continue;
                // Snap each side towards the projection UNWRAPPED NEAR ITS OWN
                // endpoint: the move is then just the residual scatter (exact
                // imports ~1e-12, chairs ~5e-4), internal chain continuity is
                // preserved, and period-apart junctions (seam corners) stay a
                // whole period apart for align_loop_charts phase B to connect
                // exactly. Oversized moves mean a bad projection — leave the
                // junction untouched.
                UV2 j0 = j, j1 = j;
                if (srf.periodic_u()) {
                    j0.u = unwrap_near(j.u, c0.pc.back().u);
                    j1.u = unwrap_near(j.u, c1.pc.front().u);
                }
                if (srf.periodic_v()) {
                    j0.v = unwrap_near(j.v, c0.pc.back().v);
                    j1.v = unwrap_near(j.v, c1.pc.front().v);
                }
                double d0u = j0.u - c0.pc.back().u, d0v = j0.v - c0.pc.back().v;
                double d1u = j1.u - c1.pc.front().u, d1v = j1.v - c1.pc.front().v;
                if (d0u * d0u + d0v * d0v < 1e-4) c0.pc.back() = j0;
                if (d1u * d1u + d1v * d1v < 1e-4) c1.pc.front() = j1;
            }
    }
    for (auto& f : s.faces) align_loop_charts(s, f);
    return s;
}

// ============================================================================
// region queries + tessellation
// ============================================================================

namespace {

// signed area of one closed UV poly-chain
double chain_area(const std::vector<UV2>& pts) {
    if (pts.size() < 3) return 0;
    double a = 0;
    for (size_t i = 0; i < pts.size(); i++) {
        const UV2& p = pts[i];
        const UV2& q = pts[(i + 1) % pts.size()];
        a += p.u * q.v - q.u * p.v;
    }
    return 0.5 * a;
}

// distance from point to segment in UV
double seg_dist(const UV2& p, const UV2& a, const UV2& b, double* sign_cross) {
    double ux = b.u - a.u, uy = b.v - a.v;
    double L2 = ux * ux + uy * uy;
    double t = L2 > 1e-300 ? ((p.u - a.u) * ux + (p.v - a.v) * uy) / L2 : 0;
    t = std::max(0.0, std::min(1.0, t));
    double dx = p.u - (a.u + ux * t), dy = p.v - (a.v + uy * t);
    if (sign_cross) *sign_cross = ux * (p.v - a.v) - uy * (p.u - a.u);
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace

bool uv_in_face(const Solid& s, const Face& f, double u, double v,
                double* signed_dist) {
    // period-aware: wrap the query into the face's band on periodic surfaces.
    // The face region is intrinsically periodic; the loops live in one band.
    const Srf& srf = s.srfs[f.srf];
    if (srf.periodic_u() || srf.periodic_v()) {
        double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
        for (const Loop& l : f.loops)
            for (int cei : l.ces)
                for (auto& q : s.coedges[cei].pc) {
                    u0 = std::min(u0, q.u); u1 = std::max(u1, q.u);
                    v0 = std::min(v0, q.v); v1 = std::max(v1, q.v);
                }
        if (u0 <= u1) {
            // tolerance: a full-period band is 2*pi +/- 1 ULP depending on
            // anchoring; excluding it here silently disables period-awareness.
            // (Kept at ULP scale: larger tolerances flip verdicts for seam-
            // pair bands that legitimately span one period plus slop.)
            if (srf.periodic_u() && u1 - u0 <= TWO_PI + 1e-12)
                u = u0 + wrap_2pi(u - u0);
            if (srf.periodic_v() && v1 - v0 <= TWO_PI + 1e-12)
                v = v0 + wrap_2pi(v - v0);
        }
    }
    UV2 q{u, v};
    bool inside = false;
    double best = 1e300;
    // collect loops as closed chains
    for (const Loop& l : f.loops) {
        std::vector<UV2> chain;
        for (int cei : l.ces) {
            const CoEdge& ce = s.coedges[cei];
            for (size_t i = 0; i < ce.pc.size(); i++) {
                if (!chain.empty() && i == 0 &&
                    std::abs(ce.pc[i].u - chain.back().u) < 1e-12 &&
                    std::abs(ce.pc[i].v - chain.back().v) < 1e-12)
                    continue;
                chain.push_back(ce.pc[i]);
            }
        }
        if (chain.size() < 3) continue;
        // ray +u crossings
        int crossings = 0;
        for (size_t i = 0; i < chain.size(); i++) {
            const UV2& a = chain[i];
            const UV2& b = chain[(i + 1) % chain.size()];
            if ((a.v > q.v) != (b.v > q.v)) {
                double uc = a.u + (q.v - a.v) * (b.u - a.u) / (b.v - a.v);
                if (uc > q.u) crossings++;
            }
            double sc;
            best = std::min(best, seg_dist(q, a, b, &sc));
        }
        bool lin = (crossings & 1) != 0;
        if (l.outer) { if (lin) inside = true; }
        else { if (lin) inside = false; }
    }
    if (signed_dist) *signed_dist = inside ? best : -best;
    // boundary-inclusive: a point ON the boundary chain is part of the face
    // (even-odd is ambiguous there; faces are closed sets for our purposes)
    if (!inside && best < 1e-9) inside = true;
    return inside;
}

void tessellate_face(const Solid& s, const Face& f, int grid, std::vector<V3>& out) {
    // UV bbox of all loops
    double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
    for (const Loop& l : f.loops)
        for (int cei : l.ces)
            for (auto& p : s.coedges[cei].pc) {
                u0 = std::min(u0, p.u); u1 = std::max(u1, p.u);
                v0 = std::min(v0, p.v); v1 = std::max(v1, p.v);
            }
    if (u0 > u1) return;
    const Srf& srf = s.srfs[f.srf];
    double du = (u1 - u0) / grid, dv = (v1 - v0) / grid;
    auto inside = [&](double u, double v) { return uv_in_face(s, f, u, v); };
    for (int i = 0; i < grid; i++)
        for (int j = 0; j < grid; j++) {
            double ua = u0 + i * du, ub = ua + du;
            double va = v0 + j * dv, vb = va + dv;
            double uc = 0.5 * (ua + ub), vc = 0.5 * (va + vb);
            if (!inside(uc, vc)) continue;
            V3 p00 = srf.eval(ua, va), p10 = srf.eval(ub, va);
            V3 p01 = srf.eval(ua, vb), p11 = srf.eval(ub, vb);
            out.push_back(p00); out.push_back(p10); out.push_back(p11);
            out.push_back(p00); out.push_back(p11); out.push_back(p01);
        }
}

double Solid::signed_volume(int grid) const {
    double vol = 0;
    std::vector<V3> tris;
    for (auto& f : faces) {
        tris.clear();
        tessellate_face(*this, f, grid, tris);
        double v = 0;
        for (size_t i = 0; i + 2 < tris.size(); i += 3) {
            const V3& a = tris[i];
            const V3& b = tris[i + 1];
            const V3& c = tris[i + 2];
            v += a.dot(b.cross(c)) / 6.0;
        }
        vol += f.rev ? -v : v;
    }
    return vol;
}

void shift_loop_uv(Solid& s, Face& f, Loop& l, double du, double dv) {
    (void)f;
    for (int cei : l.ces)
        for (auto& p : s.coedges[cei].pc) { p.u += du; p.v += dv; }
}

// ============================================================================
// to_brep
// ============================================================================

namespace {

struct EmittedSrf {
    NurbsSurface nrb;
    double nsign = 1.0; // +1: chart normal_at == Srf::normal
};

// build the output chart for an analytic surface covering [u0,u1]x[v0,v1].
// For u-periodic surfaces u0 is the domain anchor: the frame is pre-rotated by
// u0 around the axis so chart param u evaluates the v3 point at angle u
// EXACTLY (no pcurve remapping, nothing re-fitted).
EmittedSrf emit_chart(const Srf& s, double u0, double u1, double v0, double v1) {
    EmittedSrf e;
    Frame fr = s.f;
    if (s.periodic_u() && s.k != Srf::NURBS) {
        // rotate x,y by u0 around z (chart domain starts at u0)
        V3 xp = s.f.x * std::cos(u0) + s.f.y * std::sin(u0);
        fr.x = xp;
        fr.y = s.f.z.cross(xp);
    }
    Xform xf = Xform::frame_to_world(
        to_point(fr.o), Vector(fr.x.x, fr.x.y, fr.x.z),
        Vector(fr.y.x, fr.y.y, fr.y.z), Vector(fr.z.x, fr.z.y, fr.z.z));
    switch (s.k) {
    case Srf::PLANE: {
        NurbsSurface n;
        n.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        n.set_cv(0, 0, Point(u0, v0, 0)); n.set_cv(1, 0, Point(u1, v0, 0));
        n.set_cv(0, 1, Point(u0, v1, 0)); n.set_cv(1, 1, Point(u1, v1, 0));
        n.set_domain(0, u0, u1);
        n.set_domain(1, v0, v1);
        n.transform(xf);
        e.nrb = n;
        break;
    }
    case Srf::CYLINDER: {
        NurbsSurface n = Primitives::cylinder_surface(0, 0, v0, s.r, v1 - v0);
        n.set_domain(0, u0, u0 + TWO_PI);
        n.set_domain(1, v0, v1);
        n.transform(xf);
        e.nrb = n;
        break;
    }
    case Srf::CONE: {
        // v3: apex at origin, v = slant from apex. Build the local chart with
        // the APEX at v=0 and base at v=1 so the emitted domain ASCENDS
        // (reversed domains break downstream domain() consumers).
        double sa = std::sin(s.r2), ca = std::cos(s.r2);
        double h = v1 * ca;         // height of used band top above apex
        double rb = v1 * sa;        // radius at top of band
        const double wq = std::sqrt(2.0) / 2.0;
        double cw[] = {1, wq, 1, wq, 1, wq, 1, wq, 1};
        double cx_[] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
        double cy_[] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
        double uknots[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
        NurbsSurface n(3, true, 3, 2, 9, 2);
        for (int i = 0; i < 10; i++) n.set_nurbsknot(0, i, uknots[i]);
        n.set_nurbsknot(1, 0, 0.0); n.set_nurbsknot(1, 1, 1.0);
        for (int i = 0; i < 9; i++) {
            double wi = cw[i];
            n.set_cv_4d(i, 0, 0, 0, 0, wi); // apex row (v=0)
            n.set_cv_4d(i, 1, rb * cx_[i] * wi, rb * cy_[i] * wi, h * wi, wi);
        }
        // local v: 0=apex, 1=base ; v3 v = local_fraction * v1
        n.set_domain(0, u0, u0 + TWO_PI);
        n.set_domain(1, 0.0, v1);
        n.transform(xf);
        e.nrb = n;
        break;
    }
    case Srf::SPHERE: {
        NurbsSurface n = Primitives::sphere_surface(0, 0, 0, s.r);
        n.set_domain(0, u0, u0 + TWO_PI);
        n.set_domain(1, -PI / 2, PI / 2);
        n.transform(xf);
        e.nrb = n;
        break;
    }
    case Srf::TORUS: {
        NurbsSurface n = Primitives::torus_surface(0, 0, 0, s.r, s.r2);
        n.set_domain(0, u0, u0 + TWO_PI);
        n.set_domain(1, 0, TWO_PI);
        n.transform(xf);
        e.nrb = n;
        break;
    }
    case Srf::NURBS:
        e.nrb = s.nrb;
        break;
    }
    if (s.k != Srf::NURBS) {
        // measure chart normal sign vs Srf::normal at one interior sample
        double um = 0.5 * (u0 + u1), vm = 0.5 * (v0 + v1);
        if (s.k == Srf::SPHERE) vm = std::max(-PI / 2 + 0.1, std::min(PI / 2 - 0.1, vm));
        V3 n_out = to_v3(e.nrb.normal_at(um, vm));
        V3 n_v3 = s.normal(um, vm);
        e.nsign = n_out.dot(n_v3) >= 0 ? 1.0 : -1.0;
    }
    return e;
}

NurbsCurve emit_curve_3d(const Edge& e) {
    if (e.c.k == Cur::LINE) {
        return NurbsCurve::create(false, 1, {to_point(e.c.eval(e.t0)),
                                             to_point(e.c.eval(e.t1))});
    }
    if (e.c.k == Cur::CIRCLE && e.t1 - e.t0 >= TWO_PI - 1e-9) {
        NurbsCurve n = Primitives::circle(0, 0, 0, e.c.r);
        Xform xf = Xform::frame_to_world(
            to_point(e.c.f.o), Vector(e.c.f.x.x, e.c.f.x.y, e.c.f.x.z),
            Vector(e.c.f.y.x, e.c.f.y.y, e.c.f.y.z),
            Vector(e.c.f.z.x, e.c.f.z.y, e.c.f.z.z));
        n.transform(xf);
        return n;
    }
    if (e.c.k == Cur::CIRCLE) {
        // partial arc: exact rational quadratic segments (<= 120 deg each),
        // C0-joined -- never a polyline (exact-curve requirement; the v1
        // massprops Green path also needs true 3D curves for sign work)
        double span = e.t1 - e.t0;
        int nseg = std::max(1, (int)std::ceil(span / (TWO_PI / 3)));
        std::vector<NurbsCurve> segs;
        for (int i = 0; i < nseg; i++) {
            double ta = e.t0 + span * i / nseg;
            double tb = e.t0 + span * (i + 1) / nseg;
            double tm = 0.5 * (ta + tb);
            segs.push_back(Primitives::arc(to_point(e.c.eval(ta)),
                                           to_point(e.c.eval(tm)),
                                           to_point(e.c.eval(tb))));
        }
        std::vector<NurbsCurve> joined = NurbsCurve::join(segs);
        if (joined.size() == 1) return joined[0];
        // fall through to polyline if the join refused
    }
    if (e.c.k == Cur::ELLIPSE && e.t1 - e.t0 >= TWO_PI - 1e-9) {
        NurbsCurve n = Primitives::ellipse(0, 0, 0, e.c.r, e.c.ry);
        Xform xf = Xform::frame_to_world(
            to_point(e.c.f.o), Vector(e.c.f.x.x, e.c.f.x.y, e.c.f.x.z),
            Vector(e.c.f.y.x, e.c.f.y.y, e.c.f.y.z),
            Vector(e.c.f.z.x, e.c.f.z.y, e.c.f.z.z));
        n.transform(xf);
        return n;
    }
    if (e.c.k == Cur::ELLIPSE) {
        // partial arc: exact rational quadratic segments (<= 120 deg each).
        // The shoulder is the tangent intersection recovered from the
        // parametric mid point with w = cos(dspan/2) -- the circular arc
        // formula, exact for the ellipse by affine invariance.
        double span = e.t1 - e.t0;
        int nseg = std::max(1, (int)std::ceil(span / (TWO_PI / 3)));
        std::vector<NurbsCurve> segs;
        for (int i = 0; i < nseg; i++) {
            double ta = e.t0 + span * i / nseg;
            double tb = e.t0 + span * (i + 1) / nseg;
            double tm = 0.5 * (ta + tb);
            double w = std::cos(0.5 * (tb - ta));
            V3 p0 = e.c.eval(ta), pm = e.c.eval(tm), p1 = e.c.eval(tb);
            V3 ch = (p0 + p1) * 0.5;
            V3 sh = ch + (pm - ch) / w;
            NurbsCurve seg(3, true, 3, 3);
            seg.set_cv_4d(0, p0.x, p0.y, p0.z, 1.0);
            seg.set_cv_4d(1, sh.x * w, sh.y * w, sh.z * w, w);
            seg.set_cv_4d(2, p1.x, p1.y, p1.z, 1.0);
            seg.set_nurbsknot(0, 0);
            seg.set_nurbsknot(1, 0);
            seg.set_nurbsknot(2, 1);
            seg.set_nurbsknot(3, 1);
            segs.push_back(seg);
        }
        std::vector<NurbsCurve> joined = NurbsCurve::join(segs);
        if (joined.size() == 1) return joined[0];
        // fall through to polyline if the join refused
    }
    if (e.c.k == Cur::NURBS) {
        // passthrough: keep the curve, trim to the edge range (no polyline
        // re-sampling of fitted/carried NURBS)
        NurbsCurve n = e.c.nrb;
        auto dom = n.domain();
        double t0 = std::max(e.t0, dom.first), t1 = std::min(e.t1, dom.second);
        if (t1 > t0 && n.trim(t0, t1)) return n;
        // fall through to polyline if the trim refused
    }
    // everything else: dense degree-1 polyline (measured deflection in Edge.tol)
    int n;
    if (e.c.k == Cur::POLY) {
        std::vector<Point> pts;
        int i0 = (int)std::ceil(e.t0), i1 = (int)std::floor(e.t1);
        pts.push_back(to_point(e.c.eval(e.t0)));
        for (int i = i0; i <= i1 && i < (int)e.c.poly.size(); i++)
            pts.push_back(to_point(e.c.poly[i]));
        pts.push_back(to_point(e.c.eval(e.t1)));
        if (e.c.poly_closed && e.t1 - e.t0 >= e.c.poly.size() - 1e-9)
            pts.push_back(pts.front());
        return NurbsCurve::create(false, 1, pts);
    }
    double span = e.t1 - e.t0;
    n = std::max(8, std::min(256, (int)std::ceil(span / 0.02)));
    std::vector<Point> pts;
    for (int i = 0; i <= n; i++)
        pts.push_back(to_point(e.c.eval(e.t0 + span * i / n)));
    return NurbsCurve::create(false, 1, pts);
}

} // namespace

BRep to_brep(const Solid& s) {
    BRep b;
    b.name = "v3_brep";

    // used UV rect per surface (UNWRAPPED pcurve values — charts are anchored
    // to them so emitted pcurves always land inside the chart domain)
    std::vector<double> u0(s.srfs.size(), 1e300), u1(s.srfs.size(), -1e300),
        v0(s.srfs.size(), 1e300), v1(s.srfs.size(), -1e300);
    for (const Face& f : s.faces)
        for (const Loop& l : f.loops)
            for (int cei : l.ces) {
                const CoEdge& ce = s.coedges[cei];
                for (auto& p : ce.pc) {
                    u0[f.srf] = std::min(u0[f.srf], p.u);
                    u1[f.srf] = std::max(u1[f.srf], p.u);
                    v0[f.srf] = std::min(v0[f.srf], p.v);
                    v1[f.srf] = std::max(v1[f.srf], p.v);
                }
            }

    std::vector<int> srf_map(s.srfs.size(), -1);
    std::vector<double> srf_nsign(s.srfs.size(), 1.0);
    // Canonicalize periodic-u charts into [-pi, pi): the used range can sit
    // anywhere in the unwrapped chart (split placement), but downstream
    // integrators (brep_massprops' Green form for spheres) assume a
    // canonical window -- hole loops above u=2pi integrate on the wrong
    // branch. A whole-period shift of the anchor AND all pcurves is
    // geometry-preserving.
    std::vector<double> srf_ushift(s.srfs.size(), 0.0);
    for (size_t i = 0; i < s.srfs.size(); i++)
        if (s.srfs[i].periodic_u() && u0[i] <= u1[i])
            srf_ushift[i] = TWO_PI * std::floor((u0[i] + PI) / TWO_PI);
    for (size_t i = 0; i < s.srfs.size(); i++) {
        u0[i] -= srf_ushift[i];
        u1[i] -= srf_ushift[i];
    }
    for (size_t i = 0; i < s.srfs.size(); i++) {
        const Srf& S = s.srfs[i];
        if (u0[i] > u1[i]) continue; // unused
        double du = u1[i] - u0[i], dv = v1[i] - v0[i];
        if (!S.periodic_u()) { u0[i] -= 0.05 * du + 1e-6; u1[i] += 0.05 * du + 1e-6; }
        else { u1[i] = u0[i] + TWO_PI; } // anchor = used min; frame pre-rotated
        if (S.k == Srf::SPHERE) { v0[i] = -PI / 2; v1[i] = PI / 2; }
        else if (S.k == Srf::TORUS) { v0[i] = 0; v1[i] = TWO_PI; }
        else if (S.k == Srf::CONE) { v0[i] = std::max(1e-6, v0[i] - 0.05 * dv); v1[i] += 0.05 * dv + 1e-6; }
        else { v0[i] -= 0.05 * dv + 1e-6; v1[i] += 0.05 * dv + 1e-6; }
        if (S.k == Srf::NURBS) {
            EmittedSrf e;
            e.nrb = S.nrb;
            e.nsign = 1.0;
            srf_map[i] = b.add_surface(e.nrb);
            srf_nsign[i] = 1.0;
            continue;
        }
        EmittedSrf e = emit_chart(S, u0[i], u1[i], v0[i], v1[i]);
        srf_map[i] = b.add_surface(e.nrb);
        srf_nsign[i] = e.nsign;
    }

    // vertices
    std::vector<int> vmap(s.verts.size(), -1);
    for (size_t i = 0; i < s.verts.size(); i++)
        vmap[i] = b.add_vertex(to_point(s.verts[i].p));

    // topology vertices
    for (size_t i = 0; i < s.verts.size(); i++) {
        session_cpp::BRepVertex tv;
        tv.point_index = vmap[i];
        b.m_topology_vertices.push_back(tv);
    }

    // edges
    std::vector<int> emap(s.edges.size(), -1);
    for (size_t i = 0; i < s.edges.size(); i++) {
        const Edge& e = s.edges[i];
        if (e.degenerate) {
            // zero-length point curve: BRep::is_solid skips degenerate edges by
            // curve extent (a -1 curve index would force the 2-trim rule).
            V3 pv = s.verts[e.v0].p;
            int ci = b.add_curve_3d(NurbsCurve::create(
                false, 1, {to_point(pv), to_point(pv)}));
            emap[i] = b.add_edge(ci, vmap[e.v0], vmap[e.v1]);
            continue;
        }
        int ci = b.add_curve_3d(emit_curve_3d(e));
        emap[i] = b.add_edge(ci, vmap[e.v0], vmap[e.v1]);
    }

    // faces / loops / trims
    for (const Face& f : s.faces) {
        if (srf_map[f.srf] < 0) continue;
        bool rev = f.rev;
        if (srf_nsign[f.srf] < 0) rev = !rev;
        int fi = b.add_face(srf_map[f.srf], rev);
        for (const Loop& l : f.loops) {
            int li = b.add_loop(fi, l.outer ? session_cpp::BRepLoopType::Outer
                                            : session_cpp::BRepLoopType::Inner);
            for (int cei : l.ces) {
                const CoEdge& ce = s.coedges[cei];
                if (emap[ce.edge] < 0) continue;
                std::vector<Point> pts;
                for (auto& p : ce.pc)
                    pts.push_back(Point(p.u - srf_ushift[f.srf], p.v, 0));
                if (pts.size() < 2) continue;
                int c2d = b.add_curve_2d(NurbsCurve::create(false, 1, pts));
                b.add_trim(c2d, emap[ce.edge], li, !ce.fwd,
                           session_cpp::BRepTrimType::Mated);
            }
        }
    }
    return b;
}

} // namespace v3
