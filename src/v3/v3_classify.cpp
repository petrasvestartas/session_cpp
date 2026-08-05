#include "v3_classify.h"
#include "v3_ssi.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <queue>
#include <set>

namespace v3 {

V3 face_outward_normal(const Solid& s, const Face& f, double u, double v) {
    V3 n = s.srfs[f.srf].normal(u, v);
    return f.rev ? n * -1.0 : n;
}

namespace {

// shift a UV by whole periods to land inside [mid-π, mid+π] of the face band
void reanchor_periodic(const Srf& srf, double u0, double u1, double v0, double v1,
                       double& u, double& v) {
    if (srf.periodic_u())
        u += TWO_PI * std::round((0.5 * (u0 + u1) - u) / TWO_PI);
    if (srf.periodic_v())
        v += TWO_PI * std::round((0.5 * (v0 + v1) - v) / TWO_PI);
}

// Newton-minimize |S(u,v) - p|^2 from (u,v); returns converged UV.
bool newton_project(const Srf& srf, const V3& p, double& u, double& v) {
    double err_prev = 1e300;
    for (int it = 0; it < 30; it++) {
        V3 sp, du, dv;
        srf.d0d1(u, v, sp, du, dv);
        V3 r = sp - p;
        double err = r.norm();
        double a = du.norm2(), b = du.dot(dv), c = dv.norm2();
        double det = a * c - b * b;
        if (det < 1e-300) return false;
        double ru = r.dot(du), rv = r.dot(dv);
        double ddu = (-ru * c + rv * b) / det;
        double ddv = (-rv * a + ru * b) / det;
        // backtracking line search: never accept an increase of |S - p|
        double lam = 1.0;
        double u0 = u, v0 = v;
        bool accepted = false;
        for (int bt = 0; bt < 10; bt++) {
            double un = u0 + ddu * lam, vn = v0 + ddv * lam;
            double en = (srf.eval(un, vn) - p).norm();
            if (en <= err || en < 1e-14) {
                u = un; v = vn;
                accepted = true;
                if (en < 1e-14) return true;
                break;
            }
            lam *= 0.5;
        }
        if (!accepted) return true; // stuck at a (local) point no better than this
        if (err_prev - err < 1e-12 && err_prev < 1e300) {
            // converged when the error stops decreasing
            if (std::abs(ddu * lam) + std::abs(ddv * lam) < 1e-12) return true;
        }
        err_prev = err;
    }
    return true;
}

} // namespace

bool closest_on_face(const Solid& s, const Face& f, const V3& p,
                     double& u_out, double& v_out, double& dist2_out,
                     bool* on_bnd, int* edge_out) {
    if (std::getenv("V3COFDBG")) {
        auto bad = [&](const char* what, long long val, long long lim) {
            std::fprintf(stderr,
                         "[COF] CORRUPT %s=%lld lim=%lld face=%ld srf=%d loops=%zu "
                         "nedges=%zu ncoedges=%zu p=(%.4f,%.4f,%.4f)\n",
                         what, val, lim, (long)(&f - &s.faces[0]), f.srf,
                         f.loops.size(), s.edges.size(), s.coedges.size(), p.x,
                         p.y, p.z);
            std::abort();
        };
        if (f.srf < 0 || f.srf >= (int)s.srfs.size()) bad("srf", f.srf, (long long)s.srfs.size());
        for (const Loop& l : f.loops)
            for (int cei : l.ces) {
                if (cei < 0 || cei >= (int)s.coedges.size()) bad("cei", cei, (long long)s.coedges.size());
                const CoEdge& ce = s.coedges[cei];
                if (ce.edge < 0 || ce.edge >= (int)s.edges.size()) bad("ce.edge", ce.edge, (long long)s.edges.size());
                if (ce.face < 0 || ce.face >= (int)s.faces.size()) bad("ce.face", ce.face, (long long)s.faces.size());
                if (!s.edges[ce.edge].degenerate) {
                    if (s.edges[ce.edge].v0 < 0 || s.edges[ce.edge].v0 >= (int)s.verts.size())
                        bad("e.v0", s.edges[ce.edge].v0, (long long)s.verts.size());
                }
            }
    }
    const Srf& srf = s.srfs[f.srf];
    // UV bbox of loops
    double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
    for (const Loop& l : f.loops)
        for (int cei : l.ces)
            for (auto& q : s.coedges[cei].pc) {
                u0 = std::min(u0, q.u); u1 = std::max(u1, q.u);
                v0 = std::min(v0, q.v); v1 = std::max(v1, q.v);
            }
    if (u0 > u1) return false;
    // seed grid (small; Newton does the work)
    double best = 1e300, bu = 0, bv = 0;
    bool found = false;
    bool best_bnd = false; // winner came from the boundary path
    int best_edge = -1;
    const int G = 5;
    for (int i = 0; i <= G; i++)
        for (int j = 0; j <= G; j++) {
            double u = u0 + (u1 - u0) * i / G, v = v0 + (v1 - v0) * j / G;
            if (!uv_in_face(s, f, u, v)) continue;
            double uu = u, vv = v;
            newton_project(srf, p, uu, vv);
            // result must lie in the face: re-anchor by whole periods into the
            // face's band (unwrap_near cannot bridge gaps > pi)
            reanchor_periodic(srf, u0, u1, v0, v1, uu, vv);
            if (!uv_in_face(s, f, uu, vv)) continue;
            double d2 = (srf.eval(uu, vv) - p).norm2();
            if (d2 < best) { best = d2; bu = uu; bv = vv; found = true; }
        }
    // boundary: project onto each coedge's 3D curve (the min for outside points
    // is usually ON the boundary, which unconstrained Newton walks past)
    for (const Loop& l : f.loops)
        for (int cei : l.ces) {
            const CoEdge& ce = s.coedges[cei];
            const Edge& e = s.edges[ce.edge];
            if (e.degenerate) {
                double d2 = (s.verts[e.v0].p - p).norm2();
                if (d2 < best && !ce.pc.empty()) {
                    best = d2; bu = ce.pc[0].u; bv = ce.pc[0].v; found = true;
                    best_bnd = true; best_edge = ce.edge;
                }
                continue;
            }
            double t;
            if (!e.c.project(p, t)) continue;
            if (e.c.is_closed() && e.c.period() > 0)
                t = unwrap_near(t, 0.5 * (e.t0 + e.t1));
            t = std::max(e.t0, std::min(e.t1, t));
            double d2 = (e.c.eval(t) - p).norm2();
            if (d2 >= best) continue;
            // UV of the boundary point: lerp the coedge pcurve at t
            double uu = 0, vv = 0;
            bool got = false;
            if (ce.pt.size() >= 2 && ce.pt.size() == ce.pc.size()) {
                bool asc = ce.pt.back() > ce.pt.front();
                size_t i0 = 0, i1 = ce.pt.size() - 1;
                for (size_t i = 0; i + 1 < ce.pt.size(); i++) {
                    double ta = ce.pt[i], tb = ce.pt[i + 1];
                    if ((t >= ta && t <= tb) || (t >= tb && t <= ta)) { i0 = i; i1 = i + 1; break; }
                }
                double ta = ce.pt[i0], tb = ce.pt[i1];
                double fr = std::abs(tb - ta) > 1e-300 ? (t - ta) / (tb - ta) : 0;
                fr = std::max(0.0, std::min(1.0, fr));
                uu = ce.pc[i0].u + (ce.pc[i1].u - ce.pc[i0].u) * fr;
                vv = ce.pc[i0].v + (ce.pc[i1].v - ce.pc[i0].v) * fr;
                got = true;
                (void)asc;
            } else {
                double u2, v2;
                if (srf.uv_of(e.c.eval(t), u2, v2)) {
                    reanchor_periodic(srf, u0, u1, v0, v1, u2, v2);
                    uu = u2; vv = v2; got = true;
                }
            }
            if (got) {
                best = d2; bu = uu; bv = vv; found = true;
                best_bnd = true; best_edge = ce.edge;
            }
        }
    if (!found) return false;
    u_out = bu; v_out = bv; dist2_out = best;
    if (on_bnd) *on_bnd = best_bnd;
    if (edge_out) *edge_out = best_edge;
    return true;
}

ClosestInfo closest_on_solid(const Solid& s, const V3& p) {
    ClosestInfo ci;
    for (const Face& f : s.faces) {
        double u, v, d2;
        bool ob = false;
        int eo = -1;
        if (closest_on_face(s, f, p, u, v, d2, &ob, &eo) && d2 < ci.dist2) {
            ci.face = (int)(&f - &s.faces[0]);
            ci.u = u; ci.v = v; ci.edge = eo;
            ci.on_boundary = ob;
            ci.dist2 = d2;
        }
    }
    // edges (boundary-dominant cases)
    for (size_t e = 0; e < s.edges.size(); e++) {
        const Edge& ed = s.edges[e];
        if (ed.degenerate) {
            double d2 = (s.verts[ed.v0].p - p).norm2();
            if (d2 < ci.dist2) {
                ci.face = -1; ci.edge = (int)e; ci.et = 0; ci.dist2 = d2;
            }
            continue;
        }
        double t;
        if (!ed.c.project(p, t)) continue;
        if (ed.c.is_closed() && ed.c.period() > 0)
            t = unwrap_near(t, 0.5 * (ed.t0 + ed.t1));
        t = std::max(ed.t0, std::min(ed.t1, t));
        double d2 = (ed.c.eval(t) - p).norm2();
        if (d2 < ci.dist2) {
            ci.face = -1; ci.edge = (int)e; ci.et = t; ci.dist2 = d2;
        }
    }
    return ci;
}

namespace {

// ray parity: count crossings of ray p + d*t (t > 0) through all faces,
// deduplicated in t (a ray through a shared edge counts once)
int ray_crossings(const Solid& s, const V3& p, const V3& d) {
    std::vector<double> ts;
    Cur ray = cur_line(p, d);
    // ray extent: beyond the solid bbox
    V3 lo, hi;
    s.bbox(lo, hi);
    double reach = (hi - lo).norm() * 2 + 1.0;
    for (const Face& f : s.faces) {
        auto hits = csi(ray, 1e-9, reach, s.srfs[f.srf], 1e-9);
        // face band for re-anchoring
        double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
        for (const Loop& l : f.loops)
            for (int cei : l.ces)
                for (auto& q : s.coedges[cei].pc) {
                    u0 = std::min(u0, q.u); u1 = std::max(u1, q.u);
                    v0 = std::min(v0, q.v); v1 = std::max(v1, q.v);
                }
        for (auto& h : hits) {
            double u = h.u, v = h.v;
            if (u0 <= u1) reanchor_periodic(s.srfs[f.srf], u0, u1, v0, v1, u, v);
            if (uv_in_face(s, f, u, v)) ts.push_back(h.t);
        }
    }
    std::sort(ts.begin(), ts.end());
    int crossings = 0;
    double last = -1e300;
    for (double t : ts) {
        if (t - last > 1e-6) crossings++;
        last = t;
    }
    return crossings;
}

// classify via the faces adjacent to the closest edge/vertex: p is OUT iff it
// is on the outward side of ANY adjacent face's local support (dihedral cone
// rule). Returns false when undecidable (no adjacent faces).
bool classify_by_adjacent(const Solid& s, const V3& p, const ClosestInfo& ci,
                          double tol, PtCls& out) {
    if (ci.edge < 0) return false;
    // sign from each adjacent face's OWN closest point to p (the edge/vertex's
    // local support planes at p's projection)
    double best = -1e300;
    bool any = false;
    for (const CoEdge& ce : s.coedges) {
        if (ce.edge != ci.edge) continue;
        // split_face appends PART coedges to the shared pool whose ce.face is a
        // part index (not a solid face index) -- skip those; the original
        // faces' coedges for the same edge carry the same surface geometry
        if (ce.face < 0 || ce.face >= (int)s.faces.size()) continue;
        const Face& f = s.faces[ce.face];
        double u, v, d2;
        if (!closest_on_face(s, f, p, u, v, d2)) continue;
        V3 q = s.srfs[f.srf].eval(u, v);
        V3 n = face_outward_normal(s, f, u, v);
        // at a surface singularity (cone apex, sphere pole) the normal is
        // unit garbage -- nudge the probe UV toward the face's interior and
        // use the normal there (the support fan of the apex is approximated
        // by the ring just inside it)
        {
            V3 sp, du, dv;
            s.srfs[f.srf].d0d1(u, v, sp, du, dv);
            V3 cr = du.cross(dv);
            if (cr.norm() < 1e-8) {
                double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
                for (const Loop& l : f.loops)
                    for (int cei : l.ces)
                        for (auto& w : s.coedges[cei].pc) {
                            u0 = std::min(u0, w.u); u1 = std::max(u1, w.u);
                            v0 = std::min(v0, w.v); v1 = std::max(v1, w.v);
                        }
                if (u0 <= u1) {
                    double un = u + (0.5 * (u0 + u1) - u) * 0.02;
                    double vn = v + (0.5 * (v0 + v1) - v) * 0.02;
                    n = face_outward_normal(s, f, un, vn);
                }
            }
        }
        best = std::max(best, (p - q).dot(n));
        any = true;
    }
    if (!any) return false;
    if (best > tol) { out = PtCls::OUT; return true; }
    if (best < -tol) { out = PtCls::IN; return true; }
    out = PtCls::ON;
    return true;
}

} // namespace

PtCls classify_point(const Solid& s, const V3& p, double tol) {
    ClosestInfo ci = closest_on_solid(s, p);
    double dist = std::sqrt(ci.dist2);
    if (dist < tol) return PtCls::ON;
    if (ci.face >= 0 && !ci.on_boundary) {
        // half-space test: valid only when the closest point is in the face
        // INTERIOR (at a boundary winner the local support plane says nothing
        // about off-face directions -- the cylinder base rim said "IN" for a
        // point radially outside the solid)
        const Face& f = s.faces[ci.face];
        V3 n = face_outward_normal(s, f, ci.u, ci.v);
        V3 q = s.srfs[f.srf].eval(ci.u, ci.v);
        return (p - q).dot(n) < 0 ? PtCls::IN : PtCls::OUT;
    }
    // closest feature is an edge/vertex (or a face boundary): dihedral rule,
    // then parity fallback
    PtCls out;
    if (classify_by_adjacent(s, p, ci, tol, out)) return out;
    V3 dir{1.0, 0.6180339887, 0.3819660112}; // irrational-ish to avoid alignment
    dir = dir.normalized();
    int cr = ray_crossings(s, p, dir);
    return (cr & 1) ? PtCls::IN : PtCls::OUT;
}

// ============================================================================
// orientation
// ============================================================================

// Each face independently: probe an interior point slightly along +/-normal and
// decide by RAY PARITY (sign-free, works with arbitrarily flipped input
// normals): the side whose probe is in free space (even crossings) is outward.
void orient_solid(Solid& s) {
    V3 lo, hi;
    s.bbox(lo, hi);
    double diag = (hi - lo).norm() + 1e-9;
    V3 dir{1.0, 0.6180339887, 0.3819660112};
    dir = dir.normalized();
    for (size_t fi = 0; fi < s.faces.size(); fi++) {
        Face& f = s.faces[fi];
        // interior point of the face region
        double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
        for (const Loop& l : f.loops)
            for (int cei : l.ces)
                for (auto& q : s.coedges[cei].pc) {
                    u0 = std::min(u0, q.u); u1 = std::max(u1, q.u);
                    v0 = std::min(v0, q.v); v1 = std::max(v1, q.v);
                }
        if (u0 > u1) continue;
        double uc = 0, vc = 0;
        bool ok = false;
        for (int i = 1; i < 8 && !ok; i++)
            for (int j = 1; j < 8 && !ok; j++) {
                double u = u0 + (u1 - u0) * i / 8.0, v = v0 + (v1 - v0) * j / 8.0;
                if (uv_in_face(s, f, u, v)) { uc = u; vc = v; ok = true; }
            }
        if (!ok) continue;
        V3 p = s.srfs[f.srf].eval(uc, vc);
        V3 n = s.srfs[f.srf].normal(uc, vc);
        if (n.norm() < 0.5) continue;
        // step must stay well clear of other faces: shrink until the probes
        // disagree (one IN one OUT) or accept when both agree
        for (double step : {diag * 1e-4, diag * 1e-5, diag * 1e-6}) {
            int cp = ray_crossings(s, p + n * step, dir);
            int cm = ray_crossings(s, p - n * step, dir);
            bool in_p = (cp & 1), in_m = (cm & 1);
            if (in_p && !in_m) { f.rev = true; break; }   // +n points into material
            if (!in_p && in_m) { f.rev = false; break; }  // +n is outward
        }
    }
}

} // namespace v3
