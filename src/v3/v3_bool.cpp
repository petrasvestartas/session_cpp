// v3 boolean driver.
#include "v3_bool.h"
#include "v3_classify.h"
#include "v3_ssi.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace v3 {

using session_cpp::NurbsCurve;
using session_cpp::Point;

namespace {

// ---------------------------------------------------------------------------
// vertex weld by 3D hash
struct Weld {
    double tol;
    std::vector<Vertex> verts;
    std::map<long long, std::vector<int>> map;
    static long long key(double x, double y, double z, double cell) {
        long long a = llround(x / cell), b = llround(y / cell), c = llround(z / cell);
        return (a & 0x1FFFFF) << 42 | (b & 0x1FFFFF) << 21 | (c & 0x1FFFFF);
    }
    int at(const V3& p) {
        double cell = tol * 8;
        long long k0 = key(p.x, p.y, p.z, cell);
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++) {
                    long long k = k0 + ((long long)dx << 42) + ((long long)dy << 21) + dz;
                    auto it = map.find(k);
                    if (it == map.end()) continue;
                    for (int i : it->second)
                        if (verts[i].p.dist(p) < tol) return i;
                }
        int id = (int)verts.size();
        verts.push_back({p, (float)tol});
        map[k0].push_back(id);
        return id;
    }
};

// per-face 3D bbox from tessellation + boundary samples
void face_bbox(const Solid& S, const Face& f, V3& lo, V3& hi) {
    lo = {1e300, 1e300, 1e300};
    hi = {-1e300, -1e300, -1e300};
    std::vector<V3> tris;
    tessellate_face(S, f, 6, tris);
    for (auto& p : tris) {
        lo.x = std::min(lo.x, p.x); hi.x = std::max(hi.x, p.x);
        lo.y = std::min(lo.y, p.y); hi.y = std::max(hi.y, p.y);
        lo.z = std::min(lo.z, p.z); hi.z = std::max(hi.z, p.z);
    }
    for (const Loop& l : f.loops)
        for (int cei : l.ces) {
            const CoEdge& ce = S.coedges[cei];
            const Edge& e = S.edges[ce.edge];
            if (e.degenerate) continue;
            for (int i = 0; i <= 8; i++) {
                V3 p = e.c.eval(e.t0 + (e.t1 - e.t0) * i / 8);
                lo.x = std::min(lo.x, p.x); hi.x = std::max(hi.x, p.x);
                lo.y = std::min(lo.y, p.y); hi.y = std::max(hi.y, p.y);
                lo.z = std::min(lo.z, p.z); hi.z = std::max(hi.z, p.z);
            }
        }
}
bool boxes_overlap(const V3& alo, const V3& ahi, const V3& blo, const V3& bhi,
                   double pad) {
    return alo.x - pad <= bhi.x && ahi.x + pad >= blo.x &&
           alo.y - pad <= bhi.y && ahi.y + pad >= blo.y &&
           alo.z - pad <= bhi.z && ahi.z + pad >= blo.z;
}

// reverse a face (flip orientation): reverse loop order, flip coedge fwd,
// reverse pcurves, flip rev. Used for CUT's tool parts.
void reverse_face(Solid& S, Face& f) {
    f.rev = !f.rev;
    for (auto& l : f.loops) {
        std::reverse(l.ces.begin(), l.ces.end());
        for (int cei : l.ces) {
            CoEdge& ce = S.coedges[cei];
            ce.fwd = !ce.fwd;
            std::reverse(ce.pc.begin(), ce.pc.end());
            std::reverse(ce.pt.begin(), ce.pt.end());
        }
    }
}

// append another solid's topology to R with index offsets
void append_solid(Solid& R, const Solid& X, bool reversed) {
    int voff = (int)R.verts.size(), eoff = (int)R.edges.size();
    int coff = (int)R.coedges.size(), soff = (int)R.srfs.size();
    int foff = (int)R.faces.size();
    for (auto& v : X.verts) R.verts.push_back(v);
    for (auto& e : X.edges) {
        Edge ne = e;
        ne.v0 += voff; ne.v1 += voff;
        R.edges.push_back(ne);
    }
    for (auto& ce : X.coedges) {
        CoEdge nce = ce;
        nce.edge += eoff;
        nce.face += foff;
        R.coedges.push_back(nce);
    }
    for (auto& s : X.srfs) R.srfs.push_back(s);
    for (auto& f : X.faces) {
        Face nf = f;
        nf.srf += soff;
        for (auto& l : nf.loops)
            for (int& ce : l.ces) ce += coff;
        R.faces.push_back(nf);
        if (reversed) reverse_face(R, R.faces.back());
    }
}

// section edge ids live in their own numeric space so they can never collide
// with operand boundary edge ids
constexpr int SEC_BASE = 1000000;

// first-order distance to an implicit surface: |F| / |gradF|. F itself is
// quadratic (quartic for the torus) and not comparable to a length tolerance.
double srf_residual(const Srf& s, const V3& p) {
    if (!s.has_implicit()) return 0.0;
    double g = s.gradF(p).norm();
    double f = std::abs(s.F(p));
    return g > 1e-300 ? f / g : f;
}

// Plug B: fit a marched section run with a real curve. Ladder:
//  (i) analytic recognition of the run 3D points (line/circle/ellipse,
//      residual-gated — recognize_curve_pts)
//  (ii) LSQ NURBS fit (create_fitted), growing ncv 8->64 until max dev passes
//  (iii) caller falls back to the dense polyline
// A candidate is accepted only when every run point lies within fit_tol of it
// AND its dense samples stay within fit_tol of both sectioned surfaces.
bool fit_section(const std::vector<V3>& poly, bool closed_run, const Srf& sa,
                 const Srf& sb, double tol, Cur& out_c, double& out_t0,
                 double& out_t1, bool& out_closed, double& out_dev) {
    double fit_tol = std::max(tol * 10, 1e-6);
    bool fdbg = std::getenv("V3FITDBG") != nullptr;
    auto verify = [&](const Cur& cand, double t0, double t1, double& dev) {
        dev = 0;
        for (auto& q : poly) {
            double t;
            if (!cand.project(q, t)) return false;
            dev = std::max(dev, cand.eval(t).dist(q));
            if (dev > fit_tol) {
                if (fdbg)
                    std::fprintf(stderr, "[fit] reject: pt dev %.3g\n", dev);
                return false;
            }
        }
        int ns = std::max(32, (int)poly.size() * 2);
        for (int i = 0; i <= ns; i++) {
            V3 p = cand.eval(t0 + (t1 - t0) * i / ns);
            dev = std::max(dev, srf_residual(sa, p));
            dev = std::max(dev, srf_residual(sb, p));
            if (dev > fit_tol) {
                if (fdbg)
                    std::fprintf(stderr, "[fit] reject: srf resid %.3g\n", dev);
                return false;
            }
        }
        return true;
    };
    // (i) analytic recognition (subsample the run for conditioning)
    {
        V3 samp[64];
        int ns = std::min<size_t>(64, poly.size());
        for (int i = 0; i < ns; i++) samp[i] = poly[i * (poly.size() - 1) / (ns - 1)];
        Cur rec;
        if (recognize_curve_pts(samp, ns, tol, rec)) {
            double t0 = 0, t1 = 0;
            bool rng_ok = true;
            if (rec.k == Cur::LINE) {
                rec.project(poly.front(), t0);
                rec.project(poly.back(), t1);
                rng_ok = t1 > t0 && !closed_run;
            } else { // CIRCLE / ELLIPSE: continuous angular range along the run
                double tm = 0;
                rng_ok = rec.project(poly.front(), t0) &&
                         rec.project(poly[poly.size() / 2], tm) &&
                         rec.project(poly.back(), t1);
                if (rng_ok) {
                    tm = unwrap_near(tm, t0);
                    t1 = unwrap_near(t1, tm);
                    if (closed_run) {
                        t1 = t0 + rec.period();
                    } else if (t1 <= t0) {
                        // run traverses against the frame orientation: flip it
                        // (negating y,z reverses the parametrization)
                        rec.f.y = rec.f.y * -1.0;
                        rec.f.z = rec.f.z * -1.0;
                        rec.project(poly.front(), t0);
                        rec.project(poly[poly.size() / 2], tm);
                        rec.project(poly.back(), t1);
                        tm = unwrap_near(tm, t0);
                        t1 = unwrap_near(t1, tm);
                        rng_ok = t1 > t0;
                    }
                }
            }
            double dev;
            if (rng_ok && verify(rec, t0, t1, dev)) {
                if (fdbg)
                    std::fprintf(stderr, "[fit] analytic kind=%d dev=%.3g\n",
                                 (int)rec.k, dev);
                out_c = rec;
                out_t0 = t0;
                out_t1 = t1;
                out_closed = closed_run;
                out_dev = dev;
                return true;
            }
        }
    }
    // (ii) LSQ NURBS fit, growing ncv until the deviation passes
    {
        std::vector<Point> P;
        P.reserve(poly.size());
        V3 last{1e300, 1e300, 1e300};
        for (auto& q : poly)
            if (q.dist(last) > 1e-12) {
                P.push_back(to_point(q));
                last = q;
            }
        if (closed_run && P.size() > 2 &&
            to_v3(P.front()).dist(to_v3(P.back())) < fit_tol)
            P.pop_back();
        for (int ncv = 8; ncv <= 128; ncv *= 2) {
            if ((int)P.size() <= ncv) break;
            NurbsCurve fit = NurbsCurve::create_fitted(P, ncv, 3, closed_run);
            if (!fit.is_valid()) continue;
            Cur fc = cur_nurbs(fit);
            auto dom = fit.domain();
            double dev;
            if (verify(fc, dom.first, dom.second, dev)) {
                if (fdbg)
                    std::fprintf(stderr, "[fit] nurbs ncv=%d dev=%.3g\n", ncv,
                                 dev);
                out_c = fc;
                out_t0 = dom.first;
                out_t1 = dom.second;
                out_closed = closed_run;
                out_dev = dev;
                return true;
            }
        }
    }
    if (fdbg)
        std::fprintf(stderr, "[fit] fallback poly (%zu pts, closed=%d)\n",
                     poly.size(), (int)closed_run);
    return false;
}

} // namespace

Solid boolean(const Solid& A_in, const Solid& B_in, BoolOp op, double tol,
              bool* intersected) {
    Solid A = A_in, B = B_in;
    orient_solid(A);
    orient_solid(B);
    bool dbg = std::getenv("V3BOOLDBG") != nullptr;
    if (intersected) *intersected = false;

    // operand bboxes
    V3 alo, ahi, blo, bhi;
    A.bbox(alo, ahi);
    B.bbox(blo, bhi);
    double diag = ((ahi - alo).norm() + (bhi - blo).norm()) / 2 + 1e-9;

    if (!boxes_overlap(alo, ahi, blo, bhi, tol * 10)) {
        // disjoint: no boundary interference
        if (op == BoolOp::CUT) return A;
        if (op == BoolOp::COMMON) return Solid{};
        Solid R = A; // FUSE: disjoint assembly
        append_solid(R, B, false);
        return R;
    }

    // ---- FF sections ---------------------------------------------------------
    std::vector<Edge> sec_edges;   // shared section edge pool
    std::vector<std::vector<SecPoint>> sec_pts; // per edge: the run's samples
    std::map<int, std::vector<SecPC>> pcs_A, pcs_B;
    std::map<int, std::vector<double>> paves_A, paves_B;
    bool any_section = false;
    bool sd_pair = false;
    Weld weld{tol * 10};

    // face bboxes for pruning
    std::vector<V3> fa_lo(A.faces.size()), fa_hi(A.faces.size());
    std::vector<V3> fb_lo(B.faces.size()), fb_hi(B.faces.size());
    for (size_t i = 0; i < A.faces.size(); i++) face_bbox(A, A.faces[i], fa_lo[i], fa_hi[i]);
    for (size_t i = 0; i < B.faces.size(); i++) face_bbox(B, B.faces[i], fb_lo[i], fb_hi[i]);

    // pass 1: compute raw sections per face pair (keep the SSIResults alive)
    struct PairSecs {
        int ai, bi;
        SSIResult r;
    };
    std::vector<PairSecs> pair_secs;
    for (size_t ai = 0; ai < A.faces.size(); ai++) {
        const Face& FA = A.faces[ai];
        for (size_t bi = 0; bi < B.faces.size(); bi++) {
            const Face& FB = B.faces[bi];
            // deflection-aware pad: tessellation bboxes underestimate curved
            // faces (tangent contacts would be pruned). Pad by a fraction of
            // the face diagonal -- generous padding is safe (SSI just returns
            // empty for truly disjoint pairs), too-tight padding loses
            // tangent sections.
            double pad = tol * 10 +
                         0.01 * ((fa_hi[ai] - fa_lo[ai]).norm() +
                                 (fb_hi[bi] - fb_lo[bi]).norm());
            if (!boxes_overlap(fa_lo[ai], fa_hi[ai], fb_lo[bi], fb_hi[bi], pad)) {
                if (dbg)
                    std::fprintf(stderr,
                                 "[bool] bbox-skip A%zu B%zu  A[%.5f..%.5f z] B[%.5f..%.5f z]\n",
                                 ai, bi, fa_lo[ai].z, fa_hi[ai].z, fb_lo[bi].z,
                                 fb_hi[bi].z);
                continue;
            }
            const Srf& sa = A.srfs[FA.srf];
            const Srf& sb = B.srfs[FB.srf];
            SSIResult r = ssi(sa, sb, tol);
            if (r.same_domain) {
                sd_pair = true;
                if (dbg)
                    std::fprintf(stderr, "[bool] same-domain faces A%zu B%zu\n", ai, bi);
                continue;
            }
            if (!r.curves.empty())
                pair_secs.push_back({(int)ai, (int)bi, std::move(r)});
        }
    }

    // pass 1b: re-sample LINE sections to the face bbox overlap (the exact-case
    // lines are unbounded; runs need interior samples)
    for (auto& ps : pair_secs) {
        V3 lo{std::max(fa_lo[ps.ai].x, fb_lo[ps.bi].x) - tol,
              std::max(fa_lo[ps.ai].y, fb_lo[ps.bi].y) - tol,
              std::max(fa_lo[ps.ai].z, fb_lo[ps.bi].z) - tol};
        V3 hi{std::min(fa_hi[ps.ai].x, fb_hi[ps.bi].x) + tol,
              std::min(fa_hi[ps.ai].y, fb_hi[ps.bi].y) + tol,
              std::min(fa_hi[ps.ai].z, fb_hi[ps.bi].z) + tol};
        for (auto& sc : ps.r.curves) {
            if (!sc.has_exact || sc.exact.k != Cur::LINE) continue;
            // slab-clip the line
            const Cur& L = sc.exact;
            double t0 = -1e300, t1 = 1e300;
            for (int ax = 0; ax < 3; ax++) {
                double d = L.f.x[ax], oo = L.f.o[ax];
                if (std::abs(d) < 1e-300) {
                    if (oo < lo[ax] || oo > hi[ax]) { t0 = 1; t1 = 0; break; }
                    continue;
                }
                double ta = (lo[ax] - oo) / d, tb = (hi[ax] - oo) / d;
                if (ta > tb) std::swap(ta, tb);
                t0 = std::max(t0, ta);
                t1 = std::min(t1, tb);
            }
            SSIResult clipped;
            if (t1 > t0) {
                const Srf& sa = A.srfs[A.faces[ps.ai].srf];
                const Srf& sb = B.srfs[B.faces[ps.bi].srf];
                sample_exact(sa, sb, L, t0, t1, false, clipped, tol);
            }
            if (clipped.curves.empty()) sc.pts.clear();
            else sc = std::move(clipped.curves[0]);
        }
    }

    // pass 3: clip runs and register section pieces
    // point-to-polyline distance (marched duplicates of the same physical
    // curve have independent sample phasing: point-to-point distances read
    // ~half the sample spacing and the dedupe gate would miss them)
    auto dist_to_sec = [&](const V3& p, const std::vector<SecPoint>& qs) {
        double best = 1e300;
        for (size_t i = 0; i < qs.size(); i++) {
            best = std::min(best, p.dist(qs[i].p));
            if (i + 1 < qs.size()) {
                V3 ab = qs[i + 1].p - qs[i].p;
                double L2 = ab.norm2();
                double fr = L2 > 1e-300
                                ? std::max(0.0, std::min(1.0, (p - qs[i].p).dot(ab) / L2))
                                : 0.0;
                best = std::min(best, (qs[i].p + ab * fr).dist(p));
            }
        }
        return best;
    };
    for (auto& ps : pair_secs) {
        const Face& FA = A.faces[ps.ai];
        const Face& FB = B.faces[ps.bi];
        std::vector<int> pair_registered; // sec ids already kept for this pair
        for (auto& sc : ps.r.curves) {
            std::vector<SecPoint> pts_out;
            auto runs = section_runs_refined(A, FA, B, FB, sc, pts_out, tol);
            if (dbg && runs.empty() && !sc.pts.empty()) {
                // diagnose culled curves: count samples inside each face
                int ina = 0, inb = 0;
                for (auto& q : sc.pts) {
                    if (uv_in_face(A, FA, q.u1, q.v1)) ina++;
                    if (uv_in_face(B, FB, q.u2, q.v2)) inb++;
                }
                double bu0 = 1e300, bu1 = -1e300, bv0 = 1e300, bv1 = -1e300;
                for (const Loop& l : FB.loops)
                    for (int cei : l.ces)
                        for (auto& q : B.coedges[cei].pc) {
                            bu0 = std::min(bu0, q.u); bu1 = std::max(bu1, q.u);
                            bv0 = std::min(bv0, q.v); bv1 = std::max(bv1, q.v);
                        }
                const SecPoint& q0 = sc.pts.front();
                V3 pe = B.srfs[FB.srf].eval(q0.u2, q0.v2);
                std::fprintf(stderr,
                             "[p3drop] A%d B%d pts=%zu closed=%d inA=%d inB=%d "
                             "uvA0=(%.4f,%.4f) uvB0=(%.4f,%.4f) bandB u[%.3f,%.3f] "
                             "v[%.3f,%.3f] evalerr=%.2e p=(%.3f,%.3f,%.3f)\n",
                             ps.ai, ps.bi, sc.pts.size(), (int)sc.closed, ina,
                             inb, q0.u1, q0.v1, q0.u2, q0.v2, bu0, bu1, bv0, bv1,
                             pe.dist(q0.p), q0.p.x, q0.p.y, q0.p.z);
            }
            for (auto& run : runs) {
                if (run.i1 - run.i0 < 1 && !sc.closed) continue;
                std::vector<SecPoint> pts(pts_out.begin() + run.i0,
                                          pts_out.begin() + run.i1 + 1);
                // dedupe coincident curves from the same pair (tangent
                // contacts show up as multiplicity-2 curves from the implicit
                // solve -- registering both would put coincident loops in the
                // face arrangements)
                bool dup = false;
                if (pts.size() > 1) {
                    int ns = std::min<size_t>(32, pts.size());
                    // marched duplicates of one physical curve drift apart by
                    // up to the walk's closure radius (~tol*1000) — the gate
                    // must cover that, not just the deflection tolerance
                    double dup_gate = std::max(tol * 100, tol * 2000);
                    for (int prev : pair_registered) {
                        bool all_near = true;
                        for (int s = 0; s < ns && all_near; s++) {
                            size_t idx = s * (pts.size() - 1) / (ns - 1);
                            if (dist_to_sec(pts[idx].p, sec_pts[prev]) > dup_gate)
                                all_near = false;
                        }
                        if (all_near) { dup = true; break; }
                    }
                }
                if (dup) continue;
                // tangent (osculating) contact: the surfaces touch along this
                // run without crossing (normals parallel at EVERY sample) --
                // it must not split either face (box_torus ridge circles).
                // Genuine crossings keep a dihedral angle; only true
                // tangencies read |dot| ~ 1 along the whole run.
                {
                    int ns = std::min<size_t>(8, pts.size());
                    bool tangent = ns > 0;
                    for (int s = 0; s < ns && tangent; s++) {
                        size_t idx = s * (pts.size() - 1) / (ns > 1 ? ns - 1 : 1);
                        V3 na = face_outward_normal(A, FA, pts[idx].u1, pts[idx].v1);
                        V3 nb = face_outward_normal(B, FB, pts[idx].u2, pts[idx].v2);
                        if (std::abs(na.dot(nb)) < 1.0 - 1e-6) tangent = false;
                    }
                    if (tangent) {
                        if (dbg)
                            std::fprintf(stderr, "[p3reg] skip tangent contact A%d B%d\n",
                                         ps.ai, ps.bi);
                        continue;
                    }
                }
                Edge se;
                bool closed_edge = false;
                bool full_cover = runs.size() == 1 && sc.closed &&
                                  pts.size() >= sc.pts.size() - 2;
                if (sc.has_exact) {
                    se.c = sc.exact;
                    se.t0 = pts.front().t;
                    se.t1 = pts.back().t;
                    if (full_cover && sc.exact.is_closed()) {
                        closed_edge = true;
                        se.t1 = se.t0 + sc.exact.period();
                    }
                } else {
                    std::vector<V3> poly;
                    for (auto& q : pts) poly.push_back(q.p);
                    if (poly.size() < 2) continue;
                    // marched run: fit a real curve (Plug B ladder) instead of
                    // emitting the raw polyline; polyline stays the fallback.
                    Cur fc;
                    double ft0, ft1, fdev;
                    bool fclosed;
                    if (fit_section(poly, full_cover, A.srfs[FA.srf],
                                    B.srfs[FB.srf], tol, fc, ft0, ft1, fclosed,
                                    fdev)) {
                        se.c = fc;
                        se.t0 = ft0;
                        se.t1 = ft1;
                        se.tol = std::max(fdev, tol);
                        closed_edge = fclosed;
                    } else {
                        se.c = cur_poly(poly, false);
                        se.t0 = 0;
                        se.t1 = (double)poly.size() - 1;
                        if (full_cover) {
                            se.c.poly_closed = true;
                            closed_edge = true;
                            se.t1 = (double)poly.size();
                        }
                    }
                }
                int sec_id = (int)sec_edges.size();
                se.v0 = weld.at(pts.front().p);
                se.v1 = closed_edge ? se.v0 : weld.at(pts.back().p);
                sec_edges.push_back(se);
                sec_pts.push_back(pts);
                pair_registered.push_back(sec_id);
                any_section = true;
                SecPC pa, pb;
                pa.sec_edge = pb.sec_edge = SEC_BASE + sec_id;
                for (auto& q : pts) {
                    pa.pc.push_back({q.u1, q.v1});
                    pb.pc.push_back({q.u2, q.v2});
                }
                pa.end_vtx[0] = pb.end_vtx[0] = se.v0;
                pa.end_vtx[1] = pb.end_vtx[1] = se.v1;
                pa.closed = pb.closed = closed_edge;
                pcs_A[ps.ai].push_back(pa);
                pcs_B[ps.bi].push_back(pb);
                if (dbg)
                    std::fprintf(stderr,
                                 "[p3reg] sec=%d firstB=(%.6f,%.4f) lastB=(%.6f,%.4f) firstA=(%.4f,%.4f)\n",
                                 sec_id, pb.pc.front().u, pb.pc.front().v,
                                 pb.pc.back().u, pb.pc.back().v, pa.pc.front().u,
                                 pa.pc.front().v);
            }
        }
        if (dbg && !pair_registered.empty()) {
            // per-pair registered section count + pairwise midpoint distances
            // (marched-duplicate diagnostics)
            std::fprintf(stderr, "[p3cnt] A%d B%d: registered=%zu mids:", ps.ai,
                         ps.bi, pair_registered.size());
            for (int id : pair_registered) {
                auto& q = sec_pts[id];
                V3 m = q[q.size() / 2].p;
                std::fprintf(stderr, " (%.3f,%.3f,%.3f)", m.x, m.y, m.z);
            }
            std::fprintf(stderr, "\n");
        }
    }
    if (intersected) *intersected = any_section;
    if (std::getenv("V3PREPDBG")) {
        auto dump = [&](const char* tag, std::map<int, std::vector<SecPC>>& pcs) {
            for (auto& kv : pcs)
                for (auto& sp : kv.second) {
                    double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
                    for (auto& q : sp.pc) {
                        u0 = std::min(u0, q.u); u1 = std::max(u1, q.u);
                        v0 = std::min(v0, q.v); v1 = std::max(v1, q.v);
                    }
                    std::fprintf(stderr,
                                 "[%s] f=%d e=%d cl=%d n=%zu u[%.4f,%.4f] v[%.4f,%.4f]\n",
                                 tag, kv.first, sp.sec_edge, (int)sp.closed,
                                 sp.pc.size(), u0, u1, v0, v1);
                    for (size_t i = 1; i < sp.pc.size(); i++) {
                        double du = std::abs(sp.pc[i].u - sp.pc[i - 1].u);
                        double dv = std::abs(sp.pc[i].v - sp.pc[i - 1].v);
                        if (du > 1.0 || dv > 1.0)
                            std::fprintf(stderr,
                                         "[%s] JUMP f=%d e=%d i=%zu du=%.4f dv=%.4f at (%.4f,%.4f)\n",
                                         tag, kv.first, sp.sec_edge, i, du, dv,
                                         sp.pc[i].u, sp.pc[i].v);
                    }
                    if (std::getenv("V3SEAMDBG")) {
                        std::fprintf(stderr, "[%s] vtrace f=%d e=%d:", tag,
                                     kv.first, sp.sec_edge);
                        for (size_t i = 0; i < sp.pc.size(); i++)
                            if (std::abs(std::fmod(sp.pc[i].v, TWO_PI)) > PI)
                                std::fprintf(stderr, " %zu:%.3f", i,
                                             sp.pc[i].v);
                        std::fprintf(stderr, "\n");
                    }
                }
        };
        dump("rawA", pcs_A);
        dump("rawB", pcs_B);
    }

    if (dbg)
        std::fprintf(stderr, "[bool] sections=%zu sd=%d volA=%.3f volB=%.3f\n",
                     sec_edges.size(), (int)sd_pair, A.signed_volume(10),
                     B.signed_volume(10));
    if (dbg) {
        for (auto& ps : pair_secs)
            std::fprintf(stderr, "[pair] A%d(%d) B%d(%d): curves=%zu\n", ps.ai,
                         (int)A.srfs[A.faces[ps.ai].srf].k, ps.bi,
                         (int)B.srfs[B.faces[ps.bi].srf].k, ps.r.curves.size());
    }

    // ---- containment / touch handling ----------------------------------------
    if (!any_section) {
        // No boundary interference curves. Containment must be decided with
        // MULTI-POINT sampling: a single probe vertex can land exactly ON the
        // other boundary in tangent-contact configurations (e.g. a torus
        // touching the containing box's walls), which a point test would call
        // neither IN nor OUT. Rule: X counts as inside Y iff no sample is OUT
        // and at least one is strictly IN.
        auto sample_points = [&](const Solid& X) {
            std::vector<V3> out;
            for (auto& v : X.verts) out.push_back(v.p);
            for (auto& e : X.edges)
                if (!e.degenerate)
                    out.push_back(e.c.eval(0.5 * (e.t0 + e.t1)));
            for (auto& f : X.faces) {
                std::vector<V3> tris;
                tessellate_face(X, f, 4, tris);
                for (size_t i = 0; i + 2 < tris.size(); i += 3)
                    out.push_back((tris[i] + tris[i + 1] + tris[i + 2]) *
                                  (1.0 / 3.0));
            }
            return out;
        };
        auto inside_of = [&](const Solid& X, const Solid& Y) {
            auto pts = sample_points(X);
            bool any_in = false;
            for (auto& p : pts) {
                PtCls c = classify_point(Y, p, tol * 10);
                if (c == PtCls::OUT) return false;
                if (c == PtCls::IN) any_in = true;
            }
            return any_in;
        };
        bool ca = inside_of(A, B); // A contained in B
        bool cb = inside_of(B, A); // B contained in A
        if (dbg)
            std::fprintf(stderr, "[bool] no-section containment: AinB=%d BinA=%d\n",
                         (int)ca, (int)cb);
        if (intersected) *intersected = ca || cb;
        if (ca) {
            // A inside B
            if (op == BoolOp::COMMON) return A;
            if (op == BoolOp::CUT) return Solid{};
            return B; // FUSE
        }
        if (cb) {
            // B inside A
            if (op == BoolOp::COMMON) return B;
            if (op == BoolOp::FUSE) return A;
            // CUT: A with B as a void shell (reversed)
            Solid R = A;
            append_solid(R, B, true);
            return R;
        }
        // truly disjoint (bboxes overlapped but no containment)
        if (op == BoolOp::CUT) return A;
        if (op == BoolOp::COMMON) return Solid{};
        {
            Solid R = A;
            append_solid(R, B, false);
            return R;
        }
    }

    // ---- prepare SecPCs for their faces: band alignment on periodic surfaces.
    // Each face's polyline is anchored INDEPENDENTLY (open pieces: whole-period
    // shift; closed pieces: cut at the band-cut into junction arcs). The 3D
    // section edge is shared; pcurves are per-face views. Runs once per periodic
    // coordinate (a torus face has BOTH seams as boundary edges): mc is the
    // coordinate being aligned, mo the carried/interpolated one. ---------------
    auto prepare_dir = [&](Solid& X, std::map<int, std::vector<SecPC>>& pcs,
                           double UV2::*mc, double UV2::*mo) {
        for (auto& kv : pcs) {
            Face& F = X.faces[kv.first];
            const Srf& S = X.srfs[F.srf];
            bool per = mc == &UV2::u ? S.periodic_u() : S.periodic_v();
            if (!per) continue;
            double u0 = 1e300, u1 = -1e300;
            for (const Loop& l : F.loops)
                for (int cei : l.ces)
                    for (auto& q : X.coedges[cei].pc) {
                        u0 = std::min(u0, q.*mc);
                        u1 = std::max(u1, q.*mc);
                    }
            if (u0 > u1) continue;
            double uc = 0.5 * (u0 + u1);
            std::vector<SecPC> out;
            for (auto& sp : kv.second) {
                if (sp.pc.size() < 2) { out.push_back(sp); continue; }
                // re-unwrap the chain in place: refine_boundary computes run
                // endpoint UVs with a fresh canonical uv_of, leaving 2*pi
                // jumps INSIDE a chain; those corrupt the center rule below
                // (identity for already-continuous chains)
                for (size_t i = 1; i < sp.pc.size(); i++)
                    sp.pc[i].*mc = unwrap_near(sp.pc[i].*mc, sp.pc[i - 1].*mc);
                // whole-period shift into the band (chain center rule)
                double span = sp.pc.back().*mc - sp.pc.front().*mc;
                double cen = sp.pc.front().*mc + 0.5 * span;
                double k = TWO_PI * std::round((uc - cen) / TWO_PI);
                if (k != 0)
                    for (auto& q : sp.pc) q.*mc += k;
                if (std::getenv("V3PREPDBG"))
                    std::fprintf(stderr,
                                 "[prepd] %c f=%d e=%d cl=%d uc=%.3f cen=%.3f k=%g\n",
                                 mc == &UV2::u ? 'u' : 'v', kv.first, sp.sec_edge,
                                 (int)sp.closed, uc, cen, k);
                if (!sp.closed) { out.push_back(sp); continue; }
                // closed section: cut at the band-cut lines into junction arcs
                auto at_cut = [&](double u) {
                    return std::abs(u - u0) < 1e-6 ||
                           std::abs(u - u0 - TWO_PI) < 1e-6;
                };
                std::vector<double> cuts;
                for (double c : {u0, u0 + TWO_PI}) {
                    // cut only when the chain genuinely SPANS the line (samples
                    // strictly on both sides — a walker seed can sit exactly on
                    // the seam, defeating a strict-straddle test). A chain that
                    // merely touches the line (its closure point on the seam)
                    // must stay whole: it closes as a self-loop in the
                    // arrangement (sph_cyl's full-latitude circle).
                    double lo = 1e300, hi = -1e300;
                    for (auto& q : sp.pc) {
                        lo = std::min(lo, q.*mc);
                        hi = std::max(hi, q.*mc);
                    }
                    if (lo < c - 1e-9 && hi > c + 1e-9) cuts.push_back(c);
                }
                if (std::getenv("V3PREPDBG"))
                    std::fprintf(stderr, "[prepd]   cuts=%zu\n", cuts.size());
                if (cuts.empty()) { out.push_back(sp); continue; }
                // per-sample edge ids (merged chains span several sections)
                std::vector<int> sp_e;
                if (!sp.pc_edge.empty()) sp_e = sp.pc_edge;
                else sp_e.assign(sp.pc.size(), sp.sec_edge);
                // insert interpolated crossing points
                std::vector<UV2> aug;
                std::vector<int> aug_e;
                for (size_t i = 0; i + 1 < sp.pc.size(); i++) {
                    aug.push_back(sp.pc[i]);
                    aug_e.push_back(sp_e[i]);
                    double ua = sp.pc[i].*mc, ub = sp.pc[i + 1].*mc;
                    for (double c : cuts)
                        if ((ua < c && ub > c) || (ua > c && ub < c)) {
                            double fr = (c - ua) / (ub - ua);
                            UV2 nq;
                            nq.*mc = c;
                            nq.*mo = sp.pc[i].*mo +
                                     (sp.pc[i + 1].*mo - sp.pc[i].*mo) * fr;
                            aug.push_back(nq);
                            aug_e.push_back(sp_e[i]);
                        }
                }
                aug.push_back(sp.pc.back());
                aug_e.push_back(sp_e.back());
                // closed: also consider the wrap segment (back -> front), but
                // ONLY when the chain is not already explicitly closed
                // (last == first mod period): there the wrap segment is
                // degenerate and re-finds the same crossing in reverse,
                // appending a phantom point that becomes a retrace spur
                {
                    double du = aug.back().*mc - aug.front().*mc;
                    double dv = aug.back().*mo - aug.front().*mo;
                    double dper = std::abs(du) - TWO_PI;
                    bool expl_closed =
                        (du * du + dv * dv < 1e-12) ||
                        (dper * dper < 1e-12 && dv * dv < 1e-12);
                    if (!expl_closed) {
                        double ua = aug.back().*mc, ub = aug.front().*mc + TWO_PI;
                        for (double c : cuts)
                            if ((ua < c && ub > c) || (ua > c && ub < c)) {
                                double fr = (c - ua) / (ub - ua);
                                UV2 nq;
                                nq.*mc = c;
                                nq.*mo = aug.back().*mo +
                                         (aug.front().*mo - aug.back().*mo) * fr;
                                aug.push_back(nq);
                                aug_e.push_back(aug_e.back());
                            }
                    }
                }
                // rotate to start at a cut
                int rot = 0;
                for (size_t i = 0; i < aug.size(); i++)
                    if (at_cut(aug[i].*mc)) { rot = (int)i; break; }
                if (rot > 0) {
                    std::rotate(aug.begin(), aug.begin() + rot, aug.end());
                    std::rotate(aug_e.begin(), aug_e.begin() + rot, aug_e.end());
                }
                // continuity unwrap (rotation broke monotonicity at the join)
                for (size_t i = 1; i < aug.size(); i++)
                    aug[i].*mc = unwrap_near(aug[i].*mc, aug[i - 1].*mc);
                // shift the rotated chain into the face band (chain center rule)
                {
                    double c0 = aug.front().*mc, c1 = aug.back().*mc;
                    double cc = 0.5 * (c0 + c1);
                    double k2 = TWO_PI * std::round((uc - cc) / TWO_PI);
                    if (k2 != 0)
                        for (auto& q : aug) q.*mc += k2;
                }
                // split at cut samples into junction arcs; skip zero-extent
                // phantoms (the walker can put two consecutive samples within
                // 1e-6 of the cut line: seed exactly on an iso + walk endpoint)
                if (std::getenv("V3PREPDBG")) {
                    std::fprintf(stderr, "[prepd]   aug=%zu cutsamples:",
                                 aug.size());
                    for (size_t i = 0; i < aug.size(); i++)
                        if (at_cut(aug[i].*mc)) std::fprintf(stderr, " %zu", i);
                    std::fprintf(stderr, "\n");
                    for (size_t i = 0; i < aug.size(); i++)
                        if (at_cut(aug[i].*mc)) {
                            std::fprintf(stderr, "[prepd]   around %zu:", i);
                            for (size_t k = i > 2 ? i - 2 : 0;
                                 k < i + 3 && k < aug.size(); k++)
                                std::fprintf(stderr, " (%.4f,%.4f)", aug[k].u,
                                             aug[k].v);
                            std::fprintf(stderr, "\n");
                        }
                }
                auto arc_extent = [&](const SecPC& a) {
                    double ext = 0;
                    for (size_t k = 1; k < a.pc.size(); k++)
                        ext += std::hypot(a.pc[k].*mc - a.pc[k - 1].*mc,
                                          a.pc[k].*mo - a.pc[k - 1].*mo);
                    return ext;
                };
                SecPC cur;
                cur.sec_edge = sp.sec_edge;
                cur.closed = false;
                cur.pc.push_back(aug[0]);
                cur.pc_edge.push_back(aug_e[0]);
                for (size_t i = 1; i < aug.size(); i++) {
                    if (at_cut(aug[i].*mc)) {
                        // split only at a genuine CROSSING: a sample sitting
                        // on the line with both neighbors on the same side is
                        // a TOUCH (a walker seed lies exactly on an iso line)
                        // — cutting there spawns spur arcs
                        double c = std::abs(aug[i].*mc - u0) < 1e-6
                                       ? u0
                                       : u0 + TWO_PI;
                        double sa = aug[i - 1].*mc - c;
                        double nxt = i + 1 < aug.size()
                                         ? aug[i + 1].*mc
                                         : aug[0].*mc + TWO_PI;
                        double sb = nxt - c;
                        if (sa * sb > 0) {
                            cur.pc.push_back(aug[i]);
                            cur.pc_edge.push_back(aug_e[i]);
                            continue;
                        }
                        cur.pc.push_back(aug[i]);
                        cur.pc_edge.push_back(aug_e[i]);
                        if (arc_extent(cur) > 1e-9) out.push_back(cur);
                        cur = SecPC{};
                        cur.sec_edge = sp.sec_edge;
                        cur.closed = false;
                        cur.pc.push_back(aug[i]);
                        cur.pc_edge.push_back(aug_e[i]);
                    } else {
                        cur.pc.push_back(aug[i]);
                        cur.pc_edge.push_back(aug_e[i]);
                    }
                }
                // tail arc: append the first cut's image one period along the
                // chain's own direction (continuity-based: +2pi for ascending
                // chains, -2pi for descending -- a blind += TWO_PI is
                // discontinuous for descending chains)
                if (cur.pc.size() > 1 && arc_extent(cur) > 1e-9) {
                    UV2 last = aug[0];
                    last.*mc = unwrap_near(aug[0].*mc + TWO_PI, cur.pc.back().*mc);
                    cur.pc.push_back(last);
                    cur.pc_edge.push_back(aug_e[0]);
                    out.push_back(cur);
                }
            }
            // keep every arc INSIDE the band: arcs are cut at the band lines,
            // so none spans a line and a whole-period shift is exact. An arc
            // left outside the band (a section loop crossing the seam) lets
            // the flat-chart traversal walk across the frame boundary,
            // producing peninsula zones whose holes mis-emit (sph_tor_seam).
            for (auto& sp : out) {
                if (sp.pc.size() < 2) continue;
                double c0 = 1e300, c1 = -1e300;
                for (auto& q : sp.pc) {
                    c0 = std::min(c0, q.*mc);
                    c1 = std::max(c1, q.*mc);
                }
                double cc = 0.5 * (c0 + c1);
                double k = TWO_PI * std::floor((cc - u0) / TWO_PI);
                if (k != 0)
                    for (auto& q : sp.pc) q.*mc -= k;
            }
            // re-split junction arcs at section-edge transitions: every arc
            // must carry the single edge id its counterpart on the other
            // operand's face uses, or the output weld sees one physical curve
            // under two ids (naked edges)
            {
                std::vector<SecPC> split_out;
                for (auto& sp : out) {
                    if (sp.pc_edge.size() != sp.pc.size() || sp.pc.size() < 2) {
                        split_out.push_back(std::move(sp));
                        continue;
                    }
                    SecPC cur;
                    cur.closed = false;
                    cur.sec_edge = sp.pc_edge[0];
                    cur.pc.push_back(sp.pc[0]);
                    for (size_t i = 1; i < sp.pc.size(); i++) {
                        if (sp.pc_edge[i] != cur.sec_edge) {
                            if (cur.pc.size() > 1)
                                split_out.push_back(cur);
                            cur = SecPC{};
                            cur.sec_edge = sp.pc_edge[i];
                            cur.pc.push_back(sp.pc[i - 1]); // shared joint
                        }
                        cur.pc.push_back(sp.pc[i]);
                    }
                    if (cur.pc.size() > 1) split_out.push_back(cur);
                }
                kv.second = split_out;
            }
        }
    };
    auto prepare = [&](Solid& X, std::map<int, std::vector<SecPC>>& pcs) {
        prepare_dir(X, pcs, &UV2::u, &UV2::v);
        prepare_dir(X, pcs, &UV2::v, &UV2::u);
    };
    // merge section pieces that join end-to-end (3D-coincident endpoints,
    // whole-period chart shifts allowed) into single chains. A chain that
    // physically closes becomes a CLOSED SecPC so prepare() cuts it at the
    // band cuts into junction arcs -- the arrangement merges nodes by chart
    // position, so a loop crossing the face seam only closes via the
    // closed-section machinery (rot* cylinder loops, box_sph corner triangles).
    auto merge_sections = [&](Solid& X, std::map<int, std::vector<SecPC>>& pcs) {
        for (auto& kv : pcs) {
            Face& F = X.faces[kv.first];
            const Srf& S = X.srfs[F.srf];
            // only periodic faces need this: open pieces on planes already
            // chain correctly via chart-position node merging
            if (!S.periodic_u() && !S.periodic_v()) continue;
            double gate = 0.02 * diag + tol * 100;
            auto& v = kv.second;
            auto dist3 = [&](const UV2& a, const UV2& b) {
                return S.eval(a.u, a.v).dist(S.eval(b.u, b.v));
            };
            auto rev = [](SecPC& s) {
                std::reverse(s.pc.begin(), s.pc.end());
                std::reverse(s.pc_edge.begin(), s.pc_edge.end());
                std::swap(s.end_vtx[0], s.end_vtx[1]);
                std::swap(s.end_edge[0], s.end_edge[1]);
                std::swap(s.end_t[0], s.end_t[1]);
            };
            // 3D distance to the face boundary: a junction ON the boundary
            // must stay a free end so attach() can pave it -- merging there
            // would internalize the point and lose the pave
            auto bnd_dist = [&](const V3& p) {
                double best = 1e300;
                for (const Loop& l : F.loops)
                    for (int cei : l.ces) {
                        const CoEdge& ce = X.coedges[cei];
                        const Edge& e = X.edges[ce.edge];
                        if (e.degenerate) {
                            best = std::min(best, X.verts[e.v0].p.dist(p));
                            continue;
                        }
                        double t;
                        if (!e.c.project(p, t)) continue;
                        t = std::max(e.t0, std::min(e.t1, t));
                        best = std::min(best, e.c.eval(t).dist(p));
                    }
                return best;
            };
            // join B_ onto the end of A_ (whole-period shift of B_ allowed)
            auto try_join = [&](SecPC& A_, SecPC& B_) {
                if (dist3(A_.pc.back(), B_.pc.front()) > gate) return false;
                // boundary junctions (section endpoints ON a boundary edge =
                // pierce points, deflection-close) must stay free for attach;
                // junctions merely NEAR the boundary (seam neighborhood) must
                // still merge -- the gate is deflection-scale, not diag-scale
                double bd = bnd_dist(S.eval(A_.pc.back().u, A_.pc.back().v));
                if (bd <= tol * 100 + 0.005) {
                    if (std::getenv("V3MRGDBG"))
                        std::fprintf(stderr,
                                     "[mrg] face %ld: BLOCKED bnd=%.4g at (%.4f,%.4f)\n",
                                     (long)(&F - &X.faces[0]), bd, A_.pc.back().u,
                                     A_.pc.back().v);
                    return false;
                }
                if (std::getenv("V3MRGDBG"))
                    std::fprintf(stderr,
                                 "[mrg] face %ld: join e%d->e%d at (%.4f,%.4f)\n",
                                 (long)(&F - &X.faces[0]), A_.sec_edge - 1000000,
                                 B_.sec_edge - 1000000, A_.pc.back().u,
                                 A_.pc.back().v);
                if (S.periodic_u()) {
                    double sh = std::round((A_.pc.back().u - B_.pc.front().u) /
                                           TWO_PI) *
                                TWO_PI;
                    for (auto& q : B_.pc) q.u += sh;
                }
                B_.pc.front() = A_.pc.back(); // exact joint
                // per-sample edge ids: the merged chain spans several section
                // edges; preserve which edge each sample came from so the
                // junction arcs can be re-split with the correct id
                if (A_.pc_edge.empty())
                    A_.pc_edge.assign(A_.pc.size(), A_.sec_edge);
                if (B_.pc_edge.empty())
                    B_.pc_edge.assign(B_.pc.size(), B_.sec_edge);
                A_.pc.insert(A_.pc.end(), B_.pc.begin() + 1, B_.pc.end());
                A_.pc_edge.insert(A_.pc_edge.end(), B_.pc_edge.begin() + 1,
                                  B_.pc_edge.end());
                A_.end_vtx[1] = B_.end_vtx[1];
                A_.end_edge[1] = B_.end_edge[1];
                A_.end_t[1] = B_.end_t[1];
                return true;
            };
            bool any = true;
            while (any) {
                any = false;
                for (size_t i = 0; i < v.size() && !any; i++)
                    for (size_t j = i + 1; j < v.size() && !any; j++) {
                        SecPC &P = v[i], &Q = v[j];
                        if (P.closed || Q.closed || P.pc.size() < 2 ||
                            Q.pc.size() < 2)
                            continue;
                        bool ok = try_join(P, Q);        // P.back - Q.front
                        if (!ok) { rev(Q); ok = try_join(P, Q); }  // P.back - Q.back
                        if (!ok) { rev(P); ok = try_join(P, Q); }  // P.front - Q.back
                        if (!ok) { rev(Q); ok = try_join(P, Q); }  // P.front - Q.front
                        if (!ok) { rev(P); rev(Q); continue; }     // restore
                        v.erase(v.begin() + j);
                        any = true;
                    }
            }
            // a merged chain whose ends physically coincide is a closed loop
            // (unless the closure point sits on the boundary: then both ends
            // must stay free for attach)
            for (auto& sp : v)
                if (!sp.closed && sp.pc.size() > 8 &&
                    dist3(sp.pc.front(), sp.pc.back()) < gate &&
                    bnd_dist(S.eval(sp.pc.front().u, sp.pc.front().v)) >
                        tol * 100 + 0.005) {
                    sp.closed = true;
                    // exact closure (mod period): deflection-twin endpoints
                    // would otherwise miss the chart-position node merge
                    if (S.periodic_u()) {
                        double per = std::round((sp.pc.back().u - sp.pc.front().u) /
                                                TWO_PI) *
                                     TWO_PI;
                        sp.pc.back() = {sp.pc.front().u + per, sp.pc.front().v};
                    } else {
                        sp.pc.back() = sp.pc.front();
                    }
                    sp.end_vtx[0] = sp.end_vtx[1] = -1;
                    sp.end_edge[0] = sp.end_edge[1] = -1;
                }
        }
    };
    merge_sections(A, pcs_A);
    merge_sections(B, pcs_B);

    // Non-periodic (freeform) faces: marched section endpoints scatter by
    // ~walk-step (~3e-3) at interior triple points — far above the
    // arrangement's chart-position merge tolerance (~1e-6) — so the junctions
    // never chain and whole faces drop out unsplit (chairs: kept=0). Snap
    // 3D-coincident endpoints to the SAME chart point and unify their weld
    // vertices. Chains are NOT concatenated (unlike periodic merge_sections):
    // concatenation mixes several section edge ids into one SecPC and the
    // pc_edge re-split only exists in the periodic closed-cut path (one
    // physical curve under two ids -> naked). Pierce points on the boundary
    // stay free for attach() (same guard as try_join).
    std::map<int, int> vtx_alias; // unified section-endpoint vertices: qv -> pv
    auto vtx_resolve = [&](int v) {
        while (true) {
            auto it = vtx_alias.find(v);
            if (it == vtx_alias.end()) return v;
            v = it->second;
        }
    };
    auto snap_section_ends = [&](Solid& X, std::map<int, std::vector<SecPC>>& pcs) {
        const double gate = std::max(tol * 100, 0.01); // attach's 3D gate scale
        for (auto& kv : pcs) {
            Face& F = X.faces[kv.first];
            const Srf& S = X.srfs[F.srf];
            if (S.periodic_u() || S.periodic_v()) continue; // merge_sections' turf
            auto& v = kv.second;
            auto dist3 = [&](const UV2& a, const UV2& b) {
                return S.eval(a.u, a.v).dist(S.eval(b.u, b.v));
            };
            auto bnd_dist = [&](const V3& p) {
                double best = 1e300;
                for (const Loop& l : F.loops)
                    for (int cei : l.ces) {
                        const CoEdge& ce = X.coedges[cei];
                        const Edge& e = X.edges[ce.edge];
                        if (e.degenerate) {
                            best = std::min(best, X.verts[e.v0].p.dist(p));
                            continue;
                        }
                        double t;
                        if (!e.c.project(p, t)) continue;
                        t = std::max(e.t0, std::min(e.t1, t));
                        best = std::min(best, e.c.eval(t).dist(p));
                    }
                return best;
            };
            for (size_t i = 0; i < v.size(); i++)
                for (size_t j = i + 1; j < v.size(); j++) {
                    SecPC &P = v[i], &Q = v[j];
                    if (P.closed || Q.closed || P.pc.empty() || Q.pc.empty())
                        continue;
                    for (int pe = 0; pe < 2; pe++)
                        for (int qe = 0; qe < 2; qe++) {
                            UV2& a = pe ? P.pc.back() : P.pc.front();
                            UV2& b = qe ? Q.pc.back() : Q.pc.front();
                            if (dist3(a, b) > gate) continue;
                            if (bnd_dist(S.eval(a.u, a.v)) <= tol * 100 + 0.005)
                                continue; // pierce point: attach owns it
                            b = a; // exact joint: one node in the arrangement
                            int pv = vtx_resolve(P.end_vtx[pe]);
                            int qv = vtx_resolve(Q.end_vtx[qe]);
                            if (pv < 0 || qv < 0 || pv == qv) continue;
                            double d3 = weld.verts[pv].p.dist(weld.verts[qv].p);
                            weld.verts[pv].tol =
                                std::max(weld.verts[pv].tol,
                                         weld.verts[qv].tol + d3);
                            vtx_alias[qv] = pv; // sec_edges/end_vtx fixed below
                            if (std::getenv("V3MRGDBG"))
                                std::fprintf(stderr,
                                             "[mrg] face %ld: snap e%d.%d=e%d.%d "
                                             "at (%.4f,%.4f) d3=%.3g\n",
                                             (long)(&F - &X.faces[0]),
                                             P.sec_edge - SEC_BASE, pe,
                                             Q.sec_edge - SEC_BASE, qe, a.u, a.v,
                                             d3);
                        }
                }
        }
    };
    snap_section_ends(A, pcs_A);
    snap_section_ends(B, pcs_B);
    if (!vtx_alias.empty()) {
        for (auto& se : sec_edges) {
            se.v0 = vtx_resolve(se.v0);
            se.v1 = vtx_resolve(se.v1);
        }
        for (auto* pp : {&pcs_A, &pcs_B})
            for (auto& kv : *pp)
                for (auto& sp : kv.second)
                    for (int e = 0; e < 2; e++)
                        if (sp.end_vtx[e] >= 0)
                            sp.end_vtx[e] = vtx_resolve(sp.end_vtx[e]);
    }

    prepare(A, pcs_A);
    prepare(B, pcs_B);
    if (std::getenv("V3PREPDBG")) {
        auto dump = [&](const char* tag, std::map<int, std::vector<SecPC>>& pcs) {
            for (auto& kv : pcs)
                for (auto& sp : kv.second) {
                    double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
                    for (auto& q : sp.pc) {
                        u0 = std::min(u0, q.u); u1 = std::max(u1, q.u);
                        v0 = std::min(v0, q.v); v1 = std::max(v1, q.v);
                    }
                    std::fprintf(stderr,
                                 "[%s] f=%d e=%d cl=%d n=%zu u[%.4f,%.4f] v[%.4f,%.4f]\n",
                                 tag, kv.first, sp.sec_edge, (int)sp.closed,
                                 sp.pc.size(), u0, u1, v0, v1);
                    for (size_t i = 1; i < sp.pc.size(); i++) {
                        double du = std::abs(sp.pc[i].u - sp.pc[i - 1].u);
                        double dv = std::abs(sp.pc[i].v - sp.pc[i - 1].v);
                        if (du > 1.0 || dv > 1.0)
                            std::fprintf(stderr,
                                         "[%s] JUMP f=%d e=%d i=%zu du=%.4f dv=%.4f at (%.4f,%.4f)\n",
                                         tag, kv.first, sp.sec_edge, i, du, dv,
                                         sp.pc[i].u, sp.pc[i].v);
                    }
                    if (std::getenv("V3SEAMDBG")) {
                        std::fprintf(stderr, "[%s] vtrace f=%d e=%d:", tag,
                                     kv.first, sp.sec_edge);
                        for (size_t i = 0; i < sp.pc.size(); i++)
                            if (std::abs(std::fmod(sp.pc[i].v, TWO_PI)) > PI)
                                std::fprintf(stderr, " %zu:%.3f", i,
                                             sp.pc[i].v);
                        std::fprintf(stderr, "\n");
                    }
                }
        };
        dump("prepA", pcs_A);
        dump("prepB", pcs_B);
    }

    // ---- EF piercings: operand edges vs other solid's faces -------------------
    struct Pierce {
        V3 p;
        int edge;
        double t;
    };
    std::vector<Pierce> pierce_pts;
    auto pierce = [&](Solid& X, Solid& Y, std::map<int, std::vector<double>>& paves,
                      std::vector<V3>& y_lo, std::vector<V3>& y_hi) {
        for (size_t ei = 0; ei < X.edges.size(); ei++) {
            Edge& e = X.edges[ei];
            if (e.degenerate) continue;
            for (size_t fi = 0; fi < Y.faces.size(); fi++) {
                Face& F = Y.faces[fi];
                V3 p0 = e.c.eval(e.t0), p1 = e.c.eval(e.t1);
                V3 lo{std::min(p0.x, p1.x), std::min(p0.y, p1.y), std::min(p0.z, p1.z)};
                V3 hi{std::max(p0.x, p1.x), std::max(p0.y, p1.y), std::max(p0.z, p1.z)};
                if (!boxes_overlap(lo, hi, y_lo[fi], y_hi[fi], tol * 10)) continue;
                auto hits = csi(e.c, e.t0, e.t1, Y.srfs[F.srf], tol);
                if (std::getenv("V3ATTDBG") && !hits.empty())
                    std::fprintf(stderr, "[ef] edge %zu x face %zu: %zu hits\n", ei, fi,
                                 hits.size());
                for (auto& h : hits) {
                    if (h.t < e.t0 + 1e-9 || h.t > e.t1 - 1e-9) continue;
                    double u = h.u, v = h.v;
                    if (std::getenv("V3ATTDBG"))
                        std::fprintf(stderr,
                                     "[ef]   hit t=%.4f uv=(%.4f,%.4f) inface=%d\n", h.t,
                                     u, v, (int)uv_in_face(Y, F, u, v));
                    if (uv_in_face(Y, F, u, v)) {
                        paves[(int)ei].push_back(h.t);
                        pierce_pts.push_back({h.p, (int)ei, h.t});
                    }
                }
            }
        }
    };
    pierce(A, B, paves_A, fb_lo, fb_hi);
    pierce(B, A, paves_B, fa_lo, fa_hi);
    if (dbg)
        std::fprintf(stderr, "[bool] pierce points: %zu\n", pierce_pts.size());

    // snap section endpoints to 3D-coincident pierce points: the pierce is the
    // EXACT edge-side intersection; section polyline endpoints are deflection-
    // approximate. Snapping makes shared junctions exact so the UV arrangement
    // merges them (OCCT: one vertex per feature, incident paves share it).
    auto snap_to_pierces = [&](Solid& X, std::map<int, std::vector<SecPC>>& pcs,
                               const std::vector<Pierce>& pts) {
        if (pts.empty()) return;
        double gate = 0.02 * diag + tol * 100;
        for (auto& kv : pcs) {
            Face& F = X.faces[kv.first];
            const Srf& S = X.srfs[F.srf];
            for (auto& sp : kv.second) {
                for (int end = 0; end < 2; end++) {
                    if (sp.end_edge[end] >= 0) continue; // already attached
                    V3 q3 = sp.end_vtx[end] >= 0 ? weld.verts[sp.end_vtx[end]].p
                                                 : S.eval(end == 0 ? sp.pc.front().u
                                                                   : sp.pc.back().u,
                                                          end == 0 ? sp.pc.front().v
                                                                   : sp.pc.back().v);
                    double best = gate;
                    int bi = -1;
                    for (size_t i = 0; i < pts.size(); i++)
                        if (pts[i].p.dist(q3) < best) { best = pts[i].p.dist(q3); bi = (int)i; }
                    if (bi < 0) continue;
                    if (std::getenv("V3ATTDBG"))
                        std::fprintf(stderr,
                                     "[snap] sec=%d end=%d -> pierce %d dist=%.4g\n",
                                     sp.sec_edge - SEC_BASE, end, bi, best);
                    // recompute the endpoint UV on this face at the pierce point
                    double u, v;
                    if (!S.uv_of(pts[bi].p, u, v)) continue;
                    // anchor at the CURRENT endpoint's chart position (not the
                    // adjacent sample): merged chains place junctions at
                    // shifted periods; re-anchoring at the neighbor would
                    // tear the joint one period apart again
                    if (S.periodic_u())
                        u = unwrap_near(u, end == 0 ? sp.pc.front().u
                                                    : sp.pc.back().u);
                    if (S.periodic_v())
                        v = unwrap_near(v, end == 0 ? sp.pc.front().v
                                                    : sp.pc.back().v);
                    if (end == 0) sp.pc.front() = {u, v};
                    else sp.pc.back() = {u, v};
                    // and the 3D vertex: attach re-derives the pave t from it
                    sp.end_vtx[end] = weld.at(pts[bi].p);
                }
            }
        }
    };
    snap_to_pierces(A, pcs_A, pierce_pts);
    snap_to_pierces(B, pcs_B, pierce_pts);

    // ---- attach section endpoints to boundary edges (paves) -------------------
    auto attach = [&](Solid& X, std::map<int, std::vector<SecPC>>& pcs,
                      std::map<int, std::vector<double>>& paves) {
        for (auto& kv : pcs) {
            Face& F = X.faces[kv.first];
            for (auto& sp : kv.second) {
                for (int end = 0; end < 2; end++) {
                    UV2 q = end == 0 ? sp.pc.front() : sp.pc.back();
                    // nearest boundary coedge point in UV (with 3D distance gate)
                    double best = 1e300;
                    int best_ce = -1;
                    double best_t = 0;
                    V3 q3 = sp.end_vtx[end] >= 0 ? weld.verts[sp.end_vtx[end]].p
                                                 : F.srf >= 0 ? X.srfs[F.srf].eval(q.u, q.v) : V3{};
                    for (const Loop& l : F.loops)
                        for (int cei : l.ces) {
                            const CoEdge& ce = X.coedges[cei];
                            const Edge& e = X.edges[ce.edge];
                            if (e.degenerate) continue;
                            for (size_t i = 0; i + 1 < ce.pc.size(); i++) {
                                double s, t2;
                                (void)s; (void)t2;
                                // project q onto segment
                                const UV2& a = ce.pc[i];
                                const UV2& b = ce.pc[i + 1];
                                double ux = b.u - a.u, uy = b.v - a.v;
                                double L2 = ux * ux + uy * uy;
                                double fr = L2 > 1e-300
                                                ? ((q.u - a.u) * ux + (q.v - a.v) * uy) / L2
                                                : 0;
                                fr = std::max(0.0, std::min(1.0, fr));
                                double du = q.u - (a.u + ux * fr),
                                       dv = q.v - (a.v + uy * fr);
                                double d2 = du * du + dv * dv;
                                if (d2 < best) {
                                    // 3D gate: project the endpoint's 3D point
                                    // onto the edge curve (frame-independent;
                                    // the pt<->UV lerp is not)
                                    double te;
                                    if (!e.c.project(q3, te)) continue;
                                    if (e.c.is_closed() && e.c.period() > 0)
                                        te = unwrap_near(te, 0.5 * (e.t0 + e.t1));
                                    if (te < e.t0 - 1e-9 || te > e.t1 + 1e-9) continue;
                                    te = std::max(e.t0, std::min(e.t1, te));
                                    V3 p3 = e.c.eval(te);
                                    // 3D gate: the UV check above is the hard
                                    // guard; the 3D gate absorbs accumulated
                                    // deflection/fit tolerance (OCCT vertex-
                                    // tolerance enlargement for incident paves)
                                    double gate = std::max(tol * 100, 0.01);
                                    if (p3.dist(q3) < gate) {
                                        best = d2;
                                        best_ce = ce.edge;
                                        best_t = te;
                                    }
                                }
                            }
                        }
                    if (best_ce >= 0 && best < 4e-4) {
                        paves[best_ce].push_back(best_t);
                        sp.end_edge[end] = best_ce;
                        sp.end_t[end] = best_t;
                        if (std::getenv("V3BOOLDBG"))
                            std::fprintf(stderr,
                                         "[attach] sec=%d end=%d -> edge=%d t=%.4f\n",
                                         sp.sec_edge - SEC_BASE, end, best_ce, best_t);
                    } else if (std::getenv("V3BOOLDBG") && sp.sec_edge >= SEC_BASE) {
                        std::fprintf(stderr,
                                     "[attach] MISS sec=%d end=%d best=%.3g best_ce=%d q=(%.4f,%.4f)\n",
                                     sp.sec_edge - SEC_BASE, end, best, best_ce, q.u, q.v);
                    }
                }
            }
        }
    };
    attach(A, pcs_A, paves_A);
    attach(B, pcs_B, paves_B);

    // ---- split edges -----------------------------------------------------------
    if (dbg)
        for (auto& kv : paves_A) {
            std::fprintf(stderr, "[pavesA] edge %d:", kv.first);
            for (double t : kv.second) std::fprintf(stderr, " %.6f", t);
            std::fprintf(stderr, "\n");
        }
    std::map<std::pair<int, double>, int> vtx_A, vtx_B;
    auto pieces_A = split_edges_at_paves(A, paves_A, &vtx_A);
    auto pieces_B = split_edges_at_paves(B, paves_B, &vtx_B);
    if (dbg) {
        for (size_t fi = 0; fi < A.faces.size() && fi < 2; fi++)
            for (auto& l : A.faces[fi].loops) {
                std::fprintf(stderr, "[loopA] face %zu:", fi);
                for (int cei : l.ces) {
                    const CoEdge& ce = A.coedges[cei];
                    std::fprintf(stderr, " e%d[%.2f>%.2f]", ce.edge, ce.pc.front().u,
                                 ce.pc.back().u);
                }
                std::fprintf(stderr, "\n");
            }
    }

    // snap attached SecPC endpoints to their pave junctions (exact by
    // construction): pave vertex id + the junction UV on this face
    auto snap_endpoints = [&](Solid& X, std::map<int, std::vector<SecPC>>& pcs,
                              std::map<std::pair<int, double>, int>& vtx_of,
                              const std::map<int, std::vector<std::array<double, 3>>>& pieces) {
        for (auto& kv : pcs) {
            Face& F = X.faces[kv.first];
            for (auto& sp : kv.second) {
                for (int end = 0; end < 2; end++) {
                    int be = sp.end_edge[end];
                    if (be < 0) continue;
                    double bt = sp.end_t[end];
                    int pvtx = -1;
                    auto it = vtx_of.find({be, bt});
                    if (it != vtx_of.end()) {
                        pvtx = it->second;
                    } else {
                        // the pave dedupe (3D) may have kept a TWIN t of the
                        // same physical point; the exact-double lookup then
                        // misses and the endpoint stays unsnapped (period
                        // seams: unmerged arrangement nodes). Fall back to the
                        // nearest pave on this edge in 3D.
                        const Edge& be_e = X.edges[be];
                        V3 q3 = sp.end_vtx[end] >= 0
                                    ? weld.verts[sp.end_vtx[end]].p
                                    : be_e.c.eval(bt);
                        double bestd3 = std::max(tol * 100, 0.01);
                        for (auto& kv2 : vtx_of) {
                            if (kv2.first.first != be) continue;
                            double d = be_e.c.eval(kv2.first.second).dist(q3);
                            if (d < bestd3) {
                                bestd3 = d;
                                pvtx = kv2.second;
                                bt = kv2.first.second;
                            }
                        }
                    }
                    if (pvtx < 0) {
                        if (dbg)
                            std::fprintf(stderr,
                                         "[snap] MISS sec=%d end=%d be=%d bt=%.9f\n",
                                         sp.sec_edge - SEC_BASE, end, be, bt);
                        continue;
                    }
                    if (dbg)
                        std::fprintf(stderr,
                                     "[snap] sec=%d end=%d be=%d bt=%.9f pvtx=%d\n",
                                     sp.sec_edge - SEC_BASE, end, be, bt, pvtx);
                    sp.end_vtx[end] = pvtx;
                    // junction UV on this face at (be, bt): lerp the point on
                    // the coedge piece chain containing bt. The search must
                    // cover ALL split pieces of the parent edge (originally it
                    // only saw the first piece, which still carries the parent
                    // id) — a pave on a later piece stayed unsnapped and left
                    // unmerged arrangement nodes on periodic seams.
                    UV2 cur = end == 0 ? sp.pc.front() : sp.pc.back();
                    double bestd = 1e300;
                    UV2 best_uv = cur;
                    auto lerp_in = [&](int edge_id) {
                        for (const Loop& l : F.loops)
                            for (int cei : l.ces) {
                                const CoEdge& ce = X.coedges[cei];
                                if (ce.edge != edge_id || ce.pt.size() < 2) continue;
                                double lo = std::min(ce.pt.front(), ce.pt.back());
                                double hi = std::max(ce.pt.front(), ce.pt.back());
                                if (bt < lo - 1e-9 || bt > hi + 1e-9) continue;
                                for (size_t i = 0; i + 1 < ce.pt.size(); i++) {
                                    double ta = ce.pt[i], tb = ce.pt[i + 1];
                                    if ((bt >= ta && bt <= tb) || (bt >= tb && bt <= ta)) {
                                        double fr = std::abs(tb - ta) > 1e-300
                                                        ? (bt - ta) / (tb - ta)
                                                        : 0;
                                        fr = std::max(0.0, std::min(1.0, fr));
                                        UV2 juv{ce.pc[i].u + (ce.pc[i + 1].u - ce.pc[i].u) * fr,
                                                ce.pc[i].v + (ce.pc[i + 1].v - ce.pc[i].v) * fr};
                                        double d = (juv.u - cur.u) * (juv.u - cur.u) +
                                                   (juv.v - cur.v) * (juv.v - cur.v);
                                        if (d < bestd) { bestd = d; best_uv = juv; }
                                    }
                                }
                            }
                    };
                    lerp_in(be);
                    auto itp = pieces.find(be);
                    if (itp != pieces.end())
                        for (auto& pc3 : itp->second)
                            if ((int)pc3[2] != be) lerp_in((int)pc3[2]);
                    if (end == 0) sp.pc.front() = best_uv;
                    else sp.pc.back() = best_uv;
                }
            }
        }
    };
    snap_endpoints(A, pcs_A, vtx_A, pieces_A);
    snap_endpoints(B, pcs_B, vtx_B, pieces_B);

    // close SecPC polylines of closed sections exactly (append the first point)
    auto close_pcs = [&](std::map<int, std::vector<SecPC>>& pcs) {
        for (auto& kv : pcs)
            for (auto& sp : kv.second)
                if (sp.closed && sp.pc.size() > 2 && sp.end_edge[0] < 0 &&
                    sp.end_edge[1] < 0) {
                    UV2 d{sp.pc.front().u - sp.pc.back().u,
                          sp.pc.front().v - sp.pc.back().v};
                    if (d.u * d.u + d.v * d.v > 0)
                        sp.pc.push_back(sp.pc.front());
                }
    };
    if (dbg)
        for (auto& kv : pcs_B)
            for (auto& sp : kv.second)
                std::fprintf(stderr,
                             "[presplit] B face %d sec first=(%.6f,%.4f) last=(%.6f,%.4f) n=%zu ee0=%d ee1=%d\n",
                             kv.first, sp.pc.front().u, sp.pc.front().v,
                             sp.pc.back().u, sp.pc.back().v, sp.pc.size(),
                             sp.end_edge[0], sp.end_edge[1]);
    close_pcs(pcs_A);
    close_pcs(pcs_B);

    // ---- split faces ------------------------------------------------------------
    std::vector<Face> parts_A, parts_B;
    std::vector<int> src_A, src_B; // part -> operand face
    for (size_t ai = 0; ai < A.faces.size(); ai++) {
        auto it = pcs_A.find((int)ai);
        std::vector<SecPC> empty;
        auto parts = split_face(A, A.faces[ai],
                                it != pcs_A.end() ? it->second : empty, tol);
        for (auto& p : parts) {
            src_A.push_back((int)ai);
            parts_A.push_back(std::move(p));
        }
    }
    for (size_t bi = 0; bi < B.faces.size(); bi++) {
        auto it = pcs_B.find((int)bi);
        std::vector<SecPC> empty;
        if (dbg && it != pcs_B.end())
            for (auto& sp : it->second)
                std::fprintf(stderr,
                             "[prespiltB] face %zu sec first=(%.6f,%.4f) last=(%.6f,%.4f)\n",
                             bi, sp.pc.front().u, sp.pc.front().v, sp.pc.back().u,
                             sp.pc.back().v);
        auto parts = split_face(B, B.faces[bi],
                                it != pcs_B.end() ? it->second : empty, tol);
        for (auto& p : parts) {
            src_B.push_back((int)bi);
            parts_B.push_back(std::move(p));
        }
    }

    // ---- classify parts ----------------------------------------------------------
    // interior point per part: grid search in the part's own loops.
    // Dual-probe classification: a part test point can land exactly on the
    // OTHER solid's boundary plane (coplanar regions, grazing contacts) where
    // single-point ray casting is degenerate. Probe both sides along the
    // part's outward normal: both-IN => IN, both-OUT => OUT, differing => the
    // part lies ON the other boundary. cm (the probe on the part's MATERIAL
    // side) disambiguates same-orientation vs opposite-orientation ON parts
    // for the op table.
    struct PartCls {
        PtCls c, cp, cm;
    };
    auto classify_part = [&](Solid& X, const Face& part, Solid& Y) {
        double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
        for (const Loop& l : part.loops)
            for (int cei : l.ces)
                for (auto& q : X.coedges[cei].pc) {
                    u0 = std::min(u0, q.u); u1 = std::max(u1, q.u);
                    v0 = std::min(v0, q.v); v1 = std::max(v1, q.v);
                }
        // probe the interior grid point with MAX clearance from the part
        // boundary: a probe near a boundary chord can sit within eps of the
        // other solid's surface and flip the dual probe to a false ON
        // (box_box_rot corner triangles)
        double best_clr = -1, bu = 0, bv = 0;
        for (int i = 1; i < 10; i++)
            for (int j = 1; j < 10; j++) {
                double u = u0 + (u1 - u0) * i / 10.0, v = v0 + (v1 - v0) * j / 10.0;
                if (!uv_in_face(X, part, u, v)) continue;
                double clr = 1e300;
                for (const Loop& l : part.loops)
                    for (int cei : l.ces) {
                        const auto& pc = X.coedges[cei].pc;
                        for (size_t k = 0; k + 1 < pc.size(); k++) {
                            double dx = pc[k + 1].u - pc[k].u,
                                   dy = pc[k + 1].v - pc[k].v;
                            double L2 = dx * dx + dy * dy;
                            double t = L2 > 1e-300
                                           ? ((u - pc[k].u) * dx + (v - pc[k].v) * dy) / L2
                                           : 0;
                            t = std::max(0.0, std::min(1.0, t));
                            double ex = pc[k].u + t * dx - u,
                                   ey = pc[k].v + t * dy - v;
                            clr = std::min(clr, ex * ex + ey * ey);
                        }
                    }
                if (clr > best_clr) { best_clr = clr; bu = u; bv = v; }
            }
        if (best_clr < 0)
            return PartCls{PtCls::ON, PtCls::ON, PtCls::ON};
        V3 p = X.srfs[part.srf].eval(bu, bv);
        V3 n = face_outward_normal(X, part, bu, bv);
        double eps = tol * 100;
        PtCls cp = classify_point(Y, p + n * eps, tol * 10);
        PtCls cm = classify_point(Y, p - n * eps, tol * 10);
        PtCls c = (cp == cm) ? cp : PtCls::ON;
        if (std::getenv("V3CLSDBG"))
            std::fprintf(stderr,
                         "[clspt] part p=(%.4f,%.4f,%.4f) +:%d -:%d -> %d\n",
                         p.x, p.y, p.z, (int)cp, (int)cm, (int)c);
        return PartCls{c, cp, cm};
    };
    std::vector<PartCls> cls_A, cls_B;
    for (auto& p : parts_A) cls_A.push_back(classify_part(A, p, B));
    for (auto& p : parts_B) cls_B.push_back(classify_part(B, p, A));
    if (dbg) {
        auto uv_rng = [&](Solid& X, const Face& p, double& u0, double& u1, double& v0,
                          double& v1) {
            u0 = v0 = 1e300; u1 = v1 = -1e300;
            for (const Loop& l : p.loops)
                for (int cei : l.ces)
                    for (auto& q : X.coedges[cei].pc) {
                        u0 = std::min(u0, q.u); u1 = std::max(u1, q.u);
                        v0 = std::min(v0, q.v); v1 = std::max(v1, q.v);
                    }
        };
        for (size_t i = 0; i < parts_A.size(); i++) {
            double u0, u1, v0, v1;
            uv_rng(A, parts_A[i], u0, u1, v0, v1);
            std::fprintf(stderr, "[cls] A part %zu (src %d): cls=%d loops=%zu uv=[%.2f,%.2f]x[%.2f,%.2f]\n",
                         i, src_A[i], (int)cls_A[i].c, parts_A[i].loops.size(), u0, u1, v0, v1);
        }
        for (size_t i = 0; i < parts_B.size(); i++) {
            double u0, u1, v0, v1;
            uv_rng(B, parts_B[i], u0, u1, v0, v1);
            std::fprintf(stderr, "[cls] B part %zu (src %d): cls=%d loops=%zu uv=[%.2f,%.2f]x[%.2f,%.2f]\n",
                         i, src_B[i], (int)cls_B[i].c, parts_B[i].loops.size(), u0, u1, v0, v1);
        }
    }

    // ---- select + assemble ---------------------------------------------------------
    Solid R;
    R.srfs = A.srfs;
    int b_srf_off = (int)A.srfs.size();
    for (auto& s : B.srfs) R.srfs.push_back(s);

    Weld rw{tol * 10};
    std::map<long long, int> edge_map; // (operand+1, edge) -> result edge
    auto get_edge = [&](int operand, int oe) -> int {
        long long k = (long long)(operand + 1) * 1000000000LL + oe;
        auto it = edge_map.find(k);
        if (it != edge_map.end()) return it->second;
        const Edge& e = operand == 0 ? A.edges[oe] : B.edges[oe];
        Edge ne = e;
        ne.v0 = rw.at((operand == 0 ? A : B).verts[e.v0].p);
        ne.v1 = rw.at((operand == 0 ? A : B).verts[e.v1].p);
        int id = (int)R.edges.size();
        R.edges.push_back(ne);
        edge_map[k] = id;
        return id;
    };
    std::map<int, int> sec_map; // section edge id (pool) -> result edge
    auto sec_edge_id = [&](int se) -> int {
        auto it = sec_map.find(se);
        if (it != sec_map.end()) return it->second;
        Edge ne = sec_edges[se];
        ne.v0 = rw.at(weld.verts[sec_edges[se].v0].p);
        ne.v1 = rw.at(weld.verts[sec_edges[se].v1].p);
        int id = (int)R.edges.size();
        R.edges.push_back(ne);
        sec_map[se] = id;
        return id;
    };

    auto remap_and_add = [&](Solid& X, const Face& part, int operand, bool reversed) {
        int face_id = (int)R.faces.size();
        Face built;
        built.srf = part.srf + (operand == 1 ? b_srf_off : 0);
        built.rev = part.rev;
        built.src = part.src;
        for (size_t li = 0; li < part.loops.size(); li++) {
            Loop nl;
            nl.outer = part.loops[li].outer;
            for (int cei : part.loops[li].ces) {
                const CoEdge& ce = X.coedges[cei];
                int re = ce.edge >= SEC_BASE ? sec_edge_id(ce.edge - SEC_BASE)
                                             : get_edge(operand, ce.edge);
                CoEdge nce = ce;
                nce.edge = re;
                nce.face = face_id;
                nl.ces.push_back((int)R.coedges.size());
                R.coedges.push_back(nce);
            }
            built.loops.push_back(nl);
        }
        R.faces.push_back(built);
        if (reversed) reverse_face(R, R.faces.back());
    };

    // Op table with same-domain (ON) disambiguation: for an A-ON part, cm is
    // the classification of A's MATERIAL side vs B. CUT keeps the part only
    // when its material survives (cm==OUT -- else it borders only the
    // cavity); COMMON keeps it only when the material is shared (cm==IN).
    auto keep = [&](const PartCls& pc, bool isA) {
        PtCls c = pc.c;
        switch (op) {
        case BoolOp::FUSE: return c == PtCls::OUT || (isA && c == PtCls::ON);
        case BoolOp::COMMON:
            return c == PtCls::IN ||
                   (isA && c == PtCls::ON && pc.cm == PtCls::IN);
        case BoolOp::CUT:
            return isA ? (c == PtCls::OUT ||
                          (c == PtCls::ON && pc.cm == PtCls::OUT))
                       : (c == PtCls::IN);
        }
        return false;
    };
    for (size_t i = 0; i < parts_A.size(); i++)
        if (keep(cls_A[i], true)) remap_and_add(A, parts_A[i], 0, false);
    for (size_t i = 0; i < parts_B.size(); i++)
        if (keep(cls_B[i], false))
            remap_and_add(B, parts_B[i], 1, op == BoolOp::CUT);

    // glue seam-split coedges: a section arc crossing a face seam is cut at
    // the band cut, so ONE face's loop can hold two coedges of the same edge
    // that meet end-to-start at the seam point. Together with the other
    // operand's coedge that makes 3 trims on one edge (false nonmanifold).
    // Merge cyclically-adjacent same-edge coedges (period-continuous pcurves).
    for (auto& f : R.faces)
        for (auto& l : f.loops) {
            bool merged = true;
            while (merged && l.ces.size() >= 2) {
                merged = false;
                size_t n = l.ces.size();
                for (size_t i = 0; i < n && !merged; i++) {
                    size_t j = (i + 1) % n;
                    CoEdge& c1 = R.coedges[l.ces[i]];
                    CoEdge& c2 = R.coedges[l.ces[j]];
                    if (c1.edge != c2.edge || c1.fwd != c2.fwd) continue;
                    if (c1.pc.size() < 2 || c2.pc.size() < 2) continue;
                    const Srf& S = R.srfs[f.srf];
                    UV2 e1 = c1.pc.back(), s2 = c2.pc.front();
                    double sh = S.periodic_u()
                                    ? std::round((e1.u - s2.u) / TWO_PI) * TWO_PI
                                    : 0;
                    double du = e1.u - (s2.u + sh), dv = e1.v - s2.v;
                    if (du * du + dv * dv > 1e-12) continue;
                    for (auto& q : c2.pc) q.u += sh;
                    c1.pc.insert(c1.pc.end(), c2.pc.begin() + 1, c2.pc.end());
                    l.ces.erase(l.ces.begin() + j);
                    merged = true;
                }
            }
        }
    // compact the coedge pool (merged-away slots must not count in is_closed)
    {
        std::vector<int> remap(R.coedges.size(), -1);
        std::vector<CoEdge> kept;
        for (auto& f : R.faces)
            for (auto& l : f.loops)
                for (int& cei : l.ces) {
                    if (remap[cei] < 0) {
                        remap[cei] = (int)kept.size();
                        kept.push_back(R.coedges[cei]);
                    }
                    cei = remap[cei];
                }
        R.coedges = std::move(kept);
    }

    // T-junction split of over-used edges: a section arc crossing a face seam
    // is cut there on that face, while the other operand's face uses the full
    // run -- 3+ trims on one edge (false nonmanifold). Split every coedge of
    // such an edge at each 3D endpoint of its sibling coedges that lands
    // interior to its pc, then regroup coedges by coincident 3D endpoints so
    // each physical sub-edge carries exactly its own trims.
    for (int guard = 0; guard < 16; guard++) {
        std::map<int, int> use;
        for (auto& ce : R.coedges) use[ce.edge]++;
        int bad = -1;
        for (auto& kv : use)
            if (kv.second > 2 && !R.edges[kv.first].degenerate) {
                bad = kv.first;
                break;
            }
        if (bad < 0) break;
        if (dbg) {
            std::fprintf(stderr, "[tj] guard=%d bad edge=%d use>2\n", guard,
                         bad);
        }
        // 3D endpoints of every coedge of this edge (on their own faces) —
        // even full-range coedges: a closed loop's arbitrary start point is
        // a genuine junction whenever a sibling side is cut there. Duplicates
        // and curve-off values are cleaned below.
        // Gate scale: junction 3D points come from chart lerps (~1e-3 off the
        // section polyline on unit geometry). Same philosophy as attach's
        // OCCT-style vertex-tolerance enlargement.
        double gate = std::max(tol * 100, 0.01);
        std::vector<V3> spts;
        for (auto& f : R.faces) {
            const Srf& S = R.srfs[f.srf];
            for (auto& l : f.loops)
                for (int cei : l.ces) {
                    CoEdge& ce = R.coedges[cei];
                    if (ce.edge != bad || ce.pc.size() < 2) continue;
                    spts.push_back(S.eval(ce.pc.front().u, ce.pc.front().v));
                    spts.push_back(S.eval(ce.pc.back().u, ce.pc.back().v));
                }
        }
        // dedupe: the same junction appearing in several spts entries would
        // be cut twice, producing zero-length sub-coedges
        {
            std::vector<V3> ded;
            for (auto& p : spts) {
                bool seen = false;
                for (auto& q : ded)
                    if (p.dist(q) < gate) { seen = true; break; }
                if (!seen) ded.push_back(p);
            }
            spts = std::move(ded);
        }
        // refine each split point onto the shared 3D curve (UNclamped — coedge
        // endpoints of sibling pieces legitimately lie outside the parent
        // edge's current t-range): junction UVs from frame-chart lerps can
        // sit ~1e-2 off the section curve and the cut gates would miss them
        {
            const Edge& be = R.edges[bad];
            for (auto& p : spts) {
                double t;
                if (be.c.project(p, t)) p = be.c.eval(t);
            }
        }
        if (dbg) {
            std::fprintf(stderr, "[tj]  spts:");
            for (auto& p : spts)
                std::fprintf(stderr, " (%.4f,%.4f,%.4f)", p.x, p.y, p.z);
            std::fprintf(stderr, "\n");
        }
        // split coedges at the split points: project each onto the polyline
        // (it lies on the same section curve, usually mid-segment), insert it
        // as a sample, and cut there (keeps loop walk contiguous)
        bool any_split = false;
        for (auto& f : R.faces) {
            const Srf& S = R.srfs[f.srf];
            for (auto& l : f.loops) {
                for (size_t ci = 0; ci < l.ces.size(); ci++) {
                    CoEdge& ce = R.coedges[l.ces[ci]];
                    if (ce.edge != bad || ce.pc.size() < 2) continue;
                    // projection position (sample + fraction) per split point
                    std::vector<std::pair<double, int>> cuts; // (pos, spts idx)
                    V3 cep0 = S.eval(ce.pc.front().u, ce.pc.front().v);
                    V3 cep1 = S.eval(ce.pc.back().u, ce.pc.back().v);
                    for (size_t si = 0; si < spts.size(); si++) {
                        // a junction at this coedge's own endpoint is no cut
                        // (the projection would land a hair inside and shave
                        // an end stub)
                        if (spts[si].dist(cep0) < gate ||
                            spts[si].dist(cep1) < gate)
                            continue;
                        double best = gate, bpos = -1;
                        for (size_t k = 0; k + 1 < ce.pc.size(); k++) {
                            V3 a = S.eval(ce.pc[k].u, ce.pc[k].v);
                            V3 b = S.eval(ce.pc[k + 1].u, ce.pc[k + 1].v);
                            V3 ab{b.x - a.x, b.y - a.y, b.z - a.z};
                            double L2 = ab.norm2();
                            if (L2 < 1e-300) continue;
                            V3 ap{spts[si].x - a.x, spts[si].y - a.y,
                                  spts[si].z - a.z};
                            // NB: no interior-endpoint rejection — a junction
                            // from a sibling arc can coincide with a polyline
                            // SAMPLE (shared section samples)
                            double fr = ap.dot(ab) / L2;
                            fr = std::max(0.0, std::min(1.0, fr));
                            V3 q{a.x + ab.x * fr, a.y + ab.y * fr,
                                 a.z + ab.z * fr};
                            double d = q.dist(spts[si]);
                            if (d < best) { best = d; bpos = k + fr; }
                        }
                        // only the polyline's global ENDS are no-cut positions
                        // (an endpoint-coincident junction that the polyline
                        // passes interiorly — a loop start — must still cut)
                        if (bpos >= 0 && bpos > 1e-9 &&
                            bpos < (double)ce.pc.size() - 1.0 - 1e-9)
                            cuts.push_back({bpos, (int)si});
                    }
                    if (cuts.empty()) continue;
                    if (dbg) {
                        std::fprintf(stderr, "[tj] ce pcsz=%zu cuts=",
                                     ce.pc.size());
                        for (auto& ct : cuts)
                            std::fprintf(stderr, " %.1f", ct.first);
                        std::fprintf(stderr, "\n");
                    }
                    std::sort(cuts.begin(), cuts.end());
                    cuts.erase(std::unique(cuts.begin(), cuts.end(),
                                           [](const auto& a, const auto& b) {
                                               return std::abs(a.first - b.first) < 1e-9;
                                           }),
                               cuts.end());
                    // rebuild the polyline with the cut points inserted
                    std::vector<UV2> npc;
                    std::vector<int> cut_idx; // npc indices that are cuts
                    size_t k = 0;
                    for (auto& ct : cuts) {
                        size_t ks = (size_t)ct.first;
                        double fr = ct.first - ks;
                        while (k <= ks) npc.push_back(ce.pc[k++]);
                        UV2 a = ce.pc[ks], b = ce.pc[ks + 1];
                        npc.push_back({a.u + (b.u - a.u) * fr,
                                       a.v + (b.v - a.v) * fr});
                        cut_idx.push_back((int)npc.size() - 1);
                    }
                    while (k < ce.pc.size()) npc.push_back(ce.pc[k++]);
                    // emit sub-coedges between consecutive cuts
                    std::vector<CoEdge> subs;
                    int prev = 0;
                    for (int idx : cut_idx) {
                        CoEdge sub = ce;
                        sub.pc.assign(npc.begin() + prev,
                                      npc.begin() + idx + 1);
                        if (sub.pc.size() > 1) subs.push_back(sub);
                        prev = idx;
                    }
                    CoEdge sub = ce;
                    sub.pc.assign(npc.begin() + prev, npc.end());
                    if (sub.pc.size() > 1) subs.push_back(sub);
                    if (subs.size() < 2) continue;
                    ce = subs[0];
                    for (size_t s = 1; s < subs.size(); s++) {
                        l.ces.insert(l.ces.begin() + ci + s,
                                     (int)R.coedges.size());
                        R.coedges.push_back(subs[s]);
                    }
                    ci += subs.size() - 1;
                    any_split = true;
                }
            }
        }
        if (!any_split) break; // genuine nonmanifold or duplicate ranges
        // regroup coedges by coincident 3D endpoints: one edge per sub-range.
        // Endpoint pairs alone do NOT identify a sub-range: two different
        // paths between the same junctions (the arcs above/below a lens) share
        // the endpoint pair. Disambiguate by the 3D path midpoint.
        struct Grp {
            int w0, w1;
            V3 mid;
            int edge;
        };
        std::vector<Grp> grp;
        double mid_gate = std::max(gate, tol * 1000 + 0.01);
        for (auto& f : R.faces) {
            const Srf& S = R.srfs[f.srf];
            for (auto& l : f.loops)
                for (int cei : l.ces) {
                    CoEdge& ce = R.coedges[cei];
                    if (ce.edge != bad || ce.pc.empty()) continue;
                    V3 p0 = S.eval(ce.pc.front().u, ce.pc.front().v);
                    V3 p1 = S.eval(ce.pc.back().u, ce.pc.back().v);
                    // snap endpoints to the split points: inserted lerp
                    // points and exact band-cut points must weld identically
                    for (auto& sp3 : spts) {
                        if (p0.dist(sp3) < gate) p0 = sp3;
                        if (p1.dist(sp3) < gate) p1 = sp3;
                    }
                    V3 pm = S.eval(ce.pc[ce.pc.size() / 2].u,
                                   ce.pc[ce.pc.size() / 2].v);
                    int w0 = rw.at(p0), w1 = rw.at(p1);
                    int lo = std::min(w0, w1), hi = std::max(w0, w1);
                    int ne_id = -1;
                    for (auto& g : grp)
                        if (g.w0 == lo && g.w1 == hi &&
                            g.mid.dist(pm) < mid_gate) {
                            ne_id = g.edge;
                            break;
                        }
                    if (dbg)
                        std::fprintf(stderr,
                                     "[tj]   grp ce edge=%d w=(%d,%d) p0=(%.4f,%.4f,%.4f) p1=(%.4f,%.4f,%.4f) mid=(%.4f,%.4f,%.4f) -> %d\n",
                                     ce.edge, lo, hi, p0.x, p0.y, p0.z, p1.x,
                                     p1.y, p1.z, pm.x, pm.y, pm.z, ne_id);
                    if (ne_id < 0) {
                        double t0, t1;
                        Edge ne = R.edges[bad];
                        if (ne.c.project(p0, t0) && ne.c.project(p1, t1)) {
                            // closed curves: a piece may wrap the period
                            // origin — a blind ascending swap would emit the
                            // COMPLEMENTARY arc. Chain-unwrap the end through
                            // the path midpoint instead.
                            double per = 0;
                            if (ne.c.k == Cur::CIRCLE || ne.c.k == Cur::ELLIPSE)
                                per = TWO_PI;
                            else if (ne.c.k == Cur::POLY && ne.c.poly_closed)
                                per = (double)ne.c.poly.size();
                            if (per > 0) {
                                double tm;
                                if (ne.c.project(pm, tm)) {
                                    auto wrapto = [&](double x, double ref) {
                                        double w = std::fmod(x - ref, per);
                                        return w < 0 ? w + per : w;
                                    };
                                    double w1 = wrapto(tm, t0);
                                    t1 = t0 + w1 + wrapto(t1, t0 + w1);
                                } else if (t0 > t1)
                                    std::swap(t0, t1);
                            } else if (t0 > t1)
                                std::swap(t0, t1);
                            ne.t0 = t0;
                            ne.t1 = t1;
                            ne.v0 = w0;
                            ne.v1 = w1;
                            ne_id = (int)R.edges.size();
                            R.edges.push_back(ne);
                        } else {
                            ne_id = bad; // projection failed: keep the parent
                        }
                        grp.push_back({lo, hi, pm, ne_id});
                    }
                    ce.edge = ne_id;
                }
        }
    }
    // drop point-sliver coedges (junction chatter at a shared section start:
    // zero 3D extent, endpoints weld to one vertex — loops stay chained
    // without them and their 1-use edge would read naked)
    for (auto& f : R.faces) {
        const Srf& S = R.srfs[f.srf];
        for (auto& l : f.loops) {
            for (size_t ci = 0; ci < l.ces.size();) {
                const CoEdge& ce = R.coedges[l.ces[ci]];
                const Edge& e = R.edges[ce.edge];
                bool sliver = false;
                if (!e.degenerate && ce.pc.size() >= 2) {
                    V3 p0 = S.eval(ce.pc.front().u, ce.pc.front().v);
                    double ext = 0;
                    for (auto& q : ce.pc)
                        ext = std::max(ext, p0.dist(S.eval(q.u, q.v)));
                    sliver = ext < tol * 100;
                }
                if (sliver) l.ces.erase(l.ces.begin() + ci);
                else ci++;
            }
        }
    }
    // coincident-edge unification (common blocks): same-domain configurations
    // produce a section edge that coincides with an operand edge (cone_cyl:
    // the r=0.8 section circle IS the cylinder base rim). Two result edges
    // for one physical curve each collect one trim -> naked. Merge
    // geometrically identical closed edges and redirect coedges to the
    // keeper. Conservative: only full-period circles today (t-ranges of
    // partial/offset curves would need reparameterization).
    for (size_t i = 0; i < R.edges.size(); i++) {
        const Edge& ei = R.edges[i];
        if (ei.degenerate || ei.c.k != Cur::CIRCLE) continue;
        if (ei.t1 - ei.t0 < TWO_PI - 1e-9) continue;
        for (size_t j = i + 1; j < R.edges.size(); j++) {
            const Edge& ej = R.edges[j];
            if (ej.degenerate || ej.c.k != Cur::CIRCLE) continue;
            if (ej.t1 - ej.t0 < TWO_PI - 1e-9) continue;
            if (std::abs(ei.c.r - ej.c.r) > tol * 100) continue;
            if (ei.c.f.o.dist(ej.c.f.o) > tol * 100) continue;
            if (std::abs(ei.c.f.z.dot(ej.c.f.z)) < 1.0 - 1e-9) continue;
            for (auto& ce : R.coedges)
                if (ce.edge == (int)j) ce.edge = (int)i;
        }
    }
    // compact edges (regrouping can orphan the pre-split edge records)
    {
        std::vector<int> eremap(R.edges.size(), -1);
        std::vector<Edge> kept;
        for (auto& ce : R.coedges) {
            if (eremap[ce.edge] < 0) {
                eremap[ce.edge] = (int)kept.size();
                kept.push_back(R.edges[ce.edge]);
            }
            ce.edge = eremap[ce.edge];
        }
        R.edges = std::move(kept);
    }

    // vertices: weld table -> R.verts; edges' v0/v1 already point into it
    R.verts = rw.verts;

    if (std::getenv("V3OUTDBG"))
        for (size_t fi = 0; fi < R.faces.size(); fi++) {
            std::fprintf(stderr, "[out] face %zu srf=%d rev=%d:", fi,
                         R.faces[fi].srf, (int)R.faces[fi].rev);
            for (auto& l : R.faces[fi].loops) {
                std::fprintf(stderr, " [%s", l.outer ? "o" : "h");
                for (int cei : l.ces) {
                    const CoEdge& ce = R.coedges[cei];
                    std::fprintf(stderr, " %d%s(%.2f>%.2f|%.2f>%.2f)",
                                 ce.edge, ce.fwd ? "+" : "-", ce.pc.front().u,
                                 ce.pc.back().u, ce.pc.front().v, ce.pc.back().v);
                }
                std::fprintf(stderr, " ]");
            }
            std::fprintf(stderr, "\n");
        }

    // final orientation safety + stats
    if (dbg) {
        int naked, nm;
        bool cl = R.is_closed(&naked, &nm);
        std::fprintf(stderr,
                     "[bool] op=%d parts A=%zu/%zu B=%zu/%zu -> faces=%zu closed=%d "
                     "(naked=%d nm=%d) vol=%.4f\n",
                     (int)op, parts_A.size(), A.faces.size(), parts_B.size(),
                     B.faces.size(), R.faces.size(), (int)cl, naked, nm,
                     R.faces.empty() ? 0.0 : R.signed_volume(24));
        if (naked > 0 || nm > 0 || R.faces.size() <= 12) {
            for (size_t fi = 0; fi < R.faces.size(); fi++) {
                double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
                for (const Loop& l : R.faces[fi].loops)
                    for (int cei : l.ces)
                        for (auto& q : R.coedges[cei].pc) {
                            u0 = std::min(u0, q.u); u1 = std::max(u1, q.u);
                            v0 = std::min(v0, q.v); v1 = std::max(v1, q.v);
                        }
                V3 c{0, 0, 0};
                int nn = 0;
                for (const Loop& l : R.faces[fi].loops)
                    for (int cei : l.ces) {
                        const CoEdge& ce = R.coedges[cei];
                        const Edge& e = R.edges[ce.edge];
                        if (e.degenerate) continue;
                        for (int i = 0; i <= 2; i++) {
                            c += e.c.eval(e.t0 + (e.t1 - e.t0) * i / 2.0);
                            nn++;
                        }
                    }
                if (nn) c = c / nn;
                std::fprintf(stderr,
                             "  [face %zu] srf=%d(%d) rev=%d loops=%zu uv=[%.2f,%.2f]x[%.2f,%.2f] c=(%.2f,%.2f,%.2f)\n",
                             fi, R.faces[fi].srf, (int)R.srfs[R.faces[fi].srf].k,
                             (int)R.faces[fi].rev, R.faces[fi].loops.size(), u0, u1, v0,
                             v1, c.x, c.y, c.z);
                for (const Loop& l : R.faces[fi].loops) {
                    std::fprintf(stderr, "    loop edges:");
                    for (int cei : l.ces) {
                        const CoEdge& ce = R.coedges[cei];
                        std::fprintf(stderr, " %d(%c)", R.coedges[cei].edge,
                                     R.coedges[cei].fwd ? '+' : '-');
                        if (std::getenv("V3BOOLDBG2"))
                            std::fprintf(stderr, "[%.2f..%.2f|%.2f..%.2f]",
                                         ce.pc.front().u, ce.pc.back().u,
                                         ce.pc.front().v, ce.pc.back().v);
                    }
                    std::fprintf(stderr, "\n");
                }
            }
        }
        if (naked > 0 || nm > 0) {
            std::map<int, int> use;
            for (auto& ce : R.coedges) use[ce.edge]++;
            for (auto& kv : use)
                if (kv.second != 2) {
                    const Edge& e = R.edges[kv.first];
                    std::fprintf(stderr,
                                 "  [naked] edge %d use=%d degen=%d t=[%.3f,%.3f] p0=(%.3f,%.3f,%.3f)\n",
                                 kv.first, kv.second, (int)e.degenerate, e.t0, e.t1,
                                 e.c.eval(e.t0).x, e.c.eval(e.t0).y, e.c.eval(e.t0).z);
                }
        }
    }
    return R;
}

session_cpp::BRep boolean(const session_cpp::BRep& A, const session_cpp::BRep& B,
                          BoolOp op, double tol, bool* intersected) {
    Solid sa = from_brep(A);
    Solid sb = from_brep(B);
    Solid r = boolean(sa, sb, op, tol, intersected);
    return to_brep(r);
}

} // namespace v3
