// Shared section scaffold for imported-freeform booleans (OCCT PaveFiller analog).
//
// OCCT's booleans never sew section edges after the fact: each face pair is intersected
// ONCE, the section curve is paved ONCE, and the resulting section edges (with pcurves
// on BOTH faces) bound BOTH operands' splits by construction (BOPAlgo_PaveFiller_6.cxx
// MakeBlocks / FaceInfo.Sc). build_section_scaffold is that stage for this kernel: it
// turns every surface-pair SSI into index-corresponded (3D, uvA, uvB) chains, paves
// them, applies one shared keep-verdict per interval, and welds pave vertices in 3D so
// sections continuing across adjacent cutter faces meet at one vertex.
#include "brep_section.h"
#include "intersection.h"
#include "closest.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "tolerance.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <tuple>

namespace session_cpp {

namespace {

struct SEval {
    const NurbsSurface* srf;
    void operator()(double u, double v, Vector& S, Vector& Su, Vector& Sv) const {
        auto d = srf->evaluate(u, v, 1);
        S = d[0]; Su = d[2]; Sv = d[1];
    }
};

// Min-norm 4-var Gauss-Newton on A(uA,vA) - B(uB,vB) = 0 (the marcher's unpinned
// corrector). Refines a chain sample so p3 lies on BOTH surfaces within conv_tol.
bool correct7(const SEval& ea, const SEval& eb, double& uA, double& vA, double& uB, double& vB,
              double conv_tol) {
    for (int it = 0; it < 8; ++it) {
        Vector Sa, Sau, Sav, Sb, Sbu, Sbv;
        ea(uA, vA, Sa, Sau, Sav);
        eb(uB, vB, Sb, Sbu, Sbv);
        double F[3] = {Sa[0]-Sb[0], Sa[1]-Sb[1], Sa[2]-Sb[2]};
        double fn = std::sqrt(F[0]*F[0] + F[1]*F[1] + F[2]*F[2]);
        if (fn < conv_tol) return true;
        double J[3][4];
        for (int k = 0; k < 3; ++k) { J[k][0] = Sau[k]; J[k][1] = Sav[k]; J[k][2] = -Sbu[k]; J[k][3] = -Sbv[k]; }
        // solve (J J^T) y = F, dx = J^T y
        double M[3][3], y[3];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) {
                double s = 0.0;
                for (int k = 0; k < 4; ++k) s += J[r][k] * J[c][k];
                M[r][c] = s;
            }
        // 3x3 solve (Cramer)
        double det = M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1])
                   - M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0])
                   + M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]);
        if (std::abs(det) < 1e-30) return false;
        auto solve_col = [&](int col) {
            double T[3][3];
            for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) T[r][c] = M[r][c];
            for (int r = 0; r < 3; ++r) T[r][col] = F[r];
            return (T[0][0]*(T[1][1]*T[2][2]-T[1][2]*T[2][1])
                  - T[0][1]*(T[1][0]*T[2][2]-T[1][2]*T[2][0])
                  + T[0][2]*(T[1][0]*T[2][1]-T[1][1]*T[2][0])) / det;
        };
        y[0] = solve_col(0); y[1] = solve_col(1); y[2] = solve_col(2);
        double dx[4];
        for (int k = 0; k < 4; ++k) dx[k] = J[0][k]*y[0] + J[1][k]*y[1] + J[2][k]*y[2];
        uA -= dx[0]; vA -= dx[1]; uB -= dx[2]; vB -= dx[3];
    }
    Vector Sa, Sau, Sav, Sb, Sbu, Sbv;
    ea(uA, vA, Sa, Sau, Sav);
    eb(uB, vB, Sb, Sbu, Sbv);
    double g = std::sqrt((Sa[0]-Sb[0])*(Sa[0]-Sb[0]) + (Sa[1]-Sb[1])*(Sa[1]-Sb[1]) + (Sa[2]-Sb[2])*(Sa[2]-Sb[2]));
    return g < conv_tol * 10.0;
}

bool seg_seg_2d(const Point& p1, const Point& p2, const Point& p3, const Point& p4,
                double& s_out, double& t_out) {
    double d1u = p2[0]-p1[0], d1v = p2[1]-p1[1];
    double d2u = p4[0]-p3[0], d2v = p4[1]-p3[1];
    double den = d1u*d2v - d1v*d2u;
    if (std::abs(den) < 1e-20) return false;
    double s = ((p3[0]-p1[0])*d2v - (p3[1]-p1[1])*d2u) / den;
    double t = ((p3[0]-p1[0])*d1v - (p3[1]-p1[1])*d1u) / den;
    if (-1e-12 <= s && s <= 1.0+1e-12 && -1e-12 <= t && t <= 1.0+1e-12) { s_out = s; t_out = t; return true; }
    return false;
}

// UV trim-loop polylines of every face on a given surface index: [face][loop][pts].
// Loop 0 is the outer loop; the rest are inner (holes).
std::vector<std::vector<std::vector<Point>>> face_loops_uv(const BRep& X, int si) {
    std::vector<std::vector<std::vector<Point>>> out;
    for (const auto& face : X.m_faces) {
        if (face.surface_index != si) continue;
        std::vector<std::vector<Point>> loops;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)X.m_loops.size()) continue;
            const BRepLoop& lp = X.m_loops[li];
            std::vector<Point> poly;
            for (int ti : lp.trim_indices) {
                if (ti < 0 || ti >= (int)X.m_trims.size()) continue;
                int c2 = X.m_trims[ti].curve_2d_index;
                if (c2 < 0 || c2 >= (int)X.m_curves_2d.size()) continue;
                const NurbsCurve& pc = X.m_curves_2d[c2];
                auto dc = pc.domain();
                int n = std::min(std::max(pc.cv_count() * 8, 32), 1024);
                for (int i = 0; i < n; ++i)   // skip last: next trim starts there
                    poly.push_back(pc.point_at(dc.first + (dc.second - dc.first) * i / n));
            }
            if (poly.size() >= 3) {
                // order inner-after-outer by BRepLoopType
                if (lp.type == BRepLoopType::Outer) loops.insert(loops.begin(), poly);
                else loops.push_back(poly);
            }
        }
        if (!loops.empty()) out.push_back(std::move(loops));
    }
    return out;
}

double dist_to_poly_uv(const std::vector<Point>& poly, double u, double v) {
    double best = 1e300;
    size_t n = poly.size();
    for (size_t j = 0; j < n; ++j) {
        const Point& a = poly[j];
        const Point& b = poly[(j + 1) % n];
        double ex = b[0]-a[0], ey = b[1]-a[1];
        double L2 = ex*ex + ey*ey;
        double t = L2 > 1e-30 ? ((u-a[0])*ex + (v-a[1])*ey) / L2 : 0.0;
        t = std::min(std::max(t, 0.0), 1.0);
        double dx = u-a[0]-t*ex, dy = v-a[1]-t*ey;
        best = std::min(best, dx*dx + dy*dy);
    }
    return std::sqrt(best);
}

bool point_in_poly_uv(const std::vector<Point>& poly, double u, double v) {
    bool inside = false;
    size_t n = poly.size();
    for (size_t j = 0; j < n; ++j) {
        const Point& a = poly[j];
        const Point& b = poly[(j + 1) % n];
        if ((a[1] > v) != (b[1] > v) && u < (b[0]-a[0])*(v-a[1])/(b[1]-a[1]) + a[0])
            inside = !inside;
    }
    return inside;
}

// Inside any face of the surface: inside outer, outside inners; within eps of any
// loop counts as inside (grazing-tolerant, per the r15 overshoot lesson).
bool in_faces_uv(const std::vector<std::vector<std::vector<Point>>>& faces, double u, double v, double eps) {
    for (const auto& loops : faces) {
        bool in_outer = point_in_poly_uv(loops[0], u, v) || dist_to_poly_uv(loops[0], u, v) < eps;
        if (!in_outer) continue;
        bool in_hole = false;
        for (size_t h = 1; h < loops.size() && !in_hole; ++h)
            in_hole = point_in_poly_uv(loops[h], u, v) && dist_to_poly_uv(loops[h], u, v) > eps;
        if (!in_hole) return true;
    }
    return false;
}

std::pair<std::array<double,3>, std::array<double,3>> srf_aabb(const NurbsSurface& s) {
    auto du = s.domain(0); auto dv = s.domain(1);
    std::array<double,3> mn = {1e300,1e300,1e300}, mx = {-1e300,-1e300,-1e300};
    for (int i = 0; i <= 8; ++i)
        for (int j = 0; j <= 8; ++j) {
            Point p = s.point_at(du.first + (du.second-du.first)*i/8.0, dv.first + (dv.second-dv.first)*j/8.0);
            for (int k = 0; k < 3; ++k) { mn[k] = std::min(mn[k], p[k]); mx[k] = std::max(mx[k], p[k]); }
        }
    return {mn, mx};
}

// A raw chain before paving: index-corresponded samples on one surface pair.
struct Chain {
    int surfA, surfB;
    std::vector<Point> p3, uvA, uvB;
    bool closed = false;
    std::vector<std::pair<int,double>> paves;  // (segment index i, frac in [0,1)) sorted
    int pave_kinds[5] = {0,0,0,0,0};           // trimA, trimB, xing, vertex, closing
};

Point lerp(const Point& a, const Point& b, double t) {
    return Point(a[0] + (b[0]-a[0])*t, a[1] + (b[1]-a[1])*t, a[2] + (b[2]-a[2])*t);
}

} // namespace

SectionScaffold build_section_scaffold(const BRep& A, const BRep& B, double tolerance) {
    SectionScaffold scaf;
    if (tolerance <= 0.0) tolerance = Tolerance::ZERO_TOLERANCE;

    // joint bbox diagonal -> weld tolerance (matches the trim-cut chaining scale)
    std::array<double,3> mn = {1e300,1e300,1e300}, mx = {-1e300,-1e300,-1e300};
    for (const auto& p : A.m_vertices) for (int k = 0; k < 3; ++k) { mn[k]=std::min(mn[k],p[k]); mx[k]=std::max(mx[k],p[k]); }
    for (const auto& p : B.m_vertices) for (int k = 0; k < 3; ++k) { mn[k]=std::min(mn[k],p[k]); mx[k]=std::max(mx[k],p[k]); }
    double diag = std::sqrt((mx[0]-mn[0])*(mx[0]-mn[0]) + (mx[1]-mn[1])*(mx[1]-mn[1]) + (mx[2]-mn[2])*(mx[2]-mn[2]));
    if (!(diag > 0.0)) diag = 1.0;
    double weld_tol = diag * 5e-4;
    double conv_tol = std::max(tolerance, diag * 1e-9);
    scaf.tol3 = weld_tol;
    scaf.segs_by_surfA.resize(A.m_surfaces.size());
    scaf.segs_by_surfB.resize(B.m_surfaces.size());

    // ---- 1. One fenced SSI per overlapping pair -> canonicalized chains ----
    std::vector<std::pair<std::array<double,3>, std::array<double,3>>> abbs, bbbs;
    for (const auto& s : A.m_surfaces) abbs.push_back(srf_aabb(s));
    for (const auto& s : B.m_surfaces) bbbs.push_back(srf_aabb(s));
    auto overlap = [&](int ai, int bi, double m) {
        return !(bbbs[bi].first[0] > abbs[ai].second[0]+m || bbbs[bi].second[0] < abbs[ai].first[0]-m ||
                 bbbs[bi].first[1] > abbs[ai].second[1]+m || bbbs[bi].second[1] < abbs[ai].first[1]-m ||
                 bbbs[bi].first[2] > abbs[ai].second[2]+m || bbbs[bi].second[2] < abbs[ai].first[2]-m);
    };

    std::vector<Chain> chains;
    for (int ai = 0; ai < (int)A.m_surfaces.size(); ++ai) {
        double margin = std::max({abbs[ai].second[0]-abbs[ai].first[0],
                                  abbs[ai].second[1]-abbs[ai].first[1],
                                  abbs[ai].second[2]-abbs[ai].first[2]}) * 1e-3;
        for (int bi = 0; bi < (int)B.m_surfaces.size(); ++bi) {
            if (!overlap(ai, bi, margin)) continue;
            const NurbsSurface& sa = A.m_surfaces[ai];
            const NurbsSurface& sb = B.m_surfaces[bi];
            auto trs = Intersection::surface_surface(sa, sb, tolerance);
            bool swapped = false;
            if (trs.empty()) {   // order-sensitive marcher: retry swapped (G1 fence kept)
                trs = Intersection::surface_surface(sb, sa, tolerance);
                swapped = true;
            }
            SEval ea{&sa}, eb{&sb};
            auto dua = sa.domain(0); auto dva = sa.domain(1);
            auto dub = sb.domain(0); auto dvb = sb.domain(1);
            for (auto& tr : trs) {
                // pcurve on A drives the sampling; swapped runs use get<2> for A
                const NurbsCurve& pca = swapped ? std::get<2>(tr) : std::get<1>(tr);
                const NurbsCurve& pcb = swapped ? std::get<1>(tr) : std::get<2>(tr);
                if (!pca.is_valid() || !pcb.is_valid()) continue;
                auto dca = pca.domain();
                auto dcb = pcb.domain();
                int n = std::min(std::max(pca.cv_count() * 4, 48), 1024);
                Chain ch;
                ch.surfA = ai; ch.surfB = bi;
                ch.p3.reserve(n + 1); ch.uvA.reserve(n + 1); ch.uvB.reserve(n + 1);
                for (int i = 0; i <= n; ++i) {
                    double ta = dca.first + (dca.second - dca.first) * i / n;
                    Point qa = pca.point_at(ta);
                    double uA = qa[0], vA = qa[1];
                    // seed uvB from B's own pcurve at the SAME normalized fraction --
                    // parameterizations differ (F1) but the fraction is a good local seed
                    Point qb = pcb.point_at(dcb.first + (dcb.second - dcb.first) * i / n);
                    double uB = qb[0], vB = qb[1];
                    if (!correct7(ea, eb, uA, vA, uB, vB, conv_tol)) {
                        // re-seed via global closest point, then correct once more
                        Vector S, Su, Sv; ea(uA, vA, S, Su, Sv);
                        auto [cu, cv, cd] = Closest::surface_point(sb, Point(S[0], S[1], S[2]));
                        uB = cu; vB = cv;
                        correct7(ea, eb, uA, vA, uB, vB, conv_tol);
                    }
                    // clamp open directions into domain
                    uA = std::min(std::max(uA, dua.first), dua.second);
                    vA = std::min(std::max(vA, dva.first), dva.second);
                    uB = std::min(std::max(uB, dub.first), dub.second);
                    vB = std::min(std::max(vB, dvb.first), dvb.second);
                    Vector S, Su, Sv; ea(uA, vA, S, Su, Sv);
                    ch.p3.push_back(Point(S[0], S[1], S[2]));
                    ch.uvA.push_back(Point(uA, vA, 0.0));
                    ch.uvB.push_back(Point(uB, vB, 0.0));
                }
                if ((int)ch.p3.size() < 2) continue;
                ch.closed = ch.p3.front().distance(ch.p3.back()) < weld_tol;
                chains.push_back(std::move(ch));
            }
        }
    }
    scaf.n_chains = (int)chains.size();

    // measure chain fidelity ([SCAF] gate)
    for (const auto& ch : chains) {
        const NurbsSurface& sa = A.m_surfaces[ch.surfA];
        const NurbsSurface& sb = B.m_surfaces[ch.surfB];
        for (size_t i = 0; i < ch.p3.size(); i += std::max<size_t>(1, ch.p3.size() / 16)) {
            scaf.max_devA = std::max(scaf.max_devA, sa.point_at(ch.uvA[i][0], ch.uvA[i][1]).distance(ch.p3[i]));
            scaf.max_devB = std::max(scaf.max_devB, sb.point_at(ch.uvB[i][0], ch.uvB[i][1]).distance(ch.p3[i]));
        }
    }

    // ---- 2. Paves ----
    // trim-loop UV polylines per used surface (faces of A on surfA / B on surfB)
    std::map<int, std::vector<std::vector<std::vector<Point>>>> loopsA, loopsB;
    for (const auto& ch : chains) {
        if (!loopsA.count(ch.surfA)) loopsA[ch.surfA] = face_loops_uv(A, ch.surfA);
        if (!loopsB.count(ch.surfB)) loopsB[ch.surfB] = face_loops_uv(B, ch.surfB);
    }
    auto add_pave = [](Chain& ch, int i, double f, int kind) {
        // dedup: skip if an existing pave is within half a sample of this one
        for (auto& pv : ch.paves)
            if (std::abs((pv.first + pv.second) - (i + f)) < 0.5) return false;
        ch.paves.push_back({i, f});
        ch.pave_kinds[kind] += 1;
        return true;
    };

    for (auto& ch : chains) {
        // (a) trim crossings on A and B
        for (int side = 0; side < 2; ++side) {
            const auto& uv = side == 0 ? ch.uvA : ch.uvB;
            const auto& faces = side == 0 ? loopsA[ch.surfA] : loopsB[ch.surfB];
            for (const auto& loops : faces)
                for (const auto& poly : loops) {
                    size_t np = poly.size();
                    for (size_t i = 0; i + 1 < uv.size(); ++i)
                        for (size_t j = 0; j < np; ++j) {
                            double s, t;
                            if (seg_seg_2d(uv[i], uv[i+1], poly[j], poly[(j+1)%np], s, t)) {
                                if (add_pave(ch, (int)i, s, side)) {
                                    if (side == 0) scaf.n_paves_trimA += 1; else scaf.n_paves_trimB += 1;
                                }
                            }
                        }
                }
        }
    }
    // (b) chain-chain crossings on shared surfaces (both inserted), UV-bbox prefiltered
    auto uv_bbox = [](const std::vector<Point>& uv) {
        std::array<double,4> b = {1e300, -1e300, 1e300, -1e300};
        for (const auto& q : uv) {
            b[0] = std::min(b[0], q[0]); b[1] = std::max(b[1], q[0]);
            b[2] = std::min(b[2], q[1]); b[3] = std::max(b[3], q[1]);
        }
        return b;
    };
    for (size_t c1 = 0; c1 < chains.size(); ++c1)
        for (size_t c2 = c1 + 1; c2 < chains.size(); ++c2) {
            for (int side = 0; side < 2; ++side) {
                bool share = side == 0 ? chains[c1].surfA == chains[c2].surfA
                                       : chains[c1].surfB == chains[c2].surfB;
                if (!share) continue;
                const auto& uv1 = side == 0 ? chains[c1].uvA : chains[c1].uvB;
                const auto& uv2 = side == 0 ? chains[c2].uvA : chains[c2].uvB;
                auto b1 = uv_bbox(uv1), b2 = uv_bbox(uv2);
                if (b1[0] > b2[1] || b1[1] < b2[0] || b1[2] > b2[3] || b1[3] < b2[2]) continue;
                for (size_t i = 0; i + 1 < uv1.size(); ++i)
                    for (size_t j = 0; j + 1 < uv2.size(); ++j) {
                        double s, t;
                        if (seg_seg_2d(uv1[i], uv1[i+1], uv2[j], uv2[j+1], s, t)) {
                            add_pave(chains[c1], (int)i, s, 2);
                            add_pave(chains[c2], (int)j, t, 2);
                            scaf.n_paves_xing += 1;
                        }
                    }
            }
        }
    // (c) existing operand vertices near a chain
    auto vertex_paves = [&](const BRep& X) {
        for (const auto& tv : X.m_topology_vertices) {
            if (tv.point_index < 0 || tv.point_index >= (int)X.m_vertices.size()) continue;
            const Point& q = X.m_vertices[tv.point_index];
            for (auto& ch : chains) {
                double best = 1e300; int bi2 = -1; double bf = 0.0;
                for (size_t i = 0; i + 1 < ch.p3.size(); ++i) {
                    const Point& a = ch.p3[i];
                    const Point& b = ch.p3[i+1];
                    double ex = b[0]-a[0], ey = b[1]-a[1], ez = b[2]-a[2];
                    double L2 = ex*ex + ey*ey + ez*ez;
                    double t = L2 > 1e-30 ? ((q[0]-a[0])*ex + (q[1]-a[1])*ey + (q[2]-a[2])*ez) / L2 : 0.0;
                    t = std::min(std::max(t, 0.0), 1.0);
                    Point c = lerp(a, b, t);
                    double d = q.distance(c);
                    if (d < best) { best = d; bi2 = (int)i; bf = t; }
                }
                if (bi2 >= 0 && best < weld_tol * 2.0) {
                    if (add_pave(ch, bi2, bf, 3)) scaf.n_paves_vertex += 1;
                }
            }
        }
    };
    vertex_paves(A);
    vertex_paves(B);
    // (d) closing pave for closed chains with no pave yet
    for (auto& ch : chains)
        if (ch.closed && ch.paves.empty()) {
            int imax = 0;
            for (size_t i = 1; i < ch.p3.size(); ++i)
                if (std::make_tuple(ch.p3[i][0], ch.p3[i][1], ch.p3[i][2]) >
                    std::make_tuple(ch.p3[imax][0], ch.p3[imax][1], ch.p3[imax][2])) imax = (int)i;
            add_pave(ch, imax, 0.0, 4);
            scaf.n_paves_closing += 1;
        }

    // ---- 3. Intervals -> shared verdict -> micro filter -> segments + welded vertices ----
    auto weld_vertex = [&](const Point& p) -> int {
        for (int vi = 0; vi < (int)scaf.vertices.size(); ++vi)
            if (scaf.vertices[vi].distance(p) < weld_tol) return vi;
        scaf.vertices.push_back(p);
        return (int)scaf.vertices.size() - 1;
    };

    for (auto& ch : chains) {
        std::sort(ch.paves.begin(), ch.paves.end(), [](const std::pair<int,double>& a, const std::pair<int,double>& b) {
            return a.first + a.second < b.first + b.second;
        });
        // breakpoints in "sample position" space: pos in [0, n-1]
        std::vector<double> bps;
        if (!ch.closed) bps.push_back(0.0);
        for (auto& pv : ch.paves) bps.push_back(pv.first + pv.second);
        if (!ch.closed) bps.push_back((double)ch.p3.size() - 1.0);
        if (ch.closed && !bps.empty()) bps.push_back(bps.front() + (double)ch.p3.size() - 1.0);  // wrap
        if ((int)bps.size() < 2) continue;

        const NurbsSurface& sa = A.m_surfaces[ch.surfA];
        const NurbsSurface& sb = B.m_surfaces[ch.surfB];
        auto duaR = sa.domain(0); auto dvaR = sa.domain(1);
        auto dubR = sb.domain(0); auto dvbR = sb.domain(1);
        double epsA = std::min(duaR.second - duaR.first, dvaR.second - dvaR.first) * 1e-3;
        double epsB = std::min(dubR.second - dubR.first, dvbR.second - dvbR.first) * 1e-3;
        int nS = (int)ch.p3.size();
        auto at = [&](double pos, Point& p3o, Point& uvAo, Point& uvBo) {
            double w = pos;
            while (w >= nS - 1) w -= (nS - 1);   // wrap for closed chains
            int i = (int)w;
            double f = w - i;
            p3o = lerp(ch.p3[i], ch.p3[std::min(i+1, nS-1)], f);
            uvAo = lerp(ch.uvA[i], ch.uvA[std::min(i+1, nS-1)], f);
            uvBo = lerp(ch.uvB[i], ch.uvB[std::min(i+1, nS-1)], f);
        };

        for (size_t k = 0; k + 1 < bps.size(); ++k) {
            double lo = bps[k], hi = bps[k+1];
            if (hi - lo < 1e-9) continue;
            // shared keep-verdict at the interval midpoint
            Point pm, am, bm;
            at((lo + hi) * 0.5, pm, am, bm);
            bool keep = in_faces_uv(loopsA[ch.surfA], am[0], am[1], epsA)
                     && in_faces_uv(loopsB[ch.surfB], bm[0], bm[1], epsB);
            if (!keep) { scaf.n_dropped_verdict += 1; continue; }
            // micro filter (FindValidRange analog): symmetric drop
            Point p0, a0, b0, p1, a1, b1;
            at(lo, p0, a0, b0);
            at(hi, p1, a1, b1);
            double ext = 0.0;
            {
                Point prev = p0;
                for (double w = std::ceil(lo); w < hi; w += 1.0) {
                    Point q, qa, qb;
                    at(w, q, qa, qb);
                    ext += prev.distance(q);
                    prev = q;
                }
                ext += prev.distance(p1);
            }
            if (ext < weld_tol) { scaf.n_dropped_micro += 1; continue; }

            SectionSegment seg;
            seg.seg_id = (int)scaf.segments.size();
            seg.surfA = ch.surfA; seg.surfB = ch.surfB;
            // Endpoints stay at the RAW chain-lerped pave position: a trim-crossing pave's
            // reason to exist is to lie ON the boundary polyline -- Newton-refining it onto
            // the exact section moves it ~sag (up to ~0.02 UV) OFF the boundary and the
            // arrangement then clips the cut elsewhere. The raw point is within chain sag
            // (~1e-3) of both surfaces, identical on both operands, absorbed by weld_tol.
            (void)conv_tol;
            seg.p3.push_back(p0);
            seg.uvA.push_back(a0);
            seg.uvB.push_back(b0);
            for (double w = std::ceil(lo + 1e-9); w < hi - 1e-9; w += 1.0) {
                Point q, qa, qb;
                at(w, q, qa, qb);
                seg.p3.push_back(q);
                seg.uvA.push_back(qa);
                seg.uvB.push_back(qb);
            }
            seg.p3.push_back(p1);
            seg.uvA.push_back(a1);
            seg.uvB.push_back(b1);
            seg.v_start = weld_vertex(seg.p3.front());
            seg.v_end = weld_vertex(seg.p3.back());
            seg.closed = seg.v_start == seg.v_end && seg.p3.size() > 3;
            scaf.segs_by_surfA[ch.surfA].push_back(seg.seg_id);
            scaf.segs_by_surfB[ch.surfB].push_back(seg.seg_id);
            scaf.segments.push_back(std::move(seg));
        }
    }
    return scaf;
}

} // namespace session_cpp
