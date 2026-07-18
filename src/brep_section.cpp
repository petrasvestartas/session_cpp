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

// correct7 with one param PINNED to a chart bound (OCCT SeekPointOnBoundary analog):
// 3 unknowns, 3 equations -- direct Newton on A(uA,vA) - B(uB,vB) = 0.
bool correct7_pinned(const SEval& ea, const SEval& eb, double& uA, double& vA, double& uB, double& vB,
                     int pin, double conv_tol) {
    double* vars[4] = {&uA, &vA, &uB, &vB};
    for (int it = 0; it < 12; ++it) {
        Vector Sa, Sau, Sav, Sb, Sbu, Sbv;
        ea(uA, vA, Sa, Sau, Sav);
        eb(uB, vB, Sb, Sbu, Sbv);
        double F[3] = {Sa[0]-Sb[0], Sa[1]-Sb[1], Sa[2]-Sb[2]};
        double fn = std::sqrt(F[0]*F[0] + F[1]*F[1] + F[2]*F[2]);
        if (fn < conv_tol) return true;
        double Jfull[3][4];
        for (int k = 0; k < 3; ++k) { Jfull[k][0] = Sau[k]; Jfull[k][1] = Sav[k]; Jfull[k][2] = -Sbu[k]; Jfull[k][3] = -Sbv[k]; }
        double J[3][3];
        for (int k = 0; k < 3; ++k) {
            int c = 0;
            for (int m = 0; m < 4; ++m) { if (m == pin) continue; J[k][c++] = Jfull[k][m]; }
        }
        double det = J[0][0]*(J[1][1]*J[2][2]-J[1][2]*J[2][1])
                   - J[0][1]*(J[1][0]*J[2][2]-J[1][2]*J[2][0])
                   + J[0][2]*(J[1][0]*J[2][1]-J[1][1]*J[2][0]);
        if (std::abs(det) < 1e-30) return false;
        auto col = [&](int c) {
            double T[3][3];
            for (int r = 0; r < 3; ++r) for (int c2 = 0; c2 < 3; ++c2) T[r][c2] = J[r][c2];
            for (int r = 0; r < 3; ++r) T[r][c] = F[r];
            return (T[0][0]*(T[1][1]*T[2][2]-T[1][2]*T[2][1])
                  - T[0][1]*(T[1][0]*T[2][2]-T[1][2]*T[2][0])
                  + T[0][2]*(T[1][0]*T[2][1]-T[1][1]*T[2][0])) / det;
        };
        double dx[3] = {col(0), col(1), col(2)};
        int c = 0;
        for (int m = 0; m < 4; ++m) { if (m == pin) continue; *vars[m] -= dx[c++]; }
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
                // Adaptive to chord sag: paves are seg-seg crossings against these polylines,
                // and downstream forced-node injection tolerates only ~1e-2 UV -- fixed-count
                // sampling of curvy imported trims sags 0.02-0.08 and displaces every pave.
                int n = std::min(std::max(pc.cv_count() * 4, 16), 256);
                std::vector<std::pair<double, Point>> ent;
                ent.reserve(n + 1);
                for (int i = 0; i <= n; ++i) {
                    double t = dc.first + (dc.second - dc.first) * i / n;
                    ent.push_back({t, pc.point_at(t)});
                }
                double bx0 = 1e300, bx1 = -1e300, by0 = 1e300, by1 = -1e300;
                for (auto& e2 : ent) {
                    bx0 = std::min(bx0, e2.second[0]); bx1 = std::max(bx1, e2.second[0]);
                    by0 = std::min(by0, e2.second[1]); by1 = std::max(by1, e2.second[1]);
                }
                double samp_tol_lp = std::max(1e-9, std::max(bx1 - bx0, by1 - by0) * 2e-4);
                {
                    // refine chords whose midpoint deviates (same criterion as the arrangement)
                    auto dev2 = [](const Point& a, const Point& b, const Point& m) {
                        double ex = b[0]-a[0], ey = b[1]-a[1];
                        double L2 = ex*ex + ey*ey;
                        double t = L2 > 1e-30 ? ((m[0]-a[0])*ex + (m[1]-a[1])*ey) / L2 : 0.0;
                        t = std::min(std::max(t, 0.0), 1.0);
                        double dx = m[0]-a[0]-t*ex, dy = m[1]-a[1]-t*ey;
                        return dx*dx + dy*dy;
                    };
                    for (int pass = 0; pass < 6; ++pass) {
                        int ins = 0;
                        for (size_t i = 0; i + 1 < ent.size() && ent.size() < 4096; ++i) {
                            double tm = (ent[i].first + ent[i+1].first) * 0.5;
                            Point pm = pc.point_at(tm);
                            if (dev2(ent[i].second, ent[i+1].second, pm) > samp_tol_lp * samp_tol_lp) {
                                ent.insert(ent.begin() + i + 1, {tm, pm});
                                ++ins; ++i;
                            }
                        }
                        if (!ins) break;
                    }
                }
                for (size_t i = 0; i + 1 < ent.size(); ++i)   // skip last: next trim starts there
                    poly.push_back(ent[i].second);
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
    // Newton-refined pave positions (section ∩ trim-curve, sag-free), keyed by pos i+f.
    // Without these a trim pave sits on a COARSE chain chord, displaced up to half a
    // chord (~0.1) along the section; the two charts' copies of one junction then land
    // ~0.1 apart and never weld (the si=19 triple-point class).
    std::map<double, std::array<Point,3>> pave_fix;   // pos -> {p3, uvA, uvB}
};

// OCCT IntPatch_CurvIntSurf analog: exact section ∩ trim point. Solve
// S_side(c(t)) = S_other(u,v) with c(t) = pa + t (pb - pa) on the trim polyline
// segment -- 3 unknowns (t,u,v), 3 equations; the pave stays ON the boundary
// polyline by construction while landing exactly on the section.
bool refine_trim_pave(const SEval& es, const SEval& eo, const Point& pa, const Point& pb,
                      double& t, double& uo, double& vo, double conv_tol) {
    double du = pb[0] - pa[0], dv = pb[1] - pa[1];
    for (int it = 0; it < 12; ++it) {
        double cu = pa[0] + du * t, cv2 = pa[1] + dv * t;
        Vector S, Su, Sv, O, Ou, Ov;
        es(cu, cv2, S, Su, Sv);
        eo(uo, vo, O, Ou, Ov);
        double F[3] = {S[0]-O[0], S[1]-O[1], S[2]-O[2]};
        double fn = std::sqrt(F[0]*F[0] + F[1]*F[1] + F[2]*F[2]);
        if (fn < conv_tol) return true;
        double J[3][3];
        for (int k = 0; k < 3; ++k) {
            J[k][0] = Su[k]*du + Sv[k]*dv;
            J[k][1] = -Ou[k];
            J[k][2] = -Ov[k];
        }
        double det = J[0][0]*(J[1][1]*J[2][2]-J[1][2]*J[2][1])
                   - J[0][1]*(J[1][0]*J[2][2]-J[1][2]*J[2][0])
                   + J[0][2]*(J[1][0]*J[2][1]-J[1][1]*J[2][0]);
        if (std::abs(det) < 1e-30) return false;
        auto col = [&](int c) {
            double T[3][3];
            for (int r = 0; r < 3; ++r) for (int c2 = 0; c2 < 3; ++c2) T[r][c2] = J[r][c2];
            for (int r = 0; r < 3; ++r) T[r][c] = F[r];
            return (T[0][0]*(T[1][1]*T[2][2]-T[1][2]*T[2][1])
                  - T[0][1]*(T[1][0]*T[2][2]-T[1][2]*T[2][0])
                  + T[0][2]*(T[1][0]*T[2][1]-T[1][1]*T[2][0])) / det;
        };
        t  -= col(0);
        uo -= col(1);
        vo -= col(2);
        if (t < -0.25) t = -0.25;
        if (t > 1.25) t = 1.25;
    }
    double cu = pa[0] + du * t, cv2 = pa[1] + dv * t;
    Vector S, Su, Sv, O, Ou, Ov;
    es(cu, cv2, S, Su, Sv);
    eo(uo, vo, O, Ou, Ov);
    double g = std::sqrt((S[0]-O[0])*(S[0]-O[0]) + (S[1]-O[1])*(S[1]-O[1]) + (S[2]-O[2])*(S[2]-O[2]));
    return g < conv_tol * 10.0;
}

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
    // Junction paves of one section arriving via ADJACENT chart pairs are computed as
    // chain-lerp crossings against different faces' loop polylines -- their 3D positions
    // disagree by chain+loop sag (~diag*1.5e-3 measured on the chairs). The weld must
    // absorb that or continuations never share a vertex and both cut ends get pruned.
    double weld_tol = diag * 2e-3;
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
    // ---- 1a2. Seam decomposition (OCCT DecompositionOfWLine analog) ----
    // A chain on a PERIODIC surface can wrap the seam: its UV polyline then jumps a
    // period mid-chain and every downstream 2D stage (paves, arrangement) sees garbage.
    // Split each chain at seam crossings, inserting the EXACT crossing sample pinned to
    // the seam on BOTH pieces (identical 3D point, own coord at the two seam bounds) --
    // the weld reconnects them across the seam. Chairs (non-periodic) no-op here.
    {
        auto split_seams = [&](std::vector<Chain>& in, int side) {
            std::vector<Chain> res;
            res.reserve(in.size());
            for (auto& ch : in) {
                const NurbsSurface& s2 = side == 0 ? A.m_surfaces[ch.surfA] : B.m_surfaces[ch.surfB];
                bool per[2] = {s2.is_closed(0), s2.is_closed(1)};
                if (!per[0] && !per[1]) { res.push_back(std::move(ch)); continue; }
                std::pair<double,double> dom[2] = {s2.domain(0), s2.domain(1)};
                auto uvs = [&](Chain& c) -> std::vector<Point>& { return side == 0 ? c.uvA : c.uvB; };
                auto jump_dir = [&](const std::vector<Point>& uv, size_t i) -> int {
                    for (int d = 0; d < 2; ++d) {
                        if (!per[d]) continue;
                        double period = dom[d].second - dom[d].first;
                        if (std::abs(uv[i+1][d] - uv[i][d]) > period * 0.5) return d;
                    }
                    return -1;
                };
                {
                    auto& uv = uvs(ch);
                    size_t n = uv.size();
                    std::vector<size_t> jumps;
                    for (size_t i = 0; i + 1 < n; ++i) if (jump_dir(uv, i) >= 0) jumps.push_back(i);
                    if (jumps.empty()) { res.push_back(std::move(ch)); continue; }
                    if (ch.closed) {   // rotate so the walk starts right after a jump
                        size_t r = (jumps[0] + 1) % n;
                        std::rotate(ch.p3.begin(),  ch.p3.begin()  + r, ch.p3.end());
                        std::rotate(ch.uvA.begin(), ch.uvA.begin() + r, ch.uvA.end());
                        std::rotate(ch.uvB.begin(), ch.uvB.begin() + r, ch.uvB.end());
                        ch.closed = false;
                        jumps.clear();
                        for (size_t i = 0; i + 1 < n; ++i) if (jump_dir(uv, i) >= 0) jumps.push_back(i);
                    }
                    SEval eS{&s2};
                    bool has_pend = false;
                    Point pend_p3(0,0,0), pend_A(0,0,0), pend_B(0,0,0);
                    size_t start2 = 0;
                    for (size_t ji = 0; ji <= jumps.size(); ++ji) {
                        size_t end2 = ji < jumps.size() ? jumps[ji] : n - 1;
                        Chain pc;
                        pc.surfA = ch.surfA; pc.surfB = ch.surfB; pc.closed = false;
                        if (has_pend) {
                            pc.p3.push_back(pend_p3);
                            pc.uvA.push_back(pend_A);
                            pc.uvB.push_back(pend_B);
                            has_pend = false;
                        }
                        for (size_t k = start2; k <= end2 && k < n; ++k) {
                            pc.p3.push_back(ch.p3[k]);
                            pc.uvA.push_back(ch.uvA[k]);
                            pc.uvB.push_back(ch.uvB[k]);
                        }
                        if (ji < jumps.size()) {
                            size_t i = jumps[ji];
                            int d = jump_dir(uv, i);
                            double period = dom[d].second - dom[d].first;
                            double u0 = uv[i][d];
                            double u1 = uv[i+1][d] + (uv[i+1][d] < uv[i][d] ? period : -period);
                            double bnd = (u1 > u0) ? dom[d].second : dom[d].first;
                            double f = std::abs(u1 - u0) > 1e-30 ? (bnd - u0) / (u1 - u0) : 0.5;
                            f = std::min(std::max(f, 0.0), 1.0);
                            Point xA = lerp(ch.uvA[i], ch.uvA[i+1], f);
                            Point xB = lerp(ch.uvB[i], ch.uvB[i+1], f);
                            double q4[4] = {xA[0], xA[1], xB[0], xB[1]};
                            int pin = (side == 0 ? 0 : 2) + d;
                            q4[pin] = bnd;
                            {
                                SEval eA2{&A.m_surfaces[ch.surfA]}, eB2{&B.m_surfaces[ch.surfB]};
                                correct7_pinned(eA2, eB2, q4[0], q4[1], q4[2], q4[3], pin, conv_tol);
                            }
                            q4[pin] = bnd;
                            Vector S, Su, Sv;
                            if (side == 0) eS(q4[0], q4[1], S, Su, Sv); else eS(q4[2], q4[3], S, Su, Sv);
                            Point p3x(S[0], S[1], S[2]);
                            pc.p3.push_back(p3x);
                            pc.uvA.push_back(Point(q4[0], q4[1], 0.0));
                            pc.uvB.push_back(Point(q4[2], q4[3], 0.0));
                            // mirrored start of the NEXT piece: same 3D, own coord at the
                            // OTHER seam bound
                            double other = (bnd == dom[d].second) ? dom[d].first : dom[d].second;
                            double m4[4] = {q4[0], q4[1], q4[2], q4[3]};
                            m4[pin] = other;
                            pend_p3 = p3x;
                            pend_A = Point(m4[0], m4[1], 0.0);
                            pend_B = Point(m4[2], m4[3], 0.0);
                            has_pend = true;
                        }
                        if ((int)pc.p3.size() >= 2) res.push_back(std::move(pc));
                        start2 = end2 + 1;
                    }
                }
            }
            in = std::move(res);
        };
        split_seams(chains, 0);
        split_seams(chains, 1);
    }

    scaf.n_chains = (int)chains.size();

    // ---- 1b. PutToBoundary (OCCT IntWalk analog) ----
    // The marcher stops when the NEXT step would exit a chart, leaving the chain end up
    // to a step SHORT of the bound that stopped it; the adjacent chart's continuation
    // chain starts exactly ON the shared border, so the two junction copies dangle a
    // step apart and never weld (the si=19 mid-face break class). Extend each open end:
    // extrapolate along the end tangent, find the first chart bound crossed, pin that
    // param to the bound, and re-converge the other three (correct7_pinned).
    for (auto& ch : chains) {
        if (ch.closed || ch.p3.size() < 2) continue;
        const NurbsSurface& sa = A.m_surfaces[ch.surfA];
        const NurbsSurface& sb = B.m_surfaces[ch.surfB];
        SEval ea{&sa}, eb{&sb};
        auto dua = sa.domain(0); auto dva = sa.domain(1);
        auto dub = sb.domain(0); auto dvb = sb.domain(1);
        double bnds[4][2] = {{dua.first, dua.second}, {dva.first, dva.second},
                             {dub.first, dub.second}, {dvb.first, dvb.second}};
        for (int end = 0; end < 2; ++end) {
            size_t i0 = end == 0 ? 0 : ch.p3.size() - 1;
            size_t i1 = end == 0 ? 1 : ch.p3.size() - 2;
            double cur[4] = {ch.uvA[i0][0], ch.uvA[i0][1], ch.uvB[i0][0], ch.uvB[i0][1]};
            double prv[4] = {ch.uvA[i1][0], ch.uvA[i1][1], ch.uvB[i1][0], ch.uvB[i1][1]};
            int on_k = -1, on_s = -1;
            for (int k = 0; k < 4 && on_k < 0; ++k)
                for (int s2 = 0; s2 < 2; ++s2)
                    if (std::abs(cur[k] - bnds[k][s2]) < 1e-9) { on_k = k; on_s = s2; break; }
            if (on_k >= 0) {
                // Already ON a bound -- but the clamp put it there with the marcher's
                // along-border drift (~one step). Re-converge ALONG the border (pin that
                // param): the true junction = section ∩ border curve, and the adjacent
                // chart's continuation converges to the SAME 3D point -- the weld closes
                // the 0.098-gap double-junction (si=19 class).
                double ext[4] = {cur[0], cur[1], cur[2], cur[3]};
                if (!correct7_pinned(ea, eb, ext[0], ext[1], ext[2], ext[3], on_k, conv_tol)) continue;
                ext[on_k] = bnds[on_k][on_s];
                ext[0] = std::min(std::max(ext[0], dua.first), dua.second);
                ext[1] = std::min(std::max(ext[1], dva.first), dva.second);
                ext[2] = std::min(std::max(ext[2], dub.first), dub.second);
                ext[3] = std::min(std::max(ext[3], dvb.first), dvb.second);
                Vector S, Su, Sv; ea(ext[0], ext[1], S, Su, Sv);
                Point pnew(S[0], S[1], S[2]);
                double step3 = ch.p3[i0].distance(ch.p3[i1]);
                double moved = pnew.distance(ch.p3[i0]);
                if (moved > std::max(step3 * 4.0, weld_tol * 4.0)) continue;
                if (moved > 1e-12) {
                    ch.p3[i0] = pnew;
                    ch.uvA[i0] = Point(ext[0], ext[1], 0.0);
                    ch.uvB[i0] = Point(ext[2], ext[3], 0.0);
                    if (std::getenv("SESSION_SPLIT_DBG"))
                        std::fprintf(stderr, "[SCAF-EXT] sA=%d sB=%d end=%d pin=%d ONBND moved=%.4f\n",
                                     ch.surfA, ch.surfB, end, on_k, moved);
                }
                continue;
            }
            double dir[4]; double dn = 0.0;
            for (int k = 0; k < 4; ++k) { dir[k] = cur[k] - prv[k]; dn += dir[k]*dir[k]; }
            if (dn < 1e-60) continue;
            // first bound crossed along the extrapolation, within 4 end-steps
            double best_f = 4.0; int best_k = -1, best_s = -1;
            for (int k = 0; k < 4; ++k) {
                if (std::abs(dir[k]) < 1e-30) continue;
                for (int s2 = 0; s2 < 2; ++s2) {
                    double f = (bnds[k][s2] - cur[k]) / dir[k];
                    if (f > 1e-9 && f < best_f) { best_f = f; best_k = k; best_s = s2; }
                }
            }
            if (best_k < 0) continue;   // interior stop (tangency class) -- leave
            double ext[4];
            for (int k = 0; k < 4; ++k) ext[k] = cur[k] + dir[k] * best_f;
            ext[best_k] = bnds[best_k][best_s];
            if (!correct7_pinned(ea, eb, ext[0], ext[1], ext[2], ext[3], best_k, conv_tol)) continue;
            ext[best_k] = bnds[best_k][best_s];   // keep the pin exact
            ext[0] = std::min(std::max(ext[0], dua.first), dua.second);
            ext[1] = std::min(std::max(ext[1], dva.first), dva.second);
            ext[2] = std::min(std::max(ext[2], dub.first), dub.second);
            ext[3] = std::min(std::max(ext[3], dvb.first), dvb.second);
            Vector S, Su, Sv; ea(ext[0], ext[1], S, Su, Sv);
            Point pnew(S[0], S[1], S[2]);
            double step3 = ch.p3[i0].distance(ch.p3[i1]);
            // reject wild landings: the true border point is ~one marcher step away
            if (pnew.distance(ch.p3[i0]) > std::max(step3 * 4.0, weld_tol * 4.0)) continue;
            Point uvA2(ext[0], ext[1], 0.0), uvB2(ext[2], ext[3], 0.0);
            if (end == 0) {
                ch.p3.insert(ch.p3.begin(), pnew);
                ch.uvA.insert(ch.uvA.begin(), uvA2);
                ch.uvB.insert(ch.uvB.begin(), uvB2);
            } else {
                ch.p3.push_back(pnew);
                ch.uvA.push_back(uvA2);
                ch.uvB.push_back(uvB2);
            }
            if (std::getenv("SESSION_SPLIT_DBG"))
                std::fprintf(stderr, "[SCAF-EXT] sA=%d sB=%d end=%d pin=%d d3=%.4f\n",
                             ch.surfA, ch.surfB, end, best_k, pnew.distance(ch.p3[end == 0 ? 1 : ch.p3.size()-2]));
        }
    }

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
    // Dedup by 3D DISTANCE, not sample count: the old half-sample window (chains have as
    // few as 48 samples -> 0.05-0.3 UV) collapsed distinct pave types -- a tB trim
    // crossing within half a sample of a tA crossing was DROPPED (trimA inserted first),
    // so B-side cuts ended at the tA position 0.02-0.09 INTERIOR to B's boundary and the
    // face never split (the B-side parts=1 class). Two paves are one point only if their
    // 3D positions weld.
    double pave_dedup = weld_tol * 0.5;
    auto add_pave = [&](Chain& ch, int i, double f, int kind) {
        Point p = lerp(ch.p3[i], ch.p3[std::min<size_t>(i + 1, ch.p3.size() - 1)], f);
        for (auto& pv : ch.paves) {
            Point q = lerp(ch.p3[pv.first], ch.p3[std::min<size_t>(pv.first + 1, ch.p3.size() - 1)], pv.second);
            if (p.distance(q) < pave_dedup) return false;
        }
        ch.paves.push_back({i, f});
        ch.pave_kinds[kind] += 1;
        return true;
    };

    for (auto& ch : chains) {
        // (a) trim crossings on A and B -- Newton-refined onto the exact section ∩ trim
        // point (chord-lerp paves carry up to half-chord sag; cross-chart copies of one
        // junction then miss the weld)
        const NurbsSurface& sa_r = A.m_surfaces[ch.surfA];
        const NurbsSurface& sb_r = B.m_surfaces[ch.surfB];
        SEval ea_r{&sa_r}, eb_r{&sb_r};
        for (int side = 0; side < 2; ++side) {
            const auto& uv = side == 0 ? ch.uvA : ch.uvB;
            const auto& ouv = side == 0 ? ch.uvB : ch.uvA;
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
                                    // refine: c(t) on the loop segment x the other surface
                                    const Point& qa = poly[j];
                                    const Point& qb = poly[(j+1)%np];
                                    double tr = t;
                                    Point osd = lerp(ouv[i], ouv[std::min(i+1, ouv.size()-1)], s);
                                    double uo = osd[0], vo = osd[1];
                                    bool okr = side == 0
                                        ? refine_trim_pave(ea_r, eb_r, qa, qb, tr, uo, vo, conv_tol)
                                        : refine_trim_pave(eb_r, ea_r, qa, qb, tr, uo, vo, conv_tol);
                                    if (!okr && std::getenv("SESSION_SPLIT_DBG"))
                                        std::fprintf(stderr, "[PVFIX] FAIL sA=%d sB=%d side=%d i=%zu s=%.3f\n",
                                                     ch.surfA, ch.surfB, side, i, s);
                                    if (okr) {
                                        Point cuv(qa[0] + (qb[0]-qa[0])*tr, qa[1] + (qb[1]-qa[1])*tr, 0.0);
                                        Vector S, Su, Sv;
                                        if (side == 0) ea_r(cuv[0], cuv[1], S, Su, Sv);
                                        else           eb_r(cuv[0], cuv[1], S, Su, Sv);
                                        Point p3r(S[0], S[1], S[2]);
                                        // sanity: refined point stays near the chord crossing
                                        Point p3c = lerp(ch.p3[i], ch.p3[std::min(i+1, ch.p3.size()-1)], s);
                                        if (std::getenv("SESSION_SPLIT_DBG") && p3r.distance(p3c) >= weld_tol * 6.0)
                                            std::fprintf(stderr, "[PVFIX] REJ sA=%d sB=%d side=%d i=%zu d=%.4f\n",
                                                         ch.surfA, ch.surfB, side, i, p3r.distance(p3c));
                                        if (p3r.distance(p3c) < weld_tol * 6.0) {
                                            Point ouv2(uo, vo, 0.0);
                                            ch.pave_fix[i + s] = side == 0
                                                ? std::array<Point,3>{p3r, cuv, ouv2}
                                                : std::array<Point,3>{p3r, ouv2, cuv};
                                        }
                                    }
                                }
                            }
                        }
                }
        }
    }
    // (a2) chart-border touch paves: a chain that grazes out of the chart gets CLAMPED
    // onto the border (correct7 clamp above) and runs COLLINEAR with a trim that lies on
    // that border -- seg_seg (den~0) never detects the crossing, the pave is missed, and
    // the two inside runs dangle 0.05-0.1 apart (the si=19 class). Pave every
    // inside<->on-border transition; the on-border run itself is dropped later by the
    // split's own-border test, and the inside runs end AT the border, fnode-snappable.
    for (auto& ch : chains) {
        for (int side = 0; side < 2; ++side) {
            const auto& uv = side == 0 ? ch.uvA : ch.uvB;
            const NurbsSurface& srf = side == 0 ? A.m_surfaces[ch.surfA] : B.m_surfaces[ch.surfB];
            auto du = srf.domain(0); auto dv = srf.domain(1);
            auto on_bord = [&](const Point& q) {
                return q[0] <= du.first + 1e-12 || q[0] >= du.second - 1e-12 ||
                       q[1] <= dv.first + 1e-12 || q[1] >= dv.second - 1e-12;
            };
            for (size_t i = 0; i + 1 < uv.size(); ++i) {
                bool b0 = on_bord(uv[i]), b1 = on_bord(uv[i+1]);
                if (b0 == b1) continue;
                if (add_pave(ch, (int)i, b1 ? 1.0 : 0.0, side)) {
                    if (side == 0) scaf.n_paves_trimA += 1; else scaf.n_paves_trimB += 1;
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
        for (size_t c2 = c1; c2 < chains.size(); ++c2) {   // c2==c1: SELF-crossings too --
            // a chain's UV image can cross itself in one chart (U-shaped sections); the
            // arrangement Newton-nodes that crossing, and without a matching pave the
            // segment splits into partial runs the alias/sew layers cannot pair.
            for (int side = 0; side < 2; ++side) {
                bool share = side == 0 ? chains[c1].surfA == chains[c2].surfA
                                       : chains[c1].surfB == chains[c2].surfB;
                if (!share) continue;
                const auto& uv1 = side == 0 ? chains[c1].uvA : chains[c1].uvB;
                const auto& uv2 = side == 0 ? chains[c2].uvA : chains[c2].uvB;
                auto b1 = uv_bbox(uv1), b2 = uv_bbox(uv2);
                if (b1[0] > b2[1] || b1[1] < b2[0] || b1[2] > b2[3] || b1[3] < b2[2]) continue;
                for (size_t i = 0; i + 1 < uv1.size(); ++i)
                    for (size_t j = (c1 == c2 ? i + 2 : 0); j + 1 < uv2.size(); ++j) {
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
            // Newton-refined pave override (sag-free section ∩ trim point)
            double keyw = pos;
            if (ch.closed) { while (keyw >= nS - 1) keyw -= (nS - 1); }
            auto itF = ch.pave_fix.lower_bound(keyw - 1e-9);
            if (itF != ch.pave_fix.end() && std::abs(itF->first - keyw) < 1e-9) {
                p3o = itF->second[0];
                uvAo = itF->second[1];
                uvBo = itF->second[2];
                return;
            }
            double w = pos;
            if (ch.closed) { while (w >= nS - 1) w -= (nS - 1); }   // wrap ONLY closed chains
            else if (w > nS - 1) w = (double)(nS - 1);              // open: clamp to the true tail
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
            bool inA = in_faces_uv(loopsA[ch.surfA], am[0], am[1], epsA);
            bool inB = in_faces_uv(loopsB[ch.surfB], bm[0], bm[1], epsB);
            if (!(inA && inB)) {
                scaf.n_dropped_verdict += 1;
                if (std::getenv("SESSION_SPLIT_DBG"))
                    std::fprintf(stderr, "[SCAF-DROP] sA=%d sB=%d inA=%d inB=%d uvA(%.3f,%.3f) uvB(%.3f,%.3f)\n",
                                 ch.surfA, ch.surfB, inA ? 1 : 0, inB ? 1 : 0, am[0], am[1], bm[0], bm[1]);
                continue;
            }
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

    // ---- 4. Unify per-side UV at shared vertices ----
    // Sections continuing across an operand's face border arrive as chains from DIFFERENT
    // surface pairs; their junction is one welded 3D vertex, but each chain's UV footprint
    // of it differs by sampling (~1e-3) -- far beyond the arrangement's 1e-6 vertex weld,
    // so the junction never nodes and both cut ends dangle (valence-1 prune eats them).
    // Snap every segment-end's UV to ONE canonical value per (vertex, side, surface):
    // exact-equal doubles weld by identity in the arrangement's vertex pool.
    {
        std::map<std::tuple<int, int, int>, Point> canon;   // (vertex, side, surf) -> uv
        for (auto& seg : scaf.segments) {
            for (int end = 0; end < 2; ++end) {
                int v = end == 0 ? seg.v_start : seg.v_end;
                for (int side = 0; side < 2; ++side) {
                    auto& uv = side == 0 ? seg.uvA : seg.uvB;
                    int surf = side == 0 ? seg.surfA : seg.surfB;
                    Point& q = end == 0 ? uv.front() : uv.back();
                    auto key = std::make_tuple(v, side, surf);
                    auto it = canon.find(key);
                    if (it == canon.end()) canon[key] = q;
                    else q = it->second;
                }
            }
            // 3D endpoints likewise unified to the welded vertex position
            if (seg.v_start >= 0) seg.p3.front() = scaf.vertices[seg.v_start];
            if (seg.v_end >= 0) seg.p3.back() = scaf.vertices[seg.v_end];
        }
    }

    // ---- 5. Valence audit ([SCAF-VAL]) ----
    // Sections of closed solids are closed loops: every scaffold vertex should have even
    // valence (2 normally, 4 at crossings). A valence-1 vertex = a dangling section end =
    // a genuinely missing continuation (SSI gap) or an endpoint drop -- the residual
    // naked-edge budget. Print each with its nearest other valence-1 vertex distance to
    // size a future chain-end bridging pass.
    if (std::getenv("SESSION_SPLIT_DBG")) {
        for (const auto& seg : scaf.segments)
            std::fprintf(stderr, "[SCAF-SEG] id=%d sA=%d sB=%d v=%d..%d uvB(%.4f,%.4f)->(%.4f,%.4f) p3(%.4f,%.4f,%.4f)->(%.4f,%.4f,%.4f)\n",
                         seg.seg_id, seg.surfA, seg.surfB, seg.v_start, seg.v_end,
                         seg.uvB.front()[0], seg.uvB.front()[1], seg.uvB.back()[0], seg.uvB.back()[1],
                         seg.p3.front()[0], seg.p3.front()[1], seg.p3.front()[2],
                         seg.p3.back()[0], seg.p3.back()[1], seg.p3.back()[2]);
        std::vector<int> val(scaf.vertices.size(), 0);
        for (const auto& seg : scaf.segments) {
            if (seg.v_start >= 0) val[seg.v_start] += 1;
            if (seg.v_end >= 0) val[seg.v_end] += 1;
        }
        int n1 = 0;
        for (size_t vi = 0; vi < val.size(); ++vi) {
            if (val[vi] != 1) continue;
            ++n1;
            double dnear = 1e300; int vnear = -1;
            for (size_t vj = 0; vj < val.size(); ++vj)
                if (vj != vi && val[vj] == 1) {
                    double d = scaf.vertices[vi].distance(scaf.vertices[vj]);
                    if (d < dnear) { dnear = d; vnear = (int)vj; }
                }
            std::fprintf(stderr, "[SCAF-VAL] v=%zu val=1 p(%.3f,%.3f,%.3f) nearest_v1=%d d=%.4f\n",
                         vi, scaf.vertices[vi][0], scaf.vertices[vi][1], scaf.vertices[vi][2],
                         vnear, vnear >= 0 ? dnear : -1.0);
        }
        std::fprintf(stderr, "[SCAF-VAL] valence1=%d of %zu vertices\n", n1, scaf.vertices.size());
        for (size_t vi = 0; vi < scaf.vertices.size(); ++vi)
            std::fprintf(stderr, "[SCAF-V] %zu %.4f %.4f %.4f val=%d\n",
                         vi, scaf.vertices[vi][0], scaf.vertices[vi][1], scaf.vertices[vi][2], val[vi]);
        std::fflush(stderr);
    }
    return scaf;
}

} // namespace session_cpp
