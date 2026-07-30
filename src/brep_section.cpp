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
#include <set>
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

// True corner = 3-surface intersection point (OCCT: vertices come from EF/FF interference
// solves, never from polyline crossings). Newton on 6 unknowns: S0(u0,v0) = S1(u1,v1) and
// S0(u0,v0) = S2(u2,v2). Without this, x-pave corners sit at chord-lerp crossings with up
// to half-a-chord (~0.1) of sag -- the 'badly computed corners' the user sees.
bool refine_triple_point(const SEval& e0, const SEval& e1, const SEval& e2,
                         double x[6], double conv_tol) {
    for (int it = 0; it < 14; ++it) {
        Vector P0, P0u, P0v, P1, P1u, P1v, P2, P2u, P2v;
        e0(x[0], x[1], P0, P0u, P0v);
        e1(x[2], x[3], P1, P1u, P1v);
        e2(x[4], x[5], P2, P2u, P2v);
        double F[6] = {P0[0]-P1[0], P0[1]-P1[1], P0[2]-P1[2],
                       P0[0]-P2[0], P0[1]-P2[1], P0[2]-P2[2]};
        double fn = 0; for (int k = 0; k < 6; ++k) fn += F[k]*F[k];
        if (std::sqrt(fn) < conv_tol) return true;
        double J[6][6] = {};
        for (int k = 0; k < 3; ++k) {
            J[k][0] = P0u[k];   J[k][1] = P0v[k];
            J[k][2] = -P1u[k];  J[k][3] = -P1v[k];
            J[k+3][0] = P0u[k]; J[k+3][1] = P0v[k];
            J[k+3][4] = -P2u[k]; J[k+3][5] = -P2v[k];
        }
        // 6x6 Gaussian elimination with partial pivoting
        double M[6][7];
        for (int r = 0; r < 6; ++r) { for (int c = 0; c < 6; ++c) M[r][c] = J[r][c]; M[r][6] = F[r]; }
        for (int c = 0; c < 6; ++c) {
            int piv = c;
            for (int r = c + 1; r < 6; ++r) if (std::abs(M[r][c]) > std::abs(M[piv][c])) piv = r;
            if (std::abs(M[piv][c]) < 1e-30) return false;
            if (piv != c) for (int k = c; k < 7; ++k) std::swap(M[piv][k], M[c][k]);
            for (int r = c + 1; r < 6; ++r) {
                double f = M[r][c] / M[c][c];
                for (int k = c; k < 7; ++k) M[r][k] -= f * M[c][k];
            }
        }
        double dx[6];
        for (int r = 5; r >= 0; --r) {
            double s = M[r][6];
            for (int c = r + 1; c < 6; ++c) s -= M[r][c] * dx[c];
            dx[r] = s / M[r][r];
        }
        for (int k = 0; k < 6; ++k) x[k] -= dx[k];
    }
    Vector P0, P0u, P0v, P1, P1u, P1v, P2, P2u, P2v;
    e0(x[0], x[1], P0, P0u, P0v);
    e1(x[2], x[3], P1, P1u, P1v);
    e2(x[4], x[5], P2, P2u, P2v);
    double g1 = std::sqrt((P0[0]-P1[0])*(P0[0]-P1[0])+(P0[1]-P1[1])*(P0[1]-P1[1])+(P0[2]-P1[2])*(P0[2]-P1[2]));
    double g2 = std::sqrt((P0[0]-P2[0])*(P0[0]-P2[0])+(P0[1]-P2[1])*(P0[1]-P2[1])+(P0[2]-P2[2])*(P0[2]-P2[2]));
    return g1 < conv_tol * 10.0 && g2 < conv_tol * 10.0;
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

namespace {
// Are two surfaces the SAME surface over their shared domain (same-domain / coincident)?
// Grid-sample sa and, for each sample, take the closest point on sb: coincident iff EVERY
// sample lies within band. Areal by construction (Law 3) -- a tangent line or a single
// touching point can never pass, because only a measure-zero subset of samples would hit.
bool surfaces_coincident(const NurbsSurface& sa, const NurbsSurface& sb, double band) {
    auto du = sa.domain(0); auto dv = sa.domain(1);
    const int N = 4;   // 5x5 = 25 probes
    for (int i = 0; i <= N; ++i)
        for (int j = 0; j <= N; ++j) {
            double u = du.first + (du.second - du.first) * i / N;
            double v = dv.first + (dv.second - dv.first) * j / N;
            Point p = sa.point_at(u, v);
            auto [cu, cv, cd] = Closest::surface_point(sb, p);
            if (!(cd < band)) return false;
            // ORIENTATION GATE: distance alone is the unguarded-predicate bug this campaign
            // has now found twice (the legacy ON-imprint had exactly this shape). band is
            // weld_tol ~ 3.3e-2 here, fat enough that two surfaces MEETING at an angle can
            // sit inside it over a whole patch. Coincident surfaces must also be PARALLEL.
            Vector na = sa.normal_at(u, v), nb = sb.normal_at(cu, cv);
            if (std::abs(na[0]*nb[0] + na[1]*nb[1] + na[2]*nb[2]) < 0.9) return false;
        }
    return true;
}
}  // namespace

SectionScaffold build_section_scaffold(const BRep& A, const BRep& B, double tolerance) {
    // Same-domain SSI guard: opt-in while it is measured (SESSION_SD). See [SD-SKIP] below.
    static const bool s_sd_guard = (std::getenv("SESSION_SD") != nullptr);
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
    if (const char* wm = std::getenv("SESSION_TOL3_MULT")) weld_tol *= std::atof(wm);
    double conv_tol = std::max(tolerance, diag * 1e-9);
    scaf.tol3 = weld_tol;
    scaf.tol3_rep = weld_tol;
    if (const char* rm = std::getenv("SESSION_REP_MULT")) scaf.tol3_rep = weld_tol * std::atof(rm);
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
            // SAME-DOMAIN GUARD (SESSION_SD): a surface cannot be intersected with itself.
            // When sa and sb are the SAME surface (coincident within band), the true section
            // is the whole shared patch, not a curve -- the marcher instead emits a mess of
            // spurious branches. MEASURED on A-op-A (chair1 = byte copy of chair0): 481
            // "section" segments that shatter A's 20 faces into 312 fragments, destroying the
            // operands before classification ever runs (cut 68 faces/16 naked, common 203/42,
            // fuse times out). Coincident pairs belong to the same-domain path (op-table),
            // never to SSI. Detection is a direct sampled test on this pair, so it costs
            // nothing when the surfaces are apart.
            if (s_sd_guard && surfaces_coincident(sa, sb, weld_tol)) {
                if (std::getenv("SESSION_SPLIT_DBG"))
                    std::fprintf(stderr, "[SD-SKIP] surfA=%d surfB=%d coincident -> no SSI\n", ai, bi);
                continue;
            }
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
            // ---- 1a1. BRANCH ENUMERATOR (SESSION_SEED2; OCCT IntPolyh analog) ----
            // The marcher is seed-limited: branches it was never seeded on (the 0.128
            // mini-branch class, grazing arcs) simply do not exist in `trs`. Enumerate
            // intersection branches POLYHEDRALLY (grid triangles x grid triangles ->
            // couple soup -> proximity components) and MARCH any component no existing
            // chain covers, seeding from the component's own barycentric UVs.
            if (std::getenv("SESSION_SEED2")) {
                const int GN = 24;
                struct GridS {
                    std::vector<Point> p3;               // (GN+1)^2 nodes
                    std::vector<std::pair<double,double>> uv;
                };
                auto sample_grid = [&](const NurbsSurface& s) {
                    GridS g;
                    auto du = s.domain(0); auto dv = s.domain(1);
                    for (int i = 0; i <= GN; ++i)
                        for (int j = 0; j <= GN; ++j) {
                            double u = du.first + (du.second - du.first) * i / GN;
                            double v = dv.first + (dv.second - dv.first) * j / GN;
                            g.p3.push_back(s.point_at(u, v));
                            g.uv.push_back({u, v});
                        }
                    return g;
                };
                GridS ga = sample_grid(sa), gb = sample_grid(sb);
                auto node = [&](const GridS& g, int i, int j) -> const Point& {
                    return g.p3[i * (GN + 1) + j];
                };
                // segment of tri-tri intersection (Moller-style via plane clipping)
                auto tri_seg = [&](const Point& a0, const Point& a1, const Point& a2,
                                   const Point& b0, const Point& b1, const Point& b2,
                                   Point& s0, Point& s1) -> bool {
                    auto nrm = [](const Point& p0, const Point& p1, const Point& p2) {
                        double ux = p1[0]-p0[0], uy = p1[1]-p0[1], uz = p1[2]-p0[2];
                        double vx = p2[0]-p0[0], vy = p2[1]-p0[1], vz = p2[2]-p0[2];
                        return Point(uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx);
                    };
                    auto dot3 = [](const Point& n, const Point& p, const Point& o) {
                        return n[0]*(p[0]-o[0]) + n[1]*(p[1]-o[1]) + n[2]*(p[2]-o[2]);
                    };
                    // clip triangle A's edges against B's plane, collect crossings inside B
                    Point nb = nrm(b0, b1, b2);
                    double d0 = dot3(nb, a0, b0), d1 = dot3(nb, a1, b0), d2 = dot3(nb, a2, b0);
                    const Point* T[3] = {&a0, &a1, &a2};
                    double D[3] = {d0, d1, d2};
                    std::vector<Point> hits;
                    for (int e = 0; e < 3; ++e) {
                        double da = D[e], db2 = D[(e+1)%3];
                        if ((da > 0) == (db2 > 0)) continue;
                        double f = da / (da - db2);
                        const Point& pa = *T[e];
                        const Point& pb = *T[(e+1)%3];
                        Point q(pa[0]+f*(pb[0]-pa[0]), pa[1]+f*(pb[1]-pa[1]), pa[2]+f*(pb[2]-pa[2]));
                        // inside B? barycentric via areas against nb
                        Point w0 = nrm(b0, b1, q), w1 = nrm(b1, b2, q), w2 = nrm(b2, b0, q);
                        double s0d = w0[0]*nb[0]+w0[1]*nb[1]+w0[2]*nb[2];
                        double s1d = w1[0]*nb[0]+w1[1]*nb[1]+w1[2]*nb[2];
                        double s2d = w2[0]*nb[0]+w2[1]*nb[1]+w2[2]*nb[2];
                        if (s0d >= -1e-18 && s1d >= -1e-18 && s2d >= -1e-18) hits.push_back(q);
                    }
                    // symmetric: B's edges against A's plane
                    Point na = nrm(a0, a1, a2);
                    double e0 = dot3(na, b0, a0), e1 = dot3(na, b1, a0), e2 = dot3(na, b2, a0);
                    const Point* U[3] = {&b0, &b1, &b2};
                    double E[3] = {e0, e1, e2};
                    for (int e = 0; e < 3; ++e) {
                        double da = E[e], db2 = E[(e+1)%3];
                        if ((da > 0) == (db2 > 0)) continue;
                        double f = da / (da - db2);
                        const Point& pa = *U[e];
                        const Point& pb = *U[(e+1)%3];
                        Point q(pa[0]+f*(pb[0]-pa[0]), pa[1]+f*(pb[1]-pa[1]), pa[2]+f*(pb[2]-pa[2]));
                        Point w0 = nrm(a0, a1, q), w1 = nrm(a1, a2, q), w2 = nrm(a2, a0, q);
                        double s0d = w0[0]*na[0]+w0[1]*na[1]+w0[2]*na[2];
                        double s1d = w1[0]*na[0]+w1[1]*na[1]+w1[2]*na[2];
                        double s2d = w2[0]*na[0]+w2[1]*na[1]+w2[2]*na[2];
                        if (s0d >= -1e-18 && s1d >= -1e-18 && s2d >= -1e-18) hits.push_back(q);
                    }
                    if (hits.size() < 2) return false;
                    // farthest pair among hits (robust to duplicates)
                    double bd = -1;
                    for (size_t x = 0; x < hits.size(); ++x)
                        for (size_t y = x + 1; y < hits.size(); ++y) {
                            double d = hits[x].distance(hits[y]);
                            if (d > bd) { bd = d; s0 = hits[x]; s1 = hits[y]; }
                        }
                    return bd > 1e-14;
                };
                // couple soup with a coarse 3D hash on B's cell bboxes
                struct Couple { Point m; int ia, ja, ib, jb; };
                std::vector<Couple> soup;
                double cell = std::max(1e-12, weld_tol * 8.0);
                std::map<std::tuple<long long,long long,long long>, std::vector<std::pair<int,int>>> bhash;
                auto keyof = [&](const Point& p) {
                    return std::make_tuple((long long)std::floor(p[0]/cell),
                                           (long long)std::floor(p[1]/cell),
                                           (long long)std::floor(p[2]/cell));
                };
                for (int i = 0; i < GN; ++i)
                    for (int j = 0; j < GN; ++j) {
                        Point c0 = node(gb,i,j), c1 = node(gb,i+1,j), c2 = node(gb,i,j+1), c3 = node(gb,i+1,j+1);
                        Point mid((c0[0]+c3[0])*0.5, (c0[1]+c3[1])*0.5, (c0[2]+c3[2])*0.5);
                        bhash[keyof(mid)].push_back({i, j});
                    }
                for (int i = 0; i < GN; ++i)
                    for (int j = 0; j < GN; ++j) {
                        Point a0 = node(ga,i,j), a1 = node(ga,i+1,j), a2 = node(ga,i,j+1), a3 = node(ga,i+1,j+1);
                        Point am((a0[0]+a3[0])*0.5, (a0[1]+a3[1])*0.5, (a0[2]+a3[2])*0.5);
                        auto k0 = keyof(am);
                        for (long long dx = -2; dx <= 2; ++dx)
                        for (long long dy = -2; dy <= 2; ++dy)
                        for (long long dz = -2; dz <= 2; ++dz) {
                            auto it = bhash.find(std::make_tuple(std::get<0>(k0)+dx, std::get<1>(k0)+dy, std::get<2>(k0)+dz));
                            if (it == bhash.end()) continue;
                            for (auto [bi2, bj2] : it->second) {
                                Point b0 = node(gb,bi2,bj2), b1 = node(gb,bi2+1,bj2), b2 = node(gb,bi2,bj2+1), b3 = node(gb,bi2+1,bj2+1);
                                Point s0, s1;
                                for (int ta = 0; ta < 2; ++ta)
                                    for (int tb = 0; tb < 2; ++tb) {
                                        const Point& A1 = ta ? a3 : a0;
                                        const Point& B1 = tb ? b3 : b0;
                                        if (tri_seg(ta ? a1 : a0, ta ? a2 : a1, ta ? a3 : a2,
                                                    tb ? b1 : b0, tb ? b2 : b1, tb ? b3 : b2, s0, s1)) {
                                            (void)A1; (void)B1;
                                            Point m((s0[0]+s1[0])*0.5, (s0[1]+s1[1])*0.5, (s0[2]+s1[2])*0.5);
                                            soup.push_back({m, i, j, bi2, bj2});
                                        }
                                    }
                            }
                        }
                    }
                if (!soup.empty()) {
                    // components: union-find by proximity (couples within 2 cells)
                    std::vector<int> par(soup.size());
                    for (size_t k = 0; k < par.size(); ++k) par[k] = (int)k;
                    std::function<int(int)> find = [&](int x) { return par[x] == x ? x : par[x] = find(par[x]); };
                    double link = cell * 2.0;
                    for (size_t x = 0; x < soup.size(); ++x)
                        for (size_t y = x + 1; y < soup.size(); ++y)
                            if (soup[x].m.distance(soup[y].m) < link) par[find((int)x)] = find((int)y);
                    std::map<int, std::vector<int>> comps;
                    for (size_t k = 0; k < soup.size(); ++k) comps[find((int)k)].push_back((int)k);
                    int n_marched2 = 0;
                    for (auto& cmp : comps) {
                        if ((int)cmp.second.size() < 2) continue;         // isolated touch
                        // covered by an existing chain of THIS pair?
                        const Couple& rep = soup[cmp.second[cmp.second.size()/2]];
                        double dmin = 1e300;
                        for (const auto& ch2 : chains) {
                            if (ch2.surfA != ai || ch2.surfB != bi) continue;
                            for (const auto& q : ch2.p3) dmin = std::min(dmin, q.distance(rep.m));
                        }
                        double defl = cell;                               // grid deflection scale
                        if (dmin < defl * 2.0) continue;                  // covered
                        // MISSING BRANCH: march from the representative couple's UVs
                        double uA0 = 0.5 * (ga.uv[rep.ia*(GN+1)+rep.ja].first  + ga.uv[(rep.ia+1)*(GN+1)+rep.ja+1].first);
                        double vA0 = 0.5 * (ga.uv[rep.ia*(GN+1)+rep.ja].second + ga.uv[(rep.ia+1)*(GN+1)+rep.ja+1].second);
                        double uB0 = 0.5 * (gb.uv[rep.ib*(GN+1)+rep.jb].first  + gb.uv[(rep.ib+1)*(GN+1)+rep.jb+1].first);
                        double vB0 = 0.5 * (gb.uv[rep.ib*(GN+1)+rep.jb].second + gb.uv[(rep.ib+1)*(GN+1)+rep.jb+1].second);
                        if (!correct7(ea, eb, uA0, vA0, uB0, vB0, conv_tol)) continue;
                        Vector S0, Su0, Sv0; ea(uA0, vA0, S0, Su0, Sv0);
                        Point P0(S0[0], S0[1], S0[2]);
                        Vector na2 = sa.normal_at(uA0, vA0), nb2 = sb.normal_at(uB0, vB0);
                        Vector T2(na2[1]*nb2[2]-na2[2]*nb2[1], na2[2]*nb2[0]-na2[0]*nb2[2], na2[0]*nb2[1]-na2[1]*nb2[0]);
                        double T2l = T2.magnitude();
                        if (T2l < 1e-10) continue;
                        T2 = Vector(T2[0]/T2l, T2[1]/T2l, T2[2]/T2l);
                        double h = std::max(weld_tol * 0.5, defl * 0.25);
                        std::vector<Point> mp3{P0}, mua{Point(uA0,vA0,0)}, mub{Point(uB0,vB0,0)};
                        for (int sgn = -1; sgn <= 1; sgn += 2) {
                            double uA2 = uA0, vA2 = vA0, uB2 = uB0, vB2 = vB0;
                            Point cur = P0;
                            Vector Td = T2;
                            std::vector<Point> part3, partA, partB;
                            for (int st = 0; st < 400; ++st) {
                                Point nxt(cur[0]+sgn*h*Td[0], cur[1]+sgn*h*Td[1], cur[2]+sgn*h*Td[2]);
                                auto [ua1, va1, dq1] = Closest::surface_point(sa, nxt, uA2-0.3, uA2+0.3, vA2-0.3, vA2+0.3);
                                auto [ub1, vb1, dq2] = Closest::surface_point(sb, nxt, uB2-0.3, uB2+0.3, vB2-0.3, vB2+0.3);
                                (void)dq1; (void)dq2;
                                uA2 = ua1; vA2 = va1; uB2 = ub1; vB2 = vb1;
                                if (!correct7(ea, eb, uA2, vA2, uB2, vB2, conv_tol)) break;
                                if (uA2 < dua.first - 1e-9 || uA2 > dua.second + 1e-9 ||
                                    vA2 < dva.first - 1e-9 || vA2 > dva.second + 1e-9 ||
                                    uB2 < dub.first - 1e-9 || uB2 > dub.second + 1e-9 ||
                                    vB2 < dvb.first - 1e-9 || vB2 > dvb.second + 1e-9) break;
                                Vector Sx, Sxu, Sxv; ea(uA2, vA2, Sx, Sxu, Sxv);
                                Point pn(Sx[0], Sx[1], Sx[2]);
                                if (pn.distance(P0) < h * 0.5 && st > 4) { part3.push_back(P0); break; }  // closed loop
                                part3.push_back(pn);
                                partA.push_back(Point(uA2, vA2, 0));
                                partB.push_back(Point(uB2, vB2, 0));
                                Vector na3 = sa.normal_at(uA2, vA2), nb3 = sb.normal_at(uB2, vB2);
                                Vector T3(na3[1]*nb3[2]-na3[2]*nb3[1], na3[2]*nb3[0]-na3[0]*nb3[2], na3[0]*nb3[1]-na3[1]*nb3[0]);
                                double T3l = T3.magnitude();
                                if (T3l > 1e-10) Td = Vector(T3[0]/T3l, T3[1]/T3l, T3[2]/T3l);
                                cur = pn;
                            }
                            if (sgn < 0) {
                                std::reverse(part3.begin(), part3.end());
                                std::reverse(partA.begin(), partA.end());
                                std::reverse(partB.begin(), partB.end());
                                part3.insert(part3.end(), mp3.begin(), mp3.end());
                                partA.insert(partA.end(), mua.begin(), mua.end());
                                partB.insert(partB.end(), mub.begin(), mub.end());
                                mp3 = std::move(part3); mua = std::move(partA); mub = std::move(partB);
                            } else {
                                mp3.insert(mp3.end(), part3.begin(), part3.end());
                                mua.insert(mua.end(), partA.begin(), partA.end());
                                mub.insert(mub.end(), partB.begin(), partB.end());
                            }
                        }
                        if ((int)mp3.size() < 3) continue;
                        Chain nc;
                        nc.surfA = ai; nc.surfB = bi;
                        nc.p3 = std::move(mp3); nc.uvA = std::move(mua); nc.uvB = std::move(mub);
                        nc.closed = nc.p3.front().distance(nc.p3.back()) < weld_tol;
                        chains.push_back(std::move(nc));
                        ++n_marched2;
                        if (std::getenv("SESSION_SPLIT_DBG"))
                            std::fprintf(stderr, "[SEED2] pair(%d,%d) comp n=%zu dmin=%.4f MARCHED %zu pts\n",
                                         ai, bi, cmp.second.size(), dmin, chains.back().p3.size());
                    }
                    if (n_marched2 && std::getenv("SESSION_NT_DBG"))
                        std::fprintf(stderr, "[SEED2] pair(%d,%d): %d missing branch(es) marched\n", ai, bi, n_marched2);
                }
            }
        }
    }
    // ---- 1a1b. COMMON-BLOCK SUPPRESSION (SESSION_SD; OCCT PerformCommonBlocks slot) ----
    // Step 2 of the same-domain requirement. When a region of the two boundaries COINCIDES,
    // the marcher still runs on every ADJACENT face pair (A's face i x B's face j, i != j)
    // and those pairs genuinely intersect -- along the solids' SHARED EDGES. On A-op-A that
    // is 481 "sections" from 20x20 pairs, shattering A's 20 faces into 312 fragments. Such a
    // curve is not a transversal section at all: it is a COMMON BLOCK (one edge belonging to
    // both operands), and OCCT builds it with BOPAlgo_Tools::PerformCommonBlocks instead of
    // marching it (kb/audit_occt_pavefiller-core.md). We do not yet SYNTHESISE common blocks;
    // this pass only refuses to treat them as new section curves. Test is deliberately
    // two-sided -- the chain must lie on an EXISTING edge of A *and* of B -- so the grazing
    // class where a section runs along ONE operand's trim (the y30 family) is untouched.
    if (s_sd_guard && !chains.empty()) {
        auto edge_polys = [&](const BRep& X) {
            std::vector<std::vector<Point>> out;
            for (const auto& e : X.m_topology_edges) {
                if (e.curve_3d_index < 0 || e.curve_3d_index >= (int)X.m_curves_3d.size()) continue;
                const NurbsCurve& c = X.m_curves_3d[e.curve_3d_index];
                if (!c.is_valid()) continue;
                auto d = c.domain();
                std::vector<Point> ps;
                for (int k = 0; k <= 24; ++k) ps.push_back(c.point_at(d.first + (d.second-d.first)*k/24.0));
                out.push_back(std::move(ps));
            }
            return out;
        };
        std::vector<std::vector<Point>> ea = edge_polys(A), eb = edge_polys(B);
        auto on_some_edge = [&](const std::vector<std::vector<Point>>& polys, const Chain& ch) {
            // every probe of the chain must be within band of ONE single edge polyline
            for (const auto& poly : polys) {
                bool all_on = true;
                for (int k = 0; k <= 8 && all_on; ++k) {
                    const Point& q = ch.p3[(ch.p3.size() - 1) * k / 8];
                    double bd = 1e300;
                    for (size_t j = 0; j + 1 < poly.size(); ++j) {
                        double ex = poly[j+1][0]-poly[j][0], ey = poly[j+1][1]-poly[j][1], ez = poly[j+1][2]-poly[j][2];
                        double L2 = ex*ex + ey*ey + ez*ez;
                        double t = L2 > 1e-30 ? ((q[0]-poly[j][0])*ex + (q[1]-poly[j][1])*ey + (q[2]-poly[j][2])*ez)/L2 : 0.0;
                        t = std::min(1.0, std::max(0.0, t));
                        Point cp(poly[j][0]+t*ex, poly[j][1]+t*ey, poly[j][2]+t*ez);
                        bd = std::min(bd, q.distance(cp));
                    }
                    if (!(bd < weld_tol)) all_on = false;
                }
                if (all_on) return true;
            }
            return false;
        };
        std::vector<Chain> kept;
        int n_cb = 0;
        for (auto& ch : chains) {
            if ((int)ch.p3.size() >= 2 && on_some_edge(ea, ch) && on_some_edge(eb, ch)) {
                ++n_cb;
                continue;                       // common block: not a new section curve
            }
            kept.push_back(std::move(ch));
        }
        if (n_cb) {
            std::fprintf(stderr, "[SD-CB] common-block chains suppressed: %d of %zu\n",
                         n_cb, chains.size());
            std::fflush(stderr);
        }
        chains = std::move(kept);
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
    std::map<int, std::vector<std::vector<std::vector<Point>>>> lpA, lpB;  // EXT_TRIM loop cache
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
                // EXT_TRIM pinned variant: an end pinned on THIS chart bound can still be
                // short of the OTHER operand's TRIM (x20 knot: uvB on B's bound, uvA 0.128
                // inside A-face-18). March ALONG the pinned border (correct7_pinned) until
                // the other operand's trim state flips; end the chain at that crossing.
                if (std::getenv("SESSION_EXT_TRIM")) {
                    if (!lpA.count(ch.surfA)) lpA[ch.surfA] = face_loops_uv(A, ch.surfA);
                    if (!lpB.count(ch.surfB)) lpB[ch.surfB] = face_loops_uv(B, ch.surfB);
                    const auto& fA = lpA[ch.surfA];
                    const auto& fB = lpB[ch.surfB];
                    double p[4] = {ch.uvA[i0][0], ch.uvA[i0][1], ch.uvB[i0][0], ch.uvB[i0][1]};
                    double q2[4] = {prv[0], prv[1], prv[2], prv[3]};
                    bool inA0 = in_faces_uv(fA, p[0], p[1], 0.0);
                    bool inB0 = in_faces_uv(fB, p[2], p[3], 0.0);
                    double step3 = ch.p3[i0].distance(ch.p3[i1]);
                    bool crossed = false;
                    std::vector<std::array<double,4>> ext_pts;
                    for (int stp = 0; stp < 6 && !crossed; ++stp) {
                        double nx[4];
                        for (int k = 0; k < 4; ++k) nx[k] = p[k] + (p[k] - q2[k]);
                        nx[on_k] = bnds[on_k][on_s];
                        if (!correct7_pinned(ea, eb, nx[0], nx[1], nx[2], nx[3], on_k, conv_tol)) {
                            if (std::getenv("SESSION_SPLIT_DBG"))
                                std::fprintf(stderr, "[EXTTRIM-DBG] pinned corr FAIL sA=%d sB=%d end=%d stp=%d\n",
                                             ch.surfA, ch.surfB, end, stp);
                            break;
                        }
                        nx[on_k] = bnds[on_k][on_s];
                        double fw = 0;
                        for (int k = 0; k < 4; ++k) if (k != on_k) fw += (nx[k] - p[k]) * (p[k] - q2[k]);
                        if (fw <= 0) {
                            if (std::getenv("SESSION_SPLIT_DBG"))
                                std::fprintf(stderr, "[EXTTRIM-DBG] pinned BACKWARD sA=%d sB=%d end=%d stp=%d\n",
                                             ch.surfA, ch.surfB, end, stp);
                            break;
                        }
                        bool inA1 = in_faces_uv(fA, nx[0], nx[1], 0.0);
                        bool inB1 = in_faces_uv(fB, nx[2], nx[3], 0.0);
                        if (inA1 != inA0 || inB1 != inB0) {
                            double aa[4], bb[4];
                            for (int k = 0; k < 4; ++k) { aa[k] = p[k]; bb[k] = nx[k]; }
                            for (int it2 = 0; it2 < 24; ++it2) {
                                double mm[4];
                                for (int k = 0; k < 4; ++k) mm[k] = 0.5 * (aa[k] + bb[k]);
                                mm[on_k] = bnds[on_k][on_s];
                                if (!correct7_pinned(ea, eb, mm[0], mm[1], mm[2], mm[3], on_k, conv_tol)) break;
                                mm[on_k] = bnds[on_k][on_s];
                                bool mA = in_faces_uv(fA, mm[0], mm[1], 0.0);
                                bool mB = in_faces_uv(fB, mm[2], mm[3], 0.0);
                                if (mA == inA0 && mB == inB0) std::copy(mm, mm+4, aa);
                                else                          std::copy(mm, mm+4, bb);
                            }
                            for (int k = 0; k < 4; ++k) nx[k] = bb[k];
                            crossed = true;
                        }
                        ext_pts.push_back({nx[0], nx[1], nx[2], nx[3]});
                        for (int k = 0; k < 4; ++k) { q2[k] = p[k]; p[k] = nx[k]; }
                    }
                    if (crossed && !ext_pts.empty()) {
                        double total = 0;
                        Point prev3 = ch.p3[i0];
                        std::vector<Point> np3, nuvA, nuvB;
                        for (const auto& e4 : ext_pts) {
                            Vector S, Su, Sv; ea(e4[0], e4[1], S, Su, Sv);
                            Point pn(S[0], S[1], S[2]);
                            total += prev3.distance(pn);
                            prev3 = pn;
                            np3.push_back(pn);
                            nuvA.push_back(Point(e4[0], e4[1], 0.0));
                            nuvB.push_back(Point(e4[2], e4[3], 0.0));
                        }
                        if (total <= std::max(step3 * 6.0, weld_tol * 6.0)) {
                            for (size_t m2 = 0; m2 < np3.size(); ++m2) {
                                if (end == 0) {
                                    ch.p3.insert(ch.p3.begin(), np3[m2]);
                                    ch.uvA.insert(ch.uvA.begin(), nuvA[m2]);
                                    ch.uvB.insert(ch.uvB.begin(), nuvB[m2]);
                                } else {
                                    ch.p3.push_back(np3[m2]);
                                    ch.uvA.push_back(nuvA[m2]);
                                    ch.uvB.push_back(nuvB[m2]);
                                }
                            }
                            if (std::getenv("SESSION_SPLIT_DBG"))
                                std::fprintf(stderr, "[SCAF-EXTTRIM] PINNED sA=%d sB=%d end=%d pts=%zu len=%.4f\n",
                                             ch.surfA, ch.surfB, end, np3.size(), total);
                        }
                    } else if (std::getenv("SESSION_SPLIT_DBG") && (inA0 && inB0)) {
                        std::fprintf(stderr, "[EXTTRIM-DBG] pinned NOCROSS sA=%d sB=%d end=%d (inA=%d inB=%d)\n",
                                     ch.surfA, ch.surfB, end, inA0, inB0);
                    }
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
            if (best_k < 0) {
                // ---- interior stop: EXTEND TO TRIM BOUNDARY (SESSION_EXT_TRIM; OCCT
                // PutPaveOnCurve/ExtendedTolerance analog). The marcher stalled mid-face
                // (tangency/step failure at a graze), leaving the end short of the OTHER
                // kind of border: an operand's TRIM boundary (not a chart bound). The fed
                // cut then DANGLES inside that operand's face and its arrangement prunes
                // the whole segment => one-sided imprint (x20 seg29: 0.128 short of A's
                // 16|18 border; A lost the seg, B kept it 2-trim). March the true SSI
                // (correct7 steps) along the end tangent until the A- or B-trim state
                // flips, bisect the crossing, and END the chain exactly there -- the
                // pave stage then sees a real trim crossing on BOTH operands.
                if (!std::getenv("SESSION_EXT_TRIM")) continue;
                if (!lpA.count(ch.surfA)) lpA[ch.surfA] = face_loops_uv(A, ch.surfA);
                if (!lpB.count(ch.surfB)) lpB[ch.surfB] = face_loops_uv(B, ch.surfB);
                const auto& fA = lpA[ch.surfA];
                const auto& fB = lpB[ch.surfB];
                bool inA0 = in_faces_uv(fA, cur[0], cur[1], 0.0);
                bool inB0 = in_faces_uv(fB, cur[2], cur[3], 0.0);
                double step3 = ch.p3[i0].distance(ch.p3[i1]);
                double p[4] = {cur[0], cur[1], cur[2], cur[3]};
                double q[4] = {prv[0], prv[1], prv[2], prv[3]};
                bool crossed = false;
                std::vector<std::array<double,4>> ext_pts;
                for (int stp = 0; stp < 6 && !crossed; ++stp) {
                    double nx[4];
                    for (int k = 0; k < 4; ++k) nx[k] = p[k] + (p[k] - q[k]);
                    if (!correct7(ea, eb, nx[0], nx[1], nx[2], nx[3], conv_tol)) {
                        if (std::getenv("SESSION_SPLIT_DBG"))
                            std::fprintf(stderr, "[EXTTRIM-DBG] free corr FAIL sA=%d sB=%d end=%d stp=%d\n",
                                         ch.surfA, ch.surfB, end, stp);
                        break;
                    }
                    // corrector must keep marching FORWARD (a stall re-converges backward)
                    double fw = 0;
                    for (int k = 0; k < 4; ++k) fw += (nx[k] - p[k]) * (p[k] - q[k]);
                    if (fw <= 0) {
                        if (std::getenv("SESSION_SPLIT_DBG"))
                            std::fprintf(stderr, "[EXTTRIM-DBG] free BACKWARD sA=%d sB=%d end=%d stp=%d\n",
                                         ch.surfA, ch.surfB, end, stp);
                        break;
                    }
                    bool inA1 = in_faces_uv(fA, nx[0], nx[1], 0.0);
                    bool inB1 = in_faces_uv(fB, nx[2], nx[3], 0.0);
                    if (inA1 != inA0 || inB1 != inB0) {
                        // bisect the flip between p and nx (correct7 at each midpoint)
                        double aa[4], bb[4];
                        for (int k = 0; k < 4; ++k) { aa[k] = p[k]; bb[k] = nx[k]; }
                        for (int it = 0; it < 24; ++it) {
                            double mm[4];
                            for (int k = 0; k < 4; ++k) mm[k] = 0.5 * (aa[k] + bb[k]);
                            if (!correct7(ea, eb, mm[0], mm[1], mm[2], mm[3], conv_tol)) break;
                            bool mA = in_faces_uv(fA, mm[0], mm[1], 0.0);
                            bool mB = in_faces_uv(fB, mm[2], mm[3], 0.0);
                            if (mA == inA0 && mB == inB0) std::copy(mm, mm+4, aa);
                            else                          std::copy(mm, mm+4, bb);
                        }
                        for (int k = 0; k < 4; ++k) nx[k] = bb[k];
                        crossed = true;
                    }
                    ext_pts.push_back({nx[0], nx[1], nx[2], nx[3]});
                    for (int k = 0; k < 4; ++k) { q[k] = p[k]; p[k] = nx[k]; }
                }
                if (crossed && !ext_pts.empty()) {
                    double total = 0;
                    Point prev3 = ch.p3[i0];
                    std::vector<Point> np3, nuvA, nuvB;
                    bool ok2 = true;
                    for (const auto& e4 : ext_pts) {
                        Vector S, Su, Sv; ea(e4[0], e4[1], S, Su, Sv);
                        Point pn(S[0], S[1], S[2]);
                        total += prev3.distance(pn);
                        prev3 = pn;
                        np3.push_back(pn);
                        nuvA.push_back(Point(e4[0], e4[1], 0.0));
                        nuvB.push_back(Point(e4[2], e4[3], 0.0));
                    }
                    if (total > std::max(step3 * 6.0, weld_tol * 6.0)) ok2 = false;
                    if (ok2) {
                        for (size_t m2 = 0; m2 < np3.size(); ++m2) {
                            if (end == 0) {
                                ch.p3.insert(ch.p3.begin(), np3[m2]);
                                ch.uvA.insert(ch.uvA.begin(), nuvA[m2]);
                                ch.uvB.insert(ch.uvB.begin(), nuvB[m2]);
                            } else {
                                ch.p3.push_back(np3[m2]);
                                ch.uvA.push_back(nuvA[m2]);
                                ch.uvB.push_back(nuvB[m2]);
                            }
                        }
                        if (std::getenv("SESSION_SPLIT_DBG"))
                            std::fprintf(stderr, "[SCAF-EXTTRIM] sA=%d sB=%d end=%d pts=%zu len=%.4f\n",
                                         ch.surfA, ch.surfB, end, np3.size(), total);
                    }
                }
                continue;
            }
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
                            bool a1 = add_pave(chains[c1], (int)i, s, 2);
                            bool a2 = add_pave(chains[c2], (int)j, t, 2);
                            scaf.n_paves_xing += 1;
                            // TRUE CORNER: two sections crossing = a 3-surface point
                            // (shared surface + each chain's other surface). Solve it
                            // exactly and pin BOTH paves to the identical point --
                            // chord-lerp corners carry up to half-a-chord (~0.1) of sag
                            // and weld visibly off (the 'badly computed corners').
                            if ((a1 || a2) && c1 != c2) {
                                int o1 = side == 0 ? chains[c1].surfB : chains[c1].surfA;
                                int o2 = side == 0 ? chains[c2].surfB : chains[c2].surfA;
                                int sh = side == 0 ? chains[c1].surfA : chains[c1].surfB;
                                if (o1 != o2) {
                                    const NurbsSurface& S0 = side == 0 ? A.m_surfaces[sh] : B.m_surfaces[sh];
                                    const NurbsSurface& S1 = side == 0 ? B.m_surfaces[o1] : A.m_surfaces[o1];
                                    const NurbsSurface& S2 = side == 0 ? B.m_surfaces[o2] : A.m_surfaces[o2];
                                    SEval e0{&S0}, e1{&S1}, e2{&S2};
                                    const auto& ov1 = side == 0 ? chains[c1].uvB : chains[c1].uvA;
                                    const auto& ov2 = side == 0 ? chains[c2].uvB : chains[c2].uvA;
                                    Point shp = lerp(uv1[i], uv1[i+1], s);
                                    Point q1 = lerp(ov1[i], ov1[std::min(i+1, ov1.size()-1)], s);
                                    Point q2 = lerp(ov2[j], ov2[std::min(j+1, ov2.size()-1)], t);
                                    double x6[6] = {shp[0], shp[1], q1[0], q1[1], q2[0], q2[1]};
                                    if (refine_triple_point(e0, e1, e2, x6, conv_tol)) {
                                        Vector S, Su, Sv;
                                        e0(x6[0], x6[1], S, Su, Sv);
                                        Point p3r(S[0], S[1], S[2]);
                                        Point p3c = lerp(chains[c1].p3[i],
                                                         chains[c1].p3[std::min(i+1, chains[c1].p3.size()-1)], s);
                                        if (p3r.distance(p3c) < weld_tol * 6.0) {
                                            Point sh_uv(x6[0], x6[1], 0.0);
                                            Point o1_uv(x6[2], x6[3], 0.0);
                                            Point o2_uv(x6[4], x6[5], 0.0);
                                            if (a1) chains[c1].pave_fix[i + s] = side == 0
                                                ? std::array<Point,3>{p3r, sh_uv, o1_uv}
                                                : std::array<Point,3>{p3r, o1_uv, sh_uv};
                                            if (a2) chains[c2].pave_fix[j + t] = side == 0
                                                ? std::array<Point,3>{p3r, sh_uv, o2_uv}
                                                : std::array<Point,3>{p3r, o2_uv, sh_uv};
                                        }
                                    }
                                }
                            }
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
        // NEAREST match, not first-match: with a fat weld radius, first-match binds a
        // point to whichever cluster happened to be inserted earlier (order-dependent
        // junction smearing); nearest-match is deterministic in geometry.
        int best = -1; double bd = weld_tol;
        for (int vi = 0; vi < (int)scaf.vertices.size(); ++vi) {
            double d = scaf.vertices[vi].distance(p);
            if (d < bd) { bd = d; best = vi; }
        }
        if (best >= 0) return best;
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

        // Trim-crossing bisection for the interval keep-verdict. A pave interval is kept
        // only where the section lies inside BOTH operands' trims. Judging that from the
        // single midpoint is wrong whenever the interval STRADDLES a trim boundary that was
        // not paved: the whole interval is discarded, and the surviving chain then stops in
        // the middle of a face. That leaves a valence-1 end in the shared section network,
        // and the two operands' arrangements each close the resulting open network their own
        // way -- two different bridge curves across the same gap, which is exactly the naked
        // rim seen on y30/x20/z45 (their dangling scaffold vertices ARE the rim corners,
        // while the one watertight config, z30x20, has none). So: sample the interval, and
        // when the verdict changes along it, bisect to the crossing and keep the inside
        // part, ending the chain ON the boundary where it belongs.
        // OCCT keeps a section block when its midpoint is in-or-ON both faces at 3D
        // tolerance; our 1e-3-UV band drops whole grazing intervals (the y30 class:
        // drop(verdict) severs the network exactly where the section hugs a trim).
        // SESSION_VERDICT_EPS_MULT widens the VERDICT band only (pave/crossing bands
        // untouched).
        double eps_vm = 1.0;
        if (const char* vm2 = std::getenv("SESSION_VERDICT_EPS_MULT")) eps_vm = std::atof(vm2);
        double epsAv = epsA * eps_vm, epsBv = epsB * eps_vm;
        auto verdict_eps = [&](double t, double scale) {
            Point p, a, b;
            at(t, p, a, b);
            return in_faces_uv(loopsA[ch.surfA], a[0], a[1], epsAv * scale)
                && in_faces_uv(loopsB[ch.surfB], b[0], b[1], epsBv * scale);
        };
        auto verdict_at = [&](double t) { return verdict_eps(t, 1.0); };
        // IN-OR-ON QUORUM (SESSION_ON_QUORUM=<W>): OCCT keeps a pave block whose middle
        // is in-or-ON both faces at 3D tolerance. Analog: keep a whole interval when
        // EVERY sample is within the wide band (W x verdict eps) AND >=2 samples are
        // strictly inside -- grazing intervals that hug a trim survive, while pure
        // boundary-runs (0 strict-in = the coincident slivers that broke base at
        // VERDICT_EPS_MULT=30) still drop.
        double on_w = 0.0;
        if (const char* ow = std::getenv("SESSION_ON_QUORUM")) on_w = std::atof(ow);
        std::vector<std::pair<double, double>> keep_iv;
        std::vector<std::pair<double, double>> drop_iv;
        for (size_t k = 0; k + 1 < bps.size(); ++k) {
            double lo = bps[k], hi = bps[k+1];
            if (hi - lo < 1e-9) continue;
            const int NSMP = 9;
            std::vector<double> ts(NSMP);
            std::vector<char> vs(NSMP);
            for (int i = 0; i < NSMP; ++i) {
                ts[i] = lo + (hi - lo) * (i + 0.5) / NSMP;
                vs[i] = verdict_at(ts[i]) ? 1 : 0;
            }
            int nin = 0;
            for (char v : vs) nin += v;
            if (on_w > 1.0 && nin >= 2 && nin < NSMP) {
                int nwide = 0;
                for (int i = 0; i < NSMP; ++i) nwide += (vs[i] || verdict_eps(ts[i], on_w)) ? 1 : 0;
                if (nwide == NSMP) {
                    keep_iv.push_back({lo, hi});
                    if (std::getenv("SESSION_SPLIT_DBG"))
                        std::fprintf(stderr, "[SCAF-ONKEEP] sA=%d sB=%d [%.4f,%.4f] nin=%d/9 grazing kept\n",
                                     ch.surfA, ch.surfB, lo, hi, nin);
                    continue;
                }
            }
            if (nin == 0) {
                drop_iv.push_back({lo, hi});
                scaf.n_dropped_verdict += 1;
                if (std::getenv("SESSION_SPLIT_DBG")) {
                    Point pm, am, bm;
                    at((lo + hi) * 0.5, pm, am, bm);
                    std::fprintf(stderr, "[SCAF-DROP] sA=%d sB=%d ALL-OUT uvA(%.3f,%.3f) uvB(%.3f,%.3f)\n",
                                 ch.surfA, ch.surfB, am[0], am[1], bm[0], bm[1]);
                }
                continue;
            }
            if (nin == NSMP) { keep_iv.push_back({lo, hi}); continue; }
            // Mixed: recover the inside runs, with each end bisected onto the crossing.
            auto bisect = [&](double tin, double tout) {
                for (int it = 0; it < 40; ++it) {
                    double tm = 0.5 * (tin + tout);
                    if (verdict_at(tm)) tin = tm; else tout = tm;
                }
                return tin;
            };
            int i = 0;
            while (i < NSMP) {
                if (!vs[i]) { ++i; continue; }
                int j = i;
                while (j + 1 < NSMP && vs[j + 1]) ++j;
                double a_end = vs[0] && i == 0 ? lo : bisect(ts[i], i > 0 ? ts[i - 1] : lo);
                double b_end = (j == NSMP - 1) ? hi : bisect(ts[j], ts[j + 1]);
                if (b_end - a_end > 1e-9) keep_iv.push_back({a_end, b_end});
                i = j + 1;
            }
            if (std::getenv("SESSION_SPLIT_DBG"))
                std::fprintf(stderr, "[SCAF-SPLITV] sA=%d sB=%d interval [%.4f,%.4f] mixed %d/%d -> %zu run(s)\n",
                             ch.surfA, ch.surfB, lo, hi, nin, NSMP, keep_iv.size());
        }
        // CONNECTIVITY STUB (SESSION_CONN_STUB, T-junction materializer): a kept interval
        // whose pave end abuts a VERDICT-DROPPED interval extends a short tail into it.
        // The tail is out-of-trims, so the target arrangement prunes it as a dangler --
        // but only AFTER it nodes its crossings, which is exactly the T-junction split
        // the joining segment needs (the y30 class: the network was severed here).
        if (const char* cs = std::getenv("SESSION_CONN_STUB")) {
            double stub = std::atof(cs);
            if (stub <= 0) stub = 2.0;               // chain-index units (~2 chords)
            int nst = 0;
            for (auto& kv : keep_iv) {
                for (const auto& dv : drop_iv) {
                    if (std::abs(kv.second - dv.first) < 1e-9) {
                        kv.second = std::min(dv.second, kv.second + stub);
                        ++nst;
                    }
                    if (std::abs(kv.first - dv.second) < 1e-9) {
                        kv.first = std::max(dv.first, kv.first - stub);
                        ++nst;
                    }
                }
            }
            if (nst && std::getenv("SESSION_SPLIT_DBG"))
                std::fprintf(stderr, "[CONNSTUB] sA=%d sB=%d extended %d kept ends\n",
                             ch.surfA, ch.surfB, nst);
        }
        for (size_t k = 0; k < keep_iv.size(); ++k) {
            double lo = keep_iv[k].first, hi = keep_iv[k].second;
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
            // Experiment gate: the micro filter is a real anti-sliver rule, but at a grazing
            // contact many GENUINE section intervals are this short, and every dropped
            // interval is a missing rim segment on both operands (the hole class). Scale the
            // floor down to measure how much of the residual it accounts for.
            static const double s_micro_k = [] {
                const char* e = std::getenv("SESSION_SCAF_MICRO");
                double v = e ? std::atof(e) : 1.0;
                return v > 0.0 ? v : 1.0;
            }();
            if (ext < weld_tol * s_micro_k) { scaf.n_dropped_micro += 1; continue; }

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

    // ---- 4a2. SD-WELD: OCCT MakeSDVertices port (BOPAlgo_PaveFiller_1 PerformVV + Tools::MakeBlocks) ----
    // OCCT proves the tracer PRUNES dangling chords too (BOPAlgo_BuilderFace::PerformShapesToAvoid); the
    // reason its faces still split is that a section endpoint interior to a face is NEVER valence-1 in the
    // arrangement -- the DS-level PostTreatFF/MakeSDVertices runs BEFORE splitting and welds every
    // coincident section endpoint (across ALL surface pairs) into ONE shared valence>=2 node. This is that
    // pass: a TRANSITIVE union-find over ALL segment endpoints in 3D (not the mutual-nearest valence-1
    // pairing of the legacy bridge), with the aDist/2 tolerance bridge, then re-derive uvA/uvB for the
    // merged node by projecting the ONE shared 3D point onto each surface -- so both operands' footprints
    // reference the same node and staggered/dangling endpoints become impossible. Gate SESSION_SDWELD.
    if (std::getenv("SESSION_SDWELD")) {
        // VALENCE-1 ONLY: correct junctions (valence>=2) are already shared; re-averaging/re-projecting
        // them corrupts the exact network (base 35->127 faces). Only DANGLING ends (valence-1) are
        // separate-but-should-be-one, so only those are welded (transitive union-find over dangles).
        std::vector<int> val(scaf.vertices.size(), 0);
        for (const auto& sg : scaf.segments) {
            if (sg.v_start >= 0 && sg.v_start < (int)val.size()) val[sg.v_start] += 1;
            if (sg.v_end   >= 0 && sg.v_end   < (int)val.size()) val[sg.v_end]   += 1;
        }
        struct Nd { int seg, end, surfA, surfB, v; Point p; };
        std::vector<Nd> nd;
        for (int si = 0; si < (int)scaf.segments.size(); ++si) {
            const auto& sg = scaf.segments[si];
            if (sg.v_start == sg.v_end) continue;   // closed loop: no free ends
            if (sg.v_start >= 0 && sg.v_start < (int)val.size() && val[sg.v_start] == 1)
                nd.push_back({si, 0, sg.surfA, sg.surfB, sg.v_start, sg.p3.front()});
            if (sg.v_end >= 0 && sg.v_end < (int)val.size() && val[sg.v_end] == 1)
                nd.push_back({si, 1, sg.surfA, sg.surfB, sg.v_end,   sg.p3.back()});
        }
        int N = (int)nd.size();
        std::vector<int> uf(N); for (int i=0;i<N;++i) uf[i]=i;
        std::function<int(int)> ff = [&](int a){ while(uf[a]!=a){uf[a]=uf[uf[a]];a=uf[a];} return a; };
        auto uni = [&](int a,int b){ a=ff(a); b=ff(b); if(a!=b) uf[a]=b; };
        auto pd = [&](int surf, bool isA, const Point& p) -> double {
            if (surf < 0) return 1e300;
            const NurbsSurface& s = isA ? A.m_surfaces[surf] : B.m_surfaces[surf];
            auto [u,v,d] = Closest::surface_point(s, p); (void)u; (void)v; return d;
        };
        const double max_gap = 0.5;
        for (int i=0;i<N;++i) for (int j=i+1;j<N;++j) {
            if (ff(i)==ff(j)) continue;
            if (nd[i].v >= 0 && nd[i].v == nd[j].v) { uni(i,j); continue; }   // already one vertex
            double gap = nd[i].p.distance(nd[j].p);
            if (gap >= max_gap) continue;
            if (gap < weld_tol) { uni(i,j); continue; }
            // real junction: the fused point must project onto ALL four referenced surfaces (aDist/2 tol)
            Point T((nd[i].p[0]+nd[j].p[0])*0.5, (nd[i].p[1]+nd[j].p[1])*0.5, (nd[i].p[2]+nd[j].p[2])*0.5);
            double tj = std::max(weld_tol, gap*0.5);
            if (pd(nd[i].surfA,true,T)<=tj && pd(nd[i].surfB,false,T)<=tj &&
                pd(nd[j].surfA,true,T)<=tj && pd(nd[j].surfB,false,T)<=tj)
                uni(i,j);
        }
        std::map<int,std::vector<int>> comps;
        for (int i=0;i<N;++i) comps[ff(i)].push_back(i);
        int n_weld = 0;
        for (auto& c : comps) {
            if ((int)c.second.size() < 2) continue;
            // representative = mean of the component's 3D points (OCCT MakeVertex covering point)
            Point T(0,0,0);
            for (int i : c.second) T = Point(T[0]+nd[i].p[0], T[1]+nd[i].p[1], T[2]+nd[i].p[2]);
            double m = (double)c.second.size();
            T = Point(T[0]/m, T[1]/m, T[2]/m);
            int vm = -1;
            for (int i : c.second) if (nd[i].v >= 0) { vm = nd[i].v; break; }
            if (vm < 0) { scaf.vertices.push_back(T); vm = (int)scaf.vertices.size()-1; }
            else scaf.vertices[vm] = T;
            for (int i : c.second) {
                SectionSegment& sg = scaf.segments[nd[i].seg];
                // re-derive uvA/uvB by projecting the ONE shared 3D point onto THIS node's surfaces
                Point uvA = nd[i].end==0 ? sg.uvA.front() : sg.uvA.back();
                Point uvB = nd[i].end==0 ? sg.uvB.front() : sg.uvB.back();
                if (nd[i].surfA >= 0) { auto [u,v,d]=Closest::surface_point(A.m_surfaces[nd[i].surfA],T); (void)d; uvA=Point(u,v,0); }
                if (nd[i].surfB >= 0) { auto [u,v,d]=Closest::surface_point(B.m_surfaces[nd[i].surfB],T); (void)d; uvB=Point(u,v,0); }
                if (nd[i].end==0) { sg.p3.front()=T; sg.uvA.front()=uvA; sg.uvB.front()=uvB; sg.v_start=vm; }
                else              { sg.p3.back()=T;  sg.uvA.back()=uvA;  sg.uvB.back()=uvB;  sg.v_end=vm; }
            }
            ++n_weld;
        }
        if (std::getenv("SESSION_SPLIT_DBG") || std::getenv("SESSION_NT_DBG"))
            std::fprintf(stderr, "[SDWELD] endpoints=%d components-welded=%d\n", N, n_weld);
    }

    // ---- 4b. JUNCTION WELD (SSI-completeness / PostTreatFF closure) ----
    // A closed solid's section is a set of CLOSED loops, so every scaffold vertex must have
    // even valence. A valence-1 vertex is a DANGLING end: the SSI marcher terminated short of
    // a junction where the section crosses one operand's face border onto the adjacent face.
    // The section arrives at that junction from TWO surface pairs (e.g. A-face-2 x B-face-16
    // and A-face-3 x B-face-16), and each marcher undershoots it -- leaving two vertices
    // ~0.1-0.2 apart that should be ONE shared junction vertex. Left dangling, the arrangement
    // prunes them and each operand closes the open network its own way, producing the naked
    // rim (proven: the dangling-vertex coordinates ARE the rim corners; the one watertight
    // rotated config has zero valence-1 vertices). Repair = WELD the pair into one vertex at
    // their common junction and reproject each incident segment's endpoint onto its OWN
    // surfaces, so both segments meet there (valence 2) and the arrangement nodes instead of
    // prunes. A candidate weld is accepted only if the merged point projects onto EVERY
    // involved surface within tolerance -- a real junction -- so unrelated dangles are never
    // fused. Gated OFF by default (SESSION_BRIDGE) until validated.
    // Default-ON (SESSION_NO_BRIDGE opts out). Two repair modes, chosen by surface-pair sharing:
    //   CASE A (SAME surface pair): the SSI marcher terminated mid-face -> a genuine gap in ONE
    //     section curve. Re-march the missing sub-arc between the two dangling ends on the shared
    //     (surfA,surfB) pair (predictor UV step + correct7 corrector), VERIFYING every sample lies
    //     on BOTH surfaces AND inside BOTH operands' trims. Append it as a new SectionSegment
    //     welded to the two existing vertices -> valence 2, closure by construction. VERIFY-OR-
    //     REFUSE: if any sample leaves a surface or a trim (the fictitious straight bridge the OCCT
    //     oracle flags on y30 e122), do NOT synthesize -- the pair falls through to the residual.
    //   CASE B (CROSS surface pair): a junction undershoot -- two marchers stopped short of one
    //     shared 3-surface junction. Fuse the pair to a single vertex at the verified junction.
    if (std::getenv("SESSION_BRIDGE")) {   // opt-in: closes dangles but can expose one-sided imprints (z30)
        auto recompute_val = [&](std::vector<int>& val) {
            val.assign(scaf.vertices.size(), 0);
            for (const auto& seg : scaf.segments) {
                if (seg.v_start >= 0 && seg.v_start < (int)val.size()) val[seg.v_start] += 1;
                if (seg.v_end   >= 0 && seg.v_end   < (int)val.size()) val[seg.v_end]   += 1;
            }
        };
        std::vector<int> val0;
        recompute_val(val0);
        // dangling end: which segment, which end, the segment's surface pair, and the end's
        // parametric footprints (seeds for the march / junction solve)
        struct Dang { int v, seg, end, surfA, surfB; Point uvA, uvB; };
        std::vector<Dang> dang;
        for (int si = 0; si < (int)scaf.segments.size(); ++si) {
            const auto& seg = scaf.segments[si];
            for (int end = 0; end < 2; ++end) {
                int v = end == 0 ? seg.v_start : seg.v_end;
                if (v < 0 || v >= (int)val0.size() || val0[v] != 1) continue;
                dang.push_back({v, si, end, seg.surfA, seg.surfB,
                                end == 0 ? seg.uvA.front() : seg.uvA.back(),
                                end == 0 ? seg.uvB.front() : seg.uvB.back()});
            }
        }
        // mutually-nearest pairing over ALL dangles within a bounded 3D gap (a junction is a
        // cross-surface-pair meeting, so pairing must NOT require the same pair)
        const double max_gap = 0.5;
        std::vector<int> partner(dang.size(), -1);
        for (size_t i = 0; i < dang.size(); ++i) {
            double bd = max_gap; int bj = -1;
            for (size_t j = 0; j < dang.size(); ++j) {
                if (j == i || dang[j].v == dang[i].v) continue;
                double d = scaf.vertices[dang[i].v].distance(scaf.vertices[dang[j].v]);
                if (d < bd) { bd = d; bj = (int)j; }
            }
            partner[i] = bj;
        }
        auto proj_dist = [&](int surf, bool isA, const Point& p, Point& uv) -> double {
            const NurbsSurface& s = isA ? A.m_surfaces[surf] : B.m_surfaces[surf];
            auto [cu, cv, cd] = Closest::surface_point(s, p);
            uv = Point(cu, cv, 0.0);
            return cd;
        };
        auto ensure_loops = [&](int sa, int sb) {
            if (!loopsA.count(sa)) loopsA[sa] = face_loops_uv(A, sa);
            if (!loopsB.count(sb)) loopsB[sb] = face_loops_uv(B, sb);
        };
        int n_marched = 0, n_weld = 0, n_reject = 0;
        std::vector<bool> used(dang.size(), false);
        // ---- EF-SEEDED CONTINUATION MARCH (SESSION_EF_MARCH, PerformEF increment 2) ----
        // A dangle that sits ON the other operand's EDGE is an exact EF junction: the
        // section CONTINUES there as (same own-surface) x (the NEXT face across that
        // edge) -- a pair with no seg at all, which no weld/merge can conjure. March it
        // from the exact anchor; the new seg starts at the dangle vertex, closing its
        // valence with true geometry.
        if (std::getenv("SESSION_EF_MARCH")) {
            // edge -> adjacent surface indices, per operand
            auto edge_surfs = [](const BRep& OP) {
                std::map<int, std::set<int>> m;
                for (const auto& f : OP.m_faces)
                    for (int li : f.loop_indices) {
                        if (li < 0 || li >= (int)OP.m_loops.size()) continue;
                        for (int ti : OP.m_loops[li].trim_indices) {
                            if (ti < 0 || ti >= (int)OP.m_trims.size()) continue;
                            int e = OP.m_trims[ti].edge_index;
                            if (e >= 0) m[e].insert(f.surface_index);
                        }
                    }
                return m;
            };
            std::map<int, std::set<int>> eSA = edge_surfs(A), eSB = edge_surfs(B);
            int n_efm = 0;
            for (size_t i = 0; i < dang.size(); ++i) {
                if (used[i]) continue;
                const Dang di = dang[i];
                Point P0 = scaf.vertices[di.v];
                // find an other-operand edge through P0 and its far face
                auto find_cont = [&](const BRep& OP, const std::map<int, std::set<int>>& eS,
                                     int keep_surf) -> int {
                    for (const auto& kv : eS) {
                        const auto& E = OP.m_topology_edges[kv.first];
                        if (E.curve_3d_index < 0 || E.curve_3d_index >= (int)OP.m_curves_3d.size()) continue;
                        if (!kv.second.count(keep_surf)) continue;   // edge must border the CURRENT pair's face
                        const NurbsCurve& C = OP.m_curves_3d[E.curve_3d_index];
                        if (!C.is_valid()) continue;
                        double t = C.closest_parameter(P0);
                        if (C.point_at(t).distance(P0) > weld_tol * 0.5) continue;
                        for (int s2 : kv.second)
                            if (s2 != keep_surf) return s2;
                    }
                    return -1;
                };
                // continuation across B's edge: pair (di.surfA, sB2); or across A's edge:
                // pair (sA2, di.surfB)
                int sB2 = find_cont(B, eSB, di.surfB);
                int sA2 = sB2 < 0 ? find_cont(A, eSA, di.surfA) : -1;
                int mA = sB2 >= 0 ? di.surfA : sA2;
                int mB = sB2 >= 0 ? sB2 : (sA2 >= 0 ? di.surfB : -1);
                if (mA < 0 || mB < 0) continue;
                if (mA >= (int)A.m_surfaces.size() || mB >= (int)B.m_surfaces.size()) continue;
                const NurbsSurface& sa = A.m_surfaces[mA];
                const NurbsSurface& sb = B.m_surfaces[mB];
                SEval ea{&sa}, eb{&sb};
                ensure_loops(mA, mB);
                auto duaR = sa.domain(0); auto dvaR = sa.domain(1);
                auto dubR = sb.domain(0); auto dvbR = sb.domain(1);
                double epsA2 = std::min(duaR.second - duaR.first, dvaR.second - dvaR.first) * 1e-3;
                double epsB2 = std::min(dubR.second - dubR.first, dvbR.second - dvbR.first) * 1e-3;
                auto [ua0, va0, da0] = Closest::surface_point(sa, P0);
                auto [ub0, vb0, db0] = Closest::surface_point(sb, P0);
                if (da0 > weld_tol || db0 > weld_tol) continue;
                // tangent of the (sa, sb) intersection at P0
                Vector na = sa.normal_at(ua0, va0), nb = sb.normal_at(ub0, vb0);
                Vector T(na[1]*nb[2]-na[2]*nb[1], na[2]*nb[0]-na[0]*nb[2], na[0]*nb[1]-na[1]*nb[0]);
                double Tl = T.magnitude();
                if (Tl < 1e-10) continue;                 // tangential: not marchable here
                T = Vector(T[0]/Tl, T[1]/Tl, T[2]/Tl);
                // CONTINUATION-ANGLE GATE (OCCT CheckArgumentsToExtend, myMaxConcatAngle
                // = pi/6): the continuation across an edge cannot turn sharply -- its
                // tangent must align with the dangling chain's OUTWARD end tangent, and
                // only that direction is marched. Ungated EF-march regressed y30/z90 by
                // +9 each (spurious marches from dangles that are not true EF junctions).
                int sgn_lo = -1, sgn_hi = 1;
                if (!std::getenv("SESSION_NO_EFGATE")) {
                    const SectionSegment& dsg = scaf.segments[di.seg];
                    size_t nn = dsg.p3.size();
                    Point pe = di.end == 0 ? dsg.p3[0] : dsg.p3[nn-1];
                    Point pi2 = di.end == 0 ? dsg.p3[std::min<size_t>(1, nn-1)] : dsg.p3[nn >= 2 ? nn-2 : 0];
                    Vector dout(pe[0]-pi2[0], pe[1]-pi2[1], pe[2]-pi2[2]);
                    double dl = dout.magnitude();
                    if (dl > 1e-30) {
                        double alin = (T[0]*dout[0]+T[1]*dout[1]+T[2]*dout[2]) / dl;
                        if (std::abs(alin) < 0.866) continue;    // > pi/6 turn: not a continuation
                        if (alin > 0) sgn_lo = 1; else sgn_hi = -1;
                    }
                }
                double h = weld_tol;
                int best_len = 0;
                std::vector<Point> best_p3, best_ua, best_ub;
                for (int sgn = sgn_lo; sgn <= sgn_hi; sgn += 2) {
                    std::vector<Point> mp3{P0}, mua{Point(ua0, va0, 0)}, mub{Point(ub0, vb0, 0)};
                    double uA2 = ua0, vA2 = va0, uB2 = ub0, vB2 = vb0;
                    Point cur = P0;
                    for (int st = 0; st < 200; ++st) {
                        Point nxt(cur[0] + sgn*h*T[0], cur[1] + sgn*h*T[1], cur[2] + sgn*h*T[2]);
                        auto [ua1, va1, d1] = Closest::surface_point(sa, nxt, uA2 - 0.2, uA2 + 0.2, vA2 - 0.2, vA2 + 0.2);
                        auto [ub1, vb1, d2] = Closest::surface_point(sb, nxt, uB2 - 0.2, uB2 + 0.2, vB2 - 0.2, vB2 + 0.2);
                        uA2 = ua1; vA2 = va1; uB2 = ub1; vB2 = vb1;
                        if (!correct7(ea, eb, uA2, vA2, uB2, vB2, conv_tol)) break;
                        Point pA = sa.point_at(uA2, vA2), pB = sb.point_at(uB2, vB2);
                        if (pA.distance(pB) > conv_tol * 10.0) break;
                        if (!in_faces_uv(loopsA[mA], uA2, vA2, epsA2)) break;
                        if (!in_faces_uv(loopsB[mB], uB2, vB2, epsB2)) break;
                        cur = Point((pA[0]+pB[0])*0.5, (pA[1]+pB[1])*0.5, (pA[2]+pB[2])*0.5);
                        mp3.push_back(cur);
                        mua.push_back(Point(uA2, vA2, 0));
                        mub.push_back(Point(uB2, vB2, 0));
                        // refresh tangent
                        Vector na2 = sa.normal_at(uA2, vA2), nb2 = sb.normal_at(uB2, vB2);
                        Vector T2(na2[1]*nb2[2]-na2[2]*nb2[1], na2[2]*nb2[0]-na2[0]*nb2[2], na2[0]*nb2[1]-na2[1]*nb2[0]);
                        double T2l = T2.magnitude();
                        if (T2l > 1e-10) {
                            T2 = Vector(T2[0]/T2l, T2[1]/T2l, T2[2]/T2l);
                            if (T2[0]*T[0] + T2[1]*T[1] + T2[2]*T[2] < 0)
                                T2 = Vector(-T2[0], -T2[1], -T2[2]);
                            T = T2;
                        }
                    }
                    if ((int)mp3.size() > best_len) {
                        best_len = (int)mp3.size();
                        best_p3 = std::move(mp3); best_ua = std::move(mua); best_ub = std::move(mub);
                    }
                }
                if (best_len < 3) continue;
                // 3D length gate: a real continuation spans beyond tolerance
                double clen = 0;
                for (size_t k2 = 1; k2 < best_p3.size(); ++k2) clen += best_p3[k2-1].distance(best_p3[k2]);
                if (clen < weld_tol * 2.0) continue;
                SectionSegment cont;
                cont.seg_id = (int)scaf.segments.size();
                cont.surfA = mA; cont.surfB = mB;
                cont.v_start = di.v;
                cont.v_end = weld_vertex(best_p3.back());
                cont.p3 = std::move(best_p3);
                cont.uvA = std::move(best_ua);
                cont.uvB = std::move(best_ub);
                cont.p3.front() = scaf.vertices[di.v];
                scaf.segs_by_surfA[mA].push_back(cont.seg_id);
                scaf.segs_by_surfB[mB].push_back(cont.seg_id);
                scaf.segments.push_back(std::move(cont));
                used[i] = true;
                ++n_efm;
                if (std::getenv("SESSION_SPLIT_DBG") || std::getenv("SESSION_NT_DBG"))
                    std::fprintf(stderr, "[EFMARCH] v%d pair(%d,%d) len=%.4f pts=%d\n",
                                 di.v, mA, mB, clen, best_len);
            }
            if (n_efm && (std::getenv("SESSION_SPLIT_DBG") || std::getenv("SESSION_NT_DBG")))
                std::fprintf(stderr, "[EFMARCH] continuations=%d\n", n_efm);
        }
        for (size_t i = 0; i < dang.size(); ++i) {
            int j = partner[i];
            if (j < 0 || used[i] || used[(size_t)j] || partner[(size_t)j] != (int)i) continue;   // mutual only
            const Dang di = dang[i], dj = dang[(size_t)j];
            if (di.surfA < 0 || di.surfA >= (int)A.m_surfaces.size() ||
                di.surfB < 0 || di.surfB >= (int)B.m_surfaces.size() ||
                dj.surfA < 0 || dj.surfA >= (int)A.m_surfaces.size() ||
                dj.surfB < 0 || dj.surfB >= (int)B.m_surfaces.size()) continue;
            Point pi = scaf.vertices[di.v], pj = scaf.vertices[dj.v];
            double gap = pi.distance(pj);
            if (gap < weld_tol) { used[i] = used[(size_t)j] = true; continue; }  // already one vertex
            bool same_pair = (di.surfA == dj.surfA && di.surfB == dj.surfB);

            // SESSION_BRIDGE=2: weld + EF-march only. CASE-A same-pair re-march is the
            // measured source of the z30/z37/y30 regressions (one-sided imprint exposure);
            // mode 2 leaves same-pair dangles as residual and keeps the junction fuse.
            if (same_pair && std::atoi(std::getenv("SESSION_BRIDGE")) >= 2) continue;
            if (same_pair) {
                // ---- CASE A: local SSI re-march on the shared (surfA,surfB) pair ----
                // A midpoint FUSE is forbidden for same-pair dangles: the straight chord between
                // two ends of the SAME section curve lies OFF the curve (the fictitious y30 e122
                // bridge that sits inside B), and its midpoint still projects onto both surfaces so
                // the projection gate would wrongly accept it -> naked-worse (z30 22->30). So a
                // same-pair pair is closed ONLY by a verified re-march; if the march refuses, the
                // pair is left as residual (never midpoint-welded).
                const char* refuse = nullptr;
                if (di.seg == dj.seg) refuse = "same-seg";  // a segment's own two ends: ambiguous, skip
                std::vector<Point> bp3, bua, bub;
                if (!refuse) {
                    const NurbsSurface& sa = A.m_surfaces[di.surfA];
                    const NurbsSurface& sb = B.m_surfaces[di.surfB];
                    SEval ea{&sa}, eb{&sb};
                    auto duaR = sa.domain(0); auto dvaR = sa.domain(1);
                    auto dubR = sb.domain(0); auto dvbR = sb.domain(1);
                    double epsA = std::min(duaR.second - duaR.first, dvaR.second - dvaR.first) * 1e-3;
                    double epsB = std::min(dubR.second - dubR.first, dvbR.second - dvbR.first) * 1e-3;
                    ensure_loops(di.surfA, di.surfB);
                    const int NB = 12;
                    double uA = di.uvA[0], vA = di.uvA[1], uB = di.uvB[0], vB = di.uvB[1];
                    double sAu = (dj.uvA[0]-di.uvA[0])/NB, sAv = (dj.uvA[1]-di.uvA[1])/NB;
                    double sBu = (dj.uvB[0]-di.uvB[0])/NB, sBv = (dj.uvB[1]-di.uvB[1])/NB;
                    bp3.push_back(sa.point_at(uA, vA)); bua.push_back(Point(uA,vA,0)); bub.push_back(Point(uB,vB,0));
                    for (int s = 1; s <= NB && !refuse; ++s) {
                        uA += sAu; vA += sAv; uB += sBu; vB += sBv;   // predictor (constant UV step)
                        if (s < NB) {
                            if (!correct7(ea, eb, uA, vA, uB, vB, conv_tol)) { refuse = "newton"; break; }
                            Point pA = sa.point_at(uA, vA), pB = sb.point_at(uB, vB);
                            if (pA.distance(pB) > conv_tol * 10.0) { refuse = "resid3d"; break; }
                            if (!in_faces_uv(loopsA[di.surfA], uA, vA, epsA)) { refuse = "outA"; break; }
                            if (!in_faces_uv(loopsB[di.surfB], uB, vB, epsB)) { refuse = "outB"; break; }
                        }
                        bp3.push_back(sa.point_at(uA, vA)); bua.push_back(Point(uA,vA,0)); bub.push_back(Point(uB,vB,0));
                    }
                }
                if (!refuse && bp3.size() >= 2) {
                    // pin endpoints to the canonical welded vertices/footprints so the arrangement
                    // nodes them (exact-equal doubles) instead of adding a divergent overshoot stub
                    bp3.front() = scaf.vertices[di.v]; bua.front() = di.uvA; bub.front() = di.uvB;
                    bp3.back()  = scaf.vertices[dj.v]; bua.back()  = dj.uvA; bub.back()  = dj.uvB;
                    SectionSegment bridge;
                    bridge.seg_id = (int)scaf.segments.size();
                    bridge.surfA = di.surfA; bridge.surfB = di.surfB;
                    bridge.p3 = std::move(bp3); bridge.uvA = std::move(bua); bridge.uvB = std::move(bub);
                    bridge.v_start = di.v; bridge.v_end = dj.v;
                    scaf.segs_by_surfA[di.surfA].push_back(bridge.seg_id);
                    scaf.segs_by_surfB[di.surfB].push_back(bridge.seg_id);
                    scaf.segments.push_back(std::move(bridge));
                    used[i] = used[(size_t)j] = true; ++n_marched;
                    if (std::getenv("SESSION_SPLIT_DBG"))
                        std::fprintf(stderr, "[SCAF-MARCH] v%d..v%d gap=%.4f pair(%d,%d) MARCHED\n",
                                     di.v, dj.v, gap, di.surfA, di.surfB);
                } else if (std::getenv("SESSION_SPLIT_DBG")) {
                    std::fprintf(stderr, "[SCAF-MARCH-REJ] v%d..v%d gap=%.4f pair(%d,%d) refuse=%s\n",
                                 di.v, dj.v, gap, di.surfA, di.surfB, refuse ? refuse : "size");
                }
                continue;   // same-pair: closed by march or left residual -- NEVER midpoint-welded
            }

            // ---- CASE B: CROSS-pair junction fuse (midpoint T, projection-gated) ----
            // Only genuine cross-surface-pair undershoots reach here: two marchers stopped short of
            // one shared 3-surface junction, so the midpoint IS on the junction (both curves pass
            // through it) and the projection gate is meaningful.
            // PI/6 TRIPLE ANGLE GATE (OCCT ExtendTwoWLines/CheckArgumentsToExtend,
            // myMaxConcatAngle): the two chains must approach the junction HEAD-ON --
            // each outward end tangent within pi/6 of the gap vector, and the tangents
            // within pi/6 of head-on to each other. Kills the spurious welds that
            // fused unrelated dangles (the y30 +2 class under ungated CASE B).
            if (!std::getenv("SESSION_NO_EFGATE")) {
                auto out_tan = [&](const Dang& d) -> Vector {
                    const SectionSegment& sg = scaf.segments[d.seg];
                    size_t nn = sg.p3.size();
                    Point pe = d.end == 0 ? sg.p3[0] : sg.p3[nn-1];
                    Point pin = d.end == 0 ? sg.p3[std::min<size_t>(1, nn-1)] : sg.p3[nn >= 2 ? nn-2 : 0];
                    Vector v(pe[0]-pin[0], pe[1]-pin[1], pe[2]-pin[2]);
                    double m = v.magnitude();
                    return m > 1e-30 ? Vector(v[0]/m, v[1]/m, v[2]/m) : Vector(0, 0, 0);
                };
                Vector ti_ = out_tan(di), tj_ = out_tan(dj);
                Vector g(pj[0]-pi[0], pj[1]-pi[1], pj[2]-pi[2]);
                double gm = g.magnitude();
                if (gm > 1e-30) { g = Vector(g[0]/gm, g[1]/gm, g[2]/gm); }
                const double cosg = 0.866;   // pi/6
                double a1 = ti_[0]*g[0]+ti_[1]*g[1]+ti_[2]*g[2];
                double a2 = -(tj_[0]*g[0]+tj_[1]*g[1]+tj_[2]*g[2]);
                double a3 = -(ti_[0]*tj_[0]+ti_[1]*tj_[1]+ti_[2]*tj_[2]);
                bool head_on = a1 >= cosg && a2 >= cosg && a3 >= cosg;
                if (!head_on) {
                    // CORNER route: two DIFFERENT section curves meeting where the
                    // section crosses a face border -- the junction must lie ON an
                    // operand EDGE (that crossing is WHY the junction exists). This
                    // is what separates a real corner from the y30-class spurious
                    // pairing of unrelated dangles (which floats mid-face).
                    Point Tm((pi[0]+pj[0])*0.5, (pi[1]+pj[1])*0.5, (pi[2]+pj[2])*0.5);
                    double edge_band = std::max(weld_tol, gap * 0.5);
                    bool on_edge = false;
                    auto near_edge = [&](const BRep& OP) {
                        for (const auto& E2 : OP.m_topology_edges) {
                            if (E2.curve_3d_index < 0 || E2.curve_3d_index >= (int)OP.m_curves_3d.size()) continue;
                            const NurbsCurve& C2 = OP.m_curves_3d[E2.curve_3d_index];
                            if (!C2.is_valid()) continue;
                            double t2 = C2.closest_parameter(Tm);
                            if (C2.point_at(t2).distance(Tm) <= edge_band) return true;
                        }
                        return false;
                    };
                    on_edge = near_edge(A) || near_edge(B);
                    if (!on_edge) {
                        ++n_reject;
                        if (std::getenv("SESSION_SPLIT_DBG"))
                            std::fprintf(stderr, "[SCAF-WELD-ANG] v%d..v%d gap=%.4f cos(%.3f,%.3f,%.3f) no-edge REJ\n",
                                         di.v, dj.v, gap, a1, a2, a3);
                        continue;
                    }
                }
            }
            Point T((pi[0]+pj[0])*0.5, (pi[1]+pj[1])*0.5, (pi[2]+pj[2])*0.5);
            Point uvAi, uvBi, uvAj, uvBj;
            double dAi = proj_dist(di.surfA, true, T, uvAi);
            double dBi = proj_dist(di.surfB, false, T, uvBi);
            double dAj = proj_dist(dj.surfA, true, T, uvAj);
            double dBj = proj_dist(dj.surfB, false, T, uvBj);
            double tol_junc = std::max(weld_tol, gap * 0.5);    // midpoint of a real junction sits <= gap/2 off
            if (dAi > tol_junc || dBi > tol_junc || dAj > tol_junc || dBj > tol_junc) {
                ++n_reject;
                if (std::getenv("SESSION_SPLIT_DBG"))
                    std::fprintf(stderr, "[SCAF-WELD-REJ] v%d..v%d gap=%.4f proj(%.4f,%.4f,%.4f,%.4f) tol=%.4f\n",
                                 di.v, dj.v, gap, dAi, dBi, dAj, dBj, tol_junc);
                continue;
            }
            int vm = di.v;
            scaf.vertices[vm] = T;
            auto set_end = [&](const Dang& d, const Point& uvA, const Point& uvB) {
                SectionSegment& sg = scaf.segments[d.seg];
                if (d.end == 0) {
                    sg.p3.front() = T; sg.uvA.front() = uvA; sg.uvB.front() = uvB; sg.v_start = vm;
                } else {
                    sg.p3.back() = T;  sg.uvA.back() = uvA;  sg.uvB.back() = uvB;  sg.v_end = vm;
                }
            };
            set_end(di, uvAi, uvBi);
            set_end(dj, uvAj, uvBj);
            used[i] = used[(size_t)j] = true;
            ++n_weld;
            if (std::getenv("SESSION_SPLIT_DBG"))
                std::fprintf(stderr, "[SCAF-WELD] v%d+v%d -> vm%d T(%.3f,%.3f,%.3f) gap=%.4f\n",
                             di.v, dj.v, vm, T[0], T[1], T[2], gap);
        }
        std::vector<int> valf; recompute_val(valf);
        int residual = 0;
        for (int v : valf) if (v == 1) ++residual;
        scaf.n_bridge_marched = n_marched;
        scaf.n_bridge_welded = n_weld;
        scaf.n_bridge_residual = residual;
        if (std::getenv("SESSION_SPLIT_DBG") || std::getenv("SESSION_NT_DBG"))
            std::fprintf(stderr, "[SCAF-BRIDGE] dangling=%zu marched=%d welded=%d rejected=%d residual=%d\n",
                         dang.size(), n_marched, n_weld, n_reject, residual);
    }

    // ---- 4c. EF interference diagnostic (OCCT PerformEF, first increment) ----
    // Exact surface x other-operand-EDGE intersection points: coarse scan each edge
    // against each overlapping surface, Gauss-Newton polish E(t) = S(u,v). These ARE the
    // true junction anchors the dangling chain ends undershoot; the histogram of
    // dangle-to-EF distances decides the EF-anchored welding design.
    if (std::getenv("SESSION_EF_DIAG")) {
        auto ef_scan = [&](const BRep& SURFOP, const BRep& EDGEOP, bool surf_is_A,
                           std::vector<Point>& out) {
            for (int si = 0; si < (int)SURFOP.m_surfaces.size(); ++si) {
                const NurbsSurface& S = SURFOP.m_surfaces[si];
                for (const auto& E : EDGEOP.m_topology_edges) {
                    if (E.curve_3d_index < 0 || E.curve_3d_index >= (int)EDGEOP.m_curves_3d.size()) continue;
                    const NurbsCurve& C = EDGEOP.m_curves_3d[E.curve_3d_index];
                    if (!C.is_valid()) continue;
                    auto cd = C.domain();
                    double prevd = 1e300;
                    for (int k = 0; k <= 32; ++k) {
                        double t = cd.first + (cd.second - cd.first) * k / 32.0;
                        Point p = C.point_at(t);
                        auto pr = Closest::surface_point(S, p, 0.0, 0.0, 0.0, 0.0);
                        double u = std::get<0>(pr), v = std::get<1>(pr);
                        Point q = S.point_at(u, v);
                        double d = q.distance(p);
                        // local minimum near the surface -> Newton polish
                        if (d < weld_tol * 2.0 && d <= prevd) {
                            double tt = t, uu = u, vv = v;
                            for (int it = 0; it < 12; ++it) {
                                auto cder = C.evaluate(tt, 1);          // [P, P']
                                auto sder = S.evaluate(uu, vv, 1);      // [S, Sv, Su] (kernel order)
                                double r0 = cder[0][0]-sder[0][0], r1 = cder[0][1]-sder[0][1], r2 = cder[0][2]-sder[0][2];
                                // J columns: dC/dt, -dS/du, -dS/dv
                                double J[3][3] = {
                                    {cder[1][0], -sder[2][0], -sder[1][0]},
                                    {cder[1][1], -sder[2][1], -sder[1][1]},
                                    {cder[1][2], -sder[2][2], -sder[1][2]}};
                                // solve J * dx = -r (Cramer)
                                double det = J[0][0]*(J[1][1]*J[2][2]-J[1][2]*J[2][1])
                                           - J[0][1]*(J[1][0]*J[2][2]-J[1][2]*J[2][0])
                                           + J[0][2]*(J[1][0]*J[2][1]-J[1][1]*J[2][0]);
                                if (std::abs(det) < 1e-18) break;
                                auto solve1 = [&](int col) {
                                    double M[3][3];
                                    for (int a2 = 0; a2 < 3; ++a2)
                                        for (int b2 = 0; b2 < 3; ++b2)
                                            M[a2][b2] = (b2 == col) ? -((a2==0)?r0:(a2==1)?r1:r2) : J[a2][b2];
                                    return (M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1])
                                          - M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0])
                                          + M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0])) / det;
                                };
                                double dt = solve1(0), du = solve1(1), dv = solve1(2);
                                tt += dt; uu += du; vv += dv;
                                tt = std::min(std::max(tt, cd.first), cd.second);
                                if (std::abs(dt) + std::abs(du) + std::abs(dv) < 1e-14) break;
                            }
                            Point pe = C.point_at(tt);
                            Point qs = S.point_at(uu, vv);
                            if (pe.distance(qs) < conv_tol * 100.0) {
                                bool dup = false;
                                for (const auto& x : out) if (x.distance(pe) < weld_tol * 0.1) { dup = true; break; }
                                if (!dup) out.push_back(pe);
                            }
                        }
                        prevd = d;
                    }
                }
            }
            (void)surf_is_A;
        };
        std::vector<Point> efpts;
        ef_scan(A, B, true, efpts);
        ef_scan(B, A, false, efpts);
        // dangle-to-EF histogram
        std::vector<int> val(scaf.vertices.size(), 0);
        for (const auto& seg : scaf.segments) {
            if (seg.v_start >= 0) val[seg.v_start] += 1;
            if (seg.v_end >= 0) val[seg.v_end] += 1;
        }
        int nd = 0;
        for (size_t vi = 0; vi < val.size(); ++vi) {
            if (val[vi] != 1) continue;
            ++nd;
            double best = 1e300;
            for (const auto& x : efpts) best = std::min(best, x.distance(scaf.vertices[vi]));
            std::fprintf(stderr, "[EFDIAG] dangle v%zu dEF=%.5f\n", vi, best < 1e300 ? best : -1.0);
        }
        std::fprintf(stderr, "[EFDIAG] ef_points=%zu dangles=%d weld_tol=%.4f\n",
                     efpts.size(), nd, weld_tol);
    }

    // ---- 4d. CROSS-PAIR SEGMENT UNIFICATION (SESSION_SEG_UNIFY; OCCT PostTreatFF /
    // same-domain analog). Where the section GRAZES along one operand's face border,
    // BOTH adjacent face pairs march the same physical curve, minting two near-parallel
    // segments (x20: seg21 vs seg29, 0.1 apart at graze scale). Each operand then keeps
    // a DIFFERENT copy and the rims never mate. Cure: snap the shorter segment's samples
    // onto the longer master's polyline over the overlapping span (geometry becomes ONE),
    // split the master at the snapped end's fractional index (both networks node there),
    // and weld end vertices. Downstream identity machinery mates the now-identical copies.
    if (const char* su = std::getenv("SESSION_SEG_UNIFY")) {
        double bandm = std::atof(su);
        if (bandm <= 1.0) bandm = 5.0;
        double band = bandm * scaf.tol3;
        auto proj_poly = [&](const Point& q, const std::vector<Point>& pl, double& fidx) -> double {
            double bd2 = 1e300; fidx = 0.0;
            for (size_t j = 0; j + 1 < pl.size(); ++j) {
                double ex = pl[j+1][0]-pl[j][0], ey = pl[j+1][1]-pl[j][1], ez = pl[j+1][2]-pl[j][2];
                double L2 = ex*ex + ey*ey + ez*ez;
                double t = L2 > 1e-30 ? ((q[0]-pl[j][0])*ex + (q[1]-pl[j][1])*ey + (q[2]-pl[j][2])*ez) / L2 : 0.0;
                t = std::min(std::max(t, 0.0), 1.0);
                double dx = q[0]-pl[j][0]-t*ex, dy = q[1]-pl[j][1]-t*ey, dz = q[2]-pl[j][2]-t*ez;
                double d2 = dx*dx + dy*dy + dz*dz;
                if (d2 < bd2) { bd2 = d2; fidx = (double)j + t; }
            }
            return std::sqrt(bd2);
        };
        auto seg_len = [](const SectionSegment& s) {
            double L = 0;
            for (size_t k = 0; k + 1 < s.p3.size(); ++k) L += s.p3[k].distance(s.p3[k+1]);
            return L;
        };
        std::map<int, std::vector<double>> breaks;
        std::set<int> touched;                       // unify-affected vertex ids (weld scope)
        int n_unified = 0;
        int nvert0 = (int)scaf.vertices.size();
        int nseg0 = (int)scaf.segments.size();
        for (int i = 0; i < nseg0; ++i) {
            auto& Si = scaf.segments[i];
            if (Si.closed || (int)Si.p3.size() < 3) continue;
            double Li = seg_len(Si);
            for (int j = 0; j < nseg0; ++j) {
                if (j == i) continue;
                auto& Sj = scaf.segments[j];
                if (Sj.closed || (int)Sj.p3.size() < 3) continue;
                if (Si.surfA == Sj.surfA && Si.surfB == Sj.surfB) continue;  // same pair = one marcher
                // graze duplicates come from ADJACENT pairs: one operand surface SHARED
                // (v1 allowed any pair combo and snapped distinct nearby sections:
                // base cut 0->9, z30x20 0->47 naked)
                if (Si.surfA != Sj.surfA && Si.surfB != Sj.surfB) continue;
                double Lj = seg_len(Sj);
                if (Lj < Li || (Lj == Li && j > i)) continue;                // master = longer, ties by id
                int n = (int)Si.p3.size();
                std::vector<double> fid(n);
                std::vector<char> inb(n);
                for (int k = 0; k < n; ++k) {
                    inb[k] = proj_poly(Si.p3[k], Sj.p3, fid[k]) < band ? 1 : 0;
                    if (!inb[k]) continue;
                    // duplicates run PARALLEL along the graze; curves that merely pass
                    // near (junction crossings) meet at an angle -- exclude them
                    int k1 = std::min(k + 1, n - 1), k0 = std::max(k - 1, 0);
                    Point ti_(Si.p3[k1][0]-Si.p3[k0][0], Si.p3[k1][1]-Si.p3[k0][1], Si.p3[k1][2]-Si.p3[k0][2]);
                    int jj = std::min((int)fid[k], (int)Sj.p3.size() - 2);
                    Point tj_(Sj.p3[jj+1][0]-Sj.p3[jj][0], Sj.p3[jj+1][1]-Sj.p3[jj][1], Sj.p3[jj+1][2]-Sj.p3[jj][2]);
                    double ni = std::sqrt(ti_[0]*ti_[0]+ti_[1]*ti_[1]+ti_[2]*ti_[2]);
                    double nj = std::sqrt(tj_[0]*tj_[0]+tj_[1]*tj_[1]+tj_[2]*tj_[2]);
                    if (ni > 1e-30 && nj > 1e-30) {
                        double dt = (ti_[0]*tj_[0]+ti_[1]*tj_[1]+ti_[2]*tj_[2]) / (ni*nj);
                        if (std::abs(dt) < 0.95) inb[k] = 0;
                    }
                }
                // end-anchored run only (graze duplicates emanate from a shared junction)
                int lo = 0, hi = n - 1;
                while (lo < n && !inb[lo]) ++lo;
                while (hi >= 0 && !inb[hi]) --hi;
                if (lo != 0 && hi != n - 1) continue;                        // interior-only: skip
                int a = lo == 0 ? 0 : hi, b2 = lo == 0 ? hi : n - 1;        // covered run [a,b2]
                bool okrun = a <= b2 && b2 - a + 1 >= 3;
                for (int k = a; k <= b2 && okrun; ++k) if (!inb[k]) okrun = false;
                if (!okrun) continue;
                double runL = 0;
                for (int k = a; k < b2; ++k) runL += Si.p3[k].distance(Si.p3[k+1]);
                if (runL < 4.0 * scaf.tol3) continue;                       // junction touch, not a graze
                // snap covered samples onto the master polyline + reproject own-pair uv
                for (int k = a; k <= b2; ++k) {
                    int jj = std::min((int)fid[k], (int)Sj.p3.size() - 2);
                    double ft = fid[k] - jj;
                    Point np = lerp(Sj.p3[jj], Sj.p3[jj+1], ft);
                    Si.p3[k] = np;
                    auto pa = Closest::surface_point(A.m_surfaces[Si.surfA], np);
                    Si.uvA[k] = Point(std::get<0>(pa), std::get<1>(pa), 0.0);
                    auto pb = Closest::surface_point(B.m_surfaces[Si.surfB], np);
                    Si.uvB[k] = Point(std::get<0>(pb), std::get<1>(pb), 0.0);
                }
                // the DEEP end of the run (where Si stops mid-master) must become a node
                // of the master too: record the break at its fractional index
                int deep = lo == 0 ? b2 : a;
                double fdeep = fid[deep];
                double margin = 1.5;
                if (fdeep > margin && fdeep < (double)Sj.p3.size() - 1.0 - margin)
                    breaks[Sj.seg_id].push_back(fdeep);
                if (Si.v_start >= 0) touched.insert(Si.v_start);
                if (Si.v_end >= 0) touched.insert(Si.v_end);
                if (Sj.v_start >= 0) touched.insert(Sj.v_start);
                if (Sj.v_end >= 0) touched.insert(Sj.v_end);
                ++n_unified;
                if (std::getenv("SESSION_SPLIT_DBG") || std::getenv("SESSION_NT_DBG"))
                    std::fprintf(stderr, "[SEGUNIFY] seg%d(%d,%d) run[%d,%d]/%d L=%.3f -> master seg%d(%d,%d) fdeep=%.2f\n",
                                 Si.seg_id, Si.surfA, Si.surfB, a, b2, n, runL,
                                 Sj.seg_id, Sj.surfA, Sj.surfB, fdeep);
                break;                                                       // one master per segment
            }
        }
        if (!breaks.empty()) refine_scaffold_at_breaks(scaf, breaks);
        if (n_unified) {
            // end-vertex weld, SCOPED: only unify-touched ends + refine-minted split
            // vertices may merge (a global tol3 weld would fuse junctions the builder
            // deliberately kept apart under the decoupled rep tolerance).
            for (int v = nvert0; v < (int)scaf.vertices.size(); ++v) touched.insert(v);
            std::vector<int> vmapU(scaf.vertices.size());
            for (int v = 0; v < (int)scaf.vertices.size(); ++v) vmapU[v] = v;
            for (int v = 0; v < (int)scaf.vertices.size(); ++v) {
                if (vmapU[v] != v) continue;
                for (int w = v + 1; w < (int)scaf.vertices.size(); ++w) {
                    if (vmapU[w] != w) continue;
                    if (!touched.count(v) && !touched.count(w)) continue;
                    if (scaf.vertices[v].distance(scaf.vertices[w]) <= scaf.tol3)
                        vmapU[w] = v;
                }
            }
            for (auto& s : scaf.segments) {
                if (s.v_start >= 0) s.v_start = vmapU[s.v_start];
                if (s.v_end >= 0) s.v_end = vmapU[s.v_end];
            }
            if (std::getenv("SESSION_SPLIT_DBG") || std::getenv("SESSION_NT_DBG"))
                std::fprintf(stderr, "[SEGUNIFY] unified=%d breaks=%zu\n", n_unified, breaks.size());
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

int refine_scaffold_at_breaks(SectionScaffold& scaf,
                              const std::map<int, std::vector<double>>& breaks) {
    // Fractional-index weld radius for discovered breakpoints. Ends reported within this
    // distance of a segment end are the segment's own ends (arrangement noise, stub clamps),
    // not new crossings; two reports within it are the same crossing seen from both operands.
    // It MUST match the whole-segment tolerance used when the lift decides to emit an alias
    // key (brep.cpp, `whole_seg`: fa < 1e-2 && fb > nCh-1-1e-2). A larger value here would
    // open a dead band -- a run starting at fa = 0.03 would be too far in to be keyed as a
    // whole segment, yet be dismissed here as "already at the end" and never split, so that
    // segment could never acquire identity by either route.
    const double eps_f = 1e-2;
    int n_new = 0, n_closed_skip = 0;
    auto weld_vertex = [&](const Point& p) {
        int best = -1; double bd = scaf.tol3;   // nearest-match (see build weld_vertex)
        for (size_t i = 0; i < scaf.vertices.size(); ++i) {
            double d = scaf.vertices[i].distance(p);
            if (d < bd) { bd = d; best = (int)i; }
        }
        if (best >= 0) return best;
        scaf.vertices.push_back(p);
        return (int)scaf.vertices.size() - 1;
    };
    // Linear interpolation IN CHAIN-INDEX SPACE -- the one operation that guarantees both
    // operands land on the same point: p3/uvA/uvB are index-corresponded, so splitting at
    // index t splits the 3D curve and BOTH parametric footprints at the same place.
    auto lerp_at = [](const std::vector<Point>& a, double t, int n) {
        int k = std::min((int)t, n - 2);
        if (k < 0) k = 0;
        double u = t - k;
        return Point(a[k][0] + (a[k + 1][0] - a[k][0]) * u,
                     a[k][1] + (a[k + 1][1] - a[k][1]) * u,
                     a[k][2] + (a[k + 1][2] - a[k][2]) * u);
    };
    int n0 = (int)scaf.segments.size();
    std::vector<SectionSegment> extra;
    for (const auto& kv : breaks) {
        int sid = kv.first;
        if (sid < 0 || sid >= n0) continue;
        int nCh = (int)scaf.segments[sid].p3.size();
        if (nCh < 3) continue;
        if (scaf.segments[sid].closed || scaf.segments[sid].v_start == scaf.segments[sid].v_end) {
            ++n_closed_skip;     // wrap splitting needs a seam choice; left to the safety net
            continue;
        }
        std::vector<double> f;
        for (double v : kv.second)
            if (v > eps_f && v < (double)(nCh - 1) - eps_f) f.push_back(v);
        if (f.empty()) continue;
        std::sort(f.begin(), f.end());
        // Cluster to the MEAN of each group (the same rule normalize_section_blocks uses),
        // so a crossing reported slightly differently by the two operands lands on one pave.
        std::vector<double> w;
        {
            double lo = f[0], sum = f[0];
            int cnt = 1;
            for (size_t i = 1; i <= f.size(); ++i) {
                if (i == f.size() || f[i] - lo > eps_f) {
                    w.push_back(sum / cnt);
                    if (i == f.size()) break;
                    lo = f[i]; sum = 0.0; cnt = 0;
                }
                lo = f[i]; sum += f[i]; ++cnt;
            }
        }
        if (w.empty()) continue;
        const SectionSegment& s = scaf.segments[sid];
        if ((int)s.uvA.size() != nCh || (int)s.uvB.size() != nCh) continue;   // lockstep invariant
        // Reject any breakpoint that would carve off a piece shorter than the 3D weld
        // tolerance: both of its ends would weld to the SAME scaffold vertex, which the
        // splitter reads as a CLOSED segment (v_start == v_end) and routes down the
        // full-wrap path -- losing exactly the identity this refinement exists to create.
        std::vector<double> alen(nCh, 0.0);
        for (int k = 1; k < nCh; ++k) alen[k] = alen[k-1] + s.p3[k-1].distance(s.p3[k]);
        auto arc_at = [&](double t) {
            t = std::min(std::max(t, 0.0), (double)(nCh - 1));
            int k = std::min((int)t, nCh - 2);
            return alen[k] + (alen[k+1] - alen[k]) * (t - k);
        };
        std::vector<double> bnds;
        bnds.push_back(0.0);
        for (double v : w)
            if (arc_at(v) - arc_at(bnds.back()) > scaf.tol3) bnds.push_back(v);
        if (bnds.size() < 2 || arc_at((double)(nCh - 1)) - arc_at(bnds.back()) <= scaf.tol3)
            continue;   // nothing splittable without creating a micro piece
        bnds.push_back((double)(nCh - 1));
        std::vector<SectionSegment> parts;
        bool ok = true;
        for (size_t b = 0; b + 1 < bnds.size() && ok; ++b) {
            double lo = bnds[b], hi = bnds[b + 1];
            SectionSegment t;
            t.surfA = s.surfA;
            t.surfB = s.surfB;
            t.closed = false;
            bool at_start = lo <= 1e-12;
            t.p3.push_back(at_start ? s.p3.front() : lerp_at(s.p3, lo, nCh));
            t.uvA.push_back(at_start ? s.uvA.front() : lerp_at(s.uvA, lo, nCh));
            t.uvB.push_back(at_start ? s.uvB.front() : lerp_at(s.uvB, lo, nCh));
            for (int k = (int)std::ceil(lo + 1e-9); k <= (int)std::floor(hi - 1e-9) && k < nCh; ++k) {
                t.p3.push_back(s.p3[k]);
                t.uvA.push_back(s.uvA[k]);
                t.uvB.push_back(s.uvB[k]);
            }
            // The trailing sample at hi == nCh-1 must be the STORED endpoint, bit for bit.
            // lerp_at computes a[n-2] + (a[n-1]-a[n-2])*1.0, which is not IEEE-identical to
            // a[n-1]; phase 4 of build_section_scaffold canonicalizes junction UVs precisely
            // so that segments meeting at a vertex compare equal as exact doubles, and the
            // splitter's junction detection keys a std::map on those doubles. One ULP of
            // drift there makes a junction look like two distinct ends, so both incident
            // cuts get overshoot stubs pointing different ways and the weld is destroyed.
            bool at_end = hi >= (double)(nCh - 1) - 1e-12;
            t.p3.push_back(at_end ? s.p3.back() : lerp_at(s.p3, hi, nCh));
            t.uvA.push_back(at_end ? s.uvA.back() : lerp_at(s.uvA, hi, nCh));
            t.uvB.push_back(at_end ? s.uvB.back() : lerp_at(s.uvB, hi, nCh));
            if (t.p3.size() < 2) { ok = false; break; }
            parts.push_back(std::move(t));
        }
        if (!ok || parts.size() < 2) continue;
        int v0 = s.v_start, v1 = s.v_end, surfA = s.surfA, surfB = s.surfB;
        for (size_t b = 0; b < parts.size(); ++b) {
            parts[b].v_start = (b == 0) ? v0 : weld_vertex(parts[b].p3.front());
            parts[b].v_end = (b + 1 == parts.size()) ? v1 : weld_vertex(parts[b].p3.back());
        }
        // The first part keeps the original seg_id so existing per-surface lists stay valid;
        // the remainder are appended with fresh ids.
        SectionSegment& dst = scaf.segments[sid];
        dst.p3 = parts[0].p3;
        dst.uvA = parts[0].uvA;
        dst.uvB = parts[0].uvB;
        dst.v_start = parts[0].v_start;
        dst.v_end = parts[0].v_end;
        dst.closed = false;
        for (size_t b = 1; b < parts.size(); ++b) {
            parts[b].surfA = surfA;
            parts[b].surfB = surfB;
            extra.push_back(std::move(parts[b]));
        }
    }
    for (auto& t : extra) {
        t.seg_id = (int)scaf.segments.size();
        if (t.surfA >= 0 && t.surfA < (int)scaf.segs_by_surfA.size())
            scaf.segs_by_surfA[t.surfA].push_back(t.seg_id);
        if (t.surfB >= 0 && t.surfB < (int)scaf.segs_by_surfB.size())
            scaf.segs_by_surfB[t.surfB].push_back(t.seg_id);
        scaf.segments.push_back(std::move(t));
        ++n_new;
    }
    if (std::getenv("SESSION_NT_DBG") && (n_new || n_closed_skip))
        std::fprintf(stderr, "[REFINE] split %d new segments, skipped %d closed\n",
                     n_new, n_closed_skip);
    return n_new;
}

// BOP2 M1 — mint each section pave-block ONCE as a shared topological edge whose endpoints are the
// scaffold's 3D-welded pave vertices. Both operands' arrangements will REFERENCE these edges (Phase 4),
// so the two copies + alias/sew machinery disappear. Mirrors append_face's 3D/vertex construction
// (brep.cpp add_curve_3d/add_edge) but keyed by welded scaffold identity, not a coordinate hash.
SharedEdgePool build_shared_edge_pool(const SectionScaffold& scaf) {
    SharedEdgePool P;
    P.vert_tv.assign(scaf.vertices.size(), -1);
    for (size_t i = 0; i < scaf.vertices.size(); ++i) {
        int pi = P.arena.add_vertex(scaf.vertices[i]);
        BRepVertex tv; tv.point_index = pi;
        P.arena.m_topology_vertices.push_back(tv);
        P.vert_tv[i] = (int)P.arena.m_topology_vertices.size() - 1;   // topology-vertex index
    }
    P.seg_edge.assign(scaf.segments.size(), -1);
    for (const auto& s : scaf.segments) {
        if (s.seg_id < 0 || (int)s.p3.size() < 2) continue;
        if (s.v_start < 0 || s.v_start >= (int)P.vert_tv.size() ||
            s.v_end   < 0 || s.v_end   >= (int)P.vert_tv.size()) continue;
        NurbsCurve c3d = NurbsCurve::create(false, 1, s.p3);
        if (!c3d.is_valid()) continue;
        int ci3 = P.arena.add_curve_3d(c3d);
        int v0 = P.vert_tv[s.v_start], v1 = P.vert_tv[s.v_end];   // SHARED endpoints
        int E = P.arena.add_edge(ci3, v0, v1);
        if (s.seg_id < (int)P.seg_edge.size()) P.seg_edge[s.seg_id] = E;
        P.block_edge[{s.seg_id, 0}] = E;
    }
    return P;
}

} // namespace session_cpp
