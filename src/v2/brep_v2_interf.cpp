// V2 INTERFERENCE STAGES — implementation. See brep_v2_interf.h for the contract and for the
// three documented divergences from OCCT (D1 EE recursion, D2 EF signed residual, D3 tolerances).

#include "brep_v2_interf.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <queue>
#include <set>

namespace session_cpp {
namespace v2int {

///////////////////////////////////////////////////////////////////////////////////////////
// 0. Small helpers. Everything here is model space unless the name says uv/param.
///////////////////////////////////////////////////////////////////////////////////////////

inline double v2i_dist3(const Point& a, const Point& b) {
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
inline double v2i_dot3(const Vector& a, const Vector& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline Vector v2i_sub3(const Point& a, const Point& b) {
    return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
inline Point v2i_as_point(const Vector& v) { return Point(v[0], v[1], v[2]); }

/// Sample count for a curve: proportional to its span count, clamped. A line gets `lo`; a
/// 64-point interpolated seam gets `hi`.
int v2i_sample_count(const NurbsCurve& c, int mult, int lo, int hi) {
    const int spans = std::max(1, c.cv_count() - c.degree());
    return std::min(hi, std::max(lo, mult * spans));
}

/// Curve point + first derivative in one evaluation.
void v2i_curve_pd(const NurbsCurve& c, double t, Point& p, Vector& d1) {
    const std::vector<Vector> ev = c.evaluate(t, 1);
    if (ev.empty()) { p = Point(0, 0, 0); d1 = Vector(0, 0, 0); return; }
    p = v2i_as_point(ev[0]);
    d1 = ev.size() > 1 ? ev[1] : Vector(0, 0, 0);
}


///////////////////////////////////////////////////////////////////////////////////////////
// 1. Face view
///////////////////////////////////////////////////////////////////////////////////////////

V2State V2FaceView::classify(double u, double v, double du, double dv) const {
    if (loops.empty()) {
        // No trims: the whole parameter rectangle is the face.
        const bool on = std::fabs(u - umin) <= du || std::fabs(u - umax) <= du ||
                        std::fabs(v - vmin) <= dv || std::fabs(v - vmax) <= dv;
        if (on) return V2State::On;
        return (u > umin && u < umax && v > vmin && v < vmax) ? V2State::In : V2State::Out;
    }
    const double su = (du > 0) ? du : 1e-12;
    const double sv = (dv > 0) ? dv : 1e-12;
    bool inside = false;
    double best_on = 1e300;
    for (const std::vector<std::array<double, 2>>& lp : loops) {
        const size_t n = lp.size();
        if (n < 2) continue;
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            const double ui = lp[i][0], vi = lp[i][1];
            const double uj = lp[j][0], vj = lp[j][1];
            // ON band: distance to the segment in the anisotropic metric (du, dv).
            const double au = (ui - uj) / su, av = (vi - vj) / sv;
            const double pu = (u - uj) / su, pv = (v - vj) / sv;
            const double len2 = au * au + av * av;
            double h = 0.0;
            if (len2 > 0) h = std::max(0.0, std::min(1.0, (pu * au + pv * av) / len2));
            const double ru = pu - h * au, rv = pv - h * av;
            best_on = std::min(best_on, std::sqrt(ru * ru + rv * rv));
            // Even-odd crossing of the +u ray.
            if ((vi > v) != (vj > v)) {
                const double x = uj + (v - vj) / (vi - vj) * (ui - uj);
                if (u < x) inside = !inside;
            }
        }
    }
    if (best_on <= 1.0) return V2State::On;
    return inside ? V2State::In : V2State::Out;
}

V2FaceView v2_make_face_view(const BRep& b, int local_face, int arena_face, int loop_samples,
                             int grid) {
    V2FaceView f;
    f.face_index = arena_face;
    if (local_face < 0 || local_face >= (int)b.m_faces.size()) return f;
    const BRepFace& bf = b.m_faces[local_face];
    if (bf.surface_index < 0 || bf.surface_index >= (int)b.m_surfaces.size()) return f;
    f.surface = &b.m_surfaces[bf.surface_index];
    const std::pair<double, double> du = f.surface->domain(0);
    const std::pair<double, double> dv = f.surface->domain(1);

    double lo_u = 1e300, hi_u = -1e300, lo_v = 1e300, hi_v = -1e300;
    for (int li : bf.loop_indices) {
        if (li < 0 || li >= (int)b.m_loops.size()) continue;
        std::vector<std::array<double, 2>> poly;
        for (int ti : b.m_loops[li].trim_indices) {
            if (ti < 0 || ti >= (int)b.m_trims.size()) continue;
            const BRepTrim& tr = b.m_trims[ti];
            if (tr.curve_2d_index < 0 || tr.curve_2d_index >= (int)b.m_curves_2d.size()) continue;
            const NurbsCurve& c2 = b.m_curves_2d[tr.curve_2d_index];
            const std::pair<double, double> dm = c2.domain();
            const int ns = std::max(2, loop_samples * std::max(1, c2.cv_count() - c2.degree()));
            // CHAIN HEAD-TO-TAIL BY GEOMETRY, never by BRepTrim::reversed. In this kernel the
            // pcurves are already stored in loop-traversal order and `reversed` describes the
            // EDGE's orientation, not the pcurve's (brep.cpp:388-397 stores c2d_top as
            // (u1,v1)->(u0,v1) AND sets reversed=true). Honouring the flag turned the trim
            // rectangle into a bow-tie, which inverts the even-odd parity over part of the
            // domain — measured: 17 of 297 box-edge/sphere piercings silently classified OUT.
            const Point ps = c2.point_at(dm.first), pe = c2.point_at(dm.second);
            bool forward = true;
            if (!poly.empty()) {
                const double du0 = std::fabs(poly.back()[0] - ps[0]) +
                                   std::fabs(poly.back()[1] - ps[1]);
                const double du1 = std::fabs(poly.back()[0] - pe[0]) +
                                   std::fabs(poly.back()[1] - pe[1]);
                forward = (du0 <= du1);
            }
            for (int k = 0; k < ns; ++k) {
                const double a = (double)k / (double)ns;
                const double t = forward ? dm.first + (dm.second - dm.first) * a
                                         : dm.second + (dm.first - dm.second) * a;
                const Point p = c2.point_at(t);
                poly.push_back({p[0], p[1]});
                lo_u = std::min(lo_u, p[0]); hi_u = std::max(hi_u, p[0]);
                lo_v = std::min(lo_v, p[1]); hi_v = std::max(hi_v, p[1]);
            }
        }
        if (poly.size() >= 3) f.loops.push_back(poly);
    }
    if (lo_u > hi_u) { lo_u = du.first; hi_u = du.second; lo_v = dv.first; hi_v = dv.second; }
    // The adaptor rectangle: the trims' extent, never outside the surface's own domain.
    f.umin = std::max(du.first, lo_u);
    f.umax = std::min(du.second, hi_u);
    f.vmin = std::max(dv.first, lo_v);
    f.vmax = std::min(dv.second, hi_v);
    if (!(f.umax > f.umin)) { f.umin = du.first; f.umax = du.second; }
    if (!(f.vmax > f.vmin)) { f.vmin = dv.first; f.vmax = dv.second; }

    f.closed_u = f.surface->is_closed(0);
    f.closed_v = f.surface->is_closed(1);
    f.nu = std::max(5, grid);
    f.nv = std::max(5, grid);
    f.gu.resize(f.nu);
    f.gv.resize(f.nv);
    for (int i = 0; i < f.nu; ++i)
        f.gu[i] = f.umin + (f.umax - f.umin) * (double)i / (double)(f.nu - 1);
    for (int j = 0; j < f.nv; ++j)
        f.gv[j] = f.vmin + (f.vmax - f.vmin) * (double)j / (double)(f.nv - 1);
    f.gp.resize((size_t)f.nu * f.nv);
    for (int i = 0; i < f.nu; ++i)
        for (int j = 0; j < f.nv; ++j)
            f.gp[(size_t)i * f.nv + j] = f.surface->point_at(f.gu[i], f.gv[j]);
    return f;
}

///////////////////////////////////////////////////////////////////////////////////////////
// 2. Projections
///////////////////////////////////////////////////////////////////////////////////////////

int v2_proj_pc(const NurbsCurve& c, double t0, double t1, const Point& p, double& t,
               double& dist) {
    t = t0;
    dist = 1e300;
    if (!(t1 > t0)) return 0;
    const int n = v2i_sample_count(c, 8, 64, 400);
    const double span = t1 - t0;
    std::vector<double> ts((size_t)n + 1), gs((size_t)n + 1);
    for (int i = 0; i <= n; ++i) {
        ts[i] = t0 + span * (double)i / (double)n;
        Point pc;
        Vector d1;
        v2i_curve_pd(c, ts[i], pc, d1);
        gs[i] = v2i_dot3(v2i_sub3(pc, p), d1);
    }
    // Interior stationary points of |P - C(t)|^2 only. NO CLAMPING (kb/port_02 §2.3.3).
    const double eps_t = span * 1e-12;
    int count = 0;
    for (int i = 0; i < n; ++i) {
        double a = ts[i], b = ts[i + 1], ga = gs[i], gb = gs[i + 1];
        if (ga == 0.0 && i > 0) {
            const double d = v2i_dist3(c.point_at(a), p);
            ++count;
            if (d < dist) { dist = d; t = a; }
            continue;
        }
        if (!(ga * gb < 0.0)) continue;
        for (int it = 0; it < 100 && (b - a) > 1e-15 * (1.0 + std::fabs(a)); ++it) {
            const double m = 0.5 * (a + b);
            Point pc;
            Vector d1;
            v2i_curve_pd(c, m, pc, d1);
            const double gm = v2i_dot3(v2i_sub3(pc, p), d1);
            if (gm == 0.0) { a = b = m; break; }
            if ((ga < 0) == (gm < 0)) { a = m; ga = gm; } else { b = m; gb = gm; }
        }
        const double r = 0.5 * (a + b);
        if (r <= t0 + eps_t || r >= t1 - eps_t) continue;
        const double d = v2i_dist3(c.point_at(r), p);
        ++count;
        if (d < dist) { dist = d; t = r; }
    }
    // OCCT appends an ENDPOINT only when its squared distance is below Precision::SquareConfusion
    // (Extrema_GGExtPC.hxx:474-501). That is the entire endpoint story; there is no fallback.
    for (int k = 0; k < 2; ++k) {
        const double te = k ? t1 : t0;
        const double d = v2i_dist3(c.point_at(te), p);
        if (d * d < 1e-14) {
            ++count;
            if (d < dist) { dist = d; t = te; }
        }
    }
    if (count == 0) { dist = 1e300; t = t0; }
    return count;
}


/// Newton on (S-P).Su = (S-P).Sv = 0, clamped to the face's rectangle.
bool v2i_newton_ps(const V2FaceView& f, const Point& p, double u, double v, double& uo, double& vo,
               double& dist) {
    const NurbsSurface& s = *f.surface;
    const double ru = f.umax - f.umin, rv = f.vmax - f.vmin;
    u = std::min(f.umax, std::max(f.umin, u));
    v = std::min(f.vmax, std::max(f.vmin, v));
    Point cur = s.point_at(u, v);
    double best = v2i_dist3(cur, p);
    uo = u; vo = v; dist = best;
    for (int it = 0; it < 60; ++it) {
        const std::vector<Vector> ev = s.evaluate(u, v, 2);
        if (ev.size() < 6) break;
        const Vector S = ev[0], Sv = ev[1], Svv = ev[2], Su = ev[3], Suv = ev[4], Suu = ev[5];
        const Vector D(S[0] - p[0], S[1] - p[1], S[2] - p[2]);
        const double f1 = v2i_dot3(D, Su), f2 = v2i_dot3(D, Sv);
        const double J11 = v2i_dot3(Su, Su) + v2i_dot3(D, Suu);
        const double J12 = v2i_dot3(Su, Sv) + v2i_dot3(D, Suv);
        const double J22 = v2i_dot3(Sv, Sv) + v2i_dot3(D, Svv);
        double det = J11 * J22 - J12 * J12;
        double duu, dvv;
        if (std::fabs(det) < 1e-300) {
            // Degenerate metric (a pole): fall back to a scaled gradient step.
            const double g = v2i_dot3(Su, Su) + v2i_dot3(Sv, Sv);
            if (g < 1e-300) break;
            duu = -f1 / g;
            dvv = -f2 / g;
        } else {
            duu = -(J22 * f1 - J12 * f2) / det;
            dvv = -(-J12 * f1 + J11 * f2) / det;
        }
        // Damping: never leap more than a quarter of the rectangle in one step.
        const double cap_u = 0.25 * ru, cap_v = 0.25 * rv;
        if (duu > cap_u) duu = cap_u;
        if (duu < -cap_u) duu = -cap_u;
        if (dvv > cap_v) dvv = cap_v;
        if (dvv < -cap_v) dvv = -cap_v;
        double un = std::min(f.umax, std::max(f.umin, u + duu));
        double vn = std::min(f.vmax, std::max(f.vmin, v + dvv));
        const double dn = v2i_dist3(s.point_at(un, vn), p);
        if (dn <= best + 1e-15 * (1.0 + best)) {
            best = dn;
            u = un;
            v = vn;
            uo = u; vo = v; dist = best;
        } else {
            // Backtrack once; if that also fails the iteration is converged enough.
            un = std::min(f.umax, std::max(f.umin, u + 0.25 * duu));
            vn = std::min(f.vmax, std::max(f.vmin, v + 0.25 * dvv));
            const double db = v2i_dist3(s.point_at(un, vn), p);
            if (db < best) { best = db; u = un; v = vn; uo = u; vo = v; dist = best; }
            else break;
        }
        if (std::fabs(duu) < 1e-14 * (1.0 + std::fabs(ru)) &&
            std::fabs(dvv) < 1e-14 * (1.0 + std::fabs(rv)))
            break;
    }
    return true;
}


bool v2_proj_ps(const V2FaceView& f, const Point& p, double useed, double vseed, double& u,
                double& v, double& dist) {
    if (!f.ok()) return false;
    // The cached grid makes the global search a table scan, not a surface evaluation sweep.
    int bi = 0, bj = 0;
    double bd = 1e300;
    for (int i = 0; i < f.nu; ++i)
        for (int j = 0; j < f.nv; ++j) {
            const double d = v2i_dist3(f.gp[(size_t)i * f.nv + j], p);
            if (d < bd) { bd = d; bi = i; bj = j; }
        }
    double u1 = 0, v1 = 0, d1 = 1e300;
    v2i_newton_ps(f, p, f.gu[bi], f.gv[bj], u1, v1, d1);
    if (useed > -1e299 && vseed > -1e299) {
        double u2 = 0, v2 = 0, d2 = 1e300;
        v2i_newton_ps(f, p, useed, vseed, u2, v2, d2);
        if (d2 < d1) { u1 = u2; v1 = v2; d1 = d2; }
    }
    // SEAM RETRY. On a closed direction the rectangle's two edges are the SAME 3D curve, so a
    // Newton step that wants to leave through one of them is clamped and the reported distance
    // is wrong. Measured: 1 of 297 box-edge/sphere piercings was lost this way, at a foot 0.5
    // degrees on the far side of the sphere's seam (d came back 1.35e-2 instead of 0).
    {
        const double ru = f.umax - f.umin, rv = f.vmax - f.vmin;
        const double eu = ru * 1e-6, ev = rv * 1e-6;
        for (int pass = 0; pass < 2; ++pass) {
            bool retried = false;
            if (f.closed_u && (u1 - f.umin < eu || f.umax - u1 < eu)) {
                const double uo = (u1 - f.umin < eu) ? f.umax : f.umin;
                double u2 = 0, v2 = 0, d2 = 1e300;
                v2i_newton_ps(f, p, uo, v1, u2, v2, d2);
                if (d2 < d1) { u1 = u2; v1 = v2; d1 = d2; retried = true; }
            }
            if (f.closed_v && (v1 - f.vmin < ev || f.vmax - v1 < ev)) {
                const double vo = (v1 - f.vmin < ev) ? f.vmax : f.vmin;
                double u2 = 0, v2 = 0, d2 = 1e300;
                v2i_newton_ps(f, p, u1, vo, u2, v2, d2);
                if (d2 < d1) { u1 = u2; v1 = v2; d1 = d2; retried = true; }
            }
            if (!retried) break;
        }
    }
    u = u1;
    v = v1;
    dist = d1;
    return true;
}

bool v2_surface_normal(const NurbsSurface& s, double u, double v, double& nx, double& ny,
                       double& nz, double eps) {
    const std::vector<Vector> ev = s.evaluate(u, v, 1);
    if (ev.size() < 3) return false;
    const Vector Sv = ev[1], Su = ev[2];
    const double cx = Su[1] * Sv[2] - Su[2] * Sv[1];
    const double cy = Su[2] * Sv[0] - Su[0] * Sv[2];
    const double cz = Su[0] * Sv[1] - Su[1] * Sv[0];
    const double m = std::sqrt(cx * cx + cy * cy + cz * cz);
    if (m <= eps) return false;
    nx = cx / m;
    ny = cy / m;
    nz = cz / m;
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// 3. Stage predicates
///////////////////////////////////////////////////////////////////////////////////////////

int v2_compute_vv(const Point& p1, double tol1, const Point& p2, double tol2, double fuzz) {
    const double f1 = std::max(fuzz, V2_CONFUSION);
    const double s = tol1 + tol2 + f1;
    const double dx = p1[0] - p2[0], dy = p1[1] - p2[1], dz = p1[2] - p2[2];
    return (dx * dx + dy * dy + dz * dz > s * s) ? 1 : 0;
}

int v2_compute_ve(const Point& pv, double tolv, const NurbsCurve& c, double t0, double t1,
                  double tole, double fuzz, double& t, double& tol_new) {
    t = 0;
    tol_new = 0;
    if (!(t1 > t0)) return -1;
    double tt = 0, dd = 0;
    if (v2_proj_pc(c, t0, t1, pv, tt, dd) == 0) return -3;
    const double tolsum = tolv + tole + std::max(fuzz, V2_CONFUSION);
    tol_new = dd + tole;          // NOT + tol(V)  (%IT%/IntTools_Context.cxx:534)
    t = tt;
    if (dd > tolsum) return -4;
    return 0;
}

V2VF v2_compute_vf(const Point& p, double tolv, const V2FaceView& f, double tolf, double fuzz,
                   double& u, double& v, double& tol_new, double& dist) {
    u = v = 0;
    tol_new = 0;
    dist = 1e300;
    if (!f.ok()) return V2VF::NoProjection;
    if (!v2_proj_ps(f, p, -1e300, -1e300, u, v, dist)) return V2VF::NoProjection;
    const double tolsum = tolv + tolf + std::max(fuzz, V2_CONFUSION);
    tol_new = dist + tolf;        // CTX:574 — the candidate new tolerance, not a max
    if (dist > tolsum) return V2VF::TooFar;
    // UV tolerance through the surface metric, AT THE POINT OF USE (invariant I5 / G7).
    double du = 0, dv = 0;
    if (!bds_uv_tolerance(*f.surface, u, v, std::max(tolf, V2_CONFUSION), du, dv)) {
        du = (f.umax - f.umin) * 1e-9;
        dv = (f.vmax - f.vmin) * 1e-9;
    }
    // STRICT IN: a foot exactly ON the trim boundary is VE's business (CTX:604-608).
    if (f.classify(u, v, du, dv) != V2State::In) return V2VF::OutsideFace;
    return V2VF::Ok;
}

///////////////////////////////////////////////////////////////////////////////////////////
// 4. Edge/edge
///////////////////////////////////////////////////////////////////////////////////////////


/// Local minimiser of |Ca(t) - Cb(s)|^2 by damped Newton, clamped to both ranges.
void v2i_newton_cc(const NurbsCurve& ca, double a0, double a1, const NurbsCurve& cb, double b0,
               double b1, double& t, double& s, double& d) {
    t = std::min(a1, std::max(a0, t));
    s = std::min(b1, std::max(b0, s));
    d = v2i_dist3(ca.point_at(t), cb.point_at(s));
    for (int it = 0; it < 60; ++it) {
        const std::vector<Vector> ea = ca.evaluate(t, 2);
        const std::vector<Vector> eb = cb.evaluate(s, 2);
        if (ea.size() < 2 || eb.size() < 2) break;
        const Vector Pa = ea[0], Ta = ea[1];
        const Vector Pb = eb[0], Tb = eb[1];
        const Vector Aa = ea.size() > 2 ? ea[2] : Vector(0, 0, 0);
        const Vector Ab = eb.size() > 2 ? eb[2] : Vector(0, 0, 0);
        const Vector D(Pa[0] - Pb[0], Pa[1] - Pb[1], Pa[2] - Pb[2]);
        const double g1 = v2i_dot3(D, Ta), g2 = -v2i_dot3(D, Tb);
        const double H11 = v2i_dot3(Ta, Ta) + v2i_dot3(D, Aa);
        const double H12 = -v2i_dot3(Ta, Tb);
        const double H22 = v2i_dot3(Tb, Tb) - v2i_dot3(D, Ab);
        const double det = H11 * H22 - H12 * H12;
        double dt, ds;
        if (std::fabs(det) < 1e-300) {
            const double g = v2i_dot3(Ta, Ta) + v2i_dot3(Tb, Tb);
            if (g < 1e-300) break;
            dt = -g1 / g;
            ds = -g2 / g;
        } else {
            dt = -(H22 * g1 - H12 * g2) / det;
            ds = -(-H12 * g1 + H11 * g2) / det;
        }
        const double capa = 0.25 * (a1 - a0), capb = 0.25 * (b1 - b0);
        dt = std::min(capa, std::max(-capa, dt));
        ds = std::min(capb, std::max(-capb, ds));
        double tn = std::min(a1, std::max(a0, t + dt));
        double sn = std::min(b1, std::max(b0, s + ds));
        double dn = v2i_dist3(ca.point_at(tn), cb.point_at(sn));
        if (dn > d) {
            tn = std::min(a1, std::max(a0, t + 0.25 * dt));
            sn = std::min(b1, std::max(b0, s + 0.25 * ds));
            dn = v2i_dist3(ca.point_at(tn), cb.point_at(sn));
            if (dn > d) break;
        }
        const double mv = std::fabs(tn - t) + std::fabs(sn - s);
        t = tn; s = sn; d = dn;
        if (mv < 1e-15 * (1.0 + std::fabs(t) + std::fabs(s))) break;
    }
}

/// Closest parameters of two 3D segments; returns the pair in [0,1]^2.
void v2i_seg_seg(const Point& p1, const Point& q1, const Point& p2, const Point& q2, double& h1,
             double& h2) {
    const Vector d1 = v2i_sub3(q1, p1), d2 = v2i_sub3(q2, p2), r = v2i_sub3(p1, p2);
    const double a = v2i_dot3(d1, d1), e = v2i_dot3(d2, d2), fq = v2i_dot3(d2, r);
    if (a <= 1e-300 && e <= 1e-300) { h1 = h2 = 0; return; }
    if (a <= 1e-300) { h1 = 0; h2 = std::min(1.0, std::max(0.0, fq / e)); return; }
    const double c = v2i_dot3(d1, r);
    if (e <= 1e-300) { h2 = 0; h1 = std::min(1.0, std::max(0.0, -c / a)); return; }
    const double b = v2i_dot3(d1, d2);
    const double den = a * e - b * b;
    h1 = (den > 1e-300) ? std::min(1.0, std::max(0.0, (b * fq - c * e) / den)) : 0.0;
    h2 = (b * h1 + fq) / e;
    if (h2 < 0) { h2 = 0; h1 = std::min(1.0, std::max(0.0, -c / a)); }
    else if (h2 > 1) { h2 = 1; h1 = std::min(1.0, std::max(0.0, (b - c) / a)); }
}


bool v2_edge_edge(const NurbsCurve& ca, double a0, double a1, double tola, const NurbsCurve& cb,
                  double b0, double b1, double tolb, double fuzz, std::vector<V2EEPart>& parts) {
    parts.clear();
    if (!(a1 > a0) || !(b1 > b0)) return false;
    const double crit = tola + tolb + std::max(fuzz, V2_CONFUSION);

    const int na = v2i_sample_count(ca, 8, 48, 256);
    const int nb = v2i_sample_count(cb, 8, 48, 256);
    std::vector<double> ta((size_t)na + 1), tb((size_t)nb + 1);
    std::vector<Point> pa((size_t)na + 1), pb((size_t)nb + 1);
    for (int i = 0; i <= na; ++i) {
        ta[i] = a0 + (a1 - a0) * (double)i / (double)na;
        pa[i] = ca.point_at(ta[i]);
    }
    for (int j = 0; j <= nb; ++j) {
        tb[j] = b0 + (b1 - b0) * (double)j / (double)nb;
        pb[j] = cb.point_at(tb[j]);
    }

    // Per-sample distance from A to B's polyline: the coincidence detector.
    std::vector<double> dA((size_t)na + 1, 1e300), sB((size_t)na + 1, b0);
    for (int i = 0; i <= na; ++i)
        for (int j = 0; j < nb; ++j) {
            double h1 = 0, h2 = 0;
            v2i_seg_seg(pa[i], pa[i], pb[j], pb[j + 1], h1, h2);
            const Point q(pb[j][0] + (pb[j + 1][0] - pb[j][0]) * h2,
                          pb[j][1] + (pb[j + 1][1] - pb[j][1]) * h2,
                          pb[j][2] + (pb[j + 1][2] - pb[j][2]) * h2);
            const double d = v2i_dist3(pa[i], q);
            if (d < dA[i]) { dA[i] = d; sB[i] = tb[j] + (tb[j + 1] - tb[j]) * h2; }
        }

    // ---- coincidence: maximal runs of dA <= crit, refined on the true curves --------------
    const double band = crit + 0.0;
    int i = 0;
    std::vector<std::pair<int, int>> runs;
    while (i <= na) {
        if (dA[i] <= band) {
            int j = i;
            while (j + 1 <= na && dA[j + 1] <= band) ++j;
            runs.push_back({i, j});
            i = j + 1;
        } else {
            ++i;
        }
    }
    for (const std::pair<int, int>& r : runs) {
        // 3D length of the run; below 2*crit it is a point contact, not an overlap.
        const double len = v2i_dist3(pa[r.first], pa[r.second]);
        if (len <= 2.0 * crit) continue;
        V2EEPart p;
        p.type = V2PartType::Edge;
        p.a0 = ta[r.first];
        p.a1 = ta[r.second];
        p.b0 = std::min(sB[r.first], sB[r.second]);
        p.b1 = std::max(sB[r.first], sB[r.second]);
        p.ta = 0.5 * (p.a0 + p.a1);
        p.tb = 0.5 * (p.b0 + p.b1);
        p.dist = dA[(r.first + r.second) / 2];
        parts.push_back(p);
    }
    if (!parts.empty()) return true;   // an overlap is never also reported as a crossing

    // ---- transversal / touching crossings -------------------------------------------------
    std::vector<V2EEPart> cands;
    for (int ii = 0; ii < na; ++ii) {
        for (int jj = 0; jj < nb; ++jj) {
            // Cheap reject: segment/segment distance beyond the band plus both chord sagittas.
            double h1 = 0, h2 = 0;
            v2i_seg_seg(pa[ii], pa[ii + 1], pb[jj], pb[jj + 1], h1, h2);
            const Point q1(pa[ii][0] + (pa[ii + 1][0] - pa[ii][0]) * h1,
                           pa[ii][1] + (pa[ii + 1][1] - pa[ii][1]) * h1,
                           pa[ii][2] + (pa[ii + 1][2] - pa[ii][2]) * h1);
            const Point q2(pb[jj][0] + (pb[jj + 1][0] - pb[jj][0]) * h2,
                           pb[jj][1] + (pb[jj + 1][1] - pb[jj][1]) * h2,
                           pb[jj][2] + (pb[jj + 1][2] - pb[jj][2]) * h2);
            const double sag = 0.5 * (v2i_dist3(pa[ii], pa[ii + 1]) + v2i_dist3(pb[jj], pb[jj + 1]));
            if (v2i_dist3(q1, q2) > crit + sag) continue;
            double t = ta[ii] + (ta[ii + 1] - ta[ii]) * h1;
            double s = tb[jj] + (tb[jj + 1] - tb[jj]) * h2;
            double d = 0;
            v2i_newton_cc(ca, a0, a1, cb, b0, b1, t, s, d);
            if (d > crit) continue;
            V2EEPart p;
            p.type = V2PartType::Vertex;
            p.ta = t;
            p.tb = s;
            p.a0 = p.a1 = t;
            p.b0 = p.b1 = s;
            p.dist = d;
            Point pt;
            Vector d1a, d1b;
            v2i_curve_pd(ca, t, pt, d1a);
            v2i_curve_pd(cb, s, pt, d1b);
            const double ma = std::sqrt(v2i_dot3(d1a, d1a)), mb = std::sqrt(v2i_dot3(d1b, d1b));
            if (ma > 1e-300 && mb > 1e-300) {
                const double cs = std::fabs(v2i_dot3(d1a, d1b) / (ma * mb));
                p.tangential = (cs > 1.0 - 1e-6);
            }
            cands.push_back(p);
        }
    }
    // Cluster by 3D proximity of the solution points.
    std::vector<char> used(cands.size(), 0);
    for (size_t k = 0; k < cands.size(); ++k) {
        if (used[k]) continue;
        V2EEPart best = cands[k];
        used[k] = 1;
        const Point pk = ca.point_at(best.ta);
        for (size_t l = k + 1; l < cands.size(); ++l) {
            if (used[l]) continue;
            if (v2i_dist3(pk, ca.point_at(cands[l].ta)) > std::max(10.0 * crit, 1e-9)) continue;
            used[l] = 1;
            if (cands[l].dist < best.dist) best = cands[l];
        }
        parts.push_back(best);
    }
    std::stable_sort(parts.begin(), parts.end(),
                     [](const V2EEPart& x, const V2EEPart& y) { return x.ta < y.ta; });
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// 5. Edge/face — the stage the kernel has never had
///////////////////////////////////////////////////////////////////////////////////////////


struct V2iEFSample {
    double t = 0, u = 0, v = 0, d = 0, s = 0;
    bool   has_normal = false;
};

V2iEFSample v2i_ef_probe(const NurbsCurve& c, double t, const V2FaceView& f, double useed, double vseed) {
    V2iEFSample e;
    e.t = t;
    const Point p = c.point_at(t);
    v2_proj_ps(f, p, useed, vseed, e.u, e.v, e.d);
    double nx = 0, ny = 0, nz = 0;
    if (v2_surface_normal(*f.surface, e.u, e.v, nx, ny, nz)) {
        const Point q = f.surface->point_at(e.u, e.v);
        e.s = (p[0] - q[0]) * nx + (p[1] - q[1]) * ny + (p[2] - q[2]) * nz;
        e.has_normal = true;
    } else {
        e.s = e.d;
    }
    return e;
}


bool v2_edge_face(const NurbsCurve& c, double t0, double t1, double tole, const V2FaceView& f,
                  double tolf, double fuzz, std::vector<V2EFPart>& parts, int samples) {
    parts.clear();
    if (!f.ok() || !(t1 > t0)) return false;
    const double crit = tole + tolf + std::max(fuzz, V2_CONFUSION);
    const int n = samples > 0 ? samples : v2i_sample_count(c, 12, 96, 320);

    std::vector<V2iEFSample> sm((size_t)n + 1);
    double us = -1e300, vs = -1e300;
    for (int i = 0; i <= n; ++i) {
        const double t = t0 + (t1 - t0) * (double)i / (double)n;
        sm[i] = v2i_ef_probe(c, t, f, us, vs);
        us = sm[i].u;
        vs = sm[i].v;
    }

    auto uv_tol = [&](double u, double v, double& du, double& dv) {
        if (!bds_uv_tolerance(*f.surface, u, v, std::max(tolf, V2_CONFUSION), du, dv)) {
            du = (f.umax - f.umin) * 1e-9;
            dv = (f.vmax - f.vmin) * 1e-9;
        }
    };
    auto state_at = [&](const V2iEFSample& e) {
        double du = 0, dv = 0;
        uv_tol(e.u, e.v, du, dv);
        return f.classify(e.u, e.v, du, dv);
    };

    // ---- 1. COINCIDENCE: maximal runs of d <= crit whose 3D extent exceeds 2*crit ----------
    std::vector<std::pair<int, int>> runs;
    {
        int i = 0;
        while (i <= n) {
            if (sm[i].d <= crit) {
                int j = i;
                while (j + 1 <= n && sm[j + 1].d <= crit) ++j;
                runs.push_back({i, j});
                i = j + 1;
            } else {
                ++i;
            }
        }
    }
    std::vector<std::pair<double, double>> covered;
    for (const std::pair<int, int>& r : runs) {
        // OCCT MakeType (IEF:326-347): an EDGE common part needs BOTH
        //   (a) isWholeRange — the part spans the whole WORKING range, and
        //   (b) |C(first) - C(last)| > 2*criteria — it is not a point contact.
        // (a) is why OCCT works per PAVE BLOCK: a coincident piece is a whole block once VE/EE
        // have split the edge at the ends of the contact. Dropping (a) and promoting any long
        // enough run turns a TANGENCY WIDENED BY A LARGE FUZZ into a false coincidence: at
        // fuzz = 1e-3 a line tangent to a unit cylinder is within criteria over |x| <= 0.0447,
        // a 0.09-long run that is not coincidence at all (kb/port_03 acceptance test I3d).
        if (r.first > 1 || r.second < n - 1) continue;
        const Point pf = c.point_at(sm[r.first].t), pl = c.point_at(sm[r.second].t);
        if (v2i_dist3(pf, pl) <= 2.0 * crit) continue;   // IEF:332-335, a point contact
        const int mid = (r.first + r.second) / 2;
        if (state_at(sm[mid]) == V2State::Out) continue;   // IsProjectable (IEF:582)
        V2EFPart p;
        p.type = V2PartType::Edge;
        p.kind = V2EFKind::Coincident;
        p.t0 = sm[r.first].t;
        p.t1 = sm[r.second].t;
        p.tv = sm[mid].t;
        p.u = sm[mid].u;
        p.v = sm[mid].v;
        p.dist = sm[mid].d;
        p.state = state_at(sm[mid]);
        parts.push_back(p);
        covered.push_back({p.t0, p.t1});
    }
    auto inside_covered = [&](double t) {
        for (const std::pair<double, double>& r : covered)
            if (t >= r.first && t <= r.second) return true;
        return false;
    };

    const double dt = (t1 - t0) / (double)n;

    // ---- 1b. END CONTACT: the pave block's own END sits on the face ------------------------
    // Decided by DISTANCE, never by a sign change. At a block end the crossing coincides with
    // the sample, so whether s(t) flips there is settled at the 1e-16 level and the answer
    // becomes pose-dependent — measured: cone x cone lost 2 of its 5 interferences under 2 of 8
    // rigid motions before this branch existed. The driver routes these to ForceInterfVF
    // (PF5:447-457): the EXISTING end vertex becomes known to the face; no vertex is minted.
    bool end_hit[2] = {false, false};
    for (int k = 0; k < 2; ++k) {
        const V2iEFSample& e = k ? sm[n] : sm[0];
        if (e.d > crit || !e.has_normal) continue;
        if (inside_covered(e.t)) continue;
        const V2State st = state_at(e);
        if (st == V2State::Out) continue;
        // Type it by the angle between the curve tangent and the surface normal — the only
        // information available at a range end, and a geometric criterion rather than a
        // floating-point accident.
        Point pp;
        Vector d1;
        v2i_curve_pd(c, e.t, pp, d1);
        double nx = 0, ny = 0, nz = 0;
        v2_surface_normal(*f.surface, e.u, e.v, nx, ny, nz);
        const double m = std::sqrt(v2i_dot3(d1, d1));
        const double cs = (m > 1e-300) ? std::fabs((d1[0] * nx + d1[1] * ny + d1[2] * nz) / m) : 0.0;
        V2EFPart p;
        p.type = V2PartType::Vertex;
        p.kind = (cs > 0.0871557427476582) ? V2EFKind::Transversal : V2EFKind::Tangential;
        p.t0 = t0;
        p.t1 = t1;
        p.tv = e.t;
        p.u = e.u;
        p.v = e.v;
        p.dist = e.d;
        p.state = st;
        parts.push_back(p);
        end_hit[k] = true;
    }
    auto near_end_hit = [&](double t) {
        return (end_hit[0] && t - t0 <= 2.0 * dt) || (end_hit[1] && t1 - t <= 2.0 * dt);
    };

    // ---- 2. TRANSVERSAL: sign changes of the normal residual s(t) --------------------------
    // s is exactly +-dist at a genuine projection foot, so a sign change is a crossing. This is
    // the event that is pose-independent; a distance threshold is not (divergence D2).
    std::vector<double> pierced;
    std::vector<V2EFPart> tvp;
    for (int i = 0; i < n; ++i) {
        if (!sm[i].has_normal || !sm[i + 1].has_normal) continue;
        if (!(sm[i].s * sm[i + 1].s < 0.0)) continue;
        double a = sm[i].t, b = sm[i + 1].t;
        double sa = sm[i].s;
        V2iEFSample mid = sm[i];
        for (int it = 0; it < 90 && (b - a) > 1e-16 * (1.0 + std::fabs(a)); ++it) {
            const double m = 0.5 * (a + b);
            mid = v2i_ef_probe(c, m, f, mid.u, mid.v);
            if (mid.s == 0.0) { a = b = m; break; }
            if ((sa < 0) == (mid.s < 0)) { a = m; sa = mid.s; } else { b = m; }
        }
        const double tm = 0.5 * (a + b);
        V2iEFSample e = v2i_ef_probe(c, tm, f, mid.u, mid.v);
        const V2State stdbg = state_at(e);
        if (std::getenv("V2DBG"))
            std::fprintf(stderr, "[XV] i=%d tm=%.6f d=%.3g uv=(%.5f,%.5f) state=%d cov=%d\n", i,
                         tm, e.d, e.u, e.v, (int)stdbg, (int)inside_covered(tm));
        if (e.d > std::max(crit, 1e-9)) continue;   // a foot that jumped, not a crossing
        if (inside_covered(tm) || near_end_hit(tm)) continue;
        const V2State st = stdbg;
        if (st == V2State::Out) continue;           // IsProjectable
        V2EFPart p;
        p.type = V2PartType::Vertex;
        p.kind = V2EFKind::Transversal;
        p.t0 = sm[i].t;
        p.t1 = sm[i + 1].t;
        p.tv = tm;
        p.u = e.u;
        p.v = e.v;
        p.dist = e.d;
        p.state = st;
        tvp.push_back(p);
    }
    // TANGENCY RECOVERY. A curve tangent to a surface is a DOUBLE root of s(t). Any perturbation
    // -- including the 1e-16 one a rigid motion introduces -- splits it into two simple roots a
    // few 1e-8 apart, and the same contact is then reported as two piercings instead of one
    // touch. Two crossings merge back into one Tangential part when they are within two sample
    // steps AND the curve never leaves the tolerance band between them: a genuine chord reaches
    // a real depth between its two ends, a tangency does not. Measured on the equal-radius
    // perpendicular cross-cylinder (an exact double tangency at (0,+-1,1)): without this the EF
    // part count oscillated 14/15/16 across rigid motions; with it, it is 14 in all 20.
    for (size_t k = 0; k < tvp.size(); ++k) {
        if (k + 1 < tvp.size() && tvp[k + 1].tv - tvp[k].tv < 2.0 * dt) {
            double smax = 0;
            V2iEFSample probe = sm[0];
            for (int q = 1; q <= 5; ++q) {
                const double tq = tvp[k].tv + (tvp[k + 1].tv - tvp[k].tv) * (double)q / 6.0;
                probe = v2i_ef_probe(c, tq, f, probe.u, probe.v);
                smax = std::max(smax, std::fabs(probe.s));
            }
            if (smax < crit) {
                V2EFPart p = tvp[k];
                p.kind = V2EFKind::Tangential;
                p.tv = 0.5 * (tvp[k].tv + tvp[k + 1].tv);
                p.t1 = tvp[k + 1].t1;
                const V2iEFSample e2 = v2i_ef_probe(c, p.tv, f, p.u, p.v);
                p.u = e2.u;
                p.v = e2.v;
                p.dist = e2.d;
                parts.push_back(p);
                pierced.push_back(p.tv);
                ++k;
                continue;
            }
        }
        parts.push_back(tvp[k]);
        pierced.push_back(tvp[k].tv);
    }

    // ---- 3. TANGENTIAL: an interior minimum of d below crit with NO sign change ------------
    for (int i = 1; i < n; ++i) {
        if (!(sm[i].d <= sm[i - 1].d && sm[i].d <= sm[i + 1].d)) continue;
        // Golden-section on d over the bracket; the touch may be far below the sample scale.
        double a = sm[i - 1].t, b = sm[i + 1].t;
        V2iEFSample seed = sm[i];
        const double gr = 0.6180339887498948;
        double x1 = b - gr * (b - a), x2 = a + gr * (b - a);
        V2iEFSample e1 = v2i_ef_probe(c, x1, f, seed.u, seed.v);
        V2iEFSample e2 = v2i_ef_probe(c, x2, f, seed.u, seed.v);
        for (int it = 0; it < 80 && (b - a) > 1e-14 * (1.0 + std::fabs(a)); ++it) {
            if (e1.d < e2.d) { b = x2; x2 = x1; e2 = e1; x1 = b - gr * (b - a);
                               e1 = v2i_ef_probe(c, x1, f, e2.u, e2.v); }
            else             { a = x1; x1 = x2; e1 = e2; x2 = a + gr * (b - a);
                               e2 = v2i_ef_probe(c, x2, f, e1.u, e1.v); }
        }
        const V2iEFSample e = (e1.d < e2.d) ? e1 : e2;
        if (e.d > crit) continue;
        if (inside_covered(e.t) || near_end_hit(e.t)) continue;
        bool near_pierce = false;
        for (double tp : pierced)
            if (std::fabs(tp - e.t) <= 2.0 * dt) near_pierce = true;
        if (near_pierce) continue;
        const V2State st = state_at(e);
        if (st == V2State::Out) continue;
        V2EFPart p;
        p.type = V2PartType::Vertex;
        p.kind = V2EFKind::Tangential;
        p.t0 = sm[i - 1].t;
        p.t1 = sm[i + 1].t;
        p.tv = e.t;
        p.u = e.u;
        p.v = e.v;
        p.dist = e.d;
        p.state = st;
        parts.push_back(p);
    }

    std::stable_sort(parts.begin(), parts.end(),
                     [](const V2EFPart& x, const V2EFPart& y) { return x.tv < y.tv; });
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// 6. The driver
///////////////////////////////////////////////////////////////////////////////////////////

V2Interf::V2Interf(BdsArena& ds, const std::vector<const BRep*>& operands,
                   const V2InterfOptions& opt)
    : m_ds(ds), m_ops(operands), m_opt(opt) {
    if (m_opt.fuzzy < V2_CONFUSION) m_opt.fuzzy = V2_CONFUSION;   // BOPAlgo_Options.cxx:107
}

const V2FaceView& V2Interf::face_view(int arena_face) {
    std::map<int, V2FaceView>::iterator it = m_fv.find(arena_face);
    if (it != m_fv.end()) return it->second;
    V2FaceView fv;
    const BdsShape& s = m_ds.shape(arena_face);
    if (s.operand >= 0 && s.operand < (int)m_ops.size() && m_ops[s.operand])
        fv = v2_make_face_view(*m_ops[s.operand], s.source, arena_face);
    return m_fv.emplace(arena_face, fv).first->second;
}

std::vector<V2Interf::Cand> V2Interf::candidates(BdsType ta, BdsType tb) const {
    // BOPDS_Iterator::Intersect (%DS%:270-357): box-overlap pairs, never within one operand's
    // own index range, then a STABLE sort — the only determinism source in the subsystem.
    std::vector<Cand> out;
    const int n = m_ds.nb_shapes();
    for (int i = 0; i < n; ++i) {
        if (m_ds.shape(i).type != ta) continue;
        const int ri = m_ds.rank(i);
        for (int j = 0; j < n; ++j) {
            if (m_ds.shape(j).type != tb) continue;
            if (ta == tb && j <= i) continue;
            const int rj = m_ds.rank(j);
            if (ri < 0 || rj < 0 || ri == rj) continue;   // no self-interference pairs
            if (m_ds.shape(i).box.is_out(m_ds.shape(j).box)) continue;
            out.push_back({i, j});
        }
    }
    std::stable_sort(out.begin(), out.end(), [](const Cand& a, const Cand& b) {
        return a.a != b.a ? a.a < b.a : a.b < b.b;
    });
    return out;
}

bool V2Interf::sub_shape_of(int container, int sub) const {
    if (!m_ds.valid(container) || !m_ds.valid(sub)) return false;
    const std::vector<int>& subs = m_ds.shape(container).subs;
    if (std::find(subs.begin(), subs.end(), sub) != subs.end()) return true;
    for (int s : subs) {
        const std::vector<int>& ss = m_ds.shape(s).subs;
        if (std::find(ss.begin(), ss.end(), sub) != ss.end()) return true;
    }
    return false;
}

bool V2Interf::interferes_with_sub(int x, int container) const {
    // BOPDS_DS::HasInterfShapeSubShapes (%DS%:356-375) — the VV->VE / VE->VF dependency in code.
    for (int s : m_ds.shape(container).subs)
        if (m_ds.has_interf(x, s)) return true;
    return false;
}

void V2Interf::chain_new_vertices(const std::vector<Point>& pts, const std::vector<double>& tols,
                                  std::vector<std::vector<int>>& chains) const {
    // BOPAlgo_Tools::IntersectVertices (%BA%:1119-1205).
    chains.clear();
    const int n = (int)pts.size();
    std::vector<std::vector<int>> adj((size_t)n);
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j) {
            const double s = tols[i] + tols[j] + m_opt.fuzzy;
            if (v2i_dist3(pts[i], pts[j]) < s) { adj[i].push_back(j); adj[j].push_back(i); }
        }
    std::vector<char> seen((size_t)n, 0);
    for (int i = 0; i < n; ++i) {
        if (seen[i]) continue;
        std::vector<int> comp;
        std::queue<int> q;
        q.push(i);
        seen[i] = 1;
        while (!q.empty()) {
            const int k = q.front();
            q.pop();
            comp.push_back(k);
            for (int m : adj[k]) if (!seen[m]) { seen[m] = 1; q.push(m); }
        }
        std::sort(comp.begin(), comp.end());
        chains.push_back(comp);
    }
}

//--- STAGE 1: VV --------------------------------------------------------------------------

void V2Interf::perform_vv() {
    const std::vector<Cand> cs = candidates(BdsType::Vertex, BdsType::Vertex);
    m_st.vv_pairs = (int)cs.size();
    std::map<int, std::vector<int>> adj;
    for (const Cand& c : cs) {
        const int a = m_ds.resolve_sd(c.a), b = m_ds.resolve_sd(c.b);
        if (a == b) continue;
        if (v2_compute_vv(m_ds.vertex_point(a), m_ds.tolerance(a), m_ds.vertex_point(b),
                          m_ds.tolerance(b), m_opt.fuzzy) != 0)
            continue;
        // FillMap: SYMMETRIC, so the BFS below can never dereference a missing key
        // (%BA%/BOPAlgo_Tools.hxx:83-102).
        adj[c.a].push_back(c.b);
        adj[c.b].push_back(c.a);
        if (m_opt.add_interfs) m_ds.add_interf(BdsInterfType::VV, c.a, c.b);
    }
    // MakeBlocks: plain BFS over the symmetric adjacency, components in first-seen key order.
    std::set<int> seen;
    for (const std::pair<const int, std::vector<int>>& kv : adj) {
        if (seen.count(kv.first)) continue;
        std::vector<int> comp;
        std::queue<int> q;
        q.push(kv.first);
        seen.insert(kv.first);
        while (!q.empty()) {
            const int k = q.front();
            q.pop();
            comp.push_back(k);
            std::map<int, std::vector<int>>::const_iterator it = adj.find(k);
            if (it == adj.end()) continue;
            for (int m : it->second)
                if (!seen.count(m)) { seen.insert(m); q.push(m); }
        }
        if (comp.size() < 2) continue;
        ++m_st.vv_clusters;
        const int nv = m_ds.fuse_vertices(comp);   // idempotent + order independent (I3)
        if (nv >= 0) {
            ++m_st.vv_fused;
            for (int k : comp) {
                const int idx = m_ds.add_interf(BdsInterfType::VV, k, nv);
                if (idx >= 0 && idx < (int)m_ds.interfs().size())
                    m_ds.change_interf(idx).new_vertex = nv;
            }
        }
    }
    // Rebuild every edge's paves through the SD map: InitPaveBlocksForVertex (PF_1.cxx:117-127).
    for (int i = 0; i < m_ds.nb_shapes(); ++i) {
        if (m_ds.shape(i).type != BdsType::Edge) continue;
        std::vector<BdsPB>& pool = m_ds.change_pave_blocks(i);
        for (BdsPB& pb : pool) {
            pb->pave1.vertex = m_ds.resolve_sd(pb->pave1.vertex);
            pb->pave2.vertex = m_ds.resolve_sd(pb->pave2.vertex);
        }
    }
}

//--- STAGE 2: VE --------------------------------------------------------------------------

void V2Interf::perform_ve() {
    const std::vector<Cand> cs = candidates(BdsType::Vertex, BdsType::Edge);
    m_st.ve_pairs = (int)cs.size();
    std::vector<int> touched;
    for (const Cand& c : cs) {
        const int nV = c.a, nE = c.b;
        if (sub_shape_of(nE, nV)) continue;                     // PF_2.cxx:166-169
        if (m_ds.has_interf(nV, nE)) continue;                  // PF_2.cxx:176-179
        if (interferes_with_sub(nV, nE)) continue;              // PF_2.cxx:181-184
        const int nVSD = m_ds.resolve_sd(nV);
        if (sub_shape_of(nE, nVSD)) continue;
        const NurbsCurve* cur = m_ds.edge_curve(nE);
        if (!cur) continue;
        const std::pair<double, double> r = m_ds.edge_range(nE);
        double t = 0, tol_new = 0;
        const int flag = v2_compute_ve(m_ds.vertex_point(nVSD), m_ds.tolerance(nVSD), *cur,
                                       r.first, r.second, m_ds.tolerance(nE), m_opt.fuzzy, t,
                                       tol_new);
        if (flag != 0) { ++m_st.ve_rejected; continue; }
        // UpdateVertex: the tolerance grows to cover the gap; the growth is RECORDED (G5).
        m_ds.absorb_tolerance(nVSD, tol_new);
        const BdsArena::PaveStatus st = m_ds.add_pave(nE, t, nVSD, m_opt.fuzzy);
        if (st != BdsArena::PaveStatus::Inserted) { ++m_st.ve_rejected; continue; }
        ++m_st.ve_paves;
        touched.push_back(nE);
        if (m_opt.add_interfs) {
            const int idx = m_ds.add_interf(BdsInterfType::VE, nV, nE);
            if (idx >= 0 && idx < (int)m_ds.interfs().size()) m_ds.change_interf(idx).ta = t;
        }
    }
    std::sort(touched.begin(), touched.end());
    touched.erase(std::unique(touched.begin(), touched.end()), touched.end());
    for (int e : touched) m_ds.update_pave_blocks(e);
}

//--- STAGE 3: EE --------------------------------------------------------------------------

void V2Interf::perform_ee() {
    const std::vector<Cand> cs = candidates(BdsType::Edge, BdsType::Edge);
    m_st.ee_pairs = (int)cs.size();

    struct Pending { int e1, e2; double t1, t2; Point p; double tol; bool tang; };
    std::vector<Pending> pend;

    for (const Cand& c : cs) {
        const int nE1 = c.a, nE2 = c.b;
        const NurbsCurve* c1 = m_ds.edge_curve(nE1);
        const NurbsCurve* c2 = m_ds.edge_curve(nE2);
        if (!c1 || !c2) continue;
        const std::vector<BdsPB>& pool1 = m_ds.pave_blocks(nE1);
        const std::vector<BdsPB>& pool2 = m_ds.pave_blocks(nE2);
        const double tol1 = m_ds.tolerance(nE1), tol2 = m_ds.tolerance(nE2);
        for (const BdsPB& pb1 : pool1) {
            for (const BdsPB& pb2 : pool2) {
                std::vector<V2EEPart> parts;
                if (!v2_edge_edge(*c1, pb1->pave1.t, pb1->pave2.t, tol1, *c2, pb2->pave1.t,
                                  pb2->pave2.t, tol2, m_opt.fuzzy, parts))
                    continue;
                for (const V2EEPart& p : parts) {
                    if (p.type == V2PartType::Edge) {
                        ++m_st.ee_edge_parts;
                        // G9/G10: an overlap is ONE object, and only when the two blocks already
                        // span the same two vertex indices (PF_3.cxx:535-540). Without VE having
                        // run this gate is unreachable — that is the order dependency, in code.
                        if (!pb1->has_same_bounds(*pb2)) continue;
                        const BdsCB cb = m_ds.make_common_block({pb1, pb2});
                        if (cb) ++m_st.ee_common_blocks;
                        if (m_opt.add_interfs) m_ds.add_interf(BdsInterfType::EE, nE1, nE2);
                        V2EERecord r;
                        r.edge_a = nE1; r.edge_b = nE2; r.type = V2PartType::Edge;
                        r.ta = p.ta; r.tb = p.tb;
                        m_ee.push_back(r);
                        continue;
                    }
                    ++m_st.ee_vertex_parts;
                    // MakeNewVertex (%BT%/BOPTools_AlgoTools_2.cxx:224-250).
                    const Point P1 = c1->point_at(p.ta), P2 = c2->point_at(p.tb);
                    const double d = v2i_dist3(P1, P2);
                    Pending q;
                    q.e1 = nE1; q.e2 = nE2; q.t1 = p.ta; q.t2 = p.tb;
                    q.p = Point(0.5 * (P1[0] + P2[0]), 0.5 * (P1[1] + P2[1]),
                                0.5 * (P1[2] + P2[2]));
                    q.tol = std::max(tol1, tol2) + 0.5 * d;
                    q.tang = p.tangential;
                    pend.push_back(q);
                }
            }
        }
    }
    if (pend.empty()) return;

    // PerformNewVertices (PF_3.cxx:594-688): fuse first, THEN place the paves — so the A-side and
    // B-side of the same crossing become ONE arena vertex, not two a hair apart.
    std::vector<Point> pts;
    std::vector<double> tols;
    for (const Pending& q : pend) { pts.push_back(q.p); tols.push_back(q.tol); }
    std::vector<std::vector<int>> chains;
    chain_new_vertices(pts, tols, chains);

    std::vector<int> touched;
    for (const std::vector<int>& ch : chains) {
        std::vector<Point> cp;
        std::vector<double> ct;
        for (int k : ch) { cp.push_back(pts[k]); ct.push_back(tols[k]); }
        Point cc = cp[0];
        double ctol = ct[0];
        if (cp.size() >= 2) bds_bounding_vertex(cp, ct, cc, ctol);
        const int nv = m_ds.append_vertex(cc, ctol, -1);
        ++m_st.ee_new_vertices;
        for (int k : ch) {
            const Pending& q = pend[k];
            if (m_ds.add_pave(q.e1, q.t1, nv, m_opt.fuzzy) == BdsArena::PaveStatus::Inserted) {
                ++m_st.ee_paves;
                touched.push_back(q.e1);
            }
            if (m_ds.add_pave(q.e2, q.t2, nv, m_opt.fuzzy) == BdsArena::PaveStatus::Inserted) {
                ++m_st.ee_paves;
                touched.push_back(q.e2);
            }
            if (m_opt.add_interfs) {
                const int idx = m_ds.add_interf(BdsInterfType::EE, q.e1, q.e2);
                if (idx >= 0 && idx < (int)m_ds.interfs().size()) {
                    m_ds.change_interf(idx).new_vertex = nv;
                    m_ds.change_interf(idx).ta = q.t1;
                    m_ds.change_interf(idx).tb = q.t2;
                }
            }
            V2EERecord r;
            r.edge_a = q.e1; r.edge_b = q.e2; r.type = V2PartType::Vertex;
            r.ta = q.t1; r.tb = q.t2; r.new_vertex = nv; r.tangential = q.tang;
            m_ee.push_back(r);
        }
    }
    std::sort(touched.begin(), touched.end());
    touched.erase(std::unique(touched.begin(), touched.end()), touched.end());
    for (int e : touched) m_ds.update_pave_blocks(e);
}

//--- STAGE 4: VF --------------------------------------------------------------------------

void V2Interf::perform_vf() {
    const std::vector<Cand> cs = candidates(BdsType::Vertex, BdsType::Face);
    m_st.vf_pairs = (int)cs.size();
    for (const Cand& c : cs) {
        const int nV = c.a, nF = c.b;
        if (sub_shape_of(nF, nV)) continue;                     // PF4:189
        if (m_ds.has_interf(nV, nF)) continue;                  // PF4:194
        m_ds.change_face_info(nF);                              // PF4:199 — side effect, not a skip
        if (interferes_with_sub(nV, nF)) continue;              // PF4:200
        const int nVSD = m_ds.resolve_sd(nV);
        const V2FaceView& fv = face_view(nF);
        if (!fv.ok()) continue;
        double u = 0, v = 0, tol_new = 0, d = 0;
        const V2VF st = v2_compute_vf(m_ds.vertex_point(nVSD), m_ds.tolerance(nVSD), fv,
                                      m_ds.tolerance(nF), m_opt.fuzzy, u, v, tol_new, d);
        if (st == V2VF::OutsideFace) { ++m_st.vf_rejected_on; continue; }
        if (st != V2VF::Ok) { ++m_st.vf_rejected_far; continue; }
        m_ds.absorb_tolerance(nVSD, tol_new);                   // PF4:286
        BdsFaceInfo& fi = m_ds.change_face_info(nF);
        if (std::find(fi.v_in.begin(), fi.v_in.end(), nVSD) == fi.v_in.end()) {
            fi.v_in.push_back(nVSD);
            std::sort(fi.v_in.begin(), fi.v_in.end());
        }
        ++m_st.vf_in;
        if (m_opt.add_interfs) {
            const int idx = m_ds.add_interf(BdsInterfType::VF, nV, nF);
            if (idx >= 0 && idx < (int)m_ds.interfs().size()) {
                m_ds.change_interf(idx).u = u;
                m_ds.change_interf(idx).v = v;
                m_ds.change_interf(idx).new_vertex = nVSD;
            }
        }
        V2VFRecord r;
        r.vertex = nVSD; r.face = nF; r.u = u; r.v = v; r.dist = d;
        m_vf.push_back(r);
    }
}

//--- STAGE 5: EF --------------------------------------------------------------------------

void V2Interf::perform_ef() {
    const std::vector<Cand> cs = candidates(BdsType::Edge, BdsType::Face);
    m_st.ef_pairs = (int)cs.size();

    struct Pending { int edge, face; double t; Point p; double tol; int rec; };
    std::vector<Pending> pend;

    for (const Cand& c : cs) {
        const int nE = c.a, nF = c.b;
        if (sub_shape_of(nF, nE)) continue;
        const NurbsCurve* cur = m_ds.edge_curve(nE);
        if (!cur) continue;
        const V2FaceView& fv = face_view(nF);
        if (!fv.ok()) continue;
        m_ds.change_face_info(nF);                              // PF5:237
        const double tolE = m_ds.tolerance(nE), tolF = m_ds.tolerance(nF);
        const std::vector<BdsPB> pool = m_ds.pave_blocks(nE);    // copy: paves are added below
        for (const BdsPB& pb : pool) {
            const double r0 = pb->pave1.t, r1 = pb->pave2.t;
            if (!(r1 - r0 > V2_PCONFUSION)) continue;           // GetPBBox (PF3:925-929)
            std::vector<V2EFPart> parts;
            if (!v2_edge_face(*cur, r0, r1, tolE, fv, tolF, m_opt.fuzzy, parts, m_opt.ef_samples))
                continue;
            for (const V2EFPart& p : parts) {
                if (p.type == V2PartType::Edge) {
                    ++m_st.ef_coincident;
                    if (m_opt.add_interfs) m_ds.add_interf(BdsInterfType::EF, nE, nF);
                    // PF5:545-566: the block joins the face only when BOTH its vertices are
                    // already known to the face; otherwise the interference is recorded and the
                    // contact is revisited once tolerances have grown.
                    const BdsFaceInfo& fi = m_ds.face_info(nF);
                    auto known = [&](int v) {
                        v = m_ds.resolve_sd(v);
                        return std::find(fi.v_on.begin(), fi.v_on.end(), v) != fi.v_on.end() ||
                               std::find(fi.v_in.begin(), fi.v_in.end(), v) != fi.v_in.end();
                    };
                    V2EFRecord rec;
                    rec.edge = nE; rec.face = nF; rec.kind = V2EFKind::Coincident;
                    rec.type = V2PartType::Edge;
                    rec.t = p.tv; rec.t0 = p.t0; rec.t1 = p.t1; rec.u = p.u; rec.v = p.v;
                    m_ef.push_back(rec);
                    if (known(pb->pave1.vertex) && known(pb->pave2.vertex)) {
                        m_ds.make_common_block_on_faces(pb, {nF});
                        m_ds.change_face_info(nF).pb_in.add(pb);
                    }
                    continue;
                }
                if (p.kind == V2EFKind::Tangential) ++m_st.ef_tangential;
                else ++m_st.ef_transversal;

                // "On a pave": the piercing coincides with one of the block's end vertices.
                // PF5:418-419 with aTolToDecide = 5e-8, in PARAMETER space.
                const bool on0 = std::fabs(p.tv - r0) <= V2_TOL_TO_DECIDE;
                const bool on1 = std::fabs(p.tv - r1) <= V2_TOL_TO_DECIDE;
                V2EFRecord rec;
                rec.edge = nE; rec.face = nF; rec.kind = p.kind; rec.type = V2PartType::Vertex;
                rec.t = p.tv; rec.t0 = p.t0; rec.t1 = p.t1; rec.u = p.u; rec.v = p.v;
                if (on0 || on1) {
                    // ForceInterfVF (PF5:631-681): do NOT mint a vertex a hair from an existing
                    // one; make the EXISTING end vertex known to the face instead.
                    ++m_st.ef_on_pave;
                    rec.on_pave = true;
                    const int nv = m_ds.resolve_sd(on0 ? pb->pave1.vertex : pb->pave2.vertex);
                    double u = 0, v = 0, tn = 0, dd = 0;
                    const V2VF st = v2_compute_vf(m_ds.vertex_point(nv), m_ds.tolerance(nv), fv,
                                                  tolF, m_opt.fuzzy, u, v, tn, dd);
                    if (st == V2VF::Ok || st == V2VF::TooFar) {   // -2 accepted (PF5:642)
                        m_ds.absorb_tolerance(nv, tn);
                        BdsFaceInfo& fi = m_ds.change_face_info(nF);
                        if (std::find(fi.v_in.begin(), fi.v_in.end(), nv) == fi.v_in.end()) {
                            fi.v_in.push_back(nv);
                            std::sort(fi.v_in.begin(), fi.v_in.end());
                        }
                        rec.new_vertex = nv;
                    }
                    if (m_opt.add_interfs) m_ds.add_interf(BdsInterfType::EF, nE, nF);
                    m_ef.push_back(rec);
                    continue;
                }
                if (p.state != V2State::In) { ++m_st.ef_rejected_out; continue; }  // G6, PF5:523
                if (p.kind == V2EFKind::Tangential && !m_opt.vertex_for_tangential) {
                    if (m_opt.add_interfs) m_ds.add_interf(BdsInterfType::EF, nE, nF);
                    m_ef.push_back(rec);
                    continue;
                }
                // CheckFacePaves (PF5:605-627): never mint a vertex on top of one the face
                // already owns.
                const Point pv = cur->point_at(p.tv);
                const double tolv = std::max(tolE, tolF) + V2_DTOLERANCE;
                bool clash = false;
                for (int vv : m_ds.face_info(nF).v_on)
                    if (v2_compute_vv(pv, tolv, m_ds.vertex_point(vv), m_ds.tolerance(vv), 0.0) == 0)
                        clash = true;
                if (clash) { ++m_st.ef_on_pave; rec.on_pave = true; m_ef.push_back(rec); continue; }
                Pending q;
                q.edge = nE; q.face = nF; q.t = p.tv; q.p = pv; q.tol = tolv;
                q.rec = (int)m_ef.size();
                m_ef.push_back(rec);
                pend.push_back(q);
            }
        }
    }
    if (pend.empty()) return;

    // PerformNewVertices again: fuse the piercings first (a box corner piercing two faces at the
    // same point must be ONE vertex — invariant G2 of kb/port_03), then place the paves.
    std::vector<Point> pts;
    std::vector<double> tols;
    for (const Pending& q : pend) { pts.push_back(q.p); tols.push_back(q.tol); }
    std::vector<std::vector<int>> chains;
    chain_new_vertices(pts, tols, chains);

    std::vector<int> touched;
    for (const std::vector<int>& ch : chains) {
        std::vector<Point> cp;
        std::vector<double> ct;
        for (int k : ch) { cp.push_back(pts[k]); ct.push_back(tols[k]); }
        Point cc = cp[0];
        double ctol = ct[0];
        if (cp.size() >= 2) bds_bounding_vertex(cp, ct, cc, ctol);
        const int nv = m_ds.append_vertex(cc, ctol, -1);
        ++m_st.ef_new_vertices;
        for (int k : ch) {
            const Pending& q = pend[k];
            m_ef[q.rec].new_vertex = nv;
            if (m_ds.add_pave(q.edge, q.t, nv, 0.0) == BdsArena::PaveStatus::Inserted) {
                ++m_st.ef_paves;
                touched.push_back(q.edge);
            }
            BdsFaceInfo& fi = m_ds.change_face_info(q.face);
            if (std::find(fi.v_in.begin(), fi.v_in.end(), nv) == fi.v_in.end()) {
                fi.v_in.push_back(nv);
                std::sort(fi.v_in.begin(), fi.v_in.end());
            }
            if (m_opt.add_interfs) {
                const int idx = m_ds.add_interf(BdsInterfType::EF, q.edge, q.face);
                if (idx >= 0 && idx < (int)m_ds.interfs().size()) {
                    m_ds.change_interf(idx).new_vertex = nv;
                    m_ds.change_interf(idx).ta = q.t;
                }
            }
        }
    }
    std::sort(touched.begin(), touched.end());
    touched.erase(std::unique(touched.begin(), touched.end()), touched.end());
    for (int e : touched) m_ds.update_pave_blocks(e);
    for (int i = 0; i < m_ds.nb_shapes(); ++i)
        if (m_ds.shape(i).type == BdsType::Face && m_ds.has_face_info(i))
            m_ds.refine_face_info_in(i);
}

void V2Interf::perform_all() {
    perform_vv();
    m_ds.update_pave_blocks();
    perform_ve();
    m_ds.update_pave_blocks();
    perform_ee();
    m_ds.update_pave_blocks();
    perform_vf();
    perform_ef();
    m_ds.update_pave_blocks();
}

int V2Interf::interf_count() const {
    return (int)m_ds.interfs().size();
}

bool V2Interf::check_invariants(std::string* why) const {
    auto fail = [&](const std::string& s) { if (why) *why = s; return false; };
    // A2/G6: pave blocks partition each edge's range with no gap and no overlap.
    for (int i = 0; i < m_ds.nb_shapes(); ++i) {
        if (m_ds.shape(i).type != BdsType::Edge) continue;
        if (!m_ds.has_pave_blocks(i)) continue;
        std::vector<BdsPave> pv;
        if (!m_ds.paves(i, pv)) return fail("pave tiling law violated on edge " +
                                            std::to_string(i));
        if (!m_ds.pave_blocks_cover_edge(i))
            return fail("pave blocks do not cover edge " + std::to_string(i));
    }
    // A3/G7: every pave's vertex is within its own tolerance of the curve point.
    for (int i = 0; i < m_ds.nb_shapes(); ++i) {
        if (m_ds.shape(i).type != BdsType::Edge) continue;
        if (!m_ds.has_pave_blocks(i)) continue;
        const NurbsCurve* c = m_ds.edge_curve(i);
        if (!c) continue;
        std::vector<BdsPave> pv;
        m_ds.paves(i, pv);
        for (const BdsPave& p : pv) {
            if (!m_ds.is_vertex(p.vertex)) return fail("pave names a non-vertex");
            const double d = v2i_dist3(m_ds.vertex_point(p.vertex), c->point_at(p.t));
            if (d > m_ds.tolerance(p.vertex) + 1e-6)
                return fail("pave containment violated on edge " + std::to_string(i));
        }
    }
    // Every EF record that created a vertex must have a pave on its edge at that vertex.
    for (const V2EFRecord& r : m_ef) {
        if (r.new_vertex < 0 || r.on_pave || r.type == V2PartType::Edge) continue;
        std::vector<BdsPave> pv;
        if (!m_ds.has_pave_blocks(r.edge)) return fail("EF edge has no pave blocks");
        m_ds.paves(r.edge, pv);
        bool found = false;
        for (const BdsPave& p : pv)
            if (m_ds.resolve_sd(p.vertex) == m_ds.resolve_sd(r.new_vertex)) found = true;
        if (!found) return fail("EF vertex has no pave on its edge");
    }
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// 7. Verdict metric
///////////////////////////////////////////////////////////////////////////////////////////

int v2_naked_edges(const BRep& b, int* non_manifold, int* degenerate) {
    if (non_manifold) *non_manifold = 0;
    if (degenerate) *degenerate = 0;
    double xmn = 1e300, ymn = 1e300, zmn = 1e300, xmx = -1e300, ymx = -1e300, zmx = -1e300;
    for (const Point& p : b.m_vertices) {
        xmn = std::min(xmn, p[0]); ymn = std::min(ymn, p[1]); zmn = std::min(zmn, p[2]);
        xmx = std::max(xmx, p[0]); ymx = std::max(ymx, p[1]); zmx = std::max(zmx, p[2]);
    }
    const double diag = b.m_vertices.empty() ? 1.0
        : std::sqrt((xmx - xmn) * (xmx - xmn) + (ymx - ymn) * (ymx - ymn) +
                    (zmx - zmn) * (zmx - zmn));
    const double deg_tol = std::max(diag * 1e-7, 1e-12);
    int naked = 0;
    for (const BRepEdge& e : b.m_topology_edges) {
        const int nt = (int)e.trim_indices.size();
        if (nt == 2) continue;
        if (nt == 0) continue;                       // orphaned record, no topological role
        // A DEGENERATE edge (3D curve collapsed to a point: a sphere pole, a cone apex) is
        // watertight by construction. THIS IS THE LINE FIVE MEASUREMENTS GOT WRONG.
        const int ci = e.curve_3d_index;
        if (ci >= 0 && ci < (int)b.m_curves_3d.size()) {
            const NurbsCurve& c = b.m_curves_3d[ci];
            const std::pair<double, double> dc = c.domain();
            const Point p0 = c.point_at(dc.first);
            double ext = 0.0;
            for (int k = 1; k <= 4; ++k)
                ext = std::max(ext, p0.distance(c.point_at(dc.first +
                                                           (dc.second - dc.first) * k / 4.0)));
            if (ext < deg_tol) { if (degenerate) ++*degenerate; continue; }
        }
        if (nt == 1) ++naked;
        else if (non_manifold) ++*non_manifold;
    }
    return naked;
}

bool v2_closed(const BRep& b) {
    int nm = 0;
    return v2_naked_edges(b, &nm, nullptr) == 0 && nm == 0;
}

}  // namespace v2int
}  // namespace session_cpp
