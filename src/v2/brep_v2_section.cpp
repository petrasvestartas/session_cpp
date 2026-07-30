#include "brep_v2_section.h"

#include "intersection.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>

namespace session_cpp {
namespace v2sec {

/// SESSION_V2_SECDBG=1 — per-curve seam/pave diagnostics. getenv()!=nullptr is TRUE for an
/// empty value, so the gate is (p && p[0]).
bool v2sec_dbg() {
    static const bool s = []() {
        const char* p = std::getenv("SESSION_V2_SECDBG");
        return p && p[0];
    }();
    return s;
}

namespace {

///////////////////////////////////////////////////////////////////////////////////////////
// tiny vector helpers (no dependency on Vector's arithmetic surface)
///////////////////////////////////////////////////////////////////////////////////////////

inline double v2dot3(const double a[3], const double b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline double v2nrm3(const double a[3]) { return std::sqrt(v2dot3(a, a)); }
inline double v2dist3(const Point& a, const Point& b) {
    const double d[3] = {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    return v2nrm3(d);
}
inline void v2sub3(const Point& a, const Point& b, double o[3]) {
    o[0] = a[0] - b[0];
    o[1] = a[1] - b[1];
    o[2] = a[2] - b[2];
}

/// Distance from p to the segment [a,b] in 3D.
double v2dist_point_seg(const Point& p, const Point& a, const Point& b) {
    double ab[3], ap[3];
    v2sub3(b, a, ab);
    v2sub3(p, a, ap);
    const double dd = v2dot3(ab, ab);
    double t = dd > 0.0 ? v2dot3(ap, ab) / dd : 0.0;
    t = std::max(0.0, std::min(1.0, t));
    const double q[3] = {a[0] + t * ab[0] - p[0], a[1] + t * ab[1] - p[1], a[2] + t * ab[2] - p[2]};
    return v2nrm3(q);
}

/// Squared distance between segments [a0,a1] and [b0,b1]; also returns the local parameters.
double v2seg_seg_dist2(const Point& a0, const Point& a1, const Point& b0, const Point& b1,
                     double& sa, double& sb) {
    double u[3], v[3], w[3];
    v2sub3(a1, a0, u);
    v2sub3(b1, b0, v);
    v2sub3(a0, b0, w);
    const double a = v2dot3(u, u), b = v2dot3(u, v), c = v2dot3(v, v), d = v2dot3(u, w), e = v2dot3(v, w);
    const double den = a * c - b * b;
    double s = 0.0, t = 0.0;
    if (den > 1e-30 * (a * c + 1.0)) {
        s = (b * e - c * d) / den;
        t = (a * e - b * d) / den;
    } else {
        s = 0.0;
        t = c > 0.0 ? e / c : 0.0;
    }
    s = std::max(0.0, std::min(1.0, s));
    t = std::max(0.0, std::min(1.0, t));
    // one refinement pass after clamping
    if (a > 0.0) s = std::max(0.0, std::min(1.0, (b * t - d) / a));
    if (c > 0.0) t = std::max(0.0, std::min(1.0, (b * s + e) / c));
    sa = s;
    sb = t;
    const double q[3] = {w[0] + s * u[0] - t * v[0], w[1] + s * u[1] - t * v[1],
                         w[2] + s * u[2] - t * v[2]};
    return v2dot3(q, q);
}

///////////////////////////////////////////////////////////////////////////////////////////
// surface evaluation
///////////////////////////////////////////////////////////////////////////////////////////

/// evaluate() returns [S, Sv, Svv, Su, Suv, Suu] for num_derivs == 2 and [S, Sv, Su] for 1
/// (nurbssurface.cpp:1254). Wrapped here so the ordering appears in exactly one place.
struct V2SurfJet {
    double S[3] = {0, 0, 0};
    double Su[3] = {0, 0, 0}, Sv[3] = {0, 0, 0};
    double Suu[3] = {0, 0, 0}, Suv[3] = {0, 0, 0}, Svv[3] = {0, 0, 0};
    bool ok = false;
};

V2SurfJet v2surf_jet(const NurbsSurface& s, double u, double v, int nd) {
    V2SurfJet j;
    const std::vector<Vector> d = s.evaluate(u, v, nd);
    if (nd >= 2 ? d.size() < 6 : d.size() < 3) return j;
    for (int k = 0; k < 3; ++k) {
        j.S[k] = d[0][k];
        j.Sv[k] = d[1][k];
    }
    if (nd >= 2) {
        for (int k = 0; k < 3; ++k) {
            j.Svv[k] = d[2][k];
            j.Su[k] = d[3][k];
            j.Suv[k] = d[4][k];
            j.Suu[k] = d[5][k];
        }
    } else {
        for (int k = 0; k < 3; ++k) j.Su[k] = d[2][k];
    }
    j.ok = true;
    return j;
}

inline double v2wrap_into(double x, double lo, double hi) {
    const double r = hi - lo;
    if (r <= 0.0) return lo;
    double f = std::fmod(x - lo, r);
    if (f < 0.0) f += r;
    return lo + f;
}

}  // namespace

///////////////////////////////////////////////////////////////////////////////////////////
// names
///////////////////////////////////////////////////////////////////////////////////////////

const char* v2_status_name(V2FFStatus s) {
    switch (s) {
        case V2FFStatus::Ok: return "Ok";
        case V2FFStatus::Tangent: return "Tangent";
        case V2FFStatus::Empty: return "Empty";
        case V2FFStatus::NoGeometricSolution: return "NoGeometricSolution";
        default: return "Failed";
    }
}

const char* v2_origin_name(V2PaveOrigin o) {
    switch (o) {
        case V2PaveOrigin::CurveBound: return "curve_bound";
        case V2PaveOrigin::Closing: return "closing";
        case V2PaveOrigin::TrimCrossing: return "trim_crossing";
        case V2PaveOrigin::SeamCrossing: return "seam_crossing";
        case V2PaveOrigin::PoleTouch: return "pole_touch";
        default: return "fused";
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// V2FaceRef / V2UvRect
///////////////////////////////////////////////////////////////////////////////////////////

bool V2FaceRef::valid() const {
    if (!brep || face < 0 || face >= (int)brep->m_faces.size()) return false;
    const int si = brep->m_faces[face].surface_index;
    return si >= 0 && si < (int)brep->m_surfaces.size() && brep->m_surfaces[si].is_valid();
}

const NurbsSurface& V2FaceRef::surface() const { return brep->m_surfaces[brep->m_faces[face].surface_index]; }

V2UvRect v2_uv_rect(const V2FaceRef& f) {
    V2UvRect r;
    if (!f.valid()) return r;
    const NurbsSurface& s = f.surface();
    const std::pair<double, double> du = s.domain(0);
    const std::pair<double, double> dv = s.domain(1);
    r.u0 = du.first;
    r.u1 = du.second;
    r.v0 = dv.first;
    r.v1 = dv.second;
    r.u_closed = s.is_closed(0);
    r.v_closed = s.is_closed(1);
    r.u_period = r.u_closed ? (r.u1 - r.u0) : 0.0;
    r.v_period = r.v_closed ? (r.v1 - r.v0) : 0.0;
    return r;
}

///////////////////////////////////////////////////////////////////////////////////////////
// inversion
///////////////////////////////////////////////////////////////////////////////////////////

namespace {

/// Newton point inversion from a seed. Closed directions wrap, open ones clamp, so the answer
/// always names a point of the PATCH and the returned distance is the honest patch distance.
double v2invert_newton(const NurbsSurface& s, const Point& p, double& u, double& v,
                       const V2UvRect& r) {
    double bu = u, bv = v;
    {
        const V2SurfJet j0 = v2surf_jet(s, bu, bv, 2);
        if (!j0.ok) return -1.0;
    }
    double best = v2dist3(s.point_at(bu, bv), p);
    const double span = (r.u1 - r.u0) + (r.v1 - r.v0);
    auto clampu = [&](double x) {
        return r.u_closed ? v2wrap_into(x, r.u0, r.u1) : std::max(r.u0, std::min(r.u1, x));
    };
    auto clampv = [&](double x) {
        return r.v_closed ? v2wrap_into(x, r.v0, r.v1) : std::max(r.v0, std::min(r.v1, x));
    };
    // Two candidate steps per iteration: full Newton (fast near the solution) and Gauss-Newton
    // (the metric only, always a descent direction). Newton alone stalls in the near-singular
    // band around a pole, and a stalled inversion reports a distance that is really just the
    // sampling step -- the defect that made this stage's first G1 residual 2.8e-1.
    auto descend = [&]() {
        for (int it = 0; it < 64; ++it) {
            const V2SurfJet j = v2surf_jet(s, bu, bv, 2);
            if (!j.ok) break;
            const double rr[3] = {j.S[0] - p[0], j.S[1] - p[1], j.S[2] - p[2]};
            const double f1 = v2dot3(rr, j.Su), f2 = v2dot3(rr, j.Sv);
            const double g11 = v2dot3(j.Su, j.Su), g12 = v2dot3(j.Su, j.Sv),
                         g22 = v2dot3(j.Sv, j.Sv);
            double steps[2][2] = {{0, 0}, {0, 0}};
            const double a11 = g11 + v2dot3(rr, j.Suu);
            const double a12 = g12 + v2dot3(rr, j.Suv);
            const double a22 = g22 + v2dot3(rr, j.Svv);
            const double detN = a11 * a22 - a12 * a12;
            if (std::abs(detN) > 1e-30) {
                steps[0][0] = (-f1 * a22 + f2 * a12) / detN;
                steps[0][1] = (-f2 * a11 + f1 * a12) / detN;
            }
            const double detG = g11 * g22 - g12 * g12;
            if (std::abs(detG) > 1e-30) {
                steps[1][0] = (-f1 * g22 + f2 * g12) / detG;
                steps[1][1] = (-f2 * g11 + f1 * g12) / detG;
            } else {
                steps[1][0] = g11 > 1e-30 ? -f1 / g11 : 0.0;
                steps[1][1] = g22 > 1e-30 ? -f2 / g22 : 0.0;
            }
            bool improved = false;
            double moved = 0.0;
            for (int m = 0; m < 2 && !improved; ++m)
                for (int bt = 0; bt < 12 && !improved; ++bt) {
                    const double sc = std::pow(0.5, bt);
                    const double nu = clampu(bu + sc * steps[m][0]);
                    const double nv = clampv(bv + sc * steps[m][1]);
                    const double d = v2dist3(s.point_at(nu, nv), p);
                    if (d < best) {
                        moved = std::abs(nu - bu) + std::abs(nv - bv);
                        bu = nu;
                        bv = nv;
                        best = d;
                        improved = true;
                    }
                }
            if (!improved || moved < 1e-15 * span) break;
        }
    };
    descend();
    // Safety net: a zooming local scan, each level followed by another descent. The FIRST
    // level is deliberately wide (half the domain): a descent that started on the wrong
    // meridian of a pole-parameterised surface sits in a genuine local valley, and a narrow
    // zoom cannot leave it. That valley is what reported a 0.25 residual for a point that
    // lies exactly ON the sphere.
    static const double half[3] = {0.5, 0.18, 0.06};
    for (int lev = 0; lev < 3 && best > 1e-14 * std::max(1.0, span); ++lev) {
        const double hu = (r.u1 - r.u0) * half[lev];
        const double hv = (r.v1 - r.v0) * half[lev];
        double cu = bu, cv = bv, cd = best;
        for (int i = -4; i <= 4; ++i)
            for (int k = -4; k <= 4; ++k) {
                if (!i && !k) continue;
                const double uu = clampu(bu + hu * i / 4.0), vv = clampv(bv + hv * k / 4.0);
                const double d = v2dist3(s.point_at(uu, vv), p);
                if (d < cd) {
                    cd = d;
                    cu = uu;
                    cv = vv;
                }
            }
        if (cd >= best) continue;
        bu = cu;
        bv = cv;
        best = cd;
        descend();
    }
    u = bu;
    v = bv;
    return best;
}

}  // namespace

double v2_invert(const NurbsSurface& s, const Point& p, double& u, double& v, bool seeded) {
    V2FaceRef dummy;
    V2UvRect r;
    const std::pair<double, double> du = s.domain(0);
    const std::pair<double, double> dv = s.domain(1);
    r.u0 = du.first;
    r.u1 = du.second;
    r.v0 = dv.first;
    r.v1 = dv.second;
    r.u_closed = s.is_closed(0);
    r.v_closed = s.is_closed(1);
    (void)dummy;
    if (!seeded) {
        const int nu = std::max(8, std::min(40, s.cv_count(0) * 4));
        const int nv = std::max(8, std::min(40, s.cv_count(1) * 4));
        std::vector<std::pair<double, std::pair<double, double>>> cands;
        cands.reserve((size_t)(nu + 1) * (nv + 1));
        for (int i = 0; i <= nu; ++i) {
            const double uu = r.u0 + (r.u1 - r.u0) * i / nu;
            for (int k = 0; k <= nv; ++k) {
                const double vv = r.v0 + (r.v1 - r.v0) * k / nv;
                cands.push_back({v2dist3(s.point_at(uu, vv), p), {uu, vv}});
            }
        }
        std::partial_sort(cands.begin(), cands.begin() + std::min<size_t>(8, cands.size()),
                          cands.end());
        // MULTI-START: the nearest grid cell is not always the basin of the true foot point
        // (a cylinder wall, a pole band). Newton from the four nearest cells, keep the best.
        double bd = 1e300, bu = cands[0].second.first, bv = cands[0].second.second;
        for (size_t i = 0; i < cands.size() && i < 8; ++i) {
            double uu = cands[i].second.first, vv = cands[i].second.second;
            const double d = v2invert_newton(s, p, uu, vv, r);
            if (d >= 0 && d < bd) {
                bd = d;
                bu = uu;
                bv = vv;
            }
        }
        u = bu;
        v = bv;
        return bd;
    }
    return v2invert_newton(s, p, u, v, r);
}

///////////////////////////////////////////////////////////////////////////////////////////
// arclength — composite 5-point Gauss-Legendre on |C'(t)|. Own integrator, so a block's
// measured length never depends on NurbsCurve::trim() reproducing the parameterisation, and a
// rational conic integrates to machine precision.
///////////////////////////////////////////////////////////////////////////////////////////

double v2sec_arclen(const NurbsCurve& c, double t0, double t1) {
    if (!c.is_valid() || !(t1 > t0)) return 0.0;
    static const double gx[5] = {0.0, 0.5384693101056831, -0.5384693101056831,
                                 0.9061798459386640, -0.9061798459386640};
    static const double gw[5] = {0.5688888888888889, 0.4786286704993665, 0.4786286704993665,
                                 0.2369268850561891, 0.2369268850561891};
    // Integrate each KNOT SPAN separately. |C'(t)| of a rational conic has a kink at every
    // span joint; a panel that straddles one costs ~1e-7 of relative accuracy, which is what
    // made this stage's first measured arclengths disagree with the closed form at 1e-6.
    std::vector<double> cuts;
    cuts.push_back(t0);
    for (double k : c.get_span_vector())
        if (k > t0 + 1e-15 && k < t1 - 1e-15) cuts.push_back(k);
    cuts.push_back(t1);
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());

    double total = 0.0;
    for (size_t seg = 0; seg + 1 < cuts.size(); ++seg) {
        const double a0 = cuts[seg], a1 = cuts[seg + 1];
        const int n = 64;
        const double h = (a1 - a0) / n;
        double s = 0.0;
        for (int i = 0; i < n; ++i) {
            const double m = a0 + (i + 0.5) * h;
            for (int k = 0; k < 5; ++k) {
                const std::vector<Vector> d = c.evaluate(m + 0.5 * h * gx[k], 1);
                if (d.size() < 2) continue;
                const double dd[3] = {d[1][0], d[1][1], d[1][2]};
                s += gw[k] * v2nrm3(dd);
            }
        }
        total += s * 0.5 * h;
    }
    return total;
}

///////////////////////////////////////////////////////////////////////////////////////////
// V2FaceClassifier
///////////////////////////////////////////////////////////////////////////////////////////

V2FaceClassifier::V2FaceClassifier(const V2FaceRef& f) : m_f(f) {
    if (!f.valid()) return;
    m_s = &f.surface();
    m_rect = v2_uv_rect(f);
    const BRep& B = *f.brep;
    const NurbsSurface& S = *m_s;

    // 3D sag target for the wire tessellation: a MODEL-SPACE length tied to the patch size,
    // never to range(u)/range(v) (kb/port_04 G6).
    double diag = 0.0;
    {
        Point lo = S.point_at(m_rect.u0, m_rect.v0), hi = lo;
        for (int i = 0; i <= 4; ++i)
            for (int k = 0; k <= 4; ++k) {
                const Point q = S.point_at(m_rect.u0 + (m_rect.u1 - m_rect.u0) * i / 4.0,
                                           m_rect.v0 + (m_rect.v1 - m_rect.v0) * k / 4.0);
                for (int d = 0; d < 3; ++d) {
                    lo[d] = std::min(lo[d], q[d]);
                    hi[d] = std::max(hi[d], q[d]);
                }
            }
        diag = v2dist3(lo, hi);
    }
    const double sag = std::max(1e-9, diag * 1e-5);

    // tessellate every trim pcurve of the face into a UV polyline with 3D sag control
    struct Seg {
        std::vector<double> u, v;
        std::vector<Point> p3;
    };
    std::vector<std::vector<Seg>> loop_segs;
    for (int li : B.m_faces[f.face].loop_indices) {
        if (li < 0 || li >= (int)B.m_loops.size()) continue;
        std::vector<Seg> segs;
        for (int ti : B.m_loops[li].trim_indices) {
            if (ti < 0 || ti >= (int)B.m_trims.size()) continue;
            const int ci = B.m_trims[ti].curve_2d_index;
            if (ci < 0 || ci >= (int)B.m_curves_2d.size()) continue;
            const NurbsCurve& pc = B.m_curves_2d[ci];
            if (!pc.is_valid()) continue;
            const std::pair<double, double> dm = pc.domain();
            int n = std::max(16, std::min(256, pc.span_count() * 12));
            std::vector<double> ts;
            for (int i = 0; i <= n; ++i) ts.push_back(dm.first + (dm.second - dm.first) * i / n);
            for (int pass = 0; pass < 4 && (int)ts.size() < 2048; ++pass) {
                std::vector<double> nt;
                bool split = false;
                for (size_t i = 0; i + 1 < ts.size(); ++i) {
                    nt.push_back(ts[i]);
                    const double tm = 0.5 * (ts[i] + ts[i + 1]);
                    const Point a2 = pc.point_at(ts[i]), b2 = pc.point_at(ts[i + 1]),
                                m2 = pc.point_at(tm);
                    const Point A = S.point_at(a2[0], a2[1]), Bp = S.point_at(b2[0], b2[1]),
                                M = S.point_at(m2[0], m2[1]);
                    const Point C(0.5 * (A[0] + Bp[0]), 0.5 * (A[1] + Bp[1]), 0.5 * (A[2] + Bp[2]));
                    if (v2dist3(C, M) > sag) {
                        nt.push_back(tm);
                        split = true;
                    }
                }
                nt.push_back(ts.back());
                ts.swap(nt);
                if (!split) break;
            }
            Seg sg;
            for (double t : ts) {
                const Point q2 = pc.point_at(t);
                sg.u.push_back(q2[0]);
                sg.v.push_back(q2[1]);
                sg.p3.push_back(S.point_at(q2[0], q2[1]));
            }
            if (sg.u.size() >= 2) segs.push_back(std::move(sg));
        }
        if (!segs.empty()) loop_segs.push_back(std::move(segs));
    }

    // Chain each loop's trims head-to-tail, flipping when needed. The matching epsilon is a
    // TOPOLOGICAL one: it only decides which stored polyline continues which, never a
    // geometric verdict, so a relative parameter epsilon is safe here.
    const double eu = std::max(1e-12, (m_rect.u1 - m_rect.u0) * 1e-6);
    const double ev = std::max(1e-12, (m_rect.v1 - m_rect.v0) * 1e-6);
    for (std::vector<Seg>& segs : loop_segs) {
        std::vector<char> used(segs.size(), 0);
        for (size_t start = 0; start < segs.size(); ++start) {
            if (used[start]) continue;
            Poly poly;
            auto append = [&](const Seg& sg, bool flip, bool first) {
                const int n = (int)sg.u.size();
                for (int k = 0; k < n; ++k) {
                    const int i = flip ? (n - 1 - k) : k;
                    if (!first && k == 0) continue;
                    poly.u.push_back(sg.u[i]);
                    poly.v.push_back(sg.v[i]);
                    poly.p3.push_back(sg.p3[i]);
                }
            };
            append(segs[start], false, true);
            used[start] = 1;
            for (;;) {
                const double cu = poly.u.back(), cv = poly.v.back();
                int best = -1;
                bool bflip = false;
                for (size_t k = 0; k < segs.size(); ++k) {
                    if (used[k]) continue;
                    if (std::abs(segs[k].u.front() - cu) <= eu &&
                        std::abs(segs[k].v.front() - cv) <= ev) {
                        best = (int)k;
                        bflip = false;
                        break;
                    }
                    if (std::abs(segs[k].u.back() - cu) <= eu &&
                        std::abs(segs[k].v.back() - cv) <= ev) {
                        best = (int)k;
                        bflip = true;
                        break;
                    }
                }
                if (best < 0) break;
                append(segs[best], bflip, false);
                used[best] = 1;
            }
            if (poly.u.size() >= 3) m_loops.push_back(std::move(poly));
        }
    }

    // seed grid for inversion
    m_gnu = std::max(6, std::min(24, S.cv_count(0) * 2));
    m_gnv = std::max(6, std::min(24, S.cv_count(1) * 2));
    m_grid.reserve((size_t)(m_gnu + 1) * (m_gnv + 1));
    for (int i = 0; i <= m_gnu; ++i)
        for (int k = 0; k <= m_gnv; ++k)
            m_grid.push_back(S.point_at(m_rect.u0 + (m_rect.u1 - m_rect.u0) * i / m_gnu,
                                        m_rect.v0 + (m_rect.v1 - m_rect.v0) * k / m_gnv));
    m_ok = !m_loops.empty();
}

double V2FaceClassifier::invert(const Point& p, double& u, double& v, bool seeded) const {
    if (!m_s) return -1.0;
    if (seeded || m_grid.empty()) return v2invert_newton(*m_s, p, u, v, m_rect);
    std::vector<std::pair<double, std::pair<double, double>>> cands;
    cands.reserve(m_grid.size());
    for (int i = 0; i <= m_gnu; ++i)
        for (int k = 0; k <= m_gnv; ++k)
            cands.push_back({v2dist3(m_grid[(size_t)i * (m_gnv + 1) + k], p),
                             {m_rect.u0 + (m_rect.u1 - m_rect.u0) * i / m_gnu,
                              m_rect.v0 + (m_rect.v1 - m_rect.v0) * k / m_gnv}});
    std::partial_sort(cands.begin(), cands.begin() + std::min<size_t>(8, cands.size()), cands.end());
    double bd = 1e300, bu = cands[0].second.first, bv = cands[0].second.second;
    for (size_t i = 0; i < cands.size() && i < 8; ++i) {
        double uu = cands[i].second.first, vv = cands[i].second.second;
        const double d = v2invert_newton(*m_s, p, uu, vv, m_rect);
        if (d >= 0 && d < bd) {
            bd = d;
            bu = uu;
            bv = vv;
        }
    }
    u = bu;
    v = bv;
    return bd;
}

V2State V2FaceClassifier::state(double u, double v, double tol3d) const {
    if (!m_s) return V2State::Out;
    if (m_loops.empty()) {
        // No wires: the face IS its rectangle.
        const bool in = u > m_rect.u0 && u < m_rect.u1 && v > m_rect.v0 && v < m_rect.v1;
        return in ? V2State::In : V2State::On;
    }
    double uu = m_rect.u_closed ? v2wrap_into(u, m_rect.u0, m_rect.u1) : u;
    double vv = m_rect.v_closed ? v2wrap_into(v, m_rect.v0, m_rect.v1) : v;

    // ON is measured in MODEL SPACE against the 3D image of the wire (never a parameter band).
    const Point P = m_s->point_at(uu, vv);
    double dmin = 1e300;
    for (const Poly& L : m_loops)
        for (size_t i = 0; i + 1 < L.p3.size(); ++i)
            dmin = std::min(dmin, v2dist_point_seg(P, L.p3[i], L.p3[i + 1]));
    if (dmin <= tol3d) return V2State::On;

    // even-odd crossing in UV; period shifts are retried for a closed direction
    const int nshift = m_rect.u_closed ? 2 : 1;
    for (int sh = 0; sh < nshift; ++sh) {
        const double us = uu + (sh == 1 ? -m_rect.u_period : 0.0);
        int cross = 0;
        for (const Poly& L : m_loops) {
            const size_t n = L.u.size();
            for (size_t i = 0; i + 1 < n; ++i) {
                const double a0 = L.u[i], a1 = L.v[i], b0 = L.u[i + 1], b1 = L.v[i + 1];
                if ((a1 > vv) == (b1 > vv)) continue;
                const double x = a0 + (vv - a1) / (b1 - a1) * (b0 - a0);
                if (x > us) ++cross;
            }
            // close the polygon
            const double a0 = L.u[n - 1], a1 = L.v[n - 1], b0 = L.u[0], b1 = L.v[0];
            if ((a1 > vv) != (b1 > vv)) {
                const double x = a0 + (vv - a1) / (b1 - a1) * (b0 - a0);
                if (x > us) ++cross;
            }
        }
        if (cross & 1) return V2State::In;
    }
    return V2State::Out;
}

bool V2FaceClassifier::valid_point(const Point& p, double tol3d, double* u_out,
                                   double* v_out) const {
    if (!m_s) return false;
    double u = 0, v = 0;
    const double d = invert(p, u, v, false);
    if (u_out) *u_out = u;
    if (v_out) *v_out = v;
    if (d < 0.0 || d > tol3d) return false;
    return state(u, v, tol3d) != V2State::Out;
}

V2State v2_state_point_face(const V2FaceRef& f, double u, double v, double tol3d) {
    return V2FaceClassifier(f).state(u, v, tol3d);
}
bool v2_is_point_in_on_face(const V2FaceRef& f, double u, double v, double tol3d) {
    return v2_state_point_face(f, u, v, tol3d) != V2State::Out;
}
bool v2_is_valid_point_for_face(const V2FaceRef& f, const Point& p, double tol3d, double* u_out,
                                double* v_out) {
    return V2FaceClassifier(f).valid_point(p, tol3d, u_out, v_out);
}
bool v2_is_valid_point_for_faces(const V2FaceRef& a, const V2FaceRef& b, const Point& p,
                                 double tol3d) {
    return v2_is_valid_point_for_face(a, p, tol3d) && v2_is_valid_point_for_face(b, p, tol3d);
}

///////////////////////////////////////////////////////////////////////////////////////////
// curve/curve 3D intersection — the ENTITY behind every trim and seam pave
///////////////////////////////////////////////////////////////////////////////////////////

namespace {

/// Returns the MEASURED max chord sag of the tessellation it produced. The bracket test in
/// v2_curve_curve_points needs the sag that was actually achieved, not the one that was asked
/// for: a refinement that hits its point budget silently widens the bracket it needs, and a
/// gate built on the target instead of the measurement misses real crossings.
double v2tess_curve(const NurbsCurve& c, double sag, std::vector<double>& ts,
                    std::vector<Point>& ps) {
    const std::pair<double, double> d = c.domain();
    int n = std::max(64, std::min(1024, c.span_count() * 48));
    ts.clear();
    for (int i = 0; i <= n; ++i) ts.push_back(d.first + (d.second - d.first) * i / n);
    for (int pass = 0; pass < 6 && (int)ts.size() < 12000; ++pass) {
        std::vector<double> nt;
        bool split = false;
        for (size_t i = 0; i + 1 < ts.size(); ++i) {
            nt.push_back(ts[i]);
            const double tm = 0.5 * (ts[i] + ts[i + 1]);
            const Point A = c.point_at(ts[i]), B = c.point_at(ts[i + 1]), M = c.point_at(tm);
            const Point C(0.5 * (A[0] + B[0]), 0.5 * (A[1] + B[1]), 0.5 * (A[2] + B[2]));
            if (v2dist3(C, M) > sag) {
                nt.push_back(tm);
                split = true;
            }
        }
        nt.push_back(ts.back());
        ts.swap(nt);
        if (!split) break;
    }
    ps.clear();
    ps.reserve(ts.size());
    for (double t : ts) ps.push_back(c.point_at(t));
    double worst = 0.0;
    for (size_t i = 0; i + 1 < ts.size(); ++i) {
        const Point M = c.point_at(0.5 * (ts[i] + ts[i + 1]));
        const Point C(0.5 * (ps[i][0] + ps[i + 1][0]), 0.5 * (ps[i][1] + ps[i + 1][1]),
                      0.5 * (ps[i][2] + ps[i + 1][2]));
        worst = std::max(worst, v2dist3(C, M));
    }
    return worst;
}

/// 2x2 Newton on ((A(s)-B(t)).A'(s), (A(s)-B(t)).B'(t)) = 0.
bool v2refine_cc(const NurbsCurve& A, const NurbsCurve& B, double& s, double& t, double tol,
               double& dout) {
    const std::pair<double, double> da = A.domain(), db = B.domain();
    double best = v2dist3(A.point_at(s), B.point_at(t));
    for (int it = 0; it < 40; ++it) {
        const std::vector<Vector> ja = A.evaluate(s, 2);
        const std::vector<Vector> jb = B.evaluate(t, 2);
        if (ja.size() < 3 || jb.size() < 3) break;
        double r[3], a1[3], a2[3], b1[3], b2[3];
        for (int k = 0; k < 3; ++k) {
            r[k] = ja[0][k] - jb[0][k];
            a1[k] = ja[1][k];
            a2[k] = ja[2][k];
            b1[k] = jb[1][k];
            b2[k] = jb[2][k];
        }
        const double f1 = v2dot3(r, a1), f2 = -v2dot3(r, b1);
        const double m11 = v2dot3(a1, a1) + v2dot3(r, a2);
        const double m12 = -v2dot3(a1, b1);
        const double m21 = -v2dot3(a1, b1);
        const double m22 = v2dot3(b1, b1) - v2dot3(r, b2);
        const double det = m11 * m22 - m12 * m21;
        if (std::abs(det) < 1e-30) break;
        const double ds = (-f1 * m22 + f2 * m12) / det;
        const double dt = (-f2 * m11 + f1 * m21) / det;
        bool improved = false;
        for (int bt = 0; bt < 8 && !improved; ++bt) {
            const double sc = std::pow(0.5, bt);
            const double ns = std::max(da.first, std::min(da.second, s + sc * ds));
            const double nt = std::max(db.first, std::min(db.second, t + sc * dt));
            const double d = v2dist3(A.point_at(ns), B.point_at(nt));
            if (d <= best) {
                const double moved = std::abs(ns - s) + std::abs(nt - t);
                s = ns;
                t = nt;
                best = d;
                improved = true;
                if (moved < 1e-15 * ((da.second - da.first) + (db.second - db.first))) it = 100;
            }
        }
        if (!improved) break;
    }
    dout = best;
    return best <= tol;
}

}  // namespace

namespace {

/// Where does the section curve meet the SEAM of a periodic surface?
///
/// The seam is a property of the SURFACE -- the iso-line at the periodic boundary parameter --
/// not of any stored curve. Two measured reasons this must not be done any other way:
///   * create_sphere's stored seam EDGE misses its own sphere by 3.4e-4 and misses the true
///     section by 4.0e-3, so intersecting it finds nothing;
///   * NurbsSurface::iso_curve() ignores the rational weights, and on the same sphere returns
///     a curve of radius 1.1497 instead of 1.
/// So the crossing is solved directly against the surface: find (t, w) minimising
/// |C(t) - S(seam, w)|, a Gauss-Newton on the exact evaluator, bracketed by the UNWRAPPED
/// footprint trail (kb/port_08 S10 -> §2.8: the seam is detected by PARAMETER, then the point
/// is recomputed to land exactly on the boundary).
bool v2seam_refine(const NurbsCurve& c, const NurbsSurface& S, int dir, double seam,
                   double& t, double& w, double tol, const V2UvRect& R) {
    const std::pair<double, double> dc = c.domain();
    const double w0 = dir == 0 ? R.v0 : R.u0, w1 = dir == 0 ? R.v1 : R.u1;
    auto surf = [&](double ww) { return dir == 0 ? S.point_at(seam, ww) : S.point_at(ww, seam); };
    double best = v2dist3(c.point_at(t), surf(w));
    for (int it = 0; it < 60; ++it) {
        const std::vector<Vector> jc = c.evaluate(t, 1);
        const V2SurfJet js = v2surf_jet(S, dir == 0 ? seam : w, dir == 0 ? w : seam, 1);
        if (jc.size() < 2 || !js.ok) break;
        const Point Pc = c.point_at(t), Ps = surf(w);
        const double r[3] = {Pc[0] - Ps[0], Pc[1] - Ps[1], Pc[2] - Ps[2]};
        const double ct[3] = {jc[1][0], jc[1][1], jc[1][2]};
        const double sw[3] = {dir == 0 ? js.Sv[0] : js.Su[0], dir == 0 ? js.Sv[1] : js.Su[1],
                              dir == 0 ? js.Sv[2] : js.Su[2]};
        const double a11 = v2dot3(ct, ct), a12 = -v2dot3(ct, sw), a22 = v2dot3(sw, sw);
        const double b1 = -v2dot3(r, ct), b2 = v2dot3(r, sw);
        const double det = a11 * a22 - a12 * a12;
        if (std::abs(det) < 1e-30) break;
        const double dt = (b1 * a22 - a12 * b2) / det;
        const double dw = (a11 * b2 - a12 * b1) / det;
        bool improved = false;
        for (int bt = 0; bt < 12 && !improved; ++bt) {
            const double sc = std::pow(0.5, bt);
            const double nt = std::max(dc.first, std::min(dc.second, t + sc * dt));
            const double nw = std::max(w0, std::min(w1, w + sc * dw));
            const double d = v2dist3(c.point_at(nt), dir == 0 ? S.point_at(seam, nw)
                                                              : S.point_at(nw, seam));
            if (d < best) {
                t = nt;
                w = nw;
                best = d;
                improved = true;
            }
        }
        if (!improved) break;
    }
    return best <= tol;
}

}  // namespace

std::vector<std::pair<double, double>> v2_curve_curve_points(const NurbsCurve& a,
                                                            const NurbsCurve& b, double tol) {
    std::vector<std::pair<double, double>> out;
    if (!a.is_valid() || !b.is_valid()) return out;
    // Sag target: NOT driven by `tol`. Resolving a 1e-7 sag on a unit conic needs ~14000
    // chords, and the bracket does not need it — the bracket only has to ISOLATE a crossing,
    // and the 2x2 Newton below converges to machine precision from anywhere inside it. What
    // must be honest is the gate, and the gate is built on the MEASURED sag.
    double scale = 0.0;
    {
        const std::pair<double, double> da = a.domain();
        const Point p0 = a.point_at(da.first);
        for (int i = 1; i <= 8; ++i)
            scale = std::max(scale, v2dist3(p0, a.point_at(da.first + (da.second - da.first) * i / 8.0)));
    }
    const double sag = std::max(std::max(tol, 1e-12), 3e-6 * std::max(scale, 1e-6));
    std::vector<double> ta, tb;
    std::vector<Point> pa, pb;
    const double sa_meas = v2tess_curve(a, sag, ta, pa);
    const double sb_meas = v2tess_curve(b, sag, tb, pb);
    if (pa.size() < 2 || pb.size() < 2) return out;
    // If the true curves meet, their polylines come within sag_a + sag_b of each other.
    const double gate = tol + 2.0 * (sa_meas + sb_meas);

    // Chunked bounding boxes over `b` so the bracket search is not quadratic in the sample
    // count (12000^2 segment tests is minutes, not milliseconds).
    const int CH = 32;
    const int nch = ((int)pb.size() - 1 + CH - 1) / CH;
    std::vector<std::array<double, 6>> box((size_t)std::max(nch, 0));
    for (int ci = 0; ci < nch; ++ci) {
        std::array<double, 6> bb = {1e300, 1e300, 1e300, -1e300, -1e300, -1e300};
        const int k0 = ci * CH, k1 = std::min((int)pb.size(), (ci + 1) * CH + 1);
        for (int k = k0; k < k1; ++k)
            for (int d = 0; d < 3; ++d) {
                bb[d] = std::min(bb[d], pb[k][d]);
                bb[3 + d] = std::max(bb[3 + d], pb[k][d]);
            }
        box[ci] = bb;
    }

    const std::pair<double, double> da = a.domain();
    const double dedup = (da.second - da.first) * 1e-7;
    for (size_t i = 0; i + 1 < pa.size(); ++i) {
        double lo[3], hi[3];
        for (int d = 0; d < 3; ++d) {
            lo[d] = std::min(pa[i][d], pa[i + 1][d]) - gate;
            hi[d] = std::max(pa[i][d], pa[i + 1][d]) + gate;
        }
        for (int ci = 0; ci < nch; ++ci) {
            const std::array<double, 6>& bb = box[ci];
            if (bb[0] > hi[0] || bb[3] < lo[0] || bb[1] > hi[1] || bb[4] < lo[1] ||
                bb[2] > hi[2] || bb[5] < lo[2])
                continue;
            const int k0 = ci * CH, k1 = std::min((int)pb.size() - 1, (ci + 1) * CH);
            for (int k = k0; k < k1; ++k) {
                double sa = 0, sb = 0;
                const double d2 = v2seg_seg_dist2(pa[i], pa[i + 1], pb[k], pb[k + 1], sa, sb);
                if (d2 > gate * gate) continue;
                double sp = ta[i] + sa * (ta[i + 1] - ta[i]);
                double tp = tb[k] + sb * (tb[k + 1] - tb[k]);
                double dd = 0;
                if (!v2refine_cc(a, b, sp, tp, tol, dd)) continue;
                bool dup = false;
                for (const std::pair<double, double>& q : out)
                    if (std::abs(q.first - sp) <= dedup) {
                        dup = true;
                        break;
                    }
                if (!dup) out.push_back({sp, tp});
                if (out.size() > 64) return {};   // running ALONG the curve: not a set of paves
            }
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

///////////////////////////////////////////////////////////////////////////////////////////
// V2Section
///////////////////////////////////////////////////////////////////////////////////////////

V2Section::V2Section(BdsArena& ds, const std::vector<const BRep*>& operands,
                     const V2SectionParams& prm)
    : m_ds(ds), m_operands(operands), m_prm(prm) {}

V2FaceRef V2Section::face_ref(int arena_face) const {
    V2FaceRef r;
    if (!m_ds.is_face(arena_face)) return r;
    const BdsShape& sh = m_ds.shape(arena_face);
    if (sh.operand < 0 || sh.operand >= (int)m_operands.size()) return r;
    r.brep = m_operands[sh.operand];
    r.face = sh.source;
    return r;
}

const V2FaceClassifier& V2Section::classifier(int arena_face) const {
    if ((int)m_fc.size() <= arena_face) m_fc.resize(arena_face + 1);
    if (!m_fc[arena_face]) m_fc[arena_face] = std::make_shared<V2FaceClassifier>(face_ref(arena_face));
    return *m_fc[arena_face];
}

namespace {

/// Are the two surfaces tangent over the region where they touch? kb/port_04 G9: a tangency is
/// a TYPED outcome, never "Ok with 0 curves".
bool v2tangent_faces(const NurbsSurface& s1, const NurbsSurface& s2, double tol, bool& opposite) {
    const std::pair<double, double> u1 = s1.domain(0), v1 = s1.domain(1);
    double bd = 1e300, bu = 0, bv = 0;
    for (int i = 0; i <= 12; ++i)
        for (int k = 0; k <= 12; ++k) {
            const double uu = u1.first + (u1.second - u1.first) * i / 12.0;
            const double vv = v1.first + (v1.second - v1.first) * k / 12.0;
            const Point p = s1.point_at(uu, vv);
            double u2 = 0, v2 = 0;
            const double d = v2_invert(s2, p, u2, v2, false);
            if (d >= 0 && d < bd) {
                bd = d;
                bu = uu;
                bv = vv;
            }
        }
    if (bd > tol) return false;
    const Point p = s1.point_at(bu, bv);
    double u2 = 0, v2 = 0;
    v2_invert(s2, p, u2, v2, false);
    const Vector n1 = s1.normal_at(bu, bv), n2 = s2.normal_at(u2, v2);
    const double a[3] = {n1[0], n1[1], n1[2]}, b[3] = {n2[0], n2[1], n2[2]};
    const double na = v2nrm3(a), nb = v2nrm3(b);
    if (na < 1e-12 || nb < 1e-12) return false;
    const double c = v2dot3(a, b) / (na * nb);
    opposite = c < 0.0;
    return std::abs(c) > 1.0 - 1e-7;
}

}  // namespace

namespace {

/// Distance from `q` to the curve `C`, foot point sought from `t_seed` by Gauss-Newton on
/// (C(t) - q).C'(t) = 0. Used to MEASURE the sagitta of the trail's UV polygon: `q` is the
/// surface image of the polygon's midpoint, and the honest error is its distance to the curve
/// as a SET, not to the curve at some matched parameter (that would fold in a purely tangential
/// parameter mismatch and refine a trail that is already exact).
double v2curve_gap(const NurbsCurve& C, const Point& q, double t_seed,
                   const std::pair<double, double>& dc) {
    double t = std::max(dc.first, std::min(dc.second, t_seed));
    double best = v2dist3(C.point_at(t), q);
    for (int it = 0; it < 8; ++it) {
        const std::vector<Vector> j = C.evaluate(t, 1);
        if (j.size() < 2) break;
        const Point P = C.point_at(t);
        const double r[3] = {P[0] - q[0], P[1] - q[1], P[2] - q[2]};
        const double d1[3] = {j[1][0], j[1][1], j[1][2]};
        const double den = v2dot3(d1, d1);
        if (!(den > 1e-300)) break;
        const double step = -v2dot3(r, d1) / den;
        const double nt = std::max(dc.first, std::min(dc.second, t + step));
        const double nd = v2dist3(C.point_at(nt), q);
        if (!(nd < best)) break;
        best = nd;
        t = nt;
        if (std::fabs(step) < 1e-15 * (1.0 + std::fabs(t))) break;
    }
    return best;
}

}  // namespace

void V2Section::build_trail(V2Curve& c, const V2FaceRef& r1, const V2FaceRef& r2) {
    const NurbsSurface& S1 = r1.surface();
    const NurbsSurface& S2 = r2.surface();
    const V2UvRect R1 = v2_uv_rect(r1), R2 = v2_uv_rect(r2);
    const std::pair<double, double> d = c.c3d->domain();
    std::vector<double> ts;
    const int n = std::max(32, m_prm.trail_samples);
    for (int i = 0; i <= n; ++i) ts.push_back(d.first + (d.second - d.first) * i / n);
    for (double k : c.c3d->get_span_vector()) ts.push_back(k);
    std::sort(ts.begin(), ts.end());
    ts.erase(std::unique(ts.begin(), ts.end()), ts.end());

    // ONE trail sample, inverted onto both surfaces and unwrapped RELATIVE to `prev`
    // (kb/port_08 S10). The SEED is always the WRAPPED parameter (the surface can only be
    // evaluated inside its own rectangle); the UNWRAPPED value is carried alongside and stored
    // in the trail. Feeding an unwrapped seed back into the evaluator was the first defect of
    // this stage: it made every sample after the first seam approach evaluate out of range.
    auto sample = [&](double t, const V2PntOn2S* prev, double& u1w, double& v1w, double& u2w,
                      double& v2w) {
        V2PntOn2S s;
        s.t = t;
        s.p = c.c3d->point_at(t);
        const double dd1 = prev ? v2invert_newton(S1, s.p, u1w, v1w, R1)
                                : v2_invert(S1, s.p, u1w, v1w, false);
        const double dd2 = prev ? v2invert_newton(S2, s.p, u2w, v2w, R2)
                                : v2_invert(S2, s.p, u2w, v2w, false);
        double u1r = u1w, v1r = v1w, u2r = u2w, v2r = v2w;
        if (prev) {
            if (R1.u_period > 0) u1r += R1.u_period * std::round((prev->u1 - u1r) / R1.u_period);
            if (R1.v_period > 0) v1r += R1.v_period * std::round((prev->v1 - v1r) / R1.v_period);
            if (R2.u_period > 0) u2r += R2.u_period * std::round((prev->u2 - u2r) / R2.u_period);
            if (R2.v_period > 0) v2r += R2.v_period * std::round((prev->v2 - v2r) / R2.v_period);
        }
        s.u1 = u1r;
        s.v1 = v1r;
        s.u2 = u2r;
        s.v2 = v2r;
        if (dd1 >= 0) c.dev_all = std::max(c.dev_all, dd1);
        if (dd2 >= 0) c.dev_all = std::max(c.dev_all, dd2);
        return s;
    };

    c.trail.clear();
    c.trail.reserve(ts.size());
    {
        double u1w = 0, v1w = 0, u2w = 0, v2w = 0;
        for (size_t i = 0; i < ts.size(); ++i)
            c.trail.push_back(
                sample(ts[i], i ? &c.trail.back() : nullptr, u1w, v1w, u2w, v2w));
    }

    // ---- ADAPTIVE REFINEMENT ---------------------------------------------------------------
    // The caller imprints the trail's UV footprints on the face as a POLYGON, so the face
    // boundary carries exactly the sagitta of that polygon against the true section curve, and
    // that sagitta -- not the topology -- is what caps the boolean's closure residual. It is
    // MEASURED here, per segment, on BOTH surfaces, and the segment is bisected while it is
    // over target. A uniform sample count cannot do this: the same count is wasteful on a
    // near-straight section and insufficient on a tightly curved one.
    if (m_prm.trail_sag_rel > 0 && m_prm.trail_refine_passes > 0 && c.trail.size() >= 2) {
        double scale = 0.0;
        for (size_t i = 1; i < c.trail.size(); ++i)
            scale = std::max(scale, v2dist3(c.trail[i].p, c.trail[0].p));
        if (!(scale > 0)) scale = 1.0;
        const double target = std::max(m_prm.trail_sag_rel * scale, 1e-15);
        const size_t cap = (size_t)std::max(64, m_prm.trail_max_samples);
        auto seg_sag = [&](const V2PntOn2S& a, const V2PntOn2S& b) {
            const double tm = 0.5 * (a.t + b.t);
            const double uu1 = R1.u_closed ? v2wrap_into(0.5 * (a.u1 + b.u1), R1.u0, R1.u1)
                                           : 0.5 * (a.u1 + b.u1);
            const double vv1 = R1.v_closed ? v2wrap_into(0.5 * (a.v1 + b.v1), R1.v0, R1.v1)
                                           : 0.5 * (a.v1 + b.v1);
            const double uu2 = R2.u_closed ? v2wrap_into(0.5 * (a.u2 + b.u2), R2.u0, R2.u1)
                                           : 0.5 * (a.u2 + b.u2);
            const double vv2 = R2.v_closed ? v2wrap_into(0.5 * (a.v2 + b.v2), R2.v0, R2.v1)
                                           : 0.5 * (a.v2 + b.v2);
            const double g1 = v2curve_gap(*c.c3d, S1.point_at(uu1, vv1), tm, d);
            const double g2 = v2curve_gap(*c.c3d, S2.point_at(uu2, vv2), tm, d);
            return std::max(g1, g2);
        };
        for (int pass = 0; pass < m_prm.trail_refine_passes; ++pass) {
            if (c.trail.size() >= cap) break;
            std::vector<V2PntOn2S> out;
            out.reserve(c.trail.size() * 2);
            out.push_back(c.trail.front());
            bool any = false;
            for (size_t i = 0; i + 1 < c.trail.size(); ++i) {
                const V2PntOn2S& a = c.trail[i];
                const V2PntOn2S& b = c.trail[i + 1];
                const double tm = 0.5 * (a.t + b.t);
                if (out.size() + (c.trail.size() - i) < cap && b.t - a.t > 1e-13 * (d.second - d.first) &&
                    seg_sag(a, b) > target) {
                    double u1w = a.u1, v1w = a.v1, u2w = a.u2, v2w = a.v2;
                    if (R1.u_closed) u1w = v2wrap_into(u1w, R1.u0, R1.u1);
                    if (R1.v_closed) v1w = v2wrap_into(v1w, R1.v0, R1.v1);
                    if (R2.u_closed) u2w = v2wrap_into(u2w, R2.u0, R2.u1);
                    if (R2.v_closed) v2w = v2wrap_into(v2w, R2.v0, R2.v1);
                    out.push_back(sample(tm, &a, u1w, v1w, u2w, v2w));
                    any = true;
                }
                out.push_back(b);
            }
            if (!any) break;
            c.trail.swap(out);
        }
        if (v2sec_dbg()) {
            double worst = 0.0;
            for (size_t i = 0; i + 1 < c.trail.size(); ++i)
                worst = std::max(worst, seg_sag(c.trail[i], c.trail[i + 1]));
            std::printf("[SECDBG] trail c(%d,%d) n=%zu scale=%.6g target=%.3e worst_sag=%.3e\n",
                        c.face1, c.face2, c.trail.size(), scale, target, worst);
        }
    }
}

/// G8 — MEASURE the curve/surface deviation, and measure it only where the section is CLAIMED
/// to be: over the KEPT blocks. Measuring over the whole analytic carrier would fold in the
/// distance from the parts that lie off the trimmed patch, which is not a tolerance of
/// anything (that mistake inflated this stage's tolerance to 1.2 and made every block pass
/// the trim test on the first run).
void V2Section::measure_tolerance(V2Curve& c) {
    const V2FaceClassifier& C1 = classifier(c.face1);
    const V2FaceClassifier& C2 = classifier(c.face2);
    c.dev1 = 0;
    c.dev2 = 0;
    for (const V2Block& b : c.blocks) {
        if (!b.kept) continue;
        for (int i = 0; i <= 24; ++i) {
            const Point P = c.c3d->point_at(b.t0 + (b.t1 - b.t0) * i / 24.0);
            double u = 0, v = 0;
            const double d1 = C1.invert(P, u, v, false);
            double u2 = 0, v2 = 0;
            const double d2 = C2.invert(P, u2, v2, false);
            if (d1 >= 0) c.dev1 = std::max(c.dev1, d1);
            if (d2 >= 0) c.dev2 = std::max(c.dev2, d2);
        }
    }
    c.tol = std::max(c.tol, std::max(c.dev1, c.dev2) * (1.0 + 1e-5));
}

void V2Section::collect_paves(V2Curve& c, const V2FaceRef& r1, const V2FaceRef& r2) {
    const std::pair<double, double> d = c.c3d->domain();
    const double tol = c.tol;
    c.paves.clear();

    struct Cand {
        double t;
        V2PaveOrigin o;
        int face;
        int edge;
    };
    std::vector<Cand> cand;

    const V2FaceRef refs[2] = {r1, r2};
    const int afaces[2] = {c.face1, c.face2};
    for (int side = 0; side < 2; ++side) {
        const BRep& B = *refs[side].brep;
        for (int li : B.m_faces[refs[side].face].loop_indices) {
            if (li < 0 || li >= (int)B.m_loops.size()) continue;
            for (int ti : B.m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)B.m_trims.size()) continue;
                const BRepTrim& tr = B.m_trims[ti];
                // SEAMS are handled below from the surface's own ISO-CURVE, not from the
                // stored 3D edge. Measured on create_sphere: the stored seam meridian misses
                // the sphere by 3.4e-4 and misses the true section by 4e-3, so intersecting it
                // finds no crossing at all. The seam is a property of the SURFACE (a parameter
                // boundary, kb/port_08 §2.8 detects it by parameter, never by a stored curve),
                // and the exact iso-curve is the entity that owns it.
                if (tr.type == BRepTrimType::Seam) continue;
                if (tr.edge_index < 0 || tr.edge_index >= (int)B.m_topology_edges.size()) continue;
                const int ci = B.m_topology_edges[tr.edge_index].curve_3d_index;
                if (ci < 0 || ci >= (int)B.m_curves_3d.size()) continue;
                const NurbsCurve& E = B.m_curves_3d[ci];
                if (!E.is_valid()) continue;
                // kb/port_08 S6: a DEGENERATE (pole) edge is never an interference operand.
                const std::pair<double, double> de = E.domain();
                double ext = 0.0;
                const Point e0 = E.point_at(de.first);
                for (int k = 1; k <= 4; ++k)
                    ext = std::max(ext, v2dist3(e0, E.point_at(de.first + (de.second - de.first) * k / 4.0)));
                if (ext <= std::max(tol, 1e-9)) continue;
                const V2PaveOrigin org = (tr.type == BRepTrimType::Seam)
                                             ? V2PaveOrigin::SeamCrossing
                                             : V2PaveOrigin::TrimCrossing;
                for (const std::pair<double, double>& q : v2_curve_curve_points(*c.c3d, E, tol))
                    cand.push_back({q.first, org, afaces[side], tr.edge_index});
            }
        }
    }

    // Seam crossings, bracketed by the unwrapped footprint trail and solved against the
    // SURFACE (see v2seam_refine): the stored seam edge and NurbsSurface::iso_curve are both
    // too inaccurate to be intersection operands here, and both failures were measured.
    if (m_prm.seam_split && c.trail.size() > 1) {
        for (int side = 0; side < 2; ++side) {
            const BRep& B = *refs[side].brep;
            const NurbsSurface& S = refs[side].surface();
            const V2UvRect R = v2_uv_rect(refs[side]);
            int seam_edge = -1;
            for (int li : B.m_faces[refs[side].face].loop_indices)
                for (int ti : B.m_loops[li].trim_indices)
                    if (B.m_trims[ti].type == BRepTrimType::Seam && seam_edge < 0)
                        seam_edge = B.m_trims[ti].edge_index;
            if (v2sec_dbg()) {
                double lo0 = 1e300, hi0 = -1e300, lo1 = 1e300, hi1 = -1e300;
                for (const V2PntOn2S& s : c.trail) {
                    const double pu = side == 0 ? s.u1 : s.u2;
                    const double pv = side == 0 ? s.v1 : s.v2;
                    lo0 = std::min(lo0, pu); hi0 = std::max(hi0, pu);
                    lo1 = std::min(lo1, pv); hi1 = std::max(hi1, pv);
                }
                std::printf("[SECDBG] c(%d,%d) side=%d af=%d seam_edge=%d rect u[%.6f,%.6f]"
                            " per_u=%.6f v[%.6f,%.6f] per_v=%.6f trail u[%.6f,%.6f]"
                            " v[%.6f,%.6f]\n",
                            c.face1, c.face2, side, afaces[side], seam_edge, R.u0, R.u1,
                            R.u_period, R.v0, R.v1, R.v_period, lo0, hi0, lo1, hi1);
            }
            for (int dir = 0; dir < 2; ++dir) {
                const double period = dir == 0 ? R.u_period : R.v_period;
                if (!(period > 0)) continue;
                const double base = dir == 0 ? R.u0 : R.v0;
                for (size_t i = 0; i + 1 < c.trail.size(); ++i) {
                    const V2PntOn2S& A = c.trail[i];
                    const V2PntOn2S& Bp = c.trail[i + 1];
                    const double pa = side == 0 ? (dir == 0 ? A.u1 : A.v1)
                                                : (dir == 0 ? A.u2 : A.v2);
                    const double pb = side == 0 ? (dir == 0 ? Bp.u1 : Bp.v1)
                                                : (dir == 0 ? Bp.u2 : Bp.v2);
                    const double wa = side == 0 ? (dir == 0 ? A.v1 : A.u1)
                                                : (dir == 0 ? A.v2 : A.u2);
                    // A SEAM VALUE THAT LANDS EXACTLY ON A TRAIL SAMPLE IS STILL A CROSSING.
                    // The bracket used to be the OPEN interval (min,max) of the segment's two
                    // footprint parameters. MEASURED on cone x cone at the identity pose: the
                    // section circle A_lateral x B_base has its footprint on the cone's OWN seam
                    // at a span knot of the NURBS circle, so the inversion returns u = 0.0
                    // EXACTLY -- and the open test then rejects it from the segment before (whose
                    // hi is 0) and from the segment after (whose lo is 0). The crossing was lost,
                    // A's seam edge received one pave instead of two, its lateral face came back
                    // as 2 images instead of 3, the wire walk dead-ended, and the whole front end
                    // refused. The bracket is therefore the CLOSED interval; the duplicate that a
                    // shared sample produces from both neighbouring segments lands on the same t
                    // and is removed by the dedup below.
                    //
                    // A segment whose footprint parameter does not move at all (a section that
                    // RUNS ALONG the seam) would otherwise put a candidate on every one of its
                    // samples, so a zero-span segment is skipped: a genuine crossing at a shared
                    // sample always has at least one neighbour with a non-zero span.
                    if (!(std::fabs(pb - pa) > 0.0)) continue;
                    const double lo = std::min(pa, pb), hi = std::max(pa, pb);
                    const double keps = 1e-12 * (std::fabs(period) + std::fabs(hi - lo));
                    const long k0 = (long)std::ceil((lo - base) / period - 1e-9);
                    const long k1 = (long)std::floor((hi - base) / period + 1e-9);
                    for (long k = k0; k <= k1; ++k) {
                        const double sv = base + k * period;
                        if (sv < lo - keps || sv > hi + keps) continue;
                        double t = A.t + (Bp.t - A.t) * (sv - pa) / (pb - pa);
                        t = std::max(std::min(A.t, Bp.t), std::min(std::max(A.t, Bp.t), t));
                        double w = wa;
                        const double t_guess = t;
                        const bool okr =
                            v2seam_refine(*c.c3d, S, dir, base, t, w, std::max(tol, 1e-9), R);
                        if (v2sec_dbg())
                            std::printf("[SECDBG]   c(%d,%d) side=%d dir=%d k=%ld sv=%.9f "
                                        "t_guess=%.9f -> t=%.9f w=%.9f refine=%d tol=%.3e\n",
                                        c.face1, c.face2, side, dir, k, sv, t_guess, t, w,
                                        (int)okr, std::max(tol, 1e-9));
                        if (!okr) continue;
                        cand.push_back({t, V2PaveOrigin::SeamCrossing, afaces[side], seam_edge});
                    }
                }
            }
        }
    }

    std::sort(cand.begin(), cand.end(), [](const Cand& a, const Cand& b) { return a.t < b.t; });
    const double dt = (d.second - d.first) * 1e-7;
    std::vector<Cand> uniq;
    for (const Cand& q : cand) {
        if (!uniq.empty() && std::abs(q.t - uniq.back().t) <= dt) continue;
        if (q.t <= d.first + dt || q.t >= d.second - dt) continue;   // folded into the bounds
        uniq.push_back(q);
    }

    V2Pave p0;
    p0.t = d.first;
    p0.origin = c.closed ? V2PaveOrigin::Closing : V2PaveOrigin::CurveBound;
    c.paves.push_back(p0);
    for (const Cand& q : uniq) {
        V2Pave p;
        p.t = q.t;
        p.origin = q.o;
        p.face = q.face;
        p.edge = q.edge;
        c.paves.push_back(p);
        if (q.o == V2PaveOrigin::SeamCrossing) ++c.seam_paves;
    }
    V2Pave p1;
    p1.t = d.second;
    p1.origin = c.closed ? V2PaveOrigin::Closing : V2PaveOrigin::CurveBound;
    c.paves.push_back(p1);
}

void V2Section::make_blocks(V2Curve& c, const V2FaceRef&, const V2FaceRef&) {
    const std::pair<double, double> d = c.c3d->domain();
    // 1. carrier edge in the arena, with the curve's own bounds as paves
    const int va = m_ds.append_vertex(c.c3d->point_at(d.first), c.tol);
    const int vb = c.closed ? va : m_ds.append_vertex(c.c3d->point_at(d.second), c.tol);
    c.carrier_edge = m_ds.append_edge(*c.c3d, va, vb, c.tol);
    c.paves.front().vertex = va;
    c.paves.back().vertex = vb;

    // 2. interior paves — every one names a real entity (G3)
    for (size_t i = 1; i + 1 < c.paves.size(); ++i) {
        V2Pave& p = c.paves[i];
        const int v = m_ds.append_vertex(c.c3d->point_at(p.t), c.tol);
        if (m_ds.add_pave(c.carrier_edge, p.t, v, m_prm.fuzzy) != BdsArena::PaveStatus::Inserted) {
            p.vertex = -1;   // refused (coincident with an existing pave): not a separate node
            continue;
        }
        p.vertex = v;
    }
    c.paves.erase(std::remove_if(c.paves.begin() + 1, c.paves.end() - 1,
                                 [](const V2Pave& p) { return p.vertex < 0; }),
                  c.paves.end() - 1);
    m_ds.update_pave_blocks(c.carrier_edge);

    // 3. one 43.2% verdict per block; kept or dropped WHOLE, nothing is bisected (G2/G3)
    const V2FaceClassifier& C1 = classifier(c.face1);
    const V2FaceClassifier& C2 = classifier(c.face2);
    const std::vector<BdsPB>& pool = m_ds.pave_blocks(c.carrier_edge);
    c.blocks.clear();
    for (const BdsPB& pb : pool) {
        V2Block b;
        pb->range(b.t0, b.t1);
        b.pb = pb;
        b.v0 = pb->pave1.vertex;
        b.v1 = pb->pave2.vertex;
        const double tm = v2_intermediate_point(b.t0, b.t1);
        const Point P = c.c3d->point_at(tm);
        b.kept = C1.valid_point(P, c.tol) && C2.valid_point(P, c.tol);
        if (b.kept) {
            NurbsCurve piece = *c.c3d;
            const bool full = (b.t0 <= d.first + 1e-15) && (b.t1 >= d.second - 1e-15);
            if (!full) piece.trim(b.t0, b.t1);
            if (piece.is_valid()) {
                b.length = v2sec_arclen(*c.c3d, b.t0, b.t1);
                b.edge = m_ds.append_edge(piece, b.v0, b.v1, c.tol);
                pb->edge = b.edge;
            } else {
                b.kept = false;
            }
        }
        c.blocks.push_back(b);
    }
}

V2FFStatus V2Section::section_for_pair(int f1, int f2, std::vector<V2Curve>& out) {
    const V2FaceRef r1 = face_ref(f1), r2 = face_ref(f2);
    if (!r1.valid() || !r2.valid()) return V2FFStatus::Failed;
    if (m_ds.shape(f1).box.is_out(m_ds.shape(f2).box)) return V2FFStatus::Empty;

    const NurbsSurface& S1 = r1.surface();
    const NurbsSurface& S2 = r2.surface();
    const double tolF = std::max(m_ds.tolerance(f1), m_ds.tolerance(f2));
    const double tol = std::max(tolF, m_prm.fuzzy);

    // L1: the EXISTING analytic SSI. Exact for every recognised quadric pair, and
    // rigid-motion equivariant; never reimplemented here.
    const std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> trs =
        Intersection::surface_surface(S1, S2, tol);
    if (trs.empty()) {
        bool opp = false;
        if (v2tangent_faces(S1, S2, tol * 10.0, opp)) return V2FFStatus::Tangent;
        return V2FFStatus::NoGeometricSolution;
    }

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& tr : trs) {
        const NurbsCurve& c3 = std::get<0>(tr);
        if (!c3.is_valid()) continue;
        V2Curve c;
        c.face1 = f1;
        c.face2 = f2;
        c.c3d = std::make_shared<NurbsCurve>(c3);
        const std::pair<double, double> d = c.c3d->domain();
        c.closed = v2dist3(c.c3d->point_at(d.first), c.c3d->point_at(d.second)) <=
                   std::max(tol, 1e-9);
        // The L3 verdict tolerance is the FACE tolerance (OCCT's tolR3D = max(tolF1,tolF2)
        // floored at the fuzzy value), never the curve's own global deviation.
        c.tol = tol;
        c.tang_tol = tolF;
        build_trail(c, r1, r2);
        out.push_back(std::move(c));
    }
    return out.empty() ? V2FFStatus::NoGeometricSolution : V2FFStatus::Ok;
}

V2FFStatus V2Section::perform_pair(int f1, int f2) {
    ++m_stats.pairs;
    std::vector<V2Curve> made;
    const V2FFStatus st = section_for_pair(f1, f2, made);
    switch (st) {
        case V2FFStatus::Ok: ++m_stats.ok; break;
        case V2FFStatus::Tangent: ++m_stats.tangent; break;
        case V2FFStatus::Empty: ++m_stats.empty; break;
        case V2FFStatus::NoGeometricSolution: ++m_stats.nogeom; break;
        default: ++m_stats.failed; break;
    }
    if (st != V2FFStatus::Ok) return st;

    const V2FaceRef r1 = face_ref(f1), r2 = face_ref(f2);
    for (V2Curve& c : made) {
        c.pair = m_stats.pairs - 1;
        collect_paves(c, r1, r2);
        make_blocks(c, r1, r2);
        measure_tolerance(c);
        m_stats.max_dev = std::max(m_stats.max_dev, std::max(c.dev1, c.dev2));
        for (const V2Pave& p : c.paves) {
            if (p.origin == V2PaveOrigin::TrimCrossing) ++m_stats.trim_paves;
            else if (p.origin == V2PaveOrigin::SeamCrossing) ++m_stats.seam_paves;
            else ++m_stats.bound_paves;
        }
        for (const V2Block& b : c.blocks) {
            ++m_stats.blocks;
            if (b.kept) ++m_stats.kept; else ++m_stats.dropped;
        }
        ++m_stats.curves;
        m_curves.push_back(std::move(c));
    }
    return st;
}

int V2Section::perform_all() {
    std::vector<int> fa, fb;
    for (int i = 0; i < m_ds.nb_shapes(); ++i) {
        if (!m_ds.is_face(i)) continue;
        const int rk = m_ds.rank(i);
        if (rk == 0) fa.push_back(i);
        else if (rk == 1) fb.push_back(i);
    }
    int ok = 0;
    for (int a : fa)
        for (int b : fb)
            if (perform_pair(a, b) == V2FFStatus::Ok) ++ok;
    if (m_prm.posttreat) post_treat_ff();
    return ok;
}

///////////////////////////////////////////////////////////////////////////////////////////
// PostTreatFF — cross-pair fusion. Law 1 for section curves.
///////////////////////////////////////////////////////////////////////////////////////////

namespace {

struct V2UF {
    std::vector<int> p;
    void reset(int n) {
        p.resize(n);
        for (int i = 0; i < n; ++i) p[i] = i;
    }
    int find(int i) {
        while (p[i] != i) {
            p[i] = p[p[i]];
            i = p[i];
        }
        return i;
    }
    void join(int a, int b) {
        a = find(a);
        b = find(b);
        if (a != b) p[a] = b;
    }
};

}  // namespace

void V2Section::post_treat_ff() {
    m_fused.clear();

    // --- 1. one arena vertex per geometric node -------------------------------------------
    std::vector<int> nodes;
    for (const V2Curve& c : m_curves)
        for (const V2Block& b : c.blocks)
            if (b.kept) {
                nodes.push_back(b.v0);
                nodes.push_back(b.v1);
            }
    std::sort(nodes.begin(), nodes.end());
    nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());

    V2UF uf;
    uf.reset((int)nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i)
        for (size_t k = i + 1; k < nodes.size(); ++k) {
            const double band = m_ds.tolerance(nodes[i]) + m_ds.tolerance(nodes[k]) + m_prm.fuzzy;
            if (v2dist3(m_ds.vertex_point(nodes[i]), m_ds.vertex_point(nodes[k])) <= band)
                uf.join((int)i, (int)k);
        }
    std::map<int, std::vector<int>> groups;
    for (size_t i = 0; i < nodes.size(); ++i) groups[uf.find((int)i)].push_back(nodes[i]);
    for (const std::pair<const int, std::vector<int>>& g : groups) {
        if (g.second.size() < 2) continue;
        m_ds.fuse_vertices(g.second);
        m_fused.push_back(g.second);
    }
    m_stats.fused_groups = (int)m_fused.size();
    // rewrite every reference through the same-domain map
    for (V2Curve& c : m_curves) {
        for (V2Pave& p : c.paves)
            if (p.vertex >= 0) {
                const int r = m_ds.resolve_sd(p.vertex);
                if (r != p.vertex) {
                    p.vertex = r;
                    p.fused = true;
                }
            }
        for (V2Block& b : c.blocks) {
            b.v0 = m_ds.resolve_sd(b.v0);
            b.v1 = m_ds.resolve_sd(b.v1);
        }
    }

    // --- 2. one arena edge per geometric curve --------------------------------------------
    struct Ref {
        int ci, bi;
    };
    std::vector<Ref> kept;
    for (size_t i = 0; i < m_curves.size(); ++i)
        for (size_t k = 0; k < m_curves[i].blocks.size(); ++k)
            if (m_curves[i].blocks[k].kept) kept.push_back({(int)i, (int)k});

    V2UF ub;
    ub.reset((int)kept.size());
    for (size_t i = 0; i < kept.size(); ++i)
        for (size_t k = i + 1; k < kept.size(); ++k) {
            if (kept[i].ci == kept[k].ci) continue;   // same curve: never fused with itself
            const BdsPB& a = m_curves[kept[i].ci].blocks[kept[i].bi].pb;
            const BdsPB& b = m_curves[kept[k].ci].blocks[kept[k].bi].pb;
            if (!a || !b) continue;
            if (bds_check_coincidence(m_ds, a, b, m_prm.fuzzy) &&
                bds_check_coincidence(m_ds, b, a, m_prm.fuzzy))
                ub.join((int)i, (int)k);
        }
    std::map<int, std::vector<int>> bg;
    for (size_t i = 0; i < kept.size(); ++i) bg[ub.find((int)i)].push_back((int)i);
    m_stats.common_blocks = 0;
    for (const std::pair<const int, std::vector<int>>& g : bg) {
        if (g.second.size() < 2) continue;
        std::vector<BdsPB> members;
        std::vector<int> faces;
        int emin = -1;
        for (int i : g.second) {
            const V2Curve& c = m_curves[kept[i].ci];
            const V2Block& b = c.blocks[kept[i].bi];
            members.push_back(b.pb);
            faces.push_back(c.face1);
            faces.push_back(c.face2);
            if (emin < 0 || (b.edge >= 0 && b.edge < emin)) emin = b.edge;
        }
        BdsCB cb = m_ds.make_common_block(members);
        if (!cb) continue;
        std::sort(faces.begin(), faces.end());
        faces.erase(std::unique(faces.begin(), faces.end()), faces.end());
        for (int f : faces)
            if (!cb->contains_face(f)) cb->faces.push_back(f);
        if (emin >= 0) cb->set_edge(emin);   // ONE materialised edge for EVERY member (I2)
        for (int i : g.second) m_curves[kept[i].ci].blocks[kept[i].bi].edge = emin;
        ++m_stats.common_blocks;
    }
    m_stats.section_edges = (int)section_edges().size();
}

///////////////////////////////////////////////////////////////////////////////////////////
// reporting
///////////////////////////////////////////////////////////////////////////////////////////

std::vector<int> V2Section::section_edges() const {
    std::vector<int> e;
    for (const V2Curve& c : m_curves)
        for (const V2Block& b : c.blocks)
            if (b.kept && b.edge >= 0) e.push_back(b.edge);
    std::sort(e.begin(), e.end());
    e.erase(std::unique(e.begin(), e.end()), e.end());
    return e;
}

std::vector<int> V2Section::section_nodes() const {
    std::vector<int> v;
    for (const V2Curve& c : m_curves)
        for (const V2Block& b : c.blocks)
            if (b.kept) {
                v.push_back(m_ds.resolve_sd(b.v0));
                v.push_back(m_ds.resolve_sd(b.v1));
            }
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
    return v;
}

int V2Section::distinct_node_locations(double tol) const {
    const std::vector<int> v = section_nodes();
    V2UF uf;
    uf.reset((int)v.size());
    for (size_t i = 0; i < v.size(); ++i)
        for (size_t k = i + 1; k < v.size(); ++k)
            if (v2dist3(m_ds.vertex_point(v[i]), m_ds.vertex_point(v[k])) <= tol) uf.join((int)i, (int)k);
    std::set<int> roots;
    for (size_t i = 0; i < v.size(); ++i) roots.insert(uf.find((int)i));
    return (int)roots.size();
}

int V2Section::component_count() const {
    struct Ref {
        int ci, bi;
    };
    std::vector<Ref> kept;
    for (size_t i = 0; i < m_curves.size(); ++i)
        for (size_t k = 0; k < m_curves[i].blocks.size(); ++k)
            if (m_curves[i].blocks[k].kept) kept.push_back({(int)i, (int)k});
    if (kept.empty()) return 0;
    V2UF uf;
    uf.reset((int)kept.size());
    for (size_t i = 0; i < kept.size(); ++i) {
        const V2Block& a = m_curves[kept[i].ci].blocks[kept[i].bi];
        for (size_t k = i + 1; k < kept.size(); ++k) {
            const V2Block& b = m_curves[kept[k].ci].blocks[kept[k].bi];
            const int a0 = m_ds.resolve_sd(a.v0), a1 = m_ds.resolve_sd(a.v1);
            const int b0 = m_ds.resolve_sd(b.v0), b1 = m_ds.resolve_sd(b.v1);
            if (a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1) uf.join((int)i, (int)k);
        }
    }
    std::set<int> roots;
    for (size_t i = 0; i < kept.size(); ++i) roots.insert(uf.find((int)i));
    return (int)roots.size();
}

double V2Section::kept_length() const {
    double s = 0;
    for (const V2Curve& c : m_curves)
        for (const V2Block& b : c.blocks)
            if (b.kept) s += b.length;
    return s;
}

double V2Section::g1_residual() const {
    double worst = 0;
    for (const V2Curve& c : m_curves) {
        const V2FaceClassifier& C1 = classifier(c.face1);
        const V2FaceClassifier& C2 = classifier(c.face2);
        for (const V2Block& b : c.blocks) {
            if (!b.kept) continue;
            for (int i = 0; i <= 32; ++i) {
                const double t = b.t0 + (b.t1 - b.t0) * i / 32.0;
                const Point P = c.c3d->point_at(t);
                double u = 0, v = 0;
                const double d1 = C1.invert(P, u, v, false);
                double u2 = 0, v2 = 0;
                const double d2 = C2.invert(P, u2, v2, false);
                worst = std::max(worst, std::max(d1, d2));
            }
        }
    }
    return worst;
}

std::string V2Section::signature() const {
    char buf[512];
    std::string s;
    std::snprintf(buf, sizeof(buf),
                  "pairs=%d ok=%d tan=%d empty=%d nogeom=%d fail=%d curves=%d blocks=%d kept=%d "
                  "edges=%d nodes=%d comps=%d fused=%d cb=%d\n",
                  m_stats.pairs, m_stats.ok, m_stats.tangent, m_stats.empty, m_stats.nogeom,
                  m_stats.failed, m_stats.curves, m_stats.blocks, m_stats.kept,
                  (int)section_edges().size(), (int)section_nodes().size(), component_count(),
                  m_stats.fused_groups, m_stats.common_blocks);
    s += buf;
    for (const V2Curve& c : m_curves) {
        std::snprintf(buf, sizeof(buf), "  c f=%d/%d closed=%d paves=%d blocks=%d seam=%d len=%.9f\n",
                      c.face1, c.face2, (int)c.closed, (int)c.paves.size(), (int)c.blocks.size(),
                      c.seam_paves, [&] {
                          double l = 0;
                          for (const V2Block& b : c.blocks)
                              if (b.kept) l += b.length;
                          return l;
                      }());
        s += buf;
    }
    return s;
}

}  // namespace v2sec
}  // namespace session_cpp
