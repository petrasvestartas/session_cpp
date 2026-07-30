#include "brep_massprops.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace session_cpp {
namespace {

///////////////////////////////////////////////////////////////////////////////////////////
// Gauss-Kronrod 7/15 (QUADPACK qk15 constants). The 15 abscissae give BOTH the 15-point
// Kronrod estimate and the embedded 7-point Gauss estimate, so every panel carries an error
// estimate at no extra cost. Kronrod-15 is exact for polynomials of degree <= 22.
///////////////////////////////////////////////////////////////////////////////////////////

constexpr int NC = 8;  // 0: area, 1: flux (3V), 2..4: first moments, 5..7: vector area

const double kXGK[8] = {0.99145537112081263921, 0.94910791234275852453, 0.86486442335976907279,
                        0.74153118559939443986, 0.58608723546769113029, 0.40584515137739716691,
                        0.20778495500789846760, 0.00000000000000000000};
const double kWGK[8] = {0.02293532201052922496, 0.06309209262997855329, 0.10479001032225018384,
                        0.14065325971552591875, 0.16900472663926790283, 0.19035057806478540991,
                        0.20443294007529889241, 0.20948214108472782801};
const double kWG[4] = {0.12948496616886969327, 0.27970539148927666790, 0.38183005050511894495,
                       0.41795918367346938776};

struct Quad {
    double r[NC];  // integral estimate (Kronrod 15)
    double e[NC];  // absolute error estimate
    double a[NC];  // integral of |f|: the magnitude reference of the relative criterion
    void zero() { for (int k = 0; k < NC; ++k) r[k] = e[k] = a[k] = 0.0; }
};

template <class F>
void qk15(F& f, double a, double b, Quad& q) {
    const double c = 0.5 * (a + b), h = 0.5 * (b - a), ah = std::abs(h);
    double fc[NC], fv1[7][NC], fv2[7][NC];
    f(c, fc);
    for (int j = 0; j < 7; ++j) {
        const double x = h * kXGK[j];
        f(c - x, fv1[j]);
        f(c + x, fv2[j]);
    }
    for (int k = 0; k < NC; ++k) {
        double resg = kWG[3] * fc[k];
        double resk = kWGK[7] * fc[k];
        double resabs = kWGK[7] * std::abs(fc[k]);
        for (int j = 0; j < 7; ++j) {
            const double s = fv1[j][k] + fv2[j][k];
            resk += kWGK[j] * s;
            resabs += kWGK[j] * (std::abs(fv1[j][k]) + std::abs(fv2[j][k]));
        }
        for (int j = 0; j < 3; ++j) {
            const int idx = 2 * j + 1;
            resg += kWG[j] * (fv1[idx][k] + fv2[idx][k]);
        }
        const double reskh = 0.5 * resk;
        double resasc = kWGK[7] * std::abs(fc[k] - reskh);
        for (int j = 0; j < 7; ++j)
            resasc += kWGK[j] * (std::abs(fv1[j][k] - reskh) + std::abs(fv2[j][k] - reskh));
        resasc *= ah;
        double abserr = std::abs((resk - resg) * h);
        if (resasc != 0.0 && abserr != 0.0)
            abserr = resasc * std::min(1.0, std::pow(200.0 * abserr / resasc, 1.5));
        q.r[k] = resk * h;
        q.a[k] = resabs * ah;
        q.e[k] = std::max(abserr, 50.0 * std::numeric_limits<double>::epsilon() * q.a[k]);
    }
}

struct Panel {
    double a, b, emax;
    int depth;
    Quad q;
};

/// Globally-adaptive Gauss-Kronrod over a fixed break-point list.
///
/// TERMINATION PROOF: the worklist grows by exactly one panel per iteration, the loop
/// condition is `panels < cap`, `cap` is a finite constant, every panel has `depth <
/// max_depth`, and every integrand call decrements `budget_left` which is also checked.
/// So the loop executes at most `cap` times and always returns a value plus its error
/// estimate. `hit_cap` reports that a bound (not the tolerance) ended the refinement.
struct GKStats {
    long long calls = 0, panels = 0, caps = 0;
    double worst_ratio = 0.0;
    int worst_k = -1;
};

/// `atol` is a per-component ABSOLUTE floor derived from the model's own geometric scale.
/// Without it a component whose exact value is zero by symmetry (every first moment of a
/// centred solid) makes the relative test compare roundoff against roundoff, which never
/// converges and burns the entire budget; the floor makes such a component converge at once.
template <class F>
void adaptive_gk(F& f, const std::vector<double>& breaks, double rtol, const double* atol,
                 int max_panels, int max_depth, long long& budget_left, Quad& out, bool& hit_cap,
                 GKStats* st = nullptr) {
    out.zero();
    hit_cap = false;
    if (breaks.size() < 2) return;

    // The INITIAL panel list is always integrated in full -- its size is bounded by the
    // caller's break list, so this is a fixed cost, and skipping panels here would silently
    // drop whole chunks of the integral rather than merely lose accuracy. Only the adaptive
    // refinement below is budget-gated.
    std::vector<Panel> pan;
    pan.reserve(breaks.size() + static_cast<size_t>(std::max(max_panels, 0)));
    for (size_t i = 0; i + 1 < breaks.size(); ++i) {
        if (!(breaks[i + 1] > breaks[i])) continue;
        Panel p;
        p.a = breaks[i];
        p.b = breaks[i + 1];
        p.depth = 0;
        qk15(f, p.a, p.b, p.q);
        p.emax = 0.0;
        for (int k = 0; k < NC; ++k) p.emax = std::max(p.emax, p.q.e[k]);
        pan.push_back(p);
    }
    if (pan.empty()) return;

    auto retotal = [&]() {
        out.zero();
        for (const Panel& p : pan)
            for (int k = 0; k < NC; ++k) {
                out.r[k] += p.q.r[k];
                out.e[k] += p.q.e[k];
                out.a[k] += p.q.a[k];
            }
    };
    auto converged = [&]() {
        bool ok = true;
        for (int k = 0; k < NC; ++k) {
            if (st) {
                const double den = std::max(out.a[k], std::abs(out.r[k]));
                const double ratio = den > 0.0 ? out.e[k] / den : (out.e[k] > 0.0 ? 1e300 : 0.0);
                if (ratio > st->worst_ratio) { st->worst_ratio = ratio; st->worst_k = k; }
            }
            const double lim = std::max({rtol * out.a[k], rtol * std::abs(out.r[k]),
                                        atol ? atol[k] : 0.0});
            if (out.e[k] > lim) ok = false;
        }
        return ok;
    };
    retotal();

    const int cap = std::max<int>(max_panels, static_cast<int>(pan.size()) + 1);
    bool done = false;
    while (!done && static_cast<int>(pan.size()) < cap) {
        if (budget_left <= 0) { hit_cap = true; done = true; break; }
        if (converged()) { done = true; break; }
        size_t worst = pan.size();
        double we = -1.0;
        for (size_t i = 0; i < pan.size(); ++i)
            if (pan[i].emax > we && pan[i].depth < max_depth) { we = pan[i].emax; worst = i; }
        if (worst == pan.size()) { hit_cap = true; done = true; break; }  // all depth-capped
        const double m = 0.5 * (pan[worst].a + pan[worst].b);
        if (!(m > pan[worst].a && m < pan[worst].b)) { pan[worst].depth = max_depth; continue; }
        Panel lo, hi;
        lo.a = pan[worst].a; lo.b = m; lo.depth = pan[worst].depth + 1;
        hi.a = m; hi.b = pan[worst].b; hi.depth = pan[worst].depth + 1;
        qk15(f, lo.a, lo.b, lo.q);
        qk15(f, hi.a, hi.b, hi.q);
        lo.emax = hi.emax = 0.0;
        for (int k = 0; k < NC; ++k) {
            lo.emax = std::max(lo.emax, lo.q.e[k]);
            hi.emax = std::max(hi.emax, hi.q.e[k]);
        }
        pan[worst] = lo;
        pan.push_back(hi);
        retotal();
    }
    if (!done) hit_cap = !converged();
    if (st) {
        ++st->calls;
        st->panels += static_cast<long long>(pan.size());
        if (hit_cap) ++st->caps;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////

inline Vector vcross(const Vector& a, const Vector& b) {
    return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}
inline double vdot(const Vector& a, const Vector& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

std::vector<double> interior_knots_srf(const NurbsSurface& s, int dir) {
    auto d = s.domain(dir);
    std::vector<double> ks;
    const double eps = std::max(1e-13, std::abs(d.second - d.first) * 1e-12);
    for (int i = 0; i < s.nurbsknot_count(dir); ++i) {
        const double t = s.nurbsknot(dir, i);
        if (t > d.first + eps && t < d.second - eps) ks.push_back(t);
    }
    std::sort(ks.begin(), ks.end());
    ks.erase(std::unique(ks.begin(), ks.end(),
                         [&](double x, double y) { return std::abs(x - y) <= eps; }),
             ks.end());
    return ks;
}

/// Break list for one trim: the pcurve knot spans, merged uniformly down to `cap` when the
/// pcurve carries more (a boolean-produced degree-1 pcurve can have thousands of spans).
std::vector<double> trim_breaks(const NurbsCurve& pc, int cap) {
    auto d = pc.domain();
    const double eps = std::max(1e-14, std::abs(d.second - d.first) * 1e-12);
    std::vector<double> ks;
    for (int i = 0; i < pc.nurbsknot_count(); ++i) {
        const double t = pc.nurbsknot(i);
        if (t > d.first + eps && t < d.second - eps) ks.push_back(t);
    }
    std::sort(ks.begin(), ks.end());
    ks.erase(std::unique(ks.begin(), ks.end(),
                         [&](double x, double y) { return std::abs(x - y) <= eps; }),
             ks.end());
    std::vector<double> br;
    if (static_cast<int>(ks.size()) + 1 <= cap) {
        br.push_back(d.first);
        for (double t : ks) br.push_back(t);
        br.push_back(d.second);
    } else {
        // FOLLOW THE CURVE'S OWN KNOTS. The old branch replaced the knot list with `cap`
        // UNIFORM parameters, so a finely represented boundary was integrated as a decimated
        // COPY of itself: every panel then straddled interior knots, where the integrand is
        // only C^(p-1), and Gauss-Kronrod's smoothness assumption -- and its error estimate --
        // no longer hold. (Raising `cap` is not the fix and was measured: 4096 improves
        // tor x tor common 3.66e-05 -> 1.60e-05 but degrades its cut 6.30e-06 -> 2.71e-03,
        // because at 512 part of the cut's error was cancellation.) Keeping every `stride`-th
        // REAL knot instead makes every panel boundary a genuine break of the integrand, so
        // each panel is smooth and the adaptive refinement inside it is well posed.
        const size_t nk = ks.size();
        const size_t stride = (nk + static_cast<size_t>(cap) - 1) / static_cast<size_t>(cap);
        br.reserve(static_cast<size_t>(cap) + 2);
        br.push_back(d.first);
        for (size_t i = stride - 1; i < nk; i += stride) br.push_back(ks[i]);
        if (br.back() < d.second - eps) br.push_back(d.second);
        else br.back() = d.second;
    }
    return br;
}

struct TrimRef {
    const NurbsCurve* pc = nullptr;
    const NurbsCurve* c3 = nullptr;  ///< the edge's 3D curve, when it matches the pcurve image
    bool c3_fwd = true;              ///< walk the 3D curve with increasing parameter
    int trim_index = -1;
    bool fwd = true;                 ///< traverse the pcurve with increasing parameter
};

/// Resolve the traversal ORDER and DIRECTION of every trim in a loop by chaining head-to-tail
/// in UV. Convention-free: it does not care whether the stored pcurve is already in loop
/// order, whether `BRepTrim::reversed` is meant to flip it, or whether `loop.trim_indices` is
/// stored in walk order. Four candidate chains are built -- {stored order, greedy
/// nearest-endpoint reorder} x {both seed directions} -- and the one with the smallest worst
/// gap wins; ties keep the stored orientation, the only evidence a single closed pcurve has.
/// Returns that worst gap: Green's theorem needs a CLOSED boundary, so a large value is a
/// certificate that the input's trim data is not a loop.
double resolve_loop_chain(std::vector<TrimRef>& trs, double close_tol) {
    const size_t n = trs.size();
    if (n == 0) return 0.0;
    std::vector<Point> ps(n, Point(0, 0, 0)), pe(n, Point(0, 0, 0));
    for (size_t i = 0; i < n; ++i) {
        auto d = trs[i].pc->domain();
        ps[i] = trs[i].pc->point_at(d.first);
        pe[i] = trs[i].pc->point_at(d.second);
    }
    auto dist = [](const Point& a, const Point& b) {
        const double dx = a[0] - b[0], dy = a[1] - b[1];
        return std::sqrt(dx * dx + dy * dy);
    };
    double best_gap = 1e300, stored_gap = 1e300;
    std::vector<size_t> best_ord;
    std::vector<char> best_fwd;
    for (int mode = 0; mode < 2; ++mode) {        // 0 = stored order, 1 = greedy reorder
        for (int seed = 0; seed < 2; ++seed) {
            std::vector<char> used(n, 0), fw(n, 1);
            std::vector<size_t> ord;
            ord.reserve(n);
            const bool s0 = (seed == 0) ? trs[0].fwd : !trs[0].fwd;
            ord.push_back(0);
            used[0] = 1;
            fw[0] = static_cast<char>(s0 ? 1 : 0);
            Point tail = s0 ? pe[0] : ps[0];
            double gap = 0.0;
            for (size_t k = 1; k < n; ++k) {
                size_t best = n;
                bool bf = true;
                double bd = 1e300;
                if (mode == 0) {
                    best = k;
                    const double ds = dist(ps[k], tail), de = dist(pe[k], tail);
                    bf = ds <= de;
                    bd = std::min(ds, de);
                } else {
                    for (size_t j = 0; j < n; ++j) {
                        if (used[j]) continue;
                        const double ds = dist(ps[j], tail), de = dist(pe[j], tail);
                        if (ds < bd) { bd = ds; best = j; bf = true; }
                        if (de < bd) { bd = de; best = j; bf = false; }
                    }
                }
                if (best >= n) break;
                used[best] = 1;
                fw[best] = static_cast<char>(bf ? 1 : 0);
                ord.push_back(best);
                gap = std::max(gap, bd);
                tail = bf ? pe[best] : ps[best];
            }
            gap = std::max(gap, dist(tail, fw[ord[0]] ? ps[ord[0]] : pe[ord[0]]));
            // REORDERING IS A LAST RESORT. It is only accepted when it actually CLOSES the
            // chain: on data whose loop cannot close either way (a seam-collapsing importer)
            // a "smaller but still huge" gap is not evidence of a better region, and the
            // stored order is the only remaining information about the intended walk.
            const bool accept = (mode == 0) ? (gap < best_gap * (1.0 - 1e-12))
                                            : (stored_gap > close_tol && gap <= close_tol &&
                                               gap < best_gap * (1.0 - 1e-12));
            if (mode == 0) stored_gap = std::min(stored_gap, gap);
            if (accept) { best_gap = gap; best_ord = ord; best_fwd = fw; }
        }
    }
    if (best_ord.size() != n) return best_gap;   // degenerate: leave the loop as stored
    std::vector<TrimRef> out;
    out.reserve(n);
    for (size_t i : best_ord) {
        TrimRef t = trs[i];
        t.fwd = best_fwd[i] != 0;
        out.push_back(t);
    }
    trs.swap(out);
    return best_gap;
}

/// Signed UV area enclosed by a resolved loop chain, from sampled pcurve points (shoelace).
/// Used only to detect a DEGENERATE trim region -- one that encloses no parameter area, which
/// is what a seam-collapsing importer produces for a fully periodic face.
double loop_uv_area(const std::vector<TrimRef>& trs, int per_trim = 24) {
    double a2 = 0.0;
    bool first = true;
    Point prev(0, 0, 0), start(0, 0, 0);
    for (const TrimRef& tr : trs) {
        auto d = tr.pc->domain();
        for (int i = 0; i <= per_trim; ++i) {
            const double f = static_cast<double>(i) / per_trim;
            const double t = tr.fwd ? d.first + (d.second - d.first) * f
                                    : d.second - (d.second - d.first) * f;
            const Point p = tr.pc->point_at(t);
            if (first) { start = p; first = false; }
            else a2 += prev[0] * p[1] - p[0] * prev[1];
            prev = p;
        }
    }
    if (first) return 0.0;
    a2 += prev[0] * start[1] - start[0] * prev[1];
    return 0.5 * a2;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Green-reduced integration over the TRIMMED UV region of one face.
//
//   II_D f(u,v) du dv = oint_{dD} H(u,v) dv,   H(u,v) = I_{uref}^{u} f(s,v) ds
//
// (identical reduction to OCCT BRepGProp_VinertGK / _TFunction / _UFunction). The value is
// independent of uref: two choices differ by oint of an exact 1-form, which vanishes on a
// closed loop. uref is therefore chosen to make as many trims as possible drop out.
///////////////////////////////////////////////////////////////////////////////////////////

class FaceIntegrator {
public:
    FaceIntegrator(const NurbsSurface& srf, const MassPropsOptions& o, long long& budget)
        : S(srf), opt(o), budget_left(budget) {
        du = S.domain(0);
        dv = S.domain(1);
        uk = interior_knots_srf(S, 0);
    }

    /// Integrands over the UV domain with the NATURAL normal n = Su x Sv:
    ///   [0] |n|                    -> trimmed area
    ///   [1] S . n                  -> 3V             (div r = 3)
    ///   [2+k] (S_k^2 / 2) n_k      -> integral of x_k over the enclosed volume
    ///                                 (div of (x^2/2,0,0) = x, etc.)
    void integrand(double u, double v, double* out) const {
        u = std::min(std::max(u, du.first), du.second);
        v = std::min(std::max(v, dv.first), dv.second);
        --budget_left;
        auto e = S.evaluate(u, v, 1);  // [S, Sv, Su]
        if (e.size() < 3) { for (int k = 0; k < NC; ++k) out[k] = 0.0; return; }
        const Vector n = vcross(e[2], e[1]);
        out[0] = n.magnitude();
        out[1] = vdot(e[0], n);
        out[2] = 0.5 * e[0][0] * e[0][0] * n[0];
        out[3] = 0.5 * e[0][1] * e[0][1] * n[1];
        out[4] = 0.5 * e[0][2] * e[0][2] * n[2];
        out[5] = n[0];
        out[6] = n[1];
        out[7] = n[2];
    }

    void inner_H(double uref, double ut, double v, double* H) const {
        for (int k = 0; k < NC; ++k) H[k] = 0.0;
        const double A = std::min(std::max(uref, du.first), du.second);
        const double B = std::min(std::max(ut, du.first), du.second);
        const double lo = std::min(A, B), hi = std::max(A, B);
        if (!(hi - lo > std::abs(du.second - du.first) * 1e-15)) return;
        std::vector<double> br;
        br.reserve(uk.size() + 2);
        br.push_back(lo);
        for (double t : uk)
            if (t > lo + 1e-14 && t < hi - 1e-14) br.push_back(t);
        br.push_back(hi);
        Quad q;
        bool cap = false;
        auto f = [this, v](double s, double* o) { integrand(s, v, o); };
        adaptive_gk(f, br, inner_rtol, atol_inner, inner_panels, 12, budget_left, q, cap,
                    &inner_stats);
        if (cap) inner_capped = true;
        const double sgn = (B >= A) ? 1.0 : -1.0;
        for (int k = 0; k < NC; ++k) {
            H[k] = sgn * q.r[k];
            inner_err[k] += q.e[k];
        }
    }

    /// Green reduction over the FULL parameter rectangle: only the u = umax edge of the
    /// rectangle survives (v = const edges have dv = 0, the u = uref edge has H = 0), so the
    /// whole face integral is one 1D integral. Used when a face carries no usable trim loops
    /// (an importer that drops the pcurves of a periodic quadric leaves exactly that), and as
    /// the SESSION_MP_DBG cross-check for trimmed faces.
    void full_domain(Quad& acc, bool& capped) {
        acc.zero();
        std::vector<double> br;
        br.push_back(dv.first);
        for (double t : interior_knots_srf(S, 1))
            if (t > dv.first + 1e-14 && t < dv.second - 1e-14) br.push_back(t);
        br.push_back(dv.second);
        auto f = [&](double v, double* o) { inner_H(du.first, du.second, v, o); };
        bool cap = false;
        adaptive_gk(f, br, outer_rtol, atol, outer_panels, opt.max_depth, budget_left, acc, cap,
                    &outer_stats);
        if (cap) capped = true;
    }

    /// Raw Green boundary sum for one loop in its RESOLVED traversal sense.
    void loop_green(const std::vector<TrimRef>& trims, double uref, Quad& acc, bool& capped) {
        acc.zero();
        const double vspan = std::max(1.0, std::abs(dv.second - dv.first));
        const double uspan = std::max(1.0, std::abs(du.second - du.first));
        for (const TrimRef& tr : trims) {
            const NurbsCurve& pc = *tr.pc;
            auto dc = pc.domain();
            if (!(dc.second > dc.first)) continue;
            // Trims that provably cannot contribute: v is constant along the whole trim
            // (isoparametric cap circles, sphere pole lines -> dv == 0), or u sits on uref
            // (H == 0 there). This is what makes an untrimmed quadric face cost one 1D
            // integral instead of a 2D sweep.
            bool dv_zero = true, u_at_ref = true;
            const double v0 = pc.point_at(dc.first)[1];
            for (int i = 0; i <= 16; ++i) {
                Point p = pc.point_at(dc.first + (dc.second - dc.first) * i / 16.0);
                if (std::abs(p[1] - v0) > 1e-12 * vspan) dv_zero = false;
                if (std::abs(p[0] - uref) > 1e-12 * uspan) u_at_ref = false;
                if (!dv_zero && !u_at_ref) break;
            }
            if (dv_zero || u_at_ref) continue;

            const bool fwd = tr.fwd;
            auto f = [&](double t, double* o) {
                const double tt = fwd ? t : (dc.first + dc.second - t);
                auto pe = pc.evaluate(tt, 1);
                if (pe.size() < 2) { for (int k = 0; k < NC; ++k) o[k] = 0.0; return; }
                double dvdt = pe[1][1];
                if (!fwd) dvdt = -dvdt;
                if (dvdt == 0.0) { for (int k = 0; k < NC; ++k) o[k] = 0.0; return; }
                double H[NC];
                inner_H(uref, pe[0][0], pe[0][1], H);
                for (int k = 0; k < NC; ++k) o[k] = H[k] * dvdt;
            };
            std::vector<double> br = trim_breaks(pc, opt.max_init_intervals);
            Quad q;
            bool cap = false;
            adaptive_gk(f, br, outer_rtol, atol, outer_panels, opt.max_depth, budget_left, q, cap,
                        &outer_stats);
            if (cap) capped = true;
            for (int k = 0; k < NC; ++k) {
                acc.r[k] += q.r[k];
                acc.e[k] += q.e[k];
                acc.a[k] += q.a[k];
            }
            if (budget_left <= 0) { capped = true; break; }
        }
    }

    const NurbsSurface& S;
    const MassPropsOptions& opt;
    long long& budget_left;
    std::pair<double, double> du{0, 1}, dv{0, 1};
    std::vector<double> uk;
    mutable double inner_err[NC] = {0, 0, 0, 0, 0};
    mutable bool inner_capped = false;
    mutable GKStats inner_stats, outer_stats;
    double inner_rtol = 1e-13;
    double outer_rtol = 1e-11;
    double atol[NC] = {0, 0, 0, 0, 0};        ///< absolute floors, set from the face scale
    double atol_inner[NC] = {0, 0, 0, 0, 0};  ///< same, 100x tighter for the inner integral
    int inner_panels = 32;
    int outer_panels = 512;
};

/// Geometric scale of a face's surface: `span` = bbox diagonal of a sampled grid, `far` =
/// largest distance from the world origin (the first moments carry x^2, so they scale as
/// far^2 * span^2). Turned into the per-component absolute floors of the quadrature.
void face_scale_atol(const NurbsSurface& s, double rtol, double* atol) {
    auto d0 = s.domain(0), d1 = s.domain(1);
    double mn[3] = {1e300, 1e300, 1e300}, mx[3] = {-1e300, -1e300, -1e300}, far = 0.0;
    for (int i = 0; i <= 4; ++i)
        for (int j = 0; j <= 4; ++j) {
            const Point p = s.point_at(d0.first + (d0.second - d0.first) * i / 4.0,
                                       d1.first + (d1.second - d1.first) * j / 4.0);
            for (int k = 0; k < 3; ++k) {
                mn[k] = std::min(mn[k], p[k]);
                mx[k] = std::max(mx[k], p[k]);
            }
            far = std::max(far, std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]));
        }
    double span = 0.0;
    for (int k = 0; k < 3; ++k) span += (mx[k] - mn[k]) * (mx[k] - mn[k]);
    span = std::sqrt(span);
    if (!(span > 0.0)) span = 1.0;
    if (!(far > 0.0)) far = span;
    const double a2 = span * span;
    atol[0] = rtol * a2;
    atol[1] = rtol * far * a2;
    atol[2] = atol[3] = atol[4] = rtol * far * far * a2;
    atol[5] = atol[6] = atol[7] = rtol * a2;
}

/// Geometric planarity: every sampled point lies on the plane through the centre point with
/// the centre normal, to a relative tolerance.
bool surface_is_planar(const NurbsSurface& s, Point& P0, Vector& N) {
    auto d0 = s.domain(0), d1 = s.domain(1);
    const double uc = 0.5 * (d0.first + d0.second), vc = 0.5 * (d1.first + d1.second);
    P0 = s.point_at(uc, vc);
    N = s.normal_at(uc, vc);
    if (N.magnitude() < 1e-12) {
        N = s.normal_at(d0.first + 0.3 * (d0.second - d0.first),
                        d1.first + 0.3 * (d1.second - d1.first));
        if (N.magnitude() < 1e-12) return false;
    }
    const double nl = N.magnitude();
    N = Vector(N[0] / nl, N[1] / nl, N[2] / nl);
    double scale = 0.0;
    const Point cs[4] = {s.point_at(d0.first, d1.first), s.point_at(d0.second, d1.first),
                         s.point_at(d0.first, d1.second), s.point_at(d0.second, d1.second)};
    for (const Point& c : cs) scale = std::max(scale, c.distance(P0));
    if (scale < 1e-14) return false;
    const int NG = 5;
    for (int i = 0; i <= NG; ++i)
        for (int j = 0; j <= NG; ++j) {
            const Point p = s.point_at(d0.first + (d0.second - d0.first) * i / NG,
                                       d1.first + (d1.second - d1.first) * j / NG);
            const Vector w(p[0] - P0[0], p[1] - P0[1], p[2] - P0[2]);
            if (std::abs(vdot(w, N)) > scale * 1e-11 + 1e-13) return false;
        }
    return true;
}

}  // namespace

///////////////////////////////////////////////////////////////////////////////////////////

MassProps brep_massprops(const BRep& brep, const MassPropsOptions& opt) {
    using clk = std::chrono::steady_clock;
    const auto t_start = clk::now();

    MassProps out;
    const int nf = static_cast<int>(brep.m_faces.size());
    out.faces.assign(static_cast<size_t>(std::max(nf, 0)), FaceMassProps());

    long long budget = opt.max_surface_evals;
    const long long face_share =
        std::max(opt.min_face_evals, nf > 0 ? opt.max_surface_evals / nf : opt.max_surface_evals);

    for (const BRepEdge& e : brep.m_topology_edges)
        if (e.trim_indices.size() != 2) ++out.naked_edges;
    out.closed = (out.naked_edges == 0);

    // Resolved traversal direction of every trim (filled in PASS 1, reused in PASS 2).
    std::vector<char> trim_fwd(brep.m_trims.size(), 1);
    // +1/-1: the loop's material-left walk runs with/against the edge's 3D curve; 0: unknown.
    std::vector<signed char> trim_c3(brep.m_trims.size(), 0);
    // Faces integrated over the whole parameter rectangle: their stored loops are degenerate,
    // so the material-left walk is the CCW walk of the DOMAIN rectangle, not of those loops.
    std::vector<char> face_fulldom(static_cast<size_t>(std::max(nf, 0)), 0);

    ///////////////////////////////////////////////////////////////////////////////////////
    // PASS 1 -- per-face trimmed integrals, NATURAL normal.
    ///////////////////////////////////////////////////////////////////////////////////////
    for (int fi = 0; fi < nf; ++fi) {
        FaceMassProps& F = out.faces[static_cast<size_t>(fi)];
        F.face_index = fi;
        const BRepFace& face = brep.m_faces[static_cast<size_t>(fi)];
        if (face.surface_index < 0 || face.surface_index >= static_cast<int>(brep.m_surfaces.size()))
            continue;
        const NurbsSurface& srf = brep.m_surfaces[static_cast<size_t>(face.surface_index)];
        if (!srf.is_valid()) continue;
        auto d0 = srf.domain(0), d1 = srf.domain(1);

        std::vector<std::vector<TrimRef>> loops;
        std::vector<std::array<double, 4>> lbb;
        double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= static_cast<int>(brep.m_loops.size())) continue;
            std::vector<TrimRef> trs;
            std::array<double, 4> bb = {1e300, -1e300, 1e300, -1e300};
            for (int ti : brep.m_loops[static_cast<size_t>(li)].trim_indices) {
                if (ti < 0 || ti >= static_cast<int>(brep.m_trims.size())) continue;
                const BRepTrim& tm = brep.m_trims[static_cast<size_t>(ti)];
                if (tm.curve_2d_index < 0 ||
                    tm.curve_2d_index >= static_cast<int>(brep.m_curves_2d.size()))
                    continue;
                const NurbsCurve& pc = brep.m_curves_2d[static_cast<size_t>(tm.curve_2d_index)];
                if (!pc.is_valid()) continue;
                auto dc = pc.domain();
                for (int i = 0; i <= 24; ++i) {
                    const Point p = pc.point_at(dc.first + (dc.second - dc.first) * i / 24.0);
                    bb[0] = std::min(bb[0], p[0]); bb[1] = std::max(bb[1], p[0]);
                    bb[2] = std::min(bb[2], p[1]); bb[3] = std::max(bb[3], p[1]);
                }
                TrimRef tr;
                tr.pc = &pc;
                tr.trim_index = ti;
                tr.fwd = !tm.reversed;
                // The edge's exact 3D curve, when its image agrees with the pcurve pulled onto
                // the surface. A planar face's mass properties depend ONLY on its 3D boundary,
                // so using the exact edge curve there sidesteps a polyline-approximated pcurve
                // entirely (our STEP reader emits 48-segment degree-1 pcurves for exact arcs).
                if (tm.edge_index >= 0 &&
                    tm.edge_index < static_cast<int>(brep.m_topology_edges.size())) {
                    const int ci =
                        brep.m_topology_edges[static_cast<size_t>(tm.edge_index)].curve_3d_index;
                    if (ci >= 0 && ci < static_cast<int>(brep.m_curves_3d.size())) {
                        const NurbsCurve& c3 = brep.m_curves_3d[static_cast<size_t>(ci)];
                        if (c3.is_valid()) {
                            auto d3 = c3.domain();
                            auto pat = [&](double f) {
                                const Point q = pc.point_at(dc.first + (dc.second - dc.first) * f);
                                return srf.point_at(q[0], q[1]);
                            };
                            auto cat = [&](double f) {
                                return c3.point_at(d3.first + (d3.second - d3.first) * f);
                            };
                            const Point s0 = pat(0.0), s1 = pat(1.0), sm = pat(0.5),
                                        sq = pat(0.25);
                            const Point a0 = cat(0.0), a1 = cat(1.0), am = cat(0.5),
                                        aq = cat(0.25), a7 = cat(0.75);
                            const double sc =
                                std::max(1e-12, s0.distance(sm) + sm.distance(s1));
                            const double tol = 1e-7 * std::max(1.0, sc);
                            const bool closed = a0.distance(a1) < tol;
                            if (sm.distance(am) < tol) {
                                if (closed) {
                                    // endpoints carry no direction on a closed edge -- the
                                    // quarter point does.
                                    if (s0.distance(a0) < tol) {
                                        tr.c3 = &c3;
                                        tr.c3_fwd = sq.distance(aq) <= sq.distance(a7);
                                    }
                                } else if (s0.distance(a0) < tol && s1.distance(a1) < tol) {
                                    tr.c3 = &c3;
                                    tr.c3_fwd = true;
                                } else if (s0.distance(a1) < tol && s1.distance(a0) < tol) {
                                    tr.c3 = &c3;
                                    tr.c3_fwd = false;
                                }
                            }
                        }
                    }
                }
                trs.push_back(tr);
            }
            if (trs.empty()) continue;
            {
                const double su = std::max(1e-12, std::abs(d0.second - d0.first));
                const double sv = std::max(1e-12, std::abs(d1.second - d1.first));
                const double dscale = std::sqrt(su * su + sv * sv);
                const double g = resolve_loop_chain(trs, 1e-6 * dscale);
                F.chain_gap = std::max(F.chain_gap, g / dscale);
            }
            for (const TrimRef& tr : trs) {
                trim_fwd[static_cast<size_t>(tr.trim_index)] = static_cast<char>(tr.fwd ? 1 : 0);
                trim_c3[static_cast<size_t>(tr.trim_index)] =
                    tr.c3 ? static_cast<signed char>((tr.fwd == tr.c3_fwd) ? 1 : -1) : 0;
            }
            umin = std::min(umin, bb[0]); umax = std::max(umax, bb[1]);
            vmin = std::min(vmin, bb[2]); vmax = std::max(vmax, bb[3]);
            loops.push_back(std::move(trs));
            lbb.push_back(bb);
        }
        // DEGENERATE TRIM REGION. A face whose loops enclose (essentially) no parameter area
        // cannot be integrated over "its trim": there is none. Our STEP reader produces this
        // for a fully periodic quadric face, whose two seam copies it maps to the SAME
        // parametric u (0 and 0 instead of 0 and 2*pi), and for the sphere it emits no loops
        // at all. The only defensible reading of such a face is the whole parameter patch,
        // which is exactly what a periodic face without real trims is. Detected on the cheap
        // sampled UV polygon, so a real face never pays for it.
        if (!loops.empty()) {
            double uva = 0.0;
            for (const auto& L : loops) uva += loop_uv_area(L);
            const double dom_a = std::abs((d0.second - d0.first) * (d1.second - d1.first));
            if (std::abs(uva) <= 1e-9 * dom_a) loops.clear();
        }
        if (loops.empty()) {
            // No usable trim loops -> the face IS the whole parameter rectangle.
            long long fb = std::min<long long>(face_share, std::max<long long>(budget, 0));
            const long long fb0 = fb;
            if (fb <= 0) { out.converged = false; F.budget_hit = true; continue; }
            FaceIntegrator fint(srf, opt, fb);
            fint.inner_rtol = std::max(opt.rel_tolerance * 0.01, 1e-13);
            fint.outer_rtol = std::max(opt.rel_tolerance, 1e-11);
            fint.outer_panels = std::max(64, opt.max_init_intervals);
            face_scale_atol(srf, fint.outer_rtol, fint.atol);
            face_scale_atol(srf, fint.inner_rtol, fint.atol_inner);
            Quad q;
            bool cap = false;
            fint.full_domain(q, cap);
            F.path = MassPropsPath::FullDomain;
            face_fulldom[static_cast<size_t>(fi)] = 1;
            for (int li : face.loop_indices) {
                if (li < 0 || li >= static_cast<int>(brep.m_loops.size())) continue;
                for (int ti : brep.m_loops[static_cast<size_t>(li)].trim_indices)
                    if (ti >= 0 && ti < static_cast<int>(brep.m_trims.size()))
                        trim_c3[static_cast<size_t>(ti)] = 0;
            }
            F.traversal = (q.r[0] >= 0.0) ? 1.0 : -1.0;
            F.area = std::abs(q.r[0]);
            F.area_err = q.e[0];
            F.flux = F.traversal * q.r[1];
            F.flux_err = q.e[1];
            for (int k = 0; k < 3; ++k) {
                F.mom[k] = F.traversal * q.r[2 + k];
                F.vec_area[k] = F.traversal * q.r[5 + k];
            }
            F.evals = fb0 - fb;
            F.budget_hit = cap;
            budget -= F.evals;
            if (cap) out.converged = false;
            if (std::getenv("SESSION_MP_DBG"))
                std::fprintf(stderr,
                             "[MP] FULL f=%d area=%.10f flux=%.10f evals=%lld cap=%d | inner "
                             "calls=%lld panels=%lld caps=%lld worst=%.3e(k%d) | outer calls=%lld "
                             "panels=%lld caps=%lld worst=%.3e(k%d)\n",
                             fi, F.area, F.flux, F.evals, cap ? 1 : 0, fint.inner_stats.calls,
                             fint.inner_stats.panels, fint.inner_stats.caps,
                             fint.inner_stats.worst_ratio, fint.inner_stats.worst_k,
                             fint.outer_stats.calls, fint.outer_stats.panels,
                             fint.outer_stats.caps, fint.outer_stats.worst_ratio,
                             fint.outer_stats.worst_k);
            continue;
        }
        if (!(umax > umin)) { umin = d0.first; umax = d0.second; }
        if (!(vmax > vmin)) { vmin = d1.first; vmax = d1.second; }
        const double uref = std::min(std::max(umin, d0.first), d0.second);

        long long face_budget = std::min<long long>(face_share, std::max<long long>(budget, 0));
        if (face_budget <= 0) { out.converged = false; F.budget_hit = true; continue; }
        const long long face_budget0 = face_budget;

        Point P0(0, 0, 0);
        Vector Npl(0, 0, 0);
        const bool planar = opt.use_analytic && surface_is_planar(srf, P0, Npl);

        std::vector<Quad> lq(loops.size());
        bool capped = false;

        if (planar) {
            F.path = MassPropsPath::PlanarExact;
            // A planar face collapses to 1D integrals over its 3D boundary:
            //   vector area   A = 1/2 oint S x dS              (exact for any patch)
            //   flux          = (P0.N) * (A.N)                 (S.N is the constant plane offset)
            //   moment_k      = N_k * oint G_k db              (Green in the plane frame (e1,e2,N))
            // with G_k(a,b) = I_0^a 1/2 (P0_k + s e1_k + b e2_k)^2 ds, a=(S-P0).e1, b=(S-P0).e2.
            Vector e1 = (std::abs(Npl[0]) > 0.9) ? Vector(0, 1, 0) : Vector(1, 0, 0);
            Vector t = vcross(Npl, e1);
            const double tl = t.magnitude();
            e1 = Vector(t[0] / tl, t[1] / tl, t[2] / tl);
            const Vector e2 = vcross(Npl, e1);
            const double d_off = vdot(Vector(P0[0], P0[1], P0[2]), Npl);
            double atol_pl[NC];
            face_scale_atol(srf, std::max(opt.rel_tolerance, 1e-14), atol_pl);
            // the planar integrands are the boundary forms, one power of length lower
            for (int k = 0; k < NC; ++k) atol_pl[k] *= 1e-2;

            for (size_t L = 0; L < loops.size(); ++L) {
                Quad acc;
                acc.zero();
                for (const TrimRef& tr : loops[L]) {
                    // Prefer the exact 3D edge curve; fall back to pcurve-on-surface.
                    const bool use3 = (tr.c3 != nullptr);
                    const NurbsCurve& pc = use3 ? *tr.c3 : *tr.pc;
                    auto dc = pc.domain();
                    if (!(dc.second > dc.first)) continue;
                    const bool fwd = use3 ? (tr.fwd == tr.c3_fwd) : tr.fwd;
                    auto acc5 = [&](const Vector& Sp, const Vector& dS, double* o) {
                        const Vector cr = vcross(Sp, dS);
                        o[0] = 0.5 * vdot(cr, Npl);
                        o[1] = 0.0;
                        o[5] = 0.5 * cr[0];
                        o[6] = 0.5 * cr[1];
                        o[7] = 0.5 * cr[2];
                        const Vector w(Sp[0] - P0[0], Sp[1] - P0[1], Sp[2] - P0[2]);
                        const double aa = vdot(w, e1), bb2 = vdot(w, e2), dbdt = vdot(dS, e2);
                        for (int k = 0; k < 3; ++k) {
                            const double c0 = P0[k] + bb2 * e2[k], a1 = e1[k];
                            o[2 + k] = 0.5 *
                                       (c0 * c0 * aa + c0 * a1 * aa * aa +
                                        a1 * a1 * aa * aa * aa / 3.0) *
                                       dbdt;
                        }
                    };
                    auto f = [&](double t0, double* o) {
                        const double tt = fwd ? t0 : (dc.first + dc.second - t0);
                        auto pe = pc.evaluate(tt, 1);
                        if (pe.size() < 2) { for (int k = 0; k < NC; ++k) o[k] = 0.0; return; }
                        --face_budget;
                        if (use3) {
                            Vector dS = pe[1];
                            if (!fwd) dS = Vector(-dS[0], -dS[1], -dS[2]);
                            acc5(pe[0], dS, o);
                            return;
                        }
                        const double u = std::min(std::max(pe[0][0], d0.first), d0.second);
                        const double v = std::min(std::max(pe[0][1], d1.first), d1.second);
                        double dudt = pe[1][0], dvdt = pe[1][1];
                        if (!fwd) { dudt = -dudt; dvdt = -dvdt; }
                        auto se = srf.evaluate(u, v, 1);
                        if (se.size() < 3) { for (int k = 0; k < NC; ++k) o[k] = 0.0; return; }
                        acc5(se[0], Vector(se[2][0] * dudt + se[1][0] * dvdt,
                                           se[2][1] * dudt + se[1][1] * dvdt,
                                           se[2][2] * dudt + se[1][2] * dvdt),
                             o);
                    };
                    std::vector<double> br = trim_breaks(pc, opt.max_init_intervals);
                    Quad q;
                    bool cap = false;
                    adaptive_gk(f, br, std::max(opt.rel_tolerance, 1e-14), atol_pl,
                                std::max(64, opt.max_init_intervals * 4), opt.max_depth,
                                face_budget, q, cap);
                    if (cap) capped = true;
                    for (int k = 0; k < NC; ++k) {
                        acc.r[k] += q.r[k]; acc.e[k] += q.e[k]; acc.a[k] += q.a[k];
                    }
                    if (face_budget <= 0) { capped = true; break; }
                }
                acc.r[1] = d_off * acc.r[0];
                acc.e[1] = std::abs(d_off) * acc.e[0];
                acc.a[1] = std::abs(d_off) * acc.a[0];
                for (int k = 0; k < 3; ++k) {
                    acc.r[2 + k] *= Npl[k];
                    acc.e[2 + k] *= std::abs(Npl[k]);
                    acc.a[2 + k] *= std::abs(Npl[k]);
                }
                lq[L] = acc;
            }
        } else {
            F.path = MassPropsPath::GreenUV;
            FaceIntegrator fint(srf, opt, face_budget);
            fint.inner_rtol = std::max(opt.rel_tolerance * 0.01, 1e-13);
            fint.outer_rtol = std::max(opt.rel_tolerance, 1e-11);
            fint.outer_panels = std::max(64, opt.max_init_intervals);
            face_scale_atol(srf, fint.outer_rtol, fint.atol);
            face_scale_atol(srf, fint.inner_rtol, fint.atol_inner);
            for (size_t L = 0; L < loops.size(); ++L)
                fint.loop_green(loops[L], uref, lq[L], capped);
            if (fint.inner_capped) capped = true;
            if (std::getenv("SESSION_MP_DBG"))
                std::fprintf(stderr,
                             "[MP] GK f=%d inner calls=%lld panels=%lld caps=%lld worst=%.3e(k%d)"
                             " | outer calls=%lld panels=%lld caps=%lld worst=%.3e(k%d)\n",
                             fi, fint.inner_stats.calls, fint.inner_stats.panels,
                             fint.inner_stats.caps, fint.inner_stats.worst_ratio,
                             fint.inner_stats.worst_k, fint.outer_stats.calls,
                             fint.outer_stats.panels, fint.outer_stats.caps,
                             fint.outer_stats.worst_ratio, fint.outer_stats.worst_k);
        }

        // Hole loops must run opposite to the dominant (outer) loop. Self-heal a loop whose
        // stored sense agrees with the dominant one while its UV box sits inside it.
        size_t dom = 0;
        for (size_t L = 1; L < loops.size(); ++L)
            if (std::abs(lq[L].r[0]) > std::abs(lq[dom].r[0])) dom = L;
        for (size_t L = 0; L < loops.size(); ++L) {
            if (L == dom) continue;
            const bool same = (lq[L].r[0] * lq[dom].r[0]) > 0.0;
            const auto& b = lbb[L];
            const auto& B = lbb[dom];
            const double tu = 1e-9 * std::max(1.0, B[1] - B[0]);
            const double tv = 1e-9 * std::max(1.0, B[3] - B[2]);
            const bool inside = b[0] >= B[0] - tu && b[1] <= B[1] + tu && b[2] >= B[2] - tv &&
                                b[3] <= B[3] + tv;
            if (same && inside) {
                for (int k = 0; k < NC; ++k) lq[L].r[k] = -lq[L].r[k];
                // The heal reverses this loop's walk, so the recorded per-trim walk senses
                // (consumed by the orientation propagation in PASS 2) must reverse with it.
                // A single closed pcurve -- the usual hole loop -- carries no chain evidence
                // of its own direction, so this is the ONLY place its sense gets fixed.
                for (const TrimRef& tr : loops[L]) {
                    const size_t ti2 = static_cast<size_t>(tr.trim_index);
                    trim_c3[ti2] = static_cast<signed char>(-trim_c3[ti2]);
                    trim_fwd[ti2] = static_cast<char>(trim_fwd[ti2] ? 0 : 1);
                }
            }
        }

        Quad tot;
        tot.zero();
        for (const Quad& q : lq)
            for (int k = 0; k < NC; ++k) {
                tot.r[k] += q.r[k]; tot.e[k] += q.e[k]; tot.a[k] += q.a[k];
            }

        // The true area is positive by definition, so the SIGN of the raw area integral is
        // exactly the stored traversal sense. No `reversed` flag, no UV polygon needed.
        F.traversal = (tot.r[0] >= 0.0) ? 1.0 : -1.0;
        F.area = std::abs(tot.r[0]);
        F.area_err = tot.e[0];
        F.flux = F.traversal * tot.r[1];
        F.flux_err = tot.e[1];
        for (int k = 0; k < 3; ++k) {
            F.mom[k] = F.traversal * tot.r[2 + k];
            F.vec_area[k] = F.traversal * tot.r[5 + k];
        }
        F.evals = face_budget0 - face_budget;
        F.budget_hit = capped;
        if (std::getenv("SESSION_MP_DBG")) {
            long long fb = 5000000;
            FaceIntegrator fd(srf, opt, fb);
            face_scale_atol(srf, fd.outer_rtol, fd.atol);
            face_scale_atol(srf, fd.inner_rtol, fd.atol_inner);
            Quad qf;
            bool cf = false;
            fd.full_domain(qf, cf);
            std::fprintf(stderr, "[MP] FULLDOMAIN f=%d area=%.10f flux=%.10f\n", fi, qf.r[0],
                         qf.r[1]);
            std::fprintf(stderr,
                         "[MP] f=%d srf=%d path=%d loops=%zu deg=%dx%d rat=%d "
                         "dom=[%.6f,%.6f]x[%.6f,%.6f] uv=[%.6f,%.6f]x[%.6f,%.6f] uref=%.6f "
                         "area=%.10f flux=%.10f trav=%+.0f evals=%lld cap=%d\n",
                         fi, face.surface_index, (int)F.path, loops.size(), srf.degree(0),
                         srf.degree(1), srf.is_rational() ? 1 : 0, d0.first, d0.second, d1.first,
                         d1.second, umin, umax, vmin, vmax, uref, F.area, F.flux, F.traversal,
                         F.evals, capped ? 1 : 0);
            for (size_t L = 0; L < loops.size(); ++L) {
                std::fprintf(stderr, "[MP]   loop %zu raw_area=%.10f raw_flux=%.10f trims=%zu\n", L,
                             lq[L].r[0], lq[L].r[1], loops[L].size());
                for (const TrimRef& tr : loops[L]) {
                    auto dc = tr.pc->domain();
                    Point a = tr.pc->point_at(dc.first), b = tr.pc->point_at(dc.second),
                          mm = tr.pc->point_at(0.5 * (dc.first + dc.second));
                    std::fprintf(stderr,
                                 "[MP]     trim %d fwd=%d deg=%d cv=%d c3=%d(d%d cv%d r%d) uv "
                                 "%.6f,%.6f -> %.6f,%.6f mid %.6f,%.6f\n",
                                 tr.trim_index, tr.fwd ? 1 : 0, tr.pc->degree(), tr.pc->cv_count(),
                                 tr.c3 ? (tr.c3_fwd ? 1 : -1) : 0,
                                 tr.c3 ? tr.c3->degree() : -1, tr.c3 ? tr.c3->cv_count() : -1,
                                 tr.c3 ? (tr.c3->is_rational() ? 1 : 0) : -1,
                                 a[0], a[1], b[0], b[1], mm[0], mm[1]);
                }
            }
        }
        budget -= F.evals;
        if (capped || budget <= 0) out.converged = false;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // PASS 2 -- relative face orientation propagated across every 2-trim edge.
    // Two faces sharing an edge are consistently oriented iff their material-left walks
    // traverse that edge in OPPOSITE 3D directions.
    ///////////////////////////////////////////////////////////////////////////////////////
    auto face_of_trim = [&](int ti) -> int {
        if (ti < 0 || ti >= static_cast<int>(brep.m_trims.size())) return -1;
        const int li = brep.m_trims[static_cast<size_t>(ti)].loop_index;
        if (li < 0 || li >= static_cast<int>(brep.m_loops.size())) return -1;
        return brep.m_loops[static_cast<size_t>(li)].face_index;
    };
    auto trim_dir = [&](int ti) -> Vector {
        const int fi = face_of_trim(ti);
        if (fi < 0 || fi >= nf) return Vector(0, 0, 0);
        const BRepTrim& tm = brep.m_trims[static_cast<size_t>(ti)];
        const BRepFace& face = brep.m_faces[static_cast<size_t>(fi)];
        if (face.surface_index < 0 || face.surface_index >= static_cast<int>(brep.m_surfaces.size()))
            return Vector(0, 0, 0);
        if (tm.curve_2d_index < 0 || tm.curve_2d_index >= static_cast<int>(brep.m_curves_2d.size()))
            return Vector(0, 0, 0);
        const NurbsSurface& srf = brep.m_surfaces[static_cast<size_t>(face.surface_index)];
        const NurbsCurve& pc = brep.m_curves_2d[static_cast<size_t>(tm.curve_2d_index)];
        if (!pc.is_valid() || !srf.is_valid()) return Vector(0, 0, 0);
        auto dc = pc.domain();
        const double s = out.faces[static_cast<size_t>(fi)].traversal;
        if (face_fulldom[static_cast<size_t>(fi)]) {
            // Material-left = CCW around the domain rectangle: +u on v=vmin, +v on u=umax,
            // -u on v=vmax, -v on u=umin. The stored loop is degenerate and says nothing.
            auto du = srf.domain(0), dvd = srf.domain(1);
            const Point m2 = pc.point_at(0.5 * (dc.first + dc.second));
            const double tu = 1e-6 * std::max(1.0, du.second - du.first);
            const double tv = 1e-6 * std::max(1.0, dvd.second - dvd.first);
            double gu = 0.0, gv = 0.0;
            if (std::abs(m2[1] - dvd.first) < tv) gu = 1.0;
            else if (std::abs(m2[1] - dvd.second) < tv) gu = -1.0;
            else if (std::abs(m2[0] - du.first) < tu) gv = -1.0;
            else if (std::abs(m2[0] - du.second) < tu) gv = 1.0;
            else return Vector(0, 0, 0);
            auto se = srf.evaluate(std::min(std::max(m2[0], du.first), du.second),
                                   std::min(std::max(m2[1], dvd.first), dvd.second), 1);
            if (se.size() < 3) return Vector(0, 0, 0);
            return Vector(s * (se[2][0] * gu + se[1][0] * gv), s * (se[2][1] * gu + se[1][1] * gv),
                          s * (se[2][2] * gu + se[1][2] * gv));
        }
        const bool fwd = trim_fwd[static_cast<size_t>(ti)] != 0;
        const double t25 = dc.first + 0.25 * (dc.second - dc.first);
        const double t75 = dc.first + 0.75 * (dc.second - dc.first);
        const Point a2 = pc.point_at(fwd ? t25 : t75);
        const Point b2 = pc.point_at(fwd ? t75 : t25);
        const Point A = srf.point_at(a2[0], a2[1]);
        const Point B = srf.point_at(b2[0], b2[1]);
        return Vector(s * (B[0] - A[0]), s * (B[1] - A[1]), s * (B[2] - A[2]));
    };

    // Direction of a trim's material-left walk AT A GIVEN 3D POINT. The plain trim_dir above
    // samples each pcurve at ITS OWN 25%/75% parameters, which is only comparable when the two
    // trims of an edge are parametrised alike. For a CLOSED shared edge -- a cap circle, where
    // one face's pcurve is a full circle and the other's is the same circle started at a
    // different parameter -- those two samples sit at unrelated places on the ring, the |cos|
    // test falls under the 0.05 guard, the edge is dropped from the adjacency, and the shell
    // decomposition shatters. Measured: box(2) intersect cylinder(r=0.7) came out as THREE
    // one-face shells with volumes [-0.5131, -0.5131, +2.0525] summing to 1.026253600, while
    // the per-face fluxes were all correct and sum(outward*flux)/3 = 3.078760801 = the analytic
    // value. Comparing both trims at the SAME 3D point removes the parametrisation dependence.
    auto trim_dir_at = [&](int ti, const Point& ref) -> Vector {
        const int fi = face_of_trim(ti);
        if (fi < 0 || fi >= nf) return Vector(0, 0, 0);
        if (face_fulldom[static_cast<size_t>(fi)]) return trim_dir(ti);
        const BRepTrim& tm = brep.m_trims[static_cast<size_t>(ti)];
        const BRepFace& face = brep.m_faces[static_cast<size_t>(fi)];
        if (face.surface_index < 0 || face.surface_index >= static_cast<int>(brep.m_surfaces.size()))
            return Vector(0, 0, 0);
        if (tm.curve_2d_index < 0 || tm.curve_2d_index >= static_cast<int>(brep.m_curves_2d.size()))
            return Vector(0, 0, 0);
        const NurbsSurface& srf = brep.m_surfaces[static_cast<size_t>(face.surface_index)];
        const NurbsCurve& pc = brep.m_curves_2d[static_cast<size_t>(tm.curve_2d_index)];
        if (!pc.is_valid() || !srf.is_valid()) return Vector(0, 0, 0);
        const auto dc = pc.domain();
        const double span = dc.second - dc.first;
        if (span <= 0.0) return Vector(0, 0, 0);
        const int N = 96;
        double bt = dc.first, bd = 1e300;
        for (int k = 0; k <= N; ++k) {
            const double t = dc.first + span * k / N;
            const Point uv = pc.point_at(t);
            const Point p3 = srf.point_at(uv[0], uv[1]);
            const double d = p3.distance(ref);
            if (d < bd) { bd = d; bt = t; }
        }
        const double h = span * 0.01;
        const double t0 = std::max(dc.first, bt - h), t1 = std::min(dc.second, bt + h);
        if (t1 - t0 < 1e-14) return Vector(0, 0, 0);
        const Point ua = pc.point_at(t0), ub = pc.point_at(t1);
        const Point A = srf.point_at(ua[0], ua[1]);
        const Point B = srf.point_at(ub[0], ub[1]);
        const double s = out.faces[static_cast<size_t>(fi)].traversal *
                         (trim_fwd[static_cast<size_t>(ti)] != 0 ? 1.0 : -1.0);
        return Vector(s * (B[0] - A[0]), s * (B[1] - A[1]), s * (B[2] - A[2]));
    };

    std::vector<int> comp(static_cast<size_t>(std::max(nf, 0)), -1);
    std::vector<double> sgn(static_cast<size_t>(std::max(nf, 0)), 1.0);
    std::vector<std::vector<std::pair<int, double>>> adj(static_cast<size_t>(std::max(nf, 0)));
    for (const BRepEdge& e : brep.m_topology_edges) {
        if (e.trim_indices.size() != 2) continue;
        const int fa = face_of_trim(e.trim_indices[0]);
        const int fb = face_of_trim(e.trim_indices[1]);
        if (fa < 0 || fb < 0 || fa == fb) continue;
        const int ta = e.trim_indices[0], tb = e.trim_indices[1];
        double rel = 0.0;
        if (trim_c3[static_cast<size_t>(ta)] != 0 && trim_c3[static_cast<size_t>(tb)] != 0) {
            // EXACT test: both trims are parametrised by the SAME 3D edge curve, so the sense
            // of each material-left walk along that curve is unambiguous. Consistent
            // orientation <=> the two walks run in OPPOSITE directions along the edge.
            // trim_c3 already folds in the pcurve traversal sense; `traversal` lifts the
            // stored walk to the material-left walk for the face's natural normal.
            const double sa = out.faces[static_cast<size_t>(fa)].traversal *
                              trim_c3[static_cast<size_t>(ta)];
            const double sb = out.faces[static_cast<size_t>(fb)].traversal *
                              trim_c3[static_cast<size_t>(tb)];
            rel = (sa * sb < 0.0) ? 1.0 : -1.0;
        } else {
            // Common 3D reference on the shared edge (its own curve midpoint when it has one,
            // else the lift of trim ta's pcurve midpoint), so both walks are sampled at the
            // same place on the edge regardless of pcurve parametrisation.
            Point ref(0, 0, 0);
            bool have_ref = false;
            if (e.curve_3d_index >= 0 &&
                e.curve_3d_index < static_cast<int>(brep.m_curves_3d.size())) {
                const NurbsCurve& c3 = brep.m_curves_3d[static_cast<size_t>(e.curve_3d_index)];
                if (c3.is_valid()) {
                    const auto d3 = c3.domain();
                    ref = c3.point_at(0.5 * (d3.first + d3.second));
                    have_ref = true;
                }
            }
            if (!have_ref) {
                const int fa2 = face_of_trim(ta);
                const BRepTrim& tma = brep.m_trims[static_cast<size_t>(ta)];
                if (fa2 >= 0 && tma.curve_2d_index >= 0 &&
                    tma.curve_2d_index < static_cast<int>(brep.m_curves_2d.size())) {
                    const BRepFace& fca = brep.m_faces[static_cast<size_t>(fa2)];
                    if (fca.surface_index >= 0 &&
                        fca.surface_index < static_cast<int>(brep.m_surfaces.size())) {
                        const NurbsCurve& pca = brep.m_curves_2d[static_cast<size_t>(tma.curve_2d_index)];
                        const auto dca = pca.domain();
                        const Point uvm = pca.point_at(0.5 * (dca.first + dca.second));
                        ref = brep.m_surfaces[static_cast<size_t>(fca.surface_index)]
                                  .point_at(uvm[0], uvm[1]);
                        have_ref = true;
                    }
                }
            }
            const Vector da = have_ref ? trim_dir_at(ta, ref) : trim_dir(ta);
            const Vector db = have_ref ? trim_dir_at(tb, ref) : trim_dir(tb);
            const double la = da.magnitude(), lb = db.magnitude();
            if (la < 1e-12 || lb < 1e-12) continue;
            const double c = vdot(da, db) / (la * lb);
            if (std::abs(c) < 0.05) continue;
            rel = (c > 0.0) ? -1.0 : 1.0;
        }
        adj[static_cast<size_t>(fa)].push_back({fb, rel});
        adj[static_cast<size_t>(fb)].push_back({fa, rel});
    }
    int ncomp = 0;
    for (int i = 0; i < nf; ++i) {
        if (comp[static_cast<size_t>(i)] >= 0) continue;
        if (out.faces[static_cast<size_t>(i)].path == MassPropsPath::Skipped) continue;
        std::vector<int> stack{i};
        comp[static_cast<size_t>(i)] = ncomp;
        while (!stack.empty()) {
            const int c = stack.back();
            stack.pop_back();
            for (const auto& nb : adj[static_cast<size_t>(c)]) {
                if (comp[static_cast<size_t>(nb.first)] >= 0) continue;
                comp[static_cast<size_t>(nb.first)] = ncomp;
                sgn[static_cast<size_t>(nb.first)] = sgn[static_cast<size_t>(c)] * nb.second;
                stack.push_back(nb.first);
            }
        }
        ++ncomp;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // PASS 3 -- per-shell normalisation, then cavity sign from bbox nesting depth.
    ///////////////////////////////////////////////////////////////////////////////////////
    const size_t nc = static_cast<size_t>(std::max(ncomp, 0));
    std::vector<double> shell_v(nc, 0.0), shell_e(nc, 0.0);
    std::vector<std::array<double, 6>> sbb(nc, {1e300, -1e300, 1e300, -1e300, 1e300, -1e300});
    for (int i = 0; i < nf; ++i) {
        const int c = comp[static_cast<size_t>(i)];
        if (c < 0) continue;
        shell_v[static_cast<size_t>(c)] +=
            sgn[static_cast<size_t>(i)] * out.faces[static_cast<size_t>(i)].flux / 3.0;
        shell_e[static_cast<size_t>(c)] += out.faces[static_cast<size_t>(i)].flux_err / 3.0;
    }
    for (int i = 0; i < nf; ++i) {
        const int c = comp[static_cast<size_t>(i)];
        if (c < 0) continue;
        if (shell_v[static_cast<size_t>(c)] < 0.0)
            sgn[static_cast<size_t>(i)] = -sgn[static_cast<size_t>(i)];
    }
    for (size_t c = 0; c < nc; ++c) shell_v[c] = std::abs(shell_v[c]);

    for (int fi = 0; fi < nf; ++fi) {
        const int c = comp[static_cast<size_t>(fi)];
        if (c < 0) continue;
        const BRepFace& face = brep.m_faces[static_cast<size_t>(fi)];
        if (face.surface_index < 0 || face.surface_index >= static_cast<int>(brep.m_surfaces.size()))
            continue;
        const NurbsSurface& srf = brep.m_surfaces[static_cast<size_t>(face.surface_index)];
        auto& bb = sbb[static_cast<size_t>(c)];
        for (int li : face.loop_indices) {
            if (li < 0 || li >= static_cast<int>(brep.m_loops.size())) continue;
            for (int ti : brep.m_loops[static_cast<size_t>(li)].trim_indices) {
                if (ti < 0 || ti >= static_cast<int>(brep.m_trims.size())) continue;
                const int c2 = brep.m_trims[static_cast<size_t>(ti)].curve_2d_index;
                if (c2 < 0 || c2 >= static_cast<int>(brep.m_curves_2d.size())) continue;
                const NurbsCurve& pc = brep.m_curves_2d[static_cast<size_t>(c2)];
                if (!pc.is_valid()) continue;
                auto dc = pc.domain();
                for (int i = 0; i <= 8; ++i) {
                    const Point uv = pc.point_at(dc.first + (dc.second - dc.first) * i / 8.0);
                    const Point p = srf.point_at(uv[0], uv[1]);
                    bb[0] = std::min(bb[0], p[0]); bb[1] = std::max(bb[1], p[0]);
                    bb[2] = std::min(bb[2], p[1]); bb[3] = std::max(bb[3], p[1]);
                    bb[4] = std::min(bb[4], p[2]); bb[5] = std::max(bb[5], p[2]);
                }
            }
        }
    }
    std::vector<int> depth(nc, 0);
    for (size_t a = 0; a < nc; ++a)
        for (size_t b = 0; b < nc; ++b) {
            if (a == b || shell_v[b] <= shell_v[a]) continue;
            const auto& A = sbb[a];
            const auto& B = sbb[b];
            const double tol = 1e-9 * std::max(1.0, (B[1] - B[0]) + (B[3] - B[2]) + (B[5] - B[4]));
            if (A[0] >= B[0] - tol && A[1] <= B[1] + tol && A[2] >= B[2] - tol &&
                A[3] <= B[3] + tol && A[4] >= B[4] - tol && A[5] <= B[5] + tol)
                ++depth[a];
        }

    ///////////////////////////////////////////////////////////////////////////////////////
    // PASS 4 -- totals.
    ///////////////////////////////////////////////////////////////////////////////////////
    double M[3] = {0, 0, 0}, AV[3] = {0, 0, 0};
    out.shell_volumes.assign(nc, 0.0);
    for (size_t c = 0; c < nc; ++c) {
        const double s = (depth[c] % 2 == 0) ? 1.0 : -1.0;
        out.shell_volumes[c] = s * shell_v[c];
        out.volume += s * shell_v[c];
        out.volume_error += shell_e[c];
    }
    for (int i = 0; i < nf; ++i) {
        FaceMassProps& F = out.faces[static_cast<size_t>(i)];
        const int c = comp[static_cast<size_t>(i)];
        F.shell = c;
        F.outward = (c < 0) ? 1.0 : sgn[static_cast<size_t>(i)];
        out.area += F.area;
        out.area_error += F.area_err;
        out.surface_evals += F.evals;
        if (c < 0) continue;
        const double s = (depth[static_cast<size_t>(c)] % 2 == 0) ? 1.0 : -1.0;
        for (int k = 0; k < 3; ++k) {
            M[k] += s * F.outward * F.mom[k];
            AV[k] += s * F.outward * F.vec_area[k];
        }
    }
    out.shell_count = ncomp;
    for (const FaceMassProps& F : out.faces) out.max_chain_gap = std::max(out.max_chain_gap, F.chain_gap);
    // CLOSURE CERTIFICATE. The net outward vector area of a closed shell is exactly zero
    // (oint n dA = 0). Its residual, normalised by total area, is a dimensionless measure of
    // how far the boundary is from closing -- and the divergence-theorem volume is wrong by
    // O(|residual| * |origin offset| * area), i.e. it is ORIGIN-DEPENDENT, whenever it is not
    // small. Cheap: the components ride along in the same quadrature.
    out.closure_residual =
        std::sqrt(AV[0] * AV[0] + AV[1] * AV[1] + AV[2] * AV[2]) / std::max(1e-300, out.area);
    if (out.closure_residual > 1e-6) out.closed = false;
    // The volume error estimate must own the closure defect: shifting the origin by the model's
    // own size changes the answer by residual*area*size/3, so that is a real uncertainty.
    {
        double mn[3] = {1e300, 1e300, 1e300}, mx[3] = {-1e300, -1e300, -1e300};
        for (const auto& p : brep.m_vertices)
            for (int k = 0; k < 3; ++k) {
                mn[k] = std::min(mn[k], p[k]);
                mx[k] = std::max(mx[k], p[k]);
            }
        double diag = 0.0;
        for (int k = 0; k < 3; ++k)
            if (mx[k] > mn[k]) diag += (mx[k] - mn[k]) * (mx[k] - mn[k]);
        diag = std::sqrt(diag);
        out.volume_error += out.closure_residual * out.area * diag / 3.0;
    }
    if (std::abs(out.volume) > 1e-300)
        out.centroid = Point(M[0] / out.volume, M[1] / out.volume, M[2] / out.volume);
    out.seconds = std::chrono::duration<double>(clk::now() - t_start).count();
    return out;
}

double brep_volume(const BRep& brep, const MassPropsOptions& opt) {
    return brep_massprops(brep, opt).volume;
}
double brep_area(const BRep& brep, const MassPropsOptions& opt) {
    return brep_massprops(brep, opt).area;
}
Point brep_centroid(const BRep& brep, const MassPropsOptions& opt) {
    return brep_massprops(brep, opt).centroid;
}

}  // namespace session_cpp
