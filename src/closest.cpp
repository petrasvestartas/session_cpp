#include "closest.h"
#include "mesh.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "pointcloud.h"
#include "spatial_aabbtree.h"
#include "spatial_bvh.h"
#include "spatial_kdtree.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>

namespace session_cpp {

const int STACK_SIZE = 64;

// ═══════════════════════════════════════════════════════════════════════════
// Curve helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Parameter of the closest sample on a dense grid over [t0, t1].
static double curve_seed(const NurbsCurve& curve, const Point& test_point, double t0, double t1) {

    const int num_samples = std::max(50, curve.cv_count() * 10);
    const double dt = (t1 - t0) / num_samples;
    double best_t = t0;
    double best_dist = curve.point_at(t0).distance(test_point);

    for (int i = 0; i <= num_samples; i++) {
        const double t = t0 + i * dt;
        const double dist = curve.point_at(t).distance(test_point);

        if (dist < best_dist) {
            best_dist = dist;
            best_t = t;
        }
    }

    return best_t;
}

/// Newton on (C(t) - P) . C'(t) = 0 from t, clamped to [t0, t1].
static double curve_newton(const NurbsCurve& curve, const Point& test_point, double t0, double t1, double t) {

    const int max_iterations = 32;
    const double step_tolerance = (t1 - t0) * 1e-12;

    for (int iter = 0; iter < max_iterations; iter++) {
        const std::vector<Vector> derivs = curve.evaluate(t, 2);

        if (derivs.size() < 3)
            break;

        const Vector& pt = derivs[0];
        const Vector& d1 = derivs[1];
        const Vector& d2 = derivs[2];
        const double rx = pt[0] - test_point[0];
        const double ry = pt[1] - test_point[1];
        const double rz = pt[2] - test_point[2];
        const double f = rx * d1[0] + ry * d1[1] + rz * d1[2];

        if (std::abs(f) < step_tolerance)
            break;

        const double df = d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2] + rx * d2[0] + ry * d2[1] + rz * d2[2];

        if (std::abs(df) < 1e-14)
            break;

        double dt_step = -f / df;

        if (std::abs(dt_step) > (t1 - t0) * 0.5)
            dt_step = std::copysign((t1 - t0) * 0.5, dt_step);

        t += dt_step;

        if (t < t0)
            t = t0;

        if (t > t1)
            t = t1;

        if (std::abs(dt_step) < step_tolerance)
            break;
    }

    return t;
}

/// Parameters of the closest pair on dense grids over both domains.
static std::pair<double, double> curve_curve_seed(const NurbsCurve& curve0, const NurbsCurve& curve1) {

    const double u0 = curve0.domain_start();
    const double u1 = curve0.domain_end();
    const double v0 = curve1.domain_start();
    const double v1 = curve1.domain_end();
    const int n0 = std::max(40, curve0.cv_count() * 8);
    const int n1 = std::max(40, curve1.cv_count() * 8);
    std::vector<Point> p0(n0 + 1);
    std::vector<Point> p1(n1 + 1);

    for (int i = 0; i <= n0; i++)
        p0[i] = curve0.point_at(u0 + (u1 - u0) * i / n0);

    for (int j = 0; j <= n1; j++)
        p1[j] = curve1.point_at(v0 + (v1 - v0) * j / n1);

    double best = std::numeric_limits<double>::infinity();
    double u = u0;
    double v = v0;

    for (int i = 0; i <= n0; i++) {
        for (int j = 0; j <= n1; j++) {
            const double d2 = (p0[i] - p1[j]).magnitude_squared();

            if (d2 < best) {
                best = d2;
                u = u0 + (u1 - u0) * i / n0;
                v = v0 + (v1 - v0) * j / n1;
            }
        }
    }

    return {u, v};
}

// ═══════════════════════════════════════════════════════════════════════════
// Surface helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Parameters of the closest sample on a grid whose resolution follows the window size.
static std::pair<double, double> surface_seed(
    const NurbsSurface& surface,
    const Point& test_point,
    double u0,
    double u1,
    double v0,
    double v1
) {

    double domain_u0 = 0.0;
    double domain_u1 = 0.0;
    double domain_v0 = 0.0;
    double domain_v1 = 0.0;
    std::tie(domain_u0, domain_u1) = surface.domain(0);
    std::tie(domain_v0, domain_v1) = surface.domain(1);

    const int full_u = std::max(10, surface.order(0));
    const int full_v = std::max(10, surface.order(1));
    const double u_frac = (u1 - u0) / std::max(domain_u1 - domain_u0, 1e-12);
    const double v_frac = (v1 - v0) / std::max(domain_v1 - domain_v0, 1e-12);
    const int u_samples = std::max(3, (int)std::ceil(full_u * std::min(1.0, u_frac)));
    const int v_samples = std::max(3, (int)std::ceil(full_v * std::min(1.0, v_frac)));
    const double du_param = (u1 - u0) / u_samples;
    const double dv_param = (v1 - v0) / v_samples;
    double best_u = u0;
    double best_v = v0;
    double best_dist = std::numeric_limits<double>::infinity();

    for (int i = 0; i <= u_samples; i++) {
        for (int j = 0; j <= v_samples; j++) {
            const double uu = u0 + i * du_param;
            const double vv = v0 + j * dv_param;
            const double dist = surface.point_at(uu, vv).distance(test_point);

            if (dist < best_dist) {
                best_dist = dist;
                best_u = uu;
                best_v = vv;
            }
        }
    }

    return {best_u, best_v};
}

/// Newton on the perpendicular-foot conditions from the seed, clamped to the window.
static std::pair<double, double> surface_newton(
    const NurbsSurface& surface,
    const Point& test_point,
    double u0,
    double u1,
    double v0,
    double v1,
    std::pair<double, double> seed
) {

    double u = seed.first;
    double v = seed.second;
    const int max_iterations = 20;
    const double step_tolerance = std::min(u1 - u0, v1 - v0) * 1e-10;
    const double max_step = std::min(u1 - u0, v1 - v0) * 0.5;

    for (int iter = 0; iter < max_iterations; iter++) {
        const std::vector<Vector> derivs = surface.evaluate(u, v, 1);

        if (derivs.size() < 3)
            break;

        const Point pt = surface.point_at(u, v);
        const Vector du_vec = derivs[2];
        const Vector dv_vec = derivs[1];
        const Vector delta = test_point - pt;
        const double fu = -delta.dot(du_vec);
        const double fv = -delta.dot(dv_vec);

        if (std::abs(fu) < step_tolerance && std::abs(fv) < step_tolerance)
            break;

        const double duu = du_vec.dot(du_vec);
        const double dvv = dv_vec.dot(dv_vec);
        const double duv = du_vec.dot(dv_vec);
        const double det = duu * dvv - duv * duv;

        if (std::abs(det) < 1e-12)
            break;

        double du_step = (dvv * fu - duv * fv) / det;
        double dv_step = (duu * fv - duv * fu) / det;

        if (std::abs(du_step) > max_step)
            du_step = std::copysign(max_step, du_step);

        if (std::abs(dv_step) > max_step)
            dv_step = std::copysign(max_step, dv_step);

        u = std::max(u0, std::min(u1, u - du_step));
        v = std::max(v0, std::min(v1, v - dv_step));

        if (std::abs(du_step) < step_tolerance && std::abs(dv_step) < step_tolerance)
            break;
    }

    return {u, v};
}

// ═══════════════════════════════════════════════════════════════════════════
// Pullback helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Surface domain, trace step and tolerances shared by the surface_curve steps.
struct Pullback {
    double u0 = 0.0; // Surface domain start in u.
    double u1 = 0.0; // Surface domain end in u.
    double v0 = 0.0; // Surface domain start in v.
    double v1 = 0.0; // Surface domain end in v.
    double range_u = 0.0; // Domain length in u.
    double range_v = 0.0; // Domain length in v.
    bool closed_u = false; // Surface closed in u.
    bool closed_v = false; // Surface closed in v.
    double du = 0.0; // Quarter-span step in u.
    double dv = 0.0; // Quarter-span step in v.
    double step = 0.0; // Uv deviation bound of a fitted pcurve.
    double fit_tol = 0.0; // 3d deviation bound of a lifted uv midpoint.
    double reject_tol = 0.0; // Residual above which a sample is re-inverted globally.
    double on_surf_tol = 0.0; // Residual above which the curve is off the surface.
};

/// Domain, steps and tolerances of the surface for one pullback.
static Pullback pullback_setup(const NurbsSurface& surface, double tolerance) {

    Pullback pb;
    std::tie(pb.u0, pb.u1) = surface.domain(0);
    std::tie(pb.v0, pb.v1) = surface.domain(1);
    pb.range_u = pb.u1 - pb.u0;
    pb.range_v = pb.v1 - pb.v0;
    pb.closed_u = surface.is_closed(0);
    pb.closed_v = surface.is_closed(1);

    const int nu = std::max((int)surface.get_span_vector(0).size() - 1, 1) * 4;
    const int nv = std::max((int)surface.get_span_vector(1).size() - 1, 1) * 4;
    pb.du = pb.range_u / nu;
    pb.dv = pb.range_v / nv;

    const double mu = (pb.u0 + pb.u1) * 0.5;
    const double mv = (pb.v0 + pb.v1) * 0.5;
    const Point pmid = surface.point_at(mu, mv);
    const double wu_probe = std::min(mu + pb.du, pb.u1);
    const double wv_probe = std::min(mv + pb.dv, pb.v1);
    const double uv_to_3d_u = pmid.distance(surface.point_at(wu_probe, mv)) / pb.du;
    const double uv_to_3d_v = pmid.distance(surface.point_at(mu, wv_probe)) / pb.dv;
    double uv_to_3d = std::max(uv_to_3d_u, uv_to_3d_v);
    double uv_to_3d_min = std::min(uv_to_3d_u, uv_to_3d_v);

    if (uv_to_3d < 1e-10)
        uv_to_3d = 1.0;

    if (uv_to_3d_min < 1e-10)
        uv_to_3d_min = 1.0;

    pb.step = std::min(pb.du, pb.dv) * 0.25;
    pb.fit_tol = tolerance > 0.0 ? tolerance : pb.step * (uv_to_3d + uv_to_3d_min) * 0.5;
    pb.reject_tol = pb.fit_tol * 100.0;

    double corner_diag = surface.point_at(pb.u0, pb.v0).distance(surface.point_at(pb.u1, pb.v1));

    if (corner_diag < 1e-12)
        corner_diag = std::max(pb.range_u, pb.range_v);

    pb.on_surf_tol = corner_diag * 0.05;

    return pb;
}

/// x folded into [x0, x1] by period when closed, clamped otherwise.
static double pullback_wrap(double x, double x0, double x1, bool closed) {

    if (!closed)
        return std::max(x0, std::min(x, x1));

    double t = std::fmod(x - x0, x1 - x0);

    if (t < 0)
        t += x1 - x0;

    return x0 + t;
}

/// x shifted by whole periods to within half a period of prev.
static double pullback_unwrap(double prev, double x, double range, bool closed) {

    if (!closed)
        return x;

    while (x - prev > range * 0.5)
        x -= range;

    while (x - prev < -range * 0.5)
        x += range;

    return x;
}

/// Windowed inversion of pt around (up, vp), trying the seam-mirrored windows when closed.
static std::tuple<double, double, double> pullback_invert(
    const NurbsSurface& surface,
    const Pullback& pb,
    const Point& pt,
    double up,
    double vp,
    double wu,
    double wv
) {

    std::vector<double> u_centers = {up};

    if (pb.closed_u && up - wu < pb.u0)
        u_centers.push_back(up + pb.range_u);

    if (pb.closed_u && up + wu > pb.u1)
        u_centers.push_back(up - pb.range_u);

    std::vector<double> v_centers = {vp};

    if (pb.closed_v && vp - wv < pb.v0)
        v_centers.push_back(vp + pb.range_v);

    if (pb.closed_v && vp + wv > pb.v1)
        v_centers.push_back(vp - pb.range_v);

    std::tuple<double, double, double> best = {up, vp, std::numeric_limits<double>::infinity()};

    for (const double uc : u_centers) {
        for (const double vc : v_centers) {
            const double wu0 = std::max(uc - wu, pb.u0);
            const double wu1 = std::min(uc + wu, pb.u1);
            const double wv0 = std::max(vc - wv, pb.v0);
            const double wv1 = std::min(vc + wv, pb.v1);

            if (wu1 - wu0 < 1e-14 || wv1 - wv0 < 1e-14)
                continue;

            const std::tuple<double, double, double> res = Closest::surface_point(surface, pt, wu0, wu1, wv0, wv1);

            if (std::get<2>(res) < std::get<2>(best))
                best = res;

            if (std::get<2>(best) < pb.fit_tol * 0.01)
                break;
        }
    }

    return best;
}

/// Warm-started samples [t, u, v, residual] along [t0, t1], empty when the curve is off the surface.
static std::vector<std::array<double, 4>> pullback_samples(
    const NurbsSurface& surface,
    const NurbsCurve& curve,
    const Pullback& pb,
    double t0,
    double t1
) {

    const int n0 = std::max(16, 4 * curve.span_count());
    std::vector<std::array<double, 4>> samples;
    double max_residual = 0.0;
    double min_residual = std::numeric_limits<double>::infinity();

    for (int i = 0; i <= n0; i++) {
        const double t = t0 + (t1 - t0) * i / n0;
        const Point pt = curve.point_at(t);
        double uu = 0.0;
        double vv = 0.0;
        double rd = 0.0;

        if (i == 0) {
            std::tie(uu, vv, rd) = Closest::surface_point(surface, pt, 0.0, 0.0, 0.0, 0.0);
        } else {
            const std::array<double, 4> prev = samples.back();
            const std::array<double, 4> prev2 = samples[std::max(0, (int)samples.size() - 2)];
            const double wu = std::max(pb.du, pb.dv) * 2.0 + std::abs(prev[1] - prev2[1]);
            const double wv = std::max(pb.du, pb.dv) * 2.0 + std::abs(prev[2] - prev2[2]);
            const double up = pullback_wrap(prev[1], pb.u0, pb.u1, pb.closed_u);
            const double vp = pullback_wrap(prev[2], pb.v0, pb.v1, pb.closed_v);
            double ru = 0.0;
            double rv = 0.0;
            std::tie(ru, rv, rd) = pullback_invert(surface, pb, pt, up, vp, wu, wv);

            if (rd > pb.reject_tol)
                std::tie(ru, rv, rd) = Closest::surface_point(surface, pt, 0.0, 0.0, 0.0, 0.0);

            uu = pullback_unwrap(prev[1], ru, pb.range_u, pb.closed_u);
            vv = pullback_unwrap(prev[2], rv, pb.range_v, pb.closed_v);
        }

        samples.push_back(std::array<double, 4>{t, uu, vv, rd});
        max_residual = std::max(max_residual, rd);
        min_residual = std::min(min_residual, rd);
    }

    if (max_residual > pb.reject_tol || min_residual > pb.on_surf_tol)
        samples.clear();

    return samples;
}

/// Bisect every span whose lifted uv midpoint strays from the curve, up to 8 rounds or 4096 samples.
static void pullback_refine(
    const NurbsSurface& surface,
    const NurbsCurve& curve,
    const Pullback& pb,
    std::vector<std::array<double, 4>>& samples
) {

    for (int depth = 0; depth < 8; depth++) {
        int inserted = 0;
        size_t i = 0;

        while (i + 1 < samples.size()) {
            const std::array<double, 4> a = samples[i];
            const std::array<double, 4> b = samples[i + 1];
            const double tm = (a[0] + b[0]) * 0.5;
            const double um = pullback_wrap((a[1] + b[1]) * 0.5, pb.u0, pb.u1, pb.closed_u);
            const double vm = pullback_wrap((a[2] + b[2]) * 0.5, pb.v0, pb.v1, pb.closed_v);
            const Point pm = curve.point_at(tm);

            if (surface.point_at(um, vm).distance(pm) <= pb.fit_tol || samples.size() >= 4096) {
                i += 1;
                continue;
            }

            const double wu = std::max(std::abs(b[1] - a[1]), pb.du);
            const double wv = std::max(std::abs(b[2] - a[2]), pb.dv);
            double ru = 0.0;
            double rv = 0.0;
            double rd = 0.0;
            std::tie(ru, rv, rd) = pullback_invert(surface, pb, pm, um, vm, wu, wv);

            if (rd > pb.on_surf_tol) {
                i += 1;
                continue;
            }

            const double uu = pullback_unwrap(a[1], ru, pb.range_u, pb.closed_u);
            const double vv = pullback_unwrap(a[2], rv, pb.range_v, pb.closed_v);

            samples.insert(samples.begin() + i + 1, std::array<double, 4>{tm, uu, vv, rd});
            inserted += 1;
            i += 2;
        }

        if (inserted == 0)
            break;
    }
}

/// Smallest seam crossing of one axis between a and b that beats bestt; level is the seam value.
static bool pullback_seam_axis(
    double a,
    double b,
    double x0,
    double range,
    bool closed,
    double& bestt,
    double& level
) {

    if (!closed || std::abs(b - a) <= 1e-15)
        return false;

    const int k0 = (int)std::floor((a - x0) / range);
    const int k1 = (int)std::floor((b - x0) / range);
    bool found = false;

    for (int k = std::min(k0, k1) + 1; k <= std::max(k0, k1); k++) {
        const double seam = x0 + k * range;
        const double t = (seam - a) / (b - a);

        if (t > 1e-9 && t < 1.0 - 1e-9 && t < bestt) {
            bestt = t;
            level = seam;
            found = true;
        }
    }

    return found;
}

/// First seam crossing on segment a -> b, written to (cu, cv).
static bool pullback_first_seam(
    const Pullback& pb,
    const std::pair<double, double>& a,
    const std::pair<double, double>& b,
    double& cu,
    double& cv
) {

    double bestt = 2.0;
    double level = 0.0;
    bool found = false;

    if (pullback_seam_axis(a.first, b.first, pb.u0, pb.range_u, pb.closed_u, bestt, level)) {
        found = true;
        cu = level;
        cv = a.second + (b.second - a.second) * bestt;
    }

    if (pullback_seam_axis(a.second, b.second, pb.v0, pb.range_v, pb.closed_v, bestt, level)) {
        found = true;
        cv = level;
        cu = a.first + (b.first - a.first) * bestt;
    }

    return found;
}

/// True when x sits on a seam level of a closed axis.
static bool pullback_at_seam(double x, double x0, double range, bool closed) {

    if (!closed)
        return false;

    const double seam = x0 + std::round((x - x0) / range) * range;

    return std::abs(x - seam) < range * 1e-6;
}

/// True when p sits on a seam of either closed axis.
static bool pullback_on_seam(const Pullback& pb, const std::pair<double, double>& p) {

    return pullback_at_seam(p.first, pb.u0, pb.range_u, pb.closed_u) ||
        pullback_at_seam(p.second, pb.v0, pb.range_v, pb.closed_v);
}

/// Shift a segment by whole periods so its middle point lies inside the domain.
static void pullback_shift(const Pullback& pb, std::vector<std::pair<double, double>>& seg) {

    const std::pair<double, double> mid = seg[seg.size() / 2];
    const int k_u = pb.closed_u ? (int)std::floor((mid.first - pb.u0) / pb.range_u) : 0;
    const int k_v = pb.closed_v ? (int)std::floor((mid.second - pb.v0) / pb.range_v) : 0;

    for (std::pair<double, double>& p : seg) {
        p.first -= k_u * pb.range_u;
        p.second -= k_v * pb.range_v;
    }
}

/// Split the unwrapped uv polyline at every seam crossing; rejoin the two arcs of a mid-arc loop start.
static std::vector<std::vector<std::pair<double, double>>> pullback_split(
    const Pullback& pb,
    const std::vector<std::pair<double, double>>& pts,
    bool rejoin,
    bool& any_cross
) {

    std::vector<std::vector<std::pair<double, double>>> raw;
    std::vector<std::pair<double, double>> cur = {pts[0]};
    any_cross = false;

    for (size_t i = 1; i < pts.size(); i++) {
        std::pair<double, double> a = pts[i - 1];
        const std::pair<double, double> b = pts[i];
        double cu = 0.0;
        double cv = 0.0;

        while (pullback_first_seam(pb, a, b, cu, cv)) {
            cur.push_back({cu, cv});
            raw.push_back(cur);
            cur = {{cu, cv}};
            any_cross = true;
            a = {cu, cv};
        }

        cur.push_back(b);

        if (i + 1 < pts.size() && pullback_on_seam(pb, b)) {
            raw.push_back(cur);
            cur = {b};
            any_cross = true;
        }
    }

    raw.push_back(cur);

    if (rejoin && raw.size() > 1) {
        std::vector<std::pair<double, double>> merged = raw.back();

        for (size_t k = 1; k < raw.front().size(); k++)
            merged.push_back(raw.front()[k]);

        raw.erase(raw.begin());
        raw.back() = merged;
    }

    return raw;
}

/// In-domain uv pieces with a closed flag, slivers dropped.
static std::vector<std::pair<std::vector<std::pair<double, double>>, bool>> pullback_pieces(
    const Pullback& pb,
    const NurbsCurve& curve,
    const std::vector<std::array<double, 4>>& samples
) {

    std::vector<std::pair<double, double>> pts;

    for (const std::array<double, 4>& s : samples)
        pts.push_back({s[1], s[2]});

    const Point p_first = curve.point_at(samples.front()[0]);
    const Point p_last = curve.point_at(samples.back()[0]);
    const bool is_loop = p_first.distance(p_last) < pb.fit_tol * 4.0 && pts.size() >= 6;

    if (is_loop)
        pts.pop_back();

    const double wind_u = pb.closed_u ? samples.back()[1] - samples.front()[1] : 0.0;
    const double wind_v = pb.closed_v ? samples.back()[2] - samples.front()[2] : 0.0;
    const bool crosses = std::abs(wind_u) > pb.range_u * 0.5 || std::abs(wind_v) > pb.range_v * 0.5;
    const bool rejoin = is_loop && !crosses && !pullback_on_seam(pb, pts[0]);
    bool any_cross = false;
    std::vector<std::vector<std::pair<double, double>>> raw = pullback_split(pb, pts, rejoin, any_cross);
    std::vector<std::pair<std::vector<std::pair<double, double>>, bool>> pieces;

    for (std::vector<std::pair<double, double>>& seg : raw) {
        if (seg.size() < 2)
            continue;

        pullback_shift(pb, seg);

        double umin = 1e300;
        double umax = -1e300;
        double vmin = 1e300;
        double vmax = -1e300;
        double len = 0.0;

        for (size_t i = 0; i < seg.size(); i++) {
            umin = std::min(umin, seg[i].first);
            umax = std::max(umax, seg[i].first);
            vmin = std::min(vmin, seg[i].second);
            vmax = std::max(vmax, seg[i].second);

            if (i > 0)
                len += std::hypot(seg[i].first - seg[i - 1].first, seg[i].second - seg[i - 1].second);
        }

        if (len < std::min(pb.range_u, pb.range_v) * 1e-4)
            continue;

        const bool seg_loop = is_loop && !any_cross && umax - umin < pb.range_u * 0.9 && vmax - vmin < pb.range_v * 0.9;

        pieces.push_back({seg, seg_loop});
    }

    return pieces;
}

/// Total turning angle of a uv polyline.
static double pullback_turning(const std::vector<Point>& pts_uv) {

    const int mp = (int)pts_uv.size();
    double total_turning = 0.0;

    for (int i = 1; i < mp - 1; i++) {
        const double dx1 = pts_uv[i][0] - pts_uv[i - 1][0];
        const double dy1 = pts_uv[i][1] - pts_uv[i - 1][1];
        const double dx2 = pts_uv[i + 1][0] - pts_uv[i][0];
        const double dy2 = pts_uv[i + 1][1] - pts_uv[i][1];
        const double l1 = std::hypot(dx1, dy1);
        const double l2 = std::hypot(dx2, dy2);

        if (l1 <= 1e-14 || l2 <= 1e-14)
            continue;

        const double c = std::max(-1.0, std::min(1.0, (dx1 * dx2 + dy1 * dy2) / (l1 * l2)));
        total_turning += std::acos(c);
    }

    return total_turning;
}

/// Normalized chord-length parameters of a uv polyline.
static std::vector<double> pullback_chords(const std::vector<Point>& pts_uv, bool piece_loop) {

    const int mp = (int)pts_uv.size();
    std::vector<double> chords(mp, 0.0);
    double total_len = 0.0;

    for (int i = 1; i < mp; i++) {
        total_len += pts_uv[i].distance(pts_uv[i - 1]);
        chords[i] = total_len;
    }

    if (piece_loop)
        total_len += pts_uv[0].distance(pts_uv[mp - 1]);

    if (total_len > 1e-14)
        for (int i = 1; i < mp; i++)
            chords[i] /= total_len;

    return chords;
}

/// Fit one piece as a uv pcurve on [0, 1]; interpolation and a degree-1 polyline are the fallbacks.
static NurbsCurve pullback_fit(
    const Pullback& pb,
    std::vector<std::pair<double, double>>& piece_pts,
    bool piece_loop
) {

    pullback_shift(pb, piece_pts);

    std::vector<Point> pts_uv(piece_pts.size());

    for (size_t i = 0; i < piece_pts.size(); i++)
        pts_uv[i] = Point(piece_pts[i].first, piece_pts[i].second, 0.0);

    const int mp = (int)pts_uv.size();
    const std::vector<double> chords = pullback_chords(pts_uv, piece_loop);
    int target_cvs = std::max(8, (int)(pullback_turning(pts_uv) / 0.5) + 6);
    const int max_cvs = mp - 1;
    NurbsCurve pcurve;

    for (int attempt = 0; attempt < 5; attempt++) {
        if (target_cvs > max_cvs)
            break;

        pcurve = NurbsCurve::create_fitted(pts_uv, target_cvs, 3, piece_loop);

        if (!pcurve.is_valid())
            break;

        const double ft0 = pcurve.domain_start();
        const double ft1 = pcurve.domain_end();
        double max_dev = 0.0;

        for (int i = 0; i < mp; i++)
            max_dev = std::max(max_dev, pcurve.point_at(ft0 + (ft1 - ft0) * chords[i]).distance(pts_uv[i]));

        if (max_dev < pb.step)
            break;

        target_cvs = std::min(target_cvs * 2, max_cvs);
    }

    if (!pcurve.is_valid()) {
        if (piece_loop)
            pcurve = NurbsCurve::create_interpolated(pts_uv, CurveNurbsKnotStyle::ChordPeriodic);
        else
            pcurve = NurbsCurve::create_interpolated(pts_uv);
    }

    if (!pcurve.is_valid())
        pcurve = NurbsCurve::create(false, 1, pts_uv);

    if (pcurve.is_valid())
        pcurve.set_domain(0.0, 1.0);

    return pcurve;
}

// ═══════════════════════════════════════════════════════════════════════════
// Mesh helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection 5.1.5).
static Point closest_point_on_triangle(const Point& p, const Point& a, const Point& b, const Point& c) {

    const Vector ab = b - a;
    const Vector ac = c - a;
    const Vector ap = p - a;
    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);

    if (d1 <= 0.0 && d2 <= 0.0)
        return a;

    const Vector bp = p - b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);

    if (d3 >= 0.0 && d4 <= d3)
        return b;

    const double vc = d1 * d4 - d3 * d2;

    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);

        return a + ab * v;
    }

    const Vector cp = p - c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);

    if (d6 >= 0.0 && d5 <= d6)
        return c;

    const double vb = d5 * d2 - d1 * d6;

    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double w = d2 / (d2 - d6);

        return a + ac * w;
    }

    const double va = d3 * d6 - d5 * d4;

    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));

        return b + (c - b) * w;
    }

    const double denom = 1.0 / (va + vb + vc);
    const double v = vb * denom;
    const double w = vc * denom;

    return a + ab * v + ac * w;
}

/// Distance from p to the box, zero inside.
static double aabb_min_distance(const AABB& aabb, const Point& p) {

    const double dx = std::max(0.0, std::abs(p[0] - aabb.cx) - aabb.hx);
    const double dy = std::max(0.0, std::abs(p[1] - aabb.cy) - aabb.hy);
    const double dz = std::max(0.0, std::abs(p[2] - aabb.cz) - aabb.hz);

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// Distance between two boxes, zero when they overlap.
static double aabb_to_aabb_min_dist(const AABB& a, const AABB& b) {

    const double dx = std::max(0.0, std::abs(a.cx - b.cx) - a.hx - b.hx);
    const double dy = std::max(0.0, std::abs(a.cy - b.cy) - a.hy - b.hy);
    const double dz = std::max(0.0, std::abs(a.cz - b.cz) - a.hz - b.hz);

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// Face keys in the face-index order used by the triangle caches.
static std::vector<size_t> mesh_face_keys(const Mesh& mesh) {

    std::vector<size_t> face_keys;
    face_keys.reserve(mesh.face.size());

    for (const std::pair<const size_t, std::vector<size_t>>& entry : mesh.face)
        face_keys.push_back(entry.first);

    return face_keys;
}

/// Closest point, face key and distance on triangle object_id of mesh; infinite distance for an invalid id.
static std::tuple<Point, size_t, double> mesh_triangle_point(const Mesh& mesh, const std::vector<size_t>& face_keys, int object_id, const Point& test_point) {

    Point v0;
    Point v1;
    Point v2;
    size_t face_idx = 0;
    size_t sub_idx = 0;

    if (!mesh.get_triangle_by_id(object_id, face_idx, sub_idx, v0, v1, v2))
        return {Point(0, 0, 0), 0, std::numeric_limits<double>::infinity()};

    const Point cp = closest_point_on_triangle(test_point, v0, v1, v2);

    return {cp, face_keys[face_idx], cp.distance(test_point)};
}

/// Push the children nearer than best_dist, the nearer one last so it pops first.
static void push_nearer_last(int* stack, int& top, int left, int right, double ld, double rd, double best_dist) {

    assert(top + 2 <= STACK_SIZE);

    if (ld <= rd) {
        if (rd < best_dist)
            stack[top++] = right;

        if (ld < best_dist)
            stack[top++] = left;
    } else {
        if (ld < best_dist)
            stack[top++] = left;

        if (rd < best_dist)
            stack[top++] = right;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Curves
// ═══════════════════════════════════════════════════════════════════════════
std::pair<double, double> Closest::curve_point(const NurbsCurve& curve, const Point& test_point, double t0, double t1) {

    if (!curve.is_valid())
        return {0.0, std::numeric_limits<double>::infinity()};

    const double domain_start = curve.domain_start();
    const double domain_end = curve.domain_end();

    if (t0 <= 0.0)
        t0 = domain_start;

    if (t1 <= 0.0)
        t1 = domain_end;

    t0 = std::max(t0, domain_start);
    t1 = std::min(t1, domain_end);

    double t = curve_newton(curve, test_point, t0, t1, curve_seed(curve, test_point, t0, t1));
    double final_dist = curve.point_at(t).distance(test_point);
    const double dist_start = curve.point_at(t0).distance(test_point);
    const double dist_end = curve.point_at(t1).distance(test_point);

    if (dist_start < final_dist) {
        t = t0;
        final_dist = dist_start;
    }

    if (dist_end < final_dist) {
        t = t1;
        final_dist = dist_end;
    }

    return {t, final_dist};
}

std::tuple<double, double, double> Closest::curve_curve(const NurbsCurve& curve0, const NurbsCurve& curve1) {

    if (!curve0.is_valid() || !curve1.is_valid())
        return {0.0, 0.0, std::numeric_limits<double>::infinity()};

    const double u0 = curve0.domain_start();
    const double u1 = curve0.domain_end();
    const double v0 = curve1.domain_start();
    const double v1 = curve1.domain_end();
    double u = 0.0;
    double v = 0.0;
    std::tie(u, v) = curve_curve_seed(curve0, curve1);

    for (int iter = 0; iter < 64; iter++) {
        const std::vector<Vector> e0 = curve0.evaluate(u, 2);
        const std::vector<Vector> e1 = curve1.evaluate(v, 2);

        if (e0.size() < 3 || e1.size() < 3)
            break;

        const Vector& c0 = e0[0];
        const Vector& c0p = e0[1];
        const Vector& c0pp = e0[2];
        const Vector& c1 = e1[0];
        const Vector& c1p = e1[1];
        const Vector& c1pp = e1[2];
        const double rx = c0[0] - c1[0];
        const double ry = c0[1] - c1[1];
        const double rz = c0[2] - c1[2];
        const double gu = rx * c0p[0] + ry * c0p[1] + rz * c0p[2];
        const double gv = -(rx * c1p[0] + ry * c1p[1] + rz * c1p[2]);
        const double huu = c0p[0] * c0p[0] + c0p[1] * c0p[1] + c0p[2] * c0p[2] + rx * c0pp[0] + ry * c0pp[1] + rz * c0pp[2];
        const double huv = -(c0p[0] * c1p[0] + c0p[1] * c1p[1] + c0p[2] * c1p[2]);
        const double hvv = c1p[0] * c1p[0] + c1p[1] * c1p[1] + c1p[2] * c1p[2] - (rx * c1pp[0] + ry * c1pp[1] + rz * c1pp[2]);
        const double det = huu * hvv - huv * huv;

        if (std::abs(det) < 1e-14)
            break;

        double du = -(hvv * gu - huv * gv) / det;
        double dv = -(-huv * gu + huu * gv) / det;

        if (std::abs(du) > (u1 - u0) * 0.5)
            du = std::copysign((u1 - u0) * 0.5, du);

        if (std::abs(dv) > (v1 - v0) * 0.5)
            dv = std::copysign((v1 - v0) * 0.5, dv);

        u = std::min(std::max(u + du, u0), u1);
        v = std::min(std::max(v + dv, v0), v1);

        if (std::max(std::abs(du), std::abs(dv)) < 1e-13)
            break;
    }

    const double dist = curve0.point_at(u).distance(curve1.point_at(v));

    return {u, v, dist};
}

std::tuple<Point, double, double> Closest::line_point(const Line& line, const Point& test_point) {

    const Point start = line.start();
    const Point end = line.end();
    const Vector direction = end - start;
    const double len_sq = direction.magnitude_squared();

    if (len_sq < 1e-20)
        return {start, 0.0, start.distance(test_point)};

    const double t = std::max(0.0, std::min(1.0, (test_point - start).dot(direction) / len_sq));
    const Point closest = start + direction * t;

    return {closest, t, closest.distance(test_point)};
}

std::tuple<Point, double, double> Closest::polyline_point(const Polyline& polyline, const Point& test_point) {

    const std::vector<Point> points = polyline.get_points();

    if (points.empty())
        return {Point(0, 0, 0), 0.0, std::numeric_limits<double>::infinity()};

    if (points.size() == 1)
        return {points[0], 0.0, points[0].distance(test_point)};

    Point best_point = points[0];
    double best_param = 0.0;
    double best_dist = std::numeric_limits<double>::infinity();
    double cumulative_length = 0.0;
    const double total_length = polyline.length();

    for (size_t i = 0; i < points.size() - 1; i++) {
        const Line segment = Line::from_points(points[i], points[i + 1]);
        const double segment_length = segment.length();
        Point closest;
        double t = 0.0;
        double dist = 0.0;
        std::tie(closest, t, dist) = line_point(segment, test_point);

        if (dist < best_dist) {
            best_dist = dist;
            best_point = closest;

            if (total_length > 1e-20)
                best_param = (cumulative_length + t * segment_length) / total_length;
            else
                best_param = static_cast<double>(i) / (points.size() - 1);
        }

        cumulative_length += segment_length;
    }

    return {best_point, best_param, best_dist};
}

// ═══════════════════════════════════════════════════════════════════════════
// Surfaces
// ═══════════════════════════════════════════════════════════════════════════
std::tuple<double, double, double> Closest::surface_point(
    const NurbsSurface& surface,
    const Point& test_point,
    double u0,
    double u1,
    double v0,
    double v1
) {

    if (!surface.is_valid())
        return {0.0, 0.0, std::numeric_limits<double>::infinity()};

    double domain_u0 = 0.0;
    double domain_u1 = 0.0;
    double domain_v0 = 0.0;
    double domain_v1 = 0.0;
    std::tie(domain_u0, domain_u1) = surface.domain(0);
    std::tie(domain_v0, domain_v1) = surface.domain(1);

    if (u0 <= 0.0)
        u0 = domain_u0;

    if (u1 <= 0.0)
        u1 = domain_u1;

    if (v0 <= 0.0)
        v0 = domain_v0;

    if (v1 <= 0.0)
        v1 = domain_v1;

    u0 = std::max(u0, domain_u0);
    u1 = std::min(u1, domain_u1);
    v0 = std::max(v0, domain_v0);
    v1 = std::min(v1, domain_v1);

    const std::pair<double, double> seed = surface_seed(surface, test_point, u0, u1, v0, v1);
    double u = 0.0;
    double v = 0.0;
    std::tie(u, v) = surface_newton(surface, test_point, u0, u1, v0, v1, seed);

    return {u, v, surface.point_at(u, v).distance(test_point)};
}

std::vector<NurbsCurve> Closest::surface_curve(
    const NurbsSurface& surface,
    const NurbsCurve& curve,
    double t0,
    double t1,
    double tolerance
) {

    if (!surface.is_valid() || !curve.is_valid())
        return {};

    const double ct0 = curve.domain_start();
    const double ct1 = curve.domain_end();

    if (t0 <= 0.0)
        t0 = ct0;

    if (t1 <= 0.0)
        t1 = ct1;

    t0 = std::max(t0, ct0);
    t1 = std::min(t1, ct1);

    if (t1 - t0 < 1e-14)
        return {};

    const Pullback pb = pullback_setup(surface, tolerance);
    std::vector<std::array<double, 4>> samples = pullback_samples(surface, curve, pb, t0, t1);

    if (samples.empty())
        return {};

    pullback_refine(surface, curve, pb, samples);

    std::vector<NurbsCurve> result;

    for (std::pair<std::vector<std::pair<double, double>>, bool>& piece : pullback_pieces(pb, curve, samples)) {
        const NurbsCurve pcurve = pullback_fit(pb, piece.first, piece.second);

        if (pcurve.is_valid())
            result.push_back(pcurve);
    }

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Meshes and clouds
// ═══════════════════════════════════════════════════════════════════════════
std::tuple<Point, size_t, double> Closest::mesh_point(const Mesh& mesh, const Point& test_point) {

    Point best_point(0, 0, 0);
    size_t best_face_key = 0;
    double best_dist = std::numeric_limits<double>::infinity();

    if (mesh.number_of_faces() == 0)
        return {best_point, best_face_key, best_dist};

    mesh.build_triangle_bvh();
    const SpatialBVH* bvh = mesh.get_cached_bvh();

    if (!bvh || bvh->empty())
        return {best_point, best_face_key, best_dist};

    const std::vector<size_t> face_keys = mesh_face_keys(mesh);
    int stack[STACK_SIZE];
    int top = 0;
    stack[top++] = 0;

    while (top > 0) {
        const SpatialBVH::Node& node = bvh->nodes[stack[--top]];

        if (aabb_min_distance(node.aabb, test_point) >= best_dist)
            continue;

        if (node.is_leaf()) {
            const std::tuple<Point, size_t, double> hit = mesh_triangle_point(mesh, face_keys, node.object_id, test_point);

            if (std::get<2>(hit) < best_dist)
                std::tie(best_point, best_face_key, best_dist) = hit;

            continue;
        }

        const double ld = aabb_min_distance(bvh->nodes[node.left].aabb, test_point);
        const double rd = aabb_min_distance(bvh->nodes[node.right].aabb, test_point);

        push_nearer_last(stack, top, node.left, node.right, ld, rd, best_dist);
    }

    return {best_point, best_face_key, best_dist};
}

std::tuple<Point, size_t, double> Closest::mesh_point_aabb(const Mesh& mesh, const Point& test_point) {

    Point best_point(0, 0, 0);
    size_t best_face_key = 0;
    double best_dist = std::numeric_limits<double>::infinity();

    if (mesh.number_of_faces() == 0)
        return {best_point, best_face_key, best_dist};

    mesh.build_triangle_aabb_tree();
    const SpatialAABBTree* tree = mesh.get_cached_aabb_tree();

    if (!tree || tree->empty())
        return {best_point, best_face_key, best_dist};

    const std::vector<size_t> face_keys = mesh_face_keys(mesh);
    int stack[STACK_SIZE];
    int top = 0;
    stack[top++] = 0;

    while (top > 0) {
        const int ni = stack[--top];
        const SpatialAABBTree::Node& node = tree->nodes[ni];

        if (aabb_min_distance(node.aabb, test_point) >= best_dist)
            continue;

        if (node.object_id >= 0) {
            const std::tuple<Point, size_t, double> hit = mesh_triangle_point(mesh, face_keys, node.object_id, test_point);

            if (std::get<2>(hit) < best_dist)
                std::tie(best_point, best_face_key, best_dist) = hit;

            continue;
        }

        const int left = ni + 1;
        const int right = node.right;
        const double ld = aabb_min_distance(tree->nodes[left].aabb, test_point);
        const double rd = aabb_min_distance(tree->nodes[right].aabb, test_point);

        push_nearer_last(stack, top, left, right, ld, rd, best_dist);
    }

    return {best_point, best_face_key, best_dist};
}

std::tuple<Point, size_t, double> Closest::pointcloud_point(const PointCloud& cloud, const Point& test_point) {

    if (cloud.point_count() == 0)
        return {Point(0, 0, 0), 0, std::numeric_limits<double>::infinity()};

    Point best_point = cloud.get_point(0);
    size_t best_index = 0;
    double best_dist = best_point.distance(test_point);

    for (size_t i = 1; i < cloud.point_count(); i++) {
        const Point p = cloud.get_point(i);
        const double dist = p.distance(test_point);

        if (dist < best_dist) {
            best_dist = dist;
            best_point = p;
            best_index = i;
        }
    }

    return {best_point, best_index, best_dist};
}

std::tuple<Point, size_t, double> Closest::pointcloud_point_kdtree(const PointCloud& cloud, const Point& test_point) {

    if (cloud.point_count() == 0)
        return {Point(0, 0, 0), 0, std::numeric_limits<double>::infinity()};

    std::vector<Point> pts;
    pts.reserve(cloud.point_count());

    for (size_t i = 0; i < cloud.point_count(); i++)
        pts.push_back(cloud.get_point(i));

    const SpatialKDTree kd(std::move(pts));
    int idx = 0;
    double dist = 0.0;
    std::tie(idx, dist) = kd.nearest(test_point);

    return {cloud.get_point(idx), static_cast<size_t>(idx), dist};
}

// ═══════════════════════════════════════════════════════════════════════════
// Collections
// ═══════════════════════════════════════════════════════════════════════════
std::vector<std::pair<size_t, size_t>> Closest::lines_closest(const std::vector<Line>& lines, double threshold) {

    std::vector<std::pair<size_t, size_t>> pairs;

    if (threshold < 0.0 || lines.size() < 2)
        return pairs;

    std::vector<AABB> aabbs;
    aabbs.reserve(lines.size());

    for (const Line& ln : lines)
        aabbs.push_back(AABB::from_line(ln, threshold));

    SpatialAABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    for (size_t i = 0; i < lines.size(); i++) {
        for (const int j_raw : tree.query_aabb(aabbs[i])) {
            const size_t j = static_cast<size_t>(j_raw);

            if (j <= i)
                continue;

            const double d_a = std::get<2>(line_point(lines[j], lines[i].start()));
            const double d_b = std::get<2>(line_point(lines[j], lines[i].end()));
            const double d_c = std::get<2>(line_point(lines[i], lines[j].start()));
            const double d_d = std::get<2>(line_point(lines[i], lines[j].end()));

            if (std::min({d_a, d_b, d_c, d_d}) <= threshold)
                pairs.push_back({i, j});
        }
    }

    return pairs;
}

std::vector<std::pair<size_t, size_t>> Closest::polylines_closest(
    const std::vector<Polyline>& polylines,
    double threshold
) {

    std::vector<std::pair<size_t, size_t>> pairs;

    if (threshold < 0.0 || polylines.size() < 2)
        return pairs;

    std::vector<AABB> aabbs;
    aabbs.reserve(polylines.size());

    for (const Polyline& pl : polylines)
        aabbs.push_back(AABB::from_polyline(pl, threshold));

    SpatialAABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    for (size_t i = 0; i < polylines.size(); i++) {
        for (const int j_raw : tree.query_aabb(aabbs[i])) {
            const size_t j = static_cast<size_t>(j_raw);

            if (j <= i)
                continue;

            double dist = std::numeric_limits<double>::infinity();

            for (const Point& pt : polylines[i].get_points()) {
                const double d = std::get<2>(polyline_point(polylines[j], pt));

                if (d < dist)
                    dist = d;
            }

            if (dist <= threshold)
                pairs.push_back({i, j});
        }
    }

    return pairs;
}

std::vector<std::pair<size_t, size_t>> Closest::nurbscurves_closest(
    const std::vector<NurbsCurve>& curves,
    double threshold
) {

    std::vector<std::pair<size_t, size_t>> pairs;

    if (threshold < 0.0 || curves.size() < 2)
        return pairs;

    std::vector<AABB> aabbs;
    aabbs.reserve(curves.size());

    for (const NurbsCurve& crv : curves)
        aabbs.push_back(AABB::from_nurbscurve(crv, threshold, false));

    SpatialAABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    for (size_t i = 0; i < curves.size(); i++) {
        for (const int j_raw : tree.query_aabb(aabbs[i])) {
            const size_t j = static_cast<size_t>(j_raw);

            if (j <= i)
                continue;

            const Point p_start = curves[i].point_at(curves[i].domain_start());
            const Point p_end = curves[i].point_at(curves[i].domain_end());
            const double d_a = curve_point(curves[j], p_start).second;
            const double d_b = curve_point(curves[j], p_end).second;

            if (std::min(d_a, d_b) <= threshold)
                pairs.push_back({i, j});
        }
    }

    return pairs;
}

std::vector<std::pair<size_t, size_t>> Closest::boxes_closest(const std::vector<AABB>& boxes, double threshold) {

    std::vector<std::pair<size_t, size_t>> pairs;

    if (threshold < 0.0 || boxes.size() < 2)
        return pairs;

    std::vector<AABB> inflated;
    inflated.reserve(boxes.size());

    for (const AABB& b : boxes) {
        AABB inf = b;
        inf.inflate(threshold);
        inflated.push_back(inf);
    }

    SpatialAABBTree tree;
    tree.build(inflated.data(), inflated.size());

    for (size_t i = 0; i < boxes.size(); i++) {
        for (const int j_raw : tree.query_aabb(inflated[i])) {
            const size_t j = static_cast<size_t>(j_raw);

            if (j <= i)
                continue;

            if (aabb_to_aabb_min_dist(boxes[i], boxes[j]) <= threshold)
                pairs.push_back({i, j});
        }
    }

    return pairs;
}

} // namespace session_cpp
