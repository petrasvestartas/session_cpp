#include "closest.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "mesh.h"
#include "pointcloud.h"
#include "spatial_kdtree.h"
#include "spatial_bvh.h"
#include "aabb.h"
#include "spatial_aabbtree.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>

namespace session_cpp {

std::pair<double, double> Closest::curve_point(
    const NurbsCurve& curve,
    const Point& test_point,
    double t0,
    double t1
) {
    if (!curve.is_valid()) return {0.0, std::numeric_limits<double>::infinity()};

    auto [domain_start, domain_end] = curve.domain();
    if (t0 <= 0.0) t0 = domain_start;
    if (t1 <= 0.0) t1 = domain_end;

    t0 = std::max(t0, domain_start);
    t1 = std::min(t1, domain_end);

    // Dense seed grid: sample every knot span several times so the global minimum's
    // basin is captured before Newton refines (matches OCCT's robust initial sampling
    // in GeomAPI_ProjectPointOnCurve).
    const int num_samples = std::max(50, curve.cv_count() * 10);
    double dt = (t1 - t0) / num_samples;

    double best_t = t0;
    double best_dist = curve.point_at(t0).distance(test_point);

    for (int i = 0; i <= num_samples; i++) {
        double t = t0 + i * dt;
        double dist = curve.point_at(t).distance(test_point);
        if (dist < best_dist) {
            best_dist = dist;
            best_t = t;
        }
    }

    const int max_iterations = 32;
    const double step_tolerance = (t1 - t0) * 1e-12;

    double t = best_t;

    // Newton on h(t) = (C(t) - P) . C'(t)  (= 0 at a foot of perpendicular).
    // h'(t) = |C'(t)|^2 + (C(t) - P) . C''(t).  Use the RAW derivatives C', C''.
    for (int iter = 0; iter < max_iterations; iter++) {
        auto derivs = curve.evaluate(t, 2);
        if (derivs.size() < 3) break;
        const Vector& pt = derivs[0];
        const Vector& d1 = derivs[1];
        const Vector& d2 = derivs[2];

        double rx = pt[0] - test_point[0];
        double ry = pt[1] - test_point[1];
        double rz = pt[2] - test_point[2];

        double f = rx * d1[0] + ry * d1[1] + rz * d1[2];

        if (std::abs(f) < step_tolerance) break;

        double df = d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2]
                  + rx * d2[0] + ry * d2[1] + rz * d2[2];

        if (std::abs(df) < 1e-14) break;

        double dt_step = -f / df;

        if (std::abs(dt_step) > (t1 - t0) * 0.5) {
            dt_step = std::copysign((t1 - t0) * 0.5, dt_step);
        }

        t += dt_step;

        if (t < t0) t = t0;
        if (t > t1) t = t1;

        if (std::abs(dt_step) < step_tolerance) break;
    }

    double final_dist = curve.point_at(t).distance(test_point);

    double dist_start = curve.point_at(t0).distance(test_point);
    double dist_end = curve.point_at(t1).distance(test_point);

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

std::tuple<double, double, double> Closest::curve_curve(
    const NurbsCurve& curve0,
    const NurbsCurve& curve1
) {
    if (!curve0.is_valid() || !curve1.is_valid())
        return {0.0, 0.0, std::numeric_limits<double>::infinity()};

    auto [u0, u1] = curve0.domain();
    auto [v0, v1] = curve1.domain();
    int n0 = std::max(40, curve0.cv_count() * 8);
    int n1 = std::max(40, curve1.cv_count() * 8);

    // Dense grid seed.
    std::vector<Point> p0(n0 + 1), p1(n1 + 1);
    for (int i = 0; i <= n0; i++) p0[i] = curve0.point_at(u0 + (u1 - u0) * i / n0);
    for (int j = 0; j <= n1; j++) p1[j] = curve1.point_at(v0 + (v1 - v0) * j / n1);

    double best = std::numeric_limits<double>::infinity();
    double u = u0, v = v0;
    for (int i = 0; i <= n0; i++) {
        for (int j = 0; j <= n1; j++) {
            double dx = p0[i][0] - p1[j][0], dy = p0[i][1] - p1[j][1], dz = p0[i][2] - p1[j][2];
            double d2 = dx*dx + dy*dy + dz*dz;
            if (d2 < best) { best = d2; u = u0 + (u1-u0)*i/n0; v = v0 + (v1-v0)*j/n1; }
        }
    }

    // 2D Newton on f(u,v) = |C0(u) - C1(v)|^2.
    for (int iter = 0; iter < 64; iter++) {
        auto e0 = curve0.evaluate(u, 2);
        auto e1 = curve1.evaluate(v, 2);
        if (e0.size() < 3 || e1.size() < 3) break;
        const Vector& c0 = e0[0]; const Vector& c0p = e0[1]; const Vector& c0pp = e0[2];
        const Vector& c1 = e1[0]; const Vector& c1p = e1[1]; const Vector& c1pp = e1[2];
        double rx = c0[0]-c1[0], ry = c0[1]-c1[1], rz = c0[2]-c1[2];

        double gu =  rx*c0p[0] + ry*c0p[1] + rz*c0p[2];          // 0.5 df/du
        double gv = -(rx*c1p[0] + ry*c1p[1] + rz*c1p[2]);        // 0.5 df/dv

        double huu = c0p[0]*c0p[0] + c0p[1]*c0p[1] + c0p[2]*c0p[2]
                   + rx*c0pp[0] + ry*c0pp[1] + rz*c0pp[2];
        double huv = -(c0p[0]*c1p[0] + c0p[1]*c1p[1] + c0p[2]*c1p[2]);
        double hvv = c1p[0]*c1p[0] + c1p[1]*c1p[1] + c1p[2]*c1p[2]
                   - (rx*c1pp[0] + ry*c1pp[1] + rz*c1pp[2]);

        double det = huu*hvv - huv*huv;
        if (std::abs(det) < 1e-14) break;
        double du = -(hvv*gu - huv*gv) / det;
        double dv = -(-huv*gu + huu*gv) / det;

        if (std::abs(du) > (u1-u0)*0.5) du = std::copysign((u1-u0)*0.5, du);
        if (std::abs(dv) > (v1-v0)*0.5) dv = std::copysign((v1-v0)*0.5, dv);

        u = std::min(std::max(u + du, u0), u1);
        v = std::min(std::max(v + dv, v0), v1);
        if (std::max(std::abs(du), std::abs(dv)) < 1e-13) break;
    }

    double dist = curve0.point_at(u).distance(curve1.point_at(v));
    return {u, v, dist};
}

std::tuple<Point, double, double> Closest::line_point(
    const Line& line,
    const Point& test_point
) {
    Point start = line.start();
    Point end = line.end();

    double dx = end[0] - start[0];
    double dy = end[1] - start[1];
    double dz = end[2] - start[2];

    double len_sq = dx * dx + dy * dy + dz * dz;

    if (len_sq < 1e-20) {
        double dist = start.distance(test_point);
        return std::make_tuple(start, 0.0, dist);
    }

    double t = ((test_point[0] - start[0]) * dx +
                (test_point[1] - start[1]) * dy +
                (test_point[2] - start[2]) * dz) / len_sq;

    t = std::max(0.0, std::min(1.0, t));

    Point closest(
        start[0] + t * dx,
        start[1] + t * dy,
        start[2] + t * dz
    );

    double dist = closest.distance(test_point);

    return std::make_tuple(closest, t, dist);
}

std::tuple<Point, double, double> Closest::polyline_point(
    const Polyline& polyline,
    const Point& test_point
) {
    auto points = polyline.get_points();

    if (points.empty()) {
        return std::make_tuple(Point(0, 0, 0), 0.0, std::numeric_limits<double>::infinity());
    }

    if (points.size() == 1) {
        double dist = points[0].distance(test_point);
        return std::make_tuple(points[0], 0.0, dist);
    }

    Point best_point = points[0];
    double best_param = 0.0;
    double best_dist = std::numeric_limits<double>::infinity();

    double cumulative_length = 0.0;
    double total_length = polyline.length();

    for (size_t i = 0; i < points.size() - 1; i++) {
        Line segment = Line::from_points(points[i], points[i + 1]);
        auto [closest, t, dist] = line_point(segment, test_point);

        if (dist < best_dist) {
            best_dist = dist;
            best_point = closest;
            double segment_length = segment.length();
            if (total_length > 1e-20) {
                best_param = (cumulative_length + t * segment_length) / total_length;
            } else {
                best_param = static_cast<double>(i) / (points.size() - 1);
            }
        }

        cumulative_length += segment.length();
    }

    return std::make_tuple(best_point, best_param, best_dist);
}

std::tuple<double, double, double> Closest::surface_point(
    const NurbsSurface& surface,
    const Point& test_point,
    double u0,
    double u1,
    double v0,
    double v1
) {
    if (!surface.is_valid()) {
        return {0.0, 0.0, std::numeric_limits<double>::infinity()};
    }

    auto [domain_u0, domain_u1] = surface.domain(0);
    auto [domain_v0, domain_v1] = surface.domain(1);

    if (u0 <= 0.0) u0 = domain_u0;
    if (u1 <= 0.0) u1 = domain_u1;
    if (v0 <= 0.0) v0 = domain_v0;
    if (v1 <= 0.0) v1 = domain_v1;

    u0 = std::max(u0, domain_u0);
    u1 = std::min(u1, domain_u1);
    v0 = std::max(v0, domain_v0);
    v1 = std::min(v1, domain_v1);

    // Seed-grid resolution scales with the search-window size relative to the full domain.
    // A full-domain call keeps a dense grid; a small warm-start window (used when projecting
    // successive points of a continuous curve) needs only a few seeds before Newton — this
    // turns surface_curve's per-sample 100-point grid into ~3x3, the boolean SSI hot path.
    const int full_u = std::max(10, surface.order(0));
    const int full_v = std::max(10, surface.order(1));
    double u_frac = (u1 - u0) / std::max(domain_u1 - domain_u0, 1e-12);
    double v_frac = (v1 - v0) / std::max(domain_v1 - domain_v0, 1e-12);
    const int u_samples = std::max(3, (int)std::ceil(full_u * std::min(1.0, u_frac)));
    const int v_samples = std::max(3, (int)std::ceil(full_v * std::min(1.0, v_frac)));

    double du_param = (u1 - u0) / u_samples;
    double dv_param = (v1 - v0) / v_samples;

    double best_u = u0;
    double best_v = v0;
    double best_dist = std::numeric_limits<double>::infinity();

    for (int i = 0; i <= u_samples; i++) {
        for (int j = 0; j <= v_samples; j++) {
            double uu = u0 + i * du_param;
            double vv = v0 + j * dv_param;
            Point pt = surface.point_at(uu, vv);
            double dist = pt.distance(test_point);
            if (dist < best_dist) {
                best_dist = dist;
                best_u = uu;
                best_v = vv;
            }
        }
    }

    const int max_iterations = 20;
    const double step_tolerance = std::min(u1 - u0, v1 - v0) * 1e-10;

    double u = best_u;
    double v = best_v;

    for (int iter = 0; iter < max_iterations; iter++) {
        auto derivs = surface.evaluate(u, v, 1);
        if (derivs.size() < 3) break;

        Point pt = surface.point_at(u, v);
        Vector du_vec = derivs[2];  // evaluate returns [S, Sv, Su, ...]
        Vector dv_vec = derivs[1];

        Vector delta(test_point[0] - pt[0],
                    test_point[1] - pt[1],
                    test_point[2] - pt[2]);

        double fu = -delta.dot(du_vec);
        double fv = -delta.dot(dv_vec);

        if (std::abs(fu) < step_tolerance && std::abs(fv) < step_tolerance) break;

        double duu = du_vec.dot(du_vec);
        double dvv = dv_vec.dot(dv_vec);
        double duv = du_vec.dot(dv_vec);

        double det = duu * dvv - duv * duv;
        if (std::abs(det) < 1e-12) break;

        double du_step = (dvv * fu - duv * fv) / det;
        double dv_step = (duu * fv - duv * fu) / det;

        double max_step = std::min(u1 - u0, v1 - v0) * 0.5;
        if (std::abs(du_step) > max_step) du_step = std::copysign(max_step, du_step);
        if (std::abs(dv_step) > max_step) dv_step = std::copysign(max_step, dv_step);

        u -= du_step;
        v -= dv_step;

        u = std::max(u0, std::min(u1, u));
        v = std::max(v0, std::min(v1, v));

        if (std::abs(du_step) < step_tolerance && std::abs(dv_step) < step_tolerance) break;
    }

    double final_dist = surface.point_at(u, v).distance(test_point);

    return {u, v, final_dist};
}

std::vector<NurbsCurve> Closest::surface_curve(
    const NurbsSurface& surface,
    const NurbsCurve& curve,
    double t0,
    double t1,
    double tolerance
) {
    if (!surface.is_valid() || !curve.is_valid()) return {};

    auto [u0, u1] = surface.domain(0);
    auto [v0, v1] = surface.domain(1);
    double range_u = u1 - u0;
    double range_v = v1 - v0;
    bool closed_u = surface.is_closed(0);
    bool closed_v = surface.is_closed(1);

    auto [ct0, ct1] = curve.domain();
    if (t0 <= 0.0) t0 = ct0;
    if (t1 <= 0.0) t1 = ct1;
    t0 = std::max(t0, ct0);
    t1 = std::min(t1, ct1);
    if (t1 - t0 < 1e-14) return {};

    std::vector<double> spans_u = surface.get_span_vector(0);
    std::vector<double> spans_v = surface.get_span_vector(1);
    int nu = std::max((int)spans_u.size() - 1, 1) * 4;
    int nv = std::max((int)spans_v.size() - 1, 1) * 4;
    double du = range_u / nu;
    double dv = range_v / nv;

    double mu = (u0 + u1) * 0.5;
    double mv = (v0 + v1) * 0.5;
    Point pmid = surface.point_at(mu, mv);
    double wu_probe = std::min(mu + du, u1);
    double wv_probe = std::min(mv + dv, v1);
    double uv_to_3d_u = pmid.distance(surface.point_at(wu_probe, mv)) / du;
    double uv_to_3d_v = pmid.distance(surface.point_at(mu, wv_probe)) / dv;
    double uv_to_3d = std::max(uv_to_3d_u, uv_to_3d_v);
    double uv_to_3d_min = std::min(uv_to_3d_u, uv_to_3d_v);
    if (uv_to_3d < 1e-10) uv_to_3d = 1.0;
    if (uv_to_3d_min < 1e-10) uv_to_3d_min = 1.0;

    double step = std::min(du, dv) * 0.25;
    double fit_tol = tolerance > 0.0 ? tolerance : step * (uv_to_3d + uv_to_3d_min) * 0.5;
    double reject_tol = fit_tol * 100.0;
    // Absolute "lies on the surface" gate (fraction of the surface size).
    // Used to (a) reject a curve that nowhere touches the surface and (b) stop
    // bisecting stick-out portions of a curve that extends past the face, both
    // of which otherwise burn a full 4096-sample bisection.
    double corner_diag = surface.point_at(u0, v0).distance(surface.point_at(u1, v1));
    if (corner_diag < 1e-12) corner_diag = std::max(range_u, range_v);
    double on_surf_tol = corner_diag * 0.05;

    auto wrap_u = [&](double u) -> double {
        if (closed_u) {
            double t = std::fmod(u - u0, range_u);
            if (t < 0) t += range_u;
            return u0 + t;
        }
        return std::max(u0, std::min(u, u1));
    };
    auto wrap_v = [&](double v) -> double {
        if (closed_v) {
            double t = std::fmod(v - v0, range_v);
            if (t < 0) t += range_v;
            return v0 + t;
        }
        return std::max(v0, std::min(v, v1));
    };

    // Windowed inversion with seam-aware candidate windows
    auto invert_near = [&](const Point& pt, double up, double vp, double wu, double wv) -> std::tuple<double, double, double> {
        std::vector<double> u_centers = {up};
        if (closed_u) {
            if (up - wu < u0) u_centers.push_back(up + range_u);
            if (up + wu > u1) u_centers.push_back(up - range_u);
        }
        std::vector<double> v_centers = {vp};
        if (closed_v) {
            if (vp - wv < v0) v_centers.push_back(vp + range_v);
            if (vp + wv > v1) v_centers.push_back(vp - range_v);
        }
        std::tuple<double, double, double> best = {up, vp, std::numeric_limits<double>::infinity()};
        for (double uc : u_centers) {
            for (double vc : v_centers) {
                double wu0 = std::max(uc - wu, u0);
                double wu1 = std::min(uc + wu, u1);
                double wv0 = std::max(vc - wv, v0);
                double wv1 = std::min(vc + wv, v1);
                if (wu1 - wu0 < 1e-14 || wv1 - wv0 < 1e-14) continue;
                auto res = surface_point(surface, pt, wu0, wu1, wv0, wv1);
                if (std::get<2>(res) < std::get<2>(best)) best = res;
                if (std::get<2>(best) < fit_tol * 0.01) break;
            }
        }
        return best;
    };

    auto unwrap_to = [&](double prev_u, double prev_v, double u, double v) -> std::pair<double, double> {
        if (closed_u) {
            while (u - prev_u > range_u * 0.5) u -= range_u;
            while (u - prev_u < -range_u * 0.5) u += range_u;
        }
        if (closed_v) {
            while (v - prev_v > range_v * 0.5) v -= range_v;
            while (v - prev_v < -range_v * 0.5) v += range_v;
        }
        return {u, v};
    };

    // 1. Initial samples with warm-started inversion
    int n0 = std::max(16, 4 * curve.span_count());
    std::vector<std::array<double, 4>> samples;  // [t, u_unwrapped, v_unwrapped, residual]
    double max_residual = 0.0;
    double min_residual = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= n0; i++) {
        double t = t0 + (t1 - t0) * i / n0;
        Point pt = curve.point_at(t);
        double uu, vv, rd;
        if (i == 0) {
            double ru, rv;
            std::tie(ru, rv, rd) = surface_point(surface, pt, 0.0, 0.0, 0.0, 0.0);
            uu = ru;
            vv = rv;
        } else {
            std::array<double, 4> prev = samples.back();
            std::array<double, 4> prev2 = samples[std::max(0, (int)samples.size() - 2)];
            double wu = std::max(du, dv) * 2.0 + std::abs(prev[1] - prev2[1]);
            double wv = std::max(du, dv) * 2.0 + std::abs(prev[2] - prev2[2]);
            double ru, rv;
            std::tie(ru, rv, rd) = invert_near(pt, wrap_u(prev[1]), wrap_v(prev[2]), wu, wv);
            if (rd > reject_tol) {
                std::tie(ru, rv, rd) = surface_point(surface, pt, 0.0, 0.0, 0.0, 0.0);
            }
            std::tie(uu, vv) = unwrap_to(prev[1], prev[2], ru, rv);
        }
        samples.push_back(std::array<double, 4>{t, uu, vv, rd});
        max_residual = std::max(max_residual, rd);
        min_residual = std::min(min_residual, rd);
    }

    // Reject a curve that nowhere lies on the surface (no sample touches it).
    if (max_residual > reject_tol || min_residual > on_surf_tol) return {};

    // 2. Adaptive bisection where the lifted UV midpoint strays from the curve
    int depth = 0;
    while (depth < 8) {
        int inserted = 0;
        size_t i = 0;
        while (i + 1 < samples.size()) {
            std::array<double, 4> a = samples[i];
            std::array<double, 4> b = samples[i + 1];
            double tm = (a[0] + b[0]) * 0.5;
            double um = (a[1] + b[1]) * 0.5;
            double vm = (a[2] + b[2]) * 0.5;
            Point pm = curve.point_at(tm);
            Point lift = surface.point_at(wrap_u(um), wrap_v(vm));
            if (lift.distance(pm) > fit_tol && samples.size() < 4096) {
                double wu = std::max(std::abs(b[1] - a[1]), du) * 1.0;
                double wv = std::max(std::abs(b[2] - a[2]), dv) * 1.0;
                double ru, rv, rd;
                std::tie(ru, rv, rd) = invert_near(pm, wrap_u(um), wrap_v(vm), wu, wv);
                if (rd > on_surf_tol) {
                    // Midpoint is off the surface: stick-out portion of a curve
                    // that extends past the face, not a curvature stray. Do not
                    // refine it (avoids unbounded bisection).
                    i += 1;
                    continue;
                }
                auto [uu, vv] = unwrap_to(a[1], a[2], ru, rv);
                samples.insert(samples.begin() + i + 1, std::array<double, 4>{tm, uu, vv, rd});
                inserted += 1;
                i += 2;
            } else {
                i += 1;
            }
        }
        if (inserted == 0) break;
        depth += 1;
    }

    std::vector<std::pair<double, double>> pts;
    pts.reserve(samples.size());
    for (const auto& s : samples) pts.push_back({s[1], s[2]});

    // 3. Convert the continuous unwrapped UV polyline into in-domain pieces, split at periodic
    //    seam lines (u = u0 + k*range_u, v = v0 + k*range_v). Each emitted piece lies within
    //    [u0,u1]x[v0,v1]; a piece is marked closed only if it is a true interior loop (no seam
    //    crossing and not spanning a full period). A loop that starts off a seam is rotated to
    //    begin at its first seam crossing so its arcs split cleanly. This replaces the previous
    //    wrap-piece scheme, which produced spurious degenerate slivers on the sphere/cylinder.
    Point p_first = curve.point_at(samples[0][0]);
    Point p_last = curve.point_at(samples.back()[0]);
    bool is_loop = p_first.distance(p_last) < fit_tol * 4.0 && pts.size() >= 6;
    if (is_loop && pts.size() >= 2) pts.pop_back();  // drop duplicate closing sample

    // Net unwrapped winding over a loop: ~0 = the loop straddles the seam (its arbitrary start
    // splits one arc), ~+/-range = the loop encircles the surface (the closure itself crosses a
    // seam, so the first and last arcs are genuinely separate).
    double wind_u = closed_u ? (samples.back()[1] - samples.front()[1]) : 0.0;
    double wind_v = closed_v ? (samples.back()[2] - samples.front()[2]) : 0.0;
    bool closure_crosses_seam = std::abs(wind_u) > range_u * 0.5 || std::abs(wind_v) > range_v * 0.5;

    // First seam crossing in segment (a -> b): the smallest t in (0,1) where u or v hits a seam.
    auto first_seam = [&](const std::pair<double,double>& a, const std::pair<double,double>& b,
                          double& cu, double& cv) -> bool {
        double bestt = 2.0; bool found = false;
        if (closed_u && std::abs(b.first - a.first) > 1e-15) {
            int k0 = (int)std::floor((a.first - u0)/range_u), k1 = (int)std::floor((b.first - u0)/range_u);
            for (int k = std::min(k0,k1)+1; k <= std::max(k0,k1); ++k) {
                double L = u0 + k*range_u, t = (L - a.first)/(b.first - a.first);
                if (t > 1e-9 && t < 1.0 - 1e-9 && t < bestt) { bestt = t; found = true; cu = L; cv = a.second + (b.second-a.second)*t; }
            }
        }
        if (closed_v && std::abs(b.second - a.second) > 1e-15) {
            int k0 = (int)std::floor((a.second - v0)/range_v), k1 = (int)std::floor((b.second - v0)/range_v);
            for (int k = std::min(k0,k1)+1; k <= std::max(k0,k1); ++k) {
                double L = v0 + k*range_v, t = (L - a.second)/(b.second - a.second);
                if (t > 1e-9 && t < 1.0 - 1e-9 && t < bestt) { bestt = t; found = true; cv = L; cu = a.first + (b.first-a.first)*t; }
            }
        }
        return found;
    };

    // Walk the continuous polyline, split at every interior seam crossing.
    std::vector<std::pair<std::vector<std::pair<double, double>>, bool>> pieces;
    {
        std::vector<std::vector<std::pair<double, double>>> raw;
        std::vector<std::pair<double, double>> cur;
        cur.push_back(pts[0]);
        bool any_cross = false;
        auto on_seam = [&](const std::pair<double,double>& p) -> bool {
            if (closed_u) { double L = u0 + std::round((p.first - u0)/range_u)*range_u;
                            if (std::abs(p.first - L) < range_u*1e-6) return true; }
            if (closed_v) { double L = v0 + std::round((p.second - v0)/range_v)*range_v;
                            if (std::abs(p.second - L) < range_v*1e-6) return true; }
            return false;
        };
        for (size_t i = 1; i < pts.size(); ++i) {
            std::pair<double,double> a = pts[i-1], b = pts[i];
            while (true) {
                double cu, cv;
                if (!first_seam(a, b, cu, cv)) break;
                cur.push_back({cu, cv});
                raw.push_back(cur);
                cur.clear();
                cur.push_back({cu, cv});
                any_cross = true;
                a = {cu, cv};  // continue scanning the remainder for further seams
            }
            cur.push_back(b);
            // An interior sample landing exactly on a seam line is itself a crossing.
            if (i + 1 < pts.size() && on_seam(b)) {
                raw.push_back(cur);
                cur.clear();
                cur.push_back(b);
                any_cross = true;
            }
        }
        raw.push_back(cur);
        // A straddling loop's first and last raw segments are the two halves of one arc split by
        // the arbitrary loop start; rejoin them (last + first) into a single continuous arc.
        // Only when the start is genuinely mid-arc -- if it already sits on a seam, the first and
        // last segments are distinct arcs meeting there and must stay separate.
        if (is_loop && !closure_crosses_seam && raw.size() > 1 && !on_seam(pts[0])) {
            std::vector<std::pair<double,double>> merged = raw.back();
            for (size_t k = 1; k < raw.front().size(); ++k) merged.push_back(raw.front()[k]);
            raw.erase(raw.begin());
            raw.back() = merged;
        }
        for (auto& seg : raw) {
            if (seg.size() < 2) continue;
            std::pair<double,double> mid = seg[seg.size()/2];
            if (closed_u) { int k = (int)std::floor((mid.first - u0)/range_u); if (k) for (auto& p : seg) p.first -= k*range_u; }
            if (closed_v) { int k = (int)std::floor((mid.second - v0)/range_v); if (k) for (auto& p : seg) p.second -= k*range_v; }
            double umin=1e300,umax=-1e300,vmin=1e300,vmax=-1e300,len=0;
            for (auto& p : seg) { umin=std::min(umin,p.first); umax=std::max(umax,p.first); vmin=std::min(vmin,p.second); vmax=std::max(vmax,p.second); }
            for (size_t i = 1; i < seg.size(); ++i) len += std::hypot(seg[i].first-seg[i-1].first, seg[i].second-seg[i-1].second);
            if (len < std::min(range_u, range_v) * 1e-4) continue;  // degenerate sliver
            bool seg_loop = is_loop && !any_cross && (umax-umin < range_u*0.9) && (vmax-vmin < range_v*0.9);
            pieces.push_back({seg, seg_loop});
        }
    }

    // 4. Refit each piece as a UV pcurve
    std::vector<NurbsCurve> result;
    for (auto& [piece_pts, piece_loop] : pieces) {
        if (piece_pts.size() < 2) continue;
        std::pair<double, double> mid = piece_pts[piece_pts.size() / 2];
        if (closed_u) {
            int k_u = (int)std::floor((mid.first - u0) / range_u);
            if (k_u != 0)
                for (auto& p : piece_pts) p.first -= k_u * range_u;
        }
        if (closed_v) {
            int k_v = (int)std::floor((mid.second - v0) / range_v);
            if (k_v != 0)
                for (auto& p : piece_pts) p.second -= k_v * range_v;
        }

        std::vector<Point> pts_uv(piece_pts.size());
        for (size_t i = 0; i < piece_pts.size(); i++)
            pts_uv[i] = Point(piece_pts[i].first, piece_pts[i].second, 0.0);
        int mp = (int)pts_uv.size();
        double fit_tol_uv = step;
        double total_turning = 0.0;
        for (int i = 1; i < mp - 1; i++) {
            double dx1 = pts_uv[i][0] - pts_uv[i-1][0];
            double dy1 = pts_uv[i][1] - pts_uv[i-1][1];
            double dx2 = pts_uv[i+1][0] - pts_uv[i][0];
            double dy2 = pts_uv[i+1][1] - pts_uv[i][1];
            double l1 = std::hypot(dx1, dy1);
            double l2 = std::hypot(dx2, dy2);
            if (l1 > 1e-14 && l2 > 1e-14) {
                double c = (dx1*dx2 + dy1*dy2) / (l1*l2);
                c = std::max(-1.0, std::min(1.0, c));
                total_turning += std::acos(c);
            }
        }

        std::vector<double> chords(mp, 0.0);
        double total_len = 0.0;
        for (int i = 1; i < mp; i++) {
            total_len += pts_uv[i].distance(pts_uv[i-1]);
            chords[i] = total_len;
        }
        if (piece_loop && mp > 1) total_len += pts_uv[0].distance(pts_uv[mp-1]);
        if (total_len > 1e-14)
            for (int i = 1; i < mp; i++) chords[i] /= total_len;

        int target_cvs = std::max(8, (int)(total_turning / 0.5) + 6);
        int max_cvs = mp - 1;
        NurbsCurve pcurve;
        for (int attempt = 0; attempt < 5; attempt++) {
            if (target_cvs > max_cvs) break;
            pcurve = NurbsCurve::create_fitted(pts_uv, target_cvs, 3, piece_loop);
            if (!pcurve.is_valid()) break;
            auto [ft0, ft1] = pcurve.domain();
            double max_dev = 0.0;
            for (int i = 0; i < mp; i++) {
                double t = ft0 + (ft1 - ft0) * chords[i];
                max_dev = std::max(max_dev, pcurve.point_at(t).distance(pts_uv[i]));
            }
            if (max_dev < fit_tol_uv) break;
            target_cvs = std::min(target_cvs * 2, max_cvs);
        }

        if (!pcurve.is_valid())
            pcurve = piece_loop
                ? NurbsCurve::create_interpolated(pts_uv, CurveNurbsKnotStyle::ChordPeriodic)
                : NurbsCurve::create_interpolated(pts_uv);
        if (!pcurve.is_valid() && pts_uv.size() >= 2)
            // Last resort: a degree-1 polyline through the inverted UV samples
            // (always valid; lies on the surface piecewise-linearly in UV).
            pcurve = NurbsCurve::create(false, 1, pts_uv);
        if (!pcurve.is_valid()) continue;

        pcurve.set_domain(0.0, 1.0);
        result.push_back(pcurve);
    }

    return result;
}

static Point closest_point_on_triangle(const Point& p, const Point& a, const Point& b, const Point& c) {
    double abx = b[0]-a[0], aby = b[1]-a[1], abz = b[2]-a[2];
    double acx = c[0]-a[0], acy = c[1]-a[1], acz = c[2]-a[2];
    double apx = p[0]-a[0], apy = p[1]-a[1], apz = p[2]-a[2];

    double d1 = abx*apx + aby*apy + abz*apz;
    double d2 = acx*apx + acy*apy + acz*apz;
    if (d1 <= 0.0 && d2 <= 0.0) return a;

    double bpx = p[0]-b[0], bpy = p[1]-b[1], bpz = p[2]-b[2];
    double d3 = abx*bpx + aby*bpy + abz*bpz;
    double d4 = acx*bpx + acy*bpy + acz*bpz;
    if (d3 >= 0.0 && d4 <= d3) return b;

    double vc = d1*d4 - d3*d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        double v = d1 / (d1 - d3);
        return Point(a[0] + v*abx, a[1] + v*aby, a[2] + v*abz);
    }

    double cpx = p[0]-c[0], cpy = p[1]-c[1], cpz = p[2]-c[2];
    double d5 = abx*cpx + aby*cpy + abz*cpz;
    double d6 = acx*cpx + acy*cpy + acz*cpz;
    if (d6 >= 0.0 && d5 <= d6) return c;

    double vb = d5*d2 - d1*d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        double w = d2 / (d2 - d6);
        return Point(a[0] + w*acx, a[1] + w*acy, a[2] + w*acz);
    }

    double va = d3*d6 - d5*d4;
    if (va <= 0.0 && (d4-d3) >= 0.0 && (d5-d6) >= 0.0) {
        double w = (d4-d3) / ((d4-d3) + (d5-d6));
        return Point(b[0] + w*(c[0]-b[0]), b[1] + w*(c[1]-b[1]), b[2] + w*(c[2]-b[2]));
    }

    double denom = 1.0 / (va + vb + vc);
    double v = vb * denom;
    double w = vc * denom;
    return Point(a[0] + abx*v + acx*w, a[1] + aby*v + acy*w, a[2] + abz*v + acz*w);
}

static double aabb_min_distance(const AABB& aabb, const Point& p) {
    double dx = std::max(0.0, std::max(aabb.cx - aabb.hx - p[0], p[0] - aabb.cx - aabb.hx));
    double dy = std::max(0.0, std::max(aabb.cy - aabb.hy - p[1], p[1] - aabb.cy - aabb.hy));
    double dz = std::max(0.0, std::max(aabb.cz - aabb.hz - p[2], p[2] - aabb.cz - aabb.hz));
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

std::tuple<Point, size_t, double> Closest::mesh_point(
    const Mesh& mesh,
    const Point& test_point
) {
    if (mesh.number_of_faces() == 0) {
        return {Point(0, 0, 0), 0, std::numeric_limits<double>::infinity()};
    }

    mesh.build_triangle_bvh();
    const SpatialBVH* bvh = mesh.get_cached_bvh();

    std::vector<size_t> face_keys;
    face_keys.reserve(mesh.face.size());
    for (const auto& [key, _] : mesh.face) face_keys.push_back(key);

    Point best_point(0, 0, 0);
    size_t best_face_key = 0;
    double best_dist = std::numeric_limits<double>::infinity();

    if (!bvh || !bvh->root) {
        return {best_point, best_face_key, best_dist};
    }

    using PQEntry = std::pair<double, const SpatialBVHNode*>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;
    pq.push({aabb_min_distance(bvh->root->aabb, test_point), bvh->root});

    while (!pq.empty()) {
        auto [d, node] = pq.top();
        pq.pop();
        if (d >= best_dist) break;

        if (node->is_leaf()) {
            Point v0, v1, v2;
            size_t face_idx, sub_idx;
            if (mesh.get_triangle_by_id(node->object_id, face_idx, sub_idx, v0, v1, v2)) {
                Point cp = closest_point_on_triangle(test_point, v0, v1, v2);
                double dist = cp.distance(test_point);
                if (dist < best_dist) {
                    best_dist = dist;
                    best_point = cp;
                    best_face_key = face_keys[face_idx];
                }
            }
        } else {
            if (node->left) {
                double ld = aabb_min_distance(node->left->aabb, test_point);
                if (ld < best_dist) pq.push({ld, node->left});
            }
            if (node->right) {
                double rd = aabb_min_distance(node->right->aabb, test_point);
                if (rd < best_dist) pq.push({rd, node->right});
            }
        }
    }

    return {best_point, best_face_key, best_dist};
}

static void aabb_dfs_closest(
    const SpatialAABBTree& tree, int ni, const Point& tp,
    const Mesh& mesh, const std::vector<size_t>& fkeys,
    Point& bp, size_t& bk, double& bd
) {
    const auto& n = tree.nodes[ni];
    if (aabb_min_distance(n.aabb, tp) >= bd) return;

    if (n.object_id >= 0) {
        Point v0, v1, v2;
        size_t fi, si;
        if (mesh.get_triangle_by_id(n.object_id, fi, si, v0, v1, v2)) {
            Point cp = closest_point_on_triangle(tp, v0, v1, v2);
            double d = cp.distance(tp);
            if (d < bd) { bd = d; bp = cp; bk = fkeys[fi]; }
        }
        return;
    }

    int left = ni + 1;
    int right = n.right;
    double ld = aabb_min_distance(tree.nodes[left].aabb, tp);
    double rd = aabb_min_distance(tree.nodes[right].aabb, tp);

    if (ld <= rd) {
        if (ld < bd) aabb_dfs_closest(tree, left, tp, mesh, fkeys, bp, bk, bd);
        if (rd < bd) aabb_dfs_closest(tree, right, tp, mesh, fkeys, bp, bk, bd);
    } else {
        if (rd < bd) aabb_dfs_closest(tree, right, tp, mesh, fkeys, bp, bk, bd);
        if (ld < bd) aabb_dfs_closest(tree, left, tp, mesh, fkeys, bp, bk, bd);
    }
}

std::tuple<Point, size_t, double> Closest::mesh_point_aabb(
    const Mesh& mesh,
    const Point& test_point
) {
    if (mesh.number_of_faces() == 0) {
        return {Point(0, 0, 0), 0, std::numeric_limits<double>::infinity()};
    }

    mesh.build_triangle_aabb_tree();
    const SpatialAABBTree* tree = mesh.get_cached_aabb_tree();

    std::vector<size_t> face_keys;
    face_keys.reserve(mesh.face.size());
    for (const auto& [key, _] : mesh.face) face_keys.push_back(key);

    Point best_point(0, 0, 0);
    size_t best_face_key = 0;
    double best_dist = std::numeric_limits<double>::infinity();

    if (!tree || tree->empty()) {
        return {best_point, best_face_key, best_dist};
    }

    aabb_dfs_closest(*tree, 0, test_point, mesh, face_keys, best_point, best_face_key, best_dist);

    return {best_point, best_face_key, best_dist};
}

std::tuple<Point, size_t, double> Closest::pointcloud_point(
    const PointCloud& cloud,
    const Point& test_point
) {
    if (cloud.point_count() == 0) {
        return {Point(0, 0, 0), 0, std::numeric_limits<double>::infinity()};
    }

    Point best_point = cloud.get_point(0);
    size_t best_index = 0;
    double best_dist = best_point.distance(test_point);

    for (size_t i = 1; i < cloud.point_count(); i++) {
        Point p = cloud.get_point(i);
        double dist = p.distance(test_point);
        if (dist < best_dist) {
            best_dist = dist;
            best_point = p;
            best_index = i;
        }
    }

    return {best_point, best_index, best_dist};
}

static double aabb_to_aabb_min_dist(const AABB& a, const AABB& b) {
    double dx = std::max(0.0, std::abs(a.cx - b.cx) - a.hx - b.hx);
    double dy = std::max(0.0, std::abs(a.cy - b.cy) - a.hy - b.hy);
    double dz = std::max(0.0, std::abs(a.cz - b.cz) - a.hz - b.hz);
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

static void collection_aabb_dfs(
    const SpatialAABBTree& tree, int ni, const AABB& query_aabb,
    std::vector<int>& result
) {
    const auto& n = tree.nodes[ni];
    if (!n.aabb.intersects(query_aabb)) return;
    if (n.object_id >= 0) {
        result.push_back(n.object_id);
        return;
    }
    collection_aabb_dfs(tree, ni + 1, query_aabb, result);
    collection_aabb_dfs(tree, n.right, query_aabb, result);
}

std::tuple<Point, size_t, double> Closest::pointcloud_point_kdtree(
    const PointCloud& cloud,
    const Point& test_point
) {
    if (cloud.point_count() == 0) {
        return {Point(0, 0, 0), 0, std::numeric_limits<double>::infinity()};
    }
    std::vector<Point> pts;
    pts.reserve(cloud.point_count());
    for (size_t i = 0; i < cloud.point_count(); i++) pts.push_back(cloud.get_point(i));
    SpatialKDTree kd(std::move(pts));
    auto [idx, dist] = kd.nearest(test_point);
    return {cloud.get_point(idx), static_cast<size_t>(idx), dist};
}

std::vector<std::pair<size_t, size_t>> Closest::lines_closest(
    const std::vector<Line>& lines,
    double threshold
) {
    std::vector<std::pair<size_t, size_t>> result;
    if (lines.size() < 2) return result;

    std::vector<AABB> aabbs;
    aabbs.reserve(lines.size());
    for (const auto& ln : lines) aabbs.push_back(AABB::from_line(ln, threshold));

    SpatialAABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    for (size_t i = 0; i < lines.size(); i++) {
        std::vector<int> candidates;
        collection_aabb_dfs(tree, 0, aabbs[i], candidates);
        for (int j_raw : candidates) {
            size_t j = static_cast<size_t>(j_raw);
            if (j <= i) continue;
            auto [cp_a, t_a, d_a] = line_point(lines[j], lines[i].start());
            auto [cp_b, t_b, d_b] = line_point(lines[j], lines[i].end());
            auto [cp_c, t_c, d_c] = line_point(lines[i], lines[j].start());
            auto [cp_d, t_d, d_d] = line_point(lines[i], lines[j].end());
            double dist = std::min({d_a, d_b, d_c, d_d});
            if (dist <= threshold) result.push_back({i, j});
        }
    }
    return result;
}

std::vector<std::pair<size_t, size_t>> Closest::polylines_closest(
    const std::vector<Polyline>& polylines,
    double threshold
) {
    std::vector<std::pair<size_t, size_t>> result;
    if (polylines.size() < 2) return result;

    std::vector<AABB> aabbs;
    aabbs.reserve(polylines.size());
    for (const auto& pl : polylines) aabbs.push_back(AABB::from_polyline(pl, threshold));

    SpatialAABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    for (size_t i = 0; i < polylines.size(); i++) {
        std::vector<int> candidates;
        collection_aabb_dfs(tree, 0, aabbs[i], candidates);
        for (int j_raw : candidates) {
            size_t j = static_cast<size_t>(j_raw);
            if (j <= i) continue;
            const auto pts_a = polylines[i].get_points();
            double dist = std::numeric_limits<double>::infinity();
            for (const auto& pt : pts_a) {
                auto [cp, t, d] = polyline_point(polylines[j], pt);
                if (d < dist) dist = d;
            }
            if (dist <= threshold) result.push_back({i, j});
        }
    }
    return result;
}

std::vector<std::pair<size_t, size_t>> Closest::nurbscurves_closest(
    const std::vector<NurbsCurve>& curves,
    double threshold
) {
    std::vector<std::pair<size_t, size_t>> result;
    if (curves.size() < 2) return result;

    std::vector<AABB> aabbs;
    aabbs.reserve(curves.size());
    for (const auto& crv : curves) aabbs.push_back(AABB::from_nurbscurve(crv, threshold, false));

    SpatialAABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    for (size_t i = 0; i < curves.size(); i++) {
        std::vector<int> candidates;
        collection_aabb_dfs(tree, 0, aabbs[i], candidates);
        for (int j_raw : candidates) {
            size_t j = static_cast<size_t>(j_raw);
            if (j <= i) continue;
            auto [domain_s, domain_e] = curves[i].domain();
            Point p_start = curves[i].point_at(domain_s);
            Point p_end = curves[i].point_at(domain_e);
            auto [t_a, d_a] = curve_point(curves[j], p_start);
            auto [t_b, d_b] = curve_point(curves[j], p_end);
            double dist = std::min(d_a, d_b);
            if (dist <= threshold) result.push_back({i, j});
        }
    }
    return result;
}

std::vector<std::pair<size_t, size_t>> Closest::boxes_closest(
    const std::vector<AABB>& boxes,
    double threshold
) {
    std::vector<std::pair<size_t, size_t>> result;
    if (boxes.size() < 2) return result;

    std::vector<AABB> inflated;
    inflated.reserve(boxes.size());
    for (const auto& b : boxes) {
        AABB inf = b;
        inf.inflate(threshold);
        inflated.push_back(inf);
    }

    SpatialAABBTree tree;
    tree.build(inflated.data(), inflated.size());

    for (size_t i = 0; i < boxes.size(); i++) {
        std::vector<int> candidates;
        collection_aabb_dfs(tree, 0, inflated[i], candidates);
        for (int j_raw : candidates) {
            size_t j = static_cast<size_t>(j_raw);
            if (j <= i) continue;
            double dist = aabb_to_aabb_min_dist(boxes[i], boxes[j]);
            if (dist <= threshold) result.push_back({i, j});
        }
    }
    return result;
}

} // namespace session_cpp
