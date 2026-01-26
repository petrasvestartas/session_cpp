#include "closest.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include <cmath>
#include <algorithm>
#include <limits>

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

    const int num_samples = std::max(10, curve.degree() * 2);
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

    const int max_iterations = 20;
    const double step_tolerance = (t1 - t0) * 1e-10;

    double t = best_t;

    for (int iter = 0; iter < max_iterations; iter++) {
        Point pt = curve.point_at(t);
        Vector tangent = curve.tangent_at(t);

        Vector delta(test_point[0] - pt[0],
                    test_point[1] - pt[1],
                    test_point[2] - pt[2]);

        double f = -delta.dot(tangent);

        if (std::abs(f) < step_tolerance) break;

        auto derivs = curve.evaluate(t, 2);
        if (derivs.size() < 3) break;

        Vector d2(derivs[2][0], derivs[2][1], derivs[2][2]);
        double tangent_mag = tangent.magnitude();
        double df = delta.dot(d2) - tangent_mag * tangent_mag;

        if (std::abs(df) < 1e-12) break;

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

    const int u_samples = std::max(10, surface.order(0));
    const int v_samples = std::max(10, surface.order(1));

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
        Vector du_vec = derivs[1];
        Vector dv_vec = derivs[2];

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

} // namespace session_cpp
