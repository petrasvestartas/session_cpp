#include "mini_test.h"
#include "intersection.h"
#include "nurbssurface.h"
#include "primitives.h"
#include "xform.h"
#include "tolerance.h"
#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

static double lies_on_curve(const NurbsCurve& curve3d, const NurbsCurve& pcurve, const NurbsSurface& surface) {

    const std::pair<double, double> domain_u = surface.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = surface.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    std::vector<Point> dense;

    for (int j = 0; j < 129; j++)
        dense.push_back(curve3d.point_at(j / 128.0));

    double worst = 0.0;

    for (int i = 0; i < 33; i++) {
        Point q = pcurve.point_at(i / 32.0);
        Point s = surface.point_at(std::min(std::max(q[0], u0), u1), std::min(std::max(q[1], v0), v1));
        double best = dense[0].distance(s);

        for (const Point& p : dense)
            best = std::min(best, p.distance(s));

        worst = std::max(worst, best);
    }

    return worst;
}

static double on_both(const NurbsCurve& c3, double (*da)(const Point&), double (*db)(const Point&)) {

    double worst = 0.0;

    for (int i = 0; i <= 64; i++) {
        Point p = c3.point_at(i / 64.0);
        worst = std::max(worst, std::max(da(p), db(p)));
    }

    return worst;
}

static double distance_sphere(const Point& p) {
    return std::abs(std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]) - 2.0);
}

static double distance_cylinder(const Point& p) {
    return std::abs(std::sqrt((p[0] - 1.3) * (p[0] - 1.3) + p[1] * p[1]) - 0.3);
}

static double distance_sphere2(const Point& p) {
    return std::abs(std::sqrt((p[0] - 2.0) * (p[0] - 2.0) + p[1] * p[1] + p[2] * p[2]) - 2.0);
}

static double distance_torus(const Point& p) {

    double ring = std::sqrt(p[0] * p[0] + p[1] * p[1]) - 2.0;

    return std::abs(std::sqrt(ring * ring + p[2] * p[2]) - 0.5);
}

static double distance_flat(const Point& p) {
    return std::abs(p[2]);
}

static NurbsSurface bilinear(const Point& p00, const Point& p01, const Point& p10, const Point& p11) {
    return NurbsSurface::create(false, false, 1, 1, 2, 2, {p00, p01, p10, p11});
}

static double lifted_distance(const NurbsCurve& pcurve, const NurbsSurface& surface, double (*d)(const Point&)) {

    const std::pair<double, double> domain = pcurve.domain();
    double worst = 0.0;

    for (int i = 0; i <= 32; i++) {
        Point uv = pcurve.point_at(domain.first + (domain.second - domain.first) * i / 32.0);
        worst = std::max(worst, d(surface.point_at(uv[0], uv[1])));
    }

    return worst;
}

static double distance_cone(const Point& p) {
    return std::abs(std::sqrt(p[0] * p[0] + p[1] * p[1]) - (3.0 - p[2]) * 0.5);
}

static double distance_flat_half(const Point& p) {
    return std::abs(p[2] - 0.5);
}

static double distance_wall(const Point& p) {
    return std::abs(p[0] - 0.2);
}

static double distance_unit_cylinder(const Point& p) {
    return std::abs(std::sqrt(p[0] * p[0] + p[1] * p[1]) - 1.0);
}

static double distance_x_cylinder(const Point& p) {
    return std::abs(std::sqrt(p[1] * p[1] + p[2] * p[2]) - 1.0);
}

static double distance_wide_cylinder(const Point& p) {
    return std::abs(std::sqrt(p[0] * p[0] + p[1] * p[1]) - 2.2);
}

static double distance_high_torus(const Point& p) {

    double ring = std::sqrt(p[0] * p[0] + p[1] * p[1]) - 1.0;

    return std::abs(std::sqrt(ring * ring + (p[2] - 1.0) * (p[2] - 1.0)) - 0.3);
}

static double distance_wide_torus(const Point& p) {

    double ring = std::sqrt(p[0] * p[0] + p[1] * p[1]) - 2.3;

    return std::abs(std::sqrt(ring * ring + (p[2] - 0.3) * (p[2] - 0.3)) - 0.5);
}

static double distance_square(const Point& p) {
    return std::max(std::abs(p[2] - 0.5), std::max(std::max(0.0, std::abs(p[0]) - 1.6), std::max(0.0, std::abs(p[1]) - 1.6)));
}

static double distance_slanted(const Point& p) {
    return std::abs(-2.0 * p[0] + p[1] + 10.0 * p[2] - 3.0) / std::sqrt(105.0);
}

static double pcurve_end_gap(const NurbsCurve& curve3d, const NurbsCurve& pcurve, const NurbsSurface& surface) {

    const Point uv0 = pcurve.point_at_start();
    const Point uv1 = pcurve.point_at_end();

    return std::max(surface.point_at(uv0[0], uv0[1]).distance(curve3d.point_at_start()), surface.point_at(uv1[0], uv1[1]).distance(curve3d.point_at_end()));
}

static double pcurve_line_deviation(const NurbsCurve& line, const NurbsCurve& pcurve, const NurbsSurface& surface) {

    const std::pair<double, double> domain = pcurve.domain();
    const Point a = line.point_at_start();
    const Vector d = line.point_at_end() - a;
    double worst = pcurve_end_gap(line, pcurve, surface);

    for (int i = 0; i <= 32; i++) {
        for (double f : {i / 640.0, 1.0 - i / 640.0}) {
            Point uv = pcurve.point_at(domain.first + (domain.second - domain.first) * f);
            worst = std::max(worst, (surface.point_at(uv[0], uv[1]) - a).cross(d).magnitude() / d.magnitude());
        }
    }

    return worst;
}

MINI_TEST("Intersection", "Line Line") {
    // using session_cpp::Intersection;
    // using session_cpp::Line;
    // using session_cpp::Tolerance;
    // using session_cpp::Point;

    Line line0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line line1(0.5, -1.0, 0.0, 0.5, 1.0, 0.0);

    Point output;
    bool result = Intersection::line_line(line0, line1, output, Tolerance::APPROXIMATION);

    MINI_CHECK(result);
    MINI_CHECK(TOLERANCE.is_close(output[0], 0.5));
    MINI_CHECK(TOLERANCE.is_close(output[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(output[2], 0.0));
}

MINI_TEST("Intersection", "Line Line Parallel") {
    // using session_cpp::Intersection;
    // using session_cpp::Line;
    // using session_cpp::Tolerance;
    // using session_cpp::Point;

    Line line0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line line1(0.0, 1.0, 0.0, 1.0, 1.0, 0.0);

    Point output;
    bool result = Intersection::line_line(line0, line1, output, Tolerance::APPROXIMATION);

    MINI_CHECK(!result);
}

MINI_TEST("Intersection", "Line Line Parameters") {
    // using session_cpp::Intersection;
    // using session_cpp::Line;
    // using session_cpp::Tolerance;

    Line line0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line line1(0.5, -1.0, 0.0, 0.5, 1.0, 0.0);

    double t0;
    double t1;
    bool result = Intersection::line_line_parameters(line0, line1, t0, t1, Tolerance::APPROXIMATION);

    MINI_CHECK(result);
    MINI_CHECK(TOLERANCE.is_close(t0, 0.5));
    MINI_CHECK(TOLERANCE.is_close(t1, 0.5));
}

MINI_TEST("Intersection", "Line Line Parameters Endpoints") {
    // using session_cpp::Intersection;
    // using session_cpp::Line;
    // using session_cpp::Tolerance;

    Line line0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line line1(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    double t0;
    double t1;
    bool result = Intersection::line_line_parameters(line0, line1, t0, t1, Tolerance::APPROXIMATION);

    MINI_CHECK(result);
    MINI_CHECK(TOLERANCE.is_close(t0, 0.0));
    MINI_CHECK(TOLERANCE.is_close(t1, 0.0));
}

MINI_TEST("Intersection", "Line Line Parameters Infinite") {
    // using session_cpp::Intersection;
    // using session_cpp::Line;
    // using session_cpp::Tolerance;

    Line line0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line line1(2.0, -1.0, 0.0, 2.0, 1.0, 0.0);

    double t0;
    double t1;
    bool result =
        Intersection::line_line_parameters(line0, line1, t0, t1, static_cast<double>(Tolerance::APPROXIMATION), false);

    MINI_CHECK(result);
    MINI_CHECK(TOLERANCE.is_close(t0, 2.0));
}

MINI_TEST("Intersection", "Plane Plane") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;
    // using session_cpp::Line;

    Point p0(0.0, 0.0, 0.0);
    Vector n0(0.0, 0.0, 1.0);
    Plane plane0 = Plane::from_point_normal(p0, n0);

    Point p1(0.0, 0.0, 0.0);
    Vector n1(0.0, 1.0, 0.0);
    Plane plane1 = Plane::from_point_normal(p1, n1);

    Line output;
    bool result = Intersection::plane_plane(plane0, plane1, output);

    MINI_CHECK(result);

    Vector line_dir = output.to_vector();

    MINI_CHECK(std::fabs(std::fabs(line_dir[0]) - 1.0) < 1e-4);
    MINI_CHECK(std::fabs(line_dir[1]) < 1e-4);
    MINI_CHECK(std::fabs(line_dir[2]) < 1e-4);
}

MINI_TEST("Intersection", "Plane Plane Complex") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;
    // using session_cpp::Line;

    Point plane_origin_0(213.787107, 513.797811, -24.743845);
    Vector plane_xaxis_0(0.907673, -0.258819, 0.330366);
    Vector plane_yaxis_0(0.272094, 0.96225, 0.006285);
    Plane pl0(plane_origin_0, plane_xaxis_0, plane_yaxis_0);

    Point plane_origin_1(247.17924, 499.115486, 59.619568);
    Vector plane_xaxis_1(0.552465, 0.816035, 0.16991);
    Vector plane_yaxis_1(0.172987, 0.087156, -0.98106);
    Plane pl1(plane_origin_1, plane_xaxis_1, plane_yaxis_1);

    Line intersection_line;
    bool result = Intersection::plane_plane(pl0, pl1, intersection_line);

    MINI_CHECK(result);

    Point start = intersection_line.start();
    Point end = intersection_line.end();

    MINI_CHECK(std::fabs(start[0] - 252.4632) < 0.01);
    MINI_CHECK(std::fabs(start[1] - 495.32248) < 0.01);
    MINI_CHECK(std::fabs(start[2] - (-10.002656)) < 0.01);

    MINI_CHECK(std::fabs(end[0] - 253.01033) < 0.01);
    MINI_CHECK(std::fabs(end[1] - 496.1218) < 0.01);
    MINI_CHECK(std::fabs(end[2] - (-9.888727)) < 0.01);
}

MINI_TEST("Intersection", "Plane Plane To Line Canonical") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;
    // using session_cpp::Line;

    Point p0(0.0, 0.0, 2.0);
    Vector n0(0.0, 0.0, 1.0);
    Plane plane0 = Plane::from_point_normal(p0, n0);

    Point p1(3.0, 0.0, 0.0);
    Vector n1(1.0, 0.0, 0.0);
    Plane plane1 = Plane::from_point_normal(p1, n1);

    Line output;
    bool result = Intersection::plane_plane_to_line_canonical(plane0, plane1, output);

    MINI_CHECK(result);
    MINI_CHECK(TOLERANCE.is_close(output.start()[0], 3.0));
    MINI_CHECK(TOLERANCE.is_close(output.start()[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(output.start()[2], 2.0));
    MINI_CHECK(TOLERANCE.is_close(output.end()[1], -1.0));

    Point p2(0.0, 0.0, 5.0);
    Plane plane2 = Plane::from_point_normal(p2, n0);
    Line parallel;

    MINI_CHECK(!Intersection::plane_plane_to_line_canonical(plane0, plane2, parallel));
}

MINI_TEST("Intersection", "Line Plane") {
    // using session_cpp::Intersection;
    // using session_cpp::Line;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Point p(0.0, 0.0, 1.0);
    Vector n(0.0, 0.0, 1.0);
    Plane plane = Plane::from_point_normal(p, n);

    Line line(0.0, 0.0, 0.0, 0.0, 0.0, 2.0);

    Point output;
    bool result = Intersection::line_plane(line, plane, output, true);

    MINI_CHECK(result);
    MINI_CHECK(TOLERANCE.is_close(output[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(output[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(output[2], 1.0));
}

MINI_TEST("Intersection", "Line Plane Parallel") {
    // using session_cpp::Intersection;
    // using session_cpp::Line;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Point p(0.0, 0.0, 1.0);
    Vector n(0.0, 0.0, 1.0);
    Plane plane = Plane::from_point_normal(p, n);

    Line line(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

    Point output;
    bool result = Intersection::line_plane(line, plane, output, true);

    MINI_CHECK(!result);
}

MINI_TEST("Intersection", "Line Plane Real World") {
    // using session_cpp::Intersection;
    // using session_cpp::Line;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Line l0(500.000, -573.576, -819.152, 500.000, 573.576, 819.152);

    Point plane_origin_0(213.787107, 513.797811, -24.743845);
    Vector plane_xaxis_0(0.907673, -0.258819, 0.330366);
    Vector plane_yaxis_0(0.272094, 0.96225, 0.006285);
    Plane pl0(plane_origin_0, plane_xaxis_0, plane_yaxis_0);

    Point lp;
    bool result = Intersection::line_plane(l0, pl0, lp);

    MINI_CHECK(result);
    MINI_CHECK(std::fabs(lp[0] - 500.0) < 0.1);
    MINI_CHECK(std::fabs(lp[1] - 77.7531) < 0.01);
    MINI_CHECK(std::fabs(lp[2] - 111.043) < 0.01);
}

MINI_TEST("Intersection", "Plane Plane Plane") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Point plane_origin_0(213.787107, 513.797811, -24.743845);
    Vector plane_xaxis_0(0.907673, -0.258819, 0.330366);
    Vector plane_yaxis_0(0.272094, 0.96225, 0.006285);
    Plane pl0(plane_origin_0, plane_xaxis_0, plane_yaxis_0);

    Point plane_origin_1(247.17924, 499.115486, 59.619568);
    Vector plane_xaxis_1(0.552465, 0.816035, 0.16991);
    Vector plane_yaxis_1(0.172987, 0.087156, -0.98106);
    Plane pl1(plane_origin_1, plane_xaxis_1, plane_yaxis_1);

    Point plane_origin_2(221.399816, 605.893667, -54.000116);
    Vector plane_xaxis_2(0.903451, -0.360516, -0.231957);
    Vector plane_yaxis_2(0.172742, -0.189057, 0.966653);
    Plane pl2(plane_origin_2, plane_xaxis_2, plane_yaxis_2);

    Point output;
    bool result = Intersection::plane_plane_plane(pl0, pl1, pl2, output);

    MINI_CHECK(result);
    MINI_CHECK(std::fabs(output[0] - 300.5) < 0.1);
    MINI_CHECK(std::fabs(output[1] - 565.5) < 0.1);
    MINI_CHECK(std::fabs(output[2] - 0.0) < 0.1);
}

MINI_TEST("Intersection", "Plane Plane Plane Parallel") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Point p0(0.0, 0.0, 0.0);
    Vector n0(0.0, 0.0, 1.0);
    Plane plane0 = Plane::from_point_normal(p0, n0);

    Point p1(0.0, 0.0, 1.0);
    Vector n1(0.0, 0.0, 1.0);
    Plane plane1 = Plane::from_point_normal(p1, n1);

    Point p2(0.0, 0.0, 0.0);
    Vector n2(1.0, 0.0, 0.0);
    Plane plane2 = Plane::from_point_normal(p2, n2);

    Point output;
    bool result = Intersection::plane_plane_plane(plane0, plane1, plane2, output);

    MINI_CHECK(!result);
}

MINI_TEST("Intersection", "Ray Box") {
    // using session_cpp::Intersection;
    // using session_cpp::OBB;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Point center(0.0, 0.0, 0.0);
    Vector x_axis(1.0, 0.0, 0.0);
    Vector y_axis(0.0, 1.0, 0.0);
    Vector z_axis(0.0, 0.0, 1.0);
    Vector half_size(1.0, 1.0, 1.0);
    OBB box(center, x_axis, y_axis, z_axis, half_size);

    Point origin(-5.0, 0.0, 0.0);
    Vector direction(1.0, 0.0, 0.0);

    double tmin;
    double tmax;
    bool result = Intersection::ray_box(origin, direction, box, 0.0, 100.0, tmin, tmax);

    MINI_CHECK(result);
    MINI_CHECK(std::fabs(tmin - 4.0) < 1e-4);
    MINI_CHECK(std::fabs(tmax - 6.0) < 1e-4);
}

MINI_TEST("Intersection", "Ray Box Miss") {
    // using session_cpp::Intersection;
    // using session_cpp::OBB;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Point center(0.0, 0.0, 0.0);
    Vector x_axis(1.0, 0.0, 0.0);
    Vector y_axis(0.0, 1.0, 0.0);
    Vector z_axis(0.0, 0.0, 1.0);
    Vector half_size(1.0, 1.0, 1.0);
    OBB box(center, x_axis, y_axis, z_axis, half_size);

    Point origin(-5.0, 5.0, 0.0);
    Vector direction(1.0, 0.0, 0.0);

    double tmin;
    double tmax;
    bool result = Intersection::ray_box(origin, direction, box, 0.0, 100.0, tmin, tmax);

    MINI_CHECK(!result);
}

MINI_TEST("Intersection", "Ray Sphere") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Point origin(-5.0, 0.0, 0.0);
    Vector direction(1.0, 0.0, 0.0);
    Point center(0.0, 0.0, 0.0);
    double radius = 2.0;

    double t0;
    double t1;
    int hits = Intersection::ray_sphere(origin, direction, center, radius, t0, t1);

    MINI_CHECK(hits == 2);
    MINI_CHECK(std::fabs(t0 - 3.0) < 1e-4);
    MINI_CHECK(std::fabs(t1 - 7.0) < 1e-4);
}

MINI_TEST("Intersection", "Ray Sphere Tangent") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Point origin(-5.0, 2.0, 0.0);
    Vector direction(1.0, 0.0, 0.0);
    Point center(0.0, 0.0, 0.0);
    double radius = 2.0;

    double t0;
    double t1;
    int hits = Intersection::ray_sphere(origin, direction, center, radius, t0, t1);

    MINI_CHECK(hits == 1);
    MINI_CHECK(std::fabs(t0 - 5.0) < 1e-4);
}

MINI_TEST("Intersection", "Ray Sphere Miss") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Point origin(-5.0, 5.0, 0.0);
    Vector direction(1.0, 0.0, 0.0);
    Point center(0.0, 0.0, 0.0);
    double radius = 2.0;

    double t0;
    double t1;
    int hits = Intersection::ray_sphere(origin, direction, center, radius, t0, t1);

    MINI_CHECK(hits == 0);
}

MINI_TEST("Intersection", "Ray Triangle") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Point origin(0.5, 0.5, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    Point v0(0.0, 0.0, 0.0);
    Point v1(1.0, 0.0, 0.0);
    Point v2(0.0, 1.0, 0.0);

    double t;
    double u;
    double v;
    bool parallel;
    bool result = Intersection::ray_triangle(origin, direction, v0, v1, v2, 1e-6, t, u, v, parallel);

    MINI_CHECK(result);
    MINI_CHECK(!parallel);
    MINI_CHECK(std::fabs(t - 1.0) < 1e-4);
}

MINI_TEST("Intersection", "Ray Triangle Miss") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Point origin(2.0, 2.0, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    Point v0(0.0, 0.0, 0.0);
    Point v1(1.0, 0.0, 0.0);
    Point v2(0.0, 1.0, 0.0);

    double t;
    double u;
    double v;
    bool parallel;
    bool result = Intersection::ray_triangle(origin, direction, v0, v1, v2, 1e-6, t, u, v, parallel);

    MINI_CHECK(!result);
}

MINI_TEST("Intersection", "Ray Triangle Parallel") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Point origin(0.5, 0.5, -1.0);
    Vector direction(1.0, 0.0, 0.0);

    Point v0(0.0, 0.0, 0.0);
    Point v1(1.0, 0.0, 0.0);
    Point v2(0.0, 1.0, 0.0);

    double t;
    double u;
    double v;
    bool parallel;
    bool result = Intersection::ray_triangle(origin, direction, v0, v1, v2, 1e-6, t, u, v, parallel);

    MINI_CHECK(!result);
    MINI_CHECK(parallel);
}

MINI_TEST("Intersection", "Ray Mesh") {
    // using session_cpp::Intersection;
    // using session_cpp::Mesh;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    std::vector<std::vector<Point>> polygons = {
        {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        },
        {
            Point(0.0, 0.0, 1.0),
            Point(1.0, 0.0, 1.0),
            Point(1.0, 1.0, 1.0),
            Point(0.0, 1.0, 1.0),
        },
    };

    Mesh mesh = Mesh::from_polylines(polygons);

    Point origin(0.5, 0.5, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh(origin, direction, mesh, hits, true);

    MINI_CHECK(result);
    MINI_CHECK(hits.size() >= 1);
    MINI_CHECK(std::fabs(hits[0].t - 1.0) < 1e-3);
}

MINI_TEST("Intersection", "Ray Mesh First") {
    // using session_cpp::Intersection;
    // using session_cpp::Mesh;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    std::vector<std::vector<Point>> polygons = {
        {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        },
        {
            Point(0.0, 0.0, 1.0),
            Point(1.0, 0.0, 1.0),
            Point(1.0, 1.0, 1.0),
            Point(0.0, 1.0, 1.0),
        },
    };

    Mesh mesh = Mesh::from_polylines(polygons);

    Point origin(0.5, 0.5, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh(origin, direction, mesh, hits, false);

    MINI_CHECK(result);
    MINI_CHECK(hits.size() == 1);
}

MINI_TEST("Intersection", "Ray Mesh Miss") {
    // using session_cpp::Intersection;
    // using session_cpp::Mesh;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    std::vector<std::vector<Point>> polygons = {
        {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        },
    };

    Mesh mesh = Mesh::from_polylines(polygons);

    Point origin(5.0, 5.0, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh(origin, direction, mesh, hits, true);

    MINI_CHECK(!result);
    MINI_CHECK(hits.size() == 0);
}

MINI_TEST("Intersection", "Ray Mesh Bvh") {
    // using session_cpp::Intersection;
    // using session_cpp::Mesh;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    std::vector<std::vector<Point>> polygons = {
        {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        },
        {
            Point(0.0, 0.0, 1.0),
            Point(1.0, 0.0, 1.0),
            Point(1.0, 1.0, 1.0),
            Point(0.0, 1.0, 1.0),
        },
    };

    Mesh mesh = Mesh::from_polylines(polygons);

    Point origin(0.5, 0.5, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh_bvh(origin, direction, mesh, hits, true);

    MINI_CHECK(result);
    MINI_CHECK(hits.size() >= 1);
    MINI_CHECK(std::fabs(hits[0].t - 1.0) < 1e-3);
}

MINI_TEST("Intersection", "Ray Mesh Bvh First") {
    // using session_cpp::Intersection;
    // using session_cpp::Mesh;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    std::vector<std::vector<Point>> polygons = {
        {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        },
        {
            Point(0.0, 0.0, 1.0),
            Point(1.0, 0.0, 1.0),
            Point(1.0, 1.0, 1.0),
            Point(0.0, 1.0, 1.0),
        },
    };

    Mesh mesh = Mesh::from_polylines(polygons);

    Point origin(0.5, 0.5, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh_bvh(origin, direction, mesh, hits, false);

    MINI_CHECK(result);
    MINI_CHECK(hits.size() == 1);
}

MINI_TEST("Intersection", "Ray Mesh Bvh Miss") {
    // using session_cpp::Intersection;
    // using session_cpp::Mesh;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    std::vector<std::vector<Point>> polygons = {
        {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        },
    };

    Mesh mesh = Mesh::from_polylines(polygons);

    Point origin(5.0, 5.0, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh_bvh(origin, direction, mesh, hits, true);

    MINI_CHECK(!result);
    MINI_CHECK(hits.size() == 0);
}

MINI_TEST("Intersection", "Ray Mesh Bvh Vs Naive") {
    // using session_cpp::Intersection;
    // using session_cpp::Mesh;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    std::vector<std::vector<Point>> polygons;

    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            double x = static_cast<double>(i);
            double y = static_cast<double>(j);

            polygons.push_back(
                {Point(x, y, 0.0), Point(x + 1.0, y, 0.0), Point(x + 1.0, y + 1.0, 0.0), Point(x, y + 1.0, 0.0)}
            );
        }
    }

    Mesh mesh = Mesh::from_polylines(polygons);

    Point origin(5.5, 5.5, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits_naive;
    bool result_naive = Intersection::ray_mesh(origin, direction, mesh, hits_naive, true);

    std::vector<Intersection::RayHit> hits_bvh;
    bool result_bvh = Intersection::ray_mesh_bvh(origin, direction, mesh, hits_bvh, true);

    MINI_CHECK(result_naive == result_bvh);
    MINI_CHECK(hits_naive.size() == hits_bvh.size());

    if (!hits_naive.empty()) {
        MINI_CHECK(std::fabs(hits_naive[0].t - hits_bvh[0].t) < 1e-4);
        MINI_CHECK(hits_naive[0].face_index == hits_bvh[0].face_index);
    }
}

MINI_TEST("Intersection", "Ray Box Real World") {
    // using session_cpp::Intersection;
    // using session_cpp::OBB;
    // using session_cpp::Line;
    // using session_cpp::Point;

    Line l0(500.0, -573.576, -819.152, 500.0, 573.576, 819.152);
    Point min(214.0, 192.0, 484.0);
    Point max(694.0, 567.0, 796.0);
    std::vector<Point> points{min, max};
    OBB box = OBB::from_points(points);

    std::vector<Point> intersection_points;
    bool result = Intersection::ray_box(l0, box, 0.0, 1000.0, intersection_points);

    MINI_CHECK(result);
    MINI_CHECK(intersection_points.size() == 2);

    MINI_CHECK(std::fabs(intersection_points[0][0] - 500.0) < 0.1);
    MINI_CHECK(std::fabs(intersection_points[0][1] - 338.9) < 0.1);
    MINI_CHECK(std::fabs(intersection_points[0][2] - 484.0) < 0.1);

    MINI_CHECK(std::fabs(intersection_points[1][0] - 500.0) < 0.1);
    MINI_CHECK(std::fabs(intersection_points[1][1] - 557.365) < 0.1);
    MINI_CHECK(std::fabs(intersection_points[1][2] - 796.0) < 0.1);
}

MINI_TEST("Intersection", "Ray Sphere Real World") {
    // using session_cpp::Intersection;
    // using session_cpp::Line;
    // using session_cpp::Point;

    Line l0(500.0, -573.576, -819.152, 500.0, 573.576, 819.152);
    Point sphere_center(457.0, 192.0, 207.0);
    double radius = 265.0;

    std::vector<Point> sphere_points;
    bool result = Intersection::ray_sphere(l0, sphere_center, radius, sphere_points);

    MINI_CHECK(result);
    MINI_CHECK(sphere_points.size() == 2);

    MINI_CHECK(std::fabs(sphere_points[0][0] - 500.0) < 0.1);
    MINI_CHECK(std::fabs(sphere_points[0][1] - 12.08) < 0.1);
    MINI_CHECK(std::fabs(sphere_points[0][2] - 17.25) < 0.1);

    MINI_CHECK(std::fabs(sphere_points[1][0] - 500.0) < 0.1);
    MINI_CHECK(std::fabs(sphere_points[1][1] - 308.77) < 0.1);
    MINI_CHECK(std::fabs(sphere_points[1][2] - 440.97) < 0.1);
}

MINI_TEST("Intersection", "Ray Triangle Real World") {
    // using session_cpp::Intersection;
    // using session_cpp::Line;
    // using session_cpp::Point;
    // using session_cpp::Tolerance;

    Line l0(500.0, -573.576, -819.152, 500.0, 573.576, 819.152);
    Point p1(214.0, 567.0, 484.0);
    Point p2(214.0, 192.0, 796.0);
    Point p3(694.0, 192.0, 484.0);

    Point triangle_hit;
    bool result = Intersection::ray_triangle(l0, p1, p2, p3, Tolerance::APPROXIMATION, triangle_hit);

    MINI_CHECK(result);
    MINI_CHECK(std::fabs(triangle_hit[0] - 500.0) < 0.1);
    MINI_CHECK(std::fabs(triangle_hit[1] - 340.616) < 0.01);
    MINI_CHECK(std::fabs(triangle_hit[2] - 486.451) < 0.01);
}

MINI_TEST("Intersection", "Curve Plane") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::Vector;
    // using session_cpp::NurbsCurve;

    NurbsCurve circle = Primitives::circle(0.0, 0.0, 0.0, 2.0);
    Point origin(1.0, 0.0, 0.0);
    Vector normal(1.0, 0.0, 0.0);
    Plane plane = Plane::from_point_normal(origin, normal);
    std::vector<double> params = Intersection::curve_plane(circle, plane);
    std::vector<Point> points = Intersection::curve_plane_points(circle, plane);

    MINI_CHECK(params.size() == 2);
    MINI_CHECK(points.size() == 2);

    for (const Point& p : points) {
        MINI_CHECK(std::fabs(p[0] - 1.0) < 1e-9);
        MINI_CHECK(std::fabs(std::fabs(p[1]) - std::sqrt(3.0)) < 1e-9);
    }

    const double offset = 1.98 / std::sqrt(2.0);
    Plane diagonal = Plane::from_point_normal(Point(offset, offset, 0.0), Vector(1.0, 1.0, 0.0));
    std::vector<double> hidden = Intersection::curve_plane(circle, diagonal);

    MINI_CHECK(hidden.size() == 2);
}

MINI_TEST("Intersection", "Curve Plane Bezier Clipping") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::Vector;
    // using session_cpp::NurbsCurve;

    NurbsCurve circle = Primitives::circle(0.0, 0.0, 0.0, 2.0);
    Point origin(1.0, 0.0, 0.0);
    Vector normal(1.0, 0.0, 0.0);
    Plane plane = Plane::from_point_normal(origin, normal);
    std::vector<double> params = Intersection::curve_plane_bezier_clipping(circle, plane);

    MINI_CHECK(params.size() == 2);

    for (double t : params)
        MINI_CHECK(std::fabs(circle.point_at(t)[0] - 1.0) < 1e-9);

    NurbsCurve unit = Primitives::circle(0.0, 0.0, 0.0, 1.0);
    Plane tilted = Plane::from_point_normal(Point(0.0, 0.0, 0.2), Vector(0.3, 0.1, 1.0));
    std::vector<double> roots = Intersection::curve_plane_bezier_clipping(unit, tilted);

    MINI_CHECK(roots.size() == 2);
}

MINI_TEST("Intersection", "Curve Plane Algebraic") {
    // using session_cpp::Intersection;
    // using session_cpp::NurbsCurve;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::Vector;

    NurbsCurve curve = NurbsCurve::create(
        false,
        3,
        {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 2.0, 0.0),
            Point(2.0, 2.0, 0.0),
            Point(3.0, 0.0, 0.0),
        }
    );

    Point origin(1.0, 0.0, 0.0);
    Vector normal(1.0, 0.0, 0.0);
    Plane plane = Plane::from_point_normal(origin, normal);
    std::vector<double> params = Intersection::curve_plane_algebraic(curve, plane);

    MINI_CHECK(params.size() == 1);
    MINI_CHECK(std::fabs(curve.point_at(params[0])[0] - 1.0) < 1e-9);
    MINI_CHECK(std::fabs(curve.point_at(params[0])[1] - 4.0 / 3.0) < 1e-9);

    NurbsCurve circle = Primitives::circle(0.0, 0.0, 0.0, 2.0);
    std::vector<double> circle_params = Intersection::curve_plane_algebraic(circle, plane);

    MINI_CHECK(circle_params.size() == 2);
}

MINI_TEST("Intersection", "Curve Plane Production") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::Vector;
    // using session_cpp::NurbsCurve;

    NurbsCurve circle = Primitives::circle(0.0, 0.0, 0.0, 2.0);
    Point origin(1.0, 0.0, 0.0);
    Vector normal(1.0, 0.0, 0.0);
    Plane plane = Plane::from_point_normal(origin, normal);
    std::vector<double> params = Intersection::curve_plane_production(circle, plane);

    MINI_CHECK(params.size() == 2);

    for (double t : params)
        MINI_CHECK(std::fabs(circle.point_at(t)[0] - 1.0) < 1e-9);
}

MINI_TEST("Intersection", "Curve Closest Point") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsCurve;

    NurbsCurve circle = Primitives::circle(0.0, 0.0, 0.0, 2.0);
    Point test_point(3.0, 0.0, 0.0);
    std::pair<double, double> result = Intersection::curve_closest_point(circle, test_point);
    Point closest = circle.point_at(result.first);

    MINI_CHECK(std::fabs(result.second - 1.0) < 1e-6);
    MINI_CHECK(std::fabs(closest[0] - 2.0) < 1e-6);
    MINI_CHECK(std::fabs(closest[1]) < 1e-6);
}

MINI_TEST("Intersection", "Surface Plane") {
    // using session_cpp::Intersection;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;
    // using session_cpp::NurbsCurve;

    std::vector<Point> pts = {
        Point(0, 0, 0),
        Point(0, 10, 0),
        Point(10, 0, 10),
        Point(10, 10, 10),
    };

    NurbsSurface srf = NurbsSurface::create(false, false, 1, 1, 2, 2, pts);

    Point pp(0, 0, 5);
    Vector pn(0, 0, 1);
    Plane plane = Plane::from_point_normal(pp, pn);
    std::vector<NurbsCurve> curves = Intersection::surface_plane(srf, plane);

    MINI_CHECK(curves.size() == 1);
    MINI_CHECK(curves[0].is_valid());

    const std::pair<double, double> domain = curves[0].domain();
    const double t0 = domain.first;
    const double t1 = domain.second;

    for (int i = 0; i <= 10; i++) {
        double t = t0 + (t1 - t0) * i / 10.0;
        Point p = curves[0].point_at(t);

        MINI_CHECK(std::fabs(p[0] - 5.0) < 0.5);
        MINI_CHECK(std::fabs(p[2] - 5.0) < 0.5);
    }
}

MINI_TEST("Intersection", "Surface Plane Curved") {
    // using session_cpp::Intersection;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;
    // using session_cpp::NurbsCurve;

    std::vector<Point> pts;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double x = i * 10.0;
            double y = j * 10.0;
            double z = ((i == 1 || i == 2) && (j == 1 || j == 2)) ? 10.0 : 0.0;
            pts.push_back(Point(x, y, z));
        }
    }

    NurbsSurface srf = NurbsSurface::create(false, false, 3, 3, 4, 4, pts);

    Point pp(0, 0, 3);
    Vector pn(0, 0, 1);
    Plane plane = Plane::from_point_normal(pp, pn);
    std::vector<NurbsCurve> curves = Intersection::surface_plane(srf, plane);

    MINI_CHECK(curves.size() >= 1);
    MINI_CHECK(curves[0].is_valid());
    MINI_CHECK(curves[0].degree() == 3);

    const std::pair<double, double> domain = curves[0].domain();
    const double t0 = domain.first;
    const double t1 = domain.second;

    for (int i = 0; i <= 10; i++) {
        double t = t0 + (t1 - t0) * i / 10.0;
        Point p = curves[0].point_at(t);

        MINI_CHECK(std::fabs(p[2] - 3.0) < 1.0);
    }
}

MINI_TEST("Intersection", "Surface Plane Miss") {
    // using session_cpp::Intersection;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;
    // using session_cpp::NurbsCurve;

    std::vector<Point> pts = {
        Point(0, 0, 0),
        Point(0, 10, 0),
        Point(10, 0, 0),
        Point(10, 10, 0),
    };

    NurbsSurface srf = NurbsSurface::create(false, false, 1, 1, 2, 2, pts);

    Point pp(0, 0, 5);
    Vector pn(0, 0, 1);
    Plane plane = Plane::from_point_normal(pp, pn);
    std::vector<NurbsCurve> curves = Intersection::surface_plane(srf, plane);

    MINI_CHECK(curves.size() == 0);
}

MINI_TEST("Intersection", "Surface Plane UV") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::NurbsCurve;

    NurbsSurface cyl = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 4.0);
    Point pp(0.0, 0.0, 2.0);
    Vector pnv(0.3, 0.0, 1.0);
    Plane plane = Plane::from_point_normal(pp, pnv);
    std::vector<std::pair<NurbsCurve, NurbsCurve>> pairs = Intersection::surface_plane_uv(cyl, plane);

    MINI_CHECK(pairs.size() == 1);

    NurbsCurve curve3 = pairs[0].first;
    NurbsCurve pcurve = pairs[0].second;

    MINI_CHECK(curve3.is_valid());
    MINI_CHECK(pcurve.is_valid());
    MINI_CHECK(curve3.is_closed());

    const std::pair<double, double> domain_u = cyl.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;

    MINI_CHECK(std::fabs(pcurve.point_at(0.0)[0] - u1) < 1e-9 || std::fabs(pcurve.point_at(0.0)[0] - u0) < 1e-9);
    MINI_CHECK(std::fabs(pcurve.point_at(1.0)[0] - u1) < 1e-9 || std::fabs(pcurve.point_at(1.0)[0] - u0) < 1e-9);

    Vector pn = plane.z_axis();
    Point po = plane.origin();
    double max_off = 0.0;

    for (int i = 0; i < 17; i++) {
        Point p2 = pcurve.point_at(i / 16.0);
        Point s = cyl.point_at(p2[0], p2[1]);
        double off = std::fabs((s[0] - po[0]) * pn[0] + (s[1] - po[1]) * pn[1] + (s[2] - po[2]) * pn[2]);
        max_off = std::max(max_off, off);
    }

    MINI_CHECK(max_off < 0.05);

    NurbsSurface torus = Primitives::torus_surface(0.0, 0.0, 0.0, 2.0, 0.5);
    Point pp2(0.0, 0.0, 0.0);
    Vector pnv2(0.0, 0.0, 1.0);
    Plane plane2 = Plane::from_point_normal(pp2, pnv2);
    std::vector<std::pair<NurbsCurve, NurbsCurve>> pairs2 = Intersection::surface_plane_uv(torus, plane2);

    MINI_CHECK(pairs2.size() == 2);

    const std::pair<double, double> domain_tu = torus.domain(0);
    const double tu0 = domain_tu.first;
    const double tu1 = domain_tu.second;
    const std::pair<double, double> domain_tv = torus.domain(1);
    const double tv0 = domain_tv.first;
    const double tv1 = domain_tv.second;
    bool inside = true;

    for (std::pair<NurbsCurve, NurbsCurve>& pair : pairs2) {
        for (int i = 0; i < 17; i++) {
            Point p2 = pair.second.point_at(i / 16.0);

            if (p2[0] < tu0 - 1e-6 || p2[0] > tu1 + 1e-6 || p2[1] < tv0 - 1e-6 || p2[1] > tv1 + 1e-6)
                inside = false;
        }
    }

    MINI_CHECK(inside);
}

MINI_TEST("Intersection", "Surface Surface") {
    // using session_cpp::Intersection;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsCurve;

    NurbsSurface flat = NurbsSurface::create(
        false,
        false,
        1,
        1,
        2,
        2,
        {
            Point(-3.0, -3.0, 0.5),
            Point(-3.0, 3.0, 0.5),
            Point(3.0, -3.0, 0.5),
            Point(3.0, 3.0, 0.5),
        }
    );

    NurbsSurface cyl = Primitives::cylinder_surface(0.0, 0.0, -2.0, 1.0, 4.0);
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> flat_triples = Intersection::surface_surface(flat, cyl);

    MINI_CHECK(flat_triples.size() == 1);

    NurbsCurve c3 = std::get<0>(flat_triples[0]);
    NurbsCurve pa = std::get<1>(flat_triples[0]);
    NurbsCurve pb = std::get<2>(flat_triples[0]);

    MINI_CHECK(c3.is_valid() && pa.is_valid() && pb.is_valid());
    MINI_CHECK(c3.is_closed());
    MINI_CHECK(lies_on_curve(c3, pa, flat) < 0.05);
    MINI_CHECK(lies_on_curve(c3, pb, cyl) < 0.05);

    NurbsSurface sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
    NurbsSurface cyl2 = Primitives::cylinder_surface(1.3, 0.0, -3.0, 0.3, 6.0);
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> triples = Intersection::surface_surface(sphere, cyl2);

    MINI_CHECK(triples.size() >= 2);

    int clean = 0;

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& triple : triples) {
        NurbsCurve tc3 = std::get<0>(triple);
        NurbsCurve tpa = std::get<1>(triple);
        NurbsCurve tpb = std::get<2>(triple);

        MINI_CHECK(tc3.is_valid() && tpa.is_valid() && tpb.is_valid());

        if (lies_on_curve(tc3, tpa, sphere) < 0.05 && lies_on_curve(tc3, tpb, cyl2) < 0.05)
            clean++;
    }

    MINI_CHECK(clean >= 2);

    NurbsSurface sphere2 = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);

    NurbsSurface flat04 = NurbsSurface::create(
        false,
        false,
        1,
        1,
        2,
        2,
        {
            Point(-3.0, -3.0, 0.4),
            Point(-3.0, 3.0, 0.4),
            Point(3.0, -3.0, 0.4),
            Point(3.0, 3.0, 0.4),
        }
    );

    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> ex_triples = Intersection::surface_surface(sphere2, flat04);

    MINI_CHECK(ex_triples.size() == 1);

    NurbsCurve ex_c3 = std::get<0>(ex_triples[0]);
    double expected_r = std::sqrt(3.84);
    double max_dev = 0.0;

    for (int j = 0; j <= 256; j++) {
        Point p = ex_c3.point_at(j / 256.0);
        double rr = std::sqrt(p[0] * p[0] + p[1] * p[1]);
        max_dev = std::max(max_dev, std::abs(rr - expected_r));
        max_dev = std::max(max_dev, std::abs(p[2] - 0.4));
    }

    MINI_CHECK(max_dev < 1e-9);
}

MINI_TEST("Intersection", "Surface Surface Accuracy") {
    // using session_cpp::Intersection;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsCurve;

    NurbsSurface sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
    NurbsSurface cyl = Primitives::cylinder_surface(1.3, 0.0, -3.0, 0.3, 6.0);
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> tr = Intersection::surface_surface(sphere, cyl);

    MINI_CHECK(tr.size() >= 2);

    for (std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : tr)
        MINI_CHECK(on_both(std::get<0>(t), distance_sphere, distance_cylinder) < 1e-5);

    NurbsSurface sphere2 = Primitives::sphere_surface(2.0, 0.0, 0.0, 2.0);
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> tr2 = Intersection::surface_surface(sphere, sphere2);

    MINI_CHECK(tr2.size() >= 1);

    for (std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : tr2)
        MINI_CHECK(on_both(std::get<0>(t), distance_sphere, distance_sphere2) < 1e-6);

    NurbsSurface torus = Primitives::torus_surface(0.0, 0.0, 0.0, 2.0, 0.5);

    NurbsSurface flat = NurbsSurface::create(
        false,
        false,
        1,
        1,
        2,
        2,
        {Point(-9.0, -9.0, 0.0), Point(-9.0, 9.0, 0.0), Point(9.0, -9.0, 0.0), Point(9.0, 9.0, 0.0)}
    );

    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> tr3 = Intersection::surface_surface(torus, flat);

    MINI_CHECK(tr3.size() == 2);

    for (std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : tr3)
        MINI_CHECK(on_both(std::get<0>(t), distance_torus, distance_flat) < 1e-6);
}

MINI_TEST("Intersection", "Surface Surface Planes") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::NurbsSurface;
    // using session_cpp::NurbsCurve;

    NurbsSurface flat = bilinear(Point(-3.0, -3.0, 0.5), Point(-3.0, 3.0, 0.5), Point(3.0, -3.0, 0.5), Point(3.0, 3.0, 0.5));
    NurbsSurface wall = bilinear(Point(0.2, -3.0, -3.0), Point(0.2, -3.0, 3.0), Point(0.2, 3.0, -3.0), Point(0.2, 3.0, 3.0));
    NurbsSurface far = bilinear(Point(5.0, -3.0, -3.0), Point(5.0, -3.0, 3.0), Point(5.0, 3.0, -3.0), Point(5.0, 3.0, 3.0));
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> tr = Intersection::surface_surface(flat, wall);

    MINI_CHECK(tr.size() == 1);

    NurbsCurve c3 = std::get<0>(tr[0]);
    Point start = c3.point_at_start();
    Point end = c3.point_at_end();

    MINI_CHECK(TOLERANCE.is_close(start[0], 0.2) && TOLERANCE.is_close(start[1], -3.0) && TOLERANCE.is_close(start[2], 0.5));
    MINI_CHECK(TOLERANCE.is_close(end[0], 0.2) && TOLERANCE.is_close(end[1], 3.0) && TOLERANCE.is_close(end[2], 0.5));
    MINI_CHECK(lies_on_curve(c3, std::get<1>(tr[0]), flat) < 1e-9);
    MINI_CHECK(lies_on_curve(c3, std::get<2>(tr[0]), wall) < 1e-9);
    MINI_CHECK(Intersection::surface_surface(flat, far).empty());
}

MINI_TEST("Intersection", "Surface Surface Plane Cone") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::NurbsCurve;

    NurbsSurface cone = Primitives::cone_surface(0.0, 0.0, 0.0, 1.5, 3.0);
    NurbsSurface flat = bilinear(Point(-3.0, -3.0, 0.5), Point(-3.0, 3.0, 0.5), Point(3.0, -3.0, 0.5), Point(3.0, 3.0, 0.5));
    NurbsSurface steep = bilinear(Point(1.1, -3.0, -3.0), Point(1.1, 3.0, -3.0), Point(-0.3, -3.0, 4.0), Point(-0.3, 3.0, 4.0));
    NurbsSurface slant = bilinear(Point(-3.0, -3.0, 8.5), Point(-3.0, 3.0, 8.5), Point(3.0, -3.0, -3.5), Point(3.0, 3.0, -3.5));
    NurbsSurface axial = bilinear(Point(0.0, -3.0, -3.0), Point(0.0, -3.0, 4.0), Point(0.0, 3.0, -3.0), Point(0.0, 3.0, 4.0));
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> circle = Intersection::surface_surface(cone, flat);

    MINI_CHECK(circle.size() == 1);
    MINI_CHECK(on_both(std::get<0>(circle[0]), distance_cone, distance_flat_half) < 1e-9);
    MINI_CHECK(lies_on_curve(std::get<0>(circle[0]), std::get<1>(circle[0]), cone) < 1e-9);

    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> hyperbola = Intersection::surface_surface(cone, steep);

    MINI_CHECK(hyperbola.size() == 1);
    MINI_CHECK(std::get<0>(hyperbola[0]).degree() == 2);
    MINI_CHECK(on_both(std::get<0>(hyperbola[0]), distance_cone, distance_cone) < 1e-9);

    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> parabola = Intersection::surface_surface(cone, slant);

    MINI_CHECK(parabola.size() == 1);
    MINI_CHECK(on_both(std::get<0>(parabola[0]), distance_cone, distance_cone) < 1e-6);

    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> lines = Intersection::surface_surface(cone, axial);

    MINI_CHECK(lines.size() == 2);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& line : lines) {
        Point apex = std::get<0>(line).point_at_start();

        MINI_CHECK(TOLERANCE.is_close(apex[0], 0.0) && TOLERANCE.is_close(apex[1], 0.0) && TOLERANCE.is_close(apex[2], 3.0));
        MINI_CHECK(on_both(std::get<0>(line), distance_cone, distance_cone) < 1e-9);
    }
}

MINI_TEST("Intersection", "Surface Surface Plane Torus") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::NurbsCurve;

    NurbsSurface torus = Primitives::torus_surface(0.0, 0.0, 0.0, 2.0, 0.5);
    NurbsSurface wall = bilinear(Point(0.2, -3.0, -3.0), Point(0.2, -3.0, 3.0), Point(0.2, 3.0, -3.0), Point(0.2, 3.0, 3.0));
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> tr = Intersection::surface_surface(torus, wall);

    MINI_CHECK(tr.size() == 2);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : tr) {
        MINI_CHECK(std::get<0>(t).is_closed());
        MINI_CHECK(on_both(std::get<0>(t), distance_torus, distance_wall) < 1e-4);
        MINI_CHECK(lies_on_curve(std::get<0>(t), std::get<2>(t), wall) < 1e-4);
    }
}

MINI_TEST("Intersection", "Surface Surface Cylinders") {
    // using session_cpp::Intersection;
    // using session_cpp::Primitives;
    // using session_cpp::Tolerance;
    // using session_cpp::Vector;
    // using session_cpp::Xform;
    // using session_cpp::NurbsSurface;
    // using session_cpp::NurbsCurve;
    // using session_cpp::Point;

    NurbsSurface cyl = Primitives::cylinder_surface(0.0, 0.0, -2.0, 1.0, 4.0);
    NurbsSurface beside = Primitives::cylinder_surface(1.5, 0.0, -2.0, 1.0, 4.0);
    NurbsSurface across = Primitives::cylinder_surface(0.0, 0.0, -2.0, 1.0, 4.0).transformed(Xform::rotation(Vector(0.0, 1.0, 0.0), Tolerance::HALF_PI));
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> lines = Intersection::surface_surface(cyl, beside);

    MINI_CHECK(lines.size() == 2);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& line : lines) {
        Point start = std::get<0>(line).point_at_start();
        Point end = std::get<0>(line).point_at_end();

        MINI_CHECK(TOLERANCE.is_close(start[0], 0.75) && TOLERANCE.is_close(end[0], 0.75));
        MINI_CHECK(TOLERANCE.is_close(std::abs(start[1]), std::sqrt(0.4375)) && TOLERANCE.is_close(start[1], end[1]));
        MINI_CHECK(lies_on_curve(std::get<0>(line), std::get<1>(line), cyl) < 1e-9);
    }

    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> ellipses = Intersection::surface_surface(cyl, across);

    MINI_CHECK(ellipses.size() == 3);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& ellipse : ellipses) {
        MINI_CHECK(on_both(std::get<0>(ellipse), distance_unit_cylinder, distance_x_cylinder) < 1e-9);
        MINI_CHECK(pcurve_end_gap(std::get<0>(ellipse), std::get<1>(ellipse), cyl) < 1e-4);
        MINI_CHECK(pcurve_end_gap(std::get<0>(ellipse), std::get<2>(ellipse), across) < 1e-4);
    }
}

MINI_TEST("Intersection", "Surface Surface Coaxial Quadrics") {
    // using session_cpp::Intersection;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::NurbsCurve;

    NurbsSurface sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
    NurbsSurface cyl = Primitives::cylinder_surface(0.0, 0.0, -2.0, 1.0, 4.0);
    NurbsSurface cone = Primitives::cone_surface(0.0, 0.0, 0.0, 1.5, 3.0);
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> sphere_cyl = Intersection::surface_surface(sphere, cyl);

    MINI_CHECK(sphere_cyl.size() == 2);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : sphere_cyl) {
        MINI_CHECK(TOLERANCE.is_close(std::abs(std::get<0>(t).point_at_start()[2]), std::sqrt(3.0)));
        MINI_CHECK(on_both(std::get<0>(t), distance_sphere, distance_unit_cylinder) < 1e-9);
        MINI_CHECK(lies_on_curve(std::get<0>(t), std::get<1>(t), sphere) < 1e-9);
        MINI_CHECK(lies_on_curve(std::get<0>(t), std::get<2>(t), cyl) < 1e-9);
    }

    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> cyl_cone = Intersection::surface_surface(cyl, cone);

    MINI_CHECK(cyl_cone.size() == 1);
    MINI_CHECK(TOLERANCE.is_close(std::get<0>(cyl_cone[0]).point_at_start()[2], 1.0));
    MINI_CHECK(on_both(std::get<0>(cyl_cone[0]), distance_unit_cylinder, distance_cone) < 1e-9);
    MINI_CHECK(lies_on_curve(std::get<0>(cyl_cone[0]), std::get<2>(cyl_cone[0]), cone) < 1e-9);

    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> cone_sphere = Intersection::surface_surface(cone, sphere);

    MINI_CHECK(cone_sphere.size() == 2);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : cone_sphere)
        MINI_CHECK(on_both(std::get<0>(t), distance_cone, distance_sphere) < 1e-9);
}

MINI_TEST("Intersection", "Surface Surface Coaxial Tori") {
    // using session_cpp::Intersection;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::NurbsCurve;

    NurbsSurface torus = Primitives::torus_surface(0.0, 0.0, 0.0, 2.0, 0.5);
    NurbsSurface wide_cyl = Primitives::cylinder_surface(0.0, 0.0, -2.0, 2.2, 4.0);
    NurbsSurface cone = Primitives::cone_surface(0.0, 0.0, 0.0, 1.5, 3.0);
    NurbsSurface high_torus = Primitives::torus_surface(0.0, 0.0, 1.0, 1.0, 0.3);
    NurbsSurface sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
    NurbsSurface wide_torus = Primitives::torus_surface(0.0, 0.0, 0.3, 2.3, 0.5);
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> cyl_torus = Intersection::surface_surface(wide_cyl, torus);

    MINI_CHECK(cyl_torus.size() == 2);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : cyl_torus) {
        MINI_CHECK(TOLERANCE.is_close(std::abs(std::get<0>(t).point_at_start()[2]), std::sqrt(0.21)));
        MINI_CHECK(on_both(std::get<0>(t), distance_wide_cylinder, distance_torus) < 1e-9);
        MINI_CHECK(lies_on_curve(std::get<0>(t), std::get<2>(t), torus) < 1e-9);
    }

    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> cone_torus = Intersection::surface_surface(cone, high_torus);

    MINI_CHECK(cone_torus.size() == 2);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : cone_torus) {
        MINI_CHECK(on_both(std::get<0>(t), distance_cone, distance_high_torus) < 1e-9);
        MINI_CHECK(lies_on_curve(std::get<0>(t), std::get<2>(t), high_torus) < 1e-9);
    }

    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> sphere_torus = Intersection::surface_surface(sphere, torus);

    MINI_CHECK(sphere_torus.size() == 2);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : sphere_torus)
        MINI_CHECK(on_both(std::get<0>(t), distance_sphere, distance_torus) < 1e-9);

    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> torus_torus = Intersection::surface_surface(torus, wide_torus);

    MINI_CHECK(torus_torus.size() == 2);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : torus_torus) {
        MINI_CHECK(on_both(std::get<0>(t), distance_torus, distance_wide_torus) < 1e-9);
        MINI_CHECK(lies_on_curve(std::get<0>(t), std::get<1>(t), torus) < 1e-9);
    }
}

MINI_TEST("Intersection", "Surface Surface Cone Apex") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::NurbsCurve;

    NurbsSurface cone = Primitives::cone_surface(0.0, 0.0, 0.0, 1.5, 3.0);
    NurbsSurface axial = bilinear(Point(0.0, -3.0, -3.0), Point(0.0, -3.0, 4.0), Point(0.0, 3.0, -3.0), Point(0.0, 3.0, 4.0));
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> lines = Intersection::surface_surface(cone, axial);

    MINI_CHECK(lines.size() == 2);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& line : lines)
        MINI_CHECK(pcurve_line_deviation(std::get<0>(line), std::get<1>(line), cone) < 1e-9);
}

MINI_TEST("Intersection", "Surface Surface Seam Pieces") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::Tolerance;
    // using session_cpp::NurbsSurface;
    // using session_cpp::NurbsCurve;

    NurbsSurface sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
    NurbsSurface cone = Primitives::cone_surface(0.0, 0.0, 0.0, 1.5, 3.0);
    NurbsSurface wall = bilinear(Point(0.2, -3.0, -3.0), Point(0.2, -3.0, 3.0), Point(0.2, 3.0, -3.0), Point(0.2, 3.0, 3.0));
    NurbsSurface slanted = bilinear(Point(-3.0, -3.0, 0.0), Point(-3.0, 3.0, -0.6), Point(3.0, -3.0, 1.2), Point(3.0, 3.0, 0.6));
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> circle = Intersection::surface_surface(sphere, wall);
    double length = 0.0;

    MINI_CHECK(circle.size() == 2);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : circle) {
        length += std::get<0>(t).length();

        MINI_CHECK(pcurve_end_gap(std::get<0>(t), std::get<1>(t), sphere) < 1e-9);
        MINI_CHECK(lifted_distance(std::get<1>(t), sphere, distance_wall) < 5e-3);
    }

    MINI_CHECK(std::abs(length - 2.0 * Tolerance::PI * std::sqrt(3.96)) < 1e-4);

    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> conic = Intersection::surface_surface(cone, slanted);

    MINI_CHECK(conic.size() == 2);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : conic) {
        MINI_CHECK(pcurve_end_gap(std::get<0>(t), std::get<1>(t), cone) < 1e-4);
        MINI_CHECK(lifted_distance(std::get<1>(t), cone, distance_slanted) < 1e-3);
    }
}

MINI_TEST("Intersection", "Surface Surface Seam Crossings") {
    // using session_cpp::Intersection;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::NurbsCurve;
    // using session_cpp::Point;

    NurbsSurface sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
    NurbsSurface cyl = Primitives::cylinder_surface(1.3, 0.0, -3.0, 0.3, 6.0);
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> tr = Intersection::surface_surface(sphere, cyl);
    double seam_gap = 0.0;

    MINI_CHECK(tr.size() == 4);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : tr) {
        for (const Point& uv : {std::get<1>(t).point_at_start(), std::get<1>(t).point_at_end()})
            seam_gap = std::max(seam_gap, std::min(std::abs(uv[0]), std::abs(uv[0] - 4.0)));

        MINI_CHECK(on_both(std::get<0>(t), distance_sphere, distance_cylinder) < 1e-5);
    }

    MINI_CHECK(seam_gap < 1e-8);
}

MINI_TEST("Intersection", "Cut Curves On Surface") {
    // using session_cpp::Intersection;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsCurve;

    NurbsSurface flat = NurbsSurface::create(
        false,
        false,
        1,
        1,
        2,
        2,
        {
            Point(-3.0, -3.0, 0.0),
            Point(-3.0, 3.0, 0.0),
            Point(3.0, -3.0, 0.0),
            Point(3.0, 3.0, 0.0),
        }
    );

    NurbsSurface cyl = Primitives::cylinder_surface(0.0, 0.0, -2.0, 1.0, 4.0);
    std::vector<NurbsCurve> pcurves = Intersection::cut_curves_on_surface(flat, cyl);

    MINI_CHECK(pcurves.size() == 1);
    MINI_CHECK(pcurves[0].is_valid());

    double max_off = 0.0;

    for (int i = 0; i <= 16; i++) {
        Point uv = pcurves[0].point_at(i / 16.0);
        Point p = flat.point_at(uv[0], uv[1]);
        max_off = std::max(max_off, std::fabs(std::sqrt(p[0] * p[0] + p[1] * p[1]) - 1.0));
    }

    MINI_CHECK(max_off < 1e-3);
}

MINI_TEST("Intersection", "Cut Curves On Surface Pullbacks") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::NurbsCurve;

    NurbsSurface sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
    NurbsSurface cone = Primitives::cone_surface(0.0, 0.0, 0.0, 1.5, 3.0);
    NurbsSurface wall = bilinear(Point(0.2, -3.0, -3.0), Point(0.2, -3.0, 3.0), Point(0.2, 3.0, -3.0), Point(0.2, 3.0, 3.0));
    NurbsSurface square = bilinear(Point(-1.6, -1.6, 0.5), Point(-1.6, 1.6, 0.5), Point(1.6, -1.6, 0.5), Point(1.6, 1.6, 0.5));
    std::vector<NurbsCurve> sphere_cuts = Intersection::cut_curves_on_surface(sphere, wall);

    MINI_CHECK(sphere_cuts.size() == 2);

    for (const NurbsCurve& pc : sphere_cuts)
        MINI_CHECK(lifted_distance(pc, sphere, distance_wall) < 5e-3);

    std::vector<NurbsCurve> cone_cuts = Intersection::cut_curves_on_surface(cone, wall);

    MINI_CHECK(cone_cuts.size() == 2);

    for (const NurbsCurve& pc : cone_cuts)
        MINI_CHECK(lifted_distance(pc, cone, distance_wall) < 1e-3);

    std::vector<NurbsCurve> square_cuts = Intersection::cut_curves_on_surface(sphere, square);

    MINI_CHECK(square_cuts.size() == 4);

    for (const NurbsCurve& pc : square_cuts)
        MINI_CHECK(lifted_distance(pc, sphere, distance_square) < 2e-3);
}

MINI_TEST("Intersection", "Cut Curves On Surface Torus") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::NurbsCurve;

    NurbsSurface torus = Primitives::torus_surface(0.0, 0.0, 0.0, 2.0, 0.5);
    NurbsSurface wall = bilinear(Point(0.2, -3.0, -3.0), Point(0.2, -3.0, 3.0), Point(0.2, 3.0, -3.0), Point(0.2, 3.0, 3.0));
    std::vector<NurbsCurve> cuts = Intersection::cut_curves_on_surface(torus, wall);

    MINI_CHECK(cuts.size() == 2);

    for (const NurbsCurve& pc : cuts)
        MINI_CHECK(lifted_distance(pc, torus, distance_wall) < 1e-5);
}

MINI_TEST("Intersection", "Cut Curves Slanted Cutter") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;
    // using session_cpp::NurbsCurve;

    NurbsSurface cone = Primitives::cone_surface(0.0, 0.0, 0.0, 1.5, 3.0);
    NurbsSurface slanted = bilinear(Point(-3.0, -3.0, 0.0), Point(-3.0, 3.0, -0.6), Point(3.0, -3.0, 1.2), Point(3.0, 3.0, 0.6));
    std::vector<NurbsCurve> cuts = Intersection::cut_curves_on_surface(cone, slanted);

    MINI_CHECK(cuts.size() == 2);

    const std::pair<double, double> domain = cuts[0].domain();
    Point uv = cuts[0].point_at((domain.first + domain.second) * 0.5);
    Point p = cone.point_at(uv[0], uv[1]);

    MINI_CHECK(distance_cone(p) < 1e-3);
    MINI_CHECK(distance_slanted(p) < 1e-3);
}

MINI_TEST("Intersection", "Cut Curves Plane Trapezoid") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::NurbsSurface;
    // using session_cpp::NurbsCurve;

    const NurbsSurface trapezoid = bilinear(Point(-3.0, -3.0, 0.0), Point(-1.0, 3.0, 0.0), Point(3.0, -3.0, 0.0), Point(7.0, 3.0, 0.0));
    const NurbsSurface wall = bilinear(Point(6.0, -5.0, -1.0), Point(6.0, 5.0, -1.0), Point(6.0, -5.0, 1.0), Point(6.0, 5.0, 1.0));
    const std::vector<NurbsCurve> target_cuts = Intersection::cut_curves_on_surface(trapezoid, wall);
    const std::vector<NurbsCurve> cutter_cuts = Intersection::cut_curves_on_surface(wall, trapezoid);

    MINI_CHECK(target_cuts.size() == 1);
    MINI_CHECK(cutter_cuts.size() == 1);

    const std::pair<double, double> target_domain = target_cuts[0].domain();
    const Point target_uv0 = target_cuts[0].point_at(target_domain.first);
    const Point target_uv1 = target_cuts[0].point_at(target_domain.second);
    const Point target_p0 = trapezoid.point_at(target_uv0[0], target_uv0[1]);
    const Point target_p1 = trapezoid.point_at(target_uv1[0], target_uv1[1]);

    MINI_CHECK(std::abs(target_p0[0] - 6.0) < 1e-3);
    MINI_CHECK(std::abs(target_p1[0] - 6.0) < 1e-3);
    MINI_CHECK(std::abs(std::min(target_p0[1], target_p1[1]) - 1.5) < 1e-3);
    MINI_CHECK(std::abs(std::max(target_p0[1], target_p1[1]) - 3.0) < 1e-3);

    const std::pair<double, double> cutter_domain = cutter_cuts[0].domain();
    const Point cutter_uv0 = cutter_cuts[0].point_at(cutter_domain.first);
    const Point cutter_uv1 = cutter_cuts[0].point_at(cutter_domain.second);
    const Point cutter_p0 = wall.point_at(cutter_uv0[0], cutter_uv0[1]);
    const Point cutter_p1 = wall.point_at(cutter_uv1[0], cutter_uv1[1]);

    MINI_CHECK(std::abs(std::min(cutter_p0[1], cutter_p1[1]) - 1.5) < 1e-3);
    MINI_CHECK(std::abs(std::max(cutter_p0[1], cutter_p1[1]) - 3.0) < 1e-3);
}

MINI_TEST("Intersection", "Remap") {
    // using session_cpp::Intersection;

    MINI_CHECK(std::fabs(Intersection::remap(5.0, 0.0, 10.0, 0.0, 1.0) - 0.5) < 1e-9);
    MINI_CHECK(std::fabs(Intersection::remap(0.0, 0.0, 10.0, 0.0, 1.0) - 0.0) < 1e-9);
    MINI_CHECK(std::fabs(Intersection::remap(10.0, 0.0, 10.0, 0.0, 1.0) - 1.0) < 1e-9);
}

MINI_TEST("Intersection", "Closest Point On Segment") {
    // using session_cpp::Intersection;
    // using session_cpp::Line;
    // using session_cpp::Point;

    Line seg(0.0, 0.0, 0.0, 4.0, 0.0, 0.0);
    Point pt(2.0, 3.0, 0.0);
    Point cp;
    double t;
    bool result = Intersection::closest_point_on_segment(pt, seg, cp, t);

    MINI_CHECK(result);
    MINI_CHECK(std::fabs(cp[0] - 2.0) < 1e-9);
    MINI_CHECK(std::fabs(cp[1] - 0.0) < 1e-9);
    MINI_CHECK(std::fabs(t - 0.5) < 1e-9);

    Point pt2(-2.0, 1.0, 0.0);
    Point cp2;
    double t2;
    Intersection::closest_point_on_segment(pt2, seg, cp2, t2);

    MINI_CHECK(std::fabs(cp2[0] - 0.0) < 1e-9);
    MINI_CHECK(std::fabs(t2 - 0.0) < 1e-9);
}

MINI_TEST("Intersection", "Plane Plane Plane Check Parallel") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Point o0(0.0, 0.0, 0.0);
    Vector n0(0.0, 0.0, 1.0);
    Point o1(0.0, 0.0, 1.0);
    Vector n1(0.0, 0.0, 1.0);
    Point o2(0.0, 0.0, 2.0);
    Vector n2(0.0, 0.0, 1.0);
    Plane p0 = Plane::from_point_normal(o0, n0);
    Plane p1 = Plane::from_point_normal(o1, n1);
    Plane p2 = Plane::from_point_normal(o2, n2);
    Point out;

    MINI_CHECK(!Intersection::plane_plane_plane_check(p0, p1, p2, 0.1, out));

    Point px_o(1.0, 0.0, 0.0);
    Vector px_n(1.0, 0.0, 0.0);
    Point py_o(0.0, 2.0, 0.0);
    Vector py_n(0.0, 1.0, 0.0);
    Point pz_o(0.0, 0.0, 3.0);
    Vector pz_n(0.0, 0.0, 1.0);
    Plane px = Plane::from_point_normal(px_o, px_n);
    Plane py = Plane::from_point_normal(py_o, py_n);
    Plane pz = Plane::from_point_normal(pz_o, pz_n);
    Point pt;
    bool ok = Intersection::plane_plane_plane_check(px, py, pz, 0.1, pt);

    MINI_CHECK(ok);
    MINI_CHECK(std::fabs(pt[0] - 1.0) < 1e-6);
    MINI_CHECK(std::fabs(pt[1] - 2.0) < 1e-6);
    MINI_CHECK(std::fabs(pt[2] - 3.0) < 1e-6);
}

MINI_TEST("Intersection", "Plane 4 Planes Closed") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;
    // using session_cpp::Polyline;

    Point mp(0.0, 0.0, 0.0);
    Vector mn(0.0, 0.0, 1.0);
    Plane main_plane = Plane::from_point_normal(mp, mn);
    Point o0(-1.0, 0.0, 0.0);
    Vector n0(1.0, 0.0, 0.0);
    Point o1(0.0, -1.0, 0.0);
    Vector n1(0.0, 1.0, 0.0);
    Point o2(1.0, 0.0, 0.0);
    Vector n2(1.0, 0.0, 0.0);
    Point o3(0.0, 1.0, 0.0);
    Vector n3(0.0, 1.0, 0.0);

    std::array<Plane, 4> planes = {
        Plane::from_point_normal(o0, n0),
        Plane::from_point_normal(o1, n1),
        Plane::from_point_normal(o2, n2),
        Plane::from_point_normal(o3, n3),
    };

    Polyline result;
    bool ok = Intersection::plane_4planes(main_plane, planes, result);

    MINI_CHECK(ok);
    MINI_CHECK(result.point_count() == 5);

    for (size_t i = 0; i < result.point_count(); i++) {
        Point p = result.get_point(i);

        MINI_CHECK(std::fabs(p[2]) < 1e-6);
    }

    Point first = result.get_point(0);
    Point last = result.get_point(4);

    MINI_CHECK(std::fabs(first[0] - last[0]) < 1e-6);
    MINI_CHECK(std::fabs(first[1] - last[1]) < 1e-6);
}

MINI_TEST("Intersection", "Plane 4 Planes Open") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;
    // using session_cpp::Polyline;

    Point mp(0.0, 0.0, 0.0);
    Vector mn(0.0, 0.0, 1.0);
    Plane main_plane = Plane::from_point_normal(mp, mn);
    Point o0(-1.0, 0.0, 0.0);
    Vector n0(1.0, 0.0, 0.0);
    Point o1(0.0, -1.0, 0.0);
    Vector n1(0.0, 1.0, 0.0);
    Point o2(1.0, 0.0, 0.0);
    Vector n2(1.0, 0.0, 0.0);
    Point o3(0.0, 1.0, 0.0);
    Vector n3(0.0, 1.0, 0.0);

    std::array<Plane, 4> planes = {
        Plane::from_point_normal(o0, n0),
        Plane::from_point_normal(o1, n1),
        Plane::from_point_normal(o2, n2),
        Plane::from_point_normal(o3, n3),
    };

    Polyline result;
    bool ok = Intersection::plane_4planes_open(main_plane, planes, result);

    MINI_CHECK(ok);
    MINI_CHECK(result.point_count() == 4);
}

MINI_TEST("Intersection", "Plane 4 Lines") {
    // using session_cpp::Intersection;
    // using session_cpp::Line;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;
    // using session_cpp::Polyline;

    Point pp(0.0, 0.0, 0.0);
    Vector pn(0.0, 0.0, 1.0);
    Plane plane = Plane::from_point_normal(pp, pn);
    Line l0(-1.0, -1.0, -1.0, -1.0, 1.0, 1.0);
    Line l1(1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    Line l2(-1.0, -1.0, -1.0, 1.0, -1.0, 1.0);
    Line l3(-1.0, 1.0, -1.0, 1.0, 1.0, 1.0);
    Polyline result;
    bool ok = Intersection::plane_4lines(plane, l0, l1, l2, l3, result);

    MINI_CHECK(ok);
    MINI_CHECK(result.point_count() == 5);

    for (size_t i = 0; i < result.point_count(); i++) {
        Point p = result.get_point(i);

        MINI_CHECK(std::fabs(p[2]) < 1e-6);
    }
}

MINI_TEST("Intersection", "Line Two Planes") {
    // using session_cpp::Intersection;
    // using session_cpp::Line;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Line line(0.0, 0.0, -5.0, 0.0, 0.0, 5.0);
    Point o0(0.0, 0.0, -1.0);
    Point o1(0.0, 0.0, 2.0);
    Vector n(0.0, 0.0, 1.0);
    Plane plane0 = Plane::from_point_normal(o0, n);
    Plane plane1 = Plane::from_point_normal(o1, n);
    Line output;
    bool ok = Intersection::line_two_planes(line, plane0, plane1, output);

    MINI_CHECK(ok);
    MINI_CHECK(TOLERANCE.is_close(output.start()[2], -1.0));
    MINI_CHECK(TOLERANCE.is_close(output.end()[2], 2.0));
}

MINI_TEST("Intersection", "Scale Vector To Distance Of 2 Planes") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Point p0o(0.0, 0.0, 0.0);
    Vector p0n(0.0, 0.0, 1.0);
    Point p1o(0.0, 0.0, 3.0);
    Vector p1n(0.0, 0.0, 1.0);
    Plane pl0 = Plane::from_point_normal(p0o, p0n);
    Plane pl1 = Plane::from_point_normal(p1o, p1n);
    Vector direction(0.0, 0.0, 1.0);
    Vector result;
    bool ok = Intersection::scale_vector_to_distance_of_2planes(direction, pl0, pl1, result);

    MINI_CHECK(ok);
    MINI_CHECK(std::fabs(result[2] - 3.0) < 1e-6);
}

MINI_TEST("Intersection", "Polyline Plane") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Polyline;
    // using session_cpp::Vector;

    std::vector<Point> pts = {
        Point(-1.0, -1.0, 0.0),
        Point(1.0, -1.0, 0.0),
        Point(1.0, 1.0, 0.0),
        Point(-1.0, 1.0, 0.0),
        Point(-1.0, -1.0, 0.0),
    };

    Polyline poly(pts);
    Point pp(0.0, 0.0, 0.0);
    Vector pn(1.0, 0.0, 0.0);
    Plane plane = Plane::from_point_normal(pp, pn);
    std::vector<Point> points;
    std::vector<int> edge_ids;
    bool result = Intersection::polyline_plane(poly, plane, points, edge_ids);

    MINI_CHECK(result);
    MINI_CHECK(points.size() == 2);

    for (const Point& p : points)
        MINI_CHECK(std::fabs(p[0]) < 1e-9);
}

MINI_TEST("Intersection", "Line Line 3D") {
    // using session_cpp::Intersection;
    // using session_cpp::Line;
    // using session_cpp::Point;

    Line cutter(0.0, 1.0, 0.0, 2.0, 1.0, 0.0);
    Line seg(1.0, 0.0, 0.0, 1.0, 2.0, 0.0);
    Point result;
    bool ok = Intersection::line_line_3d(cutter, seg, result);

    MINI_CHECK(ok);
    MINI_CHECK(std::fabs(result[0] - 1.0) < 1e-6);
    MINI_CHECK(std::fabs(result[1] - 1.0) < 1e-6);
    MINI_CHECK(std::fabs(result[2] - 0.0) < 1e-6);

    Line par0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line par1(0.0, 1.0, 0.0, 1.0, 1.0, 0.0);
    Point out2;

    MINI_CHECK(!Intersection::line_line_3d(par0, par1, out2));
}

MINI_TEST("Intersection", "Polyline Boolean") {
    // using session_cpp::Intersection;
    // using session_cpp::Point;
    // using session_cpp::Polyline;

    Polyline a({
        Point(0.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(2.0, 2.0, 0.0),
        Point(0.0, 2.0, 0.0),
        Point(0.0, 0.0, 0.0),
    });

    Polyline b({
        Point(1.0, 1.0, 0.0),
        Point(3.0, 1.0, 0.0),
        Point(3.0, 3.0, 0.0),
        Point(1.0, 3.0, 0.0),
        Point(1.0, 1.0, 0.0),
    });

    std::vector<Polyline> intersection = Intersection::polyline_boolean(a, b, 0);
    std::vector<Polyline> united = Intersection::polyline_boolean(a, b, 1);
    std::vector<Polyline> difference = Intersection::polyline_boolean(a, b, 2);

    MINI_CHECK(intersection.size() == 1);
    MINI_CHECK(united.size() == 1);
    MINI_CHECK(difference.size() == 1);

    for (size_t i = 0; i < intersection[0].point_count(); i++) {
        Point p = intersection[0].get_point(i);

        MINI_CHECK(p[0] > 1.0 - 1e-9 && p[0] < 2.0 + 1e-9);
        MINI_CHECK(p[1] > 1.0 - 1e-9 && p[1] < 2.0 + 1e-9);
    }
}

MINI_TEST("Intersection", "Offset In 3D") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Polyline;

    Polyline square({
        Point(0.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(2.0, 2.0, 0.0),
        Point(0.0, 2.0, 0.0),
        Point(0.0, 0.0, 0.0),
    });

    Plane plane = Plane::xy_plane();
    bool ok = Intersection::offset_in_3d(square, plane, 0.5);

    MINI_CHECK(ok);
    MINI_CHECK(TOLERANCE.is_close(square.get_point(0)[0], -0.5));
    MINI_CHECK(TOLERANCE.is_close(square.get_point(0)[1], -0.5));

    for (size_t i = 0; i < square.point_count(); i++) {
        Point p = square.get_point(i);

        MINI_CHECK(TOLERANCE.is_close(std::fabs(p[0] - 1.0), 1.5));
        MINI_CHECK(TOLERANCE.is_close(std::fabs(p[1] - 1.0), 1.5));
    }
}

MINI_TEST("Intersection", "Polyline Boolean 2D In Plane") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Polyline;

    Polyline a({
        Point(0.0, 0.0, 1.0),
        Point(2.0, 0.0, 1.0),
        Point(2.0, 2.0, 1.0),
        Point(0.0, 2.0, 1.0),
        Point(0.0, 0.0, 1.0),
    });

    Polyline b({
        Point(1.0, 1.0, 1.0),
        Point(3.0, 1.0, 1.0),
        Point(3.0, 3.0, 1.0),
        Point(1.0, 3.0, 1.0),
        Point(1.0, 1.0, 1.0),
    });

    Plane plane = Plane::xy_plane();
    Polyline result;
    bool ok = Intersection::polyline_boolean_2d_in_plane(a, b, plane, result, 0);

    MINI_CHECK(ok);
    MINI_CHECK(result.point_count() >= 4);

    for (size_t i = 0; i < result.point_count(); i++) {
        Point p = result.get_point(i);

        MINI_CHECK(p[0] > 1.0 - 1e-9 && p[0] < 2.0 + 1e-9);
        MINI_CHECK(p[1] > 1.0 - 1e-9 && p[1] < 2.0 + 1e-9);
        MINI_CHECK(TOLERANCE.is_close(p[2], 1.0));
    }

    Polyline tiny;

    MINI_CHECK(!Intersection::polyline_boolean_2d_in_plane(a, b, plane, tiny, 0, false, 2.0));
}

MINI_TEST("Intersection", "Polyline Plane To Line") {
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Polyline;
    // using session_cpp::Vector;
    // using session_cpp::Intersection::polyline_plane_to_line;
    // using session_cpp::Line;

    Polyline poly({
        Point(0.0, 0.0, 0.0),
        Point(4.0, 0.0, 0.0),
        Point(4.0, 4.0, 0.0),
        Point(0.0, 4.0, 0.0),
        Point(0.0, 0.0, 0.0),
    });

    Point pln_origin(0.0, 2.0, 0.0);
    Vector pln_normal(0.0, 1.0, 0.0);
    Plane pln = Plane::from_point_normal(pln_origin, pln_normal);
    Point align_start(0.0, 0.0, 0.0);
    Line out;
    bool ok = Intersection::polyline_plane_to_line(poly, pln, align_start, out);

    MINI_CHECK(ok);
    MINI_CHECK(TOLERANCE.is_close(out.start()[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(out.end()[0], 4.0));
}

MINI_TEST("Intersection", "Quad From Line Top Bottom Planes") {
    // using session_cpp::Line;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;
    // using session_cpp::Intersection::quad_from_line_top_bottom_planes;
    // using session_cpp::Polyline;

    Plane face = Plane::xy_plane();
    Line line(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    Point p0_o(0.0, -2.0, 0.0);
    Vector p0_n(0.0, 1.0, 0.0);
    Point p1_o(0.0, 2.0, 0.0);
    Vector p1_n(0.0, 1.0, 0.0);
    Plane plane0 = Plane::from_point_normal(p0_o, p0_n);
    Plane plane1 = Plane::from_point_normal(p1_o, p1_n);
    Polyline out;
    bool ok = Intersection::quad_from_line_top_bottom_planes(face, line, plane0, plane1, out);

    MINI_CHECK(ok);
    MINI_CHECK(out.point_count() == 5);
    MINI_CHECK(TOLERANCE.is_close(std::abs(out.get_point(0)[1]), 2.0));
    MINI_CHECK(TOLERANCE.is_close(std::abs(out.get_point(2)[1]), 2.0));
    MINI_CHECK(TOLERANCE.is_close(out.get_point(2)[0], 10.0));
}

MINI_TEST("Intersection", "Orthogonal Vector Between Two Plane Pairs") {
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;
    // using session_cpp::Intersection::orthogonal_vector_between_two_plane_pairs;

    Plane pp00 = Plane::xy_plane();
    Point yz0_o(0.0, 0.0, 0.0);
    Vector yz_n(1.0, 0.0, 0.0);
    Plane pp10 = Plane::from_point_normal(yz0_o, yz_n);
    Point yz4_o(4.0, 0.0, 0.0);
    Vector yz_n2(1.0, 0.0, 0.0);
    Plane pp11 = Plane::from_point_normal(yz4_o, yz_n2);
    Vector out;
    bool ok = Intersection::orthogonal_vector_between_two_plane_pairs(pp00, pp10, pp11, out);

    MINI_CHECK(ok);

    double mag = std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);

    MINI_CHECK(TOLERANCE.is_close(mag, 4.0));
    MINI_CHECK(TOLERANCE.is_close(out[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(out[2], 0.0));
}

MINI_TEST("Intersection", "Closed And Open Paths 2D") {
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Polyline;
    // using session_cpp::Intersection::closed_and_open_paths_2d;

    Polyline plate({
        Point(0.0, 0.0, 0.0),
        Point(10.0, 0.0, 0.0),
        Point(10.0, 10.0, 0.0),
        Point(0.0, 10.0, 0.0),
        Point(0.0, 0.0, 0.0),
    });

    Polyline joint({
        Point(-2.0, 5.0, 0.0),
        Point(12.0, 5.0, 0.0),
    });

    Plane pln = Plane::xy_plane();
    Polyline out;
    std::pair<double, double> cp_pair;
    bool ok = Intersection::closed_and_open_paths_2d(plate, joint, pln, out, cp_pair);

    MINI_CHECK(ok);
    MINI_CHECK(out.point_count() == 2);
    MINI_CHECK(TOLERANCE.is_close(out.get_point(0)[1], 5.0));
    MINI_CHECK(TOLERANCE.is_close(out.get_point(1)[1], 5.0));

    double t_lo = std::min(cp_pair.first, cp_pair.second);
    double t_hi = std::max(cp_pair.first, cp_pair.second);

    MINI_CHECK(TOLERANCE.is_close(t_lo, 1.5));
    MINI_CHECK(TOLERANCE.is_close(t_hi, 3.5));
}

MINI_TEST("Intersection", "Face To Face") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Polyline;
    // using session_cpp::Vector;

    std::vector<std::vector<Polyline>> polylines = {
        {
            Polyline({
                Point(0.0, 0.0, 0.0),
                Point(2.0, 0.0, 0.0),
                Point(2.0, 1.0, 0.0),
                Point(0.0, 1.0, 0.0),
                Point(0.0, 0.0, 0.0),
            }),
            Polyline({
                Point(0.0, 0.0, 1.0),
                Point(2.0, 0.0, 1.0),
                Point(2.0, 1.0, 1.0),
                Point(0.0, 1.0, 1.0),
                Point(0.0, 0.0, 1.0),
            }),
        },
        {
            Polyline({
                Point(1.0, 0.5, 1.0),
                Point(3.0, 0.5, 1.0),
                Point(3.0, 1.5, 1.0),
                Point(1.0, 1.5, 1.0),
                Point(1.0, 0.5, 1.0),
            }),
            Polyline({
                Point(1.0, 0.5, 2.0),
                Point(3.0, 0.5, 2.0),
                Point(3.0, 1.5, 2.0),
                Point(1.0, 1.5, 2.0),
                Point(1.0, 0.5, 2.0),
            }),
        },
    };

    Point o00(1.0, 0.5, 0.0);
    Point o01(1.0, 0.5, 1.0);
    Point o10(2.0, 1.0, 1.0);
    Point o11(2.0, 1.0, 2.0);
    Vector down(0.0, 0.0, -1.0);
    Vector up(0.0, 0.0, 1.0);

    std::vector<std::vector<Plane>> planes = {
        {Plane::from_point_normal(o00, down), Plane::from_point_normal(o01, up)},
        {Plane::from_point_normal(o10, down), Plane::from_point_normal(o11, up)},
    };

    std::vector<int> adjacency = {0, 1, -1, -1};
    std::vector<std::tuple<int, int, int, int, int, Polyline>> contacts = Intersection::face_to_face(adjacency, polylines, planes, 0.01);

    MINI_CHECK(contacts.size() == 1);
    MINI_CHECK(std::get<0>(contacts[0]) == 0);
    MINI_CHECK(std::get<1>(contacts[0]) == 1);
    MINI_CHECK(std::get<2>(contacts[0]) == 1);
    MINI_CHECK(std::get<3>(contacts[0]) == 0);
    MINI_CHECK(std::get<4>(contacts[0]) == 2);
    MINI_CHECK(std::get<5>(contacts[0]).is_closed());

    for (size_t i = 0; i < std::get<5>(contacts[0]).point_count(); i++) {
        Point p = std::get<5>(contacts[0]).get_point(i);

        MINI_CHECK(p[0] > 1.0 - 1e-9 && p[0] < 2.0 + 1e-9);
        MINI_CHECK(p[1] > 0.5 - 1e-9 && p[1] < 1.0 + 1e-9);
        MINI_CHECK(TOLERANCE.is_close(p[2], 1.0));
    }
}

MINI_TEST("Intersection", "Adjacency Search") {
    // using session_cpp::Intersection;
    // using session_cpp::Element;
    // using session_cpp::Mesh;
    // using session_cpp::Point;

    Element a(Mesh::from_polylines({
        {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        },
    }));

    Element b(Mesh::from_polylines({
        {
            Point(1.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(2.0, 1.0, 0.0),
            Point(1.0, 1.0, 0.0),
        },
    }));

    Element c(Mesh::from_polylines({
        {
            Point(5.0, 0.0, 0.0),
            Point(6.0, 0.0, 0.0),
            Point(6.0, 1.0, 0.0),
            Point(5.0, 1.0, 0.0),
        },
    }));

    std::vector<Element*> elements = {&a, &b, &c};
    std::vector<int> adjacency = Intersection::adjacency_search(elements, 0.01);

    MINI_CHECK(adjacency.size() == 4);
    MINI_CHECK(adjacency[0] == 0);
    MINI_CHECK(adjacency[1] == 1);
    MINI_CHECK(adjacency[2] == -1);
    MINI_CHECK(adjacency[3] == -1);
}

MINI_TEST("Intersection", "Line Line Classified") {
    // using session_cpp::Line;
    // using session_cpp::Intersection::line_line_classified;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    Line s0(-1.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line s1(0.0, -1.0, 0.0, 0.0, 1.0, 0.0);
    Point p0;
    Point p1;
    Vector v0;
    Vector v1;
    Vector normal;
    bool type0 = false;
    bool type1 = false;
    bool is_parallel = false;
    bool ok =
        Intersection::line_line_classified(s0, s1, 1, 1, 0, 0, 0.5, p0, p1, v0, v1, normal, type0, type1, is_parallel);

    MINI_CHECK(ok);
    MINI_CHECK(!is_parallel);
    MINI_CHECK(std::abs(p0[0]) < 1e-6);
    MINI_CHECK(std::abs(p0[1]) < 1e-6);
    MINI_CHECK(std::abs(p1[0]) < 1e-6);
    MINI_CHECK(std::abs(p1[1]) < 1e-6);
    MINI_CHECK(std::abs(std::abs(normal[2]) - 1.0) < 1e-6);

    Line e0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line e1(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    bool ok2 =
        Intersection::line_line_classified(e0, e1, 1, 1, 0, 0, 0.5, p0, p1, v0, v1, normal, type0, type1, is_parallel);

    MINI_CHECK(ok2);
    MINI_CHECK(!type0);
    MINI_CHECK(!type1);
    MINI_CHECK(std::abs(p0[0]) < 1e-6);
    MINI_CHECK(std::abs(p0[1]) < 1e-6);

    Line q0(0.0, 0.0, 0.0, 2.0, 0.0, 0.0);
    Line q1(0.0, 1.0, 0.0, 2.0, 1.0, 0.0);
    bool ok3 =
        Intersection::line_line_classified(q0, q1, 1, 1, 0, 0, 0.5, p0, p1, v0, v1, normal, type0, type1, is_parallel);

    MINI_CHECK(ok3);
    MINI_CHECK(is_parallel);
    MINI_CHECK(!type0);
    MINI_CHECK(!type1);
}

} // namespace session_cpp
