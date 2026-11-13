#include <iostream>
#include "mini_test.h"
#include "intersection.h"
#include "file_step.h"
#include "curve_surface_intersection.h"
#include "closest.h"
#include "nurbs_hermite.h"
#include "nurbssurface.h"
#include "primitives.h"
#include "xform.h"
#include "tolerance.h"
#include <cmath>
#include <filesystem>
#include <chrono>
#include <cstdlib>
#include <fstream>

using namespace session_cpp::mini_test;

namespace session_cpp {

static void save_section_geometry(
    const std::string& name,
    const NurbsSurface& a,
    const NurbsSurface& b,
    const std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>>& sections
) {
    const std::filesystem::path folder = "build/native-geometry-tests";
    std::filesystem::create_directories(folder);
    nlohmann::ordered_json report;
    report["surfaces"] = {a.jsondump(), b.jsondump()};
    report["surface_samples"] = nlohmann::ordered_json::array();
    for (const NurbsSurface* surface : {&a, &b}) {
        nlohmann::ordered_json grid = nlohmann::ordered_json::array();
        for (int i = 0; i <= 32; ++i) {
            nlohmann::ordered_json row = nlohmann::ordered_json::array();
            for (int j = 0; j <= 32; ++j) {
                const std::pair<double, double> u = surface->domain(0);
                const std::pair<double, double> v = surface->domain(1);
                const Point p = surface->point_at(u.first + (u.second - u.first) * i / 32.0,
                                                v.first + (v.second - v.first) * j / 32.0);
                row.push_back({p[0], p[1], p[2]});
            }
            grid.push_back(row);
        }
        report["surface_samples"].push_back(grid);
    }
    report["sections"] = nlohmann::ordered_json::array();
    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& section : sections) {
        nlohmann::ordered_json samples = nlohmann::ordered_json::array();
        double gap = 0.0;
        for (int i = 0; i <= 256; ++i) {
            const double t = i / 256.0;
            const Point p = std::get<0>(section).point_at(t);
            const Point ua = std::get<1>(section).point_at(t);
            const Point ub = std::get<2>(section).point_at(t);
            gap = std::max({gap, p.distance(a.point_at(ua[0], ua[1])), p.distance(b.point_at(ub[0], ub[1]))});
            samples.push_back({p[0], p[1], p[2]});
        }
        report["sections"].push_back({{"curve", std::get<0>(section).jsondump()},
            {"length", std::get<0>(section).length()}, {"max_sampled_gap", gap}, {"samples", samples}});
    }
    std::ofstream(folder / (name + ".sections.json")) << report.dump(2);
}

MINI_TEST("Intersection", "Surface Surface Freeform Pcurves") {
    std::vector<Point> first_points, second_points;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            const double height = 0.15 * (i - 1.5) * (j - 1.5);
            first_points.push_back({(double)i, (double)j, height});
            second_points.push_back({(double)i, (double)j, 0.1 - height});
        }
    const NurbsSurface a = NurbsSurface::create(false, false, 3, 3, 4, 4, first_points);
    const NurbsSurface b = NurbsSurface::create(false, false, 3, 3, 4, 4, second_points);
    const std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> sections = Intersection::surface_surface(a, b, 1e-7);
    save_section_geometry("freeform_pcurves", a, b, sections);
    MINI_CHECK(sections.size() == 2);
    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& section : sections)
        for (int i = 0; i <= 64; ++i) {
            const double parameter = i / 64.0;
            const Point point = std::get<0>(section).point_at(parameter);
            const Point ua = std::get<1>(section).point_at(parameter);
            const Point ub = std::get<2>(section).point_at(parameter);
            MINI_CHECK(point.distance(a.point_at(ua[0], ua[1])) < 2e-6);
            MINI_CHECK(point.distance(b.point_at(ub[0], ub[1])) < 2e-6);
        }
}

MINI_TEST("Intersection", "Surface Surface Small Interior Loop") {
    std::vector<Point> points;
    const std::array<double, 3> heights = {1.0, -1.0, 1.0};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            points.push_back({i - 1.0, j - 1.0, heights[i] + heights[j]});
    const NurbsSurface bowl = NurbsSurface::create(false, false, 2, 2, 3, 3, points);
    const NurbsSurface plane = NurbsSurface::create(
        false, false, 1, 1, 2, 2, {{-1, -1, 0.01}, {-1, 1, 0.01}, {1, -1, 0.01}, {1, 1, 0.01}}
    );
    const std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> sections = Intersection::surface_surface(bowl, plane, 1e-7);
    save_section_geometry("small_interior_loop", bowl, plane, sections);
    MINI_CHECK(!sections.empty());
    double length = 0.0;
    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& section : sections) {
        length += std::get<0>(section).length();
        for (int i = 0; i <= 256; ++i) {
            const Point p = std::get<0>(section).point_at(i / 256.0);
            const Point uv = std::get<1>(section).point_at(i / 256.0);
            MINI_CHECK(p.distance(bowl.point_at(uv[0], uv[1])) < 2e-6);
        }
    }
    MINI_CHECK(std::abs(length - 0.2 * Tolerance::PI) < 1e-5);
}

MINI_TEST("Intersection", "Curve Curve Crossings And Overlaps") {
    const NurbsCurve a = NurbsCurve::create(false, 1, {{-2, 0, 0}, {2, 0, 0}});
    const NurbsCurve b = NurbsCurve::create(false, 1, {{0, -2, 0}, {0, 2, 0}});
    const SectionResult crossing = Intersection::curve_curve(a, b);
    MINI_CHECK(crossing.points.size() == 1);
    MINI_CHECK(crossing.points[0].distance({0, 0, 0}) < 1e-7);
    MINI_CHECK(crossing.curves.empty());

    const NurbsCurve overlapping = NurbsCurve::create(false, 1, {{-1, 0, 0}, {3, 0, 0}});
    const SectionResult overlap = Intersection::curve_curve(a, overlapping);
    MINI_CHECK(overlap.points.empty());
    MINI_CHECK(overlap.curves.size() == 1);
    MINI_CHECK(std::abs(overlap.curves[0].length() - 3.0) < 1e-6);

    const NurbsCurve skew = b.transformed(Xform::translation(0, 0, 0.01));
    const SectionResult miss = Intersection::curve_curve(a, skew);
    MINI_CHECK(miss.points.empty());
    MINI_CHECK(miss.curves.empty());
}

MINI_TEST("Intersection", "Curve Surface Crossings And Tangency") {
    const NurbsSurface surface = NurbsSurface::create(
        false, false, 1, 1, 2, 2, {{-2, -2, 0}, {-2, 2, 0}, {2, -2, 0}, {2, 2, 0}}
    );
    const NurbsCurve line = NurbsCurve::create(false, 1, {{0, 0, -1}, {0, 0, 1}});
    const SectionResult crossing = Intersection::curve_surface(line, surface);
    MINI_CHECK(crossing.points.size() == 1);
    MINI_CHECK(crossing.points[0].distance({0, 0, 0}) < 1e-7);
    const NurbsCurve tangent = NurbsCurve::create(false, 2, {{-1, 0, 1}, {0, 0, -1}, {1, 0, 1}});
    const SectionResult contact = Intersection::curve_surface(tangent, surface);
    MINI_CHECK(contact.points.size() == 1);
    MINI_CHECK(contact.points[0].distance({0, 0, 0}) < 1e-5);
    const NurbsCurve coplanar = NurbsCurve::create(false, 1, {{-3, 0, 0}, {3, 0, 0}});
    const SectionResult overlap = Intersection::curve_surface(coplanar, surface);
    MINI_CHECK(overlap.curves.size() == 1);
    MINI_CHECK(std::abs(overlap.curves[0].length() - 4.0) < 1e-5);
}

MINI_TEST("Intersection", "Curve BRep Trimmed Boundaries") {
    const BRep bore = BRep::create_block_with_hole(4.0, 4.0, 2.0, 0.5);
    const NurbsCurve axis = NurbsCurve::create(false, 1, {{0, 0, -1}, {0, 0, 3}});
    const SectionResult hole = Intersection::curve_brep(axis, bore);
    MINI_CHECK(hole.points.empty());
    MINI_CHECK(hole.curves.empty());
    const NurbsCurve crossing = NurbsCurve::create(false, 1, {{-3, 0, 0}, {3, 0, 0}});
    const SectionResult cuts = Intersection::curve_brep(crossing, bore);
    MINI_CHECK(cuts.points.size() == 4);
    MINI_CHECK(cuts.curves.empty());
}

/// Save samples and computed roots from a curve-plane regression.
static void save_curve_geometry(
    const std::string& name,
    const NurbsCurve& curve,
    const std::vector<double>& roots
) {

    const std::filesystem::path folder = std::filesystem::path("build") / "geometry-tests";
    std::filesystem::create_directories(folder);
    nlohmann::ordered_json data;
    data["curve"] = curve.jsondump();
    data["roots"] = roots;
    data["samples"] = nlohmann::ordered_json::array();
    for (int i = 0; i <= 1024; ++i) {
        const double t = curve.domain_start() + (curve.domain_end() - curve.domain_start()) * i / 1024.0;
        const Point point = curve.point_at(t);
        data["samples"].push_back({t, point[0], point[1], point[2]});
    }
    data["points"] = nlohmann::ordered_json::array();
    for (double t : roots) {
        const Point point = curve.point_at(t);
        data["points"].push_back({point[0], point[1], point[2]});
    }

    std::ofstream stream(folder / (name + ".curve.json"));
    stream << data.dump(2);
}

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

MINI_TEST("Intersection", "Curve Plane Tangency") {
    const double root = 0.137;
    const Plane plane = Plane::from_point_normal({0, 0, 0}, {0, 0, 1});
    NurbsCurve curve = NurbsCurve::create(false, 2, {
        {0.0, 0.0, root * root},
        {0.5, 0.0, root * root - root},
        {1.0, 0.0, (1.0 - root) * (1.0 - root)}
    });
    curve.set_domain(0.0, 1.0);
    const std::vector<double> roots = Intersection::curve_plane(curve, plane);

    MINI_CHECK(roots.size() == 1);
    MINI_CHECK(std::abs(roots[0] - root) < 1e-12);

    save_curve_geometry("curve_plane_tangency", curve, roots);

    curve.set_cv(1, {0.5, 0.0, root * root - root + 0.01});

    MINI_CHECK(Intersection::curve_plane(curve, plane).empty());
}

MINI_TEST("Intersection", "Curve Plane Hidden Crossings") {
    const Plane plane = Plane::from_point_normal({0, 0, 0}, {0, 0, 1});
    const double d = -0.1 * 0.12 * 0.8;
    const double c = 0.1 * 0.12 + 0.1 * 0.8 + 0.12 * 0.8;
    const double b = -(0.1 + 0.12 + 0.8);
    NurbsCurve curve = NurbsCurve::create(false, 3, {
        {0.0, 0.0, d},
        {1.0 / 3.0, 0.0, d + c / 3.0},
        {2.0 / 3.0, 0.0, d + 2.0 * c / 3.0 + b / 3.0},
        {1.0, 0.0, d + c + b + 1.0}
    });
    curve.set_domain(0.0, 1.0);
    const std::vector<double> roots = Intersection::curve_plane(curve, plane);

    MINI_CHECK(roots.size() == 3);
    MINI_CHECK(std::abs(roots[0] - 0.1) < 1e-12);
    MINI_CHECK(std::abs(roots[1] - 0.12) < 1e-12);
    MINI_CHECK(std::abs(roots[2] - 0.8) < 1e-12);
    MINI_CHECK(Intersection::curve_plane_algebraic(curve, plane).size() == 3);
    MINI_CHECK(Intersection::curve_plane_production(curve, plane).size() == 3);
    MINI_CHECK(Intersection::curve_plane_bezier_clipping(curve, plane).size() == 3);

    save_curve_geometry("curve_plane_hidden_crossings", curve, roots);

    for (double& knot : curve.m_nurbsknot)
        knot *= 1e-10;
    const std::vector<double> scaled = Intersection::curve_plane(curve, plane);

    MINI_CHECK(scaled.size() == 3);
    MINI_CHECK(std::abs(scaled[0] / 1e-10 - 0.1) < 1e-12);
    MINI_CHECK(std::abs(scaled[1] / 1e-10 - 0.12) < 1e-12);
    MINI_CHECK(std::abs(scaled[2] / 1e-10 - 0.8) < 1e-12);

    save_curve_geometry("curve_plane_small_domain", curve, scaled);
}

MINI_TEST("Intersection", "Curve Plane Rational Tangency") {
    const Plane plane = Plane::from_point_normal({0, 0, 0}, {0, 0, 1});
    NurbsCurve curve(3, true, 3, 3);
    curve.m_nurbsknot = {0.0, 0.0, 1.0, 1.0};
    curve.set_cv_4d(0, 0.0, 0.0, 0.25, 1.0);
    curve.set_cv_4d(1, 1.0, 0.0, -0.25, 2.0);
    curve.set_cv_4d(2, 1.0, 0.0, 0.25, 1.0);
    const std::vector<double> roots = Intersection::curve_plane(curve, plane);

    MINI_CHECK(roots.size() == 1);
    MINI_CHECK(std::abs(roots[0] - 0.5) < 1e-12);

    save_curve_geometry("curve_plane_rational_tangency", curve, roots);
}

MINI_TEST("Intersection", "Curve Plane Coplanar Span") {
    const Plane plane = Plane::from_point_normal({0, 0, 0}, {0, 0, 1});
    const NurbsCurve curve = NurbsCurve::create(false, 1, {
        {0, 0, 1}, {1, 0, 0}, {2, 0, 0}, {3, 0, 1}
    });
    const std::vector<double> spans = curve.get_span_vector();
    const std::vector<double> roots = Intersection::curve_plane(curve, plane);

    MINI_CHECK(roots.size() == 2);
    MINI_CHECK(roots[0] == spans[1]);
    MINI_CHECK(roots[1] == spans[2]);

    save_curve_geometry("curve_plane_coplanar_span", curve, roots);
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
    std::cerr << "[analytic-pullback-case] Surface Surface Accuracy :: std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> tr = Intersection::surface_surface(sphere, cyl);" << std::endl;
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> tr = Intersection::surface_surface(sphere, cyl);

    MINI_CHECK(tr.size() >= 2);

    for (std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : tr)
        MINI_CHECK(on_both(std::get<0>(t), distance_sphere, distance_cylinder) < 1e-5);

    NurbsSurface sphere2 = Primitives::sphere_surface(2.0, 0.0, 0.0, 2.0);
    std::cerr << "[analytic-pullback-case] Surface Surface Accuracy :: std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> tr2 = Intersection::surface_surface(sphere, sphere2);" << std::endl;
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

    std::cerr << "[analytic-pullback-case] Surface Surface Accuracy :: std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> tr3 = Intersection::surface_surface(torus, flat);" << std::endl;
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
    std::cerr << "[analytic-pullback-case] Surface Surface Plane Cone :: std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> circle = Intersection::surface_surface(cone, flat);" << std::endl;
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> circle = Intersection::surface_surface(cone, flat);

    MINI_CHECK(circle.size() == 1);
    MINI_CHECK(on_both(std::get<0>(circle[0]), distance_cone, distance_flat_half) < 1e-9);
    MINI_CHECK(lies_on_curve(std::get<0>(circle[0]), std::get<1>(circle[0]), cone) < 1e-9);

    std::cerr << "[analytic-pullback-case] Surface Surface Plane Cone :: std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> hyperbola = Intersection::surface_surface(cone, steep);" << std::endl;
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> hyperbola = Intersection::surface_surface(cone, steep);

    MINI_CHECK(hyperbola.size() == 1);
    MINI_CHECK(std::get<0>(hyperbola[0]).degree() == 2);
    MINI_CHECK(on_both(std::get<0>(hyperbola[0]), distance_cone, distance_cone) < 1e-9);

    std::cerr << "[analytic-pullback-case] Surface Surface Plane Cone :: std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> parabola = Intersection::surface_surface(cone, slant);" << std::endl;
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> parabola = Intersection::surface_surface(cone, slant);

    MINI_CHECK(parabola.size() == 1);
    MINI_CHECK(on_both(std::get<0>(parabola[0]), distance_cone, distance_cone) < 1e-6);

    std::cerr << "[analytic-pullback-case] Surface Surface Plane Cone :: std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> lines = Intersection::surface_surface(cone, axial);" << std::endl;
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

MINI_TEST("Intersection", "Surface Plane Loop Seeded On The Torus Seam") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::Vector;
    // using session_cpp::NurbsSurface;
    // using session_cpp::NurbsCurve;

    // A plane parallel to the axis cuts an oval about the outer equator, the tube seam: the trace starts on the seam and is cut there too.
    NurbsSurface torus = Primitives::torus_surface(0.0, 0.0, 0.0, 0.87278740435838708, 0.32987454484682532);
    Plane wall = Plane::from_point_normal(Point(0.0, 0.98669363572262236, 0.0), Vector(0.0, 1.0, 0.0));
    std::vector<std::pair<NurbsCurve, NurbsCurve>> sections = Intersection::surface_plane_uv(torus, wall, 1e-8);

    MINI_CHECK(sections.size() == 2);

    for (const std::pair<NurbsCurve, NurbsCurve>& section : sections) {
        for (int i = 0; i <= 64; ++i) {
            const double t = section.first.domain_start() + (section.first.domain_end() - section.first.domain_start()) * i / 64.0;
            const Point world = section.first.point_at(t);
            const Point uv = section.second.point_at(t);
            MINI_CHECK(uv[0] >= -1e-9 && uv[0] <= 4.0 + 1e-9 && uv[1] >= -1e-9 && uv[1] <= 4.0 + 1e-9);
            MINI_CHECK(torus.point_at(uv[0], uv[1]).distance(world) < 1e-7);
            MINI_CHECK(std::abs(world[1] - 0.98669363572262236) < 1e-7);
        }
    }
}

MINI_TEST("Intersection", "Surface Plane Pieces End On The Torus Seam") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::Vector;
    // using session_cpp::NurbsSurface;
    // using session_cpp::NurbsCurve;

    // Tilted planes cross both seams: a piece cut on a seam ends on it, not a hair outside the domain.
    NurbsSurface torus = Primitives::torus_surface(0.0, 0.0, 0.0, 0.87278740435838708, 0.32987454484682532);
    int pieces = 0;

    for (const Vector& normal : {Vector(0.31, 0.89, 0.33), Vector(-0.52, 0.27, 0.81), Vector(0.12, -0.35, 0.93)}) {
        Plane cut = Plane::from_point_normal(Point(0.21, -0.08, 0.05), normal);

        for (const std::pair<NurbsCurve, NurbsCurve>& section : Intersection::surface_plane_uv(torus, cut, 1e-8)) {
            ++pieces;
            for (double t : {section.second.domain_start(), section.second.domain_end()}) {
                const Point uv = section.second.point_at(t);
                MINI_CHECK(uv[0] >= 0.0 && uv[0] <= 4.0 && uv[1] >= 0.0 && uv[1] <= 4.0);
                MINI_CHECK(torus.point_at(uv[0], uv[1]).distance(section.first.point_at(t)) < 1e-7);
            }
        }
    }

    MINI_CHECK(pieces > 6);
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
    std::cerr << "[analytic-pullback-case] Surface Surface Cylinders :: std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> lines = Intersection::surface_surface(cyl, beside);" << std::endl;
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> lines = Intersection::surface_surface(cyl, beside);

    MINI_CHECK(lines.size() == 2);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& line : lines) {
        Point start = std::get<0>(line).point_at_start();
        Point end = std::get<0>(line).point_at_end();

        MINI_CHECK(TOLERANCE.is_close(start[0], 0.75) && TOLERANCE.is_close(end[0], 0.75));
        MINI_CHECK(TOLERANCE.is_close(std::abs(start[1]), std::sqrt(0.4375)) && TOLERANCE.is_close(start[1], end[1]));
        MINI_CHECK(lies_on_curve(std::get<0>(line), std::get<1>(line), cyl) < 1e-9);
    }

    std::cerr << "[analytic-pullback-case] Surface Surface Cylinders :: std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> ellipses = Intersection::surface_surface(cyl, across);" << std::endl;
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> ellipses = Intersection::surface_surface(cyl, across);

    MINI_CHECK(ellipses.size() == 3);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& ellipse : ellipses) {
        MINI_CHECK(on_both(std::get<0>(ellipse), distance_unit_cylinder, distance_x_cylinder) < 1e-9);
        MINI_CHECK(pcurve_end_gap(std::get<0>(ellipse), std::get<1>(ellipse), cyl) < 1e-4);
        MINI_CHECK(pcurve_end_gap(std::get<0>(ellipse), std::get<2>(ellipse), across) < 1e-4);
    }
}

MINI_TEST("Intersection", "Plane Cone Cap Clipping Preserves Exact Conic") {
    const double tolerance = 1e-12;
    const auto cone = Primitives::cone_surface(0, 0, 0, 1.5, 3);
    const auto plane = bilinear(Point(-3, -3, 0), Point(-3, 3, -.6), Point(3, -3, 1.2), Point(3, 3, .6));
    const auto before_cone = cone.jsondump(), before_plane = plane.jsondump();
    for (bool swap : {false, true}) {
        const auto& first = swap ? plane : cone;
        const auto& second = swap ? cone : plane;
        const auto result = Intersection::surface_surface(first, second, tolerance);
        MINI_CHECK(result.size() == 2);
        int base_a = 0, base_b = 0, seam = 0;
        for (const auto& section : result) {
            const auto& world = std::get<0>(section);
            const auto& ua = std::get<1>(section);
            const auto& ub = std::get<2>(section);
            MINI_CHECK(world.is_rational() && world.degree() == 2);
            MINI_CHECK(world.domain() == ua.domain() && world.domain() == ub.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(world, ua, first, tolerance) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world, ub, second, tolerance) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            const auto domain = world.domain();
            for (const auto& point : {world.point_at_start(), world.point_at_end()}) {
                base_a += point.distance({-.9, 1.2, 0}) < 1e-10;
                base_b += point.distance({-1.5, 0, 0}) < 1e-10;
                seam += point.distance({27. / 22, 0, 6. / 11}) < 1e-10;
            }
            for (int sample = 0; sample <= 32; ++sample) {
                const auto point = world.point_at(std::lerp(domain.first, domain.second, sample / 32.));
                const double radius = 1.5 - point[2] * .5;
                MINI_CHECK(std::abs(point[0] * point[0] + point[1] * point[1] - radius * radius) < 1e-11);
                MINI_CHECK(std::abs(point[2] - .2 * point[0] + .1 * point[1] - .3) < 1e-11);
                MINI_CHECK(point[2] >= -tolerance && point[2] <= 3 + tolerance);
            }
        }
        if (!(base_a == 1 && base_b == 1 && seam == 2))
            if (const char* sink = std::getenv("SESSION_EXACT_CONIC_FAILURE_DIR"); sink && *sink)
                try {
                    const std::filesystem::path root(sink);
                    if (!root.is_absolute() || !std::filesystem::is_directory(root))
                        throw std::runtime_error("Exact-conic observation requires an existing absolute external directory");
                    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
                    const auto folder = root / ("exact-conic-failure-" + std::to_string(stamp));
                    if (!std::filesystem::create_directory(folder))
                        throw std::runtime_error("Cannot exclusively create exact-conic failure directory");
                    auto sections = nlohmann::ordered_json::array();
                    for (const auto& [world, pa, pb] : result) {
                        auto ends = nlohmann::ordered_json::array();
                        for (double t : {world.domain_start(), world.domain_end()}) {
                            const auto p = world.point_at(t), ua = pa.point_at(t), ub = pb.point_at(t);
                            const auto a = first.point_at(ua[0], ua[1]), b = second.point_at(ub[0], ub[1]);
                            ends.push_back({{"raw_parameter",t},{"world",{p[0],p[1],p[2]}},
                                {"pa",{ua[0],ua[1]}},{"pb",{ub[0],ub[1]}},
                                {"lift_a",{a[0],a[1],a[2]}},{"lift_b",{b[0],b[1],b[2]}}});
                        }
                        sections.push_back({{"world",world.jsondump()},{"pa",pa.jsondump()},
                            {"pb",pb.jsondump()},{"endpoints",ends}});
                    }
                    const nlohmann::ordered_json row = {
                        {"schema","native_exact_conic_endpoint_counts_failure_v1"},
                        {"swap",swap},{"tolerance",tolerance},{"base_a",base_a},{"base_b",base_b},
                        {"seam",seam},{"section_count",result.size()},
                        {"surface_a",first.jsondump()},{"surface_b",second.jsondump()},{"sections",sections}};
                    const std::string bytes = row.dump(2);
                    if (bytes.size() > 8*1024*1024)
                        throw std::runtime_error("Exact-conic observation exceeds diagnostic size bound");
                    const auto path = folder / "failure.json";
                    std::ofstream output(path,std::ios::binary|std::ios::out);
                    if (!output || !(output << bytes << '\n'))
                        throw std::runtime_error("Cannot write exclusive exact-conic observation");
                    output.flush();
                    if (!output) throw std::runtime_error("Cannot flush exact-conic observation");
                    output.close();
                    if (!output) throw std::runtime_error("Cannot close exact-conic observation");
                    std::cerr << "[exact-conic-failure] " << path.string() << std::endl;
                } catch (const std::exception& error) {
                    std::cerr << "Exact-conic observation failed: " << error.what() << std::endl;
                }
        MINI_CHECK(base_a == 1 && base_b == 1 && seam == 2);
    }
    MINI_CHECK(cone.jsondump() == before_cone && plane.jsondump() == before_plane);
}

MINI_TEST("Intersection", "Near Cap Positive Conic Arc Is Not Discarded") {
    const double tolerance = 1e-12, rise = std::ldexp(1., -46);
    const auto cone = Primitives::cone_surface(0, 0, 0, 1.5, 3);
    const auto plane = bilinear(Point(-3, -3, -4.5 + rise), Point(-3, 3, -4.5 + rise),
                                Point(3, -3, 1.5 + rise), Point(3, 3, 1.5 + rise));
    const auto result = Intersection::surface_surface(cone, plane, tolerance);
    MINI_CHECK(!result.empty());
    double low = 1, high = -1, peak = 0;
    for (const auto& section : result) {
        const auto& world = std::get<0>(section);
        MINI_CHECK(world.is_rational() && world.degree() == 2);
        MINI_CHECK(world.domain() == std::get<1>(section).domain() &&
                   world.domain() == std::get<2>(section).domain());
        MINI_CHECK(curve_surface_intersection::same_parameter(world, std::get<1>(section), cone, tolerance) ==
                   curve_surface_intersection::ParameterMatch::Matched);
        MINI_CHECK(
            curve_surface_intersection::same_parameter(world, std::get<2>(section), plane, tolerance) ==
            curve_surface_intersection::ParameterMatch::Matched);
        const auto domain = world.domain();
        for (int i = 0; i <= 32; ++i) {
            const auto point = world.point_at(std::lerp(domain.first, domain.second, i / 32.));
            low = std::min(low, point[1]);
            high = std::max(high, point[1]);
            peak = std::max(peak, point[2]);
            MINI_CHECK(point[2] >= -tolerance);
        }
    }
    // The positive cap arc has O(sqrt(rise)) tangential length although its
    // entire axial rise is smaller than the requested model tolerance.
    MINI_CHECK(high - low > 1e-8 && peak > rise * .125);
}

MINI_TEST("Intersection", "World Rational Parameter Law Has Certified Pullback") {
    const double tolerance = 1e-12;
    for (int chart = 0; chart < 2; ++chart) {
        auto cone = Primitives::cone_surface(0, 0, 0, 1.5, 3);
        if (chart) {
            MINI_CHECK(cone.set_domain(0, -7, 13));
            MINI_CHECK(cone.set_domain(1, 32, 35));
        }
        const auto plane =
            bilinear(Point(1.1, -3, -3), Point(1.1, 3, -3), Point(-.3, -3, 4), Point(-.3, 3, 4));
        const auto before_cone = cone.jsondump(), before_plane = plane.jsondump();
        for (bool swap : {false, true}) {
            const auto& a = swap ? plane : cone;
            const auto& b = swap ? cone : plane;
            const auto result = Intersection::surface_surface(a, b, tolerance);
            MINI_CHECK(result.size() == 1);
            for (const auto& section : result) {
                const auto& world = std::get<0>(section);
                const auto& ua = std::get<1>(section);
                const auto& ub = std::get<2>(section);
                MINI_CHECK(world.is_rational() && world.degree() == 2 && world.cv_count() == 3);
                MINI_CHECK(world.domain_start() == 0 && world.domain_end() == 1);
                MINI_CHECK(ua.domain() == world.domain() && ub.domain() == world.domain());
                MINI_CHECK(curve_surface_intersection::same_parameter(world, ua, a, tolerance) ==
                           curve_surface_intersection::ParameterMatch::Matched);
                MINI_CHECK(curve_surface_intersection::same_parameter(world, ub, b, tolerance) ==
                           curve_surface_intersection::ParameterMatch::Matched);
                MINI_CHECK(world.weight(1) > 12.5 && world.weight(1) < 13.5);
                // Rational conic of x+.2*z=.5 and x*x+y*y=(1.5-.5*z)^2.
                // Its supplied raw parameter law has denominator1+24*t*(1-t).
                for (int i = 0; i <= 32; ++i) {
                    const double t = i / 32., d = 1 + 24 * t * (1 - t), z = 80 * t * (1 - t) / d;
                    const double x = .5 - .2 * z, y = std::sqrt(2.) * (1 - 2 * t) / d;
                    const auto point = world.point_at(t);
                    MINI_CHECK(std::min(point.distance({x, y, z}), point.distance({x, -y, z})) < 1e-10);
                }
                for (const auto* pc : {&ua, &ub})
                    for (int i = 0; i < pc->cv_count(); ++i)
                        MINI_CHECK(pc->weight(i) > 0 && std::isfinite(pc->weight(i)));
            }
        }
        MINI_CHECK(cone.jsondump() == before_cone && plane.jsondump() == before_plane);
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
    std::cerr << "[analytic-pullback-case] Surface Surface Coaxial Quadrics :: std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> sphere_cyl = Intersection::surface_surface(sphere, cyl);" << std::endl;
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> sphere_cyl = Intersection::surface_surface(sphere, cyl);

    MINI_CHECK(sphere_cyl.size() == 2);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : sphere_cyl) {
        MINI_CHECK(TOLERANCE.is_close(std::abs(std::get<0>(t).point_at_start()[2]), std::sqrt(3.0)));
        MINI_CHECK(on_both(std::get<0>(t), distance_sphere, distance_unit_cylinder) < 1e-9);
        MINI_CHECK(lies_on_curve(std::get<0>(t), std::get<1>(t), sphere) < 1e-9);
        MINI_CHECK(lies_on_curve(std::get<0>(t), std::get<2>(t), cyl) < 1e-9);
    }

    std::cerr << "[analytic-pullback-case] Surface Surface Coaxial Quadrics :: std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> cyl_cone = Intersection::surface_surface(cyl, cone);" << std::endl;
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> cyl_cone = Intersection::surface_surface(cyl, cone);

    MINI_CHECK(cyl_cone.size() == 1);
    MINI_CHECK(TOLERANCE.is_close(std::get<0>(cyl_cone[0]).point_at_start()[2], 1.0));
    MINI_CHECK(on_both(std::get<0>(cyl_cone[0]), distance_unit_cylinder, distance_cone) < 1e-9);
    MINI_CHECK(lies_on_curve(std::get<0>(cyl_cone[0]), std::get<2>(cyl_cone[0]), cone) < 1e-9);

    std::cerr << "[analytic-pullback-case] Surface Surface Coaxial Quadrics :: std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> cone_sphere = Intersection::surface_surface(cone, sphere);" << std::endl;
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
    std::cerr << "[analytic-pullback-case] Surface Surface Cone Apex :: std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> lines = Intersection::surface_surface(cone, axial);" << std::endl;
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
    std::cerr << "[analytic-pullback-case] Surface Surface Seam Pieces :: std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> circle = Intersection::surface_surface(sphere, wall);" << std::endl;
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> circle = Intersection::surface_surface(sphere, wall);
    double length = 0.0;

    MINI_CHECK(circle.size() == 2);

    for (const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : circle) {
        length += std::get<0>(t).length();

        MINI_CHECK(pcurve_end_gap(std::get<0>(t), std::get<1>(t), sphere) < 1e-9);
        MINI_CHECK(lifted_distance(std::get<1>(t), sphere, distance_wall) < 5e-3);
    }

    MINI_CHECK(std::abs(length - 2.0 * Tolerance::PI * std::sqrt(3.96)) < 1e-4);

    std::cerr << "[analytic-pullback-case] Surface Surface Seam Pieces :: std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> conic = Intersection::surface_surface(cone, slanted);" << std::endl;
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
    std::cerr << "[analytic-pullback-case] Cut Curves On Surface Pullbacks :: std::vector<NurbsCurve> sphere_cuts = Intersection::cut_curves_on_surface(sphere, wall);" << std::endl;
    std::vector<NurbsCurve> sphere_cuts = Intersection::cut_curves_on_surface(sphere, wall);

    MINI_CHECK(sphere_cuts.size() == 2);

    for (const NurbsCurve& pc : sphere_cuts)
        MINI_CHECK(lifted_distance(pc, sphere, distance_wall) < 5e-3);

    std::cerr << "[analytic-pullback-case] Cut Curves On Surface Pullbacks :: std::vector<NurbsCurve> cone_cuts = Intersection::cut_curves_on_surface(cone, wall);" << std::endl;
    std::vector<NurbsCurve> cone_cuts = Intersection::cut_curves_on_surface(cone, wall);

    MINI_CHECK(cone_cuts.size() == 2);

    for (const NurbsCurve& pc : cone_cuts)
        MINI_CHECK(lifted_distance(pc, cone, distance_wall) < 1e-3);

    std::cerr << "[analytic-pullback-case] Cut Curves On Surface Pullbacks :: std::vector<NurbsCurve> square_cuts = Intersection::cut_curves_on_surface(sphere, square);" << std::endl;
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
    save_section_geometry("torus_wall", torus, wall, Intersection::surface_surface(torus, wall));
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
    std::cerr << "[analytic-pullback-case] Cut Curves Slanted Cutter :: std::vector<NurbsCurve> cuts = Intersection::cut_curves_on_surface(cone, slanted);" << std::endl;
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

MINI_TEST("Intersection", "Plane Plane Optional") {
    // using session_cpp::Intersection;
    // using session_cpp::Line;
    // using session_cpp::Plane;
    // using session_cpp::Vector;

    const Plane xy = Plane::xy_plane();
    const Plane xz = Plane::xz_plane();
    const std::optional<Line> line = Intersection::plane_plane(xy, xz);
    const std::optional<Line> parallel = Intersection::plane_plane(xy, xy + Vector(0.0, 0.0, 1.0));

    MINI_CHECK(line.has_value());
    MINI_CHECK(TOLERANCE.is_close(std::abs(line->to_direction()[0]), 1.0));
    MINI_CHECK(!parallel.has_value());
}

MINI_TEST("Intersection", "Line Plane Optional") {
    // using session_cpp::Intersection;
    // using session_cpp::Line;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    const Plane plane = Plane::from_point_normal(Point(0.0, 0.0, 1.0), Vector(0.0, 0.0, 1.0));
    const Line line(0.0, 0.0, 0.0, 0.0, 0.0, 2.0);
    const Line short_line(0.0, 0.0, 0.0, 0.0, 0.0, 0.5);
    const Line flat(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    const std::optional<Point> hit = Intersection::line_plane(line, plane);
    const std::optional<Point> beyond = Intersection::line_plane(short_line, plane, false);

    MINI_CHECK(hit.has_value());
    MINI_CHECK(TOLERANCE.is_close((*hit)[2], 1.0));
    MINI_CHECK(!Intersection::line_plane(short_line, plane).has_value());
    MINI_CHECK(beyond.has_value());
    MINI_CHECK(TOLERANCE.is_close((*beyond)[2], 1.0));
    MINI_CHECK(!Intersection::line_plane(flat, plane, false).has_value());
}

MINI_TEST("Intersection", "Plane Plane Plane Optional") {
    // using session_cpp::Intersection;
    // using session_cpp::Plane;
    // using session_cpp::Point;
    // using session_cpp::Vector;

    const Plane xy = Plane::from_point_normal(Point(0.0, 0.0, 3.0), Vector(0.0, 0.0, 1.0));
    const Plane yz = Plane::from_point_normal(Point(1.0, 0.0, 0.0), Vector(1.0, 0.0, 0.0));
    const Plane xz = Plane::from_point_normal(Point(0.0, 2.0, 0.0), Vector(0.0, 1.0, 0.0));
    const std::optional<Point> corner = Intersection::plane_plane_plane(xy, yz, xz);
    const std::optional<Point> parallel = Intersection::plane_plane_plane(xy, yz, xy + Vector(0.0, 0.0, 1.0));

    MINI_CHECK(corner.has_value());
    MINI_CHECK(TOLERANCE.is_close((*corner)[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close((*corner)[1], 2.0));
    MINI_CHECK(TOLERANCE.is_close((*corner)[2], 3.0));
    MINI_CHECK(!parallel.has_value());
}

MINI_TEST("Intersection", "Preserved Affine Surface Plane Single Track") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_triple_affine_plane_support.json");
    MINI_CHECK(input.good());
    const auto fixture = nlohmann::json::parse(input);
    for (bool normalized : {false, true}) {
        NurbsSurface support = NurbsSurface::jsonload(fixture["surface"]);
        if (normalized) {
            MINI_CHECK(support.set_domain(0, 0.0, 1.0));
            MINI_CHECK(support.set_domain(1, 0.0, 1.0));
        }
        const double tolerance = fixture["tolerance"];
        const Plane plane = Plane::from_point_normal({0, 0, 0}, {0, 0, 1});
        const auto worlds = Intersection::surface_plane(support, plane, tolerance);
        MINI_CHECK(worlds.size() == 1);
        if (worlds.size() == 1) {
            MINI_CHECK(!worlds[0].is_closed());
            for (const auto& endpoint : fixture["expected_endpoints"]) {
                const Point expected(endpoint[0], endpoint[1], endpoint[2]);
                MINI_CHECK(std::min(expected.distance(worlds[0].point_at_start()),
                    expected.distance(worlds[0].point_at_end())) <= tolerance);
            }
        }
        const auto sections = Intersection::surface_plane_uv(support, plane, tolerance);
        MINI_CHECK(sections.size() == 1);
        if (sections.size() != 1)
            continue;
        const NurbsCurve& world = sections[0].first;
        const NurbsCurve& pcurve = sections[0].second;
        MINI_CHECK(world.domain() == pcurve.domain());
        MINI_CHECK(!world.is_closed());
        MINI_CHECK(curve_surface_intersection::same_parameter(world, pcurve, support, tolerance)
            == curve_surface_intersection::ParameterMatch::Matched);
        for (const auto& endpoint : fixture["expected_endpoints"]) {
            const Point expected(endpoint[0], endpoint[1], endpoint[2]);
            MINI_CHECK(std::min(expected.distance(world.point_at_start()),
                expected.distance(world.point_at_end())) <= tolerance);
        }
        for (int i = 0; i <= 16; ++i) {
            const auto domain = world.domain();
            const Point point = world.point_at(domain.first + (domain.second - domain.first) * i / 16.0);
            MINI_CHECK(std::abs(point[2]) <= tolerance);
            MINI_CHECK(std::abs(point[1] + 1.0 - 256.0 * point[0] * point[0] / 65025.0) <= tolerance);
        }
    }
}

} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Strict Analytic Affine Pullback Keeps Cone Apex Alias") {
    const auto cone = Primitives::cone_surface(0, 0, 0, 1.5, 3);
    const auto plane = NurbsSurface::create(false, false, 1, 1, 2, 2,
        {{0, -3, -3}, {0, -3, 4}, {0, 3, -3}, {0, 3, 4}});
    const auto before_cone = cone.jsondump(), before_plane = plane.jsondump();
    for (bool swap : {false, true}) {
        const auto& a = swap ? plane : cone;
        const auto& b = swap ? cone : plane;
        const auto curves = Intersection::surface_surface(a, b, 1e-12);
        MINI_CHECK(curves.size() == 2);
        int positive_base = 0, negative_base = 0;
        for (const auto& [world, pa, pb] : curves) {
            MINI_CHECK(world.domain() == pa.domain() && world.domain() == pb.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pa, a, 1e-12) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pb, b, 1e-12) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            const auto p = world.point_at_start(), q = world.point_at_end();
            MINI_CHECK(std::min(p.distance({0, 0, 3}), q.distance({0, 0, 3})) <= 1e-12);
            const Point base = p.distance({0, 0, 3}) < q.distance({0, 0, 3}) ? q : p;
            const bool positive = base.distance({0, 1.5, 0}) <= 1e-12;
            const bool negative = base.distance({0, -1.5, 0}) <= 1e-12;
            MINI_CHECK(positive || negative);
            positive_base += positive;
            negative_base += negative;
        }
        MINI_CHECK(positive_base == 1 && negative_base == 1);
    }
    MINI_CHECK(cone.jsondump() == before_cone && plane.jsondump() == before_plane);
}

MINI_TEST("Intersection", "Strict Analytic Affine Pullback Keeps Original Cone Endspan") {
    const auto cone = Primitives::cone_surface(0, 0, 0, 1.5, 3);
    const auto sphere = Primitives::sphere_surface(0, 0, 0, 2);
    const auto before_cone = cone.jsondump(), before_sphere = sphere.jsondump();
    const double low = (1.5 - std::sqrt(11.)) / 2.5, high = (1.5 + std::sqrt(11.)) / 2.5;
    for (bool swap : {false, true}) {
        const auto& a = swap ? sphere : cone;
        const auto& b = swap ? cone : sphere;
        const auto curves = Intersection::surface_surface(a, b, 1e-12);
        MINI_CHECK(curves.size() == 2);
        bool has_low = false, has_high = false;
        for (const auto& [world, pa, pb] : curves) {
            MINI_CHECK(world.domain() == pa.domain() && world.domain() == pb.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pa, a, 1e-12) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pb, b, 1e-12) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            const auto p = world.point_at_middle();
            has_low = has_low || std::abs(p[2] - low) <= 1e-12;
            has_high = has_high || std::abs(p[2] - high) <= 1e-12;
            MINI_CHECK(std::abs(p[0] * p[0] + p[1] * p[1] + p[2] * p[2] - 4) <= 1e-11);
        }
        // Existing analytic supporting-quadric contract includes the lower endspan branch.
        MINI_CHECK(has_low && has_high && low < 0);
    }
    MINI_CHECK(cone.jsondump() == before_cone && sphere.jsondump() == before_sphere);
}

MINI_TEST("Intersection", "Strict Radius Two Sphere Raw Pullback Across Knot Charts") {
    const auto first = Primitives::sphere_surface(0, 0, 0, 2);
    const auto second = Primitives::sphere_surface(2, 0, 0, 2);
    const auto before_a = first.jsondump(), before_b = second.jsondump();
    for (bool swap : {false, true}) {
        const auto& a = swap ? second : first;
        const auto& b = swap ? first : second;
        const auto triples = Intersection::surface_surface(a, b, 1e-12);
        MINI_CHECK(triples.size() == 2);
        int positive = 0, negative = 0;
        for (const auto& [world, pa, pb] : triples) {
            MINI_CHECK(world.domain() == pa.domain() && world.domain() == pb.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pa, a, 1e-12) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pb, b, 1e-12) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            for (int i = 0; i <= 16; ++i) {
                const auto p = world.point_at(std::lerp(world.domain_start(), world.domain_end(), i / 16.));
                MINI_CHECK(std::abs(p[0] - 1) <= 1e-12);
                MINI_CHECK(std::abs(p[1] * p[1] + p[2] * p[2] - 3) <= 1e-11);
            }
            const auto middle = world.point_at_middle();
            if (middle[1] > 0) ++positive; else ++negative;
        }
        MINI_CHECK(positive == 1 && negative == 1);
    }
    MINI_CHECK(first.jsondump() == before_a && second.jsondump() == before_b);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Plane Cone Finite Cap Arc Preserves Wrapped Raw Range") {
    const double tolerance = 1e-12;
    const auto cone = Primitives::cone_surface(0, 0, 0, 1.5, 3);
    // z=x-1: exact cap points (1,+/-sqrt(5)/2,0), highest point (4/3,0,1/3).
    const auto plane = bilinear(Point(-3, -3, -4), Point(-3, 3, -4),
                                Point(3, -3, 2), Point(3, 3, 2));
    const auto before_cone = cone.jsondump(), before_plane = plane.jsondump();
    for (bool swap : {false, true}) {
        const auto& a = swap ? plane : cone;
        const auto& b = swap ? cone : plane;
        const auto triples = Intersection::surface_surface(a, b, tolerance);
        MINI_CHECK(triples.size() == 2);
        int positive_cap = 0, negative_cap = 0, peak = 0;
        bool past_original_end = false;
        const Point positive(1, std::sqrt(5.) / 2, 0), negative(1, -std::sqrt(5.) / 2, 0);
        const Point top(4. / 3, 0, 1. / 3);
        for (const auto& [world, pa, pb] : triples) {
            MINI_CHECK(world.is_rational() && world.degree() == 2);
            MINI_CHECK(world.domain() == pa.domain() && world.domain() == pb.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pa, a, tolerance) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pb, b, tolerance) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            past_original_end = past_original_end || world.domain_end() > 1;
            for (const auto& p : {world.point_at_start(), world.point_at_end()}) {
                positive_cap += p.distance(positive) <= 1e-10;
                negative_cap += p.distance(negative) <= 1e-10;
                peak += p.distance(top) <= 1e-10;
            }
            for (int i = 0; i <= 16; ++i) {
                const auto p = world.point_at(std::lerp(world.domain_start(), world.domain_end(), i / 16.));
                const double radius = 1.5 - .5 * p[2];
                MINI_CHECK(p[2] >= -tolerance && p[2] <= 1. / 3 + tolerance);
                MINI_CHECK(std::abs(p[2] - p[0] + 1) <= 1e-11);
                MINI_CHECK(std::abs(p[0] * p[0] + p[1] * p[1] - radius * radius) <= 1e-11);
            }
        }
        MINI_CHECK(positive_cap == 1 && negative_cap == 1 && peak == 2);
        MINI_CHECK(past_original_end);
    }
    MINI_CHECK(cone.jsondump() == before_cone && plane.jsondump() == before_plane);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Strict Hyperbola Pullback Near Coincident Chart Sample") {
    const auto cone = Primitives::cone_surface(0,0,0,1.5,3);
    const auto plane = bilinear(Point(1.1,-3,-3),Point(1.1,3,-3),
                                Point(-.3,-3,4),Point(-.3,3,4));
    const auto before_cone=cone.jsondump(), before_plane=plane.jsondump();
    for (bool swap : {false,true}) {
        const auto& a=swap?plane:cone; const auto& b=swap?cone:plane;
        const auto result=Intersection::surface_surface(a,b,1e-12);
        MINI_CHECK(result.size()==1);
        for (const auto& [world,pa,pb] : result) {
            MINI_CHECK(world.degree()==2);
            MINI_CHECK(world.domain()==pa.domain() && world.domain()==pb.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(world,pa,a,1e-12)==
                       curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world,pb,b,1e-12)==
                       curve_surface_intersection::ParameterMatch::Matched);
            for(int i=0;i<=32;++i) {
                const auto p=world.point_at(std::lerp(world.domain_start(),world.domain_end(),i/32.));
                MINI_CHECK(std::abs(p[0]+.2*p[2]-.5)<1e-11);
                MINI_CHECK(std::abs(p[0]*p[0]+p[1]*p[1]-(1.5-.5*p[2])*(1.5-.5*p[2]))<1e-10);
            }
        }
    }
    MINI_CHECK(cone.jsondump()==before_cone && plane.jsondump()==before_plane);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Quintic Hermite Preserves Nonuniform Raw Polynomial Jets") {
    // Independent degree-five monomials specify values and both raw derivatives.
    const auto point = [](double s) {
        return Point(std::pow(s, 5) - 2 * s * s, std::pow(s, 4) + s, s * s * s - s * s);
    };
    const auto first = [](double s) {
        return Vector((5 * std::pow(s, 4) - 4 * s) / 20, (4 * s * s * s + 1) / 20, (3 * s * s - 2 * s) / 20);
    };
    const auto second = [](double s) {
        return Vector((20 * s * s * s - 4) / 400, 12 * s * s / 400, (6 * s - 2) / 400);
    };
    const std::vector<double> stations{0, .125, .5, .625, 1};
    std::vector<nurbs_hermite::QuinticSpan> spans;
    for (size_t i = 1; i < stations.size(); ++i) {
        const double a = stations[i - 1], b = stations[i];
        spans.push_back(
            {-7 + 20 * a, -7 + 20 * b, point(a), point(b), first(a), first(b), second(a), second(b)});
    }
    const auto curve = nurbs_hermite::quintic(spans);
    MINI_CHECK(curve.is_valid() && curve.degree() == 5 && !curve.is_rational());
    MINI_CHECK(curve.domain_start() == -7 && curve.domain_end() == 13);
    for (int i = 0; i <= 64; ++i) {
        const double s = i / 64.;
        const auto value = curve.evaluate(-7 + 20 * s, 2);
        MINI_CHECK(value.size() == 3);
        MINI_CHECK(Point(value[0][0], value[0][1], value[0][2]).distance(point(s)) < 2e-12);
        MINI_CHECK((value[1] - first(s)).magnitude() < 2e-12);
        MINI_CHECK((value[2] - second(s)).magnitude() < 2e-12);
    }
    // A C1 join need not be C2: retain the independent one-sided quadratic jet.
    const std::vector<nurbs_hermite::QuinticSpan> joined{
        {-1, 0, Point(), Point(), Vector(), Vector(), Vector(), Vector()},
        {0, 1, Point(), Point(1, 0, 0), Vector(), Vector(2, 0, 0), Vector(2, 0, 0), Vector(2, 0, 0)}};
    const auto hinge = nurbs_hermite::quintic(joined);
    MINI_CHECK(hinge.is_valid());
    MINI_CHECK(hinge.point_at(-.5).distance(Point()) < 1e-14);
    MINI_CHECK(hinge.point_at(.5).distance(Point(.25, 0, 0)) < 1e-14);
    auto invalid = spans;
    invalid[1].start[0] += 1e-5;
    MINI_CHECK(!nurbs_hermite::quintic(invalid).is_valid());
    invalid = spans;
    invalid[1].first = std::nextafter(invalid[1].first, invalid[1].last);
    MINI_CHECK(!nurbs_hermite::quintic(invalid).is_valid());
}

MINI_TEST("Intersection", "Strict Near Pole Sphere Raw Pullback Endpoint Jets") {
    const auto first = Primitives::sphere_surface(0, 0, 0, 2);
    const auto second = Primitives::sphere_surface(.4, 0, 0, 2);
    const auto before_a = first.jsondump(), before_b = second.jsondump();
    for (bool swap : {false, true}) {
        const auto& a = swap ? second : first;
        const auto& b = swap ? first : second;
        const auto triples = Intersection::surface_surface(a, b, 1e-12);
        MINI_CHECK(triples.size() == 2);
        int positive = 0, negative = 0;
        for (const auto& [world, pa, pb] : triples) {
            MINI_CHECK(world.domain() == pa.domain() && world.domain() == pb.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pa, a, 1e-12) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pb, b, 1e-12) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            for (int i = 0; i <= 16; ++i) {
                const auto p = world.point_at(std::lerp(world.domain_start(), world.domain_end(), i / 16.));
                MINI_CHECK(std::abs(p[0] - .2) <= 1e-12);
                MINI_CHECK(std::abs(p[1] * p[1] + p[2] * p[2] - 3.96) <= 1e-11);
            }
            const auto middle = world.point_at_middle();
            if (middle[1] > 0) {
                ++positive;
                // Exact stored-world witness at raw t=.49739583333333337. A
                // previously inserted projection sample missed this by1.34e-12.
                // Closest locates the point only; all full certificates above and
                // these ORIGINAL-geometry distance checks remain mandatory.
                const Point captured(.2, .02940320672283397, 1.989757636355346);
                const auto [t, unused_gap] = Closest::curve_point(world, captured);
                MINI_CHECK(world.point_at(t).distance(captured) <= 1e-12);
                const auto ua = pa.point_at(t), ub = pb.point_at(t);
                MINI_CHECK(a.point_at(ua[0], ua[1]).distance(captured) <= 1e-12);
                MINI_CHECK(b.point_at(ub[0], ub[1]).distance(captured) <= 1e-12);
            } else
                ++negative;
        }
        MINI_CHECK(positive == 1 && negative == 1);
    }
    MINI_CHECK(first.jsondump() == before_a && second.jsondump() == before_b);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Strict Near Pole Pullback Reversed Chart Coverage") {
    auto first=Primitives::sphere_surface(0,0,0,2);
    const auto second=Primitives::sphere_surface(.4,0,0,2);
    MINI_CHECK(first.reverse(0));
    const auto before_a=first.jsondump(),before_b=second.jsondump();
    const double z=std::sqrt(3.96);
    for(bool swap : {false,true}) {
        const auto& a=swap?second:first; const auto& b=swap?first:second;
        const auto result=Intersection::surface_surface(a,b,1e-12);
        MINI_CHECK(result.size()==2);
        int positive=0,negative=0,top=0,bottom=0;
        for(const auto& [world,pa,pb] : result) {
            MINI_CHECK(world.domain()==pa.domain() && world.domain()==pb.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(world,pa,a,1e-12)==
                curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world,pb,b,1e-12)==
                curve_surface_intersection::ParameterMatch::Matched);
            for(const auto p : {world.point_at_start(),world.point_at_end()}) {
                if(p.distance(Point(.2,0,z))<=1e-12) ++top;
                else if(p.distance(Point(.2,0,-z))<=1e-12) ++bottom;
                else MINI_CHECK(false);
            }
            const auto middle=world.point_at_middle();
            if(middle[1]>0) ++positive; else ++negative;
            for(int i=0;i<=32;++i) {
                const auto p=world.point_at(std::lerp(world.domain_start(),world.domain_end(),i/32.));
                MINI_CHECK(std::abs(p[0]-.2)<=1e-12);
                MINI_CHECK(std::abs(p[1]*p[1]+p[2]*p[2]-3.96)<=1e-11);
            }
        }
        MINI_CHECK(positive==1 && negative==1 && top==2 && bottom==2);
    }
    MINI_CHECK(first.jsondump()==before_a && second.jsondump()==before_b);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Near Cap Seam Proposal Preserves Reversed Chart Arc") {
    const double tolerance = 1e-12, rise = std::ldexp(1., -46);
    auto cone = Primitives::cone_surface(0, 0, 0, 1.5, 3);
    MINI_CHECK(cone.reverse(0));
    const auto plane = bilinear(Point(-3, -3, -4.5 + rise), Point(-3, 3, -4.5 + rise),
                                Point(3, -3, 1.5 + rise), Point(3, 3, 1.5 + rise));
    const auto before_cone = cone.jsondump(), before_plane = plane.jsondump();
    for (bool swap : {false, true}) {
        const auto& a = swap ? plane : cone;
        const auto& b = swap ? cone : plane;
        const auto result = Intersection::surface_surface(a, b, tolerance);
        MINI_CHECK(!result.empty());
        double low = 1, high = -1, peak = 0;
        for (const auto& [world, pa, pb] : result) {
            MINI_CHECK(world.is_rational() && world.degree() == 2);
            MINI_CHECK(world.domain() == pa.domain() && world.domain() == pb.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pa, a, tolerance) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pb, b, tolerance) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            for (int i = 0; i <= 32; ++i) {
                const auto p = world.point_at(std::lerp(world.domain_start(), world.domain_end(), i / 32.));
                low = std::min(low, p[1]); high = std::max(high, p[1]); peak = std::max(peak, p[2]);
                MINI_CHECK(p[2] >= -tolerance);
                MINI_CHECK(std::abs(p[2] - p[0] + 1.5 - rise) <= tolerance);
                MINI_CHECK(std::abs(p[0]*p[0] + p[1]*p[1] - (1.5-.5*p[2])*(1.5-.5*p[2])) <= 1e-11);
            }
        }
        MINI_CHECK(high-low > 1e-8 && peak > rise*.125);
    }
    MINI_CHECK(cone.jsondump() == before_cone && plane.jsondump() == before_plane);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Near Cap Closed Projection Preserves Shifted Raw Chart") {
    const double tolerance = 1e-12, rise = std::ldexp(1., -46);
    auto cone = Primitives::cone_surface(0, 0, 0, 1.5, 3);
    MINI_CHECK(cone.reverse(0));
    MINI_CHECK(cone.set_domain(0, -7, 13));
    MINI_CHECK(cone.set_domain(1, 32, 35));
    const auto plane = bilinear(Point(-3, -3, -4.5 + rise), Point(-3, 3, -4.5 + rise),
                                Point(3, -3, 1.5 + rise), Point(3, 3, 1.5 + rise));
    const auto before_cone = cone.jsondump(), before_plane = plane.jsondump();
    for (bool swap : {false, true}) {
        const auto& a = swap ? plane : cone;
        const auto& b = swap ? cone : plane;
        const auto result = Intersection::surface_surface(a, b, tolerance);
        MINI_CHECK(!result.empty());
        double low = 1, high = -1, peak = 0;
        for (const auto& [world, pa, pb] : result) {
            MINI_CHECK(world.is_rational() && world.degree() == 2);
            MINI_CHECK(world.domain() == pa.domain() && world.domain() == pb.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pa, a, tolerance) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pb, b, tolerance) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            for (int i = 0; i <= 32; ++i) {
                const auto p = world.point_at(std::lerp(world.domain_start(), world.domain_end(), i / 32.));
                low = std::min(low, p[1]); high = std::max(high, p[1]); peak = std::max(peak, p[2]);
                MINI_CHECK(p[2] >= -tolerance);
                MINI_CHECK(std::abs(p[2] - p[0] + 1.5 - rise) <= tolerance);
                MINI_CHECK(std::abs(p[0]*p[0] + p[1]*p[1] - (1.5-.5*p[2])*(1.5-.5*p[2])) <= 1e-11);
            }
        }
        MINI_CHECK(high-low > 1e-8 && peak > rise*.125);
    }
    MINI_CHECK(cone.jsondump() == before_cone && plane.jsondump() == before_plane);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Analytic Latitude Raw Range Preserves Actual Chart Phase") {
    const auto cone=Primitives::cone_surface(0,0,0,1.5,3);
    const auto sphere=Primitives::sphere_surface(0,0,0,2);
    const auto before_a=cone.jsondump(),before_b=sphere.jsondump();
    const double low=(1.5-std::sqrt(11.))/2.5, high=(1.5+std::sqrt(11.))/2.5;
    const auto result=Intersection::surface_surface(cone,sphere,1e-12);
    MINI_CHECK(result.size()==2);
    bool found_low=false,found_high=false;
    for(const auto& [world,pa,pb] : result) {
        MINI_CHECK(world.domain()==pa.domain() && world.domain()==pb.domain());
        MINI_CHECK(curve_surface_intersection::same_parameter(world,pa,cone,1e-12)==
            curve_surface_intersection::ParameterMatch::Matched);
        MINI_CHECK(curve_surface_intersection::same_parameter(world,pb,sphere,1e-12)==
            curve_surface_intersection::ParameterMatch::Matched);
        const bool lower=std::abs(world.point_at_middle()[2]-low)<1e-12;
        if(lower) {
            found_low=true;
            MINI_CHECK(world.domain_start()==.5 && world.domain_end()==1.5);
        } else {
            found_high=true;
            MINI_CHECK(std::abs(world.point_at_middle()[2]-high)<1e-12);
            MINI_CHECK(world.domain_start()==0 && world.domain_end()==1);
            const auto left=pb.point_at(.5-1e-7),right=pb.point_at(.5+1e-7);
            MINI_CHECK(std::abs(left[0]-right[0])>3.9);
        }
        for(int i=0;i<=32;++i) {
            const auto p=world.point_at(std::lerp(world.domain_start(),world.domain_end(),i/32.));
            MINI_CHECK(std::abs(p[2]-(lower?low:high))<1e-12);
            MINI_CHECK(std::abs(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]-4)<1e-11);
        }
    }
    MINI_CHECK(found_low && found_high);
    MINI_CHECK(cone.jsondump()==before_a && sphere.jsondump()==before_b);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Strict Plane Cone Parabola Uses Focal Parameter") {
    const auto cone=Primitives::cone_surface(0,0,0,1.5,3);
    const auto plane=bilinear(Point(-3,-3,8.5),Point(-3,3,8.5),
                              Point(3,-3,-3.5),Point(3,3,-3.5));
    const auto before_a=cone.jsondump(),before_b=plane.jsondump();
    const double endpoint_y=std::sqrt(11.)/4;
    for(bool swap : {false,true}) {
        const auto& a=swap?plane:cone; const auto& b=swap?cone:plane;
        const auto result=Intersection::surface_surface(a,b,1e-12);
        MINI_CHECK(result.size()==1);
        for(const auto& [world,pa,pb] : result) {
            MINI_CHECK(world.degree()==2 && world.cv_count()==3);
            MINI_CHECK(world.domain()==pa.domain() && world.domain()==pb.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(world,pa,a,1e-12)==
                curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world,pb,b,1e-12)==
                curve_surface_intersection::ParameterMatch::Matched);
            const auto p=world.point_at_start(),q=world.point_at_end();
            MINI_CHECK(std::min(std::max(p.distance(Point(1.25,-endpoint_y,0)),q.distance(Point(1.25,endpoint_y,0))),
                                std::max(p.distance(Point(1.25,endpoint_y,0)),q.distance(Point(1.25,-endpoint_y,0))))<=1e-12);
            MINI_CHECK(world.point_at_middle().distance(Point(-.125,0,2.75))<=1e-12);
            for(int i=0;i<=32;++i) {
                const auto p=world.point_at(std::lerp(world.domain_start(),world.domain_end(),i/32.));
                MINI_CHECK(std::abs(p[0]-(2*p[1]*p[1]-.125))<=1e-11);
                MINI_CHECK(std::abs(p[2]-(2.75-4*p[1]*p[1]))<=1e-11);
            }
        }
    }
    MINI_CHECK(cone.jsondump()==before_a && plane.jsondump()==before_b);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Slanted Cone Original Endspan Pullback Is Certified") {
    const double tolerance = 1e-12;
    auto cone = Primitives::cone_surface(0, 0, 0, 1.5, 3);
    MINI_CHECK(cone.set_domain(0, -7, 13));
    MINI_CHECK(cone.set_domain(1, 32, 35));
    const auto plane = bilinear(Point(-3,-3,0),Point(-3,3,-.6),Point(3,-3,1.2),Point(3,3,.6));
    const auto before_a = cone.jsondump(), before_b = plane.jsondump();
    for (bool swap : {false, true}) {
        const auto& a = swap ? plane : cone; const auto& b = swap ? cone : plane;
        const auto sections = Intersection::surface_surface(a,b,tolerance);
        MINI_CHECK(sections.size() == 2);
        for (const auto& [world,pa,pb] : sections) {
            MINI_CHECK(world.is_rational() && world.degree() == 2);
            MINI_CHECK(world.domain() == pa.domain() && world.domain() == pb.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(world,pa,a,tolerance) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world,pb,b,tolerance) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            for (int i=0;i<=32;++i) {
                const auto p = world.point_at(std::lerp(world.domain_start(),world.domain_end(),i/32.));
                const double radius = 1.5-.5*p[2];
                MINI_CHECK(std::abs(p[0]*p[0]+p[1]*p[1]-radius*radius) <= 1e-11);
                MINI_CHECK(std::abs(p[2]-.2*p[0]+.1*p[1]-.3) <= 1e-11);
                MINI_CHECK(p[2] >= -tolerance && p[2] <= 3+tolerance);
            }
        }
    }
    MINI_CHECK(cone.jsondump() == before_a && plane.jsondump() == before_b);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Strict Trapezoid Pullback Preserves Nonlinear Raw Law") {
    const auto trapezoid=bilinear(Point(-3,-3,0),Point(-1,3,0),Point(3,-3,0),Point(7,3,0));
    const auto wall=bilinear(Point(6,-5,-1),Point(6,5,-1),Point(6,-5,1),Point(6,5,1));
    const auto before_a=trapezoid.jsondump(),before_b=wall.jsondump();
    for(bool swap : {false,true}) {
        const auto& a=swap?wall:trapezoid; const auto& b=swap?trapezoid:wall;
        const auto result=Intersection::surface_surface(a,b,1e-12);
        MINI_CHECK(result.size()==1);
        for(const auto& [world,pa,pb] : result) {
            MINI_CHECK(world.domain()==pa.domain() && world.domain()==pb.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(world,pa,a,1e-12)==
                curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world,pb,b,1e-12)==
                curve_surface_intersection::ParameterMatch::Matched);
            const auto p=world.point_at_start(),q=world.point_at_end();
            MINI_CHECK(std::min(std::max(p.distance(Point(6,1.5,0)),q.distance(Point(6,3,0))),
                                std::max(p.distance(Point(6,3,0)),q.distance(Point(6,1.5,0))))<=1e-12);
            MINI_CHECK(world.point_at_middle().distance(Point(6,2.25,0))<=1e-12);
            const auto& pc=swap?pb:pa;
            for(int i=0;i<=32;++i) {
                const double t=std::lerp(world.domain_start(),world.domain_end(),i/32.);
                const auto uv=pc.point_at(t),point=world.point_at(t);
                // Independent inverse of S=(-3+6u+2v+2uv,-3+6v,0).
                MINI_CHECK(std::abs(uv[1]-(point[1]+3)/6)<=1e-12);
                MINI_CHECK(std::abs(uv[0]-(9-2*uv[1])/(6+2*uv[1]))<=1e-12);
            }
        }
    }
    MINI_CHECK(trapezoid.jsondump()==before_a && wall.jsondump()==before_b);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Certified Nonlinear Raw Seam Split Preserves Physical Junction") {
    const double tolerance = 1e-12;
    auto cone = Primitives::cone_surface(0, 0, 0, 1.5, 3);
    MINI_CHECK(cone.set_domain(0,-7,13));
    MINI_CHECK(cone.set_domain(1,32,35));
    const auto plane = bilinear(Point(-3, -3, 0), Point(-3, 3, -.6), Point(3, -3, 1.2), Point(3, 3, .6));
    const auto before_cone = cone.jsondump(), before_plane = plane.jsondump();
    for (bool swap : {false, true}) {
        const auto& first = swap ? plane : cone;
        const auto& second = swap ? cone : plane;
        const auto result = Intersection::surface_surface(first, second, tolerance);
        MINI_CHECK(result.size() == 2);
        int base_a = 0, base_b = 0, seam = 0;
        for (const auto& section : result) {
            const auto& world = std::get<0>(section);
            const auto& ua = std::get<1>(section);
            const auto& ub = std::get<2>(section);
            MINI_CHECK(world.is_rational() && world.degree() == 2);
            MINI_CHECK(world.domain() == ua.domain() && world.domain() == ub.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(world, ua, first, tolerance) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world, ub, second, tolerance) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            const auto domain = world.domain();
            for (const auto& point : {world.point_at_start(), world.point_at_end()}) {
                base_a += point.distance({-.9, 1.2, 0}) < 1e-10;
                base_b += point.distance({-1.5, 0, 0}) < 1e-10;
                seam += point.distance({27. / 22, 0, 6. / 11}) < 1e-10;
            }
            for (int sample = 0; sample <= 32; ++sample) {
                const auto point = world.point_at(std::lerp(domain.first, domain.second, sample / 32.));
                const double radius = 1.5 - point[2] * .5;
                MINI_CHECK(std::abs(point[0] * point[0] + point[1] * point[1] - radius * radius) < 1e-11);
                MINI_CHECK(std::abs(point[2] - .2 * point[0] + .1 * point[1] - .3) < 1e-11);
                MINI_CHECK(point[2] >= -tolerance && point[2] <= 3 + tolerance);
            }
        }
        MINI_CHECK(base_a == 1 && base_b == 1 && seam == 2);
    }
    MINI_CHECK(cone.jsondump() == before_cone && plane.jsondump() == before_plane);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Certified Seam Query Preserves Shifted Supporting Cone Endspan") {
    auto cone = Primitives::cone_surface(0, 0, 0, 1.5, 3);
    MINI_CHECK(cone.set_domain(0,-7,13));
    MINI_CHECK(cone.set_domain(1,32,35));
    const auto sphere = Primitives::sphere_surface(0, 0, 0, 2);
    const auto before_cone = cone.jsondump(), before_sphere = sphere.jsondump();
    const double low = (1.5 - std::sqrt(11.)) / 2.5, high = (1.5 + std::sqrt(11.)) / 2.5;
    for (bool swap : {false, true}) {
        const auto& a = swap ? sphere : cone;
        const auto& b = swap ? cone : sphere;
        const auto curves = Intersection::surface_surface(a, b, 1e-12);
        MINI_CHECK(curves.size() == 2);
        bool has_low = false, has_high = false;
        for (const auto& [world, pa, pb] : curves) {
            MINI_CHECK(world.domain() == pa.domain() && world.domain() == pb.domain());
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pa, a, 1e-12) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pb, b, 1e-12) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            const auto p = world.point_at_middle();
            has_low = has_low || std::abs(p[2] - low) <= 1e-12;
            has_high = has_high || std::abs(p[2] - high) <= 1e-12;
            MINI_CHECK(std::abs(p[0] * p[0] + p[1] * p[1] + p[2] * p[2] - 4) <= 1e-11);
        }
        // Existing analytic supporting-quadric contract includes the lower endspan branch.
        MINI_CHECK(has_low && has_high && low < 0);
    }
    MINI_CHECK(cone.jsondump() == before_cone && sphere.jsondump() == before_sphere);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Quintic Hermite Preserves Positive Tiny Raw Span") {
    const double end = 3.543711097672514e-33;
    const auto point = [](double s) { return Point(s*s,2*s*s*s-s,0); };
    const auto first = [&](double s) { return Vector(2*s/end,(6*s*s-1)/end,0); };
    const auto second = [&](double s) { return Vector(2/(end*end),12*s/(end*end),0); };
    const std::vector<nurbs_hermite::QuinticSpan> spans{
        {0,end,point(0),point(1),first(0),first(1),second(0),second(1)}};
    const auto curve = nurbs_hermite::quintic(spans);
    MINI_CHECK(curve.is_valid() && curve.degree()==5);
    MINI_CHECK(curve.domain_start()==0 && curve.domain_end()==end);
    MINI_CHECK(curve.m_nurbsknot==std::vector<double>({0,0,0,0,0,end,end,end,end,end}));
    for(int i=0;i<=32;++i) {
        const double s=i/32.;
        MINI_CHECK(curve.point_at(s*end).distance(point(s))<=1e-12);
    }
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Analytic Recognition Keeps Rhino Torus Physical Precision") {
    const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() /
        "tests/fixtures/rhino_booleans/inputs/InputTorus-object-0.step";
    const auto supports = file_step::read_file_step_nurbssurfaces(path.string());
    MINI_CHECK(supports.size() == 1);
    if (supports.size() != 1) return;
    const auto& torus = supports.front();
    const auto before = torus.jsondump();
    const double cx = .259563270114015, cy = 54.2018251668541;
    const double major = 16.7189880045964, minor = 6.64007692750769;
    for (double height : {0., 10.}) {
        const auto plane = bilinear(Point(cx-30,cy-30,height), Point(cx-30,cy+30,height),
                                    Point(cx+30,cy-30,height), Point(cx+30,cy+30,height));
        for (bool swap : {false, true}) {
            const auto curves = swap ? Intersection::surface_surface(plane, torus, .001) :
                                       Intersection::surface_surface(torus, plane, .001);
            MINI_CHECK(curves.size() == (height == 0 ? 2u : 0u));
            bool inner = false, outer = false;
            for (const auto& section : curves) {
                const auto& world = std::get<0>(section);
                const auto& pc_torus = swap ? std::get<2>(section) : std::get<1>(section);
                const auto& pc_plane = swap ? std::get<1>(section) : std::get<2>(section);
                MINI_CHECK(world.is_closed());
                MINI_CHECK(curve_surface_intersection::same_parameter(world,pc_torus,torus,.001) ==
                           curve_surface_intersection::ParameterMatch::Matched);
                MINI_CHECK(curve_surface_intersection::same_parameter(world,pc_plane,plane,.001) ==
                           curve_surface_intersection::ParameterMatch::Matched);
                const auto first = world.point_at_start();
                const double radius = std::hypot(first[0]-cx, first[1]-cy);
                inner = inner || std::abs(radius-(major-minor)) < 1e-7;
                outer = outer || std::abs(radius-(major+minor)) < 1e-7;
                for (int k=0;k<=16;++k) {
                    const auto p = world.point_at(world.domain_start()+
                        (world.domain_end()-world.domain_start())*k/16.);
                    MINI_CHECK(std::abs(p[2]) < 1e-7);
                    const double rho = std::hypot(p[0]-cx,p[1]-cy);
                    MINI_CHECK(std::min(std::abs(rho-(major-minor)),std::abs(rho-(major+minor))) < 1e-7);
                }
            }
            if (height == 0) MINI_CHECK(inner && outer);
        }
    }
    MINI_CHECK(torus.jsondump() == before);
}

MINI_TEST("Intersection", "Analytic Recognition Does Not Flatten Opposite Bilinear Saddles") {
    const auto a = bilinear(Point(-1,-1,.25),Point(-1,1,-.25),Point(1,-1,-.25),Point(1,1,.25));
    const auto b = bilinear(Point(-1,-1,-.25),Point(-1,1,.25),Point(1,-1,.25),Point(1,1,-.25));
    const auto before_a = a.jsondump(), before_b = b.jsondump();
    for (bool swap : {false,true}) {
        const auto sections = swap ? Intersection::surface_surface(b,a,.001) :
                                     Intersection::surface_surface(a,b,.001);
        MINI_CHECK(sections.size() == 2);
        bool x_axis = false, y_axis = false;
        for (const auto& section : sections) {
            const auto& world = std::get<0>(section);
            const auto& pa = swap ? std::get<2>(section) : std::get<1>(section);
            const auto& pb = swap ? std::get<1>(section) : std::get<2>(section);
            MINI_CHECK(curve_surface_intersection::same_parameter(world,pa,a,.001) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(curve_surface_intersection::same_parameter(world,pb,b,.001) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            const auto first = world.point_at_start(), last = world.point_at_end();
            const bool along_x = std::abs(first[1])+std::abs(last[1]) < 1e-12;
            const bool along_y = std::abs(first[0])+std::abs(last[0]) < 1e-12;
            MINI_CHECK(along_x != along_y);
            const int varying = along_x ? 0 : 1;
            MINI_CHECK(std::abs(std::min(first[varying],last[varying])+1) < 1e-12);
            MINI_CHECK(std::abs(std::max(first[varying],last[varying])-1) < 1e-12);
            x_axis = x_axis || along_x;
            y_axis = y_axis || along_y;
            for (int k=0;k<=8;++k) {
                const auto p = world.point_at(world.domain_start()+
                    (world.domain_end()-world.domain_start())*k/8.);
                MINI_CHECK(std::abs(p[2]) < 1e-12 && std::abs(p[0]*p[1]) < 1e-12);
            }
        }
        MINI_CHECK(x_axis && y_axis);
    }
    MINI_CHECK(a.jsondump() == before_a && b.jsondump() == before_b);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Genuine Positive Clipped Sections Survive Both Length Floors") {
    for (bool absolute_floor : {false,true}) {
        const double length=std::ldexp(1.,absolute_floor?-32:-16);
        const double tolerance=length/(absolute_floor?16:4);
        const auto a=bilinear(Point(-1,-1,0),Point(-1,1,0),Point(1,-1,0),Point(1,1,0));
        const auto b=bilinear(Point(1-length,0,-1),Point(1-length,0,1),
                              Point(3-length,0,-1),Point(3-length,0,1));
        const auto before_a=a.jsondump(), before_b=b.jsondump();
        for(bool swap : {false,true}) {
            const auto sections=swap?Intersection::surface_surface(b,a,tolerance):
                                     Intersection::surface_surface(a,b,tolerance);
            MINI_CHECK(sections.size()==1);
            for(const auto& section : sections) {
                const auto& world=std::get<0>(section);
                const auto p=world.point_at_start(), q=world.point_at_end();
                MINI_CHECK(std::abs(std::min(p[0],q[0])-(1-length))<=tolerance);
                MINI_CHECK(std::abs(std::max(p[0],q[0])-1)<=tolerance);
                MINI_CHECK(std::abs(p[1])+std::abs(p[2])+std::abs(q[1])+std::abs(q[2])<=tolerance);
                MINI_CHECK(curve_surface_intersection::same_parameter(world,
                    swap?std::get<2>(section):std::get<1>(section),a,tolerance)==
                    curve_surface_intersection::ParameterMatch::Matched);
                MINI_CHECK(curve_surface_intersection::same_parameter(world,
                    swap?std::get<1>(section):std::get<2>(section),b,tolerance)==
                    curve_surface_intersection::ParameterMatch::Matched);
            }
        }
        MINI_CHECK(a.jsondump()==before_a && b.jsondump()==before_b);
    }
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Intersection", "Original Collapsed Pole Endpoints Keep One Sided Meridian Chart") {
    const double tolerance = 1e-7;
    for (int variant = 0; variant < 3; ++variant) {
        auto sphere = BRep::create_sphere(2).m_surfaces[0];
        if (variant == 1) {
            MINI_CHECK(sphere.set_domain(0,32,35));
            MINI_CHECK(sphere.set_domain(1,-7,13));
        }
        if (variant == 2) MINI_CHECK(sphere.reverse(0));
        const auto plane = bilinear(Point(0,-3,-3),Point(0,3,-3),Point(0,-3,3),Point(0,3,3));
        const auto before_s = sphere.jsondump(), before_p = plane.jsondump();
        for (bool swap : {false,true}) {
            const auto result = swap ? Intersection::surface_surface(plane,sphere,tolerance)
                                     : Intersection::surface_surface(sphere,plane,tolerance);
            MINI_CHECK(!result.empty());
            for (const auto& [world,first,second] : result) {
                MINI_CHECK(world.domain() == first.domain() && world.domain() == second.domain());
                MINI_CHECK(world.domain_start() < world.domain_end());
                MINI_CHECK(curve_surface_intersection::same_parameter(world,swap?second:first,sphere,tolerance)
                    == curve_surface_intersection::ParameterMatch::Matched);
                MINI_CHECK(curve_surface_intersection::same_parameter(world,swap?first:second,plane,tolerance)
                    == curve_surface_intersection::ParameterMatch::Matched);
            }
            for (const Point target : {Point(0,2,0),Point(0,-2,0),Point(0,0,2),Point(0,0,-2)}) {
                bool found = false;
                for (const auto& section : result) {
                    const auto& world = std::get<0>(section);
                    const auto [parameter,gap] = Closest::curve_point(world,target);
                    found = found || (std::isfinite(parameter) && std::isfinite(gap) &&
                        world.point_at(parameter).distance(target) <= tolerance);
                }
                MINI_CHECK(found);
            }
        }
        MINI_CHECK(sphere.jsondump() == before_s && plane.jsondump() == before_p);
    }
}
} // namespace session_cpp
