#include "curve_intersection.h"
#include "mini_test.h"
#include "primitives.h"
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>

using namespace session_cpp::mini_test;

namespace session_cpp {
namespace {

NurbsCurve curve_from_points(const std::vector<Point>& points, int degree = 1) {
    NurbsCurve curve = NurbsCurve::create(false, degree, points);
    if (!curve.set_domain(0.0, 1.0))
        throw std::runtime_error("Could not normalize test curve");

    return curve;
}

void check_events(
    const NurbsCurve& a,
    const NurbsCurve& b,
    const curve_intersection::Result& result,
    double tolerance
) {
    MINI_CHECK(result.complete);
    for (const curve_intersection::Hit& hit : result.points) {
        MINI_CHECK(hit.a >= a.domain_start() && hit.a <= a.domain_end());
        MINI_CHECK(hit.b >= b.domain_start() && hit.b <= b.domain_end());
        MINI_CHECK(a.point_at(hit.a).distance(b.point_at(hit.b)) <= tolerance);
        MINI_CHECK(a.point_at(hit.a).distance(hit.point) <= tolerance);
    }
    for (const curve_intersection::Overlap& shared : result.overlaps) {
        MINI_CHECK(shared.a1 > shared.a0);
        MINI_CHECK(a.point_at(shared.a0).distance(b.point_at(shared.b0)) <= tolerance);
        MINI_CHECK(a.point_at(shared.a1).distance(b.point_at(shared.b1)) <= tolerance);
    }
}

void save_case(
    const std::string& name,
    const NurbsCurve& a,
    const NurbsCurve& b,
    const curve_intersection::Result& result
) {
    static nlohmann::ordered_json cases = nlohmann::ordered_json::object();
    nlohmann::ordered_json item;
    item["complete"] = result.complete;
    item["failure"] = result.failure;
    for (const NurbsCurve* curve : {&a, &b}) {
        nlohmann::ordered_json samples = nlohmann::ordered_json::array();
        for (int i = 0; i <= 256; ++i) {
            const double t = curve->domain_start() + i * (curve->domain_end() - curve->domain_start()) / 256.0;
            const Point p = curve->point_at(t);
            samples.push_back({p[0], p[1], p[2]});
        }
        item[curve == &a ? "a" : "b"] = samples;
    }
    item["points"] = nlohmann::ordered_json::array();
    for (const curve_intersection::Hit& hit : result.points)
        item["points"].push_back({{"point", {hit.point[0], hit.point[1], hit.point[2]}},
            {"a", hit.a}, {"b", hit.b}, {"tangent", hit.tangent}});

    item["overlaps"] = nlohmann::ordered_json::array();
    for (const curve_intersection::Overlap& shared : result.overlaps) {
        nlohmann::ordered_json samples = nlohmann::ordered_json::array();
        for (int i = 0; i <= 64; ++i) {
            const Point p = a.point_at(shared.a0 + i * (shared.a1 - shared.a0) / 64.0);
            samples.push_back({p[0], p[1], p[2]});
        }
        item["overlaps"].push_back({{"a0", shared.a0}, {"a1", shared.a1},
            {"b0", shared.b0}, {"b1", shared.b1}, {"samples", samples}});
    }
    cases[name] = item;
    std::filesystem::create_directories("build/native-geometry-tests");
    std::ofstream("build/native-geometry-tests/curve_intersection_cases.json") << cases.dump(2);
}

}

MINI_TEST("CurveIntersection", "Crossing And Skew Miss") {
    const NurbsCurve a = curve_from_points({{-1, 0, 0}, {1, 0, 0}});
    const NurbsCurve b = curve_from_points({{0, -1, 0}, {0, 1, 0}});
    const curve_intersection::Result crossing = curve_intersection::intersect(a, b);
    check_events(a, b, crossing, 1e-7);
    MINI_CHECK(crossing.points.size() == 1);
    MINI_CHECK(crossing.overlaps.empty());
    MINI_CHECK(std::abs(crossing.points[0].a - 0.5) < 1e-12);
    MINI_CHECK(std::abs(crossing.points[0].b - 0.5) < 1e-12);
    MINI_CHECK(!crossing.points[0].tangent);
    save_case("Crossing", a, b, crossing);

    const NurbsCurve c = curve_from_points({{0, -1, 1e-4}, {0, 1, 1e-4}});
    const curve_intersection::Result miss = curve_intersection::intersect(a, c);
    check_events(a, c, miss, 1e-7);
    MINI_CHECK(miss.points.empty());
    MINI_CHECK(miss.overlaps.empty());
    save_case("Skew Miss", a, c, miss);
}

MINI_TEST("CurveIntersection", "Endpoint And Disjoint") {
    const NurbsCurve a = curve_from_points({{0, 0, 0}, {1, 0, 0}});
    const NurbsCurve b = curve_from_points({{1, 0, 0}, {2, 1, 0}});
    const curve_intersection::Result endpoint = curve_intersection::intersect(a, b);
    check_events(a, b, endpoint, 1e-7);
    MINI_CHECK(endpoint.points.size() == 1);
    MINI_CHECK(std::abs(endpoint.points[0].a - 1.0) < 1e-12);
    MINI_CHECK(std::abs(endpoint.points[0].b) < 1e-12);

    const NurbsCurve c = curve_from_points({{2, 0, 0}, {3, 0, 0}});
    const curve_intersection::Result disjoint = curve_intersection::intersect(a, c);
    check_events(a, c, disjoint, 1e-7);
    MINI_CHECK(disjoint.points.empty());
    MINI_CHECK(disjoint.overlaps.empty());
}

MINI_TEST("CurveIntersection", "Tangency And Near Miss") {
    const NurbsCurve parabola = curve_from_points({{-1, 1, 0}, {0, -1, 0}, {1, 1, 0}}, 2);
    const NurbsCurve tangent = curve_from_points({{-1, 0, 0}, {1, 0, 0}});
    const curve_intersection::Result result = curve_intersection::intersect(parabola, tangent);
    check_events(parabola, tangent, result, 1e-7);
    MINI_CHECK(result.points.size() == 1);
    MINI_CHECK(result.overlaps.empty());
    MINI_CHECK(result.points[0].tangent);
    MINI_CHECK(std::abs(result.points[0].a - 0.5) < 1e-6);
    save_case("Parabola Tangency", parabola, tangent, result);

    const NurbsCurve miss = curve_from_points({{-1, -1e-5, 0}, {1, -1e-5, 0}});
    const curve_intersection::Result absent = curve_intersection::intersect(parabola, miss);
    check_events(parabola, miss, absent, 1e-7);
    MINI_CHECK(absent.points.empty());
    MINI_CHECK(absent.overlaps.empty());

    const NurbsCurve near = curve_from_points({{-1, 1e-10, 0}, {1, 1e-10, 0}});
    const curve_intersection::Result close_roots = curve_intersection::intersect(parabola, near, 1e-12);
    check_events(parabola, near, close_roots, 1e-12);
    MINI_CHECK(close_roots.points.size() == 2);
    MINI_CHECK(close_roots.overlaps.empty());
    MINI_CHECK(close_roots.points[1].a - close_roots.points[0].a > 9e-6);
}

MINI_TEST("CurveIntersection", "Rational Circle And Seam") {
    const NurbsCurve circle = Primitives::circle(0, 0, 0, 1);
    const NurbsCurve line = curve_from_points({{-2, 0, 0}, {2, 0, 0}});
    const curve_intersection::Result result = curve_intersection::intersect(circle, line);
    check_events(circle, line, result, 1e-7);
    MINI_CHECK(result.points.size() == 3);
    MINI_CHECK(result.overlaps.empty());
    MINI_CHECK(std::abs(result.points.front().a - circle.domain_start()) < 1e-10);
    MINI_CHECK(std::abs(result.points.back().a - circle.domain_end()) < 1e-10);
    save_case("Rational Circle Seam", circle, line, result);
}

MINI_TEST("CurveIntersection", "Linear Rational Overlap") {
    const NurbsCurve a = curve_from_points({{0, 0, 0}, {3, 0, 0}});
    NurbsCurve b = curve_from_points({{4, 0, 0}, {1, 0, 0}});
    MINI_CHECK(b.to_rational());
    MINI_CHECK(b.set_cv_4d(1, 4.0, 0.0, 0.0, 4.0));
    const curve_intersection::Result result = curve_intersection::intersect(a, b);
    check_events(a, b, result, 1e-7);
    MINI_CHECK(result.points.empty());
    MINI_CHECK(result.overlaps.size() == 1);
    MINI_CHECK(std::abs(result.overlaps[0].a0 - 1.0 / 3.0) < 1e-10);
    MINI_CHECK(std::abs(result.overlaps[0].a1 - 1.0) < 1e-10);
    MINI_CHECK(result.overlaps[0].b1 < result.overlaps[0].b0);
    save_case("Rational Line Overlap", a, b, result);
}

MINI_TEST("CurveIntersection", "Rational Circle Crossing And Tangency") {
    const NurbsCurve a = Primitives::circle(0, 0, 0, 1);
    const NurbsCurve b = Primitives::circle(1, 0, 0, 1);
    const curve_intersection::Result crossing = curve_intersection::intersect(a, b);
    check_events(a, b, crossing, 1e-7);
    MINI_CHECK(crossing.points.size() == 2);
    MINI_CHECK(crossing.overlaps.empty());
    for (const curve_intersection::Hit& hit : crossing.points) {
        MINI_CHECK(std::abs(hit.point[0] - 0.5) < 1e-7);
        MINI_CHECK(std::abs(std::abs(hit.point[1]) - std::sqrt(0.75)) < 1e-7);
    }
    save_case("Rational Circle Crossing", a, b, crossing);

    const NurbsCurve c = Primitives::circle(0, 2, 0, 1);
    const curve_intersection::Result tangent = curve_intersection::intersect(a, c);
    check_events(a, c, tangent, 1e-7);
    save_case("Rational Circle Tangency", a, c, tangent);
    MINI_CHECK(tangent.points.size() == 1);
    MINI_CHECK(tangent.overlaps.empty());
    MINI_CHECK(tangent.points[0].tangent);
    MINI_CHECK(tangent.points[0].point.distance({0, 1, 0}) < 1e-6);

    const curve_intersection::Result precise = curve_intersection::intersect(a, c, 1e-10);
    check_events(a, c, precise, 1e-10);
    MINI_CHECK(precise.points.size() == 1);
    MINI_CHECK(precise.overlaps.empty());
}

MINI_TEST("CurveIntersection", "Projective Curved Coincidence") {
    NurbsCurve a = Primitives::circle(0, 0, 0, 1);
    MINI_CHECK(a.set_domain(0.0, 1.0));
    MINI_CHECK(a.trim(0.0, 0.25));
    NurbsCurve b = a;
    for (int i = 0; i < b.cv_count(); ++i) {
        const Point point = b.get_cv(i);
        const double weight = b.weight(i) * std::pow(3.0, i);
        MINI_CHECK(b.set_cv_4d(
            i,
            point[0] * weight,
            point[1] * weight,
            point[2] * weight,
            weight
        ));
    }
    MINI_CHECK(b.set_domain(10.0, 20.0));
    MINI_CHECK(b.trim(12.0, 18.0));
    const curve_intersection::Result result = curve_intersection::intersect(a, b);
    check_events(a, b, result, 1e-7);
    save_case("Projective Curved Coincidence", a, b, result);
    MINI_CHECK(result.points.empty());
    MINI_CHECK(result.overlaps.size() == 1);
    MINI_CHECK(std::abs(result.overlaps[0].b0 - 12.0) < 1e-9);
    MINI_CHECK(std::abs(result.overlaps[0].b1 - 18.0) < 1e-9);
}

MINI_TEST("CurveIntersection", "Full Reversed Curved Overlap") {
    const NurbsCurve a = curve_from_points({{-2, 0, 0}, {-1, 2, 1}, {1, -1, 1}, {2, 0, 0}}, 3);
    NurbsCurve b = a;
    MINI_CHECK(b.reverse());
    MINI_CHECK(b.set_domain(17.0, 24.0));
    const curve_intersection::Result result = curve_intersection::intersect(a, b);
    check_events(a, b, result, 1e-7);
    MINI_CHECK(result.points.empty());
    MINI_CHECK(result.overlaps.size() == 1);
    MINI_CHECK(std::abs(result.overlaps[0].a0) < 1e-10);
    MINI_CHECK(std::abs(result.overlaps[0].a1 - 1.0) < 1e-10);
    MINI_CHECK(std::abs(result.overlaps[0].b0 - 24.0) < 1e-10);
    MINI_CHECK(std::abs(result.overlaps[0].b1 - 17.0) < 1e-10);
    save_case("Reversed Curved Overlap", a, b, result);
}

MINI_TEST("CurveIntersection", "Partial Curved Overlap") {
    NurbsCurve a = curve_from_points({{-2, 0, 0}, {-1, 2, 1}, {1, -1, 1}, {2, 0, 0}}, 3);
    NurbsCurve b = a;
    MINI_CHECK(a.trim(0.1, 0.75));
    MINI_CHECK(b.trim(0.5, 0.9));
    MINI_CHECK(b.reverse());
    const curve_intersection::Result result = curve_intersection::intersect(a, b);
    check_events(a, b, result, 1e-7);
    MINI_CHECK(result.points.empty());
    MINI_CHECK(result.overlaps.size() == 1);
    MINI_CHECK(std::abs(result.overlaps[0].a0 - 0.5) < 1e-9);
    MINI_CHECK(std::abs(result.overlaps[0].a1 - 0.75) < 1e-9);
    save_case("Partial Curved Overlap", a, b, result);
}

MINI_TEST("CurveIntersection", "Degree Elevated Rational Coincidence") {
    NurbsCurve a = Primitives::circle(0, 0, 0, 1);
    MINI_CHECK(a.set_domain(0.0, 1.0));
    MINI_CHECK(a.trim(0.03, 0.68));
    NurbsCurve b = a;
    MINI_CHECK(b.increase_degree(4));
    MINI_CHECK(b.insert_nurbsknot(0.32, 2));
    const curve_intersection::Result result = curve_intersection::intersect(a, b);
    check_events(a, b, result, 1e-7);
    MINI_CHECK(result.points.empty());
    MINI_CHECK(result.overlaps.size() == 1);
    MINI_CHECK(std::abs(result.overlaps[0].a0 - 0.03) < 1e-9);
    MINI_CHECK(std::abs(result.overlaps[0].a1 - 0.68) < 1e-9);
    save_case("Elevated Rational Coincidence", a, b, result);

    const NurbsCurve circle = Primitives::circle(0, 0, 0, 1);
    NurbsCurve reversed = circle;
    MINI_CHECK(reversed.reverse());
    const curve_intersection::Result closed = curve_intersection::intersect(circle, reversed);
    check_events(circle, reversed, closed, 1e-7);
    MINI_CHECK(closed.points.empty());
    MINI_CHECK(closed.overlaps.size() == 1);
    MINI_CHECK(closed.overlaps[0].b1 < closed.overlaps[0].b0);
}

MINI_TEST("CurveIntersection", "Shared Endpoints Are Not Coincidence") {
    const NurbsCurve a = curve_from_points({{-1, 0, 0}, {0, 1, 0}, {1, 0, 0}}, 2);
    const NurbsCurve b = curve_from_points({{-1, 0, 0}, {0, -1, 0}, {1, 0, 0}}, 2);
    const curve_intersection::Result result = curve_intersection::intersect(a, b);
    check_events(a, b, result, 1e-7);
    MINI_CHECK(result.points.size() == 2);
    MINI_CHECK(result.overlaps.empty());
    save_case("Shared Endpoints", a, b, result);
}

MINI_TEST("CurveIntersection", "Repeated Knots And Repeated World Point") {
    const NurbsCurve a = curve_from_points({{-1, 0, 0}, {1, 0, 0}, {-1, 0, 0}, {1, 0, 0}});
    const NurbsCurve b = curve_from_points({{0, -1, 0}, {0, 1, 0}});
    const curve_intersection::Result result = curve_intersection::intersect(a, b);
    check_events(a, b, result, 1e-7);
    MINI_CHECK(result.points.size() == 3);
    MINI_CHECK(result.overlaps.empty());
    MINI_CHECK(result.points[1].a - result.points[0].a > 0.2);
    MINI_CHECK(result.points[2].a - result.points[1].a > 0.2);
    save_case("Repeated World Point", a, b, result);

    NurbsCurve smooth = curve_from_points({{-1, 1, 0}, {0, -1, 0}, {1, 1, 0}}, 2);
    MINI_CHECK(smooth.insert_nurbsknot(0.5, 2));
    const NurbsCurve tangent = curve_from_points({{-1, 0, 0}, {1, 0, 0}});
    const curve_intersection::Result knot = curve_intersection::intersect(smooth, tangent);
    check_events(smooth, tangent, knot, 1e-7);
    MINI_CHECK(knot.points.size() == 1);
    MINI_CHECK(knot.overlaps.empty());
}

MINI_TEST("CurveIntersection", "Different Domains And Scales") {
    for (const double scale : {1e-5, 1.0, 1e5}) {
        NurbsCurve a = curve_from_points({{-scale, 0, 0}, {scale, 0, 0}});
        NurbsCurve b = curve_from_points({{0, -scale, 0}, {0, scale, 0}});
        MINI_CHECK(a.set_domain(1e6, 1e6 + 2.0));
        MINI_CHECK(b.set_domain(-1e-10, 3e-10));
        const double tolerance = scale * 1e-7;
        const curve_intersection::Result result = curve_intersection::intersect(a, b, tolerance);
        check_events(a, b, result, tolerance);
        MINI_CHECK(result.points.size() == 1);
        MINI_CHECK(std::abs(result.points[0].a - (1e6 + 1.0)) < 1e-9);
        MINI_CHECK(std::abs(result.points[0].b - 1e-10) < 1e-20);
    }
}

MINI_TEST("CurveIntersection", "Uniform Rational Weight Scales") {
    const NurbsCurve reference = Primitives::circle(0, 0, 0, 1);
    const NurbsCurve line = curve_from_points({{-2, 0, 0}, {2, 0, 0}});
    for (const double scale : {1e-150, 1e150}) {
        NurbsCurve curve = reference;
        for (int i = 0; i < curve.cv_count(); ++i) {
            const Point point = reference.get_cv(i);
            const double weight = reference.weight(i) * scale;
            MINI_CHECK(curve.set_cv_4d(
                i,
                point[0] * weight,
                point[1] * weight,
                point[2] * weight,
                weight
            ));
        }
        const curve_intersection::Result crossing = curve_intersection::intersect(curve, line);
        check_events(curve, line, crossing, 1e-7);
        MINI_CHECK(crossing.points.size() == 3);
        MINI_CHECK(crossing.overlaps.empty());
        const curve_intersection::Result overlap = curve_intersection::intersect(curve, reference);
        check_events(curve, reference, overlap, 1e-7);
        MINI_CHECK(overlap.points.empty());
        MINI_CHECK(overlap.overlaps.size() == 1);
    }
}

MINI_TEST("CurveIntersection", "Invalid Inputs Are Explicit") {
    const NurbsCurve a = curve_from_points({{0, 0, 0}, {1, 0, 0}});
    for (const double tolerance : {0.0, -1.0, std::numeric_limits<double>::infinity()}) {
        bool rejected = false;
        try {
            curve_intersection::intersect(a, a, tolerance);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        MINI_CHECK(rejected);
    }
    NurbsCurve rational = a;
    MINI_CHECK(rational.to_rational());
    MINI_CHECK(rational.set_weight(0, -1.0));
    const curve_intersection::Result unsupported = curve_intersection::intersect(a, rational);
    MINI_CHECK(!unsupported.complete);
    MINI_CHECK(!unsupported.failure.empty());

    const NurbsCurve far = curve_from_points({{1e12, 0, 0}, {1e12 + 1, 0, 0}});
    const curve_intersection::Result resolution = curve_intersection::intersect(far, far, 1e-7);
    MINI_CHECK(!resolution.complete);
    MINI_CHECK(!resolution.failure.empty());
}

MINI_TEST("CurveIntersection", "Preserved Dense Chair Boundary Endpoints") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_chair_face_zero_cuts.json");
    MINI_CHECK(input.good());
    const auto fixture = nlohmann::json::parse(input);
    const double tolerance = 6.954173602564384e-8;
    const std::array<std::array<double, 2>, 2> bounds = {{
        {37.523025442843874, 37.93079963218852},
        {42.00854152563501, 42.41631571497966}}};
    for (size_t i = 0; i < bounds.size(); ++i) {
        NurbsCurve boundary = NurbsCurve::create(false, 1,
            {{bounds[i][0], 0, 0}, {bounds[i][1], 0, 0}}, 2);
        MINI_CHECK(boundary.set_domain(i == 0 ? 31.0 : 42.0, i == 0 ? 32.0 : 43.0));
        NurbsCurve cut = NurbsCurve::jsonload(fixture["cuts"][i]["pcurve"]);
        const auto spans = cut.get_span_vector();
        MINI_CHECK(cut.trim(i == 0 ? spans[spans.size() - 2] : spans[0],
            i == 0 ? spans.back() : spans[1]));
        for (bool reversed : {false, true}) {
            const auto& a = reversed ? cut : boundary;
            const auto& b = reversed ? boundary : cut;
            const auto result = curve_intersection::intersect(a, b, tolerance);
            check_events(a, b, result, tolerance);
            MINI_CHECK(result.points.size() == 1);
            MINI_CHECK(result.overlaps.empty());
            if (result.points.size() == 1) {
                const auto& hit = result.points[0];
                const double endpoint = i == 0 ? cut.domain_end() : cut.domain_start();
                MINI_CHECK(std::abs((reversed ? hit.a : hit.b) - endpoint) < 1e-12);
                MINI_CHECK(boundary.point_at(reversed ? hit.b : hit.a).distance(cut.point_at(endpoint)) <= tolerance);
            }
        }
    }
}

MINI_TEST("CurveIntersection", "Near Parallel Singleton Endpoint") {
    const NurbsCurve a = curve_from_points({{0, 0, 0}, {1, 0, 0}});
    const NurbsCurve b = curve_from_points({{7.0 / 64, 0, 0}, {71.0 / 64, std::ldexp(1.0, -26), 0}});
    const double tolerance = 1e-10;
    for (bool reversed : {false, true}) {
        const auto result = curve_intersection::intersect(reversed ? b : a, reversed ? a : b, tolerance);
        check_events(reversed ? b : a, reversed ? a : b, result, tolerance);
        MINI_CHECK(result.overlaps.empty());
        MINI_CHECK(result.points.size() == 1);
        if (result.points.size() == 1)
            MINI_CHECK(result.points[0].point.distance({7.0 / 64, 0, 0}) <= tolerance);
    }
}

MINI_TEST("CurveIntersection", "Near Parallel Interior Crossing") {
    const NurbsCurve a = curve_from_points({{0, 0, 0}, {1, 0, 0}});
    const NurbsCurve b = curve_from_points({{0, -std::ldexp(7.0, -32), 0},
        {1, std::ldexp(57.0, -32), 0}});
    const double tolerance = 1e-10;
    for (bool reversed : {false, true}) {
        const auto result = curve_intersection::intersect(reversed ? b : a, reversed ? a : b, tolerance);
        check_events(reversed ? b : a, reversed ? a : b, result, tolerance);
        MINI_CHECK(result.overlaps.empty());
        MINI_CHECK(result.points.size() == 1);
        if (result.points.size() == 1)
            MINI_CHECK(result.points[0].point.distance({7.0 / 64, 0, 0}) <= tolerance);
    }
}

MINI_TEST("CurveIntersection", "Preserved Windows Dodecahedron Boundary Endpoint") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_windows_dodeca_endpoint20_pair.json");
    MINI_CHECK(input.good());
    const auto fixture = nlohmann::json::parse(input);
    const NurbsCurve boundary = NurbsCurve::jsonload(fixture["boundary"]);
    const NurbsCurve cut = NurbsCurve::jsonload(fixture["cut"]);
    const double tolerance = fixture["tolerance"];
    const double endpoint = cut.domain_end();
    for (bool reversed : {false, true}) {
        const auto& a = reversed ? cut : boundary;
        const auto& b = reversed ? boundary : cut;
        const auto result = curve_intersection::intersect(a, b, tolerance);
        check_events(a, b, result, tolerance);
        MINI_CHECK(result.points.size() == 1);
        MINI_CHECK(result.overlaps.empty());
        if (result.points.size() == 1) {
            const auto& hit = result.points[0];
            const double on_cut = reversed ? hit.a : hit.b;
            const double on_boundary = reversed ? hit.b : hit.a;
            MINI_CHECK(std::abs(on_cut - endpoint) < 1e-12);
            MINI_CHECK(on_boundary > boundary.domain_start() && on_boundary < boundary.domain_end());
            MINI_CHECK(boundary.point_at(on_boundary).distance(cut.point_at(endpoint)) <= tolerance);
        }
    }
}


MINI_TEST("CurveIntersection", "Flat Single Span Distinct Parameter Crossings") {
    const double tolerance = std::ldexp(1.0, -20);
    const double scale = std::ldexp(1.0, -24);
    const NurbsCurve a = curve_from_points({{0, 3 * scale / 16, 0},
        {0.5, -5 * scale / 16, 0}, {1, 3 * scale / 16, 0}}, 2);
    const NurbsCurve b = curve_from_points({{0, 0, 0}, {1, 0, 0}});
    MINI_CHECK(scale / 2 < tolerance * 0.05);
    for (bool reversed : {false, true}) {
        const auto result = curve_intersection::intersect(reversed ? b : a, reversed ? a : b, tolerance);
        check_events(reversed ? b : a, reversed ? a : b, result, tolerance);
        bool first = false, second = false, range = false;
        for (const auto& hit : result.points) {
            first = first || (std::abs(hit.a - 0.25) <= tolerance && std::abs(hit.b - 0.25) <= tolerance);
            second = second || (std::abs(hit.a - 0.75) <= tolerance && std::abs(hit.b - 0.75) <= tolerance);
        }
        for (const auto& shared : result.overlaps) {
            const bool covers = shared.a0 <= 0.25 && shared.a1 >= 0.75 &&
                shared.b0 <= 0.25 && shared.b1 >= 0.75;
            const bool bounded = shared.a0 >= 0 && shared.a1 <= 1 && shared.b0 >= 0 && shared.b1 <= 1;
            const double shift = std::max(std::abs(shared.a0 - shared.b0), std::abs(shared.a1 - shared.b1));
            const bool correspondence = std::hypot(shift, 3 * scale / 16) <= tolerance;
            range = range || (covers && bounded && correspondence);
        }
        MINI_CHECK((first && second) || range);
    }
}

MINI_TEST("CurveIntersection", "Certified Flat Range Raw Domains And Reversal") {
    const double tolerance = std::ldexp(1.0, -20);
    const double scale = std::ldexp(1.0, -24);
    NurbsCurve a = curve_from_points({{0, 3 * scale / 16, 0},
        {0.5, -5 * scale / 16, 0}, {1, 3 * scale / 16, 0}}, 2);
    NurbsCurve b = curve_from_points({{0, 0, 0}, {1, 0, 0}});
    MINI_CHECK(a.to_rational() && b.to_rational());
    for (NurbsCurve* curve : {&a, &b})
        for (int i = 0; i < curve->cv_count(); ++i) {
            const Point point = curve->get_cv(i);
            const double weight = curve == &a ? 2 : 3;
            MINI_CHECK(curve->set_cv_4d(i, point[0] * weight, point[1] * weight, 0, weight));
        }
    MINI_CHECK(a.set_domain(-7, 13) && b.set_domain(32, 35));
    MINI_CHECK(b.reverse());
    for (bool swapped : {false, true}) {
        const auto& first = swapped ? b : a;
        const auto& second = swapped ? a : b;
        const auto result = curve_intersection::intersect(first, second, tolerance);
        check_events(first, second, result, tolerance);
        MINI_CHECK(result.points.empty());
        MINI_CHECK(result.overlaps.size() == 1);
        if (result.overlaps.size() == 1) {
            const auto& range = result.overlaps[0];
            MINI_CHECK(range.a0 == first.domain_start() && range.a1 == first.domain_end());
            MINI_CHECK(range.b0 == second.domain_end() && range.b1 == second.domain_start());
        }
    }
}

MINI_TEST("CurveIntersection", "Flat Range Beyond Tolerance Is Rejected") {
    const double tolerance = std::ldexp(1.0, -20);
    const NurbsCurve a = curve_from_points({{0, 0, 0}, {1, 0, 0}});
    for (bool crossing : {false, true}) {
        const NurbsCurve b = curve_from_points({{0, crossing ? -2 * tolerance : 2 * tolerance, 0},
            {1, 2 * tolerance, 0}});
        for (bool swapped : {false, true}) {
            const auto& first = swapped ? b : a;
            const auto& second = swapped ? a : b;
            const auto result = curve_intersection::intersect(first, second, tolerance);
            check_events(first, second, result, tolerance);
            MINI_CHECK(result.overlaps.empty());
            MINI_CHECK(result.points.size() == (crossing ? 1 : 0));
            if (crossing && result.points.size() == 1)
                MINI_CHECK(result.points[0].point.distance({0.5, 0, 0}) <= tolerance);
        }
    }
}

MINI_TEST("CurveIntersection", "Extended Line Retains Both Flat Parameter Branches") {
    const double tolerance = std::ldexp(1.0, -20);
    const double scale = std::ldexp(1.0, -24);
    const NurbsCurve a = curve_from_points({{0, 3 * scale / 16, 0},
        {0.5, -5 * scale / 16, 0}, {1, 3 * scale / 16, 0}}, 2);
    const NurbsCurve b = curve_from_points({{-1, 0, 0}, {2, 0, 0}});
    MINI_CHECK(scale / 2 < tolerance * 0.05);
    for (bool swapped : {false, true}) {
        const auto& first = swapped ? b : a;
        const auto& second = swapped ? a : b;
        const auto result = curve_intersection::intersect(first, second, tolerance);
        check_events(first, second, result, tolerance);
        bool first_root = false, second_root = false, clipped_range = false;
        for (const auto& hit : result.points) {
            const double ta = swapped ? hit.b : hit.a;
            const double tb = swapped ? hit.a : hit.b;
            first_root = first_root || (std::abs(ta - 0.25) <= tolerance && std::abs(tb - 5.0 / 12) <= tolerance);
            second_root = second_root || (std::abs(ta - 0.75) <= tolerance && std::abs(tb - 7.0 / 12) <= tolerance);
        }
        for (const auto& shared : result.overlaps) {
            const double a0 = swapped ? shared.b0 : shared.a0;
            const double a1 = swapped ? shared.b1 : shared.a1;
            const double b0 = swapped ? shared.a0 : shared.b0;
            const double b1 = swapped ? shared.a1 : shared.b1;
            const bool bounded = a0 >= 0 && a1 <= 1 && b0 >= 0 && b1 <= 1;
            const bool covers = a0 <= 0.25 && a1 >= 0.75 && b0 <= 5.0 / 12 && b1 >= 7.0 / 12;
            const double shift = std::max(std::abs(a0 - (-1 + 3 * b0)), std::abs(a1 - (-1 + 3 * b1)));
            clipped_range = clipped_range || (bounded && covers && std::hypot(shift, 3 * scale / 16) <= tolerance);
        }
        MINI_CHECK((first_root && second_root) || clipped_range);
    }
}

MINI_TEST("CurveIntersection", "Clipped Range Keeps Raw Domains And Reversal") {
    const double tolerance = std::ldexp(1.0, -20);
    const double scale = std::ldexp(1.0, -24);
    NurbsCurve a = curve_from_points({{0, 3 * scale / 16, 0},
        {0.5, -5 * scale / 16, 0}, {1, 3 * scale / 16, 0}}, 2);
    NurbsCurve b = curve_from_points({{-1, 0, 0}, {2, 0, 0}});
    MINI_CHECK(a.to_rational() && b.to_rational());
    for (NurbsCurve* curve : {&a, &b})
        for (int i = 0; i < curve->cv_count(); ++i) {
            const Point point = curve->get_cv(i);
            const double weight = curve == &a ? 2 : 3;
            MINI_CHECK(curve->set_cv_4d(i, point[0] * weight, point[1] * weight, 0, weight));
        }
    MINI_CHECK(a.set_domain(-7, 13) && b.set_domain(32, 35) && b.reverse());
    for (bool swapped : {false, true}) {
        const auto& first = swapped ? b : a;
        const auto& second = swapped ? a : b;
        const auto result = curve_intersection::intersect(first, second, tolerance);
        check_events(first, second, result, tolerance);
        MINI_CHECK(result.overlaps.size() == 1);
        if (result.overlaps.size() == 1) {
            const auto& shared = result.overlaps[0];
            MINI_CHECK(std::abs(shared.a0 - (swapped ? 33 : -7)) <= tolerance);
            MINI_CHECK(std::abs(shared.a1 - (swapped ? 34 : 13)) <= tolerance);
            MINI_CHECK(std::abs(shared.b0 - (swapped ? 13 : 34)) <= tolerance);
            MINI_CHECK(std::abs(shared.b1 - (swapped ? -7 : 33)) <= tolerance);
        }
    }
}

MINI_TEST("CurveIntersection", "Close Endpoints Cannot Certify Distant Interior") {
    const double tolerance = std::ldexp(1.0, -20);
    const double end = 127 * tolerance / 128, middle = 133 * tolerance / 128;
    const NurbsCurve a = curve_from_points({{0, end, 0}, {0.5, middle, 0}, {1, end, 0}}, 2);
    const NurbsCurve b = curve_from_points({{-1, 0, 0}, {2, 0, 0}});
    MINI_CHECK(end < tolerance && middle - end < tolerance * 0.05);
    MINI_CHECK(a.point_at(0.5).distance({0.5, 0, 0}) > tolerance);
    for (bool swapped : {false, true}) {
        const auto& first = swapped ? b : a;
        const auto& second = swapped ? a : b;
        const auto result = curve_intersection::intersect(first, second, tolerance);
        check_events(first, second, result, tolerance);
        MINI_CHECK(result.overlaps.empty());
    }
}



MINI_TEST("CurveIntersection", "Dense Knot Spans Preserve Sparse Events") {
    for (int variant = 0; variant < 3; ++variant) {
        std::vector<Point> horizontal, vertical;
        for (int i = 0; i <= 512; ++i) {
            const double t = -1 + i / 256.0;
            horizontal.push_back({t, 0, 0});
            vertical.push_back(variant == 1 ? Point(t, 0, 1) : Point(.125, t, 0));
        }
        NurbsCurve a = curve_from_points(horizontal, 3), b = curve_from_points(vertical, 3);
        MINI_CHECK(a.get_span_vector().size() > 500 && b.get_span_vector().size() > 500);
        MINI_CHECK(a.set_domain(-7, 13) && b.set_domain(32, 35));
        if (variant == 2) {
            MINI_CHECK(b.reverse() && a.to_rational() && b.to_rational());
            for (NurbsCurve* curve : {&a, &b})
                for (int i = 0; i < curve->cv_count(); ++i) {
                    const Point point = curve->get_cv(i);
                    MINI_CHECK(curve->set_cv_4d(i, 2*point[0], 2*point[1], 2*point[2], 2));
                }
        }
        const auto result = curve_intersection::intersect(a, b, 1e-7);
        check_events(a, b, result, 1e-7);
        MINI_CHECK(result.overlaps.empty());
        MINI_CHECK(result.points.size() == (variant == 1 ? 0 : 1));
        if (variant != 1 && result.points.size() == 1) {
            MINI_CHECK(result.points[0].point.distance({.125, 0, 0}) <= 1e-7);
            MINI_CHECK(!result.points[0].tangent);
        }
    }
}


MINI_TEST("CurveIntersection", "Certified Disjoint Before Coordinate Resolution") {
    using namespace session_cpp;
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_hollow_affine_disjoint_curve_pair.json");
    MINI_CHECK(input.good());
    const auto fixture = nlohmann::ordered_json::parse(input);
    const NurbsCurve a = NurbsCurve::jsonload(fixture.at("section_pcurve"));
    const NurbsCurve b = NurbsCurve::jsonload(fixture.at("boundary_pcurve"));
    const double tolerance = fixture.at("uv_tolerance");
    MINI_CHECK(a.cv_count() == 2053 && b.cv_count() == 2);
    // Independent actual geometry: all section poles are more than 0.1 UV above the line.
    for (int i = 0; i < a.cv_count(); ++i)
        MINI_CHECK(a.get_cv(i)[1] - b.get_cv(0)[1] > 0.1);
    for (bool swapped : {false, true}) {
        const auto events = curve_intersection::intersect(swapped ? b : a, swapped ? a : b, tolerance);
        MINI_CHECK(events.complete && events.failure.empty());
        MINI_CHECK(events.points.empty() && events.overlaps.empty());
    }
    // A separated positive rational hull can prove absence even at a huge coordinate origin.
    NurbsCurve first = NurbsCurve::create(false, 1, {{0x1p40, 0, 0}, {0x1p40+1, 0, 0}});
    NurbsCurve second = NurbsCurve::create(false, 1, {{0x1p40, 1, 0}, {0x1p40+1, 1, 0}});
    MINI_CHECK(second.to_rational());
    for (int i = 0; i < second.cv_count(); ++i) {
        const Point point = second.get_cv(i);
        const double weight = i ? 3 : 2;
        MINI_CHECK(second.set_cv_4d(i, point[0]*weight, point[1]*weight, 0, weight));
    }
    const auto absent = curve_intersection::intersect(first, second, 1e-7);
    MINI_CHECK(absent.complete && absent.points.empty() && absent.overlaps.empty());
    // Overlapping hulls MUST retain the unresolved coordinate-resolution contract.
    const auto unresolved = curve_intersection::intersect(first, first, 1e-7);
    MINI_CHECK(!unresolved.complete && !unresolved.failure.empty());
}

}
