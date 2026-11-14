#include "nurbs_coincidence.h"
#include "nurbs_interval.h"
#include "nurbs_memo.h"
#include "curve_surface_intersection.h"
#include "brep.h"
#include "intersection.h"
#include "xform.h"
#include "mini_test.h"
#include "primitives.h"
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>

using namespace session_cpp;
using namespace session_cpp::mini_test;

namespace {

NurbsSurface test_plane() {
    return NurbsSurface::create(
        false,
        false,
        1,
        1,
        2,
        2,
        {{-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}, {1, 1, 0}}
    );
}

NurbsSurface test_bowl() {
    std::vector<Point> points;
    const std::array<double, 3> heights = {1.0, -1.0, 1.0};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            points.push_back({i - 1.0, j - 1.0, heights[i] + heights[j]});
    return NurbsSurface::create(false, false, 2, 2, 3, 3, points);
}

void save_regression(
    const std::string& name,
    const NurbsCurve& curve,
    const NurbsSurface& surface,
    const curve_surface_intersection::Result& result,
    double tolerance
) {
    nlohmann::ordered_json item;
    item["complete"] = result.complete;
    item["curve"] = curve.jsondump();
    item["failure"] = result.failure;
    item["points"] = nlohmann::ordered_json::array();
    for (const curve_surface_intersection::Hit& hit : result.points)
        item["points"].push_back({{"t", hit.t}, {"u", hit.u}, {"v", hit.v}, {"tangent", hit.tangent},
            {"gap", curve.point_at(hit.t).distance(surface.point_at(hit.u, hit.v))}});
    item["surface"] = surface.jsondump();
    item["tolerance"] = tolerance;
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    const std::filesystem::path folder = root / "build/native-geometry-tests";
    std::filesystem::create_directories(folder);
    std::ofstream output(folder / ("curve_surface_" + name + ".json"));
    output << item.dump(2);
}

void check_lifts(
    const curve_surface_intersection::Result& result,
    const NurbsCurve& curve,
    const NurbsSurface& surface,
    double tolerance
) {
    if (!result.complete)
        throw std::runtime_error(result.failure);
    for (const curve_surface_intersection::Hit& hit : result.points) {
        MINI_CHECK(hit.point.distance(curve.point_at(hit.t)) <= tolerance);
        MINI_CHECK(hit.point.distance(surface.point_at(hit.u, hit.v)) <= tolerance);
    }
    for (const curve_surface_intersection::Overlap& overlap : result.overlaps)
        for (int i = 0; i <= 32; ++i) {
            const double t = overlap.t0 + (overlap.t1 - overlap.t0) * i / 32.0;
            const Point uv = overlap.pcurve.point_at(t);
            MINI_CHECK(curve.point_at(t).distance(surface.point_at(uv[0], uv[1])) <= tolerance);
        }
}

void save_parameter_regression(
    const std::string& name,
    const NurbsCurve& curve,
    const NurbsCurve& pcurve,
    const NurbsSurface& surface,
    curve_surface_intersection::ParameterMatch match,
    double tolerance = 1e-7
) {
    NurbsCurve normalized = pcurve;
    normalized.set_domain(0, 1);
    const nlohmann::ordered_json item = {{"curve", curve.jsondump()}, {"pcurve", pcurve.jsondump()},
        {"surface", surface.jsondump()}, {"match", static_cast<int>(match)}, {"tolerance", tolerance},
        {"normalized_pcurve_domain", {normalized.domain_start(), normalized.domain_end()}}};
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    const std::filesystem::path folder = root / "build/native-geometry-tests";
    std::filesystem::create_directories(folder);
    std::ofstream output(folder / ("same_parameter_" + name + ".json"));
    output << item.dump(2);
}

}

MINI_TEST("Curve Surface Native", "Planar Crossings And Tangency") {
    const NurbsSurface surface = test_plane();
    const NurbsCurve crossing = NurbsCurve::create(false, 1, {{0.2, 0.3, -1}, {0.2, 0.3, 1}});
    const curve_surface_intersection::Result first = curve_surface_intersection::intersect(crossing, surface);
    check_lifts(first, crossing, surface, 1e-10);
    MINI_CHECK(first.points.size() == 1 && first.overlaps.empty());
    MINI_CHECK(!first.points[0].tangent);
    const NurbsCurve tangent = NurbsCurve::create(false, 2, {{-0.5, 0, 0.25}, {0, 0, -0.25}, {0.5, 0, 0.25}});
    const curve_surface_intersection::Result second = curve_surface_intersection::intersect(tangent, surface);
    check_lifts(second, tangent, surface, 1e-10);
    MINI_CHECK(second.points.size() == 1 && second.overlaps.empty());
    MINI_CHECK(second.points[0].tangent);
    MINI_CHECK(second.points[0].point.distance({0, 0, 0}) < 1e-10);
}

MINI_TEST("Curve Surface Native", "Clipped Planar Overlap And Corner") {
    const NurbsSurface surface = test_plane();
    const NurbsCurve line = NurbsCurve::create(false, 1, {{-2, 0.2, 0}, {2, 0.2, 0}});
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(line, surface);
    check_lifts(result, line, surface, 1e-10);
    MINI_CHECK(result.points.empty() && result.overlaps.size() == 1);
    MINI_CHECK(std::abs(line.point_at(result.overlaps[0].t0)[0] + 1.0) < 1e-10);
    MINI_CHECK(std::abs(line.point_at(result.overlaps[0].t1)[0] - 1.0) < 1e-10);
    const NurbsCurve corner = NurbsCurve::create(false, 1, {{0, 2, 0}, {2, 0, 0}});
    const curve_surface_intersection::Result contact = curve_surface_intersection::intersect(corner, surface);
    check_lifts(contact, corner, surface, 1e-10);
    MINI_CHECK(contact.points.size() == 1 && contact.overlaps.empty());
    MINI_CHECK(contact.points[0].point.distance({1, 1, 0}) < 1e-10);
}

MINI_TEST("Curve Surface Native", "Same Point Distinct Parameters") {
    const NurbsSurface surface = test_plane();
    const NurbsCurve curve = NurbsCurve::create(false, 1, {{0, 0, -1}, {0, 0, 1}, {0, 0, -1}});
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(curve, surface);
    check_lifts(result, curve, surface, 1e-10);
    MINI_CHECK(result.points.size() == 2);
    MINI_CHECK(result.points[0].t < result.points[1].t);
    MINI_CHECK(result.points[0].point.distance(result.points[1].point) < 1e-10);
}

MINI_TEST("Curve Surface Native", "Rational Planar Overlap Weights") {
    NurbsSurface surface = test_plane();
    MINI_CHECK(surface.set_domain(0, 10, 20));
    MINI_CHECK(surface.set_domain(1, -30, -10));
    NurbsCurve curve = NurbsCurve::create(false, 2, {{-0.75, 0, 0}, {0, 0.8, 0}, {0.75, 0, 0}});
    MINI_CHECK(curve.set_weight(1, 0.35));
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(curve, surface);
    check_lifts(result, curve, surface, 1e-10);
    MINI_CHECK(result.points.empty() && result.overlaps.size() == 1);
    MINI_CHECK(std::abs(result.overlaps[0].pcurve.weight(1) - 0.35) < 1e-12);
}

MINI_TEST("Curve Surface Native", "Freeform Multiple Roots") {
    const NurbsSurface surface = test_bowl();
    const NurbsCurve curve = NurbsCurve::create(false, 1, {{-1, 0.2, 0.29}, {1, 0.2, 0.29}});
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(curve, surface, 1e-7);
    check_lifts(result, curve, surface, 1e-7);
    MINI_CHECK(result.points.size() == 2);
    MINI_CHECK(result.points[0].point.distance({-0.5, 0.2, 0.29}) < 1e-7);
    MINI_CHECK(result.points[1].point.distance({0.5, 0.2, 0.29}) < 1e-7);
    MINI_CHECK(!result.points[0].tangent && !result.points[1].tangent);
}

MINI_TEST("Curve Surface Native", "Freeform Tangency And Near Miss") {
    const NurbsSurface surface = test_bowl();
    const NurbsCurve tangent = NurbsCurve::create(false, 1, {{-1, 0, 0}, {1, 0, 0}});
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(tangent, surface, 1e-6);
    check_lifts(result, tangent, surface, 1e-6);
    MINI_CHECK(result.points.size() == 1 && result.overlaps.empty());
    MINI_CHECK(result.points[0].point.distance({0, 0, 0}) < 1e-6);
    MINI_CHECK(result.points[0].tangent);
    const NurbsCurve miss = NurbsCurve::create(false, 1, {{-1, 0, -1e-4}, {1, 0, -1e-4}});
    const curve_surface_intersection::Result empty = curve_surface_intersection::intersect(miss, surface, 1e-7);
    MINI_CHECK(empty.complete && empty.points.empty() && empty.overlaps.empty());
}

MINI_TEST("Curve Surface Native", "Off Grid Tangency And Nearby Roots") {
    const NurbsSurface surface = test_bowl();
    const NurbsCurve tangent = NurbsCurve::create(false, 1, {{-1, 0.2, 0.04}, {1, 0.2, 0.04}});
    for (const double tolerance : {1e-6, 1e-8}) {
        const curve_surface_intersection::Result contact = curve_surface_intersection::intersect(tangent, surface, tolerance);
        save_regression(
            tolerance == 1e-6 ? "off_grid_tangent_1e-6" : "off_grid_tangent_1e-8",
            tangent,
            surface,
            contact,
            tolerance
        );
        if (!contact.complete)
            throw std::runtime_error(std::string(tolerance == 1e-6 ? "tol 1e-6: " : "tol 1e-8: ") + contact.failure);
        check_lifts(contact, tangent, surface, tolerance);
        MINI_CHECK(contact.points.size() == 1);
        MINI_CHECK(contact.overlaps.empty());
        MINI_CHECK(contact.points[0].tangent);
        MINI_CHECK(contact.points[0].point.distance({0, 0.2, 0.04}) < tolerance);
    }
    const NurbsCurve miss = NurbsCurve::create(false, 1, {{-1, 0.2, 0.03999}, {1, 0.2, 0.03999}});
    const curve_surface_intersection::Result empty = curve_surface_intersection::intersect(miss, surface, 1e-6);
    MINI_CHECK(empty.complete && empty.points.empty() && empty.overlaps.empty());

    const NurbsCurve close = NurbsCurve::create(false, 1, {{-1, 0.2, 0.04000001}, {1, 0.2, 0.04000001}});
    const curve_surface_intersection::Result roots = curve_surface_intersection::intersect(close, surface, 1e-7);
    save_regression("off_grid_close_roots", close, surface, roots, 1e-7);
    check_lifts(roots, close, surface, 1e-7);
    MINI_CHECK(roots.points.size() == 2 && roots.overlaps.empty());
    MINI_CHECK(std::abs(roots.points[0].point[0] + 1e-4) < 1e-7);
    MINI_CHECK(std::abs(roots.points[1].point[0] - 1e-4) < 1e-7);
}

MINI_TEST("Curve Surface Native", "Tangency Reparameterized Domains") {
    NurbsSurface surface = test_bowl();
    MINI_CHECK(surface.set_domain(0, 1000.0, 1000.01));
    MINI_CHECK(surface.set_domain(1, -3e-9, 5e-9));
    NurbsCurve curve = NurbsCurve::create(false, 1, {{-1, 0.2, 0.04}, {1, 0.2, 0.04}});
    MINI_CHECK(curve.set_domain(-20.0, 40.0));
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(curve, surface, 1e-6);
    save_regression("off_grid_shifted_domains", curve, surface, result, 1e-6);
    check_lifts(result, curve, surface, 1e-6);
    MINI_CHECK(result.points.size() == 1 && result.overlaps.empty());
    MINI_CHECK(result.points[0].tangent);
    MINI_CHECK(std::abs(result.points[0].t - 10.0) < 1e-6);
    MINI_CHECK(std::abs(result.points[0].u - 1000.005) < 1e-9);
    MINI_CHECK(std::abs(result.points[0].v - 1.8e-9) < 1e-15);
}

MINI_TEST("Curve Surface Native", "Rational Freeform Crossing") {
    NurbsSurface surface(
        3,
        true,
        3,
        3,
        3,
        3
    );
    const NurbsSurface bowl = test_bowl();
    for (int axis = 0; axis < 2; ++axis)
        for (int i = 0; i < surface.nurbsknot_count(axis); ++i)
            MINI_CHECK(surface.set_nurbsknot(axis, i, bowl.nurbsknot(axis, i)));
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            const Point point = bowl.get_cv(i, j);
            const double weight = 1.0 + 0.2 * i + 0.15 * j;
            MINI_CHECK(surface.set_cv_4d(i, j, point[0] * weight, point[1] * weight, point[2] * weight, weight));
        }
    const Point target = surface.point_at(0.37, 0.61);
    const NurbsCurve curve = NurbsCurve::create(false, 1, {target + Vector(0, 0, -3), target + Vector(0, 0, 3)});
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(curve, surface, 1e-7);
    check_lifts(result, curve, surface, 1e-7);
    MINI_CHECK(result.points.size() == 1 && result.overlaps.empty());
    MINI_CHECK(result.points[0].point.distance(target) < 1e-7);
}

MINI_TEST("Curve Surface Native", "Exact Isoparametric Overlap") {
    const NurbsSurface surface = test_bowl();
    NurbsCurve curve = surface.iso_curve(0, 0.31);
    MINI_CHECK(curve.set_domain(12.0, 18.0));
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(curve, surface, 1e-7);
    check_lifts(result, curve, surface, 1e-10);
    MINI_CHECK(result.points.empty() && result.overlaps.size() == 1);
    MINI_CHECK(result.overlaps[0].t0 == 12.0 && result.overlaps[0].t1 == 18.0);
    MINI_CHECK(curve.reverse());
    const curve_surface_intersection::Result reversed = curve_surface_intersection::intersect(curve, surface, 1e-7);
    check_lifts(reversed, curve, surface, 1e-10);
    MINI_CHECK(reversed.points.empty() && reversed.overlaps.size() == 1);
}

MINI_TEST("Curve Surface Native", "Narrow Parameter Domains") {
    NurbsSurface surface = test_bowl();
    MINI_CHECK(surface.set_domain(0, 2e-12, 3e-12));
    MINI_CHECK(surface.set_domain(1, -8e-12, -7e-12));
    const Point target = surface.point_at(2.37e-12, -7.39e-12);
    NurbsCurve curve = NurbsCurve::create(false, 1, {target + Vector(0, 0, -2), target + Vector(0, 0, 2)});
    MINI_CHECK(curve.set_domain(8e-12, 9e-12));
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(curve, surface, 1e-6);
    check_lifts(result, curve, surface, 1e-6);
    MINI_CHECK(result.points.size() == 1);
    MINI_CHECK(std::abs(result.points[0].t - 8.5e-12) < 1e-19);
    MINI_CHECK(std::abs(result.points[0].u - 2.37e-12) < 1e-19);
    MINI_CHECK(std::abs(result.points[0].v + 7.39e-12) < 1e-19);
}

MINI_TEST("Curve Surface Native", "Partial Isoparametric Overlap") {
    const NurbsSurface surface = test_bowl();
    NurbsCurve curve = surface.iso_curve(1, 0.42);
    MINI_CHECK(curve.trim(0.17, 0.83));
    MINI_CHECK(curve.reverse());
    MINI_CHECK(curve.set_domain(-8.0, -3.0));
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(curve, surface);
    check_lifts(result, curve, surface, 1e-10);
    MINI_CHECK(result.points.empty() && result.overlaps.size() == 1);
    MINI_CHECK(result.overlaps[0].t0 == -8.0 && result.overlaps[0].t1 == -3.0);
}

MINI_TEST("Curve Surface Native", "Distinct Surface Parameters") {
    const NurbsSurface surface = NurbsSurface::create(
        false,
        false,
        2,
        1,
        3,
        2,
        {{1, -1, 0}, {1, 1, 0}, {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}, {1, 1, 0}}
    );
    const NurbsCurve curve = NurbsCurve::create(false, 1, {{0.25, 0.3, -1}, {0.25, 0.3, 1}});
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(curve, surface);
    check_lifts(result, curve, surface, 1e-7);
    MINI_CHECK(result.points.size() == 2);
    MINI_CHECK(std::abs(result.points[0].t - result.points[1].t) < 1e-7);
    MINI_CHECK(std::abs(result.points[0].u - result.points[1].u) > 0.4);
}

MINI_TEST("Curve Surface Native", "Certified Nonlinear Graph Overlap") {
    const NurbsSurface surface = test_bowl();
    const NurbsCurve curve = NurbsCurve::create(false, 2, {{-1, -1, 2}, {0, 0, -2}, {1, 1, 2}});
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(curve, surface);
    check_lifts(result, curve, surface, 1e-10);
    MINI_CHECK(result.complete && result.points.empty());
    MINI_CHECK(result.overlaps.size() == 1);
}

MINI_TEST("Curve Surface Native", "Rational Nonlinear Graph Overlap") {
    const NurbsSurface surface = test_bowl();
    NurbsCurve curve(
        3,
        true,
        5,
        5
    );
    for (int i = 0; i < curve.nurbsknot_count(); ++i)
        MINI_CHECK(curve.set_nurbsknot(i, i < 4 ? 0.0 : 1.0));
    const std::array<double, 5> weights = {1.0, 0.5, 0.5, 0.5, 1.0};
    const std::array<double, 5> xs = {-1.0, -0.25, 0.0, 0.25, 1.0};
    const std::array<double, 5> zs = {2.0, 0.0, -2.0 / 3.0, 0.0, 2.0};
    for (int i = 0; i < 5; ++i)
        MINI_CHECK(curve.set_cv_4d(i, xs[i], xs[i], zs[i], weights[i]));
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(curve, surface);
    check_lifts(result, curve, surface, 1e-10);
    MINI_CHECK(result.points.empty() && result.overlaps.size() == 1);
}

MINI_TEST("Curve Surface Native", "Unsupported Coincidence Is Explicit") {
    NurbsSurface surface = test_bowl();
    MINI_CHECK(surface.to_rational());
    MINI_CHECK(surface.set_weight(1, 1, 2.0));
    NurbsCurve curve(
        3,
        true,
        5,
        5
    );
    for (int i = 0; i < curve.nurbsknot_count(); ++i)
        MINI_CHECK(curve.set_nurbsknot(i, i < 4 ? 0.0 : 1.0));
    const std::array<double, 5> heights = {2, 0, -2, 0, 2};
    for (int i = 0; i < 5; ++i)
        MINI_CHECK(curve.set_cv_4d(i, i * 0.5 - 1.0, i * 0.5 - 1.0, heights[i], i == 2 ? 5.0 / 3.0 : 1.0));
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(curve, surface);
    MINI_CHECK(!result.complete && !result.failure.empty());
    MINI_CHECK(result.overlaps.empty());
}

MINI_TEST("Curve Surface Native", "Rational Overflow Is Not A Hit") {
    NurbsCurve curve = NurbsCurve::create(false, 2, {{0, 0, -1}, {0, 0, 0}, {0, 0, 1}});
    MINI_CHECK(curve.set_weight(1, 1e300));
    const NurbsSurface surface = test_bowl();
    const curve_surface_intersection::Result result = curve_surface_intersection::intersect(curve, surface);
    for (const curve_surface_intersection::Hit& hit : result.points) {
        MINI_CHECK(std::isfinite(hit.t) && std::isfinite(hit.u) && std::isfinite(hit.v));
        MINI_CHECK(std::isfinite(hit.point[0]) && std::isfinite(hit.point[1]) && std::isfinite(hit.point[2]));
        MINI_CHECK(hit.point.distance(surface.point_at(hit.u, hit.v)) < 1e-7);
    }
    MINI_CHECK(!result.complete || !result.points.empty());
}

MINI_TEST("Curve Surface Native", "Same Parameter Certificates") {
    const NurbsSurface plane = test_plane();
    NurbsCurve pcurve = NurbsCurve::create(false, 1, {{0.1, 0.2, 0}, {0.9, 0.8, 0}});
    NurbsCurve curve = NurbsCurve::create(false, 1,
        {plane.point_at(0.1, 0.2), plane.point_at(0.9, 0.8)});
    MINI_CHECK(curve.set_domain(-5, 7));
    MINI_CHECK(pcurve.set_domain(-1e-9, 3e-9));
    const curve_surface_intersection::ParameterMatch match =
        curve_surface_intersection::same_parameter(curve, pcurve, plane);
    save_parameter_regression("tiny_degree_one_domain", curve, pcurve, plane, match);
    MINI_CHECK(match == curve_surface_intersection::ParameterMatch::Matched);
    const NurbsCurve near = curve.transformed(Xform::translation(0, 0, 7e-13));
    MINI_CHECK(curve_surface_intersection::same_parameter(near, pcurve, plane, 1e-12) ==
        curve_surface_intersection::ParameterMatch::Matched);

    const NurbsCurve wrong = NurbsCurve::create(false, 2, {{0.1, 0.2, 0}, {0.7, 0.5, 0}, {0.9, 0.8, 0}});
    MINI_CHECK(curve_surface_intersection::same_parameter(curve, wrong, plane) ==
        curve_surface_intersection::ParameterMatch::Mismatched);
    NurbsSurface surface = test_bowl();
    MINI_CHECK(surface.to_rational());
    MINI_CHECK(surface.set_weight(1, 1, 2.0));
    const NurbsCurve iso = surface.iso_curve(0, 0.37);
    const NurbsCurve uv = NurbsCurve::create(false, 1, {{0, 0.37, 0}, {1, 0.37, 0}});
    MINI_CHECK(curve_surface_intersection::same_parameter(iso, uv, surface) ==
        curve_surface_intersection::ParameterMatch::Matched);
    NurbsCurve weighted = pcurve;
    MINI_CHECK(weighted.to_rational());
    MINI_CHECK(weighted.set_weight(1, -1.0));
    MINI_CHECK(curve_surface_intersection::same_parameter(curve, weighted, plane) ==
        curve_surface_intersection::ParameterMatch::Unresolved);
}

MINI_TEST("Curve Surface Native", "Same Parameter Straight Edge On A Flat Face") {
    const NurbsSurface face = test_plane().transformed(Xform::translation(2000, -2000, 2000) * Xform::rotation(Vector(1, 2, 3), 0.7));
    const NurbsCurve pcurve = NurbsCurve::create(false, 1, {{0.1, 0.2, 0}, {0.9, 0.8, 0}});
    const NurbsCurve edge = NurbsCurve::create(false, 1, {face.point_at(0.1, 0.2), face.point_at(0.9, 0.8)});
    std::string diagnostic = "stale";
    MINI_CHECK(curve_surface_intersection::same_parameter(edge, pcurve, face, 1e-7, &diagnostic) ==
        curve_surface_intersection::ParameterMatch::Matched);
    MINI_CHECK(diagnostic.empty());
    const NurbsCurve past_rim = NurbsCurve::create(false, 1, {{1.0000000000000002, 0.2, 0}, {0.9, -8e-18, 0}});
    const NurbsCurve rim_edge = NurbsCurve::create(false, 1, {face.point_at(1, 0.2), face.point_at(0.9, 0)});
    MINI_CHECK(curve_surface_intersection::same_parameter(rim_edge, past_rim, face) ==
        curve_surface_intersection::ParameterMatch::Matched);
    const NurbsCurve lifted = edge.transformed(Xform::translation(0, 0, 3e-7));
    MINI_CHECK(curve_surface_intersection::same_parameter(lifted, pcurve, face) ==
        curve_surface_intersection::ParameterMatch::Mismatched);
    const NurbsCurve reversed = NurbsCurve::create(false, 1, {face.point_at(0.9, 0.8), face.point_at(0.1, 0.2)});
    MINI_CHECK(curve_surface_intersection::same_parameter(reversed, pcurve, face) ==
        curve_surface_intersection::ParameterMatch::Mismatched);
}

MINI_TEST("Curve Surface Native", "Same Parameter Across Knots And Seams") {
    NurbsSurface surface = test_bowl();
    MINI_CHECK(surface.insert_nurbsknot(0, 0.37, 1));
    MINI_CHECK(surface.insert_nurbsknot(1, 0.41, 1));
    NurbsCurve curve = surface.iso_curve(0, 0.41);
    MINI_CHECK(curve.insert_nurbsknot(0.73, 1));
    const NurbsCurve pcurve = NurbsCurve::create(false, 1, {{0, 0.41, 0}, {1, 0.41, 0}});
    MINI_CHECK(curve_surface_intersection::same_parameter(curve, pcurve, surface) ==
        curve_surface_intersection::ParameterMatch::Matched);
    const BRep torus = BRep::create_torus(2, 0.5);
    for (const BRepEdge& edge : torus.m_edges)
        for (const BRepCurveOnSurface& record : edge.pcurves)
            for (const int index : {record.curve_2d_index, record.curve_2d_index_2})
                if (index >= 0)
                    MINI_CHECK(curve_surface_intersection::same_parameter(torus.m_curves_3d[edge.curve_3d_index],
                        torus.m_curves_2d[index], torus.m_surfaces[record.surface_index]) ==
                        curve_surface_intersection::ParameterMatch::Matched);
}

MINI_TEST("Curve Surface Native", "Same Parameter Boundary Extension") {
    const NurbsSurface surface = NurbsSurface::create(false, false, 1, 1, 2, 2,
        {{1, -1, -1.2}, {1, -1, 0.8}, {1, 1, -0.8}, {1, 1, 1.2}});
    NurbsCurve curve = NurbsCurve::create(false, 1,
        {{1, 0.5714285714285703, 1.1142857142857143}, {1, 1, 1.05}});
    NurbsCurve pcurve = NurbsCurve::create(false, 1,
        {{0.7857142857142853, 1, 0}, {1.0000000000000002, 0.9249999999999998, 0}});
    MINI_CHECK(curve.set_domain(0.68571428571428517, 0.9));
    MINI_CHECK(pcurve.set_domain(0.68571428571428517, 0.9));
    const curve_surface_intersection::ParameterMatch match =
        curve_surface_intersection::same_parameter(curve, pcurve, surface, 1e-6);
    save_parameter_regression("saddle_boundary_extension", curve, pcurve, surface, match, 1e-6);
    MINI_CHECK(match == curve_surface_intersection::ParameterMatch::Matched);
    const NurbsCurve wrong = curve.transformed(Xform::translation(0.001, 0, 0));
    MINI_CHECK(curve_surface_intersection::same_parameter(wrong, pcurve, surface, 1e-6) ==
        curve_surface_intersection::ParameterMatch::Mismatched);
}

MINI_TEST("Curve Surface Native", "Same Parameter Rounded Box Boundary") {
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_offset_rounded_box_parameter_failure.json");
    nlohmann::ordered_json fixture;
    input >> fixture;
    const NurbsCurve curve = NurbsCurve::jsonload(fixture["curve"]);
    const NurbsCurve pcurve = NurbsCurve::jsonload(fixture["pcurve"]);
    const NurbsSurface surface = NurbsSurface::jsonload(fixture["surface"]);
    const double tolerance = fixture["tolerance"].get<double>();
    std::string diagnostic;
    const curve_surface_intersection::ParameterMatch match =
        curve_surface_intersection::same_parameter(curve, pcurve, surface, tolerance, &diagnostic);
    fixture["match"] = static_cast<int>(match);
    fixture["diagnostic"] = diagnostic;
    std::filesystem::create_directories(root / "build/native-geometry-tests");
    std::ofstream output(root / "build/native-geometry-tests/same_parameter_rounded_box_boundary.json");
    output << fixture.dump(2);
    output.close();
    if (match != curve_surface_intersection::ParameterMatch::Matched)
        throw std::runtime_error(diagnostic);
}

MINI_TEST("Curve Surface Native", "Same Parameter Inward Knot Contraction") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_offset_inward_contraction_parameter_failure.json");
    MINI_CHECK(input.good());
    auto fixture = nlohmann::ordered_json::parse(input);
    const NurbsCurve curve = NurbsCurve::jsonload(fixture["world"]);
    const NurbsCurve pcurve = NurbsCurve::jsonload(fixture["pcurve"]);
    const NurbsSurface surface = NurbsSurface::jsonload(fixture["surface"]);
    const double tolerance = fixture["tolerance"];
    std::string diagnostic;
    const auto match = curve_surface_intersection::same_parameter(curve, pcurve, surface, tolerance, &diagnostic);
    fixture["diagnostic"] = diagnostic;
    fixture["actual_match"] = static_cast<int>(match);
    const auto folder = root / "build/native-geometry-tests";
    std::filesystem::create_directories(folder);
    std::ofstream(folder / "native_inward_parameter_diagnostic.json") << fixture.dump(2);
    if (match != curve_surface_intersection::ParameterMatch::Matched)
        throw std::runtime_error(diagnostic);
    const NurbsCurve displaced = curve.transformed(Xform::translation(0, 0, tolerance * 100));
    MINI_CHECK(curve_surface_intersection::same_parameter(displaced, pcurve, surface, tolerance) !=
        curve_surface_intersection::ParameterMatch::Matched);
}

MINI_TEST("Curve Surface Native", "Same Parameter Tiny Interval At Surface Jump") {
    NurbsSurface surface = NurbsSurface::create(false, false, 1, 1, 4, 2,
        {{0, 0, 0}, {0, 1, 0}, {0.5, 0, 0}, {0.5, 1, 0},
         {0.5, 0, 1}, {0.5, 1, 1}, {1, 0, 1}, {1, 1, 1}});
    surface.m_nurbsknot[0] = {0, 0.5, 0.5, 1};
    MINI_CHECK(surface.is_valid());
    const double next = std::nextafter(0.5, 1.0);
    NurbsCurve pcurve = NurbsCurve::create(false, 1,
        {{0, 0.5, 0}, {0.5, 0.5, 0}, {0.5, 0.5, 0}, {0.25, 0.5, 0}});
    pcurve.m_nurbsknot = {0, 0.5, next, 1};
    MINI_CHECK(pcurve.is_valid());
    const NurbsCurve curve = pcurve;
    const auto match = curve_surface_intersection::same_parameter(curve, pcurve, surface, 1e-7);
    save_parameter_regression("tiny_interval_surface_jump", curve, pcurve, surface, match);
    MINI_CHECK(match != curve_surface_intersection::ParameterMatch::Matched);
}

MINI_TEST("Curve Surface Native", "Degenerate And Invalid Inputs") {
    const NurbsSurface surface = test_plane();
    const NurbsCurve collapsed = NurbsCurve::create(false, 1, {{0, 0, 0}, {0, 0, 0}});
    const curve_surface_intersection::Result degenerate = curve_surface_intersection::intersect(collapsed, surface);
    MINI_CHECK(!degenerate.complete && !degenerate.failure.empty());
    const NurbsCurve curve = NurbsCurve::create(false, 1, {{0, 0, -1}, {0, 0, 1}});
    bool rejected = false;
    try {
        curve_surface_intersection::intersect(curve, surface, std::numeric_limits<double>::quiet_NaN());
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    MINI_CHECK(rejected);
    const NurbsSurface shifted = surface.transformed(Xform::translation(1e9, 0, 0));
    const NurbsCurve translated = curve.transformed(Xform::translation(1e9, 0, 0));
    const curve_surface_intersection::Result uncertain = curve_surface_intersection::intersect(translated, shifted, 1e-9);
    MINI_CHECK(!uncertain.complete && !uncertain.failure.empty());
}

MINI_TEST("Curve Surface Native", "Same Parameter Trimmed Focal Support") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_thicken_trimmed_focal_parameter_failure.json");
    MINI_CHECK(input.good());
    const auto fixture = nlohmann::ordered_json::parse(input);
    const NurbsCurve curve = NurbsCurve::jsonload(fixture.at("curve"));
    const NurbsCurve pcurve = NurbsCurve::jsonload(fixture.at("pcurve"));
    const NurbsSurface surface = NurbsSurface::jsonload(fixture.at("surface"));
    const double tolerance = fixture.at("tolerance");
    MINI_CHECK(tolerance == 1e-5);
    MINI_CHECK(curve.cv_count() == 20 && pcurve.cv_count() == 2);
    MINI_CHECK(surface.cv_count(0) == 62 && surface.cv_count(1) == 7);
    MINI_CHECK(curve.domain_start() == -4 && curve.domain_end() == 2);
    MINI_CHECK(curve.domain() == pcurve.domain());
    std::string diagnostic;
    const auto match = curve_surface_intersection::same_parameter(curve, pcurve, surface, tolerance, &diagnostic);
    if (match != curve_surface_intersection::ParameterMatch::Matched)
        throw std::runtime_error(diagnostic);
    const NurbsCurve displaced = curve.transformed(Xform::translation(0, 4 * tolerance, 0));
    MINI_CHECK(curve_surface_intersection::same_parameter(displaced, pcurve, surface, tolerance)
        == curve_surface_intersection::ParameterMatch::Mismatched);
    MINI_CHECK(curve.domain_start() == -4 && curve.domain_end() == 2);
    MINI_CHECK(pcurve.domain() == curve.domain());
}

MINI_TEST("Curve Surface Native", "Interval Surface Adjacent Float Span") {
    NurbsSurface surface = NurbsSurface::create(false, false, 1, 1, 3, 2,
        {{0, 0, 0}, {0, 1, 0}, {.5, 0, 1}, {.5, 1, 1}, {1, 0, 0}, {1, 1, 0}});
    surface.m_nurbsknot[0] = {0, .5, 1};
    MINI_CHECK(surface.is_valid());
    const double knot = surface.m_nurbsknot[0][1];
    const double left = std::nextafter(knot, -std::numeric_limits<double>::infinity());
    MINI_CHECK(std::midpoint(left, knot) == knot);
    nurbs_interval::SurfaceBounds bounds;
    MINI_CHECK(nurbs_interval::surface_bounds(surface, {left, knot, 0, 1}, {0, 0, 0}, bounds, false));
    MINI_CHECK(bounds.point[0].lo <= left && bounds.point[0].hi >= knot);
    MINI_CHECK(bounds.point[2].lo <= 1 && bounds.point[2].hi >= 1);
    MINI_CHECK(!nurbs_interval::surface_controls(surface, {knot, knot, 0, 1}, {0, 0, 0}).empty());
    MINI_CHECK(nurbs_interval::surface_controls(surface, {.25, .75, 0, 1}, {0, 0, 0}).empty());
}


MINI_TEST("Curve Surface Native", "Certified Focal Endpoint Negative Witness") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_thicken_trimmed_focal_parameter_failure.json");
    MINI_CHECK(input.good());
    const auto fixture = nlohmann::ordered_json::parse(input);
    const NurbsCurve original = NurbsCurve::jsonload(fixture.at("curve"));
    const NurbsSurface surface = NurbsSurface::jsonload(fixture.at("surface"));
    const double tolerance = fixture.at("tolerance");
    // Independent geometry: this entire source boundary and its world curve have y=0.
    // Every translated endpoint therefore has exact y separation 4*tolerance.
    MINI_CHECK(!surface.is_rational());
    for (int i = 0; i < surface.cv_count(0); ++i)
        MINI_CHECK(surface.cv(i, 0)[1] == 0);
    for (int i = 0; i < original.cv_count(); ++i)
        MINI_CHECK(original.cv(i)[1] == 0);
    for (int variant = 0; variant < 4; ++variant) {
        NurbsCurve pcurve = NurbsCurve::jsonload(fixture.at("pcurve"));
        MINI_CHECK(pcurve.set_domain(32, 35)); // Corresponding endpoints; unequal raw domains.
        NurbsCurve displaced = original.transformed(Xform::translation(0,
            (variant & 1 ? -4 : 4)*tolerance, 0));
        if (variant & 2) {
            MINI_CHECK(displaced.reverse());
            MINI_CHECK(pcurve.reverse());
            MINI_CHECK(pcurve.to_rational());
            for (int i = 0; i < pcurve.cv_count(); ++i) {
                const Point point = pcurve.get_cv(i);
                MINI_CHECK(pcurve.set_cv_4d(i, 2*point[0], 2*point[1], 2*point[2], 2));
            }
        }
        MINI_CHECK(curve_surface_intersection::same_parameter(displaced, pcurve, surface, tolerance)
            == curve_surface_intersection::ParameterMatch::Mismatched);
    }
}

namespace {
session_cpp::NurbsSurface exact_chart_test_plane() {
    using namespace session_cpp;
    const Point origin(1048576, -524288, 262144);
    return NurbsSurface::create(false, false, 1, 1, 2, 2,
        {origin, origin + Vector(0,1,0), origin + Vector(1,0,0), origin + Vector(1,1,0)});
}
session_cpp::NurbsCurve exact_chart_test_pc() {
    auto pc = session_cpp::NurbsCurve::create(false, 1, {{0,.5,0},{1,.5,0}});
    MINI_CHECK(pc.set_domain(-7,13));
    return pc;
}
}

MINI_TEST("Curve Surface Native", "Exact World Chart Shifted Polynomial Certificate") {
    using namespace session_cpp;
    const auto surface = exact_chart_test_plane();
    auto curve = NurbsCurve::create(false, 1, {{1048576,-524287.5,262144},{1048577,-524287.5,262144}});
    MINI_CHECK(curve.set_domain(-7,13));
    const auto pc = exact_chart_test_pc();
    const auto c = curve.jsondump(), p = pc.jsondump(), s = surface.jsondump();
    std::string diagnostic;
    MINI_CHECK(curve_surface_intersection::same_parameter(curve,pc,surface,1e-7,&diagnostic)
        == curve_surface_intersection::ParameterMatch::Matched);
    MINI_CHECK(diagnostic.empty());
    MINI_CHECK(curve.jsondump()==c && pc.jsondump()==p && surface.jsondump()==s);
    MINI_CHECK(curve.domain_start()==-7 && curve.domain_end()==13);
}

MINI_TEST("Curve Surface Native", "Exact World Chart Rational Homogeneous Certificate") {
    using namespace session_cpp;
    auto surface = exact_chart_test_plane();
    MINI_CHECK(surface.to_rational());
    for (int i=0;i<2;++i)
        for (int j=0;j<2;++j) {
            const Point point = surface.get_cv(i,j);
            const double w = i ? 2 : 1;
            MINI_CHECK(surface.set_cv_4d(i,j,w*point[0],w*point[1],w*point[2],w));
        }
    NurbsCurve curve(3,true,2,2);
    curve.m_nurbsknot={-7,13};
    MINI_CHECK(curve.set_cv_4d(0,1048576,-524287.5,262144,1));
    MINI_CHECK(curve.set_cv_4d(1,2*1048577.,2*-524287.5,2*262144.,2));
    const auto pc = exact_chart_test_pc();
    const auto c=curve.jsondump(), s=surface.jsondump(), p=pc.jsondump();
    MINI_CHECK(curve_surface_intersection::same_parameter(curve,pc,surface,1e-7)
        == curve_surface_intersection::ParameterMatch::Matched);
    MINI_CHECK(curve.jsondump()==c && surface.jsondump()==s && pc.jsondump()==p);
    MINI_CHECK(surface.weight(0,0)==1 && surface.weight(1,1)==2);
}

MINI_TEST("Curve Surface Native", "Exact World Chart Interior Negative Certificate") {
    using namespace session_cpp;
    const auto surface = exact_chart_test_plane();
    auto curve = NurbsCurve::create(false, 2,
        {{1048576,-524287.5,262144},{1048576.5,-524287.5,262144.0009765625},
         {1048577,-524287.5,262144}});
    MINI_CHECK(curve.set_domain(-7,13));
    const auto pc=exact_chart_test_pc();
    MINI_CHECK(curve.point_at_start().distance(surface.point_at(0,.5))==0);
    MINI_CHECK(curve.point_at_end().distance(surface.point_at(1,.5))==0);
    MINI_CHECK(curve.point_at(3)[2]-262144 > 1000e-7);
    const auto original=curve.jsondump();
    std::string diagnostic = "stale sentinel";
    MINI_CHECK(curve_surface_intersection::same_parameter(curve,pc,surface,1e-7,&diagnostic)
        == curve_surface_intersection::ParameterMatch::Mismatched);
    MINI_CHECK(diagnostic.starts_with("Certified common-interval point residual exceeds tolerance"));
    MINI_CHECK(diagnostic.find("Common parameter partition failed") == std::string::npos);
    MINI_CHECK(curve.jsondump()==original);
}

MINI_TEST("Curve Surface Native", "Exact World Chart Inexact Rational Original Fallback") {
    using namespace session_cpp;
    const double origin=1000000.1, w=1.1, product=origin*w;
    MINI_CHECK(!nurbs_coincidence::exact_products_equal(origin,w,product,1));
    NurbsCurve curve(3,true,2,2);
    curve.m_nurbsknot={-7,13};
    MINI_CHECK(curve.set_cv_4d(0,origin,0,0,1));
    MINI_CHECK(curve.set_cv_4d(1,(origin+1)*w,0,0,w));
    const auto surface=exact_chart_test_plane();
    const auto pc=exact_chart_test_pc();
    const auto c=curve.jsondump(),s=surface.jsondump(),p=pc.jsondump();
    std::string diagnostic="untouched sentinel";
    MINI_CHECK(curve_surface_intersection::same_parameter(curve,pc,surface,1e-7,&diagnostic)
        == curve_surface_intersection::ParameterMatch::Unresolved);
    MINI_CHECK(diagnostic=="Input or coordinate-resolution validation failed");
    MINI_CHECK(curve.jsondump()==c && surface.jsondump()==s && pc.jsondump()==p);
}

namespace session_cpp {
MINI_TEST("Curve Surface Native", "Same Parameter Closed C1 Chart Upper Endpoint") {
    // Independently specified S(u,v)=(u,v,0) for u<=1 and
    // S(u,v)=(u,v,4*(u-1)^2) for u>=1. The patches are C1, not C2.
    NurbsSurface surface = NurbsSurface::create(false, false, 2, 1, 5, 2,
        {{0,0,0},{0,1,0},{0.5,0,0},{0.5,1,0},{1,0,0},{1,1,0},
         {1.5,0,0},{1.5,1,0},{2,0,4},{2,1,4}});
    surface.m_nurbsknot[0] = {0,0,1,1,2,2};
    MINI_CHECK(surface.is_valid());
    const NurbsCurve world = NurbsCurve::create(false,1,{{0,0.5,0},{1,0.5,0}});
    const NurbsCurve pc = world;
    MINI_CHECK(curve_surface_intersection::same_parameter(world,pc,surface,1e-12)
        == curve_surface_intersection::ParameterMatch::Matched);
    const NurbsCurve displaced = world.transformed(Xform::translation(0,0,1e-8));
    MINI_CHECK(curve_surface_intersection::same_parameter(displaced,pc,surface,1e-12)
        == curve_surface_intersection::ParameterMatch::Mismatched);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Curve Surface Native", "Structured Parameter Failure Preserves Raw Locations") {
    const auto surface=NurbsSurface::create(false,false,1,1,2,2,
        {{0,0,0},{0,1,0},{1,0,0},{1,1,0}});
    NurbsCurve world=NurbsCurve::create(false,1,{{0,.5,0},{1,.5,0}});
    world.m_nurbsknot={-3,2};
    NurbsCurve pc=NurbsCurve::create(false,2,{{0,.5,0},{1,.5,0},{1,.5,0}});
    pc.m_nurbsknot={7,7,11,11};
    curve_surface_intersection::ParameterFailure failure;
    const auto match=curve_surface_intersection::same_parameter(world,pc,surface,1e-10,nullptr,&failure);
    MINI_CHECK(match==curve_surface_intersection::ParameterMatch::Mismatched);
    MINI_CHECK(failure.kind==curve_surface_intersection::ParameterFailure::Kind::PointResidual);
    MINI_CHECK(failure.has_interval && failure.has_point);
    MINI_CHECK(failure.curve_point>-3 && failure.curve_point<2);
    MINI_CHECK(failure.pcurve_point>7 && failure.pcurve_point<11);
    MINI_CHECK(std::abs((failure.curve_point+3)/5-(failure.pcurve_point-7)/4)<1e-14);
    MINI_CHECK(curve_surface_intersection::same_parameter(world,world,surface,1e-10,nullptr,&failure)==
        curve_surface_intersection::ParameterMatch::Matched);
    MINI_CHECK(failure.kind==curve_surface_intersection::ParameterFailure::Kind::None && !failure.has_interval);
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Curve Surface Native", "Directed Bilinear Agreement Preserves Actual Critical Witnesses") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_critical_bilinear_agreement.json");
    MINI_CHECK(input.good());
    const auto fixture = nlohmann::json::parse(input);
    MINI_CHECK(fixture["records"].size() == 2);
    for (const auto& record : fixture["records"]) {
        const auto world = NurbsCurve::jsonload(record["world"]);
        const double tolerance = record["accuracy"];
        for (bool second : {false, true}) {
            const auto pc = NurbsCurve::jsonload(record[second ? "second_pcurve" : "first_pcurve"]);
            const auto support = NurbsSurface::jsonload(record[second ? "second" : "first"]);
            const auto before_world = world.jsondump(), before_pc = pc.jsondump(),
                       before_surface = support.jsondump();
            std::string diagnostic = "previous failure";
            curve_surface_intersection::ParameterFailure failure;
            failure.kind = curve_surface_intersection::ParameterFailure::Kind::Bound;
            failure.has_interval = true;
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pc, support, tolerance, &diagnostic,
                                                                  &failure) ==
                       curve_surface_intersection::ParameterMatch::Matched);
            MINI_CHECK(diagnostic.empty() &&
                       failure.kind == curve_surface_intersection::ParameterFailure::Kind::None);
            MINI_CHECK(!failure.has_interval && !failure.has_point);
            MINI_CHECK(world.jsondump() == before_world && pc.jsondump() == before_pc &&
                       support.jsondump() == before_surface);
        }
    }
}

MINI_TEST("Curve Surface Native", "Directed Bilinear Agreement Uses Original Homogeneous Raw Laws") {
    for (bool translated : {false, true})
        for (int pc_dimension : {2, 3}) {
            const Point origin =
                translated ? Point(std::ldexp(1., 20), -std::ldexp(1., 19), std::ldexp(1., 18)) : Point();
            NurbsSurface surface(3, true, 2, 2, 2, 2);
            for (int i = 0; i < 2; ++i)
                for (int j = 0; j < 2; ++j) {
                    const Point p(origin[0] + i, origin[1] + j, origin[2] + .5 * i * j);
                    MINI_CHECK(surface.set_cv_4d(i, j, p[0] * 5, p[1] * 5, p[2] * 5, 5));
                }
            MINI_CHECK(surface.set_domain(0, -7, 13) && surface.set_domain(1, 32, 35));
            NurbsCurve world(3, true, 2, 2), pc(pc_dimension, true, 2, 2);
            world.m_nurbsknot = {-3, 2};
            pc.m_nurbsknot = {11, 17};
            for (int i = 0; i < 2; ++i) {
                const Point p(origin[0] + .25, origin[1] + i, origin[2] + .125 * i);
                MINI_CHECK(world.set_cv_4d(i, p[0] * 2, p[1] * 2, p[2] * 2, 2));
                MINI_CHECK(pc.set_cv_4d(i, -2 * 3, (32 + 3 * i) * 3, 0, 3));
            }
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pc, surface, 2.5e-13) ==
                       curve_surface_intersection::ParameterMatch::Matched);
        }
}

MINI_TEST("Curve Surface Native", "Directed Bilinear Agreement Rejects Interior And Weight Excursions") {
    for (bool translated : {false, true}) {
        const Point origin =
            translated ? Point(std::ldexp(1., 20), -std::ldexp(1., 19), std::ldexp(1., 18)) : Point();
        for (bool variable_weight : {false, true}) {
            NurbsSurface surface(3, false, 2, 2, 2, 2);
            for (int i = 0; i < 2; ++i)
                for (int j = 0; j < 2; ++j)
                    MINI_CHECK(surface.set_cv(
                        i, j, {origin[0] + i, origin[1] + j, origin[2] + (variable_weight ? 0. : i * j)}));
            MINI_CHECK(surface.set_domain(0, -7, 13) && surface.set_domain(1, 32, 35));
            NurbsCurve world(3, variable_weight, 2, 2), pc(3, false, 2, 2);
            world.m_nurbsknot = {-3, 2};
            pc.m_nurbsknot = {11, 17};
            MINI_CHECK(pc.set_cv(0, {-7, 32, 0}) && pc.set_cv(1, {13, 35, 0}));
            for (int i = 0; i < 2; ++i) {
                const Point p(origin[0] + i, origin[1] + i, origin[2] + (variable_weight ? 0. : i));
                if (variable_weight) {
                    const double weight = 2 + i;
                    MINI_CHECK(world.set_cv_4d(i, p[0] * weight, p[1] * weight, p[2] * weight, weight));
                } else
                    MINI_CHECK(world.set_cv(i, p));
            }
            const auto middle = pc.point_at(pc.domain_middle());
            MINI_CHECK(
                world.point_at(world.domain_middle()).distance(surface.point_at(middle[0], middle[1])) > .09);
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pc, surface, 1e-12) !=
                       curve_surface_intersection::ParameterMatch::Matched);
        }
    }
}

MINI_TEST("Curve Surface Native",
          "Directed Bilinear Agreement Does Not Assume PC Or Support Weights Constant") {
    const Point origin(std::ldexp(1., 20), -std::ldexp(1., 19), std::ldexp(1., 18));
    for (bool variable_support : {false, true}) {
        NurbsSurface surface(3, true, 2, 2, 2, 2);
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j) {
                const double weight = variable_support ? 2. + i : 1.;
                MINI_CHECK(surface.set_cv_4d(i, j, (origin[0] + i) * weight, (origin[1] + j) * weight,
                                             origin[2] * weight, weight));
            }
        MINI_CHECK(surface.set_domain(0, -7, 13) && surface.set_domain(1, 32, 35));
        NurbsCurve world(3, false, 2, 2), pc(2, true, 2, 2);
        world.m_nurbsknot = {-3, 2};
        pc.m_nurbsknot = {11, 17};
        for (int i = 0; i < 2; ++i) {
            MINI_CHECK(world.set_cv(i, {origin[0] + i, origin[1] + i, origin[2]}));
            const double weight = variable_support ? 1. : 2. + i;
            MINI_CHECK(pc.set_cv_4d(i, (-7 + 20 * i) * weight, (32 + 3 * i) * weight, 0, weight));
        }
        const auto middle = pc.point_at(pc.domain_middle());
        MINI_CHECK(world.point_at(world.domain_middle()).distance(surface.point_at(middle[0], middle[1])) >
                   .09);
        MINI_CHECK(curve_surface_intersection::same_parameter(world, pc, surface, 1e-12) !=
                   curve_surface_intersection::ParameterMatch::Matched);
    }
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Curve Surface Native", "Whole Constant Image Uses Exact Homogeneous Proportions") {
    NurbsCurve point(3,true,3,3);
    point.m_nurbsknot={-7,-7,13,13};
    for (int i=0;i<3;++i) {
        const double factor=std::ldexp(1.,i);
        // The exact image is (1/3,-2/3,4/3), even though 1/3 is not a double.
        MINI_CHECK(point.set_cv_4d(i,factor,-2*factor,4*factor,3*factor));
    }
    const auto before=point.jsondump();
    MINI_CHECK(nurbs_coincidence::curve_constant_image(point));
    for (double t : {-7.,3.,13.})
        MINI_CHECK(point.point_at(t).distance(Point(1./3,-2./3,4./3))<1e-15);
    MINI_CHECK(point.jsondump()==before);
    MINI_CHECK(point.reverse());
    MINI_CHECK(nurbs_coincidence::curve_constant_image(point));
    point.m_cv[point.m_cv_stride+1]=std::nextafter(point.m_cv[point.m_cv_stride+1],0.);
    // A one-ULP change in an interior homogeneous coefficient is not exact point identity.
    MINI_CHECK(!nurbs_coincidence::curve_constant_image(point));

    NurbsCurve feature(3,true,3,3);
    feature.m_nurbsknot={0,0,1,1};
    const double epsilon=std::ldexp(1.,-30);
    const double weights[3]={2,3,5};
    for(int i=0;i<3;++i) {
        const double y=i==1?epsilon:0;
        MINI_CHECK(feature.set_cv_4d(i,0,y*weights[i],0,weights[i]));
    }
    // Independently, rational quadratic y(.5)=(6/13)*epsilon >0 while both ends are0.
    MINI_CHECK(std::abs(feature.point_at(.5)[1]-(6./13)*epsilon)<1e-24);
    MINI_CHECK(!nurbs_coincidence::curve_constant_image(feature));
    MINI_CHECK(feature.point_at_start().distance(Point())==0 && feature.point_at_end().distance(Point())==0);
    feature.m_cv[3]=0;
    MINI_CHECK(!nurbs_coincidence::curve_constant_image(feature));
    feature.m_cv[3]=2;
    feature.m_cv[0]=std::numeric_limits<double>::infinity();
    MINI_CHECK(!nurbs_coincidence::curve_constant_image(feature));
}

MINI_TEST("Curve Surface Native", "Line Running Past A Surface Along Its Ruling") {
    const Xform turn = Xform::rotation_z(Tolerance::PI / 4.0);
    const NurbsSurface cylinder = Primitives::cylinder_surface(0, 0, 0, 100, 425).transformed(turn);
    const Point corner = cylinder.point_at(1.3, 0.5);
    const NurbsCurve line = NurbsCurve::create(false, 1, {Point(corner[0], corner[1], 275), Point(corner[0], corner[1], 437)});
    const auto events = curve_surface_intersection::intersect(line, cylinder, 1e-7);
    MINI_CHECK(events.complete && events.overlaps.size() == 1);
    MINI_CHECK(std::abs(line.point_at(events.overlaps[0].t0)[2] - 275) < 1e-9);
    MINI_CHECK(std::abs(line.point_at(events.overlaps[0].t1)[2] - 425) < 1e-9);
    const Point uv = events.overlaps[0].pcurve.point_at_end();
    MINI_CHECK(cylinder.point_at(uv[0], uv[1]).distance(Point(corner[0], corner[1], 425)) < 1e-9);
}

MINI_TEST("Curve Surface Native", "Line Tangent To A Turned Cylinder Hits Once") {
    for (double turn : {0.0, 0.3, 1.0, 2.2}) {
        const NurbsSurface cylinder = Primitives::cylinder_surface(0, 0, 0, 100, 425).transformed(Xform::rotation_z(turn));
        const Point start = Point(100, -40, 240).transformed(Xform::rotation_z(turn + 0.7));
        const Point end = Point(100, 40, 240).transformed(Xform::rotation_z(turn + 0.7));
        const auto events = curve_surface_intersection::intersect(NurbsCurve::create(false, 1, {start, end}), cylinder, 1e-7);
        MINI_CHECK(events.complete && events.points.size() == 1 && events.points[0].tangent);
        MINI_CHECK(std::abs(events.points[0].t - 40.0) < 1e-6);
    }
}

MINI_TEST("Curve Surface Native", "Lines Through Recognised Surfaces Hit Where Their Forms Vanish") {
    const Xform turn = Xform::rotation(Vector(0.3, -0.5, 0.8), 0.9);
    const NurbsSurface surfaces[] = {Primitives::sphere_surface(0, 0, 0, 2).transformed(turn), Primitives::cylinder_surface(0, 0, -3, 2, 6).transformed(turn),
        Primitives::cone_surface(0, 0, -3, 2, 6).transformed(turn), Primitives::torus_surface(0, 0, 0, 3, 1).transformed(turn)};
    const NurbsCurve line = NurbsCurve::create(false, 1, {Point(-5, 0.31, 0.17).transformed(turn), Point(5, 0.31, 0.17).transformed(turn)});
    const size_t expected[] = {2, 2, 2, 4};
    for (int k = 0; k < 4; ++k) {
        const auto events = curve_surface_intersection::intersect(line, surfaces[k], 1e-7);
        MINI_CHECK(events.complete && events.overlaps.empty() && events.points.size() == expected[k]);
        for (const auto& hit : events.points) {
            MINI_CHECK(!hit.tangent);
            MINI_CHECK(surfaces[k].point_at(hit.u, hit.v).distance(line.point_at(hit.t)) < 1e-9);
        }
    }
    const NurbsCurve miss = NurbsCurve::create(false, 1, {Point(-5, 2.5, 0).transformed(turn), Point(5, 2.5, 0).transformed(turn)});
    MINI_CHECK(curve_surface_intersection::intersect(miss, surfaces[0], 1e-7).points.empty());
}

MINI_TEST("Curve Surface Native", "Crossing Cylinders Section Stays Small And Certified") {
    NurbsSurface drill = Primitives::cylinder_surface(0, 0, -150, 10, 300).transformed(Xform::rotation_x(0.5 * Tolerance::PI));
    drill = drill.transformed(Xform::translation(0, 0, 200));
    const NurbsSurface body = Primitives::cylinder_surface(0, 0, 0, 100, 425);
    const auto sections = Intersection::surface_surface(body, drill, 1e-7, true);
    MINI_CHECK(sections.size() >= 2);
    for (const auto& section : sections) {
        MINI_CHECK(std::get<0>(section).cv_count() <= 300);
        MINI_CHECK(curve_surface_intersection::same_parameter(std::get<0>(section), std::get<1>(section), body, 1e-7)
            == curve_surface_intersection::ParameterMatch::Matched);
        MINI_CHECK(curve_surface_intersection::same_parameter(std::get<0>(section), std::get<2>(section), drill, 1e-7)
            == curve_surface_intersection::ParameterMatch::Matched);
    }
}

MINI_TEST("Curve Surface Native", "A Piece Of A Matched Curve Keeps The Certificate Of Its Spans") {
    using curve_surface_intersection::ParameterMatch;
    NurbsSurface drill = Primitives::cylinder_surface(0, 0, -150, 10, 300).transformed(Xform::rotation_x(0.5 * Tolerance::PI));
    drill = drill.transformed(Xform::translation(0, 0, 200));
    const NurbsSurface body = Primitives::cylinder_surface(0, 0, 0, 100, 425);
    const auto sections = Intersection::surface_surface(body, drill, 1e-7, true);
    MINI_CHECK(!sections.empty());
    const NurbsCurve& world = std::get<0>(sections[0]);
    const NurbsCurve& pcurve = std::get<1>(sections[0]);
    MINI_CHECK(world.domain() == pcurve.domain() && world.span_count() > 8);
    const double first = std::lerp(world.domain_start(), world.domain_end(), 0.21);
    const double last = std::lerp(world.domain_start(), world.domain_end(), 0.68);
    NurbsCurve piece = world, piece_pcurve = pcurve;
    MINI_CHECK(piece.trim(first, last) && piece_pcurve.trim(first, last) && piece.domain() == piece_pcurve.domain());
    // An end of the pcurve moved off its place by far more than the tolerance.
    NurbsCurve moved = piece_pcurve;
    const Point end = moved.get_cv(moved.cv_count() - 1);
    MINI_CHECK(moved.set_cv(moved.cv_count() - 1, Point(end[0] + 1e-3, end[1] + 1e-3, 0)));
    const ParameterMatch off = curve_surface_intersection::same_parameter(piece, moved, body, 1e-7);
    MINI_CHECK(off != ParameterMatch::Matched);

    nurbs_memo::Scope scope;
    const auto spans = [&scope] {
        size_t count = 0;
        for (const auto& [surface, pairs] : scope.spans)
            count += pairs.size();
        return count;
    };
    MINI_CHECK(curve_surface_intersection::same_parameter(world, pcurve, body, 1e-7) == ParameterMatch::Matched);
    const size_t whole = spans();
    MINI_CHECK(whole >= static_cast<size_t>(world.span_count()));
    // A tolerance above one already matched asks nothing new, and neither does the same curve again.
    MINI_CHECK(curve_surface_intersection::same_parameter(world, pcurve, body, 1e-6) == ParameterMatch::Matched && spans() == whole);
    // The piece is certified on the spans its two cuts made and on no other.
    MINI_CHECK(curve_surface_intersection::same_parameter(piece, piece_pcurve, body, 1e-7) == ParameterMatch::Matched);
    const size_t cut = spans() - whole;
    MINI_CHECK(cut > 0 && cut * 2 < whole);
    // The moved end is asked again and refused as it is without the scope; the spans away from it are not asked.
    MINI_CHECK(curve_surface_intersection::same_parameter(piece, moved, body, 1e-7) == off && spans() == whole + cut);
    for (double tolerance : {1e-7, 1e-9, 1e-11}) {
        const ParameterMatch kept = curve_surface_intersection::same_parameter(world, pcurve, body, tolerance);
        nurbs_memo::Scope fresh;
        MINI_CHECK(kept == curve_surface_intersection::same_parameter(world, pcurve, body, tolerance));
    }
}

MINI_TEST("Curve Surface Native", "A Piece Inherits The Certificate Of Its Whole Within The Rounding Of The Cut") {
    using curve_surface_intersection::ParameterMatch;
    NurbsSurface drill = Primitives::cylinder_surface(0, 0, -150, 10, 300).transformed(Xform::rotation_x(0.5 * Tolerance::PI));
    drill = drill.transformed(Xform::translation(0, 0, 200));
    const NurbsSurface body = Primitives::cylinder_surface(0, 0, 0, 100, 425);
    const auto sections = Intersection::surface_surface(body, drill, 1e-7, true);
    MINI_CHECK(!sections.empty());
    const NurbsCurve& world = std::get<0>(sections[0]);
    const NurbsCurve& pcurve = std::get<1>(sections[0]);
    const double first = std::lerp(world.domain_start(), world.domain_end(), 0.21);
    const double last = std::lerp(world.domain_start(), world.domain_end(), 0.68);
    NurbsCurve piece = world, piece_pcurve = pcurve;
    MINI_CHECK(piece.trim(first, last) && piece_pcurve.trim(first, last));
    NurbsCurve moved = piece_pcurve;
    const Point end = moved.get_cv(moved.cv_count() - 1);
    MINI_CHECK(moved.set_cv(moved.cv_count() - 1, Point(end[0] + 1e-3, end[1] + 1e-3, 0)));

    nurbs_memo::Scope scope;
    const auto spans = [&scope] {
        size_t count = 0;
        for (const auto& [surface, pairs] : scope.spans)
            count += pairs.size();
        return count;
    };
    MINI_CHECK(curve_surface_intersection::same_parameter(world, pcurve, body, 1e-7) == ParameterMatch::Matched);
    const size_t whole = spans();
    // A moved end is not covered by the whole: its span is left to be asked, and refused.
    curve_surface_intersection::inherit(piece, moved, world, pcurve, body, 1e-7);
    MINI_CHECK(curve_surface_intersection::same_parameter(piece, moved, body, 1e-7) != ParameterMatch::Matched);
    // The two spans the cuts made are covered without a certificate of their own.
    const size_t before = spans();
    curve_surface_intersection::inherit(piece, piece_pcurve, world, pcurve, body, 1e-7);
    MINI_CHECK(spans() > before && spans() >= whole + 2);
    const size_t inherited = spans();
    MINI_CHECK(curve_surface_intersection::same_parameter(piece, piece_pcurve, body, 1e-7) == ParameterMatch::Matched && spans() == inherited);
    // Nothing is inherited below the tolerance the whole reached.
    NurbsCurve other = world;
    MINI_CHECK(other.trim(first, std::lerp(world.domain_start(), world.domain_end(), 0.5)));
    NurbsCurve other_pcurve = pcurve;
    MINI_CHECK(other_pcurve.trim(first, std::lerp(world.domain_start(), world.domain_end(), 0.5)));
    curve_surface_intersection::inherit(other, other_pcurve, world, pcurve, body, 1e-30);
    MINI_CHECK(spans() == inherited);
}

MINI_TEST("Curve Surface Native", "A Witness On A Knot Line Is Bounded By Both Cells") {
    using curve_surface_intersection::ParameterMatch;
    const NurbsSurface cylinder = Primitives::cylinder_surface(0, 0, 0, 100, 425);
    const int around = cylinder.get_span_vector(0).size() > 2 ? 0 : 1, along = 1 - around;
    const std::vector<double> turns = cylinder.get_span_vector(around);
    MINI_CHECK(turns.size() > 2);
    const std::pair<double, double> height = cylinder.domain(along);
    // A ruling on an inner knot line and one on the seam, with its straight pcurve; then the same lifted off the surface.
    for (const double turn : {turns[1], turns.front(), turns.back()}) {
        const NurbsCurve ruling = cylinder.iso_curve(along, turn);
        NurbsCurve pcurve(3, false, 2, 2);
        pcurve.m_nurbsknot = {height.first, height.second};
        Point start, end;
        start[around] = end[around] = turn;
        start[along] = height.first;
        end[along] = height.second;
        MINI_CHECK(ruling.domain() == height && pcurve.set_cv(0, start) && pcurve.set_cv(1, end));
        MINI_CHECK(curve_surface_intersection::same_parameter(ruling, pcurve, cylinder, 1e-7) == ParameterMatch::Matched);
        const Point foot = ruling.point_at_start();
        const NurbsCurve lifted = ruling.transformed(Xform::translation(foot[0] * 3e-9, foot[1] * 3e-9, 0));
        MINI_CHECK(curve_surface_intersection::same_parameter(lifted, pcurve, cylinder, 1e-7) == ParameterMatch::Mismatched);
        MINI_CHECK(curve_surface_intersection::same_parameter(lifted, pcurve, cylinder, 1e-6) == ParameterMatch::Matched);
    }
}

} // namespace session_cpp
