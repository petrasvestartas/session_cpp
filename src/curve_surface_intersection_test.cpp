#include "curve_surface_intersection.h"
#include "brep.h"
#include "mini_test.h"
#include "primitives.h"
#include "xform.h"
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
