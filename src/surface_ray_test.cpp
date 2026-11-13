#include "surface_ray.h"
#include "brep.h"
#include "mini_test.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>

using namespace session_cpp;
using namespace session_cpp::mini_test;

namespace {

void save(const std::string& name, const NurbsSurface& surface, const Point& origin,
    const Vector& direction, double tolerance, const surface_ray::Result& result) {
    const auto folder = std::filesystem::path("build/native-geometry-tests/surface-ray");
    std::filesystem::create_directories(folder);
    nlohmann::ordered_json record = {{"surface", surface.jsondump()}, {"origin", {origin[0], origin[1], origin[2]}},
        {"direction", {direction[0], direction[1], direction[2]}}, {"tolerance", tolerance},
        {"complete", result.complete}, {"failure", result.failure}, {"cells", result.cells},
        {"roots", nlohmann::ordered_json::array()}};
    for (const auto& root : result.roots)
        record["roots"].push_back({{"point", {root.point[0], root.point[1], root.point[2]}},
            {"uv", {root.u, root.v}}, {"distance", root.distance}, {"enclosure", root.enclosure},
            {"chart", root.chart}, {"distance_enclosure", root.distance_enclosure}, {"crossing", root.crossing}});
    std::ofstream output(folder / (name + ".json"));
    if (!(output << record.dump(2)))
        throw std::runtime_error("Could not preserve certified surface-ray fixture");
}

surface_ray::Result run(const std::string& name, const NurbsSurface& surface, const Point& origin,
    const Vector& direction, double tolerance = 1e-7, const surface_ray::Options& options = {}) {
    const auto result = surface_ray::intersect(surface, origin, direction, tolerance, options);
    save(name, surface, origin, direction, tolerance, result);
    return result;
}

void complete(const surface_ray::Result& result) {
    if (!result.complete)
        throw std::runtime_error(result.failure + "; cells=" + std::to_string(result.cells));
}

void verify(const NurbsSurface& surface, const Point& origin, const Vector& direction,
    const surface_ray::Root& root, double tolerance) {
    MINI_CHECK(root.point.distance(surface.point_at(root.u, root.v)) <= tolerance);
    const Vector unit = direction.normalized();
    MINI_CHECK(root.point.distance(origin + unit * root.distance) <= tolerance * 2.0);
    MINI_CHECK(root.distance_enclosure[0] > 0.0);
    MINI_CHECK(root.distance >= root.distance_enclosure[0] && root.distance <= root.distance_enclosure[1]);
    MINI_CHECK(root.enclosure[0] >= root.chart[0] && root.enclosure[1] <= root.chart[1]);
    MINI_CHECK(root.enclosure[2] >= root.chart[2] && root.enclosure[3] <= root.chart[3]);
    MINI_CHECK(root.u >= root.chart[0] && root.u <= root.chart[1]);
    MINI_CHECK(root.v >= root.chart[2] && root.v <= root.chart[3]);
    MINI_CHECK(root.crossing == -1 || root.crossing == 1);
}

NurbsSurface plane() {
    return NurbsSurface::create(false, false, 1, 1, 2, 2,
        {{-1, -1, 2}, {-1, 1, 2}, {1, -1, 2}, {1, 1, 2}});
}

NurbsSurface bowl() {
    const std::array<double, 3> linear = {-1, 0, 1};
    const std::array<double, 3> quadratic = {1, -1, 1};
    std::vector<Point> points;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            points.push_back({linear[i], linear[j], quadratic[i] + quadratic[j]});
    return NurbsSurface::create(false, false, 2, 2, 3, 3, points);
}

}

MINI_TEST("Surface Ray", "Saved Grazing Crossing") {
    const auto folder = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(folder / "tests/fixtures/native_rotated_cube_classification_failure.json");
    MINI_CHECK(input.good());
    const auto fixture = nlohmann::json::parse(input);
    const NurbsSurface source = NurbsSurface::jsonload(fixture["shape"]["surfaces"][4]);
    const Point original(fixture["query"][0], fixture["query"][1], fixture["query"][2]);
    const Vector direction(-0.217, 1, 0.413);
    int index = 0;
    for (double scale : {0.001, 1.0, 1000.0}) {
        const Xform transform = Xform::translation(31 * scale, -17 * scale, 13 * scale) * Xform::scale_xyz(scale, scale, scale);
        const NurbsSurface surface = source.transformed(transform);
        const Point origin = original.transformed(transform);
        const double tolerance = scale * 1e-7;
        const auto result = run("saved_grazing_" + std::to_string(index++), surface, origin, direction, tolerance);
        complete(result);
        MINI_CHECK(result.roots.size() == 1);
        MINI_CHECK(std::abs(result.roots[0].distance - scale * 0.5486470850147471) <= tolerance);
        verify(surface, origin, direction, result.roots[0], tolerance);
    }
}

MINI_TEST("Surface Ray", "Affine Nonuniform Domains") {
    NurbsSurface surface = NurbsSurface::create(false, false, 1, 1, 2, 2,
        {{-1, -1, 1.5}, {-1, 1, 2.1}, {1, -1, 1.9}, {1, 1, 2.5}});
    MINI_CHECK(surface.set_domain(0, 17.0, 17.001));
    MINI_CHECK(surface.set_domain(1, -4e-6, 9e-6));
    const Point origin(-0.3, 0.4, -2);
    const Vector direction(0, 0, 7);
    const auto result = run("nonuniform_domains", surface, origin, direction, 1e-8);
    complete(result);
    MINI_CHECK(result.roots.size() == 1);
    MINI_CHECK(result.roots[0].point.distance({-0.3, 0.4, 2.06}) <= 1e-8);
    verify(surface, origin, direction, result.roots[0], 1e-8);
    const auto miss = run("affine_miss", surface, {1.01, 0.4, -2}, direction, 1e-8);
    complete(miss);
    MINI_CHECK(miss.roots.empty());
}

MINI_TEST("Surface Ray", "Rational Sphere Generic Rays") {
    const NurbsSurface surface = BRep::create_sphere(1.0).m_surfaces[0];
    const Point origin(-2, 0.13, 0.21);
    const Vector direction(3, 0, 0);
    const auto result = run("rational_sphere", surface, origin, direction);
    complete(result);
    MINI_CHECK(result.roots.size() == 2);
    const double x = std::sqrt(1.0 - 0.13 * 0.13 - 0.21 * 0.21);
    MINI_CHECK(std::abs(result.roots[0].distance - (2.0 - x)) <= 1e-7);
    MINI_CHECK(std::abs(result.roots[1].distance - (2.0 + x)) <= 1e-7);
    for (const auto& root : result.roots)
        verify(surface, origin, direction, root, 1e-7);
}

MINI_TEST("Surface Ray", "Positive Near Tangent Gap") {
    const NurbsSurface surface = bowl();
    const Vector direction(1, 0, 0);
    for (double gap : {1e-5, 1e-8}) {
        const auto result = run("positive_gap_" + std::to_string(gap == 1e-8), surface, {-2, 0, -gap}, direction);
        complete(result);
        MINI_CHECK(result.roots.empty());
    }
    const auto tangent = run("tangent_bowl", surface, {-2, 0, 0}, direction, 1e-7, {4096, 64});
    MINI_CHECK(!tangent.complete && !tangent.failure.empty());
}

MINI_TEST("Surface Ray", "Compressed Distinct Parameter Branches") {
    const double lambda = 1e8;
    const std::vector<std::vector<Point>> points = {
        {{0.25, -1, 0}, {-0.25, -1, 0.5}, {0.25, -1, 1}},
        {{0.25, 1, 0}, {-0.25, 1, 0.5}, {0.25, 1, 1}}};
    const std::vector<std::vector<double>> weights(2, {1, lambda, lambda * lambda});
    const NurbsSurface surface = NurbsSurface::create_from_parameters(points, weights,
        {0, 1}, {0, 1}, {3, 3}, {2, 2}, 2, 1);
    const Point origin(0.0625, 0.17, -1);
    const Vector direction(0, 0, 1);
    const auto result = run("compressed_branches", surface, origin, direction);
    complete(result);
    MINI_CHECK(result.roots.size() == 2);
    for (int i = 0; i < 2; ++i) {
        const auto& root = result.roots[i];
        const double s = lambda * root.u / (1.0 + (lambda - 1.0) * root.u);
        MINI_CHECK(std::abs(s - (i == 0 ? 0.25 : 0.75)) <= 1e-7);
        MINI_CHECK(std::abs(root.distance - (i == 0 ? 1.25 : 1.75)) <= 1e-7);
        MINI_CHECK(root.crossing == (i == 0 ? -1 : 1));
        verify(surface, origin, direction, root, 1e-7);
    }
    MINI_CHECK(result.roots[1].u - result.roots[0].u < 1e-7);
    MINI_CHECK(result.roots[0].enclosure[1] < result.roots[1].enclosure[0]);
}

MINI_TEST("Surface Ray", "Original Knot Boundary Root") {
    const NurbsSurface surface = NurbsSurface::create(false, false, 1, 1, 3, 2,
        {{-1, -1, 2}, {-1, 1, 2}, {0, -1, 2}, {0, 1, 2}, {1, -1, 2}, {1, 1, 2}});
    const auto result = run("internal_knot_root", surface, {0, 0.3, -1}, {0, 0, 1});
    complete(result);
    MINI_CHECK(result.roots.size() == 1);
    MINI_CHECK(std::abs(result.roots[0].distance - 3.0) <= 1e-7);
    MINI_CHECK(result.roots[0].chart[0] < result.roots[0].u && result.roots[0].chart[1] > result.roots[0].u);
}

MINI_TEST("Surface Ray", "Same World Point Distinct Branches") {
    const double lambda = 1e8;
    const std::vector<std::vector<Point>> points = {
        {{0.25, -1, 0}, {-0.25, -1, 0}, {0.25, -1, 0}},
        {{0.25, 1, 0}, {-0.25, 1, 0}, {0.25, 1, 0}}};
    const std::vector<std::vector<double>> weights(2, {1, lambda, lambda * lambda});
    const NurbsSurface surface = NurbsSurface::create_from_parameters(points, weights,
        {0, 1}, {0, 1}, {3, 3}, {2, 2}, 2, 1);
    const auto result = run("same_world_branches", surface, {0.0625, 0.17, -1}, {0, 0, 1});
    complete(result);
    MINI_CHECK(result.roots.size() == 2);
    std::vector<double> parameters;
    for (const auto& root : result.roots) {
        parameters.push_back(lambda * root.u / (1.0 + (lambda - 1.0) * root.u));
        MINI_CHECK(root.crossing == (parameters.back() < 0.5 ? -1 : 1));
        MINI_CHECK(std::abs(root.distance - 1.0) <= 1e-7);
        MINI_CHECK(root.point.distance({0.0625, 0.17, 0}) <= 1e-7);
    }
    std::sort(parameters.begin(), parameters.end());
    MINI_CHECK(std::abs(parameters[0] - 0.25) <= 1e-7 && std::abs(parameters[1] - 0.75) <= 1e-7);
}

MINI_TEST("Surface Ray", "Boundary Coincidence And Budget Are Explicit") {
    const NurbsSurface surface = plane();
    const auto boundary = run("natural_boundary", surface, {-1, 0, -1}, {0, 0, 1}, 1e-7, {4096, 64});
    MINI_CHECK(!boundary.complete && !boundary.failure.empty());
    const auto coincident = run("coincident_line", surface, {-2, 0, 2}, {1, 0, 0});
    MINI_CHECK(!coincident.complete && !coincident.failure.empty());
    const auto limited = run("zero_depth_tangency", bowl(), {-2, 0, 0}, {1, 0, 0}, 1e-7, {1, 0});
    MINI_CHECK(!limited.complete && !limited.failure.empty());
    const auto behind = run("behind_origin", surface, {0, 0, 3}, {0, 0, 1});
    complete(behind);
    MINI_CHECK(behind.roots.empty());
    const auto at_origin = run("root_at_origin", surface, {0, 0, 2}, {0, 0, 1}, 1e-7, {4096, 64});
    MINI_CHECK(!at_origin.complete && !at_origin.failure.empty());
}

MINI_TEST("Surface Ray", "Certified Crossing And Orientation") {
    for (int dominant = 0; dominant < 3; ++dominant) {
        std::array<int, 2> transverse;
        int index = 0;
        for (int d = 0; d < 3; ++d)
            if (d != dominant)
                transverse[index++] = d;
        Point target;
        target[dominant] = 2.0;
        std::vector<Point> points;
        for (double u : {-1.0, 1.0})
            for (double v : {-1.0, 1.0}) {
                Point point = target;
                point[transverse[0]] = u;
                point[transverse[1]] = v;
                points.push_back(point);
            }
        const NurbsSurface source = NurbsSurface::create(false, false, 1, 1, 2, 2, points);
        for (int direction_sign : {-1, 1})
            for (int orientation = 0; orientation < 3; ++orientation) {
                NurbsSurface surface = source;
                if (orientation == 1)
                    MINI_CHECK(surface.reverse(0));
                if (orientation == 2)
                    MINI_CHECK(surface.transpose());
                Vector direction(0.2, 0.3, 0.4);
                direction[dominant] = direction_sign * 2.0;
                const Point origin = target - direction.normalized() * 3.0;
                const auto result = run("orientation_" + std::to_string(dominant) + "_" +
                    std::to_string(direction_sign) + "_" + std::to_string(orientation), surface, origin, direction);
                complete(result);
                MINI_CHECK(result.roots.size() == 1);
                const int expected = (dominant % 2 == 0 ? 1 : -1) * direction_sign * (orientation == 0 ? 1 : -1);
                MINI_CHECK(result.roots[0].crossing == expected);
                MINI_CHECK(result.roots[0].point.distance(target) <= 1e-7);
                verify(surface, origin, direction, result.roots[0], 1e-7);
            }
    }
}

MINI_TEST("Surface Ray", "Origin Candidates Before Trim Classification") {
    const NurbsSurface surface = plane();
    const surface_ray::Options options{200000, 96, true};
    for (double sign : {-1.0, 1.0}) {
        const auto result = run("origin_candidate_" + std::to_string(sign), surface,
            {0, 0, 2}, {0.2, 0.3, sign}, 1e-7, options);
        complete(result);
        MINI_CHECK(result.roots.size() == 1);
        const auto& root = result.roots.front();
        MINI_CHECK(root.distance_enclosure[0] <= 0.0 && root.distance_enclosure[1] >= 0.0);
        MINI_CHECK(root.point.distance({0, 0, 2}) <= 1e-7);
        MINI_CHECK(root.crossing == static_cast<int>(sign));
    }
    const auto behind = run("origin_option_behind", surface, {0, 0, 2.0000002},
        {0, 0, 1}, 1e-7, options);
    complete(behind);
    MINI_CHECK(behind.roots.empty());
    const auto forward = run("origin_option_forward", surface, {0, 0, 1.9999998},
        {0, 0, 1}, 1e-7, options);
    complete(forward);
    MINI_CHECK(forward.roots.size() == 1);
    MINI_CHECK(forward.roots.front().distance_enclosure[0] > 0.0);
    const auto coincident = run("origin_option_coincident", surface, {0, 0, 2},
        {1, 0, 0}, 1e-7, {4096, 64, true});
    MINI_CHECK(!coincident.complete && !coincident.failure.empty());
}

MINI_TEST("Surface Ray", "Bore Supporting Plane Origin Candidates") {
    const BRep solid = BRep::create_block_with_hole(4, 4, 2, 1);
    const auto& surface = solid.m_surfaces[solid.m_faces.back().surface_index];
    const std::array<Vector, 7> directions = {
        Vector(1.0, 0.371, 0.529), Vector(-0.217, 1.0, 0.413), Vector(0.319, -0.271, 1.0),
        Vector(-1.0, 0.723, -0.219), Vector(0.613, -1.0, -0.417), Vector(-0.531, -0.337, -1.0), Vector(1.0, 1.0, 1.0)
    };
    int index = 0;
    for (const Vector& direction : directions) {
        const auto result = run("bore_origin_candidate_" + std::to_string(index++), surface,
            {0, 0, 1}, direction, 1e-7, {200000, 96, true});
        complete(result);
        MINI_CHECK(result.roots.size() == 1);
        const auto& root = result.roots.front();
        MINI_CHECK(root.distance_enclosure[0] <= 0.0 && root.distance_enclosure[1] >= 0.0);
        MINI_CHECK(root.point.distance({0, 0, 1}) <= 1e-7);
    }
}

MINI_TEST("Surface Ray", "Flat Patches In Closed Form") {
    const NurbsSurface surface = plane();
    const Vector up(0, 0, 1);
    const auto inside = run("flat_inside", surface, {0.3, -0.2, -1}, up);
    complete(inside);
    MINI_CHECK(inside.cells == 1 && inside.roots.size() == 1 && inside.roots[0].crossing == 1);
    MINI_CHECK(std::abs(inside.roots[0].distance - 3.0) < 1e-12 && inside.roots[0].point.distance({0.3, -0.2, 2}) < 1e-12);
    verify(surface, {0.3, -0.2, -1}, up, inside.roots[0], 1e-7);
    const auto outside = run("flat_outside", surface, {1.5, 0, -1}, up);
    MINI_CHECK(outside.complete && outside.cells == 1 && outside.roots.empty());
    const auto edge = run("flat_edge", surface, {-1, 0.25, -1}, up);
    MINI_CHECK(!edge.complete && edge.cells == 1 && edge.roots.empty());
    const auto corner = run("flat_corner", surface, {1, 1, -1}, up);
    MINI_CHECK(!corner.complete && corner.cells == 1 && corner.roots.empty());
    const auto edge_behind = run("flat_edge_behind", surface, {-1, 0.25, 3}, up);
    MINI_CHECK(edge_behind.complete && edge_behind.roots.empty());
    const auto in_plane = run("flat_in_plane", surface, {-2, 0, 2}, {1, 0, 0});
    MINI_CHECK(!in_plane.complete && in_plane.cells == 1);
    const auto beside = run("flat_beside", surface, {-2, 0, 1}, {1, 0, 0});
    MINI_CHECK(beside.complete && beside.cells == 1 && beside.roots.empty());
    const auto on_plane = run("flat_on_plane", surface, {0, 0, 2}, {0.2, 0.3, 1}, 1e-7, {200000, 96, true});
    complete(on_plane);
    MINI_CHECK(on_plane.cells == 1 && on_plane.roots.size() == 1);
    MINI_CHECK(on_plane.roots[0].distance_enclosure[0] <= 0.0 && on_plane.roots[0].distance_enclosure[1] >= 0.0);

    const Xform placed = Xform::translation(2000, -2000, 2000) * Xform::rotation(Vector(1, 2, 3), 0.7);
    const NurbsSurface turned = surface.transformed(placed);
    const Point origin = Point(0.3, -0.2, -1).transformed(placed);
    const Vector direction = Point(0.3, -0.2, 2).transformed(placed) - origin;
    const auto far = run("flat_far", turned, origin, direction);
    complete(far);
    MINI_CHECK(far.cells == 1 && far.roots.size() == 1 && std::abs(far.roots[0].distance - 3.0) < 1e-9);
    verify(turned, origin, direction, far.roots[0], 1e-7);

    const NurbsSurface triangle = NurbsSurface::create(false, false, 1, 1, 2, 2,
        {{-1, -1, 2}, {0, 1, 2}, {1, -1, 2}, {0, 1, 2}});
    const auto in_triangle = run("triangle_inside", triangle, {0.1, -0.5, -1}, up);
    complete(in_triangle);
    MINI_CHECK(in_triangle.cells == 1 && in_triangle.roots.size() == 1);
    MINI_CHECK(in_triangle.roots[0].point.distance({0.1, -0.5, 2}) < 1e-12);
    verify(triangle, {0.1, -0.5, -1}, up, in_triangle.roots[0], 1e-7);
    const auto off_triangle = run("triangle_outside", triangle, {0.9, 0.5, -1}, up);
    MINI_CHECK(off_triangle.complete && off_triangle.cells == 1 && off_triangle.roots.empty());
    const auto triangle_base = run("triangle_base", triangle, {0.25, -1, -1}, up);
    MINI_CHECK(!triangle_base.complete && triangle_base.cells == 1);
}

MINI_TEST("Surface Ray", "Invalid Inputs") {
    for (double tolerance : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try {
            static_cast<void>(surface_ray::intersect(plane(), {}, {0, 0, 1}, tolerance));
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        MINI_CHECK(rejected);
    }
}
