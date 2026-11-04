#include "brep_thicken.h"
#include "brep_offset.h"
#include "nurbs_offset.h"
#include "nurbs_normal_sweep.h"
#include "brep_assemble.h"
#include "brep_acceptance.h"
#include "curve_surface_intersection.h"
#include "brep_classify.h"
#include "brep_measure.h"
#include "mini_test.h"
#include "primitives.h"
#include "session.h"
#include "tolerance.h"

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>

using namespace session_cpp;
using namespace session_cpp::mini_test;

namespace {

constexpr BRepOrientation Forward = BRepOrientation::Forward;
constexpr BRepOrientation Reversed = BRepOrientation::Reversed;

int trim_wire(BRep& brep, const std::array<double, 4>& bounds) {
    const NurbsSurface& surface = brep.m_surfaces[0];
    const std::array<Point, 4> uv = {Point(bounds[0], bounds[2], 0), Point(bounds[1], bounds[2], 0),
        Point(bounds[1], bounds[3], 0), Point(bounds[0], bounds[3], 0)};
    const std::array<double, 4> fixed = {bounds[2], bounds[1], bounds[3], bounds[0]};
    const int first_vertex = brep.vertex_count();
    for (const Point& point : uv)
        brep.add_vertex(surface.point_at(point[0], point[1]));
    std::vector<BRepRef> edges;
    for (int i = 0; i < 4; ++i) {
        const int direction = i % 2;
        NurbsCurve curve = surface.iso_curve(direction, fixed[i]);
        MINI_CHECK(curve.trim(bounds[2 * direction], bounds[2 * direction + 1]));
        if (i >= 2)
            MINI_CHECK(curve.reverse());
        MINI_CHECK(curve.set_domain(5, 9));
        NurbsCurve pc = NurbsCurve::create(false, 1, {uv[i], uv[(i + 1) % 4]});
        MINI_CHECK(pc.set_domain(-4, 2));
        const int edge = brep.add_edge(brep.add_curve_3d(curve), first_vertex + i, first_vertex + (i + 1) % 4);
        brep.add_pcurve(edge, 0, brep.add_curve_2d(pc));
        edges.push_back({edge, Forward});
    }
    return brep.add_wire(edges);
}

BRep saddle_face(bool rational = false) {
    std::vector<Point> points;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            const double x = -1 + 2.0 * i / 3;
            const double y = -1 + 2.0 * j / 3;
            points.push_back({x, y, 0.2 * x * y});
        }
    NurbsSurface surface = NurbsSurface::create(false, false, 3, 3, 4, 4, points);
    MINI_CHECK(surface.set_domain(0, 3, 7) && surface.set_domain(1, -2, 4));
    if (rational) {
        MINI_CHECK(surface.to_rational());
        const std::array<double, 4> weights = {1, 0.8, 1.3, 1.1};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                double* cv = surface.cv(i, j);
                for (int d = 0; d < 4; ++d)
                    cv[d] *= weights[i] * weights[3 - j];
            }
    }
    BRep result;
    result.add_surface(surface);
    const int wire = trim_wire(result, {3, 7, -2, 4});
    result.add_face(0, {{wire, Forward}});
    result.add_shell({{0, Forward}});
    return result;
}

void save_geometry(const std::string& name, const BRep& brep, const Color& color) {
    const std::filesystem::path folder = "build/native-geometry-tests/viewer";
    std::filesystem::create_directories(folder);
    BRep geometry = brep;
    geometry.name = name;
    geometry.surfacecolor = color;
    geometry.file_json_dump((folder / (name + ".brep.json")).string());
    Session scene(name);
    scene.add_brep(geometry);
    scene.pb_dump((folder / (name + ".pb")).string());
}

BRep saved_thickening(
    const BRep& source,
    double thickness,
    double tolerance,
    const std::string& name
) {
    save_geometry(name + "_source", source, Color(75 / 255.f, 125 / 255.f, 170 / 255.f));
    std::ofstream("build/native-geometry-tests/viewer/" + name + ".assertions.json")
        << nlohmann::ordered_json({{"passed", false}, {"stage", "construction"},
            {"source_scene", name + "_source.pb"}, {"result_scene", name + "_result.pb"}}).dump(2);
    try {
        const BRep result = brep_thicken::face(source, thickness, tolerance);
        save_geometry(name + "_result", result, Color(180 / 255.f, 110 / 255.f, 155 / 255.f));
        return result;
    } catch (const brep_assemble::ParameterError& error) {
        const nlohmann::ordered_json fixture = {{"curve", error.curve.jsondump()}, {"pcurve", error.pcurve.jsondump()},
            {"surface", error.surface.jsondump()}, {"tolerance", error.tolerance}, {"match", static_cast<int>(error.match)},
            {"failure", error.what()}, {"thickness", thickness}};
        std::ofstream("build/native-geometry-tests/viewer/" + name + "_parameter_failure.json") << fixture.dump(2);
        throw;
    } catch (const std::exception& error) {
        const nlohmann::ordered_json fixture = {{"failure", error.what()}, {"thickness", thickness}, {"tolerance", tolerance}};
        std::ofstream("build/native-geometry-tests/viewer/" + name + "_failure.json") << fixture.dump(2);
        throw;
    }
}

double boundary_residual(const BRep& brep) {
    double maximum = 0;
    for (const BRepEdge& edge : brep.m_edges)
        for (const BRepCurveOnSurface& relation : edge.pcurves)
            for (int index : {relation.curve_2d_index, relation.curve_2d_index_2}) {
                if (index < 0)
                    continue;
                const NurbsCurve& pc = brep.m_curves_2d[index];
                const NurbsSurface& support = brep.m_surfaces[relation.surface_index];
                for (int i = 0; i <= 97; ++i) {
                    const double fraction = i / 97.0;
                    const Point uv = pc.point_at(pc.domain_start() + fraction * (pc.domain_end() - pc.domain_start()));
                    Point world = brep.m_vertices[edge.start_vertex].point;
                    if (!edge.degenerated) {
                        const NurbsCurve& curve = brep.m_curves_3d[edge.curve_3d_index];
                        world = curve.point_at(curve.domain_start() + fraction * (curve.domain_end() - curve.domain_start()));
                    }
                    maximum = std::max(maximum, world.distance(support.point_at(uv[0], uv[1])));
                }
            }
    return maximum;
}

void check_result(
    const BRep& result,
    double expected,
    double tolerance,
    const std::string& name
) {
    const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    nlohmann::ordered_json record = {{"expected_volume", expected}, {"tolerance", tolerance}, {"stage", "volume"},
        {"passed", false}, {"source_scene", name + "_source.pb"}, {"result_scene", name + "_result.pb"}};
    const auto save_stage = [&]() {
        record["elapsed_seconds"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        std::ofstream("build/native-geometry-tests/viewer/" + name + ".assertions.json") << record.dump(2);
    };
    save_stage();
    const double volume = brep_measure::volume(result, 1e-7);
    record["volume"] = volume;
    record["volume_seconds"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    record["stage"] = "boundary_residual";
    save_stage();
    const double residual = boundary_residual(result);
    record["boundary_residual"] = residual;
    record["faces"] = result.face_count();
    record["shells"] = result.shell_count();
    record["solids"] = result.solid_count();
    record["is_solid"] = result.is_solid();
    record["stage"] = "topology_and_interior_checks";
    save_stage();
    MINI_CHECK(result.is_solid() && result.solid_count() == 1 && result.shell_count() == 1);
    MINI_CHECK(residual <= tolerance);
    MINI_CHECK(volume > 0 && std::abs(volume - expected) < std::max(1e-7, tolerance * 12));
}

void finish_checks(const std::string& name) {
    const std::string path = "build/native-geometry-tests/viewer/" + name + ".assertions.json";
    std::ifstream input(path);
    MINI_CHECK(input.good());
    nlohmann::ordered_json record = nlohmann::ordered_json::parse(input);
    input.close();
    record["passed"] = true;
    record["stage"] = "complete";
    std::ofstream(path) << record.dump(2);
}

double saddle_volume(double thickness, double extent = 1) {
    const std::array<double, 5> points = {-0.9061798459386640, -0.5384693101056831, 0,
        0.5384693101056831, 0.9061798459386640};
    const std::array<double, 5> weights = {0.2369268850561891, 0.4786286704993665, 0.5688888888888889,
        0.4786286704993665, 0.2369268850561891};
    double integral = 0;
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j) {
            const double x = points[i] * extent;
            const double y = points[j] * extent;
            const double metric = 1 + 0.04 * (x * x + y * y);
            const double curvature = -0.04 / (metric * metric);
            integral += weights[i] * weights[j] * extent * extent * std::sqrt(metric)
                * (1 + thickness * thickness * curvature / 3);
        }
    return std::abs(thickness) * integral;
}

void check_interior_probe(
    const BRep& result,
    const Point& query,
    const Point& uv,
    double tolerance,
    brep_classify::State expected,
    const std::string& name
) {
    const std::string path = "build/native-geometry-tests/viewer/" + name + ".assertions.json";
    std::ifstream input(path);
    MINI_CHECK(input.good());
    nlohmann::ordered_json record = nlohmann::ordered_json::parse(input);
    input.close();
    if (!record.contains("interior_checks"))
        record["interior_checks"] = nlohmann::ordered_json::array();
    record["stage"] = "interior_classifier";
    record["interior_checks"].push_back({{"query", {query[0], query[1], query[2]}},
        {"source_uv", {uv[0], uv[1]}}, {"expected", static_cast<int>(expected)},
        {"expected_name", expected == brep_classify::State::Inside ? "inside" : "outside"},
        {"tolerance", tolerance}, {"elapsed_seconds", 0.0}, {"status", "running"}});
    const auto save = [&]() { std::ofstream(path) << record.dump(2); };
    save();
    nlohmann::ordered_json& probe = record["interior_checks"].back();
    const auto started = std::chrono::steady_clock::now();
    brep_classify::State actual;
    try {
        actual = brep_classify::point(result, query, tolerance);
    } catch (const std::exception& error) {
        probe["elapsed_seconds"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        probe["status"] = "exception";
        probe["failure"] = error.what();
        save();
        throw;
    }
    probe["elapsed_seconds"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    probe["actual"] = static_cast<int>(actual);
    probe["status"] = actual == expected ? "passed" : "failed";
    save();
    MINI_CHECK(actual == expected);
}

void check_interior(
    const BRep& source,
    const BRep& result,
    double thickness,
    double tolerance,
    const std::string& name
) {
    const NurbsSurface& surface = source.m_surfaces[source.m_faces[0].surface_index];
    const double orientation = source.face_orientation(0) == Forward ? 1 : -1;
    for (const Point uv : {Point(3.8, -0.7, 0), Point(6.1, 2.6, 0), Point(5.9, -0.8, 0)}) {
        const Point point = surface.point_at(uv[0], uv[1]);
        const Vector normal = surface.normal_at(uv[0], uv[1]) * orientation;
        for (double distance : {0.5, -0.25, 1.25})
            check_interior_probe(
                result,
                point + normal * (thickness * distance),
                uv,
                tolerance,
                distance == 0.5 ? brep_classify::State::Inside : brep_classify::State::Outside,
                name
            );
    }
}

BRep cylinder_face() {
    BRep source = BRep::create_cylinder(1, 2);
    int lateral = -1;
    for (int i = 0; i < source.face_count(); ++i)
        if (!source.m_surfaces[source.m_faces[i].surface_index].is_planar())
            lateral = i;
    MINI_CHECK(lateral >= 0);
    const BRepOrientation orientation = source.face_orientation(lateral);
    source.m_faces = {source.m_faces[lateral]};
    source.m_shells.clear();
    source.m_solids.clear();
    source.add_shell({{0, orientation}});
    return source;
}

bool rejected(const BRep& source, double thickness, double tolerance = 1e-7) {
    try {
        brep_thicken::face(source, thickness, tolerance);
        return false;
    } catch (const std::exception&) {
        return true;
    }
}

}

MINI_TEST("BRepThicken", "Planar Face Both Signs") {
    const BRep source = BRep::from_polylines({Polyline({{0, 0, 0}, {2, 0, 0}, {2, 3, 0}, {0, 3, 0}, {0, 0, 0}})});
    for (double thickness : {-0.3, 0.3}) {
        const std::string name = thickness > 0 ? "native_thicken_plane_positive" : "native_thicken_plane_negative";
        const BRep result = saved_thickening(source, thickness, 1e-7, name);
        MINI_CHECK(result.face_count() == 6 && result.edge_count() == 12 && result.vertex_count() == 8);
        check_result(result, 1.8, 1e-7, name);
        MINI_CHECK(brep_classify::point(result, {1, 1, thickness * 0.5}, 1e-7) == brep_classify::State::Inside);
        finish_checks(name);
    }
}

MINI_TEST("BRepThicken", "Doubly Curved Face Both Signs And Orientations") {
    BRep source = saddle_face();
    for (BRepOrientation orientation : {Forward, Reversed}) {
        source.m_shells[0].faces[0].orientation = orientation;
        for (double thickness : {-0.12, 0.12}) {
            const std::string name = "native_thicken_saddle_" + std::to_string(static_cast<int>(orientation))
                + (thickness > 0 ? "_positive" : "_negative");
            const BRep result = saved_thickening(source, thickness, 2e-5, name);
            MINI_CHECK(result.face_count() == 6);
            check_result(result, saddle_volume(thickness), 2e-5, name);
            check_interior(
                source,
                result,
                thickness,
                2e-5,
                name
            );
            finish_checks(name);
        }
    }
}

MINI_TEST("BRepThicken", "Rational Freeform Boundaries With Different Weights") {
    const BRep source = saddle_face(true);
    const std::string name = "native_thicken_rational_saddle";
    const BRep result = saved_thickening(source, 0.1, 3e-5, name);
    MINI_CHECK(source.m_curves_3d[0].is_rational());
    MINI_CHECK(result.face_count() == 6);
    check_result(result, saddle_volume(0.1), 3e-5, name);
    check_interior(
        source,
        result,
        0.1,
        3e-5,
        name
    );
    finish_checks(name);
}

MINI_TEST("BRepThicken", "Freeform Face With Trimmed Hole") {
    BRep source = saddle_face();
    const int hole = trim_wire(source, {4.5, 5.5, 0.25, 1.75});
    source.m_faces[0].wires.push_back({hole, Reversed});
    const std::string name = "native_thicken_saddle_hole";
    const BRep result = saved_thickening(source, 0.12, 2e-5, name);
    MINI_CHECK(result.face_count() == 10);
    check_result(result, saddle_volume(0.12) - saddle_volume(0.12, 0.25), 2e-5, name);
    check_interior(
        source,
        result,
        0.12,
        2e-5,
        name
    );
    MINI_CHECK(brep_classify::point(result, {0, 0, 0.06}, 2e-5) == brep_classify::State::Outside);
    finish_checks(name);
}

MINI_TEST("BRepThicken", "Curved Hole And Closed Boundary Seams") {
    const NurbsCurve outer = Primitives::circle(0, 0, 0, 2);
    NurbsCurve inner = Primitives::circle(0.3, -0.1, 0, 0.5);
    MINI_CHECK(inner.reverse());
    const BRep source = BRep::from_nurbscurves({outer}, {{inner}});
    for (double thickness : {-0.2, 0.2}) {
        const std::string name = thickness > 0 ? "native_thicken_annulus_positive" : "native_thicken_annulus_negative";
        const BRep result = saved_thickening(source, thickness, 1e-7, name);
        MINI_CHECK(result.face_count() == 4);
        check_result(result, 0.75 * Tolerance::PI, 1e-7, name);
        int seams = 0;
        for (const BRepEdge& edge : result.m_edges)
            for (const BRepCurveOnSurface& pc : edge.pcurves)
                seams += pc.curve_2d_index_2 >= 0 ? 1 : 0;
        MINI_CHECK(seams == 2);
        MINI_CHECK(brep_classify::point(result, {0.3, -0.1, thickness * 0.5}, 1e-7) == brep_classify::State::Outside);
        MINI_CHECK(brep_classify::point(result, {-1, 0, thickness * 0.5}, 1e-7) == brep_classify::State::Inside);
        finish_checks(name);
    }
}

MINI_TEST("BRepThicken", "Open Cylinder With Original And Wall Seams") {
    const BRep source = cylinder_face();
    for (double thickness : {-0.1, 0.1}) {
        const std::string name = thickness > 0 ? "native_thicken_cylinder_positive" : "native_thicken_cylinder_negative";
        const BRep result = saved_thickening(source, thickness, 1e-7, name);
        MINI_CHECK(result.face_count() == 4);
        const double expected = 2 * Tolerance::PI * std::abs((1 + thickness) * (1 + thickness) - 1);
        check_result(result, expected, 1e-7, name);
        MINI_CHECK(brep_classify::point(result, {1 + thickness * 0.5, 0, 1}, 1e-7) == brep_classify::State::Inside);
        MINI_CHECK(brep_classify::point(result, {0, 0, 1}, 1e-7) == brep_classify::State::Outside);
        finish_checks(name);
    }
}

MINI_TEST("BRepThicken", "Invalid Inputs And Collapse") {
    const BRep source = saddle_face();
    MINI_CHECK(rejected(source, 0));
    MINI_CHECK(rejected(source, 1e-9));
    MINI_CHECK(rejected(source, 0.1, 0));
    MINI_CHECK(rejected(source, std::numeric_limits<double>::infinity()));
    MINI_CHECK(rejected(source, 0.1, std::numeric_limits<double>::quiet_NaN()));
    MINI_CHECK(rejected(BRep(), 0.1));
    MINI_CHECK(rejected(BRep::create_box(1, 1, 1), 0.1));
    MINI_CHECK(rejected(BRep::create_sphere(1), 0.1));
    MINI_CHECK(rejected(cylinder_face(), -1));
}

namespace {

BRep parabolic_focal_face() {
    const NurbsSurface surface = NurbsSurface::create(false, false, 2, 1, 3, 2,
        {{0, 0, 0}, {0, 1, 0}, {.5, 0, 0}, {.5, 1, 0}, {1, 0, 16}, {1, 1, 16}});
    BRep source;
    source.add_surface(surface);
    source.add_face(0, {{trim_wire(source, {0, 1, 0, 1}), Forward}});
    source.add_shell({{0, Forward}});
    MINI_CHECK(source.is_valid() && source.face_count() == 1 && source.solid_count() == 0);
    for (double u : {0.0, .01, .25, .5, 1.0})
        MINI_CHECK(surface.point_at(u, .5).distance({u, .5, 16 * u * u}) < 1e-12);
    return source;
}

double parabolic_offset_factor(double u, double thickness) {
    return 1 - 32 * thickness / std::pow(1 + 1024 * u * u, 1.5);
}

}

MINI_TEST("BRepThicken", "Localized Parabolic Focal Fold Is Rejected") {
    const BRep source = parabolic_focal_face();
    const double thickness = 1.0 / 16;
    MINI_CHECK(parabolic_offset_factor(.01, thickness) < 0);
    for (double u : {.25, .5, .75})
        MINI_CHECK(parabolic_offset_factor(u, thickness) > 0);
    const std::string name = "native_thicken_localized_parabolic_focal";
    bool rejected = false;
    try {
        saved_thickening(source, thickness, 1e-5, name);
    } catch (const std::exception& error) {
        const std::string failure = error.what();
        rejected = failure.find("focal") != std::string::npos || failure.find("collapse") != std::string::npos;
        if (!rejected)
            throw;
    }
    MINI_CHECK(rejected);
}

MINI_TEST("BRepThicken", "Subfocal Parabolic Normal Sweep") {
    const BRep source = parabolic_focal_face();
    const double thickness = 1.0 / 64;
    MINI_CHECK(parabolic_offset_factor(0, thickness) == .5);
    const std::string name = "native_thicken_subfocal_parabolic";
    const BRep result = saved_thickening(source, thickness, 1e-5, name);
    const double area = .5 * std::sqrt(1025.0) + std::asinh(32.0) / 64;
    const double expected = thickness * area - .5 * thickness * thickness * std::atan(32.0);
    check_result(result, expected, 1e-5, name);
    MINI_CHECK(brep_acceptance::shell_vertex_links(result, result.m_shells[0])
        == brep_acceptance::VertexLinks::Matched);
    for (const BRepEdge& edge : result.m_edges)
        for (const BRepCurveOnSurface& relation : edge.pcurves)
            for (int index : {relation.curve_2d_index, relation.curve_2d_index_2})
                if (index >= 0)
                    MINI_CHECK(curve_surface_intersection::same_parameter(result.m_curves_3d[edge.curve_3d_index],
                        result.m_curves_2d[index], result.m_surfaces[relation.surface_index], 1e-5)
                        == curve_surface_intersection::ParameterMatch::Matched);
    for (double u : {.1, .5, .9}) {
        const Point point = source.m_surfaces[0].point_at(u, .5);
        const Vector normal = source.m_surfaces[0].normal_at(u, .5);
        for (double fraction : {-.5, .5, 1.5})
            check_interior_probe(result, point + normal * (thickness * fraction), {u, .5, 0},
                1e-5, fraction == .5 ? brep_classify::State::Inside : brep_classify::State::Outside, name);
    }
    finish_checks(name);
}


MINI_TEST("BRepThicken", "Interval Signed Normal Sweep Regularity") {
    const NurbsSurface surface = parabolic_focal_face().m_surfaces[0];
    const auto inside = [](double u, double v) { return u > 0 && u < 1 && v > 0 && v < 1; };
    const normal_sweep::Result subfocal = normal_sweep::check(surface, {0, 1, 0, 1}, 1.0 / 64, 4096, inside);
    MINI_CHECK(subfocal.state == normal_sweep::State::Regular);
    const normal_sweep::Result folded = normal_sweep::check(surface, {0, 1.0 / 64, 0, 1}, 1.0 / 16, 4096, inside);
    MINI_CHECK(folded.state == normal_sweep::State::Fold);
    MINI_CHECK(inside(folded.witness[0], folded.witness[1]));
    MINI_CHECK(parabolic_offset_factor(folded.witness[0], 1.0 / 16) < 0);
    MINI_CHECK(normal_sweep::check(surface, {0, 1, 0, 1}, -1.0 / 16, 4096, inside).state
        == normal_sweep::State::Regular);
    MINI_CHECK(normal_sweep::check(surface, {0, 1, 0, 1}, 1.0 / 64, 0, inside).state
        == normal_sweep::State::Unknown);
}

MINI_TEST("BRepThicken", "Interval Sweep Original Trim And Mixed Curvature") {
    const NurbsSurface surface = parabolic_focal_face().m_surfaces[0];
    const auto inside = [](double u, double v) { return u > .25 && u < 1 && v > 0 && v < 1; };
    MINI_CHECK(normal_sweep::check(surface, {.25, 1, 0, 1}, 1.0 / 16, 4096, inside).state
        == normal_sweep::State::Regular);
    MINI_CHECK(normal_sweep::check(surface, {0, 1.0 / 64, 0, 1}, 1.0 / 16, 64, inside).state
        == normal_sweep::State::Unknown);
    const NurbsSurface saddle = NurbsSurface::create(false, false, 1, 1, 2, 2,
        {{-1, -1, 4}, {-1, 1, -4}, {1, -1, -4}, {1, 1, 4}});
    const auto square = [](double u, double v) { return u > 0 && u < 1 && v > 0 && v < 1; };
    const normal_sweep::Result crossed = normal_sweep::check(saddle, {.49, .51, .49, .51}, .5, 4096, square);
    MINI_CHECK(crossed.state == normal_sweep::State::Fold);
    MINI_CHECK(square(crossed.witness[0], crossed.witness[1]));
}

MINI_TEST("BRepThicken", "Interval Rational Sweep Derivative Bounds") {
    NurbsSurface plane = NurbsSurface::create(false, false, 1, 1, 2, 2,
        {{0, 0, 0}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}});
    MINI_CHECK(plane.to_rational());
    for (int j = 0; j < 2; ++j)
        for (int d = 0; d < 4; ++d)
            plane.cv(1, j)[d] *= 2;
    MINI_CHECK(plane.weight(0, 0) == 1 && plane.weight(1, 0) == 2);
    const auto inside = [](double u, double v) { return u > 0 && u < 1 && v > 0 && v < 1; };
    MINI_CHECK(normal_sweep::check(plane, {0, 1, 0, 1}, .125, 4096, inside).state
        == normal_sweep::State::Regular);
}


MINI_TEST("BRepThicken", "Analytic Sweep Crossing Versus Raw Offset") {
    for (const NurbsSurface& surface : {Primitives::sphere_surface(0, 0, 0, 1),
            Primitives::cylinder_surface(0, 0, 0, 1, 2)}) {
        MINI_CHECK(nurbs_offset::surface(surface, -1.5, 1e-7).is_valid());
        MINI_CHECK(nurbs_offset::surface_sweep(surface, -.125, 1e-7).is_valid());
        bool rejected = false;
        try {
            const NurbsSurface crossed = nurbs_offset::surface_sweep(surface, -1.5, 1e-7);
            MINI_CHECK(crossed.is_valid());
        } catch (const std::exception& error) {
            rejected = std::string(error.what()).find("focal") != std::string::npos;
            if (!rejected)
                throw;
        }
        MINI_CHECK(rejected);
    }
    bool rejected = false;
    try {
        const BRep crossed = brep_thicken::face(cylinder_face(), -1.5, 1e-7);
        MINI_CHECK(crossed.is_valid());
    } catch (const std::exception& error) {
        rejected = std::string(error.what()).find("focal") != std::string::npos;
        if (!rejected)
            throw;
    }
    MINI_CHECK(rejected);
}

MINI_TEST("BRepThicken", "Piecewise Sweep Fold Does Not Assert Global Continuity") {
    NurbsSurface surface = parabolic_focal_face().m_surfaces[0];
    MINI_CHECK(surface.insert_nurbsknot(0, .5, 2));
    const auto inside = [](double u, double v) { return u > 0 && u < 1 && v > 0 && v < 1; };
    MINI_CHECK(normal_sweep::check(surface, {0, 1, 0, 1}, 1.0 / 64, 4096, inside).state
        == normal_sweep::State::Unknown);
    MINI_CHECK(normal_sweep::check(surface, {0, 1, 0, 1}, 1.0 / 16, 4096, inside).state
        == normal_sweep::State::Fold);
}

MINI_TEST("BRepThicken", "Original Trim Excludes Full Support Focal Zone") {
    BRep source = parabolic_focal_face();
    source.m_faces[0].wires = {{trim_wire(source, {.25, 1, 0, 1}), Forward}};
    const double thickness = 1.0 / 16;
    const std::string name = "native_thicken_trimmed_parabolic_focal_support";
    const BRep result = saved_thickening(source, thickness, 1e-5, name);
    const double area = .5 * std::sqrt(1025.0) - .125 * std::sqrt(65.0)
        + (std::asinh(32.0) - std::asinh(8.0)) / 64;
    const double expected = thickness * area
        - .5 * thickness * thickness * (std::atan(32.0) - std::atan(8.0));
    check_result(result, expected, 1e-5, name);
    for (double u : {.3, .6, .9}) {
        const Point point = source.m_surfaces[0].point_at(u, .5);
        const Vector normal = source.m_surfaces[0].normal_at(u, .5);
        for (double fraction : {-.5, .5, 1.5})
            check_interior_probe(result, point + normal * (thickness * fraction), {u, .5, 0},
                1e-5, fraction == .5 ? brep_classify::State::Inside : brep_classify::State::Outside, name);
    }
    finish_checks(name);
}
