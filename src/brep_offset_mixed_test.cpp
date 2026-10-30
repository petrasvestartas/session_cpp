#include "brep_offset.h"
#include "brep_offset_inward.h"
#include "brep_assemble.h"
#include "brep_classify.h"
#include "brep_measure.h"
#include "curve_surface_intersection.h"
#include "mini_test.h"
#include "session.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>

namespace session_cpp {
namespace {

constexpr BRepOrientation Forward = BRepOrientation::Forward;
constexpr BRepOrientation Reversed = BRepOrientation::Reversed;
const std::filesystem::path Evidence = "build/native-geometry-tests/viewer";
const std::array<Point, 6> Profile = {
    Point(0, 0, 0), Point(2, 0, 0), Point(2, 1, 0),
    Point(1, 1, 0), Point(1, 2, 0), Point(0, 2, 0)
};

struct Fixture {
    BRep brep;
    std::map<std::pair<int, int>, int> edges;
    std::vector<BRepRef> faces;
    double warp = 0.0;
    int variant = 0;
};

Point deformed(const Point& point, double warp) {
    return {point[0], point[1], point[2] + warp * point[0] * point[1]};
}

Point parameter(const Point& point, int face, int variant) {
    Point result = point;
    if (variant == 2)
        result[0] = 1.0 - result[0];
    if (variant > 0) {
        result[0] = 3.0 + face + 4.0 * result[0];
        result[1] = -2.0 - face * 0.5 + (6.0 + face * 0.75) * result[1];
    }
    return result;
}

void append_face(
    Fixture& fixture,
    std::vector<Point> controls,
    const std::vector<int>& vertices,
    const std::vector<Point>& coordinates,
    BRepOrientation orientation
) {
    for (Point& point : controls)
        point = deformed(point, fixture.warp);
    NurbsSurface surface = NurbsSurface::create(false, false, 1, 1, 2, 2, controls);
    const int face = static_cast<int>(fixture.brep.m_surfaces.size());
    if (fixture.variant == 2)
        MINI_CHECK(surface.reverse(0));
    const Point lo = parameter({fixture.variant == 2 ? 1.0 : 0.0, 0, 0}, face, fixture.variant);
    const Point hi = parameter({fixture.variant == 2 ? 0.0 : 1.0, 1, 0}, face, fixture.variant);
    MINI_CHECK(surface.set_domain(0, lo[0], hi[0]) && surface.set_domain(1, lo[1], hi[1]));
    const int support = fixture.brep.add_surface(surface);
    std::vector<BRepRef> uses;
    for (size_t i = 0; i < vertices.size(); ++i) {
        const size_t next = (i + 1) % vertices.size();
        const int a = vertices[i];
        const int b = vertices[next];
        const std::pair<int, int> key = std::minmax(a, b);
        if (fixture.edges.find(key) == fixture.edges.end()) {
            const Point first = fixture.brep.m_vertices[a].point;
            const Point last = fixture.brep.m_vertices[b].point;
            const NurbsCurve world = NurbsCurve::create(false, 1, {first, last});
            fixture.edges[key] = fixture.brep.add_edge(fixture.brep.add_curve_3d(world), a, b);
        }
        const int edge = fixture.edges.at(key);
        const bool forward = fixture.brep.m_edges[edge].start_vertex == a;
        Point first = parameter(coordinates[i], face, fixture.variant);
        Point last = parameter(coordinates[next], face, fixture.variant);
        if (!forward)
            std::swap(first, last);
        const NurbsCurve pcurve = NurbsCurve::create(false, 1, {first, last});
        fixture.brep.add_pcurve(edge, support, fixture.brep.add_curve_2d(pcurve));
        uses.push_back({edge, forward ? Forward : Reversed});
    }
    const BRepOrientation wire = fixture.variant == 2 ? Reversed : Forward;
    const int index = fixture.brep.add_face(support, {{fixture.brep.add_wire(uses), wire}});
    fixture.faces.push_back({index, fixture.variant == 2 ? brep_reverse(orientation) : orientation});
}

BRep source_prism(double warp, int variant) {
    Fixture fixture;
    fixture.warp = warp;
    fixture.variant = variant;
    for (double height : {0.0, 2.0})
        for (const Point& point : Profile)
            fixture.brep.add_vertex(deformed({point[0], point[1], height}, warp));
    for (int cap = 0; cap < 2; ++cap) {
        const double height = 2.0 * cap;
        std::vector<int> vertices;
        std::vector<Point> uv;
        for (int i = 0; i < 6; ++i) {
            vertices.push_back(i + cap * 6);
            uv.push_back({Profile[i][0] * 0.5, Profile[i][1] * 0.5, 0});
        }
        append_face(
            fixture,
            {{0, 0, height}, {0, 2, height}, {2, 0, height}, {2, 2, height}},
            vertices,
            uv,
            cap == 0 ? Reversed : Forward
        );
    }
    for (int i = 0; i < 6; ++i) {
        const int next = (i + 1) % 6;
        const Point a = Profile[i];
        const Point b = Profile[next];
        append_face(
            fixture,
            {a, {a[0], a[1], 2}, b, {b[0], b[1], 2}},
            {i, next, next + 6, i + 6},
            {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}},
            Forward
        );
    }
    fixture.brep.add_solid({{fixture.brep.add_shell(fixture.faces), Forward}});
    return fixture.brep;
}

std::string source_name(double warp, int variant) {
    const std::array<std::string, 3> variants = {"standard", "affine", "affine_reversed"};
    return "native_offset_l_prism_" + std::string(warp == 0.0 ? "planar_" : "freeform_") + variants[variant];
}

void scene(const BRep& shape, const std::string& name, const Color& color) {
    std::filesystem::create_directories(Evidence);
    BRep result = shape;
    result.name = name;
    result.surfacecolor = color;
    result.file_json_dump((Evidence / (name + ".brep.json")).string());
    Session document(name);
    document.add_brep(result);
    document.pb_dump((Evidence / (name + ".pb")).string());
}

void save_failure(const std::string& name, const brep_offset::InwardError& error, double tolerance) {
    nlohmann::ordered_json fixture = {{"failure", error.what()}, {"stage", error.stage},
        {"tolerance", tolerance}, {"source_edge", error.source_edge}, {"source_vertex", error.source_vertex},
        {"source_face", error.source_face}, {"candidate_faces", error.candidate_faces},
        {"source_faces", error.source_faces}, {"section_edges", error.section_edges},
        {"surfaces", nlohmann::ordered_json::array()}, {"sections", nlohmann::ordered_json::array()}};
    for (const NurbsSurface& surface : error.surfaces)
        fixture["surfaces"].push_back(surface.jsondump());
    for (size_t i = 0; i < error.curves.size(); ++i)
        fixture["sections"].push_back({{"world", error.curves[i].jsondump()},
            {"first_pcurve", error.first_pcurves[i].jsondump()}, {"second_pcurve", error.second_pcurves[i].jsondump()}});
    if (error.first_support.face_count() > 0) {
        fixture["first_support"] = error.first_support.jsondump();
        fixture["second_support"] = error.second_support.jsondump();
        fixture["section_complete"] = error.section_result.complete;
        fixture["section_failure"] = error.section_result.failure;
        fixture["candidate_sections"] = nlohmann::ordered_json::array();
        fixture["overlaps"] = nlohmann::ordered_json::array();
        for (const brep_section::FaceCurve& curve : error.section_result.curves)
            fixture["candidate_sections"].push_back({{"first_face", curve.first_face}, {"second_face", curve.second_face},
                {"world", curve.world.jsondump()}, {"first_pcurve", curve.first_pcurve.jsondump()},
                {"second_pcurve", curve.second_pcurve.jsondump()}});
        for (const brep_section::FaceOverlap& overlap : error.section_result.overlaps)
            fixture["overlaps"].push_back({{"first_face", overlap.first_face}, {"second_face", overlap.second_face}});
    }
    if (error.support_index >= 0) {
        fixture["support_index"] = error.support_index;
        fixture["arrangement_tolerance"] = error.arrangement_tolerance;
        fixture["arrangement_support"] = error.arrangement_support.jsondump();
        fixture["arrangement_cuts"] = nlohmann::ordered_json::array();
        for (const split::FaceCut& cut : error.arrangement_cuts)
            fixture["arrangement_cuts"].push_back({{"section", cut.section},
                {"world", cut.world.jsondump()}, {"pcurve", cut.pcurve.jsondump()}});
    }
    if (!error.assembly_parts.empty()) {
        fixture["assembly_inputs"] = nlohmann::ordered_json::array();
        for (size_t i = 0; i < error.assembly_parts.size(); ++i) {
            const split::FaceSplit& part = error.assembly_parts[i];
            nlohmann::ordered_json pieces = nlohmann::ordered_json::array();
            for (const split::EdgePiece& piece : part.edges)
                pieces.push_back({{"source_edge", piece.source_edge}, {"section", piece.section},
                    {"edge", piece.edge}, {"first", piece.first}, {"last", piece.last}});
            fixture["assembly_inputs"].push_back({{"brep", part.brep.jsondump()},
                {"face", error.assembly_faces.at(i)}, {"reversed", false}, {"operand", -1}, {"pieces", pieces}});
        }
    }
    if (error.partition.face_count() > 0) {
        fixture["partition"] = error.partition.jsondump();
        fixture["pieces"] = nlohmann::ordered_json::array();
        for (const split::EdgePiece& piece : error.pieces)
            fixture["pieces"].push_back({{"source_edge", piece.source_edge}, {"section", piece.section},
                {"edge", piece.edge}, {"first", piece.first}, {"last", piece.last}});
    }
    std::ofstream(Evidence / (name + ".failure.json")) << fixture.dump(2);
}

void save_failure(const std::string& name, const brep_assemble::ParameterError& error, double tolerance) {
    const nlohmann::ordered_json fixture = {{"failure", error.what()}, {"tolerance", tolerance},
        {"certificate_tolerance", error.tolerance}, {"match", static_cast<int>(error.match)},
        {"surface", error.surface.jsondump()}, {"world", error.curve.jsondump()}, {"pcurve", error.pcurve.jsondump()}};
    std::ofstream(Evidence / (name + ".parameter_failure.json")) << fixture.dump(2);
}

void check_boundaries(const BRep& shape, double tolerance) {
    for (const BRepEdge& edge : shape.m_edges) {
        MINI_CHECK(!edge.degenerated && edge.pcurves.size() == 2);
        const NurbsCurve& world = shape.m_curves_3d[edge.curve_3d_index];
        for (const BRepCurveOnSurface& relation : edge.pcurves) {
            const NurbsCurve& pcurve = shape.m_curves_2d[relation.curve_2d_index];
            const NurbsSurface& surface = shape.m_surfaces[relation.surface_index];
            MINI_CHECK(curve_surface_intersection::same_parameter(world, pcurve, surface, tolerance)
                == curve_surface_intersection::ParameterMatch::Matched);
        }
    }
}

void probe(const BRep& shape, const Point& point, const Vector& normal, double tolerance) {
    MINI_CHECK(brep_classify::point(shape, point, tolerance * 2) == brep_classify::State::Boundary);
    MINI_CHECK(brep_classify::point(shape, point - normal * (tolerance * 8), tolerance) == brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(shape, point + normal * (tolerance * 8), tolerance) == brep_classify::State::Outside);
}

void check_geometry(const BRep& result, double warp, double radius, double tolerance) {
    for (int cap = 0; cap < 2; ++cap)
        for (const Point& xy : {Point(0.5, 0.5, 0), Point(1.5, 0.5, 0), Point(0.5, 1.5, 0)}) {
            Vector normal(-warp * xy[1], -warp * xy[0], 1);
            normal *= (cap == 0 ? -1.0 : 1.0) / std::hypot(normal[0], normal[1], normal[2]);
            const Point source = deformed({xy[0], xy[1], 2.0 * cap}, warp);
            probe(result, source - normal * radius, normal, tolerance);
        }
    for (int i = 1; i < 8; ++i) {
        const double angle = std::acos(-1.0) * i / 16.0;
        const Vector normal(std::cos(angle), std::sin(angle), 0);
        const Point point(1 - radius * normal[0], 1 - radius * normal[1], 1 + warp);
        probe(result, point, normal, tolerance);
    }
    MINI_CHECK(brep_classify::point(result, deformed({0.5, 0.5, 1}, warp), tolerance) == brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(result, deformed({1.5, 1.5, 1}, warp), tolerance) == brep_classify::State::Outside);
}

double expected_volume(double radius) {
    return (2 - 2 * radius) * (3 - 8 * radius + (5 - std::acos(-1.0) / 4) * radius * radius);
}

void check_reference(const std::string& name, double radius, nlohmann::ordered_json& report) {
    std::ifstream input(Evidence / "occt_inward_l_prism_reference.json");
    if (!input.good())
        return;
    const nlohmann::json reference = nlohmann::json::parse(input);
    bool found = false;
    for (const nlohmann::json& item : reference.at("cases"))
        if (item.at("source_name") == name && item.at("radius").get<double>() == radius) {
            MINI_CHECK(item.at("valid").get<bool>() && item.at("probes_passed").get<bool>());
            const double volume = item.at("volume").get<double>();
            report["oracle_volume"] = volume;
            report["volume_difference"] = std::abs(report.at("volume").get<double>() - volume);
            const double area = item.at("area").get<double>();
            const double geometry = area * report.at("tolerance").get<double>();
            const double numerical = (1e-7 + 1e-9) * std::max(1.0, volume);
            const double oracle_error = area * reference.at("offset_tolerance").get<double>();
            report["volume_acceptance_budget"] = geometry + numerical + oracle_error;
            report["volume_budget_scope"] = "Area-scaled acceptance budget; sampled geometric offset approximation";
            MINI_CHECK(report.at("volume_difference").get<double>() <= geometry + numerical + oracle_error);
            found = true;
        }
    MINI_CHECK(found);
}

void mixed_case(double warp, int variant, double radius) {
    const double tolerance = 3e-5;
    const BRep source = source_prism(warp, variant);
    const std::string input = source_name(warp, variant);
    const std::string name = input + (radius < 0.15 ? "_small" : "_large");
    scene(source, input + "_source", Color(75 / 255.f, 125 / 255.f, 170 / 255.f));
    nlohmann::ordered_json report = {{"passed", false}, {"stage", "native mixed inward offset"},
        {"source_scene", input + "_source.pb"}, {"result_scene", name + "_result.pb"},
        {"warp", warp}, {"variant", variant}, {"distance", -radius}, {"tolerance", tolerance}};
    const auto save = [&]() { std::ofstream(Evidence / (name + ".assertions.json")) << report.dump(2); };
    save();
    try {
        const BRep result = brep_offset::shape(source, -radius, tolerance);
        scene(result, name + "_result", Color(180 / 255.f, 110 / 255.f, 155 / 255.f));
        report["stage"] = "topology and numerical validation";
        report["faces"] = result.face_count();
        report["edges"] = result.edge_count();
        report["vertices"] = result.vertex_count();
        save();
        MINI_CHECK(result.is_valid() && result.is_solid() && result.solid_count() == 1 && result.shell_count() == 1);
        MINI_CHECK(result.face_count() == 9 && result.edge_count() == 21 && result.vertex_count() == 14);
        check_boundaries(result, tolerance);
        report["volume"] = brep_measure::volume(result, 1e-7);
        if (warp == 0.0) {
            report["analytic_volume"] = expected_volume(radius);
            MINI_CHECK(std::abs(report.at("volume").get<double>() - expected_volume(radius)) < 22 * tolerance + 1e-6);
        }
        check_reference(input, radius, report);
        check_geometry(result, warp, radius, tolerance);
        report["stage"] = "complete";
        report["passed"] = true;
        save();
    } catch (const brep_offset::InwardError& error) {
        save_failure(name, error, tolerance);
        report["failure"] = error.what();
        report["failure_fixture"] = name + ".failure.json";
        save();
        throw;
    } catch (const brep_assemble::ParameterError& error) {
        save_failure(name, error, tolerance);
        report["failure"] = error.what();
        report["failure_fixture"] = name + ".parameter_failure.json";
        save();
        throw;
    } catch (const std::exception& error) {
        report["failure"] = error.what();
        save();
        throw;
    }
}

}

MINI_TEST("BRepOffsetMixed", "L Prism Source Fixtures") {
    for (double warp : {0.0, 0.16})
        for (int variant = 0; variant < 3; ++variant) {
            const BRep source = source_prism(warp, variant);
            scene(source, source_name(warp, variant) + "_source", Color(75 / 255.f, 125 / 255.f, 170 / 255.f));
            MINI_CHECK(source.is_valid() && source.is_solid());
            MINI_CHECK(source.face_count() == 8 && source.edge_count() == 18 && source.vertex_count() == 12);
            MINI_CHECK(std::abs(brep_measure::volume(source, 1e-9) - 6.0) < 1e-8);
            check_boundaries(source, 1e-9);
        }
}

MINI_TEST("BRepOffsetMixed", "L Prism Small Inward Join") {
    mixed_case(0.0, 0, 0.1);
}

MINI_TEST("BRepOffsetMixed", "L Prism Larger Inward Join") {
    mixed_case(0.0, 0, 0.2);
}

MINI_TEST("BRepOffsetMixed", "L Prism Affine Reversed Small Inward Join") {
    mixed_case(0.0, 2, 0.1);
}

MINI_TEST("BRepOffsetMixed", "L Prism Affine Reversed Larger Inward Join") {
    mixed_case(0.0, 2, 0.2);
}

MINI_TEST("BRepOffsetMixed", "L Prism Freeform Caps Small Inward Join") {
    mixed_case(0.16, 0, 0.1);
}

MINI_TEST("BRepOffsetMixed", "L Prism Freeform Caps Larger Inward Join") {
    mixed_case(0.16, 0, 0.2);
}

MINI_TEST("BRepOffsetMixed", "L Prism Freeform Caps Affine Reversed Small Inward Join") {
    mixed_case(0.16, 2, 0.1);
}

MINI_TEST("BRepOffsetMixed", "L Prism Freeform Caps Affine Reversed Larger Inward Join") {
    mixed_case(0.16, 2, 0.2);
}

}
