#include "brep_offset_inward.h"
#include "brep_offset.h"
#include "brep_assemble.h"
#include "brep_classify.h"
#include "brep_measure.h"
#include "mini_test.h"
#include "session.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>

namespace session_cpp {
namespace {

const std::filesystem::path Evidence = "build/native-geometry-tests/viewer";

BRep inward_source(bool nonzero_domains) {
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    const std::string name = nonzero_domains ? "native_offset_inward_freeform_nonzero_domains_source.brep.json"
        : "native_offset_inward_freeform_source.brep.json";
    std::ifstream input(root / "tests/fixtures" / name);
    MINI_CHECK(input.good());
    return BRep::jsonload(nlohmann::json::parse(input));
}

void scene(const BRep& source, const std::string& name, const Color& color) {
    std::filesystem::create_directories(Evidence);
    BRep geometry = source;
    geometry.name = name;
    geometry.surfacecolor = color;
    geometry.file_json_dump((Evidence / (name + ".brep.json")).string());
    Session document(name);
    document.add_brep(geometry);
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

double boundary_residual(const BRep& result) {
    double residual = 0.0;
    for (const BRepEdge& edge : result.m_edges) {
        MINI_CHECK(!edge.degenerated && edge.pcurves.size() == 2);
        const NurbsCurve& world = result.m_curves_3d[edge.curve_3d_index];
        for (const BRepCurveOnSurface& relation : edge.pcurves) {
            const NurbsCurve& pcurve = result.m_curves_2d[relation.curve_2d_index];
            const NurbsSurface& surface = result.m_surfaces[relation.surface_index];
            for (int i = 0; i <= 151; ++i) {
                const double fraction = i / 151.0;
                const double t = pcurve.domain_start() + fraction * (pcurve.domain_end() - pcurve.domain_start());
                const double s = world.domain_start() + fraction * (world.domain_end() - world.domain_start());
                const Point uv = pcurve.point_at(t);
                residual = std::max(residual, surface.point_at(uv[0], uv[1]).distance(world.point_at(s)));
            }
        }
    }
    return residual;
}

double normal_residual(const BRep& source, const BRep& result, double distance, double tolerance) {
    double residual = 0.0;
    for (int face = 0; face < source.face_count(); ++face) {
        const NurbsSurface& original = source.m_surfaces[source.m_faces[face].surface_index];
        const NurbsSurface& shifted = result.m_surfaces[result.m_faces[face].surface_index];
        MINI_CHECK(original.domain(0) == shifted.domain(0) && original.domain(1) == shifted.domain(1));
        const auto u = original.domain(0);
        const auto v = original.domain(1);
        const double sign = source.face_orientation(face) == BRepOrientation::Reversed ? -1.0 : 1.0;
        for (int i = 1; i <= 7; ++i)
            for (int j = 1; j <= 7; ++j) {
                const double x = u.first + (u.second - u.first) * i / 8.0;
                const double y = v.first + (v.second - v.first) * j / 8.0;
                const Point expected = original.point_at(x, y) + original.normal_at(x, y) * (sign * distance);
                residual = std::max(residual, expected.distance(shifted.point_at(x, y)));
            }
        const double x = (u.first + u.second) * 0.5;
        const double y = (v.first + v.second) * 0.5;
        const Vector normal = original.normal_at(x, y) * sign;
        const Point expected = original.point_at(x, y) + normal * distance;
        MINI_CHECK(brep_classify::point(result, expected, tolerance * 2) == brep_classify::State::Boundary);
        MINI_CHECK(brep_classify::point(result, expected - normal * (tolerance * 8), tolerance) == brep_classify::State::Inside);
        MINI_CHECK(brep_classify::point(result, expected + normal * (tolerance * 8), tolerance) == brep_classify::State::Outside);
    }
    return residual;
}

double support_area_bound(const BRep& source, double distance) {
    const auto length = [](const Vector& vector) { return std::hypot(vector[0], vector[1], vector[2]); };
    double area = 0.0;
    for (const NurbsSurface& surface : source.m_surfaces) {
        MINI_CHECK(!surface.is_rational() && surface.cv_count(0) == 2 && surface.cv_count(1) == 2);
        const Point origin = surface.get_cv(0, 0);
        const Vector u = surface.get_cv(1, 0) - origin;
        const Vector v = surface.get_cv(0, 1) - origin;
        const Vector mixed = surface.get_cv(1, 1) - surface.get_cv(1, 0) - v;
        const Vector center = (u + mixed * 0.5).cross(v + mixed * 0.5);
        MINI_CHECK(length(center) > 0.0);
        const Vector normal = center * (1.0 / length(center));
        double minimum = center.dot(normal);
        for (double x : {0.0, 1.0})
            for (double y : {0.0, 1.0})
                minimum = std::min(minimum, (u + mixed * y).cross(v + mixed * x).dot(normal));
        MINI_CHECK(minimum > 0.0);
        const double du = std::max(length(u), length(u + mixed)) + std::abs(distance) * length(u.cross(mixed)) / minimum;
        const double dv = std::max(length(v), length(v + mixed)) + std::abs(distance) * length(mixed.cross(v)) / minimum;
        area += du * dv;
    }
    return area;
}

void check_oracle_volume(
    const BRep& source,
    bool nonzero_domains,
    double distance,
    double tolerance,
    nlohmann::ordered_json& report
) {
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_offset_inward_occt_reference.json");
    MINI_CHECK(input.good());
    const nlohmann::json oracle = nlohmann::json::parse(input);
    const std::string variant = nonzero_domains ? "affine" : "standard";
    double reference = -1.0;
    double other = -1.0;
    for (const nlohmann::json& item : oracle.at("cases"))
        if (item.at("distance").get<double>() == distance) {
            MINI_CHECK(item.at("valid").get<bool>() && item.at("done").get<bool>());
            if (item.at("variant").get<std::string>() == variant)
                reference = item.at("volume").get<double>();
            else
                other = item.at("volume").get<double>();
        }
    MINI_CHECK(reference > 0.0 && other > 0.0);
    const double area = support_area_bound(source, distance);
    const double quadrature = (1e-6 + 1e-9) * std::max(1.0, reference);
    const double oracle_error = area * oracle.at("offset_tolerance").get<double>() + std::abs(reference - other);
    const double budget = area * tolerance + quadrature + oracle_error;
    const double difference = std::abs(report.at("volume").get<double>() - reference);
    report["oracle_volume"] = reference;
    report["volume_difference"] = difference;
    report["full_support_area_bound"] = area;
    report["volume_geometry_budget"] = area * tolerance;
    report["volume_quadrature_budget"] = quadrature;
    report["volume_oracle_budget"] = oracle_error;
    report["volume_acceptance_budget"] = budget;
    report["volume_budget_scope"] = "Acceptance budget; geometric offset approximation remains sampled";
    MINI_CHECK(difference <= budget);
}

void inward_case(bool nonzero_domains, double distance) {
    const double tolerance = 3e-5;
    const std::string name = "native_offset_inward_" + std::string(nonzero_domains ? "affine_" : "standard_")
        + (distance > -0.1 ? "small" : "large");
    const BRep source = inward_source(nonzero_domains);
    scene(source, name + "_source", Color(75 / 255.f, 125 / 255.f, 170 / 255.f));
    nlohmann::ordered_json report = {{"passed", false}, {"stage", "native inward offset"},
        {"distance", distance}, {"tolerance", tolerance}, {"source_scene", name + "_source.pb"},
        {"result_scene", name + "_result.pb"}, {"source_volume", 7.8816}};
    const auto save = [&]() { std::ofstream(Evidence / (name + ".assertions.json")) << report.dump(2); };
    save();
    try {
        const BRep result = brep_offset::shape(source, distance, tolerance);
        scene(result, name + "_result", Color(180 / 255.f, 110 / 255.f, 155 / 255.f));
        report["stage"] = "topology and numerical validation";
        report["faces"] = result.face_count();
        report["edges"] = result.edge_count();
        report["vertices"] = result.vertex_count();
        save();
        MINI_CHECK(result.is_solid() && result.solid_count() == 1 && result.shell_count() == 1);
        MINI_CHECK(result.face_count() == 6 && result.edge_count() == 12 && result.vertex_count() == 8);
        report["volume"] = brep_measure::volume(result, 1e-6);
        report["boundary_residual"] = boundary_residual(result);
        save();
        MINI_CHECK(report["volume"].get<double>() > 0 && report["volume"].get<double>() < 7.8816);
        check_oracle_volume(
            source,
            nonzero_domains,
            distance,
            tolerance,
            report
        );
        MINI_CHECK(report["boundary_residual"].get<double>() < tolerance);
        report["normal_residual"] = normal_residual(source, result, distance, tolerance);
        MINI_CHECK(report["normal_residual"].get<double>() < tolerance);
        MINI_CHECK(brep_classify::point(result, Point(), tolerance) == brep_classify::State::Inside);
        MINI_CHECK(brep_classify::point(result, Point(0, 0, 2), tolerance) == brep_classify::State::Outside);
        report["passed"] = true;
        report["stage"] = "complete";
        save();
    } catch (const brep_offset::InwardError& error) {
        save_failure(name, error, tolerance);
        report["failure"] = error.what();
        report["stage"] = error.stage;
        save();
        throw;
    } catch (const brep_assemble::ParameterError& error) {
        save_failure(name, error, tolerance);
        report["failure"] = error.what();
        save();
        throw;
    } catch (const std::exception& error) {
        report["failure"] = error.what();
        save();
        throw;
    }
}

}

MINI_TEST("BRepOffsetInward", "Six Freeform Supports Small Inward Join") {
    inward_case(false, -0.04);
}

MINI_TEST("BRepOffsetInward", "Six Freeform Supports Larger Inward Join") {
    inward_case(false, -0.16);
}

MINI_TEST("BRepOffsetInward", "Affine Domains Small Inward Join") {
    inward_case(true, -0.04);
}

MINI_TEST("BRepOffsetInward", "Affine Domains Larger Inward Join") {
    inward_case(true, -0.16);
}

}
