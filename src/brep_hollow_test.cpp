#include "brep.h"
#include <cstdlib>
#include "brep_hollow_affine.h"
#include "brep_hollow_source_extension.h"
#include "closest.h"
#include "brep_offset_inward.h"
#include <filesystem>
#include <chrono>
#include <fstream>
#include <iostream>
#include "brep_acceptance.h"
#include "brep_classify.h"
#include "brep_measure.h"
#include "curve_surface_intersection.h"
#include "mini_test.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <stdexcept>
using namespace session_cpp::mini_test;
namespace session_cpp {
namespace {
nlohmann::ordered_json hollow_failure(const brep_offset::InwardError& error, double tolerance) {
    nlohmann::ordered_json fixture = {{"failure", error.what()}, {"stage", error.stage},
        {"tolerance", tolerance}, {"source_edge", error.source_edge}, {"source_vertex", error.source_vertex},
        {"source_face", error.source_face}, {"candidate_faces", error.candidate_faces},
        {"source_faces", error.source_faces}, {"section_edges", error.section_edges},
        {"surfaces", nlohmann::ordered_json::array()}, {"sections", nlohmann::ordered_json::array()}};
    for (const NurbsSurface& surface : error.surfaces)
        fixture["surfaces"].push_back(surface.jsondump());
    for (size_t i = 0; i < error.curves.size(); ++i)
        fixture["sections"].push_back({{"world", error.curves[i].jsondump()},
            {"first_pcurve", error.first_pcurves.at(i).jsondump()}, {"second_pcurve", error.second_pcurves.at(i).jsondump()}});
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
    return fixture;
}


BRep captured_hollow(const BRep& source, const std::vector<int>& openings, double thickness,
    const std::string& label) {
    try {
        BRep result = source.hollow(openings, thickness);
        if (const char* path = std::getenv("SESSION_HOLLOW_RESULT_OBSERVATION"); path && *path) {
            const nlohmann::ordered_json witness = {{"label", label}, {"source", source.jsondump()},
                {"result", result.jsondump()}, {"openings", openings}, {"thickness", thickness},
                {"tolerance", 1e-7}};
            const std::string bytes = witness.dump() + "\n";
            const std::filesystem::path folder(path);
            std::filesystem::create_directories(folder);
            const auto stamp = std::chrono::system_clock::now().time_since_epoch().count();
            const auto capture = folder / (label + "-" + std::to_string(stamp));
            if (!std::filesystem::create_directory(capture))
                throw std::runtime_error("Hollow result observation already exists");
            std::ofstream output(capture / "result.json", std::ios::binary);
            if (!output || !output.write(bytes.data(), static_cast<std::streamsize>(bytes.size())))
                throw std::runtime_error("Cannot write hollow result observation");
            output.flush();
            if (!output)
                throw std::runtime_error("Cannot flush hollow result observation");
            output.close();
            if (!output)
                throw std::runtime_error("Cannot close hollow result observation");
        }
        return result;
    } catch (const brep_offset::InwardError& error) {
        try {
            nlohmann::ordered_json fixture = hollow_failure(error, 1e-7);
            fixture["source"] = source.jsondump();
            fixture["openings"] = openings;
            fixture["thickness"] = thickness;
            const std::filesystem::path folder = std::filesystem::path("build/native-geometry-tests/hollow-failures") / label;
            std::filesystem::create_directories(folder);
            const std::chrono::system_clock::duration stamp = std::chrono::system_clock::now().time_since_epoch();
            const std::filesystem::path capture = folder / std::to_string(stamp.count());
            if (!std::filesystem::create_directory(capture))
                throw std::runtime_error("Hollow failure capture already exists");
            std::ofstream output(capture / "failure.json", std::ios::binary);
            const std::string bytes = fixture.dump(2);
            if (!output || !output.write(bytes.data(), static_cast<std::streamsize>(bytes.size())))
                throw std::runtime_error("Cannot write hollow failure capture");
            output.flush();
            if (!output)
                throw std::runtime_error("Cannot flush hollow failure capture");
            output.close();
            if (!output)
                throw std::runtime_error("Cannot close hollow failure capture");
        } catch (const std::exception& capture_error) {
            std::cerr << "Hollow failure capture: " << capture_error.what() << '\n';
        }
        throw;
    }
}

void hollow_boundary(const BRep& shape) {
    MINI_CHECK(shape.is_valid() && shape.is_solid() && shape.solid_count() == 1);
    for (const BRepShell& shell : shape.m_shells)
        MINI_CHECK(brep_acceptance::shell_vertex_links(shape, shell) == brep_acceptance::VertexLinks::Matched);
    for (const BRepEdge& edge : shape.m_edges)
        for (const BRepCurveOnSurface& relation : edge.pcurves)
            for (int pc : {relation.curve_2d_index, relation.curve_2d_index_2})
                if (pc >= 0)
                    MINI_CHECK(curve_surface_intersection::same_parameter(shape.m_curves_3d[edge.curve_3d_index],
                        shape.m_curves_2d[pc], shape.m_surfaces[relation.surface_index], 1e-7)
                        == curve_surface_intersection::ParameterMatch::Matched);
}
BRep hollow_warped_box() {
    BRep result = BRep::create_box(2, 2, 2);
    const auto warp = [](const Point& point) {
        return Point(point[0], point[1], point[2] + .125 * point[0] * point[1]);
    };
    for (BRepVertex& vertex : result.m_vertices)
        vertex.point = warp(vertex.point);
    for (NurbsCurve& curve : result.m_curves_3d)
        for (int i = 0; i < curve.cv_count(); ++i)
            MINI_CHECK(curve.set_cv(i, warp(curve.get_cv(i))));
    for (NurbsSurface& surface : result.m_surfaces)
        for (int i = 0; i < surface.cv_count(0); ++i)
            for (int j = 0; j < surface.cv_count(1); ++j)
                MINI_CHECK(surface.set_cv(i, j, warp(surface.get_cv(i, j))));
    hollow_boundary(result);
    return result;
}
BRep hollow_concave_prism() {
    const std::vector<Point> profile = {{0, 0, 0}, {2, 0, 0}, {2, 1, 0},
        {1, 1, 0}, {1, 2, 0}, {0, 2, 0}};
    BRep result;
    for (double z : {0.0, 2.0})
        for (const Point& point : profile)
            result.add_vertex({point[0], point[1], z});
    std::map<std::pair<int, int>, int> edges;
    std::vector<BRepRef> faces;
    const auto append = [&](const std::vector<Point>& controls, const std::vector<int>& vertices,
        const std::vector<Point>& uv, BRepOrientation orientation) {
        const int support = result.add_surface(NurbsSurface::create(false, false, 1, 1, 2, 2, controls));
        std::vector<BRepRef> uses;
        for (size_t i = 0; i < vertices.size(); ++i) {
            const size_t next = (i + 1) % vertices.size();
            const int first = vertices[i], last = vertices[next];
            const std::pair<int, int> key = std::minmax(first, last);
            if (!edges.count(key)) {
                const NurbsCurve curve = NurbsCurve::create(false, 1,
                    {result.m_vertices[first].point, result.m_vertices[last].point});
                edges[key] = result.add_edge(result.add_curve_3d(curve), first, last);
            }
            const int edge = edges.at(key);
            const bool forward = result.m_edges[edge].start_vertex == first;
            const NurbsCurve pc = NurbsCurve::create(false, 1,
                forward ? std::vector<Point>{uv[i], uv[next]} : std::vector<Point>{uv[next], uv[i]});
            result.add_pcurve(edge, support, result.add_curve_2d(pc));
            uses.push_back({edge, forward ? BRepOrientation::Forward : BRepOrientation::Reversed});
        }
        const int face = result.add_face(support, {{result.add_wire(uses), BRepOrientation::Forward}});
        faces.push_back({face, orientation});
    };
    for (int cap = 0; cap < 2; ++cap) {
        const double z = 2.0 * cap;
        std::vector<int> vertices;
        std::vector<Point> uv;
        for (int i = 0; i < 6; ++i) {
            vertices.push_back(i + 6 * cap);
            uv.push_back({profile[i][0] * .5, profile[i][1] * .5, 0});
        }
        append({{0, 0, z}, {0, 2, z}, {2, 0, z}, {2, 2, z}}, vertices, uv,
            cap ? BRepOrientation::Forward : BRepOrientation::Reversed);
    }
    for (int i = 0; i < 6; ++i) {
        const int next = (i + 1) % 6;
        const Point& a = profile[i];
        const Point& b = profile[next];
        append({a, {a[0], a[1], 2}, b, {b[0], b[1], 2}}, {i, next, next + 6, i + 6},
            {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}}, BRepOrientation::Forward);
    }
    result.add_solid({{result.add_shell(faces), BRepOrientation::Forward}});
    return result;
}

}
MINI_TEST("BRep Native Hollow", "Original Solid With Opening Contract") {
    const BRep box = BRep::create_box(2, 2, 2);
    const BRep hollow = captured_hollow(box, {0}, -.2, "original-box");
    hollow_boundary(hollow);
    MINI_CHECK(hollow.shell_count() == 1 && hollow.face_count() == 11);
    MINI_CHECK(std::abs(hollow.volume_exact() - (8 - 1.6 * 1.6 * 1.8)) < 1e-5);
    MINI_CHECK(brep_classify::point(hollow, {0, 0, 0}, 1e-7) == brep_classify::State::Outside);
    int rims = 0;
    for (const BRepFace& face : hollow.m_faces)
        rims += face.wires.size() == 2;
    MINI_CHECK(rims == 1);
}
MINI_TEST("BRep Native Hollow", "Two Curved Opening Rims") {
    const BRep source = hollow_warped_box();
    std::vector<int> openings;
    for (int face = 0; face < source.face_count(); ++face)
        if (!source.m_surfaces[source.m_faces[face].surface_index].is_planar())
            openings.push_back(face);
    MINI_CHECK(openings.size() == 2);
    const BRep hollow = captured_hollow(source, openings, -.2, "curved-two-mouths");
    hollow_boundary(hollow);
    MINI_CHECK(hollow.shell_count() == 1);
    MINI_CHECK(std::abs(hollow.volume_exact() - (8 - 1.6 * 1.6 * 2)) < 1e-5);
    int curved_rims = 0;
    for (const BRepFace& face : hollow.m_faces)
        curved_rims += face.wires.size() == 2 && !hollow.m_surfaces[face.surface_index].is_planar();
    MINI_CHECK(curved_rims == 2);
}
MINI_TEST("BRep Native Hollow", "Curved Retained Walls") {
    const BRep source = hollow_warped_box();
    int opening = -1;
    for (int face = 0; face < source.face_count(); ++face)
        if (source.m_surfaces[source.m_faces[face].surface_index].is_planar()) {
            opening = face;
            break;
        }
    MINI_CHECK(opening >= 0);
    const BRep hollow = captured_hollow(source, {opening}, -.1, "curved-retained-walls");
    hollow_boundary(hollow);
    MINI_CHECK(hollow.shell_count() == 1);
    const double volume = hollow.volume_exact();
    MINI_CHECK(volume > 0 && volume < 8);
    int curved = 0;
    for (const NurbsSurface& surface : hollow.m_surfaces)
        curved += !surface.is_planar();
    MINI_CHECK(curved >= 4);
}
MINI_TEST("BRep Native Hollow", "No Closing Faces And Original Argument Validation") {
    const BRep source = BRep::create_box(2, 2, 2);
    const BRep cavity = source.hollow({}, -.2);
    hollow_boundary(cavity);
    MINI_CHECK(cavity.shell_count() == 1);
    MINI_CHECK(std::abs(cavity.volume_exact() - (1.6 * 1.6 * 1.6)) < 1e-5);
    bool invalid_index = false;
    try { source.hollow({source.face_count()}, -.2); }
    catch (const std::out_of_range&) { invalid_index = true; }
    MINI_CHECK(invalid_index);
    for (double value : {0.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        bool invalid_thickness = false;
        try { source.hollow({0}, value); }
        catch (const std::invalid_argument&) { invalid_thickness = true; }
        MINI_CHECK(invalid_thickness);
    }
}
MINI_TEST("BRep Native Hollow", "Affine Opening Chart Keeps Physical Mouth") {
    BRep source = BRep::create_box(2, 2, 2);
    for (NurbsSurface& surface : source.m_surfaces) {
        const std::pair<double, double> u = surface.domain(0), v = surface.domain(1);
        MINI_CHECK(surface.set_domain(0, 1024 + 8 * u.first, 1024 + 8 * u.second));
        MINI_CHECK(surface.set_domain(1, -32 + .125 * v.first, -32 + .125 * v.second));
    }
    for (NurbsCurve& pc : source.m_curves_2d)
        for (int i = 0; i < pc.cv_count(); ++i) {
            pc.cv(i)[0] = 1024 * pc.weight(i) + 8 * pc.cv(i)[0];
            pc.cv(i)[1] = -32 * pc.weight(i) + .125 * pc.cv(i)[1];
        }
    hollow_boundary(source);
    const BRep hollow = captured_hollow(source, {0}, -.2, "affine-opening-chart");
    hollow_boundary(hollow);
    MINI_CHECK(std::abs(hollow.volume_exact() - 3.392) < 1e-5);
}
MINI_TEST("BRep Native Hollow", "Two Dimensional Rational And Polynomial Mouth Charts") {
    for (bool rational : {false, true}) {
        BRep source = BRep::create_box(2, 2, 2);
        for (NurbsCurve& original : source.m_curves_2d) {
            NurbsCurve pc(2, rational, original.degree() + 1, original.cv_count());
            pc.m_nurbsknot = original.m_nurbsknot;
            for (int i = 0; i < pc.cv_count(); ++i) {
                const Point point = original.get_cv(i);
                const double weight = rational ? 2.0 : 1.0;
                pc.cv(i)[0] = point[0] * weight;
                pc.cv(i)[1] = point[1] * weight;
                if (rational)
                    pc.cv(i)[2] = weight;
            }
            MINI_CHECK(pc.dimension() == 2 && pc.is_rational() == rational);
            original = std::move(pc);
        }
        hollow_boundary(source);
        const BRep result = captured_hollow(source, {0}, -.2, rational ? "dimension2-rational" : "dimension2-polynomial");
        hollow_boundary(result);
        MINI_CHECK(std::abs(result.volume_exact() - 3.392) < 1e-5);
    }
}


MINI_TEST("BRep Native Hollow", "Affine Pcurve Requires Constant Weight Parameter Law") {
    NurbsCurve curve(2, true, 2, 2);
    curve.m_nurbsknot = {0, 1};
    curve.cv(0)[0] = 0;
    curve.cv(0)[1] = 0;
    curve.cv(0)[2] = 2;
    curve.cv(1)[0] = 2;
    curve.cv(1)[1] = 0;
    curve.cv(1)[2] = 2;
    MINI_CHECK(curve.is_valid());
    MINI_CHECK(curve.point_at(.5).distance({.5, 0, 0}) < 1e-15);
    MINI_CHECK(brep_hollow::affine_working::straight_pcurve(curve));
    curve.cv(1)[0] = 3;
    curve.cv(1)[2] = 3;
    MINI_CHECK(curve.is_valid());
    MINI_CHECK(curve.point_at(.5).distance({.6, 0, 0}) < 1e-15);
    MINI_CHECK(!brep_hollow::affine_working::straight_pcurve(curve));
}

MINI_TEST("BRep Native Hollow", "Curved Walls Affine Two Dimensional Rational Mouth") {
    const double tolerance = 1e-7;
    const double thickness = 0.1;
    BRep source = hollow_warped_box();
    int opening = -1;
    for (int face = 0; face < source.face_count(); ++face) {
        const auto& surface = source.m_surfaces[source.m_faces[face].surface_index];
        bool plane = true;
        for (int i = 0; i < surface.cv_count(0); ++i)
            for (int j = 0; j < surface.cv_count(1); ++j)
                plane = plane && surface.get_cv(i, j)[0] == -1;
        if (plane)
            opening = face;
    }
    MINI_CHECK(opening >= 0);
    if (opening < 0)
        return;
    for (auto& surface : source.m_surfaces) {
        const auto u = surface.domain(0), v = surface.domain(1);
        MINI_CHECK(surface.set_domain(0, 1024 + 8 * u.first, 1024 + 8 * u.second));
        MINI_CHECK(surface.set_domain(1, -32 + 0.125 * v.first, -32 + 0.125 * v.second));
    }
    for (auto& original : source.m_curves_2d) {
        NurbsCurve curve(2, true, original.degree() + 1, original.cv_count());
        curve.m_nurbsknot = original.m_nurbsknot;
        for (int i = 0; i < curve.cv_count(); ++i) {
            const Point point = original.get_cv(i);
            MINI_CHECK(curve.set_cv_4d(i, 2 * (1024 + 8 * point[0]),
                2 * (-32 + 0.125 * point[1]), 0, 2));
        }
        original = std::move(curve);
    }
    hollow_boundary(source);
    const auto original = source.jsondump();
    const BRep result = captured_hollow(source, {opening}, -thickness, "curved-affine-rational-mouth");
    MINI_CHECK(source.jsondump() == original);
    hollow_boundary(result);
    MINI_CHECK(result.shell_count() == 1);
    const double volume = result.volume_exact();
    MINI_CHECK(volume > 0 && volume < 8);
    int rim = -1, rims = 0;
    for (int face = 0; face < result.face_count(); ++face) {
        if (result.m_faces[face].wires.size() != 2)
            continue;
        const auto& surface = result.m_surfaces[result.m_faces[face].surface_index];
        bool plane = true;
        for (int i = 0; i < surface.cv_count(0); ++i)
            for (int j = 0; j < surface.cv_count(1); ++j)
                plane = plane && std::abs(surface.get_cv(i, j)[0] + 1) <= tolerance;
        if (plane) {
            rim = face;
            ++rims;
        }
    }
    MINI_CHECK(rims == 1);
    if (rim < 0)
        return;
    const auto wire_distance = [&](const BRepRef& wire, const Point& point) {
        double distance = std::numeric_limits<double>::infinity();
        for (const auto& use : result.wire_edges(wire)) {
            const auto& edge = result.m_edges[use.index];
            MINI_CHECK(!edge.degenerated && edge.curve_3d_index >= 0);
            if (!edge.degenerated && edge.curve_3d_index >= 0)
                distance = std::min(distance, Closest::curve_point(result.m_curves_3d[edge.curve_3d_index], point).second);
        }
        return distance;
    };
    const auto& outer = result.m_faces[rim].wires[0];
    const auto& inner = result.m_faces[rim].wires[1];
    for (const auto& use : source.wire_edges(source.m_faces[opening].wires[0])) {
        const auto& curve = source.m_curves_3d[source.m_edges[use.index].curve_3d_index];
        for (double fraction : {0.0, 0.5, 1.0})
            MINI_CHECK(wire_distance(outer, curve.point_at(curve.domain_start()
                + fraction * (curve.domain_end() - curve.domain_start()))) <= tolerance);
    }
    double minimum_y = std::numeric_limits<double>::infinity(), maximum_y = -minimum_y;
    for (const auto& use : result.wire_edges(inner)) {
        const auto& curve = result.m_curves_3d[result.m_edges[use.index].curve_3d_index];
        for (int i = 0; i < curve.cv_count(); ++i)
            MINI_CHECK(std::abs(curve.get_cv(i)[0] + 1) <= tolerance);
        for (const Point& point : {curve.point_at_start(), curve.point_at_end()}) {
            minimum_y = std::min(minimum_y, point[1]);
            maximum_y = std::max(maximum_y, point[1]);
        }
    }
    MINI_CHECK(std::abs(minimum_y + 1 - thickness) <= tolerance);
    MINI_CHECK(std::abs(maximum_y - 1 + thickness) <= tolerance);
    const double normal_length = std::sqrt(1 + 0.125 * 0.125);
    for (double side : {-1.0, 1.0}) {
        const Point foot(-1, 0, side);
        const Point displaced(-1, -side * thickness * 0.125 / normal_length,
            side * (1 - thickness / normal_length));
        MINI_CHECK(std::abs(displaced.distance(foot) - thickness) <= tolerance);
        MINI_CHECK(wire_distance(inner, displaced) <= tolerance);
    }
    MINI_CHECK(brep_classify::point(result, {0, 0, 0}, tolerance) == brep_classify::State::Outside);
    MINI_CHECK(brep_classify::point(result, {-0.95, 0, 0}, tolerance) == brep_classify::State::Outside);
    MINI_CHECK(brep_classify::point(result, {0.95, 0, 0}, tolerance) == brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(result, {0, 0.95, 0}, tolerance) == brep_classify::State::Inside);
}

MINI_TEST("BRep Native Hollow", "Unsupported Source Extension Preserves Working Face") {
    const BRep source = hollow_warped_box();
    const auto original = source.jsondump();
    int curved = -1;
    for (int face = 0; face < source.face_count(); ++face)
        if (!source.m_surfaces[source.m_faces[face].surface_index].is_planar()) {
            curved = face;
            break;
        }
    MINI_CHECK(curved >= 0);
    if (curved < 0)
        return;
    const auto support = brep_offset::displaced_face(source, curved, 0, 1e-7 * 0.125);
    for (bool rational : {false, true}) {
        auto working = support;
        if (rational) {
            auto& surface = working.brep.m_surfaces[working.brep.m_faces[0].surface_index];
            MINI_CHECK(surface.to_rational());
        }
        MINI_CHECK(working.brep.is_valid());
        const auto before = working.brep.jsondump();
        const auto ancestry = working.edges;
        MINI_CHECK(!brep_hollow::source_working::prepare(source, working, rational ? -0.1 : -2, 1e-7 * 0.125));
        MINI_CHECK(working.brep.jsondump() == before);
        MINI_CHECK(working.edges == ancestry && working.source == support.source);
        MINI_CHECK(source.jsondump() == original);
    }
}


// Replaces the historical unsupported assertion after the unchanged physical gate passed.
MINI_TEST("BRep Native Hollow", "Concave Prism Physical Semicylinder Mouth") {
    const BRep source = hollow_concave_prism();
    const auto original = source.jsondump();
    const double tolerance = 1e-7;
    for (double h : {.1, .2}) {
        const BRep result = captured_hollow(source, {4}, -h, "concave-context-prism");
        MINI_CHECK(source.jsondump() == original);
        hollow_boundary(result);
        // Original opening is y=1, x in [1,2]. Cavity section is two rectangles
        // minus the lower radius-h halfdisk at (1,1), extruded over z=[h,2-h].
        const double area = 3 - 7 * h + (4 - std::acos(-1.) * .5) * h * h;
        const double expected = 6 - (2 - 2 * h) * area;
        const double actual = result.volume_exact(tolerance * .1);
        MINI_CHECK(actual > 0 && expected > 0);
        MINI_CHECK(std::abs(actual - expected) <= tolerance * std::max(1., expected));
        for (const Point p : {Point(.5, 1.5, 1), Point(1.5, .5, 1), Point(1.5, 1.5, 1),
                              Point(1 + .8 * h, 1 - .8 * h, 1), Point(1 - 1.1 * h, 1.2, 1)})
            MINI_CHECK(brep_classify::point(result, p, tolerance) == brep_classify::State::Outside);
        for (const Point p :
             {Point(1 + .6 * h, 1 - .6 * h, 1), Point(1 - .5 * h, 1.2, 1), Point(.5, h * .5, 1)})
            MINI_CHECK(brep_classify::point(result, p, tolerance) == brep_classify::State::Inside);
        int mouths = 0;
        for (int face = 0; face < result.face_count(); ++face) {
            const auto& f = result.m_faces[face];
            if (f.wires.size() != 2)
                continue;
            const auto& surface = result.m_surfaces[f.surface_index];
            bool plane = true;
            for (int i = 0; i < surface.cv_count(0); ++i)
                for (int j = 0; j < surface.cv_count(1); ++j)
                    plane = plane && std::abs(surface.get_cv(i, j)[1] - 1) <= tolerance;
            if (!plane)
                continue;
            ++mouths;
            for (int loop = 0; loop < 2; ++loop) {
                double xmin = 1e100, xmax = -1e100, zmin = 1e100, zmax = -1e100;
                for (const auto& use : result.wire_edges(f.wires[loop])) {
                    const auto& edge = result.m_edges[use.index];
                    const auto& world = result.m_curves_3d[edge.curve_3d_index];
                    MINI_CHECK(world.is_linear(tolerance));
                    for (int i = 0; i < world.cv_count(); ++i) {
                        const Point p = world.get_cv(i);
                        MINI_CHECK(std::abs(p[1] - 1) <= tolerance);
                        xmin = std::min(xmin, p[0]);
                        xmax = std::max(xmax, p[0]);
                        zmin = std::min(zmin, p[2]);
                        zmax = std::max(zmax, p[2]);
                    }
                }
                MINI_CHECK(std::abs(xmin - (loop ? 1 + h : 1)) <= tolerance);
                MINI_CHECK(std::abs(xmax - (loop ? 2 - h : 2)) <= tolerance);
                MINI_CHECK(std::abs(zmin - (loop ? h : 0)) <= tolerance);
                MINI_CHECK(std::abs(zmax - (loop ? 2 - h : 2)) <= tolerance);
            }
        }
        MINI_CHECK(mouths == 1);
    }
}

}


MINI_TEST("BRep Native Hollow Context", "Bilinear Working Patch Keeps Prescribed Raw Ends") {
    using namespace session_cpp;
    // Actual source-frame/extent combination from the concave prism at thickness-.2.
    // The right U endpoint is NOT recovered by rounded a+(b-a).
    const std::array<double,4> rectangle = {-1.2000000000000013,1.200000000000002,
        -0.10000000000000009,1.1000000000000012};
    MINI_CHECK(rectangle[0]+(rectangle[1]-rectangle[0]) != rectangle[1]);
    const NurbsSurface plane = NurbsSurface::create(false,false,1,1,2,2,
        {{.8,1,0},{.8,1,2},{.8,2,0},{.8,2,2}});
    brep_hollow::affine_working::Frame frame;
    MINI_CHECK(brep_hollow::affine_working::exact_frame(plane,frame));
    const NurbsSurface extended = brep_hollow::affine_working::enlarged_surface(plane,frame,rectangle,1.25e-8);
    MINI_CHECK(extended.is_valid());
    MINI_CHECK(extended.domain(0) == std::make_pair(rectangle[0],rectangle[1]));
    MINI_CHECK(extended.domain(1) == std::make_pair(rectangle[2],rectangle[3]));
    for (double u : {rectangle[0],0.0,rectangle[1]})
        for (double v : {rectangle[2],0.5,rectangle[3]})
            MINI_CHECK(extended.point_at(u,v).distance(Point(.8,1+u,2*v)) < 1e-12);
}
