#include "mini_test.h"
#include "split.h"
#include "brep.h"
#include "curve_surface_intersection.h"
#include "file_step.h"
#include "line.h"
#include "mesh.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "point.h"
#include "polyline.h"
#include "primitives.h"
#include "tolerance.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("Split", "One Sided Knot Directions") {
    const BRep source = BRep::from_polylines({Polyline({{-2, -2, 0}, {2, -2, 0},
        {2, 2, 0}, {-2, 2, 0}, {-2, -2, 0}})});
    const NurbsCurve diagonal = NurbsCurve::create(false, 1, {{-2, -2, 0}, {2, 2, 0}});
    for (bool reversed : {false, true})
        for (bool swapped : {false, true}) {
            NurbsCurve bent = NurbsCurve::create(false, 1, {{-2, 0, 0}, {0, 0, 0}, {0, 2, 0}});
            MINI_CHECK(bent.set_domain(0, 1));
            if (reversed)
                MINI_CHECK(bent.reverse());
            const std::vector<NurbsCurve> cuts = swapped ? std::vector<NurbsCurve>{diagonal, bent}
                : std::vector<NurbsCurve>{bent, diagonal};
            const BRep result = split::split_brep_face_by_curves(source, 0, cuts, 1e-7);
            const auto folder = std::filesystem::path(__FILE__).parent_path().parent_path() / "build/native-geometry-tests";
            std::filesystem::create_directories(folder);
            result.file_json_dump((folder / "split_one_sided_knot_result.brep.json").string());
            MINI_CHECK(result.is_valid() && result.face_count() == 4);
            double area = 0;
            for (const Mesh& mesh : result.face_meshes_q(true, 20.0, 0.005))
                area += mesh.area();
            MINI_CHECK(std::abs(area - 16.0) < 1e-9);
        }
}

MINI_TEST("Split", "Preserved Inward Offset Cycle Nesting") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_offset_inward_large_cycle_nesting_failure.json");
    MINI_CHECK(input.good());
    const auto fixture = nlohmann::json::parse(input);
    BRep source = BRep::jsonload(fixture["partition"]);
    source.m_edges.resize(4);
    source.m_wires.clear();
    source.m_faces.clear();
    source.m_shells.clear();
    source.m_solids.clear();
    const int wire = source.add_wire({{0}, {1}, {2, BRepOrientation::Reversed}, {3, BRepOrientation::Reversed}});
    source.add_face(0, {{wire}});
    MINI_CHECK(source.is_valid());
    std::vector<split::FaceCut> cuts;
    for (int section : {2, 5, 8, 10}) {
        const auto& saved = fixture["sections"][section];
        const auto& uv = saved[section == 10 ? "first_pcurve" : "second_pcurve"];
        cuts.push_back({NurbsCurve::jsonload(saved["world"]), NurbsCurve::jsonload(uv), section});
    }
    const auto result = split::split_brep_face_by_pcurves(source, 0, cuts, fixture["tolerance"].get<double>() * 0.5);
    MINI_CHECK(result.brep.is_valid() && result.faces.size() == 2);
    int holes = 0;
    for (int face : result.faces) {
        std::set<int> edges;
        const BRepFace& region = result.brep.m_faces[face];
        holes += static_cast<int>(region.wires.size()) - 1;
        for (const BRepRef& boundary : region.wires)
            for (const BRepRef& edge : result.brep.wire_edges(boundary))
                MINI_CHECK(edges.insert(edge.index).second);
    }
    MINI_CHECK(holes == 1);
}

MINI_TEST("Split", "Knot Vertices And Genuine Intersection Events") {
    const BRep box = BRep::create_box(2, 3, 4);
    const NurbsSurface& surface = box.m_surfaces[box.m_faces[0].surface_index];
    const auto u = surface.domain(0);
    const auto v = surface.domain(1);
    const auto cut = [&](bool vertical, double first, double last, int section) {
        std::vector<Point> parameters;
        std::vector<Point> points;
        for (int i = 0; i <= 16; ++i) {
            const double t = first + (last - first) * i / 16.0;
            const Point uv(u.first + (u.second - u.first) * (vertical ? 0.5 : t),
                v.first + (v.second - v.first) * (vertical ? t : 0.5), 0);
            parameters.push_back(uv);
            points.push_back(surface.point_at(uv[0], uv[1]));
        }
        NurbsCurve world = NurbsCurve::create(false, 1, points);
        NurbsCurve pcurve = NurbsCurve::create(false, 1, parameters);
        MINI_CHECK(world.set_domain(0, 1) && pcurve.set_domain(0, 1));
        return split::FaceCut{world, pcurve, section};
    };
    for (int mode = 0; mode < 3; ++mode) {
        std::vector<split::FaceCut> cuts = {cut(false, 0, 1, 42)};
        if (mode == 1)
            cuts.push_back(cut(true, 0, 1, 43));
        if (mode == 2)
            cuts.push_back(cut(false, 0.25, 0.75, 43));
        const auto result = split::split_brep_face_by_pcurves(box, 0, cuts, 1e-7);
        MINI_CHECK(result.brep.is_valid() && result.brep.is_solid());
        MINI_CHECK(result.faces.size() == (mode == 1 ? 4 : 2));
        int pieces = 0;
        bool first_overlap = false;
        bool last_overlap = false;
        for (const auto& piece : result.edges)
            if (piece.section == 42) {
                ++pieces;
                first_overlap = first_overlap || std::abs(piece.last - 0.25) < 1e-10;
                last_overlap = last_overlap || std::abs(piece.first - 0.75) < 1e-10;
            }
        if (mode < 2)
            MINI_CHECK(pieces == (mode == 0 ? 1 : 2));
        else
            MINI_CHECK(pieces >= 3 && first_overlap && last_overlap);
    }
}

MINI_TEST("Split", "Preserved Rhino Chair Trim Sampling") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_chair_trim_sampling_failure.json");
    MINI_CHECK(input.good());
    const auto fixture = nlohmann::json::parse(input);
    const BRep source = BRep::jsonload(fixture["shape"]);
    std::vector<split::FaceCut> cuts;
    for (const auto& cut : fixture["cuts"])
        cuts.push_back({NurbsCurve::jsonload(cut["world"]), NurbsCurve::jsonload(cut["pcurve"]), cut["section"]});
    const auto result = split::split_brep_face_by_pcurves(source, fixture["face"], cuts, fixture["tolerance"]);
    MINI_CHECK(result.brep.is_valid() && result.brep.is_solid());
    MINI_CHECK(result.faces.size() > 1);
}

MINI_TEST("Split", "Preserved Rhino Chair Face Partition Replay") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_chair_sections.json");
    MINI_CHECK(input.good());
    const auto fixture = nlohmann::json::parse(input);
    const auto chairs = file_step::read_file_step_breps((root / "tests/fixtures/solidbrep.stp").string());
    const auto folder = root / "build/native-geometry-tests";
    std::filesystem::create_directories(folder);
    for (size_t operand = 0; operand < chairs.size(); ++operand) {
        BRep partition = chairs[operand];
        for (int face = 0; face < chairs[operand].face_count(); ++face) {
            std::vector<split::FaceCut> cuts;
            nlohmann::ordered_json saved = nlohmann::ordered_json::array();
            int index = 0;
            for (const auto& section : fixture["curves"]) {
                if (section[operand == 0 ? "first_face" : "second_face"] == face) {
                    const auto& uv = section[operand == 0 ? "first_pcurve" : "second_pcurve"];
                    cuts.push_back({NurbsCurve::jsonload(section["world"]), NurbsCurve::jsonload(uv), index});
                    saved.push_back({{"world", section["world"]}, {"pcurve", uv}, {"section", index}});
                }
                ++index;
            }
            if (cuts.empty())
                continue;
            std::cout << "chair partition " << operand << " face " << face << std::endl;
            try {
                const auto result = split::split_brep_face_by_pcurves(partition, face, cuts, fixture["tolerance"]);
                MINI_CHECK(result.brep.is_valid() && result.brep.is_solid());
                partition = result.brep;
            } catch (const split::SamplingError& error) {
                const nlohmann::ordered_json diagnostic = {{"operand", operand}, {"face", face},
                    {"shape", partition.jsondump()}, {"cuts", saved}, {"tolerance", fixture["tolerance"]},
                    {"curve", error.curve.jsondump()}, {"part", error.part.jsondump()},
                    {"uv_tolerance", error.tolerance}, {"visited", error.visited}, {"error", error.what()}};
                std::ofstream(folder / "chair_trim_sampling_failure.json") << diagnostic.dump(2);
                throw;
            }
        }
    }
}

MINI_TEST("Split", "Two Nondegenerate Loops At One Vertex") {
    const BRep face = BRep::from_polylines({Polyline({{-18, -20, 0}, {18, -20, 0},
        {18, 20, 0}, {-18, 20, 0}, {-18, -20, 0}})});
    for (bool inserted : {false, true}) {
        NurbsCurve cut = NurbsCurve::create(false, 4, {{-18, 0, 0}, {15, -4.5, 0},
            {0, 13, 0}, {-15, -19.5, 0}, {18, 18, 0}});
        MINI_CHECK(cut.set_domain(0, 1));
        if (inserted)
            MINI_CHECK(cut.insert_nurbsknot(0.25) && cut.insert_nurbsknot(0.75));
        const auto folder = std::filesystem::path(__FILE__).parent_path().parent_path() / "build/native-geometry-tests";
        std::filesystem::create_directories(folder);
        BRep result;
        try {
            result = split::split_brep_face_by_curves(face, 0, {cut}, 1e-7);
        } catch (const split::CurveError& error) {
            const double fraction = (error.parameter - error.pcurve.domain_start())
                / (error.pcurve.domain_end() - error.pcurve.domain_start());
            const double parameter = error.curve.domain_start()
                + fraction * (error.curve.domain_end() - error.curve.domain_start());
            const Point uv = error.pcurve.point_at(error.parameter);
            const Point lifted = error.surface.point_at(uv[0], uv[1]);
            std::string certificate;
            const auto match = curve_surface_intersection::same_parameter(error.curve, error.pcurve,
                error.surface, error.tolerance, &certificate);
            const nlohmann::ordered_json diagnostic = {{"world", error.curve.jsondump()},
                {"pcurve", error.pcurve.jsondump()}, {"surface", error.surface.jsondump()},
                {"parameter", error.parameter}, {"distance", error.distance}, {"tolerance", error.tolerance},
                {"affine_parameter", parameter}, {"affine_distance", error.curve.point_at(parameter).distance(lifted)},
                {"parameter_match", static_cast<int>(match)}, {"certificate_diagnostic", certificate},
                {"inserted_knots", inserted}};
            std::ofstream(folder / "two_loops_split_correspondence_failure.json") << diagnostic.dump(2);
            throw;
        }
        result.file_json_dump((folder / (inserted ? "two_loops_inserted.brep.json" : "two_loops_single_span.brep.json")).string());
        MINI_CHECK(result.is_valid() && result.face_count() == 4);
        int loops = 0;
        for (const BRepEdge& edge : result.m_edges)
            if (edge.start_vertex == edge.end_vertex && !edge.degenerated) {
                const NurbsCurve& curve = result.m_curves_3d[edge.curve_3d_index];
                const double middle = (curve.domain_start() + curve.domain_end()) * 0.5;
                MINI_CHECK(curve.point_at(middle).distance(result.m_vertices[edge.start_vertex].point) > 1e-3);
                ++loops;
            }
        MINI_CHECK(loops == 2);
    }
}

MINI_TEST("Split", "Single Span Self Crossing Cut") {
    const BRep face = BRep::from_polylines({Polyline({{0, -0.7, 0}, {1.2, -0.7, 0},
        {1.2, 0.7, 0}, {0, 0.7, 0}, {0, -0.7, 0}})});
    NurbsCurve cut = NurbsCurve::create(false, 3, {{0, 0, 0}, {1, 0.1875, 0},
        {1, -0.625, 0}, {0, 0.5625, 0}});
    MINI_CHECK(cut.set_domain(0, 1));
    MINI_CHECK(cut.get_span_vector().size() == 2 && !cut.is_closed());
    const BRep result = split::split_brep_face_by_curves(face, 0, {cut}, 1e-7);
    const auto folder = std::filesystem::path(__FILE__).parent_path().parent_path() / "build/native-geometry-tests";
    std::filesystem::create_directories(folder);
    result.file_json_dump((folder / "single_span_self_crossing_split.brep.json").string());
    MINI_CHECK(result.is_valid());
    MINI_CHECK(result.face_count() == 3);
}

MINI_TEST("Split", "Preserved Rhino Chair Dense Face Cuts") {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_chair_face_zero_cuts.json");
    MINI_CHECK(input.good());
    const auto fixture = nlohmann::json::parse(input);
    const auto chairs = file_step::read_file_step_breps((root / "tests/fixtures/solidbrep.stp").string());
    const BRep& source = chairs.at(fixture["operand"].get<size_t>());
    std::vector<split::FaceCut> cuts;
    for (const auto& cut : fixture["cuts"])
        cuts.push_back({NurbsCurve::jsonload(cut["world"]), NurbsCurve::jsonload(cut["pcurve"]),
            static_cast<int>(cuts.size())});
    const auto result = split::split_brep_face_by_pcurves(source, fixture["face"], cuts, fixture["tolerance"]);
    MINI_CHECK(result.brep.is_valid() && result.brep.is_solid());
    MINI_CHECK(result.faces.size() > 1);
}

MINI_TEST("Split", "Sphere Seams And Pole Edges") {
    const BRep sphere = BRep::create_sphere(2);
    const NurbsSurface& surface = sphere.m_surfaces[sphere.m_faces[0].surface_index];
    for (int direction : {0, 1}) {
        const auto variable = surface.domain(direction);
        const auto fixed = surface.domain(1 - direction);
        const double at = fixed.first + (fixed.second - fixed.first) * (direction == 0 ? 0.5 : 0.25);
        NurbsCurve world = surface.iso_curve(direction, at);
        Point first, last;
        first[direction] = variable.first;
        last[direction] = variable.second;
        first[1 - direction] = last[1 - direction] = at;
        NurbsCurve pcurve = NurbsCurve::create(false, 1, {first, last});
        MINI_CHECK(pcurve.set_domain(world.domain_start(), world.domain_end()));
        const split::FaceSplit result = split::split_brep_face_by_pcurves(sphere, 0, {{world, pcurve, 19}}, 1e-6);
        MINI_CHECK(result.faces.size() == 2);
        MINI_CHECK(result.brep.is_valid() && result.brep.is_solid());
        for (const BRepEdge& edge : result.brep.m_edges)
            if (edge.degenerated)
                MINI_CHECK(edge.start_vertex == edge.end_vertex && edge.curve_3d_index == -1);
    }
}

MINI_TEST("Split", "Paired Face Cut Identity") {
    const BRep box = BRep::create_box(2, 3, 4);
    const NurbsSurface& surface = box.m_surfaces[box.m_faces[0].surface_index];
    const std::pair<double, double> u = surface.domain(0);
    const std::pair<double, double> v = surface.domain(1);
    const double middle = (v.first + v.second) * 0.5;
    NurbsCurve world = surface.iso_curve(0, middle);
    NurbsCurve pcurve = NurbsCurve::create(false, 1, {{u.first, middle, 0}, {u.second, middle, 0}});
    MINI_CHECK(world.set_domain(10, 20) && pcurve.set_domain(10, 20));
    const split::FaceSplit result = split::split_brep_face_by_pcurves(box, 0, {{world, pcurve, 42}}, 1e-7);
    MINI_CHECK(result.brep.is_valid() && result.brep.is_solid());
    MINI_CHECK(result.faces.size() == 2 && result.brep.face_count() == 7);
    MINI_CHECK(box.face_count() == 6);
    int sections = 0;
    for (const split::EdgePiece& piece : result.edges) {
        if (piece.section != 42)
            continue;
        ++sections;
        MINI_CHECK(piece.source_edge == -1);
        MINI_CHECK(std::abs(piece.first - 10.0) < 1e-10 && std::abs(piece.last - 20.0) < 1e-10);
        int uses = 0;
        for (int face : result.faces)
            for (const BRepRef& wire : result.brep.m_faces[face].wires)
                for (const BRepRef& edge : result.brep.wire_edges(wire))
                    uses += edge.index == piece.edge ? 1 : 0;
        MINI_CHECK(uses == 2);
    }
    MINI_CHECK(sections == 1);
}

/// Point at fractions x and y along the first two control edges of a surface.
static Point mapped(const NurbsSurface& surface, double x, double y) {

    const Point a = surface.get_cv(0, 0);
    return a + (surface.get_cv(1, 0) - a) * x + (surface.get_cv(0, 1) - a) * y;
}

static split::FaceCut planar_cut(const NurbsSurface& surface, const NurbsCurve& pcurve, int section) {
    NurbsCurve world = pcurve;
    for (int i = 0; i < world.cv_count(); ++i) {
        const Point uv = pcurve.get_cv(i);
        const Point point = mapped(surface, uv[0], uv[1]);
        const double weight = world.weight(i);
        double* coordinates = world.cv(i);
        for (int d = 0; d < 3; ++d)
            coordinates[d] = point[d] * weight;
    }
    return {world, pcurve, section};
}

MINI_TEST("Split", "Partial Reversed Common Trim Intervals") {
    const BRep box = BRep::create_box(10, 10, 10);
    const NurbsSurface& surface = box.m_surfaces[box.m_faces[0].surface_index];
    const NurbsCurve horizontal = NurbsCurve::create(false, 1, {{0, 0.5, 0}, {1, 0.5, 0}});
    const NurbsCurve shared = NurbsCurve::create(false, 1, {{0.75, 0.5, 0}, {0.25, 0.5, 0}});
    const NurbsCurve vertical = NurbsCurve::create(false, 1, {{0.5, 0, 0}, {0.5, 1, 0}});
    const NurbsCurve boundary = NurbsCurve::create(false, 1, {{0.8, 0, 0}, {0.2, 0, 0}});
    const std::vector<split::FaceCut> cuts = {
        planar_cut(surface, horizontal, 11), planar_cut(surface, shared, 12),
        planar_cut(surface, vertical, 13), planar_cut(surface, boundary, 14)
    };
    const split::FaceSplit result = split::split_brep_face_by_pcurves(box, 0, cuts, 1e-7);
    MINI_CHECK(result.faces.size() == 4);
    MINI_CHECK(result.brep.is_solid());
    int horizontal_parts = 0;
    for (const split::EdgePiece& piece : result.edges) {
        MINI_CHECK(piece.section != 12 && piece.section != 14);
        horizontal_parts += piece.section == 11 ? 1 : 0;
    }
    MINI_CHECK(horizontal_parts == 4);
}

MINI_TEST("Split", "Rational Common Closed Trims") {
    const BRep box = BRep::create_box(10, 10, 10);
    const NurbsSurface& surface = box.m_surfaces[box.m_faces[0].surface_index];
    const NurbsCurve circle = Primitives::circle(0.5, 0.5, 0, 0.25);
    NurbsCurve reversed = circle;
    MINI_CHECK(reversed.reverse());
    NurbsCurve partial = circle;
    const double start = circle.domain_start();
    const double length = circle.domain_end() - start;
    MINI_CHECK(partial.trim(start + length * 0.25, start + length * 0.75));
    const std::vector<split::FaceCut> cuts = {
        planar_cut(surface, circle, 21), planar_cut(surface, reversed, 22),
        planar_cut(surface, partial, 23)
    };
    const split::FaceSplit result = split::split_brep_face_by_pcurves(box, 0, cuts, 1e-7);
    MINI_CHECK(result.faces.size() == 2);
    MINI_CHECK(result.brep.is_solid());
    int common_parts = 0;
    for (const split::EdgePiece& piece : result.edges) {
        MINI_CHECK(piece.section != 22 && piece.section != 23);
        common_parts += piece.section == 21 ? 1 : 0;
    }
    MINI_CHECK(common_parts == 4);
}

MINI_TEST("Split", "Distinct Curved Arcs With Shared Endpoints") {
    const BRep box = BRep::create_box(10, 10, 10);
    const NurbsSurface& surface = box.m_surfaces[box.m_faces[0].surface_index];
    const NurbsCurve upper = NurbsCurve::create(false, 2, {{0, 0.5, 0}, {0.5, 0.9, 0}, {1, 0.5, 0}});
    const NurbsCurve lower = NurbsCurve::create(false, 2, {{0, 0.5, 0}, {0.5, 0.1, 0}, {1, 0.5, 0}});
    const std::vector<split::FaceCut> cuts = {
        planar_cut(surface, upper, 31), planar_cut(surface, lower, 32), planar_cut(surface, upper, 33)
    };
    const split::FaceSplit result = split::split_brep_face_by_pcurves(box, 0, cuts, 1e-7);
    MINI_CHECK(result.faces.size() == 3);
    MINI_CHECK(result.brep.is_solid());
    bool have_upper = false;
    bool have_lower = false;
    for (const split::EdgePiece& piece : result.edges) {
        MINI_CHECK(piece.section != 33);
        have_upper = have_upper || piece.section == 31;
        have_lower = have_lower || piece.section == 32;
    }
    MINI_CHECK(have_upper && have_lower);
}

MINI_TEST("Split", "Split Curve By Curves") {
    // using session_cpp::NurbsCurve;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::split::split_curve_by_curves;

    const NurbsCurve curve = NurbsCurve::create(false, 1, {Point(-2.0, 0.0, 0.0), Point(2.0, 0.0, 0.0)});
    const NurbsCurve cutter = NurbsCurve::create(false, 1, {Point(0.0, -2.0, 0.0), Point(0.0, 2.0, 0.0)});
    const std::vector<NurbsCurve> pieces = split::split_curve_by_curves(curve, {cutter}, 1e-6);

    MINI_CHECK(pieces.size() == 2);
    MINI_CHECK(pieces[0].point_at_start().distance(Point(-2.0, 0.0, 0.0)) < 1e-6);
    MINI_CHECK(pieces[0].point_at_end().distance(Point(0.0, 0.0, 0.0)) < 1e-6);
    MINI_CHECK(pieces[1].point_at_end().distance(Point(2.0, 0.0, 0.0)) < 1e-6);
    MINI_CHECK(curve.point_at_end().distance(Point(2.0, 0.0, 0.0)) < 1e-6);

    const NurbsCurve skew = NurbsCurve::create(false, 1, {Point(0.0, -2.0, 1.0), Point(0.0, 2.0, 1.0)});

    MINI_CHECK(split::split_curve_by_curves(curve, {skew}, 1e-6).size() == 1);

    const NurbsCurve crossing = NurbsCurve::create(
        false,
        1,
        {
            Point(-2.0, -2.0, 0.0),
            Point(2.0, 2.0, 0.0),
            Point(-2.0, 2.0, 0.0),
            Point(2.0, -2.0, 0.0),
        }
    );
    const NurbsCurve short_cut = NurbsCurve::create(false, 1, {Point(0.0, -0.2, 0.0), Point(0.0, 0.2, 0.0)});

    MINI_CHECK(split::split_curve_by_curves(crossing, {short_cut}, 1e-6).size() == 3);

    const NurbsCurve circle = Primitives::circle(0.0, 0.0, 0.0, 1.0);
    const NurbsCurve chord = NurbsCurve::create(false, 1, {Point(-2.0, 0.5, 0.0), Point(2.0, 0.5, 0.0)});
    const std::vector<NurbsCurve> arcs = split::split_curve_by_curves(circle, {chord}, 1e-6);

    MINI_CHECK(arcs.size() == 2);
    MINI_CHECK(arcs[0].is_rational() && arcs[1].is_rational());

    const NurbsCurve tangent = NurbsCurve::create(false, 1, {Point(-2.0, 1.0, 0.0), Point(2.0, 1.0, 0.0)});

    MINI_CHECK(split::split_curve_by_curves(circle, {tangent}, 1e-6).size() == 1);

    bool rejected = false;

    try {
        split::split_curve_by_curves(curve, {curve}, 1e-6);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    MINI_CHECK(rejected);

    rejected = false;

    try {
        split::split_curve_by_curves(curve, {cutter}, std::numeric_limits<double>::quiet_NaN());
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    MINI_CHECK(rejected);
}

MINI_TEST("Split", "Split BRep Face By Curves") {
    // using session_cpp::BRep;
    // using session_cpp::NurbsCurve;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::split::split_brep_face_by_curves;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Mesh;
    // using session_cpp::BRepFace;

    const BRep box = BRep::create_box(10.0, 10.0, 10.0);
    const NurbsSurface surface = box.m_surfaces[0];
    const NurbsCurve cutter = NurbsCurve::create(false, 1, {mapped(surface, 0.5, -1.0), mapped(surface, 0.5, 2.0)});
    const BRep split = split::split_brep_face_by_curves(box, 0, {cutter}, 1e-6);

    MINI_CHECK(split.face_count() == 7);
    MINI_CHECK(split.is_valid() && split.is_solid());
    MINI_CHECK(box.face_count() == 6);

    const std::vector<Mesh> meshes = split.face_meshes_q(true, 20.0, 0.005);

    MINI_CHECK(std::abs(meshes[0].area() - 50.0) < 1e-6);
    MINI_CHECK(std::abs(meshes[6].area() - 50.0) < 1e-6);

    double neighbor_area = 0.0;

    for (int i = 1; i < 6; ++i)
        neighbor_area += meshes[i].area();

    MINI_CHECK(std::abs(neighbor_area - 500.0) < 1e-6);

    const NurbsCurve closed = NurbsCurve::create(
        false,
        3,
        {
            mapped(surface, 0.2, 0.3),
            mapped(surface, 0.8, 0.3),
            mapped(surface, 0.5, 0.9),
            mapped(surface, 0.2, 0.3),
        }
    );
    const BRep island = split::split_brep_face_by_curves(box, 0, {closed}, 1e-6);

    MINI_CHECK(island.face_count() == 7 && island.is_solid());

    const NurbsCurve crossing = NurbsCurve::create(false, 1, {mapped(surface, -1.0, 0.5), mapped(surface, 2.0, 0.5)});
    const BRep quarters = split::split_brep_face_by_curves(box, 0, {cutter, crossing}, 1e-6);

    MINI_CHECK(quarters.face_count() == 9 && quarters.is_solid());

    const BRep repeated = split::split_brep_face_by_curves(split, 0, {crossing}, 1e-6);

    MINI_CHECK(repeated.face_count() == 8 && repeated.is_solid());

    const NurbsCurve loop = NurbsCurve::create(
        false,
        1,
        {
            mapped(surface, 0.2, 0.2),
            mapped(surface, 0.8, 0.2),
            mapped(surface, 0.8, 0.8),
            mapped(surface, 0.2, 0.8),
            mapped(surface, 0.2, 0.2),
        }
    );
    const BRep regions = split::split_brep_face_by_curves(box, 0, {loop}, 1e-6);

    MINI_CHECK(regions.face_count() == 7 && regions.is_solid());

    const BRep restored = BRep::file_json_loads(quarters.file_json_dumps());

    MINI_CHECK(restored.face_count() == 9 && restored.is_solid());

    const BRep protobuf = BRep::pb_loads(quarters.pb_dumps());

    MINI_CHECK(protobuf.face_count() == 9 && protobuf.is_solid());

    const NurbsCurve outer = NurbsCurve::create(
        false,
        1,
        {
            Point(0.0, 0.0, 0.0),
            Point(10.0, 0.0, 0.0),
            Point(10.0, 10.0, 0.0),
            Point(0.0, 10.0, 0.0),
            Point(0.0, 0.0, 0.0),
        }
    );
    const NurbsCurve inner = NurbsCurve::create(
        false,
        1,
        {
            Point(3.0, 3.0, 0.0),
            Point(7.0, 3.0, 0.0),
            Point(7.0, 7.0, 0.0),
            Point(3.0, 7.0, 0.0),
            Point(3.0, 3.0, 0.0),
        }
    );
    const BRep ring = BRep::from_nurbscurves({outer}, {{inner}});
    const NurbsCurve through = NurbsCurve::create(false, 1, {Point(5.0, -1.0, 0.0), Point(5.0, 11.0, 0.0)});
    const BRep divided = split::split_brep_face_by_curves(ring, 0, {through}, 1e-6);

    MINI_CHECK(divided.face_count() == 2);

    const NurbsCurve outside = NurbsCurve::create(false, 1, {Point(1.0, -1.0, 0.0), Point(1.0, 11.0, 0.0)});
    const BRep preserved = split::split_brep_face_by_curves(ring, 0, {outside}, 1e-6);
    int holes = 0;

    for (const BRepFace& face : preserved.m_faces)
        holes += static_cast<int>(face.wires.size()) - 1;

    if (preserved.face_count() != 2 || holes != 1) {
        const std::filesystem::path folder = "build/native-geometry-tests";
        std::filesystem::create_directories(folder);
        const nlohmann::ordered_json failure = {{"source", ring.jsondump()},
            {"cutter", outside.jsondump()}, {"result", preserved.jsondump()},
            {"faces", preserved.face_count()}, {"holes", holes}, {"tolerance", 1e-6}};
        std::ofstream(folder / "split_preserved_hole_failure.json") << failure.dump(2);
    }
    MINI_CHECK(preserved.face_count() == 2 && holes == 1);

    const BRep disk = BRep::from_nurbscurves({Primitives::circle(0.0, 0.0, 0.0, 5.0)});
    const NurbsCurve chord = NurbsCurve::create(false, 1, {Point(-6.0, 1.2, 0.0), Point(6.0, 1.2, 0.0)});
    const BRep halves = split::split_brep_face_by_curves(disk, 0, {chord}, 1e-6);

    MINI_CHECK(halves.face_count() == 2);
    MINI_CHECK(disk.face_count() == 1);

    const BRep cylinder = BRep::create_cylinder(5.0, 10.0);
    const NurbsSurface body = cylinder.m_surfaces[cylinder.m_faces[0].surface_index];
    const std::pair<double, double> domain = body.domain(0);
    const NurbsCurve generator = body.iso_curve(1, (domain.first + domain.second) * 0.5);
    const BRep seamed = split::split_brep_face_by_curves(cylinder, 0, {generator}, 1e-6);

    MINI_CHECK(seamed.face_count() == 4 && seamed.is_solid());
    MINI_CHECK(cylinder.face_count() == 3);
}

MINI_TEST("Split", "Closed Hole Affine And Reversed Clip") {
    for (bool reversed : {false, true}) {
        NurbsCurve outer = NurbsCurve::create(false, 1, {{0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0}, {0, 0, 0}});
        NurbsCurve inner = NurbsCurve::create(false, 1, {{3, 3, 0}, {7, 3, 0}, {7, 7, 0}, {3, 7, 0}, {3, 3, 0}});
        MINI_CHECK(outer.set_domain(-11, 23) && inner.set_domain(3, 19));
        if (reversed)
            MINI_CHECK(inner.reverse());
        const BRep ring = BRep::from_nurbscurves({outer}, {{inner}});
        NurbsCurve cutter = NurbsCurve::create(false, 1, {{1, -1, 0}, {1, 11, 0}});
        MINI_CHECK(cutter.set_domain(-7, 11));
        const BRep result = split::split_brep_face_by_curves(ring, 0, {cutter}, 1e-6);
        int holes = 0;
        for (const BRepFace& face : result.m_faces)
            holes += static_cast<int>(face.wires.size()) - 1;
        MINI_CHECK(result.is_valid() && result.face_count() == 2 && holes == 1);
    }
}

MINI_TEST("Split", "Split Surface By Curves") {
    // using session_cpp::BRep;
    // using session_cpp::NurbsCurve;
    // using session_cpp::Point;
    // using session_cpp::split::split_surface_by_curves;
    // using session_cpp::NurbsSurface;

    const NurbsSurface surface = BRep::create_box(10.0, 10.0, 10.0).m_surfaces[0];
    const NurbsCurve cutter = NurbsCurve::create(false, 1, {mapped(surface, 0.5, -1.0), mapped(surface, 0.5, 2.0)});
    const BRep split = split::split_surface_by_curves(surface, {cutter}, 1e-6);

    MINI_CHECK(split.face_count() == 2);
    MINI_CHECK(split.is_valid() && !split.is_solid());

    const NurbsCurve outside = NurbsCurve::create(false, 1, {mapped(surface, 2.0, -1.0), mapped(surface, 2.0, 2.0)});
    const BRep untouched = split::split_surface_by_curves(surface, {outside}, 1e-6);

    MINI_CHECK(untouched.face_count() == 1);
    MINI_CHECK(surface.is_valid());

    NurbsSurface invalid = surface;
    invalid.set_cv(0, 0, Point(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
    bool rejected = false;

    try {
        split::split_surface_by_curves(invalid, {outside}, 1e-6);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    MINI_CHECK(rejected);
}

MINI_TEST("Split", "Split Line By Curves") {
    // using session_cpp::Arrowhead;
    // using session_cpp::Line;
    // using session_cpp::NurbsCurve;
    // using session_cpp::Point;
    // using session_cpp::split::split_line_by_curves;

    Line line = Line::from_points(Point(-2.0, 0.0, 0.0), Point(2.0, 0.0, 0.0));
    line.name = "retained";
    line.width = 3.0;
    line.dash = {1.0, 2.0};
    line.arrowhead = Arrowhead::BOTH;
    const NurbsCurve cutter = NurbsCurve::create(false, 1, {Point(0.0, -2.0, 0.0), Point(0.0, 2.0, 0.0)});
    const std::vector<Line> pieces = split::split_line_by_curves(line, {cutter}, 1e-6);

    MINI_CHECK(pieces.size() == 2);
    MINI_CHECK(pieces[0].point_at(1.0).distance(Point(0.0, 0.0, 0.0)) < 1e-6);
    MINI_CHECK(pieces[1].point_at(0.0).distance(Point(0.0, 0.0, 0.0)) < 1e-6);
    MINI_CHECK(pieces[0].name == line.name && pieces[0].width == line.width && pieces[0].dash == line.dash);
    MINI_CHECK(pieces[0].arrowhead == Arrowhead::START);
    MINI_CHECK(pieces[1].arrowhead == Arrowhead::END);
    MINI_CHECK(line.length() == 4.0);
}

MINI_TEST("Split", "Split Polyline By Curves") {
    // using session_cpp::Arrowhead;
    // using session_cpp::NurbsCurve;
    // using session_cpp::Point;
    // using session_cpp::Polyline;
    // using session_cpp::split::split_polyline_by_curves;

    Polyline polyline({Point(-2.0, 0.0, 0.0), Point(2.0, 0.0, 0.0), Point(2.0, 3.0, 0.0)});
    polyline.name = "retained";
    polyline.width = 3.0;
    polyline.dash = {1.0, 2.0};
    polyline.arrowhead = Arrowhead::END;
    const NurbsCurve cutter = NurbsCurve::create(false, 1, {Point(0.0, -2.0, 0.0), Point(0.0, 2.0, 0.0)});
    const std::vector<Polyline> pieces = split::split_polyline_by_curves(polyline, {cutter}, 1e-6);

    MINI_CHECK(pieces.size() == 2);
    MINI_CHECK(pieces[0].point_count() == 2 && pieces[1].point_count() == 3);
    MINI_CHECK(pieces[1].get_point(1).distance(Point(2.0, 0.0, 0.0)) < 1e-6);
    MINI_CHECK(pieces[1].get_point(2).distance(Point(2.0, 3.0, 0.0)) < 1e-6);
    MINI_CHECK(pieces[0].name == polyline.name && pieces[0].width == polyline.width && pieces[0].dash == polyline.dash);
    MINI_CHECK(pieces[0].arrowhead == Arrowhead::NONE);
    MINI_CHECK(pieces[1].arrowhead == Arrowhead::END);
    MINI_CHECK(polyline.point_count() == 3);
}

} // namespace session_cpp
