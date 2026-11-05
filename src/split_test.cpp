#include "mini_test.h"
#include "split.h"
#include "split_event_ownership.h"
#include "nurbs_interval.h"
#include "brep_classify.h"
#include <numeric>
#include "brep.h"
#include "curve_surface_intersection.h"
#include "curve_intersection.h"
#include "nurbs_bounds.h"
#include "file_step.h"
#include "line.h"
#include "mesh.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "point.h"
#include "polyline.h"
#include "primitives.h"
#include "tolerance.h"
#include "xform.h"
#include <cmath>
#include <algorithm>
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
    split::ArrangementDiagnostic arrangement;
    const auto result = split::split_brep_face_by_pcurves(source, fixture["face"], cuts, fixture["tolerance"], &arrangement);
    if (result.faces.size() <= 1) {
        nlohmann::ordered_json witness = {{"source", "Preserved Rhino Chair Dense Face Cuts"},
            {"fixture", "tests/fixtures/native_chair_face_zero_cuts.json"},
            {"operand", fixture["operand"]}, {"face", fixture["face"]},
            {"tolerance", fixture["tolerance"]}, {"input_cut_count", cuts.size()},
            {"result_faces", result.faces}, {"result_face_count", result.brep.face_count()},
            {"result_edge_count", result.brep.edge_count()},
            {"pieces", nlohmann::ordered_json::array()},
            {"cut_domains", nlohmann::ordered_json::array()}};
        witness["arrangement"] = {{"sources", arrangement.sources}, {"spans", arrangement.spans},
            {"nodes", arrangement.nodes}, {"half_edges", arrangement.half_edges},
            {"retained_cycles", arrangement.retained_cycles}, {"regions", arrangement.regions},
            {"uv_tolerance", arrangement.uv_tolerance}, {"cycles", nlohmann::ordered_json::array()},
            {"first_discarded_reason", arrangement.first_discarded_reason},
            {"first_discarded_uv", nlohmann::ordered_json::array()}};
        witness["arrangement"]["span_boxes"] = nlohmann::ordered_json::array();
        for (const auto& span : arrangement.span_boxes)
            witness["arrangement"]["span_boxes"].push_back({
                {"source", span.source}, {"edge", span.edge}, {"section", span.section},
                {"a", span.a}, {"b", span.b}, {"lo", span.lo}, {"hi", span.hi},
                {"query_lo", span.query_lo}, {"query_hi", span.query_hi}});
        witness["arrangement"]["candidate_pairs"] = arrangement.candidate_pairs;
        witness["arrangement"]["boundary_cut_pairs"] = nlohmann::ordered_json::array();
        for (const auto& pair : arrangement.boundary_cut_pairs)
            witness["arrangement"]["boundary_cut_pairs"].push_back({
                {"first_source", pair.first_source}, {"second_source", pair.second_source},
                {"first", pair.first.jsondump()}, {"second", pair.second.jsondump()},
                {"first_hits", pair.first_hits}, {"second_hits", pair.second_hits},
                {"overlaps", pair.overlaps}});
        for (const split::ArrangementCycleDiagnostic& cycle : arrangement.cycles) {
            nlohmann::ordered_json row = {{"area", cycle.area}, {"runs", cycle.runs},
                {"first_source", cycle.first_source}, {"first_parameter", cycle.first_parameter},
                {"last_parameter", cycle.last_parameter}, {"disposition", cycle.disposition}};
            if (cycle.probe_evaluated)
                row["left_probe"] = {cycle.probe[0], cycle.probe[1]};
            witness["arrangement"]["cycles"].push_back(std::move(row));
        }
        for (const Point& point : arrangement.first_discarded_uv)
            witness["arrangement"]["first_discarded_uv"].push_back({point[0], point[1]});
        for (const split::EdgePiece& piece : result.edges)
            witness["pieces"].push_back({{"source_edge", piece.source_edge},
                {"section", piece.section}, {"edge", piece.edge},
                {"first", piece.first}, {"last", piece.last}});
        for (const split::FaceCut& cut : cuts)
            witness["cut_domains"].push_back({{"world", {cut.world.domain_start(), cut.world.domain_end()}},
                {"pcurve", {cut.pcurve.domain_start(), cut.pcurve.domain_end()}}});
        try {
            const std::filesystem::path folder = root / "build/native-geometry-tests";
            std::filesystem::create_directories(folder);
            std::ofstream output(folder / "native_chair_dense_face_failure.json");
            output << witness.dump(2);
            output.flush();
            const bool written = output.good();
            output.close();
            if (!written || output.fail())
                std::cerr << "dense_face_failure_report_write_failed" << std::endl;
        } catch (const std::exception& error) {
            std::cerr << "dense_face_failure_report_write_failed: " << error.what() << std::endl;
        }
        std::cerr << "dense_face_failure: " << witness.dump() << std::endl;
    }
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

MINI_TEST("Split", "Many Stretches Of One Source Curve") {
    for (bool rational : {false, true}) {
        BRep box = BRep::create_box(10, 10, 10);
        box.transform(Xform::translation(1048576, -524288, 262144));
        const NurbsSurface& surface = box.m_surfaces[box.m_faces[0].surface_index];
        NurbsCurve horizontal = NurbsCurve::create(false, 1, {{0, 0.5, 0}, {1, 0.5, 0}});
        if (rational) {
            MINI_CHECK(horizontal.to_rational());
            for (int i = 0; i < horizontal.cv_count(); ++i) {
                const Point point = horizontal.get_cv(i);
                const double weight = 2 + i;
                MINI_CHECK(horizontal.set_cv_4d(i, point[0] * weight, point[1] * weight, 0, weight));
            }
        }
        std::vector<split::FaceCut> cuts = {planar_cut(surface, horizontal, 11)};
        for (double x : {0.25, 0.5, 0.75})
            cuts.push_back(planar_cut(surface, NurbsCurve::create(false, 1, {{x, 0, 0}, {x, 1, 0}}),
                11 + static_cast<int>(cuts.size())));
        const auto result = split::split_brep_face_by_pcurves(box, 0, cuts, 1e-7);
        MINI_CHECK(result.faces.size() == 8);
        MINI_CHECK(result.brep.is_valid() && result.brep.is_solid());
        int horizontal_parts = 0;
        for (const auto& piece : result.edges) {
            if (piece.section != 11)
                continue;
            ++horizontal_parts;
            MINI_CHECK(piece.last > piece.first);
            int uses = 0;
            for (int face : result.faces)
                for (const auto& wire : result.brep.m_faces[face].wires)
                    for (const auto& edge : result.brep.wire_edges(wire))
                        uses += edge.index == piece.edge ? 1 : 0;
            MINI_CHECK(uses == 2);
        }
        MINI_CHECK(horizontal_parts == 4);
    }
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


MINI_TEST("Split", "Preserved Affine Rational Boundary Endpoint Events") {
    using namespace session_cpp;
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(root / "tests/fixtures/native_hollow_affine_adjacent_boundary_pair.json");
    MINI_CHECK(input.good());
    const auto fixture = nlohmann::ordered_json::parse(input);
    const NurbsCurve a = NurbsCurve::jsonload(fixture.at("section_pcurve"));
    const NurbsCurve b = NurbsCurve::jsonload(fixture.at("boundary_pcurve"));
    const double uv_tolerance = fixture.at("uv_tolerance");
    MINI_CHECK(a.domain_start() == 0 && a.domain_end() == 2 && a.domain() == b.domain());
    MINI_CHECK(a.point_at_end().distance(b.point_at_end()) == 0);
    MINI_CHECK(a.point_at_end()[0] == 1040 && a.point_at_end()[1] == -32.125);
    // Raw kernel contract is unchanged; only the parameter-only UV caller conditions the pair.
    MINI_CHECK(!curve_intersection::intersect(a, b, uv_tolerance).complete);
    NurbsSurface surface(3, false, 2, 2, 2, 2);
    surface.m_nurbsknot[0] = {1016, 1040};
    surface.m_nurbsknot[1] = {-32.125, -31.75};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            MINI_CHECK(surface.set_cv(i, j, {3.0*i, 3.0*j, 0}));
    BRep source;
    source.add_surface(surface);
    std::vector<NurbsCurve> parameters = {a, b,
        NurbsCurve::create(false, 1, {{1016,-31.75,0},{1040,-31.75,0}}),
        NurbsCurve::create(false, 1, {{1016,-32.125,0},{1016,-31.75,0}})};
    std::vector<BRepRef> wire;
    const auto vertex = [&](const Point& point) {
        for (int i = 0; i < source.vertex_count(); ++i)
            if (source.m_vertices[i].point.distance(point) == 0) return i;
        return source.add_vertex(point, 1e-10);
    };
    for (int i = 0; i < 4; ++i) {
        if (i >= 2) MINI_CHECK(parameters[i].set_domain(0, 2));
        const Point first = parameters[i].point_at_start(), last = parameters[i].point_at_end();
        const Point p = surface.point_at(first[0], first[1]), q = surface.point_at(last[0], last[1]);
        NurbsCurve world = NurbsCurve::create(false, 1, {p,q});
        MINI_CHECK(world.set_domain(0,2));
        const int va = vertex(p), vb = vertex(q);
        const int edge = source.add_edge(source.add_curve_3d(world), va, vb);
        source.add_pcurve(edge, 0, source.add_curve_2d(parameters[i]));
        wire.push_back({edge, i == 0 ? BRepOrientation::Forward : BRepOrientation::Reversed});
    }
    source.add_face(0, {{source.add_wire(wire), BRepOrientation::Forward}});
    MINI_CHECK(source.is_valid());
    const auto original = source.jsondump();
    // Match the CAPTURED UV kernel tolerance on this independent analytic plane.
    const double tolerance = uv_tolerance * nurbs_bounds::surface_metric(surface);
    for (bool reverse : {false, true}) {
        NurbsCurve pc = NurbsCurve::create(false, 1, {{1028,-32.125,0},{1028,-31.75,0}});
        NurbsCurve world = NurbsCurve::create(false, 1, {{1.5,0,0},{1.5,3,0}});
        MINI_CHECK(pc.set_domain(-7,13) && world.set_domain(-7,13));
        if (reverse) MINI_CHECK(pc.reverse() && world.reverse());
        const auto result = split::split_brep_face_by_pcurves(source,0,{{world,pc,42}},tolerance);
        MINI_CHECK(result.brep.is_valid() && result.faces.size() == 2);
        int shared = 0;
        for (const auto& piece : result.edges)
            if (piece.section == 42) {
                int uses = 0;
                for (int face : result.faces)
                    for (const auto& w : result.brep.m_faces[face].wires)
                        for (const auto& use : result.brep.wire_edges(w)) uses += use.index == piece.edge;
                MINI_CHECK(uses == 2);
                ++shared;
            }
        MINI_CHECK(shared == 1);
        for (const auto& edge : result.brep.m_edges)
            for (const auto& relation : edge.pcurves)
                if (!edge.degenerated)
                    MINI_CHECK(curve_surface_intersection::same_parameter(
                        result.brep.m_curves_3d[edge.curve_3d_index],
                        result.brep.m_curves_2d[relation.curve_2d_index],
                        result.brep.m_surfaces[relation.surface_index],tolerance)
                        == curve_surface_intersection::ParameterMatch::Matched);
    }
    MINI_CHECK(source.jsondump() == original);
}

MINI_TEST("Split", "Face Boundary Consumer Keeps Independent Raw Domains") {
    using namespace session_cpp;
    BRep box = BRep::create_box(2, 2, 2);
    const int face = 0;
    for (const auto& use : box.wire_edges(box.m_faces[face].wires[0])) {
        const int pc = box.pcurve_index(use.index, face, use.orientation);
        MINI_CHECK(box.m_curves_2d[pc].set_domain(-7, 13));
        MINI_CHECK(box.m_curves_3d[box.m_edges[use.index].curve_3d_index].set_domain(32, 35));
    }
    const auto& support = box.m_surfaces[box.m_faces[face].surface_index];
    const auto pc = NurbsCurve::create(false, 1, {Point(.5, 0, 0), Point(.5, 1, 0)});
    const auto world = NurbsCurve::create(false, 1, {support.point_at(.5, 0), support.point_at(.5, 1)});
    const auto frozen = box.jsondump();
    const auto result = split::split_brep_face_by_pcurves(box, face, {{world, pc, 81}}, 1e-7);
    MINI_CHECK(result.faces.size() == 2);
    MINI_CHECK(result.brep.is_valid() && result.brep.is_solid());
    MINI_CHECK(box.jsondump() == frozen);
}

namespace {
void graph_domain_remove_guids(nlohmann::ordered_json& value) {
    if (value.is_object())
        value.erase("guid");
    if (value.is_object() || value.is_array())
        for (auto& child : value)
            graph_domain_remove_guids(child);
}

session_cpp::BRep graph_domain_disk() {
    using namespace session_cpp;
    BRep shape;
    auto surface = NurbsSurface::create(false, false, 1, 1, 2, 2,
        {Point(-2,-2,0), Point(-2,2,0), Point(2,-2,0), Point(2,2,0)});
    MINI_CHECK(surface.set_domain(0, -2, 2) && surface.set_domain(1, -2, 2));
    const int support = shape.add_surface(surface);
    const auto circle = Primitives::circle(0, 0, 0, 1);
    const int vertex = shape.add_vertex(circle.point_at_start());
    const int edge = shape.add_edge(shape.add_curve_3d(circle), vertex, vertex);
    shape.add_pcurve(edge, support, shape.add_curve_2d(circle));
    shape.add_face(support, {{shape.add_wire({{edge, BRepOrientation::Forward}}), BRepOrientation::Forward}});
    return shape;
}
}
MINI_TEST("Split", "Actual Circle Graph Admission Keeps Inside And Excludes Outside") {
    using namespace session_cpp;
    const BRep disk = graph_domain_disk();
    for (bool outside : {false, true}) {
        const double y = outside ? 1.5 : 0;
        const auto curve = NurbsCurve::create(false, 1, {Point(-1.5,y,0), Point(1.5,y,0)});
        const auto result = split::split_brep_face_by_pcurves(disk, 0, {{curve, curve, 91}}, 1e-5);
        MINI_CHECK(result.faces.size() == (outside ? 1 : 2));
        MINI_CHECK(result.brep.is_valid());
    }
}
MINI_TEST("Split", "Actual Rational Boundary Arc Reuses Original Ownership") {
    using namespace session_cpp;
    const BRep disk = graph_domain_disk();
    NurbsCurve arc = disk.m_curves_2d.front();
    MINI_CHECK(arc.trim(0, 1));
    MINI_CHECK(arc.is_rational() && arc.degree() == 2 && arc.cv_count() == 3);
    const auto frozen = disk.jsondump();
    const auto result = split::split_brep_face_by_pcurves(disk, 0, {{arc, arc, 92}}, .001);
    MINI_CHECK(result.faces.size() == 1);
    MINI_CHECK(result.brep.face_count() == 1 && result.brep.edge_count() == 1);
    MINI_CHECK(result.edges.empty());
    auto output_geometry = result.brep.jsondump();
    auto expected_geometry = frozen;
    graph_domain_remove_guids(output_geometry);
    graph_domain_remove_guids(expected_geometry);
    MINI_CHECK(output_geometry == expected_geometry);
    MINI_CHECK(disk.jsondump() == frozen);
}


namespace session_cpp {
namespace {
nlohmann::ordered_json event_geometry(nlohmann::ordered_json value) {
    if (value.is_object())
        value.erase("guid");
    if (value.is_object() || value.is_array())
        for (auto& item : value)
            item = event_geometry(std::move(item));
    return value;
}

std::vector<split::EdgePiece> event_pieces(const split::FaceSplit& result, int edge) {
    std::vector<split::EdgePiece> pieces;
    for (const auto& piece : result.edges)
        if (piece.source_edge == edge)
            pieces.push_back(piece);
    std::sort(pieces.begin(), pieces.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    return pieces;
}

void event_wire_receipt(const BRep& source, const split::FaceSplit& result, int edge,
                        const std::vector<split::EdgePiece>& pieces) {
    MINI_CHECK(result.brep.m_wires.size() == source.m_wires.size());
    for (size_t i = 0; i < source.m_wires.size(); ++i) {
        std::vector<BRepRef> expected;
        for (const auto& use : source.m_wires[i].edges) {
            if (use.index != edge) {
                expected.push_back(use);
                continue;
            }
            if (use.orientation == BRepOrientation::Reversed)
                for (auto at = pieces.rbegin(); at != pieces.rend(); ++at)
                    expected.push_back({at->edge, use.orientation});
            else
                for (const auto& piece : pieces)
                    expected.push_back({piece.edge, use.orientation});
        }
        MINI_CHECK(result.brep.m_wires[i].edges == expected);
    }
}

void event_piece_pcurves(const BRep& source, const split::FaceSplit& result, int edge,
                         const split::EdgePiece& piece) {
    const auto& generated = result.brep.m_edges.at(piece.edge);
    const auto& world = result.brep.m_curves_3d.at(generated.curve_3d_index);
    const auto domain = source.m_curves_3d.at(source.m_edges.at(edge).curve_3d_index).domain();
    MINI_CHECK(generated.pcurves.size() == source.m_edges.at(edge).pcurves.size());
    for (const auto& image : generated.pcurves) {
        const auto& old_images = source.m_edges.at(edge).pcurves;
        const auto original = std::find_if(old_images.begin(), old_images.end(), [&](const auto& old) {
            return old.surface_index == image.surface_index;
        });
        MINI_CHECK(original != old_images.end());
        const int old_slots[2] = {original->curve_2d_index, original->curve_2d_index_2};
        const int slots[2] = {image.curve_2d_index, image.curve_2d_index_2};
        for (int at = 0; at < 2; ++at) {
            MINI_CHECK((old_slots[at] >= 0) == (slots[at] >= 0));
            if (slots[at] < 0)
                continue;
            const auto& pc = result.brep.m_curves_2d.at(slots[at]);
            const auto pd = source.m_curves_2d.at(old_slots[at]).domain();
            const auto mapped = [&](double t) {
                return t == domain.first    ? pd.first
                       : t == domain.second ? pd.second
                                            : pd.first + (t - domain.first) / (domain.second - domain.first) *
                                                             (pd.second - pd.first);
            };
            MINI_CHECK(pc.domain() == std::make_pair(mapped(piece.first), mapped(piece.last)));
            MINI_CHECK(curve_surface_intersection::same_parameter(
                           world, pc, result.brep.m_surfaces.at(image.surface_index), 1e-7) ==
                       curve_surface_intersection::ParameterMatch::Matched);
        }
    }
}

void event_partition_receipt(const BRep& source, const split::FaceSplit& result, int edge, size_t count) {
    MINI_CHECK(result.brep.is_valid() && result.brep.is_solid());
    MINI_CHECK(result.brep.face_count() == source.face_count());
    const auto before = event_geometry(source.jsondump()), after = event_geometry(result.brep.jsondump());
    for (const std::string key : {"vertices", "curves_3d", "curves_2d", "surfaces", "edges"})
        for (size_t i = 0; i < before[key].size(); ++i)
            MINI_CHECK(before[key][i] == after[key][i]);
    MINI_CHECK(before["faces"] == after["faces"] && before["shells"] == after["shells"] &&
               before["solids"] == after["solids"]);
    const auto pieces = event_pieces(result, edge);
    MINI_CHECK(pieces.size() == count);
    const auto domain = source.m_curves_3d.at(source.m_edges.at(edge).curve_3d_index).domain();
    MINI_CHECK(pieces.front().first == domain.first && pieces.back().last == domain.second);
    for (size_t i = 0; i < pieces.size(); ++i) {
        const auto& piece = pieces[i];
        MINI_CHECK(piece.last > piece.first && piece.section == -1);
        if (i)
            MINI_CHECK(pieces[i - 1].last == piece.first);
        const auto& generated = result.brep.m_edges.at(piece.edge);
        const auto& world = result.brep.m_curves_3d.at(generated.curve_3d_index);
        MINI_CHECK(world.domain() == std::make_pair(piece.first, piece.last));
        event_piece_pcurves(source, result, edge, piece);
    }
    MINI_CHECK(result.edge_events.size() == count + 1);
    for (size_t i = 0; i < result.edge_events.size(); ++i) {
        const auto& event = result.edge_events[i];
        MINI_CHECK(event.edge == edge && event.vertex >= 0 && event.vertex < result.brep.vertex_count());
        if (i < pieces.size())
            MINI_CHECK(event.parameter == pieces[i].first);
        else
            MINI_CHECK(event.parameter == pieces.back().last);
        const int actual = i < pieces.size() ? result.brep.m_edges[pieces[i].edge].start_vertex
                                             : result.brep.m_edges[pieces.back().edge].end_vertex;
        MINI_CHECK(event.vertex == actual);
    }
    event_wire_receipt(source, result, edge, pieces);
}

void event_ranges(BRep& source, int edge) {
    MINI_CHECK(source.m_curves_3d.at(source.m_edges[edge].curve_3d_index).set_domain(11, 17));
    int slot = 0;
    for (const auto& image : source.m_edges[edge].pcurves)
        for (int pc : {image.curve_2d_index, image.curve_2d_index_2})
            if (pc >= 0) {
                const bool first = slot++ == 0;
                MINI_CHECK(source.m_curves_2d.at(pc).set_domain(first ? -7 : 32, first ? 13 : 35));
            }
    MINI_CHECK(slot == 2);
}

std::pair<BRep, int> event_source(bool periodic) {
    BRep source = periodic ? BRep::create_cylinder(2, 3) : BRep::create_box(2, 3, 4);
    int edge = 0;
    if (periodic) {
        edge = -1;
        for (int i = 0; i < source.edge_count(); ++i)
            for (const auto& image : source.m_edges[i].pcurves)
                if (image.curve_2d_index_2 >= 0)
                    edge = i;
        MINI_CHECK(edge >= 0);
    }
    event_ranges(source, edge);
    return {source, edge};
}
} // namespace

MINI_TEST("Split", "Required World Events Split Every Seam And Adjacent Face Use") {
    for (bool periodic : {false, true}) {
        std::vector<int> first_receipt;
        for (bool reverse_order : {false, true}) {
            auto [source, edge] = event_source(periodic);
            if (reverse_order) {
                auto& reference = source.m_faces.front().wires.front();
                auto& wire = source.m_wires.at(reference.index);
                std::reverse(wire.edges.begin(), wire.edges.end());
                for (auto& use : wire.edges)
                    use.orientation = brep_reverse(use.orientation);
                reference.orientation = brep_reverse(reference.orientation);
            }
            const auto& world = source.m_curves_3d.at(source.m_edges[edge].curve_3d_index);
            // A shared operation node may already exist as an orphan in this working owner.
            const int canonical = source.add_vertex(world.point_at(14), 0);
            std::vector<split::EdgeEvent> events{
                {edge, 13, -1}, {edge, 14, canonical}, {edge, 15, -1}, {edge, 14, canonical}};
            if (reverse_order)
                std::reverse(events.begin(), events.end());
            const auto frozen = source.jsondump();
            const auto result = split::split_brep_edges_at_events(source, events, 1e-7);
            event_partition_receipt(source, result, edge, 4);
            MINI_CHECK(result.edge_events.at(2).parameter == 14 &&
                       result.edge_events.at(2).vertex == canonical);
            MINI_CHECK(result.faces.size() == (periodic ? 1u : 2u));
            MINI_CHECK(std::abs(result.brep.volume_exact(1e-9) - (periodic ? 12 * std::acos(-1.) : 24)) <
                       1e-6);
            std::vector<int> receipt;
            for (const auto& event : result.edge_events)
                receipt.push_back(event.vertex);
            if (!reverse_order)
                first_receipt = receipt;
            else
                MINI_CHECK(receipt == first_receipt);
            MINI_CHECK(source.jsondump() == frozen);
        }
    }
}

MINI_TEST("Split", "Required Distinct Nearby Edge Events Keep Separate Vertex Identities") {
    auto [source, edge] = event_source(false);
    const double first = 14, second = 14 + std::ldexp(1., -25);
    const auto frozen = source.jsondump();
    const auto result =
        split::split_brep_edges_at_events(source, {{edge, second, -1}, {edge, first, -1}}, 1e-7);
    event_partition_receipt(source, result, edge, 3);
    MINI_CHECK(result.edge_events.at(1).parameter == first && result.edge_events.at(2).parameter == second);
    MINI_CHECK(result.edge_events.at(1).vertex != result.edge_events.at(2).vertex);
    MINI_CHECK(result.brep.m_vertices[result.edge_events[1].vertex].point.distance(
                   result.brep.m_vertices[result.edge_events[2].vertex].point) < 1e-7);
    MINI_CHECK(source.jsondump() == frozen);
}

MINI_TEST("Split", "Required Edge Events Reject Missing Or Ambiguous Face Pcurves") {
    for (bool duplicate : {false, true}) {
        auto [source, edge] = event_source(false);
        MINI_CHECK(source.m_edges[edge].pcurves.size() == 2);
        if (duplicate) {
            auto image = source.m_edges[edge].pcurves.front();
            image.curve_2d_index = source.add_curve_2d(source.m_curves_2d[image.curve_2d_index]);
            source.m_edges[edge].pcurves.push_back(image);
        } else
            source.m_edges[edge].pcurves.pop_back();
        MINI_CHECK(source.is_valid()); // Structural validity alone does not guarantee per-face ownership.
        const auto frozen = source.jsondump();
        bool rejected = false;
        try {
            split::split_brep_edges_at_events(source, {{edge, 14, -1}}, 1e-7);
        } catch (const split::EdgeEventError&) {
            rejected = true;
        }
        MINI_CHECK(rejected);
        MINI_CHECK(source.jsondump() == frozen);
    }
}

MINI_TEST("Split", "Required Edge Events Reject Conflicting Canonical Vertex Identities") {
    auto [source, edge] = event_source(false);
    const auto& world = source.m_curves_3d.at(source.m_edges[edge].curve_3d_index);
    const int a = source.add_vertex(world.point_at(14), 0), b = source.add_vertex(world.point_at(14), 0);
    MINI_CHECK(a != b);
    const auto frozen = source.jsondump();
    bool rejected = false;
    try {
        split::split_brep_edges_at_events(source, {{edge, 14, a}, {edge, 14, b}}, 1e-7);
    } catch (const split::EdgeEventError& error) {
        rejected = true;
        MINI_CHECK(error.edge == edge && error.first == 14 && error.last == 14);
    }
    MINI_CHECK(rejected);
    MINI_CHECK(source.jsondump() == frozen);
}

MINI_TEST("Split", "Required Edge Event Near A Pcurve Knot Is Preserved Or Explicitly Unresolved") {
    auto [source, edge] = event_source(false);
    const int pc = source.m_edges[edge].pcurves.front().curve_2d_index;
    MINI_CHECK(source.m_curves_2d[pc].insert_nurbsknot(3 + std::ldexp(1., -36)));
    const auto frozen = source.jsondump();
    try {
        const auto result = split::split_brep_edges_at_events(source, {{edge, 14, -1}}, 1e-7);
        event_partition_receipt(source, result, edge, 2);
        MINI_CHECK(result.edge_events.at(1).parameter == 14);
    } catch (const split::EdgeEventError& error) {
        MINI_CHECK(error.edge == edge);
        MINI_CHECK(error.first == 14 || error.last == 14);
        MINI_CHECK(std::string(error.what()).find("raw") != std::string::npos);
    }
    MINI_CHECK(source.jsondump() == frozen);
}

MINI_TEST("Split", "Required Edge Events Reject Foreign Parameters And Vertex Centers") {
    auto [source, edge] = event_source(false);
    const auto frozen = source.jsondump();
    const std::vector<split::EdgeEvent> invalid{{edge, 18, -1},
                                                {edge, std::numeric_limits<double>::quiet_NaN(), -1},
                                                {edge, 14, source.m_edges[edge].start_vertex}};
    for (const auto& event : invalid) {
        bool rejected = false;
        try {
            split::split_brep_edges_at_events(source, {event}, 1e-7);
        } catch (const split::EdgeEventError& error) {
            rejected = true;
            MINI_CHECK(error.edge == edge);
        }
        MINI_CHECK(rejected);
    }
    MINI_CHECK(source.jsondump() == frozen);
}

MINI_TEST("Split", "Already Owned Edge End Events Preserve Original Identity") {
    auto [source, edge] = event_source(false);
    const auto frozen = source.jsondump();
    const auto result = split::split_brep_edges_at_events(source, {{edge, 17, -1}, {edge, 11, -1}}, 1e-7);
    event_partition_receipt(source, result, edge, 1);
    MINI_CHECK(result.edges.front().edge == edge);
    MINI_CHECK(event_geometry(result.brep.jsondump()) == event_geometry(frozen));
    MINI_CHECK(result.edge_events.front().vertex == source.m_edges[edge].start_vertex);
    MINI_CHECK(result.edge_events.back().vertex == source.m_edges[edge].end_vertex);
    MINI_CHECK(source.jsondump() == frozen);
}

MINI_TEST("Split", "Closed Original Edge Events Preserve The Original Seam Vertex") {
    BRep source = BRep::create_cylinder(2, 3);
    int edge = -1;
    for (int i = 0; i < source.edge_count(); ++i)
        if (!source.m_edges[i].degenerated &&
            source.m_edges[i].start_vertex == source.m_edges[i].end_vertex) {
            edge = i;
            break;
        }
    MINI_CHECK(edge >= 0);
    event_ranges(source, edge);
    const auto frozen = source.jsondump();
    const auto result = split::split_brep_edges_at_events(source, {{edge, 15.5, -1}, {edge, 12.5, -1}}, 1e-7);
    event_partition_receipt(source, result, edge, 3);
    MINI_CHECK(result.edge_events.front().vertex == result.edge_events.back().vertex);
    MINI_CHECK(result.edge_events.front().vertex == source.m_edges[edge].start_vertex);
    MINI_CHECK(source.jsondump() == frozen);
}

} // namespace session_cpp


namespace session_cpp {
namespace {
bool circular_contains(const NurbsCurve& curve,double a,double b,const Point& center,double tolerance) {
    return nurbs_interval::curve_within_ball(curve,{std::min(a,b),std::max(a,b)},
        {center[0],center[1],center[2]},tolerance);
}
std::vector<split::ArrangementPointIncidence> circular_incidence(
    const NurbsCurve& boundary,const NurbsCurve& cutter,double lo,double hi,double anchor) {
    const double before=std::lerp(boundary.domain_start(),boundary.domain_end(),.25);
    const double after=std::lerp(boundary.domain_start(),boundary.domain_end(),.75);
    return {{0,1,anchor,lo,before,anchor,cutter.domain_start(),cutter.domain_end()},
            {1,0,lo,anchor,cutter.domain_start(),cutter.domain_end(),before,anchor},
            {0,1,anchor,hi,anchor,after,cutter.domain_start(),cutter.domain_end()},
            {1,0,hi,anchor,cutter.domain_start(),cutter.domain_end(),anchor,after}};
}
}

MINI_TEST("Split", "Original Circular Point Aliases Keep Raw Receipts") {
    const auto boundary=Primitives::circle(0,0,0,1);
    const auto cutter=NurbsCurve::create(false,1,{{-1.5,0,0},{1.5,0,0}});
    const double lo=.4999999999999998,hi=.5,tolerance=2.5e-6;
    const auto events=circular_incidence(boundary,cutter,lo,hi,2);
    const std::vector<split::detail::EventSource> sources={{&boundary,true},{&cutter,false}};
    const auto frozen_b=boundary.jsondump(),frozen_c=cutter.jsondump();
    size_t budget=200000;
    const auto key=split::detail::boundary_vertex_key(cutter,1,sources,events,{lo,hi},false,
        lo,hi,{-1,0,0},tolerance,budget,circular_contains);
    MINI_CHECK(key.has_value());
    if (key) MINI_CHECK(key->first==0 && key->second==2);
    MINI_CHECK(lo<hi && events[1].parameter==lo && events[3].parameter==hi);
    // Original source/raw identities differ even though every point fits the same Tol ball.
    auto distinct=events;
    distinct[2].parameter=std::nextafter(2.,3.);distinct[3].other_parameter=distinct[2].parameter;
    budget=200000;
    MINI_CHECK(!split::detail::boundary_vertex_key(cutter,1,sources,distinct,{lo,hi},false,
        lo,hi,{-1,0,0},tolerance,budget,circular_contains));
    auto other_source=events;
    other_source[2].source=2;other_source[3].other_source=2;
    auto aliases=sources;aliases.push_back({&boundary,true});
    budget=200000;
    MINI_CHECK(!split::detail::boundary_vertex_key(cutter,1,aliases,other_source,{lo,hi},false,
        lo,hi,{-1,0,0},tolerance,budget,circular_contains));
    budget=200000;
    MINI_CHECK(!split::detail::boundary_vertex_key(cutter,1,sources,events,{lo,std::midpoint(lo,hi),hi},false,
        lo,hi,{-1,0,0},tolerance,budget,circular_contains));
    budget=200000;
    MINI_CHECK(!split::detail::boundary_vertex_key(cutter,1,sources,events,{lo,hi},true,
        lo,hi,{-1,0,0},tolerance,budget,circular_contains));
    budget=200000;
    MINI_CHECK(!split::detail::boundary_vertex_key(cutter,1,sources,events,{lo,hi},false,
        lo,hi,{-1+2*tolerance,0,0},tolerance,budget,circular_contains));
    MINI_CHECK(boundary.jsondump()==frozen_b && cutter.jsondump()==frozen_c);
}

MINI_TEST("Split", "Event Vertex Proof Keeps Variable Weight Reversed Raw Laws") {
    auto boundary=Primitives::circle(0,0,0,1);
    MINI_CHECK(boundary.set_domain(32,35));
    for (bool rational:{false,true})
        for (bool reverse:{false,true}) {
            auto cutter=NurbsCurve::create(false,1,{{-1.5,0,0},{1.5,0,0}});
            if (rational) {
                MINI_CHECK(cutter.set_cv_4d(0,-3,0,0,2));
                MINI_CHECK(cutter.set_cv_4d(1,4.5,0,0,3));
            }
            MINI_CHECK(cutter.set_domain(-7,13));
            if (reverse) MINI_CHECK(cutter.reverse());
            const double fraction=rational?(reverse?15./17:2./17):(reverse?5./6:1./6);
            const double root=-7+20*fraction;
            const double lo=std::nextafter(root,-std::numeric_limits<double>::infinity());
            const double hi=std::nextafter(root,std::numeric_limits<double>::infinity());
            const auto events=circular_incidence(boundary,cutter,lo,hi,33.5);
            const std::vector<split::detail::EventSource> sources={{&boundary,true},{&cutter,false}};
            const auto before=cutter.jsondump();
            size_t budget=200000;
            const auto key=split::detail::boundary_vertex_key(cutter,1,sources,events,{lo,hi},false,
                lo,hi,{-1,0,0},2.5e-6,budget,circular_contains);
            MINI_CHECK(key.has_value());
            if (key) MINI_CHECK(key->first==0 && key->second==33.5);
            MINI_CHECK((cutter.domain()==std::pair<double,double>(-7,13)));
            MINI_CHECK(cutter.jsondump()==before);
        }
}

MINI_TEST("Split", "Event Vertex Proof Does Not Contract Positive Reentrant Loops") {
    const double tolerance=std::ldexp(1.,-18),radius=tolerance/8;
    const auto boundary=Primitives::circle(0,0,0,1);
    const auto loop=Primitives::circle(-1-radius,0,0,radius);
    MINI_CHECK(loop.point_at_start().distance({-1,0,0})==0 && loop.is_closed());
    MINI_CHECK(radius*radius>0);
    // Even a genuine positive loop wholly within the ball cannot pass injectivity.
    MINI_CHECK(circular_contains(loop,loop.domain_start(),loop.domain_end(),{-1,0,0},tolerance));
    auto events=circular_incidence(boundary,loop,loop.domain_start(),loop.domain_end(),2);
    std::vector<split::detail::EventSource> sources={{&boundary,true},{&loop,false}};
    size_t budget=200000;
    MINI_CHECK(!split::detail::boundary_vertex_key(loop,1,sources,events,{loop.domain_start(),loop.domain_end()},false,
        loop.domain_start(),loop.domain_end(),{-1,0,0},tolerance,budget,circular_contains));
    NurbsCurve excursion(3,false,3,3);
    excursion.m_nurbsknot={0,0,1,1};
    MINI_CHECK(excursion.set_cv(0,{-1,0,0}) && excursion.set_cv(1,{-.5,0,0}) && excursion.set_cv(2,{-1,0,0}));
    nurbs_interval::CurveBounds midpoint;
    MINI_CHECK(nurbs_interval::curve_bounds(excursion,{.5,.5},{-1,0,0},midpoint));
    MINI_CHECK(midpoint.point[0].lo>.2);
    MINI_CHECK(!circular_contains(excursion,0,1,{-1,0,0},tolerance));
    events=circular_incidence(boundary,excursion,0,1,2);
    sources[1].uv=&excursion;budget=200000;
    MINI_CHECK(!split::detail::boundary_vertex_key(excursion,1,sources,events,{0,1},false,
        0,1,{-1,0,0},tolerance,budget,circular_contains));
}

MINI_TEST("Split", "Circular Event Vertex Admission Keeps Physical Regions And Parameters") {
    const double tolerance=1e-5;
    for (int variant=0;variant<4;++variant) {
        auto disk=graph_domain_disk();
        if (variant) {
            MINI_CHECK(disk.m_curves_2d[0].set_domain(32,35));
            MINI_CHECK(disk.m_curves_3d[0].set_domain(32,35));
        }
        if (variant%2) disk.m_wires[0].edges[0].orientation=BRepOrientation::Reversed;
        auto cut=NurbsCurve::create(false,1,{{-1.5,0,0},{1.5,0,0}});
        if (variant>=2) {
            MINI_CHECK(cut.set_cv_4d(0,-3,0,0,2));
            MINI_CHECK(cut.set_cv_4d(1,4.5,0,0,3));
        }
        if (variant) MINI_CHECK(cut.set_domain(-7,13));
        if (variant%2) MINI_CHECK(cut.reverse());
        const auto frozen=disk.jsondump(),frozen_cut=cut.jsondump();
        split::ArrangementDiagnostic diagnostic;
        const auto result=split::split_brep_face_by_pcurves(disk,0,{{cut,cut,191}},tolerance,&diagnostic);
        MINI_CHECK(result.faces.size()==2 && result.brep.is_valid());
        const auto& support=disk.m_surfaces[0];
        const double uv_tol=tolerance/nurbs_bounds::surface_metric(support);
        for (const auto& receipt:diagnostic.vertex_owned_intervals) {
            MINI_CHECK(receipt.first<receipt.last && receipt.source_edge<0 && receipt.section==191);
            MINI_CHECK(receipt.raw_parameters.size()>=2);
            MINI_CHECK(std::find(receipt.raw_parameters.begin(),receipt.raw_parameters.end(),receipt.first)!=receipt.raw_parameters.end());
            MINI_CHECK(std::find(receipt.raw_parameters.begin(),receipt.raw_parameters.end(),receipt.last)!=receipt.raw_parameters.end());
            MINI_CHECK(circular_contains(cut,receipt.first,receipt.last,receipt.vertex,uv_tol));
            for (const auto& incidence:receipt.incidences) {
                const size_t boundary=incidence.source==receipt.boundary_source?incidence.source:incidence.other_source;
                const double raw=incidence.source==receipt.boundary_source?incidence.parameter:incidence.other_parameter;
                MINI_CHECK(boundary==receipt.boundary_source);
                const auto& original=disk.m_curves_2d[0];
                const auto& edge=disk.m_edges[0];
                MINI_CHECK(raw==receipt.boundary_parameter ||
                    (receipt.boundary_parameter==original.domain_start() && raw==original.domain_end() &&
                     edge.start_vertex==edge.end_vertex && original.is_clamped() &&
                     std::equal(original.cv(0),original.cv(0)+original.dimension(),original.cv(original.cv_count()-1)) &&
                     original.weight(0)==original.weight(original.cv_count()-1)));
            }
        }
        int upper=0,lower=0;
        for (int face:result.faces) {
            const auto up=brep_classify::face_uv(result.brep,face,{0,.5,0},uv_tol);
            const auto down=brep_classify::face_uv(result.brep,face,{0,-.5,0},uv_tol);
            MINI_CHECK(up!=brep_classify::State::Unknown && down!=brep_classify::State::Unknown);
            upper+=up==brep_classify::State::Inside;lower+=down==brep_classify::State::Inside;
            MINI_CHECK((up==brep_classify::State::Inside)!=(down==brep_classify::State::Inside));
        }
        MINI_CHECK(upper==1 && lower==1);
        for (const auto& edge:result.brep.m_edges)
            if (!edge.degenerated)
                for (const auto& image:edge.pcurves)
                    for (int index:{image.curve_2d_index,image.curve_2d_index_2})
                        if (index>=0)
                            MINI_CHECK(curve_surface_intersection::same_parameter(result.brep.m_curves_3d[edge.curve_3d_index],
                                result.brep.m_curves_2d[index],result.brep.m_surfaces[image.surface_index],tolerance)==
                                curve_surface_intersection::ParameterMatch::Matched);
        MINI_CHECK(disk.jsondump()==frozen && cut.jsondump()==frozen_cut);
    }
    const auto disk=graph_domain_disk();
    const auto outside=NurbsCurve::create(false,1,{{-1.5,1.5,0},{1.5,1.5,0}});
    split::ArrangementDiagnostic diagnostic;
    const auto result=split::split_brep_face_by_pcurves(disk,0,{{outside,outside,192}},tolerance,&diagnostic);
    MINI_CHECK(result.faces.size()==1 && diagnostic.vertex_owned_intervals.empty());
}
} // namespace session_cpp


namespace session_cpp {
MINI_TEST("Split", "Event Vertex Ownership Preserves Other Original Source Occurrences") {
    const double tolerance=std::ldexp(1.,-18),r=tolerance/8;
    const Point vertex(-1,0,0);
    const auto boundary=Primitives::circle(0,0,0,1);
    for (bool closed:{false,true}) {
        const std::vector<Point> points=closed?
            std::vector<Point>{{-1,0,0},{-1+r,0,0},{-1,r,0},{-1,0,0}}:
            std::vector<Point>{{-1-.8*tolerance,0,0},{-1-.6*tolerance,0,0},
                {-1-.6*tolerance,10*tolerance,0},{-1+.8*tolerance,10*tolerance,0},
                {-1+.8*tolerance,0,0},{-1+.8*tolerance,-10*tolerance,0}};
        NurbsCurve original(3,false,2,static_cast<int>(points.size()));
        for (size_t i=0;i<points.size();++i) {
            MINI_CHECK(original.set_cv(static_cast<int>(i),points[i]));
            original.m_nurbsknot[i]=static_cast<double>(i);
        }
        MINI_CHECK(original.is_valid());
        auto first_span=original;
        MINI_CHECK(first_span.trim(0,1));
        MINI_CHECK(split::detail::injective_span(first_span));
        MINI_CHECK(circular_contains(original,0,1,vertex,tolerance));
        const double other=closed?3:4;
        MINI_CHECK(circular_contains(original,other,other,vertex,tolerance));
        if (closed) MINI_CHECK(original.is_closed() && r*r*.5>0); // Genuine positive polygon loop.
        auto events=circular_incidence(boundary,original,0,1,2);
        for (auto& event:events)
            if (event.source==1) { event.first=0;event.last=1; }
            else { event.other_first=0;event.other_last=1; }
        const double other_first=closed?2:4,other_last=closed?3:5;
        events.push_back({1,0,other,2,other_first,other_last,1,2});
        events.push_back({0,1,2,other,1,2,other_first,other_last});
        const std::vector<split::detail::EventSource> sources={{&boundary,true},{&original,false}};
        const auto frozen=original.jsondump();
        size_t budget=200000;
        // All positions fit one ball and the local span is injective, but this SAME
        // original source has another required raw occurrence at that node.
        MINI_CHECK(!split::detail::boundary_vertex_key(first_span,1,sources,events,{0,1},false,
            0,1,vertex,tolerance,budget,circular_contains));
        MINI_CHECK(events[1].parameter==0 && events[3].parameter==1 && events[4].parameter==other);
        MINI_CHECK(original.jsondump()==frozen);
    }
}
} // namespace session_cpp

namespace session_cpp {
MINI_TEST("Split", "Closed Boundary Seam Aliases Require Original Endpoint Identity") {
    auto circle=Primitives::circle(0,0,0,1);
    MINI_CHECK(circle.set_domain(32,35));
    const auto frozen=circle.jsondump();
    std::vector<split::detail::EventSource> sources{{&circle,true,9,9}};
    split::ArrangementPointIncidence event{0,0,32,35,32,35,32,35};
    auto key=split::detail::boundary_point_key(event,sources);
    MINI_CHECK(key && key->first==0 && key->second==32);
    MINI_CHECK(event.parameter==32 && event.other_parameter==35);
    sources[0].last_vertex=10;
    MINI_CHECK(!split::detail::boundary_point_key(event,sources));
    sources[0].last_vertex=9;
    auto interior=event;interior.other_parameter=33.5;
    MINI_CHECK(!split::detail::boundary_point_key(interior,sources));
    auto near=circle;
    near.cv(near.cv_count()-1)[0]=std::nextafter(near.cv(0)[0],2.);
    sources[0].uv=&near;
    MINI_CHECK(!split::detail::boundary_point_key(event,sources));
    sources[0]={&circle,false,9,9};
    MINI_CHECK(!split::detail::boundary_point_key(event,sources));
    MINI_CHECK(circle.jsondump()==frozen);
}
} // namespace session_cpp
