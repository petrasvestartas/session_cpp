#include "brep_assemble.h"
#include "brep_classify.h"
#include "file_step.h"
#include "mini_test.h"
#include "split.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

using namespace session_cpp::mini_test;

namespace session_cpp {
namespace {

void append_faces(std::vector<brep_assemble::FaceUse>& faces, const BRep& brep, bool reversed = false) {
    for (int i = 0; i < brep.face_count(); ++i)
        faces.push_back({&brep, i, reversed});
}

void save_assembly(const std::string& name, const BRep& brep) {
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    const std::filesystem::path folder = root / "build/native-geometry-tests";
    std::filesystem::create_directories(folder);
    brep.file_json_dump((folder / (name + ".assembly.brep.json")).string());
    brep.mesh().file_json_dump((folder / (name + ".assembly.mesh.json")).string());
}

double signed_volume(const Mesh& mesh, const Point& origin) {
    double volume = 0.0;
    for (const std::pair<const size_t, std::vector<size_t>>& item : mesh.face) {
        const std::vector<size_t>& indices = item.second;
        if (indices.size() < 3)
            continue;
        const Vector first = *mesh.vertex_point(indices.front()) - origin;
        for (size_t i = 1; i + 1 < indices.size(); ++i)
            volume += first.dot((*mesh.vertex_point(indices[i]) - origin).cross(
                *mesh.vertex_point(indices[i + 1]) - origin));
    }
    return volume / 6.0;
}

nlohmann::ordered_json mesh_diagnostic(const Mesh& mesh, const Point& origin) {
    return {{"vertices", mesh.number_of_vertices()}, {"triangles", mesh.number_of_faces()},
        {"volume", mesh.volume()}, {"signed_volume", signed_volume(mesh, origin)},
        {"valid", mesh.is_valid()}, {"closed", mesh.is_closed()}};
}

nlohmann::ordered_json surface_mesh_diagnostic(const Mesh& mesh, const NurbsSurface& surface) {
    double vertex_gap = 0, centroid_gap = 0, midpoint_gap = 0;
    double normal_deflection = 0, line_deflection = 0;
    int projected_parameters = 0;
    for (const auto& [key, polygon] : mesh.face) {
        if (polygon.size() != 3)
            continue;
        Point uv[3], points[3];
        for (int i = 0; i < 3; ++i) {
            const VertexData& vertex = mesh.vertex.at(polygon[i]);
            points[i] = vertex.position();
            if (vertex.attributes.find("u") != vertex.attributes.end()
                && vertex.attributes.find("v") != vertex.attributes.end())
                uv[i] = Point(vertex.attributes.at("u"), vertex.attributes.at("v"), 0);
            else {
                const auto parameters = surface.closest_parameters(points[i]);
                uv[i] = Point(parameters.first, parameters.second, 0);
                ++projected_parameters;
            }
            vertex_gap = std::max(vertex_gap, points[i].distance(surface.point_at(uv[i][0], uv[i][1])));
        }
        Point center, parameter;
        for (int d = 0; d < 3; ++d) {
            center[d] = (points[0][d] + points[1][d] + points[2][d]) / 3;
            parameter[d] = (uv[0][d] + uv[1][d] + uv[2][d]) / 3;
        }
        const Point lifted = surface.point_at(parameter[0], parameter[1]);
        centroid_gap = std::max(centroid_gap, center.distance(lifted));
        const Vector normal = (points[1] - points[0]).cross(points[2] - points[0]);
        if (normal.magnitude() > 0)
            normal_deflection = std::max(normal_deflection, std::abs((lifted - points[0]).dot(normal)) / normal.magnitude());
        for (int i = 0; i < 3; ++i) {
            const int j = (i + 1) % 3;
            for (int d = 0; d < 3; ++d)
                center[d] = (points[i][d] + points[j][d]) / 2;
            const Point midpoint = surface.point_at((uv[i][0] + uv[j][0]) / 2, (uv[i][1] + uv[j][1]) / 2);
            midpoint_gap = std::max(midpoint_gap, center.distance(midpoint));
            const Vector edge = points[j] - points[i];
            if (edge.magnitude() > 0)
                line_deflection = std::max(line_deflection, (midpoint - points[i]).cross(edge).magnitude() / edge.magnitude());
        }
    }
    return {{"vertex_lift_error", vertex_gap}, {"centroid_parameter_interpolation_gap", centroid_gap},
        {"edge_parameter_interpolation_gap", midpoint_gap}, {"centroid_normal_deflection", normal_deflection},
        {"edge_line_deflection", line_deflection}, {"projected_parameter_uses", projected_parameters}};
}

nlohmann::ordered_json pcurve_diagnostics(const BRep& brep) {
    nlohmann::ordered_json rows = nlohmann::ordered_json::array();
    for (int face = 0; face < brep.face_count(); ++face)
        for (const BRepRef& wire : brep.m_faces[face].wires)
            for (const BRepRef& use : brep.m_wires[wire.index].edges) {
                const int index = brep.pcurve_index(use.index, face, use.orientation);
                if (index < 0)
                    continue;
                const NurbsCurve& curve = brep.m_curves_2d[index];
                const BRepEdge& edge = brep.m_edges[use.index];
                nlohmann::ordered_json row = {{"face", face}, {"wire", wire.index}, {"edge", use.index},
                    {"orientation", static_cast<int>(use.orientation)}, {"pcurve", index},
                    {"domain", {curve.domain_start(), curve.domain_end()}},
                    {"normalized_uv", nlohmann::ordered_json::array()}};
                for (int sample = 0; sample <= 4; ++sample) {
                    const double t = curve.domain_start() +
                        (curve.domain_end() - curve.domain_start()) * sample / 4.0;
                    const Point uv = curve.point_at(t);
                    row["normalized_uv"].push_back({uv[0], uv[1]});
                }
                if (edge.curve_3d_index >= 0) {
                    const NurbsCurve& world = brep.m_curves_3d[edge.curve_3d_index];
                    row["world_domain"] = {world.domain_start(), world.domain_end()};
                }
                rows.push_back(std::move(row));
            }
    return rows;
}

nlohmann::ordered_json sample_comparison(
    const NurbsCurve& a,
    const NurbsCurve& b,
    const NurbsSurface& surface
) {
    const int count = std::min(std::max(a.cv_count() * 4, 24), 4096);
    const std::pair<std::vector<Point>, std::vector<double>> first = a.divide_by_count(count, true);
    const std::pair<std::vector<Point>, std::vector<double>> second = b.divide_by_count(count, true);
    double uv_gap = 0.0;
    double world_gap = 0.0;
    double fraction_gap = 0.0;
    for (size_t i = 0; i < first.first.size() && i < second.first.size(); ++i) {
        const Point& p = first.first[i];
        const Point& q = second.first[i];
        uv_gap = std::max(uv_gap, p.distance(q));
        world_gap = std::max(world_gap, surface.point_at(p[0], p[1]).distance(surface.point_at(q[0], q[1])));
        const double ta = (first.second[i] - a.domain_start()) / (a.domain_end() - a.domain_start());
        const double tb = (second.second[i] - b.domain_start()) / (b.domain_end() - b.domain_start());
        fraction_gap = std::max(fraction_gap, std::abs(ta - tb));
    }
    return {{"count", count}, {"original_count", first.first.size()}, {"result_count", second.first.size()},
        {"max_uv_gap", uv_gap}, {"max_world_gap", world_gap}, {"max_normalized_parameter_gap", fraction_gap}};
}

nlohmann::ordered_json sampling_diagnostics(const BRep& source, const BRep& result) {
    nlohmann::ordered_json rows = nlohmann::ordered_json::array();
    for (const int face : {1, 12, 14}) {
        const BRepFace& a = source.m_faces[face];
        const BRepFace& b = result.m_faces[face];
        for (size_t wire = 0; wire < a.wires.size(); ++wire) {
            const std::vector<BRepRef>& first = source.m_wires[a.wires[wire].index].edges;
            const std::vector<BRepRef>& second = result.m_wires[b.wires[wire].index].edges;
            for (size_t use = 0; use < first.size(); ++use) {
                const int ai = source.pcurve_index(first[use].index, face, first[use].orientation);
                const int bi = result.pcurve_index(second[use].index, face, second[use].orientation);
                nlohmann::ordered_json row = sample_comparison(source.m_curves_2d[ai], result.m_curves_2d[bi],
                    source.m_surfaces[a.surface_index]);
                row["face"] = face;
                row["original_edge"] = first[use].index;
                row["result_edge"] = second[use].index;
                row["original_pcurve"] = ai;
                row["result_pcurve"] = bi;
                rows.push_back(std::move(row));
            }
        }
    }
    return rows;
}

nlohmann::ordered_json save_chair_diagnostics(const std::string& name, const BRep& source, const BRep& result) {
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    const std::filesystem::path folder = root / "build/native-geometry-tests";
    std::filesystem::create_directories(folder);
    const Mesh original_mesh = source.mesh();
    const Mesh result_mesh = result.mesh();
    source.file_json_dump((folder / (name + ".original.brep.json")).string());
    original_mesh.file_json_dump((folder / (name + ".original.mesh.json")).string());
    result.file_json_dump((folder / (name + ".assembly.brep.json")).string());
    result_mesh.file_json_dump((folder / (name + ".assembly.mesh.json")).string());
    const Point origin = source.m_vertices.front().point;
    nlohmann::ordered_json data = {{"original", mesh_diagnostic(original_mesh, origin)},
        {"result", mesh_diagnostic(result_mesh, origin)}, {"faces", nlohmann::ordered_json::array()},
        {"surface_integral_volume", {source.volume_exact(1e-8), result.volume_exact(1e-8)}},
        {"original_pcurves", pcurve_diagnostics(source)}, {"result_pcurves", pcurve_diagnostics(result)},
        {"arc_length_sampling", sampling_diagnostics(source, result)}};
    const std::vector<Mesh> original_faces = source.face_meshes();
    const std::vector<Mesh> result_faces = result.face_meshes();
    for (size_t face = 0; face < original_faces.size() && face < result_faces.size(); ++face)
        data["faces"].push_back({{"face", face}, {"original", mesh_diagnostic(original_faces[face], origin)},
            {"result", mesh_diagnostic(result_faces[face], origin)},
            {"original_surface_error", surface_mesh_diagnostic(original_faces[face],
                source.m_surfaces[source.m_faces[face].surface_index])},
            {"result_surface_error", surface_mesh_diagnostic(result_faces[face],
                result.m_surfaces[result.m_faces[face].surface_index])},
            {"original_orientation", static_cast<int>(source.face_orientation(static_cast<int>(face)))},
            {"result_orientation", static_cast<int>(result.face_orientation(static_cast<int>(face)))}});
    std::ofstream output(folder / (name + ".diagnostic.json"));
    output << data.dump(2);
    return data;
}

}

MINI_TEST("BRepAssemble", "Independent Box Faces") {
    const BRep box = BRep::create_box(2, 2, 2);
    const std::vector<BRep> copies(6, box);
    std::vector<brep_assemble::FaceUse> faces;
    for (int i = 0; i < 6; ++i)
        faces.push_back({&copies[i], i});

    const BRep result = brep_assemble::assemble(faces);
    MINI_CHECK(result.is_valid());
    MINI_CHECK(result.is_solid());
    MINI_CHECK(result.face_count() == 6);
    MINI_CHECK(result.edge_count() == 12);
    MINI_CHECK(result.vertex_count() == 8);
    MINI_CHECK(result.shell_count() == 1);
    MINI_CHECK(result.solid_count() == 1);
    MINI_CHECK(std::abs(result.volume() - 8.0) < 1e-9);
    save_assembly("reassembled_box", result);
}

MINI_TEST("BRepAssemble", "Seams And Degenerated Edges") {
    const std::vector<BRep> sources = {BRep::create_sphere(1), BRep::create_cone(1, 2),
        BRep::create_cylinder(1, 2), BRep::create_torus(2, 0.5)};
    for (const BRep& source : sources) {
        std::vector<brep_assemble::FaceUse> faces;
        append_faces(faces, source);
        const BRep result = brep_assemble::assemble(faces);
        MINI_CHECK(result.is_valid());
        MINI_CHECK(result.is_solid());
        MINI_CHECK(result.face_count() == source.face_count());
        MINI_CHECK(result.edge_count() == source.edge_count());
        MINI_CHECK(result.solid_count() == 1);
        int degenerated = 0;
        int original = 0;
        for (const BRepEdge& edge : result.m_edges)
            degenerated += edge.degenerated ? 1 : 0;
        for (const BRepEdge& edge : source.m_edges)
            original += edge.degenerated ? 1 : 0;
        MINI_CHECK(degenerated == original);
    }
}

MINI_TEST("BRepAssemble", "Disjoint Solids") {
    const BRep a = BRep::create_box(2, 2, 2);
    BRep b = a;
    b.transform(Xform::translation(4, 0, 0));
    std::vector<brep_assemble::FaceUse> faces;
    append_faces(faces, a);
    append_faces(faces, b);
    const BRep result = brep_assemble::assemble(faces);
    MINI_CHECK(result.is_solid());
    MINI_CHECK(result.shell_count() == 2);
    MINI_CHECK(result.solid_count() == 2);
    MINI_CHECK(std::abs(result.volume() - 16.0) < 1e-9);
    MINI_CHECK(brep_classify::point(result, {0, 0, 0}, 1e-7) == brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(result, {4, 0, 0}, 1e-7) == brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(result, {2, 0, 0}, 1e-7) == brep_classify::State::Outside);
    save_assembly("reassembled_disjoint", result);
}

MINI_TEST("BRepAssemble", "Cavity And Nested Island") {
    const BRep outer = BRep::create_box(4, 4, 4);
    const BRep inner = BRep::create_box(2, 2, 2);
    const BRep island = BRep::create_box(0.5, 0.5, 0.5);
    std::vector<brep_assemble::FaceUse> faces;
    append_faces(faces, outer);
    append_faces(faces, inner, true);
    const BRep cavity = brep_assemble::assemble(faces);
    MINI_CHECK(cavity.is_solid());
    MINI_CHECK(cavity.shell_count() == 2);
    MINI_CHECK(cavity.solid_count() == 1);
    MINI_CHECK(cavity.m_solids[0].shells.size() == 2);
    MINI_CHECK(std::abs(cavity.volume() - 56.0) < 1e-9);
    MINI_CHECK(brep_classify::point(cavity, {0, 0, 0}, 1e-7) == brep_classify::State::Outside);
    MINI_CHECK(brep_classify::point(cavity, {1.5, 0, 0}, 1e-7) == brep_classify::State::Inside);
    save_assembly("reassembled_cavity", cavity);

    append_faces(faces, island);
    const BRep nested = brep_assemble::assemble(faces);
    MINI_CHECK(nested.is_solid());
    MINI_CHECK(nested.shell_count() == 3);
    MINI_CHECK(nested.solid_count() == 2);
    MINI_CHECK(std::abs(nested.volume() - 56.125) < 1e-9);
    MINI_CHECK(brep_classify::point(nested, {0, 0, 0}, 1e-7) == brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(nested, {0.75, 0, 0}, 1e-7) == brep_classify::State::Outside);
}

MINI_TEST("BRepAssemble", "Partial Shared Edge Partitions") {
    const BRep box = BRep::create_box(2, 2, 2);
    const NurbsSurface& surface = box.m_surfaces[box.m_faces[0].surface_index];
    const std::pair<double, double> u = surface.domain(0);
    const std::pair<double, double> v = surface.domain(1);
    const double middle = (u.first + u.second) * 0.5;
    const NurbsCurve world = NurbsCurve::create(false, 1,
        {surface.point_at(middle, v.first), surface.point_at(middle, v.second)});
    const NurbsCurve pcurve = NurbsCurve::create(false, 1,
        {{middle, v.first, 0}, {middle, v.second, 0}});
    const split::FaceSplit parts = split::split_brep_face_by_pcurves(box, 0, {{world, pcurve, 17}}, 1e-7);
    MINI_CHECK(parts.faces.size() == 2);
    std::vector<brep_assemble::FaceUse> faces;
    for (const int face : parts.faces)
        faces.push_back({&parts.brep, face, false, &parts.edges, 0});
    for (int i = 1; i < box.face_count(); ++i)
        faces.push_back({&box, i});

    const BRep result = brep_assemble::assemble(faces);
    MINI_CHECK(result.is_solid());
    MINI_CHECK(result.face_count() == 7);
    MINI_CHECK(result.edge_count() == 15);
    MINI_CHECK(result.vertex_count() == 10);
    MINI_CHECK(std::abs(result.volume() - 8.0) < 1e-9);
    save_assembly("reassembled_split_box", result);
}

MINI_TEST("BRepAssemble", "Open And Nonmanifold Inputs") {
    const BRep box = BRep::create_box(2, 2, 2);
    bool open = false;
    try {
        brep_assemble::assemble({{&box, 0}});
    } catch (const std::runtime_error&) {
        open = true;
    }
    MINI_CHECK(open);

    bool duplicate = false;
    try {
        brep_assemble::assemble({{&box, 0}, {&box, 0}});
    } catch (const std::runtime_error&) {
        duplicate = true;
    }
    MINI_CHECK(duplicate);

    const BRep copy = box;
    std::vector<brep_assemble::FaceUse> faces;
    append_faces(faces, box);
    append_faces(faces, copy);
    bool nonmanifold = false;
    try {
        brep_assemble::assemble(faces);
    } catch (const std::runtime_error&) {
        nonmanifold = true;
    }
    MINI_CHECK(nonmanifold);
    MINI_CHECK(brep_assemble::assemble({}).face_count() == 0);
}

MINI_TEST("BRepAssemble", "Pcurve Parameter Repair And Invalid Interior") {
    BRep source = BRep::create_box(2, 2, 2);
    for (NurbsCurve& curve : source.m_curves_3d)
        MINI_CHECK(curve.set_domain(-5, 7));
    for (NurbsCurve& curve : source.m_curves_2d)
        MINI_CHECK(curve.set_domain(10, 20));
    NurbsCurve& weighted = source.m_curves_2d.front();
    const Point last = weighted.point_at_end();
    MINI_CHECK(weighted.to_rational());
    MINI_CHECK(weighted.set_cv_4d(1, last[0] * 4, last[1] * 4, 0, 4));
    std::vector<brep_assemble::FaceUse> faces;
    append_faces(faces, source);
    const BRep result = brep_assemble::assemble(faces);
    MINI_CHECK(result.is_solid());
    MINI_CHECK(std::abs(result.volume() - 8.0) < 1e-9);
    for (const BRepEdge& edge : result.m_edges)
        for (const BRepCurveOnSurface& record : edge.pcurves)
            for (const int index : {record.curve_2d_index, record.curve_2d_index_2})
                if (index >= 0) {
                    MINI_CHECK(result.m_curves_2d[index].domain_start() == -5.0);
                    MINI_CHECK(result.m_curves_2d[index].domain_end() == 7.0);
                }

    BRep invalid = BRep::create_box(2, 2, 2);
    const Point first = invalid.m_curves_2d.front().point_at_start();
    const Point end = invalid.m_curves_2d.front().point_at_end();
    const Vector tangent = end - first;
    const Point middle = first + tangent * 0.5 + Vector(-tangent[1], tangent[0], 0) * 0.1;
    invalid.m_curves_2d.front() = NurbsCurve::create(false, 2, {first, middle, end});
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::filesystem::create_directories(root / "build/native-geometry-tests");
    invalid.file_json_dump((root / "build/native-geometry-tests/invalid_pcurve_interior.brep.json").string());
    faces.clear();
    append_faces(faces, invalid);
    bool rejected = false;
    try {
        brep_assemble::assemble(faces);
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    MINI_CHECK(rejected);
}

MINI_TEST("BRepAssemble", "Freeform Chair Reconstruction") {
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    const std::vector<BRep> chairs = file_step::read_file_step_breps((root / "tests/fixtures/solidbrep.stp").string());
    MINI_CHECK(chairs.size() == 2);
    for (size_t i = 0; i < chairs.size(); ++i) {
        const BRep& chair = chairs[i];
        std::vector<brep_assemble::FaceUse> faces;
        append_faces(faces, chair);
        const BRep result = brep_assemble::assemble(faces, 1e-6);
        MINI_CHECK(result.is_solid());
        MINI_CHECK(result.face_count() == chair.face_count());
        MINI_CHECK(result.edge_count() == chair.edge_count());
        MINI_CHECK(result.vertex_count() == chair.vertex_count());
        MINI_CHECK(result.solid_count() == 1);
        const auto diagnostic = save_chair_diagnostics("reassembled_chair_" + std::to_string(i), chair, result);
        const double original_volume = diagnostic["surface_integral_volume"][0].get<double>();
        const double result_volume = diagnostic["surface_integral_volume"][1].get<double>();
        MINI_CHECK(std::abs(result_volume - original_volume) <= std::abs(original_volume) * 1e-6);
        MINI_CHECK(std::abs(result_volume - 2944.704344237944) <= 2944.704344237944 * 1e-9);
        MINI_CHECK(diagnostic["original"]["valid"].get<bool>() && diagnostic["original"]["closed"].get<bool>());
        MINI_CHECK(diagnostic["result"]["valid"].get<bool>() && diagnostic["result"]["closed"].get<bool>());
        for (int face = 0; face < chair.face_count(); ++face) {
            MINI_CHECK(chair.m_surfaces[chair.m_faces[face].surface_index]
                == result.m_surfaces[result.m_faces[face].surface_index]);
            MINI_CHECK(chair.face_orientation(face) == result.face_orientation(face));
        }
    }
}

}
