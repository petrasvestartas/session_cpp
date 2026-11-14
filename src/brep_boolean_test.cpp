#include <iostream>
#include "brep_boolean.h"
#include "file_step.h"
#include "nurbs_coincidence.h"
#include "curve_surface_intersection.h"
#include <limits>
#include "brep_acceptance.h"
#include "brep_assemble.h"
#include "brep_classify.h"
#include "brep_measure.h"
#include "mini_test.h"
#include "primitives.h"
#include "session.h"
#include "split.h"
#include "tolerance.h"
#include "xform.h"
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <numeric>
#include <set>

using namespace session_cpp;
using namespace session_cpp::mini_test;

namespace {

const std::array<brep_boolean::Operation, 4> operations = {
    brep_boolean::Operation::Union, brep_boolean::Operation::Intersection,
    brep_boolean::Operation::Difference, brep_boolean::Operation::Xor
};
const std::array<std::string, 4> names = {"union", "intersection", "difference", "xor"};

void save(const std::string& name, const BRep& result) {
    const std::filesystem::path folder = "build/native-geometry-tests/viewer";
    std::filesystem::create_directories(folder);
    BRep geometry = result;
    geometry.name = name;
    geometry.surfacecolor = Color(180 / 255.f, 110 / 255.f, 155 / 255.f);
    geometry.file_json_dump((folder / (name + ".brep.json")).string());
    Session scene(name);
    scene.add_brep(geometry);
    scene.pb_dump((folder / (name + ".pb")).string());
}

void check_volumes(const std::string& name, const BRep& a, const BRep& b,
                   const std::array<double, 4>& expected) {
    nlohmann::ordered_json records = nlohmann::ordered_json::array();
    for (size_t i = 0; i < operations.size(); ++i) {
        const BRep result = brep_boolean::apply(a, b, operations[i], 1e-7);
        MINI_CHECK(result.face_count() == 0 || result.is_valid());
        MINI_CHECK(result.face_count() == 0 || result.is_solid());
        const double volume = result.face_count() ? std::abs(brep_measure::volume(result, 1e-8)) : 0.0;
        MINI_CHECK(std::abs(volume - expected[i]) < 1e-6 * std::max(1.0, expected[i]));
        save(name + "_" + names[i], result);
        records.push_back({{"operation", names[i]}, {"expected_volume", expected[i]}, {"volume", volume},
            {"faces", result.face_count()}, {"shells", result.shell_count()}, {"solids", result.solid_count()}});
    }
    std::ofstream("build/native-geometry-tests/viewer/" + name + ".assertions.json") << records.dump(2);
}

Point warped(const Point& point, double curvature) {
    return {point[0], point[1], point[2] + curvature * point[0] * point[1]};
}

BRep saddle_box(double curvature, const Vector& translation) {
    BRep result = BRep::create_box(2, 2, 2).transformed(Xform::translation(translation[0], translation[1], translation[2]));
    for (BRepVertex& vertex : result.m_vertices)
        vertex.point = warped(vertex.point, curvature);
    for (NurbsCurve& curve : result.m_curves_3d)
        for (int i = 0; i < curve.cv_count(); ++i)
            curve.set_cv(i, warped(curve.get_cv(i), curvature));
    for (NurbsSurface& surface : result.m_surfaces)
        for (int i = 0; i < surface.cv_count(0); ++i)
            for (int j = 0; j < surface.cv_count(1); ++j)
                surface.set_cv(i, j, warped(surface.get_cv(i, j), curvature));
    return result;
}

bool truth(bool a, bool b, brep_boolean::Operation operation) {
    switch (operation) {
        case brep_boolean::Operation::Union: return a || b;
        case brep_boolean::Operation::Intersection: return a && b;
        case brep_boolean::Operation::Difference: return a && !b;
        case brep_boolean::Operation::Xor: return a != b;
    }
    return false;
}

BRep split_identity_face(const BRep& source, int face, double fraction) {
    const NurbsSurface& surface = source.m_surfaces[source.m_faces[face].surface_index];
    const std::pair<double, double> u = surface.domain(0);
    const std::pair<double, double> v = surface.domain(1);
    const double fixed = u.first + (u.second - u.first) * fraction;
    const NurbsCurve world = surface.iso_curve(1, fixed);
    NurbsCurve pcurve = NurbsCurve::create(false, 1, {{fixed, v.first, 0}, {fixed, v.second, 0}});
    MINI_CHECK(pcurve.set_domain(world.domain_start(), world.domain_end()));
    return split::split_brep_face_by_pcurves(source, face, {{world, pcurve, 0}}, 1e-7).brep;
}

const std::array<brep_boolean::Operation, 5> matrix_operations = {
    brep_boolean::Operation::Intersection, brep_boolean::Operation::Union,
    brep_boolean::Operation::Difference, brep_boolean::Operation::Difference,
    brep_boolean::Operation::Xor
};
const std::array<std::string, 5> matrix_names = {
    "intersection", "union", "difference_a_b", "difference_b_a", "xor"
};

bool connected_shell(const BRep& brep, const BRepShell& shell) {
    std::map<int, std::vector<int>> incidence;
    std::map<int, int> balance;
    std::set<int> faces;
    for (const BRepRef& face : shell.faces) {
        if (!faces.insert(face.index).second)
            return false;
        for (const BRepRef& wire : brep.m_faces[face.index].wires)
            for (const BRepRef& edge : brep.wire_edges(wire)) {
                if (brep.m_edges[edge.index].degenerated)
                    continue;
                const BRepOrientation orientation = brep_compose(face.orientation, edge.orientation);
                if (orientation != BRepOrientation::Forward && orientation != BRepOrientation::Reversed)
                    return false;
                incidence[edge.index].push_back(face.index);
                balance[edge.index] += orientation == BRepOrientation::Forward ? 1 : -1;
            }
    }
    std::map<int, std::vector<int>> neighbours;
    for (const std::pair<const int, std::vector<int>>& edge : incidence) {
        if (edge.second.size() != 2 || balance[edge.first] != 0)
            return false;
        neighbours[edge.second[0]].push_back(edge.second[1]);
        neighbours[edge.second[1]].push_back(edge.second[0]);
    }
    if (faces.empty())
        return false;
    std::vector<int> queue{*faces.begin()};
    faces.erase(queue.front());
    for (size_t i = 0; i < queue.size(); ++i)
        for (int neighbour : neighbours[queue[i]])
            if (faces.erase(neighbour))
                queue.push_back(neighbour);
    return faces.empty();
}

bool manifold_components(const BRep& brep) {
    if (brep.face_count() == 0)
        return brep.edge_count() == 0 && brep.shell_count() == 0 && brep.solid_count() == 0;
    if (!brep.is_valid() || !brep.is_solid())
        return false;
    std::set<int> faces;
    std::set<int> shells;
    for (const BRepSolid& solid : brep.m_solids)
        for (const BRepRef& use : solid.shells) {
            if (!shells.insert(use.index).second)
                return false;
            const BRepShell& shell = brep.m_shells[use.index];
            if (!connected_shell(brep, shell))
                return false;
            if (brep_acceptance::shell_vertex_links(brep, shell) != brep_acceptance::VertexLinks::Matched)
                return false;
            for (const BRepRef& face : shell.faces)
                if (!faces.insert(face.index).second)
                    return false;
        }
    return faces.size() == brep.m_faces.size() && shells.size() == brep.m_shells.size();
}

void write_matrix(const std::string& name, const nlohmann::ordered_json& report) {
    const std::filesystem::path folder = "build/native-geometry-tests/viewer";
    std::filesystem::create_directories(folder);
    std::ofstream output(folder / (name + ".assertions.json"));
    if (!(output << report.dump(2)))
        throw std::runtime_error("Could not preserve native Boolean acceptance report");
}

void save_classification_failure(const std::string& name, const brep_boolean::ClassificationError& error) {
    const std::filesystem::path folder = "build/native-geometry-tests/viewer";
    nlohmann::ordered_json record = {{"shape", error.shape.jsondump()},
        {"query", {error.query[0], error.query[1], error.query[2]}}, {"tolerance", error.tolerance},
        {"source_face", error.source_face}, {"second_operand", error.second_operand},
        {"rays", nlohmann::ordered_json::array()}};
    const std::array<Vector, 7> directions = {
        Vector(1, .371, .529), Vector(-.217, 1, .413), Vector(.319, -.271, 1),
        Vector(-1, .723, -.219), Vector(.613, -1, -.417), Vector(-.531, -.337, -1), Vector(1, 1, 1)
    };
    for (const Vector& direction : directions) {
        const auto ray = brep_classify::ray(error.shape, error.query, direction, error.tolerance);
        nlohmann::ordered_json row = {{"direction", {direction[0], direction[1], direction[2]}},
            {"complete", ray.complete}, {"failure", ray.failure}, {"unresolved_face", ray.unresolved_face},
            {"unresolved_domain", ray.unresolved_domain}, {"hits", nlohmann::ordered_json::array()}};
        for (const auto& hit : ray.hits)
            row["hits"].push_back({{"point", {hit.point[0], hit.point[1], hit.point[2]}},
                {"distance", hit.distance}, {"uv", {hit.u, hit.v}}, {"face", hit.face},
                {"crossing", hit.crossing}, {"trim", static_cast<int>(hit.trim)},
                {"distance_enclosure", hit.distance_enclosure}});
        record["rays"].push_back(std::move(row));
    }
    std::ofstream(folder / (name + ".classification_failure.json")) << record.dump(2);
}

nlohmann::ordered_json topology_witness(const brep_assemble::TopologyError& error,
    const std::array<BRep, 2>& operands, const std::string& operation, double tolerance) {
    nlohmann::ordered_json witness = {{"operation", operation}, {"tolerance", tolerance},
        {"partial", error.partial.jsondump()},
        {"operands", {operands[0].jsondump(), operands[1].jsondump()}},
        {"face_orientations", nlohmann::ordered_json::array()},
        {"images", nlohmann::ordered_json::array()}};
    for (BRepOrientation orientation : error.face_orientations)
        witness["face_orientations"].push_back(static_cast<int>(orientation));
    for (const brep_assemble::TopologyImage& image : error.images)
        witness["images"].push_back({{"input", image.input}, {"local_edge", image.local_edge},
            {"original_edge", image.original_edge}, {"section", image.section},
            {"output_edge", image.output_edge}, {"first", image.first},
            {"last", image.last}, {"reversed", image.reversed}});
    witness["input_index_contract"] = "image.input is the first selected FaceUse for its BRep owner, not operand index";
    witness["output_index_contract"] = "image.output_edge indexes partial.m_edges; original_edge indexes its input owner";
    return witness;
}

nlohmann::ordered_json matrix_result(
    const std::string& name,
    const std::array<BRep, 2>& operands,
    int operation,
    double expected,
    int components,
    int shells
) {
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    nlohmann::ordered_json record = {{"operation", matrix_names[operation]}, {"passed", false}};
    if (expected >= 0)
        record["expected_volume"] = expected;
    if (components >= 0)
        record["expected_solids"] = components;
    if (shells >= 0)
        record["expected_shells"] = shells;
    try {
        const int first = operation == 3 ? 1 : 0;
        const BRep result = brep_boolean::apply(operands[first], operands[1 - first], matrix_operations[operation], 1e-7);
        save(name + "_" + matrix_names[operation], result);
        const double volume = result.face_count() ? brep_measure::volume(result, 1e-9) : 0;
        const bool manifold = manifold_components(result);
        record["faces"] = result.face_count();
        record["shells"] = result.shell_count();
        record["solids"] = result.solid_count();
        record["volume"] = volume;
        record["oriented_manifold_components"] = manifold;
        record["passed"] = manifold && (result.face_count() == 0 || volume > 0) &&
            (expected < 0 || std::abs(volume - expected) <= 1e-6 * std::max(1.0, expected)) &&
            (components < 0 || result.solid_count() == components) &&
            (shells < 0 || result.shell_count() == shells);
    } catch (const brep_assemble::TopologyError& error) {
        record["failure"] = error.what();
        if (operation == 0 && name == "native_platonic_dodecahedron") {
            try {
                record["topology_witness"] = topology_witness(error, operands, matrix_names[operation], 1e-7);
            } catch (const std::exception& capture_error) {
                record["topology_witness_failure"] = capture_error.what();
            }
        }
    } catch (const brep_boolean::ClassificationError& error) {
        save_classification_failure(name + "_" + matrix_names[operation], error);
        record["failure"] = error.what();
    } catch (const std::exception& error) {
        record["failure"] = error.what();
    }
    const std::chrono::duration<double, std::milli> elapsed = std::chrono::steady_clock::now() - start;
    record["elapsed_ms"] = elapsed.count();
    return record;
}

bool matrix_algebra(nlohmann::ordered_json& record, const std::array<double, 2>& inputs) {
    std::array<double, 5> volumes{};
    for (int i = 0; i < 5; ++i) {
        if (!record["operations"][i].value("passed", false)) {
            record["algebra_passed"] = false;
            return false;
        }
        volumes[i] = record["operations"][i]["volume"].get<double>();
    }
    const std::array<double, 4> residuals = {
        volumes[1] + volumes[0] - inputs[0] - inputs[1],
        volumes[2] + volumes[0] - inputs[0], volumes[3] + volumes[0] - inputs[1],
        volumes[4] + 2 * volumes[0] - inputs[0] - inputs[1]
    };
    bool valid = true;
    for (double residual : residuals)
        valid = valid && std::abs(residual) <= 1e-6 * std::max({1.0, inputs[0], inputs[1]});
    record["algebra_residuals"] = residuals;
    record["algebra_passed"] = valid;
    return valid;
}

struct BoxCells {
    std::array<std::vector<double>, 3> coordinates;
    std::array<int, 3> dimensions;
    std::vector<std::array<int, 3>> indices;
    std::vector<std::array<bool, 2>> inside;
    std::vector<double> volumes;
};

bool inside_box(const Point& point, const std::array<double, 6>& box) {
    for (int axis = 0; axis < 3; ++axis)
        if (point[axis] <= box[axis] || point[axis] >= box[axis] + box[axis + 3])
            return false;
    return true;
}

BoxCells box_cells(const std::array<std::array<double, 6>, 2>& boxes) {
    BoxCells result;
    for (int axis = 0; axis < 3; ++axis) {
        std::vector<double>& values = result.coordinates[axis];
        for (const std::array<double, 6>& box : boxes) {
            values.push_back(box[axis]);
            values.push_back(box[axis] + box[axis + 3]);
        }
        std::sort(values.begin(), values.end());
        values.erase(std::unique(values.begin(), values.end()), values.end());
        const double span = values.back() - values.front();
        values.insert(values.begin(), values.front() - span);
        values.push_back(values.back() + span);
        result.dimensions[axis] = static_cast<int>(values.size()) - 1;
    }
    for (int x = 0; x < result.dimensions[0]; ++x)
        for (int y = 0; y < result.dimensions[1]; ++y)
            for (int z = 0; z < result.dimensions[2]; ++z) {
                const std::array<int, 3> index{x, y, z};
                Point point;
                double volume = 1;
                for (int axis = 0; axis < 3; ++axis) {
                    const std::vector<double>& values = result.coordinates[axis];
                    point[axis] = (values[index[axis]] + values[index[axis] + 1]) * 0.5;
                    volume *= values[index[axis] + 1] - values[index[axis]];
                }
                result.indices.push_back(index);
                result.inside.push_back({inside_box(point, boxes[0]), inside_box(point, boxes[1])});
                result.volumes.push_back(volume);
            }
    return result;
}

struct CellRegion {
    std::vector<size_t> cells;
    bool boundary = false;
};

std::vector<CellRegion> cell_regions(const BoxCells& cells, const std::vector<bool>& mask) {
    std::vector<CellRegion> regions;
    std::vector<bool> visited(mask.size(), false);
    for (size_t seed = 0; seed < mask.size(); ++seed) {
        if (visited[seed] || !mask[seed])
            continue;
        CellRegion region{{seed}, false};
        visited[seed] = true;
        for (size_t next = 0; next < region.cells.size(); ++next) {
            const std::array<int, 3>& index = cells.indices[region.cells[next]];
            for (int axis = 0; axis < 3; ++axis)
                for (int direction : {-1, 1}) {
                    std::array<int, 3> adjacent = index;
                    adjacent[axis] += direction;
                    if (adjacent[axis] < 0 || adjacent[axis] >= cells.dimensions[axis]) {
                        region.boundary = true;
                        continue;
                    }
                    const size_t id = (adjacent[0] * cells.dimensions[1] + adjacent[1]) * cells.dimensions[2] + adjacent[2];
                    if (!visited[id] && mask[id]) {
                        visited[id] = true;
                        region.cells.push_back(id);
                    }
                }
        }
        regions.push_back(std::move(region));
    }
    return regions;
}

std::pair<int, int> cell_components(const BoxCells& cells, const std::vector<bool>& material) {
    const std::vector<CellRegion> components = cell_regions(cells, material);
    const int solids = static_cast<int>(components.size());
    int cavities = 0;
    for (const CellRegion& component : components) {
        std::vector<bool> complement(material.size(), true);
        for (size_t id : component.cells)
            complement[id] = false;
        for (const CellRegion& region : cell_regions(cells, complement))
            cavities += region.boundary ? 0 : 1;
    }
    return {solids, solids + cavities};
}

nlohmann::ordered_json box_expectation(const BoxCells& cells, int operation) {
    std::vector<bool> material;
    double volume = 0;
    for (size_t i = 0; i < cells.inside.size(); ++i) {
        const int first = operation == 3 ? 1 : 0;
        const bool occupied = truth(cells.inside[i][first], cells.inside[i][1 - first], matrix_operations[operation]);
        material.push_back(occupied);
        if (occupied)
            volume += cells.volumes[i];
    }
    const std::pair<int, int> components = cell_components(cells, material);
    return {{"volume", volume}, {"solids", components.first}, {"shells", components.second}};
}

bool box_case(nlohmann::ordered_json& report, const nlohmann::json& fixture, int index) {
    const std::string name = "native_occt_boxes_" + std::to_string(index);
    std::array<std::array<double, 6>, 2> bounds;
    std::array<BRep, 2> operands;
    std::array<double, 2> volumes;
    for (int i = 0; i < 2; ++i) {
        bounds[i] = fixture["boxes"][i].get<std::array<double, 6>>();
        const std::array<double, 6>& box = bounds[i];
        const Xform translation = Xform::translation(box[0] + box[3] / 2, box[1] + box[4] / 2, box[2] + box[5] / 2);
        operands[i] = BRep::create_box(box[3], box[4], box[5]).transformed(translation);
        volumes[i] = box[3] * box[4] * box[5];
        save(name + "_operand_" + std::to_string(i), operands[i]);
    }
    const BoxCells cells = box_cells(bounds);
    report["cases"].push_back({{"name", name}, {"fixture", fixture}, {"input_volumes", volumes},
        {"operations", nlohmann::ordered_json::array()}, {"passed", false}});
    nlohmann::ordered_json& record = report["cases"].back();
    for (int i = 0; i < 5; ++i) {
        const nlohmann::ordered_json expected = box_expectation(cells, i);
        if (i == 0)
            MINI_CHECK(std::abs(expected["volume"].get<double>() - fixture["intersection_volume"].get<double>()) < 1e-12);
        record["operations"].push_back(matrix_result(name, operands, i,
            expected["volume"].get<double>(), expected["solids"].get<int>(), expected["shells"].get<int>()));
        write_matrix("native_occt_boxes", report);
    }
    const bool passed = matrix_algebra(record, volumes);
    record["passed"] = passed;
    write_matrix("native_occt_boxes", report);
    return passed;
}

std::vector<Point> clip_polygon(const std::vector<Point>& points, const Plane& plane) {
    std::vector<Point> result;
    for (size_t i = 0; i < points.size(); ++i) {
        const Point& a = points[i];
        const Point& b = points[(i + 1) % points.size()];
        const Vector first = a - plane.origin();
        const Vector second = b - plane.origin();
        const double da = first.dot(plane.z_axis());
        const double db = second.dot(plane.z_axis());
        if (da <= 0)
            result.push_back(a);
        if ((da < 0 && db > 0) || (da > 0 && db < 0))
            result.push_back(a + (b - a) * (da / (da - db)));
    }
    return result;
}

double clipped_polyhedron_volume(const std::array<BRep, 2>& operands) {
    double volume = 0;
    for (int side = 0; side < 2; ++side) {
        const std::vector<Polyline> polygons = operands[side].face_polylines();
        const std::vector<Plane> support = operands[side].face_planes();
        const std::vector<Plane> clips = operands[1 - side].face_planes();
        for (size_t face = 0; face < polygons.size(); ++face) {
            std::vector<Point> points = polygons[face].open_points();
            const Vector first = points[1] - points[0];
            const Vector second = points[2] - points[0];
            const Vector normal = first.cross(second);
            if (normal.dot(support[face].z_axis()) < 0)
                std::reverse(points.begin(), points.end());
            for (const Plane& plane : clips)
                points = clip_polygon(points, plane);
            for (size_t i = 1; i + 1 < points.size(); ++i) {
                const Vector a(points[0][0], points[0][1], points[0][2]);
                const Vector b(points[i][0], points[i][1], points[i][2]);
                const Vector c(points[i + 1][0], points[i + 1][1], points[i + 1][2]);
                volume += a.dot(b.cross(c)) / 6;
            }
        }
    }
    return volume;
}

bool platonic_case(nlohmann::ordered_json& report, const Mesh& mesh, const std::string& label) {
    const std::string name = "native_platonic_" + label;
    std::vector<Polyline> polygons;
    for (const std::pair<const size_t, std::vector<size_t>>& face : mesh.face)
        polygons.push_back(mesh.face_polygon(face.first).value());
    const BRep first = BRep::from_polylines(polygons);
    const std::array<BRep, 2> operands = {first, first.transformed(Xform::rotation({1, 2, 3}, 23.0, true))};
    for (int i = 0; i < 2; ++i)
        save(name + "_operand_" + std::to_string(i), operands[i]);
    const double volume = mesh.volume();
    const double overlap = clipped_polyhedron_volume(operands);
    MINI_CHECK(overlap > 0 && overlap < volume);
    const std::array<double, 5> expected{overlap, 2 * volume - overlap, volume - overlap, volume - overlap, 2 * volume - 2 * overlap};
    report["cases"].push_back({{"name", name}, {"rotation_axis", {1, 2, 3}}, {"rotation_degrees", 23.0},
        {"input_volume", volume}, {"clipped_polygon_overlap_volume", overlap},
        {"operations", nlohmann::ordered_json::array()}, {"passed", false}});
    nlohmann::ordered_json& record = report["cases"].back();
    for (int i = 0; i < 5; ++i) {
        record["operations"].push_back(matrix_result(name, operands, i, expected[i], i < 2 ? 1 : -1, i < 2 ? 1 : -1));
        write_matrix("native_platonic", report);
    }
    bool passed = matrix_algebra(record, {volume, volume});
    if (passed) {
        const int a = record["operations"][2]["solids"].get<int>();
        const int b = record["operations"][3]["solids"].get<int>();
        const int exclusive = record["operations"][4]["solids"].get<int>();
        passed = a > 0 && b > 0 && exclusive == a + b;
        record["xor_components_additive"] = passed;
    }
    record["passed"] = passed;
    write_matrix("native_platonic", report);
    return passed;
}

}

MINI_TEST("BRep Native Boolean", "Transverse Boxes All Operations") {
    const BRep a = BRep::create_box(2, 2, 2);
    const BRep b = a.transformed(Xform::translation(0.5, 0.7, 0.9));
    const double common = 1.5 * 1.3 * 1.1;
    check_volumes("native_boxes_transverse", a, b, {16 - common, common, 8 - common, 16 - 2 * common});
    MINI_CHECK(std::abs(brep_boolean::apply(b, a, brep_boolean::Operation::Difference, 1e-7).volume() - (8 - common)) < 1e-6);
}

MINI_TEST("BRep Native Boolean", "Partial Coplanar Boxes") {
    const BRep a = BRep::create_box(2, 2, 2);
    const BRep b = a.transformed(Xform::translation(1, 0, 0));
    check_volumes("native_boxes_coplanar", a, b, {12, 4, 4, 8});
}

MINI_TEST("BRep Native Boolean", "Identical And Disjoint Boxes") {
    const BRep a = BRep::create_box(2, 2, 2);
    check_volumes("native_boxes_identical", a, a, {8, 8, 0, 0});
    check_volumes("native_boxes_disjoint", a, a.transformed(Xform::translation(3, 0, 0)), {16, 0, 8, 16});
}

MINI_TEST("BRep Native Boolean", "Contained Cavity And Empty") {
    const BRep a = BRep::create_box(4, 4, 4);
    const BRep b = BRep::create_box(2, 2, 2);
    check_volumes("native_boxes_cavity", a, b, {64, 8, 56, 56});
    const BRep cavity = brep_boolean::apply(a, b, brep_boolean::Operation::Difference, 1e-7);
    MINI_CHECK(cavity.solid_count() == 1 && cavity.shell_count() == 2);
    MINI_CHECK(brep_classify::point(cavity, {0, 0, 0}, 1e-7) == brep_classify::State::Outside);
    MINI_CHECK(brep_classify::point(cavity, {1.5, 0, 0}, 1e-7) == brep_classify::State::Inside);
    check_volumes("native_boxes_empty", a, BRep(), {64, 0, 64, 64});
}

MINI_TEST("BRep Native Boolean", "Face Edge And Vertex Contact") {
    const BRep a = BRep::create_box(2, 2, 2);
    for (const Vector offset : {Vector(2, 0, 0), Vector(2, 2, 0), Vector(2, 2, 2)}) {
        const BRep b = a.transformed(Xform::translation(offset[0], offset[1], offset[2]));
        check_volumes("native_boxes_contact_" + std::to_string(static_cast<int>(offset[1] + offset[2])), a, b, {16, 0, 8, 16});
    }
}

MINI_TEST("BRep Native Boolean", "Doubly Curved Saddle Solids") {
    const BRep a = saddle_box(0.2, {0, 0, 0});
    const BRep b = saddle_box(-0.15, {0.3, 0.2, 0.2});
    MINI_CHECK(a.is_solid() && b.is_solid());
    MINI_CHECK(!a.m_surfaces[0].is_planar(nullptr, 1e-7) || !a.m_surfaces[1].is_planar(nullptr, 1e-7));
    save("native_saddle_operand_a", a);
    save("native_saddle_operand_b", b);
    for (size_t i = 0; i < operations.size(); ++i) {
        BRep result;
        try {
            result = brep_boolean::apply(a, b, operations[i], 1e-6);
        } catch (const brep_assemble::ParameterError& error) {
            const nlohmann::ordered_json fixture = {
                {"curve", error.curve.jsondump()}, {"pcurve", error.pcurve.jsondump()},
                {"surface", error.surface.jsondump()}, {"tolerance", error.tolerance},
                {"match", static_cast<int>(error.match)}, {"operation", names[i]}, {"failure", error.what()}
            };
            std::ofstream("build/native-geometry-tests/viewer/native_saddle_parameter_failure.json") << fixture.dump(2);
            throw;
        }
        MINI_CHECK(result.is_solid());
        save("native_saddle_" + names[i], result);
        for (int x = -2; x <= 2; ++x)
            for (int y = -2; y <= 2; ++y)
                for (int z = -2; z <= 2; ++z) {
                    const Point p(x * 0.613 + 0.071, y * 0.571 + 0.037, z * 0.683 + 0.031);
                    const auto sa = brep_classify::point(a, p, 1e-6);
                    const auto sb = brep_classify::point(b, p, 1e-6);
                    const auto sr = brep_classify::point(result, p, 1e-6);
                    MINI_CHECK(sa != brep_classify::State::Unknown && sb != brep_classify::State::Unknown && sr != brep_classify::State::Unknown);
                    if (sa == brep_classify::State::Boundary || sb == brep_classify::State::Boundary)
                        continue;
                    MINI_CHECK((sr == brep_classify::State::Inside) == truth(sa == brep_classify::State::Inside, sb == brep_classify::State::Inside, operations[i]));
                }
    }
}

MINI_TEST("BRep Native Boolean", "Empty Identities Both Operand Orders") {
    const BRep source = BRep::create_box(2, 3, 4);
    check_volumes("native_empty_right", source, BRep(), {24, 0, 24, 24});
    check_volumes("native_empty_left", BRep(), source, {24, 0, 0, 24});
    check_volumes("native_empty_both", BRep(), BRep(), {0, 0, 0, 0});
    BRep invalid;
    invalid.add_vertex({0, 0, 0});
    bool rejected = false;
    try {
        brep_boolean::apply(invalid, BRep(), brep_boolean::Operation::Union, 1e-7);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    MINI_CHECK(rejected);
}

MINI_TEST("BRep Native Boolean", "Isolated Tangent Spheres At Interior Witness") {
    const BRep first = BRep::create_sphere(1.0);
    const BRep second = first.transformed(Xform::translation(-2, 0, 0));
    save("native_tangent_sphere_operand_a", first);
    save("native_tangent_sphere_operand_b", second);
    const double volume = 4.0 * Tolerance::PI / 3.0;
    check_volumes("native_tangent_spheres", first, second, {2 * volume, 0, volume, 2 * volume});
    for (brep_boolean::Operation operation : operations) {
        const BRep result = brep_boolean::apply(second, first, operation, 1e-7);
        for (const Point center : {Point(0, 0, 0), Point(-2, 0, 0), Point(3, 0, 0)}) {
            const bool a = center.distance({-2, 0, 0}) < 1.0;
            const bool b = center.distance({0, 0, 0}) < 1.0;
            const brep_classify::State state = brep_classify::point(result, center, 1e-7);
            MINI_CHECK(state == (truth(a, b, operation) ? brep_classify::State::Inside : brep_classify::State::Outside));
        }
    }
}

MINI_TEST("BRep Native Boolean", "Coincident Curved Support With Adjacent Face Partitions") {
    const BRep source = saddle_box(0.2, {0, 0, 0});
    const BRep once = split_identity_face(source, 0, 0.37);
    const BRep twice = split_identity_face(once, 2, 0.61);
    MINI_CHECK(twice.is_solid());
    MINI_CHECK(twice.face_count() == source.face_count() + 2);
    MINI_CHECK(twice.edge_count() > once.edge_count());
    save("native_partitioned_saddle_operand", twice);
    check_volumes("native_partitioned_saddle_identity", source, twice, {8, 8, 0, 0});
}

MINI_TEST("BRep Native Boolean", "Identical Spheres Preserve One Owner") {
    const BRep sphere = BRep::create_sphere(1.0);
    const double volume = 4.0 * Tolerance::PI / 3.0;
    check_volumes("native_spheres_identical", sphere, sphere, {volume, volume, 0, 0});
}

MINI_TEST("BRep Native Boolean", "Box Cell Oracle Touching Shells And True Cavity") {
    const std::array<double, 6> unit{0, 0, 0, 1, 1, 1};
    for (const std::array<double, 6>& other : std::array<std::array<double, 6>, 2>{{
        {-0.25, -0.25, 0.5, 0.5, 0.5, 1}, {0.25, -0.25, 0.5, 0.5, 1.5, 1}}}) {
        const nlohmann::ordered_json expected = box_expectation(box_cells({unit, other}), 4);
        MINI_CHECK(expected["solids"].get<int>() == 2);
        MINI_CHECK(expected["shells"].get<int>() == 2);
    }
    const BoxCells nested = box_cells({{{0, 0, 0, 2, 2, 2}, {0.5, 0.5, 0.5, 1, 1, 1}}});
    for (int operation : {2, 4}) {
        const nlohmann::ordered_json expected = box_expectation(nested, operation);
        MINI_CHECK(expected["volume"].get<double>() == 7);
        MINI_CHECK(expected["solids"].get<int>() == 1);
        MINI_CHECK(expected["shells"].get<int>() == 2);
    }
}

MINI_TEST("BRep Native Boolean", "OCCT Box Configuration Matrix") {
    const std::filesystem::path source = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::ifstream input(source / "tests/fixtures/occt_boolean_boxes.json");
    MINI_CHECK(input.good());
    const nlohmann::json fixtures = nlohmann::json::parse(input);
    MINI_CHECK(fixtures["cases"].size() == 38);
    nlohmann::ordered_json report = {{"backend", "session_cpp_native"}, {"occt_commit", fixtures["occt_commit"]},
        {"tolerance", 1e-7}, {"oracle", "axis-aligned cell volume, face-connected material and per-solid cavity shells"},
        {"cases", nlohmann::ordered_json::array()}, {"passed", false}};
    int passed = 0;
    for (size_t i = 0; i < fixtures["cases"].size(); ++i)
        passed += box_case(report, fixtures["cases"][i], static_cast<int>(i)) ? 1 : 0;
    report["passed_cases"] = passed;
    report["passed"] = passed == 38;
    write_matrix("native_occt_boxes", report);
    MINI_CHECK(passed == 38);
}

MINI_TEST("BRep Native Boolean", "All Five Rotated Platonic Solids") {
    const std::array<Mesh, 5> meshes = {Primitives::tetrahedron(), Primitives::cube(), Primitives::octahedron(),
        Mesh::create_dodecahedron(), Primitives::icosahedron()};
    const std::array<std::string, 5> labels = {"tetrahedron", "cube", "octahedron", "dodecahedron", "icosahedron"};
    nlohmann::ordered_json report = {{"backend", "session_cpp_native"}, {"tolerance", 1e-7},
        {"oracle", "convex polygon half-space clipping and signed tetrahedral volume"},
        {"cases", nlohmann::ordered_json::array()}, {"passed", false}};
    int passed = 0;
    for (size_t i = 0; i < meshes.size(); ++i)
        passed += platonic_case(report, meshes[i], labels[i]) ? 1 : 0;
    report["passed_cases"] = passed;
    report["passed"] = passed == 5;
    write_matrix("native_platonic", report);
    if (passed != 5)
        for (const nlohmann::ordered_json& record : report["cases"])
            if (!record.value("passed", false))
                std::cerr << "platonic_case_failure: " << record.dump() << std::endl;
    MINI_CHECK(passed == 5);
}

MINI_TEST("BRep Native Boolean", "Rotated Cube Classification") {
    const Mesh mesh = Primitives::cube();
    std::vector<Polyline> polygons;
    for (const auto& [key, vertices] : mesh.face)
        polygons.push_back(mesh.face_polygon(key).value());
    const BRep first = BRep::from_polylines(polygons);
    const std::array<BRep, 2> operands = {first, first.transformed(Xform::rotation({1, 2, 3}, 23.0, true))};
    const double expected = clipped_polyhedron_volume(operands);
    const auto result = matrix_result("native_rotated_cube_diagnostic", operands, 0, expected, 1, 1);
    std::ofstream("build/native-geometry-tests/viewer/native_rotated_cube_diagnostic.assertions.json")
        << nlohmann::ordered_json::array({result}).dump(2);
    MINI_CHECK(result["passed"].get<bool>());
}

MINI_TEST("BRep Native Boolean", "Vertex Link Cycle Multiplicity") {
    brep_acceptance::VertexLink loop;
    loop.neighbours[{0, 0}] = {{0, 0}, {0, 0}};
    MINI_CHECK(brep_acceptance::link_cycle(loop));
    brep_acceptance::VertexLink parallel;
    parallel.neighbours[{0, 0}] = {{0, 1}, {0, 1}};
    parallel.neighbours[{0, 1}] = {{0, 0}, {0, 0}};
    MINI_CHECK(brep_acceptance::link_cycle(parallel));
    brep_acceptance::VertexLink two_cycles = parallel;
    two_cycles.neighbours[{1, 0}] = {{1, 1}, {1, 1}};
    two_cycles.neighbours[{1, 1}] = {{1, 0}, {1, 0}};
    MINI_CHECK(!brep_acceptance::link_cycle(two_cycles));
    brep_acceptance::VertexLink degree_four = loop;
    degree_four.neighbours[{0, 0}].push_back({0, 0});
    degree_four.neighbours[{0, 0}].push_back({0, 0});
    MINI_CHECK(!brep_acceptance::link_cycle(degree_four));
}

MINI_TEST("BRep Native Boolean", "Vertex Link Seams Poles And Pinch") {
    for (const BRep& shape : {BRep::create_box(2, 3, 4), BRep::create_cylinder(1, 2),
                              BRep::create_sphere(1), BRep::create_torus(3, 1)}) {
        MINI_CHECK(manifold_components(shape));
        for (const BRepShell& shell : shape.m_shells)
            MINI_CHECK(brep_acceptance::shell_vertex_links(shape, shell) == brep_acceptance::VertexLinks::Matched);
    }
    BRep pinched = BRep::create_box(2, 3, 4);
    int other = -1;
    for (int v = 1; v < pinched.vertex_count(); ++v) {
        bool adjacent = false;
        for (const BRepEdge& edge : pinched.m_edges)
            adjacent |= (edge.start_vertex == 0 && edge.end_vertex == v)
                || (edge.end_vertex == 0 && edge.start_vertex == v);
        if (!adjacent) {
            other = v;
            break;
        }
    }
    MINI_CHECK(other >= 0);
    for (BRepEdge& edge : pinched.m_edges) {
        if (edge.start_vertex == other)
            edge.start_vertex = 0;
        if (edge.end_vertex == other)
            edge.end_vertex = 0;
    }
    MINI_CHECK(connected_shell(pinched, pinched.m_shells.front()));
    MINI_CHECK(brep_acceptance::shell_vertex_links(pinched, pinched.m_shells.front()) == brep_acceptance::VertexLinks::NonManifold);
}

MINI_TEST("BRep Native Boolean", "Vertex Link Degenerate Sector Contract") {
    BRep sphere = BRep::create_sphere(1);
    const int wire = sphere.m_faces.front().wires.front().index;
    int degenerate = -1;
    for (size_t i = 0; i < sphere.m_wires[wire].edges.size(); ++i)
        if (sphere.m_edges[sphere.m_wires[wire].edges[i].index].degenerated) {
            degenerate = static_cast<int>(i);
            break;
        }
    MINI_CHECK(degenerate >= 0);
    const BRepRef pole = sphere.m_wires[wire].edges.at(degenerate);
    sphere.m_wires[wire].edges.insert(sphere.m_wires[wire].edges.begin() + degenerate, pole);
    MINI_CHECK(brep_acceptance::shell_vertex_links(sphere, sphere.m_shells.front()) == brep_acceptance::VertexLinks::Matched);
    sphere.m_wires[wire].edges = {pole};
    MINI_CHECK(brep_acceptance::shell_vertex_links(sphere, sphere.m_shells.front()) == brep_acceptance::VertexLinks::Unsupported);
    sphere.m_edges[pole.index].end_vertex = sphere.vertex_count();
    sphere.m_vertices.push_back(sphere.m_vertices.front());
    MINI_CHECK(brep_acceptance::shell_vertex_links(sphere, sphere.m_shells.front()) == brep_acceptance::VertexLinks::Unsupported);
}

namespace {

NurbsSurface affine_support(NurbsSurface surface, int variant) {
    if (variant & 4)
        MINI_CHECK(surface.transpose());
    for (int d = 0; d < 2; ++d) {
        if (variant & (1 << d))
            MINI_CHECK(surface.reverse(d));
        const std::pair<double, double> domain = surface.domain(d);
        const double scale = d == 0 ? 2 : 4, offset = d == 0 ? 4 : -8;
        MINI_CHECK(surface.set_domain(d, scale * domain.first + offset, scale * domain.second + offset));
    }
    return surface;
}

NurbsCurve affine_trim(NurbsCurve curve, const NurbsSurface& original, int variant) {
    for (int i = 0; i < curve.cv_count(); ++i) {
        double x, y, z, w;
        MINI_CHECK(curve.get_cv_4d(i, x, y, z, w));
        const double source[2] = {x, y};
        double result[2];
        for (int d = 0; d < 2; ++d) {
            const int axis = (variant & 4) ? 1 - d : d;
            const std::pair<double, double> domain = original.domain(axis);
            const double coordinate = (variant & (1 << d))
                ? (domain.first + domain.second) * w - source[axis] : source[axis];
            result[d] = (d == 0 ? 2 : 4) * coordinate + (d == 0 ? 4 : -8) * w;
        }
        MINI_CHECK(curve.set_cv_4d(i, result[0], result[1], z, w));
    }
    return curve;
}

bool closed_coincident_fixture_loops(const BRep& shape) {
    for (int face = 0; face < shape.face_count(); ++face)
        for (const BRepRef& wire : shape.m_faces[face].wires) {
            const std::vector<BRepRef> uses = shape.wire_edges(wire);
            if (uses.empty())
                return false;
            std::vector<std::pair<Point, Point>> ends;
            for (const BRepRef& use : uses) {
                const int pc = shape.pcurve_index(use.index, face, use.orientation);
                if (pc < 0)
                    return false;
                const NurbsCurve& curve = shape.m_curves_2d[pc];
                const bool reverse = use.orientation == BRepOrientation::Reversed;
                ends.push_back({reverse ? curve.point_at_end() : curve.point_at_start(),
                    reverse ? curve.point_at_start() : curve.point_at_end()});
            }
            for (size_t i = 0; i < ends.size(); ++i)
                for (int d = 0; d < 2; ++d)
                    if (ends[i].second[d] != ends[(i + 1) % ends.size()].first[d])
                        return false;
        }
    return true;
}

BRep affine_solid(BRep shape, int variant) {
    const std::vector<NurbsSurface> original = shape.m_surfaces;
    for (size_t i = 0; i < original.size(); ++i)
        shape.m_surfaces[i] = affine_support(original[i], variant);
    for (BRepEdge& edge : shape.m_edges)
        for (BRepCurveOnSurface& relation : edge.pcurves) {
            for (int* index : {&relation.curve_2d_index, &relation.curve_2d_index_2})
                if (*index >= 0) {
                    const NurbsCurve mapped = affine_trim(shape.m_curves_2d[*index], original[relation.surface_index], variant);
                    *index = shape.add_curve_2d(mapped);
                }
        }
    const bool odd = ((variant & 1) != 0) ^ ((variant & 2) != 0) ^ ((variant & 4) != 0);
    if (odd) {
        for (BRepEdge& edge : shape.m_edges)
            for (BRepCurveOnSurface& relation : edge.pcurves)
                if (relation.curve_2d_index_2 >= 0)
                    std::swap(relation.curve_2d_index, relation.curve_2d_index_2);
        for (BRepFace& face : shape.m_faces)
            for (BRepRef& wire : face.wires)
                wire.orientation = brep_reverse(wire.orientation);
        for (BRepShell& shell : shape.m_shells)
            for (BRepRef& face : shell.faces)
                face.orientation = brep_reverse(face.orientation);
    }
    return shape;
}

void check_mapped_iso(const NurbsSurface& source, const NurbsSurface& target,
                      const nurbs_coincidence::ParameterMap& map) {
    const std::pair<double, double> u = source.domain(0), v = source.domain(1);
    const double fixed = (v.first + v.second) * .5;
    const NurbsCurve world = source.iso_curve(0, fixed);
    NurbsCurve pcurve = NurbsCurve::create(false, 1, {{u.first, fixed, 0}, {u.second, fixed, 0}});
    MINI_CHECK(pcurve.set_domain(world.domain_start(), world.domain_end()));
    MINI_CHECK(nurbs_coincidence::map_pcurve(map, pcurve));
    MINI_CHECK(curve_surface_intersection::same_parameter(world, pcurve, target, 1e-7)
        == curve_surface_intersection::ParameterMatch::Matched);
}

void check_affine_saddle(const BRep& result, int operation) {
    const bool material = operation < 2;
    MINI_CHECK(manifold_components(result));
    MINI_CHECK(result.solid_count() == (material ? 1 : 0));
    MINI_CHECK(result.shell_count() == (material ? 1 : 0));
    const double volume = result.face_count() ? brep_measure::volume(result, 1e-9) : 0;
    MINI_CHECK(std::abs(volume - (material ? 8 : 0)) < 1e-6);
    for (const BRepEdge& edge : result.m_edges)
        if (!edge.degenerated)
            for (const BRepCurveOnSurface& relation : edge.pcurves)
                for (int index : {relation.curve_2d_index, relation.curve_2d_index_2})
                    if (index >= 0)
                        MINI_CHECK(curve_surface_intersection::same_parameter(result.m_curves_3d[edge.curve_3d_index],
                            result.m_curves_2d[index], result.m_surfaces[relation.surface_index], 1e-7)
                            == curve_surface_intersection::ParameterMatch::Matched);
    for (const Point& point : {Point(0, 0, 0), Point(.5, -.5, .5), Point(-.5, .5, -.5),
                               Point(1.5, 0, 0), Point(0, -1.5, 0), Point(0, 0, 1.5)}) {
        const bool inside = material && std::max({std::abs(point[0]), std::abs(point[1]), std::abs(point[2])}) < 1;
        MINI_CHECK(brep_classify::point(result, warped(point, .25), 1e-7)
            == (inside ? brep_classify::State::Inside : brep_classify::State::Outside));
    }
}

}

MINI_TEST("BRep Native Boolean", "Affine Freeform Coincident Identity And Partitions") {
    const BRep source = saddle_box(.25, {0, 0, 0});
    const BRep partitioned = split_identity_face(split_identity_face(source, 0, .375), 2, .625);
    for (int variant = 0; variant < 8; ++variant)
        for (const BRep& original : {source, partitioned}) {
            const BRep counterpart = affine_solid(original, variant);
            MINI_CHECK(counterpart.is_valid() && counterpart.is_solid());
            MINI_CHECK(manifold_components(counterpart));
            for (int operation = 0; operation < 5; ++operation) {
                const BRep& a = operation == 3 ? counterpart : source;
                const BRep& b = operation == 3 ? source : counterpart;
                check_affine_saddle(brep_boolean::apply(a, b, matrix_operations[operation], 1e-7), operation);
            }
        }
}


MINI_TEST("BRep Native Boolean", "Affine Coincident Basis And Rational Pcurve Proof") {
    NurbsSurface source = saddle_box(.25, {0, 0, 0}).m_surfaces.front();
    MINI_CHECK(source.to_rational());
    for (int i = 0; i < source.cv_count(0); ++i)
        for (int j = 0; j < source.cv_count(1); ++j)
            MINI_CHECK(source.set_weight(i, j, 1 + i + 2 * j));
    for (int variant = 0; variant < 8; ++variant) {
        NurbsSurface target = affine_support(source, variant);
        for (int i = 0; i < target.cv_count(0); ++i)
            for (int j = 0; j < target.cv_count(1); ++j)
                MINI_CHECK(target.set_weight(i, j, target.weight(i, j) * (variant % 2 ? 3 : 2)));
        nurbs_coincidence::ParameterMap forward, inverse;
        MINI_CHECK(nurbs_coincidence::parameter_mapping(source, target, 1e-7, forward));
        MINI_CHECK(nurbs_coincidence::parameter_mapping(target, source, 1e-7, inverse));
        check_mapped_iso(source, target, forward);
        check_mapped_iso(target, source, inverse);
        const std::pair<double, double> u = source.domain(0), v = source.domain(1);
        for (double x : {.125, .375, .875})
            for (double y : {.125, .625, .875}) {
                const Point uv(u.first + x * (u.second - u.first), v.first + y * (v.second - v.first), 0);
                const Point mapped = forward.apply(uv);
                MINI_CHECK(source.point_at(uv[0], uv[1]).distance(target.point_at(mapped[0], mapped[1])) < 1e-12);
                MINI_CHECK(inverse.apply(mapped).distance(uv) < 1e-12);
            }
        NurbsCurve pcurve = NurbsCurve::create(false, 2, {{u.first, v.first, 0},
            {(u.first + u.second) * .5, v.second, 0}, {u.second, v.first, 0}});
        MINI_CHECK(pcurve.to_rational());
        double x, y, z, w;
        MINI_CHECK(pcurve.get_cv_4d(1, x, y, z, w));
        MINI_CHECK(pcurve.set_cv_4d(1, x * 2, y * 2, z * 2, 2));
        NurbsCurve mapped = pcurve;
        MINI_CHECK(nurbs_coincidence::map_pcurve(forward, mapped));
        MINI_CHECK(mapped.domain() == pcurve.domain() && mapped.m_nurbsknot == pcurve.m_nurbsknot);
        for (int i = 0; i <= 8; ++i) {
            const double t = pcurve.domain_start() + (pcurve.domain_end() - pcurve.domain_start()) * i / 8;
            MINI_CHECK(mapped.point_at(t).distance(forward.apply(pcurve.point_at(t))) < 1e-12);
        }
        NurbsSurface wrong_weight = target;
        MINI_CHECK(wrong_weight.set_weight(0, 0, std::nextafter(target.weight(0, 0), std::numeric_limits<double>::infinity())));
        MINI_CHECK(!nurbs_coincidence::parameter_mapping(source, wrong_weight, 1e-7, forward));
        NurbsSurface different_basis = target;
        const std::pair<double, double> domain = target.domain(0);
        MINI_CHECK(different_basis.insert_nurbsknot(0, (domain.first + domain.second) * .5));
        MINI_CHECK(!nurbs_coincidence::parameter_mapping(source, different_basis, 1e-7, forward));
    }
    NurbsSurface span_three = source, span_one = source;
    MINI_CHECK(span_three.set_domain(0, -1, 2) && span_one.set_domain(0, -4, -3));
    nurbs_coincidence::ParameterMap unsupported;
    MINI_CHECK(!nurbs_coincidence::parameter_mapping(span_three, span_one, 1e-7, unsupported));
}


MINI_TEST("BRep Native Boolean", "Affine Coincident Origin Underflow Is Unsupported") {
    const double tiny = std::ldexp(1.0, -600);
    MINI_CHECK(!nurbs_coincidence::exact_products_equal(tiny, tiny, 0, 1));
    MINI_CHECK(nurbs_coincidence::exact_products_equal(0, tiny, 0, 1));
    MINI_CHECK(nurbs_coincidence::exact_products_equal(2, 3, 6, 1));
    const NurbsSurface original = saddle_box(.25, {0, 0, 0}).m_surfaces.front();
    NurbsSurface source = original, target = original;
    MINI_CHECK(source.set_domain(0, tiny, tiny + std::ldexp(1.0, -548)));
    MINI_CHECK(target.set_domain(0, 0, std::ldexp(1.0, -1023)));
    nurbs_coincidence::ParameterMap unsupported;
    MINI_CHECK(!nurbs_coincidence::parameter_mapping(source, target, 1e-7, unsupported));
}

namespace {

NurbsSurface rational_bilinear_identity(const NurbsSurface& source) {
    MINI_CHECK(source.degree(0) == 1 && source.degree(1) == 1);
    MINI_CHECK(source.cv_count(0) == 2 && source.cv_count(1) == 2 && !source.is_rational());
    NurbsSurface result(3, true, 3, 2, 3, 2);
    const std::pair<double, double> u = source.domain(0), v = source.domain(1);
    result.m_nurbsknot[0] = {u.first, u.first, u.second, u.second};
    result.m_nurbsknot[1] = {v.first, v.second};
    for (int j = 0; j < 2; ++j) {
        const Point first = source.get_cv(0, j), last = source.get_cv(1, j);
        MINI_CHECK(result.set_cv_4d(0, j, first[0], first[1], first[2], 1));
        MINI_CHECK(result.set_cv_4d(1, j, first[0] + last[0] * .5,
            first[1] + last[1] * .5, first[2] + last[2] * .5, 1.5));
        MINI_CHECK(result.set_cv_4d(2, j, last[0] * 2, last[1] * 2, last[2] * 2, 2));
    }
    MINI_CHECK(result.is_valid());
    return result;
}

BRep refined_coincident_basis(BRep shape) {
    for (NurbsSurface& surface : shape.m_surfaces) {
        for (int dir = 0; dir < 2; ++dir) {
            const std::pair<double, double> domain = surface.domain(dir);
            MINI_CHECK(surface.increase_degree(dir, surface.degree(dir) + 1));
            MINI_CHECK(surface.insert_nurbsknot(dir, std::midpoint(domain.first, domain.second)));
        }
    }
    return shape;
}

}

MINI_TEST("BRep Native Boolean", "Refined Freeform Coincident Support Basis") {
    for (bool rational : {false, true}) {
        BRep original = saddle_box(.25, {0, 0, 0});
        if (rational)
            for (NurbsSurface& surface : original.m_surfaces)
                surface = rational_bilinear_identity(surface);
        const BRep refined = refined_coincident_basis(original);
        MINI_CHECK(original.is_valid() && original.is_solid());
        MINI_CHECK(refined.is_valid() && refined.is_solid());
        check_affine_saddle(original, 0);
        check_affine_saddle(refined, 0);
        for (size_t i = 0; i < original.m_surfaces.size(); ++i) {
            const NurbsSurface& a = original.m_surfaces[i];
            const NurbsSurface& b = refined.m_surfaces[i];
            MINI_CHECK(a.degree(0) != b.degree(0) && a.cv_count(0) != b.cv_count(0));
            nurbs_coincidence::ParameterMap forward, inverse;
            MINI_CHECK(!nurbs_coincidence::parameter_mapping(a, b, 1e-7, forward));
            MINI_CHECK(!nurbs_coincidence::parameter_mapping(b, a, 1e-7, inverse));
            MINI_CHECK(nurbs_coincidence::common_basis_agreement(a, b, 1e-7));
            MINI_CHECK(nurbs_coincidence::common_basis_agreement(b, a, 1e-7));
            check_mapped_iso(a, b, forward);
            check_mapped_iso(b, a, inverse);
        }
        for (int operation = 0; operation < 5; ++operation) {
            const BRep& a = operation == 3 ? refined : original;
            const BRep& b = operation == 3 ? original : refined;
            check_affine_saddle(brep_boolean::apply(a, b, matrix_operations[operation], 1e-7), operation);
        }
    }
}

MINI_TEST("BRep Native Boolean", "Common Basis Bounded Agreement And Weight Contracts") {
    const NurbsSurface source = saddle_box(.25, {0, 0, 0}).m_surfaces.front();
    NurbsSurface refined = source;
    MINI_CHECK(refined.increase_degree(0, 2));
    MINI_CHECK(refined.insert_nurbsknot(0, .5));
    MINI_CHECK(nurbs_coincidence::common_basis_agreement(source, refined, 1e-7));
    NurbsSurface small = refined, large = refined;
    for (int i = 0; i < refined.cv_count(0); ++i)
        for (int j = 0; j < refined.cv_count(1); ++j) {
            small.cv(i, j)[2] += 1e-10;
            large.cv(i, j)[2] += 2e-7;
        }
    MINI_CHECK(nurbs_coincidence::common_basis_agreement(source, small, 1e-7));
    MINI_CHECK(!nurbs_coincidence::common_basis_agreement(source, large, 1e-7));
    NurbsSurface wrong_domain = refined;
    MINI_CHECK(wrong_domain.set_domain(0, 2, 4));
    MINI_CHECK(!nurbs_coincidence::common_basis_agreement(source, wrong_domain, 1e-7));
    NurbsSurface a = source, b = source;
    MINI_CHECK(a.to_rational() && b.to_rational());
    for (int i = 0; i < a.cv_count(0); ++i)
        for (int j = 0; j < a.cv_count(1); ++j) {
            MINI_CHECK(a.set_weight(i, j, 3));
            MINI_CHECK(b.set_weight(i, j, 2));
        }
    MINI_CHECK(a.set_weight(0, 1, 1));
    MINI_CHECK(b.set_weight(0, 1, 2.0 / 3));
    MINI_CHECK(a.weight(0, 1) / a.weight(0, 0) == b.weight(0, 1) / b.weight(0, 0));
    MINI_CHECK(!nurbs_coincidence::exact_products_equal(a.weight(0, 1), b.weight(0, 0),
        b.weight(0, 1), a.weight(0, 0)));
    MINI_CHECK(!nurbs_coincidence::same_parameters(a, b, 1e-7));
    nurbs_coincidence::ParameterMap strict;
    MINI_CHECK(!nurbs_coincidence::parameter_mapping(a, b, 1e-7, strict));
    MINI_CHECK(!nurbs_coincidence::common_basis_agreement(a, b, 1e-7));
}

namespace {

BRep void_operand_box(double size, double curvature) {
    BRep result = BRep::create_box(size, size, size);
    for (BRepVertex& vertex : result.m_vertices)
        vertex.point = warped(vertex.point, curvature);
    for (NurbsCurve& curve : result.m_curves_3d)
        for (int i = 0; i < curve.cv_count(); ++i)
            MINI_CHECK(curve.set_cv(i, warped(curve.get_cv(i), curvature)));
    for (NurbsSurface& surface : result.m_surfaces)
        for (int i = 0; i < surface.cv_count(0); ++i)
            for (int j = 0; j < surface.cv_count(1); ++j)
                MINI_CHECK(surface.set_cv(i, j, warped(surface.get_cv(i, j), curvature)));
    return result;
}

void check_void_boundary(const BRep& result) {
    MINI_CHECK(manifold_components(result));
    std::set<int> edges, vertices;
    for (const BRepFace& face : result.m_faces)
        for (const BRepRef& wire : face.wires)
            for (const BRepRef& edge : result.wire_edges(wire)) {
                edges.insert(edge.index);
                vertices.insert(result.m_edges[edge.index].start_vertex);
                vertices.insert(result.m_edges[edge.index].end_vertex);
            }
    MINI_CHECK(edges.size() == result.m_edges.size());
    MINI_CHECK(vertices.size() == result.m_vertices.size());
    for (const BRepEdge& edge : result.m_edges) {
        MINI_CHECK(!edge.pcurves.empty());
        for (const BRepCurveOnSurface& relation : edge.pcurves)
            for (int index : {relation.curve_2d_index, relation.curve_2d_index_2})
                if (index >= 0)
                    MINI_CHECK(curve_surface_intersection::same_parameter(
                        result.m_curves_3d[edge.curve_3d_index], result.m_curves_2d[index],
                        result.m_surfaces[relation.surface_index], 1e-7)
                        == curve_surface_intersection::ParameterMatch::Matched);
    }
    if (result.face_count() == 0)
        MINI_CHECK(result.m_wires.empty() && result.m_vertices.empty());
}

nlohmann::ordered_json check_void_shell_volumes(const BRep& result, std::vector<double> expected,
                             std::vector<double> expected_components) {
    std::vector<double> actual;
    for (const BRepSolid& solid : result.m_solids)
        for (const BRepRef& shell : solid.shells) {
            BRep isolated = result;
            isolated.m_solids.clear();
            isolated.add_solid({shell});
            actual.push_back(brep_measure::volume(isolated, 1e-9));
        }
    std::vector<double> components;
    for (const BRepSolid& solid : result.m_solids) {
        BRep isolated = result;
        isolated.m_solids = {solid};
        components.push_back(brep_measure::volume(isolated, 1e-9));
    }
    std::sort(components.begin(), components.end());
    std::sort(expected_components.begin(), expected_components.end());
    MINI_CHECK(components.size() == expected_components.size());
    for (size_t i = 0; i < std::min(components.size(), expected_components.size()); ++i)
        MINI_CHECK(std::abs(components[i] - expected_components[i]) < 1e-6);
    std::sort(actual.begin(), actual.end());
    std::sort(expected.begin(), expected.end());
    MINI_CHECK(actual.size() == expected.size());
    for (size_t i = 0; i < std::min(actual.size(), expected.size()); ++i)
        MINI_CHECK(std::abs(actual[i] - expected[i]) < 1e-6);
    return {{"expected_signed_shells", expected}, {"actual_signed_shells", actual},
        {"expected_components", expected_components}, {"actual_components", components}};
}

nlohmann::ordered_json check_void_occupancy(const BRep& result, int operation, bool fill, double curvature) {
    nlohmann::ordered_json probes = nlohmann::ordered_json::array();
    for (const Point& point : {Point(0, 0, 0), Point(.75, 0, 0), Point(1.5, 0, 0),
            Point(2.5, 0, 0), Point(.25, -.25, .25), Point(.8, -.7, .6),
            Point(1.4, -.8, 1.3), Point(-1.4, 1.3, -.8), Point(0, 0, 2.5)}) {
        const double radius = std::max({std::abs(point[0]), std::abs(point[1]), std::abs(point[2])});
        const bool a = radius < 2 && radius > 1;
        const bool b = radius < (fill ? 1 : .5);
        const bool expected = operation == 3 ? b && !a : truth(a, b, matrix_operations[operation]);
        const Point query = warped(point, curvature);
        const brep_classify::State actual = brep_classify::point(result, query, 1e-7);
        MINI_CHECK(actual == (expected ? brep_classify::State::Inside : brep_classify::State::Outside));
        probes.push_back({{"point", {query[0], query[1], query[2]}}, {"operand_a_inside", a},
            {"operand_b_inside", b}, {"expected_inside", expected}, {"actual_inside", actual == brep_classify::State::Inside}});
    }
    MINI_CHECK(probes.size() == 9);
    return probes;
}

void check_void_operation(const BRep& cavity, const BRep& other, int operation,
                          bool fill, double curvature) {
    const std::string name = std::string("native_void_") + (curvature == 0 ? "flat_" : "curved_")
        + (fill ? "fill_" : "island_") + matrix_names[operation];
    const double size = fill ? 8 : 1;
    const std::array<double, 5> volumes = {0, 56 + size, 56, size, 56 + size};
    const std::array<int, 5> solids = {0, fill ? 1 : 2, 1, 1, fill ? 1 : 2};
    const std::array<int, 5> shells = {0, fill ? 1 : 3, 2, 1, fill ? 1 : 3};
    nlohmann::ordered_json record = {{"operation", matrix_names[operation]}, {"passed", false},
        {"expected_volume", volumes[operation]}, {"expected_solids", solids[operation]},
        {"expected_shells", shells[operation]}, {"curvature", curvature}, {"fill", fill}};
    if (operation == 0 || (fill && (operation == 1 || operation == 4)))
        record["expected_faces"] = operation == 0 ? 0 : 6;
    try {
        const BRep result = operation == 3
            ? brep_boolean::apply(other, cavity, matrix_operations[operation], 1e-7)
            : brep_boolean::apply(cavity, other, matrix_operations[operation], 1e-7);
        record["faces"] = result.face_count();
        record["solids"] = result.solid_count();
        record["shells"] = result.shell_count();
        if (result.face_count()) {
            save(name, result);
            record["session_file"] = name + ".pb";
        }
        check_void_boundary(result);
        MINI_CHECK(result.solid_count() == solids[operation] && result.shell_count() == shells[operation]);
        const double volume = result.face_count() ? brep_measure::volume(result, 1e-9) : 0;
        record["volume"] = volume;
        MINI_CHECK(std::abs(volume - volumes[operation]) < 1e-6);
        std::vector<double> signed_shells, components;
        if (operation == 2) {
            signed_shells = {-8, 64};
            components = {56};
        } else if (operation == 3) {
            signed_shells = {size};
            components = {size};
        } else if (operation != 0) {
            signed_shells = fill ? std::vector<double>{64} : std::vector<double>{-8, size, 64};
            components = fill ? std::vector<double>{64} : std::vector<double>{size, 56};
        }
        record["volume_checks"] = check_void_shell_volumes(result, signed_shells, components);
        if (fill && (operation == 1 || operation == 4))
            MINI_CHECK(result.face_count() == 6);
        record["probes"] = check_void_occupancy(result, operation, fill, curvature);
        record["topology_vertex_links_full_pcurves"] = true;
        record["passed"] = true;
        write_matrix(name, record);
    } catch (const std::exception& error) {
        record["passed"] = false;
        record["failure"] = error.what();
        write_matrix(name, record);
        throw;
    }
}

void check_void_operand_matrix(double curvature) {
    const BRep outer = void_operand_box(4, curvature);
    const BRep inner = void_operand_box(2, curvature);
    const BRep cavity = brep_boolean::apply(outer, inner, brep_boolean::Operation::Difference, 1e-7);
    check_void_boundary(cavity);
    MINI_CHECK(cavity.solid_count() == 1 && cavity.shell_count() == 2);
    check_void_shell_volumes(cavity, {-8, 64}, {56});
    save(curvature == 0 ? "native_void_flat_operand" : "native_void_curved_operand", cavity);
    for (bool fill : {false, true}) {
        const BRep other = fill ? inner : void_operand_box(1, curvature);
        for (int operation = 0; operation < 5; ++operation)
            check_void_operation(cavity, other, operation, fill, curvature);
    }
}

}

MINI_TEST("BRep Native Boolean", "Void Operand Island And Exact Cavity Fill") {
    check_void_operand_matrix(0);
}

MINI_TEST("BRep Native Boolean", "Doubly Curved Void Operand Island And Exact Fill") {
    check_void_operand_matrix(.25);
}


MINI_TEST("BRep Native Boolean", "Reversed Generator Periodic Coincident Owner") {
    const BRep original = BRep::create_cylinder(1, 2);
    const BRep reversed = affine_solid(original, 2);
    check_void_boundary(original);
    check_void_boundary(reversed);
    MINI_CHECK(closed_coincident_fixture_loops(original));
    MINI_CHECK(closed_coincident_fixture_loops(reversed));
    MINI_CHECK(original.solid_count() == 1 && reversed.solid_count() == 1);
    for (const BRep* shape : {&original, &reversed})
        MINI_CHECK(std::abs(brep_measure::volume(*shape, 1e-9) - 2 * Tolerance::PI) < 1e-6);
    for (const BRep* shape : {&original, &reversed})
        for (const Point& point : {Point(.3, .2, 1), Point(-.3, .2, 1), Point(-.3, -.2, 1),
                Point(.3, -.2, 1), Point(1.3, .2, 1), Point(.2, .3, -1), Point(.2, .3, 3)}) {
            const bool inside = point[0] * point[0] + point[1] * point[1] < 1
                && point[2] > 0 && point[2] < 2;
            MINI_CHECK(brep_classify::point(*shape, point, 1e-7)
                == (inside ? brep_classify::State::Inside : brep_classify::State::Outside));
        }
    for (int operation = 0; operation < 5; ++operation) {
        const BRep result = operation == 3
            ? brep_boolean::apply(reversed, original, matrix_operations[operation], 1e-7)
            : brep_boolean::apply(original, reversed, matrix_operations[operation], 1e-7);
        const bool material = operation < 2;
        check_void_boundary(result);
        MINI_CHECK(result.solid_count() == (material ? 1 : 0));
        MINI_CHECK(result.shell_count() == (material ? 1 : 0));
        MINI_CHECK(result.face_count() == (material ? 3 : 0));
        const double volume = material ? brep_measure::volume(result, 1e-9) : 0;
        MINI_CHECK(std::abs(volume - (material ? 2 * Tolerance::PI : 0)) < 1e-6);
        for (const Point& point : {Point(.3, .2, 1), Point(-.3, -.2, 1), Point(1.3, .2, 1)}) {
            const bool inside = material && point[0] * point[0] + point[1] * point[1] < 1;
            MINI_CHECK(brep_classify::point(result, point, 1e-7)
                == (inside ? brep_classify::State::Inside : brep_classify::State::Outside));
        }
    }
}

MINI_TEST("BRep Native Boolean", "Refined Rational Boundary Curve Basis") {
    const BRep original = BRep::create_cylinder(1, 2);
    BRep refined = original;
    bool nonuniform_rational = false;
    for (NurbsCurve& curve : refined.m_curves_3d) {
        for (int i = 1; i < curve.cv_count(); ++i)
            nonuniform_rational |= curve.weight(i) != curve.weight(0);
        const int degree = curve.degree();
        MINI_CHECK(curve.increase_degree(degree + 1));
        MINI_CHECK(curve.insert_nurbsknot(curve.domain_start()
            + .375 * (curve.domain_end() - curve.domain_start())));
        MINI_CHECK(curve.degree() == degree + 1);
    }
    for (NurbsCurve& curve : refined.m_curves_2d) {
        const int degree = curve.degree();
        MINI_CHECK(curve.increase_degree(degree + 2));
        MINI_CHECK(curve.insert_nurbsknot(curve.domain_start()
            + .625 * (curve.domain_end() - curve.domain_start())));
        MINI_CHECK(curve.degree() == degree + 2);
    }
    MINI_CHECK(nonuniform_rational);
    check_void_boundary(original);
    check_void_boundary(refined);
    for (int operation = 0; operation < 5; ++operation) {
        const BRep result = operation == 3
            ? brep_boolean::apply(refined, original, matrix_operations[operation], 1e-7)
            : brep_boolean::apply(original, refined, matrix_operations[operation], 1e-7);
        const bool material = operation < 2;
        check_void_boundary(result);
        MINI_CHECK(result.solid_count() == (material ? 1 : 0));
        MINI_CHECK(result.shell_count() == (material ? 1 : 0));
        MINI_CHECK(result.face_count() == (material ? 3 : 0));
        const double volume = material ? brep_measure::volume(result, 1e-9) : 0;
        MINI_CHECK(std::abs(volume - (material ? 2 * Tolerance::PI : 0)) < 1e-6);
        for (const Point& point : {Point(.3, .2, 1), Point(-.3, -.2, 1), Point(1.3, .2, 1)}) {
            const bool inside = material && point[0] * point[0] + point[1] * point[1] < 1;
            MINI_CHECK(brep_classify::point(result, point, 1e-7)
                == (inside ? brep_classify::State::Inside : brep_classify::State::Outside));
        }
    }
}

namespace {
void event_progress(bool affine, size_t operation, const char* phase) {
    if (!std::getenv("SESSION_BOOLEAN_PROGRESS"))
        return;
    std::fprintf(stderr, "triple_progress affine=%d operation=%zu phase=%s\n", affine, operation, phase);
    std::fflush(stderr);
}

void event_boundary(const BRep& shape) {
    MINI_CHECK(manifold_components(shape));
    for (const BRepShell& shell : shape.m_shells)
        MINI_CHECK(brep_acceptance::shell_vertex_links(shape, shell) == brep_acceptance::VertexLinks::Matched);
    for (const BRepEdge& edge : shape.m_edges)
        for (const BRepCurveOnSurface& relation : edge.pcurves)
            for (int pc : {relation.curve_2d_index, relation.curve_2d_index_2})
                if (pc >= 0)
                    MINI_CHECK(curve_surface_intersection::same_parameter(shape.m_curves_3d[edge.curve_3d_index],
                        shape.m_curves_2d[pc], shape.m_surfaces[relation.surface_index], 1e-6)
                        == curve_surface_intersection::ParameterMatch::Matched);
}
BRep event_warped_box(bool affine) {
    BRep shape = BRep::create_box(2, 2, 2);
    const auto warp = [](const Point& p) {
        return Point(p[0] + p[1] * p[2] / 16, p[1] + p[0] * p[2] / 16,
            p[2] + p[0] * p[1] / 16);
    };
    for (BRepVertex& vertex : shape.m_vertices)
        vertex.point = warp(vertex.point);
    for (NurbsCurve& curve : shape.m_curves_3d)
        for (int i = 0; i < curve.cv_count(); ++i)
            MINI_CHECK(curve.set_cv(i, warp(curve.get_cv(i))));
    for (NurbsSurface& surface : shape.m_surfaces) {
        for (int i = 0; i < surface.cv_count(0); ++i)
            for (int j = 0; j < surface.cv_count(1); ++j)
                MINI_CHECK(surface.set_cv(i, j, warp(surface.get_cv(i, j))));
        MINI_CHECK(!surface.is_planar());
        if (affine) {
            const auto u = surface.domain(0), v = surface.domain(1);
            MINI_CHECK(surface.set_domain(0, 8 * u.first + 32, 8 * u.second + 32));
            MINI_CHECK(surface.set_domain(1, .25 * v.first - 4, .25 * v.second - 4));
        }
    }
    if (affine)
        for (NurbsCurve& pc : shape.m_curves_2d)
            for (int i = 0; i < pc.cv_count(); ++i) {
                pc.cv(i)[0] = 8 * pc.cv(i)[0] + 32 * pc.weight(i);
                pc.cv(i)[1] = .25 * pc.cv(i)[1] - 4 * pc.weight(i);
            }
    event_boundary(shape);
    return shape;
}
}
namespace {
nlohmann::json self_event_points(const std::vector<curve_intersection::Hit>& events) {
    nlohmann::json result = nlohmann::json::array();
    for (const curve_intersection::Hit& hit : events)
        result.push_back({{"a", hit.a}, {"b", hit.b}, {"tangent", hit.tangent},
            {"point", {hit.point[0], hit.point[1], hit.point[2]}}});
    return result;
}

void save_self_event(const split::SelfIntersectionError& error, const BRep& a, const BRep& b,
    bool affine, size_t operation, bool swapped) {
    nlohmann::json overlaps = nlohmann::json::array();
    for (const curve_intersection::Overlap& overlap : error.events.overlaps)
        overlaps.push_back({overlap.a0, overlap.a1, overlap.b0, overlap.b1});
    const nlohmann::json witness = {{"failure", error.what()}, {"affine", affine},
        {"operation_index", operation}, {"swapped", swapped},
        {"original_a", a.jsondump()}, {"original_b", b.jsondump()},
        {"world", error.world.jsondump()}, {"pcurve", error.pcurve.jsondump()},
        {"source", error.source}, {"source_edge", error.source_edge}, {"section", error.section},
        {"tolerance", error.tolerance}, {"complete", error.events.complete},
        {"events_failure", error.events.failure}, {"cells", error.events.cells},
        {"has_failure_pair", error.events.has_failure_pair},
        {"failure_pair", error.events.failure_pair}, {"failure_depth", error.events.failure_depth},
        {"points", self_event_points(error.events.points)},
        {"endpoints", self_event_points(error.events.endpoints)}, {"overlaps", overlaps}};
    const std::filesystem::path root("build/native-geometry-tests/viewer");
    std::filesystem::create_directories(root);
    const std::string label = "native_triple_self_overlap_" + std::to_string(affine)
        + "_" + std::to_string(operation) + "_"
        + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    const std::filesystem::path folder = root / label;
    if (!std::filesystem::create_directory(folder))
        throw std::runtime_error("Triple self-intersection failure directory already exists");
    std::ofstream output(folder / "failure.json", std::ios::binary);
    if (!output || !(output << witness.dump(2)))
        throw std::runtime_error("Cannot write triple self-intersection failure evidence");
    output.flush();
    if (!output)
        throw std::runtime_error("Cannot flush triple self-intersection failure evidence");
    output.close();
    if (!output)
        throw std::runtime_error("Cannot close triple self-intersection failure evidence");
}
}

MINI_TEST("BRep Native Boolean", "Persistent Triple Face Boundary Events") {
    const std::array<brep_boolean::Operation, 5> event_operations = {
        brep_boolean::Operation::Intersection, brep_boolean::Operation::Union,
        brep_boolean::Operation::Difference, brep_boolean::Operation::Difference, brep_boolean::Operation::Xor};
    const std::array<double, 5> volumes = {3.984375, 35.984375, 3.984375, 28.015625, 32};
    const std::array<std::array<bool, 4>, 5> occupancy = {{{true, false, false, false},
        {true, true, true, false}, {false, true, false, false}, {false, false, true, false},
        {false, true, true, false}}};
    const std::array<Point, 4> probes = {Point(0, 0, -.5), Point(0, 0, .5), Point(1.5, 0, -.5), Point(3, 0, 0)};
    for (bool affine : {false, true}) {
        const BRep a = event_warped_box(affine);
        BRep b = BRep::create_box(4, 4, 2);
        b.transform(Xform::translation(0, 0, -1));
        event_boundary(b);
        MINI_CHECK(std::abs(a.volume_exact() - 7.96875) < 1e-6);
        for (size_t operation = 0; operation < event_operations.size(); ++operation) {
            const bool swapped = operation == 3;
            event_progress(affine, operation, "apply");
            BRep result;
            try {
                result = brep_boolean::apply(swapped ? b : a, swapped ? a : b, event_operations[operation], 1e-6);
            } catch (const split::SelfIntersectionError& error) {
                try {
                    save_self_event(error, a, b, affine, operation, swapped);
                } catch (const std::exception& capture_error) {
                    std::cerr << "Triple self-intersection capture failed: " << capture_error.what() << '\n';
                } catch (...) {
                    std::cerr << "Triple self-intersection capture failed with unknown error\n";
                }
                throw;
            }
            event_progress(affine, operation, "boundary");
            event_boundary(result);
            event_progress(affine, operation, "volume");
            MINI_CHECK(std::abs(result.volume_exact() - volumes[operation]) < 1e-5);
            event_progress(affine, operation, "classify");
            for (size_t i = 0; i < probes.size(); ++i)
                MINI_CHECK(brep_classify::point(result, probes[i], 1e-6)
                    == (occupancy[operation][i] ? brep_classify::State::Inside : brep_classify::State::Outside));
            event_progress(affine, operation, "corner_vertices");
            if (operation == 0)
                for (double x : {-255.0 / 256, 255.0 / 256})
                    for (double y : {-255.0 / 256, 255.0 / 256}) {
                        int matches = 0;
                        for (const BRepVertex& vertex : result.m_vertices)
                            matches += vertex.point.distance({x, y, 0}) <= 1e-6;
                        MINI_CHECK(matches == 1);
                    }
        }
    }
}


namespace {
BRep public_boolean_operation(const BRep& a, const BRep& b, int operation, double tolerance = 1e-7) {
    switch (operation) {
        case 0: return a.boolean_union(b, tolerance);
        case 1: return a.boolean_intersection(b, tolerance);
        case 2: return a.boolean_difference(b, tolerance);
        case 3: return b.boolean_difference(a, tolerance);
        case 4: return a.boolean_xor(b, tolerance);
    }
    throw std::invalid_argument("Unknown public Boolean test operation");
}

void public_boolean_analytic_matrix(double curvature) {
    // Existing exact tensor fixture: Phi(x,y,z)=(x,y,z+c*x*y), det(DPhi)=1.
    // A=[-1,1]^3, B=[-.5,1.5]^3 before Phi. No internal Boolean is an oracle.
    const BRep a = saddle_box(curvature, {0,0,0});
    const BRep b = saddle_box(curvature, {.5,.5,.5});
    MINI_CHECK(a.is_valid() && b.is_valid() && a.is_solid() && b.is_solid());
    if (curvature != 0) {
        int curved = 0;
        for (const auto& surface : a.m_surfaces)
            curved += !surface.is_planar(nullptr, 1e-7);
        MINI_CHECK(curved >= 2); // Both retained top/bottom are doubly curved saddles.
    }
    const auto original_a = a.jsondump(), original_b = b.jsondump();
    const std::array<double,5> volumes = {12.625, 3.375, 4.625, 4.625, 9.25};
    const std::array<Point,12> probes = {
        Point(.25,.25,.25), Point(-.75,0,0), Point(1.25,.5,.5),
        Point(.25,.25,-.75), Point(.25,.25,1.25), Point(.25,-.75,.25),
        Point(.25,1.25,.25), Point(1.75,.5,.5), Point(-1.25,0,0),
        Point(0,-1.25,0), Point(0,0,-1.25), Point(.25,.25,1.75)};
    for (int operation = 0; operation < 5; ++operation) {
        const BRep result = public_boolean_operation(a,b,operation);
        MINI_CHECK(result.face_count() > 0 && result.is_valid() && result.is_solid());
        MINI_CHECK(manifold_components(result)); // Balanced edge incidence + connected shells + vertex links.
        MINI_CHECK(result.solid_count() == (operation == 4 ? 2 : 1));
        MINI_CHECK(result.shell_count() == (operation == 4 ? 2 : 1));
        const double volume = brep_measure::volume(result,1e-8);
        MINI_CHECK(volume > 0);
        MINI_CHECK(std::abs(volume-volumes[operation]) <= 1e-6*volumes[operation]);
        for (const auto& edge : result.m_edges)
            if (!edge.degenerated)
                for (const auto& relation : edge.pcurves)
                    for (int pc : {relation.curve_2d_index,relation.curve_2d_index_2})
                        if (pc >= 0)
                            MINI_CHECK(curve_surface_intersection::same_parameter(
                                result.m_curves_3d[edge.curve_3d_index],result.m_curves_2d[pc],
                                result.m_surfaces[relation.surface_index],1e-7)
                                == curve_surface_intersection::ParameterMatch::Matched);
        for (const Point& p : probes) {
            const bool in_a = p[0]>-1 && p[0]<1 && p[1]>-1 && p[1]<1 && p[2]>-1 && p[2]<1;
            const bool in_b = p[0]>-.5 && p[0]<1.5 && p[1]>-.5 && p[1]<1.5 && p[2]>-.5 && p[2]<1.5;
            const bool expected = operation == 0 ? in_a||in_b : operation == 1 ? in_a&&in_b
                : operation == 2 ? in_a&&!in_b : operation == 3 ? in_b&&!in_a : in_a!=in_b;
            MINI_CHECK(brep_classify::point(result,warped(p,curvature),1e-7)
                == (expected ? brep_classify::State::Inside : brep_classify::State::Outside));
        }
        MINI_CHECK(a.jsondump() == original_a && b.jsondump() == original_b);
    }
}
}

MINI_TEST("BRep Public Boolean", "Overlapping Boxes Analytic Five Operations") {
    public_boolean_analytic_matrix(0);
}

MINI_TEST("BRep Public Boolean", "Doubly Curved Volume Preserving Five Operations") {
    public_boolean_analytic_matrix(.25);
}

MINI_TEST("BRep Public Boolean", "Opposite Saddles Preserve Critical Contact Material Lobes") {
    // Two independently sheared boxes, NOT a shared change of coordinates.
    // A: |z-.25*x*y|<1; B: |z+.25*x*y|<1; |x|,|y|<1.
    // Their top/bottom intersections xy=0 cross at an interior tangent point.
    const BRep a=saddle_box(.25,{0,0,0}),b=saddle_box(-.25,{0,0,0});
    check_void_boundary(a); check_void_boundary(b);
    for(const BRep* shape : {&a,&b}) {
        int curved=0;
        for(const auto& surface : shape->m_surfaces) curved+=!surface.is_planar(nullptr,1e-7);
        MINI_CHECK(curved==2);
        MINI_CHECK(std::abs(shape->volume_exact(1e-8)-8)<1e-6);
    }
    const auto before_a=a.jsondump(),before_b=b.jsondump();
    const std::array<double,5> volumes{8.5,7.5,.5,.5,1};
    const std::array<int,5> components{1,1,4,4,8};
    std::vector<Point> probes{Point(),Point(0,0,1.5),Point(1.25,0,0)};
    for(double x : {-.5,.5}) for(double y : {-.5,.5})
        for(double z : {-1.,1.}) probes.emplace_back(x,y,z);
    const auto inside=[](const Point& p,double c) {
        return std::abs(p[0])<1 && std::abs(p[1])<1 && std::abs(p[2]-c*p[0]*p[1])<1;
    };
    for(const auto& point : probes) {
        MINI_CHECK(brep_classify::point(a,point,1e-7)==
            (inside(point,.25)?brep_classify::State::Inside:brep_classify::State::Outside));
        MINI_CHECK(brep_classify::point(b,point,1e-7)==
            (inside(point,-.25)?brep_classify::State::Inside:brep_classify::State::Outside));
    }
    for(int operation=0;operation<5;++operation) {
        const BRep result=public_boolean_operation(a,b,operation);
        check_void_boundary(result);
        MINI_CHECK(result.is_valid() && result.is_solid());
        MINI_CHECK(result.solid_count()==components[operation] && result.shell_count()==components[operation]);
        const double volume=result.volume_exact(1e-8);
        MINI_CHECK(volume>0 && std::abs(volume-volumes[operation])<1e-6);
        for(const auto& solid : result.m_solids) {
            BRep component=result;
            component.m_solids={solid};
            const double expected=operation<2?volumes[operation]:.125;
            MINI_CHECK(std::abs(component.volume_exact(1e-8)-expected)<1e-6);
        }
        for(const auto& point : probes) {
            const bool ia=inside(point,.25),ib=inside(point,-.25);
            const bool expected=operation==0?ia||ib:operation==1?ia&&ib:
                operation==2?ia&&!ib:operation==3?ib&&!ia:ia!=ib;
            MINI_CHECK(brep_classify::point(result,point,1e-7)==
                (expected?brep_classify::State::Inside:brep_classify::State::Outside));
        }
        MINI_CHECK(a.jsondump()==before_a && b.jsondump()==before_b);
    }
}

namespace {
void rhino_flat_result_pcurves(const BRep& shape, double tolerance) {
    size_t images = 0;
    for (const auto& edge : shape.m_edges) {
        MINI_CHECK(!edge.pcurves.empty());
        NurbsCurve world;
        if (edge.degenerated) {
            MINI_CHECK(edge.start_vertex == edge.end_vertex);
            const auto& point = shape.m_vertices.at(edge.start_vertex).point;
            world = NurbsCurve(3, false, 2, 2);
            world.m_nurbsknot = {0, 1};
            MINI_CHECK(world.set_cv(0, point));
            MINI_CHECK(world.set_cv(1, point));
        } else world = shape.m_curves_3d.at(edge.curve_3d_index);
        for (const auto& relation : edge.pcurves) {
            MINI_CHECK(relation.curve_2d_index >= 0);
            for (int pc : {relation.curve_2d_index, relation.curve_2d_index_2}) {
                if (pc < 0) continue;
                MINI_CHECK(curve_surface_intersection::same_parameter(
                    world, shape.m_curves_2d.at(pc),
                    shape.m_surfaces.at(relation.surface_index), tolerance) ==
                    curve_surface_intersection::ParameterMatch::Matched);
                ++images;
            }
        }
    }
    MINI_CHECK(images > 0);
}

void rhino_public_fixture_matrix(const std::string& family, bool union_only = false) {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path() /
                      "tests" / "fixtures" / "rhino_booleans";
    std::ifstream stream(root / "compact-reference.json");
    MINI_CHECK(stream.good());
    nlohmann::json reference;
    stream >> reference;
    const auto found = std::find_if(reference["families"].begin(), reference["families"].end(),
        [&](const auto& item) { return item["family"] == family; });
    MINI_CHECK(found != reference["families"].end());
    const auto& oracle = *found;
    const double tolerance = oracle["model_absolute_tolerance"].get<double>();
    MINI_CHECK(tolerance == 0.001);
    MINI_CHECK(oracle["compact_probes"].size() == 32);
    const auto read_operand = [&](int operand) {
        const auto& identity = oracle["input_order"][operand];
        MINI_CHECK(identity["object_table_index"] == operand);
        const auto shapes = file_step::read_file_step_breps(
            (root / identity["qualified_step"].get<std::string>()).string(), tolerance);
        MINI_CHECK(shapes.size() == 1);
        return shapes.at(0);
    };
    const BRep a = read_operand(0), b = read_operand(1);
    MINI_CHECK(a.is_valid() && b.is_valid() && a.is_solid() && b.is_solid());
    MINI_CHECK(manifold_components(a) && manifold_components(b));
    rhino_flat_result_pcurves(a, tolerance);
    rhino_flat_result_pcurves(b, tolerance);
    const auto original_a = a.jsondump(), original_b = b.jsondump();
    const double va = oracle["input_order"][0]["signed_volume"].get<double>();
    const double vb = oracle["input_order"][1]["signed_volume"].get<double>();
    const auto output = [&](const std::string& operation) -> const nlohmann::json& {
        const auto& outputs = oracle["output_references"];
        const auto it = std::find_if(outputs.begin(), outputs.end(),
            [&](const auto& item) { return item["operation"] == operation; });
        MINI_CHECK(it != outputs.end());
        return *it;
    };
    const double original_union = output("union")["reference"]["signed_volume"].get<double>();
    std::array<double, 5> expected{};
    if (union_only) {
        // No Intersection/Difference/XOR oracle is constructed for Spheres.
        expected[0] = original_union;
    } else {
        const double vi = output("intersection")["reference"]["signed_volume"].get<double>();
        expected = {va + vb - vi, vi, va - vi, vb - vi, va + vb - 2 * vi};
    }
    const double vu = output("union")["reference"]["signed_volume"].get<double>();
    MINI_CHECK(std::abs(vu - expected[0]) <= 1e-6 * expected[0]);
    if (!union_only) {
        const auto& difference = output("difference");
        const auto order = difference["difference_order"].get<std::string>();
        MINI_CHECK(order == "ab" || order == "ba");
        const double vd = difference["reference"]["signed_volume"].get<double>();
        MINI_CHECK(std::abs(vd - expected[order == "ab" ? 2 : 3]) <= 1e-6 * vd);
    }
    // Rhino's curved outputs miss the true volume by more than this gate; exact-volumes.json says why and how it was found.
    double gate = 1e-6;
    std::ifstream exact_stream(root / "exact-volumes.json");
    MINI_CHECK(exact_stream.good());
    nlohmann::json exact_volumes;
    exact_stream >> exact_volumes;
    if (exact_volumes["families"].contains(family)) {
        const auto& exact = exact_volumes["families"][family];
        const double vi = exact["exact_intersection"].get<double>();
        expected = {va + vb - vi, vi, va - vi, vb - vi, va + vb - 2 * vi};
        gate = exact["gate"].get<double>();
    }
    const std::array<std::string, 5> labels = {"union", "intersection", "difference_ab", "difference_ba", "xor"};
    for (int operation = 0; operation < (union_only ? 1 : 5); ++operation) {
        const BRep result = public_boolean_operation(a, b, operation, tolerance);
        // Save only the actual native result, including a result failing a later gate.
        save("rhino_" + family + "_" + labels[operation], result);
        MINI_CHECK(result.face_count() > 0 && result.is_valid() && result.is_solid());
        MINI_CHECK(manifold_components(result));
        rhino_flat_result_pcurves(result, tolerance);
        const double volume = brep_measure::volume(result, 1e-8);
        MINI_CHECK(std::isfinite(volume) && volume > 0);
        MINI_CHECK(std::abs(volume - expected[operation]) <= gate * expected[operation]);
        for (const auto& probe : oracle["compact_probes"]) {
            const bool in_a = probe["states"][0].get<int>() == 1;
            const bool in_b = probe["states"][1].get<int>() == 1;
            const bool inside = operation == 0 ? in_a || in_b : operation == 1 ? in_a && in_b
                : operation == 2 ? in_a && !in_b : operation == 3 ? in_b && !in_a : in_a != in_b;
            const auto& xyz = probe["point"];
            MINI_CHECK(brep_classify::point(result,
                Point(xyz[0].get<double>(), xyz[1].get<double>(), xyz[2].get<double>()), tolerance) ==
                (inside ? brep_classify::State::Inside : brep_classify::State::Outside));
        }
        MINI_CHECK(a.jsondump() == original_a && b.jsondump() == original_b);
    }
}
}
MINI_TEST("BRep Public Boolean", "Rhino Boxes Five Operations Original Oracle") {
    rhino_public_fixture_matrix("Boxes");
}
MINI_TEST("BRep Public Boolean", "Rhino Pyramids Five Operations Original Oracle") {
    rhino_public_fixture_matrix("Pyramids");
}

MINI_TEST("BRep Public Boolean", "Rhino Cones Five Operations Original Oracle") {
    rhino_public_fixture_matrix("Cones");
}

MINI_TEST("BRep Public Boolean", "Rhino Cylinders Five Operations Original Oracle") {
    rhino_public_fixture_matrix("Cylinders");
}

MINI_TEST("BRep Public Boolean", "Rhino Torus Five Operations Original Oracle") {
    rhino_public_fixture_matrix("Torus");
}

MINI_TEST("BRep Public Boolean", "Rhino Twisted Five Operations Original Oracle") {
    rhino_public_fixture_matrix("Twisted");
}

MINI_TEST("BRep Public Boolean", "Rhino Spheres Union Original Oracle") {
    rhino_public_fixture_matrix("Spheres", true);
}

namespace {

using CutPlan = std::vector<std::array<double, 2>>;

/// Area of a closed plan inside a circle about the origin: each side's triangle with the centre, clipped to the circle.
double plan_area_in_circle(const CutPlan& plan, double radius) {
    const auto triangle = [](const std::array<double, 2>& a, const std::array<double, 2>& b) {
        return 0.5 * (a[0] * b[1] - a[1] * b[0]);
    };
    const auto sector = [radius](const std::array<double, 2>& a, const std::array<double, 2>& b) {
        return 0.5 * radius * radius * std::atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1]);
    };
    double area = 0.0;
    for (size_t i = 0; i < plan.size(); ++i) {
        const std::array<double, 2>& a = plan[i];
        const std::array<double, 2>& b = plan[(i + 1) % plan.size()];
        const double dx = b[0] - a[0], dy = b[1] - a[1];
        const double qa = dx * dx + dy * dy, qb = 2.0 * (a[0] * dx + a[1] * dy);
        const double qc = a[0] * a[0] + a[1] * a[1] - radius * radius;
        const double discriminant = qb * qb - 4.0 * qa * qc;
        const double root = discriminant > 0.0 ? std::sqrt(discriminant) : 0.0;
        const double t0 = (-qb - root) / (2.0 * qa), t1 = (-qb + root) / (2.0 * qa);
        const std::array<double, 2> p0 = {a[0] + t0 * dx, a[1] + t0 * dy}, p1 = {a[0] + t1 * dx, a[1] + t1 * dy};
        const bool a_in = qc < 0.0, b_in = b[0] * b[0] + b[1] * b[1] < radius * radius;
        if (a_in && b_in)
            area += triangle(a, b);
        else if (a_in)
            area += triangle(a, p1) + sector(p1, b);
        else if (b_in)
            area += sector(a, p0) + triangle(p0, b);
        else if (discriminant > 0.0 && t0 > 0.0 && t1 < 1.0)
            area += sector(a, p0) + triangle(p0, p1) + sector(p1, b);
        else
            area += sector(a, b);
    }
    return std::abs(area);
}

BRep plan_prism(const CutPlan& plan, double low, double high) {
    const size_t count = plan.size();
    std::vector<Polyline> faces;
    std::vector<Point> bottom, top;
    for (size_t i = 0; i < count; ++i) {
        bottom.push_back(Point(plan[i][0], plan[i][1], low));
        top.push_back(Point(plan[i][0], plan[i][1], high));
    }
    bottom.push_back(bottom.front());
    top.push_back(top.front());
    faces.emplace_back(bottom);
    faces.emplace_back(top);
    for (size_t i = 0; i < count; ++i) {
        const std::array<double, 2>& a = plan[i];
        const std::array<double, 2>& b = plan[(i + 1) % count];
        faces.emplace_back(std::vector<Point>{Point(a[0], a[1], low), Point(b[0], b[1], low), Point(b[0], b[1], high),
            Point(a[0], a[1], high), Point(a[0], a[1], low)});
    }
    return BRep::from_polylines(faces);
}

/// The cross slot of a column head: a cross of tenons, widened to the beam past the shoulders, all in shares of the beam width.
CutPlan column_slot(double width, double reach) {
    const double half = 0.35, beam = 0.5, shoulder = 0.8;
    CutPlan plan = {{half, shoulder}, {beam, shoulder}, {beam, reach}, {-beam, reach}, {-beam, shoulder}, {-half, shoulder},
        {-half, half}, {-shoulder, half}, {-shoulder, beam}, {-reach, beam}, {-reach, -beam}, {-shoulder, -beam},
        {-shoulder, -half}, {-half, -half}, {-half, -shoulder}, {-beam, -shoulder}, {-beam, -reach}, {beam, -reach},
        {beam, -shoulder}, {half, -shoulder}, {half, -half}, {shoulder, -half}, {shoulder, -beam}, {reach, -beam},
        {reach, beam}, {shoulder, beam}, {shoulder, half}, {half, half}};
    for (std::array<double, 2>& point : plan) {
        point[0] *= width;
        point[1] *= width;
    }
    return plan;
}

/// An exact sewn solid of ordinary trimmed faces: the volume to a millionth of a cubic millimetre, the given number of round faces, the mesh closed.
void check_cut_result(const BRep& result, double exact, int round_expected, double mesh_share = 0.02) {
    int round_faces = 0;
    for (const BRepFace& face : result.m_faces)
        round_faces += result.m_surfaces[face.surface_index].degree(0) > 1 ? 1 : 0;
    const double volume = result.volume_exact();
    const Mesh mesh = result.mesh();
    const std::string measured = "volume " + std::to_string(volume) + " of " + std::to_string(exact) + " mm3, off "
        + std::to_string(volume - exact) + ", mesh " + std::to_string(mesh.volume()) + ", round faces "
        + std::to_string(round_faces) + ", solid " + std::to_string(result.is_solid()) + ", mesh closed "
        + std::to_string(mesh.is_closed());
    if (!result.is_solid() || std::abs(volume - exact) > 1e-6 || round_faces != round_expected || !mesh.is_closed()
        || std::abs(mesh.volume() - exact) > mesh_share * exact)
        throw std::runtime_error(measured);
    MINI_CHECK(result.is_solid());
}

/// A cylinder turned about its axis less a prism down from its top.
void check_cylinder_cut(double radius, double height, double depth, double turn, const CutPlan& plan, double exact) {
    BRep cylinder = BRep::create_cylinder(radius, height);
    cylinder.transform(Xform::rotation_z(turn));
    check_cut_result(cylinder.boolean_difference(plan_prism(plan, height - depth, height + 10.0)), exact, 1);
}

/// The same cut with cylinder and tool turned together: the wall stays where it is on the cylinder and only the world numbers change.
void check_held_wall(double radius, double height, double depth, double turn, const CutPlan& plan, std::string& failures) {
    const double exact = Tolerance::PI * radius * radius * height - plan_area_in_circle(plan, radius) * depth;
    const Xform hold = Xform::rotation_z(turn);
    try {
        check_cut_result(BRep::create_cylinder(radius, height).transformed(hold)
            .boolean_difference(plan_prism(plan, height - depth, height + 10.0).transformed(hold)), exact, 1);
    } catch (const std::exception& error) {
        failures += "\nradius " + std::to_string(radius) + " turn " + std::to_string(turn) + ": " + error.what();
    }
}

}

MINI_TEST("BRep Native Boolean", "Cylinder Less Box Seam In And Out Of The Cut") {
    const CutPlan box = {{-110, -35}, {110, -35}, {110, 35}, {-110, 35}};
    const double exact = Tolerance::PI * 1e4 * 425 - plan_area_in_circle(box, 100) * 185;
    check_cylinder_cut(100, 425, 185, 0.0, box, exact);
    check_cylinder_cut(100, 425, 185, 1.0, box, exact);
}

MINI_TEST("BRep Native Boolean", "Cylinder Less Cross Leaves Four Curved Posts") {
    const CutPlan cross = {{35, -35}, {110, -35}, {110, 35}, {35, 35}, {35, 110}, {-35, 110}, {-35, 35}, {-110, 35},
        {-110, -35}, {-35, -35}, {-35, -110}, {35, -110}};
    const double exact = Tolerance::PI * 1e4 * 425 - plan_area_in_circle(cross, 100) * 185;
    check_cylinder_cut(100, 425, 185, 0.0, cross, exact);
    check_cylinder_cut(100, 425, 185, 0.7, cross, exact);
}

MINI_TEST("BRep Native Boolean", "Cylinder Less Column Head Slot") {
    const CutPlan slot = column_slot(100, 1.1);
    const double exact = Tolerance::PI * 1e4 * 425 - plan_area_in_circle(slot, 100) * 185;
    for (double turn : {0.0, Tolerance::PI / 6.0, Tolerance::PI / 4.0, 0.7})
        check_cylinder_cut(100, 425, 185, turn, slot, exact);
    const CutPlan narrow = column_slot(80, 1.6);
    check_cylinder_cut(120, 425, 200, 0.0, narrow,
        Tolerance::PI * 14400 * 425 - plan_area_in_circle(narrow, 120) * 200);
    const CutPlan wide = column_slot(120, 0.93);
    check_cylinder_cut(100, 425, 150, Tolerance::PI / 4.0, wide,
        Tolerance::PI * 1e4 * 425 - plan_area_in_circle(wide, 100) * 150);
}

MINI_TEST("BRep Native Boolean", "Cylinder Less Prism Tilted Off The Axes") {
    const CutPlan cross = {{35, -35}, {110, -35}, {110, 35}, {35, 35}, {35, 110}, {-35, 110}, {-35, 35}, {-110, 35},
        {-110, -35}, {-35, -35}, {-35, -110}, {35, -110}};
    const Xform tilt = Xform::rotation(Vector(1, 2, 3), 0.8);
    const BRep result = BRep::create_cylinder(100, 425).transformed(tilt)
        .boolean_difference(plan_prism(cross, 240, 435).transformed(tilt));
    const double exact = Tolerance::PI * 1e4 * 425 - plan_area_in_circle(cross, 100) * 185;
    MINI_CHECK(result.is_solid() && std::abs(result.volume_exact() - exact) < 1e-6 && result.mesh().is_closed());
}

MINI_TEST("BRep Native Boolean", "Cylinder Union And Intersection With A Bar") {
    const CutPlan bar = {{-150, -30}, {150, -30}, {150, 30}, {-150, 30}};
    const BRep cylinder = BRep::create_cylinder(100, 425);
    const BRep tool = plan_prism(bar, 100, 200);
    const double common = plan_area_in_circle(bar, 100) * 100;
    const BRep inside = cylinder.boolean_intersection(tool);
    MINI_CHECK(inside.is_solid() && std::abs(inside.volume_exact() - common) < 1e-6 && inside.mesh().is_closed());
    const BRep joined = cylinder.boolean_union(tool);
    const double total = Tolerance::PI * 1e4 * 425 + 300 * 60 * 100 - common;
    MINI_CHECK(joined.is_solid() && std::abs(joined.volume_exact() - total) < 1e-6 && joined.mesh().is_closed());
}

MINI_TEST("BRep Native Boolean", "Blind Round Hole In A Plate") {
    const BRep drill = BRep::create_cylinder(10, 80).transformed(Xform::translation(13.5, -7.25, -10));
    const BRep result = BRep::create_box(100, 100, 50).boolean_difference(drill);
    const double exact = 500000 - Tolerance::PI * 100 * 35;
    MINI_CHECK(result.is_solid() && std::abs(result.volume_exact() - exact) < 1e-6 && result.mesh().is_closed());
}

namespace {
/// All five operations of two overlapping solids close, and their volumes add up around the common part.
void five_operation_identities(const BRep& a, const BRep& b) {
    const double va = a.volume_exact(), vb = b.volume_exact();
    const BRep common = a.boolean_intersection(b);
    MINI_CHECK(common.is_valid() && common.is_solid());
    const double vi = common.volume_exact();
    MINI_CHECK(vi > 1e-3 && vi < std::min(va, vb) - 1e-3);
    const std::array<BRep, 4> results = {a.boolean_union(b), a.boolean_difference(b), b.boolean_difference(a), a.boolean_xor(b)};
    const std::array<double, 4> expected = {va + vb - vi, va - vi, vb - vi, va + vb - 2 * vi};
    for (size_t i = 0; i < results.size(); ++i) {
        MINI_CHECK(results[i].is_valid() && results[i].is_solid());
        MINI_CHECK(std::abs(results[i].volume_exact() - expected[i]) < 1e-6);
    }
}
}

namespace {
/// A pyramid whose sides are bilinear patches collapsed at the apex, one degenerate edge each.
BRep collapsed_pyramid(double base, double height) {
    BRep b;
    const double h = base * 0.5;
    const std::array<Point, 5> p = {Point(-h, -h, 0), Point(h, -h, 0), Point(h, h, 0), Point(-h, h, 0), Point(0, 0, height)};
    for (const Point& q : p)
        b.add_vertex(q);
    const auto line = [](const Point& a, const Point& c) {
        NurbsCurve result = NurbsCurve::create(false, 1, {a, c});
        result.set_domain(0.0, 1.0);
        return result;
    };
    std::map<std::pair<int, int>, int> edges;
    const auto face = [&](const std::array<Point, 4>& corners, const std::vector<std::pair<int, Point>>& ring) {
        NurbsSurface patch(3, false, 2, 2, 2, 2);
        for (int k = 0; k < 4; ++k)
            patch.set_cv(k % 2, k / 2, corners[k]);
        const int surface = b.add_surface(patch);
        std::vector<BRepRef> wire;
        for (size_t k = 0; k < ring.size(); ++k) {
            const auto& [va, ua] = ring[k];
            const auto& [vb, ub] = ring[(k + 1) % ring.size()];
            const std::pair<int, int> key = std::minmax(va, vb);
            if (va != vb && !edges.count(key))
                edges[key] = b.add_edge(b.add_curve_3d(line(p[key.first], p[key.second])), key.first, key.second);
            const int edge = va == vb ? b.add_edge(-1, va, va) : edges[key];
            const bool forward = b.m_edges[edge].start_vertex == va;
            b.add_pcurve(edge, surface, b.add_curve_2d(forward ? line(ua, ub) : line(ub, ua)));
            wire.push_back({edge, forward ? BRepOrientation::Forward : BRepOrientation::Reversed});
        }
        return BRepRef{b.add_face(surface, {{b.add_wire(wire), BRepOrientation::Forward}}), BRepOrientation::Forward};
    };
    std::vector<BRepRef> shell = {face({p[0], p[3], p[1], p[2]}, {{0, Point(0, 0, 0)}, {3, Point(1, 0, 0)}, {2, Point(1, 1, 0)}, {1, Point(0, 1, 0)}})};
    for (int i = 0; i < 4; ++i)
        shell.push_back(face({p[i], p[(i + 1) % 4], p[4], p[4]},
            {{i, Point(0, 0, 0)}, {(i + 1) % 4, Point(1, 0, 0)}, {4, Point(1, 1, 0)}, {4, Point(0, 1, 0)}}));
    b.add_solid({{b.add_shell(shell), BRepOrientation::Forward}});
    return b;
}

/// A pyramid side collapsed at the apex is a plane whose parameters are not affine, cut by conics that leave the triangle; the planar sides of create_pyramid are cut the same way.
void pyramid_against(const BRep& solid) {
    const Xform placed = Xform::translation(0.15, -0.1, 0.35) * Xform::rotation(Vector(1, 2, 3), 0.7);
    const BRep collapsed = collapsed_pyramid(1.2, 1.4);
    MINI_CHECK(collapsed.is_valid() && collapsed.is_solid() && collapsed.edge_count() == 12);
    MINI_CHECK(std::abs(collapsed.volume_exact() - 1.2 * 1.2 * 1.4 / 3) < 1e-12);
    five_operation_identities(collapsed, solid.transformed(placed));
    five_operation_identities(BRep::create_pyramid(1.2, 1.4), solid.transformed(placed));
}
}

MINI_TEST("BRep Native Boolean", "Pyramid Collapsed Faces Against A Tilted Sphere") {
    pyramid_against(BRep::create_sphere(0.6));
}

MINI_TEST("BRep Native Boolean", "Pyramid Collapsed Faces Against A Tilted Cylinder") {
    pyramid_against(BRep::create_cylinder(0.45, 1.1));
}

MINI_TEST("BRep Native Boolean", "Pyramid Collapsed Faces Against A Tilted Cone") {
    pyramid_against(BRep::create_cone(0.55, 1.2));
}

MINI_TEST("BRep Native Boolean", "Sphere Sections Meeting On An Edge Of The Other Solid") {
    // Two section arcs on the sphere end at one point of a box edge or a cone rim, their rational pcurves a hair apart: one node, not a piece inside the vertex.
    const Xform box_placed = Xform::from_matrix({0.057027908583589706, -0.71205414662124078, -0.69980476557542715, 0,
        0.99819556599766945, 0.053865002254347608, 0.026536268666331964, 0,
        0.018799725135606571, -0.70005532196508291, 0.71384110033199255, 0,
        -0.40488519294593361, -0.34100767895003881, -0.2827601462323161, 1});
    five_operation_identities(BRep::create_sphere(0.6 * 0.94077304168792808),
        BRep::create_box(1.1011832113083302, 1.5461308640858129, 1.0642055477237329).transformed(box_placed));
    const Xform sphere_placed = Xform::from_matrix({0.81447359529995822, 0.44791330354442249, 0.36879050295131222, 0,
        -0.57719080709051829, 0.69016928383912213, 0.43648268219390951, 0,
        -0.059021477187172572, -0.56836610749842864, 0.82065609915343274, 0,
        -0.31691755065104443, -0.44854478354990124, -0.12868429973653137, 1});
    five_operation_identities(BRep::create_cone(0.6 * 1.0582795637279456, 0.62566623233237839),
        BRep::create_sphere(0.6 * 0.96727887625466313).transformed(sphere_placed));
}

MINI_TEST("BRep Native Boolean", "Torus Section Dipping Across Its Seam Beside A Knot Line") {
    // The section crosses the torus seam twice within a hair, a knot line between: the short piece from the knot line to the seam belongs to the period before it.
    const Xform placed = Xform::from_matrix({0.94750334969242633, -0.25564145045403325, 0.19205429214518893, 0,
        0.1670291357393402, 0.90790468897029653, 0.38446110799392724, 0,
        -0.27265118766622048, -0.33219952521866608, 0.90294230452930502, 0,
        0.32036832311288754, -0.29713112557879529, 0.19312218990049262, 1});
    five_operation_identities(BRep::create_torus(0.8 * 1.3692296040935199, 0.25 * 1.1149230602792839),
        BRep::create_sphere(0.6 * 1.3781385267651347).transformed(placed));
}

MINI_TEST("BRep Native Boolean", "Section And Pcurve Knots A Rounding Apart") {
    // An axis-parallel cone through a drilled block: a section's world curve and a pcurve end one rounding apart, and a cut there names both.
    const BRep block = BRep::create_block_with_hole(1.2 * 1.4338411151035795, 1.2 * 1.5145944866517971, 1.4577257003922226, 0.3);
    const BRep cone = BRep::create_cone(0.6 * 1.0036137660421676, 1.3484577905685238)
        .transformed(Xform::translation(0.35990501766051547, 0.24731141955836952, -0.29949569698687217));
    const double va = block.volume_exact(), vb = cone.volume_exact();
    const BRep common = block.boolean_intersection(cone);
    MINI_CHECK(common.is_valid() && common.is_solid());
    const double vi = common.volume_exact();
    MINI_CHECK(vi > 1e-3 && vi < std::min(va, vb) - 1e-3);
    const BRep joined = block.boolean_union(cone), cut = block.boolean_difference(cone);
    MINI_CHECK(joined.is_valid() && joined.is_solid() && std::abs(joined.volume_exact() - (va + vb - vi)) < 1e-6);
    MINI_CHECK(cut.is_valid() && cut.is_solid() && std::abs(cut.volume_exact() - (va - vi)) < 1e-6);
}

MINI_TEST("BRep Native Boolean", "Sphere Section Turning Back Beside Its Pole") {
    // The bore of the block cuts the sphere beside a pole, where the section makes a hairpin: the walk's first step is judged like any other, or it lands across the hairpin.
    const BRep block = BRep::create_block_with_hole(1.2 * 0.84170802859589455, 1.2 * 1.5481386144179852, 1.5598235045559705, 0.3);
    const BRep sphere = BRep::create_sphere(0.6 * 1.1454732507932932).transformed(
        Xform::translation(-0.26428989472333342, -0.28393812412396074, -0.014818908716551961)
        * Xform::rotation(Vector(0.25010916963219643, 0.021657375618815422, -0.12511622160673141), 2.1255293400026858));
    five_operation_identities(sphere, block);
}

MINI_TEST("BRep Native Boolean", "Cone Cut By Planes Parallel To Its Axis") {
    // A box side parallel to the axis cuts a hyperbola that crosses the cone seam at its apex side; the seam cut lands one rounding from a sample of the pcurve fit, and the fit keeps no span between them.
    const BRep cone = BRep::create_cone(0.6 * 1.3975388618145015, 1.5526146667042293);
    const BRep box = BRep::create_box(1.4248876423166217, 1.2709414038171074, 0.97973732797664581)
        .transformed(Xform::translation(-0.011692123688115858, -0.40125375759228554, -0.027971456215258962));
    five_operation_identities(cone, box);
}

MINI_TEST("BRep Native Boolean", "Cone Arc From Rim To Rim Is No Loop") {
    // A tilted face of the block cuts the cone in an arc from the base rim back to it, its ends nearer each other than a cell of the one-span cone: an open arc, not a loop traced there and back.
    const BRep cone = BRep::create_cone(0.6 * 1.4819729324299926, 0.6270523628347805);
    const BRep block = BRep::create_block_with_hole(1.2 * 1.5819112356177447, 1.2 * 0.98778585350351233, 0.87125815218067082, 0.3)
        .transformed(Xform::from_matrix({0.06669239165747555, 0.99468450724756263, 0.078452889919234547, 0,
            -0.64648349571223906, 0.10296911915467326, -0.75594738591531241, 0,
            -0.76000737803437768, -0.00030255938980561226, 0.64991437419949916, 0,
            -0.38361927293972137, -0.40482652664952684, -0.18794412814763461, 1}));
    five_operation_identities(cone, block);
}

MINI_TEST("BRep Native Boolean", "Round Caps Flush On Box Faces") {
    // OCCT tests/boolean/bop*_simple ZN5-like and ZF7: a cylinder and a frustum standing in a box, both caps in its faces. A witness of the box face lies in the cap's plane outside the cap, and the frustum's rim is the section itself.
    const BRep box = BRep::create_box(4, 4, 4).transformed(Xform::translation(2, 2, 2));
    const std::array<BRep, 2> solids = {BRep::create_cylinder(1, 4), BRep::create_frustum(1, 0.5, 4)};
    const std::array<double, 2> inner = {Tolerance::PI * 4, Tolerance::PI * 4 * (1 + 0.5 + 0.25) / 3};
    for (size_t i = 0; i < solids.size(); ++i) {
        const BRep solid = solids[i].transformed(Xform::translation(2, 2, 0));
        const BRep joined = box.boolean_union(solid), common = box.boolean_intersection(solid), cut = box.boolean_difference(solid);
        MINI_CHECK(joined.is_valid() && joined.is_solid() && std::abs(joined.volume_exact() - 64) < 1e-6);
        MINI_CHECK(common.is_valid() && common.is_solid() && std::abs(common.volume_exact() - inner[i]) < 1e-6);
        MINI_CHECK(cut.is_valid() && cut.is_solid() && std::abs(cut.volume_exact() - (64 - inner[i])) < 1e-6);
        MINI_CHECK(solid.boolean_difference(box).face_count() == 0);
    }
}

MINI_TEST("BRep Native Boolean", "Frustum Standing On A Box") {
    // OCCT tests/boolean/bop*_simple/ZF5: box b1 4 4 4; pcone b2 1 0.5 2; ttranslate b2 2 2 4. The base lies in the top face: nothing in common, one solid joined.
    const BRep box = BRep::create_box(4, 4, 4).transformed(Xform::translation(2, 2, 2));
    const BRep frustum = BRep::create_frustum(1, 0.5, 2).transformed(Xform::translation(2, 2, 4));
    const double above = Tolerance::PI * 2 * (1 + 0.5 + 0.25) / 3;
    const BRep joined = box.boolean_union(frustum), cut = box.boolean_difference(frustum), rest = frustum.boolean_difference(box);
    MINI_CHECK(box.boolean_intersection(frustum).face_count() == 0);
    MINI_CHECK(joined.is_valid() && joined.is_solid() && joined.solid_count() == 1 && std::abs(joined.volume_exact() - (64 + above)) < 1e-6);
    MINI_CHECK(cut.is_valid() && cut.is_solid() && std::abs(cut.volume_exact() - 64) < 1e-6);
    MINI_CHECK(rest.is_valid() && rest.is_solid() && std::abs(rest.volume_exact() - above) < 1e-6);
}

MINI_TEST("BRep Native Boolean", "Frustum Across A Cylinder") {
    // OCCT tests/boolean/bop*_simple/ZK5: pcylinder b1 4 8; pcone b2 2 1 10; ttranslate b2 0 0 -1; trotate b2 0 0 4 0 1 0 90.
    const BRep cylinder = BRep::create_cylinder(4, 8);
    const BRep frustum = BRep::create_frustum(2, 1, 10).transformed(
        Xform::translation(0, 0, 4) * Xform::rotation(Vector(0, 1, 0), 90, true) * Xform::translation(0, 0, -5));
    five_operation_identities(cylinder, frustum);
}

MINI_TEST("BRep Native Boolean", "Column Head Slots Of Every Measured Node") {
    // The real joint's outlines, sizes, turns and asymmetric nodes, cut with the seam where the column has it and turned into the slot.
    std::ifstream input(std::filesystem::path(__FILE__).parent_path().parent_path() / "tests/fixtures/column_head_slots.json");
    MINI_CHECK(input.good());
    nlohmann::json cases;
    input >> cases;
    MINI_CHECK(cases.size() == 32);
    std::string failures;
    for (const nlohmann::json& node : cases) {
        CutPlan plan;
        for (const nlohmann::json& point : node["outline"])
            plan.push_back({point[0].get<double>(), point[1].get<double>()});
        plan.pop_back();
        const double radius = node["radius"], length = node["length"], depth = -node["floor"].get<double>();
        const double exact = Tolerance::PI * radius * radius * length - plan_area_in_circle(plan, radius) * depth;
        for (const double turn : {0.0, node["seam"].get<double>()})
            try {
                check_cylinder_cut(radius, length, depth, turn, plan, exact);
            } catch (const std::exception& error) {
                failures += "\n" + node["name"].get<std::string>() + ", seam turned " + std::to_string(turn) + ": " + error.what();
            }
    }
    if (!failures.empty())
        throw std::runtime_error(failures);
}

MINI_TEST("BRep Cylinder Cuts", "Wall Exactly Through The Cylinder Seam") {
    std::string failures;
    for (const std::array<double, 3>& size : std::vector<std::array<double, 3>>{{100, 425, 185}, {60, 300, 120}, {150, 500, 200}})
        for (double turn : {0.0, 0.3, 1.0, 2.2}) {
            const double r = size[0];
            check_held_wall(r, size[1], size[2], turn, {{-1.1 * r, 0}, {1.1 * r, 0}, {1.1 * r, 0.4 * r}, {-1.1 * r, 0.4 * r}}, failures);
            check_held_wall(r, size[1], size[2], turn, {{-1.1 * r, -0.4 * r}, {1.1 * r, -0.4 * r}, {1.1 * r, 0}, {-1.1 * r, 0}}, failures);
            check_held_wall(r, size[1], size[2], turn, {{0, 0}, {1.1 * r, 0}, {1.1 * r, 0.4 * r}, {0, 0.4 * r}}, failures);
        }
    if (!failures.empty())
        throw std::runtime_error(failures);
}

MINI_TEST("BRep Cylinder Cuts", "Wall Tangent To The Cylinder") {
    std::string failures;
    for (const std::array<double, 3>& size : std::vector<std::array<double, 3>>{{100, 425, 185}, {60, 300, 120}, {150, 500, 200}})
        for (double turn : {0.0, 0.3, 1.0, 2.2}) {
            const double r = size[0];
            check_held_wall(r, size[1], size[2], turn, {{-r, -0.4 * r}, {r, -0.4 * r}, {r, 0.4 * r}, {-r, 0.4 * r}}, failures);
            check_held_wall(r, size[1], size[2], turn, {{0, -0.4 * r}, {r, -0.4 * r}, {r, 0.4 * r}, {0, 0.4 * r}}, failures);
            check_held_wall(r, size[1], size[2], turn, {{-1.1 * r, -r}, {1.1 * r, -r}, {1.1 * r, 0}, {-1.1 * r, 0}}, failures);
            check_held_wall(r, size[1], size[2], turn, {{r, -0.5 * r}, {1.5 * r, -0.5 * r}, {1.5 * r, 0.5 * r}, {r, 0.5 * r}}, failures);
        }
    if (!failures.empty())
        throw std::runtime_error(failures);
}

MINI_TEST("BRep Cylinder Cuts", "Cross Drill Through A Cylinder") {
    std::string failures;
    for (const std::array<double, 5>& hole : std::vector<std::array<double, 5>>{{100, 425, 10, 0, 0}, {100, 425, 15, 20, 0.4},
        {60, 300, 6, 0, 1.1}, {150, 500, 12, 40, 2.0}, {100, 425, 40, 0, 0}, {100, 425, 40, 25, 0.7}}) {
        const double r = hole[0], height = hole[1], a = hole[2], offset = hole[3];
        double removed = 0.0;
        const int steps = 4000;
        for (int i = 0; i < steps; ++i) {
            const double t = -0.5 * Tolerance::PI + Tolerance::PI * (i + 0.5) / steps;
            const double x = a * std::sin(t) + offset;
            removed += 4.0 * a * a * std::cos(t) * std::cos(t) * std::sqrt(r * r - x * x) * Tolerance::PI / steps;
        }
        BRep drill = BRep::create_cylinder(a, 3 * r);
        drill.transform(Xform::rotation_x(0.5 * Tolerance::PI));
        drill.transform(Xform::translation(offset, 1.5 * r, 0.5 * height));
        const Xform hold = Xform::rotation_z(hole[4]);
        try {
            check_cut_result(BRep::create_cylinder(r, height).transformed(hold).boolean_difference(drill.transformed(hold)),
                Tolerance::PI * r * r * height - removed, 2, 0.03);
        } catch (const std::exception& error) {
            failures += "\nradius " + std::to_string(r) + " drill " + std::to_string(a) + " offset " + std::to_string(offset) + ": " + error.what();
        }
    }
    if (!failures.empty())
        throw std::runtime_error(failures);
}

/// Two solids sharing a line and perhaps an overlap: the exact volume of every operation, each in closed shells.
static void check_touching(const BRep& a, const BRep& b, double va, double vb, double vi, int shells, const std::string& label, std::string& failures) {
    const std::array<double, 4> exact = {va + vb - vi, vi, va - vi, vb - vi};
    const std::array<std::string, 4> names = {"union", "intersection", "difference", "reversed difference"};
    for (size_t operation = 0; operation < exact.size(); ++operation)
        try {
            const BRep result = operation == 0 ? a.boolean_union(b) : operation == 1 ? a.boolean_intersection(b)
                : operation == 2 ? a.boolean_difference(b) : b.boolean_difference(a);
            if (exact[operation] == 0.0) {
                if (result.face_count() != 0)
                    throw std::runtime_error("faces in an empty result");
                continue;
            }
            const double volume = result.volume_exact();
            const Mesh mesh = result.mesh();
            if (result.shell_count() != (operation == 0 ? shells : 1) || std::abs(volume - exact[operation]) > 1e-6 || !mesh.is_closed())
                throw std::runtime_error("volume " + std::to_string(volume) + " of " + std::to_string(exact[operation]) + ", shells "
                    + std::to_string(result.shell_count()) + ", mesh closed " + std::to_string(mesh.is_closed()));
        } catch (const std::exception& error) {
            failures += "\n" + label + " " + names[operation] + ": " + error.what();
        }
}

MINI_TEST("BRep Cylinder Cuts", "Solids Touching Along A Line") {
    std::string failures;
    for (const std::array<double, 2>& size : std::vector<std::array<double, 2>>{{100, 425}, {60, 300}})
        for (double turn : {1.0, 2.2})
            for (double around : {0.0, 1.0}) {
                const double r = size[0], h = size[1];
                const Xform tool = Xform::rotation_z(turn + around);
                const BRep cylinder = BRep::create_cylinder(r, h).transformed(Xform::rotation_z(turn));
                const double round = Tolerance::PI * r * r * h;
                const std::string label = "radius " + std::to_string(r) + " turn " + std::to_string(turn) + " around " + std::to_string(around);
                const CutPlan wall = {{r, -0.5 * r}, {1.5 * r, -0.5 * r}, {1.5 * r, 0.5 * r}, {r, 0.5 * r}};
                check_touching(cylinder, plan_prism(wall, 0, h).transformed(tool), round, 0.5 * r * r * h, 0, 2, label + " flush", failures);
                check_touching(cylinder, plan_prism(wall, -10, h + 10).transformed(tool), round, 0.5 * r * r * (h + 20), 0, 2, label + " long", failures);
                check_touching(cylinder, plan_prism(wall, 0.25 * h, 0.75 * h).transformed(tool), round, 0.25 * r * r * h, 0, 2, label + " short", failures);
                check_touching(cylinder, BRep::create_cylinder(r, h).transformed(Xform::translation(2 * r, 0, 0)).transformed(tool), round, round, 0, 2, label + " two round", failures);
                // Touches at x = r and overlaps above y = 0.8 r, so the union is one body with the contact line inside it.
                const CutPlan hook = {{r, -1.5 * r}, {1.5 * r, -1.5 * r}, {1.5 * r, 1.5 * r}, {0, 1.5 * r}, {0, 0.8 * r}, {r, 0.8 * r}};
                const double hook_area = 3.0 * r * 0.5 * r + 0.7 * r * r;
                check_touching(cylinder, plan_prism(hook, -10, h + 10).transformed(tool), round, hook_area * (h + 20), plan_area_in_circle(hook, r) * h, 1, label + " hook", failures);
            }
    if (!failures.empty())
        throw std::runtime_error(failures);
}

static NurbsCurve unit_line(const Point& a, const Point& b) {
    NurbsCurve line = NurbsCurve::create(false, 1, {a, b});
    line.set_domain(0.0, 1.0);
    return line;
}

/// The closed solid of six ruled patches a curved timber element is built from, between four corner curves.
static BRep ruled_solid(const std::array<NurbsCurve, 4>& corners) {
    const NurbsCurve& outer_a = corners[3];
    const NurbsCurve& outer_b = corners[2];
    const NurbsCurve& inner_a = corners[0];
    const NurbsCurve& inner_b = corners[1];
    const auto first = [](const NurbsCurve& curve) { return curve.point_at(curve.domain_start()); };
    const auto last = [](const NurbsCurve& curve) { return curve.point_at(curve.domain_end()); };
    const std::array<NurbsSurface, 6> patches = {
        Primitives::create_ruled(outer_a, outer_b), Primitives::create_ruled(inner_a, inner_b),
        Primitives::create_ruled(outer_a, inner_a), Primitives::create_ruled(outer_b, inner_b),
        Primitives::create_ruled(unit_line(first(outer_a), first(outer_b)), unit_line(first(inner_a), first(inner_b))),
        Primitives::create_ruled(unit_line(last(outer_a), last(outer_b)), unit_line(last(inner_a), last(inner_b)))};
    BRep solid;
    const std::array<NurbsCurve, 4> meridians = {outer_a, outer_b, inner_a, inner_b};
    for (const NurbsCurve& meridian : meridians) {
        solid.add_vertex(first(meridian));
        solid.add_vertex(last(meridian));
    }
    std::map<std::pair<int, int>, int> edges;
    for (int k = 0; k < 4; ++k) {
        NurbsCurve unit = meridians[k];
        unit.set_domain(0.0, 1.0);
        edges[{2 * k, 2 * k + 1}] = solid.add_edge(solid.add_curve_3d(unit), 2 * k, 2 * k + 1);
    }
    for (int end = 0; end < 2; ++end)
        for (const std::pair<int, int>& ruling : std::array<std::pair<int, int>, 4>{{{0, 2}, {4, 6}, {0, 4}, {2, 6}}}) {
            const int a = ruling.first + end, b = ruling.second + end;
            edges[{a, b}] = solid.add_edge(solid.add_curve_3d(unit_line(solid.m_vertices[a].point, solid.m_vertices[b].point)), a, b);
        }
    const auto corner = [&solid](const Point& point) {
        int nearest = 0;
        for (int i = 1; i < static_cast<int>(solid.m_vertices.size()); ++i)
            if (solid.m_vertices[i].point.distance(point) < solid.m_vertices[nearest].point.distance(point))
                nearest = i;
        return nearest;
    };
    std::array<int, 6> faces;
    for (size_t i = 0; i < 6; ++i) {
        const NurbsSurface& patch = patches[i];
        const auto du = patch.domain(0), dv = patch.domain(1);
        const std::array<Point, 4> uv = {Point(du.first, dv.first, 0), Point(du.second, dv.first, 0), Point(du.second, dv.second, 0), Point(du.first, dv.second, 0)};
        const std::array<int, 4> ring = {corner(patch.point_at_corner(0, 0)), corner(patch.point_at_corner(1, 0)),
            corner(patch.point_at_corner(1, 1)), corner(patch.point_at_corner(0, 1))};
        const int surface = solid.add_surface(patch);
        std::vector<BRepRef> wire;
        for (size_t k = 0; k < 4; ++k) {
            const size_t next = (k + 1) % 4;
            const int edge = edges.at({std::min(ring[k], ring[next]), std::max(ring[k], ring[next])});
            const bool forward = solid.m_edges[edge].start_vertex == ring[k];
            solid.add_pcurve(edge, surface, solid.add_curve_2d(forward ? unit_line(uv[k], uv[next]) : unit_line(uv[next], uv[k])));
            wire.push_back({edge, forward ? BRepOrientation::Forward : BRepOrientation::Reversed});
        }
        faces[i] = solid.add_face(surface, {{solid.add_wire(wire), BRepOrientation::Forward}});
    }
    const auto middle = [](const NurbsSurface& patch) {
        return std::make_pair(0.5 * (patch.domain(0).first + patch.domain(0).second), 0.5 * (patch.domain(1).first + patch.domain(1).second));
    };
    std::vector<BRepRef> shell;
    for (size_t i = 0; i < 6; ++i) {
        const auto own = middle(patches[i]);
        const auto opposite = middle(patches[i ^ 1]);
        const Vector away = patches[i].point_at(own.first, own.second) - patches[i ^ 1].point_at(opposite.first, opposite.second);
        shell.push_back({faces[i], patches[i].normal_at(own.first, own.second).dot(away) > 0.0 ? BRepOrientation::Forward : BRepOrientation::Reversed});
    }
    solid.add_solid({{solid.add_shell(shell), BRepOrientation::Forward}});
    return solid;
}

/// A rectangular section r to r + width by 0 to height bent round the z axis: exact arcs, or cubics through stations as a glulam is built.
static BRep bent_solid(double r, double width, double height, double a0, double a1, int stations) {
    std::array<NurbsCurve, 4> corners;
    const std::array<std::array<double, 2>, 4> section = {{{r, 0}, {r + width, 0}, {r + width, height}, {r, height}}};
    for (size_t k = 0; k < 4; ++k) {
        const auto at = [&](double angle) { return Point(section[k][0] * std::cos(angle), section[k][0] * std::sin(angle), section[k][1]); };
        if (stations == 0) {
            corners[k] = Primitives::arc(at(a0), at(0.5 * (a0 + a1)), at(a1));
        } else {
            std::vector<Point> points;
            for (int i = 0; i <= stations; ++i)
                points.push_back(at(a0 + (a1 - a0) * i / stations));
            corners[k] = NurbsCurve::create_interpolated(points, CurveNurbsKnotStyle::Uniform, CurveInterpStyle::Occt);
        }
    }
    return ruled_solid(corners);
}

/// One side of a plane through a closed solid: sewn, closed in the mesh, and the two sides together are the whole.
static double check_side(const BRep& solid, const Point& origin, const Vector& normal, double whole, const std::string& label, std::string& failures) {
    double kept = 0.0;
    try {
        const BRep side = solid.cut_by_plane(Plane::from_point_normal(origin, normal));
        const BRep rest = solid.cut_by_plane(Plane::from_point_normal(origin, normal * -1.0));
        kept = side.volume_exact();
        const double other = rest.volume_exact();
        if (side.face_count() == 0 || !side.is_solid() || !rest.is_solid() || !side.mesh().is_closed() || !rest.mesh().is_closed()
            || std::abs(kept + other - whole) > 4e-6)
            throw std::runtime_error("faces " + std::to_string(side.face_count()) + " and " + std::to_string(rest.face_count()) + ", volumes "
                + std::to_string(kept) + " and " + std::to_string(other) + " of " + std::to_string(whole));
    } catch (const std::exception& error) {
        failures += "\n" + label + ": " + error.what();
    }
    return kept;
}

MINI_TEST("BRep Curved Cuts", "Bent Beam Cut Through Its Axis") {
    std::string failures;
    for (const std::array<double, 5>& size : std::vector<std::array<double, 5>>{{2000, 120, 240, 0.2, 1.2}, {3500, 140, 320, 0.1, 0.8}, {800, 100, 200, 0.5, 2.1}})
        for (double share : {0.3, 0.45, 0.7}) {
            const double r = size[0], width = size[1], height = size[2], a0 = size[3], a1 = size[4];
            const double angle = a0 + share * (a1 - a0);
            const double area = 0.5 * ((r + width) * (r + width) - r * r) * height;
            const BRep solid = bent_solid(r, width, height, a0, a1, 0);
            const std::string label = "radius " + std::to_string(r) + " angle " + std::to_string(angle);
            const double kept = check_side(solid, Point(0, 0, 0), Vector(-std::sin(angle), std::cos(angle), 0), area * (a1 - a0), label, failures);
            if (std::abs(kept - area * (a1 - angle)) > 1e-6)
                failures += "\n" + label + ": volume " + std::to_string(kept) + " of " + std::to_string(area * (a1 - angle));
            try {
                const CutPlan past = {{-1e4 * std::cos(angle), -1e4 * std::sin(angle)}, {1e4 * std::cos(angle), 1e4 * std::sin(angle)},
                    {1e4 * std::cos(angle) - 1e4 * std::sin(angle), 1e4 * std::sin(angle) + 1e4 * std::cos(angle)},
                    {-1e4 * std::cos(angle) - 1e4 * std::sin(angle), -1e4 * std::sin(angle) + 1e4 * std::cos(angle)}};
                const BRep result = solid.boolean_difference(plan_prism(past, -500, height + 500));
                int round_faces = 0;
                for (const BRepFace& face : result.m_faces)
                    round_faces += result.m_surfaces[face.surface_index].degree(0) > 1 ? 1 : 0;
                if (!result.is_solid() || !result.mesh().is_closed() || result.face_count() != 6 || round_faces != 4
                    || std::abs(result.volume_exact() - area * (angle - a0)) > 1e-6)
                    throw std::runtime_error("faces " + std::to_string(result.face_count()) + ", round " + std::to_string(round_faces)
                        + ", volume " + std::to_string(result.volume_exact()) + " of " + std::to_string(area * (angle - a0)));
            } catch (const std::exception& error) {
                failures += "\n" + label + " half space: " + error.what();
            }
        }
    if (!failures.empty())
        throw std::runtime_error(failures);
}

MINI_TEST("BRep Curved Cuts", "Bent Beam And Plate Cut By Oblique Planes") {
    // Volumes on the normal side from tests/fixtures/curved_cuts/bent_beam.py, cylindrical integrals sharing nothing with the kernel.
    struct Case { double r, width, height, a0, a1; Point origin; Vector normal; double exact; };
    const std::vector<Case> cases = {
        {2000, 120, 240, 0.2, 1.2, Point(1400, 0, 0), Vector(1, 0, 0), 36988071.6212525},
        {2000, 120, 240, 0.2, 1.2, Point(1400, 1400, 120), Vector(0.5, 0.7, 0.25), 33325430.8680800},
        {2000, 120, 240, 0.2, 1.2, Point(1500, 1200, 120), Vector(0.1, 0.2, 1.0), 33541305.1630877},
        {2000, 120, 240, 0.2, 1.2, Point(1500, 1300, 100), Vector(1, 1, 0.3), 33121783.6647558},
        {3000, 40, 1000, 0.3, 1.1, Point(0, 0, 0), Vector(-std::sin(0.7), std::cos(0.7), 0), 48320000.0},
        {3000, 40, 1000, 0.3, 1.1, Point(0, 0, 400), Vector(0, 0, 1), 57984000.0},
        {3000, 40, 1000, 0.3, 1.1, Point(2200, 1800, 500), Vector(1, -0.4, 0.5), 50405279.0815020}};
    std::string failures;
    for (const Case& c : cases) {
        const double whole = 0.5 * ((c.r + c.width) * (c.r + c.width) - c.r * c.r) * c.height * (c.a1 - c.a0);
        const std::string label = "radius " + std::to_string(c.r) + " to " + std::to_string(c.exact);
        const double kept = check_side(bent_solid(c.r, c.width, c.height, c.a0, c.a1, 0), c.origin, c.normal, whole, label, failures);
        if (std::abs(kept - c.exact) > (c.r == 3000 ? 4e-6 : 1e-6)) // The plate through its axis stays 2.5e-6 high, at any placement and measuring tolerance.
            failures += "\n" + label + ": volume " + std::to_string(kept);
    }
    if (!failures.empty())
        throw std::runtime_error(failures);
}

MINI_TEST("BRep Curved Cuts", "Glulam Cubics And A Double Curved Shell Cut By Planes") {
    std::string failures;
    const std::vector<std::pair<Point, Vector>> beam_planes = {{Point(0, 0, 0), Vector(-std::sin(0.7), std::cos(0.7), 0)}, {Point(1400, 0, 0), Vector(1, 0, 0)},
        {Point(1400, 1400, 120), Vector(0.5, 0.7, 0.25)}, {Point(1500, 1300, 100), Vector(1, 1, 0.3)}};
    for (int stations : {6, 24}) {
        const BRep beam = bent_solid(2000, 120, 240, 0.2, 1.2, stations);
        const double whole = beam.volume_exact();
        for (size_t i = 0; i < beam_planes.size(); ++i) {
            const std::string label = "stations " + std::to_string(stations) + " plane " + std::to_string(i);
            const double kept = check_side(beam, beam_planes[i].first, beam_planes[i].second, whole, label, failures);
            // Uniform stations are symmetric about the middle angle, so that plane halves the beam.
            if (i == 0 && std::abs(kept - 0.5 * whole) > 1e-5)
                failures += "\n" + label + ": half " + std::to_string(kept) + " of " + std::to_string(whole);
        }
    }
    // The sections of a dome, so that no wall is an extrusion and no face is planar: 60 thick, 1000 wide, 3000 long.
    std::array<NurbsCurve, 4> corners;
    const std::array<std::array<double, 2>, 4> section = {{{-500, 0}, {500, 0}, {500, 60}, {-500, 60}}};
    for (size_t k = 0; k < 4; ++k) {
        std::vector<Point> points;
        for (int i = 0; i <= 8; ++i) {
            const double t = -1.0 + 0.25 * i, y = section[k][0];
            points.push_back(Point(1500.0 * t, y, section[k][1] + 300.0 * (1.0 - t * t) - 0.00008 * y * y));
        }
        corners[k] = NurbsCurve::create_interpolated(points, CurveNurbsKnotStyle::Uniform, CurveInterpStyle::Occt);
    }
    const BRep shell = ruled_solid(corners);
    const double whole = shell.volume_exact();
    MINI_CHECK(std::abs(whole - 1.8e8) < 1e-6);
    MINI_CHECK(std::abs(check_side(shell, Point(0, 0, 0), Vector(1, 0, 0), whole, "shell middle", failures) - 0.5 * whole) < 1e-6);
    check_side(shell, Point(300, 0, 0), Vector(1, 0.3, 0.1), whole, "shell oblique", failures);
    check_side(shell, Point(0, 0, 150), Vector(0.1, 0.1, 1), whole, "shell across", failures);
    if (!failures.empty())
        throw std::runtime_error(failures);
}

MINI_TEST("BRep Curved Cuts", "Cut Past A Curved Solid Is Whole Or Empty") {
    const BRep beam = bent_solid(2000, 120, 240, 0.2, 1.2, 6);
    const double whole = beam.volume_exact();
    const BRep kept = beam.cut_by_plane(Plane::from_point_normal(Point(0, 0, -100), Vector(0, 0, 1)));
    MINI_CHECK(kept.face_count() == 6 && kept.is_solid() && std::abs(kept.volume_exact() - whole) < 1e-6);
    MINI_CHECK(beam.cut_by_plane(Plane::from_point_normal(Point(0, 0, 1000), Vector(0, 0, 1))).face_count() == 0);
}

/// The vault strip of the curved face examples: quarter arcs in xz at radius 2000 and 1880, y 0 to 600, exact or through seven points.
static BRep vault_strip(bool exact) {
    const auto arc = [exact](double r, double y) {
        if (exact) {
            const double middle = 0.25 * Tolerance::PI;
            return Primitives::arc(Point(r, y, 0), Point(r * std::cos(middle), y, r * std::sin(middle)), Point(0, y, r));
        }
        std::vector<Point> points;
        for (int k = 0; k <= 6; ++k) {
            const double angle = 0.5 * Tolerance::PI * k / 6.0;
            points.push_back(Point(r * std::cos(angle), y, r * std::sin(angle)));
        }
        return NurbsCurve::create_interpolated(points);
    };
    return ruled_solid({arc(1880, 0), arc(1880, 600), arc(2000, 600), arc(2000, 0)});
}

/// The arch of the curved edge examples: the parabola through nine points, 140 wide and 240 deep, corner cubics through 33 frames.
static BRep glulam_arch() {
    std::vector<Point> points;
    for (int i = 0; i <= 8; ++i) {
        const double u = i / 8.0;
        points.push_back(Point(2400 * u, 0, 2800 * u * (1 - u)));
    }
    const NurbsCurve axis = NurbsCurve::create_interpolated(points);
    std::array<std::vector<Point>, 4> rows;
    for (int i = 0; i <= 32; ++i) {
        const std::vector<Vector> jet = axis.evaluate(axis.domain_start() + (axis.domain_end() - axis.domain_start()) * i / 32.0, 1);
        Vector tangent = jet[1];
        tangent.normalize_self();
        const Vector side(0, 70, 0);
        const Vector up = tangent.cross(Vector(0, -120, 0));
        const Point centre(jet[0][0], jet[0][1], jet[0][2]);
        rows[0].push_back(centre - side - up);
        rows[1].push_back(centre + side - up);
        rows[2].push_back(centre + side + up);
        rows[3].push_back(centre - side + up);
    }
    std::array<NurbsCurve, 4> corners;
    for (size_t k = 0; k < 4; ++k)
        corners[k] = NurbsCurve::create_interpolated(rows[k], CurveNurbsKnotStyle::Uniform, CurveInterpStyle::Occt);
    return ruled_solid(corners);
}

/// A box on half sizes a and b round origin along the frame, from low to high along z.
static BRep frame_box(const Point& origin, const Vector& x, const Vector& y, const Vector& z, double a, double b, double low, double high) {
    const auto at = [&](double i, double j, double k) { return origin + x * i + y * j + z * k; };
    const std::array<Point, 4> bottom = {at(-a, -b, low), at(a, -b, low), at(a, b, low), at(-a, b, low)};
    const std::array<Point, 4> top = {at(-a, -b, high), at(a, -b, high), at(a, b, high), at(-a, b, high)};
    std::vector<Polyline> faces = {Polyline({bottom[0], bottom[1], bottom[2], bottom[3], bottom[0]}), Polyline({top[0], top[1], top[2], top[3], top[0]})};
    for (size_t i = 0; i < 4; ++i)
        faces.emplace_back(std::vector<Point>{bottom[i], bottom[(i + 1) % 4], top[(i + 1) % 4], top[i], bottom[i]});
    return BRep::from_polylines(faces);
}

/// The kernel cylinder laid from a to b.
static BRep drill_along(const Point& a, const Point& b, double radius) {
    BRep drill = BRep::create_cylinder(radius, a.distance(b));
    Vector z = b - a;
    z.normalize_self();
    drill.transform(Xform::rotation_y(std::acos(std::clamp(z[2], -1.0, 1.0))));
    drill.transform(Xform::rotation_z(std::atan2(z[1], z[0])));
    drill.transform(Xform::translation(a[0], a[1], a[2]));
    return drill;
}

/// Cutting a tool from a solid: exact sewn solids on both sides of the cut, the kept and the removed parts adding up to the whole.
static void check_feature(const BRep& solid, const BRep& tool, double removed, int faces, const std::string& label, std::string& failures, double allowed = 1e-6) {
    try {
        const double whole = solid.volume_exact();
        const BRep kept = solid.boolean_difference(tool);
        const BRep common = solid.boolean_intersection(tool);
        const double a = kept.volume_exact(), b = common.volume_exact();
        // The three volumes round apart: their sum is held to 64 units in the last place of the whole, and to 1e-6 where that is less.
        const double rounding = std::max(1e-6, 64.0 * (std::nextafter(whole, std::numeric_limits<double>::infinity()) - whole));
        int round_faces = 0;
        for (const BRepFace& face : kept.m_faces)
            round_faces += kept.m_surfaces[face.surface_index].degree(0) > 1 || kept.m_surfaces[face.surface_index].degree(1) > 1 ? 1 : 0;
        if (!kept.is_solid() || !common.is_solid() || !kept.mesh().is_closed() || !common.mesh().is_closed() || std::abs(a + b - whole) > rounding
            || kept.face_count() != faces || round_faces < 4 || (removed > 0.0 && std::abs(b - removed) > allowed))
            throw std::runtime_error("faces " + std::to_string(kept.face_count()) + ", round " + std::to_string(round_faces) + ", kept "
                + std::to_string(a) + ", removed " + std::to_string(b) + " of " + std::to_string(removed) + ", whole " + std::to_string(whole));
    } catch (const std::exception& error) {
        failures += "\n" + label + ": " + error.what();
    }
}

MINI_TEST("BRep Curved Cuts", "Vault Strip Drilled And Mortised") {
    // Removed volumes from tests/fixtures/curved_cuts/vault_strip.py, which integrates the shell along radial lines; zero for the cubics, held by their sum.
    const double turn = 0.4 * Tolerance::PI;
    const Vector out(std::cos(turn), 0, std::sin(turn)), along(-std::sin(turn), 0, std::cos(turn)), y(0, 1, 0);
    const Point foot(0, 300, 0), middle = foot + out * 1940.0;
    std::string failures;
    for (bool exact : {true, false}) {
        const BRep strip = vault_strip(exact);
        const std::string label = exact ? "arcs " : "cubics ";
        // The drill through the rational walls measures 1.4e-6 low, the one case here above 1e-6.
        check_feature(strip, drill_along(foot + out * 2060.0, foot + out * 1820.0, 30), exact ? 339302.159189224 : 0.0, 7, label + "drill", failures, 2e-6);
        check_feature(strip, frame_box(middle, along, y, out, 60, 40, -100, 100), exact ? 1152183.909136945 : 0.0, 10, label + "mortise", failures);
        check_feature(strip, frame_box(middle, along, y, out, 60, 40, 0, 100), exact ? 573119.611074974 : 0.0, 11, label + "blind mortise", failures);
        check_feature(strip, frame_box(Point(0, 600, 0) + out * 1940.0, along, y, out, 50, 60, -100, 100), exact ? 720079.811146517 : 0.0, 10, label + "notch", failures);
        check_feature(strip, drill_along(Point(0, 650, 0) + out * 1940.0, Point(0, 550, 0) + out * 1940.0, 8), 100.0 * Tolerance::PI * 0.64 * 50.0, 8, label + "pin", failures);
    }
    if (!failures.empty())
        throw std::runtime_error(failures);
}

MINI_TEST("BRep Curved Cuts", "Glulam Arch Drilled And Mortised") {
    const BRep arch = glulam_arch();
    const Vector x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
    const Point crown(1200, 0, 700);
    std::string failures;
    check_feature(arch, drill_along(Point(1200, 120, 700), Point(1200, -120, 700), 30), Tolerance::PI * 900 * 140, 7, "drill", failures);
    check_feature(arch, frame_box(crown, x, z, y, 40, 60, -120, 120), 80.0 * 120 * 140, 10, "mortise", failures);
    check_feature(arch, frame_box(crown, x, z, y, 40, 60, 0, 120), 80.0 * 120 * 70, 11, "blind mortise", failures);
    check_feature(arch, frame_box(Point(1200, 0, 820), x, y, z, 60, 100, -50, 100), 0.0, 10, "notch", failures);
    check_feature(arch, drill_along(Point(1100, 0, 700), Point(1300, 0, 700), 8), Tolerance::PI * 64 * 200, 9, "pin", failures);
    if (!failures.empty())
        throw std::runtime_error(failures);
}

MINI_TEST("BRep Curved Cuts", "Vault Strip And Arch Cut At The Element Planes") {
    std::string failures;
    for (bool exact : {true, false}) {
        const BRep strip = vault_strip(exact);
        check_side(strip, Point(1400, 0, 1400), Vector(-1, 0, -0.4), strip.volume_exact(), exact ? "strip arcs" : "strip cubics", failures);
    }
    const BRep arch = glulam_arch();
    check_side(arch, Point(1800, 0, 300), Vector(-1, 0, -0.3), arch.volume_exact(), "arch", failures);
    if (!failures.empty())
        throw std::runtime_error(failures);
}

MINI_TEST("BRep Curved Cuts", "Vault Strips Pinned Across Their Joint") {
    // element_face_curved_joints: two strips meet at y = 600; four pins of radius 8, 160 long with 10 overshoot,
    // stand 50 in from the joint face's outline, at radius 1930 and 1950 near both straight ends.
    const BRep first = vault_strip(false);
    const BRep second = first.transformed(Xform::translation(0, 600, 0));
    std::vector<std::array<double, 2>> stations;
    for (double r : {1930.0, 1950.0}) {
        const double along = std::sqrt(r * r - 50.0 * 50.0);
        stations.push_back({along, 50.0});
        stations.push_back({50.0, along});
    }
    std::string failures;
    for (const auto& [strip, label] : {std::pair<const BRep&, std::string>{first, "first"}, std::pair<const BRep&, std::string>{second, "second"}}) {
        BRep drilled = strip;
        const double whole = strip.volume_exact();
        try {
            for (const std::array<double, 2>& station : stations)
                drilled = drilled.boolean_difference(drill_along(Point(station[0], 510, station[1]), Point(station[0], 690, station[1]), 8));
            const double removed = whole - drilled.volume_exact();
            if (!drilled.is_solid() || !drilled.mesh().is_closed() || drilled.face_count() != 14
                || std::abs(removed - 4.0 * Tolerance::PI * 64.0 * 90.0) > 1e-6)
                throw std::runtime_error("faces " + std::to_string(drilled.face_count()) + ", removed " + std::to_string(removed)
                    + " of " + std::to_string(4.0 * Tolerance::PI * 64.0 * 90.0) + ", solid " + std::to_string(drilled.is_solid())
                    + ", mesh closed " + std::to_string(drilled.mesh().is_closed()));
        } catch (const std::exception& error) {
            failures += "\n" + label + ": " + error.what();
        }
    }
    if (!failures.empty())
        throw std::runtime_error(failures);
}

MINI_TEST("BRep Curved Cuts", "Drilled Vault Strip Meshes Clean Along Its Hole") {
    // element_face_curved_holes: a drill of radius 30 through the strip along the radius at 0.4 pi. Along each rim the
    // mesh edges are of one size: none shorter than a tenth of their median, so the rim reads round and not serrated.
    const double turn = 0.4 * Tolerance::PI;
    const Vector out(std::cos(turn), 0, std::sin(turn));
    const Point foot(0, 300, 0);
    for (bool exact : {true, false}) {
        const BRep drilled = vault_strip(exact).boolean_difference(drill_along(foot + out * 2060.0, foot + out * 1820.0, 30));
        MINI_CHECK(drilled.is_solid() && drilled.face_count() == 7);
        const Mesh mesh = drilled.mesh();
        MINI_CHECK(mesh.is_closed());
        std::set<std::pair<size_t, size_t>> directed;
        for (const auto& face : mesh.face)
            for (size_t i = 0; i < face.second.size(); ++i)
                directed.emplace(face.second[i], face.second[(i + 1) % face.second.size()]);
        int unpaired = 0;
        for (const auto& edge : directed)
            unpaired += directed.count({edge.second, edge.first}) ? 0 : 1;
        MINI_CHECK(unpaired == 0); // Every edge used once each way: the faces agree in orientation.
        std::vector<double> rim;
        for (const auto& edge : directed) {
            if (edge.first > edge.second)
                continue;
            const Point a = mesh.vertex.at(edge.first).position(), b = mesh.vertex.at(edge.second).position();
            const auto off_axis = [&](const Point& p) {
                const Vector d = p - foot;
                return (d - out * d.dot(out)).magnitude();
            };
            const auto on_shell = [&](const Point& p) {
                const double r = std::sqrt(p[0] * p[0] + p[2] * p[2]);
                return std::abs(r - 1880.0) < 1.0 || std::abs(r - 2000.0) < 1.0;
            };
            if (std::abs(off_axis(a) - 30.0) < 1e-3 && std::abs(off_axis(b) - 30.0) < 1e-3 && on_shell(a) && on_shell(b))
                rim.push_back(a.distance(b));
        }
        MINI_CHECK(rim.size() >= 16);
        std::vector<double> sorted = rim;
        std::sort(sorted.begin(), sorted.end());
        const double median = sorted[sorted.size() / 2];
        MINI_CHECK(sorted.front() >= 0.1 * median);
    }
}

MINI_TEST("BRep Curved Cuts", "Vault Strip Pinned Where Wood Lays The Pin") {
    // element_face_curved_joints, as wood builds it: four pins of radius 8 on the joint face's faceted outline, through y 520
    // to 690 for the first strip and 510 to 680 for the second, each drill a kernel cylinder in the frame of the plane
    // through its start normal to its axis. Plain GCC once split this scene into pieces too small to intersect.
    const std::array<std::array<double, 2>, 4> stations = {{{1378.8553025526362, 1378.8553025526362},
        {1929.1433025526362, 50.000302552636128}, {1949.1353025526362, 50.000302552636128}, {50.000302552636178, 1949.1353025526362}}};
    for (const auto& [shift, low] : std::vector<std::pair<double, double>>{{0.0, 520.0}, {600.0, 510.0}}) {
        const BRep strip = vault_strip(false).transformed(Xform::translation(0, shift, 0));
        BRep drilled = strip;
        for (const std::array<double, 2>& station : stations) {
            const Point start(station[0], low, station[1]), end(station[0], low + 170.0, station[1]);
            const Plane frame = Plane::from_point_normal(start, end - start);
            drilled = drilled.boolean_difference(BRep::create_cylinder(8, start.distance(end)).transformed(
                Xform::frame_to_world(start, frame.x_axis(), frame.y_axis(), frame.z_axis())));
        }
        MINI_CHECK(drilled.is_solid() && drilled.face_count() == 14 && drilled.mesh().is_closed());
        MINI_CHECK(std::abs(strip.volume_exact() - drilled.volume_exact() - 4.0 * Tolerance::PI * 64.0 * 80.0) < 1e-6);
    }
}
