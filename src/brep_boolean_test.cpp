#include <iostream>
#include <cstdio>
#include <cstdlib>
#include "brep_boolean.h"
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
            const BRep result = brep_boolean::apply(swapped ? b : a, swapped ? a : b, event_operations[operation], 1e-6);
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
