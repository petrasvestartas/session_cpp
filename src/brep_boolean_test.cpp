#include "brep_boolean.h"
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
