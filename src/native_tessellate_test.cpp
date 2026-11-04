#include "brep.h"
#include "brep_assemble.h"
#include "curve_surface_intersection.h"
#include "mesh_parametric_metadata.h"
#include "native_tessellate_diagnostics.h"
#include <fstream>
#include <cstdlib>
#include "file_step.h"
#include "primitives.h"
#include <filesystem>
#include <set>
#include <map>
#include "mini_test.h"
#include "tolerance.h"
#include "xform.h"
#include <cmath>
#include <limits>
#include <stdexcept>
using namespace session_cpp::mini_test;
namespace session_cpp {
namespace {
Mesh observed_tessellation(const BRep& shape, double chord, double angle, const std::string& label) {
    const char* folder = std::getenv("SESSION_TESSELLATE_PROGRESS");
    if (!folder)
        return shape.tessellate(chord, angle);
    const std::filesystem::path call_folder = std::filesystem::path(folder) / label;
    if (!std::filesystem::create_directory(call_folder))
        throw std::runtime_error("Tessellation progress call directory already exists");
    const std::filesystem::path path = call_folder / "progress.jsonl";
    const std::function<void(const native_tessellate_diagnostics::State&)> sink =
        [&](const native_tessellate_diagnostics::State& state) {
            const nlohmann::json record = {{"label", label}, {"deflection", chord}, {"angle", angle},
                {"phase", state.phase}, {"pass", state.pass}, {"previous_phase", state.previous_phase},
                {"elapsed_previous_phase_seconds", state.elapsed_previous_phase_seconds},
                {"checkpoint", state.checkpoint},
                {"elapsed_current_phase_seconds", state.elapsed_current_phase_seconds},
                {"triangles", state.triangles}, {"visited", state.visited}, {"certified", state.certified},
                {"sampled", state.sampled}, {"measured_passed", state.measured_passed},
                {"face", state.face}, {"uv", state.uv}, {"metadata_failure", state.metadata_failure}};
            std::ofstream output(path, std::ios::binary | std::ios::app);
            if (!output || !(output << record.dump() << '\n'))
                throw std::runtime_error("Cannot write tessellation progress artifact");
            output.flush();
            if (!output)
                throw std::runtime_error("Cannot flush tessellation progress artifact");
            output.close();
            if (!output)
                throw std::runtime_error("Cannot close tessellation progress artifact");
        };
    native_tessellate_diagnostics::State boundary;
    boundary.phase = "test_call";
    sink(boundary);
    const native_tessellate_diagnostics::Capture capture(sink);
    return shape.tessellate(chord, angle);
}

double tessellation_signed_volume(const Mesh& mesh) {
    MINI_CHECK(!mesh.vertex.empty());
    const Point origin = mesh.vertex.begin()->second.position();
    long double sum = 0;
    for (const auto& face : mesh.face) {
        MINI_CHECK(face.second.size() == 3);
        const Vector a = mesh.vertex.at(face.second[0]).position() - origin;
        const Vector b = mesh.vertex.at(face.second[1]).position() - origin;
        const Vector c = mesh.vertex.at(face.second[2]).position() - origin;
        sum += static_cast<long double>(a.dot(b.cross(c)));
    }
    return static_cast<double>(sum / 6);
}

BRep tessellation_ruled_patch() {
    const NurbsCurve first = NurbsCurve::create(false, 2, {{-1, 0, 1}, {0, 0, -1}, {1, 0, 1}});
    NurbsCurve second = first;
    for (int i = 0; i < second.cv_count(); ++i) {
        Point point = second.get_cv(i);
        point[1] += 2;
        MINI_CHECK(second.set_cv(i, point));
    }
    const NurbsSurface surface = Primitives::create_ruled(first, second);
    const std::pair<double, double> u = surface.domain(0), v = surface.domain(1);
    const std::array<Point, 4> uv = {Point(u.first, v.first, 0), Point(u.second, v.first, 0),
        Point(u.second, v.second, 0), Point(u.first, v.second, 0)};
    BRep result;
    const int support = result.add_surface(surface);
    for (const Point& point : uv)
        result.add_vertex(surface.point_at(point[0], point[1]));
    const std::array<NurbsCurve, 4> curves = {surface.iso_curve(0, v.first), surface.iso_curve(1, u.second),
        surface.iso_curve(0, v.second), surface.iso_curve(1, u.first)};
    const std::array<int, 4> starts = {0, 1, 3, 0}, ends = {1, 2, 2, 3};
    std::vector<BRepRef> wire;
    for (int i = 0; i < 4; ++i) {
        const int edge = result.add_edge(result.add_curve_3d(curves[i]), starts[i], ends[i]);
        NurbsCurve pc = NurbsCurve::create(false, 1, {uv[starts[i]], uv[ends[i]]});
        MINI_CHECK(pc.set_domain(curves[i].domain_start(), curves[i].domain_end()));
        result.add_pcurve(edge, support, result.add_curve_2d(pc));
        wire.push_back({edge, i < 2 ? BRepOrientation::Forward : BRepOrientation::Reversed});
    }
    const int face = result.add_face(support, {{result.add_wire(wire), BRepOrientation::Forward}});
    result.add_shell({{face, BRepOrientation::Forward}});
    MINI_CHECK(result.is_valid());
    return result;
}
}
MINI_TEST("BRep Native Tessellate", "Absolute Chord Scale And Radian Angle") {
    for (double scale : {.01, 1.0, 100.0}) {
        const BRep sphere = BRep::create_sphere(scale);
        const Mesh mesh = observed_tessellation(sphere, .005 * scale, .15, "sphere_" + std::to_string(scale));
        MINI_CHECK(!mesh.is_empty() && mesh.is_closed());
        const double expected = 4 * Tolerance::PI * scale * scale * scale / 3;
        MINI_CHECK(std::abs(std::abs(mesh.volume()) - expected) < expected * .01);
    }
    const BRep sphere = BRep::create_sphere(1);
    const Mesh loose = observed_tessellation(sphere, .02, .5, "sphere_angle_loose");
    const Mesh tight = observed_tessellation(sphere, .02, .1, "sphere_angle_tight");
    MINI_CHECK(tight.number_of_faces() >= loose.number_of_faces());
}
MINI_TEST("BRep Native Tessellate", "Trimmed Hole Shared Edges And Orientation") {
    const BRep shape = BRep::create_block_with_hole(4, 4, 2, .5);
    const Mesh mesh = shape.tessellate(.005, .15);
    MINI_CHECK(!mesh.is_empty() && mesh.is_closed());
    const double expected = 32 - .5 * Tolerance::PI;
    MINI_CHECK(std::abs(std::abs(mesh.volume()) - expected) < expected * .01);
    BRep reversed = shape;
    reversed.m_solids.front().shells.front().orientation = BRepOrientation::Reversed;
    const Mesh inward = reversed.tessellate(.005, .15);
    MINI_CHECK(inward.is_closed());
    const double outward_volume = tessellation_signed_volume(mesh);
    const double inward_volume = tessellation_signed_volume(inward);
    MINI_CHECK(std::abs(std::abs(outward_volume) - mesh.volume()) < expected * 1e-12);
    MINI_CHECK(std::abs(std::abs(inward_volume) - inward.volume()) < expected * 1e-12);
    MINI_CHECK(inward_volume * outward_volume < 0);
}
MINI_TEST("BRep Native Tessellate", "Finite Positive Arguments") {
    const BRep shape = BRep::create_box(2, 2, 2);
    for (double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::quiet_NaN()}) {
        bool chord = false, angle = false;
        try { shape.tessellate(invalid, .15); }
        catch (const std::invalid_argument&) { chord = true; }
        try { shape.tessellate(.005, invalid); }
        catch (const std::invalid_argument&) { angle = true; }
        MINI_CHECK(chord && angle);
    }
}
MINI_TEST("BRep Native Tessellate", "Parametric Metadata Is Opt In And Face Local") {
    std::set<int> paths;
    for (const BRep& shape : {BRep::create_box(2, 3, 4), BRep::create_sphere(1),
            BRep::create_cylinder(1, 2), BRep::create_block_with_hole(4, 4, 2, .5), tessellation_ruled_patch()}) {
        const std::vector<Mesh> ordinary = shape.face_meshes_q(true, 10, .0025);
        std::vector<Mesh> recorded;
        {
            const mesh_parametric_metadata::Capture capture;
            recorded = shape.face_meshes_q(true, 10, .0025);
        }
        MINI_CHECK(ordinary.size() == recorded.size());
        for (size_t face = 0; face < recorded.size(); ++face) {
            MINI_CHECK(ordinary[face].face == recorded[face].face);
            MINI_CHECK(ordinary[face].vertex.size() == recorded[face].vertex.size());
            const NurbsSurface& surface = shape.m_surfaces[shape.m_faces[face].surface_index];
            for (const auto& vertex : recorded[face].vertex) {
                const Point first = vertex.second.position();
                const Point second = ordinary[face].vertex.at(vertex.first).position();
                MINI_CHECK(first[0] == second[0] && first[1] == second[1] && first[2] == second[2]);
            }
            for (const auto& triangle : recorded[face].face) {
                paths.insert(static_cast<int>(recorded[face].facedata.at(triangle.first).at("tessellation_path")));
                for (size_t vertex : triangle.second) {
                    mesh_parametric_metadata::UV uv, absent;
                    MINI_CHECK(mesh_parametric_metadata::read(recorded[face], triangle.first, vertex, uv));
                    MINI_CHECK(!mesh_parametric_metadata::read(ordinary[face], triangle.first, vertex, absent));
                    MINI_CHECK(surface.point_at(uv[0], uv[1]).distance(recorded[face].vertex.at(vertex).position()) < .005);
                }
            }
        }
    }
    for (int path : {1, 2, 3, 4})
        MINI_CHECK(paths.count(path) == 1);
}

MINI_TEST("BRep Native Tessellate", "Close Disjoint Solids Keep Separate Ownership") {
    const BRep first = BRep::create_box(2, 2, 2);
    const BRep second = first.transformed(Xform::translation(2.00001, 0, 0));
    std::vector<brep_assemble::FaceUse> uses;
    for (int face = 0; face < first.face_count(); ++face)
        uses.push_back({&first, face, false, nullptr, 0});
    for (int face = 0; face < second.face_count(); ++face)
        uses.push_back({&second, face, false, nullptr, 1});
    const BRep shape = brep_assemble::assemble(uses, 1e-8);
    MINI_CHECK(shape.solid_count() == 2);
    const Mesh mesh = shape.tessellate(.01, .15);
    MINI_CHECK(mesh.is_closed());
    std::map<size_t, std::set<size_t>> neighbours;
    for (const auto& face : mesh.face)
        for (size_t i = 0; i < face.second.size(); ++i) {
            const size_t a = face.second[i], b = face.second[(i + 1) % face.second.size()];
            neighbours[a].insert(b);
            neighbours[b].insert(a);
        }
    int components = 0;
    std::set<size_t> remaining;
    for (const auto& entry : neighbours)
        remaining.insert(entry.first);
    while (!remaining.empty()) {
        ++components;
        std::vector<size_t> queue{*remaining.begin()};
        remaining.erase(queue[0]);
        for (size_t i = 0; i < queue.size(); ++i)
            for (size_t vertex : neighbours[queue[i]])
                if (remaining.erase(vertex))
                    queue.push_back(vertex);
    }
    MINI_CHECK(components == 2);
}

MINI_TEST("BRep Native Tessellate", "Original Chair Trimmed Faces") {
    const std::filesystem::path source = std::filesystem::path(__FILE__).parent_path().parent_path();
    const std::vector<BRep> chairs = file_step::read_file_step_breps((source / "tests/fixtures/solidbrep.stp").string());
    MINI_CHECK(chairs.size() == 2);
    for (const BRep& chair : chairs) {
        const Mesh mesh = chair.tessellate(.005, .15);
        MINI_CHECK(!mesh.is_empty() && mesh.is_closed());
        MINI_CHECK(std::abs(std::abs(mesh.volume()) - chair.volume_exact()) < chair.volume_exact() * .01);
    }
}

MINI_TEST("BRep Native Tessellate", "Shared C0 Boundary Nodes Precede Face Triangulation") {
    BRep shape = BRep::create_box(2, 2, 2);
    const auto warped = [](const Point& point) {
        return Point(point[0], point[1], point[2] + .25 * point[0] * point[1]);
    };
    for (BRepVertex& vertex : shape.m_vertices)
        vertex.point = warped(vertex.point);
    for (NurbsCurve& curve : shape.m_curves_3d)
        for (int i = 0; i < curve.cv_count(); ++i)
            MINI_CHECK(curve.set_cv(i, warped(curve.get_cv(i))));
    for (NurbsSurface& surface : shape.m_surfaces)
        for (int i = 0; i < surface.cv_count(0); ++i)
            for (int j = 0; j < surface.cv_count(1); ++j)
                MINI_CHECK(surface.set_cv(i, j, warped(surface.get_cv(i, j))));
    int cap = -1;
    for (int face = 0; face < shape.face_count(); ++face)
        if (!shape.m_surfaces[shape.m_faces[face].surface_index].is_planar()) {
            cap = face;
            break;
        }
    MINI_CHECK(cap >= 0);
    NurbsSurface& support = shape.m_surfaces[shape.m_faces[cap].surface_index];
    const std::pair<double, double> u = support.domain(0), v = support.domain(1);
    const double knot = u.first + (u.second - u.first) * .37109375;
    MINI_CHECK(support.insert_nurbsknot(0, knot));
    for (const BRepRef& wire : shape.m_faces[cap].wires)
        for (const BRepRef& use : shape.wire_edges(wire)) {
            const int pc = shape.pcurve_index(use.index, cap, use.orientation);
            MINI_CHECK(pc >= 0 && shape.m_curves_2d[pc].increase_degree(2));
        }
    for (const BRepEdge& edge : shape.m_edges)
        for (const BRepCurveOnSurface& relation : edge.pcurves)
            for (int pc : {relation.curve_2d_index, relation.curve_2d_index_2})
                if (pc >= 0)
                    MINI_CHECK(curve_surface_intersection::same_parameter(shape.m_curves_3d[edge.curve_3d_index],
                        shape.m_curves_2d[pc], shape.m_surfaces[relation.surface_index], 1e-7)
                        == curve_surface_intersection::ParameterMatch::Matched);
    std::vector<Mesh> meshes;
    {
        const mesh_parametric_metadata::Capture capture;
        meshes = shape.face_meshes_q(true, 8, .001);
    }
    std::set<std::array<double, 3>> crossings;
    for (const auto& triangle : meshes[cap].face)
        for (size_t vertex : triangle.second) {
            mesh_parametric_metadata::UV uv;
            MINI_CHECK(mesh_parametric_metadata::read(meshes[cap], triangle.first, vertex, uv));
            if (uv[0] == knot && (uv[1] == v.first || uv[1] == v.second)) {
                const Point point = meshes[cap].vertex.at(vertex).position();
                crossings.insert({point[0], point[1], point[2]});
            }
        }
    MINI_CHECK(crossings.size() == 2);
    for (const std::array<double, 3>& point : crossings) {
        bool shared = false;
        for (int face = 0; face < shape.face_count(); ++face)
            if (face != cap)
                for (const auto& vertex : meshes[face].vertex) {
                    const Point other = vertex.second.position();
                    shared |= point[0] == other[0] && point[1] == other[1] && point[2] == other[2];
                }
        MINI_CHECK(shared);
    }
    const Mesh result = shape.tessellate(.005, .15);
    MINI_CHECK(result.is_closed());
    MINI_CHECK(std::abs(std::abs(result.volume()) - 8) < .08);
}

}
