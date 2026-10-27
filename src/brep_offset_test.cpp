#include "brep_offset.h"
#include "brep_assemble.h"
#include "brep_classify.h"
#include "brep_measure.h"
#include "mini_test.h"
#include "mesh.h"
#include "primitives.h"
#include "session.h"
#include "split.h"
#include "tolerance.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>

namespace session_cpp {
namespace {

constexpr BRepOrientation Forward = BRepOrientation::Forward;
constexpr BRepOrientation Reversed = BRepOrientation::Reversed;

NurbsSurface brep_offset_saddle() {
    std::vector<Point> controls;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            controls.push_back({double(i), double(j), 0.3 * (i - 1.5) * (j - 1.5)});
    return NurbsSurface::create(
        false,
        false,
        3,
        3,
        4,
        4,
        controls
    );
}

BRep natural_face(const NurbsSurface& surface) {
    BRep result;
    const int si = result.add_surface(surface);
    const std::pair<double, double> u = surface.domain(0);
    const std::pair<double, double> v = surface.domain(1);
    const std::array<Point, 4> uv = {
        Point(u.first, v.first, 0), Point(u.second, v.first, 0),
        Point(u.second, v.second, 0), Point(u.first, v.second, 0)
    };
    for (const Point& point : uv)
        result.add_vertex(surface.point_at(point[0], point[1]));
    const std::array<double, 4> at = {v.first, u.second, v.second, u.first};
    std::vector<BRepRef> edges;
    for (int i = 0; i < 4; ++i) {
        NurbsCurve world = surface.iso_curve(i % 2, at[i]);
        if (i >= 2)
            MINI_CHECK(world.reverse());
        NurbsCurve pcurve = NurbsCurve::create(false, 1, {uv[i], uv[(i + 1) % 4]});
        MINI_CHECK(pcurve.set_domain(-7.0, -3.0));
        MINI_CHECK(world.set_domain(-7.0, -3.0));
        const int edge = result.add_edge(result.add_curve_3d(world), i, (i + 1) % 4);
        result.add_pcurve(edge, si, result.add_curve_2d(pcurve));
        edges.push_back({edge, Forward});
    }
    result.add_face(si, {{result.add_wire(edges), Forward}});
    return result;
}

void check_offset_boundaries(const BRep& result, double tolerance) {
    MINI_CHECK(result.is_valid());
    for (const BRepEdge& edge : result.m_edges) {
        for (const BRepCurveOnSurface& relation : edge.pcurves) {
            const NurbsSurface& surface = result.m_surfaces[relation.surface_index];
            for (int index : {relation.curve_2d_index, relation.curve_2d_index_2}) {
                if (index < 0)
                    continue;
                const NurbsCurve& pcurve = result.m_curves_2d[index];
                for (int i = 0; i <= 401; ++i) {
                    const double t = pcurve.domain_start() + (pcurve.domain_end() - pcurve.domain_start()) * i / 401.0;
                    const Point uv = pcurve.point_at(t);
                    const Point expected = surface.point_at(uv[0], uv[1]);
                    const Point actual = edge.degenerated ? result.m_vertices[edge.start_vertex].point
                        : result.m_curves_3d[edge.curve_3d_index].point_at(t);
                    MINI_CHECK(actual.distance(expected) < tolerance);
                }
            }
        }
    }
}

void check_offset_surface(
    const BRep& source,
    const BRep& result,
    double distance,
    double tolerance
) {
    const NurbsSurface& a = source.m_surfaces[source.m_faces[0].surface_index];
    const NurbsSurface& b = result.m_surfaces[0];
    const std::pair<double, double> u = a.domain(0);
    const std::pair<double, double> v = a.domain(1);
    MINI_CHECK(u == b.domain(0) && v == b.domain(1));
    const double sign = source.face_orientation(0) == Reversed ? -1.0 : 1.0;
    for (int i = 1; i < 37; ++i)
        for (int j = 1; j < 31; ++j) {
            const double x = u.first + (u.second - u.first) * i / 37.0;
            const double y = v.first + (v.second - v.first) * j / 31.0;
            const Point expected = a.point_at(x, y) + a.normal_at(x, y) * (sign * distance);
            MINI_CHECK(expected.distance(b.point_at(x, y)) < tolerance);
        }
}

bool offset_rejected(const BRep& source, double distance, double tolerance = 1e-7) {
    try {
        brep_offset::shape(source, distance, tolerance);
        return false;
    } catch (const std::exception&) {
        return true;
    }
}

void save_offset(const std::string& name, const BRep& source) {
    const std::filesystem::path folder = "build/native-geometry-tests/viewer";
    std::filesystem::create_directories(folder);
    BRep geometry = source;
    geometry.name = name;
    geometry.surfacecolor = Color(180 / 255.f, 110 / 255.f, 155 / 255.f);
    geometry.file_json_dump((folder / (name + ".brep.json")).string());
    Session scene(name);
    scene.add_brep(geometry);
    scene.pb_dump((folder / (name + ".pb")).string());
}

BRep offset_fixture(const BRep& source, double distance, double tolerance, const std::string& name) {
    try {
        return brep_offset::shape(source, distance, tolerance);
    } catch (const brep_assemble::ParameterError& error) {
        const nlohmann::ordered_json fixture = {
            {"curve", error.curve.jsondump()}, {"pcurve", error.pcurve.jsondump()},
            {"surface", error.surface.jsondump()}, {"tolerance", error.tolerance},
            {"match", static_cast<int>(error.match)}, {"distance", distance}, {"failure", error.what()}
        };
        std::ofstream("build/native-geometry-tests/viewer/" + name + "_parameter_failure.json") << fixture.dump(2);
        throw;
    }
}

BRep offset_saddle_solid() {
    BRep result = BRep::create_box(2, 2, 2);
    const auto warp = [](const Point& p) { return Point(p[0], p[1], p[2] + 0.16 * p[0] * p[1]); };
    for (BRepVertex& vertex : result.m_vertices)
        vertex.point = warp(vertex.point);
    for (NurbsCurve& curve : result.m_curves_3d)
        for (int i = 0; i < curve.cv_count(); ++i)
            curve.set_cv(i, warp(curve.get_cv(i)));
    for (NurbsSurface& surface : result.m_surfaces)
        for (int i = 0; i < surface.cv_count(0); ++i)
            for (int j = 0; j < surface.cv_count(1); ++j)
                surface.set_cv(i, j, warp(surface.get_cv(i, j)));
    return result;
}

BRep inward_freeform_fixture(bool nonzero_domains) {
    BRep result = BRep::create_box(2, 2, 2);
    const auto warp = [](const Point& p) {
        return Point(p[0] + 0.12 * p[1] * p[2], p[1] + 0.09 * p[0] * p[2], p[2] + 0.16 * p[0] * p[1]);
    };
    for (BRepVertex& vertex : result.m_vertices)
        vertex.point = warp(vertex.point);
    for (NurbsCurve& curve : result.m_curves_3d)
        for (int i = 0; i < curve.cv_count(); ++i)
            MINI_CHECK(curve.set_cv(i, warp(curve.get_cv(i))));
    for (size_t index = 0; index < result.m_surfaces.size(); ++index) {
        NurbsSurface& surface = result.m_surfaces[index];
        for (int i = 0; i < surface.cv_count(0); ++i)
            for (int j = 0; j < surface.cv_count(1); ++j)
                MINI_CHECK(surface.set_cv(i, j, warp(surface.get_cv(i, j))));
        if (!nonzero_domains)
            continue;
        const std::array<std::pair<double, double>, 2> before = {surface.domain(0), surface.domain(1)};
        const std::array<std::pair<double, double>, 2> after = {
            std::pair<double, double>{3.0 + index, 7.0 + index}, {-2.0 - index * 0.5, 4.0 + index * 0.25}
        };
        for (int dir = 0; dir < 2; ++dir)
            MINI_CHECK(surface.set_domain(dir, after[dir].first, after[dir].second));
        for (const BRepEdge& edge : result.m_edges)
            for (const BRepCurveOnSurface& relation : edge.pcurves)
                if (relation.surface_index == static_cast<int>(index))
                    for (int pc_index : {relation.curve_2d_index, relation.curve_2d_index_2}) {
                        if (pc_index < 0)
                            continue;
                        NurbsCurve& pc = result.m_curves_2d[pc_index];
                        MINI_CHECK(!pc.is_rational());
                        for (int i = 0; i < pc.cv_count(); ++i) {
                            Point uv = pc.get_cv(i);
                            for (int dir = 0; dir < 2; ++dir)
                                uv[dir] = after[dir].first + (uv[dir] - before[dir].first)
                                    * (after[dir].second - after[dir].first) / (before[dir].second - before[dir].first);
                            MINI_CHECK(pc.set_cv(i, uv));
                        }
                    }
    }
    return result;
}

nlohmann::ordered_json inward_incidence(const BRep& source) {
    std::vector<std::vector<int>> faces(source.edge_count());
    std::vector<std::vector<int>> vertices(source.vertex_count());
    for (int face = 0; face < source.face_count(); ++face)
        for (const BRepRef& wire : source.m_faces[face].wires)
            for (const BRepRef& use : source.wire_edges(wire)) {
                faces[use.index].push_back(face);
                const BRepEdge& edge = source.m_edges[use.index];
                vertices[edge.start_vertex].push_back(face);
                vertices[edge.end_vertex].push_back(face);
            }
    nlohmann::ordered_json result = {{"edges", nlohmann::ordered_json::array()}, {"triple_vertices", nlohmann::ordered_json::array()}};
    for (int i = 0; i < source.edge_count(); ++i) {
        MINI_CHECK(faces[i].size() == 2);
        result["edges"].push_back({{"source_edge", i}, {"faces", faces[i]},
            {"vertices", {source.m_edges[i].start_vertex, source.m_edges[i].end_vertex}}});
    }
    for (int i = 0; i < source.vertex_count(); ++i) {
        std::vector<int>& incident = vertices[i];
        std::sort(incident.begin(), incident.end());
        incident.erase(std::unique(incident.begin(), incident.end()), incident.end());
        MINI_CHECK(incident.size() == 3);
        result["triple_vertices"].push_back({{"source_vertex", i}, {"faces", incident}});
    }
    return result;
}

}

MINI_TEST("BRepOffset", "Trimmed Planar Face With Curved Hole") {
    NurbsCurve outer = Primitives::circle(0, 0, 0, 2);
    NurbsCurve hole = Primitives::circle(0.3, 0.2, 0, 0.4);
    MINI_CHECK(hole.reverse());
    const BRep source = BRep::from_nurbscurves({outer}, {{hole}});
    MINI_CHECK(source.is_valid() && source.face_count() == 1);
    for (double distance : {-0.3, 0.3}) {
        const BRep result = brep_offset::shape(source, distance, 1e-7);
        MINI_CHECK(result.wire_count() == 2);
        MINI_CHECK(result.edge_count() == source.edge_count());
        MINI_CHECK(result.solid_count() == 0);
        check_offset_surface(source, result, distance, 1e-11);
        check_offset_boundaries(result, 1e-7);
        for (size_t i = 0; i < source.m_curves_2d.size(); ++i)
            MINI_CHECK(source.m_curves_2d[i].m_cv == result.m_curves_2d[i].m_cv);
    }
}

MINI_TEST("BRepOffset", "Doubly Curved Oriented Open Shell") {
    NurbsSurface surface = brep_offset_saddle();
    MINI_CHECK(surface.set_domain(0, 11.0, 17.0));
    MINI_CHECK(surface.set_domain(1, -8.0, -2.0));
    BRep source = natural_face(surface);
    const int shell = source.add_shell({{0, Forward}});
    for (BRepOrientation orientation : {Forward, Reversed}) {
        source.m_shells[shell].faces[0].orientation = orientation;
        for (double distance : {-0.15, 0.15}) {
            const BRep result = brep_offset::shape(source, distance, 2e-6);
            MINI_CHECK(result.shell_count() == 1 && result.solid_count() == 0);
            MINI_CHECK(result.face_orientation(0) == orientation);
            check_offset_surface(source, result, distance, 2e-6);
            check_offset_boundaries(result, 2e-6);
            for (const NurbsCurve& curve : result.m_curves_3d)
                MINI_CHECK(curve.domain_start() == -7.0 && curve.domain_end() == -3.0);
        }
    }
}

MINI_TEST("BRepOffset", "Sphere Solid Seams Poles And Classification") {
    const BRep source = BRep::create_sphere(2.0);
    for (double distance : {-0.4, 0.4}) {
        const BRep result = brep_offset::shape(source, distance, 1e-7);
        MINI_CHECK(result.is_solid());
        MINI_CHECK(result.vertex_count() == 2 && result.edge_count() == 3);
        int poles = 0;
        int seams = 0;
        for (const BRepEdge& edge : result.m_edges) {
            poles += edge.degenerated ? 1 : 0;
            seams += edge.pcurves[0].curve_2d_index_2 >= 0 ? 1 : 0;
        }
        MINI_CHECK(poles == 2 && seams == 1);
        check_offset_boundaries(result, 1e-10);
        check_offset_surface(source, result, distance, 1e-10);
        MINI_CHECK(brep_classify::point(result, {0, 0, 0}, 1e-7) == brep_classify::State::Inside);
        MINI_CHECK(brep_classify::point(result, {0, 0, 2.0 + distance}, 1e-7) == brep_classify::State::Boundary);
        MINI_CHECK(brep_classify::point(result, {2.01 + distance, 0, 0}, 1e-7) == brep_classify::State::Outside);
    }
}

MINI_TEST("BRepOffset", "Torus Solid Double Seam And Placement") {
    BRep source = BRep::create_torus(2.0, 0.5);
    for (double distance : {-0.1, 0.1}) {
        const BRep result = brep_offset::shape(source, distance, 1e-7);
        MINI_CHECK(result.is_solid());
        MINI_CHECK(result.vertex_count() == 1 && result.edge_count() == 2);
        for (const BRepEdge& edge : result.m_edges)
            MINI_CHECK(edge.pcurves[0].curve_2d_index_2 >= 0);
        check_offset_boundaries(result, 1e-10);
        check_offset_surface(source, result, distance, 1e-10);
        MINI_CHECK(brep_classify::point(result, {0, 0, 0}, 1e-7) == brep_classify::State::Outside);
        MINI_CHECK(brep_classify::point(result, {2, 0, 0}, 1e-7) == brep_classify::State::Inside);
        MINI_CHECK(brep_classify::point(result, {2.51 + distance, 0, 0}, 1e-7) == brep_classify::State::Outside);
    }
    source.transform(Xform::rotation_x(0.4));
    source.transform(Xform::translation(3, -4, 5));
    const BRep placed = brep_offset::shape(source, 0.1, 1e-7);
    MINI_CHECK(placed.is_solid());
    check_offset_surface(source, placed, 0.1, 1e-10);
    check_offset_boundaries(placed, 1e-10);
}

MINI_TEST("BRepOffset", "Collapsed Solids And Unresolved Inward Joins") {
    const BRep sphere = BRep::create_sphere(2.0);
    MINI_CHECK(offset_rejected(sphere, -2.0));
    MINI_CHECK(offset_rejected(sphere, -3.0));
    const BRep torus = BRep::create_torus(2.0, 0.5);
    MINI_CHECK(offset_rejected(torus, -0.5));
    MINI_CHECK(offset_rejected(torus, 1.5));
    const BRep box = BRep::create_box(2, 2, 2);
    MINI_CHECK(offset_rejected(box, -1.0));
    MINI_CHECK(offset_rejected(box, -1.1));
    MINI_CHECK(sphere.m_vertices[0].point.distance({0, 0, -2}) < 1e-12);
    MINI_CHECK(box.is_solid());
}

MINI_TEST("BRepOffset", "Smooth Partitioned Sphere Both Signs") {
    const BRep sphere = BRep::create_sphere(2.0);
    const NurbsSurface& surface = sphere.m_surfaces[0];
    const std::pair<double, double> u = surface.domain(0);
    const std::pair<double, double> v = surface.domain(1);
    const double middle = (v.first + v.second) * 0.5;
    NurbsCurve world = surface.iso_curve(0, middle);
    NurbsCurve pcurve = NurbsCurve::create(false, 1, {{u.first, middle, 0}, {u.second, middle, 0}});
    MINI_CHECK(pcurve.set_domain(world.domain_start(), world.domain_end()));
    const BRep source = split::split_brep_face_by_pcurves(sphere, 0, {{world, pcurve, 0}}, 1e-7).brep;
    MINI_CHECK(source.is_solid() && source.face_count() == 2);
    save_offset("native_offset_partitioned_sphere_source", source);
    for (double distance : {-0.2, 0.2}) {
        const BRep result = brep_offset::shape(source, distance, 1e-6);
        MINI_CHECK(result.is_solid() && result.face_count() == 2);
        check_offset_boundaries(result, 1e-6);
        const double radius = 2.0 + distance;
        MINI_CHECK(std::abs(brep_measure::volume(result, 1e-8) - 4.0 * Tolerance::PI * radius * radius * radius / 3.0) < 1e-6);
        save_offset(distance > 0.0 ? "native_offset_partitioned_sphere_positive" : "native_offset_partitioned_sphere_negative", result);
    }
}

MINI_TEST("BRepOffset", "Rounded Box Edge Tubes And Vertex Spheres") {
    const BRep source = BRep::create_box(2, 2, 2);
    const double distance = 0.2;
    const double tolerance = 2e-5;
    save_offset("native_offset_rounded_box_source", source);
    const BRep result = offset_fixture(source, distance, tolerance, "native_offset_rounded_box");
    MINI_CHECK(result.is_solid() && result.solid_count() == 1);
    MINI_CHECK(result.face_count() == 42);
    check_offset_boundaries(result, tolerance);
    const double expected = 8.0 + 24.0 * distance + 6.0 * Tolerance::PI * distance * distance
        + 4.0 * Tolerance::PI * distance * distance * distance / 3.0;
    MINI_CHECK(std::abs(brep_measure::volume(result, 1e-6) - expected) < 2e-4);
    MINI_CHECK(brep_classify::point(result, {1.19, 0, 0}, tolerance) == brep_classify::State::Inside);
    MINI_CHECK(brep_classify::point(result, {1.15, 1.15, 0}, tolerance) == brep_classify::State::Outside);
    save_offset("native_offset_rounded_box_result", result);
}

MINI_TEST("BRepOffset", "Closed Curved Sharp Edges On Cylinder") {
    const BRep source = BRep::create_cylinder(1.0, 2.0);
    const double tolerance = 2e-5;
    save_offset("native_offset_cylinder_solid_source", source);
    const BRep result = offset_fixture(source, 0.15, tolerance, "native_offset_cylinder_solid");
    MINI_CHECK(result.is_solid() && result.solid_count() == 1);
    MINI_CHECK(result.face_count() == source.face_count() + 2);
    check_offset_boundaries(result, tolerance);
    MINI_CHECK(brep_measure::volume(result, 1e-6) > brep_measure::volume(source, 1e-6));
    save_offset("native_offset_cylinder_solid_result", result);
}

MINI_TEST("BRepOffset", "Doubly Curved Sharp Join Solid") {
    const BRep source = offset_saddle_solid();
    const double distance = 0.08;
    const double tolerance = 3e-5;
    MINI_CHECK(source.is_solid());
    save_offset("native_offset_saddle_solid_source", source);
    const BRep result = offset_fixture(source, distance, tolerance, "native_offset_saddle_solid");
    MINI_CHECK(result.is_solid() && result.solid_count() == 1);
    MINI_CHECK(result.face_count() == 42);
    check_offset_boundaries(result, tolerance);
    MINI_CHECK(brep_measure::volume(result, 1e-6) > 8.0);
    for (int face = 0; face < source.face_count(); ++face) {
        const NurbsSurface& surface = source.m_surfaces[source.m_faces[face].surface_index];
        const std::pair<double, double> u = surface.domain(0);
        const std::pair<double, double> v = surface.domain(1);
        const double x = u.first + 0.43 * (u.second - u.first);
        const double y = v.first + 0.37 * (v.second - v.first);
        const double sign = source.face_orientation(face) == Reversed ? -1.0 : 1.0;
        const Point expected = surface.point_at(x, y) + surface.normal_at(x, y) * (sign * distance);
        MINI_CHECK(brep_classify::point(result, expected, tolerance * 2.0) == brep_classify::State::Boundary);
    }
    save_offset("native_offset_saddle_solid_result", result);
}

MINI_TEST("BRepOffset", "Inward Adjacent Freeform Fixture Prerequisites") {
    for (bool nonzero_domains : {false, true}) {
        const BRep source = inward_freeform_fixture(nonzero_domains);
        const std::string name = nonzero_domains ? "native_offset_inward_freeform_nonzero_domains" : "native_offset_inward_freeform";
        save_offset(name + "_source", source);
        MINI_CHECK(source.is_solid() && source.face_count() == 6 && source.edge_count() == 12 && source.vertex_count() == 8);
        const double expected = 8 * (1 - (0.12 * 0.09 + 0.12 * 0.16 + 0.09 * 0.16) / 3);
        const double volume = brep_measure::volume(source, 1e-9);
        MINI_CHECK(std::abs(volume - expected) < 1e-8);
        check_offset_boundaries(source, 1e-10);
        nlohmann::ordered_json record = inward_incidence(source);
        record["scope"] = "source fixture prerequisites only; inward intersection cleanup is not implemented";
        record["warp_coefficients"] = {0.12, 0.09, 0.16};
        record["jacobian_lower_bound"] = 0.952144;
        record["expected_source_volume"] = expected;
        record["source_volume"] = volume;
        record["planned_distances"] = {-0.04, -0.16};
        record["planned_tolerance"] = 3e-5;
        record["face_curvatures"] = nlohmann::ordered_json::array();
        for (const NurbsSurface& surface : source.m_surfaces) {
            MINI_CHECK(!surface.is_planar());
            const std::pair<double, double> u = surface.domain(0);
            const std::pair<double, double> v = surface.domain(1);
            const double curvature = surface.gaussian_curvature((u.first + u.second) * 0.5, (v.first + v.second) * 0.5);
            MINI_CHECK(curvature < -1e-4);
            record["face_curvatures"].push_back(curvature);
        }
        std::ofstream("build/native-geometry-tests/viewer/" + name + ".prerequisites.json") << record.dump(2);
    }
}

MINI_TEST("BRepOffset", "Inward Convex Platonic Solids And Collapse") {
    const std::array<Mesh, 5> meshes = {
        Primitives::tetrahedron(), Primitives::cube(), Primitives::octahedron(),
        Mesh::create_dodecahedron(), Primitives::icosahedron()
    };
    for (size_t i = 0; i < meshes.size(); ++i) {
        std::vector<Polyline> polygons;
        for (const std::pair<const size_t, std::vector<size_t>>& face : meshes[i].face)
            polygons.push_back(meshes[i].face_polygon(face.first).value());
        const BRep source = BRep::from_polylines(polygons);
        const std::vector<Plane> planes = source.face_planes();
        const double radius = planes[0].z_axis().dot(planes[0].origin() - Point());
        save_offset("native_offset_inward_platonic_source_" + std::to_string(i), source);
        const BRep result = brep_offset::shape(source, -0.2 * radius, 1e-7);
        MINI_CHECK(result.is_solid() && result.solid_count() == 1);
        MINI_CHECK(result.face_count() == source.face_count());
        MINI_CHECK(std::abs(brep_measure::volume(result, 1e-8) / brep_measure::volume(source, 1e-8) - 0.512) < 1e-7);
        check_offset_boundaries(result, 1e-7);
        MINI_CHECK(offset_rejected(source, -1.1 * radius));
        save_offset("native_offset_inward_platonic_result_" + std::to_string(i), result);
    }
}

MINI_TEST("BRepOffset", "Disconnected Pinched Shell Rejected") {
    const BRep first = BRep::create_box(2, 2, 2);
    const BRep second = first.transformed(Xform::translation(2, 2, 2));
    std::vector<brep_assemble::FaceUse> faces;
    for (int face = 0; face < first.face_count(); ++face) {
        faces.push_back({&first, face});
        faces.push_back({&second, face});
    }
    BRep source = brep_assemble::assemble(faces, 1e-7);
    MINI_CHECK(source.shell_count() == 2 && source.solid_count() == 2);
    const std::vector<BRepRef> extra = source.m_shells[1].faces;
    source.m_shells[0].faces.insert(source.m_shells[0].faces.end(), extra.begin(), extra.end());
    source.m_shells.resize(1);
    source.m_solids.resize(1);
    source.m_solids[0].shells = {{0, Forward}};
    MINI_CHECK(source.is_solid());
    save_offset("native_offset_pinched_disconnected_shell", source);
    MINI_CHECK(offset_rejected(source, 0.1));
}

MINI_TEST("BRepOffset", "C0 Creases And Smooth Rational Seams") {
    NurbsSurface crease(3, false, 2, 2, 3, 2);
    crease.m_nurbsknot[0] = {3, 4, 7};
    crease.m_nurbsknot[1] = {-2, 4};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 2; ++j)
            crease.set_cv(i, j, {double(i - 1), double(j), double(std::abs(i - 1))});
    const BRep folded = natural_face(crease);
    save_offset("native_offset_c0_crease_source", folded);
    nlohmann::ordered_json records = nlohmann::ordered_json::array();
    for (double tolerance : {1e-5, 0.1})
        for (double distance : {-0.01, 0.01}) {
            std::string failure;
            try {
                brep_offset::shape(folded, distance, tolerance);
            } catch (const std::exception& error) {
                failure = error.what();
            }
            records.push_back({{"distance", distance}, {"tolerance", tolerance}, {"failure", failure}});
            std::ofstream("build/native-geometry-tests/viewer/native_offset_c0_crease.assertions.json") << records.dump(2);
            MINI_CHECK(failure.find("discontinuous one-sided normals") != std::string::npos);
        }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 2; ++j)
            crease.set_cv(i, j, {double(i - 1), double(j), 0});
    const BRep flat = natural_face(crease);
    save_offset("native_offset_c0_parameter_speed_source", flat);
    const auto stage = [](const std::string& name) {
        std::ofstream("build/native-geometry-tests/viewer/native_offset_c0_stage.json")
            << nlohmann::ordered_json({{"stage", name}}).dump(2);
    };
    stage("piecewise_planar_offset");
    const BRep shifted = brep_offset::shape(flat, 0.1, 1e-7);
    check_offset_surface(flat, shifted, 0.1, 1e-10);
    check_offset_boundaries(shifted, 1e-9);
    save_offset("native_offset_c0_parameter_speed_plane", shifted);
    for (const BRep source : {BRep::create_sphere(1), BRep::create_cylinder(1, 2)}) {
        stage(source.face_count() == 1 ? "sphere_offset" : "cylinder_offset");
        const BRep result = brep_offset::shape(source, 0.1, 2e-5);
        MINI_CHECK(result.is_solid());
        check_offset_boundaries(result, 2e-5);
    }
    stage("complete");
}

MINI_TEST("BRepOffset", "Active Topology Orientation And Input Validation") {
    BRep source = BRep::create_sphere(2.0);
    source.m_solids[0].shells[0].orientation = Reversed;
    source.m_shells[0].faces[0].orientation = Reversed;
    source.add_surface(brep_offset_saddle());
    const BRep result = brep_offset::shape(source, 0.2, 1e-7);
    MINI_CHECK(result.is_solid() && result.m_surfaces.size() == 1);
    check_offset_surface(source, result, 0.2, 1e-10);
    const BRep unchanged = brep_offset::shape(source, 0.0, 1e-7);
    MINI_CHECK(unchanged.m_surfaces.size() == source.m_surfaces.size());
    MINI_CHECK(unchanged.m_surfaces[0].m_cv == source.m_surfaces[0].m_cv);
    MINI_CHECK(offset_rejected(source, 0.1, 0.0));
    MINI_CHECK(offset_rejected(source, std::numeric_limits<double>::infinity()));
    BRep open = natural_face(brep_offset_saddle());
    open.add_shell({{0, Forward}, {0, Reversed}});
    MINI_CHECK(offset_rejected(open, 0.1));

    BRep invalid = BRep::create_torus(2.0, 0.5);
    for (double& value : invalid.m_surfaces[0].m_cv)
        value *= -1.0;
    MINI_CHECK(offset_rejected(invalid, 0.1));
    invalid = BRep::create_torus(2.0, 0.5);
    invalid.transform(Xform::translation(1e12, 0, 0));
    MINI_CHECK(offset_rejected(invalid, 0.1, 1e-8));

    const NurbsCurve circle = Primitives::circle(0, 0, 0, 2);
    invalid = BRep::from_nurbscurves({circle});
    for (double& value : invalid.m_curves_2d[0].m_cv)
        value *= -1.0;
    MINI_CHECK(invalid.m_curves_2d[0].is_rational());
    MINI_CHECK(offset_rejected(invalid, 0.1));
}

}
