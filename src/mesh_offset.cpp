#include "mesh_offset.h"
#include "matrix.h"
#include <algorithm>
#include <set>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════
namespace {

/// Least-squares point on the planes, fallback fills any free direction.
Point intersect_planes(const std::vector<Plane>& planes, const Point& fallback) {

    if (planes.empty())
        return fallback;

    if (planes.size() == 1) {
        const Plane& plane = planes[0];
        const double t = -plane.d() - (plane.a() * fallback[0] + plane.b() * fallback[1] + plane.c() * fallback[2]);

        return fallback + plane.z_axis() * t;
    }

    const double eps = 1e-8;

    Matrix lhs(3, 3);
    Matrix rhs(3, 1);

    for (const Plane& plane : planes) {
        const double row[3] = {plane.a(), plane.b(), plane.c()};

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j)
                lhs(i, j) += row[i] * row[j];

            rhs(i, 0) -= row[i] * plane.d();
        }
    }

    for (int i = 0; i < 3; ++i) {
        lhs(i, i) += eps;
        rhs(i, 0) += eps * fallback[i];
    }

    const std::optional<Matrix> solution = lhs.solve(rhs);

    if (!solution.has_value())
        return fallback;

    return Point((*solution)(0, 0), (*solution)(1, 0), (*solution)(2, 0));
}

/// Naked edges wound the way their face walks them.
std::vector<std::pair<size_t, size_t>> boundary_edges(const Mesh& mesh) {

    std::set<std::pair<size_t, size_t>> directed;

    for (size_t fkey : mesh.faces()) {
        const std::vector<size_t>& vertices = mesh.face.at(fkey);

        for (size_t i = 0; i < vertices.size(); ++i)
            directed.insert({vertices[i], vertices[(i + 1) % vertices.size()]});
    }

    std::vector<std::pair<size_t, size_t>> edges;

    for (const std::pair<size_t, size_t>& edge : mesh.naked_edges(true))
        if (directed.count(edge))
            edges.push_back(edge);
        else
            edges.push_back({edge.second, edge.first});

    return edges;
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Static constructors
// ═══════════════════════════════════════════════════════════════════════════
Mesh MeshOffset::from_mesh(const Mesh& mesh, double distance) {

    const std::map<size_t, Plane> planes = offset_planes(mesh, distance);
    const std::map<size_t, Point> offsets = offset_vertices(mesh, planes);

    Mesh result;
    std::map<size_t, size_t> bottom;
    std::map<size_t, size_t> top;

    for (size_t vkey : mesh.vertices()) {
        bottom[vkey] = result.add_vertex(mesh.vertex_point(vkey).value());
        top[vkey] = result.add_vertex(offsets.at(vkey));
    }

    for (size_t fkey : mesh.faces()) {
        const std::vector<size_t> vertices = mesh.face_vertices(fkey).value();
        std::vector<size_t> bottom_face;
        std::vector<size_t> top_face;

        for (size_t vkey : vertices) {
            bottom_face.push_back(bottom.at(vkey));
            top_face.push_back(top.at(vkey));
        }

        std::reverse(bottom_face.begin(), bottom_face.end());
        result.add_face(bottom_face);
        result.add_face(top_face);
    }

    for (const std::pair<size_t, size_t>& edge : boundary_edges(mesh))
        result.add_face({bottom.at(edge.first), bottom.at(edge.second), top.at(edge.second), top.at(edge.first)});

    return result;
}

MeshOffset::Layers MeshOffset::from_mesh_layers(const Mesh& mesh, double distance) {

    const std::map<size_t, Plane> planes = offset_planes(mesh, distance);
    const std::map<size_t, Point> offsets = offset_vertices(mesh, planes);

    Layers layers;
    std::map<size_t, size_t> bottom;
    std::map<size_t, size_t> top;

    for (size_t vkey : mesh.vertices()) {
        bottom[vkey] = layers.bottom.add_vertex(mesh.vertex_point(vkey).value());
        top[vkey] = layers.top.add_vertex(offsets.at(vkey));
    }

    for (size_t fkey : mesh.faces()) {
        const std::vector<size_t> vertices = mesh.face_vertices(fkey).value();
        std::vector<size_t> bottom_face;
        std::vector<size_t> top_face;

        for (size_t vkey : vertices) {
            bottom_face.push_back(bottom.at(vkey));
            top_face.push_back(top.at(vkey));
        }

        std::reverse(bottom_face.begin(), bottom_face.end());
        layers.bottom.add_face(bottom_face);
        layers.top.add_face(top_face);
    }

    std::map<size_t, size_t> side_bottom;
    std::map<size_t, size_t> side_top;

    for (const std::pair<size_t, size_t>& edge : boundary_edges(mesh)) {
        for (size_t vkey : {edge.first, edge.second}) {
            if (!side_bottom.count(vkey))
                side_bottom[vkey] = layers.sides.add_vertex(mesh.vertex_point(vkey).value());

            if (!side_top.count(vkey))
                side_top[vkey] = layers.sides.add_vertex(offsets.at(vkey));
        }

        layers.sides.add_face({side_bottom.at(edge.first), side_bottom.at(edge.second), side_top.at(edge.second), side_top.at(edge.first)});
    }

    return layers;
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometry
// ═══════════════════════════════════════════════════════════════════════════
std::map<size_t, Plane> MeshOffset::offset_planes(const Mesh& mesh, double distance) {

    std::map<size_t, Plane> planes;

    for (size_t fkey : mesh.faces()) {
        const std::optional<Point> centroid = mesh.face_centroid(fkey);
        const std::optional<Vector> normal = mesh.face_normal(fkey);

        if (!centroid.has_value() || !normal.has_value())
            continue;

        planes[fkey] = Plane::from_point_normal(*centroid + *normal * distance, *normal);
    }

    return planes;
}

std::map<size_t, Point> MeshOffset::offset_vertices(const Mesh& mesh, const std::map<size_t, Plane>& planes) {

    std::map<size_t, std::vector<size_t>> vertex_faces;

    for (size_t fkey : mesh.faces())
        for (size_t vkey : mesh.face.at(fkey))
            vertex_faces[vkey].push_back(fkey);

    std::map<size_t, Point> result;

    for (size_t vkey : mesh.vertices()) {
        const std::optional<Point> point = mesh.vertex_point(vkey);

        if (!point.has_value())
            continue;

        std::vector<Plane> adjacent;

        for (size_t fkey : vertex_faces[vkey]) {
            const auto found = planes.find(fkey);

            if (found != planes.end())
                adjacent.push_back(found->second);
        }

        result[vkey] = intersect_planes(adjacent, *point);
    }

    return result;
}

} // namespace session_cpp
