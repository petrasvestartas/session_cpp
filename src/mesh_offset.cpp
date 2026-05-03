#include "mesh_offset.h"
#include "matrix.h"
#include "plane.h"
#include "tolerance.h"

namespace session_cpp {

// Compute offset planes: each face plane translated by distance along its normal.
static std::map<size_t, Plane> _offset_planes(const Mesh& mesh, double distance) {
    std::map<size_t, Plane> planes;
    for (size_t fk : mesh.faces()) {
        auto c = mesh.face_centroid(fk);
        auto n = mesh.face_normal(fk);
        if (!c.has_value() || !n.has_value()) continue;
        Point origin = Point(
            (*c)[0] + distance * (*n)[0],
            (*c)[1] + distance * (*n)[1],
            (*c)[2] + distance * (*n)[2]);
        Vector normal = *n;
        planes[fk] = Plane::from_point_normal(origin, normal);
    }
    return planes;
}

// Find offset position of a vertex via least-squares intersection of adjacent offset planes.
// Uses Tikhonov regularization so unconstrained directions fall back to the original position.
static Point _intersect_planes(
    const std::vector<Plane>& planes,
    const Point& fallback)
{
    int n = (int)planes.size();
    if (n == 0) return fallback;

    // For a single plane: project fallback onto it.
    if (n == 1) {
        double nx = planes[0].a(), ny = planes[0].b(), nz = planes[0].c();
        double d_rhs = -planes[0].d();
        double t = d_rhs - (nx * fallback[0] + ny * fallback[1] + nz * fallback[2]);
        return Point(fallback[0] + t * nx, fallback[1] + t * ny, fallback[2] + t * nz);
    }

    // Normal equations: (N^T N + eps I) x = N^T d + eps * fallback
    // eps regularization pulls unconstrained directions (e.g. flat mesh) toward fallback.
    constexpr double eps = 1e-8;
    double A[3][3] = {};
    double b[3] = {};
    for (const auto& pl : planes) {
        double r0 = pl.a(), r1 = pl.b(), r2 = pl.c();
        double d_rhs = -pl.d();
        A[0][0] += r0*r0; A[0][1] += r0*r1; A[0][2] += r0*r2;
        A[1][0] += r1*r0; A[1][1] += r1*r1; A[1][2] += r1*r2;
        A[2][0] += r2*r0; A[2][1] += r2*r1; A[2][2] += r2*r2;
        b[0] += r0 * d_rhs;
        b[1] += r1 * d_rhs;
        b[2] += r2 * d_rhs;
    }
    A[0][0] += eps; A[1][1] += eps; A[2][2] += eps;
    b[0] += eps * fallback[0];
    b[1] += eps * fallback[1];
    b[2] += eps * fallback[2];

    Matrix mat = Matrix::from_vec(3, 3, {
        A[0][0], A[0][1], A[0][2],
        A[1][0], A[1][1], A[1][2],
        A[2][0], A[2][1], A[2][2]});
    Matrix rhs = Matrix::from_vec(3, 1, {b[0], b[1], b[2]});
    auto sol = mat.solve(rhs);
    if (!sol.has_value()) return fallback;
    return Point((*sol)(0, 0), (*sol)(1, 0), (*sol)(2, 0));
}

// Compute new vertex positions from the offset planes of adjacent faces.
static std::map<size_t, Point> _offset_vertices(
    const Mesh& mesh,
    const std::map<size_t, Plane>& planes)
{
    std::map<size_t, Point> result;
    for (size_t vk : mesh.vertices()) {
        auto vp = mesh.vertex_point(vk);
        if (!vp.has_value()) continue;
        auto fkeys = mesh.vertex_faces(vk);
        if (!fkeys.has_value() || fkeys->empty()) {
            result[vk] = *vp;
            continue;
        }
        std::vector<Plane> adj;
        for (size_t fk : *fkeys) {
            auto it = planes.find(fk);
            if (it != planes.end()) adj.push_back(it->second);
        }
        result[vk] = _intersect_planes(adj, *vp);
    }
    return result;
}

Mesh MeshOffset::from_mesh(const Mesh& mesh, double distance) {
    auto planes = _offset_planes(mesh, distance);
    auto off_verts = _offset_vertices(mesh, planes);

    Mesh result;
    std::map<size_t, size_t> bot_vmap, top_vmap;
    for (size_t vk : mesh.vertices()) {
        bot_vmap[vk] = result.add_vertex(mesh.vertex_point(vk).value());
        top_vmap[vk] = result.add_vertex(off_verts[vk]);
    }

    for (size_t fk : mesh.faces()) {
        auto fv = mesh.face_vertices(fk).value();
        std::vector<size_t> bkeys, tkeys;
        for (size_t v : fv) {
            bkeys.push_back(bot_vmap[v]);
            tkeys.push_back(top_vmap[v]);
        }
        std::reverse(bkeys.begin(), bkeys.end());
        result.add_face(bkeys);
        result.add_face(tkeys);
    }

    for (const auto& [u, v] : mesh.naked_edges(true))
        result.add_face({bot_vmap[u], bot_vmap[v], top_vmap[v], top_vmap[u]});

    return result;
}

MeshOffset::Layers MeshOffset::from_mesh_layers(const Mesh& mesh, double distance) {
    auto planes = _offset_planes(mesh, distance);
    auto off_verts = _offset_vertices(mesh, planes);

    Layers layers;
    std::map<size_t, size_t> bot_vmap, top_vmap;
    for (size_t vk : mesh.vertices()) {
        bot_vmap[vk] = layers.bottom.add_vertex(mesh.vertex_point(vk).value());
        top_vmap[vk] = layers.top.add_vertex(off_verts[vk]);
    }

    for (size_t fk : mesh.faces()) {
        auto fv = mesh.face_vertices(fk).value();
        std::vector<size_t> bkeys, tkeys;
        for (size_t v : fv) {
            bkeys.push_back(bot_vmap[v]);
            tkeys.push_back(top_vmap[v]);
        }
        std::reverse(bkeys.begin(), bkeys.end());
        layers.bottom.add_face(bkeys);
        layers.top.add_face(tkeys);
    }

    std::map<size_t, size_t> s_bot, s_top;
    for (const auto& [u, v] : mesh.naked_edges(true)) {
        if (!s_bot.count(u))
            s_bot[u] = layers.sides.add_vertex(mesh.vertex_point(u).value());
        if (!s_bot.count(v))
            s_bot[v] = layers.sides.add_vertex(mesh.vertex_point(v).value());
        if (!s_top.count(u))
            s_top[u] = layers.sides.add_vertex(off_verts[u]);
        if (!s_top.count(v))
            s_top[v] = layers.sides.add_vertex(off_verts[v]);
        layers.sides.add_face({s_bot[u], s_bot[v], s_top[v], s_top[u]});
    }

    return layers;
}

std::map<size_t, Plane> MeshOffset::offset_planes(const Mesh& mesh, double distance) {
    return _offset_planes(mesh, distance);
}

std::map<size_t, Point> MeshOffset::offset_vertices(
    const Mesh& mesh,
    const std::map<size_t, Plane>& planes)
{
    return _offset_vertices(mesh, planes);
}

} // namespace session_cpp
