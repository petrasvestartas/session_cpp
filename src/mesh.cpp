#include "mesh.h"
#include "triangulation_2d.h"
#include "trimesh_cdt.h"
#include "fmt/core.h"
#include <fstream>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <limits>
#include <thread>

#include "mesh.pb.h"
#include "color.pb.h"
#include "xform.pb.h"

namespace session_cpp {

Mesh::Mesh() {
    xform = Xform::identity();
    default_vertex_attributes["x"] = 0.0;
    default_vertex_attributes["y"] = 0.0;
    default_vertex_attributes["z"] = 0.0;
}

Mesh::Mesh(const Mesh& other) {
    guid = ::guid();
    name = other.name;
    halfedge = other.halfedge;
    vertex = other.vertex;
    face = other.face;
    facedata = other.facedata;
    edgedata = other.edgedata;
    default_vertex_attributes = other.default_vertex_attributes;
    default_face_attributes = other.default_face_attributes;
    default_edge_attributes = other.default_edge_attributes;
    pointcolors = other.pointcolors;
    facecolors = other.facecolors;
    linecolors = other.linecolors;
    widths = other.widths;
    xform = other.xform;
    max_vertex = other.max_vertex;
    max_face = other.max_face;
    triangulation = other.triangulation;
}

Mesh& Mesh::operator=(const Mesh& other) {
    if (this != &other) {
        guid = ::guid();
        name = other.name;
        halfedge = other.halfedge;
        vertex = other.vertex;
        face = other.face;
        facedata = other.facedata;
        edgedata = other.edgedata;
        default_vertex_attributes = other.default_vertex_attributes;
        default_face_attributes = other.default_face_attributes;
        default_edge_attributes = other.default_edge_attributes;
        pointcolors = other.pointcolors;
        facecolors = other.facecolors;
        linecolors = other.linecolors;
        widths = other.widths;
        xform = other.xform;
        max_vertex = other.max_vertex;
        max_face = other.max_face;
        triangulation = other.triangulation;
        triangle_bvh_built = false;
        triangle_bvh.reset();
        triangle_aabb_tree.reset();
    }
    return *this;
}

bool Mesh::operator==(const Mesh& other) const {
    if (name != other.name) return false;
    if (vertex != other.vertex) return false;
    if (face != other.face) return false;
    if (xform != other.xform) return false;
    return true;
}

bool Mesh::operator!=(const Mesh& other) const { return !(*this == other); }

Mesh::~Mesh() {}

size_t Mesh::number_of_edges() const {
    std::set<std::pair<size_t, size_t>> seen;
    size_t count = 0;
    
    for (const auto& [u, neighbors] : halfedge) {
        for (const auto& [v, _] : neighbors) {
            auto edge = std::minmax(u, v);
            if (seen.insert(edge).second) {
                count++;
            }
        }
    }
    return count;
}

bool Mesh::is_valid() const {
    if (vertex.empty() || face.empty()) return false;
    for (const auto& [fkey, vkeys] : face) {
        if (vkeys.size() < 3) return false;
        for (size_t vk : vkeys) {
            if (vertex.find(vk) == vertex.end()) return false;
        }
    }
    return true;
}

int Mesh::euler() const {
    return static_cast<int>(number_of_vertices()) - 
           static_cast<int>(number_of_edges()) + 
           static_cast<int>(number_of_faces());
}

void Mesh::clear() {
    halfedge.clear();
    vertex.clear();
    face.clear();
    facedata.clear();
    edgedata.clear();
    triangulation.clear();
    max_vertex = 0;
    max_face = 0;
    pointcolors.clear();
    facecolors.clear();
    linecolors.clear();
    widths.clear();
    triangle_bvh_built = false;
    triangle_bvh.reset();
    triangle_boxes_cache.clear();
    triangle_aabbs_cache.clear();
    triangle_indices_cache.clear();
    triangle_face_subidx_cache.clear();
    vertices_cache.clear();
}

bool Mesh::unify_winding() {
    if (face.size() < 2) return false;

    std::map<std::pair<size_t,size_t>, std::vector<std::tuple<size_t,size_t,size_t>>> edge_faces;
    for (auto& [fkey, verts] : face) {
        size_t n = verts.size();
        for (size_t i = 0; i < n; ++i) {
            size_t u = verts[i];
            size_t v = verts[(i + 1) % n];
            auto edge = u < v ? std::make_pair(u, v) : std::make_pair(v, u);
            edge_faces[edge].emplace_back(fkey, u, v);
        }
    }

    std::set<size_t> visited;
    std::set<size_t> flipped;
    for (auto& [seed, _dummy] : face) {
        if (visited.count(seed)) continue;
        visited.insert(seed);
        std::vector<size_t> queue = {seed};
        while (!queue.empty()) {
            size_t f = queue.back(); queue.pop_back();
            bool is_flipped = flipped.count(f) > 0;
            const auto& verts = face[f];
            size_t n = verts.size();
            for (size_t i = 0; i < n; ++i) {
                size_t u_orig = verts[i];
                size_t v_orig = verts[(i + 1) % n];
                size_t eff_u = is_flipped ? v_orig : u_orig;
                size_t eff_v = is_flipped ? u_orig : v_orig;
                auto edge = u_orig < v_orig ? std::make_pair(u_orig, v_orig) : std::make_pair(v_orig, u_orig);
                auto it = edge_faces.find(edge);
                if (it == edge_faces.end()) continue;
                for (auto& [adj_key, adj_u, adj_v] : it->second) {
                    if (adj_key == f || visited.count(adj_key)) continue;
                    if (!(adj_u == eff_v && adj_v == eff_u))
                        flipped.insert(adj_key);
                    visited.insert(adj_key);
                    queue.push_back(adj_key);
                }
            }
        }
    }

    if (flipped.empty()) return false;

    for (size_t fkey : flipped)
        std::reverse(face[fkey].begin(), face[fkey].end());

    for (auto& [u, nbrs] : halfedge)
        nbrs.clear();
    for (auto& [fkey, verts] : face) {
        size_t n = verts.size();
        for (size_t i = 0; i < n; ++i) {
            size_t u = verts[i];
            size_t v = verts[(i + 1) % n];
            halfedge[u][v] = fkey;
            if (!halfedge[v].count(u))
                halfedge[v][u] = std::nullopt;
        }
    }

    return true;
}

size_t Mesh::add_vertex(const Point& position, std::optional<size_t> vkey) {
    size_t vertex_key = vkey.value_or(max_vertex);
    
    if (vertex_key >= max_vertex) {
        max_vertex = vertex_key + 1;
    }
    
    vertex[vertex_key] = VertexData(position);
    halfedge[vertex_key] = {};
    pointcolors.push_back(Color::white());
    triangle_bvh_built = false;
    triangle_bvh.reset();
    triangle_boxes_cache.clear();
    triangle_aabbs_cache.clear();
    triangle_indices_cache.clear();
    triangle_face_subidx_cache.clear();
    vertices_cache.clear();
    return vertex_key;
}

std::optional<size_t> Mesh::add_face(const std::vector<size_t>& vertices, std::optional<size_t> fkey) {
    if (vertices.size() < 3) {
        return std::nullopt;
    }
    
    for (size_t v : vertices) {
        if (vertex.find(v) == vertex.end()) {
            return std::nullopt;
        }
    }
    
    std::set<size_t> unique_vertices(vertices.begin(), vertices.end());
    if (unique_vertices.size() != vertices.size()) {
        return std::nullopt;
    }
    
    size_t face_key = fkey.value_or(max_face);
    
    if (face_key >= max_face) {
        max_face = face_key + 1;
    }
    
    face[face_key] = vertices;
    triangulation.erase(face_key);
    facecolors.push_back(Color::white());
    
    for (size_t i = 0; i < vertices.size(); ++i) {
        size_t u = vertices[i];
        size_t v = vertices[(i + 1) % vertices.size()];
        
        bool is_new_edge = (halfedge[v].find(u) == halfedge[v].end());
        
        halfedge[u][v] = face_key;
        
        if (is_new_edge) {
            halfedge[v][u] = std::nullopt;
            linecolors.push_back(Color::white());
            widths.push_back(1.0);
        }
    }
    triangle_bvh_built = false;
    triangle_bvh.reset();
    triangle_boxes_cache.clear();
    triangle_aabbs_cache.clear();
    triangle_indices_cache.clear();
    triangle_face_subidx_cache.clear();
    vertices_cache.clear();
    return face_key;
}

std::optional<Point> Mesh::vertex_position(size_t vertex_key) const {
    auto it = vertex.find(vertex_key);
    if (it == vertex.end()) {
        return std::nullopt;
    }
    return it->second.position();
}

std::optional<std::vector<size_t>> Mesh::face_vertices(size_t face_key) const {
    auto it = face.find(face_key);
    if (it == face.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::vector<size_t> Mesh::vertex_neighbors(size_t vertex_key) const {
    std::vector<size_t> neighbors;
    auto it = halfedge.find(vertex_key);
    if (it != halfedge.end()) {
        for (const auto& [v, _] : it->second) {
            neighbors.push_back(v);
        }
    }
    return neighbors;
}

std::vector<size_t> Mesh::vertex_faces(size_t vertex_key) const {
    std::vector<size_t> faces;
    auto it = halfedge.find(vertex_key);
    if (it == halfedge.end()) return faces;
    for (const auto& [v, face_opt] : it->second) {
        if (face_opt.has_value()) faces.push_back(*face_opt);
    }
    return faces;
}

std::vector<std::pair<size_t, size_t>> Mesh::vertex_edges(size_t vertex_key) const {
    std::vector<std::pair<size_t, size_t>> edges;
    auto it = halfedge.find(vertex_key);
    if (it == halfedge.end()) return edges;
    for (const auto& [u, _] : it->second) {
        edges.push_back({vertex_key, u});
    }
    return edges;
}

std::vector<std::pair<size_t, size_t>> Mesh::face_edges(size_t face_key) const {
    std::vector<std::pair<size_t, size_t>> edges;
    auto it = face.find(face_key);
    if (it == face.end()) return edges;
    const auto& verts = it->second;
    size_t n = verts.size();
    for (size_t i = 0; i < n; ++i) {
        edges.push_back({verts[i], verts[(i + 1) % n]});
    }
    return edges;
}

std::vector<size_t> Mesh::face_neighbors(size_t face_key) const {
    std::vector<size_t> neighbors;
    for (const auto& [u, v] : face_edges(face_key)) {
        auto it = halfedge.find(v);
        if (it == halfedge.end()) continue;
        auto jt = it->second.find(u);
        if (jt != it->second.end() && jt->second.has_value()) {
            neighbors.push_back(*jt->second);
        }
    }
    return neighbors;
}

std::array<size_t, 2> Mesh::edge_vertices(size_t u, size_t v) const {
    return {u, v};
}

std::pair<std::optional<size_t>, std::optional<size_t>> Mesh::edge_faces(size_t u, size_t v) const {
    std::optional<size_t> f0, f1;
    auto it = halfedge.find(u);
    if (it != halfedge.end()) {
        auto jt = it->second.find(v);
        if (jt != it->second.end()) f0 = jt->second;
    }
    auto it2 = halfedge.find(v);
    if (it2 != halfedge.end()) {
        auto jt2 = it2->second.find(u);
        if (jt2 != it2->second.end()) f1 = jt2->second;
    }
    return {f0, f1};
}

std::vector<std::pair<size_t, size_t>> Mesh::edge_edges(size_t u, size_t v) const {
    std::vector<std::pair<size_t, size_t>> edges;
    auto it = halfedge.find(u);
    if (it != halfedge.end()) {
        for (const auto& [w, _] : it->second) {
            if (w != v) edges.push_back({u, w});
        }
    }
    auto it2 = halfedge.find(v);
    if (it2 != halfedge.end()) {
        for (const auto& [w, _] : it2->second) {
            if (w != u) edges.push_back({v, w});
        }
    }
    return edges;
}

bool Mesh::is_edge_on_boundary(size_t u, size_t v) const {
    auto check = [&](size_t a, size_t b) {
        auto it = halfedge.find(a);
        if (it == halfedge.end()) return true;
        auto jt = it->second.find(b);
        return jt == it->second.end() || !jt->second.has_value();
    };
    return check(u, v) || check(v, u);
}

bool Mesh::is_face_on_boundary(size_t face_key) const {
    for (const auto& [u, v] : face_edges(face_key)) {
        if (is_edge_on_boundary(u, v)) return true;
    }
    return false;
}

bool Mesh::is_vertex_on_boundary(size_t vertex_key) const {
    auto it = halfedge.find(vertex_key);
    if (it == halfedge.end()) {
        return false;
    }
    
    for (const auto& [v, face_opt] : it->second) {
        if (!face_opt.has_value()) {
            return true;
        }
    }
    
    for (const auto& [u, neighbors] : halfedge) {
        auto neighbor_it = neighbors.find(vertex_key);
        if (neighbor_it != neighbors.end() && !neighbor_it->second.has_value()) {
            return true;
        }
    }
    return false;
}

std::optional<Vector> Mesh::face_normal(size_t face_key) const {
    auto vertices_opt = face_vertices(face_key);
    if (!vertices_opt.has_value() || vertices_opt->size() < 3) {
        return std::nullopt;
    }
    
    const auto& vertices = *vertices_opt;
    auto p0_opt = vertex_position(vertices[0]);
    auto p1_opt = vertex_position(vertices[1]);
    auto p2_opt = vertex_position(vertices[2]);
    
    if (!p0_opt || !p1_opt || !p2_opt) {
        return std::nullopt;
    }
    
    const auto& p0 = *p0_opt;
    const auto& p1 = *p1_opt;
    const auto& p2 = *p2_opt;
    
    Vector u(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    Vector v(p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]);
    
    Vector normal = u.cross(v);
    double len = normal.magnitude();
    
    if (len > Tolerance::ZERO_TOLERANCE) {
        return Vector(normal[0] / len, normal[1] / len, normal[2] / len);
    }
    return std::nullopt;
}

std::optional<Vector> Mesh::vertex_normal(size_t vertex_key) const {
    return vertex_normal_weighted(vertex_key, NormalWeighting::Area);
}

std::optional<Vector> Mesh::vertex_normal_weighted(size_t vertex_key, NormalWeighting weighting) const {
    auto faces = vertex_faces(vertex_key);
    if (faces.empty()) {
        return std::nullopt;
    }
    
    Vector normal_acc(0.0, 0.0, 0.0);
    
    for (size_t face_key : faces) {
        auto face_normal_opt = face_normal(face_key);
        if (!face_normal_opt) continue;
        
        const auto& fn = *face_normal_opt;
        double weight = 1.0;
        
        switch (weighting) {
            case NormalWeighting::Area:
                weight = face_area(face_key).value_or(1.0);
                break;
            case NormalWeighting::Angle:
                weight = vertex_angle_in_face(vertex_key, face_key).value_or(1.0);
                break;
            case NormalWeighting::Uniform:
                weight = 1.0;
                break;
        }
        
        normal_acc[0] = normal_acc[0] + fn[0] * weight;
        normal_acc[1] = normal_acc[1] + fn[1] * weight;
        normal_acc[2] = normal_acc[2] + fn[2] * weight;
    }
    
    double len = normal_acc.magnitude();
    if (len > Tolerance::ZERO_TOLERANCE) {
        return Vector(normal_acc[0] / len, normal_acc[1] / len, normal_acc[2] / len);
    }
    return std::nullopt;
}

std::optional<double> Mesh::face_area(size_t face_key) const {
    auto vertices_opt = face_vertices(face_key);
    if (!vertices_opt.has_value() || vertices_opt->size() < 3) {
        return 0.0;
    }
    
    const auto& vertices = *vertices_opt;
    double area = 0.0;
    auto p0_opt = vertex_position(vertices[0]);
    if (!p0_opt) return std::nullopt;
    const auto& p0 = *p0_opt;
    
    for (size_t i = 1; i < vertices.size() - 1; ++i) {
        auto p1_opt = vertex_position(vertices[i]);
        auto p2_opt = vertex_position(vertices[i + 1]);
        if (!p1_opt || !p2_opt) return std::nullopt;
        
        const auto& p1 = *p1_opt;
        const auto& p2 = *p2_opt;
        
        Vector u(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
        Vector v(p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]);
        
        area += u.cross(v).magnitude() * 0.5;
    }
    return area;
}

std::optional<double> Mesh::vertex_angle_in_face(size_t vertex_key, size_t face_key) const {
    auto vertices_opt = face_vertices(face_key);
    if (!vertices_opt) return std::nullopt;
    
    const auto& vertices = *vertices_opt;
    auto it = std::find(vertices.begin(), vertices.end(), vertex_key);
    if (it == vertices.end()) return std::nullopt;
    
    size_t vertex_index = std::distance(vertices.begin(), it);
    size_t n = vertices.size();
    size_t prev_vertex = vertices[(vertex_index + n - 1) % n];
    size_t next_vertex = vertices[(vertex_index + 1) % n];
    
    auto center_opt = vertex_position(vertex_key);
    auto prev_opt = vertex_position(prev_vertex);
    auto next_opt = vertex_position(next_vertex);
    
    if (!center_opt || !prev_opt || !next_opt) return std::nullopt;
    
    const auto& center = *center_opt;
    const auto& prev_pos = *prev_opt;
    const auto& next_pos = *next_opt;
    
    Vector u(prev_pos[0] - center[0], prev_pos[1] - center[1], prev_pos[2] - center[2]);
    Vector v(next_pos[0] - center[0], next_pos[1] - center[1], next_pos[2] - center[2]);
    
    double u_len = u.magnitude();
    double v_len = v.magnitude();
    
    if (u_len < Tolerance::ZERO_TOLERANCE || v_len < Tolerance::ZERO_TOLERANCE) {
        return 0.0;
    }
    
    double cos_angle = u.dot(v) / (u_len * v_len);
    cos_angle = std::clamp(cos_angle, -1.0, 1.0);
    return std::acos(cos_angle);
}

std::map<size_t, Vector> Mesh::face_normals() const {
    std::map<size_t, Vector> normals;
    for (const auto& [face_key, _] : face) {
        auto normal_opt = face_normal(face_key);
        if (normal_opt) {
            normals[face_key] = *normal_opt;
        }
    }
    return normals;
}

std::map<size_t, Vector> Mesh::vertex_normals() const {
    return vertex_normals_weighted(NormalWeighting::Area);
}

std::map<size_t, Vector> Mesh::vertex_normals_weighted(NormalWeighting weighting) const {
    std::map<size_t, Vector> normals;
    for (const auto& [vertex_key, _] : vertex) {
        auto normal_opt = vertex_normal_weighted(vertex_key, weighting);
        if (normal_opt) {
            normals[vertex_key] = *normal_opt;
        }
    }
    return normals;
}

Mesh Mesh::from_polylines(const std::vector<std::vector<Point>>& polygons, std::optional<double> precision) {
    Mesh mesh;
    
    std::map<std::tuple<int64_t, int64_t, int64_t>, size_t> map_eps;
    std::map<std::tuple<uint64_t, uint64_t, uint64_t>, size_t> map_exact;
    
    auto get_vkey = [&](const Point& p) -> size_t {
        if (precision.has_value()) {
            double eps = *precision;
            int64_t kx = static_cast<int64_t>(std::round(p[0] / eps));
            int64_t ky = static_cast<int64_t>(std::round(p[1] / eps));
            int64_t kz = static_cast<int64_t>(std::round(p[2] / eps));
            auto key = std::make_tuple(kx, ky, kz);
            
            auto it = map_eps.find(key);
            if (it != map_eps.end()) {
                return it->second;
            }
            size_t vk = mesh.add_vertex(p);
            map_eps[key] = vk;
            return vk;
        } else {
            union { double f; uint64_t i; } ux, uy, uz;
            ux.f = p[0]; uy.f = p[1]; uz.f = p[2];
            auto key = std::make_tuple(ux.i, uy.i, uz.i);
            
            auto it = map_exact.find(key);
            if (it != map_exact.end()) {
                return it->second;
            }
            size_t vk = mesh.add_vertex(p);
            map_exact[key] = vk;
            return vk;
        }
    };
    
    for (const auto& poly : polygons) {
        if (poly.size() < 3) continue;
        
        std::vector<size_t> vkeys;
        vkeys.reserve(poly.size());
        for (const auto& p : poly) {
            vkeys.push_back(get_vkey(p));
        }
        mesh.add_face(vkeys);
    }
    return mesh;
}

Mesh Mesh::from_vertices_and_faces(const std::vector<Point>& vertices,
                                    const std::vector<std::vector<size_t>>& faces) {
    Mesh mesh;
    for (const auto& pt : vertices)
        mesh.add_vertex(pt);
    for (const auto& f : faces)
        mesh.add_face(f);
    return mesh;
}

Mesh Mesh::from_lines(const std::vector<Line>& lines, bool delete_boundary_face, std::optional<double> precision) {
    if (lines.empty()) return Mesh();

    // Collect all endpoints
    std::vector<Point> all_pts;
    all_pts.reserve(lines.size() * 2);
    for (const auto& ln : lines) {
        all_pts.push_back(ln.start());
        all_pts.push_back(ln.end());
    }

    // Compute precision from bbox if not given
    double eps = precision.value_or(0.0);
    if (eps <= 0.0) {
        double minx = all_pts[0][0], miny = all_pts[0][1], minz = all_pts[0][2];
        double maxx = minx, maxy = miny, maxz = minz;
        for (const auto& p : all_pts) {
            if (p[0] < minx) minx = p[0]; if (p[0] > maxx) maxx = p[0];
            if (p[1] < miny) miny = p[1]; if (p[1] > maxy) maxy = p[1];
            if (p[2] < minz) minz = p[2]; if (p[2] > maxz) maxz = p[2];
        }
        double diag = std::sqrt((maxx-minx)*(maxx-minx) + (maxy-miny)*(maxy-miny) + (maxz-minz)*(maxz-minz));
        eps = diag * 1e-6;
        if (eps < 1e-12) eps = 1e-12;
    }

    // Merge vertices using grid quantization
    std::map<std::tuple<int64_t, int64_t, int64_t>, size_t> vmap;
    std::vector<Point> verts;
    auto get_vid = [&](const Point& p) -> size_t {
        int64_t kx = static_cast<int64_t>(std::round(p[0] / eps));
        int64_t ky = static_cast<int64_t>(std::round(p[1] / eps));
        int64_t kz = static_cast<int64_t>(std::round(p[2] / eps));
        auto key = std::make_tuple(kx, ky, kz);
        auto it = vmap.find(key);
        if (it != vmap.end()) return it->second;
        size_t id = verts.size();
        verts.push_back(p);
        vmap[key] = id;
        return id;
    };

    // Build adjacency from line segments
    size_t nv = 0;
    std::map<size_t, std::vector<size_t>> adj;
    for (const auto& ln : lines) {
        size_t a = get_vid(ln.start());
        size_t b = get_vid(ln.end());
        if (a == b) continue;
        adj[a].push_back(b);
        adj[b].push_back(a);
    }
    nv = verts.size();

    // Sort neighbors CCW by angle at each vertex
    for (auto& [v, nbrs] : adj) {
        // Remove duplicates
        std::sort(nbrs.begin(), nbrs.end());
        nbrs.erase(std::unique(nbrs.begin(), nbrs.end()), nbrs.end());
        // Sort by atan2 angle
        double vx = verts[v][0], vy = verts[v][1];
        std::sort(nbrs.begin(), nbrs.end(), [&](size_t a, size_t b) {
            double aa = std::atan2(verts[a][1] - vy, verts[a][0] - vx);
            double ba = std::atan2(verts[b][1] - vy, verts[b][0] - vx);
            return aa < ba;
        });
    }

    // Half-edge traversal to find face cycles
    // For directed edge u->v, next edge starts at v, going to the neighbor
    // just CW after u in v's CCW-sorted list (predecessor of u)
    std::set<std::pair<size_t, size_t>> visited;
    std::vector<std::vector<size_t>> face_cycles;

    for (auto& [u, nbrs] : adj) {
        for (size_t v : nbrs) {
            if (visited.count({u, v})) continue;

            std::vector<size_t> cycle;
            size_t cu = u, cv = v;
            bool valid = true;
            while (true) {
                if (visited.count({cu, cv})) break;
                visited.insert({cu, cv});
                cycle.push_back(cu);

                // Find next: at vertex cv, find cu in cv's neighbor list,
                // then take the predecessor (CW neighbor = previous in CCW order)
                auto& cv_nbrs = adj[cv];
                auto it = std::find(cv_nbrs.begin(), cv_nbrs.end(), cu);
                if (it == cv_nbrs.end()) { valid = false; break; }
                size_t idx = static_cast<size_t>(it - cv_nbrs.begin());
                size_t prev_idx = (idx == 0) ? cv_nbrs.size() - 1 : idx - 1;
                size_t nxt = cv_nbrs[prev_idx];

                cu = cv;
                cv = nxt;

                if (cycle.size() > nv * 2) { valid = false; break; }
            }

            if (valid && cycle.size() >= 3) {
                face_cycles.push_back(cycle);
            }
        }
    }

    // Build mesh from cycles
    Mesh mesh;
    for (const auto& pt : verts) mesh.add_vertex(pt);

    // Optionally delete the largest face (boundary)
    if (delete_boundary_face && !face_cycles.empty()) {
        size_t max_idx = 0;
        size_t max_size = face_cycles[0].size();
        for (size_t i = 1; i < face_cycles.size(); ++i) {
            if (face_cycles[i].size() > max_size) {
                max_size = face_cycles[i].size();
                max_idx = i;
            }
        }
        face_cycles.erase(face_cycles.begin() + max_idx);
    }

    for (const auto& cycle : face_cycles) {
        std::vector<Point> pts;
        pts.reserve(cycle.size());
        for (size_t vid : cycle) pts.push_back(verts[vid]);
        auto tris = Triangulation2D::triangulate(Polyline(pts));
        for (const auto& t : tris)
            mesh.add_face({cycle[static_cast<size_t>(t.v0)], cycle[static_cast<size_t>(t.v1)], cycle[static_cast<size_t>(t.v2)]});
    }

    return mesh;
}


Mesh Mesh::from_polygon_with_holes(const std::vector<std::vector<Point>>& polylines, bool sort_by_bbox) {
    if (polylines.empty()) return Mesh();

    // Find border polyline index
    int border_idx = 0;
    if (sort_by_bbox && polylines.size() > 1) {
        double max_diag = 0.0;
        for (size_t i = 0; i < polylines.size(); ++i) {
            if (polylines[i].size() < 3) continue;
            double minx = polylines[i][0][0], miny = polylines[i][0][1], minz = polylines[i][0][2];
            double maxx = minx, maxy = miny, maxz = minz;
            for (const auto& p : polylines[i]) {
                if (p[0] < minx) minx = p[0]; if (p[0] > maxx) maxx = p[0];
                if (p[1] < miny) miny = p[1]; if (p[1] > maxy) maxy = p[1];
                if (p[2] < minz) minz = p[2]; if (p[2] > maxz) maxz = p[2];
            }
            double dx = maxx - minx, dy = maxy - miny, dz = maxz - minz;
            double diag = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (diag > max_diag) { max_diag = diag; border_idx = static_cast<int>(i); }
        }
    }

    // Strip closing duplicates
    auto strip_close = [](const std::vector<Point>& pts) -> std::vector<Point> {
        if (pts.size() > 1) {
            const auto& f = pts.front();
            const auto& b = pts.back();
            if (std::abs(f[0]-b[0]) < 1e-12 && std::abs(f[1]-b[1]) < 1e-12 && std::abs(f[2]-b[2]) < 1e-12) {
                return std::vector<Point>(pts.begin(), pts.end() - 1);
            }
        }
        return pts;
    };

    auto border = strip_close(polylines[border_idx]);
    if (border.size() < 3) return Mesh();

    // Collect holes
    std::vector<std::vector<Point>> hole_pts_3d;
    for (size_t i = 0; i < polylines.size(); ++i) {
        if (static_cast<int>(i) == border_idx) continue;
        auto hole = strip_close(polylines[i]);
        if (hole.size() < 3) continue;
        hole_pts_3d.push_back(hole);
    }

    // Compute average plane from all points (border + holes) for stable normal
    std::vector<Point> all_pts_for_plane = border;
    for (const auto& h : hole_pts_3d)
        for (const auto& p : h) all_pts_for_plane.push_back(p);
    Polyline all_pl(all_pts_for_plane);
    Point origin;
    Vector xaxis, yaxis, zaxis;
    all_pl.get_average_plane(origin, xaxis, yaxis, zaxis);

    auto project_2d = [&](const Point& p) -> std::pair<double,double> {
        double dx = p[0] - origin[0], dy = p[1] - origin[1], dz = p[2] - origin[2];
        return { dx * xaxis[0] + dy * xaxis[1] + dz * xaxis[2],
                 dx * yaxis[0] + dy * yaxis[1] + dz * yaxis[2] };
    };

    // Project border to 2D
    std::vector<std::pair<double,double>> boundary_2d;
    for (const auto& p : border) boundary_2d.push_back(project_2d(p));

    // Enforce boundary = CCW
    auto signed_area = [](const std::vector<std::pair<double,double>>& pts) -> double {
        double area = 0.0;
        size_t n = pts.size();
        for (size_t i = 0; i < n; ++i) {
            size_t j = (i + 1) % n;
            area += pts[i].first * pts[j].second - pts[j].first * pts[i].second;
        }
        return area * 0.5;
    };
    if (signed_area(boundary_2d) < 0.0) {
        std::reverse(border.begin(), border.end());
        std::reverse(boundary_2d.begin(), boundary_2d.end());
    }

    // Project holes to 2D, enforce CW
    std::vector<std::vector<std::pair<double,double>>> holes_2d;
    for (auto& hole : hole_pts_3d) {
        std::vector<std::pair<double,double>> h2d;
        for (const auto& p : hole) h2d.push_back(project_2d(p));
        if (signed_area(h2d) > 0.0) {
            std::reverse(hole.begin(), hole.end());
            std::reverse(h2d.begin(), h2d.end());
        }
        holes_2d.push_back(std::move(h2d));
    }

    auto tris = cdt_triangulate(boundary_2d, holes_2d);

    std::vector<Point> all_pts = border;
    for (const auto& h : hole_pts_3d)
        for (const auto& p : h) all_pts.push_back(p);

    Mesh mesh;
    std::vector<size_t> vkeys;
    for (const auto& p : all_pts) vkeys.push_back(mesh.add_vertex(p));

    for (const auto& f : tris) {
        if (vkeys[f[0]] == vkeys[f[1]] || vkeys[f[1]] == vkeys[f[2]] || vkeys[f[2]] == vkeys[f[0]]) continue;
        mesh.add_face({vkeys[f[0]], vkeys[f[1]], vkeys[f[2]]});
    }
    return mesh;
}

Mesh Mesh::loft(const std::vector<Polyline>& polylines0, const std::vector<Polyline>& polylines1, bool cap) {
    if (polylines0.empty() || polylines1.empty()) return Mesh();
    if (polylines0.size() != polylines1.size()) return Mesh();

    // Find border polyline (largest bbox diagonal) among polylines0
    int border_idx = 0;
    double max_diag = 0.0;
    for (size_t i = 0; i < polylines0.size(); ++i) {
        auto pts = polylines0[i].get_points();
        if (pts.empty()) continue;
        double minx = pts[0][0], miny = pts[0][1], minz = pts[0][2];
        double maxx = minx, maxy = miny, maxz = minz;
        for (const auto& p : pts) {
            if (p[0] < minx) minx = p[0]; if (p[0] > maxx) maxx = p[0];
            if (p[1] < miny) miny = p[1]; if (p[1] > maxy) maxy = p[1];
            if (p[2] < minz) minz = p[2]; if (p[2] > maxz) maxz = p[2];
        }
        double dx = maxx - minx, dy = maxy - miny, dz = maxz - minz;
        double diag = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (diag > max_diag) { max_diag = diag; border_idx = static_cast<int>(i); }
    }

    auto get_open_points = [](const Polyline& pl) -> std::vector<Point> {
        auto pts = pl.get_points();
        if (pts.size() > 1) {
            const auto& f = pts.front();
            const auto& b = pts.back();
            if (std::abs(f[0]-b[0]) < 1e-12 && std::abs(f[1]-b[1]) < 1e-12 && std::abs(f[2]-b[2]) < 1e-12)
                pts.pop_back();
        }
        return pts;
    };

    // Compute average plane from border polyline for 2D projection
    Point origin;
    Vector xaxis, yaxis, zaxis;
    polylines0[border_idx].get_average_plane(origin, xaxis, yaxis, zaxis);

    auto project_2d = [&](const Point& p) -> std::pair<double, double> {
        double dx = p[0] - origin[0];
        double dy = p[1] - origin[1];
        double dz = p[2] - origin[2];
        double u = dx * xaxis[0] + dy * xaxis[1] + dz * xaxis[2];
        double v = dx * yaxis[0] + dy * yaxis[1] + dz * yaxis[2];
        return {u, v};
    };

    // 2D signed area: positive = CCW, negative = CW
    auto signed_area_2d = [&](const std::vector<Point>& pts) -> double {
        double area = 0.0;
        size_t n = pts.size();
        for (size_t i = 0; i < n; ++i) {
            size_t j = (i + 1) % n;
            auto [xi, yi] = project_2d(pts[i]);
            auto [xj, yj] = project_2d(pts[j]);
            area += xi * yj - xj * yi;
        }
        return area * 0.5;
    };

    // Build ordered list: border first, then holes
    std::vector<int> order;
    order.push_back(border_idx);
    for (size_t i = 0; i < polylines0.size(); ++i)
        if (static_cast<int>(i) != border_idx) order.push_back(static_cast<int>(i));

    // Collect open points, enforce winding: border=CCW, holes=CW
    struct PolyInfo { size_t offset; size_t count; };
    std::vector<PolyInfo> poly_infos;
    std::vector<Point> all_bot_pts;
    std::vector<Point> all_top_pts;

    for (size_t oi = 0; oi < order.size(); ++oi) {
        int idx = order[oi];
        auto bot = get_open_points(polylines0[idx]);
        auto top = get_open_points(polylines1[idx]);
        size_t n = std::min(bot.size(), top.size());
        bot.resize(n);
        top.resize(n);

        bool is_border = (oi == 0);
        double area = signed_area_2d(bot);
        if ((is_border && area < 0) || (!is_border && area > 0)) {
            std::reverse(bot.begin(), bot.end());
            std::reverse(top.begin(), top.end());
        }

        poly_infos.push_back({all_bot_pts.size(), n});
        for (size_t i = 0; i < n; ++i) all_bot_pts.push_back(bot[i]);
        for (size_t i = 0; i < n; ++i) all_top_pts.push_back(top[i]);
    }

    size_t total_pts = all_bot_pts.size();

    // Build mesh
    Mesh mesh;

    std::vector<size_t> bot_vkeys;
    for (size_t i = 0; i < total_pts; ++i)
        bot_vkeys.push_back(mesh.add_vertex(all_bot_pts[i]));

    std::vector<size_t> top_vkeys;
    for (size_t i = 0; i < total_pts; ++i)
        top_vkeys.push_back(mesh.add_vertex(all_top_pts[i]));

    if (cap) {
        std::vector<std::pair<double,double>> border_2d_pairs;
        for (size_t i = poly_infos[0].offset; i < poly_infos[0].offset + poly_infos[0].count; ++i)
            border_2d_pairs.push_back(project_2d(all_bot_pts[i]));

        std::vector<std::vector<std::pair<double,double>>> holes_2d_pairs;
        for (size_t h = 1; h < poly_infos.size(); ++h) {
            std::vector<std::pair<double,double>> hole;
            for (size_t i = poly_infos[h].offset; i < poly_infos[h].offset + poly_infos[h].count; ++i)
                hole.push_back(project_2d(all_bot_pts[i]));
            holes_2d_pairs.push_back(std::move(hole));
        }

        auto cap_tris = cdt_triangulate(border_2d_pairs, holes_2d_pairs);

        for (const auto& f : cap_tris)
            mesh.add_face({bot_vkeys[f[0]], bot_vkeys[f[2]], bot_vkeys[f[1]]});
        for (const auto& f : cap_tris)
            mesh.add_face({top_vkeys[f[0]], top_vkeys[f[1]], top_vkeys[f[2]]});
    }

    // Side quads from original polyline edges
    for (size_t p = 0; p < poly_infos.size(); ++p) {
        size_t off = poly_infos[p].offset;
        size_t n = poly_infos[p].count;
        bool is_border = (p == 0);

        for (size_t i = 0; i < n; ++i) {
            size_t j = (i + 1) % n;
            size_t bi = off + i, bj = off + j;

            if (is_border)
                mesh.add_face({bot_vkeys[bi], bot_vkeys[bj], top_vkeys[bj], top_vkeys[bi]});
            else
                mesh.add_face({bot_vkeys[bi], top_vkeys[bi], top_vkeys[bj], bot_vkeys[bj]});
        }
    }

    return mesh;
}

std::map<size_t, size_t> Mesh::vertex_index() const {
    // Collect and sort keys to ensure consistent ordering
    std::vector<size_t> keys;
    keys.reserve(vertex.size());
    for (const auto& [key, _] : vertex) {
        keys.push_back(key);
    }
    std::sort(keys.begin(), keys.end());
    
    // Create index mapping
    std::map<size_t, size_t> index_map;
    for (size_t index = 0; index < keys.size(); ++index) {
        index_map[keys[index]] = index;
    }
    return index_map;
}

std::pair<std::vector<Point>, std::vector<std::vector<size_t>>> Mesh::to_vertices_and_faces() const {
    auto vertex_idx = vertex_index();
    std::vector<Point> vertices(vertex.size());
    
    for (const auto& [key, vdata] : vertex) {
        size_t idx = vertex_idx[key];
        vertices[idx] = vdata.position();
    }
    
    // Sort face keys to ensure consistent ordering
    std::vector<size_t> face_keys;
    face_keys.reserve(face.size());
    for (const auto& [key, _] : face) {
        face_keys.push_back(key);
    }
    std::sort(face_keys.begin(), face_keys.end());
    
    std::vector<std::vector<size_t>> faces;
    for (size_t face_key : face_keys) {
        const auto& face_vertices = face.at(face_key);
        std::vector<size_t> remapped;
        remapped.reserve(face_vertices.size());
        for (size_t v : face_vertices) {
            remapped.push_back(vertex_idx.at(v));
        }
        faces.push_back(remapped);
    }
    return {vertices, faces};
}

void Mesh::transform() {
  for (auto& [idx, vdata] : vertex) {
    Point pt(vdata.x, vdata.y, vdata.z);
    xform.transform_point(pt);
    vdata.x = pt[0];
    vdata.y = pt[1];
    vdata.z = pt[2];
  }
  xform = Xform::identity();
  triangle_bvh_built = false;
  triangle_bvh.reset();
  triangle_boxes_cache.clear();
  triangle_aabbs_cache.clear();
  triangle_indices_cache.clear();
  triangle_face_subidx_cache.clear();
  vertices_cache.clear();
}

Mesh Mesh::transformed() const {
  Mesh result = *this;
  result.transform();
  return result;
}

nlohmann::ordered_json Mesh::jsondump() const {
    nlohmann::ordered_json data;
    
    // Alphabetical order to match Rust's serde_json output
    data["default_edge_attributes"] = default_edge_attributes;
    data["default_face_attributes"] = default_face_attributes;
    data["default_vertex_attributes"] = default_vertex_attributes;
    
    // Edge attributes
    nlohmann::ordered_json edgedata_json;
    for (const auto& [edge, attrs] : edgedata) {
        std::string edge_key = std::to_string(edge.first) + "," + std::to_string(edge.second);
        edgedata_json[edge_key] = attrs;
    }
    data["edgedata"] = edgedata_json;
    
    // Face data
    nlohmann::ordered_json face_data;
    for (const auto& [key, vertices] : face) {
        face_data[std::to_string(key)] = vertices;
    }
    data["face"] = face_data;

    // Face colors (RGBA)
    nlohmann::ordered_json facecolors_arr = nlohmann::ordered_json::array();
    for (const auto& c : facecolors) {
        facecolors_arr.push_back(c.r); facecolors_arr.push_back(c.g);
        facecolors_arr.push_back(c.b); facecolors_arr.push_back(c.a);
    }
    data["facecolors"] = facecolors_arr;
    
    // Face attributes
    nlohmann::ordered_json facedata_json;
    for (const auto& [key, attrs] : facedata) {
        facedata_json[std::to_string(key)] = attrs;
    }
    data["facedata"] = facedata_json;
    
    data["guid"] = guid;
    
    // Halfedge connectivity
    nlohmann::ordered_json halfedge_data;
    for (const auto& [u, neighbors] : halfedge) {
        nlohmann::ordered_json neighbor_data;
        for (const auto& [v, face_opt] : neighbors) {
            neighbor_data[std::to_string(v)] = face_opt.has_value() ? nlohmann::json(face_opt.value()) : nlohmann::json(nullptr);
        }
        halfedge_data[std::to_string(u)] = neighbor_data;
    }
    data["halfedge"] = halfedge_data;

    // Line colors (RGBA)
    nlohmann::ordered_json linecolors_arr = nlohmann::ordered_json::array();
    for (const auto& c : linecolors) {
        linecolors_arr.push_back(c.r); linecolors_arr.push_back(c.g);
        linecolors_arr.push_back(c.b); linecolors_arr.push_back(c.a);
    }
    data["linecolors"] = linecolors_arr;

    data["max_face"] = max_face;
    data["max_vertex"] = max_vertex;
    data["name"] = name;

    // Point colors (RGBA)
    nlohmann::ordered_json pointcolors_arr = nlohmann::ordered_json::array();
    for (const auto& c : pointcolors) {
        pointcolors_arr.push_back(c.r); pointcolors_arr.push_back(c.g);
        pointcolors_arr.push_back(c.b); pointcolors_arr.push_back(c.a);
    }
    data["pointcolors"] = pointcolors_arr;

    data["type"] = "Mesh";
    
    // Vertex data
    nlohmann::ordered_json vertex_data;
    for (const auto& [key, vdata] : vertex) {
        nlohmann::ordered_json v;
        v["attributes"] = vdata.attributes;
        v["x"] = vdata.x;
        v["y"] = vdata.y;
        v["z"] = vdata.z;
        vertex_data[std::to_string(key)] = v;
    }
    data["vertex"] = vertex_data;

    data["widths"] = widths;

    return data;
}

Mesh Mesh::jsonload(const nlohmann::json& data) {
    Mesh mesh;
    
    if (data.contains("guid")) mesh.guid = data["guid"];
    if (data.contains("name")) mesh.name = data["name"];
    
    // Load halfedge connectivity
    if (data.contains("halfedge")) {
        for (const auto& [u_str, neighbors] : data["halfedge"].items()) {
            size_t u = std::stoull(u_str);
            mesh.halfedge[u] = {};
            for (const auto& [v_str, face_val] : neighbors.items()) {
                size_t v = std::stoull(v_str);
                if (face_val.is_null()) {
                    mesh.halfedge[u][v] = std::nullopt;
                } else {
                    mesh.halfedge[u][v] = face_val.get<size_t>();
                }
            }
        }
    }
    
    // Load vertex data
    if (data.contains("vertex")) {
        for (const auto& [key_str, vdata] : data["vertex"].items()) {
            size_t key = std::stoull(key_str);
            VertexData vertex_data;
            vertex_data.x = vdata["x"];
            vertex_data.y = vdata["y"];
            vertex_data.z = vdata["z"];
            if (vdata.contains("attributes")) {
                vertex_data.attributes = vdata["attributes"].get<std::map<std::string, double>>();
            }
            mesh.vertex[key] = vertex_data;
            if (!data.contains("halfedge")) {
                mesh.halfedge[key] = {};
            }
            if (key >= mesh.max_vertex) mesh.max_vertex = key + 1;
        }
    }
    
    // Load face data
    if (data.contains("face")) {
        for (const auto& [key_str, vertices] : data["face"].items()) {
            size_t key = std::stoull(key_str);
            mesh.face[key] = vertices.get<std::vector<size_t>>();
            if (key >= mesh.max_face) mesh.max_face = key + 1;
        }
    }
    
    // Load face attributes
    if (data.contains("facedata")) {
        for (const auto& [key_str, attrs] : data["facedata"].items()) {
            size_t key = std::stoull(key_str);
            mesh.facedata[key] = attrs.get<std::map<std::string, double>>();
        }
    }
    
    // Load edge attributes
    if (data.contains("edgedata")) {
        for (const auto& [edge_str, attrs] : data["edgedata"].items()) {
            size_t comma_pos = edge_str.find(',');
            size_t u = std::stoull(edge_str.substr(0, comma_pos));
            size_t v = std::stoull(edge_str.substr(comma_pos + 1));
            mesh.edgedata[{u, v}] = attrs.get<std::map<std::string, double>>();
        }
    }
    
    if (data.contains("default_vertex_attributes")) {
        mesh.default_vertex_attributes = data["default_vertex_attributes"];
    }
    if (data.contains("default_face_attributes")) {
        mesh.default_face_attributes = data["default_face_attributes"];
    }
    if (data.contains("default_edge_attributes")) {
        mesh.default_edge_attributes = data["default_edge_attributes"];
    }
    if (data.contains("max_vertex")) {
        mesh.max_vertex = data["max_vertex"];
    }
    if (data.contains("max_face")) {
        mesh.max_face = data["max_face"];
    }

    // Load colors from flat RGB arrays
    if (data.contains("pointcolors") && data["pointcolors"].is_array()) {
        const auto& arr = data["pointcolors"];
        mesh.pointcolors.clear();
        for (size_t i = 0; i + 3 < arr.size(); i += 4) {
            mesh.pointcolors.push_back(Color(arr[i].get<int>(), arr[i+1].get<int>(),
                arr[i+2].get<int>(), arr[i+3].get<int>()));
        }
    }

    if (data.contains("facecolors") && data["facecolors"].is_array()) {
        const auto& arr = data["facecolors"];
        mesh.facecolors.clear();
        for (size_t i = 0; i + 3 < arr.size(); i += 4) {
            mesh.facecolors.push_back(Color(arr[i].get<int>(), arr[i+1].get<int>(),
                arr[i+2].get<int>(), arr[i+3].get<int>()));
        }
    }

    if (data.contains("linecolors") && data["linecolors"].is_array()) {
        const auto& arr = data["linecolors"];
        mesh.linecolors.clear();
        for (size_t i = 0; i + 3 < arr.size(); i += 4) {
            mesh.linecolors.push_back(Color(arr[i].get<int>(), arr[i+1].get<int>(),
                arr[i+2].get<int>(), arr[i+3].get<int>()));
        }
    }

    if (data.contains("widths") && data["widths"].is_array()) {
        mesh.widths = data["widths"].get<std::vector<double>>();
    }

    return mesh;
}

void Mesh::build_triangle_bvh(bool force) const {
    if (triangle_bvh_built && !force) return;

    triangle_boxes_cache.clear();
    triangle_aabbs_cache.clear();
    triangle_indices_cache.clear();
    triangle_face_subidx_cache.clear();
    vertices_cache.clear();

    auto vf = to_vertices_and_faces();
    vertices_cache = vf.first;
    const std::vector<std::vector<size_t>>& faces_vec = vf.second;

    size_t tri_count = 0;
    for (const auto& f : faces_vec) if (f.size() >= 3) tri_count += (f.size() - 2);
    triangle_aabbs_cache.resize(tri_count);
    triangle_indices_cache.resize(tri_count);
    triangle_face_subidx_cache.resize(tri_count);

    struct TriTask { uint32_t i0, i1, i2; size_t face_idx; size_t sub_idx; size_t out_idx; };
    std::vector<TriTask> tasks;
    tasks.reserve(tri_count);

    size_t out = 0;
    for (size_t fi = 0; fi < faces_vec.size(); ++fi) {
        const auto& fv = faces_vec[fi];
        if (fv.size() < 3) continue;
        for (size_t j = 1; j + 1 < fv.size(); ++j) {
            tasks.push_back(TriTask{static_cast<uint32_t>(fv[0]), static_cast<uint32_t>(fv[j]), static_cast<uint32_t>(fv[j+1]), fi, j, out});
            out++;
        }
    }

    auto worker = [&](size_t begin, size_t end){
        for (size_t k = begin; k < end; ++k) {
            const auto& t = tasks[k];
            const Point& p0 = vertices_cache[t.i0];
            const Point& p1 = vertices_cache[t.i1];
            const Point& p2 = vertices_cache[t.i2];

            double min_x = std::min(std::min(p0[0], p1[0]), p2[0]) - 0.001;
            double min_y = std::min(std::min(p0[1], p1[1]), p2[1]) - 0.001;
            double min_z = std::min(std::min(p0[2], p1[2]), p2[2]) - 0.001;
            double max_x = std::max(std::max(p0[0], p1[0]), p2[0]) + 0.001;
            double max_y = std::max(std::max(p0[1], p1[1]), p2[1]) + 0.001;
            double max_z = std::max(std::max(p0[2], p1[2]), p2[2]) + 0.001;

            double cx = (min_x + max_x) * 0.5;
            double cy = (min_y + max_y) * 0.5;
            double cz = (min_z + max_z) * 0.5;
            double hx = (max_x - min_x) * 0.5;
            double hy = (max_y - min_y) * 0.5;
            double hz = (max_z - min_z) * 0.5;

            BvhAABB bb{cx, cy, cz, hx, hy, hz};
            triangle_aabbs_cache[t.out_idx] = bb;
            triangle_indices_cache[t.out_idx] = TriangleIndex{t.i0, t.i1, t.i2};
            triangle_face_subidx_cache[t.out_idx] = {t.face_idx, t.sub_idx};
        }
    };

    unsigned int threads = std::max(1u, std::thread::hardware_concurrency());
    size_t total = tasks.size();
    size_t block = (total + threads - 1) / threads;
    std::vector<std::thread> pool;
    pool.reserve(threads);
    size_t beginIdx = 0;
    for (unsigned int ti = 0; ti < threads && beginIdx < total; ++ti) {
        size_t endIdx = std::min(beginIdx + block, total);
        pool.emplace_back(worker, beginIdx, endIdx);
        beginIdx = endIdx;
    }
    for (auto& th : pool) th.join();

    // Compute world size from object bounds (triangle AABBs)
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    for (const auto& bb : triangle_aabbs_cache) {
        double bx0 = bb.cx - bb.hx; double bx1 = bb.cx + bb.hx;
        double by0 = bb.cy - bb.hy; double by1 = bb.cy + bb.hy;
        double bz0 = bb.cz - bb.hz; double bz1 = bb.cz + bb.hz;
        if (bx0 < min_x) min_x = bx0;
        if (bx1 > max_x) max_x = bx1;
        if (by0 < min_y) min_y = by0;
        if (by1 > max_y) max_y = by1;
        if (bz0 < min_z) min_z = bz0;
        if (bz1 > max_z) max_z = bz1;
    }
    double extent_x = std::max(std::fabs(min_x), std::fabs(max_x));
    double extent_y = std::max(std::fabs(min_y), std::fabs(max_y));
    double extent_z = std::max(std::fabs(min_z), std::fabs(max_z));
    double max_extent = std::max({extent_x, extent_y, extent_z});
    double world_size = std::max(2.2 * max_extent, 10.0);

    triangle_bvh = std::make_shared<BVH>();
    triangle_bvh->build_from_aabbs(triangle_aabbs_cache.data(), triangle_aabbs_cache.size(), world_size);
    triangle_bvh_built = true;
}

bool Mesh::triangle_bvh_ray_cast(const Point& origin, const Vector& direction, std::vector<int>& candidate_ids, bool find_all) const {
    build_triangle_bvh(false);
    if (!triangle_bvh) return false;
    return triangle_bvh->ray_cast(origin, direction, candidate_ids, find_all);
}

bool Mesh::get_triangle_by_id(int tri_id, size_t& face_idx, size_t& sub_idx, Point& v0, Point& v1, Point& v2) const {
    if (tri_id < 0) return false;
    size_t id = static_cast<size_t>(tri_id);
    if (id >= triangle_indices_cache.size() || id >= triangle_face_subidx_cache.size()) return false;
    const auto& tri = triangle_indices_cache[id];
    const auto& fs = triangle_face_subidx_cache[id];
    face_idx = fs.first;
    sub_idx = fs.second;
    if (tri.i0 >= vertices_cache.size() || tri.i1 >= vertices_cache.size() || tri.i2 >= vertices_cache.size()) return false;
    v0 = vertices_cache[tri.i0];
    v1 = vertices_cache[tri.i1];
    v2 = vertices_cache[tri.i2];
    return true;
}

void Mesh::clear_triangle_bvh() const {
    triangle_bvh_built = false;
    triangle_bvh.reset();
    triangle_aabb_tree.reset();
    triangle_boxes_cache.clear();
    triangle_aabbs_cache.clear();
    triangle_indices_cache.clear();
    triangle_face_subidx_cache.clear();
    vertices_cache.clear();
}

void Mesh::build_triangle_aabb_tree(bool force) const {
    build_triangle_bvh(false);
    if (triangle_aabb_tree && !force) return;
    triangle_aabb_tree = std::make_shared<AABBTree>();
    triangle_aabb_tree->build(triangle_aabbs_cache.data(), triangle_aabbs_cache.size());
}

std::string Mesh::json_dumps() const {
    return jsondump().dump();
}

Mesh Mesh::json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

///////////////////////////////////////////////////////////////////////////////////////////
// Protobuf
///////////////////////////////////////////////////////////////////////////////////////////


std::string Mesh::pb_dumps() const {
    session_proto::Mesh proto;
    proto.set_guid(this->guid);
    proto.set_name(this->name);

    // Vertices
    for (const auto& [vkey, vdata] : vertex) {
        auto& vertex_proto = (*proto.mutable_vertices())[vkey];
        vertex_proto.set_x(vdata.x);
        vertex_proto.set_y(vdata.y);
        vertex_proto.set_z(vdata.z);
        for (const auto& [k, v] : vdata.attributes) {
            (*vertex_proto.mutable_attributes())[k] = v;
        }
    }

    // Faces
    for (const auto& [fkey, fverts] : face) {
        auto& face_proto = (*proto.mutable_faces())[fkey];
        for (size_t v : fverts) {
            face_proto.add_vertices(v);
        }
        auto it = facedata.find(fkey);
        if (it != facedata.end()) {
            for (const auto& [k, v] : it->second) {
                (*face_proto.mutable_attributes())[k] = v;
            }
        }
    }

    // Halfedges
    for (const auto& [u, neighbors] : halfedge) {
        auto& hmap = (*proto.mutable_halfedges())[u];
        for (const auto& [v, fkey_opt] : neighbors) {
            (*hmap.mutable_neighbors())[v] = fkey_opt.value_or(UINT64_MAX);
        }
    }

    // Edge data
    for (const auto& [edge, attrs] : edgedata) {
        auto* edge_proto = proto.add_edge_data();
        edge_proto->set_vertex1(edge.first);
        edge_proto->set_vertex2(edge.second);
        for (const auto& [k, v] : attrs) {
            (*edge_proto->mutable_attributes())[k] = v;
        }
    }

    // Default attributes
    for (const auto& [k, v] : default_vertex_attributes) {
        (*proto.mutable_default_vertex_attributes())[k] = v;
    }
    for (const auto& [k, v] : default_face_attributes) {
        (*proto.mutable_default_face_attributes())[k] = v;
    }
    for (const auto& [k, v] : default_edge_attributes) {
        (*proto.mutable_default_edge_attributes())[k] = v;
    }

    // Colors
    for (const auto& c : pointcolors) {
        auto* color_proto = proto.add_pointcolors();
        color_proto->set_guid(c.guid);
        color_proto->set_name(c.name);
        color_proto->set_r(c.r);
        color_proto->set_g(c.g);
        color_proto->set_b(c.b);
        color_proto->set_a(c.a);
    }

    for (const auto& c : facecolors) {
        auto* color_proto = proto.add_facecolors();
        color_proto->set_guid(c.guid);
        color_proto->set_name(c.name);
        color_proto->set_r(c.r);
        color_proto->set_g(c.g);
        color_proto->set_b(c.b);
        color_proto->set_a(c.a);
    }

    for (const auto& c : linecolors) {
        auto* color_proto = proto.add_linecolors();
        color_proto->set_guid(c.guid);
        color_proto->set_name(c.name);
        color_proto->set_r(c.r);
        color_proto->set_g(c.g);
        color_proto->set_b(c.b);
        color_proto->set_a(c.a);
    }

    // Widths
    for (double w : widths) {
        proto.add_widths(w);
    }

    // Xform
    auto* xform_proto = proto.mutable_xform();
    xform_proto->set_guid(xform.guid);
    xform_proto->set_name(xform.name);
    for (int i = 0; i < 16; ++i) {
        xform_proto->add_matrix(xform.m[i]);
    }

    return proto.SerializeAsString();
}

Mesh Mesh::pb_loads(const std::string& data) {
    session_proto::Mesh proto;
    proto.ParseFromString(data);

    Mesh mesh;
    mesh.guid = proto.guid();
    mesh.name = proto.name();

    // Vertices
    for (const auto& [vkey, vdata] : proto.vertices()) {
        VertexData vd;
        vd.x = vdata.x();
        vd.y = vdata.y();
        vd.z = vdata.z();
        for (const auto& [k, v] : vdata.attributes()) {
            vd.attributes[k] = v;
        }
        mesh.vertex[vkey] = vd;
        mesh.halfedge[vkey] = {};
    }

    // Faces
    for (const auto& [fkey, fdata] : proto.faces()) {
        std::vector<size_t> verts;
        for (uint64_t v : fdata.vertices()) {
            verts.push_back(v);
        }
        mesh.face[fkey] = verts;
        if (fdata.attributes_size() > 0) {
            for (const auto& [k, v] : fdata.attributes()) {
                mesh.facedata[fkey][k] = v;
            }
        }
    }

    // Halfedges
    for (const auto& [u, hmap] : proto.halfedges()) {
        std::map<size_t, std::optional<size_t>> neighbors;
        for (const auto& [v, fkey] : hmap.neighbors()) {
            if (fkey == UINT64_MAX) {
                neighbors[v] = std::nullopt;
            } else {
                neighbors[v] = fkey;
            }
        }
        mesh.halfedge[u] = neighbors;
    }

    // Edge data
    for (const auto& edata : proto.edge_data()) {
        auto key = std::make_pair(static_cast<size_t>(edata.vertex1()), static_cast<size_t>(edata.vertex2()));
        for (const auto& [k, v] : edata.attributes()) {
            mesh.edgedata[key][k] = v;
        }
    }

    // Default attributes
    for (const auto& [k, v] : proto.default_vertex_attributes()) {
        mesh.default_vertex_attributes[k] = v;
    }
    for (const auto& [k, v] : proto.default_face_attributes()) {
        mesh.default_face_attributes[k] = v;
    }
    for (const auto& [k, v] : proto.default_edge_attributes()) {
        mesh.default_edge_attributes[k] = v;
    }

    // Colors
    for (const auto& c : proto.pointcolors()) {
        Color color(c.r(), c.g(), c.b(), c.a());
        color.guid = c.guid();
        color.name = c.name();
        mesh.pointcolors.push_back(color);
    }

    for (const auto& c : proto.facecolors()) {
        Color color(c.r(), c.g(), c.b(), c.a());
        color.guid = c.guid();
        color.name = c.name();
        mesh.facecolors.push_back(color);
    }

    for (const auto& c : proto.linecolors()) {
        Color color(c.r(), c.g(), c.b(), c.a());
        color.guid = c.guid();
        color.name = c.name();
        mesh.linecolors.push_back(color);
    }

    // Widths
    for (double w : proto.widths()) {
        mesh.widths.push_back(w);
    }

    // Xform
    const auto& xform_proto = proto.xform();
    mesh.xform.guid = xform_proto.guid();
    mesh.xform.name = xform_proto.name();
    for (int i = 0; i < 16 && i < xform_proto.matrix_size(); ++i) {
        mesh.xform.m[i] = xform_proto.matrix(i);
    }

    // Update max counters
    if (!mesh.vertex.empty()) {
        mesh.max_vertex = mesh.vertex.rbegin()->first + 1;
    }
    if (!mesh.face.empty()) {
        mesh.max_face = mesh.face.rbegin()->first + 1;
    }

    return mesh;
}

void Mesh::pb_dump(const std::string& filename) const {
    std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Mesh Mesh::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return pb_loads(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// String Representation
///////////////////////////////////////////////////////////////////////////////////////////

std::string Mesh::str() const {
    return fmt::format("Mesh(name={}, vertices={}, faces={})",
                       name, number_of_vertices(), number_of_faces());
}

std::string Mesh::repr() const {
    return fmt::format("Mesh(\n  name={},\n  vertices={},\n  faces={},\n  edges={}\n)",
                       name, number_of_vertices(), number_of_faces(), number_of_edges());
}

std::ostream& operator<<(std::ostream& os, const Mesh& mesh) {
    os << mesh.str();
    return os;
}

} // namespace session_cpp
