#include "mesh.h"
#include "triangulation_2d.h"
#include "trimesh_cdt.h"
#include "fmt/core.h"
#include <fstream>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <thread>
#include <atomic>

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
    objectcolor = other.objectcolor;
    color_mode = other.color_mode;
    xform = other.xform;
    max_vertex = other.max_vertex;
    max_face = other.max_face;
    triangulation = other.triangulation;
    face_holes = other.face_holes;
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
        color_mode = other.color_mode;
        xform = other.xform;
        max_vertex = other.max_vertex;
        max_face = other.max_face;
        triangulation = other.triangulation;
        face_holes = other.face_holes;
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

std::vector<size_t> Mesh::vertices() const {
    std::vector<size_t> result;
    result.reserve(vertex.size());
    for (const auto& [k, _] : vertex)
        result.push_back(k);
    return result;
}

std::vector<size_t> Mesh::faces() const {
    std::vector<size_t> result;
    result.reserve(face.size());
    for (const auto& [k, _] : face)
        result.push_back(k);
    return result;
}

std::vector<std::pair<size_t, size_t>> Mesh::edges() const {
    std::set<std::pair<size_t, size_t>> seen;
    std::vector<std::pair<size_t, size_t>> result;
    for (const auto& [u, neighbors] : halfedge) {
        for (const auto& [v, _] : neighbors) {
            auto edge = std::minmax(u, v);
            if (seen.insert(edge).second)
                result.push_back(edge);
        }
    }
    return result;
}

std::vector<std::pair<size_t, size_t>> Mesh::naked_edges(bool boundary) const {
    std::set<std::pair<size_t, size_t>> seen;
    std::vector<std::pair<size_t, size_t>> result;
    for (const auto& [u, neighbors] : halfedge) {
        for (const auto& [v, _] : neighbors) {
            auto edge = std::minmax(u, v);
            if (!seen.insert(edge).second) continue;
            if (is_edge_on_boundary(u, v) == boundary)
                result.push_back(edge);
        }
    }
    return result;
}

std::vector<size_t> Mesh::naked_vertices(bool boundary) const {
    std::vector<size_t> result;
    for (const auto& [vk, _] : vertex)
        if (is_vertex_on_boundary(vk) == boundary)
            result.push_back(vk);
    return result;
}

std::vector<size_t> Mesh::naked_faces(bool boundary) const {
    std::vector<size_t> result;
    for (const auto& [fk, _] : face)
        if (is_face_on_boundary(fk) == boundary)
            result.push_back(fk);
    return result;
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

bool Mesh::is_closed() const {
    for (const auto& [u, nbrs] : halfedge)
        for (const auto& [v, fkey] : nbrs)
            if (!fkey.has_value()) return false;
    return !halfedge.empty();
}

static void parallel_for(size_t n, std::function<void(size_t)> fn) {
    unsigned int hw = std::max(1u, std::thread::hardware_concurrency());
    unsigned int nthreads = static_cast<unsigned int>(std::min((size_t)hw, n));
    std::atomic<size_t> idx{0};
    std::vector<std::thread> threads;
    threads.reserve(nthreads);
    for (unsigned int t = 0; t < nthreads; ++t)
        threads.emplace_back([&] {
            for (size_t i = idx.fetch_add(1); i < n; i = idx.fetch_add(1))
                fn(i);
        });
    for (auto& th : threads) th.join();
}

std::vector<Mesh> Mesh::from_polygon_with_holes_many(
    const std::vector<std::vector<std::vector<Point>>>& inputs,
    bool sort_by_bbox, bool parallel)
{
    std::vector<Mesh> results(inputs.size());
    auto fn = [&](size_t i) { results[i] = from_polygon_with_holes(inputs[i], sort_by_bbox); };
    if (parallel && inputs.size() > 1) parallel_for(inputs.size(), fn);
    else for (size_t i = 0; i < inputs.size(); ++i) fn(i);
    return results;
}

std::vector<Mesh> Mesh::loft_many(
    const std::vector<std::pair<std::vector<Polyline>, std::vector<Polyline>>>& pairs,
    bool cap, bool parallel)
{
    std::vector<Mesh> results(pairs.size());
    auto fn = [&](size_t i) { results[i] = loft(pairs[i].first, pairs[i].second, cap); };
    if (parallel && pairs.size() > 1) parallel_for(pairs.size(), fn);
    else for (size_t i = 0; i < pairs.size(); ++i) fn(i);
    return results;
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
    face_holes.clear();
    max_vertex = 0;
    max_face = 0;
    pointcolors.clear();
    facecolors.clear();
    linecolors.clear();
    widths.clear();
    objectcolor = Color::white();
    color_mode = ColorMode::OBJECTCOLOR;
    triangle_bvh_built = false;
    triangle_bvh.reset();
    triangle_boxes_cache.clear();
    triangle_aabbs_cache.clear();
    triangle_indices_cache.clear();
    triangle_face_subidx_cache.clear();
    vertices_cache.clear();
}

Mesh Mesh::unweld() const {
    Mesh m;
    for (const auto& [fkey, vkeys] : face) {
        std::vector<size_t> new_vkeys;
        new_vkeys.reserve(vkeys.size());
        for (size_t vk : vkeys) {
            const auto& vd = vertex.at(vk);
            new_vkeys.push_back(m.add_vertex(Point(vd.x, vd.y, vd.z)));
        }
        m.add_face(new_vkeys);
    }
    return m;
}

Mesh Mesh::weld(double tolerance) const {
    if (vertex.empty()) return Mesh();

    std::vector<size_t> vkeys;
    std::vector<Point> positions;
    vkeys.reserve(vertex.size());
    positions.reserve(vertex.size());
    for (const auto& [vk, vd] : vertex) {
        vkeys.push_back(vk);
        positions.push_back(vd.position());
    }
    size_t n = vkeys.size();

    std::vector<size_t> parent(n);
    std::iota(parent.begin(), parent.end(), 0);
    std::function<size_t(size_t)> find = [&](size_t x) -> size_t {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    };

    if (tolerance > 0.0) {
        std::vector<Obb> boxes;
        boxes.reserve(n);
        for (const auto& p : positions)
            boxes.push_back(Obb::from_point(p, tolerance));
        double ws = BVH::compute_world_size(boxes);
        BVH bvh = BVH::from_boxes(boxes, ws);
        auto [pairs, ignore1, ignore2] = bvh.check_all_collisions(boxes);
        for (const auto& [i, j] : pairs) {
            if (positions[i].distance(positions[j]) <= tolerance) {
                size_t ri = find(i), rj = find(j);
                if (ri != rj) parent[ri] = rj;
            }
        }
    }

    std::map<size_t, size_t> root_to_rep;
    for (size_t i = 0; i < n; i++) {
        size_t root = find(i);
        auto [it, inserted] = root_to_rep.emplace(root, vkeys[i]);
        if (!inserted && vkeys[i] < it->second) it->second = vkeys[i];
    }
    std::map<size_t, size_t> vkey_to_rep;
    for (size_t i = 0; i < n; i++)
        vkey_to_rep[vkeys[i]] = root_to_rep.at(find(i));

    Mesh m;
    std::set<size_t> added;
    for (size_t i = 0; i < n; i++) {
        size_t rep = vkey_to_rep.at(vkeys[i]);
        if (added.insert(rep).second)
            m.add_vertex(vertex.at(rep).position(), rep);
    }
    for (const auto& [fk, fvkeys] : face) {
        std::vector<size_t> new_vkeys;
        new_vkeys.reserve(fvkeys.size());
        for (size_t vk : fvkeys) new_vkeys.push_back(vkey_to_rep.at(vk));
        m.add_face(new_vkeys, fk);
    }
    return m;
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

    orient_outward();
    return true;
}

bool Mesh::orient_outward() {
    if (face.empty() || !naked_edges(true).empty()) return false;
    double vol = 0.0;
    for (const auto& [fk, verts] : face) {
        size_t n = verts.size();
        auto p0 = *vertex_point(verts[0]);
        for (size_t i = 1; i + 1 < n; ++i) {
            auto p1 = *vertex_point(verts[i]);
            auto p2 = *vertex_point(verts[i + 1]);
            vol += p0[0] * (p1[1] * p2[2] - p1[2] * p2[1])
                 + p0[1] * (p1[2] * p2[0] - p1[0] * p2[2])
                 + p0[2] * (p1[0] * p2[1] - p1[1] * p2[0]);
        }
    }
    if (vol >= 0.0) return false;
    for (auto& [fk, verts] : face)
        std::reverse(verts.begin(), verts.end());
    for (auto& [u, nbrs] : halfedge) nbrs.clear();
    for (const auto& [fk, verts] : face) {
        size_t n = verts.size();
        for (size_t i = 0; i < n; ++i) {
            size_t u = verts[i], v = verts[(i + 1) % n];
            halfedge[u][v] = fk;
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

void Mesh::remove_face(size_t fkey) {
    auto it = face.find(fkey);
    if (it == face.end()) return;
    const auto& verts = it->second;
    size_t n = verts.size();
    for (size_t i = 0; i < n; ++i) {
        size_t u = verts[i];
        size_t v = verts[(i + 1) % n];
        auto uit = halfedge.find(u);
        if (uit == halfedge.end()) continue;
        auto vit = uit->second.find(v);
        if (vit == uit->second.end()) continue;
        vit->second = std::nullopt;
        auto vit2 = halfedge.find(v);
        if (vit2 != halfedge.end()) {
            auto uit2 = vit2->second.find(u);
            if (uit2 != vit2->second.end() && !uit2->second.has_value()) {
                uit->second.erase(v);
                vit2->second.erase(u);
            }
        }
    }
    face.erase(fkey);
    triangulation.erase(fkey);
    facedata.erase(fkey);
    face_holes.erase(fkey);
    size_t n_edges = number_of_edges();
    if (linecolors.size() > n_edges) linecolors.resize(n_edges);
    if (widths.size() > n_edges) widths.resize(n_edges);
    size_t n_faces = face.size();
    if (facecolors.size() > n_faces) facecolors.resize(n_faces);
    triangle_bvh_built = false;
    triangle_bvh.reset();
    triangle_boxes_cache.clear();
    triangle_aabbs_cache.clear();
    triangle_indices_cache.clear();
    triangle_face_subidx_cache.clear();
    vertices_cache.clear();
}

void Mesh::remove_vertex(size_t vkey) {
    if (vertex.find(vkey) == vertex.end()) return;
    std::vector<size_t> faces_to_remove;
    for (const auto& [fk, verts] : face)
        for (size_t vk : verts)
            if (vk == vkey) { faces_to_remove.push_back(fk); break; }
    for (size_t fk : faces_to_remove)
        remove_face(fk);
    auto hit = halfedge.find(vkey);
    if (hit != halfedge.end()) {
        for (auto& [v, _] : hit->second) {
            auto vit = halfedge.find(v);
            if (vit != halfedge.end()) vit->second.erase(vkey);
        }
        halfedge.erase(vkey);
    }
    for (auto it = edgedata.begin(); it != edgedata.end(); ) {
        if (it->first.first == vkey || it->first.second == vkey)
            it = edgedata.erase(it);
        else ++it;
    }
    vertex.erase(vkey);
    size_t n_vertices = vertex.size();
    if (pointcolors.size() > n_vertices) pointcolors.resize(n_vertices);
    triangle_bvh_built = false;
    triangle_bvh.reset();
    triangle_boxes_cache.clear();
    triangle_aabbs_cache.clear();
    triangle_indices_cache.clear();
    triangle_face_subidx_cache.clear();
    vertices_cache.clear();
}

void Mesh::remove_edge(size_t u, size_t v) {
    std::vector<size_t> faces_to_remove;
    auto uit = halfedge.find(u);
    if (uit != halfedge.end()) {
        auto vit = uit->second.find(v);
        if (vit != uit->second.end() && vit->second.has_value())
            faces_to_remove.push_back(*vit->second);
    }
    auto vit = halfedge.find(v);
    if (vit != halfedge.end()) {
        auto uit2 = vit->second.find(u);
        if (uit2 != vit->second.end() && uit2->second.has_value())
            faces_to_remove.push_back(*uit2->second);
    }
    for (size_t fk : faces_to_remove)
        remove_face(fk);
    auto hit = halfedge.find(u);
    if (hit != halfedge.end()) hit->second.erase(v);
    auto hit2 = halfedge.find(v);
    if (hit2 != halfedge.end()) hit2->second.erase(u);
    edgedata.erase({u, v});
    edgedata.erase({v, u});
    size_t n_edges = number_of_edges();
    if (linecolors.size() > n_edges) linecolors.resize(n_edges);
    if (widths.size() > n_edges) widths.resize(n_edges);
    triangle_bvh_built = false;
    triangle_bvh.reset();
    triangle_boxes_cache.clear();
    triangle_aabbs_cache.clear();
    triangle_indices_cache.clear();
    triangle_face_subidx_cache.clear();
    vertices_cache.clear();
}

void Mesh::flip_face(size_t fkey) {
    auto it = face.find(fkey);
    if (it == face.end()) return;
    auto fv = it->second;
    remove_face(fkey);
    std::reverse(fv.begin(), fv.end());
    add_face(fv, fkey);
}

void Mesh::flip() {
    for (auto& [fkey, verts] : face)
        std::reverse(verts.begin(), verts.end());
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
}

std::optional<std::vector<std::pair<size_t, size_t>>> Mesh::edge_edges(size_t u, size_t v) const {
    bool uv = halfedge.count(u) && halfedge.at(u).count(v);
    bool vu = halfedge.count(v) && halfedge.at(v).count(u);
    if (!uv && !vu) return std::nullopt;
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

std::optional<std::vector<size_t>> Mesh::edge_faces(size_t u, size_t v) const {
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
    if (!f0 && !f1) return std::nullopt;
    std::vector<size_t> result;
    if (f0) result.push_back(*f0);
    if (f1) result.push_back(*f1);
    return result;
}

std::optional<Line> Mesh::edge_line(size_t u, size_t v) const {
    bool uv = halfedge.count(u) && halfedge.at(u).count(v);
    bool vu = halfedge.count(v) && halfedge.at(v).count(u);
    if (!uv && !vu) return std::nullopt;
    auto pu = vertex_point(u);
    auto pv = vertex_point(v);
    if (!pu || !pv) return std::nullopt;
    return Line::from_points(*pu, *pv);
}

std::optional<std::vector<std::pair<size_t, size_t>>> Mesh::face_edges(size_t face_key) const {
    auto it = face.find(face_key);
    if (it == face.end()) return std::nullopt;
    std::vector<std::pair<size_t, size_t>> edges;
    const auto& verts = it->second;
    size_t n = verts.size();
    for (size_t i = 0; i < n; ++i) {
        edges.push_back({verts[i], verts[(i + 1) % n]});
    }
    return edges;
}

std::optional<std::vector<size_t>> Mesh::face_faces(size_t face_key) const {
    auto fe = face_edges(face_key);
    if (!fe) return std::nullopt;
    std::vector<size_t> neighbors;
    for (const auto& [u, v] : *fe) {
        auto it = halfedge.find(v);
        if (it == halfedge.end()) continue;
        auto jt = it->second.find(u);
        if (jt != it->second.end() && jt->second.has_value()) {
            neighbors.push_back(*jt->second);
        }
    }
    return neighbors;
}

std::optional<std::vector<Point>> Mesh::face_points(size_t face_key) const {
    auto fv = face_vertices(face_key);
    if (!fv) return std::nullopt;
    std::vector<Point> pts;
    for (auto vk : *fv) {
        auto p = vertex_point(vk);
        if (!p) return std::nullopt;
        pts.push_back(*p);
    }
    return pts;
}

std::optional<Polyline> Mesh::face_polyline(size_t face_key) const {
    auto pts = face_points(face_key);
    if (!pts) return std::nullopt;
    return Polyline(*pts);
}

std::optional<std::vector<size_t>> Mesh::face_vertices(size_t face_key) const {
    auto it = face.find(face_key);
    if (it == face.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::optional<std::vector<std::pair<size_t, size_t>>> Mesh::vertex_edges(size_t vertex_key) const {
    auto it = halfedge.find(vertex_key);
    if (it == halfedge.end()) return std::nullopt;
    std::vector<std::pair<size_t, size_t>> edges;
    for (const auto& [u, _] : it->second) {
        edges.push_back({vertex_key, u});
    }
    return edges;
}

std::optional<std::vector<size_t>> Mesh::vertex_faces(size_t vertex_key) const {
    auto it = halfedge.find(vertex_key);
    if (it == halfedge.end()) return std::nullopt;
    std::vector<size_t> faces;
    for (const auto& [v, face_opt] : it->second) {
        if (face_opt.has_value()) faces.push_back(*face_opt);
    }
    return faces;
}

std::optional<Point> Mesh::vertex_point(size_t vertex_key) const {
    auto it = vertex.find(vertex_key);
    if (it == vertex.end()) {
        return std::nullopt;
    }
    return it->second.position();
}

std::optional<std::vector<size_t>> Mesh::vertex_vertices(size_t vertex_key) const {
    auto it = halfedge.find(vertex_key);
    if (it == halfedge.end()) return std::nullopt;
    std::vector<size_t> neighbors;
    for (const auto& [v, _] : it->second) {
        neighbors.push_back(v);
    }
    return neighbors;
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
    auto fe = face_edges(face_key);
    if (!fe) return false;
    for (const auto& [u, v] : *fe) {
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

double Mesh::area() const {
    double total = 0.0;
    for (const auto& [fk, _] : face) {
        auto a = face_area(fk);
        if (a) total += *a;
    }
    return total;
}

Point Mesh::centroid() const {
    double x = 0, y = 0, z = 0;
    for (const auto& [vk, v] : vertex) {
        auto p = v.position();
        x += p[0]; y += p[1]; z += p[2];
    }
    double n = vertex.empty() ? 1.0 : (double)vertex.size();
    return Point(x / n, y / n, z / n);
}

std::optional<double> Mesh::dihedral_angle(size_t u, size_t v) const {
    auto ef = edge_faces(u, v);
    if (!ef || ef->size() < 2) return std::nullopt;
    auto n0_opt = face_normal((*ef)[0]);
    auto n1_opt = face_normal((*ef)[1]);
    if (!n0_opt.has_value() || !n1_opt.has_value()) return std::nullopt;
    double dot = std::clamp(n0_opt->dot(*n1_opt), -1.0, 1.0);
    return (Tolerance::PI - std::acos(dot)) * 180.0 / Tolerance::PI;
}

std::tuple<std::map<std::pair<size_t,size_t>,double>, std::vector<Polyline>, std::vector<Point>>
Mesh::dihedral_angles(double scale, bool with_arcs, bool with_points) const {
    std::map<std::pair<size_t,size_t>,double> angles;
    std::vector<Polyline> arcs;
    std::vector<Point> points;
    const int arc_n = 12;
    for (auto& [u, v] : edges()) {
        auto da = dihedral_angle(u, v);
        if (!da) continue;
        angles[{u, v}] = *da;
        double deg = *da;
        auto ep0 = vertex_point(u);
        auto ep1 = vertex_point(v);
        if (!ep0 || !ep1) continue;
        double mx = ((*ep0)[0]+(*ep1)[0])*0.5;
        double my = ((*ep0)[1]+(*ep1)[1])*0.5;
        double mz = ((*ep0)[2]+(*ep1)[2])*0.5;
        if (scale == 0.0) {
            if (with_points) {
                Point pt(mx, my, mz, std::to_string(deg));
                pt.pointcolor = Color(240, 220, 0);
                points.push_back(pt);
            }
            continue;
        }
        auto ef = edge_faces(u, v);
        if (!ef || ef->size() < 2) continue;
        double ex=(*ep1)[0]-(*ep0)[0], ey=(*ep1)[1]-(*ep0)[1], ez=(*ep1)[2]-(*ep0)[2];
        double elen=std::sqrt(ex*ex+ey*ey+ez*ez);
        if (elen < 1e-10) continue;
        ex/=elen; ey/=elen; ez/=elen;
        auto fc0 = face_centroid((*ef)[0]);
        auto fc1 = face_centroid((*ef)[1]);
        if (!fc0 || !fc1) continue;
        double d0x=(*fc0)[0]-mx, d0y=(*fc0)[1]-my, d0z=(*fc0)[2]-mz;
        double dot0=d0x*ex+d0y*ey+d0z*ez;
        d0x-=dot0*ex; d0y-=dot0*ey; d0z-=dot0*ez;
        double d0len=std::sqrt(d0x*d0x+d0y*d0y+d0z*d0z);
        if (d0len < 1e-10) continue;
        d0x/=d0len; d0y/=d0len; d0z/=d0len;
        double d1x=(*fc1)[0]-mx, d1y=(*fc1)[1]-my, d1z=(*fc1)[2]-mz;
        double dot1=d1x*ex+d1y*ey+d1z*ez;
        d1x-=dot1*ex; d1y-=dot1*ey; d1z-=dot1*ez;
        double d1len=std::sqrt(d1x*d1x+d1y*d1y+d1z*d1z);
        if (d1len < 1e-10) continue;
        d1x/=d1len; d1y/=d1len; d1z/=d1len;
        double theta=std::acos(std::max(-1.0, std::min(1.0, d0x*d1x+d0y*d1y+d0z*d1z)));
        if (std::abs(std::sin(theta)) < 1e-10) continue;
        std::vector<Point> arc_pts;
        for (int j = 0; j <= arc_n; j++) {
            double t=(double)j/arc_n;
            double w1=std::sin((1.0-t)*theta)/std::sin(theta);
            double w2=std::sin(t*theta)/std::sin(theta);
            arc_pts.push_back(Point(
                mx+(w1*d0x+w2*d1x)*scale,
                my+(w1*d0y+w2*d1y)*scale,
                mz+(w1*d0z+w2*d1z)*scale));
        }
        if (with_arcs) {
            Polyline arc(arc_pts);
            arc.name = "dihedral_e"+std::to_string(u)+"_"+std::to_string(v)+"="+std::to_string(deg);
            arc.linecolor = Color(240, 220, 0);
            arcs.push_back(arc);
        }
        if (with_points) {
            Point pt(arc_pts[arc_n/2][0], arc_pts[arc_n/2][1], arc_pts[arc_n/2][2], std::to_string(deg));
            pt.pointcolor = Color(240, 220, 0);
            points.push_back(pt);
        }
    }
    return {angles, arcs, points};
}

std::optional<double> Mesh::face_area(size_t face_key) const {
    auto vertices_opt = face_vertices(face_key);
    if (!vertices_opt.has_value() || vertices_opt->size() < 3) {
        return 0.0;
    }
    
    const auto& vertices = *vertices_opt;
    double area = 0.0;
    auto p0_opt = vertex_point(vertices[0]);
    if (!p0_opt) return std::nullopt;
    const auto& p0 = *p0_opt;
    
    for (size_t i = 1; i < vertices.size() - 1; ++i) {
        auto p1_opt = vertex_point(vertices[i]);
        auto p2_opt = vertex_point(vertices[i + 1]);
        if (!p1_opt || !p2_opt) return std::nullopt;
        
        const auto& p1 = *p1_opt;
        const auto& p2 = *p2_opt;
        
        Vector u(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
        Vector v(p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]);
        
        area += u.cross(v).magnitude() * 0.5;
    }
    return area;
}

std::optional<Point> Mesh::face_centroid(size_t face_key) const {
    auto verts = face_vertices(face_key);
    if (!verts || verts->empty()) return std::nullopt;
    double x = 0, y = 0, z = 0;
    for (auto vk : *verts) {
        auto p = vertex_point(vk);
        if (!p) return std::nullopt;
        x += (*p)[0]; y += (*p)[1]; z += (*p)[2];
    }
    double n = (double)verts->size();
    return Point(x / n, y / n, z / n);
}

std::optional<Vector> Mesh::face_normal(size_t face_key) const {
    auto vertices_opt = face_vertices(face_key);
    if (!vertices_opt.has_value() || vertices_opt->size() < 3) {
        return std::nullopt;
    }

    const auto& vertices = *vertices_opt;
    auto p0_opt = vertex_point(vertices[0]);
    auto p1_opt = vertex_point(vertices[1]);
    auto p2_opt = vertex_point(vertices[2]);

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

    auto center_opt = vertex_point(vertex_key);
    auto prev_opt = vertex_point(prev_vertex);
    auto next_opt = vertex_point(next_vertex);

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

std::optional<Vector> Mesh::vertex_normal(size_t vertex_key) const {
    return vertex_normal_weighted(vertex_key, NormalWeighting::Area);
}

std::optional<Vector> Mesh::vertex_normal_weighted(size_t vertex_key, NormalWeighting weighting) const {
    auto faces_opt = vertex_faces(vertex_key);
    if (!faces_opt || faces_opt->empty()) {
        return std::nullopt;
    }

    Vector normal_acc(0.0, 0.0, 0.0);

    for (size_t face_key : *faces_opt) {
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

std::map<size_t, Vector> Mesh::vertex_normals() const {
    return vertex_normals_weighted(NormalWeighting::Area);
}

std::map<size_t, Vector> Mesh::vertex_normals_weighted(NormalWeighting weighting) const {
    std::map<size_t, Vector> acc;
    for (const auto& [fk, vkeys] : face) {
        size_t n = vkeys.size();
        if (n < 3) continue;
        std::vector<double> px(n), py(n), pz(n);
        bool ok = true;
        for (size_t i = 0; i < n; ++i) {
            auto p = vertex_point(vkeys[i]);
            if (!p) { ok = false; break; }
            px[i] = (*p)[0]; py[i] = (*p)[1]; pz[i] = (*p)[2];
        }
        if (!ok) continue;
        double ex = px[1]-px[0], ey = py[1]-py[0], ez = pz[1]-pz[0];
        double fx = px[2]-px[0], fy = py[2]-py[0], fz = pz[2]-pz[0];
        double cnx = ey*fz-ez*fy, cny = ez*fx-ex*fz, cnz = ex*fy-ey*fx;
        double len = std::sqrt(cnx*cnx + cny*cny + cnz*cnz);
        if (len < Tolerance::ZERO_TOLERANCE) continue;
        double ux = cnx/len, uy = cny/len, uz = cnz/len;
        double area = 0.0;
        if (weighting == NormalWeighting::Area) {
            for (size_t i = 1; i+1 < n; ++i) {
                double ax = px[i]-px[0], ay = py[i]-py[0], az = pz[i]-pz[0];
                double bx = px[i+1]-px[0], by = py[i+1]-py[0], bz = pz[i+1]-pz[0];
                double cx = ay*bz-az*by, cy = az*bx-ax*bz, cz = ax*by-ay*bx;
                area += std::sqrt(cx*cx + cy*cy + cz*cz) * 0.5;
            }
        }
        for (size_t i = 0; i < n; ++i) {
            double weight;
            if (weighting == NormalWeighting::Uniform) {
                weight = 1.0;
            } else if (weighting == NormalWeighting::Area) {
                weight = area;
            } else {
                size_t prev = (i + n - 1) % n, next = (i + 1) % n;
                double ax = px[prev]-px[i], ay = py[prev]-py[i], az = pz[prev]-pz[i];
                double bx = px[next]-px[i], by = py[next]-py[i], bz = pz[next]-pz[i];
                double a_len = std::sqrt(ax*ax + ay*ay + az*az);
                double b_len = std::sqrt(bx*bx + by*by + bz*bz);
                if (a_len < Tolerance::ZERO_TOLERANCE || b_len < Tolerance::ZERO_TOLERANCE) weight = 0.0;
                else { double cos_a = std::clamp((ax*bx+ay*by+az*bz)/(a_len*b_len), -1.0, 1.0); weight = std::acos(cos_a); }
            }
            auto& v = acc[vkeys[i]];
            v[0] = v[0] + ux * weight;
            v[1] = v[1] + uy * weight;
            v[2] = v[2] + uz * weight;
        }
    }
    std::map<size_t, Vector> normals;
    for (auto& [vk, v] : acc) {
        double len = v.magnitude();
        if (len > Tolerance::ZERO_TOLERANCE)
            normals[vk] = Vector(v[0] / len, v[1] / len, v[2] / len);
    }
    return normals;
}

double Mesh::volume() const {
    double total = 0.0;
    for (const auto& [fk, vkeys] : face) {
        if (vkeys.size() < 3) continue;
        auto p0o = vertex_point(vkeys[0]);
        if (!p0o) continue;
        const auto& p0 = *p0o;
        for (size_t i = 1; i + 1 < vkeys.size(); ++i) {
            auto p1o = vertex_point(vkeys[i]);
            auto p2o = vertex_point(vkeys[i + 1]);
            if (!p1o || !p2o) continue;
            const auto& p1 = *p1o;
            const auto& p2 = *p2o;
            total += p0[0] * (p1[1] * p2[2] - p1[2] * p2[1])
                   + p0[1] * (p1[2] * p2[0] - p1[0] * p2[2])
                   + p0[2] * (p1[0] * p2[1] - p1[1] * p2[0]);
        }
    }
    return std::abs(total) / 6.0;
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
        if (vkeys.size() > 1 && vkeys.back() == vkeys.front())
            vkeys.pop_back();
        if (vkeys.size() < 3) continue;
        auto fk = mesh.add_face(vkeys);
        if (fk && vkeys.size() >= 4) {
            int np = (int)poly.size();
            double nx=0, ny=0, nz=0;
            for (int i = 0; i < np; i++) {
                const auto& a = poly[i]; const auto& b = poly[(i+1)%np];
                nx += (a[1]-b[1])*(a[2]+b[2]);
                ny += (a[2]-b[2])*(a[0]+b[0]);
                nz += (a[0]-b[0])*(a[1]+b[1]);
            }
            double nlen = std::sqrt(nx*nx+ny*ny+nz*nz);
            if (nlen > 1e-12) {
                nx/=nlen; ny/=nlen; nz/=nlen;
                double ux=1,uy=0,uz=0;
                if (std::abs(nx) > 0.9) { ux=0; uy=1; }
                double dot=ux*nx+uy*ny+uz*nz;
                ux-=dot*nx; uy-=dot*ny; uz-=dot*nz;
                double um=std::sqrt(ux*ux+uy*uy+uz*uz);
                ux/=um; uy/=um; uz/=um;
                double vx=ny*uz-nz*uy, vy=nz*ux-nx*uz, vz=nx*uy-ny*ux;
                std::vector<std::pair<double,double>> bpts;
                int nk = (int)vkeys.size();
                for (int i = 0; i < nk; ++i)
                    bpts.push_back({poly[i][0]*ux+poly[i][1]*uy+poly[i][2]*uz,
                                    poly[i][0]*vx+poly[i][1]*vy+poly[i][2]*vz});
                auto tris = cdt_triangulate(bpts, {});
                std::vector<std::array<size_t,3>> tri_list;
                for (const auto& t : tris)
                    tri_list.push_back({vkeys[t[0]], vkeys[t[1]], vkeys[t[2]]});
                mesh.triangulation[*fk] = tri_list;
            }
        }
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

Mesh Mesh::create_box(double x, double y, double z) {
    double hx = x * 0.5, hy = y * 0.5, hz = z * 0.5;
    std::vector<Point> vertices = {
        Point(-hx, -hy, -hz),
        Point( hx, -hy, -hz),
        Point( hx,  hy, -hz),
        Point(-hx,  hy, -hz),
        Point(-hx, -hy,  hz),
        Point( hx, -hy,  hz),
        Point( hx,  hy,  hz),
        Point(-hx,  hy,  hz),
    };
    std::vector<std::vector<size_t>> faces = {
        {0, 3, 2, 1},  // bottom
        {4, 5, 6, 7},  // top
        {0, 1, 5, 4},  // front
        {2, 3, 7, 6},  // back
        {0, 4, 7, 3},  // left
        {1, 2, 6, 5},  // right
    };
    return from_vertices_and_faces(vertices, faces);
}

Mesh Mesh::create_dodecahedron(double edge) {
    double phi = (1.0 + std::sqrt(5.0)) / 2.0;
    double ip = 1.0 / phi;
    double s = edge / (2.0 * ip);
    Point verts[20] = {
        Point(s, s, s),
        Point(s, s, -s),
        Point(s, -s, s),
        Point(s, -s, -s),
        Point(-s, s, s),
        Point(-s, s, -s),
        Point(-s, -s, s),
        Point(-s, -s, -s),
        Point(0, s*ip, s*phi),
        Point(0, s*ip, -s*phi),
        Point(0, -s*ip, s*phi),
        Point(0, -s*ip, -s*phi),
        Point(s*ip, s*phi, 0),
        Point(s*ip, -s*phi, 0),
        Point(-s*ip, s*phi, 0),
        Point(-s*ip, -s*phi, 0),
        Point(s*phi, 0, s*ip),
        Point(s*phi, 0, -s*ip),
        Point(-s*phi, 0, s*ip),
        Point(-s*phi, 0, -s*ip),
    };
    int idx[12][5] = {
        {0, 8,10, 2,16}, {0,16,17, 1,12}, {0,12,14, 4, 8},
        {1,17, 3,11, 9}, {1, 9, 5,14,12}, {2,10, 6,15,13},
        {2,13, 3,17,16}, {3,13,15, 7,11}, {4,14, 5,19,18},
        {4,18, 6,10, 8}, {5, 9,11, 7,19}, {6,18,19, 7,15},
    };
    std::vector<std::vector<Point>> faces;
    for (auto& f : idx)
        faces.push_back({verts[f[0]], verts[f[1]], verts[f[2]], verts[f[3]], verts[f[4]]});
    return from_polylines(faces, 1e-10);
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
    std::vector<size_t> vkeys;
    vkeys.reserve(verts.size());
    for (const auto& pt : verts) vkeys.push_back(mesh.add_vertex(pt));

    // Optionally delete the exterior face (most-negative signed area)
    if (delete_boundary_face && !face_cycles.empty()) {
        size_t min_idx = 0;
        double min_area = std::numeric_limits<double>::max();
        for (size_t i = 0; i < face_cycles.size(); ++i) {
            double area = 0.0;
            const auto& cyc = face_cycles[i];
            size_t cn = cyc.size();
            for (size_t j = 0; j < cn; ++j) {
                size_t a = cyc[j], b = cyc[(j+1)%cn];
                area += verts[a][0] * verts[b][1] - verts[b][0] * verts[a][1];
            }
            area *= 0.5;
            if (area < min_area) { min_area = area; min_idx = i; }
        }
        face_cycles.erase(face_cycles.begin() + min_idx);
    }

    for (const auto& cycle : face_cycles) {
        std::vector<size_t> fvkeys;
        fvkeys.reserve(cycle.size());
        for (size_t vid : cycle) fvkeys.push_back(vkeys[vid]);
        auto fkey_opt = mesh.add_face(fvkeys);
        if (fkey_opt.has_value()) {
            std::vector<Point> pts;
            pts.reserve(cycle.size());
            for (size_t vid : cycle) pts.push_back(verts[vid]);
            auto tris = Triangulation2D::triangulate(Polyline(pts));
            std::vector<std::array<size_t, 3>> tri_list;
            for (const auto& t : tris)
                tri_list.push_back({vkeys[cycle[t.v0]], vkeys[cycle[t.v1]], vkeys[cycle[t.v2]]});
            mesh.triangulation[fkey_opt.value()] = tri_list;
        }
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

    if (hole_pts_3d.empty()) {
        std::vector<size_t> fvkeys(vkeys.begin(), vkeys.begin() + border.size());
        auto fkey = mesh.add_face(fvkeys);
        if (fkey.has_value()) {
            std::vector<std::array<size_t,3>> tri_list;
            for (const auto& f : tris) {
                if (vkeys[f[0]] == vkeys[f[1]] || vkeys[f[1]] == vkeys[f[2]] || vkeys[f[2]] == vkeys[f[0]]) continue;
                tri_list.push_back({vkeys[f[0]], vkeys[f[1]], vkeys[f[2]]});
            }
            std::unordered_set<size_t> covered;
            for (const auto& t : tri_list) { covered.insert(t[0]); covered.insert(t[1]); covered.insert(t[2]); }
            size_t n_vk = border.size();
            for (size_t m = 0; m < n_vk; ++m) {
                if (!covered.count(vkeys[m]))
                    tri_list.push_back({vkeys[(m + n_vk - 1) % n_vk], vkeys[m], vkeys[(m + 1) % n_vk]});
            }
            mesh.triangulation[fkey.value()] = tri_list;
        }
    } else {
        std::vector<size_t> fvkeys(vkeys.begin(), vkeys.begin() + border.size());
        auto fkey = mesh.add_face(fvkeys);
        if (fkey.has_value()) {
            std::vector<std::vector<size_t>> hole_rings;
            size_t off = border.size();
            for (const auto& h : hole_pts_3d) {
                std::vector<size_t> ring(vkeys.begin()+off, vkeys.begin()+off+h.size());
                hole_rings.push_back(ring); off += h.size();
            }
            mesh.set_face_holes(fkey.value(), hole_rings);
            std::vector<std::array<size_t,3>> tri_list;
            for (const auto& f : tris)
                if (vkeys[f[0]]!=vkeys[f[1]] && vkeys[f[1]]!=vkeys[f[2]] && vkeys[f[2]]!=vkeys[f[0]])
                    tri_list.push_back({vkeys[f[0]], vkeys[f[1]], vkeys[f[2]]});
            mesh.triangulation[fkey.value()] = tri_list;
        }
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

    // Orient zaxis from bottom toward top so winding check is consistent
    {
        Point c0 = polylines0[border_idx].center();
        Point c1 = polylines1[border_idx].center();
        Vector bottom_to_top(c1[0]-c0[0], c1[1]-c0[1], c1[2]-c0[2]);
        if (zaxis.dot(bottom_to_top) < 0) {
            zaxis = Vector(-zaxis[0], -zaxis[1], -zaxis[2]);
            yaxis = Vector(-yaxis[0], -yaxis[1], -yaxis[2]);
        }
    }

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
    struct PolyInfo { size_t bot_off; size_t bot_n; size_t top_off; size_t top_n; };
    std::vector<PolyInfo> poly_infos;
    std::vector<Point> all_bot_pts;
    std::vector<Point> all_top_pts;

    for (size_t oi = 0; oi < order.size(); ++oi) {
        int idx = order[oi];
        auto bot = get_open_points(polylines0[idx]);
        auto top = get_open_points(polylines1[idx]);

        bool is_border = (oi == 0);
        double area = signed_area_2d(bot);
        if ((is_border && area < 0) || (!is_border && area > 0)) {
            std::reverse(bot.begin(), bot.end());
            std::reverse(top.begin(), top.end());
        }

        poly_infos.push_back({all_bot_pts.size(), bot.size(), all_top_pts.size(), top.size()});
        for (const auto& p : bot) all_bot_pts.push_back(p);
        for (const auto& p : top) all_top_pts.push_back(p);
    }

    // Build mesh
    Mesh mesh;

    std::vector<size_t> bot_vkeys;
    for (const auto& p : all_bot_pts)
        bot_vkeys.push_back(mesh.add_vertex(p));

    std::vector<size_t> top_vkeys;
    for (const auto& p : all_top_pts)
        top_vkeys.push_back(mesh.add_vertex(p));

    if (cap) {
        // Bottom cap CDT
        std::vector<std::pair<double,double>> b_border_2d;
        for (size_t i = 0; i < poly_infos[0].bot_n; ++i)
            b_border_2d.push_back(project_2d(all_bot_pts[i]));
        std::vector<std::vector<std::pair<double,double>>> b_holes_2d;
        for (size_t h = 1; h < poly_infos.size(); ++h) {
            std::vector<std::pair<double,double>> hole;
            for (size_t i = poly_infos[h].bot_off; i < poly_infos[h].bot_off + poly_infos[h].bot_n; ++i)
                hole.push_back(project_2d(all_bot_pts[i]));
            b_holes_2d.push_back(std::move(hole));
        }
        auto b_tris = cdt_triangulate(b_border_2d, b_holes_2d);
        std::vector<size_t> bot_fvkeys(poly_infos[0].bot_n);
        for (size_t i = 0; i < poly_infos[0].bot_n; ++i) bot_fvkeys[i] = bot_vkeys[poly_infos[0].bot_n - 1 - i];
        auto fk_bot = mesh.add_face(bot_fvkeys);
        if (fk_bot.has_value()) {
            if (!b_holes_2d.empty()) {
                std::vector<std::vector<size_t>> hole_rings;
                for (size_t h = 1; h < poly_infos.size(); ++h) {
                    std::vector<size_t> ring;
                    for (size_t i = poly_infos[h].bot_off; i < poly_infos[h].bot_off + poly_infos[h].bot_n; ++i)
                        ring.push_back(bot_vkeys[i]);
                    hole_rings.push_back(std::move(ring));
                }
                mesh.set_face_holes(fk_bot.value(), std::move(hole_rings));
            }
            std::vector<std::array<size_t,3>> tri_list;
            for (const auto& f : b_tris) tri_list.push_back({bot_vkeys[f[0]], bot_vkeys[f[2]], bot_vkeys[f[1]]});
            mesh.triangulation[fk_bot.value()] = tri_list;
        }

        // Top cap CDT
        std::vector<std::pair<double,double>> t_border_2d;
        for (size_t i = 0; i < poly_infos[0].top_n; ++i)
            t_border_2d.push_back(project_2d(all_top_pts[i]));
        std::vector<std::vector<std::pair<double,double>>> t_holes_2d;
        for (size_t h = 1; h < poly_infos.size(); ++h) {
            std::vector<std::pair<double,double>> hole;
            for (size_t i = poly_infos[h].top_off; i < poly_infos[h].top_off + poly_infos[h].top_n; ++i)
                hole.push_back(project_2d(all_top_pts[i]));
            t_holes_2d.push_back(std::move(hole));
        }
        auto t_tris = cdt_triangulate(t_border_2d, t_holes_2d);
        std::vector<size_t> top_fvkeys(poly_infos[0].top_n);
        for (size_t i = 0; i < poly_infos[0].top_n; ++i) top_fvkeys[i] = top_vkeys[i];
        auto fk_top = mesh.add_face(top_fvkeys);
        if (fk_top.has_value()) {
            if (!t_holes_2d.empty()) {
                std::vector<std::vector<size_t>> hole_rings;
                for (size_t h = 1; h < poly_infos.size(); ++h) {
                    std::vector<size_t> ring;
                    for (size_t i = poly_infos[h].top_off; i < poly_infos[h].top_off + poly_infos[h].top_n; ++i)
                        ring.push_back(top_vkeys[i]);
                    hole_rings.push_back(std::move(ring));
                }
                mesh.set_face_holes(fk_top.value(), std::move(hole_rings));
            }
            std::vector<std::array<size_t,3>> tri_list;
            for (const auto& f : t_tris) tri_list.push_back({top_vkeys[f[0]], top_vkeys[f[1]], top_vkeys[f[2]]});
            mesh.triangulation[fk_top.value()] = tri_list;
        }
        for (size_t h = 1; h < poly_infos.size(); ++h) {
            size_t bn = poly_infos[h].bot_n, bo = poly_infos[h].bot_off;
            size_t tn = poly_infos[h].top_n, t_off = poly_infos[h].top_off;
            std::vector<size_t> bh(bn);
            for (size_t j = 0; j < bn; ++j) bh[j] = bot_vkeys[bo + bn - 1 - j];
            mesh.add_face(bh);
            std::vector<size_t> th(tn);
            for (size_t j = 0; j < tn; ++j) th[j] = top_vkeys[t_off + j];
            mesh.add_face(th);
        }
    }

    // Side faces: align by longest edge, quads for equal counts, zipper+triangles otherwise
    auto edsq = [](const std::vector<Point>& pts, size_t i) -> double {
        size_t j = (i + 1) % pts.size();
        double dx = pts[j][0]-pts[i][0], dy = pts[j][1]-pts[i][1], dz = pts[j][2]-pts[i][2];
        return dx*dx + dy*dy + dz*dz;
    };
    for (const auto& pi : poly_infos) {
        size_t bot_off = pi.bot_off, bot_n = pi.bot_n, top_off = pi.top_off, top_n = pi.top_n;
        std::vector<Point> bpts(all_bot_pts.begin()+bot_off, all_bot_pts.begin()+bot_off+bot_n);
        std::vector<Point> tpts(all_top_pts.begin()+top_off, all_top_pts.begin()+top_off+top_n);
        size_t ia = 0, ib = 0;
        double max_b = 0, max_t = 0;
        for (size_t k = 0; k < bot_n; ++k) { double v = edsq(bpts, k); if (v > max_b) { max_b = v; ia = k; } }
        for (size_t k = 0; k < top_n; ++k) { double v = edsq(tpts, k); if (v > max_t) { max_t = v; ib = k; } }
        if (bot_n == top_n) {
            for (size_t k = 0; k < bot_n; ++k) {
                size_t cb = bot_off+(ia+k)%bot_n, ct = top_off+(ib+k)%top_n;
                size_t nb = bot_off+(ia+k+1)%bot_n, nt = top_off+(ib+k+1)%top_n;
                mesh.add_face({bot_vkeys[cb], bot_vkeys[nb], top_vkeys[nt], top_vkeys[ct]});
            }
            continue;
        }
        std::vector<double> b_arcs(bot_n+1, 0.0);
        for (size_t k = 0; k < bot_n; ++k) {
            size_t i = (ia+k)%bot_n, j = (ia+k+1)%bot_n;
            double dx = bpts[j][0]-bpts[i][0], dy = bpts[j][1]-bpts[i][1], dz = bpts[j][2]-bpts[i][2];
            b_arcs[k+1] = b_arcs[k] + std::sqrt(dx*dx+dy*dy+dz*dz);
        }
        std::vector<double> t_arcs(top_n+1, 0.0);
        for (size_t k = 0; k < top_n; ++k) {
            size_t i = (ib+k)%top_n, j = (ib+k+1)%top_n;
            double dx = tpts[j][0]-tpts[i][0], dy = tpts[j][1]-tpts[i][1], dz = tpts[j][2]-tpts[i][2];
            t_arcs[k+1] = t_arcs[k] + std::sqrt(dx*dx+dy*dy+dz*dz);
        }
        double inv_b = b_arcs[bot_n] > 0 ? 1.0/b_arcs[bot_n] : 1.0;
        double inv_t = t_arcs[top_n] > 0 ? 1.0/t_arcs[top_n] : 1.0;
        size_t bi = 0, ti = 0;
        while (bi < bot_n || ti < top_n) {
            size_t cb = bot_off+(ia+bi)%bot_n, ct = top_off+(ib+ti)%top_n;
            size_t nb = bot_off+(ia+bi+1)%bot_n, nt = top_off+(ib+ti+1)%top_n;
            if (bi >= bot_n) {
                mesh.add_face({bot_vkeys[cb], top_vkeys[ct], top_vkeys[nt]}); ++ti;
            } else if (ti >= top_n) {
                mesh.add_face({bot_vkeys[cb], bot_vkeys[nb], top_vkeys[ct]}); ++bi;
            } else {
                double bp = b_arcs[bi+1]*inv_b, tp = t_arcs[ti+1]*inv_t;
                if (std::abs(bp-tp) < 1e-9) {
                    mesh.add_face({bot_vkeys[cb], bot_vkeys[nb], top_vkeys[nt], top_vkeys[ct]}); ++bi; ++ti;
                } else if (bp < tp) {
                    mesh.add_face({bot_vkeys[cb], bot_vkeys[nb], top_vkeys[ct]}); ++bi;
                } else {
                    mesh.add_face({bot_vkeys[cb], top_vkeys[ct], top_vkeys[nt]}); ++ti;
                }
            }
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

bool Mesh::transform(const Xform& xf) {
  for (auto& [idx, vdata] : vertex) {
    Point pt(vdata.x, vdata.y, vdata.z);
    xf.transform_point(pt);
    vdata.x = pt[0];
    vdata.y = pt[1];
    vdata.z = pt[2];
  }
  triangle_bvh_built = false;
  triangle_bvh.reset();
  triangle_boxes_cache.clear();
  triangle_aabbs_cache.clear();
  triangle_indices_cache.clear();
  triangle_face_subidx_cache.clear();
  vertices_cache.clear();
  return true;
}

void Mesh::transform() {
  transform(xform);
}

Mesh Mesh::transformed() const {
  Mesh result = *this;
  result.transform();
  return result;
}

Mesh Mesh::transformed(const Xform& xf) const {
  Mesh result = *this;
  result.transform(xf);
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

    // Face holes
    nlohmann::ordered_json face_holes_json;
    for (const auto& [fkey, rings] : face_holes) {
        nlohmann::json rings_arr = nlohmann::json::array();
        for (const auto& ring : rings)
            rings_arr.push_back(ring);
        face_holes_json[std::to_string(fkey)] = rings_arr;
    }
    data["face_holes"] = face_holes_json;

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
    data["objectcolor"] = objectcolor.jsondump();
    data["color_mode"] = color_mode_to_string(color_mode);

    // Point colors (RGBA)
    nlohmann::ordered_json pointcolors_arr = nlohmann::ordered_json::array();
    for (const auto& c : pointcolors) {
        pointcolors_arr.push_back(c.r); pointcolors_arr.push_back(c.g);
        pointcolors_arr.push_back(c.b); pointcolors_arr.push_back(c.a);
    }
    data["pointcolors"] = pointcolors_arr;

    nlohmann::ordered_json triangulation_json;
    for (const auto& [fkey, tris] : triangulation) {
        nlohmann::json tri_arr = nlohmann::json::array();
        for (const auto& t : tris)
            tri_arr.push_back({t[0], t[1], t[2]});
        triangulation_json[std::to_string(fkey)] = tri_arr;
    }
    data["triangulation"] = triangulation_json;

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
    data["xform"] = xform.jsondump();

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
    
    // Load face holes
    if (data.contains("face_holes")) {
        for (const auto& [fk_str, rings_val] : data["face_holes"].items()) {
            size_t fk = std::stoull(fk_str);
            std::vector<std::vector<size_t>> rings;
            for (const auto& ring : rings_val)
                rings.push_back(ring.get<std::vector<size_t>>());
            mesh.face_holes[fk] = rings;
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
    if (data.contains("xform")) {
        mesh.xform = Xform::jsonload(data["xform"]);
    }

    if (data.contains("objectcolor")) {
        mesh.objectcolor = Color::jsonload(data["objectcolor"]);
    }
    if (data.contains("color_mode")) {
        mesh.color_mode = color_mode_from_string(data["color_mode"].get<std::string>());
    }

    if (data.contains("triangulation")) {
        for (const auto& [fk_str, tris_val] : data["triangulation"].items()) {
            size_t fk = std::stoull(fk_str);
            std::vector<std::array<size_t,3>> tris;
            for (const auto& t : tris_val)
                tris.push_back({t[0].get<size_t>(), t[1].get<size_t>(), t[2].get<size_t>()});
            mesh.triangulation[fk] = tris;
        }
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

    std::vector<size_t> vertex_keys;
    vertex_keys.reserve(vertex.size());
    for (const auto& kv : vertex) vertex_keys.push_back(kv.first);
    std::sort(vertex_keys.begin(), vertex_keys.end());
    std::unordered_map<size_t, size_t> vkey_to_idx;
    vkey_to_idx.reserve(vertex_keys.size());
    for (size_t i = 0; i < vertex_keys.size(); ++i) vkey_to_idx[vertex_keys[i]] = i;

    std::vector<size_t> face_keys;
    face_keys.reserve(face.size());
    for (const auto& kv : face) face_keys.push_back(kv.first);
    std::sort(face_keys.begin(), face_keys.end());

    size_t tri_count = 0;
    for (size_t fi = 0; fi < faces_vec.size(); ++fi) {
        const auto& fv = faces_vec[fi];
        if (fv.size() < 3) continue;
        if (fv.size() >= 5 && fi < face_keys.size()) {
            auto it = triangulation.find(face_keys[fi]);
            if (it != triangulation.end()) { tri_count += it->second.size(); continue; }
        }
        tri_count += fv.size() - 2;
    }
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
        if (fv.size() >= 5 && fi < face_keys.size()) {
            auto it = triangulation.find(face_keys[fi]);
            if (it != triangulation.end()) {
                for (size_t j = 0; j < it->second.size(); ++j) {
                    const auto& t = it->second[j];
                    tasks.push_back(TriTask{static_cast<uint32_t>(vkey_to_idx.at(t[0])), static_cast<uint32_t>(vkey_to_idx.at(t[1])), static_cast<uint32_t>(vkey_to_idx.at(t[2])), fi, j, out});
                    out++;
                }
                continue;
            }
        }
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

void Mesh::json_dump(const std::string& filename) const {
    nlohmann::ordered_json j = jsondump();
    std::ofstream f(filename);
    f << j.dump(2);
}

Mesh Mesh::json_load(const std::string& filename) {
    std::ifstream f(filename);
    nlohmann::json j;
    f >> j;
    return jsonload(j);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Loft panels
///////////////////////////////////////////////////////////////////////////////////////////

static std::array<double,3> lp_newell_normal(const std::vector<Point>& pts) {
    double nx=0, ny=0, nz=0;
    int n = (int)pts.size();
    for (int i = 0; i < n; i++) {
        const auto& a = pts[i];
        const auto& b = pts[(i+1)%n];
        nx += (a[1]-b[1]) * (a[2]+b[2]);
        ny += (a[2]-b[2]) * (a[0]+b[0]);
        nz += (a[0]-b[0]) * (a[1]+b[1]);
    }
    return {nx, ny, nz};
}

static void lp_merge_collinear(std::vector<Point>& pts, std::vector<size_t>& vkeys) {
    const double tol = Tolerance::APPROXIMATION;
    const double zt2 = Tolerance::ZERO_TOLERANCE * Tolerance::ZERO_TOLERANCE;
    bool changed = true;
    while (changed) {
        changed = false;
        int m = (int)pts.size();
        if (m < 3) break;
        std::vector<Point> np; std::vector<size_t> nk;
        for (int i = 0; i < m; i++) {
            int p=(i-1+m)%m, nx=(i+1)%m;
            double ax=pts[i][0]-pts[p][0], ay=pts[i][1]-pts[p][1], az=pts[i][2]-pts[p][2];
            double bx=pts[nx][0]-pts[i][0], by=pts[nx][1]-pts[i][1], bz=pts[nx][2]-pts[i][2];
            double cx=ay*bz-az*by, cy=az*bx-ax*bz, cz=ax*by-ay*bx;
            double a2=ax*ax+ay*ay+az*az, b2=bx*bx+by*by+bz*bz;
            if (a2<zt2||b2<zt2||cx*cx+cy*cy+cz*cz<tol*tol*a2*b2) changed=true;
            else { np.push_back(pts[i]); nk.push_back(vkeys[i]); }
        }
        pts=np; vkeys=nk;
    }
}

static Point lp_offset_toward(const Point& p, double cx, double cy, double cz, double gap) {
    double dx=cx-p[0], dy=cy-p[1], dz=cz-p[2];
    double len=std::sqrt(dx*dx+dy*dy+dz*dz);
    if (len>1e-10) { dx*=gap/len; dy*=gap/len; dz*=gap/len; }
    return Point(p[0]+dx, p[1]+dy, p[2]+dz);
}

static Point lp_face_centroid(const Mesh& m, size_t fk) {
    auto vkeys = *m.face_vertices(fk);
    double cx=0,cy=0,cz=0;
    for (auto vk : vkeys) { auto p=*m.vertex_point(vk); cx+=p[0]; cy+=p[1]; cz+=p[2]; }
    return Point(cx/vkeys.size(), cy/vkeys.size(), cz/vkeys.size());
}

LoftResult Mesh::loft_panels(
    const std::vector<std::vector<Point>>& top_polygons,
    const std::vector<std::vector<Point>>& bot_polygons,
    double merge_precision,
    double edge_gap,
    double edge_match_threshold,
    bool   add_caps,
    bool   skip_triangles)
{
    Mesh top_mesh = Mesh::from_polylines(top_polygons, merge_precision);
    Mesh bot_mesh = Mesh::from_polylines(bot_polygons, merge_precision);

    std::vector<size_t> tfks, bfks;
    for (auto& [fk, _] : top_mesh.face) tfks.push_back(fk);
    for (auto& [fk, _] : bot_mesh.face) bfks.push_back(fk);

    std::vector<std::tuple<double, size_t, size_t>> dists;
    dists.reserve(tfks.size() * bfks.size());
    for (size_t ti = 0; ti < tfks.size(); ti++)
        for (size_t bi = 0; bi < bfks.size(); bi++)
            dists.push_back({lp_face_centroid(top_mesh, tfks[ti]).distance(
                             lp_face_centroid(bot_mesh, bfks[bi])), ti, bi});
    std::sort(dists.begin(), dists.end());

    std::vector<bool> top_used(tfks.size(), false), bot_used(bfks.size(), false);
    std::vector<std::pair<size_t,size_t>> face_match;
    for (auto& [d, ti, bi] : dists) {
        if (top_used[ti] || bot_used[bi]) continue;
        face_match.push_back({tfks[ti], bfks[bi]});
        top_used[ti] = true;
        bot_used[bi] = true;
    }
    std::sort(face_match.begin(), face_match.end());

    std::vector<LoftPanel> panels;
    panels.reserve(face_match.size());
    for (size_t si = 0; si < face_match.size(); si++) {
        auto& [tfk, bfk] = face_match[si];
        LoftPanel panel;
        panel.top_face_key = 0;
        panel.bot_face_key = 0;

        auto top_vkeys = *top_mesh.face_vertices(tfk);
        auto bot_vkeys = *bot_mesh.face_vertices(bfk);

        std::vector<Point> top_pts, bot_pts;
        for (auto vk : top_vkeys) top_pts.push_back(*top_mesh.vertex_point(vk));
        for (auto vk : bot_vkeys) bot_pts.push_back(*bot_mesh.vertex_point(vk));

        lp_merge_collinear(top_pts, top_vkeys);
        lp_merge_collinear(bot_pts, bot_vkeys);
        {
            double max_te = 0;
            int sz = (int)top_pts.size();
            for (int i = 0; i < sz; i++)
                max_te = std::max(max_te, top_pts[i].distance(top_pts[(i+1)%sz]));
            const double stol = max_te * 0.001;
            {
                std::vector<Point> tp; std::vector<size_t> tk;
                for (int i = 0; i < (int)top_pts.size(); i++) {
                    if (tp.empty() || tp.back().distance(top_pts[i]) > stol) {
                        tp.push_back(top_pts[i]); tk.push_back(top_vkeys[i]);
                    }
                }
                while ((int)tp.size() >= 3 && tp.back().distance(tp.front()) <= stol) {
                    tp.pop_back(); tk.pop_back();
                }
                if ((int)tp.size() >= 3) { top_pts = tp; top_vkeys = tk; }
            }
        }
        int n = (int)top_pts.size();
        int m = (int)bot_pts.size();

        double tcx=0,tcy=0,tcz=0, bcx=0,bcy=0,bcz=0;
        for (auto& p : top_pts) { tcx+=p[0]; tcy+=p[1]; tcz+=p[2]; }
        for (auto& p : bot_pts) { bcx+=p[0]; bcy+=p[1]; bcz+=p[2]; }
        tcx/=n; tcy/=n; tcz/=n;
        bcx/=m; bcy/=m; bcz/=m;
        double ax=tcx-bcx, ay=tcy-bcy, az=tcz-bcz;
        double alen=std::sqrt(ax*ax+ay*ay+az*az);
        if (alen>1e-12) {ax/=alen; ay/=alen; az/=alen;}

        auto [tnx,tny,tnz] = lp_newell_normal(top_pts);
        if (tnx*ax+tny*ay+tnz*az < 0) {
            std::reverse(top_pts.begin(), top_pts.end());
            std::reverse(top_vkeys.begin(), top_vkeys.end());
        }
        auto [bnx,bny,bnz] = lp_newell_normal(bot_pts);
        if (bnx*ax+bny*ay+bnz*az > 0) {
            std::reverse(bot_pts.begin(), bot_pts.end());
            std::reverse(bot_vkeys.begin(), bot_vkeys.end());
        }
        for (int i = 0; i < n; i++) {
            size_t lk = panel.mesh.add_vertex(top_pts[i]);
            panel.orig_top_to_local[top_vkeys[i]] = lk;
            panel.top_vertices.push_back(lk);
        }
        for (int j = 0; j < m; j++) {
            size_t lk = panel.mesh.add_vertex(bot_pts[j]);
            panel.orig_bot_to_local[bot_vkeys[j]] = lk;
            panel.bot_vertices.push_back(lk);
        }

        if (add_caps) {
            std::vector<size_t> top_cap;
            for (auto vk : top_vkeys) top_cap.push_back(panel.orig_top_to_local[vk]);
            auto fk = panel.mesh.add_face(top_cap);
            panel.top_face_key = fk;
            if (fk && top_cap.size() >= 3) {
                auto [nx, ny, nz] = lp_newell_normal(top_pts);
                double mag = std::sqrt(nx*nx + ny*ny + nz*nz);
                if (mag > 1e-12) {
                    nx /= mag; ny /= mag; nz /= mag;
                    double ux = 1, uy = 0, uz = 0;
                    if (std::abs(nx) > 0.9) { ux = 0; uy = 1; }
                    double dot = ux*nx + uy*ny + uz*nz;
                    ux -= dot*nx; uy -= dot*ny; uz -= dot*nz;
                    double um = std::sqrt(ux*ux + uy*uy + uz*uz);
                    ux /= um; uy /= um; uz /= um;
                    double vx = ny*uz - nz*uy, vy = nz*ux - nx*uz, vz = nx*uy - ny*ux;
                    std::vector<std::pair<double,double>> bpts;
                    for (const auto& p : top_pts)
                        bpts.push_back({p[0]*ux + p[1]*uy + p[2]*uz, p[0]*vx + p[1]*vy + p[2]*vz});
                    auto tris = cdt_triangulate(bpts, {});
                    if (!tris.empty()) {
                        std::vector<std::array<size_t,3>> tri_list;
                        for (const auto& t : tris)
                            tri_list.push_back({top_cap[t[0]], top_cap[t[1]], top_cap[t[2]]});
                        panel.mesh.set_face_triangulation(*fk, std::move(tri_list));
                    }
                }
            }
        }

        std::vector<Point> top_mids(n), bot_mids(m);
        for (int i = 0; i < n; i++)
            top_mids[i] = Point((top_pts[i][0]+top_pts[(i+1)%n][0])*0.5,
                                (top_pts[i][1]+top_pts[(i+1)%n][1])*0.5,
                                (top_pts[i][2]+top_pts[(i+1)%n][2])*0.5);
        for (int j = 0; j < m; j++)
            bot_mids[j] = Point((bot_pts[j][0]+bot_pts[(j+1)%m][0])*0.5,
                                (bot_pts[j][1]+bot_pts[(j+1)%m][1])*0.5,
                                (bot_pts[j][2]+bot_pts[(j+1)%m][2])*0.5);

        std::vector<int> bot_to_top(m, -1);
        std::vector<double> bot_dist(m, 1e300);
        for (int j = 0; j < m; j++)
            for (int i = 0; i < n; i++) {
                double d = bot_mids[j].distance(top_mids[i]);
                if (d < bot_dist[j]) { bot_dist[j] = d; bot_to_top[j] = i; }
            }

        std::vector<int> top_to_bot(n, -1);
        std::vector<double> top_dist(n, 1e300);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++) {
                double d = top_mids[i].distance(bot_mids[j]);
                if (d < top_dist[i]) { top_dist[i] = d; top_to_bot[i] = j; }
            }

        double avg = 0;
        for (int j = 0; j < m; j++) avg += bot_dist[j];
        avg /= m;
        double threshold = avg * edge_match_threshold;

        std::vector<bool> top_used_edge(n, false);
        for (int j = 0; j < m; j++) {
            size_t b0 = panel.orig_bot_to_local[bot_vkeys[j]];
            size_t b1 = panel.orig_bot_to_local[bot_vkeys[(j+1)%m]];
            int ti = bot_to_top[j];
            if (ti >= 0 && bot_dist[j] <= threshold && top_to_bot[ti] == j) {
                size_t t0 = panel.orig_top_to_local[top_vkeys[ti]];
                size_t t1 = panel.orig_top_to_local[top_vkeys[(ti+1)%n]];
                std::optional<size_t> fk;
                if (edge_gap > 0.0) {
                    auto pb0 = *panel.mesh.vertex_point(b0);
                    auto pb1 = *panel.mesh.vertex_point(b1);
                    auto pt0 = *panel.mesh.vertex_point(t0);
                    auto pt1 = *panel.mesh.vertex_point(t1);
                    double cx=(pb0[0]+pb1[0]+pt0[0]+pt1[0])*0.25;
                    double cy=(pb0[1]+pb1[1]+pt0[1]+pt1[1])*0.25;
                    double cz=(pb0[2]+pb1[2]+pt0[2]+pt1[2])*0.25;
                    size_t nb0 = panel.mesh.add_vertex(lp_offset_toward(pb0, cx, cy, cz, edge_gap));
                    size_t nb1 = panel.mesh.add_vertex(lp_offset_toward(pb1, cx, cy, cz, edge_gap));
                    fk = panel.mesh.add_face({nb0, t1, t0, nb1});
                } else {
                    fk = panel.mesh.add_face({b0, t1, t0, b1});
                }
                if (fk) {
                    LoftWallFace w; w.face_key = *fk; w.is_quad = true;
                    w.top_v0 = top_vkeys[ti]; w.top_v1 = top_vkeys[(ti+1)%n];
                    w.bot_v0 = bot_vkeys[(j+1)%m]; w.bot_v1 = bot_vkeys[j];
                    panel.wall_faces.push_back(w);
                }
                top_used_edge[ti] = true;
            } else if (!skip_triangles) {
                double best_d = 1e300; int best_tv = 0;
                for (int i = 0; i < n; i++) {
                    double d = bot_mids[j].distance(top_pts[i]);
                    if (d < best_d) { best_d = d; best_tv = i; }
                }
                size_t tv = panel.orig_top_to_local[top_vkeys[best_tv]];
                auto fk = panel.mesh.add_face({b0, tv, b1});
                if (fk) { LoftWallFace w; w.face_key = *fk; w.is_quad = false; panel.wall_faces.push_back(w); }
            }
        }

        if (!skip_triangles) {
            for (int i = 0; i < n; i++) {
                if (top_used_edge[i]) continue;
                size_t t0 = panel.orig_top_to_local[top_vkeys[i]];
                size_t t1 = panel.orig_top_to_local[top_vkeys[(i+1)%n]];
                double best_d = 1e300; int best_bv = 0;
                for (int j = 0; j < m; j++) {
                    double d = top_mids[i].distance(bot_pts[j]);
                    if (d < best_d) { best_d = d; best_bv = j; }
                }
                size_t bv = panel.orig_bot_to_local[bot_vkeys[best_bv]];
                auto fk = panel.mesh.add_face({t1, t0, bv});
                if (fk) { LoftWallFace w; w.face_key = *fk; w.is_quad = false; panel.wall_faces.push_back(w); }
            }
        }

        if (add_caps) {
            std::vector<size_t> bot_cap;
            for (int j = 0; j < m; j++)
                bot_cap.push_back(panel.orig_bot_to_local[bot_vkeys[j]]);
            auto bot_cap_fk = panel.mesh.add_face(bot_cap);
            panel.bot_face_key = bot_cap_fk;
            if (bot_cap_fk && bot_cap.size() >= 3) {
                auto [bcnx, bcny, bcnz] = lp_newell_normal(bot_pts);
                double bcmag = std::sqrt(bcnx*bcnx + bcny*bcny + bcnz*bcnz);
                if (bcmag > 1e-12) {
                    bcnx /= bcmag; bcny /= bcmag; bcnz /= bcmag;
                    double bcux = 1, bcuy = 0, bcuz = 0;
                    if (std::abs(bcnx) > 0.9) { bcux = 0; bcuy = 1; }
                    double bcdot = bcux*bcnx + bcuy*bcny + bcuz*bcnz;
                    bcux -= bcdot*bcnx; bcuy -= bcdot*bcny; bcuz -= bcdot*bcnz;
                    double bcum = std::sqrt(bcux*bcux + bcuy*bcuy + bcuz*bcuz);
                    bcux /= bcum; bcuy /= bcum; bcuz /= bcum;
                    double bcvx = bcny*bcuz - bcnz*bcuy, bcvy = bcnz*bcux - bcnx*bcuz, bcvz = bcnx*bcuy - bcny*bcux;
                    std::vector<std::pair<double,double>> bpts2;
                    for (const auto& p : bot_pts)
                        bpts2.push_back({p[0]*bcux + p[1]*bcuy + p[2]*bcuz, p[0]*bcvx + p[1]*bcvy + p[2]*bcvz});
                    auto btris = cdt_triangulate(bpts2, {});
                    if (!btris.empty()) {
                        std::vector<std::array<size_t,3>> tri_list;
                        for (const auto& t : btris)
                            tri_list.push_back({bot_cap[t[0]], bot_cap[t[1]], bot_cap[t[2]]});
                        panel.mesh.set_face_triangulation(*bot_cap_fk, std::move(tri_list));
                    }
                }
            }
        }

        std::map<size_t,size_t> fkey_to_idx;
        size_t fi = 0;
        for (const auto& [fk, _] : panel.mesh.face)
            fkey_to_idx[fk] = fi++;
        for (auto& w : panel.wall_faces) {
            w.face_index = fkey_to_idx[w.face_key];
            panel.face_roles[w.face_key] = w.is_quad ? LoftFaceRole::QuadWall : LoftFaceRole::TriWall;
        }
        if (panel.top_face_key) panel.face_roles[*panel.top_face_key] = LoftFaceRole::TopCap;
        if (panel.bot_face_key) panel.face_roles[*panel.bot_face_key] = LoftFaceRole::BotCap;

        panels.push_back(std::move(panel));
    }
    std::map<std::pair<size_t,size_t>, std::pair<size_t,size_t>> edge_to_wall;
    for (size_t pi = 0; pi < panels.size(); pi++)
        for (size_t wi = 0; wi < panels[pi].wall_faces.size(); wi++) {
            const auto& w = panels[pi].wall_faces[wi];
            if (!w.is_quad) continue;
            edge_to_wall[{w.top_v0, w.top_v1}] = {pi, wi};
        }
    std::vector<LoftAdjPair> adjacency;
    for (size_t pi = 0; pi < panels.size(); pi++)
        for (size_t wi = 0; wi < panels[pi].wall_faces.size(); wi++) {
            const auto& w = panels[pi].wall_faces[wi];
            if (!w.is_quad) continue;
            auto it = edge_to_wall.find({w.top_v1, w.top_v0});
            if (it != edge_to_wall.end() && it->second.first > pi)
                adjacency.push_back({pi, wi, it->second.first, it->second.second});
        }
    Mesh top_ordered, bot_ordered;
    for (size_t i = 0; i < panels.size(); i++) {
        std::vector<size_t> tvks, bvks;
        for (auto lk : panels[i].top_vertices) tvks.push_back(top_ordered.add_vertex(*panels[i].mesh.vertex_point(lk)));
        for (auto lk : panels[i].bot_vertices) bvks.push_back(bot_ordered.add_vertex(*panels[i].mesh.vertex_point(lk)));
        top_ordered.add_face(tvks, i);
        bot_ordered.add_face(bvks, i);
    }
    return {std::move(panels), std::move(adjacency), std::move(top_ordered), std::move(bot_ordered)};
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
        auto hit = face_holes.find(fkey);
        if (hit != face_holes.end()) {
            for (const auto& ring : hit->second) {
                auto* hole_proto = face_proto.add_holes();
                for (size_t v : ring) hole_proto->add_vertices(v);
            }
        }
    }

    // Triangulation
    for (const auto& [fkey, tris] : triangulation) {
        auto& tri_list = (*proto.mutable_triangulation())[fkey];
        for (const auto& t : tris) {
            tri_list.add_vertices(t[0]);
            tri_list.add_vertices(t[1]);
            tri_list.add_vertices(t[2]);
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

    // Object color
    auto* oc_proto = proto.mutable_objectcolor();
    oc_proto->set_guid(objectcolor.guid);
    oc_proto->set_name(objectcolor.name);
    oc_proto->set_r(objectcolor.r);
    oc_proto->set_g(objectcolor.g);
    oc_proto->set_b(objectcolor.b);
    oc_proto->set_a(objectcolor.a);
    proto.set_color_mode(static_cast<int>(color_mode));

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
        if (fdata.holes_size() > 0) {
            std::vector<std::vector<size_t>> rings;
            for (const auto& hole : fdata.holes()) {
                std::vector<size_t> ring;
                for (uint64_t v : hole.vertices()) ring.push_back(v);
                rings.push_back(ring);
            }
            mesh.face_holes[fkey] = rings;
        }
    }

    // Triangulation
    for (const auto& [fkey, tri_list] : proto.triangulation()) {
        std::vector<std::array<size_t, 3>> tris;
        const auto& vlist = tri_list.vertices();
        for (int i = 0; i + 2 < vlist.size(); i += 3)
            tris.push_back({static_cast<size_t>(vlist[i]), static_cast<size_t>(vlist[i+1]), static_cast<size_t>(vlist[i+2])});
        mesh.triangulation[fkey] = tris;
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

    // Object color
    {
        const auto& oc = proto.objectcolor();
        mesh.objectcolor = Color(oc.r(), oc.g(), oc.b(), oc.a());
        mesh.objectcolor.guid = oc.guid();
        mesh.objectcolor.name = oc.name();
    }
    mesh.color_mode = static_cast<ColorMode>(proto.color_mode());

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

Mesh Mesh::from_polyline_pairs(const std::vector<Polyline>& pairs, double scale) {
    if (pairs.empty() || pairs.size() % 2 != 0) return Mesh();

    // Number of open (non-duplicate) points in a polyline
    auto open_count = [](const Polyline& pl) -> size_t {
        size_t n = pl.point_count();
        if (n > 1 && pl.is_closed()) return n - 1;
        return n;
    };

    // Validate: each top/bottom pair must have the same vertex count and at least 3 points
    for (size_t i = 0; i < pairs.size(); i += 2) {
        size_t a = open_count(pairs[i]);
        size_t b = open_count(pairs[i + 1]);
        if (a != b || a < 3) return Mesh();
    }

    // Separate into top / bottom polylines, strip closing vertex, apply scale
    std::vector<Polyline> top_polys, bot_polys;
    top_polys.reserve(pairs.size() / 2);
    bot_polys.reserve(pairs.size() / 2);

    for (size_t i = 0; i < pairs.size(); i += 2) {
        auto make_scaled = [&](const Polyline& src) -> Polyline {
            size_t limit = open_count(src);
            std::vector<Point> pts;
            pts.reserve(limit);
            for (size_t j = 0; j < limit; ++j) {
                Point p = src.get_point(j);
                pts.push_back(Point(p[0] / scale, p[1] / scale, p[2] / scale));
            }
            return Polyline(pts);
        };
        top_polys.push_back(make_scaled(pairs[i]));
        bot_polys.push_back(make_scaled(pairs[i + 1]));
    }

    return loft(top_polys, bot_polys, /*cap=*/true);
}

void Mesh::from_polyline_pairs_vnf(
    const std::vector<Polyline>& pairs,
    std::vector<double>& out_vertices,
    std::vector<double>& out_normals,
    std::vector<int>& out_triangles,
    double scale)
{
    Mesh m = from_polyline_pairs(pairs, scale);
    if (m.is_empty()) return;

    auto face_nrms = m.face_normals();

    for (size_t fk : m.faces()) {
        auto maybe_verts = m.face_vertices(fk);
        if (!maybe_verts || maybe_verts->size() < 3) continue;
        const auto& fverts = *maybe_verts;

        // Collect all vertex positions up front; skip degenerate faces
        std::vector<Point> fpts;
        fpts.reserve(fverts.size());
        bool valid = true;
        for (size_t vk : fverts) {
            auto pos = m.vertex_point(vk);
            if (!pos) { valid = false; break; }
            fpts.push_back(*pos);
        }
        if (!valid) continue;

        Vector nrm(0.0, 0.0, 1.0);
        auto it = face_nrms.find(fk);
        if (it != face_nrms.end()) nrm = it->second;

        // Fan-triangulate and emit flat VNF (each triangle owns its 3 vertex slots)
        for (size_t i = 1; i + 1 < fpts.size(); ++i) {
            for (const Point* p : {&fpts[0], &fpts[i], &fpts[i + 1]}) {
                out_triangles.push_back(static_cast<int>(out_triangles.size()));
                out_vertices.push_back((*p)[0]);
                out_vertices.push_back((*p)[1]);
                out_vertices.push_back((*p)[2]);
                out_normals.push_back(nrm[0]);
                out_normals.push_back(nrm[1]);
                out_normals.push_back(nrm[2]);
            }
        }
    }
}

} // namespace session_cpp
