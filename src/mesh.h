#pragma once
#include "point.h"
#include "vector.h"
#include "color.h"
#include "xform.h"
#include "obb.h"
#include "spatial_bvh.h"
#include "spatial_aabbtree.h"
#include "aabb.h"
#include "json.h"
#include "line.h"
#include "polyline.h"
#include <map>
#include <set>
#include <vector>
#include <string>
#include <optional>
#include <memory>
#include <cmath>
#include <tuple>
#include <functional>
#include <cstdint>

namespace session_cpp {

struct LoftPanel;
struct LoftAdjPair;
struct LoftResult;

/// Which stored colors a mesh renders with
enum class ColorMode : int {
    OBJECTCOLOR = 0, POINTCOLORS = 1, FACECOLORS = 2, NONE = 3
};
inline std::string color_mode_to_string(ColorMode m) {
    switch (m) {
        case ColorMode::POINTCOLORS: return "pointcolors";
        case ColorMode::FACECOLORS:  return "facecolors";
        case ColorMode::NONE:        return "none";
        default:                     return "objectcolor";
    }
}
inline ColorMode color_mode_from_string(const std::string& s) {
    if (s == "pointcolors") return ColorMode::POINTCOLORS;
    if (s == "facecolors")  return ColorMode::FACECOLORS;
    if (s == "none")        return ColorMode::NONE;
    return ColorMode::OBJECTCOLOR;
}

/// Weighting scheme for vertex normals
enum class NormalWeighting {
    Area,
    Angle,
    Uniform
};

/// A vertex's attribute map, allocated only once something is stored in it
class Attributes {
public:
    using Map = std::map<std::string, double>;
    using const_iterator = Map::const_iterator;
    using value_type = Map::value_type;

    Attributes() = default;
    Attributes(const Attributes& o) : m_(o.m_ ? std::make_unique<Map>(*o.m_) : nullptr) {}
    Attributes(Attributes&&) noexcept = default;
    Attributes(const Map& m) { if (!m.empty()) m_ = std::make_unique<Map>(m); }
    Attributes& operator=(const Attributes& o) {
        m_ = o.m_ ? std::make_unique<Map>(*o.m_) : nullptr;
        return *this;
    }
    Attributes& operator=(Attributes&&) noexcept = default;
    Attributes& operator=(const Map& m) {
        m_ = m.empty() ? nullptr : std::make_unique<Map>(m);
        return *this;
    }

    /// The map itself, or a shared empty one
    const Map& map() const {
        static const Map empty;
        return m_ ? *m_ : empty;
    }
    const_iterator begin() const { return map().begin(); }
    const_iterator end() const { return map().end(); }
    const_iterator find(const std::string& k) const { return map().find(k); }
    size_t count(const std::string& k) const { return map().count(k); }
    const double& at(const std::string& k) const { return map().at(k); }
    size_t size() const { return map().size(); }
    bool empty() const { return map().empty(); }

    /// The only mutating entry point, and the only one that can allocate
    double& operator[](const std::string& k) {
        if (!m_) m_ = std::make_unique<Map>();
        return (*m_)[k];
    }
    size_t erase(const std::string& k) {
        if (!m_) return 0;
        size_t n = m_->erase(k);
        if (m_->empty()) m_.reset();
        return n;
    }
    void clear() { m_.reset(); }

    bool operator==(const Attributes& o) const { return map() == o.map(); }
    bool operator!=(const Attributes& o) const { return !(*this == o); }
    bool operator==(const Map& o) const { return map() == o; }
    bool operator!=(const Map& o) const { return !(*this == o); }

private:
    std::unique_ptr<Map> m_;
};

/// nlohmann-json hooks, templated so both json and ordered_json pick them up
template <typename J>
void to_json(J& j, const Attributes& a) { j = a.map(); }
template <typename J>
void from_json(const J& j, Attributes& a) { a = j.template get<Attributes::Map>(); }

/// Vertex position and attributes
struct VertexData {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    Attributes attributes;

    VertexData() = default;
    VertexData(const Point& p) : x(p[0]), y(p[1]), z(p[2]) {}
    bool operator==(const VertexData& other) const {
        return x == other.x && y == other.y && z == other.z && attributes == other.attributes;
    }
    bool operator!=(const VertexData& other) const { return !(*this == other); }

    /// Position as a Point
    Point position() const { return Point(x, y, z); }

    /// Set the position from a Point
    void set_position(const Point& p) {
        x = p[0];
        y = p[1];
        z = p[2];
    }

    /// Vertex color as RGB, 0.5 grey when unset
    std::array<double, 3> color() const {
        return {
            attributes.count("r") ? attributes.at("r") : 0.5,
            attributes.count("g") ? attributes.at("g") : 0.5,
            attributes.count("b") ? attributes.at("b") : 0.5
        };
    }

    /// Set the vertex color
    void set_color(double r, double g, double b) {
        attributes["r"] = r;
        attributes["g"] = g;
        attributes["b"] = b;
    }

    /// Vertex normal if set
    std::optional<std::array<double, 3>> normal() const {
        if (attributes.count("nx") && attributes.count("ny") && attributes.count("nz")) {
            return std::array<double, 3>{
                attributes.at("nx"),
                attributes.at("ny"),
                attributes.at("nz")
            };
        }
        return std::nullopt;
    }

    /// Set the vertex normal
    void set_normal(double nx, double ny, double nz) {
        attributes["nx"] = nx;
        attributes["ny"] = ny;
        attributes["nz"] = nz;
    }
};

/// A halfedge mesh data structure for representing polygonal surfaces
class Mesh {
public:
    std::map<size_t, std::map<size_t, std::optional<size_t>>> halfedge;  ///< Halfedge connectivity
    std::map<size_t, VertexData> vertex;                                  ///< Vertex data
    std::map<size_t, std::vector<size_t>> face;                          ///< Face vertex lists
    std::map<size_t, std::vector<std::vector<size_t>>> face_holes;       ///< Face hole rings
    std::map<size_t, std::map<std::string, double>> facedata;             ///< Face attributes
    std::map<std::pair<size_t, size_t>, std::map<std::string, double>> edgedata;  ///< Edge attributes
    std::map<std::string, double> default_vertex_attributes;              ///< Default vertex attrs
    std::map<std::string, double> default_face_attributes;                ///< Default face attrs
    std::map<std::string, double> default_edge_attributes;                ///< Default edge attrs
    bool has_guid() const { return !_guid.empty(); }
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    /// Clear the guid so a fresh one mints lazily on next read
    void refresh_guid() { _guid.clear(); }
    std::string name = "my_mesh";                                        ///< Mesh name
    ColorMode color_mode = ColorMode::OBJECTCOLOR;                        ///< Active color mode

    void set_pointcolors(std::vector<Color> v) { pointcolors = std::move(v); color_mode = ColorMode::POINTCOLORS; }
    void set_facecolors(std::vector<Color> v) { facecolors = std::move(v); color_mode = ColorMode::FACECOLORS; }
    void set_linecolors(std::vector<Color> v, std::vector<double> w = {}) { linecolors = std::move(v); if (!w.empty()) widths = std::move(w); }
    void set_objectcolor(Color c) { objectcolor = std::move(c); }
    void clear_pointcolors() { pointcolors.clear(); if (color_mode == ColorMode::POINTCOLORS) color_mode = ColorMode::OBJECTCOLOR; }
    void clear_facecolors() { facecolors.clear(); if (color_mode == ColorMode::FACECOLORS) color_mode = ColorMode::OBJECTCOLOR; }
    void clear_linecolors() { linecolors.clear(); widths.clear(); }

    const std::vector<Color>& get_pointcolors() const { return pointcolors; }
    const std::vector<Color>& get_facecolors() const  { return facecolors; }
    const std::vector<Color>& get_linecolors() const  { return linecolors; }
    const std::vector<double>& get_widths() const     { return widths; }
    const Color& get_objectcolor() const              { return objectcolor; }
    const std::map<size_t, std::vector<std::array<size_t, 3>>>& get_triangulation() const { return triangulation; }
    void set_face_triangulation(size_t fk, std::vector<std::array<size_t,3>> tris) { triangulation[fk] = std::move(tris); }
    const auto& get_face_holes() const { return face_holes; }
    void set_face_holes(size_t fkey, std::vector<std::vector<size_t>> rings) { face_holes[fkey] = std::move(rings); }

private:
    mutable std::string _guid;
    std::vector<Color> pointcolors;                                      ///< Vertex colors
    std::vector<Color> facecolors;                                       ///< Face colors
    std::vector<Color> linecolors;                                       ///< Edge colors
    std::vector<double> widths;                                           ///< Edge widths
    Color objectcolor = Color::white();                                  ///< Object color
    size_t max_vertex = 0;                                               ///< Next vertex key
    size_t max_face = 0;                                                 ///< Next face key
    std::map<size_t, std::vector<std::array<size_t, 3>>> triangulation; ///< Cached triangulations

    mutable bool triangle_bvh_built = false;
    mutable std::shared_ptr<SpatialBVH> triangle_bvh;                    ///< BVH over cached triangle AABBs
    mutable std::vector<AABB> triangle_aabbs_cache;                      ///< Per-triangle AABBs
    struct TriangleIndex { uint32_t i0, i1, i2; };
    mutable std::vector<TriangleIndex> triangle_indices_cache;           ///< Triangle vertex indices
    mutable std::vector<std::pair<size_t, size_t>> triangle_face_subidx_cache; ///< (face_idx, sub_idx)
    mutable std::vector<Point> vertices_cache;                           ///< Sequential vertex positions
    mutable std::shared_ptr<SpatialAABBTree> triangle_aabb_tree;         ///< AABB tree over cached triangle AABBs

    /// Every directed edge (u, v) some face ring walks
    std::set<std::pair<size_t, size_t>> directed_face_edges() const;

    /// Face-derived halfedge connectivity, computed without mutating
    std::map<size_t, std::map<size_t, std::optional<size_t>>> compute_halfedges() const;

public:

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════

    Mesh();
    Mesh(const Mesh& other);
    Mesh& operator=(const Mesh& other);
    bool operator==(const Mesh& other) const;
    bool operator!=(const Mesh& other) const;
    ~Mesh();

    /// Mesh from a list of vertices and faces
    static Mesh from_vertices_and_faces(const std::vector<Point>& vertices, const std::vector<std::vector<size_t>>& faces);

    /// Mesh from a list of polygons, merging vertices within precision when given
    static Mesh from_polylines(const std::vector<std::vector<Point>>& polygons, std::optional<double> precision = std::nullopt);
    static Mesh from_polylines(const std::vector<Polyline>& polylines, std::optional<double> precision = std::nullopt);

    /// Planar mesh from a line network, optionally without its outer boundary face
    static Mesh from_lines(const std::vector<Line>& lines, bool delete_boundary_face = false, std::optional<double> precision = std::nullopt);

    /// Mesh from a polygon boundary with optional holes; sort_by_bbox picks the largest polyline as boundary
    static Mesh from_polygon_with_holes(const std::vector<std::vector<Point>>& polylines, bool sort_by_bbox = false);

    /// Loft between two sets of polylines into a mesh volume, capped when cap is true
    static Mesh loft(const std::vector<Polyline>& polylines0, const std::vector<Polyline>& polylines1, bool cap = true, bool fix_collinear = true);

    /// Batch from_polygon_with_holes, parallel when asked
    static std::vector<Mesh> from_polygon_with_holes_many(
        const std::vector<std::vector<std::vector<Point>>>& inputs,
        bool sort_by_bbox = false, bool parallel = true);

    /// Batch loft, parallel when asked
    static std::vector<Mesh> loft_many(
        const std::vector<std::pair<std::vector<Polyline>, std::vector<Polyline>>>& pairs,
        bool cap = true, bool parallel = true, bool fix_collinear = true);

    /// Loft matched top/bottom polygon pairs into one panel each, with matched quad walls and triangle fill
    static LoftResult loft_panels(
        const std::vector<std::vector<Point>>& top_polygons,
        const std::vector<std::vector<Point>>& bot_polygons,
        double merge_precision      = 0.001,
        double edge_gap             = 0.0,
        double edge_match_threshold = 2.0,
        bool   add_caps             = true,
        bool   skip_triangles       = false);

    /// Closed box mesh centered at the origin: 8 vertices, 6 quads
    static Mesh create_box(double x, double y, double z);

    /// Dodecahedron mesh with the given edge length
    static Mesh create_dodecahedron(double edge = 2.0);

    /// Closed mesh from interleaved top/bottom polyline pairs [top0, bot0, ...], coordinates divided by scale
    static Mesh from_polyline_pairs(const std::vector<Polyline>& pairs, double scale = 1.0);

    /// Flat vertex, normal and triangle arrays of a closed mesh from interleaved top/bottom polyline pairs
    static void from_polyline_pairs_vnf(
        const std::vector<Polyline>& pairs,
        std::vector<double>& out_vertices,
        std::vector<double>& out_normals,
        std::vector<int>& out_triangles,
        double scale = 1.0);

    /// Ruled quad mesh by projecting profile onto planes perpendicular to cross_section
    static Mesh reflex_fold(const Polyline& cross_section, const Polyline& profile);

    /// Per-face miter plate contours of a shell: (top_chamfered, bot_chamfered, top_raw, bot_raw, face_normal)
    static std::vector<std::tuple<
        std::vector<Point>, std::vector<Point>,
        std::vector<Point>, std::vector<Point>,
        Vector>>
    miter_contours(const Mesh& shell, double thickness,
                   double chamfer_bot, double chamfer_top, bool flatter,
                   double chamfer_angle_deg = 90.0);

    // ═══════════════════════════════════════════════════════════════════════════
    // Boolean Queries
    // ═══════════════════════════════════════════════════════════════════════════

    /// True when the mesh has no vertices
    bool is_empty() const { return vertex.empty(); }

    /// True when every face has at least three existing vertices
    bool is_valid() const;

    /// True when every face edge has a twin face or a declared hole ring
    bool is_closed() const;

    /// True when the vertex touches a boundary edge
    bool is_vertex_on_boundary(size_t vertex_key) const;

    /// True when the edge has a face on one side only
    bool is_edge_on_boundary(size_t u, size_t v) const;

    /// True when the face has a boundary edge
    bool is_face_on_boundary(size_t face_key) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Attributes
    // ═══════════════════════════════════════════════════════════════════════════

    size_t number_of_vertices() const { return vertex.size(); }
    size_t number_of_faces() const { return face.size(); }
    size_t number_of_edges() const;

    /// Euler characteristic V - E + F
    int euler() const;

    /// Sorted vertex keys
    std::vector<size_t> vertices() const;

    /// Sorted face keys
    std::vector<size_t> faces() const;

    /// Undirected edges as sorted (u, v) pairs
    std::vector<std::pair<size_t, size_t>> edges() const;

    /// Vertices and faces with sequential 0-based indices
    std::pair<std::vector<Point>, std::vector<std::vector<size_t>>> to_vertices_and_faces() const;

    /// Sparse vertex key to sequential index
    std::map<size_t, size_t> vertex_index() const;

    /// Boundary (true) or interior (false) edges
    std::vector<std::pair<size_t, size_t>> naked_edges(bool boundary = true) const;

    /// Boundary (true) or interior (false) vertices
    std::vector<size_t> naked_vertices(bool boundary = true) const;

    /// Boundary (true) or interior (false) faces
    std::vector<size_t> naked_faces(bool boundary = true) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Vertex and Face Operations
    // ═══════════════════════════════════════════════════════════════════════════

    /// Add a vertex, with an explicit key when given; returns the key
    size_t add_vertex(const Point& position, std::optional<size_t> vkey = std::nullopt);

    /// Add a face, with an explicit key when given; returns the key or nullopt when invalid
    std::optional<size_t> add_face(const std::vector<size_t>& vertices, std::optional<size_t> fkey = std::nullopt);

    /// Remove a vertex and every face that uses it
    void remove_vertex(size_t vkey);

    /// Remove a face and its orphaned halfedges
    void remove_face(size_t fkey);

    /// Remove an edge, its adjacent faces and its halfedges
    void remove_edge(size_t u, size_t v);

    /// Reverse the winding of one face in place
    void flip_face(size_t fkey);

    /// Reverse the winding of every face
    void flip();

    /// Clear all mesh data
    void clear();

    /// Copy where every face owns its own vertices
    Mesh unweld() const;

    /// Copy with vertices closer than tolerance merged; degenerate faces are dropped
    Mesh weld(double tolerance = 0.001) const;

    /// Unify face winding by BFS; returns true when any face was flipped
    bool unify_winding();

    /// Flip a closed mesh whose normals point inward; returns true when flipped
    bool orient_outward();

    /// Recreate halfedge from vertex and face alone
    void rebuild_halfedges();

    /// Build the lazy halfedge map when it is empty and faces exist
    void ensure_halfedges();

    // ═══════════════════════════════════════════════════════════════════════════
    // Connectivity Queries
    // ═══════════════════════════════════════════════════════════════════════════

    /// Edges sharing a vertex with (u, v), excluding (u, v) and (v, u)
    std::optional<std::vector<std::pair<size_t, size_t>>> edge_edges(size_t u, size_t v) const;

    /// Faces on each side of an edge
    std::optional<std::vector<size_t>> edge_faces(size_t u, size_t v) const;

    /// Every directed face edge to its face key, in one face walk
    std::map<std::pair<size_t, size_t>, size_t> edge_face_map() const;

    /// The edge as a Line
    std::optional<Line> edge_line(size_t u, size_t v) const;

    /// Edges of a face as (vi, vi+1) pairs
    std::optional<std::vector<std::pair<size_t, size_t>>> face_edges(size_t face_key) const;

    /// Faces sharing an edge with a face
    std::optional<std::vector<size_t>> face_faces(size_t face_key) const;

    /// Points of a face
    std::optional<std::vector<Point>> face_points(size_t face_key) const;

    /// The face as a Polyline
    std::optional<Polyline> face_polyline(size_t face_key) const;

    /// Vertex keys of a face
    std::optional<std::vector<size_t>> face_vertices(size_t face_key) const;

    /// Edges incident to a vertex as (vertex_key, neighbor) pairs
    std::optional<std::vector<std::pair<size_t, size_t>>> vertex_edges(size_t vertex_key) const;

    /// Faces incident to a vertex
    std::optional<std::vector<size_t>> vertex_faces(size_t vertex_key) const;

    /// Position of a vertex
    std::optional<Point> vertex_point(size_t vertex_key) const;

    /// Neighboring vertices of a vertex
    std::optional<std::vector<size_t>> vertex_vertices(size_t vertex_key) const;

    /// Neighbors of a vertex, in face-cycle order when ordered is true
    std::optional<std::vector<size_t>> vertex_neighbors(size_t vertex_key, bool ordered = false) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Boundary
    // ═══════════════════════════════════════════════════════════════════════════

    std::vector<size_t> vertices_on_boundary() const;
    std::vector<std::pair<size_t, size_t>> edges_on_boundary() const;
    std::vector<size_t> faces_on_boundary() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Halfedge Navigation
    // ═══════════════════════════════════════════════════════════════════════════

    std::optional<size_t> halfedge_face(std::pair<size_t, size_t> edge) const;
    std::optional<std::pair<size_t, size_t>> halfedge_after(std::pair<size_t, size_t> edge) const;
    std::optional<std::pair<size_t, size_t>> halfedge_before(std::pair<size_t, size_t> edge) const;
    std::vector<std::pair<size_t, size_t>> halfedge_loop(std::pair<size_t, size_t> edge) const;
    std::vector<std::pair<size_t, size_t>> halfedge_strip(std::pair<size_t, size_t> edge) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Sampling
    // ═══════════════════════════════════════════════════════════════════════════

    /// seed 0 takes the first keys, any other seed drives a deterministic LCG
    std::vector<size_t> vertex_sample(size_t size, uint32_t seed = 0) const;
    std::vector<std::pair<size_t, size_t>> edge_sample(size_t size, uint32_t seed = 0) const;
    std::vector<size_t> face_sample(size_t size, uint32_t seed = 0) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Aliases
    // ═══════════════════════════════════════════════════════════════════════════

    std::optional<Point> face_center(size_t face_key) const;
    std::optional<Polyline> face_polygon(size_t face_key) const;

    /// Every face as a closed outline in face-key order; faces under three vertices are skipped
    std::vector<Polyline> face_outlines() const;
    void flip_cycles();

    // ═══════════════════════════════════════════════════════════════════════════
    // Attribute API
    // ═══════════════════════════════════════════════════════════════════════════

    void update_default_vertex_attributes(const std::vector<std::pair<std::string, double>>& attrs);
    void update_default_face_attributes(const std::vector<std::pair<std::string, double>>& attrs);
    void update_default_edge_attributes(const std::vector<std::pair<std::string, double>>& attrs);

    std::optional<double> vertex_attribute(size_t key, const std::string& name) const;
    void set_vertex_attribute(size_t key, const std::string& name, double value);
    std::optional<double> face_attribute(size_t fkey, const std::string& name) const;
    void set_face_attribute(size_t fkey, const std::string& name, double value);
    std::optional<double> edge_attribute(std::pair<size_t, size_t> edge, const std::string& name) const;
    void set_edge_attribute(std::pair<size_t, size_t> edge, const std::string& name, double value);

    /// keys nullptr means all; the result holds nullopt for missing values
    std::vector<std::optional<double>> vertices_attribute(const std::string& name, const std::vector<size_t>* keys = nullptr) const;
    void set_vertices_attribute(const std::string& name, double value, const std::vector<size_t>* keys = nullptr);
    std::vector<std::optional<double>> faces_attribute(const std::string& name, const std::vector<size_t>* keys = nullptr) const;
    void set_faces_attribute(const std::string& name, double value, const std::vector<size_t>* keys = nullptr);
    std::vector<std::optional<double>> edges_attribute(const std::string& name, const std::vector<std::pair<size_t, size_t>>* keys = nullptr) const;
    void set_edges_attribute(const std::string& name, double value, const std::vector<std::pair<size_t, size_t>>* keys = nullptr);

    std::vector<size_t> vertices_where(const std::vector<std::pair<std::string, double>>& conditions) const;
    std::vector<size_t> faces_where(const std::vector<std::pair<std::string, double>>& conditions) const;
    std::vector<std::pair<size_t, size_t>> edges_where(const std::vector<std::pair<std::string, double>>& conditions) const;

    std::vector<size_t> vertices_where_predicate(const std::function<bool(size_t, const std::map<std::string, double>&)>& pred) const;
    std::vector<size_t> faces_where_predicate(const std::function<bool(size_t, const std::map<std::string, double>&)>& pred) const;
    std::vector<std::pair<size_t, size_t>> edges_where_predicate(const std::function<bool(std::pair<size_t, size_t>, const std::map<std::string, double>&)>& pred) const;

    /// Face normal from the first three vertices; unitized false keeps twice the first-triangle area as length
    std::optional<Vector> face_normal_unitized(size_t face_key, bool unitized) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometric Properties
    // ═══════════════════════════════════════════════════════════════════════════

    /// Total surface area of all faces
    double area() const;

    /// Average of all vertex positions
    Point centroid() const;

    /// Dihedral angle in degrees between the two faces sharing edge (u, v), nullopt on a boundary edge
    std::optional<double> dihedral_angle(size_t u, size_t v) const;

    /// Dihedral angles of all interior edges as (angles, arcs, points); arcs and label points are built when asked
    std::tuple<std::map<std::pair<size_t,size_t>,double>, std::vector<Polyline>, std::vector<Point>>
    dihedral_angles(double scale = 0.3, bool with_arcs = true, bool with_points = true) const;

    /// Area of a face
    std::optional<double> face_area(size_t face_key) const;

    /// Average of a face's vertex positions
    std::optional<Point> face_centroid(size_t face_key) const;

    /// Unit normal of a face
    std::optional<Vector> face_normal(size_t face_key) const;

    /// Unit normals of all faces
    std::map<size_t, Vector> face_normals() const;

    /// Angle at a vertex inside a face
    std::optional<double> vertex_angle_in_face(size_t vertex_key, size_t face_key) const;

    /// Area-weighted vertex normal
    std::optional<Vector> vertex_normal(size_t vertex_key) const;

    /// Vertex normal with the given weighting
    std::optional<Vector> vertex_normal_weighted(size_t vertex_key, NormalWeighting weighting) const;

    /// Area-weighted normals of all vertices
    std::map<size_t, Vector> vertex_normals() const;

    /// Normals of all vertices with the given weighting
    std::map<size_t, Vector> vertex_normals_weighted(NormalWeighting weighting) const;

    /// Enclosed volume of a closed mesh
    double volume() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Triangle BVH
    // ═══════════════════════════════════════════════════════════════════════════

    /// Build and cache the BVH over the triangulated faces
    void build_triangle_bvh(bool force = false) const;

    /// Candidate triangle ids along a ray, from the cached BVH
    bool triangle_bvh_ray_cast(const Point& origin, const Vector& direction, std::vector<int>& candidate_ids, bool find_all = false) const;

    /// Face index, sub-triangle index and corners of a cached triangle id
    bool get_triangle_by_id(int tri_id, size_t& face_idx, size_t& sub_idx, Point& v0, Point& v1, Point& v2) const;

    /// Drop the cached BVH, AABB tree and triangle data
    void clear_triangle_bvh() const;

    /// Build and cache the AABB tree over the triangulated faces
    void build_triangle_aabb_tree(bool force = false) const;

    const SpatialBVH* get_cached_bvh() const { return triangle_bvh.get(); }
    const SpatialAABBTree* get_cached_aabb_tree() const { return triangle_aabb_tree.get(); }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════

    bool transform(const Xform& xf);
    Mesh transformed(const Xform& xf) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════

    nlohmann::ordered_json jsondump() const;
    static Mesh jsonload(const nlohmann::json& data);
    std::string file_json_dumps() const;
    static Mesh file_json_loads(const std::string& json_string);
    void file_json_dump(const std::string& filename) const;
    static Mesh file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════

    std::string pb_dumps() const;
    static Mesh pb_loads(const std::string& data);
    void pb_dump(const std::string& filename) const;
    static Mesh pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String Representation
    // ═══════════════════════════════════════════════════════════════════════════

    std::string str() const;
    std::string repr() const;
    friend std::ostream& operator<<(std::ostream& os, const Mesh& mesh);
};

// ═══════════════════════════════════════════════════════════════════════════
// Loft
// ═══════════════════════════════════════════════════════════════════════════

enum class LoftFaceRole { TopCap, BotCap, QuadWall, TriWall };

struct LoftWallFace {
    size_t face_key = 0;    ///< local panel mesh face key
    size_t face_index = 0;  ///< 0-based position of face_key in panel mesh.face
    bool   is_quad = false;
    size_t top_v0 = 0, top_v1 = 0; ///< original top-mesh vertex keys
    size_t bot_v0 = 0, bot_v1 = 0; ///< original bot-mesh vertex keys (valid if is_quad)
};

struct LoftPanel {
    Mesh   mesh;
    std::optional<size_t> top_face_key;         ///< local key of top cap face
    std::optional<size_t> bot_face_key;         ///< local key of bot cap face
    std::vector<LoftWallFace>      wall_faces;
    std::map<size_t,LoftFaceRole>  face_roles;  ///< face_key to role for every face in mesh
    std::map<size_t,size_t>    orig_top_to_local; ///< original top vertex key to local key
    std::map<size_t,size_t>    orig_bot_to_local; ///< original bot vertex key to local key
    std::vector<size_t>        top_vertices;
    std::vector<size_t>        bot_vertices;
};

struct LoftAdjPair {
    size_t pi, wi;   ///< panel index + wall_faces index for side i
    size_t pj, wj;   ///< panel index + wall_faces index for side j
};

struct LoftResult {
    std::vector<LoftPanel>   panels;
    std::vector<LoftAdjPair> adjacency;
    Mesh top_mesh;   ///< top polygons of the matched panels, one face per panel
    Mesh bot_mesh;   ///< bot polygons of the matched panels, one face per panel
};

} // namespace session_cpp
