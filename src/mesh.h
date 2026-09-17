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

/// Which stored colors a mesh renders with.
enum class ColorMode : int { OBJECTCOLOR = 0, POINTCOLORS = 1, FACECOLORS = 2, NONE = 3 };

/// Return the lowercase name of a color mode.
inline std::string color_mode_to_string(ColorMode m) {

    switch (m) {
    case ColorMode::POINTCOLORS:
        return "pointcolors";
    case ColorMode::FACECOLORS:
        return "facecolors";
    case ColorMode::NONE:
        return "none";
    default:
        return "objectcolor";
    }
}

/// Return the color mode named s, objectcolor when unknown.
inline ColorMode color_mode_from_string(const std::string& s) {

    if (s == "pointcolors")
        return ColorMode::POINTCOLORS;

    if (s == "facecolors")
        return ColorMode::FACECOLORS;

    if (s == "none")
        return ColorMode::NONE;

    return ColorMode::OBJECTCOLOR;
}

/// Weighting scheme for vertex normals.
enum class NormalWeighting { Area, Angle, Uniform };

/// A vertex's attribute map, allocated only once something is stored in it.
class Attributes {
public:
    using Map = std::map<std::string, double>;
    using const_iterator = Map::const_iterator;
    using value_type = Map::value_type;

    /// Construct an empty map.
    Attributes() = default;

    /// Copy the map when the other holds one.
    Attributes(const Attributes& o) : m_(o.m_ ? std::make_unique<Map>(*o.m_) : nullptr) {}

    /// Move the map.
    Attributes(Attributes&&) noexcept = default;

    /// Construct from a map, allocating only when it is not empty.
    Attributes(const Map& m) {
        if (!m.empty())
            m_ = std::make_unique<Map>(m);
    }

    /// Copy-assign the map when the other holds one.
    Attributes& operator=(const Attributes& o) {
        m_ = o.m_ ? std::make_unique<Map>(*o.m_) : nullptr;

        return *this;
    }

    /// Move-assign the map.
    Attributes& operator=(Attributes&&) noexcept = default;

    /// Assign from a map, allocating only when it is not empty.
    Attributes& operator=(const Map& m) {
        m_ = m.empty() ? nullptr : std::make_unique<Map>(m);

        return *this;
    }

    /// Return the map itself, or a shared empty one.
    const Map& map() const {
        static const Map empty;

        return m_ ? *m_ : empty;
    }

    /// Return the first entry.
    const_iterator begin() const { return map().begin(); }

    /// Return one past the last entry.
    const_iterator end() const { return map().end(); }

    /// Return the entry named k, or end().
    const_iterator find(const std::string& k) const { return map().find(k); }

    /// Return 1 when k is stored, else 0.
    size_t count(const std::string& k) const { return map().count(k); }

    /// Return the value named k; throws when missing.
    const double& at(const std::string& k) const { return map().at(k); }

    /// Return the number of entries.
    size_t size() const { return map().size(); }

    /// Return whether nothing is stored.
    bool empty() const { return map().empty(); }

    /// Return the mutable value named k; the only mutating entry point, and the only one that can allocate.
    double& operator[](const std::string& k) {
        if (!m_)
            m_ = std::make_unique<Map>();

        return (*m_)[k];
    }

    /// Remove k and free the map when it becomes empty; returns the number removed.
    size_t erase(const std::string& k) {

        if (!m_)
            return 0;

        size_t n = m_->erase(k);

        if (m_->empty())
            m_.reset();

        return n;
    }

    /// Free the map.
    void clear() { m_.reset(); }

    /// Compare the stored maps.
    bool operator==(const Attributes& o) const { return map() == o.map(); }

    /// Compare the stored maps.
    bool operator!=(const Attributes& o) const { return !(*this == o); }

    /// Compare the stored map with a map.
    bool operator==(const Map& o) const { return map() == o; }

    /// Compare the stored map with a map.
    bool operator!=(const Map& o) const { return !(*this == o); }

private:
    std::unique_ptr<Map> m_;
};

/// Write the map to json; templated so both json and ordered_json pick it up.
template <typename J> void to_json(J& j, const Attributes& a) {
    j = a.map();
}

/// Read the map from json; templated so both json and ordered_json pick it up.
template <typename J> void from_json(const J& j, Attributes& a) {
    a = j.template get<Attributes::Map>();
}

/// Vertex position and attributes.
struct VertexData {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    Attributes attributes;

    /// Construct at the origin.
    VertexData() = default;

    /// Construct at a point.
    VertexData(const Point& p) : x(p[0]), y(p[1]), z(p[2]) {}

    /// Compare position and attributes exactly.
    bool operator==(const VertexData& other) const {
        return x == other.x && y == other.y && z == other.z && attributes == other.attributes;
    }

    /// Compare position and attributes exactly.
    bool operator!=(const VertexData& other) const { return !(*this == other); }

    /// Return the position as a Point.
    Point position() const { return Point(x, y, z); }

    /// Set the position from a Point.
    void set_position(const Point& p) {
        x = p[0];
        y = p[1];
        z = p[2];
    }

    /// Return the vertex color as RGB, 0.5 grey when unset.
    std::array<double, 3> color() const {

        return {
            attributes.count("r") ? attributes.at("r") : 0.5,
            attributes.count("g") ? attributes.at("g") : 0.5,
            attributes.count("b") ? attributes.at("b") : 0.5
        };
    }

    /// Set the vertex color.
    void set_color(double r, double g, double b) {
        attributes["r"] = r;
        attributes["g"] = g;
        attributes["b"] = b;
    }

    /// Return the vertex normal when set.
    std::optional<std::array<double, 3>> normal() const {
        if (attributes.count("nx") && attributes.count("ny") && attributes.count("nz"))
            return std::array<double, 3>{attributes.at("nx"), attributes.at("ny"), attributes.at("nz")};

        return std::nullopt;
    }

    /// Set the vertex normal.
    void set_normal(double nx, double ny, double nz) {
        attributes["nx"] = nx;
        attributes["ny"] = ny;
        attributes["nz"] = nz;
    }
};

/// A halfedge mesh data structure for representing polygonal surfaces.
class Mesh {
public:
    std::map<size_t, std::map<size_t, std::optional<size_t>>> halfedge;          // Halfedge connectivity.
    std::map<size_t, VertexData> vertex;                                         // Vertex data.
    std::map<size_t, std::vector<size_t>> face;                                  // Face vertex lists.
    std::map<size_t, std::vector<std::vector<size_t>>> face_holes;               // Face hole rings.
    std::map<size_t, std::map<std::string, double>> facedata;                    // Face attributes.
    std::map<std::pair<size_t, size_t>, std::map<std::string, double>> edgedata; // Edge attributes.
    std::map<std::string, double> default_vertex_attributes;                     // Default vertex attrs.
    std::map<std::string, double> default_face_attributes;                       // Default face attrs.
    std::map<std::string, double> default_edge_attributes;                       // Default edge attrs.

    /// Return whether the lazy guid has been created.
    bool has_guid() const { return !_guid.empty(); }

    /// Return the guid, creating it on first access.
    const std::string& guid() const {
        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return the mutable guid, creating it on first access.
    std::string& guid() {
        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Clear the guid so a fresh one mints lazily on the next read.
    void refresh_guid() { _guid.clear(); }
    std::string name = "my_mesh";                  // Mesh name.
    ColorMode color_mode = ColorMode::OBJECTCOLOR; // Active color mode.

    /// Store vertex colors and render with them.
    void set_pointcolors(std::vector<Color> v) {
        pointcolors = std::move(v);
        color_mode = ColorMode::POINTCOLORS;
    }

    /// Store face colors and render with them.
    void set_facecolors(std::vector<Color> v) {
        facecolors = std::move(v);
        color_mode = ColorMode::FACECOLORS;
    }

    /// Store edge colors and, when given, edge widths.
    void set_linecolors(std::vector<Color> v, std::vector<double> w = {}) {
        linecolors = std::move(v);

        if (!w.empty())
            widths = std::move(w);
    }

    /// Store the object color.
    void set_objectcolor(Color c) { objectcolor = std::move(c); }

    /// Drop vertex colors, falling back to the object color when they were active.
    void clear_pointcolors() {
        pointcolors.clear();

        if (color_mode == ColorMode::POINTCOLORS)
            color_mode = ColorMode::OBJECTCOLOR;
    }

    /// Drop face colors, falling back to the object color when they were active.
    void clear_facecolors() {
        facecolors.clear();

        if (color_mode == ColorMode::FACECOLORS)
            color_mode = ColorMode::OBJECTCOLOR;
    }

    /// Drop edge colors and widths.
    void clear_linecolors() {
        linecolors.clear();
        widths.clear();
    }

    /// Return the vertex colors.
    const std::vector<Color>& get_pointcolors() const { return pointcolors; }

    /// Return the face colors.
    const std::vector<Color>& get_facecolors() const { return facecolors; }

    /// Return the edge colors.
    const std::vector<Color>& get_linecolors() const { return linecolors; }

    /// Return the edge widths.
    const std::vector<double>& get_widths() const { return widths; }

    /// Return the object color.
    const Color& get_objectcolor() const { return objectcolor; }

    /// Return the cached triangulation per face.
    const std::map<size_t, std::vector<std::array<size_t, 3>>>& get_triangulation() const { return triangulation; }

    /// Cache the triangles of face fk.
    void set_face_triangulation(size_t fk, std::vector<std::array<size_t, 3>> tris) {
        triangulation[fk] = std::move(tris);
    }

    /// Return the hole rings per face.
    const std::map<size_t, std::vector<std::vector<size_t>>>& get_face_holes() const { return face_holes; }

    /// Store the hole rings of face fkey.
    void set_face_holes(size_t fkey, std::vector<std::vector<size_t>> rings) { face_holes[fkey] = std::move(rings); }

private:
    mutable std::string _guid;
    std::vector<Color> pointcolors;                                     // Vertex colors.
    std::vector<Color> facecolors;                                      // Face colors.
    std::vector<Color> linecolors;                                      // Edge colors.
    std::vector<double> widths;                                         // Edge widths.
    Color objectcolor = Color::white();                                 // Object color.
    size_t max_vertex = 0;                                              // Next vertex key.
    size_t max_face = 0;                                                // Next face key.
    std::map<size_t, std::vector<std::array<size_t, 3>>> triangulation; // Cached triangulations.

    mutable bool triangle_bvh_built = false;
    mutable std::shared_ptr<SpatialBVH> triangle_bvh; // BVH over cached triangle AABBs.
    mutable std::vector<AABB> triangle_aabbs_cache;   // Per-triangle AABBs.
    struct TriangleIndex {
        uint32_t i0;
        uint32_t i1;
        uint32_t i2;
    };
    mutable std::vector<TriangleIndex> triangle_indices_cache; // Triangle vertex indices.
    mutable std::vector<std::pair<size_t, size_t>>
        triangle_face_subidx_cache;                              // Face index and sub-triangle index per triangle.
    mutable std::vector<Point> vertices_cache;                   // Sequential vertex positions.
    mutable std::shared_ptr<SpatialAABBTree> triangle_aabb_tree; // AABB tree over cached triangle AABBs.

    /// Return every directed edge (u, v) some face ring walks.
    std::set<std::pair<size_t, size_t>> directed_face_edges() const;

    /// Return the face-derived halfedge connectivity, computed without mutating.
    std::map<size_t, std::map<size_t, std::optional<size_t>>> compute_halfedges() const;

public:
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════

    /// Construct an empty mesh.
    Mesh();

    /// Copy with a new guid and the same data.
    Mesh(const Mesh& other);

    /// Copy-assign with a new guid and the same data.
    Mesh& operator=(const Mesh& other);

    /// Compare vertices, faces, attributes and colors; guid ignored.
    bool operator==(const Mesh& other) const;

    /// Compare vertices, faces, attributes and colors; guid ignored.
    bool operator!=(const Mesh& other) const;

    /// Destroy the mesh.
    ~Mesh();

    /// Construct from a list of vertices and faces.
    static Mesh from_vertices_and_faces(
        const std::vector<Point>& vertices,
        const std::vector<std::vector<size_t>>& faces
    );

    /// Construct from a list of polygons, merging vertices within precision when given.
    static Mesh from_polylines(
        const std::vector<std::vector<Point>>& polygons,
        std::optional<double> precision = std::nullopt
    );

    /// Construct from a list of polygons, merging vertices within precision when given.
    static Mesh from_polylines(const std::vector<Polyline>& polylines, std::optional<double> precision = std::nullopt);

    /// Construct a planar mesh from a line network, optionally without its outer boundary face.
    static Mesh from_lines(
        const std::vector<Line>& lines,
        bool delete_boundary_face = false,
        std::optional<double> precision = std::nullopt
    );

    /// Construct from a polygon boundary with optional holes; sort_by_bbox picks the largest polyline as boundary.
    static Mesh from_polygon_with_holes(const std::vector<std::vector<Point>>& polylines, bool sort_by_bbox = false);

    /// Construct a loft between two sets of polylines into a mesh volume, capped when cap is true.
    static Mesh loft(
        const std::vector<Polyline>& polylines0,
        const std::vector<Polyline>& polylines1,
        bool cap = true,
        bool fix_collinear = true
    );

    /// Construct a batch of from_polygon_with_holes, parallel when asked.
    static std::vector<Mesh> from_polygon_with_holes_many(
        const std::vector<std::vector<std::vector<Point>>>& inputs,
        bool sort_by_bbox = false,
        bool parallel = true
    );

    /// Construct a batch of loft, parallel when asked.
    static std::vector<Mesh> loft_many(
        const std::vector<std::pair<std::vector<Polyline>, std::vector<Polyline>>>& pairs,
        bool cap = true,
        bool parallel = true,
        bool fix_collinear = true
    );

    /// Construct a loft of matched top/bottom polygon pairs into one panel each, with matched quad walls and triangle fill.
    static LoftResult loft_panels(
        const std::vector<std::vector<Point>>& top_polygons,
        const std::vector<std::vector<Point>>& bot_polygons,
        double merge_precision = 0.001,
        double edge_gap = 0.0,
        double edge_match_threshold = 2.0,
        bool add_caps = true,
        bool skip_triangles = false
    );

    /// Construct a closed box centered at the origin: 8 vertices, 6 quads.
    static Mesh create_box(double x, double y, double z);

    /// Construct a dodecahedron with the given edge length.
    static Mesh create_dodecahedron(double edge = 2.0);

    /// Construct a closed mesh from interleaved top/bottom polyline pairs [top0, bot0, ...], coordinates divided by scale.
    static Mesh from_polyline_pairs(const std::vector<Polyline>& pairs, double scale = 1.0);

    /// Write the flat vertex, normal and triangle arrays of a closed mesh from interleaved top/bottom polyline pairs.
    static void from_polyline_pairs_vnf(
        const std::vector<Polyline>& pairs,
        std::vector<double>& out_vertices,
        std::vector<double>& out_normals,
        std::vector<int>& out_triangles,
        double scale = 1.0
    );

    /// Construct a ruled quad mesh by projecting profile onto planes perpendicular to cross_section.
    static Mesh reflex_fold(const Polyline& cross_section, const Polyline& profile);

    /// Compute the per-face miter plate contours of a shell: (top_chamfered, bot_chamfered, top_raw, bot_raw, face_normal).
    static std::vector<
        std::tuple<std::vector<Point>, std::vector<Point>, std::vector<Point>, std::vector<Point>, Vector>>
    miter_contours(
        const Mesh& shell,
        double thickness,
        double chamfer_bot,
        double chamfer_top,
        bool flatter,
        double chamfer_angle_deg = 90.0
    );

    // ═══════════════════════════════════════════════════════════════════════════
    // Boolean Queries
    // ═══════════════════════════════════════════════════════════════════════════

    /// Return whether the mesh has no vertices.
    bool is_empty() const { return vertex.empty(); }

    /// Return whether every face has at least three existing vertices.
    bool is_valid() const;

    /// Return whether every face edge has a twin face or a declared hole ring.
    bool is_closed() const;

    /// Return whether the vertex touches a boundary edge.
    bool is_vertex_on_boundary(size_t vertex_key) const;

    /// Return whether the edge has a face on one side only.
    bool is_edge_on_boundary(size_t u, size_t v) const;

    /// Return whether the face has a boundary edge.
    bool is_face_on_boundary(size_t face_key) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Attributes
    // ═══════════════════════════════════════════════════════════════════════════

    /// Return the vertex count.
    size_t number_of_vertices() const { return vertex.size(); }

    /// Return the face count.
    size_t number_of_faces() const { return face.size(); }

    /// Return the undirected edge count.
    size_t number_of_edges() const;

    /// Return the Euler characteristic V - E + F.
    int euler() const;

    /// Return the sorted vertex keys.
    std::vector<size_t> vertices() const;

    /// Return the sorted face keys.
    std::vector<size_t> faces() const;

    /// Return the undirected edges as sorted (u, v) pairs.
    std::vector<std::pair<size_t, size_t>> edges() const;

    /// Return the vertices and faces with sequential 0-based indices.
    std::pair<std::vector<Point>, std::vector<std::vector<size_t>>> to_vertices_and_faces() const;

    /// Return the map from sparse vertex key to sequential index.
    std::map<size_t, size_t> vertex_index() const;

    /// Return the boundary (true) or interior (false) edges.
    std::vector<std::pair<size_t, size_t>> naked_edges(bool boundary = true) const;

    /// Return the boundary (true) or interior (false) vertices.
    std::vector<size_t> naked_vertices(bool boundary = true) const;

    /// Return the boundary (true) or interior (false) faces.
    std::vector<size_t> naked_faces(bool boundary = true) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Vertex and Face Operations
    // ═══════════════════════════════════════════════════════════════════════════

    /// Add a vertex, with an explicit key when given; returns the key.
    size_t add_vertex(const Point& position, std::optional<size_t> vkey = std::nullopt);

    /// Add a face, with an explicit key when given; returns the key or nullopt when invalid.
    std::optional<size_t> add_face(const std::vector<size_t>& vertices, std::optional<size_t> fkey = std::nullopt);

    /// Remove a vertex and every face that uses it.
    void remove_vertex(size_t vkey);

    /// Remove a face and its orphaned halfedges.
    void remove_face(size_t fkey);

    /// Remove an edge, its adjacent faces and its halfedges.
    void remove_edge(size_t u, size_t v);

    /// Reverse the winding of one face in place.
    void flip_face(size_t fkey);

    /// Reverse the winding of every face.
    void flip();

    /// Clear all mesh data.
    void clear();

    /// Copy where every face owns its own vertices.
    Mesh unweld() const;

    /// Copy with vertices closer than tolerance merged; degenerate faces are dropped.
    Mesh weld(double tolerance = 0.001) const;

    /// Unify face winding by BFS; returns true when any face was flipped.
    bool unify_winding();

    /// Flip a closed mesh whose normals point inward; returns true when flipped.
    bool orient_outward();

    /// Recreate halfedge from vertex and face alone.
    void rebuild_halfedges();

    /// Build the lazy halfedge map when it is empty and faces exist.
    void ensure_halfedges();

    // ═══════════════════════════════════════════════════════════════════════════
    // Connectivity Queries
    // ═══════════════════════════════════════════════════════════════════════════

    /// Return the edges sharing a vertex with (u, v), excluding (u, v) and (v, u).
    std::optional<std::vector<std::pair<size_t, size_t>>> edge_edges(size_t u, size_t v) const;

    /// Return the faces on each side of an edge.
    std::optional<std::vector<size_t>> edge_faces(size_t u, size_t v) const;

    /// Return every directed face edge mapped to its face key, in one face walk.
    std::map<std::pair<size_t, size_t>, size_t> edge_face_map() const;

    /// Return the edge as a Line.
    std::optional<Line> edge_line(size_t u, size_t v) const;

    /// Return the edges of a face as (vi, vi+1) pairs.
    std::optional<std::vector<std::pair<size_t, size_t>>> face_edges(size_t face_key) const;

    /// Return the faces sharing an edge with a face.
    std::optional<std::vector<size_t>> face_faces(size_t face_key) const;

    /// Return the points of a face.
    std::optional<std::vector<Point>> face_points(size_t face_key) const;

    /// Return the face as a Polyline.
    std::optional<Polyline> face_polyline(size_t face_key) const;

    /// Return the vertex keys of a face.
    std::optional<std::vector<size_t>> face_vertices(size_t face_key) const;

    /// Return the edges incident to a vertex as (vertex_key, neighbor) pairs.
    std::optional<std::vector<std::pair<size_t, size_t>>> vertex_edges(size_t vertex_key) const;

    /// Return the faces incident to a vertex.
    std::optional<std::vector<size_t>> vertex_faces(size_t vertex_key) const;

    /// Return the position of a vertex.
    std::optional<Point> vertex_point(size_t vertex_key) const;

    /// Return the neighboring vertices of a vertex.
    std::optional<std::vector<size_t>> vertex_vertices(size_t vertex_key) const;

    /// Return the neighbors of a vertex, in face-cycle order when ordered is true.
    std::optional<std::vector<size_t>> vertex_neighbors(size_t vertex_key, bool ordered = false) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Boundary
    // ═══════════════════════════════════════════════════════════════════════════

    /// Return the vertices touching a boundary edge.
    std::vector<size_t> vertices_on_boundary() const;

    /// Return the edges with a face on one side only.
    std::vector<std::pair<size_t, size_t>> edges_on_boundary() const;

    /// Return the faces with a boundary edge.
    std::vector<size_t> faces_on_boundary() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Halfedge Navigation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Return the face of a directed edge, nullopt when unknown or on the boundary.
    std::optional<size_t> halfedge_face(std::pair<size_t, size_t> edge) const;

    /// Return the next directed edge around the face of edge.
    std::optional<std::pair<size_t, size_t>> halfedge_after(std::pair<size_t, size_t> edge) const;

    /// Return the previous directed edge around the face of edge.
    std::optional<std::pair<size_t, size_t>> halfedge_before(std::pair<size_t, size_t> edge) const;

    /// Return the directed edges around the face of edge, starting at edge.
    std::vector<std::pair<size_t, size_t>> halfedge_loop(std::pair<size_t, size_t> edge) const;

    /// Return the directed edges straight across quads from edge until a boundary or a non-quad.
    std::vector<std::pair<size_t, size_t>> halfedge_strip(std::pair<size_t, size_t> edge) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Sampling
    // ═══════════════════════════════════════════════════════════════════════════

    /// Return size vertex keys; seed 0 takes the first keys, any other seed drives a deterministic LCG.
    std::vector<size_t> vertex_sample(size_t size, uint32_t seed = 0) const;

    /// Return size edges; seed 0 takes the first edges, any other seed drives a deterministic LCG.
    std::vector<std::pair<size_t, size_t>> edge_sample(size_t size, uint32_t seed = 0) const;

    /// Return size face keys; seed 0 takes the first keys, any other seed drives a deterministic LCG.
    std::vector<size_t> face_sample(size_t size, uint32_t seed = 0) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Aliases
    // ═══════════════════════════════════════════════════════════════════════════

    /// Return the average of a face's vertex positions.
    std::optional<Point> face_center(size_t face_key) const;

    /// Return the face as a Polyline.
    std::optional<Polyline> face_polygon(size_t face_key) const;

    /// Return every face as a closed outline in face-key order; faces under three vertices are skipped.
    std::vector<Polyline> face_outlines() const;

    /// Reverse the winding of every face.
    void flip_cycles();

    // ═══════════════════════════════════════════════════════════════════════════
    // Attribute API
    // ═══════════════════════════════════════════════════════════════════════════

    /// Merge attrs into the default vertex attributes.
    void update_default_vertex_attributes(const std::vector<std::pair<std::string, double>>& attrs);

    /// Merge attrs into the default face attributes.
    void update_default_face_attributes(const std::vector<std::pair<std::string, double>>& attrs);

    /// Merge attrs into the default edge attributes.
    void update_default_edge_attributes(const std::vector<std::pair<std::string, double>>& attrs);

    /// Return the attribute of a vertex, falling back to the default; nullopt when neither exists.
    std::optional<double> vertex_attribute(size_t key, const std::string& name) const;

    /// Store an attribute on a vertex.
    void set_vertex_attribute(size_t key, const std::string& name, double value);

    /// Return the attribute of a face, falling back to the default; nullopt when neither exists.
    std::optional<double> face_attribute(size_t fkey, const std::string& name) const;

    /// Store an attribute on a face.
    void set_face_attribute(size_t fkey, const std::string& name, double value);

    /// Return the attribute of an edge, falling back to the default; nullopt when neither exists.
    std::optional<double> edge_attribute(std::pair<size_t, size_t> edge, const std::string& name) const;

    /// Store an attribute on an edge.
    void set_edge_attribute(std::pair<size_t, size_t> edge, const std::string& name, double value);

    /// Return the attribute of every vertex in keys; keys nullptr means all; the result holds nullopt for missing values.
    std::vector<std::optional<double>> vertices_attribute(
        const std::string& name,
        const std::vector<size_t>* keys = nullptr
    ) const;

    /// Store an attribute on every vertex in keys; keys nullptr means all.
    void set_vertices_attribute(const std::string& name, double value, const std::vector<size_t>* keys = nullptr);

    /// Return the attribute of every face in keys; keys nullptr means all; the result holds nullopt for missing values.
    std::vector<std::optional<double>> faces_attribute(
        const std::string& name,
        const std::vector<size_t>* keys = nullptr
    ) const;

    /// Store an attribute on every face in keys; keys nullptr means all.
    void set_faces_attribute(const std::string& name, double value, const std::vector<size_t>* keys = nullptr);

    /// Return the attribute of every edge in keys; keys nullptr means all; the result holds nullopt for missing values.
    std::vector<std::optional<double>> edges_attribute(
        const std::string& name,
        const std::vector<std::pair<size_t, size_t>>* keys = nullptr
    ) const;

    /// Store an attribute on every edge in keys; keys nullptr means all.
    void set_edges_attribute(
        const std::string& name,
        double value,
        const std::vector<std::pair<size_t, size_t>>* keys = nullptr
    );

    /// Return the vertices whose attributes match every (name, value) condition.
    std::vector<size_t> vertices_where(const std::vector<std::pair<std::string, double>>& conditions) const;

    /// Return the faces whose attributes match every (name, value) condition.
    std::vector<size_t> faces_where(const std::vector<std::pair<std::string, double>>& conditions) const;

    /// Return the edges whose attributes match every (name, value) condition.
    std::vector<std::pair<size_t, size_t>> edges_where(
        const std::vector<std::pair<std::string, double>>& conditions
    ) const;

    /// Return the vertices for which pred(key, attributes) is true.
    std::vector<size_t> vertices_where_predicate(
        const std::function<bool(size_t, const std::map<std::string, double>&)>& pred
    ) const;

    /// Return the faces for which pred(key, attributes) is true.
    std::vector<size_t> faces_where_predicate(
        const std::function<bool(size_t, const std::map<std::string, double>&)>& pred
    ) const;

    /// Return the edges for which pred(edge, attributes) is true.
    std::vector<std::pair<size_t, size_t>> edges_where_predicate(
        const std::function<bool(std::pair<size_t, size_t>, const std::map<std::string, double>&)>& pred
    ) const;

    /// Return the face normal from the first three vertices; unitized false keeps twice the first-triangle area as length.
    std::optional<Vector> face_normal_unitized(size_t face_key, bool unitized) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometric Properties
    // ═══════════════════════════════════════════════════════════════════════════

    /// Return the total surface area of all faces.
    double area() const;

    /// Return the average of all vertex positions.
    Point centroid() const;

    /// Return the dihedral angle in degrees between the two faces sharing edge (u, v), nullopt on a boundary edge.
    std::optional<double> dihedral_angle(size_t u, size_t v) const;

    /// Return the dihedral angles of all interior edges as (angles, arcs, points); arcs and label points are built when asked.
    std::tuple<std::map<std::pair<size_t, size_t>, double>, std::vector<Polyline>, std::vector<Point>> dihedral_angles(
        double scale = 0.3,
        bool with_arcs = true,
        bool with_points = true
    ) const;

    /// Return the area of a face.
    std::optional<double> face_area(size_t face_key) const;

    /// Return the average of a face's vertex positions.
    std::optional<Point> face_centroid(size_t face_key) const;

    /// Return the unit normal of a face.
    std::optional<Vector> face_normal(size_t face_key) const;

    /// Return the unit normals of all faces.
    std::map<size_t, Vector> face_normals() const;

    /// Return the angle at a vertex inside a face.
    std::optional<double> vertex_angle_in_face(size_t vertex_key, size_t face_key) const;

    /// Return the area-weighted vertex normal.
    std::optional<Vector> vertex_normal(size_t vertex_key) const;

    /// Return the vertex normal with the given weighting.
    std::optional<Vector> vertex_normal_weighted(size_t vertex_key, NormalWeighting weighting) const;

    /// Return the area-weighted normals of all vertices.
    std::map<size_t, Vector> vertex_normals() const;

    /// Return the normals of all vertices with the given weighting.
    std::map<size_t, Vector> vertex_normals_weighted(NormalWeighting weighting) const;

    /// Return the enclosed volume of a closed mesh.
    double volume() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Triangle BVH
    // ═══════════════════════════════════════════════════════════════════════════

    /// Build and cache the BVH over the triangulated faces.
    void build_triangle_bvh(bool force = false) const;

    /// Collect the candidate triangle ids along a ray from the cached BVH; true when any.
    bool triangle_bvh_ray_cast(
        const Point& origin,
        const Vector& direction,
        std::vector<int>& candidate_ids,
        bool find_all = false
    ) const;

    /// Look up the face index, sub-triangle index and corners of a cached triangle id; false when out of range.
    bool get_triangle_by_id(int tri_id, size_t& face_idx, size_t& sub_idx, Point& v0, Point& v1, Point& v2) const;

    /// Drop the cached BVH, AABB tree and triangle data.
    void clear_triangle_bvh() const;

    /// Build and cache the AABB tree over the triangulated faces.
    void build_triangle_aabb_tree(bool force = false) const;

    /// Return the cached triangle BVH, nullptr before build_triangle_bvh.
    const SpatialBVH* get_cached_bvh() const { return triangle_bvh.get(); }

    /// Return the cached triangle AABB tree, nullptr before build_triangle_aabb_tree.
    const SpatialAABBTree* get_cached_aabb_tree() const { return triangle_aabb_tree.get(); }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Transform every vertex in place and drop the triangle caches; always true.
    bool transform(const Xform& xf);

    /// Return a transformed copy.
    Mesh transformed(const Xform& xf) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════

    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Mesh jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static Mesh file_json_loads(const std::string& json_string);

    /// Write to a JSON file.
    void file_json_dump(const std::string& filename) const;

    /// Read from a JSON file.
    static Mesh file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static Mesh pb_loads(const std::string& data);

    /// Write to a protobuf file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file.
    static Mesh pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String Representation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Return the "Mesh(name=..., vertices=..., faces=...)" form.
    std::string str() const;

    /// Return the multi-line form with name, vertices, faces and edges.
    std::string repr() const;

    /// Write the str() form to a stream.
    friend std::ostream& operator<<(std::ostream& os, const Mesh& mesh);
};

// ═══════════════════════════════════════════════════════════════════════════
// Loft
// ═══════════════════════════════════════════════════════════════════════════

/// Role of a face inside a loft panel.
enum class LoftFaceRole { TopCap, BotCap, QuadWall, TriWall };

/// One wall face of a loft panel and the original vertices it spans.
struct LoftWallFace {
    size_t face_key = 0;   // Local panel mesh face key.
    size_t face_index = 0; // Zero-based position of face_key in panel mesh.face.
    bool is_quad = false;  // Whether the wall is a quad rather than a triangle.
    size_t top_v0 = 0;     // Original top-mesh vertex key.
    size_t top_v1 = 0;     // Original top-mesh vertex key.
    size_t bot_v0 = 0;     // Original bot-mesh vertex key, valid when is_quad.
    size_t bot_v1 = 0;     // Original bot-mesh vertex key, valid when is_quad.
};

/// One lofted panel with its cap faces, walls and vertex maps.
struct LoftPanel {
    Mesh mesh;                                  // Panel mesh.
    std::optional<size_t> top_face_key;         // Local key of top cap face.
    std::optional<size_t> bot_face_key;         // Local key of bot cap face.
    std::vector<LoftWallFace> wall_faces;       // Wall faces in order.
    std::map<size_t, LoftFaceRole> face_roles;  // Face key to role for every face in mesh.
    std::map<size_t, size_t> orig_top_to_local; // Original top vertex key to local key.
    std::map<size_t, size_t> orig_bot_to_local; // Original bot vertex key to local key.
    std::vector<size_t> top_vertices;           // Local keys of the top cap.
    std::vector<size_t> bot_vertices;           // Local keys of the bot cap.
};

/// Two panel walls that face each other.
struct LoftAdjPair {
    size_t pi; // Panel index for side i.
    size_t wi; // Wall face index for side i.
    size_t pj; // Panel index for side j.
    size_t wj; // Wall face index for side j.
};

/// Panels of loft_panels with their wall adjacency.
struct LoftResult {
    std::vector<LoftPanel> panels;      // One panel per matched polygon pair.
    std::vector<LoftAdjPair> adjacency; // Facing wall pairs.
    Mesh top_mesh;                      // Top polygons of the matched panels, one face per panel.
    Mesh bot_mesh;                      // Bot polygons of the matched panels, one face per panel.
};

} // namespace session_cpp
