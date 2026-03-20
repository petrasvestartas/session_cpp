#pragma once
#include "point.h"
#include "vector.h"
#include "color.h"
#include "xform.h"
#include "xform.h"
#include "boundingbox.h"
#include "bvh.h"
#include "aabb.h"
#include "tolerance.h"
#include "json.h"
#include "line.h"
#include "polyline.h"
#include <map>
#include <set>
#include <vector>
#include <string>
#include <optional>
#include <cmath>
#include <tuple>

namespace session_cpp {

struct LoftPanel;     // defined in loft section below
struct LoftAdjPair;   // defined in loft section below
struct LoftResult;    // defined in loft section below


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

/// Normal weighting scheme for vertex normal computation
enum class NormalWeighting {
    Area,    ///< Weight by face area
    Angle,   ///< Weight by vertex angle in face
    Uniform  ///< Uniform weighting
};

/// Vertex data containing position and attributes
struct VertexData {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    std::map<std::string, double> attributes;

    VertexData() = default;
    VertexData(const Point& p) : x(p[0]), y(p[1]), z(p[2]) {}
    bool operator==(const VertexData& other) const {
        return x == other.x && y == other.y && z == other.z && attributes == other.attributes;
    }
    bool operator!=(const VertexData& other) const { return !(*this == other); }

    /// Get vertex position as Point
    Point position() const { return Point(x, y, z); }
    
    /// Set vertex position from Point
    void set_position(const Point& p) {
        x = p[0];
        y = p[1];
        z = p[2];
    }

    /// Get vertex color as RGB array
    std::array<double, 3> color() const {
        return {
            attributes.count("r") ? attributes.at("r") : 0.5,
            attributes.count("g") ? attributes.at("g") : 0.5,
            attributes.count("b") ? attributes.at("b") : 0.5
        };
    }

    /// Set vertex color
    void set_color(double r, double g, double b) {
        attributes["r"] = r;
        attributes["g"] = g;
        attributes["b"] = b;
    }

    /// Get vertex normal if set
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

    /// Set vertex normal
    void set_normal(double nx, double ny, double nz) {
        attributes["nx"] = nx;
        attributes["ny"] = ny;
        attributes["nz"] = nz;
    }
};

/// A halfedge mesh data structure for representing polygonal surfaces.
class Mesh {
public:
    std::map<size_t, std::map<size_t, std::optional<size_t>>> halfedge;  ///< Halfedge connectivity
    std::map<size_t, VertexData> vertex;                                  ///< Vertex data
    std::map<size_t, std::vector<size_t>> face;                          ///< Face vertex lists
    std::map<size_t, std::vector<std::vector<size_t>>> face_holes;       ///< Face hole rings (inner boundaries)
    std::map<size_t, std::map<std::string, double>> facedata;             ///< Face attributes
    std::map<std::pair<size_t, size_t>, std::map<std::string, double>> edgedata;  ///< Edge attributes
    std::map<std::string, double> default_vertex_attributes;              ///< Default vertex attrs
    std::map<std::string, double> default_face_attributes;                ///< Default face attrs
    std::map<std::string, double> default_edge_attributes;                ///< Default edge attrs
    std::string guid = ::guid();                                         ///< Unique identifier
    std::string name = "my_mesh";                                        ///< Mesh name
    ColorMode color_mode = ColorMode::OBJECTCOLOR;                        ///< Active color mode
    Xform xform;                                     ///< Transformation matrix

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
    std::vector<Color> pointcolors;                                      ///< Vertex colors
    std::vector<Color> facecolors;                                       ///< Face colors
    std::vector<Color> linecolors;                                       ///< Edge colors
    std::vector<double> widths;                                           ///< Edge widths
    Color objectcolor = Color::white();                                  ///< Object color
    size_t max_vertex = 0;                                               ///< Next vertex key
    size_t max_face = 0;                                                 ///< Next face key
    std::map<size_t, std::vector<std::array<size_t, 3>>> triangulation; ///< Cached triangulations

    // Cached per-triangle data for BVH ray casting
    mutable bool triangle_bvh_built = false;
    mutable std::shared_ptr<BVH> triangle_bvh;  ///< BVH over cached triangle AABBs
    mutable std::vector<BoundingBox> triangle_boxes_cache;  ///< Per-triangle AABBs (legacy, may be empty)
    mutable std::vector<BvhAABB> triangle_aabbs_cache;      ///< Lightweight AABBs for fast BVH build
    struct TriangleIndex { uint32_t i0, i1, i2; };
    mutable std::vector<TriangleIndex> triangle_indices_cache; ///< Triangle vertex indices
    mutable std::vector<std::pair<size_t, size_t>> triangle_face_subidx_cache; ///< (face_idx, sub_idx)
    mutable std::vector<Point> vertices_cache; ///< Cached vertices for fast lookup
    mutable std::shared_ptr<AABBTree> triangle_aabb_tree;

public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors
    ///////////////////////////////////////////////////////////////////////////////////////////

    Mesh();
    Mesh(const Mesh& other);
    Mesh& operator=(const Mesh& other);
    bool operator==(const Mesh& other) const;
    bool operator!=(const Mesh& other) const;
    ~Mesh();

    /// Create a mesh from a list of vertices and faces.
    static Mesh from_vertices_and_faces(const std::vector<Point>& vertices, const std::vector<std::vector<size_t>>& faces);

    /// Create a mesh from a list of polygons.
    /// precision: optional tolerance for vertex merging.
    static Mesh from_polylines(const std::vector<std::vector<Point>>& polygons, std::optional<double> precision = std::nullopt);
    
    /// Create a mesh from a list of lines.
    /// delete_boundary_face: if true, removes the boundary face.
    /// precision: optional tolerance for vertex merging.
    static Mesh from_lines(const std::vector<Line>& lines, bool delete_boundary_face = false, std::optional<double> precision = std::nullopt);

    /// Create a mesh from a polygon boundary with optional holes.
    /// sort_by_bbox: if true, picks the largest polyline by bbox diagonal as boundary.
    static Mesh from_polygon_with_holes(const std::vector<std::vector<Point>>& polylines, bool sort_by_bbox = false);

    /// Loft between two sets of polylines to create a mesh volume.
    /// cap: if true, adds bottom and top cap faces.
    static Mesh loft(const std::vector<Polyline>& polylines0, const std::vector<Polyline>& polylines1, bool cap = true);

    /// Batch version of from_polygon_with_holes, with optional parallel execution.
    static std::vector<Mesh> from_polygon_with_holes_many(
        const std::vector<std::vector<std::vector<Point>>>& inputs,
        bool sort_by_bbox = false, bool parallel = true);

    /// Batch version of loft, with optional parallel execution.
    static std::vector<Mesh> loft_many(
        const std::vector<std::pair<std::vector<Polyline>, std::vector<Polyline>>>& pairs,
        bool cap = true, bool parallel = true);

    /// Loft between matched pairs of top/bottom polygons, producing one panel per pair.
    /// Each panel has a top cap, optional bottom cap, matched quad walls, and triangle fill.
    /// merge_precision: vertex-merge tolerance.
    /// edge_gap: if > 0, insets bottom wall vertices toward face center.
    /// edge_match_threshold: multiplier on average edge-midpoint distance for quad matching.
    /// add_caps: if true, adds top and bottom cap faces.
    /// skip_triangles: if true, omits triangle fill for unmatched edges.
    /// Returns one LoftPanel per matched face pair, in centroid-distance order.
    static LoftResult loft_panels(
        const std::vector<std::vector<Point>>& top_polygons,
        const std::vector<std::vector<Point>>& bot_polygons,
        double merge_precision      = 0.001,
        double edge_gap             = 0.0,
        double edge_match_threshold = 2.0,
        bool   add_caps             = true,
        bool   skip_triangles       = false);

    /// Create a box mesh centered at the origin.
    /// Returns a closed mesh with 8 vertices and 6 quad faces.
    static Mesh create_box(double x, double y, double z);

    /// Create a dodecahedron mesh (12 pentagonal faces) with given edge length.
    static Mesh create_dodecahedron(double edge = 2.0);

    /// Create a closed mesh from interleaved top/bottom polyline pairs [top0, bot0, top1, bot1, ...].
    /// top0/top1/... are the outer boundary and optional inner holes; bot* are the matching bottoms.
    /// Equivalent to loft() but accepts the CGAL-style interleaved format directly.
    /// scale: divide all coordinates by this factor (e.g. 1000.0 to convert mm to m).
    static Mesh from_polyline_pairs(const std::vector<Polyline>& pairs, double scale = 1.0);

    /// Output a closed mesh as flat VNF arrays from interleaved top/bottom polyline pairs.
    /// out_vertices: flat [x,y,z,...], out_normals: flat [nx,ny,nz,...] per vertex (flat-shaded),
    /// out_triangles: sequential indices [0,1,2, 3,4,5, ...]. Coordinates divided by scale.
    static void from_polyline_pairs_vnf(
        const std::vector<Polyline>& pairs,
        std::vector<double>& out_vertices,
        std::vector<double>& out_normals,
        std::vector<int>& out_triangles,
        double scale = 1.0);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Boolean Queries
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Check if mesh is empty
    bool is_empty() const { return vertex.empty(); }

    /// Check if mesh is valid (has vertices, faces, and all face vertex keys exist)
    bool is_valid() const;

    /// Check if mesh is closed (no boundary edges — every halfedge has an opposite face)
    bool is_closed() const;

    /// Check if a vertex is on the boundary
    bool is_vertex_on_boundary(size_t vertex_key) const;

    /// Check if an edge is on the boundary
    bool is_edge_on_boundary(size_t u, size_t v) const;

    /// Check if a face is on the boundary
    bool is_face_on_boundary(size_t face_key) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Attributes
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Get number of vertices
    size_t number_of_vertices() const { return vertex.size(); }

    /// Get number of faces
    size_t number_of_faces() const { return face.size(); }

    /// Get number of edges
    size_t number_of_edges() const;

    /// Calculate Euler characteristic (V - E + F)
    int euler() const;

    /// Returns sorted vertex keys.
    std::vector<size_t> vertices() const;

    /// Returns sorted face keys.
    std::vector<size_t> faces() const;

    /// Returns all undirected edges as (u, v) pairs in stable sorted order.
    /// linecolors[i] corresponds to edges()[i].
    std::vector<std::pair<size_t, size_t>> edges() const;

    /// Returns vertices and faces with sequential 0-based indices.
    std::pair<std::vector<Point>, std::vector<std::vector<size_t>>> to_vertices_and_faces() const;

    /// Returns a map from sparse vertex keys to sequential indices (0, 1, 2, ...).
    std::map<size_t, size_t> vertex_index() const;

    /// Returns boundary (naked=true) or interior (naked=false) edges.
    std::vector<std::pair<size_t, size_t>> naked_edges(bool boundary = true) const;

    /// Returns boundary (naked=true) or interior (naked=false) vertices.
    std::vector<size_t> naked_vertices(bool boundary = true) const;

    /// Returns boundary (naked=true) or interior (naked=false) faces.
    std::vector<size_t> naked_faces(bool boundary = true) const;


    ///////////////////////////////////////////////////////////////////////////////////////////
    // Vertex and Face Operations
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Add a vertex to the mesh.
    /// vkey: optional explicit key.
    /// Returns the vertex key.
    size_t add_vertex(const Point& position, std::optional<size_t> vkey = std::nullopt);
    
    /// Add a face to the mesh.
    /// fkey: optional explicit key.
    /// Returns the face key, or nullopt if invalid.
    std::optional<size_t> add_face(const std::vector<size_t>& vertices, std::optional<size_t> fkey = std::nullopt);

    /// Remove a vertex and all faces that use it, then clean up orphaned halfedges.
    void remove_vertex(size_t vkey);

    /// Remove a face and its orphaned halfedges. Resizes color arrays to new counts.
    void remove_face(size_t fkey);

    /// Remove an edge, its adjacent faces, and its halfedges.
    void remove_edge(size_t u, size_t v);

    /// Reverse the winding of a face in place (remove → reverse → re-add with same key).
    void flip_face(size_t fkey);

    /// Reverse the winding of all faces in place.
    void flip();

    /// Clear all mesh data
    void clear();

    /// Returns a new mesh with all vertices duplicated so each face has its own unique vertices.
    /// After unweld, number_of_vertices() == sum of each face's vertex count.
    Mesh unweld() const;

    /// Merge vertices within Euclidean distance <= tolerance using BVH spatial acceleration.
    /// Degenerate faces (collapsed vertices) are discarded.
    Mesh weld(double tolerance = 0.001) const;

    /// Unify face winding by BFS.
    /// Returns true if any face was flipped.
    bool unify_winding();

    /// For closed meshes: flip entire mesh if normals point inward. Returns true if flipped.
    bool orient_outward();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Connectivity Queries
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Get all edges sharing a vertex with edge (u,v), excluding (u,v) and (v,u)
    std::optional<std::vector<std::pair<size_t, size_t>>> edge_edges(size_t u, size_t v) const;

    /// Get the faces on each side of an edge
    std::optional<std::vector<size_t>> edge_faces(size_t u, size_t v) const;

    /// Get the edge as a Line
    std::optional<Line> edge_line(size_t u, size_t v) const;

    /// Get edges of a face as (vi, vi+1) pairs
    std::optional<std::vector<std::pair<size_t, size_t>>> face_edges(size_t face_key) const;

    /// Get faces adjacent to a face (sharing an edge)
    std::optional<std::vector<size_t>> face_faces(size_t face_key) const;

    /// Get the points of a face
    std::optional<std::vector<Point>> face_points(size_t face_key) const;

    /// Get the face as a Polyline
    std::optional<Polyline> face_polyline(size_t face_key) const;

    /// Get the vertices of a face
    std::optional<std::vector<size_t>> face_vertices(size_t face_key) const;

    /// Get edges incident to a vertex as (vertex_key, neighbor) pairs
    std::optional<std::vector<std::pair<size_t, size_t>>> vertex_edges(size_t vertex_key) const;

    /// Get faces incident to a vertex
    std::optional<std::vector<size_t>> vertex_faces(size_t vertex_key) const;

    /// Get the position of a vertex
    std::optional<Point> vertex_point(size_t vertex_key) const;

    /// Get neighboring vertices of a vertex
    std::optional<std::vector<size_t>> vertex_vertices(size_t vertex_key) const;


    ///////////////////////////////////////////////////////////////////////////////////////////
    // Geometric Properties
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Calculate the normal of a face
    std::optional<Vector> face_normal(size_t face_key) const;

    /// Calculate the centroid of a face (average of vertex positions)
    std::optional<Point> face_centroid(size_t face_key) const;

    /// Calculate the centroid of the mesh (average of all vertex positions)
    Point centroid() const;
    
    /// Calculate the normal of a vertex (area-weighted)
    std::optional<Vector> vertex_normal(size_t vertex_key) const;
    
    /// Calculate the normal of a vertex with specified weighting
    std::optional<Vector> vertex_normal_weighted(size_t vertex_key, NormalWeighting weighting) const;
    
    /// Calculate the area of a face
    std::optional<double> face_area(size_t face_key) const;

    /// Calculate the total surface area of all faces
    double area() const;

    /// Calculate the enclosed volume of a closed mesh
    double volume() const;

    /// Calculate the angle at a vertex in a face
    std::optional<double> vertex_angle_in_face(size_t vertex_key, size_t face_key) const;

    /// Calculate the dihedral angle (in radians) between two faces sharing edge (u,v).
    /// Returns nullopt for boundary edges (only one face) or invalid edges.
    std::optional<double> dihedral_angle(size_t u, size_t v) const;

    /// Calculate normals for all faces
    std::map<size_t, Vector> face_normals() const;
    
    /// Calculate normals for all vertices (area-weighted)
    std::map<size_t, Vector> vertex_normals() const;
    
    /// Calculate normals for all vertices with specified weighting
    std::map<size_t, Vector> vertex_normals_weighted(NormalWeighting weighting) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    bool transform(const Xform& xf);
    void transform();
    Mesh transformed() const;
    Mesh transformed(const Xform& xf) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to JSON-serializable object
    nlohmann::ordered_json jsondump() const;
    
    /// Create mesh from JSON data
    static Mesh jsonload(const nlohmann::json& data);

    /// Convert to JSON string
    std::string json_dumps() const;

    /// Load from JSON string
    static Mesh json_loads(const std::string& json_string);

    /// Write JSON to file
    void json_dump(const std::string& filename) const;

    /// Read JSON from file
    static Mesh json_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to protobuf binary string
    std::string pb_dumps() const;

    /// Load from protobuf binary string
    static Mesh pb_loads(const std::string& data);

    /// Write protobuf to file
    void pb_dump(const std::string& filename) const;

    /// Read protobuf from file
    static Mesh pb_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representation
    ///////////////////////////////////////////////////////////////////////////////////////////

    std::string str() const;
    std::string repr() const;
    friend std::ostream& operator<<(std::ostream& os, const Mesh& mesh);

    friend class Closest;
    friend class Intersection;

    void build_triangle_bvh(bool force = false) const;
    bool triangle_bvh_ray_cast(const Point& origin, const Vector& direction, std::vector<int>& candidate_ids, bool find_all = false) const;
    bool get_triangle_by_id(int tri_id, size_t& face_idx, size_t& sub_idx, Point& v0, Point& v1, Point& v2) const;
    void clear_triangle_bvh() const;
    void build_triangle_aabb_tree(bool force = false) const;
    const BVH* get_cached_bvh() const { return triangle_bvh.get(); }
    const AABBTree* get_cached_aabb_tree() const { return triangle_aabb_tree.get(); }
};

///////////////////////////////////////////////////////////////////////////////////////////
// Loft
///////////////////////////////////////////////////////////////////////////////////////////



enum class LoftFaceRole { TopCap, BotCap, QuadWall, TriWall };

struct LoftWallFace {
    size_t face_key;        ///< local panel mesh face key
    size_t face_index;      ///< 0-based position of face_key in panel mesh.face (sorted map order)
    bool   is_quad;
    size_t top_v0, top_v1; ///< original top-mesh vertex keys
    size_t bot_v0, bot_v1; ///< original bot-mesh vertex keys (valid if is_quad)
};

// serve as a bridge between the original input meshes and the new local panel mesh:
struct LoftPanel {
    Mesh   mesh;
    std::optional<size_t> top_face_key;         ///< local key of top cap face (nullopt if no cap)
    std::optional<size_t> bot_face_key;         ///< local key of bot cap face (nullopt if no cap)
    std::vector<LoftWallFace>      wall_faces;
    std::map<size_t,LoftFaceRole>  face_roles;  ///< face_key → role for every face in mesh
    //  original vertex key → local panel mesh vertex key
    std::map<size_t,size_t>    orig_top_to_local;
    std::map<size_t,size_t>    orig_bot_to_local;
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
    Mesh top_mesh;   ///< merged top input mesh (from_polylines of all top polygons)
    Mesh bot_mesh;   ///< merged bot input mesh (from_polylines of all bot polygons)
};

} // namespace session_cpp
