#pragma once

#include "color.h"
#include "curve.h"
#include "guid.h"
#include "json.h"
#include "mesh.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "plane.h"
#include "point.h"
#include "polyline.h"
#include "surface.h"
#include "xform.h"
#include <array>
#include <functional>
#include <ranges>
#include <string>
#include <vector>

namespace session_proto {
class BRep;
}

namespace session_cpp {

/// A pool read as NURBS, for consumers not yet on Surface, Curve and Curve2d
template <typename T>
auto nurbs(const std::vector<T>& pool) {
    return pool | std::views::transform([](const T& item) -> decltype(auto) { return item.nurbs(); });
}

/// A pooled surface or curve edited as NURBS by a consumer not yet on the types: the copy is stored back when the handle goes
template <typename T>
struct NurbsEdit {
    T& item;
    std::remove_cvref_t<decltype(item.nurbs())> value = item.nurbs();
    ~NurbsEdit() { item = T(value); }
    auto* operator->() { return &value; }
    auto& operator*() { return value; }
};

template <typename T>
NurbsEdit<T> edit(T& item) {
    return {item};
}

class BRep;

/// The pcurve of a curve lying on a plane: the same line, circle or ellipse in the plane's parameters over the same parameter, the affine image of any other curve
Curve2d plane_pcurve(const Surface& plane, const Curve& c);

/// The pcurve of an edge curve on a surface in its true parameters, by closed-form inversion along the curve: each periodic parameter on the turn nearest `guess` (a rough uv at a curve parameter), which also gives the angle where it is free. A 2D line over its own length where the image is one at even speed, the same conic on a plane, else a cubic fit over the curve's parameter. `deviation` is the largest distance found from the curve to the surface
Curve2d true_pcurve(const Surface& surface, const Curve& world, const std::function<Point(double)>& guess, double& deviation);

/// TopAbs_Orientation: carried by the parent -> child reference, never by the shape
enum class BRepOrientation {
    Forward = 0, // Same direction as the shape.
    Reversed = 1, // Opposite direction.
    Internal = 2, // Inside the parent, both sides.
    External = 3, // Outside the parent, no side.
};

/// TopAbs::Reverse
BRepOrientation brep_reverse(BRepOrientation o);

/// TopAbs::Compose: the orientation of a sub-shape reached through a parent with orientation `a`
BRepOrientation brep_compose(BRepOrientation a, BRepOrientation b);

/// TopoDS_Shape: an oriented reference to a sub-shape (index into the owning table)
struct BRepRef {
    int index = -1; // Index into the owning table.
    BRepOrientation orientation = BRepOrientation::Forward; // Orientation of this use.

    /// Compare index and orientation.
    bool operator==(const BRepRef& o) const {

        return index == o.index && orientation == o.orientation;
    }
};

/// BRep_TVertex
struct BRepVertex {
    Point point; // Position.
    double tolerance = 0.0; // Vertex tolerance.
};

/// BRep_CurveOnSurface: curve_2d_index_2 is the pcurve of the REVERSED use on a closed surface (seam), -1 otherwise; pcurves run in the edge's own direction
struct BRepCurveOnSurface {
    int surface_index = -1; // Surface the pcurve lies on.
    int curve_2d_index = -1; // Pcurve of the forward use.
    int curve_2d_index_2 = -1; // Reversed use on a seam, else -1.
    std::array<double, 2> range = {0.0, 0.0}; // Part of the pcurve in use, the whole pcurve when empty; it maps linearly onto the edge's range.
    std::array<double, 2> range_2 = {0.0, 0.0}; // The same for the reversed use.
};

/// BRep_TEdge: curve_3d_index is -1 for a degenerated edge (sphere pole, cone apex)
struct BRepEdge {
    int curve_3d_index = -1; // 3D curve, -1 when degenerated.
    int start_vertex = -1; // Start vertex.
    int end_vertex = -1; // End vertex.
    double tolerance = 0.0; // Edge tolerance.
    bool degenerated = false; // True for a pole or apex edge.
    std::vector<BRepCurveOnSurface> pcurves; // One per surface.
    std::array<double, 2> range = {0.0, 0.0}; // Part of the 3D curve the edge runs over, the whole curve when empty: split edges share a curve and differ in range.
};

/// TopoDS_TWire
struct BRepWire {
    std::vector<BRepRef> edges; // Traversal order.
};

/// BRep_TFace: the first wire is the outer boundary; facecolor alpha 0 means unset
struct BRepFace {
    int surface_index = -1; // Underlying surface.
    std::vector<BRepRef> wires; // Outer wire first, then holes.
    double tolerance = 0.0; // Face tolerance.
    Color facecolor = Color(0, 0, 0, 0); // Display color, alpha 0 when unset.
};

/// TopoDS_TShell
struct BRepShell {
    std::vector<BRepRef> faces; // Oriented faces.
};

/// TopoDS_TSolid
struct BRepSolid {
    std::vector<BRepRef> shells; // Outer first.
};

/// Boundary representation after OCCT's TopoDS/BRep model: geometry pools, indexed shape tables, every parent -> child link a BRepRef carrying the orientation
class BRep {
private:
    mutable std::string _guid; // Lazily minted GUID.
    mutable Mesh _mesh; // Cached trimmed mesh, never serialized.
    mutable std::vector<double> _mesh_key; // Geometry the cached mesh was built from.

public:
    std::string name = "my_brep"; // BRep name.
    bool is_visible = true; // Whether a viewer draws it.
    bool is_locked = false; // Whether a viewer keeps it from moving.
    double width = 1.0; // Display width.
    Color surfacecolor = Color::lightgrey(); // Display color of the faces.
    std::vector<Surface> m_surfaces; // Surface pool.
    std::vector<Curve> m_curves_3d; // 3D edge curve pool.
    std::vector<Curve2d> m_curves_2d; // Pcurve pool.
    std::vector<BRepVertex> m_vertices; // Vertex table.
    std::vector<BRepEdge> m_edges; // Edge table.
    std::vector<BRepWire> m_wires; // Wire table.
    std::vector<BRepFace> m_faces; // Face table.
    std::vector<BRepShell> m_shells; // Shell table.
    std::vector<BRepSolid> m_solids; // Solid table.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty BRep.
    BRep();

    /// Copy with a new guid and the same data.
    BRep(const BRep& other);

    /// Copy-assign with a new guid and the same data.
    BRep& operator=(const BRep& other);

    /// Destroy the BRep.
    ~BRep();

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Axis-aligned box centered at the origin: 6 faces, 12 edges, 8 vertices, one solid
    static BRep create_box(double sx, double sy, double sz);

    /// Cylinder along +Z: one periodic body face (seam edge) and two planar caps
    static BRep create_cylinder(double radius, double height);

    /// Sphere centered at the origin: one face, a seam meridian and two degenerated pole edges
    static BRep create_sphere(double radius);

    /// Cone along +Z: base circle at z=0, apex at z=height (degenerated apex edge), planar base
    static BRep create_cone(double radius, double height);

    /// Truncated cone along +Z: base circle of radius_base at z=0, top circle of radius_top at z=height, two planar caps; a cone when radius_top is zero
    static BRep create_frustum(double radius_base, double radius_top, double height);

    /// Square pyramid: base edge `base` centered at the origin in z=0, apex at (0,0,height)
    static BRep create_pyramid(double base, double height);

    /// Torus in the XY plane: one face closed in both directions, two seam edges, one vertex
    static BRep create_torus(double major_radius, double minor_radius);

    /// Axis-aligned box with a cylindrical through-hole along Z
    static BRep create_block_with_hole(double sx, double sy, double sz, double hole_radius);

    /// One planar face per closed polyline, holes[i] the closed polylines bounding the holes of face i; coincident vertices and edges are shared, closed sheets become solids
    static BRep from_polylines(
        const std::vector<Polyline>& polylines,
        const std::vector<std::vector<Polyline>>& holes = {}
    );

    /// One planar face per closed curve with optional hole curves (inner wires); closed sheets become solids
    static BRep from_nurbscurves(
        const std::vector<NurbsCurve>& curves,
        const std::vector<std::vector<NurbsCurve>>& holes = {}
    );

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Compare name, width, color and table sizes; guid ignored.
    bool operator==(const BRep& other) const;

    /// Compare name, width, color and table sizes; guid ignored.
    bool operator!=(const BRep& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    bool has_guid() const {

        return !_guid.empty();
    }

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

    /// Clear the guid so a fresh one mints lazily on next read
    void refresh_guid() {

        _guid.clear();
    }

    /// Return the number of vertices.
    int vertex_count() const;

    /// Return the number of edges.
    int edge_count() const;

    /// Return the number of wires.
    int wire_count() const;

    /// Return the number of faces.
    int face_count() const;

    /// Return the number of shells.
    int shell_count() const;

    /// Return the number of solids.
    int solid_count() const;

    /// Check finite geometry, table references, tolerances, and connected closed boundary wires.
    bool is_valid() const;

    /// BRep_Tool::IsClosed(shell): every non-degenerated edge is used exactly twice by the shell's faces (a seam counts twice through its two pcurves)
    bool is_closed(int shell_index) const;

    /// Check valid topology and closed shells with opposite boundary-edge uses.
    bool is_solid() const;

    /// Compose the first solid-shell-face orientation; Forward for a free face.
    BRepOrientation face_orientation(int face_index) const;

    /// BRep_Tool::CurveOnSurface(E, F): the pcurve index of an edge on a face's surface for the given use orientation (the REVERSED pcurve on a seam); -1 if none
    int pcurve_index(int edge_index, int face_index, BRepOrientation orientation) const;

    /// The edges of a wire composed with the wire's own orientation (a Reversed wire is traversed backwards with every edge reversed)
    std::vector<BRepRef> wire_edges(const BRepRef& wire) const;

    /// Faces sharing an edge, each with the orientation of that edge use
    std::vector<BRepRef> edge_faces(int edge_index) const;

    /// Vertex positions, in vertex order
    std::vector<Point> vertex_points() const;

    /// One closed polyline per PLANAR face: the outer wire walked in wire order with its winding untouched (lofts pair loops by it), inner wires ignored; index-aligned with face_planes
    std::vector<Polyline> face_polylines() const;

    /// The plane of every face face_polylines emits: centroid origin, Newell normal flipped for a Reversed face, and the whole set flipped when a closed solid encloses negative volume so normals point outward; free faces keep the wire's sign
    std::vector<Plane> face_planes() const;

    /// BRep_Tool::Range: the part of its 3D curve an edge runs over, the curve's domain when the edge has no range
    std::pair<double, double> edge_range(int edge_index) const;

    /// The part of a pcurve an edge use runs over (the reversed use of a seam with second), the pcurve's domain when it has no range
    std::pair<double, double> pcurve_range(const BRepCurveOnSurface& use, bool second = false) const;

    /// An edge's 3D curve over its range
    Curve edge_curve(int edge_index) const;

    /// A pcurve over its range
    Curve2d pcurve(const BRepCurveOnSurface& use, bool second = false) const;

    /// An edge's 3D curve over its range as NURBS: for consumers not yet on ranges
    NurbsCurve edge_nurbs(int edge_index) const;

    /// A pcurve over its range as NURBS
    NurbsCurve pcurve_nurbs(const BRepCurveOnSurface& use, bool second = false) const;

    /// Whether an edge or a pcurve use runs over a part of its curve.
    bool has_ranges() const;

    /// A copy in which every edge and pcurve use owns its whole curve (ranges cut into curves of their own, of the same kind)
    BRep unshared() const;

    /// Whether every surface, curve and pcurve is freeform, with or without ranges.
    bool is_freeform() const;

    /// Whether every surface, curve and pcurve is freeform and no edge has a range: what a consumer of NURBS can read as it is.
    bool is_nurbs() const;

    /// The bridge for consumers of NURBS: the same solid with every surface and curve as its NURBS and every pcurve in the parameter of that NURBS, over the parameter of its edge's NURBS; exact for lines on planes and for circles and rulings of a surface, fitted otherwise with the distance left put into the edge tolerance. An angle becomes a parameter of one unit a quarter turn; a length runs from 0 to 1, or without unit_lengths keeps its parameter. Itself when is_nurbs()
    BRep as_nurbs(bool unit_lengths = true) const;

    /// BRepLib::UpdateTolerances: raise every vertex tolerance to its edges' and to the worst gap between their curve ends (3D and lifted pcurves) and it; returns the largest
    double update_tolerances();

    /// Volume of the tessellated boundary (divergence theorem); meaningful for solids only
    double volume() const;

    /// Integrate the trimmed NURBS solid volume natively without tessellation.
    double volume_exact(double tolerance = 1e-7) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Building
    // ═══════════════════════════════════════════════════════════════════════════
    /// Append a surface to the pool; returns its index.
    int add_surface(const Surface& srf);

    /// Append a 3D curve to the pool; returns its index.
    int add_curve_3d(const Curve& crv);

    /// Append a pcurve to the pool; returns its index.
    int add_curve_2d(const Curve2d& crv);

    /// MakeVertex
    int add_vertex(const Point& pt, double tolerance = 0.0);

    /// MakeEdge: curve_3d_index -1 makes a degenerated edge (start == end vertex)
    int add_edge(int curve_3d_index, int start_vertex, int end_vertex, double tolerance = 0.0, const std::array<double, 2>& range = {0.0, 0.0});

    /// UpdateEdge(E, pcurve, S): attach a pcurve on a surface, curve_2d_index_2 for the reversed use on a closed surface; replaces an existing record for the same surface
    void add_pcurve(int edge_index, int surface_index, int curve_2d_index, int curve_2d_index_2 = -1, const std::array<double, 2>& range = {0.0, 0.0}, const std::array<double, 2>& range_2 = {0.0, 0.0});

    /// MakeWire + Add(edges)
    int add_wire(const std::vector<BRepRef>& edges);

    /// MakeFace(S) + Add(wires); the first wire is the outer boundary
    int add_face(int surface_index, const std::vector<BRepRef>& wires, double tolerance = 0.0);

    /// MakeShell + Add(faces)
    int add_shell(const std::vector<BRepRef>& faces);

    /// MakeSolid + Add(shells)
    int add_solid(const std::vector<BRepRef>& shells);

    // ═══════════════════════════════════════════════════════════════════════════
    // Meshing
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the cached welded mesh of every trimmed face, rebuilt with a new guid when the geometry changes.
    const Mesh& mesh() const;

    /// Return whether the cached mesh matches the current geometry.
    bool has_mesh() const;

    /// Tessellate trimmed faces with absolute chord deflection and angular deflection in radians.
    Mesh tessellate(double deflection, double angle = 0.35) const;

    /// One mesh per face, in face order (vertices not shared across faces)
    std::vector<Mesh> face_meshes() const;

    /// As face_meshes with a tessellation-quality override (max_angle_deg, chord_factor) when has_quality
    std::vector<Mesh> face_meshes_q(bool has_quality, double max_angle_deg, double chord_factor) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Evaluation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Surface point of a face at (u, v)
    Point point_at(int face_index, double u, double v) const;

    /// Surface normal of a face at (u, v), flipped when the face is Reversed in its shell
    Vector normal_at(int face_index, double u, double v) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform surfaces, 3D curves and vertices in place (pcurves are parametric, untouched)
    void transform(const Xform& xform);

    /// Return a transformed copy
    BRep transformed(const Xform& xform) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Cutting
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the part on the side the plane normal points to, every section loop capped by one planar face; a copy when everything lies on that side, empty when the plane cuts a BRep with a curved face or edge
    BRep cut_by_plane(const Plane& plane) const;

    /// Miter-offset a convex planar solid; reject curved or nonconvex inputs and return empty after complete erosion.
    BRep offset_convex(double distance, double tolerance = 1e-6) const;

    /// Union of solid BReps using native face intersections and shell assembly.
    BRep boolean_union(const BRep& other, double tolerance = 1e-7) const;

    /// Common volume of solid BReps using native face intersections and shell assembly.
    BRep boolean_intersection(const BRep& other, double tolerance = 1e-7) const;

    /// Subtract another solid BRep using native face intersections and shell assembly.
    BRep boolean_difference(const BRep& other, double tolerance = 1e-7) const;

    /// Symmetric difference of solid BReps using native face intersections and shell assembly.
    BRep boolean_xor(const BRep& other, double tolerance = 1e-7) const;

    /// Offset a BRep natively.
    BRep offset(double distance, double tolerance = 1e-7) const;

    /// Build a hollow solid with signed wall thickness, removing the specified faces as openings.
    BRep hollow(const std::vector<int>& openings, double thickness, double tolerance = 1e-7) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static BRep jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static BRep file_json_loads(const std::string& json_string);

    /// Write to a JSON file.
    void file_json_dump(const std::string& filename) const;

    /// Read from a JSON file.
    static BRep file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::BRep to_proto() const;

    /// Construct from the protobuf message.
    static BRep from_proto(const session_proto::BRep& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static BRep pb_loads(const std::string& data);

    /// Write to a protobuf file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file.
    static BRep pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "BRep(name=..., faces=..., edges=..., vertices=...)".
    std::string str() const;

    /// Return the multi-line form with the solid flag.
    std::string repr() const;

    /// Stream the str() form.
    friend std::ostream& operator<<(std::ostream& os, const BRep& brep);
};

} // namespace session_cpp
