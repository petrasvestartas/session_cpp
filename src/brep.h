#pragma once

#include "nurbssurface.h"
#include "nurbscurve.h"
#include "polyline.h"
#include "mesh.h"
#include "point.h"
#include "xform.h"
#include "color.h"
#include "guid.h"
#include "json.h"
#include <vector>
#include <string>

namespace session_cpp {

/// TopAbs_Orientation: carried by the parent -> child reference, never by the shape.
enum class BRepOrientation { Forward = 0, Reversed = 1, Internal = 2, External = 3 };

/// TopAbs::Reverse.
BRepOrientation brep_reverse(BRepOrientation o);

/// TopAbs::Compose: the orientation of a sub-shape reached through a parent with orientation `a`.
BRepOrientation brep_compose(BRepOrientation a, BRepOrientation b);

/// TopoDS_Shape: an oriented reference to a sub-shape (index into the owning table).
struct BRepRef {
    int index = -1;
    BRepOrientation orientation = BRepOrientation::Forward;
    bool operator==(const BRepRef& o) const { return index == o.index && orientation == o.orientation; }
};

/// BRep_TVertex.
struct BRepVertex {
    Point point;
    double tolerance = 0.0;
};

/// BRep_CurveOnSurface / BRep_CurveOnClosedSurface. curve_2d_index_2 is the pcurve of the
/// REVERSED use of the edge on a closed surface (seam); -1 otherwise. Pcurves run in the
/// edge's own direction (OCCT SameParameter convention).
struct BRepCurveOnSurface {
    int surface_index = -1;
    int curve_2d_index = -1;
    int curve_2d_index_2 = -1;
};

/// BRep_TEdge. curve_3d_index is -1 for a degenerated edge (sphere pole, cone apex).
struct BRepEdge {
    int curve_3d_index = -1;
    int start_vertex = -1;
    int end_vertex = -1;
    double tolerance = 0.0;
    bool degenerated = false;
    std::vector<BRepCurveOnSurface> pcurves;
};

/// TopoDS_TWire.
struct BRepWire {
    std::vector<BRepRef> edges;
};

/// BRep_TFace. The first wire is the outer boundary.
struct BRepFace {
    int surface_index = -1;
    std::vector<BRepRef> wires;
    double tolerance = 0.0;
    Color facecolor = Color(0, 0, 0, 0);  // a=0 -> not set
};

/// TopoDS_TShell.
struct BRepShell {
    std::vector<BRepRef> faces;
};

/// TopoDS_TSolid.
struct BRepSolid {
    std::vector<BRepRef> shells;
};

/**
 * @class BRep
 * @brief Boundary representation after OCCT's TopoDS/BRep model, with indexed tables.
 *
 * Geometry pools (surfaces, 3D curves, 2D pcurves) and shape tables (vertices, edges, wires,
 * faces, shells, solids). Every parent -> child link is a BRepRef carrying the orientation.
 * The BRep itself is the compound: its free shapes are those no parent references.
 */
class BRep {
public:
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    /// Clear the guid so a FRESH one mints lazily on next read.
    void refresh_guid() { _guid.clear(); }
    std::string name = "my_brep";
    double width = 1.0;
    Color surfacecolor = Color::black();

    // Geometry pools
    std::vector<NurbsSurface> m_surfaces;
    std::vector<NurbsCurve> m_curves_3d;
    std::vector<NurbsCurve> m_curves_2d;

    // Shape tables
    std::vector<BRepVertex> m_vertices;
    std::vector<BRepEdge> m_edges;
    std::vector<BRepWire> m_wires;
    std::vector<BRepFace> m_faces;
    std::vector<BRepShell> m_shells;
    std::vector<BRepSolid> m_solids;

public:
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Static Factory Methods
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Axis-aligned box centered at the origin: 6 faces, 12 edges, 8 vertices, one solid.
    static BRep create_box(double sx, double sy, double sz);

    /// Cylinder along +Z: one periodic body face (seam edge) and two planar caps.
    static BRep create_cylinder(double radius, double height);

    /// Sphere centered at the origin: one face, a seam meridian and two degenerated pole edges.
    static BRep create_sphere(double radius);

    /// Cone along +Z: base circle at z=0, apex at z=height (degenerated apex edge), planar base.
    static BRep create_cone(double radius, double height);

    /// Square pyramid: base edge `base` centered at the origin in z=0, apex at (0,0,height).
    static BRep create_pyramid(double base, double height);

    /// Torus in the XY plane: one face closed in both directions, two seam edges, one vertex.
    static BRep create_torus(double major_radius, double minor_radius);

    /// Axis-aligned box with a cylindrical through-hole along Z.
    static BRep create_block_with_hole(double sx, double sy, double sz, double hole_radius);

    /// One planar face per closed polyline; coincident vertices and edges are shared. Free faces.
    static BRep from_polylines(const std::vector<Polyline>& polylines);

    /// One planar face per closed curve with optional hole curves (inner wires). Free faces.
    static BRep from_nurbscurves(const std::vector<NurbsCurve>& curves, const std::vector<std::vector<NurbsCurve>>& holes = {});

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors & Destructor
    ///////////////////////////////////////////////////////////////////////////////////////////

    BRep();
    BRep(const BRep& other);
    BRep& operator=(const BRep& other);
    bool operator==(const BRep& other) const;
    bool operator!=(const BRep& other) const;
    ~BRep();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Accessors
    ///////////////////////////////////////////////////////////////////////////////////////////

    int vertex_count() const;
    int edge_count() const;
    int wire_count() const;
    int face_count() const;
    int shell_count() const;
    int solid_count() const;

    /// Every reference resolves into its table, every face has a surface and an outer wire,
    /// every edge has two vertices and (unless degenerated) a 3D curve.
    bool is_valid() const;

    /// BRep_Tool::IsClosed(shell): every non-degenerated edge is used exactly twice by the
    /// shell's faces (a seam counts twice through its two pcurves).
    bool is_closed(int shell_index) const;

    /// At least one solid, and every shell of every solid is closed.
    bool is_solid() const;

    /// Orientation of a face inside its first parent shell; Forward for a free face.
    BRepOrientation face_orientation(int face_index) const;

    /// BRep_Tool::CurveOnSurface(E, F): the pcurve index of an edge on a face's surface for the
    /// given use orientation (the REVERSED pcurve on a seam); -1 if none.
    int pcurve_index(int edge_index, int face_index, BRepOrientation orientation) const;

    /// The edges of a wire composed with the wire's own orientation (a Reversed wire is
    /// traversed backwards with every edge reversed).
    std::vector<BRepRef> wire_edges(const BRepRef& wire) const;

    /// Faces sharing an edge, each with the orientation of that edge use.
    std::vector<BRepRef> edge_faces(int edge_index) const;

    /// Vertex positions, in vertex order.
    std::vector<Point> vertex_points() const;

    /// BRepLib::UpdateTolerances: raise every edge tolerance to the worst distance between its
    /// curve ends (3D curve and each pcurve lifted through its surface) and its vertices, and
    /// every vertex tolerance to the worst incident edge end. Returns the largest tolerance.
    double update_tolerances();

    /// Volume of the tessellated boundary (divergence theorem); meaningful for solids only.
    double volume() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Building (BRep_Builder)
    ///////////////////////////////////////////////////////////////////////////////////////////

    int add_surface(const NurbsSurface& srf);
    int add_curve_3d(const NurbsCurve& crv);
    int add_curve_2d(const NurbsCurve& crv);

    /// MakeVertex.
    int add_vertex(const Point& pt, double tolerance = 0.0);

    /// MakeEdge: curve_3d_index -1 makes a degenerated edge (start == end vertex).
    int add_edge(int curve_3d_index, int start_vertex, int end_vertex, double tolerance = 0.0);

    /// UpdateEdge(E, pcurve, S): attach a pcurve on a surface; curve_2d_index_2 for the
    /// reversed use on a closed surface. Replaces an existing record for the same surface.
    void add_pcurve(int edge_index, int surface_index, int curve_2d_index, int curve_2d_index_2 = -1);

    /// MakeWire + Add(edges).
    int add_wire(const std::vector<BRepRef>& edges);

    /// MakeFace(S) + Add(wires); the first wire is the outer boundary.
    int add_face(int surface_index, const std::vector<BRepRef>& wires, double tolerance = 0.0);

    /// MakeShell + Add(faces).
    int add_shell(const std::vector<BRepRef>& faces);

    /// MakeSolid + Add(shells).
    int add_solid(const std::vector<BRepRef>& shells);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Meshing
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// One welded triangle mesh of every face, wound to the face's outward orientation.
    Mesh mesh() const;

    /// One mesh per face, in face order (vertices not shared across faces).
    std::vector<Mesh> face_meshes() const;

    /// As face_meshes with a tessellation-quality override for the grid-meshed faces.
    std::vector<Mesh> face_meshes_q(bool has_quality, double max_angle_deg, double chord_factor) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Evaluation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Surface point of a face at (u, v).
    Point point_at(int face_index, double u, double v) const;

    /// Surface normal of a face at (u, v), flipped when the face is Reversed in its shell.
    Vector normal_at(int face_index, double u, double v) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Transform surfaces, 3D curves and vertices in place (pcurves are parametric, untouched).
    void transform(const Xform& xform);

    /// Return a transformed copy.
    BRep transformed(const Xform& xform) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    nlohmann::ordered_json jsondump() const;
    static BRep jsonload(const nlohmann::json& data);
    void file_json_dump(const std::string& filename) const;
    static BRep file_json_load(const std::string& filename);
    std::string file_json_dumps() const;
    static BRep file_json_loads(const std::string& json_string);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    std::string pb_dumps() const;
    static BRep pb_loads(const std::string& data);
    void pb_dump(const std::string& filename) const;
    static BRep pb_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representation
    ///////////////////////////////////////////////////////////////////////////////////////////

    std::string str() const;
    std::string repr() const;
    friend std::ostream& operator<<(std::ostream& os, const BRep& brep);

private:
    mutable std::string _guid;

    /// UV polygon of one wire of a face (pcurves sampled in traversal order).
    std::vector<Point> wire_uv_points(int face_index, const BRepRef& wire) const;
};

} // namespace session_cpp
