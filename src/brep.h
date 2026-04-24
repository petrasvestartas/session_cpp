#pragma once

#include "nurbssurface.h"
#include "nurbscurve.h"
#include "nurbssurface_trimmed.h"
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

enum class BRepTrimType { Boundary = 0, Mated = 1, Seam = 2, Singular = 3 };
enum class BRepLoopType { Outer = 0, Inner = 1 };

struct BRepVertex {
    int point_index = -1;
    std::vector<int> edge_indices;
};

struct BRepEdge {
    int curve_3d_index = -1;
    int start_vertex = -1;
    int end_vertex = -1;
    std::vector<int> trim_indices;
};

struct BRepTrim {
    int curve_2d_index = -1;
    int edge_index = -1;
    int loop_index = -1;
    bool reversed = false;
    BRepTrimType type = BRepTrimType::Boundary;
};

struct BRepLoop {
    std::vector<int> trim_indices;
    int face_index = -1;
    BRepLoopType type = BRepLoopType::Outer;
};

struct BRepFace {
    int surface_index = -1;
    std::vector<int> loop_indices;
    bool reversed = false;
    Color facecolor = Color(0, 0, 0, 0);  // a=0 → not set
};

/**
 * @class BRep
 * @brief Boundary Representation solid model with indexed topology.
 *
 * Stores geometry pools (surfaces, 3D/2D curves, vertices) and topology tables
 * (faces, loops, trims, edges, vertices) that reference into the pools by index.
 * Supports construction from primitives, polylines, or NURBS curves with holes.
 */
class BRep {
public:
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string name = "my_brep";
    double width = 1.0;
    Color surfacecolor = Color::black();
    Xform xform = Xform::identity();

    // Geometry pools
    std::vector<NurbsSurface> m_surfaces;
    std::vector<NurbsCurve> m_curves_3d;
    std::vector<NurbsCurve> m_curves_2d;
    std::vector<Point> m_vertices;

    // Topology
    std::vector<BRepVertex> m_topology_vertices;
    std::vector<BRepEdge> m_topology_edges;
    std::vector<BRepTrim> m_trims;
    std::vector<BRepLoop> m_loops;
    std::vector<BRepFace> m_faces;

public:
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Static Factory Methods
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Create an axis-aligned box centered at the origin with given half-extents.
    static BRep create_box(double sx, double sy, double sz);

    /// Create a cylinder along +Z with the given radius and height.
    static BRep create_cylinder(double radius, double height);

    /// Create a sphere centered at the origin with the given radius.
    static BRep create_sphere(double radius);

    /// Create an axis-aligned box with a cylindrical hole through its center.
    static BRep create_block_with_hole(double sx, double sy, double sz, double hole_radius);

    /// Build a BRep from closed planar polylines; each polyline becomes a trimmed planar face.
    static BRep from_polylines(const std::vector<Polyline>& polylines);

    /// Build a BRep from closed NURBS curves with optional hole curves per face.
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

    /// Return the number of faces in the BRep.
    int face_count() const;

    /// Return the number of topological edges.
    int edge_count() const;

    /// Return the number of topological vertices.
    int vertex_count() const;

    /// Return true if the BRep has valid topology and non-empty geometry pools.
    bool is_valid() const;

    /// Return true if the BRep forms a closed (watertight) solid.
    bool is_solid() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Building
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Add a NURBS surface to the geometry pool. Returns the surface index.
    int add_surface(const NurbsSurface& srf);

    /// Add a 3D NURBS curve to the geometry pool. Returns the curve index.
    int add_curve_3d(const NurbsCurve& crv);

    /// Add a 2D trim curve to the geometry pool. Returns the curve index.
    int add_curve_2d(const NurbsCurve& crv);

    /// Add a vertex point to the geometry pool. Returns the vertex index.
    int add_vertex(const Point& pt);

    /// Add a topological edge referencing a 3D curve and two endpoint vertices.
    int add_edge(int curve_3d_idx, int start_vertex, int end_vertex);

    /// Add a trim referencing a 2D curve, its parent edge and loop, with orientation.
    int add_trim(int curve_2d_idx, int edge_idx, int loop_idx, bool reversed, BRepTrimType type);

    /// Add a loop (outer or inner) belonging to a face. Returns the loop index.
    int add_loop(int face_idx, BRepLoopType type);

    /// Add a face referencing a surface, with optional reversed orientation.
    int add_face(int surface_idx, bool reversed);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Meshing
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Generate a combined triangle mesh from all faces, respecting trim loops.
    Mesh mesh() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Evaluation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Evaluate a 3D point on the given face at parameters (u, v).
    Point point_at(int face_idx, double u, double v) const;

    /// Evaluate the surface normal on the given face at parameters (u, v).
    Vector normal_at(int face_idx, double u, double v) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Apply the stored xform to all geometry and reset xform to identity.
    void transform();

    /// Return a copy with the stored xform applied to all geometry.
    BRep transformed() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to JSON object with fields in alphabetical order.
    nlohmann::ordered_json jsondump() const;

    /// Construct from a JSON object.
    static BRep jsonload(const nlohmann::json& data);

    /// Write JSON to a file.
    void file_json_dump(const std::string& filename) const;

    /// Read from a JSON file.
    static BRep file_json_load(const std::string& filename);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static BRep file_json_loads(const std::string& json_string);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serialize to a protobuf binary string.
    std::string pb_dumps() const;

    /// Deserialize from a protobuf binary string.
    static BRep pb_loads(const std::string& data);

    /// Write protobuf to a file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file.
    static BRep pb_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Simple string (type, face/edge/vertex counts).
    std::string str() const;

    /// Detailed string with topology and geometry pool sizes.
    std::string repr() const;

    /// Stream output operator (calls str()).
    friend std::ostream& operator<<(std::ostream& os, const BRep& brep);

private:
    mutable std::string _guid;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Internal Helpers
    ///////////////////////////////////////////////////////////////////////////////////////////

    void deep_copy_from(const BRep& src);
};

} // namespace session_cpp
