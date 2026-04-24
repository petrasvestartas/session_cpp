#pragma once

#include "nurbssurface.h"
#include "nurbscurve.h"
#include "xform.h"
#include "color.h"
#include "mesh.h"
#include "guid.h"
#include "json.h"
#include <vector>
#include <string>

namespace session_cpp {

/**
 * @class NurbsSurfaceTrimmed
 * @brief A NURBS surface bounded by a closed outer loop and optional inner loops (holes).
 *
 * Represents a face of a boundary representation: the underlying surface provides
 * the parametric geometry, while 2D trim curves define the visible region.
 * Meshing respects both outer boundary and inner hole loops.
 */
class NurbsSurfaceTrimmed {
public:
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string name = "my_nurbssurface_trimmed";
    double width = 1.0;
    Color surfacecolor = Color::black();
    Xform xform = Xform::identity();

    NurbsSurface m_surface;
    NurbsCurve m_outer_loop;
    std::vector<NurbsCurve> m_inner_loops;

public:
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Static Factory Methods
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Create trimmed surface from a NURBS surface and a 2D outer boundary loop.
    /// The outer_loop must be a closed curve in the surface's UV parameter space.
    static NurbsSurfaceTrimmed create(const NurbsSurface& surface, const NurbsCurve& outer_loop);

    /// Create a planar trimmed surface from a closed 3D boundary curve.
    /// Fits a plane to the curve, projects it to 2D, and builds the trim loop automatically.
    static NurbsSurfaceTrimmed create_planar(const NurbsCurve& boundary);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors & Destructor
    ///////////////////////////////////////////////////////////////////////////////////////////

    NurbsSurfaceTrimmed();
    NurbsSurfaceTrimmed(const NurbsSurfaceTrimmed& other);
    NurbsSurfaceTrimmed& operator=(const NurbsSurfaceTrimmed& other);
    bool operator==(const NurbsSurfaceTrimmed& other) const;
    bool operator!=(const NurbsSurfaceTrimmed& other) const;
    ~NurbsSurfaceTrimmed();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Accessors
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Get the underlying NURBS surface.
    NurbsSurface surface() const;

    /// Get the outer boundary loop (closed 2D curve in UV space).
    NurbsCurve get_outer_loop() const;

    /// Replace the outer boundary loop with a new closed 2D curve.
    void set_outer_loop(const NurbsCurve& loop);

    /// Return true if the surface has a non-trivial outer loop (not the full domain).
    bool is_trimmed() const;

    /// Return true if the surface, outer loop, and all inner loops are valid.
    bool is_valid() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Inner Loops
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Add a pre-built 2D hole loop directly in UV parameter space.
    void add_inner_loop(const NurbsCurve& loop_2d);

    /// Add a hole from a 3D curve by projecting it onto the surface's UV domain.
    void add_hole(const NurbsCurve& curve_3d);

    /// Add multiple holes from 3D curves, projecting each onto the surface.
    void add_holes(const std::vector<NurbsCurve>& curves_3d);

    /// Get the inner loop at the given index (0-based).
    NurbsCurve get_inner_loop(int index) const;

    /// Return the number of inner (hole) loops.
    int inner_loop_count() const;

    /// Remove all inner loops.
    void clear_inner_loops();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Evaluation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Evaluate a 3D point on the underlying surface at parameters (u, v).
    Point point_at(double u, double v) const;

    /// Evaluate the surface normal at parameters (u, v).
    Vector normal_at(double u, double v) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Meshing
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Generate a triangle mesh respecting outer and inner trim loops.
    /// Uses adaptive subdivision based on surface curvature and chord-height tolerance.
    Mesh mesh() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Apply the stored xform to the surface geometry and reset xform to identity.
    void transform();

    /// Return a copy with the stored xform applied.
    NurbsSurfaceTrimmed transformed() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to JSON object with fields in alphabetical order.
    nlohmann::ordered_json jsondump() const;

    /// Construct from a JSON object.
    static NurbsSurfaceTrimmed jsonload(const nlohmann::json& data);

    /// Write JSON to a file.
    void file_json_dump(const std::string& filename) const;

    /// Read from a JSON file.
    static NurbsSurfaceTrimmed file_json_load(const std::string& filename);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static NurbsSurfaceTrimmed file_json_loads(const std::string& json_string);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serialize to a protobuf binary string.
    std::string pb_dumps() const;

    /// Deserialize from a protobuf binary string.
    static NurbsSurfaceTrimmed pb_loads(const std::string& data);

    /// Write protobuf to a file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file.
    static NurbsSurfaceTrimmed pb_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Simple string (type, surface degree, trim loop counts).
    std::string str() const;

    /// Detailed string with surface domain and CV counts.
    std::string repr() const;

    /// Stream output operator (calls str()).
    friend std::ostream& operator<<(std::ostream& os, const NurbsSurfaceTrimmed& ts);

private:
    mutable std::string _guid;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Internal Helpers
    ///////////////////////////////////////////////////////////////////////////////////////////

    void deep_copy_from(const NurbsSurfaceTrimmed& src);
};

} // namespace session_cpp
