#pragma once

#include "color.h"
#include "guid.h"
#include "json.h"
#include "mesh.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "xform.h"
#include <string>
#include <utility>
#include <vector>

namespace session_proto {
class NurbsSurfaceTrimmed;
}

namespace session_cpp {

/// Trim wires of one face as UV polygons, optional 3D points per loop vertex shared bit for bit with the neighbouring face, and interior UV seeds.
struct TrimLoops {
    std::vector<std::vector<Point>> uv; // UV polygon per loop.
    std::vector<std::vector<Point>> xyz; // 3D point per loop vertex, empty when not shared.
    std::vector<Point> interior_uv; // UV seeds inside the face.
};

/// A NURBS surface bounded by a closed outer loop and optional inner loops in its UV space.
class NurbsSurfaceTrimmed {
private:
    mutable std::string _guid; // Lazily minted GUID.

public:
    std::string name = "my_nurbssurface_trimmed"; // Face name.
    double width = 1.0; // Display width.
    Color surfacecolor = Color::black(); // Display color of the surface.
    NurbsSurface m_surface; // Underlying surface.
    NurbsCurve m_outer_loop; // Closed outer loop in UV space.
    std::vector<NurbsCurve> m_inner_loops; // Closed hole loops in UV space.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty untrimmed face.
    NurbsSurfaceTrimmed();

    /// Copy with a new guid and the same data.
    NurbsSurfaceTrimmed(const NurbsSurfaceTrimmed& other);

    /// Copy-assign with a new guid and the same data.
    NurbsSurfaceTrimmed& operator=(const NurbsSurfaceTrimmed& other);

    /// Move while preserving the guid.
    NurbsSurfaceTrimmed(NurbsSurfaceTrimmed&& other) noexcept = default;

    /// Move-assign while preserving the guid.
    NurbsSurfaceTrimmed& operator=(NurbsSurfaceTrimmed&& other) noexcept = default;

    /// Destroy the face.
    ~NurbsSurfaceTrimmed();

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Surface with a closed outer loop given in its UV parameter space.
    static NurbsSurfaceTrimmed create(const NurbsSurface& surface, const NurbsCurve& outer_loop);

    /// Planar surface fitted to a closed 3D boundary, the boundary projected as the outer loop.
    static NurbsSurfaceTrimmed create_planar(const NurbsCurve& boundary);

    /// One trimmed face per region of the UV domain carved by the pcurves (x=u, y=v, z=0); dangling cutters are discarded.
    static std::vector<NurbsSurfaceTrimmed> split_by_uv_curves(
        const NurbsSurface& srf,
        const std::vector<NurbsCurve>& pcurves,
        double tolerance = 0.0
    );

    /// One trimmed face per non-empty region carved by the planes (all 2^K sign combinations).
    static std::vector<NurbsSurfaceTrimmed> split_by_planes(
        const NurbsSurface& srf,
        const std::vector<std::pair<Point, Vector>>& planes
    );

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Compare name, width, color, surface and trim loops; guid ignored.
    bool operator==(const NurbsSurfaceTrimmed& other) const;

    /// Compare name, width, color, surface and trim loops; guid ignored.
    bool operator!=(const NurbsSurfaceTrimmed& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform the surface in place; the loops live in UV and stay.
    void transform(const Xform& xform);

    /// Return a transformed copy.
    NurbsSurfaceTrimmed transformed(const Xform& xform) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
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

    /// Return a copy of the underlying surface.
    NurbsSurface surface() const;

    /// Return a copy of the outer loop.
    NurbsCurve get_outer_loop() const;

    /// Replace the outer loop.
    void set_outer_loop(const NurbsCurve& loop);

    /// Return whether the outer loop is a valid curve.
    bool is_trimmed() const;

    /// Return whether the underlying surface is valid.
    bool is_valid() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Inner loops
    // ═══════════════════════════════════════════════════════════════════════════
    /// Hole given directly as a closed 2D curve in UV space.
    void add_inner_loop(const NurbsCurve& loop_2d);

    /// Hole from a 3D curve pulled onto the surface and normalized into [0,1]^2.
    void add_hole(const NurbsCurve& curve_3d);

    /// Add one hole per 3D curve pulled onto the surface.
    void add_holes(const std::vector<NurbsCurve>& curves_3d);

    /// Return a copy of the inner loop at index.
    NurbsCurve get_inner_loop(int index) const;

    /// Return the number of inner loops.
    int inner_loop_count() const;

    /// Remove every inner loop.
    void clear_inner_loops();

    // ═══════════════════════════════════════════════════════════════════════════
    // Evaluation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the surface point at (u, v).
    Point point_at(double u, double v) const;

    /// Return the unit surface normal at (u, v).
    Vector normal_at(double u, double v) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Meshing
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return mesh_q at 20 degrees and a chord factor of 0.005.
    Mesh mesh() const;

    /// Deflection-refined constrained Delaunay of the trim loops: angular bound in degrees, chord factor as a fraction of the bbox diagonal.
    Mesh mesh_q(double max_angle_deg, double chord_factor) const;

    /// Mesh sampled loops (outer first, then holes) keeping every loop vertex, tagged boundary/{loop}/{sample}; knot crossings add boundary_interval/{loop}/{segment}; empty mesh on invalid input.
    Mesh mesh_loops(const TrimLoops& loops, double max_angle_deg, double chord_factor) const;

    /// Mesh of the half (S-q0).n <= 0: span-adaptive grid, marching-squares clip with Newton-refined crossings, seams welded.
    Mesh mesh_by_plane(const Point& q0, const Vector& normal, double max_angle_deg, double chord_factor) const;

    /// Mesh of the region inside every half-space (S-q).n <= 0: triangle soup clipped plane by plane, seams welded.
    Mesh mesh_by_planes(
        const std::vector<std::pair<Point, Vector>>& planes,
        double max_angle_deg,
        double chord_factor
    ) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static NurbsSurfaceTrimmed jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static NurbsSurfaceTrimmed file_json_loads(const std::string& json_string);

    /// Write to a JSON file.
    void file_json_dump(const std::string& filename) const;

    /// Read from a JSON file.
    static NurbsSurfaceTrimmed file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::NurbsSurfaceTrimmed to_proto() const;

    /// Construct from the protobuf message.
    static NurbsSurfaceTrimmed from_proto(const session_proto::NurbsSurfaceTrimmed& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static NurbsSurfaceTrimmed pb_loads(const std::string& data);

    /// Write to a protobuf file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file.
    static NurbsSurfaceTrimmed pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "NurbsSurfaceTrimmed(name=..., trimmed=..., holes=...)".
    std::string str() const;

    /// Return the multi-line form with the surface.
    std::string repr() const;

    /// Stream the str() form.
    friend std::ostream& operator<<(std::ostream& os, const NurbsSurfaceTrimmed& ts);

private:

    /// Copy every field but the guid.
    void deep_copy_from(const NurbsSurfaceTrimmed& src);

    /// Return the diagonal of the control-point box, the scale every deflection tolerance is a fraction of.
    double bbox_diagonal() const;

    /// UV polygon of a trim loop: control points or samples, each edge split until its 3D chord is within deflection.
    std::vector<Point> discretize_loop(const NurbsCurve& crv, double deflection) const;

    /// Constrained Delaunay of the loops in UV, refined, trimmed, lifted and welded: the one body mesh_q and mesh_loops share.
    Mesh triangulate(const TrimLoops& loops, double max_angle_deg, double chord_factor) const;
};

} // namespace session_cpp
