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
#include <utility>
#include <array>

namespace session_cpp {

/// Trim wires of one face as UV polygons, optional 3D points per loop vertex shared bit for bit with the neighbouring face, and interior UV seeds
struct TrimLoops {
    std::vector<std::vector<Point>> uv;
    std::vector<std::vector<Point>> xyz;
    std::vector<Point> interior_uv;
};

/// A NURBS surface bounded by a closed outer loop and optional inner loops in its UV space
class NurbsSurfaceTrimmed {
public:
    bool has_guid() const { return !_guid.empty(); }
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string name = "my_nurbssurface_trimmed";
    double width = 1.0;
    Color surfacecolor = Color::black();

    NurbsSurface m_surface;
    NurbsCurve m_outer_loop;
    std::vector<NurbsCurve> m_inner_loops;

public:
    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════

    /// Surface with a closed outer loop given in its UV parameter space
    static NurbsSurfaceTrimmed create(const NurbsSurface& surface, const NurbsCurve& outer_loop);

    /// Planar surface fitted to a closed 3D boundary, the boundary projected as the outer loop
    static NurbsSurfaceTrimmed create_planar(const NurbsCurve& boundary);

    /// One trimmed face per region of the UV domain carved by the pcurves (x=u, y=v, z=0); dangling cutters are discarded
    static std::vector<NurbsSurfaceTrimmed> split_by_uv_curves(const NurbsSurface& srf, const std::vector<NurbsCurve>& pcurves, double tolerance = 0.0);

    /// One trimmed face per non-empty region carved by the planes (all 2^K sign combinations)
    static std::vector<NurbsSurfaceTrimmed> split_by_planes(const NurbsSurface& srf, const std::vector<std::pair<Point, Vector>>& planes);

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════

    NurbsSurfaceTrimmed();
    /// Copy (new guid, same data)
    NurbsSurfaceTrimmed(const NurbsSurfaceTrimmed& other);
    /// Move keeps the guid: the same object in a new place
    NurbsSurfaceTrimmed(NurbsSurfaceTrimmed&& other) noexcept = default;
    NurbsSurfaceTrimmed& operator=(NurbsSurfaceTrimmed&& other) noexcept = default;
    NurbsSurfaceTrimmed& operator=(const NurbsSurfaceTrimmed& other);
    bool operator==(const NurbsSurfaceTrimmed& other) const;
    bool operator!=(const NurbsSurfaceTrimmed& other) const;
    ~NurbsSurfaceTrimmed();

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════

    NurbsSurface surface() const;
    NurbsCurve get_outer_loop() const;
    void set_outer_loop(const NurbsCurve& loop);
    /// True when the outer loop is a valid curve
    bool is_trimmed() const;
    /// True when the underlying surface is valid
    bool is_valid() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Inner loops
    // ═══════════════════════════════════════════════════════════════════════════

    /// Hole given directly as a closed 2D curve in UV space
    void add_inner_loop(const NurbsCurve& loop_2d);
    /// Hole from a 3D curve pulled onto the surface and normalized into [0,1]^2
    void add_hole(const NurbsCurve& curve_3d);
    void add_holes(const std::vector<NurbsCurve>& curves_3d);
    NurbsCurve get_inner_loop(int index) const;
    int inner_loop_count() const;
    void clear_inner_loops();

    // ═══════════════════════════════════════════════════════════════════════════
    // Evaluation
    // ═══════════════════════════════════════════════════════════════════════════

    Point point_at(double u, double v) const;
    Vector normal_at(double u, double v) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Meshing
    // ═══════════════════════════════════════════════════════════════════════════

    /// mesh_q at 20 degrees and a chord factor of 0.005
    Mesh mesh() const;

    /// Deflection-refined constrained Delaunay of the trim loops: angular bound in degrees, chord factor as a fraction of the bbox diagonal
    Mesh mesh_q(double max_angle_deg, double chord_factor) const;

    /// Mesh sampled loops (outer first, then holes) keeping every loop vertex, tagged boundary/{loop}/{sample}; knot crossings add boundary_interval/{loop}/{segment}; empty mesh on invalid input
    Mesh mesh_loops(const TrimLoops& loops, double max_angle_deg, double chord_factor) const;

    /// Mesh of the half (S-q0).n <= 0: span-adaptive grid, marching-squares clip with Newton-refined crossings, seams welded
    Mesh mesh_by_plane(const Point& q0, const Vector& normal, double max_angle_deg, double chord_factor) const;

    /// Mesh of the region inside every half-space (S-q).n <= 0: triangle soup clipped plane by plane, seams welded
    Mesh mesh_by_planes(const std::vector<std::pair<Point, Vector>>& planes, double max_angle_deg, double chord_factor) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════

    void transform(const Xform& xform);
    NurbsSurfaceTrimmed transformed(const Xform& xform) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════

    nlohmann::ordered_json jsondump() const;
    static NurbsSurfaceTrimmed jsonload(const nlohmann::json& data);
    void file_json_dump(const std::string& filename) const;
    static NurbsSurfaceTrimmed file_json_load(const std::string& filename);
    std::string file_json_dumps() const;
    static NurbsSurfaceTrimmed file_json_loads(const std::string& json_string);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════

    std::string pb_dumps() const;
    static NurbsSurfaceTrimmed pb_loads(const std::string& data);
    void pb_dump(const std::string& filename) const;
    static NurbsSurfaceTrimmed pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════

    /// "NurbsSurfaceTrimmed(name=..., trimmed=..., holes=...)"
    std::string str() const;
    /// Multi-line form with the surface
    std::string repr() const;
    friend std::ostream& operator<<(std::ostream& os, const NurbsSurfaceTrimmed& ts);

private:
    /// Constrained Delaunay of the loops in UV, refined, trimmed, lifted and welded: the one body mesh_q and mesh_loops share
    Mesh triangulate(const TrimLoops& loops, double max_angle_deg, double chord_factor) const;
    /// Diagonal of the control-point box, the scale every deflection tolerance is a fraction of
    double bbox_diagonal() const;
    void deep_copy_from(const NurbsSurfaceTrimmed& src);

    mutable std::string _guid;
};

} // namespace session_cpp
