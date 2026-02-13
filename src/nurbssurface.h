#pragma once

#include "point.h"
#include "vector.h"
#include "plane.h"
#include "xform.h"
#include "color.h"
#include "tolerance.h"
#include "boundingbox.h"
#include "nurbscurve.h"
#include "guid.h"
#include "json.h"
#include "mesh.h"
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <utility>

namespace session_cpp {

class NurbsSurface {
public:
    std::string guid = ::guid();
    std::string name = "my_nurbssurface";
    double width = 1.0;
    std::vector<Color> pointcolors;
    std::vector<Color> facecolors;
    std::vector<Color> linecolors;
    Xform xform = Xform::identity();

    int m_dim;
    int m_is_rat;
    int m_order[2];
    int m_cv_count[2];
    int m_cv_stride[2];

    std::vector<double> m_knot[2];
    std::vector<double> m_cv;
    NurbsCurve m_outer_loop;
    std::vector<NurbsCurve> m_inner_loops;
    mutable Mesh m_mesh;

public:
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Static Factory Method
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Build surface from a flat list of 3D control points in row-major order
    /// (u varies slowest, v varies fastest). Allocates knot vectors as clamped
    /// uniform if not periodic. points.size() must equal cv_count_u * cv_count_v.
    /// Periodic flags wrap the knot structure in the respective direction.
    static NurbsSurface create(bool periodic_u, bool periodic_v,
                              int degree_u, int degree_v,
                              int cv_count_u, int cv_count_v,
                              const std::vector<Point>& points);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors & Destructor
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Default constructor. Creates an empty invalid surface with m_dim=0,
    /// zero orders, zero CV counts, and no knot/CV storage allocated.
    /// is_valid() returns false. Use create() or create_raw() to populate.
    NurbsSurface();

    /// Parametric constructor. Allocates CV array and knot vectors sized for
    /// the given orders and CV counts. Fills knots as clamped uniform [0..n].
    /// CVs are zeroed. Stride is set to dim (or dim+1 if rational).
    NurbsSurface(int dimension, bool is_rational,
                int order0, int order1,
                int cv_count0, int cv_count1);

    /// Deep copy. Copies all CV data, knots, trim loops, metadata (guid, name,
    /// color, xform). The copy gets a new guid.
    NurbsSurface(const NurbsSurface& other);

    /// Deep copy assignment. Same semantics as copy constructor —
    /// copies all data, generates new guid for the target.
    NurbsSurface& operator=(const NurbsSurface& other);

    /// Compares dimension, rationality, orders, CV counts, all knot values,
    /// and all CV coordinates within machine epsilon. Does NOT compare guid,
    /// name, color, or xform.
    bool operator==(const NurbsSurface& other) const;
    bool operator!=(const NurbsSurface& other) const;

    ~NurbsSurface();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Initialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Zero all member fields: m_dim=0, orders=0, CV counts=0.
    /// Clears knot and CV vectors. Does not free — vectors handle own memory.
    void initialize();

    /// Low-level allocation. Sets up knot vectors (periodic or clamped uniform)
    /// and CV array for the given dimensions. Returns false if params invalid
    /// (order < 2, cv_count < order). Prefer the parametric constructor or
    /// create() for normal use.
    bool create_raw(int dimension, bool is_rational,
               int order0, int order1,
               int cv_count0, int cv_count1,
               bool is_periodic_u = false, bool is_periodic_v = false,
               double knot_delta_u = 1.0, double knot_delta_v = 1.0);

    /// Convenience wrapper around create_raw that always produces clamped
    /// uniform knots with the given delta spacing. Equivalent to
    /// create_raw(..., false, false, delta0, delta1).
    bool create_clamped_uniform(int dimension,
                               int order0, int order1,
                               int cv_count0, int cv_count1,
                               double knot_delta0 = 1.0,
                               double knot_delta1 = 1.0);

    /// Clear all data and reset to empty invalid state.
    /// After this call, is_valid() returns false.
    void destroy();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Boolean Queries
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Checks structural consistency: orders >= 2, cv_count >= order in both
    /// dirs, knot vectors have correct length (cv_count + order - 2), knots
    /// are non-decreasing, CV array has correct total size, and strides are
    /// consistent with dimension and rationality.
    bool is_valid() const;

    /// Validate the knot vector: must be non-decreasing, no multiplicity
    /// exceeding order, and correct length relative to cv_count and order.
    bool is_valid_knot_vector(int dir) const;

    /// True if surface carries per-CV weights (NURBS). False for polynomial
    /// B-spline surfaces where all weights are implicitly 1.0.
    bool is_rational() const { return m_is_rat != 0; }

    /// True if the surface wraps in the given direction: the first and last
    /// rows (dir=0) or columns (dir=1) of CVs coincide within tolerance.
    /// Does not check knot periodicity — use is_periodic() for that.
    bool is_closed(int dir) const;

    /// True if the knot vector in dir has periodic structure: the first
    /// and last degree-many knot intervals are equal. A periodic surface
    /// is always closed but a closed surface may not be periodic.
    bool is_periodic(int dir) const;

    /// Tests whether all control points lie within a single plane (up to
    /// tolerance). If plane is non-null and surface is planar, writes the
    /// best-fit plane. Uses SVD of the CV centroid-relative matrix.
    bool is_planar(Plane* plane = nullptr, double tolerance = Tolerance::ZERO_TOLERANCE) const;

    /// True if the given boundary edge is collapsed to a single point —
    /// all CVs along that edge coincide within tolerance.
    /// side: 0=south (v=0), 1=east (u=1), 2=north (v=1), 3=west (u=0).
    bool is_singular(int side) const;

    /// True if end knots have full multiplicity (equal to order) in dir.
    /// end: 0=start only, 1=end only, 2=both ends.
    /// Clamped knots force the surface to interpolate the boundary CVs.
    bool is_clamped(int dir, int end = 2) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Attributes
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Surface dimension — always 3 for 3D geometry.
    /// Rational surfaces store dim+1 values per CV (x*w, y*w, z*w, w).
    int dimension() const { return m_dim; }

    /// Polynomial order = degree + 1 in the given direction.
    /// dir=0 for u, dir=1 for v. Minimum order is 2 (linear).
    int order(int dir) const;

    /// Polynomial degree = order - 1 in the given direction.
    /// Degree 1 = linear, 2 = quadratic, 3 = cubic.
    int degree(int dir) const;

    /// Number of control vertices in the given direction.
    /// Must be >= order(dir) for a valid surface.
    int cv_count(int dir) const;

    /// Total number of control vertices = cv_count(0) * cv_count(1).
    /// This is the count of Point-sized entries in the CV array.
    int cv_count() const;

    /// Number of doubles stored per control vertex.
    /// Returns dim+1 if rational (homogeneous coords), dim otherwise.
    int cv_size() const;

    /// Number of knots in the given direction = cv_count + order - 2.
    /// Follows the OpenNURBS convention (no endpoint duplication beyond
    /// what the multiplicity requires).
    int knot_count(int dir) const;

    /// Number of non-degenerate knot spans (intervals where the basis is
    /// non-zero) in the given direction. Equals the number of distinct
    /// interior knot intervals.
    int span_count(int dir) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Control Vertex Access
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Raw pointer to the start of CV[i][j] data in m_cv. Points to dim
    /// doubles (non-rational) or dim+1 doubles (rational, stored as
    /// x*w, y*w, z*w, w). Use cv_size() for the element count.
    double* cv(int i, int j);
    const double* cv(int i, int j) const;

    /// Returns CV[i][j] as a 3D Point. For rational surfaces, divides the
    /// homogeneous coordinates by weight: (x*w/w, y*w/w, z*w/w).
    /// Returns origin if indices are out of range.
    Point get_cv(int i, int j) const;

    /// Returns CV[i][j] as homogeneous coordinates (x*w, y*w, z*w, w).
    /// For non-rational surfaces, w is always 1.0 and x,y,z are the
    /// Euclidean coordinates directly.
    bool get_cv_4d(int i, int j, double& x, double& y, double& z, double& w) const;

    /// Set CV[i][j] from a 3D Point. For rational surfaces, multiplies by
    /// the existing weight: stores (x*w, y*w, z*w). Weight is preserved.
    /// Returns false if indices are out of range.
    bool set_cv(int i, int j, const Point& point);

    /// Set CV[i][j] directly from homogeneous coordinates (x*w, y*w, z*w, w).
    /// For non-rational surfaces, only x,y,z are stored and w is ignored.
    /// Returns false if indices are out of range.
    bool set_cv_4d(int i, int j, double x, double y, double z, double w);

    /// Get the weight at CV[i][j]. Returns 1.0 for non-rational surfaces.
    /// For rational, reads the last component of the homogeneous CV.
    double weight(int i, int j) const;

    /// Set the weight at CV[i][j]. For rational surfaces, updates the w
    /// component and rescales x*w, y*w, z*w to preserve the 3D position.
    /// No-op for non-rational surfaces.
    bool set_weight(int i, int j, double weight);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Knot Access
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Get the knot value at the given index in direction dir.
    /// Index must be in [0, knot_count(dir)-1].
    double knot(int dir, int knot_index) const;

    /// Set the knot value at the given index. Does not enforce ordering —
    /// caller must ensure the knot vector remains non-decreasing.
    bool set_knot(int dir, int knot_index, double knot_value);

    /// Count how many consecutive knots equal the value at knot_index.
    /// Multiplicity determines continuity: mult=order means C^-1 (discontinuous),
    /// mult=1 means C^(degree-1) (maximum smoothness).
    int knot_multiplicity(int dir, int knot_index) const;

    /// Return a copy of the full knot vector for the given direction.
    /// Length is knot_count(dir).
    std::vector<double> get_knots(int dir) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Domain & Parameterization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Returns the parameter interval [t0, t1] where the surface is defined
    /// in the given direction. Determined by the first/last "active" knots:
    /// knot[degree] and knot[cv_count-1].
    std::pair<double, double> domain(int dir) const;

    /// Reparametrize the surface so domain(dir) becomes [t0, t1].
    /// Linearly remaps all knot values. Returns false if t0 >= t1.
    bool set_domain(int dir, double t0, double t1);

    /// Returns the sorted list of distinct knot values within the active
    /// domain. These are the span boundaries — parameter values where
    /// the polynomial pieces join.
    std::vector<double> get_span_vector(int dir) const;

    /// Evaluate the surface at a regular nu x nv grid of parameters spanning
    /// the full domain. Returns (points[nu+1][nv+1], params[nu+1][nv+1])
    /// where each param is (u,v). Useful for visualization and sampling.
    std::pair<std::vector<std::vector<Point>>, std::vector<std::vector<std::pair<double,double>>>>
        divide_by_count(int nu, int nv) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Knot Vector Operations
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Replace the knot vector in dir with clamped uniform sequence:
    /// end knots repeated order times, interior knots evenly spaced by delta.
    /// Preserves CV count and order. Surface shape changes if knots differ.
    bool make_clamped_uniform_knot_vector(int dir, double delta = 1.0);

    /// Replace the knot vector in dir with periodic uniform sequence:
    /// evenly spaced without end-clamping. The surface wraps smoothly
    /// if CVs are also set up periodically.
    bool make_periodic_uniform_knot_vector(int dir, double delta = 1.0);

    /// Insert a knot at the given value in direction dir using Oslo algorithm.
    /// Adds knot_multiplicity copies. Refines the knot vector without changing
    /// surface shape — new CVs are computed to maintain geometry.
    /// Increases cv_count(dir) by knot_multiplicity.
    bool insert_knot(int dir, double knot_value, int knot_multiplicity = 1);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Evaluation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Evaluate surface point at parameters (u,v) using de Boor's algorithm.
    /// Finds knot span in each direction, computes basis functions, and sums
    /// the weighted CVs. For rational surfaces, divides by the sum of
    /// weights (perspective division).
    Point point_at(double u, double v) const;

    /// Same as point_at but writes x,y,z directly to output doubles.
    /// Avoids Point construction overhead — used in tight meshing loops.
    void point_at(double u, double v, double& x, double& y, double& z) const;

    /// Evaluate point and partial derivatives up to num_derivs order at (u,v).
    /// Returns flat array: [S, Su, Sv, Suu, Suv, Svv, ...].
    /// num_derivs=0 returns just the point. Uses basis_functions_derivatives.
    std::vector<Vector> evaluate(double u, double v, int num_derivs = 0) const;

    /// Compute unit surface normal at (u,v) as the cross product of the first
    /// partial derivatives: N = normalize(dS/du x dS/dv). Returns zero vector
    /// at singular points where partials are parallel.
    Vector normal_at(double u, double v) const;

    /// Same as normal_at but writes nx,ny,nz directly to output doubles.
    /// Avoids Vector construction overhead.
    void normal_at(double u, double v, double& nx, double& ny, double& nz) const;

    /// Compute both point and normal at (u,v) in a single call. Shares span
    /// lookup and basis function computation between point and normal, making
    /// it faster than calling point_at + normal_at separately.
    /// Used extensively by the adaptive meshing pipeline.
    void point_and_normal_at(double u, double v,
                             double& px, double& py, double& pz,
                             double& nx, double& ny, double& nz) const;

    /// Get surface point at one of the four corners.
    /// u_end=0/1 maps to domain start/end in u, v_end=0/1 in v.
    /// Equivalent to point_at(domain(0).first/second, domain(1).first/second).
    Point point_at_corner(int u_end, int v_end) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Modification
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Reverse parameterization in dir. Flips the order of CVs and mirrors
    /// the knot vector. Surface shape is unchanged but parameter direction
    /// is reversed: point_at(u) becomes point_at(1-u).
    bool reverse(int dir);

    /// Swap u and v directions. Transposes the CV grid and exchanges the
    /// two knot vectors. After transpose, what was an iso-u curve becomes
    /// an iso-v curve and vice versa.
    bool transpose();

    /// Swap two coordinate axes (e.g., axis_i=0, axis_j=2 swaps X and Z)
    /// in every control vertex. Used for coordinate system conversions.
    bool swap_coordinates(int axis_i, int axis_j);

    /// Restrict the surface to sub-domain [t0,t1] in dir. Inserts knots at
    /// trim boundaries (if needed) and discards CVs and knots outside the
    /// interval. The resulting surface is geometrically identical within
    /// the sub-domain.
    bool trim(int dir, const std::pair<double, double>& domain);

    /// Split the surface at parameter c in dir into two separate surfaces.
    /// Allocates new NurbsSurface objects via new — caller must delete.
    /// Returns false if c is outside the domain.
    bool split(int dir, double c, NurbsSurface*& west_or_south_side, NurbsSurface*& east_or_north_side) const;

    /// Stub — not yet implemented. Always returns false.
    bool extend(int dir, const std::pair<double, double>& domain);

    /// Convert polynomial B-spline to rational form by adding weights=1.0
    /// to every CV. Surface shape is unchanged. No-op if already rational.
    bool make_rational();

    /// Convert rational to polynomial by removing weights. Only succeeds
    /// if all weights are equal (within tolerance). Divides each CV by its
    /// weight and strips the w component.
    bool make_non_rational();

    /// Ensure end knots in dir have full multiplicity (equal to order).
    /// Inserts knots at domain boundaries as needed. end: 0=start, 1=end.
    /// Clamping forces the surface to interpolate boundary CVs exactly.
    bool clamp_end(int dir, int end);

    /// Elevate polynomial degree in dir to desired_degree by inserting new
    /// knots and recomputing CVs. Surface shape is preserved exactly.
    /// No-op if current degree >= desired_degree.
    bool increase_degree(int dir, int desired_degree);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Apply this->xform matrix to all CVs in-place, then reset xform to
    /// identity. For rational surfaces, transforms homogeneous coordinates.
    /// Call before mesh() to ensure the mesh reflects the transformation.
    void transform();

    /// Apply the given 4x4 transformation matrix to all CVs in-place.
    /// Supports translation, rotation, scaling, and projection.
    /// For rational surfaces, transforms homogeneous (x*w, y*w, z*w, w).
    bool transform(const Xform& xform);

    /// Return a new surface with this->xform applied to CVs.
    /// The original surface is not modified. Copy gets new guid.
    NurbsSurface transformed() const;

    /// Return a new surface with the given xform applied to CVs.
    /// The original surface is not modified. Copy gets new guid.
    NurbsSurface transformed(const Xform& xform) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Geometric Operations
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Compute the axis-aligned bounding box enclosing all control vertices.
    /// The actual surface lies within the convex hull of CVs, which is
    /// always inside this box but may be tighter.
    BoundingBox get_bounding_box() const;

    /// Approximate surface area. Currently a stub returning 0.0.
    /// Future: sum triangle areas from a fine mesh approximation.
    double area(double tolerance = 1e-6) const;

    /// Extract an isoparametric curve at parameter c.
    /// dir=0: fix u=c, return curve in v (column of surface).
    /// dir=1: fix v=c, return curve in u (row of surface).
    /// Caller owns the returned pointer and must delete it.
    NurbsCurve* iso_curve(int dir, double c) const;

    /// Find closest point on the surface to the given 3D test point.
    /// Uses Newton iteration seeded from a coarse parameter grid search.
    /// Writes the parameter values to u_out, v_out. Returns the 3D point.
    Point closest_point(const Point& point, double& u_out, double& v_out) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Trimming
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Set the outer boundary trim curve. Lives in 2D UV parameter space and
    /// defines the valid region. If set, mesh() uses Delaunay triangulation
    /// constrained to this boundary instead of the UV-grid approach.
    void set_outer_loop(const NurbsCurve& loop);

    /// Return the outer trim loop. Returns empty/invalid NurbsCurve
    /// if no outer loop has been set (untrimmed surface).
    NurbsCurve get_outer_loop() const;

    /// Remove the outer trim loop, reverting to an untrimmed surface.
    void clear_outer_loop();

    /// Add an inner trim loop (hole) as a closed 2D curve in UV space.
    /// Inner loops are subtracted from the outer boundary during meshing.
    /// The curve must be oriented clockwise in UV space.
    void add_inner_loop(const NurbsCurve& loop);

    /// Project a 3D curve onto the surface UV space using closest_point,
    /// then add it as an inner trim loop (hole). The 3D curve should lie
    /// on or very near the surface for accurate projection.
    void add_hole(const NurbsCurve& curve_3d);

    /// Project multiple 3D curves onto UV space and add each as a hole.
    /// Convenience wrapper calling add_hole for each curve.
    void add_holes(const std::vector<NurbsCurve>& curves_3d);

    /// Get the inner trim loop at the given index.
    /// Index must be in [0, inner_loop_count()-1].
    NurbsCurve get_inner_loop(int index) const;

    /// Number of inner trim loops (holes) currently defined.
    int inner_loop_count() const;

    /// Remove all inner trim loops.
    void clear_inner_loops();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Meshing
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Tessellate using adaptive UV-parameter grid. Per span, samples normals
    /// at interior points and measures chord-height deviation to decide
    /// subdivision count. max_angle is normal-angle threshold in degrees.
    /// Produces structured quad-like triangle mesh. Best for untrimmed surfaces.
    Mesh mesh_grid(double max_angle = 20.0, double max_edge_length = 0.0,
                   double min_edge_length = 0.0, double max_chord_height = 0.0) const;

    /// Tessellate using constrained Delaunay triangulation with Ruppert
    /// refinement in UV space. outer_loop and inner_loops define the boundary.
    /// Refines triangles exceeding angle/edge/chord thresholds.
    /// Required for trimmed surfaces with holes.
    Mesh mesh_delaunay(double max_angle = 20.0, double max_edge_length = 0.0,
                       double min_edge_length = 0.0, double max_chord_height = 0.0) const;

    /// Auto-select meshing strategy: mesh_grid for untrimmed surfaces
    /// (no outer loop) and mesh_delaunay for trimmed. This is the primary
    /// meshing entry point for general use.
    Mesh mesh(double max_angle = 20.0, double max_edge_length = 0.0,
              double min_edge_length = 0.0, double max_chord_height = 0.0) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serialize to JSON with alphabetically ordered keys. Includes all NURBS
    /// data (dimension, orders, knots, CVs, rationality) plus metadata
    /// (guid, name, color arrays, xform, width).
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object. Reads all fields written by jsondump.
    /// Missing fields get default values.
    static NurbsSurface jsonload(const nlohmann::json& data);

    /// Write JSON to file (calls jsondump internally).
    void json_dump(const std::string& filename) const;

    /// Read JSON from file (calls jsonload internally).
    static NurbsSurface json_load(const std::string& filename);

    /// Serialize to JSON string.
    std::string json_dumps() const;

    /// Deserialize from JSON string.
    static NurbsSurface json_loads(const std::string& json_string);

    /// Serialize to protobuf binary format using the NurbsSurface message
    /// from session.proto. Includes all NURBS data and metadata.
    std::string pb_dumps() const;

    /// Deserialize from protobuf binary string.
    static NurbsSurface pb_loads(const std::string& data);

    /// Write protobuf binary to file.
    void pb_dump(const std::string& filename) const;

    /// Read protobuf binary from file.
    static NurbsSurface pb_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Short summary: "NurbsSurface(deg 3x3, cv 10x8)".
    /// Used by operator<< for stream output.
    std::string str() const;

    /// Detailed multi-line representation including rational flag,
    /// knot counts, domain ranges, and trim loop info.
    std::string repr() const;

    friend std::ostream& operator<<(std::ostream& os, const NurbsSurface& surface);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Advanced
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Set all CV coordinates to zero. For rational surfaces, sets weights
    /// to 1.0. Useful for initializing a surface before filling CVs manually.
    bool zero_cvs();

    /// Stub — not yet implemented. Intended to compare two surfaces
    /// geometrically within tolerance, optionally ignoring parameterization
    /// differences. Currently always returns false.
    bool is_duplicate(const NurbsSurface& other,
                     bool ignore_parameterization,
                     double tolerance = Tolerance::ZERO_TOLERANCE) const;

    /// Stub — not yet implemented. Intended to collapse one boundary edge
    /// to a single point (creating a degenerate/singular side).
    /// side: 0=SW, 1=SE, 2=NE, 3=NW.
    bool collapse_side(int side, const Point& point);

private:
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Internal
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Deep copy all data from src: dimension, orders, CV counts, strides,
    /// knot vectors, CV array, trim loops, and visual metadata.
    /// Called by copy constructor and assignment operator.
    void deep_copy_from(const NurbsSurface& src);

    /// Binary search for the knot span index containing parameter t in dir.
    /// Returns i such that knot[i] <= t < knot[i+1], clamped to the valid
    /// range [degree, cv_count-1]. Handles repeated knots correctly.
    int find_span(int dir, double t) const;

    /// Compute the non-zero B-spline basis functions N_{span-deg,p}(t)
    /// through N_{span,p}(t). Writes order values into basis vector.
    /// Uses the Cox-de Boor recurrence with the standard triangular table.
    void basis_functions(int dir, int span, double t, std::vector<double>& basis) const;

    /// Compute basis functions and their derivatives up to deriv_order at t.
    /// ders[k][j] = k-th derivative of the j-th basis function.
    /// Uses algorithm from "The NURBS Book" (Piegl & Tiller, A2.3).
    void basis_functions_derivatives(int dir, int span, double t, int deriv_order,
                                   std::vector<std::vector<double>>& ders) const;
};

} // namespace session_cpp
