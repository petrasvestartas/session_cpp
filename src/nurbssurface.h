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

/**
 * @class NurbsSurface
 * @brief Non-Uniform Rational B-Spline (NURBS) surface implementation
 * 
 * A NURBS surface is defined by:
 * - 2D array of control points (CVs)
 * - Two knot vectors (one for each parameter direction)
 * - Degrees in both directions (order = degree + 1)
 * - Optional weights for rational surfaces
 * 
 * Based on OpenNURBS implementation, adapted for session_cpp data types.
 */
class NurbsSurface {
public:
    std::string guid = ::guid();
    std::string name = "my_nurbssurface";
    double width = 1.0;
    Color surfacecolor = Color::black();
    Xform xform = Xform::identity();

    // Core NURBS data
    int m_dim;              // Dimension (typically 3 for 3D surfaces)
    int m_is_rat;           // 1 if rational, 0 if non-rational
    int m_order[2];         // Order = degree + 1 (order >= 2) for u and v
    int m_cv_count[2];      // Number of control vertices (>= order) in u and v
    int m_cv_stride[2];     // Stride between control vertices in m_cv array

    std::vector<double> m_knot[2];  // Knot vectors for u and v directions
    std::vector<double> m_cv;       // Control vertex data (homogeneous if rational)
    NurbsCurve m_outer_loop;        // 2D trim curve in UV space (empty = untrimmed)
    std::vector<NurbsCurve> m_inner_loops;  // Inner trim loops (holes)
    mutable Mesh m_mesh;            // Cached mesh (computed on first mesh() call)

public:
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Static Factory Methods
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Create surface from flat list of control points (row-major: u varies slowest)
    static NurbsSurface create(bool periodic_u, bool periodic_v,
                              int degree_u, int degree_v,
                              int cv_count_u, int cv_count_v,
                              const std::vector<Point>& points);

    /// Create ruled surface from two curves
    static NurbsSurface create_ruled(const NurbsCurve& curveA, const NurbsCurve& curveB);

    /// Create planar surface from boundary curve
    static NurbsSurface create_planar(const NurbsCurve& boundary);

    /// Loft (skinning) — surface through N section curves
    static NurbsSurface create_loft(const std::vector<NurbsCurve>& curves, int degree_v = 3);

    /// Surface of revolution — revolve profile curve around axis
    static NurbsSurface create_revolve(const NurbsCurve& profile, const Point& axis_origin,
                                        const Vector& axis_direction, double angle = 2.0 * 3.14159265358979323846);

    /// Sweep1 — sweep profile along single rail
    static NurbsSurface create_sweep1(const NurbsCurve& rail, const NurbsCurve& profile);

    /// Sweep2 — sweep shapes along two rails (interpolates between multiple cross-sections)
    static NurbsSurface create_sweep2(const NurbsCurve& rail1, const NurbsCurve& rail2,
                                       const std::vector<NurbsCurve>& shapes);

    /// Edge surface (Coons patch) from 4 boundary curves forming a closed loop
    static NurbsSurface create_edge_surface(const NurbsCurve& c0, const NurbsCurve& c1,
                                             const NurbsCurve& c2, const NurbsCurve& c3);

    /// Network surface (Gordon surface) from intersecting u-curves and v-curves
    static NurbsSurface create_network(const std::vector<NurbsCurve>& u_curves,
                                        const std::vector<NurbsCurve>& v_curves);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors & Destructor
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Default constructor
    NurbsSurface();
    
    /// Constructor with dimension, rationality, orders, and CV counts
    NurbsSurface(int dimension, bool is_rational, 
                int order0, int order1,
                int cv_count0, int cv_count1);
    
    /// Copy constructor
    NurbsSurface(const NurbsSurface& other);
    
    /// Copy assignment operator
    NurbsSurface& operator=(const NurbsSurface& other);

    /// Equality operator (compares all attributes except guid)
    bool operator==(const NurbsSurface& other) const;

    /// Inequality operator
    bool operator!=(const NurbsSurface& other) const;

    /// Destructor
    ~NurbsSurface();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Initialization & Creation
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Initialize all fields to zero/empty
    void initialize();
    
    /// Create NURBS surface with specified parameters and automatic knot vector initialization
    bool create_raw(int dimension, bool is_rational,
               int order0, int order1,
               int cv_count0, int cv_count1,
               bool is_periodic_u = false, bool is_periodic_v = false,
               double knot_delta_u = 1.0, double knot_delta_v = 1.0);
    
    /// Create clamped uniform NURBS surface
    bool create_clamped_uniform(int dimension,
                               int order0, int order1,
                               int cv_count0, int cv_count1,
                               double knot_delta0 = 1.0,
                               double knot_delta1 = 1.0);
    
    /// Deallocate all memory and reset to empty state
    void destroy();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Validation
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Check if NURBS surface is valid
    bool is_valid() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Accessors
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Get dimension
    int dimension() const { return m_dim; }
    
    /// Check if surface is rational
    bool is_rational() const { return m_is_rat != 0; }
    
    /// Get order (degree + 1) in specified direction
    int order(int dir) const;
    
    /// Get degree (order - 1) in specified direction
    int degree(int dir) const;
    
    /// Get number of control vertices in specified direction
    int cv_count(int dir) const;
    
    /// Get total number of control vertices
    int cv_count() const;
    
    /// Get size of each control vertex (dimension + 1 if rational, else dimension)
    int cv_size() const;
    
    /// Get knot count in specified direction
    int knot_count(int dir) const;
    
    /// Get number of spans in specified direction
    int span_count(int dir) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Control Vertex Access
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Get pointer to CV data at indices (i, j)
    double* cv(int i, int j);
    const double* cv(int i, int j) const;
    
    /// Get control point as Point
    Point get_cv(int i, int j) const;
    
    /// Get control point as homogeneous coordinates (x, y, z, w)
    bool get_cv_4d(int i, int j, double& x, double& y, double& z, double& w) const;
    
    /// Set control point from Point
    bool set_cv(int i, int j, const Point& point);
    
    /// Set control point from homogeneous coordinates
    bool set_cv_4d(int i, int j, double x, double y, double z, double w);
    
    /// Get weight at control vertex index
    double weight(int i, int j) const;
    
    /// Set weight at control vertex index
    bool set_weight(int i, int j, double weight);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Knot Access
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Get knot value at index in specified direction
    double knot(int dir, int knot_index) const;
    
    /// Set knot value at index in specified direction
    bool set_knot(int dir, int knot_index, double knot_value);
    
    /// Get knot multiplicity at index in specified direction
    int knot_multiplicity(int dir, int knot_index) const;
    
    /// Get all knot values for specified direction
    std::vector<double> get_knots(int dir) const;
    
    /// Check if knot vector is valid in specified direction
    bool is_valid_knot_vector(int dir) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Domain & Parameterization
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Get surface domain [start_param, end_param] in specified direction
    std::pair<double, double> domain(int dir) const;
    
    /// Set surface domain in specified direction
    bool set_domain(int dir, double t0, double t1);
    
    /// Get span (distinct knot intervals) values in specified direction
    std::vector<double> get_span_vector(int dir) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Geometric Queries
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Check if surface is closed in specified direction
    bool is_closed(int dir) const;
    
    /// Check if surface is periodic in specified direction
    bool is_periodic(int dir) const;
    
    /// Check if surface is planar within tolerance
    bool is_planar(Plane* plane = nullptr, double tolerance = Tolerance::ZERO_TOLERANCE) const;
    
    /// Check if surface side is singular (collapsed to a point)
    /// side: 0=south, 1=east, 2=north, 3=west
    bool is_singular(int side) const;

    /// Check if surface is clamped in specified direction
    bool is_clamped(int dir, int end = 2) const;

    /// Subdivide surface into a grid of points and parameters
    std::pair<std::vector<std::vector<Point>>, std::vector<std::vector<std::pair<double,double>>>>
        divide_by_count(int nu, int nv) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Knot Vector Operations
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Make knot vector a clamped uniform knot vector
    bool make_clamped_uniform_knot_vector(int dir, double delta = 1.0);
    
    /// Make knot vector a periodic uniform knot vector
    bool make_periodic_uniform_knot_vector(int dir, double delta = 1.0);
    
    /// Insert knot in specified direction
    bool insert_knot(int dir, double knot_value, int knot_multiplicity = 1);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Evaluation
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Evaluate point on surface at parameter (u, v)
    Point point_at(double u, double v) const;

    /// Fast evaluation — write xyz to output doubles (no Point construction)
    void point_at(double u, double v, double& x, double& y, double& z) const;

    /// Evaluate point and derivatives on surface
    /// Returns array of [point, du, dv, duu, duv, dvv, ...]
    std::vector<Vector> evaluate(double u, double v, int num_derivs = 0) const;

    /// Get normal vector at parameter (u, v)
    Vector normal_at(double u, double v) const;

    /// Fast normal — write xyz to output doubles (no Vector construction)
    void normal_at(double u, double v, double& nx, double& ny, double& nz) const;

    /// Combined fast point + normal (single span lookup + basis computation)
    void point_and_normal_at(double u, double v,
                             double& px, double& py, double& pz,
                             double& nx, double& ny, double& nz) const;
    
    /// Get point at corner (u_end, v_end) where end is 0 or 1
    Point point_at_corner(int u_end, int v_end) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Modification Operations
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Reverse surface direction
    bool reverse(int dir);
    
    /// Transpose surface (swap u and v parameters)
    bool transpose();
    
    /// Swap two coordinate axes
    bool swap_coordinates(int axis_i, int axis_j);
    
    /// Trim surface to sub-domain in specified direction
    bool trim(int dir, const std::pair<double, double>& domain);
    
    /// Split surface at parameter in specified direction  
    bool split(int dir, double c, NurbsSurface*& west_or_south_side, NurbsSurface*& east_or_north_side) const;
    
    /// Extend surface to include domain in specified direction
    bool extend(int dir, const std::pair<double, double>& domain);
    
    /// Make surface rational (if not already)
    bool make_rational();
    
    /// Make surface non-rational if all weights are equal
    bool make_non_rational();
    
    /// Clamp end in specified direction
    bool clamp_end(int dir, int end);
    
    /// Increase degree in specified direction
    bool increase_degree(int dir, int desired_degree);
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Apply stored xform transformation (in-place)
    void transform();
    
    /// Apply custom transformation matrix (in-place)
    bool transform(const Xform& xform);
    
    /// Get transformed copy
    NurbsSurface transformed() const;
    
    /// Get transformed copy with custom xform
    NurbsSurface transformed(const Xform& xform) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Geometric Operations
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Get bounding box
    BoundingBox get_bounding_box() const;
    
    /// Get surface area (approximate)
    double area(double tolerance = 1e-6) const;
    
    /// Get isoparametric curve at parameter
    /// dir: 0=iso-u curve (v varies), 1=iso-v curve (u varies)
    /// Returns: Pointer to NurbsCurve (caller must delete)
    NurbsCurve* iso_curve(int dir, double c) const;
    
    /// Closest point on surface to test point
    Point closest_point(const Point& point, double& u_out, double& v_out) const;

    /// Set the outer trim loop (2D curve in UV space)
    void set_outer_loop(const NurbsCurve& loop);

    /// Get the outer trim loop
    NurbsCurve get_outer_loop() const;

    /// Check if surface is trimmed
    bool is_trimmed() const;

    /// Clear the outer trim loop
    void clear_outer_loop();

    /// Add an inner trim loop (hole) from 2D UV curve
    void add_inner_loop(const NurbsCurve& loop);

    /// Add a hole by projecting a 3D curve onto the surface UV space
    void add_hole(const NurbsCurve& curve_3d);

    /// Add multiple holes
    void add_holes(const std::vector<NurbsCurve>& curves_3d);

    /// Get inner trim loop at index
    NurbsCurve get_inner_loop(int index) const;

    /// Get number of inner trim loops
    int inner_loop_count() const;

    /// Clear all inner trim loops
    void clear_inner_loops();

    /// Compute mesh via TrimeshGrid (UV parameter-space grid meshing)
    Mesh mesh_grid(double max_angle = 20.0, double max_edge_length = 0.0,
                   double min_edge_length = 0.0, double max_chord_height = 0.0) const;

    /// Compute mesh via TrimeshDelaunay (constrained Delaunay with refinement)
    Mesh mesh_delaunay(double max_angle = 20.0, double max_edge_length = 0.0,
                       double min_edge_length = 0.0, double max_chord_height = 0.0) const;

    /// Compute mesh — auto-selects grid for untrimmed, Delaunay for trimmed
    Mesh mesh(double max_angle = 20.0, double max_edge_length = 0.0,
              double min_edge_length = 0.0, double max_chord_height = 0.0) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Convert to JSON
    nlohmann::ordered_json jsondump() const;
    
    /// Load from JSON
    static NurbsSurface jsonload(const nlohmann::json& data);
    
    /// Write JSON to file
    void json_dump(const std::string& filename) const;

    /// Read JSON from file
    static NurbsSurface json_load(const std::string& filename);

    /// Convert to JSON string
    std::string json_dumps() const;

    /// Load from JSON string
    static NurbsSurface json_loads(const std::string& json_string);

    /// Convert to protobuf binary string
    std::string pb_dumps() const;

    /// Load from protobuf binary string
    static NurbsSurface pb_loads(const std::string& data);

    /// Write protobuf to file
    void pb_dump(const std::string& filename) const;

    /// Read protobuf from file
    static NurbsSurface pb_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Simple string representation (like Python str)
    std::string str() const;

    /// Detailed representation (like Python repr)
    std::string repr() const;

    /// Stream output operator
    friend std::ostream& operator<<(std::ostream& os, const NurbsSurface& surface);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Advanced Operations
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Zero all control vertices (set weights to 1 if rational)
    bool zero_cvs();
    
    /// Check if this surface is duplicate of another
    bool is_duplicate(const NurbsSurface& other,
                     bool ignore_parameterization,
                     double tolerance = Tolerance::ZERO_TOLERANCE) const;
    
    /// Collapse side of surface to a point
    /// side: 0=SW, 1=SE, 2=NE, 3=NW
    bool collapse_side(int side, const Point& point);

private:
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Internal Helpers
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Deep copy from another NurbsSurface
    void deep_copy_from(const NurbsSurface& src);
    
    /// Find span index for parameter value
    int find_span(int dir, double t) const;
    
    /// Compute basis functions
    void basis_functions(int dir, int span, double t, std::vector<double>& basis) const;
    
    /// Compute basis functions and derivatives
    void basis_functions_derivatives(int dir, int span, double t, int deriv_order,
                                   std::vector<std::vector<double>>& ders) const;
};

} // namespace session_cpp
