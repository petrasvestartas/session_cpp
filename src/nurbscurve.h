#pragma once

#include "point.h"
#include "vector.h"
#include "plane.h"
#include "xform.h"
#include "color.h"
#include "tolerance.h"
#include "guid.h"
#include "json.h"
#include "knot.h"
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <array>
#include <tuple>

namespace session_cpp {

/**
 * @class NurbsCurve
 * @brief Non-Uniform Rational B-Spline (NURBS) curve implementation
 * 
 * A NURBS curve is defined by:
 * - Control points (CVs)
 * - Knot vector
 * - Degree (order = degree + 1)
 * - Optional weights for rational curves
 * 
 * Based on OpenNURBS implementation, adapted for session_cpp data types.
 */
class NurbsCurve {
public:
    std::string guid = ::guid();
    std::string name = "my_nurbscurve";
    double width = 1.0;
    std::vector<Color> pointcolors;
    std::vector<Color> linecolors;
    Xform xform = Xform::identity();

    // Core NURBS data
    int m_dim;              // Dimension (typically 3 for 3D curves)
    int m_is_rat;           // 1 if rational, 0 if non-rational
    int m_order;            // Order = degree + 1 (order >= 2)
    int m_cv_count;         // Number of control vertices (>= order)
    int m_cv_stride;        // Stride between control vertices in m_cv array
    int m_cv_capacity;      // Capacity of m_cv array
    
    std::vector<double> m_knot;  // Knot vector (length = m_order + m_cv_count - 2)
    std::vector<double> m_cv;    // Control vertex data (homogeneous if rational)

public:
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Static Factory Methods (RhinoCommon-style API)
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Create NURBS curve from points (RhinoCommon-style unified API)
    /// Equivalent to: NurbsCurve.Create(bool periodic, int degree, IEnumerable<Point3d> points)
    static NurbsCurve create(bool periodic, int degree, const std::vector<Point>& points,
                           int dimension = 3, double knot_delta = 1.0);

    /// Create an interpolated cubic NURBS curve through points (Bessel end tangents).
    static NurbsCurve create_interpolated(const std::vector<Point>& points,
                                          CurveKnotStyle parameterization = CurveKnotStyle::Chord);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors & Destructor
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Default constructor
    NurbsCurve();
    
    /// Constructor with dimension, rationality, order, and CV count
    NurbsCurve(int dimension, bool is_rational, int order, int cv_count);
    
    /// Copy constructor
    NurbsCurve(const NurbsCurve& other);
    
    /// Copy assignment operator
    NurbsCurve& operator=(const NurbsCurve& other);

    /// Equality operator
    bool operator==(const NurbsCurve& other) const;
    bool operator!=(const NurbsCurve& other) const;

    /// Destructor
    ~NurbsCurve();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Initialization & Creation
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Initialize all fields to zero/empty
    void initialize();

    /// Create NURBS curve with specified parameters
    bool create(int dimension, bool is_rational, int order, int cv_count);

    /// Create clamped uniform NURBS curve from control points
    bool create_clamped_uniform(int dimension, int order,
                               const std::vector<Point>& points,
                               double knot_delta = 1.0);

    /// Create periodic uniform NURBS curve from control points
    bool create_periodic_uniform(int dimension, int order,
                                const std::vector<Point>& points,
                                double knot_delta = 1.0);

    /// Deallocate all memory and reset to empty state
    void destroy();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Boolean Queries
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Check if NURBS curve is valid
    bool is_valid() const;

    /// Check if curve is rational
    bool is_rational() const { return m_is_rat != 0; }

    /// Check if curve is closed (start point == end point)
    bool is_closed() const;

    /// Check if curve is periodic (wraps around seamlessly)
    bool is_periodic() const;

    /// Check if curve is a straight line within tolerance
    bool is_linear(double tolerance = Tolerance::ZERO_TOLERANCE) const;

    /// Check if curve is planar within tolerance
    bool is_planar(Plane* plane = nullptr, double tolerance = Tolerance::ZERO_TOLERANCE) const;

    /// Check if curve is an arc (with optional arc output)
    bool is_arc(Plane* plane = nullptr, double tolerance = Tolerance::ZERO_TOLERANCE) const;

    /// Check if curve lies in a specific plane
    bool is_in_plane(const Plane& test_plane, double tolerance = Tolerance::ZERO_TOLERANCE) const;

    /// Test if curve has natural end (zero 2nd derivative)
    bool is_natural(int end = 2) const;

    /// Check if curve can be represented as a polyline
    int is_polyline(std::vector<Point>* points = nullptr,
                   std::vector<double>* params = nullptr) const;

    /// Check if entire curve is singular (all spans collapsed to points)
    bool is_singular() const;

    /// Check if this curve is duplicate of another
    bool is_duplicate(const NurbsCurve& other,
                     bool ignore_parameterization,
                     double tolerance = Tolerance::ZERO_TOLERANCE) const;

    /// Test continuity at parameter
    bool is_continuous(int continuity_type,
                      double t,
                      int* hint = nullptr,
                      double point_tolerance = Tolerance::ZERO_TOLERANCE,
                      double d1_tolerance = Tolerance::ZERO_TOLERANCE,
                      double d2_tolerance = Tolerance::ZERO_TOLERANCE,
                      double cos_angle_tolerance = 0.99984769515639123,
                      double curvature_tolerance = 1e-8) const;

    /// Check if knot vector is valid
    bool is_valid_knot_vector() const;

    /// Check if knot vector is clamped at ends
    bool is_clamped(int end = 2) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Accessors
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Get dimension
    int dimension() const { return m_dim; }
       
    /// Get order (degree + 1)
    int order() const { return m_order; }
    
    /// Get degree (order - 1)
    int degree() const { return m_order - 1; }
    
    /// Get number of control vertices
    int cv_count() const { return m_cv_count; }
    
    /// Get size of each control vertex (dimension + 1 if rational, else dimension)
    int cv_size() const;
    
    /// Get knot count (order + cv_count - 2)
    int knot_count() const;
    
    /// Get number of spans
    int span_count() const;
    
    /// Get CV capacity (allocated space)
    int cv_capacity() const { return m_cv_capacity; }
    
    /// Get knot capacity (from vector capacity)
    int knot_capacity() const { return static_cast<int>(m_knot.capacity()); }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Control Vertex Access
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Get pointer to CV data at index
    double* cv(int cv_index);
    const double* cv(int cv_index) const;

    /// Get control point at index as Point
    Point get_cv(int cv_index) const;

    /// Get control point at index as homogeneous point (x, y, z, w)
    bool get_cv_4d(int cv_index, double& x, double& y, double& z, double& w) const;
    std::tuple<double, double, double, double> get_cv_4d(int cv_index) const;

    /// Set control point at index from Point
    bool set_cv(int cv_index, const Point& point);

    /// Set control point at index from homogeneous coordinates
    bool set_cv_4d(int cv_index, double x, double y, double z, double w);

    /// Get weight at control vertex index (returns 1.0 if non-rational)
    double weight(int cv_index) const;

    /// Set weight at control vertex index (converts to rational if needed)
    bool set_weight(int cv_index, double weight);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Knot Access
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Get knot value at index
    double knot(int knot_index) const;

    /// Set knot value at index
    bool set_knot(int knot_index, double knot_value);

    /// Get knot multiplicity at index
    int knot_multiplicity(int knot_index) const;

    /// Get superfluous knot value at end
    double superfluous_knot(int end) const;

    /// Get pointer to knot array
    const double* knot_array() const { return m_knot.data(); }

    /// Get pointer to CV array (expert use)
    double* cv_array() { return m_cv.data(); }
    const double* cv_array() const { return m_cv.data(); }

    /// Get all knot values
    std::vector<double> get_knots() const { return m_knot; }

    /// Insert knot into curve (Boehm's algorithm)
    bool insert_knot(double knot_value, int knot_multiplicity = 1);

    /// Get Greville abcissa for a control point
    double greville_abcissa(int cv_index) const;

    /// Get all Greville abcissae
    bool get_greville_abcissae(std::vector<double>& abcissae) const;
    std::vector<double> get_greville_abcissae() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Domain & Parameterization
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Get curve domain [start_param, end_param]
    std::pair<double, double> domain() const;

    /// Get start domain
    double domain_start() const;

    /// Get end of domain
    double domain_end() const;

    /// Get domain at the middle
    double domain_middle() const;
    
    /// Set curve domain
    bool set_domain(double t0, double t1);
    
    /// Get span (distinct knot intervals) values
    std::vector<double> get_span_vector() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Geometric Queries
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Find next discontinuity in curve
    bool get_next_discontinuity(int continuity_type,
                               double t0, double t1,
                               double& t_out,
                               int* hint = nullptr,
                               double cos_angle_tolerance = 0.99984769515639123,
                               double curvature_tolerance = 1e-8) const;
    std::pair<bool, double> get_next_discontinuity(int continuity_type, double t0, double t1) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Conversion methods
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Get length method that is used by division methods
    double length(double tolerance = 1e-6) const;
    /// Convert curve to polyline with adaptive sampling (curvature-based)
    /// Returns points and optionally parameters
    /// angle_tolerance: maximum angle between segments (radians)
    /// min_edge_length: minimum distance between points
    /// max_edge_length: maximum distance between points
    bool to_polyline_adaptive(std::vector<Point>& points,
                             std::vector<double>* params = nullptr,
                             double angle_tolerance = 0.1,
                             double min_edge_length = 0.0,
                             double max_edge_length = 0.0) const;
    std::pair<std::vector<Point>, std::vector<double>> to_polyline_adaptive(
        double angle_tolerance = 0.1, double min_edge_length = 0.0, double max_edge_length = 0.0) const;
    
    /// Divide curve into uniform number of points (simple linear parameter division)
    /// count: number of points (must be >= 2)
    /// include_endpoints: if true, first and last points are at curve start/end
    bool divide_by_count(int count, std::vector<Point>& points,
                        std::vector<double>* params = nullptr,
                        bool include_endpoints = true) const;
    std::pair<std::vector<Point>, std::vector<double>> divide_by_count(int count, bool include_endpoints = true) const;
    
    /// Divide curve by approximate arc length
    /// segment_length: target length between points
    bool divide_by_length(double segment_length, std::vector<Point>& points,
                         std::vector<double>* params = nullptr) const;
    std::pair<std::vector<Point>, std::vector<double>> divide_by_length(double segment_length) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Evaluation
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Evaluate point on curve at parameter t
    Point point_at(double t) const;
    
    /// Evaluate point and derivatives on curve at parameter t
    /// Returns array of [point, 1st_derivative, 2nd_derivative, ...]
    std::vector<Vector> evaluate(double t, int derivative_count = 0) const;
    
    /// Get tangent vector at parameter t
    Vector tangent_at(double t) const;

    /// Get Frenet frame at parameter t (tangent, normal, binormal)
    Plane plane_at(double t, bool normalized) const;

    /// Get rotation minimizing frame at parameter t (Double Reflection Method, Wang et al. 2008)
    Plane perpendicular_plane_at(double t, bool normalized) const;

    /// Get multiple rotation minimizing frames along the curve
    /// count = number of subdivisions (returns count+1 frames at arc-length equidistant points)
    std::vector<Plane> get_perpendicular_planes(int count) const;

    
    /// Get start point of curve
    Point point_at_start() const;

    /// Get start point of curve
    Point point_at_middle() const;
    
    /// Get end point of curve
    Point point_at_end() const;
    
    /// Force curve to start at a specified point
    bool set_start_point(const Point& start_point);
    
    /// Force curve to end at a specified point
    bool set_end_point(const Point& end_point);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Modification Operations
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Reverse curve direction (negates knots, reverses CVs)
    bool reverse();
    
    /// Swap two coordinate axes (e.g., swap X and Y)
    bool swap_coordinates(int axis_i, int axis_j);
    
    /// Trim curve to interval [t0, t1]
    bool trim(double t0, double t1);
    
    /// Split curve at parameter t into left and right parts
    bool split(double t, NurbsCurve& left_curve, NurbsCurve& right_curve) const;
    std::pair<NurbsCurve, NurbsCurve> split(double t) const;
    
    /// Extend curve to include domain (natural NURBS extrapolation using De Boor)
    bool extend(double t0, double t1);
    
    /// Make curve rational (if not already)
    bool make_rational();
    
    /// Make curve non-rational. If force=false, fails when weights differ.
    /// If force=true, sets all weights to 1.0 (changes geometry!)
    bool make_non_rational(bool force = false);
    
    /// Clamp ends (add multiplicity to end knots)
    bool clamp_end(int end); // 0 = start, 1 = end, 2 = both

    /// Increase degree of curve
    bool increase_degree(int desired_degree);

    /// Change seam point of closed periodic curve
    bool change_closed_curve_seam(double t);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Apply stored xform transformation (in-place)
    void transform();
    
    /// Apply custom transformation matrix (in-place)
    bool transform(const Xform& xform);
    
    /// Get transformed copy
    NurbsCurve transformed() const;
    
    /// Get transformed copy with custom xform
    NurbsCurve transformed(const Xform& xform) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Convert to JSON object
    nlohmann::ordered_json jsondump() const;

    /// Load from JSON object
    static NurbsCurve jsonload(const nlohmann::json& data);

    /// Convert to JSON string
    std::string json_dumps() const;

    /// Load from JSON string
    static NurbsCurve json_loads(const std::string& json_string);

    /// Write JSON to file
    void json_dump(const std::string& filename) const;

    /// Read JSON from file
    static NurbsCurve json_load(const std::string& filename);

    /// Convert to protobuf binary string
    std::string pb_dumps() const;

    /// Load from protobuf binary string
    static NurbsCurve pb_loads(const std::string& data);

    /// Write protobuf to file
    void pb_dump(const std::string& filename) const;

    /// Read protobuf from file
    static NurbsCurve pb_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Simple string representation (like Python str)
    std::string str() const;

    /// Detailed representation (like Python repr)
    std::string repr() const;

private:
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Internal Helpers
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Reserve capacity for control vertices
    bool reserve_cv_capacity(int capacity);

    /// Reserve capacity for knots
    bool reserve_knot_capacity(int capacity);

    /// Clean up invalid knots (remove duplicates, fix spacing issues)
    bool clean_knots(double knot_tolerance = 0.0);

    /// Check if span is linear within tolerance
    bool span_is_linear(int span_index, double min_length, double tolerance) const;

    /// Check if span is linear and get the line
    bool span_is_linear(int span_index, double min_length, double tolerance,
                       Point* line_start, Point* line_end) const;

    /// Check if span is singular (collapsed to a point)
    bool span_is_singular(int span_index) const;

    /// Repair bad knots (too close, high multiplicity)
    bool repair_bad_knots(double knot_tolerance = 0.0, bool repair = true);

    /// Get parameter tolerance at point
    bool get_parameter_tolerance(double t, double* tminus, double* tplus) const;

    /// Zero all control vertices (and set weights to 1 if rational)
    bool zero_cvs();

    /// Find knot span index for parameter t (binary search)
    int find_span(double t) const;

    /// Compute basis functions at parameter t
    void basis_functions(int span, double t, std::vector<double>& basis) const;

    /// Compute basis functions and derivatives
    void basis_functions_derivatives(int span, double t, int deriv_order,
                                    std::vector<std::vector<double>>& ders) const;

    /// Deep copy from another NurbsCurve
    void deep_copy_from(const NurbsCurve& src);

    /// De Boor algorithm for trimming/extending B-spline spans
    /// Based on OpenNURBS ON_EvaluateNurbsDeBoor
    /// side: -1 = left side, +1 = right side
    /// Returns false if knot[order-2] == knot[order-1]
    static bool evaluate_nurbs_de_boor(int cv_dim, int order, int cv_stride,
                                       double* cv, const double* knot,
                                       int side, double t);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // RMF Cache (Rotation Minimizing Frames with Quaternion SLERP)
    ///////////////////////////////////////////////////////////////////////////////////////////

    mutable bool m_rmf_cached = false;
    mutable std::vector<double> m_rmf_params;
    mutable std::vector<std::array<double, 4>> m_rmf_quaternions;  // [w, x, y, z]
    mutable std::vector<Point> m_rmf_origins;

    void invalidate_rmf_cache() const;
    void ensure_rmf_cache() const;
    static std::array<double, 4> frame_to_quaternion(const Vector& r, const Vector& s, const Vector& t);
    static void quaternion_to_frame(const std::array<double, 4>& q, Vector& r, Vector& s, Vector& t);
    static std::array<double, 4> slerp(const std::array<double, 4>& q0, const std::array<double, 4>& q1, double u);
};

///////////////////////////////////////////////////////////////////////////////////////////
// Stream operator
///////////////////////////////////////////////////////////////////////////////////////////

std::ostream& operator<<(std::ostream& os, const NurbsCurve& curve);

} // namespace session_cpp

// fmt formatter specialization
template <>
struct fmt::formatter<session_cpp::NurbsCurve> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }
    
    auto format(const session_cpp::NurbsCurve& curve, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", curve.str());
    }
};
