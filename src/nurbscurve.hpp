#pragma once

#include "point.h"
#include "vector.h"
#include "plane.h"
#include "xform.h"
#include "color.h"
#include "tolerance.h"
#include "boundingbox.h"
#include "guid.h"
#include "json.h"
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <stdexcept>

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
    Color linecolor = Color::white();
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
    // Validation
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Check if NURBS curve is valid
    bool is_valid() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Accessors
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Get dimension
    int dimension() const { return m_dim; }
    
    /// Check if curve is rational
    bool is_rational() const { return m_is_rat != 0; }
    
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
    
    /// Check if knot vector is valid
    bool is_valid_knot_vector() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Domain & Parameterization
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Get curve domain [start_param, end_param]
    std::pair<double, double> domain() const;
    
    /// Set curve domain
    bool set_domain(double t0, double t1);
    
    /// Get span (distinct knot intervals) values
    std::vector<double> get_span_vector() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Geometric Queries
    ///////////////////////////////////////////////////////////////////////////////////////////
    
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
    
    /// Divide curve into uniform number of points (simple linear parameter division)
    /// count: number of points (must be >= 2)
    /// include_endpoints: if true, first and last points are at curve start/end
    bool divide_by_count(int count, std::vector<Point>& points,
                        std::vector<double>* params = nullptr,
                        bool include_endpoints = true) const;
    
    /// Divide curve by approximate arc length
    /// segment_length: target length between points
    bool divide_by_length(double segment_length, std::vector<Point>& points,
                         std::vector<double>* params = nullptr) const;

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
    
    /// Get start point of curve
    Point point_at_start() const;
    
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
    
    /// Extend curve to include domain
    bool extend(double t0, double t1);
    
    /// Make curve rational (if not already)
    bool make_rational();
    
    /// Make curve non-rational if all weights are equal
    bool make_non_rational();
    
    /// Clamp ends (add multiplicity to end knots)
    bool clamp_end(int end); // 0 = start, 1 = end, 2 = both

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
    // Geometric Operations
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Get bounding box
    BoundingBox get_bounding_box() const;
    
    /// Get length of curve (approximate)
    double length(double tolerance = 1e-6) const;
    
    /// Closest point on curve to given point
    Point closest_point(const Point& point, double& t_out) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Convert to JSON
    nlohmann::ordered_json jsondump() const;
    
    /// Load from JSON
    static NurbsCurve jsonload(const nlohmann::json& data);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representation
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Convert to string
    std::string to_string() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Helper Functions
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Reserve capacity for control vertices
    bool reserve_cv_capacity(int capacity);
    
    /// Reserve capacity for knots
    bool reserve_knot_capacity(int capacity);
    
    /// Clean up invalid knots (remove duplicates, fix spacing issues)
    bool clean_knots(double knot_tolerance = 0.0);
    
    /// Make knot vector a clamped uniform knot vector (does not change CVs)
    bool make_clamped_uniform_knot_vector(double delta = 1.0);
    
    /// Make knot vector a periodic uniform knot vector (does not change CVs)
    bool make_periodic_uniform_knot_vector(double delta = 1.0);
    
    /// Check if knot vector is clamped at ends
    bool is_clamped(int end = 2) const;
    
    /// Get length of control polygon
    double control_polygon_length() const;
    
    /// Get Greville abcissa for a control point
    double greville_abcissa(int cv_index) const;
    
    /// Get all Greville abcissae
    bool get_greville_abcissae(std::vector<double>& abcissae) const;
    
    /// Check if span is linear within tolerance
    bool span_is_linear(int span_index, double min_length, double tolerance) const;
    
    /// Check if span is linear and get the line
    bool span_is_linear(int span_index, double min_length, double tolerance,
                       Point* line_start, Point* line_end) const;
    
    /// Check if span is singular (collapsed to a point)
    bool span_is_singular(int span_index) const;
    
    /// Check if entire curve is singular
    bool is_singular() const;
    
    /// Check if curve has bezier spans (all distinct knots have multiplicity = degree)
    bool has_bezier_spans() const;
    
    /// Convert a span to Bezier curve (extract individual Bezier segment)
    bool convert_span_to_bezier(int span_index, std::vector<Point>& bezier_cvs) const;
    
    /// Remove a specific span from the curve
    bool remove_span(int span_index);
    
    /// Remove all singular spans from curve
    int remove_singular_spans();
    
    /// Get cubic Bezier approximation of entire curve
    double get_cubic_bezier_approximation(double max_deviation, std::vector<Point>& bezier_cvs) const;
    
    /// Repair bad knots (too close, high multiplicity)
    bool repair_bad_knots(double knot_tolerance = 0.0, bool repair = true);
    
    /// Make curve have piecewise bezier spans
    bool make_piecewise_bezier(bool set_end_weights_to_one = false);
    
    /// Increase degree of curve
    bool increase_degree(int desired_degree);
    
    /// Get NURBS form (returns 1 for NURBS curve)
    int get_nurbs_form(NurbsCurve& nurbs_form, double tolerance = 0.0) const;
    
    /// Check if has NURBS form (always returns 1 for NURBS curve)
    int has_nurbs_form() const;
    
    /// Append another NURBS curve to this one
    bool append(const NurbsCurve& other);
    
    /// Change dimension of curve
    bool change_dimension(int desired_dimension);
    
    /// Change seam point of closed periodic curve
    bool change_closed_curve_seam(double t);
    
    /// Get parameter tolerance at point
    bool get_parameter_tolerance(double t, double* tminus, double* tplus) const;
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Intersection Operations
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Find all intersections between curve and plane
    /// Returns parameter values where curve intersects plane
    std::vector<double> intersect_plane(const Plane& plane, double tolerance = Tolerance::ZERO_TOLERANCE) const;
    
    /// Find all intersection points between curve and plane
    std::vector<Point> intersect_plane_points(const Plane& plane, double tolerance = Tolerance::ZERO_TOLERANCE) const;
    
    /// Find closest point on curve to test point (improved with Newton-Raphson)
    std::pair<double, double> closest_point_to(const Point& test_point, 
                                               double t0 = 0.0, double t1 = 0.0) const;
    
    /// Curve-plane intersection using Bézier clipping (advanced, faster for multiple intersections)
    std::vector<double> intersect_plane_bezier_clipping(const Plane& plane, 
                                                        double tolerance = Tolerance::ZERO_TOLERANCE) const;
    
    /// Curve-plane intersection using algebraic/hodograph method (maximum precision)
    /// Uses span-based subdivision + Newton-Raphson with derivative information
    /// Achieves quadratic convergence (machine precision in 2-3 iterations)
    std::vector<double> intersect_plane_algebraic(const Plane& plane,
                                                  double tolerance = Tolerance::ZERO_TOLERANCE) const;
    
    /// Curve-plane intersection using production CAD kernel method (INDUSTRY STANDARD)
    /// This is the method used in Rhino, Parasolid, ACIS, and other professional CAD kernels
    /// Algorithm: Recursive subdivision on Bézier spans + Newton-Raphson refinement
    /// - Converts curve to Bézier spans
    /// - Recursively subdivides until nearly linear
    /// - Detects sign changes (signed distance to plane)
    /// - Applies Newton-Raphson for quadratic convergence
    /// - Robust bracketing with fallback bisection
    /// - Filters duplicates and returns sorted results
    /// Performance: O(log n) subdivision + O(1) Newton per root
    std::vector<double> intersect_plane_production(const Plane& plane,
                                                   double tolerance = Tolerance::ZERO_TOLERANCE) const;
    
    /// Zero all control vertices (and set weights to 1 if rational)
    bool zero_cvs();
    
    /// Check if this curve is duplicate of another
    bool is_duplicate(const NurbsCurve& other,
                     bool ignore_parameterization,
                     double tolerance = Tolerance::ZERO_TOLERANCE) const;
    
    /// Get span indices where control point is active
    std::pair<int, int> control_point_spans(int cv_index) const;
    
    /// Get parameter interval where control point is active
    std::pair<double, double> control_point_support(int cv_index) const;
    
    /// Find next discontinuity in curve
    bool get_next_discontinuity(int continuity_type,
                               double t0, double t1,
                               double& t_out,
                               int* hint = nullptr,
                               double cos_angle_tolerance = 0.99984769515639123,
                               double curvature_tolerance = 1e-8) const;
    
    /// Test continuity at parameter
    bool is_continuous(int continuity_type,
                      double t,
                      int* hint = nullptr,
                      double point_tolerance = Tolerance::ZERO_TOLERANCE,
                      double d1_tolerance = Tolerance::ZERO_TOLERANCE,
                      double d2_tolerance = Tolerance::ZERO_TOLERANCE,
                      double cos_angle_tolerance = 0.99984769515639123,
                      double curvature_tolerance = 1e-8) const;
    
    /// Reparameterize rational curve by linear fractional transformation
    bool reparameterize(double c);
    
    /// Change end weights of rational curve
    bool change_end_weights(double w0, double w1);

private:
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Internal Helpers
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Find knot span index for parameter t (binary search)
    int find_span(double t) const;
    
    /// Compute basis functions at parameter t
    void basis_functions(int span, double t, std::vector<double>& basis) const;
    
    /// Compute basis functions and derivatives
    void basis_functions_derivatives(int span, double t, int deriv_order,
                                    std::vector<std::vector<double>>& ders) const;
    
    bool insert_knot(double knot_value, int knot_multiplicity);

    /// Deep copy from another NurbsCurve
    void deep_copy_from(const NurbsCurve& src);
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
        return fmt::format_to(ctx.out(), "{}", curve.to_string());
    }
};
