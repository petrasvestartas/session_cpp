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

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Modification Operations
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Reverse curve direction (negates knots, reverses CVs)
    bool reverse();
    
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
