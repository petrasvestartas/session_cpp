#pragma once
#include "color.h"
#include "guid.h"
#include "json.h"
#include "line.h"
#include "nurbsknot.h"
#include "plane.h"
#include "point.h"
#include "tolerance.h"
#include "vector.h"
#include "xform.h"
#include "fmt/core.h"
#include <algorithm>
#include <cmath>
#include <array>
#include <ostream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace session_proto {
class NurbsCurve;
}

namespace session_cpp {

/// A NURBS curve: OpenNURBS layout, nurbsknot count = order + cv_count - 2, homogeneous CVs when rational.
class NurbsCurve {
private:
    mutable std::string _guid; // Lazily minted GUID.

public:
    std::string name = "my_nurbscurve"; // Curve name.
    double width = 1.0; // Display width.
    std::vector<Color> pointcolors; // Display color per control point.
    std::vector<Color> linecolors; // Display color per control polygon segment.
    Arrowhead arrowhead = Arrowhead::NONE; // Arrowhead ends.
    int m_dim; // Coordinate dimension.
    int m_is_rat; // 1 when rational, 0 otherwise.
    int m_order; // Degree + 1.
    int m_cv_count; // Number of control vertices.
    int m_cv_stride; // Doubles between consecutive CVs.
    std::vector<double> m_nurbsknot; // NurbsKnot vector, order + cv_count - 2 values.
    std::vector<double> m_cv; // Flat CV array, homogeneous when rational.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty curve.
    NurbsCurve();

    /// Construct an unset curve with the given layout.
    NurbsCurve(int dimension, bool is_rational, int order, int cv_count);

    /// Copy with a new guid and the same data.
    NurbsCurve(const NurbsCurve& other);

    /// Copy-assign with a new guid and the same data.
    NurbsCurve& operator=(const NurbsCurve& other);

    /// Move while preserving the guid.
    NurbsCurve(NurbsCurve&& other) noexcept = default;

    /// Move-assign while preserving the guid.
    NurbsCurve& operator=(NurbsCurve&& other) noexcept = default;

    /// Destroy the curve.
    ~NurbsCurve();

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct a clamped or periodic uniform curve through control points, domain rescaled to [0, arc length].
    static NurbsCurve create(bool periodic, int degree, const std::vector<Point>& points, int dimension = 3, double nurbsknot_delta = 1.0);

    /// Construct an interpolated cubic through points; Rhino (Bessel) or Occt (Lagrange) end tangents.
    static NurbsCurve create_interpolated(const std::vector<Point>& points, CurveNurbsKnotStyle parameterization = CurveNurbsKnotStyle::Chord, CurveInterpStyle end_condition = CurveInterpStyle::Rhino);

    /// Construct from poles, weights, distinct knots and multiplicities (OCCT convention).
    static NurbsCurve create_from_parameters(const std::vector<Point>& points, const std::vector<double>& weights, const std::vector<double>& knots, const std::vector<int>& mults, int degree, bool periodic = false);

    /// Construct a least-squares fit with num_cvs control points (Piegl & Tiller 9.4).
    static NurbsCurve create_fitted(const std::vector<Point>& points, int num_cvs, int degree = 3, bool is_periodic = false);

    /// Chain segments by endpoint matching, raise to a common degree and merge with C0 junctions.
    static std::vector<NurbsCurve> join(const std::vector<NurbsCurve>& curves, double tolerance = Tolerance::ZERO_TOLERANCE);

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Compare name, width, colors, arrowhead, layout, nurbsknots and CVs to 1e-12; guid ignored.
    bool operator==(const NurbsCurve& other) const;

    /// Compare name, width, colors, arrowhead, layout, nurbsknots and CVs to 1e-12; guid ignored.
    bool operator!=(const NurbsCurve& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform in place.
    bool transform(const Xform& xform);

    /// Return a transformed copy.
    NurbsCurve transformed(const Xform& xform) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Initialization
    // ═══════════════════════════════════════════════════════════════════════════
    /// Zero every field.
    void initialize();

    /// Allocate layout for dimension, rationality, order and cv_count.
    bool create(int dimension, bool is_rational, int order, int cv_count);

    /// Set clamped uniform nurbsknots over control points.
    bool create_clamped_uniform(int dimension, int order, const std::vector<Point>& points, double nurbsknot_delta = 1.0);

    /// Set periodic uniform nurbsknots over control points wrapped by order - 1.
    bool create_periodic_uniform(int dimension, int order, const std::vector<Point>& points, double nurbsknot_delta = 1.0);

    /// Reset to the empty state.
    void destroy();

    // ═══════════════════════════════════════════════════════════════════════════
    // Boolean queries
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the layout, nurbsknots and CVs are consistent.
    bool is_valid() const;

    /// Return whether the CVs carry weights.
    bool is_rational() const {
        return m_is_rat != 0;
    }

    /// Return whether the start point equals the end point.
    bool is_closed() const;

    /// Return whether the last degree CVs repeat the first and nurbsknots are uniform.
    bool is_periodic() const;

    /// Return whether every CV is within tolerance of the chord.
    bool is_linear(double tolerance = Tolerance::ZERO_TOLERANCE) const;

    /// Return whether every CV is within tolerance of one plane, written to plane when given.
    bool is_planar(Plane* plane = nullptr, double tolerance = Tolerance::ZERO_TOLERANCE) const;

    /// Return whether the curve is planar and equidistant from one center, plane written when given.
    bool is_arc(Plane* plane = nullptr, double tolerance = Tolerance::ZERO_TOLERANCE) const;

    /// Return whether every CV is within tolerance of test_plane.
    bool is_in_plane(const Plane& test_plane, double tolerance = Tolerance::ZERO_TOLERANCE) const;

    /// Return whether the second derivative is zero at end (0 = start, 1 = end, 2 = both).
    bool is_natural(int end = 2) const;

    /// Return the vertex count when every span is a line, else 0; vertices and params written when given.
    int is_polyline(std::vector<Point>* points = nullptr, std::vector<double>* params = nullptr) const;

    /// Return whether every span is collapsed to a point.
    bool is_singular() const;

    /// Return whether layout, CVs and weights match to tolerance, and nurbsknots unless ignore_parameterization.
    bool is_duplicate(const NurbsCurve& other, bool ignore_parameterization, double tolerance = Tolerance::ZERO_TOLERANCE) const;

    /// Return the continuity at t from nurbsknot multiplicity (0 = C0, 1 = C1, 2 = C2, 3 = G1, 4 = G2).
    bool is_continuous(int continuity_type, double t, int* hint = nullptr, double point_tolerance = Tolerance::ZERO_TOLERANCE, double d1_tolerance = Tolerance::ZERO_TOLERANCE, double d2_tolerance = Tolerance::ZERO_TOLERANCE, double cos_angle_tolerance = 0.99984769515639123, double curvature_tolerance = 1e-8) const;

    /// Return whether the nurbsknots have the right count, are non-decreasing and span a non-empty domain.
    bool is_valid_nurbsknot_vector() const;

    /// Return whether end has full multiplicity (0 = start, 1 = end, 2 = both).
    bool is_clamped(int end = 2) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    bool has_guid() const {
        return !_guid.empty();
    }

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

    /// Clear the guid so a fresh one mints lazily on the next read.
    void refresh_guid() {
        _guid.clear();
    }

    /// Return the coordinate dimension.
    int dimension() const {
        return m_dim;
    }

    /// Return the order (degree + 1).
    int order() const {
        return m_order;
    }

    /// Return the degree (order - 1).
    int degree() const {
        return m_order - 1;
    }

    /// Return the number of control vertices.
    int cv_count() const {
        return m_cv_count;
    }

    /// Return the doubles per CV: dimension + 1 when rational.
    int cv_size() const;

    /// Return order + cv_count - 2.
    int nurbsknot_count() const;

    /// Return the number of distinct nurbsknot intervals inside the domain.
    int span_count() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Control vertex access
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the mutable pointer to the CV doubles, nullptr when out of range.
    double* cv(int cv_index);

    /// Return the pointer to the CV doubles, nullptr when out of range.
    const double* cv(int cv_index) const;

    /// Return the Euclidean CV (divided by weight when rational).
    Point get_cv(int cv_index) const;

    /// Get the homogeneous CV (x, y, z, w) through out-parameters.
    bool get_cv_4d(int cv_index, double& x, double& y, double& z, double& w) const;

    /// Return the homogeneous CV (x, y, z, w).
    std::tuple<double, double, double, double> get_cv_4d(int cv_index) const;

    /// Set the CV from a point, weight reset to 1.
    bool set_cv(int cv_index, const Point& point);

    /// Set the homogeneous CV, making the curve rational when w != 1.
    bool set_cv_4d(int cv_index, double x, double y, double z, double w);

    /// Return the weight of a CV, 1 when non-rational.
    double weight(int cv_index) const;

    /// Set the weight, making the curve rational first.
    bool set_weight(int cv_index, double weight);

    // ═══════════════════════════════════════════════════════════════════════════
    // NurbsKnot access
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the nurbsknot at nurbsknot_index.
    double nurbsknot(int nurbsknot_index) const;

    /// Set the nurbsknot at nurbsknot_index.
    bool set_nurbsknot(int nurbsknot_index, double nurbsknot_value);

    /// Return the count of nurbsknots equal to the one at nurbsknot_index.
    int nurbsknot_multiplicity(int nurbsknot_index) const;

    /// Return the reflected end nurbsknot (0 = start, 1 = end).
    double superfluous_nurbsknot(int end) const;

    /// Return the nurbsknot array pointer.
    const double* nurbsknot_array() const {
        return m_nurbsknot.data();
    }

    /// Return the mutable CV array pointer.
    double* cv_array() {
        return m_cv.data();
    }

    /// Return the CV array pointer.
    const double* cv_array() const {
        return m_cv.data();
    }

    /// Return a copy of the nurbsknot vector.
    std::vector<double> get_nurbsknots() const {
        return m_nurbsknot;
    }

    /// Insert a nurbsknot by Boehm to the given multiplicity.
    bool insert_nurbsknot(double nurbsknot_value, int nurbsknot_multiplicity = 1);

    /// Return the Greville abcissa of a CV.
    double greville_abcissa(int cv_index) const;

    /// Get the Greville abcissae of every CV through an out-parameter.
    bool get_greville_abcissae(std::vector<double>& abcissae) const;

    /// Return the Greville abcissae of every CV.
    std::vector<double> get_greville_abcissae() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Domain
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the domain (t0, t1).
    std::pair<double, double> domain() const;

    /// Return the domain start.
    double domain_start() const;

    /// Return the domain end.
    double domain_end() const;

    /// Return the domain midpoint.
    double domain_middle() const;

    /// Rescale the nurbsknots to [t0, t1].
    bool set_domain(double t0, double t1);

    /// Return the distinct nurbsknot values inside the domain.
    std::vector<double> get_span_vector() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════
    /// Find the first interior nurbsknot in (t0, t1) whose multiplicity breaks continuity_type.
    bool get_next_discontinuity(int continuity_type, double t0, double t1, double& t_out, int* hint = nullptr, double cos_angle_tolerance = 0.99984769515639123, double curvature_tolerance = 1e-8) const;

    /// Return (found, t) for the first interior nurbsknot in (t0, t1) whose multiplicity breaks continuity_type.
    std::pair<bool, double> get_next_discontinuity(int continuity_type, double t0, double t1) const;

    /// Return the arc length by 10-point Gauss-Legendre over 4 subdivisions per span.
    double length(double tolerance = 1e-6) const;

    /// Compute the chord-deviation subdivision; angle_tolerance in radians, edge lengths default to length / 10 and / 1000.
    bool to_polyline_adaptive(std::vector<Point>& points, std::vector<double>* params = nullptr, double angle_tolerance = 0.1, double min_edge_length = 0.0, double max_edge_length = 0.0) const;

    /// Return the chord-deviation subdivision points and parameters.
    std::pair<std::vector<Point>, std::vector<double>> to_polyline_adaptive(double angle_tolerance = 0.1, double min_edge_length = 0.0, double max_edge_length = 0.0) const;

    /// Compute count points at equal arc length, ends included or excluded.
    bool divide_by_count(int count, std::vector<Point>& points, std::vector<double>* params = nullptr, bool include_endpoints = true) const;

    /// Return count points at equal arc length and their parameters.
    std::pair<std::vector<Point>, std::vector<double>> divide_by_count(int count, bool include_endpoints = true) const;

    /// Compute points every segment_length of arc length from the start.
    bool divide_by_length(double segment_length, std::vector<Point>& points, std::vector<double>* params = nullptr) const;

    /// Return points every segment_length of arc length and their parameters.
    std::pair<std::vector<Point>, std::vector<double>> divide_by_length(double segment_length) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Evaluation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the point at parameter t.
    Point point_at(double t) const;

    /// Return [point, first derivative, ..., derivative_count] with zeros past the degree.
    std::vector<Vector> evaluate(double t, int derivative_count = 0) const;

    /// Return the unit tangent by central difference.
    Vector tangent_at(double t) const;

    /// Return |C' x C''| / |C'|^3.
    double curvature_at(double t) const;

    /// Return the parameter of the closest point to test_point.
    double closest_parameter(const Point& test_point) const;

    /// Return the closest point to test_point.
    Point closest_point(const Point& test_point) const;

    /// Return the parameters (u, v) where this curve and other are closest.
    std::pair<double, double> closest_parameters_curve(const NurbsCurve& other) const;

    /// Return the points where this curve and other are closest.
    std::pair<Point, Point> closest_points_curve(const NurbsCurve& other) const;

    /// Return the Frenet frame (tangent, normal, binormal); normalized maps t from [0, 1].
    Plane plane_at(double t, bool normalized) const;

    /// Return the rotation minimizing frame by double reflection (Wang et al. 2008).
    Plane perpendicular_plane_at(double t, bool normalized) const;

    /// Return count + 1 rotation minimizing frames at equal arc length.
    std::vector<Plane> get_perpendicular_planes(int count) const;

    /// Return the point at the domain start.
    Point point_at_start() const;

    /// Return the point at the domain midpoint.
    Point point_at_middle() const;

    /// Return the point at the domain end.
    Point point_at_end() const;

    /// Clamp and move the first CV.
    bool set_start_point(const Point& start_point);

    /// Clamp and move the last CV.
    bool set_end_point(const Point& end_point);

    // ═══════════════════════════════════════════════════════════════════════════
    // Modifications
    // ═══════════════════════════════════════════════════════════════════════════
    /// Reverse the direction keeping the domain and swap the arrowhead ends.
    bool reverse();

    /// Swap two coordinate axes of every CV.
    bool swap_coordinates(int axis_i, int axis_j);

    /// Keep [t0, t1] by nurbsknot insertion.
    bool trim(double t0, double t1);

    /// Compute trimmed copies on both sides of t.
    bool split(double t, NurbsCurve& left_curve, NurbsCurve& right_curve) const;

    /// Return trimmed copies on both sides of t.
    std::pair<NurbsCurve, NurbsCurve> split(double t) const;

    /// Extrapolate the domain to cover [t0, t1] by de Boor.
    bool extend(double t0, double t1);

    /// Add unit weights.
    bool to_rational();

    /// Drop the weights; fails when they differ unless force.
    bool to_non_rational(bool force = false);

    /// Set full multiplicity at end (0 = start, 1 = end, 2 = both) with CVs adjusted.
    bool clamp_end(int end);

    /// Raise the degree by blossoming without changing the shape.
    bool increase_degree(int desired_degree);

    /// Move the seam of a closed curve to t.
    bool change_closed_curve_seam(double t);

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static NurbsCurve jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static NurbsCurve file_json_loads(const std::string& json_string);

    /// Write to a JSON file.
    void file_json_dump(const std::string& filename) const;

    /// Read from a JSON file.
    static NurbsCurve file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::NurbsCurve to_proto() const;

    /// Construct from the protobuf message.
    static NurbsCurve from_proto(const session_proto::NurbsCurve& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static NurbsCurve pb_loads(const std::string& data);

    /// Write to a protobuf file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file.
    static NurbsCurve pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "NurbsCurve(name=..., degree=..., cvs=...)".
    std::string str() const;

    /// Return the multi-line form with every control point.
    std::string repr() const;

private:
    /// Return whether the span has full end multiplicity and its CVs lie on its chord.
    bool span_is_linear(int span_index, double min_length, double tolerance) const;

    /// Return whether the span is collapsed to a point.
    bool span_is_singular(int span_index) const;

    /// Return the span index of t relative to nurbsknot[order - 2] by binary search.
    int find_span(double t) const;

    /// Compute the Cox-de Boor basis at t.
    void basis_functions(int span, double t, std::vector<double>& basis) const;

    /// Compute the basis derivatives (Piegl & Tiller A2.3).
    void basis_functions_derivatives(int span, double t, int deriv_order, std::vector<std::vector<double>>& ders) const;

    /// Compute the triangular table of basis functions and nurbsknot differences (Piegl & Tiller A2.3).
    void basis_functions_ndu(int span, double t, std::vector<std::vector<double>>& ndu) const;

    /// Copy every field but the guid.
    void deep_copy_from(const NurbsCurve& src);

    /// Reshape one span's CVs so it starts (side > 0) or ends (side < 0) at t (OpenNURBS ON_EvaluateNurbsDeBoor).
    static bool evaluate_nurbs_de_boor(int cv_dim, int order, int cv_stride, double* cv, const double* nurbsknot, int side, double t);

    /// Reshape one span's CVs so it ends at t.
    static bool de_boor_end(int cv_dim, int order, int cv_stride, double* cv, const double* nurbsknots, double t);

    /// Reshape one span's CVs so it starts at t.
    static bool de_boor_start(int cv_dim, int order, int cv_stride, double* cv, const double* nurbsknots, double t);

    /// Solve matrix * x = rhs in place by Gaussian elimination with partial pivoting, dim values per row.
    static bool solve_dense(std::vector<std::vector<double>>& matrix, std::vector<double>& rhs, int n, int dim);

    /// Return the un-normalized derivative by finite difference with step h.
    Vector derivative_at(double t, double h) const;

    /// Return the arc length of [ta, tb] by 5-point Gauss-Legendre.
    double arc_length_gauss(double ta, double tb, double h) const;

    /// Return the parameter at arc length s_target from the (t, s) table by bracketed Newton.
    double find_t_at_s(double s_target, const std::vector<double>& t_vals, const std::vector<double>& s_vals, double h) const;

    /// Return the Frenet frame from first and second derivatives, world Z then Y as normal fallback.
    static Plane frenet_frame(const Point& origin, const Vector& d1, const Vector& d2);

    /// Return the unit Bessel tangent at points[i0] from the parabola through i0, i1, i2.
    static Vector bessel_tangent(const std::vector<Point>& points, int i0, int i1, int i2);

    /// Return the derivative at t of the Lagrange polynomial through m points from i0 (OCCT BuildTangents).
    static Vector lagrange_tangent(const std::vector<Point>& points, const std::vector<double>& params, int i0, int m, double t);

    /// Construct the closed interpolated cubic through points, wrapped by three CVs.
    static NurbsCurve create_interpolated_periodic(const std::vector<Point>& points, CurveNurbsKnotStyle parameterization);

    /// Return the n + 1 parameters of the closed point loop, uniform or (square root) chord spaced.
    static std::vector<double> periodic_interpolation_parameters(const std::vector<Point>& points, CurveNurbsKnotStyle parameterization);

    /// Return the periodic nurbsknots over params, extended by the wrapped spans at both ends.
    static std::vector<double> periodic_interpolation_nurbsknots(const std::vector<double>& params, int cv_count);

    /// Construct the open interpolated cubic through points with end tangents from end_condition.
    static NurbsCurve create_interpolated_clamped(const std::vector<Point>& points, CurveNurbsKnotStyle parameterization, CurveInterpStyle end_condition);

    /// Return the n + 2 CVs: the points with an end tangent CV after the first and before the last.
    static std::vector<double> interpolation_end_cvs(const std::vector<Point>& points, const std::vector<double>& params, CurveInterpStyle end_condition);

    /// Solve the tridiagonal interpolation system and write the interior CVs into cv.
    static bool solve_interpolation_cvs(const std::vector<Point>& points, const std::vector<double>& params, const std::vector<double>& nurbsknots, std::vector<double>& cv);

    /// Return the x, y, z of the first count points as one flat array.
    static std::vector<double> flatten_points(const std::vector<Point>& points, int count);

    /// Construct the closed least-squares fit with num_cvs distinct CVs.
    static NurbsCurve create_fitted_periodic(const std::vector<Point>& points, int num_cvs, int degree);

    /// Construct the open least-squares fit through the first and last point.
    static NurbsCurve create_fitted_clamped(const std::vector<Point>& points, int num_cvs, int degree);

    /// Accumulate the banded normal equations of the open fit, end CVs fixed.
    static void fitted_band_system(const std::vector<Point>& points, const std::vector<double>& params, const std::vector<double>& nurbsknots, int num_cvs, int degree, std::vector<double>& band, std::vector<double>& rhs);

    /// Lift 2D segments to 3D when 2D and 3D segments are mixed.
    static void promote_to_3d(std::vector<NurbsCurve>& segs);

    /// Group segments into chains by endpoint matching, reversing where needed.
    static std::vector<std::vector<NurbsCurve>> chain_segments(const std::vector<NurbsCurve>& segs, double tolerance);

    /// Append the chain merged into one curve to result, or its segments when they cannot be merged.
    static void join_chain(std::vector<NurbsCurve>& chain, std::vector<NurbsCurve>& result);

    /// Append segment to joined with a C0 junction at the averaged shared CV.
    static void append_segment(NurbsCurve& joined, NurbsCurve& segment, bool rational);

    /// Compute the center of the circle through three points, false when they are collinear.
    static bool circle_center(const Point& p0, const Point& p1, const Point& p2, Point& center);

    /// Return the nurbsknots padded with one superfluous value at each end.
    std::vector<double> full_nurbsknots() const;

    /// Insert one nurbsknot by Boehm, U the padded nurbsknots.
    void insert_nurbsknot_once(double nurbsknot_value, const std::vector<double>& U);

    /// True when the last degree CVs repeat the first and the nurbsknot spacing repeats every period.
    bool is_wrapped() const;

    /// Insert one nurbsknot into a wrapped curve at every period, by Boehm on the periodic sequence.
    void insert_wrapped_nurbsknot_once(double nurbsknot_value);

    /// Return the (t, point) samples of the chord-deviation bisection, sorted by t.
    std::vector<std::pair<double, Point>> adaptive_samples(double angle_tolerance, double min_edge_length, double max_edge_length) const;

    /// Return the homogeneous derivatives (x, y, z, w) at span from the basis derivatives.
    std::vector<std::array<double, 4>> homogeneous_derivatives(int span, const std::vector<std::vector<double>>& ders) const;

    /// Compute the unit tangent and normal at the domain start, false when the derivative vanishes.
    bool start_frame(Vector& T0, Vector& r0) const;

    /// Return r0 carried from the domain start to param by double reflection.
    Vector double_reflection(double param, const Vector& r0, const Vector& T0) const;

    /// Keep the CVs and nurbsknots between the full-multiplicity nurbsknots t0 and t1.
    bool keep_span_range(double t0, double t1);

    /// Return the nurbsknots of the kept range, clamped at t0 and t1.
    std::vector<double> trimmed_nurbsknots(const std::vector<double>& U, int start_span, int new_cv_count, double t0, double t1) const;

    /// Return the index of the first nurbsknot greater than value, -1 when none.
    int first_nurbsknot_above(double value) const;

    /// Return the seam nurbsknot index near t, snapping to an existing nurbsknot or inserting one; -1 on failure.
    int seam_nurbsknot_index(double t, int nurbsknot_index);

    /// Rotate the nurbsknots and CVs of a periodic curve so the domain starts at t.
    bool rotate_periodic_seam(int nurbsknot_index, double t, double dom_len);

    /// Split at t and join the right part before the left so the domain starts at t.
    bool split_seam(double t, double dom_len);
};

/// Write the curve string to a stream.
std::ostream& operator<<(std::ostream& os, const NurbsCurve& curve);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::NurbsCurve> {
    constexpr fmt::format_parse_context::iterator parse(fmt::format_parse_context& ctx) {
        return ctx.begin();
    }

    fmt::format_context::iterator format(const session_cpp::NurbsCurve& curve, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", curve.str());
    }
};
