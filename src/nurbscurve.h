#pragma once
#include "color.h"
#include "guid.h"
#include "json.h"
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

namespace session_cpp {

/// A NURBS curve: OpenNURBS layout, nurbsknot count = order + cv_count - 2, homogeneous CVs when rational
class NurbsCurve {
public:
  std::string name = "my_nurbscurve";
  double width = 1.0;
  std::vector<Color> pointcolors;
  std::vector<Color> linecolors;
  int m_dim;
  int m_is_rat;
  int m_order;
  int m_cv_count;
  int m_cv_stride;
  std::vector<double> m_nurbsknot;
  std::vector<double> m_cv;

  NurbsCurve();
  NurbsCurve(int dimension, bool is_rational, int order, int cv_count);

  /// Copy constructor (new guid, same data)
  NurbsCurve(const NurbsCurve &other);

  /// Copy assignment (new guid, same data)
  NurbsCurve &operator=(const NurbsCurve &other);

  /// Move keeps the guid; declaring it stops `return x;` from falling back to the guid-minting copy
  NurbsCurve(NurbsCurve &&other) noexcept = default;
  NurbsCurve &operator=(NurbsCurve &&other) noexcept = default;

  ~NurbsCurve();

  bool has_guid() const { return !_guid.empty(); }
  const std::string &guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
  std::string &guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Clear the guid so a fresh one mints lazily on next read
  void refresh_guid() { _guid.clear(); }

  // ═══════════════════════════════════════════════════════════════════════════
  // Static constructors
  // ═══════════════════════════════════════════════════════════════════════════

  /// Clamped or periodic uniform curve through control points, domain rescaled to [0, arc length]
  static NurbsCurve create(bool periodic, int degree, const std::vector<Point> &points, int dimension = 3, double nurbsknot_delta = 1.0);

  /// Interpolated cubic through points; Rhino (Bessel) or Occt (Lagrange) end tangents
  static NurbsCurve create_interpolated(const std::vector<Point> &points, CurveNurbsKnotStyle parameterization = CurveNurbsKnotStyle::Chord, CurveInterpStyle end_condition = CurveInterpStyle::Rhino);

  /// Curve from poles, weights, distinct knots and multiplicities (OCCT convention)
  static NurbsCurve create_from_parameters(const std::vector<Point> &points, const std::vector<double> &weights, const std::vector<double> &knots, const std::vector<int> &mults, int degree, bool periodic = false);

  /// Least-squares fit with num_cvs control points (Piegl & Tiller 9.4)
  static NurbsCurve create_fitted(const std::vector<Point> &points, int num_cvs, int degree = 3, bool is_periodic = false);

  /// Chain segments by endpoint matching, raise to a common degree and merge with C0 junctions
  static std::vector<NurbsCurve> join(const std::vector<NurbsCurve> &curves, double tolerance = Tolerance::ZERO_TOLERANCE);

  // ═══════════════════════════════════════════════════════════════════════════
  // Operators
  // ═══════════════════════════════════════════════════════════════════════════

  /// Same name, width, colors, layout, nurbsknots and CVs to 1e-12; guid ignored
  bool operator==(const NurbsCurve &other) const;
  bool operator!=(const NurbsCurve &other) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Transformation
  // ═══════════════════════════════════════════════════════════════════════════

  /// Transform in place
  bool transform(const Xform &xform);

  /// Transformed copy
  NurbsCurve transformed(const Xform &xform) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Initialization
  // ═══════════════════════════════════════════════════════════════════════════

  /// Zero every field
  void initialize();

  /// Allocate layout for dimension, rationality, order and cv_count
  bool create(int dimension, bool is_rational, int order, int cv_count);

  /// Clamped uniform nurbsknots over control points
  bool create_clamped_uniform(int dimension, int order, const std::vector<Point> &points, double nurbsknot_delta = 1.0);

  /// Periodic uniform nurbsknots over control points wrapped by order - 1
  bool create_periodic_uniform(int dimension, int order, const std::vector<Point> &points, double nurbsknot_delta = 1.0);

  /// Reset to the empty state
  void destroy();

  // ═══════════════════════════════════════════════════════════════════════════
  // Boolean queries
  // ═══════════════════════════════════════════════════════════════════════════

  bool is_valid() const;
  bool is_rational() const { return m_is_rat != 0; }

  /// Start point equals end point
  bool is_closed() const;

  /// Last degree CVs repeat the first and nurbsknots are uniform
  bool is_periodic() const;

  /// Every CV within tolerance of the chord
  bool is_linear(double tolerance = Tolerance::ZERO_TOLERANCE) const;

  /// Every CV within tolerance of one plane, written to plane when given
  bool is_planar(Plane *plane = nullptr, double tolerance = Tolerance::ZERO_TOLERANCE) const;

  /// Planar and equidistant from one center along the curve, plane written when given
  bool is_arc(Plane *plane = nullptr, double tolerance = Tolerance::ZERO_TOLERANCE) const;

  /// Every CV within tolerance of test_plane
  bool is_in_plane(const Plane &test_plane, double tolerance = Tolerance::ZERO_TOLERANCE) const;

  /// Zero second derivative at end (0 = start, 1 = end, 2 = both)
  bool is_natural(int end = 2) const;

  /// Vertex count when every span is a line, else 0; vertices and params written when given
  int is_polyline(std::vector<Point> *points = nullptr, std::vector<double> *params = nullptr) const;

  /// Every span collapsed to a point
  bool is_singular() const;

  /// Same layout, CVs and weights to tolerance, and nurbsknots unless ignore_parameterization
  bool is_duplicate(const NurbsCurve &other, bool ignore_parameterization, double tolerance = Tolerance::ZERO_TOLERANCE) const;

  /// Continuity at t from nurbsknot multiplicity (0 = C0, 1 = C1, 2 = C2, 3 = G1, 4 = G2)
  bool is_continuous(int continuity_type, double t, int *hint = nullptr, double point_tolerance = Tolerance::ZERO_TOLERANCE, double d1_tolerance = Tolerance::ZERO_TOLERANCE, double d2_tolerance = Tolerance::ZERO_TOLERANCE, double cos_angle_tolerance = 0.99984769515639123, double curvature_tolerance = 1e-8) const;

  /// Right count, non-decreasing, non-empty domain
  bool is_valid_nurbsknot_vector() const;

  /// Full multiplicity at end (0 = start, 1 = end, 2 = both)
  bool is_clamped(int end = 2) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Accessors
  // ═══════════════════════════════════════════════════════════════════════════

  int dimension() const { return m_dim; }
  int order() const { return m_order; }
  int degree() const { return m_order - 1; }
  int cv_count() const { return m_cv_count; }

  /// Doubles per CV: dimension + 1 when rational
  int cv_size() const;

  /// order + cv_count - 2
  int nurbsknot_count() const;

  /// Distinct nurbsknot intervals inside the domain
  int span_count() const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Control vertex access
  // ═══════════════════════════════════════════════════════════════════════════

  /// Pointer to the CV doubles, nullptr when out of range
  double *cv(int cv_index);
  const double *cv(int cv_index) const;

  /// Euclidean CV (divided by weight when rational)
  Point get_cv(int cv_index) const;

  /// Homogeneous CV (x, y, z, w)
  bool get_cv_4d(int cv_index, double &x, double &y, double &z, double &w) const;
  std::tuple<double, double, double, double> get_cv_4d(int cv_index) const;

  /// Set CV from a point, weight reset to 1
  bool set_cv(int cv_index, const Point &point);

  /// Set homogeneous CV, making the curve rational when w != 1
  bool set_cv_4d(int cv_index, double x, double y, double z, double w);

  /// Weight of a CV, 1 when non-rational
  double weight(int cv_index) const;

  /// Set weight, making the curve rational first
  bool set_weight(int cv_index, double weight);

  // ═══════════════════════════════════════════════════════════════════════════
  // NurbsKnot access
  // ═══════════════════════════════════════════════════════════════════════════

  double nurbsknot(int nurbsknot_index) const;
  bool set_nurbsknot(int nurbsknot_index, double nurbsknot_value);

  /// Count of nurbsknots equal to the one at nurbsknot_index
  int nurbsknot_multiplicity(int nurbsknot_index) const;

  /// Reflected end nurbsknot (0 = start, 1 = end)
  double superfluous_nurbsknot(int end) const;

  const double *nurbsknot_array() const { return m_nurbsknot.data(); }
  double *cv_array() { return m_cv.data(); }
  const double *cv_array() const { return m_cv.data(); }
  std::vector<double> get_nurbsknots() const { return m_nurbsknot; }

  /// Boehm insertion to the given multiplicity
  bool insert_nurbsknot(double nurbsknot_value, int nurbsknot_multiplicity = 1);

  /// Greville abcissa of a CV
  double greville_abcissa(int cv_index) const;
  bool get_greville_abcissae(std::vector<double> &abcissae) const;
  std::vector<double> get_greville_abcissae() const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Domain
  // ═══════════════════════════════════════════════════════════════════════════

  std::pair<double, double> domain() const;
  double domain_start() const;
  double domain_end() const;
  double domain_middle() const;

  /// Rescale nurbsknots to [t0, t1]
  bool set_domain(double t0, double t1);

  /// Distinct nurbsknot values inside the domain
  std::vector<double> get_span_vector() const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Geometry
  // ═══════════════════════════════════════════════════════════════════════════

  /// First interior nurbsknot in (t0, t1) whose multiplicity breaks continuity_type
  bool get_next_discontinuity(int continuity_type, double t0, double t1, double &t_out, int *hint = nullptr, double cos_angle_tolerance = 0.99984769515639123, double curvature_tolerance = 1e-8) const;
  std::pair<bool, double> get_next_discontinuity(int continuity_type, double t0, double t1) const;

  /// Arc length by 10-point Gauss-Legendre over 4 subdivisions per span
  double length(double tolerance = 1e-6) const;

  /// Chord-deviation subdivision; angle_tolerance in radians, edge lengths default to length / 10 and / 1000
  bool to_polyline_adaptive(std::vector<Point> &points, std::vector<double> *params = nullptr, double angle_tolerance = 0.1, double min_edge_length = 0.0, double max_edge_length = 0.0) const;
  std::pair<std::vector<Point>, std::vector<double>> to_polyline_adaptive(double angle_tolerance = 0.1, double min_edge_length = 0.0, double max_edge_length = 0.0) const;

  /// count points at equal arc length, ends included or excluded
  bool divide_by_count(int count, std::vector<Point> &points, std::vector<double> *params = nullptr, bool include_endpoints = true) const;
  std::pair<std::vector<Point>, std::vector<double>> divide_by_count(int count, bool include_endpoints = true) const;

  /// Points every segment_length of arc length from the start
  bool divide_by_length(double segment_length, std::vector<Point> &points, std::vector<double> *params = nullptr) const;
  std::pair<std::vector<Point>, std::vector<double>> divide_by_length(double segment_length) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Evaluation
  // ═══════════════════════════════════════════════════════════════════════════

  Point point_at(double t) const;

  /// [point, first derivative, ..., derivative_count] with zeros past the degree
  std::vector<Vector> evaluate(double t, int derivative_count = 0) const;

  /// Unit tangent by central difference
  Vector tangent_at(double t) const;

  /// |C' x C''| / |C'|^3
  double curvature_at(double t) const;

  /// Parameter of the closest point to test_point
  double closest_parameter(const Point &test_point) const;
  Point closest_point(const Point &test_point) const;

  /// Parameters (u, v) where this curve and other are closest
  std::pair<double, double> closest_parameters_curve(const NurbsCurve &other) const;
  std::pair<Point, Point> closest_points_curve(const NurbsCurve &other) const;

  /// Frenet frame (tangent, normal, binormal); normalized maps t from [0, 1]
  Plane plane_at(double t, bool normalized) const;

  /// Rotation minimizing frame by double reflection (Wang et al. 2008)
  Plane perpendicular_plane_at(double t, bool normalized) const;

  /// count + 1 rotation minimizing frames at equal arc length
  std::vector<Plane> get_perpendicular_planes(int count) const;

  Point point_at_start() const;
  Point point_at_middle() const;
  Point point_at_end() const;

  /// Clamp and move the first CV
  bool set_start_point(const Point &start_point);

  /// Clamp and move the last CV
  bool set_end_point(const Point &end_point);

  // ═══════════════════════════════════════════════════════════════════════════
  // Modifications
  // ═══════════════════════════════════════════════════════════════════════════

  /// Reverse direction keeping the domain
  bool reverse();

  /// Swap two coordinate axes of every CV
  bool swap_coordinates(int axis_i, int axis_j);

  /// Keep [t0, t1] by nurbsknot insertion
  bool trim(double t0, double t1);

  /// Trimmed copies on both sides of t
  bool split(double t, NurbsCurve &left_curve, NurbsCurve &right_curve) const;
  std::pair<NurbsCurve, NurbsCurve> split(double t) const;

  /// Extrapolate the domain to cover [t0, t1] by de Boor
  bool extend(double t0, double t1);

  /// Add unit weights
  bool make_rational();

  /// Drop weights; fails when they differ unless force
  bool make_non_rational(bool force = false);

  /// Full multiplicity at end (0 = start, 1 = end, 2 = both) with CVs adjusted
  bool clamp_end(int end);

  /// Raise degree by blossoming without changing the shape
  bool increase_degree(int desired_degree);

  /// Move the seam of a closed curve to t
  bool change_closed_curve_seam(double t);

  // ═══════════════════════════════════════════════════════════════════════════
  // JSON
  // ═══════════════════════════════════════════════════════════════════════════

  nlohmann::ordered_json jsondump() const;
  static NurbsCurve jsonload(const nlohmann::json &data);
  std::string file_json_dumps() const;
  static NurbsCurve file_json_loads(const std::string &json_string);
  void file_json_dump(const std::string &filename) const;
  static NurbsCurve file_json_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // Protobuf
  // ═══════════════════════════════════════════════════════════════════════════

  std::string pb_dumps() const;
  static NurbsCurve pb_loads(const std::string &data);
  void pb_dump(const std::string &filename) const;
  static NurbsCurve pb_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // String
  // ═══════════════════════════════════════════════════════════════════════════

  /// "NurbsCurve(name=..., degree=..., cvs=...)"
  std::string str() const;

  /// Multi-line form with every control point
  std::string repr() const;

private:
  mutable std::string _guid;

  /// Span with full end multiplicity whose CVs lie on its chord
  bool span_is_linear(int span_index, double min_length, double tolerance) const;

  /// Span collapsed to a point
  bool span_is_singular(int span_index) const;

  /// Span index of t relative to nurbsknot[order - 2] by binary search
  int find_span(double t) const;

  /// Cox-de Boor basis at t
  void basis_functions(int span, double t, std::vector<double> &basis) const;

  /// Basis derivatives (Piegl & Tiller A2.3)
  void basis_functions_derivatives(int span, double t, int deriv_order, std::vector<std::vector<double>> &ders) const;

  /// Copy every field but the guid
  void deep_copy_from(const NurbsCurve &src);

  /// OpenNURBS ON_EvaluateNurbsDeBoor: reshape one span's CVs so it starts (side > 0) or ends (side < 0) at t
  static bool evaluate_nurbs_de_boor(int cv_dim, int order, int cv_stride, double *cv, const double *nurbsknot, int side, double t);

  /// Un-normalized derivative by finite difference with step h
  Vector derivative_at(double t, double h) const;

  /// Arc length of [ta, tb] by 5-point Gauss-Legendre
  double arc_length_gauss(double ta, double tb, double h) const;

  /// Parameter at arc length s_target from the (t, s) table by bracketed Newton
  double find_t_at_s(double s_target, const std::vector<double> &t_vals, const std::vector<double> &s_vals, double h) const;

  /// Frenet frame from first and second derivatives, world Z then Y as normal fallback
  static Plane frenet_frame(const Point &origin, const Vector &d1, const Vector &d2);

  /// Unit Bessel tangent at points[i0] from the parabola through i0, i1, i2
  static Vector bessel_tangent(const std::vector<Point> &points, int i0, int i1, int i2);

  /// Derivative at t of the Lagrange polynomial through m points from i0 (OCCT BuildTangents)
  static Vector lagrange_tangent(const std::vector<Point> &points, const std::vector<double> &params, int i0, int m, double t);
};

std::ostream &operator<<(std::ostream &os, const NurbsCurve &curve);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::NurbsCurve> {
  constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }
  auto format(const session_cpp::NurbsCurve &o, fmt::format_context &ctx) const {
    return fmt::format_to(ctx.out(), "{}", o.str());
  }
};
