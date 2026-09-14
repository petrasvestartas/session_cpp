#pragma once
#include "guid.h"
#include "json.h"
#include "tolerance.h"
#include "xform.h"
#include "fmt/core.h"
#include <array>
#include <cmath>
#include <fstream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace session_cpp {

class Point;

/// A 3D vector with a cached magnitude
class Vector {
public:
  std::string name = "my_vector";

  Vector() : _x(0.0), _y(0.0), _z(0.0) {}

  Vector(double x, double y, double z) : _x(x), _y(y), _z(z) {}

  /// Copy constructor (new guid, same data)
  Vector(const Vector &other);

  /// Copy assignment (new guid, same data)
  Vector &operator=(const Vector &other);

  /// Move keeps the guid; declaring it stops `return x;` from falling back to the guid-minting copy
  Vector(Vector &&other) noexcept = default;
  Vector &operator=(Vector &&other) noexcept = default;

  bool has_guid() const { return !_guid.empty(); }
  const std::string &guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
  std::string &guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Zero vector
  static Vector zero();

  /// Unit vector along x
  static Vector x_axis();

  /// Unit vector along y
  static Vector y_axis();

  /// Unit vector along z
  static Vector z_axis();

  /// Vector from p0 to p1
  static Vector from_points(const Point &p0, const Point &p1);

  // ═══════════════════════════════════════════════════════════════════════════
  // Operators
  // ═══════════════════════════════════════════════════════════════════════════

  /// Coordinate by index (0=x, 1=y, 2=z); the mutable form drops the cached magnitude
  double &operator[](int index);
  const double &operator[](int index) const;

  bool operator==(const Vector &other) const;
  bool operator!=(const Vector &other) const;

  Vector &operator*=(double factor);
  Vector &operator/=(double factor);
  Vector &operator+=(const Vector &other);
  Vector &operator-=(const Vector &other);

  Vector operator*(double factor) const;
  Vector operator/(double factor) const;
  Vector operator+(const Vector &other) const;
  Vector operator-(const Vector &other) const;
  Vector operator-() const;
  friend Vector operator*(double factor, const Vector &vector);

  // ═══════════════════════════════════════════════════════════════════════════
  // Transformation
  // ═══════════════════════════════════════════════════════════════════════════

  /// Transform in place; only rotation and scale apply, a vector has no position
  void transform(const Xform &xform);

  /// Transformed copy
  Vector transformed(const Xform &xform) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Geometry
  // ═══════════════════════════════════════════════════════════════════════════

  /// Negate every component in place
  void reverse();

  /// Cached magnitude
  double magnitude() const;

  /// Squared magnitude without the square root
  double magnitude_squared() const;

  /// Unit length in place; false when the magnitude is zero
  bool normalize_self();

  /// Unit length copy
  Vector normalized() const;

  /// Dot product
  double dot(const Vector &other) const;

  /// Cross product
  Vector cross(const Vector &other) const;

  /// Angle to other, negated when the cross product points down z; zero below tolerance
  double angle(const Vector &other, bool sign_by_cross_product = true, bool degrees = true, double tolerance = Tolerance::ZERO_TOLERANCE) const;

  /// Projection onto projection_vector: (projection, projected length, perpendicular, perpendicular length)
  std::tuple<Vector, double, Vector, double> projection(const Vector &projection_vector, double tolerance = Tolerance::ZERO_TOLERANCE) const;

  /// 1 parallel, -1 antiparallel, 0 neither
  int is_parallel_to(const Vector &other) const;

  /// True when the dot product is within tolerance of zero
  bool is_perpendicular_to(const Vector &other) const;

  /// Set this vector perpendicular to v; false when v is zero
  bool perpendicular_to(const Vector &v);

  /// True when the magnitude is within tolerance of zero
  bool is_zero() const;

  /// Copy scaled along its direction so its rise along z equals vertical_height
  Vector get_leveled_vector(double vertical_height) const;

  /// Angles to the x, y and z axes
  std::array<double, 3> coordinate_direction_3angles(bool degrees = false) const;

  /// Polar angle from z and azimuth from x
  std::array<double, 2> coordinate_direction_2angles(bool degrees = false) const;

  /// Angle in degrees of the xy projection from the x-axis
  static double angle_between_vector_xy_components(const Vector &vector);

  /// Component-wise sum
  static Vector sum_of_vectors(const std::vector<Vector> &vectors);

  /// Component-wise average; empty input returns zero
  static Vector average(const std::vector<Vector> &vectors);

  void scale(double factor);

  /// Scale by SCALE
  void scale_up();

  /// Scale by 1 / SCALE
  void scale_down();

  /// Reflection through the plane with the given unit normal
  Vector reflect(const Vector &plane_normal) const;

  /// Unit area-weighted normal of a polygon by Newell's method
  static Vector average_normal(const std::vector<Point> &points);

  // ═══════════════════════════════════════════════════════════════════════════
  // Triangle laws
  // ═══════════════════════════════════════════════════════════════════════════

  /// Third side from two sides and the angle between them
  static double cosine_law(double triangle_edge_length_a, double triangle_edge_length_b, double angle_in_between_edges, bool degrees = true);

  /// Angle opposite side b from side a, the angle opposite a and side b
  static double sine_law_angle(double triangle_edge_length_a, double angle_in_front_of_a, double triangle_edge_length_b, bool degrees = true);

  /// Side b from side a and the angles opposite a and b
  static double sine_law_length(double triangle_edge_length_a, double angle_in_front_of_a, double angle_in_front_of_b, bool degrees = true);

  /// Angle opposite side c from the three sides
  static double angle_from_cosine_law(double triangle_edge_length_a, double triangle_edge_length_b, double triangle_edge_length_c, bool degrees = true);

  /// Side opposite the first angle from two angles and the side opposite the second
  static double side_from_sine_law(double angle_in_front_of_result_side, double angle_in_front_of_known_side, double known_side_length, bool degrees = true);

  // ═══════════════════════════════════════════════════════════════════════════
  // JSON
  // ═══════════════════════════════════════════════════════════════════════════

  nlohmann::ordered_json jsondump() const;
  static Vector jsonload(const nlohmann::json &data);
  std::string file_json_dumps() const;
  static Vector file_json_loads(const std::string &json_string);
  void file_json_dump(const std::string &filename) const;
  static Vector file_json_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // Protobuf
  // ═══════════════════════════════════════════════════════════════════════════

  std::string pb_dumps() const;
  static Vector pb_loads(const std::string &data);
  void pb_dump(const std::string &filename) const;
  static Vector pb_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // String
  // ═══════════════════════════════════════════════════════════════════════════

  /// "x, y, z"
  std::string str() const;

  /// "Vector(name, x, y, z, magnitude)"
  std::string repr() const;

private:
  mutable std::string _guid;
  double _x = 0.0;
  double _y = 0.0;
  double _z = 0.0;
  mutable double _magnitude = 0.0;
  mutable bool _has_magnitude = false;

  /// Magnitude scaled to stay finite for large components
  double compute_magnitude() const;
};

std::ostream &operator<<(std::ostream &os, const Vector &vector);

} // namespace session_cpp
