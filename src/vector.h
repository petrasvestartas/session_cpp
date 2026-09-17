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
class Polyline;

/// A 3D vector with a cached magnitude.
class Vector {
public:
  std::string name = "my_vector"; // Vector name.

  /// Construct the zero vector.
  Vector() : _x(0.0), _y(0.0), _z(0.0) {}

  /// Construct from components.
  Vector(double x, double y, double z) : _x(x), _y(y), _z(z) {}

  /// Copy with a new guid and the same data.
  Vector(const Vector& other);

  /// Copy-assign with a new guid and the same data.
  Vector& operator=(const Vector& other);

  /// Move while preserving the guid.
  Vector(Vector&& other) noexcept = default;

  /// Move-assign while preserving the guid.
  Vector& operator=(Vector&& other) noexcept = default;

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

  /// Construct the zero vector.
  static Vector zero();

  /// Construct the unit vector along x.
  static Vector x_axis();

  /// Construct the unit vector along y.
  static Vector y_axis();

  /// Construct the unit vector along z.
  static Vector z_axis();

  /// Construct the vector from p0 to p1.
  static Vector from_points(const Point& p0, const Point& p1);

  // ═══════════════════════════════════════════════════════════════════════════
  // Operators
  // ═══════════════════════════════════════════════════════════════════════════

  /// Return the mutable component by index (0=x, 1=y, 2=z), dropping the cached magnitude.
  double& operator[](int index);

  /// Return the component by index (0=x, 1=y, 2=z).
  const double& operator[](int index) const;

  /// Compare components within rounding.
  bool operator==(const Vector& other) const;

  /// Compare components within rounding.
  bool operator!=(const Vector& other) const;

  /// Scale in place.
  Vector& operator*=(double factor);

  /// Divide in place.
  Vector& operator/=(double factor);

  /// Add in place.
  Vector& operator+=(const Vector& other);

  /// Subtract in place.
  Vector& operator-=(const Vector& other);

  /// Return a scaled copy.
  Vector operator*(double factor) const;

  /// Return a divided copy.
  Vector operator/(double factor) const;

  /// Return the sum.
  Vector operator+(const Vector& other) const;

  /// Return the difference.
  Vector operator-(const Vector& other) const;

  /// Return the negation.
  Vector operator-() const;

  /// Return a vector scaled by a factor on the left.
  friend Vector operator*(double factor, const Vector& vector);

  // ═══════════════════════════════════════════════════════════════════════════
  // Transformation
  // ═══════════════════════════════════════════════════════════════════════════

  /// Transform in place; only rotation and scale apply, a vector has no position.
  void transform(const Xform& xform);

  /// Return a transformed copy.
  Vector transformed(const Xform& xform) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Geometry
  // ═══════════════════════════════════════════════════════════════════════════

  /// Negate every component in place.
  void reverse();

  /// Return the cached magnitude.
  double magnitude() const;

  /// Return the squared magnitude without the square root.
  double magnitude_squared() const;

  /// Set unit length in place; false when the magnitude is zero.
  bool normalize_self();

  /// Return a unit length copy.
  Vector normalized() const;

  /// Return the dot product.
  double dot(const Vector& other) const;

  /// Return the cross product.
  Vector cross(const Vector& other) const;

  /// Return the angle to other, negated when the cross product points down z; zero below tolerance.
  double angle(const Vector& other, bool sign_by_cross_product = true, bool degrees = true, double tolerance = Tolerance::ZERO_TOLERANCE) const;

  /// Return the projection onto projection_vector: (projection, projected length, perpendicular, perpendicular length).
  std::tuple<Vector, double, Vector, double> projection(const Vector& projection_vector, double tolerance = Tolerance::ZERO_TOLERANCE) const;

  /// Return 1 when parallel, -1 when antiparallel, 0 otherwise.
  int is_parallel_to(const Vector& other) const;

  /// Return whether the dot product is within tolerance of zero.
  bool is_perpendicular_to(const Vector& other) const;

  /// Set this vector perpendicular to v; false when v is zero.
  bool perpendicular_to(const Vector& v);

  /// Return whether the magnitude is within tolerance of zero.
  bool is_zero() const;

  /// Return a copy scaled along its direction so its rise along z equals vertical_height.
  Vector get_leveled_vector(double vertical_height) const;

  /// Return the angles to the x, y and z axes.
  std::array<double, 3> coordinate_direction_3angles(bool degrees = false) const;

  /// Return the polar angle from z and the azimuth from x.
  std::array<double, 2> coordinate_direction_2angles(bool degrees = false) const;

  /// Return the angle in degrees of the xy projection from the x-axis.
  static double angle_between_vector_xy_components(const Vector& vector);

  /// Return the component-wise sum.
  static Vector sum_of_vectors(const std::vector<Vector>& vectors);

  /// Return the component-wise average; empty input returns zero.
  static Vector average(const std::vector<Vector>& vectors);

  /// Scale in place.
  void scale(double factor);

  /// Scale in place by SCALE.
  void scale_up();

  /// Scale in place by 1 / SCALE.
  void scale_down();

  /// Return the reflection through the plane with the given unit normal.
  Vector reflect(const Vector& plane_normal) const;

  /// Return the unit area-weighted normal of a polygon by Newell's method.
  static Vector average_normal(const std::vector<Point>& points);

  /// Return the unit area-weighted normal of a closed polyline by Newell's method.
  static Vector average_normal(const Polyline& polyline);

  // ═══════════════════════════════════════════════════════════════════════════
  // Triangle laws
  // ═══════════════════════════════════════════════════════════════════════════

  /// Return the third side from two sides and the angle between them.
  static double cosine_law(double triangle_edge_length_a, double triangle_edge_length_b, double angle_in_between_edges, bool degrees = true);

  /// Return the angle opposite side b from side a, the angle opposite a and side b.
  static double sine_law_angle(double triangle_edge_length_a, double angle_in_front_of_a, double triangle_edge_length_b, bool degrees = true);

  /// Return side b from side a and the angles opposite a and b.
  static double sine_law_length(double triangle_edge_length_a, double angle_in_front_of_a, double angle_in_front_of_b, bool degrees = true);

  /// Return the angle opposite side c from the three sides.
  static double angle_from_cosine_law(double triangle_edge_length_a, double triangle_edge_length_b, double triangle_edge_length_c, bool degrees = true);

  /// Return the side opposite the first angle from two angles and the side opposite the second.
  static double side_from_sine_law(double angle_in_front_of_result_side, double angle_in_front_of_known_side, double known_side_length, bool degrees = true);

  // ═══════════════════════════════════════════════════════════════════════════
  // JSON
  // ═══════════════════════════════════════════════════════════════════════════

  /// Serialize to a JSON object.
  nlohmann::ordered_json jsondump() const;

  /// Deserialize from a JSON object.
  static Vector jsonload(const nlohmann::json& data);

  /// Serialize to a JSON string.
  std::string file_json_dumps() const;

  /// Deserialize from a JSON string.
  static Vector file_json_loads(const std::string& json_string);

  /// Write to a JSON file.
  void file_json_dump(const std::string& filename) const;

  /// Read from a JSON file.
  static Vector file_json_load(const std::string& filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // Protobuf
  // ═══════════════════════════════════════════════════════════════════════════

  /// Serialize to protobuf bytes.
  std::string pb_dumps() const;

  /// Deserialize from protobuf bytes.
  static Vector pb_loads(const std::string& data);

  /// Write to a protobuf file.
  void pb_dump(const std::string& filename) const;

  /// Read from a protobuf file.
  static Vector pb_load(const std::string& filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // String
  // ═══════════════════════════════════════════════════════════════════════════

  /// Return "x, y, z".
  std::string str() const;

  /// Return "Vector(name, x, y, z, magnitude)".
  std::string repr() const;

private:
  mutable std::string _guid; // Lazy guid.
  double _x = 0.0; // X component.
  double _y = 0.0; // Y component.
  double _z = 0.0; // Z component.
  mutable double _magnitude = 0.0; // Cached magnitude.
  mutable bool _has_magnitude = false; // Whether the cached magnitude is valid.

  /// Return the magnitude scaled to stay finite for large components.
  double compute_magnitude() const;
};

/// Write the vector string to a stream.
std::ostream& operator<<(std::ostream& os, const Vector& vector);

} // namespace session_cpp
