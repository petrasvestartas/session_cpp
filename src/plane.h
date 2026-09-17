#pragma once
#include "color.h"
#include "guid.h"
#include "json.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
#include "fmt/core.h"
#include <cmath>
#include <fstream>
#include <ostream>
#include <string>
#include <vector>

namespace session_cpp {

class Polyline;

/// A plane defined by an origin and an orthonormal x, y, z frame
class Plane {
public:
  std::string name = "my_plane";
  double width = 1.0;
  Color linecolor = Color::blue();

  Plane();

  /// Origin and two axes; x is normalized, y is made orthogonal to x, z = x × y
  Plane(const Point &point, const Vector &x_axis, const Vector &y_axis, std::string name = "my_plane");

  /// Copy constructor (new guid, same data)
  Plane(const Plane &other);

  /// Copy assignment (new guid, same data)
  Plane &operator=(const Plane &other);

  /// Move keeps the guid; declaring it stops `return x;` from falling back to the guid-minting copy
  Plane(Plane &&other) noexcept = default;
  Plane &operator=(Plane &&other) noexcept = default;

  bool has_guid() const { return !_guid.empty(); }
  const std::string &guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
  std::string &guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Clear the guid so a fresh one mints lazily on next read
  void refresh_guid() { _guid.clear(); }

  const Point &origin() const { return _origin; }
  const Vector &x_axis() const { return _x_axis; }
  const Vector &y_axis() const { return _y_axis; }
  const Vector &z_axis() const { return _z_axis; }
  double a() const { return _a; }
  double b() const { return _b; }
  double c() const { return _c; }
  double d() const { return _d; }

  // ═══════════════════════════════════════════════════════════════════════════
  // Static constructors
  // ═══════════════════════════════════════════════════════════════════════════

  /// Frame taken as given, no normalization
  static Plane from_frame(const Point &origin, const Vector &x_axis, const Vector &y_axis, const Vector &z_axis);

  /// Plane through point with normal as z axis
  static Plane from_point_normal(const Point &point, const Vector &normal, bool normalize = true);

  /// Plane through the first three points, x axis along the first edge
  static Plane from_points(const std::vector<Point> &points);

  /// Least-squares plane through points by power-iteration PCA
  static Plane from_points_pca(const std::vector<Point> &points);

  /// Plane with x axis from point1 to point2
  static Plane from_two_points(const Point &point1, const Point &point2);

  /// All-zero frame; fails is_valid()
  static Plane invalid();

  static Plane xy_plane();
  static Plane yz_plane();
  static Plane xz_plane();

  // ═══════════════════════════════════════════════════════════════════════════
  // Operators
  // ═══════════════════════════════════════════════════════════════════════════

  /// Axis by index (0=x, 1=y, 2=z)
  Vector &operator[](int index);
  const Vector &operator[](int index) const;

  /// Same name, frame and linecolor; guid ignored
  bool operator==(const Plane &other) const;
  bool operator!=(const Plane &other) const;

  Plane &operator+=(const Vector &other);
  Plane &operator-=(const Vector &other);

  Plane operator+(const Vector &other) const;
  Plane operator-(const Vector &other) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Transformation
  // ═══════════════════════════════════════════════════════════════════════════

  /// Transform in place
  void transform(const Xform &xform);

  /// Transformed copy
  Plane transformed(const Xform &xform) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Geometry
  // ═══════════════════════════════════════════════════════════════════════════

  bool is_valid() const;

  /// Swap x and y and flip z
  void reverse();

  /// Rotate x and y around z
  void rotate(double angles_in_radians);

  /// True when x × y points along z
  bool is_right_hand() const;

  /// Normals parallel (can_be_flipped) or exactly opposite (!can_be_flipped)
  static bool is_same_direction(const Plane &plane0, const Plane &plane1, bool can_be_flipped = true);

  /// Each origin lies on the other plane
  static bool is_same_position(const Plane &plane0, const Plane &plane1);

  /// Same direction and same position
  static bool is_coplanar(const Plane &plane0, const Plane &plane1, bool can_be_flipped = true);

  /// is_coplanar from origin and normal pairs without building planes; tolerance < 0 uses APPROXIMATION
  static bool is_coplanar_from_normals(const Point &origin0, const Vector &normal0, const Point &origin1, const Vector &normal1, bool can_be_flipped = true, double tolerance = -1.0);

  /// Copy moved along z by distance
  Plane translate_by_normal(double distance) const;

  /// Orthogonal projection of p onto the plane
  Point project(const Point &p) const;

  /// True when a*p[0] + b*p[1] + c*p[2] + d < 0
  bool has_on_negative_side(const Point &p) const;

  /// Squared distance from p to the plane
  double squared_distance(const Point &p) const;

  /// Canonical in-plane axis from the normal alone: zero the smallest normal coordinate, negate-swap the other two
  Vector base1() const;

  /// z × base1, unit length
  Vector base2() const;

  /// Square outline of side scale plus the three axes as polylines
  std::vector<Polyline> to_polylines(double scale = 1.0) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // JSON
  // ═══════════════════════════════════════════════════════════════════════════

  nlohmann::ordered_json jsondump() const;
  static Plane jsonload(const nlohmann::json &data);
  std::string file_json_dumps() const;
  static Plane file_json_loads(const std::string &json_string);
  void file_json_dump(const std::string &filename) const;
  static Plane file_json_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // Protobuf
  // ═══════════════════════════════════════════════════════════════════════════

  std::string pb_dumps() const;
  static Plane pb_loads(const std::string &data);
  void pb_dump(const std::string &filename) const;
  static Plane pb_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // String
  // ═══════════════════════════════════════════════════════════════════════════

  /// "origin\nx_axis\ny_axis\nz_axis"
  std::string str() const;

  /// "Plane(name, ox, oy, oz, zx, zy, zz, Color(...))"
  std::string repr() const;

private:
  mutable std::string _guid;
  Point _origin = Point();
  Vector _x_axis = Vector::x_axis();
  Vector _y_axis = Vector::y_axis();
  Vector _z_axis = Vector::z_axis();
  double _a = 0.0;
  double _b = 0.0;
  double _c = 1.0;
  double _d = 0.0;

  /// Recompute a, b, c, d from z_axis and origin
  void update_equation();
};

std::ostream &operator<<(std::ostream &os, const Plane &plane);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::Plane> {
  constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }
  auto format(const session_cpp::Plane &o, fmt::format_context &ctx) const {
    return fmt::format_to(ctx.out(), "{}", o.str());
  }
};
