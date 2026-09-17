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

/// A plane defined by an origin and an orthonormal x, y, z frame.
class Plane {
public:
  std::string name = "my_plane"; // Plane name.
  double width = 1.0; // Display width.
  Color linecolor = Color::blue(); // Display color.

  /// Construct the world XY plane.
  Plane();

  /// Construct from an origin and two axes; x is normalized, y is made orthogonal to x, z = x × y.
  Plane(const Point& point, const Vector& x_axis, const Vector& y_axis, std::string name = "my_plane");

  /// Copy with a new guid and the same data.
  Plane(const Plane& other);

  /// Copy-assign with a new guid and the same data.
  Plane& operator=(const Plane& other);

  /// Move while preserving the guid.
  Plane(Plane&& other) noexcept = default;

  /// Move-assign while preserving the guid.
  Plane& operator=(Plane&& other) noexcept = default;

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

  /// Clear the guid so a fresh one mints lazily on the next read.
  void refresh_guid() { _guid.clear(); }

  /// Return the origin.
  const Point& origin() const { return _origin; }

  /// Return the unit x axis.
  const Vector& x_axis() const { return _x_axis; }

  /// Return the unit y axis.
  const Vector& y_axis() const { return _y_axis; }

  /// Return the unit z axis.
  const Vector& z_axis() const { return _z_axis; }

  /// Return plane equation coefficient a.
  double a() const { return _a; }

  /// Return plane equation coefficient b.
  double b() const { return _b; }

  /// Return plane equation coefficient c.
  double c() const { return _c; }

  /// Return plane equation coefficient d.
  double d() const { return _d; }

  // ═══════════════════════════════════════════════════════════════════════════
  // Static constructors
  // ═══════════════════════════════════════════════════════════════════════════

  /// Construct from a frame taken as given, no normalization.
  static Plane from_frame(const Point& origin, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis);

  /// Construct the plane through point with normal as z axis.
  static Plane from_point_normal(const Point& point, const Vector& normal, bool normalize = true);

  /// Construct the plane through the first three points, x axis along the first edge.
  static Plane from_points(const std::vector<Point>& points);

  /// Construct the least-squares plane through points by power-iteration PCA.
  static Plane from_points_pca(const std::vector<Point>& points);

  /// Construct the plane with x axis from point1 to point2.
  static Plane from_two_points(const Point& point1, const Point& point2);

  /// Construct an all-zero frame that fails is_valid().
  static Plane invalid();

  /// Construct the world XY plane.
  static Plane xy_plane();

  /// Construct the world YZ plane.
  static Plane yz_plane();

  /// Construct the world XZ plane.
  static Plane xz_plane();

  // ═══════════════════════════════════════════════════════════════════════════
  // Operators
  // ═══════════════════════════════════════════════════════════════════════════

  /// Return the mutable axis by index (0=x, 1=y, 2=z).
  Vector& operator[](int index);

  /// Return the axis by index (0=x, 1=y, 2=z).
  const Vector& operator[](int index) const;

  /// Compare name, frame and linecolor; guid ignored.
  bool operator==(const Plane& other) const;

  /// Compare name, frame and linecolor; guid ignored.
  bool operator!=(const Plane& other) const;

  /// Translate in place.
  Plane& operator+=(const Vector& other);

  /// Translate back in place.
  Plane& operator-=(const Vector& other);

  /// Return a translated copy.
  Plane operator+(const Vector& other) const;

  /// Return a copy translated back.
  Plane operator-(const Vector& other) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Transformation
  // ═══════════════════════════════════════════════════════════════════════════

  /// Transform in place.
  void transform(const Xform& xform);

  /// Return a transformed copy.
  Plane transformed(const Xform& xform) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Geometry
  // ═══════════════════════════════════════════════════════════════════════════

  /// Return whether the frame is non-zero.
  bool is_valid() const;

  /// Swap x and y and flip z in place.
  void reverse();

  /// Rotate x and y around z in place.
  void rotate(double angles_in_radians);

  /// Return whether x × y points along z.
  bool is_right_hand() const;

  /// Return whether the normals are parallel (can_be_flipped) or exactly opposite (!can_be_flipped).
  static bool is_same_direction(const Plane& plane0, const Plane& plane1, bool can_be_flipped = true);

  /// Return whether each origin lies on the other plane.
  static bool is_same_position(const Plane& plane0, const Plane& plane1);

  /// Return whether the planes share direction and position.
  static bool is_coplanar(const Plane& plane0, const Plane& plane1, bool can_be_flipped = true);

  /// Return is_coplanar from origin and normal pairs without building planes; tolerance < 0 uses APPROXIMATION.
  static bool is_coplanar_from_normals(const Point& origin0, const Vector& normal0, const Point& origin1, const Vector& normal1, bool can_be_flipped = true, double tolerance = -1.0);

  /// Return a copy moved along z by distance.
  Plane translate_by_normal(double distance) const;

  /// Return the orthogonal projection of p onto the plane.
  Point project(const Point& p) const;

  /// Return whether a*p[0] + b*p[1] + c*p[2] + d < 0.
  bool has_on_negative_side(const Point& p) const;

  /// Return the squared distance from p to the plane.
  double squared_distance(const Point& p) const;

  /// Return the canonical in-plane axis from the normal alone: zero the smallest normal coordinate, negate-swap the other two.
  Vector base1() const;

  /// Return z × base1 at unit length.
  Vector base2() const;

  /// Return the square outline of side scale plus the three axes as polylines.
  std::vector<Polyline> to_polylines(double scale = 1.0) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // JSON
  // ═══════════════════════════════════════════════════════════════════════════

  /// Serialize to a JSON object.
  nlohmann::ordered_json jsondump() const;

  /// Deserialize from a JSON object.
  static Plane jsonload(const nlohmann::json& data);

  /// Serialize to a JSON string.
  std::string file_json_dumps() const;

  /// Deserialize from a JSON string.
  static Plane file_json_loads(const std::string& json_string);

  /// Write to a JSON file.
  void file_json_dump(const std::string& filename) const;

  /// Read from a JSON file.
  static Plane file_json_load(const std::string& filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // Protobuf
  // ═══════════════════════════════════════════════════════════════════════════

  /// Serialize to protobuf bytes.
  std::string pb_dumps() const;

  /// Deserialize from protobuf bytes.
  static Plane pb_loads(const std::string& data);

  /// Write to a protobuf file.
  void pb_dump(const std::string& filename) const;

  /// Read from a protobuf file.
  static Plane pb_load(const std::string& filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // String
  // ═══════════════════════════════════════════════════════════════════════════

  /// Return "origin\nx_axis\ny_axis\nz_axis".
  std::string str() const;

  /// Return "Plane(name, ox, oy, oz, zx, zy, zz, Color(...))".
  std::string repr() const;

private:
  mutable std::string _guid; // Lazy guid.
  Point _origin = Point();

  /// Return the unit x axis.
  Vector _x_axis = Vector::x_axis();

  /// Return the unit y axis.
  Vector _y_axis = Vector::y_axis();

  /// Return the unit z axis.
  Vector _z_axis = Vector::z_axis();
  double _a = 0.0; // Plane equation coefficient a.
  double _b = 0.0; // Plane equation coefficient b.
  double _c = 1.0; // Plane equation coefficient c.
  double _d = 0.0; // Plane equation coefficient d.

  /// Recompute a, b, c, d from z_axis and origin.
  void update_equation();
};

/// Write the plane string to a stream.
std::ostream& operator<<(std::ostream& os, const Plane& plane);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::Plane> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }
  auto format(const session_cpp::Plane& o, fmt::format_context& ctx) const {
    return fmt::format_to(ctx.out(), "{}", o.str());
  }
};
