#pragma once
#include "color.h"
#include "guid.h"
#include "json.h"
#include "vector.h"
#include "xform.h"
#include "fmt/core.h"
#include <cmath>
#include <fstream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace session_cpp {

/// A 3D point with display width and color
class Point {
public:
  std::string name = "my_point";
  double width = 1.0;
  Color pointcolor = Color::black();

  Point() : _x(0.0), _y(0.0), _z(0.0) {}

  Point(double x, double y, double z, std::string name = "my_point")
      : name(std::move(name)), _x(x), _y(y), _z(z) {}

  /// Copy constructor (new guid, same data)
  Point(const Point &other);

  /// Copy assignment (new guid, same data)
  Point &operator=(const Point &other);

  /// Move keeps the guid; declaring it stops `return x;` from falling back to the guid-minting copy
  Point(Point &&other) noexcept = default;
  Point &operator=(Point &&other) noexcept = default;

  bool has_guid() const { return !_guid.empty(); }
  const std::string &guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
  std::string &guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Clear the guid so a fresh one mints lazily on next read
  void refresh_guid() { _guid.clear(); }

  // ═══════════════════════════════════════════════════════════════════════════
  // Operators
  // ═══════════════════════════════════════════════════════════════════════════

  /// Coordinate by index (0=x, 1=y, 2=z)
  double &operator[](int index);
  const double &operator[](int index) const;

  bool operator==(const Point &other) const;
  bool operator!=(const Point &other) const;

  Point &operator*=(double factor);
  Point &operator/=(double factor);
  Point &operator+=(const Vector &other);
  Point &operator-=(const Vector &other);

  Point operator*(double factor) const;
  Point operator/(double factor) const;
  Point operator+(const Vector &other) const;
  Point operator-(const Vector &other) const;
  Vector operator-(const Point &other) const;

  /// Coordinate-wise sum of two points
  static Point sum(const Point &p0, const Point &p1);

  /// Coordinate-wise difference of two points
  static Point sub(const Point &p0, const Point &p1);

  // ═══════════════════════════════════════════════════════════════════════════
  // Transformation
  // ═══════════════════════════════════════════════════════════════════════════

  /// Transform in place
  void transform(const Xform &xform);

  /// Transformed copy
  Point transformed(const Xform &xform) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Geometry
  // ═══════════════════════════════════════════════════════════════════════════

  /// True when a, b, c turn counter-clockwise in the xy plane
  static bool is_ccw(const Point &a, const Point &b, const Point &c);

  /// Mid point between this point and p
  Point mid_point(const Point &p) const;
  static Point mid_point(const Point &a, const Point &b);

  /// Distance to p, scaled to stay finite for large coordinates
  double distance(const Point &p, double double_min = 1e-12) const;
  static double distance(const Point &a, const Point &b, double double_min = 1e-12);

  /// Squared distance to p, scaled to stay finite for large coordinates
  double squared_distance(const Point &p, double double_min = 1e-12) const;
  static double squared_distance(const Point &a, const Point &b, double double_min = 1e-12);

  /// Point at parameter t in [0, 1] between a and b
  static Point lerp(const Point &a, const Point &b, double t);

  /// Evenly spaced points between from and to (kind: 0=no endpoints, 1=both, 2=start only)
  static std::vector<Point> interpolate(const Point &from, const Point &to, int steps, int kind = 0);

  /// Shoelace area of a polygon in the xy plane
  static double area(const std::vector<Point> &points);

  /// Area-weighted centroid of a quadrilateral
  static Point centroid_quad(const std::vector<Point> &vertices);

  /// Arithmetic mean of points; empty input returns the origin
  static Point centroid(const std::vector<Point> &points);

  /// Unsigned dihedral angle in degrees of edge pq between half-planes pqr and pqs
  static double dihedral_angle_deg(const Point &p, const Point &q, const Point &r, const Point &s);

  // ═══════════════════════════════════════════════════════════════════════════
  // JSON
  // ═══════════════════════════════════════════════════════════════════════════

  nlohmann::ordered_json jsondump() const;
  static Point jsonload(const nlohmann::json &data);
  std::string file_json_dumps() const;
  static Point file_json_loads(const std::string &json_string);
  void file_json_dump(const std::string &filename) const;
  static Point file_json_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // Protobuf
  // ═══════════════════════════════════════════════════════════════════════════

  std::string pb_dumps() const;
  static Point pb_loads(const std::string &data);
  void pb_dump(const std::string &filename) const;
  static Point pb_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // String
  // ═══════════════════════════════════════════════════════════════════════════

  /// "x, y, z"
  std::string str() const;

  /// "Point(name, x, y, z, Color(...), width)"
  std::string repr() const;

private:
  mutable std::string _guid;
  double _x = 0.0;
  double _y = 0.0;
  double _z = 0.0;
};

std::ostream &operator<<(std::ostream &os, const Point &point);

} // namespace session_cpp
