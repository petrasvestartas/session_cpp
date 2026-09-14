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
#include <utility>
#include <vector>

namespace session_cpp {

/// A 3D line segment with display width, dash pattern and color
class Line {
public:
  std::string name = "my_line";
  double width = 1.0;
  std::vector<double> dash;
  Color linecolor = Color::black();

  Line();
  Line(double x0, double y0, double z0, double x1, double y1, double z1);

  /// Copy constructor (new guid, same data)
  Line(const Line &other);

  /// Copy assignment (new guid, same data)
  Line &operator=(const Line &other);

  /// Move keeps the guid; declaring it stops `return x;` from falling back to the guid-minting copy
  Line(Line &&other) noexcept = default;
  Line &operator=(Line &&other) noexcept = default;

  bool has_guid() const { return !_guid.empty(); }
  const std::string &guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
  std::string &guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

  /// Clear the guid so a fresh one mints lazily on next read
  void refresh_guid() { _guid.clear(); }

  // ═══════════════════════════════════════════════════════════════════════════
  // Static constructors
  // ═══════════════════════════════════════════════════════════════════════════

  static Line from_points(const Point &p1, const Point &p2);

  /// Line from point to point + vector
  static Line from_point_and_vector(const Point &point, const Vector &vector);

  /// Line from point along the normalized direction
  static Line from_point_direction_length(const Point &point, const Vector &direction, double length);

  /// Least-squares line through points by power-iteration PCA; length <= 0 spans the projected extent
  static Line fit_points(const std::vector<Point> &points, double length = 0.0);

  /// Named line from coordinates
  static Line with_name(const std::string &name, double x0, double y0, double z0, double x1, double y1, double z1);

  // ═══════════════════════════════════════════════════════════════════════════
  // Operators
  // ═══════════════════════════════════════════════════════════════════════════

  /// Coordinate by index (0=x0, 1=y0, 2=z0, 3=x1, 4=y1, 5=z1)
  double &operator[](int index);
  const double &operator[](int index) const;

  /// Same name, coordinates to 1e-6, width and linecolor; guid ignored
  bool operator==(const Line &other) const;
  bool operator!=(const Line &other) const;

  Line &operator+=(const Vector &other);
  Line &operator-=(const Vector &other);
  Line &operator*=(double factor);
  Line &operator/=(double factor);

  Line operator+(const Vector &other) const;
  Line operator-(const Vector &other) const;
  Line operator*(double factor) const;
  Line operator/(double factor) const;

  /// Flipped copy (end to start)
  Line operator-() const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Transformation
  // ═══════════════════════════════════════════════════════════════════════════

  /// Transform in place
  void transform(const Xform &xform);

  /// Transformed copy
  Line transformed(const Xform &xform) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Geometry
  // ═══════════════════════════════════════════════════════════════════════════

  double length() const;
  double squared_length() const;

  /// Vector from start to end
  Vector to_vector() const;

  /// Unit vector from start to end
  Vector to_direction() const;

  Point start() const;
  Point end() const;
  Point center() const;

  /// Point at parameter t (0 = start, 1 = end)
  Point point_at(double t) const;

  /// n evenly spaced points including both ends
  std::vector<Point> subdivide(int n) const;

  /// Points spaced approximately distance apart including both ends
  std::vector<Point> subdivide_by_distance(double distance) const;

  /// Parameter and closest point; limited clamps t to [0, 1]
  std::pair<double, Point> closest_point(const Point &point, bool limited = true) const;

  /// Midpoints of the paired starts and ends
  static void get_middle_line(const Point &line0_start, const Point &line0_end, const Point &line1_start, const Point &line1_end, Point &output_start, Point &output_end);

  /// Midpoints of the paired starts and ends
  static void get_middle_line(const Line &l0, const Line &l1, Line &out);

  /// Extreme sub-segment of line spanned by the projected points
  static bool from_projected_points(const Line &line, const std::vector<Point> &points, Line &out);

  /// Collinear overlap with other; false when none or a single point
  bool overlap(const Line &other, Line &out) const;

  /// Longer of the two midpoint pairings of overlap(other) and other.overlap(this)
  bool overlap_average(const Line &other, Line &out) const;

  /// Grow start by ext_start and end by ext_end
  void extend(double ext_start, double ext_end);

  /// Grow both ends by dist, or by proportion of the length when non-zero
  void extend_equally(double dist = 0.0, double proportion = 0.0);

  /// Shrink both ends by dist as a fraction of the length
  void scale(double dist);

  // ═══════════════════════════════════════════════════════════════════════════
  // JSON
  // ═══════════════════════════════════════════════════════════════════════════

  nlohmann::ordered_json jsondump() const;
  static Line jsonload(const nlohmann::json &data);
  std::string file_json_dumps() const;
  static Line file_json_loads(const std::string &json_string);
  void file_json_dump(const std::string &filename) const;
  static Line file_json_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // Protobuf
  // ═══════════════════════════════════════════════════════════════════════════

  std::string pb_dumps() const;
  static Line pb_loads(const std::string &data);
  void pb_dump(const std::string &filename) const;
  static Line pb_load(const std::string &filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // String
  // ═══════════════════════════════════════════════════════════════════════════

  /// "x0, y0, z0, x1, y1, z1"
  std::string str() const;

  /// "Line(name, x0, y0, z0, x1, y1, z1, Color(...), width)"
  std::string repr() const;

private:
  mutable std::string _guid;
  double _x0 = 0.0;
  double _y0 = 0.0;
  double _z0 = 0.0;
  double _x1 = 0.0;
  double _y1 = 0.0;
  double _z1 = 1.0;
};

std::ostream &operator<<(std::ostream &os, const Line &line);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::Line> {
  constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }
  auto format(const session_cpp::Line &o, fmt::format_context &ctx) const {
    return fmt::format_to(ctx.out(), "{}", o.str());
  }
};
