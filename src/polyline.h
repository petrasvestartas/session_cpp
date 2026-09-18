#pragma once
#include "color.h"
#include "guid.h"
#include "json.h"
#include "line.h"
#include "plane.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
#include "fmt/core.h"
#include <array>
#include <optional>
#include <ostream>
#include <string>
#include <tuple>
#include <vector>

namespace session_cpp {

/// A polyline stored as flat coordinates [x0, y0, z0, x1, y1, z1, ...] with a lazily computed plane.
class Polyline {
public:
  std::string name = "my_polyline"; // Polyline name.
  std::vector<double> _coords; // Flat [x, y, z, ...].
  mutable Plane plane; // Lazily computed plane, see get_plane.
  mutable bool _plane_dirty = true; // True until get_plane recomputes.
  double width = 1.0; // Display width.
  std::vector<double> dash; // Dash pattern lengths.
  Color linecolor = Color::black(); // Display color.

  /// Construct an empty polyline.
  Polyline();

  /// Construct from points.
  explicit Polyline(const std::vector<Point>& pts);

  /// Copy with a new guid and the same data.
  Polyline(const Polyline& other);

  /// Copy-assign with a new guid and the same data.
  Polyline& operator=(const Polyline& other);

  /// Move while preserving the guid.
  Polyline(Polyline&& other) noexcept = default;

  /// Move-assign while preserving the guid.
  Polyline& operator=(Polyline&& other) noexcept = default;

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

  // ═══════════════════════════════════════════════════════════════════════════
  // Static constructors
  // ═══════════════════════════════════════════════════════════════════════════

  /// Construct from flat [x, y, z, ...] coordinates.
  static Polyline from_coords(const std::vector<double>& coords);

  /// Construct a regular polygon of sides around the origin in the XY plane.
  static Polyline from_sides(int sides, double radius = 1.0, bool close = false);

  /// Construct a rectangle with its corner at origin, sides along x_axis and y_axis.
  static Polyline rectangle(const Point& origin, const Vector& x_axis, const Vector& y_axis, double width, double height, bool close = true);

  /// Construct the quadratic Bezier through p0, p1, p2 sampled at divisions points.
  static Polyline quadratic_points(const Point& p0, const Point& p1, const Point& p2, int divisions = 7);

  // ═══════════════════════════════════════════════════════════════════════════
  // Accessors
  // ═══════════════════════════════════════════════════════════════════════════

  /// Return the number of points.
  size_t point_count() const;

  /// Return the number of points.
  size_t len() const;

  /// Return whether the polyline has no points.
  bool is_empty() const;

  /// Return the number of segments.
  size_t segment_count() const;

  /// Return the point at index, or the origin when out of range.
  Point get_point(size_t index) const;

  /// Return all points.
  std::vector<Point> get_points() const;

  /// Return one line per segment.
  std::vector<Line> get_lines() const;

  /// Return the plane from the first non-collinear triple, computed on first access.
  const Plane& get_plane() const;

  /// Return the total length.
  double length() const;

  /// Return the sum of squared segment lengths.
  double length_squared() const;

  /// Return whether the first and last points coincide.
  bool is_closed() const;

  /// Return a copy with the first point appended when open.
  Polyline closed() const;

  /// Return the average of the points, closing duplicate excluded.
  Point center() const;

  /// Compute the frame with origin at center, x along the first segment, z the average normal.
  void get_average_plane(Point& origin, Vector& x_axis, Vector& y_axis, Vector& z_axis) const;

  /// Compute the plane with origin at the first point, normal the average normal.
  void get_fast_plane(Point& origin, Plane& pln) const;

  /// Compute one flag per corner, true when convex against the average normal.
  void get_convex_corners(std::vector<bool>& convex_or_concave) const;

  /// Return the shoelace sign of the points projected onto pln.
  bool is_clockwise(const Plane& pln) const;

  /// Return the winding-number test on x and y.
  bool point_in_polygon_2d(const Point& p) const;

  /// Return the distance to the nearest segment, with its index and the closest point.
  double closest_distance_and_point(const Point& point, size_t& edge_id, Point& closest_point) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Mutators
  // ═══════════════════════════════════════════════════════════════════════════

  /// Set the point at index.
  void set_point(size_t index, const Point& point);

  /// Append a point.
  void add_point(const Point& point);

  /// Insert a point at index.
  void insert_point(size_t index, const Point& point);

  /// Remove the point at index into out_point; false when out of range.
  bool remove_point(size_t index, Point& out_point);

  /// Reverse the point order in place.
  void reverse();

  /// Return a reversed copy.
  Polyline reversed() const;

  /// Rotate the points by times positions, keeping the closing duplicate.
  void shift(int times);

  /// Translate in place.
  void translate(const Vector& v);

  /// Return a translated copy.
  Polyline translated(const Vector& v) const;

  /// Move the segment ends by dist0 and dist1, or by proportions of its length when non-zero.
  void extend_segment(int segment_id, double dist0, double dist1, double proportion0 = 0.0, double proportion1 = 0.0);

  /// Move both segment ends by dist, or by proportion of its length when non-zero.
  void extend_segment_equally(int segment_id, double dist, double proportion = 0.0);

  /// Slide both ends of edge edge_idx outward by distance, keeping the closing duplicate in sync.
  void extend_edge_equally(size_t edge_idx, double distance);

  /// Drop points whose neighbours are collinear within tol; closed polylines wrap around.
  void merge_collinear(double tol = Tolerance::APPROXIMATION);

  /// Drop consecutive points closer than tol.
  void remove_consecutive_duplicates(double tol = Tolerance::APPROXIMATION);

  /// Return a Ramer-Douglas-Peucker copy.
  Polyline simplify(double tolerance) const;

  /// Return the part on one side of plane; flip picks the normal side, unset keeps the arc-length midpoint side.
  Polyline cut_by_plane(const Plane& plane, std::optional<bool> flip = std::nullopt) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Operators
  // ═══════════════════════════════════════════════════════════════════════════

  /// Compare name, coordinates to 1e-6, width and linecolor; guid ignored.
  bool operator==(const Polyline& other) const;

  /// Compare name, coordinates to 1e-6, width and linecolor; guid ignored.
  bool operator!=(const Polyline& other) const;

  /// Return the point at index.
  Point operator[](size_t index) const;

  /// Translate in place.
  Polyline& operator+=(const Vector& v);

  /// Translate back in place.
  Polyline& operator-=(const Vector& v);

  /// Scale every point in place.
  Polyline& operator*=(double factor);

  /// Divide every point in place.
  Polyline& operator/=(double factor);

  /// Return a translated copy.
  Polyline operator+(const Vector& v) const;

  /// Return a copy translated back.
  Polyline operator-(const Vector& v) const;

  /// Return a scaled copy.
  Polyline operator*(double factor) const;

  /// Return a divided copy.
  Polyline operator/(double factor) const;

  /// Return a reversed copy.
  Polyline operator-() const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Transformation
  // ═══════════════════════════════════════════════════════════════════════════

  /// Transform in place.
  void transform(const Xform& xform);

  /// Return a transformed copy.
  Polyline transformed(const Xform& xform) const;

  // ═══════════════════════════════════════════════════════════════════════════
  // Segment utilities
  // ═══════════════════════════════════════════════════════════════════════════

  /// Return the point at parameter t (0 = start, 1 = end).
  static Point point_at(const Point& start, const Point& end, double t);

  /// Compute the parameter t of the closest point on the line through line_start and line_end.
  static void closest_point_to_line(const Point& point, const Point& line_start, const Point& line_end, double& t);

  /// Compute the collinear overlap of two segments; false when none or a single point.
  static bool line_line_overlap(const Point& line0_start, const Point& line0_end, const Point& line1_start, const Point& line1_end, Point& overlap_start, Point& overlap_end);

  /// Compute the midpoints of the paired starts and ends.
  static void line_line_average(const Point& line0_start, const Point& line0_end, const Point& line1_start, const Point& line1_end, Point& output_start, Point& output_end);

  /// Compute the longer of the two midpoint pairings of the mutual overlaps.
  static void line_line_overlap_average(const Point& line0_start, const Point& line0_end, const Point& line1_start, const Point& line1_end, Point& output_start, Point& output_end);

  /// Compute the extreme sub-segment of the line spanned by the projected points; false when a single point.
  static bool line_from_projected_points(const Point& line_start, const Point& line_end, const std::vector<Point>& points, Point& output_start, Point& output_end);

  /// Move both ends by dist, or by proportion of the length when non-zero.
  static void extend_segment_equally(Point& segment_start, Point& segment_end, double dist, double proportion = 0.0);

  /// Move start by d0 and end by d1 along the unit direction.
  static void extend_line_segment(Point& start, Point& end, double d0, double d1);

  /// Move both ends inward by dist as a fraction of the length.
  static void shrink_line_segment(Point& start, Point& end, double dist);

  // ═══════════════════════════════════════════════════════════════════════════
  // Polygon utilities
  // ═══════════════════════════════════════════════════════════════════════════

  /// Return the pointwise blend; polyline0 when the counts differ.
  static Polyline tween_two_polylines(const Polyline& polyline0, const Polyline& polyline1, double weight);

  /// Return steps points between from and to; kind 0 none, 1 both, 2 start endpoint.
  static std::vector<Point> interpolate_points(const Point& from, const Point& to, int steps, int kind = 0);

  /// Return the convex hull in the polygon's average plane.
  static Polyline quick_hull(const Polyline& polygon);

  /// Return the minimum-area rectangle of the hull as a closed 5-point polyline.
  static std::optional<Polyline> bounding_rectangle(const Polyline& polygon);

  /// Return a grid of interior points spaced div_dist, on the polygon miter-offset by offset_dist.
  static std::vector<Point> grid_of_points_in_polygon(const Polyline& polygon, double offset_dist, double div_dist, size_t max_pts = 100);

  /// Return the largest inscribed circle of polylines[0] minus the holes polylines[1..]: center, plane, radius.
  static std::tuple<Point, Plane, double> polylabel(const std::vector<Polyline>& polylines, double precision = 1.0);

  /// Return division points on the polylabel circle scaled by scale, oriented to the closest edge or division_direction_in_3d.
  static std::vector<Point> polylabel_circle_division_points(const Vector& division_direction_in_3d, const std::vector<Polyline>& polylines, int division = 4, double scale = 0.75, double precision = 1.0, bool orient_to_closest_edge = true);

  /// Return the Vatti boolean of two closed polylines on x and y; clip_type 0 intersection, 1 union, 2 a minus b.
  static std::vector<Polyline> boolean_op(const Polyline& a, const Polyline& b, int clip_type);

  /// Return the boolean of two coplanar polylines in plane's local frame.
  static std::vector<Polyline> boolean_op(const Polyline& a, const Polyline& b, const Plane& plane, int clip_type);

  /// Return the Ramer-Douglas-Peucker simplification of a point list.
  static std::vector<Point> simplify_points(const std::vector<Point>& points, double tolerance);

  /// Compute male rect0 and female rect1 cross-sections of radius about p along segment_vector; flip_male rotates the corners.
  static void two_rects_from_frame(const Point& p, const Vector& segment_vector, const Vector& zaxis, bool middle, double radius, double length, int flip_male, Polyline& rect0, Polyline& rect1);

  /// Cut two closed 5-point rectangles at plane, keeping the side on the positive half; false when a long edge misses the plane.
  static bool trim_rectangles_by_plane(Polyline& first, Polyline& second, const Plane& plane);

  // ═══════════════════════════════════════════════════════════════════════════
  // JSON
  // ═══════════════════════════════════════════════════════════════════════════

  /// Serialize to a JSON object.
  nlohmann::ordered_json jsondump() const;

  /// Deserialize from a JSON object.
  static Polyline jsonload(const nlohmann::json& data);

  /// Serialize to a JSON string.
  std::string file_json_dumps() const;

  /// Deserialize from a JSON string.
  static Polyline file_json_loads(const std::string& json_string);

  /// Write to a JSON file.
  void file_json_dump(const std::string& filename) const;

  /// Read from a JSON file.
  static Polyline file_json_load(const std::string& filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // Protobuf
  // ═══════════════════════════════════════════════════════════════════════════

  /// Serialize to protobuf bytes.
  std::string pb_dumps() const;

  /// Deserialize from protobuf bytes.
  static Polyline pb_loads(const std::string& data);

  /// Write to a protobuf file.
  void pb_dump(const std::string& filename) const;

  /// Read from a protobuf file.
  static Polyline pb_load(const std::string& filename);

  // ═══════════════════════════════════════════════════════════════════════════
  // String
  // ═══════════════════════════════════════════════════════════════════════════

  /// Return "[(x0, y0, z0), (x1, y1, z1), ...]".
  std::string str() const;

  /// Return "Polyline(name, N points)".
  std::string repr() const;

private:
  mutable std::string _guid; // Lazy guid.

  /// Recompute plane when _plane_dirty.
  void recompute_plane_if_needed();

  /// Compute the average normal by Newell's method.
  void average_normal(Vector& avg_normal) const;

  /// Return the point at arc length distance from the start.
  Point point_at_length(double distance) const;

  /// Compute the 2D coordinates of the points in the frame (origin, x_axis, y_axis).
  void project_to_plane(const Point& origin, const Vector& x_axis, const Vector& y_axis, std::vector<std::array<double, 2>>& pts2d) const;

  /// Return the 3D point of (u, v) in the frame (origin, x_axis, y_axis).
  static Point unproject(const Point& origin, const Vector& x_axis, const Vector& y_axis, double u, double v);

  /// Add the hull points of pts right of the segment (a, b) to hull.
  static void quick_hull_recurse(const std::vector<std::array<double, 2>>& pts, double ax, double ay, double bx, double by, std::vector<std::array<double, 2>>& hull);

  /// Miter-offset a 2D polygon in place by offset_dist.
  static void offset_polygon_2d(std::vector<std::array<double, 2>>& poly2d, double offset_dist);

  /// Return whether (px, py) is inside poly2d by ray crossing.
  static bool point_in_polygon(const std::vector<std::array<double, 2>>& poly2d, double px, double py);

  /// Find the polyline and edge closest to center.
  static bool closest_edge(const Point& center, const std::vector<Polyline>& polylines, size_t& edge_i, size_t& edge_j);

  /// Return pl projected into plane's local frame as 2D.
  static Polyline boolean_project(const Polyline& pl, const Plane& plane);

  /// Reverse p2d in place when it winds clockwise.
  static void ensure_ccw(Polyline& p2d);

  /// Return the perpendicular distance from pt to the line through line_start and line_end.
  static double simplify_perp_dist(const Point& pt, const Point& line_start, const Point& line_end);

  /// Mark the points to keep between start and end by recursive Ramer-Douglas-Peucker.
  static void simplify_rdp(const std::vector<Point>& points, int start, int end, double tolerance, std::vector<bool>& keep);
};

/// Write the polyline string to a stream.
std::ostream& operator<<(std::ostream& os, const Polyline& polyline);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::Polyline> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }
  auto format(const session_cpp::Polyline& o, fmt::format_context& ctx) const {
    return fmt::format_to(ctx.out(), "{}", o.repr());
  }
};
