#pragma once
#include "plane.h"
#include "point.h"
#include "vector.h"
#include "color.h"
#include "xform.h"
#include "line.h"
#include "guid.h"
#include "json.h"
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <optional>

namespace session_cpp {


/**
 * @class Polyline
 * @brief A polyline defined by a collection of coordinates with an associated plane.
 *
 * Internally stores coordinates as a flat array [x0, y0, z0, x1, y1, z1, ...] for
 * efficient serialization. Provides Point-based API for compatibility.
 */
class Polyline {
public:
    std::string name = "my_polyline";
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::vector<double> _coords;  // Flat array [x0, y0, z0, x1, y1, z1, ...]
    mutable Plane plane;
    mutable bool _plane_dirty = true;
    double width = 1.0;
    Color linecolor = Color::black();

    Xform xform;

    /// Get plane (lazy — computed on first access from first 3 points)
    const Plane& get_plane() const;

    /// Default constructor
    Polyline();

    /// Copy constructor (creates a new guid while copying data)
    Polyline(const Polyline& other);

    /// Copy assignment (creates a new guid while copying data)
    Polyline& operator=(const Polyline& other);

    /// Constructor with points (converts to flat coords internally)
    explicit Polyline(const std::vector<Point>& pts);

    /// Constructor with flat coords
    static Polyline from_coords(const std::vector<double>& coords);

    /// Create a regular polygon with given number of sides and radius.
    static Polyline from_sides(int sides, double radius = 1.0, bool close = false);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Core Methods (str, repr, duplicate, eq)
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Returns minimal string representation
    std::string str() const;

    /// Returns detailed string representation
    std::string repr() const;

    /// Equality operator (compares values, ignores GUIDs)
    bool operator==(const Polyline& other) const;
    bool operator!=(const Polyline& other) const;

    /// Index operator (returns point at index)
    Point operator[](size_t index) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Point Access
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Returns the number of points in the polyline
    size_t point_count() const;

    /// Returns the number of points (alias for point_count)
    size_t len() const;

    /// Returns all points as Point objects
    std::vector<Point> get_points() const;

    /// Returns true if the polyline has no points
    bool is_empty() const;

    /// Returns the number of segments (n-1 for n points)
    size_t segment_count() const;

    /// Returns all segments as Line objects
    std::vector<Line> get_lines() const;

    /// Calculates the total length of the polyline
    double length() const;

    /// Returns the point at the given index
    Point get_point(size_t index) const;

    /// Sets the point at the given index
    void set_point(size_t index, const Point& point);

    /// Adds a point to the end of the polyline
    void add_point(const Point& point);

    /// Inserts a point at the specified index
    void insert_point(size_t index, const Point& point);

    /// Removes and returns the point at the specified index
    bool remove_point(size_t index, Point& out_point);

    /// Reverses the order of points in the polyline
    void reverse();

    /// Returns a new polyline with reversed point order
    Polyline reversed() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Operators
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Translates all points by a vector (in-place)
    Polyline& operator+=(const Vector& v);

    /// Translates all points by a vector (returns new polyline)
    Polyline operator+(const Vector& v) const;

    /// Translates all points by negative vector (in-place)
    Polyline& operator-=(const Vector& v);

    /// Translates all points by negative vector (returns new polyline)
    Polyline operator-(const Vector& v) const;

    /// Multiply all coordinates by scalar (in-place)
    Polyline& operator*=(double factor);

    /// Multiply polyline by scalar (returns new polyline)
    Polyline operator*(double factor) const;

    /// Divide all coordinates by scalar (in-place)
    Polyline& operator/=(double factor);

    /// Divide polyline by scalar (returns new polyline)
    Polyline operator/(double factor) const;

    /// Negate polyline (reverse point order)
    Polyline operator-() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    void transform();
    Polyline transformed() const;

    /**
     * @brief Return a copy of this polyline with `xf` applied to every point.
     *
     * Verbatim of the inlined `xform_polyline()` helper from main_5.cpp:
     * applies a column-major affine transformation matrix.
     *
     * Named `transformed_xform` (not an overload of `transformed()`) so the
     * Rust + Python ports can share the same name (Rust + Python don't
     * support overload-by-signature).
     *
     * @param xf Column-major affine transformation matrix.
     * @return New polyline with transformed coordinates.
     */
    Polyline transformed_xform(const Xform& xf) const;

    /**
     * @brief Translate every point of this polyline by `v` (in place).
     *
     * Mirrors the free function `move(std::vector<Point>&, const Vector&)`
     * in this header — promoted to a class method for ergonomics.
     *
     * Named `translate` (not `move`) for cross-language parity: Rust's
     * `move` is a keyword, and `translate` is more descriptive.
     *
     * @param v Translation vector.
     */
    void translate(const Vector& v);

    /**
     * @brief Slide both endpoints of edge `edge_idx` outward (or inward
     *        for negative `distance`) along the edge tangent.
     *
     * For closed polylines, the closing-duplicate vertex is kept in sync
     * (i.e. modifying point 0 also modifies point n-1 if they coincide).
     * Wood's `merge_joints` calls this in opposite-edge pairs (0+2, 1+3)
     * to scale rectangle joint volumes uniformly along their two
     * principal axes.
     *
     * @param edge_idx Index of the first vertex of the edge.
     * @param distance Signed distance to extend each endpoint by.
     */
    void extend_edge_equally(size_t edge_idx, double distance);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to JSON-serializable object (uses compact coords format)
    nlohmann::ordered_json jsondump() const;

    /// Create polyline from JSON data (supports both coords and legacy points format)
    static Polyline jsonload(const nlohmann::json& data);

    /// Serialize to JSON file
    void json_dump(const std::string& filename) const;

    /// Deserialize from JSON file
    static Polyline json_load(const std::string& filename);

    /// Convert to JSON string
    std::string json_dumps() const;

    /// Load from JSON string
    static Polyline json_loads(const std::string& json_string);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to protobuf binary string
    std::string pb_dumps() const;

    /// Load from protobuf binary string
    static Polyline pb_loads(const std::string& data);

    /// Write protobuf to file
    void pb_dump(const std::string& filename) const;

    /// Read protobuf from file
    static Polyline pb_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Geometric Utilities
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Shift polyline points by specified number of positions
    void shift(int times);

    /// Calculate squared length of polyline (faster, no sqrt)
    double length_squared() const;

    /// Get point at parameter t along a line segment (t=0 is start, t=1 is end)
    static Point point_at(const Point& start, const Point& end, double t);

    /// Find closest point on line segment to given point, returns parameter t
    static void closest_point_to_line(const Point& point, const Point& line_start, 
                                     const Point& line_end, double& t);

    /// Check if two line segments overlap and return the overlapping segment
    static bool line_line_overlap(const Point& line0_start, const Point& line0_end,
                                 const Point& line1_start, const Point& line1_end,
                                 Point& overlap_start, Point& overlap_end);

    /// Calculate average of two line segments
    static void line_line_average(const Point& line0_start, const Point& line0_end,
                                 const Point& line1_start, const Point& line1_end,
                                 Point& output_start, Point& output_end);

    /// Calculate overlap average of two line segments
    static void line_line_overlap_average(const Point& line0_start, const Point& line0_end,
                                         const Point& line1_start, const Point& line1_end,
                                         Point& output_start, Point& output_end);

    /// Create line from projected points onto a base line
    static bool line_from_projected_points(const Point& line_start, const Point& line_end,
                                          const std::vector<Point>& points,
                                          Point& output_start, Point& output_end);

    /// Find closest distance and point from a point to this polyline
    double closest_distance_and_point(const Point& point, size_t& edge_id, Point& closest_point) const;

    /// Check if polyline is closed (first and last points are the same)
    bool is_closed() const;
    Polyline closed() const;

    /// Calculate center point of polyline
    Point center() const;

    /// Get average plane from polyline points
    void get_average_plane(Point& origin, Vector& x_axis, Vector& y_axis, Vector& z_axis) const;

    /// Winding-number point-in-polygon test. p.x/y tested; polygon vertex z ignored.
    bool point_in_polygon_2d(const Point& p) const;

    /// Get fast plane calculation from polyline
    void get_fast_plane(Point& origin, Plane& pln) const;

    /// Extend polyline segment
    void extend_segment(int segment_id, double dist0, double dist1, 
                       double proportion0 = 0.0, double proportion1 = 0.0);

    /// Extend segment equally on both ends (static utility)
    static void extend_segment_equally(Point& segment_start, Point& segment_end,
                                      double dist, double proportion = 0.0);

    /// Extend a line segment independently at each end by a real (normalized) distance
    static void extend_line_segment(Point& start, Point& end, double d0, double d1);

    /// Shrink a line segment equally from both ends by a fraction of its length (not normalized)
    static void shrink_line_segment(Point& start, Point& end, double dist);

    /// Extend polyline segment equally
    void extend_segment_equally(int segment_id, double dist, double proportion = 0.0);

    /// Check if polyline is clockwise oriented
    bool is_clockwise(const Plane& pln) const;

    /// Get convex/concave corners of polyline
    void get_convex_corners(std::vector<bool>& convex_or_concave) const;

    /// Interpolate between two polylines
    static Polyline tween_two_polylines(const Polyline& polyline0, const Polyline& polyline1,
                                       double weight);

    /// Linear interpolation between two points.
    /// kind: 0=no endpoints, 1=both endpoints, 2=start only
    static std::vector<Point> interpolate_points(const Point& from, const Point& to, int steps, int kind = 0);

    /// 2D convex hull (quickhull) in the polygon's local plane.
    static Polyline quick_hull(const Polyline& polygon);

    /// Minimum-area bounding rectangle via rotating calipers; returns closed 5-pt Polyline.
    static std::optional<Polyline> bounding_rectangle(const Polyline& polygon);

    /// Grid of interior points; offset_dist ignored (no Clipper2); div_dist = grid spacing.
    static std::vector<Point> grid_of_points_in_polygon(const Polyline& polygon,
                                                        double offset_dist, double div_dist,
                                                        size_t max_pts = 100);

    /// Boolean operation on two closed planar polylines (2D, uses x,y only).
    /// clip_type: 0=intersection, 1=union, 2=difference (a minus b).
    /// Returns 0+ result polygons. Uses Vatti scanline algorithm (ported from Clipper2).
    static std::vector<Polyline> boolean_op(const Polyline& a, const Polyline& b, int clip_type);

    /// Boolean operation on two 3D coplanar polylines.
    /// Projects to plane's local 2D, runs boolean, inverse-transforms back to 3D.
    static std::vector<Polyline> boolean_op(const Polyline& a, const Polyline& b, const Plane& plane, int clip_type);

    /// Merge consecutive collinear segments (in-place); closed polyline wraps around
    void merge_collinear(double tol = Tolerance::APPROXIMATION);

    /// Simplify a point list using Ramer-Douglas-Peucker
    static std::vector<Point> simplify_points(const std::vector<Point>& points, double tolerance);

    /// Simplify this polyline using Ramer-Douglas-Peucker
    Polyline simplify(double tolerance) const;

private:
    mutable std::string _guid; ///< Lazily generated unique identifier

    /// Helper to recompute plane when points change
    void recompute_plane_if_needed();

    /// Calculate average normal from polyline points
    void average_normal(Vector& average_normal) const;

    static double simplify_perp_dist(const Point& pt, const Point& line_start, const Point& line_end);
    static void simplify_rdp(const std::vector<Point>& points, int start, int end, double tolerance, std::vector<bool>& keep);
};

///////////////////////////////////////////////////////////////////////////////////////////
// Stream operator
///////////////////////////////////////////////////////////////////////////////////////////

std::ostream& operator<<(std::ostream& os, const Polyline& polyline);

// ── Free functions on raw std::vector<Point> polylines (CGAL_Polyline compat) ─

/// Cyclic shift: rotate points left by `times` positions (in-place)
void shift(std::vector<Point>& pline, int times);
/// Total arc-length of a polyline
double polyline_length(const std::vector<Point>& pline);
/// Squared arc-length (no sqrt)
double polyline_length_squared(const std::vector<Point>& pline);
/// True if last ≈ first point
bool is_closed(const std::vector<Point>& pline);
/// Arithmetic mean of all points
Point center(const std::vector<Point>& pline);
/// Center expressed as Vector from world origin
Vector center_vec(const std::vector<Point>& pline);
/// Best-fit plane axes: axes[0]=origin, [1]=x, [2]=y, [3]=z
void get_average_plane(const std::vector<Point>& pline, Vector (&axes)[4]);
/// Quick plane from first distinct points
void get_fast_plane(const std::vector<Point>& pline, Point& origin, Plane& pln);
/// Apply affine transform to all points in-place
void transform(std::vector<Point>& pline, const Xform& xf);
/// Translate all points by direction vector
void move(std::vector<Point>& pline, const Vector& dir);
/// True if polygon is clockwise relative to given plane
bool is_clockwise(std::vector<Point>& pline, const Plane& pln);
/// Reverse point order in-place
void flip(std::vector<Point>& pline);
/// Compute convex/concave corner flags
void get_convex_corners(const std::vector<Point>& pline, std::vector<bool>& flags);
/// Linearly interpolate between two raw polylines; returns result
std::vector<Point> tween_two_polylines(const std::vector<Point>& p0,
                                       const std::vector<Point>& p1, double w);
/// Average of two line segments
void line_line_average(const Line& l0, const Line& l1, Line& out);
/// Overlap segment of two co-linear lines; returns false if disjoint
bool line_line_overlap(const Line& l0, const Line& l1, Line& out);
/// Overlap-average of two line segments
void line_line_overlap_average(const Line& l0, const Line& l1, Line& out);
/// Overlap-average overload for raw 2-point polylines
void line_line_overlap_average(const std::vector<Point>& l0,
                               const std::vector<Point>& l1, Line& out);
/// Project points onto line, return extreme sub-segment
bool line_from_projected_points(const Line& line, const std::vector<Point>& pts,
                                Line& out);
/// Extend line segment: grow start by d0, grow end by d1 (in-place)
void extend_line(Line& line, double d0, double d1);
/// Extend equally: absolute dist or fraction proportion
void extend_equally(Line& line, double dist = 0, double proportion = 0);
/// Scale line to given length (shrink symmetrically from both ends)
void scale_line(Line& line, double dist);
/// Extend a polyline segment in-place
void extend(std::vector<Point>& pline, int sID, double d0, double d1,
            double proportion0 = 0, double proportion1 = 0);
/// Closest distance and point from pt to polyline
double closest_distance_and_point(const Point& pt, const std::vector<Point>& poly,
                                  size_t& edge_id, Point& closest_point);
/// Midpoint segment between two parallel line segments
void get_middle_line(const Line& l0, const Line& l1, Line& out);

} // namespace session_cpp

// fmt formatter specialization for Polyline
template <> struct fmt::formatter<session_cpp::Polyline> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    auto format(const session_cpp::Polyline& polyline, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "Polyline(guid={}, name={}, points={})",
                            polyline.guid(), polyline.name, polyline.point_count());
    }
};
