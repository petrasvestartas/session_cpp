#pragma once

#include "line.h"
#include "point.h"
#include "fmt/core.h"
#include <array>
#include <ostream>
#include <string>
#include <vector>

namespace session_cpp {

class Mesh;
class NurbsCurve;
class NurbsSurface;
class PointCloud;
class Polyline;
class Xform;

/// Axis-aligned bounding box as center and half-size.
class AABB {
private:
    static constexpr int NUM_SAMPLES = 20; // Samples per span when searching curve extrema.
    static constexpr int MAX_ITER = 20; // Newton iterations per extremum.

public:
    double cx = 0.0; // Center x.
    double cy = 0.0; // Center y.
    double cz = 0.0; // Center z.
    double hx = 0.0; // Half-size along x.
    double hy = 0.0; // Half-size along y.
    double hz = 0.0; // Half-size along z.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty box at the origin.
    AABB() = default;

    /// Construct from center and half-size.
    AABB(double cx, double cy, double cz, double hx, double hy, double hz);

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct the box of half-size inflate around point.
    static AABB from_point(const Point& point, double inflate = 0.0);

    /// Construct the tight box of points grown by inflate.
    static AABB from_points(const std::vector<Point>& points, double inflate = 0.0);

    /// Construct the tight box of the two ends grown by inflate.
    static AABB from_line(const Line& line, double inflate = 0.0);

    /// Construct the tight box of the vertices grown by inflate.
    static AABB from_polyline(const Polyline& polyline, double inflate = 0.0);

    /// Construct the tight box of the vertices grown by inflate.
    static AABB from_mesh(const Mesh& mesh, double inflate = 0.0);

    /// Construct the tight box of the points grown by inflate.
    static AABB from_pointcloud(const PointCloud& pointcloud, double inflate = 0.0);

    /// Construct the box of the control points, or of the curve extrema when tight.
    static AABB from_nurbscurve(const NurbsCurve& curve, double inflate = 0.0, bool tight = false);

    /// Construct the box of the control points grown by inflate.
    static AABB from_nurbssurface(const NurbsSurface& surface, double inflate = 0.0);

    /// Construct the box enclosing both a and b; an invalid box contributes nothing.
    static AABB merge(const AABB& a, const AABB& b);

    /// Construct the box nothing has grown yet: negative half-sizes, so is_valid is false.
    static AABB empty();

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Compare center and half-size to 1e-6.
    bool operator==(const AABB& other) const;

    /// Compare center and half-size to 1e-6.
    bool operator!=(const AABB& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the min corner.
    Point min_point() const;

    /// Return the max corner.
    Point max_point() const;

    /// Return the center.
    Point center() const;

    /// Return the surface area.
    double area() const;

    /// Return the length of the space diagonal, 0 when invalid.
    double diagonal() const;

    /// Return the volume.
    double volume() const;

    /// Return whether no half-size is negative.
    bool is_valid() const;

    /// Return pt clamped to the box.
    Point closest_point(const Point& pt) const;

    /// Return whether pt is inside or on the box.
    bool contains(const Point& pt) const;

    /// Return whether the boxes overlap or touch.
    bool intersects(const AABB& other) const;

    /// Return the corner picked by the sign of each half-size.
    Point corner(bool x_max, bool y_max, bool z_max) const;

    /// Return the bottom loop then top loop, counter-clockwise from +x+y.
    std::array<Point, 8> corners() const;

    /// Return the bottom loop then top loop, counter-clockwise from +x+y.
    std::array<Point, 8> get_corners() const;

    /// Return the bottom loop, top loop, then the four verticals.
    std::vector<Line> get_edges() const;

    /// Return the center offset by x, y, z.
    Point point_at(double x, double y, double z) const;

    /// Grow every half-size by amount.
    void inflate(double amount);

    /// Grow to enclose other; an invalid box contributes nothing.
    void union_with(const AABB& other);

    /// Grow to enclose (x, y, z); coordinates, not a Point, so a vertex loop allocates nothing.
    void union_with_point(double x, double y, double z);

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Replace the box by the box of its eight transformed corners.
    void transform(const Xform& xform);

    /// Return the box of the eight transformed corners; an invalid box stays invalid.
    AABB transformed(const Xform& xform) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "cx, cy, cz, hx, hy, hz".
    std::string str() const;

    /// Return "AABB(cx, cy, cz, hx, hy, hz)".
    std::string repr() const;

private:
    /// Return the parameter in [t_lo, t_hi] where the axis derivative crosses zero, by Newton steps bracketed by bisection.
    static double compute_extremum(const NurbsCurve& curve, int axis, double t_lo, double t_hi, double d_start);
};

/// Write the string representation to a stream.
std::ostream& operator<<(std::ostream& os, const AABB& aabb);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::AABB> {
    constexpr fmt::format_parse_context::iterator parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    fmt::format_context::iterator format(const session_cpp::AABB& aabb, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", aabb.str());
    }
};
