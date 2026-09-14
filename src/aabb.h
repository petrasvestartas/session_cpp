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

/// Axis-aligned bounding box as center and half-size
struct AABB {
    double cx = 0.0;
    double cy = 0.0;
    double cz = 0.0;
    double hx = 0.0;
    double hy = 0.0;
    double hz = 0.0;

    AABB() = default;
    AABB(double cx, double cy, double cz, double hx, double hy, double hz);

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════

    /// Box of half-size inflate around point
    static AABB from_point(const Point& point, double inflate = 0.0);

    /// Tight box of points grown by inflate
    static AABB from_points(const std::vector<Point>& points, double inflate = 0.0);

    /// Tight box of the two ends grown by inflate
    static AABB from_line(const Line& line, double inflate = 0.0);

    /// Tight box of the vertices grown by inflate
    static AABB from_polyline(const Polyline& polyline, double inflate = 0.0);

    /// Tight box of the vertices grown by inflate
    static AABB from_mesh(const Mesh& mesh, double inflate = 0.0);

    /// Tight box of the points grown by inflate
    static AABB from_pointcloud(const PointCloud& pointcloud, double inflate = 0.0);

    /// Box of the control points, or of the curve extrema when tight
    static AABB from_nurbscurve(const NurbsCurve& curve, double inflate = 0.0, bool tight = false);

    /// Box of the control points grown by inflate
    static AABB from_nurbssurface(const NurbsSurface& surface, double inflate = 0.0);

    /// Box enclosing both a and b
    static AABB merge(const AABB& a, const AABB& b);

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════

    /// Center and half-size to 1e-6
    bool operator==(const AABB& other) const;
    bool operator!=(const AABB& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════

    Point min_point() const;
    Point max_point() const;
    Point center() const;

    /// Surface area
    double area() const;

    /// Length of the space diagonal
    double diagonal() const;
    double volume() const;

    /// No negative half-size
    bool is_valid() const;

    /// pt clamped to the box
    Point closest_point(const Point& pt) const;
    bool contains(const Point& pt) const;
    bool intersects(const AABB& other) const;

    /// Corner picked by the sign of each half-size
    Point corner(bool x_max, bool y_max, bool z_max) const;

    /// Bottom loop then top loop, counter-clockwise from +x+y
    std::array<Point, 8> corners() const;
    std::array<Point, 8> get_corners() const;

    /// Bottom loop, top loop, then the four verticals
    std::vector<Line> get_edges() const;

    /// Center offset by x, y, z
    Point point_at(double x, double y, double z) const;

    /// Grow every half-size by amount
    void inflate(double amount);

    /// Grow to enclose other
    void union_with(const AABB& other);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════

    /// "cx, cy, cz, hx, hy, hz"
    std::string str() const;

    /// "AABB(cx, cy, cz, hx, hy, hz)"
    std::string repr() const;

private:
    static constexpr int NUM_SAMPLES = 20;
    static constexpr int MAX_ITER = 20;

    /// Parameter in [t_lo, t_hi] where the axis derivative crosses zero, by Newton steps bracketed by bisection
    static double compute_extremum(const NurbsCurve& curve, int axis, double t_lo, double t_hi, double d_start);
};

std::ostream& operator<<(std::ostream& os, const AABB& aabb);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::AABB> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }
    auto format(const session_cpp::AABB& o, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", o.str());
    }
};
