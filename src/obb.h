#pragma once
#include "aabb.h"
#include "guid.h"
#include "json.h"
#include "line.h"
#include "plane.h"
#include "point.h"
#include "vector.h"
#include "xform.h"
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

/// Oriented bounding box as center, three axes and half-size
class OBB {
public:
    Point center;
    Vector x_axis;
    Vector y_axis;
    Vector z_axis;
    Vector half_size;
    std::string name = "my_obb";

    OBB();
    OBB(const Point& center, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis, const Vector& half_size);

    /// Box on the plane frame with full sizes dx, dy, dz
    OBB(const Plane& plane, double dx, double dy, double dz);

    /// Copy constructor (new guid, same data)
    OBB(const OBB& other);

    /// Copy assignment (new guid, same data)
    OBB& operator=(const OBB& other);

    /// Move keeps the guid; declaring it stops `return x;` from falling back to the guid-minting copy
    OBB(OBB&& other) noexcept = default;
    OBB& operator=(OBB&& other) noexcept = default;

    bool has_guid() const { return !_guid.empty(); }
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }

    /// Clear the guid so a fresh one mints lazily on next read
    void refresh_guid() { _guid.clear(); }

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════

    /// World-aligned box of half-size inflate around point
    static OBB from_point(const Point& point, double inflate = 0.0);

    /// World-aligned tight box of points grown by inflate
    static OBB from_points(const std::vector<Point>& points, double inflate = 0.0);

    /// Tight box of points in the plane frame grown by inflate
    static OBB from_points(const std::vector<Point>& points, const Plane& plane, double inflate = 0.0);

    static OBB from_line(const Line& line, double inflate = 0.0);
    static OBB from_line(const Line& line, const Plane& plane, double inflate = 0.0);
    static OBB from_polyline(const Polyline& polyline, double inflate = 0.0);
    static OBB from_polyline(const Polyline& polyline, const Plane& plane, double inflate = 0.0);
    static OBB from_mesh(const Mesh& mesh, double inflate = 0.0);
    static OBB from_mesh(const Mesh& mesh, const Plane& plane, double inflate = 0.0);
    static OBB from_pointcloud(const PointCloud& pointcloud, double inflate = 0.0);
    static OBB from_pointcloud(const PointCloud& pointcloud, const Plane& plane, double inflate = 0.0);

    /// Box of the control points, or of the curve extrema when tight
    static OBB from_nurbscurve(const NurbsCurve& curve, double inflate = 0.0, bool tight = false);
    static OBB from_nurbscurve(const NurbsCurve& curve, const Plane& plane, double inflate = 0.0, bool tight = false);
    static OBB from_nurbssurface(const NurbsSurface& surface, double inflate = 0.0);
    static OBB from_nurbssurface(const NurbsSurface& surface, const Plane& plane, double inflate = 0.0);

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════

    /// Same name, center, axes and half-size to 1e-6; guid ignored
    bool operator==(const OBB& other) const;
    bool operator!=(const OBB& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Transform center and axes in place
    void transform(const Xform& xform);

    /// Transformed copy
    OBB transformed(const Xform& xform) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════

    /// World-aligned box enclosing the corners
    AABB aabb() const;
    Point min_point() const;
    Point max_point() const;

    /// Surface area
    double area() const;

    /// Length of the space diagonal
    double diagonal() const;
    double volume() const;

    /// No negative half-size
    bool is_valid() const;

    /// pt clamped to the box in its own frame
    Point closest_point(const Point& pt) const;
    bool contains(const Point& pt) const;

    /// Corner picked by the sign of each half-size
    Point corner(bool x_max, bool y_max, bool z_max) const;

    /// Bottom loop then top loop, counter-clockwise from +x+y
    std::array<Point, 8> corners() const;
    std::array<Point, 8> get_corners() const;

    /// Bottom loop, top loop, then the four verticals
    std::vector<Line> get_edges() const;

    /// Bottom loop and top loop, each closed by repeating its first corner
    std::array<Point, 10> two_rectangles() const;

    /// Center offset by x, y, z along the axes
    Point point_at(double x, double y, double z) const;

    /// Grow every half-size by amount
    void inflate(double amount);

    /// Grow in place to enclose the corners of other
    void union_with(const OBB& other);

    // ═══════════════════════════════════════════════════════════════════════════
    // Collision
    // ═══════════════════════════════════════════════════════════════════════════

    /// Separating axis test (collides_with_rtcd)
    bool collides_with(const OBB& other) const;

    /// AABB rejection before collides_with
    bool collides_with_broad(const OBB& other) const;

    /// Fifteen-axis test in the frame of this box (Real-Time Collision Detection)
    bool collides_with_rtcd(const OBB& other) const;

    /// Fifteen-axis test by projected extents
    bool collides_with_naive(const OBB& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════

    nlohmann::ordered_json jsondump() const;
    static OBB jsonload(const nlohmann::json& data);
    std::string file_json_dumps() const;
    static OBB file_json_loads(const std::string& json_string);
    void file_json_dump(const std::string& filename) const;
    static OBB file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════

    std::string pb_dumps() const;
    static OBB pb_loads(const std::string& data);
    void pb_dump(const std::string& filename) const;
    static OBB pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════

    /// "center\nx_axis\ny_axis\nz_axis\nhalf_size"
    std::string str() const;

    /// "OBB(name, center, x_axis, y_axis, z_axis, half_size)"
    std::string repr() const;

private:
    static constexpr int NUM_SAMPLES = 20;
    static constexpr int MAX_ITER = 20;
    mutable std::string _guid;

    /// World-aligned box with the center and half-size of aabb
    static OBB from_aabb(const AABB& aabb);

    /// Parameter in [t_lo, t_hi] where the derivative along axis crosses zero, by Newton steps bracketed by bisection
    static double compute_extremum(const NurbsCurve& curve, const Vector& axis, double t_lo, double t_hi, double d_start);

    /// Extents of both boxes projected on axis do not reach their center distance
    static bool separating_plane_exists(const Vector& relative_position, const Vector& axis, const OBB& box1, const OBB& box2);
};

std::ostream& operator<<(std::ostream& os, const OBB& obb);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::OBB> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }
    auto format(const session_cpp::OBB& o, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", o.str());
    }
};
