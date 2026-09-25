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

namespace session_proto {
class BoundingBox;
}

namespace session_cpp {

class Mesh;
class NurbsCurve;
class NurbsSurface;
class PointCloud;
class Polyline;

/// Oriented bounding box as center, three axes and half-size.
class OBB {
private:
    static constexpr int NUM_SAMPLES = 20; // Samples per span when searching curve extrema.
    static constexpr int MAX_ITER = 20; // Newton iterations per extremum.
    mutable std::string _guid; // Lazy guid.

public:
    Point center; // Box center.
    Vector x_axis; // Unit x axis.
    Vector y_axis; // Unit y axis.
    Vector z_axis; // Unit z axis.
    Vector half_size; // Half extent along each axis.
    std::string name = "my_obb"; // Box name.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct the unit box at the origin.
    OBB();

    /// Construct from center, axes and half-size.
    OBB(const Point& center, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis, const Vector& half_size);

    /// Construct on the plane frame with full sizes dx, dy, dz.
    OBB(const Plane& plane, double dx, double dy, double dz);

    /// Copy with a new guid and the same data.
    OBB(const OBB& other);

    /// Copy-assign with a new guid and the same data.
    OBB& operator=(const OBB& other);

    /// Move while preserving the guid.
    OBB(OBB&& other) noexcept = default;

    /// Move-assign while preserving the guid.
    OBB& operator=(OBB&& other) noexcept = default;

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    bool has_guid() const {
        return !_guid.empty();
    }

    /// Return the guid, creating it on first access.
    const std::string& guid() const;

    /// Return the mutable guid, creating it on first access.
    std::string& guid();

    /// Clear the guid so a fresh one mints lazily on the next read.
    void refresh_guid();

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct on the plane frame with full sizes dx, dy, dz.
    static OBB from_plane(const Plane& plane, double dx, double dy, double dz);

    /// Construct the world-aligned box with the center and half-size of aabb.
    static OBB from_aabb(const AABB& aabb);

    /// Construct the world-aligned box of half-size inflate around point.
    static OBB from_point(const Point& point, double inflate = 0.0);

    /// Construct the world-aligned tight box of points grown by inflate.
    static OBB from_points(const std::vector<Point>& points, double inflate = 0.0);

    /// Construct the tight box of points in the plane frame grown by inflate.
    static OBB from_points(const std::vector<Point>& points, const Plane& plane, double inflate = 0.0);

    /// Construct the world-aligned tight box of the two ends grown by inflate.
    static OBB from_line(const Line& line, double inflate = 0.0);

    /// Construct the tight box of the two ends in the plane frame grown by inflate.
    static OBB from_line(const Line& line, const Plane& plane, double inflate = 0.0);

    /// Construct the world-aligned tight box of the vertices grown by inflate.
    static OBB from_polyline(const Polyline& polyline, double inflate = 0.0);

    /// Construct the tight box of the vertices in the plane frame grown by inflate.
    static OBB from_polyline(const Polyline& polyline, const Plane& plane, double inflate = 0.0);

    /// Construct the world-aligned tight box of the vertices grown by inflate.
    static OBB from_mesh(const Mesh& mesh, double inflate = 0.0);

    /// Construct the tight box of the vertices in the plane frame grown by inflate.
    static OBB from_mesh(const Mesh& mesh, const Plane& plane, double inflate = 0.0);

    /// Construct the world-aligned tight box of the points grown by inflate.
    static OBB from_pointcloud(const PointCloud& pointcloud, double inflate = 0.0);

    /// Construct the tight box of the points in the plane frame grown by inflate.
    static OBB from_pointcloud(const PointCloud& pointcloud, const Plane& plane, double inflate = 0.0);

    /// Construct the world-aligned box of the control points, or of the curve extrema when tight.
    static OBB from_nurbscurve(const NurbsCurve& curve, double inflate = 0.0, bool tight = false);

    /// Construct the box in the plane frame of the control points, or of the curve extrema when tight.
    static OBB from_nurbscurve(const NurbsCurve& curve, const Plane& plane, double inflate = 0.0, bool tight = false);

    /// Construct the world-aligned box of the control points grown by inflate.
    static OBB from_nurbssurface(const NurbsSurface& surface, double inflate = 0.0);

    /// Construct the box of the control points in the plane frame grown by inflate.
    static OBB from_nurbssurface(const NurbsSurface& surface, const Plane& plane, double inflate = 0.0);

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Compare name, center, axes and half-size to 1e-6; guid ignored.
    bool operator==(const OBB& other) const;

    /// Compare name, center, axes and half-size to 1e-6; guid ignored.
    bool operator!=(const OBB& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform center and axes in place.
    void transform(const Xform& xform);

    /// Return a transformed copy.
    OBB transformed(const Xform& xform) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the world-aligned box enclosing the corners.
    AABB aabb() const;

    /// Return the min corner of the world-aligned box.
    Point min_point() const;

    /// Return the max corner of the world-aligned box.
    Point max_point() const;

    /// Return the surface area.
    double area() const;

    /// Return the length of the space diagonal.
    double diagonal() const;

    /// Return the volume.
    double volume() const;

    /// Return whether no half-size is negative.
    bool is_valid() const;

    /// Return pt clamped to the box in its own frame.
    Point closest_point(const Point& pt) const;

    /// Return whether pt lies inside or on the box.
    bool contains(const Point& pt) const;

    /// Return the corner picked by the sign of each half-size.
    Point corner(bool x_max, bool y_max, bool z_max) const;

    /// Return the bottom loop then the top loop, counter-clockwise from +x+y.
    std::array<Point, 8> corners() const;

    /// Return the bottom loop then the top loop, counter-clockwise from +x+y.
    std::array<Point, 8> get_corners() const;

    /// Return the bottom loop, the top loop, then the four verticals.
    std::vector<Line> get_edges() const;

    /// Return the bottom loop and the top loop, each closed by repeating its first corner.
    std::array<Point, 10> two_rectangles() const;

    /// Return the center offset by x, y, z along the axes.
    Point point_at(double x, double y, double z) const;

    /// Grow every half-size by amount.
    void inflate(double amount);

    /// Grow in place to enclose the corners of other.
    void union_with(const OBB& other);

    // ═══════════════════════════════════════════════════════════════════════════
    // Collision
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the boxes overlap by the separating axis test (collides_with_rtcd).
    bool collides_with(const OBB& other) const;

    /// Return whether the boxes overlap, rejecting by AABB before collides_with.
    bool collides_with_broad(const OBB& other) const;

    /// Return whether the boxes overlap by the fifteen-axis test in the frame of this box (Real-Time Collision Detection).
    bool collides_with_rtcd(const OBB& other) const;

    /// Return whether the boxes overlap by the fifteen-axis test on projected extents.
    bool collides_with_naive(const OBB& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static OBB jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static OBB file_json_loads(const std::string& json_string);

    /// Write to a JSON file.
    void file_json_dump(const std::string& filename) const;

    /// Read from a JSON file.
    static OBB file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::BoundingBox to_proto() const;

    /// Construct from the protobuf message.
    static OBB from_proto(const session_proto::BoundingBox& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static OBB pb_loads(const std::string& data);

    /// Write to a protobuf file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file.
    static OBB pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "center\nx_axis\ny_axis\nz_axis\nhalf_size".
    std::string str() const;

    /// Return "OBB(name, center, x_axis, y_axis, z_axis, half_size)".
    std::string repr() const;

private:
    /// Compute the parameter in [t_lo, t_hi] where the derivative along axis crosses zero, by Newton steps bracketed by bisection.
    static double compute_extremum(const NurbsCurve& curve, const Vector& axis, double t_lo, double t_hi, double d_start);

    /// Return whether the extents of both boxes projected on axis do not reach their center distance.
    static bool separating_plane_exists(const Vector& relative_position, const Vector& axis, const OBB& box1, const OBB& box2);
};

/// Write the str() form to a stream.
std::ostream& operator<<(std::ostream& os, const OBB& obb);

} // namespace session_cpp

template <> struct fmt::formatter<session_cpp::OBB> {
    constexpr fmt::format_parse_context::iterator parse(fmt::format_parse_context& ctx) {
        return ctx.begin();
    }

    fmt::format_context::iterator format(const session_cpp::OBB& obb, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", obb.str());
    }
};
