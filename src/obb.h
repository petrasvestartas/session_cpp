#pragma once

#include "aabb.h"
#include "point.h"
#include "vector.h"
#include "plane.h"
#include "guid.h"
#include "json.h"
#include <string>
#include <array>
#include <vector>
#include <limits>

namespace session_cpp {


class Line;
class Polyline;
class Mesh;
class PointCloud;
class NurbsCurve;
class NurbsSurface;

/**
 * @brief An oriented bounding box with collision detection capabilities
 *
 * Represents an oriented bounding box (OBB) defined by a center point,
 * three orthogonal axes, and half-size extents along each axis.
 */
class OBB {
public:
    Point center;
    Vector x_axis;
    Vector y_axis;
    Vector z_axis;
    Vector half_size;
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string name;

    Xform xform;

    OBB();
    OBB(const Point& center, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis, const Vector& half_size);
    OBB(const Plane&plane, double dx, double dy, double dz);

    static OBB from_point(const Point& point, double inflate = 0.0);
    static OBB from_points(const std::vector<Point>& points, double inflate = 0.0);
    static OBB from_points(const std::vector<Point>& points, const Plane& plane, double inflate = 0.0);
    static OBB from_line(const Line& line, double inflate = 0.0);
    static OBB from_line(const Line& line, const Plane& plane, double inflate = 0.0);
    static OBB from_polyline(const Polyline& polyline, double inflate = 0.0);
    static OBB from_polyline(const Polyline& polyline, const Plane& plane, double inflate = 0.0);
    static OBB from_mesh(const Mesh& mesh, double inflate = 0.0);
    static OBB from_mesh(const Mesh& mesh, const Plane& plane, double inflate = 0.0);
    static OBB from_pointcloud(const PointCloud& pointcloud, double inflate = 0.0);
    static OBB from_pointcloud(const PointCloud& pointcloud, const Plane& plane, double inflate = 0.0);
    static OBB from_nurbscurve(const NurbsCurve& curve, double inflate = 0.0, bool tight = false);
    static OBB from_nurbscurve(const NurbsCurve& curve, const Plane& plane, double inflate = 0.0, bool tight = false);
    static OBB from_nurbssurface(const NurbsSurface& surface, double inflate = 0.0);
    static OBB from_nurbssurface(const NurbsSurface& surface, const Plane& plane, double inflate = 0.0);

    AABB aabb() const;
    Point min_point() const;
    Point max_point() const;
    double area() const;
    double diagonal() const;
    bool is_valid() const;
    double volume() const;
    Point closest_point(const Point& pt) const;
    bool contains(const Point& pt) const;
    Point corner(bool x_max, bool y_max, bool z_max) const;
    std::array<Point, 8> corners() const;
    std::array<Point, 8> get_corners() const;
    std::vector<Line> get_edges() const;
    std::array<Point, 10> two_rectangles() const;
    Point point_at(double x, double y, double z) const;
    void inflate(double amount);
    void union_with(const OBB& other);

    bool collides_with(const OBB& other) const;
    bool collides_with_broad(const OBB& other) const;
    bool collides_with_rtcd(const OBB& other) const;
    bool collides_with_naive(const OBB& other) const;

    void transform();
    OBB transformed() const;

    nlohmann::ordered_json jsondump() const;
    static OBB jsonload(const nlohmann::json& data);

    void to_json_file(const std::string& filepath) const;
    static OBB from_json_file(const std::string& filepath);

    std::string json_dumps() const;
    static OBB json_loads(const std::string& json_string);
    void json_dump(const std::string& filename) const;
    static OBB json_load(const std::string& filename);
    std::string pb_dumps() const;
    static OBB pb_loads(const std::string& data);
    void pb_dump(const std::string& filename) const;
    static OBB pb_load(const std::string& filename);

private:
    mutable std::string _guid;
    static bool separating_plane_exists(const Vector& relative_position, const Vector& axis, const OBB& box1, const OBB& box2);
};

}

// ── session_cpp OBB (Oriented Bounding Box) collision utilities ───────────────
// Replaces cgal_box_util.h/.cpp using session_cpp types.
// Each OBB is represented as Vector[5]:
//   [0] = center (as Vector), [1..3] = local axes, [4] = half-extents per axis.

#include "xform.h"
#include <cmath>

namespace session_cpp {
namespace obb {

namespace internal {

inline bool get_separating_plane(
    const Vector& rp, const Vector& axis,
    const Vector (&box1)[5], const Vector (&box2)[5])
{
    return fabs(rp.dot(axis)) >
        (  fabs((box1[1] * box1[4][0]).dot(axis))
         + fabs((box1[2] * box1[4][1]).dot(axis))
         + fabs((box1[3] * box1[4][2]).dot(axis))
         + fabs((box2[1] * box2[4][0]).dot(axis))
         + fabs((box2[2] * box2[4][1]).dot(axis))
         + fabs((box2[3] * box2[4][2]).dot(axis)));
}

} // namespace internal

/// Transform the center component (index 0) as a point and
/// axes (indices 1–3) as vectors using the standard xform pattern.
inline void transform_plane_as_vector_array(Vector* plane, const Xform& xform)
{
    Point p = {plane[0][0], plane[0][1], plane[0][2]};
    p.xform = xform; p.transform();
    plane[0] = {p[0], p[1], p[2]};
    for (int i = 1; i < 4; i++) {
        plane[i].xform = xform; plane[i].transform();
    }
}

/// Copy n Vector elements from source to target.
inline void assign(const Vector* source, Vector* target, int n)
{
    for (int i = 0; i < n; i++)
        target[i] = source[i];
}

/// SAT OBB–OBB collision test; returns true when the two boxes overlap.
inline bool get_collision(const Vector (&box1)[5], const Vector (&box2)[5])
{
    Vector rp = box2[0] - box1[0];
    return !(
        internal::get_separating_plane(rp, box1[1], box1, box2) ||
        internal::get_separating_plane(rp, box1[2], box1, box2) ||
        internal::get_separating_plane(rp, box1[3], box1, box2) ||
        internal::get_separating_plane(rp, box2[1], box1, box2) ||
        internal::get_separating_plane(rp, box2[2], box1, box2) ||
        internal::get_separating_plane(rp, box2[3], box1, box2) ||
        internal::get_separating_plane(rp, box1[1].cross(box2[1]), box1, box2) ||
        internal::get_separating_plane(rp, box1[1].cross(box2[2]), box1, box2) ||
        internal::get_separating_plane(rp, box1[1].cross(box2[3]), box1, box2) ||
        internal::get_separating_plane(rp, box1[2].cross(box2[1]), box1, box2) ||
        internal::get_separating_plane(rp, box1[2].cross(box2[2]), box1, box2) ||
        internal::get_separating_plane(rp, box1[2].cross(box2[3]), box1, box2) ||
        internal::get_separating_plane(rp, box1[3].cross(box2[1]), box1, box2) ||
        internal::get_separating_plane(rp, box1[3].cross(box2[2]), box1, box2) ||
        internal::get_separating_plane(rp, box1[3].cross(box2[3]), box1, box2));
}

} // namespace obb
} // namespace session_cpp
