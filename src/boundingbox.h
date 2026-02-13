#pragma once

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
class BoundingBox {
public:
    Point center;
    Vector x_axis;
    Vector y_axis;
    Vector z_axis;
    Vector half_size;
    std::string guid;
    std::string name;

    Xform xform;

    BoundingBox();
    BoundingBox(const Point& center, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis, const Vector& half_size);
    BoundingBox(const Plane&plane, double dx, double dy, double dz);
    
    static BoundingBox from_point(const Point& point, double inflate = 0.0);
    static BoundingBox from_points(const std::vector<Point>& points, double inflate = 0.0);
    static BoundingBox from_points(const std::vector<Point>& points, const Plane& plane, double inflate = 0.0);
    static BoundingBox from_line(const Line& line, double inflate = 0.0);
    static BoundingBox from_line(const Line& line, const Plane& plane, double inflate = 0.0);
    static BoundingBox from_polyline(const Polyline& polyline, double inflate = 0.0);
    static BoundingBox from_polyline(const Polyline& polyline, const Plane& plane, double inflate = 0.0);
    static BoundingBox from_mesh(const Mesh& mesh, double inflate = 0.0);
    static BoundingBox from_mesh(const Mesh& mesh, const Plane& plane, double inflate = 0.0);
    static BoundingBox from_pointcloud(const PointCloud& pointcloud, double inflate = 0.0);
    static BoundingBox from_pointcloud(const PointCloud& pointcloud, const Plane& plane, double inflate = 0.0);
    static BoundingBox from_nurbscurve(const NurbsCurve& curve, double inflate = 0.0, bool tight = false);
    static BoundingBox from_nurbscurve(const NurbsCurve& curve, const Plane& plane, double inflate = 0.0, bool tight = false);
    static BoundingBox from_nurbssurface(const NurbsSurface& surface, double inflate = 0.0);
    static BoundingBox from_nurbssurface(const NurbsSurface& surface, const Plane& plane, double inflate = 0.0);

    BoundingBox aabb() const;
    Point min_point() const;
    Point max_point() const;
    std::array<Point, 8> corners() const;
    std::array<Point, 10> two_rectangles() const;
    Point point_at(double x, double y, double z) const;
    void inflate(double amount);
    
    bool collides_with(const BoundingBox& other) const;
    bool collides_with_rtcd(const BoundingBox& other) const;
    bool collides_with_naive(const BoundingBox& other) const;
    
    void transform();
    BoundingBox transformed() const;
    
    nlohmann::ordered_json jsondump() const;
    static BoundingBox jsonload(const nlohmann::json& data);

    void to_json_file(const std::string& filepath) const;
    static BoundingBox from_json_file(const std::string& filepath);

    std::string json_dumps() const;
    static BoundingBox json_loads(const std::string& json_string);
    void json_dump(const std::string& filename) const;
    static BoundingBox json_load(const std::string& filename);
    std::string pb_dumps() const;
    static BoundingBox pb_loads(const std::string& data);
    void pb_dump(const std::string& filename) const;
    static BoundingBox pb_load(const std::string& filename);

private:
    static bool separating_plane_exists(const Vector& relative_position, const Vector& axis, const BoundingBox& box1, const BoundingBox& box2);
};

}
