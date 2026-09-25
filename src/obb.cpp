#include "obb.h"
#include "mesh.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "pointcloud.h"
#include "polyline.h"
#include "boundingbox.pb.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
OBB::OBB()
    : center(0.0, 0.0, 0.0),
      x_axis(1.0, 0.0, 0.0),
      y_axis(0.0, 1.0, 0.0),
      z_axis(0.0, 0.0, 1.0),
      half_size(0.5, 0.5, 0.5) {}

OBB::OBB(const Point& center, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis, const Vector& half_size)
    : center(center), x_axis(x_axis), y_axis(y_axis), z_axis(z_axis), half_size(half_size) {}

OBB::OBB(const Plane& plane, double dx, double dy, double dz)
    : center(plane.origin()),
      x_axis(plane.x_axis()),
      y_axis(plane.y_axis()),
      z_axis(plane.z_axis()),
      half_size(dx * 0.5, dy * 0.5, dz * 0.5) {}

OBB::OBB(const OBB& other)
    : center(other.center),
      x_axis(other.x_axis),
      y_axis(other.y_axis),
      z_axis(other.z_axis),
      half_size(other.half_size),
      name(other.name) {}

OBB& OBB::operator=(const OBB& other) {

    if (this == &other)
        return *this;

    _guid.clear();
    center = other.center;
    x_axis = other.x_axis;
    y_axis = other.y_axis;
    z_axis = other.z_axis;
    half_size = other.half_size;
    name = other.name;

    return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════════════════════
const std::string& OBB::guid() const {

    if (_guid.empty())
        _guid = ::guid();

    return _guid;
}

std::string& OBB::guid() {

    if (_guid.empty())
        _guid = ::guid();

    return _guid;
}

void OBB::refresh_guid() {
    _guid.clear();
}

// ═══════════════════════════════════════════════════════════════════════════
// Static constructors
// ═══════════════════════════════════════════════════════════════════════════
OBB OBB::from_plane(const Plane& plane, double dx, double dy, double dz) {
    return OBB(plane, dx, dy, dz);
}

OBB OBB::from_aabb(const AABB& aabb) {

    return OBB(
        Point(aabb.cx, aabb.cy, aabb.cz),
        Vector(1.0, 0.0, 0.0),
        Vector(0.0, 1.0, 0.0),
        Vector(0.0, 0.0, 1.0),
        Vector(aabb.hx, aabb.hy, aabb.hz)
    );
}

OBB OBB::from_point(const Point& point, double inflate) {
    return from_aabb(AABB::from_point(point, inflate));
}

OBB OBB::from_points(const std::vector<Point>& points, double inflate) {
    return from_aabb(AABB::from_points(points, inflate));
}

OBB OBB::from_points(const std::vector<Point>& points, const Plane& plane, double inflate) {

    if (points.empty())
        return OBB();

    const Point origin = plane.origin();
    const Vector x_axis = plane.x_axis();
    const Vector y_axis = plane.y_axis();
    const Vector z_axis = plane.z_axis();
    const Xform world_to_local = Xform::world_to_frame(origin, x_axis, y_axis, z_axis);
    const Xform local_to_world = Xform::frame_to_world(origin, x_axis, y_axis, z_axis);

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double min_z = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    double max_z = std::numeric_limits<double>::lowest();

    for (const Point& pt : points) {
        const Point local = pt.transformed(world_to_local);

        min_x = std::min(min_x, local[0]);
        min_y = std::min(min_y, local[1]);
        min_z = std::min(min_z, local[2]);
        max_x = std::max(max_x, local[0]);
        max_y = std::max(max_y, local[1]);
        max_z = std::max(max_z, local[2]);
    }

    const Point local_center((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5);
    const Vector half_size(
        (max_x - min_x) * 0.5 + inflate,
        (max_y - min_y) * 0.5 + inflate,
        (max_z - min_z) * 0.5 + inflate
    );

    return OBB(local_center.transformed(local_to_world), x_axis, y_axis, z_axis, half_size);
}

OBB OBB::from_line(const Line& line, double inflate) {
    return from_aabb(AABB::from_line(line, inflate));
}

OBB OBB::from_line(const Line& line, const Plane& plane, double inflate) {
    return from_points({line.start(), line.end()}, plane, inflate);
}

OBB OBB::from_polyline(const Polyline& polyline, double inflate) {
    return from_aabb(AABB::from_polyline(polyline, inflate));
}

OBB OBB::from_polyline(const Polyline& polyline, const Plane& plane, double inflate) {
    return from_points(polyline.get_points(), plane, inflate);
}

OBB OBB::from_mesh(const Mesh& mesh, double inflate) {
    return from_aabb(AABB::from_mesh(mesh, inflate));
}

OBB OBB::from_mesh(const Mesh& mesh, const Plane& plane, double inflate) {
    return from_points(mesh.to_vertices_and_faces().first, plane, inflate);
}

OBB OBB::from_pointcloud(const PointCloud& pointcloud, double inflate) {
    return from_aabb(AABB::from_pointcloud(pointcloud, inflate));
}

OBB OBB::from_pointcloud(const PointCloud& pointcloud, const Plane& plane, double inflate) {
    return from_points(pointcloud.get_points(), plane, inflate);
}

OBB OBB::from_nurbscurve(const NurbsCurve& curve, double inflate, bool tight) {
    return from_aabb(AABB::from_nurbscurve(curve, inflate, tight));
}

OBB OBB::from_nurbscurve(const NurbsCurve& curve, const Plane& plane, double inflate, bool tight) {

    if (!curve.is_valid() || curve.cv_count() == 0)
        return OBB();

    std::vector<Point> points;

    if (!tight) {
        for (int i = 0; i < curve.cv_count(); i++)
            points.push_back(curve.get_cv(i));

        return from_points(points, plane, inflate);
    }

    const double t0 = curve.domain_start();
    const double t1 = curve.domain_end();

    points.push_back(curve.point_at(t0));
    points.push_back(curve.point_at(t1));

    for (const double t : curve.get_span_vector())
        if (t > t0 && t < t1)
            points.push_back(curve.point_at(t));

    const Vector axes[3] = {plane.x_axis(), plane.y_axis(), plane.z_axis()};
    const double dt = (t1 - t0) / NUM_SAMPLES;

    for (const Vector& axis : axes) {
        for (int i = 0; i < NUM_SAMPLES; i++) {
            const double t_start = t0 + i * dt;
            const double t_end = t_start + dt;
            const std::vector<Vector> deriv_start = curve.evaluate(t_start, 1);
            const std::vector<Vector> deriv_end = curve.evaluate(t_end, 1);

            if (deriv_start.size() < 2 || deriv_end.size() < 2)
                continue;

            const double d_start = deriv_start[1].dot(axis);
            const double d_end = deriv_end[1].dot(axis);

            if (d_start * d_end < 0) {
                const double t_root = compute_extremum(curve, axis, t_start, t_end, d_start);
                points.push_back(curve.point_at(t_root));
            }
        }
    }

    return from_points(points, plane, inflate);
}

OBB OBB::from_nurbssurface(const NurbsSurface& surface, double inflate) {
    return from_aabb(AABB::from_nurbssurface(surface, inflate));
}

OBB OBB::from_nurbssurface(const NurbsSurface& surface, const Plane& plane, double inflate) {

    if (!surface.is_valid() || surface.cv_count(0) == 0 || surface.cv_count(1) == 0)
        return OBB();

    std::vector<Point> points;

    for (int i = 0; i < surface.cv_count(0); i++)
        for (int j = 0; j < surface.cv_count(1); j++)
            points.push_back(surface.get_cv(i, j));

    return from_points(points, plane, inflate);
}

double OBB::compute_extremum(const NurbsCurve& curve, const Vector& axis, double t_lo, double t_hi, double d_start) {

    double t_root = (t_lo + t_hi) * 0.5;

    for (int it = 0; it < MAX_ITER; it++) {
        const std::vector<Vector> deriv = curve.evaluate(t_root, 2);

        if (deriv.size() < 3)
            break;

        const double d1 = deriv[1].dot(axis);
        const double d2 = deriv[2].dot(axis);

        if (std::abs(d1) < 1e-12)
            break;

        if (std::abs(d2) > 1e-14) {
            const double t_new = t_root - d1 / d2;

            if (t_new >= t_lo && t_new <= t_hi) {
                t_root = t_new;
            } else {
                if (d1 * d_start < 0)
                    t_hi = t_root;
                else
                    t_lo = t_root;

                t_root = (t_lo + t_hi) * 0.5;
            }
        } else {
            t_root = (t_lo + t_hi) * 0.5;
        }

        const std::vector<Vector> deriv_check = curve.evaluate(t_root, 1);

        if (deriv_check.size() < 2)
            continue;

        const double d_check = deriv_check[1].dot(axis);

        if (d_check * d_start < 0) {
            t_hi = t_root;
        } else {
            t_lo = t_root;
            d_start = d_check;
        }
    }

    return t_root;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
bool OBB::operator==(const OBB& other) const {

    if (name != other.name)
        return false;

    for (int i = 0; i < 3; i++) {
        if (std::round(center[i] * 1000000.0) != std::round(other.center[i] * 1000000.0))
            return false;

        if (std::round(x_axis[i] * 1000000.0) != std::round(other.x_axis[i] * 1000000.0))
            return false;

        if (std::round(y_axis[i] * 1000000.0) != std::round(other.y_axis[i] * 1000000.0))
            return false;

        if (std::round(z_axis[i] * 1000000.0) != std::round(other.z_axis[i] * 1000000.0))
            return false;

        if (std::round(half_size[i] * 1000000.0) != std::round(other.half_size[i] * 1000000.0))
            return false;
    }

    return true;
}

bool OBB::operator!=(const OBB& other) const {
    return !(*this == other);
}

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════
void OBB::transform(const Xform& xform) {

    center.transform(xform);
    x_axis.transform(xform);
    y_axis.transform(xform);
    z_axis.transform(xform);
}

OBB OBB::transformed(const Xform& xform) const {

    OBB result = *this;
    result.transform(xform);

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometry
// ═══════════════════════════════════════════════════════════════════════════
AABB OBB::aabb() const {

    const double ex = half_size[0];
    const double ey = half_size[1];
    const double ez = half_size[2];
    const double hx = std::abs(x_axis[0]) * ex + std::abs(y_axis[0]) * ey + std::abs(z_axis[0]) * ez;
    const double hy = std::abs(x_axis[1]) * ex + std::abs(y_axis[1]) * ey + std::abs(z_axis[1]) * ez;
    const double hz = std::abs(x_axis[2]) * ex + std::abs(y_axis[2]) * ey + std::abs(z_axis[2]) * ez;

    return AABB(center[0], center[1], center[2], hx, hy, hz);
}

Point OBB::min_point() const {
    return aabb().min_point();
}

Point OBB::max_point() const {
    return aabb().max_point();
}

double OBB::area() const {

    const double hx = half_size[0];
    const double hy = half_size[1];
    const double hz = half_size[2];

    return 8.0 * (hx * hy + hy * hz + hz * hx);
}

double OBB::diagonal() const {

    const double hx = half_size[0];
    const double hy = half_size[1];
    const double hz = half_size[2];

    return 2.0 * std::sqrt(hx * hx + hy * hy + hz * hz);
}

double OBB::volume() const {
    return 8.0 * half_size[0] * half_size[1] * half_size[2];
}

bool OBB::is_valid() const {
    return half_size[0] >= 0.0 && half_size[1] >= 0.0 && half_size[2] >= 0.0;
}

Point OBB::closest_point(const Point& pt) const {

    const Vector offset = pt - center;
    const double lx = std::max(-half_size[0], std::min(half_size[0], offset.dot(x_axis)));
    const double ly = std::max(-half_size[1], std::min(half_size[1], offset.dot(y_axis)));
    const double lz = std::max(-half_size[2], std::min(half_size[2], offset.dot(z_axis)));

    return point_at(lx, ly, lz);
}

bool OBB::contains(const Point& pt) const {

    const Vector offset = pt - center;
    const double lx = std::abs(offset.dot(x_axis));
    const double ly = std::abs(offset.dot(y_axis));
    const double lz = std::abs(offset.dot(z_axis));

    return lx <= half_size[0] && ly <= half_size[1] && lz <= half_size[2];
}

Point OBB::corner(bool x_max, bool y_max, bool z_max) const {

    const double ox = x_max ? half_size[0] : -half_size[0];
    const double oy = y_max ? half_size[1] : -half_size[1];
    const double oz = z_max ? half_size[2] : -half_size[2];

    return point_at(ox, oy, oz);
}

std::array<Point, 8> OBB::corners() const {

    return {
        point_at(half_size[0], half_size[1], -half_size[2]),
        point_at(-half_size[0], half_size[1], -half_size[2]),
        point_at(-half_size[0], -half_size[1], -half_size[2]),
        point_at(half_size[0], -half_size[1], -half_size[2]),
        point_at(half_size[0], half_size[1], half_size[2]),
        point_at(-half_size[0], half_size[1], half_size[2]),
        point_at(-half_size[0], -half_size[1], half_size[2]),
        point_at(half_size[0], -half_size[1], half_size[2]),
    };
}

std::array<Point, 8> OBB::get_corners() const {
    return corners();
}

std::vector<Line> OBB::get_edges() const {

    const std::array<Point, 8> points = corners();

    return {
        Line::from_points(points[0], points[1]),
        Line::from_points(points[1], points[2]),
        Line::from_points(points[2], points[3]),
        Line::from_points(points[3], points[0]),
        Line::from_points(points[4], points[5]),
        Line::from_points(points[5], points[6]),
        Line::from_points(points[6], points[7]),
        Line::from_points(points[7], points[4]),
        Line::from_points(points[0], points[4]),
        Line::from_points(points[1], points[5]),
        Line::from_points(points[2], points[6]),
        Line::from_points(points[3], points[7]),
    };
}

std::array<Point, 10> OBB::two_rectangles() const {

    return {
        point_at(half_size[0], half_size[1], -half_size[2]),
        point_at(-half_size[0], half_size[1], -half_size[2]),
        point_at(-half_size[0], -half_size[1], -half_size[2]),
        point_at(half_size[0], -half_size[1], -half_size[2]),
        point_at(half_size[0], half_size[1], -half_size[2]),
        point_at(half_size[0], half_size[1], half_size[2]),
        point_at(-half_size[0], half_size[1], half_size[2]),
        point_at(-half_size[0], -half_size[1], half_size[2]),
        point_at(half_size[0], -half_size[1], half_size[2]),
        point_at(half_size[0], half_size[1], half_size[2]),
    };
}

Point OBB::point_at(double x, double y, double z) const {
    return center + x_axis * x + y_axis * y + z_axis * z;
}

void OBB::inflate(double amount) {
    half_size += Vector(amount, amount, amount);
}

void OBB::union_with(const OBB& other) {

    double min_x = -half_size[0];
    double min_y = -half_size[1];
    double min_z = -half_size[2];
    double max_x = half_size[0];
    double max_y = half_size[1];
    double max_z = half_size[2];

    for (const Point& point : other.corners()) {
        const Vector offset = point - center;
        const double lx = offset.dot(x_axis);
        const double ly = offset.dot(y_axis);
        const double lz = offset.dot(z_axis);

        min_x = std::min(min_x, lx);
        min_y = std::min(min_y, ly);
        min_z = std::min(min_z, lz);
        max_x = std::max(max_x, lx);
        max_y = std::max(max_y, ly);
        max_z = std::max(max_z, lz);
    }

    center = point_at((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5);
    half_size = Vector((max_x - min_x) * 0.5, (max_y - min_y) * 0.5, (max_z - min_z) * 0.5);
}

// ═══════════════════════════════════════════════════════════════════════════
// Collision
// ═══════════════════════════════════════════════════════════════════════════
bool OBB::collides_with(const OBB& other) const {
    return collides_with_rtcd(other);
}

bool OBB::collides_with_broad(const OBB& other) const {

    if (!aabb().intersects(other.aabb()))
        return false;

    return collides_with(other);
}

bool OBB::collides_with_rtcd(const OBB& other) const {

    const double eps = 1e-9;
    const double a0 = half_size[0];
    const double a1 = half_size[1];
    const double a2 = half_size[2];
    const double b0 = other.half_size[0];
    const double b1 = other.half_size[1];
    const double b2 = other.half_size[2];
    const double r00 = x_axis.dot(other.x_axis);
    const double r01 = x_axis.dot(other.y_axis);
    const double r02 = x_axis.dot(other.z_axis);
    const double r10 = y_axis.dot(other.x_axis);
    const double r11 = y_axis.dot(other.y_axis);
    const double r12 = y_axis.dot(other.z_axis);
    const double r20 = z_axis.dot(other.x_axis);
    const double r21 = z_axis.dot(other.y_axis);
    const double r22 = z_axis.dot(other.z_axis);
    const Vector offset = other.center - center;
    const double t0 = offset.dot(x_axis);
    const double t1 = offset.dot(y_axis);
    const double t2 = offset.dot(z_axis);
    const double ar00 = std::abs(r00) + eps;
    const double ar01 = std::abs(r01) + eps;
    const double ar02 = std::abs(r02) + eps;
    const double ar10 = std::abs(r10) + eps;
    const double ar11 = std::abs(r11) + eps;
    const double ar12 = std::abs(r12) + eps;
    const double ar20 = std::abs(r20) + eps;
    const double ar21 = std::abs(r21) + eps;
    const double ar22 = std::abs(r22) + eps;

    if (std::abs(t0) > a0 + b0 * ar00 + b1 * ar01 + b2 * ar02)
        return false;

    if (std::abs(t1) > a1 + b0 * ar10 + b1 * ar11 + b2 * ar12)
        return false;

    if (std::abs(t2) > a2 + b0 * ar20 + b1 * ar21 + b2 * ar22)
        return false;

    if (std::abs(t0 * r00 + t1 * r10 + t2 * r20) > a0 * ar00 + a1 * ar10 + a2 * ar20 + b0)
        return false;

    if (std::abs(t0 * r01 + t1 * r11 + t2 * r21) > a0 * ar01 + a1 * ar11 + a2 * ar21 + b1)
        return false;

    if (std::abs(t0 * r02 + t1 * r12 + t2 * r22) > a0 * ar02 + a1 * ar12 + a2 * ar22 + b2)
        return false;

    if (std::abs(t2 * r10 - t1 * r20) > a1 * ar20 + a2 * ar10 + b1 * ar02 + b2 * ar01)
        return false;

    if (std::abs(t2 * r11 - t1 * r21) > a1 * ar21 + a2 * ar11 + b0 * ar02 + b2 * ar00)
        return false;

    if (std::abs(t2 * r12 - t1 * r22) > a1 * ar22 + a2 * ar12 + b0 * ar01 + b1 * ar00)
        return false;

    if (std::abs(t0 * r20 - t2 * r00) > a0 * ar20 + a2 * ar00 + b1 * ar12 + b2 * ar11)
        return false;

    if (std::abs(t0 * r21 - t2 * r01) > a0 * ar21 + a2 * ar01 + b0 * ar12 + b2 * ar10)
        return false;

    if (std::abs(t0 * r22 - t2 * r02) > a0 * ar22 + a2 * ar02 + b0 * ar11 + b1 * ar10)
        return false;

    if (std::abs(t1 * r00 - t0 * r10) > a0 * ar10 + a1 * ar00 + b1 * ar22 + b2 * ar21)
        return false;

    if (std::abs(t1 * r01 - t0 * r11) > a0 * ar11 + a1 * ar01 + b0 * ar22 + b2 * ar20)
        return false;

    if (std::abs(t1 * r02 - t0 * r12) > a0 * ar12 + a1 * ar02 + b0 * ar21 + b1 * ar20)
        return false;

    return true;
}

bool OBB::collides_with_naive(const OBB& other) const {

    const Vector offset = other.center - center;
    const Vector axes[15] = {
        x_axis,
        y_axis,
        z_axis,
        other.x_axis,
        other.y_axis,
        other.z_axis,
        x_axis.cross(other.x_axis),
        x_axis.cross(other.y_axis),
        x_axis.cross(other.z_axis),
        y_axis.cross(other.x_axis),
        y_axis.cross(other.y_axis),
        y_axis.cross(other.z_axis),
        z_axis.cross(other.x_axis),
        z_axis.cross(other.y_axis),
        z_axis.cross(other.z_axis),
    };

    for (const Vector& axis : axes)
        if (separating_plane_exists(offset, axis, *this, other))
            return false;

    return true;
}

bool OBB::separating_plane_exists(const Vector& relative_position, const Vector& axis, const OBB& box1, const OBB& box2) {

    const double proj1 = std::abs((box1.x_axis * box1.half_size[0]).dot(axis))
        + std::abs((box1.y_axis * box1.half_size[1]).dot(axis))
        + std::abs((box1.z_axis * box1.half_size[2]).dot(axis));

    const double proj2 = std::abs((box2.x_axis * box2.half_size[0]).dot(axis))
        + std::abs((box2.y_axis * box2.half_size[1]).dot(axis))
        + std::abs((box2.z_axis * box2.half_size[2]).dot(axis));

    return std::abs(relative_position.dot(axis)) > proj1 + proj2;
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json OBB::jsondump() const {

    nlohmann::ordered_json data;
    data["center"] = center.jsondump();
    data["guid"] = guid();
    data["half_size"] = half_size.jsondump();
    data["name"] = name;
    data["type"] = "OBB";
    data["x_axis"] = x_axis.jsondump();
    data["y_axis"] = y_axis.jsondump();
    data["z_axis"] = z_axis.jsondump();

    return data;
}

OBB OBB::jsonload(const nlohmann::json& data) {

    OBB obb(
        Point::jsonload(data.at("center")),
        Vector::jsonload(data.at("x_axis")),
        Vector::jsonload(data.at("y_axis")),
        Vector::jsonload(data.at("z_axis")),
        Vector::jsonload(data.at("half_size"))
    );

    obb.guid() = data.at("guid");
    obb.name = data.at("name");

    return obb;
}

std::string OBB::file_json_dumps() const {
    return jsondump().dump();
}

OBB OBB::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void OBB::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);
    file << jsondump().dump(2);
}

OBB OBB::file_json_load(const std::string& filename) {

    std::ifstream file(filename);

    return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════
session_proto::BoundingBox OBB::to_proto() const {

    session_proto::BoundingBox proto;
    *proto.mutable_center() = center.to_proto();
    *proto.mutable_x_axis() = x_axis.to_proto();
    *proto.mutable_y_axis() = y_axis.to_proto();
    *proto.mutable_z_axis() = z_axis.to_proto();
    *proto.mutable_half_size() = half_size.to_proto();

    if (has_guid())
        proto.set_guid(guid());

    proto.set_name(name);

    return proto;
}

OBB OBB::from_proto(const session_proto::BoundingBox& proto) {

    OBB obb(
        Point::from_proto(proto.center()),
        Vector::from_proto(proto.x_axis()),
        Vector::from_proto(proto.y_axis()),
        Vector::from_proto(proto.z_axis()),
        Vector::from_proto(proto.half_size())
    );

    if (!proto.guid().empty())
        obb.guid() = proto.guid();

    obb.name = proto.name();

    return obb;
}

std::string OBB::pb_dumps() const {
    return to_proto().SerializeAsString();
}

OBB OBB::pb_loads(const std::string& data) {

    session_proto::BoundingBox proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse OBB protobuf data");

    return from_proto(proto);
}

void OBB::pb_dump(const std::string& filename) const {

    std::ofstream file(filename, std::ios::binary);
    file << pb_dumps();
}

OBB OBB::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string OBB::str() const {
    return fmt::format("{}\n{}\n{}\n{}\n{}", center.str(), x_axis.str(), y_axis.str(), z_axis.str(), half_size.str());
}

std::string OBB::repr() const {

    return fmt::format(
        "OBB({}, {}, {}, {}, {}, {})",
        name,
        center.str(),
        x_axis.str(),
        y_axis.str(),
        z_axis.str(),
        half_size.str()
    );
}

std::ostream& operator<<(std::ostream& os, const OBB& obb) {
    return os << obb.str();
}

} // namespace session_cpp
