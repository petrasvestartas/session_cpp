#include "obb.h"
#include "line.h"
#include "point.h"
#include "polyline.h"
#include "mesh.h"
#include "pointcloud.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "guid.h"
#include "boundingbox.pb.h"
#include "point.pb.h"
#include "vector.pb.h"
#include "xform.pb.h"
#include <fstream>
#include <cmath>
#include <algorithm>

namespace session_cpp {

OBB::OBB()
    : center(0.0, 0.0, 0.0),
      x_axis(1.0, 0.0, 0.0),
      y_axis(0.0, 1.0, 0.0),
      z_axis(0.0, 0.0, 1.0),
      half_size(0.5, 0.5, 0.5),
      name("my_obb") {}

OBB::OBB(const Point& center, const Vector& x_axis, const Vector& y_axis, const Vector& z_axis, const Vector& half_size)
    : center(center),
      x_axis(x_axis),
      y_axis(y_axis),
      z_axis(z_axis),
      half_size(half_size),
      name("my_obb") {}

OBB::OBB(const Plane& plane, double dx, double dy, double dz)
    : center(plane.origin()),
      x_axis(plane.x_axis()),
      y_axis(plane.y_axis()),
      z_axis(plane.z_axis()),
      half_size(dx * 0.5, dy * 0.5, dz * 0.5),
      name("") {}

static OBB obb_from_aabb(const AABB& a) {
    return OBB(Point(a.cx, a.cy, a.cz), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0), Vector(0.0, 0.0, 1.0), Vector(a.hx, a.hy, a.hz));
}

OBB OBB::from_point(const Point& point, double inflate_amount) {
    return obb_from_aabb(AABB::from_point(point, inflate_amount));
}

OBB OBB::from_points(const std::vector<Point>& points, double inflate_amount) {
    return obb_from_aabb(AABB::from_points(points, inflate_amount));
}

OBB OBB::from_points(const std::vector<Point>& points, const Plane& plane, double inflate_amount) {
    if (points.empty()) {
        return OBB();
    }

    Point origin = plane.origin();
    Vector x_axis = plane.x_axis();
    Vector y_axis = plane.y_axis();
    Vector z_axis = plane.z_axis();
    // Use the new world_to_frame / frame_to_world. The legacy plane_to_xy /
    // xy_to_plane pair stores basis vectors as matrix COLUMNS (a local→world
    // rotation in both directions), so the forward call silently permutes
    // coordinates for non-axis-aligned planes; the bounding box it computed
    // happened to be correct only because the Plane::xy_plane unit test
    // uses the identity basis. See wood_main.cpp type-13 branch for the
    // failure mode this would have surfaced for rotated input.
    Xform world_to_local = Xform::world_to_frame(origin, x_axis, y_axis, z_axis);
    Xform local_to_world = Xform::frame_to_world(origin, x_axis, y_axis, z_axis);

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double min_z = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    double max_z = std::numeric_limits<double>::lowest();

    for (const auto& pt : points) {
        Point local_pt = pt; local_pt.xform = world_to_local; local_pt = local_pt.transformed();
        min_x = std::min(min_x, local_pt[0]);
        min_y = std::min(min_y, local_pt[1]);
        min_z = std::min(min_z, local_pt[2]);
        max_x = std::max(max_x, local_pt[0]);
        max_y = std::max(max_y, local_pt[1]);
        max_z = std::max(max_z, local_pt[2]);
    }

    Point local_center((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5);
    Vector half_size(
        (max_x - min_x) * 0.5 + inflate_amount,
        (max_y - min_y) * 0.5 + inflate_amount,
        (max_z - min_z) * 0.5 + inflate_amount
    );

    Point world_center = local_center; world_center.xform = local_to_world; world_center = world_center.transformed();

    return OBB(world_center, x_axis, y_axis, z_axis, half_size);
}

OBB OBB::from_line(const Line& line, double inflate_amount) {
    return obb_from_aabb(AABB::from_line(line, inflate_amount));
}

OBB OBB::from_line(const Line& line, const Plane& plane, double inflate_amount) {
    std::vector<Point> points = {line.start(), line.end()};
    return from_points(points, plane, inflate_amount);
}

OBB OBB::from_polyline(const Polyline& polyline, double inflate_amount) {
    return obb_from_aabb(AABB::from_polyline(polyline, inflate_amount));
}

OBB OBB::from_polyline(const Polyline& polyline, const Plane& plane, double inflate_amount) {
    return from_points(polyline.get_points(), plane, inflate_amount);
}

OBB OBB::from_mesh(const Mesh& mesh, double inflate_amount) {
    return obb_from_aabb(AABB::from_mesh(mesh, inflate_amount));
}

OBB OBB::from_mesh(const Mesh& mesh, const Plane& plane, double inflate_amount) {
    auto [vertices, faces] = mesh.to_vertices_and_faces();
    return from_points(vertices, plane, inflate_amount);
}

OBB OBB::from_pointcloud(const PointCloud& pointcloud, double inflate_amount) {
    return obb_from_aabb(AABB::from_pointcloud(pointcloud, inflate_amount));
}

OBB OBB::from_pointcloud(const PointCloud& pointcloud, const Plane& plane, double inflate_amount) {
    return from_points(pointcloud.get_points(), plane, inflate_amount);
}

OBB OBB::from_nurbscurve(const NurbsCurve& curve, double inflate_amount, bool tight) {
    return obb_from_aabb(AABB::from_nurbscurve(curve, inflate_amount, tight));
}

OBB OBB::from_nurbscurve(const NurbsCurve& curve, const Plane& plane, double inflate_amount, bool tight) {
    if (!curve.is_valid() || curve.cv_count() == 0) {
        return OBB();
    }

    if (!tight) {
        std::vector<Point> points;
        for (int i = 0; i < curve.cv_count(); i++) {
            points.push_back(curve.get_cv(i));
        }
        return from_points(points, plane, inflate_amount);
    }

    // Tight bounding box in plane's coordinate system
    // Find extrema along plane axes
    auto [t0, t1] = curve.domain();

    std::vector<Point> extrema_points;
    extrema_points.push_back(curve.point_at(t0));
    extrema_points.push_back(curve.point_at(t1));

    auto spans = curve.get_span_vector();
    for (double t : spans) {
        if (t > t0 && t < t1) {
            extrema_points.push_back(curve.point_at(t));
        }
    }

    // Find extrema along each plane axis
    Vector axes[3] = {plane.x_axis(), plane.y_axis(), plane.z_axis()};
    const int NUM_SAMPLES = 20;
    double dt = (t1 - t0) / NUM_SAMPLES;

    for (int axis_idx = 0; axis_idx < 3; axis_idx++) {
        Vector axis = axes[axis_idx];

        for (int i = 0; i < NUM_SAMPLES; i++) {
            double t_start = t0 + i * dt;
            double t_end = t_start + dt;

            auto deriv_start = curve.evaluate(t_start, 1);
            auto deriv_end = curve.evaluate(t_end, 1);
            if (deriv_start.size() < 2 || deriv_end.size() < 2) continue;

            // C'(t) · axis
            double d_start = deriv_start[1].dot(axis);
            double d_end = deriv_end[1].dot(axis);

            if (d_start * d_end < 0) {
                double t_lo = t_start, t_hi = t_end;
                double t_root = (t_lo + t_hi) * 0.5;

                for (int iter = 0; iter < 20; iter++) {
                    auto deriv = curve.evaluate(t_root, 2);
                    if (deriv.size() < 3) break;

                    double f = deriv[1].dot(axis);
                    double fp = deriv[2].dot(axis);

                    if (std::abs(f) < 1e-12) break;

                    if (std::abs(fp) > 1e-14) {
                        double t_new = t_root - f / fp;
                        if (t_new >= t_lo && t_new <= t_hi) {
                            t_root = t_new;
                        } else {
                            if (f * d_start < 0) t_hi = t_root;
                            else t_lo = t_root;
                            t_root = (t_lo + t_hi) * 0.5;
                        }
                    } else {
                        t_root = (t_lo + t_hi) * 0.5;
                    }

                    auto deriv_check = curve.evaluate(t_root, 1);
                    if (deriv_check.size() >= 2) {
                        double f_check = deriv_check[1].dot(axis);
                        if (f_check * d_start < 0) {
                            t_hi = t_root;
                            d_end = f_check;
                        } else {
                            t_lo = t_root;
                            d_start = f_check;
                        }
                    }
                }

                extrema_points.push_back(curve.point_at(t_root));
            }
        }
    }

    return from_points(extrema_points, plane, inflate_amount);
}

OBB OBB::from_nurbssurface(const NurbsSurface& surface, double inflate_amount) {
    return obb_from_aabb(AABB::from_nurbssurface(surface, inflate_amount));
}

OBB OBB::from_nurbssurface(const NurbsSurface& surface, const Plane& plane, double inflate_amount) {
    if (!surface.is_valid() || surface.cv_count(0) == 0 || surface.cv_count(1) == 0) {
        return OBB();
    }
    std::vector<Point> points;
    for (int i = 0; i < surface.cv_count(0); i++) {
        for (int j = 0; j < surface.cv_count(1); j++) {
            points.push_back(surface.get_cv(i, j));
        }
    }
    return from_points(points, plane, inflate_amount);
}

AABB OBB::aabb() const {
    double ex = half_size[0], ey = half_size[1], ez = half_size[2];
    double hx = std::abs(x_axis[0]) * ex + std::abs(y_axis[0]) * ey + std::abs(z_axis[0]) * ez;
    double hy = std::abs(x_axis[1]) * ex + std::abs(y_axis[1]) * ey + std::abs(z_axis[1]) * ez;
    double hz = std::abs(x_axis[2]) * ex + std::abs(y_axis[2]) * ey + std::abs(z_axis[2]) * ez;
    return AABB(center[0], center[1], center[2], hx, hy, hz);
}

Point OBB::point_at(double x, double y, double z) const {
    return Point(
        center[0] + x * x_axis[0] + y * y_axis[0] + z * z_axis[0],
        center[1] + x * x_axis[1] + y * y_axis[1] + z * z_axis[1],
        center[2] + x * x_axis[2] + y * y_axis[2] + z * z_axis[2]
    );
}

Point OBB::min_point() const {
    return aabb().min_point();
}

Point OBB::max_point() const {
    return aabb().max_point();
}

std::array<Point, 8> OBB::corners() const {
    std::array<Point, 8> result;

    result[0] = point_at(half_size[0], half_size[1], -half_size[2]);
    result[1] = point_at(-half_size[0], half_size[1], -half_size[2]);
    result[2] = point_at(-half_size[0], -half_size[1], -half_size[2]);
    result[3] = point_at(half_size[0], -half_size[1], -half_size[2]);

    result[4] = point_at(half_size[0], half_size[1], half_size[2]);
    result[5] = point_at(-half_size[0], half_size[1], half_size[2]);
    result[6] = point_at(-half_size[0], -half_size[1], half_size[2]);
    result[7] = point_at(half_size[0], -half_size[1], half_size[2]);
    return result;
}

std::array<Point, 10> OBB::two_rectangles() const {
    std::array<Point, 10> result;

    result[0] = point_at(half_size[0], half_size[1], -half_size[2]);
    result[1] = point_at(-half_size[0], half_size[1], -half_size[2]);
    result[2] = point_at(-half_size[0], -half_size[1], -half_size[2]);
    result[3] = point_at(half_size[0], -half_size[1], -half_size[2]);
    result[4] = point_at(half_size[0], half_size[1], -half_size[2]);

    result[5] = point_at(half_size[0], half_size[1], half_size[2]);
    result[6] = point_at(-half_size[0], half_size[1], half_size[2]);
    result[7] = point_at(-half_size[0], -half_size[1], half_size[2]);
    result[8] = point_at(half_size[0], -half_size[1], half_size[2]);
    result[9] = point_at(half_size[0], half_size[1], half_size[2]);
    return result;
}

void OBB::inflate(double amount) {
    half_size = Vector(
        half_size[0] + amount,
        half_size[1] + amount,
        half_size[2] + amount
    );
}

double OBB::area() const {
    double hx = half_size[0], hy = half_size[1], hz = half_size[2];
    return 8.0 * (hx * hy + hy * hz + hz * hx);
}

double OBB::diagonal() const {
    double hx = half_size[0], hy = half_size[1], hz = half_size[2];
    return 2.0 * std::sqrt(hx * hx + hy * hy + hz * hz);
}

bool OBB::is_valid() const {
    return half_size[0] >= 0.0 && half_size[1] >= 0.0 && half_size[2] >= 0.0;
}

double OBB::volume() const {
    return 8.0 * half_size[0] * half_size[1] * half_size[2];
}

Point OBB::closest_point(const Point& pt) const {
    double dx = pt[0] - center[0], dy = pt[1] - center[1], dz = pt[2] - center[2];
    double lx = dx * x_axis[0] + dy * x_axis[1] + dz * x_axis[2];
    double ly = dx * y_axis[0] + dy * y_axis[1] + dz * y_axis[2];
    double lz = dx * z_axis[0] + dy * z_axis[1] + dz * z_axis[2];
    lx = std::max(-half_size[0], std::min(half_size[0], lx));
    ly = std::max(-half_size[1], std::min(half_size[1], ly));
    lz = std::max(-half_size[2], std::min(half_size[2], lz));
    return Point(
        center[0] + lx * x_axis[0] + ly * y_axis[0] + lz * z_axis[0],
        center[1] + lx * x_axis[1] + ly * y_axis[1] + lz * z_axis[1],
        center[2] + lx * x_axis[2] + ly * y_axis[2] + lz * z_axis[2]
    );
}

bool OBB::contains(const Point& pt) const {
    double dx = pt[0] - center[0], dy = pt[1] - center[1], dz = pt[2] - center[2];
    double lx = std::abs(dx * x_axis[0] + dy * x_axis[1] + dz * x_axis[2]);
    double ly = std::abs(dx * y_axis[0] + dy * y_axis[1] + dz * y_axis[2]);
    double lz = std::abs(dx * z_axis[0] + dy * z_axis[1] + dz * z_axis[2]);
    return lx <= half_size[0] && ly <= half_size[1] && lz <= half_size[2];
}

Point OBB::corner(bool x_max, bool y_max, bool z_max) const {
    double ox = x_max ? half_size[0] : -half_size[0];
    double oy = y_max ? half_size[1] : -half_size[1];
    double oz = z_max ? half_size[2] : -half_size[2];
    return point_at(ox, oy, oz);
}

std::array<Point, 8> OBB::get_corners() const {
    return corners();
}

std::vector<Line> OBB::get_edges() const {
    auto c = corners();
    return {
        Line(c[0][0], c[0][1], c[0][2], c[1][0], c[1][1], c[1][2]),
        Line(c[1][0], c[1][1], c[1][2], c[2][0], c[2][1], c[2][2]),
        Line(c[2][0], c[2][1], c[2][2], c[3][0], c[3][1], c[3][2]),
        Line(c[3][0], c[3][1], c[3][2], c[0][0], c[0][1], c[0][2]),
        Line(c[4][0], c[4][1], c[4][2], c[5][0], c[5][1], c[5][2]),
        Line(c[5][0], c[5][1], c[5][2], c[6][0], c[6][1], c[6][2]),
        Line(c[6][0], c[6][1], c[6][2], c[7][0], c[7][1], c[7][2]),
        Line(c[7][0], c[7][1], c[7][2], c[4][0], c[4][1], c[4][2]),
        Line(c[0][0], c[0][1], c[0][2], c[4][0], c[4][1], c[4][2]),
        Line(c[1][0], c[1][1], c[1][2], c[5][0], c[5][1], c[5][2]),
        Line(c[2][0], c[2][1], c[2][2], c[6][0], c[6][1], c[6][2]),
        Line(c[3][0], c[3][1], c[3][2], c[7][0], c[7][1], c[7][2]),
    };
}

void OBB::union_with(const OBB& other) {
    double min_x = -half_size[0], max_x = half_size[0];
    double min_y = -half_size[1], max_y = half_size[1];
    double min_z = -half_size[2], max_z = half_size[2];
    for (const auto& c : other.corners()) {
        double dx = c[0] - center[0], dy = c[1] - center[1], dz = c[2] - center[2];
        double lx = dx * x_axis[0] + dy * x_axis[1] + dz * x_axis[2];
        double ly = dx * y_axis[0] + dy * y_axis[1] + dz * y_axis[2];
        double lz = dx * z_axis[0] + dy * z_axis[1] + dz * z_axis[2];
        min_x = std::min(min_x, lx); max_x = std::max(max_x, lx);
        min_y = std::min(min_y, ly); max_y = std::max(max_y, ly);
        min_z = std::min(min_z, lz); max_z = std::max(max_z, lz);
    }
    double ox = (min_x + max_x) * 0.5;
    double oy = (min_y + max_y) * 0.5;
    double oz = (min_z + max_z) * 0.5;
    center = Point(
        center[0] + ox * x_axis[0] + oy * y_axis[0] + oz * z_axis[0],
        center[1] + ox * x_axis[1] + oy * y_axis[1] + oz * z_axis[1],
        center[2] + ox * x_axis[2] + oy * y_axis[2] + oz * z_axis[2]
    );
    half_size = Vector((max_x - min_x) * 0.5, (max_y - min_y) * 0.5, (max_z - min_z) * 0.5);
}

bool OBB::separating_plane_exists(const Vector& relative_position, const Vector& axis, const OBB& box1, const OBB& box2) {
    // Fallback (unused by optimized path, but kept for API completeness)
    Vector rp = relative_position;
    double dot_rp = std::abs(rp.dot(axis));
    Vector v1 = box1.x_axis * box1.half_size[0];
    Vector v2 = box1.y_axis * box1.half_size[1];
    Vector v3 = box1.z_axis * box1.half_size[2];
    double proj1 = std::abs(v1.dot(axis)) + std::abs(v2.dot(axis)) + std::abs(v3.dot(axis));
    Vector v4 = box2.x_axis * box2.half_size[0];
    Vector v5 = box2.y_axis * box2.half_size[1];
    Vector v6 = box2.z_axis * box2.half_size[2];
    double proj2 = std::abs(v4.dot(axis)) + std::abs(v5.dot(axis)) + std::abs(v6.dot(axis));
    return dot_rp > (proj1 + proj2);
}

void OBB::transform() {
  center.xform = xform; center.transform();
  x_axis.xform = xform; x_axis.transform();
  y_axis.xform = xform; y_axis.transform();
  z_axis.xform = xform; z_axis.transform();
  xform = Xform::identity();
}

OBB OBB::transformed() const {
  OBB result = *this;
  result.transform();
  return result;
}

bool OBB::collides_with_broad(const OBB& other) const {
    if (!aabb().intersects(other.aabb())) return false;
    return collides_with(other);
}

bool OBB::collides_with(const OBB& other) const {
    return collides_with_rtcd(other);
}

bool OBB::collides_with_rtcd(const OBB& other) const {
    const double EPS = 1e-9;
    const Vector A0 = x_axis;
    const Vector A1 = y_axis;
    const Vector A2 = z_axis;
    const Vector B0 = other.x_axis;
    const Vector B1 = other.y_axis;
    const Vector B2 = other.z_axis;
    const double a0 = half_size[0];
    const double a1 = half_size[1];
    const double a2 = half_size[2];
    const double b0 = other.half_size[0];
    const double b1 = other.half_size[1];
    const double b2 = other.half_size[2];

    double R00 = A0.dot(B0), R01 = A0.dot(B1), R02 = A0.dot(B2);
    double R10 = A1.dot(B0), R11 = A1.dot(B1), R12 = A1.dot(B2);
    double R20 = A2.dot(B0), R21 = A2.dot(B1), R22 = A2.dot(B2);

    Vector d(other.center[0] - center[0], other.center[1] - center[1], other.center[2] - center[2]);
    double t0 = d.dot(A0);
    double t1 = d.dot(A1);
    double t2 = d.dot(A2);

    double AbsR00 = std::abs(R00) + EPS, AbsR01 = std::abs(R01) + EPS, AbsR02 = std::abs(R02) + EPS;
    double AbsR10 = std::abs(R10) + EPS, AbsR11 = std::abs(R11) + EPS, AbsR12 = std::abs(R12) + EPS;
    double AbsR20 = std::abs(R20) + EPS, AbsR21 = std::abs(R21) + EPS, AbsR22 = std::abs(R22) + EPS;

    double ra, rb, t;

    ra = a0; rb = b0 * AbsR00 + b1 * AbsR01 + b2 * AbsR02; t = std::abs(t0); if (t > ra + rb) return false;
    ra = a1; rb = b0 * AbsR10 + b1 * AbsR11 + b2 * AbsR12; t = std::abs(t1); if (t > ra + rb) return false;
    ra = a2; rb = b0 * AbsR20 + b1 * AbsR21 + b2 * AbsR22; t = std::abs(t2); if (t > ra + rb) return false;

    ra = a0 * AbsR00 + a1 * AbsR10 + a2 * AbsR20; rb = b0; t = std::abs(t0 * R00 + t1 * R10 + t2 * R20); if (t > ra + rb) return false;
    ra = a0 * AbsR01 + a1 * AbsR11 + a2 * AbsR21; rb = b1; t = std::abs(t0 * R01 + t1 * R11 + t2 * R21); if (t > ra + rb) return false;
    ra = a0 * AbsR02 + a1 * AbsR12 + a2 * AbsR22; rb = b2; t = std::abs(t0 * R02 + t1 * R12 + t2 * R22); if (t > ra + rb) return false;

    ra = a1 * AbsR20 + a2 * AbsR10; rb = b1 * AbsR02 + b2 * AbsR01; t = std::abs(t2 * R10 - t1 * R20); if (t > ra + rb) return false;
    ra = a1 * AbsR21 + a2 * AbsR11; rb = b0 * AbsR02 + b2 * AbsR00; t = std::abs(t2 * R11 - t1 * R21); if (t > ra + rb) return false;
    ra = a1 * AbsR22 + a2 * AbsR12; rb = b0 * AbsR01 + b1 * AbsR00; t = std::abs(t2 * R12 - t1 * R22); if (t > ra + rb) return false;

    ra = a0 * AbsR20 + a2 * AbsR00; rb = b1 * AbsR12 + b2 * AbsR11; t = std::abs(t0 * R20 - t2 * R00); if (t > ra + rb) return false;
    ra = a0 * AbsR21 + a2 * AbsR01; rb = b0 * AbsR12 + b2 * AbsR10; t = std::abs(t0 * R21 - t2 * R01); if (t > ra + rb) return false;
    ra = a0 * AbsR22 + a2 * AbsR02; rb = b0 * AbsR11 + b1 * AbsR10; t = std::abs(t0 * R22 - t2 * R02); if (t > ra + rb) return false;

    ra = a0 * AbsR10 + a1 * AbsR00; rb = b1 * AbsR22 + b2 * AbsR21; t = std::abs(t1 * R00 - t0 * R10); if (t > ra + rb) return false;
    ra = a0 * AbsR11 + a1 * AbsR01; rb = b0 * AbsR22 + b2 * AbsR20; t = std::abs(t1 * R01 - t0 * R11); if (t > ra + rb) return false;
    ra = a0 * AbsR12 + a1 * AbsR02; rb = b0 * AbsR21 + b1 * AbsR20; t = std::abs(t1 * R02 - t0 * R12); if (t > ra + rb) return false;

    return true;
}

bool OBB::collides_with_naive(const OBB& other) const {
    Point center_pt(center[0], center[1], center[2]);
    Point other_center_pt(other.center[0], other.center[1], other.center[2]);
    Vector relative_position = Vector::from_points(center_pt, other_center_pt);

    const Vector x1 = x_axis, y1 = y_axis, z1 = z_axis;
    const Vector x2 = other.x_axis, y2 = other.y_axis, z2 = other.z_axis;

    if (separating_plane_exists(relative_position, x1, *this, other)) return false;
    if (separating_plane_exists(relative_position, y1, *this, other)) return false;
    if (separating_plane_exists(relative_position, z1, *this, other)) return false;
    if (separating_plane_exists(relative_position, x2, *this, other)) return false;
    if (separating_plane_exists(relative_position, y2, *this, other)) return false;
    if (separating_plane_exists(relative_position, z2, *this, other)) return false;

    if (separating_plane_exists(relative_position, x1.cross(x2), *this, other)) return false;
    if (separating_plane_exists(relative_position, x1.cross(y2), *this, other)) return false;
    if (separating_plane_exists(relative_position, x1.cross(z2), *this, other)) return false;
    if (separating_plane_exists(relative_position, y1.cross(x2), *this, other)) return false;
    if (separating_plane_exists(relative_position, y1.cross(y2), *this, other)) return false;
    if (separating_plane_exists(relative_position, y1.cross(z2), *this, other)) return false;
    if (separating_plane_exists(relative_position, z1.cross(x2), *this, other)) return false;
    if (separating_plane_exists(relative_position, z1.cross(y2), *this, other)) return false;
    if (separating_plane_exists(relative_position, z1.cross(z2), *this, other)) return false;

    return true;
}

nlohmann::ordered_json OBB::jsondump() const {
    return {
        {"type", "OBB"},
        {"center", center.jsondump()},
        {"x_axis", x_axis.jsondump()},
        {"y_axis", y_axis.jsondump()},
        {"z_axis", z_axis.jsondump()},
        {"half_size", half_size.jsondump()},
        {"guid", guid()},
        {"name", name}
    };
}

OBB OBB::jsonload(const nlohmann::json& data) {
    OBB box;
    box.center = Point::jsonload(data["center"]);
    box.x_axis = Vector::jsonload(data["x_axis"]);
    box.y_axis = Vector::jsonload(data["y_axis"]);
    box.z_axis = Vector::jsonload(data["z_axis"]);
    box.half_size = Vector::jsonload(data["half_size"]);
    box.guid() = data["guid"];
    box.name = data["name"];
    return box;
}

void OBB::to_json_file(const std::string& filepath) const {
    std::ofstream file(filepath);
    file << jsondump().dump(4);
}

OBB OBB::from_json_file(const std::string& filepath) {
    std::ifstream file(filepath);
    nlohmann::json data;
    file >> data;
    return jsonload(data);
}

std::string OBB::file_json_dumps() const {
    return jsondump().dump();
}

OBB OBB::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::json::parse(json_string));
}

void OBB::file_json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    file << jsondump().dump(4);
}

OBB OBB::file_json_load(const std::string& filename) {
    std::ifstream file(filename);
    nlohmann::json data;
    file >> data;
    return jsonload(data);
}

std::string OBB::pb_dumps() const {
    session_proto::BoundingBox proto;
    proto.mutable_center()->ParseFromString(center.pb_dumps());
    proto.mutable_x_axis()->ParseFromString(x_axis.pb_dumps());
    proto.mutable_y_axis()->ParseFromString(y_axis.pb_dumps());
    proto.mutable_z_axis()->ParseFromString(z_axis.pb_dumps());
    proto.mutable_half_size()->ParseFromString(half_size.pb_dumps());
    proto.set_guid(guid());
    proto.set_name(name);
    auto* xform_proto = proto.mutable_xform();
    xform_proto->set_guid(xform.guid());
    xform_proto->set_name(xform.name);
    for (int i = 0; i < 16; ++i) {
        xform_proto->add_matrix(xform.m[i]);
    }
    return proto.SerializeAsString();
}

OBB OBB::pb_loads(const std::string& data) {
    session_proto::BoundingBox proto;
    proto.ParseFromString(data);
    Point c = Point::pb_loads(proto.center().SerializeAsString());
    Vector xa = Vector::pb_loads(proto.x_axis().SerializeAsString());
    Vector ya = Vector::pb_loads(proto.y_axis().SerializeAsString());
    Vector za = Vector::pb_loads(proto.z_axis().SerializeAsString());
    Vector hs = Vector::pb_loads(proto.half_size().SerializeAsString());
    OBB box(c, xa, ya, za, hs);
    box.guid() = proto.guid();
    box.name = proto.name();
    if (proto.has_xform()) {
        box.xform = Xform::pb_loads(proto.xform().SerializeAsString());
    }
    return box;
}

void OBB::pb_dump(const std::string& filename) const {
    std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

OBB OBB::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
    return pb_loads(data);
}

}
