#include "intersection.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "boolean_polyline.h"
#include "closest.h"
#include "spatial_bvh.h"
#include "tolerance.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <tuple>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Lines and planes
// ═══════════════════════════════════════════════════════════════════════════

int Intersection::solve_3x3(
    const double row0[3],
    const double row1[3],
    const double row2[3],
    double d0,
    double d1,
    double d2,
    double& x,
    double& y,
    double& z,
    double& pivot_ratio
) {

    int i;
    int j;
    double* p0;
    double* p1;
    double* p2;
    double temp;
    double workarray[12];
    double maxpiv;
    double minpiv;

    const int sizeof_row = 3 * sizeof(row0[0]);

    pivot_ratio = x = y = z = 0.0;

    temp = std::fabs(row0[0]);
    i = j = 0;
    double val = std::fabs(row0[1]);

    if (val > temp) {
        temp = val;
        j = 1;
    }

    val = std::fabs(row0[2]);

    if (val > temp) {
        temp = val;
        j = 2;
    }

    val = std::fabs(row1[0]);

    if (val > temp) {
        temp = val;
        i = 1;
        j = 0;
    }

    val = std::fabs(row1[1]);

    if (val > temp) {
        temp = val;
        i = 1;
        j = 1;
    }

    val = std::fabs(row1[2]);

    if (val > temp) {
        temp = val;
        i = 1;
        j = 2;
    }

    val = std::fabs(row2[0]);

    if (val > temp) {
        temp = val;
        i = 2;
        j = 0;
    }

    val = std::fabs(row2[1]);

    if (val > temp) {
        temp = val;
        i = 2;
        j = 1;
    }

    val = std::fabs(row2[2]);

    if (val > temp) {
        temp = val;
        i = 2;
        j = 2;
    }

    if (temp == 0.0)
        return 0;

    maxpiv = minpiv = std::fabs(temp);
    p0 = workarray;

    switch (i) {
    case 1:
        std::memcpy(p0, row1, sizeof_row);
        p0[3] = d1;
        p0 += 4;
        std::memcpy(p0, row0, sizeof_row);
        p0[3] = d0;
        p0 += 4;
        std::memcpy(p0, row2, sizeof_row);
        p0[3] = d2;
        break;
    case 2:
        std::memcpy(p0, row2, sizeof_row);
        p0[3] = d2;
        p0 += 4;
        std::memcpy(p0, row1, sizeof_row);
        p0[3] = d1;
        p0 += 4;
        std::memcpy(p0, row0, sizeof_row);
        p0[3] = d0;
        break;
    default:
        std::memcpy(p0, row0, sizeof_row);
        p0[3] = d0;
        p0 += 4;
        std::memcpy(p0, row1, sizeof_row);
        p0[3] = d1;
        p0 += 4;
        std::memcpy(p0, row2, sizeof_row);
        p0[3] = d2;
        break;
    }

    double* x_addr = &x;
    double* y_addr = &y;
    double* z_addr = &z;

    switch (j) {
    case 1:
        std::swap(x_addr, y_addr);
        p0 = &workarray[0];
        std::swap(p0[0], p0[1]);
        p0 += 4;
        std::swap(p0[0], p0[1]);
        p0 += 4;
        std::swap(p0[0], p0[1]);
        break;
    case 2:
        std::swap(x_addr, z_addr);
        p0 = &workarray[0];
        std::swap(p0[0], p0[2]);
        p0 += 4;
        std::swap(p0[0], p0[2]);
        p0 += 4;
        std::swap(p0[0], p0[2]);
        break;
    }

    temp = 1.0 / workarray[0];
    p0 = p1 = workarray + 1;
    *p1++ *= temp;
    *p1++ *= temp;
    *p1++ *= temp;
    temp = -(*p1++);

    if (temp != 0.0) {
        *p1++ += temp * (*p0++);
        *p1++ += temp * (*p0++);
        *p1++ += temp * (*p0);
        p0 -= 2;
    } else {
        p1 += 3;
    }

    temp = -(*p1++);

    if (temp != 0.0) {
        *p1++ += temp * (*p0++);
        *p1++ += temp * (*p0++);
        *p1++ += temp * (*p0);
        p0 -= 2;
    }

    temp = std::fabs(workarray[5]);
    i = j = 0;
    val = std::fabs(workarray[6]);

    if (val > temp) {
        temp = val;
        j = 1;
    }

    val = std::fabs(workarray[9]);

    if (val > temp) {
        temp = val;
        i = 1;
        j = 0;
    }

    val = std::fabs(workarray[10]);

    if (val > temp) {
        temp = val;
        i = j = 1;
    }

    if (temp == 0.0)
        return 1;

    val = std::fabs(temp);

    if (val > maxpiv)
        maxpiv = val;
    else if (val < minpiv)
        minpiv = val;

    if (j) {
        p0 = workarray + 1;
        p1 = p0 + 1;
        std::swap(*p0, *p1);
        p0 += 4;
        p1 += 4;
        std::swap(*p0, *p1);
        p0 += 4;
        p1 += 4;
        std::swap(*p0, *p1);
        std::swap(y_addr, z_addr);
    }

    if (i) {
        p0 = workarray + 1;
        p1 = p0 + 8;
        p2 = p0 + 4;
    } else {
        p0 = workarray + 1;
        p1 = p0 + 4;
        p2 = p0 + 8;
    }

    temp = 1.0 / (*p1++);
    *p1++ *= temp;
    *p1 *= temp;
    p1--;
    temp = -(*p0++);

    if (temp != 0.0) {
        *p0++ += temp * (*p1++);
        *p0 += temp * (*p1);
        p0--;
        p1--;
    }

    temp = -(*p2++);

    if (temp != 0.0) {
        *p2++ += temp * (*p1++);
        *p2 += temp * (*p1);
        p2--;
        p1--;
    }

    temp = *p2++;

    if (temp == 0.0)
        return 2;

    val = std::fabs(temp);

    if (val > maxpiv)
        maxpiv = val;
    else if (val < minpiv)
        minpiv = val;

    *p2 /= temp;
    temp = -(*p1++);

    if (temp != 0.0)
        *p1 += temp * (*p2);

    temp = -(*p0++);

    if (temp != 0.0)
        *p0 += temp * (*p2);

    *x_addr = workarray[3];

    if (i) {
        *y_addr = workarray[11];
        *z_addr = workarray[7];
    } else {
        *y_addr = workarray[7];
        *z_addr = workarray[11];
    }

    pivot_ratio = minpiv / maxpiv;

    return 3;
}

double Intersection::plane_value_at(const Plane& plane, const Point& point) {
    return plane.a() * point[0] + plane.b() * point[1] + plane.c() * point[2] + plane.d();
}

bool Intersection::line_line(const Line& line0, const Line& line1, Point& output, double tolerance) {

    double t0;
    double t1;
    bool rc = line_line_parameters(line0, line1, t0, t1, tolerance, true, false);

    if (rc) {
        Point p0 = line0.point_at(t0);
        Point p1 = line1.point_at(t1);
        output = Point((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5);
    }

    return rc;
}

bool Intersection::line_line_parameters(
    const Line& line0,
    const Line& line1,
    double& t0,
    double& t1,
    double tolerance,
    bool intersect_segments,
    bool near_parallel_as_closest
) {

    Point p0_start = line0.start();
    Point p0_end = line0.end();
    Point p1_start = line1.start();
    Point p1_end = line1.end();

    if (p0_start[0] == p1_start[0] && p0_start[1] == p1_start[1] && p0_start[2] == p1_start[2]) {
        t0 = 0.0;
        t1 = 0.0;

        return true;
    }

    if (p0_start[0] == p1_end[0] && p0_start[1] == p1_end[1] && p0_start[2] == p1_end[2]) {
        t0 = 0.0;
        t1 = 1.0;

        return true;
    }

    if (p0_end[0] == p1_start[0] && p0_end[1] == p1_start[1] && p0_end[2] == p1_start[2]) {
        t0 = 1.0;
        t1 = 0.0;

        return true;
    }

    if (p0_end[0] == p1_end[0] && p0_end[1] == p1_end[1] && p0_end[2] == p1_end[2]) {
        t0 = 1.0;
        t1 = 1.0;

        return true;
    }

    Vector A = line0.to_vector();
    Vector B = line1.to_vector();
    Vector C = p1_start - p0_start;

    double AA = A.dot(A);
    double BB = B.dot(B);
    double AB = A.dot(B);
    double AC = A.dot(C);
    double BC = B.dot(C);

    double det = AA * BB - AB * AB;

    double zero_tol = std::max(AA, BB) * std::numeric_limits<double>::epsilon();

    if (std::fabs(det) < zero_tol) {
        if (!near_parallel_as_closest)
            return false;

        t0 = (AA > 0.0) ? (AC / AA) : 0.0;
        t1 = (BB > 0.0) ? ((BC + t0 * AB) / BB) : 0.0;

        if (intersect_segments) {
            if (t0 < 0.0)
                t0 = 0.0;
            else if (t0 > 1.0)
                t0 = 1.0;

            if (t1 < 0.0)
                t1 = 0.0;
            else if (t1 > 1.0)
                t1 = 1.0;
        }

        if (tolerance > 0.0) {
            Point pt0p = line0.point_at(t0);
            Point pt1p = line1.point_at(t1);

            return pt0p.distance(pt1p) <= tolerance;
        }

        return true;
    }

    double inv_det = 1.0 / det;
    t0 = (BB * AC - AB * BC) * inv_det;
    t1 = (AB * AC - AA * BC) * inv_det;

    if (intersect_segments) {
        if (t0 < 0.0)
            t0 = 0.0;
        else if (t0 > 1.0)
            t0 = 1.0;

        if (t1 < 0.0)
            t1 = 0.0;
        else if (t1 > 1.0)
            t1 = 1.0;
    }

    if (tolerance > 0.0) {
        Point pt0 = line0.point_at(t0);
        Point pt1 = line1.point_at(t1);
        double dist = pt0.distance(pt1);

        if (dist > tolerance)
            return false;
    }

    return true;
}

bool Intersection::plane_plane(const Plane& plane0, const Plane& plane1, Line& output) {

    Vector d = plane1.z_axis().cross(plane0.z_axis());

    Point p = Point(
        (plane0.origin()[0] + plane1.origin()[0]) * 0.5,
        (plane0.origin()[1] + plane1.origin()[1]) * 0.5,
        (plane0.origin()[2] + plane1.origin()[2]) * 0.5
    );

    Plane plane2 = Plane::from_point_normal(p, d);

    Point output_p;
    bool rc = plane_plane_plane(plane0, plane1, plane2, output_p);

    if (!rc)
        return false;

    output = Line::from_points(output_p, output_p + d);

    return true;
}

bool Intersection::plane_plane_to_line_canonical(const Plane& plane0, const Plane& plane1, Line& output) {

    Vector n0 = plane0.z_axis();
    Vector n1 = plane1.z_axis();
    Vector d = n1.cross(n0);
    double d_sq = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];

    if (d_sq < 1e-20)
        return false;

    double k0 = n0[0] * plane0.origin()[0] + n0[1] * plane0.origin()[1] + n0[2] * plane0.origin()[2];
    double k1 = n1[0] * plane1.origin()[0] + n1[1] * plane1.origin()[1] + n1[2] * plane1.origin()[2];
    double n0n0 = n0[0] * n0[0] + n0[1] * n0[1] + n0[2] * n0[2];
    double n1n1 = n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2];
    double n0n1 = n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2];
    double det = n0n0 * n1n1 - n0n1 * n0n1;

    if (std::abs(det) < 1e-20)
        return false;

    double c0 = (k0 * n1n1 - k1 * n0n1) / det;
    double c1 = (k1 * n0n0 - k0 * n0n1) / det;
    Point anchor(c0 * n0[0] + c1 * n1[0], c0 * n0[1] + c1 * n1[1], c0 * n0[2] + c1 * n1[2]);
    output = Line::from_points(anchor, anchor + d);

    return true;
}

bool Intersection::line_plane(const Line& line, const Plane& plane, Point& output, bool is_finite) {

    bool rc = false;
    double a;
    double b;
    double d;
    double fd;
    double t;

    Point pt0 = line.start();
    Point pt1 = line.end();

    a = plane_value_at(plane, pt0);
    b = plane_value_at(plane, pt1);
    d = a - b;

    if (d == 0.0) {
        if (std::fabs(a) < std::fabs(b))
            t = 0.0;
        else if (std::fabs(b) < std::fabs(a))
            t = 1.0;
        else
            t = 0.5;
    } else {
        d = 1.0 / d;
        fd = std::fabs(d);

        if (fd > 1.0 &&
            (std::fabs(a) >= std::numeric_limits<double>::max() / fd ||
             std::fabs(b) >= std::numeric_limits<double>::max() / fd)) {

            t = 0.5;
        } else {
            t = a / (a - b);
            rc = true;
        }
    }

    const double s = 1.0 - t;

    output = Point(
        (line[0] == line[3]) ? line[0] : s * line[0] + t * line[3],
        (line[1] == line[4]) ? line[1] : s * line[1] + t * line[4],
        (line[2] == line[5]) ? line[2] : s * line[2] + t * line[5]
    );

    if (is_finite && (t < 0.0 || t > 1.0))
        return false;

    return rc;
}

bool Intersection::plane_plane_plane(const Plane& plane0, const Plane& plane1, const Plane& plane2, Point& output) {

    double pr = 0.0;
    double x;
    double y;
    double z;

    const double plane_0[3] = {plane0.a(), plane0.b(), plane0.c()};
    const double plane_1[3] = {plane1.a(), plane1.b(), plane1.c()};
    const double plane_2[3] = {plane2.a(), plane2.b(), plane2.c()};

    const int rank = solve_3x3(plane_0, plane_1, plane_2, -plane0.d(), -plane1.d(), -plane2.d(), x, y, z, pr);

    output = Point(x, y, z);

    return (rank == 3 && pr > 1e-12);
}

// ═══════════════════════════════════════════════════════════════════════════
// Rays
// ═══════════════════════════════════════════════════════════════════════════

bool Intersection::ray_box(
    const Point& origin,
    const Vector& direction,
    const OBB& box,
    double t0,
    double t1,
    double& tmin,
    double& tmax
) {

    Point box_min = box.min_point();
    Point box_max = box.max_point();

    Vector inv_dir(
        (direction[0] != 0.0) ? 1.0 / direction[0] : std::numeric_limits<double>::max(),
        (direction[1] != 0.0) ? 1.0 / direction[1] : std::numeric_limits<double>::max(),
        (direction[2] != 0.0) ? 1.0 / direction[2] : std::numeric_limits<double>::max()
    );

    double tx1 = (box_min[0] - origin[0]) * inv_dir[0];
    double tx2 = (box_max[0] - origin[0]) * inv_dir[0];

    tmin = std::min(tx1, tx2);
    tmax = std::max(tx1, tx2);

    double ty1 = (box_min[1] - origin[1]) * inv_dir[1];
    double ty2 = (box_max[1] - origin[1]) * inv_dir[1];

    tmin = std::max(tmin, std::min(ty1, ty2));
    tmax = std::min(tmax, std::max(ty1, ty2));

    double tz1 = (box_min[2] - origin[2]) * inv_dir[2];
    double tz2 = (box_max[2] - origin[2]) * inv_dir[2];

    tmin = std::max(tmin, std::min(tz1, tz2));
    tmax = std::min(tmax, std::max(tz1, tz2));

    tmin = std::max(tmin, t0);
    tmax = std::min(tmax, t1);

    return tmax >= tmin;
}

bool Intersection::ray_box(const Line& line, const OBB& box, double t0, double t1, double& tmin, double& tmax) {

    Point origin = line.start();
    Vector direction = line.to_vector();

    return ray_box(origin, direction, box, t0, t1, tmin, tmax);
}

bool Intersection::ray_box(
    const Line& line,
    const OBB& box,
    double t0,
    double t1,
    std::vector<Point>& intersection_points
) {

    double tmin;
    double tmax;
    Point origin = line.start();
    Vector direction = line.to_vector();

    bool hit = ray_box(origin, direction, box, t0, t1, tmin, tmax);

    if (hit) {
        intersection_points.clear();

        Point entry = origin + direction * tmin;
        intersection_points.push_back(entry);

        Point exit = origin + direction * tmax;
        intersection_points.push_back(exit);
    }

    return hit;
}

int Intersection::ray_sphere(
    const Point& origin,
    const Vector& direction,
    const Point& center,
    double radius,
    double& t0,
    double& t1
) {

    Vector o = origin - center;

    double a = direction.dot(direction);
    double b = 2.0 * direction.dot(o);
    double c = o.dot(o) - (radius * radius);

    double disc = b * b - 4.0 * a * c;

    if (disc < 0.0)
        return 0;

    double distSqrt = std::sqrt(disc);
    double q;

    if (b < 0.0)
        q = (-b - distSqrt) / 2.0;
    else
        q = (-b + distSqrt) / 2.0;

    t0 = q / a;
    double _t1 = c / q;

    if (_t1 == t0)
        return 1;

    t1 = _t1;

    if (t0 > t1)
        std::swap(t0, t1);

    return 2;
}

bool Intersection::ray_sphere(
    const Line& line,
    const Point& center,
    double radius,
    std::vector<Point>& intersection_points
) {

    Point origin = line.start();
    Vector direction = line.to_vector();

    double t0;
    double t1;
    int hits = ray_sphere(origin, direction, center, radius, t0, t1);

    if (hits == 0)
        return false;

    intersection_points.clear();

    Point p0(origin[0] + direction[0] * t0, origin[1] + direction[1] * t0, origin[2] + direction[2] * t0);
    intersection_points.push_back(p0);

    if (hits == 2) {
        Point p1(origin[0] + direction[0] * t1, origin[1] + direction[1] * t1, origin[2] + direction[2] * t1);
        intersection_points.push_back(p1);
    }

    return true;
}

bool Intersection::ray_triangle(
    const Point& origin,
    const Vector& direction,
    const Point& v0,
    const Point& v1,
    const Point& v2,
    double epsilon,
    double& t,
    double& u,
    double& v,
    bool& parallel
) {

    Vector edge1 = v1 - v0;
    Vector edge2 = v2 - v0;
    Vector pvec = direction.cross(edge2);

    double det = edge1.dot(pvec);

    if (det > -epsilon && det < epsilon) {
        parallel = true;

        return false;
    }

    parallel = false;
    double inv_det = 1.0 / det;

    Vector tvec = origin - v0;
    u = tvec.dot(pvec) * inv_det;

    if (u < 0.0 - epsilon || u > 1.0 + epsilon)
        return false;

    Vector qvec = tvec.cross(edge1);
    v = direction.dot(qvec) * inv_det;

    if (v < 0.0 - epsilon || u + v > 1.0 + epsilon)
        return false;

    t = edge2.dot(qvec) * inv_det;

    return true;
}

bool Intersection::ray_triangle(
    const Line& line,
    const Point& v0,
    const Point& v1,
    const Point& v2,
    double epsilon,
    Point& output
) {

    Point origin = line.start();
    Vector direction = line.to_vector();

    double t;
    double u;
    double v;
    bool parallel;

    if (!ray_triangle(origin, direction, v0, v1, v2, epsilon, t, u, v, parallel))
        return false;

    output = origin + direction * t;

    return true;
}

/// Whether hit a sorts before hit b: smaller t, ties within 1e-6 broken by the lower face index.
static bool ray_hit_before(const Intersection::RayHit& a, const Intersection::RayHit& b) {

    const double eps = 1e-6;
    const double dt = a.t - b.t;

    if (std::fabs(dt) <= eps)
        return a.face_index < b.face_index;

    return a.t < b.t;
}

bool Intersection::ray_mesh(
    const Point& origin,
    const Vector& direction,
    const Mesh& mesh,
    std::vector<RayHit>& hits,
    bool find_all
) {

    hits.clear();

    const std::pair<std::vector<Point>, std::vector<std::vector<size_t>>> mesh_data = mesh.to_vertices_and_faces();
    const std::vector<Point>& vertices = mesh_data.first;
    const std::vector<std::vector<size_t>>& faces = mesh_data.second;

    for (size_t i = 0; i < faces.size(); ++i) {
        const std::vector<size_t>& face = faces[i];

        if (face.size() < 3)
            continue;

        for (size_t j = 1; j < face.size() - 1; ++j) {
            const Point& v0 = vertices[face[0]];
            const Point& v1 = vertices[face[j]];
            const Point& v2 = vertices[face[j + 1]];

            double t;
            double u;
            double v;
            bool parallel;

            if (ray_triangle(
                    origin,
                    direction,
                    v0,
                    v1,
                    v2,
                    static_cast<double>(Tolerance::ZERO_TOLERANCE),
                    t,
                    u,
                    v,
                    parallel
                )) {

                if (t >= 0.0) {
                    Point hit_point(
                        origin[0] + t * direction[0],
                        origin[1] + t * direction[1],
                        origin[2] + t * direction[2]
                    );

                    hits.emplace_back(t, hit_point, u, v, static_cast<int>(i));

                    if (!find_all)
                        return true;
                }
            }
        }
    }

    if (!hits.empty()) {
        std::sort(hits.begin(), hits.end(), ray_hit_before);

        return true;
    }

    return false;
}

bool Intersection::ray_mesh_bvh(
    const Point& origin,
    const Vector& direction,
    const Mesh& mesh,
    std::vector<RayHit>& hits,
    bool find_all
) {

    hits.clear();

    std::vector<int> candidates_list;

    if (!mesh.triangle_bvh_ray_cast(origin, direction, candidates_list, find_all))
        return false;

    bool any_hit = false;
    RayHit best_hit;
    double best_t = std::numeric_limits<double>::infinity();
    int best_face = std::numeric_limits<int>::max();

    for (int tri_id : candidates_list) {
        size_t face_idx;
        size_t sub_idx;
        Point v0;
        Point v1;
        Point v2;

        if (!mesh.get_triangle_by_id(tri_id, face_idx, sub_idx, v0, v1, v2))
            continue;

        double t;
        double u;
        double v;
        bool parallel;

        if (ray_triangle(
                origin,
                direction,
                v0,
                v1,
                v2,
                static_cast<double>(Tolerance::ZERO_TOLERANCE),
                t,
                u,
                v,
                parallel
            )) {
            if (t >= 0.0) {
                Point hit_point(
                    origin[0] + t * direction[0],
                    origin[1] + t * direction[1],
                    origin[2] + t * direction[2]
                );

                if (find_all) {
                    hits.emplace_back(t, hit_point, u, v, static_cast<int>(face_idx));
                } else {
                    const double eps = 1e-6;

                    if (t < best_t - eps || (std::fabs(t - best_t) <= eps && static_cast<int>(face_idx) < best_face)) {
                        best_t = t;
                        best_face = static_cast<int>(face_idx);
                        best_hit = RayHit(t, hit_point, u, v, static_cast<int>(face_idx));
                        any_hit = true;
                    }
                }
            }
        }
    }

    if (find_all) {
        if (!hits.empty()) {
            std::sort(hits.begin(), hits.end(), ray_hit_before);

            return true;
        }

        return false;
    }

    if (any_hit) {
        hits.push_back(best_hit);

        return true;
    }

    return false;
}

std::vector<Point> Intersection::ray_mesh(const Line& line, const Mesh& mesh, double epsilon, bool find_all) {

    (void)epsilon;

    Point origin = line.start();
    Vector direction = line.to_vector();

    std::vector<RayHit> hits;
    std::vector<Point> result;

    if (ray_mesh(origin, direction, mesh, hits, find_all)) {
        result.reserve(hits.size());

        for (const RayHit& hit : hits)
            result.push_back(hit.point);
    }

    return result;
}

std::vector<Point> Intersection::ray_mesh_bvh(const Line& line, const Mesh& mesh, double epsilon, bool find_all) {

    (void)epsilon;

    Point origin = line.start();
    Vector direction = line.to_vector();

    std::vector<RayHit> hits;
    std::vector<Point> result;

    if (ray_mesh_bvh(origin, direction, mesh, hits, find_all)) {
        result.reserve(hits.size());

        for (const RayHit& hit : hits)
            result.push_back(hit.point);
    }

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// NURBS curve helpers
// ═══════════════════════════════════════════════════════════════════════════

namespace {

/// Signed distance of a point to the plane.
double curve_signed_distance_to_plane(const Point& pt, const Plane& plane) {

    Vector v = pt - plane.origin();

    return v.dot(plane.z_axis());
}

/// Bisect the plane crossing between t0 and t1 down to tolerance.
bool curve_find_root_bisection(
    const NurbsCurve& curve,
    const Plane& plane,
    double t0,
    double t1,
    double tolerance,
    double& t_result
) {

    const int max_iterations = 50;
    double d0 = curve_signed_distance_to_plane(curve.point_at(t0), plane);
    double d1 = curve_signed_distance_to_plane(curve.point_at(t1), plane);

    if (d0 * d1 > 0)
        return false;

    for (int iter = 0; iter < max_iterations; iter++) {
        double t_mid = (t0 + t1) * 0.5;
        double d_mid = curve_signed_distance_to_plane(curve.point_at(t_mid), plane);

        if (std::abs(d_mid) < tolerance || (t1 - t0) < tolerance) {
            t_result = t_mid;

            return true;
        }

        if (d0 * d_mid < 0) {
            t1 = t_mid;
            d1 = d_mid;
        } else {
            t0 = t_mid;
            d0 = d_mid;
        }
    }

    t_result = (t0 + t1) * 0.5;

    return std::abs(curve_signed_distance_to_plane(curve.point_at(t_result), plane)) < tolerance * 10.0;
}

/// Polish a plane crossing parameter with Newton steps.
bool curve_refine_intersection_newton(const NurbsCurve& curve, const Plane& plane, double& t, double tolerance) {

    const int max_iterations = 10;
    const double step_tolerance = tolerance * 0.01;

    for (int iter = 0; iter < max_iterations; iter++) {
        Point pt = curve.point_at(t);
        Vector tangent = curve.tangent_at(t);

        double f = curve_signed_distance_to_plane(pt, plane);
        double df = tangent.dot(plane.z_axis());

        if (std::abs(f) < tolerance)
            return true;

        if (std::abs(df) < 1e-12)
            return false;

        double dt = -f / df;

        if (std::abs(dt) < step_tolerance)
            return true;

        t += dt;

        const std::pair<double, double> domain = curve.domain();
        const double t0 = domain.first;
        const double t1 = domain.second;

        if (t < t0)
            t = t0;

        if (t > t1)
            t = t1;
    }

    return std::abs(curve_signed_distance_to_plane(curve.point_at(t), plane)) < tolerance * 2.0;
}

/// Bezier-clipping recursion of the curve-plane distance on [ta, tb].
void curve_plane_clip(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance,
    double ta,
    double tb,
    int depth,
    std::vector<double>& results
) {

    if (depth > 50) {
        double tm = (ta + tb) * 0.5;
        Point pm = curve.point_at(tm);
        double dist = curve_signed_distance_to_plane(pm, plane);

        if (std::abs(dist) < tolerance)
            results.push_back(tm);

        return;
    }

    if (std::abs(tb - ta) < tolerance * 0.01) {
        double tm = (ta + tb) * 0.5;
        Point pm = curve.point_at(tm);
        double dist = curve_signed_distance_to_plane(pm, plane);

        if (std::abs(dist) < tolerance) {
            double t = tm;

            for (int iter = 0; iter < 10; iter++) {
                Point pt = curve.point_at(t);
                Vector tangent = curve.tangent_at(t);

                double f = curve_signed_distance_to_plane(pt, plane);
                double df = tangent.dot(plane.z_axis());

                if (std::abs(df) < 1e-12)
                    break;

                double dt = -f / df;
                t += dt;

                if (std::abs(dt) < tolerance * 0.01)
                    break;

                if (t < ta || t > tb) {
                    t = tm;
                    break;
                }
            }

            Point pt_final = curve.point_at(t);

            if (std::abs(curve_signed_distance_to_plane(pt_final, plane)) < tolerance && t >= ta && t <= tb)
                results.push_back(t);
        }

        return;
    }

    int num_samples = std::min(curve.order() + 1, 10);
    std::vector<double> distances;
    std::vector<double> params;

    double dt = (tb - ta) / (num_samples - 1);

    for (int i = 0; i < num_samples; i++) {
        double t = ta + i * dt;
        Point p = curve.point_at(t);
        distances.push_back(curve_signed_distance_to_plane(p, plane));
        params.push_back(t);
    }

    double d_min = *std::min_element(distances.begin(), distances.end());
    double d_max = *std::max_element(distances.begin(), distances.end());

    if (d_min > tolerance || d_max < -tolerance)
        return;

    double t_min = ta;
    double t_max = tb;

    for (size_t i = 0; i < distances.size() - 1; i++) {
        if (distances[i] * distances[i + 1] < 0) {
            double d0 = distances[i];
            double d1 = distances[i + 1];
            double t_clip = params[i] - d0 * (params[i + 1] - params[i]) / (d1 - d0);

            if (d0 > 0)
                t_max = std::min(t_max, t_clip + (tb - ta) * 0.1);
            else
                t_min = std::max(t_min, t_clip - (tb - ta) * 0.1);
        }
    }

    if (t_min >= t_max) {
        t_min = ta;
        t_max = tb;
    }

    t_min = std::max(ta, t_min);
    t_max = std::min(tb, t_max);

    double reduction = (t_max - t_min) / (tb - ta);

    if (reduction > 0.8 || (t_max - t_min) < tolerance * 0.1) {
        double tm = (ta + tb) * 0.5;
        curve_plane_clip(curve, plane, tolerance, ta, tm, depth + 1, results);
        curve_plane_clip(curve, plane, tolerance, tm, tb, depth + 1, results);
    } else {
        curve_plane_clip(curve, plane, tolerance, t_min, t_max, depth + 1, results);
    }
}

/// Hodograph subdivision of one span with Newton polishing of the crossings.
void curve_plane_subdivide_algebraic(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance,
    double a,
    double b,
    int depth,
    std::vector<double>& results
) {

    if (depth > 30)
        return;

    Point p_a = curve.point_at(a);
    Point p_b = curve.point_at(b);

    Vector normal = plane.z_axis();
    double f_a = normal.dot(p_a - plane.origin());
    double f_b = normal.dot(p_b - plane.origin());

    if (f_a * f_b > 0)
        return;

    double mid_t = (a + b) * 0.5;
    Point p_mid = curve.point_at(mid_t);

    Vector line_dir = p_b - p_a;
    double line_len = line_dir.magnitude();

    if (line_len > 1e-14)
        line_dir = line_dir / line_len;

    double deviation = std::abs((p_mid - p_a).cross(line_dir).magnitude());

    if (deviation < tolerance * 10.0 || (b - a) < tolerance * 10.0) {
        double t = mid_t;
        bool converged = false;

        for (int iter = 0; iter < 10; iter++) {
            Point p = curve.point_at(t);
            double f = normal.dot(p - plane.origin());

            if (std::abs(f) < tolerance) {
                converged = true;
                break;
            }

            Vector tangent = curve.tangent_at(t);
            double df = normal.dot(tangent);

            if (std::abs(df) < 1e-14) {
                t = (a + b) * 0.5;
                break;
            }

            double t_new = t - f / df;

            if (t_new < a || t_new > b)
                t_new = (a + b) * 0.5;

            if (std::abs(t_new - t) < tolerance) {
                t = t_new;
                converged = true;
                break;
            }

            t = t_new;
        }

        if (converged && t >= a && t <= b) {
            bool is_duplicate = false;

            for (double existing : results) {
                if (std::abs(existing - t) < tolerance * 10.0) {
                    is_duplicate = true;
                    break;
                }
            }

            if (!is_duplicate)
                results.push_back(t);
        }
    } else {
        curve_plane_subdivide_algebraic(curve, plane, tolerance, a, mid_t, depth + 1, results);
        curve_plane_subdivide_algebraic(curve, plane, tolerance, mid_t, b, depth + 1, results);
    }
}

/// True when the chord of [a, b] deviates less than ten tolerances from the curve.
bool curve_nearly_linear(const NurbsCurve& curve, double tolerance, double a, double b) {

    Point p_a = curve.point_at(a);
    Point p_b = curve.point_at(b);
    Point p_mid = curve.point_at((a + b) * 0.5);

    Vector ab = p_b - p_a;
    double line_length = ab.magnitude();

    if (line_length < 1e-14)
        return true;

    Vector am = p_mid - p_a;
    double cross_mag = ab.cross(am).magnitude();
    double deviation = cross_mag / line_length;

    return deviation < tolerance * 10.0;
}

/// Span subdivision to nearly linear pieces with Newton polishing of the crossings.
void curve_plane_subdivide_production(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance,
    double a,
    double b,
    int depth,
    std::vector<double>& results
) {

    if (depth > 30)
        return;

    Point p_a = curve.point_at(a);
    Point p_b = curve.point_at(b);

    Vector normal = plane.z_axis();
    double f_a = normal.dot(p_a - plane.origin());
    double f_b = normal.dot(p_b - plane.origin());

    if (f_a * f_b > 0)
        return;

    if (curve_nearly_linear(curve, tolerance, a, b) || (b - a) < tolerance * 10.0) {
        double t = (a + b) * 0.5;
        bool converged = false;

        for (int iter = 0; iter < 10; iter++) {
            Point p = curve.point_at(t);
            double f = normal.dot(p - plane.origin());

            if (std::abs(f) < tolerance) {
                converged = true;
                break;
            }

            Vector tangent = curve.tangent_at(t);
            double df = normal.dot(tangent);

            if (std::abs(df) < 1e-14) {
                if (f * f_a < 0) {
                    b = t;
                    f_b = f;
                } else {
                    a = t;
                    f_a = f;
                }

                t = (a + b) * 0.5;
                continue;
            }

            double t_new = t - f / df;

            if (t_new < a || t_new > b)
                t_new = (a + b) * 0.5;

            if (std::abs(t_new - t) < tolerance) {
                t = t_new;
                converged = true;
                break;
            }

            t = t_new;
        }

        if (converged && t >= a && t <= b) {
            bool is_duplicate = false;

            for (double existing : results) {
                if (std::abs(existing - t) < tolerance * 10.0) {
                    is_duplicate = true;
                    break;
                }
            }

            if (!is_duplicate)
                results.push_back(t);
        }
    } else {
        double mid = (a + b) * 0.5;
        curve_plane_subdivide_production(curve, plane, tolerance, a, mid, depth + 1, results);
        curve_plane_subdivide_production(curve, plane, tolerance, mid, b, depth + 1, results);
    }
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// NURBS curves
// ═══════════════════════════════════════════════════════════════════════════

std::vector<double> Intersection::curve_plane(const NurbsCurve& curve, const Plane& plane, double tolerance) {

    std::vector<double> intersections;

    if (!curve.is_valid())
        return intersections;

    if (tolerance <= 0.0)
        tolerance = Tolerance::ZERO_TOLERANCE;

    const std::pair<double, double> domain = curve.domain();
    const double t_start = domain.first;
    const double t_end = domain.second;

    std::vector<double> span_params = curve.get_span_vector();

    for (size_t i = 0; i < span_params.size() - 1; i++) {
        double t0 = span_params[i];
        double t1 = span_params[i + 1];

        if (std::abs(t1 - t0) < tolerance)
            continue;

        double d0 = curve_signed_distance_to_plane(curve.point_at(t0), plane);
        double d1 = curve_signed_distance_to_plane(curve.point_at(t1), plane);

        if (d0 * d1 < 0) {
            double t_intersection;

            if (curve_find_root_bisection(curve, plane, t0, t1, tolerance, t_intersection)) {
                curve_refine_intersection_newton(curve, plane, t_intersection, tolerance);
                intersections.push_back(t_intersection);
            }
        } else if (std::abs(d0) < tolerance) {
            bool add = true;

            if (!intersections.empty() && std::abs(intersections.back() - t0) < tolerance)
                add = false;

            if (add)
                intersections.push_back(t0);
        }
    }

    double d_end = curve_signed_distance_to_plane(curve.point_at(t_end), plane);

    if (std::abs(d_end) < tolerance) {
        bool add = true;

        if (!intersections.empty() && std::abs(intersections.back() - t_end) < tolerance)
            add = false;

        if (add)
            intersections.push_back(t_end);
    }

    if (curve.degree() > 3 && intersections.size() < static_cast<size_t>(curve.degree())) {
        int num_samples = curve.degree() * 4;
        double dt = (t_end - t_start) / num_samples;

        for (int i = 0; i < num_samples; i++) {
            double t0 = t_start + i * dt;
            double t1 = t_start + (i + 1) * dt;

            double d0 = curve_signed_distance_to_plane(curve.point_at(t0), plane);
            double d1 = curve_signed_distance_to_plane(curve.point_at(t1), plane);

            if (d0 * d1 < 0) {
                double t_intersection;

                if (curve_find_root_bisection(curve, plane, t0, t1, tolerance, t_intersection)) {
                    bool is_new = true;

                    for (double existing : intersections) {
                        if (std::abs(existing - t_intersection) < tolerance * 2.0) {
                            is_new = false;
                            break;
                        }
                    }

                    if (is_new) {
                        curve_refine_intersection_newton(curve, plane, t_intersection, tolerance);
                        intersections.push_back(t_intersection);
                    }
                }
            }
        }
    }

    std::sort(intersections.begin(), intersections.end());

    intersections.erase(
        std::unique(
            intersections.begin(),
            intersections.end(),
            [tolerance](double a, double b) {
                return std::abs(a - b) < tolerance * 2.0;
            }
        ),
        intersections.end()
    );

    return intersections;
}

std::vector<Point> Intersection::curve_plane_points(const NurbsCurve& curve, const Plane& plane, double tolerance) {

    std::vector<double> params = curve_plane(curve, plane, tolerance);
    std::vector<Point> points;
    points.reserve(params.size());

    for (double t : params)
        points.push_back(curve.point_at(t));

    return points;
}

std::vector<double> Intersection::curve_plane_bezier_clipping(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance
) {

    std::vector<double> results;

    if (!curve.is_valid())
        return results;

    if (tolerance <= 0.0)
        tolerance = Tolerance::ZERO_TOLERANCE;

    const std::pair<double, double> domain = curve.domain();
    const double t0 = domain.first;
    const double t1 = domain.second;

    curve_plane_clip(curve, plane, tolerance, t0, t1, 0, results);

    std::sort(results.begin(), results.end());

    auto last = std::unique(results.begin(), results.end(), [tolerance](double a, double b) {
        return std::abs(a - b) < tolerance * 2.0;
    });

    results.erase(last, results.end());

    return results;
}

std::vector<double> Intersection::curve_plane_algebraic(const NurbsCurve& curve, const Plane& plane, double tolerance) {

    if (!curve.is_valid())
        return {};

    std::vector<double> results;

    std::vector<double> spans = curve.get_span_vector();

    if (spans.size() < 2)
        return {};

    for (size_t i = 0; i < spans.size() - 1; i++) {
        double span_t0 = spans[i];
        double span_t1 = spans[i + 1];

        if (std::abs(span_t1 - span_t0) < tolerance)
            continue;

        curve_plane_subdivide_algebraic(curve, plane, tolerance, span_t0, span_t1, 0, results);
    }

    std::sort(results.begin(), results.end());

    results.erase(
        std::unique(
            results.begin(),
            results.end(),
            [tolerance](double a, double b) {
                return std::abs(a - b) < tolerance * 10.0;
            }
        ),
        results.end()
    );

    return results;
}

std::vector<double> Intersection::curve_plane_production(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance
) {

    if (!curve.is_valid())
        return {};

    std::vector<double> results;

    std::vector<double> spans = curve.get_span_vector();

    if (spans.size() < 2)
        return {};

    for (size_t i = 0; i < spans.size() - 1; i++) {
        double span_t0 = spans[i];
        double span_t1 = spans[i + 1];

        if (std::abs(span_t1 - span_t0) < tolerance)
            continue;

        curve_plane_subdivide_production(curve, plane, tolerance, span_t0, span_t1, 0, results);
    }

    std::sort(results.begin(), results.end());

    results.erase(
        std::unique(
            results.begin(),
            results.end(),
            [tolerance](double a, double b) {
                return std::abs(a - b) < tolerance * 10.0;
            }
        ),
        results.end()
    );

    return results;
}

std::pair<double, double> Intersection::curve_closest_point(
    const NurbsCurve& curve,
    const Point& test_point,
    double t0,
    double t1
) {
    return Closest::curve_point(curve, test_point, t0, t1);
}

// ═══════════════════════════════════════════════════════════════════════════
// NURBS surface helpers
// ═══════════════════════════════════════════════════════════════════════════

namespace {

/// One traced surface-plane curve in parameter space.
struct SurfacePlaneTrace {
    std::vector<std::pair<double, double>> uv_trace; // Traced (u, v) samples.
    std::vector<std::pair<double, double>> uv_unwrapped; // Samples with seam wraps undone.
    bool is_loop; // Whether the trace closes on itself.
};

/// All traces of one surface-plane section with the scales used.
struct SurfacePlaneTraceResult {
    std::vector<SurfacePlaneTrace> traces; // Traced curves.
    double step; // uv step used.
    double uv_to_3d; // Largest uv-to-3D scale seen.
    double uv_to_3d_min; // Smallest uv-to-3D scale seen.
};

/// Seed and trace surface/plane intersection curves in UV space.
SurfacePlaneTraceResult surface_plane_traces(const NurbsSurface& surface, const Plane& plane, double tolerance) {

    const std::pair<double, double> domain_u = surface.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = surface.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    double range_u = u1 - u0;
    double range_v = v1 - v0;
    bool closed_u = surface.is_closed(0);
    bool closed_v = surface.is_closed(1);

    auto wrap_u = [&](double u) -> double {
        if (closed_u) {
            double t = std::fmod(u - u0, range_u);

            if (t < 0)
                t += range_u;

            return u0 + t;
        }

        return std::max(u0, std::min(u, u1));
    };

    auto wrap_v = [&](double v) -> double {
        if (closed_v) {
            double t = std::fmod(v - v0, range_v);

            if (t < 0)
                t += range_v;

            return v0 + t;
        }

        return std::max(v0, std::min(v, v1));
    };

    Vector pn = plane.z_axis();
    Point p0 = plane.origin();

    auto g = [&](double u, double v) -> double {
        Point p = surface.point_at(wrap_u(u), wrap_v(v));

        return (p[0] - p0[0]) * pn[0] + (p[1] - p0[1]) * pn[1] + (p[2] - p0[2]) * pn[2];
    };

    auto g_and_grad = [&](double u, double v, double& val, double& gu, double& gv) {
        const std::vector<Vector> derivs = surface.evaluate(wrap_u(u), wrap_v(v), 1);
        const Vector& S = derivs[0];
        const Vector& Su = derivs[2];
        const Vector& Sv = derivs[1];
        val = (S[0] - p0[0]) * pn[0] + (S[1] - p0[1]) * pn[1] + (S[2] - p0[2]) * pn[2];
        gu = Su[0] * pn[0] + Su[1] * pn[1] + Su[2] * pn[2];
        gv = Sv[0] * pn[0] + Sv[1] * pn[1] + Sv[2] * pn[2];
    };

    auto newton_correct = [&](double& u, double& v) -> bool {
        for (int iter = 0; iter < 10; iter++) {
            double val;
            double gu;
            double gv;
            g_and_grad(u, v, val, gu, gv);

            if (std::abs(val) < tolerance)
                return true;

            double mag2 = gu * gu + gv * gv;

            if (mag2 < 1e-28)
                return false;

            u -= val * gu / mag2;
            v -= val * gv / mag2;
            u = wrap_u(u);
            v = wrap_v(v);
        }

        return std::abs(g(u, v)) < tolerance * 10.0;
    };

    const std::vector<double> spans_u = surface.get_span_vector(0);
    const std::vector<double> spans_v = surface.get_span_vector(1);
    int nu = std::max((int)spans_u.size() - 1, 1) * 4;
    int nv = std::max((int)spans_v.size() - 1, 1) * 4;
    double du = range_u / nu;
    double dv = range_v / nv;

    double mu = (u0 + u1) * 0.5;
    double mv = (v0 + v1) * 0.5;
    Point pmid = surface.point_at(mu, mv);
    double uv_to_3d_u = pmid.distance(surface.point_at(wrap_u(mu + du), mv)) / du;
    double uv_to_3d_v = pmid.distance(surface.point_at(mu, wrap_v(mv + dv))) / dv;
    double uv_to_3d = std::max(uv_to_3d_u, uv_to_3d_v);
    double uv_to_3d_min = std::min(uv_to_3d_u, uv_to_3d_v);

    if (uv_to_3d < 1e-10)
        uv_to_3d = 1.0;

    if (uv_to_3d_min < 1e-10)
        uv_to_3d_min = 1.0;

    int cols = nv + 1;
    std::vector<double> dist((nu + 1) * cols);

    for (int i = 0; i <= nu; i++) {
        double u = u0 + du * i;

        for (int j = 0; j <= nv; j++) {
            double v = v0 + dv * j;
            double d = g(u, v);

            if (d == 0.0)
                d = -1e-14;

            dist[i * cols + j] = d;
        }
    }

    {
        double gmax = 0.0;

        for (double d : dist)
            gmax = std::max(gmax, std::abs(d));

        if (gmax < std::max(tolerance, 1e-9) * 10.0)
            return {{}, std::min(du, dv) * 0.25, uv_to_3d, uv_to_3d_min};
    }

    struct Seed {
        double u;
        double v;
        bool used;
    };

    std::vector<Seed> seeds;

    int h_jmax = closed_v ? nv - 1 : nv;

    for (int i = 0; i < nu; i++) {
        for (int j = 0; j <= h_jmax; j++) {
            double d0 = dist[i * cols + j];
            double d1 = dist[(i + 1) * cols + j];

            if (d0 * d1 < 0) {
                double t = d0 / (d0 - d1);
                double su = u0 + du * (i + t);
                double sv = v0 + dv * j;

                if (newton_correct(su, sv))
                    seeds.push_back({su, sv, false});
            }
        }
    }

    int v_imax = closed_u ? nu - 1 : nu;

    for (int i = 0; i <= v_imax; i++) {
        for (int j = 0; j < nv; j++) {
            double d0 = dist[i * cols + j];
            double d1 = dist[i * cols + j + 1];

            if (d0 * d1 < 0) {
                double t = d0 / (d0 - d1);
                double su = u0 + du * i;
                double sv = v0 + dv * (j + t);

                if (newton_correct(su, sv))
                    seeds.push_back({su, sv, false});
            }
        }
    }

    double seed_tol_3d = std::max(du, dv) * uv_to_3d;

    for (size_t i = 0; i < seeds.size(); i++) {
        if (seeds[i].used)
            continue;

        Point pi = surface.point_at(seeds[i].u, seeds[i].v);

        for (size_t j = i + 1; j < seeds.size(); j++) {
            if (seeds[j].used)
                continue;

            if (pi.distance(surface.point_at(seeds[j].u, seeds[j].v)) < seed_tol_3d)
                seeds[j].used = true;
        }
    }

    double step = std::min(du, dv) * 0.25;
    int max_steps = nu * nv * 32;
    double close_tol_3d = step * 4.0 * uv_to_3d_min;
    double consume_tol_3d = step * uv_to_3d * 2.0;

    std::vector<SurfacePlaneTrace> traces;

    for (Seed& seed : seeds) {
        if (seed.used)
            continue;

        seed.used = true;

        auto tangent_at_uv = [&](double u, double v, int dir, double& tu, double& tv) -> bool {
            double val;
            double gu;
            double gv;
            g_and_grad(u, v, val, gu, gv);
            double mag = std::hypot(gu, gv);

            if (mag < 1e-14)
                return false;

            tu = -gv / mag * dir;
            tv = gu / mag * dir;

            return true;
        };

        auto trace_dir = [&](double su, double sv, int dir, std::vector<std::pair<double, double>>& out) -> bool {
            double u = su;
            double v = sv;
            double prev_tu = 0;
            double prev_tv = 0;
            Point p_start = surface.point_at(su, sv);
            Point p_prev = p_start;
            double dist_traveled = 0;

            for (int s = 0; s < max_steps; s++) {
                double tu;
                double tv;

                if (!tangent_at_uv(u, v, dir, tu, tv)) {
                    if (std::hypot(prev_tu, prev_tv) < 1e-14)
                        break;

                    tu = prev_tu;
                    tv = prev_tv;
                }

                double local_step = step;

                if (std::hypot(prev_tu, prev_tv) > 1e-14) {
                    double dot = tu * prev_tu + tv * prev_tv;
                    dot = std::max(-1.0, std::min(1.0, dot));

                    if (dot < 0.95)
                        local_step = step * 0.25;
                    else if (dot < 0.985)
                        local_step = step * 0.5;
                }

                double u_mid = u + local_step * 0.5 * tu;
                double v_mid = v + local_step * 0.5 * tv;
                double tu2;
                double tv2;

                if (tangent_at_uv(u_mid, v_mid, dir, tu2, tv2)) {
                    tu = tu2;
                    tv = tv2;
                }

                prev_tu = tu;
                prev_tv = tv;

                double un = u + local_step * tu;
                double vn = v + local_step * tv;

                bool hit_boundary = false;

                if ((!closed_u && (un < u0 || un > u1)) || (!closed_v && (vn < v0 || vn > v1))) {
                    double tc = 1.0;

                    if (!closed_u && tu > 0 && un > u1)
                        tc = std::min(tc, (u1 - u) / (local_step * tu));

                    if (!closed_u && tu < 0 && un < u0)
                        tc = std::min(tc, (u0 - u) / (local_step * tu));

                    if (!closed_v && tv > 0 && vn > v1)
                        tc = std::min(tc, (v1 - v) / (local_step * tv));

                    if (!closed_v && tv < 0 && vn < v0)
                        tc = std::min(tc, (v0 - v) / (local_step * tv));

                    un = u + tc * local_step * tu;
                    vn = v + tc * local_step * tv;
                    hit_boundary = true;
                }

                un = wrap_u(un);
                vn = wrap_v(vn);

                if (!newton_correct(un, vn)) {
                    bool ok_retry = false;
                    double ls = local_step;

                    for (int rh = 0; rh < 4 && !ok_retry; ++rh) {
                        ls *= 0.5;
                        un = wrap_u(u + ls * tu);
                        vn = wrap_v(v + ls * tv);

                        if (newton_correct(un, vn))
                            ok_retry = true;
                    }

                    if (!ok_retry)
                        break;
                }

                Point p_cur = surface.point_at(un, vn);
                dist_traveled += p_prev.distance(p_cur);

                if (dist_traveled > close_tol_3d * 3.0 && p_start.distance(p_cur) < close_tol_3d) {
                    out.push_back({un, vn});

                    return true;
                }

                out.push_back({un, vn});
                u = un;
                v = vn;
                p_prev = p_cur;

                if (hit_boundary)
                    break;

                for (Seed& other : seeds) {
                    if (!other.used) {
                        if (p_cur.distance(surface.point_at(other.u, other.v)) < consume_tol_3d)
                            other.used = true;
                    }
                }
            }

            return false;
        };

        std::vector<std::pair<double, double>> fwd, bwd;
        bool fwd_closed = trace_dir(seed.u, seed.v, +1, fwd);

        if (!fwd_closed)
            trace_dir(seed.u, seed.v, -1, bwd);

        std::vector<std::pair<double, double>> uv_trace;
        uv_trace.reserve(bwd.size() + 1 + fwd.size());

        for (int i = (int)bwd.size() - 1; i >= 0; i--)
            uv_trace.push_back(bwd[i]);

        uv_trace.push_back({seed.u, seed.v});

        for (std::pair<double, double>& p : fwd)
            uv_trace.push_back(p);

        if (uv_trace.size() < 4)
            continue;

        Point p_first = surface.point_at(uv_trace.front().first, uv_trace.front().second);
        Point p_last = surface.point_at(uv_trace.back().first, uv_trace.back().second);
        bool is_loop = fwd_closed || (uv_trace.size() >= 6 && p_first.distance(p_last) < close_tol_3d);

        if (is_loop)
            uv_trace.pop_back();

        if (uv_trace.size() < 4)
            continue;

        std::vector<std::pair<double, double>> uv_unwrapped = uv_trace;

        for (size_t i = 1; i < uv_unwrapped.size(); i++) {
            double du_jump = uv_unwrapped[i].first - uv_unwrapped[i - 1].first;
            double dv_jump = uv_unwrapped[i].second - uv_unwrapped[i - 1].second;

            if (closed_u) {
                if (du_jump > range_u * 0.5)
                    uv_unwrapped[i].first -= range_u;
                else if (du_jump < -range_u * 0.5)
                    uv_unwrapped[i].first += range_u;
            }

            if (closed_v) {
                if (dv_jump > range_v * 0.5)
                    uv_unwrapped[i].second -= range_v;
                else if (dv_jump < -range_v * 0.5)
                    uv_unwrapped[i].second += range_v;
            }
        }

        traces.push_back({std::move(uv_trace), std::move(uv_unwrapped), is_loop});
    }

    double join_tol = std::max(du, dv) * uv_to_3d * 1.5;

    auto p3 = [&](const std::pair<double, double>& q) {
        return surface.point_at(q.first, q.second);
    };

    for (size_t i = 0; i < traces.size(); ++i) {
        if (traces[i].uv_trace.empty() || traces[i].is_loop)
            continue;

        for (size_t j = 0; j < traces.size(); ++j) {
            if (i == j || traces[j].uv_trace.empty())
                continue;

            if (traces[j].uv_trace.size() < traces[i].uv_trace.size())
                continue;

            bool covered = true;

            for (size_t k = 0; k < traces[i].uv_trace.size() && covered;
                 k += std::max<size_t>(1, traces[i].uv_trace.size() / 8)) {

                Point q = p3(traces[i].uv_trace[k]);
                double best = 1e300;

                for (std::pair<double, double>& r : traces[j].uv_trace)
                    best = std::min(best, q.distance(p3(r)));

                if (best > join_tol)
                    covered = false;
            }

            if (covered) {
                traces[i].uv_trace.clear();
                break;
            }
        }
    }

    bool joined = true;

    for (size_t pass = 0; pass < traces.size() && joined; pass++) {
        joined = false;

        for (size_t i = 0; i < traces.size() && !joined; ++i) {
            if (traces[i].uv_trace.size() < 2 || traces[i].is_loop)
                continue;

            Point ie = p3(traces[i].uv_trace.back());

            for (size_t j = 0; j < traces.size() && !joined; ++j) {
                if (i == j || traces[j].uv_trace.size() < 2 || traces[j].is_loop)
                    continue;

                Point ja = p3(traces[j].uv_trace.front());
                Point jb = p3(traces[j].uv_trace.back());
                bool fwd2 = ie.distance(ja) < join_tol;
                bool rev2 = ie.distance(jb) < join_tol;

                if (!fwd2 && !rev2)
                    continue;

                std::vector<std::pair<double, double>> add = traces[j].uv_trace;

                if (rev2)
                    std::reverse(add.begin(), add.end());

                traces[i].uv_trace.insert(traces[i].uv_trace.end(), add.begin(), add.end());
                traces[j].uv_trace.clear();

                if (p3(traces[i].uv_trace.front()).distance(p3(traces[i].uv_trace.back())) < join_tol) {
                    traces[i].is_loop = true;
                    traces[i].uv_trace.pop_back();
                }

                traces[i].uv_unwrapped = traces[i].uv_trace;

                for (size_t k = 1; k < traces[i].uv_unwrapped.size(); ++k) {
                    double dj = traces[i].uv_unwrapped[k].first - traces[i].uv_unwrapped[k - 1].first;
                    double dvj = traces[i].uv_unwrapped[k].second - traces[i].uv_unwrapped[k - 1].second;

                    if (closed_u) {
                        if (dj > range_u * 0.5)
                            traces[i].uv_unwrapped[k].first -= range_u;
                        else if (dj < -range_u * 0.5)
                            traces[i].uv_unwrapped[k].first += range_u;
                    }

                    if (closed_v) {
                        if (dvj > range_v * 0.5)
                            traces[i].uv_unwrapped[k].second -= range_v;
                        else if (dvj < -range_v * 0.5)
                            traces[i].uv_unwrapped[k].second += range_v;
                    }
                }

                joined = true;
            }
        }
    }

    traces.erase(
        std::remove_if(
            traces.begin(),
            traces.end(),
            [](const SurfacePlaneTrace& t) {
                return t.uv_trace.size() < 4;
            }
        ),
        traces.end()
    );

    for (SurfacePlaneTrace& t : traces) {
        if (t.is_loop || t.uv_trace.size() < 6)
            continue;

        if (p3(t.uv_trace.front()).distance(p3(t.uv_trace.back())) < join_tol) {
            t.is_loop = true;
            t.uv_trace.pop_back();
            t.uv_unwrapped.pop_back();
        }
    }

    for (SurfacePlaneTrace& t : traces) {
        if (t.is_loop || t.uv_trace.empty())
            continue;

        for (int endk = 0; endk < 2; ++endk) {
            std::pair<double, double>& q = endk ? t.uv_trace.back() : t.uv_trace.front();
            std::pair<double, double>& qu = endk ? t.uv_unwrapped.back() : t.uv_unwrapped.front();

            if (!closed_u) {
                if (std::abs(q.first - u0) < du) {
                    q.first = u0;
                    qu.first = u0;
                }

                if (std::abs(q.first - u1) < du) {
                    q.first = u1;
                    qu.first = u1;
                }
            } else {
                if (q.first - u0 < du)
                    q.first = u0;
                else if (u1 - q.first < du)
                    q.first = u1;
            }

            if (!closed_v) {
                if (std::abs(q.second - v0) < dv) {
                    q.second = v0;
                    qu.second = v0;
                }

                if (std::abs(q.second - v1) < dv) {
                    q.second = v1;
                    qu.second = v1;
                }
            } else {
                if (q.second - v0 < dv)
                    q.second = v0;
                else if (v1 - q.second < dv)
                    q.second = v1;
            }
        }
    }

    return {std::move(traces), step, uv_to_3d, uv_to_3d_min};
}

/// Fit a 3D plane-constrained NurbsCurve to traced intersection points.
NurbsCurve surface_plane_fit_3d(
    const std::vector<Point>& all_pts,
    bool is_loop,
    const Plane& plane,
    double step,
    double uv_to_3d,
    double uv_to_3d_min,
    bool allow_conics = true
) {

    NurbsCurve crv;

    if (allow_conics && is_loop && all_pts.size() >= 6) {
        Vector ax = plane.x_axis();
        Vector ay = plane.y_axis();
        Point po = plane.origin();

        auto to2d = [&](const Point& p) -> std::pair<double, double> {
            double dx = p[0] - po[0];
            double dy = p[1] - po[1];
            double dz = p[2] - po[2];

            return {dx * ax[0] + dy * ax[1] + dz * ax[2], dx * ay[0] + dy * ay[1] + dz * ay[2]};
        };

        int n = (int)all_pts.size();
        double x1;
        double y1;
        std::tie(x1, y1) = to2d(all_pts[0]);
        double x2;
        double y2;
        std::tie(x2, y2) = to2d(all_pts[n / 3]);
        double x3;
        double y3;
        std::tie(x3, y3) = to2d(all_pts[2 * n / 3]);

        double ax_ = x2 - x1;
        double ay_ = y2 - y1;
        double bx_ = x3 - x1;
        double by_ = y3 - y1;
        double D = 2.0 * (ax_ * by_ - ay_ * bx_);

        if (std::abs(D) > 1e-10) {
            double a2 = ax_ * ax_ + ay_ * ay_;
            double b2 = bx_ * bx_ + by_ * by_;
            double ccx = x1 + (by_ * a2 - ay_ * b2) / D;
            double ccy = y1 + (ax_ * b2 - bx_ * a2) / D;
            double radius = std::hypot(x1 - ccx, y1 - ccy);

            double max_dev = 0;

            for (const Point& p : all_pts) {
                double px;
                double py;
                std::tie(px, py) = to2d(p);
                max_dev = std::max(max_dev, std::abs(std::hypot(px - ccx, py - ccy) - radius));
            }

            double circle_tol = std::max(radius * 1e-5, 1e-6);

            if (radius > 1e-10 && max_dev < circle_tol) {
                double cx3d = po[0] + ccx * ax[0] + ccy * ay[0];
                double cy3d = po[1] + ccx * ax[1] + ccy * ay[1];
                double cz3d = po[2] + ccx * ax[2] + ccy * ay[2];

                const double w = std::sqrt(2.0) / 2.0;
                double cx_[] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
                double cy_[] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
                double wts[] = {1, w, 1, w, 1, w, 1, w, 1};
                crv = NurbsCurve(3, true, 3, 9);
                double nurbsknots[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};

                for (int i = 0; i < 10; i++)
                    crv.set_nurbsknot(i, nurbsknots[i]);

                for (int i = 0; i < 9; i++) {
                    double px = cx3d + radius * (cx_[i] * ax[0] + cy_[i] * ay[0]);
                    double py = cy3d + radius * (cx_[i] * ax[1] + cy_[i] * ay[1]);
                    double pz = cz3d + radius * (cx_[i] * ax[2] + cy_[i] * ay[2]);
                    crv.set_cv_4d(i, px * wts[i], py * wts[i], pz * wts[i], wts[i]);
                }
            }
        }
    }

    if (!crv.is_valid() && allow_conics && is_loop && all_pts.size() >= 8) {
        Vector ax = plane.x_axis();
        Vector ay = plane.y_axis();
        Point po = plane.origin();

        auto to2d = [&](const Point& p) -> std::pair<double, double> {
            double dx = p[0] - po[0];
            double dy = p[1] - po[1];
            double dz = p[2] - po[2];

            return {dx * ax[0] + dy * ax[1] + dz * ax[2], dx * ay[0] + dy * ay[1] + dz * ay[2]};
        };

        int n = (int)all_pts.size();
        double AtA[5][5] = {}, Atb[5] = {};

        for (int i = 0; i < n; i++) {
            double x;
            double y;
            std::tie(x, y) = to2d(all_pts[i]);
            double row[5] = {x * x, x * y, y * y, x, y};

            for (int r = 0; r < 5; r++) {
                Atb[r] += row[r];

                for (int c = 0; c < 5; c++)
                    AtA[r][c] += row[r] * row[c];
            }
        }

        double M[5][6];

        for (int r = 0; r < 5; r++) {
            for (int c = 0; c < 5; c++)
                M[r][c] = AtA[r][c];

            M[r][5] = Atb[r];
        }

        bool ok = true;

        for (int col = 0; col < 5 && ok; col++) {
            int pivot = col;

            for (int r = col + 1; r < 5; r++)
                if (std::fabs(M[r][col]) > std::fabs(M[pivot][col]))
                    pivot = r;

            if (std::fabs(M[pivot][col]) < 1e-20) {
                ok = false;
                break;
            }

            if (pivot != col)
                for (int j = col; j <= 5; j++)
                    std::swap(M[col][j], M[pivot][j]);

            for (int r = col + 1; r < 5; r++) {
                double f = M[r][col] / M[col][col];

                for (int j = col; j <= 5; j++)
                    M[r][j] -= f * M[col][j];
            }
        }

        double coef[5] = {};

        if (ok) {
            for (int i = 4; i >= 0; i--) {
                double s = M[i][5];

                for (int j = i + 1; j < 5; j++)
                    s -= M[i][j] * coef[j];

                coef[i] = s / M[i][i];
            }
        }

        double A = coef[0];
        double B = coef[1];
        double C = coef[2];
        double D = coef[3];
        double E = coef[4];
        double disc = B * B - 4 * A * C;

        if (ok && disc < -1e-10 && std::fabs(A) > 1e-14) {
            double max_conic_dev = 0;

            for (const Point& p : all_pts) {
                double x;
                double y;
                std::tie(x, y) = to2d(p);
                double val = A * x * x + B * x * y + C * y * y + D * x + E * y - 1.0;
                max_conic_dev = std::max(max_conic_dev, std::fabs(val));
            }

            double scale = std::max({std::fabs(A), std::fabs(C)});
            double norm_dev = max_conic_dev / std::max(scale, 1e-10);

            if (norm_dev < 0.01) {
                double det = 4 * A * C - B * B;
                double cx = (B * E - 2 * C * D) / det;
                double cy = (B * D - 2 * A * E) / det;

                double theta = 0.5 * std::atan2(B, A - C);

                double cos_t = std::cos(theta);
                double sin_t = std::sin(theta);
                double A2 = A * cos_t * cos_t + B * cos_t * sin_t + C * sin_t * sin_t;
                double C2 = A * sin_t * sin_t - B * cos_t * sin_t + C * cos_t * cos_t;
                double f_val = A * cx * cx + B * cx * cy + C * cy * cy + D * cx + E * cy - 1.0;
                double rhs = -f_val;

                if (rhs > 1e-14 && A2 > 1e-14 && C2 > 1e-14) {
                    double semi_a = std::sqrt(rhs / A2);
                    double semi_b = std::sqrt(rhs / C2);

                    double cx3d = po[0] + cx * ax[0] + cy * ay[0];
                    double cy3d = po[1] + cx * ax[1] + cy * ay[1];
                    double cz3d = po[2] + cx * ax[2] + cy * ay[2];

                    Vector ea;
                    Vector eb;

                    for (int d = 0; d < 3; d++) {
                        ea[d] = cos_t * ax[d] + sin_t * ay[d];
                        eb[d] = -sin_t * ax[d] + cos_t * ay[d];
                    }

                    const double w = std::sqrt(2.0) / 2.0;
                    double cx_[] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
                    double cy_[] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
                    double wts[] = {1, w, 1, w, 1, w, 1, w, 1};
                    crv = NurbsCurve(3, true, 3, 9);
                    double nurbsknots[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};

                    for (int i = 0; i < 10; i++)
                        crv.set_nurbsknot(i, nurbsknots[i]);

                    for (int i = 0; i < 9; i++) {
                        double px = cx3d + semi_a * cx_[i] * ea[0] + semi_b * cy_[i] * eb[0];
                        double py = cy3d + semi_a * cx_[i] * ea[1] + semi_b * cy_[i] * eb[1];
                        double pz = cz3d + semi_a * cx_[i] * ea[2] + semi_b * cy_[i] * eb[2];
                        crv.set_cv_4d(i, px * wts[i], py * wts[i], pz * wts[i], wts[i]);
                    }

                    double max_ell_dev = 0;

                    for (const Point& p : all_pts) {
                        double px2;
                        double py2;
                        std::tie(px2, py2) = to2d(p);
                        double lx = cos_t * (px2 - cx) + sin_t * (py2 - cy);
                        double ly = -sin_t * (px2 - cx) + cos_t * (py2 - cy);
                        double ang = std::atan2(ly / semi_b, lx / semi_a);
                        double ex = cx + semi_a * std::cos(ang) * cos_t - semi_b * std::sin(ang) * sin_t;
                        double ey = cy + semi_a * std::cos(ang) * sin_t + semi_b * std::sin(ang) * cos_t;
                        double dev = std::hypot(px2 - ex, py2 - ey);
                        max_ell_dev = std::max(max_ell_dev, dev);
                    }

                    double ell_tol = std::max(std::max(semi_a, semi_b) * 1e-5, 2e-6);

                    if (max_ell_dev > ell_tol)
                        crv = NurbsCurve();
                }
            }
        }
    }

    if (!crv.is_valid()) {
        int m = (int)all_pts.size();

        if (m < 4)
            return NurbsCurve();

        Vector ax = plane.x_axis();
        Vector ay = plane.y_axis();
        Point po = plane.origin();
        std::vector<Point> pts_2d(m);

        for (int i = 0; i < m; i++) {
            double dx = all_pts[i][0] - po[0];
            double dy = all_pts[i][1] - po[1];
            double dz = all_pts[i][2] - po[2];
            double px = dx * ax[0] + dy * ax[1] + dz * ax[2];
            double py = dx * ay[0] + dy * ay[1] + dz * ay[2];
            pts_2d[i] = Point(px, py, 0);
        }

        std::vector<double> chords(m, 0.0);
        double total_len = 0;

        for (int i = 1; i < m; i++) {
            total_len += pts_2d[i].distance(pts_2d[i - 1]);
            chords[i] = total_len;
        }

        if (is_loop && m > 1)
            total_len += pts_2d[0].distance(pts_2d[m - 1]);

        if (total_len > 1e-14)
            for (int i = 1; i < m; i++)
                chords[i] /= total_len;

        double fit_tol = step * (uv_to_3d + uv_to_3d_min) * 0.5 * 5e-4;
        double total_turning = 0;

        for (int i = 1; i < m - 1; i++) {
            double dx1 = pts_2d[i][0] - pts_2d[i - 1][0];
            double dy1 = pts_2d[i][1] - pts_2d[i - 1][1];
            double dx2 = pts_2d[i + 1][0] - pts_2d[i][0];
            double dy2 = pts_2d[i + 1][1] - pts_2d[i][1];
            double l1 = std::hypot(dx1, dy1);
            double l2 = std::hypot(dx2, dy2);

            if (l1 > 1e-14 && l2 > 1e-14) {
                double c = (dx1 * dx2 + dy1 * dy2) / (l1 * l2);
                c = std::max(-1.0, std::min(1.0, c));
                total_turning += std::acos(c);
            }
        }

        int target_cvs = std::max(8, (int)(total_turning / 0.5) + 6);
        int max_cvs = std::min(m - 1, 128);
        NurbsCurve crv_2d;
        double best_dev = 1e300;

        for (int attempt = 0; attempt < 6; attempt++) {
            if (target_cvs > max_cvs)
                break;

            NurbsCurve cand = NurbsCurve::create_fitted(pts_2d, target_cvs, 3, is_loop);

            if (!cand.is_valid())
                break;

            const std::pair<double, double> domain_ft = cand.domain();
            const double ft0 = domain_ft.first;
            const double ft1 = domain_ft.second;
            double max_dev = 0;

            for (int i = 0; i < m; i++) {
                double t = ft0 + (ft1 - ft0) * chords[i];
                double w2 = (ft1 - ft0) * 2.0 / std::max(m - 1, 1);
                double lo = std::max(ft0, t - w2);
                double hi = std::min(ft1, t + w2);

                for (int it = 0; it < 20; ++it) {
                    double m1 = lo + (hi - lo) / 3;
                    double m2 = hi - (hi - lo) / 3;

                    if (cand.point_at(m1).distance(pts_2d[i]) < cand.point_at(m2).distance(pts_2d[i]))
                        hi = m2;
                    else
                        lo = m1;
                }

                max_dev = std::max(max_dev, cand.point_at(0.5 * (lo + hi)).distance(pts_2d[i]));
            }

            if (max_dev < best_dev) {
                best_dev = max_dev;
                crv_2d = cand;
            }

            if (max_dev < fit_tol)
                break;

            target_cvs = std::min(target_cvs * 2, max_cvs + 1);
        }

        if (!crv_2d.is_valid())
            crv_2d = is_loop ? NurbsCurve::create_interpolated(pts_2d, CurveNurbsKnotStyle::ChordPeriodic)
                             : NurbsCurve::create_interpolated(pts_2d);

        if (crv_2d.is_valid()) {
            crv = crv_2d;

            for (int i = 0; i < crv.cv_count(); i++) {
                Point cv2 = crv.get_cv(i);
                double cx = cv2[0];
                double cy = cv2[1];

                crv.set_cv(
                    i,
                    Point(
                        po[0] + cx * ax[0] + cy * ay[0],
                        po[1] + cx * ax[1] + cy * ay[1],
                        po[2] + cx * ax[2] + cy * ay[2]
                    )
                );
            }
        }
    }

    return crv;
}

/// Solve an n x n linear system by Gaussian elimination with partial pivoting.
bool solve_gauss(
    const std::vector<std::vector<double>>& M,
    const std::vector<double>& rhs,
    int n,
    std::vector<double>& out
) {

    std::vector<std::vector<double>> A(n, std::vector<double>(n + 1));

    for (int r = 0; r < n; r++) {
        for (int c = 0; c < n; c++)
            A[r][c] = M[r][c];

        A[r][n] = rhs[r];
    }

    for (int col = 0; col < n; col++) {
        int pivot = col;

        for (int r = col + 1; r < n; r++)
            if (std::abs(A[r][col]) > std::abs(A[pivot][col]))
                pivot = r;

        if (std::abs(A[pivot][col]) < 1e-20)
            return false;

        if (pivot != col)
            std::swap(A[col], A[pivot]);

        for (int r = col + 1; r < n; r++) {
            double f = A[r][col] / A[col][col];

            for (int j = col; j < n + 1; j++)
                A[r][j] -= f * A[col][j];
        }
    }

    out.assign(n, 0.0);

    for (int i = n - 1; i >= 0; i--) {
        double s = A[i][n];

        for (int j = i + 1; j < n; j++)
            s -= A[i][j] * out[j];

        out[i] = s / A[i][i];
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Analytic quadric surface intersection
// ═══════════════════════════════════════════════════════════════════════════

/// Dot product of two triples.
static double ssi_dot(const std::array<double, 3>& u, const std::array<double, 3>& v) {
    return u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
}

/// Dot product of two raw triples.
static double dot3(const double a[3], const double b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/// Cross product of two triples.
static std::array<double, 3> ssi_cross(const std::array<double, 3>& u, const std::array<double, 3>& v) {
    return std::array<double, 3>{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
}

/// Unit triple, or the input when degenerate.
static std::array<double, 3> ssi_unit(const std::array<double, 3>& v) {

    double l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

    return l > 1e-300 ? std::array<double, 3>{v[0] / l, v[1] / l, v[2] / l} : v;
}

/// Two unit vectors spanning the plane perpendicular to unit n.
static std::pair<std::array<double, 3>, std::array<double, 3>> ortho_basis(const std::array<double, 3>& n) {

    double ax = (std::abs(n[0]) <= std::abs(n[1]) && std::abs(n[0]) <= std::abs(n[2])) ? 1.0 : 0.0;
    double ay = (ax == 0.0 && std::abs(n[1]) <= std::abs(n[2])) ? 1.0 : 0.0;
    double az = (ax == 0.0 && ay == 0.0) ? 1.0 : 0.0;
    double ux = ay * n[2] - az * n[1];
    double uy = az * n[0] - ax * n[2];
    double uz = ax * n[1] - ay * n[0];
    double ul = std::sqrt(ux * ux + uy * uy + uz * uz);
    ux /= ul;
    uy /= ul;
    uz /= ul;
    double vx = n[1] * uz - n[2] * uy;
    double vy = n[2] * ux - n[0] * uz;
    double vz = n[0] * uy - n[1] * ux;

    return {std::array<double, 3>{ux, uy, uz}, std::array<double, 3>{vx, vy, vz}};
}

/// Exact 9-CV rational NURBS circle.
static NurbsCurve exact_circle(double cx, double cy, double cz, const std::array<double, 3>& xa, const std::array<double, 3>& ya, double radius) {

    double w = std::sqrt(2.0) / 2.0;
    double px[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    double py[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    double wts[9] = {1, w, 1, w, 1, w, 1, w, 1};
    NurbsCurve crv(3, true, 3, 9);
    double knots[10] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};

    for (int i = 0; i < 10; i++)
        crv.set_nurbsknot(i, knots[i]);

    for (int i = 0; i < 9; i++) {
        double x = cx + radius * (px[i] * xa[0] + py[i] * ya[0]);
        double y = cy + radius * (px[i] * xa[1] + py[i] * ya[1]);
        double z = cz + radius * (px[i] * xa[2] + py[i] * ya[2]);
        crv.set_cv_4d(i, x * wts[i], y * wts[i], z * wts[i], wts[i]);
    }

    crv.set_domain(0.0, 1.0);

    return crv;
}

/// Exact 9-CV rational NURBS ellipse.
static NurbsCurve exact_ellipse(
    double cx,
    double cy,
    double cz,
    const std::array<double, 3>& ea,
    const std::array<double, 3>& eb,
    double semi_a,
    double semi_b
) {

    double w = std::sqrt(2.0) / 2.0;
    double px[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    double py[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    double wts[9] = {1, w, 1, w, 1, w, 1, w, 1};
    NurbsCurve crv(3, true, 3, 9);
    double knots[10] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};

    for (int i = 0; i < 10; i++)
        crv.set_nurbsknot(i, knots[i]);

    for (int i = 0; i < 9; i++) {
        double x = cx + semi_a * px[i] * ea[0] + semi_b * py[i] * eb[0];
        double y = cy + semi_a * px[i] * ea[1] + semi_b * py[i] * eb[1];
        double z = cz + semi_a * px[i] * ea[2] + semi_b * py[i] * eb[2];
        crv.set_cv_4d(i, x * wts[i], y * wts[i], z * wts[i], wts[i]);
    }

    crv.set_domain(0.0, 1.0);

    return crv;
}

/// Eigenvalues/vectors of a symmetric 3x3 matrix (cyclic Jacobi).
static void jacobi_eig3(const double M[3][3], double eigvals[3], std::array<double, 3> eigvecs[3]) {

    double a[3][3], v[3][3];

    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            a[r][c] = M[r][c];
            v[r][c] = (r == c) ? 1.0 : 0.0;
        }

    for (int it = 0; it < 50; it++) {
        double off = std::abs(a[0][1]) + std::abs(a[0][2]) + std::abs(a[1][2]);

        if (off < 1e-18)
            break;

        int pq[3][2] = {{0, 1}, {0, 2}, {1, 2}};

        for (int idx = 0; idx < 3; idx++) {
            int p = pq[idx][0];
            int q = pq[idx][1];

            if (std::abs(a[p][q]) < 1e-300)
                continue;

            double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            double t = (theta >= 0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
            double c = 1.0 / std::sqrt(t * t + 1.0);
            double s = t * c;

            for (int k = 0; k < 3; k++) {
                double akp = a[k][p];
                double akq = a[k][q];
                a[k][p] = c * akp - s * akq;
                a[k][q] = s * akp + c * akq;
            }

            for (int k = 0; k < 3; k++) {
                double apk = a[p][k];
                double aqk = a[q][k];
                a[p][k] = c * apk - s * aqk;
                a[q][k] = s * apk + c * aqk;
            }

            for (int k = 0; k < 3; k++) {
                double vkp = v[k][p];
                double vkq = v[k][q];
                v[k][p] = c * vkp - s * vkq;
                v[k][q] = s * vkp + c * vkq;
            }
        }
    }

    eigvals[0] = a[0][0];
    eigvals[1] = a[1][1];
    eigvals[2] = a[2][2];

    for (int k = 0; k < 3; k++)
        eigvecs[k] = std::array<double, 3>{v[0][k], v[1][k], v[2][k]};
}

/// Recognized-surface descriptor.
struct RecogSurface {
    enum Kind { NONE, PLANE, SPHERE, CYLINDER, CONE, TORUS } kind = NONE; // Recognized kind.
    std::array<double, 3> p1{}; // Origin, center or apex.
    std::array<double, 3> p2{}; // Normal or axis.
    double r = 0.0; // Radius or major radius.
    double r2 = 0.0; // Half angle or minor radius.
};

/// Recognize a cylinder from surface samples: axis point, axis direction and radius.
static bool fit_cylinder(const NurbsSurface& surface, double tol, std::array<double, 3>& axis_pt, std::array<double, 3>& axis_dir, double& radius) {

    const std::pair<double, double> domain_u = surface.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = surface.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    std::vector<std::array<double, 3>> pts;
    std::vector<std::array<double, 3>> nrm;

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            double uu = u0 + (u1 - u0) * i / 4.0;
            double vv = v0 + (v1 - v0) * j / 4.0;
            Point p = surface.point_at(uu, vv);
            pts.push_back(std::array<double, 3>{p[0], p[1], p[2]});
            Vector n = surface.normal_at(uu, vv);
            nrm.push_back(std::array<double, 3>{n[0], n[1], n[2]});
        }
    }

    double M[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    for (std::array<double, 3>& n : nrm)
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                M[r][c] += n[r] * n[c];

    double evals[3];
    std::array<double, 3> evecs[3];
    jacobi_eig3(M, evals, evecs);
    int kmin = 0;

    for (int k = 1; k < 3; k++)
        if (evals[k] < evals[kmin])
            kmin = k;

    std::array<double, 3> w = evecs[kmin];
    double wl = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);

    if (wl < 1e-12)
        return false;

    w = std::array<double, 3>{w[0] / wl, w[1] / wl, w[2] / wl};
    std::array<double, 3> ea;
    std::array<double, 3> eb;
    std::tie(ea, eb) = ortho_basis(w);
    std::array<double, 3> p0 = pts[0];
    std::vector<std::vector<double>> ata(3, std::vector<double>(3, 0.0));
    std::vector<double> atb(3, 0.0);
    std::vector<std::pair<double, double>> proj;

    for (std::array<double, 3>& p : pts) {
        std::array<double, 3> dp{p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]};
        double x = dp[0] * ea[0] + dp[1] * ea[1] + dp[2] * ea[2];
        double y = dp[0] * eb[0] + dp[1] * eb[1] + dp[2] * eb[2];
        proj.push_back({x, y});
        double row[3] = {x, y, 1.0};
        double rhs = -(x * x + y * y);

        for (int r = 0; r < 3; r++) {
            atb[r] += row[r] * rhs;

            for (int c = 0; c < 3; c++)
                ata[r][c] += row[r] * row[c];
        }
    }

    std::vector<double> sol;

    if (!solve_gauss(ata, atb, 3, sol))
        return false;

    double ccx = -sol[0] / 2.0;
    double ccy = -sol[1] / 2.0;
    double r2 = ccx * ccx + ccy * ccy - sol[2];

    if (r2 <= 1e-18)
        return false;

    double r = std::sqrt(r2);

    for (std::pair<double, double>& pr : proj)
        if (std::abs(std::sqrt((pr.first - ccx) * (pr.first - ccx) + (pr.second - ccy) * (pr.second - ccy)) - r) > tol)
            return false;

    axis_pt =
        std::array<double, 3>{p0[0] + ccx * ea[0] + ccy * eb[0], p0[1] + ccx * ea[1] + ccy * eb[1], p0[2] + ccx * ea[2] + ccy * eb[2]};

    axis_dir = w;
    radius = r;

    return true;
}

/// Recognize a cone from surface samples: apex, axis and half angle.
static bool fit_cone(const NurbsSurface& surface, double tol, std::array<double, 3>& apex, std::array<double, 3>& axis, double& half_angle) {

    const std::pair<double, double> domain_u = surface.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = surface.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    std::vector<std::array<double, 3>> pts;
    std::vector<std::pair<std::array<double, 3>, std::array<double, 3>>> nrm; // (unit normal, point)
    int nu_s = 8;

    for (int i = 0; i < nu_s; i++) {
        double uu = u0 + (u1 - u0) * i / (double)nu_s;

        for (int j = 0; j < 5; j++) {
            double vv = v0 + (v1 - v0) * j / 4.0;
            Point p = surface.point_at(uu, vv);
            pts.push_back(std::array<double, 3>{p[0], p[1], p[2]});
            Vector n = surface.normal_at(uu, vv);
            double nl = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);

            if (nl < 1e-12)
                continue;

            Point pp = surface.point_at(uu, vv);
            nrm.push_back({std::array<double, 3>{n[0] / nl, n[1] / nl, n[2] / nl}, std::array<double, 3>{pp[0], pp[1], pp[2]}});
        }
    }

    if ((int)nrm.size() < 4)
        return false;

    std::vector<std::vector<double>> ata(3, std::vector<double>(3, 0.0));
    std::vector<double> atb(3, 0.0);

    for (std::pair<std::array<double, 3>, std::array<double, 3>>& np : nrm) {
        const std::array<double, 3>& n = np.first;
        const std::array<double, 3>& p = np.second;
        double npd = n[0] * p[0] + n[1] * p[1] + n[2] * p[2];

        for (int r = 0; r < 3; r++) {
            atb[r] += n[r] * npd;

            for (int c = 0; c < 3; c++)
                ata[r][c] += n[r] * n[c];
        }
    }

    std::vector<double> V;

    if (!solve_gauss(ata, atb, 3, V))
        return false;

    std::vector<std::array<double, 3>> gs;

    for (std::array<double, 3>& p : pts) {
        std::array<double, 3> d{p[0] - V[0], p[1] - V[1], p[2] - V[2]};
        double dl = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);

        if (dl < tol)
            continue;

        gs.push_back(std::array<double, 3>{d[0] / dl, d[1] / dl, d[2] / dl});
    }

    if ((int)gs.size() < 3)
        return false;

    double G[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    for (std::array<double, 3>& g : gs)
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                G[r][c] += g[r] * g[c];

    double gevals[3];
    std::array<double, 3> gevecs[3];
    jacobi_eig3(G, gevals, gevecs);
    int kmax = 0;

    for (int k = 1; k < 3; k++)
        if (gevals[k] > gevals[kmax])
            kmax = k;

    std::array<double, 3> w = gevecs[kmax];
    std::array<double, 3> sx{0, 0, 0};

    for (std::array<double, 3>& g : gs) {
        sx[0] += g[0];
        sx[1] += g[1];
        sx[2] += g[2];
    }

    if (w[0] * sx[0] + w[1] * sx[1] + w[2] * sx[2] < 0.0)
        w = std::array<double, 3>{-w[0], -w[1], -w[2]};

    double wl = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);

    if (wl < 1e-12)
        return false;

    w = std::array<double, 3>{w[0] / wl, w[1] / wl, w[2] / wl};
    double sumang = 0.0;

    for (std::array<double, 3>& g : gs)
        sumang += std::acos(std::max(-1.0, std::min(1.0, g[0] * w[0] + g[1] * w[1] + g[2] * w[2])));

    double alpha = sumang / gs.size();

    if (alpha < 1e-4 || alpha > Tolerance::PI / 2 - 1e-4)
        return false;

    double ca = std::cos(alpha);

    for (std::array<double, 3>& p : pts) {
        std::array<double, 3> d{p[0] - V[0], p[1] - V[1], p[2] - V[2]};
        double axd = d[0] * w[0] + d[1] * w[1] + d[2] * w[2];
        double perp = std::sqrt(std::max(0.0, (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) - axd * axd));

        if (std::abs(perp - axd * std::tan(alpha)) * ca > tol)
            return false;
    }

    apex = std::array<double, 3>{V[0], V[1], V[2]};
    axis = w;
    half_angle = alpha;

    return true;
}

/// Recognize a sphere from surface samples: center and radius.
static bool fit_sphere(const NurbsSurface& surface, double tol, double& cx, double& cy, double& cz, double& radius) {

    const std::pair<double, double> domain_u = surface.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = surface.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    std::vector<std::array<double, 3>> pts;

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            double uu = u0 + (u1 - u0) * i / 4.0;
            double vv = v0 + (v1 - v0) * j / 4.0;
            Point p = surface.point_at(uu, vv);
            pts.push_back(std::array<double, 3>{p[0], p[1], p[2]});
        }
    }

    std::vector<std::vector<double>> ata(4, std::vector<double>(4, 0.0));
    std::vector<double> atb(4, 0.0);

    for (std::array<double, 3>& p : pts) {
        double row[4] = {p[0], p[1], p[2], 1.0};
        double rhs = -(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);

        for (int r = 0; r < 4; r++) {
            atb[r] += row[r] * rhs;

            for (int c = 0; c < 4; c++)
                ata[r][c] += row[r] * row[c];
        }
    }

    std::vector<double> sol;

    if (!solve_gauss(ata, atb, 4, sol))
        return false;

    double ccx = -sol[0] / 2.0;
    double ccy = -sol[1] / 2.0;
    double ccz = -sol[2] / 2.0;
    double r2 = ccx * ccx + ccy * ccy + ccz * ccz - sol[3];

    if (r2 <= 0.0)
        return false;

    double r = std::sqrt(r2);

    for (std::array<double, 3>& p : pts) {
        double d = std::sqrt((p[0] - ccx) * (p[0] - ccx) + (p[1] - ccy) * (p[1] - ccy) + (p[2] - ccz) * (p[2] - ccz));

        if (std::abs(d - r) > tol)
            return false;
    }

    cx = ccx;
    cy = ccy;
    cz = ccz;
    radius = r;

    return true;
}

/// Recognize a torus from the smallest-variance axis and a tube cross-section circle fit.
static bool fit_torus(const NurbsSurface& surface, double tol, std::array<double, 3>& center, std::array<double, 3>& axis, double& R_out, double& r_out) {

    const std::pair<double, double> domain_u = surface.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = surface.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    std::vector<std::array<double, 3>> pts;

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            Point p = surface.point_at(u0 + (u1 - u0) * i / 8.0, v0 + (v1 - v0) * j / 8.0);
            pts.push_back(std::array<double, 3>{p[0], p[1], p[2]});
        }
    }

    int n = (int)pts.size();
    std::array<double, 3> cen{0, 0, 0};

    for (std::array<double, 3>& p : pts) {
        cen[0] += p[0];
        cen[1] += p[1];
        cen[2] += p[2];
    }

    cen[0] /= n;
    cen[1] /= n;
    cen[2] /= n;
    double M[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    for (std::array<double, 3>& p : pts) {
        std::array<double, 3> d{p[0] - cen[0], p[1] - cen[1], p[2] - cen[2]};

        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                M[r][c] += d[r] * d[c];
    }

    double evals[3];
    std::array<double, 3> evecs[3];
    jacobi_eig3(M, evals, evecs);
    int kmin = 0;

    for (int k = 1; k < 3; k++)
        if (evals[k] < evals[kmin])
            kmin = k;

    std::array<double, 3> w = evecs[kmin];
    double wl = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);

    if (wl < 1e-12)
        return false;

    w = std::array<double, 3>{w[0] / wl, w[1] / wl, w[2] / wl};
    std::vector<std::vector<double>> ata(3, std::vector<double>(3, 0.0));
    std::vector<double> atb(3, 0.0);
    std::vector<std::pair<double, double>> rhoa;

    for (std::array<double, 3>& p : pts) {
        std::array<double, 3> d{p[0] - cen[0], p[1] - cen[1], p[2] - cen[2]};
        double a = d[0] * w[0] + d[1] * w[1] + d[2] * w[2];
        std::array<double, 3> perp{d[0] - a * w[0], d[1] - a * w[1], d[2] - a * w[2]};
        double rho = std::sqrt(perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]);
        rhoa.push_back({rho, a});
        double row[3] = {rho, a, 1.0};
        double rhs = -(rho * rho + a * a);

        for (int r = 0; r < 3; r++) {
            atb[r] += row[r] * rhs;

            for (int c = 0; c < 3; c++)
                ata[r][c] += row[r] * row[c];
        }
    }

    std::vector<double> sol;

    if (!solve_gauss(ata, atb, 3, sol))
        return false;

    double R = -sol[0] / 2.0;
    double a0 = -sol[1] / 2.0;
    double r2 = R * R + a0 * a0 - sol[2];

    if (r2 <= 1e-18 || R <= 0.0)
        return false;

    double r = std::sqrt(r2);

    if (R <= r * 0.5)
        return false;

    for (std::pair<double, double>& ra : rhoa)
        if (std::abs(std::sqrt((ra.first - R) * (ra.first - R) + (ra.second - a0) * (ra.second - a0)) - r) > tol)
            return false;

    center = std::array<double, 3>{cen[0] + a0 * w[0], cen[1] + a0 * w[1], cen[2] + a0 * w[2]};
    axis = w;
    R_out = R;
    r_out = r;

    return true;
}

/// Classify a surface as plane, cylinder, cone, sphere or torus within tol.
static RecogSurface recognize_surface(const NurbsSurface& surface, double tol) {

    RecogSurface rs;

    if (surface.is_planar(nullptr, tol)) {
        const std::pair<double, double> domain_u = surface.domain(0);
        const double u0 = domain_u.first;
        const double u1 = domain_u.second;
        const std::pair<double, double> domain_v = surface.domain(1);
        const double v0 = domain_v.first;
        const double v1 = domain_v.second;
        Point o = surface.point_at((u0 + u1) * 0.5, (v0 + v1) * 0.5);
        Vector n = surface.normal_at((u0 + u1) * 0.5, (v0 + v1) * 0.5);
        rs.kind = RecogSurface::PLANE;
        rs.p1 = std::array<double, 3>{o[0], o[1], o[2]};
        rs.p2 = std::array<double, 3>{n[0], n[1], n[2]};

        return rs;
    }

    double cx;
    double cy;
    double cz;
    double r;

    if (fit_sphere(surface, tol, cx, cy, cz, r)) {
        rs.kind = RecogSurface::SPHERE;
        rs.p1 = std::array<double, 3>{cx, cy, cz};
        rs.r = r;

        return rs;
    }

    std::array<double, 3> axis_pt;
    std::array<double, 3> axis_dir;
    double rad;

    if (fit_cylinder(surface, tol, axis_pt, axis_dir, rad)) {
        rs.kind = RecogSurface::CYLINDER;
        rs.p1 = axis_pt;
        rs.p2 = axis_dir;
        rs.r = rad;

        return rs;
    }

    std::array<double, 3> apex;
    std::array<double, 3> axis;
    double half_angle;

    if (fit_cone(surface, tol, apex, axis, half_angle)) {
        rs.kind = RecogSurface::CONE;
        rs.p1 = apex;
        rs.p2 = axis;
        rs.r = half_angle;

        return rs;
    }

    std::array<double, 3> tcenter;
    std::array<double, 3> taxis;
    double tR = 0.0;
    double tr = 0.0;

    if (fit_torus(surface, tol, tcenter, taxis, tR, tr)) {
        rs.kind = RecogSurface::TORUS;
        rs.p1 = tcenter;
        rs.p2 = taxis;
        rs.r = tR;
        rs.r2 = tr;

        return rs;
    }

    return rs;
}

/// Solve ((X-V).w)^2 - cos^2a |X-V|^2 = 0 along X = x0 + t d. Returns roots.
static std::vector<double> line_cone(const std::array<double, 3>& x0, const std::array<double, 3>& d, const std::array<double, 3>& V, const std::array<double, 3>& w, double alpha) {

    double ca2 = std::cos(alpha) * std::cos(alpha);
    std::array<double, 3> e{x0[0] - V[0], x0[1] - V[1], x0[2] - V[2]};
    double A = e[0] * w[0] + e[1] * w[1] + e[2] * w[2];
    double B = d[0] * w[0] + d[1] * w[1] + d[2] * w[2];
    double C = e[0] * e[0] + e[1] * e[1] + e[2] * e[2];
    double D = e[0] * d[0] + e[1] * d[1] + e[2] * d[2];
    double E = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    double qa = B * B - ca2 * E;
    double qb = 2.0 * A * B - 2.0 * ca2 * D;
    double qc = A * A - ca2 * C;

    if (std::abs(qa) < 1e-14)
        return std::abs(qb) < 1e-300 ? std::vector<double>{} : std::vector<double>{-qc / qb};

    double disc = qb * qb - 4.0 * qa * qc;

    if (disc < 0.0)
        return {};

    double sq = std::sqrt(disc);

    return {(-qb - sq) / (2.0 * qa), (-qb + sq) / (2.0 * qa)};
}

/// Exact plane-sphere circle.
static bool ssi_plane_sphere(const RecogSurface& plane, const RecogSurface& sph, NurbsCurve& c3) {

    std::array<double, 3> o = plane.p1;
    std::array<double, 3> nu = ssi_unit(plane.p2);
    std::array<double, 3> c = sph.p1;
    double r = sph.r;
    double d = (c[0] - o[0]) * nu[0] + (c[1] - o[1]) * nu[1] + (c[2] - o[2]) * nu[2];

    if (std::abs(d) >= r)
        return false;

    std::array<double, 3> cc{c[0] - d * nu[0], c[1] - d * nu[1], c[2] - d * nu[2]};
    double rr = std::sqrt(r * r - d * d);
    std::array<double, 3> xa;
    std::array<double, 3> ya;
    std::tie(xa, ya) = ortho_basis(nu);
    c3 = exact_circle(cc[0], cc[1], cc[2], xa, ya, rr);

    return true;
}

/// Exact plane-cylinder section: an ellipse or nothing.
static bool ssi_plane_cylinder(const RecogSurface& plane, const RecogSurface& cyl, NurbsCurve& c3) {

    std::array<double, 3> o = plane.p1;
    std::array<double, 3> nu = ssi_unit(plane.p2);
    std::array<double, 3> P = cyl.p1;
    std::array<double, 3> w = ssi_unit(cyl.p2);
    double r = cyl.r;
    double wn = w[0] * nu[0] + w[1] * nu[1] + w[2] * nu[2];

    if (std::abs(wn) < 1e-7)
        return false;

    double t = ((o[0] - P[0]) * nu[0] + (o[1] - P[1]) * nu[1] + (o[2] - P[2]) * nu[2]) / wn;
    std::array<double, 3> cc{P[0] + t * w[0], P[1] + t * w[1], P[2] + t * w[2]};
    std::array<double, 3> mraw = ssi_cross(w, nu);

    if (std::sqrt(mraw[0] * mraw[0] + mraw[1] * mraw[1] + mraw[2] * mraw[2]) < 1e-9) {
        std::array<double, 3> xa;
        std::array<double, 3> ya;
        std::tie(xa, ya) = ortho_basis(nu);
        c3 = exact_circle(cc[0], cc[1], cc[2], xa, ya, r);

        return true;
    }

    std::array<double, 3> minor = ssi_unit(mraw);
    std::array<double, 3> major = ssi_unit(std::array<double, 3>{w[0] - wn * nu[0], w[1] - wn * nu[1], w[2] - wn * nu[2]});
    c3 = exact_ellipse(cc[0], cc[1], cc[2], major, minor, r / std::abs(wn), r);

    return true;
}

/// Ruling lines of a plane parallel to the cylinder axis.
static bool ssi_plane_cylinder_lines(
    const RecogSurface& plane,
    const RecogSurface& cyl,
    const NurbsSurface& cyl_srf,
    std::vector<NurbsCurve>& out
) {

    std::array<double, 3> o = plane.p1;
    std::array<double, 3> nu = ssi_unit(plane.p2);
    std::array<double, 3> P = cyl.p1;
    std::array<double, 3> w = ssi_unit(cyl.p2);
    double r = cyl.r;
    double wn = w[0] * nu[0] + w[1] * nu[1] + w[2] * nu[2];

    if (std::abs(wn) >= 1e-7)
        return false;

    double ds = (P[0] - o[0]) * nu[0] + (P[1] - o[1]) * nu[1] + (P[2] - o[2]) * nu[2];
    double d = std::abs(ds);
    double tt = r * 1e-9 + 1e-12;

    if (d > r + tt)
        return true;

    double smin = 1e300;
    double smax = -1e300;
    {
        const std::pair<double, double> domain_u = cyl_srf.domain(0);
        const double u0 = domain_u.first;
        const double u1 = domain_u.second;
        const std::pair<double, double> domain_v = cyl_srf.domain(1);
        const double v0 = domain_v.first;
        const double v1 = domain_v.second;

        for (double uu : {u0, 0.5 * (u0 + u1), u1})
            for (double vv : {v0, v1}) {
                Point p = cyl_srf.point_at(uu, vv);
                double s = (p[0] - P[0]) * w[0] + (p[1] - P[1]) * w[1] + (p[2] - P[2]) * w[2];
                smin = std::min(smin, s);
                smax = std::max(smax, s);
            }

        double pad = 0.05 * std::max(1e-9, smax - smin);
        smin -= pad;
        smax += pad;
    }

    std::array<double, 3> F{P[0] - ds * nu[0], P[1] - ds * nu[1], P[2] - ds * nu[2]};

    auto emit_line = [&](const std::array<double, 3>& q) {
        std::vector<Point> pts = {
            Point(q[0] + smin * w[0], q[1] + smin * w[1], q[2] + smin * w[2]),
            Point(q[0] + smax * w[0], q[1] + smax * w[1], q[2] + smax * w[2])
        };

        NurbsCurve L = NurbsCurve::create(false, 1, pts);

        if (L.is_valid())
            out.push_back(L);
    };

    if (d >= r - tt) {
        emit_line(F);

        return true;
    }

    double h = std::sqrt(std::max(0.0, r * r - d * d));
    std::array<double, 3> s3 = ssi_unit(ssi_cross(w, nu));
    emit_line(std::array<double, 3>{F[0] + h * s3[0], F[1] + h * s3[1], F[2] + h * s3[2]});
    emit_line(std::array<double, 3>{F[0] - h * s3[0], F[1] - h * s3[1], F[2] - h * s3[2]});

    return true;
}

/// Height of the surface along the cone axis from the apex.
static double cone_axial_extent(const NurbsSurface& srf, const std::array<double, 3>& apex, const std::array<double, 3>& axis) {

    std::array<double, 3> w = ssi_unit(axis);
    const std::pair<double, double> domain_u = srf.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = srf.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    double um = 0.5 * (u0 + u1);
    double H = 0.0;

    for (double vv : {v0, v1}) {
        Point p = srf.point_at(um, vv);
        double s = (p[0] - apex[0]) * w[0] + (p[1] - apex[1]) * w[1] + (p[2] - apex[2]) * w[2];
        H = std::max(H, s);
    }

    return H;
}

/// Whether a conic lies within the cone height H.
static bool conic_within_cone(const NurbsCurve& c, const std::array<double, 3>& apex, const std::array<double, 3>& w, double H) {

    const std::pair<double, double> domain = c.domain();
    const double t0 = domain.first;
    const double t1 = domain.second;
    double pad = 1e-7 * std::max(1.0, H);

    for (int i = 0; i <= 64; ++i) {
        Point p = c.point_at(t0 + (t1 - t0) * i / 64);
        double s = (p[0] - apex[0]) * w[0] + (p[1] - apex[1]) * w[1] + (p[2] - apex[2]) * w[2];

        if (s < -pad || s > H + pad)
            return false;
    }

    return true;
}

/// Fit a degree-2 rational arc through sampled points.
static NurbsCurve fit_conic_arc(const std::vector<Point>& pts) {

    int m = (int)pts.size();

    if (m < 2)
        return NurbsCurve();

    if (m == 2)
        return NurbsCurve::create(false, 1, pts);

    if (m <= 4)
        return NurbsCurve::create_interpolated(pts, CurveNurbsKnotStyle::Chord);

    int num_cvs = std::min(std::max(m / 6, 8), 64);

    if (num_cvs >= m)
        num_cvs = m - 1;

    NurbsCurve c = NurbsCurve::create_fitted(pts, num_cvs, 3);

    if (!c.is_valid())
        c = NurbsCurve::create_interpolated(pts, CurveNurbsKnotStyle::Chord);

    return c;
}

/// Exact ellipse of a plane cutting a cone away from the apex.
static bool build_exact_plane_cone_ellipse(
    const std::array<double, 3>& o,
    const std::array<double, 3>& nu,
    const std::array<double, 3>& V,
    const std::array<double, 3>& w,
    double alpha,
    NurbsCurve& c3
) {

    double wn = w[0] * nu[0] + w[1] * nu[1] + w[2] * nu[2];
    std::array<double, 3> m = ssi_cross(w, nu);
    double ml = std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);

    if (ml < 1e-12)
        return false;

    m = std::array<double, 3>{m[0] / ml, m[1] / ml, m[2] / ml};
    std::array<double, 3> major = ssi_unit(std::array<double, 3>{w[0] - wn * nu[0], w[1] - wn * nu[1], w[2] - wn * nu[2]});
    double dV = (V[0] - o[0]) * nu[0] + (V[1] - o[1]) * nu[1] + (V[2] - o[2]) * nu[2];
    std::array<double, 3> Vp{V[0] - dV * nu[0], V[1] - dV * nu[1], V[2] - dV * nu[2]};
    std::vector<double> ts = line_cone(Vp, major, V, w, alpha);

    if (ts.size() != 2)
        return false;

    std::array<double, 3> A{Vp[0] + ts[0] * major[0], Vp[1] + ts[0] * major[1], Vp[2] + ts[0] * major[2]};
    std::array<double, 3> Bp{Vp[0] + ts[1] * major[0], Vp[1] + ts[1] * major[1], Vp[2] + ts[1] * major[2]};
    std::array<double, 3> cc{(A[0] + Bp[0]) * 0.5, (A[1] + Bp[1]) * 0.5, (A[2] + Bp[2]) * 0.5};
    double semi_major = 0.5 *
        std::sqrt((Bp[0] - A[0]) * (Bp[0] - A[0]) + (Bp[1] - A[1]) * (Bp[1] - A[1]) + (Bp[2] - A[2]) * (Bp[2] - A[2]));

    major = ssi_unit(std::array<double, 3>{Bp[0] - A[0], Bp[1] - A[1], Bp[2] - A[2]});
    std::vector<double> tm = line_cone(cc, m, V, w, alpha);

    if (tm.size() != 2)
        return false;

    double semi_minor = 0.5 * std::abs(tm[1] - tm[0]);

    if (semi_major < 1e-12 || semi_minor < 1e-12)
        return false;

    c3 = exact_ellipse(cc[0], cc[1], cc[2], major, m, semi_major, semi_minor);

    return true;
}

/// Single rational quadratic Bezier conic arc through A and B with mid control point T.
static NurbsCurve conic_bezier(const std::array<double, 3>& A, const std::array<double, 3>& T, const std::array<double, 3>& B, double wmid) {

    NurbsCurve crv(3, true, 3, 3);
    double knots[4] = {0, 0, 1, 1};

    for (int i = 0; i < 4; i++)
        crv.set_nurbsknot(i, knots[i]);

    crv.set_cv_4d(0, A[0], A[1], A[2], 1.0);
    crv.set_cv_4d(1, T[0] * wmid, T[1] * wmid, T[2] * wmid, wmid);
    crv.set_cv_4d(2, B[0], B[1], B[2], 1.0);
    crv.set_domain(0.0, 1.0);

    return crv;
}

/// Exact plane-cone HYPERBOLA / PARABOLA arc (IntAna_QuadQuadGeo.cxx:752-953 port).
static bool build_exact_plane_cone_open(
    const std::array<double, 3>& o,
    const std::array<double, 3>& nu,
    const std::array<double, 3>& V,
    const std::array<double, 3>& w,
    double alpha,
    double H,
    bool parabola,
    NurbsCurve& c3
) {

    double cosa = std::cos(alpha);
    double sina = std::sin(alpha);
    double ta = std::tan(alpha);
    double na = nu[0] * w[0] + nu[1] * w[1] + nu[2] * w[2];
    double cost = std::abs(na);
    std::array<double, 3> axey = ssi_cross(nu, w);
    double sint = std::sqrt(axey[0] * axey[0] + axey[1] * axey[1] + axey[2] * axey[2]);

    if (sint < 1e-12)
        return false;

    axey = std::array<double, 3>{axey[0] / sint, axey[1] / sint, axey[2] / sint};
    std::array<double, 3> axex = ssi_cross(axey, nu);
    double axw = axex[0] * w[0] + axex[1] * w[1] + axex[2] * w[2];

    if (axw < 0) {
        axex = std::array<double, 3>{-axex[0], -axex[1], -axex[2]};
        axw = -axw;
    }

    if (axw < 1e-12)
        return false;

    double D0 = (V[0] - o[0]) * nu[0] + (V[1] - o[1]) * nu[1] + (V[2] - o[2]) * nu[2];
    double tol = 1e-6 * std::max(1.0, H);

    auto on_both = [&](const NurbsCurve& c) {
        for (int i = 0; i <= 16; ++i) {
            Point q = c.point_at(i / 16.0);
            double dp = std::abs((q[0] - o[0]) * nu[0] + (q[1] - o[1]) * nu[1] + (q[2] - o[2]) * nu[2]);
            double zz = (q[0] - V[0]) * w[0] + (q[1] - V[1]) * w[1] + (q[2] - V[2]) * w[2];
            double wx = q[0] - V[0] - zz * w[0];
            double wy = q[1] - V[1] - zz * w[1];
            double wz = q[2] - V[2] - zz * w[2];
            double rho = std::sqrt(wx * wx + wy * wy + wz * wz);

            if (dp > tol || std::abs(rho - ta * zz) > tol * (1.0 + ta))
                return false;

            if (zz < -tol || zz > H + tol)
                return false;
        }

        return true;
    };

    if (parabola) {
        if (cost < 1e-12)
            return false;

        double sax = -D0 / na;
        std::array<double, 3> cen{V[0] + sax * w[0], V[1] + sax * w[1], V[2] + sax * w[2]};
        double distance = std::abs(sax);
        double dc = 0.5 * distance / cosa;
        double pf = dc * sina * sina;

        if (pf < 1e-15)
            return false;

        for (int cs : {-1, +1}) {
            std::array<double, 3> C2{cen[0] + cs * dc * axex[0], cen[1] + cs * dc * axex[1], cen[2] + cs * dc * axex[2]};
            double zc = (C2[0] - V[0]) * w[0] + (C2[1] - V[1]) * w[1] + (C2[2] - V[2]) * w[2];
            double t1s = 2.0 * pf * (H - zc) / axw;

            if (t1s <= 0)
                continue;

            double t1 = std::sqrt(t1s);
            double xi = t1s / (2.0 * pf);
            std::array<double, 3> A{
                C2[0] + xi * axex[0] - t1 * axey[0],
                C2[1] + xi * axex[1] - t1 * axey[1],
                C2[2] + xi * axex[2] - t1 * axey[2]
            };

            std::array<double, 3> B{
                C2[0] + xi * axex[0] + t1 * axey[0],
                C2[1] + xi * axex[1] + t1 * axey[1],
                C2[2] + xi * axex[2] + t1 * axey[2]
            };

            std::array<double, 3> T{C2[0] - xi * axex[0], C2[1] - xi * axex[1], C2[2] - xi * axex[2]};
            NurbsCurve arc = conic_bezier(A, T, B, 1.0);

            if (arc.is_valid() && on_both(arc)) {
                c3 = arc;

                return true;
            }
        }

        return false;
    }

    double a = 0;
    double b = 0;
    std::vector<std::array<double, 3>> centers;

    if (cost < 1e-6) {
        a = std::abs(D0) / ta;
        b = std::abs(D0);
        centers.push_back(std::array<double, 3>{V[0] - D0 * nu[0], V[1] - D0 * nu[1], V[2] - D0 * nu[2]});
    } else {
        double dd = sina * sina - cost * cost;

        if (dd < 1e-12)
            return false;

        double sax = -D0 / na;
        std::array<double, 3> cen{V[0] + sax * w[0], V[1] + sax * w[1], V[2] + sax * w[2]};
        double distance = std::abs(sax);
        double dc = sint * sina * sina * distance / dd;
        a = cost * sina * cosa * distance / dd;
        b = cost * sina * distance / std::sqrt(dd);
        centers.push_back(std::array<double, 3>{cen[0] - dc * axex[0], cen[1] - dc * axex[1], cen[2] - dc * axex[2]});
        centers.push_back(std::array<double, 3>{cen[0] + dc * axex[0], cen[1] + dc * axex[1], cen[2] + dc * axex[2]});
    }

    if (a < 1e-15 || b < 1e-15)
        return false;

    for (const std::array<double, 3>& C2 : centers) {
        double zc = (C2[0] - V[0]) * w[0] + (C2[1] - V[1]) * w[1] + (C2[2] - V[2]) * w[2];

        for (int sg : {+1, -1}) {
            double ch = (H - zc) / (sg * a * axw);

            if (ch <= 1.0 + 1e-12)
                continue;

            double sh = std::sqrt(ch * ch - 1.0);
            double xi = sg * a * ch;
            double xt = sg * a / ch;
            std::array<double, 3> A{
                C2[0] + xi * axex[0] - b * sh * axey[0],
                C2[1] + xi * axex[1] - b * sh * axey[1],
                C2[2] + xi * axex[2] - b * sh * axey[2]
            };

            std::array<double, 3> B{
                C2[0] + xi * axex[0] + b * sh * axey[0],
                C2[1] + xi * axex[1] + b * sh * axey[1],
                C2[2] + xi * axex[2] + b * sh * axey[2]
            };

            std::array<double, 3> T{C2[0] + xt * axex[0], C2[1] + xt * axex[1], C2[2] + xt * axex[2]};
            NurbsCurve arc = conic_bezier(A, T, B, ch);

            if (arc.is_valid() && on_both(arc)) {
                c3 = arc;

                return true;
            }
        }
    }

    return false;
}

/// Sample the plane-cone section as point runs, one per branch.
static void sample_plane_cone_arcs(
    const std::array<double, 3>& apex,
    const std::array<double, 3>& w,
    const std::array<double, 3>& e1,
    const std::array<double, 3>& e2,
    double na,
    double pP,
    double qP,
    double D0,
    double ta,
    double H,
    std::vector<std::vector<Point>>& runs,
    bool& closed
) {

    runs.clear();
    closed = false;
    const int N = 720;
    const double TWO_PI = 2.0 * 3.14159265358979323846;

    auto denom = [&](double phi) {
        return na + ta * (pP * std::cos(phi) + qP * std::sin(phi));
    };

    auto s_of = [&](double phi) {
        double d = denom(phi);

        return (std::abs(d) < 1e-300) ? 1e308 : -D0 / d;
    };

    auto pt = [&](double phi) {
        double s = s_of(phi);
        double rr = s * ta;
        double c = std::cos(phi);
        double sn = std::sin(phi);

        return Point(
            apex[0] + s * w[0] + rr * (c * e1[0] + sn * e2[0]),
            apex[1] + s * w[1] + rr * (c * e1[1] + sn * e2[1]),
            apex[2] + s * w[2] + rr * (c * e1[2] + sn * e2[2])
        );
    };

    const double eps = 1e-9 * std::max(1.0, H);
    std::vector<char> ok(N);
    int cnt = 0;

    for (int k = 0; k < N; ++k) {
        double s = s_of(TWO_PI * k / N);
        ok[k] = (s > eps && s < H + eps) ? 1 : 0;
        cnt += ok[k];
    }

    if (cnt == 0)
        return;

    if (cnt == N) {
        std::vector<Point> loop;

        for (int k = 0; k <= N; ++k)
            loop.push_back(pt(TWO_PI * (k % N) / N));

        runs.push_back(loop);
        closed = true;

        return;
    }

    int start = 0;

    while (start < N && ok[start])
        ++start;

    double dtarget = (H > 1e-300) ? (-D0 / H) : 0.0;

    auto refine_base = [&](double pa, double pb) -> double {
        double fa = denom(pa) - dtarget;

        for (int it = 0; it < 60; ++it) {
            double pm = 0.5 * (pa + pb);
            double fm = denom(pm) - dtarget;

            if ((fm < 0) == (fa < 0)) {
                pa = pm;
                fa = fm;
            } else
                pb = pm;
        }

        return 0.5 * (pa + pb);
    };

    std::vector<Point> cur;
    bool in = false;

    for (int i = 0; i <= N; ++i) {
        int k = (start + i) % N;
        double uphi = TWO_PI * start / N + TWO_PI * i / N;
        bool v = ok[k] != 0;

        if (v && !in) {
            if (i > 0)
                cur.push_back(pt(refine_base(uphi - TWO_PI / N, uphi)));

            cur.push_back(pt(uphi));
            in = true;
        } else if (v && in) {
            cur.push_back(pt(uphi));
        } else if (!v && in) {
            cur.push_back(pt(refine_base(uphi - TWO_PI / N, uphi)));

            if (cur.size() >= 2)
                runs.push_back(cur);

            cur.clear();
            in = false;
        }
    }

    if (in && cur.size() >= 2)
        runs.push_back(cur);
}

/// Plane-cone section: exact ellipse when closed, fitted arcs otherwise.
static bool ssi_plane_cone(
    const RecogSurface& plane,
    const RecogSurface& cone,
    const NurbsSurface& cone_srf,
    std::vector<NurbsCurve>& out
) {

    std::array<double, 3> o = plane.p1;
    std::array<double, 3> nu = ssi_unit(plane.p2);
    std::array<double, 3> V = cone.p1;
    std::array<double, 3> w = ssi_unit(cone.p2);
    double alpha = cone.r;

    if (alpha < 1e-7 || alpha > Tolerance::PI / 2 - 1e-7)
        return false;

    double ta = std::tan(alpha);
    double cosa = std::cos(alpha);
    double sina = std::sin(alpha);
    double H = cone_axial_extent(cone_srf, V, w);

    if (H < 1e-12)
        return false;

    std::array<double, 3> e1;
    std::array<double, 3> e2;
    std::tie(e1, e2) = ortho_basis(w);
    double na = nu[0] * w[0] + nu[1] * w[1] + nu[2] * w[2];
    double pP = nu[0] * e1[0] + nu[1] * e1[1] + nu[2] * e1[2];
    double qP = nu[0] * e2[0] + nu[1] * e2[1] + nu[2] * e2[2];
    double cost = std::abs(na);
    double sint = std::sqrt(std::max(0.0, pP * pP + qP * qP));
    double costa = cost * cosa - sint * sina;
    double D0 = (V[0] - o[0]) * nu[0] + (V[1] - o[1]) * nu[1] + (V[2] - o[2]) * nu[2];
    const double ang = 1e-6;
    const double distTol = 1e-6 * std::max(1.0, H);

    if (std::abs(D0) < distTol) {
        if (std::abs(costa) < ang) {
            std::array<double, 3> g = ssi_unit(std::array<double, 3>{w[0] - na * nu[0], w[1] - na * nu[1], w[2] - na * nu[2]});
            double gw = g[0] * w[0] + g[1] * w[1] + g[2] * w[2];

            if (gw > 1e-9) {
                double L = H / gw;

                out.push_back(
                    NurbsCurve::create(
                        false,
                        1,
                        {Point(V[0], V[1], V[2]), Point(V[0] + L * g[0], V[1] + L * g[1], V[2] + L * g[2])}
                    )
                );
            }

            return true;
        }

        if (cost < sina) {
            std::array<double, 3> axey = ssi_cross(nu, w);
            std::array<double, 3> axex = ssi_cross(axey, nu);
            double dh = std::sqrt(std::max(0.0, sina * sina - cost * cost)) / cosa;

            for (int sgn : {+1, -1}) {
                std::array<double, 3> d{axex[0] + sgn * dh * axey[0], axex[1] + sgn * dh * axey[1], axex[2] + sgn * dh * axey[2]};
                double dw = d[0] * w[0] + d[1] * w[1] + d[2] * w[2];

                if (dw < 1e-12)
                    continue;

                double L = H / dw;

                out.push_back(
                    NurbsCurve::create(
                        false,
                        1,
                        {Point(V[0], V[1], V[2]), Point(V[0] + L * d[0], V[1] + L * d[1], V[2] + L * d[2])}
                    )
                );
            }

            return true;
        }

        return true;
    }

    bool is_circle = false;
    bool is_parabola = false;
    bool is_hyperbola = false;
    bool is_ellipse = false;

    if (cost < ang)
        is_hyperbola = true;
    else if (std::abs(costa) < ang)
        is_parabola = true;
    else if (sint < ang)
        is_circle = true;
    else if (cost < sina)
        is_hyperbola = true;
    else
        is_ellipse = true;

    if (is_circle) {
        double dax = (o[0] - V[0]) * w[0] + (o[1] - V[1]) * w[1] + (o[2] - V[2]) * w[2];
        double rr = std::abs(dax) * ta;

        if (rr > 1e-12) {
            std::array<double, 3> cc{V[0] + dax * w[0], V[1] + dax * w[1], V[2] + dax * w[2]};
            NurbsCurve circ = exact_circle(cc[0], cc[1], cc[2], e1, e2, rr);

            if (conic_within_cone(circ, V, w, H)) {
                out.push_back(circ);

                return true;
            }
        }

        return true;
    }

    if (is_ellipse) {
        NurbsCurve ell;

        if (build_exact_plane_cone_ellipse(o, nu, V, w, alpha, ell) && conic_within_cone(ell, V, w, H)) {
            out.push_back(ell);

            return true;
        }
    }

    if (is_parabola || is_hyperbola) {
        NurbsCurve arc;
        bool okx = build_exact_plane_cone_open(o, nu, V, w, alpha, H, is_parabola, arc);

        if (okx) {
            out.push_back(arc);

            return true;
        }
    }

    std::vector<std::vector<Point>> runs;
    bool closed = false;
    sample_plane_cone_arcs(V, w, e1, e2, na, pP, qP, D0, ta, H, runs, closed);

    for (std::vector<Point>& r : runs) {
        NurbsCurve c = fit_conic_arc(r);

        if (c.is_valid())
            out.push_back(c);
    }

    return true;
}

/// Exact plane-torus circles for a plane perpendicular to the axis.
static bool ssi_plane_torus(const RecogSurface& plane, const RecogSurface& tor, std::vector<NurbsCurve>& out) {

    std::array<double, 3> o = plane.p1;
    std::array<double, 3> nu = ssi_unit(plane.p2);
    std::array<double, 3> C = tor.p1;
    std::array<double, 3> w = ssi_unit(tor.p2);
    double R = tor.r;
    double r = tor.r2;
    double wn = w[0] * nu[0] + w[1] * nu[1] + w[2] * nu[2];

    if (std::abs(std::abs(wn) - 1.0) > 1e-7)
        return false;

    double d = (o[0] - C[0]) * w[0] + (o[1] - C[1]) * w[1] + (o[2] - C[2]) * w[2];

    if (std::abs(d) > r)
        return true;

    double h = std::sqrt(std::max(0.0, r * r - d * d));
    std::array<double, 3> cc{C[0] + d * w[0], C[1] + d * w[1], C[2] + d * w[2]};
    std::array<double, 3> xa;
    std::array<double, 3> ya;
    std::tie(xa, ya) = ortho_basis(w);

    for (double rr : {R + h, R - h})
        if (rr > 1e-12)
            out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rr));

    return true;
}

/// Exact plane-plane line clipped to both finite faces.
static bool ssi_plane_plane(
    const NurbsSurface& sa,
    const RecogSurface& pa,
    const NurbsSurface& sb,
    const RecogSurface& pb,
    NurbsCurve& c3,
    bool& empty
) {

    empty = false;
    std::array<double, 3> na = ssi_unit(pa.p2);
    std::array<double, 3> nb = ssi_unit(pb.p2);
    std::array<double, 3> v = ssi_cross(na, nb);
    double vl = std::sqrt(ssi_dot(v, v));

    if (vl < 1e-9)
        return false;

    double dA = ssi_dot(na, pa.p1);
    double dB = ssi_dot(nb, pb.p1);
    std::array<double, 3> nb_x_v = ssi_cross(nb, v);
    std::array<double, 3> v_x_na = ssi_cross(v, na);
    double inv = 1.0 / (vl * vl);
    std::array<double, 3> anchor{
        (dA * nb_x_v[0] + dB * v_x_na[0]) * inv,
        (dA * nb_x_v[1] + dB * v_x_na[1]) * inv,
        (dA * nb_x_v[2] + dB * v_x_na[2]) * inv
    };

    std::array<double, 3> dir{v[0] / vl, v[1] / vl, v[2] / vl};

    double tmin = -1e300;
    double tmax = 1e300;

    for (const NurbsSurface* s : {&sa, &sb}) {
        const std::pair<double, double> domain_u = s->domain(0);
        const double u0 = domain_u.first;
        const double u1 = domain_u.second;
        const std::pair<double, double> domain_v = s->domain(1);
        const double v0 = domain_v.first;
        const double v1 = domain_v.second;
        Point O = s->point_at(u0, v0);
        Point Pu = s->point_at(u1, v0);
        Point Pv = s->point_at(u0, v1);
        std::array<double, 3> o{O[0], O[1], O[2]};
        std::array<double, 3> eu{Pu[0] - O[0], Pu[1] - O[1], Pu[2] - O[2]};
        std::array<double, 3> ev{Pv[0] - O[0], Pv[1] - O[1], Pv[2] - O[2]};
        double exx = ssi_dot(eu, eu);
        double eyy = ssi_dot(ev, ev);
        double exy = ssi_dot(eu, ev);
        double det = exx * eyy - exy * exy;

        if (std::abs(det) < 1e-18)
            return false;

        auto frac = [&](const std::array<double, 3>& r, double& al, double& be) {
            double rx = ssi_dot(r, eu);
            double ry = ssi_dot(r, ev);
            al = (eyy * rx - exy * ry) / det;
            be = (exx * ry - exy * rx) / det;
        };

        double a0;
        double b0;
        double da;
        double db;
        frac(std::array<double, 3>{anchor[0] - o[0], anchor[1] - o[1], anchor[2] - o[2]}, a0, b0);
        frac(dir, da, db);
        double t0 = -1e300;
        double t1 = 1e300;

        auto axis_clip = [&](double c, double d) -> bool {
            if (std::abs(d) < 1e-15)
                return (c >= -1e-9 && c <= 1.0 + 1e-9);

            double ta = (0.0 - c) / d;
            double tb = (1.0 - c) / d;

            if (ta > tb)
                std::swap(ta, tb);

            t0 = std::max(t0, ta);
            t1 = std::min(t1, tb);

            return true;
        };

        if (!axis_clip(a0, da) || !axis_clip(b0, db) || t0 > t1) {
            empty = true;

            return false;
        }

        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
    }

    if (tmax - tmin <= 1e-9) {
        empty = true;

        return false;
    }

    Point A(anchor[0] + tmin * dir[0], anchor[1] + tmin * dir[1], anchor[2] + tmin * dir[2]);
    Point B(anchor[0] + tmax * dir[0], anchor[1] + tmax * dir[1], anchor[2] + tmax * dir[2]);
    c3 = NurbsCurve::create(false, 1, {A, B});
    c3.set_domain(0.0, 1.0);

    return true;
}

/// Tri-state analytic result: not analytic, recognised empty, or curve triples.
struct AnalyticResult {
    enum { NOT_ANALYTIC, NO_HIT, HIT } status = NOT_ANALYTIC; // Whether both surfaces were recognized and whether they meet.
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> triples; // 3D curve with both pullbacks.
};

/// Analytic pcurve of an exact 3D intersection conic on a recognized quadric surface.
static NurbsCurve analytic_pcurve(const NurbsSurface& srf, const RecogSurface& recog, const NurbsCurve& c3d) {

    const std::pair<double, double> domain_u = srf.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = srf.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;

    if (recog.kind == RecogSurface::PLANE) {
        Point o = srf.point_at(u0, v0);
        Point pu = srf.point_at(u1, v0);
        Point pv = srf.point_at(u0, v1);
        double ex[3] = {pu[0] - o[0], pu[1] - o[1], pu[2] - o[2]};
        double ey[3] = {pv[0] - o[0], pv[1] - o[1], pv[2] - o[2]};
        double exx = dot3(ex, ex);
        double eyy = dot3(ey, ey);
        double exy = dot3(ex, ey);
        double det = exx * eyy - exy * exy;

        if (std::abs(det) < 1e-18)
            return NurbsCurve();

        NurbsCurve pc = c3d;
        int nc = c3d.cv_count();

        for (int i = 0; i < nc; ++i) {
            Point P = c3d.get_cv(i);
            double r[3] = {P[0] - o[0], P[1] - o[1], P[2] - o[2]};
            double rx = dot3(r, ex);
            double ry = dot3(r, ey);
            double a = (eyy * rx - exy * ry) / det;
            double b = (exx * ry - exy * rx) / det;
            double u = u0 + a * (u1 - u0);
            double v = v0 + b * (v1 - v0);

            if (c3d.is_rational()) {
                double w = c3d.weight(i);
                pc.set_cv_4d(i, u * w, v * w, 0.0, w);
            } else {
                pc.set_cv(i, Point(u, v, 0.0));
            }
        }

        return pc;
    }

    if (recog.kind == RecogSurface::CYLINDER) {
        double ap[3] = {recog.p1[0], recog.p1[1], recog.p1[2]};
        double ax[3] = {recog.p2[0], recog.p2[1], recog.p2[2]};
        double an = std::sqrt(dot3(ax, ax));

        if (an < 1e-12)
            return NurbsCurve();

        ax[0] /= an;
        ax[1] /= an;
        ax[2] /= an;

        auto height = [&](const Point& p) {
            double r[3] = {p[0] - ap[0], p[1] - ap[1], p[2] - ap[2]};

            return dot3(r, ax);
        };

        double um = 0.5 * (u0 + u1);
        double h0 = height(srf.point_at(um, v0));
        double h1 = height(srf.point_at(um, v1));

        if (std::abs(h1 - h0) < 1e-12)
            return NurbsCurve();

        double hmin = 1e300;
        double hmax = -1e300;
        double hsum = 0;
        int ns = 0;
        const std::pair<double, double> domain = c3d.domain();
        const double t0 = domain.first;
        const double t1 = domain.second;

        for (int i = 0; i <= 32; ++i) {
            double h = height(c3d.point_at(t0 + (t1 - t0) * i / 32));
            hmin = std::min(hmin, h);
            hmax = std::max(hmax, h);
            hsum += h;
            ns++;
        }

        if (hmax - hmin > 1e-5 * std::abs(h1 - h0))
            return NurbsCurve();

        if (c3d.point_at(t0).distance(c3d.point_at(t1)) > 1e-6 * (std::abs(h1 - h0) + 1.0))
            return NurbsCurve();

        double hc = hsum / ns;
        double vc = v0 + (hc - h0) / (h1 - h0) * (v1 - v0);

        if (vc < std::min(v0, v1) - 1e-9 || vc > std::max(v0, v1) + 1e-9)
            return NurbsCurve();

        return NurbsCurve::create(false, 1, {Point(u0, vc, 0.0), Point(u1, vc, 0.0)});
    }

    if (recog.kind == RecogSurface::SPHERE) {
        double um = 0.5 * (u0 + u1);
        Point sp = srf.point_at(um, v0);
        Point np = srf.point_at(um, v1);
        double ax[3] = {np[0] - sp[0], np[1] - sp[1], np[2] - sp[2]};
        double an = std::sqrt(dot3(ax, ax));

        if (an < 1e-12)
            return NurbsCurve();

        ax[0] /= an;
        ax[1] /= an;
        ax[2] /= an;
        double C[3] = {recog.p1[0], recog.p1[1], recog.p1[2]};

        auto height = [&](const Point& p) {
            double r[3] = {p[0] - C[0], p[1] - C[1], p[2] - C[2]};

            return dot3(r, ax);
        };

        const std::pair<double, double> domain = c3d.domain();
        const double t0 = domain.first;
        const double t1 = domain.second;
        double hmin = 1e300;
        double hmax = -1e300;
        double hsum = 0;
        int ns = 0;

        for (int i = 0; i <= 32; ++i) {
            double h = height(c3d.point_at(t0 + (t1 - t0) * i / 32));
            hmin = std::min(hmin, h);
            hmax = std::max(hmax, h);
            hsum += h;
            ns++;
        }

        if (hmax - hmin > recog.r * 1e-4)
            return NurbsCurve();

        if (c3d.point_at(t0).distance(c3d.point_at(t1)) > recog.r * 1e-3)
            return NurbsCurve();

        double hc = hsum / ns;
        double va = v0;
        double vb = v1;
        double ha = height(srf.point_at(um, va));
        double hb = height(srf.point_at(um, vb));

        if ((hc - ha) * (hc - hb) > 0)
            return NurbsCurve();

        for (int it = 0; it < 60; ++it) {
            double vm = 0.5 * (va + vb);
            double hm = height(srf.point_at(um, vm));

            if ((hm - hc) * (ha - hc) <= 0)
                vb = vm;
            else {
                va = vm;
                ha = hm;
            }
        }

        double vc = 0.5 * (va + vb);

        return NurbsCurve::create(false, 1, {Point(u0, vc, 0.0), Point(u1, vc, 0.0)});
    }

    if (recog.kind == RecogSurface::CONE) {
        double ax[3] = {recog.p2[0], recog.p2[1], recog.p2[2]};
        double an = std::sqrt(dot3(ax, ax));

        if (an < 1e-12)
            return NurbsCurve();

        ax[0] /= an;
        ax[1] /= an;
        ax[2] /= an;
        double A[3] = {recog.p1[0], recog.p1[1], recog.p1[2]};

        auto height = [&](const Point& p) {
            double r[3] = {p[0] - A[0], p[1] - A[1], p[2] - A[2]};

            return dot3(r, ax);
        };

        const std::pair<double, double> domain = c3d.domain();
        const double t0 = domain.first;
        const double t1 = domain.second;
        double clen = c3d.point_at(t0).distance(c3d.point_at(0.5 * (t0 + t1)));
        double hscale = std::max(clen, 1e-9);
        double hmin = 1e300;
        double hmax = -1e300;
        double hsum = 0;
        int ns = 0;

        for (int i = 0; i <= 32; ++i) {
            double h = height(c3d.point_at(t0 + (t1 - t0) * i / 32));
            hmin = std::min(hmin, h);
            hmax = std::max(hmax, h);
            hsum += h;
            ++ns;
        }

        if (hmax - hmin > hscale * 1e-4)
            return NurbsCurve();

        if (c3d.point_at(t0).distance(c3d.point_at(t1)) > hscale * 1e-3)
            return NurbsCurve();

        double hc = hsum / ns;
        double um2 = 0.5 * (u0 + u1);
        double va = v0;
        double vb = v1;
        double ha = height(srf.point_at(um2, va));
        double hb = height(srf.point_at(um2, vb));

        if ((hc - ha) * (hc - hb) > 0)
            return NurbsCurve();

        for (int it = 0; it < 60; ++it) {
            double vmid = 0.5 * (va + vb);
            double hm = height(srf.point_at(um2, vmid));

            if ((hm - hc) * (ha - hc) <= 0)
                vb = vmid;
            else {
                va = vmid;
                ha = hm;
            }
        }

        double vc = 0.5 * (va + vb);

        return NurbsCurve::create(false, 1, {Point(u0, vc, 0.0), Point(u1, vc, 0.0)});
    }

    if (recog.kind == RecogSurface::TORUS) {
        const double PI = 3.14159265358979323846;
        const double TWO_PI = 2.0 * PI;
        double C[3] = {recog.p1[0], recog.p1[1], recog.p1[2]};
        double w[3] = {recog.p2[0], recog.p2[1], recog.p2[2]};
        double wn = std::sqrt(dot3(w, w));

        if (wn < 1e-12)
            return NurbsCurve();

        w[0] /= wn;
        w[1] /= wn;
        w[2] /= wn;
        double R = recog.r;
        double rmin = recog.r2;

        if (rmin < 1e-12 || R <= rmin)
            return NurbsCurve();

        auto minor_angle = [&](const Point& p) {
            double d[3] = {p[0] - C[0], p[1] - C[1], p[2] - C[2]};
            double z = dot3(d, w);
            double hx = d[0] - z * w[0];
            double hy = d[1] - z * w[1];
            double hz = d[2] - z * w[2];
            double rho = std::sqrt(hx * hx + hy * hy + hz * hz);

            return std::atan2(z, rho - R);
        };

        const std::pair<double, double> domain = c3d.domain();
        const double t0 = domain.first;
        const double t1 = domain.second;
        double aprev = 0.0;
        double asum = 0.0;
        double amin = 1e300;
        double amax = -1e300;
        int ns = 0;

        for (int i = 0; i <= 32; ++i) {
            double a = minor_angle(c3d.point_at(t0 + (t1 - t0) * i / 32));

            if (i > 0) {
                while (a - aprev > PI)
                    a -= TWO_PI;

                while (a - aprev < -PI)
                    a += TWO_PI;
            }

            aprev = a;
            amin = std::min(amin, a);
            amax = std::max(amax, a);
            asum += a;
            ++ns;
        }

        if (amax - amin > 1e-4)
            return NurbsCurve();

        if (c3d.point_at(t0).distance(c3d.point_at(t1)) > rmin * 1e-3)
            return NurbsCurve();

        double a_target = asum / ns;
        double um = 0.5 * (u0 + u1);
        const int NV = 256;
        std::vector<double> tv(NV + 1), ta(NV + 1);
        double ap = 0.0;

        for (int k = 0; k <= NV; ++k) {
            double v = v0 + (v1 - v0) * k / NV;
            double a = minor_angle(srf.point_at(um, v));

            if (k > 0) {
                while (a - ap > PI)
                    a -= TWO_PI;

                while (a - ap < -PI)
                    a += TWO_PI;
            }

            ap = a;
            tv[k] = v;
            ta[k] = a;
        }

        double alo = std::min(ta[0], ta[NV]);
        double ahi = std::max(ta[0], ta[NV]);

        while (a_target < alo - 1e-9)
            a_target += TWO_PI;

        while (a_target > ahi + 1e-9)
            a_target -= TWO_PI;

        if (a_target < alo - 1e-9 || a_target > ahi + 1e-9)
            return NurbsCurve();

        bool incr = ta[NV] >= ta[0];
        int lo = 0;
        int hi = NV;

        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            bool above = incr ? (ta[mid] < a_target) : (ta[mid] > a_target);

            if (above)
                lo = mid;
            else
                hi = mid;
        }

        double denom = ta[hi] - ta[lo];
        double f = (std::abs(denom) > 1e-15) ? (a_target - ta[lo]) / denom : 0.0;
        double vc = tv[lo] + (tv[hi] - tv[lo]) * f;
        {
            const double dv = (v1 - v0) * 1e-7;
            const double vlo = std::min(v0, v1);
            const double vhi = std::max(v0, v1);

            auto ang_at = [&](double v) {
                double a = minor_angle(srf.point_at(um, std::min(std::max(v, vlo), vhi))) - a_target;

                while (a > PI)
                    a -= TWO_PI;

                while (a < -PI)
                    a += TWO_PI;

                return a;
            };

            for (int np = 0; np < 3; ++np) {
                double g0 = ang_at(vc);
                double g1 = ang_at(std::min(vc + dv, vhi));
                double dg = (g1 - g0) / dv;

                if (std::abs(dg) < 1e-12)
                    break;

                double vn = std::min(std::max(vc - g0 / dg, vlo), vhi);

                if (std::abs(vn - vc) <= 1e-15 * std::max(1.0, std::abs(vc))) {
                    vc = vn;
                    break;
                }

                vc = vn;
            }
        }

        return NurbsCurve::create(false, 1, {Point(u0, vc, 0.0), Point(u1, vc, 0.0)});
    }

    return NurbsCurve();
}

/// Analytic pull-back of a 3D curve onto a recognized sphere, split at the seam.
struct PBNode {
    double t; // Curve parameter.
    double u; // Surface u.
    double v; // Surface v.
};

/// Degree-1 UV polyline through pull-back nodes.
static NurbsCurve emit_pullback_curve(const std::vector<PBNode>& nodes) {

    std::vector<Point> pts;
    pts.reserve(nodes.size());

    for (const PBNode& n : nodes)
        pts.push_back(Point(n.u, n.v, 0.0));

    return NurbsCurve::create(false, 1, pts);
}

/// Pull a 3D curve back to sphere parameters through longitude and latitude.
static std::vector<NurbsCurve> analytic_sphere_pullback(
    const NurbsSurface& srf,
    const RecogSurface& recog,
    const NurbsCurve& c3d
) {

    if (recog.kind != RecogSurface::SPHERE)
        return {};

    const std::pair<double, double> domain_u = srf.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = srf.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    double range_u = u1 - u0;

    if (range_u < 1e-9)
        return {};

    double C[3] = {recog.p1[0], recog.p1[1], recog.p1[2]};
    double um = 0.5 * (u0 + u1);
    double vm = 0.5 * (v0 + v1);
    Point sp = srf.point_at(um, v0);
    Point np = srf.point_at(um, v1);
    double Zs[3] = {np[0] - sp[0], np[1] - sp[1], np[2] - sp[2]};
    double zn = std::sqrt(dot3(Zs, Zs));

    if (zn < 1e-12)
        return {};

    Zs[0] /= zn;
    Zs[1] /= zn;
    Zs[2] /= zn;
    Point P0 = srf.point_at(u0, vm);
    double x0[3] = {P0[0] - C[0], P0[1] - C[1], P0[2] - C[2]};
    double h0 = dot3(x0, Zs);
    double Xs[3] = {x0[0] - h0 * Zs[0], x0[1] - h0 * Zs[1], x0[2] - h0 * Zs[2]};
    double xn = std::sqrt(dot3(Xs, Xs));

    if (xn < 1e-12)
        return {};

    Xs[0] /= xn;
    Xs[1] /= xn;
    Xs[2] /= xn;
    double Ys[3] = {Zs[1] * Xs[2] - Zs[2] * Xs[1], Zs[2] * Xs[0] - Zs[0] * Xs[2], Zs[0] * Xs[1] - Zs[1] * Xs[0]};
    const double PI = 3.14159265358979323846;
    const double TWO_PI = 2.0 * PI;
    const int NT = 128;
    std::vector<double> tu(NT + 1), tlon(NT + 1);

    for (int k = 0; k <= NT; ++k) {
        double u = u0 + range_u * k / NT;
        Point p = srf.point_at(u, vm);
        double r[3] = {p[0] - C[0], p[1] - C[1], p[2] - C[2]};
        double lon = std::atan2(dot3(r, Ys), dot3(r, Xs));

        if (k > 0) {
            while (lon - tlon[k - 1] > PI)
                lon -= TWO_PI;

            while (lon - tlon[k - 1] < -PI)
                lon += TWO_PI;
        }

        tu[k] = u;
        tlon[k] = lon;
    }

    bool lon_incr = tlon[NT] >= tlon[0];
    double lon_lo = std::min(tlon[0], tlon[NT]);
    double lon_hi = std::max(tlon[0], tlon[NT]);

    auto u_from_lon = [&](double lon) -> double {
        while (lon < lon_lo - 1e-9)
            lon += TWO_PI;

        while (lon > lon_hi + 1e-9)
            lon -= TWO_PI;

        int lo = 0;
        int hi = NT;

        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            bool above = lon_incr ? (tlon[mid] < lon) : (tlon[mid] > lon);

            if (above)
                lo = mid;
            else
                hi = mid;
        }

        double denom = tlon[hi] - tlon[lo];
        double f = (std::abs(denom) > 1e-15) ? (lon - tlon[lo]) / denom : 0.0;

        return tu[lo] + (tu[hi] - tu[lo]) * f;
    };

    std::vector<double> tv(NT + 1), th(NT + 1);

    for (int k = 0; k <= NT; ++k) {
        double v = v0 + (v1 - v0) * k / NT;
        Point p = srf.point_at(um, v);
        double r[3] = {p[0] - C[0], p[1] - C[1], p[2] - C[2]};
        tv[k] = v;
        th[k] = dot3(r, Zs);
    }

    bool incr = th[NT] >= th[0];

    if (std::abs(th[NT] - th[0]) < 1e-12)
        return {};

    auto v_from_height = [&](double h) -> double {
        if (incr) {
            if (h <= th[0])
                return tv[0];

            if (h >= th[NT])
                return tv[NT];
        } else {
            if (h >= th[0])
                return tv[0];

            if (h <= th[NT])
                return tv[NT];
        }

        int lo = 0;
        int hi = NT;

        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            bool above = incr ? (th[mid] < h) : (th[mid] > h);

            if (above)
                lo = mid;
            else
                hi = mid;
        }

        double denom = th[hi] - th[lo];
        double f = (std::abs(denom) > 1e-15) ? (h - th[lo]) / denom : 0.0;

        return tv[lo] + (tv[hi] - tv[lo]) * f;
    };

    const std::pair<double, double> domain = c3d.domain();
    const double t0 = domain.first;
    const double t1 = domain.second;

    auto project_t = [&](double t, double& u_out, double& v_out) {
        Point p = c3d.point_at(t);
        double r[3] = {p[0] - C[0], p[1] - C[1], p[2] - C[2]};
        double lon = std::atan2(dot3(r, Ys), dot3(r, Xs));
        double h = dot3(r, Zs);
        double u = u_from_lon(lon);

        for (int np = 0; np < 2; ++np) {
            double du_ = range_u * 1e-7;
            double uc = std::min(std::max(u, u0), u1);
            Point pc0 = srf.point_at(uc, vm);
            double rc0[3] = {pc0[0] - C[0], pc0[1] - C[1], pc0[2] - C[2]};
            double g0 = std::atan2(dot3(rc0, Ys), dot3(rc0, Xs)) - lon;

            while (g0 > PI)
                g0 -= TWO_PI;

            while (g0 < -PI)
                g0 += TWO_PI;

            Point pc1 = srf.point_at(std::min(uc + du_, u1), vm);
            double rc1[3] = {pc1[0] - C[0], pc1[1] - C[1], pc1[2] - C[2]};
            double g1 = std::atan2(dot3(rc1, Ys), dot3(rc1, Xs)) - lon;

            while (g1 > PI)
                g1 -= TWO_PI;

            while (g1 < -PI)
                g1 += TWO_PI;

            double dg = (g1 - g0) / du_;

            if (std::abs(dg) < 1e-12)
                break;

            u = std::min(std::max(uc - g0 / dg, u0), u1);
        }

        double v = v_from_height(h);

        for (int np = 0; np < 2; ++np) {
            double dv_ = (v1 - v0) * 1e-7;
            double vc2 = std::min(std::max(v, std::min(v0, v1)), std::max(v0, v1));
            Point qc0 = srf.point_at(um, vc2);
            double g0 = (qc0[0] - C[0]) * Zs[0] + (qc0[1] - C[1]) * Zs[1] + (qc0[2] - C[2]) * Zs[2] - h;
            Point qc1 = srf.point_at(um, std::min(vc2 + dv_, std::max(v0, v1)));
            double g1 = (qc1[0] - C[0]) * Zs[0] + (qc1[1] - C[1]) * Zs[1] + (qc1[2] - C[2]) * Zs[2] - h;
            double dg = (g1 - g0) / dv_;

            if (std::abs(dg) < 1e-12)
                break;

            v = std::min(std::max(vc2 - g0 / dg, std::min(v0, v1)), std::max(v0, v1));
        }

        u_out = u;
        v_out = v;
    };

    int n = std::max(c3d.cv_count() * 8, 120);
    std::vector<std::array<double, 3>> tuv; // t, u_unwrapped (may leave [u0,u1]), v
    double prev_u = 0.0;

    for (int i = 0; i <= n; ++i) {
        double t = t0 + (t1 - t0) * i / n;
        double u;
        double v;
        project_t(t, u, v);

        if (i > 0) {
            while (u - prev_u > range_u * 0.5)
                u -= range_u;

            while (u - prev_u < -range_u * 0.5)
                u += range_u;
        }

        prev_u = u;
        tuv.push_back({t, u, v});
    }

    std::vector<std::array<double, 3>> uv;
    uv.reserve(tuv.size());

    for (const std::array<double, 3>& e : tuv)
        uv.push_back({e[1], e[2], e[0]});

    if (uv.size() < 2)
        return {};

    std::vector<NurbsCurve> out;
    std::vector<PBNode> seg;

    auto kof = [&](double u) -> int {
        return (int)std::floor((u - u0) / range_u + 1e-9);
    };

    int cur_k = kof(uv[0][0]);
    seg.push_back({uv[0][2], uv[0][0] - cur_k * range_u, uv[0][1]});

    for (size_t i = 1; i < uv.size(); ++i) {
        int ki = kof(uv[i][0]);

        while (ki != cur_k) {
            int step = (ki > cur_k) ? 1 : -1;
            int nk = cur_k + step;
            double seam_cont = u0 + (step > 0 ? nk : cur_k) * range_u;
            double denom = uv[i][0] - uv[i - 1][0];
            double f = (std::abs(denom) > 1e-15) ? (seam_cont - uv[i - 1][0]) / denom : 0.0;
            f = std::min(std::max(f, 0.0), 1.0);
            double vc = uv[i - 1][1] + (uv[i][1] - uv[i - 1][1]) * f;
            double tc = uv[i - 1][2] + (uv[i][2] - uv[i - 1][2]) * f;
            seg.push_back({tc, seam_cont - cur_k * range_u, vc});

            if (seg.size() >= 2)
                out.push_back(emit_pullback_curve(seg));

            seg.clear();
            seg.push_back({tc, seam_cont - nk * range_u, vc});
            cur_k = nk;
        }

        seg.push_back({uv[i][2], uv[i][0] - cur_k * range_u, uv[i][1]});
    }

    if (seg.size() >= 2)
        out.push_back(emit_pullback_curve(seg));

    return out;
}

/// Analytic pull-back of a 3D curve onto a recognized cone or cylinder.
static std::vector<NurbsCurve> analytic_cone_pullback(
    const NurbsSurface& srf,
    const RecogSurface& recog,
    const NurbsCurve& c3d
) {

    if (recog.kind != RecogSurface::CONE && recog.kind != RecogSurface::CYLINDER)
        return {};

    const std::pair<double, double> domain_u = srf.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = srf.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    double range_u = u1 - u0;

    if (range_u < 1e-9)
        return {};

    double A[3] = {recog.p1[0], recog.p1[1], recog.p1[2]};
    double Zc[3] = {recog.p2[0], recog.p2[1], recog.p2[2]};
    double zn = std::sqrt(dot3(Zc, Zc));

    if (zn < 1e-12)
        return {};

    Zc[0] /= zn;
    Zc[1] /= zn;
    Zc[2] /= zn;

    auto height = [&](const Point& p) {
        double r[3] = {p[0] - A[0], p[1] - A[1], p[2] - A[2]};

        return dot3(r, Zc);
    };

    double um = 0.5 * (u0 + u1);
    double h0 = height(srf.point_at(um, v0));
    double h1 = height(srf.point_at(um, v1));

    if (std::abs(h1 - h0) < 1e-12)
        return {};

    auto v_from_height = [&](double h) {
        return v0 + (h - h0) / (h1 - h0) * (v1 - v0);
    };

    double v_ref = (std::abs(h0) >= std::abs(h1)) ? v0 : v1;
    Point P0 = srf.point_at(u0, v_ref);
    double x0[3] = {P0[0] - A[0], P0[1] - A[1], P0[2] - A[2]};
    double hp = dot3(x0, Zc);
    double Xc[3] = {x0[0] - hp * Zc[0], x0[1] - hp * Zc[1], x0[2] - hp * Zc[2]};
    double xn = std::sqrt(dot3(Xc, Xc));

    if (xn < 1e-12)
        return {};

    Xc[0] /= xn;
    Xc[1] /= xn;
    Xc[2] /= xn;
    double Yc[3] = {Zc[1] * Xc[2] - Zc[2] * Xc[1], Zc[2] * Xc[0] - Zc[0] * Xc[2], Zc[0] * Xc[1] - Zc[1] * Xc[0]};
    const double PI = 3.14159265358979323846;
    const double TWO_PI = 2.0 * PI;
    const int NT = 128;
    std::vector<double> tu(NT + 1), tlon(NT + 1);

    for (int k = 0; k <= NT; ++k) {
        double u = u0 + range_u * k / NT;
        Point p = srf.point_at(u, v_ref);
        double r[3] = {p[0] - A[0], p[1] - A[1], p[2] - A[2]};
        double lon = std::atan2(dot3(r, Yc), dot3(r, Xc));

        if (k > 0) {
            while (lon - tlon[k - 1] > PI)
                lon -= TWO_PI;

            while (lon - tlon[k - 1] < -PI)
                lon += TWO_PI;
        }

        tu[k] = u;
        tlon[k] = lon;
    }

    bool lon_incr = tlon[NT] >= tlon[0];
    double lon_lo = std::min(tlon[0], tlon[NT]);
    double lon_hi = std::max(tlon[0], tlon[NT]);

    auto u_from_lon = [&](double lon) -> double {
        while (lon < lon_lo - 1e-9)
            lon += TWO_PI;

        while (lon > lon_hi + 1e-9)
            lon -= TWO_PI;

        int lo = 0;
        int hi = NT;

        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            bool above = lon_incr ? (tlon[mid] < lon) : (tlon[mid] > lon);

            if (above)
                lo = mid;
            else
                hi = mid;
        }

        double denom = tlon[hi] - tlon[lo];
        double f = (std::abs(denom) > 1e-15) ? (lon - tlon[lo]) / denom : 0.0;

        return tu[lo] + (tu[hi] - tu[lo]) * f;
    };

    const std::pair<double, double> domain = c3d.domain();
    const double t0 = domain.first;
    const double t1 = domain.second;
    int n = std::max(c3d.cv_count() * 8, 120);
    double prev_lon_s = 0.0;

    auto project_t = [&](double tq, double& u_out, double& v_out) {
        Point p = c3d.point_at(tq);
        double r[3] = {p[0] - A[0], p[1] - A[1], p[2] - A[2]};
        double rad = std::sqrt(std::max(0.0, dot3(r, Xc) * dot3(r, Xc) + dot3(r, Yc) * dot3(r, Yc)));
        double lon = (rad > 1e-12) ? std::atan2(dot3(r, Yc), dot3(r, Xc)) : prev_lon_s;
        prev_lon_s = lon;
        double u = u_from_lon(lon);

        if (rad > 1e-12)
            for (int np = 0; np < 2; ++np) {
                double du_ = range_u * 1e-7;
                double uc = std::min(std::max(u, u0), u1);
                Point pc0 = srf.point_at(uc, v_ref);
                double rc0[3] = {pc0[0] - A[0], pc0[1] - A[1], pc0[2] - A[2]};
                double g0 = std::atan2(dot3(rc0, Yc), dot3(rc0, Xc)) - lon;

                while (g0 > PI)
                    g0 -= TWO_PI;

                while (g0 < -PI)
                    g0 += TWO_PI;

                Point pc1 = srf.point_at(std::min(uc + du_, u1), v_ref);
                double rc1[3] = {pc1[0] - A[0], pc1[1] - A[1], pc1[2] - A[2]};
                double g1 = std::atan2(dot3(rc1, Yc), dot3(rc1, Xc)) - lon;

                while (g1 > PI)
                    g1 -= TWO_PI;

                while (g1 < -PI)
                    g1 += TWO_PI;

                double dg = (g1 - g0) / du_;

                if (std::abs(dg) < 1e-12)
                    break;

                u = std::min(std::max(uc - g0 / dg, u0), u1);
            }

        u_out = u;
        v_out = v_from_height(dot3(r, Zc));
    };

    std::vector<std::array<double, 3>> tuv;
    double prev_u = 0.0;
    double bmn[3] = {1e300, 1e300, 1e300};
    double bmx[3] = {-1e300, -1e300, -1e300};

    for (int i = 0; i <= n; ++i) {
        double tq = t0 + (t1 - t0) * i / n;
        double u;
        double v;
        project_t(tq, u, v);

        if (i > 0) {
            while (u - prev_u > range_u * 0.5)
                u -= range_u;

            while (u - prev_u < -range_u * 0.5)
                u += range_u;
        }

        prev_u = u;
        tuv.push_back({tq, u, v});
        Point pq = c3d.point_at(tq);

        for (int k = 0; k < 3; ++k) {
            bmn[k] = std::min(bmn[k], pq[k]);
            bmx[k] = std::max(bmx[k], pq[k]);
        }
    }

    std::vector<std::array<double, 3>> uv; // {u_unwrapped, v, t}
    uv.reserve(tuv.size());

    for (const std::array<double, 3>& e : tuv)
        uv.push_back({e[1], e[2], e[0]});

    if (uv.size() < 2)
        return {};

    std::vector<NurbsCurve> out;
    std::vector<PBNode> seg;

    auto kof = [&](double u) -> int {
        return (int)std::floor((u - u0) / range_u + 1e-9);
    };

    int cur_k = kof(uv[0][0]);
    seg.push_back({uv[0][2], uv[0][0] - cur_k * range_u, uv[0][1]});

    for (size_t i = 1; i < uv.size(); ++i) {
        int ki = kof(uv[i][0]);

        while (ki != cur_k) {
            int step = (ki > cur_k) ? 1 : -1;
            int nk = cur_k + step;
            double seam_cont = u0 + (step > 0 ? nk : cur_k) * range_u;
            double denom = uv[i][0] - uv[i - 1][0];
            double f = (std::abs(denom) > 1e-15) ? (seam_cont - uv[i - 1][0]) / denom : 0.0;
            f = std::min(std::max(f, 0.0), 1.0);
            double vc = uv[i - 1][1] + (uv[i][1] - uv[i - 1][1]) * f;
            double tc = uv[i - 1][2] + (uv[i][2] - uv[i - 1][2]) * f;
            seg.push_back({tc, seam_cont - cur_k * range_u, vc});

            if (seg.size() >= 2)
                out.push_back(emit_pullback_curve(seg));

            seg.clear();
            seg.push_back({tc, seam_cont - nk * range_u, vc});
            cur_k = nk;
        }

        seg.push_back({uv[i][2], uv[i][0] - cur_k * range_u, uv[i][1]});
    }

    if (seg.size() >= 2)
        out.push_back(emit_pullback_curve(seg));

    return out;
}

/// Analytic pull-back of a 3D curve onto a recognized torus.
static std::vector<NurbsCurve> analytic_torus_pullback(
    const NurbsSurface& srf,
    const RecogSurface& recog,
    const NurbsCurve& c3d
) {

    if (recog.kind != RecogSurface::TORUS)
        return {};

    const std::pair<double, double> domain_su = srf.domain(0);
    const double su0 = domain_su.first;
    const double su1 = domain_su.second;
    const std::pair<double, double> domain_sv = srf.domain(1);
    const double sv0 = domain_sv.first;
    const double sv1 = domain_sv.second;

    if (su1 - su0 < 1e-9 || sv1 - sv0 < 1e-9)
        return {};

    double C[3] = {recog.p1[0], recog.p1[1], recog.p1[2]};
    double Zc[3] = {recog.p2[0], recog.p2[1], recog.p2[2]};
    double zn = std::sqrt(dot3(Zc, Zc));

    if (zn < 1e-12)
        return {};

    Zc[0] /= zn;
    Zc[1] /= zn;
    Zc[2] /= zn;
    double R = recog.r;
    double rmin = recog.r2;

    if (R < 1e-12 || rmin < 1e-12)
        return {};

    const double PI = 3.14159265358979323846;
    const double TWO_PI = 2.0 * PI;
    double Xc[3];
    {
        Point pf = srf.point_at(su0, sv0);
        double best = -1.0;

        for (int i = 0; i <= 4; ++i)
            for (int j = 0; j <= 4; ++j) {
                Point q = srf.point_at(su0 + (su1 - su0) * i / 4.0, sv0 + (sv1 - sv0) * j / 4.0);
                double rr[3] = {q[0] - C[0], q[1] - C[1], q[2] - C[2]};
                double h = dot3(rr, Zc);
                double px = rr[0] - h * Zc[0];
                double py = rr[1] - h * Zc[1];
                double pz = rr[2] - h * Zc[2];
                double d = px * px + py * py + pz * pz;

                if (d > best) {
                    best = d;
                    pf = q;
                }
            }

        double rr[3] = {pf[0] - C[0], pf[1] - C[1], pf[2] - C[2]};
        double h = dot3(rr, Zc);
        Xc[0] = rr[0] - h * Zc[0];
        Xc[1] = rr[1] - h * Zc[1];
        Xc[2] = rr[2] - h * Zc[2];
        double xn = std::sqrt(dot3(Xc, Xc));

        if (xn < 1e-12)
            return {};

        Xc[0] /= xn;
        Xc[1] /= xn;
        Xc[2] /= xn;
    }

    double Yc[3] = {Zc[1] * Xc[2] - Zc[2] * Xc[1], Zc[2] * Xc[0] - Zc[0] * Xc[2], Zc[0] * Xc[1] - Zc[1] * Xc[0]};

    auto lon_of = [&](const Point& q) {
        double rr[3] = {q[0] - C[0], q[1] - C[1], q[2] - C[2]};

        return std::atan2(dot3(rr, Yc), dot3(rr, Xc));
    };

    auto vhat_of = [&](const Point& q) {
        double rr[3] = {q[0] - C[0], q[1] - C[1], q[2] - C[2]};
        double h = dot3(rr, Zc);
        double px = rr[0] - h * Zc[0];
        double py = rr[1] - h * Zc[1];
        double pz = rr[2] - h * Zc[2];
        double rho = std::sqrt(px * px + py * py + pz * pz);

        return std::atan2(h / rmin, (rho - R) / rmin);
    };

    double um = 0.5 * (su0 + su1);
    double vm = 0.5 * (sv0 + sv1);

    auto wrapd = [&](double d) {
        while (d > PI)
            d -= TWO_PI;

        while (d < -PI)
            d += TWO_PI;

        return std::abs(d);
    };

    double s_u =
        wrapd(lon_of(srf.point_at(su0 + 0.6 * (su1 - su0), vm)) - lon_of(srf.point_at(su0 + 0.3 * (su1 - su0), vm)));

    double s_v =
        wrapd(lon_of(srf.point_at(um, sv0 + 0.6 * (sv1 - sv0))) - lon_of(srf.point_at(um, sv0 + 0.3 * (sv1 - sv0))));

    bool swapped = s_v > s_u;
    double a0 = swapped ? sv0 : su0;
    double a1 = swapped ? sv1 : su1;
    double b0 = swapped ? su0 : sv0;
    double b1 = swapped ? su1 : sv1;
    double range_a = a1 - a0;
    double range_b = b1 - b0;

    auto pt_ab = [&](double a, double b) {
        return swapped ? srf.point_at(b, a) : srf.point_at(a, b);
    };

    double b_ref = b0;
    {
        double best = -1.0;

        for (int j = 0; j <= 16; ++j) {
            double b = b0 + range_b * j / 16.0;
            Point q = pt_ab(0.5 * (a0 + a1), b);
            double rr[3] = {q[0] - C[0], q[1] - C[1], q[2] - C[2]};
            double h = dot3(rr, Zc);
            double px = rr[0] - h * Zc[0];
            double py = rr[1] - h * Zc[1];
            double pz = rr[2] - h * Zc[2];
            double d = px * px + py * py + pz * pz;

            if (d > best) {
                best = d;
                b_ref = b;
            }
        }
    }

    const int NT = 128;
    std::vector<double> ta(NT + 1), tlon(NT + 1), tb(NT + 1), tvh(NT + 1);

    for (int k = 0; k <= NT; ++k) {
        double a = a0 + range_a * k / NT;
        double lon = lon_of(pt_ab(a, b_ref));

        if (k > 0) {
            while (lon - tlon[k - 1] > PI)
                lon -= TWO_PI;

            while (lon - tlon[k - 1] < -PI)
                lon += TWO_PI;
        }

        ta[k] = a;
        tlon[k] = lon;
    }

    double a_ref = 0.5 * (a0 + a1);

    for (int k = 0; k <= NT; ++k) {
        double b = b0 + range_b * k / NT;
        double vh = vhat_of(pt_ab(a_ref, b));

        if (k > 0) {
            while (vh - tvh[k - 1] > PI)
                vh -= TWO_PI;

            while (vh - tvh[k - 1] < -PI)
                vh += TWO_PI;
        }

        tb[k] = b;
        tvh[k] = vh;
    }

    auto inv_table = [&](const std::vector<double>& xs, const std::vector<double>& ys, double y) -> double {
        bool incr = ys[NT] >= ys[0];
        double ylo = std::min(ys[0], ys[NT]);
        double yhi = std::max(ys[0], ys[NT]);

        while (y < ylo - 1e-9)
            y += TWO_PI;

        while (y > yhi + 1e-9)
            y -= TWO_PI;

        int lo = 0;
        int hi = NT;

        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            bool above = incr ? (ys[mid] < y) : (ys[mid] > y);

            if (above)
                lo = mid;
            else
                hi = mid;
        }

        double denom = ys[hi] - ys[lo];
        double f = (std::abs(denom) > 1e-15) ? (y - ys[lo]) / denom : 0.0;

        return xs[lo] + (xs[hi] - xs[lo]) * f;
    };

    auto polish = [&](double x, double y, double xlo, double xhi, double rng, const std::function<double(double)>& yfun)
        -> double {
        for (int np = 0; np < 2; ++np) {
            double dx = rng * 1e-7;
            double xc = std::min(std::max(x, xlo), xhi);
            double g0 = yfun(xc) - y;

            while (g0 > PI)
                g0 -= TWO_PI;

            while (g0 < -PI)
                g0 += TWO_PI;

            double g1 = yfun(std::min(xc + dx, xhi)) - y;

            while (g1 > PI)
                g1 -= TWO_PI;

            while (g1 < -PI)
                g1 += TWO_PI;

            double dg = (g1 - g0) / dx;

            if (std::abs(dg) < 1e-12)
                break;

            x = std::min(std::max(xc - g0 / dg, xlo), xhi);
        }

        return x;
    };

    const std::pair<double, double> domain = c3d.domain();
    const double t0 = domain.first;
    const double t1 = domain.second;
    int n = std::max(c3d.cv_count() * 8, 4000);

    auto project_t = [&](double tq, double& a_out, double& b_out) {
        Point q = c3d.point_at(tq);
        double lon = lon_of(q);
        double vh = vhat_of(q);
        double a = inv_table(ta, tlon, lon);
        double b = inv_table(tb, tvh, vh);

        a = polish(a, lon, a0, a1, range_a, [&](double x) {
            return lon_of(pt_ab(x, b_ref));
        });

        b = polish(b, vh, b0, b1, range_b, [&](double x) {
            return vhat_of(pt_ab(a_ref, x));
        });

        a_out = a;
        b_out = b;
    };

    std::vector<std::array<double, 3>> tab;
    double prev_a = 0.0;
    double prev_b = 0.0;

    for (int i = 0; i <= n; ++i) {
        double tq = t0 + (t1 - t0) * i / n;
        double a;
        double b;
        project_t(tq, a, b);

        if (i > 0) {
            while (a - prev_a > range_a * 0.5)
                a -= range_a;

            while (a - prev_a < -range_a * 0.5)
                a += range_a;

            while (b - prev_b > range_b * 0.5)
                b -= range_b;

            while (b - prev_b < -range_b * 0.5)
                b += range_b;
        }

        prev_a = a;
        prev_b = b;
        tab.push_back({tq, a, b});
    }

    std::vector<std::array<double, 3>> ab; // {a_unwrapped, b_unwrapped, t}
    ab.reserve(tab.size());

    for (const std::array<double, 3>& e : tab)
        ab.push_back({e[1], e[2], e[0]});

    if (ab.size() < 2)
        return {};

    std::vector<NurbsCurve> out;
    std::vector<PBNode> seg;

    auto kof = [](double x, double x0, double rng) {
        return (int)std::floor((x - x0) / rng + 1e-9);
    };

    auto emit = [&](double a, double b, double t, int ka, int kb) {
        double uu = a - ka * range_a;
        double vv = b - kb * range_b;
        seg.push_back(swapped ? PBNode{t, vv, uu} : PBNode{t, uu, vv});
    };

    int ka = kof(ab[0][0], a0, range_a);
    int kb = kof(ab[0][1], b0, range_b);
    emit(ab[0][0], ab[0][1], ab[0][2], ka, kb);

    for (size_t i = 1; i < ab.size(); ++i) {
        double pa = ab[i - 1][0];
        double pb = ab[i - 1][1];
        double pt = ab[i - 1][2];
        double qa = ab[i][0];
        double qb = ab[i][1];
        double qt = ab[i][2];

        for (int guard = 0; guard < 8; ++guard) {
            int kqa = kof(qa, a0, range_a);
            int kqb = kof(qb, b0, range_b);

            if (kqa == ka && kqb == kb)
                break;

            double fa = 2.0;
            double fb = 2.0;
            int sa = 0;
            int sb = 0;

            if (kqa != ka) {
                sa = kqa > ka ? 1 : -1;
                double bound = a0 + (sa > 0 ? ka + 1 : ka) * range_a;
                double den = qa - pa;
                fa = std::abs(den) > 1e-15 ? (bound - pa) / den : 0.0;
            }

            if (kqb != kb) {
                sb = kqb > kb ? 1 : -1;
                double bound = b0 + (sb > 0 ? kb + 1 : kb) * range_b;
                double den = qb - pb;
                fb = std::abs(den) > 1e-15 ? (bound - pb) / den : 0.0;
            }

            if (fa <= fb) {
                double bound = a0 + (sa > 0 ? ka + 1 : ka) * range_a;
                double cf = std::min(std::max(fa, 0.0), 1.0);
                double bv = pb + (qb - pb) * cf;
                double bt = pt + (qt - pt) * cf;
                emit(bound, bv, bt, ka, kb);

                if (seg.size() >= 2)
                    out.push_back(emit_pullback_curve(seg));

                seg.clear();
                ka += sa;
                emit(bound, bv, bt, ka, kb);
                pa = bound;
                pb = bv;
                pt = bt;
            } else {
                double bound = b0 + (sb > 0 ? kb + 1 : kb) * range_b;
                double cf = std::min(std::max(fb, 0.0), 1.0);
                double bu = pa + (qa - pa) * cf;
                double bt = pt + (qt - pt) * cf;
                emit(bu, bound, bt, ka, kb);

                if (seg.size() >= 2)
                    out.push_back(emit_pullback_curve(seg));

                seg.clear();
                kb += sb;
                emit(bu, bound, bt, ka, kb);
                pa = bu;
                pb = bound;
                pt = bt;
            }
        }

        emit(qa, qb, qt, ka, kb);
    }

    if (seg.size() >= 2)
        out.push_back(emit_pullback_curve(seg));

    return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// Coaxial quadric pairs
// ═══════════════════════════════════════════════════════════════════════════

/// Distance of P from the axis through apt along adir.
static double point_axis_dist(const std::array<double, 3>& apt, const std::array<double, 3>& adir, const std::array<double, 3>& P) {

    std::array<double, 3> u = ssi_unit(adir);
    std::array<double, 3> dp{P[0] - apt[0], P[1] - apt[1], P[2] - apt[2]};
    double t = ssi_dot(dp, u);
    std::array<double, 3> perp{dp[0] - t * u[0], dp[1] - t * u[1], dp[2] - t * u[2]};

    return std::sqrt(ssi_dot(perp, perp));
}

/// Coordinate of P along the axis through apt along adir.
static double axial_coord(const std::array<double, 3>& apt, const std::array<double, 3>& adir, const std::array<double, 3>& P) {

    std::array<double, 3> u = ssi_unit(adir);

    return (P[0] - apt[0]) * u[0] + (P[1] - apt[1]) * u[1] + (P[2] - apt[2]) * u[2];
}

/// Whether two axes coincide within tol.
static bool axes_coaxial(const std::array<double, 3>& p1, const std::array<double, 3>& d1, const std::array<double, 3>& p2, const std::array<double, 3>& d2, double tol) {

    std::array<double, 3> u1 = ssi_unit(d1);
    std::array<double, 3> u2 = ssi_unit(d2);
    std::array<double, 3> cx = ssi_cross(u1, u2);

    if (std::sqrt(ssi_dot(cx, cx)) > tol)
        return false;

    return point_axis_dist(p1, u1, p2) <= tol;
}

/// Axial extent of the surface along the cylinder axis.
static void cyl_span(const NurbsSurface& srf, const std::array<double, 3>& apt, const std::array<double, 3>& adir, double& smin, double& smax) {

    std::array<double, 3> u = ssi_unit(adir);
    const std::pair<double, double> domain_u = srf.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = srf.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    double um = 0.5 * (u0 + u1);
    smin = 1e300;
    smax = -1e300;

    for (double vv : {v0, v1}) {
        Point p = srf.point_at(um, vv);
        double s = (p[0] - apt[0]) * u[0] + (p[1] - apt[1]) * u[1] + (p[2] - apt[2]) * u[2];
        smin = std::min(smin, s);
        smax = std::max(smax, s);
    }
}

/// Closest point of two lines, false when parallel.
static bool lines_closest_point(const std::array<double, 3>& p1, const std::array<double, 3>& d1, const std::array<double, 3>& p2, const std::array<double, 3>& d2, double tol, std::array<double, 3>& Pout) {

    std::array<double, 3> u = ssi_unit(d1);
    std::array<double, 3> v = ssi_unit(d2);
    std::array<double, 3> w0{p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]};
    double a = ssi_dot(u, u);
    double b = ssi_dot(u, v);
    double c = ssi_dot(v, v);
    double d = ssi_dot(u, w0);
    double e = ssi_dot(v, w0);
    double den = a * c - b * b;

    if (std::abs(den) < 1e-12)
        return false;

    double sc = (b * e - c * d) / den;
    double tc = (a * e - b * d) / den;
    std::array<double, 3> q1{p1[0] + sc * u[0], p1[1] + sc * u[1], p1[2] + sc * u[2]};
    std::array<double, 3> q2{p2[0] + tc * v[0], p2[1] + tc * v[1], p2[2] + tc * v[2]};
    std::array<double, 3> diff{q1[0] - q2[0], q1[1] - q2[1], q1[2] - q2[2]};

    if (std::sqrt(ssi_dot(diff, diff)) > tol)
        return false;

    Pout = std::array<double, 3>{0.5 * (q1[0] + q2[0]), 0.5 * (q1[1] + q2[1]), 0.5 * (q1[2] + q2[2])};

    return true;
}

/// Coaxial cylinder-sphere section: circles.
static bool ssi_cylinder_sphere(const RecogSurface& cyl, const RecogSurface& sph, std::vector<NurbsCurve>& out) {

    const double kTol = 1e-6;
    std::array<double, 3> P = cyl.p1;
    std::array<double, 3> w = ssi_unit(cyl.p2);
    double rc = cyl.r;
    std::array<double, 3> C = sph.p1;
    double R = sph.r;

    if (point_axis_dist(P, w, C) > kTol)
        return false;

    if (R < rc - kTol)
        return true;

    double dist = std::sqrt(std::max(0.0, R * R - rc * rc));
    std::array<double, 3> xa;
    std::array<double, 3> ya;
    std::tie(xa, ya) = ortho_basis(w);

    if (dist <= kTol) {
        out.push_back(exact_circle(C[0], C[1], C[2], xa, ya, rc));

        return true;
    }

    for (double s : {dist, -dist}) {
        std::array<double, 3> cc{C[0] + s * w[0], C[1] + s * w[1], C[2] + s * w[2]};
        out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rc));
    }

    return true;
}

/// Coaxial cylinder-cone section: circles.
static bool ssi_cylinder_cone(const RecogSurface& cyl, const RecogSurface& cone, std::vector<NurbsCurve>& out) {

    const double kTol = 1e-6;
    std::array<double, 3> Pc = cyl.p1;
    std::array<double, 3> w = ssi_unit(cyl.p2);
    double rc = cyl.r;
    std::array<double, 3> apex = cone.p1;
    std::array<double, 3> a = ssi_unit(cone.p2);
    double alpha = cone.r;

    if (!axes_coaxial(Pc, w, apex, a, kTol))
        return false;

    double ta = std::tan(alpha);

    if (ta < 1e-9)
        return false;

    double s = rc / ta;

    if (s < kTol)
        return true;

    std::array<double, 3> cc{apex[0] + s * a[0], apex[1] + s * a[1], apex[2] + s * a[2]};
    std::array<double, 3> xa;
    std::array<double, 3> ya;
    std::tie(xa, ya) = ortho_basis(a);
    out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rc));

    return true;
}

/// Coaxial cone-sphere section: circles.
static bool ssi_cone_sphere(const RecogSurface& cone, const RecogSurface& sph, std::vector<NurbsCurve>& out) {

    const double kTol = 1e-6;
    std::array<double, 3> apex = cone.p1;
    std::array<double, 3> a = ssi_unit(cone.p2);
    double alpha = cone.r;
    std::array<double, 3> C = sph.p1;
    double R = sph.r;

    if (point_axis_dist(apex, a, C) > kTol)
        return false;

    double dsign = axial_coord(apex, a, C);
    double d = std::abs(dsign);
    std::array<double, 3> dir = (d > kTol && dsign < 0.0) ? std::array<double, 3>{-a[0], -a[1], -a[2]} : a;
    double t = std::tan(alpha);
    double t2 = t * t;
    double A = 1.0 + t2;
    double B = 2.0 * t2 * d;
    double Cq = t2 * d * d - R * R;
    double disc = B * B - 4.0 * A * Cq;

    if (disc < -kTol)
        return true;

    double sq = std::sqrt(std::max(0.0, disc));
    std::vector<double> xs;

    if (sq <= kTol)
        xs = {-B / (2.0 * A)};
    else
        xs = {(-B - sq) / (2.0 * A), (-B + sq) / (2.0 * A)};

    std::array<double, 3> xa;
    std::array<double, 3> ya;
    std::tie(xa, ya) = ortho_basis(a);

    for (double x : xs) {
        double sAx = d + x;

        if (sAx < kTol)
            continue;

        double rr = t * sAx;

        if (rr < kTol)
            continue;

        std::array<double, 3> cc{apex[0] + sAx * dir[0], apex[1] + sAx * dir[1], apex[2] + sAx * dir[2]};
        out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rr));
    }

    return true;
}

/// Cylinder-cylinder section: circles when coaxial, Steinmetz curves when the axes meet.
static bool ssi_cylinder_cylinder(
    const NurbsSurface& sa,
    const RecogSurface& A,
    const NurbsSurface& sb,
    const RecogSurface& B,
    std::vector<NurbsCurve>& out
) {

    const double kTol = 1e-6;
    std::array<double, 3> P1 = A.p1;
    std::array<double, 3> w1 = ssi_unit(A.p2);
    double R1 = A.r;
    std::array<double, 3> P2 = B.p1;
    std::array<double, 3> w2 = ssi_unit(B.p2);
    double R2 = B.r;
    std::array<double, 3> cx = ssi_cross(w1, w2);
    double sinmag = std::sqrt(ssi_dot(cx, cx));

    if (sinmag <= kTol) {
        double dline = point_axis_dist(P1, w1, P2);

        if (dline <= kTol) {
            if (std::abs(R1 - R2) <= kTol)
                return false;

            return true;
        }

        double off = ssi_dot(std::array<double, 3>{P2[0] - P1[0], P2[1] - P1[1], P2[2] - P1[2]}, w1);
        std::array<double, 3> P2p{P2[0] - off * w1[0], P2[1] - off * w1[1], P2[2] - off * w1[2]};
        double d = dline;

        if (d > R1 + R2 + kTol)
            return true;

        if (d < std::abs(R1 - R2) - kTol)
            return true;

        std::array<double, 3> xdir = ssi_unit(std::array<double, 3>{P2p[0] - P1[0], P2p[1] - P1[1], P2p[2] - P1[2]});
        std::array<double, 3> ydir = ssi_unit(ssi_cross(w1, xdir));
        double aa = (R1 * R1 - R2 * R2 + d * d) / (2.0 * d);
        double h = std::sqrt(std::max(0.0, R1 * R1 - aa * aa));
        std::array<double, 3> foot{P1[0] + aa * xdir[0], P1[1] + aa * xdir[1], P1[2] + aa * xdir[2]};
        double s0a;
        double s1a;
        double s0b;
        double s1b;
        cyl_span(sa, P1, w1, s0a, s1a);
        cyl_span(sb, P1, w1, s0b, s1b);
        double slo = std::max(s0a, s0b);
        double shi = std::min(s1a, s1b);

        if (shi - slo <= kTol)
            return true;

        auto emit = [&](const std::array<double, 3>& bp) {
            std::array<double, 3> e0{bp[0] + slo * w1[0], bp[1] + slo * w1[1], bp[2] + slo * w1[2]};
            std::array<double, 3> e1{bp[0] + shi * w1[0], bp[1] + shi * w1[1], bp[2] + shi * w1[2]};
            NurbsCurve ln = NurbsCurve::create(false, 1, {Point(e0[0], e0[1], e0[2]), Point(e1[0], e1[1], e1[2])});
            ln.set_domain(0.0, 1.0);
            out.push_back(ln);
        };

        if (h <= kTol)
            emit(foot);
        else {
            emit(std::array<double, 3>{foot[0] + h * ydir[0], foot[1] + h * ydir[1], foot[2] + h * ydir[2]});
            emit(std::array<double, 3>{foot[0] - h * ydir[0], foot[1] - h * ydir[1], foot[2] - h * ydir[2]});
        }

        return true;
    }

    double Rmax = std::max(R1, R2);

    if (Rmax < 1e-12 || std::abs(R1 - R2) / Rmax > 1e-6)
        return false;

    std::array<double, 3> Pint;

    if (!lines_closest_point(P1, w1, P2, w2, kTol, Pint))
        return false;

    double R = 0.5 * (R1 + R2);
    double ang = std::acos(std::max(-1.0, std::min(1.0, ssi_dot(w1, w2))));
    double sh = std::sin(0.5 * ang);
    double ch = std::cos(0.5 * ang);

    if (sh < 1e-9 || ch < 1e-9)
        return false;

    std::array<double, 3> minor = ssi_unit(cx);
    std::array<double, 3> maj1 = ssi_unit(std::array<double, 3>{w1[0] + w2[0], w1[1] + w2[1], w1[2] + w2[2]});
    std::array<double, 3> maj2 = ssi_unit(std::array<double, 3>{w1[0] - w2[0], w1[1] - w2[1], w1[2] - w2[2]});
    out.push_back(exact_ellipse(Pint[0], Pint[1], Pint[2], maj1, minor, R / sh, R));
    out.push_back(exact_ellipse(Pint[0], Pint[1], Pint[2], maj2, minor, R / ch, R));

    return true;
}

/// Exact circles of a coaxial cylinder-torus pair.
static bool ssi_cylinder_torus(const RecogSurface& cyl, const RecogSurface& tor, std::vector<NurbsCurve>& out) {

    const double kTol = 1e-6;
    std::array<double, 3> P = cyl.p1;
    std::array<double, 3> wc = ssi_unit(cyl.p2);
    double rc = cyl.r;
    std::array<double, 3> C = tor.p1;
    std::array<double, 3> w = ssi_unit(tor.p2);
    double R = tor.r;
    double r = tor.r2;

    if (r >= R - kTol)
        return false;

    if (!axes_coaxial(P, wc, C, w, kTol))
        return false;

    double dr = rc - R;
    double h2 = r * r - dr * dr;

    if (h2 < -kTol)
        return true;

    double h = std::sqrt(std::max(0.0, h2));
    std::array<double, 3> xa;
    std::array<double, 3> ya;
    std::tie(xa, ya) = ortho_basis(w);
    std::vector<double> zs = (h <= kTol) ? std::vector<double>{0.0} : std::vector<double>{h, -h};

    for (double z : zs) {
        std::array<double, 3> cc{C[0] + z * w[0], C[1] + z * w[1], C[2] + z * w[2]};
        out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rc));
    }

    return true;
}

/// Coaxial cone-torus section: circles.
static bool ssi_cone_torus(const RecogSurface& cone, const RecogSurface& tor, std::vector<NurbsCurve>& out) {

    const double kTol = 1e-6;
    std::array<double, 3> apex = cone.p1;
    std::array<double, 3> a = ssi_unit(cone.p2);
    double alpha = cone.r;
    std::array<double, 3> C = tor.p1;
    std::array<double, 3> w = ssi_unit(tor.p2);
    double R = tor.r;
    double r = tor.r2;

    if (r >= R - kTol)
        return false;

    if (!axes_coaxial(apex, a, C, w, kTol))
        return false;

    double t = std::tan(alpha);

    if (t < 1e-9)
        return false;

    double za = axial_coord(C, w, apex);
    double A = t * t + 1.0;
    std::array<double, 3> xa;
    std::array<double, 3> ya;
    std::tie(xa, ya) = ortho_basis(w);

    auto solve_emit = [&](double Rsign) {
        double B = -2.0 * t * (t * za + Rsign);
        double Cc = (t * za + Rsign) * (t * za + Rsign) - r * r;
        double disc = B * B - 4.0 * A * Cc;

        if (disc < -kTol)
            return;

        double sq = std::sqrt(std::max(0.0, disc));
        std::vector<double> zs = (sq <= kTol) ? std::vector<double>{-B / (2.0 * A)}
                                              : std::vector<double>{(-B - sq) / (2.0 * A), (-B + sq) / (2.0 * A)};

        for (double z : zs) {
            double rad = t * std::abs(z - za);

            if (rad < kTol)
                continue;

            std::array<double, 3> cc{C[0] + z * w[0], C[1] + z * w[1], C[2] + z * w[2]};
            out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rad));
        }
    };

    solve_emit(+R);
    solve_emit(-R);

    return true;
}

/// Coaxial sphere-torus section: circles.
static bool ssi_sphere_torus(const RecogSurface& sph, const RecogSurface& tor, std::vector<NurbsCurve>& out) {

    const double kTol = 1e-6;
    std::array<double, 3> S = sph.p1;
    double rsph = sph.r;
    std::array<double, 3> C = tor.p1;
    std::array<double, 3> w = ssi_unit(tor.p2);
    double R = tor.r;
    double r = tor.r2;

    if (r >= R - kTol)
        return false;

    if (point_axis_dist(C, w, S) > kTol)
        return false;

    double zs = axial_coord(C, w, S);
    double d = std::sqrt(R * R + zs * zs);

    if (d < kTol)
        return true;

    if (d - kTol > r + rsph || d + kTol < std::abs(r - rsph))
        return true;

    double aa = 0.5 * (r * r - rsph * rsph + d * d) / d;
    double h = std::sqrt(std::max(0.0, r * r - aa * aa));
    double dirx = (0.0 - R) / d;
    double dirz = (zs - 0.0) / d;
    double phx = R + aa * dirx;
    double phz = aa * dirz;
    double perpx = -dirz;
    double perpz = dirx;
    std::array<double, 3> xa;
    std::array<double, 3> ya;
    std::tie(xa, ya) = ortho_basis(w);
    std::vector<int> signs = (h <= kTol) ? std::vector<int>{0} : std::vector<int>{+1, -1};

    for (int s : signs) {
        double xi = phx + s * h * perpx;
        double z = phz + s * h * perpz;
        double rad = std::abs(xi);

        if (rad < kTol)
            continue;

        std::array<double, 3> cc{C[0] + z * w[0], C[1] + z * w[1], C[2] + z * w[2]};
        out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rad));
    }

    return true;
}

/// Exact spiric loops of two equal parallel-axis tori.
static bool ssi_torus_torus_spiric(const RecogSurface& ta, const RecogSurface& tb, std::vector<NurbsCurve>& out) {

    const double kTol = 1e-6;
    std::array<double, 3> C1 = ta.p1;
    std::array<double, 3> w = ssi_unit(ta.p2);
    double R1 = ta.r;
    double r1 = ta.r2;
    std::array<double, 3> C2 = tb.p1;
    std::array<double, 3> w2 = ssi_unit(tb.p2);
    double R2 = tb.r;
    double r2 = tb.r2;
    std::array<double, 3> cxw = ssi_cross(w, w2);

    if (std::sqrt(ssi_dot(cxw, cxw)) > kTol)
        return false;

    if (std::abs(r1 - r2) > kTol)
        return false;

    if (std::abs(R1 - R2) > kTol)
        return false;

    if (std::abs(axial_coord(C1, w, C2)) > kTol)
        return false;

    std::array<double, 3> dp{C2[0] - C1[0], C2[1] - C1[1], C2[2] - C1[2]};
    double hax = ssi_dot(dp, w);
    std::array<double, 3> ex{dp[0] - hax * w[0], dp[1] - hax * w[1], dp[2] - hax * w[2]};
    double d = std::sqrt(ssi_dot(ex, ex));

    if (d <= kTol)
        return false;

    ex = std::array<double, 3>{ex[0] / d, ex[1] / d, ex[2] / d};
    std::array<double, 3> ey = ssi_cross(w, ex);
    double R = 0.5 * (R1 + R2);
    double r = 0.5 * (r1 + r2);
    double c = 0.5 * d;

    if (std::abs(R - c) <= kTol)
        return false;

    const double PI_ = 3.14159265358979323846;
    const int N = 512;

    auto emit_loops = [&](const std::function<bool(double, double&, double&)>& xy_of_t) {
        for (int sgn : {+1, -1}) {
            std::vector<Point> pts;
            pts.reserve(N);
            bool ok = true;

            for (int k = 0; k < N && ok; ++k) {
                double phi = 2.0 * PI_ * k / N;
                double t = r * std::cos(phi);
                double z = r * std::sin(phi);
                double x;
                double y;

                if (!xy_of_t(t, x, y)) {
                    ok = false;
                    break;
                }

                double yy = sgn * y;

                pts.push_back(Point(
                    C1[0] + x * ex[0] + yy * ey[0] + z * w[0],
                    C1[1] + x * ex[1] + yy * ey[1] + z * w[1],
                    C1[2] + x * ex[2] + yy * ey[2] + z * w[2]
                ));
            }

            if (!ok)
                return;

            NurbsCurve loop = NurbsCurve::create_interpolated(pts, CurveNurbsKnotStyle::ChordPeriodic);

            if (loop.is_valid()) {
                loop.set_domain(0.0, 1.0);
                out.push_back(loop);
            }
        }
    };

    double lo2 = (R - r) * (R - r) - c * c;
    double hi2 = (R + r) * (R + r) - c * c;

    if (hi2 > kTol && lo2 <= kTol)
        return false;

    if (R > c && r >= c - kTol)
        return false;

    if (lo2 > kTol)
        emit_loops([&](double t, double& x, double& y) {
            double rho = R + t;
            double y2 = rho * rho - c * c;

            if (y2 <= 0.0)
                return false;

            x = c;
            y = std::sqrt(y2);

            return true;
        });

    if (R > c + kTol) {
        double be = std::sqrt(R * R - c * c);
        emit_loops([&](double t, double& x, double& y) {
            double g = t / c;

            if (std::abs(g) >= 1.0)
                return false;

            x = c + R * g;
            y = be * std::sqrt(1.0 - g * g);

            return true;
        });
    }

    return true;
}

/// Coaxial torus-torus section: circles.
static bool ssi_torus_torus(const RecogSurface& ta, const RecogSurface& tb, std::vector<NurbsCurve>& out) {

    const double kTol = 1e-6;
    std::array<double, 3> C1 = ta.p1;
    std::array<double, 3> w = ssi_unit(ta.p2);
    double R1 = ta.r;
    double r1 = ta.r2;
    std::array<double, 3> C2 = tb.p1;
    std::array<double, 3> w2 = ssi_unit(tb.p2);
    double R2 = tb.r;
    double r2 = tb.r2;

    if (r1 >= R1 - kTol || r2 >= R2 - kTol)
        return false;

    if (!axes_coaxial(C1, w, C2, w2, kTol))
        return ssi_torus_torus_spiric(ta, tb, out);

    double z2 = axial_coord(C1, w, C2);
    double dxR = R2 - R1;
    double d = std::sqrt(dxR * dxR + z2 * z2);

    if (d < kTol)
        return false;

    if (d - kTol > r1 + r2 || d + kTol < std::abs(r1 - r2))
        return true;

    double aa = 0.5 * (r1 * r1 - r2 * r2 + d * d) / d;
    double h = std::sqrt(std::max(0.0, r1 * r1 - aa * aa));
    double dirx = dxR / d;
    double dirz = z2 / d;
    double phx = R1 + aa * dirx;
    double phz = aa * dirz;
    double perpx = -dirz;
    double perpz = dirx;
    std::array<double, 3> xa;
    std::array<double, 3> ya;
    std::tie(xa, ya) = ortho_basis(w);
    std::vector<int> signs = (h <= kTol) ? std::vector<int>{0} : std::vector<int>{+1, -1};

    for (int s : signs) {
        double xi = phx + s * h * perpx;
        double z = phz + s * h * perpz;
        double rad = std::abs(xi);

        if (rad < kTol)
            continue;

        std::array<double, 3> cc{C1[0] + z * w[0], C1[1] + z * w[1], C1[2] + z * w[2]};
        out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rad));
    }

    return true;
}

/// Exact section of two recognized analytic surfaces, empty when no case applies.
static AnalyticResult analytic_ssi(const NurbsSurface& a, const NurbsSurface& b, double tolerance) {

    AnalyticResult res;
    double rtol = std::max(tolerance, 1e-7) * 1e4;
    RecogSurface ra = recognize_surface(a, rtol);
    RecogSurface rb = recognize_surface(b, rtol);

    if (ra.kind == RecogSurface::NONE || rb.kind == RecogSurface::NONE)
        return res;

    std::vector<NurbsCurve> c3_list;
    bool handled = true;

    auto single = [&](bool ok, NurbsCurve& c3) {
        if (ok)
            c3_list.push_back(c3);
    };

    NurbsCurve c3;

    if (ra.kind == RecogSurface::PLANE && rb.kind == RecogSurface::PLANE) {
        bool empty = false;

        if (ssi_plane_plane(a, ra, b, rb, c3, empty))
            c3_list.push_back(c3);
        else if (!empty)
            return res;
    } else if (ra.kind == RecogSurface::PLANE && rb.kind == RecogSurface::SPHERE)
        single(ssi_plane_sphere(ra, rb, c3), c3);
    else if (ra.kind == RecogSurface::SPHERE && rb.kind == RecogSurface::PLANE)
        single(ssi_plane_sphere(rb, ra, c3), c3);
    else if (ra.kind == RecogSurface::PLANE && rb.kind == RecogSurface::CYLINDER) {
        if (!ssi_plane_cylinder_lines(ra, rb, b, c3_list))
            single(ssi_plane_cylinder(ra, rb, c3), c3);
    } else if (ra.kind == RecogSurface::CYLINDER && rb.kind == RecogSurface::PLANE) {
        if (!ssi_plane_cylinder_lines(rb, ra, a, c3_list))
            single(ssi_plane_cylinder(rb, ra, c3), c3);
    } else if (ra.kind == RecogSurface::PLANE && rb.kind == RecogSurface::CONE)
        handled = ssi_plane_cone(ra, rb, b, c3_list);
    else if (ra.kind == RecogSurface::CONE && rb.kind == RecogSurface::PLANE)
        handled = ssi_plane_cone(rb, ra, a, c3_list);
    else if (ra.kind == RecogSurface::PLANE && rb.kind == RecogSurface::TORUS)
        handled = ssi_plane_torus(ra, rb, c3_list);
    else if (ra.kind == RecogSurface::TORUS && rb.kind == RecogSurface::PLANE)
        handled = ssi_plane_torus(rb, ra, c3_list);
    else if (ra.kind == RecogSurface::SPHERE && rb.kind == RecogSurface::SPHERE) {
        std::array<double, 3> c1 = ra.p1;
        double r1 = ra.r;
        std::array<double, 3> c2 = rb.p1;
        double r2 = rb.r;
        std::array<double, 3> dv{c2[0] - c1[0], c2[1] - c1[1], c2[2] - c1[2]};
        double dist = std::sqrt(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]);
        double tan_tol = (r1 + r2) * 1e-9;

        if (1e-12 < dist && dist < r1 + r2 - tan_tol && dist > std::abs(r1 - r2) + tan_tol) {
            std::array<double, 3> nu{dv[0] / dist, dv[1] / dist, dv[2] / dist};
            double aa = (dist * dist + r1 * r1 - r2 * r2) / (2.0 * dist);
            double rr2 = r1 * r1 - aa * aa;

            if (rr2 > 0.0) {
                std::array<double, 3> cc{c1[0] + aa * nu[0], c1[1] + aa * nu[1], c1[2] + aa * nu[2]};
                std::array<double, 3> xa;
                std::array<double, 3> ya;
                std::tie(xa, ya) = ortho_basis(nu);
                c3 = exact_circle(cc[0], cc[1], cc[2], xa, ya, std::sqrt(rr2));
                c3_list.push_back(c3);
            }
        }
    } else if (ra.kind == RecogSurface::CYLINDER && rb.kind == RecogSurface::SPHERE)
        handled = ssi_cylinder_sphere(ra, rb, c3_list);
    else if (ra.kind == RecogSurface::SPHERE && rb.kind == RecogSurface::CYLINDER)
        handled = ssi_cylinder_sphere(rb, ra, c3_list);
    else if (ra.kind == RecogSurface::CYLINDER && rb.kind == RecogSurface::CONE)
        handled = ssi_cylinder_cone(ra, rb, c3_list);
    else if (ra.kind == RecogSurface::CONE && rb.kind == RecogSurface::CYLINDER)
        handled = ssi_cylinder_cone(rb, ra, c3_list);
    else if (ra.kind == RecogSurface::CONE && rb.kind == RecogSurface::SPHERE)
        handled = ssi_cone_sphere(ra, rb, c3_list);
    else if (ra.kind == RecogSurface::SPHERE && rb.kind == RecogSurface::CONE)
        handled = ssi_cone_sphere(rb, ra, c3_list);
    else if (ra.kind == RecogSurface::CYLINDER && rb.kind == RecogSurface::CYLINDER)
        handled = ssi_cylinder_cylinder(a, ra, b, rb, c3_list);
    else if (ra.kind == RecogSurface::CYLINDER && rb.kind == RecogSurface::TORUS)
        handled = ssi_cylinder_torus(ra, rb, c3_list);
    else if (ra.kind == RecogSurface::TORUS && rb.kind == RecogSurface::CYLINDER)
        handled = ssi_cylinder_torus(rb, ra, c3_list);
    else if (ra.kind == RecogSurface::CONE && rb.kind == RecogSurface::TORUS)
        handled = ssi_cone_torus(ra, rb, c3_list);
    else if (ra.kind == RecogSurface::TORUS && rb.kind == RecogSurface::CONE)
        handled = ssi_cone_torus(rb, ra, c3_list);
    else if (ra.kind == RecogSurface::SPHERE && rb.kind == RecogSurface::TORUS)
        handled = ssi_sphere_torus(ra, rb, c3_list);
    else if (ra.kind == RecogSurface::TORUS && rb.kind == RecogSurface::SPHERE)
        handled = ssi_sphere_torus(rb, ra, c3_list);
    else if (ra.kind == RecogSurface::TORUS && rb.kind == RecogSurface::TORUS)
        handled = ssi_torus_torus(ra, rb, c3_list);
    else {
        return res;
    }

    if (!handled)
        return res;

    for (const NurbsCurve& cc3 : c3_list) {
        NurbsCurve pa = analytic_pcurve(a, ra, cc3);
        NurbsCurve pb = analytic_pcurve(b, rb, cc3);

        if (!pa.is_valid() && ra.kind == RecogSurface::TORUS) {
            std::vector<NurbsCurve> v = analytic_torus_pullback(a, ra, cc3);

            if (!v.empty())
                pa = v[0];
        }

        if (!pb.is_valid() && rb.kind == RecogSurface::TORUS) {
            std::vector<NurbsCurve> v = analytic_torus_pullback(b, rb, cc3);

            if (!v.empty())
                pb = v[0];
        }

        if (!pa.is_valid() && ra.kind == RecogSurface::SPHERE) {
            std::vector<NurbsCurve> v = analytic_sphere_pullback(a, ra, cc3);

            if (!v.empty())
                pa = v[0];
        }

        if (!pb.is_valid() && rb.kind == RecogSurface::SPHERE) {
            std::vector<NurbsCurve> v = analytic_sphere_pullback(b, rb, cc3);

            if (!v.empty())
                pb = v[0];
        }

        if (!pa.is_valid() && (ra.kind == RecogSurface::CONE || ra.kind == RecogSurface::CYLINDER)) {
            std::vector<NurbsCurve> v = analytic_cone_pullback(a, ra, cc3);

            if (!v.empty())
                pa = v[0];
        }

        if (!pb.is_valid() && (rb.kind == RecogSurface::CONE || rb.kind == RecogSurface::CYLINDER)) {
            std::vector<NurbsCurve> v = analytic_cone_pullback(b, rb, cc3);

            if (!v.empty())
                pb = v[0];
        }

        if (!pa.is_valid()) {
            std::vector<NurbsCurve> v = Closest::surface_curve(a, cc3);

            if (!v.empty())
                pa = v[0];
        }

        if (!pb.is_valid()) {
            std::vector<NurbsCurve> v = Closest::surface_curve(b, cc3);

            if (!v.empty())
                pb = v[0];
        }

        if (pa.is_valid() && pb.is_valid())
            res.triples.push_back(std::make_tuple(cc3, pa, pb));
    }

    res.status = AnalyticResult::HIT;

    return res;
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// NURBS surfaces
// ═══════════════════════════════════════════════════════════════════════════

std::vector<NurbsCurve> Intersection::surface_plane(const NurbsSurface& surface, const Plane& plane, double tolerance) {

    if (!surface.is_valid())
        return {};

    if (tolerance <= 0.0)
        tolerance = Tolerance::ZERO_TOLERANCE;

    const SurfacePlaneTraceResult traced = surface_plane_traces(surface, plane, tolerance);
    const double step = traced.step;
    const double uv_to_3d = traced.uv_to_3d;
    const double uv_to_3d_min = traced.uv_to_3d_min;

    std::vector<NurbsCurve> result;

    for (const SurfacePlaneTrace& trace : traced.traces) {
        const std::vector<std::pair<double, double>>& uv_trace = trace.uv_trace;
        const bool is_loop = trace.is_loop;
        std::vector<Point> all_pts(uv_trace.size());

        for (size_t i = 0; i < uv_trace.size(); i++)
            all_pts[i] = surface.point_at(uv_trace[i].first, uv_trace[i].second);

        NurbsCurve crv = surface_plane_fit_3d(all_pts, is_loop, plane, step, uv_to_3d, uv_to_3d_min);

        if (!crv.is_valid())
            continue;

        const std::pair<double, double> domain_ct = crv.domain();
        const double ct0 = domain_ct.first;
        const double ct1 = domain_ct.second;
        double dup_tol = step * uv_to_3d * 3.0;
        bool dup = false;

        for (NurbsCurve& existing : result) {
            const std::pair<double, double> domain_et = existing.domain();
            const double et0 = domain_et.first;
            const double et1 = domain_et.second;
            bool all_close = true;

            for (double f : {0.25, 0.5, 0.75}) {
                Point cp = crv.point_at(ct0 + (ct1 - ct0) * f);
                Point ep = existing.point_at(et0 + (et1 - et0) * f);
                Point em = existing.point_at((et0 + et1) * 0.5);
                double d = std::min(cp.distance(ep), cp.distance(em));

                if (d > dup_tol) {
                    all_close = false;
                    break;
                }
            }

            if (all_close) {
                dup = true;
                break;
            }
        }

        if (!dup)
            result.push_back(std::move(crv));
    }

    return result;
}

std::vector<std::pair<NurbsCurve, NurbsCurve>> Intersection::surface_plane_uv(
    const NurbsSurface& surface,
    const Plane& plane,
    double tolerance
) {

    if (!surface.is_valid())
        return {};

    if (tolerance <= 0.0)
        tolerance = Tolerance::ZERO_TOLERANCE;

    const std::pair<double, double> domain_u = surface.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = surface.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    double range_u = u1 - u0;
    double range_v = v1 - v0;
    bool closed_u = surface.is_closed(0);
    bool closed_v = surface.is_closed(1);

    auto wrap_u = [&](double u) -> double {
        if (closed_u) {
            double t = std::fmod(u - u0, range_u);

            if (t < 0)
                t += range_u;

            return u0 + t;
        }

        return std::max(u0, std::min(u, u1));
    };

    auto wrap_v = [&](double v) -> double {
        if (closed_v) {
            double t = std::fmod(v - v0, range_v);

            if (t < 0)
                t += range_v;

            return v0 + t;
        }

        return std::max(v0, std::min(v, v1));
    };

    Vector pn = plane.z_axis();
    Point p0 = plane.origin();

    auto g_and_grad = [&](double u, double v, double& val, double& gu, double& gv) {
        const std::vector<Vector> derivs = surface.evaluate(wrap_u(u), wrap_v(v), 1);
        const Vector& S = derivs[0];
        const Vector& Su = derivs[2];
        const Vector& Sv = derivs[1];
        val = (S[0] - p0[0]) * pn[0] + (S[1] - p0[1]) * pn[1] + (S[2] - p0[2]) * pn[2];
        gu = Su[0] * pn[0] + Su[1] * pn[1] + Su[2] * pn[2];
        gv = Sv[0] * pn[0] + Sv[1] * pn[1] + Sv[2] * pn[2];
    };

    auto seam_newton = [&](double cu, double cv_, int axis) -> std::pair<double, double> {
        for (int iter = 0; iter < 10; iter++) {
            double val;
            double gu;
            double gv;
            g_and_grad(cu, cv_, val, gu, gv);

            if (std::abs(val) < tolerance)
                break;

            if (axis == 0) {
                if (std::abs(gv) < 1e-14)
                    break;

                cv_ = cv_ - val / gv;
            } else {
                if (std::abs(gu) < 1e-14)
                    break;

                cu = cu - val / gu;
            }
        }

        return {cu, cv_};
    };

    const SurfacePlaneTraceResult traced = surface_plane_traces(surface, plane, tolerance);
    const double step = traced.step;
    const double uv_to_3d = traced.uv_to_3d;
    const double uv_to_3d_min = traced.uv_to_3d_min;

    double fit_tol = step * (uv_to_3d + uv_to_3d_min) * 0.5;
    double dup_tol = step * uv_to_3d * 3.0;

    std::vector<std::pair<NurbsCurve, NurbsCurve>> result;
    std::vector<std::vector<Point>> kept_pts3;

    for (const SurfacePlaneTrace& trace : traced.traces) {
        const std::vector<std::pair<double, double>>& uv_trace = trace.uv_trace;
        const std::vector<std::pair<double, double>>& uv_unwrapped = trace.uv_unwrapped;
        const bool is_loop = trace.is_loop;
        int m = (int)uv_trace.size();
        std::vector<Point> trace_pts3(m);

        for (int i = 0; i < m; i++)
            trace_pts3[i] = surface.point_at(uv_trace[i].first, uv_trace[i].second);

        bool dup = false;

        for (std::vector<Point>& other : kept_pts3) {
            bool all_close = true;

            for (double f : {0.25, 0.5, 0.75}) {
                Point cp = trace_pts3[(int)((m - 1) * f)];
                double dmin = dup_tol + 1.0;

                for (size_t k = 0; k < other.size(); k += 5)
                    dmin = std::min(dmin, cp.distance(other[k]));

                if (dmin > dup_tol) {
                    all_close = false;
                    break;
                }
            }

            if (all_close) {
                dup = true;
                break;
            }
        }

        if (dup)
            continue;

        kept_pts3.push_back(trace_pts3);

        std::vector<std::pair<double, double>> pts = uv_unwrapped;
        double closure_du = 0.0;
        double closure_dv = 0.0;

        if (is_loop && pts.size() >= 2) {
            double du_j = pts[0].first - pts.back().first;
            double dv_j = pts[0].second - pts.back().second;

            if (closed_u) {
                while (du_j > range_u * 0.5)
                    du_j -= range_u;

                while (du_j < -range_u * 0.5)
                    du_j += range_u;
            }

            if (closed_v) {
                while (dv_j > range_v * 0.5)
                    dv_j -= range_v;

                while (dv_j < -range_v * 0.5)
                    dv_j += range_v;
            }

            closure_du = (pts.back().first + du_j) - pts[0].first;
            closure_dv = (pts.back().second + dv_j) - pts[0].second;
            pts.push_back({pts[0].first + closure_du, pts[0].second + closure_dv});
        }

        std::vector<std::pair<double, double>> out_pts;
        out_pts.push_back(pts[0]);
        std::vector<int> cross_idx;

        for (size_t i = 1; i < pts.size(); i++) {
            std::pair<double, double> pa = pts[i - 1];
            std::pair<double, double> pb = pts[i];
            std::vector<std::tuple<double, int, double>> crossings;

            if (closed_u && std::abs(pb.first - pa.first) > 1e-15) {
                int k0 = (int)std::floor((pa.first - u0) / range_u);
                int k1 = (int)std::floor((pb.first - u0) / range_u);

                for (int k = std::min(k0, k1) + 1; k <= std::max(k0, k1); k++) {
                    double L = u0 + k * range_u;
                    double t = (L - pa.first) / (pb.first - pa.first);

                    if (0.0 < t && t < 1.0)
                        crossings.push_back({t, 0, L});
                }
            }

            if (closed_v && std::abs(pb.second - pa.second) > 1e-15) {
                int k0 = (int)std::floor((pa.second - v0) / range_v);
                int k1 = (int)std::floor((pb.second - v0) / range_v);

                for (int k = std::min(k0, k1) + 1; k <= std::max(k0, k1); k++) {
                    double L = v0 + k * range_v;
                    double t = (L - pa.second) / (pb.second - pa.second);

                    if (0.0 < t && t < 1.0)
                        crossings.push_back({t, 1, L});
                }
            }

            std::sort(crossings.begin(), crossings.end());

            for (const std::tuple<double, int, double>& crossing : crossings) {
                const double t = std::get<0>(crossing);
                const int axis = std::get<1>(crossing);
                const double L = std::get<2>(crossing);
                double cu = pa.first + (pb.first - pa.first) * t;
                double cv_ = pa.second + (pb.second - pa.second) * t;

                if (axis == 0) {
                    std::pair<double, double> r = seam_newton(L, cv_, 0);
                    cu = L;
                    cv_ = r.second;
                } else {
                    std::pair<double, double> r = seam_newton(cu, L, 1);
                    cu = r.first;
                    cv_ = L;
                }

                out_pts.push_back({cu, cv_});
                cross_idx.push_back((int)out_pts.size() - 1);
            }

            out_pts.push_back({pb.first, pb.second});

            if (i < pts.size() - 1) {
                bool on_seam = false;

                if (closed_u) {
                    double k = std::round((pb.first - u0) / range_u);
                    double L = u0 + k * range_u;

                    if (std::abs(pb.first - L) < range_u * 1e-9 && std::abs(pb.first - pa.first) > range_u * 1e-9) {
                        out_pts.back().first = L;
                        on_seam = true;
                    }
                }

                if (closed_v) {
                    double k = std::round((pb.second - v0) / range_v);
                    double L = v0 + k * range_v;

                    if (std::abs(pb.second - L) < range_v * 1e-9 && std::abs(pb.second - pa.second) > range_v * 1e-9) {
                        out_pts.back().second = L;
                        on_seam = true;
                    }
                }

                if (on_seam)
                    cross_idx.push_back((int)out_pts.size() - 1);
            }
        }

        bool wrap_drift = std::fabs(closure_du) > range_u * 0.5 || std::fabs(closure_dv) > range_v * 0.5;
        std::vector<std::pair<std::vector<std::pair<double, double>>, bool>> pieces;

        if (cross_idx.size() == 0) {
            pieces.push_back({out_pts, is_loop && !wrap_drift});
        } else if (is_loop) {
            for (size_t ci = 0; ci + 1 < cross_idx.size(); ci++) {
                int a = cross_idx[ci];
                int b = cross_idx[ci + 1];

                pieces.push_back(
                    {std::vector<std::pair<double, double>>(out_pts.begin() + a, out_pts.begin() + b + 1), false}
                );
            }

            std::vector<std::pair<double, double>> wrap_piece(out_pts.begin() + cross_idx.back(), out_pts.end());

            for (int pi = 1; pi <= cross_idx[0]; pi++)
                wrap_piece.push_back({out_pts[pi].first + closure_du, out_pts[pi].second + closure_dv});

            pieces.push_back({wrap_piece, false});
        } else {
            std::vector<int> bounds;
            bounds.push_back(0);

            for (int ci : cross_idx)
                bounds.push_back(ci);

            bounds.push_back((int)out_pts.size() - 1);

            for (size_t bi = 0; bi + 1 < bounds.size(); bi++) {
                int a = bounds[bi];
                int b = bounds[bi + 1];

                if (b > a)
                    pieces.push_back(
                        {std::vector<std::pair<double, double>>(out_pts.begin() + a, out_pts.begin() + b + 1), false}
                    );
            }
        }

        for (std::pair<std::vector<std::pair<double, double>>, bool>& piece : pieces) {
            std::vector<std::pair<double, double>>& piece_pts = piece.first;
            const bool piece_loop = piece.second;

            if (piece_pts.size() < 2)
                continue;

            std::pair<double, double> mid = piece_pts[piece_pts.size() / 2];

            if (closed_u) {
                int k_u = (int)std::floor((mid.first - u0) / range_u);

                if (k_u != 0)
                    for (std::pair<double, double>& p : piece_pts)
                        p.first -= k_u * range_u;
            }

            if (closed_v) {
                int k_v = (int)std::floor((mid.second - v0) / range_v);

                if (k_v != 0)
                    for (std::pair<double, double>& p : piece_pts)
                        p.second -= k_v * range_v;
            }

            std::vector<Point> pts_uv;
            pts_uv.reserve(piece_pts.size() * 4);
            {
                auto polish_uv = [&](double& uu, double& vv) -> bool {
                    for (int iter = 0; iter < 8; iter++) {
                        double val;
                        double gu;
                        double gv;
                        g_and_grad(uu, vv, val, gu, gv);

                        if (std::abs(val) < 1e-12)
                            return true;

                        double mag2 = gu * gu + gv * gv;

                        if (mag2 < 1e-28)
                            return false;

                        uu -= val * gu / mag2;
                        vv -= val * gv / mag2;
                    }

                    return true;
                };

                std::function<void(double, double, double, double, int)> densify =
                    [&](double au, double av, double bu, double bv, int depth) {
                        double mu = 0.5 * (au + bu);
                        double mv = 0.5 * (av + bv);
                        double cu = mu;
                        double cv2 = mv;

                        if (!polish_uv(cu, cv2))
                            return;

                        double sag = std::hypot(cu - mu, cv2 - mv);

                        if (sag > step * 1e-4 && depth < 4) {
                            densify(au, av, cu, cv2, depth + 1);
                            pts_uv.push_back(Point(cu, cv2, 0.0));
                            densify(cu, cv2, bu, bv, depth + 1);
                        } else {
                            pts_uv.push_back(Point(cu, cv2, 0.0));
                        }
                    };

                for (size_t i = 1; i < piece_pts.size(); i++) {
                    pts_uv.push_back(Point(piece_pts[i - 1].first, piece_pts[i - 1].second, 0.0));

                    densify(
                        piece_pts[i - 1].first,
                        piece_pts[i - 1].second,
                        piece_pts[i].first,
                        piece_pts[i].second,
                        0
                    );
                }

                pts_uv.push_back(Point(piece_pts.back().first, piece_pts.back().second, 0.0));
            }

            std::vector<Point> pts3(pts_uv.size());

            for (size_t i = 0; i < pts_uv.size(); i++)
                pts3[i] = surface.point_at(wrap_u(pts_uv[i][0]), wrap_v(pts_uv[i][1]));

            NurbsCurve crv3 = surface_plane_fit_3d(pts3, piece_loop, plane, step, uv_to_3d, uv_to_3d_min, false);

            if (!crv3.is_valid())
                crv3 = piece_loop ? NurbsCurve::create_interpolated(pts3, CurveNurbsKnotStyle::ChordPeriodic)
                                  : NurbsCurve::create_interpolated(pts3);

            if (!crv3.is_valid())
                continue;

            int mp = (int)pts_uv.size();
            double fit_tol_uv = step * 2e-3;
            double total_turning = 0;

            for (int i = 1; i < mp - 1; i++) {
                double dx1 = pts_uv[i][0] - pts_uv[i - 1][0];
                double dy1 = pts_uv[i][1] - pts_uv[i - 1][1];
                double dx2 = pts_uv[i + 1][0] - pts_uv[i][0];
                double dy2 = pts_uv[i + 1][1] - pts_uv[i][1];
                double l1 = std::hypot(dx1, dy1);
                double l2 = std::hypot(dx2, dy2);

                if (l1 > 1e-14 && l2 > 1e-14) {
                    double c = (dx1 * dx2 + dy1 * dy2) / (l1 * l2);
                    c = std::max(-1.0, std::min(1.0, c));
                    total_turning += std::acos(c);
                }
            }

            std::vector<double> chords(mp, 0.0);
            double total_len = 0;

            for (int i = 1; i < mp; i++) {
                total_len += pts_uv[i].distance(pts_uv[i - 1]);
                chords[i] = total_len;
            }

            if (piece_loop && mp > 1)
                total_len += pts_uv[0].distance(pts_uv[mp - 1]);

            if (total_len > 1e-14)
                for (int i = 1; i < mp; i++)
                    chords[i] /= total_len;

            int target_cvs = std::max(8, (int)(total_turning / 0.5) + 6);
            int max_cvs = std::min(mp - 1, 96);
            NurbsCurve pcurve;
            double pcurve_dev = 1e300;

            for (int attempt = 0; attempt < 6; attempt++) {
                if (target_cvs > max_cvs)
                    break;

                NurbsCurve cand = NurbsCurve::create_fitted(pts_uv, target_cvs, 3, piece_loop);

                if (!cand.is_valid())
                    break;

                const std::pair<double, double> domain_ft = cand.domain();
                const double ft0 = domain_ft.first;
                const double ft1 = domain_ft.second;
                double max_dev = 0;

                for (int i = 0; i < mp; i++) {
                    double t = ft0 + (ft1 - ft0) * chords[i];
                    max_dev = std::max(max_dev, cand.point_at(t).distance(pts_uv[i]));
                }

                if (max_dev < pcurve_dev) {
                    pcurve_dev = max_dev;
                    pcurve = cand;
                }

                if (max_dev < fit_tol_uv)
                    break;

                target_cvs = std::min(target_cvs * 2, max_cvs + 1);
            }

            if (!pcurve.is_valid())
                pcurve = piece_loop ? NurbsCurve::create_interpolated(pts_uv, CurveNurbsKnotStyle::ChordPeriodic)
                                    : NurbsCurve::create_interpolated(pts_uv);

            if (!pcurve.is_valid())
                continue;

            crv3.set_domain(0.0, 1.0);
            pcurve.set_domain(0.0, 1.0);

            double vali_tol = std::max(10.0 * tolerance, fit_tol * 2.0);
            double max_off = 0;

            for (int i = 0; i < 17; i++) {
                double t = i / 16.0;
                Point pc = pcurve.point_at(t);
                double val;
                double gu;
                double gv;
                g_and_grad(pc[0], pc[1], val, gu, gv);
                max_off = std::max(max_off, std::abs(val));
            }

            if (max_off > vali_tol && target_cvs * 2 <= max_cvs) {
                NurbsCurve refit = NurbsCurve::create_fitted(pts_uv, target_cvs * 2, 3, piece_loop);

                if (refit.is_valid()) {
                    refit.set_domain(0.0, 1.0);
                    pcurve = refit;
                }
            }

            result.push_back({std::move(crv3), std::move(pcurve)});
        }
    }

    return result;
}

/// Drop near-zero-length section curves.
static void drop_point_sections(std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>>& trs, double tolerance) {

    double min_len = std::max(tolerance * 10.0, 1e-9);

    trs.erase(
        std::remove_if(
            trs.begin(),
            trs.end(),
            [&](const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t) {
                return std::get<0>(t).length() < min_len;
            }
        ),
        trs.end()
    );
}

std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> Intersection::surface_surface(
    const NurbsSurface& a,
    const NurbsSurface& b,
    double tolerance
) {

    if (!a.is_valid() || !b.is_valid())
        return {};

    if (tolerance <= 0.0)
        tolerance = Tolerance::ZERO_TOLERANCE;

    AnalyticResult _ana = analytic_ssi(a, b, tolerance);

    if (_ana.status != AnalyticResult::NOT_ANALYTIC) {
        drop_point_sections(_ana.triples, tolerance);

        return _ana.triples;
    }

    auto plane_from = [](const NurbsSurface& srf) -> Plane {
        const std::pair<double, double> domain_s = srf.domain(0);
        const double s0 = domain_s.first;
        const double s1 = domain_s.second;
        const std::pair<double, double> domain = srf.domain(1);
        const double t0 = domain.first;
        const double t1 = domain.second;
        Point po = srf.point_at((s0 + s1) * 0.5, (t0 + t1) * 0.5);
        Vector nn = srf.normal_at((s0 + s1) * 0.5, (t0 + t1) * 0.5);
        Vector nv(nn[0], nn[1], nn[2]);

        return Plane::from_point_normal(po, nv);
    };

    if (a.is_planar(nullptr, 1e-9)) {
        Plane plane = plane_from(a);
        std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> result;

        for (const std::pair<NurbsCurve, NurbsCurve>& section : surface_plane_uv(b, plane, tolerance)) {
            const NurbsCurve& c3 = section.first;
            const NurbsCurve& pb = section.second;
            std::vector<NurbsCurve> pas = Closest::surface_curve(a, c3);

            if (pas.size() == 1)
                result.push_back({c3, pas[0], pb});
        }

        drop_point_sections(result, tolerance);

        return result;
    }

    if (b.is_planar(nullptr, 1e-9)) {
        Plane plane = plane_from(b);
        std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> result;

        for (const std::pair<NurbsCurve, NurbsCurve>& section : surface_plane_uv(a, plane, tolerance)) {
            const NurbsCurve& c3 = section.first;
            const NurbsCurve& pa = section.second;
            std::vector<NurbsCurve> pbs = Closest::surface_curve(b, c3);

            if (pbs.size() == 1)
                result.push_back({c3, pa, pbs[0]});
        }

        drop_point_sections(result, tolerance);

        return result;
    }

    const std::pair<double, double> domain_au = a.domain(0);
    const double au0 = domain_au.first;
    const double au1 = domain_au.second;
    const std::pair<double, double> domain_av = a.domain(1);
    const double av0 = domain_av.first;
    const double av1 = domain_av.second;
    const std::pair<double, double> domain_bu = b.domain(0);
    const double bu0 = domain_bu.first;
    const double bu1 = domain_bu.second;
    const std::pair<double, double> domain_bv = b.domain(1);
    const double bv0 = domain_bv.first;
    const double bv1 = domain_bv.second;
    double a_range_u = au1 - au0;
    double a_range_v = av1 - av0;
    double b_range_u = bu1 - bu0;
    double b_range_v = bv1 - bv0;
    bool a_closed_u = a.is_closed(0);
    bool a_closed_v = a.is_closed(1);
    bool b_closed_u = b.is_closed(0);
    bool b_closed_v = b.is_closed(1);

    auto make_wrap = [](double c0, double c1, double rng, bool closed) {
        return [c0, c1, rng, closed](double t) -> double {
            if (closed) {
                double f = std::fmod(t - c0, rng);

                if (f < 0)
                    f += rng;

                return c0 + f;
            }

            return std::max(c0, std::min(t, c1));
        };
    };

    auto a_wrap_u = make_wrap(au0, au1, a_range_u, a_closed_u);
    auto a_wrap_v = make_wrap(av0, av1, a_range_v, a_closed_v);
    auto b_wrap_u = make_wrap(bu0, bu1, b_range_u, b_closed_u);
    auto b_wrap_v = make_wrap(bv0, bv1, b_range_v, b_closed_v);

    auto eval_a = [&](double u, double v, Vector& S, Vector& Su, Vector& Sv) {
        const std::vector<Vector> d = a.evaluate(a_wrap_u(u), a_wrap_v(v), 1);
        S = d[0];
        Su = d[2];
        Sv = d[1];
    };

    auto eval_b = [&](double u, double v, Vector& S, Vector& Su, Vector& Sv) {
        const std::vector<Vector> d = b.evaluate(b_wrap_u(u), b_wrap_v(v), 1);
        S = d[0];
        Su = d[2];
        Sv = d[1];
    };

    std::vector<double> spans_au = a.get_span_vector(0);
    std::vector<double> spans_av = a.get_span_vector(1);
    std::vector<double> spans_bu = b.get_span_vector(0);
    std::vector<double> spans_bv = b.get_span_vector(1);
    int a_nu = std::max((int)spans_au.size() - 1, 1) * 4;
    int a_nv = std::max((int)spans_av.size() - 1, 1) * 4;
    int b_nu = std::max((int)spans_bu.size() - 1, 1) * 4;
    int b_nv = std::max((int)spans_bv.size() - 1, 1) * 4;
    double a_du = a_range_u / a_nu;
    double a_dv = a_range_v / a_nv;
    double b_du = b_range_u / b_nu;
    double b_dv = b_range_v / b_nv;


    auto cell_boxes = [&](const NurbsSurface& srf, double c0u, double dcu, int ncu, double c0v, double dcv, int ncv)
        -> std::vector<std::array<double, 8>> {
        std::vector<std::vector<Point>> S;

        for (int i = 0; i < 2 * ncu + 1; i++) {
            std::vector<Point> row;

            for (int j = 0; j < 2 * ncv + 1; j++)
                row.push_back(srf.point_at(c0u + dcu * 0.5 * i, c0v + dcv * 0.5 * j));

            S.push_back(row);
        }

        std::vector<std::array<double, 8>> boxes;

        for (int ci = 0; ci < ncu; ci++) {
            for (int cj = 0; cj < ncv; cj++) {
                double minx = std::numeric_limits<double>::infinity();
                double miny = minx;
                double minz = minx;
                double maxx = -minx;
                double maxy = -minx;
                double maxz = -minx;

                for (int i = 2 * ci; i < 2 * ci + 3; i++) {
                    for (int j = 2 * cj; j < 2 * cj + 3; j++) {
                        const Point& p = S[i][j];
                        minx = std::min(minx, p[0]);
                        maxx = std::max(maxx, p[0]);
                        miny = std::min(miny, p[1]);
                        maxy = std::max(maxy, p[1]);
                        minz = std::min(minz, p[2]);
                        maxz = std::max(maxz, p[2]);
                    }
                }

                const Point& ctr = S[2 * ci + 1][2 * cj + 1];
                double cx = (S[2 * ci][2 * cj][0] + S[2 * ci + 2][2 * cj][0] + S[2 * ci][2 * cj + 2][0] +
                             S[2 * ci + 2][2 * cj + 2][0]) *
                    0.25;

                double cy = (S[2 * ci][2 * cj][1] + S[2 * ci + 2][2 * cj][1] + S[2 * ci][2 * cj + 2][1] +
                             S[2 * ci + 2][2 * cj + 2][1]) *
                    0.25;

                double cz = (S[2 * ci][2 * cj][2] + S[2 * ci + 2][2 * cj][2] + S[2 * ci][2 * cj + 2][2] +
                             S[2 * ci + 2][2 * cj + 2][2]) *
                    0.25;

                double sag = std::sqrt(
                    (ctr[0] - cx) * (ctr[0] - cx) + (ctr[1] - cy) * (ctr[1] - cy) + (ctr[2] - cz) * (ctr[2] - cz)
                );

                double inf = 2.0 * sag + tolerance;

                boxes.push_back(
                    {minx - inf,
                     miny - inf,
                     minz - inf,
                     maxx + inf,
                     maxy + inf,
                     maxz + inf,
                     c0u + dcu * (ci + 0.5),
                     c0v + dcv * (cj + 0.5)}
                );
            }
        }

        return boxes;
    };

    std::vector<std::array<double, 8>> boxes_a = cell_boxes(a, au0, a_du, a_nu, av0, a_dv, a_nv);
    std::vector<std::array<double, 8>> boxes_b = cell_boxes(b, bu0, b_du, b_nu, bv0, b_dv, b_nv);

    auto cell_3d = [](const std::vector<std::array<double, 8>>& boxes) -> double {
        double best = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < boxes.size() && i < 64; i++) {
            const std::array<double, 8>& bx = boxes[i];

            double d = std::sqrt(
                (bx[3] - bx[0]) * (bx[3] - bx[0]) + (bx[4] - bx[1]) * (bx[4] - bx[1]) +
                (bx[5] - bx[2]) * (bx[5] - bx[2])
            );

            if (1e-12 < d && d < best)
                best = d;
        }

        return best < std::numeric_limits<double>::infinity() ? best : 1.0;
    };

    double h_init = std::min(cell_3d(boxes_a), cell_3d(boxes_b)) * 0.25;
    double conv_tol = std::max(tolerance, h_init * 1e-7);

    auto clamp_open = [&](std::array<double, 4>& x) {
        if (!a_closed_u)
            x[0] = std::max(au0, std::min(x[0], au1));

        if (!a_closed_v)
            x[1] = std::max(av0, std::min(x[1], av1));

        if (!b_closed_u)
            x[2] = std::max(bu0, std::min(x[2], bu1));

        if (!b_closed_v)
            x[3] = std::max(bv0, std::min(x[3], bv1));
    };

    auto correct = [&](std::array<double, 4>& x,
                       bool has_pin,
                       const std::array<double, 3>& pd,
                       const std::array<double, 3>& pp) -> bool {
        for (int it = 0; it < 8; it++) {
            Vector Sa;
            Vector Sau;
            Vector Sav;
            Vector Sb;
            Vector Sbu;
            Vector Sbv;
            eval_a(x[0], x[1], Sa, Sau, Sav);
            eval_b(x[2], x[3], Sb, Sbu, Sbv);
            double F[3] = {Sa[0] - Sb[0], Sa[1] - Sb[1], Sa[2] - Sb[2]};

            if (std::sqrt(F[0] * F[0] + F[1] * F[1] + F[2] * F[2]) < conv_tol)
                return true;

            double J[3][4];

            for (int k = 0; k < 3; k++) {
                J[k][0] = Sau[k];
                J[k][1] = Sav[k];
                J[k][2] = -Sbu[k];
                J[k][3] = -Sbv[k];
            }

            if (!has_pin) {
                std::vector<std::vector<double>> JJt(3, std::vector<double>(3));

                for (int r = 0; r < 3; r++)
                    for (int q = 0; q < 3; q++) {
                        double s = 0.0;

                        for (int c = 0; c < 4; c++)
                            s += J[r][c] * J[q][c];

                        JJt[r][q] = s;
                    }

                std::vector<double> y;

                if (!solve_gauss(JJt, {F[0], F[1], F[2]}, 3, y))
                    return false;

                for (int c = 0; c < 4; c++) {
                    double s = 0.0;

                    for (int r = 0; r < 3; r++)
                        s += J[r][c] * y[r];

                    x[c] -= s;
                }
            } else {
                std::vector<std::vector<double>> M = {
                    {J[0][0], J[0][1], J[0][2], J[0][3]},
                    {J[1][0], J[1][1], J[1][2], J[1][3]},
                    {J[2][0], J[2][1], J[2][2], J[2][3]},
                    {pd[0] * Sau[0] + pd[1] * Sau[1] + pd[2] * Sau[2],
                     pd[0] * Sav[0] + pd[1] * Sav[1] + pd[2] * Sav[2],
                     0.0,
                     0.0}
                };

                std::vector<double> rhs =
                    {F[0], F[1], F[2], pd[0] * (Sa[0] - pp[0]) + pd[1] * (Sa[1] - pp[1]) + pd[2] * (Sa[2] - pp[2])};

                std::vector<double> dx;

                if (!solve_gauss(M, rhs, 4, dx))
                    return false;

                for (int c = 0; c < 4; c++)
                    x[c] -= dx[c];
            }

            clamp_open(x);
        }

        Vector Sa;
        Vector Sau;
        Vector Sav;
        Vector Sb;
        Vector Sbu;
        Vector Sbv;
        eval_a(x[0], x[1], Sa, Sau, Sav);
        eval_b(x[2], x[3], Sb, Sbu, Sbv);

        double g = std::sqrt(
            (Sa[0] - Sb[0]) * (Sa[0] - Sb[0]) + (Sa[1] - Sb[1]) * (Sa[1] - Sb[1]) + (Sa[2] - Sb[2]) * (Sa[2] - Sb[2])
        );

        return g < conv_tol * 10.0;
    };

    struct Seed {
        double u;
        double v;
        double s;
        double t;
        bool used;
    };

    std::vector<Seed> seeds;
    double seed_tol_3d = std::max(cell_3d(boxes_a), cell_3d(boxes_b));
    int pair_budget = 20000;
    std::array<double, 3> dummy3 = {0.0, 0.0, 0.0};

    for (const std::array<double, 8>& ba : boxes_a) {
        if (pair_budget < 0)
            break;

        for (const std::array<double, 8>& bb : boxes_b) {
            if (bb[0] > ba[3] || bb[3] < ba[0] || bb[1] > ba[4] || bb[4] < ba[1] || bb[2] > ba[5] || bb[5] < ba[2])
                continue;

            pair_budget -= 1;

            if (pair_budget < 0)
                break;

            std::array<double, 4> x = {ba[6], ba[7], bb[6], bb[7]};

            if (!correct(x, false, dummy3, dummy3))
                continue;

            Vector Sa;
            Vector Sau;
            Vector Sav;
            eval_a(x[0], x[1], Sa, Sau, Sav);
            bool dup = false;

            for (const Seed& sd : seeds) {
                Vector So;
                Vector Sou;
                Vector Sov;
                eval_a(sd.u, sd.v, So, Sou, Sov);

                if (std::sqrt(
                        (Sa[0] - So[0]) * (Sa[0] - So[0]) + (Sa[1] - So[1]) * (Sa[1] - So[1]) +
                        (Sa[2] - So[2]) * (Sa[2] - So[2])
                    ) < seed_tol_3d) {
                    dup = true;
                    break;
                }
            }

            if (!dup)
                seeds.push_back({a_wrap_u(x[0]), a_wrap_v(x[1]), b_wrap_u(x[2]), b_wrap_v(x[3]), false});
        }
    }

    int max_steps = (a_nu * a_nv + b_nu * b_nv) * 32;
    double close_tol = h_init * 3.0;
    double consume_tol = h_init * 2.0;

    auto tangent_3d = [&](const std::array<double, 4>& x,
                          double dir_sign,
                          std::array<double, 3>& dir,
                          Vector& Sa,
                          Vector& Sau,
                          Vector& Sav,
                          Vector& Sbu,
                          Vector& Sbv) -> bool {
        Vector Sb;
        eval_a(x[0], x[1], Sa, Sau, Sav);
        eval_b(x[2], x[3], Sb, Sbu, Sbv);
        double na[3] =
            {Sau[1] * Sav[2] - Sau[2] * Sav[1], Sau[2] * Sav[0] - Sau[0] * Sav[2], Sau[0] * Sav[1] - Sau[1] * Sav[0]};

        double nb[3] =
            {Sbu[1] * Sbv[2] - Sbu[2] * Sbv[1], Sbu[2] * Sbv[0] - Sbu[0] * Sbv[2], Sbu[0] * Sbv[1] - Sbu[1] * Sbv[0]};

        double d[3] = {na[1] * nb[2] - na[2] * nb[1], na[2] * nb[0] - na[0] * nb[2], na[0] * nb[1] - na[1] * nb[0]};
        double dl = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        double nal = std::sqrt(na[0] * na[0] + na[1] * na[1] + na[2] * na[2]);
        double nbl = std::sqrt(nb[0] * nb[0] + nb[1] * nb[1] + nb[2] * nb[2]);

        if (dl < 1e-4 * nal * nbl || dl < 1e-30)
            return false;

        dir = {d[0] / dl * dir_sign, d[1] / dl * dir_sign, d[2] / dl * dir_sign};

        return true;
    };

    auto trace_dir = [&](const std::array<double, 4>& x0,
                         double dir_sign,
                         std::vector<std::array<double, 4>>& out,
                         const char** why_out = nullptr) -> bool {
        out.clear();
        std::array<double, 4> x = x0;
        bool have_prev_d = false;
        std::array<double, 3> prev_d = {0.0, 0.0, 0.0};
        Vector Sa0;
        Vector Sa0u;
        Vector Sa0v;
        eval_a(x[0], x[1], Sa0, Sa0u, Sa0v);
        std::array<double, 3> p_start = {Sa0[0], Sa0[1], Sa0[2]};
        std::array<double, 3> p_prev = p_start;
        double dist_traveled = 0.0;
        double h = h_init;
        int smooth = 0;
        int tang_reuse = 0;
        const char* why = "maxsteps";

        for (int step_i = 0; step_i < max_steps; step_i++) {
            std::array<double, 3> d;
            Vector Sa;
            Vector Sau;
            Vector Sav;
            Vector Sbu;
            Vector Sbv;

            if (!tangent_3d(x, dir_sign, d, Sa, Sau, Sav, Sbu, Sbv)) {
                if (!have_prev_d || tang_reuse >= 3) {
                    why = "tangency";
                    break;
                }

                d = prev_d;
                tang_reuse += 1;
            } else {
                tang_reuse = 0;
            }

            bool accepted = false;
            int attempts = 0;
            std::array<double, 4> xn = {0, 0, 0, 0};
            std::array<double, 3> p_cur = {0, 0, 0};
            double step_len = 0.0;
            bool hit_boundary = false;

            while (attempts < 7 && !accepted) {
                std::vector<std::vector<double>> Ma = {
                    {Sau[0] * Sau[0] + Sau[1] * Sau[1] + Sau[2] * Sau[2],
                     Sau[0] * Sav[0] + Sau[1] * Sav[1] + Sau[2] * Sav[2]},
                    {Sau[0] * Sav[0] + Sau[1] * Sav[1] + Sau[2] * Sav[2],
                     Sav[0] * Sav[0] + Sav[1] * Sav[1] + Sav[2] * Sav[2]}
                };

                std::vector<double> ra = {
                    h * (d[0] * Sau[0] + d[1] * Sau[1] + d[2] * Sau[2]),
                    h * (d[0] * Sav[0] + d[1] * Sav[1] + d[2] * Sav[2])
                };

                std::vector<std::vector<double>> Mb = {
                    {Sbu[0] * Sbu[0] + Sbu[1] * Sbu[1] + Sbu[2] * Sbu[2],
                     Sbu[0] * Sbv[0] + Sbu[1] * Sbv[1] + Sbu[2] * Sbv[2]},
                    {Sbu[0] * Sbv[0] + Sbu[1] * Sbv[1] + Sbu[2] * Sbv[2],
                     Sbv[0] * Sbv[0] + Sbv[1] * Sbv[1] + Sbv[2] * Sbv[2]}
                };

                std::vector<double> rb = {
                    h * (d[0] * Sbu[0] + d[1] * Sbu[1] + d[2] * Sbu[2]),
                    h * (d[0] * Sbv[0] + d[1] * Sbv[1] + d[2] * Sbv[2])
                };

                std::vector<double> duv_a;
                std::vector<double> duv_b;

                if (!solve_gauss(Ma, ra, 2, duv_a) || !solve_gauss(Mb, rb, 2, duv_b)) {
                    why = "singular";
                    break;
                }

                double delta[4] = {duv_a[0], duv_a[1], duv_b[0], duv_b[1]};
                double tc = 1.0;
                hit_boundary = false;
                struct Ax {
                    int idx;
                    double lo;
                    double hi;
                    bool closed;
                };

                Ax axs[4] = {
                    {0, au0, au1, a_closed_u},
                    {1, av0, av1, a_closed_v},
                    {2, bu0, bu1, b_closed_u},
                    {3, bv0, bv1, b_closed_v}
                };

                for (const Ax& ax : axs) {
                    if (ax.closed || std::abs(delta[ax.idx]) < 1e-15)
                        continue;

                    if (x[ax.idx] + delta[ax.idx] > ax.hi) {
                        tc = std::min(tc, (ax.hi - x[ax.idx]) / delta[ax.idx]);
                        hit_boundary = true;
                    }

                    if (x[ax.idx] + delta[ax.idx] < ax.lo) {
                        tc = std::min(tc, (ax.lo - x[ax.idx]) / delta[ax.idx]);
                        hit_boundary = true;
                    }
                }

                for (int k = 0; k < 4; k++)
                    xn[k] = x[k] + tc * delta[k];

                std::array<double, 3> p_pred = {Sa[0] + d[0] * h * tc, Sa[1] + d[1] * h * tc, Sa[2] + d[2] * h * tc};

                if (!correct(xn, true, d, p_pred)) {
                    why = "corrector";
                    h *= 0.5;
                    attempts += 1;
                    smooth = 0;
                    continue;
                }

                Vector San;
                Vector Sanu;
                Vector Sanv;
                eval_a(xn[0], xn[1], San, Sanu, Sanv);
                p_cur = {San[0], San[1], San[2]};

                step_len = std::sqrt(
                    (p_cur[0] - p_prev[0]) * (p_cur[0] - p_prev[0]) + (p_cur[1] - p_prev[1]) * (p_cur[1] - p_prev[1]) +
                    (p_cur[2] - p_prev[2]) * (p_cur[2] - p_prev[2])
                );

                if (have_prev_d && step_len > 1e-14) {
                    double sd0 = (p_cur[0] - p_prev[0]) / step_len, sd1 = (p_cur[1] - p_prev[1]) / step_len,
                           sd2 = (p_cur[2] - p_prev[2]) / step_len;

                    double ddot = sd0 * prev_d[0] + sd1 * prev_d[1] + sd2 * prev_d[2];

                    if (ddot < 0.985 && attempts < 6 && !hit_boundary) {
                        why = "angle";
                        h *= 0.5;
                        attempts += 1;
                        smooth = 0;
                        continue;
                    }
                }

                accepted = true;
            }

            if (!accepted)
                break;

            why = "maxsteps";
            prev_d = d;
            have_prev_d = true;
            smooth += 1;

            if (smooth >= 5 && h < h_init * 2.0) {
                h *= 1.4;
                smooth = 0;
            }

            x = xn;
            dist_traveled += step_len;

            if (dist_traveled > close_tol * 3.0 &&
                std::sqrt(
                    (p_cur[0] - p_start[0]) * (p_cur[0] - p_start[0]) +
                    (p_cur[1] - p_start[1]) * (p_cur[1] - p_start[1]) +
                    (p_cur[2] - p_start[2]) * (p_cur[2] - p_start[2])
                ) < close_tol) {
                out.push_back(x);

                if (why_out)
                    *why_out = "closed";

                return true;
            }

            out.push_back(x);
            p_prev = p_cur;

            if (hit_boundary) {
                why = "boundary";
                break;
            }

            for (Seed& sd : seeds) {
                if (!sd.used) {
                    Vector So;
                    Vector Sou;
                    Vector Sov;
                    eval_a(sd.u, sd.v, So, Sou, Sov);

                    if (std::sqrt(
                            (p_cur[0] - So[0]) * (p_cur[0] - So[0]) + (p_cur[1] - So[1]) * (p_cur[1] - So[1]) +
                            (p_cur[2] - So[2]) * (p_cur[2] - So[2])
                        ) < consume_tol)
                        sd.used = true;
                }
            }
        }

        if (why_out)
            *why_out = why;

        return false;
    };

    struct Axis {
        int idx;
        double c0;
        double rng;
        bool closed;
    };

    Axis axes[4] = {
        {0, au0, a_range_u, a_closed_u},
        {1, av0, a_range_v, a_closed_v},
        {2, bu0, b_range_u, b_closed_u},
        {3, bv0, b_range_v, b_closed_v}
    };

    auto eval3_q = [&](const std::array<double, 4>& q) -> std::array<double, 3> {
        Vector Sa;
        Vector Sau;
        Vector Sav;
        eval_a(q[0], q[1], Sa, Sau, Sav);

        return {Sa[0], Sa[1], Sa[2]};
    };

    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> result;
    std::vector<std::vector<std::array<double, 3>>> kept_pts3;

    for (Seed& seed : seeds) {
        if (seed.used)
            continue;

        seed.used = true;
        std::array<double, 4> x0 = {seed.u, seed.v, seed.s, seed.t};

        if (!correct(x0, false, dummy3, dummy3))
            continue;

        std::vector<std::array<double, 4>> fwd, bwd;
        const char* fwd_why = "?";
        const char* bwd_why = "?";
        bool fwd_closed = trace_dir(x0, +1, fwd, &fwd_why);

        if (!fwd_closed)
            trace_dir(x0, -1, bwd, &bwd_why);

        std::vector<std::array<double, 4>> quad;

        for (int i = (int)bwd.size() - 1; i >= 0; i--)
            quad.push_back(bwd[i]);

        quad.push_back(x0);

        for (std::array<double, 4>& p : fwd)
            quad.push_back(p);

        int min_pts =
            (!fwd_closed && std::strcmp(fwd_why, "boundary") == 0 && std::strcmp(bwd_why, "boundary") == 0) ? 2 : 4;

        if ((int)quad.size() < min_pts)
            continue;

        for (size_t i = 1; i < quad.size(); i++) {
            for (const Axis& ax : axes) {
                if (!ax.closed)
                    continue;

                double jump = quad[i][ax.idx] - quad[i - 1][ax.idx];

                if (jump > ax.rng * 0.5)
                    quad[i][ax.idx] -= ax.rng;
                else if (jump < -ax.rng * 0.5)
                    quad[i][ax.idx] += ax.rng;
            }
        }

        std::array<double, 3> p_first = eval3_q(quad.front());
        std::array<double, 3> p_last = eval3_q(quad.back());

        double gap2 = std::sqrt(
            (p_first[0] - p_last[0]) * (p_first[0] - p_last[0]) + (p_first[1] - p_last[1]) * (p_first[1] - p_last[1]) +
            (p_first[2] - p_last[2]) * (p_first[2] - p_last[2])
        );

        bool is_loop = fwd_closed || ((int)quad.size() >= 6 && gap2 < close_tol);

        if (is_loop)
            quad.pop_back();

        if ((int)quad.size() < min_pts)
            continue;

        int m = (int)quad.size();
        std::vector<std::array<double, 3>> trace_pts3(m);

        for (int i = 0; i < m; i++)
            trace_pts3[i] = eval3_q(quad[i]);

        double dup_tol = h_init * 2.0;
        bool dup = false;

        for (std::vector<std::array<double, 3>>& other : kept_pts3) {
            bool all_close = true;

            for (double f : {0.25, 0.5, 0.75}) {
                std::array<double, 3> cp = trace_pts3[(int)((m - 1) * f)];
                double dmin = dup_tol + 1.0;

                for (size_t k = 0; k < other.size(); k += 1) {
                    const std::array<double, 3>& op = other[k];

                    dmin = std::min(
                        dmin,
                        std::sqrt(
                            (cp[0] - op[0]) * (cp[0] - op[0]) + (cp[1] - op[1]) * (cp[1] - op[1]) +
                            (cp[2] - op[2]) * (cp[2] - op[2])
                        )
                    );
                }

                if (dmin > dup_tol) {
                    all_close = false;
                    break;
                }
            }

            if (all_close) {
                dup = true;
                break;
            }
        }

        if (dup)
            continue;

        kept_pts3.push_back(trace_pts3);

        auto gap3 = [&](const std::array<double, 4>& qi, const std::array<double, 4>& qj) -> double {
            std::array<double, 3> pi = eval3_q(qi);
            std::array<double, 3> pj = eval3_q(qj);

            return std::sqrt(
                (pi[0] - pj[0]) * (pi[0] - pj[0]) + (pi[1] - pj[1]) * (pi[1] - pj[1]) +
                (pi[2] - pj[2]) * (pi[2] - pj[2])
            );
        };

        for (int gp = 0; gp < 4; gp++) {
            std::vector<double> gg;

            for (size_t i = 0; i + 1 < quad.size(); i++)
                gg.push_back(gap3(quad[i], quad[i + 1]));

            if (gg.empty())
                break;

            std::vector<double> sorted_gg = gg;
            std::sort(sorted_gg.begin(), sorted_gg.end());
            double med = sorted_gg[sorted_gg.size() / 2];

            if (med <= 0)
                break;

            bool changed = false;
            size_t i = 0;

            while (i + 1 < quad.size() && quad.size() < 4000) {
                if (gap3(quad[i], quad[i + 1]) > 1.5 * med) {
                    std::array<double, 4> midq;

                    for (int k = 0; k < 4; k++)
                        midq[k] = (quad[i][k] + quad[i + 1][k]) * 0.5;

                    if (correct(midq, false, dummy3, dummy3)) {
                        quad.insert(quad.begin() + i + 1, midq);
                        changed = true;
                        i += 2;
                        continue;
                    }
                }

                i++;
            }

            if (!changed)
                break;
        }

        std::array<double, 4> closure = {0.0, 0.0, 0.0, 0.0};

        if (is_loop && quad.size() >= 2) {
            std::array<double, 4> virt = quad[0];

            for (const Axis& ax : axes) {
                double jump = quad[0][ax.idx] - quad.back()[ax.idx];

                if (ax.closed) {
                    while (jump > ax.rng * 0.5)
                        jump -= ax.rng;

                    while (jump < -ax.rng * 0.5)
                        jump += ax.rng;
                }

                virt[ax.idx] = quad.back()[ax.idx] + jump;
                closure[ax.idx] = virt[ax.idx] - quad[0][ax.idx];
            }

            quad.push_back(virt);
        }

        std::vector<std::array<double, 4>> out_pts;
        out_pts.push_back(quad[0]);
        std::vector<int> cross_idx;

        for (size_t i = 1; i < quad.size(); i++) {
            const std::array<double, 4>& pa_ = quad[i - 1];
            const std::array<double, 4>& pb_ = quad[i];
            std::vector<std::tuple<double, int, double>> crossings;

            for (const Axis& ax : axes) {
                if (!ax.closed || std::abs(pb_[ax.idx] - pa_[ax.idx]) <= 1e-15)
                    continue;

                int k0 = (int)std::floor((pa_[ax.idx] - ax.c0) / ax.rng);
                int k1 = (int)std::floor((pb_[ax.idx] - ax.c0) / ax.rng);

                for (int k = std::min(k0, k1) + 1; k <= std::max(k0, k1); k++) {
                    double L = ax.c0 + k * ax.rng;
                    double t = (L - pa_[ax.idx]) / (pb_[ax.idx] - pa_[ax.idx]);

                    if (0.0 < t && t < 1.0)
                        crossings.push_back({t, ax.idx, L});
                }
            }

            std::sort(crossings.begin(), crossings.end());

            for (const std::tuple<double, int, double>& crossing : crossings) {
                const double t = std::get<0>(crossing);
                const int idx = std::get<1>(crossing);
                const double L = std::get<2>(crossing);
                std::array<double, 4> cp;

                for (int k = 0; k < 4; k++)
                    cp[k] = pa_[k] + (pb_[k] - pa_[k]) * t;

                cp[idx] = L;
                correct(cp, false, dummy3, dummy3);
                out_pts.push_back(cp);
                cross_idx.push_back((int)out_pts.size() - 1);
            }

            out_pts.push_back(pb_);

            if (i < quad.size() - 1) {
                bool on_seam = false;

                for (const Axis& ax : axes) {
                    if (!ax.closed)
                        continue;

                    double k = std::round((pb_[ax.idx] - ax.c0) / ax.rng);
                    double L = ax.c0 + k * ax.rng;

                    if (std::abs(pb_[ax.idx] - L) < ax.rng * 1e-9 &&
                        std::abs(pb_[ax.idx] - pa_[ax.idx]) > ax.rng * 1e-9) {

                        out_pts.back()[ax.idx] = L;
                        on_seam = true;
                    }
                }

                if (on_seam)
                    cross_idx.push_back((int)out_pts.size() - 1);
            }
        }

        bool wrap_drift = false;

        for (const Axis& ax : axes)
            if (std::abs(closure[ax.idx]) > ax.rng * 0.5)
                wrap_drift = true;

        std::vector<std::pair<std::vector<std::array<double, 4>>, bool>> pieces;

        if (cross_idx.size() == 0) {
            pieces.push_back({out_pts, is_loop && !wrap_drift});
        } else if (is_loop) {
            for (size_t ci = 0; ci + 1 < cross_idx.size(); ci++) {
                int ia = cross_idx[ci];
                int ib = cross_idx[ci + 1];

                pieces.push_back(
                    {std::vector<std::array<double, 4>>(out_pts.begin() + ia, out_pts.begin() + ib + 1), false}
                );
            }

            std::vector<std::array<double, 4>> wrap_piece(out_pts.begin() + cross_idx.back(), out_pts.end());

            for (int pi = 1; pi <= cross_idx[0]; pi++) {
                std::array<double, 4> p;

                for (int k = 0; k < 4; k++)
                    p[k] = out_pts[pi][k] + closure[k];

                wrap_piece.push_back(p);
            }

            pieces.push_back({wrap_piece, false});
        } else {
            std::vector<int> bounds;
            bounds.push_back(0);

            for (int ci : cross_idx)
                bounds.push_back(ci);

            bounds.push_back((int)out_pts.size() - 1);

            for (size_t bi = 0; bi + 1 < bounds.size(); bi++) {
                int ia = bounds[bi];
                int ib = bounds[bi + 1];

                if (ib > ia)
                    pieces.push_back(
                        {std::vector<std::array<double, 4>>(out_pts.begin() + ia, out_pts.begin() + ib + 1), false}
                    );
            }
        }

        for (std::pair<std::vector<std::array<double, 4>>, bool>& piece : pieces) {
            std::vector<std::array<double, 4>>& piece_pts = piece.first;
            const bool piece_loop = piece.second;

            if (piece_pts.size() < 2)
                continue;

            std::array<double, 4> mid = piece_pts[piece_pts.size() / 2];

            for (const Axis& ax : axes) {
                if (!ax.closed)
                    continue;

                int k_s = (int)std::floor((mid[ax.idx] - ax.c0) / ax.rng);

                if (k_s != 0)
                    for (std::array<double, 4>& p : piece_pts)
                        p[ax.idx] -= k_s * ax.rng;
            }

            std::vector<std::array<double, 3>> pts3(piece_pts.size());

            for (size_t i = 0; i < piece_pts.size(); i++)
                pts3[i] = eval3_q(piece_pts[i]);

            double chord3 = 0.0;

            for (size_t i = 1; i < pts3.size(); i++)
                chord3 += std::sqrt(
                    (pts3[i][0] - pts3[i - 1][0]) * (pts3[i][0] - pts3[i - 1][0]) +
                    (pts3[i][1] - pts3[i - 1][1]) * (pts3[i][1] - pts3[i - 1][1]) +
                    (pts3[i][2] - pts3[i - 1][2]) * (pts3[i][2] - pts3[i - 1][2])
                );

            if (chord3 < h_init * 0.05)
                continue;

            double refine_tol = std::max(tolerance * 100.0, 5e-6);

            for (int dp = 0; dp < 8; dp++) {
                bool refined = false;
                std::vector<std::array<double, 4>> new_pp;
                new_pp.push_back(piece_pts[0]);
                size_t i = 0;

                while (i + 1 < piece_pts.size() && piece_pts.size() < 3000) {
                    const std::array<double, 4>& pa2 = piece_pts[i];
                    const std::array<double, 4>& pb2 = piece_pts[i + 1];
                    std::array<double, 3> p3a = eval3_q(pa2);
                    std::array<double, 3> p3b = eval3_q(pb2);
                    std::array<double, 4> midq;

                    for (int k = 0; k < 4; k++)
                        midq[k] = (pa2[k] + pb2[k]) * 0.5;

                    if (correct(midq, false, dummy3, dummy3)) {
                        std::array<double, 3> p3m = eval3_q(midq);
                        double ex = p3b[0] - p3a[0];
                        double ey = p3b[1] - p3a[1];
                        double ez = p3b[2] - p3a[2];
                        double l2 = ex * ex + ey * ey + ez * ez;
                        double dev;

                        if (l2 > 1e-30) {
                            double tt = ((p3m[0] - p3a[0]) * ex + (p3m[1] - p3a[1]) * ey + (p3m[2] - p3a[2]) * ez) / l2;
                            double cxp = p3a[0] + tt * ex;
                            double cyp = p3a[1] + tt * ey;
                            double czp = p3a[2] + tt * ez;

                            dev = std::sqrt(
                                (p3m[0] - cxp) * (p3m[0] - cxp) + (p3m[1] - cyp) * (p3m[1] - cyp) +
                                (p3m[2] - czp) * (p3m[2] - czp)
                            );
                        } else {
                            dev = 0.0;
                        }

                        if (dev > refine_tol) {
                            new_pp.push_back(midq);
                            refined = true;
                        }
                    }

                    new_pp.push_back(pb2);
                    i++;
                }

                piece_pts = new_pp;

                if (!refined)
                    break;
            }

            pts3.assign(piece_pts.size(), std::array<double, 3>{});

            for (size_t i = 0; i < piece_pts.size(); i++)
                pts3[i] = eval3_q(piece_pts[i]);

            bool ploop = piece_loop;

            auto fit_track = [&](const std::vector<Point>& pts2, double fit_tol_track) -> NurbsCurve {
                int mp = (int)pts2.size();
                double total_turning = 0.0;

                for (int i = 1; i < mp - 1; i++) {
                    double dx1 = pts2[i][0] - pts2[i - 1][0];
                    double dy1 = pts2[i][1] - pts2[i - 1][1];
                    double dz1 = pts2[i][2] - pts2[i - 1][2];
                    double dx2 = pts2[i + 1][0] - pts2[i][0];
                    double dy2 = pts2[i + 1][1] - pts2[i][1];
                    double dz2 = pts2[i + 1][2] - pts2[i][2];
                    double l1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + dz1 * dz1);
                    double l2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);

                    if (l1 > 1e-14 && l2 > 1e-14) {
                        double c = (dx1 * dx2 + dy1 * dy2 + dz1 * dz2) / (l1 * l2);
                        c = std::max(-1.0, std::min(1.0, c));
                        total_turning += std::acos(c);
                    }
                }

                std::vector<double> chords(mp, 0.0);
                double total_len = 0.0;

                for (int i = 1; i < mp; i++) {
                    total_len += pts2[i].distance(pts2[i - 1]);
                    chords[i] = total_len;
                }

                if (ploop && mp > 1)
                    total_len += pts2[0].distance(pts2[mp - 1]);

                if (total_len > 1e-14)
                    for (int i = 1; i < mp; i++)
                        chords[i] /= total_len;

                int target_cvs = std::max(8, (int)(total_turning / 0.5) + 6);
                int max_cvs = std::max(8, std::min(mp - 1, mp / 3));
                NurbsCurve best;
                double best_dev = std::numeric_limits<double>::infinity();

                while (target_cvs <= max_cvs) {
                    NurbsCurve crv = NurbsCurve::create_fitted(pts2, target_cvs, 3, ploop);

                    if (!crv.is_valid())
                        break;

                    const std::pair<double, double> domain_ft = crv.domain();
                    const double ft0 = domain_ft.first;
                    const double ft1 = domain_ft.second;
                    double dev = 0.0;

                    for (int i = 0; i < mp; i++) {
                        double t = ft0 + (ft1 - ft0) * chords[i];
                        double w = (ft1 - ft0) * 2.0 / std::max(mp - 1, 1);
                        double lo = std::max(ft0, t - w);
                        double hi = std::min(ft1, t + w);

                        for (int it = 0; it < 24; ++it) {
                            double m1 = lo + (hi - lo) / 3;
                            double m2 = hi - (hi - lo) / 3;

                            if (crv.point_at(m1).distance(pts2[i]) < crv.point_at(m2).distance(pts2[i]))
                                hi = m2;
                            else
                                lo = m1;
                        }

                        dev = std::max(dev, crv.point_at(0.5 * (lo + hi)).distance(pts2[i]));
                    }

                    if (dev < best_dev) {
                        best = crv;
                        best_dev = dev;
                    }

                    if (dev < fit_tol_track)
                        break;

                    target_cvs *= 2;
                }

                if (best_dev >= fit_tol_track) {
                    NurbsCurve interp = ploop
                        ? NurbsCurve::create_interpolated(pts2, CurveNurbsKnotStyle::ChordPeriodic)
                        : NurbsCurve::create_interpolated(pts2);

                    if (interp.is_valid())
                        best = interp;
                }

                if (best.is_valid())
                    best.set_domain(0.0, 1.0);

                return best;
            };

            std::vector<Point> pts3_p(pts3.size());

            for (size_t i = 0; i < pts3.size(); i++)
                pts3_p[i] = Point(pts3[i][0], pts3[i][1], pts3[i][2]);

            std::vector<Point> pts_pa(piece_pts.size());
            std::vector<Point> pts_pb(piece_pts.size());

            for (size_t i = 0; i < piece_pts.size(); i++) {
                pts_pa[i] = Point(piece_pts[i][0], piece_pts[i][1], 0.0);
                pts_pb[i] = Point(piece_pts[i][2], piece_pts[i][3], 0.0);
            }

            NurbsCurve crv3 = fit_track(pts3_p, std::max(tolerance * 10.0, 1e-7));
            NurbsCurve pcurve_a = fit_track(pts_pa, std::min(a_du, a_dv) * 1e-4);
            NurbsCurve pcurve_b = fit_track(pts_pb, std::min(b_du, b_dv) * 1e-4);

            if (!crv3.is_valid() || !pcurve_a.is_valid() || !pcurve_b.is_valid())
                continue;

            result.push_back({std::move(crv3), std::move(pcurve_a), std::move(pcurve_b)});
        }
    }

    drop_point_sections(result, tolerance);

    return result;
}

namespace {
/// Keep the pcurve sub-segments whose lifted 3D point lies inside the cutter footprint.
std::vector<NurbsCurve> clip_pcurve_to_cutter(
    const NurbsSurface& target,
    const NurbsCurve& pc,
    const NurbsSurface& cutter
) {

    int n = std::max(pc.cv_count() * 4, 16);
    const std::pair<double, double> dc = pc.domain();
    double d0 = dc.first;
    double d1 = dc.second;
    const std::pair<double, double> cu = cutter.domain(0);
    const std::pair<double, double> cv = cutter.domain(1);
    double corner_diag = cutter.point_at(cu.first, cv.first).distance(cutter.point_at(cu.second, cv.second));
    double on_tol = std::max(1e-6, corner_diag * 2e-3);

    Point q00 = cutter.point_at(cu.first, cv.first);
    Point q10 = cutter.point_at(cu.second, cv.first);
    Point q01 = cutter.point_at(cu.first, cv.second);
    Vector eu(q10[0] - q00[0], q10[1] - q00[1], q10[2] - q00[2]);
    Vector ev(q01[0] - q00[0], q01[1] - q00[1], q01[2] - q00[2]);
    double eu2 = eu[0] * eu[0] + eu[1] * eu[1] + eu[2] * eu[2];
    double ev2 = ev[0] * ev[0] + ev[1] * ev[1] + ev[2] * ev[2];
    bool fast_planar = (eu2 > 1e-28 && ev2 > 1e-28);

    auto gap = [&](double t) -> double {
        Point uv = pc.point_at(t);
        Point p3 = target.point_at(uv[0], uv[1]);

        if (fast_planar) {
            double dx = p3[0] - q00[0];
            double dy = p3[1] - q00[1];
            double dz = p3[2] - q00[2];
            double a = (dx * eu[0] + dy * eu[1] + dz * eu[2]) / eu2;
            double b = (dx * ev[0] + dy * ev[1] + dz * ev[2]) / ev2;
            a = std::min(std::max(a, 0.0), 1.0);
            b = std::min(std::max(b, 0.0), 1.0);
            double cx = q00[0] + a * eu[0] + b * ev[0], cy = q00[1] + a * eu[1] + b * ev[1],
                   cz = q00[2] + a * eu[2] + b * ev[2];

            return std::sqrt((p3[0] - cx) * (p3[0] - cx) + (p3[1] - cy) * (p3[1] - cy) + (p3[2] - cz) * (p3[2] - cz));
        }

        return std::get<2>(Closest::surface_point(cutter, p3, 0.0, 0.0, 0.0, 0.0));
    };

    auto refine = [&](double t_in, double t_out) -> double {
        double a = t_in;
        double b = t_out;
        double edge_tol = std::max(1e-6, corner_diag * 2e-4);

        for (int k = 0; k < 24; ++k) {
            double tm = (a + b) * 0.5;

            if (gap(tm) < edge_tol)
                a = tm;
            else
                b = tm;
        }

        return b;
    };

    std::vector<std::pair<double, bool>> flags;
    flags.reserve(n + 1);

    for (int i = 0; i <= n; ++i) {
        double t = d0 + (d1 - d0) * i / n;
        flags.emplace_back(t, gap(t) < on_tol);
    }

    std::vector<NurbsCurve> pieces;
    std::vector<std::pair<double, double>> spans;
    int i = 0;

    while (i <= n) {
        if (flags[i].second) {
            int j = i;

            while (j + 1 <= n && flags[j + 1].second)
                ++j;

            double ta = (i == 0) ? flags[i].first : refine(flags[i].first, flags[i - 1].first);
            double tb = (j == n) ? flags[j].first : refine(flags[j].first, flags[j + 1].first);

            if (tb - ta > (d1 - d0) * 1e-6)
                spans.push_back({ta, tb});

            i = j + 1;
        } else {
            ++i;
        }
    }

    Point pcs_ = pc.point_at(d0);
    Point pce_ = pc.point_at(d1);
    bool pc_closed2 = pcs_.distance(pce_) < 1e-9;

    if (pc_closed2 && spans.size() >= 2 && spans.front().first <= d0 + (d1 - d0) * 1e-9 &&
        spans.back().second >= d1 - (d1 - d0) * 1e-9) {

        double ta = spans.back().first;
        double tb = spans.front().second;
        spans.front() = {ta, tb};
        spans.pop_back();
        int m2 = std::max(32, n / 2);
        std::vector<Point> pts;
        double len1 = d1 - ta;
        double len2 = tb - d0;
        double tot = len1 + len2;

        for (int k2 = 0; k2 <= m2; ++k2) {
            double f = tot * k2 / m2;
            double t = (f < len1) ? ta + f : d0 + (f - len1);
            pts.push_back(pc.point_at(std::min(t, d1)));
        }

        NurbsCurve joined = NurbsCurve::create(false, 1, pts);

        if (joined.is_valid())
            pieces.push_back(joined);

        spans.erase(spans.begin());
    }

    for (std::pair<double, double>& sp : spans) {
        NurbsCurve piece = pc;

        if (piece.trim(sp.first, sp.second) && piece.is_valid())
            pieces.push_back(piece);
    }

    return pieces;
}
} // namespace

std::vector<NurbsCurve> Intersection::cut_curves_on_surface(
    const NurbsSurface& target,
    const NurbsSurface& cutter,
    double tolerance
) {

    std::vector<NurbsCurve> out;
    bool cutter_planar = cutter.is_planar(nullptr, 1e-6);
    double rtol = std::max(tolerance, 1e-7) * 1e4;
    RecogSurface rt = recognize_surface(target, rtol);

    for (std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& tr : surface_surface(target, cutter, tolerance)) {
        const NurbsCurve& c3d = std::get<0>(tr);
        std::vector<NurbsCurve> pcs;
        NurbsCurve pa_an = analytic_pcurve(target, rt, c3d);

        if (pa_an.is_valid()) {
            pcs.push_back(pa_an);
        } else if (rt.kind == RecogSurface::SPHERE) {
            pcs = analytic_sphere_pullback(target, rt, c3d);

            if (pcs.empty())
                pcs = Closest::surface_curve(target, c3d, 0.0, 0.0, tolerance);

            if (pcs.empty())
                pcs.push_back(std::get<1>(tr));
        } else if (rt.kind == RecogSurface::CONE || rt.kind == RecogSurface::CYLINDER) {
            pcs = analytic_cone_pullback(target, rt, c3d);

            if (pcs.empty())
                pcs = Closest::surface_curve(target, c3d, 0.0, 0.0, tolerance);

            if (pcs.empty())
                pcs.push_back(std::get<1>(tr));
        } else if (rt.kind == RecogSurface::TORUS) {
            pcs = analytic_torus_pullback(target, rt, c3d);

            if (pcs.empty())
                pcs = Closest::surface_curve(target, c3d, 0.0, 0.0, tolerance);

            if (pcs.empty())
                pcs.push_back(std::get<1>(tr));
        } else {
            const NurbsCurve& pa_tr = std::get<1>(tr);

            if (pa_tr.is_valid())
                pcs.push_back(pa_tr);
            else
                pcs = Closest::surface_curve(target, c3d, 0.0, 0.0, tolerance);
        }

        for (const NurbsCurve& pc : pcs) {
            if (cutter_planar) {
                std::vector<NurbsCurve> clipped = clip_pcurve_to_cutter(target, pc, cutter);
                out.insert(out.end(), clipped.begin(), clipped.end());
            } else {
                out.push_back(pc);
            }
        }
    }

    return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// Polylines and plane sets
// ═══════════════════════════════════════════════════════════════════════════

/// Whether two vectors are parallel within angle_tol.
static bool vectors_nearly_parallel(const Vector& v0, const Vector& v1, double angle_tol) {

    double m0 = v0.magnitude();
    double m1 = v1.magnitude();

    if (m0 < Tolerance::ZERO_TOLERANCE || m1 < Tolerance::ZERO_TOLERANCE)
        return false;

    double cos_angle = std::fabs(v0.dot(v1) / (m0 * m1));

    return cos_angle >= std::cos(angle_tol);
}

bool Intersection::plane_plane_plane_check(
    const Plane& p0,
    const Plane& p1,
    const Plane& p2,
    double angle_tol,
    Point& output
) {

    if (vectors_nearly_parallel(p0.z_axis(), p1.z_axis(), angle_tol))
        return false;

    if (vectors_nearly_parallel(p0.z_axis(), p2.z_axis(), angle_tol))
        return false;

    if (vectors_nearly_parallel(p1.z_axis(), p2.z_axis(), angle_tol))
        return false;

    return plane_plane_plane(p0, p1, p2, output);
}

double Intersection::remap(double val, double from1, double to1, double from2, double to2) {

    double span = to1 - from1;

    if (std::fabs(span) < Tolerance::ZERO_TOLERANCE)
        return from2;

    double t = (val - from1) / span;

    return from2 + t * (to2 - from2);
}

bool Intersection::closest_point_on_segment(const Point& pt, const Line& seg, Point& output, double& t) {

    Point start = seg.start();
    Point end = seg.end();
    double dx = end[0] - start[0];
    double dy = end[1] - start[1];
    double dz = end[2] - start[2];
    double len_sq = dx * dx + dy * dy + dz * dz;

    if (len_sq < 1e-20) {
        output = start;
        t = 0.0;

        return true;
    }

    double vx = pt[0] - start[0];
    double vy = pt[1] - start[1];
    double vz = pt[2] - start[2];
    t = (vx * dx + vy * dy + vz * dz) / len_sq;

    if (t < 0.0)
        t = 0.0;

    if (t > 1.0)
        t = 1.0;

    output = Point(start[0] + t * dx, start[1] + t * dy, start[2] + t * dz);

    return true;
}

bool Intersection::plane_4planes(const Plane& main_plane, const std::array<Plane, 4>& planes, Polyline& output) {

    Point p0;
    Point p1;
    Point p2;
    Point p3;

    if (!plane_plane_plane_check(planes[0], planes[1], main_plane, 0.1, p0))
        return false;

    if (!plane_plane_plane_check(planes[1], planes[2], main_plane, 0.1, p1))
        return false;

    if (!plane_plane_plane_check(planes[2], planes[3], main_plane, 0.1, p2))
        return false;

    if (!plane_plane_plane_check(planes[3], planes[0], main_plane, 0.1, p3))
        return false;

    output = Polyline(
        std::vector<Point>{
            p0,
            p1,
            p2,
            p3,
            p0,
        }
    );

    return true;
}

bool Intersection::plane_4planes_open(const Plane& main_plane, const std::array<Plane, 4>& planes, Polyline& output) {

    std::vector<Point> corners;

    for (size_t i = 0; i < 4; i++) {
        Line edge;

        if (!plane_plane_to_line_canonical(planes[i], planes[(i + 1) % 4], edge))
            return false;

        Point corner;

        if (!line_plane(edge, main_plane, corner, false))
            return false;

        corners.push_back(corner);
    }

    output = Polyline(corners);

    return true;
}

bool Intersection::plane_4lines(
    const Plane& plane,
    const Line& l0,
    const Line& l1,
    const Line& l2,
    const Line& l3,
    Polyline& output
) {

    Point p0;
    Point p1;
    Point p2;
    Point p3;

    if (!line_plane(l0, plane, p0, false))
        return false;

    if (!line_plane(l1, plane, p1, false))
        return false;

    if (!line_plane(l2, plane, p2, false))
        return false;

    if (!line_plane(l3, plane, p3, false))
        return false;

    output = Polyline(
        std::vector<Point>{
            p0,
            p1,
            p2,
            p3,
            p0,
        }
    );

    return true;
}

bool Intersection::line_two_planes(const Line& line, const Plane& plane0, const Plane& plane1, Line& output) {

    Point q0;
    Point q1;

    if (!line_plane(line, plane0, q0, true))
        return false;

    if (!line_plane(line, plane1, q1, true))
        return false;

    output = Line(q0[0], q0[1], q0[2], q1[0], q1[1], q1[2]);

    return true;
}

bool Intersection::polyline_plane(
    const Polyline& polyline,
    const Plane& plane,
    std::vector<Point>& points,
    std::vector<int>& edge_ids
) {

    size_t n = polyline.point_count();

    if (n < 2)
        return false;

    for (size_t i = 0; i < n - 1; i++) {
        Point a = polyline.get_point(i);
        Point b = polyline.get_point(i + 1);
        double va = plane_value_at(plane, a);
        double vb = plane_value_at(plane, b);
        bool a_on = std::fabs(va) < Tolerance::ZERO_TOLERANCE;
        bool b_on = std::fabs(vb) < Tolerance::ZERO_TOLERANCE;

        if (a_on && b_on)
            continue;

        if (a_on) {
            points.push_back(a);
            edge_ids.push_back(static_cast<int>(i));
            continue;
        }

        if (b_on) {
            if (i + 2 == n) {
                Point front = polyline.get_point(0);
                bool closes = std::fabs(b[0] - front[0]) < Tolerance::ZERO_TOLERANCE &&
                    std::fabs(b[1] - front[1]) < Tolerance::ZERO_TOLERANCE &&
                    std::fabs(b[2] - front[2]) < Tolerance::ZERO_TOLERANCE;

                if (!closes) {
                    points.push_back(b);
                    edge_ids.push_back(static_cast<int>(i));
                }
            }

            continue;
        }

        Line seg(a[0], a[1], a[2], b[0], b[1], b[2]);
        Point hit;

        if (line_plane(seg, plane, hit, true)) {
            points.push_back(hit);
            edge_ids.push_back(static_cast<int>(i));
        }
    }

    return !points.empty();
}

bool Intersection::line_line_3d(const Line& cutter, const Line& seg, Point& output) {

    double t0;
    double t1;

    if (!line_line_parameters(cutter, seg, t0, t1, 0.0, false, false))
        return false;

    output = cutter.point_at(t0);

    return true;
}

bool Intersection::scale_vector_to_distance_of_2planes(
    const Vector& direction,
    const Plane& plane0,
    const Plane& plane1,
    Vector& output
) {

    if (direction.magnitude() < Tolerance::ZERO_TOLERANCE)
        return false;

    Line ray(0.0, 0.0, 0.0, direction[0], direction[1], direction[2]);
    Point q0;
    Point q1;

    if (!line_plane(ray, plane0, q0, false))
        return false;

    if (!line_plane(ray, plane1, q1, false))
        return false;

    output = q1 - q0;
    Vector n1 = plane1.z_axis();
    double n1_mag = n1.magnitude();

    if (n1_mag < Tolerance::ZERO_TOLERANCE)
        return false;

    Point o0 = plane0.origin();
    double d = ((o0[0] - plane1.origin()[0]) * n1[0] + (o0[1] - plane1.origin()[1]) * n1[1] +
                (o0[2] - plane1.origin()[2]) * n1[2]) /
        n1_mag;

    double dist_ortho_sq = d * d;

    if (dist_ortho_sq < Tolerance::ZERO_TOLERANCE)
        return false;

    double dist_sq = output.dot(output);

    if (dist_sq / dist_ortho_sq >= 10.0)
        return false;

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Plane 2D helpers
// ═══════════════════════════════════════════════════════════════════════════

namespace {

/// Project a point into plane coordinates.
std::array<double, 2> plane_to_2d(const Point& p, const Point& origin, const Vector& xax, const Vector& yax) {

    const Vector d = p - origin;

    return {d.dot(xax), d.dot(yax)};
}

/// Lift plane coordinates back to a point.
Point plane_to_3d(const std::array<double, 2>& p, const Point& origin, const Vector& xax, const Vector& yax) {
    return origin + xax * p[0] + yax * p[1];
}

/// Squared distance of two 2D points.
double distance_sq_2d(const std::array<double, 2>& a, const std::array<double, 2>& b) {

    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];

    return dx * dx + dy * dy;
}

/// Signed area of a 2D ring, positive when counter-clockwise.
double signed_area_2d(const std::vector<std::array<double, 2>>& ring) {

    double area = 0.0;
    const size_t n = ring.size();

    for (size_t i = 0; i < n; i++)
        area += ring[i][0] * ring[(i + 1) % n][1] - ring[(i + 1) % n][0] * ring[i][1];

    return area;
}

/// Project a polyline into plane coordinates.
std::vector<std::array<double, 2>> polyline_to_2d(const Polyline& polyline, const Point& origin, const Vector& xax, const Vector& yax) {

    std::vector<std::array<double, 2>> ring;
    ring.reserve(polyline.point_count());

    for (size_t i = 0; i < polyline.point_count(); i++)
        ring.push_back(plane_to_2d(polyline.get_point(i), origin, xax, yax));

    if (ring.size() > 1 && distance_sq_2d(ring.front(), ring.back()) < 1e-12)
        ring.pop_back();

    return ring;
}

/// Lift a 2D ring back to a polyline.
Polyline polyline_to_3d(const std::vector<std::array<double, 2>>& ring, const Point& origin, const Vector& xax, const Vector& yax) {

    std::vector<Point> pts;
    pts.reserve(ring.size() + 1);

    for (const std::array<double, 2>& p : ring)
        pts.push_back(plane_to_3d(p, origin, xax, yax));

    pts.push_back(pts.front());

    return Polyline(pts);
}

/// Even-odd point in polygon test.
bool point_in_polygon_2d(const std::vector<std::array<double, 2>>& ring, const std::array<double, 2>& p) {

    int wn = 0;
    const size_t n = ring.size();

    for (size_t i = 0; i < n; i++) {
        const std::array<double, 2>& a = ring[i];
        const std::array<double, 2>& b = ring[(i + 1) % n];
        const double e = (b[0] - a[0]) * (p[1] - a[1]) - (p[0] - a[0]) * (b[1] - a[1]);

        if (a[1] <= p[1] && b[1] > p[1] && e > 0.0)
            wn++;
        else if (a[1] > p[1] && b[1] <= p[1] && e < 0.0)
            wn--;
    }

    return wn != 0;
}

/// Segment-segment crossing with parameters on both.
bool seg_seg_2d(const std::array<double, 2>& s0, const std::array<double, 2>& s1, const std::array<double, 2>& e0, const std::array<double, 2>& e1, double& t_s, double& t_e) {

    const double sx = s1[0] - s0[0];
    const double sy = s1[1] - s0[1];
    const double ex = e1[0] - e0[0];
    const double ey = e1[1] - e0[1];
    const double denom = sx * ey - sy * ex;

    if (std::abs(denom) < 1e-20)
        return false;

    const double dx = e0[0] - s0[0];
    const double dy = e0[1] - s0[1];
    t_s = (dx * ey - dy * ex) / denom;
    t_e = (dx * sy - dy * sx) / denom;

    return true;
}

/// Overlap range of two collinear segments on the first.
bool collinear_overlap_2d(const std::array<double, 2>& s0, const std::array<double, 2>& s1, const std::array<double, 2>& e0, const std::array<double, 2>& e1, double& t_enter, double& t_exit) {

    const double sx = s1[0] - s0[0];
    const double sy = s1[1] - s0[1];
    const double ex = e1[0] - e0[0];
    const double ey = e1[1] - e0[1];
    const double sl2 = sx * sx + sy * sy;
    const double el2 = ex * ex + ey * ey;

    if (sl2 < 1e-20 || el2 < 1e-20)
        return false;

    if (std::abs((sx * ey - sy * ex) / std::sqrt(sl2 * el2)) > 1e-4)
        return false;

    const double apx = s0[0] - e0[0];
    const double apy = s0[1] - e0[1];

    if (std::abs((apx * ey - apy * ex) / std::sqrt(el2)) > 1e-3)
        return false;

    const double ts0 = (apx * ex + apy * ey) / el2;
    const double ts1 = ((s1[0] - e0[0]) * ex + (s1[1] - e0[1]) * ey) / el2;
    const double ov_min = std::max(0.0, std::min(ts0, ts1));
    const double ov_max = std::min(1.0, std::max(ts0, ts1));

    if (ov_max - ov_min < 1e-9)
        return false;

    const double tsr = ts1 - ts0;

    if (std::abs(tsr) < 1e-20)
        return false;

    t_enter = std::max(0.0, std::min((ov_min - ts0) / tsr, (ov_max - ts0) / tsr));
    t_exit = std::min(1.0, std::max((ov_min - ts0) / tsr, (ov_max - ts0) / tsr));

    return (t_exit - t_enter) > 1e-9;
}

/// Parameter of the closest point on segment ab to p.
double closest_param_2d(const std::array<double, 2>& p, const std::array<double, 2>& a, const std::array<double, 2>& b) {

    const double abx = b[0] - a[0];
    const double aby = b[1] - a[1];
    const double l2 = abx * abx + aby * aby;

    if (l2 < 1e-20)
        return 0.0;

    const double t = ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / l2;

    return std::max(0.0, std::min(1.0, t));
}

/// Squared distance from p to segment ab.
double distance_sq_seg_2d(const std::array<double, 2>& p, const std::array<double, 2>& a, const std::array<double, 2>& b) {

    const double t = closest_param_2d(p, a, b);
    const std::array<double, 2> q{a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])};

    return distance_sq_2d(p, q);
}

/// Parameters along one joint segment where it crosses or overlaps the plate edges.
std::vector<double> segment_plate_parameters_2d(
    const std::vector<std::array<double, 2>>& plate,
    const std::array<double, 2>& p0,
    const std::array<double, 2>& p1,
    std::vector<std::pair<double, double>>& coll_ranges
) {

    const double EPS = 1e-9;
    std::vector<double> ts;
    ts.push_back(0.0);

    for (size_t i = 0; i < plate.size(); i++) {
        const std::array<double, 2>& a = plate[i];
        const std::array<double, 2>& b = plate[(i + 1) % plate.size()];
        double t_s;
        double t_e;

        if (seg_seg_2d(p0, p1, a, b, t_s, t_e) && t_s > EPS && t_s < 1.0 - EPS && t_e >= -EPS && t_e <= 1.0 + EPS)
            ts.push_back(t_s);

        double t_in;
        double t_out;

        if (!collinear_overlap_2d(p0, p1, a, b, t_in, t_out))
            continue;

        coll_ranges.emplace_back(t_in, t_out);

        if (t_in > EPS && t_in < 1.0 - EPS)
            ts.push_back(t_in);

        if (t_out > EPS && t_out < 1.0 - EPS)
            ts.push_back(t_out);
    }

    ts.push_back(1.0);
    std::sort(ts.begin(), ts.end());
    std::vector<double> unique;

    for (const double t : ts)
        if (unique.empty() || std::abs(t - unique.back()) >= EPS)
            unique.push_back(t);

    return unique;
}

/// Whether t falls in any of the ranges.
bool in_ranges_2d(const std::vector<std::pair<double, double>>& ranges, double t) {

    for (const std::pair<double, double>& r : ranges)
        if (t >= r.first - 1e-9 && t <= r.second + 1e-9)
            return true;

    return false;
}

/// Sub-segments of the open joint path inside the plate, as separate pieces.
std::vector<std::vector<std::array<double, 2>>> clip_open_path_2d(const std::vector<std::array<double, 2>>& plate, const std::vector<std::array<double, 2>>& joint) {

    std::vector<std::vector<std::array<double, 2>>> pieces;

    for (size_t s = 0; s + 1 < joint.size(); s++) {
        const std::array<double, 2>& p0 = joint[s];
        const std::array<double, 2>& p1 = joint[s + 1];
        std::vector<std::pair<double, double>> coll_ranges;
        const std::vector<double> ts = segment_plate_parameters_2d(plate, p0, p1, coll_ranges);
        std::vector<std::array<double, 2>> current;

        for (size_t i = 0; i + 1 < ts.size(); i++) {
            const double t_mid = 0.5 * (ts[i] + ts[i + 1]);
            const std::array<double, 2> mid{p0[0] + (p1[0] - p0[0]) * t_mid, p0[1] + (p1[1] - p0[1]) * t_mid};
            const bool include = point_in_polygon_2d(plate, mid) || in_ranges_2d(coll_ranges, t_mid);

            if (!include) {
                if (!current.empty())
                    pieces.push_back(std::move(current));

                current.clear();
                continue;
            }

            const std::array<double, 2> sub_a{p0[0] + (p1[0] - p0[0]) * ts[i], p0[1] + (p1[1] - p0[1]) * ts[i]};
            const std::array<double, 2> sub_b{p0[0] + (p1[0] - p0[0]) * ts[i + 1], p0[1] + (p1[1] - p0[1]) * ts[i + 1]};

            if (!current.empty() && distance_sq_2d(current.back(), sub_a) >= 1e-18) {
                pieces.push_back(std::move(current));
                current.clear();
            }

            if (current.empty())
                current.push_back(sub_a);

            current.push_back(sub_b);
        }

        if (!current.empty())
            pieces.push_back(std::move(current));
    }

    return pieces;
}

/// Chains clipped pieces end to end into one path.
std::vector<std::array<double, 2>> chain_pieces_2d(const std::vector<std::vector<std::array<double, 2>>>& pieces) {

    const double DISTANCE_SQ = 0.01;
    std::vector<std::array<double, 2>> chain;

    for (const std::vector<std::array<double, 2>>& piece : pieces) {
        if (piece.size() <= 1)
            continue;

        if (chain.empty()) {
            chain = piece;
            continue;
        }

        std::vector<std::array<double, 2>> pts = piece;

        if (distance_sq_2d(chain.back(), pts.front()) > DISTANCE_SQ &&
            distance_sq_2d(chain.back(), pts.back()) > DISTANCE_SQ)
            std::reverse(chain.begin(), chain.end());

        if (distance_sq_2d(chain.back(), pts.front()) > distance_sq_2d(chain.back(), pts.back()))
            std::reverse(pts.begin(), pts.end());

        for (size_t j = 1; j < pts.size(); j++)
            chain.push_back(pts[j]);
    }

    return chain;
}

/// Plate edge parameters of the chain ends, or -1 when an end is off the plate.
void chain_plate_parameters_2d(const std::vector<std::array<double, 2>>& plate, const std::vector<std::array<double, 2>>& chain, double& t0, double& t1) {

    t0 = -1.0;
    t1 = -1.0;

    for (size_t i = 0; i < plate.size(); i++) {
        const std::array<double, 2>& a = plate[i];
        const std::array<double, 2>& b = plate[(i + 1) % plate.size()];

        if (distance_sq_seg_2d(chain.front(), a, b) < 1.0)
            t0 = (double)i + closest_param_2d(chain.front(), a, b);

        if (distance_sq_seg_2d(chain.back(), a, b) < 1.0)
            t1 = (double)i + closest_param_2d(chain.back(), a, b);

        if (t0 >= 0.0 && t1 >= 0.0)
            return;
    }
}

/// Miter offset of a closed 2D ring by delta along the edge normals.
std::vector<std::array<double, 2>> offset_ring_2d(const std::vector<std::array<double, 2>>& ring, double delta, bool concave_notch) {

    const size_t n = ring.size();
    std::vector<std::array<double, 2>> normals;
    normals.reserve(n);

    for (size_t i = 0; i < n; i++) {
        const double ex = ring[(i + 1) % n][0] - ring[i][0];
        const double ey = ring[(i + 1) % n][1] - ring[i][1];
        const double len = std::sqrt(ex * ex + ey * ey);

        if (len < 1e-12)
            normals.push_back({0.0, 0.0});
        else
            normals.push_back({ey / len, -ex / len});
    }

    std::vector<std::array<double, 2>> out;
    out.reserve(n * 3);

    for (size_t i = 0; i < n; i++) {
        const std::array<double, 2>& np = normals[(i + n - 1) % n];
        const std::array<double, 2>& nn = normals[i];
        const std::array<double, 2>& p = ring[i];
        const double cos_a = np[0] * nn[0] + np[1] * nn[1];
        const double sin_a = np[0] * nn[1] - np[1] * nn[0];
        const double denom = 1.0 + cos_a;

        if (cos_a > -0.999 && sin_a * delta < 0.0 && concave_notch) {
            out.push_back({p[0] + np[0] * delta, p[1] + np[1] * delta});
            out.push_back(p);
            out.push_back({p[0] + nn[0] * delta, p[1] + nn[1] * delta});
        } else if (std::abs(denom) < 1e-9) {
            const double bx = np[0] + nn[0];
            const double by = np[1] + nn[1];
            const double bl = std::sqrt(bx * bx + by * by);

            if (bl < 1e-12)
                out.push_back({p[0] + nn[0] * delta, p[1] + nn[1] * delta});
            else
                out.push_back({p[0] + (bx / bl) * delta, p[1] + (by / bl) * delta});
        } else {
            const double k = delta / denom;
            out.push_back({p[0] + (np[0] + nn[0]) * k, p[1] + (np[1] + nn[1]) * k});
        }
    }

    return out;
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Polyline booleans
// ═══════════════════════════════════════════════════════════════════════════

std::vector<Polyline> Intersection::polyline_boolean(const Polyline& a, const Polyline& b, int clip_type) {
    return BooleanPolyline::compute(a, b, clip_type);
}

bool Intersection::offset_in_3d(Polyline& polyline, const Plane& plane, double offset) {

    if (polyline.point_count() < 3)
        return false;

    const Point origin = polyline.get_point(0);
    const Vector xax = plane.base1();
    const Vector yax = plane.base2();
    const std::vector<std::array<double, 2>> ring = polyline_to_2d(polyline, origin, xax, yax);

    if (ring.size() < 3)
        return false;

    const double delta = signed_area_2d(ring) < 0.0 ? -offset : offset;
    std::vector<std::array<double, 2>> out = offset_ring_2d(ring, delta, offset > 0.0);

    if (out.size() < 3)
        return false;

    if (std::abs(signed_area_2d(out)) * 0.5 < 0.0001)
        return false;

    size_t cp = 0;

    for (size_t i = 1; i < out.size(); i++)
        if (distance_sq_2d(out[i], ring[0]) < distance_sq_2d(out[cp], ring[0]))
            cp = i;

    std::rotate(out.begin(), out.begin() + cp, out.end());
    polyline = polyline_to_3d(out, origin, xax, yax);

    return true;
}

bool Intersection::polyline_boolean_2d_in_plane(
    const Polyline& polyline0,
    const Polyline& polyline1,
    const Plane& plane,
    Polyline& intersection_result,
    int intersection_type,
    bool include_triangles,
    double min_area,
    double collapse_eps
) {

    if (polyline0.point_count() < 3 || polyline1.point_count() < 3)
        return false;

    const Point origin = polyline0.get_point(0);
    const Vector xax = plane.base1();
    const Vector yax = plane.base2();
    const Polyline a2d =
        polyline_to_3d(polyline_to_2d(polyline0, origin, xax, yax), Point(), Vector(1, 0, 0), Vector(0, 1, 0));

    const Polyline b2d =
        polyline_to_3d(polyline_to_2d(polyline1, origin, xax, yax), Point(), Vector(1, 0, 0), Vector(0, 1, 0));

    std::vector<Polyline> result_2d;

    if (intersection_type >= 0 && intersection_type <= 2) {
        result_2d = BooleanPolyline::compute(a2d, b2d, intersection_type);
    } else if (intersection_type == 3) {
        const std::vector<Polyline> u = BooleanPolyline::compute(a2d, b2d, 1);
        const std::vector<Polyline> inter = BooleanPolyline::compute(a2d, b2d, 0);

        if (u.empty())
            return false;

        if (inter.empty())
            result_2d = u;
        else
            result_2d = BooleanPolyline::compute(u[0], inter[0], 2);
    } else {
        return false;
    }

    if (result_2d.empty())
        return false;

    std::vector<std::array<double, 2>> ring = polyline_to_2d(result_2d[0], Point(), Vector(1, 0, 0), Vector(0, 1, 0));

    if (ring.size() < 3)
        return false;

    if (collapse_eps > 0.0) {
        const double eps_sq = collapse_eps * collapse_eps;
        std::vector<std::array<double, 2>> collapsed;

        for (const std::array<double, 2>& p : ring)
            if (collapsed.empty() || distance_sq_2d(p, collapsed.back()) >= eps_sq)
                collapsed.push_back(p);

        if (collapsed.size() >= 2 && distance_sq_2d(collapsed.back(), collapsed.front()) < eps_sq)
            collapsed.pop_back();

        ring.swap(collapsed);

        if (ring.size() < 3)
            return false;
    }

    if (ring.size() == 3 && !include_triangles)
        return false;

    if (std::abs(signed_area_2d(ring)) * 0.5 <= min_area)
        return false;

    intersection_result = polyline_to_3d(ring, origin, xax, yax);

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Joints
// ═══════════════════════════════════════════════════════════════════════════

bool Intersection::polyline_plane_to_line(
    const Polyline& poly,
    const Plane& plane,
    const Point& align_start,
    Line& out
) {

    std::vector<Point> pts;
    std::vector<int> edge_ids;

    if (!polyline_plane(poly, plane, pts, edge_ids))
        return false;

    if (pts.size() < 2)
        return false;

    size_t ia = 0;
    size_t ib = 1;
    double best = -1.0;

    for (size_t p1 = 0; p1 + 1 < pts.size(); p1++)
        for (size_t p2 = p1 + 1; p2 < pts.size(); p2++)
            if ((pts[p1] - pts[p2]).magnitude_squared() > best) {
                best = (pts[p1] - pts[p2]).magnitude_squared();
                ia = p1;
                ib = p2;
            }

    const Point& a = pts[ia];
    const Point& b = pts[ib];

    if ((a - align_start).magnitude_squared() <= (b - align_start).magnitude_squared())
        out = Line::from_points(a, b);
    else
        out = Line::from_points(b, a);

    return true;
}

bool Intersection::quad_from_line_top_bottom_planes(
    const Plane& face_plane,
    const Line& line,
    const Plane& plane0,
    const Plane& plane1,
    Polyline& out
) {

    const Plane lp0 = Plane::from_point_normal(line.start(), line.to_vector());
    const Plane lp1 = Plane::from_point_normal(line.end(), line.to_vector());
    Line edge0;
    Line edge1;

    if (!plane_plane(plane0, face_plane, edge0) || !plane_plane(plane1, face_plane, edge1))
        return false;

    Point p0;
    Point p1;
    Point p2;
    Point p3;

    if (!line_plane(edge0, lp0, p0, false) || !line_plane(edge1, lp0, p1, false))
        return false;

    if (!line_plane(edge1, lp1, p2, false) || !line_plane(edge0, lp1, p3, false))
        return false;

    out = Polyline(std::vector<Point>{p0, p1, p2, p3, p0});

    return true;
}

bool Intersection::orthogonal_vector_between_two_plane_pairs(
    const Plane& pp00,
    const Plane& pp10,
    const Plane& pp11,
    Vector& out
) {

    Line l0;
    Line l1;

    if (!plane_plane_to_line_canonical(pp00, pp10, l0) || !plane_plane_to_line_canonical(pp00, pp11, l1))
        return false;

    if (l0.to_vector().magnitude_squared() < 1e-20)
        return false;

    out = l1.start() - l0.closest_point(l1.start(), false).second;

    return true;
}

bool Intersection::closed_and_open_paths_2d(
    const Polyline& plate,
    const Polyline& joint,
    const Plane& plane,
    Polyline& out,
    std::pair<double, double>& cp_pair
) {

    const Point origin = plate.get_point(0);
    const Vector xax = plane.base1();
    const Vector yax = plane.base2();
    const std::vector<std::array<double, 2>> plate2d = polyline_to_2d(plate, origin, xax, yax);

    if (plate2d.size() < 3)
        return false;

    std::vector<std::array<double, 2>> joint2d;

    for (size_t i = 0; i < joint.point_count(); i++)
        joint2d.push_back(plane_to_2d(joint.get_point(i), origin, xax, yax));

    if (joint2d.size() < 2)
        return false;

    std::vector<std::array<double, 2>> c2d = chain_pieces_2d(clip_open_path_2d(plate2d, joint2d));

    if (c2d.size() < 2)
        return false;

    double t0;
    double t1;
    chain_plate_parameters_2d(plate2d, c2d, t0, t1);
    bool reverse_flag = t0 > t1;

    if ((size_t)std::floor(t0) == 0 && (size_t)std::floor(t1) == c2d.size() - 1)
        reverse_flag = !reverse_flag;

    if (reverse_flag) {
        std::swap(t0, t1);
        std::reverse(c2d.begin(), c2d.end());
    }

    if (t0 < 0.0 || t1 < 0.0)
        return false;

    std::vector<Point> out_pts;

    for (const std::array<double, 2>& p : c2d)
        out_pts.push_back(plane_to_3d(p, origin, xax, yax));

    out = Polyline(out_pts);
    cp_pair = std::pair<double, double>(t0, t1);

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Elements
// ═══════════════════════════════════════════════════════════════════════════

std::vector<std::tuple<int, int, int, int, int, Polyline>> Intersection::face_to_face(
    const std::vector<int>& adjacency,
    const std::vector<std::vector<Polyline>>& polylines,
    const std::vector<std::vector<Plane>>& planes,
    double coplanar_tolerance
) {

    std::vector<std::tuple<int, int, int, int, int, Polyline>> results;

    std::vector<std::vector<std::array<double, 6>>> face_boxes(polylines.size());

    for (size_t e = 0; e < polylines.size(); ++e) {
        face_boxes[e].reserve(polylines[e].size());

        for (const Polyline& f : polylines[e]) {
            std::array<double, 6> bx = {
                std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity()
            };

            const std::vector<double>& c = f._coords;

            for (size_t k = 0; k + 2 < c.size(); k += 3) {
                bx[0] = std::min(bx[0], c[k]);
                bx[3] = std::max(bx[3], c[k]);
                bx[1] = std::min(bx[1], c[k + 1]);
                bx[4] = std::max(bx[4], c[k + 1]);
                bx[2] = std::min(bx[2], c[k + 2]);
                bx[5] = std::max(bx[5], c[k + 2]);
            }

            for (int k = 0; k < 3; ++k) {
                bx[k] -= coplanar_tolerance;
                bx[k + 3] += coplanar_tolerance;
            }

            face_boxes[e].push_back(bx);
        }
    }

    for (size_t idx = 0; idx < adjacency.size(); idx += 4) {
        int a = adjacency[idx];
        int b = adjacency[idx + 1];

        bool found = false;

        for (int i = 0; i < (int)planes[a].size() && !found; i++) {
            const Point oa = planes[a][i].origin();
            const Vector za = planes[a][i].z_axis();
            const std::array<double, 6>& ba = face_boxes[a][i];

            for (int j = 0; j < (int)planes[b].size(); j++) {
                const std::array<double, 6>& bb = face_boxes[b][j];

                if (ba[0] > bb[3] || bb[0] > ba[3] || ba[1] > bb[4] || bb[1] > ba[4] || ba[2] > bb[5] || bb[2] > ba[5])
                    continue;

                if (!Plane::is_coplanar_from_normals(
                        oa,
                        za,
                        planes[b][j].origin(),
                        planes[b][j].z_axis(),
                        false,
                        coplanar_tolerance
                    ))
                    continue;

                std::vector<Point> pts_i = polylines[a][i].get_points();
                Vector edge(pts_i[1][0] - pts_i[0][0], pts_i[1][1] - pts_i[0][1], pts_i[1][2] - pts_i[0][2]);
                edge.normalize_self();
                Vector zax = za;
                Vector yax = zax.cross(edge);
                yax.normalize_self();
                Plane pln = Plane::from_frame(pts_i[0], edge, yax, zax);

                std::vector<Polyline> bools = Polyline::boolean_op(polylines[a][i], polylines[b][j], pln, 0);

                if (bools.empty() || bools[0].point_count() < 3)
                    continue;

                int type = (i > 1 ? 0 : 1) + (j > 1 ? 0 : 1);
                Polyline jpl = bools[0].is_closed() ? std::move(bools[0]) : bools[0].closed();
                results.emplace_back(a, b, i, j, type, std::move(jpl));
                found = true;
                break;
            }
        }
    }

    return results;
}

std::vector<int> Intersection::adjacency_search(std::vector<Element*>& elements, double inflate) {

    size_t N = elements.size();
    std::vector<OBB> obbs(N);

    for (size_t i = 0; i < N; i++) {
        std::vector<Point> pts;

        for (Polyline& pl : elements[i]->polylines())
            for (Point& p : pl.get_points())
                pts.push_back(p);

        obbs[i] = OBB::from_points(pts, inflate);
    }

    std::vector<AABB> aabbs(N);

    for (size_t i = 0; i < N; i++)
        aabbs[i] = obbs[i].aabb();

    double ws = 0;

    for (AABB& a : aabbs) {
        ws = std::max(ws, std::abs(a.cx + a.hx));
        ws = std::max(ws, std::abs(a.cy + a.hy));
        ws = std::max(ws, std::abs(a.cz + a.hz));
        ws = std::max(ws, std::abs(a.cx - a.hx));
        ws = std::max(ws, std::abs(a.cy - a.hy));
        ws = std::max(ws, std::abs(a.cz - a.hz));
    }

    SpatialBVH bvh;
    bvh.build_from_aabbs(aabbs.data(), N, ws * 2);
    std::vector<int> adjacency;

    for (size_t i = 0; i < N; i++) {
        std::vector<int> hits = bvh.query_aabb(aabbs[i]);

        for (int j : hits) {
            if ((int)i < j && obbs[i].collides_with(obbs[j])) {
                adjacency.push_back(static_cast<int>(i));
                adjacency.push_back(j);
                adjacency.push_back(-1);
                adjacency.push_back(-1);
            }
        }
    }

    return adjacency;
}

std::vector<std::tuple<int, int, int, int, int, Polyline>> Intersection::face_to_face(
    const std::vector<int>& adjacency,
    std::vector<Element*>& elements,
    double coplanar_tolerance
) {

    size_t N = elements.size();
    std::vector<std::vector<Polyline>> all_polys(N);
    std::vector<std::vector<Plane>> all_planes(N);

    for (size_t i = 0; i < N; i++) {
        all_polys[i] = elements[i]->polylines();
        all_planes[i] = elements[i]->planes();
    }

    return face_to_face(adjacency, all_polys, all_planes, coplanar_tolerance);
}

bool Intersection::line_line_classified(
    const Line& s0,
    const Line& s1,
    int n_segs_0,
    int n_segs_1,
    int cur_seg_0,
    int cur_seg_1,
    double above_closer_to_edge,
    Point& p0,
    Point& p1,
    Vector& v0,
    Vector& v1,
    Vector& normal,
    bool& type0,
    bool& type1,
    bool& is_parallel
) {

    const double DIST_SQ = 1e-6;
    const double EPS_PAR = 1.0;
    v0 = s0.to_vector();
    v1 = s1.to_vector();
    normal = v0.cross(v1);
    const double ang = v0.angle(v1, false, true);
    is_parallel = normal.magnitude_squared() < 1e-24 || (90.0 - std::abs(ang - 90.0)) < EPS_PAR;

    if (is_parallel)
        normal = Plane::from_point_normal(s0.start(), v0).base1();

    normal.normalize_self();
    const std::array<Point, 2> ends0{s0.start(), s0.end()};
    const std::array<Point, 2> ends1{s1.start(), s1.end()};

    for (size_t i = 0; i < 2; i++)
        for (size_t j = 0; j < 2; j++)
            if ((ends0[i] - ends1[j]).magnitude_squared() < DIST_SQ) {
                p0 = ends0[i];
                p1 = ends0[i];
                v0 = ends0[1 - i] - ends0[i];
                v1 = ends1[1 - j] - ends1[j];
                v0.normalize_self();
                v1.normalize_self();
                type0 = 0;
                type1 = 0;

                return true;
            }

    v0.normalize_self();
    v1.normalize_self();

    if (is_parallel) {
        std::vector<std::pair<double, double>> pts;

        for (const Point& q : {s0.start(), s0.end(), s1.start(), s1.end()}) {
            const Point q0 = s0.closest_point(q, false).second;
            const Point q1 = s1.closest_point(q, false).second;
            pts.emplace_back((q0 - s0.start()).dot(v0), (q1 - s1.start()).dot(v1));
        }

        std::sort(pts.begin(), pts.end());
        const Point m0 = s0.start() + v0 * ((pts[1].first + pts[2].first) * 0.5);
        const Point m1 = s1.start() + v1 * ((pts[1].second + pts[2].second) * 0.5);
        const Point avg = m0 + (m1 - m0) * 0.5;
        p0 = s0.closest_point(avg, false).second;
        p1 = s1.closest_point(avg, false).second;

        if (s0.closest_point(p0, false).first > 0.5)
            v0 = -v0;

        if (s1.closest_point(p1, false).first > 0.5)
            v1 = -v1;

        type0 = 0;
        type1 = 0;

        return true;
    }

    double t0_v;
    double t1_v;

    if (!line_line_parameters(s0, s1, t0_v, t1_v, 0.0, false, true))
        return false;

    const double t0c = std::max(0.0, std::min(1.0, t0_v));
    const double t1c = std::max(0.0, std::min(1.0, t1_v));
    p0 = s0.point_at(t0c);
    p1 = s1.point_at(t1c);
    const double tt0 = (t0c + (double)cur_seg_0) / (double)n_segs_0;
    const double tt1 = (t1c + (double)cur_seg_1) / (double)n_segs_1;
    const double close0 = 2.0 * std::abs(0.5 - tt0);
    const double close1 = 2.0 * std::abs(0.5 - tt1);

    if (above_closer_to_edge < 0.0) {
        type0 = 1;
        type1 = 1;
    } else if (above_closer_to_edge > 1.0) {
        type0 = tt0 < tt1 ? 0 : 1;
        type1 = tt0 < tt1 ? 1 : 0;
    } else {
        type0 = close0 > above_closer_to_edge ? 0 : 1;
        type1 = close1 > above_closer_to_edge ? 0 : 1;

        if (close0 > close1 && type0 == 0 && type1 == 0)
            type1 = 1;
        else if (close0 < close1 && type0 == 0 && type1 == 0)
            type0 = 1;
    }

    if (tt0 > 0.5 && type0 == 0)
        v0 = -v0;

    if (tt1 > 0.5 && type1 == 0)
        v1 = -v1;

    return true;
}

} // namespace session_cpp
