#include "intersection.h"
#include "boolean_polyline.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "closest.h"
#include "spatial_bvh.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <cstring>
#include <set>
#include <functional>
#include <array>
#include <optional>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <tuple>

namespace session_cpp {

int Intersection::solve_3x3(
    const double row0[3], const double row1[3], const double row2[3],
    double d0, double d1, double d2,
    double& x, double& y, double& z,
    double& pivot_ratio) {
    
    int i, j;
    double *p0, *p1, *p2;
    double temp, workarray[12], maxpiv, minpiv;
    
    const int sizeof_row = 3 * sizeof(row0[0]);
    
    pivot_ratio = x = y = z = 0.0;
    
    temp = std::fabs(row0[0]);
    i = j = 0;
    double val = std::fabs(row0[1]);
    if (val > temp) { temp = val; j = 1; }
    val = std::fabs(row0[2]);
    if (val > temp) { temp = val; j = 2; }
    val = std::fabs(row1[0]);
    if (val > temp) { temp = val; i = 1; j = 0; }
    val = std::fabs(row1[1]);
    if (val > temp) { temp = val; i = 1; j = 1; }
    val = std::fabs(row1[2]);
    if (val > temp) { temp = val; i = 1; j = 2; }
    val = std::fabs(row2[0]);
    if (val > temp) { temp = val; i = 2; j = 0; }
    val = std::fabs(row2[1]);
    if (val > temp) { temp = val; i = 2; j = 1; }
    val = std::fabs(row2[2]);
    if (val > temp) { temp = val; i = 2; j = 2; }
    
    if (temp == 0.0) return 0;
    
    maxpiv = minpiv = std::fabs(temp);
    p0 = workarray;
    
    switch (i) {
        case 1:
            std::memcpy(p0, row1, sizeof_row); p0[3] = d1; p0 += 4;
            std::memcpy(p0, row0, sizeof_row); p0[3] = d0; p0 += 4;
            std::memcpy(p0, row2, sizeof_row); p0[3] = d2;
            break;
        case 2:
            std::memcpy(p0, row2, sizeof_row); p0[3] = d2; p0 += 4;
            std::memcpy(p0, row1, sizeof_row); p0[3] = d1; p0 += 4;
            std::memcpy(p0, row0, sizeof_row); p0[3] = d0;
            break;
        default:
            std::memcpy(p0, row0, sizeof_row); p0[3] = d0; p0 += 4;
            std::memcpy(p0, row1, sizeof_row); p0[3] = d1; p0 += 4;
            std::memcpy(p0, row2, sizeof_row); p0[3] = d2;
            break;
    }
    
    double *x_addr = &x;
    double *y_addr = &y;
    double *z_addr = &z;
    
    switch (j) {
        case 1:
            std::swap(x_addr, y_addr);
            p0 = &workarray[0];
            std::swap(p0[0], p0[1]); p0 += 4;
            std::swap(p0[0], p0[1]); p0 += 4;
            std::swap(p0[0], p0[1]);
            break;
        case 2:
            std::swap(x_addr, z_addr);
            p0 = &workarray[0];
            std::swap(p0[0], p0[2]); p0 += 4;
            std::swap(p0[0], p0[2]); p0 += 4;
            std::swap(p0[0], p0[2]);
            break;
    }
    
    temp = 1.0 / workarray[0];
    p0 = p1 = workarray + 1;
    *p1++ *= temp; *p1++ *= temp; *p1++ *= temp;
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
    if (val > temp) { temp = val; j = 1; }
    val = std::fabs(workarray[9]);
    if (val > temp) { temp = val; i = 1; j = 0; }
    val = std::fabs(workarray[10]);
    if (val > temp) { temp = val; i = j = 1; }
    
    if (temp == 0.0) return 1;
    
    val = std::fabs(temp);
    if (val > maxpiv) maxpiv = val;
    else if (val < minpiv) minpiv = val;
    
    if (j) {
        p0 = workarray + 1; p1 = p0 + 1;
        std::swap(*p0, *p1); p0 += 4; p1 += 4;
        std::swap(*p0, *p1); p0 += 4; p1 += 4;
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
    *p1++ *= temp; *p1 *= temp; p1--;
    temp = -(*p0++);
    if (temp != 0.0) {
        *p0++ += temp * (*p1++);
        *p0 += temp * (*p1);
        p0--; p1--;
    }
    temp = -(*p2++);
    if (temp != 0.0) {
        *p2++ += temp * (*p1++);
        *p2 += temp * (*p1);
        p2--; p1--;
    }
    
    temp = *p2++;
    if (temp == 0.0) return 2;
    
    val = std::fabs(temp);
    if (val > maxpiv) maxpiv = val;
    else if (val < minpiv) minpiv = val;
    
    *p2 /= temp;
    temp = -(*p1++);
    if (temp != 0.0) *p1 += temp * (*p2);
    temp = -(*p0++);
    if (temp != 0.0) *p0 += temp * (*p2);
    
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
    return plane.a() * point[0] + plane.b() * point[1] + 
           plane.c() * point[2] + plane.d();
}

bool Intersection::line_line(
    const Line& line0,
    const Line& line1,
    Point& output,
    double tolerance) {
    
    // Use the robust OpenNURBS-style line_line_parameters method
    double t0, t1;
    bool rc = line_line_parameters(line0, line1, t0, t1, tolerance, true, false);
    
    if (rc) {
        // Compute the midpoint between the two closest points
        Point p0 = line0.point_at(t0);
        Point p1 = line1.point_at(t1);
        output = Point(
            (p0[0] + p1[0]) * 0.5,
            (p0[1] + p1[1]) * 0.5,
            (p0[2] + p1[2]) * 0.5
        );
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
    bool near_parallel_as_closest) {
    
    // OpenNURBS-style: Check for exact endpoint matches first
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
    
    // Use dot product method (OpenNURBS approach)
    Vector A = line0.to_vector();
    Vector B = line1.to_vector();
    Vector C(
        p1_start[0] - p0_start[0],
        p1_start[1] - p0_start[1],
        p1_start[2] - p0_start[2]
    );
    
    double AA = A.dot(A);
    double BB = B.dot(B);
    double AB = A.dot(B);
    double AC = A.dot(C);
    double BC = B.dot(C);
    
    // Solve 2x2 system: [AA -AB] [t0] = [AC]
    //                   [-AB BB] [t1]   [-BC]
    double det = AA * BB - AB * AB;
    
    // Check for parallel lines (determinant near zero)
    double zero_tol = std::max(AA, BB) * std::numeric_limits<double>::epsilon();
    if (std::fabs(det) < zero_tol) {
        if (!near_parallel_as_closest) {
            return false;
        }
        // Compute closest points for (near) parallel lines
        t0 = (AA > 0.0) ? (AC / AA) : 0.0;
        t1 = (BB > 0.0) ? ((BC + t0 * AB) / BB) : 0.0;

        if (intersect_segments) {
            if (t0 < 0.0) t0 = 0.0; else if (t0 > 1.0) t0 = 1.0;
            if (t1 < 0.0) t1 = 0.0; else if (t1 > 1.0) t1 = 1.0;
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
    
    // Clamp to segment if requested
    if (intersect_segments) {
        if (t0 < 0.0) t0 = 0.0;
        else if (t0 > 1.0) t0 = 1.0;
        
        if (t1 < 0.0) t1 = 0.0;
        else if (t1 > 1.0) t1 = 1.0;
    }
    
    // Check distance tolerance if specified
    if (tolerance > 0.0) {
        Point pt0 = line0.point_at(t0);
        Point pt1 = line1.point_at(t1);
        double dist = pt0.distance(pt1);
        if (dist > tolerance) {
            return false;
        }
    }
    
    return true;
}

bool Intersection::plane_plane(
    const Plane& plane0,
    const Plane& plane1,
    Line& output) {

    Vector d = plane1.z_axis().cross(plane0.z_axis());

    Point p = Point(
        (plane0.origin()[0] + plane1.origin()[0]) * 0.5,
        (plane0.origin()[1] + plane1.origin()[1]) * 0.5,
        (plane0.origin()[2] + plane1.origin()[2]) * 0.5
    );

    Plane plane2 = Plane::from_point_normal(p, d);

    Point output_p;
    bool rc = plane_plane_plane(plane0, plane1, plane2, output_p);
    if (!rc) {
        return false;
    }

    output = Line::from_points(output_p, Point(
        output_p[0] + d[0],
        output_p[1] + d[1],
        output_p[2] + d[2]
    ));

    return true;
}

bool Intersection::plane_plane_to_line_canonical(
    const Plane& plane0,
    const Plane& plane1,
    Line& output)
{
    // Direct port of the CGAL canonical formulation used by wood's
    // `cgal::intersection_util::plane_plane` (cgal_intersection_util.cpp:493-511)
    // and downstream `get_orthogonal_vector_between_two_plane_pairs`
    // (cgal_intersection_util.cpp:618-628). Anchor is foot-of-
    // perpendicular from world origin onto the intersection line:
    //     c0 = (k0·(n1·n1) - k1·(n0·n1)) / det
    //     c1 = (k1·(n0·n0) - k0·(n0·n1)) / det
    //     anchor = c0·n0 + c1·n1
    // where k_i = n_i·plane_i.origin() and det = |n0|²|n1|² - (n0·n1)².
    //
    // Direction convention: d = n1 × n0 (matches session's existing
    // `plane_plane` at line 325 + wood's cgal_intersection_util.cpp:497).
    Vector n0 = plane0.z_axis();
    Vector n1 = plane1.z_axis();
    Vector d(
        n1[1]*n0[2] - n1[2]*n0[1],
        n1[2]*n0[0] - n1[0]*n0[2],
        n1[0]*n0[1] - n1[1]*n0[0]
    );
    double d_sq = d[0]*d[0] + d[1]*d[1] + d[2]*d[2];
    if (d_sq < 1e-20) return false;

    double k0 = n0[0]*plane0.origin()[0] + n0[1]*plane0.origin()[1] + n0[2]*plane0.origin()[2];
    double k1 = n1[0]*plane1.origin()[0] + n1[1]*plane1.origin()[1] + n1[2]*plane1.origin()[2];
    double n0n0 = n0[0]*n0[0] + n0[1]*n0[1] + n0[2]*n0[2];
    double n1n1 = n1[0]*n1[0] + n1[1]*n1[1] + n1[2]*n1[2];
    double n0n1 = n0[0]*n1[0] + n0[1]*n1[1] + n0[2]*n1[2];
    double det = n0n0 * n1n1 - n0n1 * n0n1;
    if (std::abs(det) < 1e-20) return false;
    double c0 = (k0 * n1n1 - k1 * n0n1) / det;
    double c1 = (k1 * n0n0 - k0 * n0n1) / det;
    Point anchor(
        c0 * n0[0] + c1 * n1[0],
        c0 * n0[1] + c1 * n1[1],
        c0 * n0[2] + c1 * n1[2]
    );
    output = Line::from_points(
        anchor,
        Point(anchor[0] + d[0], anchor[1] + d[1], anchor[2] + d[2])
    );
    return true;
}

bool Intersection::line_plane(
    const Line& line,
    const Plane& plane,
    Point& output,
    bool is_finite) {
    
    bool rc = false;
    double a, b, d, fd, t;
    
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
        if (fd > 1.0 && (std::fabs(a) >= std::numeric_limits<double>::max() / fd || 
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

bool Intersection::plane_plane_plane(
    const Plane& plane0,
    const Plane& plane1,
    const Plane& plane2,
    Point& output) {

    double pr = 0.0;
    double x, y, z;

    const double plane_0[3] = { plane0.a(), plane0.b(), plane0.c() };
    const double plane_1[3] = { plane1.a(), plane1.b(), plane1.c() };
    const double plane_2[3] = { plane2.a(), plane2.b(), plane2.c() };

    const int rank = solve_3x3(
        plane_0, plane_1, plane_2,
        -plane0.d(), -plane1.d(), -plane2.d(),
        x, y, z, pr
    );

    output = Point(x, y, z);
    // `rank == 3` is a necessary but not sufficient condition: the Gaussian
    // elimination can complete with a tiny final pivot (e.g. ~1e-15), which
    // makes the reported solution mathematically nonsingular but numerically
    // meaningless — `1.0 / tiny_pivot` propagates into coordinates on the
    // order of 1e14, well past any real geometry. `solve_3x3` already reports
    // this through `pr = minpiv / maxpiv`; any ratio under ~1e-12 indicates
    // the three planes are effectively linearly dependent (normals coplanar)
    // and we cannot deliver a trustworthy intersection. Returning false puts
    // us on the same footing as CGAL's exact kernel, which declines these
    // intersections outright.
    return (rank == 3 && pr > 1e-12);
}

bool Intersection::ray_box(
    const Point& origin,
    const Vector& direction,
    const OBB& box,
    double t0,
    double t1,
    double& tmin,
    double& tmax) {
    
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

bool Intersection::ray_box(
    const Line& line,
    const OBB& box,
    double t0,
    double t1,
    double& tmin,
    double& tmax) {
    
    Point origin = line.start();
    Vector direction = line.to_vector();
    
    return ray_box(origin, direction, box, t0, t1, tmin, tmax);
}

bool Intersection::ray_box(
    const Line& line,
    const OBB& box,
    double t0,
    double t1,
    std::vector<Point>& intersection_points) {
    
    double tmin, tmax;
    Point origin = line.start();
    Vector direction = line.to_vector();
    
    bool hit = ray_box(origin, direction, box, t0, t1, tmin, tmax);
    
    if (hit) {
        intersection_points.clear();
        
        // Entry point
        Point entry(
            origin[0] + direction[0] * tmin,
            origin[1] + direction[1] * tmin,
            origin[2] + direction[2] * tmin
        );
        intersection_points.push_back(entry);
        
        // Exit point
        Point exit(
            origin[0] + direction[0] * tmax,
            origin[1] + direction[1] * tmax,
            origin[2] + direction[2] * tmax
        );
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
    double& t1) {
    
    Vector o(
        origin[0] - center[0],
        origin[1] - center[1],
        origin[2] - center[2]
    );
    
    double a = direction.dot(direction);
    double b = 2.0 * direction.dot(o);
    double c = o.dot(o) - (radius * radius);
    
    double disc = b * b - 4.0 * a * c;
    
    if (disc < 0.0) {
        return 0;
    }
    
    double distSqrt = std::sqrt(disc);
    double q;
    if (b < 0.0) {
        q = (-b - distSqrt) / 2.0;
    } else {
        q = (-b + distSqrt) / 2.0;
    }
    
    t0 = q / a;
    double _t1 = c / q;
    
    if (_t1 == t0) {
        return 1;
    }
    
    t1 = _t1;
    
    if (t0 > t1) {
        std::swap(t0, t1);
    }
    
    return 2;
}

bool Intersection::ray_sphere(
    const Line& line,
    const Point& center,
    double radius,
    std::vector<Point>& intersection_points) {
    
    Point origin = line.start();
    Vector direction = line.to_vector();
    
    double t0, t1;
    int hits = ray_sphere(origin, direction, center, radius, t0, t1);
    
    if (hits == 0) {
        return false;
    }
    
    intersection_points.clear();
    
    // First intersection point
    Point p0(
        origin[0] + direction[0] * t0,
        origin[1] + direction[1] * t0,
        origin[2] + direction[2] * t0
    );
    intersection_points.push_back(p0);
    
    // Second intersection point (if exists)
    if (hits == 2) {
        Point p1(
            origin[0] + direction[0] * t1,
            origin[1] + direction[1] * t1,
            origin[2] + direction[2] * t1
        );
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
    bool& parallel) {
    
    Vector edge1(v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]);
    Vector edge2(v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]);
    Vector pvec = direction.cross(edge2);
    
    double det = edge1.dot(pvec);
    
    if (det > -epsilon && det < epsilon) {
        parallel = true;
        return false;
    }
    
    parallel = false;
    double inv_det = 1.0 / det;
    
    Vector tvec(origin[0] - v0[0], origin[1] - v0[1], origin[2] - v0[2]);
    u = tvec.dot(pvec) * inv_det;
    
    if (u < 0.0 - epsilon || u > 1.0 + epsilon) {
        return false;
    }
    
    Vector qvec = tvec.cross(edge1);
    v = direction.dot(qvec) * inv_det;
    
    if (v < 0.0 - epsilon || u + v > 1.0 + epsilon) {
        return false;
    }
    
    t = edge2.dot(qvec) * inv_det;
    return true;
}

bool Intersection::ray_triangle(
    const Line& line,
    const Point& v0,
    const Point& v1,
    const Point& v2,
    double epsilon,
    Point& output) {
    
    Point origin = line.start();
    Vector direction = line.to_vector();
    
    double t, u, v;
    bool parallel;
    
    if (!ray_triangle(origin, direction, v0, v1, v2, epsilon, t, u, v, parallel)) {
        return false;
    }
    
    // Calculate intersection point: origin + t * direction
    output = Point(
        origin[0] + t * direction[0],
        origin[1] + t * direction[1],
        origin[2] + t * direction[2]
    );
    
    return true;
}

bool Intersection::ray_mesh(
    const Point& origin,
    const Vector& direction,
    const Mesh& mesh,
    std::vector<RayHit>& hits,
    bool find_all) {
    
    hits.clear();
    
    auto [vertices, faces] = mesh.to_vertices_and_faces();
    
    for (size_t i = 0; i < faces.size(); ++i) {
        const auto& face = faces[i];
        
        if (face.size() < 3) continue;
        
        for (size_t j = 1; j < face.size() - 1; ++j) {
            const Point& v0 = vertices[face[0]];
            const Point& v1 = vertices[face[j]];
            const Point& v2 = vertices[face[j + 1]];
            
            double t, u, v;
            bool parallel;
            
            if (ray_triangle(origin, direction, v0, v1, v2, 
                           static_cast<double>(Tolerance::ZERO_TOLERANCE), 
                           t, u, v, parallel)) {
                
                if (t >= 0.0) {
                    Point hit_point(
                        origin[0] + t * direction[0],
                        origin[1] + t * direction[1],
                        origin[2] + t * direction[2]
                    );
                    
                    hits.emplace_back(t, hit_point, u, v, static_cast<int>(i));
                    
                    if (!find_all) {
                        return true;
                    }
                }
            }
        }
    }
    
    if (!hits.empty()) {
        const double eps = 1e-6;
        std::sort(hits.begin(), hits.end(), 
                  [eps](const RayHit& a, const RayHit& b) {
                      double dt = a.t - b.t;
                      if (std::fabs(dt) <= eps) return a.face_index < b.face_index;
                      return a.t < b.t;
                  });
        return true;
    }
    
    return false;
}

bool Intersection::ray_mesh_bvh(
    const Point& origin,
    const Vector& direction,
    const Mesh& mesh,
    std::vector<RayHit>& hits,
    bool find_all) {
    
    hits.clear();
    
    // Use Mesh's cached per-triangle SpatialBVH
    std::vector<int> candidates_list;
    if (!mesh.triangle_bvh_ray_cast(origin, direction, candidates_list, find_all)) {
        return false;
    }
    
    // Perform exact ray-triangle intersection on candidates
    bool any_hit = false;
    RayHit best_hit;
    double best_t = std::numeric_limits<double>::infinity();
    int best_face = std::numeric_limits<int>::max();
    for (int tri_id : candidates_list) {
        size_t face_idx, sub_idx;
        Point v0, v1, v2;
        if (!mesh.get_triangle_by_id(tri_id, face_idx, sub_idx, v0, v1, v2)) {
            continue;
        }
        
        double t, u, v;
        bool parallel;
        
        if (ray_triangle(origin, direction, v0, v1, v2, 
                       static_cast<double>(Tolerance::ZERO_TOLERANCE), 
                       t, u, v, parallel)) {
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
            const double eps = 1e-6;
            std::sort(hits.begin(), hits.end(), 
                      [eps](const RayHit& a, const RayHit& b) {
                          double dt = a.t - b.t;
                          if (std::fabs(dt) <= eps) return a.face_index < b.face_index;
                          return a.t < b.t;
                      });
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

std::vector<Point> Intersection::ray_mesh(
    const Line& line,
    const Mesh& mesh,
    double epsilon,
    bool find_all) {
    
    (void)epsilon;  // Unused - kept for API compatibility
    
    Point origin = line.start();
    Vector direction = line.to_vector();
    
    std::vector<RayHit> hits;
    std::vector<Point> result;
    
    if (ray_mesh(origin, direction, mesh, hits, find_all)) {
        result.reserve(hits.size());
        for (const auto& hit : hits) {
            result.push_back(hit.point);
        }
    }
    
    return result;
}

std::vector<Point> Intersection::ray_mesh_bvh(
    const Line& line,
    const Mesh& mesh,
    double epsilon,
    bool find_all) {
    
    (void)epsilon;  // Unused - kept for API compatibility
    
    Point origin = line.start();
    Vector direction = line.to_vector();
    
    std::vector<RayHit> hits;
    std::vector<Point> result;
    
    if (ray_mesh_bvh(origin, direction, mesh, hits, find_all)) {
        result.reserve(hits.size());
        for (const auto& hit : hits) {
            result.push_back(hit.point);
        }
    }
    
    return result;
}

//==========================================================================================
// NURBS Curve Intersection Helper Functions
//==========================================================================================

namespace {
    double curve_signed_distance_to_plane(const Point& pt, const Plane& plane) {
        Vector v(pt[0] - plane.origin()[0],
                pt[1] - plane.origin()[1],
                pt[2] - plane.origin()[2]);
        return v.dot(plane.z_axis());
    }

    bool curve_find_root_bisection(const NurbsCurve& curve, const Plane& plane,
                            double t0, double t1, double tolerance,
                            double& t_result) {
        const int max_iterations = 50;
        double d0 = curve_signed_distance_to_plane(curve.point_at(t0), plane);
        double d1 = curve_signed_distance_to_plane(curve.point_at(t1), plane);

        if (d0 * d1 > 0) return false;

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

    bool curve_refine_intersection_newton(const NurbsCurve& curve, const Plane& plane,
                                   double& t, double tolerance) {
        const int max_iterations = 10;
        const double step_tolerance = tolerance * 0.01;

        for (int iter = 0; iter < max_iterations; iter++) {
            Point pt = curve.point_at(t);
            Vector tangent = curve.tangent_at(t);

            double f = curve_signed_distance_to_plane(pt, plane);
            double df = tangent.dot(plane.z_axis());

            if (std::abs(f) < tolerance) return true;
            if (std::abs(df) < 1e-12) return false;

            double dt = -f / df;
            if (std::abs(dt) < step_tolerance) return true;

            t += dt;

            auto [t0, t1] = curve.domain();
            if (t < t0) t = t0;
            if (t > t1) t = t1;
        }

        return std::abs(curve_signed_distance_to_plane(curve.point_at(t), plane)) < tolerance * 2.0;
    }
}

//==========================================================================================
// NURBS Curve Intersection Methods
//==========================================================================================

std::vector<double> Intersection::curve_plane(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance
) {
    std::vector<double> intersections;

    if (!curve.is_valid()) return intersections;
    if (tolerance <= 0.0) tolerance = Tolerance::ZERO_TOLERANCE;

    auto [t_start, t_end] = curve.domain();

    std::vector<double> span_params = curve.get_span_vector();

    for (size_t i = 0; i < span_params.size() - 1; i++) {
        double t0 = span_params[i];
        double t1 = span_params[i + 1];

        if (std::abs(t1 - t0) < tolerance) continue;

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
            if (!intersections.empty() && std::abs(intersections.back() - t0) < tolerance) {
                add = false;
            }
            if (add) intersections.push_back(t0);
        }
    }

    double d_end = curve_signed_distance_to_plane(curve.point_at(t_end), plane);
    if (std::abs(d_end) < tolerance) {
        bool add = true;
        if (!intersections.empty() && std::abs(intersections.back() - t_end) < tolerance) {
            add = false;
        }
        if (add) intersections.push_back(t_end);
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
        std::unique(intersections.begin(), intersections.end(),
                   [tolerance](double a, double b) {
                       return std::abs(a - b) < tolerance * 2.0;
                   }),
        intersections.end()
    );

    return intersections;
}

std::vector<Point> Intersection::curve_plane_points(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance
) {
    std::vector<double> params = curve_plane(curve, plane, tolerance);
    std::vector<Point> points;
    points.reserve(params.size());

    for (double t : params) {
        points.push_back(curve.point_at(t));
    }

    return points;
}

std::vector<double> Intersection::curve_plane_bezier_clipping(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance
) {
    std::vector<double> results;

    if (!curve.is_valid()) return results;
    if (tolerance <= 0.0) tolerance = Tolerance::ZERO_TOLERANCE;

    auto [t0, t1] = curve.domain();

    auto signed_distance = [&](const Point& p) -> double {
        Vector v(p[0] - plane.origin()[0],
                p[1] - plane.origin()[1],
                p[2] - plane.origin()[2]);
        return v.dot(plane.z_axis());
    };

    std::function<void(double, double, int)> clip_recursive;
    clip_recursive = [&](double ta, double tb, int depth) {
        if (depth > 50) {
            double tm = (ta + tb) * 0.5;
            Point pm = curve.point_at(tm);
            double dist = signed_distance(pm);

            if (std::abs(dist) < tolerance) {
                results.push_back(tm);
            }
            return;
        }

        if (std::abs(tb - ta) < tolerance * 0.01) {
            double tm = (ta + tb) * 0.5;
            Point pm = curve.point_at(tm);
            double dist = signed_distance(pm);

            if (std::abs(dist) < tolerance) {
                double t = tm;
                for (int iter = 0; iter < 10; iter++) {
                    Point pt = curve.point_at(t);
                    Vector tangent = curve.tangent_at(t);

                    double f = signed_distance(pt);
                    double df = tangent.dot(plane.z_axis());

                    if (std::abs(df) < 1e-12) break;

                    double dt = -f / df;
                    t += dt;

                    if (std::abs(dt) < tolerance * 0.01) break;
                    if (t < ta || t > tb) {
                        t = tm;
                        break;
                    }
                }

                Point pt_final = curve.point_at(t);
                if (std::abs(signed_distance(pt_final)) < tolerance && t >= ta && t <= tb) {
                    results.push_back(t);
                }
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
            distances.push_back(signed_distance(p));
            params.push_back(t);
        }

        double d_min = *std::min_element(distances.begin(), distances.end());
        double d_max = *std::max_element(distances.begin(), distances.end());

        if (d_min > tolerance || d_max < -tolerance) {
            return;
        }

        double t_min = ta;
        double t_max = tb;

        for (size_t i = 0; i < distances.size() - 1; i++) {
            if (distances[i] * distances[i + 1] < 0) {
                double d0 = distances[i];
                double d1 = distances[i + 1];
                double t_clip = params[i] - d0 * (params[i + 1] - params[i]) / (d1 - d0);

                if (d0 > 0) {
                    t_max = std::min(t_max, t_clip + (tb - ta) * 0.1);
                } else {
                    t_min = std::max(t_min, t_clip - (tb - ta) * 0.1);
                }
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
            clip_recursive(ta, tm, depth + 1);
            clip_recursive(tm, tb, depth + 1);
        } else {
            clip_recursive(t_min, t_max, depth + 1);
        }
    };

    clip_recursive(t0, t1, 0);

    std::sort(results.begin(), results.end());

    auto last = std::unique(results.begin(), results.end(),
                           [tolerance](double a, double b) {
                               return std::abs(a - b) < tolerance * 2.0;
                           });
    results.erase(last, results.end());

    return results;
}

std::vector<double> Intersection::curve_plane_algebraic(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance
) {
    if (!curve.is_valid()) return {};

    std::vector<double> results;

    std::vector<double> spans = curve.get_span_vector();
    if (spans.size() < 2) return {};

    for (size_t i = 0; i < spans.size() - 1; i++) {
        double span_t0 = spans[i];
        double span_t1 = spans[i + 1];

        if (std::abs(span_t1 - span_t0) < tolerance) continue;

        std::function<void(double, double, int)> subdivide = [&](double a, double b, int depth) {
            if (depth > 30) return;

            Point p_a = curve.point_at(a);
            Point p_b = curve.point_at(b);

            Vector normal = plane.z_axis();
            double f_a = normal.dot(p_a - plane.origin());
            double f_b = normal.dot(p_b - plane.origin());

            if (f_a * f_b > 0) return;

            double mid_t = (a + b) * 0.5;
            Point p_mid = curve.point_at(mid_t);

            Vector line_dir = p_b - p_a;
            double line_len = line_dir.magnitude();
            if (line_len > 1e-14) {
                line_dir = line_dir / line_len;
            }
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

                    if (t_new < a || t_new > b) {
                        t_new = (a + b) * 0.5;
                    }

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
                    if (!is_duplicate) {
                        results.push_back(t);
                    }
                }
            } else {
                subdivide(a, mid_t, depth + 1);
                subdivide(mid_t, b, depth + 1);
            }
        };

        subdivide(span_t0, span_t1, 0);
    }

    std::sort(results.begin(), results.end());
    results.erase(std::unique(results.begin(), results.end(),
                              [tolerance](double a, double b) {
                                  return std::abs(a - b) < tolerance * 10.0;
                              }), results.end());

    return results;
}

std::vector<double> Intersection::curve_plane_production(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance
) {
    if (!curve.is_valid()) return {};

    std::vector<double> results;

    std::vector<double> spans = curve.get_span_vector();
    if (spans.size() < 2) return {};

    auto is_nearly_linear = [&curve, tolerance](double a, double b) -> bool {
        Point p_a = curve.point_at(a);
        Point p_b = curve.point_at(b);
        Point p_mid = curve.point_at((a + b) * 0.5);

        Vector ab = p_b - p_a;
        double line_length = ab.magnitude();
        if (line_length < 1e-14) return true;

        Vector am = p_mid - p_a;
        double cross_mag = ab.cross(am).magnitude();
        double deviation = cross_mag / line_length;

        return deviation < tolerance * 10.0;
    };

    std::function<void(double, double, int)> subdivide = [&](double a, double b, int depth) {
        if (depth > 30) return;

        Point p_a = curve.point_at(a);
        Point p_b = curve.point_at(b);

        Vector normal = plane.z_axis();
        double f_a = normal.dot(p_a - plane.origin());
        double f_b = normal.dot(p_b - plane.origin());

        if (f_a * f_b > 0) return;

        if (is_nearly_linear(a, b) || (b - a) < tolerance * 10.0) {
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

                if (t_new < a || t_new > b) {
                    t_new = (a + b) * 0.5;
                }

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
                if (!is_duplicate) {
                    results.push_back(t);
                }
            }
        } else {
            double mid = (a + b) * 0.5;
            subdivide(a, mid, depth + 1);
            subdivide(mid, b, depth + 1);
        }
    };

    for (size_t i = 0; i < spans.size() - 1; i++) {
        double span_t0 = spans[i];
        double span_t1 = spans[i + 1];

        if (std::abs(span_t1 - span_t0) < tolerance) continue;

        subdivide(span_t0, span_t1, 0);
    }

    std::sort(results.begin(), results.end());
    results.erase(std::unique(results.begin(), results.end(),
                              [tolerance](double a, double b) {
                                  return std::abs(a - b) < tolerance * 10.0;
                              }), results.end());

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

//==========================================================================================
// NURBS Surface Intersection Methods (Curve Tracing)
//
// Algorithm: predictor-corrector marching on the implicit function
//   g(u,v) = n . (S(u,v) - P0)
// where n = plane normal, P0 = plane origin, S = surface point_at.
//
// 1. Seed: sample g on coarse UV grid, find sign changes, Newton-refine
// 2. Trace: march along g=0 using tangent (-gv, gu), Newton-correct each step
// 3. Fit: create_interpolated through traced 3D points → smooth NurbsCurve
//
// References:
//   OpenCASCADE IntPatch_ImpPrmIntersection / IWalking
//   Barnhill et al. "Surface/Surface Intersection" CAGD 1987
//==========================================================================================

namespace {

struct SurfacePlaneTrace {
    std::vector<std::pair<double, double>> uv_trace;
    std::vector<std::pair<double, double>> uv_unwrapped;
    bool is_loop;
};

struct SurfacePlaneTraceResult {
    std::vector<SurfacePlaneTrace> traces;
    double step;
    double uv_to_3d;
    double uv_to_3d_min;
};

// Seed and trace surface/plane intersection curves in UV space.
//
// Returns traces (wrapped UV samples, seam-unwrapped UV samples, loop flag)
// plus the marching step and UV-to-3D scale factors.
SurfacePlaneTraceResult surface_plane_traces(
    const NurbsSurface& surface,
    const Plane& plane,
    double tolerance
) {
    auto [u0, u1] = surface.domain(0);
    auto [v0, v1] = surface.domain(1);
    double range_u = u1 - u0;
    double range_v = v1 - v0;
    bool closed_u = surface.is_closed(0);
    bool closed_v = surface.is_closed(1);

    // Wrap for closed directions, clamp for open
    auto wrap_u = [&](double u) -> double {
        if (closed_u) {
            double t = std::fmod(u - u0, range_u);
            if (t < 0) t += range_u;
            return u0 + t;
        }
        return std::max(u0, std::min(u, u1));
    };
    auto wrap_v = [&](double v) -> double {
        if (closed_v) {
            double t = std::fmod(v - v0, range_v);
            if (t < 0) t += range_v;
            return v0 + t;
        }
        return std::max(v0, std::min(v, v1));
    };

    // Plane normal and origin for analytical gradient
    Vector pn = plane.z_axis();
    Point p0 = plane.origin();

    // Analytical g(u,v) = signed distance from S(u,v) to plane
    auto g = [&](double u, double v) -> double {
        Point p = surface.point_at(wrap_u(u), wrap_v(v));
        return (p[0]-p0[0])*pn[0] + (p[1]-p0[1])*pn[1] + (p[2]-p0[2])*pn[2];
    };

    // Analytical g + gradient via single evaluate call
    // evaluate order: [S, Sv, Su] for num_derivs=1
    auto g_and_grad = [&](double u, double v, double& val, double& gu, double& gv) {
        auto derivs = surface.evaluate(wrap_u(u), wrap_v(v), 1);
        const Vector& S = derivs[0];
        const Vector& Su = derivs[2];
        const Vector& Sv = derivs[1];
        val = (S[0]-p0[0])*pn[0] + (S[1]-p0[1])*pn[1] + (S[2]-p0[2])*pn[2];
        gu = Su[0]*pn[0] + Su[1]*pn[1] + Su[2]*pn[2];
        gv = Sv[0]*pn[0] + Sv[1]*pn[1] + Sv[2]*pn[2];
    };

    // Newton correction with analytical Jacobian
    auto newton_correct = [&](double& u, double& v) -> bool {
        for (int iter = 0; iter < 10; iter++) {
            double val, gu, gv;
            g_and_grad(u, v, val, gu, gv);
            if (std::abs(val) < tolerance) return true;
            double mag2 = gu * gu + gv * gv;
            if (mag2 < 1e-28) return false;
            u -= val * gu / mag2;
            v -= val * gv / mag2;
            u = wrap_u(u);
            v = wrap_v(v);
        }
        return std::abs(g(u, v)) < tolerance * 10.0;
    };

    // ------------------------------------------------------------------
    // 1. Find seeds: coarse UV grid, detect sign changes, Newton-refine
    // ------------------------------------------------------------------
    auto spans_u = surface.get_span_vector(0);
    auto spans_v = surface.get_span_vector(1);
    int nu = std::max((int)spans_u.size() - 1, 1) * 4;
    int nv = std::max((int)spans_v.size() - 1, 1) * 4;
    double du = range_u / nu;
    double dv = range_v / nv;

    // UV-to-3D scale for tolerance conversion
    double mu = (u0 + u1) * 0.5, mv = (v0 + v1) * 0.5;
    Point pmid = surface.point_at(mu, mv);
    double uv_to_3d_u = pmid.distance(surface.point_at(wrap_u(mu + du), mv)) / du;
    double uv_to_3d_v = pmid.distance(surface.point_at(mu, wrap_v(mv + dv))) / dv;
    double uv_to_3d = std::max(uv_to_3d_u, uv_to_3d_v);
    double uv_to_3d_min = std::min(uv_to_3d_u, uv_to_3d_v);
    if (uv_to_3d < 1e-10) uv_to_3d = 1.0;
    if (uv_to_3d_min < 1e-10) uv_to_3d_min = 1.0;

    int cols = nv + 1;
    std::vector<double> dist((nu + 1) * cols);
    for (int i = 0; i <= nu; i++) {
        double u = u0 + du * i;
        for (int j = 0; j <= nv; j++) {
            double v = v0 + dv * j;
            double d = g(u, v);
            if (d == 0.0) d = -1e-14; // perturb exact zeros for sign-change detection
            dist[i * cols + j] = d;
        }
    }

    struct Seed { double u, v; bool used; };
    std::vector<Seed> seeds;

    // Horizontal edges (skip duplicate boundary for closed directions)
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
    // Vertical edges (skip duplicate boundary for closed directions)
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

    // Deduplicate seeds (3D distance — robust across seams)
    double seed_tol_3d = std::max(du, dv) * uv_to_3d;
    for (size_t i = 0; i < seeds.size(); i++) {
        if (seeds[i].used) continue;
        Point pi = surface.point_at(seeds[i].u, seeds[i].v);
        for (size_t j = i + 1; j < seeds.size(); j++) {
            if (seeds[j].used) continue;
            if (pi.distance(surface.point_at(seeds[j].u, seeds[j].v)) < seed_tol_3d)
                seeds[j].used = true;
        }
    }

    // ------------------------------------------------------------------
    // 2. Trace intersection curves via predictor-corrector marching
    // ------------------------------------------------------------------
    double step = std::min(du, dv) * 0.25;
    int max_steps = nu * nv * 32;
    double close_tol_3d = step * 4.0 * uv_to_3d_min;
    double consume_tol_3d = step * uv_to_3d * 2.0;

    std::vector<SurfacePlaneTrace> traces;

    for (auto& seed : seeds) {
        if (seed.used) continue;
        seed.used = true;

        // Compute tangent direction from analytical gradient
        auto tangent_at_uv = [&](double u, double v, int dir, double& tu, double& tv) -> bool {
            double val, gu, gv;
            g_and_grad(u, v, val, gu, gv);
            double mag = std::hypot(gu, gv);
            if (mag < 1e-14) return false;
            tu = -gv / mag * dir;
            tv =  gu / mag * dir;
            return true;
        };

        // Trace one direction; returns true if loop closed
        auto trace_dir = [&](double su, double sv, int dir,
                             std::vector<std::pair<double, double>>& out) -> bool {
            double u = su, v = sv;
            double prev_tu = 0, prev_tv = 0;
            Point p_start = surface.point_at(su, sv);
            Point p_prev = p_start;
            double dist_traveled = 0;
            for (int s = 0; s < max_steps; s++) {
                // Compute tangent at current point
                double tu, tv;
                if (!tangent_at_uv(u, v, dir, tu, tv)) {
                    if (std::hypot(prev_tu, prev_tv) < 1e-14) break;
                    tu = prev_tu; tv = prev_tv;
                }

                // Adaptive step: reduce in high-curvature regions (dot product of consecutive tangents)
                double local_step = step;
                if (std::hypot(prev_tu, prev_tv) > 1e-14) {
                    double dot = tu * prev_tu + tv * prev_tv;
                    dot = std::max(-1.0, std::min(1.0, dot));
                    if (dot < 0.95) local_step = step * 0.25;
                    else if (dot < 0.985) local_step = step * 0.5;
                }

                // RK2: use midpoint tangent for the actual step
                double u_mid = u + local_step * 0.5 * tu;
                double v_mid = v + local_step * 0.5 * tv;
                double tu2, tv2;
                if (tangent_at_uv(u_mid, v_mid, dir, tu2, tv2)) {
                    tu = tu2; tv = tv2;
                }
                prev_tu = tu; prev_tv = tv;

                double un = u + local_step * tu;
                double vn = v + local_step * tv;

                bool hit_boundary = false;
                if ((!closed_u && (un < u0 || un > u1)) ||
                    (!closed_v && (vn < v0 || vn > v1))) {
                    double tc = 1.0;
                    if (!closed_u && tu > 0 && un > u1) tc = std::min(tc, (u1 - u) / (local_step * tu));
                    if (!closed_u && tu < 0 && un < u0) tc = std::min(tc, (u0 - u) / (local_step * tu));
                    if (!closed_v && tv > 0 && vn > v1) tc = std::min(tc, (v1 - v) / (local_step * tv));
                    if (!closed_v && tv < 0 && vn < v0) tc = std::min(tc, (v0 - v) / (local_step * tv));
                    un = u + tc * local_step * tu;
                    vn = v + tc * local_step * tv;
                    hit_boundary = true;
                }
                un = wrap_u(un);
                vn = wrap_v(vn);

                if (!newton_correct(un, vn)) break;

                Point p_cur = surface.point_at(un, vn);
                dist_traveled += p_prev.distance(p_cur);

                // Loop closure: only check after traveling ≥ 3× the close tolerance
                if (dist_traveled > close_tol_3d * 3.0 &&
                    p_start.distance(p_cur) < close_tol_3d) {
                    out.push_back({un, vn});
                    return true;
                }

                out.push_back({un, vn});
                u = un; v = vn;
                p_prev = p_cur;

                if (hit_boundary) break;

                for (auto& other : seeds) {
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

        // Assemble UV trace: reverse(bwd) + seed + fwd
        std::vector<std::pair<double, double>> uv_trace;
        uv_trace.reserve(bwd.size() + 1 + fwd.size());
        for (int i = (int)bwd.size() - 1; i >= 0; i--)
            uv_trace.push_back(bwd[i]);
        uv_trace.push_back({seed.u, seed.v});
        for (auto& p : fwd)
            uv_trace.push_back(p);

        if (uv_trace.size() < 4) continue;

        // Detect closed loops
        Point p_first = surface.point_at(uv_trace.front().first, uv_trace.front().second);
        Point p_last = surface.point_at(uv_trace.back().first, uv_trace.back().second);
        bool is_loop = fwd_closed || (uv_trace.size() >= 6 && p_first.distance(p_last) < close_tol_3d);
        if (is_loop) uv_trace.pop_back();
        if (uv_trace.size() < 4) continue;

        // Unwrap UV trace for smooth interpolation across seams
        std::vector<std::pair<double, double>> uv_unwrapped = uv_trace;
        for (size_t i = 1; i < uv_unwrapped.size(); i++) {
            double du_jump = uv_unwrapped[i].first - uv_unwrapped[i - 1].first;
            double dv_jump = uv_unwrapped[i].second - uv_unwrapped[i - 1].second;
            if (closed_u) {
                if (du_jump > range_u * 0.5) uv_unwrapped[i].first -= range_u;
                else if (du_jump < -range_u * 0.5) uv_unwrapped[i].first += range_u;
            }
            if (closed_v) {
                if (dv_jump > range_v * 0.5) uv_unwrapped[i].second -= range_v;
                else if (dv_jump < -range_v * 0.5) uv_unwrapped[i].second += range_v;
            }
        }

        traces.push_back({std::move(uv_trace), std::move(uv_unwrapped), is_loop});
    }

    return {std::move(traces), step, uv_to_3d, uv_to_3d_min};
}

// Fit a 3D plane-constrained NurbsCurve to traced intersection points.
//
// Tries exact circle recognition, then ellipse recognition for closed loops
// (when allow_conics), then adaptive plane-constrained least-squares fitting.
// Returns an invalid curve on failure.
NurbsCurve surface_plane_fit_3d(
    const std::vector<Point>& all_pts,
    bool is_loop,
    const Plane& plane,
    double step,
    double uv_to_3d,
    double uv_to_3d_min,
    bool allow_conics = true
) {
    // ------------------------------------------------------------------
    // 4. Circle detection: if points lie on a circle → exact rational NURBS
    // ------------------------------------------------------------------
    NurbsCurve crv;
    if (allow_conics && is_loop && all_pts.size() >= 6) {
        // Circle detection using 3 well-spaced points → circumscribed circle
        // Project onto cutting plane 2D coordinates
        Vector ax = plane.x_axis(), ay = plane.y_axis();
        Point po = plane.origin();
        auto to2d = [&](const Point& p) -> std::pair<double, double> {
            double dx = p[0] - po[0], dy = p[1] - po[1], dz = p[2] - po[2];
            return {dx*ax[0] + dy*ax[1] + dz*ax[2], dx*ay[0] + dy*ay[1] + dz*ay[2]};
        };

        // Pick 3 well-spaced points: 0, N/3, 2N/3
        int n = (int)all_pts.size();
        auto [x1, y1] = to2d(all_pts[0]);
        auto [x2, y2] = to2d(all_pts[n / 3]);
        auto [x3, y3] = to2d(all_pts[2 * n / 3]);

        // 2D circumcenter
        double ax_ = x2 - x1, ay_ = y2 - y1;
        double bx_ = x3 - x1, by_ = y3 - y1;
        double D = 2.0 * (ax_ * by_ - ay_ * bx_);

        if (std::abs(D) > 1e-10) {
            double a2 = ax_ * ax_ + ay_ * ay_;
            double b2 = bx_ * bx_ + by_ * by_;
            double ccx = x1 + (by_ * a2 - ay_ * b2) / D;
            double ccy = y1 + (ax_ * b2 - bx_ * a2) / D;
            double radius = std::hypot(x1 - ccx, y1 - ccy);

            // Check all points against this circle (in 2D plane coords)
            double max_dev = 0;
            for (auto& p : all_pts) {
                auto [px, py] = to2d(p);
                max_dev = std::max(max_dev, std::abs(std::hypot(px - ccx, py - ccy) - radius));
            }

            double circle_tol = std::max(radius * 1e-4, 1e-6);
            if (radius > 1e-10 && max_dev < circle_tol) {
                // Convert 2D center back to 3D
                double cx3d = po[0] + ccx * ax[0] + ccy * ay[0];
                double cy3d = po[1] + ccx * ax[1] + ccy * ay[1];
                double cz3d = po[2] + ccx * ax[2] + ccy * ay[2];

                const double w = std::sqrt(2.0) / 2.0;
                double cx_[] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
                double cy_[] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
                double wts[] = {1, w, 1, w, 1, w, 1, w, 1};
                crv = NurbsCurve(3, true, 3, 9);
                double nurbsknots[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
                for (int i = 0; i < 10; i++) crv.set_nurbsknot(i, nurbsknots[i]);
                for (int i = 0; i < 9; i++) {
                    double px = cx3d + radius * (cx_[i] * ax[0] + cy_[i] * ay[0]);
                    double py = cy3d + radius * (cx_[i] * ax[1] + cy_[i] * ay[1]);
                    double pz = cz3d + radius * (cx_[i] * ax[2] + cy_[i] * ay[2]);
                    crv.set_cv_4d(i, px * wts[i], py * wts[i], pz * wts[i], wts[i]);
                }
            }
        }
    }

    // ------------------------------------------------------------------
    // 4b. Ellipse (conic) detection for non-circular closed curves
    // ------------------------------------------------------------------
    if (!crv.is_valid() && allow_conics && is_loop && all_pts.size() >= 8) {
        Vector ax = plane.x_axis(), ay = plane.y_axis();
        Point po = plane.origin();
        auto to2d = [&](const Point& p) -> std::pair<double, double> {
            double dx = p[0] - po[0], dy = p[1] - po[1], dz = p[2] - po[2];
            return {dx*ax[0] + dy*ax[1] + dz*ax[2], dx*ay[0] + dy*ay[1] + dz*ay[2]};
        };

        // Fit general conic: A*x² + B*xy + C*y² + D*x + E*y = 1
        // Least squares: M * [A B C D E]^T = [1 1 ... 1]^T
        int n = (int)all_pts.size();
        // Build normal equations (5x5 symmetric system)
        double AtA[5][5] = {}, Atb[5] = {};
        for (int i = 0; i < n; i++) {
            auto [x, y] = to2d(all_pts[i]);
            double row[5] = {x*x, x*y, y*y, x, y};
            for (int r = 0; r < 5; r++) {
                Atb[r] += row[r];
                for (int c = 0; c < 5; c++)
                    AtA[r][c] += row[r] * row[c];
            }
        }
        // Solve 5x5 system by Gaussian elimination
        double M[5][6];
        for (int r = 0; r < 5; r++) {
            for (int c = 0; c < 5; c++) M[r][c] = AtA[r][c];
            M[r][5] = Atb[r];
        }
        bool ok = true;
        for (int col = 0; col < 5 && ok; col++) {
            int pivot = col;
            for (int r = col + 1; r < 5; r++)
                if (std::fabs(M[r][col]) > std::fabs(M[pivot][col])) pivot = r;
            if (std::fabs(M[pivot][col]) < 1e-20) { ok = false; break; }
            if (pivot != col) for (int j = col; j <= 5; j++) std::swap(M[col][j], M[pivot][j]);
            for (int r = col + 1; r < 5; r++) {
                double f = M[r][col] / M[col][col];
                for (int j = col; j <= 5; j++) M[r][j] -= f * M[col][j];
            }
        }
        double coef[5] = {};
        if (ok) {
            for (int i = 4; i >= 0; i--) {
                double s = M[i][5];
                for (int j = i + 1; j < 5; j++) s -= M[i][j] * coef[j];
                coef[i] = s / M[i][i];
            }
        }
        double A = coef[0], B = coef[1], C = coef[2], D = coef[3], E = coef[4];
        double disc = B * B - 4 * A * C;

        if (ok && disc < -1e-10 && std::fabs(A) > 1e-14) {
            // It's an ellipse — check fit quality
            double max_conic_dev = 0;
            for (auto& p : all_pts) {
                auto [x, y] = to2d(p);
                double val = A*x*x + B*x*y + C*y*y + D*x + E*y - 1.0;
                max_conic_dev = std::max(max_conic_dev, std::fabs(val));
            }

            // Normalize deviation by scale
            double scale = std::max({std::fabs(A), std::fabs(C)});
            double norm_dev = max_conic_dev / std::max(scale, 1e-10);

            if (norm_dev < 0.01) {
                // Convert to canonical form: center, semi-axes, rotation
                // Center: ∂f/∂x = 2Ax + By + D = 0, ∂f/∂y = Bx + 2Cy + E = 0
                double det = 4*A*C - B*B;
                double cx = (B*E - 2*C*D) / det;
                double cy = (B*D - 2*A*E) / det;

                // Rotation angle of principal axes
                double theta = 0.5 * std::atan2(B, A - C);

                // Translate & rotate conic to get semi-axes
                double cos_t = std::cos(theta), sin_t = std::sin(theta);
                double A2 = A*cos_t*cos_t + B*cos_t*sin_t + C*sin_t*sin_t;
                double C2 = A*sin_t*sin_t - B*cos_t*sin_t + C*cos_t*cos_t;
                double f_val = A*cx*cx + B*cx*cy + C*cy*cy + D*cx + E*cy - 1.0;
                double rhs = -f_val;
                if (rhs > 1e-14 && A2 > 1e-14 && C2 > 1e-14) {
                    double semi_a = std::sqrt(rhs / A2);
                    double semi_b = std::sqrt(rhs / C2);

                    // Build rational NURBS ellipse (9 CVs, degree 2)
                    double cx3d = po[0] + cx*ax[0] + cy*ay[0];
                    double cy3d = po[1] + cx*ax[1] + cy*ay[1];
                    double cz3d = po[2] + cx*ax[2] + cy*ay[2];

                    // Rotated axes in 3D plane
                    Vector ea, eb;
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
                    for (int i = 0; i < 10; i++) crv.set_nurbsknot(i, nurbsknots[i]);
                    for (int i = 0; i < 9; i++) {
                        double px = cx3d + semi_a * cx_[i] * ea[0] + semi_b * cy_[i] * eb[0];
                        double py = cy3d + semi_a * cx_[i] * ea[1] + semi_b * cy_[i] * eb[1];
                        double pz = cz3d + semi_a * cx_[i] * ea[2] + semi_b * cy_[i] * eb[2];
                        crv.set_cv_4d(i, px * wts[i], py * wts[i], pz * wts[i], wts[i]);
                    }

                    // Verify: check max deviation of traced points from ellipse
                    // Use closest-point by angle from center in 2D
                    auto [et0, et1] = crv.domain();
                    double max_ell_dev = 0;
                    for (auto& p : all_pts) {
                        auto [px2, py2] = to2d(p);
                        // Rotate to ellipse-local coordinates
                        double lx = cos_t * (px2 - cx) + sin_t * (py2 - cy);
                        double ly = -sin_t * (px2 - cx) + cos_t * (py2 - cy);
                        double ang = std::atan2(ly / semi_b, lx / semi_a);
                        // Closest point on ellipse
                        double ex = cx + semi_a * std::cos(ang) * cos_t - semi_b * std::sin(ang) * sin_t;
                        double ey = cy + semi_a * std::cos(ang) * sin_t + semi_b * std::sin(ang) * cos_t;
                        double dev = std::hypot(px2 - ex, py2 - ey);
                        max_ell_dev = std::max(max_ell_dev, dev);
                    }
                    double ell_tol = std::max(semi_a, semi_b) * 5e-3;
                    if (max_ell_dev > ell_tol) crv = NurbsCurve(); // reject
                }
            }
        }
    }

    // ------------------------------------------------------------------
    // 5. 2D plane-constrained fitting for non-circular/elliptical curves
    // ------------------------------------------------------------------
    if (!crv.is_valid()) {
        int m = (int)all_pts.size();
        if (m < 4) return NurbsCurve();

        // Project traced 3D points → 2D plane coordinates
        Vector ax = plane.x_axis(), ay = plane.y_axis();
        Point po = plane.origin();
        std::vector<Point> pts_2d(m);
        for (int i = 0; i < m; i++) {
            double dx = all_pts[i][0]-po[0], dy = all_pts[i][1]-po[1], dz = all_pts[i][2]-po[2];
            double px = dx*ax[0] + dy*ax[1] + dz*ax[2];
            double py = dx*ay[0] + dy*ay[1] + dz*ay[2];
            pts_2d[i] = Point(px, py, 0);
        }

        // Chord-length params for deviation sampling
        std::vector<double> chords(m, 0.0);
        double total_len = 0;
        for (int i = 1; i < m; i++) {
            total_len += pts_2d[i].distance(pts_2d[i-1]);
            chords[i] = total_len;
        }
        if (is_loop && m > 1) total_len += pts_2d[0].distance(pts_2d[m-1]);
        if (total_len > 1e-14) for (int i = 1; i < m; i++) chords[i] /= total_len;

        double fit_tol = step * (uv_to_3d + uv_to_3d_min) * 0.5;
        double total_turning = 0;
        for (int i = 1; i < m - 1; i++) {
            double dx1 = pts_2d[i][0]-pts_2d[i-1][0], dy1 = pts_2d[i][1]-pts_2d[i-1][1];
            double dx2 = pts_2d[i+1][0]-pts_2d[i][0], dy2 = pts_2d[i+1][1]-pts_2d[i][1];
            double l1 = std::hypot(dx1, dy1), l2 = std::hypot(dx2, dy2);
            if (l1 > 1e-14 && l2 > 1e-14) {
                double c = (dx1*dx2+dy1*dy2) / (l1*l2);
                c = std::max(-1.0, std::min(1.0, c));
                total_turning += std::acos(c);
            }
        }
        int target_cvs = std::max(8, (int)(total_turning / 0.5) + 6);
        int max_cvs = m - 1;
        NurbsCurve crv_2d;
        for (int attempt = 0; attempt < 5; attempt++) {
            if (target_cvs > max_cvs) break;
            crv_2d = NurbsCurve::create_fitted(pts_2d, target_cvs, 3, is_loop);
            if (!crv_2d.is_valid()) break;
            auto [ft0, ft1] = crv_2d.domain();
            double max_dev = 0;
            for (int i = 0; i < m; i++) {
                double t = ft0 + (ft1 - ft0) * chords[i];
                max_dev = std::max(max_dev, crv_2d.point_at(t).distance(pts_2d[i]));
            }
            if (max_dev < fit_tol) break;
            target_cvs = std::min(target_cvs * 2, max_cvs);
        }
        if (!crv_2d.is_valid())
            crv_2d = is_loop
                ? NurbsCurve::create_interpolated(pts_2d, CurveNurbsKnotStyle::ChordPeriodic)
                : NurbsCurve::create_interpolated(pts_2d);

        // Lift 2D CVs back to 3D (exactly on plane)
        if (crv_2d.is_valid()) {
            crv = crv_2d;
            for (int i = 0; i < crv.cv_count(); i++) {
                Point cv2 = crv.get_cv(i);
                double cx = cv2[0], cy = cv2[1];
                crv.set_cv(i, Point(po[0] + cx*ax[0] + cy*ay[0],
                                    po[1] + cx*ax[1] + cy*ay[1],
                                    po[2] + cx*ax[2] + cy*ay[2]));
            }
        }
    }

    return crv;
}

// Solve an n x n linear system by Gaussian elimination with partial pivoting.
// Returns false on singular (mirrors Python _solve_gauss returning None).
bool solve_gauss(const std::vector<std::vector<double>>& M, const std::vector<double>& rhs, int n, std::vector<double>& out) {
    std::vector<std::vector<double>> A(n, std::vector<double>(n + 1));
    for (int r = 0; r < n; r++) {
        for (int c = 0; c < n; c++) A[r][c] = M[r][c];
        A[r][n] = rhs[r];
    }
    for (int col = 0; col < n; col++) {
        int pivot = col;
        for (int r = col + 1; r < n; r++) {
            if (std::abs(A[r][col]) > std::abs(A[pivot][col])) pivot = r;
        }
        if (std::abs(A[pivot][col]) < 1e-20) return false;
        if (pivot != col) std::swap(A[col], A[pivot]);
        for (int r = col + 1; r < n; r++) {
            double f = A[r][col] / A[col][col];
            for (int j = col; j < n + 1; j++) A[r][j] -= f * A[col][j];
        }
    }
    out.assign(n, 0.0);
    for (int i = n - 1; i >= 0; i--) {
        double s = A[i][n];
        for (int j = i + 1; j < n; j++) s -= A[i][j] * out[j];
        out[i] = s / A[i][i];
    }
    return true;
}

// ===========================================================================
// Analytic (closed-form) surface/surface intersection for recognized quadric
// pairs (exact conics). Mirrors session_py/src/session_py/intersection.py.
// ===========================================================================

using V3 = std::array<double, 3>;

static double ssi_dot(const V3& u, const V3& v) { return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]; }
static V3 ssi_cross(const V3& u, const V3& v) {
    return V3{u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]};
}
static V3 ssi_unit(const V3& v) {
    double l = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    return l > 1e-300 ? V3{v[0]/l, v[1]/l, v[2]/l} : v;
}

// Two unit vectors spanning the plane perpendicular to unit n.
static std::pair<V3, V3> ortho_basis(const V3& n) {
    double ax = (std::abs(n[0]) <= std::abs(n[1]) && std::abs(n[0]) <= std::abs(n[2])) ? 1.0 : 0.0;
    double ay = (ax == 0.0 && std::abs(n[1]) <= std::abs(n[2])) ? 1.0 : 0.0;
    double az = (ax == 0.0 && ay == 0.0) ? 1.0 : 0.0;
    double ux = ay*n[2] - az*n[1];
    double uy = az*n[0] - ax*n[2];
    double uz = ax*n[1] - ay*n[0];
    double ul = std::sqrt(ux*ux + uy*uy + uz*uz);
    ux /= ul; uy /= ul; uz /= ul;
    double vx = n[1]*uz - n[2]*uy;
    double vy = n[2]*ux - n[0]*uz;
    double vz = n[0]*uy - n[1]*ux;
    return {V3{ux, uy, uz}, V3{vx, vy, vz}};
}

// Exact 9-CV rational NURBS circle.
static NurbsCurve exact_circle(double cx, double cy, double cz, const V3& xa, const V3& ya, double radius) {
    double w = std::sqrt(2.0) / 2.0;
    double px[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    double py[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    double wts[9] = {1, w, 1, w, 1, w, 1, w, 1};
    NurbsCurve crv(3, true, 3, 9);
    double knots[10] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
    for (int i = 0; i < 10; i++) crv.set_nurbsknot(i, knots[i]);
    for (int i = 0; i < 9; i++) {
        double x = cx + radius * (px[i]*xa[0] + py[i]*ya[0]);
        double y = cy + radius * (px[i]*xa[1] + py[i]*ya[1]);
        double z = cz + radius * (px[i]*xa[2] + py[i]*ya[2]);
        crv.set_cv_4d(i, x*wts[i], y*wts[i], z*wts[i], wts[i]);
    }
    crv.set_domain(0.0, 1.0);
    return crv;
}

// Exact 9-CV rational NURBS ellipse.
static NurbsCurve exact_ellipse(double cx, double cy, double cz, const V3& ea, const V3& eb,
                                double semi_a, double semi_b) {
    double w = std::sqrt(2.0) / 2.0;
    double px[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    double py[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    double wts[9] = {1, w, 1, w, 1, w, 1, w, 1};
    NurbsCurve crv(3, true, 3, 9);
    double knots[10] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
    for (int i = 0; i < 10; i++) crv.set_nurbsknot(i, knots[i]);
    for (int i = 0; i < 9; i++) {
        double x = cx + semi_a*px[i]*ea[0] + semi_b*py[i]*eb[0];
        double y = cy + semi_a*px[i]*ea[1] + semi_b*py[i]*eb[1];
        double z = cz + semi_a*px[i]*ea[2] + semi_b*py[i]*eb[2];
        crv.set_cv_4d(i, x*wts[i], y*wts[i], z*wts[i], wts[i]);
    }
    crv.set_domain(0.0, 1.0);
    return crv;
}

// Eigenvalues/vectors of a symmetric 3x3 matrix (cyclic Jacobi).
static void jacobi_eig3(const double M[3][3], double eigvals[3], V3 eigvecs[3]) {
    double a[3][3], v[3][3];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) { a[r][c] = M[r][c]; v[r][c] = (r == c) ? 1.0 : 0.0; }
    for (int it = 0; it < 50; it++) {
        double off = std::abs(a[0][1]) + std::abs(a[0][2]) + std::abs(a[1][2]);
        if (off < 1e-18) break;
        int pq[3][2] = {{0, 1}, {0, 2}, {1, 2}};
        for (int idx = 0; idx < 3; idx++) {
            int p = pq[idx][0], q = pq[idx][1];
            if (std::abs(a[p][q]) < 1e-300) continue;
            double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            double t = (theta >= 0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta*theta + 1.0));
            double c = 1.0 / std::sqrt(t*t + 1.0);
            double s = t * c;
            for (int k = 0; k < 3; k++) {
                double akp = a[k][p], akq = a[k][q];
                a[k][p] = c*akp - s*akq;
                a[k][q] = s*akp + c*akq;
            }
            for (int k = 0; k < 3; k++) {
                double apk = a[p][k], aqk = a[q][k];
                a[p][k] = c*apk - s*aqk;
                a[q][k] = s*apk + c*aqk;
            }
            for (int k = 0; k < 3; k++) {
                double vkp = v[k][p], vkq = v[k][q];
                v[k][p] = c*vkp - s*vkq;
                v[k][q] = s*vkp + c*vkq;
            }
        }
    }
    eigvals[0] = a[0][0]; eigvals[1] = a[1][1]; eigvals[2] = a[2][2];
    for (int k = 0; k < 3; k++) eigvecs[k] = V3{v[0][k], v[1][k], v[2][k]};
}

// Recognized-surface descriptor.
struct RecogSurface {
    enum Kind { NONE, PLANE, SPHERE, CYLINDER, CONE, TORUS } kind = NONE;
    V3 p1{};   // plane: origin; sphere: center; cylinder: axis_pt; cone: apex; torus: center
    V3 p2{};   // plane: normal; cylinder: axis_dir; cone: axis; torus: axis
    double r = 0.0;   // sphere: radius; cylinder: radius; cone: half_angle; torus: major radius R
    double r2 = 0.0;  // torus: minor radius r
};

static bool fit_cylinder(const NurbsSurface& surface, double tol, V3& axis_pt, V3& axis_dir, double& radius) {
    auto [u0, u1] = surface.domain(0);
    auto [v0, v1] = surface.domain(1);
    std::vector<V3> pts;
    std::vector<V3> nrm;
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            double uu = u0 + (u1-u0)*i/4.0;
            double vv = v0 + (v1-v0)*j/4.0;
            Point p = surface.point_at(uu, vv);
            pts.push_back(V3{p[0], p[1], p[2]});
            Vector n = surface.normal_at(uu, vv);
            nrm.push_back(V3{n[0], n[1], n[2]});
        }
    }
    double M[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    for (auto& n : nrm)
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) M[r][c] += n[r]*n[c];
    double evals[3]; V3 evecs[3];
    jacobi_eig3(M, evals, evecs);
    int kmin = 0;
    for (int k = 1; k < 3; k++) if (evals[k] < evals[kmin]) kmin = k;
    V3 w = evecs[kmin];
    double wl = std::sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
    if (wl < 1e-12) return false;
    w = V3{w[0]/wl, w[1]/wl, w[2]/wl};
    auto [ea, eb] = ortho_basis(w);
    V3 p0 = pts[0];
    std::vector<std::vector<double>> ata(3, std::vector<double>(3, 0.0));
    std::vector<double> atb(3, 0.0);
    std::vector<std::pair<double,double>> proj;
    for (auto& p : pts) {
        V3 dp{p[0]-p0[0], p[1]-p0[1], p[2]-p0[2]};
        double x = dp[0]*ea[0] + dp[1]*ea[1] + dp[2]*ea[2];
        double y = dp[0]*eb[0] + dp[1]*eb[1] + dp[2]*eb[2];
        proj.push_back({x, y});
        double row[3] = {x, y, 1.0};
        double rhs = -(x*x + y*y);
        for (int r = 0; r < 3; r++) {
            atb[r] += row[r]*rhs;
            for (int c = 0; c < 3; c++) ata[r][c] += row[r]*row[c];
        }
    }
    std::vector<double> sol;
    if (!solve_gauss(ata, atb, 3, sol)) return false;
    double ccx = -sol[0]/2.0, ccy = -sol[1]/2.0;
    double r2 = ccx*ccx + ccy*ccy - sol[2];
    if (r2 <= 1e-18) return false;
    double r = std::sqrt(r2);
    for (auto& pr : proj)
        if (std::abs(std::sqrt((pr.first-ccx)*(pr.first-ccx) + (pr.second-ccy)*(pr.second-ccy)) - r) > tol)
            return false;
    axis_pt = V3{p0[0] + ccx*ea[0] + ccy*eb[0],
                 p0[1] + ccx*ea[1] + ccy*eb[1],
                 p0[2] + ccx*ea[2] + ccy*eb[2]};
    axis_dir = w;
    radius = r;
    return true;
}

static bool fit_cone(const NurbsSurface& surface, double tol, V3& apex, V3& axis, double& half_angle) {
    auto [u0, u1] = surface.domain(0);
    auto [v0, v1] = surface.domain(1);
    std::vector<V3> pts;
    std::vector<std::pair<V3, V3>> nrm;  // (unit normal, point)
    int nu_s = 8;
    for (int i = 0; i < nu_s; i++) {
        double uu = u0 + (u1-u0)*i/(double)nu_s;
        for (int j = 0; j < 5; j++) {
            double vv = v0 + (v1-v0)*j/4.0;
            Point p = surface.point_at(uu, vv);
            pts.push_back(V3{p[0], p[1], p[2]});
            Vector n = surface.normal_at(uu, vv);
            double nl = std::sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
            if (nl < 1e-12) continue;
            Point pp = surface.point_at(uu, vv);
            nrm.push_back({V3{n[0]/nl, n[1]/nl, n[2]/nl}, V3{pp[0], pp[1], pp[2]}});
        }
    }
    if ((int)nrm.size() < 4) return false;
    std::vector<std::vector<double>> ata(3, std::vector<double>(3, 0.0));
    std::vector<double> atb(3, 0.0);
    for (auto& np : nrm) {
        const V3& n = np.first;
        const V3& p = np.second;
        double npd = n[0]*p[0] + n[1]*p[1] + n[2]*p[2];
        for (int r = 0; r < 3; r++) {
            atb[r] += n[r]*npd;
            for (int c = 0; c < 3; c++) ata[r][c] += n[r]*n[c];
        }
    }
    std::vector<double> V;
    if (!solve_gauss(ata, atb, 3, V)) return false;
    std::vector<V3> gs;
    for (auto& p : pts) {
        V3 d{p[0]-V[0], p[1]-V[1], p[2]-V[2]};
        double dl = std::sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
        if (dl < tol) continue;
        gs.push_back(V3{d[0]/dl, d[1]/dl, d[2]/dl});
    }
    if ((int)gs.size() < 3) return false;
    double G[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    for (auto& g : gs)
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) G[r][c] += g[r]*g[c];
    double gevals[3]; V3 gevecs[3];
    jacobi_eig3(G, gevals, gevecs);
    int kmax = 0;
    for (int k = 1; k < 3; k++) if (gevals[k] > gevals[kmax]) kmax = k;
    V3 w = gevecs[kmax];
    V3 sx{0, 0, 0};
    for (auto& g : gs) { sx[0] += g[0]; sx[1] += g[1]; sx[2] += g[2]; }
    if (w[0]*sx[0] + w[1]*sx[1] + w[2]*sx[2] < 0.0) w = V3{-w[0], -w[1], -w[2]};
    double wl = std::sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
    if (wl < 1e-12) return false;
    w = V3{w[0]/wl, w[1]/wl, w[2]/wl};
    double sumang = 0.0;
    for (auto& g : gs)
        sumang += std::acos(std::max(-1.0, std::min(1.0, g[0]*w[0]+g[1]*w[1]+g[2]*w[2])));
    double alpha = sumang / gs.size();
    if (alpha < 1e-4 || alpha > Tolerance::PI/2 - 1e-4) return false;
    double ca = std::cos(alpha);
    for (auto& p : pts) {
        V3 d{p[0]-V[0], p[1]-V[1], p[2]-V[2]};
        double axd = d[0]*w[0] + d[1]*w[1] + d[2]*w[2];
        double perp = std::sqrt(std::max(0.0, (d[0]*d[0]+d[1]*d[1]+d[2]*d[2]) - axd*axd));
        if (std::abs(perp - axd*std::tan(alpha)) * ca > tol) return false;
    }
    apex = V3{V[0], V[1], V[2]};
    axis = w;
    half_angle = alpha;
    return true;
}

static bool fit_sphere(const NurbsSurface& surface, double tol, double& cx, double& cy, double& cz, double& radius) {
    auto [u0, u1] = surface.domain(0);
    auto [v0, v1] = surface.domain(1);
    std::vector<V3> pts;
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            double uu = u0 + (u1-u0)*i/4.0;
            double vv = v0 + (v1-v0)*j/4.0;
            Point p = surface.point_at(uu, vv);
            pts.push_back(V3{p[0], p[1], p[2]});
        }
    }
    std::vector<std::vector<double>> ata(4, std::vector<double>(4, 0.0));
    std::vector<double> atb(4, 0.0);
    for (auto& p : pts) {
        double row[4] = {p[0], p[1], p[2], 1.0};
        double rhs = -(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
        for (int r = 0; r < 4; r++) {
            atb[r] += row[r]*rhs;
            for (int c = 0; c < 4; c++) ata[r][c] += row[r]*row[c];
        }
    }
    std::vector<double> sol;
    if (!solve_gauss(ata, atb, 4, sol)) return false;
    double ccx = -sol[0]/2.0, ccy = -sol[1]/2.0, ccz = -sol[2]/2.0;
    double r2 = ccx*ccx + ccy*ccy + ccz*ccz - sol[3];
    if (r2 <= 0.0) return false;
    double r = std::sqrt(r2);
    for (auto& p : pts) {
        double d = std::sqrt((p[0]-ccx)*(p[0]-ccx) + (p[1]-ccy)*(p[1]-ccy) + (p[2]-ccz)*(p[2]-ccz));
        if (std::abs(d - r) > tol) return false;
    }
    cx = ccx; cy = ccy; cz = ccz; radius = r;
    return true;
}

// Recognize a torus. Axis = smallest-variance direction of the surface points;
// then fit a 2D circle (rho, axial) of the tube cross-section. Returns
// (center, axis, major_radius R, minor_radius r) or false.
static bool fit_torus(const NurbsSurface& surface, double tol, V3& center, V3& axis, double& R_out, double& r_out) {
    auto [u0, u1] = surface.domain(0);
    auto [v0, v1] = surface.domain(1);
    std::vector<V3> pts;
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            Point p = surface.point_at(u0 + (u1-u0)*i/8.0, v0 + (v1-v0)*j/8.0);
            pts.push_back(V3{p[0], p[1], p[2]});
        }
    }
    int n = (int)pts.size();
    V3 cen{0, 0, 0};
    for (auto& p : pts) { cen[0] += p[0]; cen[1] += p[1]; cen[2] += p[2]; }
    cen[0] /= n; cen[1] /= n; cen[2] /= n;
    double M[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    for (auto& p : pts) {
        V3 d{p[0]-cen[0], p[1]-cen[1], p[2]-cen[2]};
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) M[r][c] += d[r]*d[c];
    }
    double evals[3]; V3 evecs[3];
    jacobi_eig3(M, evals, evecs);
    int kmin = 0;
    for (int k = 1; k < 3; k++) if (evals[k] < evals[kmin]) kmin = k;
    V3 w = evecs[kmin];
    double wl = std::sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
    if (wl < 1e-12) return false;
    w = V3{w[0]/wl, w[1]/wl, w[2]/wl};
    std::vector<std::vector<double>> ata(3, std::vector<double>(3, 0.0));
    std::vector<double> atb(3, 0.0);
    std::vector<std::pair<double,double>> rhoa;
    for (auto& p : pts) {
        V3 d{p[0]-cen[0], p[1]-cen[1], p[2]-cen[2]};
        double a = d[0]*w[0] + d[1]*w[1] + d[2]*w[2];
        V3 perp{d[0]-a*w[0], d[1]-a*w[1], d[2]-a*w[2]};
        double rho = std::sqrt(perp[0]*perp[0] + perp[1]*perp[1] + perp[2]*perp[2]);
        rhoa.push_back({rho, a});
        double row[3] = {rho, a, 1.0};
        double rhs = -(rho*rho + a*a);
        for (int r = 0; r < 3; r++) {
            atb[r] += row[r]*rhs;
            for (int c = 0; c < 3; c++) ata[r][c] += row[r]*row[c];
        }
    }
    std::vector<double> sol;
    if (!solve_gauss(ata, atb, 3, sol)) return false;
    double R = -sol[0]/2.0;
    double a0 = -sol[1]/2.0;
    double r2 = R*R + a0*a0 - sol[2];
    if (r2 <= 1e-18 || R <= 0.0) return false;
    double r = std::sqrt(r2);
    if (R <= r * 0.5) return false;  // not a clear ring torus
    for (auto& ra : rhoa)
        if (std::abs(std::sqrt((ra.first-R)*(ra.first-R) + (ra.second-a0)*(ra.second-a0)) - r) > tol)
            return false;
    center = V3{cen[0]+a0*w[0], cen[1]+a0*w[1], cen[2]+a0*w[2]};
    axis = w;
    R_out = R;
    r_out = r;
    return true;
}

static RecogSurface recognize_surface(const NurbsSurface& surface, double tol) {
    RecogSurface rs;
    if (surface.is_planar(nullptr, tol)) {
        auto [u0, u1] = surface.domain(0);
        auto [v0, v1] = surface.domain(1);
        Point o = surface.point_at((u0+u1)*0.5, (v0+v1)*0.5);
        Vector n = surface.normal_at((u0+u1)*0.5, (v0+v1)*0.5);
        rs.kind = RecogSurface::PLANE;
        rs.p1 = V3{o[0], o[1], o[2]};
        rs.p2 = V3{n[0], n[1], n[2]};
        return rs;
    }
    double cx, cy, cz, r;
    if (fit_sphere(surface, tol, cx, cy, cz, r)) {
        rs.kind = RecogSurface::SPHERE;
        rs.p1 = V3{cx, cy, cz};
        rs.r = r;
        return rs;
    }
    V3 axis_pt, axis_dir; double rad;
    if (fit_cylinder(surface, tol, axis_pt, axis_dir, rad)) {
        rs.kind = RecogSurface::CYLINDER;
        rs.p1 = axis_pt;
        rs.p2 = axis_dir;
        rs.r = rad;
        return rs;
    }
    V3 apex, axis; double half_angle;
    if (fit_cone(surface, tol, apex, axis, half_angle)) {
        rs.kind = RecogSurface::CONE;
        rs.p1 = apex;
        rs.p2 = axis;
        rs.r = half_angle;
        return rs;
    }
    V3 tcenter, taxis; double tR, tr;
    if (fit_torus(surface, tol, tcenter, taxis, tR, tr)) {
        rs.kind = RecogSurface::TORUS;
        rs.p1 = tcenter;
        rs.p2 = taxis;
        rs.r = tR;
        rs.r2 = tr;
        return rs;
    }
    return rs;  // NONE
}

// Solve ((X-V).w)^2 - cos^2a |X-V|^2 = 0 along X = x0 + t d. Returns roots.
static std::vector<double> line_cone(const V3& x0, const V3& d, const V3& V, const V3& w, double alpha) {
    double ca2 = std::cos(alpha) * std::cos(alpha);
    V3 e{x0[0]-V[0], x0[1]-V[1], x0[2]-V[2]};
    double A = e[0]*w[0]+e[1]*w[1]+e[2]*w[2];
    double B = d[0]*w[0]+d[1]*w[1]+d[2]*w[2];
    double C = e[0]*e[0]+e[1]*e[1]+e[2]*e[2];
    double D = e[0]*d[0]+e[1]*d[1]+e[2]*d[2];
    double E = d[0]*d[0]+d[1]*d[1]+d[2]*d[2];
    double qa = B*B - ca2*E;
    double qb = 2.0*A*B - 2.0*ca2*D;
    double qc = A*A - ca2*C;
    if (std::abs(qa) < 1e-14)
        return std::abs(qb) < 1e-300 ? std::vector<double>{} : std::vector<double>{-qc/qb};
    double disc = qb*qb - 4.0*qa*qc;
    if (disc < 0.0) return {};
    double sq = std::sqrt(disc);
    return {(-qb - sq)/(2.0*qa), (-qb + sq)/(2.0*qa)};
}

// plane_sphere: returns true with c3 set, or false (no intersection).
static bool ssi_plane_sphere(const RecogSurface& plane, const RecogSurface& sph, NurbsCurve& c3) {
    V3 o = plane.p1, nu = ssi_unit(plane.p2);
    V3 c = sph.p1; double r = sph.r;
    double d = (c[0]-o[0])*nu[0] + (c[1]-o[1])*nu[1] + (c[2]-o[2])*nu[2];
    if (std::abs(d) >= r) return false;
    V3 cc{c[0]-d*nu[0], c[1]-d*nu[1], c[2]-d*nu[2]};
    double rr = std::sqrt(r*r - d*d);
    auto [xa, ya] = ortho_basis(nu);
    c3 = exact_circle(cc[0], cc[1], cc[2], xa, ya, rr);
    return true;
}

static bool ssi_plane_cylinder(const RecogSurface& plane, const RecogSurface& cyl, NurbsCurve& c3) {
    V3 o = plane.p1, nu = ssi_unit(plane.p2);
    V3 P = cyl.p1, w = ssi_unit(cyl.p2); double r = cyl.r;
    double wn = w[0]*nu[0] + w[1]*nu[1] + w[2]*nu[2];
    if (std::abs(wn) < 1e-7) return false;  // plane parallel to axis -> marcher
    double t = ((o[0]-P[0])*nu[0] + (o[1]-P[1])*nu[1] + (o[2]-P[2])*nu[2]) / wn;
    V3 cc{P[0]+t*w[0], P[1]+t*w[1], P[2]+t*w[2]};
    V3 mraw = ssi_cross(w, nu);
    if (std::sqrt(mraw[0]*mraw[0] + mraw[1]*mraw[1] + mraw[2]*mraw[2]) < 1e-9) {
        auto [xa, ya] = ortho_basis(nu);
        c3 = exact_circle(cc[0], cc[1], cc[2], xa, ya, r);
        return true;
    }
    V3 minor = ssi_unit(mraw);
    V3 major = ssi_unit(V3{w[0]-wn*nu[0], w[1]-wn*nu[1], w[2]-wn*nu[2]});
    c3 = exact_ellipse(cc[0], cc[1], cc[2], major, minor, r/std::abs(wn), r);
    return true;
}

static double cone_axial_extent(const NurbsSurface& srf, const V3& apex, const V3& axis) {
    V3 w = ssi_unit(axis);
    auto [u0, u1] = srf.domain(0);
    auto [v0, v1] = srf.domain(1);
    double um = 0.5 * (u0 + u1);
    double H = 0.0;
    for (double vv : {v0, v1}) {
        Point p = srf.point_at(um, vv);
        double s = (p[0]-apex[0])*w[0] + (p[1]-apex[1])*w[1] + (p[2]-apex[2])*w[2];
        H = std::max(H, s);
    }
    return H;
}
static bool conic_within_cone(const NurbsCurve& c, const V3& apex, const V3& w, double H) {
    auto [t0, t1] = c.domain();
    double pad = 1e-7 * std::max(1.0, H);
    for (int i = 0; i <= 64; ++i) {
        Point p = c.point_at(t0 + (t1-t0)*i/64);
        double s = (p[0]-apex[0])*w[0] + (p[1]-apex[1])*w[1] + (p[2]-apex[2])*w[2];
        if (s < -pad || s > H + pad) return false;
    }
    return true;
}
static NurbsCurve fit_conic_arc(const std::vector<Point>& pts) {
    int m = (int)pts.size();
    if (m < 2) return NurbsCurve();
    if (m == 2) return NurbsCurve::create(false, 1, pts);
    if (m <= 4) return NurbsCurve::create_interpolated(pts, CurveNurbsKnotStyle::Chord);
    int num_cvs = std::min(std::max(m / 6, 8), 64);
    if (num_cvs >= m) num_cvs = m - 1;
    NurbsCurve c = NurbsCurve::create_fitted(pts, num_cvs, 3);
    if (!c.is_valid()) c = NurbsCurve::create_interpolated(pts, CurveNurbsKnotStyle::Chord);
    return c;
}
static bool build_exact_plane_cone_ellipse(const V3& o, const V3& nu, const V3& V, const V3& w,
                                           double alpha, NurbsCurve& c3) {
    double wn = w[0]*nu[0] + w[1]*nu[1] + w[2]*nu[2];
    V3 m = ssi_cross(w, nu);
    double ml = std::sqrt(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
    if (ml < 1e-12) return false;
    m = V3{m[0]/ml, m[1]/ml, m[2]/ml};
    V3 major = ssi_unit(V3{w[0]-wn*nu[0], w[1]-wn*nu[1], w[2]-wn*nu[2]});
    double dV = (V[0]-o[0])*nu[0] + (V[1]-o[1])*nu[1] + (V[2]-o[2])*nu[2];
    V3 Vp{V[0]-dV*nu[0], V[1]-dV*nu[1], V[2]-dV*nu[2]};
    std::vector<double> ts = line_cone(Vp, major, V, w, alpha);
    if (ts.size() != 2) return false;
    V3 A{Vp[0]+ts[0]*major[0], Vp[1]+ts[0]*major[1], Vp[2]+ts[0]*major[2]};
    V3 Bp{Vp[0]+ts[1]*major[0], Vp[1]+ts[1]*major[1], Vp[2]+ts[1]*major[2]};
    V3 cc{(A[0]+Bp[0])*0.5, (A[1]+Bp[1])*0.5, (A[2]+Bp[2])*0.5};
    double semi_major = 0.5*std::sqrt((Bp[0]-A[0])*(Bp[0]-A[0]) + (Bp[1]-A[1])*(Bp[1]-A[1]) + (Bp[2]-A[2])*(Bp[2]-A[2]));
    major = ssi_unit(V3{Bp[0]-A[0], Bp[1]-A[1], Bp[2]-A[2]});
    std::vector<double> tm = line_cone(cc, m, V, w, alpha);
    if (tm.size() != 2) return false;
    double semi_minor = 0.5*std::abs(tm[1]-tm[0]);
    if (semi_major < 1e-12 || semi_minor < 1e-12) return false;
    c3 = exact_ellipse(cc[0], cc[1], cc[2], major, m, semi_major, semi_minor);
    return true;
}
static void sample_plane_cone_arcs(const V3& apex, const V3& w, const V3& e1, const V3& e2,
                                   double na, double pP, double qP, double D0, double ta, double H,
                                   std::vector<std::vector<Point>>& runs, bool& closed) {
    runs.clear(); closed = false;
    const int N = 720;
    const double TWO_PI = 2.0 * 3.14159265358979323846;
    auto denom = [&](double phi) { return na + ta*(pP*std::cos(phi) + qP*std::sin(phi)); };
    auto s_of  = [&](double phi) { double d = denom(phi); return (std::abs(d) < 1e-300) ? 1e308 : -D0/d; };
    auto pt = [&](double phi) {
        double s = s_of(phi), rr = s*ta, c = std::cos(phi), sn = std::sin(phi);
        return Point(apex[0] + s*w[0] + rr*(c*e1[0] + sn*e2[0]),
                     apex[1] + s*w[1] + rr*(c*e1[1] + sn*e2[1]),
                     apex[2] + s*w[2] + rr*(c*e1[2] + sn*e2[2]));
    };
    const double eps = 1e-9 * std::max(1.0, H);
    std::vector<char> ok(N);
    int cnt = 0;
    for (int k = 0; k < N; ++k) {
        double s = s_of(TWO_PI*k/N);
        ok[k] = (s > eps && s < H + eps) ? 1 : 0;
        cnt += ok[k];
    }
    if (cnt == 0) return;
    if (cnt == N) {
        std::vector<Point> loop;
        for (int k = 0; k <= N; ++k) loop.push_back(pt(TWO_PI*(k % N)/N));
        runs.push_back(loop); closed = true; return;
    }
    int start = 0; while (start < N && ok[start]) ++start;
    double dtarget = (H > 1e-300) ? (-D0 / H) : 0.0;
    auto refine_base = [&](double pa, double pb) -> double {
        double fa = denom(pa) - dtarget;
        for (int it = 0; it < 60; ++it) {
            double pm = 0.5*(pa+pb), fm = denom(pm) - dtarget;
            if ((fm < 0) == (fa < 0)) { pa = pm; fa = fm; } else pb = pm;
        }
        return 0.5*(pa+pb);
    };
    std::vector<Point> cur; bool in = false;
    for (int i = 0; i <= N; ++i) {
        int k = (start + i) % N;
        double uphi = TWO_PI*start/N + TWO_PI*i/N;
        bool v = ok[k] != 0;
        if (v && !in) {
            if (i > 0) cur.push_back(pt(refine_base(uphi - TWO_PI/N, uphi)));
            cur.push_back(pt(uphi)); in = true;
        } else if (v && in) {
            cur.push_back(pt(uphi));
        } else if (!v && in) {
            cur.push_back(pt(refine_base(uphi - TWO_PI/N, uphi)));
            if (cur.size() >= 2) runs.push_back(cur);
            cur.clear(); in = false;
        }
    }
    if (in && cur.size() >= 2) runs.push_back(cur);
}
static bool ssi_plane_cone(const RecogSurface& plane, const RecogSurface& cone,
                           const NurbsSurface& cone_srf, std::vector<NurbsCurve>& out) {
    V3 o = plane.p1, nu = ssi_unit(plane.p2);
    V3 V = cone.p1, w = ssi_unit(cone.p2);
    double alpha = cone.r;
    if (alpha < 1e-7 || alpha > Tolerance::PI/2 - 1e-7) return false;
    double ta = std::tan(alpha), cosa = std::cos(alpha), sina = std::sin(alpha);
    double H = cone_axial_extent(cone_srf, V, w);
    if (H < 1e-12) return false;
    auto [e1, e2] = ortho_basis(w);
    double na = nu[0]*w[0]  + nu[1]*w[1]  + nu[2]*w[2];
    double pP = nu[0]*e1[0] + nu[1]*e1[1] + nu[2]*e1[2];
    double qP = nu[0]*e2[0] + nu[1]*e2[1] + nu[2]*e2[2];
    double cost  = std::abs(na);
    double sint  = std::sqrt(std::max(0.0, pP*pP + qP*qP));
    double costa = cost*cosa - sint*sina;
    double D0    = (V[0]-o[0])*nu[0] + (V[1]-o[1])*nu[1] + (V[2]-o[2])*nu[2];
    const double ang     = 1e-6;
    const double distTol = 1e-6 * std::max(1.0, H);
    if (std::abs(D0) < distTol) {
        if (std::abs(costa) < ang) {
            V3 g = ssi_unit(V3{w[0]-na*nu[0], w[1]-na*nu[1], w[2]-na*nu[2]});
            double gw = g[0]*w[0] + g[1]*w[1] + g[2]*w[2];
            if (gw > 1e-9) { double L = H / gw;
                out.push_back(NurbsCurve::create(false, 1,
                    {Point(V[0],V[1],V[2]), Point(V[0]+L*g[0], V[1]+L*g[1], V[2]+L*g[2])})); }
            return true;
        }
        if (cost < sina) {
            V3 axey = ssi_cross(nu, w);
            V3 axex = ssi_cross(axey, nu);
            double dh = std::sqrt(std::max(0.0, sina*sina - cost*cost)) / cosa;
            for (int sgn : {+1, -1}) {
                V3 d{axex[0]+sgn*dh*axey[0], axex[1]+sgn*dh*axey[1], axex[2]+sgn*dh*axey[2]};
                double dw = d[0]*w[0] + d[1]*w[1] + d[2]*w[2];
                if (dw < 1e-12) continue;
                double L = H / dw;
                out.push_back(NurbsCurve::create(false, 1,
                    {Point(V[0],V[1],V[2]), Point(V[0]+L*d[0], V[1]+L*d[1], V[2]+L*d[2])}));
            }
            return true;
        }
        return true;
    }
    bool is_circle=false, is_parabola=false, is_hyperbola=false, is_ellipse=false;
    if      (cost < ang)            is_hyperbola = true;
    else if (std::abs(costa) < ang) is_parabola  = true;
    else if (sint < ang)            is_circle     = true;
    else if (cost < sina)           is_hyperbola  = true;
    else                            is_ellipse    = true;
    if (is_circle) {
        double dax = (o[0]-V[0])*w[0] + (o[1]-V[1])*w[1] + (o[2]-V[2])*w[2];
        double rr = std::abs(dax) * ta;
        if (rr > 1e-12) {
            V3 cc{V[0]+dax*w[0], V[1]+dax*w[1], V[2]+dax*w[2]};
            NurbsCurve circ = exact_circle(cc[0], cc[1], cc[2], e1, e2, rr);
            if (conic_within_cone(circ, V, w, H)) { out.push_back(circ); return true; }
        }
        return true;
    }
    if (is_ellipse) {
        NurbsCurve ell;
        if (build_exact_plane_cone_ellipse(o, nu, V, w, alpha, ell) &&
            conic_within_cone(ell, V, w, H)) { out.push_back(ell); return true; }
    }
    std::vector<std::vector<Point>> runs; bool closed = false;
    sample_plane_cone_arcs(V, w, e1, e2, na, pP, qP, D0, ta, H, runs, closed);
    for (auto& r : runs) { NurbsCurve c = fit_conic_arc(r); if (c.is_valid()) out.push_back(c); }
    (void)is_parabola; (void)is_hyperbola;
    return true;
}

// plane_torus: only the plane-perpendicular-to-axis case is conic (two
// concentric circles). Returns true if the pair was handled analytically
// (out may then contain 0, 1, or 2 circles); false means non-perpendicular
// -> quartic, caller marches.
static bool ssi_plane_torus(const RecogSurface& plane, const RecogSurface& tor, std::vector<NurbsCurve>& out) {
    V3 o = plane.p1, nu = ssi_unit(plane.p2);
    V3 C = tor.p1, w = ssi_unit(tor.p2); double R = tor.r, r = tor.r2;
    double wn = w[0]*nu[0] + w[1]*nu[1] + w[2]*nu[2];
    if (std::abs(std::abs(wn) - 1.0) > 1e-7) return false;  // non-perpendicular -> marcher
    double d = (o[0]-C[0])*w[0] + (o[1]-C[1])*w[1] + (o[2]-C[2])*w[2];
    if (std::abs(d) > r) return true;  // plane misses the tube (empty)
    double h = std::sqrt(std::max(0.0, r*r - d*d));
    V3 cc{C[0]+d*w[0], C[1]+d*w[1], C[2]+d*w[2]};
    auto [xa, ya] = ortho_basis(w);
    for (double rr : {R + h, R - h})
        if (rr > 1e-12)
            out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rr));
    return true;
}

// Exact intersection of two finite planar faces: the infinite plane/plane line clipped to
// BOTH surfaces' UV domains (so the segment lies inside both faces). Returns an exact 2-CV
// degree-1 line. `empty` is set when the planes are recognized but their finite extents do
// not overlap (recognized -> no intersection). Returns false (without empty) only for the
// parallel/degenerate case, which the caller leaves to the marcher.
static bool ssi_plane_plane(const NurbsSurface& sa, const RecogSurface& pa,
                            const NurbsSurface& sb, const RecogSurface& pb,
                            NurbsCurve& c3, bool& empty) {
    empty = false;
    V3 na = ssi_unit(pa.p2), nb = ssi_unit(pb.p2);
    V3 v = ssi_cross(na, nb);
    double vl = std::sqrt(ssi_dot(v, v));
    if (vl < 1e-9) return false;  // parallel/coincident -> marcher
    // Anchor: the point of the intersection line closest to the origin (two-plane closed form).
    double dA = ssi_dot(na, pa.p1), dB = ssi_dot(nb, pb.p1);
    V3 nb_x_v = ssi_cross(nb, v), v_x_na = ssi_cross(v, na);
    double inv = 1.0 / (vl * vl);
    V3 anchor{(dA*nb_x_v[0] + dB*v_x_na[0]) * inv,
              (dA*nb_x_v[1] + dB*v_x_na[1]) * inv,
              (dA*nb_x_v[2] + dB*v_x_na[2]) * inv};
    V3 dir{v[0]/vl, v[1]/vl, v[2]/vl};

    // Clip the parametric line anchor + t*dir to each face's UV rectangle (in [0,1]^2 frame).
    double tmin = -1e300, tmax = 1e300;
    for (const NurbsSurface* s : {&sa, &sb}) {
        auto [u0, u1] = s->domain(0);
        auto [v0, v1] = s->domain(1);
        Point O = s->point_at(u0, v0), Pu = s->point_at(u1, v0), Pv = s->point_at(u0, v1);
        V3 o{O[0],O[1],O[2]};
        V3 eu{Pu[0]-O[0], Pu[1]-O[1], Pu[2]-O[2]};
        V3 ev{Pv[0]-O[0], Pv[1]-O[1], Pv[2]-O[2]};
        double exx = ssi_dot(eu,eu), eyy = ssi_dot(ev,ev), exy = ssi_dot(eu,ev);
        double det = exx*eyy - exy*exy;
        if (std::abs(det) < 1e-18) return false;
        auto frac = [&](const V3& r, double& al, double& be) {
            double rx = ssi_dot(r, eu), ry = ssi_dot(r, ev);
            al = (eyy*rx - exy*ry) / det;
            be = (exx*ry - exy*rx) / det;
        };
        double a0, b0, da, db;
        frac(V3{anchor[0]-o[0], anchor[1]-o[1], anchor[2]-o[2]}, a0, b0);
        frac(dir, da, db);
        double t0 = -1e300, t1 = 1e300;
        auto axis_clip = [&](double c, double d) -> bool {
            if (std::abs(d) < 1e-15) return (c >= -1e-9 && c <= 1.0 + 1e-9);
            double ta = (0.0 - c) / d, tb = (1.0 - c) / d;
            if (ta > tb) std::swap(ta, tb);
            t0 = std::max(t0, ta); t1 = std::min(t1, tb);
            return true;
        };
        if (!axis_clip(a0, da) || !axis_clip(b0, db) || t0 > t1) { empty = true; return false; }
        tmin = std::max(tmin, t0); tmax = std::min(tmax, t1);
    }
    if (tmax - tmin <= 1e-9) { empty = true; return false; }
    Point A(anchor[0]+tmin*dir[0], anchor[1]+tmin*dir[1], anchor[2]+tmin*dir[2]);
    Point B(anchor[0]+tmax*dir[0], anchor[1]+tmax*dir[1], anchor[2]+tmax*dir[2]);
    c3 = NurbsCurve::create(false, 1, {A, B});
    c3.set_domain(0.0, 1.0);
    return true;
}

// Result wraps Python's None/[]/[triples...] tri-state:
//   status == NOT_ANALYTIC -> Python None (caller marches)
//   status == NO_HIT       -> Python [] (recognized, no intersection)
//   status == HIT          -> Python [(c3,pa,pb), ...] (one or more triples)
struct AnalyticResult {
    enum { NOT_ANALYTIC, NO_HIT, HIT } status = NOT_ANALYTIC;
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> triples;
};


// Analytic pcurve of an exact 3D intersection conic onto a recognized quadric surface,
// reproducing OCCT's ProjLib closed-form projection (no sampling/fitting):
//  - PLANE: the surface (u,v)->3D map is affine (bilinear over a rectangle), so invert it
//    linearly and remap the conic's control points to UV -- preserves the exact rational
//    circle/ellipse.
//  - CYLINDER, circle in a plane perpendicular to the axis: the pcurve is a horizontal line
//    v = const in (u=theta, v=height) space, spanning the full u range.
// Returns an invalid curve when not analytically handled (caller falls back to projection).
static NurbsCurve analytic_pcurve(const NurbsSurface& srf, const RecogSurface& recog, const NurbsCurve& c3d) {
    auto [u0, u1] = srf.domain(0);
    auto [v0, v1] = srf.domain(1);
    auto dot = [](const double a[3], const double b[3]) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; };

    if (recog.kind == RecogSurface::PLANE) {
        Point o = srf.point_at(u0, v0);
        Point pu = srf.point_at(u1, v0);
        Point pv = srf.point_at(u0, v1);
        double ex[3] = {pu[0]-o[0], pu[1]-o[1], pu[2]-o[2]};
        double ey[3] = {pv[0]-o[0], pv[1]-o[1], pv[2]-o[2]};
        double exx = dot(ex,ex), eyy = dot(ey,ey), exy = dot(ex,ey);
        double det = exx*eyy - exy*exy;
        if (std::abs(det) < 1e-18) return NurbsCurve();
        NurbsCurve pc = c3d;  // same degree / knots / weights
        int nc = c3d.cv_count();
        for (int i = 0; i < nc; ++i) {
            Point P = c3d.get_cv(i);
            double r[3] = {P[0]-o[0], P[1]-o[1], P[2]-o[2]};
            double rx = dot(r,ex), ry = dot(r,ey);
            double a = (eyy*rx - exy*ry) / det;
            double b = (exx*ry - exy*rx) / det;
            double u = u0 + a*(u1-u0), v = v0 + b*(v1-v0);
            if (c3d.is_rational()) {
                double w = c3d.weight(i);
                pc.set_cv_4d(i, u*w, v*w, 0.0, w);
            } else {
                pc.set_cv(i, Point(u, v, 0.0));
            }
        }
        return pc;
    }

    // CYLINDER analytic pcurve (circle -> v=const line) is geometrically correct but does not
    // yet satisfy split_by_uv_curves' seam/boundary expectations for the cylinder face; falls
    // back to projection until the seam-aware endpoints are handled. (next B4 iteration)
    if (recog.kind == RecogSurface::CYLINDER) {
        double ap[3] = {recog.p1[0], recog.p1[1], recog.p1[2]};
        double ax[3] = {recog.p2[0], recog.p2[1], recog.p2[2]};
        double an = std::sqrt(dot(ax,ax));
        if (an < 1e-12) return NurbsCurve();
        ax[0]/=an; ax[1]/=an; ax[2]/=an;
        auto height = [&](const Point& p) { double r[3]={p[0]-ap[0],p[1]-ap[1],p[2]-ap[2]}; return dot(r,ax); };
        double um = 0.5*(u0+u1);
        double h0 = height(srf.point_at(um, v0)), h1 = height(srf.point_at(um, v1));
        if (std::abs(h1 - h0) < 1e-12) return NurbsCurve();
        // Conic must lie in a plane perpendicular to the axis (constant height) AND span the
        // full angular range of the cylinder face (otherwise it's a partial/clipped arc that
        // the generic projector handles). Both conditions fall back to projection if unmet.
        double hmin = 1e300, hmax = -1e300, hsum = 0; int ns = 0;
        auto [t0,t1] = c3d.domain();
        for (int i = 0; i <= 32; ++i) {
            double h = height(c3d.point_at(t0 + (t1-t0)*i/32));
            hmin = std::min(hmin, h); hmax = std::max(hmax, h); hsum += h; ns++;
        }
        if (hmax - hmin > 1e-5 * std::abs(h1 - h0)) return NurbsCurve();  // oblique -> fallback
        // Confirm the conic is a closed loop wrapping the full cylinder (start ~ end in 3D).
        if (c3d.point_at(t0).distance(c3d.point_at(t1)) > 1e-6 * (std::abs(h1 - h0) + 1.0))
            return NurbsCurve();
        double hc = hsum / ns;
        double vc = v0 + (hc - h0) / (h1 - h0) * (v1 - v0);
        if (vc < std::min(v0,v1) - 1e-9 || vc > std::max(v0,v1) + 1e-9) return NurbsCurve();
        return NurbsCurve::create(false, 1, {Point(u0, vc, 0.0), Point(u1, vc, 0.0)});
    }

    // SPHERE, circle in a plane perpendicular to the polar axis -> a parallel: an exact v=const
    // line spanning the full u range. (Oblique planes give a non-v-const small circle; those fall
    // back to projection.) The v<->height map is the rational meridian (nonlinear), so bisect.
    if (recog.kind == RecogSurface::SPHERE) {
        double um = 0.5 * (u0 + u1);
        Point sp = srf.point_at(um, v0), np = srf.point_at(um, v1);
        double ax[3] = {np[0]-sp[0], np[1]-sp[1], np[2]-sp[2]};
        double an = std::sqrt(dot(ax,ax));
        if (an < 1e-12) return NurbsCurve();
        ax[0]/=an; ax[1]/=an; ax[2]/=an;
        double C[3] = {recog.p1[0], recog.p1[1], recog.p1[2]};
        auto height = [&](const Point& p) { double r[3]={p[0]-C[0],p[1]-C[1],p[2]-C[2]}; return dot(r,ax); };
        auto [t0,t1] = c3d.domain();
        double hmin = 1e300, hmax = -1e300, hsum = 0; int ns = 0;
        for (int i = 0; i <= 32; ++i) {
            double h = height(c3d.point_at(t0 + (t1-t0)*i/32));
            hmin = std::min(hmin, h); hmax = std::max(hmax, h); hsum += h; ns++;
        }
        if (hmax - hmin > recog.r * 1e-4) return NurbsCurve();   // oblique small circle -> projection
        if (c3d.point_at(t0).distance(c3d.point_at(t1)) > recog.r * 1e-3) return NurbsCurve();  // not a full wrap
        double hc = hsum / ns;
        // Bisect v: height(point_at(um, v)) is monotone in v (z runs pole-to-pole).
        double va = v0, vb = v1, ha = height(srf.point_at(um, va)), hb = height(srf.point_at(um, vb));
        if ((hc - ha) * (hc - hb) > 0) return NurbsCurve();  // height out of range
        for (int it = 0; it < 60; ++it) {
            double vm = 0.5*(va + vb), hm = height(srf.point_at(um, vm));
            if ((hm - hc) * (ha - hc) <= 0) vb = vm; else { va = vm; ha = hm; }
        }
        double vc = 0.5 * (va + vb);
        return NurbsCurve::create(false, 1, {Point(u0, vc, 0.0), Point(u1, vc, 0.0)});
    }

    if (recog.kind == RecogSurface::CONE) {
        // A circle perpendicular to the cone axis (a coaxial "parallel") -> exact v=const line.
        // (recog.r is the cone HALF-ANGLE, not a length, so use a curve-length scale for tolerances.)
        double ax[3] = {recog.p2[0], recog.p2[1], recog.p2[2]};
        double an = std::sqrt(dot(ax,ax)); if (an < 1e-12) return NurbsCurve();
        ax[0]/=an; ax[1]/=an; ax[2]/=an;
        double A[3] = {recog.p1[0], recog.p1[1], recog.p1[2]};   // apex
        auto height = [&](const Point& p){ double r[3]={p[0]-A[0],p[1]-A[1],p[2]-A[2]}; return dot(r,ax); };
        auto [t0,t1] = c3d.domain();
        double clen = c3d.point_at(t0).distance(c3d.point_at(0.5*(t0+t1)));
        double hscale = std::max(clen, 1e-9);
        double hmin=1e300,hmax=-1e300,hsum=0; int ns=0;
        for (int i=0;i<=32;++i){ double h=height(c3d.point_at(t0+(t1-t0)*i/32));
            hmin=std::min(hmin,h); hmax=std::max(hmax,h); hsum+=h; ++ns; }
        if (hmax - hmin > hscale * 1e-4) return NurbsCurve();   // oblique conic -> projection
        if (c3d.point_at(t0).distance(c3d.point_at(t1)) > hscale * 1e-3) return NurbsCurve();  // not a full wrap
        double hc = hsum/ns;
        double um2 = 0.5*(u0+u1);
        double va=v0, vb=v1, ha=height(srf.point_at(um2,va)), hb=height(srf.point_at(um2,vb));
        if ((hc-ha)*(hc-hb) > 0) return NurbsCurve();   // height out of v-range
        for (int it=0; it<60; ++it){ double vmid=0.5*(va+vb), hm=height(srf.point_at(um2,vmid));
            if ((hm-hc)*(ha-hc) <= 0) vb=vmid; else { va=vmid; ha=hm; } }
        double vc=0.5*(va+vb);
        return NurbsCurve::create(false, 1, {Point(u0, vc, 0.0), Point(u1, vc, 0.0)});
    }

    // TORUS: a coaxial intersection circle (perpendicular to the torus axis, at constant horizontal
    // radius rho and constant axial z) is a v=const u-circle. theta_v = atan2(z, rho-R) is NOT
    // monotone in z but IS monotone in the surface v param, so tabulate (v -> theta_v) and invert by
    // bracketed interpolation (atan2 inverse, unwrapped) -- NOT a z-bisection. (A u=const minor
    // circle from a plane through the axis fails the spread check -> projector fallback.)
    if (recog.kind == RecogSurface::TORUS) {
        const double PI = 3.14159265358979323846, TWO_PI = 2.0 * PI;
        double C[3] = {recog.p1[0], recog.p1[1], recog.p1[2]};
        double w[3] = {recog.p2[0], recog.p2[1], recog.p2[2]};
        double wn = std::sqrt(dot(w, w)); if (wn < 1e-12) return NurbsCurve();
        w[0]/=wn; w[1]/=wn; w[2]/=wn;
        double R = recog.r, rmin = recog.r2;
        if (rmin < 1e-12 || R <= rmin) return NurbsCurve();
        auto minor_angle = [&](const Point& p) {
            double d[3] = {p[0]-C[0], p[1]-C[1], p[2]-C[2]};
            double z = dot(d, w);
            double hx = d[0]-z*w[0], hy = d[1]-z*w[1], hz = d[2]-z*w[2];
            double rho = std::sqrt(hx*hx + hy*hy + hz*hz);
            return std::atan2(z, rho - R);
        };
        auto [t0, t1] = c3d.domain();
        double aprev = 0.0, asum = 0.0, amin = 1e300, amax = -1e300; int ns = 0;
        for (int i = 0; i <= 32; ++i) {
            double a = minor_angle(c3d.point_at(t0 + (t1-t0)*i/32));
            if (i > 0) { while (a - aprev >  PI) a -= TWO_PI; while (a - aprev < -PI) a += TWO_PI; }
            aprev = a; amin = std::min(amin, a); amax = std::max(amax, a); asum += a; ++ns;
        }
        if (amax - amin > 1e-4) return NurbsCurve();
        if (c3d.point_at(t0).distance(c3d.point_at(t1)) > rmin * 1e-3) return NurbsCurve();
        double a_target = asum / ns;
        double um = 0.5 * (u0 + u1);
        const int NV = 256;
        std::vector<double> tv(NV+1), ta(NV+1);
        double ap = 0.0;
        for (int k = 0; k <= NV; ++k) {
            double v = v0 + (v1-v0)*k/NV;
            double a = minor_angle(srf.point_at(um, v));
            if (k > 0) { while (a - ap >  PI) a -= TWO_PI; while (a - ap < -PI) a += TWO_PI; }
            ap = a; tv[k] = v; ta[k] = a;
        }
        double alo = std::min(ta[0], ta[NV]), ahi = std::max(ta[0], ta[NV]);
        while (a_target < alo - 1e-9) a_target += TWO_PI;
        while (a_target > ahi + 1e-9) a_target -= TWO_PI;
        if (a_target < alo - 1e-9 || a_target > ahi + 1e-9) return NurbsCurve();
        bool incr = ta[NV] >= ta[0];
        int lo = 0, hi = NV;
        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            bool above = incr ? (ta[mid] < a_target) : (ta[mid] > a_target);
            if (above) lo = mid; else hi = mid;
        }
        double denom = ta[hi] - ta[lo];
        double f = (std::abs(denom) > 1e-15) ? (a_target - ta[lo]) / denom : 0.0;
        double vc = tv[lo] + (tv[hi] - tv[lo]) * f;
        return NurbsCurve::create(false, 1, {Point(u0, vc, 0.0), Point(u1, vc, 0.0)});
    }

    return NurbsCurve();
}

// Analytic pull-back of a 3D curve onto a recognized SPHERE, replicating OCCT ProjLib_Sphere's
// per-point inverse (EvalPnt2d): in the sphere's local frame, longitude = atan2(y,x) is EXACT
// (so a seam-straddling circle's crossing of the u-seam lands EXACTLY on u=u0/u=u1 -- the thing
// the iterative projector got ~0.18 wrong), and the nonlinear meridian v is found by bisection.
// Returns the seam-split arcs (a circle straddling the seam -> 2 arcs, each anchored on the seam),
// as exact-endpoint degree-1 polylines. Empty if not a usable sphere/circle.
static std::vector<NurbsCurve> analytic_sphere_pullback(const NurbsSurface& srf,
                                                        const RecogSurface& recog,
                                                        const NurbsCurve& c3d) {
    if (recog.kind != RecogSurface::SPHERE) return {};
    auto [u0, u1] = srf.domain(0);
    auto [v0, v1] = srf.domain(1);
    double range_u = u1 - u0;
    if (range_u < 1e-9) return {};
    auto dot = [](const double a[3], const double b[3]) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; };
    double C[3] = {recog.p1[0], recog.p1[1], recog.p1[2]};
    double um = 0.5 * (u0 + u1), vm = 0.5 * (v0 + v1);
    // Polar axis Zs (south->north pole), and the equatorial frame Xs (u=u0 meridian dir), Ys.
    Point sp = srf.point_at(um, v0), np = srf.point_at(um, v1);
    double Zs[3] = {np[0]-sp[0], np[1]-sp[1], np[2]-sp[2]};
    double zn = std::sqrt(dot(Zs,Zs)); if (zn < 1e-12) return {};
    Zs[0]/=zn; Zs[1]/=zn; Zs[2]/=zn;
    Point P0 = srf.point_at(u0, vm);
    double x0[3] = {P0[0]-C[0], P0[1]-C[1], P0[2]-C[2]};
    double h0 = dot(x0, Zs);
    double Xs[3] = {x0[0]-h0*Zs[0], x0[1]-h0*Zs[1], x0[2]-h0*Zs[2]};
    double xn = std::sqrt(dot(Xs,Xs)); if (xn < 1e-12) return {};
    Xs[0]/=xn; Xs[1]/=xn; Xs[2]/=xn;
    double Ys[3] = {Zs[1]*Xs[2]-Zs[2]*Xs[1], Zs[2]*Xs[0]-Zs[0]*Xs[2], Zs[0]*Xs[1]-Zs[1]*Xs[0]};
    const double PI = 3.14159265358979323846, TWO_PI = 2.0 * PI;
    // (u -> longitude) table along the equator. The NURBS sphere's u is the RATIONAL-quadratic
    // circle parameter, which is NOT linear in longitude (only correct at 45-deg multiples) -- a
    // linear u = u0 + (lon/2pi)*range_u approximation distorts the pulled-back circle so it bounds
    // ~2% too little flux (wrong volume). Invert the true parametrization: tabulate longitude(u)
    // on the equator (v-independent for a surface of revolution), then binary-search per point.
    const int NT = 128;
    std::vector<double> tu(NT+1), tlon(NT+1);
    for (int k = 0; k <= NT; ++k) {
        double u = u0 + range_u*k/NT;
        Point p = srf.point_at(u, vm); double r[3]={p[0]-C[0],p[1]-C[1],p[2]-C[2]};
        double lon = std::atan2(dot(r,Ys), dot(r,Xs));
        if (k > 0) { while (lon - tlon[k-1] >  PI) lon -= TWO_PI;
                     while (lon - tlon[k-1] < -PI) lon += TWO_PI; }
        tu[k] = u; tlon[k] = lon;
    }
    bool lon_incr = tlon[NT] >= tlon[0];
    double lon_lo = std::min(tlon[0], tlon[NT]), lon_hi = std::max(tlon[0], tlon[NT]);
    auto u_from_lon = [&](double lon) -> double {
        while (lon < lon_lo - 1e-9) lon += TWO_PI;
        while (lon > lon_hi + 1e-9) lon -= TWO_PI;
        int lo = 0, hi = NT;
        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            bool above = lon_incr ? (tlon[mid] < lon) : (tlon[mid] > lon);
            if (above) lo = mid; else hi = mid;
        }
        double denom = tlon[hi] - tlon[lo];
        double f = (std::abs(denom) > 1e-15) ? (lon - tlon[lo]) / denom : 0.0;
        return tu[lo] + (tu[hi] - tu[lo]) * f;
    };
    // height(v) along a meridian (independent of u by sphere symmetry) is MONOTONE pole-to-pole.
    // Precompute a (v, height) table ONCE, then invert by binary-search + linear interp per point
    // -- O(log N) with no per-point surface eval (a per-point bisection here dominated the cost).
    std::vector<double> tv(NT+1), th(NT+1);
    for (int k = 0; k <= NT; ++k) {
        double v = v0 + (v1-v0)*k/NT;
        Point p = srf.point_at(um, v); double r[3]={p[0]-C[0],p[1]-C[1],p[2]-C[2]};
        tv[k] = v; th[k] = dot(r, Zs);
    }
    bool incr = th[NT] >= th[0];
    if (std::abs(th[NT] - th[0]) < 1e-12) return {};
    auto v_from_height = [&](double h) -> double {
        if (incr) { if (h <= th[0]) return tv[0]; if (h >= th[NT]) return tv[NT]; }
        else      { if (h >= th[0]) return tv[0]; if (h <= th[NT]) return tv[NT]; }
        int lo = 0, hi = NT;
        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            bool above = incr ? (th[mid] < h) : (th[mid] > h);
            if (above) lo = mid; else hi = mid;
        }
        double denom = th[hi] - th[lo];
        double f = (std::abs(denom) > 1e-15) ? (h - th[lo]) / denom : 0.0;
        return tv[lo] + (tv[hi] - tv[lo]) * f;
    };
    // Sample the 3D curve; project each point analytically -> (u_unwrapped, v).
    auto [t0, t1] = c3d.domain();
    int n = std::max(c3d.cv_count() * 8, 120);
    std::vector<std::array<double,2>> uv;  // u_unwrapped (may go outside [u0,u1]), v
    double prev_u = 0.0;
    for (int i = 0; i <= n; ++i) {
        Point p = c3d.point_at(t0 + (t1-t0)*i/n);
        double r[3] = {p[0]-C[0], p[1]-C[1], p[2]-C[2]};
        double lon = std::atan2(dot(r,Ys), dot(r,Xs));   // (-pi, pi], exact
        double h = dot(r, Zs);
        double u = u_from_lon(lon);                       // exact NURBS u (not the linear approx)
        if (i > 0) { while (u - prev_u >  range_u*0.5) u -= range_u;
                     while (u - prev_u < -range_u*0.5) u += range_u; }
        prev_u = u;
        uv.push_back({u, v_from_height(h)});
    }
    if (uv.size() < 2) return {};
    // Split the continuous (u,v) polyline into arcs by "domain copy" index k = floor((u-u0)/range).
    // When k changes between consecutive samples the curve crosses a seam: end the current arc
    // EXACTLY on the seam (u0 or u1) and start the next on the opposite seam, each shifted into
    // [u0,u1]. So a circle straddling the seam -> two arcs anchored exactly on u0 and u1.
    std::vector<NurbsCurve> out;
    std::vector<Point> seg;
    auto kof = [&](double u) -> int { return (int)std::floor((u - u0) / range_u + 1e-9); };
    int cur_k = kof(uv[0][0]);
    seg.push_back(Point(uv[0][0] - cur_k*range_u, uv[0][1], 0.0));
    for (size_t i = 1; i < uv.size(); ++i) {
        int ki = kof(uv[i][0]);
        while (ki != cur_k) {
            int step = (ki > cur_k) ? 1 : -1;
            int nk = cur_k + step;
            double seam_cont = u0 + (step > 0 ? nk : cur_k) * range_u;  // the boundary being crossed
            double denom = uv[i][0] - uv[i-1][0];
            double f = (std::abs(denom) > 1e-15) ? (seam_cont - uv[i-1][0]) / denom : 0.0;
            f = std::min(std::max(f, 0.0), 1.0);
            double vc = uv[i-1][1] + (uv[i][1] - uv[i-1][1]) * f;
            seg.push_back(Point(seam_cont - cur_k*range_u, vc, 0.0));  // end at u1 (step>0) or u0
            if (seg.size() >= 2) out.push_back(NurbsCurve::create(false, 1, seg));
            seg.clear();
            seg.push_back(Point(seam_cont - nk*range_u, vc, 0.0));     // start at u0 (step>0) or u1
            cur_k = nk;
        }
        seg.push_back(Point(uv[i][0] - cur_k*range_u, uv[i][1], 0.0));
    }
    if (seg.size() >= 2) out.push_back(NurbsCurve::create(false, 1, seg));
    return out;
}

// Analytic pull-back of a 3D curve onto a recognized CONE, mirroring analytic_sphere_pullback:
// longitude (atan2 about the axis) is inverted via a u->longitude table (NURBS u is nonlinear in
// angle); axial height is LINEAR in v so v is a closed-form inverse. Handles a non-v-const conic
// (ellipse/hyperbola/parabola) and seam-splits on the u-seam. Empty if not a usable cone.
static std::vector<NurbsCurve> analytic_cone_pullback(const NurbsSurface& srf,
                                                      const RecogSurface& recog,
                                                      const NurbsCurve& c3d) {
    if (recog.kind != RecogSurface::CONE) return {};
    auto [u0, u1] = srf.domain(0);
    auto [v0, v1] = srf.domain(1);
    double range_u = u1 - u0;
    if (range_u < 1e-9) return {};
    auto dot = [](const double a[3], const double b[3]) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; };
    double A[3]  = {recog.p1[0], recog.p1[1], recog.p1[2]};
    double Zc[3] = {recog.p2[0], recog.p2[1], recog.p2[2]};
    double zn = std::sqrt(dot(Zc, Zc)); if (zn < 1e-12) return {};
    Zc[0]/=zn; Zc[1]/=zn; Zc[2]/=zn;
    auto height = [&](const Point& p) { double r[3]={p[0]-A[0],p[1]-A[1],p[2]-A[2]}; return dot(r, Zc); };
    double um = 0.5*(u0+u1);
    double h0 = height(srf.point_at(um, v0)), h1 = height(srf.point_at(um, v1));
    if (std::abs(h1 - h0) < 1e-12) return {};
    auto v_from_height = [&](double h) { return v0 + (h - h0)/(h1 - h0)*(v1 - v0); };
    double v_ref = (std::abs(h0) >= std::abs(h1)) ? v0 : v1;
    Point P0 = srf.point_at(u0, v_ref);
    double x0[3] = {P0[0]-A[0], P0[1]-A[1], P0[2]-A[2]};
    double hp = dot(x0, Zc);
    double Xc[3] = {x0[0]-hp*Zc[0], x0[1]-hp*Zc[1], x0[2]-hp*Zc[2]};
    double xn = std::sqrt(dot(Xc, Xc)); if (xn < 1e-12) return {};
    Xc[0]/=xn; Xc[1]/=xn; Xc[2]/=xn;
    double Yc[3] = {Zc[1]*Xc[2]-Zc[2]*Xc[1], Zc[2]*Xc[0]-Zc[0]*Xc[2], Zc[0]*Xc[1]-Zc[1]*Xc[0]};
    const double PI = 3.14159265358979323846, TWO_PI = 2.0*PI;
    const int NT = 128;
    std::vector<double> tu(NT+1), tlon(NT+1);
    for (int k = 0; k <= NT; ++k) {
        double u = u0 + range_u*k/NT;
        Point p = srf.point_at(u, v_ref); double r[3]={p[0]-A[0],p[1]-A[1],p[2]-A[2]};
        double lon = std::atan2(dot(r,Yc), dot(r,Xc));
        if (k > 0) { while (lon - tlon[k-1] >  PI) lon -= TWO_PI;
                     while (lon - tlon[k-1] < -PI) lon += TWO_PI; }
        tu[k] = u; tlon[k] = lon;
    }
    bool lon_incr = tlon[NT] >= tlon[0];
    double lon_lo = std::min(tlon[0], tlon[NT]), lon_hi = std::max(tlon[0], tlon[NT]);
    auto u_from_lon = [&](double lon) -> double {
        while (lon < lon_lo - 1e-9) lon += TWO_PI;
        while (lon > lon_hi + 1e-9) lon -= TWO_PI;
        int lo = 0, hi = NT;
        while (hi - lo > 1) { int mid = (lo+hi)/2;
            bool above = lon_incr ? (tlon[mid] < lon) : (tlon[mid] > lon);
            if (above) lo = mid; else hi = mid; }
        double denom = tlon[hi] - tlon[lo];
        double f = (std::abs(denom) > 1e-15) ? (lon - tlon[lo]) / denom : 0.0;
        return tu[lo] + (tu[hi] - tu[lo]) * f;
    };
    auto [t0, t1] = c3d.domain();
    int n = std::max(c3d.cv_count() * 8, 120);
    std::vector<std::array<double,2>> uv;
    double prev_u = 0.0, prev_lon = 0.0;
    for (int i = 0; i <= n; ++i) {
        Point p = c3d.point_at(t0 + (t1-t0)*i/n);
        double r[3] = {p[0]-A[0], p[1]-A[1], p[2]-A[2]};
        double rad = std::sqrt(std::max(0.0, dot(r,Xc)*dot(r,Xc) + dot(r,Yc)*dot(r,Yc)));
        double lon = (rad > 1e-12) ? std::atan2(dot(r,Yc), dot(r,Xc)) : prev_lon;
        prev_lon = lon;
        double u = u_from_lon(lon);
        if (i > 0) { while (u - prev_u >  range_u*0.5) u -= range_u;
                     while (u - prev_u < -range_u*0.5) u += range_u; }
        prev_u = u;
        uv.push_back({u, v_from_height(dot(r, Zc))});
    }
    if (uv.size() < 2) return {};
    std::vector<NurbsCurve> out;
    std::vector<Point> seg;
    auto kof = [&](double u) -> int { return (int)std::floor((u - u0) / range_u + 1e-9); };
    int cur_k = kof(uv[0][0]);
    seg.push_back(Point(uv[0][0] - cur_k*range_u, uv[0][1], 0.0));
    for (size_t i = 1; i < uv.size(); ++i) {
        int ki = kof(uv[i][0]);
        while (ki != cur_k) {
            int step = (ki > cur_k) ? 1 : -1;
            int nk = cur_k + step;
            double seam_cont = u0 + (step > 0 ? nk : cur_k) * range_u;
            double denom = uv[i][0] - uv[i-1][0];
            double f = (std::abs(denom) > 1e-15) ? (seam_cont - uv[i-1][0]) / denom : 0.0;
            f = std::min(std::max(f, 0.0), 1.0);
            double vc = uv[i-1][1] + (uv[i][1] - uv[i-1][1]) * f;
            seg.push_back(Point(seam_cont - cur_k*range_u, vc, 0.0));
            if (seg.size() >= 2) out.push_back(NurbsCurve::create(false, 1, seg));
            seg.clear();
            seg.push_back(Point(seam_cont - nk*range_u, vc, 0.0));
            cur_k = nk;
        }
        seg.push_back(Point(uv[i][0] - cur_k*range_u, uv[i][1], 0.0));
    }
    if (seg.size() >= 2) out.push_back(NurbsCurve::create(false, 1, seg));
    return out;
}

// ===========================================================================
// Analytic SSI for COAXIAL / canonical quadric pairs (exact conics), ported
// from OCCT IntAna_QuadQuadGeo. Each pushes the exact 3D circle(s)/line(s)/
// ellipse(s) into `out` and returns "handled": true = recognised & handled
// (out may be empty => recognised no-intersection); false = caller marches.
// ===========================================================================
static double point_axis_dist(const V3& apt, const V3& adir, const V3& P) {
    V3 u = ssi_unit(adir);
    V3 dp{P[0]-apt[0], P[1]-apt[1], P[2]-apt[2]};
    double t = ssi_dot(dp, u);
    V3 perp{dp[0]-t*u[0], dp[1]-t*u[1], dp[2]-t*u[2]};
    return std::sqrt(ssi_dot(perp, perp));
}
static double axial_coord(const V3& apt, const V3& adir, const V3& P) {
    V3 u = ssi_unit(adir);
    return (P[0]-apt[0])*u[0] + (P[1]-apt[1])*u[1] + (P[2]-apt[2])*u[2];
}
static bool axes_coaxial(const V3& p1, const V3& d1, const V3& p2, const V3& d2, double tol) {
    V3 u1 = ssi_unit(d1), u2 = ssi_unit(d2);
    V3 cx = ssi_cross(u1, u2);
    if (std::sqrt(ssi_dot(cx, cx)) > tol) return false;
    return point_axis_dist(p1, u1, p2) <= tol;
}
static void cyl_span(const NurbsSurface& srf, const V3& apt, const V3& adir, double& smin, double& smax) {
    V3 u = ssi_unit(adir);
    auto [u0, u1] = srf.domain(0); auto [v0, v1] = srf.domain(1);
    double um = 0.5 * (u0 + u1);
    smin = 1e300; smax = -1e300;
    for (double vv : {v0, v1}) {
        Point p = srf.point_at(um, vv);
        double s = (p[0]-apt[0])*u[0] + (p[1]-apt[1])*u[1] + (p[2]-apt[2])*u[2];
        smin = std::min(smin, s); smax = std::max(smax, s);
    }
}
static bool lines_closest_point(const V3& p1, const V3& d1, const V3& p2, const V3& d2, double tol, V3& Pout) {
    V3 u = ssi_unit(d1), v = ssi_unit(d2);
    V3 w0{p1[0]-p2[0], p1[1]-p2[1], p1[2]-p2[2]};
    double a = ssi_dot(u,u), b = ssi_dot(u,v), c = ssi_dot(v,v);
    double d = ssi_dot(u,w0), e = ssi_dot(v,w0);
    double den = a*c - b*b;
    if (std::abs(den) < 1e-12) return false;
    double sc = (b*e - c*d) / den, tc = (a*e - b*d) / den;
    V3 q1{p1[0]+sc*u[0], p1[1]+sc*u[1], p1[2]+sc*u[2]};
    V3 q2{p2[0]+tc*v[0], p2[1]+tc*v[1], p2[2]+tc*v[2]};
    V3 diff{q1[0]-q2[0], q1[1]-q2[1], q1[2]-q2[2]};
    if (std::sqrt(ssi_dot(diff,diff)) > tol) return false;
    Pout = V3{0.5*(q1[0]+q2[0]), 0.5*(q1[1]+q2[1]), 0.5*(q1[2]+q2[2])};
    return true;
}
static bool ssi_cylinder_sphere(const RecogSurface& cyl, const RecogSurface& sph, std::vector<NurbsCurve>& out) {
    const double kTol = 1e-6;
    V3 P = cyl.p1, w = ssi_unit(cyl.p2); double rc = cyl.r;
    V3 C = sph.p1; double R = sph.r;
    if (point_axis_dist(P, w, C) > kTol) return false;
    if (R < rc - kTol) return true;
    double dist = std::sqrt(std::max(0.0, R*R - rc*rc));
    auto [xa, ya] = ortho_basis(w);
    if (dist <= kTol) { out.push_back(exact_circle(C[0], C[1], C[2], xa, ya, rc)); return true; }
    for (double s : {dist, -dist}) {
        V3 cc{C[0]+s*w[0], C[1]+s*w[1], C[2]+s*w[2]};
        out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rc));
    }
    return true;
}
static bool ssi_cylinder_cone(const RecogSurface& cyl, const RecogSurface& cone, std::vector<NurbsCurve>& out) {
    const double kTol = 1e-6;
    V3 Pc = cyl.p1, w = ssi_unit(cyl.p2); double rc = cyl.r;
    V3 apex = cone.p1, a = ssi_unit(cone.p2); double alpha = cone.r;
    if (!axes_coaxial(Pc, w, apex, a, kTol)) return false;
    double ta = std::tan(alpha);
    if (ta < 1e-9) return false;
    double s = rc / ta;
    if (s < kTol) return true;
    V3 cc{apex[0]+s*a[0], apex[1]+s*a[1], apex[2]+s*a[2]};
    auto [xa, ya] = ortho_basis(a);
    out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rc));
    return true;
}
static bool ssi_cone_sphere(const RecogSurface& cone, const RecogSurface& sph, std::vector<NurbsCurve>& out) {
    const double kTol = 1e-6;
    V3 apex = cone.p1, a = ssi_unit(cone.p2); double alpha = cone.r;
    V3 C = sph.p1; double R = sph.r;
    if (point_axis_dist(apex, a, C) > kTol) return false;
    double dsign = axial_coord(apex, a, C);
    double d = std::abs(dsign);
    V3 dir = (d > kTol && dsign < 0.0) ? V3{-a[0],-a[1],-a[2]} : a;
    double t = std::tan(alpha), t2 = t*t;
    double A = 1.0 + t2, B = 2.0*t2*d, Cq = t2*d*d - R*R;
    double disc = B*B - 4.0*A*Cq;
    if (disc < -kTol) return true;
    double sq = std::sqrt(std::max(0.0, disc));
    std::vector<double> xs;
    if (sq <= kTol) xs = { -B/(2.0*A) };
    else            xs = { (-B - sq)/(2.0*A), (-B + sq)/(2.0*A) };
    auto [xa, ya] = ortho_basis(a);
    for (double x : xs) {
        double sAx = d + x;
        if (sAx < kTol) continue;
        double rr = t * sAx;
        if (rr < kTol) continue;
        V3 cc{apex[0]+sAx*dir[0], apex[1]+sAx*dir[1], apex[2]+sAx*dir[2]};
        out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rr));
    }
    return true;
}
static bool ssi_cylinder_cylinder(const NurbsSurface& sa, const RecogSurface& A,
                                  const NurbsSurface& sb, const RecogSurface& B, std::vector<NurbsCurve>& out) {
    const double kTol = 1e-6;
    V3 P1 = A.p1, w1 = ssi_unit(A.p2); double R1 = A.r;
    V3 P2 = B.p1, w2 = ssi_unit(B.p2); double R2 = B.r;
    V3 cx = ssi_cross(w1, w2);
    double sinmag = std::sqrt(ssi_dot(cx, cx));
    if (sinmag <= kTol) {
        double dline = point_axis_dist(P1, w1, P2);
        if (dline <= kTol) { if (std::abs(R1 - R2) <= kTol) return false; return true; }
        double off = ssi_dot(V3{P2[0]-P1[0],P2[1]-P1[1],P2[2]-P1[2]}, w1);
        V3 P2p{P2[0]-off*w1[0], P2[1]-off*w1[1], P2[2]-off*w1[2]};
        double d = dline;
        if (d > R1 + R2 + kTol) return true;
        if (d < std::abs(R1 - R2) - kTol) return true;
        V3 xdir = ssi_unit(V3{P2p[0]-P1[0], P2p[1]-P1[1], P2p[2]-P1[2]});
        V3 ydir = ssi_unit(ssi_cross(w1, xdir));
        double aa = (R1*R1 - R2*R2 + d*d) / (2.0*d);
        double h  = std::sqrt(std::max(0.0, R1*R1 - aa*aa));
        V3 foot{P1[0]+aa*xdir[0], P1[1]+aa*xdir[1], P1[2]+aa*xdir[2]};
        double s0a, s1a, s0b, s1b;
        cyl_span(sa, P1, w1, s0a, s1a); cyl_span(sb, P1, w1, s0b, s1b);
        double slo = std::max(s0a, s0b), shi = std::min(s1a, s1b);
        if (shi - slo <= kTol) return true;
        auto emit = [&](const V3& bp) {
            V3 e0{bp[0]+slo*w1[0], bp[1]+slo*w1[1], bp[2]+slo*w1[2]};
            V3 e1{bp[0]+shi*w1[0], bp[1]+shi*w1[1], bp[2]+shi*w1[2]};
            NurbsCurve ln = NurbsCurve::create(false, 1, {Point(e0[0],e0[1],e0[2]), Point(e1[0],e1[1],e1[2])});
            ln.set_domain(0.0, 1.0); out.push_back(ln);
        };
        if (h <= kTol) emit(foot);
        else { emit(V3{foot[0]+h*ydir[0], foot[1]+h*ydir[1], foot[2]+h*ydir[2]});
               emit(V3{foot[0]-h*ydir[0], foot[1]-h*ydir[1], foot[2]-h*ydir[2]}); }
        return true;
    }
    double Rmax = std::max(R1, R2);
    if (Rmax < 1e-12 || std::abs(R1 - R2) / Rmax > 1e-6) return false;
    V3 Pint;
    if (!lines_closest_point(P1, w1, P2, w2, kTol, Pint)) return false;
    double R = 0.5 * (R1 + R2);
    double ang = std::acos(std::max(-1.0, std::min(1.0, ssi_dot(w1, w2))));
    double sh = std::sin(0.5*ang), ch = std::cos(0.5*ang);
    if (sh < 1e-9 || ch < 1e-9) return false;
    V3 minor = ssi_unit(cx);
    V3 maj1  = ssi_unit(V3{w1[0]+w2[0], w1[1]+w2[1], w1[2]+w2[2]});
    V3 maj2  = ssi_unit(V3{w1[0]-w2[0], w1[1]-w2[1], w1[2]-w2[2]});
    out.push_back(exact_ellipse(Pint[0], Pint[1], Pint[2], maj1, minor, R/sh, R));
    out.push_back(exact_ellipse(Pint[0], Pint[1], Pint[2], maj2, minor, R/ch, R));
    return true;
}

// --- COAXIAL TORUS pairs (exact circles, ported from IntAna_QuadQuadGeo). Each circle is a
// v=const u-circle: center on the torus axis at C+z*w, radius=horizontal radius, normal=w. Gate on
// coaxiality + ring torus (minor r2 < major r). true+[] = recognised no-hit; false = marcher. ----
static bool ssi_cylinder_torus(const RecogSurface& cyl, const RecogSurface& tor, std::vector<NurbsCurve>& out) {
    const double kTol = 1e-6;
    V3 P = cyl.p1, wc = ssi_unit(cyl.p2); double rc = cyl.r;
    V3 C = tor.p1, w = ssi_unit(tor.p2); double R = tor.r, r = tor.r2;
    if (r >= R - kTol) return false;
    if (!axes_coaxial(P, wc, C, w, kTol)) return false;
    double dr = rc - R, h2 = r*r - dr*dr;
    if (h2 < -kTol) return true;
    double h = std::sqrt(std::max(0.0, h2));
    auto [xa, ya] = ortho_basis(w);
    std::vector<double> zs = (h <= kTol) ? std::vector<double>{0.0} : std::vector<double>{h, -h};
    for (double z : zs) { V3 cc{C[0]+z*w[0], C[1]+z*w[1], C[2]+z*w[2]};
        out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rc)); }
    return true;
}
static bool ssi_cone_torus(const RecogSurface& cone, const RecogSurface& tor, std::vector<NurbsCurve>& out) {
    const double kTol = 1e-6;
    V3 apex = cone.p1, a = ssi_unit(cone.p2); double alpha = cone.r;
    V3 C = tor.p1, w = ssi_unit(tor.p2); double R = tor.r, r = tor.r2;
    if (r >= R - kTol) return false;
    if (!axes_coaxial(apex, a, C, w, kTol)) return false;
    double t = std::tan(alpha); if (t < 1e-9) return false;
    double za = axial_coord(C, w, apex);
    double A = t*t + 1.0;
    auto [xa, ya] = ortho_basis(w);
    auto solve_emit = [&](double Rsign) {
        double B  = -2.0 * t * (t*za + Rsign);
        double Cc = (t*za + Rsign)*(t*za + Rsign) - r*r;
        double disc = B*B - 4.0*A*Cc;
        if (disc < -kTol) return;
        double sq = std::sqrt(std::max(0.0, disc));
        std::vector<double> zs = (sq <= kTol) ? std::vector<double>{ -B/(2.0*A) }
                                              : std::vector<double>{ (-B-sq)/(2.0*A), (-B+sq)/(2.0*A) };
        for (double z : zs) { double rad = t * std::abs(z - za); if (rad < kTol) continue;
            V3 cc{C[0]+z*w[0], C[1]+z*w[1], C[2]+z*w[2]};
            out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rad)); }
    };
    solve_emit(+R); solve_emit(-R);
    return true;
}
static bool ssi_sphere_torus(const RecogSurface& sph, const RecogSurface& tor, std::vector<NurbsCurve>& out) {
    const double kTol = 1e-6;
    V3 S = sph.p1; double rsph = sph.r;
    V3 C = tor.p1, w = ssi_unit(tor.p2); double R = tor.r, r = tor.r2;
    if (r >= R - kTol) return false;
    if (point_axis_dist(C, w, S) > kTol) return false;
    double zs = axial_coord(C, w, S);
    double d = std::sqrt(R*R + zs*zs);
    if (d < kTol) return true;
    if (d - kTol > r + rsph || d + kTol < std::abs(r - rsph)) return true;
    double aa = 0.5*(r*r - rsph*rsph + d*d)/d;
    double h  = std::sqrt(std::max(0.0, r*r - aa*aa));
    double dirx = (0.0 - R)/d, dirz = (zs - 0.0)/d;
    double phx = R + aa*dirx, phz = aa*dirz;
    double perpx = -dirz, perpz = dirx;
    auto [xa, ya] = ortho_basis(w);
    std::vector<int> signs = (h <= kTol) ? std::vector<int>{0} : std::vector<int>{+1, -1};
    for (int s : signs) { double xi = phx + s*h*perpx, z = phz + s*h*perpz, rad = std::abs(xi);
        if (rad < kTol) continue; V3 cc{C[0]+z*w[0], C[1]+z*w[1], C[2]+z*w[2]};
        out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rad)); }
    return true;
}
static bool ssi_torus_torus(const RecogSurface& ta, const RecogSurface& tb, std::vector<NurbsCurve>& out) {
    const double kTol = 1e-6;
    V3 C1 = ta.p1, w  = ssi_unit(ta.p2); double R1 = ta.r, r1 = ta.r2;
    V3 C2 = tb.p1, w2 = ssi_unit(tb.p2); double R2 = tb.r, r2 = tb.r2;
    if (r1 >= R1 - kTol || r2 >= R2 - kTol) return false;
    if (!axes_coaxial(C1, w, C2, w2, kTol)) return false;
    double z2 = axial_coord(C1, w, C2);
    double dxR = R2 - R1, d = std::sqrt(dxR*dxR + z2*z2);
    if (d < kTol) return false;
    if (d - kTol > r1 + r2 || d + kTol < std::abs(r1 - r2)) return true;
    double aa = 0.5*(r1*r1 - r2*r2 + d*d)/d;
    double h  = std::sqrt(std::max(0.0, r1*r1 - aa*aa));
    double dirx = dxR/d, dirz = z2/d;
    double phx = R1 + aa*dirx, phz = aa*dirz;
    double perpx = -dirz, perpz = dirx;
    auto [xa, ya] = ortho_basis(w);
    std::vector<int> signs = (h <= kTol) ? std::vector<int>{0} : std::vector<int>{+1, -1};
    for (int s : signs) { double xi = phx + s*h*perpx, z = phz + s*h*perpz, rad = std::abs(xi);
        if (rad < kTol) continue; V3 cc{C1[0]+z*w[0], C1[1]+z*w[1], C1[2]+z*w[2]};
        out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rad)); }
    return true;
}

static AnalyticResult analytic_ssi(const NurbsSurface& a, const NurbsSurface& b, double tolerance) {
    AnalyticResult res;
    double rtol = std::max(tolerance, 1e-7) * 1e4;
    RecogSurface ra = recognize_surface(a, rtol);
    RecogSurface rb = recognize_surface(b, rtol);
    if (ra.kind == RecogSurface::NONE || rb.kind == RecogSurface::NONE) return res;  // None

    // Each handler returns a list of exact 3D curves (empty = recognized but no
    // intersection); a false "handled" flag means not analytically handled
    // (caller marches).
    std::vector<NurbsCurve> c3_list;
    bool handled = true;
    using K = RecogSurface;
    auto single = [&](bool ok, NurbsCurve& c3) { if (ok) c3_list.push_back(c3); };

    NurbsCurve c3;
    if (ra.kind == K::PLANE && rb.kind == K::PLANE) {
        bool empty = false;
        if (ssi_plane_plane(a, ra, b, rb, c3, empty)) c3_list.push_back(c3);
        else if (!empty) return res;  // parallel/coincident -> marcher (NOT_ANALYTIC)
        // empty -> recognized, finite faces disjoint -> HIT with [] (no curves)
    }
    else if (ra.kind == K::PLANE && rb.kind == K::SPHERE)        single(ssi_plane_sphere(ra, rb, c3), c3);
    else if (ra.kind == K::SPHERE && rb.kind == K::PLANE)   single(ssi_plane_sphere(rb, ra, c3), c3);
    else if (ra.kind == K::PLANE && rb.kind == K::CYLINDER) single(ssi_plane_cylinder(ra, rb, c3), c3);
    else if (ra.kind == K::CYLINDER && rb.kind == K::PLANE) single(ssi_plane_cylinder(rb, ra, c3), c3);
    else if (ra.kind == K::PLANE && rb.kind == K::CONE)     handled = ssi_plane_cone(ra, rb, b, c3_list);
    else if (ra.kind == K::CONE && rb.kind == K::PLANE)     handled = ssi_plane_cone(rb, ra, a, c3_list);
    else if (ra.kind == K::PLANE && rb.kind == K::TORUS)    handled = ssi_plane_torus(ra, rb, c3_list);
    else if (ra.kind == K::TORUS && rb.kind == K::PLANE)    handled = ssi_plane_torus(rb, ra, c3_list);
    else if (ra.kind == K::SPHERE && rb.kind == K::SPHERE) {
        V3 c1 = ra.p1; double r1 = ra.r;
        V3 c2 = rb.p1; double r2 = rb.r;
        V3 dv{c2[0]-c1[0], c2[1]-c1[1], c2[2]-c1[2]};
        double dist = std::sqrt(dv[0]*dv[0] + dv[1]*dv[1] + dv[2]*dv[2]);
        if (1e-12 < dist && dist < r1 + r2 && dist > std::abs(r1 - r2)) {
            V3 nu{dv[0]/dist, dv[1]/dist, dv[2]/dist};
            double aa = (dist*dist + r1*r1 - r2*r2) / (2.0*dist);
            double rr2 = r1*r1 - aa*aa;
            if (rr2 > 0.0) {
                V3 cc{c1[0]+aa*nu[0], c1[1]+aa*nu[1], c1[2]+aa*nu[2]};
                auto [xa, ya] = ortho_basis(nu);
                c3 = exact_circle(cc[0], cc[1], cc[2], xa, ya, std::sqrt(rr2));
                c3_list.push_back(c3);
            }
        }
    }
    // Coaxial / canonical quadric pairs (exact conics from IntAna_QuadQuadGeo).
    else if (ra.kind == K::CYLINDER && rb.kind == K::SPHERE)   handled = ssi_cylinder_sphere(ra, rb, c3_list);
    else if (ra.kind == K::SPHERE   && rb.kind == K::CYLINDER) handled = ssi_cylinder_sphere(rb, ra, c3_list);
    else if (ra.kind == K::CYLINDER && rb.kind == K::CONE)     handled = ssi_cylinder_cone(ra, rb, c3_list);
    else if (ra.kind == K::CONE     && rb.kind == K::CYLINDER) handled = ssi_cylinder_cone(rb, ra, c3_list);
    else if (ra.kind == K::CONE     && rb.kind == K::SPHERE)   handled = ssi_cone_sphere(ra, rb, c3_list);
    else if (ra.kind == K::SPHERE   && rb.kind == K::CONE)     handled = ssi_cone_sphere(rb, ra, c3_list);
    else if (ra.kind == K::CYLINDER && rb.kind == K::CYLINDER) handled = ssi_cylinder_cylinder(a, ra, b, rb, c3_list);
    else if (ra.kind == K::CYLINDER && rb.kind == K::TORUS)    handled = ssi_cylinder_torus(ra, rb, c3_list);
    else if (ra.kind == K::TORUS    && rb.kind == K::CYLINDER) handled = ssi_cylinder_torus(rb, ra, c3_list);
    else if (ra.kind == K::CONE     && rb.kind == K::TORUS)    handled = ssi_cone_torus(ra, rb, c3_list);
    else if (ra.kind == K::TORUS    && rb.kind == K::CONE)     handled = ssi_cone_torus(rb, ra, c3_list);
    else if (ra.kind == K::SPHERE   && rb.kind == K::TORUS)    handled = ssi_sphere_torus(ra, rb, c3_list);
    else if (ra.kind == K::TORUS    && rb.kind == K::SPHERE)   handled = ssi_sphere_torus(rb, ra, c3_list);
    else if (ra.kind == K::TORUS    && rb.kind == K::TORUS)    handled = ssi_torus_torus(ra, rb, c3_list);
    else {
        return res;  // not an analytically-exact pair -> marcher (None)
    }

    if (!handled) return res;  // recognized but not analytically handled -> marcher (None)

    // The 3D curves are exact; pull back onto each surface. Try the closed-form analytic
    // pcurve first (OCCT ProjLib-style, no sampling); fall back to projection otherwise.
    for (const NurbsCurve& cc3 : c3_list) {
        NurbsCurve pa = analytic_pcurve(a, ra, cc3);
        NurbsCurve pb = analytic_pcurve(b, rb, cc3);
        if (!pa.is_valid()) { auto v = Closest::surface_curve(a, cc3); if (!v.empty()) pa = v[0]; }
        if (!pb.is_valid()) { auto v = Closest::surface_curve(b, cc3); if (!v.empty()) pb = v[0]; }
        if (pa.is_valid() && pb.is_valid())
            res.triples.push_back(std::make_tuple(cc3, pa, pb));
    }
    res.status = AnalyticResult::HIT;  // [] or [triples...]; either way analytic
    return res;
}

} // namespace

std::vector<NurbsCurve> Intersection::surface_plane(
    const NurbsSurface& surface,
    const Plane& plane,
    double tolerance
) {
    if (!surface.is_valid()) return {};
    if (tolerance <= 0.0) tolerance = Tolerance::ZERO_TOLERANCE;

    auto [traces, step, uv_to_3d, uv_to_3d_min] = surface_plane_traces(surface, plane, tolerance);

    std::vector<NurbsCurve> result;
    for (auto& [uv_trace, uv_unwrapped, is_loop] : traces) {
        std::vector<Point> all_pts(uv_trace.size());
        for (size_t i = 0; i < uv_trace.size(); i++)
            all_pts[i] = surface.point_at(uv_trace[i].first, uv_trace[i].second);
        NurbsCurve crv = surface_plane_fit_3d(all_pts, is_loop, plane, step, uv_to_3d, uv_to_3d_min);
        if (!crv.is_valid()) continue;

        // Deduplicate: skip if ALL sample points are close to an existing curve
        auto [ct0, ct1] = crv.domain();
        double dup_tol = step * uv_to_3d * 3.0;
        bool dup = false;
        for (auto& existing : result) {
            auto [et0, et1] = existing.domain();
            // Sample both curves at 3 points and check if all are close
            bool all_close = true;
            for (double f : {0.25, 0.5, 0.75}) {
                Point cp = crv.point_at(ct0 + (ct1 - ct0) * f);
                Point ep = existing.point_at(et0 + (et1 - et0) * f);
                Point em = existing.point_at((et0 + et1) * 0.5);
                double d = std::min(cp.distance(ep), cp.distance(em));
                if (d > dup_tol) { all_close = false; break; }
            }
            if (all_close) { dup = true; break; }
        }
        if (!dup) result.push_back(std::move(crv));
    }

    return result;
}

std::vector<std::pair<NurbsCurve, NurbsCurve>> Intersection::surface_plane_uv(
    const NurbsSurface& surface,
    const Plane& plane,
    double tolerance
) {
    if (!surface.is_valid()) return {};
    if (tolerance <= 0.0) tolerance = Tolerance::ZERO_TOLERANCE;

    auto [u0, u1] = surface.domain(0);
    auto [v0, v1] = surface.domain(1);
    double range_u = u1 - u0;
    double range_v = v1 - v0;
    bool closed_u = surface.is_closed(0);
    bool closed_v = surface.is_closed(1);

    auto wrap_u = [&](double u) -> double {
        if (closed_u) {
            double t = std::fmod(u - u0, range_u);
            if (t < 0) t += range_u;
            return u0 + t;
        }
        return std::max(u0, std::min(u, u1));
    };
    auto wrap_v = [&](double v) -> double {
        if (closed_v) {
            double t = std::fmod(v - v0, range_v);
            if (t < 0) t += range_v;
            return v0 + t;
        }
        return std::max(v0, std::min(v, v1));
    };

    Vector pn = plane.z_axis();
    Point p0 = plane.origin();

    // Analytical g + gradient via single evaluate call
    // evaluate order: [S, Sv, Su] for num_derivs=1
    auto g_and_grad = [&](double u, double v, double& val, double& gu, double& gv) {
        auto derivs = surface.evaluate(wrap_u(u), wrap_v(v), 1);
        const Vector& S = derivs[0];
        const Vector& Su = derivs[2];
        const Vector& Sv = derivs[1];
        val = (S[0]-p0[0])*pn[0] + (S[1]-p0[1])*pn[1] + (S[2]-p0[2])*pn[2];
        gu = Su[0]*pn[0] + Su[1]*pn[1] + Su[2]*pn[2];
        gv = Sv[0]*pn[0] + Sv[1]*pn[1] + Sv[2]*pn[2];
    };

    // Refine the free coordinate along a fixed seam iso-line so g = 0
    auto seam_newton = [&](double cu, double cv_, int axis) -> std::pair<double, double> {
        for (int iter = 0; iter < 10; iter++) {
            double val, gu, gv;
            g_and_grad(cu, cv_, val, gu, gv);
            if (std::abs(val) < tolerance) break;
            if (axis == 0) {
                if (std::abs(gv) < 1e-14) break;
                cv_ = cv_ - val / gv;
            } else {
                if (std::abs(gu) < 1e-14) break;
                cu = cu - val / gu;
            }
        }
        return {cu, cv_};
    };

    auto [traces, step, uv_to_3d, uv_to_3d_min] = surface_plane_traces(surface, plane, tolerance);

    double fit_tol = step * (uv_to_3d + uv_to_3d_min) * 0.5;
    double dup_tol = step * uv_to_3d * 3.0;

    std::vector<std::pair<NurbsCurve, NurbsCurve>> result;
    std::vector<std::vector<Point>> kept_pts3;
    for (auto& [uv_trace, uv_unwrapped, is_loop] : traces) {
        // Trace-level dedup against already kept traces (3-sample proximity)
        int m = (int)uv_trace.size();
        std::vector<Point> trace_pts3(m);
        for (int i = 0; i < m; i++)
            trace_pts3[i] = surface.point_at(uv_trace[i].first, uv_trace[i].second);
        bool dup = false;
        for (auto& other : kept_pts3) {
            bool all_close = true;
            for (double f : {0.25, 0.5, 0.75}) {
                Point cp = trace_pts3[(int)((m - 1) * f)];
                double dmin = dup_tol + 1.0;
                for (size_t k = 0; k < other.size(); k += 5)
                    dmin = std::min(dmin, cp.distance(other[k]));
                if (dmin > dup_tol) { all_close = false; break; }
            }
            if (all_close) { dup = true; break; }
        }
        if (dup) continue;
        kept_pts3.push_back(trace_pts3);

        // Extend closed loops with a virtual copy of the first point
        std::vector<std::pair<double, double>> pts = uv_unwrapped;
        double closure_du = 0.0;
        double closure_dv = 0.0;
        if (is_loop && pts.size() >= 2) {
            double du_j = pts[0].first - pts.back().first;
            double dv_j = pts[0].second - pts.back().second;
            if (closed_u) {
                while (du_j > range_u * 0.5) du_j -= range_u;
                while (du_j < -range_u * 0.5) du_j += range_u;
            }
            if (closed_v) {
                while (dv_j > range_v * 0.5) dv_j -= range_v;
                while (dv_j < -range_v * 0.5) dv_j += range_v;
            }
            closure_du = (pts.back().first + du_j) - pts[0].first;
            closure_dv = (pts.back().second + dv_j) - pts[0].second;
            pts.push_back({pts[0].first + closure_du, pts[0].second + closure_dv});
        }

        // Insert seam crossings (Newton-refined onto the seam iso-line)
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
                    if (0.0 < t && t < 1.0) crossings.push_back({t, 0, L});
                }
            }
            if (closed_v && std::abs(pb.second - pa.second) > 1e-15) {
                int k0 = (int)std::floor((pa.second - v0) / range_v);
                int k1 = (int)std::floor((pb.second - v0) / range_v);
                for (int k = std::min(k0, k1) + 1; k <= std::max(k0, k1); k++) {
                    double L = v0 + k * range_v;
                    double t = (L - pa.second) / (pb.second - pa.second);
                    if (0.0 < t && t < 1.0) crossings.push_back({t, 1, L});
                }
            }
            std::sort(crossings.begin(), crossings.end());
            for (const auto& [t, axis, L] : crossings) {
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
            // An interior sample sitting exactly on a seam level is a crossing
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
                if (on_seam) cross_idx.push_back((int)out_pts.size() - 1);
            }
        }

        // Split at seam crossings into continuous UV pieces
        bool wrap_drift = std::fabs(closure_du) > range_u * 0.5 || std::fabs(closure_dv) > range_v * 0.5;
        std::vector<std::pair<std::vector<std::pair<double, double>>, bool>> pieces;
        if (cross_idx.size() == 0) {
            // A loop with net unwrap drift wraps the seam with endpoints on it:
            // emit as one open piece spanning the full period
            pieces.push_back({out_pts, is_loop && !wrap_drift});
        } else if (is_loop) {
            for (size_t ci = 0; ci + 1 < cross_idx.size(); ci++) {
                int a = cross_idx[ci];
                int b = cross_idx[ci + 1];
                pieces.push_back({std::vector<std::pair<double, double>>(out_pts.begin() + a, out_pts.begin() + b + 1), false});
            }
            std::vector<std::pair<double, double>> wrap_piece(out_pts.begin() + cross_idx.back(), out_pts.end());
            for (int pi = 1; pi <= cross_idx[0]; pi++)
                wrap_piece.push_back({out_pts[pi].first + closure_du, out_pts[pi].second + closure_dv});
            pieces.push_back({wrap_piece, false});
        } else {
            std::vector<int> bounds;
            bounds.push_back(0);
            for (int ci : cross_idx) bounds.push_back(ci);
            bounds.push_back((int)out_pts.size() - 1);
            for (size_t bi = 0; bi + 1 < bounds.size(); bi++) {
                int a = bounds[bi];
                int b = bounds[bi + 1];
                if (b > a)
                    pieces.push_back({std::vector<std::pair<double, double>>(out_pts.begin() + a, out_pts.begin() + b + 1), false});
            }
        }

        for (auto& [piece_pts, piece_loop] : pieces) {
            if (piece_pts.size() < 2) continue;
            // Shift the piece into the base domain
            std::pair<double, double> mid = piece_pts[piece_pts.size() / 2];
            if (closed_u) {
                int k_u = (int)std::floor((mid.first - u0) / range_u);
                if (k_u != 0)
                    for (auto& p : piece_pts) p.first -= k_u * range_u;
            }
            if (closed_v) {
                int k_v = (int)std::floor((mid.second - v0) / range_v);
                if (k_v != 0)
                    for (auto& p : piece_pts) p.second -= k_v * range_v;
            }

            std::vector<Point> pts3(piece_pts.size());
            for (size_t i = 0; i < piece_pts.size(); i++)
                pts3[i] = surface.point_at(wrap_u(piece_pts[i].first), wrap_v(piece_pts[i].second));

            // Fit the 3D curve (plane-constrained; circle/ellipse for full loops)
            NurbsCurve crv3 = surface_plane_fit_3d(pts3, piece_loop, plane, step, uv_to_3d, uv_to_3d_min, false);
            if (!crv3.is_valid())
                crv3 = piece_loop
                    ? NurbsCurve::create_interpolated(pts3, CurveNurbsKnotStyle::ChordPeriodic)
                    : NurbsCurve::create_interpolated(pts3);
            if (!crv3.is_valid()) continue;

            // Fit the UV pcurve
            std::vector<Point> pts_uv(piece_pts.size());
            for (size_t i = 0; i < piece_pts.size(); i++)
                pts_uv[i] = Point(piece_pts[i].first, piece_pts[i].second, 0.0);
            int mp = (int)pts_uv.size();
            double fit_tol_uv = step;
            double total_turning = 0;
            for (int i = 1; i < mp - 1; i++) {
                double dx1 = pts_uv[i][0]-pts_uv[i-1][0], dy1 = pts_uv[i][1]-pts_uv[i-1][1];
                double dx2 = pts_uv[i+1][0]-pts_uv[i][0], dy2 = pts_uv[i+1][1]-pts_uv[i][1];
                double l1 = std::hypot(dx1, dy1), l2 = std::hypot(dx2, dy2);
                if (l1 > 1e-14 && l2 > 1e-14) {
                    double c = (dx1*dx2+dy1*dy2) / (l1*l2);
                    c = std::max(-1.0, std::min(1.0, c));
                    total_turning += std::acos(c);
                }
            }

            std::vector<double> chords(mp, 0.0);
            double total_len = 0;
            for (int i = 1; i < mp; i++) {
                total_len += pts_uv[i].distance(pts_uv[i-1]);
                chords[i] = total_len;
            }
            if (piece_loop && mp > 1) total_len += pts_uv[0].distance(pts_uv[mp-1]);
            if (total_len > 1e-14) for (int i = 1; i < mp; i++) chords[i] /= total_len;

            int target_cvs = std::max(8, (int)(total_turning / 0.5) + 6);
            int max_cvs = mp - 1;
            NurbsCurve pcurve;
            for (int attempt = 0; attempt < 5; attempt++) {
                if (target_cvs > max_cvs) break;
                pcurve = NurbsCurve::create_fitted(pts_uv, target_cvs, 3, piece_loop);
                if (!pcurve.is_valid()) break;
                auto [ft0, ft1] = pcurve.domain();
                double max_dev = 0;
                for (int i = 0; i < mp; i++) {
                    double t = ft0 + (ft1 - ft0) * chords[i];
                    max_dev = std::max(max_dev, pcurve.point_at(t).distance(pts_uv[i]));
                }
                if (max_dev < fit_tol_uv) break;
                target_cvs = std::min(target_cvs * 2, max_cvs);
            }

            if (!pcurve.is_valid())
                pcurve = piece_loop
                    ? NurbsCurve::create_interpolated(pts_uv, CurveNurbsKnotStyle::ChordPeriodic)
                    : NurbsCurve::create_interpolated(pts_uv);
            if (!pcurve.is_valid()) continue;

            crv3.set_domain(0.0, 1.0);
            pcurve.set_domain(0.0, 1.0);

            // Validate: lifted pcurve must stay on the plane within the fit budget
            double vali_tol = std::max(10.0 * tolerance, fit_tol * 2.0);
            double max_off = 0;
            for (int i = 0; i < 17; i++) {
                double t = i / 16.0;
                Point pc = pcurve.point_at(t);
                double val, gu, gv;
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

std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> Intersection::surface_surface(
    const NurbsSurface& a,
    const NurbsSurface& b,
    double tolerance
) {
    if (!a.is_valid() || !b.is_valid()) return {};
    if (tolerance <= 0.0) tolerance = Tolerance::ZERO_TOLERANCE;

    // Analytic dispatch: closed-form exact conics for recognized quadric pairs.
    // HIT carries all analytic triples (empty = recognized but no intersection);
    // NOT_ANALYTIC falls through to marching.
    AnalyticResult _ana = analytic_ssi(a, b, tolerance);
    if (_ana.status != AnalyticResult::NOT_ANALYTIC)
        return _ana.triples;

    // Planar dispatch: reuse the plane tracer when either surface is planar
    auto plane_from = [](const NurbsSurface& srf) -> Plane {
        auto [s0, s1] = srf.domain(0);
        auto [t0, t1] = srf.domain(1);
        Point po = srf.point_at((s0+s1)*0.5, (t0+t1)*0.5);
        Vector nn = srf.normal_at((s0+s1)*0.5, (t0+t1)*0.5);
        Vector nv(nn[0], nn[1], nn[2]);
        return Plane::from_point_normal(po, nv);
    };

    if (a.is_planar(nullptr, 1e-9)) {
        Plane plane = plane_from(a);
        std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> result;
        for (auto& [c3, pb] : surface_plane_uv(b, plane, tolerance)) {
            std::vector<NurbsCurve> pas = Closest::surface_curve(a, c3);
            if (pas.size() == 1) result.push_back({c3, pas[0], pb});
        }
        return result;
    }
    if (b.is_planar(nullptr, 1e-9)) {
        Plane plane = plane_from(b);
        std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> result;
        for (auto& [c3, pa] : surface_plane_uv(a, plane, tolerance)) {
            std::vector<NurbsCurve> pbs = Closest::surface_curve(b, c3);
            if (pbs.size() == 1) result.push_back({c3, pa, pbs[0]});
        }
        return result;
    }

    // ---- Per-surface context ----
    auto [au0, au1] = a.domain(0);
    auto [av0, av1] = a.domain(1);
    auto [bu0, bu1] = b.domain(0);
    auto [bv0, bv1] = b.domain(1);
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
                if (f < 0) f += rng;
                return c0 + f;
            }
            return std::max(c0, std::min(t, c1));
        };
    };

    auto a_wrap_u = make_wrap(au0, au1, a_range_u, a_closed_u);
    auto a_wrap_v = make_wrap(av0, av1, a_range_v, a_closed_v);
    auto b_wrap_u = make_wrap(bu0, bu1, b_range_u, b_closed_u);
    auto b_wrap_v = make_wrap(bv0, bv1, b_range_v, b_closed_v);

    // eval returns (S, Su, Sv); evaluate order is [S, Sv, Su]
    auto eval_a = [&](double u, double v, Vector& S, Vector& Su, Vector& Sv) {
        auto d = a.evaluate(a_wrap_u(u), a_wrap_v(v), 1);
        S = d[0]; Su = d[2]; Sv = d[1];
    };
    auto eval_b = [&](double u, double v, Vector& S, Vector& Su, Vector& Sv) {
        auto d = b.evaluate(b_wrap_u(u), b_wrap_v(v), 1);
        S = d[0]; Su = d[2]; Sv = d[1];
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

    // A seed cell box: AABB (0..5) + cell-center uv (6, 7)
    typedef std::array<double, 8> Box;

    // ---- Seed cells: half-resolution sample grids + sag-inflated AABBs ----
    auto cell_boxes = [&](const NurbsSurface& srf, double c0u, double dcu, int ncu, double c0v, double dcv, int ncv) -> std::vector<Box> {
        std::vector<std::vector<Point>> S;
        for (int i = 0; i < 2 * ncu + 1; i++) {
            std::vector<Point> row;
            for (int j = 0; j < 2 * ncv + 1; j++)
                row.push_back(srf.point_at(c0u + dcu * 0.5 * i, c0v + dcv * 0.5 * j));
            S.push_back(row);
        }
        std::vector<Box> boxes;
        for (int ci = 0; ci < ncu; ci++) {
            for (int cj = 0; cj < ncv; cj++) {
                double minx = std::numeric_limits<double>::infinity();
                double miny = minx, minz = minx;
                double maxx = -minx, maxy = -minx, maxz = -minx;
                for (int i = 2*ci; i < 2*ci + 3; i++) {
                    for (int j = 2*cj; j < 2*cj + 3; j++) {
                        const Point& p = S[i][j];
                        minx = std::min(minx, p[0]); maxx = std::max(maxx, p[0]);
                        miny = std::min(miny, p[1]); maxy = std::max(maxy, p[1]);
                        minz = std::min(minz, p[2]); maxz = std::max(maxz, p[2]);
                    }
                }
                const Point& ctr = S[2*ci + 1][2*cj + 1];
                double cx = (S[2*ci][2*cj][0] + S[2*ci+2][2*cj][0] + S[2*ci][2*cj+2][0] + S[2*ci+2][2*cj+2][0]) * 0.25;
                double cy = (S[2*ci][2*cj][1] + S[2*ci+2][2*cj][1] + S[2*ci][2*cj+2][1] + S[2*ci+2][2*cj+2][1]) * 0.25;
                double cz = (S[2*ci][2*cj][2] + S[2*ci+2][2*cj][2] + S[2*ci][2*cj+2][2] + S[2*ci+2][2*cj+2][2]) * 0.25;
                double sag = std::sqrt((ctr[0]-cx)*(ctr[0]-cx) + (ctr[1]-cy)*(ctr[1]-cy) + (ctr[2]-cz)*(ctr[2]-cz));
                double inf = 2.0 * sag + tolerance;
                boxes.push_back({minx-inf, miny-inf, minz-inf, maxx+inf, maxy+inf, maxz+inf,
                                 c0u + dcu * (ci + 0.5), c0v + dcv * (cj + 0.5)});
            }
        }
        return boxes;
    };

    std::vector<Box> boxes_a = cell_boxes(a, au0, a_du, a_nu, av0, a_dv, a_nv);
    std::vector<Box> boxes_b = cell_boxes(b, bu0, b_du, b_nu, bv0, b_dv, b_nv);

    auto cell_3d = [](const std::vector<Box>& boxes) -> double {
        double best = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < boxes.size() && i < 64; i++) {
            const Box& bx = boxes[i];
            double d = std::sqrt((bx[3]-bx[0])*(bx[3]-bx[0]) + (bx[4]-bx[1])*(bx[4]-bx[1]) + (bx[5]-bx[2])*(bx[5]-bx[2]));
            if (1e-12 < d && d < best) best = d;
        }
        return best < std::numeric_limits<double>::infinity() ? best : 1.0;
    };

    double h_init = std::min(cell_3d(boxes_a), cell_3d(boxes_b)) * 0.25;
    double conv_tol = std::max(tolerance, h_init * 1e-7);

    auto clamp_open = [&](std::array<double, 4>& x) {
        if (!a_closed_u) x[0] = std::max(au0, std::min(x[0], au1));
        if (!a_closed_v) x[1] = std::max(av0, std::min(x[1], av1));
        if (!b_closed_u) x[2] = std::max(bu0, std::min(x[2], bu1));
        if (!b_closed_v) x[3] = std::max(bv0, std::min(x[3], bv1));
    };

    // Newton on Sa(u,v) - Sb(s,t) = 0; minimum-norm or tangent-pinned
    auto correct = [&](std::array<double, 4>& x, bool has_pin, const std::array<double, 3>& pd, const std::array<double, 3>& pp) -> bool {
        for (int it = 0; it < 8; it++) {
            Vector Sa, Sau, Sav, Sb, Sbu, Sbv;
            eval_a(x[0], x[1], Sa, Sau, Sav);
            eval_b(x[2], x[3], Sb, Sbu, Sbv);
            double F[3] = {Sa[0]-Sb[0], Sa[1]-Sb[1], Sa[2]-Sb[2]};
            if (std::sqrt(F[0]*F[0] + F[1]*F[1] + F[2]*F[2]) < conv_tol) return true;
            double J[3][4];
            for (int k = 0; k < 3; k++) { J[k][0] = Sau[k]; J[k][1] = Sav[k]; J[k][2] = -Sbu[k]; J[k][3] = -Sbv[k]; }
            if (!has_pin) {
                std::vector<std::vector<double>> JJt(3, std::vector<double>(3));
                for (int r = 0; r < 3; r++)
                    for (int q = 0; q < 3; q++) {
                        double s = 0.0;
                        for (int c = 0; c < 4; c++) s += J[r][c] * J[q][c];
                        JJt[r][q] = s;
                    }
                std::vector<double> y;
                if (!solve_gauss(JJt, {F[0], F[1], F[2]}, 3, y)) return false;
                for (int c = 0; c < 4; c++) {
                    double s = 0.0;
                    for (int r = 0; r < 3; r++) s += J[r][c] * y[r];
                    x[c] -= s;
                }
            } else {
                std::vector<std::vector<double>> M = {
                    {J[0][0], J[0][1], J[0][2], J[0][3]},
                    {J[1][0], J[1][1], J[1][2], J[1][3]},
                    {J[2][0], J[2][1], J[2][2], J[2][3]},
                    {pd[0]*Sau[0]+pd[1]*Sau[1]+pd[2]*Sau[2],
                     pd[0]*Sav[0]+pd[1]*Sav[1]+pd[2]*Sav[2], 0.0, 0.0}
                };
                std::vector<double> rhs = {F[0], F[1], F[2],
                     pd[0]*(Sa[0]-pp[0]) + pd[1]*(Sa[1]-pp[1]) + pd[2]*(Sa[2]-pp[2])};
                std::vector<double> dx;
                if (!solve_gauss(M, rhs, 4, dx)) return false;
                for (int c = 0; c < 4; c++) x[c] -= dx[c];
            }
            clamp_open(x);
        }
        Vector Sa, Sau, Sav, Sb, Sbu, Sbv;
        eval_a(x[0], x[1], Sa, Sau, Sav);
        eval_b(x[2], x[3], Sb, Sbu, Sbv);
        double g = std::sqrt((Sa[0]-Sb[0])*(Sa[0]-Sb[0]) + (Sa[1]-Sb[1])*(Sa[1]-Sb[1]) + (Sa[2]-Sb[2])*(Sa[2]-Sb[2]));
        return g < conv_tol * 10.0;
    };

    // ---- Seeds from overlapping cell pairs (minimum-norm Gauss-Newton) ----
    struct Seed { double u, v, s, t; bool used; };
    std::vector<Seed> seeds;
    double seed_tol_3d = std::max(cell_3d(boxes_a), cell_3d(boxes_b));
    int pair_budget = 20000;
    std::array<double, 3> dummy3 = {0.0, 0.0, 0.0};
    for (const Box& ba : boxes_a) {
        if (pair_budget < 0) break;
        for (const Box& bb : boxes_b) {
            if (bb[0] > ba[3] || bb[3] < ba[0] || bb[1] > ba[4] || bb[4] < ba[1] || bb[2] > ba[5] || bb[5] < ba[2])
                continue;
            pair_budget -= 1;
            if (pair_budget < 0) break;
            std::array<double, 4> x = {ba[6], ba[7], bb[6], bb[7]};
            if (!correct(x, false, dummy3, dummy3)) continue;
            Vector Sa, Sau, Sav;
            eval_a(x[0], x[1], Sa, Sau, Sav);
            bool dup = false;
            for (const Seed& sd : seeds) {
                Vector So, Sou, Sov;
                eval_a(sd.u, sd.v, So, Sou, Sov);
                if (std::sqrt((Sa[0]-So[0])*(Sa[0]-So[0]) + (Sa[1]-So[1])*(Sa[1]-So[1]) + (Sa[2]-So[2])*(Sa[2]-So[2])) < seed_tol_3d) {
                    dup = true;
                    break;
                }
            }
            if (!dup)
                seeds.push_back({a_wrap_u(x[0]), a_wrap_v(x[1]), b_wrap_u(x[2]), b_wrap_v(x[3]), false});
        }
    }

    // ---- Trace each branch with predictor-corrector marching ----
    int max_steps = (a_nu * a_nv + b_nu * b_nv) * 32;
    double close_tol = h_init * 3.0;
    double consume_tol = h_init * 2.0;

    // tangent_3d: returns true with direction and (Sa, Sau, Sav, Sbu, Sbv); false at tangency
    auto tangent_3d = [&](const std::array<double, 4>& x, double dir_sign,
                          std::array<double, 3>& dir, Vector& Sa, Vector& Sau, Vector& Sav, Vector& Sbu, Vector& Sbv) -> bool {
        Vector Sb;
        eval_a(x[0], x[1], Sa, Sau, Sav);
        eval_b(x[2], x[3], Sb, Sbu, Sbv);
        double na[3] = {Sau[1]*Sav[2]-Sau[2]*Sav[1], Sau[2]*Sav[0]-Sau[0]*Sav[2], Sau[0]*Sav[1]-Sau[1]*Sav[0]};
        double nb[3] = {Sbu[1]*Sbv[2]-Sbu[2]*Sbv[1], Sbu[2]*Sbv[0]-Sbu[0]*Sbv[2], Sbu[0]*Sbv[1]-Sbu[1]*Sbv[0]};
        double d[3] = {na[1]*nb[2]-na[2]*nb[1], na[2]*nb[0]-na[0]*nb[2], na[0]*nb[1]-na[1]*nb[0]};
        double dl = std::sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
        double nal = std::sqrt(na[0]*na[0] + na[1]*na[1] + na[2]*na[2]);
        double nbl = std::sqrt(nb[0]*nb[0] + nb[1]*nb[1] + nb[2]*nb[2]);
        if (dl < 1e-4 * nal * nbl || dl < 1e-30) return false;
        dir = {d[0]/dl*dir_sign, d[1]/dl*dir_sign, d[2]/dl*dir_sign};
        return true;
    };

    // trace_dir: forward/backward march from x0; returns (out, closed)
    auto trace_dir = [&](const std::array<double, 4>& x0, double dir_sign,
                         std::vector<std::array<double, 4>>& out) -> bool {
        out.clear();
        std::array<double, 4> x = x0;
        bool have_prev_d = false;
        std::array<double, 3> prev_d = {0.0, 0.0, 0.0};
        Vector Sa0, Sa0u, Sa0v;
        eval_a(x[0], x[1], Sa0, Sa0u, Sa0v);
        std::array<double, 3> p_start = {Sa0[0], Sa0[1], Sa0[2]};
        std::array<double, 3> p_prev = p_start;
        double dist_traveled = 0.0;
        double h = h_init;
        int smooth = 0;
        for (int step_i = 0; step_i < max_steps; step_i++) {
            std::array<double, 3> d;
            Vector Sa, Sau, Sav, Sbu, Sbv;
            if (!tangent_3d(x, dir_sign, d, Sa, Sau, Sav, Sbu, Sbv)) break;
            bool accepted = false;
            int attempts = 0;
            std::array<double, 4> xn = {0,0,0,0};
            std::array<double, 3> p_cur = {0,0,0};
            double step_len = 0.0;
            bool hit_boundary = false;
            while (attempts < 7 && !accepted) {
                std::vector<std::vector<double>> Ma = {
                    {Sau[0]*Sau[0]+Sau[1]*Sau[1]+Sau[2]*Sau[2], Sau[0]*Sav[0]+Sau[1]*Sav[1]+Sau[2]*Sav[2]},
                    {Sau[0]*Sav[0]+Sau[1]*Sav[1]+Sau[2]*Sav[2], Sav[0]*Sav[0]+Sav[1]*Sav[1]+Sav[2]*Sav[2]}};
                std::vector<double> ra = {h*(d[0]*Sau[0]+d[1]*Sau[1]+d[2]*Sau[2]), h*(d[0]*Sav[0]+d[1]*Sav[1]+d[2]*Sav[2])};
                std::vector<std::vector<double>> Mb = {
                    {Sbu[0]*Sbu[0]+Sbu[1]*Sbu[1]+Sbu[2]*Sbu[2], Sbu[0]*Sbv[0]+Sbu[1]*Sbv[1]+Sbu[2]*Sbv[2]},
                    {Sbu[0]*Sbv[0]+Sbu[1]*Sbv[1]+Sbu[2]*Sbv[2], Sbv[0]*Sbv[0]+Sbv[1]*Sbv[1]+Sbv[2]*Sbv[2]}};
                std::vector<double> rb = {h*(d[0]*Sbu[0]+d[1]*Sbu[1]+d[2]*Sbu[2]), h*(d[0]*Sbv[0]+d[1]*Sbv[1]+d[2]*Sbv[2])};
                std::vector<double> duv_a, duv_b;
                if (!solve_gauss(Ma, ra, 2, duv_a) || !solve_gauss(Mb, rb, 2, duv_b)) return false;
                double delta[4] = {duv_a[0], duv_a[1], duv_b[0], duv_b[1]};
                double tc = 1.0;
                hit_boundary = false;
                struct Ax { int idx; double lo, hi; bool closed; };
                Ax axs[4] = {{0, au0, au1, a_closed_u}, {1, av0, av1, a_closed_v},
                             {2, bu0, bu1, b_closed_u}, {3, bv0, bv1, b_closed_v}};
                for (const Ax& ax : axs) {
                    if (ax.closed || std::abs(delta[ax.idx]) < 1e-15) continue;
                    if (x[ax.idx] + delta[ax.idx] > ax.hi) {
                        tc = std::min(tc, (ax.hi - x[ax.idx]) / delta[ax.idx]);
                        hit_boundary = true;
                    }
                    if (x[ax.idx] + delta[ax.idx] < ax.lo) {
                        tc = std::min(tc, (ax.lo - x[ax.idx]) / delta[ax.idx]);
                        hit_boundary = true;
                    }
                }
                for (int k = 0; k < 4; k++) xn[k] = x[k] + tc * delta[k];
                std::array<double, 3> p_pred = {Sa[0] + d[0]*h*tc, Sa[1] + d[1]*h*tc, Sa[2] + d[2]*h*tc};
                if (!correct(xn, true, d, p_pred)) return false;
                Vector San, Sanu, Sanv;
                eval_a(xn[0], xn[1], San, Sanu, Sanv);
                p_cur = {San[0], San[1], San[2]};
                step_len = std::sqrt((p_cur[0]-p_prev[0])*(p_cur[0]-p_prev[0]) + (p_cur[1]-p_prev[1])*(p_cur[1]-p_prev[1]) + (p_cur[2]-p_prev[2])*(p_cur[2]-p_prev[2]));
                if (have_prev_d && step_len > 1e-14) {
                    double sd0 = (p_cur[0]-p_prev[0])/step_len, sd1 = (p_cur[1]-p_prev[1])/step_len, sd2 = (p_cur[2]-p_prev[2])/step_len;
                    double ddot = sd0*prev_d[0] + sd1*prev_d[1] + sd2*prev_d[2];
                    if (ddot < 0.985 && attempts < 6 && !hit_boundary) {
                        h *= 0.5;
                        attempts += 1;
                        smooth = 0;
                        continue;
                    }
                }
                accepted = true;
            }
            if (!accepted) break;
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
                std::sqrt((p_cur[0]-p_start[0])*(p_cur[0]-p_start[0]) + (p_cur[1]-p_start[1])*(p_cur[1]-p_start[1]) + (p_cur[2]-p_start[2])*(p_cur[2]-p_start[2])) < close_tol) {
                out.push_back(x);
                return true;
            }
            out.push_back(x);
            p_prev = p_cur;
            if (hit_boundary) break;
            for (Seed& sd : seeds) {
                if (!sd.used) {
                    Vector So, Sou, Sov;
                    eval_a(sd.u, sd.v, So, Sou, Sov);
                    if (std::sqrt((p_cur[0]-So[0])*(p_cur[0]-So[0]) + (p_cur[1]-So[1])*(p_cur[1]-So[1]) + (p_cur[2]-So[2])*(p_cur[2]-So[2])) < consume_tol)
                        sd.used = true;
                }
            }
        }
        return false;
    };

    struct Axis { int idx; double c0, rng; bool closed; };
    Axis axes[4] = {{0, au0, a_range_u, a_closed_u}, {1, av0, a_range_v, a_closed_v},
                    {2, bu0, b_range_u, b_closed_u}, {3, bv0, b_range_v, b_closed_v}};

    auto eval3_q = [&](const std::array<double, 4>& q) -> std::array<double, 3> {
        Vector Sa, Sau, Sav;
        eval_a(q[0], q[1], Sa, Sau, Sav);
        return {Sa[0], Sa[1], Sa[2]};
    };

    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> result;
    std::vector<std::vector<std::array<double, 3>>> kept_pts3;
    for (Seed& seed : seeds) {
        if (seed.used) continue;
        seed.used = true;
        std::array<double, 4> x0 = {seed.u, seed.v, seed.s, seed.t};
        if (!correct(x0, false, dummy3, dummy3)) continue;
        std::vector<std::array<double, 4>> fwd, bwd;
        bool fwd_closed = trace_dir(x0, +1, fwd);
        if (!fwd_closed) trace_dir(x0, -1, bwd);

        std::vector<std::array<double, 4>> quad;
        for (int i = (int)bwd.size() - 1; i >= 0; i--) quad.push_back(bwd[i]);
        quad.push_back(x0);
        for (auto& p : fwd) quad.push_back(p);
        if ((int)quad.size() < 4) continue;

        // Unwrap all four parameters along the trace
        for (size_t i = 1; i < quad.size(); i++) {
            for (const Axis& ax : axes) {
                if (!ax.closed) continue;
                double jump = quad[i][ax.idx] - quad[i-1][ax.idx];
                if (jump > ax.rng * 0.5) quad[i][ax.idx] -= ax.rng;
                else if (jump < -ax.rng * 0.5) quad[i][ax.idx] += ax.rng;
            }
        }

        std::array<double, 3> p_first = eval3_q(quad.front());
        std::array<double, 3> p_last = eval3_q(quad.back());
        double gap2 = std::sqrt((p_first[0]-p_last[0])*(p_first[0]-p_last[0]) + (p_first[1]-p_last[1])*(p_first[1]-p_last[1]) + (p_first[2]-p_last[2])*(p_first[2]-p_last[2]));
        bool is_loop = fwd_closed || ((int)quad.size() >= 6 && gap2 < close_tol);
        if (is_loop) quad.pop_back();
        if ((int)quad.size() < 4) continue;

        // Trace-level dedup against already kept traces
        int m = (int)quad.size();
        std::vector<std::array<double, 3>> trace_pts3(m);
        for (int i = 0; i < m; i++) trace_pts3[i] = eval3_q(quad[i]);
        // A trace duplicates a kept one only if it CLOSELY FOLLOWS it. The tolerance must be tight
        // relative to the spacing between DISTINCT intersection branches: e.g. two perpendicular
        // cylinders (Steinmetz) have 4 arcs only ~1.4 apart at the 0.25/0.75 samples, so the old
        // h_init*6 (~2.3) wrongly merged 3 of the 4 arcs into one. Use h_init*2 and scan EVERY kept
        // point (a true duplicate lies within ~1 marching step everywhere; distinct branches do not).
        double dup_tol = h_init * 2.0;
        bool dup = false;
        for (auto& other : kept_pts3) {
            bool all_close = true;
            for (double f : {0.25, 0.5, 0.75}) {
                std::array<double, 3> cp = trace_pts3[(int)((m - 1) * f)];
                double dmin = dup_tol + 1.0;
                for (size_t k = 0; k < other.size(); k += 1) {
                    const std::array<double, 3>& op = other[k];
                    dmin = std::min(dmin, std::sqrt((cp[0]-op[0])*(cp[0]-op[0]) + (cp[1]-op[1])*(cp[1]-op[1]) + (cp[2]-op[2])*(cp[2]-op[2])));
                }
                if (dmin > dup_tol) { all_close = false; break; }
            }
            if (all_close) { dup = true; break; }
        }
        if (dup) continue;
        kept_pts3.push_back(trace_pts3);

        // Densify: fill large 3D gaps (grown steps / fwd-bwd junction) with
        // Newton-corrected midpoints so per-piece interpolation reaches 1e-6.
        auto gap3 = [&](const std::array<double, 4>& qi, const std::array<double, 4>& qj) -> double {
            std::array<double, 3> pi = eval3_q(qi);
            std::array<double, 3> pj = eval3_q(qj);
            return std::sqrt((pi[0]-pj[0])*(pi[0]-pj[0]) + (pi[1]-pj[1])*(pi[1]-pj[1]) + (pi[2]-pj[2])*(pi[2]-pj[2]));
        };
        for (int gp = 0; gp < 4; gp++) {
            std::vector<double> gg;
            for (size_t i = 0; i + 1 < quad.size(); i++) gg.push_back(gap3(quad[i], quad[i+1]));
            if (gg.empty()) break;
            std::vector<double> sorted_gg = gg;
            std::sort(sorted_gg.begin(), sorted_gg.end());
            double med = sorted_gg[sorted_gg.size()/2];
            if (med <= 0) break;
            bool changed = false;
            size_t i = 0;
            while (i + 1 < quad.size() && quad.size() < 4000) {
                if (gap3(quad[i], quad[i+1]) > 1.5*med) {
                    std::array<double, 4> midq;
                    for (int k = 0; k < 4; k++) midq[k] = (quad[i][k]+quad[i+1][k])*0.5;
                    if (correct(midq, false, dummy3, dummy3)) {
                        quad.insert(quad.begin() + i + 1, midq);
                        changed = true;
                        i += 2;
                        continue;
                    }
                }
                i++;
            }
            if (!changed) break;
        }

        // Closed-loop virtual closure point across all four parameters
        std::array<double, 4> closure = {0.0, 0.0, 0.0, 0.0};
        if (is_loop && quad.size() >= 2) {
            std::array<double, 4> virt = quad[0];
            for (const Axis& ax : axes) {
                double jump = quad[0][ax.idx] - quad.back()[ax.idx];
                if (ax.closed) {
                    while (jump > ax.rng * 0.5) jump -= ax.rng;
                    while (jump < -ax.rng * 0.5) jump += ax.rng;
                }
                virt[ax.idx] = quad.back()[ax.idx] + jump;
                closure[ax.idx] = virt[ax.idx] - quad[0][ax.idx];
            }
            quad.push_back(virt);
        }

        // Insert seam crossings on any closed parameter of either surface
        std::vector<std::array<double, 4>> out_pts;
        out_pts.push_back(quad[0]);
        std::vector<int> cross_idx;
        for (size_t i = 1; i < quad.size(); i++) {
            const std::array<double, 4>& pa_ = quad[i - 1];
            const std::array<double, 4>& pb_ = quad[i];
            std::vector<std::tuple<double, int, double>> crossings;
            for (const Axis& ax : axes) {
                if (!ax.closed || std::abs(pb_[ax.idx] - pa_[ax.idx]) <= 1e-15) continue;
                int k0 = (int)std::floor((pa_[ax.idx] - ax.c0) / ax.rng);
                int k1 = (int)std::floor((pb_[ax.idx] - ax.c0) / ax.rng);
                for (int k = std::min(k0, k1) + 1; k <= std::max(k0, k1); k++) {
                    double L = ax.c0 + k * ax.rng;
                    double t = (L - pa_[ax.idx]) / (pb_[ax.idx] - pa_[ax.idx]);
                    if (0.0 < t && t < 1.0) crossings.push_back({t, ax.idx, L});
                }
            }
            std::sort(crossings.begin(), crossings.end());
            for (const auto& [t, idx, L] : crossings) {
                std::array<double, 4> cp;
                for (int k = 0; k < 4; k++) cp[k] = pa_[k] + (pb_[k] - pa_[k]) * t;
                cp[idx] = L;
                // The crossing was linearly interpolated; Newton-correct it onto
                // both surfaces so the piece boundary is accurate (1e-6).
                correct(cp, false, dummy3, dummy3);
                out_pts.push_back(cp);
                cross_idx.push_back((int)out_pts.size() - 1);
            }
            out_pts.push_back(pb_);
            if (i < quad.size() - 1) {
                bool on_seam = false;
                for (const Axis& ax : axes) {
                    if (!ax.closed) continue;
                    double k = std::round((pb_[ax.idx] - ax.c0) / ax.rng);
                    double L = ax.c0 + k * ax.rng;
                    if (std::abs(pb_[ax.idx] - L) < ax.rng * 1e-9 && std::abs(pb_[ax.idx] - pa_[ax.idx]) > ax.rng * 1e-9) {
                        out_pts.back()[ax.idx] = L;
                        on_seam = true;
                    }
                }
                if (on_seam) cross_idx.push_back((int)out_pts.size() - 1);
            }
        }

        bool wrap_drift = false;
        for (const Axis& ax : axes)
            if (std::abs(closure[ax.idx]) > ax.rng * 0.5) wrap_drift = true;

        std::vector<std::pair<std::vector<std::array<double, 4>>, bool>> pieces;
        if (cross_idx.size() == 0) {
            pieces.push_back({out_pts, is_loop && !wrap_drift});
        } else if (is_loop) {
            for (size_t ci = 0; ci + 1 < cross_idx.size(); ci++) {
                int ia = cross_idx[ci];
                int ib = cross_idx[ci + 1];
                pieces.push_back({std::vector<std::array<double, 4>>(out_pts.begin() + ia, out_pts.begin() + ib + 1), false});
            }
            std::vector<std::array<double, 4>> wrap_piece(out_pts.begin() + cross_idx.back(), out_pts.end());
            for (int pi = 1; pi <= cross_idx[0]; pi++) {
                std::array<double, 4> p;
                for (int k = 0; k < 4; k++) p[k] = out_pts[pi][k] + closure[k];
                wrap_piece.push_back(p);
            }
            pieces.push_back({wrap_piece, false});
        } else {
            std::vector<int> bounds;
            bounds.push_back(0);
            for (int ci : cross_idx) bounds.push_back(ci);
            bounds.push_back((int)out_pts.size() - 1);
            for (size_t bi = 0; bi + 1 < bounds.size(); bi++) {
                int ia = bounds[bi];
                int ib = bounds[bi + 1];
                if (ib > ia)
                    pieces.push_back({std::vector<std::array<double, 4>>(out_pts.begin() + ia, out_pts.begin() + ib + 1), false});
            }
        }

        for (auto& [piece_pts, piece_loop] : pieces) {
            if (piece_pts.size() < 2) continue;
            std::array<double, 4> mid = piece_pts[piece_pts.size() / 2];
            for (const Axis& ax : axes) {
                if (!ax.closed) continue;
                int k_s = (int)std::floor((mid[ax.idx] - ax.c0) / ax.rng);
                if (k_s != 0)
                    for (auto& p : piece_pts) p[ax.idx] -= k_s * ax.rng;
            }

            std::vector<std::array<double, 3>> pts3(piece_pts.size());
            for (size_t i = 0; i < piece_pts.size(); i++) pts3[i] = eval3_q(piece_pts[i]);
            double chord3 = 0.0;
            for (size_t i = 1; i < pts3.size(); i++)
                chord3 += std::sqrt((pts3[i][0]-pts3[i-1][0])*(pts3[i][0]-pts3[i-1][0]) + (pts3[i][1]-pts3[i-1][1])*(pts3[i][1]-pts3[i-1][1]) + (pts3[i][2]-pts3[i-1][2])*(pts3[i][2]-pts3[i-1][2]));
            // Degenerate sliver pieces between near-coincident crossings
            if (chord3 < h_init * 0.5) continue;

            // Deflection-refine this piece: insert Newton-corrected midpoints
            // wherever the 3D curve deviates from its chord by more than the
            // target, so the per-piece interpolation reaches 1e-6 even in
            // high-curvature regions (the global gap-fill misses locally-curved
            // pieces because it uses the whole-curve median spacing).
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
                    for (int k = 0; k < 4; k++) midq[k] = (pa2[k] + pb2[k]) * 0.5;
                    if (correct(midq, false, dummy3, dummy3)) {
                        std::array<double, 3> p3m = eval3_q(midq);
                        double ex = p3b[0]-p3a[0], ey = p3b[1]-p3a[1], ez = p3b[2]-p3a[2];
                        double l2 = ex*ex + ey*ey + ez*ez;
                        double dev;
                        if (l2 > 1e-30) {
                            double tt = ((p3m[0]-p3a[0])*ex + (p3m[1]-p3a[1])*ey + (p3m[2]-p3a[2])*ez) / l2;
                            double cxp = p3a[0]+tt*ex, cyp = p3a[1]+tt*ey, czp = p3a[2]+tt*ez;
                            dev = std::sqrt((p3m[0]-cxp)*(p3m[0]-cxp) + (p3m[1]-cyp)*(p3m[1]-cyp) + (p3m[2]-czp)*(p3m[2]-czp));
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
                if (!refined) break;
            }
            pts3.assign(piece_pts.size(), std::array<double, 3>{});
            for (size_t i = 0; i < piece_pts.size(); i++) pts3[i] = eval3_q(piece_pts[i]);

            bool ploop = piece_loop;
            auto fit_track = [&](const std::vector<Point>& pts2, double fit_tol_track) -> NurbsCurve {
                int mp = (int)pts2.size();
                double total_turning = 0.0;
                for (int i = 1; i < mp - 1; i++) {
                    double dx1 = pts2[i][0] - pts2[i-1][0];
                    double dy1 = pts2[i][1] - pts2[i-1][1];
                    double dz1 = pts2[i][2] - pts2[i-1][2];
                    double dx2 = pts2[i+1][0] - pts2[i][0];
                    double dy2 = pts2[i+1][1] - pts2[i][1];
                    double dz2 = pts2[i+1][2] - pts2[i][2];
                    double l1 = std::sqrt(dx1*dx1 + dy1*dy1 + dz1*dz1);
                    double l2 = std::sqrt(dx2*dx2 + dy2*dy2 + dz2*dz2);
                    if (l1 > 1e-14 && l2 > 1e-14) {
                        double c = (dx1*dx2 + dy1*dy2 + dz1*dz2) / (l1*l2);
                        c = std::max(-1.0, std::min(1.0, c));
                        total_turning += std::acos(c);
                    }
                }
                std::vector<double> chords(mp, 0.0);
                double total_len = 0.0;
                for (int i = 1; i < mp; i++) {
                    total_len += pts2[i].distance(pts2[i-1]);
                    chords[i] = total_len;
                }
                if (ploop && mp > 1) total_len += pts2[0].distance(pts2[mp-1]);
                if (total_len > 1e-14)
                    for (int i = 1; i < mp; i++) chords[i] /= total_len;
                // Compact least-squares first (keep best valid); if it cannot
                // reach the tolerance, interpolate EXACTLY through the dense,
                // high-precision (on-surface) samples to reach 1e-6.
                int target_cvs = std::max(8, (int)(total_turning / 0.5) + 6);
                int max_cvs = std::max(8, std::min(mp - 1, mp / 3));
                NurbsCurve best;
                double best_dev = std::numeric_limits<double>::infinity();
                while (target_cvs <= max_cvs) {
                    NurbsCurve crv = NurbsCurve::create_fitted(pts2, target_cvs, 3, ploop);
                    if (!crv.is_valid()) break;
                    auto [ft0, ft1] = crv.domain();
                    double dev = 0.0;
                    for (int i = 0; i < mp; i++) {
                        double t = ft0 + (ft1 - ft0) * chords[i];
                        dev = std::max(dev, crv.point_at(t).distance(pts2[i]));
                    }
                    if (dev < best_dev) { best = crv; best_dev = dev; }
                    if (dev < fit_tol_track) break;
                    target_cvs *= 2;
                }
                if (best_dev >= fit_tol_track) {
                    NurbsCurve interp = ploop
                        ? NurbsCurve::create_interpolated(pts2, CurveNurbsKnotStyle::ChordPeriodic)
                        : NurbsCurve::create_interpolated(pts2);
                    if (interp.is_valid()) best = interp;
                }
                if (best.is_valid()) best.set_domain(0.0, 1.0);
                return best;
            };

            std::vector<Point> pts3_p(pts3.size());
            for (size_t i = 0; i < pts3.size(); i++) pts3_p[i] = Point(pts3[i][0], pts3[i][1], pts3[i][2]);
            std::vector<Point> pts_pa(piece_pts.size());
            std::vector<Point> pts_pb(piece_pts.size());
            for (size_t i = 0; i < piece_pts.size(); i++) {
                pts_pa[i] = Point(piece_pts[i][0], piece_pts[i][1], 0.0);
                pts_pb[i] = Point(piece_pts[i][2], piece_pts[i][3], 0.0);
            }
            NurbsCurve crv3 = fit_track(pts3_p, std::max(tolerance * 10.0, 1e-7));
            NurbsCurve pcurve_a = fit_track(pts_pa, std::min(a_du, a_dv) * 1e-4);
            NurbsCurve pcurve_b = fit_track(pts_pb, std::min(b_du, b_dv) * 1e-4);
            if (!crv3.is_valid() || !pcurve_a.is_valid() || !pcurve_b.is_valid()) continue;
            result.push_back({std::move(crv3), std::move(pcurve_a), std::move(pcurve_b)});
        }
    }

    return result;
}

namespace {
// Keep only the sub-segments of a UV pcurve on `target` whose lifted 3D point
// lies within the (bounded) cutter footprint (closest-point gap ~ 0).
std::vector<NurbsCurve> clip_pcurve_to_cutter(const NurbsSurface& target, const NurbsCurve& pc, const NurbsSurface& cutter) {
    int n = std::max(pc.cv_count() * 4, 16);
    auto dc = pc.domain();
    double d0 = dc.first, d1 = dc.second;
    auto cu = cutter.domain(0);
    auto cv = cutter.domain(1);
    double corner_diag = cutter.point_at(cu.first, cv.first).distance(cutter.point_at(cu.second, cv.second));
    // Tolerance for "this pcurve point lifts onto the (finite) cutter". It must exceed the
    // pull-back fit error of a sampled pcurve on a curved target (e.g. a projected sphere/cone
    // circle deviates ~1e-3 of the surface size from the cutter plane), otherwise a curve that
    // lies entirely on the cutter gets spuriously chopped into many tiny arcs. Parts genuinely
    // off a finite cutter are O(size) away, so a generous relative tolerance still rejects them.
    double on_tol = std::max(1e-6, corner_diag * 2e-2);

    // clip_pcurve_to_cutter is only ever called for a PLANAR cutter (the caller guards on
    // cutter_planar), so the closest-point gap to the finite cutter face is the analytic
    // point-to-rectangle distance: project p3 into the face's (eu,ev) frame, clamp the
    // parameters to the face rect, measure the 3D residual. This replaces a per-sample grid
    // search (Closest::surface_point) that dominated SSI time (~66%), with an O(1) projection.
    Point q00 = cutter.point_at(cu.first, cv.first);
    Point q10 = cutter.point_at(cu.second, cv.first);
    Point q01 = cutter.point_at(cu.first, cv.second);
    Vector eu(q10[0]-q00[0], q10[1]-q00[1], q10[2]-q00[2]);
    Vector ev(q01[0]-q00[0], q01[1]-q00[1], q01[2]-q00[2]);
    double eu2 = eu[0]*eu[0]+eu[1]*eu[1]+eu[2]*eu[2];
    double ev2 = ev[0]*ev[0]+ev[1]*ev[1]+ev[2]*ev[2];
    bool fast_planar = (eu2 > 1e-28 && ev2 > 1e-28);
    auto gap = [&](double t) -> double {
        Point uv = pc.point_at(t);
        Point p3 = target.point_at(uv[0], uv[1]);
        if (fast_planar) {
            double dx = p3[0]-q00[0], dy = p3[1]-q00[1], dz = p3[2]-q00[2];
            double a = (dx*eu[0]+dy*eu[1]+dz*eu[2]) / eu2;
            double b = (dx*ev[0]+dy*ev[1]+dz*ev[2]) / ev2;
            a = std::min(std::max(a, 0.0), 1.0);
            b = std::min(std::max(b, 0.0), 1.0);
            double cx = q00[0]+a*eu[0]+b*ev[0], cy = q00[1]+a*eu[1]+b*ev[1], cz = q00[2]+a*eu[2]+b*ev[2];
            return std::sqrt((p3[0]-cx)*(p3[0]-cx)+(p3[1]-cy)*(p3[1]-cy)+(p3[2]-cz)*(p3[2]-cz));
        }
        return std::get<2>(Closest::surface_point(cutter, p3, 0.0, 0.0, 0.0, 0.0));
    };
    auto refine = [&](double t_in, double t_out) -> double {
        double a = t_in, b = t_out;
        for (int k = 0; k < 20; ++k) {
            double tm = (a + b) * 0.5;
            if (gap(tm) < on_tol) a = tm; else b = tm;
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
    int i = 0;
    while (i <= n) {
        if (flags[i].second) {
            int j = i;
            while (j + 1 <= n && flags[j + 1].second) ++j;
            double ta = (i == 0) ? flags[i].first : refine(flags[i].first, flags[i - 1].first);
            double tb = (j == n) ? flags[j].first : refine(flags[j].first, flags[j + 1].first);
            if (tb - ta > (d1 - d0) * 1e-6) {
                NurbsCurve piece = pc;
                if (piece.trim(ta, tb) && piece.is_valid()) pieces.push_back(piece);
            }
            i = j + 1;
        } else {
            ++i;
        }
    }
    return pieces;
}
}  // namespace

std::vector<NurbsCurve> Intersection::cut_curves_on_surface(const NurbsSurface& target, const NurbsSurface& cutter, double tolerance) {
    std::vector<NurbsCurve> out;
    // Route through surface_surface so the closed-form analytic dispatch (plane/cylinder/
    // sphere/cone/torus) is used when the pair is recognized -- exact AND fast. Marching is
    // the fallback only for unrecognized freeform pairs. The intersection pcurve on `target`
    // is then clipped to the (finite) cutter's extent, matching the previous behaviour.
    bool cutter_planar = cutter.is_planar(nullptr, 1e-6);
    // Pull the exact 3D intersection back onto the TARGET, keeping ALL pieces. surface_surface
    // bundles only one pcurve per 3D curve (pas[0]); on a periodic target (sphere/cone/torus) a
    // circle pulls back to SEVERAL seam arcs and the others would be lost. Use the analytic pcurve
    // when the target is a plane/cylinder (exact, single piece), else project for every seam arc.
    double rtol = std::max(tolerance, 1e-7) * 1e4;
    RecogSurface rt = recognize_surface(target, rtol);
    for (auto& tr : surface_surface(target, cutter, tolerance)) {
        const NurbsCurve& c3d = std::get<0>(tr);
        std::vector<NurbsCurve> pcs;
        NurbsCurve pa_an = analytic_pcurve(target, rt, c3d);
        if (pa_an.is_valid()) {
            pcs.push_back(pa_an);
        } else if (rt.kind == RecogSurface::SPHERE) {
            // OCCT-style analytic per-point inverse (atan2 longitude) -> exact seam crossings.
            pcs = analytic_sphere_pullback(target, rt, c3d);
            if (pcs.empty()) pcs = Closest::surface_curve(target, c3d, 0.0, 0.0, tolerance);
            if (pcs.empty()) pcs.push_back(std::get<1>(tr));
        } else if (rt.kind == RecogSurface::CONE) {
            // Cone is a surface of revolution: longitude via atan2 (exact seam), v linear in height.
            pcs = analytic_cone_pullback(target, rt, c3d);
            if (pcs.empty()) pcs = Closest::surface_curve(target, c3d, 0.0, 0.0, tolerance);
            if (pcs.empty()) pcs.push_back(std::get<1>(tr));
        } else {
            pcs = Closest::surface_curve(target, c3d, 0.0, 0.0, tolerance);
            if (pcs.empty()) pcs.push_back(std::get<1>(tr));
        }
        for (const auto& pc : pcs) {
            if (cutter_planar) {
                auto clipped = clip_pcurve_to_cutter(target, pc, cutter);
                out.insert(out.end(), clipped.begin(), clipped.end());
            } else {
                out.push_back(pc);
            }
        }
    }
    return out;
}

// ── Joint geometry utilities ─────────────────────────────────────────────────

static bool vectors_nearly_parallel(const Vector& v0, const Vector& v1, double angle_tol) {
    double m0 = v0.magnitude();
    double m1 = v1.magnitude();
    if (m0 < Tolerance::ZERO_TOLERANCE || m1 < Tolerance::ZERO_TOLERANCE) return false;
    double cos_angle = std::fabs(v0.dot(v1) / (m0 * m1));
    return cos_angle >= std::cos(angle_tol);
}

bool Intersection::plane_plane_plane_check(const Plane& p0, const Plane& p1, const Plane& p2, double angle_tol, Point& output) {
    if (vectors_nearly_parallel(p0.z_axis(), p1.z_axis(), angle_tol)) return false;
    if (vectors_nearly_parallel(p0.z_axis(), p2.z_axis(), angle_tol)) return false;
    if (vectors_nearly_parallel(p1.z_axis(), p2.z_axis(), angle_tol)) return false;
    return plane_plane_plane(p0, p1, p2, output);
}

double Intersection::remap(double val, double from1, double to1, double from2, double to2) {
    double span = to1 - from1;
    if (std::fabs(span) < Tolerance::ZERO_TOLERANCE) return from2;
    double t = (val - from1) / span;
    return from2 + t * (to2 - from2);
}

bool Intersection::closest_point_on_segment(const Point& pt, const Line& seg, Point& output, double& t) {
    Point start = seg.start();
    Point end = seg.end();
    double dx = end[0] - start[0];
    double dy = end[1] - start[1];
    double dz = end[2] - start[2];
    double len_sq = dx*dx + dy*dy + dz*dz;
    if (len_sq < 1e-20) {
        output = start;
        t = 0.0;
        return true;
    }
    double vx = pt[0] - start[0];
    double vy = pt[1] - start[1];
    double vz = pt[2] - start[2];
    t = (vx*dx + vy*dy + vz*dz) / len_sq;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    output = Point(start[0] + t*dx, start[1] + t*dy, start[2] + t*dz);
    return true;
}

bool Intersection::plane_4planes(const Plane& main_plane, const std::array<Plane, 4>& planes, Polyline& output) {
    Point p0, p1, p2, p3;
    if (!plane_plane_plane_check(planes[0], planes[1], main_plane, 0.1, p0)) return false;
    if (!plane_plane_plane_check(planes[1], planes[2], main_plane, 0.1, p1)) return false;
    if (!plane_plane_plane_check(planes[2], planes[3], main_plane, 0.1, p2)) return false;
    if (!plane_plane_plane_check(planes[3], planes[0], main_plane, 0.1, p3)) return false;
    output = Polyline(std::vector<Point>{
        p0,
        p1,
        p2,
        p3,
        p0,
    });
    return true;
}

bool Intersection::plane_4planes_open(const Plane& main_plane, const std::array<Plane, 4>& planes, Polyline& output) {
    // Two-step formulation with a stable closed-form plane-plane → line
    // step, then line-plane → point. Each corner pair (A,B):
    //   1. line direction  = nA × nB
    //   2. line anchor     = (dA (nB × v) + dB (v × nA)) / |v|²  (closest to origin)
    //   3. corner          = intersect(line, main_plane)
    // This replaces the 3-plane determinant (plane_plane_plane_check)
    // which loses precision under steep fold angles — source of
    // vidy_folding's 2-40 mm jv[0] deviations when two side planes are
    // close to parallel or meet at a grazing angle. `plane_plane` itself
    // is kept on the original midpoint-bisector formulation to preserve
    // its published `start`/`end` anchor for existing callers.
    auto corner = [&](const Plane& a, const Plane& b, Point& out) -> bool {
        Vector nA = a.z_axis();
        nA.normalize_self();
        Vector nB = b.z_axis();
        nB.normalize_self();
        Vector v = nA.cross(nB);
        double v_sq = v.magnitude_squared();
        if (v_sq < 1e-24) return false;
        double dA = nA[0]*a.origin()[0] + nA[1]*a.origin()[1] + nA[2]*a.origin()[2];
        double dB = nB[0]*b.origin()[0] + nB[1]*b.origin()[1] + nB[2]*b.origin()[2];
        Vector nB_x_v = nB.cross(v);
        Vector v_x_nA = v.cross(nA);
        double inv = 1.0 / v_sq;
        Point anchor(
            (dA * nB_x_v[0] + dB * v_x_nA[0]) * inv,
            (dA * nB_x_v[1] + dB * v_x_nA[1]) * inv,
            (dA * nB_x_v[2] + dB * v_x_nA[2]) * inv
        );
        Line l = Line::from_points(anchor, Point(anchor[0]+v[0], anchor[1]+v[1], anchor[2]+v[2]));
        return line_plane(l, main_plane, out, false);
    };
    Point p0, p1, p2, p3;
    if (!corner(planes[0], planes[1], p0)) return false;
    if (!corner(planes[1], planes[2], p1)) return false;
    if (!corner(planes[2], planes[3], p2)) return false;
    if (!corner(planes[3], planes[0], p3)) return false;
    output = Polyline(std::vector<Point>{
        p0, p1, p2, p3,
    });
    return true;
}

bool Intersection::plane_4lines(const Plane& plane, const Line& l0, const Line& l1, const Line& l2, const Line& l3, Polyline& output) {
    Point p0, p1, p2, p3;
    if (!line_plane(l0, plane, p0, false)) return false;
    if (!line_plane(l1, plane, p1, false)) return false;
    if (!line_plane(l2, plane, p2, false)) return false;
    if (!line_plane(l3, plane, p3, false)) return false;
    output = Polyline(std::vector<Point>{
        p0,
        p1,
        p2,
        p3,
        p0,
    });
    return true;
}

bool Intersection::line_two_planes(const Line& line, const Plane& plane0, const Plane& plane1, Line& output) {
    Point q0, q1;
    if (!line_plane(line, plane0, q0, true)) return false;
    if (!line_plane(line, plane1, q1, true)) return false;
    output = Line(q0[0], q0[1], q0[2], q1[0], q1[1], q1[2]);
    return true;
}

bool Intersection::polyline_plane(const Polyline& polyline, const Plane& plane, std::vector<Point>& points, std::vector<int>& edge_ids) {
    size_t n = polyline.point_count();
    if (n < 2) return false;
    for (size_t i = 0; i < n - 1; i++) {
        Point a = polyline.get_point(i);
        Point b = polyline.get_point(i + 1);
        double va = plane_value_at(plane, a);
        double vb = plane_value_at(plane, b);
        if (std::fabs(va) < Tolerance::ZERO_TOLERANCE || std::fabs(vb) < Tolerance::ZERO_TOLERANCE)
            continue;
        Line seg(a[0], a[1], a[2], b[0], b[1], b[2]);
        Point hit;
        if (line_plane(seg, plane, hit, true)) {
            points.push_back(hit);
            edge_ids.push_back(static_cast<int>(i));
        }
    }
    return !points.empty();
}

// Note: polyline_plane_cross (static), g_cross_distance_squared, and
// Intersection::set_cross_joint_distance_squared moved to
// wood/wood_joint_detection.cpp (wood-domain cross-joint detection).

bool Intersection::line_line_3d(const Line& cutter, const Line& seg, Point& output) {
    double t0, t1;
    if (!line_line_parameters(cutter, seg, t0, t1, 0.0, false, false)) return false;
    output = cutter.point_at(t0);
    return true;
}

bool Intersection::scale_vector_to_distance_of_2planes(const Vector& direction, const Plane& plane0, const Plane& plane1, Vector& output) {
    if (direction.magnitude() < Tolerance::ZERO_TOLERANCE) return false;
    Line ray(0.0, 0.0, 0.0, direction[0], direction[1], direction[2]);
    Point q0, q1;
    if (!line_plane(ray, plane0, q0, false)) return false;
    if (!line_plane(ray, plane1, q1, false)) return false;
    output = Vector(q1[0]-q0[0], q1[1]-q0[1], q1[2]-q0[2]);
    Vector n1 = plane1.z_axis();
    double n1_mag = n1.magnitude();
    if (n1_mag < Tolerance::ZERO_TOLERANCE) return false;
    Point o0 = plane0.origin();
    double d = ((o0[0]-plane1.origin()[0])*n1[0]
              + (o0[1]-plane1.origin()[1])*n1[1]
              + (o0[2]-plane1.origin()[2])*n1[2]) / n1_mag;
    double dist_ortho_sq = d * d;
    if (dist_ortho_sq < Tolerance::ZERO_TOLERANCE) return false;
    double dist_sq = output.dot(output);
    if (dist_sq / dist_ortho_sq >= 10.0) return false;
    return true;
}

std::vector<Polyline> Intersection::polyline_boolean(const Polyline& a, const Polyline& b, int clip_type) {
    return BooleanPolyline::compute(a, b, clip_type);
}

// Miter-join polygon offset in plane-space 2D, used by wood's
// `clipper_util::offset_in_3d`. For each vertex of the CCW-oriented input
// polygon, compute the bisector of the two adjacent
// edge outward normals and translate the vertex by `delta` along the
// bisector — giving a parallel offset curve with mitered corners.
//
// For a convex vertex the miter point is exactly at the intersection of the
// two offset edges. For concave vertices the same formula produces the
// correct "inward" offset (since both normals and bisector point outward
// relative to CCW winding).
//
// Degenerate cases handled:
//   - length < 3 after closing-dup strip → fail
//   - zero-length edges → skipped
//   - miter angle ≈ 180° (`1 + n_prev·n_next` near zero) → fall back to
//     plain edge-normal offset (equivalent to square cap for that vertex).
bool Intersection::offset_in_3d(Polyline& polyline, const Plane& plane, double offset) {
    size_t n_raw = polyline.point_count();
    if (n_raw < 3) return false;

    Point origin = polyline.get_point(0);
    // Use plane's canonical in-plane axes (smallest-|coef| pivot rule) so
    // the 2D projection depends only on the normal, not on how the plane
    // was constructed. Keeps offset output deterministic across call sites.
    Vector xax = plane.base1();
    Vector yax = plane.base2();

    // Project to 2D + strip closing duplicate.
    struct P2 { double x, y; };
    std::vector<P2> path;
    path.reserve(n_raw);
    for (size_t i = 0; i < n_raw; ++i) {
        Point p = polyline.get_point(i);
        double dx = p[0]-origin[0], dy = p[1]-origin[1], dz = p[2]-origin[2];
        double u = dx*xax[0] + dy*xax[1] + dz*xax[2];
        double v = dx*yax[0] + dy*yax[1] + dz*yax[2];
        path.push_back({u, v});
    }
    if (path.size() >= 2) {
        double dx = path.back().x - path.front().x;
        double dy = path.back().y - path.front().y;
        if (dx*dx + dy*dy < 1e-12) path.pop_back();
    }
    size_t n = path.size();
    if (n < 3) return false;

    // Shoelace signed area determines winding. Wood expects outward miter
    // when delta > 0; ensure CCW winding (positive signed area) by mirroring
    // delta if the input is CW.
    double signed_area = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const P2& a = path[i];
        const P2& b = path[(i+1) % n];
        signed_area += a.x * b.y - b.x * a.y;
    }
    double delta = offset;
    if (signed_area < 0.0) delta = -delta;

    // Per-edge outward normal for CCW polygon: rotate edge vector 90° CW.
    // edge_i = path[i+1] - path[i]; outward = (edge.y, -edge.x) / |edge|.
    auto edge_normal = [&](size_t i) -> P2 {
        const P2& a = path[i];
        const P2& b = path[(i+1) % n];
        double ex = b.x - a.x, ey = b.y - a.y;
        double len = std::sqrt(ex*ex + ey*ey);
        if (len < 1e-12) return {0.0, 0.0};
        return { ey/len, -ex/len };
    };

    std::vector<P2> normals;
    normals.reserve(n);
    for (size_t i = 0; i < n; ++i) normals.push_back(edge_normal(i));

    // At vertex i, the two adjacent edges are (i-1)→i and i→(i+1). Their
    // outward normals are n_prev = normals[(i+n-1)%n] and n_next = normals[i].
    // Three cases:
    //   - concave corner (sin_a·delta < 0 AND cos_a > -0.999): emit 3 vertices
    //     (vertex offset along n_prev, original vertex, vertex offset along
    //     n_next) to preserve polygon simplicity at inward-turning corners.
    //   - near-180° turn (|1+cos_a| < 1e-9): mean-normal fallback.
    //   - convex corner (default): single miter bisector.
    // The 3-vertex branch avoids the miter-spike that would otherwise jut
    // into the offset polygon at a reflex corner (output would self-intersect
    // or cover incorrect area).
    std::vector<P2> out;
    out.reserve(n * 3);
    for (size_t i = 0; i < n; ++i) {
        const P2& np = normals[(i + n - 1) % n];
        const P2& nn = normals[i];
        double cos_a = np.x*nn.x + np.y*nn.y;
        double sin_a = np.x*nn.y - np.y*nn.x;
        double denom = 1.0 + cos_a;
        // 3-vertex concave emission is only correct for OUTER offsets
        // (user `offset > 0`). For inner offsets, use the single miter
        // bisector (equivalent to a Vatti-Negative cleanup of the 3-vertex
        // form at convex corners, without the post-union pass).
        bool concave = (cos_a > -0.999) && (sin_a * delta < 0.0) && (offset > 0.0);
        if (concave) {
            // 3-vertex emission at reflex corner.
            out.push_back({ path[i].x + np.x * delta, path[i].y + np.y * delta });
            out.push_back({ path[i].x, path[i].y });
            out.push_back({ path[i].x + nn.x * delta, path[i].y + nn.y * delta });
        } else if (std::abs(denom) < 1e-9) {
            // Anti-parallel edges (180° turn) — mean-normal displacement.
            double bx = np.x + nn.x;
            double by = np.y + nn.y;
            double bl = std::sqrt(bx*bx + by*by);
            if (bl < 1e-12) {
                out.push_back({ path[i].x + nn.x * delta, path[i].y + nn.y * delta });
            } else {
                out.push_back({ path[i].x + (bx/bl) * delta, path[i].y + (by/bl) * delta });
            }
        } else {
            // Convex corner: miter bisector.
            // bisector b = (n_prev + n_next), k = delta / (1 + cos_a).
            double k = delta / denom;
            out.push_back({ path[i].x + (np.x + nn.x) * k,
                            path[i].y + (np.y + nn.y) * k });
        }
    }
    size_t nout = out.size();
    if (nout < 3) return false;

    // Shoelace on result — reject degenerate output.
    double out_area = 0.0;
    for (size_t i = 0; i < nout; ++i) {
        const P2& a = out[i];
        const P2& b = out[(i+1) % nout];
        out_area += a.x * b.y - b.x * a.y;
    }
    if (std::abs(out_area) * 0.5 < 0.0001) return false;

    // Rotate result to start at vertex closest to input[0] (wood does this
    // so downstream code that indexes offset[0] gets a point near the
    // original first vertex).
    size_t cp = 0;
    double cd = (out[0].x - path[0].x)*(out[0].x - path[0].x)
              + (out[0].y - path[0].y)*(out[0].y - path[0].y);
    for (size_t i = 1; i < nout; ++i) {
        double d = (out[i].x - path[0].x)*(out[i].x - path[0].x)
                 + (out[i].y - path[0].y)*(out[i].y - path[0].y);
        if (d < cd) { cd = d; cp = i; }
    }
    if (cp != 0) std::rotate(out.begin(), out.begin() + cp, out.end());

    // Transform back to 3D, close polyline.
    std::vector<Point> pts;
    pts.reserve(nout + 1);
    for (const P2& p2 : out) {
        double u = p2.x, v = p2.y;
        pts.emplace_back(origin[0] + u*xax[0] + v*yax[0],
                          origin[1] + u*xax[1] + v*yax[1],
                          origin[2] + u*xax[2] + v*yax[2]);
    }
    pts.push_back(pts.front());
    polyline = Polyline(pts);
    return true;
}

// Verbatim port of wood's cgal::collider::clipper_util::get_intersection_between_two_polylines
// (clipper_util.cpp:524-620). Plane-space 2D boolean via session's native
// BooleanPolyline (Vatti), matching the pattern session already uses for
// `polyline_boolean`.
bool Intersection::polyline_boolean_2d_in_plane(
    const Polyline& polyline0,
    const Polyline& polyline1,
    const Plane& plane,
    Polyline& intersection_result,
    int intersection_type,
    bool include_triangles,
    double min_area,
    double collapse_eps)
{
    size_t n0 = polyline0.point_count();
    size_t n1 = polyline1.point_count();
    if (n0 < 3 || n1 < 3) return false;

    // Project both polylines to the plane's 2D frame (origin = polyline0[0]).
    // Use base1/base2 (smallest-|coef| pivot rule) for a deterministic frame
    // that depends only on the plane normal — avoids frame-rotation bias in
    // Vatti output vertex ordering.
    Point origin = polyline0.get_point(0);
    Vector xax = plane.base1();
    Vector yax = plane.base2();

    auto to_2d_polyline = [&](const Polyline& pl) -> Polyline {
        size_t n = pl.point_count();
        std::vector<Point> pts2d;
        pts2d.reserve(n + 1);
        for (size_t i = 0; i < n; ++i) {
            Point p = pl.get_point(i);
            double dx = p[0]-origin[0], dy = p[1]-origin[1], dz = p[2]-origin[2];
            double u = dx*xax[0] + dy*xax[1] + dz*xax[2];
            double v = dx*yax[0] + dy*yax[1] + dz*yax[2];
            pts2d.emplace_back(u, v, 0.0);
        }
        // Ensure closed for BooleanPolyline.
        if (pts2d.size() > 1) {
            const Point& f = pts2d.front();
            const Point& l = pts2d.back();
            double dx=f[0]-l[0], dy=f[1]-l[1];
            if (dx*dx+dy*dy > 1e-12) pts2d.push_back(pts2d.front());
        }
        return Polyline(pts2d);
    };
    Polyline a2d = to_2d_polyline(polyline0);
    Polyline b2d = to_2d_polyline(polyline1);

    // XOR not supported by session's BooleanPolyline — synthesise via union-minus-intersection.
    std::vector<Polyline> result_2d;
    if (intersection_type >= 0 && intersection_type <= 2) {
        result_2d = BooleanPolyline::compute(a2d, b2d, intersection_type);
    } else if (intersection_type == 3) {
        // A XOR B = (A ∪ B) − (A ∩ B). Rare in wood — done for completeness.
        auto u = BooleanPolyline::compute(a2d, b2d, 1);
        auto inter = BooleanPolyline::compute(a2d, b2d, 0);
        if (u.empty()) return false;
        if (inter.empty()) result_2d = u;
        else               result_2d = BooleanPolyline::compute(u[0], inter[0], 2);
    } else {
        return false;
    }
    if (result_2d.empty()) return false;
    const Polyline& C = result_2d[0];
    size_t nc = C.point_count();
    // Strip closing duplicate before area + triangle checks to match wood.
    if (nc > 1) {
        Point f = C.get_point(0);
        Point l = C.get_point(nc-1);
        double dx=f[0]-l[0], dy=f[1]-l[1];
        if (dx*dx+dy*dy < 1e-12) --nc;
    }
    if (nc < 3) return false;

    // Collapse near-coincident consecutive vertices when requested. Vatti
    // output can contain sub-epsilon duplicates at near-coincident input
    // edges; wood hexbox-family datasets need this merged out before the
    // triangle/area gate.
    std::vector<Point> src2d;
    src2d.reserve(nc);
    for (size_t i = 0; i < nc; ++i) src2d.push_back(C.get_point(i));
    if (collapse_eps > 0.0) {
        const double eps_sq = collapse_eps * collapse_eps;
        std::vector<Point> collapsed;
        collapsed.reserve(src2d.size());
        for (const Point& p : src2d) {
            if (!collapsed.empty()) {
                double dx = p[0] - collapsed.back()[0];
                double dy = p[1] - collapsed.back()[1];
                if (dx*dx + dy*dy < eps_sq) continue;
            }
            collapsed.push_back(p);
        }
        if (collapsed.size() >= 2) {
            double dx = collapsed.back()[0] - collapsed.front()[0];
            double dy = collapsed.back()[1] - collapsed.front()[1];
            if (dx*dx + dy*dy < eps_sq) collapsed.pop_back();
        }
        src2d.swap(collapsed);
        nc = src2d.size();
        if (nc < 3) return false;
    }
    if (nc == 3 && !include_triangles) return false;

    // Shoelace area in the projected (u,v) plane.
    double area = 0.0;
    for (size_t i = 0; i < nc; ++i) {
        const Point& p0 = src2d[i];
        const Point& p1 = src2d[(i+1) % nc];
        area += p0[0]*p1[1] - p1[0]*p0[1];
    }
    if (std::abs(area) * 0.5 <= min_area) return false;

    // Transform back to 3D (u,v → origin + u*xax + v*yax).
    std::vector<Point> pts;
    pts.reserve(nc + 1);
    for (size_t i = 0; i < nc; ++i) {
        double u = src2d[i][0], v = src2d[i][1];
        pts.emplace_back(origin[0] + u*xax[0] + v*yax[0],
                          origin[1] + u*xax[1] + v*yax[1],
                          origin[2] + u*xax[2] + v*yax[2]);
    }
    pts.push_back(pts.front());
    intersection_result = Polyline(pts);
    return true;
}

bool Intersection::polyline_plane_to_line(const Polyline& poly, const Plane& plane,
                                            const Point& align_start, Line& out) {
    std::vector<Point> pts;
    std::vector<int> edge_ids;
    if (!polyline_plane(poly, plane, pts, edge_ids)) return false;
    if (pts.size() < 2) return false;
    const Point& a = pts[0];
    const Point& b = pts[1];
    double da = (a[0]-align_start[0])*(a[0]-align_start[0]) +
                (a[1]-align_start[1])*(a[1]-align_start[1]) +
                (a[2]-align_start[2])*(a[2]-align_start[2]);
    double db = (b[0]-align_start[0])*(b[0]-align_start[0]) +
                (b[1]-align_start[1])*(b[1]-align_start[1]) +
                (b[2]-align_start[2])*(b[2]-align_start[2]);
    if (da <= db) out = Line::from_points(a, b);
    else          out = Line::from_points(b, a);
    return true;
}

bool Intersection::quad_from_line_top_bottom_planes(const Plane& face_plane,
                                                      const Line& line,
                                                      const Plane& plane0,
                                                      const Plane& plane1,
                                                      Polyline& out) {
    // End-cap planes perpendicular to the joint line at each endpoint.
    Vector dir = line.to_vector();
    Point s = line.start();
    Plane lp0 = Plane::from_point_normal(s, dir);
    Vector dir2 = line.to_vector();
    Point e = line.end();
    Plane lp1 = Plane::from_point_normal(e, dir2);

    // Two-step formulation: corner = line(topOrBot ∩ face) ∩ endcap. More
    // numerically stable than the single 3-plane determinant — source of the
    // top_to_side_box 0.5 mm x-shift when face_plane and topOrBot meet at
    // close-to-right-angle. Mirrors the `plane_4planes_open` refactor landed
    // in Phase 6.2 for vidy_folding.
    auto corner = [&](const Plane& endcap, const Plane& topOrBot, Point& outp) -> bool {
        Line l;
        if (!plane_plane(topOrBot, face_plane, l)) return false;
        return line_plane(l, endcap, outp, false);
    };
    Point p0, p1, p2, p3;
    if (!corner(lp0, plane0, p0)) return false;
    if (!corner(lp0, plane1, p1)) return false;
    if (!corner(lp1, plane1, p2)) return false;
    if (!corner(lp1, plane0, p3)) return false;
    out = Polyline(std::vector<Point>{p0, p1, p2, p3, p0});
    return true;
}

bool Intersection::orthogonal_vector_between_two_plane_pairs(const Plane& pp00,
                                                               const Plane& pp10,
                                                               const Plane& pp11,
                                                               Vector& out) {
    // Verbatim port of wood `cgal_intersection_util.cpp:619-628`:
    //   plane_plane(pp00, pp10, l0);
    //   plane_plane(pp00, pp11, l1);
    //   output = l1.point() - l0.projection(l1.point());
    // Use the CGAL-canonical anchor variant so `l1.start()` matches
    // wood's `l1.point()` (foot of perpendicular from world origin);
    // the downstream perpendicular-projection formula relies on this
    // anchor choice for parallel-line cases (plate top/bottom face
    // pairs in ts_e_p_5 → 0.5 mm drift on top_to_side_box).
    Line l0, l1;
    if (!plane_plane_to_line_canonical(pp00, pp10, l0)) return false;
    if (!plane_plane_to_line_canonical(pp00, pp11, l1)) return false;
    Point p1 = l1.start();
    Vector ldir = l0.to_vector();
    double len_sq = ldir[0]*ldir[0] + ldir[1]*ldir[1] + ldir[2]*ldir[2];
    if (len_sq < 1e-20) return false;
    Point l0s = l0.start();
    Vector v(p1[0]-l0s[0], p1[1]-l0s[1], p1[2]-l0s[2]);
    double t = (v[0]*ldir[0] + v[1]*ldir[1] + v[2]*ldir[2]) / len_sq;
    Point p1_proj_on_l0(l0s[0]+ldir[0]*t, l0s[1]+ldir[1]*t, l0s[2]+ldir[2]*t);
    out = Vector(p1[0]-p1_proj_on_l0[0],
                 p1[1]-p1_proj_on_l0[1],
                 p1[2]-p1_proj_on_l0[2]);
    return true;
}

bool Intersection::closed_and_open_paths_2d(const Polyline& plate,
                                              const Polyline& joint,
                                              const Plane& plane,
                                              Polyline& out,
                                              std::pair<double, double>& cp_pair) {
    // Port of wood `wood_element.cpp:438-651`. Clips an OPEN-path joint
    // outline against a CLOSED plate polygon in 2D and returns the clipped
    // 3D segment + parametric positions on the plate edges.
    //
    // Algorithm: project plate + joint into the plate-frame 2D, then for
    // each joint segment compute its intersection parameters against every
    // plate edge, sort, classify each sub-segment by midpoint test
    // (point-in-polygon), keep insides, concatenate using the
    // distance-based reorientation rule, then locate t0/t1 on the plate
    // boundary.

    struct P2 { double x, y; };

    Point  origin = plate.get_point(0);
    // base1/base2 frame (smallest-|coef| pivot rule) — depends only on
    // normal, consistent with wood's CGAL Plane_3::base1/base2.
    Vector xax = plane.base1();
    Vector yax = plane.base2();

    auto to_2d = [&](const Point& p) -> P2 {
        double dx = p[0]-origin[0], dy = p[1]-origin[1], dz = p[2]-origin[2];
        return {dx*xax[0]+dy*xax[1]+dz*xax[2],
                dx*yax[0]+dy*yax[1]+dz*yax[2]};
    };
    auto to_3d = [&](double u, double v) -> Point {
        return Point(origin[0] + u*xax[0] + v*yax[0],
                      origin[1] + u*xax[1] + v*yax[1],
                      origin[2] + u*xax[2] + v*yax[2]);
    };

    // Plate outline (2D): strip closing duplicate.
    std::vector<P2> plate2d;
    {
        size_t n = plate.point_count();
        if (n > 1) {
            Point f = plate.get_point(0);
            Point l = plate.get_point(n-1);
            if (std::abs(f[0]-l[0]) < 1e-6 &&
                std::abs(f[1]-l[1]) < 1e-6 &&
                std::abs(f[2]-l[2]) < 1e-6) n--;
        }
        plate2d.reserve(n);
        for (size_t i = 0; i < n; i++) plate2d.push_back(to_2d(plate.get_point(i)));
    }
    if (plate2d.size() < 3) return false;

    // Joint outline (2D, open path).
    std::vector<P2> joint2d;
    joint2d.reserve(joint.point_count());
    for (size_t i = 0; i < joint.point_count(); i++)
        joint2d.push_back(to_2d(joint.get_point(i)));
    if (joint2d.size() < 2) return false;

    // New Vatti sweep-line path for open-subject clipping, gated by
    // VATTI_OPEN env var for safe progressive rollout. Produces c2d from
    // BooleanPolyline::clip_open_against_closed; falls through to the
    // legacy per-segment algorithm when the env var is unset.
    //
    // Robust handling of near-parallel near-coincident edges (rossiniere's
    // side_removal rectangles) — the Vatti sweep treats these consistently
    // via scanline winding rather than per-segment midpoint tests.
    static const bool USE_VATTI_OPEN = []{
        const char* v = std::getenv("VATTI_OPEN");
        return v && v[0] && v[0] != '0';
    }();
    if (USE_VATTI_OPEN) {
        // Build 2D polylines from the projected P2 arrays (z=0).
        std::vector<Point> joint_2d_pts;
        joint_2d_pts.reserve(joint2d.size());
        for (const auto& p : joint2d) joint_2d_pts.emplace_back(p.x, p.y, 0.0);
        std::vector<Point> plate_2d_pts;
        plate_2d_pts.reserve(plate2d.size() + 1);
        for (const auto& p : plate2d) plate_2d_pts.emplace_back(p.x, p.y, 0.0);
        plate_2d_pts.push_back(plate_2d_pts.front()); // close
        Polyline joint_2d_pl(joint_2d_pts);
        Polyline plate_2d_pl(plate_2d_pts);
        auto pieces = BooleanPolyline::clip_open_against_closed(joint_2d_pl, plate_2d_pl);
        if (pieces.empty() || pieces[0].point_count() < 2) return false;
        // Take the first (typically longest) piece as c2d.
        const Polyline& first = pieces[0];
        std::vector<P2> c2d;
        c2d.reserve(first.point_count());
        for (size_t i = 0; i < first.point_count(); i++) {
            Point p = first.get_point(i);
            c2d.push_back({p[0], p[1]});
        }
        // Locate t0/t1 on plate edges (same logic as legacy path).
        auto closest_param_v = [](const P2& p, const P2& a, const P2& b) -> double {
            double abx = b.x-a.x, aby = b.y-a.y;
            double l2 = abx*abx + aby*aby;
            if (l2 < 1e-20) return 0.0;
            double apx = p.x-a.x, apy = p.y-a.y;
            double t = (apx*abx + apy*aby) / l2;
            if (t < 0.0) t = 0.0;
            if (t > 1.0) t = 1.0;
            return t;
        };
        auto sq_dist_seg_v = [](const P2& p, const P2& a, const P2& b) -> double {
            double abx = b.x-a.x, aby = b.y-a.y;
            double l2 = abx*abx + aby*aby;
            if (l2 < 1e-20) { double dx = p.x-a.x, dy = p.y-a.y; return dx*dx+dy*dy; }
            double apx = p.x-a.x, apy = p.y-a.y;
            double t = (apx*abx+apy*aby)/l2;
            if (t < 0.0) t = 0.0;
            if (t > 1.0) t = 1.0;
            double px = a.x+t*abx, py = a.y+t*aby;
            double dx = p.x-px, dy = p.y-py;
            return dx*dx+dy*dy;
        };
        double t0 = -1.0, t1 = -1.0;
        for (size_t i = 0; i < plate2d.size(); i++) {
            const P2& a = plate2d[i];
            const P2& b = plate2d[(i+1) % plate2d.size()];
            for (int j = 0; j < 2; j++) {
                size_t idx = (j == 0) ? 0 : c2d.size() - 1;
                double dist_sq = sq_dist_seg_v(c2d[idx], a, b);
                if (j == 0 && dist_sq < 1.0) t0 = (double)i + closest_param_v(c2d[0], a, b);
                else if (j == 1 && dist_sq < 1.0) t1 = (double)i + closest_param_v(c2d[c2d.size()-1], a, b);
            }
            if (t0 >= 0.0 && t1 >= 0.0) break;
        }
        bool reverse_flag = (t0 > t1);
        if ((size_t)std::floor(t0) == 0 && (size_t)std::floor(t1) == c2d.size() - 1)
            reverse_flag = !reverse_flag;
        if (reverse_flag) {
            std::swap(t0, t1);
            std::reverse(c2d.begin(), c2d.end());
        }
        if (t0 < 0.0 || t1 < 0.0) return false;
        std::vector<Point> out_pts;
        out_pts.reserve(c2d.size());
        for (const auto& p : c2d) out_pts.push_back(to_3d(p.x, p.y));
        out = Polyline(out_pts);
        cp_pair = std::pair<double, double>(t0, t1);
        return true;
    }

    // 2D winding-number point-in-polygon (matches Polyline::point_in_polygon_2d).
    auto pip = [&](double px, double py) -> bool {
        int wn = 0;
        size_t n = plate2d.size();
        for (size_t i = 0; i < n; i++) {
            const P2& a = plate2d[i];
            const P2& b = plate2d[(i+1)%n];
            if (a.y <= py) {
                if (b.y > py) {
                    double e = (b.x-a.x)*(py-a.y) - (px-a.x)*(b.y-a.y);
                    if (e > 0.0) wn++;
                }
            } else {
                if (b.y <= py) {
                    double e = (b.x-a.x)*(py-a.y) - (px-a.x)*(b.y-a.y);
                    if (e < 0.0) wn--;
                }
            }
        }
        return wn != 0;
    };

    // 2D segment-segment intersection. Returns true if segments overlap at
    // a single point with parameters t_s in [-eps, 1+eps] and t_e in
    // [-eps, 1+eps]; outputs the parameters.
    auto seg_seg_2d = [](const P2& s0, const P2& s1, const P2& e0, const P2& e1,
                         double& t_s, double& t_e) -> bool {
        double sx = s1.x-s0.x, sy = s1.y-s0.y;
        double ex = e1.x-e0.x, ey = e1.y-e0.y;
        double denom = sx*ey - sy*ex;
        if (std::abs(denom) < 1e-20) return false;
        double dx = e0.x-s0.x, dy = e0.y-s0.y;
        t_s = (dx*ey - dy*ex) / denom;
        t_e = (dx*sy - dy*sx) / denom;
        return true;
    };

    // Collinear-overlap detection. If joint segment s0→s1 is collinear with
    // plate edge e0→e1 (cross product of unit directions below sin(angle_eps)
    // AND perpendicular distance of s0 from edge-line below dist_eps), compute
    // the parametric range of overlap on the joint segment.
    //
    // Returns true if overlap exists; outputs t_enter, t_exit (on joint
    // segment). Handles rectangle-edge coincident with plate-edge via
    // direct angle+distance test (no grid snapping; thresholds are true
    // FP-noise tolerances).
    auto collinear_overlap = [](const P2& s0, const P2& s1, const P2& e0, const P2& e1,
                                 double& t_enter, double& t_exit) -> bool {
        double sx = s1.x-s0.x, sy = s1.y-s0.y;
        double ex = e1.x-e0.x, ey = e1.y-e0.y;
        double sl2 = sx*sx + sy*sy;
        double el2 = ex*ex + ey*ey;
        if (sl2 < 1e-20 || el2 < 1e-20) return false;
        // Normalized cross (= sin of angle between directions).
        double cross_norm = (sx*ey - sy*ex) / std::sqrt(sl2 * el2);
        const double ANGLE_SIN_EPS = 1e-4; // 0.006° — true parallel
        if (std::abs(cross_norm) > ANGLE_SIN_EPS) return false;
        // Perpendicular distance of s0 from line through (e0, e1).
        double apx = s0.x-e0.x, apy = s0.y-e0.y;
        double perp = (apx*ey - apy*ex) / std::sqrt(el2);
        const double DIST_EPS = 1e-3; // 0.001 mm — true FP noise
        if (std::abs(perp) > DIST_EPS) return false;
        // Parametric positions of s0, s1 along edge direction.
        double ts0 = (apx*ex + apy*ey) / el2;
        double bpx = s1.x-e0.x, bpy = s1.y-e0.y;
        double ts1 = (bpx*ex + bpy*ey) / el2;
        // Overlap in edge-param space: [max(0, min(ts0,ts1)), min(1, max(ts0,ts1))].
        double tmin = std::min(ts0, ts1), tmax = std::max(ts0, ts1);
        double ov_min = std::max(0.0, tmin);
        double ov_max = std::min(1.0, tmax);
        if (ov_max - ov_min < 1e-9) return false; // no overlap
        // Convert overlap back to joint segment param.
        // joint param: t such that s0 + t*(s1-s0) = point, where point is
        // at param ts along edge. ts = ts0 + t*(ts1-ts0) → t = (ts-ts0)/(ts1-ts0).
        double tsr = ts1 - ts0;
        if (std::abs(tsr) < 1e-20) return false; // degenerate
        t_enter = (ov_min - ts0) / tsr;
        t_exit  = (ov_max - ts0) / tsr;
        if (t_enter > t_exit) std::swap(t_enter, t_exit);
        // Clamp to [0, 1] in joint-segment space.
        if (t_enter < 0.0) t_enter = 0.0;
        if (t_exit  > 1.0) t_exit  = 1.0;
        return (t_exit - t_enter) > 1e-9;
    };

    // Walk each joint segment, gather inside sub-pieces. A "piece" is a
    // contiguous run of (P2) vertices that lie inside the plate polygon.
    std::vector<std::vector<P2>> pieces;
    const double EPS = 1e-9;
    for (size_t s = 0; s + 1 < joint2d.size(); s++) {
        const P2& p0 = joint2d[s];
        const P2& p1 = joint2d[s+1];
        std::vector<double> ts;
        ts.push_back(0.0);
        // Collinear-overlap flags per sub-segment: if sub-segment [ts[i], ts[i+1]]
        // is collinear with any plate edge, treat it as "on boundary" → inside.
        std::vector<std::pair<double, double>> coll_ranges;
        for (size_t i = 0; i < plate2d.size(); i++) {
            const P2& a = plate2d[i];
            const P2& b = plate2d[(i+1)%plate2d.size()];
            double t_s, t_e;
            if (seg_seg_2d(p0, p1, a, b, t_s, t_e)) {
                if (t_s > EPS && t_s < 1.0 - EPS &&
                    t_e >= -EPS && t_e <= 1.0 + EPS) {
                    ts.push_back(t_s);
                }
            }
            double t_in, t_out;
            if (collinear_overlap(p0, p1, a, b, t_in, t_out)) {
                coll_ranges.emplace_back(t_in, t_out);
                if (t_in > EPS && t_in < 1.0 - EPS) ts.push_back(t_in);
                if (t_out > EPS && t_out < 1.0 - EPS) ts.push_back(t_out);
            }
        }
        ts.push_back(1.0);
        std::sort(ts.begin(), ts.end());
        ts.erase(std::unique(ts.begin(), ts.end(),
                              [EPS](double x, double y){ return std::abs(x-y)<EPS; }),
                  ts.end());

        auto sub_is_collinear = [&](double t_a, double t_b) -> bool {
            double t_mid = 0.5 * (t_a + t_b);
            for (const auto& r : coll_ranges) {
                if (t_mid >= r.first - EPS && t_mid <= r.second + EPS) return true;
            }
            return false;
        };

        std::vector<P2> current;
        for (size_t i = 0; i + 1 < ts.size(); i++) {
            double t_mid = 0.5 * (ts[i] + ts[i+1]);
            double mx = p0.x + (p1.x-p0.x)*t_mid;
            double my = p0.y + (p1.y-p0.y)*t_mid;
            bool include = pip(mx, my) || sub_is_collinear(ts[i], ts[i+1]);
            if (include) {
                P2 sub_a{p0.x + (p1.x-p0.x)*ts[i],   p0.y + (p1.y-p0.y)*ts[i]};
                P2 sub_b{p0.x + (p1.x-p0.x)*ts[i+1], p0.y + (p1.y-p0.y)*ts[i+1]};
                if (current.empty()) {
                    current.push_back(sub_a);
                    current.push_back(sub_b);
                } else {
                    // Continuation: drop the duplicate vertex at the join.
                    double dx = current.back().x - sub_a.x;
                    double dy = current.back().y - sub_a.y;
                    if (dx*dx + dy*dy < 1e-18) {
                        current.push_back(sub_b);
                    } else {
                        pieces.push_back(std::move(current));
                        current.clear();
                        current.push_back(sub_a);
                        current.push_back(sub_b);
                    }
                }
            } else if (!current.empty()) {
                pieces.push_back(std::move(current));
                current.clear();
            }
        }
        if (!current.empty()) {
            // Try to extend across the segment-to-segment join.
            // The next iteration starts at p1 == joint2d[s+1] which equals
            // current.back() if the previous sub-segment ended at t=1.
            pieces.push_back(std::move(current));
        }
    }

    // Concatenate pieces into a single polyline using a distance-based
    // reorientation rule.
    std::vector<P2> c2d;
    int count = 0;
    auto sq2 = [](const P2& a, const P2& b) {
        double dx = a.x-b.x, dy = a.y-b.y;
        return dx*dx + dy*dy;
    };
    const double DISTANCE_SQ = 0.01;
    for (const auto& piece : pieces) {
        if (piece.size() <= 1) continue;
        if (count == 0) {
            c2d = piece;
        } else {
            std::vector<P2> pts = piece;
            if (sq2(c2d.back(), pts.front()) > DISTANCE_SQ &&
                sq2(c2d.back(), pts.back())  > DISTANCE_SQ) {
                std::reverse(c2d.begin(), c2d.end());
            }
            if (sq2(c2d.back(), pts.front()) > sq2(c2d.back(), pts.back())) {
                std::reverse(pts.begin(), pts.end());
            }
            for (size_t j = 1; j < pts.size(); j++) c2d.push_back(pts[j]);
        }
        count++;
    }

    if (c2d.size() < 2) return false;

    // Locate parametric t0/t1 on plate edges (same logic as Clipper version).
    auto closest_param = [](const P2& p, const P2& a, const P2& b) -> double {
        double abx = b.x-a.x, aby = b.y-a.y;
        double l2 = abx*abx + aby*aby;
        if (l2 < 1e-20) return 0.0;
        double apx = p.x-a.x, apy = p.y-a.y;
        double t = (apx*abx + apy*aby) / l2;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        return t;
    };
    auto sq_dist_seg = [](const P2& p, const P2& a, const P2& b) -> double {
        double abx = b.x-a.x, aby = b.y-a.y;
        double l2 = abx*abx + aby*aby;
        if (l2 < 1e-20) {
            double dx = p.x-a.x, dy = p.y-a.y;
            return dx*dx + dy*dy;
        }
        double apx = p.x-a.x, apy = p.y-a.y;
        double t = (apx*abx + apy*aby) / l2;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        double px = a.x + t*abx, py = a.y + t*aby;
        double dx = p.x-px, dy = p.y-py;
        return dx*dx + dy*dy;
    };

    double t0 = -1.0, t1 = -1.0;
    for (size_t i = 0; i < plate2d.size(); i++) {
        const P2& a = plate2d[i];
        const P2& b = plate2d[(i+1) % plate2d.size()];
        for (int j = 0; j < 2; j++) {
            size_t idx = (j == 0) ? 0 : c2d.size() - 1;
            double dist_sq = sq_dist_seg(c2d[idx], a, b);
            if (j == 0 && dist_sq < 1.0) {
                t0 = (double)i + closest_param(c2d[0], a, b);
            } else if (j == 1 && dist_sq < 1.0) {
                t1 = (double)i + closest_param(c2d[c2d.size()-1], a, b);
            }
        }
        if (t0 >= 0.0 && t1 >= 0.0) break;
    }

    bool reverse_flag = (t0 > t1);
    if ((size_t)std::floor(t0) == 0 && (size_t)std::floor(t1) == c2d.size() - 1)
        reverse_flag = !reverse_flag;
    if (reverse_flag) {
        std::swap(t0, t1);
        std::reverse(c2d.begin(), c2d.end());
    }

    if (t0 < 0.0 || t1 < 0.0) return false;

    std::vector<Point> out_pts;
    out_pts.reserve(c2d.size());
    for (const auto& p : c2d) out_pts.push_back(to_3d(p.x, p.y));
    out = Polyline(out_pts);
    cp_pair = std::pair<double, double>(t0, t1);
    return true;
}

std::vector<std::tuple<int, int, int, int, int, Polyline>> Intersection::face_to_face(
    const std::vector<int>& adjacency,
    const std::vector<std::vector<Polyline>>& polylines,
    const std::vector<std::vector<Plane>>& planes,
    double coplanar_tolerance) {

    std::vector<std::tuple<int, int, int, int, int, Polyline>> results;

    for (size_t idx = 0; idx < adjacency.size(); idx += 4) {
        int a = adjacency[idx], b = adjacency[idx + 1];

        bool found = false;
        for (int i = 0; i < (int)planes[a].size() && !found; i++) {
            for (int j = 0; j < (int)planes[b].size(); j++) {
                if (!Plane::is_coplanar(
                        planes[a][i].origin(), planes[a][i].z_axis(),
                        planes[b][j].origin(), planes[b][j].z_axis(),
                        false, coplanar_tolerance))
                    continue;

                auto pts_i = polylines[a][i].get_points();
                Vector edge(pts_i[1][0]-pts_i[0][0], pts_i[1][1]-pts_i[0][1], pts_i[1][2]-pts_i[0][2]);
                edge.normalize_self();
                Vector zax = planes[a][i].z_axis();
                Vector yax = zax.cross(edge);
                yax.normalize_self();
                Plane pln(pts_i[0], edge, yax, zax);

                auto bools = Polyline::boolean_op(polylines[a][i], polylines[b][j], pln, 0);
                if (bools.empty() || bools[0].point_count() < 3) continue;

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

std::vector<int> Intersection::adjacency_search(
    std::vector<Element*>& elements, double inflate) {
    size_t N = elements.size();
    std::vector<OBB> obbs(N);
    for (size_t i = 0; i < N; i++) {
        std::vector<Point> pts;
        for (auto& pl : elements[i]->polylines())
            for (auto& p : pl.get_points()) pts.push_back(p);
        obbs[i] = OBB::from_points(pts, inflate);
    }

    std::vector<AABB> aabbs(N);
    for (size_t i = 0; i < N; i++) aabbs[i] = obbs[i].aabb();
    double ws = 0;
    for (auto& a : aabbs) {
        ws = std::max(ws, std::abs(a.cx+a.hx)); ws = std::max(ws, std::abs(a.cy+a.hy));
        ws = std::max(ws, std::abs(a.cz+a.hz)); ws = std::max(ws, std::abs(a.cx-a.hx));
        ws = std::max(ws, std::abs(a.cy-a.hy)); ws = std::max(ws, std::abs(a.cz-a.hz));
    }
    SpatialBVH bvh;
    bvh.build_from_aabbs(aabbs.data(), N, ws * 2);
    std::vector<int> adjacency;
    for (size_t i = 0; i < N; i++) {
        auto hits = bvh.query_aabb(aabbs[i]);
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
    double coplanar_tolerance) {

    size_t N = elements.size();
    std::vector<std::vector<Polyline>> all_polys(N);
    std::vector<std::vector<Plane>> all_planes(N);
    for (size_t i = 0; i < N; i++) {
        all_polys[i] = elements[i]->polylines();
        all_planes[i] = elements[i]->planes();
    }
    return face_to_face(adjacency, all_polys, all_planes, coplanar_tolerance);
}


// Port of cgal_box_search.h:252-496 line_line_intersection_with_properties.
bool Intersection::line_line_classified(
    const Line& s0,
    const Line& s1,
    int  n_segs_0,
    int  n_segs_1,
    int  cur_seg_0,
    int  cur_seg_1,
    double above_closer_to_edge,
    Point& p0,
    Point& p1,
    Vector& v0,
    Vector& v1,
    Vector& normal,
    bool& type0,
    bool& type1,
    bool& is_parallel)
{
    constexpr double DIST_SQ  = 1e-6;
    constexpr double EPS_PAR  = 1.0;   // degrees

    v0 = s0.to_vector();
    v1 = s1.to_vector();
    normal = v0.cross(v1);
    double nmag2 = normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2];
    double ang = v0.angle(v1, false, true);
    is_parallel = (nmag2 < 1e-24) || ((90.0 - std::abs(ang - 90.0)) < EPS_PAR);
    if (is_parallel) {
        Point  tmp_origin = s0.start();
        Vector tmp_normal = v0;
        Plane  pl_tmp = Plane::from_point_normal(tmp_origin, tmp_normal);
        normal = pl_tmp.base1();
    }
    normal.normalize_self();

    auto eq = [&](const Point& a, const Point& b) {
        double dx=a[0]-b[0], dy=a[1]-b[1], dz=a[2]-b[2];
        return dx*dx+dy*dy+dz*dz < DIST_SQ;
    };
    auto endcase = [&](const Point& pp, const Vector& dv0, const Vector& dv1) {
        p0 = pp; p1 = pp;
        v0 = dv0; v0.normalize_self();
        v1 = dv1; v1.normalize_self();
        type0 = 0; type1 = 0;
    };
    if (eq(s0.start(), s1.start())) { endcase(s0.start(), s0.end()-s0.start(), s1.end()-s1.start()); return true; }
    if (eq(s0.start(), s1.end()  )) { endcase(s0.start(), s0.end()-s0.start(), s1.start()-s1.end()); return true; }
    if (eq(s0.end(),   s1.start())) { endcase(s0.end(),   s0.start()-s0.end(), s1.end()-s1.start()); return true; }
    if (eq(s0.end(),   s1.end()  )) { endcase(s0.end(),   s0.start()-s0.end(), s1.start()-s1.end()); return true; }

    if (is_parallel) {
        v0.normalize_self();
        v1.normalize_self();
        auto signed_t = [](const Point& src, const Vector& unit, const Point& q) {
            return (q[0]-src[0])*unit[0] + (q[1]-src[1])*unit[1] + (q[2]-src[2])*unit[2];
        };
        auto proj_onto_line = [](const Line& L, const Point& q) {
            return L.closest_point(q, false).second;
        };
        std::vector<std::pair<double,double>> pts;
        auto push = [&](const Point& q) {
            Point q0 = proj_onto_line(s0, q);
            Point q1 = proj_onto_line(s1, q);
            pts.emplace_back(signed_t(s0.start(), v0, q0),
                             signed_t(s1.start(), v1, q1));
        };
        push(s0.start()); push(s0.end());
        push(s1.start()); push(s1.end());
        std::sort(pts.begin(), pts.end(),
                  [](const auto& a, const auto& b){ return a.first < b.first; });
        Point seg0_a(s0.start()[0]+pts[1].first*v0[0],
                     s0.start()[1]+pts[1].first*v0[1],
                     s0.start()[2]+pts[1].first*v0[2]);
        Point seg0_b(s0.start()[0]+pts[2].first*v0[0],
                     s0.start()[1]+pts[2].first*v0[1],
                     s0.start()[2]+pts[2].first*v0[2]);
        Point seg1_a(s1.start()[0]+pts[1].second*v1[0],
                     s1.start()[1]+pts[1].second*v1[1],
                     s1.start()[2]+pts[1].second*v1[2]);
        Point seg1_b(s1.start()[0]+pts[2].second*v1[0],
                     s1.start()[1]+pts[2].second*v1[1],
                     s1.start()[2]+pts[2].second*v1[2]);
        Point m0((seg0_a[0]+seg0_b[0])*0.5, (seg0_a[1]+seg0_b[1])*0.5, (seg0_a[2]+seg0_b[2])*0.5);
        Point m1((seg1_a[0]+seg1_b[0])*0.5, (seg1_a[1]+seg1_b[1])*0.5, (seg1_a[2]+seg1_b[2])*0.5);
        Point avg((m0[0]+m1[0])*0.5, (m0[1]+m1[1])*0.5, (m0[2]+m1[2])*0.5);
        p0 = proj_onto_line(s0, avg);
        p1 = proj_onto_line(s1, avg);
        auto t_of = [](const Line& L, const Point& q) {
            return L.closest_point(q, false).first;
        };
        double t0_v = t_of(s0, p0);
        double t1_v = t_of(s1, p1);
        if (t0_v > 0.5) v0 = Vector(-v0[0], -v0[1], -v0[2]);
        if (t1_v > 0.5) v1 = Vector(-v1[0], -v1[1], -v1[2]);
        type0 = 0; type1 = 0;
        return true;
    }

    v0.normalize_self();
    v1.normalize_self();
    double t0_v, t1_v;
    if (!Intersection::line_line_parameters(s0, s1, t0_v, t1_v,
            0.0, false, true)) {
        return false;
    }
    double t0c = std::max(0.0, std::min(1.0, t0_v));
    double t1c = std::max(0.0, std::min(1.0, t1_v));
    p0 = s0.point_at(t0c);
    p1 = s1.point_at(t1c);

    double tt0 = (t0c + (double)cur_seg_0) / (double)n_segs_0;
    double tt1 = (t1c + (double)cur_seg_1) / (double)n_segs_1;
    double close0 = 2.0 * std::abs(0.5 - tt0);
    double close1 = 2.0 * std::abs(0.5 - tt1);

    if (above_closer_to_edge < 0.0) {
        type0 = 1; type1 = 1;
    } else if (above_closer_to_edge > 1.0) {
        type0 = tt0 < tt1 ? 0 : 1;
        type1 = tt0 < tt1 ? 1 : 0;
    } else {
        type0 = close0 > above_closer_to_edge ? 0 : 1;
        type1 = close1 > above_closer_to_edge ? 0 : 1;
        if (close0 > close1 && type0 == 0 && type1 == 0) { type0 = 0; type1 = 1; }
        else if (close0 < close1 && type0 == 0 && type1 == 0) { type0 = 1; type1 = 0; }
    }

    if (tt0 > 0.5 && type0 == 0) v0 = Vector(-v0[0], -v0[1], -v0[2]);
    if (tt1 > 0.5 && type1 == 0) v1 = Vector(-v1[0], -v1[1], -v1[2]);
    return true;
}

} // namespace session_cpp
