#include "intersection.h"
#include "boolean_polyline.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "closest.h"
#include "bvh.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <cstring>
#include <set>
#include <functional>

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
    return (rank == 3);
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
    
    // Use Mesh's cached per-triangle BVH
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

std::vector<NurbsCurve> Intersection::surface_plane(
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

    // Wrapped UV distance (handles seam for closed directions)
    auto uv_dist = [&](double u1, double v1, double u2, double v2) -> double {
        double ddu = u1 - u2, ddv = v1 - v2;
        if (closed_u) {
            if (ddu > range_u * 0.5) ddu -= range_u;
            else if (ddu < -range_u * 0.5) ddu += range_u;
        }
        if (closed_v) {
            if (ddv > range_v * 0.5) ddv -= range_v;
            else if (ddv < -range_v * 0.5) ddv += range_v;
        }
        return std::hypot(ddu, ddv);
    };

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

    std::vector<NurbsCurve> result;

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

        // ------------------------------------------------------------------
        // 3. Evaluate all trace points to 3D
        // ------------------------------------------------------------------
        std::vector<Point> all_pts(uv_trace.size());
        for (size_t i = 0; i < uv_trace.size(); i++)
            all_pts[i] = surface.point_at(uv_trace[i].first, uv_trace[i].second);

        // ------------------------------------------------------------------
        // 4. Circle detection: if points lie on a circle → exact rational NURBS
        // ------------------------------------------------------------------
        NurbsCurve crv;
        if (is_loop && all_pts.size() >= 6) {
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
                    double knots[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
                    for (int i = 0; i < 10; i++) crv.set_knot(i, knots[i]);
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
        if (!crv.is_valid() && is_loop && all_pts.size() >= 8) {
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
                        double knots[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
                        for (int i = 0; i < 10; i++) crv.set_knot(i, knots[i]);
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
            if (m < 4) continue;

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
                    ? NurbsCurve::create_interpolated(pts_2d, CurveKnotStyle::ChordPeriodic)
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
    Point p0, p1, p2, p3;
    if (!plane_plane_plane_check(planes[0], planes[1], main_plane, 0.1, p0)) return false;
    if (!plane_plane_plane_check(planes[1], planes[2], main_plane, 0.1, p1)) return false;
    if (!plane_plane_plane_check(planes[2], planes[3], main_plane, 0.1, p2)) return false;
    if (!plane_plane_plane_check(planes[3], planes[0], main_plane, 0.1, p3)) return false;
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

    // 4 corners as 3-plane intersections.
    Point p0, p1, p2, p3;
    if (!plane_plane_plane(lp0, plane0, face_plane, p0)) return false;
    if (!plane_plane_plane(lp0, plane1, face_plane, p1)) return false;
    if (!plane_plane_plane(lp1, plane1, face_plane, p2)) return false;
    if (!plane_plane_plane(lp1, plane0, face_plane, p3)) return false;
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
    Line l0, l1;
    if (!plane_plane(pp00, pp10, l0)) return false;
    if (!plane_plane(pp00, pp11, l1)) return false;
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
    // Native (no Clipper2) port of wood `wood_element.cpp:438-651`. Clips an
    // OPEN-path joint outline against a CLOSED plate polygon in 2D and
    // returns the clipped 3D segment + parametric positions on the plate
    // edges.
    //
    // Algorithm: project plate + joint into the plate-frame 2D, then for
    // each joint segment compute its intersection parameters against every
    // plate edge, sort, classify each sub-segment by midpoint test
    // (point-in-polygon), keep insides, concatenate using the
    // distance-based reorientation rule, then locate t0/t1 on the plate
    // boundary.

    struct P2 { double x, y; };

    Point  origin = plate.get_point(0);
    Vector xax = plane.x_axis(); xax.normalize_self();
    Vector yax = plane.y_axis(); yax.normalize_self();

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

    // Walk each joint segment, gather inside sub-pieces. A "piece" is a
    // contiguous run of (P2) vertices that lie inside the plate polygon.
    std::vector<std::vector<P2>> pieces;
    const double EPS = 1e-9;
    for (size_t s = 0; s + 1 < joint2d.size(); s++) {
        const P2& p0 = joint2d[s];
        const P2& p1 = joint2d[s+1];
        std::vector<double> ts;
        ts.push_back(0.0);
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
        }
        ts.push_back(1.0);
        std::sort(ts.begin(), ts.end());
        ts.erase(std::unique(ts.begin(), ts.end(),
                              [EPS](double x, double y){ return std::abs(x-y)<EPS; }),
                  ts.end());

        std::vector<P2> current;
        for (size_t i = 0; i + 1 < ts.size(); i++) {
            double t_mid = 0.5 * (ts[i] + ts[i+1]);
            double mx = p0.x + (p1.x-p0.x)*t_mid;
            double my = p0.y + (p1.y-p0.y)*t_mid;
            if (pip(mx, my)) {
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

    // Concatenate pieces into a single polyline using the same
    // distance-based reorientation rule as the Clipper2 version.
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
    BVH bvh;
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

//==========================================================================================
// plane_to_face: side-to-side cross/lap joint between two plate elements
//==========================================================================================

int Intersection::are_points_inside(
    const Polyline& polygon,
    const Plane& plane,
    const std::vector<Point>& test_points,
    std::vector<int>& inside_indices_out) {

    // Project polygon + test points to plane local 2D (z=0) using x_axis/y_axis dot products.
    const Point& o = plane.origin();
    const Vector& xa = plane.x_axis();
    const Vector& ya = plane.y_axis();

    auto poly_pts = polygon.get_points();
    Polyline poly2d;
    {
        std::vector<Point> projected;
        projected.reserve(poly_pts.size());
        for (const auto& p : poly_pts) {
            double dx = p[0] - o[0], dy = p[1] - o[1], dz = p[2] - o[2];
            projected.emplace_back(dx*xa[0]+dy*xa[1]+dz*xa[2],
                                   dx*ya[0]+dy*ya[1]+dz*ya[2],
                                   0.0);
        }
        poly2d = Polyline(projected);
    }

    int count = 0;
    for (size_t i = 0; i < test_points.size(); i++) {
        const Point& p = test_points[i];
        double dx = p[0] - o[0], dy = p[1] - o[1], dz = p[2] - o[2];
        Point p2d(dx*xa[0]+dy*xa[1]+dz*xa[2],
                  dx*ya[0]+dy*ya[1]+dz*ya[2],
                  0.0);
        if (poly2d.point_in_polygon_2d(p2d)) {
            inside_indices_out.push_back(static_cast<int>(i));
            count++;
        }
    }
    return count;
}

bool Intersection::polyline_plane_cross_joint(
    const Polyline& c0,
    const Polyline& c1,
    const Plane& p0,
    const Plane& p1,
    Line& contact_out,
    std::pair<int,int>& edge_pair_out) {

    // c0 vs p1
    std::vector<Point> pts0;
    std::vector<int> edge_ids_0;
    if (!polyline_plane(c0, p1, pts0, edge_ids_0)) return false;

    // c1 vs p0
    std::vector<Point> pts1;
    std::vector<int> edge_ids_1;
    if (!polyline_plane(c1, p0, pts1, edge_ids_1)) return false;

    if (pts0.size() < 2 || pts1.size() < 2) return false;

    // Filter to points actually inside the other polygon (using its plane).
    std::vector<int> ID1;
    int count0 = are_points_inside(c0, p0, pts1, ID1);

    std::vector<int> ID0;
    int count1 = are_points_inside(c1, p1, pts0, ID0);

    if (count0 == 0 && count1 == 0) return false;

    if (std::abs(count0 - count1) == 2) {
        // Rectangle curve fully inside the other rectangle curve.
        if (count0 == 2) {
            contact_out = Line::from_points(pts0[0], pts0[1]);
            edge_pair_out = std::pair<int,int>(edge_ids_0[0], edge_ids_0[1]);
        } else {
            contact_out = Line::from_points(pts1[0], pts1[1]);
            edge_pair_out = std::pair<int,int>(edge_ids_1[0], edge_ids_1[1]);
        }
        return true;
    }

    if (count0 == 1 && count1 == 1) {
        contact_out = Line::from_points(pts0[ID0[0]], pts1[ID1[0]]);
        edge_pair_out = std::pair<int,int>(edge_ids_0[ID0[0]], edge_ids_1[ID1[0]]);
        return true;
    }

    if (count0 > 1 || count1 > 1) {
        // AABB of all inside points; pick the diagonal endpoints as contact.
        std::vector<Point> pts;
        pts.reserve(ID0.size() + ID1.size());
        for (int i : ID0) pts.push_back(pts0[i]);
        for (int i : ID1) pts.push_back(pts1[i]);

        double xmin = pts[0][0], ymin = pts[0][1], zmin = pts[0][2];
        double xmax = xmin, ymax = ymin, zmax = zmin;
        for (const auto& q : pts) {
            xmin = std::min(xmin, q[0]); ymin = std::min(ymin, q[1]); zmin = std::min(zmin, q[2]);
            xmax = std::max(xmax, q[0]); ymax = std::max(ymax, q[1]); zmax = std::max(zmax, q[2]);
        }
        Point lo(xmin, ymin, zmin);
        Point hi(xmax, ymax, zmax);
        contact_out = Line::from_points(lo, hi);

        int e0 = -1, e1 = -1;
        for (size_t i = 0; i < ID0.size(); i++) {
            const Point& q = pts0[ID0[i]];
            double d_lo = (q-lo).magnitude_squared();
            double d_hi = (q-hi).magnitude_squared();
            if (d_lo < 0.001 || d_hi < 0.001) { e0 = edge_ids_0[ID0[i]]; break; }
        }
        for (size_t i = 0; i < ID1.size(); i++) {
            const Point& q = pts1[ID1[i]];
            double d_lo = (q-lo).magnitude_squared();
            double d_hi = (q-hi).magnitude_squared();
            if (d_lo < 0.001 || d_hi < 0.001) { e1 = edge_ids_1[ID1[i]]; break; }
        }
        edge_pair_out = std::pair<int,int>(e0, e1);
        return true;
    }

    return false;
}

namespace {
// Angle in degrees between two vectors, in [0, 180].
double approximate_angle_deg(const Vector& a, const Vector& b) {
    double la = a.magnitude(), lb = b.magnitude();
    if (la < Tolerance::ZERO_TOLERANCE || lb < Tolerance::ZERO_TOLERANCE) return 0.0;
    double c = a.dot(b) / (la * lb);
    if (c > 1.0) c = 1.0;
    if (c < -1.0) c = -1.0;
    return std::acos(c) * 180.0 / 3.14159265358979323846;
}

Vector segment_to_vector(const Line& l) {
    return Vector(l[3]-l[0], l[4]-l[1], l[5]-l[2]);
}

Line opposite_segment(const Line& l) {
    return Line(l[3], l[4], l[5], l[0], l[1], l[2]);
}

Polyline translate_quad(const Polyline& poly, const Vector& v) {
    auto pts = poly.get_points();
    for (auto& p : pts) {
        p[0] += v[0]; p[1] += v[1]; p[2] += v[2];
    }
    return Polyline(pts);
}
} // anonymous namespace

bool Intersection::plane_to_face(
    const std::array<Polyline,2>& polylines_a,
    const std::array<Polyline,2>& polylines_b,
    const std::array<Plane,2>& planes_a,
    const std::array<Plane,2>& planes_b,
    CrossJoint& result,
    double angle_tol,
    const std::array<double,3>& extension) {

    result.face_ids_a = {-1, -1};
    result.face_ids_b = {-1, -1};
    result.type = 30;

    // 1. Parallelism guard.
    double raw_angle = approximate_angle_deg(planes_a[0].z_axis(), planes_b[0].z_axis());
    double angle = 90.0 - std::fabs(raw_angle - 90.0);
    if (angle < angle_tol) return false;

    // 2. Four cross-joint contact segments.
    const Polyline& cx0 = polylines_a[0];
    const Polyline& cx1 = polylines_a[1];
    const Polyline& cy0 = polylines_b[0];
    const Polyline& cy1 = polylines_b[1];
    const Plane& px0 = planes_a[0];
    const Plane& px1 = planes_a[1];
    const Plane& py0 = planes_b[0];
    const Plane& py1 = planes_b[1];

    Line cx0_py0__cy0_px0;
    std::pair<int,int> e0_0__e1_0;
    if (!polyline_plane_cross_joint(cx0, cy0, px0, py0, cx0_py0__cy0_px0, e0_0__e1_0)) return false;

    Line cx0_py1__cy1_px0;
    std::pair<int,int> e0_0__e1_1;
    if (!polyline_plane_cross_joint(cx0, cy1, px0, py1, cx0_py1__cy1_px0, e0_0__e1_1)) return false;

    Line cx1_py0__cy0_px1;
    std::pair<int,int> e0_1__e1_0;
    if (!polyline_plane_cross_joint(cx1, cy0, px1, py0, cx1_py0__cy0_px1, e0_1__e1_0)) return false;

    Line cx1_py1__cy1_px1;
    std::pair<int,int> e0_1__e1_1;
    if (!polyline_plane_cross_joint(cx1, cy1, px1, py1, cx1_py1__cy1_px1, e0_1__e1_1)) return false;

    // 3. Record side-face IDs (+2 because indices 0,1 are top/bottom).
    result.face_ids_a.first  = e0_0__e1_0.first + 2;
    result.face_ids_b.first  = e0_0__e1_0.second + 2;
    result.face_ids_a.second = e0_1__e1_1.first + 2;
    result.face_ids_b.second = e0_1__e1_1.second + 2;

    // 4. Sort segments to a common orientation (flip when angle to reference > 90°).
    Vector ref_v = segment_to_vector(cx0_py0__cy0_px0);
    {
        Vector v = segment_to_vector(cx0_py1__cy1_px0);
        if (approximate_angle_deg(ref_v, v) > approximate_angle_deg(ref_v, Vector(-v[0], -v[1], -v[2])))
            cx0_py1__cy1_px0 = opposite_segment(cx0_py1__cy1_px0);
    }
    {
        Vector v = segment_to_vector(cx1_py0__cy0_px1);
        if (approximate_angle_deg(ref_v, v) > approximate_angle_deg(ref_v, Vector(-v[0], -v[1], -v[2])))
            cx1_py0__cy0_px1 = opposite_segment(cx1_py0__cy0_px1);
    }
    {
        Vector v = segment_to_vector(cx1_py1__cy1_px1);
        if (approximate_angle_deg(ref_v, v) > approximate_angle_deg(ref_v, Vector(-v[0], -v[1], -v[2])))
            cx1_py1__cy1_px1 = opposite_segment(cx1_py1__cy1_px1);
    }

    // 5. Reference axis: midline of two contact segments, grown ×10 about its midpoint.
    Line c;
    get_middle_line(cx0_py1__cy1_px0, cx1_py0__cy0_px1, c);
    {
        double len = c.length();
        if (len < Tolerance::ZERO_TOLERANCE) return false;
        double pad = len * 4.5; // grow ×10 total: each end pushed out by 4.5×len
        extend_line(c, pad, pad);
    }

    Point c_start = c.start();
    Point c_end = c.end();

    // 6. Project all 8 segment endpoints onto c, sorted parameters give lMin (gap) and lMax (extent).
    auto project_t = [&](const Point& p) -> double {
        double t;
        Polyline::closest_point_to_line(p, c_start, c_end, t);
        return t;
    };

    double cpt0[4] = {
        project_t(cx0_py0__cy0_px0.start()),
        project_t(cx0_py1__cy1_px0.start()),
        project_t(cx1_py0__cy0_px1.start()),
        project_t(cx1_py1__cy1_px1.start())
    };
    std::sort(cpt0, cpt0 + 4);

    double cpt1[4] = {
        project_t(cx0_py0__cy0_px0.end()),
        project_t(cx0_py1__cy1_px0.end()),
        project_t(cx1_py0__cy0_px1.end()),
        project_t(cx1_py1__cy1_px1.end())
    };
    std::sort(cpt1, cpt1 + 4);

    double cpt[8] = { cpt0[0], cpt0[1], cpt0[2], cpt0[3], cpt1[0], cpt1[1], cpt1[2], cpt1[3] };
    std::sort(cpt, cpt + 8);

    Line lMin = Line::from_points(c.point_at(cpt0[3]), c.point_at(cpt1[0]));
    Line lMax = Line::from_points(c.point_at(cpt[0]),  c.point_at(cpt[7]));

    // 7. Mid plane through midpoint of lMin perpendicular to lMin.
    Point lMin_mid = lMin.center();
    Vector lMin_dir = lMin.to_vector();
    if (lMin_dir.magnitude() < Tolerance::ZERO_TOLERANCE) return false;
    Vector lMin_z = lMin_dir; lMin_z.normalize_self();
    // Build an orthonormal basis around lMin_z.
    Vector helper = (std::fabs(lMin_z[0]) < 0.9) ? Vector(1, 0, 0) : Vector(0, 1, 0);
    Vector mid_x = helper.cross(lMin_z); mid_x.normalize_self();
    Vector mid_y = lMin_z.cross(mid_x);  mid_y.normalize_self();
    Plane midPlane(lMin_mid, mid_x, mid_y, lMin_z);

    // 8. Extension vector v.
    Point midPlane_lMax;
    if (!line_plane(lMax, midPlane, midPlane_lMax, false)) return false;
    Point lMax_a = lMax.start();
    Point lMax_b = lMax.end();
    int maxID = ((lMax_b - midPlane_lMax).magnitude_squared() > (lMax_a - midPlane_lMax).magnitude_squared()) ? 1 : 0;
    Vector v = (maxID == 1)
        ? (lMax_b - midPlane_lMax)
        : Vector(-(lMax_a[0]-midPlane_lMax[0]), -(lMax_a[1]-midPlane_lMax[1]), -(lMax_a[2]-midPlane_lMax[2]));

    if (extension[2] > 0.0) {
        double length = v.magnitude();
        if (length > Tolerance::ZERO_TOLERANCE) {
            double target = length + extension[2];
            v = Vector(v[0]*target/length, v[1]*target/length, v[2]*target/length);
        }
    }

    // 9. joint_area = plane_4lines(midPlane, four sorted segments as infinite lines).
    if (!plane_4lines(midPlane,
                      cx0_py0__cy0_px0,
                      cx0_py1__cy1_px0,
                      cx1_py1__cy1_px1,
                      cx1_py0__cy0_px1,
                      result.joint_area)) return false;

    // 9b. Joint lines: two perpendicular centerlines of the joint area quad,
    //     crossing at its centroid. Semantically: "line on A through the joint"
    //     and "line on B through the joint".
    {
        const auto& jpts = result.joint_area.get_points(); // 5 pts: 4 corners + closure
        auto mid = [](const Point& p, const Point& q) {
            return Point((p[0]+q[0])*0.5, (p[1]+q[1])*0.5, (p[2]+q[2])*0.5);
        };
        Point m01 = mid(jpts[0], jpts[1]);
        Point m23 = mid(jpts[2], jpts[3]);
        Point m12 = mid(jpts[1], jpts[2]);
        Point m30 = mid(jpts[3], jpts[0]);
        result.joint_lines[0] = Polyline(std::vector<Point>{m01, m23});
        result.joint_lines[1] = Polyline(std::vector<Point>{m12, m30});
    }

    // 10. Joint volume faces = joint_area ± v.
    result.joint_volumes[0] = translate_quad(result.joint_area, v);
    result.joint_volumes[1] = translate_quad(result.joint_area, Vector(-v[0], -v[1], -v[2]));

    // 11. Optional in-plane edge extensions.
    if (extension[0] + extension[1] > 0.0) {
        for (int k = 0; k < 2; k++) {
            auto pts = result.joint_volumes[k].get_points();
            extend(pts, 0, extension[0], extension[0], 0.0, 0.0);
            extend(pts, 2, extension[0], extension[0], 0.0, 0.0);
            extend(pts, 1, extension[1], extension[1], 0.0, 0.0);
            extend(pts, 3, extension[1], extension[1], 0.0, 0.0);
            result.joint_volumes[k] = Polyline(pts);
        }
    }

    return true;
}

bool Intersection::plane_to_face(
    ElementPlate* a,
    ElementPlate* b,
    CrossJoint& result,
    double angle_tol,
    const std::array<double,3>& extension) {

    if (!a || !b) return false;
    auto polys_a = a->polylines();
    auto polys_b = b->polylines();
    auto planes_a = a->planes();
    auto planes_b = b->planes();
    if (polys_a.size() < 2 || polys_b.size() < 2) return false;
    if (planes_a.size() < 2 || planes_b.size() < 2) return false;

    std::array<Polyline,2> pa{ polys_a[0], polys_a[1] };
    std::array<Polyline,2> pb{ polys_b[0], polys_b[1] };
    std::array<Plane,2> npa{ planes_a[0], planes_a[1] };
    std::array<Plane,2> npb{ planes_b[0], planes_b[1] };
    return plane_to_face(pa, pb, npa, npb, result, angle_tol, extension);
}

} // namespace session_cpp
