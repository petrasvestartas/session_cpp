#include "intersection.h"
#include "nurbscurve.h"
#include "closest.h"
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
    const BoundingBox& box,
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
    const BoundingBox& box,
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
    const BoundingBox& box,
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

} // namespace session_cpp
