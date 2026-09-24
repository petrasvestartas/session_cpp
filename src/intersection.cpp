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
#include <limits>
#include <string_view>
#include <tuple>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Lines and planes
// ═══════════════════════════════════════════════════════════════════════════

/// Largest absolute coefficient of a 3x3 system with its row and column, the first one on ties.
static double max_pivot_3x3(const double row0[3], const double row1[3], const double row2[3], int& i, int& j) {

    const double* rows[3] = {row0, row1, row2};
    double temp = std::fabs(row0[0]);
    i = 0;
    j = 0;

    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            double val = std::fabs(rows[r][c]);

            if (val > temp) {
                temp = val;
                i = r;
                j = c;
            }
        }
    }

    return temp;
}

/// Rows of a 3x3 system in a 3x4 work array, row i swapped to the top.
static void load_rows_3x3(
    double w[12],
    const double row0[3],
    const double row1[3],
    const double row2[3],
    double d0,
    double d1,
    double d2,
    int i
) {

    const double* rows[3] = {row0, row1, row2};
    const double ds[3] = {d0, d1, d2};
    int src[3] = {0, 1, 2};
    std::swap(src[0], src[i]);

    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++)
            w[4 * r + c] = rows[src[r]][c];

        w[4 * r + 3] = ds[src[r]];
    }
}

/// Swap two coefficient columns in all rows of the 3x4 work array.
static void swap_columns(double w[12], int c0, int c1) {

    for (int r = 0; r < 3; r++)
        std::swap(w[4 * r + c0], w[4 * r + c1]);
}

/// Scale the top row to a unit pivot and clear the first column of the rows below.
static void eliminate_first_column(double w[12]) {

    double temp = 1.0 / w[0];
    w[1] *= temp;
    w[2] *= temp;
    w[3] *= temp;

    for (int r = 4; r <= 8; r += 4) {
        temp = -w[r];

        if (temp != 0.0)
            for (int c = 1; c < 4; c++)
                w[r + c] += temp * w[c];
    }
}

/// Largest absolute coefficient of the lower-right 2x2 block with its row and column, the first one on ties.
static double max_pivot_2x2(const double w[12], int& i, int& j) {

    double temp = std::fabs(w[5]);
    i = 0;
    j = 0;

    for (int r = 0; r < 2; r++) {
        for (int c = 0; c < 2; c++) {
            double val = std::fabs(w[5 + 4 * r + c]);

            if (val > temp) {
                temp = val;
                i = r;
                j = c;
            }
        }
    }

    return temp;
}

/// Widen the [minpiv, maxpiv] pivot range by val.
static void update_pivot_range(double val, double& maxpiv, double& minpiv) {

    if (val > maxpiv)
        maxpiv = val;
    else if (val < minpiv)
        minpiv = val;
}

/// Eliminate the second and third columns using the rows at offsets pivot and other; false when the last pivot is zero.
static bool eliminate_last_columns(double w[12], int pivot, int other, double& maxpiv, double& minpiv) {

    double temp = 1.0 / w[pivot + 1];
    w[pivot + 2] *= temp;
    w[pivot + 3] *= temp;
    temp = -w[1];

    if (temp != 0.0) {
        w[2] += temp * w[pivot + 2];
        w[3] += temp * w[pivot + 3];
    }

    temp = -w[other + 1];

    if (temp != 0.0) {
        w[other + 2] += temp * w[pivot + 2];
        w[other + 3] += temp * w[pivot + 3];
    }

    temp = w[other + 2];

    if (temp == 0.0)
        return false;

    update_pivot_range(std::fabs(temp), maxpiv, minpiv);
    w[other + 3] /= temp;
    temp = -w[pivot + 2];

    if (temp != 0.0)
        w[pivot + 3] += temp * w[other + 3];

    temp = -w[2];

    if (temp != 0.0)
        w[3] += temp * w[other + 3];

    return true;
}

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

    pivot_ratio = x = y = z = 0.0;
    int i;
    int j;
    double temp = max_pivot_3x3(row0, row1, row2, i, j);

    if (temp == 0.0)
        return 0;

    double maxpiv = std::fabs(temp);
    double minpiv = maxpiv;
    double w[12];
    int slot[3] = {0, 1, 2};
    load_rows_3x3(w, row0, row1, row2, d0, d1, d2, i);

    if (j != 0) {
        swap_columns(w, 0, j);
        std::swap(slot[0], slot[j]);
    }

    eliminate_first_column(w);
    temp = max_pivot_2x2(w, i, j);

    if (temp == 0.0)
        return 1;

    update_pivot_range(std::fabs(temp), maxpiv, minpiv);

    if (j != 0) {
        swap_columns(w, 1, 2);
        std::swap(slot[1], slot[2]);
    }

    const int pivot = i ? 8 : 4;
    const int other = i ? 4 : 8;

    if (!eliminate_last_columns(w, pivot, other, maxpiv, minpiv))
        return 2;

    double sol[3];
    sol[slot[0]] = w[3];
    sol[slot[1]] = w[pivot + 3];
    sol[slot[2]] = w[other + 3];
    x = sol[0];
    y = sol[1];
    z = sol[2];
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

/// Parameters (0 or 1) of an exactly shared endpoint of two segments; false when none is shared.
static bool shared_endpoint_parameters(const Line& line0, const Line& line1, double& t0, double& t1) {

    const std::array<Point, 2> ends0 = {line0.start(), line0.end()};
    const std::array<Point, 2> ends1 = {line1.start(), line1.end()};

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            if (ends0[i][0] == ends1[j][0] && ends0[i][1] == ends1[j][1] && ends0[i][2] == ends1[j][2]) {
                t0 = i;
                t1 = j;

                return true;
            }
        }
    }

    return false;
}

/// Clamp a parameter to [0, 1].
static double clamp_unit(double t) {

    if (t < 0.0)
        return 0.0;

    if (t > 1.0)
        return 1.0;

    return t;
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

    if (shared_endpoint_parameters(line0, line1, t0, t1))
        return true;

    const Vector A = line0.to_vector();
    const Vector B = line1.to_vector();
    const Vector C = line1.start() - line0.start();

    const double AA = A.dot(A);
    const double BB = B.dot(B);
    const double AB = A.dot(B);
    const double AC = A.dot(C);
    const double BC = B.dot(C);

    const double det = AA * BB - AB * AB;
    const double zero_tol = std::max(AA, BB) * std::numeric_limits<double>::epsilon();
    const bool parallel = std::fabs(det) < zero_tol;

    if (parallel && !near_parallel_as_closest)
        return false;

    if (parallel) {
        t0 = (AA > 0.0) ? (AC / AA) : 0.0;
        t1 = (BB > 0.0) ? ((BC + t0 * AB) / BB) : 0.0;
    } else {
        const double inv_det = 1.0 / det;
        t0 = (BB * AC - AB * BC) * inv_det;
        t1 = (AB * AC - AA * BC) * inv_det;
    }

    if (intersect_segments) {
        t0 = clamp_unit(t0);
        t1 = clamp_unit(t1);
    }

    if (tolerance > 0.0)
        return line0.point_at(t0).distance(line1.point_at(t1)) <= tolerance;

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

    const Vector offset = origin - center;
    const double a = direction.dot(direction);
    const double b = 2.0 * direction.dot(offset);
    const double c = offset.dot(offset) - (radius * radius);
    const double disc = b * b - 4.0 * a * c;

    if (disc < 0.0)
        return 0;

    const double root = std::sqrt(disc);
    const double q = (b < 0.0) ? (-b - root) / 2.0 : (-b + root) / 2.0;

    t0 = q / a;
    t1 = c / q;

    if (t1 == t0)
        return 1;

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
    intersection_points.push_back(origin + direction * t0);

    if (hits == 2)
        intersection_points.push_back(origin + direction * t1);

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

/// Sorts hits by t and keeps only the nearest unless find_all; false when there is none.
static bool sort_ray_hits(std::vector<Intersection::RayHit>& hits, bool find_all) {

    if (hits.empty())
        return false;

    std::stable_sort(hits.begin(), hits.end(), ray_hit_before);

    if (!find_all)
        hits.resize(1);

    return true;
}

bool Intersection::ray_mesh(
    const Point& origin,
    const Vector& direction,
    const Mesh& mesh,
    std::vector<RayHit>& hits,
    bool find_all,
    double epsilon
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

            if (!ray_triangle(origin, direction, v0, v1, v2, epsilon, t, u, v, parallel) || t < 0.0)
                continue;

            hits.emplace_back(t, origin + direction * t, u, v, static_cast<int>(i));
        }
    }

    return sort_ray_hits(hits, find_all);
}

bool Intersection::ray_mesh_bvh(
    const Point& origin,
    const Vector& direction,
    const Mesh& mesh,
    std::vector<RayHit>& hits,
    bool find_all,
    double epsilon
) {

    hits.clear();

    std::vector<int> candidates;

    if (!mesh.triangle_bvh_ray_cast(origin, direction, candidates, find_all))
        return false;

    for (int tri_id : candidates) {
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

        if (!ray_triangle(origin, direction, v0, v1, v2, epsilon, t, u, v, parallel) || t < 0.0)
            continue;

        hits.emplace_back(t, origin + direction * t, u, v, static_cast<int>(face_idx));
    }

    return sort_ray_hits(hits, find_all);
}

std::vector<Point> Intersection::ray_mesh(const Line& line, const Mesh& mesh, double epsilon, bool find_all) {

    std::vector<RayHit> hits;
    std::vector<Point> result;

    if (!ray_mesh(line.start(), line.to_vector(), mesh, hits, find_all, epsilon))
        return result;

    for (const RayHit& hit : hits)
        result.push_back(hit.point);

    return result;
}

std::vector<Point> Intersection::ray_mesh_bvh(const Line& line, const Mesh& mesh, double epsilon, bool find_all) {

    std::vector<RayHit> hits;
    std::vector<Point> result;

    if (!ray_mesh_bvh(line.start(), line.to_vector(), mesh, hits, find_all, epsilon))
        return result;

    for (const RayHit& hit : hits)
        result.push_back(hit.point);

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// NURBS curve helpers
// ═══════════════════════════════════════════════════════════════════════════

namespace {

/// Sorted values without neighbours closer than tolerance to the last kept one.
std::vector<double> unique_sorted(const std::vector<double>& values, double tolerance) {

    std::vector<double> unique;

    for (double value : values)
        if (unique.empty() || std::abs(unique.back() - value) >= tolerance)
            unique.push_back(value);

    return unique;
}

/// Signed distance of a point to the plane.
double curve_signed_distance_to_plane(const Point& pt, const Plane& plane) {

    Vector v = pt - plane.origin();

    return v.dot(plane.z_axis());
}

/// Rate of change of the signed plane distance with the curve parameter.
double curve_plane_slope(const NurbsCurve& curve, const Plane& plane, double t) {

    const std::vector<Vector> derivs = curve.evaluate(t, 1);

    return derivs[1].dot(plane.z_axis());
}

/// Appends t unless a value within tolerance is already present.
void append_unique(std::vector<double>& values, double t, double tolerance) {

    for (double existing : values)
        if (std::abs(existing - t) < tolerance)
            return;

    values.push_back(t);
}

/// Newton for the plane crossing in [a, b] from the midpoint, bisecting whenever a step is flat or leaves the bracket.
bool curve_plane_newton_bracket(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance,
    double a,
    double b,
    double& t
) {

    double f_a = curve_signed_distance_to_plane(curve.point_at(a), plane);
    t = (a + b) * 0.5;

    for (int iter = 0; iter < 10; iter++) {
        const double f = curve_signed_distance_to_plane(curve.point_at(t), plane);

        if (std::abs(f) < tolerance)
            return true;

        const double df = curve_plane_slope(curve, plane, t);
        const bool flat = std::abs(df) < 1e-14;
        const double t_new = flat ? t : t - f / df;

        if (flat || t_new < a || t_new > b) {
            if (f * f_a < 0) {
                b = t;
            } else {
                a = t;
                f_a = f;
            }

            t = (a + b) * 0.5;
            continue;
        }

        if (std::abs(t_new - t) < tolerance) {
            t = t_new;

            return true;
        }

        t = t_new;
    }

    return false;
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

        double f = curve_signed_distance_to_plane(pt, plane);
        double df = curve_plane_slope(curve, plane, t);

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

                double f = curve_signed_distance_to_plane(pt, plane);
                double df = curve_plane_slope(curve, plane, t);

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
        double t;

        if (curve_plane_newton_bracket(curve, plane, tolerance, a, b, t) && t >= a && t <= b)
            append_unique(results, t, tolerance * 10.0);
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
        double t;

        if (curve_plane_newton_bracket(curve, plane, tolerance, a, b, t) && t >= a && t <= b)
            append_unique(results, t, tolerance * 10.0);
    } else {
        const double mid = (a + b) * 0.5;
        curve_plane_subdivide_production(curve, plane, tolerance, a, mid, depth + 1, results);
        curve_plane_subdivide_production(curve, plane, tolerance, mid, b, depth + 1, results);
    }
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// NURBS curves
// ═══════════════════════════════════════════════════════════════════════════

/// Appends t unless it lies within tolerance of the last parameter.
static void append_parameter(std::vector<double>& params, double t, double tolerance) {

    if (params.empty() || std::abs(params.back() - t) >= tolerance)
        params.push_back(t);
}

/// Crossing pairs hidden inside a span whose ends lie on one side, found on degree * 2 sub-intervals.
static void curve_plane_hidden_pairs(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance,
    double t0,
    double t1,
    std::vector<double>& intersections
) {

    const int count = curve.degree() * 2;
    const double dt = (t1 - t0) / count;

    for (int i = 0; i < count; i++) {
        const double s0 = t0 + i * dt;
        const double s1 = t0 + (i + 1) * dt;
        const double d0 = curve_signed_distance_to_plane(curve.point_at(s0), plane);
        const double d1 = curve_signed_distance_to_plane(curve.point_at(s1), plane);
        double t_intersection;

        if (d0 * d1 < 0 && curve_find_root_bisection(curve, plane, s0, s1, tolerance, t_intersection)) {
            curve_refine_intersection_newton(curve, plane, t_intersection, tolerance);
            intersections.push_back(t_intersection);
        }
    }
}

/// Crossings inside each knot span, plus span starts and the curve end lying on the plane.
static void curve_plane_spans(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance,
    std::vector<double>& intersections
) {

    const std::vector<double> span_params = curve.get_span_vector();

    for (size_t i = 0; i < span_params.size() - 1; i++) {
        const double t0 = span_params[i];
        const double t1 = span_params[i + 1];

        if (std::abs(t1 - t0) < tolerance)
            continue;

        const double d0 = curve_signed_distance_to_plane(curve.point_at(t0), plane);
        const double d1 = curve_signed_distance_to_plane(curve.point_at(t1), plane);
        double t_intersection;

        if (d0 * d1 < 0) {
            if (curve_find_root_bisection(curve, plane, t0, t1, tolerance, t_intersection)) {
                curve_refine_intersection_newton(curve, plane, t_intersection, tolerance);
                intersections.push_back(t_intersection);
            }
        } else if (std::abs(d0) < tolerance) {
            append_parameter(intersections, t0, tolerance);
        } else if (curve.degree() > 1) {
            curve_plane_hidden_pairs(curve, plane, tolerance, t0, t1, intersections);
        }
    }

    const double t_end = curve.domain().second;

    if (std::abs(curve_signed_distance_to_plane(curve.point_at(t_end), plane)) < tolerance)
        append_parameter(intersections, t_end, tolerance);
}

/// Extra crossings of a high-degree curve found on degree * 4 uniform samples.
static void curve_plane_samples(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance,
    std::vector<double>& intersections
) {

    const std::pair<double, double> domain = curve.domain();
    const int num_samples = curve.degree() * 4;
    const double dt = (domain.second - domain.first) / num_samples;

    for (int i = 0; i < num_samples; i++) {
        const double t0 = domain.first + i * dt;
        const double t1 = domain.first + (i + 1) * dt;
        const double d0 = curve_signed_distance_to_plane(curve.point_at(t0), plane);
        const double d1 = curve_signed_distance_to_plane(curve.point_at(t1), plane);
        double t_intersection;
        const bool crossing = d0 * d1 < 0 && curve_find_root_bisection(curve, plane, t0, t1, tolerance, t_intersection);

        if (!crossing)
            continue;

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

std::vector<double> Intersection::curve_plane(const NurbsCurve& curve, const Plane& plane, double tolerance) {

    std::vector<double> intersections;

    if (!curve.is_valid())
        return intersections;

    if (tolerance <= 0.0)
        tolerance = Tolerance::ZERO_TOLERANCE;

    curve_plane_spans(curve, plane, tolerance, intersections);

    if (curve.degree() > 3 && intersections.size() < static_cast<size_t>(curve.degree()))
        curve_plane_samples(curve, plane, tolerance, intersections);

    std::sort(intersections.begin(), intersections.end());

    return unique_sorted(intersections, tolerance * 2.0);
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

    return unique_sorted(results, tolerance * 10.0);
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

    return unique_sorted(results, tolerance * 10.0);
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

    return unique_sorted(results, tolerance * 10.0);
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

/// Grid crossing of the surface-plane distance, the start of one trace.
struct SurfacePlaneSeed {
    double u; // Seed u.
    double v; // Seed v.
    bool used; // Whether a trace already passed the seed.
};

/// Signed surface-plane distance over the surface's UV domain with the tracing scales.
class SurfacePlaneField {
public:
    const NurbsSurface& surface; // Traced surface.
    Vector pn; // Plane normal.
    Point p0; // Plane origin.
    double tolerance; // Newton tolerance.
    double u0; // Domain start in u.
    double u1; // Domain end in u.
    double v0; // Domain start in v.
    double v1; // Domain end in v.
    double range_u; // Domain length in u.
    double range_v; // Domain length in v.
    bool closed_u; // Whether u wraps around a seam.
    bool closed_v; // Whether v wraps around a seam.
    int nu; // Grid cells in u.
    int nv; // Grid cells in v.
    double du; // Grid cell size in u.
    double dv; // Grid cell size in v.
    double uv_to_3d; // Largest uv-to-3D scale.
    double uv_to_3d_min; // Smallest uv-to-3D scale.
    double step; // Marching step in uv.
    int max_steps; // Marching step cap per direction.
    double close_tol_3d; // 3D distance that closes a loop.
    double consume_tol_3d; // 3D distance that consumes a seed.
    double join_tol; // 3D distance that joins two traces.

    /// Sample the domain and derive the tracing scales.
    SurfacePlaneField(const NurbsSurface& surface_, const Plane& plane, double tolerance_);

    /// Wrap u across a closed seam or clamp it to the domain.
    double wrap_u(double u) const;

    /// Wrap v across a closed seam or clamp it to the domain.
    double wrap_v(double v) const;

    /// Signed plane distance at (u, v).
    double value(double u, double v) const;

    /// Signed plane distance and its uv gradient at (u, v).
    void value_and_gradient(double u, double v, double& val, double& gu, double& gv) const;

    /// Newton-project (u, v) onto the zero set; false when it does not converge.
    bool newton_correct(double& u, double& v) const;

    /// Unit uv tangent of the zero set at (u, v) in direction dir.
    bool tangent(double u, double v, int dir, double& tu, double& tv) const;

    /// Surface point at a uv sample.
    Point point(const std::pair<double, double>& q) const;

    /// Newton-slide (cu, cv) along one seam line, axis 0 moving v and axis 1 moving u.
    std::pair<double, double> seam_newton(double cu, double cv, int axis) const;

    /// Newton-project (u, v) onto the zero set to 1e-12; false on a flat gradient.
    bool polish(double& u, double& v) const;
};

SurfacePlaneField::SurfacePlaneField(const NurbsSurface& surface_, const Plane& plane, double tolerance_)
    : surface(surface_), pn(plane.z_axis()), p0(plane.origin()), tolerance(tolerance_) {

    const std::pair<double, double> domain_u = surface.domain(0);
    u0 = domain_u.first;
    u1 = domain_u.second;
    const std::pair<double, double> domain_v = surface.domain(1);
    v0 = domain_v.first;
    v1 = domain_v.second;
    range_u = u1 - u0;
    range_v = v1 - v0;
    closed_u = surface.is_closed(0);
    closed_v = surface.is_closed(1);

    const std::vector<double> spans_u = surface.get_span_vector(0);
    const std::vector<double> spans_v = surface.get_span_vector(1);
    nu = std::max((int)spans_u.size() - 1, 1) * 4;
    nv = std::max((int)spans_v.size() - 1, 1) * 4;
    du = range_u / nu;
    dv = range_v / nv;

    const double mu = (u0 + u1) * 0.5;
    const double mv = (v0 + v1) * 0.5;
    const Point pmid = point({mu, mv});
    const double uv_to_3d_u = pmid.distance(point({wrap_u(mu + du), mv})) / du;
    const double uv_to_3d_v = pmid.distance(point({mu, wrap_v(mv + dv)})) / dv;
    uv_to_3d = std::max(uv_to_3d_u, uv_to_3d_v);
    uv_to_3d_min = std::min(uv_to_3d_u, uv_to_3d_v);

    if (uv_to_3d < 1e-10)
        uv_to_3d = 1.0;

    if (uv_to_3d_min < 1e-10)
        uv_to_3d_min = 1.0;

    step = std::min(du, dv) * 0.25;
    max_steps = nu * nv * 32;
    close_tol_3d = step * 4.0 * uv_to_3d_min;
    consume_tol_3d = step * uv_to_3d * 2.0;
    join_tol = std::max(du, dv) * uv_to_3d * 1.5;
}

double SurfacePlaneField::wrap_u(double u) const {

    if (closed_u) {
        double t = std::fmod(u - u0, range_u);

        if (t < 0)
            t += range_u;

        return u0 + t;
    }

    return std::max(u0, std::min(u, u1));
}

double SurfacePlaneField::wrap_v(double v) const {

    if (closed_v) {
        double t = std::fmod(v - v0, range_v);

        if (t < 0)
            t += range_v;

        return v0 + t;
    }

    return std::max(v0, std::min(v, v1));
}

double SurfacePlaneField::value(double u, double v) const {

    const Point p = point({wrap_u(u), wrap_v(v)});

    return (p[0] - p0[0]) * pn[0] + (p[1] - p0[1]) * pn[1] + (p[2] - p0[2]) * pn[2];
}

void SurfacePlaneField::value_and_gradient(double u, double v, double& val, double& gu, double& gv) const {

    const std::vector<Vector> derivs = surface.evaluate(wrap_u(u), wrap_v(v), 1);
    const Vector& S = derivs[0];
    const Vector& Su = derivs[2];
    const Vector& Sv = derivs[1];
    val = (S[0] - p0[0]) * pn[0] + (S[1] - p0[1]) * pn[1] + (S[2] - p0[2]) * pn[2];
    gu = Su[0] * pn[0] + Su[1] * pn[1] + Su[2] * pn[2];
    gv = Sv[0] * pn[0] + Sv[1] * pn[1] + Sv[2] * pn[2];
}

bool SurfacePlaneField::newton_correct(double& u, double& v) const {

    for (int iter = 0; iter < 10; iter++) {
        double val;
        double gu;
        double gv;
        value_and_gradient(u, v, val, gu, gv);

        if (std::abs(val) < tolerance)
            return true;

        const double mag2 = gu * gu + gv * gv;

        if (mag2 < 1e-28)
            return false;

        u -= val * gu / mag2;
        v -= val * gv / mag2;
        u = wrap_u(u);
        v = wrap_v(v);
    }

    return std::abs(value(u, v)) < tolerance * 10.0;
}

bool SurfacePlaneField::tangent(double u, double v, int dir, double& tu, double& tv) const {

    double val;
    double gu;
    double gv;
    value_and_gradient(u, v, val, gu, gv);
    const double mag = std::hypot(gu, gv);

    if (mag < 1e-14)
        return false;

    tu = -gv / mag * dir;
    tv = gu / mag * dir;

    return true;
}

Point SurfacePlaneField::point(const std::pair<double, double>& q) const {
    return surface.point_at(q.first, q.second);
}

std::pair<double, double> SurfacePlaneField::seam_newton(double cu, double cv, int axis) const {

    for (int iter = 0; iter < 10; iter++) {
        double val;
        double gu;
        double gv;
        value_and_gradient(cu, cv, val, gu, gv);

        if (std::abs(val) < tolerance)
            break;

        if (axis == 0) {
            if (std::abs(gv) < 1e-14)
                break;

            cv = cv - val / gv;
        } else {
            if (std::abs(gu) < 1e-14)
                break;

            cu = cu - val / gu;
        }
    }

    return {cu, cv};
}

bool SurfacePlaneField::polish(double& u, double& v) const {

    for (int iter = 0; iter < 8; iter++) {
        double val;
        double gu;
        double gv;
        value_and_gradient(u, v, val, gu, gv);

        if (std::abs(val) < 1e-12)
            return true;

        const double mag2 = gu * gu + gv * gv;

        if (mag2 < 1e-28)
            return false;

        u -= val * gu / mag2;
        v -= val * gv / mag2;
    }

    return true;
}

/// Signed plane distance on the (nu + 1) x (nv + 1) grid, exact zeros nudged negative.
std::vector<double> surface_plane_grid(const SurfacePlaneField& field) {

    const int cols = field.nv + 1;
    std::vector<double> dist((field.nu + 1) * cols);

    for (int i = 0; i <= field.nu; i++) {
        const double u = field.u0 + field.du * i;

        for (int j = 0; j <= field.nv; j++) {
            const double v = field.v0 + field.dv * j;
            double d = field.value(u, v);

            if (d == 0.0)
                d = -1e-14;

            dist[i * cols + j] = d;
        }
    }

    return dist;
}

/// Newton-corrected sign changes along the grid edges, near duplicates marked used.
std::vector<SurfacePlaneSeed> surface_plane_seeds(const SurfacePlaneField& field, const std::vector<double>& dist) {

    std::vector<SurfacePlaneSeed> seeds;
    const int cols = field.nv + 1;
    const int h_jmax = field.closed_v ? field.nv - 1 : field.nv;

    for (int i = 0; i < field.nu; i++) {
        for (int j = 0; j <= h_jmax; j++) {
            const double d0 = dist[i * cols + j];
            const double d1 = dist[(i + 1) * cols + j];

            if (d0 * d1 < 0) {
                const double t = d0 / (d0 - d1);
                double su = field.u0 + field.du * (i + t);
                double sv = field.v0 + field.dv * j;

                if (field.newton_correct(su, sv))
                    seeds.push_back({su, sv, false});
            }
        }
    }

    const int v_imax = field.closed_u ? field.nu - 1 : field.nu;

    for (int i = 0; i <= v_imax; i++) {
        for (int j = 0; j < field.nv; j++) {
            const double d0 = dist[i * cols + j];
            const double d1 = dist[i * cols + j + 1];

            if (d0 * d1 < 0) {
                const double t = d0 / (d0 - d1);
                double su = field.u0 + field.du * i;
                double sv = field.v0 + field.dv * (j + t);

                if (field.newton_correct(su, sv))
                    seeds.push_back({su, sv, false});
            }
        }
    }

    const double seed_tol_3d = std::max(field.du, field.dv) * field.uv_to_3d;

    for (size_t i = 0; i < seeds.size(); i++) {
        if (seeds[i].used)
            continue;

        const Point pi = field.point({seeds[i].u, seeds[i].v});

        for (size_t j = i + 1; j < seeds.size(); j++) {
            if (seeds[j].used)
                continue;

            if (pi.distance(field.point({seeds[j].u, seeds[j].v})) < seed_tol_3d)
                seeds[j].used = true;
        }
    }

    return seeds;
}

/// Step (u, v) by local_step along (tu, tv), pulled back onto an open domain boundary; true when clamped.
bool domain_step(
    const SurfacePlaneField& field,
    double u,
    double v,
    double local_step,
    double tu,
    double tv,
    double& un,
    double& vn
) {

    un = u + local_step * tu;
    vn = v + local_step * tv;

    const bool out_u = !field.closed_u && (un < field.u0 || un > field.u1);
    const bool out_v = !field.closed_v && (vn < field.v0 || vn > field.v1);

    if (!out_u && !out_v)
        return false;

    double tc = 1.0;

    if (!field.closed_u && tu > 0 && un > field.u1)
        tc = std::min(tc, (field.u1 - u) / (local_step * tu));

    if (!field.closed_u && tu < 0 && un < field.u0)
        tc = std::min(tc, (field.u0 - u) / (local_step * tu));

    if (!field.closed_v && tv > 0 && vn > field.v1)
        tc = std::min(tc, (field.v1 - v) / (local_step * tv));

    if (!field.closed_v && tv < 0 && vn < field.v0)
        tc = std::min(tc, (field.v0 - v) / (local_step * tv));

    un = u + tc * local_step * tu;
    vn = v + tc * local_step * tv;

    return true;
}

/// Retry a failed Newton projection with the step halved up to four times.
bool newton_retry(
    const SurfacePlaneField& field,
    double u,
    double v,
    double local_step,
    double tu,
    double tv,
    double& un,
    double& vn
) {

    double ls = local_step;

    for (int rh = 0; rh < 4; ++rh) {
        ls *= 0.5;
        un = field.wrap_u(u + ls * tu);
        vn = field.wrap_v(v + ls * tv);

        if (field.newton_correct(un, vn))
            return true;
    }

    return false;
}

/// Mark every unused seed within the consume distance of p as used.
void consume_seeds(const SurfacePlaneField& field, const Point& p, std::vector<SurfacePlaneSeed>& seeds) {

    for (SurfacePlaneSeed& other : seeds)
        if (!other.used && p.distance(field.point({other.u, other.v})) < field.consume_tol_3d)
            other.used = true;
}

/// Step length for the turn between two unit tangents: a quarter or half step on sharp turns.
double turn_step(const SurfacePlaneField& field, double tu, double tv, double prev_tu, double prev_tv) {

    if (std::hypot(prev_tu, prev_tv) <= 1e-14)
        return field.step;

    double dot = tu * prev_tu + tv * prev_tv;
    dot = std::max(-1.0, std::min(1.0, dot));

    if (dot < 0.95)
        return field.step * 0.25;

    if (dot < 0.985)
        return field.step * 0.5;

    return field.step;
}

/// March the zero set from (su, sv) in direction dir; true when it closes on its start.
bool surface_plane_march(
    const SurfacePlaneField& field,
    double su,
    double sv,
    int dir,
    std::vector<SurfacePlaneSeed>& seeds,
    std::vector<std::pair<double, double>>& out
) {

    double u = su;
    double v = sv;
    double prev_tu = 0;
    double prev_tv = 0;
    const Point p_start = field.point({su, sv});
    Point p_prev = p_start;
    double dist_traveled = 0;

    for (int s = 0; s < field.max_steps; s++) {
        double tu;
        double tv;

        if (!field.tangent(u, v, dir, tu, tv)) {
            if (std::hypot(prev_tu, prev_tv) < 1e-14)
                break;

            tu = prev_tu;
            tv = prev_tv;
        }

        const double local_step = turn_step(field, tu, tv, prev_tu, prev_tv);
        double tu2;
        double tv2;

        if (field.tangent(u + local_step * 0.5 * tu, v + local_step * 0.5 * tv, dir, tu2, tv2)) {
            tu = tu2;
            tv = tv2;
        }

        prev_tu = tu;
        prev_tv = tv;

        double un;
        double vn;
        const bool hit_boundary = domain_step(field, u, v, local_step, tu, tv, un, vn);
        un = field.wrap_u(un);
        vn = field.wrap_v(vn);

        if (!field.newton_correct(un, vn) && !newton_retry(field, u, v, local_step, tu, tv, un, vn))
            break;

        const Point p_cur = field.point({un, vn});
        dist_traveled += p_prev.distance(p_cur);
        out.push_back({un, vn});

        if (dist_traveled > field.close_tol_3d * 3.0 && p_start.distance(p_cur) < field.close_tol_3d)
            return true;

        u = un;
        v = vn;
        p_prev = p_cur;

        if (hit_boundary)
            break;

        consume_seeds(field, p_cur, seeds);
    }

    return false;
}

/// Undo the seam jumps of a closed domain in the unwrapped copy of a trace.
void unwrap_trace(const SurfacePlaneField& field, std::vector<std::pair<double, double>>& uv) {

    for (size_t i = 1; i < uv.size(); i++) {
        const double du_jump = uv[i].first - uv[i - 1].first;
        const double dv_jump = uv[i].second - uv[i - 1].second;

        if (field.closed_u) {
            if (du_jump > field.range_u * 0.5)
                uv[i].first -= field.range_u;
            else if (du_jump < -field.range_u * 0.5)
                uv[i].first += field.range_u;
        }

        if (field.closed_v) {
            if (dv_jump > field.range_v * 0.5)
                uv[i].second -= field.range_v;
            else if (dv_jump < -field.range_v * 0.5)
                uv[i].second += field.range_v;
        }
    }
}

/// Trace one seed both ways into a trace; false when it is too short to keep.
bool surface_plane_trace_seed(
    const SurfacePlaneField& field,
    std::vector<SurfacePlaneSeed>& seeds,
    size_t index,
    SurfacePlaneTrace& trace
) {

    const double seed_u = seeds[index].u;
    const double seed_v = seeds[index].v;
    std::vector<std::pair<double, double>> fwd;
    std::vector<std::pair<double, double>> bwd;
    const bool fwd_closed = surface_plane_march(field, seed_u, seed_v, +1, seeds, fwd);

    if (!fwd_closed)
        surface_plane_march(field, seed_u, seed_v, -1, seeds, bwd);

    std::vector<std::pair<double, double>> uv_trace;
    uv_trace.reserve(bwd.size() + 1 + fwd.size());

    for (int i = (int)bwd.size() - 1; i >= 0; i--)
        uv_trace.push_back(bwd[i]);

    uv_trace.push_back({seed_u, seed_v});

    for (const std::pair<double, double>& p : fwd)
        uv_trace.push_back(p);

    if (uv_trace.size() < 4)
        return false;

    const Point p_first = field.point(uv_trace.front());
    const Point p_last = field.point(uv_trace.back());
    const bool is_loop = fwd_closed || (uv_trace.size() >= 6 && p_first.distance(p_last) < field.close_tol_3d);

    if (is_loop)
        uv_trace.pop_back();

    if (uv_trace.size() < 4)
        return false;

    std::vector<std::pair<double, double>> uv_unwrapped = uv_trace;
    unwrap_trace(field, uv_unwrapped);
    trace = {std::move(uv_trace), std::move(uv_unwrapped), is_loop};

    return true;
}

/// Whether every eighth sample of trace a lies within the join distance of trace b.
bool trace_covered_by(const SurfacePlaneField& field, const SurfacePlaneTrace& a, const SurfacePlaneTrace& b) {

    const size_t stride = std::max<size_t>(1, a.uv_trace.size() / 8);

    for (size_t k = 0; k < a.uv_trace.size(); k += stride) {
        const Point q = field.point(a.uv_trace[k]);
        double best = 1e300;

        for (const std::pair<double, double>& r : b.uv_trace)
            best = std::min(best, q.distance(field.point(r)));

        if (best > field.join_tol)
            return false;
    }

    return true;
}

/// Empty every open trace that a trace at least as long already covers.
void drop_covered_traces(const SurfacePlaneField& field, std::vector<SurfacePlaneTrace>& traces) {

    for (size_t i = 0; i < traces.size(); ++i) {
        if (traces[i].uv_trace.empty() || traces[i].is_loop)
            continue;

        for (size_t j = 0; j < traces.size(); ++j) {
            if (i == j || traces[j].uv_trace.empty())
                continue;

            if (traces[j].uv_trace.size() < traces[i].uv_trace.size())
                continue;

            if (trace_covered_by(field, traces[i], traces[j])) {
                traces[i].uv_trace.clear();
                break;
            }
        }
    }
}

/// Append trace b to the end of trace a, reversed when requested, and close a when it meets itself.
void append_trace(const SurfacePlaneField& field, SurfacePlaneTrace& a, SurfacePlaneTrace& b, bool reversed) {

    std::vector<std::pair<double, double>> add = b.uv_trace;

    if (reversed)
        std::reverse(add.begin(), add.end());

    a.uv_trace.insert(a.uv_trace.end(), add.begin(), add.end());
    b.uv_trace.clear();

    if (field.point(a.uv_trace.front()).distance(field.point(a.uv_trace.back())) < field.join_tol) {
        a.is_loop = true;
        a.uv_trace.pop_back();
    }

    a.uv_unwrapped = a.uv_trace;
    unwrap_trace(field, a.uv_unwrapped);
}

/// Join the first open trace pair whose end meets a start or end; false when none does.
bool join_one_trace_pair(const SurfacePlaneField& field, std::vector<SurfacePlaneTrace>& traces) {

    for (size_t i = 0; i < traces.size(); ++i) {
        if (traces[i].uv_trace.size() < 2 || traces[i].is_loop)
            continue;

        const Point ie = field.point(traces[i].uv_trace.back());

        for (size_t j = 0; j < traces.size(); ++j) {
            if (i == j || traces[j].uv_trace.size() < 2 || traces[j].is_loop)
                continue;

            const Point ja = field.point(traces[j].uv_trace.front());
            const Point jb = field.point(traces[j].uv_trace.back());
            const bool fwd2 = ie.distance(ja) < field.join_tol;
            const bool rev2 = ie.distance(jb) < field.join_tol;

            if (!fwd2 && !rev2)
                continue;

            append_trace(field, traces[i], traces[j], rev2);

            return true;
        }
    }

    return false;
}

/// Drop short traces and close the open ones whose ends meet.
void close_traces(const SurfacePlaneField& field, std::vector<SurfacePlaneTrace>& traces) {

    std::vector<SurfacePlaneTrace> kept;

    for (SurfacePlaneTrace& t : traces)
        if (t.uv_trace.size() >= 4)
            kept.push_back(std::move(t));

    traces = std::move(kept);

    for (SurfacePlaneTrace& t : traces) {
        if (t.is_loop || t.uv_trace.size() < 6)
            continue;

        if (field.point(t.uv_trace.front()).distance(field.point(t.uv_trace.back())) < field.join_tol) {
            t.is_loop = true;
            t.uv_trace.pop_back();
            t.uv_unwrapped.pop_back();
        }
    }
}

/// Snap one open trace end within a grid cell of the domain boundary onto it.
void snap_trace_end(const SurfacePlaneField& field, std::pair<double, double>& q, std::pair<double, double>& qu) {

    if (!field.closed_u) {
        if (std::abs(q.first - field.u0) < field.du) {
            q.first = field.u0;
            qu.first = field.u0;
        }

        if (std::abs(q.first - field.u1) < field.du) {
            q.first = field.u1;
            qu.first = field.u1;
        }
    } else {
        if (q.first - field.u0 < field.du)
            q.first = field.u0;
        else if (field.u1 - q.first < field.du)
            q.first = field.u1;
    }

    if (!field.closed_v) {
        if (std::abs(q.second - field.v0) < field.dv) {
            q.second = field.v0;
            qu.second = field.v0;
        }

        if (std::abs(q.second - field.v1) < field.dv) {
            q.second = field.v1;
            qu.second = field.v1;
        }
    } else {
        if (q.second - field.v0 < field.dv)
            q.second = field.v0;
        else if (field.v1 - q.second < field.dv)
            q.second = field.v1;
    }
}

/// Seed and trace surface/plane intersection curves in UV space.
SurfacePlaneTraceResult surface_plane_traces(const NurbsSurface& surface, const Plane& plane, double tolerance) {

    const SurfacePlaneField field(surface, plane, tolerance);
    const std::vector<double> dist = surface_plane_grid(field);

    double gmax = 0.0;

    for (double d : dist)
        gmax = std::max(gmax, std::abs(d));

    if (gmax < std::max(tolerance, 1e-9) * 10.0)
        return {{}, field.step, field.uv_to_3d, field.uv_to_3d_min};

    std::vector<SurfacePlaneSeed> seeds = surface_plane_seeds(field, dist);
    std::vector<SurfacePlaneTrace> traces;

    for (size_t i = 0; i < seeds.size(); ++i) {
        if (seeds[i].used)
            continue;

        seeds[i].used = true;
        SurfacePlaneTrace trace;

        if (surface_plane_trace_seed(field, seeds, i, trace))
            traces.push_back(std::move(trace));
    }

    drop_covered_traces(field, traces);

    for (size_t pass = 0; pass < traces.size(); pass++)
        if (!join_one_trace_pair(field, traces))
            break;

    close_traces(field, traces);

    for (SurfacePlaneTrace& t : traces) {
        if (t.is_loop || t.uv_trace.empty())
            continue;

        snap_trace_end(field, t.uv_trace.front(), t.uv_unwrapped.front());
        snap_trace_end(field, t.uv_trace.back(), t.uv_unwrapped.back());
    }

    return {std::move(traces), field.step, field.uv_to_3d, field.uv_to_3d_min};
}

/// Points projected into the plane's 2D frame, z = 0.
std::vector<Point> plane_points_2d(const std::vector<Point>& pts, const Plane& plane) {

    const Vector ax = plane.x_axis();
    const Vector ay = plane.y_axis();
    const Point po = plane.origin();
    std::vector<Point> pts_2d(pts.size());

    for (size_t i = 0; i < pts.size(); i++) {
        const double dx = pts[i][0] - po[0];
        const double dy = pts[i][1] - po[1];
        const double dz = pts[i][2] - po[2];
        const double px = dx * ax[0] + dy * ax[1] + dz * ax[2];
        const double py = dx * ay[0] + dy * ay[1] + dz * ay[2];
        pts_2d[i] = Point(px, py, 0);
    }

    return pts_2d;
}

/// Normalized cumulative chord length of each point, the closing chord included for loops.
std::vector<double> chord_parameters(const std::vector<Point>& pts, bool is_loop) {

    const size_t m = pts.size();
    std::vector<double> chords(m, 0.0);
    double total_len = 0;

    for (size_t i = 1; i < m; i++) {
        total_len += pts[i].distance(pts[i - 1]);
        chords[i] = total_len;
    }

    if (is_loop && m > 1)
        total_len += pts[0].distance(pts[m - 1]);

    if (total_len > 1e-14)
        for (size_t i = 1; i < m; i++)
            chords[i] /= total_len;

    return chords;
}

/// Sum of the turning angles along a planar polyline.
double total_turning(const std::vector<Point>& pts) {

    double turning = 0;

    for (size_t i = 1; i + 1 < pts.size(); i++) {
        const double dx1 = pts[i][0] - pts[i - 1][0];
        const double dy1 = pts[i][1] - pts[i - 1][1];
        const double dx2 = pts[i + 1][0] - pts[i][0];
        const double dy2 = pts[i + 1][1] - pts[i][1];
        const double l1 = std::hypot(dx1, dy1);
        const double l2 = std::hypot(dx2, dy2);

        if (l1 > 1e-14 && l2 > 1e-14) {
            double c = (dx1 * dx2 + dy1 * dy2) / (l1 * l2);
            c = std::max(-1.0, std::min(1.0, c));
            turning += std::acos(c);
        }
    }

    return turning;
}

/// Largest distance from each point to the curve, found by ternary search around its chord parameter.
double fitted_max_deviation(
    const NurbsCurve& cand,
    const std::vector<Point>& pts,
    const std::vector<double>& chords,
    int iterations
) {

    const int m = (int)pts.size();
    const std::pair<double, double> domain_ft = cand.domain();
    const double ft0 = domain_ft.first;
    const double ft1 = domain_ft.second;
    double max_dev = 0;

    for (int i = 0; i < m; i++) {
        const double t = ft0 + (ft1 - ft0) * chords[i];
        const double w2 = (ft1 - ft0) * 2.0 / std::max(m - 1, 1);
        double lo = std::max(ft0, t - w2);
        double hi = std::min(ft1, t + w2);

        for (int it = 0; it < iterations; ++it) {
            const double m1 = lo + (hi - lo) / 3;
            const double m2 = hi - (hi - lo) / 3;

            if (cand.point_at(m1).distance(pts[i]) < cand.point_at(m2).distance(pts[i]))
                hi = m2;
            else
                lo = m1;
        }

        max_dev = std::max(max_dev, cand.point_at(0.5 * (lo + hi)).distance(pts[i]));
    }

    return max_dev;
}

/// Cubic fitted to the points in the plane's frame, CVs doubled until within fit_tol, lifted back to 3D.
NurbsCurve fit_planar_freeform(const std::vector<Point>& all_pts, bool is_loop, const Plane& plane, double fit_tol) {

    const int m = (int)all_pts.size();

    if (m < 4)
        return NurbsCurve();

    const std::vector<Point> pts_2d = plane_points_2d(all_pts, plane);
    const std::vector<double> chords = chord_parameters(pts_2d, is_loop);
    int target_cvs = std::max(8, (int)(total_turning(pts_2d) / 0.5) + 6);
    const int max_cvs = std::min(m - 1, 128);
    NurbsCurve crv_2d;
    double best_dev = 1e300;

    for (int attempt = 0; attempt < 6; attempt++) {
        if (target_cvs > max_cvs)
            break;

        NurbsCurve cand = NurbsCurve::create_fitted(pts_2d, target_cvs, 3, is_loop);

        if (!cand.is_valid())
            break;

        const double max_dev = fitted_max_deviation(cand, pts_2d, chords, 20);

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

    if (!crv_2d.is_valid())
        return NurbsCurve();

    const Vector ax = plane.x_axis();
    const Vector ay = plane.y_axis();
    const Point po = plane.origin();

    for (int i = 0; i < crv_2d.cv_count(); i++) {
        const Point cv2 = crv_2d.get_cv(i);
        const double cx = cv2[0];
        const double cy = cv2[1];

        crv_2d.set_cv(
            i,
            Point(
                po[0] + cx * ax[0] + cy * ay[0],
                po[1] + cx * ax[1] + cy * ay[1],
                po[2] + cx * ax[2] + cy * ay[2]
            )
        );
    }

    return crv_2d;
}

/// Rational 9-CV circle on knots 0..4 around (cx, cy, cz) in the plane of the unit axes xa, ya.
NurbsCurve circle_nurbs(
    double cx,
    double cy,
    double cz,
    const std::array<double, 3>& xa,
    const std::array<double, 3>& ya,
    double radius
) {

    double w = std::sqrt(2.0) / 2.0;
    double px[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    double py[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    double wts[9] = {1, w, 1, w, 1, w, 1, w, 1};
    double knots[10] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
    NurbsCurve crv(3, true, 3, 9);

    for (int i = 0; i < 10; i++)
        crv.set_nurbsknot(i, knots[i]);

    for (int i = 0; i < 9; i++) {
        double x = cx + radius * (px[i] * xa[0] + py[i] * ya[0]);
        double y = cy + radius * (px[i] * xa[1] + py[i] * ya[1]);
        double z = cz + radius * (px[i] * xa[2] + py[i] * ya[2]);
        crv.set_cv_4d(i, x * wts[i], y * wts[i], z * wts[i], wts[i]);
    }

    return crv;
}

/// Rational 9-CV ellipse on knots 0..4 around (cx, cy, cz) with semi-axes along the unit axes ea, eb.
NurbsCurve ellipse_nurbs(
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
    double knots[10] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
    NurbsCurve crv(3, true, 3, 9);

    for (int i = 0; i < 10; i++)
        crv.set_nurbsknot(i, knots[i]);

    for (int i = 0; i < 9; i++) {
        double x = cx + semi_a * px[i] * ea[0] + semi_b * py[i] * eb[0];
        double y = cy + semi_a * px[i] * ea[1] + semi_b * py[i] * eb[1];
        double z = cz + semi_a * px[i] * ea[2] + semi_b * py[i] * eb[2];
        crv.set_cv_4d(i, x * wts[i], y * wts[i], z * wts[i], wts[i]);
    }

    return crv;
}

/// Coordinates of p in the 2D frame (po, ax, ay).
std::pair<double, double> plane_coords_2d(const Point& p, const Point& po, const Vector& ax, const Vector& ay) {

    double dx = p[0] - po[0];
    double dy = p[1] - po[1];
    double dz = p[2] - po[2];

    return {dx * ax[0] + dy * ax[1] + dz * ax[2], dx * ay[0] + dy * ay[1] + dz * ay[2]};
}

/// Exact circle of a closed planar trace when every point lies on the circle through three of them.
NurbsCurve fit_plane_circle(const std::vector<Point>& all_pts, const Plane& plane) {

    const Vector ax = plane.x_axis();
    const Vector ay = plane.y_axis();
    const Point po = plane.origin();
    const int n = (int)all_pts.size();
    double x1;
    double y1;
    double x2;
    double y2;
    double x3;
    double y3;
    std::tie(x1, y1) = plane_coords_2d(all_pts[0], po, ax, ay);
    std::tie(x2, y2) = plane_coords_2d(all_pts[n / 3], po, ax, ay);
    std::tie(x3, y3) = plane_coords_2d(all_pts[2 * n / 3], po, ax, ay);
    double ax_ = x2 - x1;
    double ay_ = y2 - y1;
    double bx_ = x3 - x1;
    double by_ = y3 - y1;
    double dd = 2.0 * (ax_ * by_ - ay_ * bx_);

    if (std::abs(dd) <= 1e-10)
        return NurbsCurve();

    double a2 = ax_ * ax_ + ay_ * ay_;
    double b2 = bx_ * bx_ + by_ * by_;
    double ccx = x1 + (by_ * a2 - ay_ * b2) / dd;
    double ccy = y1 + (ax_ * b2 - bx_ * a2) / dd;
    double radius = std::hypot(x1 - ccx, y1 - ccy);
    double max_dev = 0;

    for (const Point& p : all_pts) {
        double px;
        double py;
        std::tie(px, py) = plane_coords_2d(p, po, ax, ay);
        max_dev = std::max(max_dev, std::abs(std::hypot(px - ccx, py - ccy) - radius));
    }

    if (radius <= 1e-10 || max_dev >= std::max(radius * 1e-5, 1e-6))
        return NurbsCurve();

    double cx3d = po[0] + ccx * ax[0] + ccy * ay[0];
    double cy3d = po[1] + ccx * ax[1] + ccy * ay[1];
    double cz3d = po[2] + ccx * ax[2] + ccy * ay[2];

    return circle_nurbs(cx3d, cy3d, cz3d, {ax[0], ax[1], ax[2]}, {ay[0], ay[1], ay[2]}, radius);
}

/// Least-squares conic A x^2 + B xy + C y^2 + D x + E y = 1 through the points in the plane's frame.
bool fit_plane_conic(const std::vector<Point>& all_pts, const Point& po, const Vector& ax, const Vector& ay, double coef[5]) {

    double ata[5][5] = {};
    double atb[5] = {};

    for (const Point& p : all_pts) {
        double x;
        double y;
        std::tie(x, y) = plane_coords_2d(p, po, ax, ay);
        double row[5] = {x * x, x * y, y * y, x, y};

        for (int r = 0; r < 5; r++) {
            atb[r] += row[r];

            for (int c = 0; c < 5; c++)
                ata[r][c] += row[r] * row[c];
        }
    }

    double m[5][6];

    for (int r = 0; r < 5; r++) {
        for (int c = 0; c < 5; c++)
            m[r][c] = ata[r][c];

        m[r][5] = atb[r];
    }

    for (int col = 0; col < 5; col++) {
        int pivot = col;

        for (int r = col + 1; r < 5; r++)
            if (std::fabs(m[r][col]) > std::fabs(m[pivot][col]))
                pivot = r;

        if (std::fabs(m[pivot][col]) < 1e-20)
            return false;

        if (pivot != col)
            for (int j = col; j <= 5; j++)
                std::swap(m[col][j], m[pivot][j]);

        for (int r = col + 1; r < 5; r++) {
            double f = m[r][col] / m[col][col];

            for (int j = col; j <= 5; j++)
                m[r][j] -= f * m[col][j];
        }
    }

    for (int i = 4; i >= 0; i--) {
        double s = m[i][5];

        for (int j = i + 1; j < 5; j++)
            s -= m[i][j] * coef[j];

        coef[i] = s / m[i][i];
    }

    return true;
}

/// Largest distance from the points to the ellipse (cx, cy, semi_a, semi_b, theta) in the plane's frame.
double plane_ellipse_deviation(
    const std::vector<Point>& all_pts,
    const Point& po,
    const Vector& ax,
    const Vector& ay,
    double cx,
    double cy,
    double semi_a,
    double semi_b,
    double cos_t,
    double sin_t
) {

    double max_ell_dev = 0;

    for (const Point& p : all_pts) {
        double px2;
        double py2;
        std::tie(px2, py2) = plane_coords_2d(p, po, ax, ay);
        double lx = cos_t * (px2 - cx) + sin_t * (py2 - cy);
        double ly = -sin_t * (px2 - cx) + cos_t * (py2 - cy);
        double ang = std::atan2(ly / semi_b, lx / semi_a);
        double ex = cx + semi_a * std::cos(ang) * cos_t - semi_b * std::sin(ang) * sin_t;
        double ey = cy + semi_a * std::cos(ang) * sin_t + semi_b * std::sin(ang) * cos_t;
        max_ell_dev = std::max(max_ell_dev, std::hypot(px2 - ex, py2 - ey));
    }

    return max_ell_dev;
}

/// Exact ellipse of a closed planar trace from a least-squares conic, invalid when it deviates.
NurbsCurve fit_plane_ellipse(const std::vector<Point>& all_pts, const Plane& plane) {

    const Vector ax = plane.x_axis();
    const Vector ay = plane.y_axis();
    const Point po = plane.origin();
    double coef[5] = {};

    if (!fit_plane_conic(all_pts, po, ax, ay, coef))
        return NurbsCurve();

    double ca = coef[0];
    double cb = coef[1];
    double cc = coef[2];
    double cd = coef[3];
    double ce = coef[4];
    double disc = cb * cb - 4 * ca * cc;

    if (disc >= -1e-10 || std::fabs(ca) <= 1e-14)
        return NurbsCurve();

    double max_conic_dev = 0;

    for (const Point& p : all_pts) {
        double x;
        double y;
        std::tie(x, y) = plane_coords_2d(p, po, ax, ay);
        max_conic_dev = std::max(max_conic_dev, std::fabs(ca * x * x + cb * x * y + cc * y * y + cd * x + ce * y - 1.0));
    }

    if (max_conic_dev / std::max(std::max(std::fabs(ca), std::fabs(cc)), 1e-10) >= 0.01)
        return NurbsCurve();

    double det = 4 * ca * cc - cb * cb;
    double cx = (cb * ce - 2 * cc * cd) / det;
    double cy = (cb * cd - 2 * ca * ce) / det;
    double theta = 0.5 * std::atan2(cb, ca - cc);
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);
    double a2 = ca * cos_t * cos_t + cb * cos_t * sin_t + cc * sin_t * sin_t;
    double c2 = ca * sin_t * sin_t - cb * cos_t * sin_t + cc * cos_t * cos_t;
    double rhs = -(ca * cx * cx + cb * cx * cy + cc * cy * cy + cd * cx + ce * cy - 1.0);

    if (rhs <= 1e-14 || a2 <= 1e-14 || c2 <= 1e-14)
        return NurbsCurve();

    double semi_a = std::sqrt(rhs / a2);
    double semi_b = std::sqrt(rhs / c2);
    double cx3d = po[0] + cx * ax[0] + cy * ay[0];
    double cy3d = po[1] + cx * ax[1] + cy * ay[1];
    double cz3d = po[2] + cx * ax[2] + cy * ay[2];
    std::array<double, 3> ea;
    std::array<double, 3> eb;

    for (int d = 0; d < 3; d++) {
        ea[d] = cos_t * ax[d] + sin_t * ay[d];
        eb[d] = -sin_t * ax[d] + cos_t * ay[d];
    }

    NurbsCurve crv = ellipse_nurbs(cx3d, cy3d, cz3d, ea, eb, semi_a, semi_b);

    const double ell_tol = std::max(std::max(semi_a, semi_b) * 1e-5, 2e-6);

    if (plane_ellipse_deviation(all_pts, po, ax, ay, cx, cy, semi_a, semi_b, cos_t, sin_t) > ell_tol)
        return NurbsCurve();

    return crv;
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

    if (allow_conics && is_loop && all_pts.size() >= 6)
        crv = fit_plane_circle(all_pts, plane);

    if (!crv.is_valid() && allow_conics && is_loop && all_pts.size() >= 8)
        crv = fit_plane_ellipse(all_pts, plane);

    if (!crv.is_valid())
        crv = fit_planar_freeform(all_pts, is_loop, plane, step * (uv_to_3d + uv_to_3d_min) * 0.5 * 5e-4);

    return crv;
}

/// Seam-free run of uv samples cut from one trace.
struct SurfacePlanePiece {
    std::vector<std::pair<double, double>> uv; // Samples in parameter space.
    bool is_loop; // Whether the piece still closes on itself.
};

/// Whether the quarter, half and three-quarter samples of a trace all lie within dup_tol of one kept trace.
bool is_duplicate_trace(
    const std::vector<Point>& trace_pts3,
    const std::vector<std::vector<Point>>& kept_pts3,
    double dup_tol
) {

    const int m = (int)trace_pts3.size();

    for (const std::vector<Point>& other : kept_pts3) {
        bool all_close = true;

        for (double f : {0.25, 0.5, 0.75}) {
            const Point& cp = trace_pts3[(int)((m - 1) * f)];
            double dmin = dup_tol + 1.0;

            for (size_t k = 0; k < other.size(); k += 5)
                dmin = std::min(dmin, cp.distance(other[k]));

            if (dmin > dup_tol) {
                all_close = false;
                break;
            }
        }

        if (all_close)
            return true;
    }

    return false;
}

/// Append the loop start shifted by whole periods after the end; the shift goes to closure_du and closure_dv.
void close_unwrapped_loop(
    const SurfacePlaneField& field,
    std::vector<std::pair<double, double>>& pts,
    double& closure_du,
    double& closure_dv
) {

    double du_j = pts[0].first - pts.back().first;
    double dv_j = pts[0].second - pts.back().second;

    if (field.closed_u) {
        while (du_j > field.range_u * 0.5)
            du_j -= field.range_u;

        while (du_j < -field.range_u * 0.5)
            du_j += field.range_u;
    }

    if (field.closed_v) {
        while (dv_j > field.range_v * 0.5)
            dv_j -= field.range_v;

        while (dv_j < -field.range_v * 0.5)
            dv_j += field.range_v;
    }

    closure_du = (pts.back().first + du_j) - pts[0].first;
    closure_dv = (pts.back().second + dv_j) - pts[0].second;
    pts.push_back({pts[0].first + closure_du, pts[0].second + closure_dv});
}

/// Seam crossings (t, axis, seam value) of the segment pa-pb, sorted by t.
std::vector<std::tuple<double, int, double>> seam_crossings(
    const SurfacePlaneField& field,
    const std::pair<double, double>& pa,
    const std::pair<double, double>& pb
) {

    std::vector<std::tuple<double, int, double>> crossings;

    if (field.closed_u && std::abs(pb.first - pa.first) > 1e-15) {
        const int k0 = (int)std::floor((pa.first - field.u0) / field.range_u);
        const int k1 = (int)std::floor((pb.first - field.u0) / field.range_u);

        for (int k = std::min(k0, k1) + 1; k <= std::max(k0, k1); k++) {
            const double L = field.u0 + k * field.range_u;
            const double t = (L - pa.first) / (pb.first - pa.first);

            if (0.0 < t && t < 1.0)
                crossings.push_back({t, 0, L});
        }
    }

    if (field.closed_v && std::abs(pb.second - pa.second) > 1e-15) {
        const int k0 = (int)std::floor((pa.second - field.v0) / field.range_v);
        const int k1 = (int)std::floor((pb.second - field.v0) / field.range_v);

        for (int k = std::min(k0, k1) + 1; k <= std::max(k0, k1); k++) {
            const double L = field.v0 + k * field.range_v;
            const double t = (L - pa.second) / (pb.second - pa.second);

            if (0.0 < t && t < 1.0)
                crossings.push_back({t, 1, L});
        }
    }

    std::sort(crossings.begin(), crossings.end());

    return crossings;
}

/// Snap q onto a seam it lies on within 1e-9 of the period after a real move from pa; true when snapped.
bool snap_to_seam(const SurfacePlaneField& field, const std::pair<double, double>& pa, std::pair<double, double>& q) {

    bool on_seam = false;

    if (field.closed_u) {
        const double k = std::round((q.first - field.u0) / field.range_u);
        const double L = field.u0 + k * field.range_u;

        if (std::abs(q.first - L) < field.range_u * 1e-9 && std::abs(q.first - pa.first) > field.range_u * 1e-9) {
            q.first = L;
            on_seam = true;
        }
    }

    if (field.closed_v) {
        const double k = std::round((q.second - field.v0) / field.range_v);
        const double L = field.v0 + k * field.range_v;

        if (std::abs(q.second - L) < field.range_v * 1e-9 && std::abs(q.second - pa.second) > field.range_v * 1e-9) {
            q.second = L;
            on_seam = true;
        }
    }

    return on_seam;
}

/// Samples with the seam crossings inserted into out_pts; returns the indices of the samples on a seam.
std::vector<int> split_at_seams(
    const SurfacePlaneField& field,
    const std::vector<std::pair<double, double>>& pts,
    std::vector<std::pair<double, double>>& out_pts
) {

    std::vector<int> cross_idx;
    out_pts.push_back(pts[0]);

    for (size_t i = 1; i < pts.size(); i++) {
        const std::pair<double, double> pa = pts[i - 1];
        const std::pair<double, double> pb = pts[i];

        for (const std::tuple<double, int, double>& crossing : seam_crossings(field, pa, pb)) {
            const double t = std::get<0>(crossing);
            const double L = std::get<2>(crossing);
            double cu = pa.first + (pb.first - pa.first) * t;
            double cv_ = pa.second + (pb.second - pa.second) * t;

            if (std::get<1>(crossing) == 0) {
                cv_ = field.seam_newton(L, cv_, 0).second;
                cu = L;
            } else {
                cu = field.seam_newton(cu, L, 1).first;
                cv_ = L;
            }

            out_pts.push_back({cu, cv_});
            cross_idx.push_back((int)out_pts.size() - 1);
        }

        out_pts.push_back({pb.first, pb.second});

        if (i < pts.size() - 1 && snap_to_seam(field, pa, out_pts.back()))
            cross_idx.push_back((int)out_pts.size() - 1);
    }

    return cross_idx;
}

/// Cut the samples at the seam indices; a loop's last piece wraps around to its first seam.
std::vector<SurfacePlanePiece> seam_pieces(
    const std::vector<std::pair<double, double>>& out_pts,
    const std::vector<int>& cross_idx,
    bool is_loop,
    bool wrap_drift,
    double closure_du,
    double closure_dv
) {

    std::vector<SurfacePlanePiece> pieces;

    if (cross_idx.empty()) {
        pieces.push_back({out_pts, is_loop && !wrap_drift});

        return pieces;
    }

    if (is_loop) {
        for (size_t ci = 0; ci + 1 < cross_idx.size(); ci++)
            pieces.push_back({{out_pts.begin() + cross_idx[ci], out_pts.begin() + cross_idx[ci + 1] + 1}, false});

        std::vector<std::pair<double, double>> wrap_piece(out_pts.begin() + cross_idx.back(), out_pts.end());

        for (int pi = 1; pi <= cross_idx[0]; pi++)
            wrap_piece.push_back({out_pts[pi].first + closure_du, out_pts[pi].second + closure_dv});

        pieces.push_back({wrap_piece, false});

        return pieces;
    }

    std::vector<int> bounds;
    bounds.push_back(0);

    for (int ci : cross_idx)
        bounds.push_back(ci);

    bounds.push_back((int)out_pts.size() - 1);

    for (size_t bi = 0; bi + 1 < bounds.size(); bi++)
        if (bounds[bi + 1] > bounds[bi])
            pieces.push_back({{out_pts.begin() + bounds[bi], out_pts.begin() + bounds[bi + 1] + 1}, false});

    return pieces;
}

/// Seam-free uv pieces of one trace.
std::vector<SurfacePlanePiece> trace_pieces(const SurfacePlaneField& field, const SurfacePlaneTrace& trace) {

    std::vector<std::pair<double, double>> pts = trace.uv_unwrapped;
    double closure_du = 0.0;
    double closure_dv = 0.0;

    if (trace.is_loop && pts.size() >= 2)
        close_unwrapped_loop(field, pts, closure_du, closure_dv);

    std::vector<std::pair<double, double>> out_pts;
    const std::vector<int> cross_idx = split_at_seams(field, pts, out_pts);
    const bool wrap_drift = std::fabs(closure_du) > field.range_u * 0.5 || std::fabs(closure_dv) > field.range_v * 0.5;

    return seam_pieces(out_pts, cross_idx, trace.is_loop, wrap_drift, closure_du, closure_dv);
}

/// Shift a piece by whole periods so its middle sample lies in the base domain.
void shift_piece_to_domain(const SurfacePlaneField& field, std::vector<std::pair<double, double>>& piece_pts) {

    const std::pair<double, double> mid = piece_pts[piece_pts.size() / 2];

    if (field.closed_u) {
        const int k_u = (int)std::floor((mid.first - field.u0) / field.range_u);

        if (k_u != 0)
            for (std::pair<double, double>& p : piece_pts)
                p.first -= k_u * field.range_u;
    }

    if (field.closed_v) {
        const int k_v = (int)std::floor((mid.second - field.v0) / field.range_v);

        if (k_v != 0)
            for (std::pair<double, double>& p : piece_pts)
                p.second -= k_v * field.range_v;
    }
}

/// Insert zero-set samples between a and b while the chord midpoint sags more than step * 1e-4, four levels deep.
void densify_segment(
    const SurfacePlaneField& field,
    double au,
    double av,
    double bu,
    double bv,
    int depth,
    std::vector<Point>& pts_uv
) {

    const double mu = 0.5 * (au + bu);
    const double mv = 0.5 * (av + bv);
    double cu = mu;
    double cv2 = mv;

    if (!field.polish(cu, cv2))
        return;

    const double sag = std::hypot(cu - mu, cv2 - mv);

    if (sag > field.step * 1e-4 && depth < 4) {
        densify_segment(field, au, av, cu, cv2, depth + 1, pts_uv);
        pts_uv.push_back(Point(cu, cv2, 0.0));
        densify_segment(field, cu, cv2, bu, bv, depth + 1, pts_uv);
    } else {
        pts_uv.push_back(Point(cu, cv2, 0.0));
    }
}

/// Piece samples with zero-set samples inserted where a segment sags.
std::vector<Point> densify_piece(const SurfacePlaneField& field, const std::vector<std::pair<double, double>>& piece_pts) {

    std::vector<Point> pts_uv;
    pts_uv.reserve(piece_pts.size() * 4);

    for (size_t i = 1; i < piece_pts.size(); i++) {
        pts_uv.push_back(Point(piece_pts[i - 1].first, piece_pts[i - 1].second, 0.0));

        densify_segment(
            field,
            piece_pts[i - 1].first,
            piece_pts[i - 1].second,
            piece_pts[i].first,
            piece_pts[i].second,
            0,
            pts_uv
        );
    }

    pts_uv.push_back(Point(piece_pts.back().first, piece_pts.back().second, 0.0));

    return pts_uv;
}

/// Largest distance from each point to the curve at its chord parameter.
double chord_max_deviation(const NurbsCurve& cand, const std::vector<Point>& pts, const std::vector<double>& chords) {

    const std::pair<double, double> domain_ft = cand.domain();
    const double ft0 = domain_ft.first;
    const double ft1 = domain_ft.second;
    double max_dev = 0;

    for (size_t i = 0; i < pts.size(); i++) {
        const double t = ft0 + (ft1 - ft0) * chords[i];
        max_dev = std::max(max_dev, cand.point_at(t).distance(pts[i]));
    }

    return max_dev;
}

/// Cubic pcurve through the uv samples, CVs doubled until within step * 2e-3; target_cvs keeps the last count.
NurbsCurve fit_pcurve(const std::vector<Point>& pts_uv, bool piece_loop, double step, int& target_cvs) {

    const int mp = (int)pts_uv.size();
    const std::vector<double> chords = chord_parameters(pts_uv, piece_loop);
    const int max_cvs = std::min(mp - 1, 96);
    NurbsCurve pcurve;
    double pcurve_dev = 1e300;
    target_cvs = std::max(8, (int)(total_turning(pts_uv) / 0.5) + 6);

    for (int attempt = 0; attempt < 6; attempt++) {
        if (target_cvs > max_cvs)
            break;

        NurbsCurve cand = NurbsCurve::create_fitted(pts_uv, target_cvs, 3, piece_loop);

        if (!cand.is_valid())
            break;

        const double max_dev = chord_max_deviation(cand, pts_uv, chords);

        if (max_dev < pcurve_dev) {
            pcurve_dev = max_dev;
            pcurve = cand;
        }

        if (max_dev < step * 2e-3)
            break;

        target_cvs = std::min(target_cvs * 2, max_cvs + 1);
    }

    if (!pcurve.is_valid())
        pcurve = piece_loop ? NurbsCurve::create_interpolated(pts_uv, CurveNurbsKnotStyle::ChordPeriodic)
                            : NurbsCurve::create_interpolated(pts_uv);

    return pcurve;
}

/// Refit the pcurve with twice the CVs when it strays from the zero set by more than vali_tol.
void refit_pcurve(
    const SurfacePlaneField& field,
    const std::vector<Point>& pts_uv,
    bool piece_loop,
    int target_cvs,
    double vali_tol,
    NurbsCurve& pcurve
) {

    const int max_cvs = std::min((int)pts_uv.size() - 1, 96);
    double max_off = 0;

    for (int i = 0; i < 17; i++) {
        const Point pc = pcurve.point_at(i / 16.0);
        double val;
        double gu;
        double gv;
        field.value_and_gradient(pc[0], pc[1], val, gu, gv);
        max_off = std::max(max_off, std::abs(val));
    }

    if (max_off > vali_tol && target_cvs * 2 <= max_cvs) {
        NurbsCurve refit = NurbsCurve::create_fitted(pts_uv, target_cvs * 2, 3, piece_loop);

        if (refit.is_valid()) {
            refit.set_domain(0.0, 1.0);
            pcurve = refit;
        }
    }
}

/// 3D section curve and uv pcurve of one seam-free piece; false when a fit fails.
bool piece_curves(
    const SurfacePlaneField& field,
    const Plane& plane,
    SurfacePlanePiece& piece,
    NurbsCurve& crv3,
    NurbsCurve& pcurve
) {

    shift_piece_to_domain(field, piece.uv);

    const std::vector<Point> pts_uv = densify_piece(field, piece.uv);
    std::vector<Point> pts3(pts_uv.size());

    for (size_t i = 0; i < pts_uv.size(); i++)
        pts3[i] = field.point({field.wrap_u(pts_uv[i][0]), field.wrap_v(pts_uv[i][1])});

    crv3 = surface_plane_fit_3d(pts3, piece.is_loop, plane, field.step, field.uv_to_3d, field.uv_to_3d_min, false);

    if (!crv3.is_valid())
        crv3 = piece.is_loop ? NurbsCurve::create_interpolated(pts3, CurveNurbsKnotStyle::ChordPeriodic)
                             : NurbsCurve::create_interpolated(pts3);

    if (!crv3.is_valid())
        return false;

    int target_cvs;
    pcurve = fit_pcurve(pts_uv, piece.is_loop, field.step, target_cvs);

    if (!pcurve.is_valid())
        return false;

    crv3.set_domain(0.0, 1.0);
    pcurve.set_domain(0.0, 1.0);

    const double fit_tol = field.step * (field.uv_to_3d + field.uv_to_3d_min) * 0.5;
    refit_pcurve(field, pts_uv, piece.is_loop, target_cvs, std::max(10.0 * field.tolerance, fit_tol * 2.0), pcurve);

    return true;
}

/// Solve an n x n linear system by Gaussian elimination with partial pivoting.
bool solve_gauss(
    const std::vector<std::vector<double>>& m,
    const std::vector<double>& rhs,
    int n,
    std::vector<double>& out
) {

    std::vector<std::vector<double>> a(n, std::vector<double>(n + 1));

    for (int r = 0; r < n; r++) {
        for (int c = 0; c < n; c++)
            a[r][c] = m[r][c];

        a[r][n] = rhs[r];
    }

    for (int col = 0; col < n; col++) {
        int pivot = col;

        for (int r = col + 1; r < n; r++)
            if (std::abs(a[r][col]) > std::abs(a[pivot][col]))
                pivot = r;

        if (std::abs(a[pivot][col]) < 1e-20)
            return false;

        if (pivot != col)
            std::swap(a[col], a[pivot]);

        for (int r = col + 1; r < n; r++) {
            double f = a[r][col] / a[col][col];

            for (int j = col; j < n + 1; j++)
                a[r][j] -= f * a[col][j];
        }
    }

    out.assign(n, 0.0);

    for (int i = n - 1; i >= 0; i--) {
        double s = a[i][n];

        for (int j = i + 1; j < n; j++)
            s -= a[i][j] * out[j];

        out[i] = s / a[i][i];
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

/// Cross product of two triples.
static std::array<double, 3> ssi_cross(const std::array<double, 3>& u, const std::array<double, 3>& v) {
    return std::array<double, 3>{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
}

/// Unit triple, or the input when degenerate.
static std::array<double, 3> ssi_unit(const std::array<double, 3>& v) {

    double length = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

    return length > 1e-300 ? std::array<double, 3>{v[0] / length, v[1] / length, v[2] / length} : v;
}

/// Normalize a triple in place; false when shorter than 1e-12.
static bool normalize_axis(std::array<double, 3>& v) {

    double length = std::sqrt(ssi_dot(v, v));

    if (length < 1e-12)
        return false;

    v[0] /= length;
    v[1] /= length;
    v[2] /= length;

    return true;
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

/// Exact 9-CV rational NURBS circle on domain [0, 1].
static NurbsCurve exact_circle(
    double cx,
    double cy,
    double cz,
    const std::array<double, 3>& xa,
    const std::array<double, 3>& ya,
    double radius
) {

    NurbsCurve crv = circle_nurbs(cx, cy, cz, xa, ya, radius);
    crv.set_domain(0.0, 1.0);

    return crv;
}

/// Exact 9-CV rational NURBS ellipse on domain [0, 1].
static NurbsCurve exact_ellipse(
    double cx,
    double cy,
    double cz,
    const std::array<double, 3>& ea,
    const std::array<double, 3>& eb,
    double semi_a,
    double semi_b
) {

    NurbsCurve crv = ellipse_nurbs(cx, cy, cz, ea, eb, semi_a, semi_b);
    crv.set_domain(0.0, 1.0);

    return crv;
}

/// Eigenvalues/vectors of a symmetric 3x3 matrix (cyclic Jacobi).
static void jacobi_eig3(const double m[3][3], double eigvals[3], std::array<double, 3> eigvecs[3]) {

    double a[3][3];
    double v[3][3];

    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            a[r][c] = m[r][c];
            v[r][c] = (r == c) ? 1.0 : 0.0;
        }
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

/// Eigenvector of the smallest eigenvalue of a symmetric 3x3 matrix.
static std::array<double, 3> smallest_eigenvector(const double m[3][3]) {

    double evals[3];
    std::array<double, 3> evecs[3];
    jacobi_eig3(m, evals, evecs);
    int kmin = 0;

    for (int k = 1; k < 3; k++)
        if (evals[k] < evals[kmin])
            kmin = k;

    return evecs[kmin];
}

/// Eigenvector of the largest eigenvalue of a symmetric 3x3 matrix.
static std::array<double, 3> largest_eigenvector(const double m[3][3]) {

    double evals[3];
    std::array<double, 3> evecs[3];
    jacobi_eig3(m, evals, evecs);
    int kmax = 0;

    for (int k = 1; k < 3; k++)
        if (evals[k] > evals[kmax])
            kmax = k;

    return evecs[kmax];
}

/// Recognized-surface descriptor.
struct RecogSurface {
    enum Kind { NONE, PLANE, SPHERE, CYLINDER, CONE, TORUS } kind = NONE; // Recognized kind.
    std::array<double, 3> p1{}; // Origin, center or apex.
    std::array<double, 3> p2{}; // Normal or axis.
    double r = 0.0; // Radius or major radius.
    double r2 = 0.0; // Half angle or minor radius.
};

/// Points of an n x n parameter grid stepping the domain by 1 / div.
static std::vector<std::array<double, 3>> sample_grid(const NurbsSurface& surface, int n, double div) {

    const std::pair<double, double> domain_u = surface.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = surface.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    std::vector<std::array<double, 3>> pts;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Point p = surface.point_at(u0 + (u1 - u0) * i / div, v0 + (v1 - v0) * j / div);
            pts.push_back(std::array<double, 3>{p[0], p[1], p[2]});
        }
    }

    return pts;
}

/// Normals of an n x n parameter grid stepping the domain by 1 / div.
static std::vector<std::array<double, 3>> sample_grid_normals(const NurbsSurface& surface, int n, double div) {

    const std::pair<double, double> domain_u = surface.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = surface.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    std::vector<std::array<double, 3>> nrm;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Vector v = surface.normal_at(u0 + (u1 - u0) * i / div, v0 + (v1 - v0) * j / div);
            nrm.push_back(std::array<double, 3>{v[0], v[1], v[2]});
        }
    }

    return nrm;
}

/// Least-squares circle through 2D samples: center and squared radius.
static bool fit_circle_2d(const std::vector<std::pair<double, double>>& xy, double& cx, double& cy, double& r2) {

    std::vector<std::vector<double>> ata(3, std::vector<double>(3, 0.0));
    std::vector<double> atb(3, 0.0);

    for (const std::pair<double, double>& p : xy) {
        double row[3] = {p.first, p.second, 1.0};
        double rhs = -(p.first * p.first + p.second * p.second);

        for (int r = 0; r < 3; r++) {
            atb[r] += row[r] * rhs;

            for (int c = 0; c < 3; c++)
                ata[r][c] += row[r] * row[c];
        }
    }

    std::vector<double> sol;

    if (!solve_gauss(ata, atb, 3, sol))
        return false;

    cx = -sol[0] / 2.0;
    cy = -sol[1] / 2.0;
    r2 = cx * cx + cy * cy - sol[2];

    return true;
}

/// Recognize a cylinder from surface samples: axis point, axis direction and radius.
static bool fit_cylinder(
    const NurbsSurface& surface,
    double tol,
    std::array<double, 3>& axis_pt,
    std::array<double, 3>& axis_dir,
    double& radius
) {

    std::vector<std::array<double, 3>> pts = sample_grid(surface, 5, 4.0);
    std::vector<std::array<double, 3>> nrm = sample_grid_normals(surface, 5, 4.0);
    double m[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    for (std::array<double, 3>& n : nrm)
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                m[r][c] += n[r] * n[c];

    std::array<double, 3> w = smallest_eigenvector(m);

    if (!normalize_axis(w))
        return false;

    std::array<double, 3> ea;
    std::array<double, 3> eb;
    std::tie(ea, eb) = ortho_basis(w);
    std::array<double, 3> p0 = pts[0];
    std::vector<std::pair<double, double>> proj;

    for (std::array<double, 3>& p : pts) {
        std::array<double, 3> dp{p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]};
        proj.push_back({ssi_dot(dp, ea), ssi_dot(dp, eb)});
    }

    double ccx;
    double ccy;
    double r2;

    if (!fit_circle_2d(proj, ccx, ccy, r2) || r2 <= 1e-18)
        return false;

    double r = std::sqrt(r2);

    for (std::pair<double, double>& pr : proj)
        if (std::abs(std::sqrt((pr.first - ccx) * (pr.first - ccx) + (pr.second - ccy) * (pr.second - ccy)) - r) > tol)
            return false;

    axis_pt = std::array<double, 3>{
        p0[0] + ccx * ea[0] + ccy * eb[0],
        p0[1] + ccx * ea[1] + ccy * eb[1],
        p0[2] + ccx * ea[2] + ccy * eb[2]
    };

    axis_dir = w;
    radius = r;

    return true;
}

/// Cone samples on an 8 x 5 grid with the unit normals that are not degenerate.
static void sample_cone(
    const NurbsSurface& surface,
    std::vector<std::array<double, 3>>& pts,
    std::vector<std::pair<std::array<double, 3>, std::array<double, 3>>>& nrm
) {

    const std::pair<double, double> domain_u = surface.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = surface.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;

    for (int i = 0; i < 8; i++) {
        double uu = u0 + (u1 - u0) * i / 8.0;

        for (int j = 0; j < 5; j++) {
            double vv = v0 + (v1 - v0) * j / 4.0;
            Point p = surface.point_at(uu, vv);
            pts.push_back(std::array<double, 3>{p[0], p[1], p[2]});
            Vector n = surface.normal_at(uu, vv);
            double nl = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);

            if (nl < 1e-12)
                continue;

            nrm.push_back({std::array<double, 3>{n[0] / nl, n[1] / nl, n[2] / nl}, std::array<double, 3>{p[0], p[1], p[2]}});
        }
    }
}

/// Least-squares meeting point of the tangent planes through (unit normal, point) samples.
static bool cone_apex(
    const std::vector<std::pair<std::array<double, 3>, std::array<double, 3>>>& nrm,
    std::vector<double>& apex
) {

    std::vector<std::vector<double>> ata(3, std::vector<double>(3, 0.0));
    std::vector<double> atb(3, 0.0);

    for (const std::pair<std::array<double, 3>, std::array<double, 3>>& np : nrm) {
        const std::array<double, 3>& n = np.first;
        double npd = ssi_dot(n, np.second);

        for (int r = 0; r < 3; r++) {
            atb[r] += n[r] * npd;

            for (int c = 0; c < 3; c++)
                ata[r][c] += n[r] * n[c];
        }
    }

    return solve_gauss(ata, atb, 3, apex);
}

/// Mean generator direction of unit apex-to-sample vectors, oriented away from the apex.
static bool cone_axis(const std::vector<std::array<double, 3>>& gs, std::array<double, 3>& w) {

    double gram[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    for (const std::array<double, 3>& g : gs)
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                gram[r][c] += g[r] * g[c];

    w = largest_eigenvector(gram);
    std::array<double, 3> sx{0, 0, 0};

    for (const std::array<double, 3>& g : gs) {
        sx[0] += g[0];
        sx[1] += g[1];
        sx[2] += g[2];
    }

    if (ssi_dot(w, sx) < 0.0)
        w = std::array<double, 3>{-w[0], -w[1], -w[2]};

    return normalize_axis(w);
}

/// Recognize a cone from surface samples: apex, axis and half angle.
static bool fit_cone(
    const NurbsSurface& surface,
    double tol,
    std::array<double, 3>& apex,
    std::array<double, 3>& axis,
    double& half_angle
) {

    std::vector<std::array<double, 3>> pts;
    std::vector<std::pair<std::array<double, 3>, std::array<double, 3>>> nrm;
    sample_cone(surface, pts, nrm);
    std::vector<double> vertex;

    if ((int)nrm.size() < 4 || !cone_apex(nrm, vertex))
        return false;

    std::vector<std::array<double, 3>> gs;

    for (std::array<double, 3>& p : pts) {
        std::array<double, 3> d{p[0] - vertex[0], p[1] - vertex[1], p[2] - vertex[2]};
        double dl = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);

        if (dl < tol)
            continue;

        gs.push_back(std::array<double, 3>{d[0] / dl, d[1] / dl, d[2] / dl});
    }

    std::array<double, 3> w;

    if ((int)gs.size() < 3 || !cone_axis(gs, w))
        return false;

    double sumang = 0.0;

    for (std::array<double, 3>& g : gs)
        sumang += std::acos(std::max(-1.0, std::min(1.0, ssi_dot(g, w))));

    double alpha = sumang / gs.size();

    if (alpha < 1e-4 || alpha > Tolerance::PI / 2 - 1e-4)
        return false;

    double ca = std::cos(alpha);

    for (std::array<double, 3>& p : pts) {
        std::array<double, 3> d{p[0] - vertex[0], p[1] - vertex[1], p[2] - vertex[2]};
        double axd = ssi_dot(d, w);
        double perp = std::sqrt(std::max(0.0, ssi_dot(d, d) - axd * axd));

        if (std::abs(perp - axd * std::tan(alpha)) * ca > tol)
            return false;
    }

    apex = std::array<double, 3>{vertex[0], vertex[1], vertex[2]};
    axis = w;
    half_angle = alpha;

    return true;
}

/// Recognize a sphere from surface samples: center and radius.
static bool fit_sphere(const NurbsSurface& surface, double tol, double& cx, double& cy, double& cz, double& radius) {

    std::vector<std::array<double, 3>> pts = sample_grid(surface, 5, 4.0);
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

/// Centroid of the points and the unit direction of least spread about it; false when degenerate.
static bool principal_axis(const std::vector<std::array<double, 3>>& pts, std::array<double, 3>& cen, std::array<double, 3>& w) {

    int n = (int)pts.size();
    cen = std::array<double, 3>{0, 0, 0};

    for (const std::array<double, 3>& p : pts) {
        cen[0] += p[0];
        cen[1] += p[1];
        cen[2] += p[2];
    }

    cen[0] /= n;
    cen[1] /= n;
    cen[2] /= n;
    double m[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    for (const std::array<double, 3>& p : pts) {
        std::array<double, 3> d{p[0] - cen[0], p[1] - cen[1], p[2] - cen[2]};

        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                m[r][c] += d[r] * d[c];
    }

    w = smallest_eigenvector(m);

    return normalize_axis(w);
}

/// Recognize a torus from the smallest-variance axis and a tube cross-section circle fit.
static bool fit_torus(
    const NurbsSurface& surface,
    double tol,
    std::array<double, 3>& center,
    std::array<double, 3>& axis,
    double& rmaj_out,
    double& r_out
) {

    std::vector<std::array<double, 3>> pts = sample_grid(surface, 8, 8.0);
    std::array<double, 3> cen;
    std::array<double, 3> w;

    if (!principal_axis(pts, cen, w))
        return false;

    std::vector<std::pair<double, double>> rhoa;

    for (std::array<double, 3>& p : pts) {
        std::array<double, 3> d{p[0] - cen[0], p[1] - cen[1], p[2] - cen[2]};
        double a = ssi_dot(d, w);
        std::array<double, 3> perp{d[0] - a * w[0], d[1] - a * w[1], d[2] - a * w[2]};
        rhoa.push_back({std::sqrt(ssi_dot(perp, perp)), a});
    }

    double rmaj;
    double a0;
    double r2;

    if (!fit_circle_2d(rhoa, rmaj, a0, r2) || r2 <= 1e-18 || rmaj <= 0.0)
        return false;

    double r = std::sqrt(r2);

    if (rmaj <= r * 0.5)
        return false;

    for (std::pair<double, double>& ra : rhoa)
        if (std::abs(std::sqrt((ra.first - rmaj) * (ra.first - rmaj) + (ra.second - a0) * (ra.second - a0)) - r) > tol)
            return false;

    center = std::array<double, 3>{cen[0] + a0 * w[0], cen[1] + a0 * w[1], cen[2] + a0 * w[2]};
    axis = w;
    rmaj_out = rmaj;
    r_out = r;

    return true;
}

/// Point and normal at the middle of the surface domain.
static void surface_mid_frame(const NurbsSurface& srf, Point& origin, Vector& normal) {

    const std::pair<double, double> domain_u = srf.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = srf.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    origin = srf.point_at((u0 + u1) * 0.5, (v0 + v1) * 0.5);
    normal = srf.normal_at((u0 + u1) * 0.5, (v0 + v1) * 0.5);
}

/// Classify a surface as plane, cylinder, cone, sphere or torus within tol.
static RecogSurface recognize_surface(const NurbsSurface& surface, double tol) {

    RecogSurface rs;

    if (surface.is_planar(nullptr, tol)) {
        Point o;
        Vector n;
        surface_mid_frame(surface, o, n);
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

    std::array<double, 3> p1;
    std::array<double, 3> p2;
    double r1 = 0.0;
    double r2 = 0.0;

    if (fit_cylinder(surface, tol, p1, p2, r1)) {
        rs.kind = RecogSurface::CYLINDER;
    } else if (fit_cone(surface, tol, p1, p2, r1)) {
        rs.kind = RecogSurface::CONE;
    } else if (fit_torus(surface, tol, p1, p2, r1, r2)) {
        rs.kind = RecogSurface::TORUS;
        rs.r2 = r2;
    } else {
        return rs;
    }

    rs.p1 = p1;
    rs.p2 = p2;
    rs.r = r1;

    return rs;
}

/// Parameters t where x0 + t d meets the double cone of the apex, unit axis w and half angle alpha.
static std::vector<double> line_cone(
    const std::array<double, 3>& x0,
    const std::array<double, 3>& d,
    const std::array<double, 3>& apex,
    const std::array<double, 3>& w,
    double alpha
) {

    double ca2 = std::cos(alpha) * std::cos(alpha);
    std::array<double, 3> e{x0[0] - apex[0], x0[1] - apex[1], x0[2] - apex[2]};
    double a = ssi_dot(e, w);
    double b = ssi_dot(d, w);
    double c = ssi_dot(e, e);
    double dd = ssi_dot(e, d);
    double ee = ssi_dot(d, d);
    double qa = b * b - ca2 * ee;
    double qb = 2.0 * a * b - 2.0 * ca2 * dd;
    double qc = a * a - ca2 * c;

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
    std::array<double, 3> p = cyl.p1;
    std::array<double, 3> w = ssi_unit(cyl.p2);
    double r = cyl.r;
    double wn = ssi_dot(w, nu);

    if (std::abs(wn) < 1e-7)
        return false;

    double t = ((o[0] - p[0]) * nu[0] + (o[1] - p[1]) * nu[1] + (o[2] - p[2]) * nu[2]) / wn;
    std::array<double, 3> cc{p[0] + t * w[0], p[1] + t * w[1], p[2] + t * w[2]};
    std::array<double, 3> mraw = ssi_cross(w, nu);

    if (std::sqrt(ssi_dot(mraw, mraw)) < 1e-9) {
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

/// Degree-1 segment of the line through q along w between axial offsets s0 and s1.
static NurbsCurve axis_segment(const std::array<double, 3>& q, const std::array<double, 3>& w, double s0, double s1) {

    Point e0(q[0] + s0 * w[0], q[1] + s0 * w[1], q[2] + s0 * w[2]);
    Point e1(q[0] + s1 * w[0], q[1] + s1 * w[1], q[2] + s1 * w[2]);

    return NurbsCurve::create(false, 1, {e0, e1});
}

/// Axial range of a cylinder surface over three u and both v boundaries, padded by 5%.
static void cylinder_axial_range(
    const NurbsSurface& srf,
    const std::array<double, 3>& p,
    const std::array<double, 3>& w,
    double& smin,
    double& smax
) {

    const std::pair<double, double> domain_u = srf.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = srf.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    smin = 1e300;
    smax = -1e300;

    for (double uu : {u0, 0.5 * (u0 + u1), u1}) {
        for (double vv : {v0, v1}) {
            Point q = srf.point_at(uu, vv);
            double s = (q[0] - p[0]) * w[0] + (q[1] - p[1]) * w[1] + (q[2] - p[2]) * w[2];
            smin = std::min(smin, s);
            smax = std::max(smax, s);
        }
    }

    double pad = 0.05 * std::max(1e-9, smax - smin);
    smin -= pad;
    smax += pad;
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
    std::array<double, 3> p = cyl.p1;
    std::array<double, 3> w = ssi_unit(cyl.p2);
    double r = cyl.r;
    double wn = ssi_dot(w, nu);

    if (std::abs(wn) >= 1e-7)
        return false;

    double ds = (p[0] - o[0]) * nu[0] + (p[1] - o[1]) * nu[1] + (p[2] - o[2]) * nu[2];
    double d = std::abs(ds);
    double tt = r * 1e-9 + 1e-12;

    if (d > r + tt)
        return true;

    double smin;
    double smax;
    cylinder_axial_range(cyl_srf, p, w, smin, smax);
    std::array<double, 3> foot{p[0] - ds * nu[0], p[1] - ds * nu[1], p[2] - ds * nu[2]};
    std::vector<std::array<double, 3>> feet;

    if (d >= r - tt) {
        feet.push_back(foot);
    } else {
        double h = std::sqrt(std::max(0.0, r * r - d * d));
        std::array<double, 3> s3 = ssi_unit(ssi_cross(w, nu));
        feet.push_back(std::array<double, 3>{foot[0] + h * s3[0], foot[1] + h * s3[1], foot[2] + h * s3[2]});
        feet.push_back(std::array<double, 3>{foot[0] - h * s3[0], foot[1] - h * s3[1], foot[2] - h * s3[2]});
    }

    for (const std::array<double, 3>& q : feet) {
        NurbsCurve line = axis_segment(q, w, smin, smax);

        if (line.is_valid())
            out.push_back(line);
    }

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
    double height = 0.0;

    for (double vv : {v0, v1}) {
        Point p = srf.point_at(um, vv);
        double s = (p[0] - apex[0]) * w[0] + (p[1] - apex[1]) * w[1] + (p[2] - apex[2]) * w[2];
        height = std::max(height, s);
    }

    return height;
}

/// Whether 65 samples of a conic lie within the cone height.
static bool conic_within_cone(const NurbsCurve& c, const std::array<double, 3>& apex, const std::array<double, 3>& w, double height) {

    const std::pair<double, double> domain = c.domain();
    const double t0 = domain.first;
    const double t1 = domain.second;
    double pad = 1e-7 * std::max(1.0, height);

    for (int i = 0; i <= 64; ++i) {
        Point p = c.point_at(t0 + (t1 - t0) * i / 64);
        double s = (p[0] - apex[0]) * w[0] + (p[1] - apex[1]) * w[1] + (p[2] - apex[2]) * w[2];

        if (s < -pad || s > height + pad)
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
    const std::array<double, 3>& apex,
    const std::array<double, 3>& w,
    double alpha,
    NurbsCurve& c3
) {

    double wn = ssi_dot(w, nu);
    std::array<double, 3> m = ssi_cross(w, nu);
    double ml = std::sqrt(ssi_dot(m, m));

    if (ml < 1e-12)
        return false;

    m = std::array<double, 3>{m[0] / ml, m[1] / ml, m[2] / ml};
    std::array<double, 3> major = ssi_unit(std::array<double, 3>{w[0] - wn * nu[0], w[1] - wn * nu[1], w[2] - wn * nu[2]});
    double dv = (apex[0] - o[0]) * nu[0] + (apex[1] - o[1]) * nu[1] + (apex[2] - o[2]) * nu[2];
    std::array<double, 3> vp{apex[0] - dv * nu[0], apex[1] - dv * nu[1], apex[2] - dv * nu[2]};
    std::vector<double> ts = line_cone(vp, major, apex, w, alpha);

    if (ts.size() != 2)
        return false;

    std::array<double, 3> pa{vp[0] + ts[0] * major[0], vp[1] + ts[0] * major[1], vp[2] + ts[0] * major[2]};
    std::array<double, 3> pb{vp[0] + ts[1] * major[0], vp[1] + ts[1] * major[1], vp[2] + ts[1] * major[2]};
    std::array<double, 3> cc{(pa[0] + pb[0]) * 0.5, (pa[1] + pb[1]) * 0.5, (pa[2] + pb[2]) * 0.5};
    std::array<double, 3> ab{pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]};
    double semi_major = 0.5 * std::sqrt(ssi_dot(ab, ab));
    major = ssi_unit(ab);
    std::vector<double> tm = line_cone(cc, m, apex, w, alpha);

    if (tm.size() != 2)
        return false;

    double semi_minor = 0.5 * std::abs(tm[1] - tm[0]);

    if (semi_major < 1e-12 || semi_minor < 1e-12)
        return false;

    c3 = exact_ellipse(cc[0], cc[1], cc[2], major, m, semi_major, semi_minor);

    return true;
}

/// Single rational quadratic Bezier conic arc from pa to pb with middle control point pt.
static NurbsCurve conic_bezier(
    const std::array<double, 3>& pa,
    const std::array<double, 3>& pt,
    const std::array<double, 3>& pb,
    double wmid
) {

    NurbsCurve crv(3, true, 3, 3);
    double knots[4] = {0, 0, 1, 1};

    for (int i = 0; i < 4; i++)
        crv.set_nurbsknot(i, knots[i]);

    crv.set_cv_4d(0, pa[0], pa[1], pa[2], 1.0);
    crv.set_cv_4d(1, pt[0] * wmid, pt[1] * wmid, pt[2] * wmid, wmid);
    crv.set_cv_4d(2, pb[0], pb[1], pb[2], 1.0);
    crv.set_domain(0.0, 1.0);

    return crv;
}

/// Frame of an open plane-cone conic: cone data and the conic's in-plane axes.
struct PlaneConeFrame {
    std::array<double, 3> o; // Plane origin.
    std::array<double, 3> nu; // Unit plane normal.
    std::array<double, 3> apex; // Cone apex.
    std::array<double, 3> w; // Unit cone axis.
    double height; // Cone height.
    double cosa; // Cosine of the half angle.
    double sina; // Sine of the half angle.
    double ta; // Tangent of the half angle.
    double na; // Plane normal along the axis.
    double cost; // Absolute na.
    double sint; // Length of nu x w.
    std::array<double, 3> axex; // In-plane axis towards the cone axis.
    std::array<double, 3> axey; // In-plane axis across the cone axis.
    double axw; // axex along the cone axis.
    double d0; // Signed apex distance to the plane.
    double tol; // On-surface tolerance.
};

/// Conic frame of a plane cutting a cone; false when the plane is perpendicular to or contains the axis.
static bool plane_cone_frame(
    const std::array<double, 3>& o,
    const std::array<double, 3>& nu,
    const std::array<double, 3>& apex,
    const std::array<double, 3>& w,
    double alpha,
    double height,
    PlaneConeFrame& f
) {

    f.o = o;
    f.nu = nu;
    f.apex = apex;
    f.w = w;
    f.height = height;
    f.cosa = std::cos(alpha);
    f.sina = std::sin(alpha);
    f.ta = std::tan(alpha);
    f.na = ssi_dot(nu, w);
    f.cost = std::abs(f.na);
    f.axey = ssi_cross(nu, w);
    f.sint = std::sqrt(ssi_dot(f.axey, f.axey));

    if (f.sint < 1e-12)
        return false;

    f.axey = std::array<double, 3>{f.axey[0] / f.sint, f.axey[1] / f.sint, f.axey[2] / f.sint};
    f.axex = ssi_cross(f.axey, nu);
    f.axw = ssi_dot(f.axex, w);

    if (f.axw < 0) {
        f.axex = std::array<double, 3>{-f.axex[0], -f.axex[1], -f.axex[2]};
        f.axw = -f.axw;
    }

    if (f.axw < 1e-12)
        return false;

    f.d0 = (apex[0] - o[0]) * nu[0] + (apex[1] - o[1]) * nu[1] + (apex[2] - o[2]) * nu[2];
    f.tol = 1e-6 * std::max(1.0, height);

    return true;
}

/// Whether 17 samples of the curve lie on both the plane and the cone within the frame tolerance.
static bool conic_on_plane_cone(const PlaneConeFrame& f, const NurbsCurve& c) {

    for (int i = 0; i <= 16; ++i) {
        Point q = c.point_at(i / 16.0);
        double dp = std::abs((q[0] - f.o[0]) * f.nu[0] + (q[1] - f.o[1]) * f.nu[1] + (q[2] - f.o[2]) * f.nu[2]);
        double zz = (q[0] - f.apex[0]) * f.w[0] + (q[1] - f.apex[1]) * f.w[1] + (q[2] - f.apex[2]) * f.w[2];
        double wx = q[0] - f.apex[0] - zz * f.w[0];
        double wy = q[1] - f.apex[1] - zz * f.w[1];
        double wz = q[2] - f.apex[2] - zz * f.w[2];
        double rho = std::sqrt(wx * wx + wy * wy + wz * wz);

        if (dp > f.tol || std::abs(rho - f.ta * zz) > f.tol * (1.0 + f.ta))
            return false;

        if (zz < -f.tol || zz > f.height + f.tol)
            return false;
    }

    return true;
}

/// Exact plane-cone parabola arc cut at the cone height.
static bool plane_cone_parabola(const PlaneConeFrame& f, NurbsCurve& c3) {

    if (f.cost < 1e-12)
        return false;

    double sax = -f.d0 / f.na;
    std::array<double, 3> cen{f.apex[0] + sax * f.w[0], f.apex[1] + sax * f.w[1], f.apex[2] + sax * f.w[2]};
    double distance = std::abs(sax);
    double dc = 0.5 * distance / f.cosa;
    double pf = dc * f.sina * f.sina;

    if (pf < 1e-15)
        return false;

    for (int cs : {-1, +1}) {
        std::array<double, 3> c2{cen[0] + cs * dc * f.axex[0], cen[1] + cs * dc * f.axex[1], cen[2] + cs * dc * f.axex[2]};
        double zc = (c2[0] - f.apex[0]) * f.w[0] + (c2[1] - f.apex[1]) * f.w[1] + (c2[2] - f.apex[2]) * f.w[2];
        double t1s = 2.0 * pf * (f.height - zc) / f.axw;

        if (t1s <= 0)
            continue;

        double t1 = std::sqrt(t1s);
        double xi = t1s / (2.0 * pf);

        std::array<double, 3> pa{
            c2[0] + xi * f.axex[0] - t1 * f.axey[0],
            c2[1] + xi * f.axex[1] - t1 * f.axey[1],
            c2[2] + xi * f.axex[2] - t1 * f.axey[2]
        };

        std::array<double, 3> pb{
            c2[0] + xi * f.axex[0] + t1 * f.axey[0],
            c2[1] + xi * f.axex[1] + t1 * f.axey[1],
            c2[2] + xi * f.axex[2] + t1 * f.axey[2]
        };

        std::array<double, 3> pt{c2[0] - xi * f.axex[0], c2[1] - xi * f.axex[1], c2[2] - xi * f.axex[2]};
        NurbsCurve arc = conic_bezier(pa, pt, pb, 1.0);

        if (arc.is_valid() && conic_on_plane_cone(f, arc)) {
            c3 = arc;

            return true;
        }
    }

    return false;
}

/// Semi-axes and centers of the plane-cone hyperbola, one center per nappe; false when degenerate.
static bool hyperbola_centers(const PlaneConeFrame& f, double& a, double& b, std::vector<std::array<double, 3>>& centers) {

    if (f.cost < 1e-6) {
        a = std::abs(f.d0) / f.ta;
        b = std::abs(f.d0);
        centers.push_back(std::array<double, 3>{f.apex[0] - f.d0 * f.nu[0], f.apex[1] - f.d0 * f.nu[1], f.apex[2] - f.d0 * f.nu[2]});
    } else {
        double dd = f.sina * f.sina - f.cost * f.cost;

        if (dd < 1e-12)
            return false;

        double sax = -f.d0 / f.na;
        std::array<double, 3> cen{f.apex[0] + sax * f.w[0], f.apex[1] + sax * f.w[1], f.apex[2] + sax * f.w[2]};
        double distance = std::abs(sax);
        double dc = f.sint * f.sina * f.sina * distance / dd;
        a = f.cost * f.sina * f.cosa * distance / dd;
        b = f.cost * f.sina * distance / std::sqrt(dd);
        centers.push_back(std::array<double, 3>{cen[0] - dc * f.axex[0], cen[1] - dc * f.axex[1], cen[2] - dc * f.axex[2]});
        centers.push_back(std::array<double, 3>{cen[0] + dc * f.axex[0], cen[1] + dc * f.axex[1], cen[2] + dc * f.axex[2]});
    }

    return !(a < 1e-15 || b < 1e-15);
}

/// Exact plane-cone hyperbola branch cut at the cone height.
static bool plane_cone_hyperbola(const PlaneConeFrame& f, NurbsCurve& c3) {

    double a = 0;
    double b = 0;
    std::vector<std::array<double, 3>> centers;

    if (!hyperbola_centers(f, a, b, centers))
        return false;

    for (const std::array<double, 3>& c2 : centers) {
        double zc = (c2[0] - f.apex[0]) * f.w[0] + (c2[1] - f.apex[1]) * f.w[1] + (c2[2] - f.apex[2]) * f.w[2];

        for (int sg : {+1, -1}) {
            double ch = (f.height - zc) / (sg * a * f.axw);

            if (ch <= 1.0 + 1e-12)
                continue;

            double sh = std::sqrt(ch * ch - 1.0);
            double xi = sg * a * ch;
            double xt = sg * a / ch;

            std::array<double, 3> pa{
                c2[0] + xi * f.axex[0] - b * sh * f.axey[0],
                c2[1] + xi * f.axex[1] - b * sh * f.axey[1],
                c2[2] + xi * f.axex[2] - b * sh * f.axey[2]
            };

            std::array<double, 3> pb{
                c2[0] + xi * f.axex[0] + b * sh * f.axey[0],
                c2[1] + xi * f.axex[1] + b * sh * f.axey[1],
                c2[2] + xi * f.axex[2] + b * sh * f.axey[2]
            };

            std::array<double, 3> pt{c2[0] + xt * f.axex[0], c2[1] + xt * f.axex[1], c2[2] + xt * f.axex[2]};
            NurbsCurve arc = conic_bezier(pa, pt, pb, ch);

            if (arc.is_valid() && conic_on_plane_cone(f, arc)) {
                c3 = arc;

                return true;
            }
        }
    }

    return false;
}

/// Exact plane-cone hyperbola or parabola arc (IntAna_QuadQuadGeo.cxx:752-953 port).
static bool build_exact_plane_cone_open(
    const std::array<double, 3>& o,
    const std::array<double, 3>& nu,
    const std::array<double, 3>& apex,
    const std::array<double, 3>& w,
    double alpha,
    double height,
    bool parabola,
    NurbsCurve& c3
) {

    PlaneConeFrame f;

    if (!plane_cone_frame(o, nu, apex, w, alpha, height, f))
        return false;

    if (parabola)
        return plane_cone_parabola(f, c3);

    return plane_cone_hyperbola(f, c3);
}

/// Plane-cone section: the plane, the cone and the cone's polar frame.
struct PlaneConeSection {
    std::array<double, 3> o; // Plane origin.
    std::array<double, 3> nu; // Unit plane normal.
    std::array<double, 3> apex; // Cone apex.
    std::array<double, 3> w; // Unit cone axis.
    std::array<double, 3> e1; // First unit axis normal.
    std::array<double, 3> e2; // Second unit axis normal.
    double alpha; // Half angle.
    double height; // Cone height.
    double ta; // Tangent of the half angle.
    double cosa; // Cosine of the half angle.
    double sina; // Sine of the half angle.
    double na; // Plane normal along the axis.
    double pp; // Plane normal along e1.
    double qp; // Plane normal along e2.
    double cost; // Absolute na.
    double sint; // Plane normal across the axis.
    double costa; // Cosine of the plane-to-generator angle sum.
    double d0; // Signed apex distance to the plane.

    /// Plane normal along the generator at polar angle phi, scaled by cos alpha.
    double denom(double phi) const {
        return na + ta * (pp * std::cos(phi) + qp * std::sin(phi));
    }

    /// Axial height of the section point at polar angle phi.
    double height_at(double phi) const {

        double d = denom(phi);

        return (std::abs(d) < 1e-300) ? 1e308 : -d0 / d;
    }

    /// Section point at polar angle phi.
    Point point(double phi) const {

        double s = height_at(phi);
        double rr = s * ta;
        double c = std::cos(phi);
        double sn = std::sin(phi);

        return Point(
            apex[0] + s * w[0] + rr * (c * e1[0] + sn * e2[0]),
            apex[1] + s * w[1] + rr * (c * e1[1] + sn * e2[1]),
            apex[2] + s * w[2] + rr * (c * e1[2] + sn * e2[2])
        );
    }

    /// Polar angle in [pa, pb] where the denominator reaches dtarget, by bisection.
    double refine_base(double pa, double pb, double dtarget) const {

        double fa = denom(pa) - dtarget;

        for (int it = 0; it < 60; ++it) {
            double pm = 0.5 * (pa + pb);
            double fm = denom(pm) - dtarget;

            if ((fm < 0) == (fa < 0)) {
                pa = pm;
                fa = fm;
            } else {
                pb = pm;
            }
        }

        return 0.5 * (pa + pb);
    }
};

/// Section of a recognized plane and cone; false when the cone is flat, a line or has no height.
static bool plane_cone_section(
    const RecogSurface& plane,
    const RecogSurface& cone,
    const NurbsSurface& cone_srf,
    PlaneConeSection& s
) {

    s.o = plane.p1;
    s.nu = ssi_unit(plane.p2);
    s.apex = cone.p1;
    s.w = ssi_unit(cone.p2);
    s.alpha = cone.r;

    if (s.alpha < 1e-7 || s.alpha > Tolerance::PI / 2 - 1e-7)
        return false;

    s.ta = std::tan(s.alpha);
    s.cosa = std::cos(s.alpha);
    s.sina = std::sin(s.alpha);
    s.height = cone_axial_extent(cone_srf, s.apex, s.w);

    if (s.height < 1e-12)
        return false;

    std::tie(s.e1, s.e2) = ortho_basis(s.w);
    s.na = ssi_dot(s.nu, s.w);
    s.pp = ssi_dot(s.nu, s.e1);
    s.qp = ssi_dot(s.nu, s.e2);
    s.cost = std::abs(s.na);
    s.sint = std::sqrt(std::max(0.0, s.pp * s.pp + s.qp * s.qp));
    s.costa = s.cost * s.cosa - s.sint * s.sina;
    s.d0 = (s.apex[0] - s.o[0]) * s.nu[0] + (s.apex[1] - s.o[1]) * s.nu[1] + (s.apex[2] - s.o[2]) * s.nu[2];

    return true;
}

/// Runs of consecutive in-range polar samples, closed at both ends at the cone base.
static void collect_cone_runs(
    const PlaneConeSection& s,
    const std::vector<char>& ok,
    int start,
    double dtarget,
    std::vector<std::vector<Point>>& runs
) {

    const int n = (int)ok.size();
    std::vector<Point> cur;
    bool in = false;

    for (int i = 0; i <= n; ++i) {
        int k = (start + i) % n;
        double uphi = Tolerance::TWO_PI * start / n + Tolerance::TWO_PI * i / n;
        bool v = ok[k] != 0;

        if (v && !in) {
            if (i > 0)
                cur.push_back(s.point(s.refine_base(uphi - Tolerance::TWO_PI / n, uphi, dtarget)));

            cur.push_back(s.point(uphi));
            in = true;
        } else if (v && in) {
            cur.push_back(s.point(uphi));
        } else if (!v && in) {
            cur.push_back(s.point(s.refine_base(uphi - Tolerance::TWO_PI / n, uphi, dtarget)));

            if (cur.size() >= 2)
                runs.push_back(cur);

            cur.clear();
            in = false;
        }
    }

    if (in && cur.size() >= 2)
        runs.push_back(cur);
}

/// Sample the plane-cone section as point runs, one per branch.
static void sample_plane_cone_arcs(const PlaneConeSection& s, std::vector<std::vector<Point>>& runs, bool& closed) {

    runs.clear();
    closed = false;
    const int n = 720;
    const double eps = 1e-9 * std::max(1.0, s.height);
    std::vector<char> ok(n);
    int cnt = 0;

    for (int k = 0; k < n; ++k) {
        double h = s.height_at(Tolerance::TWO_PI * k / n);
        ok[k] = (h > eps && h < s.height + eps) ? 1 : 0;
        cnt += ok[k];
    }

    if (cnt == 0)
        return;

    if (cnt == n) {
        std::vector<Point> loop;

        for (int k = 0; k <= n; ++k)
            loop.push_back(s.point(Tolerance::TWO_PI * (k % n) / n));

        runs.push_back(loop);
        closed = true;

        return;
    }

    int start = 0;

    while (start < n && ok[start])
        ++start;

    double dtarget = (s.height > 1e-300) ? (-s.d0 / s.height) : 0.0;
    collect_cone_runs(s, ok, start, dtarget, runs);
}

/// Degree-1 segment from q to q + len d.
static NurbsCurve ray_segment(const std::array<double, 3>& q, const std::array<double, 3>& d, double len) {

    Point e0(q[0], q[1], q[2]);
    Point e1(q[0] + len * d[0], q[1] + len * d[1], q[2] + len * d[2]);

    return NurbsCurve::create(false, 1, {e0, e1});
}

/// Plane through the cone apex: one tangent generator or two generator lines.
static void plane_cone_through_apex(const PlaneConeSection& s, std::vector<NurbsCurve>& out) {

    const std::array<double, 3>& nu = s.nu;
    const std::array<double, 3>& w = s.w;

    if (std::abs(s.costa) < 1e-6) {
        std::array<double, 3> g = ssi_unit(std::array<double, 3>{w[0] - s.na * nu[0], w[1] - s.na * nu[1], w[2] - s.na * nu[2]});
        double gw = ssi_dot(g, w);

        if (gw > 1e-9)
            out.push_back(ray_segment(s.apex, g, s.height / gw));

        return;
    }

    if (s.cost < s.sina) {
        std::array<double, 3> axey = ssi_cross(nu, w);
        std::array<double, 3> axex = ssi_cross(axey, nu);
        double dh = std::sqrt(std::max(0.0, s.sina * s.sina - s.cost * s.cost)) / s.cosa;

        for (int sgn : {+1, -1}) {
            std::array<double, 3> d{axex[0] + sgn * dh * axey[0], axex[1] + sgn * dh * axey[1], axex[2] + sgn * dh * axey[2]};
            double dw = ssi_dot(d, w);

            if (dw < 1e-12)
                continue;

            out.push_back(ray_segment(s.apex, d, s.height / dw));
        }
    }
}

/// Exact plane-cone conic: circle, ellipse, parabola or hyperbola; false when none fits the cone.
static bool plane_cone_exact(const PlaneConeSection& s, std::vector<NurbsCurve>& out) {

    const double ang = 1e-6;
    bool is_circle = false;
    bool is_parabola = false;
    bool is_hyperbola = false;
    bool is_ellipse = false;

    if (s.cost < ang)
        is_hyperbola = true;
    else if (std::abs(s.costa) < ang)
        is_parabola = true;
    else if (s.sint < ang)
        is_circle = true;
    else if (s.cost < s.sina)
        is_hyperbola = true;
    else
        is_ellipse = true;

    if (is_circle) {
        const std::array<double, 3>& apex = s.apex;
        const std::array<double, 3>& w = s.w;
        double dax = (s.o[0] - apex[0]) * w[0] + (s.o[1] - apex[1]) * w[1] + (s.o[2] - apex[2]) * w[2];
        double rr = std::abs(dax) * s.ta;

        if (rr > 1e-12) {
            std::array<double, 3> cc{apex[0] + dax * w[0], apex[1] + dax * w[1], apex[2] + dax * w[2]};
            NurbsCurve circ = exact_circle(cc[0], cc[1], cc[2], s.e1, s.e2, rr);

            if (conic_within_cone(circ, apex, w, s.height))
                out.push_back(circ);
        }

        return true;
    }

    NurbsCurve c3;

    if (is_ellipse && build_exact_plane_cone_ellipse(s.o, s.nu, s.apex, s.w, s.alpha, c3) && conic_within_cone(c3, s.apex, s.w, s.height)) {
        out.push_back(c3);

        return true;
    }

    if ((is_parabola || is_hyperbola) && build_exact_plane_cone_open(s.o, s.nu, s.apex, s.w, s.alpha, s.height, is_parabola, c3)) {
        out.push_back(c3);

        return true;
    }

    return false;
}

/// Plane-cone section: exact lines or conic when possible, fitted arcs otherwise.
static bool ssi_plane_cone(
    const RecogSurface& plane,
    const RecogSurface& cone,
    const NurbsSurface& cone_srf,
    std::vector<NurbsCurve>& out
) {

    PlaneConeSection s;

    if (!plane_cone_section(plane, cone, cone_srf, s))
        return false;

    if (std::abs(s.d0) < 1e-6 * std::max(1.0, s.height)) {
        plane_cone_through_apex(s, out);

        return true;
    }

    if (plane_cone_exact(s, out))
        return true;

    std::vector<std::vector<Point>> runs;
    bool closed = false;
    sample_plane_cone_arcs(s, runs, closed);

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
    std::array<double, 3> center = tor.p1;
    std::array<double, 3> w = ssi_unit(tor.p2);
    double rmaj = tor.r;
    double r = tor.r2;
    double wn = ssi_dot(w, nu);

    if (std::abs(std::abs(wn) - 1.0) > 1e-7)
        return false;

    double d = (o[0] - center[0]) * w[0] + (o[1] - center[1]) * w[1] + (o[2] - center[2]) * w[2];

    if (std::abs(d) > r)
        return true;

    double h = std::sqrt(std::max(0.0, r * r - d * d));
    std::array<double, 3> cc{center[0] + d * w[0], center[1] + d * w[1], center[2] + d * w[2]};
    std::array<double, 3> xa;
    std::array<double, 3> ya;
    std::tie(xa, ya) = ortho_basis(w);

    for (double rr : {rmaj + h, rmaj - h})
        if (rr > 1e-12)
            out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rr));

    return true;
}

/// Corner frame of a bilinear face: origin, edge vectors and their Gram matrix.
struct FaceFrame {
    std::array<double, 3> o; // Corner at (u0, v0).
    std::array<double, 3> eu; // Edge to (u1, v0).
    std::array<double, 3> ev; // Edge to (u0, v1).
    double exx; // eu . eu
    double eyy; // ev . ev
    double exy; // eu . ev
    double det; // Gram determinant.

    /// Face fractions (al, be) of the offset r from the corner.
    void fraction(const std::array<double, 3>& r, double& al, double& be) const {

        double rx = ssi_dot(r, eu);
        double ry = ssi_dot(r, ev);
        al = (eyy * rx - exy * ry) / det;
        be = (exx * ry - exy * rx) / det;
    }
};

/// Corner frame of a surface from its corners (u0, v0), (u1, v0) and (u0, v1).
static FaceFrame face_frame(const NurbsSurface& s) {

    const std::pair<double, double> domain_u = s.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = s.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    Point o = s.point_at(u0, v0);
    Point pu = s.point_at(u1, v0);
    Point pv = s.point_at(u0, v1);
    FaceFrame f;
    f.o = std::array<double, 3>{o[0], o[1], o[2]};
    f.eu = std::array<double, 3>{pu[0] - o[0], pu[1] - o[1], pu[2] - o[2]};
    f.ev = std::array<double, 3>{pv[0] - o[0], pv[1] - o[1], pv[2] - o[2]};
    f.exx = ssi_dot(f.eu, f.eu);
    f.eyy = ssi_dot(f.ev, f.ev);
    f.exy = ssi_dot(f.eu, f.ev);
    f.det = f.exx * f.eyy - f.exy * f.exy;

    return f;
}

/// Narrow [t0, t1] to where c + t d lies in [0, 1]; false when d is zero and c is outside.
static bool clip_axis(double c, double d, double& t0, double& t1) {

    if (std::abs(d) < 1e-15)
        return (c >= -1e-9 && c <= 1.0 + 1e-9);

    double ta = (0.0 - c) / d;
    double tb = (1.0 - c) / d;

    if (ta > tb)
        std::swap(ta, tb);

    t0 = std::max(t0, ta);
    t1 = std::min(t1, tb);

    return true;
}

/// Narrow [tmin, tmax] to the part of the line inside the face; empty when it misses.
static bool clip_line_to_face(
    const NurbsSurface& s,
    const std::array<double, 3>& anchor,
    const std::array<double, 3>& dir,
    double& tmin,
    double& tmax,
    bool& empty
) {

    FaceFrame f = face_frame(s);

    if (std::abs(f.det) < 1e-18)
        return false;

    double a0;
    double b0;
    double da;
    double db;
    f.fraction(std::array<double, 3>{anchor[0] - f.o[0], anchor[1] - f.o[1], anchor[2] - f.o[2]}, a0, b0);
    f.fraction(dir, da, db);
    double t0 = -1e300;
    double t1 = 1e300;

    if (!clip_axis(a0, da, t0, t1) || !clip_axis(b0, db, t0, t1) || t0 > t1) {
        empty = true;

        return false;
    }

    tmin = std::max(tmin, t0);
    tmax = std::min(tmax, t1);

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

    double da = ssi_dot(na, pa.p1);
    double db = ssi_dot(nb, pb.p1);
    std::array<double, 3> nb_x_v = ssi_cross(nb, v);
    std::array<double, 3> v_x_na = ssi_cross(v, na);
    double inv = 1.0 / (vl * vl);

    std::array<double, 3> anchor{
        (da * nb_x_v[0] + db * v_x_na[0]) * inv,
        (da * nb_x_v[1] + db * v_x_na[1]) * inv,
        (da * nb_x_v[2] + db * v_x_na[2]) * inv
    };

    std::array<double, 3> dir{v[0] / vl, v[1] / vl, v[2] / vl};
    double tmin = -1e300;
    double tmax = 1e300;

    if (!clip_line_to_face(sa, anchor, dir, tmin, tmax, empty) || !clip_line_to_face(sb, anchor, dir, tmin, tmax, empty))
        return false;

    if (tmax - tmin <= 1e-9) {
        empty = true;

        return false;
    }

    Point start(anchor[0] + tmin * dir[0], anchor[1] + tmin * dir[1], anchor[2] + tmin * dir[2]);
    Point end(anchor[0] + tmax * dir[0], anchor[1] + tmax * dir[1], anchor[2] + tmax * dir[2]);
    c3 = NurbsCurve::create(false, 1, {start, end});
    c3.set_domain(0.0, 1.0);

    return true;
}

/// Tri-state analytic result: not analytic, recognised empty, or curve triples.
struct AnalyticResult {
    enum { NOT_ANALYTIC, NO_HIT, HIT } status = NOT_ANALYTIC; // Whether both surfaces were recognized and whether they meet.
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> triples; // 3D curve with both pullbacks.
};

/// Angle shifted by whole turns to within half a turn of prev.
static double unwrap_angle(double a, double prev) {

    while (a - prev > Tolerance::PI)
        a -= Tolerance::TWO_PI;

    while (a - prev < -Tolerance::PI)
        a += Tolerance::TWO_PI;

    return a;
}

/// Angle shifted by whole turns into [-pi, pi].
static double wrap_angle(double a) {

    while (a > Tolerance::PI)
        a -= Tolerance::TWO_PI;

    while (a < -Tolerance::PI)
        a += Tolerance::TWO_PI;

    return a;
}

/// Angle shifted by whole turns into [lo - 1e-9, hi + 1e-9] when the range allows.
static double wrap_to_range(double a, double lo, double hi) {

    while (a < lo - 1e-9)
        a += Tolerance::TWO_PI;

    while (a > hi + 1e-9)
        a -= Tolerance::TWO_PI;

    return a;
}

/// Value shifted by whole periods to within half a period of prev.
static double unwrap_period(double x, double prev, double period) {

    while (x - prev > period * 0.5)
        x -= period;

    while (x - prev < -period * 0.5)
        x += period;

    return x;
}

/// Index of the period cell of x counted from x0.
static int period_index(double x, double x0, double period) {
    return (int)std::floor((x - x0) / period + 1e-9);
}

/// Height of p along the unit axis through origin.
static double axis_height(const Point& p, const std::array<double, 3>& origin, const std::array<double, 3>& axis) {

    std::array<double, 3> r{p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]};

    return ssi_dot(r, axis);
}

/// Squared distance of p from the unit axis through origin.
static double axis_radial_sq(const Point& p, const std::array<double, 3>& origin, const std::array<double, 3>& axis) {

    std::array<double, 3> r{p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]};
    double h = ssi_dot(r, axis);
    double px = r[0] - h * axis[0];
    double py = r[1] - h * axis[1];
    double pz = r[2] - h * axis[2];

    return px * px + py * py + pz * pz;
}

/// Distance between the curve's end points.
static double curve_gap(const NurbsCurve& c) {

    const std::pair<double, double> domain = c.domain();

    return c.point_at(domain.first).distance(c.point_at(domain.second));
}

/// Constant-v UV line from u0 to u1.
static NurbsCurve iso_v_line(double u0, double u1, double vc) {
    return NurbsCurve::create(false, 1, {Point(u0, vc, 0.0), Point(u1, vc, 0.0)});
}

/// Height range and mean of 33 curve samples along the unit axis through origin.
static void curve_height_stats(
    const NurbsCurve& c3d,
    const std::array<double, 3>& origin,
    const std::array<double, 3>& axis,
    double& hmin,
    double& hmax,
    double& hmean
) {

    const std::pair<double, double> domain = c3d.domain();
    const double t0 = domain.first;
    const double t1 = domain.second;
    const int ns = 33;
    double hsum = 0;
    hmin = 1e300;
    hmax = -1e300;

    for (int i = 0; i < ns; ++i) {
        double h = axis_height(c3d.point_at(t0 + (t1 - t0) * i / 32), origin, axis);
        hmin = std::min(hmin, h);
        hmax = std::max(hmax, h);
        hsum += h;
    }

    hmean = hsum / ns;
}

/// v on the line u = um where the axial height reaches hc, by bisection; false when hc is outside.
static bool bisect_height_v(
    const NurbsSurface& srf,
    double um,
    double v0,
    double v1,
    double hc,
    const std::array<double, 3>& origin,
    const std::array<double, 3>& axis,
    double& vc
) {

    double va = v0;
    double vb = v1;
    double ha = axis_height(srf.point_at(um, va), origin, axis);
    double hb = axis_height(srf.point_at(um, vb), origin, axis);

    if ((hc - ha) * (hc - hb) > 0)
        return false;

    for (int it = 0; it < 60; ++it) {
        double vm = 0.5 * (va + vb);
        double hm = axis_height(srf.point_at(um, vm), origin, axis);

        if ((hm - hc) * (ha - hc) <= 0) {
            vb = vm;
        } else {
            va = vm;
            ha = hm;
        }
    }

    vc = 0.5 * (va + vb);

    return true;
}

/// Plane pcurve: the curve's control points mapped to the face's bilinear parameters.
static NurbsCurve plane_pcurve(const NurbsSurface& srf, const NurbsCurve& c3d) {

    const std::pair<double, double> domain_u = srf.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = srf.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    FaceFrame f = face_frame(srf);

    if (std::abs(f.det) < 1e-18)
        return NurbsCurve();

    NurbsCurve pc = c3d;

    for (int i = 0; i < c3d.cv_count(); ++i) {
        Point cv = c3d.get_cv(i);
        double a;
        double b;
        f.fraction(std::array<double, 3>{cv[0] - f.o[0], cv[1] - f.o[1], cv[2] - f.o[2]}, a, b);
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

/// Cylinder pcurve of a circle perpendicular to the axis: a constant-v line.
static NurbsCurve cylinder_pcurve(const NurbsSurface& srf, const RecogSurface& recog, const NurbsCurve& c3d) {

    const std::pair<double, double> domain_u = srf.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = srf.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    std::array<double, 3> ax = recog.p2;

    if (!normalize_axis(ax))
        return NurbsCurve();

    double um = 0.5 * (u0 + u1);
    double h0 = axis_height(srf.point_at(um, v0), recog.p1, ax);
    double h1 = axis_height(srf.point_at(um, v1), recog.p1, ax);

    if (std::abs(h1 - h0) < 1e-12)
        return NurbsCurve();

    double hmin;
    double hmax;
    double hc;
    curve_height_stats(c3d, recog.p1, ax, hmin, hmax, hc);

    if (hmax - hmin > 1e-5 * std::abs(h1 - h0))
        return NurbsCurve();

    if (curve_gap(c3d) > 1e-6 * (std::abs(h1 - h0) + 1.0))
        return NurbsCurve();

    double vc = v0 + (hc - h0) / (h1 - h0) * (v1 - v0);

    if (vc < std::min(v0, v1) - 1e-9 || vc > std::max(v0, v1) + 1e-9)
        return NurbsCurve();

    return iso_v_line(u0, u1, vc);
}

/// Sphere pcurve of a latitude circle: a constant-v line.
static NurbsCurve sphere_pcurve(const NurbsSurface& srf, const RecogSurface& recog, const NurbsCurve& c3d) {

    const std::pair<double, double> domain_u = srf.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = srf.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    double um = 0.5 * (u0 + u1);
    Point sp = srf.point_at(um, v0);
    Point np = srf.point_at(um, v1);
    std::array<double, 3> ax{np[0] - sp[0], np[1] - sp[1], np[2] - sp[2]};

    if (!normalize_axis(ax))
        return NurbsCurve();

    double hmin;
    double hmax;
    double hc;
    curve_height_stats(c3d, recog.p1, ax, hmin, hmax, hc);

    if (hmax - hmin > recog.r * 1e-4)
        return NurbsCurve();

    if (curve_gap(c3d) > recog.r * 1e-3)
        return NurbsCurve();

    double vc;

    if (!bisect_height_v(srf, um, v0, v1, hc, recog.p1, ax, vc))
        return NurbsCurve();

    return iso_v_line(u0, u1, vc);
}

/// Cone pcurve of a circle perpendicular to the axis: a constant-v line.
static NurbsCurve cone_pcurve(const NurbsSurface& srf, const RecogSurface& recog, const NurbsCurve& c3d) {

    const std::pair<double, double> domain_u = srf.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = srf.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    std::array<double, 3> ax = recog.p2;

    if (!normalize_axis(ax))
        return NurbsCurve();

    const std::pair<double, double> domain = c3d.domain();
    const double t0 = domain.first;
    const double t1 = domain.second;
    double clen = c3d.point_at(t0).distance(c3d.point_at(0.5 * (t0 + t1)));
    double hscale = std::max(clen, 1e-9);
    double hmin;
    double hmax;
    double hc;
    curve_height_stats(c3d, recog.p1, ax, hmin, hmax, hc);

    if (hmax - hmin > hscale * 1e-4)
        return NurbsCurve();

    if (curve_gap(c3d) > hscale * 1e-3)
        return NurbsCurve();

    double vc;

    if (!bisect_height_v(srf, 0.5 * (u0 + u1), v0, v1, hc, recog.p1, ax, vc))
        return NurbsCurve();

    return iso_v_line(u0, u1, vc);
}

/// Tube angle of p about the circle of radius rmaj around the unit axis w through center.
static double torus_minor_angle(const Point& p, const std::array<double, 3>& center, const std::array<double, 3>& w, double rmaj) {

    std::array<double, 3> d{p[0] - center[0], p[1] - center[1], p[2] - center[2]};
    double z = ssi_dot(d, w);
    double hx = d[0] - z * w[0];
    double hy = d[1] - z * w[1];
    double hz = d[2] - z * w[2];
    double rho = std::sqrt(hx * hx + hy * hy + hz * hz);

    return std::atan2(z, rho - rmaj);
}

/// Range and mean of the unwrapped tube angle over 33 curve samples.
static void torus_angle_stats(
    const NurbsCurve& c3d,
    const std::array<double, 3>& center,
    const std::array<double, 3>& w,
    double rmaj,
    double& amin,
    double& amax,
    double& amean
) {

    const std::pair<double, double> domain = c3d.domain();
    const double t0 = domain.first;
    const double t1 = domain.second;
    const int ns = 33;
    double aprev = 0.0;
    double asum = 0.0;
    amin = 1e300;
    amax = -1e300;

    for (int i = 0; i < ns; ++i) {
        double a = torus_minor_angle(c3d.point_at(t0 + (t1 - t0) * i / 32), center, w, rmaj);

        if (i > 0)
            a = unwrap_angle(a, aprev);

        aprev = a;
        amin = std::min(amin, a);
        amax = std::max(amax, a);
        asum += a;
    }

    amean = asum / ns;
}

/// Unwrapped tube angle at 257 samples of the line u = um.
static void torus_angle_table(
    const NurbsSurface& srf,
    double um,
    double v0,
    double v1,
    const std::array<double, 3>& center,
    const std::array<double, 3>& w,
    double rmaj,
    std::vector<double>& tv,
    std::vector<double>& ta
) {

    const int nv = 256;
    tv.assign(nv + 1, 0.0);
    ta.assign(nv + 1, 0.0);
    double ap = 0.0;

    for (int k = 0; k <= nv; ++k) {
        double v = v0 + (v1 - v0) * k / nv;
        double a = torus_minor_angle(srf.point_at(um, v), center, w, rmaj);

        if (k > 0)
            a = unwrap_angle(a, ap);

        ap = a;
        tv[k] = v;
        ta[k] = a;
    }
}

/// Parameter and value arrays read backwards: the x where the tabulated y reaches y.
static double inverse_table(const std::vector<double>& xs, const std::vector<double>& ys, double y) {

    const int nt = (int)ys.size() - 1;
    bool incr = ys[nt] >= ys[0];
    y = wrap_to_range(y, std::min(ys[0], ys[nt]), std::max(ys[0], ys[nt]));
    int lo = 0;
    int hi = nt;

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
}

/// Newton-refine v on the line u = um so the tube angle reaches a_target.
static double torus_refine_v(
    const NurbsSurface& srf,
    double um,
    double vc,
    double a_target,
    double v0,
    double v1,
    const std::array<double, 3>& center,
    const std::array<double, 3>& w,
    double rmaj
) {

    const double dv = (v1 - v0) * 1e-7;
    const double vlo = std::min(v0, v1);
    const double vhi = std::max(v0, v1);

    for (int np = 0; np < 3; ++np) {
        double g0 = wrap_angle(torus_minor_angle(srf.point_at(um, std::min(std::max(vc, vlo), vhi)), center, w, rmaj) - a_target);
        double vd = std::min(vc + dv, vhi);
        double g1 = wrap_angle(torus_minor_angle(srf.point_at(um, std::min(std::max(vd, vlo), vhi)), center, w, rmaj) - a_target);
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

    return vc;
}

/// Torus pcurve of a circle of constant tube angle: a constant-v line.
static NurbsCurve torus_pcurve(const NurbsSurface& srf, const RecogSurface& recog, const NurbsCurve& c3d) {

    const std::pair<double, double> domain_u = srf.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = srf.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    std::array<double, 3> w = recog.p2;

    if (!normalize_axis(w))
        return NurbsCurve();

    double rmaj = recog.r;
    double rmin = recog.r2;

    if (rmin < 1e-12 || rmaj <= rmin)
        return NurbsCurve();

    double amin;
    double amax;
    double a_target;
    torus_angle_stats(c3d, recog.p1, w, rmaj, amin, amax, a_target);

    if (amax - amin > 1e-4)
        return NurbsCurve();

    if (curve_gap(c3d) > rmin * 1e-3)
        return NurbsCurve();

    double um = 0.5 * (u0 + u1);
    std::vector<double> tv;
    std::vector<double> ta;
    torus_angle_table(srf, um, v0, v1, recog.p1, w, rmaj, tv, ta);
    double alo = std::min(ta.front(), ta.back());
    double ahi = std::max(ta.front(), ta.back());
    a_target = wrap_to_range(a_target, alo, ahi);

    if (a_target < alo - 1e-9 || a_target > ahi + 1e-9)
        return NurbsCurve();

    double vc = torus_refine_v(srf, um, inverse_table(tv, ta, a_target), a_target, v0, v1, recog.p1, w, rmaj);

    return iso_v_line(u0, u1, vc);
}

/// Analytic pcurve of an exact 3D intersection conic on a recognized quadric surface.
static NurbsCurve analytic_pcurve(const NurbsSurface& srf, const RecogSurface& recog, const NurbsCurve& c3d) {

    if (recog.kind == RecogSurface::PLANE)
        return plane_pcurve(srf, c3d);

    if (recog.kind == RecogSurface::CYLINDER)
        return cylinder_pcurve(srf, recog, c3d);

    if (recog.kind == RecogSurface::SPHERE)
        return sphere_pcurve(srf, recog, c3d);

    if (recog.kind == RecogSurface::CONE)
        return cone_pcurve(srf, recog, c3d);

    if (recog.kind == RecogSurface::TORUS)
        return torus_pcurve(srf, recog, c3d);

    return NurbsCurve();
}

/// Orthonormal frame about a surface axis.
struct AxisFrame {
    std::array<double, 3> o; // Origin on the axis.
    std::array<double, 3> x; // First radial direction.
    std::array<double, 3> y; // Second radial direction.
    std::array<double, 3> z; // Unit axis.
};

/// Frame about the unit axis z through origin with x towards p; false when p lies on the axis.
static bool axis_frame(const std::array<double, 3>& origin, const std::array<double, 3>& z, const Point& p, AxisFrame& f) {

    std::array<double, 3> r{p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]};
    double h = ssi_dot(r, z);
    std::array<double, 3> x{r[0] - h * z[0], r[1] - h * z[1], r[2] - h * z[2]};

    if (!normalize_axis(x))
        return false;

    f.o = origin;
    f.x = x;
    f.y = ssi_cross(z, x);
    f.z = z;

    return true;
}

/// Longitude of q about the frame axis.
static double frame_longitude(const AxisFrame& f, const Point& q) {

    std::array<double, 3> r{q[0] - f.o[0], q[1] - f.o[1], q[2] - f.o[2]};

    return std::atan2(ssi_dot(r, f.y), ssi_dot(r, f.x));
}

/// Torus tube angle of q for major radius rmaj and minor radius rmin.
static double frame_tube_angle(const AxisFrame& f, double rmaj, double rmin, const Point& q) {

    double rho = std::sqrt(axis_radial_sq(q, f.o, f.z));

    return std::atan2(axis_height(q, f.o, f.z) / rmin, (rho - rmaj) / rmin);
}

/// Angle of surface points along one parameter line: longitude, or the torus tube angle.
struct AngleProbe {
    const NurbsSurface& srf; // Sampled surface.
    AxisFrame frame; // Frame about the surface axis.
    double fixed; // The parameter held fixed.
    bool x_is_u; // Whether the free parameter is u.
    bool tube; // Tube angle instead of longitude.
    double rmaj; // Torus major radius.
    double rmin; // Torus minor radius.

    /// Surface point at free parameter x.
    Point point(double x) const {
        return x_is_u ? srf.point_at(x, fixed) : srf.point_at(fixed, x);
    }

    /// Angle at free parameter x.
    double angle(double x) const {

        Point q = point(x);

        return tube ? frame_tube_angle(frame, rmaj, rmin, q) : frame_longitude(frame, q);
    }
};

/// Tabulated angle along one parameter line.
struct AngleMap {
    AngleProbe probe; // Angle along the parameter line.
    double lo; // Parameter start.
    double hi; // Parameter end.
    std::vector<double> xs; // Tabulated parameters.
    std::vector<double> ys; // Unwrapped angles at xs.
};

/// Tabulate 129 unwrapped angles of the probe over [lo, hi].
static AngleMap angle_map(const AngleProbe& probe, double lo, double hi) {

    const int nt = 128;
    const double range = hi - lo;
    AngleMap m{probe, lo, hi, std::vector<double>(nt + 1), std::vector<double>(nt + 1)};

    for (int k = 0; k <= nt; ++k) {
        double x = lo + range * k / nt;
        double y = probe.angle(x);

        if (k > 0)
            y = unwrap_angle(y, m.ys[k - 1]);

        m.xs[k] = x;
        m.ys[k] = y;
    }

    return m;
}

/// Two Newton steps moving x in [lo, hi] until the probe angle reaches y.
static double polish_angle(const AngleProbe& probe, double x, double y, double lo, double hi) {

    const double dx = (hi - lo) * 1e-7;

    for (int np = 0; np < 2; ++np) {
        double xc = std::min(std::max(x, lo), hi);
        double g0 = wrap_angle(probe.angle(xc) - y);
        double g1 = wrap_angle(probe.angle(std::min(xc + dx, hi)) - y);
        double dg = (g1 - g0) / dx;

        if (std::abs(dg) < 1e-12)
            break;

        x = std::min(std::max(xc - g0 / dg, lo), hi);
    }

    return x;
}

/// Parameter where the tabulated angle reaches y, Newton-polished.
static double map_parameter(const AngleMap& m, double y) {
    return polish_angle(m.probe, inverse_table(m.xs, m.ys, y), y, m.lo, m.hi);
}

/// Height along the frame axis at 129 samples of the line u = um.
static void height_table(
    const NurbsSurface& srf,
    const AxisFrame& f,
    double um,
    double v0,
    double v1,
    std::vector<double>& tv,
    std::vector<double>& th
) {

    const int nt = 128;
    tv.assign(nt + 1, 0.0);
    th.assign(nt + 1, 0.0);

    for (int k = 0; k <= nt; ++k) {
        double v = v0 + (v1 - v0) * k / nt;
        tv[k] = v;
        th[k] = axis_height(srf.point_at(um, v), f.o, f.z);
    }
}

/// The x where the tabulated y reaches y, clamped to the table ends.
static double clamped_table(const std::vector<double>& xs, const std::vector<double>& ys, double y) {

    const int nt = (int)ys.size() - 1;
    bool incr = ys[nt] >= ys[0];

    if (incr && y <= ys[0])
        return xs[0];

    if (incr && y >= ys[nt])
        return xs[nt];

    if (!incr && y >= ys[0])
        return xs[0];

    if (!incr && y <= ys[nt])
        return xs[nt];

    int lo = 0;
    int hi = nt;

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
}

/// Two Newton steps moving v on the line u = um until the axial height reaches h.
static double sphere_refine_v(const NurbsSurface& srf, const AxisFrame& f, double um, double v, double h, double v0, double v1) {

    const double vlo = std::min(v0, v1);
    const double vhi = std::max(v0, v1);

    for (int np = 0; np < 2; ++np) {
        double dv = (v1 - v0) * 1e-7;
        double vc = std::min(std::max(v, vlo), vhi);
        double g0 = axis_height(srf.point_at(um, vc), f.o, f.z) - h;
        double g1 = axis_height(srf.point_at(um, std::min(vc + dv, vhi)), f.o, f.z) - h;
        double dg = (g1 - g0) / dv;

        if (std::abs(dg) < 1e-12)
            break;

        v = std::min(std::max(vc - g0 / dg, vlo), vhi);
    }

    return v;
}

/// Degree-1 pcurves of (u, v) samples with u unwrapped, split where u crosses the seam.
static std::vector<NurbsCurve> split_pullback_u(const std::vector<std::array<double, 2>>& uv, double u0, double range_u) {

    std::vector<NurbsCurve> out;
    std::vector<Point> seg;
    int cur_k = period_index(uv[0][0], u0, range_u);
    seg.push_back(Point(uv[0][0] - cur_k * range_u, uv[0][1], 0.0));

    for (size_t i = 1; i < uv.size(); ++i) {
        int ki = period_index(uv[i][0], u0, range_u);

        while (ki != cur_k) {
            int step = (ki > cur_k) ? 1 : -1;
            int nk = cur_k + step;
            double seam_cont = u0 + (step > 0 ? nk : cur_k) * range_u;
            double denom = uv[i][0] - uv[i - 1][0];
            double f = (std::abs(denom) > 1e-15) ? (seam_cont - uv[i - 1][0]) / denom : 0.0;
            f = std::min(std::max(f, 0.0), 1.0);
            double vc = uv[i - 1][1] + (uv[i][1] - uv[i - 1][1]) * f;
            seg.push_back(Point(seam_cont - cur_k * range_u, vc, 0.0));

            if (seg.size() >= 2)
                out.push_back(NurbsCurve::create(false, 1, seg));

            seg.clear();
            seg.push_back(Point(seam_cont - nk * range_u, vc, 0.0));
            cur_k = nk;
        }

        seg.push_back(Point(uv[i][0] - cur_k * range_u, uv[i][1], 0.0));
    }

    if (seg.size() >= 2)
        out.push_back(NurbsCurve::create(false, 1, seg));

    return out;
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

    double um = 0.5 * (u0 + u1);
    double vm = 0.5 * (v0 + v1);
    Point sp = srf.point_at(um, v0);
    Point np = srf.point_at(um, v1);
    std::array<double, 3> axis{np[0] - sp[0], np[1] - sp[1], np[2] - sp[2]};
    AxisFrame frame;

    if (!normalize_axis(axis) || !axis_frame(recog.p1, axis, srf.point_at(u0, vm), frame))
        return {};

    AngleMap lon_map = angle_map(AngleProbe{srf, frame, vm, true, false, 0.0, 0.0}, u0, u1);
    std::vector<double> tv;
    std::vector<double> th;
    height_table(srf, frame, um, v0, v1, tv, th);

    if (std::abs(th.back() - th.front()) < 1e-12)
        return {};

    const std::pair<double, double> domain = c3d.domain();
    const double t0 = domain.first;
    const double t1 = domain.second;
    int n = std::max(c3d.cv_count() * 8, 120);
    std::vector<std::array<double, 2>> uv;
    double prev_u = 0.0;

    for (int i = 0; i <= n; ++i) {
        Point p = c3d.point_at(t0 + (t1 - t0) * i / n);
        double h = axis_height(p, frame.o, frame.z);
        double u = map_parameter(lon_map, frame_longitude(frame, p));
        double v = sphere_refine_v(srf, frame, um, clamped_table(tv, th, h), h, v0, v1);

        if (i > 0)
            u = unwrap_period(u, prev_u, range_u);

        prev_u = u;
        uv.push_back({u, v});
    }

    return split_pullback_u(uv, u0, range_u);
}

/// Samples (u, v) of a curve on a cone or cylinder, u unwrapped and v linear in the axial height.
static std::vector<std::array<double, 2>> cone_pullback_samples(
    const NurbsCurve& c3d,
    const AxisFrame& frame,
    const AngleMap& lon_map,
    double h0,
    double h1,
    double v0,
    double v1
) {

    const std::pair<double, double> domain = c3d.domain();
    const double t0 = domain.first;
    const double t1 = domain.second;
    const double range_u = lon_map.hi - lon_map.lo;
    int n = std::max(c3d.cv_count() * 8, 120);
    double prev_lon = 0.0;
    std::vector<std::array<double, 2>> uv;
    double prev_u = 0.0;

    for (int i = 0; i <= n; ++i) {
        Point p = c3d.point_at(t0 + (t1 - t0) * i / n);
        std::array<double, 3> r{p[0] - frame.o[0], p[1] - frame.o[1], p[2] - frame.o[2]};
        double rx = ssi_dot(r, frame.x);
        double ry = ssi_dot(r, frame.y);
        double rad = std::sqrt(std::max(0.0, rx * rx + ry * ry));
        double lon = (rad > 1e-12) ? std::atan2(ry, rx) : prev_lon;
        prev_lon = lon;
        double u = (rad > 1e-12) ? map_parameter(lon_map, lon) : inverse_table(lon_map.xs, lon_map.ys, lon);
        double v = v0 + (ssi_dot(r, frame.z) - h0) / (h1 - h0) * (v1 - v0);

        if (i > 0)
            u = unwrap_period(u, prev_u, range_u);

        prev_u = u;
        uv.push_back({u, v});
    }

    return uv;
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
    std::array<double, 3> axis = recog.p2;

    if (range_u < 1e-9 || !normalize_axis(axis))
        return {};

    double um = 0.5 * (u0 + u1);
    double h0 = axis_height(srf.point_at(um, v0), recog.p1, axis);
    double h1 = axis_height(srf.point_at(um, v1), recog.p1, axis);

    if (std::abs(h1 - h0) < 1e-12)
        return {};

    double v_ref = (std::abs(h0) >= std::abs(h1)) ? v0 : v1;
    AxisFrame frame;

    if (!axis_frame(recog.p1, axis, srf.point_at(u0, v_ref), frame))
        return {};

    AngleMap lon_map = angle_map(AngleProbe{srf, frame, v_ref, true, false, 0.0, 0.0}, u0, u1);
    std::vector<std::array<double, 2>> uv = cone_pullback_samples(c3d, frame, lon_map, h0, h1, v0, v1);

    return split_pullback_u(uv, u0, range_u);
}

/// Period cells of a torus pull-back in (a, b), swapped when a is the surface v.
struct PeriodGrid {
    double a0; // Start of a.
    double range_a; // Period of a.
    double b0; // Start of b.
    double range_b; // Period of b.
    bool swapped; // Whether a is the surface v.
};

/// Append the point (a, b) shifted into cell (ka, kb) in surface (u, v) order.
static void push_pullback_point(const PeriodGrid& g, std::vector<Point>& seg, double a, double b, int ka, int kb) {

    double uu = a - ka * g.range_a;
    double vv = b - kb * g.range_b;
    seg.push_back(g.swapped ? Point(vv, uu, 0.0) : Point(uu, vv, 0.0));
}

/// Split the step p -> q at its first cell boundary, advancing p; false when q is in the current cell.
static bool cross_period(
    const PeriodGrid& g,
    std::array<double, 2>& p,
    const std::array<double, 2>& q,
    int& ka,
    int& kb,
    std::vector<Point>& seg,
    std::vector<NurbsCurve>& out
) {

    int kqa = period_index(q[0], g.a0, g.range_a);
    int kqb = period_index(q[1], g.b0, g.range_b);

    if (kqa == ka && kqb == kb)
        return false;

    double fa = 2.0;
    double fb = 2.0;
    int sa = 0;
    int sb = 0;

    if (kqa != ka) {
        sa = kqa > ka ? 1 : -1;
        double bound = g.a0 + (sa > 0 ? ka + 1 : ka) * g.range_a;
        double den = q[0] - p[0];
        fa = std::abs(den) > 1e-15 ? (bound - p[0]) / den : 0.0;
    }

    if (kqb != kb) {
        sb = kqb > kb ? 1 : -1;
        double bound = g.b0 + (sb > 0 ? kb + 1 : kb) * g.range_b;
        double den = q[1] - p[1];
        fb = std::abs(den) > 1e-15 ? (bound - p[1]) / den : 0.0;
    }

    std::array<double, 2> c;

    if (fa <= fb)
        c = {g.a0 + (sa > 0 ? ka + 1 : ka) * g.range_a, p[1] + (q[1] - p[1]) * std::min(std::max(fa, 0.0), 1.0)};
    else
        c = {p[0] + (q[0] - p[0]) * std::min(std::max(fb, 0.0), 1.0), g.b0 + (sb > 0 ? kb + 1 : kb) * g.range_b};

    push_pullback_point(g, seg, c[0], c[1], ka, kb);

    if (seg.size() >= 2)
        out.push_back(NurbsCurve::create(false, 1, seg));

    seg.clear();

    if (fa <= fb)
        ka += sa;
    else
        kb += sb;

    push_pullback_point(g, seg, c[0], c[1], ka, kb);
    p = c;

    return true;
}

/// Degree-1 pcurves of (a, b) samples with a and b unwrapped, split at both seams.
static std::vector<NurbsCurve> split_pullback_ab(const std::vector<std::array<double, 2>>& ab, const PeriodGrid& g) {

    std::vector<NurbsCurve> out;
    std::vector<Point> seg;
    int ka = period_index(ab[0][0], g.a0, g.range_a);
    int kb = period_index(ab[0][1], g.b0, g.range_b);
    push_pullback_point(g, seg, ab[0][0], ab[0][1], ka, kb);

    for (size_t i = 1; i < ab.size(); ++i) {
        std::array<double, 2> p = ab[i - 1];

        for (int guard = 0; guard < 8; ++guard)
            if (!cross_period(g, p, ab[i], ka, kb, seg, out))
                break;

        push_pullback_point(g, seg, ab[i][0], ab[i][1], ka, kb);
    }

    if (seg.size() >= 2)
        out.push_back(NurbsCurve::create(false, 1, seg));

    return out;
}

/// Sample of a 5 x 5 grid farthest from the unit axis through center.
static Point farthest_from_axis(const NurbsSurface& srf, const std::array<double, 3>& center, const std::array<double, 3>& axis) {

    const std::pair<double, double> domain_u = srf.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = srf.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    Point pf = srf.point_at(u0, v0);
    double best = -1.0;

    for (int i = 0; i <= 4; ++i) {
        for (int j = 0; j <= 4; ++j) {
            Point q = srf.point_at(u0 + (u1 - u0) * i / 4.0, v0 + (v1 - v0) * j / 4.0);
            double d = axis_radial_sq(q, center, axis);

            if (d > best) {
                best = d;
                pf = q;
            }
        }
    }

    return pf;
}

/// Whether the torus's longitude runs along v rather than u.
static bool torus_swapped(const NurbsSurface& srf, const AxisFrame& frame) {

    const std::pair<double, double> domain_u = srf.domain(0);
    const double u0 = domain_u.first;
    const double u1 = domain_u.second;
    const std::pair<double, double> domain_v = srf.domain(1);
    const double v0 = domain_v.first;
    const double v1 = domain_v.second;
    double um = 0.5 * (u0 + u1);
    double vm = 0.5 * (v0 + v1);
    double lu1 = frame_longitude(frame, srf.point_at(u0 + 0.6 * (u1 - u0), vm));
    double lu0 = frame_longitude(frame, srf.point_at(u0 + 0.3 * (u1 - u0), vm));
    double lv1 = frame_longitude(frame, srf.point_at(um, v0 + 0.6 * (v1 - v0)));
    double lv0 = frame_longitude(frame, srf.point_at(um, v0 + 0.3 * (v1 - v0)));

    return std::abs(wrap_angle(lv1 - lv0)) > std::abs(wrap_angle(lu1 - lu0));
}

/// Free parameter of 17 samples along the probe line farthest from the axis.
static double farthest_on_line(const AngleProbe& probe, double lo, double hi) {

    const double range = hi - lo;
    double x_ref = lo;
    double best = -1.0;

    for (int j = 0; j <= 16; ++j) {
        double x = lo + range * j / 16.0;
        double d = axis_radial_sq(probe.point(x), probe.frame.o, probe.frame.z);

        if (d > best) {
            best = d;
            x_ref = x;
        }
    }

    return x_ref;
}

/// Analytic pull-back of a 3D curve onto a recognized torus.
static std::vector<NurbsCurve> analytic_torus_pullback(
    const NurbsSurface& srf,
    const RecogSurface& recog,
    const NurbsCurve& c3d
) {

    const std::pair<double, double> domain_u = srf.domain(0);
    const std::pair<double, double> domain_v = srf.domain(1);
    std::array<double, 3> axis = recog.p2;
    double rmaj = recog.r;
    double rmin = recog.r2;

    if (recog.kind != RecogSurface::TORUS || domain_u.second - domain_u.first < 1e-9 || domain_v.second - domain_v.first < 1e-9)
        return {};

    if (!normalize_axis(axis) || rmaj < 1e-12 || rmin < 1e-12)
        return {};

    AxisFrame frame;

    if (!axis_frame(recog.p1, axis, farthest_from_axis(srf, recog.p1, axis), frame))
        return {};

    bool swapped = torus_swapped(srf, frame);
    const std::pair<double, double> domain_a = swapped ? domain_v : domain_u;
    const std::pair<double, double> domain_b = swapped ? domain_u : domain_v;
    double a0 = domain_a.first;
    double a1 = domain_a.second;
    double b0 = domain_b.first;
    double b1 = domain_b.second;
    AngleProbe tube_probe{srf, frame, 0.5 * (a0 + a1), swapped, true, rmaj, rmin};
    double b_ref = farthest_on_line(tube_probe, b0, b1);
    AngleMap lon_map = angle_map(AngleProbe{srf, frame, b_ref, !swapped, false, 0.0, 0.0}, a0, a1);
    AngleMap tube_map = angle_map(tube_probe, b0, b1);
    const std::pair<double, double> domain = c3d.domain();
    const double t0 = domain.first;
    const double t1 = domain.second;
    int n = std::max(c3d.cv_count() * 8, 4000);
    std::vector<std::array<double, 2>> ab;
    double prev_a = 0.0;
    double prev_b = 0.0;

    for (int i = 0; i <= n; ++i) {
        Point q = c3d.point_at(t0 + (t1 - t0) * i / n);
        double a = map_parameter(lon_map, frame_longitude(frame, q));
        double b = map_parameter(tube_map, frame_tube_angle(frame, rmaj, rmin, q));

        if (i > 0) {
            a = unwrap_period(a, prev_a, a1 - a0);
            b = unwrap_period(b, prev_b, b1 - b0);
        }

        prev_a = a;
        prev_b = b;
        ab.push_back({a, b});
    }

    return split_pullback_ab(ab, PeriodGrid{a0, a1 - a0, b0, b1 - b0, swapped});
}

/// Analytic pull-back matching the recognized kind: sphere, cone or cylinder, torus.
static std::vector<NurbsCurve> analytic_pullback(const NurbsSurface& srf, const RecogSurface& recog, const NurbsCurve& c3d) {

    if (recog.kind == RecogSurface::TORUS)
        return analytic_torus_pullback(srf, recog, c3d);

    if (recog.kind == RecogSurface::SPHERE)
        return analytic_sphere_pullback(srf, recog, c3d);

    if (recog.kind == RecogSurface::CONE || recog.kind == RecogSurface::CYLINDER)
        return analytic_cone_pullback(srf, recog, c3d);

    return {};
}

// ═══════════════════════════════════════════════════════════════════════════
// Coaxial quadric pairs
// ═══════════════════════════════════════════════════════════════════════════

/// Distance of p from the axis through apt along adir.
static double point_axis_dist(const std::array<double, 3>& apt, const std::array<double, 3>& adir, const std::array<double, 3>& p) {

    std::array<double, 3> u = ssi_unit(adir);
    std::array<double, 3> dp{p[0] - apt[0], p[1] - apt[1], p[2] - apt[2]};
    double t = ssi_dot(dp, u);
    std::array<double, 3> perp{dp[0] - t * u[0], dp[1] - t * u[1], dp[2] - t * u[2]};

    return std::sqrt(ssi_dot(perp, perp));
}

/// Coordinate of p along the axis through apt along adir.
static double axial_coord(const std::array<double, 3>& apt, const std::array<double, 3>& adir, const std::array<double, 3>& p) {

    std::array<double, 3> u = ssi_unit(adir);

    return (p[0] - apt[0]) * u[0] + (p[1] - apt[1]) * u[1] + (p[2] - apt[2]) * u[2];
}

/// Whether two axes coincide within tol.
static bool axes_coaxial(
    const std::array<double, 3>& p1,
    const std::array<double, 3>& d1,
    const std::array<double, 3>& p2,
    const std::array<double, 3>& d2,
    double tol
) {

    std::array<double, 3> u1 = ssi_unit(d1);
    std::array<double, 3> u2 = ssi_unit(d2);
    std::array<double, 3> cx = ssi_cross(u1, u2);

    if (std::sqrt(ssi_dot(cx, cx)) > tol)
        return false;

    return point_axis_dist(p1, u1, p2) <= tol;
}

/// Axial extent of the surface along the cylinder axis.
static void cyl_span(
    const NurbsSurface& srf,
    const std::array<double, 3>& apt,
    const std::array<double, 3>& adir,
    double& smin,
    double& smax
) {

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
static bool lines_closest_point(
    const std::array<double, 3>& p1,
    const std::array<double, 3>& d1,
    const std::array<double, 3>& p2,
    const std::array<double, 3>& d2,
    double tol,
    std::array<double, 3>& pout
) {

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

    pout = std::array<double, 3>{0.5 * (q1[0] + q2[0]), 0.5 * (q1[1] + q2[1]), 0.5 * (q1[2] + q2[2])};

    return true;
}

/// Circles of radius rad around the unit axis w through center at axial offsets zs.
static void axis_circles(
    const std::array<double, 3>& center,
    const std::array<double, 3>& w,
    const std::vector<double>& zs,
    double rad,
    std::vector<NurbsCurve>& out
) {

    std::array<double, 3> xa;
    std::array<double, 3> ya;
    std::tie(xa, ya) = ortho_basis(w);

    for (double z : zs) {
        std::array<double, 3> cc{center[0] + z * w[0], center[1] + z * w[1], center[2] + z * w[2]};
        out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rad));
    }
}

/// Coaxial cylinder-sphere section: circles.
static bool ssi_cylinder_sphere(const RecogSurface& cyl, const RecogSurface& sph, std::vector<NurbsCurve>& out) {

    const double ktol = 1e-6;
    std::array<double, 3> p = cyl.p1;
    std::array<double, 3> w = ssi_unit(cyl.p2);
    double rc = cyl.r;
    std::array<double, 3> center = sph.p1;
    double rsph = sph.r;

    if (point_axis_dist(p, w, center) > ktol)
        return false;

    if (rsph < rc - ktol)
        return true;

    double dist = std::sqrt(std::max(0.0, rsph * rsph - rc * rc));

    if (dist <= ktol) {
        std::array<double, 3> xa;
        std::array<double, 3> ya;
        std::tie(xa, ya) = ortho_basis(w);
        out.push_back(exact_circle(center[0], center[1], center[2], xa, ya, rc));

        return true;
    }

    axis_circles(center, w, {dist, -dist}, rc, out);

    return true;
}

/// Coaxial cylinder-cone section: circles.
static bool ssi_cylinder_cone(const RecogSurface& cyl, const RecogSurface& cone, std::vector<NurbsCurve>& out) {

    const double ktol = 1e-6;
    std::array<double, 3> pc = cyl.p1;
    std::array<double, 3> w = ssi_unit(cyl.p2);
    double rc = cyl.r;
    std::array<double, 3> apex = cone.p1;
    std::array<double, 3> a = ssi_unit(cone.p2);
    double alpha = cone.r;

    if (!axes_coaxial(pc, w, apex, a, ktol))
        return false;

    double ta = std::tan(alpha);

    if (ta < 1e-9)
        return false;

    double s = rc / ta;

    if (s < ktol)
        return true;

    axis_circles(apex, a, {s}, rc, out);

    return true;
}

/// Coaxial cone-sphere section: circles.
static bool ssi_cone_sphere(const RecogSurface& cone, const RecogSurface& sph, std::vector<NurbsCurve>& out) {

    const double ktol = 1e-6;
    std::array<double, 3> apex = cone.p1;
    std::array<double, 3> a = ssi_unit(cone.p2);
    double alpha = cone.r;
    std::array<double, 3> center = sph.p1;
    double rsph = sph.r;

    if (point_axis_dist(apex, a, center) > ktol)
        return false;

    double dsign = axial_coord(apex, a, center);
    double d = std::abs(dsign);
    std::array<double, 3> dir = (d > ktol && dsign < 0.0) ? std::array<double, 3>{-a[0], -a[1], -a[2]} : a;
    double t = std::tan(alpha);
    double t2 = t * t;
    double qa = 1.0 + t2;
    double qb = 2.0 * t2 * d;
    double qc = t2 * d * d - rsph * rsph;
    double disc = qb * qb - 4.0 * qa * qc;

    if (disc < -ktol)
        return true;

    double sq = std::sqrt(std::max(0.0, disc));
    std::vector<double> xs;

    if (sq <= ktol)
        xs = {-qb / (2.0 * qa)};
    else
        xs = {(-qb - sq) / (2.0 * qa), (-qb + sq) / (2.0 * qa)};

    std::array<double, 3> xa;
    std::array<double, 3> ya;
    std::tie(xa, ya) = ortho_basis(a);

    for (double x : xs) {
        double sAx = d + x;

        if (sAx < ktol)
            continue;

        double rr = t * sAx;

        if (rr < ktol)
            continue;

        std::array<double, 3> cc{apex[0] + sAx * dir[0], apex[1] + sAx * dir[1], apex[2] + sAx * dir[2]};
        out.push_back(exact_circle(cc[0], cc[1], cc[2], xa, ya, rr));
    }

    return true;
}

/// Parallel cylinders: shared ruling lines, false when coaxial with equal radii.
static bool ssi_parallel_cylinders(
    const NurbsSurface& sa,
    const RecogSurface& ra,
    const NurbsSurface& sb,
    const RecogSurface& rb,
    std::vector<NurbsCurve>& out
) {

    const double ktol = 1e-6;
    std::array<double, 3> p1 = ra.p1;
    std::array<double, 3> w1 = ssi_unit(ra.p2);
    double r1 = ra.r;
    std::array<double, 3> p2 = rb.p1;
    double r2 = rb.r;
    double d = point_axis_dist(p1, w1, p2);

    if (d <= ktol) {
        if (std::abs(r1 - r2) <= ktol)
            return false;

        return true;
    }

    if (d > r1 + r2 + ktol || d < std::abs(r1 - r2) - ktol)
        return true;

    double off = ssi_dot(std::array<double, 3>{p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]}, w1);
    std::array<double, 3> p2p{p2[0] - off * w1[0], p2[1] - off * w1[1], p2[2] - off * w1[2]};
    std::array<double, 3> xdir = ssi_unit(std::array<double, 3>{p2p[0] - p1[0], p2p[1] - p1[1], p2p[2] - p1[2]});
    std::array<double, 3> ydir = ssi_unit(ssi_cross(w1, xdir));
    double aa = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
    double h = std::sqrt(std::max(0.0, r1 * r1 - aa * aa));
    std::array<double, 3> foot{p1[0] + aa * xdir[0], p1[1] + aa * xdir[1], p1[2] + aa * xdir[2]};
    double s0a;
    double s1a;
    double s0b;
    double s1b;
    cyl_span(sa, p1, w1, s0a, s1a);
    cyl_span(sb, p1, w1, s0b, s1b);
    double slo = std::max(s0a, s0b);
    double shi = std::min(s1a, s1b);

    if (shi - slo <= ktol)
        return true;

    std::vector<std::array<double, 3>> feet;

    if (h <= ktol) {
        feet.push_back(foot);
    } else {
        feet.push_back(std::array<double, 3>{foot[0] + h * ydir[0], foot[1] + h * ydir[1], foot[2] + h * ydir[2]});
        feet.push_back(std::array<double, 3>{foot[0] - h * ydir[0], foot[1] - h * ydir[1], foot[2] - h * ydir[2]});
    }

    for (const std::array<double, 3>& bp : feet) {
        NurbsCurve line = axis_segment(bp, w1, slo, shi);
        line.set_domain(0.0, 1.0);
        out.push_back(line);
    }

    return true;
}

/// Cylinder-cylinder section: lines when parallel, Steinmetz ellipses when equal axes meet.
static bool ssi_cylinder_cylinder(
    const NurbsSurface& sa,
    const RecogSurface& ra,
    const NurbsSurface& sb,
    const RecogSurface& rb,
    std::vector<NurbsCurve>& out
) {

    const double ktol = 1e-6;
    std::array<double, 3> p1 = ra.p1;
    std::array<double, 3> w1 = ssi_unit(ra.p2);
    double r1 = ra.r;
    std::array<double, 3> p2 = rb.p1;
    std::array<double, 3> w2 = ssi_unit(rb.p2);
    double r2 = rb.r;
    std::array<double, 3> cx = ssi_cross(w1, w2);

    if (std::sqrt(ssi_dot(cx, cx)) <= ktol)
        return ssi_parallel_cylinders(sa, ra, sb, rb, out);

    double rmax = std::max(r1, r2);

    if (rmax < 1e-12 || std::abs(r1 - r2) / rmax > 1e-6)
        return false;

    std::array<double, 3> pint;

    if (!lines_closest_point(p1, w1, p2, w2, ktol, pint))
        return false;

    double r = 0.5 * (r1 + r2);
    double ang = std::acos(std::max(-1.0, std::min(1.0, ssi_dot(w1, w2))));
    double sh = std::sin(0.5 * ang);
    double ch = std::cos(0.5 * ang);

    if (sh < 1e-9 || ch < 1e-9)
        return false;

    std::array<double, 3> minor = ssi_unit(cx);
    std::array<double, 3> maj1 = ssi_unit(std::array<double, 3>{w1[0] + w2[0], w1[1] + w2[1], w1[2] + w2[2]});
    std::array<double, 3> maj2 = ssi_unit(std::array<double, 3>{w1[0] - w2[0], w1[1] - w2[1], w1[2] - w2[2]});
    out.push_back(exact_ellipse(pint[0], pint[1], pint[2], maj1, minor, r / sh, r));
    out.push_back(exact_ellipse(pint[0], pint[1], pint[2], maj2, minor, r / ch, r));

    return true;
}

/// Exact circles of a coaxial cylinder-torus pair.
static bool ssi_cylinder_torus(const RecogSurface& cyl, const RecogSurface& tor, std::vector<NurbsCurve>& out) {

    const double ktol = 1e-6;
    std::array<double, 3> p = cyl.p1;
    std::array<double, 3> wc = ssi_unit(cyl.p2);
    double rc = cyl.r;
    std::array<double, 3> center = tor.p1;
    std::array<double, 3> w = ssi_unit(tor.p2);
    double rmaj = tor.r;
    double r = tor.r2;

    if (r >= rmaj - ktol)
        return false;

    if (!axes_coaxial(p, wc, center, w, ktol))
        return false;

    double dr = rc - rmaj;
    double h2 = r * r - dr * dr;

    if (h2 < -ktol)
        return true;

    double h = std::sqrt(std::max(0.0, h2));

    if (h <= ktol)
        axis_circles(center, w, {0.0}, rc, out);
    else
        axis_circles(center, w, {h, -h}, rc, out);

    return true;
}

/// Circles of a coaxial cone and one side (rsign = +rmaj or -rmaj) of the torus tube.
static void cone_torus_circles(
    const std::array<double, 3>& center,
    const std::array<double, 3>& w,
    double t,
    double za,
    double r,
    double rsign,
    std::vector<NurbsCurve>& out
) {

    const double ktol = 1e-6;
    double qa = t * t + 1.0;
    double qb = -2.0 * t * (t * za + rsign);
    double qc = (t * za + rsign) * (t * za + rsign) - r * r;
    double disc = qb * qb - 4.0 * qa * qc;

    if (disc < -ktol)
        return;

    double sq = std::sqrt(std::max(0.0, disc));
    std::vector<double> zs;

    if (sq <= ktol)
        zs = {-qb / (2.0 * qa)};
    else
        zs = {(-qb - sq) / (2.0 * qa), (-qb + sq) / (2.0 * qa)};

    for (double z : zs) {
        double rad = t * std::abs(z - za);

        if (rad < ktol)
            continue;

        axis_circles(center, w, {z}, rad, out);
    }
}

/// Coaxial cone-torus section: circles.
static bool ssi_cone_torus(const RecogSurface& cone, const RecogSurface& tor, std::vector<NurbsCurve>& out) {

    const double ktol = 1e-6;
    std::array<double, 3> apex = cone.p1;
    std::array<double, 3> a = ssi_unit(cone.p2);
    double alpha = cone.r;
    std::array<double, 3> center = tor.p1;
    std::array<double, 3> w = ssi_unit(tor.p2);
    double rmaj = tor.r;
    double r = tor.r2;

    if (r >= rmaj - ktol)
        return false;

    if (!axes_coaxial(apex, a, center, w, ktol))
        return false;

    double t = std::tan(alpha);

    if (t < 1e-9)
        return false;

    double za = axial_coord(center, w, apex);
    cone_torus_circles(center, w, t, za, r, +rmaj, out);
    cone_torus_circles(center, w, t, za, r, -rmaj, out);

    return true;
}

/// Circles where the tube circle (rmaj, 0) of radius r meets the circle of radius r2 at offset (dx, dz) from it.
static void meridian_circles(
    const std::array<double, 3>& center,
    const std::array<double, 3>& w,
    double rmaj,
    double r,
    double dx,
    double dz,
    double r2,
    std::vector<NurbsCurve>& out
) {

    const double ktol = 1e-6;
    double d = std::sqrt(dx * dx + dz * dz);
    double aa = 0.5 * (r * r - r2 * r2 + d * d) / d;
    double h = std::sqrt(std::max(0.0, r * r - aa * aa));
    double dirx = dx / d;
    double dirz = dz / d;
    double phx = rmaj + aa * dirx;
    double phz = aa * dirz;
    double perpx = -dirz;
    double perpz = dirx;
    std::vector<int> signs = (h <= ktol) ? std::vector<int>{0} : std::vector<int>{+1, -1};

    for (int s : signs) {
        double xi = phx + s * h * perpx;
        double z = phz + s * h * perpz;
        double rad = std::abs(xi);

        if (rad < ktol)
            continue;

        axis_circles(center, w, {z}, rad, out);
    }
}

/// Coaxial sphere-torus section: circles.
static bool ssi_sphere_torus(const RecogSurface& sph, const RecogSurface& tor, std::vector<NurbsCurve>& out) {

    const double ktol = 1e-6;
    std::array<double, 3> sc = sph.p1;
    double rsph = sph.r;
    std::array<double, 3> center = tor.p1;
    std::array<double, 3> w = ssi_unit(tor.p2);
    double rmaj = tor.r;
    double r = tor.r2;

    if (r >= rmaj - ktol)
        return false;

    if (point_axis_dist(center, w, sc) > ktol)
        return false;

    double zs = axial_coord(center, w, sc);
    double d = std::sqrt(rmaj * rmaj + zs * zs);

    if (d < ktol)
        return true;

    if (d - ktol > r + rsph || d + ktol < std::abs(r - rsph))
        return true;

    meridian_circles(center, w, rmaj, r, 0.0 - rmaj, zs - 0.0, rsph, out);

    return true;
}

/// Spiric loop frame of two equal parallel-axis tori.
struct SpiricFrame {
    std::array<double, 3> c1; // First torus center.
    std::array<double, 3> ex; // Unit direction between the centers.
    std::array<double, 3> ey; // Unit axis cross ex.
    std::array<double, 3> w; // Unit common axis.
    double rmaj; // Major radius.
    double r; // Minor radius.
    double c; // Half the center distance.
    double be; // Semi-axis of the inner loops.
};

/// In-plane point (x, y) of a spiric loop at tube offset t; false when the loop does not reach t.
static bool spiric_xy(const SpiricFrame& f, bool inner, double t, double& x, double& y) {

    if (inner) {
        double g = t / f.c;

        if (std::abs(g) >= 1.0)
            return false;

        x = f.c + f.rmaj * g;
        y = f.be * std::sqrt(1.0 - g * g);

        return true;
    }

    double rho = f.rmaj + t;
    double y2 = rho * rho - f.c * f.c;

    if (y2 <= 0.0)
        return false;

    x = f.c;
    y = std::sqrt(y2);

    return true;
}

/// Both mirrored spiric loops as periodic interpolants, none when either misses a sample.
static void emit_spiric_loops(const SpiricFrame& f, bool inner, std::vector<NurbsCurve>& out) {

    const int n = 512;

    for (int sgn : {+1, -1}) {
        std::vector<Point> pts;
        pts.reserve(n);

        for (int k = 0; k < n; ++k) {
            double phi = Tolerance::TWO_PI * k / n;
            double t = f.r * std::cos(phi);
            double z = f.r * std::sin(phi);
            double x;
            double y;

            if (!spiric_xy(f, inner, t, x, y))
                return;

            double yy = sgn * y;

            pts.push_back(Point(
                f.c1[0] + x * f.ex[0] + yy * f.ey[0] + z * f.w[0],
                f.c1[1] + x * f.ex[1] + yy * f.ey[1] + z * f.w[1],
                f.c1[2] + x * f.ex[2] + yy * f.ey[2] + z * f.w[2]
            ));
        }

        NurbsCurve loop = NurbsCurve::create_interpolated(pts, CurveNurbsKnotStyle::ChordPeriodic);

        if (loop.is_valid()) {
            loop.set_domain(0.0, 1.0);
            out.push_back(loop);
        }
    }
}

/// Exact spiric loops of two equal parallel-axis tori.
static bool ssi_torus_torus_spiric(const RecogSurface& ta, const RecogSurface& tb, std::vector<NurbsCurve>& out) {

    const double ktol = 1e-6;
    std::array<double, 3> c1 = ta.p1;
    std::array<double, 3> w = ssi_unit(ta.p2);
    std::array<double, 3> c2 = tb.p1;
    std::array<double, 3> cxw = ssi_cross(w, ssi_unit(tb.p2));

    if (std::sqrt(ssi_dot(cxw, cxw)) > ktol || std::abs(ta.r2 - tb.r2) > ktol || std::abs(ta.r - tb.r) > ktol)
        return false;

    if (std::abs(axial_coord(c1, w, c2)) > ktol)
        return false;

    std::array<double, 3> dp{c2[0] - c1[0], c2[1] - c1[1], c2[2] - c1[2]};
    double hax = ssi_dot(dp, w);
    std::array<double, 3> ex{dp[0] - hax * w[0], dp[1] - hax * w[1], dp[2] - hax * w[2]};
    double d = std::sqrt(ssi_dot(ex, ex));

    if (d <= ktol)
        return false;

    ex = std::array<double, 3>{ex[0] / d, ex[1] / d, ex[2] / d};
    SpiricFrame f{c1, ex, ssi_cross(w, ex), w, 0.5 * (ta.r + tb.r), 0.5 * (ta.r2 + tb.r2), 0.5 * d, 0.0};

    if (std::abs(f.rmaj - f.c) <= ktol)
        return false;

    double lo2 = (f.rmaj - f.r) * (f.rmaj - f.r) - f.c * f.c;
    double hi2 = (f.rmaj + f.r) * (f.rmaj + f.r) - f.c * f.c;

    if (hi2 > ktol && lo2 <= ktol)
        return false;

    if (f.rmaj > f.c && f.r >= f.c - ktol)
        return false;

    if (lo2 > ktol)
        emit_spiric_loops(f, false, out);

    if (f.rmaj > f.c + ktol) {
        f.be = std::sqrt(f.rmaj * f.rmaj - f.c * f.c);
        emit_spiric_loops(f, true, out);
    }

    return true;
}

/// Coaxial torus-torus section: circles.
static bool ssi_torus_torus(const RecogSurface& ta, const RecogSurface& tb, std::vector<NurbsCurve>& out) {

    const double ktol = 1e-6;
    std::array<double, 3> c1 = ta.p1;
    std::array<double, 3> w = ssi_unit(ta.p2);
    double rmaj1 = ta.r;
    double r1 = ta.r2;
    std::array<double, 3> c2 = tb.p1;
    std::array<double, 3> w2 = ssi_unit(tb.p2);
    double rmaj2 = tb.r;
    double r2 = tb.r2;

    if (r1 >= rmaj1 - ktol || r2 >= rmaj2 - ktol)
        return false;

    if (!axes_coaxial(c1, w, c2, w2, ktol))
        return ssi_torus_torus_spiric(ta, tb, out);

    double z2 = axial_coord(c1, w, c2);
    double dxR = rmaj2 - rmaj1;
    double d = std::sqrt(dxR * dxR + z2 * z2);

    if (d < ktol)
        return false;

    if (d - ktol > r1 + r2 || d + ktol < std::abs(r1 - r2))
        return true;

    meridian_circles(c1, w, rmaj1, r1, dxR, z2, r2, out);

    return true;
}

/// Exact sphere-sphere circle.
static void ssi_sphere_sphere(const RecogSurface& ra, const RecogSurface& rb, std::vector<NurbsCurve>& out) {

    std::array<double, 3> c1 = ra.p1;
    double r1 = ra.r;
    std::array<double, 3> c2 = rb.p1;
    double r2 = rb.r;
    std::array<double, 3> dv{c2[0] - c1[0], c2[1] - c1[1], c2[2] - c1[2]};
    double dist = std::sqrt(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]);
    double tan_tol = (r1 + r2) * 1e-9;

    if (dist <= 1e-12 || dist >= r1 + r2 - tan_tol || dist <= std::abs(r1 - r2) + tan_tol)
        return;

    std::array<double, 3> nu{dv[0] / dist, dv[1] / dist, dv[2] / dist};
    double aa = (dist * dist + r1 * r1 - r2 * r2) / (2.0 * dist);
    double rr2 = r1 * r1 - aa * aa;

    if (rr2 > 0.0)
        axis_circles(c1, nu, {aa}, std::sqrt(rr2), out);
}

/// Exact sections of a plane with a recognized surface; false when the case is not analytic.
static bool plane_section_curves(
    const RecogSurface& plane,
    const NurbsSurface& srf,
    const RecogSurface& rs,
    std::vector<NurbsCurve>& out
) {

    NurbsCurve c3;

    if (rs.kind == RecogSurface::SPHERE) {
        if (ssi_plane_sphere(plane, rs, c3))
            out.push_back(c3);

        return true;
    }

    if (rs.kind == RecogSurface::CYLINDER) {
        if (!ssi_plane_cylinder_lines(plane, rs, srf, out) && ssi_plane_cylinder(plane, rs, c3))
            out.push_back(c3);

        return true;
    }

    if (rs.kind == RecogSurface::CONE)
        return ssi_plane_cone(plane, rs, srf, out);

    return ssi_plane_torus(plane, rs, out);
}

/// Exact sections of two recognized curved surfaces; false when the case is not analytic.
static bool quadric_section_curves(
    const NurbsSurface& a,
    const RecogSurface& ra,
    const NurbsSurface& b,
    const RecogSurface& rb,
    std::vector<NurbsCurve>& out
) {

    RecogSurface::Kind ka = ra.kind;
    RecogSurface::Kind kb = rb.kind;

    if (ka == RecogSurface::SPHERE && kb == RecogSurface::SPHERE) {
        ssi_sphere_sphere(ra, rb, out);

        return true;
    }

    if (ka == RecogSurface::CYLINDER && kb == RecogSurface::SPHERE)
        return ssi_cylinder_sphere(ra, rb, out);

    if (ka == RecogSurface::SPHERE && kb == RecogSurface::CYLINDER)
        return ssi_cylinder_sphere(rb, ra, out);

    if (ka == RecogSurface::CYLINDER && kb == RecogSurface::CONE)
        return ssi_cylinder_cone(ra, rb, out);

    if (ka == RecogSurface::CONE && kb == RecogSurface::CYLINDER)
        return ssi_cylinder_cone(rb, ra, out);

    if (ka == RecogSurface::CONE && kb == RecogSurface::SPHERE)
        return ssi_cone_sphere(ra, rb, out);

    if (ka == RecogSurface::SPHERE && kb == RecogSurface::CONE)
        return ssi_cone_sphere(rb, ra, out);

    if (ka == RecogSurface::CYLINDER && kb == RecogSurface::CYLINDER)
        return ssi_cylinder_cylinder(a, ra, b, rb, out);

    if (ka == RecogSurface::CYLINDER && kb == RecogSurface::TORUS)
        return ssi_cylinder_torus(ra, rb, out);

    if (ka == RecogSurface::TORUS && kb == RecogSurface::CYLINDER)
        return ssi_cylinder_torus(rb, ra, out);

    if (ka == RecogSurface::CONE && kb == RecogSurface::TORUS)
        return ssi_cone_torus(ra, rb, out);

    if (ka == RecogSurface::TORUS && kb == RecogSurface::CONE)
        return ssi_cone_torus(rb, ra, out);

    if (ka == RecogSurface::SPHERE && kb == RecogSurface::TORUS)
        return ssi_sphere_torus(ra, rb, out);

    if (ka == RecogSurface::TORUS && kb == RecogSurface::SPHERE)
        return ssi_sphere_torus(rb, ra, out);

    if (ka == RecogSurface::TORUS && kb == RecogSurface::TORUS)
        return ssi_torus_torus(ra, rb, out);

    return false;
}

/// Exact 3D sections of two recognized surfaces; false when the pair is not analytic.
static bool analytic_curves(
    const NurbsSurface& a,
    const RecogSurface& ra,
    const NurbsSurface& b,
    const RecogSurface& rb,
    std::vector<NurbsCurve>& out
) {

    if (ra.kind == RecogSurface::PLANE && rb.kind == RecogSurface::PLANE) {
        NurbsCurve c3;
        bool empty = false;

        if (ssi_plane_plane(a, ra, b, rb, c3, empty)) {
            out.push_back(c3);

            return true;
        }

        return empty;
    }

    if (ra.kind == RecogSurface::PLANE)
        return plane_section_curves(ra, b, rb, out);

    if (rb.kind == RecogSurface::PLANE)
        return plane_section_curves(rb, a, ra, out);

    return quadric_section_curves(a, ra, b, rb, out);
}

/// Pcurve of an exact section on one recognized surface: analytic, pulled back, then projected.
static NurbsCurve analytic_side_pcurve(const NurbsSurface& srf, const RecogSurface& recog, const NurbsCurve& c3) {

    NurbsCurve pc = analytic_pcurve(srf, recog, c3);

    if (!pc.is_valid()) {
        std::vector<NurbsCurve> v = analytic_pullback(srf, recog, c3);

        if (!v.empty())
            pc = v[0];
    }

    if (!pc.is_valid()) {
        std::vector<NurbsCurve> v = Closest::surface_curve(srf, c3);

        if (!v.empty())
            pc = v[0];
    }

    return pc;
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

    if (!analytic_curves(a, ra, b, rb, c3_list))
        return res;

    for (const NurbsCurve& cc3 : c3_list) {
        NurbsCurve pa = analytic_side_pcurve(a, ra, cc3);
        NurbsCurve pb = analytic_side_pcurve(b, rb, cc3);

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

    const SurfacePlaneField field(surface, plane, tolerance);
    const SurfacePlaneTraceResult traced = surface_plane_traces(surface, plane, tolerance);
    const double dup_tol = traced.step * traced.uv_to_3d * 3.0;

    std::vector<std::pair<NurbsCurve, NurbsCurve>> result;
    std::vector<std::vector<Point>> kept_pts3;

    for (const SurfacePlaneTrace& trace : traced.traces) {
        std::vector<Point> trace_pts3(trace.uv_trace.size());

        for (size_t i = 0; i < trace.uv_trace.size(); i++)
            trace_pts3[i] = field.point(trace.uv_trace[i]);

        if (is_duplicate_trace(trace_pts3, kept_pts3, dup_tol))
            continue;

        kept_pts3.push_back(trace_pts3);

        for (SurfacePlanePiece& piece : trace_pieces(field, trace)) {
            if (piece.uv.size() < 2)
                continue;

            NurbsCurve crv3;
            NurbsCurve pcurve;

            if (piece_curves(field, plane, piece, crv3, pcurve))
                result.push_back({std::move(crv3), std::move(pcurve)});
        }
    }

    return result;
}

/// Drop near-zero-length section curves.
static void drop_point_sections(std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>>& trs, double tolerance) {

    const double min_len = std::max(tolerance * 10.0, 1e-9);
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> kept;

    for (std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& t : trs)
        if (std::get<0>(t).length() >= min_len)
            kept.push_back(std::move(t));

    trs = std::move(kept);
}

namespace {

/// Plane through the middle of the surface domain.
Plane surface_mid_plane(const NurbsSurface& srf) {

    Point po;
    Vector nn;
    surface_mid_frame(srf, po, nn);

    return Plane::from_point_normal(po, Vector(nn[0], nn[1], nn[2]));
}

/// Plane-surface section triples, the plane's pcurve found by projection.
std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> planar_section_triples(
    const NurbsSurface& planar,
    const NurbsSurface& other,
    bool planar_first,
    double tolerance
) {

    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> result;

    for (const std::pair<NurbsCurve, NurbsCurve>& section : Intersection::surface_plane_uv(other, surface_mid_plane(planar), tolerance)) {
        const NurbsCurve& c3 = section.first;
        std::vector<NurbsCurve> pps = Closest::surface_curve(planar, c3);

        if (pps.size() != 1)
            continue;

        if (planar_first)
            result.push_back({c3, pps[0], section.second});
        else
            result.push_back({c3, section.second, pps[0]});
    }

    drop_point_sections(result, tolerance);

    return result;
}

/// Bounding box of one grid cell from its 3 x 3 samples, inflated by twice its sag, and the cell center.
std::array<double, 8> cell_box(
    const std::vector<std::vector<Point>>& samples,
    int ci,
    int cj,
    double c0u,
    double dcu,
    double c0v,
    double dcv,
    double tolerance
) {

    double minx = std::numeric_limits<double>::infinity();
    double miny = minx;
    double minz = minx;
    double maxx = -minx;
    double maxy = -minx;
    double maxz = -minx;

    for (int i = 2 * ci; i < 2 * ci + 3; i++) {
        for (int j = 2 * cj; j < 2 * cj + 3; j++) {
            const Point& p = samples[i][j];
            minx = std::min(minx, p[0]);
            maxx = std::max(maxx, p[0]);
            miny = std::min(miny, p[1]);
            maxy = std::max(maxy, p[1]);
            minz = std::min(minz, p[2]);
            maxz = std::max(maxz, p[2]);
        }
    }

    const Point& ctr = samples[2 * ci + 1][2 * cj + 1];
    const Point& p00 = samples[2 * ci][2 * cj];
    const Point& p10 = samples[2 * ci + 2][2 * cj];
    const Point& p01 = samples[2 * ci][2 * cj + 2];
    const Point& p11 = samples[2 * ci + 2][2 * cj + 2];
    double cx = (p00[0] + p10[0] + p01[0] + p11[0]) * 0.25;
    double cy = (p00[1] + p10[1] + p01[1] + p11[1]) * 0.25;
    double cz = (p00[2] + p10[2] + p01[2] + p11[2]) * 0.25;
    double sag = std::sqrt((ctr[0] - cx) * (ctr[0] - cx) + (ctr[1] - cy) * (ctr[1] - cy) + (ctr[2] - cz) * (ctr[2] - cz));
    double inf = 2.0 * sag + tolerance;

    return {minx - inf, miny - inf, minz - inf, maxx + inf, maxy + inf, maxz + inf, c0u + dcu * (ci + 0.5), c0v + dcv * (cj + 0.5)};
}

/// Inflated bounding boxes and centers of an ncu x ncv grid of surface cells.
std::vector<std::array<double, 8>> surface_cell_boxes(
    const NurbsSurface& srf,
    double c0u,
    double dcu,
    int ncu,
    double c0v,
    double dcv,
    int ncv,
    double tolerance
) {

    std::vector<std::vector<Point>> samples;

    for (int i = 0; i < 2 * ncu + 1; i++) {
        std::vector<Point> row;

        for (int j = 0; j < 2 * ncv + 1; j++)
            row.push_back(srf.point_at(c0u + dcu * 0.5 * i, c0v + dcv * 0.5 * j));

        samples.push_back(row);
    }

    std::vector<std::array<double, 8>> boxes;

    for (int ci = 0; ci < ncu; ci++)
        for (int cj = 0; cj < ncv; cj++)
            boxes.push_back(cell_box(samples, ci, cj, c0u, dcu, c0v, dcv, tolerance));

    return boxes;
}

/// Smallest non-degenerate diagonal among the first 64 boxes, 1 when none.
double cell_diagonal(const std::vector<std::array<double, 8>>& boxes) {

    double best = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < boxes.size() && i < 64; i++) {
        const std::array<double, 8>& bx = boxes[i];
        double d = std::sqrt((bx[3] - bx[0]) * (bx[3] - bx[0]) + (bx[4] - bx[1]) * (bx[4] - bx[1]) + (bx[5] - bx[2]) * (bx[5] - bx[2]));

        if (1e-12 < d && d < best)
            best = d;
    }

    return best < std::numeric_limits<double>::infinity() ? best : 1.0;
}

/// Grid seed of a surface-surface trace in joint parameters.
struct SurfaceSurfaceSeed {
    double u; // Seed u on a.
    double v; // Seed v on a.
    double s; // Seed u on b.
    double t; // Seed v on b.
    bool used; // Whether a trace already passed the seed.
};

/// Joint parameter space (au, av, bu, bv) of two surfaces with the marching scales.
class SurfaceSurfaceField {
public:
    const NurbsSurface& a; // First surface.
    const NurbsSurface& b; // Second surface.
    double tolerance; // Section tolerance.
    std::array<double, 4> lo; // Domain starts.
    std::array<double, 4> hi; // Domain ends.
    std::array<double, 4> range; // Domain lengths.
    std::array<bool, 4> closed; // Whether each parameter wraps around a seam.
    std::array<double, 4> step; // Grid cell size per parameter.
    std::vector<std::array<double, 8>> boxes_a; // Cell boxes of a.
    std::vector<std::array<double, 8>> boxes_b; // Cell boxes of b.
    double h_init; // Initial 3D marching step.
    double conv_tol; // Corrector convergence tolerance.
    double seed_tol; // 3D distance that merges two seeds.
    int max_steps; // Marching step cap per direction.
    double close_tol; // 3D distance that closes a loop.
    double consume_tol; // 3D distance that consumes a seed.

    /// Sample both domains into cell boxes and derive the marching scales.
    SurfaceSurfaceField(const NurbsSurface& a_, const NurbsSurface& b_, double tolerance_);

    /// Wrap parameter k across a closed seam or clamp it to the domain.
    double wrap(int k, double t) const;

    /// Point and first derivatives of a at (u, v).
    void eval_a(double u, double v, Vector& s, Vector& su, Vector& sv) const;

    /// Point and first derivatives of b at (u, v).
    void eval_b(double u, double v, Vector& s, Vector& su, Vector& sv) const;

    /// Point of a at the joint parameters q.
    std::array<double, 3> point(const std::array<double, 4>& q) const;

    /// Clamp the open parameters of x to their domains.
    void clamp_open(std::array<double, 4>& x) const;

    /// Newton-project x onto the section, optionally pinned to the plane through pp normal to pd.
    bool correct(
        std::array<double, 4>& x,
        bool has_pin,
        const std::array<double, 3>& pd,
        const std::array<double, 3>& pp
    ) const;

    /// Unit 3D section tangent at x in direction dir_sign, with both surfaces' derivatives.
    bool tangent(
        const std::array<double, 4>& x,
        double dir_sign,
        std::array<double, 3>& dir,
        Vector& sa,
        Vector& sau,
        Vector& sav,
        Vector& sbu,
        Vector& sbv
    ) const;
};

SurfaceSurfaceField::SurfaceSurfaceField(const NurbsSurface& a_, const NurbsSurface& b_, double tolerance_)
    : a(a_), b(b_), tolerance(tolerance_) {

    const NurbsSurface* srfs[2] = {&a, &b};
    std::array<int, 4> cells;

    for (int k = 0; k < 4; k++) {
        const NurbsSurface& srf = *srfs[k / 2];
        const std::pair<double, double> domain = srf.domain(k % 2);
        lo[k] = domain.first;
        hi[k] = domain.second;
        range[k] = hi[k] - lo[k];
        closed[k] = srf.is_closed(k % 2);
        cells[k] = std::max((int)srf.get_span_vector(k % 2).size() - 1, 1) * 4;
        step[k] = range[k] / cells[k];
    }

    boxes_a = surface_cell_boxes(a, lo[0], step[0], cells[0], lo[1], step[1], cells[1], tolerance);
    boxes_b = surface_cell_boxes(b, lo[2], step[2], cells[2], lo[3], step[3], cells[3], tolerance);
    h_init = std::min(cell_diagonal(boxes_a), cell_diagonal(boxes_b)) * 0.25;
    conv_tol = std::max(tolerance, h_init * 1e-7);
    seed_tol = std::max(cell_diagonal(boxes_a), cell_diagonal(boxes_b));
    max_steps = (cells[0] * cells[1] + cells[2] * cells[3]) * 32;
    close_tol = h_init * 3.0;
    consume_tol = h_init * 2.0;
}

double SurfaceSurfaceField::wrap(int k, double t) const {

    if (closed[k]) {
        double f = std::fmod(t - lo[k], range[k]);

        if (f < 0)
            f += range[k];

        return lo[k] + f;
    }

    return std::max(lo[k], std::min(t, hi[k]));
}

void SurfaceSurfaceField::eval_a(double u, double v, Vector& s, Vector& su, Vector& sv) const {

    const std::vector<Vector> d = a.evaluate(wrap(0, u), wrap(1, v), 1);
    s = d[0];
    su = d[2];
    sv = d[1];
}

void SurfaceSurfaceField::eval_b(double u, double v, Vector& s, Vector& su, Vector& sv) const {

    const std::vector<Vector> d = b.evaluate(wrap(2, u), wrap(3, v), 1);
    s = d[0];
    su = d[2];
    sv = d[1];
}

std::array<double, 3> SurfaceSurfaceField::point(const std::array<double, 4>& q) const {

    Vector sa;
    Vector sau;
    Vector sav;
    eval_a(q[0], q[1], sa, sau, sav);

    return {sa[0], sa[1], sa[2]};
}

void SurfaceSurfaceField::clamp_open(std::array<double, 4>& x) const {

    for (int k = 0; k < 4; k++)
        if (!closed[k])
            x[k] = std::max(lo[k], std::min(x[k], hi[k]));
}

/// Minimum-norm Newton step x -= jac^T (jac jac^T)^-1 res.
bool newton_step_free(const double jac[3][4], const double res[3], std::array<double, 4>& x) {

    std::vector<std::vector<double>> jjt(3, std::vector<double>(3));

    for (int r = 0; r < 3; r++) {
        for (int q = 0; q < 3; q++) {
            double s = 0.0;

            for (int c = 0; c < 4; c++)
                s += jac[r][c] * jac[q][c];

            jjt[r][q] = s;
        }
    }

    std::vector<double> y;

    if (!solve_gauss(jjt, {res[0], res[1], res[2]}, 3, y))
        return false;

    for (int c = 0; c < 4; c++) {
        double s = 0.0;

        for (int r = 0; r < 3; r++)
            s += jac[r][c] * y[r];

        x[c] -= s;
    }

    return true;
}

/// Newton step with a fourth row pinning a's point to the plane through pp normal to pd.
bool newton_step_pinned(
    const double jac[3][4],
    const double res[3],
    const Vector& sa,
    const Vector& sau,
    const Vector& sav,
    const std::array<double, 3>& pd,
    const std::array<double, 3>& pp,
    std::array<double, 4>& x
) {

    std::vector<std::vector<double>> m = {
        {jac[0][0], jac[0][1], jac[0][2], jac[0][3]},
        {jac[1][0], jac[1][1], jac[1][2], jac[1][3]},
        {jac[2][0], jac[2][1], jac[2][2], jac[2][3]},
        {pd[0] * sau[0] + pd[1] * sau[1] + pd[2] * sau[2], pd[0] * sav[0] + pd[1] * sav[1] + pd[2] * sav[2], 0.0, 0.0}
    };

    std::vector<double> rhs = {res[0], res[1], res[2], pd[0] * (sa[0] - pp[0]) + pd[1] * (sa[1] - pp[1]) + pd[2] * (sa[2] - pp[2])};
    std::vector<double> dx;

    if (!solve_gauss(m, rhs, 4, dx))
        return false;

    for (int c = 0; c < 4; c++)
        x[c] -= dx[c];

    return true;
}

bool SurfaceSurfaceField::correct(
    std::array<double, 4>& x,
    bool has_pin,
    const std::array<double, 3>& pd,
    const std::array<double, 3>& pp
) const {

    Vector sa;
    Vector sau;
    Vector sav;
    Vector sb;
    Vector sbu;
    Vector sbv;

    for (int it = 0; it < 8; it++) {
        eval_a(x[0], x[1], sa, sau, sav);
        eval_b(x[2], x[3], sb, sbu, sbv);
        double res[3] = {sa[0] - sb[0], sa[1] - sb[1], sa[2] - sb[2]};

        if (std::sqrt(res[0] * res[0] + res[1] * res[1] + res[2] * res[2]) < conv_tol)
            return true;

        double jac[3][4];

        for (int k = 0; k < 3; k++) {
            jac[k][0] = sau[k];
            jac[k][1] = sav[k];
            jac[k][2] = -sbu[k];
            jac[k][3] = -sbv[k];
        }

        bool ok = has_pin ? newton_step_pinned(jac, res, sa, sau, sav, pd, pp, x) : newton_step_free(jac, res, x);

        if (!ok)
            return false;

        clamp_open(x);
    }

    eval_a(x[0], x[1], sa, sau, sav);
    eval_b(x[2], x[3], sb, sbu, sbv);
    double g = std::sqrt((sa[0] - sb[0]) * (sa[0] - sb[0]) + (sa[1] - sb[1]) * (sa[1] - sb[1]) + (sa[2] - sb[2]) * (sa[2] - sb[2]));

    return g < conv_tol * 10.0;
}

bool SurfaceSurfaceField::tangent(
    const std::array<double, 4>& x,
    double dir_sign,
    std::array<double, 3>& dir,
    Vector& sa,
    Vector& sau,
    Vector& sav,
    Vector& sbu,
    Vector& sbv
) const {

    Vector sb;
    eval_a(x[0], x[1], sa, sau, sav);
    eval_b(x[2], x[3], sb, sbu, sbv);
    double na[3] = {sau[1] * sav[2] - sau[2] * sav[1], sau[2] * sav[0] - sau[0] * sav[2], sau[0] * sav[1] - sau[1] * sav[0]};
    double nb[3] = {sbu[1] * sbv[2] - sbu[2] * sbv[1], sbu[2] * sbv[0] - sbu[0] * sbv[2], sbu[0] * sbv[1] - sbu[1] * sbv[0]};
    double d[3] = {na[1] * nb[2] - na[2] * nb[1], na[2] * nb[0] - na[0] * nb[2], na[0] * nb[1] - na[1] * nb[0]};
    double dl = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    double nal = std::sqrt(na[0] * na[0] + na[1] * na[1] + na[2] * na[2]);
    double nbl = std::sqrt(nb[0] * nb[0] + nb[1] * nb[1] + nb[2] * nb[2]);

    if (dl < 1e-4 * nal * nbl || dl < 1e-30)
        return false;

    dir = {d[0] / dl * dir_sign, d[1] / dl * dir_sign, d[2] / dl * dir_sign};

    return true;
}

/// Distance between two 3D triples.
double triple_distance(const std::array<double, 3>& p, const std::array<double, 3>& q) {
    return std::sqrt((p[0] - q[0]) * (p[0] - q[0]) + (p[1] - q[1]) * (p[1] - q[1]) + (p[2] - q[2]) * (p[2] - q[2]));
}

/// Whether a's point at x lies within the seed tolerance of an existing seed.
bool seed_is_duplicate(const SurfaceSurfaceField& field, const std::array<double, 4>& x, const std::vector<SurfaceSurfaceSeed>& seeds) {

    std::array<double, 3> p = field.point(x);

    for (const SurfaceSurfaceSeed& sd : seeds)
        if (triple_distance(p, field.point({sd.u, sd.v, 0.0, 0.0})) < field.seed_tol)
            return true;

    return false;
}

/// Corrected centers of overlapping cell-box pairs, one per distinct 3D point, at most 20000 pairs.
std::vector<SurfaceSurfaceSeed> surface_surface_seeds(const SurfaceSurfaceField& field) {

    std::vector<SurfaceSurfaceSeed> seeds;
    int pair_budget = 20000;
    const std::array<double, 3> dummy3 = {0.0, 0.0, 0.0};

    for (const std::array<double, 8>& ba : field.boxes_a) {
        if (pair_budget < 0)
            break;

        for (const std::array<double, 8>& bb : field.boxes_b) {
            if (bb[0] > ba[3] || bb[3] < ba[0] || bb[1] > ba[4] || bb[4] < ba[1] || bb[2] > ba[5] || bb[5] < ba[2])
                continue;

            pair_budget -= 1;

            if (pair_budget < 0)
                break;

            std::array<double, 4> x = {ba[6], ba[7], bb[6], bb[7]};

            if (!field.correct(x, false, dummy3, dummy3) || seed_is_duplicate(field, x, seeds))
                continue;

            seeds.push_back({field.wrap(0, x[0]), field.wrap(1, x[1]), field.wrap(2, x[2]), field.wrap(3, x[3]), false});
        }
    }

    return seeds;
}

/// Marching state of one trace direction.
struct SurfaceSurfaceMarch {
    std::array<double, 4> x; // Current joint parameters.
    std::array<double, 3> d; // Current 3D direction.
    Vector sa; // Point of a at x.
    Vector sau; // u-derivative of a at x.
    Vector sav; // v-derivative of a at x.
    Vector sbu; // u-derivative of b at x.
    Vector sbv; // v-derivative of b at x.
    bool have_prev_d; // Whether a previous step exists.
    std::array<double, 3> prev_d; // Previous 3D direction.
    std::array<double, 3> p_prev; // Previous 3D point.
    double h; // Current 3D step.
    int smooth; // Accepted steps since the last change of h.
    int tang_reuse; // Steps that reused the previous direction.
    std::string_view why; // Reason the march stopped.
    std::array<double, 4> xn; // Accepted next parameters.
    std::array<double, 3> p_cur; // Accepted next 3D point.
    double step_len; // Accepted 3D step length.
    bool hit_boundary; // Whether the accepted step reached an open boundary.
};

/// Direction of the next step, reusing the previous one up to three times at tangencies.
bool march_direction(const SurfaceSurfaceField& field, SurfaceSurfaceMarch& m, double dir_sign) {

    if (field.tangent(m.x, dir_sign, m.d, m.sa, m.sau, m.sav, m.sbu, m.sbv)) {
        m.tang_reuse = 0;

        return true;
    }

    if (!m.have_prev_d || m.tang_reuse >= 3) {
        m.why = "tangency";

        return false;
    }

    m.d = m.prev_d;
    m.tang_reuse += 1;

    return true;
}


/// Parameters after a step h along d, cut back at open boundaries, and the predicted point; false when a surface is singular.
bool march_predict(const SurfaceSurfaceField& field, SurfaceSurfaceMarch& m, std::array<double, 3>& p_pred) {

    const Vector& sau = m.sau;
    const Vector& sav = m.sav;
    const Vector& sbu = m.sbu;
    const Vector& sbv = m.sbv;
    const std::array<double, 3>& d = m.d;

    std::vector<std::vector<double>> ma = {
        {sau[0] * sau[0] + sau[1] * sau[1] + sau[2] * sau[2], sau[0] * sav[0] + sau[1] * sav[1] + sau[2] * sav[2]},
        {sau[0] * sav[0] + sau[1] * sav[1] + sau[2] * sav[2], sav[0] * sav[0] + sav[1] * sav[1] + sav[2] * sav[2]}
    };

    std::vector<double> ra = {m.h * (d[0] * sau[0] + d[1] * sau[1] + d[2] * sau[2]), m.h * (d[0] * sav[0] + d[1] * sav[1] + d[2] * sav[2])};

    std::vector<std::vector<double>> mb = {
        {sbu[0] * sbu[0] + sbu[1] * sbu[1] + sbu[2] * sbu[2], sbu[0] * sbv[0] + sbu[1] * sbv[1] + sbu[2] * sbv[2]},
        {sbu[0] * sbv[0] + sbu[1] * sbv[1] + sbu[2] * sbv[2], sbv[0] * sbv[0] + sbv[1] * sbv[1] + sbv[2] * sbv[2]}
    };

    std::vector<double> rb = {m.h * (d[0] * sbu[0] + d[1] * sbu[1] + d[2] * sbu[2]), m.h * (d[0] * sbv[0] + d[1] * sbv[1] + d[2] * sbv[2])};
    std::vector<double> duv_a;
    std::vector<double> duv_b;

    if (!solve_gauss(ma, ra, 2, duv_a) || !solve_gauss(mb, rb, 2, duv_b))
        return false;

    double delta[4] = {duv_a[0], duv_a[1], duv_b[0], duv_b[1]};
    double tc = 1.0;
    m.hit_boundary = false;

    for (int k = 0; k < 4; k++) {
        if (field.closed[k] || std::abs(delta[k]) < 1e-15)
            continue;

        if (m.x[k] + delta[k] > field.hi[k]) {
            tc = std::min(tc, (field.hi[k] - m.x[k]) / delta[k]);
            m.hit_boundary = true;
        }

        if (m.x[k] + delta[k] < field.lo[k]) {
            tc = std::min(tc, (field.lo[k] - m.x[k]) / delta[k]);
            m.hit_boundary = true;
        }
    }

    for (int k = 0; k < 4; k++)
        m.xn[k] = m.x[k] + tc * delta[k];

    p_pred = {m.sa[0] + d[0] * m.h * tc, m.sa[1] + d[1] * m.h * tc, m.sa[2] + d[2] * m.h * tc};

    return true;
}

/// Whether the step from p_prev to p_cur turns more than acos(0.985) from the previous direction.
bool march_turns_sharply(const SurfaceSurfaceMarch& m) {

    double sd0 = (m.p_cur[0] - m.p_prev[0]) / m.step_len;
    double sd1 = (m.p_cur[1] - m.p_prev[1]) / m.step_len;
    double sd2 = (m.p_cur[2] - m.p_prev[2]) / m.step_len;

    return sd0 * m.prev_d[0] + sd1 * m.prev_d[1] + sd2 * m.prev_d[2] < 0.985;
}

/// Up to seven attempts at one step, halving h after a failed corrector or a sharp turn.
bool march_step(const SurfaceSurfaceField& field, SurfaceSurfaceMarch& m) {

    int attempts = 0;

    while (attempts < 7) {
        std::array<double, 3> p_pred;

        if (!march_predict(field, m, p_pred)) {
            m.why = "singular";

            return false;
        }

        if (!field.correct(m.xn, true, m.d, p_pred)) {
            m.why = "corrector";
            m.h *= 0.5;
            attempts += 1;
            m.smooth = 0;
            continue;
        }

        m.p_cur = field.point(m.xn);
        m.step_len = triple_distance(m.p_cur, m.p_prev);

        if (m.have_prev_d && m.step_len > 1e-14 && march_turns_sharply(m) && attempts < 6 && !m.hit_boundary) {
            m.why = "angle";
            m.h *= 0.5;
            attempts += 1;
            m.smooth = 0;
            continue;
        }

        return true;
    }

    return false;
}

/// Mark the unused seeds within the consume tolerance of p as used.
void consume_seeds_near(const SurfaceSurfaceField& field, const std::array<double, 3>& p, std::vector<SurfaceSurfaceSeed>& seeds) {

    for (SurfaceSurfaceSeed& sd : seeds)
        if (!sd.used && triple_distance(p, field.point({sd.u, sd.v, 0.0, 0.0})) < field.consume_tol)
            sd.used = true;
}

/// Marching state at x0 with the initial step and no previous direction.
SurfaceSurfaceMarch start_march(const SurfaceSurfaceField& field, const std::array<double, 4>& x0, const std::array<double, 3>& p_start) {

    SurfaceSurfaceMarch m;
    m.x = x0;
    m.have_prev_d = false;
    m.prev_d = {0.0, 0.0, 0.0};
    m.p_prev = p_start;
    m.h = field.h_init;
    m.smooth = 0;
    m.tang_reuse = 0;
    m.why = "maxsteps";
    m.xn = {0.0, 0.0, 0.0, 0.0};
    m.p_cur = {0.0, 0.0, 0.0};
    m.step_len = 0.0;
    m.hit_boundary = false;

    return m;
}

/// March from x0 in direction dir_sign until it closes, leaves the domain, stalls or hits the step cap; true when it closed.
bool trace_dir(
    const SurfaceSurfaceField& field,
    const std::array<double, 4>& x0,
    double dir_sign,
    std::vector<SurfaceSurfaceSeed>& seeds,
    std::vector<std::array<double, 4>>& out,
    std::string_view& why_out
) {

    out.clear();
    const std::array<double, 3> p_start = field.point(x0);
    SurfaceSurfaceMarch m = start_march(field, x0, p_start);
    double dist_traveled = 0.0;

    for (int step_i = 0; step_i < field.max_steps; step_i++) {
        if (!march_direction(field, m, dir_sign) || !march_step(field, m))
            break;

        m.why = "maxsteps";
        m.prev_d = m.d;
        m.have_prev_d = true;
        m.smooth += 1;

        if (m.smooth >= 5 && m.h < field.h_init * 2.0) {
            m.h *= 1.4;
            m.smooth = 0;
        }

        m.x = m.xn;
        dist_traveled += m.step_len;
        out.push_back(m.x);

        if (dist_traveled > field.close_tol * 3.0 && triple_distance(m.p_cur, p_start) < field.close_tol) {
            why_out = "closed";

            return true;
        }

        m.p_prev = m.p_cur;

        if (m.hit_boundary) {
            m.why = "boundary";
            break;
        }

        consume_seeds_near(field, m.p_cur, seeds);
    }

    why_out = m.why;

    return false;
}

/// Shift closed parameters by whole periods so consecutive samples never jump more than half a period.
void unwrap_quad(const SurfaceSurfaceField& field, std::vector<std::array<double, 4>>& quad) {

    for (size_t i = 1; i < quad.size(); i++) {
        for (int k = 0; k < 4; k++) {
            if (!field.closed[k])
                continue;

            double jump = quad[i][k] - quad[i - 1][k];

            if (jump > field.range[k] * 0.5)
                quad[i][k] -= field.range[k];
            else if (jump < -field.range[k] * 0.5)
                quad[i][k] += field.range[k];
        }
    }
}

/// Trace both directions from a seed into one unwrapped run; false when it is too short.
bool trace_seed(
    const SurfaceSurfaceField& field,
    const std::array<double, 4>& x0,
    std::vector<SurfaceSurfaceSeed>& seeds,
    std::vector<std::array<double, 4>>& quad,
    bool& is_loop
) {

    std::vector<std::array<double, 4>> fwd;
    std::vector<std::array<double, 4>> bwd;
    std::string_view fwd_why = "?";
    std::string_view bwd_why = "?";
    bool fwd_closed = trace_dir(field, x0, +1, seeds, fwd, fwd_why);

    if (!fwd_closed)
        trace_dir(field, x0, -1, seeds, bwd, bwd_why);

    quad.clear();

    for (int i = (int)bwd.size() - 1; i >= 0; i--)
        quad.push_back(bwd[i]);

    quad.push_back(x0);

    for (const std::array<double, 4>& p : fwd)
        quad.push_back(p);

    const size_t min_pts = (!fwd_closed && fwd_why == "boundary" && bwd_why == "boundary") ? 2 : 4;

    if (quad.size() < min_pts)
        return false;

    unwrap_quad(field, quad);
    double gap = triple_distance(field.point(quad.front()), field.point(quad.back()));
    is_loop = fwd_closed || (quad.size() >= 6 && gap < field.close_tol);

    if (is_loop)
        quad.pop_back();

    return quad.size() >= min_pts;
}

/// Whether the quarter, half and three-quarter samples all lie within dup_tol of one kept run.
bool is_duplicate_quad(
    const std::vector<std::array<double, 3>>& trace_pts3,
    const std::vector<std::vector<std::array<double, 3>>>& kept_pts3,
    double dup_tol
) {

    const int m = (int)trace_pts3.size();

    for (const std::vector<std::array<double, 3>>& other : kept_pts3) {
        bool all_close = true;

        for (double f : {0.25, 0.5, 0.75}) {
            const std::array<double, 3>& cp = trace_pts3[(int)((m - 1) * f)];
            double dmin = dup_tol + 1.0;

            for (const std::array<double, 3>& op : other)
                dmin = std::min(dmin, triple_distance(cp, op));

            if (dmin > dup_tol) {
                all_close = false;
                break;
            }
        }

        if (all_close)
            return true;
    }

    return false;
}

/// Insert corrected midpoints into gaps longer than 1.5 median gaps, at most four passes.
void densify_quad(const SurfaceSurfaceField& field, std::vector<std::array<double, 4>>& quad) {

    const std::array<double, 3> dummy3 = {0.0, 0.0, 0.0};

    for (int gp = 0; gp < 4; gp++) {
        std::vector<double> gg;

        for (size_t i = 0; i + 1 < quad.size(); i++)
            gg.push_back(triple_distance(field.point(quad[i]), field.point(quad[i + 1])));

        if (gg.empty())
            break;

        std::sort(gg.begin(), gg.end());
        double med = gg[gg.size() / 2];

        if (med <= 0)
            break;

        bool changed = false;
        size_t i = 0;

        while (i + 1 < quad.size() && quad.size() < 4000) {
            if (triple_distance(field.point(quad[i]), field.point(quad[i + 1])) > 1.5 * med) {
                std::array<double, 4> midq;

                for (int k = 0; k < 4; k++)
                    midq[k] = (quad[i][k] + quad[i + 1][k]) * 0.5;

                if (field.correct(midq, false, dummy3, dummy3)) {
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
}

/// Append the loop start shifted by whole periods after the end; returns the shift.
std::array<double, 4> close_quad(const SurfaceSurfaceField& field, std::vector<std::array<double, 4>>& quad, bool is_loop) {

    std::array<double, 4> closure = {0.0, 0.0, 0.0, 0.0};

    if (!is_loop || quad.size() < 2)
        return closure;

    std::array<double, 4> virt = quad[0];

    for (int k = 0; k < 4; k++) {
        double jump = quad[0][k] - quad.back()[k];

        if (field.closed[k])
            jump = unwrap_period(jump, 0.0, field.range[k]);

        virt[k] = quad.back()[k] + jump;
        closure[k] = virt[k] - quad[0][k];
    }

    quad.push_back(virt);

    return closure;
}

/// Seam crossings (t, parameter, seam value) of the step pa -> pb, sorted by t.
std::vector<std::tuple<double, int, double>> quad_seam_crossings(
    const SurfaceSurfaceField& field,
    const std::array<double, 4>& pa,
    const std::array<double, 4>& pb
) {

    std::vector<std::tuple<double, int, double>> crossings;

    for (int k = 0; k < 4; k++) {
        if (!field.closed[k] || std::abs(pb[k] - pa[k]) <= 1e-15)
            continue;

        int k0 = (int)std::floor((pa[k] - field.lo[k]) / field.range[k]);
        int k1 = (int)std::floor((pb[k] - field.lo[k]) / field.range[k]);

        for (int j = std::min(k0, k1) + 1; j <= std::max(k0, k1); j++) {
            double seam = field.lo[k] + j * field.range[k];
            double t = (seam - pa[k]) / (pb[k] - pa[k]);

            if (0.0 < t && t < 1.0)
                crossings.push_back({t, k, seam});
        }
    }

    std::sort(crossings.begin(), crossings.end());

    return crossings;
}

/// Snap closed parameters of p that sit on a seam onto it; false when none moved.
bool snap_quad_to_seam(const SurfaceSurfaceField& field, const std::array<double, 4>& prev, std::array<double, 4>& p) {

    bool on_seam = false;

    for (int k = 0; k < 4; k++) {
        if (!field.closed[k])
            continue;

        double j = std::round((p[k] - field.lo[k]) / field.range[k]);
        double seam = field.lo[k] + j * field.range[k];

        if (std::abs(p[k] - seam) < field.range[k] * 1e-9 && std::abs(p[k] - prev[k]) > field.range[k] * 1e-9) {
            p[k] = seam;
            on_seam = true;
        }
    }

    return on_seam;
}

/// Insert corrected seam crossings into the run and record the indices of every seam sample.
void split_quad_at_seams(
    const SurfaceSurfaceField& field,
    const std::vector<std::array<double, 4>>& quad,
    std::vector<std::array<double, 4>>& out_pts,
    std::vector<int>& cross_idx
) {

    const std::array<double, 3> dummy3 = {0.0, 0.0, 0.0};
    out_pts.push_back(quad[0]);

    for (size_t i = 1; i < quad.size(); i++) {
        const std::array<double, 4>& pa = quad[i - 1];
        const std::array<double, 4>& pb = quad[i];

        for (const std::tuple<double, int, double>& crossing : quad_seam_crossings(field, pa, pb)) {
            const double t = std::get<0>(crossing);
            std::array<double, 4> cp;

            for (int k = 0; k < 4; k++)
                cp[k] = pa[k] + (pb[k] - pa[k]) * t;

            cp[std::get<1>(crossing)] = std::get<2>(crossing);
            field.correct(cp, false, dummy3, dummy3);
            out_pts.push_back(cp);
            cross_idx.push_back((int)out_pts.size() - 1);
        }

        out_pts.push_back(pb);

        if (i < quad.size() - 1 && snap_quad_to_seam(field, pa, out_pts.back()))
            cross_idx.push_back((int)out_pts.size() - 1);
    }
}

/// Seam-free pieces of a split run, the last loop piece wrapped past the start by the closure shift.
std::vector<std::pair<std::vector<std::array<double, 4>>, bool>> quad_pieces(
    const SurfaceSurfaceField& field,
    const std::vector<std::array<double, 4>>& out_pts,
    const std::vector<int>& cross_idx,
    bool is_loop,
    const std::array<double, 4>& closure
) {

    std::vector<std::pair<std::vector<std::array<double, 4>>, bool>> pieces;
    bool wrap_drift = false;

    for (int k = 0; k < 4; k++)
        if (std::abs(closure[k]) > field.range[k] * 0.5)
            wrap_drift = true;

    if (cross_idx.empty()) {
        pieces.push_back({out_pts, is_loop && !wrap_drift});

        return pieces;
    }

    if (!is_loop) {
        std::vector<int> bounds;
        bounds.push_back(0);
        bounds.insert(bounds.end(), cross_idx.begin(), cross_idx.end());
        bounds.push_back((int)out_pts.size() - 1);

        for (size_t bi = 0; bi + 1 < bounds.size(); bi++)
            if (bounds[bi + 1] > bounds[bi])
                pieces.push_back(
                    {std::vector<std::array<double, 4>>(out_pts.begin() + bounds[bi], out_pts.begin() + bounds[bi + 1] + 1), false}
                );

        return pieces;
    }

    for (size_t ci = 0; ci + 1 < cross_idx.size(); ci++)
        pieces.push_back(
            {std::vector<std::array<double, 4>>(out_pts.begin() + cross_idx[ci], out_pts.begin() + cross_idx[ci + 1] + 1), false}
        );

    std::vector<std::array<double, 4>> wrap_piece(out_pts.begin() + cross_idx.back(), out_pts.end());

    for (int pi = 1; pi <= cross_idx[0]; pi++) {
        std::array<double, 4> p;

        for (int k = 0; k < 4; k++)
            p[k] = out_pts[pi][k] + closure[k];

        wrap_piece.push_back(p);
    }

    pieces.push_back({wrap_piece, false});

    return pieces;
}

/// Shift closed parameters of a piece by whole periods so its middle sample lies in the domain.
void shift_quad_piece(const SurfaceSurfaceField& field, std::vector<std::array<double, 4>>& piece_pts) {

    const std::array<double, 4> mid = piece_pts[piece_pts.size() / 2];

    for (int k = 0; k < 4; k++) {
        if (!field.closed[k])
            continue;

        int k_s = (int)std::floor((mid[k] - field.lo[k]) / field.range[k]);

        if (k_s != 0)
            for (std::array<double, 4>& p : piece_pts)
                p[k] -= k_s * field.range[k];
    }
}

/// Distance of pm from the line through pa and pb, 0 for a degenerate chord.
double chord_deviation(const std::array<double, 3>& pa, const std::array<double, 3>& pm, const std::array<double, 3>& pb) {

    double ex = pb[0] - pa[0];
    double ey = pb[1] - pa[1];
    double ez = pb[2] - pa[2];
    double l2 = ex * ex + ey * ey + ez * ez;

    if (l2 <= 1e-30)
        return 0.0;

    double tt = ((pm[0] - pa[0]) * ex + (pm[1] - pa[1]) * ey + (pm[2] - pa[2]) * ez) / l2;
    std::array<double, 3> c{pa[0] + tt * ex, pa[1] + tt * ey, pa[2] + tt * ez};

    return triple_distance(pm, c);
}

/// Insert corrected midpoints that deviate from their chord, at most eight passes and 3000 samples.
void refine_quad_piece(const SurfaceSurfaceField& field, std::vector<std::array<double, 4>>& piece_pts) {

    const std::array<double, 3> dummy3 = {0.0, 0.0, 0.0};
    const double refine_tol = std::max(field.tolerance * 100.0, 5e-6);

    for (int dp = 0; dp < 8; dp++) {
        bool refined = false;
        std::vector<std::array<double, 4>> new_pp;
        new_pp.push_back(piece_pts[0]);

        for (size_t i = 0; i + 1 < piece_pts.size() && piece_pts.size() < 3000; i++) {
            std::array<double, 4> midq;

            for (int k = 0; k < 4; k++)
                midq[k] = (piece_pts[i][k] + piece_pts[i + 1][k]) * 0.5;

            if (field.correct(midq, false, dummy3, dummy3)) {
                double dev = chord_deviation(field.point(piece_pts[i]), field.point(midq), field.point(piece_pts[i + 1]));

                if (dev > refine_tol) {
                    new_pp.push_back(midq);
                    refined = true;
                }
            }

            new_pp.push_back(piece_pts[i + 1]);
        }

        piece_pts = new_pp;

        if (!refined)
            break;
    }
}

/// Sum of the turning angles along a 3D polyline.
double total_turning_3d(const std::vector<Point>& pts) {

    double turning = 0.0;

    for (size_t i = 1; i + 1 < pts.size(); i++) {
        double dx1 = pts[i][0] - pts[i - 1][0];
        double dy1 = pts[i][1] - pts[i - 1][1];
        double dz1 = pts[i][2] - pts[i - 1][2];
        double dx2 = pts[i + 1][0] - pts[i][0];
        double dy2 = pts[i + 1][1] - pts[i][1];
        double dz2 = pts[i + 1][2] - pts[i][2];
        double l1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + dz1 * dz1);
        double l2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);

        if (l1 > 1e-14 && l2 > 1e-14) {
            double c = (dx1 * dx2 + dy1 * dy2 + dz1 * dz2) / (l1 * l2);
            c = std::max(-1.0, std::min(1.0, c));
            turning += std::acos(c);
        }
    }

    return turning;
}

/// Cubic fitted to a traced run, CVs doubled until within fit_tol, interpolated when fitting fails.
NurbsCurve fit_track(const std::vector<Point>& pts, double fit_tol, bool is_loop) {

    const int mp = (int)pts.size();
    const std::vector<double> chords = chord_parameters(pts, is_loop);
    int target_cvs = std::max(8, (int)(total_turning_3d(pts) / 0.5) + 6);
    const int max_cvs = std::max(8, std::min(mp - 1, mp / 3));
    NurbsCurve best;
    double best_dev = std::numeric_limits<double>::infinity();

    while (target_cvs <= max_cvs) {
        NurbsCurve crv = NurbsCurve::create_fitted(pts, target_cvs, 3, is_loop);

        if (!crv.is_valid())
            break;

        double dev = fitted_max_deviation(crv, pts, chords, 24);

        if (dev < best_dev) {
            best = crv;
            best_dev = dev;
        }

        if (dev < fit_tol)
            break;

        target_cvs *= 2;
    }

    if (best_dev >= fit_tol) {
        NurbsCurve interp = is_loop ? NurbsCurve::create_interpolated(pts, CurveNurbsKnotStyle::ChordPeriodic)
                                    : NurbsCurve::create_interpolated(pts);

        if (interp.is_valid())
            best = interp;
    }

    if (best.is_valid())
        best.set_domain(0.0, 1.0);

    return best;
}

/// Section triple of one seam-free piece: refined, then its 3D curve and both pcurves fitted.
bool piece_triple(
    const SurfaceSurfaceField& field,
    std::vector<std::array<double, 4>>& piece_pts,
    bool piece_loop,
    std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& triple
) {

    shift_quad_piece(field, piece_pts);
    double chord3 = 0.0;

    for (size_t i = 1; i < piece_pts.size(); i++)
        chord3 += triple_distance(field.point(piece_pts[i]), field.point(piece_pts[i - 1]));

    if (chord3 < field.h_init * 0.05)
        return false;

    refine_quad_piece(field, piece_pts);
    std::vector<Point> pts3(piece_pts.size());
    std::vector<Point> pts_pa(piece_pts.size());
    std::vector<Point> pts_pb(piece_pts.size());

    for (size_t i = 0; i < piece_pts.size(); i++) {
        const std::array<double, 3> p = field.point(piece_pts[i]);
        pts3[i] = Point(p[0], p[1], p[2]);
        pts_pa[i] = Point(piece_pts[i][0], piece_pts[i][1], 0.0);
        pts_pb[i] = Point(piece_pts[i][2], piece_pts[i][3], 0.0);
    }

    NurbsCurve crv3 = fit_track(pts3, std::max(field.tolerance * 10.0, 1e-7), piece_loop);
    NurbsCurve pcurve_a = fit_track(pts_pa, std::min(field.step[0], field.step[1]) * 1e-4, piece_loop);
    NurbsCurve pcurve_b = fit_track(pts_pb, std::min(field.step[2], field.step[3]) * 1e-4, piece_loop);

    if (!crv3.is_valid() || !pcurve_a.is_valid() || !pcurve_b.is_valid())
        return false;

    triple = std::make_tuple(std::move(crv3), std::move(pcurve_a), std::move(pcurve_b));

    return true;
}

/// Section triples of two freeform surfaces by seeding, marching and fitting every trace.
std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> marched_section_triples(
    const NurbsSurface& a,
    const NurbsSurface& b,
    double tolerance
) {

    const SurfaceSurfaceField field(a, b, tolerance);
    const std::array<double, 3> dummy3 = {0.0, 0.0, 0.0};
    std::vector<SurfaceSurfaceSeed> seeds = surface_surface_seeds(field);
    std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> result;
    std::vector<std::vector<std::array<double, 3>>> kept_pts3;

    for (size_t si = 0; si < seeds.size(); si++) {
        if (seeds[si].used)
            continue;

        seeds[si].used = true;
        std::array<double, 4> x0 = {seeds[si].u, seeds[si].v, seeds[si].s, seeds[si].t};
        std::vector<std::array<double, 4>> quad;
        bool is_loop = false;

        if (!field.correct(x0, false, dummy3, dummy3) || !trace_seed(field, x0, seeds, quad, is_loop))
            continue;

        std::vector<std::array<double, 3>> trace_pts3(quad.size());

        for (size_t i = 0; i < quad.size(); i++)
            trace_pts3[i] = field.point(quad[i]);

        if (is_duplicate_quad(trace_pts3, kept_pts3, field.h_init * 2.0))
            continue;

        kept_pts3.push_back(trace_pts3);
        densify_quad(field, quad);
        const std::array<double, 4> closure = close_quad(field, quad, is_loop);
        std::vector<std::array<double, 4>> out_pts;
        std::vector<int> cross_idx;
        split_quad_at_seams(field, quad, out_pts, cross_idx);

        for (std::pair<std::vector<std::array<double, 4>>, bool>& piece : quad_pieces(field, out_pts, cross_idx, is_loop, closure)) {
            std::tuple<NurbsCurve, NurbsCurve, NurbsCurve> triple;

            if (piece.first.size() >= 2 && piece_triple(field, piece.first, piece.second, triple))
                result.push_back(std::move(triple));
        }
    }

    drop_point_sections(result, tolerance);

    return result;
}

} // namespace

std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> Intersection::surface_surface(
    const NurbsSurface& a,
    const NurbsSurface& b,
    double tolerance
) {

    if (!a.is_valid() || !b.is_valid())
        return {};

    if (tolerance <= 0.0)
        tolerance = Tolerance::ZERO_TOLERANCE;

    AnalyticResult analytic = analytic_ssi(a, b, tolerance);

    if (analytic.status != AnalyticResult::NOT_ANALYTIC) {
        drop_point_sections(analytic.triples, tolerance);

        return analytic.triples;
    }

    if (a.is_planar(nullptr, 1e-9))
        return planar_section_triples(a, b, true, tolerance);

    if (b.is_planar(nullptr, 1e-9))
        return planar_section_triples(b, a, false, tolerance);

    return marched_section_triples(a, b, tolerance);
}

namespace {

/// Distance from a pcurve's lifted point to the cutter, projected onto the corner frame when it is not degenerate.
class CutterGap {
public:
    const NurbsSurface& target; // Surface the pcurve lives on.
    const NurbsCurve& pc; // Pcurve on the target.
    const NurbsSurface& cutter; // Cutting surface.
    Point q00; // Cutter corner at (u0, v0).
    Vector eu; // Cutter edge to (u1, v0).
    Vector ev; // Cutter edge to (u0, v1).
    double eu2; // Squared length of eu.
    double ev2; // Squared length of ev.
    bool fast_planar; // Whether both edges are usable.

    /// Corner frame of the cutter.
    CutterGap(const NurbsSurface& target_, const NurbsCurve& pc_, const NurbsSurface& cutter_);

    /// Distance to the cutter at pcurve parameter t.
    double gap(double t) const;
};

CutterGap::CutterGap(const NurbsSurface& target_, const NurbsCurve& pc_, const NurbsSurface& cutter_)
    : target(target_), pc(pc_), cutter(cutter_) {

    const std::pair<double, double> cu = cutter.domain(0);
    const std::pair<double, double> cv = cutter.domain(1);
    q00 = cutter.point_at(cu.first, cv.first);
    Point q10 = cutter.point_at(cu.second, cv.first);
    Point q01 = cutter.point_at(cu.first, cv.second);
    eu = Vector(q10[0] - q00[0], q10[1] - q00[1], q10[2] - q00[2]);
    ev = Vector(q01[0] - q00[0], q01[1] - q00[1], q01[2] - q00[2]);
    eu2 = eu[0] * eu[0] + eu[1] * eu[1] + eu[2] * eu[2];
    ev2 = ev[0] * ev[0] + ev[1] * ev[1] + ev[2] * ev[2];
    fast_planar = (eu2 > 1e-28 && ev2 > 1e-28);
}

double CutterGap::gap(double t) const {

    Point uv = pc.point_at(t);
    Point p3 = target.point_at(uv[0], uv[1]);

    if (!fast_planar)
        return std::get<2>(Closest::surface_point(cutter, p3, 0.0, 0.0, 0.0, 0.0));

    double dx = p3[0] - q00[0];
    double dy = p3[1] - q00[1];
    double dz = p3[2] - q00[2];
    double a = (dx * eu[0] + dy * eu[1] + dz * eu[2]) / eu2;
    double b = (dx * ev[0] + dy * ev[1] + dz * ev[2]) / ev2;
    a = std::min(std::max(a, 0.0), 1.0);
    b = std::min(std::max(b, 0.0), 1.0);
    double cx = q00[0] + a * eu[0] + b * ev[0];
    double cy = q00[1] + a * eu[1] + b * ev[1];
    double cz = q00[2] + a * eu[2] + b * ev[2];

    return std::sqrt((p3[0] - cx) * (p3[0] - cx) + (p3[1] - cy) * (p3[1] - cy) + (p3[2] - cz) * (p3[2] - cz));
}

/// Footprint edge between an inside and an outside parameter, by 24 bisections.
double refine_footprint_edge(const CutterGap& g, double t_in, double t_out, double edge_tol) {

    double a = t_in;
    double b = t_out;

    for (int k = 0; k < 24; ++k) {
        double tm = (a + b) * 0.5;

        if (g.gap(tm) < edge_tol)
            a = tm;
        else
            b = tm;
    }

    return b;
}

/// Parameter spans of the pcurve inside the footprint from n + 1 samples, ends bisected to the edge.
std::vector<std::pair<double, double>> footprint_spans(const CutterGap& g, int n, double on_tol, double edge_tol) {

    const std::pair<double, double> dc = g.pc.domain();
    const double d0 = dc.first;
    const double d1 = dc.second;
    std::vector<std::pair<double, bool>> flags;
    flags.reserve(n + 1);

    for (int i = 0; i <= n; ++i) {
        double t = d0 + (d1 - d0) * i / n;
        flags.emplace_back(t, g.gap(t) < on_tol);
    }

    std::vector<std::pair<double, double>> spans;
    int i = 0;

    while (i <= n) {
        if (!flags[i].second) {
            ++i;
            continue;
        }

        int j = i;

        while (j + 1 <= n && flags[j + 1].second)
            ++j;

        double ta = (i == 0) ? flags[i].first : refine_footprint_edge(g, flags[i].first, flags[i - 1].first, edge_tol);
        double tb = (j == n) ? flags[j].first : refine_footprint_edge(g, flags[j].first, flags[j + 1].first, edge_tol);

        if (tb - ta > (d1 - d0) * 1e-6)
            spans.push_back({ta, tb});

        i = j + 1;
    }

    return spans;
}

/// Join the first and last spans of a closed pcurve across its start into one polyline piece.
void join_wrapped_spans(const NurbsCurve& pc, int n, std::vector<std::pair<double, double>>& spans, std::vector<NurbsCurve>& pieces) {

    const std::pair<double, double> dc = pc.domain();
    const double d0 = dc.first;
    const double d1 = dc.second;
    bool pc_closed = pc.point_at(d0).distance(pc.point_at(d1)) < 1e-9;

    if (!pc_closed || spans.size() < 2 || spans.front().first > d0 + (d1 - d0) * 1e-9 || spans.back().second < d1 - (d1 - d0) * 1e-9)
        return;

    double ta = spans.back().first;
    double tb = spans.front().second;
    spans.pop_back();
    spans.erase(spans.begin());
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
}

/// Keep the pcurve sub-segments whose lifted 3D point lies inside the cutter footprint.
std::vector<NurbsCurve> clip_pcurve_to_cutter(const NurbsSurface& target, const NurbsCurve& pc, const NurbsSurface& cutter) {

    const CutterGap g(target, pc, cutter);
    const std::pair<double, double> cu = cutter.domain(0);
    const std::pair<double, double> cv = cutter.domain(1);
    const double corner_diag = g.q00.distance(cutter.point_at(cu.second, cv.second));
    const int n = std::max(pc.cv_count() * 4, 16);
    std::vector<std::pair<double, double>> spans = footprint_spans(
        g,
        n,
        std::max(1e-6, corner_diag * 2e-3),
        std::max(1e-6, corner_diag * 2e-4)
    );
    std::vector<NurbsCurve> pieces;
    join_wrapped_spans(pc, n, spans, pieces);

    for (std::pair<double, double>& sp : spans) {
        NurbsCurve piece = pc;

        if (piece.trim(sp.first, sp.second) && piece.is_valid())
            pieces.push_back(piece);
    }

    return pieces;
}

/// Pcurves of one section on the target: analytic, pulled back, projected, then the traced pcurve.
std::vector<NurbsCurve> target_pcurves(
    const NurbsSurface& target,
    const RecogSurface& rt,
    const std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& tr,
    double tolerance
) {

    const NurbsCurve& c3d = std::get<0>(tr);
    const NurbsCurve& pa_tr = std::get<1>(tr);
    NurbsCurve pa_an = analytic_pcurve(target, rt, c3d);

    if (pa_an.is_valid())
        return {pa_an};

    if (rt.kind == RecogSurface::NONE || rt.kind == RecogSurface::PLANE) {
        if (pa_tr.is_valid())
            return {pa_tr};

        return Closest::surface_curve(target, c3d, 0.0, 0.0, tolerance);
    }

    std::vector<NurbsCurve> pcs = analytic_pullback(target, rt, c3d);

    if (pcs.empty())
        pcs = Closest::surface_curve(target, c3d, 0.0, 0.0, tolerance);

    if (pcs.empty())
        pcs.push_back(pa_tr);

    return pcs;
}

} // namespace

std::vector<NurbsCurve> Intersection::cut_curves_on_surface(
    const NurbsSurface& target,
    const NurbsSurface& cutter,
    double tolerance
) {

    std::vector<NurbsCurve> out;
    bool cutter_planar = cutter.is_planar(nullptr, 1e-6);
    RecogSurface rt = recognize_surface(target, std::max(tolerance, 1e-7) * 1e4);

    for (std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>& tr : surface_surface(target, cutter, tolerance)) {
        for (const NurbsCurve& pc : target_pcurves(target, rt, tr, tolerance)) {
            if (!cutter_planar) {
                out.push_back(pc);
                continue;
            }

            std::vector<NurbsCurve> clipped = clip_pcurve_to_cutter(target, pc, cutter);
            out.insert(out.end(), clipped.begin(), clipped.end());
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
    t = clamp_unit((vx * dx + vy * dy + vz * dz) / len_sq);
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
