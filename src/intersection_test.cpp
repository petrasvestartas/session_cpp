#include "mini_test.h"
#include "intersection.h"
#include "tolerance.h"
#include <cmath>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Intersection", "line_line") {
    Line line0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line line1(0.5, -1.0, 0.0, 0.5, 1.0, 0.0);

    Point output;
    bool result = Intersection::line_line(line0, line1, output, Tolerance::APPROXIMATION);

    MINI_CHECK(result);
    MINI_CHECK(TOLERANCE.is_close(output[0], 0.5));
    MINI_CHECK(TOLERANCE.is_close(output[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(output[2], 0.0));
}

MINI_TEST("Intersection", "line_line_parallel") {
    Line line0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line line1(0.0, 1.0, 0.0, 1.0, 1.0, 0.0);

    Point output;
    bool result = Intersection::line_line(line0, line1, output, Tolerance::APPROXIMATION);

    MINI_CHECK(!result);
}

MINI_TEST("Intersection", "line_line_parameters") {
    Line line0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line line1(0.5, -1.0, 0.0, 0.5, 1.0, 0.0);

    double t0, t1;
    bool result = Intersection::line_line_parameters(line0, line1, t0, t1, Tolerance::APPROXIMATION);

    MINI_CHECK(result);
    MINI_CHECK(TOLERANCE.is_close(t0, 0.5));
    MINI_CHECK(TOLERANCE.is_close(t1, 0.5));
}

MINI_TEST("Intersection", "line_line_parameters_endpoints") {
    Line line0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line line1(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    double t0, t1;
    bool result = Intersection::line_line_parameters(line0, line1, t0, t1, Tolerance::APPROXIMATION);

    MINI_CHECK(result);
    MINI_CHECK(TOLERANCE.is_close(t0, 0.0));
    MINI_CHECK(TOLERANCE.is_close(t1, 0.0));
}

MINI_TEST("Intersection", "line_line_parameters_infinite") {
    Line line0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line line1(2.0, -1.0, 0.0, 2.0, 1.0, 0.0);

    double t0, t1;
    bool result = Intersection::line_line_parameters(line0, line1, t0, t1, static_cast<double>(Tolerance::APPROXIMATION), false);

    MINI_CHECK(result);
    MINI_CHECK(TOLERANCE.is_close(t0, 2.0));
}

MINI_TEST("Intersection", "plane_plane") {
    Point p0(0.0, 0.0, 0.0);
    Vector n0(0.0, 0.0, 1.0);
    Plane plane0 = Plane::from_point_normal(p0, n0);

    Point p1(0.0, 0.0, 0.0);
    Vector n1(0.0, 1.0, 0.0);
    Plane plane1 = Plane::from_point_normal(p1, n1);

    Line output;
    bool result = Intersection::plane_plane(plane0, plane1, output);

    MINI_CHECK(result);

    Vector line_dir = output.to_vector();
    MINI_CHECK(std::fabs(std::fabs(line_dir[0]) - 1.0) < 1e-4);
    MINI_CHECK(std::fabs(line_dir[1]) < 1e-4);
    MINI_CHECK(std::fabs(line_dir[2]) < 1e-4);
}

MINI_TEST("Intersection", "plane_plane_complex") {
    Point plane_origin_0(213.787107, 513.797811, -24.743845);
    Vector plane_xaxis_0(0.907673, -0.258819, 0.330366);
    Vector plane_yaxis_0(0.272094, 0.96225, 0.006285);
    Plane pl0(plane_origin_0, plane_xaxis_0, plane_yaxis_0);

    Point plane_origin_1(247.17924, 499.115486, 59.619568);
    Vector plane_xaxis_1(0.552465, 0.816035, 0.16991);
    Vector plane_yaxis_1(0.172987, 0.087156, -0.98106);
    Plane pl1(plane_origin_1, plane_xaxis_1, plane_yaxis_1);

    Line intersection_line;
    bool result = Intersection::plane_plane(pl0, pl1, intersection_line);

    MINI_CHECK(result);

    Point start = intersection_line.start();
    Point end = intersection_line.end();

    MINI_CHECK(std::fabs(start[0] - 252.4632) < 0.01);
    MINI_CHECK(std::fabs(start[1] - 495.32248) < 0.01);
    MINI_CHECK(std::fabs(start[2] - (-10.002656)) < 0.01);

    MINI_CHECK(std::fabs(end[0] - 253.01033) < 0.01);
    MINI_CHECK(std::fabs(end[1] - 496.1218) < 0.01);
    MINI_CHECK(std::fabs(end[2] - (-9.888727)) < 0.01);
}

MINI_TEST("Intersection", "line_plane") {
    Point p(0.0, 0.0, 1.0);
    Vector n(0.0, 0.0, 1.0);
    Plane plane = Plane::from_point_normal(p, n);

    Line line(0.0, 0.0, 0.0, 0.0, 0.0, 2.0);

    Point output;
    bool result = Intersection::line_plane(line, plane, output, true);

    MINI_CHECK(result);
    MINI_CHECK(TOLERANCE.is_close(output[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(output[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(output[2], 1.0));
}

MINI_TEST("Intersection", "line_plane_parallel") {
    Point p(0.0, 0.0, 1.0);
    Vector n(0.0, 0.0, 1.0);
    Plane plane = Plane::from_point_normal(p, n);

    Line line(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

    Point output;
    bool result = Intersection::line_plane(line, plane, output, true);

    MINI_CHECK(!result);
}

MINI_TEST("Intersection", "line_plane_real_world") {
    Line l0(500.000, -573.576, -819.152, 500.000, 573.576, 819.152);

    Point plane_origin_0(213.787107, 513.797811, -24.743845);
    Vector plane_xaxis_0(0.907673, -0.258819, 0.330366);
    Vector plane_yaxis_0(0.272094, 0.96225, 0.006285);
    Plane pl0(plane_origin_0, plane_xaxis_0, plane_yaxis_0);

    Point lp;
    bool result = Intersection::line_plane(l0, pl0, lp);

    MINI_CHECK(result);
    MINI_CHECK(std::fabs(lp[0] - 500.0) < 0.1);
    MINI_CHECK(std::fabs(lp[1] - 77.7531) < 0.01);
    MINI_CHECK(std::fabs(lp[2] - 111.043) < 0.01);
}

MINI_TEST("Intersection", "plane_plane_plane") {
    Point plane_origin_0(213.787107, 513.797811, -24.743845);
    Vector plane_xaxis_0(0.907673, -0.258819, 0.330366);
    Vector plane_yaxis_0(0.272094, 0.96225, 0.006285);
    Plane pl0(plane_origin_0, plane_xaxis_0, plane_yaxis_0);

    Point plane_origin_1(247.17924, 499.115486, 59.619568);
    Vector plane_xaxis_1(0.552465, 0.816035, 0.16991);
    Vector plane_yaxis_1(0.172987, 0.087156, -0.98106);
    Plane pl1(plane_origin_1, plane_xaxis_1, plane_yaxis_1);

    Point plane_origin_2(221.399816, 605.893667, -54.000116);
    Vector plane_xaxis_2(0.903451, -0.360516, -0.231957);
    Vector plane_yaxis_2(0.172742, -0.189057, 0.966653);
    Plane pl2(plane_origin_2, plane_xaxis_2, plane_yaxis_2);

    Point output;
    bool result = Intersection::plane_plane_plane(pl0, pl1, pl2, output);

    MINI_CHECK(result);
    MINI_CHECK(std::fabs(output[0] - 300.5) < 0.1);
    MINI_CHECK(std::fabs(output[1] - 565.5) < 0.1);
    MINI_CHECK(std::fabs(output[2] - 0.0) < 0.1);
}

MINI_TEST("Intersection", "plane_plane_plane_parallel") {
    Point p0(0.0, 0.0, 0.0);
    Vector n0(0.0, 0.0, 1.0);
    Plane plane0 = Plane::from_point_normal(p0, n0);

    Point p1(0.0, 0.0, 1.0);
    Vector n1(0.0, 0.0, 1.0);
    Plane plane1 = Plane::from_point_normal(p1, n1);

    Point p2(0.0, 0.0, 0.0);
    Vector n2(1.0, 0.0, 0.0);
    Plane plane2 = Plane::from_point_normal(p2, n2);

    Point output;
    bool result = Intersection::plane_plane_plane(plane0, plane1, plane2, output);

    MINI_CHECK(!result);
}

MINI_TEST("Intersection", "ray_box") {
    Point center(0.0, 0.0, 0.0);
    Vector x_axis(1.0, 0.0, 0.0);
    Vector y_axis(0.0, 1.0, 0.0);
    Vector z_axis(0.0, 0.0, 1.0);
    Vector half_size(1.0, 1.0, 1.0);
    BoundingBox box(center, x_axis, y_axis, z_axis, half_size);

    Point origin(-5.0, 0.0, 0.0);
    Vector direction(1.0, 0.0, 0.0);

    double tmin, tmax;
    bool result = Intersection::ray_box(origin, direction, box, 0.0, 100.0, tmin, tmax);

    MINI_CHECK(result);
    MINI_CHECK(std::fabs(tmin - 4.0) < 1e-4);
    MINI_CHECK(std::fabs(tmax - 6.0) < 1e-4);
}

MINI_TEST("Intersection", "ray_box_miss") {
    Point center(0.0, 0.0, 0.0);
    Vector x_axis(1.0, 0.0, 0.0);
    Vector y_axis(0.0, 1.0, 0.0);
    Vector z_axis(0.0, 0.0, 1.0);
    Vector half_size(1.0, 1.0, 1.0);
    BoundingBox box(center, x_axis, y_axis, z_axis, half_size);

    Point origin(-5.0, 5.0, 0.0);
    Vector direction(1.0, 0.0, 0.0);

    double tmin, tmax;
    bool result = Intersection::ray_box(origin, direction, box, 0.0, 100.0, tmin, tmax);

    MINI_CHECK(!result);
}

MINI_TEST("Intersection", "ray_sphere") {
    Point origin(-5.0, 0.0, 0.0);
    Vector direction(1.0, 0.0, 0.0);
    Point center(0.0, 0.0, 0.0);
    double radius = 2.0;

    double t0, t1;
    int hits = Intersection::ray_sphere(origin, direction, center, radius, t0, t1);

    MINI_CHECK(hits == 2);
    MINI_CHECK(std::fabs(t0 - 3.0) < 1e-4);
    MINI_CHECK(std::fabs(t1 - 7.0) < 1e-4);
}

MINI_TEST("Intersection", "ray_sphere_tangent") {
    Point origin(-5.0, 2.0, 0.0);
    Vector direction(1.0, 0.0, 0.0);
    Point center(0.0, 0.0, 0.0);
    double radius = 2.0;

    double t0, t1;
    int hits = Intersection::ray_sphere(origin, direction, center, radius, t0, t1);

    MINI_CHECK(hits == 1);
    MINI_CHECK(std::fabs(t0 - 5.0) < 1e-4);
}

MINI_TEST("Intersection", "ray_sphere_miss") {
    Point origin(-5.0, 5.0, 0.0);
    Vector direction(1.0, 0.0, 0.0);
    Point center(0.0, 0.0, 0.0);
    double radius = 2.0;

    double t0, t1;
    int hits = Intersection::ray_sphere(origin, direction, center, radius, t0, t1);

    MINI_CHECK(hits == 0);
}

MINI_TEST("Intersection", "ray_triangle") {
    Point origin(0.5, 0.5, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    Point v0(0.0, 0.0, 0.0);
    Point v1(1.0, 0.0, 0.0);
    Point v2(0.0, 1.0, 0.0);

    double t, u, v;
    bool parallel;
    bool result = Intersection::ray_triangle(origin, direction, v0, v1, v2, 1e-6, t, u, v, parallel);

    MINI_CHECK(result);
    MINI_CHECK(!parallel);
    MINI_CHECK(std::fabs(t - 1.0) < 1e-4);
}

MINI_TEST("Intersection", "ray_triangle_miss") {
    Point origin(2.0, 2.0, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    Point v0(0.0, 0.0, 0.0);
    Point v1(1.0, 0.0, 0.0);
    Point v2(0.0, 1.0, 0.0);

    double t, u, v;
    bool parallel;
    bool result = Intersection::ray_triangle(origin, direction, v0, v1, v2, 1e-6, t, u, v, parallel);

    MINI_CHECK(!result);
}

MINI_TEST("Intersection", "ray_triangle_parallel") {
    Point origin(0.5, 0.5, -1.0);
    Vector direction(1.0, 0.0, 0.0);

    Point v0(0.0, 0.0, 0.0);
    Point v1(1.0, 0.0, 0.0);
    Point v2(0.0, 1.0, 0.0);

    double t, u, v;
    bool parallel;
    bool result = Intersection::ray_triangle(origin, direction, v0, v1, v2, 1e-6, t, u, v, parallel);

    MINI_CHECK(!result);
    MINI_CHECK(parallel);
}

MINI_TEST("Intersection", "ray_mesh") {
    std::vector<std::vector<Point>> polygons = {
        {Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0), Point(0.0, 1.0, 0.0)},
        {Point(0.0, 0.0, 1.0), Point(1.0, 0.0, 1.0), Point(1.0, 1.0, 1.0), Point(0.0, 1.0, 1.0)}
    };

    Mesh mesh = Mesh::from_polygons(polygons);

    Point origin(0.5, 0.5, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh(origin, direction, mesh, hits, true);

    MINI_CHECK(result);
    MINI_CHECK(hits.size() >= 1);
    MINI_CHECK(std::fabs(hits[0].t - 1.0) < 1e-3);
}

MINI_TEST("Intersection", "ray_mesh_first") {
    std::vector<std::vector<Point>> polygons = {
        {Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0), Point(0.0, 1.0, 0.0)},
        {Point(0.0, 0.0, 1.0), Point(1.0, 0.0, 1.0), Point(1.0, 1.0, 1.0), Point(0.0, 1.0, 1.0)}
    };

    Mesh mesh = Mesh::from_polygons(polygons);

    Point origin(0.5, 0.5, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh(origin, direction, mesh, hits, false);

    MINI_CHECK(result);
    MINI_CHECK(hits.size() == 1);
}

MINI_TEST("Intersection", "ray_mesh_miss") {
    std::vector<std::vector<Point>> polygons = {
        {Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0), Point(0.0, 1.0, 0.0)}
    };

    Mesh mesh = Mesh::from_polygons(polygons);

    Point origin(5.0, 5.0, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh(origin, direction, mesh, hits, true);

    MINI_CHECK(!result);
    MINI_CHECK(hits.size() == 0);
}

MINI_TEST("Intersection", "ray_mesh_bvh") {
    std::vector<std::vector<Point>> polygons = {
        {Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0), Point(0.0, 1.0, 0.0)},
        {Point(0.0, 0.0, 1.0), Point(1.0, 0.0, 1.0), Point(1.0, 1.0, 1.0), Point(0.0, 1.0, 1.0)}
    };

    Mesh mesh = Mesh::from_polygons(polygons);

    Point origin(0.5, 0.5, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh_bvh(origin, direction, mesh, hits, true);

    MINI_CHECK(result);
    MINI_CHECK(hits.size() >= 1);
    MINI_CHECK(std::fabs(hits[0].t - 1.0) < 1e-3);
}

MINI_TEST("Intersection", "ray_mesh_bvh_first") {
    std::vector<std::vector<Point>> polygons = {
        {Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0), Point(0.0, 1.0, 0.0)},
        {Point(0.0, 0.0, 1.0), Point(1.0, 0.0, 1.0), Point(1.0, 1.0, 1.0), Point(0.0, 1.0, 1.0)}
    };

    Mesh mesh = Mesh::from_polygons(polygons);

    Point origin(0.5, 0.5, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh_bvh(origin, direction, mesh, hits, false);

    MINI_CHECK(result);
    MINI_CHECK(hits.size() == 1);
}

MINI_TEST("Intersection", "ray_mesh_bvh_miss") {
    std::vector<std::vector<Point>> polygons = {
        {Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0), Point(0.0, 1.0, 0.0)}
    };

    Mesh mesh = Mesh::from_polygons(polygons);

    Point origin(5.0, 5.0, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh_bvh(origin, direction, mesh, hits, true);

    MINI_CHECK(!result);
    MINI_CHECK(hits.size() == 0);
}

MINI_TEST("Intersection", "ray_mesh_bvh_vs_naive") {
    std::vector<std::vector<Point>> polygons;
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            double x = static_cast<double>(i);
            double y = static_cast<double>(j);
            polygons.push_back({
                Point(x, y, 0.0),
                Point(x + 1.0, y, 0.0),
                Point(x + 1.0, y + 1.0, 0.0),
                Point(x, y + 1.0, 0.0)
            });
        }
    }

    Mesh mesh = Mesh::from_polygons(polygons);

    Point origin(5.5, 5.5, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits_naive;
    bool result_naive = Intersection::ray_mesh(origin, direction, mesh, hits_naive, true);

    std::vector<Intersection::RayHit> hits_bvh;
    bool result_bvh = Intersection::ray_mesh_bvh(origin, direction, mesh, hits_bvh, true);

    MINI_CHECK(result_naive == result_bvh);
    MINI_CHECK(hits_naive.size() == hits_bvh.size());

    if (!hits_naive.empty()) {
        MINI_CHECK(std::fabs(hits_naive[0].t - hits_bvh[0].t) < 1e-4);
        MINI_CHECK(hits_naive[0].face_index == hits_bvh[0].face_index);
    }
}

MINI_TEST("Intersection", "ray_box_real_world") {
    Line l0(500.0, -573.576, -819.152, 500.0, 573.576, 819.152);
    Point min(214.0, 192.0, 484.0);
    Point max(694.0, 567.0, 796.0);
    std::vector<Point> points {min, max};
    BoundingBox box = BoundingBox::from_points(points);

    std::vector<Point> intersection_points;
    bool result = Intersection::ray_box(l0, box, 0.0, 1000.0, intersection_points);

    MINI_CHECK(result);
    MINI_CHECK(intersection_points.size() == 2);

    MINI_CHECK(std::fabs(intersection_points[0][0] - 500.0) < 0.1);
    MINI_CHECK(std::fabs(intersection_points[0][1] - 338.9) < 0.1);
    MINI_CHECK(std::fabs(intersection_points[0][2] - 484.0) < 0.1);

    MINI_CHECK(std::fabs(intersection_points[1][0] - 500.0) < 0.1);
    MINI_CHECK(std::fabs(intersection_points[1][1] - 557.365) < 0.1);
    MINI_CHECK(std::fabs(intersection_points[1][2] - 796.0) < 0.1);
}

MINI_TEST("Intersection", "ray_sphere_real_world") {
    Line l0(500.0, -573.576, -819.152, 500.0, 573.576, 819.152);
    Point sphere_center(457.0, 192.0, 207.0);
    double radius = 265.0;

    std::vector<Point> sphere_points;
    bool result = Intersection::ray_sphere(l0, sphere_center, radius, sphere_points);

    MINI_CHECK(result);
    MINI_CHECK(sphere_points.size() == 2);

    MINI_CHECK(std::fabs(sphere_points[0][0] - 500.0) < 0.1);
    MINI_CHECK(std::fabs(sphere_points[0][1] - 12.08) < 0.1);
    MINI_CHECK(std::fabs(sphere_points[0][2] - 17.25) < 0.1);

    MINI_CHECK(std::fabs(sphere_points[1][0] - 500.0) < 0.1);
    MINI_CHECK(std::fabs(sphere_points[1][1] - 308.77) < 0.1);
    MINI_CHECK(std::fabs(sphere_points[1][2] - 440.97) < 0.1);
}

MINI_TEST("Intersection", "ray_triangle_real_world") {
    Line l0(500.0, -573.576, -819.152, 500.0, 573.576, 819.152);
    Point p1(214.0, 567.0, 484.0);
    Point p2(214.0, 192.0, 796.0);
    Point p3(694.0, 192.0, 484.0);

    Point triangle_hit;
    bool result = Intersection::ray_triangle(l0, p1, p2, p3, Tolerance::APPROXIMATION, triangle_hit);

    MINI_CHECK(result);
    MINI_CHECK(std::fabs(triangle_hit[0] - 500.0) < 0.1);
    MINI_CHECK(std::fabs(triangle_hit[1] - 340.616) < 0.01);
    MINI_CHECK(std::fabs(triangle_hit[2] - 486.451) < 0.01);
}

} // namespace session_cpp
