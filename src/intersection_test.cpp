#include "mini_test.h"
#include "intersection.h"
#include "nurbssurface.h"
#include "tolerance.h"
#include <cmath>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Intersection", "Line Line") {
    // uncomment #include "intersection.h"
    // uncomment #include "line.h"
    // uncomment #include "point.h"
    Line line0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line line1(0.5, -1.0, 0.0, 0.5, 1.0, 0.0);

    Point output;
    bool result = Intersection::line_line(line0, line1, output, Tolerance::APPROXIMATION);

    MINI_CHECK(result);
    MINI_CHECK(TOLERANCE.is_close(output[0], 0.5));
    MINI_CHECK(TOLERANCE.is_close(output[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(output[2], 0.0));
}

MINI_TEST("Intersection", "Line Line Parallel") {
    // uncomment #include "intersection.h"
    // uncomment #include "line.h"
    // uncomment #include "point.h"
    Line line0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line line1(0.0, 1.0, 0.0, 1.0, 1.0, 0.0);

    Point output;
    bool result = Intersection::line_line(line0, line1, output, Tolerance::APPROXIMATION);

    MINI_CHECK(!result);
}

MINI_TEST("Intersection", "Line Line Parameters") {
    // uncomment #include "intersection.h"
    // uncomment #include "line.h"
    Line line0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line line1(0.5, -1.0, 0.0, 0.5, 1.0, 0.0);

    double t0, t1;
    bool result = Intersection::line_line_parameters(line0, line1, t0, t1, Tolerance::APPROXIMATION);

    MINI_CHECK(result);
    MINI_CHECK(TOLERANCE.is_close(t0, 0.5));
    MINI_CHECK(TOLERANCE.is_close(t1, 0.5));
}

MINI_TEST("Intersection", "Line Line Parameters Endpoints") {
    // uncomment #include "intersection.h"
    // uncomment #include "line.h"
    Line line0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line line1(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    double t0, t1;
    bool result = Intersection::line_line_parameters(line0, line1, t0, t1, Tolerance::APPROXIMATION);

    MINI_CHECK(result);
    MINI_CHECK(TOLERANCE.is_close(t0, 0.0));
    MINI_CHECK(TOLERANCE.is_close(t1, 0.0));
}

MINI_TEST("Intersection", "Line Line Parameters Infinite") {
    // uncomment #include "intersection.h"
    // uncomment #include "line.h"
    Line line0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line line1(2.0, -1.0, 0.0, 2.0, 1.0, 0.0);

    double t0, t1;
    bool result = Intersection::line_line_parameters(line0, line1, t0, t1, static_cast<double>(Tolerance::APPROXIMATION), false);

    MINI_CHECK(result);
    MINI_CHECK(TOLERANCE.is_close(t0, 2.0));
}

MINI_TEST("Intersection", "Plane Plane") {
    // uncomment #include "intersection.h"
    // uncomment #include "line.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
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

MINI_TEST("Intersection", "Plane Plane Complex") {
    // uncomment #include "intersection.h"
    // uncomment #include "line.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
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

MINI_TEST("Intersection", "Line Plane") {
    // uncomment #include "intersection.h"
    // uncomment #include "line.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
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

MINI_TEST("Intersection", "Line Plane Parallel") {
    // uncomment #include "intersection.h"
    // uncomment #include "line.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    Point p(0.0, 0.0, 1.0);
    Vector n(0.0, 0.0, 1.0);
    Plane plane = Plane::from_point_normal(p, n);

    Line line(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

    Point output;
    bool result = Intersection::line_plane(line, plane, output, true);

    MINI_CHECK(!result);
}

MINI_TEST("Intersection", "Line Plane Real World") {
    // uncomment #include "intersection.h"
    // uncomment #include "line.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
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

MINI_TEST("Intersection", "Plane Plane Plane") {
    // uncomment #include "intersection.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
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

MINI_TEST("Intersection", "Plane Plane Plane Parallel") {
    // uncomment #include "intersection.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
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

MINI_TEST("Intersection", "Ray Box") {
    // uncomment #include "intersection.h"
    // uncomment #include "obb.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    Point center(0.0, 0.0, 0.0);
    Vector x_axis(1.0, 0.0, 0.0);
    Vector y_axis(0.0, 1.0, 0.0);
    Vector z_axis(0.0, 0.0, 1.0);
    Vector half_size(1.0, 1.0, 1.0);
    OBB box(center, x_axis, y_axis, z_axis, half_size);

    Point origin(-5.0, 0.0, 0.0);
    Vector direction(1.0, 0.0, 0.0);

    double tmin, tmax;
    bool result = Intersection::ray_box(origin, direction, box, 0.0, 100.0, tmin, tmax);

    MINI_CHECK(result);
    MINI_CHECK(std::fabs(tmin - 4.0) < 1e-4);
    MINI_CHECK(std::fabs(tmax - 6.0) < 1e-4);
}

MINI_TEST("Intersection", "Ray Box Miss") {
    // uncomment #include "intersection.h"
    // uncomment #include "obb.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    Point center(0.0, 0.0, 0.0);
    Vector x_axis(1.0, 0.0, 0.0);
    Vector y_axis(0.0, 1.0, 0.0);
    Vector z_axis(0.0, 0.0, 1.0);
    Vector half_size(1.0, 1.0, 1.0);
    OBB box(center, x_axis, y_axis, z_axis, half_size);

    Point origin(-5.0, 5.0, 0.0);
    Vector direction(1.0, 0.0, 0.0);

    double tmin, tmax;
    bool result = Intersection::ray_box(origin, direction, box, 0.0, 100.0, tmin, tmax);

    MINI_CHECK(!result);
}

MINI_TEST("Intersection", "Ray Sphere") {
    // uncomment #include "intersection.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
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

MINI_TEST("Intersection", "Ray Sphere Tangent") {
    // uncomment #include "intersection.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    Point origin(-5.0, 2.0, 0.0);
    Vector direction(1.0, 0.0, 0.0);
    Point center(0.0, 0.0, 0.0);
    double radius = 2.0;

    double t0, t1;
    int hits = Intersection::ray_sphere(origin, direction, center, radius, t0, t1);

    MINI_CHECK(hits == 1);
    MINI_CHECK(std::fabs(t0 - 5.0) < 1e-4);
}

MINI_TEST("Intersection", "Ray Sphere Miss") {
    // uncomment #include "intersection.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    Point origin(-5.0, 5.0, 0.0);
    Vector direction(1.0, 0.0, 0.0);
    Point center(0.0, 0.0, 0.0);
    double radius = 2.0;

    double t0, t1;
    int hits = Intersection::ray_sphere(origin, direction, center, radius, t0, t1);

    MINI_CHECK(hits == 0);
}

MINI_TEST("Intersection", "Ray Triangle") {
    // uncomment #include "intersection.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
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

MINI_TEST("Intersection", "Ray Triangle Miss") {
    // uncomment #include "intersection.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
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

MINI_TEST("Intersection", "Ray Triangle Parallel") {
    // uncomment #include "intersection.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
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

MINI_TEST("Intersection", "Ray Mesh") {
    // uncomment #include "intersection.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    std::vector<std::vector<Point>> polygons = {
        {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        },
        {
            Point(0.0, 0.0, 1.0),
            Point(1.0, 0.0, 1.0),
            Point(1.0, 1.0, 1.0),
            Point(0.0, 1.0, 1.0),
        },
    };

    Mesh mesh = Mesh::from_polylines(polygons);

    Point origin(0.5, 0.5, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh(origin, direction, mesh, hits, true);

    MINI_CHECK(result);
    MINI_CHECK(hits.size() >= 1);
    MINI_CHECK(std::fabs(hits[0].t - 1.0) < 1e-3);
}

MINI_TEST("Intersection", "Ray Mesh First") {
    // uncomment #include "intersection.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    std::vector<std::vector<Point>> polygons = {
        {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        },
        {
            Point(0.0, 0.0, 1.0),
            Point(1.0, 0.0, 1.0),
            Point(1.0, 1.0, 1.0),
            Point(0.0, 1.0, 1.0),
        },
    };

    Mesh mesh = Mesh::from_polylines(polygons);

    Point origin(0.5, 0.5, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh(origin, direction, mesh, hits, false);

    MINI_CHECK(result);
    MINI_CHECK(hits.size() == 1);
}

MINI_TEST("Intersection", "Ray Mesh Miss") {
    // uncomment #include "intersection.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    std::vector<std::vector<Point>> polygons = {
        {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        },
    };

    Mesh mesh = Mesh::from_polylines(polygons);

    Point origin(5.0, 5.0, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh(origin, direction, mesh, hits, true);

    MINI_CHECK(!result);
    MINI_CHECK(hits.size() == 0);
}

MINI_TEST("Intersection", "Ray Mesh Bvh") {
    // uncomment #include "intersection.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    std::vector<std::vector<Point>> polygons = {
        {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        },
        {
            Point(0.0, 0.0, 1.0),
            Point(1.0, 0.0, 1.0),
            Point(1.0, 1.0, 1.0),
            Point(0.0, 1.0, 1.0),
        },
    };

    Mesh mesh = Mesh::from_polylines(polygons);

    Point origin(0.5, 0.5, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh_bvh(origin, direction, mesh, hits, true);

    MINI_CHECK(result);
    MINI_CHECK(hits.size() >= 1);
    MINI_CHECK(std::fabs(hits[0].t - 1.0) < 1e-3);
}

MINI_TEST("Intersection", "Ray Mesh Bvh First") {
    // uncomment #include "intersection.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    std::vector<std::vector<Point>> polygons = {
        {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        },
        {
            Point(0.0, 0.0, 1.0),
            Point(1.0, 0.0, 1.0),
            Point(1.0, 1.0, 1.0),
            Point(0.0, 1.0, 1.0),
        },
    };

    Mesh mesh = Mesh::from_polylines(polygons);

    Point origin(0.5, 0.5, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh_bvh(origin, direction, mesh, hits, false);

    MINI_CHECK(result);
    MINI_CHECK(hits.size() == 1);
}

MINI_TEST("Intersection", "Ray Mesh Bvh Miss") {
    // uncomment #include "intersection.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    std::vector<std::vector<Point>> polygons = {
        {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
        },
    };

    Mesh mesh = Mesh::from_polylines(polygons);

    Point origin(5.0, 5.0, -1.0);
    Vector direction(0.0, 0.0, 1.0);

    std::vector<Intersection::RayHit> hits;
    bool result = Intersection::ray_mesh_bvh(origin, direction, mesh, hits, true);

    MINI_CHECK(!result);
    MINI_CHECK(hits.size() == 0);
}

MINI_TEST("Intersection", "Ray Mesh Bvh Vs Naive") {
    // uncomment #include "intersection.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
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

    Mesh mesh = Mesh::from_polylines(polygons);

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

MINI_TEST("Intersection", "Ray Box Real World") {
    // uncomment #include "intersection.h"
    // uncomment #include "line.h"
    // uncomment #include "obb.h"
    // uncomment #include "point.h"
    Line l0(500.0, -573.576, -819.152, 500.0, 573.576, 819.152);
    Point min(214.0, 192.0, 484.0);
    Point max(694.0, 567.0, 796.0);
    std::vector<Point> points {min, max};
    OBB box = OBB::from_points(points);

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

MINI_TEST("Intersection", "Ray Sphere Real World") {
    // uncomment #include "intersection.h"
    // uncomment #include "line.h"
    // uncomment #include "point.h"
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

MINI_TEST("Intersection", "Ray Triangle Real World") {
    // uncomment #include "intersection.h"
    // uncomment #include "line.h"
    // uncomment #include "point.h"
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

MINI_TEST("Intersection", "Surface Plane") {
    // uncomment #include "intersection.h"
    // uncomment #include "nurbssurface.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // Bilinear surface: z = x (0 to 10), cut at z=5
    std::vector<Point> pts = {
        Point(0, 0, 0),
        Point(0, 10, 0),
        Point(10, 0, 10),
        Point(10, 10, 10),
    };
    auto srf = NurbsSurface::create(false, false, 1, 1, 2, 2, pts);

    Point pp(0, 0, 5);
    Vector pn(0, 0, 1);
    Plane plane = Plane::from_point_normal(pp, pn);
    auto curves = Intersection::surface_plane(srf, plane);

    MINI_CHECK(curves.size() == 1);
    MINI_CHECK(curves[0].is_valid());

    // Evaluate curve — all points should lie on x≈5, z≈5
    auto [t0, t1] = curves[0].domain();
    for (int i = 0; i <= 10; i++) {
        double t = t0 + (t1 - t0) * i / 10.0;
        Point p = curves[0].point_at(t);
        MINI_CHECK(std::fabs(p[0] - 5.0) < 0.5);
        MINI_CHECK(std::fabs(p[2] - 5.0) < 0.5);
    }
}

MINI_TEST("Intersection", "Surface Plane Curved") {
    // uncomment #include "intersection.h"
    // uncomment #include "nurbssurface.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // Degree 3 surface with bump, cut at z=3
    std::vector<Point> pts;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double x = i * 10.0;
            double y = j * 10.0;
            double z = ((i == 1 || i == 2) && (j == 1 || j == 2)) ? 10.0 : 0.0;
            pts.push_back(Point(x, y, z));
        }
    }
    auto srf = NurbsSurface::create(false, false, 3, 3, 4, 4, pts);

    Point pp(0, 0, 3);
    Vector pn(0, 0, 1);
    Plane plane = Plane::from_point_normal(pp, pn);
    auto curves = Intersection::surface_plane(srf, plane);

    MINI_CHECK(curves.size() >= 1);
    MINI_CHECK(curves[0].is_valid());
    MINI_CHECK(curves[0].degree() == 3);

    // Evaluate — points should be near z=3
    auto [t0, t1] = curves[0].domain();
    for (int i = 0; i <= 10; i++) {
        double t = t0 + (t1 - t0) * i / 10.0;
        Point p = curves[0].point_at(t);
        MINI_CHECK(std::fabs(p[2] - 3.0) < 1.0);
    }
}

MINI_TEST("Intersection", "Surface Plane Miss") {
    // uncomment #include "intersection.h"
    // uncomment #include "nurbssurface.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // Flat surface at z=0, plane at z=5 -> no intersection
    std::vector<Point> pts = {
        Point(0, 0, 0),
        Point(0, 10, 0),
        Point(10, 0, 0),
        Point(10, 10, 0),
    };
    auto srf = NurbsSurface::create(false, false, 1, 1, 2, 2, pts);

    Point pp(0, 0, 5);
    Vector pn(0, 0, 1);
    Plane plane = Plane::from_point_normal(pp, pn);
    auto curves = Intersection::surface_plane(srf, plane);

    MINI_CHECK(curves.size() == 0);
}

MINI_TEST("Intersection", "Remap") {
    // uncomment #include "intersection.h"
    MINI_CHECK(std::fabs(Intersection::remap(5.0, 0.0, 10.0, 0.0, 1.0) - 0.5) < 1e-9);
    MINI_CHECK(std::fabs(Intersection::remap(0.0, 0.0, 10.0, 0.0, 1.0) - 0.0) < 1e-9);
    MINI_CHECK(std::fabs(Intersection::remap(10.0, 0.0, 10.0, 0.0, 1.0) - 1.0) < 1e-9);
}

MINI_TEST("Intersection", "Closest Point On Segment") {
    // uncomment #include "intersection.h"
    // uncomment #include "line.h"
    // uncomment #include "point.h"
    Line seg(0.0, 0.0, 0.0, 4.0, 0.0, 0.0);
    Point pt(2.0, 3.0, 0.0);
    Point cp;
    double t;
    bool result = Intersection::closest_point_on_segment(pt, seg, cp, t);

    MINI_CHECK(result);
    MINI_CHECK(std::fabs(cp[0] - 2.0) < 1e-9);
    MINI_CHECK(std::fabs(cp[1] - 0.0) < 1e-9);
    MINI_CHECK(std::fabs(t - 0.5) < 1e-9);

    Point pt2(-2.0, 1.0, 0.0);
    Point cp2;
    double t2;
    Intersection::closest_point_on_segment(pt2, seg, cp2, t2);
    MINI_CHECK(std::fabs(cp2[0] - 0.0) < 1e-9);
    MINI_CHECK(std::fabs(t2 - 0.0) < 1e-9);
}

MINI_TEST("Intersection", "Plane Plane Plane Check Parallel") {
    // uncomment #include "intersection.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    Point o0(0.0, 0.0, 0.0);
    Vector n0(0.0, 0.0, 1.0);
    Point o1(0.0, 0.0, 1.0);
    Vector n1(0.0, 0.0, 1.0);
    Point o2(0.0, 0.0, 2.0);
    Vector n2(0.0, 0.0, 1.0);
    Plane p0 = Plane::from_point_normal(o0, n0);
    Plane p1 = Plane::from_point_normal(o1, n1);
    Plane p2 = Plane::from_point_normal(o2, n2);
    Point out;

    MINI_CHECK(!Intersection::plane_plane_plane_check(p0, p1, p2, 0.1, out));

    Point px_o(1.0, 0.0, 0.0);
    Vector px_n(1.0, 0.0, 0.0);
    Point py_o(0.0, 2.0, 0.0);
    Vector py_n(0.0, 1.0, 0.0);
    Point pz_o(0.0, 0.0, 3.0);
    Vector pz_n(0.0, 0.0, 1.0);
    Plane px = Plane::from_point_normal(px_o, px_n);
    Plane py = Plane::from_point_normal(py_o, py_n);
    Plane pz = Plane::from_point_normal(pz_o, pz_n);
    Point pt;
    bool ok = Intersection::plane_plane_plane_check(px, py, pz, 0.1, pt);
    MINI_CHECK(ok);
    MINI_CHECK(std::fabs(pt[0] - 1.0) < 1e-6);
    MINI_CHECK(std::fabs(pt[1] - 2.0) < 1e-6);
    MINI_CHECK(std::fabs(pt[2] - 3.0) < 1e-6);
}

MINI_TEST("Intersection", "Plane 4 Planes Closed") {
    // uncomment #include "intersection.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "polyline.h"
    // uncomment #include "vector.h"
    Point mp(0.0, 0.0, 0.0);
    Vector mn(0.0, 0.0, 1.0);
    Plane main_plane = Plane::from_point_normal(mp, mn);
    Point o0(-1.0, 0.0, 0.0); Vector n0(1.0, 0.0, 0.0);
    Point o1(0.0, -1.0, 0.0); Vector n1(0.0, 1.0, 0.0);
    Point o2(1.0,  0.0, 0.0); Vector n2(1.0, 0.0, 0.0);
    Point o3(0.0,  1.0, 0.0); Vector n3(0.0, 1.0, 0.0);
    std::array<Plane, 4> planes = {
        Plane::from_point_normal(o0, n0),
        Plane::from_point_normal(o1, n1),
        Plane::from_point_normal(o2, n2),
        Plane::from_point_normal(o3, n3),
    };
    Polyline result;
    bool ok = Intersection::plane_4planes(main_plane, planes, result);

    MINI_CHECK(ok);
    MINI_CHECK(result.point_count() == 5);
    for (size_t i = 0; i < result.point_count(); i++) {
        Point p = result.get_point(i);
        MINI_CHECK(std::fabs(p[2]) < 1e-6);
    }
    Point first = result.get_point(0);
    Point last = result.get_point(4);
    MINI_CHECK(std::fabs(first[0] - last[0]) < 1e-6);
    MINI_CHECK(std::fabs(first[1] - last[1]) < 1e-6);
}

MINI_TEST("Intersection", "Plane 4 Planes Open") {
    // uncomment #include "intersection.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "polyline.h"
    // uncomment #include "vector.h"
    Point mp(0.0, 0.0, 0.0);
    Vector mn(0.0, 0.0, 1.0);
    Plane main_plane = Plane::from_point_normal(mp, mn);
    Point o0(-1.0, 0.0, 0.0); Vector n0(1.0, 0.0, 0.0);
    Point o1(0.0, -1.0, 0.0); Vector n1(0.0, 1.0, 0.0);
    Point o2(1.0,  0.0, 0.0); Vector n2(1.0, 0.0, 0.0);
    Point o3(0.0,  1.0, 0.0); Vector n3(0.0, 1.0, 0.0);
    std::array<Plane, 4> planes = {
        Plane::from_point_normal(o0, n0),
        Plane::from_point_normal(o1, n1),
        Plane::from_point_normal(o2, n2),
        Plane::from_point_normal(o3, n3),
    };
    Polyline result;
    bool ok = Intersection::plane_4planes_open(main_plane, planes, result);

    MINI_CHECK(ok);
    MINI_CHECK(result.point_count() == 4);
}

MINI_TEST("Intersection", "Plane 4 Lines") {
    // uncomment #include "intersection.h"
    // uncomment #include "line.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "polyline.h"
    // uncomment #include "vector.h"
    Point pp(0.0, 0.0, 0.0);
    Vector pn(0.0, 0.0, 1.0);
    Plane plane = Plane::from_point_normal(pp, pn);
    Line l0(-1.0, -1.0, -1.0, -1.0,  1.0, 1.0);
    Line l1( 1.0, -1.0, -1.0,  1.0,  1.0, 1.0);
    Line l2(-1.0, -1.0, -1.0,  1.0, -1.0, 1.0);
    Line l3(-1.0,  1.0, -1.0,  1.0,  1.0, 1.0);
    Polyline result;
    bool ok = Intersection::plane_4lines(plane, l0, l1, l2, l3, result);

    MINI_CHECK(ok);
    MINI_CHECK(result.point_count() == 5);
    for (size_t i = 0; i < result.point_count(); i++) {
        Point p = result.get_point(i);
        MINI_CHECK(std::fabs(p[2]) < 1e-6);
    }
}

MINI_TEST("Intersection", "Scale Vector To Distance Of 2 Planes") {
    // uncomment #include "intersection.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    Point p0o(0.0, 0.0, 0.0);
    Vector p0n(0.0, 0.0, 1.0);
    Point p1o(0.0, 0.0, 3.0);
    Vector p1n(0.0, 0.0, 1.0);
    Plane pl0 = Plane::from_point_normal(p0o, p0n);
    Plane pl1 = Plane::from_point_normal(p1o, p1n);
    Vector direction(0.0, 0.0, 1.0);
    Vector result;
    bool ok = Intersection::scale_vector_to_distance_of_2planes(direction, pl0, pl1, result);

    MINI_CHECK(ok);
    MINI_CHECK(std::fabs(result[2] - 3.0) < 1e-6);
}

MINI_TEST("Intersection", "Polyline Plane") {
    // uncomment #include "intersection.h"
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "polyline.h"
    // uncomment #include "vector.h"
    std::vector<Point> pts = {
        Point(-1.0, -1.0, 0.0),
        Point( 1.0, -1.0, 0.0),
        Point( 1.0,  1.0, 0.0),
        Point(-1.0,  1.0, 0.0),
        Point(-1.0, -1.0, 0.0),
    };
    Polyline poly(pts);
    Point pp(0.0, 0.0, 0.0);
    Vector pn(1.0, 0.0, 0.0);
    Plane plane = Plane::from_point_normal(pp, pn);
    std::vector<Point> points;
    std::vector<int> edge_ids;
    bool result = Intersection::polyline_plane(poly, plane, points, edge_ids);

    MINI_CHECK(result);
    MINI_CHECK(points.size() == 2);
    for (const auto& p : points) {
        MINI_CHECK(std::fabs(p[0]) < 1e-9);
    }
}

MINI_TEST("Intersection", "Line Line 3D") {
    // uncomment #include "intersection.h"
    // uncomment #include "line.h"
    // uncomment #include "point.h"
    Line cutter(0.0, 1.0, 0.0, 2.0, 1.0, 0.0);
    Line seg(1.0, 0.0, 0.0, 1.0, 2.0, 0.0);
    Point result;
    bool ok = Intersection::line_line_3d(cutter, seg, result);

    MINI_CHECK(ok);
    MINI_CHECK(std::fabs(result[0] - 1.0) < 1e-6);
    MINI_CHECK(std::fabs(result[1] - 1.0) < 1e-6);
    MINI_CHECK(std::fabs(result[2] - 0.0) < 1e-6);

    Line par0(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line par1(0.0, 1.0, 0.0, 1.0, 1.0, 0.0);
    Point out2;
    MINI_CHECK(!Intersection::line_line_3d(par0, par1, out2));
}

} // namespace session_cpp
