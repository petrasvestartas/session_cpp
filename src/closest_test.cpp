#include "mini_test.h"
#include "aabb.h"
#include "closest.h"
#include "line.h"
#include "mesh.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "point.h"
#include "pointcloud.h"
#include "polyline.h"
#include "primitives.h"
#include "tolerance.h"
#include <cmath>
#include <tuple>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("Closest", "Curve Point") {
    // using session_cpp::Closest;
    // using session_cpp::NurbsCurve;
    // using session_cpp::Point;

    const std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 2.0, 0.0),
        Point(3.0, 2.0, 0.0),
        Point(4.0, 0.0, 0.0),
    };
    const NurbsCurve crv = NurbsCurve::create(false, 3, pts);

    double t = 0.0;
    double dist = 0.0;
    std::tie(t, dist) = Closest::curve_point(crv, Point(2.0, 3.0, 0.0));

    MINI_CHECK(dist < 1.6);

    const Point cp = crv.point_at(t);

    MINI_CHECK(TOLERANCE.is_close(cp.distance(Point(2.0, 3.0, 0.0)), dist));

    const double dist2 = Closest::curve_point(crv, Point(0.0, 0.0, 0.0)).second;

    MINI_CHECK(dist2 < 0.01);
}

MINI_TEST("Closest", "Curve Curve") {
    // using session_cpp::Closest;
    // using session_cpp::NurbsCurve;
    // using session_cpp::Point;

    const NurbsCurve curve0 = NurbsCurve::create(false, 1, {Point(0.0, 0.0, 0.0), Point(10.0, 0.0, 0.0)});
    const NurbsCurve curve1 = NurbsCurve::create(false, 1, {Point(5.0, -5.0, 1.0), Point(5.0, 5.0, 1.0)});

    double u = 0.0;
    double v = 0.0;
    double dist = 0.0;
    std::tie(u, v, dist) = Closest::curve_curve(curve0, curve1);
    const Point p0 = curve0.point_at(u);
    const Point p1 = curve1.point_at(v);

    MINI_CHECK(TOLERANCE.is_close(dist, 1.0));
    MINI_CHECK(TOLERANCE.is_close(p0[0], 5.0));
    MINI_CHECK(TOLERANCE.is_close(p1[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(p0.distance(p1), dist));
}

MINI_TEST("Closest", "Line Point") {
    // using session_cpp::Closest;
    // using session_cpp::Line;
    // using session_cpp::Point;

    const Line line(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);

    Point cp1;
    double t1 = 0.0;
    double d1 = 0.0;
    std::tie(cp1, t1, d1) = Closest::line_point(line, Point(5.0, 5.0, 0.0));

    MINI_CHECK(TOLERANCE.is_close(cp1[0], 5.0));
    MINI_CHECK(TOLERANCE.is_close(cp1[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(t1, 0.5));
    MINI_CHECK(TOLERANCE.is_close(d1, 5.0));

    Point cp2;
    double t2 = 0.0;
    double d2 = 0.0;
    std::tie(cp2, t2, d2) = Closest::line_point(line, Point(-5.0, 0.0, 0.0));

    MINI_CHECK(TOLERANCE.is_close(cp2[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(t2, 0.0));
    MINI_CHECK(TOLERANCE.is_close(d2, 5.0));

    Point cp3;
    double t3 = 0.0;
    double d3 = 0.0;
    std::tie(cp3, t3, d3) = Closest::line_point(line, Point(15.0, 0.0, 0.0));

    MINI_CHECK(TOLERANCE.is_close(cp3[0], 10.0));
    MINI_CHECK(TOLERANCE.is_close(t3, 1.0));
    MINI_CHECK(TOLERANCE.is_close(d3, 5.0));
}

MINI_TEST("Closest", "Polyline Point") {
    // using session_cpp::Closest;
    // using session_cpp::Polyline;
    // using session_cpp::Point;

    const Polyline pl({
        Point(0.0, 0.0, 0.0),
        Point(10.0, 0.0, 0.0),
        Point(10.0, 10.0, 0.0),
    });

    const double d1 = std::get<2>(Closest::polyline_point(pl, Point(5.0, 5.0, 0.0)));

    MINI_CHECK(TOLERANCE.is_close(d1, 5.0));

    Point cp2;
    double d2 = 0.0;
    std::tie(cp2, std::ignore, d2) = Closest::polyline_point(pl, Point(10.0, 5.0, 0.0));

    MINI_CHECK(TOLERANCE.is_close(cp2[0], 10.0));
    MINI_CHECK(TOLERANCE.is_close(cp2[1], 5.0));
    MINI_CHECK(TOLERANCE.is_close(d2, 0.0));
}

MINI_TEST("Closest", "Surface Point") {
    // using session_cpp::Closest;
    // using session_cpp::NurbsSurface;
    // using session_cpp::Point;

    const std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(3.0, 0.0, 0.0),
        Point(0.0, 1.0, 0.0),
        Point(1.0, 1.0, 1.0),
        Point(2.0, 1.0, 1.0),
        Point(3.0, 1.0, 0.0),
        Point(0.0, 2.0, 0.0),
        Point(1.0, 2.0, 1.0),
        Point(2.0, 2.0, 1.0),
        Point(3.0, 2.0, 0.0),
        Point(0.0, 3.0, 0.0),
        Point(1.0, 3.0, 0.0),
        Point(2.0, 3.0, 0.0),
        Point(3.0, 3.0, 0.0),
    };
    const NurbsSurface srf = NurbsSurface::create(false, false, 3, 3, 4, 4, pts);

    double u = 0.0;
    double v = 0.0;
    double dist = 0.0;
    std::tie(u, v, dist) = Closest::surface_point(srf, Point(1.5, 1.5, 2.0));

    MINI_CHECK(dist < 1.5);

    const Point cp = srf.point_at(u, v);

    MINI_CHECK(TOLERANCE.is_close(cp.distance(Point(1.5, 1.5, 2.0)), dist));

    const double dist2 = std::get<2>(Closest::surface_point(srf, Point(0.0, 0.0, 0.0)));

    MINI_CHECK(dist2 < 0.01);
}

MINI_TEST("Closest", "Surface Curve") {
    // using session_cpp::Closest;
    // using session_cpp::NurbsCurve;
    // using session_cpp::Point;
    // using session_cpp::Primitives;
    // using session_cpp::NurbsSurface;

    const NurbsSurface cyl = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 4.0);
    double u0 = 0.0;
    double u1 = 0.0;
    double v0 = 0.0;
    double v1 = 0.0;
    std::tie(u0, u1) = cyl.domain(0);
    std::tie(v0, v1) = cyl.domain(1);
    const Point ps = cyl.point_at(u0, 0.5);
    const double seam_ang = std::atan2(ps[1], ps[0]);
    std::vector<Point> crv_pts;

    for (int i = 0; i < 21; i++) {
        const double a = seam_ang - 0.8 + 1.6 * i / 20.0;
        const double z = 1.0 + 2.0 * i / 20.0;
        crv_pts.push_back(Point(std::cos(a), std::sin(a), z));
    }

    const NurbsCurve crv = NurbsCurve::create_interpolated(crv_pts);

    const std::vector<NurbsCurve> pcurves = Closest::surface_curve(cyl, crv);

    MINI_CHECK(pcurves.size() == 2);

    int on_border = 0;
    bool inside = true;

    for (const NurbsCurve& pcurve : pcurves) {
        MINI_CHECK(pcurve.is_valid());

        for (double e : {0.0, 1.0}) {
            const Point p2 = pcurve.point_at(e);

            if (std::abs(p2[0] - u0) < 1e-9 || std::abs(p2[0] - u1) < 1e-9)
                on_border += 1;
        }

        for (int i = 0; i < 17; i++) {
            const Point p2 = pcurve.point_at(i / 16.0);

            if (p2[0] < u0 - 1e-6 || p2[0] > u1 + 1e-6 || p2[1] < v0 - 1e-6 || p2[1] > v1 + 1e-6)
                inside = false;
        }
    }

    MINI_CHECK(on_border == 2);
    MINI_CHECK(inside);

    const NurbsCurve off = NurbsCurve::create(false, 1, {Point(20.0, 20.0, 20.0), Point(30.0, 30.0, 30.0)});

    MINI_CHECK(Closest::surface_curve(cyl, off).size() == 0);
}

MINI_TEST("Closest", "Mesh Point") {
    // using session_cpp::Closest;
    // using session_cpp::Primitives;
    // using session_cpp::Point;
    // using session_cpp::Mesh;

    const Mesh m = Primitives::cube(2.0);

    Point cp1;
    double d1 = 0.0;
    std::tie(cp1, std::ignore, d1) = Closest::mesh_point(m, Point(0.0, 0.0, 2.0));

    MINI_CHECK(TOLERANCE.is_close(cp1[2], 1.0));
    MINI_CHECK(TOLERANCE.is_close(d1, 1.0));

    const double d2 = std::get<2>(Closest::mesh_point(m, Point(1.0, 1.0, 1.0)));

    MINI_CHECK(TOLERANCE.is_close(d2, 0.0));
}

MINI_TEST("Closest", "Mesh Point AABB") {
    // using session_cpp::Closest;
    // using session_cpp::Primitives;
    // using session_cpp::Point;
    // using session_cpp::Mesh;

    const Mesh m = Primitives::cube(2.0);

    Point cp1;
    double d1 = 0.0;
    std::tie(cp1, std::ignore, d1) = Closest::mesh_point_aabb(m, Point(0.0, 0.0, 2.0));

    MINI_CHECK(TOLERANCE.is_close(cp1[2], 1.0));
    MINI_CHECK(TOLERANCE.is_close(d1, 1.0));

    const double d2 = std::get<2>(Closest::mesh_point_aabb(m, Point(1.0, 1.0, 1.0)));

    MINI_CHECK(TOLERANCE.is_close(d2, 0.0));
}

MINI_TEST("Closest", "Pointcloud Point") {
    // using session_cpp::Closest;
    // using session_cpp::PointCloud;
    // using session_cpp::Point;

    const PointCloud pc(
        {
            Point(0.0, 0.0, 0.0),
            Point(5.0, 0.0, 0.0),
            Point(10.0, 0.0, 0.0),
            Point(10.0, 10.0, 0.0),
        },
        {},
        {}
    );

    Point cp1;
    size_t i1 = 0;
    double d1 = 0.0;
    std::tie(cp1, i1, d1) = Closest::pointcloud_point(pc, Point(4.0, 0.0, 0.0));

    MINI_CHECK(TOLERANCE.is_close(cp1[0], 5.0));
    MINI_CHECK(i1 == 1);
    MINI_CHECK(TOLERANCE.is_close(d1, 1.0));

    size_t i2 = 0;
    double d2 = 0.0;
    std::tie(std::ignore, i2, d2) = Closest::pointcloud_point(pc, Point(10.0, 10.0, 0.0));

    MINI_CHECK(TOLERANCE.is_close(d2, 0.0));
    MINI_CHECK(i2 == 3);
}

MINI_TEST("Closest", "Pointcloud Point SpatialKDTree") {
    // using session_cpp::Closest;
    // using session_cpp::PointCloud;
    // using session_cpp::Point;

    const PointCloud pc(
        {
            Point(0.0, 0.0, 0.0),
            Point(5.0, 0.0, 0.0),
            Point(10.0, 0.0, 0.0),
            Point(10.0, 10.0, 0.0),
        },
        {},
        {}
    );

    Point cp1;
    size_t i1 = 0;
    double d1 = 0.0;
    std::tie(cp1, i1, d1) = Closest::pointcloud_point_kdtree(pc, Point(4.0, 0.0, 0.0));

    MINI_CHECK(TOLERANCE.is_close(cp1[0], 5.0));
    MINI_CHECK(i1 == 1);
    MINI_CHECK(TOLERANCE.is_close(d1, 1.0));

    size_t i2 = 0;
    double d2 = 0.0;
    std::tie(std::ignore, i2, d2) = Closest::pointcloud_point_kdtree(pc, Point(10.0, 10.0, 0.0));

    MINI_CHECK(TOLERANCE.is_close(d2, 0.0));
    MINI_CHECK(i2 == 3);
}

MINI_TEST("Closest", "Lines Closest") {
    // using session_cpp::Closest;
    // using session_cpp::Line;

    const std::vector<Line> lines = {
        Line(0.0, 0.0, 0.0, 5.0, 0.0, 0.0),
        Line(5.0, 0.0, 0.0, 10.0, 0.0, 0.0),
        Line(100.0, 0.0, 0.0, 110.0, 0.0, 0.0),
    };

    const std::vector<std::pair<size_t, size_t>> pairs = Closest::lines_closest(lines, 0.01);

    MINI_CHECK(pairs.size() == 1);
    MINI_CHECK(pairs[0].first == 0);
    MINI_CHECK(pairs[0].second == 1);
}

MINI_TEST("Closest", "Polylines Closest") {
    // using session_cpp::Closest;
    // using session_cpp::Polyline;
    // using session_cpp::Point;

    const std::vector<Polyline> pls = {
        Polyline({Point(0.0, 0.0, 0.0), Point(5.0, 0.0, 0.0)}),
        Polyline({Point(5.0, 0.0, 0.0), Point(10.0, 0.0, 0.0)}),
        Polyline({Point(100.0, 0.0, 0.0), Point(110.0, 0.0, 0.0)}),
    };

    const std::vector<std::pair<size_t, size_t>> pairs = Closest::polylines_closest(pls, 0.01);

    MINI_CHECK(pairs.size() == 1);
    MINI_CHECK(pairs[0].first == 0);
    MINI_CHECK(pairs[0].second == 1);
}

MINI_TEST("Closest", "Nurbscurves Closest") {
    // using session_cpp::Closest;
    // using session_cpp::NurbsCurve;
    // using session_cpp::Point;

    const std::vector<NurbsCurve> curves = {
        NurbsCurve::create(false, 1, {Point(0.0, 0.0, 0.0), Point(5.0, 0.0, 0.0)}),
        NurbsCurve::create(false, 1, {Point(5.0, 0.0, 0.0), Point(10.0, 0.0, 0.0)}),
        NurbsCurve::create(false, 1, {Point(100.0, 0.0, 0.0), Point(110.0, 0.0, 0.0)}),
    };

    const std::vector<std::pair<size_t, size_t>> pairs = Closest::nurbscurves_closest(curves, 0.01);

    MINI_CHECK(pairs.size() == 1);
    MINI_CHECK(pairs[0].first == 0);
    MINI_CHECK(pairs[0].second == 1);
}

MINI_TEST("Closest", "Boxes Closest") {
    // using session_cpp::AABB;
    // using session_cpp::Closest;

    const std::vector<AABB> boxes = {
        AABB(0.0, 0.0, 0.0, 1.0, 1.0, 1.0),
        AABB(2.0, 0.0, 0.0, 1.0, 1.0, 1.0),
        AABB(20.0, 0.0, 0.0, 1.0, 1.0, 1.0),
    };

    const std::vector<std::pair<size_t, size_t>> pairs = Closest::boxes_closest(boxes, 0.01);

    MINI_CHECK(pairs.size() == 1);
    MINI_CHECK(pairs[0].first == 0);
    MINI_CHECK(pairs[0].second == 1);
    MINI_CHECK(Closest::boxes_closest(boxes, -0.01).empty());
}

} // namespace session_cpp
