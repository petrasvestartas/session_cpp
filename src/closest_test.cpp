#include "mini_test.h"
#include "closest.h"
#include "line.h"
#include "polyline.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "mesh.h"
#include "pointcloud.h"
#include "primitives.h"
#include "point.h"
#include "tolerance.h"

#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("Closest", "Line Point") {
    Line l(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);

    auto [cp1, t1, d1] = Closest::line_point(l, Point(5.0, 5.0, 0.0));

    MINI_CHECK(TOLERANCE.is_close(cp1[0], 5.0));
    MINI_CHECK(TOLERANCE.is_close(cp1[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(t1, 0.5));
    MINI_CHECK(TOLERANCE.is_close(d1, 5.0));

    auto [cp2, t2, d2] = Closest::line_point(l, Point(-5.0, 0.0, 0.0));
    MINI_CHECK(TOLERANCE.is_close(cp2[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(t2, 0.0));
    MINI_CHECK(TOLERANCE.is_close(d2, 5.0));

    auto [cp3, t3, d3] = Closest::line_point(l, Point(15.0, 0.0, 0.0));
    MINI_CHECK(TOLERANCE.is_close(cp3[0], 10.0));
    MINI_CHECK(TOLERANCE.is_close(t3, 1.0));
    MINI_CHECK(TOLERANCE.is_close(d3, 5.0));
}

MINI_TEST("Closest", "Polyline Point") {
    Polyline pl({
        Point(0.0, 0.0, 0.0),
        Point(10.0, 0.0, 0.0),
        Point(10.0, 10.0, 0.0),
    });

    auto [cp1, t1, d1] = Closest::polyline_point(pl, Point(5.0, 5.0, 0.0));

    MINI_CHECK(TOLERANCE.is_close(d1, 5.0));

    auto [cp2, t2, d2] = Closest::polyline_point(pl, Point(10.0, 5.0, 0.0));
    MINI_CHECK(TOLERANCE.is_close(cp2[0], 10.0));
    MINI_CHECK(TOLERANCE.is_close(cp2[1], 5.0));
    MINI_CHECK(TOLERANCE.is_close(d2, 0.0));
}

MINI_TEST("Closest", "Curve Point") {
    std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 2.0, 0.0),
        Point(3.0, 2.0, 0.0),
        Point(4.0, 0.0, 0.0),
    };
    NurbsCurve crv = NurbsCurve::create(false, 3, pts);

    auto [t, dist] = Closest::curve_point(crv, Point(2.0, 3.0, 0.0));

    MINI_CHECK(dist < 1.6);
    Point cp = crv.point_at(t);
    MINI_CHECK(TOLERANCE.is_close(cp.distance(Point(2.0, 3.0, 0.0)), dist));

    auto [t2, dist2] = Closest::curve_point(crv, Point(0.0, 0.0, 0.0));
    MINI_CHECK(dist2 < 0.01);
}

MINI_TEST("Closest", "Surface Point") {
    std::vector<Point> pts = {
        // i=0
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(3.0, 0.0, 0.0),
        // i=1
        Point(0.0, 1.0, 0.0),
        Point(1.0, 1.0, 1.0),
        Point(2.0, 1.0, 1.0),
        Point(3.0, 1.0, 0.0),
        // i=2
        Point(0.0, 2.0, 0.0),
        Point(1.0, 2.0, 1.0),
        Point(2.0, 2.0, 1.0),
        Point(3.0, 2.0, 0.0),
        // i=3
        Point(0.0, 3.0, 0.0),
        Point(1.0, 3.0, 0.0),
        Point(2.0, 3.0, 0.0),
        Point(3.0, 3.0, 0.0),
    };
    NurbsSurface srf = NurbsSurface::create(false, false, 3, 3, 4, 4, pts);

    auto [u, v, dist] = Closest::surface_point(srf, Point(1.5, 1.5, 2.0));

    MINI_CHECK(dist < 1.5);
    Point cp = srf.point_at(u, v);
    MINI_CHECK(TOLERANCE.is_close(cp.distance(Point(1.5, 1.5, 2.0)), dist));

    auto [u2, v2, dist2] = Closest::surface_point(srf, Point(0.0, 0.0, 0.0));
    MINI_CHECK(dist2 < 0.01);
}

MINI_TEST("Closest", "Mesh Point") {
    Mesh m = Primitives::cube(2.0);

    auto [cp1, fk1, d1] = Closest::mesh_point(m, Point(0.0, 0.0, 2.0));

    MINI_CHECK(TOLERANCE.is_close(cp1[2], 1.0));
    MINI_CHECK(TOLERANCE.is_close(d1, 1.0));

    auto [cp2, fk2, d2] = Closest::mesh_point(m, Point(1.0, 1.0, 1.0));
    MINI_CHECK(TOLERANCE.is_close(d2, 0.0));
}

MINI_TEST("Closest", "Mesh Point AABB") {
    Mesh m = Primitives::cube(2.0);

    auto [cp1, fk1, d1] = Closest::mesh_point_aabb(m, Point(0.0, 0.0, 2.0));

    MINI_CHECK(TOLERANCE.is_close(cp1[2], 1.0));
    MINI_CHECK(TOLERANCE.is_close(d1, 1.0));

    auto [cp2, fk2, d2] = Closest::mesh_point_aabb(m, Point(1.0, 1.0, 1.0));
    MINI_CHECK(TOLERANCE.is_close(d2, 0.0));
}

MINI_TEST("Closest", "Pointcloud Point") {
    PointCloud pc({
        Point(0.0, 0.0, 0.0),
        Point(5.0, 0.0, 0.0),
        Point(10.0, 0.0, 0.0),
        Point(10.0, 10.0, 0.0),
    }, {}, {});

    auto [cp1, i1, d1] = Closest::pointcloud_point(pc, Point(4.0, 0.0, 0.0));

    MINI_CHECK(TOLERANCE.is_close(cp1[0], 5.0));
    MINI_CHECK(i1 == 1);
    MINI_CHECK(TOLERANCE.is_close(d1, 1.0));

    auto [cp2, i2, d2] = Closest::pointcloud_point(pc, Point(10.0, 10.0, 0.0));
    MINI_CHECK(TOLERANCE.is_close(d2, 0.0));
    MINI_CHECK(i2 == 3);
}

}
