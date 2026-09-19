#include "mini_test.h"
#include "aabb.h"
#include "color.h"
#include "line.h"
#include "mesh.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "point.h"
#include "pointcloud.h"
#include "polyline.h"
#include "primitives.h"
#include "tolerance.h"
#include "vector.h"
#include "xform.h"
#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("AABB", "Constructor") {

    AABB a(0.0, 0.0, 0.0, 1.0, 2.0, 3.0);
    AABB empty;

    MINI_CHECK(empty == AABB(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    MINI_CHECK(a == AABB(0.0, 0.0, 0.0, 1.0, 2.0, 3.0));
    MINI_CHECK(a != empty);
    MINI_CHECK(a.str() == "0.000000, 0.000000, 0.000000, 1.000000, 2.000000, 3.000000");
    MINI_CHECK(a.repr() == "AABB(0.000000, 0.000000, 0.000000, 1.000000, 2.000000, 3.000000)");
    MINI_CHECK(TOLERANCE.is_close(a.area(), 88.0));
    MINI_CHECK(a.center() == Point(0.0, 0.0, 0.0));
    MINI_CHECK(TOLERANCE.is_close(a.diagonal(), 2.0 * std::sqrt(14.0)));
    MINI_CHECK(a.is_valid());
    MINI_CHECK(TOLERANCE.is_close(a.volume(), 48.0));

    MINI_CHECK(a.closest_point(Point(0.0, 0.0, 0.0)) == Point(0.0, 0.0, 0.0));
    MINI_CHECK(a.closest_point(Point(10.0, 0.0, 0.0)) == Point(1.0, 0.0, 0.0));
    MINI_CHECK(a.contains(Point(0.0, 0.0, 0.0)));
    MINI_CHECK(!a.contains(Point(10.0, 0.0, 0.0)));

    MINI_CHECK(a.corner(false, false, false) == Point(-1.0, -2.0, -3.0));
    MINI_CHECK(a.corner(true, true, true) == Point(1.0, 2.0, 3.0));
    MINI_CHECK(a.get_corners().size() == 8);
    MINI_CHECK(a.get_edges().size() == 12);

    MINI_CHECK(a.point_at(1.0, 0.0, 0.0) == Point(1.0, 0.0, 0.0));
    MINI_CHECK(a.point_at(0.0, 0.0, 0.0) == Point(0.0, 0.0, 0.0));

    MINI_CHECK(a.intersects(AABB(0.5, 0.0, 0.0, 0.5, 0.5, 0.5)));
    MINI_CHECK(!a.intersects(AABB(10.0, 0.0, 0.0, 0.5, 0.5, 0.5)));
    AABB b(5.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    a.union_with(b);
    MINI_CHECK(a.min_point() == Point(-1.0, -2.0, -3.0));
    MINI_CHECK(a.max_point() == Point(6.0, 2.0, 3.0));
    AABB c = AABB::merge(AABB(0.0, 0.0, 0.0, 1.0, 1.0, 1.0), AABB(4.0, 0.0, 0.0, 1.0, 1.0, 1.0));
    MINI_CHECK(c.min_point() == Point(-1.0, -1.0, -1.0));
    MINI_CHECK(c.max_point() == Point(5.0, 1.0, 1.0));
}

MINI_TEST("AABB", "Empty") {

    AABB a = AABB::empty();
    MINI_CHECK(!a.is_valid());
    MINI_CHECK(TOLERANCE.is_close(a.diagonal(), 0.0));
    a.union_with(AABB::empty());
    MINI_CHECK(!a.is_valid());
    a.union_with_point(1.0, 2.0, 3.0);
    MINI_CHECK(a.is_valid());
    MINI_CHECK(a.min_point() == Point(1.0, 2.0, 3.0));
    MINI_CHECK(a.max_point() == Point(1.0, 2.0, 3.0));
    a.union_with_point(-1.0, 0.0, 5.0);
    MINI_CHECK(a.min_point() == Point(-1.0, 0.0, 3.0));
    MINI_CHECK(a.max_point() == Point(1.0, 2.0, 5.0));
    a.union_with(AABB::empty());
    MINI_CHECK(a.max_point() == Point(1.0, 2.0, 5.0));
    AABB b = AABB::merge(AABB::empty(), AABB(4.0, 0.0, 0.0, 1.0, 1.0, 1.0));
    MINI_CHECK(b.min_point() == Point(3.0, -1.0, -1.0));
    MINI_CHECK(b.max_point() == Point(5.0, 1.0, 1.0));
}

MINI_TEST("AABB", "Transform") {

    AABB a(0.0, 0.0, 0.0, 1.0, 2.0, 3.0);
    AABB moved = a.transformed(Xform::translation(1.0, 2.0, 3.0));
    MINI_CHECK(moved.min_point() == Point(0.0, 0.0, 0.0));
    MINI_CHECK(moved.max_point() == Point(2.0, 4.0, 6.0));
    AABB turned = a.transformed(Xform::rotation_z(90.0, true));
    MINI_CHECK(turned.min_point() == Point(-2.0, -1.0, -3.0));
    MINI_CHECK(turned.max_point() == Point(2.0, 1.0, 3.0));
    a.transform(Xform::scale_xyz(2.0, 2.0, 2.0));
    MINI_CHECK(a.max_point() == Point(2.0, 4.0, 6.0));
    MINI_CHECK(!AABB::empty().transformed(Xform::translation(1.0, 0.0, 0.0)).is_valid());
}

MINI_TEST("AABB", "From Geometry") {

    AABB a_pt = AABB::from_point(Point(1.0, 2.0, 3.0), 0.5);

    MINI_CHECK(a_pt.center() == Point(1.0, 2.0, 3.0));
    MINI_CHECK(TOLERANCE.is_close(a_pt.hx, 0.5));

    AABB a_pts = AABB::from_points({
        Point(0.0, 0.0, 0.0),
        Point(3.0, 4.0, 5.0),
    }, 0.0);

    MINI_CHECK(a_pts.min_point() == Point(0.0, 0.0, 0.0));
    MINI_CHECK(a_pts.max_point() == Point(3.0, 4.0, 5.0));

    AABB a_negative = AABB::from_points({Point(-5.0, -4.0, -3.0), Point(-1.0, -2.0, -1.0)});
    MINI_CHECK(a_negative.min_point() == Point(-5.0, -4.0, -3.0));
    MINI_CHECK(a_negative.max_point() == Point(-1.0, -2.0, -1.0));

    Line ln(0.0, 0.0, 0.0, 4.0, 0.0, 0.0);
    AABB a_line = AABB::from_line(ln, 1.0);

    MINI_CHECK(a_line.min_point() == Point(-1.0, -1.0, -1.0));
    MINI_CHECK(a_line.max_point() == Point(5.0, 1.0, 1.0));

    Polyline pl({
        Point(0.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(2.0, 2.0, 0.0),
    });
    AABB a_pl = AABB::from_polyline(pl, 0.0);

    MINI_CHECK(a_pl.min_point() == Point(0.0, 0.0, 0.0));
    MINI_CHECK(a_pl.max_point() == Point(2.0, 2.0, 0.0));

    Mesh cube = Primitives::cube(2.0);
    AABB a_mesh = AABB::from_mesh(cube, 0.0);

    MINI_CHECK(a_mesh.min_point() == Point(-1.0, -1.0, -1.0));
    MINI_CHECK(a_mesh.max_point() == Point(1.0, 1.0, 1.0));

    PointCloud pc(
        {
            Point(0.0, 0.0, 0.0),
            Point(4.0, 2.0, 6.0),
        },
        {
            Vector(0.0, 0.0, 1.0),
            Vector(0.0, 0.0, 1.0),
        },
        {
            Color(255, 0, 0, 255),
            Color(0, 255, 0, 255),
        }
    );
    AABB a_pc = AABB::from_pointcloud(pc, 0.0);

    MINI_CHECK(a_pc.min_point() == Point(0.0, 0.0, 0.0));
    MINI_CHECK(a_pc.max_point() == Point(4.0, 2.0, 6.0));

    NurbsCurve curve = NurbsCurve::create(false, 2, {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(3.0, 0.0, 0.0),
    });
    AABB a_nc = AABB::from_nurbscurve(curve, 0.5, false);

    MINI_CHECK(a_nc.is_valid());
    MINI_CHECK(a_nc.contains(Point(1.5, 0.0, 0.0)));

    NurbsSurface surf = NurbsSurface::create(false, false, 1, 1, 2, 2, {
        Point(0.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(0.0, 2.0, 0.0),
        Point(2.0, 2.0, 2.0),
    });
    AABB a_ns = AABB::from_nurbssurface(surf, 0.0);

    MINI_CHECK(a_ns.is_valid());
    MINI_CHECK(TOLERANCE.is_close(a_ns.volume(), 8.0));
}

}
