#include "mini_test.h"
#include "boolean_polyline.h"
#include "plane.h"
#include "point.h"
#include "polyline.h"
#include <cmath>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

static constexpr double PI2 = 6.283185307179586476;

MINI_TEST("Boolean Polyline", "Overlapping Squares") {

    const Polyline a({
        Point(-1, -1, 0),
        Point(1, -1, 0),
        Point(1, 1, 0),
        Point(-1, 1, 0),
        Point(-1, -1, 0)
    });
    const Polyline b({
        Point(0, 0, 0),
        Point(2, 0, 0),
        Point(2, 2, 0),
        Point(0, 2, 0),
        Point(0, 0, 0)
    });
    const std::vector<Polyline> isect = Polyline::boolean_op(a, b, 0);
    const std::vector<Polyline> uni = Polyline::boolean_op(a, b, 1);
    const std::vector<Polyline> diff = Polyline::boolean_op(a, b, 2);

    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);

    const Polyline far_a({
        Point(10, -1, 0),
        Point(12, -1, 0),
        Point(12, 1, 0),
        Point(10, 1, 0),
        Point(10, -1, 0)
    });
    const Polyline far_b({
        Point(14, -1, 0),
        Point(16, -1, 0),
        Point(16, 1, 0),
        Point(14, 1, 0),
        Point(14, -1, 0)
    });

    MINI_CHECK(BooleanPolyline::compute_count(far_a, far_b, 0) == 0);
    MINI_CHECK(
        BooleanPolyline::compute_count(far_a, far_b, 1) == static_cast<int>(far_a.point_count() + far_b.point_count())
    );
    MINI_CHECK(BooleanPolyline::compute_count(far_a, far_b, 2) == static_cast<int>(far_a.point_count()));
}

MINI_TEST("Boolean Polyline", "Circle Vs Rectangle") {

    std::vector<Point> pts;

    for (int i = 0; i < 64; i++) {
        const double a = PI2 * i / 64;
        pts.push_back(Point(5.0 + 1.5 * std::cos(a), 1.5 * std::sin(a), 0.0));
    }

    pts.push_back(pts[0]);
    const Polyline circle(pts);
    const Polyline rect({
        Point(4, -0.5, 0),
        Point(7, -0.5, 0),
        Point(7, 0.5, 0),
        Point(4, 0.5, 0),
        Point(4, -0.5, 0)
    });
    const std::vector<Polyline> isect = Polyline::boolean_op(circle, rect, 0);
    const std::vector<Polyline> uni = Polyline::boolean_op(circle, rect, 1);
    const std::vector<Polyline> diff = Polyline::boolean_op(circle, rect, 2);

    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Star Vs Circle") {

    std::vector<Point> star_pts;

    for (int i = 0; i < 10; i++) {
        const double a = PI2 * i / 10;
        const double r = (i % 2 == 0) ? 2.0 : 0.8;
        star_pts.push_back(Point(10.0 + r * std::cos(a), r * std::sin(a), 0.0));
    }

    star_pts.push_back(star_pts[0]);
    const Polyline star(star_pts);
    std::vector<Point> circ_pts;

    for (int i = 0; i < 32; i++) {
        const double a = PI2 * i / 32;
        circ_pts.push_back(Point(10.5 + 1.2 * std::cos(a), 0.5 + 1.2 * std::sin(a), 0.0));
    }

    circ_pts.push_back(circ_pts[0]);
    const Polyline circle(circ_pts);
    const std::vector<Polyline> isect = Polyline::boolean_op(star, circle, 0);
    const std::vector<Polyline> uni = Polyline::boolean_op(star, circle, 1);
    const std::vector<Polyline> diff = Polyline::boolean_op(star, circle, 2);

    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "L Shape Vs Rectangle") {

    const Polyline l_shape({
        Point(15, -1, 0),
        Point(18, -1, 0),
        Point(18, 0, 0),
        Point(16, 0, 0),
        Point(16, 2, 0),
        Point(15, 2, 0),
        Point(15, -1, 0)
    });
    const Polyline rect({
        Point(15.5, -0.5, 0),
        Point(18.5, -0.5, 0),
        Point(18.5, 1.5, 0),
        Point(15.5, 1.5, 0),
        Point(15.5, -0.5, 0)
    });
    const std::vector<Polyline> isect = Polyline::boolean_op(l_shape, rect, 0);
    const std::vector<Polyline> uni = Polyline::boolean_op(l_shape, rect, 1);
    const std::vector<Polyline> diff = Polyline::boolean_op(l_shape, rect, 2);

    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Two Large Circles") {

    std::vector<Point> pts_a;
    std::vector<Point> pts_b;

    for (int i = 0; i < 256; i++) {
        const double a = PI2 * i / 256;
        pts_a.push_back(Point(22.0 + 2.0 * std::cos(a), 2.0 * std::sin(a), 0.0));
        pts_b.push_back(Point(23.0 + 2.0 * std::cos(a), 0.5 + 2.0 * std::sin(a), 0.0));
    }

    pts_a.push_back(pts_a[0]);
    pts_b.push_back(pts_b[0]);
    const Polyline ca(pts_a);
    const Polyline cb(pts_b);
    const std::vector<Polyline> isect = Polyline::boolean_op(ca, cb, 0);
    const std::vector<Polyline> uni = Polyline::boolean_op(ca, cb, 1);
    const std::vector<Polyline> diff = Polyline::boolean_op(ca, cb, 2);

    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Diamond Vs Triangle") {

    const Polyline diamond({
        Point(28, 0, 0),
        Point(30, -2, 0),
        Point(32, 0, 0),
        Point(30, 2, 0),
        Point(28, 0, 0)
    });
    const Polyline tri({
        Point(29, -2, 0),
        Point(33, 0, 0),
        Point(29, 2, 0),
        Point(29, -2, 0)
    });
    const std::vector<Polyline> isect = Polyline::boolean_op(diamond, tri, 0);
    const std::vector<Polyline> uni = Polyline::boolean_op(diamond, tri, 1);
    const std::vector<Polyline> diff = Polyline::boolean_op(diamond, tri, 2);

    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Star Vs Star") {

    std::vector<Point> pts_a;

    for (int i = 0; i < 12; i++) {
        const double a = PI2 * i / 12;
        const double r = (i % 2 == 0) ? 2.5 : 1.0;
        pts_a.push_back(Point(36.0 + r * std::cos(a), r * std::sin(a), 0.0));
    }

    pts_a.push_back(pts_a[0]);
    std::vector<Point> pts_b;

    for (int i = 0; i < 10; i++) {
        const double a = PI2 * i / 10;
        const double r = (i % 2 == 0) ? 2.0 : 0.8;
        pts_b.push_back(Point(37.0 + r * std::cos(a), 0.5 + r * std::sin(a), 0.0));
    }

    pts_b.push_back(pts_b[0]);
    const Polyline sa(pts_a);
    const Polyline sb(pts_b);
    const std::vector<Polyline> isect = Polyline::boolean_op(sa, sb, 0);
    const std::vector<Polyline> uni = Polyline::boolean_op(sa, sb, 1);
    const std::vector<Polyline> diff = Polyline::boolean_op(sa, sb, 2);

    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Cross Shape") {

    const Polyline narrow({
        Point(42, -2, 0),
        Point(44, -2, 0),
        Point(44, 2, 0),
        Point(42, 2, 0),
        Point(42, -2, 0)
    });
    const Polyline wide({
        Point(40, -0.5, 0),
        Point(46, -0.5, 0),
        Point(46, 0.5, 0),
        Point(40, 0.5, 0),
        Point(40, -0.5, 0)
    });
    const std::vector<Polyline> isect = Polyline::boolean_op(narrow, wide, 0);
    const std::vector<Polyline> uni = Polyline::boolean_op(narrow, wide, 1);
    const std::vector<Polyline> diff = Polyline::boolean_op(narrow, wide, 2);

    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Concave Arrow Vs Circle") {

    const Polyline arrow({
        Point(49, 0, 0),
        Point(52, 2, 0),
        Point(51, 0.5, 0),
        Point(53, 0.5, 0),
        Point(53, -0.5, 0),
        Point(51, -0.5, 0),
        Point(52, -2, 0),
        Point(49, 0, 0)
    });
    std::vector<Point> pts;

    for (int i = 0; i < 48; i++) {
        const double a = PI2 * i / 48;
        pts.push_back(Point(51.5 + 1.5 * std::cos(a), 1.5 * std::sin(a), 0.0));
    }

    pts.push_back(pts[0]);
    const Polyline circle(pts);
    const std::vector<Polyline> isect = Polyline::boolean_op(arrow, circle, 0);
    const std::vector<Polyline> uni = Polyline::boolean_op(arrow, circle, 1);
    const std::vector<Polyline> diff = Polyline::boolean_op(arrow, circle, 2);

    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Two Large Circles 1000") {

    std::vector<Point> pts_a;
    std::vector<Point> pts_b;

    for (int i = 0; i < 1000; i++) {
        const double a = PI2 * i / 1000;
        pts_a.push_back(Point(58.0 + 3.0 * std::cos(a), 3.0 * std::sin(a), 0.0));
        pts_b.push_back(Point(59.5 + 3.0 * std::cos(a), 3.0 * std::sin(a), 0.0));
    }

    pts_a.push_back(pts_a[0]);
    pts_b.push_back(pts_b[0]);
    const Polyline ca(pts_a);
    const Polyline cb(pts_b);
    const std::vector<Polyline> isect = Polyline::boolean_op(ca, cb, 0);
    const std::vector<Polyline> uni = Polyline::boolean_op(ca, cb, 1);
    const std::vector<Polyline> diff = Polyline::boolean_op(ca, cb, 2);

    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Large Coords Auto Scale") {

    const Polyline a({
        Point(64e6, 1e6, 0),
        Point(64e6 + 2e6, 1e6, 0),
        Point(64e6 + 2e6, 1e6 + 2e6, 0),
        Point(64e6, 1e6 + 2e6, 0),
        Point(64e6, 1e6, 0)
    });
    const Polyline b({
        Point(64e6 + 1e6, 1e6 + 1e6, 0),
        Point(64e6 + 3e6, 1e6 + 1e6, 0),
        Point(64e6 + 3e6, 1e6 + 3e6, 0),
        Point(64e6 + 1e6, 1e6 + 3e6, 0),
        Point(64e6 + 1e6, 1e6 + 1e6, 0)
    });
    const std::vector<Polyline> isect = Polyline::boolean_op(a, b, 0);
    const std::vector<Polyline> uni = Polyline::boolean_op(a, b, 1);
    const std::vector<Polyline> diff = Polyline::boolean_op(a, b, 2);

    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Regions") {

    const Plane plane = Plane::xy_plane();
    const Polyline outer = Polyline::rectangle(Point(0.0, 0.0, 0.0), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0), 10.0, 10.0);
    const Polyline inner = Polyline::rectangle(Point(3.0, 3.0, 0.0), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0), 4.0, 4.0);
    const Polyline left = Polyline::rectangle(Point(0.0, -1.0, 0.0), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0), 5.0, 12.0);
    const Polyline apart = Polyline::rectangle(Point(20.0, 0.0, 0.0), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0), 10.0, 10.0);
    const std::vector<Polyline> frame = BooleanPolyline::compute_regions({outer}, {inner}, 2);
    const std::vector<Polyline> half = BooleanPolyline::compute_regions(frame, {left}, 0);
    const std::vector<Polyline> both = BooleanPolyline::compute_regions({outer}, {apart}, 1);
    int clockwise = 0;

    for (const Polyline& ring : frame)
        clockwise += ring.is_clockwise(plane) ? 1 : 0;

    MINI_CHECK(frame.size() == 2);
    MINI_CHECK(frame[0].is_closed());
    MINI_CHECK(clockwise == 1);
    MINI_CHECK(half.size() == 1);
    MINI_CHECK(half[0].point_count() == 9);
    MINI_CHECK(!half[0].is_clockwise(plane));
    MINI_CHECK(both.size() == 2);
}

MINI_TEST("Boolean Polyline", "Regions Orientation") {

    const Plane plane = Plane::xy_plane();
    const Polyline outer = Polyline::rectangle(Point(0.0, 0.0, 0.0), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0), 10.0, 10.0);
    const Polyline inner = Polyline::rectangle(Point(3.0, 3.0, 0.0), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0), 4.0, 4.0);
    const std::vector<Polyline> frame = BooleanPolyline::compute_regions({outer, inner}, {}, 1);
    const std::vector<Polyline> turned = BooleanPolyline::compute_regions({outer.reversed(), inner}, {}, 1);

    MINI_CHECK(frame.size() == 2);
    MINI_CHECK(turned.size() == 2);

    for (const Polyline& ring : frame)
        MINI_CHECK(ring.is_clockwise(plane) == (ring.get_point(0)[0] > 1.0 && ring.get_point(0)[0] < 9.0));

    for (const Polyline& ring : turned)
        MINI_CHECK(ring.is_clockwise(plane) == (ring.get_point(0)[0] > 1.0 && ring.get_point(0)[0] < 9.0));
}

MINI_TEST("Boolean Polyline Open", "Horizontal Line Vs Unit Square") {

    const Polyline open_line({
        Point(-2, 0, 0),
        Point(2, 0, 0)
    });
    const Polyline sq({
        Point(-1, -1, 0),
        Point(1, -1, 0),
        Point(1, 1, 0),
        Point(-1, 1, 0),
        Point(-1, -1, 0)
    });
    const std::vector<Polyline> out = BooleanPolyline::clip_open_against_closed(open_line, sq);

    MINI_CHECK(out.size() == 1);
    MINI_CHECK(out[0].point_count() == 2);

    const Point p0 = out[0].get_point(0);
    const Point p1 = out[0].get_point(1);

    MINI_CHECK(std::fabs(std::fabs(p0[0]) - 1.0) < 1e-6);
    MINI_CHECK(std::fabs(std::fabs(p1[0]) - 1.0) < 1e-6);
    MINI_CHECK(std::fabs(p0[1]) < 1e-6);
    MINI_CHECK(std::fabs(p1[1]) < 1e-6);
}

MINI_TEST("Boolean Polyline Open", "Diagonal Line Vs Unit Square") {

    const Polyline open_line({
        Point(-2, -2, 0),
        Point(2, 2, 0)
    });
    const Polyline sq({
        Point(-1, -1, 0),
        Point(1, -1, 0),
        Point(1, 1, 0),
        Point(-1, 1, 0),
        Point(-1, -1, 0)
    });
    const std::vector<Polyline> out = BooleanPolyline::clip_open_against_closed(open_line, sq);

    MINI_CHECK(out.size() == 1);
    MINI_CHECK(out[0].point_count() == 2);

    const Point p0 = out[0].get_point(0);
    const Point p1 = out[0].get_point(1);

    MINI_CHECK(std::fabs(std::fabs(p0[0]) - 1.0) < 1e-6);
    MINI_CHECK(std::fabs(std::fabs(p1[0]) - 1.0) < 1e-6);
}

MINI_TEST("Boolean Polyline Open", "Interior Open Path Passes Through") {

    const Polyline open_path({
        Point(-2, 0, 0),
        Point(0, 0.2, 0),
        Point(2, 0, 0)
    });
    const Polyline sq({
        Point(-1, -1, 0),
        Point(1, -1, 0),
        Point(1, 1, 0),
        Point(-1, 1, 0),
        Point(-1, -1, 0)
    });
    const std::vector<Polyline> out = BooleanPolyline::clip_open_against_closed(open_path, sq);

    MINI_CHECK(out.size() == 1);
    MINI_CHECK(out[0].point_count() >= 3);
}

} // namespace session_cpp
