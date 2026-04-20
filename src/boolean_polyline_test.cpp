#include "mini_test.h"
#include "polyline.h"
#include "boolean_polyline.h"
#include <cmath>

namespace session_cpp {
using namespace session_cpp::mini_test;

static constexpr double PI2 = 6.283185307179586476;

MINI_TEST("Boolean Polyline", "Overlapping Squares") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"
    Polyline a({Point(-1,-1,0), Point(1,-1,0), Point(1,1,0), Point(-1,1,0), Point(-1,-1,0)});
    Polyline b({Point(0,0,0), Point(2,0,0), Point(2,2,0), Point(0,2,0), Point(0,0,0)});
    auto isect = Polyline::boolean_op(a, b, 0);
    auto uni   = Polyline::boolean_op(a, b, 1);
    auto diff  = Polyline::boolean_op(a, b, 2);
    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Circle Vs Rectangle") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"
    std::vector<Point> pts;
    for (int i = 0; i < 64; i++) {
        double a = PI2 * i / 64;
        pts.push_back(Point(5.0 + 1.5 * std::cos(a), 1.5 * std::sin(a), 0.0));
    }
    pts.push_back(pts[0]);
    Polyline circle(pts);
    Polyline rect({Point(4,-0.5,0), Point(7,-0.5,0), Point(7,0.5,0), Point(4,0.5,0), Point(4,-0.5,0)});
    auto isect = Polyline::boolean_op(circle, rect, 0);
    auto uni   = Polyline::boolean_op(circle, rect, 1);
    auto diff  = Polyline::boolean_op(circle, rect, 2);
    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Star Vs Circle") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"
    std::vector<Point> star_pts;
    for (int i = 0; i < 10; i++) {
        double a = PI2 * i / 10;
        double r = (i % 2 == 0) ? 2.0 : 0.8;
        star_pts.push_back(Point(10.0 + r * std::cos(a), r * std::sin(a), 0.0));
    }
    star_pts.push_back(star_pts[0]);
    Polyline star(star_pts);
    std::vector<Point> circ_pts;
    for (int i = 0; i < 32; i++) {
        double a = PI2 * i / 32;
        circ_pts.push_back(Point(10.5 + 1.2 * std::cos(a), 0.5 + 1.2 * std::sin(a), 0.0));
    }
    circ_pts.push_back(circ_pts[0]);
    Polyline circle(circ_pts);
    auto isect = Polyline::boolean_op(star, circle, 0);
    auto uni   = Polyline::boolean_op(star, circle, 1);
    auto diff  = Polyline::boolean_op(star, circle, 2);
    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "L Shape Vs Rectangle") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"
    Polyline l_shape({
        Point(15,-1,0), Point(18,-1,0), Point(18,0,0),
        Point(16,0,0), Point(16,2,0), Point(15,2,0), Point(15,-1,0)
    });
    Polyline rect({Point(15.5,-0.5,0), Point(18.5,-0.5,0), Point(18.5,1.5,0), Point(15.5,1.5,0), Point(15.5,-0.5,0)});
    auto isect = Polyline::boolean_op(l_shape, rect, 0);
    auto uni   = Polyline::boolean_op(l_shape, rect, 1);
    auto diff  = Polyline::boolean_op(l_shape, rect, 2);
    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Two Large Circles") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"
    std::vector<Point> pts_a, pts_b;
    for (int i = 0; i < 256; i++) {
        double a = PI2 * i / 256;
        pts_a.push_back(Point(22.0 + 2.0 * std::cos(a), 2.0 * std::sin(a), 0.0));
        pts_b.push_back(Point(23.0 + 2.0 * std::cos(a), 0.5 + 2.0 * std::sin(a), 0.0));
    }
    pts_a.push_back(pts_a[0]);
    pts_b.push_back(pts_b[0]);
    Polyline ca(pts_a), cb(pts_b);
    auto isect = Polyline::boolean_op(ca, cb, 0);
    auto uni   = Polyline::boolean_op(ca, cb, 1);
    auto diff  = Polyline::boolean_op(ca, cb, 2);
    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Diamond Vs Triangle") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"
    Polyline diamond({Point(28,0,0), Point(30,-2,0), Point(32,0,0), Point(30,2,0), Point(28,0,0)});
    Polyline tri({Point(29,-2,0), Point(33,0,0), Point(29,2,0), Point(29,-2,0)});
    auto isect = Polyline::boolean_op(diamond, tri, 0);
    auto uni   = Polyline::boolean_op(diamond, tri, 1);
    auto diff  = Polyline::boolean_op(diamond, tri, 2);
    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Star Vs Star") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"
    std::vector<Point> pts_a;
    for (int i = 0; i < 12; i++) {
        double a = PI2 * i / 12;
        double r = (i % 2 == 0) ? 2.5 : 1.0;
        pts_a.push_back(Point(36.0 + r * std::cos(a), r * std::sin(a), 0.0));
    }
    pts_a.push_back(pts_a[0]);
    std::vector<Point> pts_b;
    for (int i = 0; i < 10; i++) {
        double a = PI2 * i / 10;
        double r = (i % 2 == 0) ? 2.0 : 0.8;
        pts_b.push_back(Point(37.0 + r * std::cos(a), 0.5 + r * std::sin(a), 0.0));
    }
    pts_b.push_back(pts_b[0]);
    Polyline sa(pts_a), sb(pts_b);
    auto isect = Polyline::boolean_op(sa, sb, 0);
    auto uni   = Polyline::boolean_op(sa, sb, 1);
    auto diff  = Polyline::boolean_op(sa, sb, 2);
    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Cross Shape") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"
    Polyline narrow({Point(42,-2,0), Point(44,-2,0), Point(44,2,0), Point(42,2,0), Point(42,-2,0)});
    Polyline wide({Point(40,-0.5,0), Point(46,-0.5,0), Point(46,0.5,0), Point(40,0.5,0), Point(40,-0.5,0)});
    auto isect = Polyline::boolean_op(narrow, wide, 0);
    auto uni   = Polyline::boolean_op(narrow, wide, 1);
    auto diff  = Polyline::boolean_op(narrow, wide, 2);
    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Concave Arrow Vs Circle") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"
    Polyline arrow({
        Point(49,0,0), Point(52,2,0), Point(51,0.5,0), Point(53,0.5,0),
        Point(53,-0.5,0), Point(51,-0.5,0), Point(52,-2,0), Point(49,0,0)
    });
    std::vector<Point> pts;
    for (int i = 0; i < 48; i++) {
        double a = PI2 * i / 48;
        pts.push_back(Point(51.5 + 1.5 * std::cos(a), 1.5 * std::sin(a), 0.0));
    }
    pts.push_back(pts[0]);
    Polyline circle(pts);
    auto isect = Polyline::boolean_op(arrow, circle, 0);
    auto uni   = Polyline::boolean_op(arrow, circle, 1);
    auto diff  = Polyline::boolean_op(arrow, circle, 2);
    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Two Large Circles 1000") {
    // uncomment #include "polyline.h"
    // uncomment #include "point.h"
    std::vector<Point> pts_a, pts_b;
    for (int i = 0; i < 1000; i++) {
        double a = PI2 * i / 1000;
        pts_a.push_back(Point(58.0 + 3.0 * std::cos(a), 3.0 * std::sin(a), 0.0));
        pts_b.push_back(Point(59.5 + 3.0 * std::cos(a), 3.0 * std::sin(a), 0.0));
    }
    pts_a.push_back(pts_a[0]);
    pts_b.push_back(pts_b[0]);
    Polyline ca(pts_a), cb(pts_b);
    auto isect = Polyline::boolean_op(ca, cb, 0);
    auto uni   = Polyline::boolean_op(ca, cb, 1);
    auto diff  = Polyline::boolean_op(ca, cb, 2);
    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

MINI_TEST("Boolean Polyline", "Large Coords Auto Scale") {
    // Coordinates near 1e7 would overflow int64 cross products with the old
    // fixed 1e9 scale: (1e7*1e9)^2 = 1e32 >> int64::max (~9.2e18)
    Polyline a({Point(64e6,1e6,0), Point(64e6+2e6,1e6,0), Point(64e6+2e6,1e6+2e6,0), Point(64e6,1e6+2e6,0), Point(64e6,1e6,0)});
    Polyline b({Point(64e6+1e6,1e6+1e6,0), Point(64e6+3e6,1e6+1e6,0), Point(64e6+3e6,1e6+3e6,0), Point(64e6+1e6,1e6+3e6,0), Point(64e6+1e6,1e6+1e6,0)});
    auto isect = Polyline::boolean_op(a, b, 0);
    auto uni   = Polyline::boolean_op(a, b, 1);
    auto diff  = Polyline::boolean_op(a, b, 2);
    MINI_CHECK(isect.size() >= 1);
    MINI_CHECK(isect[0].point_count() > 0);
    MINI_CHECK(uni.size() >= 1);
    MINI_CHECK(uni[0].point_count() > 0);
    MINI_CHECK(diff.size() >= 1);
    MINI_CHECK(diff[0].point_count() > 0);
}

// ── Open-subject × closed-clip Vatti tests ──────────────────────────────
// Exercises BooleanPolyline::clip_open_against_closed. Covers the three
// primary behaviours needed by rossiniere joints:
//   1) simple transverse crossing (enter + exit),
//   2) diagonal transverse crossing,
//   3) full joint-rectangle-vs-plate fixture (multi-segment open path
//      traversing a rotated-plane plate).

MINI_TEST("Boolean Polyline Open", "Horizontal Line Vs Unit Square") {
    Polyline open_line({Point(-2, 0, 0), Point(2, 0, 0)});
    Polyline sq({Point(-1,-1,0), Point(1,-1,0), Point(1,1,0), Point(-1,1,0), Point(-1,-1,0)});
    auto out = BooleanPolyline::clip_open_against_closed(open_line, sq);
    MINI_CHECK(out.size() == 1);
    MINI_CHECK(out[0].point_count() == 2);
    Point p0 = out[0].get_point(0);
    Point p1 = out[0].get_point(1);
    MINI_CHECK(std::fabs(std::fabs(p0[0]) - 1.0) < 1e-6);
    MINI_CHECK(std::fabs(std::fabs(p1[0]) - 1.0) < 1e-6);
    MINI_CHECK(std::fabs(p0[1]) < 1e-6);
    MINI_CHECK(std::fabs(p1[1]) < 1e-6);
}

MINI_TEST("Boolean Polyline Open", "Diagonal Line Vs Unit Square") {
    Polyline open_line({Point(-2, -2, 0), Point(2, 2, 0)});
    Polyline sq({Point(-1,-1,0), Point(1,-1,0), Point(1,1,0), Point(-1,1,0), Point(-1,-1,0)});
    auto out = BooleanPolyline::clip_open_against_closed(open_line, sq);
    MINI_CHECK(out.size() == 1);
    MINI_CHECK(out[0].point_count() == 2);
    Point p0 = out[0].get_point(0);
    Point p1 = out[0].get_point(1);
    MINI_CHECK(std::fabs(std::fabs(p0[0]) - 1.0) < 1e-6);
    MINI_CHECK(std::fabs(std::fabs(p1[0]) - 1.0) < 1e-6);
}

MINI_TEST("Boolean Polyline Open", "Interior Open Path Passes Through") {
    // Multi-vertex open path with both endpoints OUTSIDE clip — expect a
    // single piece crossing from entry to exit.
    Polyline open_path({Point(-2, 0, 0), Point(0, 0.2, 0), Point(2, 0, 0)});
    Polyline sq({Point(-1,-1,0), Point(1,-1,0), Point(1,1,0), Point(-1,1,0), Point(-1,-1,0)});
    auto out = BooleanPolyline::clip_open_against_closed(open_path, sq);
    MINI_CHECK(out.size() == 1);
    MINI_CHECK(out[0].point_count() >= 3);
}

} // namespace session_cpp
