#include "mini_test.h"
#include "kdtree.h"
#include "point.h"
#include "tolerance.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("KDTree", "Constructor") {
    // uncomment #include "kdtree.h"
    // uncomment #include "point.h"
    // KDTree: O(n log n) build, O(log n) nearest — static point set closest search
    std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(3.0, 0.0, 0.0),
        Point(10.0, 0.0, 0.0),
    };
    KDTree tree(pts);
    auto [idx, dist] = tree.nearest(Point(2.0, 0.0, 0.0));

    MINI_CHECK(idx == 1);
    MINI_CHECK(TOLERANCE.is_close(dist, 1.0));
}

MINI_TEST("KDTree", "Nearest") {
    // uncomment #include "kdtree.h"
    // uncomment #include "point.h"
    // 5 known points on a line: 0, 1, 2, 3, 4
    // Query at 1.1 — nearest should be index 1 (point at x=1), distance 0.1
    std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(3.0, 0.0, 0.0),
        Point(4.0, 0.0, 0.0),
    };
    KDTree tree(pts);
    Point query(1.1, 0.0, 0.0);
    auto [idx, dist] = tree.nearest(query);

    MINI_CHECK(idx == 1);
    MINI_CHECK(TOLERANCE.is_close(dist, 0.1));
}

MINI_TEST("KDTree", "Nearest K") {
    // uncomment #include "kdtree.h"
    // uncomment #include "point.h"
    // 5 points on X axis: 0, 1, 2, 3, 4
    // Query at 1.5 — 3 nearest are: x=1 (d=0.5), x=2 (d=0.5), x=3 (d=1.5)
    std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(3.0, 0.0, 0.0),
        Point(4.0, 0.0, 0.0),
    };
    KDTree tree(pts);
    Point query(1.5, 0.0, 0.0);
    auto result = tree.nearest_k(query, 3);

    MINI_CHECK(result.size() == 3);
    MINI_CHECK(TOLERANCE.is_close(result[0].second, 0.5));
    MINI_CHECK(TOLERANCE.is_close(result[1].second, 0.5));
    MINI_CHECK(TOLERANCE.is_close(result[2].second, 1.5));
}

MINI_TEST("KDTree", "Radius Search") {
    // uncomment #include "kdtree.h"
    // uncomment #include "point.h"
    // 4 points: 0, 1, 2, 5 on X axis
    // Query at 0.5, radius 1.1 — finds x=0 (d=0.5) and x=1 (d=0.5)
    std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(5.0, 0.0, 0.0),
    };
    KDTree tree(pts);
    Point query(0.5, 0.0, 0.0);
    auto result = tree.radius_search(query, 1.1);

    MINI_CHECK(result.size() == 2);
    MINI_CHECK(TOLERANCE.is_close(result[0].second, 0.5));
    MINI_CHECK(TOLERANCE.is_close(result[1].second, 0.5));
}

} // namespace session_cpp
