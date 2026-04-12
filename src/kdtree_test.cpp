#include "mini_test.h"
#include "kdtree.h"
#include "point.h"
#include "tolerance.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

using namespace session_cpp::mini_test;

namespace session_cpp {

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

MINI_TEST("KDTree", "Single Point") {
    // uncomment #include "kdtree.h"
    // uncomment #include "point.h"
    // Tree with one point at (3,4,0). Query from origin.
    // Distance = sqrt(9+16) = 5 (3-4-5 right triangle)
    std::vector<Point> pts = {Point(3.0, 4.0, 0.0)};
    KDTree tree(pts);
    Point query(0.0, 0.0, 0.0);
    auto [idx, dist] = tree.nearest(query);

    MINI_CHECK(idx == 0);
    MINI_CHECK(TOLERANCE.is_close(dist, 5.0));
}

MINI_TEST("KDTree", "Nearest Brute Force") {
    // uncomment #include "kdtree.h"
    // uncomment #include "point.h"
    // 8 points in 3D — verify KDTree matches brute-force for several queries
    std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(0.0, 1.0, 0.0),
        Point(0.0, 0.0, 1.0),
        Point(5.0, 5.0, 5.0),
        Point(-3.0, 2.0, 1.0),
        Point(2.0, -1.0, 3.0),
        Point(-1.0, -1.0, -1.0),
    };
    KDTree tree(pts);
    std::vector<Point> queries = {
        Point(0.5, 0.5, 0.5),
        Point(4.0, 4.0, 4.0),
        Point(-2.0, 1.0, 0.0),
    };
    bool all_match = true;
    for (const auto& q : queries) {
        auto [idx, dist] = tree.nearest(q);
        // Brute-force: find closest by scanning all points
        int brute = 0;
        for (int i = 1; i < (int)pts.size(); ++i) {
            if (Point::squared_distance(pts[i], q) < Point::squared_distance(pts[brute], q))
                brute = i;
        }
        double brute_d = Point::distance(pts[brute], q);
        if (!TOLERANCE.is_close(dist, brute_d)) all_match = false;
    }

    MINI_CHECK(all_match);
}

} // namespace session_cpp
