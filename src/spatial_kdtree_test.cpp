#include "mini_test.h"
#include "spatial_kdtree.h"
#include "point.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("SpatialKDTree", "Constructor") {

    std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(3.0, 0.0, 0.0),
        Point(10.0, 0.0, 0.0),
    };
    SpatialKDTree tree(pts);
    auto [idx, dist] = tree.nearest(Point(2.0, 0.0, 0.0));

    MINI_CHECK(idx == 1);
    MINI_CHECK(TOLERANCE.is_close(dist, 1.0));
}

MINI_TEST("SpatialKDTree", "Nearest") {

    std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(3.0, 0.0, 0.0),
        Point(4.0, 0.0, 0.0),
    };
    SpatialKDTree tree(pts);
    Point query(1.1, 0.0, 0.0);
    auto [idx, dist] = tree.nearest(query);

    MINI_CHECK(idx == 1);
    MINI_CHECK(TOLERANCE.is_close(dist, 0.1));
}

MINI_TEST("SpatialKDTree", "Nearest K") {

    std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(3.0, 0.0, 0.0),
        Point(4.0, 0.0, 0.0),
    };
    SpatialKDTree tree(pts);
    Point query(1.5, 0.0, 0.0);
    std::vector<std::pair<int, double>> result = tree.nearest_k(query, 3);

    MINI_CHECK(result.size() == 3);
    MINI_CHECK(TOLERANCE.is_close(result[0].second, 0.5));
    MINI_CHECK(TOLERANCE.is_close(result[1].second, 0.5));
    MINI_CHECK(TOLERANCE.is_close(result[2].second, 1.5));
}

MINI_TEST("SpatialKDTree", "Radius Search") {

    std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(5.0, 0.0, 0.0),
    };
    SpatialKDTree tree(pts);
    Point query(0.5, 0.0, 0.0);
    std::vector<std::pair<int, double>> result = tree.radius_search(query, 1.1);

    MINI_CHECK(result.size() == 2);
    MINI_CHECK(TOLERANCE.is_close(result[0].second, 0.5));
    MINI_CHECK(TOLERANCE.is_close(result[1].second, 0.5));
}

} // namespace session_cpp
