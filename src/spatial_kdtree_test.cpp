#include "mini_test.h"
#include "spatial_kdtree.h"
#include "point.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("SpatialKDTree", "Constructor") {
    // using session_cpp::Point;
    // using session_cpp::SpatialKDTree;

    const std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(3.0, 0.0, 0.0),
        Point(10.0, 0.0, 0.0),
    };

    const SpatialKDTree tree(pts);
    const std::pair<int, double> hit = tree.nearest(Point(2.0, 0.0, 0.0));

    MINI_CHECK(hit.first == 1);
    MINI_CHECK(TOLERANCE.is_close(hit.second, 1.0));
}

MINI_TEST("SpatialKDTree", "Nearest") {
    // using session_cpp::Point;
    // using session_cpp::SpatialKDTree;

    const std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(3.0, 0.0, 0.0),
        Point(4.0, 0.0, 0.0),
    };

    const SpatialKDTree tree(pts);
    const Point query(1.1, 0.0, 0.0);
    const std::pair<int, double> hit = tree.nearest(query);

    MINI_CHECK(hit.first == 1);
    MINI_CHECK(TOLERANCE.is_close(hit.second, 0.1));
}

MINI_TEST("SpatialKDTree", "Nearest K") {
    // using session_cpp::Point;
    // using session_cpp::SpatialKDTree;

    const std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(3.0, 0.0, 0.0),
        Point(4.0, 0.0, 0.0),
    };

    const SpatialKDTree tree(pts);
    const Point query(1.5, 0.0, 0.0);
    const std::vector<std::pair<int, double>> result = tree.nearest_k(query, 3);

    MINI_CHECK(result.size() == 3);
    MINI_CHECK(TOLERANCE.is_close(result[0].second, 0.5));
    MINI_CHECK(TOLERANCE.is_close(result[1].second, 0.5));
    MINI_CHECK(TOLERANCE.is_close(result[2].second, 1.5));
}

MINI_TEST("SpatialKDTree", "Radius Search") {
    // using session_cpp::Point;
    // using session_cpp::SpatialKDTree;

    const std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(5.0, 0.0, 0.0),
    };

    const SpatialKDTree tree(pts);
    const Point query(0.5, 0.0, 0.0);
    const std::vector<std::pair<int, double>> result = tree.radius_search(query, 1.1);

    MINI_CHECK(result.size() == 2);
    MINI_CHECK(TOLERANCE.is_close(result[0].second, 0.5));
    MINI_CHECK(TOLERANCE.is_close(result[1].second, 0.5));
}

} // namespace session_cpp
