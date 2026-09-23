#include "mini_test.h"
#include "convex_hull.h"
#include "mesh.h"
#include "point.h"
#include "tolerance.h"
#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("ConvexHull", "Hull 2d") {

    const std::vector<Point> points = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(1.0, 1.0, 0.0),
        Point(0.0, 1.0, 0.0),
        Point(0.5, 0.5, 0.0),
        Point(0.3, 0.3, 0.0),
    };
    const std::vector<Point> hull = ConvexHull::hull_2d(points);

    MINI_CHECK(hull.size() == 4);
}

MINI_TEST("ConvexHull", "Hull 2d Collinear") {

    const std::vector<Point> points = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(3.0, 0.0, 0.0),
        Point(1.5, 1.0, 0.0),
    };
    const std::vector<Point> hull = ConvexHull::hull_2d(points);

    MINI_CHECK(hull.size() >= 3);
}

MINI_TEST("ConvexHull", "Hull 2d Circle") {

    const size_t n = 12;
    std::vector<Point> points;

    for (size_t i = 0; i < n; ++i) {
        const double angle = 2.0 * Tolerance::PI * i / n;
        points.push_back(Point(std::cos(angle), std::sin(angle), 0.0));
    }

    points.push_back(Point(0.0, 0.0, 0.0));

    const std::vector<Point> hull = ConvexHull::hull_2d(points);

    MINI_CHECK(hull.size() == n);
}

MINI_TEST("ConvexHull", "Hull 3d") {

    const std::vector<Point> points = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(0.0, 1.0, 0.0),
        Point(0.0, 0.0, 1.0),
        Point(0.25, 0.25, 0.25),
    };
    const Mesh mesh = ConvexHull::hull_3d(points);

    MINI_CHECK(mesh.number_of_vertices() == 4);
    MINI_CHECK(mesh.number_of_faces() == 4);

    const Mesh degenerate = ConvexHull::hull_3d({Point(0, 0, 0), Point(1, 0, 0), Point(2, 0, 0), Point(3, 0, 0)});

    MINI_CHECK(degenerate.number_of_vertices() == 4);
    MINI_CHECK(degenerate.number_of_faces() == 0);
}

MINI_TEST("ConvexHull", "Hull 3d Cube") {

    const std::vector<Point> points = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(1.0, 1.0, 0.0),
        Point(0.0, 1.0, 0.0),
        Point(0.0, 0.0, 1.0),
        Point(1.0, 0.0, 1.0),
        Point(1.0, 1.0, 1.0),
        Point(0.0, 1.0, 1.0),
        Point(0.5, 0.5, 0.5),
    };
    const Mesh mesh = ConvexHull::hull_3d(points);

    MINI_CHECK(mesh.number_of_vertices() == 8);
    MINI_CHECK(mesh.number_of_faces() == 12);
}

} // namespace session_cpp
