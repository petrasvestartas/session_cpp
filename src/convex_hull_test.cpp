#include "mini_test.h"
#include "convex_hull.h"
#include "point.h"
#include <cmath>
#include <numbers>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("ConvexHull", "Hull2d") {
    // uncomment #include "convex_hull.h"
    // uncomment #include "point.h"
    std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(1.0, 1.0, 0.0),
        Point(0.0, 1.0, 0.0),
        Point(0.5, 0.5, 0.0),
        Point(0.3, 0.3, 0.0),
    };
    std::vector<Point> hull = ConvexHull::hull_2d(pts);

    MINI_CHECK(hull.size() == 4);
}

MINI_TEST("ConvexHull", "Hull2dCollinear") {
    // uncomment #include "convex_hull.h"
    // uncomment #include "point.h"
    std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(3.0, 0.0, 0.0),
        Point(1.5, 1.0, 0.0),
    };
    std::vector<Point> hull = ConvexHull::hull_2d(pts);

    MINI_CHECK(hull.size() >= 3);
}

MINI_TEST("ConvexHull", "Hull2dCircle") {
    // uncomment #include "convex_hull.h"
    // uncomment #include "point.h"
    int n = 12;
    std::vector<Point> pts;
    for (int i = 0; i < n; ++i) {
        double angle = 2.0 * std::numbers::pi * i / n;
        pts.push_back(Point(std::cos(angle), std::sin(angle), 0.0));
    }
    pts.push_back(Point(0.0, 0.0, 0.0));
    std::vector<Point> hull = ConvexHull::hull_2d(pts);

    MINI_CHECK(hull.size() == static_cast<size_t>(n));
}

MINI_TEST("ConvexHull", "Hull3d") {
    // uncomment #include "convex_hull.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    std::vector<Point> pts = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 0.0, 0.0),
        Point(0.0, 1.0, 0.0),
        Point(0.0, 0.0, 1.0),
        Point(0.25, 0.25, 0.25),
    };
    Mesh mesh = ConvexHull::hull_3d(pts);

    MINI_CHECK(mesh.number_of_vertices() == 4);
    MINI_CHECK(mesh.number_of_faces() == 4);
}

MINI_TEST("ConvexHull", "Hull3dCube") {
    // uncomment #include "convex_hull.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    std::vector<Point> pts = {
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
    Mesh mesh = ConvexHull::hull_3d(pts);

    MINI_CHECK(mesh.number_of_vertices() == 8);
    MINI_CHECK(mesh.number_of_faces() == 12);
}

} // namespace session_cpp
