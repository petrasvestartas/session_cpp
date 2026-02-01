#include "mini_test.h"
#include "triangulation_2d.h"
#include "tolerance.h"

#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

static double triangle_area_2d(const std::vector<Point>& pts, const Triangle2D& t) {
    double ax = pts[t.v0][0], ay = pts[t.v0][1];
    double bx = pts[t.v1][0], by = pts[t.v1][1];
    double cx = pts[t.v2][0], cy = pts[t.v2][1];
    return std::abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) * 0.5;
}

static double total_area(const std::vector<Point>& pts, const std::vector<Triangle2D>& tris) {
    double a = 0.0;
    for (const auto& t : tris) a += triangle_area_2d(pts, t);
    return a;
}

MINI_TEST("Triangulation2D", "triangle") {
    Polyline boundary({Point(0, 0, 0), Point(1, 0, 0), Point(0, 1, 0), Point(0, 0, 0)});
    auto tris = Triangulation2D::triangulate(boundary);

    MINI_CHECK(tris.size() == 1);
    MINI_CHECK(tris[0].v0 >= 0);
    MINI_CHECK(tris[0].v1 >= 0);
    MINI_CHECK(tris[0].v2 >= 0);
}

MINI_TEST("Triangulation2D", "square") {
    Polyline boundary({Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 1, 0), Point(0, 0, 0)});
    auto tris = Triangulation2D::triangulate(boundary);
    std::vector<Point> pts = {Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 1, 0)};
    double a = total_area(pts, tris);

    MINI_CHECK(tris.size() == 2);
    MINI_CHECK(std::abs(a - 1.0) < 1e-10);
}

MINI_TEST("Triangulation2D", "convex_polygon") {
    double r = 1.0;
    std::vector<Point> hex_pts;
    for (int i = 0; i < 6; ++i) {
        double angle = i * TOLERANCE.PI / 3.0;
        hex_pts.push_back(Point(r * std::cos(angle), r * std::sin(angle), 0));
    }
    hex_pts.push_back(hex_pts[0]);
    Polyline boundary(hex_pts);
    auto tris = Triangulation2D::triangulate(boundary);

    MINI_CHECK(tris.size() == 4);
}

MINI_TEST("Triangulation2D", "concave_polygon") {
    Polyline boundary({
        Point(0, 0, 0), Point(2, 0, 0), Point(2, 2, 0),
        Point(1, 1, 0), Point(0, 2, 0), Point(0, 0, 0)
    });
    auto tris = Triangulation2D::triangulate(boundary);
    std::vector<Point> pts = {
        Point(0, 0, 0), Point(2, 0, 0), Point(2, 2, 0),
        Point(1, 1, 0), Point(0, 2, 0)
    };
    double a = total_area(pts, tris);
    double expected = 3.0;

    MINI_CHECK(tris.size() == 3);
    MINI_CHECK(std::abs(a - expected) < 1e-10);
}

MINI_TEST("Triangulation2D", "polygon_with_hole") {
    Polyline boundary({
        Point(0, 0, 0), Point(4, 0, 0), Point(4, 4, 0), Point(0, 4, 0), Point(0, 0, 0)
    });
    Polyline hole({
        Point(1, 1, 0), Point(3, 1, 0), Point(3, 3, 0), Point(1, 3, 0), Point(1, 1, 0)
    });
    auto tris = Triangulation2D::triangulate(boundary, {hole});
    std::vector<Point> all_pts = {
        Point(0, 0, 0), Point(4, 0, 0), Point(4, 4, 0), Point(0, 4, 0),
        Point(1, 1, 0), Point(3, 1, 0), Point(3, 3, 0), Point(1, 3, 0)
    };
    double a = total_area(all_pts, tris);
    double expected = 4.0 * 4.0 - 2.0 * 2.0;

    MINI_CHECK(tris.size() >= 8);
    MINI_CHECK(std::abs(a - expected) < 1e-10);
}

MINI_TEST("Triangulation2D", "polygon_with_multiple_holes") {
    Polyline boundary({
        Point(0, 0, 0), Point(10, 0, 0), Point(10, 8, 0), Point(0, 8, 0), Point(0, 0, 0)
    });
    Polyline hole0({
        Point(1, 1, 0), Point(3, 1, 0), Point(3, 3, 0), Point(1, 3, 0), Point(1, 1, 0)
    });
    Polyline hole1({
        Point(5, 4, 0), Point(9, 4, 0), Point(9, 7, 0), Point(5, 7, 0), Point(5, 4, 0)
    });
    auto tris = Triangulation2D::triangulate(boundary, {hole0, hole1});
    std::vector<Point> all_pts = {
        Point(0, 0, 0), Point(10, 0, 0), Point(10, 8, 0), Point(0, 8, 0),
        Point(1, 1, 0), Point(3, 1, 0), Point(3, 3, 0), Point(1, 3, 0),
        Point(5, 4, 0), Point(9, 4, 0), Point(9, 7, 0), Point(5, 7, 0)
    };
    double a = total_area(all_pts, tris);
    double expected = 10.0 * 8.0 - 2.0 * 2.0 - 4.0 * 3.0;

    MINI_CHECK(tris.size() >= 10);
    MINI_CHECK(std::abs(a - expected) < 1e-6);
}

MINI_TEST("Triangulation2D", "winding_correction") {
    Polyline boundary({
        Point(0, 1, 0), Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 1, 0)
    });
    auto tris = Triangulation2D::triangulate(boundary);
    std::vector<Point> pts = {Point(0, 1, 0), Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0)};
    double a = total_area(pts, tris);

    MINI_CHECK(tris.size() == 2);
    MINI_CHECK(std::abs(a - 1.0) < 1e-10);
}

} // namespace session_cpp
