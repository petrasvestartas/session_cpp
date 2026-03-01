#include "mini_test.h"
#include "trimesh_cdt.h"
#include "tolerance.h"

#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

static double tri_area(const std::pair<double,double>& a, const std::pair<double,double>& b, const std::pair<double,double>& c) {
    return std::abs((b.first-a.first)*(c.second-a.second) - (c.first-a.first)*(b.second-a.second)) * 0.5;
}

MINI_TEST("TrimeshCdt", "Triangle") {
    std::vector<std::array<int, 3>> tris = cdt_triangulate(
        {{0, 0}, {1, 0}, {0, 1}},
        {});

    MINI_CHECK(tris.size() == 1);
    MINI_CHECK(tris[0][0] != tris[0][1]);
    MINI_CHECK(tris[0][1] != tris[0][2]);
    MINI_CHECK(tris[0][0] != tris[0][2]);
}

MINI_TEST("TrimeshCdt", "Square") {
    std::vector<std::array<int, 3>> tris = cdt_triangulate(
        {{0, 0}, {1, 0}, {1, 1}, {0, 1}},
        {});

    MINI_CHECK(tris.size() == 2);
}

MINI_TEST("TrimeshCdt", "Convex Polygon") {
    std::vector<std::pair<double, double>> hex;
    for (int i = 0; i < 6; ++i) {
        double a = i * TOLERANCE.PI / 3.0;
        hex.push_back({std::cos(a), std::sin(a)});
    }
    std::vector<std::array<int, 3>> tris = cdt_triangulate(hex, {});

    MINI_CHECK(tris.size() == 4);
}

MINI_TEST("TrimeshCdt", "Polygon With Hole") {
    std::vector<std::pair<double,double>> border = {{0,0},{4,0},{4,4},{0,4}};
    std::vector<std::pair<double,double>> hole   = {{1,1},{1,3},{3,3},{3,1}};
    std::vector<std::array<int, 3>> tris = cdt_triangulate(border, {hole});

    std::vector<std::pair<double,double>> flat = {{0,0},{4,0},{4,4},{0,4},{1,1},{1,3},{3,3},{3,1}};
    double area = 0.0;
    for (auto& t : tris) area += tri_area(flat[t[0]], flat[t[1]], flat[t[2]]);

    MINI_CHECK(tris.size() > 0);
    MINI_CHECK(TOLERANCE.is_close(area, 4.0*4.0 - 2.0*2.0));
}

} // namespace session_cpp
