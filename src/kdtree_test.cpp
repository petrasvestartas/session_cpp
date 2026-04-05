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
    // Simple deterministic random using LCG
    uint64_t seed = 42;
    auto rng = [&]() -> double {
        seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(seed >> 33) / static_cast<double>(0xFFFFFFFFU) * 20.0 - 10.0;
    };
    std::vector<Point> pts;
    for (int i = 0; i < 100; ++i) pts.push_back(Point(rng(), rng(), rng()));
    KDTree tree(pts);
    Point query(0.0, 0.0, 0.0);
    auto [idx, dist] = tree.nearest(query);
    int brute_idx = 0;
    for (int i = 1; i < (int)pts.size(); ++i) {
        if (pts[i][0]*pts[i][0]+pts[i][1]*pts[i][1]+pts[i][2]*pts[i][2] < pts[brute_idx][0]*pts[brute_idx][0]+pts[brute_idx][1]*pts[brute_idx][1]+pts[brute_idx][2]*pts[brute_idx][2])
            brute_idx = i;
    }
    double brute_dist = std::sqrt(pts[brute_idx][0]*pts[brute_idx][0]+pts[brute_idx][1]*pts[brute_idx][1]+pts[brute_idx][2]*pts[brute_idx][2]);

    MINI_CHECK(idx == brute_idx);
    MINI_CHECK(TOLERANCE.is_close(dist, brute_dist));
}

MINI_TEST("KDTree", "Nearest K") {
    // uncomment #include "kdtree.h"
    // uncomment #include "point.h"
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
    uint64_t seed = 7;
    auto rng = [&]() -> double {
        seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(seed >> 33) / static_cast<double>(0xFFFFFFFFU) * 100.0;
    };
    std::vector<Point> pts;
    for (int i = 0; i < 50; ++i) pts.push_back(Point(rng(), rng(), rng()));
    std::vector<Point> queries;
    for (int i = 0; i < 10; ++i) queries.push_back(Point(rng(), rng(), rng()));
    KDTree tree(pts);
    bool all_match = true;
    for (const auto& q : queries) {
        auto [idx, dist] = tree.nearest(q);
        int brute = 0;
        for (int i = 1; i < (int)pts.size(); ++i) {
            double da = (pts[i][0]-q[0])*(pts[i][0]-q[0])+(pts[i][1]-q[1])*(pts[i][1]-q[1])+(pts[i][2]-q[2])*(pts[i][2]-q[2]);
            double db = (pts[brute][0]-q[0])*(pts[brute][0]-q[0])+(pts[brute][1]-q[1])*(pts[brute][1]-q[1])+(pts[brute][2]-q[2])*(pts[brute][2]-q[2]);
            if (da < db) brute = i;
        }
        double brute_d = std::sqrt((pts[brute][0]-q[0])*(pts[brute][0]-q[0])+(pts[brute][1]-q[1])*(pts[brute][1]-q[1])+(pts[brute][2]-q[2])*(pts[brute][2]-q[2]));
        if (!TOLERANCE.is_close(dist, brute_d)) all_match = false;
    }

    MINI_CHECK(all_match);
}

} // namespace session_cpp
