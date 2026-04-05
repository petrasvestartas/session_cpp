#include "mini_test.h"
#include "aabb.h"
#include "closest.h"
#include "mesh.h"
#include "primitives.h"
#include "point.h"
#include "tolerance.h"
#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("AABBTree", "Build Empty") {
    // uncomment #include "aabb.h"
    AABBTree tree;
    tree.build(nullptr, 0);

    MINI_CHECK(tree.empty());
}

MINI_TEST("AABBTree", "Build Single") {
    // uncomment #include "aabb.h"
    AABB aabb = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    AABBTree tree;
    tree.build(&aabb, 1);

    MINI_CHECK(tree.size() == 1);
    MINI_CHECK(tree.nodes[0].object_id == 0);
}

MINI_TEST("AABBTree", "Build Multiple") {
    // uncomment #include "aabb.h"
    std::vector<AABB> aabbs = {
        {0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        {5.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        {10.0, 0.0, 0.0, 1.0, 1.0, 1.0}
    };
    AABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    MINI_CHECK(tree.size() == 5);
    MINI_CHECK(tree.nodes[0].object_id == -1);
}

MINI_TEST("AABBTree", "Node Count") {
    // uncomment #include "aabb.h"
    std::vector<AABB> aabbs;
    for (int i = 0; i < 100; i++) {
        aabbs.push_back({static_cast<double>(i), 0.0, 0.0, 0.5, 0.5, 0.5});
    }
    AABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    MINI_CHECK(tree.size() == 199);
}

MINI_TEST("AABBTree", "Mesh Point Aabb") {
    // uncomment #include "aabb.h"
    // uncomment #include "closest.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    // uncomment #include "primitives.h"
    Mesh m = Primitives::cube(2.0);

    auto [cp1, fk1, d1] = Closest::mesh_point_aabb(m, Point(0.0, 0.0, 2.0));

    MINI_CHECK(TOLERANCE.is_close(cp1[2], 1.0));
    MINI_CHECK(TOLERANCE.is_close(d1, 1.0));

    auto [cp2, fk2, d2] = Closest::mesh_point_aabb(m, Point(1.0, 1.0, 1.0));
    MINI_CHECK(TOLERANCE.is_close(d2, 0.0));
}

MINI_TEST("AABBTree", "Mesh Point Aabb Matches Bvh") {
    // uncomment #include "aabb.h"
    // uncomment #include "closest.h"
    // uncomment #include "mesh.h"
    // uncomment #include "point.h"
    // uncomment #include "primitives.h"
    Mesh m = Primitives::cube(2.0);
    Point tp(0.3, 0.7, 1.5);

    auto [cp_bvh, fk_bvh, d_bvh] = Closest::mesh_point(m, tp);
    auto [cp_aabb, fk_aabb, d_aabb] = Closest::mesh_point_aabb(m, tp);

    MINI_CHECK(TOLERANCE.is_close(d_bvh, d_aabb));
    MINI_CHECK(TOLERANCE.is_close(cp_bvh[0], cp_aabb[0]));
    MINI_CHECK(TOLERANCE.is_close(cp_bvh[1], cp_aabb[1]));
    MINI_CHECK(TOLERANCE.is_close(cp_bvh[2], cp_aabb[2]));
}

MINI_TEST("Aabb", "Constructor") {
    // uncomment #include "aabb.h"
    // uncomment #include "point.h"
    // AABB(0,0,0, 1,2,3) — dims 2×4×6
    AABB a(0.0, 0.0, 0.0, 1.0, 2.0, 3.0);

    MINI_CHECK(TOLERANCE.is_close(a.area(), 88.0));
    MINI_CHECK(a.center() == Point(0.0, 0.0, 0.0));
    MINI_CHECK(TOLERANCE.is_close(a.diagonal(), 2.0 * std::sqrt(14.0)));
    MINI_CHECK(a.is_valid());
    MINI_CHECK(TOLERANCE.is_close(a.volume(), 48.0));

    MINI_CHECK(a.closest_point(Point(0.0, 0.0, 0.0)) == Point(0.0, 0.0, 0.0));
    MINI_CHECK(a.closest_point(Point(10.0, 0.0, 0.0)) == Point(1.0, 0.0, 0.0));
    MINI_CHECK(a.contains(Point(0.0, 0.0, 0.0)));
    MINI_CHECK(!a.contains(Point(10.0, 0.0, 0.0)));

    MINI_CHECK(a.corner(false, false, false) == Point(-1.0, -2.0, -3.0));
    MINI_CHECK(a.corner(true, true, true) == Point(1.0, 2.0, 3.0));
    MINI_CHECK(a.get_corners().size() == 8);
    MINI_CHECK(a.get_edges().size() == 12);

    MINI_CHECK(a.point_at(1.0, 0.0, 0.0) == Point(1.0, 0.0, 0.0));
    MINI_CHECK(a.point_at(0.0, 0.0, 0.0) == Point(0.0, 0.0, 0.0));

    AABB b(5.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    a.union_with(b);
    MINI_CHECK(a.min_point() == Point(-1.0, -2.0, -3.0));
    MINI_CHECK(a.max_point() == Point(6.0, 2.0, 3.0));
}

}
