#include "mini_test.h"
#include "aabb.h"
#include "closest.h"
#include "mesh.h"
#include "primitives.h"
#include "point.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("AABBTree", "Build_empty") {
    AABBTree tree;
    tree.build(nullptr, 0);
    MINI_CHECK(tree.empty());
}

MINI_TEST("AABBTree", "Build_single") {
    BvhAABB aabb = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    AABBTree tree;
    tree.build(&aabb, 1);
    MINI_CHECK(tree.size() == 1);
    MINI_CHECK(tree.nodes[0].object_id == 0);
}

MINI_TEST("AABBTree", "Build_multiple") {
    std::vector<BvhAABB> aabbs = {
        {0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        {5.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        {10.0, 0.0, 0.0, 1.0, 1.0, 1.0}
    };
    AABBTree tree;
    tree.build(aabbs.data(), aabbs.size());
    MINI_CHECK(tree.size() == 5);
    MINI_CHECK(tree.nodes[0].object_id == -1);
}

MINI_TEST("AABBTree", "Node_count") {
    std::vector<BvhAABB> aabbs;
    for (int i = 0; i < 100; i++) {
        aabbs.push_back({static_cast<double>(i), 0.0, 0.0, 0.5, 0.5, 0.5});
    }
    AABBTree tree;
    tree.build(aabbs.data(), aabbs.size());
    MINI_CHECK(tree.size() == 199);
}

MINI_TEST("AABBTree", "Mesh_point_aabb") {
    Mesh m = Primitives::cube(2.0);

    auto [cp1, fk1, d1] = Closest::mesh_point_aabb(m, Point(0.0, 0.0, 2.0));
    MINI_CHECK(TOLERANCE.is_close(cp1[2], 1.0));
    MINI_CHECK(TOLERANCE.is_close(d1, 1.0));

    auto [cp2, fk2, d2] = Closest::mesh_point_aabb(m, Point(1.0, 1.0, 1.0));
    MINI_CHECK(TOLERANCE.is_close(d2, 0.0));
}

MINI_TEST("AABBTree", "Mesh_point_aabb_matches_bvh") {
    Mesh m = Primitives::cube(2.0);
    Point tp(0.3, 0.7, 1.5);

    auto [cp_bvh, fk_bvh, d_bvh] = Closest::mesh_point(m, tp);
    auto [cp_aabb, fk_aabb, d_aabb] = Closest::mesh_point_aabb(m, tp);

    MINI_CHECK(TOLERANCE.is_close(d_bvh, d_aabb));
    MINI_CHECK(TOLERANCE.is_close(cp_bvh[0], cp_aabb[0]));
    MINI_CHECK(TOLERANCE.is_close(cp_bvh[1], cp_aabb[1]));
    MINI_CHECK(TOLERANCE.is_close(cp_bvh[2], cp_aabb[2]));
}

}
