#include "mini_test.h"
#include "aabb.h"
#include "spatial_aabbtree.h"
#include "closest.h"
#include "mesh.h"
#include "point.h"
#include "primitives.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("SpatialAABBTree", "Constructor") {
    // uncomment #include "aabb.h"
    // uncomment #include "closest.h"
    // SpatialAABBTree: O(n log n) build, O(log n) cull — prune candidates before exact test
    std::vector<AABB> boxes = {
        AABB(0.0, 0.0, 0.0, 0.5, 0.5, 0.5),
        AABB(5.0, 0.0, 0.0, 0.5, 0.5, 0.5),
        AABB(10.0, 0.0, 0.0, 0.5, 0.5, 0.5),
    };
    auto pairs = Closest::boxes_closest(boxes, 0.0);

    MINI_CHECK(pairs.empty());

    std::vector<AABB> boxes_near = {
        AABB(0.0, 0.0, 0.0, 0.5, 0.5, 0.5),
        AABB(1.0, 0.0, 0.0, 0.5, 0.5, 0.5),
    };
    auto pairs_near = Closest::boxes_closest(boxes_near, 0.0);

    MINI_CHECK(pairs_near.size() == 1);
    MINI_CHECK(pairs_near[0].first == 0);
    MINI_CHECK(pairs_near[0].second == 1);
}

MINI_TEST("SpatialAABBTree", "Build Empty") {
    // uncomment #include "spatial_aabbtree.h"
    SpatialAABBTree tree;
    tree.build(nullptr, 0);

    MINI_CHECK(tree.empty());
}

MINI_TEST("SpatialAABBTree", "Build Single") {
    // uncomment #include "spatial_aabbtree.h"
    AABB aabb = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    SpatialAABBTree tree;
    tree.build(&aabb, 1);

    MINI_CHECK(tree.size() == 1);
    MINI_CHECK(tree.nodes[0].object_id == 0);
}

MINI_TEST("SpatialAABBTree", "Build Multiple") {
    // uncomment #include "spatial_aabbtree.h"
    std::vector<AABB> aabbs = {
        {0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        {5.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        {10.0, 0.0, 0.0, 1.0, 1.0, 1.0}
    };
    SpatialAABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    MINI_CHECK(tree.size() == 5);
    MINI_CHECK(tree.nodes[0].object_id == -1);
}

MINI_TEST("SpatialAABBTree", "Node Count") {
    // uncomment #include "spatial_aabbtree.h"
    std::vector<AABB> aabbs;
    for (int i = 0; i < 100; i++) {
        aabbs.push_back({static_cast<double>(i), 0.0, 0.0, 0.5, 0.5, 0.5});
    }
    SpatialAABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    MINI_CHECK(tree.size() == 199);
}

MINI_TEST("SpatialAABBTree", "Mesh Point Aabb") {
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

MINI_TEST("SpatialAABBTree", "Mesh Point Aabb Matches Bvh") {
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

MINI_TEST("SpatialAABBTree", "Query Aabb") {
    // uncomment #include "spatial_aabbtree.h"
    std::vector<AABB> aabbs = {
        {0.0, 0.0, 0.0, 0.5, 0.5, 0.5},
        {5.0, 0.0, 0.0, 0.5, 0.5, 0.5},
        {10.0, 0.0, 0.0, 0.5, 0.5, 0.5},
    };
    SpatialAABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    std::vector<int> hits = tree.query_aabb(AABB(0.0, 0.0, 0.0, 1.0, 1.0, 1.0));

    MINI_CHECK(hits.size() == 1);
    MINI_CHECK(hits[0] == 0);

    std::vector<int> none = tree.query_aabb(AABB(20.0, 0.0, 0.0, 0.5, 0.5, 0.5));

    MINI_CHECK(none.empty());

    std::vector<int> all = tree.query_aabb(AABB(5.0, 0.0, 0.0, 10.0, 1.0, 1.0));

    MINI_CHECK(all.size() == 3);
}

}
