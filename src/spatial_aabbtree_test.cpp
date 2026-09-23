#include "mini_test.h"
#include "aabb.h"
#include "spatial_aabbtree.h"
#include "closest.h"
#include "mesh.h"
#include "point.h"
#include "primitives.h"
#include "tolerance.h"
#include <tuple>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("SpatialAABBTree", "Constructor") {

    const std::vector<AABB> boxes = {
        AABB(0.0, 0.0, 0.0, 0.5, 0.5, 0.5),
        AABB(5.0, 0.0, 0.0, 0.5, 0.5, 0.5),
        AABB(10.0, 0.0, 0.0, 0.5, 0.5, 0.5),
    };
    const std::vector<std::pair<size_t, size_t>> pairs = Closest::boxes_closest(boxes, 0.0);

    MINI_CHECK(pairs.empty());

    const std::vector<AABB> boxes_near = {
        AABB(0.0, 0.0, 0.0, 0.5, 0.5, 0.5),
        AABB(1.0, 0.0, 0.0, 0.5, 0.5, 0.5),
    };
    const std::vector<std::pair<size_t, size_t>> pairs_near = Closest::boxes_closest(boxes_near, 0.0);

    MINI_CHECK(pairs_near.size() == 1);
    MINI_CHECK(pairs_near[0].first == 0);
    MINI_CHECK(pairs_near[0].second == 1);
}

MINI_TEST("SpatialAABBTree", "Build Empty") {

    SpatialAABBTree tree;
    tree.build(nullptr, 0);

    MINI_CHECK(tree.empty());
}

MINI_TEST("SpatialAABBTree", "Build Single") {

    const AABB aabb(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

    SpatialAABBTree tree;
    tree.build(&aabb, 1);

    MINI_CHECK(tree.size() == 1);
    MINI_CHECK(tree.nodes[0].object_id == 0);
}

MINI_TEST("SpatialAABBTree", "Build Multiple") {

    const std::vector<AABB> aabbs = {
        AABB(0.0, 0.0, 0.0, 1.0, 1.0, 1.0),
        AABB(5.0, 0.0, 0.0, 1.0, 1.0, 1.0),
        AABB(10.0, 0.0, 0.0, 1.0, 1.0, 1.0),
    };

    SpatialAABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    MINI_CHECK(tree.size() == 5);
    MINI_CHECK(tree.nodes[0].object_id == -1);
}

MINI_TEST("SpatialAABBTree", "Node Count") {

    std::vector<AABB> aabbs;

    for (int i = 0; i < 100; i++)
        aabbs.push_back(AABB(static_cast<double>(i), 0.0, 0.0, 0.5, 0.5, 0.5));

    SpatialAABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    MINI_CHECK(tree.size() == 199);
}

MINI_TEST("SpatialAABBTree", "Mesh Point Aabb") {

    const Mesh m = Primitives::cube(2.0);

    Point cp1;
    double d1 = 0.0;
    std::tie(cp1, std::ignore, d1) = Closest::mesh_point_aabb(m, Point(0.0, 0.0, 2.0));

    MINI_CHECK(TOLERANCE.is_close(cp1[2], 1.0));
    MINI_CHECK(TOLERANCE.is_close(d1, 1.0));

    const double d2 = std::get<2>(Closest::mesh_point_aabb(m, Point(1.0, 1.0, 1.0)));

    MINI_CHECK(TOLERANCE.is_close(d2, 0.0));
}

MINI_TEST("SpatialAABBTree", "Mesh Point Aabb Matches Bvh") {

    const Mesh m = Primitives::cube(2.0);
    const Point tp(0.3, 0.7, 1.5);

    Point cp_bvh;
    double d_bvh = 0.0;
    std::tie(cp_bvh, std::ignore, d_bvh) = Closest::mesh_point(m, tp);

    Point cp_aabb;
    double d_aabb = 0.0;
    std::tie(cp_aabb, std::ignore, d_aabb) = Closest::mesh_point_aabb(m, tp);

    MINI_CHECK(TOLERANCE.is_close(d_bvh, d_aabb));
    MINI_CHECK(TOLERANCE.is_close(cp_bvh[0], cp_aabb[0]));
    MINI_CHECK(TOLERANCE.is_close(cp_bvh[1], cp_aabb[1]));
    MINI_CHECK(TOLERANCE.is_close(cp_bvh[2], cp_aabb[2]));
}

MINI_TEST("SpatialAABBTree", "Query Aabb") {

    const std::vector<AABB> aabbs = {
        AABB(0.0, 0.0, 0.0, 0.5, 0.5, 0.5),
        AABB(5.0, 0.0, 0.0, 0.5, 0.5, 0.5),
        AABB(10.0, 0.0, 0.0, 0.5, 0.5, 0.5),
    };

    SpatialAABBTree tree;
    tree.build(aabbs.data(), aabbs.size());

    const std::vector<int> hits = tree.query_aabb(AABB(0.0, 0.0, 0.0, 1.0, 1.0, 1.0));

    MINI_CHECK(hits.size() == 1);
    MINI_CHECK(hits[0] == 0);

    const std::vector<int> none = tree.query_aabb(AABB(20.0, 0.0, 0.0, 0.5, 0.5, 0.5));

    MINI_CHECK(none.empty());

    const std::vector<int> all = tree.query_aabb(AABB(5.0, 0.0, 0.0, 10.0, 1.0, 1.0));

    MINI_CHECK(all.size() == 3);
}

} // namespace session_cpp
