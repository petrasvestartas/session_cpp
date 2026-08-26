#include "mini_test.h"
#include "spatial_octree.h"
#include "point.h"
#include "tolerance.h"
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

static std::vector<Point> octree_test_points() {
    std::vector<Point> pts;
    for (int x = 0; x < 9; x++) {
        pts.push_back(Point(static_cast<double>(x), 0.0, 0.0));
    }
    return pts;
}

MINI_TEST("SpatialOctree", "Constructor") {
    // uncomment #include "spatial_octree.h"
    // uncomment #include "point.h"
    // SpatialOctree: per-node spacing-limited subsamples for LOD point rendering
    // leaf_capacity 16 > 9 points: the root absorbs everything, one node
    SpatialOctree tree(octree_test_points(), 4.0, 16);

    MINI_CHECK(tree.node_count() == 1);
    MINI_CHECK(tree.node_range(0) == std::make_pair(0, 9));
    MINI_CHECK(tree.order() == std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7, 8}));
}

MINI_TEST("SpatialOctree", "Node Count") {
    // 9 points on X: root cube size 8, spacing 4 -> 2 cells, first-wins accepts x=0
    // and x=4; the 7 leftovers split into two octants -> two leaf children
    SpatialOctree tree(octree_test_points(), 4.0, 4);

    MINI_CHECK(tree.node_count() == 3);
}

MINI_TEST("SpatialOctree", "Node Cube") {
    // Root cube: aabb (0..8, 0, 0) grown to a cube -> center (4,0,0), size 8.
    // Child in octant 6 (x<cx, y>=cy, z>=cz): min (0,0,0), size 4 -> center (2,2,2)
    SpatialOctree tree(octree_test_points(), 4.0, 4);
    auto [center, size] = tree.node_cube(0);
    auto [child_center, child_size] = tree.node_cube(1);

    MINI_CHECK(TOLERANCE.is_close(center[0], 4.0) && TOLERANCE.is_close(center[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(size, 8.0));
    MINI_CHECK(TOLERANCE.is_close(child_center[0], 2.0) && TOLERANCE.is_close(child_center[2], 2.0));
    MINI_CHECK(TOLERANCE.is_close(child_size, 4.0));
}

MINI_TEST("SpatialOctree", "Node Level") {
    SpatialOctree tree(octree_test_points(), 4.0, 4);

    MINI_CHECK(tree.node_level(0) == 0);
    MINI_CHECK(tree.node_level(1) == 1);
    MINI_CHECK(tree.node_level(2) == 1);
}

MINI_TEST("SpatialOctree", "Node Spacing") {
    // Spacing halves per level, like Potree
    SpatialOctree tree(octree_test_points(), 4.0, 4);

    MINI_CHECK(TOLERANCE.is_close(tree.node_spacing(0), 4.0));
    MINI_CHECK(TOLERANCE.is_close(tree.node_spacing(1), 2.0));
    MINI_CHECK(TOLERANCE.is_close(tree.node_spacing(2), 2.0));
}

MINI_TEST("SpatialOctree", "Node Range") {
    // Every node's points are contiguous in order(): root [0..2), children after
    SpatialOctree tree(octree_test_points(), 4.0, 4);

    MINI_CHECK(tree.node_range(0) == std::make_pair(0, 2));
    MINI_CHECK(tree.node_range(1) == std::make_pair(2, 3));
    MINI_CHECK(tree.node_range(2) == std::make_pair(5, 4));
}

MINI_TEST("SpatialOctree", "Children") {
    SpatialOctree tree(octree_test_points(), 4.0, 4);

    MINI_CHECK(tree.children(0) == std::vector<int>({1, 2}));
    MINI_CHECK(tree.children(1).empty());
}

MINI_TEST("SpatialOctree", "Order") {
    // Root's grid accepts x=0 and x=4 (first point wins its cell); the octant
    // leaves absorb the rest in input order
    SpatialOctree tree(octree_test_points(), 4.0, 4);

    MINI_CHECK(tree.order() == std::vector<int>({0, 4, 1, 2, 3, 5, 6, 7, 8}));
}

MINI_TEST("SpatialOctree", "From Coords") {
    // The flat-array constructor is the renderer's path (no per-point Point allocs)
    std::vector<double> coords;
    for (int x = 0; x < 9; x++) {
        coords.push_back(static_cast<double>(x));
        coords.push_back(0.0);
        coords.push_back(0.0);
    }
    SpatialOctree tree = SpatialOctree::from_coords(coords, 4.0, 4);

    MINI_CHECK(tree.node_count() == 3);
    MINI_CHECK(tree.order() == std::vector<int>({0, 4, 1, 2, 3, 5, 6, 7, 8}));
}

} // namespace session_cpp
