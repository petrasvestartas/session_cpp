#include "mini_test.h"
#include "spatial_octree.h"
#include "point.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("SpatialOctree", "Constructor") {

    std::vector<Point> pts;

    for (int x = 0; x < 9; x++)
        pts.push_back(Point((double)x, 0.0, 0.0));

    const SpatialOctree tree(pts, 4.0, 16);

    MINI_CHECK(tree.node_count() == 1);
    MINI_CHECK(tree.node_range(0) == std::make_pair(0, 9));
    MINI_CHECK(tree.order() == std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7, 8}));
}

MINI_TEST("SpatialOctree", "Node Count") {

    std::vector<Point> pts;

    for (int x = 0; x < 9; x++)
        pts.push_back(Point((double)x, 0.0, 0.0));

    const SpatialOctree tree(pts, 4.0, 4);

    MINI_CHECK(tree.node_count() == 3);
}

MINI_TEST("SpatialOctree", "Node Cube") {

    std::vector<Point> pts;

    for (int x = 0; x < 9; x++)
        pts.push_back(Point((double)x, 0.0, 0.0));

    const SpatialOctree tree(pts, 4.0, 4);
    const std::pair<Point, double> cube = tree.node_cube(0);
    const std::pair<Point, double> child = tree.node_cube(1);

    MINI_CHECK(TOLERANCE.is_close(cube.first[0], 4.0) && TOLERANCE.is_close(cube.first[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(cube.second, 8.0));
    MINI_CHECK(TOLERANCE.is_close(child.first[0], 2.0) && TOLERANCE.is_close(child.first[2], 2.0));
    MINI_CHECK(TOLERANCE.is_close(child.second, 4.0));
}

MINI_TEST("SpatialOctree", "Node Level") {

    std::vector<Point> pts;

    for (int x = 0; x < 9; x++)
        pts.push_back(Point((double)x, 0.0, 0.0));

    const SpatialOctree tree(pts, 4.0, 4);

    MINI_CHECK(tree.node_level(0) == 0);
    MINI_CHECK(tree.node_level(1) == 1);
    MINI_CHECK(tree.node_level(2) == 1);
}

MINI_TEST("SpatialOctree", "Node Spacing") {

    std::vector<Point> pts;

    for (int x = 0; x < 9; x++)
        pts.push_back(Point((double)x, 0.0, 0.0));

    const SpatialOctree tree(pts, 4.0, 4);

    MINI_CHECK(TOLERANCE.is_close(tree.node_spacing(0), 4.0));
    MINI_CHECK(TOLERANCE.is_close(tree.node_spacing(1), 2.0));
    MINI_CHECK(TOLERANCE.is_close(tree.node_spacing(2), 2.0));
}

MINI_TEST("SpatialOctree", "Node Range") {

    std::vector<Point> pts;

    for (int x = 0; x < 9; x++)
        pts.push_back(Point((double)x, 0.0, 0.0));

    const SpatialOctree tree(pts, 4.0, 4);

    MINI_CHECK(tree.node_range(0) == std::make_pair(0, 2));
    MINI_CHECK(tree.node_range(1) == std::make_pair(2, 3));
    MINI_CHECK(tree.node_range(2) == std::make_pair(5, 4));
}

MINI_TEST("SpatialOctree", "Children") {

    std::vector<Point> pts;

    for (int x = 0; x < 9; x++)
        pts.push_back(Point((double)x, 0.0, 0.0));

    const SpatialOctree tree(pts, 4.0, 4);

    MINI_CHECK(tree.children(0) == std::vector<int>({1, 2}));
    MINI_CHECK(tree.children(1).empty());
}

MINI_TEST("SpatialOctree", "Order") {

    std::vector<Point> pts;

    for (int x = 0; x < 9; x++)
        pts.push_back(Point((double)x, 0.0, 0.0));

    const SpatialOctree tree(pts, 4.0, 4);

    MINI_CHECK(tree.order() == std::vector<int>({0, 4, 1, 2, 3, 5, 6, 7, 8}));
}

MINI_TEST("SpatialOctree", "From Coords") {

    std::vector<double> coords;

    for (int x = 0; x < 9; x++) {
        coords.push_back((double)x);
        coords.push_back(0.0);
        coords.push_back(0.0);
    }

    const SpatialOctree tree = SpatialOctree::from_coords(coords, 4.0, 4);

    MINI_CHECK(tree.node_count() == 3);
    MINI_CHECK(tree.order() == std::vector<int>({0, 4, 1, 2, 3, 5, 6, 7, 8}));
}

} // namespace session_cpp
