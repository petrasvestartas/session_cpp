// SpatialOctree — Potree-style multi-resolution LOD octree over bare 3D points.
// Use for: level-of-detail point-cloud rendering. Each node owns a spacing-limited
//   SUBSAMPLE of the points (grid accept, first point wins, leftovers descend into
//   octants at half the spacing), so drawing shallow nodes far away and deep nodes up
//   close gives Potree's uniform on-screen density. `order()` is the permutation that
//   makes every node's points CONTIGUOUS - upload points in that order and a node is
//   one (first, count) range.
// Prefer over SpatialKDTree when the question is "which points at what density",
//   not "which point is nearest".
// Note: static structure; rebuild required after point insertion.
#pragma once
#include "point.h"
#include <array>
#include <utility>
#include <vector>

namespace session_cpp {

/**
 * @class SpatialOctree
 * @brief LOD octree: per-node spacing-limited subsamples over a reordered point set.
 *
 * Build on construction; the root cube is the bounding box grown to a cube.
 * Complements SpatialKDTree (nearest queries) and SpatialRTree (box queries).
 */
class SpatialOctree {
public:
    SpatialOctree(std::vector<Point> points, double root_spacing, int leaf_capacity);
    static SpatialOctree from_coords(const std::vector<double>& coords, double root_spacing, int leaf_capacity);

    int node_count() const;
    std::pair<Point, double> node_cube(int i) const;
    int node_level(int i) const;
    double node_spacing(int i) const;
    std::pair<int, int> node_range(int i) const;
    std::vector<int> children(int i) const;
    const std::vector<int>& order() const;

private:
    struct Node {
        std::array<double, 3> min;
        double size;
        int level;
        double spacing;
        int first;
        int count;
        std::array<int, 8> children;
    };

    std::vector<Node> _nodes;
    std::vector<int> _order;

    SpatialOctree() = default;
    void init(const std::vector<double>& coords, double root_spacing, int leaf_capacity);
    int build(const std::vector<double>& coords, const std::array<double, 3>& min, double size, int level, double spacing, const std::vector<int>& idxs, int leaf_capacity);
};

} // namespace session_cpp
