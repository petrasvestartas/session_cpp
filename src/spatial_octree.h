#pragma once

#include "point.h"
#include <array>
#include <utility>
#include <vector>

namespace session_cpp {

/// Potree-style LOD octree: every node keeps a spacing-limited subsample and order() makes each node's points contiguous.
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
    static const int MAX_LEVEL = 21;
    static const int STACK_SIZE = 8 * MAX_LEVEL;
    static const int NULL_IDX = -1;

    struct Node {
        std::array<double, 3> min;
        double size;
        int level;
        double spacing;
        int first;
        int count;
        std::array<int, 8> children;
    };

    struct Task {
        std::array<double, 3> min;
        double size;
        int level;
        double spacing;
        int lo;
        int hi;
        int parent;
        int octant;
    };

    std::vector<Node> _nodes;
    std::vector<int> _order;

    SpatialOctree() = default;
    std::pair<std::array<double, 3>, double> root_cube(const std::vector<double>& coords) const;
    void build(const std::vector<double>& coords, double root_spacing, int leaf_capacity);
    std::array<int, 9> accept(const std::vector<double>& coords, const Task& task, std::vector<int>& indices);
    void push(Task stack[], int& top, const Task& task) const;
};

} // namespace session_cpp
