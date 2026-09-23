#pragma once

#include "point.h"
#include <array>
#include <utility>
#include <vector>

namespace session_cpp {

/// Potree-style LOD octree: every node keeps a spacing-limited subsample and order() makes each node's points contiguous.
class SpatialOctree {
private:
    static const int MAX_LEVEL = 21; // Deepest subdivision level.
    static const int STACK_SIZE = 8 * MAX_LEVEL; // Explicit stack depth, 8 children per level.
    static const int NULL_IDX = -1; // Missing child marker.

    /// A cube node with its subsample range and children.
    struct Node {
        std::array<double, 3> min; // Cube min corner.
        double size; // Cube edge length.
        int level; // Depth from the root.
        double spacing; // Grid-accept spacing.
        int first; // First point index into order.
        int count; // Point count in order.
        std::array<int, 8> children; // Child node per octant or NULL_IDX.
    };

    /// A pending cube on the build stack with its index range.
    struct Task {
        std::array<double, 3> min; // Cube min corner.
        double size; // Cube edge length.
        int level; // Depth from the root.
        double spacing; // Grid-accept spacing.
        int lo; // Index range start.
        int hi; // Index range end, exclusive.
        int parent; // Parent node or NULL_IDX.
        int octant; // Octant of the parent this task fills.
    };

    std::vector<Node> _nodes; // Nodes in build order, root first.
    std::vector<int> _order; // Point indices permuted so each node's points are contiguous.

public:
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct the tree over points.
    SpatialOctree(const std::vector<Point>& points, double root_spacing, int leaf_capacity);

    /// Construct the tree over flat [x, y, z, ...] coordinates.
    static SpatialOctree from_coords(const std::vector<double>& coords, double root_spacing, int leaf_capacity);

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the number of nodes.
    int node_count() const;

    /// Return the node cube center and edge length.
    std::pair<Point, double> node_cube(int i) const;

    /// Return the node depth from the root.
    int node_level(int i) const;

    /// Return the grid-accept spacing of a node.
    double node_spacing(int i) const;

    /// Return the node point range as (first, count) into order.
    std::pair<int, int> node_range(int i) const;

    /// Return the present child node indices.
    std::vector<int> children(int i) const;

    /// Return the point indices permuted so each node's points are contiguous.
    const std::vector<int>& order() const;

private:
    // ═══════════════════════════════════════════════════════════════════════════
    // Build
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty tree for from_coords.
    SpatialOctree() = default;

    /// Return the min corner and edge length of the cube bounding coords.
    std::pair<std::array<double, 3>, double> root_cube(const std::vector<double>& coords) const;

    /// Build the nodes by iterative subdivision over an explicit stack.
    void build(const std::vector<double>& coords, double root_spacing, int leaf_capacity);

    /// Keep one point per spacing cell in the node, bucket the rest by octant and return the 9 octant bounds.
    std::array<int, 9> accept(const std::vector<double>& coords, const Task& task, std::vector<int>& indices);

    /// Push a task onto the build stack.
    void push(Task stack[], int& top, const Task& task) const;
};

} // namespace session_cpp
