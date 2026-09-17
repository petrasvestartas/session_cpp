#pragma once

#include "aabb.h"
#include <vector>

namespace session_cpp {

/// Flat AABB tree with longest-axis median split; the left child of node i is i + 1, the right child is stored.
class SpatialAABBTree {
public:
    /// Tree node.
    struct Node {
        AABB aabb; // Bounds of the subtree.
        int right; // Right child index, NULL_IDX on a leaf.
        int object_id; // Primitive id on a leaf, NULL_IDX on an internal node.
    };

    std::vector<Node> nodes; // Nodes in depth-first order.

    /// Construct an empty tree.
    SpatialAABBTree() = default;

    /// Return whether the tree has no nodes.
    bool empty() const;

    /// Return the node count.
    size_t size() const;

    /// Build the tree over count boxes, one leaf per box.
    void build(const AABB* aabbs, size_t count);

    /// Return the ids of every leaf box that intersects query.
    std::vector<int> query_aabb(const AABB& query) const;

private:
    static const int STACK_SIZE = 64; // Depth bound of the explicit traversal stack.
    static const int NULL_IDX = -1; // Index of a missing child or object.

    /// Pending id range of the build stack.
    struct Range {
        int lo; // First id of the range.
        int hi; // One past the last id of the range.
        int parent; // Parent node index, NULL_IDX at the root.
        bool is_left; // Whether the range is the left child of parent.
    };

    /// Return the box enclosing ids[lo, hi).
    AABB bounds(const std::vector<int>& ids, int lo, int hi, const AABB* aabbs) const;

    /// Return the axis of the largest half-size.
    int longest_axis(const AABB& aabb) const;

    /// Return the center coordinate of aabb along axis.
    double center(const AABB& aabb, int axis) const;
};

} // namespace session_cpp
