#pragma once

#include "aabb.h"
#include <vector>

namespace session_cpp {

/// Flat AABB tree with longest-axis median split; the left child of node i is i + 1, the right child is stored.
class SpatialAABBTree {
public:
    struct Node {
        AABB aabb;
        /// Right child index, NULL_IDX on a leaf
        int right;
        /// Primitive id on a leaf, NULL_IDX on an internal node
        int object_id;
    };

    std::vector<Node> nodes;

    SpatialAABBTree() = default;

    bool empty() const;
    size_t size() const;

    void build(const AABB* aabbs, size_t count);

    /// Ids of every leaf box that intersects query
    std::vector<int> query_aabb(const AABB& query) const;

private:
    static const int STACK_SIZE = 64;
    static const int NULL_IDX = -1;

    struct Range {
        int lo;
        int hi;
        int parent;
        bool is_left;
    };

    AABB bounds(const std::vector<int>& ids, int lo, int hi, const AABB* aabbs) const;
    int longest_axis(const AABB& aabb) const;
    double center(const AABB& aabb, int axis) const;
};

} // namespace session_cpp
