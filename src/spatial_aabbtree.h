// SpatialAABBTree — flat contiguous SpatialBVH over axis-aligned boxes (SAH median split).
// Use for: closest-point on static mesh faces, ray-mesh intersection.
//   Build once, query many times. Cache-friendly 56-byte nodes.
// Prefer over SpatialBVH  when geometry is static and all volumes are world-aligned.
// Prefer over SpatialRTree when no dynamic insert/delete is needed.
// Prefer over SpatialKDTree when querying faces/volumes, not bare point clouds.
#pragma once

#include "aabb.h"
#include <vector>

namespace session_cpp {

class SpatialAABBTree {
public:
    struct Node {
        AABB aabb;
        int right;      // right child index; left child = this_index + 1
        int object_id;  // leaf: primitive id (>=0), internal: -1
    };

    std::vector<Node> nodes;

    SpatialAABBTree() = default;
    void build(const AABB* aabbs, size_t count);
    bool empty() const { return nodes.empty(); }
    size_t size() const { return nodes.size(); }

    std::vector<int> query_aabb(const AABB& query) const;

private:
    void build_node(int* ids, int count, const AABB* aabbs);
};

}
