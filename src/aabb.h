#pragma once

#include "bvh.h"
#include <vector>

namespace session_cpp {

class AABBTree {
public:
    struct Node {
        BvhAABB aabb;
        int right;      // right child index; left child = this_index + 1
        int object_id;  // leaf: primitive id (>=0), internal: -1
    };

    std::vector<Node> nodes;

    AABBTree() = default;
    void build(const BvhAABB* aabbs, size_t count);
    bool empty() const { return nodes.empty(); }
    size_t size() const { return nodes.size(); }

private:
    void build_node(int* ids, int count, const BvhAABB* aabbs);
};

}
