#include "spatial_aabbtree.h"
#include <algorithm>
#include <cassert>

namespace session_cpp {

bool SpatialAABBTree::empty() const {
    return nodes.empty();
}

size_t SpatialAABBTree::size() const {
    return nodes.size();
}

void SpatialAABBTree::build(const AABB* aabbs, size_t count) {

    nodes.clear();
    const int n = (int)count;
    std::vector<int> ids(n);

    for (int i = 0; i < n; i++)
        ids[i] = i;

    nodes.reserve(2 * n);
    Range stack[STACK_SIZE];
    int top = 0;

    if (n > 0)
        stack[top++] = {0, n, NULL_IDX, false};

    while (top > 0) {
        const Range range = stack[--top];
        const int node = (int)nodes.size();
        const AABB aabb = bounds(ids, range.lo, range.hi, aabbs);
        nodes.push_back({aabb, NULL_IDX, NULL_IDX});

        if (range.parent != NULL_IDX && !range.is_left)
            nodes[range.parent].right = node;

        if (range.hi - range.lo == 1) {
            nodes[node].object_id = ids[range.lo];
            continue;
        }

        const int axis = longest_axis(nodes[node].aabb);
        const int mid = range.lo + (range.hi - range.lo) / 2;
        std::nth_element(ids.begin() + range.lo, ids.begin() + mid, ids.begin() + range.hi, [&](int a, int b) {
            return center(aabbs[a], axis) < center(aabbs[b], axis);
        });
        assert(top + 2 <= STACK_SIZE);
        stack[top++] = {mid, range.hi, node, false};
        stack[top++] = {range.lo, mid, node, true};
    }
}

std::vector<int> SpatialAABBTree::query_aabb(const AABB& query) const {

    std::vector<int> hits;
    int stack[STACK_SIZE];
    int top = 0;

    if (!nodes.empty())
        stack[top++] = 0;

    while (top > 0) {
        const int idx = stack[--top];
        const Node& node = nodes[idx];

        if (!node.aabb.intersects(query))
            continue;

        if (node.object_id != NULL_IDX) {
            hits.push_back(node.object_id);
            continue;
        }

        assert(top + 2 <= STACK_SIZE);
        stack[top++] = idx + 1;
        stack[top++] = node.right;
    }

    return hits;
}

AABB SpatialAABBTree::bounds(const std::vector<int>& ids, int lo, int hi, const AABB* aabbs) const {

    AABB aabb = aabbs[ids[lo]];

    for (int i = lo + 1; i < hi; i++)
        aabb = AABB::merge(aabb, aabbs[ids[i]]);

    return aabb;
}

int SpatialAABBTree::longest_axis(const AABB& aabb) const {

    if (aabb.hx >= aabb.hy && aabb.hx >= aabb.hz)
        return 0;

    if (aabb.hy >= aabb.hz)
        return 1;

    return 2;
}

double SpatialAABBTree::center(const AABB& aabb, int axis) const {

    if (axis == 0)
        return aabb.cx;

    if (axis == 1)
        return aabb.cy;

    return aabb.cz;
}

} // namespace session_cpp
