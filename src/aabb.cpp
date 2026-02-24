#include "aabb.h"
#include <algorithm>

namespace session_cpp {

void AABBTree::build(const BvhAABB* aabbs, size_t count) {
    nodes.clear();
    if (count == 0) return;
    nodes.reserve(2 * count - 1);
    std::vector<int> ids(count);
    for (size_t i = 0; i < count; i++) ids[i] = static_cast<int>(i);
    build_node(ids.data(), static_cast<int>(count), aabbs);
}

void AABBTree::build_node(int* ids, int count, const BvhAABB* aabbs) {
    int idx = static_cast<int>(nodes.size());
    nodes.push_back({{}, -1, -1});

    double lo_x = 1e308, lo_y = 1e308, lo_z = 1e308;
    double hi_x = -1e308, hi_y = -1e308, hi_z = -1e308;
    for (int i = 0; i < count; i++) {
        const auto& b = aabbs[ids[i]];
        lo_x = std::min(lo_x, b.cx - b.hx); hi_x = std::max(hi_x, b.cx + b.hx);
        lo_y = std::min(lo_y, b.cy - b.hy); hi_y = std::max(hi_y, b.cy + b.hy);
        lo_z = std::min(lo_z, b.cz - b.hz); hi_z = std::max(hi_z, b.cz + b.hz);
    }
    nodes[idx].aabb = {
        (lo_x + hi_x) * 0.5, (lo_y + hi_y) * 0.5, (lo_z + hi_z) * 0.5,
        (hi_x - lo_x) * 0.5, (hi_y - lo_y) * 0.5, (hi_z - lo_z) * 0.5
    };

    if (count == 1) {
        nodes[idx].object_id = ids[0];
        return;
    }

    double dx = hi_x - lo_x, dy = hi_y - lo_y, dz = hi_z - lo_z;
    int axis = (dx >= dy && dx >= dz) ? 0 : (dy >= dz) ? 1 : 2;
    int mid = count / 2;

    std::nth_element(ids, ids + mid, ids + count, [&](int a, int b) {
        const auto& ba = aabbs[a]; const auto& bb = aabbs[b];
        if (axis == 0) return ba.cx < bb.cx;
        if (axis == 1) return ba.cy < bb.cy;
        return ba.cz < bb.cz;
    });

    build_node(ids, mid, aabbs);
    nodes[idx].right = static_cast<int>(nodes.size());
    build_node(ids + mid, count - mid, aabbs);
}

}
