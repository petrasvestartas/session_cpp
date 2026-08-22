#include "spatial_bvh.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <array>
#include <functional>

namespace session_cpp {

static AABB aabb_from_obb(const OBB& bb) {
    double hx = std::abs(bb.x_axis[0])*bb.half_size[0] + std::abs(bb.y_axis[0])*bb.half_size[1] + std::abs(bb.z_axis[0])*bb.half_size[2];
    double hy = std::abs(bb.x_axis[1])*bb.half_size[0] + std::abs(bb.y_axis[1])*bb.half_size[1] + std::abs(bb.z_axis[1])*bb.half_size[2];
    double hz = std::abs(bb.x_axis[2])*bb.half_size[0] + std::abs(bb.y_axis[2])*bb.half_size[1] + std::abs(bb.z_axis[2])*bb.half_size[2];
    return AABB{bb.center[0], bb.center[1], bb.center[2], hx, hy, hz};
}

bool SpatialBVH::aabb_intersect(const AABB& aabb1, const AABB& aabb2) const {
    double min1_x = aabb1.cx - aabb1.hx;
    double max1_x = aabb1.cx + aabb1.hx;
    double min1_y = aabb1.cy - aabb1.hy;
    double max1_y = aabb1.cy + aabb1.hy;
    double min1_z = aabb1.cz - aabb1.hz;
    double max1_z = aabb1.cz + aabb1.hz;

    double min2_x = aabb2.cx - aabb2.hx;
    double max2_x = aabb2.cx + aabb2.hx;
    double min2_y = aabb2.cy - aabb2.hy;
    double max2_y = aabb2.cy + aabb2.hy;
    double min2_z = aabb2.cz - aabb2.hz;
    double max2_z = aabb2.cz + aabb2.hz;

    return (min1_x <= max2_x && max1_x >= min2_x &&
            min1_y <= max2_y && max1_y >= min2_y &&
            min1_z <= max2_z && max1_z >= min2_z);
}

void SpatialBVH::build_from_aabbs(const AABB* aabbs, size_t count, double ws) {
    if (count == 0) {
        root = nullptr;
        node_arena.clear();
        return;
    }

    this->world_size = ws;

    // Morton codes normalized over the INPUT's own bounds - not the
    // origin-centered world_size. Sized by max |coordinate|, a scene far from the origin
    // collapses into a handful of Morton cells: the tree stays balanced (index tiebreak)
    // but loses all spatial coherence - measured 480x slower queries for the same boxes
    // moved 5 km out. Bounds normalization makes tree quality translation-invariant;
    // query results are unaffected (they test exact AABBs), world_size stays as metadata.
    double lo[3] = {aabbs[0].cx, aabbs[0].cy, aabbs[0].cz};
    double hi[3] = {aabbs[0].cx, aabbs[0].cy, aabbs[0].cz};
    for (size_t i = 1; i < count; ++i) {
        lo[0] = std::min(lo[0], aabbs[i].cx); hi[0] = std::max(hi[0], aabbs[i].cx);
        lo[1] = std::min(lo[1], aabbs[i].cy); hi[1] = std::max(hi[1], aabbs[i].cy);
        lo[2] = std::min(lo[2], aabbs[i].cz); hi[2] = std::max(hi[2], aabbs[i].cz);
    }
    // ONE scale for all three axes (the scene's bounding CUBE): per-axis stretch would
    // blow a nearly-flat axis up to the full 1024 cells and scatter xy-neighbours in the
    // sort - measured 4x slower queries on a sheet-like scene. Cubic cells keep the sort
    // spatially honest; a flat axis simply occupies few cells, which is the truth.
    double ext = std::max(hi[0] - lo[0], std::max(hi[1] - lo[1], hi[2] - lo[2]));
    double s = ext > 0.0 ? 1023.0 / ext : 0.0;
    auto q = [&](double c, int k) -> uint32_t {
        return std::min(static_cast<uint32_t>((c - lo[k]) * s), 1023u);
    };
    std::vector<ObjectInfo> objects;
    objects.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const auto& bb = aabbs[i];
        uint32_t morton_code = expand_bits(q(bb.cx, 0))
            | (expand_bits(q(bb.cy, 1)) << 1)
            | (expand_bits(q(bb.cz, 2)) << 2);
        objects.emplace_back(ObjectInfo{static_cast<int>(i), morton_code});
    }

    // Radix sort 30-bit Morton codes
    {
        auto radix_sort = [&](std::vector<ObjectInfo>& arr) {
            const int RADIX = 1024;
            const int PASSES = 3;
            std::vector<ObjectInfo> tmp(arr.size());
            for (int pass = 0; pass < PASSES; ++pass) {
                std::array<int, RADIX> count{};
                int shift = pass * 10;
                for (const auto& e : arr) {
                    int b = static_cast<int>((e.morton_code >> shift) & (RADIX - 1));
                    count[b]++;
                }
                int sum = 0;
                for (int i = 0; i < RADIX; ++i) {
                    int c = count[i];
                    count[i] = sum;
                    sum += c;
                }
                for (const auto& e : arr) {
                    int b = static_cast<int>((e.morton_code >> shift) & (RADIX - 1));
                    tmp[count[b]++] = e;
                }
                arr.swap(tmp);
            }
        };
        radix_sort(objects);
    }

    node_arena.clear();
    node_arena.reserve(static_cast<size_t>(2 * count - 1));

    const int N = static_cast<int>(count);
    if (N == 1) {
        SpatialBVHNode* leaf = alloc_node();
        leaf->object_id = objects[0].id;
        const auto& bb = aabbs[objects[0].id];
        leaf->aabb = AABB{bb.cx, bb.cy, bb.cz, bb.hx, bb.hy, bb.hz};
        leaf->left = leaf->right = nullptr;
        root = leaf;
        return;
    }

    std::vector<uint32_t> codes(N);
    for (int i = 0; i < N; ++i) codes[i] = objects[i].morton_code;

    auto clz32 = [](uint32_t x) -> int {
        if (x == 0) return 32;
#if defined(__clang__) || defined(__GNUC__)
        return __builtin_clz(x);
#else
        int n = 0; while ((x & 0x80000000u) == 0) { n++; x <<= 1; } return n;
#endif
    };

    auto commonPrefix = [&](int i, int j) -> int {
        if (j < 0 || j >= N) return -1;
        uint32_t ci = codes[i];
        uint32_t cj = codes[j];
        if (ci != cj) return clz32(ci ^ cj);
        uint32_t di = static_cast<uint32_t>(i);
        uint32_t dj = static_cast<uint32_t>(j);
        return 32 + clz32(di ^ dj);
    };

    auto determineRange = [&](int i, int& first, int& last) {
        int d = (commonPrefix(i, i + 1) - commonPrefix(i, i - 1)) > 0 ? 1 : -1;
        int deltaMin = commonPrefix(i, i - d);
        int l = 1;
        while (commonPrefix(i, i + l * d) > deltaMin) { l <<= 1; }
        int bound = 0;
        for (int t = l >> 1; t > 0; t >>= 1) {
            if (commonPrefix(i, i + (bound + t) * d) > deltaMin) bound += t;
        }
        int j = i + bound * d;
        first = std::min(i, j);
        last = std::max(i, j);
    };

    auto findSplit = [&](int first, int last) -> int {
        int common = commonPrefix(first, last);
        int split = first;
        int step = last - first;
        do {
            step = (step + 1) >> 1;
            int newSplit = split + step;
            if (newSplit < last) {
                int splitPrefix = commonPrefix(first, newSplit);
                if (splitPrefix > common) split = newSplit;
            }
        } while (step > 1);
        return split;
    };

    std::vector<SpatialBVHNode*> leaves(N);
    for (int i = 0; i < N; ++i) {
        SpatialBVHNode* leaf = alloc_node();
        leaf->object_id = objects[i].id;
        const auto& bb = aabbs[objects[i].id];
        leaf->aabb = AABB{bb.cx, bb.cy, bb.cz, bb.hx, bb.hy, bb.hz};
        leaf->left = leaf->right = nullptr;
        leaves[i] = leaf;
    }

    std::vector<SpatialBVHNode*> internals(N - 1);
    for (int i = 0; i < N - 1; ++i) {
        SpatialBVHNode* in = alloc_node();
        in->object_id = -1;
        in->left = in->right = nullptr;
        internals[i] = in;
    }

    std::vector<char> has_parent(N - 1, 0);
    for (int i = 0; i < N - 1; ++i) {
        int first, last;
        determineRange(i, first, last);
        int split = findSplit(first, last);
        if (split == first) { internals[i]->left = leaves[split]; }
        else { internals[i]->left = internals[split]; has_parent[split] = 1; }
        if (split + 1 == last) { internals[i]->right = leaves[split + 1]; }
        else { internals[i]->right = internals[split + 1]; has_parent[split + 1] = 1; }
    }

    int rootIdx = 0;
    for (int i = 0; i < N - 1; ++i) { if (!has_parent[i]) { rootIdx = i; break; } }
    root = internals[rootIdx];

    std::function<void(SpatialBVHNode*)> compute = [&](SpatialBVHNode* n) {
        if (!n || n->is_leaf()) return;
        compute(n->left);
        compute(n->right);
        const auto& a = n->left->aabb;
        const auto& b = n->right->aabb;
        double min_x = std::min(a.cx - a.hx, b.cx - b.hx);
        double min_y = std::min(a.cy - a.hy, b.cy - b.hy);
        double min_z = std::min(a.cz - a.hz, b.cz - b.hz);
        double max_x = std::max(a.cx + a.hx, b.cx + b.hx);
        double max_y = std::max(a.cy + a.hy, b.cy + b.hy);
        double max_z = std::max(a.cz + a.hz, b.cz + b.hz);
        n->aabb = AABB{(min_x + max_x) * 0.5,
                          (min_y + max_y) * 0.5,
                          (min_z + max_z) * 0.5,
                          (max_x - min_x) * 0.5,
                          (max_y - min_y) * 0.5,
                          (max_z - min_z) * 0.5};
    };
    compute(root);
}

std::vector<int> SpatialBVH::query_aabb(const AABB& query) const {
    std::vector<int> hits;
    if (!root) return hits;
    std::vector<const SpatialBVHNode*> stack;
    stack.reserve(64);
    stack.push_back(root);
    while (!stack.empty()) {
        const SpatialBVHNode* node = stack.back(); stack.pop_back();
        if (!aabb_intersect(node->aabb, query)) continue;
        if (node->is_leaf()) {
            hits.push_back(node->object_id);
        } else {
            if (node->left)  stack.push_back(node->left);
            if (node->right) stack.push_back(node->right);
        }
    }
    return hits;
}

std::vector<int> SpatialBVH::query_aabb(const OBB& query) const {
    return query_aabb(aabb_from_obb(query));
}

void SpatialBVH::build_with_guids(const std::vector<std::pair<OBB, std::string>>& boxes_with_guids) {
    if (boxes_with_guids.empty()) {
        object_guids.clear();
        build({});
        return;
    }

    std::vector<OBB> bounding_boxes;
    bounding_boxes.reserve(boxes_with_guids.size());
    object_guids.clear();
    object_guids.reserve(boxes_with_guids.size());
    for (const auto& [bbox, id] : boxes_with_guids) {
        bounding_boxes.push_back(bbox);
        object_guids.push_back(id);
    }

    world_size = compute_world_size(bounding_boxes);
    build(bounding_boxes);
}

std::vector<std::pair<std::string, std::string>> SpatialBVH::check_all_collisions_guids(const std::vector<OBB>& bounding_boxes) {
    auto [collision_pairs, colliding_indices, checks] = check_all_collisions(bounding_boxes);

    std::vector<std::pair<std::string, std::string>> guid_collisions;
    for (const auto& [i, j] : collision_pairs) {
        if (i < (int)object_guids.size() && j < (int)object_guids.size()) {
            guid_collisions.emplace_back(object_guids[i], object_guids[j]);
        }
    }
    return guid_collisions;
}

std::pair<std::vector<int>, int> SpatialBVH::find_collisions(int object_id, const OBB& query_bbox, const std::vector<OBB>& bounding_boxes) const {
    std::vector<int> collisions;
    int check_count = 0;

    if (!root) {
        return {collisions, check_count};
    }

    AABB query = aabb_from_obb(query_bbox);
    std::vector<const SpatialBVHNode*> stack;
    stack.reserve(64);
    stack.push_back(root);

    while (!stack.empty()) {
        const SpatialBVHNode* node = stack.back(); stack.pop_back();

        // Early exit if query doesn't intersect this node's AABB
        if (!aabb_intersect(query, node->aabb)) continue;
        check_count++;

        // If leaf node, check for collision
        if (node->is_leaf()) {
            int node_object_id = node->object_id;
            // Don't check collision with self
            if (node_object_id != object_id
                && node_object_id < (int)bounding_boxes.size()
                && aabb_intersect(query, aabb_from_obb(bounding_boxes[node_object_id]))) {
                collisions.push_back(node_object_id);
            }
            continue;
        }

        // Internal node: push children
        if (node->left)  stack.push_back(node->left);
        if (node->right) stack.push_back(node->right);
    }

    return {collisions, check_count};
}

std::vector<int> SpatialBVH::nearest_neighbors(int object_id,
                                        const std::vector<OBB>& bounding_boxes,
                                        double inflate) const {
    std::vector<int> result;
    if (object_id < 0 || static_cast<size_t>(object_id) >= bounding_boxes.size()) return result;
    AABB query = aabb_from_obb(bounding_boxes[object_id]);
    query.hx *= inflate;
    query.hy *= inflate;
    query.hz *= inflate;
    std::vector<int> hits = query_aabb(query);
    result.reserve(hits.size());
    for (int id : hits) {
        if (id != object_id) result.push_back(id);
    }
    return result;
}

// Overload for lightweight internal SpatialBVH AABB
static bool ray_aabb_intersect(const Point& origin,
                               const Vector& direction,
                               const AABB& box,
                               double& tmin_out,
                               double& tmax_out)
{
    double min_x = box.cx - box.hx;
    double max_x = box.cx + box.hx;
    double min_y = box.cy - box.hy;
    double max_y = box.cy + box.hy;
    double min_z = box.cz - box.hz;
    double max_z = box.cz + box.hz;

    auto inv = [](double v) {
        return (v != 0.0) ? (1.0 / v) : std::numeric_limits<double>::infinity();
    };

    double invx = inv(direction[0]);
    double invy = inv(direction[1]);
    double invz = inv(direction[2]);

    double tx1 = (min_x - origin[0]) * invx;
    double tx2 = (max_x - origin[0]) * invx;
    double tmin = std::min(tx1, tx2);
    double tmax = std::max(tx1, tx2);

    double ty1 = (min_y - origin[1]) * invy;
    double ty2 = (max_y - origin[1]) * invy;
    tmin = std::max(tmin, std::min(ty1, ty2));
    tmax = std::min(tmax, std::max(ty1, ty2));

    double tz1 = (min_z - origin[2]) * invz;
    double tz2 = (max_z - origin[2]) * invz;
    tmin = std::max(tmin, std::min(tz1, tz2));
    tmax = std::min(tmax, std::max(tz1, tz2));

    tmin_out = tmin;
    tmax_out = tmax;
    return tmax >= tmin;
}

// SpatialBVHNode implementation
SpatialBVHNode::SpatialBVHNode() : left(nullptr), right(nullptr), object_id(-1) {}

bool SpatialBVHNode::is_leaf() const {
    return object_id != -1;
}

// Morton code functions
uint32_t expand_bits(uint32_t v) {
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

uint32_t calculate_morton_code(double x, double y, double z, double world_size) {
    // Normalize coordinates to [0,1] range
    const double inv_world = 1.0 / world_size;
    const double half_world = world_size * 0.5;
    double nx = (x + half_world) * inv_world;
    double ny = (y + half_world) * inv_world;
    double nz = (z + half_world) * inv_world;

    // Clamp to [0,1] and scale to [0, 1023] in one step
    uint32_t ix = std::min(static_cast<uint32_t>(std::max(0.0, std::min(1.0, nx)) * 1023.0), 1023u);
    uint32_t iy = std::min(static_cast<uint32_t>(std::max(0.0, std::min(1.0, ny)) * 1023.0), 1023u);
    uint32_t iz = std::min(static_cast<uint32_t>(std::max(0.0, std::min(1.0, nz)) * 1023.0), 1023u);

    // Inline expand_bits for all three coordinates (avoid 15k function calls)
    ix = (ix * 0x00010001u) & 0xFF0000FFu;
    ix = (ix * 0x00000101u) & 0x0F00F00Fu;
    ix = (ix * 0x00000011u) & 0xC30C30C3u;
    ix = (ix * 0x00000005u) & 0x49249249u;
    
    iy = (iy * 0x00010001u) & 0xFF0000FFu;
    iy = (iy * 0x00000101u) & 0x0F00F00Fu;
    iy = (iy * 0x00000011u) & 0xC30C30C3u;
    iy = (iy * 0x00000005u) & 0x49249249u;
    
    iz = (iz * 0x00010001u) & 0xFF0000FFu;
    iz = (iz * 0x00000101u) & 0x0F00F00Fu;
    iz = (iz * 0x00000011u) & 0xC30C30C3u;
    iz = (iz * 0x00000005u) & 0x49249249u;

    return ix | (iy << 1) | (iz << 2);
}

// SpatialBVH implementation
SpatialBVH::SpatialBVH(double world_size) : name("my_bvh"), root(nullptr), world_size(world_size) {}

double SpatialBVH::compute_world_size(const std::vector<OBB>& bounding_boxes) {
    if (bounding_boxes.empty()) {
        return 1000.0;
    }
    
    double max_extent = 0.0;
    for (const auto& bbox : bounding_boxes) {
        // Find maximum absolute coordinate in any dimension
        double x_extent = std::max(std::abs(bbox.center[0] + bbox.half_size[0]), 
                                   std::abs(bbox.center[0] - bbox.half_size[0]));
        double y_extent = std::max(std::abs(bbox.center[1] + bbox.half_size[1]), 
                                   std::abs(bbox.center[1] - bbox.half_size[1]));
        double z_extent = std::max(std::abs(bbox.center[2] + bbox.half_size[2]), 
                                   std::abs(bbox.center[2] - bbox.half_size[2]));
        
        max_extent = std::max({max_extent, x_extent, y_extent, z_extent});
    }
    
    // World size should be at least 2x the maximum extent, plus padding
    return std::max(max_extent * 2.2, 10.0);
}

SpatialBVH SpatialBVH::from_boxes(const std::vector<OBB>& bounding_boxes, double world_size) {
    SpatialBVH bvh(world_size);
    bvh.build_from_boxes(bounding_boxes.data(), bounding_boxes.size(), world_size);
    return bvh;
}

void SpatialBVH::build(const std::vector<OBB>& bounding_boxes) {
    build_from_boxes(bounding_boxes.data(), bounding_boxes.size(), world_size);
}

void SpatialBVH::build_from_boxes(const OBB* boxes, size_t count, double ws) {
    if (count == 0) {
        root = nullptr;
        node_arena.clear();
        return;
    }

    this->world_size = ws;

    // Morton codes normalized over the INPUT's own bounds - not the
    // origin-centered world_size. Sized by max |coordinate|, a scene far from the origin
    // collapses into a handful of Morton cells: the tree stays balanced (index tiebreak)
    // but loses all spatial coherence - measured 480x slower queries for the same boxes
    // moved 5 km out. Bounds normalization makes tree quality translation-invariant;
    // query results are unaffected (they test exact AABBs), world_size stays as metadata.
    double lo[3] = {boxes[0].center[0], boxes[0].center[1], boxes[0].center[2]};
    double hi[3] = {boxes[0].center[0], boxes[0].center[1], boxes[0].center[2]};
    for (size_t i = 1; i < count; ++i) {
        for (int k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], boxes[i].center[k]);
            hi[k] = std::max(hi[k], boxes[i].center[k]);
        }
    }
    // ONE scale for all three axes (the scene's bounding CUBE): per-axis stretch would
    // blow a nearly-flat axis up to the full 1024 cells and scatter xy-neighbours in the
    // sort - measured 4x slower queries on a sheet-like scene. Cubic cells keep the sort
    // spatially honest; a flat axis simply occupies few cells, which is the truth.
    double ext = std::max(hi[0] - lo[0], std::max(hi[1] - lo[1], hi[2] - lo[2]));
    double s = ext > 0.0 ? 1023.0 / ext : 0.0;
    auto q = [&](double c, int k) -> uint32_t {
        return std::min(static_cast<uint32_t>((c - lo[k]) * s), 1023u);
    };
    std::vector<ObjectInfo> objects;
    objects.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const auto& bbox = boxes[i];
        uint32_t morton_code = expand_bits(q(bbox.center[0], 0))
            | (expand_bits(q(bbox.center[1], 1)) << 1)
            | (expand_bits(q(bbox.center[2], 2)) << 2);
        objects.emplace_back(ObjectInfo{static_cast<int>(i), morton_code});
    }

    // Radix sort by 30-bit Morton code (3 passes of 10 bits) for speed
    {
        auto radix_sort = [&](std::vector<ObjectInfo>& arr) {
            const int RADIX = 1024; // 2^10
            const int PASSES = 3;   // 30 bits total
            std::vector<ObjectInfo> tmp(arr.size());
            for (int pass = 0; pass < PASSES; ++pass) {
                std::array<int, RADIX> count{};
                int shift = pass * 10;
                for (const auto& e : arr) {
                    int b = static_cast<int>((e.morton_code >> shift) & (RADIX - 1));
                    count[b]++;
                }
                // Prefix sums
                int sum = 0;
                for (int i = 0; i < RADIX; ++i) {
                    int c = count[i];
                    count[i] = sum;
                    sum += c;
                }
                // Stable scatter
                for (const auto& e : arr) {
                    int b = static_cast<int>((e.morton_code >> shift) & (RADIX - 1));
                    tmp[count[b]++] = e;
                }
                arr.swap(tmp);
            }
        };
        radix_sort(objects);
    }

    // Prepare node arena: 2*n - 1 nodes total
    node_arena.clear();
    node_arena.reserve(static_cast<size_t>(2 * count - 1));

    // LBVH (Karras 2012) O(N) construction
    const int N = static_cast<int>(count);
    if (N == 1) {
        // Single leaf
        SpatialBVHNode* leaf = alloc_node();
        leaf->object_id = objects[0].id;
        const OBB& bb = boxes[objects[0].id];
        leaf->aabb = aabb_from_obb(bb);
        leaf->left = leaf->right = nullptr;
        root = leaf;
        return;
    }

    std::vector<uint32_t> codes(N);
    for (int i = 0; i < N; ++i) codes[i] = objects[i].morton_code;

    auto clz32 = [](uint32_t x) -> int {
        if (x == 0) return 32;
#if defined(__clang__) || defined(__GNUC__)
        return __builtin_clz(x);
#else
        // Fallback
        int n = 0; while ((x & 0x80000000u) == 0) { n++; x <<= 1; } return n;
#endif
    };

    auto commonPrefix = [&](int i, int j) -> int {
        if (j < 0 || j >= N) return -1; // out of range
        uint32_t ci = codes[i];
        uint32_t cj = codes[j];
        if (ci != cj) return clz32(ci ^ cj);
        // Disambiguate equal Morton codes using index bits
        uint32_t di = static_cast<uint32_t>(i);
        uint32_t dj = static_cast<uint32_t>(j);
        return 32 + clz32(di ^ dj);
    };

    auto determineRange = [&](int i, int& first, int& last) {
        int d = (commonPrefix(i, i + 1) - commonPrefix(i, i - 1)) > 0 ? 1 : -1;
        int deltaMin = commonPrefix(i, i - d);

        // Find lmax by exponential search
        int l = 1;
        while (commonPrefix(i, i + l * d) > deltaMin) {
            l <<= 1;
        }

        // Binary search to find exact bound
        int bound = 0;
        for (int t = l >> 1; t > 0; t >>= 1) {
            if (commonPrefix(i, i + (bound + t) * d) > deltaMin) bound += t;
        }
        int j = i + bound * d;
        first = std::min(i, j);
        last = std::max(i, j);
    };

    auto findSplit = [&](int first, int last) -> int {
        int common = commonPrefix(first, last);
        int split = first;
        int step = last - first;
        do {
            step = (step + 1) >> 1; // ceil div 2
            int newSplit = split + step;
            if (newSplit < last) {
                int splitPrefix = commonPrefix(first, newSplit);
                if (splitPrefix > common) split = newSplit;
            }
        } while (step > 1);
        return split;
    };

    // Allocate leaves
    std::vector<SpatialBVHNode*> leaves(N);
    for (int i = 0; i < N; ++i) {
        SpatialBVHNode* leaf = alloc_node();
        leaf->object_id = objects[i].id;
        const OBB& bb = boxes[objects[i].id];
        leaf->aabb = aabb_from_obb(bb);
        leaf->left = leaf->right = nullptr;
        leaves[i] = leaf;
    }

    // Allocate internal nodes
    std::vector<SpatialBVHNode*> internals(N - 1);
    for (int i = 0; i < N - 1; ++i) {
        SpatialBVHNode* in = alloc_node();
        in->object_id = -1;
        in->left = in->right = nullptr;
        internals[i] = in;
    }

    // Build topology
    std::vector<char> has_parent(N - 1, 0);
    for (int i = 0; i < N - 1; ++i) {
        int first, last;
        determineRange(i, first, last);
        int split = findSplit(first, last);

        // Left child
        if (split == first) {
            internals[i]->left = leaves[split];
        } else {
            internals[i]->left = internals[split];
            has_parent[split] = 1;
        }

        // Right child
        if (split + 1 == last) {
            internals[i]->right = leaves[split + 1];
        } else {
            internals[i]->right = internals[split + 1];
            has_parent[split + 1] = 1;
        }
    }

    // Find root (internal node without parent)
    int rootIdx = 0;
    for (int i = 0; i < N - 1; ++i) {
        if (!has_parent[i]) { rootIdx = i; break; }
    }
    root = internals[rootIdx];

    // Post-order compute internal AABBs
    std::function<void(SpatialBVHNode*)> compute = [&](SpatialBVHNode* n) {
        if (!n || n->is_leaf()) return;
        compute(n->left);
        compute(n->right);
        const auto& a = n->left->aabb;
        const auto& b = n->right->aabb;
        double min_x = std::min(a.cx - a.hx, b.cx - b.hx);
        double min_y = std::min(a.cy - a.hy, b.cy - b.hy);
        double min_z = std::min(a.cz - a.hz, b.cz - b.hz);
        double max_x = std::max(a.cx + a.hx, b.cx + b.hx);
        double max_y = std::max(a.cy + a.hy, b.cy + b.hy);
        double max_z = std::max(a.cz + a.hz, b.cz + b.hz);
        n->aabb = AABB{(min_x + max_x) * 0.5,
                          (min_y + max_y) * 0.5,
                          (min_z + max_z) * 0.5,
                          (max_x - min_x) * 0.5,
                          (max_y - min_y) * 0.5,
                          (max_z - min_z) * 0.5};
    };
    compute(root);
}

SpatialBVHNode* SpatialBVH::alloc_node() {
    node_arena.emplace_back();
    return &node_arena.back();
}

OBB SpatialBVH::merge_aabb(const OBB& aabb1, const OBB& aabb2) {
    // Cache center and half_size components (avoid repeated method calls)
    double c1x = aabb1.center[0], c1y = aabb1.center[1], c1z = aabb1.center[2];
    double h1x = aabb1.half_size[0], h1y = aabb1.half_size[1], h1z = aabb1.half_size[2];
    double c2x = aabb2.center[0], c2y = aabb2.center[1], c2z = aabb2.center[2];
    double h2x = aabb2.half_size[0], h2y = aabb2.half_size[1], h2z = aabb2.half_size[2];
    
    // Calculate min and max corners
    double min_x = std::min(c1x - h1x, c2x - h2x);
    double min_y = std::min(c1y - h1y, c2y - h2y);
    double min_z = std::min(c1z - h1z, c2z - h2z);
    
    double max_x = std::max(c1x + h1x, c2x + h2x);
    double max_y = std::max(c1y + h1y, c2y + h2y);
    double max_z = std::max(c1z + h1z, c2z + h2z);

    // Calculate new center and half_size in one go
    return OBB(
        Point((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5),
        Vector(1, 0, 0),
        Vector(0, 1, 0),
        Vector(0, 0, 1),
        Vector((max_x - min_x) * 0.5, (max_y - min_y) * 0.5, (max_z - min_z) * 0.5)
    );
}

bool SpatialBVH::aabb_intersect(const OBB& aabb1, const OBB& aabb2) const {
    double min1_x = aabb1.center[0] - aabb1.half_size[0];
    double max1_x = aabb1.center[0] + aabb1.half_size[0];
    double min1_y = aabb1.center[1] - aabb1.half_size[1];
    double max1_y = aabb1.center[1] + aabb1.half_size[1];
    double min1_z = aabb1.center[2] - aabb1.half_size[2];
    double max1_z = aabb1.center[2] + aabb1.half_size[2];

    double min2_x = aabb2.center[0] - aabb2.half_size[0];
    double max2_x = aabb2.center[0] + aabb2.half_size[0];
    double min2_y = aabb2.center[1] - aabb2.half_size[1];
    double max2_y = aabb2.center[1] + aabb2.half_size[1];
    double min2_z = aabb2.center[2] - aabb2.half_size[2];
    double max2_z = aabb2.center[2] + aabb2.half_size[2];

    return (min1_x <= max2_x && max1_x >= min2_x &&
            min1_y <= max2_y && max1_y >= min2_y &&
            min1_z <= max2_z && max1_z >= min2_z);
}

std::tuple<std::vector<std::pair<int, int>>, std::vector<int>, int> SpatialBVH::check_all_collisions(const std::vector<OBB>& bounding_boxes) {
    std::vector<std::pair<int, int>> all_collisions;
    all_collisions.reserve(bounding_boxes.size());
    std::vector<char> visited(bounding_boxes.size(), 0);
    int total_checks = 0;

    if (!root) {
        return {all_collisions, {}, total_checks};
    }

    // Pair traversal stack (canonicalized by pointer address to avoid duplicate work)
    std::vector<std::pair<SpatialBVHNode*, SpatialBVHNode*>> stack;
    stack.reserve(64);
    stack.emplace_back(root, root);

    auto push_pair = [&](SpatialBVHNode* a, SpatialBVHNode* b) {
        if (!a || !b) return;
        // Canonicalize order
        if (a > b) std::swap(a, b);
        stack.emplace_back(a, b);
    };

    while (!stack.empty()) {
        auto [a, b] = stack.back();
        stack.pop_back();
        if (!a || !b) continue;

        // AABB overlap test
        if (!aabb_intersect(a->aabb, b->aabb)) continue;
        total_checks++;

        bool aLeaf = a->is_leaf();
        bool bLeaf = b->is_leaf();

        if (aLeaf && bLeaf) {
            int i = a->object_id;
            int j = b->object_id;
            if (i >= 0 && j >= 0 && i != j) {
                if (i > j) std::swap(i, j);
                all_collisions.emplace_back(i, j);
                if (static_cast<size_t>(i) < visited.size()) visited[static_cast<size_t>(i)] = 1;
                if (static_cast<size_t>(j) < visited.size()) visited[static_cast<size_t>(j)] = 1;
            }
            continue;
        }

        if (a == b) {
            // Same node: expand unique child pairs without symmetry duplicates
            if (!aLeaf) {
                if (a->left)  push_pair(a->left,  a->left);
                if (a->left && a->right) push_pair(a->left, a->right);
                if (a->right) push_pair(a->right, a->right);
            }
            continue;
        }

        if (!aLeaf && !bLeaf) {
            push_pair(a->left,  b->left);
            push_pair(a->left,  b->right);
            push_pair(a->right, b->left);
            push_pair(a->right, b->right);
        } else if (aLeaf && !bLeaf) {
            push_pair(a, b->left);
            push_pair(a, b->right);
        } else if (!aLeaf && bLeaf) {
            push_pair(a->left,  b);
            push_pair(a->right, b);
        }
    }

    std::vector<int> colliding_indices;
    colliding_indices.reserve(bounding_boxes.size());
    for (size_t idx = 0; idx < visited.size(); ++idx) {
        if (visited[idx]) colliding_indices.push_back(static_cast<int>(idx));
    }
    return {all_collisions, colliding_indices, total_checks};
}

bool SpatialBVH::ray_cast(const Point& origin,
                   const Vector& direction,
                   std::vector<int>& candidate_leaf_ids,
                   bool find_all) const
{
    candidate_leaf_ids.clear();
    if (!root) return false;

    struct HeapItem {
        SpatialBVHNode* node;
        double tmin;
        
        bool operator>(const HeapItem& other) const {
            return tmin > other.tmin; // min-heap (smallest tmin at top)
        }
    };

    std::priority_queue<HeapItem, std::vector<HeapItem>, std::greater<HeapItem>> heap;

    double rtmin, rtmax;
    if (!ray_aabb_intersect(origin, direction, root->aabb, rtmin, rtmax)) {
        return false;
    }
    
    // Prune boxes entirely behind ray origin
    if (rtmax < 0.0) return false;
    
    heap.push({root, rtmin});

    bool any = false;
    while (!heap.empty()) {
        HeapItem item = heap.top();
        heap.pop();

        auto node = item.node;
        if (!node) continue;

        if (node->is_leaf()) {
            candidate_leaf_ids.push_back(node->object_id);
            any = true;
            if (!find_all && candidate_leaf_ids.size() >= 1) {
                // Early exit for find_first, but keep traversing for near-first ordering
            }
            continue;
        }

        // Intersect children and push (prune boxes behind ray)
        if (node->left) {
            double cmin, cmax;
            if (ray_aabb_intersect(origin, direction, node->left->aabb, cmin, cmax) && cmax >= 0.0) {
                heap.push({node->left, cmin});
            }
        }
        if (node->right) {
            double cmin, cmax;
            if (ray_aabb_intersect(origin, direction, node->right->aabb, cmin, cmax) && cmax >= 0.0) {
                heap.push({node->right, cmin});
            }
        }
    }

    return any;
}

}
