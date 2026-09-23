#include "spatial_bvh.h"
#include <algorithm>
#include <bit>
#include <cassert>
#include <cmath>
#include <limits>

namespace session_cpp {

/// Return t clamped to [0, 1] scaled to 10 bits.
static uint32_t quantize(double t) {
    return (uint32_t)(std::clamp(t, 0.0, 1.0) * 1023.0);
}

bool SpatialBVH::Node::is_leaf() const {
    return object_id != NULL_IDX;
}

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
SpatialBVH::SpatialBVH(double world_size) : name("my_bvh"), world_size(world_size) {}

SpatialBVH SpatialBVH::from_boxes(const std::vector<OBB>& bounding_boxes, double world_size) {

    SpatialBVH bvh(world_size);
    bvh.build(bounding_boxes);

    return bvh;
}

// ═══════════════════════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════════════════════
bool SpatialBVH::empty() const {
    return nodes.empty();
}

size_t SpatialBVH::size() const {
    return nodes.size();
}

// ═══════════════════════════════════════════════════════════════════════════
// Mutators
// ═══════════════════════════════════════════════════════════════════════════
void SpatialBVH::build(const std::vector<OBB>& bounding_boxes) {
    build_from_boxes(bounding_boxes.data(), bounding_boxes.size(), world_size);
}

void SpatialBVH::build_from_boxes(const OBB* boxes, size_t count, double ws) {

    std::vector<AABB> aabbs(count);

    for (size_t i = 0; i < count; i++)
        aabbs[i] = aabb_from_obb(boxes[i]);

    build_from_aabbs(aabbs.data(), count, ws);
}

void SpatialBVH::build_from_aabbs(const AABB* aabbs, size_t count, double ws) {

    world_size = ws;
    nodes.clear();

    const int n = (int)count;

    if (n == 0)
        return;

    const std::vector<std::pair<uint32_t, int>> codes = sorted_codes(aabbs, n);
    const int leaf = n - 1;

    nodes.resize(n - 1);

    for (const std::pair<uint32_t, int>& code : codes)
        nodes.push_back({aabbs[code.second], NULL_IDX, NULL_IDX, code.second});

    std::vector<std::pair<int, int>> order(n - 1);

    for (int i = 0; i < n - 1; i++) {
        const std::pair<int, int> range = determine_range(codes, i);
        const int split = find_split(codes, range.first, range.second);

        nodes[i].left = split == range.first ? leaf + split : split;
        nodes[i].right = split + 1 == range.second ? leaf + split + 1 : split + 1;
        order[i] = {range.second - range.first, i};
    }

    std::sort(order.begin(), order.end());

    for (const std::pair<int, int>& item : order) {
        Node& node = nodes[item.second];
        node.aabb = AABB::merge(nodes[node.left].aabb, nodes[node.right].aabb);
    }
}

void SpatialBVH::build_with_guids(const std::vector<std::pair<OBB, std::string>>& boxes_with_guids) {

    std::vector<OBB> bounding_boxes;
    object_guids.clear();

    for (const std::pair<OBB, std::string>& item : boxes_with_guids) {
        bounding_boxes.push_back(item.first);
        object_guids.push_back(item.second);
    }

    world_size = compute_world_size(bounding_boxes);
    build(bounding_boxes);
}

// ═══════════════════════════════════════════════════════════════════════════
// Queries
// ═══════════════════════════════════════════════════════════════════════════
std::tuple<std::vector<std::pair<int, int>>, std::vector<int>, int> SpatialBVH::check_all_collisions(const std::vector<OBB>& bounding_boxes) {

    std::vector<std::pair<int, int>> pairs;
    std::vector<bool> visited(bounding_boxes.size(), false);
    int total_checks = 0;

    for (int i = 0; i < (int)bounding_boxes.size(); i++) {
        const std::pair<std::vector<int>, int> found = find_collisions(i, bounding_boxes[i], bounding_boxes);
        total_checks += found.second;

        for (const int j : found.first) {
            if (j < i)
                continue;

            pairs.emplace_back(i, j);
            visited[i] = true;
            visited[j] = true;
        }
    }

    std::vector<int> colliding_indices;

    for (int i = 0; i < (int)visited.size(); i++)
        if (visited[i])
            colliding_indices.push_back(i);

    return {pairs, colliding_indices, total_checks};
}

std::vector<std::pair<std::string, std::string>> SpatialBVH::check_all_collisions_guids(const std::vector<OBB>& bounding_boxes) {

    const std::vector<std::pair<int, int>> pairs = std::get<0>(check_all_collisions(bounding_boxes));
    std::vector<std::pair<std::string, std::string>> guid_pairs;

    for (const std::pair<int, int>& pair : pairs)
        if (pair.first < (int)object_guids.size() && pair.second < (int)object_guids.size())
            guid_pairs.emplace_back(object_guids[pair.first], object_guids[pair.second]);

    return guid_pairs;
}

std::pair<std::vector<int>, int> SpatialBVH::find_collisions(int object_id, const OBB& query_bbox, const std::vector<OBB>& bounding_boxes) const {

    std::vector<int> collisions;
    int check_count = 0;
    const AABB query = aabb_from_obb(query_bbox);

    int stack[STACK_SIZE];
    int top = 0;

    if (!nodes.empty())
        stack[top++] = 0;

    while (top > 0) {
        const Node& node = nodes[stack[--top]];

        if (!node.aabb.intersects(query))
            continue;

        check_count++;

        if (node.is_leaf()) {
            const int id = node.object_id;

            if (id != object_id && id < (int)bounding_boxes.size() && query.intersects(aabb_from_obb(bounding_boxes[id])))
                collisions.push_back(id);

            continue;
        }

        assert(top + 2 <= STACK_SIZE);
        stack[top++] = node.left;
        stack[top++] = node.right;
    }

    return {collisions, check_count};
}

std::vector<int> SpatialBVH::query_aabb(const AABB& query) const {

    std::vector<int> hits;

    int stack[STACK_SIZE];
    int top = 0;

    if (!nodes.empty())
        stack[top++] = 0;

    while (top > 0) {
        const Node& node = nodes[stack[--top]];

        if (!node.aabb.intersects(query))
            continue;

        if (node.is_leaf()) {
            hits.push_back(node.object_id);
            continue;
        }

        assert(top + 2 <= STACK_SIZE);
        stack[top++] = node.left;
        stack[top++] = node.right;
    }

    return hits;
}

std::vector<int> SpatialBVH::query_aabb(const OBB& query) const {
    return query_aabb(aabb_from_obb(query));
}

std::vector<int> SpatialBVH::nearest_neighbors(int object_id, const std::vector<OBB>& bounding_boxes, double inflate) const {

    std::vector<int> result;

    if (object_id < 0 || object_id >= (int)bounding_boxes.size())
        return result;

    AABB query = aabb_from_obb(bounding_boxes[object_id]);
    query.hx *= inflate;
    query.hy *= inflate;
    query.hz *= inflate;

    for (const int id : query_aabb(query))
        if (id != object_id)
            result.push_back(id);

    return result;
}

bool SpatialBVH::ray_cast(const Point& origin, const Vector& direction, std::vector<int>& candidate_leaf_ids, bool) const {

    candidate_leaf_ids.clear();
    std::vector<std::pair<double, int>> found;

    int stack[STACK_SIZE];
    int top = 0;

    if (!nodes.empty())
        stack[top++] = 0;

    while (top > 0) {
        const Node& node = nodes[stack[--top]];
        const std::pair<double, double> span = ray_aabb(origin, direction, node.aabb);

        if (span.second < span.first || span.second < 0.0)
            continue;

        if (node.is_leaf()) {
            found.emplace_back(span.first, node.object_id);
            continue;
        }

        assert(top + 2 <= STACK_SIZE);
        stack[top++] = node.left;
        stack[top++] = node.right;
    }

    std::sort(found.begin(), found.end());

    for (const std::pair<double, int>& hit : found)
        candidate_leaf_ids.push_back(hit.second);

    return !candidate_leaf_ids.empty();
}

// ═══════════════════════════════════════════════════════════════════════════
// Boxes
// ═══════════════════════════════════════════════════════════════════════════
double SpatialBVH::compute_world_size(const std::vector<OBB>& bounding_boxes) {

    if (bounding_boxes.empty())
        return 1000.0;

    double max_extent = 0.0;

    for (const OBB& bbox : bounding_boxes)
        for (int k = 0; k < 3; k++)
            max_extent = std::max(max_extent, std::abs(bbox.center[k]) + bbox.half_size[k]);

    return std::max(max_extent * 2.2, 10.0);
}

OBB SpatialBVH::merge_aabb(const OBB& aabb1, const OBB& aabb2) const {
    return OBB::from_aabb(AABB::merge(aabb_from_obb(aabb1), aabb_from_obb(aabb2)));
}

bool SpatialBVH::aabb_intersect(const OBB& aabb1, const OBB& aabb2) const {
    return aabb_from_obb(aabb1).intersects(aabb_from_obb(aabb2));
}

bool SpatialBVH::aabb_intersect(const AABB& aabb1, const AABB& aabb2) const {
    return aabb1.intersects(aabb2);
}

// ═══════════════════════════════════════════════════════════════════════════
// Build
// ═══════════════════════════════════════════════════════════════════════════
std::vector<std::pair<uint32_t, int>> SpatialBVH::sorted_codes(const AABB* aabbs, int n) const {

    double lo[3];
    double hi[3];

    for (int k = 0; k < 3; k++) {
        lo[k] = center(aabbs[0], k);
        hi[k] = lo[k];
    }

    for (int i = 1; i < n; i++)
        for (int k = 0; k < 3; k++) {
            lo[k] = std::min(lo[k], center(aabbs[i], k));
            hi[k] = std::max(hi[k], center(aabbs[i], k));
        }

    const double ext = std::max(hi[0] - lo[0], std::max(hi[1] - lo[1], hi[2] - lo[2]));
    std::vector<std::pair<uint32_t, int>> codes(n);

    for (int i = 0; i < n; i++) {
        uint32_t code = 0;

        for (int k = 0; k < 3; k++) {
            const double t = ext > 0.0 ? (center(aabbs[i], k) - lo[k]) / ext : 0.0;
            code |= expand_bits(quantize(t)) << k;
        }

        codes[i] = {code, i};
    }

    std::sort(codes.begin(), codes.end());

    return codes;
}

int SpatialBVH::common_prefix(const std::vector<std::pair<uint32_t, int>>& codes, int i, int j) const {

    if (j < 0 || j >= (int)codes.size())
        return -1;

    if (codes[i].first != codes[j].first)
        return std::countl_zero(codes[i].first ^ codes[j].first);

    return 32 + std::countl_zero((uint32_t)i ^ (uint32_t)j);
}

std::pair<int, int> SpatialBVH::determine_range(const std::vector<std::pair<uint32_t, int>>& codes, int i) const {

    const int d = common_prefix(codes, i, i + 1) > common_prefix(codes, i, i - 1) ? 1 : -1;
    const int delta_min = common_prefix(codes, i, i - d);
    int length = 1;

    while (common_prefix(codes, i, i + length * d) > delta_min)
        length *= 2;

    int bound = 0;

    for (int step = length / 2; step > 0; step /= 2)
        if (common_prefix(codes, i, i + (bound + step) * d) > delta_min)
            bound += step;

    const int j = i + bound * d;

    return {std::min(i, j), std::max(i, j)};
}

int SpatialBVH::find_split(const std::vector<std::pair<uint32_t, int>>& codes, int first, int last) const {

    const int common = common_prefix(codes, first, last);
    int split = first;
    int step = last - first;

    while (step > 1) {
        step = (step + 1) / 2;

        if (split + step < last && common_prefix(codes, first, split + step) > common)
            split += step;
    }

    return split;
}

// ═══════════════════════════════════════════════════════════════════════════
// Traversal
// ═══════════════════════════════════════════════════════════════════════════
std::pair<double, double> SpatialBVH::ray_aabb(const Point& origin, const Vector& direction, const AABB& aabb) const {

    double tmin = -std::numeric_limits<double>::infinity();
    double tmax = std::numeric_limits<double>::infinity();

    for (int k = 0; k < 3; k++) {
        const double inv = direction[k] != 0.0 ? 1.0 / direction[k] : std::numeric_limits<double>::infinity();
        const double t1 = (center(aabb, k) - half(aabb, k) - origin[k]) * inv;
        const double t2 = (center(aabb, k) + half(aabb, k) - origin[k]) * inv;

        tmin = std::max(tmin, std::min(t1, t2));
        tmax = std::min(tmax, std::max(t1, t2));
    }

    return {tmin, tmax};
}

AABB SpatialBVH::aabb_from_obb(const OBB& obb) {

    double half[3];

    for (int k = 0; k < 3; k++)
        half[k] = std::abs(obb.x_axis[k]) * obb.half_size[0] + std::abs(obb.y_axis[k]) * obb.half_size[1] + std::abs(obb.z_axis[k]) * obb.half_size[2];

    return AABB(obb.center[0], obb.center[1], obb.center[2], half[0], half[1], half[2]);
}

double SpatialBVH::center(const AABB& aabb, int axis) const {

    if (axis == 0)
        return aabb.cx;

    if (axis == 1)
        return aabb.cy;

    return aabb.cz;
}

double SpatialBVH::half(const AABB& aabb, int axis) const {

    if (axis == 0)
        return aabb.hx;

    if (axis == 1)
        return aabb.hy;

    return aabb.hz;
}

// ═══════════════════════════════════════════════════════════════════════════
// Morton codes
// ═══════════════════════════════════════════════════════════════════════════
uint32_t expand_bits(uint32_t v) {

    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;

    return v;
}

uint32_t calculate_morton_code(double x, double y, double z, double world_size) {

    const double half = world_size * 0.5;
    const uint32_t ix = quantize((x + half) / world_size);
    const uint32_t iy = quantize((y + half) / world_size);
    const uint32_t iz = quantize((z + half) / world_size);

    return expand_bits(ix) | (expand_bits(iy) << 1) | (expand_bits(iz) << 2);
}

} // namespace session_cpp
