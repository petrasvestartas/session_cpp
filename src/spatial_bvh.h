#pragma once

#include "aabb.h"
#include "guid.h"
#include "obb.h"
#include "point.h"
#include "vector.h"
#include <cstdint>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace session_cpp {

/// Linear BVH (Karras 2012): leaves in Morton order, internal node i splits the sorted range it covers, node 0 is the root.
class SpatialBVH {
public:
    struct Node {
        AABB aabb;
        /// Left child index, NULL_IDX on a leaf
        int left = NULL_IDX;
        /// Right child index, NULL_IDX on a leaf
        int right = NULL_IDX;
        /// Object id on a leaf, NULL_IDX on an internal node
        int object_id = NULL_IDX;

        bool is_leaf() const;
    };

    bool has_guid() const { return !_guid.empty(); }
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string name;
    double world_size;
    std::vector<std::string> object_guids;
    std::vector<Node> nodes;

    SpatialBVH(double world_size = 1000.0);
    static SpatialBVH from_boxes(const std::vector<OBB>& bounding_boxes, double world_size);

    bool empty() const;
    size_t size() const;

    /// Largest absolute box coordinate times 2.2, at least 10
    static double compute_world_size(const std::vector<OBB>& bounding_boxes);

    void build(const std::vector<OBB>& bounding_boxes);
    void build_from_boxes(const OBB* boxes, size_t count, double ws);
    void build_from_aabbs(const AABB* aabbs, size_t count, double ws);

    /// Boxes paired with their guids, world size computed from the boxes
    void build_with_guids(const std::vector<std::pair<OBB, std::string>>& boxes_with_guids);

    /// Overlapping (i, j) pairs with i < j, the ids in any pair, and the number of nodes tested
    std::tuple<std::vector<std::pair<int, int>>, std::vector<int>, int> check_all_collisions(const std::vector<OBB>& bounding_boxes);
    std::vector<std::pair<std::string, std::string>> check_all_collisions_guids(const std::vector<OBB>& bounding_boxes);

    /// Ids overlapping query_bbox other than object_id, and the number of nodes tested
    std::pair<std::vector<int>, int> find_collisions(int object_id, const OBB& query_bbox, const std::vector<OBB>& bounding_boxes) const;

    /// Ids of every leaf box that intersects query
    std::vector<int> query_aabb(const AABB& query) const;
    std::vector<int> query_aabb(const OBB& query) const;

    /// Ids overlapping the box of object_id with its half-sizes scaled by inflate, object_id excluded
    std::vector<int> nearest_neighbors(int object_id, const std::vector<OBB>& bounding_boxes, double inflate = 1.2) const;

    /// Leaf ids whose box the ray enters, nearest entry first; true when any
    bool ray_cast(const Point& origin, const Vector& direction, std::vector<int>& candidate_leaf_ids, bool find_all = false) const;

    /// Axis-aligned box enclosing both boxes
    OBB merge_aabb(const OBB& aabb1, const OBB& aabb2) const;
    bool aabb_intersect(const OBB& aabb1, const OBB& aabb2) const;
    bool aabb_intersect(const AABB& aabb1, const AABB& aabb2) const;

private:
    static const int STACK_SIZE = 64;
    static const int NULL_IDX = -1;

    mutable std::string _guid;

    static AABB aabb_from_obb(const OBB& obb);
    double center(const AABB& aabb, int axis) const;
    double half(const AABB& aabb, int axis) const;

    /// (morton code, id) sorted by code, codes quantized over the bounding cube of the box centers
    std::vector<std::pair<uint32_t, int>> sorted_codes(const AABB* aabbs, int n) const;

    /// Leading bits shared by codes i and j, ties broken by index; -1 when j is out of range
    int common_prefix(const std::vector<std::pair<uint32_t, int>>& codes, int i, int j) const;

    /// Sorted range [first, last] covered by internal node i
    std::pair<int, int> determine_range(const std::vector<std::pair<uint32_t, int>>& codes, int i) const;

    /// Last index of the left half of [first, last]
    int find_split(const std::vector<std::pair<uint32_t, int>>& codes, int first, int last) const;

    /// (entry, exit) ray parameters of the box slabs; a miss when exit < entry
    std::pair<double, double> ray_aabb(const Point& origin, const Vector& direction, const AABB& aabb) const;
};

uint32_t expand_bits(uint32_t v);
uint32_t calculate_morton_code(double x, double y, double z, double world_size = 100.0);

} // namespace session_cpp
