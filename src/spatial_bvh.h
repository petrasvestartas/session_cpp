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
private:
    static const int STACK_SIZE = 64; // Depth bound of the explicit traversal stack.
    static const int NULL_IDX = -1; // Index of a missing child or object.

    mutable std::string _guid; // Lazy guid.

public:

    /// Tree node.
    struct Node {
        AABB aabb; // Bounds of the subtree.
        int left = NULL_IDX; // Left child index, NULL_IDX on a leaf.
        int right = NULL_IDX; // Right child index, NULL_IDX on a leaf.
        int object_id = NULL_IDX; // Object id on a leaf, NULL_IDX on an internal node.

        /// Return whether the node holds an object.
        bool is_leaf() const;
    };

    std::string name; // Tree name.
    double world_size; // Extent of the Morton cube.
    std::vector<std::string> object_guids; // Guid per object id, set by build_with_guids.
    std::vector<Node> nodes; // Internal nodes first, then the leaves in Morton order.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty tree over a Morton cube of world_size.
    SpatialBVH(double world_size = 1000.0);

    /// Construct and build over the boxes with the given world size.
    static SpatialBVH from_boxes(const std::vector<OBB>& bounding_boxes, double world_size);

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    bool has_guid() const { return !_guid.empty(); }

    /// Return the guid, creating it on first access.
    const std::string& guid() const {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return the mutable guid, creating it on first access.
    std::string& guid() {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return whether the tree has no nodes.
    bool empty() const;

    /// Return the node count.
    size_t size() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Mutators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Build over the boxes with the current world size.
    void build(const std::vector<OBB>& bounding_boxes);

    /// Build over count boxes with world size ws.
    void build_from_boxes(const OBB* boxes, size_t count, double ws);

    /// Build over count axis-aligned boxes with world size ws.
    void build_from_aabbs(const AABB* aabbs, size_t count, double ws);

    /// Build over boxes paired with their guids, world size computed from the boxes.
    void build_with_guids(const std::vector<std::pair<OBB, std::string>>& boxes_with_guids);

    // ═══════════════════════════════════════════════════════════════════════════
    // Queries
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the overlapping (i, j) pairs with i < j, the ids in any pair, and the number of nodes tested.
    std::tuple<std::vector<std::pair<int, int>>, std::vector<int>, int> check_all_collisions(const std::vector<OBB>& bounding_boxes);

    /// Return the overlapping pairs as guid pairs.
    std::vector<std::pair<std::string, std::string>> check_all_collisions_guids(const std::vector<OBB>& bounding_boxes);

    /// Return the ids overlapping query_bbox other than object_id, and the number of nodes tested.
    std::pair<std::vector<int>, int> find_collisions(int object_id, const OBB& query_bbox, const std::vector<OBB>& bounding_boxes) const;

    /// Return the ids of every leaf box that intersects query.
    std::vector<int> query_aabb(const AABB& query) const;

    /// Return the ids of every leaf box that intersects the axis-aligned bounds of query.
    std::vector<int> query_aabb(const OBB& query) const;

    /// Return the ids overlapping the box of object_id with its half-sizes scaled by inflate, object_id excluded.
    std::vector<int> nearest_neighbors(int object_id, const std::vector<OBB>& bounding_boxes, double inflate = 1.2) const;

    /// Collect the leaf ids whose box the ray enters, nearest entry first; true when any.
    bool ray_cast(const Point& origin, const Vector& direction, std::vector<int>& candidate_leaf_ids, bool find_all = false) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Boxes
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the largest absolute box coordinate times 2.2, at least 10.
    static double compute_world_size(const std::vector<OBB>& bounding_boxes);

    /// Return the axis-aligned box enclosing both boxes.
    OBB merge_aabb(const OBB& aabb1, const OBB& aabb2) const;

    /// Return whether the axis-aligned bounds of the boxes overlap.
    bool aabb_intersect(const OBB& aabb1, const OBB& aabb2) const;

    /// Return whether the boxes overlap.
    bool aabb_intersect(const AABB& aabb1, const AABB& aabb2) const;

private:
    // ═══════════════════════════════════════════════════════════════════════════
    // Build
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return (morton code, id) sorted by code, codes quantized over the bounding cube of the box centers.
    std::vector<std::pair<uint32_t, int>> sorted_codes(const AABB* aabbs, int n) const;

    /// Return the leading bits shared by codes i and j, ties broken by index; -1 when j is out of range.
    int common_prefix(const std::vector<std::pair<uint32_t, int>>& codes, int i, int j) const;

    /// Return the sorted range [first, last] covered by internal node i.
    std::pair<int, int> determine_range(const std::vector<std::pair<uint32_t, int>>& codes, int i) const;

    /// Return the last index of the left half of [first, last].
    int find_split(const std::vector<std::pair<uint32_t, int>>& codes, int first, int last) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Traversal
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the (entry, exit) ray parameters of the box slabs; a miss when exit < entry.
    std::pair<double, double> ray_aabb(const Point& origin, const Vector& direction, const AABB& aabb) const;

    /// Return the axis-aligned bounds of obb.
    static AABB aabb_from_obb(const OBB& obb);

    /// Return the center coordinate of aabb along axis.
    double center(const AABB& aabb, int axis) const;

    /// Return the half-size of aabb along axis.
    double half(const AABB& aabb, int axis) const;
};

// ═══════════════════════════════════════════════════════════════════════════
// Morton codes
// ═══════════════════════════════════════════════════════════════════════════
/// Spread the low 10 bits of v to every third bit.
uint32_t expand_bits(uint32_t v);

/// Return the Morton code of a point in the cube of world_size centered at the origin.
uint32_t calculate_morton_code(double x, double y, double z, double world_size = 100.0);

} // namespace session_cpp
