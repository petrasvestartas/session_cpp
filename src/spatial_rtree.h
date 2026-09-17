#pragma once

#include <algorithm>
#include <cassert>
#include <functional>
#include <vector>

namespace session_cpp {

/// R-tree with dynamic insert and remove (Guttman quadratic split, fan-out 4 to 8) for box overlap queries.
template<class DATATYPE, class ELEMTYPE, int NUMDIMS>
class SpatialRTree {
public:
    /// Construct an empty tree with a single leaf root.
    SpatialRTree() {
        m_root = alloc_node();
        m_size = 0;
    }

    /// Free every node.
    ~SpatialRTree() {
        free_subtree(m_root);
    }

    /// Number of stored items.
    int count() const { return m_size; }

    /// Insert an item with its bounding box.
    void insert(const ELEMTYPE a_min[NUMDIMS], const ELEMTYPE a_max[NUMDIMS], const DATATYPE& a_data) {

        Branch branch;
        branch.m_rect = make_rect(a_min, a_max);
        branch.m_child = nullptr;
        branch.m_data = a_data;
        insert_branch_internal(branch, 0);
        m_size++;
    }

    /// Remove an item by its bounding box and data; false when not found.
    bool remove(const ELEMTYPE a_min[NUMDIMS], const ELEMTYPE a_max[NUMDIMS], const DATATYPE& a_data) {

        const Rect rect = make_rect(a_min, a_max);
        std::vector<Node*> reinsert_list;

        if (!remove_rect_internal(rect, a_data, reinsert_list))
            return false;

        for (Node* node : reinsert_list) {
            for (int i = 0; i < node->m_count; i++)
                insert_branch_internal(node->m_branch[i], node->m_level);

            free_node(node);
        }

        while (!m_root->is_leaf() && m_root->m_count == 1) {
            Node* old_root = m_root;
            m_root = m_root->m_branch[0].m_child;
            free_node(old_root);
        }

        m_size--;

        return true;
    }

    /// Remove every item.
    void remove_all() {
        free_subtree(m_root);
        m_root = alloc_node();
        m_size = 0;
    }

    /// Visit every item overlapping the box until the callback returns false; returns the visit count.
    int search(const ELEMTYPE a_min[NUMDIMS], const ELEMTYPE a_max[NUMDIMS], const std::function<bool(const DATATYPE&)>& a_callback) const {

        const Rect rect = make_rect(a_min, a_max);
        Visit stack[STACK_SIZE];
        int top = 0;
        stack[top++] = {m_root, 0};
        int count = 0;

        while (top > 0) {
            Visit& visit = stack[top - 1];

            if (visit.index == visit.node->m_count) {
                top--;
                continue;
            }

            const Branch& branch = visit.node->m_branch[visit.index++];

            if (!overlaps(rect, branch.m_rect))
                continue;

            if (!visit.node->is_leaf()) {
                assert(top < STACK_SIZE);
                stack[top++] = {branch.m_child, 0};
                continue;
            }

            count++;

            if (!a_callback(branch.m_data))
                return count;
        }

        return count;
    }

private:
    static const int MAXNODES = 8; // Fan-out ceiling.
    static const int MINNODES = 4; // Fan-out floor.
    static const int NOT_TAKEN = -1; // Partition slot not yet assigned.
    static const int STACK_SIZE = 64; // Explicit traversal stack depth.

    /// Axis-aligned box.
    struct Rect {
        ELEMTYPE m_min[NUMDIMS];
        ELEMTYPE m_max[NUMDIMS];
    };

    struct Node;

    /// Child pointer or leaf datum with its cover.
    struct Branch {
        Rect     m_rect; // Cover of the child or datum.
        Node*    m_child; // Child node, null on leaves.
        DATATYPE m_data; // Leaf datum.
    };

    /// Inner or leaf node with up to MAXNODES + 1 branches during a split.
    struct Node {
        int    m_count; // Branches in use.
        int    m_level; // 0 for leaves.
        Branch m_branch[MAXNODES + 1];

        /// Whether the node is a leaf.
        bool is_leaf() const { return m_level == 0; }
    };

    /// Traversal stack entry.
    struct Visit {
        Node* node; // Node being walked.
        int   index; // Next branch to visit.
    };

    /// Scratch state for a quadratic split.
    struct PartitionVars {
        int      m_partition[MAXNODES + 1];
        int      m_total;
        int      m_min_fill;
        int      m_count[2];
        Rect     m_cover[2];
        ELEMTYPE m_area[2];
        Branch   m_branch_buf[MAXNODES + 1];
        int      m_branch_count;
        Rect     m_cover_split;
        ELEMTYPE m_cover_split_area;
    };

    Node* m_root; // Tree root.
    int   m_size; // Stored item count.

    /// Allocate an empty leaf node.
    Node* alloc_node() {

        Node* node = new Node();
        node->m_count = 0;
        node->m_level = 0;

        return node;
    }

    /// Free one node.
    void free_node(Node* node) {
        delete node;
    }

    /// Free a node and every descendant with an explicit stack.
    void free_subtree(Node* root) {

        Visit stack[STACK_SIZE];
        int top = 0;
        stack[top++] = {root, 0};

        while (top > 0) {
            Visit& visit = stack[top - 1];

            if (visit.node->is_leaf() || visit.index == visit.node->m_count) {
                free_node(visit.node);
                top--;
                continue;
            }

            Node* child = visit.node->m_branch[visit.index++].m_child;
            assert(top < STACK_SIZE);
            stack[top++] = {child, 0};
        }
    }

    /// Build a rect from min and max corners.
    Rect make_rect(const ELEMTYPE a_min[NUMDIMS], const ELEMTYPE a_max[NUMDIMS]) const {

        Rect rect;

        for (int i = 0; i < NUMDIMS; i++) {
            rect.m_min[i] = std::min(a_min[i], a_max[i]);
            rect.m_max[i] = std::max(a_min[i], a_max[i]);
        }

        return rect;
    }

    /// Volume of a rect.
    ELEMTYPE calc_rect_volume(const Rect& rect) const {

        ELEMTYPE volume = (ELEMTYPE)1;

        for (int i = 0; i < NUMDIMS; i++)
            volume *= rect.m_max[i] - rect.m_min[i];

        return volume;
    }

    /// Smallest rect covering both.
    Rect combine_rect(const Rect& a, const Rect& b) const {

        Rect rect;

        for (int i = 0; i < NUMDIMS; i++) {
            rect.m_min[i] = std::min(a.m_min[i], b.m_min[i]);
            rect.m_max[i] = std::max(a.m_max[i], b.m_max[i]);
        }

        return rect;
    }

    /// Whether two rects overlap.
    bool overlaps(const Rect& a, const Rect& b) const {

        for (int i = 0; i < NUMDIMS; i++)
            if (a.m_max[i] < b.m_min[i] || b.m_max[i] < a.m_min[i])
                return false;

        return true;
    }

    /// Rect covering every branch of a node.
    Rect node_cover(const Node* node) const {

        Rect rect = node->m_branch[0].m_rect;

        for (int i = 1; i < node->m_count; i++)
            rect = combine_rect(rect, node->m_branch[i].m_rect);

        return rect;
    }

    /// Add a branch, splitting the node when full; returns the new sibling or null.
    Node* add_branch(const Branch& branch, Node* node) {

        if (node->m_count == MAXNODES)
            return split_node(node, branch);

        node->m_branch[node->m_count] = branch;
        node->m_count++;

        return nullptr;
    }

    /// Remove a branch by swapping in the last one.
    void disconnect_branch(Node* node, int index) {
        assert(index >= 0 && index < node->m_count);
        node->m_branch[index] = node->m_branch[node->m_count - 1];
        node->m_count--;
    }

    /// Branch whose rect grows least when covering the rect.
    int pick_branch(const Rect& rect, const Node* node) const {

        ELEMTYPE best_incr = (ELEMTYPE)-1;
        ELEMTYPE best_area = (ELEMTYPE)-1;
        int best = 0;

        for (int i = 0; i < node->m_count; i++) {
            const Rect& cur = node->m_branch[i].m_rect;
            const ELEMTYPE area = calc_rect_volume(cur);
            const Rect combined = combine_rect(rect, cur);
            const ELEMTYPE incr = calc_rect_volume(combined) - area;

            if (i == 0 || incr < best_incr || (incr == best_incr && area < best_area)) {
                best = i;
                best_incr = incr;
                best_area = area;
            }
        }

        return best;
    }

    /// Collect the node's branches plus one extra into the partition buffer.
    void get_branches(Node* node, const Branch& branch, PartitionVars& part_vars) {

        assert(node->m_count == MAXNODES);

        for (int i = 0; i < MAXNODES; i++)
            part_vars.m_branch_buf[i] = node->m_branch[i];

        part_vars.m_branch_buf[MAXNODES] = branch;
        part_vars.m_branch_count = MAXNODES + 1;
        part_vars.m_cover_split = part_vars.m_branch_buf[0].m_rect;

        for (int i = 1; i < MAXNODES + 1; i++)
            part_vars.m_cover_split = combine_rect(part_vars.m_cover_split, part_vars.m_branch_buf[i].m_rect);

        part_vars.m_cover_split_area = calc_rect_volume(part_vars.m_cover_split);
        node->m_count = 0;
    }

    /// Reset the partition buffer.
    void init_part_vars(PartitionVars& part_vars, int max_rects, int min_fill) {

        part_vars.m_count[0] = 0;
        part_vars.m_count[1] = 0;
        part_vars.m_area[0] = (ELEMTYPE)0;
        part_vars.m_area[1] = (ELEMTYPE)0;
        part_vars.m_total = max_rects;
        part_vars.m_min_fill = min_fill;

        for (int i = 0; i < max_rects; i++)
            part_vars.m_partition[i] = NOT_TAKEN;
    }

    /// Assign a branch to a group and grow the group cover.
    void classify_branch(int index, int group, PartitionVars& part_vars) {

        assert(part_vars.m_partition[index] == NOT_TAKEN);
        part_vars.m_partition[index] = group;

        if (part_vars.m_count[group] == 0)
            part_vars.m_cover[group] = part_vars.m_branch_buf[index].m_rect;
        else
            part_vars.m_cover[group] = combine_rect(part_vars.m_branch_buf[index].m_rect, part_vars.m_cover[group]);

        part_vars.m_area[group] = calc_rect_volume(part_vars.m_cover[group]);
        part_vars.m_count[group]++;
    }

    /// Seed the two groups with the most wasteful pair.
    void pick_seeds(PartitionVars& part_vars) {

        int seed0 = 0;
        int seed1 = 1;
        ELEMTYPE worst = -part_vars.m_cover_split_area - (ELEMTYPE)1;
        ELEMTYPE area[MAXNODES + 1];

        for (int i = 0; i < part_vars.m_total; i++)
            area[i] = calc_rect_volume(part_vars.m_branch_buf[i].m_rect);

        for (int i = 0; i < part_vars.m_total - 1; i++) {
            for (int j = i + 1; j < part_vars.m_total; j++) {
                const Rect combined = combine_rect(part_vars.m_branch_buf[i].m_rect, part_vars.m_branch_buf[j].m_rect);
                const ELEMTYPE waste = calc_rect_volume(combined) - area[i] - area[j];

                if (waste > worst) {
                    worst = waste;
                    seed0 = i;
                    seed1 = j;
                }
            }
        }

        classify_branch(seed0, 0, part_vars);
        classify_branch(seed1, 1, part_vars);
    }

    /// Quadratic split of the partition buffer into two groups.
    void choose_partition(PartitionVars& part_vars, int min_fill) {

        init_part_vars(part_vars, part_vars.m_branch_count, min_fill);
        pick_seeds(part_vars);

        while ((part_vars.m_count[0] + part_vars.m_count[1]) < part_vars.m_total &&
               part_vars.m_count[0] < (part_vars.m_total - part_vars.m_min_fill) &&
               part_vars.m_count[1] < (part_vars.m_total - part_vars.m_min_fill)) {

            ELEMTYPE biggest_diff = (ELEMTYPE)-1;
            int chosen = 0;
            int better_group = 0;

            for (int i = 0; i < part_vars.m_total; i++) {
                if (part_vars.m_partition[i] != NOT_TAKEN)
                    continue;

                const Rect r0 = combine_rect(part_vars.m_branch_buf[i].m_rect, part_vars.m_cover[0]);
                const Rect r1 = combine_rect(part_vars.m_branch_buf[i].m_rect, part_vars.m_cover[1]);
                const ELEMTYPE growth0 = calc_rect_volume(r0) - part_vars.m_area[0];
                const ELEMTYPE growth1 = calc_rect_volume(r1) - part_vars.m_area[1];
                ELEMTYPE diff = growth1 - growth0;
                int group = 0;

                if (diff < 0) {
                    group = 1;
                    diff = -diff;
                }

                if (diff > biggest_diff) {
                    biggest_diff = diff;
                    chosen = i;
                    better_group = group;
                } else if (diff == biggest_diff && part_vars.m_count[group] < part_vars.m_count[better_group]) {
                    chosen = i;
                    better_group = group;
                }
            }

            classify_branch(chosen, better_group, part_vars);
        }

        if ((part_vars.m_count[0] + part_vars.m_count[1]) < part_vars.m_total) {
            const int group = (part_vars.m_count[0] >= part_vars.m_total - part_vars.m_min_fill) ? 1 : 0;

            for (int i = 0; i < part_vars.m_total; i++)
                if (part_vars.m_partition[i] == NOT_TAKEN)
                    classify_branch(i, group, part_vars);
        }
    }

    /// Move partitioned branches into the two nodes.
    void load_nodes(Node* node_a, Node* node_b, PartitionVars& part_vars) {

        for (int i = 0; i < part_vars.m_total; i++) {
            Node* target = (part_vars.m_partition[i] == 0) ? node_a : node_b;
            add_branch(part_vars.m_branch_buf[i], target);
        }
    }

    /// Split a full node with the extra branch; returns the new sibling.
    Node* split_node(Node* node, const Branch& branch) {

        PartitionVars part_vars;
        get_branches(node, branch, part_vars);
        choose_partition(part_vars, MINNODES);
        Node* new_node = alloc_node();
        new_node->m_level = node->m_level;
        load_nodes(node, new_node, part_vars);

        return new_node;
    }

    /// Insert a branch at a level; returns the root's new sibling or null.
    Node* insert_rect_internal(const Branch& branch, int level) {

        Visit stack[STACK_SIZE];
        int top = 0;
        Node* node = m_root;

        while (node->m_level != level) {
            assert(node->m_level > level);
            assert(top < STACK_SIZE);
            const int idx = pick_branch(branch.m_rect, node);
            stack[top++] = {node, idx};
            node = node->m_branch[idx].m_child;
        }

        Node* other = add_branch(branch, node);

        for (int d = top - 1; d >= 0; d--) {
            Node* parent = stack[d].node;
            const int idx = stack[d].index;

            if (!other) {
                parent->m_branch[idx].m_rect = combine_rect(parent->m_branch[idx].m_rect, branch.m_rect);
                continue;
            }

            parent->m_branch[idx].m_rect = node_cover(parent->m_branch[idx].m_child);
            Branch new_b;
            new_b.m_rect = node_cover(other);
            new_b.m_child = other;
            other = add_branch(new_b, parent);
        }

        return other;
    }

    /// Insert a branch and grow the root when it splits.
    void insert_branch_internal(const Branch& branch, int level) {

        Node* new_node = insert_rect_internal(branch, level);

        if (!new_node)
            return;

        Node* old_root = m_root;
        m_root = alloc_node();
        m_root->m_level = old_root->m_level + 1;
        Branch b1;
        b1.m_rect = node_cover(old_root);
        b1.m_child = old_root;
        Branch b2;
        b2.m_rect = node_cover(new_node);
        b2.m_child = new_node;
        add_branch(b1, m_root);
        add_branch(b2, m_root);
    }

    /// Remove the matching leaf branch; underfull nodes go to the reinsert list.
    bool remove_rect_internal(const Rect& rect, const DATATYPE& data, std::vector<Node*>& reinsert_list) {

        Visit stack[STACK_SIZE];
        int top = 0;
        stack[top++] = {m_root, 0};

        while (top > 0) {
            Visit& visit = stack[top - 1];

            if (visit.index == visit.node->m_count) {
                top--;
                continue;
            }

            const Branch& branch = visit.node->m_branch[visit.index++];

            if (!overlaps(rect, branch.m_rect))
                continue;

            if (!visit.node->is_leaf()) {
                assert(top < STACK_SIZE);
                stack[top++] = {branch.m_child, 0};
                continue;
            }

            if (branch.m_data != data)
                continue;

            disconnect_branch(visit.node, visit.index - 1);

            for (int d = top - 2; d >= 0; d--)
                shrink_branch(stack[d].node, stack[d].index - 1, reinsert_list);

            return true;
        }

        return false;
    }

    /// Recompute a child's cover or queue it for reinsertion when underfull.
    void shrink_branch(Node* node, int index, std::vector<Node*>& reinsert_list) {

        Node* child = node->m_branch[index].m_child;

        if (child->m_count >= MINNODES) {
            node->m_branch[index].m_rect = node_cover(child);

            return;
        }

        reinsert_list.push_back(child);
        disconnect_branch(node, index);
    }
};

using RTree3 = SpatialRTree<int, double, 3>;

} // namespace session_cpp
