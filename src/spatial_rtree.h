// SpatialRTree — R-tree with dynamic insert/delete (Guttman split, fan-out 4–8).
// Use for: "find all objects overlapping this region" (spatial range queries).
//   Supports live insertion and deletion; good for mutable object sets.
// Prefer over SpatialAABBTree/SpatialBVH when data changes frequently.
// Prefer over SpatialKDTree  when querying volumes/boxes, not bare point clouds.
// Note: k-NN is possible but slower than SpatialKDTree for pure point queries.
#pragma once

#include <cassert>
#include <cmath>
#include <algorithm>
#include <functional>
#include <vector>
#include <limits>

#define ASSERT assert

namespace session_cpp {

template<class DATATYPE, class ELEMTYPE, int NUMDIMS>
class SpatialRTree {
public:
    static const int MAXNODES = 8;
    static const int MINNODES = 4;
    static const int NOT_TAKEN = -1;

    struct Rect {
        ELEMTYPE m_min[NUMDIMS];
        ELEMTYPE m_max[NUMDIMS];
    };

    struct Node;

    struct Branch {
        Rect     m_rect;
        Node*    m_child;
        DATATYPE m_data;
    };

    struct Node {
        int    m_count;
        int    m_level;
        Branch m_branch[MAXNODES + 1];
        bool   is_leaf() const { return m_level == 0; }
    };

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

private:
    Node* m_root;
    int   m_size;

    Node* alloc_node() {
        Node* node = new Node();
        node->m_count = 0;
        node->m_level = 0;
        return node;
    }

    void free_node(Node* node) {
        delete node;
    }

    void free_subtree(Node* node) {
        if (!node) return;
        if (!node->is_leaf()) {
            for (int i = 0; i < node->m_count; i++) {
                free_subtree(node->m_branch[i].m_child);
            }
        }
        free_node(node);
    }

    Rect make_rect(const ELEMTYPE a_min[NUMDIMS], const ELEMTYPE a_max[NUMDIMS]) const {
        Rect rect;
        for (int i = 0; i < NUMDIMS; i++) {
            rect.m_min[i] = a_min[i];
            rect.m_max[i] = a_max[i];
        }
        return rect;
    }

    ELEMTYPE calc_rect_volume(const Rect& rect) const {
        ELEMTYPE volume = (ELEMTYPE)1;
        for (int i = 0; i < NUMDIMS; i++) {
            volume *= (rect.m_max[i] - rect.m_min[i]);
        }
        return volume;
    }

    Rect combine_rect(const Rect& a, const Rect& b) const {
        Rect rect;
        for (int i = 0; i < NUMDIMS; i++) {
            rect.m_min[i] = std::min(a.m_min[i], b.m_min[i]);
            rect.m_max[i] = std::max(a.m_max[i], b.m_max[i]);
        }
        return rect;
    }

    bool overlaps(const Rect& a, const Rect& b) const {
        for (int i = 0; i < NUMDIMS; i++) {
            if (a.m_max[i] < b.m_min[i] || b.m_max[i] < a.m_min[i]) {
                return false;
            }
        }
        return true;
    }

    Rect node_cover(const Node* node) const {
        Rect rect = node->m_branch[0].m_rect;
        for (int i = 1; i < node->m_count; i++) {
            rect = combine_rect(rect, node->m_branch[i].m_rect);
        }
        return rect;
    }

    bool add_branch(const Branch& branch, Node* node, Node** new_node) {
        if (node->m_count < MAXNODES) {
            node->m_branch[node->m_count] = branch;
            node->m_count++;
            return false;
        } else {
            ASSERT(new_node);
            split_node(node, branch, new_node);
            return true;
        }
    }

    void disconnect_branch(Node* node, int index) {
        ASSERT(index >= 0 && index < node->m_count);
        node->m_branch[index] = node->m_branch[node->m_count - 1];
        node->m_count--;
    }

    int pick_branch(const Rect& rect, const Node* node) const {
        ELEMTYPE best_incr = (ELEMTYPE)-1;
        ELEMTYPE best_area = (ELEMTYPE)-1;
        int best = 0;
        for (int i = 0; i < node->m_count; i++) {
            const Rect& cur = node->m_branch[i].m_rect;
            ELEMTYPE area = calc_rect_volume(cur);
            Rect combined = combine_rect(rect, cur);
            ELEMTYPE incr = calc_rect_volume(combined) - area;
            if (i == 0 || incr < best_incr || (incr == best_incr && area < best_area)) {
                best = i;
                best_incr = incr;
                best_area = area;
            }
        }
        return best;
    }

    void get_branches(Node* node, const Branch& branch, PartitionVars& part_vars) {
        ASSERT(node->m_count == MAXNODES);
        for (int i = 0; i < MAXNODES; i++) {
            part_vars.m_branch_buf[i] = node->m_branch[i];
        }
        part_vars.m_branch_buf[MAXNODES] = branch;
        part_vars.m_branch_count = MAXNODES + 1;
        part_vars.m_cover_split = part_vars.m_branch_buf[0].m_rect;
        for (int i = 1; i < MAXNODES + 1; i++) {
            part_vars.m_cover_split = combine_rect(part_vars.m_cover_split, part_vars.m_branch_buf[i].m_rect);
        }
        part_vars.m_cover_split_area = calc_rect_volume(part_vars.m_cover_split);
        node->m_count = 0;
    }

    void init_part_vars(PartitionVars& part_vars, int max_rects, int min_fill) {
        part_vars.m_count[0] = part_vars.m_count[1] = 0;
        part_vars.m_area[0] = part_vars.m_area[1] = (ELEMTYPE)0;
        part_vars.m_total = max_rects;
        part_vars.m_min_fill = min_fill;
        for (int i = 0; i < max_rects; i++) {
            part_vars.m_partition[i] = NOT_TAKEN;
        }
    }

    void classify_branch(int index, int group, PartitionVars& part_vars) {
        ASSERT(part_vars.m_partition[index] == NOT_TAKEN);
        part_vars.m_partition[index] = group;
        if (part_vars.m_count[group] == 0) {
            part_vars.m_cover[group] = part_vars.m_branch_buf[index].m_rect;
        } else {
            part_vars.m_cover[group] = combine_rect(part_vars.m_branch_buf[index].m_rect, part_vars.m_cover[group]);
        }
        part_vars.m_area[group] = calc_rect_volume(part_vars.m_cover[group]);
        part_vars.m_count[group]++;
    }

    void pick_seeds(PartitionVars& part_vars) {
        int seed0 = 0, seed1 = 1;
        ELEMTYPE worst = -part_vars.m_cover_split_area - (ELEMTYPE)1;
        ELEMTYPE area[MAXNODES + 1];
        for (int i = 0; i < part_vars.m_total; i++) {
            area[i] = calc_rect_volume(part_vars.m_branch_buf[i].m_rect);
        }
        for (int i = 0; i < part_vars.m_total - 1; i++) {
            for (int j = i + 1; j < part_vars.m_total; j++) {
                Rect combined = combine_rect(part_vars.m_branch_buf[i].m_rect, part_vars.m_branch_buf[j].m_rect);
                ELEMTYPE waste = calc_rect_volume(combined) - area[i] - area[j];
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

    void choose_partition(PartitionVars& part_vars, int min_fill) {
        init_part_vars(part_vars, part_vars.m_branch_count, min_fill);
        pick_seeds(part_vars);
        while ((part_vars.m_count[0] + part_vars.m_count[1]) < part_vars.m_total &&
               part_vars.m_count[0] < (part_vars.m_total - part_vars.m_min_fill) &&
               part_vars.m_count[1] < (part_vars.m_total - part_vars.m_min_fill)) {
            ELEMTYPE biggest_diff = (ELEMTYPE)-1;
            int chosen = 0, better_group = 0;
            for (int i = 0; i < part_vars.m_total; i++) {
                if (part_vars.m_partition[i] == NOT_TAKEN) {
                    Rect r0 = combine_rect(part_vars.m_branch_buf[i].m_rect, part_vars.m_cover[0]);
                    Rect r1 = combine_rect(part_vars.m_branch_buf[i].m_rect, part_vars.m_cover[1]);
                    ELEMTYPE growth0 = calc_rect_volume(r0) - part_vars.m_area[0];
                    ELEMTYPE growth1 = calc_rect_volume(r1) - part_vars.m_area[1];
                    ELEMTYPE diff = growth1 - growth0;
                    int group;
                    if (diff >= 0) {
                        group = 0;
                    } else {
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
            }
            classify_branch(chosen, better_group, part_vars);
        }
        if ((part_vars.m_count[0] + part_vars.m_count[1]) < part_vars.m_total) {
            int group = (part_vars.m_count[0] >= part_vars.m_total - part_vars.m_min_fill) ? 1 : 0;
            for (int i = 0; i < part_vars.m_total; i++) {
                if (part_vars.m_partition[i] == NOT_TAKEN) {
                    classify_branch(i, group, part_vars);
                }
            }
        }
    }

    void load_nodes(Node* node_a, Node* node_b, PartitionVars& part_vars) {
        for (int i = 0; i < part_vars.m_total; i++) {
            int g = part_vars.m_partition[i];
            Node* target = (g == 0) ? node_a : node_b;
            Node* dummy = nullptr;
            add_branch(part_vars.m_branch_buf[i], target, &dummy);
        }
    }

    void split_node(Node* node, const Branch& branch, Node** new_node) {
        PartitionVars part_vars;
        get_branches(node, branch, part_vars);
        choose_partition(part_vars, MINNODES);
        *new_node = alloc_node();
        (*new_node)->m_level = node->m_level;
        load_nodes(node, *new_node, part_vars);
    }

    Node* insert_rect_rec(const Branch& branch, Node* node, int level) {
        ASSERT(node);
        if (node->m_level > level) {
            int idx = pick_branch(branch.m_rect, node);
            Node* other = insert_rect_rec(branch, node->m_branch[idx].m_child, level);
            if (!other) {
                node->m_branch[idx].m_rect = combine_rect(node->m_branch[idx].m_rect, branch.m_rect);
                return nullptr;
            } else {
                node->m_branch[idx].m_rect = node_cover(node->m_branch[idx].m_child);
                Branch new_b;
                new_b.m_rect = node_cover(other);
                new_b.m_child = other;
                Node* new_node = nullptr;
                add_branch(new_b, node, &new_node);
                return new_node;
            }
        } else if (node->m_level == level) {
            Node* new_node = nullptr;
            add_branch(branch, node, &new_node);
            return new_node;
        }
        ASSERT(false);
        return nullptr;
    }

    void insert_branch_internal(const Branch& branch, int level) {
        Node* new_node = insert_rect_rec(branch, m_root, level);
        if (new_node) {
            Node* old_root = m_root;
            m_root = alloc_node();
            m_root->m_level = old_root->m_level + 1;
            Branch b;
            b.m_rect = node_cover(old_root);
            b.m_child = old_root;
            Node* dummy = nullptr;
            add_branch(b, m_root, &dummy);
            b.m_rect = node_cover(new_node);
            b.m_child = new_node;
            add_branch(b, m_root, &dummy);
        }
    }

    bool search_internal(const Rect& rect, Node* node, int& count, std::function<bool(const DATATYPE&)>& callback) const {
        if (node->is_leaf()) {
            for (int i = 0; i < node->m_count; i++) {
                if (overlaps(rect, node->m_branch[i].m_rect)) {
                    count++;
                    if (!callback(node->m_branch[i].m_data)) {
                        return false;
                    }
                }
            }
        } else {
            for (int i = 0; i < node->m_count; i++) {
                if (overlaps(rect, node->m_branch[i].m_rect)) {
                    if (!search_internal(rect, node->m_branch[i].m_child, count, callback)) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    bool remove_rect_internal(const Rect& rect, const DATATYPE& data, Node* node, std::vector<Node*>& reinsert_list) {
        if (node->is_leaf()) {
            for (int i = 0; i < node->m_count; i++) {
                if (node->m_branch[i].m_data == data && overlaps(rect, node->m_branch[i].m_rect)) {
                    disconnect_branch(node, i);
                    return true;
                }
            }
            return false;
        } else {
            for (int i = 0; i < node->m_count; i++) {
                if (overlaps(rect, node->m_branch[i].m_rect)) {
                    if (remove_rect_internal(rect, data, node->m_branch[i].m_child, reinsert_list)) {
                        if (node->m_branch[i].m_child->m_count >= MINNODES) {
                            node->m_branch[i].m_rect = node_cover(node->m_branch[i].m_child);
                        } else {
                            reinsert_list.push_back(node->m_branch[i].m_child);
                            disconnect_branch(node, i);
                        }
                        return true;
                    }
                }
            }
            return false;
        }
    }

public:
    SpatialRTree() {
        m_root = alloc_node();
        m_root->m_level = 0;
        m_size = 0;
    }

    ~SpatialRTree() {
        free_subtree(m_root);
    }

    void insert(const ELEMTYPE a_min[NUMDIMS], const ELEMTYPE a_max[NUMDIMS], const DATATYPE& a_data) {
        Branch branch;
        branch.m_rect = make_rect(a_min, a_max);
        branch.m_child = nullptr;
        branch.m_data = a_data;
        insert_branch_internal(branch, 0);
        m_size++;
    }

    bool remove(const ELEMTYPE a_min[NUMDIMS], const ELEMTYPE a_max[NUMDIMS], const DATATYPE& a_data) {
        Rect rect = make_rect(a_min, a_max);
        std::vector<Node*> reinsert_list;
        if (!remove_rect_internal(rect, a_data, m_root, reinsert_list)) {
            return false;
        }
        for (Node* node : reinsert_list) {
            for (int i = 0; i < node->m_count; i++) {
                insert_branch_internal(node->m_branch[i], node->m_level);
            }
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

    int search(const ELEMTYPE a_min[NUMDIMS], const ELEMTYPE a_max[NUMDIMS], std::function<bool(const DATATYPE&)> a_callback) const {
        Rect rect = make_rect(a_min, a_max);
        int count = 0;
        search_internal(rect, m_root, count, a_callback);
        return count;
    }

    void remove_all() {
        free_subtree(m_root);
        m_root = alloc_node();
        m_root->m_level = 0;
        m_size = 0;
    }

    int count() const { return m_size; }
};

using RTree3 = SpatialRTree<int, double, 3>;

} // namespace session_cpp
