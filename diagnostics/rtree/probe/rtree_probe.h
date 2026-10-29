#pragma once
#include <array>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <vector>
#ifdef _MSC_VER
#define RTREE_PROBE_NOINLINE __declspec(noinline)
#else
#define RTREE_PROBE_NOINLINE __attribute__((noinline))
#endif
namespace session_cpp {
struct RTreeProbe {
    using Tree = SpatialRTree<std::size_t, double, 2>;
    using Rect = Tree::Rect;
    using Node = Tree::Node;
    struct Summary { Rect box{}; std::size_t count = 0; };
    struct Step { int level; int slot; Rect box; };
    static std::uint64_t bits(double value) {
        std::uint64_t result; std::memcpy(&result, &value, sizeof(result)); return result;
    }
    static std::uint64_t key(double value) {
        auto raw = bits(value);
        if ((raw & 0x7fffffffffffffffULL) == 0) raw = 0;
        return raw >> 63 ? ~raw : raw ^ (std::uint64_t(1) << 63);
    }
    static bool contains(const Rect& outer, const Rect& inner) {
        for (int d = 0; d < 2; ++d)
            if (key(outer.m_min[d]) > key(inner.m_min[d]) || key(outer.m_max[d]) < key(inner.m_max[d])) return false;
        return true;
    }
    static bool intersects(const Rect& a, const Rect& b) {
        for (int d = 0; d < 2; ++d)
            if (key(a.m_max[d]) < key(b.m_min[d]) || key(b.m_max[d]) < key(a.m_min[d])) return false;
        return true;
    }
    static void print_box(const Rect& box) {
        std::cout << std::hex << " lo_bits=[" << bits(box.m_min[0]) << ',' << bits(box.m_min[1])
            << "] hi_bits=[" << bits(box.m_max[0]) << ',' << bits(box.m_max[1]) << ']' << std::dec;
    }
    static void merge(Summary& total, const Summary& part) {
        if (!part.count) return;
        if (!total.count) { total = part; return; }
        for (int d = 0; d < 2; ++d) {
            if (key(part.box.m_min[d]) < key(total.box.m_min[d])) total.box.m_min[d] = part.box.m_min[d];
            if (key(part.box.m_max[d]) > key(total.box.m_max[d])) total.box.m_max[d] = part.box.m_max[d];
        }
        total.count += part.count;
    }
    static Summary scan(const Node* node, std::size_t inserted, std::vector<int>& path, bool& first_bad) {
        Summary result;
        for (int i = 0; i < node->m_count; ++i) {
            const auto& branch = node->m_branch[i];
            path.push_back(i);
            Summary part;
            if (node->m_level == 0) { part.box = branch.m_rect; part.count = 1; }
            else {
                part = scan(branch.m_child, inserted, path, first_bad);
                if (!contains(branch.m_rect, part.box) && !first_bad) {
                    first_bad = true;
                    std::cout << "probe_first_bad_cover after_insert=" << inserted << " level=" << node->m_level << " path=";
                    for (int slot : path) std::cout << slot << '/';
                    std::cout << " stored"; print_box(branch.m_rect);
                    std::cout << " descendant_hull"; print_box(part.box);
                    std::cout << " descendant_leaves=" << part.count << '\n';
                }
            }
            merge(result, part);
            path.pop_back();
        }
        return result;
    }
    static RTREE_PROBE_NOINLINE void after_insert(const Tree& tree, std::size_t inserted, std::size_t expected_count) {
        static bool first_bad = false;
        std::vector<int> path;
        const auto result = scan(tree.m_root, inserted, path, first_bad);
        if (result.count != expected_count)
            std::cout << "probe_leaf_count after_insert=" << inserted << " actual=" << result.count << " expected=" << expected_count << '\n';
    }
    static void seek(const Tree& tree, const Node* node, const Rect& query, std::size_t target,
        std::vector<Step>& path, std::size_t& found) {
        for (int i = 0; i < node->m_count; ++i) {
            const auto& branch = node->m_branch[i];
            path.push_back({node->m_level, i, branch.m_rect});
            if (node->m_level == 0) {
                if (branch.m_data == target) {
                    ++found;
                    std::cout << "probe_target_leaf id=" << target << " path_length=" << path.size() << '\n';
                    for (const auto& step : path) {
                        std::cout << "probe_path level=" << step.level << " slot=" << step.slot
                            << " integer_overlap=" << intersects(query, step.box)
                            << " production_overlap=" << tree.overlaps(query, step.box);
                        print_box(step.box); std::cout << '\n';
                    }
                }
            } else seek(tree, branch.m_child, query, target, path, found);
            path.pop_back();
        }
    }
    static RTREE_PROBE_NOINLINE void dump_query(const Tree& tree, const double* lo, const double* hi, std::size_t target) {
        Rect query;
        for (int d = 0; d < 2; ++d) { query.m_min[d] = lo[d]; query.m_max[d] = hi[d]; }
        const auto production_query = tree.to_rect(lo, hi);
        std::cout << "probe_query target=" << target << " manual"; print_box(query);
        std::cout << " production_to_rect"; print_box(production_query); std::cout << '\n';
        std::vector<Step> path; std::size_t found = 0;
        seek(tree, tree.m_root, query, target, path, found);
        std::cout << "probe_target_occurrences id=" << target << " count=" << found << '\n';
    }
};
}
#undef RTREE_PROBE_NOINLINE
