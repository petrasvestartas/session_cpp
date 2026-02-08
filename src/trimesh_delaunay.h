#pragma once

#include "nurbssurface.h"
#include "mesh.h"
#include "point.h"
#include "vector.h"
#include "tolerance.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// FlatMap64 - Open-addressing hash map with uint64_t keys (Fibonacci hashing)
///////////////////////////////////////////////////////////////////////////////////////////

template<typename V>
class FlatMap64 {
    static constexpr uint64_t EMPTY_KEY = ~uint64_t(0);
    struct Slot { uint64_t key = EMPTY_KEY; V value; };
    std::vector<Slot> slots_;
    size_t size_ = 0;
    size_t shift_ = 64;

    static size_t log2_pot(size_t n) {
        size_t r = 0;
        while ((size_t(1) << r) < n) ++r;
        return r;
    }
    size_t probe(uint64_t key) const {
        return (size_t)((key * uint64_t(0x9E3779B97F4A7C15ULL)) >> shift_);
    }
    void grow() {
        size_t new_cap = slots_.empty() ? 16 : slots_.size() * 2;
        std::vector<Slot> old = std::move(slots_);
        slots_.assign(new_cap, Slot{EMPTY_KEY, V{}});
        shift_ = 64 - log2_pot(new_cap);
        size_ = 0;
        for (auto& s : old)
            if (s.key != EMPTY_KEY) insert_impl(s.key, std::move(s.value));
    }
    void insert_impl(uint64_t key, V val) {
        size_t mask = slots_.size() - 1;
        size_t i = probe(key);
        while (slots_[i].key != EMPTY_KEY) {
            if (slots_[i].key == key) { slots_[i].value = std::move(val); return; }
            i = (i + 1) & mask;
        }
        slots_[i].key = key;
        slots_[i].value = std::move(val);
        ++size_;
    }
public:
    FlatMap64() = default;
    void reserve(size_t n) {
        size_t need = n * 2;
        if (need <= slots_.size()) return;
        size_t cap = 16;
        while (cap < need) cap *= 2;
        std::vector<Slot> old = std::move(slots_);
        slots_.assign(cap, Slot{EMPTY_KEY, V{}});
        shift_ = 64 - log2_pot(cap);
        size_ = 0;
        for (auto& s : old)
            if (s.key != EMPTY_KEY) insert_impl(s.key, std::move(s.value));
    }
    V* find(uint64_t key) {
        if (slots_.empty()) return nullptr;
        size_t mask = slots_.size() - 1;
        size_t i = probe(key);
        while (slots_[i].key != EMPTY_KEY) {
            if (slots_[i].key == key) return &slots_[i].value;
            i = (i + 1) & mask;
        }
        return nullptr;
    }
    const V* find(uint64_t key) const {
        if (slots_.empty()) return nullptr;
        size_t mask = slots_.size() - 1;
        size_t i = probe(key);
        while (slots_[i].key != EMPTY_KEY) {
            if (slots_[i].key == key) return &slots_[i].value;
            i = (i + 1) & mask;
        }
        return nullptr;
    }
    V& operator[](uint64_t key) {
        if (size_ * 2 >= slots_.size()) grow();
        size_t mask = slots_.size() - 1;
        size_t i = probe(key);
        while (slots_[i].key != EMPTY_KEY) {
            if (slots_[i].key == key) return slots_[i].value;
            i = (i + 1) & mask;
        }
        slots_[i].key = key;
        slots_[i].value = V{};
        ++size_;
        return slots_[i].value;
    }
    void erase(uint64_t key) {
        if (slots_.empty()) return;
        size_t mask = slots_.size() - 1;
        size_t i = probe(key);
        while (slots_[i].key != EMPTY_KEY) {
            if (slots_[i].key == key) {
                --size_;
                size_t j = i;
                while (true) {
                    j = (j + 1) & mask;
                    if (slots_[j].key == EMPTY_KEY) break;
                    size_t k = probe(slots_[j].key);
                    bool move = (i <= j) ? (k <= i || k > j) : (k <= i && k > j);
                    if (move) { slots_[i] = std::move(slots_[j]); i = j; }
                }
                slots_[i].key = EMPTY_KEY;
                return;
            }
            i = (i + 1) & mask;
        }
    }
    void clear() {
        for (auto& s : slots_) s.key = EMPTY_KEY;
        size_ = 0;
    }
    size_t size() const { return size_; }
};

///////////////////////////////////////////////////////////////////////////////////////////
// Delaunay2D - Internal 2D constrained Delaunay triangulation engine (Bowyer-Watson)
///////////////////////////////////////////////////////////////////////////////////////////

struct Vertex2D {
    double x = 0.0;
    double y = 0.0;
};

struct Triangle {
    int v[3] = {-1, -1, -1};
    int adj[3] = {-1, -1, -1};
    bool constrained[3] = {false, false, false};
    bool alive = true;
};

class Delaunay2D {
public:
    std::vector<Vertex2D> vertices;
    std::vector<Triangle> triangles;
    int super_v[3] = {-1, -1, -1};

    FlatMap64<std::pair<int, int>> edge_map;

    static uint64_t edge_key(int a, int b) {
        return ((uint64_t)(uint32_t)std::min(a, b) << 32) | (uint64_t)(uint32_t)std::max(a, b);
    }

    Delaunay2D(double xmin, double ymin, double xmax, double ymax);

    int insert(double x, double y);
    void insert_constraint(int v0, int v1);
    void cleanup();
    std::vector<std::array<int, 3>> get_triangles() const;

    static double in_circumcircle(double ax, double ay, double bx, double by,
                                   double cx, double cy, double dx, double dy);
    static double orient2d(double ax, double ay, double bx, double by,
                           double cx, double cy);
    static void circumcenter(double ax, double ay, double bx, double by,
                             double cx, double cy, double& ux, double& uy);

    int locate(double x, double y, int start_tri = 0) const;

    int last_found_ = 0;

private:
    int visit_epoch_ = 0;
    std::vector<int> visit_stamp_;

    mutable std::vector<int> bad_;
    struct BEdge { int e0, e1; };
    mutable std::vector<BEdge> polygon_;

    void register_triangle_edges(int ti);
    void unregister_triangle_edges(int ti);
};

///////////////////////////////////////////////////////////////////////////////////////////
// TrimeshDelaunay - Surface-aware Delaunay mesher (public API)
///////////////////////////////////////////////////////////////////////////////////////////

class TrimeshDelaunay {
public:
    explicit TrimeshDelaunay(const NurbsSurface& surface);

    TrimeshDelaunay& set_max_angle(double degrees);
    TrimeshDelaunay& set_max_edge_length(double length);
    TrimeshDelaunay& set_min_edge_length(double length);
    TrimeshDelaunay& set_max_chord_height(double height);
    TrimeshDelaunay& set_max_iterations(int iterations);

    double get_max_angle() const { return m_max_angle; }
    double get_max_edge_length() const { return m_max_edge_length; }
    double get_min_edge_length() const { return m_min_edge_length; }
    double get_max_chord_height() const { return m_max_chord_height; }
    int get_max_iterations() const { return m_max_iterations; }

    Mesh mesh() const;

private:
    const NurbsSurface& m_surface;
    double m_max_angle = 20.0;
    double m_max_edge_length = 0.0;
    double m_min_edge_length = 0.0;
    double m_max_chord_height = 0.0;
    int m_max_iterations = 5000;

    double compute_bbox_diagonal() const;
};

} // namespace session_cpp
