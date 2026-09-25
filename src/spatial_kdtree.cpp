#include "spatial_kdtree.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

namespace session_cpp {

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
SpatialKDTree::SpatialKDTree(std::vector<Point> points) : _points(std::move(points)) {
    build();
}

// ═══════════════════════════════════════════════════════════════════════════
// Queries
// ═══════════════════════════════════════════════════════════════════════════
std::pair<int, double> SpatialKDTree::nearest(const Point& query) const {

    int best = 0;
    double best_d2 = std::numeric_limits<double>::infinity();

    Visit stack[STACK_SIZE];
    int top = 0;

    if (!_nodes.empty())
        push(stack, top, 0, 0.0);

    while (top > 0) {
        const Visit visit = stack[--top];

        if (visit.bound >= best_d2)
            continue;

        const Node& node = _nodes[visit.node];
        const double d2 = dist_sq(query, _points[node.idx]);

        if (d2 < best_d2) {
            best_d2 = d2;
            best = node.idx;
        }

        const double diff = query[node.axis] - _points[node.idx][node.axis];
        const int near = diff <= 0 ? node.left : node.right;
        const int far = diff <= 0 ? node.right : node.left;

        push(stack, top, far, diff * diff);
        push(stack, top, near, 0.0);
    }

    return {best, std::sqrt(best_d2)};
}

std::vector<std::pair<int, double>> SpatialKDTree::nearest_k(const Point& query, int k) const {

    std::vector<std::pair<int, double>> best;

    if (k <= 0)
        return best;

    Visit stack[STACK_SIZE];
    int top = 0;

    if (!_nodes.empty())
        push(stack, top, 0, 0.0);

    while (top > 0) {
        const Visit visit = stack[--top];
        const bool full = (int)best.size() == k;

        if (full && visit.bound >= best.back().second)
            continue;

        const Node& node = _nodes[visit.node];
        const double d2 = dist_sq(query, _points[node.idx]);

        if (!full || d2 < best.back().second)
            insert_sorted(best, node.idx, d2, k);

        const double diff = query[node.axis] - _points[node.idx][node.axis];
        const int near = diff <= 0 ? node.left : node.right;
        const int far = diff <= 0 ? node.right : node.left;

        push(stack, top, far, diff * diff);
        push(stack, top, near, 0.0);
    }

    for (std::pair<int, double>& hit : best)
        hit.second = std::sqrt(hit.second);

    return best;
}

/// Order two hits by distance.
static bool hit_before(const std::pair<int, double>& a, const std::pair<int, double>& b) {
    return a.second < b.second;
}

std::vector<std::pair<int, double>> SpatialKDTree::radius_search(const Point& query, double radius) const {

    std::vector<std::pair<int, double>> result;
    const double r2 = radius * radius;

    Visit stack[STACK_SIZE];
    int top = 0;

    if (!_nodes.empty())
        push(stack, top, 0, 0.0);

    while (top > 0) {
        const Visit visit = stack[--top];

        if (visit.bound > r2)
            continue;

        const Node& node = _nodes[visit.node];
        const double d2 = dist_sq(query, _points[node.idx]);

        if (d2 <= r2)
            result.emplace_back(node.idx, std::sqrt(d2));

        const double diff = query[node.axis] - _points[node.idx][node.axis];
        const int near = diff <= 0 ? node.left : node.right;
        const int far = diff <= 0 ? node.right : node.left;

        push(stack, top, far, diff * diff);
        push(stack, top, near, 0.0);
    }

    std::sort(result.begin(), result.end(), hit_before);

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Build
// ═══════════════════════════════════════════════════════════════════════════
void SpatialKDTree::build() {

    const int n = (int)_points.size();
    std::vector<int> indices(n);

    for (int i = 0; i < n; i++)
        indices[i] = i;

    _nodes.reserve(n);

    Range stack[STACK_SIZE];
    int top = 0;

    if (n > 0)
        stack[top++] = {0, n, 0, NULL_IDX, false};

    while (top > 0) {
        const Range range = stack[--top];
        const int axis = range.depth % 3;
        const int mid = range.lo + (range.hi - range.lo) / 2;

        std::nth_element(
            indices.begin() + range.lo,
            indices.begin() + mid,
            indices.begin() + range.hi,
            [&](int a, int b) { return _points[a][axis] < _points[b][axis]; }
        );

        const int node = (int)_nodes.size();
        _nodes.push_back({indices[mid], axis, NULL_IDX, NULL_IDX});

        if (range.parent != NULL_IDX && range.is_left)
            _nodes[range.parent].left = node;

        if (range.parent != NULL_IDX && !range.is_left)
            _nodes[range.parent].right = node;

        if (range.lo < mid) {
            assert(top < STACK_SIZE);
            stack[top++] = {range.lo, mid, range.depth + 1, node, true};
        }

        if (mid + 1 < range.hi) {
            assert(top < STACK_SIZE);
            stack[top++] = {mid + 1, range.hi, range.depth + 1, node, false};
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Traversal
// ═══════════════════════════════════════════════════════════════════════════
void SpatialKDTree::push(Visit stack[], int& top, int node, double bound) const {

    if (node == NULL_IDX)
        return;

    assert(top < STACK_SIZE);
    stack[top++] = {node, bound};
}

double SpatialKDTree::dist_sq(const Point& a, const Point& b) const {

    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    const double dz = a[2] - b[2];

    return dx * dx + dy * dy + dz * dz;
}

void SpatialKDTree::insert_sorted(std::vector<std::pair<int, double>>& best, int idx, double d2, int k) const {

    int pos = (int)best.size();

    while (pos > 0 && best[pos - 1].second > d2)
        pos--;

    best.insert(best.begin() + pos, {idx, d2});

    if ((int)best.size() > k)
        best.pop_back();
}

} // namespace session_cpp
