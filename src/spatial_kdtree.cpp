#include "spatial_kdtree.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace session_cpp {

SpatialKDTree::SpatialKDTree(std::vector<Point> points) : _points(std::move(points)) {
    if (!_points.empty()) {
        std::vector<int> idx(_points.size());
        for (int i = 0; i < (int)idx.size(); ++i) idx[i] = i;
        _root = build(_points, idx, 0);
    }
}

double SpatialKDTree::dist_sq(const Point& a, const Point& b) {
    double dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
    return dx*dx + dy*dy + dz*dz;
}

std::unique_ptr<SpatialKDTree::Node> SpatialKDTree::build(const std::vector<Point>& pts, std::vector<int>& indices, int depth) {
    if (indices.empty()) return nullptr;
    int axis = depth % 3;
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        return pts[a][axis] < pts[b][axis];
    });
    int mid = static_cast<int>(indices.size()) / 2;
    auto node = std::make_unique<Node>();
    node->idx = indices[mid];
    node->axis = axis;
    std::vector<int> left_idx(indices.begin(), indices.begin() + mid);
    std::vector<int> right_idx(indices.begin() + mid + 1, indices.end());
    node->left = build(pts, left_idx, depth + 1);
    node->right = build(pts, right_idx, depth + 1);
    return node;
}

void SpatialKDTree::nearest_1(const Node* node, const std::vector<Point>& pts, const Point& q, int& best_idx, double& best_d2) {
    if (!node) return;
    double d = dist_sq(q, pts[node->idx]);
    if (d < best_d2) { best_d2 = d; best_idx = node->idx; }
    double diff = q[node->axis] - pts[node->idx][node->axis];
    const Node* near = (diff <= 0) ? node->left.get() : node->right.get();
    const Node* far  = (diff <= 0) ? node->right.get() : node->left.get();
    nearest_1(near, pts, q, best_idx, best_d2);
    if (diff * diff < best_d2) nearest_1(far, pts, q, best_idx, best_d2);
}

std::pair<int, double> SpatialKDTree::nearest(const Point& query) const {
    int best_idx = 0;
    double best_d2 = std::numeric_limits<double>::infinity();
    nearest_1(_root.get(), _points, query, best_idx, best_d2);
    return {best_idx, std::sqrt(best_d2)};
}

void SpatialKDTree::nearest_k_rec(const Node* node, const std::vector<Point>& pts, const Point& q, int k, std::vector<std::pair<double, int>>& heap) {
    if (!node) return;
    double d = dist_sq(q, pts[node->idx]);
    if ((int)heap.size() < k) {
        heap.emplace_back(d, node->idx);
        std::sort(heap.begin(), heap.end(), [](const auto& a, const auto& b){ return a.first > b.first; });
    } else if (d < heap[0].first) {
        heap[0] = {d, node->idx};
        std::sort(heap.begin(), heap.end(), [](const auto& a, const auto& b){ return a.first > b.first; });
    }
    double diff = q[node->axis] - pts[node->idx][node->axis];
    const Node* near = (diff <= 0) ? node->left.get() : node->right.get();
    const Node* far  = (diff <= 0) ? node->right.get() : node->left.get();
    nearest_k_rec(near, pts, q, k, heap);
    if ((int)heap.size() < k || diff * diff < heap[0].first) nearest_k_rec(far, pts, q, k, heap);
}

std::vector<std::pair<int, double>> SpatialKDTree::nearest_k(const Point& query, int k) const {
    std::vector<std::pair<double, int>> heap;
    nearest_k_rec(_root.get(), _points, query, k, heap);
    std::vector<std::pair<int, double>> result;
    for (auto& [d2, i] : heap) result.emplace_back(i, std::sqrt(d2));
    std::sort(result.begin(), result.end(), [](const auto& a, const auto& b){ return a.second < b.second; });
    return result;
}

void SpatialKDTree::radius_rec(const Node* node, const std::vector<Point>& pts, const Point& q, double r2, std::vector<std::pair<int, double>>& result) {
    if (!node) return;
    double d = dist_sq(q, pts[node->idx]);
    if (d <= r2) result.emplace_back(node->idx, std::sqrt(d));
    double diff = q[node->axis] - pts[node->idx][node->axis];
    const Node* near = (diff <= 0) ? node->left.get() : node->right.get();
    const Node* far  = (diff <= 0) ? node->right.get() : node->left.get();
    radius_rec(near, pts, q, r2, result);
    if (diff * diff <= r2) radius_rec(far, pts, q, r2, result);
}

std::vector<std::pair<int, double>> SpatialKDTree::radius_search(const Point& query, double radius) const {
    std::vector<std::pair<int, double>> result;
    radius_rec(_root.get(), _points, query, radius * radius, result);
    std::sort(result.begin(), result.end(), [](const auto& a, const auto& b){ return a.second < b.second; });
    return result;
}

} // namespace session_cpp
