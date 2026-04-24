// SpatialKDTree — alternating-axis median split over bare 3D points.
// Use for: k-nearest-neighbor queries on point clouds (fastest option).
//   Points only — no volumes, no boxes, no rotation.
// Prefer over SpatialAABBTree/SpatialBVH when data is a point cloud, not triangle faces.
// Prefer over SpatialRTree   when queries are k-NN, not region overlap.
// Note: static structure; rebuild required after point insertion.
#pragma once
#include "point.h"
#include <memory>
#include <vector>

namespace session_cpp {

/**
 * @class SpatialKDTree
 * @brief KD-tree for point-to-point nearest-neighbor queries.
 *
 * Build on construction using alternating-axis median split.
 * Complements SpatialRTree (box queries) and SpatialBVH (collision/ray).
 */
class SpatialKDTree {
public:
    explicit SpatialKDTree(std::vector<Point> points);

    std::pair<int, double> nearest(const Point& query) const;
    std::vector<std::pair<int, double>> nearest_k(const Point& query, int k) const;
    std::vector<std::pair<int, double>> radius_search(const Point& query, double radius) const;

private:
    struct Node {
        int idx;
        int axis;
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;
    };

    std::vector<Point> _points;
    std::unique_ptr<Node> _root;

    static double dist_sq(const Point& a, const Point& b);
    static std::unique_ptr<Node> build(const std::vector<Point>& pts, std::vector<int>& indices, int depth);
    static void nearest_1(const Node* node, const std::vector<Point>& pts, const Point& q, int& best_idx, double& best_d2);
    static void nearest_k_rec(const Node* node, const std::vector<Point>& pts, const Point& q, int k, std::vector<std::pair<double, int>>& heap);
    static void radius_rec(const Node* node, const std::vector<Point>& pts, const Point& q, double r2, std::vector<std::pair<int, double>>& result);
};

} // namespace session_cpp
