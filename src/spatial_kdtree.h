#pragma once

#include "point.h"
#include <utility>
#include <vector>

namespace session_cpp {

/// KD-tree with alternating-axis median split over points for nearest, k-nearest and radius queries.
class SpatialKDTree {
public:
    explicit SpatialKDTree(std::vector<Point> points);

    std::pair<int, double> nearest(const Point& query) const;
    std::vector<std::pair<int, double>> nearest_k(const Point& query, int k) const;
    std::vector<std::pair<int, double>> radius_search(const Point& query, double radius) const;

private:
    static const int STACK_SIZE = 64;
    static const int NULL_IDX = -1;

    struct Node {
        int idx;
        int axis;
        int left;
        int right;
    };

    struct Range {
        int lo;
        int hi;
        int depth;
        int parent;
        bool is_left;
    };

    struct Visit {
        int node;
        double bound;
    };

    std::vector<Point> _points;
    std::vector<Node> _nodes;

    void build();
    void push(Visit stack[], int& top, int node, double bound) const;
    double dist_sq(const Point& a, const Point& b) const;
    void insert_sorted(std::vector<std::pair<int, double>>& best, int idx, double d2, int k) const;
};

} // namespace session_cpp
