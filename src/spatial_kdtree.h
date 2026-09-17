#pragma once

#include "point.h"
#include <utility>
#include <vector>

namespace session_cpp {

/// KD-tree with alternating-axis median split over points for nearest, k-nearest and radius queries.
class SpatialKDTree {
public:
    /// Construct the tree over points.
    explicit SpatialKDTree(std::vector<Point> points);

    /// Return the index and distance of the nearest point.
    std::pair<int, double> nearest(const Point& query) const;

    /// Return the k nearest (index, distance) pairs sorted by distance.
    std::vector<std::pair<int, double>> nearest_k(const Point& query, int k) const;

    /// Return every (index, distance) pair within radius sorted by distance.
    std::vector<std::pair<int, double>> radius_search(const Point& query, double radius) const;

private:
    static const int STACK_SIZE = 64; // Explicit stack depth, covers any binary tree over int.
    static const int NULL_IDX = -1; // Missing child marker.

    struct Node {
        int idx; // Point index.
        int axis; // Split axis (0=x, 1=y, 2=z).
        int left; // Left child node or NULL_IDX.
        int right; // Right child node or NULL_IDX.
    };

    struct Range {
        int lo; // Range start.
        int hi; // Range end, exclusive.
        int depth; // Depth from the root.
        int parent; // Parent node or NULL_IDX.
        bool is_left; // True when this range is the parent's left child.
    };

    struct Visit {
        int node; // Node to visit.
        double bound; // Lower bound on the squared distance to the node's half-space.
    };

    std::vector<Point> _points; // Indexed points.
    std::vector<Node> _nodes; // Nodes in build order, root first.

    /// Build the nodes by iterative median splits over an explicit stack.
    void build();

    /// Push a node with its bound onto the visit stack.
    void push(Visit stack[], int& top, int node, double bound) const;

    /// Return the squared distance between a and b.
    double dist_sq(const Point& a, const Point& b) const;

    /// Insert (idx, d2) into best keeping it sorted and at most k long.
    void insert_sorted(std::vector<std::pair<int, double>>& best, int idx, double d2, int k) const;
};

} // namespace session_cpp
