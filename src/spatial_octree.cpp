#include "spatial_octree.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace session_cpp {

// Duplicate points can never be separated by subdivision: below this level the node
// absorbs everything instead of recursing forever (spacing has shrunk by 2^21 anyway).
static const int MAX_LEVEL = 21;

SpatialOctree::SpatialOctree(std::vector<Point> points, double root_spacing, int leaf_capacity) {
    std::vector<double> coords;
    coords.reserve(points.size() * 3);
    for (const Point& p : points) {
        coords.push_back(p[0]);
        coords.push_back(p[1]);
        coords.push_back(p[2]);
    }
    init(coords, root_spacing, leaf_capacity);
}

// Coords are only read during construction - nothing is stored, so a renderer can
// hand its flat table over without a copy.
SpatialOctree SpatialOctree::from_coords(const std::vector<double>& coords, double root_spacing, int leaf_capacity) {
    SpatialOctree tree;
    tree.init(coords, root_spacing, leaf_capacity);
    return tree;
}

void SpatialOctree::init(const std::vector<double>& coords, double root_spacing, int leaf_capacity) {
    int n = static_cast<int>(coords.size() / 3);
    if (n == 0) {
        return;
    }
    std::array<double, 3> lo = {coords[0], coords[1], coords[2]};
    std::array<double, 3> hi = lo;
    for (int i = 1; i < n; i++) {
        for (int k = 0; k < 3; k++) {
            lo[k] = std::min(lo[k], coords[i * 3 + k]);
            hi[k] = std::max(hi[k], coords[i * 3 + k]);
        }
    }
    double size = std::max(hi[0] - lo[0], std::max(hi[1] - lo[1], hi[2] - lo[2]));
    if (size <= 0.0) {
        size = 1.0;
    }
    std::array<double, 3> root_min;
    for (int k = 0; k < 3; k++) {
        root_min[k] = (lo[k] + hi[k]) * 0.5 - size * 0.5;
    }
    std::vector<int> idxs(n);
    for (int i = 0; i < n; i++) {
        idxs[i] = i;
    }
    build(coords, root_min, size, 0, root_spacing, idxs, leaf_capacity);
}

int SpatialOctree::build(const std::vector<double>& coords, const std::array<double, 3>& min, double size, int level, double spacing, const std::vector<int>& idxs, int leaf_capacity) {
    int node_id = static_cast<int>(_nodes.size());
    _nodes.push_back(Node{min, size, level, spacing, static_cast<int>(_order.size()), 0, {-1, -1, -1, -1, -1, -1, -1, -1}});
    if (static_cast<int>(idxs.size()) <= leaf_capacity || level >= MAX_LEVEL) {
        _nodes[node_id].count = static_cast<int>(idxs.size());
        _order.insert(_order.end(), idxs.begin(), idxs.end());
        return node_id;
    }
    long long cells = std::max(1LL, static_cast<long long>(std::ceil(size / spacing)));
    std::array<double, 3> center;
    for (int k = 0; k < 3; k++) {
        center[k] = min[k] + size * 0.5;
    }
    std::set<std::array<long long, 3>> seen;
    std::vector<int> accepted;
    std::array<std::vector<int>, 8> buckets;
    for (int i : idxs) {
        std::array<long long, 3> key;
        for (int k = 0; k < 3; k++) {
            long long c = static_cast<long long>(std::floor((coords[i * 3 + k] - min[k]) / spacing));
            key[k] = std::min(std::max(c, 0LL), cells - 1);
        }
        if (seen.insert(key).second) {
            accepted.push_back(i);
        } else {
            int b = 0;
            if (coords[i * 3] >= center[0]) b |= 1;
            if (coords[i * 3 + 1] >= center[1]) b |= 2;
            if (coords[i * 3 + 2] >= center[2]) b |= 4;
            buckets[b].push_back(i);
        }
    }
    _nodes[node_id].count = static_cast<int>(accepted.size());
    _order.insert(_order.end(), accepted.begin(), accepted.end());
    double half = size * 0.5;
    for (int b = 0; b < 8; b++) {
        if (!buckets[b].empty()) {
            std::array<double, 3> child_min = {
                min[0] + (b & 1) * half,
                min[1] + ((b >> 1) & 1) * half,
                min[2] + ((b >> 2) & 1) * half,
            };
            int child_id = build(coords, child_min, half, level + 1, spacing * 0.5, buckets[b], leaf_capacity);
            _nodes[node_id].children[b] = child_id;
        }
    }
    return node_id;
}

int SpatialOctree::node_count() const {
    return static_cast<int>(_nodes.size());
}

std::pair<Point, double> SpatialOctree::node_cube(int i) const {
    const Node& node = _nodes[i];
    double half = node.size * 0.5;
    return {Point(node.min[0] + half, node.min[1] + half, node.min[2] + half), node.size};
}

int SpatialOctree::node_level(int i) const {
    return _nodes[i].level;
}

double SpatialOctree::node_spacing(int i) const {
    return _nodes[i].spacing;
}

std::pair<int, int> SpatialOctree::node_range(int i) const {
    const Node& node = _nodes[i];
    return {node.first, node.count};
}

std::vector<int> SpatialOctree::children(int i) const {
    std::vector<int> result;
    for (int c : _nodes[i].children) {
        if (c >= 0) {
            result.push_back(c);
        }
    }
    return result;
}

const std::vector<int>& SpatialOctree::order() const {
    return _order;
}

} // namespace session_cpp
