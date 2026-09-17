#include "spatial_octree.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <set>

namespace session_cpp {

SpatialOctree::SpatialOctree(std::vector<Point> points, double root_spacing, int leaf_capacity) {

    std::vector<double> coords;
    coords.reserve(points.size() * 3);

    for (const Point& p : points) {
        coords.push_back(p[0]);
        coords.push_back(p[1]);
        coords.push_back(p[2]);
    }

    build(coords, root_spacing, leaf_capacity);
}

SpatialOctree SpatialOctree::from_coords(const std::vector<double>& coords, double root_spacing, int leaf_capacity) {
    SpatialOctree tree;
    tree.build(coords, root_spacing, leaf_capacity);

    return tree;
}

std::pair<std::array<double, 3>, double> SpatialOctree::root_cube(const std::vector<double>& coords) const {

    const int n = (int)coords.size() / 3;
    std::array<double, 3> lo = {coords[0], coords[1], coords[2]};
    std::array<double, 3> hi = lo;

    for (int i = 1; i < n; i++)
        for (int k = 0; k < 3; k++) {
            lo[k] = std::min(lo[k], coords[i * 3 + k]);
            hi[k] = std::max(hi[k], coords[i * 3 + k]);
        }

    double size = std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]});

    if (size <= 0.0)
        size = 1.0;

    std::array<double, 3> min;

    for (int k = 0; k < 3; k++)
        min[k] = (lo[k] + hi[k]) * 0.5 - size * 0.5;

    return {min, size};
}

void SpatialOctree::build(const std::vector<double>& coords, double root_spacing, int leaf_capacity) {

    const int n = (int)coords.size() / 3;

    if (n == 0)
        return;

    const std::pair<std::array<double, 3>, double> root = root_cube(coords);
    std::vector<int> indices(n);

    for (int i = 0; i < n; i++)
        indices[i] = i;

    Task stack[STACK_SIZE];
    int top = 0;
    push(stack, top, {root.first, root.second, 0, root_spacing, 0, n, NULL_IDX, 0});

    while (top > 0) {
        const Task task = stack[--top];
        const int node = (int)_nodes.size();
        _nodes.push_back({task.min, task.size, task.level, task.spacing, (int)_order.size(), 0, {NULL_IDX, NULL_IDX, NULL_IDX, NULL_IDX, NULL_IDX, NULL_IDX, NULL_IDX, NULL_IDX}});

        if (task.parent != NULL_IDX)
            _nodes[task.parent].children[task.octant] = node;

        if (task.hi - task.lo <= leaf_capacity || task.level >= MAX_LEVEL) {
            _order.insert(_order.end(), indices.begin() + task.lo, indices.begin() + task.hi);
            _nodes[node].count = task.hi - task.lo;
            continue;
        }

        const std::array<int, 9> bounds = accept(coords, task, indices);
        _nodes[node].count = (int)_order.size() - _nodes[node].first;
        const double half = task.size * 0.5;

        for (int b = 7; b >= 0; b--) {
            if (bounds[b] == bounds[b + 1])
                continue;

            const std::array<double, 3> min = {
                task.min[0] + (b & 1) * half,
                task.min[1] + ((b >> 1) & 1) * half,
                task.min[2] + ((b >> 2) & 1) * half,
            };
            push(stack, top, {min, half, task.level + 1, task.spacing * 0.5, bounds[b], bounds[b + 1], node, b});
        }
    }
}

std::array<int, 9> SpatialOctree::accept(const std::vector<double>& coords, const Task& task, std::vector<int>& indices) {

    const long long cells = std::max(1LL, (long long)std::ceil(task.size / task.spacing));
    const double half = task.size * 0.5;
    const std::array<double, 3> center = {task.min[0] + half, task.min[1] + half, task.min[2] + half};
    std::set<std::array<long long, 3>> seen;
    std::array<std::vector<int>, 8> buckets;

    for (int i = task.lo; i < task.hi; i++) {
        const int idx = indices[i];
        std::array<long long, 3> key;

        for (int k = 0; k < 3; k++)
            key[k] = std::clamp((long long)std::floor((coords[idx * 3 + k] - task.min[k]) / task.spacing), 0LL, cells - 1);

        if (seen.insert(key).second) {
            _order.push_back(idx);
            continue;
        }

        int octant = 0;

        for (int k = 0; k < 3; k++)
            if (coords[idx * 3 + k] >= center[k])
                octant |= 1 << k;

        buckets[octant].push_back(idx);
    }

    std::array<int, 9> bounds;
    bounds[0] = task.lo;

    for (int b = 0; b < 8; b++) {
        bounds[b + 1] = bounds[b] + (int)buckets[b].size();
        std::copy(buckets[b].begin(), buckets[b].end(), indices.begin() + bounds[b]);
    }

    return bounds;
}

void SpatialOctree::push(Task stack[], int& top, const Task& task) const {
    assert(top < STACK_SIZE);
    stack[top++] = task;
}

int SpatialOctree::node_count() const {
    return (int)_nodes.size();
}

std::pair<Point, double> SpatialOctree::node_cube(int i) const {
    const Node& node = _nodes[i];
    const double half = node.size * 0.5;

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

    for (int c : _nodes[i].children)
        if (c != NULL_IDX)
            result.push_back(c);

    return result;
}

const std::vector<int>& SpatialOctree::order() const {
    return _order;
}

} // namespace session_cpp
