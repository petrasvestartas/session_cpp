#include "convex_hull.h"
#include "vector.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <set>

namespace session_cpp {
namespace {

/// Twice the signed area of o-a-b in XY, positive for a left turn
double cross_2d(const Point& o, const Point& a, const Point& b) {
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

/// Appends point i to the chain after popping every tail that no longer turns left towards it
void extend_chain(const std::vector<Point>& points, std::vector<int>& chain, int i) {
    while (chain.size() >= 2 && cross_2d(points[chain[chain.size() - 2]], points[chain[chain.size() - 1]], points[i]) <= 0.0)
        chain.pop_back();
    chain.push_back(i);
}

/// Six times the signed volume of a-b-c-d, positive when d is on the normal side of a-b-c
double signed_volume(const Point& a, const Point& b, const Point& c, const Point& d) {
    return (b - a).cross(c - a).dot(d - a);
}

/// Indices of the points above the face a-b-c
std::vector<int> visible_from(const std::vector<int>& indices, const std::vector<Point>& points, const Point& a, const Point& b, const Point& c) {
    std::vector<int> result;
    for (int i : indices)
        if (signed_volume(a, b, c, points[i]) > 1e-10)
            result.push_back(i);
    return result;
}

/// Index of the point highest above the face a-b-c, -1 when none is above
int farthest_point(const std::vector<int>& indices, const std::vector<Point>& points, const Point& a, const Point& b, const Point& c) {
    int best = -1;
    double best_volume = 0.0;
    for (int i : indices) {
        const double volume = signed_volume(a, b, c, points[i]);
        if (volume > best_volume) {
            best_volume = volume;
            best = i;
        }
    }
    return best;
}

/// Hull faces over a-b-c: the face itself when no candidate is above it, else the three faces to the farthest candidate, recursively
void quickhull_faces(const std::vector<Point>& points, const std::vector<int>& indices, int a, int b, int c, std::vector<std::array<int, 3>>& faces) {
    const std::vector<int> visible = visible_from(indices, points, points[a], points[b], points[c]);
    const int apex = farthest_point(visible, points, points[a], points[b], points[c]);
    if (apex == -1) {
        faces.push_back({a, b, c});
        return;
    }
    quickhull_faces(points, visible_from(visible, points, points[a], points[b], points[apex]), a, b, apex, faces);
    quickhull_faces(points, visible_from(visible, points, points[b], points[c], points[apex]), b, c, apex, faces);
    quickhull_faces(points, visible_from(visible, points, points[c], points[a], points[apex]), c, a, apex, faces);
}

} // namespace

std::vector<Point> ConvexHull::hull_2d(const std::vector<Point>& points) {
    const int n = static_cast<int>(points.size());
    if (n < 3)
        return points;
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&points](int a, int b) { return points[a][0] < points[b][0] || (points[a][0] == points[b][0] && points[a][1] < points[b][1]); });
    std::vector<int> lower;
    for (int i : order)
        extend_chain(points, lower, i);
    std::vector<int> upper;
    for (int i = n - 1; i >= 0; --i)
        extend_chain(points, upper, order[i]);
    lower.pop_back();
    upper.pop_back();
    std::vector<Point> hull;
    for (int i : lower)
        hull.push_back(points[i]);
    for (int i : upper)
        hull.push_back(points[i]);
    return hull;
}

Mesh ConvexHull::hull_3d(const std::vector<Point>& points) {
    const int n = static_cast<int>(points.size());
    Mesh mesh;
    if (n < 4) {
        std::vector<size_t> vkeys;
        for (const Point& point : points)
            vkeys.push_back(mesh.add_vertex(point));
        if (n == 3)
            mesh.add_face(vkeys);
        return mesh;
    }
    int p0 = 0;
    for (int i = 1; i < n; ++i)
        if (points[i][0] < points[p0][0])
            p0 = i;
    int p1 = 0;
    for (int i = 1; i < n; ++i)
        if ((points[i] - points[p0]).magnitude_squared() > (points[p1] - points[p0]).magnitude_squared())
            p1 = i;
    const Vector axis = points[p1] - points[p0];
    int p2 = -1;
    double best_distance = -1.0;
    for (int i = 0; i < n; ++i) {
        if (i == p0 || i == p1)
            continue;
        const double distance = axis.cross(points[i] - points[p0]).magnitude_squared();
        if (distance > best_distance) {
            best_distance = distance;
            p2 = i;
        }
    }
    int p3 = -1;
    double best_volume = -1.0;
    for (int i = 0; i < n; ++i) {
        if (i == p0 || i == p1 || i == p2)
            continue;
        const double volume = std::abs(signed_volume(points[p0], points[p1], points[p2], points[i]));
        if (volume > best_volume) {
            best_volume = volume;
            p3 = i;
        }
    }
    if (p2 < 0 || p3 < 0 || best_distance <= 1e-20 || best_volume <= 1e-20) {
        for (const Point& point : points) mesh.add_vertex(point);
        return mesh;
    }
    if (signed_volume(points[p0], points[p1], points[p2], points[p3]) > 0.0)
        std::swap(p1, p2);
    std::vector<int> rest;
    for (int i = 0; i < n; ++i)
        if (i != p0 && i != p1 && i != p2 && i != p3)
            rest.push_back(i);
    std::vector<std::array<int, 3>> faces;
    quickhull_faces(points, rest, p0, p1, p2, faces);
    quickhull_faces(points, rest, p0, p3, p1, faces);
    quickhull_faces(points, rest, p1, p3, p2, faces);
    quickhull_faces(points, rest, p2, p3, p0, faces);
    std::set<int> used;
    for (const std::array<int, 3>& face : faces)
        used.insert(face.begin(), face.end());
    std::vector<size_t> vkeys(n, 0);
    for (int i : used)
        vkeys[i] = mesh.add_vertex(points[i]);
    for (const std::array<int, 3>& face : faces)
        mesh.add_face({vkeys[face[0]], vkeys[face[1]], vkeys[face[2]]});
    return mesh;
}

} // namespace session_cpp
