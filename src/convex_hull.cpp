#include "convex_hull.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace session_cpp {

double ConvexHull::cross_2d(const Point& o, const Point& a, const Point& b) {
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

std::vector<Point> ConvexHull::hull_2d(const std::vector<Point>& points) {
    int n = static_cast<int>(points.size());
    if (n < 3) return points;
    std::vector<const Point*> pts;
    for (const auto& p : points) pts.push_back(&p);
    std::sort(pts.begin(), pts.end(), [](const Point* a, const Point* b) {
        return (*a)[0] < (*b)[0] || ((*a)[0] == (*b)[0] && (*a)[1] < (*b)[1]);
    });
    std::vector<const Point*> lower;
    for (auto* p : pts) {
        while (lower.size() >= 2 && cross_2d(*lower[lower.size() - 2], *lower[lower.size() - 1], *p) <= 0.0)
            lower.pop_back();
        lower.push_back(p);
    }
    std::vector<const Point*> upper;
    for (int i = n - 1; i >= 0; --i) {
        while (upper.size() >= 2 && cross_2d(*upper[upper.size() - 2], *upper[upper.size() - 1], *pts[i]) <= 0.0)
            upper.pop_back();
        upper.push_back(pts[i]);
    }
    lower.pop_back();
    upper.pop_back();
    std::vector<Point> result;
    for (auto* p : lower) result.push_back(*p);
    for (auto* p : upper) result.push_back(*p);
    return result;
}

std::tuple<double, double, double> ConvexHull::normal(const Point& a, const Point& b, const Point& c) {
    double ax = b[0] - a[0], ay = b[1] - a[1], az = b[2] - a[2];
    double bx = c[0] - a[0], by = c[1] - a[1], bz = c[2] - a[2];
    return {ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx};
}

double ConvexHull::signed_volume(const Point& a, const Point& b, const Point& c, const Point& d) {
    auto [nx, ny, nz] = normal(a, b, c);
    return nx * (d[0] - a[0]) + ny * (d[1] - a[1]) + nz * (d[2] - a[2]);
}

int ConvexHull::farthest_point(const std::vector<int>& pts_idx, const std::vector<Point>& points, const Point& a, const Point& b, const Point& c) {
    int best_idx = -1;
    double best_vol = 0.0;
    for (int i : pts_idx) {
        double v = signed_volume(a, b, c, points[i]);
        if (v > best_vol) { best_vol = v; best_idx = i; }
    }
    return best_idx;
}

std::vector<int> ConvexHull::visible_from(const std::vector<int>& pts_idx, const std::vector<Point>& points, const Point& a, const Point& b, const Point& c) {
    std::vector<int> result;
    for (int i : pts_idx) {
        if (signed_volume(a, b, c, points[i]) > 1e-10) result.push_back(i);
    }
    return result;
}

void ConvexHull::quickhull_3d_faces(const std::vector<Point>& points, const std::vector<int>& pts_idx, int a, int b, int c, std::vector<std::tuple<int, int, int>>& faces) {
    std::vector<int> vis = visible_from(pts_idx, points, points[a], points[b], points[c]);
    if (vis.empty()) { faces.emplace_back(a, b, c); return; }
    int apex = farthest_point(vis, points, points[a], points[b], points[c]);
    if (apex == -1) { faces.emplace_back(a, b, c); return; }
    auto v_ab = visible_from(vis, points, points[a], points[b], points[apex]);
    auto v_bc = visible_from(vis, points, points[b], points[c], points[apex]);
    auto v_ca = visible_from(vis, points, points[c], points[a], points[apex]);
    quickhull_3d_faces(points, v_ab, a, b, apex, faces);
    quickhull_3d_faces(points, v_bc, b, c, apex, faces);
    quickhull_3d_faces(points, v_ca, c, a, apex, faces);
}

Mesh ConvexHull::hull_3d(const std::vector<Point>& points) {
    int n = static_cast<int>(points.size());
    Mesh mesh;
    if (n < 4) {
        std::vector<size_t> vkeys;
        for (const auto& p : points) vkeys.push_back(mesh.add_vertex(p));
        if (n == 3) mesh.add_face(vkeys);
        return mesh;
    }
    int p0 = 0;
    for (int i = 1; i < n; ++i) if (points[i][0] < points[p0][0]) p0 = i;
    int p1 = 0;
    for (int i = 0; i < n; ++i) {
        double d = (points[i][0]-points[p0][0])*(points[i][0]-points[p0][0])+(points[i][1]-points[p0][1])*(points[i][1]-points[p0][1])+(points[i][2]-points[p0][2])*(points[i][2]-points[p0][2]);
        double d1 = (points[p1][0]-points[p0][0])*(points[p1][0]-points[p0][0])+(points[p1][1]-points[p0][1])*(points[p1][1]-points[p0][1])+(points[p1][2]-points[p0][2])*(points[p1][2]-points[p0][2]);
        if (d > d1) p1 = i;
    }
    double ax = points[p1][0]-points[p0][0], ay = points[p1][1]-points[p0][1], az = points[p1][2]-points[p0][2];
    auto dist_line = [&](int i) {
        double bx = points[i][0]-points[p0][0], by = points[i][1]-points[p0][1], bz = points[i][2]-points[p0][2];
        double cx = ay*bz-az*by, cy = az*bx-ax*bz, cz = ax*by-ay*bx;
        return cx*cx+cy*cy+cz*cz;
    };
    int p2 = -1;
    double best2 = -1.0;
    for (int i = 0; i < n; ++i) { if (i == p0 || i == p1) continue; double d = dist_line(i); if (d > best2) { best2 = d; p2 = i; } }
    int p3 = -1;
    double best3 = -1.0;
    for (int i = 0; i < n; ++i) { if (i == p0 || i == p1 || i == p2) continue; double v = std::abs(signed_volume(points[p0], points[p1], points[p2], points[i])); if (v > best3) { best3 = v; p3 = i; } }
    if (signed_volume(points[p0], points[p1], points[p2], points[p3]) > 0.0) std::swap(p1, p2);
    std::vector<int> rest;
    for (int i = 0; i < n; ++i) if (i != p0 && i != p1 && i != p2 && i != p3) rest.push_back(i);
    std::vector<std::tuple<int, int, int>> faces;
    quickhull_3d_faces(points, rest, p0, p1, p2, faces);
    quickhull_3d_faces(points, rest, p0, p3, p1, faces);
    quickhull_3d_faces(points, rest, p1, p3, p2, faces);
    quickhull_3d_faces(points, rest, p2, p3, p0, faces);
    std::set<int> used_set;
    for (auto& [a, b, c] : faces) { used_set.insert(a); used_set.insert(b); used_set.insert(c); }
    std::vector<size_t> idx_to_vkey(n, 0);
    for (int idx : used_set) idx_to_vkey[idx] = mesh.add_vertex(points[idx]);
    for (auto& [a, b, c] : faces) mesh.add_face({idx_to_vkey[a], idx_to_vkey[b], idx_to_vkey[c]});
    return mesh;
}

} // namespace session_cpp
