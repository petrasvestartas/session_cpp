#include "aabb.h"
#include "line.h"
#include "polyline.h"
#include "mesh.h"
#include "pointcloud.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace session_cpp {

// ── AABB factory methods ──────────────────────────────────────────────────────

AABB AABB::from_point(const Point& point, double inflate) {
    return AABB(point[0], point[1], point[2], inflate, inflate, inflate);
}

AABB AABB::from_points(const std::vector<Point>& points, double inflate) {
    if (points.empty()) return AABB();
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double min_z = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    double max_z = std::numeric_limits<double>::lowest();
    for (const auto& pt : points) {
        min_x = std::min(min_x, pt[0]); max_x = std::max(max_x, pt[0]);
        min_y = std::min(min_y, pt[1]); max_y = std::max(max_y, pt[1]);
        min_z = std::min(min_z, pt[2]); max_z = std::max(max_z, pt[2]);
    }
    return AABB(
        (min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5,
        (max_x - min_x) * 0.5 + inflate, (max_y - min_y) * 0.5 + inflate, (max_z - min_z) * 0.5 + inflate
    );
}

AABB AABB::from_line(const Line& line, double inflate) {
    return from_points({line.start(), line.end()}, inflate);
}

AABB AABB::from_polyline(const Polyline& polyline, double inflate) {
    return from_points(polyline.get_points(), inflate);
}

AABB AABB::from_mesh(const Mesh& mesh, double inflate) {
    auto [vertices, faces] = mesh.to_vertices_and_faces();
    return from_points(vertices, inflate);
}

AABB AABB::from_pointcloud(const PointCloud& pointcloud, double inflate) {
    return from_points(pointcloud.get_points(), inflate);
}

AABB AABB::from_nurbssurface(const NurbsSurface& surface, double inflate) {
    if (!surface.is_valid() || surface.cv_count(0) == 0 || surface.cv_count(1) == 0) return AABB();
    std::vector<Point> points;
    for (int i = 0; i < surface.cv_count(0); i++)
        for (int j = 0; j < surface.cv_count(1); j++)
            points.push_back(surface.get_cv(i, j));
    return from_points(points, inflate);
}

AABB AABB::from_nurbscurve(const NurbsCurve& curve, double inflate, bool tight) {
    if (!curve.is_valid() || curve.cv_count() == 0) return AABB();
    if (!tight) {
        std::vector<Point> points;
        for (int i = 0; i < curve.cv_count(); i++) points.push_back(curve.get_cv(i));
        return from_points(points, inflate);
    }
    auto [t0, t1] = curve.domain();
    std::vector<Point> extrema_points;
    extrema_points.push_back(curve.point_at(t0));
    extrema_points.push_back(curve.point_at(t1));
    auto spans = curve.get_span_vector();
    for (double t : spans) {
        if (t > t0 && t < t1) extrema_points.push_back(curve.point_at(t));
    }
    const int NUM_SAMPLES = 20;
    double dt = (t1 - t0) / NUM_SAMPLES;
    for (int axis = 0; axis < 3; axis++) {
        for (int i = 0; i < NUM_SAMPLES; i++) {
            double t_start = t0 + i * dt;
            double t_end = t_start + dt;
            auto deriv_start = curve.evaluate(t_start, 1);
            auto deriv_end = curve.evaluate(t_end, 1);
            if (deriv_start.size() < 2 || deriv_end.size() < 2) continue;
            double d_start = deriv_start[1][axis];
            double d_end = deriv_end[1][axis];
            if (d_start * d_end < 0) {
                double t_lo = t_start, t_hi = t_end;
                double t_root = (t_lo + t_hi) * 0.5;
                for (int iter = 0; iter < 20; iter++) {
                    auto deriv = curve.evaluate(t_root, 2);
                    if (deriv.size() < 3) break;
                    double f = deriv[1][axis];
                    double fp = deriv[2][axis];
                    if (std::abs(f) < 1e-12) break;
                    if (std::abs(fp) > 1e-14) {
                        double t_new = t_root - f / fp;
                        if (t_new >= t_lo && t_new <= t_hi) {
                            t_root = t_new;
                        } else {
                            if (f * d_start < 0) t_hi = t_root;
                            else t_lo = t_root;
                            t_root = (t_lo + t_hi) * 0.5;
                        }
                    } else {
                        t_root = (t_lo + t_hi) * 0.5;
                    }
                    auto deriv_check = curve.evaluate(t_root, 1);
                    if (deriv_check.size() >= 2) {
                        double f_check = deriv_check[1][axis];
                        if (f_check * d_start < 0) { t_hi = t_root; d_end = f_check; }
                        else { t_lo = t_root; d_start = f_check; }
                    }
                }
                extrema_points.push_back(curve.point_at(t_root));
            }
        }
    }
    return from_points(extrema_points, inflate);
}

// ── AABB instance methods ─────────────────────────────────────────────────────

Point AABB::min_point() const {
    return Point(cx - hx, cy - hy, cz - hz);
}

Point AABB::max_point() const {
    return Point(cx + hx, cy + hy, cz + hz);
}

std::array<Point, 8> AABB::corners() const {
    return {
        Point(cx + hx, cy + hy, cz - hz),
        Point(cx - hx, cy + hy, cz - hz),
        Point(cx - hx, cy - hy, cz - hz),
        Point(cx + hx, cy - hy, cz - hz),
        Point(cx + hx, cy + hy, cz + hz),
        Point(cx - hx, cy + hy, cz + hz),
        Point(cx - hx, cy - hy, cz + hz),
        Point(cx + hx, cy - hy, cz + hz),
    };
}

void AABB::inflate(double amount) {
    hx += amount;
    hy += amount;
    hz += amount;
}

bool AABB::intersects(const AABB& other) const {
    return cx - hx <= other.cx + other.hx &&
           cx + hx >= other.cx - other.hx &&
           cy - hy <= other.cy + other.hy &&
           cy + hy >= other.cy - other.hy &&
           cz - hz <= other.cz + other.hz &&
           cz + hz >= other.cz - other.hz;
}

AABB AABB::merge(const AABB& a, const AABB& b) {
    double min_x = std::min(a.cx - a.hx, b.cx - b.hx);
    double min_y = std::min(a.cy - a.hy, b.cy - b.hy);
    double min_z = std::min(a.cz - a.hz, b.cz - b.hz);
    double max_x = std::max(a.cx + a.hx, b.cx + b.hx);
    double max_y = std::max(a.cy + a.hy, b.cy + b.hy);
    double max_z = std::max(a.cz + a.hz, b.cz + b.hz);
    return AABB(
        (min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5,
        (max_x - min_x) * 0.5, (max_y - min_y) * 0.5, (max_z - min_z) * 0.5
    );
}

Point AABB::center() const {
    return Point(cx, cy, cz);
}

double AABB::area() const {
    return 8.0 * (hx * hy + hy * hz + hz * hx);
}

double AABB::diagonal() const {
    return 2.0 * std::sqrt(hx * hx + hy * hy + hz * hz);
}

bool AABB::is_valid() const {
    return hx >= 0.0 && hy >= 0.0 && hz >= 0.0;
}

double AABB::volume() const {
    return 8.0 * hx * hy * hz;
}

Point AABB::closest_point(const Point& pt) const {
    double x = std::max(cx - hx, std::min(cx + hx, pt[0]));
    double y = std::max(cy - hy, std::min(cy + hy, pt[1]));
    double z = std::max(cz - hz, std::min(cz + hz, pt[2]));
    return Point(x, y, z);
}

bool AABB::contains(const Point& pt) const {
    return pt[0] >= cx - hx && pt[0] <= cx + hx &&
           pt[1] >= cy - hy && pt[1] <= cy + hy &&
           pt[2] >= cz - hz && pt[2] <= cz + hz;
}

Point AABB::corner(bool x_max, bool y_max, bool z_max) const {
    return Point(
        cx + (x_max ? hx : -hx),
        cy + (y_max ? hy : -hy),
        cz + (z_max ? hz : -hz)
    );
}

std::array<Point, 8> AABB::get_corners() const {
    return corners();
}

std::vector<Line> AABB::get_edges() const {
    auto c = corners();
    return {
        Line(c[0][0], c[0][1], c[0][2], c[1][0], c[1][1], c[1][2]),
        Line(c[1][0], c[1][1], c[1][2], c[2][0], c[2][1], c[2][2]),
        Line(c[2][0], c[2][1], c[2][2], c[3][0], c[3][1], c[3][2]),
        Line(c[3][0], c[3][1], c[3][2], c[0][0], c[0][1], c[0][2]),
        Line(c[4][0], c[4][1], c[4][2], c[5][0], c[5][1], c[5][2]),
        Line(c[5][0], c[5][1], c[5][2], c[6][0], c[6][1], c[6][2]),
        Line(c[6][0], c[6][1], c[6][2], c[7][0], c[7][1], c[7][2]),
        Line(c[7][0], c[7][1], c[7][2], c[4][0], c[4][1], c[4][2]),
        Line(c[0][0], c[0][1], c[0][2], c[4][0], c[4][1], c[4][2]),
        Line(c[1][0], c[1][1], c[1][2], c[5][0], c[5][1], c[5][2]),
        Line(c[2][0], c[2][1], c[2][2], c[6][0], c[6][1], c[6][2]),
        Line(c[3][0], c[3][1], c[3][2], c[7][0], c[7][1], c[7][2]),
    };
}

Point AABB::point_at(double x, double y, double z) const {
    return Point(cx + x, cy + y, cz + z);
}

void AABB::union_with(const AABB& other) {
    double min_x = std::min(cx - hx, other.cx - other.hx);
    double min_y = std::min(cy - hy, other.cy - other.hy);
    double min_z = std::min(cz - hz, other.cz - other.hz);
    double max_x = std::max(cx + hx, other.cx + other.hx);
    double max_y = std::max(cy + hy, other.cy + other.hy);
    double max_z = std::max(cz + hz, other.cz + other.hz);
    cx = (min_x + max_x) * 0.5; hx = (max_x - min_x) * 0.5;
    cy = (min_y + max_y) * 0.5; hy = (max_y - min_y) * 0.5;
    cz = (min_z + max_z) * 0.5; hz = (max_z - min_z) * 0.5;
}

// ── AABBTree ──────────────────────────────────────────────────────────────────

void AABBTree::build(const AABB* aabbs, size_t count) {
    nodes.clear();
    if (count == 0) return;
    nodes.reserve(2 * count - 1);
    std::vector<int> ids(count);
    for (size_t i = 0; i < count; i++) ids[i] = static_cast<int>(i);
    build_node(ids.data(), static_cast<int>(count), aabbs);
}

void AABBTree::build_node(int* ids, int count, const AABB* aabbs) {
    int idx = static_cast<int>(nodes.size());
    nodes.push_back({{}, -1, -1});

    double lo_x = 1e308, lo_y = 1e308, lo_z = 1e308;
    double hi_x = -1e308, hi_y = -1e308, hi_z = -1e308;
    for (int i = 0; i < count; i++) {
        const auto& b = aabbs[ids[i]];
        lo_x = std::min(lo_x, b.cx - b.hx); hi_x = std::max(hi_x, b.cx + b.hx);
        lo_y = std::min(lo_y, b.cy - b.hy); hi_y = std::max(hi_y, b.cy + b.hy);
        lo_z = std::min(lo_z, b.cz - b.hz); hi_z = std::max(hi_z, b.cz + b.hz);
    }
    nodes[idx].aabb = {
        (lo_x + hi_x) * 0.5, (lo_y + hi_y) * 0.5, (lo_z + hi_z) * 0.5,
        (hi_x - lo_x) * 0.5, (hi_y - lo_y) * 0.5, (hi_z - lo_z) * 0.5
    };

    if (count == 1) {
        nodes[idx].object_id = ids[0];
        return;
    }

    double dx = hi_x - lo_x, dy = hi_y - lo_y, dz = hi_z - lo_z;
    int axis = (dx >= dy && dx >= dz) ? 0 : (dy >= dz) ? 1 : 2;
    int mid = count / 2;

    std::nth_element(ids, ids + mid, ids + count, [&](int a, int b) {
        const auto& ba = aabbs[a]; const auto& bb = aabbs[b];
        if (axis == 0) return ba.cx < bb.cx;
        if (axis == 1) return ba.cy < bb.cy;
        return ba.cz < bb.cz;
    });

    build_node(ids, mid, aabbs);
    nodes[idx].right = static_cast<int>(nodes.size());
    build_node(ids + mid, count - mid, aabbs);
}

std::vector<int> AABBTree::query_aabb(const AABB& query) const {
    std::vector<int> hits;
    if (empty()) return hits;
    std::vector<int> stack = {0};
    while (!stack.empty()) {
        int idx = stack.back(); stack.pop_back();
        const auto& node = nodes[idx];
        if (!node.aabb.intersects(query)) continue;
        if (node.object_id >= 0) {
            hits.push_back(node.object_id);
        } else {
            stack.push_back(idx + 1);
            stack.push_back(node.right);
        }
    }
    return hits;
}

}
