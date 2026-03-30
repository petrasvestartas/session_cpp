// AABBTree — flat contiguous BVH over axis-aligned boxes (SAH median split).
// Use for: closest-point on static mesh faces, ray-mesh intersection.
//   Build once, query many times. Cache-friendly 56-byte nodes.
// Prefer over BVH  when geometry is static and all volumes are world-aligned.
// Prefer over RTree when no dynamic insert/delete is needed.
// Prefer over KDTree when querying faces/volumes, not bare point clouds.
#pragma once

#include "point.h"
#include "vector.h"
#include "line.h"
#include <array>
#include <vector>

namespace session_cpp {

class Polyline;
class Mesh;
class PointCloud;
class NurbsCurve;
class NurbsSurface;

struct AABB {
    double cx, cy, cz; // center
    double hx, hy, hz; // half-size

    AABB() : cx(0), cy(0), cz(0), hx(0), hy(0), hz(0) {}
    AABB(double cx, double cy, double cz, double hx, double hy, double hz)
        : cx(cx), cy(cy), cz(cz), hx(hx), hy(hy), hz(hz) {}

    static AABB from_point(const Point& point, double inflate = 0.0);
    static AABB from_points(const std::vector<Point>& points, double inflate = 0.0);
    static AABB from_line(const Line& line, double inflate = 0.0);
    static AABB from_polyline(const Polyline& polyline, double inflate = 0.0);
    static AABB from_mesh(const Mesh& mesh, double inflate = 0.0);
    static AABB from_pointcloud(const PointCloud& pointcloud, double inflate = 0.0);
    static AABB from_nurbscurve(const NurbsCurve& curve, double inflate = 0.0, bool tight = false);
    static AABB from_nurbssurface(const NurbsSurface& surface, double inflate = 0.0);

    Point min_point() const;
    Point max_point() const;
    Point center() const;
    double area() const;
    double diagonal() const;
    bool is_valid() const;
    double volume() const;
    Point closest_point(const Point& pt) const;
    bool contains(const Point& pt) const;
    Point corner(bool x_max, bool y_max, bool z_max) const;
    std::array<Point, 8> corners() const;
    std::array<Point, 8> get_corners() const;
    std::vector<Line> get_edges() const;
    Point point_at(double x, double y, double z) const;
    void inflate(double amount);
    bool intersects(const AABB& other) const;
    void union_with(const AABB& other);
    static AABB merge(const AABB& a, const AABB& b);
};

class AABBTree {
public:
    struct Node {
        AABB aabb;
        int right;      // right child index; left child = this_index + 1
        int object_id;  // leaf: primitive id (>=0), internal: -1
    };

    std::vector<Node> nodes;

    AABBTree() = default;
    void build(const AABB* aabbs, size_t count);
    bool empty() const { return nodes.empty(); }
    size_t size() const { return nodes.size(); }

    std::vector<int> query_aabb(const AABB& query) const;

private:
    void build_node(int* ids, int count, const AABB* aabbs);
};

}
