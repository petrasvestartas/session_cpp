#pragma once

#include "point.h"
#include "vector.h"
#include "line.h"
#include "polyline.h"
#include "aabb.h"
#include <tuple>
#include <vector>

namespace session_cpp {

class NurbsCurve;
class NurbsSurface;
class Mesh;
class PointCloud;

/// Closest-point queries between points, curves, surfaces, meshes and clouds.
class Closest {
public:
    // ═══════════════════════════════════════════════════════════════════════════
    // Curves
    // ═══════════════════════════════════════════════════════════════════════════
    /// Parameter and distance of the closest curve point within [t0, t1] (0 means the domain end).
    static std::pair<double, double> curve_point(
        const NurbsCurve& curve,
        const Point& test_point,
        double t0 = 0.0,
        double t1 = 0.0
    );

    /// Return the parameters and distance of the closest approach between two curves.
    static std::tuple<double, double, double> curve_curve(const NurbsCurve& curve0, const NurbsCurve& curve1);

    /// Return the closest point, parameter in [0, 1] and distance on a segment.
    static std::tuple<Point, double, double> line_point(const Line& line, const Point& test_point);

    /// Return the closest point, length parameter in [0, 1] and distance on a polyline.
    static std::tuple<Point, double, double> polyline_point(const Polyline& polyline, const Point& test_point);

    // ═══════════════════════════════════════════════════════════════════════════
    // Surfaces
    // ═══════════════════════════════════════════════════════════════════════════
    /// Parameters and distance of the closest surface point within a uv window (0 means the domain end).
    static std::tuple<double, double, double> surface_point(
        const NurbsSurface& surface,
        const Point& test_point,
        double u0 = 0.0,
        double u1 = 0.0,
        double v0 = 0.0,
        double v1 = 0.0
    );

    /// Seam-split uv pcurves of a curve lying on the surface, empty when it does not.
    static std::vector<NurbsCurve> surface_curve(
        const NurbsSurface& surface,
        const NurbsCurve& curve,
        double t0 = 0.0,
        double t1 = 0.0,
        double tolerance = 0.0
    );

    // ═══════════════════════════════════════════════════════════════════════════
    // Meshes and clouds
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the closest point, face key and distance on a mesh via its triangle BVH.
    static std::tuple<Point, size_t, double> mesh_point(const Mesh& mesh, const Point& test_point);

    /// Return the closest point, face key and distance on a mesh via its triangle AABB tree.
    static std::tuple<Point, size_t, double> mesh_point_aabb(const Mesh& mesh, const Point& test_point);

    /// Return the closest point, index and distance in a cloud by linear scan.
    static std::tuple<Point, size_t, double> pointcloud_point(const PointCloud& cloud, const Point& test_point);

    /// Return the closest point, index and distance in a cloud via a kd-tree.
    static std::tuple<Point, size_t, double> pointcloud_point_kdtree(const PointCloud& cloud, const Point& test_point);

    // ═══════════════════════════════════════════════════════════════════════════
    // Collections
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the index pairs of lines whose endpoints come within threshold of each other.
    static std::vector<std::pair<size_t, size_t>> lines_closest(const std::vector<Line>& lines, double threshold = 0.0);

    /// Index pairs of polylines whose vertices come within threshold of each other.
    static std::vector<std::pair<size_t, size_t>> polylines_closest(
        const std::vector<Polyline>& polylines,
        double threshold = 0.0
    );

    /// Index pairs of curves whose endpoints come within threshold of each other.
    static std::vector<std::pair<size_t, size_t>> nurbscurves_closest(
        const std::vector<NurbsCurve>& curves,
        double threshold = 0.0
    );

    /// Return the index pairs of boxes within threshold of each other.
    static std::vector<std::pair<size_t, size_t>> boxes_closest(const std::vector<AABB>& boxes, double threshold = 0.0);
};

} // namespace session_cpp
