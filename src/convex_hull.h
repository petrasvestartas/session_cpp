#pragma once
#include "mesh.h"
#include "point.h"
#include <tuple>
#include <vector>

namespace session_cpp {

/**
 * @class ConvexHull
 * @brief Convex hull: Graham scan (2D) and Quickhull (3D).
 */
class ConvexHull {
public:
    static std::vector<Point> hull_2d(const std::vector<Point>& points);
    static Mesh hull_3d(const std::vector<Point>& points);

private:
    static double cross_2d(const Point& o, const Point& a, const Point& b);
    static std::tuple<double, double, double> normal(const Point& a, const Point& b, const Point& c);
    static double signed_volume(const Point& a, const Point& b, const Point& c, const Point& d);
    static int farthest_point(const std::vector<int>& pts_idx, const std::vector<Point>& points, const Point& a, const Point& b, const Point& c);
    static std::vector<int> visible_from(const std::vector<int>& pts_idx, const std::vector<Point>& points, const Point& a, const Point& b, const Point& c);
    static void quickhull_3d_faces(const std::vector<Point>& points, const std::vector<int>& pts_idx, int a, int b, int c, std::vector<std::tuple<int, int, int>>& faces);
};

} // namespace session_cpp
