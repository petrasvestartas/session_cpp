#pragma once
#include "polyline.h"
#include <vector>

namespace session_cpp {

struct Triangle2D {
    int v0, v1, v2;
};

class Triangulation2D {
public:
    static std::vector<Triangle2D> triangulate(
        const Polyline& boundary,
        const std::vector<Polyline>& holes = {}
    );

private:
    static double cross_2d(double ax, double ay, double bx, double by, double cx, double cy);
    static double signed_area_2d(const std::vector<double>& coords);
    static bool point_in_triangle_2d(double px, double py,
                                     double ax, double ay,
                                     double bx, double by,
                                     double cx, double cy);
    static bool is_convex(const std::vector<double>& coords, const std::vector<int>& indices, int prev, int curr, int next);
    static bool is_ear(const std::vector<double>& coords, const std::vector<int>& indices,
                       const std::vector<bool>& reflex, int prev, int curr, int next);
    static std::vector<Triangle2D> ear_clip(const std::vector<double>& coords, std::vector<int> indices);
    static bool segments_intersect_proper(double ax, double ay, double bx, double by,
                                        double cx, double cy, double dx, double dy);
    static bool is_visible(const std::vector<double>& coords, const std::vector<int>& polygon,
                           int from_idx, int to_idx);
    static std::vector<int> merge_holes(std::vector<double>& coords, std::vector<int>& boundary_indices,
                                        const std::vector<std::vector<int>>& hole_indices_list);
};

} // namespace session_cpp
