#pragma once
#include "point.h"
#include "polyline.h"
#include <functional>
#include <vector>

namespace session_cpp {

/**
 * @class MarchingSquares
 * @brief Marching squares iso-contour extraction from 2D scalar fields.
 */
class MarchingSquares {
public:
    static std::vector<Polyline> extract(const std::vector<std::vector<double>>& grid, double iso_value, double cell_size = 1.0);
    static std::vector<Polyline> extract_from_func(
        const std::function<double(double, double)>& func,
        std::pair<double, double> x_range,
        std::pair<double, double> y_range,
        int nx, int ny,
        double iso_value
    );

private:
    static const int EDGE_TABLE[16][4]; // [case][segment_idx] = ea*10+eb, -1=unused
    static double interp(double a, double b, double va, double vb, double iso);
    static Point edge_pt(int e, double x0, double y0, double x1, double y1, double v0, double v1, double v2, double v3, double iso);
    static std::vector<Polyline> connect_segments(std::vector<std::pair<Point, Point>>& segs);
};

} // namespace session_cpp
