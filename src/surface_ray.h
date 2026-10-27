#pragma once

#include "nurbssurface.h"
#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace session_cpp::surface_ray {

using Rectangle = std::array<double, 4>; // U minimum/maximum, V minimum/maximum.

struct Root {
    Point point;
    double u;
    double v;
    double distance; // Signed world distance along the input direction.
    Rectangle enclosure; // Contains the exact root, within the unique chart.
    Rectangle chart; // Original-domain rectangle containing exactly one root.
    std::array<double, 2> distance_enclosure;
    int crossing; // Certified sign of (Su cross Sv) dot direction, before face orientation.
};

struct Options {
    std::size_t max_cells = 200000;
    int max_depth = 96;
    bool include_origin = false; // Keep certified roots whose signed distance enclosure contains zero.
};

struct Result {
    std::vector<Root> roots;
    bool complete = true;
    std::string failure;
    std::size_t cells = 0;
};

/// Certify regular isolated forward-ray roots on a continuous positive-rational support.
/// Every completed parameter cell is excluded or contained in a proven unique chart.
/// Natural-boundary, tangent, coincident and unresolved roots return complete=false.
/// Origin-straddling roots require include_origin for subsequent trim classification.
Result intersect(const NurbsSurface& surface, const Point& origin, const Vector& direction,
    double tolerance = 1e-7, const Options& options = {});

}
