#pragma once

#include "curve_intersection.h"
#include <cstddef>

namespace session_cpp::curve_self_intersection {

struct Options {
    std::size_t max_cells = 200000;
    int max_depth = 96;
};

struct Result {
    std::vector<curve_intersection::Hit> points;
    std::vector<curve_intersection::Hit> endpoints;
    std::vector<curve_intersection::Overlap> overlaps;
    bool complete = true;
    std::string failure;
    std::size_t cells = 0;
};

/// Find parameter-distinct self contacts, separating closure endpoints and reporting unresolved regions explicitly.
Result intersect(const NurbsCurve& curve, double tolerance = 1e-7, const Options& options = {});

}
