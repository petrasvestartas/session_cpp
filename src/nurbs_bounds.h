#pragma once

#include "nurbssurface.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace session_cpp::nurbs_bounds {

/// Conservative Euclidean displacement per unit UV distance for positive rational surfaces.
inline double surface_metric(const NurbsSurface& surface) {
    Point lo = surface.get_cv(0, 0), hi = lo;
    double minimum_weight = std::numeric_limits<double>::infinity();
    double maximum_weight = 0.0;
    for (int i = 0; i < surface.cv_count(0); ++i)
        for (int j = 0; j < surface.cv_count(1); ++j) {
            const Point p = surface.get_cv(i, j);
            const double weight = surface.weight(i, j);
            if (!std::isfinite(weight) || weight <= 0.0)
                throw std::invalid_argument("Surface metric requires finite positive rational weights");
            minimum_weight = std::min(minimum_weight, weight);
            maximum_weight = std::max(maximum_weight, weight);
            for (int d = 0; d < 3; ++d) {
                if (!std::isfinite(p[d]))
                    throw std::invalid_argument("Surface metric requires finite control points");
                lo[d] = std::min(lo[d], p[d]);
                hi[d] = std::max(hi[d], p[d]);
            }
        }
    double bounds[2] = {};
    for (int direction = 0; direction < 2; ++direction) {
        const auto knots = surface.get_span_vector(direction);
        for (size_t i = 1; i < knots.size(); ++i)
            if (knots[i] > knots[i - 1])
                bounds[direction] = std::max(bounds[direction], 2.0 * surface.degree(direction) *
                    (maximum_weight / minimum_weight) * lo.distance(hi) / (knots[i] - knots[i - 1]));
    }
    const double metric = std::hypot(bounds[0], bounds[1]);
    if (!std::isfinite(metric) || metric <= 0.0)
        throw std::runtime_error("Surface metric exceeds numerical resolution or is degenerate");
    return metric;
}

}
