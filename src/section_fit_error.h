#pragma once

#include "point.h"
#include <stdexcept>
#include <vector>

namespace session_cpp::section_fit {

struct Error : std::runtime_error {
    std::vector<Point> samples;
    std::vector<double> parameters;
    std::vector<Point> pcurve_samples;
    std::vector<Point> second_pcurve_samples;
    std::vector<Point> world_samples;
    bool closed;

    Error(const std::vector<Point>& points, const std::vector<double>& values, bool loop)
        : std::runtime_error("Native surface intersection could not interpolate a section"),
          samples(points), parameters(values), closed(loop) {
    }
};

}
