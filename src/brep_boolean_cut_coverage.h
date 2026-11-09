#pragma once
#include "curve_intersection.h"
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace session_cpp::brep_boolean::detail {
// Only established positive common ranges cover a cut. Raw-parameter gaps
// cannot be replaced by a machine epsilon: their physical image may be large.
inline bool whole_raw_cut_coverage(const std::pair<double,double>& domain,
                                  const curve_intersection::Result& events) {
    if (!events.complete || !std::isfinite(domain.first) || !std::isfinite(domain.second) ||
        !(domain.first<domain.second)) return false;
    std::vector<std::pair<double,double>> intervals;
    for (const auto& overlap:events.overlaps) {
        if (!std::isfinite(overlap.a0) || !std::isfinite(overlap.a1) || !(overlap.a0<overlap.a1) ||
            !std::isfinite(overlap.b0) || !std::isfinite(overlap.b1) || overlap.b0==overlap.b1)
            return false;
        intervals.push_back({overlap.a0,overlap.a1});
    }
    std::sort(intervals.begin(),intervals.end());
    double last=domain.first;
    for (const auto& interval:intervals) {
        if (interval.first>last) return false;
        last=std::max(last,interval.second);
    }
    return last>=domain.second;
}
} // namespace session_cpp::brep_boolean::detail
