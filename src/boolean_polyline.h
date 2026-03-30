#pragma once
#include "polyline.h"
#include <vector>

namespace session_cpp {

class BooleanPolyline {
public:
    /// Boolean operation on two closed planar polylines.
    /// clip_type: 0=intersection, 1=union, 2=difference (a minus b).
    static std::vector<Polyline> compute(const Polyline& a, const Polyline& b, int clip_type);

    /// Engine-only: runs Vatti but returns total output point count (no Polyline construction).
    static int compute_count(const Polyline& a, const Polyline& b, int clip_type);
};

} // namespace session_cpp
