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

    /// Raw-array version: takes flat 2D coords (x,y pairs, stride=2), returns flat 2D result coords.
    /// No Polyline construction. Returns number of result points (0 if no intersection).
    static int compute_raw(const double* a_xy, int na, const double* b_xy, int nb,
                           int clip_type, double* out_xy, int max_out);

};

} // namespace session_cpp
