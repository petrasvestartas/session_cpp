#pragma once
#include "polyline.h"
#include <vector>

namespace session_cpp {

/// Vatti boolean operations on closed planar polylines.
class BooleanPolyline {
public:
    /// Compute the Vatti boolean of two closed planar polylines; clip_type 0 intersection, 1 union, 2 a minus b.
    static std::vector<Polyline> compute(const Polyline& a, const Polyline& b, int clip_type);

    /// Return the number of output points of compute, without building polylines.
    static int compute_count(const Polyline& a, const Polyline& b, int clip_type);

    /// Compute on flat xy arrays; write up to max_out result points to out_xy and return the total.
    static int compute_raw(const double* a_xy, int na, const double* b_xy, int nb, int clip_type, double* out_xy, int max_out);

    /// Return the pieces of an open polyline that lie inside a closed clip polygon, in the xy plane.
    static std::vector<Polyline> clip_open_against_closed(const Polyline& open_subject, const Polyline& closed_clip);
};

} // namespace session_cpp
