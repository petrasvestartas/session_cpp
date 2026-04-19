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

    /// Open-subject × closed-clip Intersection (NonZero fill rule).
    /// Returns the portion(s) of `open_subject` that lie inside `closed_clip`,
    /// as one or more open polylines. Input `open_subject` is treated as a
    /// polyline (not closed, not filled); `closed_clip` is a filled polygon.
    /// Robust for near-parallel near-coincident geometries via full Vatti
    /// sweep-line — unlike naive per-segment clipping, this handles edges
    /// that run along clip boundary without spurious vertex collapse.
    static std::vector<Polyline> clip_open_against_closed(
        const Polyline& open_subject,
        const Polyline& closed_clip);

};

} // namespace session_cpp
