#pragma once

#include "brep.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include <vector>

namespace session_cpp {

class Line;
class Polyline;

} // namespace session_cpp

namespace session_cpp::simple_split {

// ═══════════════════════════════════════════════════════════════════════════
// Split
// ═══════════════════════════════════════════════════════════════════════════
/// Split a curve at isolated 3D intersections, retaining every piece and rejecting overlapping cutters.
std::vector<NurbsCurve> split_curve_by_curves(
    const NurbsCurve& curve,
    const std::vector<NurbsCurve>& cutters,
    double tolerance
);

/// Partition one face inside its owning BRep, retaining all regions and shared shell topology.
BRep split_brep_face_by_curves(
    const BRep& brep,
    int face_index,
    const std::vector<NurbsCurve>& cutters,
    double tolerance
);

/// Wrap a surface's natural boundary in a BRep and partition it with on-surface curves.
BRep split_surface_by_curves(
    const NurbsSurface& surface,
    const std::vector<NurbsCurve>& cutters,
    double tolerance
);

/// Split a line at isolated 3D intersections, retaining line types and display attributes.
std::vector<Line> split_line_by_curves(
    const Line& line,
    const std::vector<NurbsCurve>& cutters,
    double tolerance
);

/// Split a polyline, retaining each original corner, piece order and display attributes.
std::vector<Polyline> split_polyline_by_curves(
    const Polyline& polyline,
    const std::vector<NurbsCurve>& cutters,
    double tolerance
);

} // namespace session_cpp::simple_split
