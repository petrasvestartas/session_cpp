#pragma once

#include "brep.h"
#include <array>
#include <vector>

namespace session_cpp::brep_measure {

/// Integrate signed trimmed NURBS volume without tessellation; tolerance is relative.
double volume(const BRep& brep, double tolerance = 1e-7);

struct Volume {
    double value;
    double closure_error;
};

/// volume() and how far it can move where the faces do not close: (|A| + a) R / 3 for their net vector area A, the area a of the
/// ribbons between the lifts of each edge's pcurves, and R the largest distance of a control point from their mean.
Volume volume_closure(const BRep& brep, double tolerance = 1e-7);

/// Per face, to find where a solid's measure goes wrong: the area of its tessellation and its signed share of volume(), which add up to the volume.
std::vector<std::array<double, 2>> faces(const BRep& brep, double tolerance = 1e-7);

}
