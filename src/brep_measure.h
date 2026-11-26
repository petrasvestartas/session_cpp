#pragma once

#include "brep.h"
#include <array>
#include <vector>

namespace session_cpp::brep_measure {

/// Integrate signed trimmed NURBS volume without tessellation; tolerance is relative.
double volume(const BRep& brep, double tolerance = 1e-7);

/// Per face, to find where a solid's measure goes wrong: the area of its tessellation and its signed share of volume(), which add up to the volume.
std::vector<std::array<double, 2>> faces(const BRep& brep, double tolerance = 1e-7);

}
