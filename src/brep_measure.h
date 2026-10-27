#pragma once

#include "brep.h"

namespace session_cpp::brep_measure {

/// Integrate signed trimmed NURBS volume without tessellation; tolerance is relative.
double volume(const BRep& brep, double tolerance = 1e-7);

}
