#pragma once

#include "brep.h"

namespace session_cpp::brep_thicken {

/// Thicken one regular trimmed open face using an offset support and exact ruled boundary walls.
BRep face(const BRep& source, double thickness, double tolerance = 1e-7);

}
