#pragma once

#include "brep.h"

namespace session_cpp::brep_offset {

/// Offset trimmed faces and regular solids, joining smooth boundaries and outward convex sharp edges.
BRep shape(const BRep& source, double distance, double tolerance = 1e-7);

}
