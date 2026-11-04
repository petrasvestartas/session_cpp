#pragma once
#include "brep.h"
namespace session_cpp::brep_hollow {
BRep shape(const BRep& source, const std::vector<int>& openings, double thickness, double tolerance);
}
