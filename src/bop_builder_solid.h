#pragma once

#include "brep.h"
#include <vector>

namespace session_cpp::bop {

/// BOPAlgo_BuilderSolid with BOPAlgo_ShellSplitter: oriented faces of a result grouped by shared edges into shells, split at an edge of more than two faces by the angle between them, each hole shell put into the solid around it
void build_solids(BRep& result, const std::vector<BRepRef>& faces);

/// A copy holding only the given oriented faces as one closed shell of one solid
BRep subset(const BRep& brep, const std::vector<BRepRef>& faces);

/// The generalized winding number of a closed triangulated boundary around a point
double winding(const Mesh& mesh, const Point& p);

}
