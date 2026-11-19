#pragma once

#include "brep.h"
#include <string>
#include <vector>

namespace session_cpp::bop {

enum class Operation { Intersection, Union, Difference, ReverseDifference, Xor };

/// What the boolean noticed but did not refuse for: interferences it could not compute, faces it could not split or classify, whether the result passed BRep::is_solid, and in failure why the result is not to be trusted (empty when nothing says so)
struct Report {
    std::vector<std::string> log;
    bool solid = false;
    std::string failure;
};

/// BOPAlgo_BOP on two solids: interferences within the operands' own tolerances plus fuzzy, tolerances grown where the data needs it; a check that fails is recorded in the report, never a refusal
BRep apply(const BRep& a, const BRep& b, Operation operation, double fuzzy = 0.0, Report* report = nullptr);

}
