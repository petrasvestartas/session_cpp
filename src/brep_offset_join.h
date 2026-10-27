#pragma once

#include "brep.h"
#include <map>

namespace session_cpp::brep_offset {

struct OffsetFace {
    BRep brep;
    int source;
    std::map<int, int> edges;
};

/// Join displaced faces with shared smooth boundaries, curved edge tubes and spherical vertex patches.
BRep join(
    const BRep& source,
    std::vector<OffsetFace> faces,
    double distance,
    double tolerance
);

}
