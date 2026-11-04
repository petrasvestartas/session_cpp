#pragma once

#include "brep.h"
#include <map>

namespace session_cpp::brep_offset {

struct OffsetFace {
    BRep brep;
    int source;
    std::map<int, int> edges;
};

/// Preserve original face/edge ancestry while constructing one displaced support.
OffsetFace displaced_face(const BRep& source, int face, double distance, double tolerance);

/// Working input must be a certified extension in the same original UV chart.
OffsetFace displaced_working_face(const BRep& original, int face, const OffsetFace& working,
    double distance, double tolerance);

/// Construct the inward tube support between the displaced images of one concave edge.
OffsetFace inward_tube(const BRep& source, const std::vector<OffsetFace>& faces,
    int edge, int first, int second, double tolerance);

/// Join displaced faces with shared smooth boundaries, curved edge tubes and spherical vertex patches.
BRep join(
    const BRep& source,
    std::vector<OffsetFace> faces,
    double distance,
    double tolerance
);

}
