#pragma once

#include "brep_offset_join.h"
#include "split.h"
#include <stdexcept>
#include <string>

namespace session_cpp::brep_offset {

/// Preserve exact inputs to an unresolved inward section or triple-vertex stage.
struct InwardError : std::runtime_error {
    std::string stage;
    int source_edge = -1;
    int source_vertex = -1;
    int source_face = -1;
    std::vector<int> source_faces;
    std::vector<int> section_edges;
    std::vector<NurbsSurface> surfaces;
    std::vector<NurbsCurve> curves;
    std::vector<NurbsCurve> first_pcurves;
    std::vector<NurbsCurve> second_pcurves;
    BRep partition;
    std::vector<int> candidate_faces;
    std::vector<split::EdgePiece> pieces;

    explicit InwardError(const std::string& message)
        : std::runtime_error(message) {
    }
};

/// Trim consistently convex sharp joins of a simple valence-three closed shell.
/// Displaced supports must already correspond to distance. Mixed joins and global
/// self-intersection cleanup are outside this restricted construction.
BRep inward(
    const BRep& source,
    std::vector<OffsetFace> faces,
    double distance,
    double tolerance
);

}
