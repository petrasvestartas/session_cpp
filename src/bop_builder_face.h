#pragma once

#include "bop_ds.h"
#include <string>
#include <vector>

namespace session_cpp::bop {

/// One traversal of an edge image across a face being split
struct EdgeUse {
    int image = -1;
    int start = -1, end = -1; // real vertices in traversal order
    NurbsCurve pcurve; // along the traversal
    bool reversed = false; // traversal against the image's own direction
    bool internal = false; // In or Sc edge, offered once each way
    double gap = 0.0; // how far in uv its ends may sit from its vertices: their tolerance
};

/// BOPAlgo_BuilderFace with BOPAlgo_WireSplitter: the faces bounded by the uses, each a list of loops (outer first) of use indices.
/// Internal pcurves are moved by a period of the region onto the face.
std::vector<std::vector<std::vector<int>>> build_faces(std::vector<EdgeUse>& uses, const Region& region, std::vector<std::string>& log);

/// The sampled uv polygon of a loop
std::vector<Point> loop_polygon(const std::vector<EdgeUse>& uses, const std::vector<int>& loop);

}
