#pragma once

#include "bop_ds.h"
#include <string>
#include <vector>

namespace session_cpp::bop {

/// One traversal of an edge image across a face being split
struct EdgeUse {
    int image = -1;
    int start = -1, end = -1; // real vertices in traversal order
    Span pcurve; // along the traversal
    bool reversed = false; // traversal against the image's own direction
    bool internal = false; // In or Sc edge, offered once each way
    double gap = 0.0; // how far in uv its ends may sit from its vertices: their tolerance
};

/// BOPAlgo_BuilderFace with BOPAlgo_WireSplitter: the faces bounded by the uses, each a list of loops (outer first) of use indices.
/// Internal pcurves are moved by whole periods of the region onto the face (their spans' shift), no curve is edited.
std::vector<std::vector<std::vector<int>>> build_faces(std::vector<EdgeUse>& uses, const Region& region, std::vector<std::string>& log);

/// The sampled uv polygon of a loop. With the periods of a surface whose parameters run on past them, each use is moved by whole periods (`turns`) to start where the one before it ends, and `end` is where the loop arrives: at its start, or one period on.
std::vector<Point> loop_polygon(const std::vector<EdgeUse>& uses, const std::vector<int>& loop, const std::array<double, 2>& period = {0.0, 0.0}, Point* end = nullptr, std::vector<std::array<double, 2>>* turns = nullptr);

}
