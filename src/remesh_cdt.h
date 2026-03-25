#pragma once
#include <vector>
#include <array>
#include <utility>

namespace session_cpp {

// Constrained Delaunay Triangulation — sweep-line + Delaunay legalization.
// border_2d: CCW outer boundary.  holes_2d: CW inner boundaries (zero or more).
// Returns face index triples into flat array [border..., hole0..., hole1...].
std::vector<std::array<int,3>> cdt_triangulate(
    const std::vector<std::pair<double,double>>& border_2d,
    const std::vector<std::vector<std::pair<double,double>>>& holes_2d);

bool point_in_polygon_2d(double px, double py, const std::vector<double>& coords);

} // namespace session_cpp
