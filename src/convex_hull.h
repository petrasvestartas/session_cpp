#pragma once
#include "mesh.h"
#include "point.h"
#include <vector>

namespace session_cpp {

/// Convex hull: monotone chain in XY for 2D, quickhull for 3D.
struct ConvexHull {
    /// Counter-clockwise hull of the points projected to XY, collinear points dropped; fewer than three points come back as given.
    static std::vector<Point> hull_2d(const std::vector<Point>& points);

    /// Triangle mesh of the hull with outward faces; fewer than four points give the points and, for three, one face.
    static Mesh hull_3d(const std::vector<Point>& points);
};

} // namespace session_cpp
