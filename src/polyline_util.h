#pragma once
#include "line.h"
#include "point.h"
#include "polyline.h"
#include <vector>

// ── session_cpp polyline utilities ───────────────────────────────────────────
// Convenience wrappers that match the old cgal_polyline_util / cgal_rectangle_util
// naming so existing wood call sites compile without modification.

namespace session_cpp {

    /// Point on a line at parameter t.
    inline Point point_at(const Line& line, double t) { return line.point_at(t); }

    /// Closest parameter on a line to a given point (fills t).
    inline void closest_point_to(const Point& pt, const Line& line, double& t) {
        Polyline::closest_point_to_line(pt, line.start(), line.end(), t);
    }

    /// Grid of points inside a polygon; returns true if any points were generated.
    inline bool grid_of_points_in_a_polygon(
        std::vector<Point>& polygon,
        double offset, double div_dist, int max_pts,
        std::vector<Point>& points)
    {
        points = Polyline::grid_of_points_in_polygon(Polyline(polygon), offset, div_dist, max_pts);
        return !points.empty();
    }

} // namespace session_cpp
