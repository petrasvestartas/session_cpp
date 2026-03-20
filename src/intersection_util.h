#pragma once
#include "intersection.h"
#include "line.h"
#include "plane.h"
#include "point.h"
#include "vector.h"
#include <vector>
#include <utility>

// ── session_cpp intersection utilities ───────────────────────────────────────
// Free-function wrappers for session_cpp::Intersection class methods.
// Replaces cgal_intersection_util.h/.cpp, using session_cpp types throughout.
// NOTE: polyline_plane_cross_joint is NOT here — it depends on collider::clipper_util
//       (a wood project dependency) and lives in wood_joint_util.h instead.

namespace session_cpp {

// line_line_3d
inline bool line_line_3d(const Line& cutter, const Line& seg, Point& output)
{
    return Intersection::line_line_3d(cutter, seg, output);
}

// plane_plane_plane
inline bool plane_plane_plane(const Plane& p0, const Plane& p1, const Plane& p2, Point& output)
{
    return Intersection::plane_plane_plane(p0, p1, p2, output);
}

// plane_plane
inline bool plane_plane(const Plane& p0, const Plane& p1, Line& output)
{
    return Intersection::plane_plane(p0, p1, output);
}

// line_plane — default is_finite=false matches old CGAL behavior (infinite line)
inline bool line_plane(const Line& line, const Plane& plane, Point& output, bool is_finite = false)
{
    return Intersection::line_plane(line, plane, output, is_finite);
}

// get_quad_from_line_topbottomplanes
inline bool get_quad_from_line_topbottomplanes(
    const Plane& face, const Line& line,
    const Plane& p0, const Plane& p1,
    std::vector<Point>& output)
{
    return Intersection::get_quad_from_line_topbottomplanes(face, line, p0, p1, output);
}

// scale_vector_to_distance_of_2planes
inline bool scale_vector_to_distance_of_2planes(
    const Vector& dir, const Plane& p0, const Plane& p1, Vector& output)
{
    return Intersection::scale_vector_to_distance_of_2planes(dir, p0, p1, output);
}

// get_orthogonal_vector_between_two_plane_pairs
inline bool get_orthogonal_vector_between_two_plane_pairs(
    const Plane& pp0_0, const Plane& pp1_0, const Plane& pp1_1, Vector& output)
{
    return Intersection::get_orthogonal_vector_between_two_plane_pairs(pp0_0, pp1_0, pp1_1, output);
}

// plane_4planes
inline bool plane_4planes(const Plane& main, const Plane (&seq)[4], std::vector<Point>& output)
{
    return Intersection::plane_4planes(main, seq, output);
}

// plane_4planes_open
inline bool plane_4planes_open(const Plane& main, const Plane (&seq)[4], std::vector<Point>& output)
{
    return Intersection::plane_4planes_open(main, seq, output);
}

// plane_4lines
inline bool plane_4lines(Plane& plane, Line& l0, Line& l1, Line& l2, Line& l3, std::vector<Point>& output)
{
    return Intersection::plane_4lines(plane, l0, l1, l2, l3, output);
}

// plane_plane_plane_with_parallel_check (calls Intersection::plane_plane_plane_check)
inline bool plane_plane_plane_with_parallel_check(
    const Plane& p0, const Plane& p1, const Plane& p2, Point& output)
{
    return Intersection::plane_plane_plane_check(p0, p1, p2, output);
}

// line_two_planes
inline bool line_two_planes(Line& line, const Plane& p0, const Plane& p1)
{
    return Intersection::line_two_planes(line, p0, p1);
}

// polyline_plane_to_line
inline bool polyline_plane_to_line(
    const std::vector<Point>& poly, const Plane& plane, Line& align, Line& output)
{
    return Intersection::polyline_plane_to_line(poly, plane, align, output);
}

// polyline_plane
inline bool polyline_plane(
    const std::vector<Point>& poly, const Plane& plane,
    std::vector<Point>& pts, std::vector<int>& edge_ids)
{
    return Intersection::polyline_plane(poly, plane, pts, edge_ids);
}

} // namespace session_cpp
