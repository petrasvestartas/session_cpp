#pragma once

#include "point.h"
#include "vector.h"
#include "plane.h"
#include "line.h"
#include "polyline.h"
#include "mesh.h"
#include "obb.h"
#include "spatial_bvh.h"
#include "element.h"
#include "tolerance.h"
#include <array>
#include <tuple>
#include <utility>
#include <vector>
#include <optional>

namespace session_cpp {

class NurbsCurve;
class NurbsSurface;

/// Static intersection routines for lines, planes, rays, NURBS, polylines and elements.
class Intersection {
public:
    // ═══════════════════════════════════════════════════════════════════════════
    // Types
    // ═══════════════════════════════════════════════════════════════════════════
    /// Ray-mesh hit.
    struct RayHit {
        double t;       // Parameter along the ray.
        Point point;    // Hit point.
        double u;       // Barycentric u.
        double v;       // Barycentric v.
        int face_index; // Hit face.

        /// Construct an empty miss.
        RayHit() : t(0.0), point(), u(0.0), v(0.0), face_index(-1) {}

        /// Construct from ray parameter, point, barycentrics and face.
        RayHit(double t_, const Point& p, double u_ = 0.0, double v_ = 0.0, int face_idx = -1)
            : t(t_), point(p), u(u_), v(v_), face_index(face_idx) {}
    };

    // ═══════════════════════════════════════════════════════════════════════════
    // Lines and planes
    // ═══════════════════════════════════════════════════════════════════════════
    /// Intersection point of two segments, the midpoint of closest approach within tolerance.
    static bool line_line(const Line& line0, const Line& line1, Point& output, double tolerance);

    /// Parameters of closest approach of two lines, clamped to the segments when requested.
    static bool line_line_parameters(
        const Line& line0,
        const Line& line1,
        double& t0,
        double& t1,
        double tolerance,
        bool intersect_segments = true,
        bool near_parallel_as_closest = false
    );

    /// Intersection line of two planes, anchored on plane0's origin.
    static bool plane_plane(const Plane& plane0, const Plane& plane1, Line& output);

    /// Intersection line of two planes, anchored at the foot of the world origin.
    static bool plane_plane_to_line_canonical(const Plane& plane0, const Plane& plane1, Line& output);

    /// Intersection point of a line and a plane.
    static bool line_plane(const Line& line, const Plane& plane, Point& output, bool is_finite = true);

    /// Intersection point of three planes.
    static bool plane_plane_plane(const Plane& plane0, const Plane& plane1, const Plane& plane2, Point& output);

    // ═══════════════════════════════════════════════════════════════════════════
    // Rays
    // ═══════════════════════════════════════════════════════════════════════════
    /// Ray-box slab test returning the entry and exit parameters.
    static bool ray_box(
        const Point& origin,
        const Vector& direction,
        const OBB& box,
        double t0,
        double t1,
        double& tmin,
        double& tmax
    );

    /// Line-box slab test returning the entry and exit parameters.
    static bool ray_box(const Line& line, const OBB& box, double t0, double t1, double& tmin, double& tmax);

    /// Line-box entry and exit points.
    static bool ray_box(
        const Line& line,
        const OBB& box,
        double t0,
        double t1,
        std::vector<Point>& intersection_points
    );

    /// Ray-sphere parameters, returning the hit count.
    static int ray_sphere(
        const Point& origin,
        const Vector& direction,
        const Point& center,
        double radius,
        double& t0,
        double& t1
    );

    /// Line-sphere hit points.
    static bool ray_sphere(
        const Line& line,
        const Point& center,
        double radius,
        std::vector<Point>& intersection_points
    );

    /// Moller-Trumbore ray-triangle test.
    static bool ray_triangle(
        const Point& origin,
        const Vector& direction,
        const Point& v0,
        const Point& v1,
        const Point& v2,
        double epsilon,
        double& t,
        double& u,
        double& v,
        bool& parallel
    );

    /// Line-triangle hit point.
    static bool ray_triangle(
        const Line& line,
        const Point& v0,
        const Point& v1,
        const Point& v2,
        double epsilon,
        Point& output
    );

    /// Ray-mesh hits by brute force sorted by t, only the nearest unless find_all.
    static bool ray_mesh(
        const Point& origin,
        const Vector& direction,
        const Mesh& mesh,
        std::vector<RayHit>& hits,
        bool find_all = false,
        double epsilon = Tolerance::ZERO_TOLERANCE
    );

    /// Ray-mesh hits through the mesh's triangle BVH sorted by t, only the nearest unless find_all.
    static bool ray_mesh_bvh(
        const Point& origin,
        const Vector& direction,
        const Mesh& mesh,
        std::vector<RayHit>& hits,
        bool find_all = false,
        double epsilon = Tolerance::ZERO_TOLERANCE
    );

    /// Line-mesh hit points by brute force sorted by t, only the nearest unless find_all.
    static std::vector<Point> ray_mesh(const Line& line, const Mesh& mesh, double epsilon, bool find_all = false);

    /// Line-mesh hit points through the mesh's triangle BVH sorted by t, only the nearest unless find_all.
    static std::vector<Point> ray_mesh_bvh(const Line& line, const Mesh& mesh, double epsilon, bool find_all = false);

    // ═══════════════════════════════════════════════════════════════════════════
    // NURBS curves
    // ═══════════════════════════════════════════════════════════════════════════
    /// Curve-plane intersection parameters by sampling, bisection and Newton refinement.
    static std::vector<double> curve_plane(
        const NurbsCurve& curve,
        const Plane& plane,
        double tolerance = Tolerance::ZERO_TOLERANCE
    );

    /// Curve-plane intersection points.
    static std::vector<Point> curve_plane_points(
        const NurbsCurve& curve,
        const Plane& plane,
        double tolerance = Tolerance::ZERO_TOLERANCE
    );

    /// Curve-plane intersection parameters by Bezier clipping.
    static std::vector<double> curve_plane_bezier_clipping(
        const NurbsCurve& curve,
        const Plane& plane,
        double tolerance = Tolerance::ZERO_TOLERANCE
    );

    /// Curve-plane intersection parameters by hodograph subdivision.
    static std::vector<double> curve_plane_algebraic(
        const NurbsCurve& curve,
        const Plane& plane,
        double tolerance = Tolerance::ZERO_TOLERANCE
    );

    /// Curve-plane intersection parameters by span subdivision and Newton polishing.
    static std::vector<double> curve_plane_production(
        const NurbsCurve& curve,
        const Plane& plane,
        double tolerance = Tolerance::ZERO_TOLERANCE
    );

    /// Closest curve parameter and distance to a point, optionally within [t0, t1].
    static std::pair<double, double> curve_closest_point(
        const NurbsCurve& curve,
        const Point& test_point,
        double t0 = 0.0,
        double t1 = 0.0
    );

    // ═══════════════════════════════════════════════════════════════════════════
    // NURBS surfaces
    // ═══════════════════════════════════════════════════════════════════════════
    /// Surface-plane section curves.
    static std::vector<NurbsCurve> surface_plane(
        const NurbsSurface& surface,
        const Plane& plane,
        double tolerance = Tolerance::ZERO_TOLERANCE
    );

    /// Surface-plane section curves paired with their UV pcurves.
    static std::vector<std::pair<NurbsCurve, NurbsCurve>> surface_plane_uv(
        const NurbsSurface& surface,
        const Plane& plane,
        double tolerance = Tolerance::ZERO_TOLERANCE
    );

    /// Surface-surface section curves with their UV pcurves on both surfaces.
    static std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> surface_surface(
        const NurbsSurface& a,
        const NurbsSurface& b,
        double tolerance = Tolerance::ZERO_TOLERANCE
    );

    /// UV pcurves of the cutter's section on the target, clipped to the cutter footprint.
    static std::vector<NurbsCurve> cut_curves_on_surface(
        const NurbsSurface& target,
        const NurbsSurface& cutter,
        double tolerance = Tolerance::ZERO_TOLERANCE
    );

    // ═══════════════════════════════════════════════════════════════════════════
    // Polylines and plane sets
    // ═══════════════════════════════════════════════════════════════════════════
    /// Three-plane intersection that rejects near-parallel pairs.
    static bool plane_plane_plane_check(
        const Plane& p0,
        const Plane& p1,
        const Plane& p2,
        double angle_tol,
        Point& output
    );

    /// Linear remap of val from [from1, to1] to [from2, to2].
    static double remap(double val, double from1, double to1, double from2, double to2);

    /// Closest point on a finite segment and its parameter in [0, 1].
    static bool closest_point_on_segment(const Point& pt, const Line& seg, Point& output, double& t);

    /// Closed quad of the main plane cut by four ordered boundary planes.
    static bool plane_4planes(const Plane& main_plane, const std::array<Plane, 4>& planes, Polyline& output);

    /// Open four-point polyline of the main plane cut by four ordered boundary planes.
    static bool plane_4planes_open(const Plane& main_plane, const std::array<Plane, 4>& planes, Polyline& output);

    /// Closed quad of a plane cut by four infinite lines.
    static bool plane_4lines(
        const Plane& plane,
        const Line& l0,
        const Line& l1,
        const Line& l2,
        const Line& l3,
        Polyline& output
    );

    /// Clips a segment to the two plane intersections.
    static bool line_two_planes(const Line& line, const Plane& plane0, const Plane& plane1, Line& output);

    /// Polyline edge crossings with a plane and their edge indices.
    static bool polyline_plane(
        const Polyline& polyline,
        const Plane& plane,
        std::vector<Point>& points,
        std::vector<int>& edge_ids
    );

    /// Closest approach point on the infinite cutter to the segment.
    static bool line_line_3d(const Line& cutter, const Line& seg, Point& output);

    /// Direction scaled to span the distance between two planes.
    static bool scale_vector_to_distance_of_2planes(
        const Vector& direction,
        const Plane& plane0,
        const Plane& plane1,
        Vector& output
    );

    // ═══════════════════════════════════════════════════════════════════════════
    // Polyline booleans
    // ═══════════════════════════════════════════════════════════════════════════
    /// Boolean of two closed planar polylines, clip_type 0 intersection, 1 union, 2 difference.
    static std::vector<Polyline> polyline_boolean(const Polyline& a, const Polyline& b, int clip_type);

    /// Miter offset of a closed polyline in the plane's 2D frame, positive outward, in place.
    static bool offset_in_3d(Polyline& polyline, const Plane& plane, double offset);

    /// Boolean in the plane's 2D frame, intersection_type 0 intersect, 1 union, 2 difference, 3 xor.
    static bool polyline_boolean_2d_in_plane(
        const Polyline& polyline0,
        const Polyline& polyline1,
        const Plane& plane,
        Polyline& intersection_result,
        int intersection_type,
        bool include_triangles = false,
        double min_area = 0.01,
        double collapse_eps = 0.0
    );

    // ═══════════════════════════════════════════════════════════════════════════
    // Joints
    // ═══════════════════════════════════════════════════════════════════════════
    /// Polyline-plane crossings as one line oriented from align_start.
    static bool polyline_plane_to_line(const Polyline& poly, const Plane& plane, const Point& align_start, Line& out);

    /// Closed quad from a joint line bounded by top and bottom planes on a face plane.
    static bool quad_from_line_top_bottom_planes(
        const Plane& face_plane,
        const Line& line,
        const Plane& plane0,
        const Plane& plane1,
        Polyline& out
    );

    /// Vector orthogonal to the (pp00, pp10) line, anchored on the (pp00, pp11) line.
    static bool orthogonal_vector_between_two_plane_pairs(
        const Plane& pp00,
        const Plane& pp10,
        const Plane& pp11,
        Vector& out
    );

    /// Open joint outline clipped to a closed plate polygon with the plate edge parameters.
    static bool closed_and_open_paths_2d(
        const Polyline& plate,
        const Polyline& joint,
        const Plane& plane,
        Polyline& out,
        std::pair<double, double>& cp_pair
    );

    // ═══════════════════════════════════════════════════════════════════════════
    // Elements
    // ═══════════════════════════════════════════════════════════════════════════
    /// Face-to-face contacts (a, b, face_a, face_b, type, polyline) with type 0 side-side, 1 side-top, 2 top-top.
    static std::vector<std::tuple<int, int, int, int, int, Polyline>> face_to_face(
        const std::vector<int>& adjacency,
        const std::vector<std::vector<Polyline>>& polylines,
        const std::vector<std::vector<Plane>>& planes,
        double coplanar_tolerance = 5.0
    );

    /// Face-to-face contacts from elements.
    static std::vector<std::tuple<int, int, int, int, int, Polyline>> face_to_face(
        const std::vector<int>& adjacency,
        std::vector<Element*>& elements,
        double coplanar_tolerance = 5.0
    );

    /// Adjacent element pairs by BVH broad phase and OBB narrow phase.
    static std::vector<int> adjacency_search(std::vector<Element*>& elements, double inflate = 5.0);

    /// Classifies two segments as end-to-end, side-to-end or cross with closest points and directions.
    static bool line_line_classified(
        const Line& s0,
        const Line& s1,
        int n_segs_0,
        int n_segs_1,
        int cur_seg_0,
        int cur_seg_1,
        double above_closer_to_edge,
        Point& p0,
        Point& p1,
        Vector& v0,
        Vector& v1,
        Vector& normal,
        bool& type0,
        bool& type1,
        bool& is_parallel
    );

private:
    // ═══════════════════════════════════════════════════════════════════════════
    // Helpers
    // ═══════════════════════════════════════════════════════════════════════════
    /// Gaussian elimination of a 3x3 system with full pivoting, returning the rank.
    static int solve_3x3(
        const double row0[3],
        const double row1[3],
        const double row2[3],
        double d0,
        double d1,
        double d2,
        double& x,
        double& y,
        double& z,
        double& pivot_ratio
    );

    /// Signed plane equation value at a point.
    static double plane_value_at(const Plane& plane, const Point& point);
};

} // namespace session_cpp
