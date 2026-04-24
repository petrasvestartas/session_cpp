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
#include <vector>
#include <optional>

namespace session_cpp {

// Forward declarations
class NurbsCurve;
class NurbsSurface;

class Intersection {
public:

  /**
   * @brief Ray-mesh intersection result containing hit information
   */
  struct RayHit {
    double t;           ///< Parameter along ray where intersection occurs
    Point point;       ///< 3D intersection point
    double u;           ///< Barycentric coordinate u
    double v;           ///< Barycentric coordinate v
    int face_index;    ///< Index of the intersected face
    
    RayHit() : t(0.0), point(), u(0.0), v(0.0), face_index(-1) {}
    RayHit(double t_, const Point& p, double u_ = 0.0, double v_ = 0.0, int face_idx = -1)
      : t(t_), point(p), u(u_), v(v_), face_index(face_idx) {}
  };

  /**
   * @brief Find intersection point between two 3D lines
   * @param line0 First line
   * @param line1 Second line
   * @param output Intersection point (midpoint of closest approach for skew lines)
   * @param tolerance Maximum distance between lines to consider them intersecting
   * @return true if lines intersect within tolerance, false otherwise
   */
  static bool line_line(
    const Line& line0,
    const Line& line1,
    Point& output,
    double tolerance
  );

  /**
   * @brief Find parametric values where two lines are closest
   * @param line0 First line
   * @param line1 Second line
   * @param t0 Parameter on line0 (0=start, 1=end)
   * @param t1 Parameter on line1 (0=start, 1=end)
   * @param tolerance Maximum distance to consider intersection
   * @param intersect_segments If true, clamp parameters to [0,1]; if false, treat as infinite lines
   * @param near_parallel_as_closest If true, return closest point for near-parallel lines
   * @return true if intersection found within tolerance, false otherwise
   */
  static bool line_line_parameters(
    const Line& line0,
    const Line& line1,
    double& t0,
    double& t1,
    double tolerance,
    bool intersect_segments = true,
    bool near_parallel_as_closest = false
  );

  /**
   * @brief Find intersection line between two planes
   * @param plane0 First plane
   * @param plane1 Second plane
   * @param output Intersection line (infinite)
   * @return true if planes intersect (not parallel), false otherwise
   */
  static bool plane_plane(
    const Plane& plane0,
    const Plane& plane1,
    Line& output
  );

  /**
   * @brief Plane-plane intersection line with CGAL-canonical anchor
   * (closest-to-world-origin point on the line).
   *
   * Matches wood's `cgal::intersection_util::plane_plane` at
   * `cgal_intersection_util.cpp:493-511` which uses CGAL's
   * `Plane_3::point()` = foot-of-perpendicular from origin canonical.
   * The anchor is independent of input-plane origin choice, so the
   * downstream `l1.start() - l0.projection(l1.start())` formula in
   * `orthogonal_vector_between_two_plane_pairs` gives a bit-exact
   * match to wood for parallel input planes (e.g., plate top/bottom
   * face pairs in ts_e_p joint construction).
   *
   * Existing `plane_plane` is left intact — pinned by the
   * `Plane Plane Complex` minitest which hardcodes the specific
   * anchor values produced by the midpoint-bisector formulation.
   *
   * @param plane0 First plane
   * @param plane1 Second plane
   * @param output Intersection line (anchor = canonical foot of origin)
   * @return true if planes intersect (not parallel), false otherwise
   */
  static bool plane_plane_to_line_canonical(
    const Plane& plane0,
    const Plane& plane1,
    Line& output
  );

  /**
   * @brief Find intersection point between a line and a plane
   * @param line Line to intersect
   * @param plane Plane to intersect
   * @param output Intersection point
   * @param is_finite If true, treat line as finite segment; if false, treat as infinite
   * @return true if intersection exists, false if line is parallel to plane
   */
  static bool line_plane(
    const Line& line,
    const Plane& plane,
    Point& output,
    bool is_finite = true
  );

  /**
   * @brief Find intersection point of three planes
   * @param plane0 First plane
   * @param plane1 Second plane
   * @param plane2 Third plane
   * @param output Intersection point
   * @return true if planes intersect at a point, false if parallel or degenerate
   */
  static bool plane_plane_plane(
    const Plane& plane0,
    const Plane& plane1,
    const Plane& plane2,
    Point& output
  );

  /**
   * @brief Find parametric intersection of ray with axis-aligned bounding box
   * @param origin Ray origin point
   * @param direction Ray direction vector (not necessarily normalized)
   * @param box Axis-aligned bounding box
   * @param t0 Minimum parameter value to consider (e.g., 0.0 for ray origin)
   * @param t1 Maximum parameter value to consider (e.g., 1000.0 for max distance)
   * @param tmin Output: parameter where ray enters box
   * @param tmax Output: parameter where ray exits box
   * @return true if ray intersects box within [t0, t1], false otherwise
   */
  static bool ray_box(
    const Point& origin,
    const Vector& direction,
    const OBB& box,
    double t0,
    double t1,
    double& tmin,
    double& tmax
  );

  /**
   * @brief Find parametric intersection of line with axis-aligned bounding box
   * @param line Line to intersect (start point + direction)
   * @param box Axis-aligned bounding box
   * @param t0 Minimum parameter value to consider
   * @param t1 Maximum parameter value to consider
   * @param tmin Output: parameter where line enters box
   * @param tmax Output: parameter where line exits box
   * @return true if line intersects box within [t0, t1], false otherwise
   */
  static bool ray_box(
    const Line& line,
    const OBB& box,
    double t0,
    double t1,
    double& tmin,
    double& tmax
  );

  /**
   * @brief Find intersection points between line and axis-aligned bounding box
   * @param line Line to intersect
   * @param box Axis-aligned bounding box
   * @param t0 Minimum parameter value to consider
   * @param t1 Maximum parameter value to consider
   * @param intersection_points Output: vector of intersection points (entry and exit)
   * @return true if intersection exists, false otherwise
   * @note Returns 2 points: [0]=entry (closer to line start), [1]=exit (farther from line start)
   */
  static bool ray_box(
    const Line& line,
    const OBB& box,
    double t0,
    double t1,
    std::vector<Point>& intersection_points
  );

  /**
   * @brief Find parametric intersection of ray with sphere
   * @param origin Ray origin point
   * @param direction Ray direction vector (not necessarily normalized)
   * @param center Sphere center point
   * @param radius Sphere radius
   * @param t0 Output: parameter of first intersection (closer to origin)
   * @param t1 Output: parameter of second intersection (farther from origin)
   * @return Number of intersections: 0 (miss), 1 (tangent), or 2 (entry/exit)
   */
  static int ray_sphere(
    const Point& origin,
    const Vector& direction,
    const Point& center,
    double radius,
    double& t0,
    double& t1
  );

  /**
   * @brief Find intersection points between line and sphere
   * @param line Line to intersect
   * @param center Sphere center point
   * @param radius Sphere radius
   * @param intersection_points Output: vector of intersection points
   * @return true if intersection exists, false otherwise
   * @note Returns 1 point (tangent) or 2 points (entry/exit), sorted from line start
   */
  static bool ray_sphere(
    const Line& line,
    const Point& center,
    double radius,
    std::vector<Point>& intersection_points
  );

  /**
   * @brief Find intersection of ray with triangle using Möller-Trumbore algorithm
   * @param origin Ray origin point
   * @param direction Ray direction vector (not necessarily normalized)
   * @param v0 First vertex of triangle
   * @param v1 Second vertex of triangle
   * @param v2 Third vertex of triangle
   * @param epsilon Tolerance for parallel detection
   * @param t Output: parameter along ray where intersection occurs
   * @param u Output: barycentric coordinate u
   * @param v Output: barycentric coordinate v (w = 1-u-v)
   * @param parallel Output: true if ray is parallel to triangle
   * @return true if intersection exists, false otherwise
   */
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

  /**
   * @brief Find intersection point between line and triangle
   * @param line Line to intersect (start point used as origin, direction computed internally)
   * @param v0 First vertex of triangle
   * @param v1 Second vertex of triangle
   * @param v2 Third vertex of triangle
   * @param epsilon Tolerance for parallel detection
   * @param output Output: intersection point
   * @return true if intersection exists, false otherwise
   */
  static bool ray_triangle(
    const Line& line,
    const Point& v0,
    const Point& v1,
    const Point& v2,
    double epsilon,
    Point& output
  );

  /**
   * @brief Find intersection of ray with mesh (naive brute-force)
   * @param origin Ray origin point
   * @param direction Ray direction vector (not necessarily normalized)
   * @param mesh Mesh to intersect
   * @param hits Output: vector of ray-triangle intersections
   * @param find_all If true, find all intersections; if false, stop at first hit
   * @return true if any intersection found, false otherwise
   */
  static bool ray_mesh(
    const Point& origin,
    const Vector& direction,
    const Mesh& mesh,
    std::vector<RayHit>& hits,
    bool find_all = false
  );

  /**
   * @brief Find intersection of ray with mesh using SpatialBVH acceleration
   * @param origin Ray origin point
   * @param direction Ray direction vector (not necessarily normalized)
   * @param mesh Mesh to intersect
   * @param hits Output: vector of ray-triangle intersections
   * @param find_all If true, find all intersections; if false, stop at first hit
   * @return true if any intersection found, false otherwise
   * @note Uses broad-phase culling with per-triangle AABBs (no hierarchical traversal)
   */
  static bool ray_mesh_bvh(
    const Point& origin,
    const Vector& direction,
    const Mesh& mesh,
    std::vector<RayHit>& hits,
    bool find_all = false
  );

  /**
   * @brief Find intersection(s) of line with mesh (simplified API)
   * @param line Line to intersect with mesh
   * @param mesh Mesh to intersect
   * @param epsilon Tolerance for intersection detection
   * @param find_all If true, find all intersections; if false, return only closest
   * @return Vector of intersection points sorted along the line (empty if no hits)
   */
  static std::vector<Point> ray_mesh(
    const Line& line,
    const Mesh& mesh,
    double epsilon,
    bool find_all = false
  );

  /**
   * @brief Find intersection(s) of line with mesh using SpatialBVH (simplified API)
   * @param line Line to intersect with mesh
   * @param mesh Mesh to intersect
   * @param epsilon Tolerance for intersection detection
   * @param find_all If true, find all intersections; if false, return only closest
   * @return Vector of intersection points sorted along the line (empty if no hits)
   */
  static std::vector<Point> ray_mesh_bvh(
    const Line& line,
    const Mesh& mesh,
    double epsilon,
    bool find_all = false
  );

  //==========================================================================================
  // NURBS Curve Intersection Methods
  //==========================================================================================

  /**
   * @brief Find all intersections between NURBS curve and plane
   * @param curve NURBS curve to intersect
   * @param plane Plane to intersect with
   * @param tolerance Intersection tolerance
   * @return Vector of parameter values where curve intersects plane
   */
  static std::vector<double> curve_plane(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance = Tolerance::ZERO_TOLERANCE
  );

  /**
   * @brief Find all intersection points between NURBS curve and plane
   * @param curve NURBS curve to intersect
   * @param plane Plane to intersect with
   * @param tolerance Intersection tolerance
   * @return Vector of intersection points
   */
  static std::vector<Point> curve_plane_points(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance = Tolerance::ZERO_TOLERANCE
  );

  /**
   * @brief Curve-plane intersection using Bézier clipping (advanced method)
   * @param curve NURBS curve to intersect
   * @param plane Plane to intersect with
   * @param tolerance Intersection tolerance
   * @return Vector of parameter values where curve intersects plane
   * @note Faster for multiple intersections, used in professional CAD software
   */
  static std::vector<double> curve_plane_bezier_clipping(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance = Tolerance::ZERO_TOLERANCE
  );

  /**
   * @brief Curve-plane intersection using algebraic/hodograph method
   * @param curve NURBS curve to intersect
   * @param plane Plane to intersect with
   * @param tolerance Intersection tolerance
   * @return Vector of parameter values where curve intersects plane
   * @note Maximum precision method using Newton-Raphson with derivatives
   */
  static std::vector<double> curve_plane_algebraic(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance = Tolerance::ZERO_TOLERANCE
  );

  /**
   * @brief Curve-plane intersection using production CAD kernel method
   * @param curve NURBS curve to intersect
   * @param plane Plane to intersect with
   * @param tolerance Intersection tolerance
   * @return Vector of parameter values where curve intersects plane
   * @note Industry standard method used in Rhino, Parasolid, ACIS
   */
  static std::vector<double> curve_plane_production(
    const NurbsCurve& curve,
    const Plane& plane,
    double tolerance = Tolerance::ZERO_TOLERANCE
  );

  /**
   * @brief Find closest point on NURBS curve to test point
   * @param curve NURBS curve
   * @param test_point Point to find closest point to
   * @param t0 Start of search interval (0=use curve start)
   * @param t1 End of search interval (0=use curve end)
   * @return Pair of (parameter, distance)
   */
  static std::pair<double, double> curve_closest_point(
    const NurbsCurve& curve,
    const Point& test_point,
    double t0 = 0.0,
    double t1 = 0.0
  );

  //==========================================================================================
  // NURBS Surface Intersection Methods
  //==========================================================================================

  /**
   * @brief Find intersection curves between NURBS surface and plane
   * @param surface NURBS surface to intersect
   * @param plane Plane to intersect with
   * @param tolerance Intersection tolerance
   * @return Vector of NurbsCurve intersection curves
   * @note Uses curve tracing: seed finding on UV grid, predictor-corrector
   *       marching along g(u,v)=0 with Newton refinement, then interpolation
   */
  static std::vector<NurbsCurve> surface_plane(
    const NurbsSurface& surface,
    const Plane& plane,
    double tolerance = Tolerance::ZERO_TOLERANCE
  );

  /**
   * @brief Three-plane intersection with parallel-pair guard
   * @param p0 First plane
   * @param p1 Second plane
   * @param p2 Third plane
   * @param angle_tol Angle tolerance in radians; pairs with |cos| >= cos(tol) are rejected
   * @param output Intersection point
   * @return true if no two planes are nearly parallel and a point exists
   */
  static bool plane_plane_plane_check(
    const Plane& p0,
    const Plane& p1,
    const Plane& p2,
    double angle_tol,
    Point& output
  );

  /**
   * @brief Linear remap: map val from [from1,to1] to [from2,to2]
   */
  static double remap(double val, double from1, double to1, double from2, double to2);

  /**
   * @brief Project point onto finite segment; return closest point and parameter t in [0,1]
   * @param pt Test point
   * @param seg Segment
   * @param output Closest point on segment
   * @param t Parameter in [0,1]
   * @return true always
   */
  static bool closest_point_on_segment(
    const Point& pt,
    const Line& seg,
    Point& output,
    double& t
  );

  /**
   * @brief Intersect main plane with 4 ordered boundary planes; closed quad (5 pts)
   * @param main_plane Main cutting plane
   * @param planes Array of 4 boundary planes (consecutive pairs define edges)
   * @param output Closed polyline (first == last, 5 points)
   * @return true if all 4 corner intersections succeeded
   */
  static bool plane_4planes(
    const Plane& main_plane,
    const std::array<Plane, 4>& planes,
    Polyline& output
  );

  /**
   * @brief Intersect main plane with 4 ordered boundary planes; open polyline (4 pts)
   * @param main_plane Main cutting plane
   * @param planes Array of 4 boundary planes
   * @param output Open polyline (4 points)
   * @return true if all 4 corner intersections succeeded
   */
  static bool plane_4planes_open(
    const Plane& main_plane,
    const std::array<Plane, 4>& planes,
    Polyline& output
  );

  /**
   * @brief Intersect plane with 4 infinite lines; closed quad (5 pts)
   * @param plane Cutting plane
   * @param l0 First line
   * @param l1 Second line
   * @param l2 Third line
   * @param l3 Fourth line
   * @param output Closed polyline (5 points, first == last)
   * @return true if all 4 intersections succeeded
   */
  static bool plane_4lines(
    const Plane& plane,
    const Line& l0,
    const Line& l1,
    const Line& l2,
    const Line& l3,
    Polyline& output
  );

  /**
   * @brief Clip line segment so start/end snap to two plane intersections
   * @param line Input segment (finite)
   * @param plane0 Plane for start point
   * @param plane1 Plane for end point
   * @param output Clipped line
   * @return true if both intersections exist within the segment
   */
  static bool line_two_planes(
    const Line& line,
    const Plane& plane0,
    const Plane& plane1,
    Line& output
  );

  /**
   * @brief Find all polyline perimeter edge intersections with a plane
   * @param polyline Input polyline
   * @param plane Cutting plane
   * @param points Output intersection points
   * @param edge_ids Output edge indices (0-based) where each hit occurred
   * @return true if any intersections found
   */
  static bool polyline_plane(
    const Polyline& polyline,
    const Plane& plane,
    std::vector<Point>& points,
    std::vector<int>& edge_ids
  );

  /**
   * @brief 3D skew-line intersection via closest approach on cutter (infinite lines)
   * @param cutter Line whose point is returned
   * @param seg Second line
   * @param output Point on cutter at closest approach
   * @return true if lines are not parallel
   */
  static bool line_line_3d(
    const Line& cutter,
    const Line& seg,
    Point& output
  );

  /**
   * @brief Scale direction vector to span the distance between two planes
   * @param direction Direction vector
   * @param plane0 First plane
   * @param plane1 Second plane
   * @param output Scaled vector connecting plane0 to plane1 along direction
   * @return true if successful
   */
  static bool scale_vector_to_distance_of_2planes(
    const Vector& direction,
    const Plane& plane0,
    const Plane& plane1,
    Vector& output
  );

  /**
   * @brief Polyline-perimeter ∩ plane → single oriented Line.
   *
   * Walks the perimeter of `poly`, finds all edge-plane intersections,
   * and returns a 2-point line through the first two hits. The output
   * orientation is chosen so `out.start()` is the closer endpoint to
   * `align_start`. Mirrors the wood `polyline_plane_to_line` helper used
   * by `face_to_face` to align joint lines along a side-face axis.
   *
   * @param poly Polyline whose edges are tested.
   * @param plane Plane to intersect.
   * @param align_start Reference point for picking which intersection is the start.
   * @param out Output line (start = closer to align_start, end = farther).
   * @return true if at least 2 intersections were found.
   */
  static bool polyline_plane_to_line(const Polyline& poly,
                                      const Plane& plane,
                                      const Point& align_start,
                                      Line& out);

  /**
   * @brief Build a closed quad polyline from a joint line plus the
   *        top/bottom thickness planes of an element.
   *
   * The quad's two long edges lie in `face_plane`, running along
   * `line`, and its short edges are clipped by `plane0` (bottom) and
   * `plane1` (top). Used by the wood face_to_face top-side branch to
   * assemble joint volumes.
   *
   * @param face_plane The element's side-face plane (containing line).
   * @param line The joint line (segment along the side face).
   * @param plane0 Element's bottom plane.
   * @param plane1 Element's top plane.
   * @param out Output closed quad (5 points: p0, p1, p2, p3, p0).
   * @return true if all four 3-plane intersections succeeded.
   */
  static bool quad_from_line_top_bottom_planes(const Plane& face_plane,
                                                const Line& line,
                                                const Plane& plane0,
                                                const Plane& plane1,
                                                Polyline& out);

  /**
   * @brief Orthogonal vector connecting two infinite lines defined by
   *        `(pp00 ∩ pp10)` and `(pp00 ∩ pp11)`.
   *
   * Both intersection lines share `pp00`. The result is the
   * displacement from a point on `l0` to its closest approach on `l1`.
   * Wood uses this in the type-20 (top-side) joint branch to compute
   * the offset vector that takes the male's quad corners INTO the
   * female plate.
   *
   * @param pp00 Common plane (typically the male's top plane).
   * @param pp10 First intersected plane (typically the female's collision face).
   * @param pp11 Second intersected plane (typically the female's opposite face).
   * @param out Output displacement vector pointing from l0 → l1.
   * @return true on success (false if either plane-plane fails).
   */
  static bool orthogonal_vector_between_two_plane_pairs(const Plane& pp00,
                                                         const Plane& pp10,
                                                         const Plane& pp11,
                                                         Vector& out);

  /**
   * @brief Clip an OPEN-path joint outline against a CLOSED plate
   *        polygon in 2D and return the clipped 3D segment plus
   *        parametric positions on the plate edges where it touches.
   *
   * Uses Clipper2's `AddOpenSubject` for the joint outline and
   * `AddClip` for the plate polygon. Mirrors wood's
   * `intersection_closed_and_open_paths_2D` (`wood_element.cpp:438-651`).
   * Used by `merge_joints` for the case-5 rectangle joint branch.
   *
   * @param plate The plate polygon (closed; first point may equal last).
   * @param joint The joint outline (treated as an open path).
   * @param plane The plate plane (used for the 2D projection basis).
   * @param out Output clipped 3D polyline (oriented start → end matching the
   *            plate's edge order).
   * @param cp_pair Output `(t0, t1)` parametric positions on the plate
   *                edges where the clip starts/ends. Each `t` is
   *                `edge_index + fractional_position_on_edge`.
   * @return true if a non-empty clip was found.
   */
  static bool closed_and_open_paths_2d(const Polyline& plate,
                                        const Polyline& joint,
                                        const Plane& plane,
                                        Polyline& out,
                                        std::pair<double, double>& cp_pair);

  /// Boolean operation on two closed planar polylines.
  /// clip_type: 0=intersection, 1=union, 2=difference (a minus b).
  static std::vector<Polyline> polyline_boolean(const Polyline& a, const Polyline& b, int clip_type);

  /// 2D polygon offset in plane space using Clipper2 InflatePaths (Miter,
  /// Polygon end). Transforms `polyline` to the 2D frame of `plane`, offsets
  /// by `offset` (positive = outward), rotates result to best-align with the
  /// original first vertex, and transforms back to 3D. Mutates `polyline` to
  /// the offset result. Returns false if Clipper2 returns no paths or the
  /// result is degenerate (area < 0.0001). Verbatim port of wood's
  /// `cgal::collider::clipper_util::offset_in_3d` at
  /// `clipper_util.cpp:707-790`.
  static bool offset_in_3d(Polyline& polyline, const Plane& plane, double offset);

  /// 2D boolean between two closed planar polylines, projected into the
  /// plane's canonical 2D frame (base1/base2).
  /// `intersection_type`: 0=Intersect, 1=Union, 2=Difference, 3=Xor.
  /// `include_triangles`: when false, 3-vertex results are rejected.
  /// `collapse_eps`: when > 0, consecutive output vertices closer than this
  /// distance (in 2D) are merged. Removes Vatti FP-noise on near-coincident
  /// edges — needed by wood hexbox-family datasets.
  /// Writes result back to `intersection_result`; returns false on empty,
  /// degenerate (<3 vertices), triangle-reject, or area <= `min_area`.
  /// Verbatim port of wood's `get_intersection_between_two_polylines` at
  /// `clipper_util.cpp:524-620`.
  static bool polyline_boolean_2d_in_plane(
      const Polyline& polyline0,
      const Polyline& polyline1,
      const Plane& plane,
      Polyline& intersection_result,
      int intersection_type,
      bool include_triangles = false,
      double min_area = 0.01,
      double collapse_eps = 0.0);

  /// Face-to-face joint detection between elements via coplanar boolean intersection.
  /// adjacency: flat array [a0,b0,?,?, a1,b1,?,?, ...] processed in groups of 4.
  /// Returns: vector of (element_a, element_b, face_a, face_b, type, polyline).
  /// type: 0=side-side, 1=side-top, 2=top-top.
  static std::vector<std::tuple<int, int, int, int, int, Polyline>> face_to_face(
    const std::vector<int>& adjacency,
    const std::vector<std::vector<Polyline>>& polylines,
    const std::vector<std::vector<Plane>>& planes,
    double coplanar_tolerance = 5.0
  );

  /// Adjacency search: SpatialBVH broad phase + OBB SAT narrow phase.
  static std::vector<int> adjacency_search(
    std::vector<Element*>& elements, double inflate = 5.0);

  /// Face-to-face overload taking elements directly.
  static std::vector<std::tuple<int, int, int, int, int, Polyline>> face_to_face(
    const std::vector<int>& adjacency,
    std::vector<Element*>& elements,
    double coplanar_tolerance = 5.0
  );

  // Note: CrossJoint struct + plane_to_face overloads + set_cross_joint_distance_squared
  // were wood-domain API and have been relocated to wood/wood_session.h +
  // wood/wood_joint_detection.cpp. Callers should use `wood_session::CrossJoint`
  // and `wood_session::plane_to_face(...)` from `wood/wood_session.h`.

  // Port of cgal_box_search.h:252-496. Classifies two finite segments s0/s1
  // as end-to-end, side-to-end, or cross based on above_closer_to_edge ∈ [0,1].
  // n_segs_*/cur_seg_* give the segment's position within its parent polyline
  // (used for full-polyline parameter remapping that determines type0/type1).
  // Outputs: p0/p1 = closest approach points (clamped to [0,1] on each segment);
  //          v0/v1 = unit directions, flipped away from the far end when type=0;
  //          normal = unit perpendicular to both segments (Plane::base1 if parallel);
  //          type0/type1 = 0 means segment-end, 1 means segment-interior;
  //          is_parallel = true when v0 × v1 is near-zero.
  // Returns false for degenerate cases.
  static bool line_line_classified(
    const Line& s0,
    const Line& s1,
    int  n_segs_0,
    int  n_segs_1,
    int  cur_seg_0,
    int  cur_seg_1,
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
  static int solve_3x3(
    const double row0[3],
    const double row1[3],
    const double row2[3],
    double d0, double d1, double d2,
    double& x, double& y, double& z,
    double& pivot_ratio
  );
  static double plane_value_at(const Plane& plane, const Point& point);
};

} // namespace session_cpp
