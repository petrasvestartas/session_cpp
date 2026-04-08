// main_5.cpp — Example usage of `face_to_face_wood`, the detailed timber-joint
// topology detector ported from `cmake/src/wood/include/wood_main.cpp`.
//
// This file is intentionally self-contained: every helper the wood algorithm
// needs that is NOT already provided by session_cpp lives in the anonymous
// namespace below. There is no `WoodConfig` struct and no `wood::GLOBALS::*`
// state — every tunable that the original wood library reads from globals is
// passed in as an explicit function parameter, and `main()` defines them all
// as local variables right where they are used.
//
// Pipeline:
//   1. Read polylines from an OBJ, pair top/bottom faces, build PlateElements.
//   2. BVH broad-phase to find candidate adjacent pairs.
//   3. For every adjacent pair, call `face_to_face_wood` with locally-defined
//      wood parameters.
//   4. Print a histogram of joint types.

#include "session.h"
#include "element.h"
#include "obj.h"
#include "intersection.h"
#include "plane.h"
#include "polyline.h"
#include "line.h"
#include "vector.h"
#include "point.h"
#include "xform.h"
#include "tolerance.h"
#include "aabb.h"
#include "bvh.h"
#include <fmt/core.h>
#include <chrono>
#include <filesystem>
#include <vector>
#include <array>
#include <optional>
#include <utility>
#include <cmath>
#include <algorithm>

using namespace session_cpp;

namespace {

// ───────────────────────────────────────────────────────────────────────────
// Output struct — mirrors the seven out-parameters of the original C++
// wood::main::face_to_face (joint_id, el_ids, face_ids, type, joint_area,
// joint_lines, joint_volumes_pairA_pairB).
// ───────────────────────────────────────────────────────────────────────────
struct WoodJoint {
    std::pair<int, int> el_ids = {0, 0};
    std::pair<std::array<int, 2>, std::array<int, 2>> face_ids = { {{0,0}}, {{0,0}} };
    int joint_type = 0;
    Polyline joint_area = Polyline(std::vector<Point>{});
    std::array<Line, 2> joint_lines = {
        Line::from_points(Point(0,0,0), Point(0,0,0)),
        Line::from_points(Point(0,0,0), Point(0,0,0)),
    };
    std::array<std::optional<Polyline>, 4> joint_volumes_pair_a_pair_b{};
};

// ───────────────────────────────────────────────────────────────────────────
// Missing helpers — small geometric utilities that the wood algorithm needs
// but session_cpp does not currently expose. Each one is intentionally short
// and self-explanatory; if they prove generally useful they can be promoted
// to `intersection.h` / `plane.h` later.
// ───────────────────────────────────────────────────────────────────────────

// Sign test: signed distance from `p` to plane along its unit normal is < 0?
// session_cpp::Plane caches the standard ax+by+cz+d=0 form so this is just
// one mul-add — no square roots or normalization.
static bool plane_has_on_negative_side(const Plane& pl, const Point& p) {
    return (pl.a()*p[0] + pl.b()*p[1] + pl.c()*p[2] + pl.d()) < 0.0;
}

// Approximate dihedral angle (degrees, unsigned [0, 180]) of edge `pq` in the
// tetrahedron `pqrs`. Equivalent to `std::abs(CGAL::approximate_dihedral_angle(p,q,r,s))`,
// which is what the original wood code calls.
//
// Geometric definition: angle between half-plane (pqr) and half-plane (pqs)
// measured in the plane perpendicular to edge pq.
static double approximate_dihedral_angle_deg(const Point& p, const Point& q,
                                             const Point& r, const Point& s) {
    Vector pq(q[0]-p[0], q[1]-p[1], q[2]-p[2]);
    Vector pr(r[0]-p[0], r[1]-p[1], r[2]-p[2]);
    Vector ps(s[0]-p[0], s[1]-p[1], s[2]-p[2]);
    Vector n1 = pq.cross(pr);
    Vector n2 = pq.cross(ps);
    double m1 = n1.magnitude();
    double m2 = n2.magnitude();
    if (m1 < Tolerance::ZERO_TOLERANCE || m2 < Tolerance::ZERO_TOLERANCE) return 0.0;
    double cos_t = n1.dot(n2) / (m1 * m2);
    if (cos_t > 1.0) cos_t = 1.0;
    if (cos_t < -1.0) cos_t = -1.0;
    return std::acos(cos_t) * (180.0 / 3.141592653589793);
}

// Polyline-perimeter ∩ plane → single oriented Line, with output orientation
// chosen so that `out_line.start()` is the closer of the two intersection
// points to `align_start`. The wood code uses this to align the joint line
// to the side-face's natural axis.
static bool polyline_plane_to_line(const Polyline& poly, const Plane& plane,
                                   const Point& align_start, Line& out_line) {
    std::vector<Point> pts;
    std::vector<int> edge_ids;
    if (!Intersection::polyline_plane(poly, plane, pts, edge_ids)) return false;
    if (pts.size() < 2) return false;
    const Point& a = pts[0];
    const Point& b = pts[1];
    double da = (a[0]-align_start[0])*(a[0]-align_start[0]) +
                (a[1]-align_start[1])*(a[1]-align_start[1]) +
                (a[2]-align_start[2])*(a[2]-align_start[2]);
    double db = (b[0]-align_start[0])*(b[0]-align_start[0]) +
                (b[1]-align_start[1])*(b[1]-align_start[1]) +
                (b[2]-align_start[2])*(b[2]-align_start[2]);
    if (da <= db) out_line = Line::from_points(a, b);
    else          out_line = Line::from_points(b, a);
    return true;
}

// Build a closed quad polyline from a "joint line" segment plus the
// top/bottom thickness planes of an element. The quad's two long edges lie in
// `face_plane`, running along `line`, and its short edges are clipped by
// plane0 (bottom) and plane1 (top). Used to assemble the joint volumes for
// the top-side branch.
static bool get_quad_from_line_topbottomplanes(const Plane& face_plane,
                                               const Line& line,
                                               const Plane& plane0,
                                               const Plane& plane1,
                                               Polyline& out) {
    // End-cap planes perpendicular to the joint line at each endpoint.
    Vector dir = line.to_vector();
    Point s = line.start();
    Plane lp0 = Plane::from_point_normal(s, dir);
    Vector dir2 = line.to_vector();
    Point e = line.end();
    Plane lp1 = Plane::from_point_normal(e, dir2);

    // 4 corners as 3-plane intersections.
    Point p0, p1, p2, p3;
    if (!Intersection::plane_plane_plane(lp0, plane0, face_plane, p0)) return false;
    if (!Intersection::plane_plane_plane(lp0, plane1, face_plane, p1)) return false;
    if (!Intersection::plane_plane_plane(lp1, plane1, face_plane, p2)) return false;
    if (!Intersection::plane_plane_plane(lp1, plane0, face_plane, p3)) return false;
    out = Polyline(std::vector<Point>{p0, p1, p2, p3, p0});
    return true;
}

// Orthogonal vector connecting the two infinite lines defined by
//   (pp0_0 ∩ pp1_0)  and  (pp0_0 ∩ pp1_1).
// Both intersections share the first plane (pp0_0); the connecting vector
// equals the offset that, when added to a point on the first line, lands on
// the second line at its closest approach.
static bool get_orthogonal_vector_between_two_plane_pairs(const Plane& pp0_0,
                                                          const Plane& pp1_0,
                                                          const Plane& pp1_1,
                                                          Vector& out) {
    Line l0, l1;
    if (!Intersection::plane_plane(pp0_0, pp1_0, l0)) return false;
    if (!Intersection::plane_plane(pp0_0, pp1_1, l1)) return false;
    Point p_on_l0;
    if (!Intersection::line_line_3d(l0, l1, p_on_l0)) return false;
    Point p_on_l1;
    if (!Intersection::line_line_3d(l1, l0, p_on_l1)) return false;
    out = Vector(p_on_l1[0]-p_on_l0[0], p_on_l1[1]-p_on_l0[1], p_on_l1[2]-p_on_l0[2]);
    return true;
}

// Slide both endpoints of polyline edge `edge_idx` outward (or inward, for
// negative `distance`) along the edge tangent. Closing-duplicate vertices
// of closed polylines are kept in sync. The wood code calls this in
// opposite-edge pairs (0+2 and 1+3) which preserves rectangle shape and
// scales the rectangle uniformly along its two principal axes.
static void extend_polyline_edge_equally(Polyline& poly, size_t edge_idx, double distance) {
    size_t n = poly.point_count();
    if (n < 2 || edge_idx + 1 >= n) return;
    size_t i = edge_idx;
    size_t j = edge_idx + 1;
    Point pi = poly.get_point(i);
    Point pj = poly.get_point(j);
    double dx = pj[0] - pi[0];
    double dy = pj[1] - pi[1];
    double dz = pj[2] - pi[2];
    double len = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (len < 1e-12) return;
    double inv = 1.0 / len;
    double ux = dx * inv * distance;
    double uy = dy * inv * distance;
    double uz = dz * inv * distance;
    Point new_pi(pi[0]-ux, pi[1]-uy, pi[2]-uz);
    Point new_pj(pj[0]+ux, pj[1]+uy, pj[2]+uz);
    poly.set_point(i, new_pi);
    poly.set_point(j, new_pj);
    if (i == 0)        poly.set_point(n - 1, new_pi);
    if (j == n - 1)    poly.set_point(0,     new_pj);
}

// ───────────────────────────────────────────────────────────────────────────
// face_to_face_wood — main timber-joint topology detector
// ───────────────────────────────────────────────────────────────────────────
//
// Direct port of `wood::main::face_to_face` from
// `cmake/src/wood/include/wood_main.cpp`, with one critical departure:
//
//   * Every parameter that the original C++ reads from `wood::GLOBALS::*` is
//     hoisted into the function signature. There is NO global config and NO
//     WoodConfig struct — the caller passes each tunable explicitly. This
//     keeps the function pure and re-entrant.
//
// Conventions (kept identical to the C++ original):
//
//   * polylines_0[0] = top face, [1] = bottom face, [2..] = side faces.
//   * planes_0 has the same shape and ordering.
//   * Joint type code combines a geometric class with a refinement:
//        side-side, parallel out-of-plane → 11
//        side-side, parallel in-plane     → 12
//        side-side, rotated/perpendicular → 13
//        top-side                          → 20
//        top-top                           → 40
//
// Returns true if a valid joint was constructed; false otherwise. On success,
// `out_joint` is filled with the seven outputs of the original function.
//
// Wood-tunable parameters:
//   joint_volume_extension : per-joint extension parameters, packed in
//                            triples (width_ext, height_ext, line_ext). The
//                            function picks one triple by `joint_id`,
//                            clamping to the last available triple.
//   limit_min_joint_length : minimum joint length (linear). Joints whose
//                            alignment line — minus the line extension — is
//                            shorter than this are rejected.
//   distance_squared       : squared-distance threshold below which an
//                            alignment line is treated as degenerate.
//   dihedral_angle_threshold:
//                            cutoff (degrees) between out-of-plane (≤) and
//                            in-plane (>) parallel side-to-side joints.
//                            Wood default is 150°.
//   all_treated_as_rotated : force every side-to-side joint through the
//                            rotated branch (type 13).
//   rotated_joint_as_average:
//                            in the rotated branch, average both alignment
//                            lines (true) or use joint_line0 directly (false).
static bool face_to_face_wood(
    size_t joint_id,
    const std::vector<Polyline>& polylines_0,
    const std::vector<Polyline>& polylines_1,
    const std::vector<Plane>& planes_0,
    const std::vector<Plane>& planes_1,
    const std::vector<Vector>& insertion_vectors_0,
    const std::vector<Vector>& insertion_vectors_1,
    std::pair<int, int> el_ids_in,
    const std::vector<double>& joint_volume_extension,
    double limit_min_joint_length,
    double distance_squared,
    double dihedral_angle_threshold,
    bool all_treated_as_rotated,
    bool rotated_joint_as_average,
    WoodJoint& out_joint
) {
    // Pick which extension triple to use.
    // Original C++:
    //   extension_variables_count = floor(JOINT_VOLUME_EXTENSION.size() / 3.0) - 1
    //   extension_id = (count == 0) ? 0 : min(joint_id, count) * 3
    size_t triple_count = joint_volume_extension.size() / 3;
    size_t extension_variables_count = (triple_count == 0) ? 0 : (triple_count - 1);
    size_t extension_id = (extension_variables_count == 0)
        ? 0 : std::min(joint_id, extension_variables_count) * 3;
    auto ext = [&](size_t k) -> double {
        size_t idx = k + extension_id;
        return idx < joint_volume_extension.size() ? joint_volume_extension[idx] : 0.0;
    };
    double ext_w = ext(0); // edges 0,2 → joint width  scaling
    double ext_h = ext(1); // edges 1,3 → joint height scaling
    double ext_l = ext(2); // joint-line axial extension

    // Mutable copies that may be reordered by the male/female flip branch.
    std::pair<int, int> el_ids = el_ids_in;
    std::pair<std::array<int, 2>, std::array<int, 2>> face_ids = { {{0,0}}, {{0,0}} };

    // ── Outer loop over face pairs ─────────────────────────────────────────
    for (size_t i = 0; i < planes_0.size(); ++i) {
        for (size_t j = 0; j < planes_1.size(); ++j) {

            // 1. Coplanarity test (antiparallel-only — touching back-to-back).
            Point  o0 = planes_0[i].origin();
            Vector n0 = planes_0[i].z_axis();
            Point  o1 = planes_1[j].origin();
            Vector n1 = planes_1[j].z_axis();
            bool coplanar = Plane::is_coplanar(o0, n0, o1, n1, false, Tolerance::APPROXIMATION);
            if (!coplanar) continue;

            // 2. 2D Boolean intersection between the two coplanar faces.
            std::vector<Polyline> isect_results =
                Polyline::boolean_op(polylines_0[i], polylines_1[j], planes_0[i], 0);
            if (isect_results.empty()) continue;
            Polyline joint_area_open = isect_results.front();
            if (joint_area_open.point_count() < 3) continue;
            Polyline joint_area = joint_area_open.is_closed() ? joint_area_open
                                                              : joint_area_open.closed();

            // 3. Record matched face indices for the output.
            face_ids.first[0]  = static_cast<int>(i);
            face_ids.first[1]  = static_cast<int>(i);
            face_ids.second[0] = static_cast<int>(j);
            face_ids.second[1] = static_cast<int>(j);

            // 4. Joint type from face class.
            //    type0/type1 = 0 if face is a side, 1 if face is top/bottom.
            int type0 = (i > 1) ? 0 : 1;
            int type1 = (j > 1) ? 0 : 1;
            int joint_type = type0 + type1;

            // 5. Build the side-A alignment line (`joint_line0`) when face A
            //    is a side face. For top faces, this stays a degenerate
            //    sentinel — its length is later required to pass the
            //    LIMIT_MIN_JOINT_LENGTH check, so top-top naturally bypasses
            //    it via the `joint_type == 2` branch below.
            Line joint_line0 = Line::from_points(Point(0,0,0), Point(0,0,0));
            // Mid-thickness average plane of element 0.
            Point  avg_origin_0 = Point::mid_point(polylines_0[0].get_point(0),
                                                   polylines_0[1].get_point(0));
            Vector avg_normal_0 = planes_0[0].z_axis();
            Plane  avg_plane_0  = Plane::from_point_normal(avg_origin_0, avg_normal_0);
            Polyline joint_quads0(std::vector<Point>{});
            bool has_quads0 = false;
            if (i > 1) {
                Point a0 = polylines_0[0].get_point(i - 2);
                Point a1 = polylines_0[1].get_point(i - 2);
                Point b0 = polylines_0[0].get_point(i - 1);
                Point b1 = polylines_0[1].get_point(i - 1);
                Line alignment_segment = Line::from_points(Point::mid_point(a0, a1),
                                                           Point::mid_point(b0, b1));
                if (!polyline_plane_to_line(joint_area, avg_plane_0,
                                            alignment_segment.start(), joint_line0)) {
                    return false;
                }
                if (joint_line0.squared_length() <= distance_squared) return false;
                if (!get_quad_from_line_topbottomplanes(planes_0[i], joint_line0,
                                                        planes_0[0], planes_0[1],
                                                        joint_quads0)) {
                    return false;
                }
                has_quads0 = true;
            }

            // 6. Same for side-B alignment line (`joint_line1`).
            Line joint_line1 = Line::from_points(Point(0,0,0), Point(0,0,0));
            Point  avg_origin_1 = Point::mid_point(polylines_1[0].get_point(0),
                                                   polylines_1[1].get_point(0));
            Vector avg_normal_1 = planes_1[0].z_axis();
            Plane  avg_plane_1  = Plane::from_point_normal(avg_origin_1, avg_normal_1);
            Polyline joint_quads1(std::vector<Point>{});
            bool has_quads1 = false;
            if (j > 1) {
                Point a0 = polylines_1[0].get_point(j - 2);
                Point a1 = polylines_1[1].get_point(j - 2);
                Point b0 = polylines_1[0].get_point(j - 1);
                Point b1 = polylines_1[1].get_point(j - 1);
                Line alignment_segment = Line::from_points(Point::mid_point(a0, a1),
                                                           Point::mid_point(b0, b1));
                if (!polyline_plane_to_line(joint_area, avg_plane_1,
                                            alignment_segment.start(), joint_line1)) {
                    return false;
                }
                if (joint_line1.squared_length() <= distance_squared) return false;
                if (!get_quad_from_line_topbottomplanes(planes_1[j], joint_line1,
                                                        planes_1[0], planes_1[1],
                                                        joint_quads1)) {
                    return false;
                }
                has_quads1 = true;
            }

            // 7. Validate joint line lengths and apply axial extension.
            //    The C++ rejects when (ext_l*2)^2 > line.squared_length() - min^2,
            //    i.e. when extending the line by ext_l on each end would
            //    shrink the original line below the configured minimum.
            if (joint_type < 2) {
                double limit = (ext_l * 2.0) * (ext_l * 2.0);
                double minsq = limit_min_joint_length * limit_min_joint_length;
                if (i > 1 && limit > joint_line0.squared_length() - minsq) return false;
                if (j > 1 && limit > joint_line1.squared_length() - minsq) return false;
                extend_equally(joint_line0, ext_l);
                extend_equally(joint_line1, ext_l);
            }

            // 8. Optional insertion direction (the male takes priority).
            Vector dir(0, 0, 0);
            bool dir_set = false;
            if (!insertion_vectors_0.empty() && !insertion_vectors_1.empty()) {
                dir = (i > j) ? insertion_vectors_0[i] : insertion_vectors_1[j];
                dir_set = (std::abs(dir[0]) + std::abs(dir[1]) + std::abs(dir[2])) > 0.01;
            }

            // 9. Branch on joint type.
            std::array<Line, 2> joint_lines = {
                Line::from_points(Point(0,0,0), Point(0,0,0)),
                Line::from_points(Point(0,0,0), Point(0,0,0)),
            };
            std::array<std::optional<Polyline>, 4> joint_volumes{};

            if (joint_type == 0) {
                // ════════════════════════════════════════════════════════════
                // SIDE-SIDE
                // ════════════════════════════════════════════════════════════
                joint_lines[0] = joint_line0;
                joint_lines[1] = joint_line1;

                // Are the two side faces' alignment lines parallel?
                Vector v0(joint_line0.start()[0] - joint_line0.end()[0],
                          joint_line0.start()[1] - joint_line0.end()[1],
                          joint_line0.start()[2] - joint_line0.end()[2]);
                Vector v1(joint_line1.start()[0] - joint_line1.end()[0],
                          joint_line1.start()[1] - joint_line1.end()[1],
                          joint_line1.start()[2] - joint_line1.end()[2]);
                int parallel = v0.is_parallel_to(v1);

                if (parallel == 0 || all_treated_as_rotated) {
                    // ────────────────────────────────────────────────────────
                    // Rotated / perpendicular elements (type 13)
                    // ────────────────────────────────────────────────────────
                    // Build a single averaged segment between the two
                    // alignment lines, then construct a local 2D frame
                    // around it, project the joint area into 2D, take its
                    // axis-aligned bounding rectangle, and extrude it by
                    // ±half-thickness in 3D.

                    Line average_segment = Line::from_points(Point(0,0,0), Point(0,0,0));
                    {
                        double d_ss = Point::distance(joint_line0.start(),
                                                      joint_line1.start());
                        double d_se = Point::distance(joint_line0.start(),
                                                      joint_line1.end());
                        if (d_ss < d_se) {
                            average_segment = Line::from_points(
                                Point::mid_point(joint_line0.start(), joint_line1.start()),
                                Point::mid_point(joint_line0.end(),   joint_line1.end()));
                        } else {
                            average_segment = Line::from_points(
                                Point::mid_point(joint_line0.start(), joint_line1.end()),
                                Point::mid_point(joint_line0.end(),   joint_line1.start()));
                        }
                    }
                    Line axis_segment = rotated_joint_as_average ? average_segment
                                                                 : joint_line0;

                    // Local frame: x = along axis, z = face normal,
                    //              y = z × x (re-orthogonalized below).
                    Point  o = axis_segment.start();
                    Vector x = axis_segment.to_vector();
                    Vector z = planes_0[i].z_axis();
                    Vector y = z.cross(x);
                    y.normalize_self();

                    // Alternative branch from the C++ original (rare path):
                    if (!rotated_joint_as_average) {
                        y = planes_0[0].z_axis();
                        z = x.cross(y);
                    }

                    // Re-orient y by intersecting a thick test segment
                    // through the joint center with the two outer plates'
                    // top/bottom planes — this picks up the actual signed
                    // direction across the assembly.
                    Point center_pt = polylines_0[i].center();
                    double thick_a = std::max(
                        Point::distance(planes_0[0].origin(),
                                        planes_0[1].project(planes_0[0].origin())),
                        Point::distance(planes_1[0].origin(),
                                        planes_1[1].project(planes_1[0].origin())));
                    Vector y_scaled = y;
                    y_scaled = Vector(y_scaled[0] * thick_a * 2.0,
                                      y_scaled[1] * thick_a * 2.0,
                                      y_scaled[2] * thick_a * 2.0);
                    Line y_line = Line::from_points(
                        Point(center_pt[0]+y_scaled[0],
                              center_pt[1]+y_scaled[1],
                              center_pt[2]+y_scaled[2]),
                        Point(center_pt[0]-y_scaled[0],
                              center_pt[1]-y_scaled[1],
                              center_pt[2]-y_scaled[2]));
                    Line clipped;
                    if (Intersection::line_two_planes(y_line, planes_0[0],
                                                      planes_1[1], clipped)) {
                        y = Vector(clipped.end()[0]-clipped.start()[0],
                                   clipped.end()[1]-clipped.start()[1],
                                   clipped.end()[2]-clipped.start()[2]);
                    }
                    x = y.cross(z);

                    Xform xform = Xform::plane_to_xy(o, x, y, z);

                    // Project joint area into local 2D, grab AABB.
                    std::vector<Point> proj_pts = joint_area.get_points();
                    transform(proj_pts, xform);
                    if (proj_pts.empty()) return false;
                    double xmin = proj_pts[0][0], xmax = xmin;
                    double ymin = proj_pts[0][1], ymax = ymin;
                    for (size_t k = 1; k < proj_pts.size(); ++k) {
                        if      (proj_pts[k][0] < xmin) xmin = proj_pts[k][0];
                        else if (proj_pts[k][0] > xmax) xmax = proj_pts[k][0];
                        if      (proj_pts[k][1] < ymin) ymin = proj_pts[k][1];
                        else if (proj_pts[k][1] > ymax) ymax = proj_pts[k][1];
                    }
                    double zmin = proj_pts[0][2];
                    // Average rectangle in local 2D, vertices ordered to
                    // match the C++ original: { p0+x+y, p3, p1, p2 }.
                    std::vector<Point> rect_local = {
                        Point(xmax, ymax, zmin),
                        Point(xmin, ymax, zmin),
                        Point(xmin, ymin, zmin),
                        Point(xmax, ymin, zmin),
                    };
                    auto xform_inv_opt = xform.inverse();
                    if (!xform_inv_opt) return false;
                    transform(rect_local, *xform_inv_opt);

                    // Offset by element thickness along z (or insertion dir).
                    Vector offset_vector = dir_set ? dir : z;
                    offset_vector.normalize_self();
                    double d0 = 0.5 * Point::distance(
                        planes_0[0].origin(),
                        planes_0[1].project(planes_0[0].origin()));
                    offset_vector = Vector(offset_vector[0]*d0,
                                           offset_vector[1]*d0,
                                           offset_vector[2]*d0);

                    // Two extruded rectangles (closed quads, 5 points each).
                    Polyline vol0(std::vector<Point>{
                        Point(rect_local[3][0]+offset_vector[0], rect_local[3][1]+offset_vector[1], rect_local[3][2]+offset_vector[2]),
                        Point(rect_local[3][0]-offset_vector[0], rect_local[3][1]-offset_vector[1], rect_local[3][2]-offset_vector[2]),
                        Point(rect_local[0][0]-offset_vector[0], rect_local[0][1]-offset_vector[1], rect_local[0][2]-offset_vector[2]),
                        Point(rect_local[0][0]+offset_vector[0], rect_local[0][1]+offset_vector[1], rect_local[0][2]+offset_vector[2]),
                        Point(rect_local[3][0]+offset_vector[0], rect_local[3][1]+offset_vector[1], rect_local[3][2]+offset_vector[2]),
                    });
                    Polyline vol1(std::vector<Point>{
                        Point(rect_local[2][0]+offset_vector[0], rect_local[2][1]+offset_vector[1], rect_local[2][2]+offset_vector[2]),
                        Point(rect_local[2][0]-offset_vector[0], rect_local[2][1]-offset_vector[1], rect_local[2][2]-offset_vector[2]),
                        Point(rect_local[1][0]-offset_vector[0], rect_local[1][1]-offset_vector[1], rect_local[1][2]-offset_vector[2]),
                        Point(rect_local[1][0]+offset_vector[0], rect_local[1][1]+offset_vector[1], rect_local[1][2]+offset_vector[2]),
                        Point(rect_local[2][0]+offset_vector[0], rect_local[2][1]+offset_vector[1], rect_local[2][2]+offset_vector[2]),
                    });

                    // Apply joint width/height extensions.
                    extend_polyline_edge_equally(vol0, 0, ext_w);
                    extend_polyline_edge_equally(vol0, 2, ext_w);
                    extend_polyline_edge_equally(vol1, 0, ext_w);
                    extend_polyline_edge_equally(vol1, 2, ext_w);
                    extend_polyline_edge_equally(vol0, 1, ext_h);
                    extend_polyline_edge_equally(vol0, 3, ext_h);
                    extend_polyline_edge_equally(vol1, 1, ext_h);
                    extend_polyline_edge_equally(vol1, 3, ext_h);

                    joint_volumes[0] = vol0;
                    joint_volumes[1] = vol1;
                    joint_type = 13;

                    out_joint.el_ids       = el_ids;
                    out_joint.face_ids     = face_ids;
                    out_joint.joint_type   = joint_type;
                    out_joint.joint_area   = joint_area;
                    out_joint.joint_lines  = joint_lines;
                    out_joint.joint_volumes_pair_a_pair_b = joint_volumes;
                    return true;
                } else {
                    // ────────────────────────────────────────────────────────
                    // Parallel elements: split on dihedral angle
                    // ────────────────────────────────────────────────────────
                    Line lj;
                    line_line_overlap_average(joint_line0, joint_line1, lj);
                    joint_lines[0] = lj;
                    joint_lines[1] = lj;

                    // End-cap planes along the joint axis.
                    Vector lj_v = lj.to_vector();
                    Point  lj_s = lj.start();
                    Point  lj_e = lj.end();
                    Plane pl_end0 = Plane::from_point_normal(lj_s, lj_v);
                    if (dir_set) {
                        Vector dir_copy = dir;
                        Point  lj_s_copy = lj_s;
                        pl_end0 = Plane::from_point_normal(lj_s_copy, dir_copy);
                    }
                    Vector pl_end0_n = pl_end0.z_axis();
                    Plane pl_end1 = Plane::from_point_normal(lj_e, pl_end0_n);

                    // Dihedral angle of the joint edge in the tetrahedron
                    // (lj.start, lj.end, center0, center1).
                    Point center0 = avg_plane_0.project(polylines_0[0].center());
                    Point center1 = avg_plane_1.project(polylines_1[0].center());
                    double dihedral = approximate_dihedral_angle_deg(
                        lj.start(), lj.end(), center0, center1);

                    if (dihedral < 20.0) return false;

                    if (dihedral <= dihedral_angle_threshold) {
                        // ── Out-of-plane parallel (type 11) ─────────────
                        // Probe the joint axis 90° (in the face plane) to
                        // figure out which adjacent element planes are
                        // closer, then build an "open" plane×4-plane
                        // intersection to get a quad.

                        Vector connection_normal = planes_0[i].z_axis();
                        Vector lj_normal = lj.to_vector();
                        Vector lj_v_90_full = lj_normal.cross(connection_normal);
                        Vector lj_v_90(lj_v_90_full[0]*0.5,
                                       lj_v_90_full[1]*0.5,
                                       lj_v_90_full[2]*0.5);
                        Line lj_l_90 = Line::from_points(
                            lj.start(),
                            Point(lj.start()[0]+lj_v_90[0],
                                  lj.start()[1]+lj_v_90[1],
                                  lj.start()[2]+lj_v_90[2]));
                        Point pl0_0_p, pl1_0_p, pl1_1_p;
                        if (!Intersection::line_plane(lj_l_90, planes_0[0], pl0_0_p, false)) return false;
                        if (!Intersection::line_plane(lj_l_90, planes_1[0], pl1_0_p, false)) return false;
                        if (!Intersection::line_plane(lj_l_90, planes_1[1], pl1_1_p, false)) return false;

                        double d_to_pl1_0 = Point::distance(pl0_0_p, pl1_0_p);
                        double d_to_pl1_1 = Point::distance(pl0_0_p, pl1_1_p);
                        bool larger_to_pl1_0 = d_to_pl1_0 > d_to_pl1_1;
                        std::array<Plane, 4> planes4;
                        if (larger_to_pl1_0) {
                            planes4 = { planes_1[1], planes_0[0], planes_1[0], planes_0[1] };
                        } else {
                            planes4 = { planes_1[0], planes_0[0], planes_1[1], planes_0[1] };
                        }

                        Polyline vol0(std::vector<Point>{});
                        Polyline vol1(std::vector<Point>{});
                        if (!Intersection::plane_4planes_open(pl_end0, planes4, vol0)) return false;
                        if (!Intersection::plane_4planes_open(pl_end1, planes4, vol1)) return false;

                        // Consistent volume orientation: rotate by 2 if
                        // vertex 1 is not on the negative side of plane[i].
                        bool need_rotate = !plane_has_on_negative_side(planes_0[i], vol0.get_point(1));
                        if (need_rotate) {
                            std::vector<Point> pts0 = vol0.get_points();
                            std::vector<Point> pts1 = vol1.get_points();
                            std::rotate(pts0.begin(), pts0.begin() + 2, pts0.end());
                            std::rotate(pts1.begin(), pts1.begin() + 2, pts1.end());
                            vol0 = Polyline(pts0);
                            vol1 = Polyline(pts1);
                        }

                        // Reverse + rotate(3) — the male/female flip from the
                        // wood C++ original. We also swap el_ids and reverse
                        // joint_lines so the joint library always sees the
                        // male element first.
                        {
                            std::vector<Point> pts0 = vol0.get_points();
                            std::vector<Point> pts1 = vol1.get_points();
                            std::reverse(pts0.begin(), pts0.end());
                            std::reverse(pts1.begin(), pts1.end());
                            std::rotate(pts0.begin(), pts0.begin() + 3, pts0.end());
                            std::rotate(pts1.begin(), pts1.begin() + 3, pts1.end());
                            vol0 = Polyline(pts0);
                            vol1 = Polyline(pts1);
                        }
                        std::swap(el_ids.first, el_ids.second);
                        std::swap(face_ids.first, face_ids.second);
                        std::swap(joint_lines[0], joint_lines[1]);

                        // Close the rectangles (append the first vertex).
                        {
                            std::vector<Point> pts0 = vol0.get_points();
                            std::vector<Point> pts1 = vol1.get_points();
                            pts0.push_back(pts0.front());
                            pts1.push_back(pts1.front());
                            vol0 = Polyline(pts0);
                            vol1 = Polyline(pts1);
                        }

                        extend_polyline_edge_equally(vol0, 0, ext_w);
                        extend_polyline_edge_equally(vol0, 2, ext_w);
                        extend_polyline_edge_equally(vol1, 0, ext_w);
                        extend_polyline_edge_equally(vol1, 2, ext_w);
                        extend_polyline_edge_equally(vol0, 1, ext_h);
                        extend_polyline_edge_equally(vol0, 3, ext_h);
                        extend_polyline_edge_equally(vol1, 1, ext_h);
                        extend_polyline_edge_equally(vol1, 3, ext_h);

                        joint_volumes[0] = vol0;
                        joint_volumes[1] = vol1;
                        joint_type = 11;

                        out_joint.el_ids       = el_ids;
                        out_joint.face_ids     = face_ids;
                        out_joint.joint_type   = joint_type;
                        out_joint.joint_area   = joint_area;
                        out_joint.joint_lines  = joint_lines;
                        out_joint.joint_volumes_pair_a_pair_b = joint_volumes;
                        return true;
                    } else {
                        // ── In-plane parallel (type 12) ────────────────
                        // Compute two planes offset from the matched face
                        // plane by ±half the element thickness, then form
                        // two 4-plane loops (one per element) and intersect
                        // each with the two end planes → 4 joint volumes.

                        double d0 = 0.5 * Point::distance(
                            planes_0[0].origin(),
                            planes_0[1].project(planes_0[0].origin()));
                        Plane offset_plane_0 = planes_0[i].translate_by_normal(-d0);
                        Plane offset_plane_1 = planes_0[i].translate_by_normal( d0);

                        // Winding fix: if plane1[0] is farther from
                        // plane0[0] than plane1[1] is, swap so the loop
                        // goes around the joint consistently.
                        Point pt00   = planes_0[0].origin();
                        Point proj00 = planes_1[0].project(pt00);
                        Point proj01 = planes_1[1].project(pt00);
                        double w0 = Point::distance(pt00, proj00);
                        double w1 = Point::distance(pt00, proj01);
                        Plane p1_0 = (w0 > w1) ? planes_1[1] : planes_1[0];
                        Plane p1_1 = (w0 > w1) ? planes_1[0] : planes_1[1];

                        std::array<Plane, 4> loop_planes_0 = {
                            offset_plane_0, planes_0[0], offset_plane_1, planes_0[1]
                        };
                        std::array<Plane, 4> loop_planes_1 = {
                            offset_plane_0, p1_0,        offset_plane_1, p1_1
                        };

                        Polyline vol0(std::vector<Point>{});
                        Polyline vol1(std::vector<Point>{});
                        Polyline vol2(std::vector<Point>{});
                        Polyline vol3(std::vector<Point>{});
                        if (!Intersection::plane_4planes(pl_end0, loop_planes_0, vol0)) return false;
                        if (!Intersection::plane_4planes(pl_end1, loop_planes_0, vol1)) return false;
                        if (!Intersection::plane_4planes(pl_end0, loop_planes_1, vol2)) return false;
                        if (!Intersection::plane_4planes(pl_end1, loop_planes_1, vol3)) return false;

                        for (Polyline* vp : {&vol0, &vol1, &vol2, &vol3}) {
                            extend_polyline_edge_equally(*vp, 0, ext_w);
                            extend_polyline_edge_equally(*vp, 2, ext_w);
                            extend_polyline_edge_equally(*vp, 1, ext_h);
                            extend_polyline_edge_equally(*vp, 3, ext_h);
                        }

                        joint_volumes[0] = vol0;
                        joint_volumes[1] = vol1;
                        joint_volumes[2] = vol2;
                        joint_volumes[3] = vol3;
                        joint_type = 12;

                        out_joint.el_ids       = el_ids;
                        out_joint.face_ids     = face_ids;
                        out_joint.joint_type   = joint_type;
                        out_joint.joint_area   = joint_area;
                        out_joint.joint_lines  = joint_lines;
                        out_joint.joint_volumes_pair_a_pair_b = joint_volumes;
                        return true;
                    }
                }
            } else if (joint_type == 1) {
                // ════════════════════════════════════════════════════════════
                // TOP-SIDE (type 20)
                // ════════════════════════════════════════════════════════════
                // The element with the higher face index is the male (its
                // side face defines the joint axis). The female is the other
                // element's top/bottom face. The joint volume is built by
                // extruding the male's side-face quad along an offset vector
                // that spans the female's thickness.

                bool male_or_female = i > j; // true → male = element 0
                Line jline = male_or_female ? joint_line0 : joint_line1;
                joint_lines[0] = jline;
                joint_lines[1] = jline;

                Plane plane0_0 = male_or_female ? planes_0[0] : planes_1[0];
                Plane plane1_0 = !male_or_female ? planes_0[i] : planes_1[j];
                size_t other_idx = !male_or_female
                    ? (i == 0 ? 1 : 0)
                    : (j == 0 ? 1 : 0);
                Plane plane1_1 = !male_or_female ? planes_0[other_idx] : planes_1[other_idx];

                bool quad_available = male_or_female ? has_quads0 : has_quads1;
                if (!quad_available) return false;
                Polyline quad_0 = male_or_female ? joint_quads0 : joint_quads1;

                Vector offset_vector;
                if (!get_orthogonal_vector_between_two_plane_pairs(
                        plane0_0, plane1_0, plane1_1, offset_vector)) {
                    return false;
                }
                if (dir_set) {
                    Vector scaled;
                    if (Intersection::scale_vector_to_distance_of_2planes(
                            dir, plane1_0, plane1_1, scaled)) {
                        offset_vector = scaled;
                    }
                }

                if (!male_or_female) {
                    std::swap(el_ids.first, el_ids.second);
                    std::swap(face_ids.first, face_ids.second);
                }

                size_t m_id = male_or_female ? 0 : 1;
                size_t f_id = male_or_female ? 1 : 0;
                Point q0 = quad_0.get_point(0);
                Point q1 = quad_0.get_point(1);
                Point q2 = quad_0.get_point(2);
                Point q3 = quad_0.get_point(3);

                Polyline male_vol(std::vector<Point>{
                    q0, q1,
                    Point(q1[0]+offset_vector[0], q1[1]+offset_vector[1], q1[2]+offset_vector[2]),
                    Point(q0[0]+offset_vector[0], q0[1]+offset_vector[1], q0[2]+offset_vector[2]),
                    q0,
                });
                Polyline female_vol(std::vector<Point>{
                    q3, q2,
                    Point(q2[0]+offset_vector[0], q2[1]+offset_vector[1], q2[2]+offset_vector[2]),
                    Point(q3[0]+offset_vector[0], q3[1]+offset_vector[1], q3[2]+offset_vector[2]),
                    q3,
                });

                extend_polyline_edge_equally(male_vol,   0, ext_w);
                extend_polyline_edge_equally(male_vol,   2, ext_w);
                extend_polyline_edge_equally(female_vol, 0, ext_w);
                extend_polyline_edge_equally(female_vol, 2, ext_w);
                extend_polyline_edge_equally(male_vol,   1, ext_h);
                extend_polyline_edge_equally(male_vol,   3, ext_h);
                extend_polyline_edge_equally(female_vol, 1, ext_h);
                extend_polyline_edge_equally(female_vol, 3, ext_h);

                joint_volumes[m_id] = male_vol;
                joint_volumes[f_id] = female_vol;
                joint_type = 20;

                out_joint.el_ids       = el_ids;
                out_joint.face_ids     = face_ids;
                out_joint.joint_type   = joint_type;
                out_joint.joint_area   = joint_area;
                out_joint.joint_lines  = joint_lines;
                out_joint.joint_volumes_pair_a_pair_b = joint_volumes;
                return true;
            } else {
                // ════════════════════════════════════════════════════════════
                // TOP-TOP (type 40)
                // ════════════════════════════════════════════════════════════
                // Build the bounding rectangle of the joint area in the
                // shared plane, then translate it ±thickness along each
                // element's normal to form two extruded slabs. The four
                // corners are reorganised into two rectangles matching the
                // wood::joint_lib convention.

                auto rect_opt = Polyline::bounding_rectangle(joint_area);
                if (!rect_opt) return false;
                Polyline vol_a = *rect_opt;
                Polyline vol_b = *rect_opt;

                // Movement direction (insertion vector if available, else
                // the face normal). The original C++ flips the sign twice,
                // leaving dir0 in its original direction and dir1 = -dir0.
                Vector dir0 = dir_set
                    ? (i < insertion_vectors_0.size() ? insertion_vectors_0[i]
                                                      : planes_0[i].z_axis())
                    : planes_0[i].z_axis();
                dir0.normalize_self();
                Vector dir1_pre(-dir0[0], -dir0[1], -dir0[2]);
                dir0 = Vector(-dir0[0], -dir0[1], -dir0[2]);
                Vector dir1(-dir1_pre[0], -dir1_pre[1], -dir1_pre[2]);

                // Element thicknesses across the matched face.
                int next_plane_0 = (i == 0) ? 1 : 0;
                int next_plane_1 = (j == 0) ? 1 : 0;
                double dist_0 = Point::distance(
                    planes_0[i].origin(),
                    planes_0[next_plane_0].project(planes_0[i].origin()));
                double dist_1 = Point::distance(
                    planes_1[j].origin(),
                    planes_1[next_plane_1].project(planes_1[j].origin()));
                dir0 = Vector(dir0[0]*dist_0, dir0[1]*dist_0, dir0[2]*dist_0);
                dir1 = Vector(dir1[0]*dist_1, dir1[1]*dist_1, dir1[2]*dist_1);

                // Translate the rectangles.
                for (size_t k = 0; k < vol_a.point_count(); ++k) {
                    Point p = vol_a.get_point(k);
                    vol_a.set_point(k, Point(p[0]+dir0[0], p[1]+dir0[1], p[2]+dir0[2]));
                }
                for (size_t k = 0; k < vol_b.point_count(); ++k) {
                    Point p = vol_b.get_point(k);
                    vol_b.set_point(k, Point(p[0]+dir1[0], p[1]+dir1[1], p[2]+dir1[2]));
                }

                // Reformat into the two-rectangle convention used by the
                // joint library:
                //   temp0 = (a[0], a[1], b[1], b[0], a[0])
                //   temp1 = (a[3], a[2], b[2], b[3], a[3])
                Point a0 = vol_a.get_point(0);
                Point a1 = vol_a.get_point(1);
                Point a2 = vol_a.get_point(2);
                Point a3 = vol_a.get_point(3);
                Point b0 = vol_b.get_point(0);
                Point b1 = vol_b.get_point(1);
                Point b2 = vol_b.get_point(2);
                Point b3 = vol_b.get_point(3);

                Polyline temp0(std::vector<Point>{a0, a1, b1, b0, a0});
                Polyline temp1(std::vector<Point>{a3, a2, b2, b3, a3});

                extend_polyline_edge_equally(temp0, 0, ext_w);
                extend_polyline_edge_equally(temp0, 2, ext_w);
                extend_polyline_edge_equally(temp1, 0, ext_w);
                extend_polyline_edge_equally(temp1, 2, ext_w);
                extend_polyline_edge_equally(temp0, 1, ext_h);
                extend_polyline_edge_equally(temp0, 3, ext_h);
                extend_polyline_edge_equally(temp1, 1, ext_h);
                extend_polyline_edge_equally(temp1, 3, ext_h);

                joint_volumes[0] = temp0;
                joint_volumes[1] = temp1;
                joint_type = 40;

                out_joint.el_ids       = el_ids;
                out_joint.face_ids     = face_ids;
                out_joint.joint_type   = joint_type;
                out_joint.joint_area   = joint_area;
                out_joint.joint_lines  = joint_lines;
                out_joint.joint_volumes_pair_a_pair_b = joint_volumes;
                return true;
            }
        }
    }
    return false;
}

} // anonymous namespace

int main() {
    using Clock = std::chrono::high_resolution_clock;
    auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
    auto t0 = Clock::now();

    // 1. Load polylines and pair top/bottom faces (same OBJ as main_1.cpp).
    auto polylines = obj::read_obj_polylines(
        (base / "session_data" / "annen_polylines.obj").string());
    auto pairs = obj::pair_polylines(polylines);

    // 2. Build plate elements.
    Session session("WoodComplete");
    auto g = session.add_group("Elements");
    std::vector<std::shared_ptr<PlateElement>> plates;
    plates.reserve(pairs.size());
    for (auto [a, b] : pairs) {
        auto plate = std::make_shared<PlateElement>(
            polylines[a], polylines[b], "plate_" + std::to_string(a));
        session.add_element(plate, g);
        plates.push_back(plate);
    }
    auto t1 = Clock::now();

    // 3. BVH broad-phase: find candidate adjacent pairs.
    size_t n = plates.size();
    std::vector<AABB> aabbs(n);
    for (size_t k = 0; k < n; ++k) {
        aabbs[k] = plates[k]->compute_aabb_fast(5.0);
    }
    double world_size = 0.0;
    for (const auto& a : aabbs) {
        world_size = std::max(world_size, std::abs(a.cx + a.hx));
        world_size = std::max(world_size, std::abs(a.cy + a.hy));
        world_size = std::max(world_size, std::abs(a.cz + a.hz));
        world_size = std::max(world_size, std::abs(a.cx - a.hx));
        world_size = std::max(world_size, std::abs(a.cy - a.hy));
        world_size = std::max(world_size, std::abs(a.cz - a.hz));
    }
    BVH bvh;
    bvh.build_from_aabbs(aabbs.data(), n, world_size * 2);
    std::vector<std::pair<int, int>> adjacency_pairs;
    for (size_t i = 0; i < n; ++i) {
        auto hits = bvh.query_aabb(aabbs[i]);
        for (int j : hits) {
            if (static_cast<int>(i) < j) {
                adjacency_pairs.emplace_back(static_cast<int>(i), j);
            }
        }
    }
    auto t2 = Clock::now();

    // 4. Define ALL wood-joint detection parameters as LOCAL variables.
    //    No globals, no config struct — every tunable is right here so the
    //    caller can see what knobs the algorithm exposes. The names match
    //    the original wood::GLOBALS::* fields one-for-one.
    std::vector<double> joint_volume_extension = {
        // (width_ext, height_ext, line_ext) triple, applied to every joint.
        // Width and height grow each joint volume rectangle in its plane;
        // line_ext grows the joint axis at both ends. Negative values shrink.
        0.0,   // width  extension (edges 0,2)
        0.0,   // height extension (edges 1,3)
        0.0,   // joint  axis extension (extend_equally on the alignment line)
    };
    double  limit_min_joint_length      = 0.0;
    double  distance_squared            = 1e-6;
    double  dihedral_angle_threshold    = 150.0;
    bool    all_treated_as_rotated      = false;
    bool    rotated_joint_as_average    = true;

    // 5. Iterate every adjacent pair and run the wood joint detector.
    int counts[5] = {0, 0, 0, 0, 0}; // [11, 12, 13, 20, 40]
    int n_failed  = 0;
    int n_success = 0;
    for (size_t k = 0; k < adjacency_pairs.size(); ++k) {
        int ia = adjacency_pairs[k].first;
        int ib = adjacency_pairs[k].second;

        std::vector<Polyline> polys_a  = plates[ia]->polylines();
        std::vector<Polyline> polys_b  = plates[ib]->polylines();
        std::vector<Plane>    planes_a = plates[ia]->planes();
        std::vector<Plane>    planes_b = plates[ib]->planes();

        WoodJoint joint;
        bool ok = face_to_face_wood(
            /*joint_id*/                 k,
            /*polylines_0*/              polys_a,
            /*polylines_1*/              polys_b,
            /*planes_0*/                 planes_a,
            /*planes_1*/                 planes_b,
            /*insertion_vectors_0*/      std::vector<Vector>{},
            /*insertion_vectors_1*/      std::vector<Vector>{},
            /*el_ids_in*/                std::pair<int, int>(ia, ib),
            /*joint_volume_extension*/   joint_volume_extension,
            /*limit_min_joint_length*/   limit_min_joint_length,
            /*distance_squared*/         distance_squared,
            /*dihedral_angle_threshold*/ dihedral_angle_threshold,
            /*all_treated_as_rotated*/   all_treated_as_rotated,
            /*rotated_joint_as_average*/ rotated_joint_as_average,
            /*out_joint*/                joint);
        if (!ok) {
            ++n_failed;
            continue;
        }
        ++n_success;
        switch (joint.joint_type) {
            case 11: ++counts[0]; break;
            case 12: ++counts[1]; break;
            case 13: ++counts[2]; break;
            case 20: ++counts[3]; break;
            case 40: ++counts[4]; break;
            default: break;
        }
    }
    auto t3 = Clock::now();

    auto ms = [](auto a, auto b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    fmt::print("{} polylines -> {} elements -> {} adjacency pairs\n",
               polylines.size(), pairs.size(), adjacency_pairs.size());
    fmt::print("  joints: {} success / {} failed\n", n_success, n_failed);
    fmt::print("  by type: 11={} 12={} 13={} 20={} 40={}\n",
               counts[0], counts[1], counts[2], counts[3], counts[4]);
    fmt::print("  import+pair: {:.0f}ms  bvh: {:.0f}ms  wood_f2f: {:.0f}ms  total: {:.0f}ms\n",
               ms(t0, t1), ms(t1, t2), ms(t2, t3), ms(t0, t3));
    return 0;
}
