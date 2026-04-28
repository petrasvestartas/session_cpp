// wood/wood_face_to_face.cpp — plate face-to-face joint detection.
// Implementation of face_to_face_wood declared in wood_face_to_face.h.
// No file I/O: operates purely on geometry data passed by the caller.
// (Stage 1 — building a WoodElement from a bottom/top polyline pair — lives
//  on the WoodElement constructor in wood_element.cpp.)
#include "wood_face_to_face.h"
#include "wood_element.h"
#include "wood_session.h"
#include "../src/intersection.h"
#include "../src/polyline.h"
#include "../src/line.h"
#include "../src/vector.h"
#include "../src/point.h"
#include "../src/xform.h"
#include "../src/plane.h"
#include "../src/tolerance.h"
#include <cmath>
#include <algorithm>
#include <utility>
#include <vector>
#include <array>
#include <optional>
#include <fmt/core.h>

using namespace session_cpp;
using wood_session::WoodJoint;
using wood_session::WoodElement;

// Geometry helpers two_rect_from_point_vector_and_zaxis and
// line_line_intersection_with_properties have moved to the session kernel:
//   polyline_two_rects_from_frame   → src/polyline.h (src/polyline.cpp)
//   Intersection::line_line_classified → src/intersection.h (src/intersection.cpp)
// Call sites below use the kernel versions directly.

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
//   * el0.polylines[0] = top face, [1] = bottom face, [2..] = side faces.
//   * el0.planes has the same shape and ordering.
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


bool face_to_face_wood(
    size_t joint_id,
    const WoodElement& el0,
    const WoodElement& el1,
    std::pair<int, int> el_ids_in,
    const std::vector<double>& joint_volume_extension,
    double limit_min_joint_length,
    double distance_squared,
    double coplanar_tolerance,
    double dihedral_angle_threshold,
    bool all_treated_as_rotated,
    bool rotated_joint_as_average,
    int  search_type,
    WoodJoint& out_joint,
    bool& out_swap_planes_1
) {
    out_swap_planes_1 = false;
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
    int dbg_coplanar = 0, dbg_boolean = 0;
    std::string dbg_fail_reason;
    if (search_type != 1) {
    for (size_t i = 0; i < el0.planes.size(); ++i) {
        for (size_t j = 0; j < el1.planes.size(); ++j) {

            // 1. Coplanarity test (antiparallel-only — touching back-to-back).
            Point  o0 = el0.planes[i].origin();
            Vector n0 = el0.planes[i].z_axis();
            Point  o1 = el1.planes[j].origin();
            Vector n1 = el1.planes[j].z_axis();
            // Coplanarity check matching wood's cgal::plane_util::is_coplanar:
            // 1. Check anti-parallel (is_parallel_to == -1)
            // 2. Check squared_distance(projection, point) < DISTANCE_SQUARED
            // Wood uses ANGLE = 0.11 RADIANS (cos ≈ 0.9940, ~6.3°), not 0.11 degrees.
            // Session's Vector::is_parallel_to uses ANGLE_TOLERANCE_DEGREES=0.11° which
            // is far too strict and misses ts_e_p connections in one_layer/full datasets.
            {
                double n0n1 = n0[0]*n1[0]+n0[1]*n1[1]+n0[2]*n1[2];
                double ll = std::sqrt((n0[0]*n0[0]+n0[1]*n0[1]+n0[2]*n0[2])*(n1[0]*n1[0]+n1[1]*n1[1]+n1[2]*n1[2]));
                if (ll <= 0.0 || n0n1/ll > -std::cos(wood_session::globals::ANGLE)) continue; // not antiparallel
            }
            // Projection-based distance (invariant to normal magnitude):
            double mag0_sq = n0[0]*n0[0]+n0[1]*n0[1]+n0[2]*n0[2];
            double mag1_sq = n1[0]*n1[0]+n1[1]*n1[1]+n1[2]*n1[2];
            double dot0 = n0[0]*(o1[0]-o0[0])+n0[1]*(o1[1]-o0[1])+n0[2]*(o1[2]-o0[2]);
            double dot1 = n1[0]*(o0[0]-o1[0])+n1[1]*(o0[1]-o1[1])+n1[2]*(o0[2]-o1[2]);
            double sq_dist0 = (mag0_sq > 1e-20) ? (dot0*dot0/mag0_sq) : 1e30;
            double sq_dist1 = (mag1_sq > 1e-20) ? (dot1*dot1/mag1_sq) : 1e30;
            bool coplanar = (sq_dist0 < coplanar_tolerance) && (sq_dist1 < coplanar_tolerance);
            if (!coplanar) continue;
            dbg_coplanar++;

            // 2. 2D Boolean intersection using the kernel's plane-frame
            //    Vatti wrapper. Wood accepts 3-vertex triangles only for
            //    top/bottom face pairs (i<2 && j<2); side-face pairs reject
            //    them. collapse_eps=1/1024mm removes Vatti FP duplicates on
            //    near-coincident edges (hexbox-family datasets).
            Polyline joint_area(std::vector<Point>{});
            bool include_triangles = (i < 2 && j < 2);
            if (!Intersection::polyline_boolean_2d_in_plane(
                    el0.polylines[i], el1.polylines[j], el0.planes[i],
                    joint_area, 0, include_triangles, 0.01, 1.0/1024.0)) {
                dbg_fail_reason = fmt::format("bool_empty f({},{})", i, j);
                continue;
            }
            dbg_boolean++;
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
            Point  avg_origin_0 = Point::mid_point(el0.polylines[0].get_point(0),
                                                   el0.polylines[1].get_point(0));
            Vector avg_normal_0 = el0.planes[0].z_axis();
            Plane  avg_plane_0  = Plane::from_point_normal(avg_origin_0, avg_normal_0);
            Polyline joint_quads0(std::vector<Point>{});
            bool has_quads0 = false;
            if (i > 1) {
                Point a0 = el0.polylines[0].get_point(i - 2);
                Point a1 = el0.polylines[1].get_point(i - 2);
                Point b0 = el0.polylines[0].get_point(i - 1);
                Point b1 = el0.polylines[1].get_point(i - 1);
                Line alignment_segment = Line::from_points(Point::mid_point(a0, a1),
                                                           Point::mid_point(b0, b1));
                if (!Intersection::polyline_plane_to_line(joint_area, avg_plane_0,
                                            alignment_segment.start(), joint_line0)) {
                    dbg_fail_reason = fmt::format("ppl0_fail f({},{})", i, j); continue;
                }
                if (joint_line0.squared_length() <= distance_squared) { dbg_fail_reason = fmt::format("jl0_short f({},{})", i, j); continue; }
                if (!Intersection::quad_from_line_top_bottom_planes(el0.planes[i], joint_line0,
                                                        el0.planes[0], el0.planes[1],
                                                        joint_quads0)) {
                    dbg_fail_reason = fmt::format("quad0_fail f({},{})", i, j); continue;
                }
                has_quads0 = true;
            }

            // 6. Same for side-B alignment line (`joint_line1`).
            Line joint_line1 = Line::from_points(Point(0,0,0), Point(0,0,0));
            Point  avg_origin_1 = Point::mid_point(el1.polylines[0].get_point(0),
                                                   el1.polylines[1].get_point(0));
            Vector avg_normal_1 = el1.planes[0].z_axis();
            Plane  avg_plane_1  = Plane::from_point_normal(avg_origin_1, avg_normal_1);
            Polyline joint_quads1(std::vector<Point>{});
            bool has_quads1 = false;
            if (j > 1) {
                Point a0 = el1.polylines[0].get_point(j - 2);
                Point a1 = el1.polylines[1].get_point(j - 2);
                Point b0 = el1.polylines[0].get_point(j - 1);
                Point b1 = el1.polylines[1].get_point(j - 1);
                Line alignment_segment = Line::from_points(Point::mid_point(a0, a1),
                                                           Point::mid_point(b0, b1));
                if (!Intersection::polyline_plane_to_line(joint_area, avg_plane_1,
                                            alignment_segment.start(), joint_line1)) {
                    dbg_fail_reason = fmt::format("ppl1_fail f({},{})", i, j); continue;
                }
                if (joint_line1.squared_length() <= distance_squared) { dbg_fail_reason = fmt::format("jl1_short f({},{})", i, j); continue; }
                if (!Intersection::quad_from_line_top_bottom_planes(el1.planes[j], joint_line1,
                                                        el1.planes[0], el1.planes[1],
                                                        joint_quads1)) {
                    dbg_fail_reason = fmt::format("quad1_fail f({},{})", i, j); continue;
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
                if (i > 1 && limit > joint_line0.squared_length() - minsq) { dbg_fail_reason = fmt::format("jl0_ext f({},{})", i, j); continue; }
                if (j > 1 && limit > joint_line1.squared_length() - minsq) { dbg_fail_reason = fmt::format("jl1_ext f({},{})", i, j); continue; }
                extend_equally(joint_line0, ext_l);
                extend_equally(joint_line1, ext_l);
            }

            // 8. Optional insertion direction (the male takes priority).
            Vector dir(0, 0, 0);
            bool dir_set = false;
            if (i < el0.insertion_vectors.size() && j < el1.insertion_vectors.size()) {
                dir = (i > j) ? el0.insertion_vectors[i] : el1.insertion_vectors[j];
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
                // Wood uses ANGLE=0.11 radians (~6.3°) not degrees.
                Vector v0(joint_line0.start()[0] - joint_line0.end()[0],
                          joint_line0.start()[1] - joint_line0.end()[1],
                          joint_line0.start()[2] - joint_line0.end()[2]);
                Vector v1(joint_line1.start()[0] - joint_line1.end()[0],
                          joint_line1.start()[1] - joint_line1.end()[1],
                          joint_line1.start()[2] - joint_line1.end()[2]);
                int parallel;
                {
                    const double wood_cos_tol = std::cos(wood_session::globals::ANGLE);
                    double ll_ = v0.magnitude() * v1.magnitude();
                    if (ll_ <= 0.0) { parallel = 0; }
                    else {
                        double ca = (v0[0]*v1[0] + v0[1]*v1[1] + v0[2]*v1[2]) / ll_;
                        if      (ca >=  wood_cos_tol) parallel =  1;
                        else if (ca <= -wood_cos_tol) parallel = -1;
                        else                          parallel =  0;
                    }
                }

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
                    // Wood has a BUG at wood_main.cpp:697-698: the
                    // `if (!FLAG) IK::Segment_3 average_segment = joint_line0`
                    // declares a NEW local variable that shadows and is
                    // immediately discarded. So wood ALWAYS uses the
                    // calculated `average_segment`, regardless of the flag.
                    // Mirror exactly — do NOT substitute joint_line0.
                    Line axis_segment = average_segment;

                    // Local frame: x = along axis, z = face normal, y = x × z.
                    // Wood: `IK::Vector_3 y = CGAL::cross_product (x, z)` —
                    // must match sign exactly (z × x is opposite direction
                    // and produces a mirrored jv rectangle).
                    Point  o = axis_segment.start();
                    Vector x = axis_segment.to_vector();
                    Vector z = el0.planes[i].z_axis();
                    Vector y = x.cross(z);
                    y.normalize_self();

                    // Alternative branch from the C++ original (wood ALWAYS
                    // runs this when !rotated_joint_as_average, because its
                    // axis_segment shadow bug doesn't shadow this sub-block).
                    if (!rotated_joint_as_average) {
                        y = el0.planes[0].z_axis();
                        z = x.cross(y);
                    }

                    // Re-orient y by intersecting a thick test segment
                    // through the joint center with the two outer plates'
                    // top/bottom planes — this picks up the actual signed
                    // direction across the assembly.
                    Point center_pt = el0.polylines[i].center();
                    double thick_a = std::max(
                        Point::distance(el0.planes[0].origin(),
                                        el0.planes[1].project(el0.planes[0].origin())),
                        Point::distance(el1.planes[0].origin(),
                                        el1.planes[1].project(el1.planes[0].origin())));
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
                    if (Intersection::line_two_planes(y_line, el0.planes[0],
                                                      el1.planes[1], clipped)) {
                        y = Vector(clipped.end()[0]-clipped.start()[0],
                                   clipped.end()[1]-clipped.start()[1],
                                   clipped.end()[2]-clipped.start()[2]);
                    }
                    x = y.cross(z);

                    // Project into local (x,y,z) frame. Uses Xform::world_to_frame
                    // (the correct world-to-local). Legacy Xform::plane_to_xy
                    // stores the basis as matrix COLUMNS — actually a
                    // local-to-world rotation despite the name — which
                    // collapses a dimension when the input geometry's normal
                    // aligns with a basis axis (the hilti failure mode).
                    Xform world_to_local = Xform::world_to_frame(o, x, y, z);
                    std::vector<Point> proj_pts = joint_area.get_points();
                    for (auto& p : proj_pts) { p.xform = world_to_local; p.transform(); }
                    if (proj_pts.empty()) { dbg_fail_reason = fmt::format("proj_empty f({},{})", i, j); continue; }
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
                    // Inverse projection (local→world) using the same basis.
                    Xform local_to_world = Xform::frame_to_world(o, x, y, z);
                    for (auto& p : rect_local) { p.xform = local_to_world; p.transform(); }

                    // Offset by element thickness along z (or insertion dir).
                    Vector offset_vector = dir_set ? dir : z;
                    offset_vector.normalize_self();
                    double d0 = 0.5 * Point::distance(
                        el0.planes[0].origin(),
                        el0.planes[1].project(el0.planes[0].origin()));
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
                    vol0.extend_edge_equally(0, ext_w);
                    vol0.extend_edge_equally(2, ext_w);
                    vol1.extend_edge_equally(0, ext_w);
                    vol1.extend_edge_equally(2, ext_w);
                    vol0.extend_edge_equally(1, ext_h);
                    vol0.extend_edge_equally(3, ext_h);
                    vol1.extend_edge_equally(1, ext_h);
                    vol1.extend_edge_equally(3, ext_h);

                    if (const char* fp = std::getenv("WOOD_F2F_DUMP")) {
                        std::ofstream flog(fp, std::ios::app);
                        flog << "F2F type13 el=(" << el_ids.first << "," << el_ids.second << ") i=" << i << " j=" << j << "\n";
                        flog << "  vol0: ";
                        for (size_t k=0;k<vol0.point_count();k++) { Point p=vol0.get_point(k); flog<<"("<<p[0]<<","<<p[1]<<","<<p[2]<<") "; }
                        flog << "\n  vol1: ";
                        for (size_t k=0;k<vol1.point_count();k++) { Point p=vol1.get_point(k); flog<<"("<<p[0]<<","<<p[1]<<","<<p[2]<<") "; }
                        flog << "\n  rect_local: ";
                        for (auto& p : rect_local) flog<<"("<<p[0]<<","<<p[1]<<","<<p[2]<<") ";
                        flog << "\n  offset_vec: ("<<offset_vector[0]<<","<<offset_vector[1]<<","<<offset_vector[2]<<")\n";
                    }
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
                    // (lj.start, lj.end, center0, center1). Matches CGAL::approximate_dihedral_angle.
                    Point center0 = avg_plane_0.project(el0.polylines[0].center());
                    Point center1 = avg_plane_1.project(el1.polylines[0].center());
                    double dihedral = Point::dihedral_angle_deg(
                        lj.start(), lj.end(), center0, center1);

                    if (dihedral < 20.0) { dbg_fail_reason = fmt::format("dihedral<20 f({},{})", i, j); continue; }

                    if (dihedral <= dihedral_angle_threshold) {
                        // ── Out-of-plane parallel (type 11) ─────────────
                        // Probe the joint axis 90° (in the face plane) to
                        // figure out which adjacent element planes are
                        // closer, then build an "open" plane×4-plane
                        // intersection to get a quad.

                        Vector connection_normal = el0.planes[i].z_axis();
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
                        if (!Intersection::line_plane(lj_l_90, el0.planes[0], pl0_0_p, false)) { dbg_fail_reason = fmt::format("lp0 f({},{})", i, j); continue; }
                        if (!Intersection::line_plane(lj_l_90, el1.planes[0], pl1_0_p, false)) { dbg_fail_reason = fmt::format("lp1 f({},{})", i, j); continue; }
                        if (!Intersection::line_plane(lj_l_90, el1.planes[1], pl1_1_p, false)) { dbg_fail_reason = fmt::format("lp2 f({},{})", i, j); continue; }

                        double d_to_pl1_0 = Point::distance(pl0_0_p, pl1_0_p);
                        double d_to_pl1_1 = Point::distance(pl0_0_p, pl1_1_p);
                        bool larger_to_pl1_0 = d_to_pl1_0 > d_to_pl1_1;
                        std::array<Plane, 4> planes4;
                        if (larger_to_pl1_0) {
                            planes4 = { el1.planes[1], el0.planes[0], el1.planes[0], el0.planes[1] };
                        } else {
                            planes4 = { el1.planes[0], el0.planes[0], el1.planes[1], el0.planes[1] };
                        }

                        Polyline vol0(std::vector<Point>{});
                        Polyline vol1(std::vector<Point>{});
                        if (!Intersection::plane_4planes_open(pl_end0, planes4, vol0)) { dbg_fail_reason = fmt::format("p4p_open0 f({},{})", i, j); continue; }
                        if (!Intersection::plane_4planes_open(pl_end1, planes4, vol1)) { dbg_fail_reason = fmt::format("p4p_open1 f({},{})", i, j); continue; }

                        // Consistent volume orientation: rotate by 2 if
                        // vertex 1 is not on the negative side of plane[i].
                        bool need_rotate = !el0.planes[i].has_on_negative_side(vol0.get_point(1));
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

                        vol0.extend_edge_equally(0, ext_w);
                        vol0.extend_edge_equally(2, ext_w);
                        vol1.extend_edge_equally(0, ext_w);
                        vol1.extend_edge_equally(2, ext_w);
                        vol0.extend_edge_equally(1, ext_h);
                        vol0.extend_edge_equally(3, ext_h);
                        vol1.extend_edge_equally(1, ext_h);
                        vol1.extend_edge_equally(3, ext_h);

                        joint_volumes[0] = vol0;
                        joint_volumes[1] = vol1;
                        joint_type = 11;

                        // DEBUG

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
                            el0.planes[0].origin(),
                            el0.planes[1].project(el0.planes[0].origin()));
                        Plane offset_plane_0 = el0.planes[i].translate_by_normal(-d0);
                        Plane offset_plane_1 = el0.planes[i].translate_by_normal( d0);

                        // Winding fix — mirror wood_main.cpp:977-987. Wood
                        // computes `w0 = squared_distance(Plane0[0].point(),
                        // Plane1[0].projection(Plane0[0].point()))` and
                        // `w1` analogously against Plane1[1]; when w0 > w1
                        // the two plates are wired such that the "near" and
                        // "far" role of Plane1[0] vs Plane1[1] has to be
                        // flipped so the quad loop visits its corners in
                        // the correct winding order.
                        //
                        // Use squared distance (same `>` result, no sqrt
                        // round-trip) so the comparison matches wood's
                        // CGAL-side decision bit-for-bit.
                        // CGAL's `IK::Plane_3::point()` picks the axis with
                        // LARGEST-MAGNITUDE plane-equation coefficient and
                        // returns the point on the plane where the other two
                        // coords are zero. (See
                        // `CGAL/constructions_on_planes_3.h` —
                        // point_on_plane helper). Session's previous
                        // foot-of-perpendicular formulation gave a different
                        // arbitrary point on the plane, and for non-parallel
                        // plane pairs the `squared_distance(point0,
                        // Plane1[0].projection(point0))` outcome depends on
                        // which point is picked. inplane_hexshell el 8's
                        // plane-swap decisions diverged from wood until this
                        // matched.
                        auto cgal_point_on_plane = [](const Plane& pl) -> Point {
                            Vector n = pl.z_axis();
                            Point  o = pl.origin();
                            double d = -(n[0]*o[0] + n[1]*o[1] + n[2]*o[2]);
                            double fa = std::abs(n[0]);
                            double fb = std::abs(n[1]);
                            double fc = std::abs(n[2]);
                            if (fa > fb && fa > fc) return Point(-d/n[0], 0.0, 0.0);
                            if (fb > fc)            return Point(0.0, -d/n[1], 0.0);
                            return Point(0.0, 0.0, -d/n[2]);
                        };
                        Point pt00   = cgal_point_on_plane(el0.planes[0]);
                        Point proj00 = el1.planes[0].project(pt00);
                        Point proj01 = el1.planes[1].project(pt00);
                        double w0 = (pt00 - proj00).magnitude_squared();
                        double w1 = (pt00 - proj01).magnitude_squared();
                        // Wood mutates Plane1[0]/Plane1[1] in place via std::swap
                        // (wood_main.cpp:986-987). We can't mutate const session
                        // params, so we instead signal the caller to swap
                        // wood_elems[ib].planes[0]/[1] (and polylines[0]/[1]) so
                        // downstream merge_joints sees the same orientation as
                        // wood. Required for simple_corners el 1/21/35 where the
                        // adjacent type-12 joint flips the "near"/"far" plate-1
                        // face role.
                        if (w0 > w1) out_swap_planes_1 = true;
                        Plane p1_0 = (w0 > w1) ? el1.planes[1] : el1.planes[0];
                        Plane p1_1 = (w0 > w1) ? el1.planes[0] : el1.planes[1];

                        std::array<Plane, 4> loop_planes_0 = {
                            offset_plane_0, el0.planes[0], offset_plane_1, el0.planes[1]
                        };
                        std::array<Plane, 4> loop_planes_1 = {
                            offset_plane_0, p1_0,        offset_plane_1, p1_1
                        };

                        Polyline vol0(std::vector<Point>{});
                        Polyline vol1(std::vector<Point>{});
                        Polyline vol2(std::vector<Point>{});
                        Polyline vol3(std::vector<Point>{});
                        if (!Intersection::plane_4planes(pl_end0, loop_planes_0, vol0)) { dbg_fail_reason = fmt::format("p4p0 f({},{})", i, j); continue; }
                        if (!Intersection::plane_4planes(pl_end1, loop_planes_0, vol1)) { dbg_fail_reason = fmt::format("p4p1 f({},{})", i, j); continue; }
                        if (!Intersection::plane_4planes(pl_end0, loop_planes_1, vol2)) { dbg_fail_reason = fmt::format("p4p2 f({},{})", i, j); continue; }
                        if (!Intersection::plane_4planes(pl_end1, loop_planes_1, vol3)) { dbg_fail_reason = fmt::format("p4p3 f({},{})", i, j); continue; }

                        for (Polyline* vp : {&vol0, &vol1, &vol2, &vol3}) {
                            (*vp).extend_edge_equally(0, ext_w);
                            (*vp).extend_edge_equally(2, ext_w);
                            (*vp).extend_edge_equally(1, ext_h);
                            (*vp).extend_edge_equally(3, ext_h);
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

                Plane plane0_0 = male_or_female ? el0.planes[0] : el1.planes[0];
                Plane plane1_0 = !male_or_female ? el0.planes[i] : el1.planes[j];
                size_t other_idx = !male_or_female
                    ? (i == 0 ? 1 : 0)
                    : (j == 0 ? 1 : 0);
                Plane plane1_1 = !male_or_female ? el0.planes[other_idx] : el1.planes[other_idx];

                bool quad_available = male_or_female ? has_quads0 : has_quads1;
                if (!quad_available) { dbg_fail_reason = fmt::format("no_quad f({},{})", i, j); continue; }
                Polyline quad_0 = male_or_female ? joint_quads0 : joint_quads1;

                Vector offset_vector(0,0,0);
                Intersection::orthogonal_vector_between_two_plane_pairs(
                        plane0_0, plane1_0, plane1_1, offset_vector);
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

                male_vol.extend_edge_equally(0, ext_w);
                male_vol.extend_edge_equally(2, ext_w);
                female_vol.extend_edge_equally(0, ext_w);
                female_vol.extend_edge_equally(2, ext_w);
                male_vol.extend_edge_equally(1, ext_h);
                male_vol.extend_edge_equally(3, ext_h);
                female_vol.extend_edge_equally(1, ext_h);
                female_vol.extend_edge_equally(3, ext_h);

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
                if (!rect_opt) { dbg_fail_reason = fmt::format("no_rect f({},{})", i, j); continue; }
                Polyline vol_a = *rect_opt;
                Polyline vol_b = *rect_opt;

                // Movement direction (insertion vector if available, else
                // the face normal). The original C++ flips the sign twice,
                // leaving dir0 in its original direction and dir1 = -dir0.
                Vector dir0 = dir_set
                    ? (i < el0.insertion_vectors.size() ? el0.insertion_vectors[i]
                                                      : el0.planes[i].z_axis())
                    : el0.planes[i].z_axis();
                dir0.normalize_self();
                Vector dir1_pre(-dir0[0], -dir0[1], -dir0[2]);
                dir0 = Vector(-dir0[0], -dir0[1], -dir0[2]);
                Vector dir1(-dir1_pre[0], -dir1_pre[1], -dir1_pre[2]);

                // Element thicknesses across the matched face.
                int next_plane_0 = (i == 0) ? 1 : 0;
                int next_plane_1 = (j == 0) ? 1 : 0;
                double dist_0 = Point::distance(
                    el0.planes[i].origin(),
                    el0.planes[next_plane_0].project(el0.planes[i].origin()));
                double dist_1 = Point::distance(
                    el1.planes[j].origin(),
                    el1.planes[next_plane_1].project(el1.planes[j].origin()));
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

                temp0.extend_edge_equally(0, ext_w);
                temp0.extend_edge_equally(2, ext_w);
                temp1.extend_edge_equally(0, ext_w);
                temp1.extend_edge_equally(2, ext_w);
                temp0.extend_edge_equally(1, ext_h);
                temp0.extend_edge_equally(3, ext_h);
                temp1.extend_edge_equally(1, ext_h);
                temp1.extend_edge_equally(3, ext_h);

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
    } // if (search_type != 1)
    // ── Cross-joint fallback (type 30) ──────────────────────────────────
    // Wood search_type=1: plane_to_face only; search_type=2: f2f first, then fallback.
    if (search_type != 0 &&
        el0.polylines.size() >= 2 && el1.polylines.size() >= 2 &&
        el0.planes.size() >= 2 && el1.planes.size() >= 2) {
        std::array<Polyline, 2> pa = { el0.polylines[0], el0.polylines[1] };
        std::array<Polyline, 2> pb = { el1.polylines[0], el1.polylines[1] };
        std::array<Plane, 2> pla = { el0.planes[0], el0.planes[1] };
        std::array<Plane, 2> plb = { el1.planes[0], el1.planes[1] };
        wood_session::CrossJoint cj;
        std::array<double, 3> cj_ext = { ext_w, ext_h, ext_l };
        if (wood_session::plane_to_face(pa, pb, pla, plb, cj, coplanar_tolerance, cj_ext)) {
            out_joint.el_ids       = el_ids;
            out_joint.face_ids     = { {{ cj.face_ids_a.first,  cj.face_ids_a.second }},
                                       {{ cj.face_ids_b.first,  cj.face_ids_b.second }} };
            out_joint.joint_type   = 30;
            out_joint.joint_area   = cj.joint_area;
            out_joint.joint_lines  = {{
                Line::from_points(cj.joint_lines[0].get_point(0), cj.joint_lines[0].get_point(1)),
                Line::from_points(cj.joint_lines[1].get_point(0), cj.joint_lines[1].get_point(1)),
            }};
            out_joint.joint_volumes_pair_a_pair_b = {
                cj.joint_volumes[0], cj.joint_volumes[1],
                std::nullopt, std::nullopt
            };
            return true;
        }
    }

    out_joint.dbg_coplanar = dbg_coplanar;
    out_joint.dbg_boolean = dbg_boolean;
    out_joint.dbg_fail_reason = dbg_fail_reason;
    return false;
}

// ───────────────────────────────────────────────────────────────────────────
// Three-valence joint addition (Vidy method).
// Creates shadow joints for the cross-connections at 3-plate intersections.
// Sets `linked_joints` on the primary joint so `ss_e_op_5` can generate
// geometry for them simultaneously.
// Ported from wood_main.cpp three_valence_joint_addition_vidy (~1552-1847).
// ───────────────────────────────────────────────────────────────────────────
