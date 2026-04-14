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
//   1. Read polylines from an OBJ, pair top/bottom faces, build ElementPlates.
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
#include "mesh.h"
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
    // Joinery library output
    std::array<std::vector<Polyline>, 2> m_outlines; // male [0]=top face, [1]=bottom face
    std::array<std::vector<Polyline>, 2> f_outlines; // female [0]=top, [1]=bottom
    // Per-polyline cut types (Stage 3). Each entry indexes 1:1 with
    // `m_outlines[face][i]` / `f_outlines[face][i]`. Empty vectors mean
    // "all entries are edge_insertion" (the legacy default before Stage 3
    // landed). See `wood_cut::cut_type` in main_5_joint_lib.h.
    std::array<std::vector<int>, 2> m_cut_types;
    std::array<std::vector<int>, 2> f_cut_types;
    int divisions = 1;
    double shift = 0.5;
    double length = 0;
    std::array<double, 3> scale = {1.0, 1.0, 1.0};
    // Wood `joint::unit_scale` (`wood_joint.h:85`). When true,
    // `joint_orient_to_connection_area` rescales the joint volumes along
    // the Z axis (joint-line direction) so the unit-cube → world transform
    // doesn't stretch the joint geometry. Set to true by joint constructors
    // like `ss_e_op_2..6`, `ts_e_p_4..5`, `ss_e_r_3`, `ss_e_ip_5`. Annen
    // joints (`ss_e_op_0/1`, `ts_e_p_3`) leave it false → no rescale.
    bool unit_scale = false;
    double unit_scale_distance = 0.0;
    // Joint linking (Vidy three-valence). The primary joint stores indices
    // of shadow joints it links to. `ss_e_op_5` uses these to generate
    // geometry for the linked joints simultaneously.
    std::vector<int> linked_joints;
    std::vector<std::vector<std::array<int, 4>>> linked_joints_seq;
    bool link = false;     // true for shadow joints created by three_valence_addition
    bool no_orient = false; // true for world-space joints (ss_e_r_0): skip orient step
    // Debug
    int dbg_coplanar = 0;
    int dbg_boolean = 0;
    std::string dbg_fail_reason;
};

// ───────────────────────────────────────────────────────────────────────────
// 2D plate-vs-plate polygon intersection in the plate plane frame, using
// the session-side Vatti boolean (Intersection::polyline_boolean). Matches
// wood's collider::clipper_util::get_intersection_between_two_polylines
// in shape; numerical results are within Vatti scale precision of Clipper2.
// ───────────────────────────────────────────────────────────────────────────
static bool clipper2_intersect(const Polyline& poly_a, const Polyline& poly_b,
                               const Plane& plane, Polyline& result) {
    // Project to 2D using poly_a[0] as origin, plane's x/y axes.
    // The plane axes are already CGAL-compatible (computed in build_wood_element).
    Point origin = poly_a.get_point(0);
    Vector xax = plane.x_axis();
    Vector yax = plane.y_axis();
    xax.normalize_self();
    yax.normalize_self();

    auto project_2d = [&](const Polyline& pl) -> Polyline {
        size_t n = pl.point_count();
        if (n > 3) {
            auto f = pl.get_point(0); auto l = pl.get_point(n-1);
            if (std::abs(f[0]-l[0])<1e-6 && std::abs(f[1]-l[1])<1e-6 && std::abs(f[2]-l[2])<1e-6)
                n--;
        }
        std::vector<Point> pts2d;
        pts2d.reserve(n + 1);
        for (size_t k = 0; k < n; k++) {
            auto p = pl.get_point(k);
            double dx = p[0]-origin[0], dy = p[1]-origin[1], dz = p[2]-origin[2];
            double u = dx*xax[0]+dy*xax[1]+dz*xax[2];
            double v = dx*yax[0]+dy*yax[1]+dz*yax[2];
            pts2d.emplace_back(u, v, 0.0);
        }
        // BooleanPolyline expects a closing-duplicate vertex.
        pts2d.push_back(pts2d.front());
        return Polyline(pts2d);
    };

    Polyline pa = project_2d(poly_a);
    Polyline pb = project_2d(poly_b);

    // clip_type: 0 = intersection (matches BooleanPolyline::compute).
    std::vector<Polyline> result_2d = Intersection::polyline_boolean(pa, pb, 0);
    if (result_2d.empty() || result_2d[0].point_count() < 3) return false;

    const Polyline& C = result_2d[0];
    size_t n = C.point_count();
    // Strip closing duplicate before computing area.
    {
        auto f = C.get_point(0); auto l = C.get_point(n-1);
        if (n > 1 && std::abs(f[0]-l[0])<1e-9 && std::abs(f[1]-l[1])<1e-9) n--;
    }

    // Shoelace area in the projected (u,v) plane.
    double area = 0.0;
    for (size_t i = 0; i < n; i++) {
        auto p0 = C.get_point(i);
        auto p1 = C.get_point((i+1) % n);
        area += p0[0]*p1[1] - p1[0]*p0[1];
    }
    area = std::abs(area) * 0.5;
    if (area < 0.01) return false;

    // Transform back to 3D in the plate plane.
    std::vector<Point> pts(n + 1);
    for (size_t k = 0; k < n; k++) {
        auto pp = C.get_point(k);
        double u = pp[0], v = pp[1];
        pts[k] = Point(origin[0] + u*xax[0] + v*yax[0],
                        origin[1] + u*xax[1] + v*yax[1],
                        origin[2] + u*xax[2] + v*yax[2]);
    }
    pts[n] = pts[0]; // close
    result = Polyline(pts);
    return true;
}

// ───────────────────────────────────────────────────────────────────────────
// Joint library: create unit joinery + orient to connection area.
// All wood-equivalent joint geometry constructors live in
// `main_5_joint_lib.h` (the session equivalent of wood's
// `wood_joint_lib.cpp`). Adding a new joint variant means editing that
// header AND wiring it into the dispatcher in `joint_create_geometry`
// further down this file.
// ───────────────────────────────────────────────────────────────────────────
#include "main_5_joint_lib.h"


// Wood `joint::orient_to_connection_area` (`wood_joint.cpp:270-355`) has an
// optional unit_scale block at lines 276-319. When `joint.unit_scale` is
// true, both joint volume rectangles are moved toward each other along the
// Z axis (joint-line direction) so their separation equals
// `unit_scale_distance` instead of the natural joint length. This prevents
// `change_basis` from stretching the unit-cube joint outline along Z when
// the joint length differs from a fixed-size template. Annen joints don't
// enable it, so this is dormant for the current test datasets — but it's
// here for any future joint variant that does (`ss_e_op_2..6`, `ts_e_p_4/5`,
// `ss_e_r_3`, `ss_e_ip_5`).
static void apply_unit_scale(WoodJoint& joint) {
    if (!joint.unit_scale) return;
    auto& vols = joint.joint_volumes_pair_a_pair_b;
    if (!vols[0].has_value() || !vols[1].has_value()) return;

    auto move_pair = [&](Polyline& a, Polyline& b) {
        // Wood `wood_joint.cpp:281` — compute unit_scale_distance lazily
        // from the rectangle's [1]→[2] edge length when not user-set.
        if (joint.unit_scale_distance == 0.0) {
            Point p1 = a.get_point(1);
            Point p2 = a.get_point(2);
            double dx = p2[0]-p1[0], dy = p2[1]-p1[1], dz = p2[2]-p1[2];
            joint.unit_scale_distance = std::floor(std::sqrt(dx*dx + dy*dy + dz*dz));
        }
        // Wood `wood_joint.cpp:290-299`. volume_segment goes from rect_a's
        // first vertex to rect_b's first vertex (along the joint line).
        Point a0 = a.get_point(0);
        Point b0 = b.get_point(0);
        Vector seg(b0[0]-a0[0], b0[1]-a0[1], b0[2]-a0[2]);
        Vector vec(seg[0]*0.5, seg[1]*0.5, seg[2]*0.5);
        Vector vec_unit = seg;
        double len = std::sqrt(vec_unit[0]*vec_unit[0] +
                               vec_unit[1]*vec_unit[1] +
                               vec_unit[2]*vec_unit[2]);
        if (len < 1e-12) return;
        double s = (joint.unit_scale_distance * 0.5) / len;
        vec_unit = Vector(seg[0]*s, seg[1]*s, seg[2]*s);
        Vector neg_vec(-vec[0], -vec[1], -vec[2]);
        Vector neg_unit(-vec_unit[0], -vec_unit[1], -vec_unit[2]);
        // 1) move both rects to the midpoint
        // 2) move them apart by ±unit_scale_distance/2 along the joint line
        a.translate(vec);
        b.translate(neg_vec);
        a.translate(neg_unit);
        b.translate(vec_unit);
    };

    move_pair(*vols[0], *vols[1]);
    if (vols[2].has_value() && vols[3].has_value()) {
        move_pair(*vols[2], *vols[3]);
    }
}

// Orient unit joinery to connection area using change_basis.
static void joint_orient_to_connection_area(WoodJoint& joint) {
    auto& vols = joint.joint_volumes_pair_a_pair_b;
    if (!vols[0].has_value() || !vols[1].has_value()) return;

    // Wood `wood_joint.cpp:276-319`: rescale joint volumes along the joint
    // line direction BEFORE change_basis so the unit-cube → world map
    // doesn't stretch the joint geometry. No-op when joint.unit_scale=false.
    apply_unit_scale(joint);

    Xform xf0 = Xform::from_change_of_basis(*vols[0], *vols[1]);
    Xform xf1 = (vols[2].has_value() && vols[3].has_value())
        ? Xform::from_change_of_basis(*vols[2], *vols[3])
        : Xform::from_change_of_basis(*vols[0], *vols[1]);
    if (xf0.is_identity() || xf1.is_identity()) {
        // Either rect is degenerate; skip orientation gracefully.
        // (The previous helper signaled failure via a bool return —
        // is_identity() is the equivalent guard for the new factory.)
        // We still attempt the transform; downstream consumers handle
        // identity-transformed outlines as a no-op.
    }

    // Transform male outlines with xf0, female with xf1.
    for (int face = 0; face < 2; face++) {
        for (auto& pl : joint.m_outlines[face]) pl = pl.transformed_xform(xf0);
        for (auto& pl : joint.f_outlines[face]) pl = pl.transformed_xform(xf1);
    }
}

// Port of wood_joint.cpp:418-510
// remove_geo_from_linked_joint_and_merge_with_current_joint
// Interleaves shadow joint outlines into the primary joint after both are oriented.
static void merge_linked_joints(WoodJoint& joint, std::vector<WoodJoint>& all_joints) {
    if (joint.linked_joints_seq.size() != joint.linked_joints.size()) return;

    for (int i = 0; i < (int)joint.linked_joints.size(); i++) {
        // wood: m_f_curr = v0 == linked.v0
        bool m_f_curr = joint.el_ids.first == all_joints[joint.linked_joints[i]].el_ids.first;
        bool m_f_next = m_f_curr;
        if (i == 1) m_f_next = !m_f_next; // wood: invert for second link

        // (true,true)=m[0], (true,false)=m[1], (false,true)=f[0], (false,false)=f[1]
        auto& curr = m_f_curr ? joint.m_outlines : joint.f_outlines;
        auto& next = m_f_next ? all_joints[joint.linked_joints[i]].m_outlines
                               : all_joints[joint.linked_joints[i]].f_outlines;

        if (joint.linked_joints_seq[i].size() * 2 != curr[0].size()) continue;

        for (int j = 0; j < (int)curr[0].size(); j += 2) {
            auto& seq      = joint.linked_joints_seq[i][j / 2];
            int start_curr = seq[0];
            int step_curr  = seq[1];
            int start_next = seq[2];
            int step_next  = seq[3];

            if (start_curr == 0 && step_curr == 0 && start_next == 0 && step_next == 0) continue;
            if (step_curr == 0 || step_next == 0) continue;

            // wood always operates on [0] — the main outline of each face
            auto pts_t  = curr[0][0].get_points(); // copy before curr is modified
            auto pts_f  = curr[1][0].get_points();
            auto npts_t = next[0][0].get_points();
            auto npts_f = next[1][0].get_points();

            std::vector<Point> m0, m1;
            m0.reserve(pts_t.size() + npts_t.size());
            m1.reserve(pts_f.size() + npts_f.size());

            // begin shift (wood line 472)
            m0.insert(m0.end(), pts_t.begin(), pts_t.begin() + start_curr);
            m1.insert(m1.end(), pts_f.begin(), pts_f.begin() + start_curr);

            // wood loop: k from start_curr while k < size - start_curr, step step_curr
            int loop_limit = (int)pts_t.size() - start_curr;
            for (int k = start_curr, it = 0; k < loop_limit; k += step_curr, it++) {
                int half = step_curr / 2; // step_curr * 0.5 from wood (always even)
                // current 1st half (wood line 479)
                m0.insert(m0.end(),
                    pts_t.begin() + start_curr + it * step_curr,
                    pts_t.begin() + start_curr + it * step_curr + half);
                m1.insert(m1.end(),
                    pts_f.begin() + start_curr + it * step_curr,
                    pts_f.begin() + start_curr + it * step_curr + half);
                // linked insertion (wood line 485)
                m0.insert(m0.end(),
                    npts_t.begin() + start_next + it * step_next,
                    npts_t.begin() + start_next + (it + 1) * step_next);
                m1.insert(m1.end(),
                    npts_f.begin() + start_next + it * step_next,
                    npts_f.begin() + start_next + (it + 1) * step_next);
                // current 2nd half (wood line 491)
                m0.insert(m0.end(),
                    pts_t.begin() + start_curr + it * step_curr + half,
                    pts_t.begin() + start_curr + (it + 1) * step_curr);
                m1.insert(m1.end(),
                    pts_f.begin() + start_curr + it * step_curr + half,
                    pts_f.begin() + start_curr + (it + 1) * step_curr);
            }

            // end shift (wood line 498)
            m0.insert(m0.end(), pts_t.end() - start_curr, pts_t.end());
            m1.insert(m1.end(), pts_f.end() - start_curr, pts_f.end());

            curr[0][0] = Polyline(m0);
            curr[1][0] = Polyline(m1);
        }

        // clear shadow geometry (wood line 507)
        next[0].clear();
        next[1].clear();
    }
}

// Compute divisions from joint line length and division_distance.
static void joint_get_divisions(WoodJoint& joint, double division_distance) {
    if (joint.joint_lines[0].squared_length() > 1e-10) {
        joint.length = std::sqrt(joint.joint_lines[0].squared_length());
        joint.divisions = std::max(1, std::min(100,
            (int)std::ceil(joint.length / division_distance)));
    }
}

// Create unit joinery geometry based on `id_representing_joint_name`.
//
// Wood's variant dispatcher lives at `wood_joint_lib.cpp:6075-6448`. Given a
// per-face joint id (from the `JOINTS_TYPES` file), it picks the family by
// numeric range and then the variant by exact id within the family. The
// `JOINT_NAMES` comment table in `wood_globals.cpp:38-47` is **misleading**:
// it claims `JOINT_NAMES[10] = ss_e_op_0`, but the actual switch at
// `wood_joint_lib.cpp:6337-6340` calls `ss_e_op_1` for case 10. Annen uses
// id=10 → `ss_e_op_1` → 8-point outline → byte-exact wood reference match.
//
// Pass `id = -1` when no `JOINTS_TYPES` file is loaded — the function then
// falls back to a topology-based default that mirrors wood's `default:`
// branches. This keeps behavior unchanged for datasets without a per-face
// id (annen_box_pair, hexbox).
static void joint_create_geometry(WoodJoint& joint, double division_distance,
                                  double shift_param, int id,
                                  std::vector<WoodJoint>* all_joints = nullptr) {
    joint_get_divisions(joint, division_distance);
    joint.shift = shift_param;

    if (id == 0) return; // already filtered upstream; defensive only

    // Wood's id-family ranges (`wood_joint_lib.cpp:6113-6140`):
    //   1-9   → group 0 (ss_e_ip)
    //   10-19 → group 1 (ss_e_op)
    //   20-29 → group 2 (ts_e_p)
    //   30-39 → group 3 (cr_c_ip)
    //   40-49 → group 4 (tt_e_p)
    //   50-59 → group 5 (ss_e_r)
    //   60-69 → group 6 (b)
    int group = -1;
    if      (id >= 1  && id <= 9 ) group = 0;
    else if (id >= 10 && id <= 19) group = 1;
    else if (id >= 20 && id <= 29) group = 2;
    else if (id >= 30 && id <= 39) group = 3;
    else if (id >= 40 && id <= 49) group = 4;
    else if (id >= 50 && id <= 59) group = 5;
    else if (id >= 60 && id <= 69) group = 6;
    else {
        // No JOINTS_TYPES file (id < 0). Infer the group from the joint's
        // detected topology so existing datasets without a per-face id keep
        // working unchanged.
        switch (joint.joint_type) {
            case 11: group = 1; break; // ss_e_op
            case 12: group = 0; break; // ss_e_ip
            case 13: group = 5; break; // ss_e_r
            case 20: group = 2; break; // ts_e_p
            case 30: group = 3; break; // cr_c_ip
            case 40: group = 4; break; // tt_e_p
            default: group = -1;
        }
    }

    switch (group) {
        case 0: // ss_e_ip (side-side in-plane, type 12) -- wood:6290-6332
            switch (id) {
                case 1: ss_e_ip_1(joint); break;
                case 2: ss_e_ip_0(joint); break;
                // case 3: ss_e_ip_2(joint, elements); break; // TODO element catalog
                case 4: ss_e_ip_3(joint); break;
                case 5: ss_e_ip_4(joint); break;
                // case 6: ss_e_ip_5(joint, elements); break; // TODO element catalog
                // case 8: side_removal(joint, elements); break; // TODO Stage 9
                // case 9: ss_e_ip_custom(joint); break; // TODO XML loader
                default: ss_e_ip_1(joint); break;
            }
            break;
        case 1: // ss_e_op (side-side out-of-plane, type 11) -- wood:6334-6392
            switch (id) {
                case 10: ss_e_op_1(joint); break;
                case 11: ss_e_op_2(joint); break;
                case 12: ss_e_op_0(joint); break;
                case 13: ss_e_op_3(joint); break;
                case 14: ss_e_op_4(joint, 0.0, true); break;
                case 15: if (all_joints) ss_e_op_5(joint, *all_joints, false); else ss_e_op_4(joint); break;
                case 16: if (all_joints) ss_e_op_5(joint, *all_joints, true); else ss_e_op_4(joint); break;
                default:
                    // id<0: topology-based (no jt file, vidy-style) → ss_e_op_5
                    // id≥0 with no specific case: wood falls to ss_e_op_1
                    if (id < 0) { if (all_joints) ss_e_op_5(joint, *all_joints, false); else ss_e_op_4(joint); }
                    else ss_e_op_1(joint);
                    break;
            }
            break;
        case 2: // ts_e_p (top-side, type 20) -- wood:6395-6448
            switch (id) {
                case 20: ts_e_p_3(joint); break;        // wood says ts_e_p_3
                case 21: ts_e_p_2(joint); break;        // ported
                case 22: ts_e_p_3(joint); break;        // wood says ts_e_p_3
                case 23: ts_e_p_0(joint); break;        // ported (hardcoded 12-pt)
                // case 24: ts_e_p_4(joint); break;     // TODO: 458 lines, complex
                                                        // chamfer/extension logic
                case 25: ts_e_p_5(joint); break;
                // ts_e_p_1 is portable but wood's dispatcher does not call it.
                // Available as a TODO if you want to wire it in (currently
                // ported to main_5.cpp but unreferenced.)
                default: ts_e_p_3(joint); break;        // wood default
            }
            break;
        case 3: // cr_c_ip (cross, type 30) -- wood:6450-6502
            switch (id) {
                case 30: cr_c_ip_0(joint); break;
                case 31: cr_c_ip_1(joint); break;
                case 32: cr_c_ip_2(joint); break;
                case 33: cr_c_ip_3(joint); break;
                case 34: cr_c_ip_4(joint); break;
                case 35: cr_c_ip_5(joint); break;
                default: cr_c_ip_0(joint); break;
            }
            break;
        case 5: // ss_e_r (side-side rotated, type 13) -- wood:6525-6553
            switch (id) {
                case 54: ss_e_r_3(joint); break;
                case 55: ss_e_r_2(joint); break;
                case 56: ss_e_r_0(joint); break;
                default: ss_e_r_0(joint); break;
            }
            break;
        // Groups 4, 6 (tt_e_p, b) not yet wired.
        default:
            switch (joint.joint_type) {
                case 11: case 12: ss_e_op_1(joint); break;
                case 20:          ts_e_p_3(joint);  break;
                default: break;
            }
            break;
    }
}

// ───────────────────────────────────────────────────────────────────────────
// Wood-compatible element — polylines and planes built exactly as
// wood::main::get_elements (wood_main.cpp:14-201) does it.
// ───────────────────────────────────────────────────────────────────────────
struct WoodElement {
    std::vector<Polyline> polylines;
    std::vector<Plane>    planes;
    bool reversed = false;  // true if build_wood_element reversed the winding
    double thickness = 0.0; // perpendicular distance between face planes 0 and 1
};

// Build wood-compatible element from a polyline pair.
// Matches wood_main.cpp get_elements: orientation check, side planes from
// 3 raw points, side polylines from 4 corners.
static WoodElement build_wood_element(std::vector<Point> pp0, std::vector<Point> pp1) {
    WoodElement el;

    // Strip closing point if present
    auto strip = [](std::vector<Point>& v) {
        if (v.size() > 3) {
            auto& f=v.front(); auto& l=v.back();
            if (std::abs(f[0]-l[0])<1e-6 && std::abs(f[1]-l[1])<1e-6 && std::abs(f[2]-l[2])<1e-6)
                v.pop_back();
        }
    };

    // Wood orientation check: transform to XY via average plane of pp0,
    // then reverse both if last transformed point has z > 0.
    // Simplified: check if average of pp1 is above pp0's plane.
    // Wood orientation check: transform merged polyline to XY via pp0's average
    // plane, then check if twoPolylines.back().z() > 0. The "back" of twoPolylines
    // is the last point of pp1.
    //
    // The XY transform uses pp0's average plane: origin=centroid, x=first_edge,
    // z=average_normal, y=cross(z,x). The z-coordinate after transform is the
    // signed distance along the local z-axis (which is the average normal).
    //
    // For a point p, its z in the local frame = dot(p - origin, z_axis).
    // This IS the signed distance from p to the XY plane at origin.
    Vector normal = Vector::average_normal(pp0);
    auto pp0_open = pp0;
    strip(pp0_open);
    Point c0 = Point::centroid(pp0_open);
    Point last_p1 = pp1.back();
    double last_z = (last_p1[0]-c0[0])*normal[0] + (last_p1[1]-c0[1])*normal[1] + (last_p1[2]-c0[2])*normal[2];
    if (last_z > 0) {
        std::reverse(pp0.begin(), pp0.end());
        std::reverse(pp1.begin(), pp1.end());
        normal = Vector::average_normal(pp0);
        el.reversed = true;
    }

    // Wood uses the CLOSED polyline for side iteration: j = 0..size()-2.
    // For a closed quad (5 points), that gives 4 side faces.
    // Compute normal/centroid from open points, but iterate closed points for sides.
    size_t n_sides = pp0.size() > 1 ? pp0.size() - 1 : 0;

    el.polylines.resize(2 + n_sides, Polyline(std::vector<Point>{}));
    el.polylines[0] = Polyline(pp0);
    el.polylines[1] = Polyline(pp1);

    // Strip for centroid computation only
    auto pp0_stripped = pp0;
    auto pp1_stripped = pp1;
    strip(pp0_stripped);
    strip(pp1_stripped);
    Point cen0 = Point::centroid(pp0_stripped);
    Point cen1 = Point::centroid(pp1_stripped);
    el.planes.resize(2 + n_sides);
    Vector neg_normal(-normal[0],-normal[1],-normal[2]);
    el.planes[0] = Plane::from_point_normal(cen0, normal);
    el.planes[1] = Plane::from_point_normal(cen1, neg_normal);
    el.thickness = Point::distance(cen0, el.planes[1].project(cen0));

    // Side planes from 3 points on the CLOSED polyline: (pp0[j+1], pp0[j], pp1[j+1])
    for (size_t j = 0; j < n_sides; j++) {
        // 3-point plane: normal = (pp0[j]-pp0[j+1]) × (pp1[j+1]-pp0[j+1])
        double ax = pp0[j][0]-pp0[j+1][0];
        double ay = pp0[j][1]-pp0[j+1][1];
        double az = pp0[j][2]-pp0[j+1][2];
        double bx = pp1[j+1][0]-pp0[j+1][0];
        double by = pp1[j+1][1]-pp0[j+1][1];
        double bz = pp1[j+1][2]-pp0[j+1][2];
        double nx = ay*bz-az*by;
        double ny = az*bx-ax*bz;
        double nz = ax*by-ay*bx;
        // Wood does NOT normalize — CGAL stores equation coefficients.
        // But session_cpp's Plane::is_coplanar divides by ||n|| only if
        // it normalizes. Keep non-unit to match wood exactly? No — session's
        // is_coplanar uses raw ax+by+cz+d which scales with ||n||.
        // Build plane with CGAL-compatible base1/base2 axes.
        Point side_origin = pp0[j+1];
        double anx = std::abs(nx), any = std::abs(ny), anz = std::abs(nz);
        Vector sb1;
        if (anx < 1e-12)      sb1 = Vector(1,0,0);
        else if (any < 1e-12) sb1 = Vector(0,1,0);
        else if (anz < 1e-12) sb1 = Vector(0,0,1);
        else if (anx<=any && anx<=anz) sb1 = Vector(0,-nz,ny);
        else if (any<=anx && any<=anz) sb1 = Vector(-nz,0,nx);
        else                           sb1 = Vector(-ny,nx,0);
        Vector snv(nx,ny,nz);
        Vector sb2 = snv.cross(sb1);
        sb1.normalize_self();
        sb2.normalize_self();
        snv.normalize_self();
        el.planes[2+j] = Plane(side_origin, sb1, sb2, snv);
        el.polylines[2+j] = Polyline(std::vector<Point>{
            pp0[j], pp0[j+1], pp1[j+1], pp1[j], pp0[j]});
    }

    return el;
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
    double coplanar_tolerance,
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
    int dbg_coplanar = 0, dbg_boolean = 0;
    std::string dbg_fail_reason;
    for (size_t i = 0; i < planes_0.size(); ++i) {
        for (size_t j = 0; j < planes_1.size(); ++j) {

            // 1. Coplanarity test (antiparallel-only — touching back-to-back).
            Point  o0 = planes_0[i].origin();
            Vector n0 = planes_0[i].z_axis();
            Point  o1 = planes_1[j].origin();
            Vector n1 = planes_1[j].z_axis();
            // Coplanarity check matching wood's cgal::plane_util::is_coplanar:
            // 1. Check anti-parallel (is_parallel_to == -1)
            // 2. Check squared_distance(projection, point) < DISTANCE_SQUARED
            // Wood uses ANGLE = 0.11 RADIANS (cos ≈ 0.9940, ~6.3°), not 0.11 degrees.
            // Session's Vector::is_parallel_to uses ANGLE_TOLERANCE_DEGREES=0.11° which
            // is far too strict and misses ts_e_p connections in one_layer/full datasets.
            {
                double n0n1 = n0[0]*n1[0]+n0[1]*n1[1]+n0[2]*n1[2];
                double ll = std::sqrt((n0[0]*n0[0]+n0[1]*n0[1]+n0[2]*n0[2])*(n1[0]*n1[0]+n1[1]*n1[1]+n1[2]*n1[2]));
                if (ll <= 0.0 || n0n1/ll > -std::cos(0.11)) continue; // not antiparallel
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

            // 2. 2D Boolean intersection using Clipper2 (matches wood exactly).
            Polyline joint_area(std::vector<Point>{});
            if (!clipper2_intersect(polylines_0[i], polylines_1[j], planes_0[i], joint_area)) {
                dbg_fail_reason = fmt::format("bool_empty f({},{})", i, j);
                continue;
            }
            if (joint_area.point_count() < 4) { // 3 unique + closing
                dbg_fail_reason = fmt::format("bool_<3pts f({},{})", i, j);
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
                if (!Intersection::polyline_plane_to_line(joint_area, avg_plane_0,
                                            alignment_segment.start(), joint_line0)) {
                    dbg_fail_reason = fmt::format("ppl0_fail f({},{})", i, j); continue;
                }
                if (joint_line0.squared_length() <= distance_squared) { dbg_fail_reason = fmt::format("jl0_short f({},{})", i, j); continue; }
                if (!Intersection::quad_from_line_top_bottom_planes(planes_0[i], joint_line0,
                                                        planes_0[0], planes_0[1],
                                                        joint_quads0)) {
                    dbg_fail_reason = fmt::format("quad0_fail f({},{})", i, j); continue;
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
                if (!Intersection::polyline_plane_to_line(joint_area, avg_plane_1,
                                            alignment_segment.start(), joint_line1)) {
                    dbg_fail_reason = fmt::format("ppl1_fail f({},{})", i, j); continue;
                }
                if (joint_line1.squared_length() <= distance_squared) { dbg_fail_reason = fmt::format("jl1_short f({},{})", i, j); continue; }
                if (!Intersection::quad_from_line_top_bottom_planes(planes_1[j], joint_line1,
                                                        planes_1[0], planes_1[1],
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
                // Wood uses ANGLE=0.11 radians (~6.3°) not degrees.
                Vector v0(joint_line0.start()[0] - joint_line0.end()[0],
                          joint_line0.start()[1] - joint_line0.end()[1],
                          joint_line0.start()[2] - joint_line0.end()[2]);
                Vector v1(joint_line1.start()[0] - joint_line1.end()[0],
                          joint_line1.start()[1] - joint_line1.end()[1],
                          joint_line1.start()[2] - joint_line1.end()[2]);
                int parallel;
                {
                    static const double wood_cos_tol = std::cos(0.11);
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
                    auto xform_inv_opt = xform.inverse();
                    if (!xform_inv_opt) { dbg_fail_reason = fmt::format("inv_xform f({},{})", i, j); continue; }
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
                    Point center0 = avg_plane_0.project(polylines_0[0].center());
                    Point center1 = avg_plane_1.project(polylines_1[0].center());
                    double dihedral = Point::dihedral_angle_deg(
                        lj.start(), lj.end(), center0, center1);

                    if (dihedral < 20.0) { dbg_fail_reason = fmt::format("dihedral<20 f({},{})", i, j); continue; }

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
                        if (!Intersection::line_plane(lj_l_90, planes_0[0], pl0_0_p, false)) { dbg_fail_reason = fmt::format("lp0 f({},{})", i, j); continue; }
                        if (!Intersection::line_plane(lj_l_90, planes_1[0], pl1_0_p, false)) { dbg_fail_reason = fmt::format("lp1 f({},{})", i, j); continue; }
                        if (!Intersection::line_plane(lj_l_90, planes_1[1], pl1_1_p, false)) { dbg_fail_reason = fmt::format("lp2 f({},{})", i, j); continue; }

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
                        if (!Intersection::plane_4planes_open(pl_end0, planes4, vol0)) { dbg_fail_reason = fmt::format("p4p_open0 f({},{})", i, j); continue; }
                        if (!Intersection::plane_4planes_open(pl_end1, planes4, vol1)) { dbg_fail_reason = fmt::format("p4p_open1 f({},{})", i, j); continue; }

                        // Consistent volume orientation: rotate by 2 if
                        // vertex 1 is not on the negative side of plane[i].
                        bool need_rotate = !planes_0[i].has_on_negative_side(vol0.get_point(1));
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

                Plane plane0_0 = male_or_female ? planes_0[0] : planes_1[0];
                Plane plane1_0 = !male_or_female ? planes_0[i] : planes_1[j];
                size_t other_idx = !male_or_female
                    ? (i == 0 ? 1 : 0)
                    : (j == 0 ? 1 : 0);
                Plane plane1_1 = !male_or_female ? planes_0[other_idx] : planes_1[other_idx];

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
    // ── Cross-joint fallback (type 30) ──────────────────────────────────
    // If no coplanar match was found, try perpendicular cross/lap detection.
    // Wood does this in wood_main.cpp after the coplanar loop.
    if (polylines_0.size() >= 2 && polylines_1.size() >= 2 &&
        planes_0.size() >= 2 && planes_1.size() >= 2) {
        std::array<Polyline, 2> pa = { polylines_0[0], polylines_0[1] };
        std::array<Polyline, 2> pb = { polylines_1[0], polylines_1[1] };
        std::array<Plane, 2> pla = { planes_0[0], planes_0[1] };
        std::array<Plane, 2> plb = { planes_1[0], planes_1[1] };
        Intersection::CrossJoint cj;
        std::array<double, 3> cj_ext = { ext_w, ext_h, ext_l };
        if (Intersection::plane_to_face(pa, pb, pla, plb, cj, coplanar_tolerance, cj_ext)) {
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
static void three_valence_joint_addition_vidy(
    const std::vector<std::vector<int>>& tv_groups,
    std::vector<WoodElement>& elements,
    std::vector<WoodJoint>& joints,
    std::unordered_map<uint64_t, int>& joints_map,
    const std::vector<std::pair<int,int>>& adjacency_pairs)
{
    if (tv_groups.size() < 2) return;

    // Pre-reserve to prevent reallocation during push_back (which would
    // invalidate joints[id] references). Each group can add up to 2 joints.
    joints.reserve(joints.size() + (tv_groups.size() - 1) * 2);

    auto pair_key = [](int a, int b) -> uint64_t {
        if (a > b) std::swap(a, b);
        return ((uint64_t)a << 32) | (uint64_t)b;
    };

    // FIX #2: Proper CGAL-compatible squared distance from point to plane.
    // CGAL::squared_distance(point, plane) = (a*px+b*py+c*pz+d)^2 / (a^2+b^2+c^2)
    auto sq_dist_pt_plane = [](const Point& p, const Plane& pl) -> double {
        double v = pl.a()*p[0] + pl.b()*p[1] + pl.c()*p[2] + pl.d();
        double n2 = pl.a()*pl.a() + pl.b()*pl.b() + pl.c()*pl.c();
        return (n2 > 1e-20) ? (v * v) / n2 : (v * v);
    };

    for (size_t gi = 1; gi < tv_groups.size(); gi++) {
        auto& g = tv_groups[gi];
        if (g.size() != 4) continue;
        int s0 = g[0], s1 = g[1], e20 = g[2], e31 = g[3];
        int n_elems = (int)elements.size();
        if (s0 < 0 || s1 < 0 || e20 < 0 || e31 < 0) continue;
        if (s0 >= n_elems || s1 >= n_elems) continue;
        // Match wood's behavior: out-of-range e20/e31 causes UB → huge vsum → return
        if (e20 >= n_elems || e31 >= n_elems) return;

        // Parallel check matching wood's is_same_direction(can_be_flipped=true)
        // wood: is_parallel_to != 0 → |cos_angle| >= cos(0.11) ≈ 0.994
        if (e20 != e31) {
            auto is_parallel_wood = [](const Vector& a, const Vector& b) -> bool {
                double ll = a.magnitude() * b.magnitude();
                if (ll <= 0.0) return false;
                return std::abs(a.dot(b) / ll) >= std::cos(0.11);
            };
            Vector n_s0 = elements[s0].planes[0].z_axis();
            Vector n_e31 = elements[e31].planes[0].z_axis();
            Vector n_s1 = elements[s1].planes[0].z_axis();
            Vector n_e20 = elements[e20].planes[0].z_axis();
            if (!is_parallel_wood(n_s0, n_e31) || !is_parallel_wood(n_s1, n_e20)) continue;
        }

        // Find primary joint between s0-s1
        auto it = joints_map.find(pair_key(s0, s1));
        if (it == joints_map.end()) continue;
        int id = it->second;
        if (!joints[id].joint_volumes_pair_a_pair_b[0].has_value()) continue;

        // Find nearest/farthest planes between element pairs
        double d00 = sq_dist_pt_plane(elements[s0].planes[0].origin(), elements[e31].planes[0]);
        double d01 = sq_dist_pt_plane(elements[s0].planes[0].origin(), elements[e31].planes[1]);
        Plane plane00_far = d00 < d01 ? elements[e31].planes[0] : elements[e31].planes[1];

        d00 = sq_dist_pt_plane(plane00_far.origin(), elements[s0].planes[0]);
        d01 = sq_dist_pt_plane(plane00_far.origin(), elements[s0].planes[1]);
        Plane plane01_near = d00 < d01 ? elements[s0].planes[1] : elements[s0].planes[0];

        double d10 = sq_dist_pt_plane(elements[s1].planes[0].origin(), elements[e20].planes[0]);
        double d11 = sq_dist_pt_plane(elements[s1].planes[0].origin(), elements[e20].planes[1]);
        Plane plane10_far = d10 < d11 ? elements[e20].planes[0] : elements[e20].planes[1];

        d10 = sq_dist_pt_plane(plane10_far.origin(), elements[s1].planes[0]);
        d11 = sq_dist_pt_plane(plane10_far.origin(), elements[s1].planes[1]);
        Plane plane11_near = d10 < d11 ? elements[s1].planes[1] : elements[s1].planes[0];

        // Joint volume edge lines for projection (wood lines 1685-1686)
        auto& jvol = *joints[id].joint_volumes_pair_a_pair_b[0];
        Line l0 = Line::from_points(jvol.get_point(0), jvol.get_point(1));
        Line l1 = Line::from_points(jvol.get_point(1), jvol.get_point(2));

        // Determine which edge is parallel to which projection (wood 1703-1730)
        // Project volume edges onto plane01_near and check parallelism
        Point proj_p0 = plane01_near.project(jvol.get_point(0));
        Point proj_p1 = plane01_near.project(jvol.get_point(1));
        Point proj_p2 = plane01_near.project(jvol.get_point(2));
        Vector proj_l1_dir(proj_p1[0]-proj_p2[0], proj_p1[1]-proj_p2[1], proj_p1[2]-proj_p2[2]);
        // is_parallel_01: 0 means NOT parallel (projection collapsed the edge)
        bool is_parallel_01 = (proj_l1_dir.is_parallel_to(l1.to_vector()) == 0);
        // Wood: if l1's projection is NOT parallel → use l1 first
        std::array<Line, 2> ll = is_parallel_01
            ? std::array<Line, 2>{l1, l0}
            : std::array<Line, 2>{l0, l1};

        // Find translation endpoints via line-plane intersection (infinite).
        // Wood does not check for failure here; it just calls line_plane unconditionally
        // and then overrides p10/p11 when e20==e31. We must skip the e20==e31 checks
        // to avoid a spurious continue when ll[1] is parallel to the plane.
        Point p00, p01, p10, p11;
        if (!Intersection::line_plane(ll[0], plane00_far, p00, false)) continue;
        if (!Intersection::line_plane(ll[0], plane01_near, p01, false)) continue;
        if (e20 == e31) {
            p10 = p00; p11 = p01;
        } else {
            if (!Intersection::line_plane(ll[1], plane10_far, p10, false)) continue;
            if (!Intersection::line_plane(ll[1], plane11_near, p11, false)) continue;
        }

        Vector trans0(p00[0]-p01[0], p00[1]-p01[1], p00[2]-p01[2]);
        Vector trans1(p10[0]-p11[0], p10[1]-p11[1], p10[2]-p11[2]);

        // Validate translations (wood lines 1767-1778, signed sum check)
        double vsum = trans0[0] + trans0[1] + trans0[2]
                    + trans1[0] + trans1[1] + trans1[2];
        if (vsum < -1e8 || vsum > 1e8) continue;

        // Copy joint volumes (wood lines 1761-1762)
        auto copy_vols = [&]() -> std::array<Polyline, 4> {
            std::array<Polyline, 4> vols;
            for (int k = 0; k < 4; k++) {
                if (joints[id].joint_volumes_pair_a_pair_b[k].has_value())
                    vols[k] = *joints[id].joint_volumes_pair_a_pair_b[k];
            }
            return vols;
        };
        auto jv0_copy = copy_vols();
        auto jv1_copy = copy_vols();

        // Shift jv1 volumes to align with translation direction (wood 1782-1796)
        int shift_amt = 0;
        for (int j = 0; j < 4; j++) {
            Vector v(jv1_copy[0].get_point(j)[0] - jv1_copy[0].get_point(j+1)[0],
                     jv1_copy[0].get_point(j)[1] - jv1_copy[0].get_point(j+1)[1],
                     jv1_copy[0].get_point(j)[2] - jv1_copy[0].get_point(j+1)[2]);
            if (v.is_parallel_to(trans1) == 1) { shift_amt = j; break; }
        }
        for (size_t k = 0; k < 4; k++)
            if (jv1_copy[k].point_count() == 5)
                jv1_copy[k].shift(shift_amt);

        // Copy joint lines (wood lines 1798-1799)
        auto jlines0 = joints[id].joint_lines;
        auto jlines1 = joints[id].joint_lines;

        // FIX #3: Translate BOTH volumes AND lines (wood lines 1802-1806)
        for (int k = 0; k < 2; k++) {
            jv0_copy[k].translate(trans0);
            jv1_copy[k].translate(trans1);
            // Translate joint lines too
            Point js0 = jlines0[k].start();
            Point je0 = jlines0[k].end();
            jlines0[k] = Line::from_points(
                Point(js0[0]+trans0[0], js0[1]+trans0[1], js0[2]+trans0[2]),
                Point(je0[0]+trans0[0], je0[1]+trans0[1], je0[2]+trans0[2]));
            Point js1 = jlines1[k].start();
            Point je1 = jlines1[k].end();
            jlines1[k] = Line::from_points(
                Point(js1[0]+trans1[0], js1[1]+trans1[1], js1[2]+trans1[2]),
                Point(je1[0]+trans1[0], je1[1]+trans1[1], je1[2]+trans1[2]));
        }

        // Check if joint order was reversed (wood line 1813)
        if (joints[id].el_ids.first == s1) {
            std::swap(e20, e31);
            std::swap(s0, s1);
        }

        // Shadow joints go to j_mf.back() — the extra slot beyond planes (wood line 1827-1828).
        // face_ids = -1 so the j_mf build loop routes them via the link==true path.
        // (wood: joints.emplace_back(..., -1, -1, -1, -1, ...))

        // Create shadow joint 0 (s0 ↔ e20) — wood lines 1824-1829
        WoodJoint shadow0;
        shadow0.el_ids = {s0, e20};
        shadow0.face_ids = { {{-1,-1}}, {{-1,-1}} };
        shadow0.joint_type = joints[id].joint_type;
        shadow0.joint_area = joints[id].joint_area;
        shadow0.joint_lines = jlines0;
        shadow0.joint_volumes_pair_a_pair_b = {jv0_copy[0], jv0_copy[1], std::nullopt, std::nullopt};
        shadow0.link = true;
        int shadow0_idx = (int)joints.size();
        joints.push_back(shadow0);
        joints_map[pair_key(s0, e20)] = shadow0_idx;

        // Create shadow joint 1 if e20 != e31 — wood lines 1831-1839
        int shadow1_idx = -1;
        if (e20 != e31) {
            WoodJoint shadow1;
            shadow1.el_ids = {s1, e31};
            shadow1.face_ids = { {{-1,-1}}, {{-1,-1}} };
            shadow1.joint_type = joints[id].joint_type;
            shadow1.joint_area = joints[id].joint_area;
            shadow1.joint_lines = jlines1;
            shadow1.joint_volumes_pair_a_pair_b = {jv1_copy[0], jv1_copy[1], std::nullopt, std::nullopt};
            shadow1.link = true;
            shadow1_idx = (int)joints.size();
            joints.push_back(shadow1);
            joints_map[pair_key(s1, e31)] = shadow1_idx;
        }

        // Wire linked_joints on the primary joint — wood line 1841
        if (e20 != e31)
            joints[id].linked_joints = {shadow0_idx, shadow1_idx};
        else
            joints[id].linked_joints = {shadow0_idx};
    }
}

// ───────────────────────────────────────────────────────────────────────────
// Three-valence joint alignment (Annen method).
// Shortens overlapping joint lines at 3-plate intersections to avoid collisions.
// Ported from wood_main.cpp three_valence_joint_alignment_annen.
// ───────────────────────────────────────────────────────────────────────────
static void three_valence_joint_alignment_annen(
    const std::vector<std::vector<int>>& tv_groups,
    const std::vector<WoodElement>& elements,
    std::vector<WoodJoint>& joints,
    const std::vector<std::pair<int,int>>& adjacency_pairs)
{
    // Build a map from element pair → joint index.
    auto pair_key = [](int a, int b) -> uint64_t {
        if (a > b) std::swap(a, b);
        return ((uint64_t)a << 32) | (uint64_t)b;
    };
    std::unordered_map<uint64_t, int> joints_map;
    for (size_t ji = 0; ji < joints.size(); ji++) {
        int e0 = joints[ji].el_ids.first, e1 = joints[ji].el_ids.second;
        joints_map[pair_key(e0, e1)] = (int)ji;
    }

    for (size_t gi = 1; gi < tv_groups.size(); gi++) {
        auto& g = tv_groups[gi];
        if (g.size() != 4) continue;
        int s0 = g[0], s1 = g[1], e20 = g[2], e31 = g[3];

        auto it0 = joints_map.find(pair_key(s0, s1));
        auto it1 = joints_map.find(pair_key(e20, e31));
        if (it0 == joints_map.end() || it1 == joints_map.end()) continue;

        int id_0 = it0->second, id_1 = it1->second;
        auto& j0 = joints[id_0];
        auto& j1 = joints[id_1];

        // Compute overlap of the two joint lines.
        Line l0 = j0.joint_lines[0];
        // Orient l1 to match l0 direction.
        double d_s = Point::distance(l0.start(), j1.joint_lines[0].start());
        double d_e = Point::distance(l0.start(), j1.joint_lines[0].end());
        Line l1 = (d_s <= d_e)
            ? j1.joint_lines[0]
            : Line::from_points(j1.joint_lines[0].end(), j1.joint_lines[0].start());

        Line overlap;
        l0.overlap_average(l1, overlap);

        // Shorten by element thickness.
        double thickness = 0;
        int e0_idx = j0.el_ids.first;
        if (e0_idx >= 0 && e0_idx < (int)elements.size()) {
            auto& el = elements[e0_idx];
            if (el.polylines.size() >= 2 && el.polylines[0].point_count() > 0 && el.polylines[1].point_count() > 0) {
                auto p0 = el.polylines[0].get_point(0);
                auto p1_proj = el.planes[1].project(p0);
                thickness = Point::distance(p0, p1_proj);
            }
        }
        overlap.extend(-thickness, -thickness);

        // Update both joint lines to the shortened overlap.
        j0.joint_lines[0] = overlap;
        j1.joint_lines[0] = overlap;

        // Clip joint volumes using planes at the overlap endpoints.
        auto vol_normal = [](const Polyline& vol) -> Vector {
            if (vol.point_count() < 3) return Vector(0,0,1);
            Point p0 = vol.get_point(0), p1 = vol.get_point(1), p2 = vol.get_point(2);
            Vector a(p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]);
            Vector b(p0[0]-p1[0], p0[1]-p1[1], p0[2]-p1[2]);
            return a.cross(b);
        };

        // Clip j0 volumes
        if (j0.joint_volumes_pair_a_pair_b[0].has_value()) {
            Vector cross0 = vol_normal(*j0.joint_volumes_pair_a_pair_b[0]);
            cross0.normalize_self();
            Point ol_s = overlap.start(), ol_e = overlap.end();
            Plane pl0_0 = Plane::from_point_normal(ol_s, cross0);
            Plane pl0_1 = Plane::from_point_normal(ol_e, cross0);
            for (int vp = 0; vp < 4; vp += 2) {
                if (!j0.joint_volumes_pair_a_pair_b[vp].has_value() || !j0.joint_volumes_pair_a_pair_b[vp+1].has_value()) continue;
                auto& v0 = *j0.joint_volumes_pair_a_pair_b[vp];
                auto& v1 = *j0.joint_volumes_pair_a_pair_b[vp+1];
                Line s0l = Line::from_points(v0.get_point(0), v1.get_point(0));
                Line s1l = Line::from_points(v0.get_point(1), v1.get_point(1));
                Line s2l = Line::from_points(v0.get_point(2), v1.get_point(2));
                Line s3l = Line::from_points(v0.get_point(3), v1.get_point(3));
                Intersection::plane_4lines(pl0_0, s0l, s1l, s2l, s3l, v0);
                Intersection::plane_4lines(pl0_1, s0l, s1l, s2l, s3l, v1);
            }
        }

        // Clip j1 volumes
        if (j1.joint_volumes_pair_a_pair_b[0].has_value()) {
            Vector cross1 = vol_normal(*j1.joint_volumes_pair_a_pair_b[0]);
            cross1.normalize_self();
            Point ol_s = overlap.start(), ol_e = overlap.end();
            Plane pl1_0 = Plane::from_point_normal(ol_e, cross1);
            Plane pl1_1 = Plane::from_point_normal(ol_s, cross1);
            for (int vp = 0; vp < 4; vp += 2) {
                if (!j1.joint_volumes_pair_a_pair_b[vp].has_value() || !j1.joint_volumes_pair_a_pair_b[vp+1].has_value()) continue;
                auto& v0 = *j1.joint_volumes_pair_a_pair_b[vp];
                auto& v1 = *j1.joint_volumes_pair_a_pair_b[vp+1];
                Line s0l = Line::from_points(v0.get_point(0), v1.get_point(0));
                Line s1l = Line::from_points(v0.get_point(1), v1.get_point(1));
                Line s2l = Line::from_points(v0.get_point(2), v1.get_point(2));
                Line s3l = Line::from_points(v0.get_point(3), v1.get_point(3));
                Intersection::plane_4lines(pl1_0, s0l, s1l, s2l, s3l, v0);
                Intersection::plane_4lines(pl1_1, s0l, s1l, s2l, s3l, v1);
            }
        }
    }
}

// ───────────────────────────────────────────────────────────────────────────
// merge_joints: merge oriented joint cuts into element plate polylines.
// Produces per-element output: [merged_top, merged_bottom, hole0_t, hole0_b, ...]
//
// j_mf[element_id][face_id] = vector of (joint_index, is_male)
// ───────────────────────────────────────────────────────────────────────────
using JMF = std::vector<std::vector<std::vector<std::pair<int,bool>>>>;
// Verbatim port of `wood::element::merge_joints` (wood_element.cpp:654-1424).
// Operates on CLOSED polylines (size = open_count + 1) so the wood indexing
// `id = i-2`, `prev = (n+id-1)%n`, `next = (id+1)%n`, `n = pline0.size()-1`
// applies 1:1. Mutates the joints vector in two places (joint.reverse swaps
// the m[0]↔m[1] or f[0]↔f[1] outline slots) — same mutation strategy as wood.
//
// Handles both case (2) — line joint with 2-point endpoint marker — and
// case (5) — rectangle joint with 5-point endpoint marker — matching
// `wood_element.cpp:816-1108`.
static std::vector<Polyline> merge_joints_for_element(
    const WoodElement& el,
    const std::vector<std::vector<std::pair<int,bool>>>& el_jmf,
    std::vector<WoodJoint>& joints)
{
    // CLOSED polylines (mutable copies — wood relocates vertices in place).
    auto pline0 = el.polylines[0].get_points();
    auto pline1 = el.polylines[1].get_points();
    auto joint_planes = el.planes;     // mutable copy of side planes

    const double scale_0 = 1000000.0;
    const double scale_1 = 1000.0;
    const double DISTANCE_SQUARED = 0.01; // wood::GLOBALS::DISTANCE_SQUARED

    std::map<size_t, std::pair<std::pair<double,double>, Polyline>> sorted_by_id_plines_0;
    std::map<size_t, std::pair<std::pair<double,double>, Polyline>> sorted_by_id_plines_1;

    int last_id = -1;

    // ── STEP 1: iterate side edges (face indices i = 2..N) ─────────────────
    // wood_element.cpp:740-1110
    for (size_t i = 2; i < el_jmf.size() && i < el.planes.size(); i++) {
        for (size_t j = 0; j < el_jmf[i].size(); j++) {
            int joint_id = el_jmf[i][j].first;
            bool male_or_female = el_jmf[i][j].second;
            WoodJoint& jt = joints[joint_id];

            // jm aliases either m_outlines or f_outlines on the joint.
            // jm[0] / jm[1] correspond to wood's `(male, true)` / `(male, false)`.
            auto& jm = male_or_female ? jt.m_outlines : jt.f_outlines;

            // Sanity: skip if either outline (top or bottom) is missing
            // its main outline OR its 2-point endpoint marker.
            // wood_element.cpp:758-762
            if (jm[0].size() < 2 || jm[1].size() < 2) continue;
            if (jm[0][1].point_count() == 0 || jm[1][1].point_count() == 0) continue;

            // is_geo_reversed: if jm[0]'s endpoint marker pt0 is FARTHER from
            // plane[0] (the element's top plane) than jm[1]'s endpoint marker
            // pt0 is, the data has top/bottom swapped → call joint::reverse.
            // wood_element.cpp:764-768
            Point ep_top0 = jm[0][1].get_point(0);
            Point ep_bot0 = jm[1][1].get_point(0);
            double d_top = Point::distance(ep_top0, el.planes[0].project(ep_top0));
            double d_bot = Point::distance(ep_bot0, el.planes[0].project(ep_bot0));
            bool is_geo_reversed = (d_top * d_top) > (d_bot * d_bot);
            if (is_geo_reversed) std::swap(jm[0], jm[1]);

            // Switch on the endpoint marker's point count, mirroring
            // wood_element.cpp:774's `switch(joints[](male, true)[1].size())`.
            //   2 → line joint (case 2, the common path)
            //   5 → rectangle joint (case 5, e.g. cross/boundary joints)
            // Anything else → skip (matches wood's `default: continue;`).
            size_t endpoint_pt_count = jm[0][1].point_count();
            if (endpoint_pt_count != 2 && endpoint_pt_count != 5) continue;

            if (endpoint_pt_count == 5) {
                // ── case (5): rectangle joint ──────────────────────────────
                // wood_element.cpp:1049-1105. The first outline polyline of
                // each side (`jm[0][0]` and `jm[1][0]`) is the rectangle that
                // gets clipped against the plate polygon via Clipper2 open-
                // path intersection. The result is a clipped joint polyline +
                // a parametric `(t0, t1)` pair on the plate edges.
                Polyline joint_pline_0;
                std::pair<double, double> cp_pair_0;
                if (!Intersection::closed_and_open_paths_2d(
                        el.polylines[0], jm[0][0], el.planes[0],
                        joint_pline_0, cp_pair_0))
                    continue;

                Polyline joint_pline_1;
                std::pair<double, double> cp_pair_1;
                if (!Intersection::closed_and_open_paths_2d(
                        el.polylines[1], jm[1][0], el.planes[1],
                        joint_pline_1, cp_pair_1))
                    continue;

                // wood_element.cpp:1089-1099 — insert into the sorted maps
                // with the parametric position derived from the clipping
                // result, NOT a fake (id+0.1, id+0.9) range.
                size_t key0 = (size_t)(scale_0 * std::floor(cp_pair_0.first))
                            + (size_t)(scale_1 * std::fmod(cp_pair_0.first, 1.0));
                size_t key1 = (size_t)(scale_0 * std::floor(cp_pair_1.first))
                            + (size_t)(scale_1 * std::fmod(cp_pair_1.first, 1.0));
                sorted_by_id_plines_0.insert({key0, {cp_pair_0, joint_pline_0}});
                sorted_by_id_plines_1.insert({key1, {cp_pair_1, joint_pline_1}});
                continue; // case 5 is done; skip the case 2 logic below
            }

            // ── case (2): line joint (2-point endpoint marker) ──────────────
            // wood_element.cpp:816-1028

            // Get top/bottom joint lines from the endpoint markers.
            // wood_element.cpp:826-827
            Polyline& joint_line_0 = jm[0][1];
            Polyline& joint_line_1 = jm[1][1];
            Point j0_s = joint_line_0.get_point(0);
            Point j0_e = joint_line_0.get_point(1);
            Point j1_s = joint_line_1.get_point(0);
            Point j1_e = joint_line_1.get_point(1);
            (void)j1_e;

            // Update joint_planes[i] from the cross of (top edge) × (top→bot).
            // wood_element.cpp:842-845
            Vector x_axis(j0_e[0]-j0_s[0], j0_e[1]-j0_s[1], j0_e[2]-j0_s[2]);
            Vector y_axis(j0_s[0]-j1_s[0], j0_s[1]-j1_s[1], j0_s[2]-j1_s[2]);
            Vector z_axis = x_axis.cross(y_axis);
            bool z_axis_valid = (z_axis.magnitude() > 1e-12);
            if (z_axis_valid) {
                joint_planes[i] = Plane::from_point_normal(j0_s, z_axis);
            }

            // wood_element.cpp:861-867
            size_t n = pline0.size() - 1;       // open count
            int    id = static_cast<int>(i) - 2;
            int    prev = ((int)n + id - 1) % (int)n;
            int    next = (id + 1) % (int)n;

            // 4 plane-plane-plane intersections to compute the relocated
            // plate vertices at this joint's two ends, top and bottom.
            // wood_element.cpp:894-901
            // When z_axis=0 (degenerate endpoint marker), wood assigns a zero-normal
            // plane which makes all intersections fail → plate vertices stay at
            // original positions. We replicate this by skipping the intersections.
            Point p0_int, p1_int, p2_int, p3_int;
            bool is_intersected_0 = z_axis_valid && Intersection::plane_plane_plane(joint_planes[2 + prev], joint_planes[i], joint_planes[0], p0_int);
            bool is_intersected_1 = z_axis_valid && Intersection::plane_plane_plane(joint_planes[2 + next], joint_planes[i], joint_planes[0], p1_int);
            bool is_intersected_2 = z_axis_valid && Intersection::plane_plane_plane(joint_planes[2 + prev], joint_planes[i], joint_planes[1], p2_int);
            bool is_intersected_3 = z_axis_valid && Intersection::plane_plane_plane(joint_planes[2 + next], joint_planes[i], joint_planes[1], p3_int);

            // Back-relocation: if the immediately previous edge also had a
            // joint AND the current joint line is OFFSET from the original
            // edge (perpendicular distance > DISTANCE_SQUARED), use the
            // joint-joint plane intersection to override p0_int / p2_int with
            // the shared corner that BOTH joints should snap to.
            // wood_element.cpp:927-945
            if (last_id == (int)i - 1) {
                Point e0a = el.polylines[0].get_point(i - 2);
                Point e0b = el.polylines[0].get_point(i - 1);
                Point e1a = el.polylines[1].get_point(i - 2);
                Point e1b = el.polylines[1].get_point(i - 1);
                auto perp_dist_sq_to_infinite_line = [](const Point& p, const Point& la, const Point& lb) -> double {
                    Vector d(lb[0]-la[0], lb[1]-la[1], lb[2]-la[2]);
                    double l2 = d[0]*d[0] + d[1]*d[1] + d[2]*d[2];
                    if (l2 < 1e-20) return 0.0;
                    double t = ((p[0]-la[0])*d[0] + (p[1]-la[1])*d[1] + (p[2]-la[2])*d[2]) / l2;
                    Point pr(la[0]+d[0]*t, la[1]+d[1]*t, la[2]+d[2]*t);
                    double dx = p[0]-pr[0], dy = p[1]-pr[1], dz = p[2]-pr[2];
                    return dx*dx + dy*dy + dz*dz;
                };
                bool gd0 = perp_dist_sq_to_infinite_line(j0_s, e0a, e0b) > DISTANCE_SQUARED;
                bool gd1 = perp_dist_sq_to_infinite_line(j1_s, e1a, e1b) > DISTANCE_SQUARED;
                if (gd0 || gd1) {
                    Point p0, p1;
                    bool ji0 = Intersection::plane_plane_plane(joint_planes[i], joint_planes[i - 1], joint_planes[0], p0);
                    bool ji1 = Intersection::plane_plane_plane(joint_planes[i], joint_planes[i - 1], joint_planes[1], p1);
                    if (ji0 && ji1) {
                        p0_int = p0;
                        p2_int = p1;
                    }
                }
            }

            // Relocate plate vertices to the intersection points.
            // wood_element.cpp:953-975
            if (is_intersected_0) pline0[id]   = p0_int;
            if (is_intersected_1) pline0[next] = p1_int;
            if (is_intersected_2) pline1[id]   = p2_int;
            if (is_intersected_3) pline1[next] = p3_int;

            last_id = (int)i;

            // is_geo_flipped: reverse the joint outline if the data orientation
            // doesn't match the polygon walk direction. Wood reads
            // `(male, !is_geo_reversed)[0]` as the reference. After our swap,
            // that slot is jm[is_geo_reversed ? 1 : 0][0].
            // wood_element.cpp:1003-1010
            //
            // Wood compares against pline0[id+1]. id+1 may equal n; pline0 is
            // CLOSED with size n+1 so pline0[n] == pline0[0] and the index is
            // always valid.
            Polyline& flip_ref = jm[is_geo_reversed ? 1 : 0][0];
            if (flip_ref.point_count() >= 1) {
                Point fr_front = flip_ref.get_point(0);
                Point fr_back  = flip_ref.get_point(flip_ref.point_count() - 1);
                Point ref_pt   = pline0[id + 1];   // pline0 is closed
                double dx_f = fr_front[0]-ref_pt[0], dy_f = fr_front[1]-ref_pt[1], dz_f = fr_front[2]-ref_pt[2];
                double dx_b = fr_back[0]-ref_pt[0],  dy_b = fr_back[1]-ref_pt[1],  dz_b = fr_back[2]-ref_pt[2];
                double d_front_sq = dx_f*dx_f + dy_f*dy_f + dz_f*dz_f;
                double d_back_sq  = dx_b*dx_b + dy_b*dy_b + dz_b*dz_b;
                if (d_front_sq < d_back_sq) {
                    // Wood reverses BOTH `(male, !is_geo_reversed)[0]` and
                    // `(male,  is_geo_reversed)[0]`. After our swap, those
                    // are respectively jm[is_geo_reversed?1:0][0] and
                    // jm[is_geo_reversed?0:1][0] — i.e. always BOTH jm[0][0]
                    // and jm[1][0] regardless of is_geo_reversed.
                    jm[0][0].reverse();
                    jm[1][0].reverse();
                }
            }

            // Insert the joint outlines into the sorted maps using a key
            // built from `(id + 0.1, id + 0.9)`. wood_element.cpp:1015-1024
            // After our swap, jm[0][0] is geometric top, jm[1][0] is bottom.
            std::pair<double, double> cp_pair(id + 0.1, id + 0.9);
            size_t key = (size_t)(scale_0 * std::floor(cp_pair.first))
                       + (size_t)(scale_1 * std::fmod(cp_pair.first, 1.0));
            sorted_by_id_plines_0.insert({key, {cp_pair, jm[0][0]}});
            sorted_by_id_plines_1.insert({key, {cp_pair, jm[1][0]}});
        }
    }

    // ── Build merged polylines from sorted maps ────────────────────────────
    // wood_element.cpp:1129-1253
    auto build_merged = [&](std::vector<Point>& pline,
                            std::map<size_t, std::pair<std::pair<double,double>, Polyline>>& sorted)
        -> Polyline
    {
        // pline is CLOSED (size = open + 1). Flag every vertex as kept,
        // then walk each joint's parametric span and unflag interior vertices.
        std::vector<bool> point_flags(pline.size(), true);
        for (auto& kv : sorted) {
            const auto& cp = kv.second.first;
            for (size_t k = (size_t)std::ceil(cp.first);
                 k <= (size_t)std::floor(cp.second) && k < point_flags.size();
                 k++)
            {
                point_flags[k] = false;
            }
        }
        if (!point_flags.empty()) point_flags.back() = false;  // ignore closing duplicate

        // Re-add surviving vertices as single-point entries in the sorted map.
        // wood_element.cpp:1170-1174
        for (size_t k = 0; k < point_flags.size(); k++) {
            if (point_flags[k]) {
                size_t kk = (size_t)(k * scale_0);
                sorted.insert({kk, {{(double)k, (double)k}, Polyline(std::vector<Point>{pline[k]})}});
            }
        }

        // Concatenate everything in sorted order then close.
        // wood_element.cpp:1178-1182, 1252 — no deduplication, wood preserves C_dup
        std::vector<Point> merged;
        for (auto& kv : sorted)
            for (size_t pi = 0; pi < kv.second.second.point_count(); pi++)
                merged.push_back(kv.second.second.get_point(pi));
        if (!merged.empty()) merged.push_back(merged.front());
        return Polyline(merged);
    };

    Polyline merged_top = build_merged(pline0, sorted_by_id_plines_0);
    Polyline merged_bot = build_merged(pline1, sorted_by_id_plines_1);

    // ── Phase B: collect HOLES from top/bottom face joints (i = 0, 1) ──────
    // Wood emits holes BEFORE the merged plate polylines so the consumer
    // sees [hole0_top, hole0_bot, hole1_top, hole1_bot, ..., merged_top, merged_bot].
    // wood_element.cpp:1285-1343
    //
    // Stage 3: cut-type-aware iteration. We read `f_cut_types` (or
    // `m_cut_types` if is_male) and only emit polylines tagged
    // `wood_cut::hole`. Bounding rectangles are tagged
    // `insert_between_multiple_edges` (skipped). Drilled holes / mill cuts
    // / slices for non-edge_insertion joint variants will use this same
    // filter when those constructors land.
    //
    // Backwards compatibility: an EMPTY cut_types vector means "all
    // entries are edge_insertion" (the legacy default). For Phase B we
    // fall back to the previous "skip last entry" behavior in that case.
    std::vector<Polyline> result;
    for (size_t i = 0; i < 2 && i < el_jmf.size(); i++) {
        for (size_t k = 0; k < el_jmf[i].size(); k++) {
            int joint_id = el_jmf[i][k].first;
            bool male_or_female = el_jmf[i][k].second;
            WoodJoint& jt = joints[joint_id];
            auto& jm = male_or_female ? jt.m_outlines : jt.f_outlines;
            auto& jct = male_or_female ? jt.m_cut_types : jt.f_cut_types;
            if (jm[0].empty() || jm[1].empty()) continue;

            // Phase B uses a DIFFERENT is_geo_reversed test: it compares the
            // BACK (last) outline's point[0], not the endpoint marker.
            // wood_element.cpp:1313-1314
            Point t_back0 = jm[0].back().get_point(0);
            Point f_back0 = jm[1].back().get_point(0);
            double dt = Point::distance(t_back0, el.planes[0].project(t_back0));
            double df = Point::distance(f_back0, el.planes[0].project(f_back0));
            if ((dt * dt) > (df * df)) {
                std::swap(jm[0], jm[1]);
                std::swap(jct[0], jct[1]);
            }

            const bool have_cut_types = !jct[0].empty();
            // Legacy fallback: no cut types → iterate up to size-1 (skip
            // bounding rectangle), tag everything as hole.
            size_t lim = have_cut_types
                ? jm[0].size()
                : (jm[0].size() > 1 ? jm[0].size() - 1 : 0);

            for (size_t kk = 0; kk < lim && kk < jm[1].size(); kk++) {
                if (have_cut_types) {
                    int ct = (kk < jct[0].size()) ? jct[0][kk] : wood_cut::edge_insertion;
                    if (ct != wood_cut::hole) continue; // skip bounding rect, slices, drills, etc.
                }
                Polyline top = jm[0][kk];
                Polyline bot = jm[1][kk];
                // Winding check uses planes[0] (the TOP plane) for BOTH holes,
                // matching wood_element.cpp:1329-1336.
                bool is_cw = top.is_clockwise(el.planes[0]);
                if (!is_cw) {
                    top.reverse();
                    bot.reverse();
                }
                result.push_back(top);
                result.push_back(bot);
            }
        }
    }

    // ── Phase C: collect HOLES from side joints (i = 2..N) ──────────────────
    // Wood: wood_element.cpp:1352-1412
    // Shadow joints (link=true) ARE included — ss_e_op_5 creates ss_e_op_4
    // geometry for them; the female side has holes that must be collected here.
    for (size_t i = 2; i < el_jmf.size(); i++) {
        for (size_t k = 0; k < el_jmf[i].size(); k++) {
            int joint_id = el_jmf[i][k].first;
            bool male_or_female = el_jmf[i][k].second;
            WoodJoint& jt = joints[joint_id];
            auto& jm  = male_or_female ? jt.m_outlines : jt.f_outlines;
            auto& jct = male_or_female ? jt.m_cut_types : jt.f_cut_types;
            if (jm[0].empty() || jm[1].empty()) continue;
            if (jct[0].empty()) continue;
            std::vector<int> id_of_holes;
            for (int ki = 0; ki < (int)jct[0].size(); ki += 2)
                if (jct[0][ki] == wood_cut::hole)
                    id_of_holes.push_back(ki);
            if (id_of_holes.empty()) continue;
            Point t_back = jm[0].back().get_point(0);
            Point f_back = jm[1].back().get_point(0);
            double dt = Point::distance(t_back, el.planes[0].project(t_back));
            double df = Point::distance(f_back, el.planes[0].project(f_back));
            if ((dt * dt) > (df * df)) {
                std::swap(jm[0], jm[1]);
                std::swap(jct[0], jct[1]);
            }
            for (int ki : id_of_holes) {
                if (ki >= (int)jm[0].size() || ki >= (int)jm[1].size()) continue;
                Polyline top = jm[0][ki];
                Polyline bot = jm[1][ki];
                if (!top.is_clockwise(el.planes[0])) { top.reverse(); bot.reverse(); }
                result.push_back(top);
                result.push_back(bot);
            }
        }
    }

    // ── Output: merged plate polylines last ────────────────────────────────
    // wood_element.cpp:1421-1422
    result.push_back(merged_top);
    result.push_back(merged_bot);

    return result;
}

} // anonymous namespace

static void run_dataset(const std::string& obj_name, const std::string& adj_name,
                        const std::string& pb_name,
                        const std::string& tv_name = "",
                        const std::string& iv_name = "",
                        const std::string& jt_name = "",
                        double ss_div_dist = 200.0,
                        double line_ext    = 0.0,
                        std::vector<double> ext_vec = {},
                        bool remove_duplicate_pts = false,
                        int joint_count = 0,
                        double ss_op_shift = 0.64) {
    using Clock = std::chrono::high_resolution_clock;
    auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
    auto t0 = Clock::now();

    fmt::print("\n=== {} ===\n", obj_name);

    // 1. Load polylines (consecutive pairs from wood XML export).
    auto polylines = obj::read_obj_polylines(
        (base / "session_data" / obj_name).string());
    // wood_xml.cpp:135-138: remove consecutive duplicates when flagged
    // (wood's type_plates_name_joint_linking_vidychapel_one_layer passes true)
    if (remove_duplicate_pts) {
        const double DIST_SQ = 0.01;
        for (auto& pl : polylines) {
            auto pts = pl.get_points();
            std::vector<Point> cleaned;
            cleaned.reserve(pts.size());
            for (auto& p : pts) {
                if (cleaned.empty()) { cleaned.push_back(p); continue; }
                double dx=p[0]-cleaned.back()[0], dy=p[1]-cleaned.back()[1], dz=p[2]-cleaned.back()[2];
                if (dx*dx+dy*dy+dz*dz >= DIST_SQ) cleaned.push_back(p);
            }
            pl = Polyline(cleaned);
        }
    }
    std::vector<std::pair<int,int>> pairs;
    for (size_t i = 0; i + 1 < polylines.size(); i += 2)
        pairs.emplace_back(static_cast<int>(i), static_cast<int>(i + 1));

    // 2. Build wood-compatible elements.
    std::vector<WoodElement> wood_elems;
    wood_elems.reserve(pairs.size());
    for (auto [a, b] : pairs)
        wood_elems.push_back(build_wood_element(
            polylines[a].get_points(), polylines[b].get_points()));

    // ElementPlates for session storage + OBB adjacency.
    Session session("WoodF2F");
    auto g = session.add_group("Elements");
    std::vector<std::shared_ptr<ElementPlate>> plates;
    plates.reserve(pairs.size());
    for (auto [a, b] : pairs) {
        auto plate = std::make_shared<ElementPlate>(
            polylines[a], polylines[b], "plate_" + std::to_string(a));
        session.add_element(plate, g);
        plates.push_back(plate);
    }
    // Build plate meshes (closed box: top + bottom + sides) for Rhino viz.
    auto g_mesh = session.add_group("PlateMeshes");
    for (size_t ei = 0; ei < wood_elems.size(); ei++) {
        auto& we = wood_elems[ei];
        auto pts0 = we.polylines[0].get_points(); // top
        auto pts1 = we.polylines[1].get_points(); // bottom
        // Strip closing
        if (pts0.size()>3 && std::abs(pts0.front()[0]-pts0.back()[0])<1e-6) pts0.pop_back();
        if (pts1.size()>3 && std::abs(pts1.front()[0]-pts1.back()[0])<1e-6) pts1.pop_back();
        size_t nv = pts0.size();
        if (nv < 3 || pts1.size() < 3) continue;
        auto m = std::make_shared<Mesh>();
        m->name = "plate_" + std::to_string(ei);
        // Add vertices: 0..nv-1 = top, nv..2nv-1 = bottom
        for (auto& p : pts0) m->add_vertex(p);
        for (auto& p : pts1) m->add_vertex(p);
        // Top face
        std::vector<size_t> top_f; for (size_t k=0;k<nv;k++) top_f.push_back(k);
        m->add_face(top_f);
        // Bottom face (reversed winding)
        std::vector<size_t> bot_f; for (size_t k=nv;k-->0;) bot_f.push_back(k+nv);
        m->add_face(bot_f);
        // Side faces (winding: top_k+1 → top_k → bot_k → bot_k+1)
        for (size_t k=0;k<nv;k++) {
            size_t k1 = (k+1)%nv;
            m->add_face({k1, k, k+nv, k1+nv});
        }
        session.add_mesh(m, g_mesh);
    }
    auto t1 = Clock::now();

    // 3. Adjacency: load from file if provided, otherwise OBB+BVH search.
    std::vector<std::pair<int, int>> adjacency_pairs;
    if (!adj_name.empty()) {
        std::ifstream adj_in((base / "session_data" / adj_name).string());
        int a, b;
        while (adj_in >> a >> b) adjacency_pairs.emplace_back(a, b);
        fmt::print("adjacency: {} pairs from {}\n", adjacency_pairs.size(), adj_name);
    }
    if (adjacency_pairs.empty()) {
        std::vector<Element*> elem_ptrs(plates.size());
        for (size_t k = 0; k < plates.size(); k++) elem_ptrs[k] = plates[k].get();
        auto adj_flat = Intersection::adjacency_search(elem_ptrs, 200.0);
        for (size_t i = 0; i + 3 < adj_flat.size(); i += 4)
            adjacency_pairs.emplace_back(adj_flat[i], adj_flat[i + 1]);
        fmt::print("adjacency: {} pairs from OBB+BVH\n", adjacency_pairs.size());
    }
    auto t2 = Clock::now();

    // 4. Define ALL wood-joint detection parameters as LOCAL variables.
    //    No globals, no config struct — every tunable is right here so the
    //    caller can see what knobs the algorithm exposes. The names match
    //    the original wood::GLOBALS::* fields one-for-one.
    std::vector<double> joint_volume_extension =
        ext_vec.empty() ? std::vector<double>{0.0, 0.0, line_ext} : ext_vec;
    double  limit_min_joint_length      = 0.0;
    double  distance_squared            = 1e-6;
    double  coplanar_tolerance          = 0.01; // DISTANCE_SQUARED, used as squared distance
    double  dihedral_angle_threshold    = 150.0;
    bool    all_treated_as_rotated      = false;
    bool    rotated_joint_as_average    = false;

    // 5. Iterate every adjacent pair and run the wood joint detector.
    //    Per-type groups with colors for visualization.
    Color col_ss(220, 50, 50, 255, "ss_red");
    Color col_ts(50, 100, 220, 255, "ts_blue");
    Color col_tt(50, 200, 50, 255, "tt_green");
    auto g_area_11  = session.add_group("JointAreas_SS_11");   g_area_11->color  = col_ss;
    auto g_area_20  = session.add_group("JointAreas_TS_20");   g_area_20->color  = col_ts;
    auto g_area_oth = session.add_group("JointAreas_Other");   g_area_oth->color = col_tt;
    auto g_line_11  = session.add_group("JointLines_SS_11");   g_line_11->color  = col_ss;
    auto g_line_20  = session.add_group("JointLines_TS_20");   g_line_20->color  = col_ts;
    auto g_line_oth = session.add_group("JointLines_Other");   g_line_oth->color = col_tt;
    auto g_vol_11   = session.add_group("JointVols_SS_11");    g_vol_11->color   = col_ss;
    auto g_vol_20   = session.add_group("JointVols_TS_20");    g_vol_20->color   = col_ts;
    auto g_vol_oth  = session.add_group("JointVols_Other");    g_vol_oth->color  = col_tt;
    // Per-element insertion vectors. Wood's annen XML stores one
    // <insertion_vectors> block per element, each with `n_faces` <vector>
    // entries (faces 0/1 = top/bottom = (0,0,0); faces 2..N = side faces
    // with the assembly direction the carpenter slides the joint along).
    // We pre-extract these to a flat .txt file (one element per line, all
    // vectors as space-separated `x y z x y z ...`) and load here.
    //
    // Empty file → empty per-element vectors → face_to_face_wood's
    // `dir_set` stays false and the algorithm falls back to the orthogonal
    // perpendicular vector helper (which is geometrically valid but
    // produces only orthogonal joints — same as before this load).
    std::vector<std::vector<Vector>> per_element_insertion_vectors(
        wood_elems.size(), std::vector<Vector>{});
    if (!iv_name.empty()) {
        std::ifstream iv_in((base / "session_data" / iv_name).string());
        std::string iv_line;
        size_t ei = 0;
        size_t total_loaded = 0;
        while (std::getline(iv_in, iv_line) && ei < per_element_insertion_vectors.size()) {
            std::istringstream iss(iv_line);
            std::vector<Vector>& vecs = per_element_insertion_vectors[ei];
            double x, y, z;
            while (iss >> x >> y >> z) {
                vecs.emplace_back(x, y, z);
                total_loaded++;
            }
            ei++;
        }
        fmt::print("insertion_vectors: {} vectors across {} elements from {}\n",
                   total_loaded, ei, iv_name);
    }
    // Wood reverses side-face insertion vectors when an element is orientation-
    // reversed (same as wood_main.cpp get_elements line 156-157).
    for (size_t ei = 0; ei < wood_elems.size(); ei++) {
        if (wood_elems[ei].reversed) {
            auto& vecs = per_element_insertion_vectors[ei];
            if (vecs.size() > 2)
                std::reverse(vecs.begin() + 2, vecs.end());
        }
    }

    int counts[6] = {0, 0, 0, 0, 0, 0}; // [11, 12, 13, 20, 30, 40]
    int n_failed  = 0;
    int n_success = 0;
    std::vector<WoodJoint> all_joints;
    for (size_t k = 0; k < adjacency_pairs.size(); ++k) {
        int ia = adjacency_pairs[k].first;
        int ib = adjacency_pairs[k].second;

        const std::vector<Polyline>& polys_a  = wood_elems[ia].polylines;
        const std::vector<Polyline>& polys_b  = wood_elems[ib].polylines;
        const std::vector<Plane>&    planes_a = wood_elems[ia].planes;
        const std::vector<Plane>&    planes_b = wood_elems[ib].planes;
        const std::vector<Vector>&   ins_a    = per_element_insertion_vectors[ia];
        const std::vector<Vector>&   ins_b    = per_element_insertion_vectors[ib];

        WoodJoint joint;
        bool ok = face_to_face_wood(
            /*joint_id*/                 k,
            /*polylines_0*/              polys_a,
            /*polylines_1*/              polys_b,
            /*planes_0*/                 planes_a,
            /*planes_1*/                 planes_b,
            /*insertion_vectors_0*/      ins_a,
            /*insertion_vectors_1*/      ins_b,
            /*el_ids_in*/                std::pair<int, int>(ia, ib),
            /*joint_volume_extension*/   joint_volume_extension,
            /*limit_min_joint_length*/   limit_min_joint_length,
            /*distance_squared*/         distance_squared,
            /*coplanar_tolerance*/       coplanar_tolerance,
            /*dihedral_angle_threshold*/ dihedral_angle_threshold,
            /*all_treated_as_rotated*/   all_treated_as_rotated,
            /*rotated_joint_as_average*/ rotated_joint_as_average,
            /*out_joint*/                joint);
        if (!ok) {
            fmt::print("  FAIL pair ({},{}) coplanar={} boolean={} reason={}\n",
                       ia, ib, joint.dbg_coplanar, joint.dbg_boolean, joint.dbg_fail_reason);
            ++n_failed;
            continue;
        }
        ++n_success;
        switch (joint.joint_type) {
            case 11: ++counts[0]; break;
            case 12: ++counts[1]; break;
            case 13: ++counts[2]; break;
            case 20: ++counts[3]; break;
            case 30: ++counts[4]; break;
            case 40: ++counts[5]; break;
            default: break;
        }
        all_joints.push_back(std::move(joint));
    }
    auto t3 = Clock::now();

    // Sort joints by element pair for deterministic comparison with wood.
    std::sort(all_joints.begin(), all_joints.end(), [](const WoodJoint& a, const WoodJoint& b) {
        int a1=std::min(a.el_ids.first,a.el_ids.second), a2=std::max(a.el_ids.first,a.el_ids.second);
        int b1=std::min(b.el_ids.first,b.el_ids.second), b2=std::max(b.el_ids.first,b.el_ids.second);
        return a1 < b1 || (a1 == b1 && a2 < b2);
    });


    // Three-valence joint alignment (Annen method).
    if (!tv_name.empty()) {
        std::vector<std::vector<int>> tv_groups;
        std::ifstream tv_in((base / "session_data" / tv_name).string());
        std::string tv_line;
        while (std::getline(tv_in, tv_line)) {
            std::istringstream iss(tv_line);
            std::vector<int> group;
            int v;
            while (iss >> v) group.push_back(v);
            if (!group.empty()) tv_groups.push_back(group);
        }
        if (tv_groups.size() > 1) {
            // Build joints_map for the addition function
            auto pair_key = [](int a, int b) -> uint64_t {
                if (a > b) std::swap(a, b);
                return ((uint64_t)a << 32) | (uint64_t)b;
            };
            std::unordered_map<uint64_t, int> joints_map;
            for (size_t ji = 0; ji < all_joints.size(); ji++) {
                int e0 = all_joints[ji].el_ids.first, e1 = all_joints[ji].el_ids.second;
                joints_map[pair_key(e0, e1)] = (int)ji;
            }
            // The first group's first element is the instruction flag:
            //   0 = annen alignment only
            //   1 = vidy addition (create shadow joints) + alignment
            int instruction = tv_groups[0].empty() ? 0 : tv_groups[0][0];
            if (instruction == 1) {
                // Vidy addition: create shadow joints for joint linking.
                // Wood switch case 1 — only this runs, NOT annen alignment.
                size_t before_vidy = all_joints.size();
                three_valence_joint_addition_vidy(tv_groups, wood_elems, all_joints, joints_map, adjacency_pairs);
                fmt::print("vidy_addition: {} shadow joints created (total {})\n",
                           all_joints.size() - before_vidy, all_joints.size());
            } else {
                // Annen alignment: shorten overlapping joint lines (instruction == 0 only).
                three_valence_joint_alignment_annen(tv_groups, wood_elems, all_joints, adjacency_pairs);
            }
        }
        fmt::print("three_valence: {} groups applied\n", tv_groups.size());
    }

    // Per-element per-face joint type IDs (the wood JOINTS_TYPES filter).
    // 6 ints per line, one line per element. Values:
    //   0  → no joint on this face (skip joint construction)
    //   1-9   → ss_e_ip variant id  (side-side in-plane,    type 12)
    //   10-19 → ss_e_op variant id  (side-side out-of-plane, type 11)
    //   20-29 → ts_e_p  variant id  (top-side,               type 20)
    //   30-39 → cr_c_ip variant id  (cross,                  type 30)
    //   40-49 → tt_e_p  variant id  (top-top,                type 40)
    //   50-59 → ss_e_r  variant id  (side-side rotated,      type 13)
    //   60-69 → b       variant id  (boundary,               type 60)
    //
    // Wood at `wood_joint_lib.cpp:6075-6112` computes
    //   id_representing_joint_name = max(JOINTS_TYPES[v0][f0_0], JOINTS_TYPES[v1][f1_0])
    // and `continue`s (skips construction) if the result is 0. The same
    // gating is applied below before `joint_create_geometry`. We do NOT use
    // the id to pick a joint variant — session keeps the variant fixed at
    // ss_e_op_1 / ts_e_p_3 to maintain byte-exact parity with the wood
    // reference; promote to a real `id_representing_joint_name → variant`
    // dispatcher when the user wants per-face variant overrides.
    std::vector<std::vector<int>> per_element_joints_types(wood_elems.size());
    if (!jt_name.empty()) {
        std::ifstream jt_in((base / "session_data" / jt_name).string());
        std::string jt_line;
        size_t ei = 0;
        size_t total_loaded = 0;
        while (std::getline(jt_in, jt_line) && ei < per_element_joints_types.size()) {
            std::istringstream iss(jt_line);
            int v;
            while (iss >> v) {
                per_element_joints_types[ei].push_back(v);
                total_loaded++;
            }
            ei++;
        }
        fmt::print("joints_types: {} ids across {} elements from {}\n",
                   total_loaded, ei, jt_name);
    }

    // Create unit joinery + orient to connection area.
    // Per-type div_dist + shift, mirroring wood's
    // GLOBALS::JOINTS_PARAMETERS_AND_TYPES table for the annen test
    // (wood_test.cpp:2720-2730 + wood_globals.cpp:49-54).
    int n_filtered = 0;
    for (auto& j : all_joints) {
        // Wood-style id_representing_joint_name (`wood_joint_lib.cpp:6075-6079`).
        // Sentinel `-1` = no JOINTS_TYPES file → topology-based default in
        // `joint_create_geometry`. Empty per-element vector = same effect.
        int id_representing_joint_name = -1;
        if (!per_element_joints_types.empty()) {
            int e0 = j.el_ids.first, e1 = j.el_ids.second;
            int f0 = j.face_ids.first[0], f1 = j.face_ids.second[0];
            // Remap post-reversal face index back to original face index for
            // JOINTS_TYPES lookup. build_wood_element may reverse the winding,
            // which reorders the side planes: orig_side_j = n_sides-1 - rev_side_j.
            // The joints_types file uses original (pre-reversal) face indices.
            auto orig_face = [&](int ei, int fi) -> int {
                if (ei < 0 || ei >= (int)wood_elems.size()) return fi;
                if (!wood_elems[ei].reversed) return fi;
                if (fi < 2) return 1 - fi; // top(0)↔bottom(1)
                int n_sides = (int)wood_elems[ei].planes.size() - 2;
                return 2 + (n_sides - 1 - (fi - 2));
            };
            int of0 = orig_face(e0, f0);
            int of1 = orig_face(e1, f1);
            int id0 = (e0 >= 0 && e0 < (int)per_element_joints_types.size()
                       && of0 >= 0 && of0 < (int)per_element_joints_types[e0].size())
                      ? std::abs(per_element_joints_types[e0][of0]) : 0;
            int id1 = (e1 >= 0 && e1 < (int)per_element_joints_types.size()
                       && of1 >= 0 && of1 < (int)per_element_joints_types[e1].size())
                      ? std::abs(per_element_joints_types[e1][of1]) : 0;
            // Only treat the file as authoritative if either element actually
            // had a non-empty per-face id list. Elements with an empty list
            // (e.g. parsing skipped a line) fall through to the topology
            // default rather than getting silently filtered.
            if (e0 >= 0 && e0 < (int)per_element_joints_types.size() &&
                e1 >= 0 && e1 < (int)per_element_joints_types.size() &&
                (per_element_joints_types[e0].size() > 0 ||
                 per_element_joints_types[e1].size() > 0)) {
                id_representing_joint_name = std::max(id0, id1);
                // Wood: `if (id_representing_joint_name == 0) continue;`
                // (`wood_joint_lib.cpp:6109-6112`)
                if (id_representing_joint_name == 0) {
                    n_filtered++;
                    continue;
                }
            }
        }

        // When no jt file (id=-1) and caller provided a joint_count override,
        // resolve to that id — mirrors wood's default_parameters_for_four_types[group*3+2]
        // lookup in wood_joint_lib.cpp:6191-6196.
        if (id_representing_joint_name == -1 && joint_count > 0)
            id_representing_joint_name = joint_count;

        if (j.link) continue; // shadow joints: geometry set by ss_e_op_5, orient+merge below

        double div_dist;
        double shift_val;
        switch (j.joint_type) {
            case 11: case 12:
                div_dist  = ss_div_dist;
                shift_val = ss_op_shift;
                break;
            case 13:
                div_dist  = 300.0;  // ss_e_r default
                shift_val = 0.5;
                // Pre-set element thickness for ss_e_r_2/3 (unit_scale_distance).
                // The constructors read it as joint_volume_edge_length, then
                // override it with 120*shift at the end.
                {
                    int ei = j.el_ids.first;
                    if (ei >= 0 && ei < (int)wood_elems.size())
                        j.unit_scale_distance = wood_elems[ei].thickness;
                }
                break;
            case 20:
                div_dist  = 450.0;  // ts_e_p default
                shift_val = 0.5;    // ts_e_p default
                break;
            default:
                div_dist  = 300.0;
                shift_val = 0.5;
        }
        joint_create_geometry(j, div_dist, shift_val, id_representing_joint_name, &all_joints);
        if (!j.no_orient)
            joint_orient_to_connection_area(j);
        // wood_joint_lib.cpp:6621-6626: orient shadows then interleave geometry into primary
        if (!j.linked_joints.empty() &&
            (id_representing_joint_name == 15 || id_representing_joint_name == 16)) {
            for (int sid : j.linked_joints)
                if (!all_joints[sid].no_orient)
                    joint_orient_to_connection_area(all_joints[sid]);
            merge_linked_joints(j, all_joints);
        }
    }

    // Build per-element j_mf mapping: j_mf[element_id][face_id] = [(joint_idx, is_male)]
    // Wood sizes j_mf as (sides)+2+1: +1 is the extra slot for shadow joints (j_mf.back()).
    // Phase C in merge_joints_for_element processes j_mf.back() — shadow joints carry
    // f_outlines (holes) set by ss_e_op_4 called inside ss_e_op_5 for the primary joint.
    size_t n_elems = wood_elems.size();
    JMF j_mf(n_elems);
    for (size_t ei = 0; ei < n_elems; ei++)
        j_mf[ei].resize(wood_elems[ei].planes.size() + 1); // +1 = extra slot for shadow joints
    for (size_t ji = 0; ji < all_joints.size(); ji++) {
        auto& j = all_joints[ji];
        int e0 = j.el_ids.first, e1 = j.el_ids.second;
        if (j.link) {
            // Shadow joints → j_mf.back() (wood line 1827-1828)
            if (e0 >= 0 && e0 < (int)n_elems) j_mf[e0].back().push_back({(int)ji, true});
            if (e1 >= 0 && e1 < (int)n_elems) j_mf[e1].back().push_back({(int)ji, false});
        } else {
            int f0 = j.face_ids.first[0], f1 = j.face_ids.second[0];
            if (e0 >= 0 && e0 < (int)n_elems && f0 >= 0 && f0 < (int)j_mf[e0].size())
                j_mf[e0][f0].push_back({(int)ji, true});
            if (e1 >= 0 && e1 < (int)n_elems && f1 >= 0 && f1 < (int)j_mf[e1].size())
                j_mf[e1][f1].push_back({(int)ji, false});
        }
    }

    // Merge joints with plate polylines (polylines only, no meshes).
    // Each element gets its own group so merged plates are selectable per element.
    Color col_merged(50, 50, 50, 255, "merged_dark");
    // Per-element summary (meta) + coordinate dump for byte-by-byte comparison with wood.
    std::ofstream meta_out((base / "session_data" /
                            (std::string(pb_name) + "_meta.txt")).string());
    std::ofstream coord_out((base / "session_data" /
                             (std::string(pb_name) + "_coords.txt")).string());
    for (size_t ei = 0; ei < n_elems; ei++) {
        auto merged = merge_joints_for_element(wood_elems[ei], j_mf[ei], all_joints);
        auto g_el = session.add_group(fmt::format("element_{}", ei));
        g_el->color = col_merged;
        for (size_t mi = 0; mi < merged.size(); mi++) {
            auto mpl = std::make_shared<Polyline>(merged[mi]);
            mpl->name = fmt::format("merged_{}_{}", ei, mi);
            mpl->linecolor = col_merged;
            session.add_polyline(mpl, g_el);
        }
        // Meta line matches wood: "<n_polylines> <pt_count_1> <pt_count_2> ..."
        meta_out << merged.size();
        for (size_t mi = 0; mi < merged.size(); mi++)
            meta_out << ' ' << merged[mi].point_count();
        meta_out << '\n';
        // Coordinate dump: matches wood's out.xml polyline_group[ei] structure
        coord_out << "element " << ei << "\n";
        for (size_t mi = 0; mi < merged.size(); mi++) {
            coord_out << "  poly " << mi << ":";
            for (size_t pi = 0; pi < merged[mi].point_count(); pi++) {
                Point p = merged[mi].get_point(pi);
                coord_out << " " << p[0] << " " << p[1] << " " << p[2];
            }
            coord_out << "\n";
        }
    }
    meta_out.close();
    coord_out.close();

    // Store joint detection results (areas, lines, volumes) in typed/colored groups.
    for (size_t ji = 0; ji < all_joints.size(); ji++) {
        auto& j = all_joints[ji];
        int jt = j.joint_type;
        Color& col = (jt == 11) ? col_ss : (jt == 20) ? col_ts : col_tt;
        auto& ga = (jt == 11) ? g_area_11 : (jt == 20) ? g_area_20 : g_area_oth;
        auto& gv = (jt == 11) ? g_vol_11  : (jt == 20) ? g_vol_20  : g_vol_oth;
        auto jpl = std::make_shared<Polyline>(j.joint_area);
        jpl->name = "joint_area_" + std::to_string(ji);
        jpl->linecolor = col;
        session.add_polyline(jpl, ga);
        for (int vi = 0; vi < 4; ++vi) {
            if (j.joint_volumes_pair_a_pair_b[vi].has_value()) {
                auto jvol = std::make_shared<Polyline>(*j.joint_volumes_pair_a_pair_b[vi]);
                jvol->name = "joint_vol_" + std::to_string(ji) + "_" + std::to_string(vi);
                jvol->linecolor = col;
                session.add_polyline(jvol, gv);
            }
        }
    }

    // 6. Save protobuf.
    session.pb_dump((base / "session_data" / pb_name).string());
    auto t4 = Clock::now();

    auto ms = [](auto a, auto b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    fmt::print("{} polylines -> {} elements -> {} adjacency pairs\n",
               polylines.size(), pairs.size(), adjacency_pairs.size());
    fmt::print("  joints: {} success / {} failed\n", n_success, n_failed);
    fmt::print("  by type: 11={} 12={} 13={} 20={} 30={} 40={}\n",
               counts[0], counts[1], counts[2], counts[3], counts[4], counts[5]);
    fmt::print("  time: {:.0f}ms\n", ms(t0, t4));
}

int main() {
    // Calls ordered to match wood_test.cpp function order.

    // wood line 204: hexbox_and_corner
    run_dataset("hexbox_and_corner.obj", "",
                "WoodF2F_hexbox.pb");

    // wood line 265: vidy_corner
    run_dataset("vidy_corner.obj",
                "vidy_corner_adjacency.txt",
                "WoodF2F_vidy_corner.pb",
                "vidy_corner_three_valence.txt",
                "",
                "vidy_corner_joints_types.txt",
                150.0, -10.0);

    // wood line 428: vidy_one_layer
    run_dataset("vidy_one_layer.obj", "",
                "WoodF2F_vidy_one_layer.pb",
                "", "", "",
                50.0, -15.0, {}, true);

    // wood line 488: vidy_one_axis_two_layers
    run_dataset("vidy_one_axis_two_layers.obj",
                "vidy_one_axis_two_layers_adjacency.txt",
                "WoodF2F_vidy_one_axis_two_layers.pb",
                "vidy_one_axis_two_layers_three_valence.txt",
                "vidy_one_axis_two_layers_insertion_vectors.txt",
                "vidy_one_axis_two_layers_joints_types.txt",
                50.0, 0.0,
                {0,0,-200,0,0,-200,0,0,-20,0,0,-20});

    // wood line 611: vidy_full
    run_dataset("vidy_full.obj",
                "vidy_full_adjacency.txt",
                "WoodF2F_vidy_full.pb",
                "vidy_full_three_valence.txt",
                "vidy_full_insertion_vectors.txt",
                "vidy_full_joints_types.txt",
                50.0, -10.0);

    // wood line 888: inplane_butterflies
    run_dataset("inplane_butterflies.obj", "",
                "WoodF2F_inplane_butterflies.pb");

    // wood line 1129: outofplane_folding
    // JOINT_VOLUME_EXTENSION[2]=-20; per-element insertion vectors
    run_dataset("vidy_folding.obj", "",
                "WoodF2F_vidy_folding.pb",
                "", "vidy_folding_insertion_vectors.txt", "",
                200.0, -20.0);

    // wood line 1384: outofplane_box
    // [1*3+2]=17; JOINT_VOLUME_EXTENSION[2]=-100; div_dist=450 (global default)
    run_dataset("outofplane_box.obj", "",
                "WoodF2F_outofplane_box.pb",
                "", "", "",
                450.0, -100.0, {}, false, 17);

    // wood line 1440: outofplane_tetra
    // [1*3+1]=0.5; [1*3+2]=17; JOINT_VOLUME_EXTENSION[2]=-100; div_dist=450
    run_dataset("outofplane_tetra.obj", "",
                "WoodF2F_outofplane_tetra.pb",
                "", "", "",
                450.0, -100.0, {}, false, 17, 0.5);

    // wood line 1497: outofplane_dodecahedron
    // [1*3+0]=1000; [1*3+1]=0.5; [1*3+2]=14; JOINT_VOLUME_EXTENSION[2]=-250
    run_dataset("outofplane_dodecahedron.obj", "",
                "WoodF2F_outofplane_dodecahedron.pb",
                "", "", "",
                1000.0, -250.0, {}, false, 14, 0.5);

    // wood line 1555: outofplane_icosahedron
    // [1*3+0]=1000; [1*3+1]=0.5; [1*3+2]=14; JOINT_VOLUME_EXTENSION[2]=-250
    run_dataset("outofplane_icosahedron.obj", "",
                "WoodF2F_outofplane_icosahedron.pb",
                "", "", "",
                1000.0, -250.0, {}, false, 14, 0.5);

    // wood line 1613: outofplane_octahedron
    // [1*3+0]=1000; [1*3+1]=0.5; [1*3+2]=14; JOINT_VOLUME_EXTENSION[2]=-250
    run_dataset("outofplane_octahedron.obj", "",
                "WoodF2F_outofplane_octahedron.pb",
                "", "", "",
                1000.0, -250.0, {}, false, 14, 0.5);

    // wood line 1671: simple_corners
    // [0*3+2]=2; [1*3+2]=12; JOINT_VOLUME_EXTENSION[2]=-20
    run_dataset("simple_corners.obj", "",
                "WoodF2F_simple_corners.pb",
                "", "", "",
                200.0, -20.0);

    // wood line 1729: simple_corners_combined
    // [0*3+2]=2; [1*3+2]=12; JOINT_VOLUME_EXTENSION[2]=-20
    run_dataset("simple_corners_combined.obj", "",
                "WoodF2F_simple_corners_combined.pb",
                "", "", "",
                200.0, -20.0);

    // wood line 1787: simple_corners_diff_lengths
    // [0,1]*3+{0,1,2}={140,0.5,1}; {140,0.5,10}; JOINT_VOLUME_EXTENSION[2]=-20
    run_dataset("simple_corners_diff_lengths.obj", "",
                "WoodF2F_simple_corners_diff_lengths.pb",
                "", "", "",
                140.0, -20.0);

    // wood line 1912: top_to_top_pairs
    // JOINT_VOLUME_EXTENSION[2]=-10
    run_dataset("top_to_top_pairs.obj", "",
                "WoodF2F_top_to_top_pairs.pb",
                "", "", "",
                200.0, -10.0);

    // wood line 2488: annen_box_pair
    run_dataset("annen_box_pair.obj", "",
                "WoodF2F_annen_box_pair.pb");

    // wood line 2698: annen_grid_small
    run_dataset("annen_grid_small.obj",
                "annen_grid_small_adjacency.txt",
                "WoodF2F_annen.pb",
                "annen_grid_small_three_valence.txt",
                "annen_grid_small_insertion_vectors.txt",
                "annen_grid_small_joints_types.txt");

    // wood line 3016: cross_corners
    // search_type=1 in wood; JOINT_VOLUME_EXTENSION[1]=2 → ext_vec={0,2,0}
    run_dataset("cross_corners.obj", "",
                "WoodF2F_cross_corners.pb",
                "", "", "",
                200.0, 0.0, {0, 2, 0});

    // wood line 3080: cross_vda_corner
    // search_type=1 in wood; JOINT_VOLUME_EXTENSION[1]=2 → ext_vec={0,2,0}
    run_dataset("cross_vda_corner.obj", "",
                "WoodF2F_cross_vda_corner.pb",
                "", "", "",
                200.0, 0.0, {0, 2, 0});

    return 0;
}
