// ═══════════════════════════════════════════════════════════════════════════
// wood/wood_main.cpp — wood joint detection pipeline orchestration.
//
// Public entry: get_connection_zones(ElementPlate[], Session&, SearchType)
//   Caller loads plates via internal::load_plates(name), then calls this
//   function, then calls session.pb_dump(path) to persist.
//
// Internal pipeline (run_connection_zones_pipeline):
//   1. Convert ElementPlates → WoodElements (build face planes, thickness).
//   2. BVH broad-phase to find candidate adjacent element pairs.
//   3. For every adjacent pair, call face_to_face_wood to classify the
//      joint (type 11/12/13/20/30/40) and compute area / lines / volumes.
//   4. Three-valence alignment or shadow-joint insertion (if tv file exists).
//   5. joint_create_geometry dispatch → unit-cube outlines from joint library.
//   6. joint_orient_to_connection_area → world-frame outlines.
//   7. merge_joints_for_element → cut outlines into plate polylines.
//   8. Add all geometry (plates, joints, merged cuts, meshes) to session.
//
// Globals (wood_globals.cpp / wood_session.h): tuning parameters read by the
// pipeline. Tests call reset_defaults() then override specific entries before
// calling get_connection_zones. Auxiliary per-dataset txt files (adjacency,
// three_valence, insertion_vectors, joints_types) are resolved from
// globals::DATA_SET_INPUT_NAME which load_plates() sets.
// ═══════════════════════════════════════════════════════════════════════════

#include "../src/session.h"
#include "../src/element.h"
#include "../src/intersection.h"
#include "../src/plane.h"
#include "../src/polyline.h"
#include "../src/line.h"
#include "../src/vector.h"
#include "../src/point.h"
#include "../src/xform.h"
#include "../src/tolerance.h"
#include "../src/aabb.h"
#include "../src/spatial_bvh.h"
#include "../src/mesh.h"
#include "wood_element.h"
#include "wood_face_to_face.h"
#include "wood_joint.h"
#include "wood_merge.h"
#include "wood_session.h"
#include <fmt/core.h>
#include <chrono>
#include <filesystem>
#include <vector>
#include <array>
#include <optional>
#include <utility>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <set>
#include <fstream>
#include <sstream>
#include <string>

using namespace session_cpp;

namespace {

using wood_session::WoodJoint;
using wood_session::WoodElement;
using wood_session::joint_orient_to_connection_area;
using wood_session::merge_linked_joints;
using wood_session::joint_get_divisions;
using wood_session::side_removal_ss_e_r_1_port;
using wood_session::tt_e_p_3;


// ───────────────────────────────────────────────────────────────────────────
// Joint library: all joint geometry constructors live in `wood_joint_lib.h`
// (the session equivalent of wood's `wood_joint_lib.cpp`). Adding a new
// joint variant means editing that header AND wiring it into the dispatcher
// in `joint_create_geometry` further down this file.
// ───────────────────────────────────────────────────────────────────────────
#include "wood_joint_lib.h"


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
                                  std::vector<WoodJoint>* all_joints = nullptr,
                                  const std::vector<struct WoodElement>* elements = nullptr) {
    joint_get_divisions(joint, division_distance);
    joint.shift = shift_param;

    if (id == 0) return; // already filtered upstream; defensive only

    // Type-id compatibility check (wood_joint_lib.cpp:6109-6140). Wood's
    // group assignment requires id to fall in the type's range OR be -1.
    // If id is present but incompatible with the joint's detected type,
    // wood falls through to empty-joint detection and skips emission.
    // Mirror by returning early without setting any outlines.
    auto id_matches_type = [](int t, int jid) -> bool {
        if (jid == -1) return true;
        switch (t) {
            case 11: return jid >= 10 && jid <= 19;
            case 12: return jid >= 1  && jid <= 9;
            case 13: return jid >= 50 && jid <= 59;
            case 20: return jid >= 20 && jid <= 29;
            case 30: return jid >= 30 && jid <= 39;
            case 40: return jid >= 40 && jid <= 49;
            case 60: return jid >= 60 && jid <= 69;
        }
        return false;
    };
    if (!id_matches_type(joint.joint_type, id)) return;

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

    // Warn-once per unique id when an unimplemented case is reached. The
    // constructor is still called (falls through to the group's default)
    // so behaviour is unchanged — but the first encounter is now visible
    // in the log instead of silently producing wrong geometry.
    static std::set<int> warned_ids;
    auto warn_unimpl = [&](const char* family) {
        if (warned_ids.insert(id).second)
            fmt::print("joint_create_geometry: id={} ({}) not ported, using family default\n",
                       id, family);
    };

    switch (group) {
        // ── ss_e_ip (side-side in-plane, type 12) ─ wood_joint_lib.cpp:6290-6332
        case 0:
            switch (id) {
                case 1: ss_e_ip_1(joint); break;
                case 2: ss_e_ip_0(joint); break;
                case 3: ss_e_ip_2(joint); break;
                case 4: ss_e_ip_3(joint); break;
                case 5: ss_e_ip_4(joint); break;
                // Not ported: 6 (ss_e_ip_5, elements catalog),
                //             8 (side_removal, pre-detection face removal),
                //             9 (ss_e_ip_custom, XML loader).
                default: warn_unimpl("ss_e_ip"); ss_e_ip_1(joint); break;
            }
            break;

        // ── ss_e_op (side-side out-of-plane, type 11) ─ wood_joint_lib.cpp:6334-6392
        case 1:
            switch (id) {
                case 10: ss_e_op_1(joint); break;
                case 11: ss_e_op_2(joint); break;
                case 12: ss_e_op_0(joint); break;
                case 13: ss_e_op_3(joint); break;
                case 14: ss_e_op_4(joint, 0.0, true); break;
                case 15:
                    if (all_joints) ss_e_op_5(joint, *all_joints, false);
                    else            ss_e_op_4(joint);
                    break;
                case 16:
                    if (all_joints) ss_e_op_5(joint, *all_joints, true);
                    else            ss_e_op_4(joint);
                    break;
                case 18: ss_e_op_tutorial(joint); break;
                default:
                    // Wood falls to ss_e_op_1 by default (wood_joint_lib.cpp:6387).
                    // Session's legacy sentinel `id=-1` (no JOINTS_TYPES file,
                    // vidy-style) keeps the shadow-joint-linking path alive.
                    if (id < 0) {
                        if (all_joints) ss_e_op_5(joint, *all_joints, false);
                        else            ss_e_op_4(joint);
                    } else {
                        warn_unimpl("ss_e_op");
                        ss_e_op_1(joint);
                    }
                    break;
            }
            break;

        // ── ts_e_p (top-side, type 20) ─ wood_joint_lib.cpp:6395-6448
        case 2:
            switch (id) {
                case 20: ts_e_p_3(joint); break;   // wood default
                case 21: ts_e_p_2(joint); break;
                case 22: ts_e_p_3(joint); break;   // wood also routes 22 → ts_e_p_3
                case 23: ts_e_p_0(joint); break;
                case 25: ts_e_p_5(joint); break;
                // Not ported: 24 (ts_e_p_4, 458-line chamfer/extension).
                default: warn_unimpl("ts_e_p"); ts_e_p_3(joint); break;
            }
            break;

        // ── cr_c_ip (cross, type 30) ─ wood_joint_lib.cpp:6450-6502
        case 3:
            switch (id) {
                case 30: cr_c_ip_0(joint); break;
                case 31: cr_c_ip_1(joint); break;
                case 32: cr_c_ip_2(joint); break;
                case 33: cr_c_ip_3(joint); break;
                case 34: cr_c_ip_4(joint); break;
                case 35: cr_c_ip_5(joint); break;
                default: warn_unimpl("cr_c_ip"); cr_c_ip_0(joint); break;
            }
            break;

        // ── tt_e_p (top-top, type 40) ─ wood_joint_lib.cpp:6490-6523
        case 4:
            switch (id) {
                case 43:
                    if (elements) tt_e_p_3(joint, *elements);
                    break;
                // Not ported: 40-42, 44-48. Falls through to unwired-group
                // default (empty geometry, joint drops from merge).
                default: warn_unimpl("tt_e_p"); break;
            }
            break;

        // ── ss_e_r (side-side rotated, type 13) ─ wood_joint_lib.cpp:6525-6553
        case 5:
            switch (id) {
                case 54: ss_e_r_3(joint); break;
                case 55: ss_e_r_2(joint); break;
                case 56: ss_e_r_0(joint); break;
                case 58:
                    if (elements) side_removal_ss_e_r_1_port(joint, *elements);
                    else          ss_e_r_0(joint);
                    break;
                default: warn_unimpl("ss_e_r"); ss_e_r_0(joint); break;
            }
            break;

        // Group 6 (b, needs joint.scale field + slice cut type) is not yet
        // wired. Falling back to topology-based default keeps the joint
        // available to the merge — constructors that emit no outlines
        // simply drop out.
        default:
            warn_unimpl("unwired-group");
            switch (joint.joint_type) {
                case 11: case 12: ss_e_op_1(joint); break;
                case 20:          ts_e_p_3(joint);  break;
                default: break;
            }
            break;
    }
}

// side_removal_ss_e_r_1 — simplified port of wood_joint_lib.cpp:2723-3100.
// Wood's full function swaps v0/v1 + joint_lines/volumes, extends side-face
// corners by convex-corner check, offsets by plate-plane normals, and builds
// a pline0_moved/pline1_moved set as outlines. We skip the optional
// `shift>0 && merge_with_joint` branch (clipper offset + conic cut; would
// require porting `clipper_util::offset_in_3d`, `get_intersection_between_two_polylines`
// boolean, and `ss_e_r_1` unit geometry — ~1000 lines total). That branch
// only adds extra conic/mill cuts; the base mill_project cuts we emit here
// should match wood's primary plate-side removal.
//
// joint.el_ids: (v0, v1) with f0_0 and f1_0 the side-face indices.
// wood_elems[v0].polylines[f0_0] is the 5-pt side rectangle on plate v0.
static void three_valence_joint_addition_vidy(
    const std::vector<std::vector<int>>& tv_groups,
    std::vector<WoodElement>& elements,
    std::vector<WoodJoint>& joints,
    std::unordered_map<uint64_t, int>& joints_map,
    const std::vector<std::pair<int,int>>& /*adjacency_pairs*/)
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
    const std::vector<std::pair<int,int>>& /*adjacency_pairs*/)
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

} // anonymous namespace

//  definitions live in wood_globals.cpp.
//  helpers (session_data_dir, load_plates, plates_exist) live in
// wood_internal.cpp. Declarations for both live in wood_session.h.

// Internal pipeline — takes pre-loaded polylines (bottom at even, top at odd index).
// Adds plates, joints, merged outlines, and loft meshes to `session`.
// Reads dataset auxiliary files (adjacency, tv, iv, jt) via DATA_SET_INPUT_NAME global.
// Returns merged plate outline polylines per element for lofting by the caller.
static std::vector<std::vector<Polyline>> run_connection_zones_pipeline(
    const std::vector<Polyline>& polylines,
    Session& session,
    SearchType search_type) {

    using namespace wood_session::globals;
    const std::string short_name      = DATA_SET_INPUT_NAME;
    const std::string pb_name         = DATA_SET_OUTPUT_FILE;
    const double      dihedral_threshold = FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_DIHEDRAL_ANGLE;

    using Clock = std::chrono::high_resolution_clock;
    auto base = internal::session_data_dir();
    auto t0   = Clock::now();

    auto exists_in_data = [&](const std::string& rel) {
        return std::filesystem::exists(base / rel);
    };
    const std::string adj_name = exists_in_data(short_name + "_adjacency.txt")         ? short_name + "_adjacency.txt"         : "";
    const std::string tv_name  = exists_in_data(short_name + "_three_valence.txt")     ? short_name + "_three_valence.txt"     : "";
    const std::string iv_name  = exists_in_data(short_name + "_insertion_vectors.txt") ? short_name + "_insertion_vectors.txt" : "";
    const std::string jt_name  = exists_in_data(short_name + "_joints_types.txt")      ? short_name + "_joints_types.txt"      : "";
    const std::vector<double> ext_vec = JOINT_VOLUME_EXTENSION;
    const bool verbose = std::getenv("WOOD_VERBOSE") != nullptr;

    if (verbose) fmt::print("\n=== {}.obj ===\n", short_name);

    std::vector<std::pair<int,int>> pairs;
    for (size_t i = 0; i + 1 < polylines.size(); i += 2)
        pairs.emplace_back(static_cast<int>(i), static_cast<int>(i + 1));

    // 2. Build wood-compatible elements.
    std::vector<WoodElement> wood_elems;
    wood_elems.reserve(pairs.size());
    for (auto [a, b] : pairs)
        wood_elems.push_back(build_wood_element(
            polylines[a].get_points(), polylines[b].get_points()));

    // ── WOOD_EL_DUMP=<path> dump element polylines for comparison with wood ──
    if (const char* ep = std::getenv("WOOD_EL_DUMP")) {
        const char* df = std::getenv("DIAG_TEST");
        if (!df || wood_session::globals::DATA_SET_INPUT_NAME == df) {
            std::ofstream el_log(ep);
            for (size_t ei = 0; ei < wood_elems.size(); ei++) {
                const auto& we = wood_elems[ei];
                el_log << "ELEMENT " << ei << " reversed=" << (we.reversed ? 1 : 0)
                       << " thickness=" << we.thickness
                       << " n_polylines=" << we.polylines.size() << "\n";
                for (size_t pi = 0; pi < we.polylines.size(); pi++) {
                    el_log << "  poly[" << pi << "] pts=" << we.polylines[pi].point_count() << ":";
                    for (size_t k = 0; k < we.polylines[pi].point_count(); k++) {
                        Point p = we.polylines[pi].get_point(k);
                        el_log << " (" << p[0] << "," << p[1] << "," << p[2] << ")";
                    }
                    el_log << "\n";
                }
            }
        }
    }

    // ElementPlates for session storage + OBB adjacency.
    auto g = session.add_group("Elements");
    std::vector<std::shared_ptr<ElementPlate>> plates;
    plates.reserve(pairs.size());
    for (auto [a, b] : pairs) {
        auto plate = std::make_shared<ElementPlate>(
            polylines[a], polylines[b], "plate_" + std::to_string(a));
        session.add_element(plate, g);
        plates.push_back(plate);
    }
    // (Raw input-plate boxes were previously added as a "PlateMeshes" group
    // for early debugging. They overlap the merged loft output visually and
    // have the same "plate_N" names, so they're dropped here. The authoritative
    // plate geometry is the merged loft emitted by loft_merged_elements.)
    auto t1 = Clock::now();

    // 3. Adjacency: load from file if provided, otherwise OBB+BVH search.
    std::vector<std::pair<int, int>> adjacency_pairs;
    if (!adj_name.empty()) {
        std::ifstream adj_in((base / adj_name).string());
        int a, b;
        while (adj_in >> a >> b) adjacency_pairs.emplace_back(a, b);
        if (verbose) fmt::print("adjacency: {} pairs from {}\n", adjacency_pairs.size(), adj_name);
    }
    if (adjacency_pairs.empty()) {
        std::vector<Element*> elem_ptrs(plates.size());
        for (size_t k = 0; k < plates.size(); k++) elem_ptrs[k] = plates[k].get();
        // Inflation matches wood's AABB expansion at wood_main.cpp:58-63:
        // DISTANCE = 0.1 mm per side.
        auto adj_flat = Intersection::adjacency_search(elem_ptrs, 0.1);
        for (size_t i = 0; i + 3 < adj_flat.size(); i += 4)
            adjacency_pairs.emplace_back(adj_flat[i], adj_flat[i + 1]);
        if (verbose) fmt::print("adjacency: {} pairs from OBB+BVH\n", adjacency_pairs.size());
    }
    auto t2 = Clock::now();

    // 4. Define ALL wood-joint detection parameters as LOCAL variables.
    //    No globals, no config struct — every tunable is right here so the
    //    caller can see what knobs the algorithm exposes. The names match
    //    the original wood::GLOBALS::* fields one-for-one.
    const std::vector<double>& joint_volume_extension = ext_vec;
    const double limit_min_joint_length   = 0.0;              // wood GLOBALS::LIMIT_MIN_JOINT_LENGTH
    const double distance_squared         = 1e-6;             // minimum joint-line squared length
    const double coplanar_tolerance       = DISTANCE_SQUARED; // squared-distance coplanar test
    const double dihedral_angle_threshold = dihedral_threshold;
    const bool   all_treated_as_rotated   = FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ALL_TREATED_AS_ROTATED;
    const bool   rotated_joint_as_average = FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ROTATED_JOINT_AS_AVERAGE;

    // Sync cross-joint near-coplanar threshold with current session globals.
    // Hexboxes multiplies DISTANCE_SQUARED by 100; the cross-joint detector
    // must honour the test-time value.
    wood_session::set_cross_joint_distance_squared(DISTANCE_SQUARED);

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
        std::ifstream iv_in((base / iv_name).string());
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
        if (verbose) fmt::print("insertion_vectors: {} vectors across {} elements from {}\n",
                               total_loaded, ei, iv_name);
    }
    // Move insertion vectors into WoodElement so face_to_face_wood has one object per plate.
    for (size_t ei = 0; ei < wood_elems.size(); ei++) {
        wood_elems[ei].insertion_vectors = per_element_insertion_vectors[ei];
        if (wood_elems[ei].reversed) {
            auto& vecs = wood_elems[ei].insertion_vectors;
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

        WoodJoint joint;
        bool swap_planes_b = false;
        bool ok = face_to_face_wood(
            k,
            wood_elems[ia],
            wood_elems[ib],
            {ia, ib},
            joint_volume_extension,
            limit_min_joint_length,
            distance_squared,
            coplanar_tolerance,
            dihedral_angle_threshold,
            all_treated_as_rotated,
            rotated_joint_as_average,
            search_type,
            joint,
            swap_planes_b);
        if (swap_planes_b) {
            std::swap(wood_elems[ib].planes[0], wood_elems[ib].planes[1]);
            std::swap(wood_elems[ib].polylines[0], wood_elems[ib].polylines[1]);
        }
        if (!ok) {
            if (verbose && !joint.dbg_fail_reason.empty())
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

    // NO sort here: wood's construct_joint_by_index iterates joints in the
    // order they were generated (adjacency-pair order). The unique_joints
    // cache depends on iteration order — the FIRST joint with a given
    // (name, shift, divisions) key determines the edge_length used by all
    // subsequent joints with matching key. Sorting here would change which
    // joint gets cached first and make session diverge from wood.


    // Three-valence joint alignment (Annen method).
    if (!tv_name.empty()) {
        std::vector<std::vector<int>> tv_groups;
        std::ifstream tv_in((base / tv_name).string());
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
                if (verbose) fmt::print("vidy_addition: {} shadow joints created (total {})\n",
                                       all_joints.size() - before_vidy, all_joints.size());
            } else {
                // Annen alignment: shorten overlapping joint lines (instruction == 0 only).
                three_valence_joint_alignment_annen(tv_groups, wood_elems, all_joints, adjacency_pairs);
            }
        }
        if (verbose) fmt::print("three_valence: {} groups applied\n", tv_groups.size());
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
        std::ifstream jt_in((base / jt_name).string());
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
        if (verbose) fmt::print("joints_types: {} ids across {} elements from {}\n",
                               total_loaded, ei, jt_name);
    }

    // Create unit joinery + orient to connection area.
    // Per-type div_dist + shift, mirroring wood's
    // GLOBALS::JOINTS_PARAMETERS_AND_TYPES table for the annen test
    // (wood_test.cpp:2720-2730 + wood_globals.cpp:49-54).
    //
    // Wood caches unit-cube joint geometry by key (name, shift, divisions)
    // — NOT including edge_length. Subsequent joints with matching key COPY
    // the cached geometry instead of recomputing. This means all joints with
    // the same key end up sharing the FIRST joint's edge_length-dependent
    // tooth positions. See wood_joint_lib.cpp:6262-6294 + wood_joint.cpp:86
    // (get_key formatting).
    struct CachedJointGeom {
        std::string name;
        std::array<std::vector<Polyline>, 2> m_outlines;
        std::array<std::vector<Polyline>, 2> f_outlines;
        std::array<std::vector<int>, 2> m_cut_types;
        std::array<std::vector<int>, 2> f_cut_types;
        bool unit_scale;
        double unit_scale_distance;
    };
    std::map<std::string, CachedJointGeom> unique_joints_cache;
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

        // Per-type parameter lookup into wood_session::globals::JOINTS_PARAMETERS_AND_TYPES.
        // Wood's dispatcher (wood_joint_lib.cpp:6191-6196) reads
        // `default_parameters_for_four_types[group*3+{0,1,2}]` where group is
        // derived from joint_type:
        //   11 → 1 (ss_e_op)   12 → 0 (ss_e_ip)   13 → 5 (ss_e_r)
        //   20 → 2 (ts_e_p)    30 → 3 (cr_c_ip)   40 → 4 (tt_e_p)   60 → 6 (b)
        // Reading from globals (rather than hardcoded 300/0.5/450 or shim args)
        // means per-test overrides like `JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 25`
        // for top_to_side_box actually take effect.
        auto row_for_type = [](int t) -> int {
            switch (t) {
                case 11: return 1;
                case 12: return 0;
                case 13: return 5;
                case 20: return 2;
                case 30: return 3;
                case 40: return 4;
                case 60: return 6;
                default: return 1;
            }
        };
        const auto& JPT = wood_session::globals::JOINTS_PARAMETERS_AND_TYPES;
        int row = row_for_type(j.joint_type);

        // id_representing_joint_name fallback: when no JOINTS_TYPES file
        // contributed a per-face id, read the default id from the per-type
        // row (col 2). Wood defaults:  ss_e_ip=3, ss_e_op=15, ts_e_p=20,
        // cr_c_ip=30, tt_e_p=40, ss_e_r=58, b=60.
        if (id_representing_joint_name == -1) {
            id_representing_joint_name = (int)JPT[row*3 + 2];
        }

        if (j.link) continue; // shadow joints: geometry set by ss_e_op_5, orient+merge below

        double div_dist  = JPT[row*3 + 0];
        double shift_val = JPT[row*3 + 1];
        if (j.joint_type == 13 || j.joint_type == 12) {
            // Pre-set element thickness for ss_e_r_2/3 (type 13) and
            // ss_e_ip_2 (type 12, butterfly). Wood's ss_e_ip_2 (line 765)
            // and ss_e_r_2/3 both use `joint.unit_scale_distance =
            // elements[joint.v0].thickness` as `joint_volume_edge_length`
            // for the division formula. Without this pre-set, session uses
            // the hardcoded default of 40mm and teeth land off-position.
            int ei = j.el_ids.first;
            if (ei >= 0 && ei < (int)wood_elems.size())
                j.unit_scale_distance = wood_elems[ei].thickness;
        }

        // Compute divisions first so the cache key matches wood's get_key().
        joint_get_divisions(j, div_dist);
        j.shift = shift_val;

        // Cache key: use the id as a proxy for `name` (id→constructor is
        // deterministic). Wood formats shift/divisions with `std::to_string`
        // truncated at 2 decimals; we mirror that exactly.
        auto fmt_key_num = [](double v) -> std::string {
            v += 1e-9;
            std::string s = std::to_string(v);
            auto dot = s.find('.');
            if (dot != std::string::npos && dot + 3 <= s.size())
                return s.substr(0, dot + 3);
            return s;
        };
        std::string cache_key = std::to_string(id_representing_joint_name)
                              + ";" + fmt_key_num(j.shift)
                              + ";" + fmt_key_num((double)j.divisions);

        // Only apply caching for type-12 (ss_e_ip, butterflies) — wood caches
        // all joint types, but session's implementation diverges enough in
        // downstream handling (Phase B/C hole extraction, orient quirks) that
        // enabling cache for other types causes regressions in datasets like
        // top_to_side_box and vda_floor_0.
        bool use_cache = (j.joint_type == 12) && j.linked_joints.empty();

        auto cache_it = use_cache ? unique_joints_cache.find(cache_key)
                                   : unique_joints_cache.end();
        if (!use_cache) {
            joint_create_geometry(j, div_dist, shift_val, id_representing_joint_name, &all_joints, &wood_elems);
        } else if (cache_it != unique_joints_cache.end()) {
            // Cache hit: transfer unit-cube geometry, skip recompute.
            const auto& u = cache_it->second;
            j.name = u.name;
            j.m_outlines = u.m_outlines;
            j.f_outlines = u.f_outlines;
            j.m_cut_types = u.m_cut_types;
            j.f_cut_types = u.f_cut_types;
            j.unit_scale = u.unit_scale;
            j.unit_scale_distance = u.unit_scale_distance;
        } else {
            // Cache miss: compute geometry and store pre-orient state.
            joint_create_geometry(j, div_dist, shift_val, id_representing_joint_name, &all_joints, &wood_elems);
            CachedJointGeom u;
            u.name = j.name;
            u.m_outlines = j.m_outlines;
            u.f_outlines = j.f_outlines;
            u.m_cut_types = j.m_cut_types;
            u.f_cut_types = j.f_cut_types;
            u.unit_scale = j.unit_scale;
            u.unit_scale_distance = j.unit_scale_distance;
            unique_joints_cache.emplace(cache_key, std::move(u));
        }
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

    // ── DEBUG DUMP (WOOD_DUMP=<path>, DIAG_TEST=<short_name>) ─────────────
    // Mirror the wood-side dump in `wood/cmake/src/wood/include/wood_main.cpp`.
    // Emits per-joint: joint_volumes, oriented m_outlines, f_outlines. Lets
    // us line-by-line diff wood↔session state for a failing dataset. DIAG_TEST
    // filters so only the targeted dataset's joints are written (otherwise the
    // last-run test overwrites the dump).
    const char* diag_filter = std::getenv("DIAG_TEST");
    const char* dump_path = std::getenv("WOOD_DUMP");
    const bool should_dump = dump_path && (!diag_filter ||
                              wood_session::globals::DATA_SET_INPUT_NAME == diag_filter);
    if (should_dump) {
        std::ofstream df(dump_path);
        if (df) {
            df << "# session joints after joint_create_geometry + orient\n";
            df << "# count=" << all_joints.size() << "\n";
            for (size_t ji = 0; ji < all_joints.size(); ji++) {
                const auto& j = all_joints[ji];
                df << "joint " << ji << " type=" << j.joint_type
                   << " v0=" << j.el_ids.first << " v1=" << j.el_ids.second
                   << " f0_0=" << j.face_ids.first[0] << " f1_0=" << j.face_ids.second[0]
                   << " name=" << (j.name.empty() ? "undefined" : j.name)
                   << " orient=" << (j.no_orient ? 0 : 1)
                   << " div=" << j.divisions
                   << " shift=" << j.shift << "\n";
                auto dump_pl = [&](const char* tag, const Polyline& pl) {
                    df << "  " << tag << " pts=" << pl.point_count();
                    for (size_t k = 0; k < pl.point_count(); k++) {
                        Point p = pl.get_point(k);
                        df << " (" << p[0] << "," << p[1] << "," << p[2] << ")";
                    }
                    df << "\n";
                };
                for (int k = 0; k < 4; k++)
                    if (j.joint_volumes_pair_a_pair_b[k].has_value())
                        { char tag[32]; snprintf(tag, sizeof(tag), "jv[%d]", k);
                          dump_pl(tag, *j.joint_volumes_pair_a_pair_b[k]); }
                for (int face = 0; face < 2; face++) {
                    for (size_t k = 0; k < j.m_outlines[face].size(); k++)
                        { char tag[32]; snprintf(tag, sizeof(tag), "m[%d][%zu]", face, k);
                          dump_pl(tag, j.m_outlines[face][k]); }
                    for (size_t k = 0; k < j.f_outlines[face].size(); k++)
                        { char tag[32]; snprintf(tag, sizeof(tag), "f[%d][%zu]", face, k);
                          dump_pl(tag, j.f_outlines[face][k]); }
                }
            }
            fmt::print("[session_dump] wrote {} joints to {}\n", all_joints.size(), dump_path);
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
    std::ofstream meta_out((base / (std::string(pb_name) + "_meta.txt")).string());
    std::ofstream coord_out((base / (std::string(pb_name) + "_coords.txt")).string());
    std::vector<std::vector<Polyline>> merged_cache(n_elems);
    for (size_t ei = 0; ei < n_elems; ei++) {
        auto merged = merge_joints_for_element(wood_elems[ei], j_mf[ei], all_joints, (int)ei);
        merged_cache[ei] = merged;
        auto g_el = session.add_group(fmt::format("element_{}", ei));
        g_el->color = col_merged;
        for (size_t mi = 0; mi < merged.size(); mi++) {
            auto mpl = std::make_shared<Polyline>(merged[mi]);
            mpl->name = fmt::format("merged_{}_{}", ei, mi);
            mpl->linecolor = col_merged;
            session.add_polyline(mpl, g_el);
        }
        // Wood output_type=3: per-element joint outlines ("cuts"). Wood
        // convention (see wood_joint.cpp:32-43 `operator()`):
        //   male_or_female = (v0 == this_element_id) ? 0 : 1
        //   male_or_female == 0 → return f_outlines (this element is v0 →
        //                          cuts FEMALE shape into itself)
        //   male_or_female == 1 → return m_outlines (this element is v1 →
        //                          cuts MALE shape)
        // This means v0 is the FEMALE-cut side, v1 is the MALE-cut side.
        // Skips joints already integrated into the plate polyline by the
        // merge (2- or 5-point endpoint markers handled by case 2/5).
        for (size_t fi = 0; fi < j_mf[ei].size(); fi++) {
            for (size_t k = 0; k < j_mf[ei][fi].size(); k++) {
                int joint_id = j_mf[ei][fi][k].first;
                if (joint_id < 0 || joint_id >= (int)all_joints.size()) continue;
                auto& jt = all_joints[joint_id];
                size_t male_or_female = (jt.el_ids.first == (int)ei) ? 0 : 1;
                auto& outlines_cut = male_or_female ? jt.m_outlines : jt.f_outlines;
                // Skip if the merge already handled this joint.
                if (outlines_cut[0].size() < 2) continue;
                size_t marker_sz = outlines_cut[0][1].point_count();
                if (marker_sz == 2 || marker_sz == 5) continue;
                for (int face = 0; face < 2; face++) {
                    if (face < (int)outlines_cut.size() && !outlines_cut[face].empty()) {
                        auto pl = std::make_shared<Polyline>(outlines_cut[face][0]);
                        pl->name = fmt::format("jcut_{}_{}", ei, joint_id);
                        pl->linecolor = col_merged;
                        session.add_polyline(pl, g_el);
                    }
                }
            }
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
        // (Per-element cut outlines are emitted above in the element loop,
        // mirroring wood's output_type=3 semantics; no longer duplicated here.)
    }

    auto t4 = Clock::now();

    auto ms = [](auto a, auto b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    if (verbose) {
        fmt::print("{} polylines -> {} elements -> {} adjacency pairs\n",
                   polylines.size(), pairs.size(), adjacency_pairs.size());
        fmt::print("  joints: {} success / {} failed\n", n_success, n_failed);
        fmt::print("  by type: 11={} 12={} 13={} 20={} 30={} 40={}\n",
                   counts[0], counts[1], counts[2], counts[3], counts[4], counts[5]);
        fmt::print("  time: {:.0f}ms\n", ms(t0, t4));
    }
    return merged_cache;
}

// merged layout per element: [hole0_top, hole0_bot, ..., outer_top, outer_bot]
void loft_merged_elements(
    Session& session,
    const std::vector<std::vector<Polyline>>& mc)
{
    auto g = session.add_group("MergedMeshes");
    for (size_t ei = 0; ei < mc.size(); ei++) {
        const auto& merged = mc[ei];
        if (merged.size() < 2) continue;
        std::vector<Polyline> tops;
        std::vector<Polyline> bots;
        tops.push_back(merged[merged.size() - 2]);
        bots.push_back(merged[merged.size() - 1]);
        for (size_t hi = 0; hi + 2 <= merged.size() - 2; hi += 2) {
            tops.push_back(merged[hi]);
            bots.push_back(merged[hi + 1]);
        }
        auto plate = std::make_shared<Mesh>(Mesh::loft(tops, bots));
        plate->name = fmt::format("plate_{}", ei);
        session.add_mesh(plate, g);
    }
}

// Pipeline entry: run full 9-stage joint detection on a set of timber plates.
// elements    — input plates (ElementPlate from session kernel); loaded from a
//               named dataset via internal::load_plates() or constructed directly.
// session     — caller-owned; plates, joint volumes, merged outlines, and loft
//               meshes are appended. Caller calls session.pb_dump(path) to persist.
// search_type — face_to_face | cross_joint | face_to_face_then_cross
// Returns: merged plate outline polylines per element
//          [element_id][outline_id]: [hole0_top, hole0_bot, ..., outer_top, outer_bot]
std::vector<std::vector<Polyline>> get_connection_zones(
    const std::vector<ElementPlate>& elements,
    Session& session,
    SearchType search_type)
{
    // Convert ElementPlate array to flat polyline list expected by the internal
    // pipeline (bottom polyline at even index, top polyline at odd index per element).
    std::vector<Polyline> polylines;
    polylines.reserve(elements.size() * 2);
    for (const auto& e : elements) {
        polylines.emplace_back(e.polygon());
        polylines.emplace_back(e.polygon_top());
    }
    auto mc = run_connection_zones_pipeline(polylines, session, search_type);
    return mc;
}

