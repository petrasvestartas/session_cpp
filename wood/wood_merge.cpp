// wood/wood_merge.cpp — merge joint cut outlines into element plate polylines.
// Implementation of merge_joints_for_element declared in wood_merge.h.
// No file I/O: operates purely on geometry passed by the caller.
#include "wood_merge.h"
#include "wood_session.h"
#include "wood_cut.h"
#include "../src/intersection.h"
#include "../src/polyline.h"
#include "../src/line.h"
#include "../src/vector.h"
#include "../src/point.h"
#include "../src/plane.h"
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

std::vector<session_cpp::Polyline> merge_joints_for_element(
    const WoodElement& el,
    const std::vector<std::vector<std::pair<int,bool>>>& el_jmf,
    std::vector<WoodJoint>& joints,
    int dbg_element_id = -1)
{
    // ── WOOD_MERGE_DUMP=<path> per-decision diagnostic dump ────────────────
    // Filtered by DIAG_TEST=<short_name> (matches DATA_SET_INPUT_NAME). Re-checked
    // per call because DATA_SET_INPUT_NAME changes between tests in a single
    // main_5 run. Opens in append mode so multi-element runs accumulate.
    std::ofstream* dbg_merge_log = nullptr;
    std::ofstream dbg_merge_owned;
    {
        const char* dpath = std::getenv("WOOD_MERGE_DUMP");
        const char* dfilter = std::getenv("DIAG_TEST");
        if (dpath && (!dfilter || wood_session::globals::DATA_SET_INPUT_NAME == dfilter)) {
            dbg_merge_owned.open(dpath, std::ios::app);
            if (dbg_merge_owned.is_open()) {
                dbg_merge_log = &dbg_merge_owned;
            }
        }
    }
    if (dbg_merge_log && dbg_element_id >= 0) {
        *dbg_merge_log << "ELEMENT " << dbg_element_id
                       << " planes0_o=(" << el.planes[0].origin()[0] << "," << el.planes[0].origin()[1] << "," << el.planes[0].origin()[2]
                       << ") planes0_n=(" << el.planes[0].z_axis()[0] << "," << el.planes[0].z_axis()[1] << "," << el.planes[0].z_axis()[2]
                       << ") planes1_o=(" << el.planes[1].origin()[0] << "," << el.planes[1].origin()[1] << "," << el.planes[1].origin()[2]
                       << ") planes1_n=(" << el.planes[1].z_axis()[0] << "," << el.planes[1].z_axis()[1] << "," << el.planes[1].z_axis()[2]
                       << ")\n";
        *dbg_merge_log << "  pline0 pts=" << el.polylines[0].point_count();
        for (size_t k = 0; k < el.polylines[0].point_count(); k++) {
            Point p = el.polylines[0].get_point(k);
            *dbg_merge_log << " (" << p[0] << "," << p[1] << "," << p[2] << ")";
        }
        *dbg_merge_log << "\n  pline1 pts=" << el.polylines[1].point_count();
        for (size_t k = 0; k < el.polylines[1].point_count(); k++) {
            Point p = el.polylines[1].get_point(k);
            *dbg_merge_log << " (" << p[0] << "," << p[1] << "," << p[2] << ")";
        }
        *dbg_merge_log << "\n";
    }

    // Polylines (mutable copies — wood relocates vertices in place).
    auto pline0 = el.polylines[0].get_points();
    auto pline1 = el.polylines[1].get_points();
    auto joint_planes = el.planes;     // mutable copy of side planes

    // Sort keys for the merge combine the integer edge id (scaled by 1e6)
    // with the sub-edge parametric fraction (scaled by 1e3). Wood uses the
    // same double-packed key scheme in wood_element.cpp:1015.
    const double scale_0 = 1000000.0;
    const double scale_1 = 1000.0;
    // Squared-distance threshold for "joint line lies on the original side
    // edge" checks. Read from `wood_session::globals` so per-test overrides
    // (e.g. `hexboxes` multiplies DISTANCE_SQUARED by 100) take effect.
    const double DISTANCE_SQUARED = wood_session::globals::DISTANCE_SQUARED;

    // Multimap (not map): when multiple joints produce identical cp_pair
    // keys on the same plate edge (e.g. rossiniere's rectangle joints whose
    // long edges exactly align with plate edges → cp = integer corner), a
    // std::map would silently drop all but the first insert. Multimap keeps
    // all entries, so Phase 3 emits every joint's clipped polyline.
    std::multimap<size_t, std::pair<std::pair<double,double>, Polyline>> sorted_by_id_plines_0;
    std::multimap<size_t, std::pair<std::pair<double,double>, Polyline>> sorted_by_id_plines_1;

    int last_id = -1;
    // Closing-corner trackers (wood_element.cpp:684-687). When the first and
    // last side edges BOTH carry a joint, wood overrides the merged polyline's
    // closing vertex with the 3D intersection of the first and last joint
    // lines — so the plate's corner meets at the joint-joint intersection
    // instead of the original polyline corner. Required for simple_corners
    // el 1/21/35 (100 mm z-tilt) and likely cross_corners el 9.
    std::array<Point, 2> last_segment0_start{{Point(0,0,0), Point(0,0,0)}};
    std::array<Point, 2> last_segment1_start{{Point(0,0,0), Point(0,0,0)}};
    std::array<Point, 2> last_segment0{{Point(0,0,0), Point(0,0,0)}};
    std::array<Point, 2> last_segment1{{Point(0,0,0), Point(0,0,0)}};

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
            if (dbg_merge_log) {
                *dbg_merge_log << "  J el=" << dbg_element_id << " i=" << i
                               << " jid=" << joint_id << " mf=" << (male_or_female?'M':'F')
                               << " jt=" << jt.joint_type
                               << " ep_top0=(" << ep_top0[0] << "," << ep_top0[1] << "," << ep_top0[2]
                               << ") ep_bot0=(" << ep_bot0[0] << "," << ep_bot0[1] << "," << ep_bot0[2]
                               << ") d_top=" << d_top << " d_bot=" << d_bot
                               << " reversed=" << (is_geo_reversed?1:0);
            }
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

            // Track segments for the closing-corner fix. wood_element.cpp:977-984.
            last_segment0 = {{j0_s, j0_e}};
            last_segment1 = {{j1_s, joint_line_1.get_point(1)}};
            if (i == 2) {
                last_segment0_start = last_segment0;
                last_segment1_start = last_segment1;
            }
            last_id = (int)i;

            // is_geo_flipped: reverse the joint outline if the data orientation
            // doesn't match the polygon walk direction. Wood's reference is
            // `(male_or_female, !is_geo_reversed)[0]`. After our swap above,
            // post-swap jm[0] already corresponds to that slot in BOTH cases
            // (is_geo_reversed=false → jm[0] is structural top = (male,true);
            //  is_geo_reversed=true  → swap put structural bottom into jm[0]
            //                          = (male,false) = (male,!true)).
            // So we select jm[0][0] directly. wood_element.cpp:1008-1015.
            //
            // Reverse when front is closer to pline0[id+1] than back is,
            // matching CGAL::has_smaller_distance_to_point(pline0[id+1],
            // front, back) → d_front_sq < d_back_sq.
            Polyline& flip_ref = jm[0][0];
            if (flip_ref.point_count() >= 1) {
                Point fr_front = flip_ref.get_point(0);
                Point fr_back  = flip_ref.get_point(flip_ref.point_count() - 1);
                Point ref_pt   = pline0[id + 1];
                double dx_f = fr_front[0]-ref_pt[0], dy_f = fr_front[1]-ref_pt[1], dz_f = fr_front[2]-ref_pt[2];
                double dx_b = fr_back[0]-ref_pt[0],  dy_b = fr_back[1]-ref_pt[1],  dz_b = fr_back[2]-ref_pt[2];
                double d_front_sq = dx_f*dx_f + dy_f*dy_f + dz_f*dz_f;
                double d_back_sq  = dx_b*dx_b + dy_b*dy_b + dz_b*dz_b;
                bool dbg_flipped = (d_front_sq < d_back_sq);
                if (dbg_merge_log) {
                    *dbg_merge_log << " fr_front=(" << fr_front[0] << "," << fr_front[1] << "," << fr_front[2]
                                   << ") fr_back=(" << fr_back[0] << "," << fr_back[1] << "," << fr_back[2]
                                   << ") ref=(" << ref_pt[0] << "," << ref_pt[1] << "," << ref_pt[2]
                                   << ") d_f=" << d_front_sq << " d_b=" << d_back_sq
                                   << " flipped=" << (dbg_flipped?1:0)
                                   << " jm0[0].first=(" << jm[0][0].get_point(0)[0] << "," << jm[0][0].get_point(0)[1] << "," << jm[0][0].get_point(0)[2] << ")"
                                   << " jm1[0].first=(" << jm[1][0].get_point(0)[0] << "," << jm[1][0].get_point(0)[1] << "," << jm[1][0].get_point(0)[2] << ")"
                                   << "\n";
                }
                if (dbg_flipped) {
                    jm[0][0].reverse();
                    jm[1][0].reverse();
                }
            } else if (dbg_merge_log) {
                *dbg_merge_log << " (flip_ref empty)\n";
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
                            std::multimap<size_t, std::pair<std::pair<double,double>, Polyline>>& sorted)
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
        // Only unflag the last point if it IS a closing duplicate (front==back).
        // The pipeline sometimes passes already-open polylines (front!=back), in
        // which case the last point is a real corner that must be kept.
        if (pline.size() > 1 &&
            std::abs(pline.front()[0] - pline.back()[0]) < 1e-9 &&
            std::abs(pline.front()[1] - pline.back()[1]) < 1e-9 &&
            std::abs(pline.front()[2] - pline.back()[2]) < 1e-9) {
            point_flags.back() = false;
        }

        // Wrap-around handling: if the LAST joint span ends near the start
        // of the polyline (`cp.second < 1` AND `cp.first > N-2`), the joint
        // wraps across the closing corner — so the original first vertex
        // must be unflagged or it becomes a duplicate of the merged-in
        // closing corner. wood_element.cpp:1209-1223.
        if (!sorted.empty() && !point_flags.empty()) {
            auto& last_cp = sorted.rbegin()->second.first;
            if (last_cp.first > (double)pline.size() - 2.0 && last_cp.second < 1.0) {
                point_flags[0] = false;
            }
        }

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

    // Closing-corner fix. wood_element.cpp:1242-1250. When the first AND
    // last side edges both had a joint (i.e. the loop wrapped around to the
    // starting edge), replace the merged polyline's FIRST point with the
    // 3D intersection of the first and last joint lines. Without this the
    // plate's closing corner stays at the ORIGINAL polyline vertex, while
    // wood's closes to the joint-joint intersection — source of the 100 mm
    // z-tilt on simple_corners el 1/21/35.
    if (last_id == (int)pline0.size() &&
        (last_segment0_start[0] - last_segment0_start[1]).magnitude_squared() > DISTANCE_SQUARED)
    {
        Line ls0_start = Line::from_points(last_segment0_start[0], last_segment0_start[1]);
        Line ls0       = Line::from_points(last_segment0[0],       last_segment0[1]);
        Line ls1_start = Line::from_points(last_segment1_start[0], last_segment1_start[1]);
        Line ls1       = Line::from_points(last_segment1[0],       last_segment1[1]);
        Point p0_close, p1_close;
        bool ok0 = Intersection::line_line_3d(ls0_start, ls0, p0_close);
        bool ok1 = Intersection::line_line_3d(ls1_start, ls1, p1_close);
        if (ok0 && ok1) {
            auto pts_top = merged_top.get_points();
            auto pts_bot = merged_bot.get_points();
            if (!pts_top.empty()) pts_top[0] = p0_close;
            if (!pts_bot.empty()) pts_bot[0] = p1_close;
            // The polylines are closed (front == back), so keep the duplicate in sync.
            if (pts_top.size() > 1) pts_top.back() = pts_top.front();
            if (pts_bot.size() > 1) pts_bot.back() = pts_bot.front();
            merged_top = Polyline(pts_top);
            merged_bot = Polyline(pts_bot);
        }
    }

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

            // Skip the LAST outline unconditionally — it's always the
            // bounding rectangle around all holes (wood_element.cpp:1331:
            // `for (k = 0; k < size - 1; ...)`). Wood's Phase A emits ALL
            // outlines (regardless of cut_type, including drill/mill/slice).
            // This is what makes vda_floor_0 emit tt_e_p_3 drill holes as
            // top-face outlines.
            size_t lim = (jm[0].size() > 1 ? jm[0].size() - 1 : 0);

            for (size_t kk = 0; kk < lim && kk < jm[1].size(); kk++) {
                Polyline top = jm[0][kk];
                Polyline bot = jm[1][kk];
                // Winding check uses planes[0] (the TOP plane) for BOTH holes,
                // matching wood_element.cpp:1329-1336. No guard on point count:
                // wood reverses 2-pt drill lines too because is_clockwise
                // returns false for degenerate num=0 case.
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

