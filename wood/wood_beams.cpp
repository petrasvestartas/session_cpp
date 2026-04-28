// wood/wood_beams.cpp — beam volumes pipeline.
// Implementation of beam_volumes_pipeline declared in wood_session.h.
// Takes beam axes and radii as input; no OBJ file I/O.
#include "wood_session.h"
#include "wood_element.h"
#include "wood_face_to_face.h"
#include "../src/session.h"
#include "../src/element.h"
#include "../src/intersection.h"
#include "../src/polyline.h"
#include "../src/line.h"
#include "../src/vector.h"
#include "../src/point.h"
#include "../src/xform.h"
#include "../src/plane.h"
#include "../src/tolerance.h"
#include "../src/color.h"
#include <fmt/core.h>
#include <cmath>
#include <algorithm>
#include <utility>
#include <vector>
#include <map>
#include <filesystem>

using namespace session_cpp;
using wood_session::WoodJoint;
using wood_session::WoodElement;

void beam_volumes_pipeline(
    const std::vector<Polyline>& axes,
    const std::vector<std::vector<double>>& segment_radii,
    const std::vector<std::vector<Vector>>& segment_direction,
    const std::vector<int>& allowed_types_per_polyline,
    double min_distance,
    double volume_length,
    double cross_or_side_to_end,
    int    flip_male)
{
    using namespace wood_session::globals;

    const std::string pb_name = DATA_SET_OUTPUT_FILE;
    auto base = internal::session_data_dir();

    Session session("WoodF2F");
    auto g_axes = session.add_group("BeamAxes");
    auto g_vols = session.add_group("JointVolumes");
    g_axes->color = Color(180,180,180,255,"grey");
    g_vols->color = Color(220, 80,180,255,"magenta");

    // Emit each input axis as ONE polyline (matches the OBJ `curv` entry).
    for (size_t i = 0; i < axes.size(); i++) {
        auto pl = std::make_shared<Polyline>(axes[i]);
        pl->name = fmt::format("axis_{}", i);
        session.add_polyline(pl, g_axes);
    }

    // ── Pass 1: collect closest per-element-pair contact ──────────────────
    // Wood uses CGAL::box_self_intersection_d on inflated AABBs. For the
    // small beam-count datasets (phanomema has 6 axes) a plain O(N²) scan
    // is adequate and keeps the port self-contained.
    struct Contact {
        double dist_sq;
        int pid0, sid0, pid1, sid1;
    };
    std::map<uint64_t, Contact> contacts;
    for (size_t a = 0; a < axes.size(); a++) {
        auto pa = axes[a].get_points();
        for (size_t sa = 0; sa + 1 < pa.size(); sa++) {
            Line la = Line::from_points(pa[sa], pa[sa+1]);
            for (size_t b = a + 1; b < axes.size(); b++) {
                auto pb = axes[b].get_points();
                for (size_t sb = 0; sb + 1 < pb.size(); sb++) {
                    Line lb = Line::from_points(pb[sb], pb[sb+1]);
                    double t0, t1;
                    (void)Intersection::line_line_parameters(
                        la, lb, t0, t1, /*tolerance*/ 0.0,
                        /*intersect_segments*/ true,
                        /*near_parallel_as_closest*/ true);
                    Point q0 = la.point_at(t0);
                    Point q1 = lb.point_at(t1);
                    double dx=q0[0]-q1[0], dy=q0[1]-q1[1], dz=q0[2]-q1[2];
                    double d2 = dx*dx + dy*dy + dz*dz;
                    if (d2 > min_distance*min_distance) continue;
                    uint64_t id = ((uint64_t)b << 32) | (uint64_t)a;
                    Contact c{d2, (int)a,(int)sa,(int)b,(int)sb};
                    auto it = contacts.find(id);
                    if (it == contacts.end() || d2 < it->second.dist_sq)
                        contacts[id] = c;
                }
            }
        }
    }

    int n_pairs = 0, n_success = 0, n_failed = 0;
    int counts[6] = {0,0,0,0,0,0};

    // Collected rectangles per successful contact, in the same order as wood's
    // `output_plines` (wood_test.cpp:3749-3752 → 4 rects per joint, each its
    // own `polyline_group`). Written to `_meta.txt`/`_coords.txt` after the
    // loop so the compare script can diff against wood's ref XML.
    std::vector<std::array<Polyline, 4>> joint_rects;
    joint_rects.reserve(contacts.size());

    // Detected joints + their axis-space contact points (p0, p1). Wood's
    // post-detection eccentricity-scaling step (`wood_main.cpp:2591-2630`)
    // uses distance(p0, p1) vs. beam radii to pre-scale joint.scale so the
    // unit-cube → world transform doesn't stretch teeth when the two beam
    // axes don't intersect exactly.
    std::vector<WoodJoint> all_joints;
    std::vector<std::array<Point, 2>> point_pairs;
    std::vector<std::array<int, 2>> axis_ids;  // (pid0, pid1) per joint
    all_joints.reserve(contacts.size());
    point_pairs.reserve(contacts.size());
    axis_ids.reserve(contacts.size());

    // ── Pass 2: per-contact rectangle generation + joint detection ────────
    for (auto& [id, c] : contacts) {
        (void)id;
        n_pairs++;
        auto pa_pts = axes[c.pid0].get_points();
        auto pb_pts = axes[c.pid1].get_points();
        Line s0 = Line::from_points(pa_pts[c.sid0], pa_pts[c.sid0+1]);
        Line s1 = Line::from_points(pb_pts[c.sid1], pb_pts[c.sid1+1]);

        Point p0, p1;
        Vector v0, v1, normal;
        bool type0=false, type1=false, is_parallel=false;
        bool ok = Intersection::line_line_classified(
            s0, s1,
            (int)(pa_pts.size() - 1), (int)(pb_pts.size() - 1),
            c.sid0, c.sid1,
            cross_or_side_to_end,
            p0, p1, v0, v1, normal,
            type0, type1, is_parallel);
        if (!ok) { n_failed++; continue; }

        // allowed_types filter: 0 = end-only (sum=0); 1 = cross/side-to-end
        // (sum=1 or 2); -1 = all. Same as wood_main.cpp:2392-2404.
        auto is_valid = [](int sum, int allowed) {
            switch (allowed) {
                case 0:  return sum == 0;
                case 1:  return sum == 1 || sum == 2;
                case -1: return true;
                default: return false;
            }
        };
        int sum = (int)type0 + (int)type1;
        if (!allowed_types_per_polyline.empty()) {
            if (allowed_types_per_polyline.size() == 1) {
                if (!is_valid(sum, allowed_types_per_polyline[0])) continue;
            } else if (allowed_types_per_polyline.size() == axes.size()) {
                if (!is_valid(sum, allowed_types_per_polyline[c.pid0]) ||
                    !is_valid(sum, allowed_types_per_polyline[c.pid1])) continue;
            }
        }

        // Rectangle generation. The prism reference z-axis is the caller-
        // supplied segment direction when present, else the contact normal.
        Vector sn0 = segment_direction.empty() ? normal : segment_direction[c.pid0][c.sid0];
        Vector sn1 = segment_direction.empty() ? normal : segment_direction[c.pid1][c.sid1];
        double r0 = segment_radii[c.pid0][c.sid0];
        double r1 = segment_radii[c.pid1][c.sid1];

        std::array<Polyline, 4> beam_vol;
        polyline_two_rects_from_frame(
            p0, v0, sn0, type0 == 1, r0, volume_length, flip_male,
            beam_vol[0], beam_vol[1]);
        polyline_two_rects_from_frame(
            p1, v1, sn1, type1 == 1, r1, volume_length, flip_male,
            beam_vol[2], beam_vol[3]);

        // Trim rectangles by bisecting planes (wood_main.cpp:2445-2555).
        std::array<Point, 4> ip;
        auto seg_plane = [](const Point& a, const Point& b, const Plane& pl, Point& out) {
            Line ln = Line::from_points(a, b);
            Intersection::line_plane(ln, pl, out, /*is_finite*/ false);
        };

        if (sum == 0) {
            Point pm((p0[0]+p1[0])*0.5, (p0[1]+p1[1])*0.5, (p0[2]+p1[2])*0.5);
            Point pm_plus_v0(pm[0]+v0[0], pm[1]+v0[1], pm[2]+v0[2]);
            Vector bisector = is_parallel ? v0 : Vector(v0[0]-v1[0], v0[1]-v1[1], v0[2]-v1[2]);
            Point  pm_cp = pm;
            Vector bis   = bisector;
            Plane  cp    = Plane::from_point_normal(pm_cp, bis);
            bool toward_v0 = !cp.has_on_negative_side(pm_plus_v0);
            Vector npos = cp.z_axis();
            Vector nneg(-npos[0], -npos[1], -npos[2]);
            Point pa_cp = pm;  Vector npa = toward_v0 ? npos : nneg;
            Point pb_cp = pm;  Vector npb = toward_v0 ? nneg : npos;
            Plane cut_plane0 = Plane::from_point_normal(pa_cp, npa);
            Plane cut_plane1 = Plane::from_point_normal(pb_cp, npb);
            for (int lid = 0; lid < 2; lid++) {
                int shift = lid == 0 ? 0 : 2;
                const Plane& cutpl = lid == 0 ? cut_plane0 : cut_plane1;
                auto p00 = beam_vol[0+shift].get_points();
                auto p01 = beam_vol[1+shift].get_points();
                seg_plane(p00[0], p00[1], cutpl, ip[0]);
                seg_plane(p00[3], p00[2], cutpl, ip[1]);
                seg_plane(p01[0], p01[1], cutpl, ip[2]);
                seg_plane(p01[3], p01[2], cutpl, ip[3]);
                if (cutpl.has_on_negative_side(p00[0])) {
                    p00[0] = ip[0]; p00[3] = ip[1]; p00[4] = p00[0];
                    p01[0] = ip[2]; p01[3] = ip[3]; p01[4] = p01[0];
                } else {
                    p00[1] = ip[0]; p00[2] = ip[1];
                    p01[1] = ip[2]; p01[2] = ip[3];
                }
                beam_vol[0+shift] = Polyline(p00);
                beam_vol[1+shift] = Polyline(p01);
            }
        } else if (sum == 1) {
            int closer_rect, farrer_rect;
            if (type0 == 0) {
                auto q20 = beam_vol[2].get_points()[0];
                auto q30 = beam_vol[3].get_points()[0];
                Point pp(p0[0]+v0[0], p0[1]+v0[1], p0[2]+v0[2]);
                bool closer = Point::distance(pp, q20) < Point::distance(pp, q30);
                closer_rect = closer ? 2 : 3;
                farrer_rect = closer ? 3 : 2;
            } else {
                auto q00 = beam_vol[0].get_points()[0];
                auto q10 = beam_vol[1].get_points()[0];
                Point pp(p1[0]+v1[0], p1[1]+v1[1], p1[2]+v1[2]);
                bool closer = Point::distance(pp, q00) < Point::distance(pp, q10);
                closer_rect = closer ? 0 : 1;
                farrer_rect = closer ? 1 : 0;
            }
            auto qc = beam_vol[closer_rect].get_points();
            Vector rv0(qc[1][0]-qc[0][0], qc[1][1]-qc[0][1], qc[1][2]-qc[0][2]);
            Vector rv1(qc[2][0]-qc[0][0], qc[2][1]-qc[0][1], qc[2][2]-qc[0][2]);
            Vector rnrm = rv0.cross(rv1);
            Point  rorig = qc[0];
            Plane  cutpl = Plane::from_point_normal(rorig, rnrm);
            auto qf = beam_vol[farrer_rect].get_points();
            if (!cutpl.has_on_negative_side(qf[0])) {
                Vector nneg(-rnrm[0], -rnrm[1], -rnrm[2]);
                Point  rorig2 = qc[0];
                cutpl = Plane::from_point_normal(rorig2, nneg);
            }
            int shift = type0 == 0 ? 0 : 2;
            auto p00 = beam_vol[0+shift].get_points();
            auto p01 = beam_vol[1+shift].get_points();
            seg_plane(p00[0], p00[1], cutpl, ip[0]);
            seg_plane(p00[3], p00[2], cutpl, ip[1]);
            seg_plane(p01[0], p01[1], cutpl, ip[2]);
            seg_plane(p01[3], p01[2], cutpl, ip[3]);
            if (cutpl.has_on_negative_side(p00[0])) {
                p00[0] = ip[0]; p00[3] = ip[1]; p00[4] = p00[0];
                p01[0] = ip[2]; p01[3] = ip[3]; p01[4] = p01[0];
            } else {
                p00[1] = ip[0]; p00[2] = ip[1];
                p01[1] = ip[2]; p01[2] = ip[3];
            }
            beam_vol[0+shift] = Polyline(p00);
            beam_vol[1+shift] = Polyline(p01);
        }
        // sum == 2 (cross): no trimming — both beams pass through.

        // Serialize the 4 (possibly trimmed) rectangles for viz.
        for (int k = 0; k < 4; k++) {
            auto rect = std::make_shared<Polyline>(beam_vol[k]);
            rect->name = fmt::format("beam_{}_{}_rect{}", c.pid0, c.pid1, k);
            session.add_polyline(rect, g_vols);
        }

        // Build wood elements from the rectangle pairs and run the existing
        // plate-style face_to_face detector. [0,1] = beam 0 top/bottom at
        // the joint; [2,3] = beam 1.
        WoodElement el0(beam_vol[0], beam_vol[1]);
        WoodElement el1(beam_vol[2], beam_vol[3]);

        WoodJoint jt;
        bool swap_planes_1 = false;
        bool jok = face_to_face_wood(
            (size_t)(n_success + n_failed),
            el0,
            el1,
            {c.pid0, c.pid1},
            JOINT_VOLUME_EXTENSION,
            0.0,
            1e-6,
            DISTANCE_SQUARED,
            FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_DIHEDRAL_ANGLE,
            FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ALL_TREATED_AS_ROTATED,
            FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ROTATED_JOINT_AS_AVERAGE,
            (sum == 2 ? 1 : 0),
            jt,
            swap_planes_1);
        if (!jok) { n_failed++; continue; }
        n_success++;
        int t = jt.joint_type;
        if      (t == 11) counts[0]++;
        else if (t == 12) counts[1]++;
        else if (t == 13) counts[2]++;
        else if (t == 20) counts[3]++;
        else if (t == 30) counts[4]++;
        else if (t == 40) counts[5]++;

        joint_rects.push_back(beam_vol);
        all_joints.push_back(std::move(jt));
        point_pairs.push_back({p0, p1});
        axis_ids.push_back({c.pid0, c.pid1});
    }

    // ── Eccentricity-scale joints. Ports wood_main.cpp:2591-2630.
    //   L = 0.5·|p0 - p1|  (half axis-to-axis distance at contact)
    //   r_max = 0.5·(r0 + r1)
    //   scale_value = cos(asin(1 - min((r_max - L)/r_max, 1)))  if L > 0.01
    //                else 1
    //   joint.scale = {scale_value, scale_value, 1}
    // Scale is consumed by joint_create_geometry's unit-cube transform; when
    // joints aren't dispatched it's still set for algorithm parity with wood.
    for (size_t i = 0; i < all_joints.size(); i++) {
        double dx = point_pairs[i][0][0] - point_pairs[i][1][0];
        double dy = point_pairs[i][0][1] - point_pairs[i][1][1];
        double dz = point_pairs[i][0][2] - point_pairs[i][1][2];
        double L = 0.5 * std::sqrt(dx*dx + dy*dy + dz*dz);
        double scale_value = 1.0;
        if (L > 0.01) {
            int p0_id = axis_ids[i][0];
            int p1_id = axis_ids[i][1];
            double r0 = segment_radii[p0_id].empty() ? 0.0 : segment_radii[p0_id][0];
            double r1 = segment_radii[p1_id].empty() ? 0.0 : segment_radii[p1_id][0];
            double max_r = 0.5 * (r0 + r1);
            if (max_r > 0.0) {
                double v = (max_r - L) / max_r;
                scale_value = std::cos(std::asin(1.0 - std::min(v, 1.0)));
            }
        }
        all_joints[i].scale[0] = scale_value;
        all_joints[i].scale[1] = scale_value;
        all_joints[i].scale[2] = 1.0;
    }

    // Meta + coords dump. Two sections:
    //   elements 0..N-1         = input beam axes (one element per axis,
    //                             one polyline per axis)
    //   elements N..N+4*J-1     = joint rectangles (one element per rect to
    //                             match wood's `output_plines` granularity
    //                             where each CGAL_Polyline gets its own
    //                             `polyline_group` — wood_test.cpp:3749-3752)
    {
        std::ofstream meta_out((base / (pb_name + "_meta.txt")).string());
        std::ofstream coord_out((base / (pb_name + "_coords.txt")).string());
        auto emit = [&](int ei, const Polyline& pl) {
            meta_out << 1 << ' ' << pl.point_count() << '\n';
            coord_out << "element " << ei << "\n";
            coord_out << "  poly 0:";
            for (size_t pi = 0; pi < pl.point_count(); pi++) {
                Point p = pl.get_point(pi);
                coord_out << " " << p[0] << " " << p[1] << " " << p[2];
            }
            coord_out << "\n";
        };
        int ei = 0;
        for (const auto& pl : axes) emit(ei++, pl);
        for (const auto& rects : joint_rects)
            for (int k = 0; k < 4; k++) emit(ei++, rects[k]);
    }

    session.pb_dump((base / pb_name).string());
    if (std::getenv("WOOD_VERBOSE")) {
        fmt::print("\n=== beam_volumes_pipeline ===\n");
        fmt::print("{} axes -> {} contacts -> {} volumes ({} failed)\n",
                   axes.size(), n_pairs, n_success, n_failed);
        fmt::print("  by type: 11={} 12={} 13={} 20={} 30={} 40={}\n",
                   counts[0], counts[1], counts[2], counts[3], counts[4], counts[5]);
    }
}

