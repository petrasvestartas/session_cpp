// wood/wood_joint.cpp — joint orientation, linking, and element-aware constructors.
// Implementations of the functions declared in wood_joint.h.
#include "wood_joint.h"
#include "wood_cut.h"
#include "../src/intersection.h"
#include "../src/polyline.h"
#include "../src/line.h"
#include "../src/vector.h"
#include "../src/point.h"
#include "../src/xform.h"
#include "../src/plane.h"
#include "../src/tolerance.h"
#include <fmt/core.h>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <vector>

using namespace session_cpp;
using wood_session::WoodJoint;
using wood_session::WoodElement;

// ss_e_r_0 and other joint-lib primitives needed by side_removal_ss_e_r_1_port.
// Included in anonymous namespace to match wood_main.cpp's usage pattern.
namespace {
#include "wood_joint_lib.h"
}


namespace wood_session {

void apply_unit_scale(WoodJoint& joint) {
    if (const char* dp = std::getenv("WOOD_APPLY_DUMP")) {
        std::ofstream alog(dp, std::ios::app);
        alog << "apply_unit_scale: joint v0=" << joint.el_ids.first << " v1=" << joint.el_ids.second
             << " unit_scale=" << joint.unit_scale << " usd=" << joint.unit_scale_distance << "\n";
        auto& vols = joint.joint_volumes_pair_a_pair_b;
        for (int i = 0; i < 4; i++) {
            alog << "  BEFORE vols[" << i << "] ";
            if (vols[i].has_value()) {
                for (size_t k = 0; k < vols[i]->point_count(); k++) {
                    Point p = vols[i]->get_point(k);
                    alog << "(" << p[0] << "," << p[1] << "," << p[2] << ") ";
                }
            } else alog << "nullopt";
            alog << "\n";
        }
    }
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
            double raw = std::sqrt(dx*dx + dy*dy + dz*dz);
            // Wood's `std::floor(std::sqrt(CGAL::squared_distance(...)))` at
            // wood_joint.cpp:281 runs on CGAL exact arithmetic. Session uses
            // double precision, so raw values like 19.99999988 (exact = 20)
            // floor to 19 instead of 20. Nudge the value by a tiny epsilon
            // so near-integer doubles from double-precision accumulation
            // round to the same integer wood's exact math produces.
            joint.unit_scale_distance = std::floor(raw + 1e-6);
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
void joint_orient_to_connection_area(WoodJoint& joint) {
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

    // Transform male outlines with xf0, female with xf1.
    for (int face = 0; face < 2; face++) {
        for (auto& pl : joint.m_outlines[face]) pl = pl.transformed_xform(xf0);
        for (auto& pl : joint.f_outlines[face]) pl = pl.transformed_xform(xf1);
    }
}

// Port of wood_joint.cpp:418-510
// remove_geo_from_linked_joint_and_merge_with_current_joint
// Interleaves shadow joint outlines into the primary joint after both are oriented.
void merge_linked_joints(WoodJoint& joint, std::vector<WoodJoint>& all_joints) {
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
void joint_get_divisions(WoodJoint& joint, double division_distance) {
    joint.division_length = division_distance;
    if (joint.joint_lines[0].squared_length() > 1e-10) {
        joint.length = std::sqrt(joint.joint_lines[0].squared_length());
        joint.divisions = std::max(1, std::min(100,
            (int)std::ceil(joint.length / division_distance)));
    }
}

void side_removal_ss_e_r_1_port(WoodJoint& joint,
                                        const std::vector<WoodElement>& elements) {
    // Wood's case 58 dispatch calls `side_removal(jo, elements, true)` — the
    // simpler side_removal at wood_joint_lib.cpp:432, NOT the more complex
    // side_removal_ss_e_r_1 at line 2723. The simple variant emits four
    // rectangles without any boolean/conic operations.
    joint.name = "side_removal";
    joint.no_orient = true;

    // Wood swaps the joint's own fields (wood_joint_lib.cpp:438-443), not
    // just local copies.
    std::swap(joint.el_ids.first, joint.el_ids.second);
    std::swap(joint.face_ids.first[0], joint.face_ids.second[0]);
    std::swap(joint.face_ids.first[1], joint.face_ids.second[1]);
    std::swap(joint.joint_lines[0], joint.joint_lines[1]);

    int v0 = joint.el_ids.first;
    int v1 = joint.el_ids.second;
    int f0_0 = joint.face_ids.first[0];
    int f1_0 = joint.face_ids.second[0];

    if (v0 < 0 || v0 >= (int)elements.size() || v1 < 0 || v1 >= (int)elements.size()) {
        ss_e_r_0(joint); // fall back to world-space rect split
        return;
    }
    if (f0_0 < 0 || f0_0 >= (int)elements[v0].planes.size() ||
        f1_0 < 0 || f1_0 >= (int)elements[v1].planes.size() ||
        f0_0 >= (int)elements[v0].polylines.size() ||
        f1_0 >= (int)elements[v1].polylines.size()) {
        ss_e_r_0(joint);
        return;
    }

    // Normal vectors (unit), scaled by joint.scale[2].
    Vector n0 = elements[v0].planes[f0_0].z_axis(); n0.normalize_self();
    Vector n1 = elements[v1].planes[f1_0].z_axis(); n1.normalize_self();

    double s2 = joint.scale[2];
    Vector f0_0_normal(n0[0]*s2, n0[1]*s2, n0[2]*s2);
    Vector f1_0_normal(n1[0]*(s2 + 2.0), n1[1]*(s2 + 2.0), n1[2]*(s2 + 2.0));
    Vector f0_1_normal(n0[0]*(s2 + 2.0 + joint.shift),
                       n0[1]*(s2 + 2.0 + joint.shift),
                       n0[2]*(s2 + 2.0 + joint.shift));

    // Copy side-face rectangles (5-pt closed polylines).
    Polyline pline0 = elements[v0].polylines[f0_0];
    Polyline pline1 = elements[v1].polylines[f1_0];

    // Extend side rectangles at convex corners (wood_joint_lib.cpp:2760-2789).
    // For a 4-corner plate, each side-face-rect corner maps to a top-polygon
    // corner; if that top-polygon corner is convex, the side-rect's 1st/2nd
    // edges are extended by scale[0] to widen the removal zone into the
    // neighbouring plate material.
    auto get_convex = [](const Polyline& pl, const Vector& normal) -> std::vector<bool> {
        size_t n = pl.point_count();
        if (n > 1) {
            Point f = pl.get_point(0); Point l = pl.get_point(n-1);
            double dx=f[0]-l[0], dy=f[1]-l[1], dz=f[2]-l[2];
            if (dx*dx+dy*dy+dz*dz < 1e-10) --n;
        }
        std::vector<bool> conv; conv.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            size_t prev = (i == 0) ? n-1 : i-1;
            size_t next = (i+1 == n) ? 0 : i+1;
            Point pi = pl.get_point(i);
            Point pp = pl.get_point(prev);
            Point pn = pl.get_point(next);
            Vector d0(pi[0]-pp[0], pi[1]-pp[1], pi[2]-pp[2]); d0.normalize_self();
            Vector d1(pn[0]-pi[0], pn[1]-pi[1], pn[2]-pi[2]); d1.normalize_self();
            Vector cr = d0.cross(d1);
            conv.push_back(cr.dot(normal) >= 0.0);
        }
        return conv;
    };
    // Translate a specific segment (edge `i`) of a closed polyline by shifting
    // endpoint `i` backwards by s0 and endpoint `i+1` forwards by s1 along the
    // edge direction. Mirrors wood's cgal_polyline_util.cpp:406-435 including
    // the closing-vertex sync when sID==0 or sID+1==last.
    auto extend_edge = [](Polyline& pl, size_t edge_id, double s0, double s1) {
        if (s0 == 0.0 && s1 == 0.0) return;
        size_t n = pl.point_count();
        if (edge_id + 1 >= n) return;
        Point a = pl.get_point(edge_id);
        Point b = pl.get_point(edge_id + 1);
        Vector d(b[0]-a[0], b[1]-a[1], b[2]-a[2]);
        double len = std::sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
        if (len < 1e-12) return;
        Vector u(d[0]/len, d[1]/len, d[2]/len);
        Point a_new(a[0] - u[0]*s0, a[1] - u[1]*s0, a[2] - u[2]*s0);
        Point b_new(b[0] + u[0]*s1, b[1] + u[1]*s1, b[2] + u[2]*s1);
        std::vector<Point> pts;
        pts.reserve(n);
        for (size_t k = 0; k < n; ++k) {
            if (k == edge_id) pts.push_back(a_new);
            else if (k == edge_id + 1) pts.push_back(b_new);
            else pts.push_back(pl.get_point(k));
        }
        // Closing-vertex sync (wood extend() :431-434).
        if (edge_id == 0) pts.back() = pts.front();
        else if (edge_id + 1 == n - 1) pts.front() = pts.back();
        pl = Polyline(pts);
    };
    if (pline0.point_count() == 5 && pline1.point_count() == 5) {
        const Polyline& top0 = elements[v0].polylines[0];
        const Polyline& top1 = elements[v1].polylines[0];
        Vector norm0 = elements[v0].planes[0].z_axis(); norm0.normalize_self();
        Vector norm1 = elements[v1].planes[0].z_axis(); norm1.normalize_self();
        std::vector<bool> cc0 = get_convex(top0, norm0);
        std::vector<bool> cc1 = get_convex(top1, norm1);
        double sc0 = joint.scale[0];
        if (!cc0.empty()) {
            int a_idx = f0_0 - 2;
            int b_idx = (a_idx + 1) % (int)cc0.size();
            double sc0_0 = (a_idx >= 0 && a_idx < (int)cc0.size() && cc0[a_idx]) ? sc0 : 0.0;
            double sc0_1 = (b_idx >= 0 && b_idx < (int)cc0.size() && cc0[b_idx]) ? sc0 : 0.0;
            extend_edge(pline0, 0, sc0_0, sc0_1);
            extend_edge(pline0, 2, sc0_1, sc0_0);
        }
        if (!cc1.empty()) {
            int a_idx = f1_0 - 2;
            int b_idx = (a_idx + 1) % (int)cc1.size();
            double sc1_0 = (a_idx >= 0 && a_idx < (int)cc1.size() && cc1[a_idx]) ? sc0 : 0.0;
            double sc1_1 = (b_idx >= 0 && b_idx < (int)cc1.size() && cc1[b_idx]) ? sc0 : 0.0;
            extend_edge(pline1, 0, sc1_0, sc1_1);
            extend_edge(pline1, 2, sc1_1, sc1_0);
        }
        // Extend vertical edges by scale[1] on both sides.
        double sv = joint.scale[1];
        extend_edge(pline0, 1, sv, sv);
        extend_edge(pline0, 3, sv, sv);
        extend_edge(pline1, 1, sv, sv);
        extend_edge(pline1, 3, sv, sv);
    }

    // Move copies by normal offsets — produces the "removed" outline that
    // cuts the plate body.
    auto move_poly = [](const Polyline& pl, const Vector& d) -> Polyline {
        std::vector<Point> pts;
        pts.reserve(pl.point_count());
        for (size_t k = 0; k < pl.point_count(); ++k) {
            Point p = pl.get_point(k);
            pts.emplace_back(p[0]+d[0], p[1]+d[1], p[2]+d[2]);
        }
        return Polyline(pts);
    };

    Polyline pline0_moved0 = move_poly(pline0, f0_0_normal);
    Polyline pline0_moved1 = move_poly(pline0, f0_1_normal);
    Polyline pline1_moved  = move_poly(pline1, f1_0_normal);

    const bool merge_branch = (joint.shift > 0.0);
    if (!merge_branch) {
        // wood_joint_lib.cpp:2909-2924 — simple 2-outline pair.
        joint.m_outlines[0] = { pline0,        pline0 };
        joint.m_outlines[1] = { pline0_moved0, pline0_moved0 };
        joint.f_outlines[0] = { pline1,        pline1 };
        joint.f_outlines[1] = { pline1_moved,  pline1_moved };
        joint.m_cut_types[0] = { wood_cut::mill_project, wood_cut::mill_project };
        joint.m_cut_types[1] = { wood_cut::mill_project, wood_cut::mill_project };
        joint.f_cut_types[0] = { wood_cut::mill_project, wood_cut::mill_project };
        joint.f_cut_types[1] = { wood_cut::mill_project, wood_cut::mill_project };
        return;
    }

    // wood_joint_lib.cpp:580-601 merge_with_joint branch: 4 outlines per face.
    //   m[0] = {pline0_moved0, pline0_moved0, pline0, pline0}
    //   m[1] = {pline0_moved1, pline0_moved1, pline0_moved0, pline0_moved0}
    joint.m_outlines[0] = { pline0_moved0, pline0_moved0, pline0, pline0 };
    joint.m_outlines[1] = { pline0_moved1, pline0_moved1, pline0_moved0, pline0_moved0 };
    joint.f_outlines[0] = { pline1,        pline1 };
    joint.f_outlines[1] = { pline1_moved,  pline1_moved };

    joint.m_cut_types[0] = { wood_cut::mill_project, wood_cut::mill_project,
                             wood_cut::mill_project, wood_cut::mill_project };
    joint.m_cut_types[1] = { wood_cut::mill_project, wood_cut::mill_project,
                             wood_cut::mill_project, wood_cut::mill_project };
    joint.f_cut_types[0] = { wood_cut::mill_project, wood_cut::mill_project };
    joint.f_cut_types[1] = { wood_cut::mill_project, wood_cut::mill_project };
}

// tt_e_p_3: top-to-top drill-grid joint. Port of wood_joint_lib.cpp:5651-5710.
// Offsets the joint_area inward by -shift, divides each offset polygon edge
// into points, and emits 2-point drill lines per point along the thickness
// direction of each element. Emits 4 copies per point per face (m[0], m[1],
// f[0], f[1]) with wood::cut::drill cut type. unit_scale stays false; orient
// is disabled (joint geometry is world-space).
void tt_e_p_3(WoodJoint& joint,
                     const std::vector<WoodElement>& elements) {
    joint.name = "tt_e_p_3";
    joint.no_orient = true;

    int v0 = joint.el_ids.first;
    int v1 = joint.el_ids.second;
    if (v0 < 0 || v0 >= (int)elements.size() ||
        v1 < 0 || v1 >= (int)elements.size()) return;

    // Need at least one joint_volume to derive the drill axis direction.
    if (!joint.joint_volumes_pair_a_pair_b[0].has_value()) return;
    const Polyline& jv0 = *joint.joint_volumes_pair_a_pair_b[0];
    if (jv0.point_count() < 3) return;

    // offset_and_divide_to_points: wood's clipper_util.cpp:886-910.
    // 1. Get fast plane of joint_area
    // 2. Offset polygon in 3D by offset_distance
    // 3. For each edge, interpolate points (mode=2 = include start only)
    if (joint.joint_area.point_count() < 4) return;
    Polyline poly_copy = joint.joint_area;
    Point fast_origin;
    Plane fast_plane;
    poly_copy.get_fast_plane(fast_origin, fast_plane);

    double offset_distance = -joint.shift;
    double division_distance = joint.division_length;
    if (division_distance <= 0.0) return; // wood's loop would divide by zero

    Intersection::offset_in_3d(poly_copy, fast_plane, offset_distance);

    std::vector<Point> points;
    auto op_pts = poly_copy.get_points();
    for (size_t i = 0; i + 1 < op_pts.size(); i++) {
        double seg_len = Point::distance(op_pts[i], op_pts[i + 1]);
        int divisions = (int)std::min(100.0, seg_len / division_distance);
        std::vector<Point> dp = Polyline::interpolate_points(
            op_pts[i], op_pts[i + 1], divisions, /*kind=*/2);
        points.insert(points.end(), dp.begin(), dp.end());
    }
    // Wood: if polygon is open (front != back), append back vertex.
    if (!op_pts.empty()) {
        double dx = op_pts.front()[0] - op_pts.back()[0];
        double dy = op_pts.front()[1] - op_pts.back()[1];
        double dz = op_pts.front()[2] - op_pts.back()[2];
        if (dx*dx + dy*dy + dz*dz > 0.01 /* DISTANCE_SQUARED */)
            points.push_back(op_pts.back());
    }

    // Direction vectors: dir0 = unit(jv[0][1] - jv[0][2]) * thickness_v0
    //                    dir1 = -dir0_unit * thickness_v1
    Point jv_1 = jv0.get_point(1);
    Point jv_2 = jv0.get_point(2);
    Vector dir0(jv_1[0] - jv_2[0], jv_1[1] - jv_2[1], jv_1[2] - jv_2[2]);
    dir0.normalize_self();
    Vector dir1(-dir0[0], -dir0[1], -dir0[2]);
    double t0 = elements[v0].thickness;
    double t1 = elements[v1].thickness;
    dir0 = Vector(dir0[0] * t0, dir0[1] * t0, dir0[2] * t0);
    dir1 = Vector(dir1[0] * t1, dir1[1] * t1, dir1[2] * t1);

    // Clear any pre-existing outlines (defensive).
    joint.m_outlines[0].clear();
    joint.m_outlines[1].clear();
    joint.f_outlines[0].clear();
    joint.f_outlines[1].clear();
    joint.m_cut_types[0].clear();
    joint.m_cut_types[1].clear();
    joint.f_cut_types[0].clear();
    joint.f_cut_types[1].clear();

    for (const Point& pt : points) {
        Polyline line0(std::vector<Point>{
            pt,
            Point(pt[0] + dir0[0], pt[1] + dir0[1], pt[2] + dir0[2]),
        });
        Polyline line1(std::vector<Point>{
            pt,
            Point(pt[0] + dir1[0], pt[1] + dir1[1], pt[2] + dir1[2]),
        });
        // Wood emits 2 copies per face (the pair: one for female, one for
        // male on each face).
        joint.f_outlines[0].push_back(line0);
        joint.f_outlines[0].push_back(line0);
        joint.f_outlines[1].push_back(line0);
        joint.f_outlines[1].push_back(line0);
        joint.m_outlines[0].push_back(line1);
        joint.m_outlines[0].push_back(line1);
        joint.m_outlines[1].push_back(line1);
        joint.m_outlines[1].push_back(line1);
        joint.m_cut_types[0].push_back(wood_cut::drill);
        joint.m_cut_types[0].push_back(wood_cut::drill);
        joint.m_cut_types[1].push_back(wood_cut::drill);
        joint.m_cut_types[1].push_back(wood_cut::drill);
        joint.f_cut_types[0].push_back(wood_cut::drill);
        joint.f_cut_types[0].push_back(wood_cut::drill);
        joint.f_cut_types[1].push_back(wood_cut::drill);
        joint.f_cut_types[1].push_back(wood_cut::drill);
    }
}

// Build wood-compatible element from a polyline pair.
// Matches wood_main.cpp get_elements: orientation check, side planes from
// 3 raw points, side polylines from 4 corners.

} // namespace wood_session
