// ═══════════════════════════════════════════════════════════════════════════
// wood/wood_joint_detection.cpp — plane_to_face + CrossJoint implementation.
//
// Wood-domain cross/lap joint detection between two plate elements. Relocated
// from src/intersection.{h,cpp} because the API is tied to wood concepts
// (joint area/volumes/lines, plate face ids, type-30 code) — not generic
// geometry. The general kernel keeps only pure-geometry helpers (line_plane,
// plane_plane, plane_4lines, etc.) which this file consumes.
//
// Original wood references:
//   cgal::intersection_util::polyline_plane_cross_joint (cgal_intersection_util.cpp:771-803)
//   wood_main.cpp:436-437 (joint_volumes convention: [0]=area+v, [1]=area-v)
// ═══════════════════════════════════════════════════════════════════════════

#include "../src/element.h"
#include "../src/intersection.h"
#include "../src/line.h"
#include "../src/plane.h"
#include "../src/point.h"
#include "../src/polyline.h"
#include "../src/tolerance.h"
#include "../src/vector.h"
#include "wood_session.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

using namespace session_cpp;

namespace wood_session {

namespace {

// Near-coplanar rejection threshold. Wood reads from GLOBALS::DISTANCE_SQUARED
// which hexboxes multiplies by 100. Caller syncs via set_cross_joint_distance_squared.
double g_cross_distance_squared = 0.01;

// Wood's private polyline-plane variant — rejects the entire polyline if ANY
// segment endpoint is within g_cross_distance_squared of the plane. Used ONLY
// from polyline_plane_cross_joint; wood's polyline_plane_to_line does NOT use
// this check.
bool polyline_plane_cross(const Polyline& polyline, const Plane& plane,
                          std::vector<Point>& points, std::vector<int>& edge_ids) {
    size_t n = polyline.point_count();
    if (n < 2) return false;
    const double DISTANCE_SQUARED = g_cross_distance_squared;
    Vector n_plane = plane.z_axis();
    double n_mag_sq = n_plane[0]*n_plane[0] + n_plane[1]*n_plane[1] + n_plane[2]*n_plane[2];
    Point o = plane.origin();
    auto sq_dist_to_plane = [&](const Point& p) -> double {
        double dx = p[0]-o[0], dy = p[1]-o[1], dz = p[2]-o[2];
        double num = dx*n_plane[0] + dy*n_plane[1] + dz*n_plane[2];
        return (n_mag_sq > 0.0) ? (num * num / n_mag_sq) : 0.0;
    };
    for (size_t i = 0; i < n - 1; i++) {
        Point a = polyline.get_point(i);
        Point b = polyline.get_point(i + 1);
        if (sq_dist_to_plane(a) < DISTANCE_SQUARED) { points.clear(); edge_ids.clear(); return false; }
        if (sq_dist_to_plane(b) < DISTANCE_SQUARED) { points.clear(); edge_ids.clear(); return false; }
        Line seg(a[0], a[1], a[2], b[0], b[1], b[2]);
        Point hit;
        if (Intersection::line_plane(seg, plane, hit, true)) {
            points.push_back(hit);
            edge_ids.push_back(static_cast<int>(i));
        }
    }
    return points.size() == 2;
}

// Project polygon + test points to plane local 2D, run boundary-inclusive PIP,
// fill inside indices. Mirrors the cross-joint detector's convention: any point
// on a polygon edge counts as inside.
int are_points_inside(
    const Polyline& polygon,
    const Plane& plane,
    const std::vector<Point>& test_points,
    std::vector<int>& inside_indices_out) {

    const Point& o = plane.origin();
    Vector xa = plane.base1();
    Vector ya = plane.base2();

    auto poly_pts = polygon.get_points();
    size_t np_raw = poly_pts.size();
    if (np_raw > 1) {
        const Point& f = poly_pts.front();
        const Point& l = poly_pts.back();
        if (std::fabs(f[0]-l[0])<1e-12 && std::fabs(f[1]-l[1])<1e-12 && std::fabs(f[2]-l[2])<1e-12)
            np_raw--;
    }

    std::vector<double> px, py;
    px.reserve(np_raw);
    py.reserve(np_raw);
    for (size_t i = 0; i < np_raw; i++) {
        const Point& p = poly_pts[i];
        double dx = p[0] - o[0], dy = p[1] - o[1], dz = p[2] - o[2];
        px.push_back(dx*xa[0]+dy*xa[1]+dz*xa[2]);
        py.push_back(dx*ya[0]+dy*ya[1]+dz*ya[2]);
    }
    size_t np = px.size();
    if (np < 3) return 0;

    auto cross_sign = [](double ax, double ay, double bx, double by,
                          double cx, double cy) -> int {
        double v = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        if (v > 0.0) return 1;
        if (v < 0.0) return -1;
        return 0;
    };

    auto pip = [&](double tx, double ty) -> int {
        size_t first = 0;
        while (first < np && py[first] == ty) first++;
        if (first == np) {
            for (size_t i = 0; i < np; i++) {
                size_t j = (i + 1) % np;
                if ((std::min(px[i], px[j]) <= tx) && (tx <= std::max(px[i], px[j])))
                    return 2;
            }
            return 0;
        }

        bool is_above = py[first] < ty;
        bool starting_above = is_above;
        int val = 0;
        size_t curr = first + 1;
        size_t cend = np;
        while (true) {
            if (curr == cend) {
                if (cend == first || first == 0) break;
                cend = first;
                curr = 0;
            }

            if (is_above) {
                while (curr != cend && py[curr] < ty) curr++;
                if (curr == cend) continue;
            } else {
                while (curr != cend && py[curr] > ty) curr++;
                if (curr == cend) continue;
            }

            size_t prev = (curr == 0) ? (np - 1) : (curr - 1);

            if (py[curr] == ty) {
                if (px[curr] == tx ||
                    (py[curr] == py[prev] &&
                     ((tx < px[prev]) != (tx < px[curr]))))
                    return 2;
                curr++;
                if (curr == first) break;
                continue;
            }

            if (tx < px[curr] && tx < px[prev]) {
                // ray left of both endpoints — no crossing on the right.
            } else if (tx > px[prev] && tx > px[curr]) {
                val = 1 - val;
            } else {
                int d = cross_sign(px[prev], py[prev], px[curr], py[curr], tx, ty);
                if (d == 0) return 2;
                if ((d < 0) == is_above) val = 1 - val;
            }
            is_above = !is_above;
            curr++;
        }

        if (is_above != starting_above) {
            if (curr == np) curr = 0;
            size_t prev = (curr == 0) ? (np - 1) : (curr - 1);
            int d = cross_sign(px[prev], py[prev], px[curr], py[curr], tx, ty);
            if (d == 0) return 2;
            if ((d < 0) == is_above) val = 1 - val;
        }

        return val;
    };

    int count = 0;
    for (size_t i = 0; i < test_points.size(); i++) {
        const Point& p = test_points[i];
        double dx = p[0] - o[0], dy = p[1] - o[1], dz = p[2] - o[2];
        double tx = dx*xa[0]+dy*xa[1]+dz*xa[2];
        double ty = dx*ya[0]+dy*ya[1]+dz*ya[2];
        int r = pip(tx, ty);
        if (r != 0) {
            inside_indices_out.push_back(static_cast<int>(i));
            count++;
        }
    }
    return count;
}

// Cross-joint chord between two polylines via reciprocal polyline-plane
// intersections. Returns the contact segment + the (edge_in_c0, edge_in_c1) pair.
bool polyline_plane_cross_joint(
    const Polyline& c0,
    const Polyline& c1,
    const Plane& p0,
    const Plane& p1,
    Line& contact_out,
    std::pair<int,int>& edge_pair_out) {

    std::vector<Point> pts0;
    std::vector<int> edge_ids_0;
    if (!polyline_plane_cross(c0, p1, pts0, edge_ids_0)) return false;

    std::vector<Point> pts1;
    std::vector<int> edge_ids_1;
    if (!polyline_plane_cross(c1, p0, pts1, edge_ids_1)) return false;

    if (pts0.size() < 2 || pts1.size() < 2) return false;

    std::vector<int> ID1;
    int count0 = are_points_inside(c0, p0, pts1, ID1);

    std::vector<int> ID0;
    int count1 = are_points_inside(c1, p1, pts0, ID0);

    if (count0 == 0 && count1 == 0) return false;

    if (std::abs(count0 - count1) == 2) {
        if (count0 == 2) {
            contact_out = Line::from_points(pts0[0], pts0[1]);
            edge_pair_out = std::pair<int,int>(edge_ids_0[0], edge_ids_0[1]);
        } else {
            contact_out = Line::from_points(pts1[0], pts1[1]);
            edge_pair_out = std::pair<int,int>(edge_ids_1[0], edge_ids_1[1]);
        }
        return true;
    }

    if (count0 == 1 && count1 == 1) {
        contact_out = Line::from_points(pts0[ID0[0]], pts1[ID1[0]]);
        edge_pair_out = std::pair<int,int>(edge_ids_0[ID0[0]], edge_ids_1[ID1[0]]);
        return true;
    }

    if (count0 > 1 || count1 > 1) {
        std::vector<Point> pts;
        pts.reserve(ID0.size() + ID1.size());
        for (int i : ID0) pts.push_back(pts0[i]);
        for (int i : ID1) pts.push_back(pts1[i]);

        double xmin = pts[0][0], ymin = pts[0][1], zmin = pts[0][2];
        double xmax = xmin, ymax = ymin, zmax = zmin;
        for (const auto& q : pts) {
            xmin = std::min(xmin, q[0]); ymin = std::min(ymin, q[1]); zmin = std::min(zmin, q[2]);
            xmax = std::max(xmax, q[0]); ymax = std::max(ymax, q[1]); zmax = std::max(zmax, q[2]);
        }
        Point lo(xmin, ymin, zmin);
        Point hi(xmax, ymax, zmax);
        contact_out = Line::from_points(lo, hi);

        int e0 = -1, e1 = -1;
        for (size_t i = 0; i < ID0.size(); i++) {
            const Point& q = pts0[ID0[i]];
            double d_lo = (q-lo).magnitude_squared();
            double d_hi = (q-hi).magnitude_squared();
            if (d_lo < 0.001 || d_hi < 0.001) { e0 = edge_ids_0[ID0[i]]; break; }
        }
        for (size_t i = 0; i < ID1.size(); i++) {
            const Point& q = pts1[ID1[i]];
            double d_lo = (q-lo).magnitude_squared();
            double d_hi = (q-hi).magnitude_squared();
            if (d_lo < 0.001 || d_hi < 0.001) { e1 = edge_ids_1[ID1[i]]; break; }
        }
        edge_pair_out = std::pair<int,int>(e0, e1);
        return true;
    }

    return false;
}

double approximate_angle_deg(const Vector& a, const Vector& b) {
    double la = a.magnitude(), lb = b.magnitude();
    if (la < Tolerance::ZERO_TOLERANCE || lb < Tolerance::ZERO_TOLERANCE) return 0.0;
    double c = a.dot(b) / (la * lb);
    if (c > 1.0) c = 1.0;
    if (c < -1.0) c = -1.0;
    return std::acos(c) * 180.0 / 3.14159265358979323846;
}

Vector segment_to_vector(const Line& l) {
    return Vector(l[3]-l[0], l[4]-l[1], l[5]-l[2]);
}

Line opposite_segment(const Line& l) {
    return Line(l[3], l[4], l[5], l[0], l[1], l[2]);
}

Polyline translate_quad(const Polyline& poly, const Vector& v) {
    auto pts = poly.get_points();
    for (auto& p : pts) {
        p[0] += v[0]; p[1] += v[1]; p[2] += v[2];
    }
    return Polyline(pts);
}

} // anonymous namespace

void set_cross_joint_distance_squared(double dist_sq) {
    g_cross_distance_squared = dist_sq;
}

bool plane_to_face(
    const std::array<Polyline,2>& polylines_a,
    const std::array<Polyline,2>& polylines_b,
    const std::array<Plane,2>& planes_a,
    const std::array<Plane,2>& planes_b,
    CrossJoint& result,
    double angle_tol,
    const std::array<double,3>& extension) {

    result.face_ids_a = {-1, -1};
    result.face_ids_b = {-1, -1};
    result.type = 30;

    // 1. Parallelism guard.
    double raw_angle = approximate_angle_deg(planes_a[0].z_axis(), planes_b[0].z_axis());
    double angle = 90.0 - std::fabs(raw_angle - 90.0);
    if (angle < angle_tol) return false;

    // 2. Four cross-joint contact segments.
    const Polyline& cx0 = polylines_a[0];
    const Polyline& cx1 = polylines_a[1];
    const Polyline& cy0 = polylines_b[0];
    const Polyline& cy1 = polylines_b[1];
    const Plane& px0 = planes_a[0];
    const Plane& px1 = planes_a[1];
    const Plane& py0 = planes_b[0];
    const Plane& py1 = planes_b[1];

    Line cx0_py0__cy0_px0;
    std::pair<int,int> e0_0__e1_0;
    if (!polyline_plane_cross_joint(cx0, cy0, px0, py0, cx0_py0__cy0_px0, e0_0__e1_0)) return false;

    Line cx0_py1__cy1_px0;
    std::pair<int,int> e0_0__e1_1;
    if (!polyline_plane_cross_joint(cx0, cy1, px0, py1, cx0_py1__cy1_px0, e0_0__e1_1)) return false;

    Line cx1_py0__cy0_px1;
    std::pair<int,int> e0_1__e1_0;
    if (!polyline_plane_cross_joint(cx1, cy0, px1, py0, cx1_py0__cy0_px1, e0_1__e1_0)) return false;

    Line cx1_py1__cy1_px1;
    std::pair<int,int> e0_1__e1_1;
    if (!polyline_plane_cross_joint(cx1, cy1, px1, py1, cx1_py1__cy1_px1, e0_1__e1_1)) return false;

    // 3. Record side-face IDs (+2 because indices 0,1 are top/bottom).
    result.face_ids_a.first  = e0_0__e1_0.first + 2;
    result.face_ids_b.first  = e0_0__e1_0.second + 2;
    result.face_ids_a.second = e0_1__e1_1.first + 2;
    result.face_ids_b.second = e0_1__e1_1.second + 2;

    // 4. Sort segments to a common orientation (flip when angle to reference > 90°).
    Vector ref_v = segment_to_vector(cx0_py0__cy0_px0);
    {
        Vector v = segment_to_vector(cx0_py1__cy1_px0);
        if (approximate_angle_deg(ref_v, v) > approximate_angle_deg(ref_v, Vector(-v[0], -v[1], -v[2])))
            cx0_py1__cy1_px0 = opposite_segment(cx0_py1__cy1_px0);
    }
    {
        Vector v = segment_to_vector(cx1_py0__cy0_px1);
        if (approximate_angle_deg(ref_v, v) > approximate_angle_deg(ref_v, Vector(-v[0], -v[1], -v[2])))
            cx1_py0__cy0_px1 = opposite_segment(cx1_py0__cy0_px1);
    }
    {
        Vector v = segment_to_vector(cx1_py1__cy1_px1);
        if (approximate_angle_deg(ref_v, v) > approximate_angle_deg(ref_v, Vector(-v[0], -v[1], -v[2])))
            cx1_py1__cy1_px1 = opposite_segment(cx1_py1__cy1_px1);
    }

    // 5. Reference axis: midline of two contact segments, scaled ×10 about its midpoint.
    // Uses scale_line (extend-and-flip semantics) matching wood's cgal_polyline_util.cpp:395-403.
    Line c;
    get_middle_line(cx0_py1__cy1_px0, cx1_py0__cy0_px1, c);
    {
        double len = c.length();
        if (len < Tolerance::ZERO_TOLERANCE) return false;
        scale_line(c, 10.0);
    }

    Point c_start = c.start();
    Point c_end = c.end();

    // 6. Project 8 segment endpoints onto c, sorted params give lMin (gap) and lMax (extent).
    auto project_t = [&](const Point& p) -> double {
        double t;
        Polyline::closest_point_to_line(p, c_start, c_end, t);
        return t;
    };

    double cpt0[4] = {
        project_t(cx0_py0__cy0_px0.start()),
        project_t(cx0_py1__cy1_px0.start()),
        project_t(cx1_py0__cy0_px1.start()),
        project_t(cx1_py1__cy1_px1.start())
    };
    std::sort(cpt0, cpt0 + 4);

    double cpt1[4] = {
        project_t(cx0_py0__cy0_px0.end()),
        project_t(cx0_py1__cy1_px0.end()),
        project_t(cx1_py0__cy0_px1.end()),
        project_t(cx1_py1__cy1_px1.end())
    };
    std::sort(cpt1, cpt1 + 4);

    double cpt[8] = { cpt0[0], cpt0[1], cpt0[2], cpt0[3], cpt1[0], cpt1[1], cpt1[2], cpt1[3] };
    std::sort(cpt, cpt + 8);

    Line lMin = Line::from_points(c.point_at(cpt0[3]), c.point_at(cpt1[0]));
    Line lMax = Line::from_points(c.point_at(cpt[0]),  c.point_at(cpt[7]));

    // 7. Mid plane through midpoint of lMin perpendicular to lMin.
    Point lMin_mid = lMin.center();
    Vector lMin_dir = lMin.to_vector();
    if (lMin_dir.magnitude() < Tolerance::ZERO_TOLERANCE) return false;
    Vector lMin_z = lMin_dir; lMin_z.normalize_self();
    Vector helper = (std::fabs(lMin_z[0]) < 0.9) ? Vector(1, 0, 0) : Vector(0, 1, 0);
    Vector mid_x = helper.cross(lMin_z); mid_x.normalize_self();
    Vector mid_y = lMin_z.cross(mid_x);  mid_y.normalize_self();
    Plane midPlane(lMin_mid, mid_x, mid_y, lMin_z);

    // 8. Extension vector v.
    Point midPlane_lMax;
    if (!Intersection::line_plane(lMax, midPlane, midPlane_lMax, false)) return false;
    Point lMax_a = lMax.start();
    Point lMax_b = lMax.end();
    int maxID = ((lMax_b - midPlane_lMax).magnitude_squared() > (lMax_a - midPlane_lMax).magnitude_squared()) ? 1 : 0;
    Vector v = (maxID == 1)
        ? (lMax_b - midPlane_lMax)
        : Vector(-(lMax_a[0]-midPlane_lMax[0]), -(lMax_a[1]-midPlane_lMax[1]), -(lMax_a[2]-midPlane_lMax[2]));

    if (extension[2] > 0.0) {
        double length = v.magnitude();
        if (length > Tolerance::ZERO_TOLERANCE) {
            double target = length + extension[2];
            v = Vector(v[0]*target/length, v[1]*target/length, v[2]*target/length);
        }
    }

    // 9. joint_area = plane_4lines(midPlane, four sorted segments as infinite lines).
    if (!Intersection::plane_4lines(midPlane,
                                    cx0_py0__cy0_px0,
                                    cx0_py1__cy1_px0,
                                    cx1_py1__cy1_px1,
                                    cx1_py0__cy0_px1,
                                    result.joint_area)) return false;

    // 9b. Joint lines: two perpendicular centerlines of the joint area quad.
    {
        const auto& jpts = result.joint_area.get_points();
        auto mid = [](const Point& p, const Point& q) {
            return Point((p[0]+q[0])*0.5, (p[1]+q[1])*0.5, (p[2]+q[2])*0.5);
        };
        Point m01 = mid(jpts[0], jpts[1]);
        Point m23 = mid(jpts[2], jpts[3]);
        Point m12 = mid(jpts[1], jpts[2]);
        Point m30 = mid(jpts[3], jpts[0]);
        result.joint_lines[0] = Polyline(std::vector<Point>{m01, m23});
        result.joint_lines[1] = Polyline(std::vector<Point>{m12, m30});
    }

    // 10. Joint volume faces = joint_area ± v (wood_main.cpp:436-437 convention).
    result.joint_volumes[0] = translate_quad(result.joint_area, v);
    result.joint_volumes[1] = translate_quad(result.joint_area, Vector(-v[0], -v[1], -v[2]));

    // 11. Optional in-plane edge extensions.
    if (extension[0] + extension[1] > 0.0) {
        for (int k = 0; k < 2; k++) {
            auto& pl = result.joint_volumes[k];
            pl.extend_segment(0, extension[0], extension[0], 0.0, 0.0);
            pl.extend_segment(2, extension[0], extension[0], 0.0, 0.0);
            pl.extend_segment(1, extension[1], extension[1], 0.0, 0.0);
            pl.extend_segment(3, extension[1], extension[1], 0.0, 0.0);
        }
    }

    return true;
}

bool plane_to_face(
    ElementPlate* a,
    ElementPlate* b,
    CrossJoint& result,
    double angle_tol,
    const std::array<double,3>& extension) {

    if (!a || !b) return false;
    auto polys_a = a->polylines();
    auto polys_b = b->polylines();
    auto planes_a = a->planes();
    auto planes_b = b->planes();
    if (polys_a.size() < 2 || polys_b.size() < 2) return false;
    if (planes_a.size() < 2 || planes_b.size() < 2) return false;

    std::array<Polyline,2> pa{ polys_a[0], polys_a[1] };
    std::array<Polyline,2> pb{ polys_b[0], polys_b[1] };
    std::array<Plane,2> npa{ planes_a[0], planes_a[1] };
    std::array<Plane,2> npb{ planes_b[0], planes_b[1] };
    return plane_to_face(pa, pb, npa, npb, result, angle_tol, extension);
}

} // namespace wood_session
