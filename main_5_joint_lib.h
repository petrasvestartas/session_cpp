// ─────────────────────────────────────────────────────────────────────────────
// main_5_joint_lib.h — wood joint geometry constructors (the
// `wood_joint_lib.cpp` equivalent for session_cpp/main_5.cpp).
//
// This header is intentionally a single-include drop-in for `main_5.cpp`. It
// is NOT a stand-alone library — it relies on `Point`, `Vector`, `Polyline`,
// `WoodJoint` and several other names being already in scope at the
// include point. Including this anywhere other than `main_5.cpp` will fail.
//
// Mirrors the wood file `cmake/src/wood/include/wood_joint_lib.cpp`.
// Each function below carries a `wood_joint_lib.cpp:LINE-LINE` reference to
// the wood source it was ported from. The `joint_create_geometry` two-level
// dispatcher that selects between these constructors lives in `main_5.cpp`
// (after the include) — see `wood_joint_lib.cpp:6075-6448` for wood's
// equivalent dispatcher.
//
// What's currently here:
//   group 0 (ss_e_ip, type 12): ss_e_ip_0 (id 2), ss_e_ip_1 (id 1)
//   group 1 (ss_e_op, type 11): ss_e_op_0 (id 12), ss_e_op_1 (id 10),
//                               ss_e_op_2 (id 11)
//   group 2 (ts_e_p,  type 20): ts_e_p_0 (id 23), ts_e_p_1 (no dispatch),
//                               ts_e_p_2 (id 21), ts_e_p_3 (id 20, 22)
//   helpers: interpolate_points (no-endpoints), remap_numbers
//
// What's NOT here (each blocked on a specific dependency, see
// `project_main_5_full_port_roadmap.md` in memory):
//   - ss_e_ip_2/3/4/5/custom         — element catalog OR cut types OR XML
//   - ss_e_op_3/4/5/6/custom         — cut types OR extra params OR linking
//   - ts_e_p_4/5/custom              — long parametric OR XML
//   - cr_c_ip_*  (group 3, type 30) — non-coplanar detection + cut types
//   - tt_e_p_*   (group 4, type 40) — element catalog + 5 CGAL helpers + drill
//   - ss_e_r_*   (group 5, type 13) — element catalog + rotated logic
//   - b_*        (group 6, type 60) — joint.scale field + slice cut type
//   - side_removal*                  — pre-detection face removal pipeline
//
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

// Wood verbatim: `wood::cut::cut_type` enum from
// `wood/cmake/src/wood/include/wood_cut.h`. Each joint constructor
// populates `WoodJoint::m_cut_types` and `f_cut_types` with one value
// per outline polyline (index-aligned with `m_outlines[face]` and
// `f_outlines[face]`). The merge function reads these to decide
// whether each polyline is a contour, a hole, a drill, etc. Stage 3
// of the wood port (`project_main_5_full_port_roadmap.md`).
namespace wood_cut {
    enum cut_type : int {
        nothing                          = 0,
        // Plates (faces, top/bottom + edges)
        hole                             = 1,
        edge_insertion                   = 2,
        insert_between_multiple_edges    = 3,
        // Beams (always projected or inside volume)
        slice                            = 4,
        slice_projectsheer               = 5,
        mill                             = 6,
        mill_project                     = 7,
        mill_projectsheer                = 8,
        cut                              = 9,
        cut_project                      = 10,
        cut_projectsheer                 = 11,
        cut_reverse                      = 12,
        conic                            = 13,
        conic_reverse                    = 14,
        // Vertical drill (plates & beams)
        drill                            = 15,
    };
}

// Wood verbatim: `interpolate_points(from, to, steps, include_ends=false, ...)`
// from wood_joint_lib.cpp:392-416 produces `steps` points at parameters
//   i / (1 + steps)  for i = 1..steps
// — i.e. NEITHER endpoint is included. ts_e_p_3 / ss_e_op_1 rely on this:
// arrays have `steps` points (not steps+1), so the loop `for j = 0..size-1`
// stops one iteration earlier than it would if the endpoint was included,
// and the explicit `pline.push_back(arrays[*][size-1])` at the end of each
// joint outline build adds the array's LAST point — which is the
// `steps/(1+steps)` parameter point, NOT the geometric endpoint.
//
// Producing `steps+1` points with include-ends semantics introduces a
// duplicate at the end of the outline (the explicit append duplicates the
// last loop push) and creates a visible back-and-forth spike where the
// joint outline meets the surrounding plate polyline.
static std::vector<Point> interpolate_points(const Point& a, const Point& b, int steps) {
    std::vector<Point> pts;
    pts.reserve(steps);
    for (int i = 1; i < steps + 1; i++) {
        double t = static_cast<double>(i) / static_cast<double>(1 + steps);
        pts.emplace_back(a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t, a[2]+(b[2]-a[2])*t);
    }
    return pts;
}

// Wood's `internal::remap_numbers` already exists in session as
// `Intersection::remap(value, in_min, in_max, out_min, out_max)` —
// callers use it directly.

// ─────────────────────────────────────────────────────────────────────────────
// group 0 — ss_e_ip (side-side in-plane, type 12)
// ─────────────────────────────────────────────────────────────────────────────

// ss_e_ip_0: side-to-side IN-PLANE HARDCODED 3-finger joint (type=12,
// joint name id=2). Verbatim port of wood_joint_lib.cpp:638-667.
//
// Same 12-point zigzag profile as ss_e_op_0 but oriented in the x/z plane
// (y=±0.5) instead of the y/z plane (x=±0.5). The "in-plane" naming refers
// to the joint axis being IN the plate's local plane (not perpendicular).
// All 4 outlines have identical geometry — m and f for the same face are
// the same outline, because for an in-plane edge joint both elements share
// the same boundary line and just slot into each other.
static void ss_e_ip_0(WoodJoint& joint) {
    auto P = [](double x, double y, double z) { return Point(x, y, z); };
    const double a = 0.357142857142857;   // 5/14
    const double b = 0.214285714285714;   // 3/14
    const double c = 0.0714285714285715;  // 1/14

    // f[0] / m[0] at y=-0.5: 12-pt zigzag + 2-pt endpoint marker
    auto build_minus = [&]() -> std::vector<Polyline> {
        return {
            Polyline(std::vector<Point>{
                P( 0.0,-0.5, a),  P(-0.5,-0.5, a),
                P(-0.5,-0.5, b),  P( 0.5,-0.5, b),
                P( 0.5,-0.5, c),  P(-0.5,-0.5, c),
                P(-0.5,-0.5,-c),  P( 0.5,-0.5,-c),
                P( 0.5,-0.5,-b),  P(-0.5,-0.5,-b),
                P(-0.5,-0.5,-a),  P( 0.0,-0.5,-a)}),
            Polyline(std::vector<Point>{P( 0.0,-0.5, 0.5), P( 0.0,-0.5,-0.5)}),
        };
    };
    auto build_plus = [&]() -> std::vector<Polyline> {
        return {
            Polyline(std::vector<Point>{
                P( 0.0, 0.5, a),  P(-0.5, 0.5, a),
                P(-0.5, 0.5, b),  P( 0.5, 0.5, b),
                P( 0.5, 0.5, c),  P(-0.5, 0.5, c),
                P(-0.5, 0.5,-c),  P( 0.5, 0.5,-c),
                P( 0.5, 0.5,-b),  P(-0.5, 0.5,-b),
                P(-0.5, 0.5,-a),  P( 0.0, 0.5,-a)}),
            Polyline(std::vector<Point>{P( 0.0, 0.5, 0.5), P( 0.0, 0.5,-0.5)}),
        };
    };
    joint.f_outlines[0] = build_minus();
    joint.f_outlines[1] = build_plus();
    joint.m_outlines[0] = build_minus();
    joint.m_outlines[1] = build_plus();
    // Wood: f_boolean_type / m_boolean_type = {edge_insertion, edge_insertion}
    // (wood_joint_lib.cpp:665-666). The 2nd entry is the endpoint marker —
    // session merge tags it edge_insertion too for backwards compatibility.
    joint.f_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.f_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
}

// ss_e_ip_1: side-to-side IN-PLANE PARAMETRIC zigzag joint (type=12, id=1).
// Verbatim port of wood_joint_lib.cpp:670-754. Walks `divisions` interpolated
// points along the central z-axis at (0, -0.5, *) and shifts them ±x per
// interleave to build a single zigzag polyline. The opposite face polyline
// (y=+0.5) is the same line offset by `v_o = (0, 1, 0)`. Both faces use the
// same outline for m and f (in-plane joints are symmetric).
static void ss_e_ip_1(WoodJoint& joint) {
    int div = std::max(2, std::min(100, joint.divisions));
    div += div % 2; // force even

    auto pts0 = interpolate_points(Point(0, -0.5, 0.5), Point(0, -0.5, -0.5), div);
    Vector v(0.5, 0, 0);
    double shift_ = Intersection::remap(joint.shift, 0, 1.0, -0.5, 0.5);
    Vector v_d(0, 0, -(1.0 / ((div + 1) * 2)) * shift_);

    std::vector<Point> pline0;
    pline0.reserve(pts0.size() * 2);
    pline0.push_back(pts0[0]);
    pline0.push_back(Point(pts0[0][0]-v[0]-v_d[0],
                            pts0[0][1]-v[1]-v_d[1],
                            pts0[0][2]-v[2]-v_d[2]));
    for (int i = 1; i + 1 < (int)pts0.size(); i++) {
        if (i % 2 == 1) {
            pline0.push_back(Point(pts0[i][0]-v[0]+v_d[0],
                                    pts0[i][1]-v[1]+v_d[1],
                                    pts0[i][2]-v[2]+v_d[2]));
            pline0.push_back(Point(pts0[i][0]+v[0]-v_d[0],
                                    pts0[i][1]+v[1]-v_d[1],
                                    pts0[i][2]+v[2]-v_d[2]));
        } else {
            pline0.push_back(Point(pts0[i][0]+v[0]+v_d[0],
                                    pts0[i][1]+v[1]+v_d[1],
                                    pts0[i][2]+v[2]+v_d[2]));
            pline0.push_back(Point(pts0[i][0]-v[0]-v_d[0],
                                    pts0[i][1]-v[1]-v_d[1],
                                    pts0[i][2]-v[2]-v_d[2]));
        }
    }
    Point last = pts0.back();
    pline0.push_back(Point(last[0]-v[0]+v_d[0],
                            last[1]-v[1]+v_d[1],
                            last[2]-v[2]+v_d[2]));
    pline0.push_back(last);

    // pline1 = pline0 offset by v_o = (0, 1, 0)
    Vector v_o(0, 1, 0);
    std::vector<Point> pline1;
    pline1.reserve(pline0.size());
    for (const Point& p : pline0)
        pline1.emplace_back(p[0]+v_o[0], p[1]+v_o[1], p[2]+v_o[2]);

    Polyline outline0(pline0);
    Polyline outline1(pline1);
    Polyline endpoints0(std::vector<Point>{pline0.front(), pline0.back()});
    Polyline endpoints1(std::vector<Point>{pline1.front(), pline1.back()});

    joint.f_outlines[0] = { outline0, endpoints0 };
    joint.f_outlines[1] = { outline1, endpoints1 };
    joint.m_outlines[0] = { outline0, endpoints0 };
    joint.m_outlines[1] = { outline1, endpoints1 };
    // Wood: f/m boolean types = {edge_insertion, edge_insertion}
    // (wood_joint_lib.cpp:752-753).
    joint.f_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.f_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
}

// ─────────────────────────────────────────────────────────────────────────────
// group 1 — ss_e_op (side-side out-of-plane, type 11)
// ─────────────────────────────────────────────────────────────────────────────

// ss_e_op_0: side-to-side out-of-plane HARDCODED 3-finger joint (type=11,
// joint name id=12 in the dispatcher). Verbatim port of
// wood_joint_lib.cpp:1577-1606.
static void ss_e_op_0(WoodJoint& joint) {
    auto P = [](double x, double y, double z) { return Point(x, y, z); };
    const double a = 0.357142857142857;   // 5/14
    const double b = 0.214285714285714;   // 3/14
    const double c = 0.0714285714285715;  // 1/14
    // Female top face (x = +0.5): 12-pt zigzag profile + 2-pt edge marker.
    Polyline f0_outline(std::vector<Point>{
        P( 0.5,  0.5, -a), P( 0.5, -0.5, -a),
        P( 0.5, -0.5, -b), P( 0.5,  0.5, -b),
        P( 0.5,  0.5, -c), P( 0.5, -0.5, -c),
        P( 0.5, -0.5,  c), P( 0.5,  0.5,  c),
        P( 0.5,  0.5,  b), P( 0.5, -0.5,  b),
        P( 0.5, -0.5,  a), P( 0.5,  0.5,  a)
    });
    Polyline f0_endpoints(std::vector<Point>{ P( 0.5, 0.5, -0.5), P( 0.5, 0.5, 0.5) });
    joint.f_outlines[0] = { f0_outline, f0_endpoints };

    Polyline f1_outline(std::vector<Point>{
        P(-0.5,  0.5, -a), P(-0.5, -0.5, -a),
        P(-0.5, -0.5, -b), P(-0.5,  0.5, -b),
        P(-0.5,  0.5, -c), P(-0.5, -0.5, -c),
        P(-0.5, -0.5,  c), P(-0.5,  0.5,  c),
        P(-0.5,  0.5,  b), P(-0.5, -0.5,  b),
        P(-0.5, -0.5,  a), P(-0.5,  0.5,  a)
    });
    Polyline f1_endpoints(std::vector<Point>{ P(-0.5, 0.5, -0.5), P(-0.5, 0.5, 0.5) });
    joint.f_outlines[1] = { f1_outline, f1_endpoints };

    Polyline m0_outline(std::vector<Point>{
        P(-0.5,  0.5,  a), P( 0.5,  0.5,  a),
        P( 0.5,  0.5,  b), P(-0.5,  0.5,  b),
        P(-0.5,  0.5,  c), P( 0.5,  0.5,  c),
        P( 0.5,  0.5, -c), P(-0.5,  0.5, -c),
        P(-0.5,  0.5, -b), P( 0.5,  0.5, -b),
        P( 0.5,  0.5, -a), P(-0.5,  0.5, -a)
    });
    Polyline m0_endpoints(std::vector<Point>{ P(-0.5, 0.5, 0.5), P(-0.5, 0.5, -0.5) });
    joint.m_outlines[0] = { m0_outline, m0_endpoints };

    Polyline m1_outline(std::vector<Point>{
        P(-0.5, -0.5,  a), P( 0.5, -0.5,  a),
        P( 0.5, -0.5,  b), P(-0.5, -0.5,  b),
        P(-0.5, -0.5,  c), P( 0.5, -0.5,  c),
        P( 0.5, -0.5, -c), P(-0.5, -0.5, -c),
        P(-0.5, -0.5, -b), P( 0.5, -0.5, -b),
        P( 0.5, -0.5, -a), P(-0.5, -0.5, -a)
    });
    Polyline m1_endpoints(std::vector<Point>{ P(-0.5, -0.5, 0.5), P(-0.5, -0.5, -0.5) });
    joint.m_outlines[1] = { m1_outline, m1_endpoints };
    // Wood: f/m boolean types = {edge_insertion, edge_insertion}
    // (wood_joint_lib.cpp:1604-1605).
    joint.f_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.f_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
}

// ss_e_op_1: side-to-side out-of-plane PARAMETRIC finger joint (type=11,
// joint name id=10). Verbatim port of wood_joint_lib.cpp:1609-1751.
// Creates parametric geometry in the [-0.5, 0.5]³ unit cube. Default annen
// dispatch target.
static void ss_e_op_1(WoodJoint& joint) {
    int div = std::max(2, std::min(20, joint.divisions));
    div += div % 2; // force even

    // Interpolate 4 vertical edge lines of the unit cube.
    auto arr0 = interpolate_points(Point(-0.5, 0.5,-0.5), Point(-0.5, 0.5, 0.5), div);
    auto arr1 = interpolate_points(Point( 0.5, 0.5,-0.5), Point( 0.5, 0.5, 0.5), div);
    auto arr2 = interpolate_points(Point( 0.5,-0.5,-0.5), Point( 0.5,-0.5, 0.5), div);
    auto arr3 = interpolate_points(Point(-0.5,-0.5,-0.5), Point(-0.5,-0.5, 0.5), div);
    std::vector<Point>* arrays[4] = {&arr0, &arr1, &arr2, &arr3};

    // Shift: remap [0,1] → [-0.5,0.5], scale by 1/(div+1).
    double vz = (joint.shift == 0) ? 0.0
        : (joint.shift * 1.0 - 0.5) / (div + 1);
    Vector v(0, 0, vz);
    for (int i = 0; i < 4; i++) {
        auto& a = *arrays[i];
        for (int j = 0; j < (int)a.size(); j++) {
            bool flip = (j % 2 == 0);
            if (i >= 2) flip = !flip;
            if (flip) a[j] = Point(a[j][0]+v[0], a[j][1]+v[1], a[j][2]+v[2]);
        }
    }

    // Build male polylines. Wood's `ss_e_op_1` (`wood_joint_lib.cpp:1688-1703`)
    // writes `i=0 → joint.m[1]` and `i=2 → joint.m[0]` — the labels are
    // intentionally swapped from what `(i < 2) ? 0 : 1` would suggest. The
    // merge's `is_geo_reversed` test compensates either way, but we match
    // wood verbatim per CLAUDE.md ("identical APIs, variable names, test
    // logic, line counts").
    for (int i = 0; i < 4; i += 2) {
        std::vector<Point> pts;
        pts.reserve(arr0.size() * 2);
        auto& aA = *arrays[i];
        auto& aB = *arrays[i+1];
        for (int j = 0; j < (int)aA.size(); j++) {
            bool flip = (j % 2 == 0);
            if (i >= 2) flip = !flip;
            if (flip) { pts.push_back(aA[j]); pts.push_back(aB[j]); }
            else      { pts.push_back(aB[j]); pts.push_back(aA[j]); }
        }
        Polyline outline(pts);
        Polyline endpoints(std::vector<Point>{pts.front(), pts.back()});
        int idx = (i < 2) ? 1 : 0;
        joint.m_outlines[idx] = {outline, endpoints};
    }

    // Build female polylines (f[0]=top pair, f[1]=bottom pair).
    for (int i = 1; i < 4; i += 2) {
        std::vector<Point> pts;
        pts.reserve(arr0.size() * 2);
        auto& aA = *arrays[i];
        auto& aB = *arrays[(i+1)%4];
        for (int j = 0; j < (int)aA.size(); j++) {
            bool flip = (j % 2 == 0);
            if (i >= 2) flip = !flip;
            if (flip) { pts.push_back(aA[j]); pts.push_back(aB[j]); }
            else      { pts.push_back(aB[j]); pts.push_back(aA[j]); }
        }
        Polyline outline(pts);
        Polyline endpoints(std::vector<Point>{pts.front(), pts.back()});
        int idx = (i < 2) ? 0 : 1;
        joint.f_outlines[idx] = {outline, endpoints};
    }
    // Wood: f/m boolean types = {edge_insertion, edge_insertion}
    // (wood_joint_lib.cpp:1746-1747).
    joint.f_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.f_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
}

// ss_e_op_2: side-to-side out-of-plane parametric (type=11, joint name id=11).
// Verbatim port of wood_joint_lib.cpp:1754-1898. Same overall structure as
// ss_e_op_1 — interpolate 4 vertical edges of the unit cube, then build
// `m[0]/m[1]` and `f[0]/f[1]` zigzag outlines from pairs of arrays — but
// the "move segments" loop uses a different non-uniform shift pattern that
// scales the central two pairs more than the outer pairs (4*v vs 2*v) and
// flips sign across the polyline midpoint.
static void ss_e_op_2(WoodJoint& joint) {
    int div = std::max(4, std::min(20, joint.divisions));
    div += div % 2; // force even

    auto arr0 = interpolate_points(Point( 0.5,-0.5,-0.5), Point( 0.5,-0.5, 0.5), div);
    auto arr1 = interpolate_points(Point(-0.5,-0.5,-0.5), Point(-0.5,-0.5, 0.5), div);
    auto arr2 = interpolate_points(Point(-0.5, 0.5,-0.5), Point(-0.5, 0.5, 0.5), div);
    auto arr3 = interpolate_points(Point( 0.5, 0.5,-0.5), Point( 0.5, 0.5, 0.5), div);
    std::vector<Point>* arrays[4] = {&arr0, &arr1, &arr2, &arr3};

    double vz = (joint.shift == 0)
        ? 0.0
        : Intersection::remap(joint.shift, 0, 1.0, -0.5, 0.5) / (div + 1);
    Vector v(0, 0, vz);
    for (int i = 0; i < 4; i++) {
        auto& a = *arrays[i];
        int mid = (int)(a.size() * 0.5);
        for (int j = 0; j < (int)a.size(); j++) {
            int flip = (j < mid) ? 1 : -1;
            if (i == 1) {
                if (j < mid - 1 || j > mid)
                    a[j] = Point(a[j][0]-4*v[0]*flip, a[j][1]-4*v[1]*flip, a[j][2]-4*v[2]*flip);
            } else if (i == 0 || i == 2) {
                if (j < mid - 1 || j > mid)
                    a[j] = Point(a[j][0]-2*v[0]*flip, a[j][1]-2*v[1]*flip, a[j][2]-2*v[2]*flip);
            }
        }
    }

    // Build male outlines (m[1] from arr0/arr1, m[0] from arr2/arr3 — wood
    // labels them swapped, see fix #5 in project_main_5_annen_pipeline.md)
    for (int i = 0; i < 4; i += 2) {
        std::vector<Point> pts;
        pts.reserve(arr0.size() * 2);
        auto& aA = *arrays[i];
        auto& aB = *arrays[i+1];
        for (int j = 0; j < (int)aA.size(); j++) {
            bool flip = (j % 2 == 0);
            if (i >= 2) flip = !flip;
            if (flip) { pts.push_back(aA[j]); pts.push_back(aB[j]); }
            else      { pts.push_back(aB[j]); pts.push_back(aA[j]); }
        }
        Polyline outline(pts);
        Polyline endpoints(std::vector<Point>{pts.front(), pts.back()});
        int idx = (i < 2) ? 1 : 0;
        joint.m_outlines[idx] = {outline, endpoints};
    }

    // Build female outlines (f[0] from arr1/arr2, f[1] from arr3/arr0)
    for (int i = 1; i < 4; i += 2) {
        std::vector<Point> pts;
        pts.reserve(arr0.size() * 2);
        auto& aA = *arrays[i];
        auto& aB = *arrays[(i+1)%4];
        for (int j = 0; j < (int)aA.size(); j++) {
            bool flip = (j % 2 == 0);
            if (i >= 2) flip = !flip;
            if (flip) { pts.push_back(aA[j]); pts.push_back(aB[j]); }
            else      { pts.push_back(aB[j]); pts.push_back(aA[j]); }
        }
        Polyline outline(pts);
        Polyline endpoints(std::vector<Point>{pts.front(), pts.back()});
        int idx = (i < 2) ? 0 : 1;
        joint.f_outlines[idx] = {outline, endpoints};
    }
    // Wood: f/m boolean types = {edge_insertion, edge_insertion}
    // (wood_joint_lib.cpp:1896-1897).
    joint.f_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.f_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
}

// ─────────────────────────────────────────────────────────────────────────────
// group 2 — ts_e_p (top-side, type 20)
// ─────────────────────────────────────────────────────────────────────────────

// ts_e_p_0: top-to-side HARDCODED 3-finger tenon-mortise (type 20, joint
// name id=23). Verbatim port of wood_joint_lib.cpp:3527-3564.
//
// f[0] = 4 polylines (3 mortise rectangles + 1 bounding rectangle) at y=-0.5
// f[1] = 4 polylines (3 mortise rectangles + 1 bounding rectangle) at y=+0.5
// m[0] = 12-pt zigzag at x=+0.5 with 2-pt endpoint marker
// m[1] = 12-pt zigzag at x=-0.5 with 2-pt endpoint marker
// boolean: f = hole×4, m = edge_insertion×2.
static void ts_e_p_0(WoodJoint& joint) {
    auto P = [](double x, double y, double z) { return Point(x, y, z); };
    const double a = 0.357142857142857;   // 5/14
    const double b = 0.214285714285714;   // 3/14
    const double c = 0.0714285714285715;  // 1/14

    // f[0] — 3 mortise holes + 1 bounding rectangle at y = -0.5
    joint.f_outlines[0].clear();
    joint.f_outlines[0].push_back(Polyline(std::vector<Point>{
        P(-0.5,-0.5, a), P( 0.5,-0.5, a), P( 0.5,-0.5, b), P(-0.5,-0.5, b), P(-0.5,-0.5, a)}));
    joint.f_outlines[0].push_back(Polyline(std::vector<Point>{
        P(-0.5,-0.5, c), P( 0.5,-0.5, c), P( 0.5,-0.5,-c), P(-0.5,-0.5,-c), P(-0.5,-0.5, c)}));
    joint.f_outlines[0].push_back(Polyline(std::vector<Point>{
        P(-0.5,-0.5,-b), P( 0.5,-0.5,-b), P( 0.5,-0.5,-a), P(-0.5,-0.5,-a), P(-0.5,-0.5,-b)}));
    joint.f_outlines[0].push_back(Polyline(std::vector<Point>{
        P(-0.5,-0.5, a), P(-0.5,-0.5,-a), P( 0.5,-0.5,-a), P( 0.5,-0.5, a), P(-0.5,-0.5, a)}));

    // f[1] — 3 mortise holes + 1 bounding rectangle at y = +0.5
    joint.f_outlines[1].clear();
    joint.f_outlines[1].push_back(Polyline(std::vector<Point>{
        P(-0.5, 0.5, a), P( 0.5, 0.5, a), P( 0.5, 0.5, b), P(-0.5, 0.5, b), P(-0.5, 0.5, a)}));
    joint.f_outlines[1].push_back(Polyline(std::vector<Point>{
        P(-0.5, 0.5, c), P( 0.5, 0.5, c), P( 0.5, 0.5,-c), P(-0.5, 0.5,-c), P(-0.5, 0.5, c)}));
    joint.f_outlines[1].push_back(Polyline(std::vector<Point>{
        P(-0.5, 0.5,-b), P( 0.5, 0.5,-b), P( 0.5, 0.5,-a), P(-0.5, 0.5,-a), P(-0.5, 0.5,-b)}));
    joint.f_outlines[1].push_back(Polyline(std::vector<Point>{
        P(-0.5, 0.5, a), P(-0.5, 0.5,-a), P( 0.5, 0.5,-a), P( 0.5, 0.5, a), P(-0.5, 0.5, a)}));

    // m[0] — 12-pt zigzag at x = +0.5
    Polyline m0_outline(std::vector<Point>{
        P( 0.5,-0.5,-a), P( 0.5, 0.5,-a),
        P( 0.5, 0.5,-b), P( 0.5,-0.5,-b),
        P( 0.5,-0.5,-c), P( 0.5, 0.5,-c),
        P( 0.5, 0.5, c), P( 0.5,-0.5, c),
        P( 0.5,-0.5, b), P( 0.5, 0.5, b),
        P( 0.5, 0.5, a), P( 0.5,-0.5, a)
    });
    Polyline m0_endpoints(std::vector<Point>{ P( 0.5,-0.5,-a), P( 0.5,-0.5, a) });
    joint.m_outlines[0] = { m0_outline, m0_endpoints };

    // m[1] — 12-pt zigzag at x = -0.5
    Polyline m1_outline(std::vector<Point>{
        P(-0.5,-0.5,-a), P(-0.5, 0.5,-a),
        P(-0.5, 0.5,-b), P(-0.5,-0.5,-b),
        P(-0.5,-0.5,-c), P(-0.5, 0.5,-c),
        P(-0.5, 0.5, c), P(-0.5,-0.5, c),
        P(-0.5,-0.5, b), P(-0.5, 0.5, b),
        P(-0.5, 0.5, a), P(-0.5,-0.5, a)
    });
    Polyline m1_endpoints(std::vector<Point>{ P(-0.5,-0.5,-a), P(-0.5,-0.5, a) });
    joint.m_outlines[1] = { m1_outline, m1_endpoints };
    // Wood: f_boolean_type = {hole, hole, hole, insert_between_multiple_edges}×2
    //       m_boolean_type = {edge_insertion, edge_insertion}
    // (wood_joint_lib.cpp:3562-3563). Note: f_outlines[face] has 4 entries
    // (3 mortise holes + 1 bounding rectangle).
    joint.f_cut_types[0] = { wood_cut::hole, wood_cut::hole, wood_cut::hole, wood_cut::insert_between_multiple_edges };
    joint.f_cut_types[1] = { wood_cut::hole, wood_cut::hole, wood_cut::hole, wood_cut::insert_between_multiple_edges };
    joint.m_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
}

// ts_e_p_1: top-to-side HARDCODED 3-finger tenon-mortise (type=20, joint
// name id=21). Verbatim port of wood_joint_lib.cpp:3567-3601 — every point
// is a literal value from wood, no parametrization. Female (top of female
// element) outlines have 3 polylines = 2 mortise rectangles + 1 bounding
// rectangle. Male outlines = 8-point zigzag (the male tenon profile) +
// 2-point endpoint marker.
//
// NOTE: Wood's dispatcher does not currently route to ts_e_p_1
// (`wood_joint_lib.cpp:6395-6448` has no `case (21): ts_e_p_1` line). Kept
// here as a complete port; wire it into `joint_create_geometry` if a
// dataset needs it.
static void ts_e_p_1(WoodJoint& joint) {
    auto P = [](double x, double y, double z) { return Point(x, y, z); };
    const double z_top   = 0.166666666666667;
    const double z_top2  = 0.0555555555555556;
    const double z_bot   = -0.277777777777778;
    const double z_bot2  = -0.388888888888889;

    // f[0] at y=-0.5: 2 mortise holes + 1 bounding rectangle
    joint.f_outlines[0] = {
        Polyline(std::vector<Point>{
            P(-0.5,-0.5, z_bot ), P( 0.5,-0.5, z_bot ),
            P( 0.5,-0.5, z_bot2), P(-0.5,-0.5, z_bot2),
            P(-0.5,-0.5, z_bot )}),
        Polyline(std::vector<Point>{
            P(-0.5,-0.5, z_top ), P( 0.5,-0.5, z_top ),
            P( 0.5,-0.5, z_top2), P(-0.5,-0.5, z_top2),
            P(-0.5,-0.5, z_top )}),
        Polyline(std::vector<Point>{
            P(-0.5,-0.5, z_top ), P(-0.5,-0.5, z_bot2),
            P( 0.5,-0.5, z_bot2), P( 0.5,-0.5, z_top ),
            P(-0.5,-0.5, z_top )}),
    };
    // f[1] at y=+0.5: same shape, mirrored to +y
    joint.f_outlines[1] = {
        Polyline(std::vector<Point>{
            P(-0.5, 0.5, z_bot ), P( 0.5, 0.5, z_bot ),
            P( 0.5, 0.5, z_bot2), P(-0.5, 0.5, z_bot2),
            P(-0.5, 0.5, z_bot )}),
        Polyline(std::vector<Point>{
            P(-0.5, 0.5, z_top ), P( 0.5, 0.5, z_top ),
            P( 0.5, 0.5, z_top2), P(-0.5, 0.5, z_top2),
            P(-0.5, 0.5, z_top )}),
        Polyline(std::vector<Point>{
            P(-0.5, 0.5, z_top ), P(-0.5, 0.5, z_bot2),
            P( 0.5, 0.5, z_bot2), P( 0.5, 0.5, z_top ),
            P(-0.5, 0.5, z_top )}),
    };

    // m[0] at x=+0.5: 8-point male tenon zigzag + 2-pt endpoint marker
    joint.m_outlines[0] = {
        Polyline(std::vector<Point>{
            P( 0.5,-0.5, z_top ), P( 0.5, 0.5, z_top ),
            P( 0.5, 0.5, z_top2), P( 0.5,-0.5, z_top2),
            P( 0.5,-0.5, z_bot ), P( 0.5, 0.5, z_bot ),
            P( 0.5, 0.5, z_bot2), P( 0.5,-0.5, z_bot2)}),
        Polyline(std::vector<Point>{P( 0.5,-0.5, 0.5), P( 0.5,-0.5,-0.5)}),
    };
    // m[1] at x=-0.5
    joint.m_outlines[1] = {
        Polyline(std::vector<Point>{
            P(-0.5,-0.5, z_top ), P(-0.5, 0.5, z_top ),
            P(-0.5, 0.5, z_top2), P(-0.5,-0.5, z_top2),
            P(-0.5,-0.5, z_bot ), P(-0.5, 0.5, z_bot ),
            P(-0.5, 0.5, z_bot2), P(-0.5,-0.5, z_bot2)}),
        Polyline(std::vector<Point>{P(-0.5,-0.5, 0.5), P(-0.5,-0.5,-0.5)}),
    };
    // Wood: f_boolean_type = {hole, hole, insert_between_multiple_edges}×2
    //       m_boolean_type = {edge_insertion, edge_insertion}
    // (wood_joint_lib.cpp:3596-3597). f_outlines[face] has 3 entries
    // (2 mortise holes + 1 bounding rectangle).
    joint.f_cut_types[0] = { wood_cut::hole, wood_cut::hole, wood_cut::insert_between_multiple_edges };
    joint.f_cut_types[1] = { wood_cut::hole, wood_cut::hole, wood_cut::insert_between_multiple_edges };
    joint.m_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
}

// ts_e_p_2: top-to-side parametric tenon-mortise (type=20, joint name id=22).
// Verbatim port of wood_joint_lib.cpp:3604-3710. Structurally identical to
// ts_e_p_3 (already ported) except the male zigzag loop does NOT skip every
// other pair — it visits every interpolation point — and the female hole
// rectangles are built from every 4 male points (no skip). Both `m[0]/m[1]`
// have 2-pt endpoint markers; `f[0]/f[1]` are vectors of 5-point hole
// rectangles plus a bounding-rect at the end.
static void ts_e_p_2(WoodJoint& joint) {
    int div = std::max(2, std::min(20, joint.divisions));
    div += div % 2; // force even
    int size = div / 2 + 1;

    auto arr0 = interpolate_points(Point(-0.5,-0.5,-0.5), Point(-0.5,-0.5, 0.5), div);
    auto arr1 = interpolate_points(Point(-0.5, 0.5,-0.5), Point(-0.5, 0.5, 0.5), div);
    auto arr2 = interpolate_points(Point( 0.5, 0.5,-0.5), Point( 0.5, 0.5, 0.5), div);
    auto arr3 = interpolate_points(Point( 0.5,-0.5,-0.5), Point( 0.5,-0.5, 0.5), div);
    std::vector<Point>* arrays[4] = {&arr0, &arr1, &arr2, &arr3};

    double vz = (joint.shift == 0)
        ? 0.0
        : Intersection::remap(joint.shift, 0, 1.0, -0.5, 0.5) / (div + 1);
    Vector v(0, 0, vz);
    for (int i = 0; i < 4; i++) {
        auto& a = *arrays[i];
        for (int j = 0; j < (int)a.size(); j++) {
            int flip = (j % 2 == 0) ? 1 : -1;
            if (i >= 2) flip *= -1;
            a[j] = Point(a[j][0]+v[0]*flip, a[j][1]+v[1]*flip, a[j][2]+v[2]*flip);
        }
    }

    // Build male outlines — NO `j%4 > 1` skip (unlike ts_e_p_3).
    for (int i = 0; i < 4; i += 2) {
        std::vector<Point> pts;
        pts.reserve(arr0.size() * 2);
        auto& aA = *arrays[i];
        auto& aB = *arrays[i+1];
        for (int j = 0; j < (int)aA.size(); j++) {
            bool flip = (j % 2 == 0);
            if (i >= 2) flip = !flip;
            if (flip) { pts.push_back(aA[j]); pts.push_back(aB[j]); }
            else      { pts.push_back(aB[j]); pts.push_back(aA[j]); }
        }
        Polyline outline(pts);
        Polyline endpoints(std::vector<Point>{pts.front(), pts.back()});
        int idx = (i < 2) ? 1 : 0;
        joint.m_outlines[idx] = {outline, endpoints};
    }

    // Build female holes from every 4 male points (no skip), plus bounding
    // rectangle at the end. wood_joint_lib.cpp:3700-3706.
    auto& m0pts = joint.m_outlines[0][0];
    auto& m1pts = joint.m_outlines[1][0];
    int m0n = (int)m0pts.point_count();
    for (int i = 0; i + 3 < m0n; i += 4) {
        Point p00 = m0pts.get_point(i),   p03 = m0pts.get_point(i+3);
        Point p10 = m1pts.get_point(i),   p13 = m1pts.get_point(i+3);
        joint.f_outlines[0].push_back(Polyline(std::vector<Point>{p00, p03, p13, p10, p00}));
        Point p01 = m0pts.get_point(i+1), p02 = m0pts.get_point(i+2);
        Point p11 = m1pts.get_point(i+1), p12 = m1pts.get_point(i+2);
        joint.f_outlines[1].push_back(Polyline(std::vector<Point>{p01, p02, p12, p11, p01}));
    }
    if (size >= 2 && !joint.f_outlines[0].empty()) {
        auto& first0 = joint.f_outlines[0].front();
        auto& last0  = joint.f_outlines[0][joint.f_outlines[0].size()-1];
        joint.f_outlines[0].push_back(Polyline(std::vector<Point>{
            first0.get_point(0), first0.get_point(3),
            last0.get_point(3),  last0.get_point(0),  first0.get_point(0)}));
    }
    if (size >= 2 && !joint.f_outlines[1].empty()) {
        auto& first1 = joint.f_outlines[1].front();
        auto& last1  = joint.f_outlines[1][joint.f_outlines[1].size()-1];
        joint.f_outlines[1].push_back(Polyline(std::vector<Point>{
            first1.get_point(0), first1.get_point(3),
            last1.get_point(3),  last1.get_point(0),  first1.get_point(0)}));
    }
    // Wood: f_boolean_type = vector<size, hole> + insert_between_multiple_edges
    //       m_boolean_type = {edge_insertion, edge_insertion}
    // (wood_joint_lib.cpp:3708-3709). f_outlines[face] has `size-1` mortise
    // holes + 1 bounding rectangle = `size` total entries.
    auto build_f_cuts = [&](size_t entries) {
        std::vector<int> v;
        v.reserve(entries);
        for (size_t k = 0; k + 1 < entries; k++) v.push_back(wood_cut::hole);
        v.push_back(wood_cut::insert_between_multiple_edges);
        return v;
    };
    joint.f_cut_types[0] = build_f_cuts(joint.f_outlines[0].size());
    joint.f_cut_types[1] = build_f_cuts(joint.f_outlines[1].size());
    joint.m_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
}

// ts_e_p_3: top-to-side PARAMETRIC tenon-mortise (type=20, joint name id=20
// and 22; default for annen). Verbatim port of wood_joint_lib.cpp:3713-3935.
// Male: zigzag polyline with every-other-pair skip + 2-point endpoints.
// Female: rectangular holes from every 4 male points + bounding rectangle.
static void ts_e_p_3(WoodJoint& joint) {
    int div = std::max(8, std::min(100, joint.divisions));
    div -= div % 4; // force multiple of 4
    if (div == 0) return;
    int size = div / 4 + 1;

    // Interpolate 4 edge lines.
    auto arr3 = interpolate_points(Point( 0.5,-0.5,-0.5), Point( 0.5,-0.5, 0.5), div);
    auto arr0 = interpolate_points(Point(-0.5,-0.5,-0.5), Point(-0.5,-0.5, 0.5), div);
    auto arr1 = interpolate_points(Point(-0.5, 0.5,-0.5), Point(-0.5, 0.5, 0.5), div);
    auto arr2 = interpolate_points(Point( 0.5, 0.5,-0.5), Point( 0.5, 0.5, 0.5), div);
    std::vector<Point>* arrays[4] = {&arr0, &arr1, &arr2, &arr3};

    // Shift
    double vz = (joint.shift == 0) ? 0.0
        : (joint.shift * 1.0 - 0.5) / (div + 1);
    Vector v(0, 0, vz);
    for (int i = 0; i < 4; i++) {
        auto& a = *arrays[i];
        for (int j = 0; j < (int)a.size(); j++) {
            int flip = (j % 2 == 0) ? 1 : -1;
            if (i >= 2) flip *= -1;
            a[j] = Point(a[j][0]+v[0]*flip, a[j][1]+v[1]*flip, a[j][2]+v[2]*flip);
        }
    }

    // Build male outlines — skip every other pair (j%4 > 1).
    for (int i = 0; i < 4; i += 2) {
        std::vector<Point> pts;
        pts.reserve(arr0.size());
        auto& aA = *arrays[i];
        auto& aB = *arrays[i+1];
        for (int j = 0; j < (int)aA.size(); j++) {
            if (j % 4 > 1) continue; // KEY: skip every other pair
            bool flip = (j % 2 == 0);
            if (i >= 2) flip = !flip;
            if (flip) { pts.push_back(aA[j]); pts.push_back(aB[j]); }
            else      { pts.push_back(aB[j]); pts.push_back(aA[j]); }
        }
        // Add last point explicitly.
        if (i < 2) pts.push_back(aA.back());
        else       pts.push_back(aB.back());

        Polyline outline(pts);
        Polyline endpoints(std::vector<Point>{pts.front(), pts.back()});
        // Wood `ts_e_p_3` (`wood_joint_lib.cpp:3842-3858`) writes
        // `i=0 → joint.m[1]` and `i=2 → joint.m[0]`. The labels are
        // intentionally swapped from what `(i < 2) ? 0 : 1` would suggest;
        // matched verbatim per CLAUDE.md.
        int idx = (i < 2) ? 1 : 0;
        joint.m_outlines[idx] = {outline, endpoints};
    }

    // Build female holes from every 4 male points + bounding rectangle.
    auto& m0pts = joint.m_outlines[0][0];
    auto& m1pts = joint.m_outlines[1][0];
    int m0n = (int)m0pts.point_count();
    int nrects = m0n - m0n % 4;
    for (int i = 0; i < nrects; i += 4) {
        Point p00 = m0pts.get_point(i),   p03 = m0pts.get_point(i+3);
        Point p10 = m1pts.get_point(i),   p13 = m1pts.get_point(i+3);
        joint.f_outlines[0].push_back(Polyline(std::vector<Point>{p00, p03, p13, p10, p00}));
        Point p01 = m0pts.get_point(i+1), p02 = m0pts.get_point(i+2);
        Point p11 = m1pts.get_point(i+1), p12 = m1pts.get_point(i+2);
        joint.f_outlines[1].push_back(Polyline(std::vector<Point>{p01, p02, p12, p11, p01}));
    }
    // Bounding rectangle.
    if (size >= 2 && !joint.f_outlines[0].empty()) {
        auto& first0 = joint.f_outlines[0].front();
        auto& last0  = joint.f_outlines[0][joint.f_outlines[0].size()-1];
        joint.f_outlines[0].push_back(Polyline(std::vector<Point>{
            first0.get_point(0), first0.get_point(3),
            last0.get_point(3), last0.get_point(0), first0.get_point(0)}));
    }
    if (size >= 2 && !joint.f_outlines[1].empty()) {
        auto& first1 = joint.f_outlines[1].front();
        auto& last1  = joint.f_outlines[1][joint.f_outlines[1].size()-1];
        joint.f_outlines[1].push_back(Polyline(std::vector<Point>{
            first1.get_point(0), first1.get_point(3),
            last1.get_point(3), last1.get_point(0), first1.get_point(0)}));
    }
    // Wood: f_boolean_type = vector<size, hole>
    //       m_boolean_type = {edge_insertion, edge_insertion}
    // (wood_joint_lib.cpp:3873-3874). NOTE: wood populates ALL slots with
    // `hole` for ts_e_p_3 (no special insert_between_multiple_edges for the
    // bounding rect — its size==`size` and the merge's "skip last" loop
    // explicitly drops it). For consistency with ts_e_p_2 we still tag the
    // last entry as insert_between_multiple_edges so the cut-type-aware
    // merge in main_5.cpp can use a single uniform iteration rule.
    {
        std::vector<int> v0; v0.reserve(joint.f_outlines[0].size());
        for (size_t k = 0; k + 1 < joint.f_outlines[0].size(); k++)
            v0.push_back(wood_cut::hole);
        if (!joint.f_outlines[0].empty())
            v0.push_back(wood_cut::insert_between_multiple_edges);
        joint.f_cut_types[0] = std::move(v0);

        std::vector<int> v1; v1.reserve(joint.f_outlines[1].size());
        for (size_t k = 0; k + 1 < joint.f_outlines[1].size(); k++)
            v1.push_back(wood_cut::hole);
        if (!joint.f_outlines[1].empty())
            v1.push_back(wood_cut::insert_between_multiple_edges);
        joint.f_cut_types[1] = std::move(v1);
    }
    joint.m_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
}

// ─── ss_e_ip_3 ──────────────────────────────────────────────────────────────
// Verbatim port of wood_joint_lib.cpp:970-1116. Hardcoded mill+drill in-plane
// joint with 6 outlines per face (2 mill_project + 4 drill).
static void ss_e_ip_3(WoodJoint& joint) {
    joint.f_outlines[0] = {
        Polyline({Point(-1.25,-0.5,-0.5), Point(1,-0.5,-0.5), Point(1,-0.2,-0.5), Point(-1,0.2,-0.5), Point(-1,0.5,-0.5), Point(-1.25,0.5,-0.5), Point(-1.25,-0.5,-0.5)}),
        Polyline({Point(0,-0.5,0.5), Point(0,-0.5,-0.5)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
    };
    joint.f_outlines[1] = {
        Polyline({Point(-1.25,-0.5,0.5), Point(1,-0.5,0.5), Point(1,-0.2,0.5), Point(-1,0.2,0.5), Point(-1,0.5,0.5), Point(-1.25,0.5,0.5), Point(-1.25,-0.5,0.5)}),
        Polyline({Point(0,0.5,0.5), Point(0,0.5,-0.5)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
    };
    joint.m_outlines[0] = {
        Polyline({Point(1.25,0.5,-0.5), Point(-1,0.5,-0.5), Point(-1,0.2,-0.5), Point(1,-0.2,-0.5), Point(1,-0.5,-0.5), Point(1.25,-0.5,-0.5), Point(1.25,0.5,-0.5)}),
        Polyline({Point(0,-0.5,0.5), Point(0,-0.5,-0.5)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
    };
    joint.m_outlines[1] = {
        Polyline({Point(1.25,0.5,0.5), Point(-1,0.5,0.5), Point(-1,0.2,0.5), Point(1,-0.2,0.5), Point(1,-0.5,0.5), Point(1.25,-0.5,0.5), Point(1.25,0.5,0.5)}),
        Polyline({Point(0,0.5,0.5), Point(0,0.5,-0.5)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
    };
    for (int face = 0; face < 2; face++) {
        joint.f_cut_types[face] = { wood_cut::mill_project, wood_cut::mill_project, wood_cut::drill, wood_cut::drill, wood_cut::drill, wood_cut::drill };
        joint.m_cut_types[face] = { wood_cut::mill_project, wood_cut::mill_project, wood_cut::drill, wood_cut::drill, wood_cut::drill, wood_cut::drill };
    }
}

// ─── ss_e_ip_4 ──────────────────────────────────────────────────────────────
// Verbatim port of wood_joint_lib.cpp:1118-1317. Hardcoded mill+drill in-plane
// joint with 8 outlines per face (4 mill_project + 4 drill).
static void ss_e_ip_4(WoodJoint& joint) {
    joint.f_outlines[0] = {
        Polyline({Point(-1.25,-0.5,0), Point(1,-0.5,0), Point(1,-0.2,0), Point(-1,0.2,0), Point(-1,0.5,0), Point(-1.25,0.5,0), Point(-1.25,-0.5,0)}),
        Polyline({Point(0,-0.5,0.5), Point(0,-0.5,-0.5)}),
        Polyline({Point(-1.25,0.5,0), Point(1,0.5,0), Point(1,0.2,0), Point(-1,-0.2,0), Point(-1,-0.5,0), Point(-1.25,-0.5,0), Point(-1.25,0.5,0)}),
        Polyline({Point(0,-0.5,0.5), Point(0,-0.5,-0.5)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
    };
    joint.f_outlines[1] = {
        Polyline({Point(-1.25,-0.5,-0.5), Point(1,-0.5,-0.5), Point(1,-0.2,-0.5), Point(-1,0.2,-0.5), Point(-1,0.5,-0.5), Point(-1.25,0.5,-0.5), Point(-1.25,-0.5,-0.5)}),
        Polyline({Point(0,-0.5,0.5), Point(0,-0.5,-0.5)}),
        Polyline({Point(-1.25,0.5,0.5), Point(1,0.5,0.5), Point(1,0.2,0.5), Point(-1,-0.2,0.5), Point(-1,-0.5,0.5), Point(-1.25,-0.5,0.5), Point(-1.25,0.5,0.5)}),
        Polyline({Point(0,-0.5,0.5), Point(0,-0.5,-0.5)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
    };
    joint.m_outlines[0] = {
        Polyline({Point(1.25,0.5,0), Point(-1,0.5,0), Point(-1,0.2,0), Point(1,-0.2,0), Point(1,-0.5,0), Point(1.25,-0.5,0), Point(1.25,0.5,0)}),
        Polyline({Point(0,-0.5,0.5), Point(0,-0.5,-0.5)}),
        Polyline({Point(1.25,-0.5,0), Point(-1,-0.5,0), Point(-1,-0.2,0), Point(1,0.2,0), Point(1,0.5,0), Point(1.25,0.5,0), Point(1.25,-0.5,0)}),
        Polyline({Point(0,-0.5,0.5), Point(0,-0.5,-0.5)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
    };
    joint.m_outlines[1] = {
        Polyline({Point(1.25,0.5,-0.5), Point(-1,0.5,-0.5), Point(-1,0.2,-0.5), Point(1,-0.2,-0.5), Point(1,-0.5,-0.5), Point(1.25,-0.5,-0.5), Point(1.25,0.5,-0.5)}),
        Polyline({Point(0,-0.5,0.5), Point(0,-0.5,-0.5)}),
        Polyline({Point(1.25,-0.5,0.5), Point(-1,-0.5,0.5), Point(-1,-0.2,0.5), Point(1,0.2,0.5), Point(1,0.5,0.5), Point(1.25,0.5,0.5), Point(1.25,-0.5,0.5)}),
        Polyline({Point(0,-0.5,0.5), Point(0,-0.5,-0.5)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
    };
    for (int face = 0; face < 2; face++) {
        joint.f_cut_types[face] = { wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project, wood_cut::drill, wood_cut::drill, wood_cut::drill, wood_cut::drill };
        joint.m_cut_types[face] = { wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project, wood_cut::drill, wood_cut::drill, wood_cut::drill, wood_cut::drill };
    }
}

// ─── ss_e_op_3 ──────────────────────────────────────────────────────────────
// Verbatim port of wood_joint_lib.cpp:1901-2003. Miter tenon-mortise with
// 4 female outlines (2 insert_between + 2 hole) and 2 male outlines
// (2 insert_between).
static void ss_e_op_3(WoodJoint& joint) {
    joint.f_outlines[0] = {
        Polyline({Point(0.5,0.5,0.3), Point(0.5,-1.499975,0.3), Point(0.5,-1.499975,-0.3), Point(0.5,0.5,-0.3)}),
        Polyline({Point(0.5,0.5,0.3), Point(0.5,0.5,-0.3)}),
        Polyline({Point(0.5,-0.5,0.25), Point(0.5,0.5,0.25), Point(0.5,0.5,-0.25), Point(0.5,-0.5,-0.25), Point(0.5,-0.5,0.25)}),
        Polyline({Point(0.5,-0.5,0.25), Point(0.5,0.5,0.25), Point(0.5,0.5,-0.25), Point(0.5,-0.5,-0.25), Point(0.5,-0.5,0.25)}),
    };
    joint.f_outlines[1] = {
        Polyline({Point(-0.5,-0.499975,0.3), Point(-0.5,-1.49995,0.3), Point(-0.5,-1.49995,-0.3), Point(-0.5,-0.5,-0.3)}),
        Polyline({Point(-0.5,-0.499975,0.3), Point(-0.5,-0.5,-0.3)}),
        Polyline({Point(-0.5,-0.499975,0.25), Point(-0.5,0.500025,0.25), Point(-0.5,0.500025,-0.25), Point(-0.5,-0.499975,-0.25), Point(-0.5,-0.499975,0.25)}),
        Polyline({Point(-0.5,-0.499975,0.25), Point(-0.5,0.500025,0.25), Point(-0.5,0.500025,-0.25), Point(-0.5,-0.499975,-0.25), Point(-0.5,-0.499975,0.25)}),
    };
    joint.m_outlines[0] = {
        Polyline({Point(-0.5,-0.5,0.3), Point(0.5,-0.5,0.3), Point(0.5,-0.5,0.25), Point(-1,-0.5,0.25), Point(-1,-0.5,-0.25), Point(0.5,-0.5,-0.25), Point(0.5,-0.5,-0.3), Point(-0.5,-0.5,-0.3)}),
        Polyline({Point(-0.5,-0.5,0.3), Point(-0.5,-0.5,-0.3)}),
    };
    joint.m_outlines[1] = {
        Polyline({Point(0.5,0.5,0.3), Point(0.510075,0.5,0.3), Point(0.5,0.5,0.25), Point(-1,0.5,0.25), Point(-1,0.5,-0.25), Point(0.5,0.5,-0.25), Point(0.510075,0.5,-0.3), Point(0.5,0.5,-0.3)}),
        Polyline({Point(0.5,0.5,0.3), Point(0.5,0.5,-0.3)}),
    };
    for (int face = 0; face < 2; face++) {
        joint.f_cut_types[face] = { wood_cut::insert_between_multiple_edges, wood_cut::insert_between_multiple_edges, wood_cut::hole, wood_cut::hole };
        joint.m_cut_types[face] = { wood_cut::insert_between_multiple_edges, wood_cut::insert_between_multiple_edges };
    }
}

// ─── cr_c_ip_0 ──────────────────────────────────────────────────────────────
// Verbatim port of wood_joint_lib.cpp:4583-4602. Trivial cross-joint stub:
// each face has 2 identical closed rectangles.
static void cr_c_ip_0(WoodJoint& joint) {
    double s = 1.0;
    joint.f_outlines[0] = {
        Polyline({Point(-0.5,0.5,s), Point(-0.5,-0.5,s), Point(-0.5,-0.5,0), Point(-0.5,0.5,0), Point(-0.5,0.5,s)}),
        Polyline({Point(-0.5,0.5,s), Point(-0.5,-0.5,s), Point(-0.5,-0.5,0), Point(-0.5,0.5,0), Point(-0.5,0.5,s)}),
    };
    joint.f_outlines[1] = {
        Polyline({Point(0.5,0.5,s), Point(0.5,-0.5,s), Point(0.5,-0.5,0), Point(0.5,0.5,0), Point(0.5,0.5,s)}),
        Polyline({Point(0.5,0.5,s), Point(0.5,-0.5,s), Point(0.5,-0.5,0), Point(0.5,0.5,0), Point(0.5,0.5,s)}),
    };
    joint.m_outlines[0] = {
        Polyline({Point(0.5,0.5,-s), Point(-0.5,0.5,-s), Point(-0.5,0.5,0), Point(0.5,0.5,0), Point(0.5,0.5,-s)}),
        Polyline({Point(0.5,0.5,-s), Point(-0.5,0.5,-s), Point(-0.5,0.5,0), Point(0.5,0.5,0), Point(0.5,0.5,-s)}),
    };
    joint.m_outlines[1] = {
        Polyline({Point(0.5,-0.5,-s), Point(-0.5,-0.5,-s), Point(-0.5,-0.5,0), Point(0.5,-0.5,0), Point(0.5,-0.5,-s)}),
        Polyline({Point(0.5,-0.5,-s), Point(-0.5,-0.5,-s), Point(-0.5,-0.5,0), Point(0.5,-0.5,0), Point(0.5,-0.5,-s)}),
    };
    for (int face = 0; face < 2; face++) {
        joint.f_cut_types[face] = { wood_cut::insert_between_multiple_edges, wood_cut::insert_between_multiple_edges };
        joint.m_cut_types[face] = { wood_cut::insert_between_multiple_edges, wood_cut::insert_between_multiple_edges };
    }
}

// ─── cr_c_ip_1 ────────────────────────────────���─────────────────────────────
// Verbatim port of wood_joint_lib.cpp:4605-4717. Parametric cross-joint with
// 9 base polylines (center + 2 top sides + 2 bot sides + 4 corners), offset
// along normals, rotated for male, then duplicated 2×. 18 total outlines per
// face. Cut types: 6× mill_project + 12× slice = 18 entries.
static void cr_c_ip_1(WoodJoint& joint) {
    double s_param = std::max(std::min(joint.shift, 1.0), 0.0);
    s_param = 0.05 + (s_param - 0.0) * (0.4 - 0.05) / (1.0 - 0.0);

    double a = 0.5 - s_param;
    double b = 0.5;
    double c = 2.0 * (b - a);
    double z = 0.5;

    // 16 control points: center(4), center_offset(4), top(4), bottom(4)
    Point p[16] = {
        Point(a, -a, 0),          Point(-a, -a, 0),          Point(-a, a, 0),          Point(a, a, 0),
        Point(a+c, -a-c, 0),      Point(-a-c, -a-c, 0),      Point(-a-c, a+c, 0),      Point(a+c, a+c, 0),
        Point(b, -b, z),          Point(-b, -b, z),          Point(-b, b, z),          Point(b, b, z),
        Point(b, -b, -z),         Point(-b, -b, -z),         Point(-b, b, -z),         Point(b, b, -z),
    };

    // v0 = half-width offset along edge direction
    double inv_2a = (a > 1e-12) ? 1.0 / (a * 2.0) : 0.0;
    Vector d01(p[0][0]-p[1][0], p[0][1]-p[1][1], p[0][2]-p[1][2]);
    Vector v0(d01[0]*inv_2a*(0.5-a), d01[1]*inv_2a*(0.5-a), d01[2]*inv_2a*(0.5-a));

    auto add_v = [](const Point& pt, const Vector& v) { return Point(pt[0]+v[0], pt[1]+v[1], pt[2]+v[2]); };
    auto sub_v = [](const Point& pt, const Vector& v) { return Point(pt[0]-v[0], pt[1]-v[1], pt[2]-v[2]); };

    // 9 base polylines for f[0]
    std::vector<Polyline> base(9);
    base[0] = Polyline({add_v(p[0],v0), sub_v(p[1],v0), sub_v(p[2],v0), add_v(p[3],v0), add_v(p[0],v0)});
    base[1] = Polyline({sub_v(p[1],v0), add_v(p[0],v0), add_v(p[8],v0), sub_v(p[9],v0), sub_v(p[1],v0)});
    base[2] = Polyline({add_v(p[3],v0), sub_v(p[2],v0), sub_v(p[10],v0), add_v(p[11],v0), add_v(p[3],v0)});
    base[3] = Polyline({p[2], p[1], p[13], p[14], p[2]});
    base[4] = Polyline({p[0], p[3], p[15], p[12], p[0]});
    base[5] = Polyline({p[0], p[12], p[4], p[8], p[0]});
    base[6] = Polyline({p[13], p[1], p[9], p[5], p[13]});
    base[7] = Polyline({p[2], p[14], p[6], p[10], p[2]});
    base[8] = Polyline({p[15], p[3], p[11], p[7], p[15]});

    // Offset lengths per polyline (normal · length)
    double lengths[9] = { 0.5, 0.4, 0.4, 0.4, 0.4, 0.1, 0.1, 0.1, 0.1 };

    // rotation_in_xy_plane(0,1,0; 1,0,0; 0,0,-1) — swaps X↔Y and negates Z
    Xform xf_rot = Xform::from_axes(Vector(0,1,0), Vector(1,0,0), Vector(0,0,-1));

    // Build f[1] by offsetting f[0] along face normals, m[0]/m[1] by rotating
    std::vector<Polyline> f0 = base;
    std::vector<Polyline> f1(9);
    std::vector<Polyline> m0(9);
    std::vector<Polyline> m1(9);

    for (int i = 0; i < 9; i++) {
        f1[i] = base[i]; // copy
        // offset along cross-product normal
        Point pa = f1[i].get_point(0);
        Point pb = f1[i].get_point(1);
        Point pc = f1[i].get_point(2);
        Vector ab(pb[0]-pa[0], pb[1]-pa[1], pb[2]-pa[2]);
        Vector cb(pb[0]-pc[0], pb[1]-pc[1], pb[2]-pc[2]);
        Vector cross = ab.cross(cb);
        cross.normalize_self();
        for (size_t j = 0; j < f1[i].point_count(); j++) {
            Point pt = f1[i].get_point(j);
            f1[i].set_point(j, Point(pt[0]+cross[0]*lengths[i], pt[1]+cross[1]*lengths[i], pt[2]+cross[2]*lengths[i]));
        }

        m0[i] = base[i].transformed_xform(xf_rot);
        m1[i] = f1[i].transformed_xform(xf_rot);
    }

    // Duplicate each polyline 2× (wood pattern: each outline appears twice)
    joint.f_outlines[0].clear(); joint.f_outlines[1].clear();
    joint.m_outlines[0].clear(); joint.m_outlines[1].clear();
    for (int i = 0; i < 9; i++) {
        joint.f_outlines[0].push_back(f0[i]); joint.f_outlines[0].push_back(f0[i]);
        joint.f_outlines[1].push_back(f1[i]); joint.f_outlines[1].push_back(f1[i]);
        joint.m_outlines[0].push_back(m0[i]); joint.m_outlines[0].push_back(m0[i]);
        joint.m_outlines[1].push_back(m1[i]); joint.m_outlines[1].push_back(m1[i]);
    }

    // 18 cut types: 6× mill_project + 12× slice
    std::vector<int> ct(18);
    for (int i = 0; i < 6; i++) ct[i] = wood_cut::mill_project;
    for (int i = 6; i < 18; i++) ct[i] = wood_cut::slice;
    for (int face = 0; face < 2; face++) {
        joint.f_cut_types[face] = ct;
        joint.m_cut_types[face] = ct;
    }
}

// ─── cr_c_ip_2..5 shared core ──────────────────────────────────────────────
// Port of wood_joint_lib.cpp:4720-5475. All four variants share the same
// 16-point layout, rotation, offset, duplication, and side-face
// reconstruction. Differences: drill polylines, extend factors, offset skip
// condition, cut types.
static void cr_c_ip_shared(WoodJoint& joint,
                            const std::vector<Polyline>& extra_drills,
                            double ext_side, double ext_vert,
                            double ext_side_factor_a, double ext_side_factor_b,
                            size_t offset_min_pts,
                            const std::vector<int>& cut_types) {
    double s_param = std::max(std::min(joint.shift, 1.0), 0.0);
    s_param = 0.05 + (s_param - 0.0) * (0.4 - 0.05) / (1.0 - 0.0);
    double a = 0.5 - s_param;
    double b = 0.5;
    double c = 2.0 * (b - a);
    double z = 0.5;

    Point p[16] = {
        Point(a, -a, 0),      Point(-a, -a, 0),      Point(-a, a, 0),      Point(a, a, 0),
        Point(a+c, -a-c, 0),  Point(-a-c, -a-c, 0),  Point(-a-c, a+c, 0),  Point(a+c, a+c, 0),
        Point(b, -b, z),      Point(-b, -b, z),      Point(-b, b, z),      Point(b, b, z),
        Point(b, -b, -z),     Point(-b, -b, -z),     Point(-b, b, -z),     Point(b, b, -z),
    };

    double inv_2a = (a > 1e-12) ? 1.0 / (a * 2.0) : 0.0;
    Vector d01(p[0][0]-p[1][0], p[0][1]-p[1][1], p[0][2]-p[1][2]);
    Vector v0(d01[0]*inv_2a*(0.5-a), d01[1]*inv_2a*(0.5-a), d01[2]*inv_2a*(0.5-a));

    auto add_v = [](const Point& pt, const Vector& v) { return Point(pt[0]+v[0], pt[1]+v[1], pt[2]+v[2]); };
    auto sub_v = [](const Point& pt, const Vector& v) { return Point(pt[0]-v[0], pt[1]-v[1], pt[2]-v[2]); };

    // 5 base polylines (center + 2 top sides + 2 bot sides)
    std::vector<Polyline> base;
    base.push_back(Polyline({add_v(p[0],v0), sub_v(p[1],v0), sub_v(p[2],v0), add_v(p[3],v0), add_v(p[0],v0)}));
    base.push_back(Polyline({sub_v(p[1],v0), add_v(p[0],v0), add_v(p[8],v0), sub_v(p[9],v0), sub_v(p[1],v0)}));
    base.push_back(Polyline({add_v(p[3],v0), sub_v(p[2],v0), sub_v(p[10],v0), add_v(p[11],v0), add_v(p[3],v0)}));
    base.push_back(Polyline({p[2], p[1], p[13], p[14], p[2]}));
    base.push_back(Polyline({p[0], p[3], p[15], p[12], p[0]}));

    // Extend bot-side rectangles (polylines 3 and 4)
    base[3].extend_segment_equally(0, ext_side * ext_side_factor_a, ext_side * ext_side_factor_b);
    base[3].extend_segment_equally(2, ext_side * ext_side_factor_a, ext_side * ext_side_factor_b);
    base[4].extend_segment_equally(0, ext_side * ext_side_factor_a, ext_side * ext_side_factor_b);
    base[4].extend_segment_equally(2, ext_side * ext_side_factor_a, ext_side * ext_side_factor_b);
    base[3].extend_segment_equally(1, ext_vert, ext_vert);
    base[3].extend_segment_equally(3, ext_vert, ext_vert);
    base[4].extend_segment_equally(1, ext_vert, ext_vert);
    base[4].extend_segment_equally(3, ext_vert, ext_vert);

    // Append extra drill polylines
    for (const auto& dr : extra_drills) base.push_back(dr);
    int n = (int)base.size();

    double lengths[5] = { 0.5, 0.4, 0.4, 0.4, 0.4 };
    Xform xf_rot = Xform::from_axes(Vector(0,1,0), Vector(1,0,0), Vector(0,0,-1));

    std::vector<Polyline> f0 = base;
    std::vector<Polyline> f1(n);
    std::vector<Polyline> m0(n);
    std::vector<Polyline> m1(n);

    for (int i = 0; i < n; i++) {
        f1[i] = base[i];
        if (base[i].point_count() > offset_min_pts && i < 5) {
            Point pa = f1[i].get_point(0);
            Point pb = f1[i].get_point(1);
            Point pc = f1[i].get_point(2);
            Vector ab(pb[0]-pa[0], pb[1]-pa[1], pb[2]-pa[2]);
            Vector cb(pb[0]-pc[0], pb[1]-pc[1], pb[2]-pc[2]);
            Vector cross = ab.cross(cb);
            cross.normalize_self();
            for (size_t j = 0; j < f1[i].point_count(); j++) {
                Point pt = f1[i].get_point(j);
                f1[i].set_point(j, Point(pt[0]+cross[0]*lengths[i], pt[1]+cross[1]*lengths[i], pt[2]+cross[2]*lengths[i]));
            }
        }
        m0[i] = base[i].transformed_xform(xf_rot);
        m1[i] = f1[i].transformed_xform(xf_rot);
    }

    // Duplicate 2×
    joint.f_outlines[0].clear(); joint.f_outlines[1].clear();
    joint.m_outlines[0].clear(); joint.m_outlines[1].clear();
    for (int i = 0; i < n; i++) {
        joint.f_outlines[0].push_back(f0[i]); joint.f_outlines[0].push_back(f0[i]);
        joint.f_outlines[1].push_back(f1[i]); joint.f_outlines[1].push_back(f1[i]);
        joint.m_outlines[0].push_back(m0[i]); joint.m_outlines[0].push_back(m0[i]);
        joint.m_outlines[1].push_back(m1[i]); joint.m_outlines[1].push_back(m1[i]);
    }

    // Side-face reconstruction for polylines at duplication indices 2 and 4
    for (int i = 0; i < 2; i++) {
        int id = (i + 1) * 2;
        auto& fo0 = joint.f_outlines[0]; auto& fo1 = joint.f_outlines[1];
        auto& mo0 = joint.m_outlines[0]; auto& mo1 = joint.m_outlines[1];
        Polyline side00({fo0[id].get_point(0), fo0[id].get_point(1), fo1[id].get_point(1), fo1[id].get_point(0), fo0[id].get_point(0)});
        Polyline side01({fo0[id].get_point(3), fo0[id].get_point(2), fo1[id].get_point(2), fo1[id].get_point(3), fo0[id].get_point(3)});
        fo0[id] = side00; fo1[id] = side01;
        fo0[id+1] = side00; fo1[id+1] = side01;
        Polyline mside00({mo0[id].get_point(0), mo0[id].get_point(1), mo1[id].get_point(1), mo1[id].get_point(0), mo0[id].get_point(0)});
        Polyline mside01({mo0[id].get_point(3), mo0[id].get_point(2), mo1[id].get_point(2), mo1[id].get_point(3), mo0[id].get_point(3)});
        mo0[id] = mside00; mo1[id] = mside01;
        mo0[id+1] = mside00; mo1[id+1] = mside01;
    }

    for (int face = 0; face < 2; face++) {
        joint.f_cut_types[face] = cut_types;
        joint.m_cut_types[face] = cut_types;
    }
}

static void cr_c_ip_2(WoodJoint& joint) {
    // 5 base polylines, no drills. Cut: 2×mill_project + 4×slice_projectsheer + 4×mill_project = 10
    std::vector<int> ct = {
        wood_cut::mill_project, wood_cut::mill_project,
        wood_cut::slice_projectsheer, wood_cut::slice_projectsheer, wood_cut::slice_projectsheer, wood_cut::slice_projectsheer,
        wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project,
    };
    cr_c_ip_shared(joint, {}, 0.15, 0.6, 1.0, 1.0, 0, ct);
}

static void cr_c_ip_3(WoodJoint& joint) {
    // 7 base polylines = 5 base + 2 diagonal drills. Skip offset for 2-pt drills.
    // Cut: 2×mill_project + 4×slice_projectsheer + 4×mill_project + 4×drill = 14
    std::vector<Polyline> drills = {
        Polyline({Point(0.3, 0.041421, -0.928477), Point(0.041421, 0.3, 0.928477)}),
        Polyline({Point(-0.3, -0.041421, -0.928477), Point(-0.041421, -0.3, 0.928477)}),
    };
    std::vector<int> ct = {
        wood_cut::mill_project, wood_cut::mill_project,
        wood_cut::slice_projectsheer, wood_cut::slice_projectsheer, wood_cut::slice_projectsheer, wood_cut::slice_projectsheer,
        wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project,
        wood_cut::drill, wood_cut::drill, wood_cut::drill, wood_cut::drill,
    };
    cr_c_ip_shared(joint, drills, 0.15, 0.6, 1.0, 1.0, 2, ct);
}

static void cr_c_ip_4(WoodJoint& joint) {
    // 6 base polylines = 5 base + 1 vertical drill.
    // Cut: 2×mill_project + 4×slice_projectsheer + 4×mill_project + 2×drill = 12
    std::vector<Polyline> drills = {
        Polyline({Point(0.0, 0.0, -1.0), Point(0.0, 0.0, 1.0)}),
    };
    std::vector<int> ct = {
        wood_cut::mill_project, wood_cut::mill_project,
        wood_cut::slice_projectsheer, wood_cut::slice_projectsheer, wood_cut::slice_projectsheer, wood_cut::slice_projectsheer,
        wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project,
        wood_cut::drill, wood_cut::drill,
    };
    cr_c_ip_shared(joint, drills, 0.15, 0.6, 1.0, 1.0, 1, ct);
}

static void cr_c_ip_5(WoodJoint& joint) {
    // 7 base polylines = 5 base + 1 vertical drill + 1 horizontal drill.
    // Asymmetric extend: side factor a=1.8, b=-0.5 (wood lines 5357-5364).
    // Cut: 2×mill_project + 4×slice_projectsheer + 4×mill_project + 4×drill = 14
    std::vector<Polyline> drills = {
        Polyline({Point(0.0, 0.0, -1.0), Point(0.0, 0.0, 1.0)}),
        Polyline({Point(-0.5, 0.0, -0.55), Point(0.5, 0.0, -0.55)}),
    };
    std::vector<int> ct = {
        wood_cut::mill_project, wood_cut::mill_project,
        wood_cut::slice_projectsheer, wood_cut::slice_projectsheer, wood_cut::slice_projectsheer, wood_cut::slice_projectsheer,
        wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project,
        wood_cut::drill, wood_cut::drill, wood_cut::drill, wood_cut::drill,
    };
    cr_c_ip_shared(joint, drills, 0.15, 0.6, 1.8, -0.5, 2, ct);
}
