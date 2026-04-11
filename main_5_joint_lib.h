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

// Linear remap from one numeric range to another. Mirrors wood's
// `internal::remap_numbers` (used by ss_e_op_2 / ts_e_p_2 / ss_e_op_1 /
// ts_e_p_3 to translate `joint.shift` from [0,1] into the unit cube's
// [-0.5,+0.5] range).
static double remap_numbers(double value, double in_min, double in_max,
                             double out_min, double out_max) {
    if (in_max == in_min) return out_min;
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);
}

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
    double shift_ = remap_numbers(joint.shift, 0, 1.0, -0.5, 0.5);
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
        : remap_numbers(joint.shift, 0, 1.0, -0.5, 0.5) / (div + 1);
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
        : remap_numbers(joint.shift, 0, 1.0, -0.5, 0.5) / (div + 1);
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
}
