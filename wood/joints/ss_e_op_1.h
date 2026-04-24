// ss_e_op_1: side-to-side out-of-plane PARAMETRIC finger joint (type=11,
// joint name id=10). Verbatim port of wood_joint_lib.cpp:1609-1751.
// Creates parametric geometry in the [-0.5, 0.5]³ unit cube. Default annen
// dispatch target.
static void ss_e_op_1(WoodJoint& joint) {
    joint.name = "ss_e_op_1";

    ////////////////////////////////////////////////////////////////////
    // Number of divisions
    // Input wood::joint line (its lengths)
    // Input distance for division
    ////////////////////////////////////////////////////////////////////
    int div = std::max(2, std::min(20, joint.divisions));
    div += div % 2; // force even

    ////////////////////////////////////////////////////////////////////
    // Interpolate points
    ////////////////////////////////////////////////////////////////////
    auto arr0 = Point::interpolate(Point(-0.5, 0.5,-0.5), Point(-0.5, 0.5, 0.5), div, 0);
    auto arr1 = Point::interpolate(Point( 0.5, 0.5,-0.5), Point( 0.5, 0.5, 0.5), div, 0);
    auto arr2 = Point::interpolate(Point( 0.5,-0.5,-0.5), Point( 0.5,-0.5, 0.5), div, 0);
    auto arr3 = Point::interpolate(Point(-0.5,-0.5,-0.5), Point(-0.5,-0.5, 0.5), div, 0);
    std::vector<Point>* arrays[4] = {&arr0, &arr1, &arr2, &arr3};

    ////////////////////////////////////////////////////////////////////
    // Move segments
    ////////////////////////////////////////////////////////////////////
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
