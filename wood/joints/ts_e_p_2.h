// ts_e_p_2: top-to-side parametric tenon-mortise (type=20, joint name id=22).
// Verbatim port of wood_joint_lib.cpp:3604-3710. Structurally identical to
// ts_e_p_3 (already ported) except the male zigzag loop does NOT skip every
// other pair — it visits every interpolation point — and the female hole
// rectangles are built from every 4 male points (no skip). Both `m[0]/m[1]`
// have 2-pt endpoint markers; `f[0]/f[1]` are vectors of 5-point hole
// rectangles plus a bounding-rect at the end.
static void ts_e_p_2(WoodJoint& joint) {
    joint.name = "ts_e_p_2";

    ////////////////////////////////////////////////////////////////////
    // Number of divisions
    // Input wood::joint line (its lengths)
    // Input distance for division
    ////////////////////////////////////////////////////////////////////
    int div = std::max(2, std::min(20, joint.divisions));
    div += div % 2; // force even
    int size = div / 2 + 1;

    ////////////////////////////////////////////////////////////////////
    // Interpolate points
    ////////////////////////////////////////////////////////////////////
    auto arr0 = Point::interpolate(Point(-0.5,-0.5,-0.5), Point(-0.5,-0.5, 0.5), div, 0);
    auto arr1 = Point::interpolate(Point(-0.5, 0.5,-0.5), Point(-0.5, 0.5, 0.5), div, 0);
    auto arr2 = Point::interpolate(Point( 0.5, 0.5,-0.5), Point( 0.5, 0.5, 0.5), div, 0);
    auto arr3 = Point::interpolate(Point( 0.5,-0.5,-0.5), Point( 0.5,-0.5, 0.5), div, 0);
    std::vector<Point>* arrays[4] = {&arr0, &arr1, &arr2, &arr3};

    ////////////////////////////////////////////////////////////////////
    // Move segments
    ////////////////////////////////////////////////////////////////////
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
