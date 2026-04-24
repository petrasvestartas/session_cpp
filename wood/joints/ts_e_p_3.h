// ts_e_p_3: top-to-side PARAMETRIC tenon-mortise (type=20, joint name id=20
// and 22; default for annen). Verbatim port of wood_joint_lib.cpp:3713-3935.
// Male: zigzag polyline with every-other-pair skip + 2-point endpoints.
// Female: rectangular holes from every 4 male points + bounding rectangle.
static void ts_e_p_3(WoodJoint& joint) {
    joint.name = "ts_e_p_3";

    ////////////////////////////////////////////////////////////////////
    // Number of divisions
    // Input wood::joint line (its lengths)
    // Input distance for division
    ////////////////////////////////////////////////////////////////////
    int div = std::max(8, std::min(100, joint.divisions));
    div -= div % 4; // force multiple of 4
    if (div == 0) return;
    int size = div / 4 + 1;

    ////////////////////////////////////////////////////////////////////
    // Interpolate points
    ////////////////////////////////////////////////////////////////////
    // Interpolate 4 edge lines.
    auto arr3 = Point::interpolate(Point( 0.5,-0.5,-0.5), Point( 0.5,-0.5, 0.5), div, 0);
    auto arr0 = Point::interpolate(Point(-0.5,-0.5,-0.5), Point(-0.5,-0.5, 0.5), div, 0);
    auto arr1 = Point::interpolate(Point(-0.5, 0.5,-0.5), Point(-0.5, 0.5, 0.5), div, 0);
    auto arr2 = Point::interpolate(Point( 0.5, 0.5,-0.5), Point( 0.5, 0.5, 0.5), div, 0);
    std::vector<Point>* arrays[4] = {&arr0, &arr1, &arr2, &arr3};

    ////////////////////////////////////////////////////////////////////
    // Move segments
    ////////////////////////////////////////////////////////////////////
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
    // merge in wood_main.cpp can use a single uniform iteration rule.
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
