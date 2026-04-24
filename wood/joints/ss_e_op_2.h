// ss_e_op_2: side-to-side out-of-plane parametric (type=11, joint name id=11).
// Verbatim port of wood_joint_lib.cpp:1754-1898. Same overall structure as
// ss_e_op_1 — interpolate 4 vertical edges of the unit cube, then build
// `m[0]/m[1]` and `f[0]/f[1]` zigzag outlines from pairs of arrays — but
// the "move segments" loop uses a different non-uniform shift pattern that
// scales the central two pairs more than the outer pairs (4*v vs 2*v) and
// flips sign across the polyline midpoint.
static void ss_e_op_2(WoodJoint& joint) {
    joint.name = "ss_e_op_2";

    ////////////////////////////////////////////////////////////////////
    // Number of divisions
    // Input wood::joint line (its lengths)
    // Input distance for division
    ////////////////////////////////////////////////////////////////////
    int div = std::max(4, std::min(20, joint.divisions));
    div += div % 2; // force even

    ////////////////////////////////////////////////////////////////////
    // Interpolate points
    ////////////////////////////////////////////////////////////////////
    auto arr0 = Point::interpolate(Point( 0.5,-0.5,-0.5), Point( 0.5,-0.5, 0.5), div, 0);
    auto arr1 = Point::interpolate(Point(-0.5,-0.5,-0.5), Point(-0.5,-0.5, 0.5), div, 0);
    auto arr2 = Point::interpolate(Point(-0.5, 0.5,-0.5), Point(-0.5, 0.5, 0.5), div, 0);
    auto arr3 = Point::interpolate(Point( 0.5, 0.5,-0.5), Point( 0.5, 0.5, 0.5), div, 0);
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
