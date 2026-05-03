// ss_e_ip_5: side-to-side IN-PLANE reversed-tooth joint (type=12, id=6).
// Port of wood_joint_lib.cpp:1320-1544. Tiles `divisions` copies of a fixed
// 8-point tooth shape along the joint Z-axis, reversing each copy's winding
// before appending (so the concatenated polyline forms a non-self-intersecting
// outline). Male goes to -X, female goes to +X (mirror of ss_e_ip_1/2).
// Requires element thickness via elements[joint.v0].thickness.
static void ss_e_ip_5(WoodJoint& joint, const std::vector<WoodElement>& elements) {
    joint.name = "ss_e_ip_5";

    int v0 = joint.el_ids.first;
    if (v0 < 0 || v0 >= (int)elements.size()) return;
    joint.unit_scale_distance = elements[v0].thickness;

    double edge_length = 1000.0;
    {
        Point a = joint.joint_lines[0].start();
        Point b = joint.joint_lines[0].end();
        double d = Point::distance(a, b);
        if (d > 1e-9) edge_length = d;
    }
    int divisions = std::max(1, std::min(100, joint.divisions));
    double joint_volume_edge_length =
        (joint.unit_scale_distance > 0.0) ? joint.unit_scale_distance : 40.0;

    edge_length *= joint.scale[2];
    double move_length_scaled = edge_length / (divisions * joint_volume_edge_length);
    double total_length_scaled = edge_length / joint_volume_edge_length;

    // dir = (0,0,1); move_from_center_to_end = dir * (total*0.5 - move*0.5)
    double dz_start = (total_length_scaled * 0.5) - (move_length_scaled * 0.5);
    double dz_step  = -move_length_scaled; // move_length_dir along -Z

    // Hardcoded unit-cube tooth shapes (identical to upstream wood_joint_lib.cpp:1378-1427)
    // male_0: face y=-0.5, tooth in -X direction
    static const double m0_pts[8][3] = {
        { 0,        -0.5, -0.166667},
        {-0.116667, -0.5, -0.166667},
        {-0.619628, -0.5, -0.375   },
        {-1.0,      -0.5, -0.375   },
        {-1.0,      -0.5,  0.375   },
        {-0.619628, -0.5,  0.375   },
        {-0.116667, -0.5,  0.166667},
        { 0,        -0.5,  0.166667},
    };
    // male_1: face y=+0.5 (same x,z)
    // female_0: face y=-0.5, tooth in +X direction
    static const double f0_pts[8][3] = {
        { 0,        -0.5, -0.166667},
        { 0.116667, -0.5, -0.166667},
        { 0.619628, -0.5, -0.375   },
        { 1.0,      -0.5, -0.375   },
        { 1.0,      -0.5,  0.375   },
        { 0.619628, -0.5,  0.375   },
        { 0.116667, -0.5,  0.166667},
        { 0,        -0.5,  0.166667},
    };

    std::vector<Point> m0, m1, f0, f1;
    m0.reserve(8 * divisions);
    m1.reserve(8 * divisions);
    f0.reserve(8 * divisions);
    f1.reserve(8 * divisions);

    for (int i = 0; i < divisions; ++i) {
        double dz = dz_start + dz_step * i;
        // Reversed copies (std::reverse before append)
        for (int k = 7; k >= 0; --k) {
            m0.emplace_back(m0_pts[k][0],          m0_pts[k][1],   m0_pts[k][2] + dz);
            m1.emplace_back(m0_pts[k][0],           0.5,            m0_pts[k][2] + dz);
            f0.emplace_back(f0_pts[k][0],           m0_pts[k][1],   f0_pts[k][2] + dz);
            f1.emplace_back(f0_pts[k][0],           0.5,            f0_pts[k][2] + dz);
        }
    }

    Polyline pl_m0(m0), pl_m1(m1), pl_f0(f0), pl_f1(f1);
    Polyline ep_m0(std::vector<Point>{ m0.front(), m0.back() });
    Polyline ep_m1(std::vector<Point>{ m1.front(), m1.back() });
    Polyline ep_f0(std::vector<Point>{ f0.front(), f0.back() });
    Polyline ep_f1(std::vector<Point>{ f1.front(), f1.back() });

    joint.m_outlines[0] = { pl_m0, ep_m0 };
    joint.m_outlines[1] = { pl_m1, ep_m1 };
    joint.f_outlines[0] = { pl_f0, ep_f0 };
    joint.f_outlines[1] = { pl_f1, ep_f1 };
    joint.m_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.f_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.f_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.unit_scale = true;
}
