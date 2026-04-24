// ss_e_ip_2: butterfly / X-fix joint. Verbatim port of wood_joint_lib.cpp:757-967.
// Unit-cube geometry — N copies of a 4-pt butterfly tooth merged into a single
// polyline along the joint axis, plus a 2-pt endpoint marker per face.
// Cut types: edge_insertion. unit_scale = true.
static void ss_e_ip_2(WoodJoint& joint) {
    joint.name = "ss_e_ip_2";
    // Safe defaults when joint_lines missing (e.g. in library-only tests).
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
    // Axis unit vector = +z; move_from_center_to_end brings the first tooth's
    // centre out to `+Z * (total-move)/2`; each step steps BACK by move_length.
    double mv_end  = (total_length_scaled * 0.5) - (move_length_scaled * 0.5);
    double mv_step = -move_length_scaled;

    // Base butterfly teeth (4 pts each, + 2-pt markers).
    auto P = [](double x, double y, double z) { return Point(x, y, z); };
    std::vector<Point> base_m0 = {
        P(0.0, -0.5, 0.1166666667),
        P(-0.5, -0.5, 0.4),
        P(-0.5, -0.5, -0.4),
        P(0.0, -0.5, -0.1166666667),
    };
    std::vector<Point> base_m1 = {
        P(0.0, 0.5, 0.1166666667),
        P(-0.5, 0.5, 0.4),
        P(-0.5, 0.5, -0.4),
        P(0.0, 0.5, -0.1166666667),
    };
    std::vector<Point> base_f0 = {
        P(0.0, -0.5, 0.1166666667),
        P(0.5, -0.5, 0.4),
        P(0.5, -0.5, -0.4),
        P(0.0, -0.5, -0.1166666667),
    };
    std::vector<Point> base_f1 = {
        P(0.0, 0.5, 0.1166666667),
        P(0.5, 0.5, 0.4),
        P(0.5, 0.5, -0.4),
        P(0.0, 0.5, -0.1166666667),
    };

    auto build = [&](const std::vector<Point>& base) -> std::vector<Point> {
        std::vector<Point> out;
        out.reserve(base.size() * divisions);
        for (int i = 0; i < divisions; ++i) {
            double dz = mv_end + mv_step * i;
            for (const Point& p : base)
                out.emplace_back(p[0], p[1], p[2] + dz);
        }
        return out;
    };
    std::vector<Point> m0 = build(base_m0);
    std::vector<Point> m1 = build(base_m1);
    std::vector<Point> f0 = build(base_f0);
    std::vector<Point> f1 = build(base_f1);

    Polyline pl_m0(m0);
    Polyline pl_m1(m1);
    Polyline pl_f0(f0);
    Polyline pl_f1(f1);
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
