// ─────────────────────────────────────────────────────────────────────────────
// group 1 — ss_e_op (side-side out-of-plane, type 11)
// ─────────────────────────────────────────────────────────────────────────────

// ss_e_op_0: side-to-side out-of-plane HARDCODED 3-finger joint (type=11,
// joint name id=12 in the dispatcher). Verbatim port of
// wood_joint_lib.cpp:1577-1606.
static void ss_e_op_0(WoodJoint& joint) {
    joint.name = "ss_e_op_0";
    auto P = [](double x, double y, double z) { return Point(x, y, z); };
    // Joint lines, always the last line or
    // rectangle is not a wood::joint but an
    // cutting wood::element
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
