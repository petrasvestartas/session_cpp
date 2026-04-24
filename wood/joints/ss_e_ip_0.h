// ─────────────────────────────────────────────────────────────────────────────
// group 0 — ss_e_ip (side-side in-plane, type 12)
// ─────────────────────────────────────────────────────────────────────────────

// ss_e_ip_0: side-to-side IN-PLANE HARDCODED 3-finger joint (type=12,
// joint name id=2). Verbatim port of wood_joint_lib.cpp:638-667.
//
// Same 12-point zigzag profile as ss_e_op_0 but oriented in the x/z plane
// (y=±0.5) instead of the y/z plane (x=±0.5). The "in-plane" naming refers
// to the joint axis being IN the plate's local plane (not perpendicular).
//
// All 4 outlines have identical geometry — m and f for the same face are
// the same outline, because for an in-plane edge joint both elements share
// the same boundary line and just slot into each other.
static void ss_e_ip_0(WoodJoint& joint) {
    joint.name = "ss_e_ip_0";
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
    // Joint lines, always the last line or
    // rectangle is not a wood::joint but an
    // cutting wood::element
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
