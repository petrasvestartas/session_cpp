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
    joint.name = "ts_e_p_0";
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
