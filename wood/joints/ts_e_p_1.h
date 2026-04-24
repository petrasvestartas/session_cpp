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
[[maybe_unused]] static void ts_e_p_1(WoodJoint& joint) {
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
    // Wood: f_boolean_type = {hole, hole, insert_between_multiple_edges}×2
    //       m_boolean_type = {edge_insertion, edge_insertion}
    // (wood_joint_lib.cpp:3596-3597). f_outlines[face] has 3 entries
    // (2 mortise holes + 1 bounding rectangle).
    joint.f_cut_types[0] = { wood_cut::hole, wood_cut::hole, wood_cut::insert_between_multiple_edges };
    joint.f_cut_types[1] = { wood_cut::hole, wood_cut::hole, wood_cut::insert_between_multiple_edges };
    joint.m_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
}
