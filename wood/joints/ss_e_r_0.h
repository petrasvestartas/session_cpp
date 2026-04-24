// ─── ss_e_r_0 ──────────────────────────────────────────────────────────────
// Verbatim port of wood_joint_lib.cpp:2310-2379. World-space geometry built
// from joint volumes (no unit cube — orient must be skipped). Splits each
// joint volume in half along the thickness axis; the two halves form female
// and male rectangles, offset in 4 positions along the joint line.
// Cut types: all slice. joint.no_orient = true.
static void ss_e_r_0(WoodJoint& joint) {
    joint.name = "ss_e_r_0";
    if (!joint.joint_volumes_pair_a_pair_b[0] || !joint.joint_volumes_pair_a_pair_b[1])
        return;
    const Polyline& vol0 = *joint.joint_volumes_pair_a_pair_b[0];
    const Polyline& vol1 = *joint.joint_volumes_pair_a_pair_b[1];
    if (vol0.point_count() < 4 || vol1.point_count() < 4)
        return;

    // tween_two_polylines(vol0, vol1, 0.5) — midpoint of each vertex pair
    Point r0 = Point::lerp(vol0.get_point(0), vol1.get_point(0), 0.5);
    Point r1 = Point::lerp(vol0.get_point(1), vol1.get_point(1), 0.5);
    Point r2 = Point::lerp(vol0.get_point(2), vol1.get_point(2), 0.5);
    Point r3 = Point::lerp(vol0.get_point(3), vol1.get_point(3), 0.5);

    Point p_mid_01 = Point::mid_point(r0, r1);
    Point p_mid_32 = Point::mid_point(r3, r2);
    // z_scaled = (p_mid_01 - r1) * 0.75
    double zsx = (p_mid_01[0]-r1[0])*0.75;
    double zsy = (p_mid_01[1]-r1[1])*0.75;
    double zsz = (p_mid_01[2]-r1[2])*0.75;
    // x = (vol1[0] - vol0[0]) * 0.5
    double xx = (vol1.get_point(0)[0]-vol0.get_point(0)[0])*0.5;
    double xy = (vol1.get_point(0)[1]-vol0.get_point(0)[1])*0.5;
    double xz = (vol1.get_point(0)[2]-vol0.get_point(0)[2])*0.5;
    double x_len = std::sqrt(xx*xx + xy*xy + xz*xz);
    if (x_len < 1e-12) return;

    // rect_half_0: "male" half (points below midline)
    Polyline rh0(std::vector<Point>{
        p_mid_01,
        Point(r0[0]-zsx, r0[1]-zsy, r0[2]-zsz),
        Point(r3[0]-zsx, r3[1]-zsy, r3[2]-zsz),
        p_mid_32,
        p_mid_01,
    });
    // rect_half_1: "female" half (points above midline)
    Polyline rh1(std::vector<Point>{
        p_mid_01,
        Point(r1[0]+zsx, r1[1]+zsy, r1[2]+zsz),
        Point(r2[0]+zsx, r2[1]+zsy, r2[2]+zsz),
        p_mid_32,
        p_mid_01,
    });
    double y_ext = joint.scale[1];
    rh0.extend_edge_equally(1, y_ext);
    rh0.extend_edge_equally(3, y_ext);
    rh1.extend_edge_equally(1, y_ext);
    rh1.extend_edge_equally(3, y_ext);

    double x_target = joint.scale[0];
    double factor_far  = (5.0 + x_len + x_target) / x_len;
    double factor_near = 0.25 / x_len;

    // offset_vectors[j] = {x*near, x*far, -x*near, -x*far}
    auto off = [&](double f) -> std::array<double,3> {
        return {xx*f, xy*f, xz*f};
    };
    std::array<std::array<double,3>, 4> offsets = {
        off( factor_near), off( factor_far),
        off(-factor_near), off(-factor_far),
    };

    // Translate copies of rh0/rh1 by each offset
    auto translate_poly = [](const Polyline& pl, const std::array<double,3>& d) {
        return pl.translated(Vector(d[0], d[1], d[2]));
    };

    // m[0]={off0,off0,off2,off2}, m[1]={off1,off1,off3,off3}
    // f[0]={off0,off0,off2,off2}, f[1]={off1,off1,off3,off3}
    for (int fi = 0; fi < 2; fi++) {
        joint.m_outlines[fi].clear();
        joint.m_outlines[fi].reserve(4);
        joint.f_outlines[fi].clear();
        joint.f_outlines[fi].reserve(4);
    }
    for (int oi : {0, 0, 2, 2}) joint.m_outlines[0].push_back(translate_poly(rh0, offsets[oi]));
    for (int oi : {1, 1, 3, 3}) joint.m_outlines[1].push_back(translate_poly(rh0, offsets[oi]));
    for (int oi : {0, 0, 2, 2}) joint.f_outlines[0].push_back(translate_poly(rh1, offsets[oi]));
    for (int oi : {1, 1, 3, 3}) joint.f_outlines[1].push_back(translate_poly(rh1, offsets[oi]));
    joint.m_cut_types[0] = std::vector<int>(4, wood_cut::slice);
    joint.m_cut_types[1] = std::vector<int>(4, wood_cut::slice);
    joint.f_cut_types[0] = std::vector<int>(4, wood_cut::slice);
    joint.f_cut_types[1] = std::vector<int>(4, wood_cut::slice);
    joint.no_orient = true; // geometry is already world-space
}
