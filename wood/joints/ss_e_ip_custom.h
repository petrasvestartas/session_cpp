// ss_e_ip_custom: user-supplied unit-cube joint geometry, parametrically tiled.
// Concept ported from wood_joint_lib.cpp:1560-1584 — reads pairs (i, i+1)
// from CUSTOM_JOINTS_SS_E_IP_MALE/FEMALE where each pair = (face0, face1)
// for one base tooth. Diverges from upstream in three ways:
//
//   1. Upstream pushes each polyline TWICE into the same face slot, which
//      produces a 4-point "endpoint marker". Session merge requires the
//      marker at jm[0][1] to be 2 or 5 points (wood_merge.cpp:145-146);
//      4 points → merge skips the joint entirely. Upstream's own comment
//      at wood_joint_lib.cpp:6330 admits the upstream pipeline "wont work"
//      here. Session concatenates all per-tooth polylines into one cut
//      polyline + a real 2-point marker, matching ss_e_ip_2's structure.
//
//   2. Cut type = edge_insertion (upstream uses cut::nothing, which is the
//      reason the upstream pipeline can't carve the plate).
//
//   3. Parametric tiling: each user-supplied (face0, face1) pair is treated
//      as ONE base tooth and tiled `joint.divisions` times along the joint
//      axis (unit Z), exactly like ss_e_ip_2 does for its hardcoded
//      butterfly. divisions comes from joints_parameters_and_types row 0
//      col 0 (division_length) computed earlier in wood_main.cpp:1050.
//      unit_scale = true so apply_unit_scale resizes the volume to plate
//      thickness before change_of_basis, matching ss_e_ip_2's geometry
//      exactly when CUSTOM_JOINTS_* hold the butterfly defaults.
//
// To get the canonical butterfly via this path: provide ONE pair with the
// hardcoded ss_e_ip_2 tooth coords. To get a custom tooth shape: change
// the polyline points. To get multiple distinct tooth shapes alternating:
// provide multiple pairs (each pair tiles `divisions` times independently).
static void ss_e_ip_custom(WoodJoint& joint) {
    joint.name = "ss_e_ip_custom";

    const auto& cm = wood_session::globals::CUSTOM_JOINTS_SS_E_IP_MALE;
    const auto& cf = wood_session::globals::CUSTOM_JOINTS_SS_E_IP_FEMALE;
    if (cm.size() < 2 || cf.size() < 2) return;

    // Tiling formula — verbatim from ss_e_ip_2.h:7-24.
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
    double mv_end  = (total_length_scaled * 0.5) - (move_length_scaled * 0.5);
    double mv_step = -move_length_scaled;

    // Tile each user pair `divisions` times along Z and concatenate into
    // one polyline per face. Multiple pairs = alternating tooth shapes.
    auto tile_face = [&](const std::vector<session_cpp::Polyline>& src,
                         bool pick_face0,
                         std::vector<session_cpp::Point>& out) {
        size_t n_pairs = src.size() / 2;
        out.reserve(divisions * 8);
        for (int i = 0; i < divisions; ++i) {
            double dz = mv_end + mv_step * i;
            const auto& base_pl = pick_face0 ? src[2 * (i % n_pairs)]
                                              : src[2 * (i % n_pairs) + 1];
            for (size_t k = 0; k < base_pl.point_count(); k++) {
                Point p = base_pl.get_point(k);
                out.emplace_back(p[0], p[1], p[2] + dz);
            }
        }
    };

    std::vector<session_cpp::Point> m0;
    std::vector<session_cpp::Point> m1;
    std::vector<session_cpp::Point> f0;
    std::vector<session_cpp::Point> f1;
    tile_face(cm, true,  m0);
    tile_face(cm, false, m1);
    tile_face(cf, true,  f0);
    tile_face(cf, false, f1);

    session_cpp::Polyline pl_m0(m0);
    session_cpp::Polyline pl_m1(m1);
    session_cpp::Polyline pl_f0(f0);
    session_cpp::Polyline pl_f1(f1);
    session_cpp::Polyline ep_m0(std::vector<session_cpp::Point>{ m0.front(), m0.back() });
    session_cpp::Polyline ep_m1(std::vector<session_cpp::Point>{ m1.front(), m1.back() });
    session_cpp::Polyline ep_f0(std::vector<session_cpp::Point>{ f0.front(), f0.back() });
    session_cpp::Polyline ep_f1(std::vector<session_cpp::Point>{ f1.front(), f1.back() });

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
