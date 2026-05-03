// ss_e_r_custom: user-supplied unit-cube joint geometry for ss_e_r (type 13).
// Port of wood_joint_lib.cpp:3089-3113 — reads pairs (i, i+1) from
// CUSTOM_JOINTS_SS_E_R_MALE/FEMALE. Each pair = (face0, face1) polyline.
//
// Uses the same concatenation fix as ss_e_ip_custom/ss_e_op_custom:
// upstream double-push creates a 4-pt marker which wood_merge.cpp:145-146
// skips. Concatenate all per-pair polylines into one outline + 2-pt endpoint.
static void ss_e_r_custom(WoodJoint& joint) {
    joint.name = "ss_e_r_custom";

    const auto& cm = wood_session::globals::CUSTOM_JOINTS_SS_E_R_MALE;
    const auto& cf = wood_session::globals::CUSTOM_JOINTS_SS_E_R_FEMALE;
    if (cm.size() < 2 || cf.size() < 2) return;

    std::vector<Point> m0, m1, f0, f1;
    for (size_t i = 0; i + 1 < cm.size(); i += 2) {
        for (size_t k = 0; k < cm[i].point_count();   k++) m0.push_back(cm[i].get_point(k));
        for (size_t k = 0; k < cm[i+1].point_count(); k++) m1.push_back(cm[i+1].get_point(k));
    }
    for (size_t i = 0; i + 1 < cf.size(); i += 2) {
        for (size_t k = 0; k < cf[i].point_count();   k++) f0.push_back(cf[i].get_point(k));
        for (size_t k = 0; k < cf[i+1].point_count(); k++) f1.push_back(cf[i+1].get_point(k));
    }
    if (m0.empty() || m1.empty() || f0.empty() || f1.empty()) return;

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
}
