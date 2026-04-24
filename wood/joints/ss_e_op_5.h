// ─── ss_e_op_5 ──────────────────────────────────────────────────────────────
// Verbatim port of wood_joint_lib.cpp:2194-2270. Joint linking constructor.
// If no linked_joints → falls back to ss_e_op_4 with chamfer.
// Otherwise generates geometry for self AND linked shadow joints.
static void ss_e_op_5(WoodJoint& jo, std::vector<WoodJoint>& all_joints, bool disable_joint_divisions) {
    jo.name = "ss_e_op_5";
    if (jo.linked_joints.empty() || jo.linked_joints.size() > 2) {
        ss_e_op_4(jo, 0.00, true, true, -0.75, 0.5, -0.5, 0.5, -0.5, 0.5);
        return;
    }

    ss_e_op_4(jo, 0.00, false, true, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5);

    // Create geometry for linked joint 0
    int a = 0;
    int b = 1;
    all_joints[jo.linked_joints[a]].divisions = jo.divisions;
    ss_e_op_4(all_joints[jo.linked_joints[a]], 0.5, true, false, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5);

    // Set linking sequence for joint 0
    std::vector<std::array<int, 4>> linked_joints_seq_0;
    linked_joints_seq_0.push_back({2, 4, 2, 8});
    jo.linked_joints_seq.push_back(linked_joints_seq_0);

    if (jo.linked_joints.size() == 2) {
        // Create geometry for linked joint 1
        all_joints[jo.linked_joints[b]].divisions = disable_joint_divisions ? 0 : jo.divisions;
        ss_e_op_4(all_joints[jo.linked_joints[b]], 0.00, true, false, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5);

        std::vector<std::array<int, 4>> linked_joints_seq_1;
        for (size_t i = 0; i < jo.f_outlines[0].size(); i += 2) {
            if (i == 0)
                linked_joints_seq_1.push_back({1, (int)jo.f_outlines[0][0].point_count() - 2,
                                                1, (int)all_joints[jo.linked_joints[1]].m_outlines[0][0].point_count() - 2});
            else
                linked_joints_seq_1.push_back({0, 0, 0, 0});
        }
        jo.linked_joints_seq.push_back(linked_joints_seq_1);
    }
}
