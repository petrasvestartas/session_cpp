static void cr_c_ip_3(WoodJoint& joint) {
    joint.name = "cr_c_ip_3";
    // 7 base polylines = 5 base + 2 diagonal drills. Skip offset for 2-pt drills.
    // Cut: 2×mill_project + 4×slice_projectsheer + 4×mill_project + 4×drill = 14
    std::vector<Polyline> drills = {
        Polyline({Point(0.3, 0.041421, -0.928477), Point(0.041421, 0.3, 0.928477)}),
        Polyline({Point(-0.3, -0.041421, -0.928477), Point(-0.041421, -0.3, 0.928477)}),
    };
    std::vector<int> ct = {
        wood_cut::mill_project, wood_cut::mill_project,
        wood_cut::slice_projectsheer, wood_cut::slice_projectsheer, wood_cut::slice_projectsheer, wood_cut::slice_projectsheer,
        wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project,
        wood_cut::drill, wood_cut::drill, wood_cut::drill, wood_cut::drill,
    };
    cr_c_ip_shared(joint, drills, 0.15, 0.6, 1.0, 1.0, 2, ct);
}
