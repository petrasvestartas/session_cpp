static void cr_c_ip_4(WoodJoint& joint) {
    joint.name = "cr_c_ip_4";
    // 6 base polylines = 5 base + 1 vertical drill.
    // Cut: 2×mill_project + 4×slice_projectsheer + 4×mill_project + 2×drill = 12
    std::vector<Polyline> drills = {
        Polyline({Point(0.0, 0.0, -1.0), Point(0.0, 0.0, 1.0)}),
    };
    std::vector<int> ct = {
        wood_cut::mill_project, wood_cut::mill_project,
        wood_cut::slice_projectsheer, wood_cut::slice_projectsheer, wood_cut::slice_projectsheer, wood_cut::slice_projectsheer,
        wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project,
        wood_cut::drill, wood_cut::drill,
    };
    cr_c_ip_shared(joint, drills, 0.15, 0.6, 1.0, 1.0, 1, ct);
}
