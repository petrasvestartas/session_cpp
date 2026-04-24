static void cr_c_ip_5(WoodJoint& joint) {
    joint.name = "cr_c_ip_5";
    // 7 base polylines = 5 base + 1 vertical drill + 1 horizontal drill.
    // Asymmetric extend: side factor a=1.8, b=-0.5 (wood lines 5357-5364).
    // Cut: 2×mill_project + 4×slice_projectsheer + 4×mill_project + 4×drill = 14
    std::vector<Polyline> drills = {
        Polyline({Point(0.0, 0.0, -1.0), Point(0.0, 0.0, 1.0)}),
        Polyline({Point(-0.5, 0.0, -0.55), Point(0.5, 0.0, -0.55)}),
    };
    std::vector<int> ct = {
        wood_cut::mill_project, wood_cut::mill_project,
        wood_cut::slice_projectsheer, wood_cut::slice_projectsheer, wood_cut::slice_projectsheer, wood_cut::slice_projectsheer,
        wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project,
        wood_cut::drill, wood_cut::drill, wood_cut::drill, wood_cut::drill,
    };
    cr_c_ip_shared(joint, drills, 0.15, 0.6, 1.8, -0.5, 2, ct);
}
