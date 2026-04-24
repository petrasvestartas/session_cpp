static void cr_c_ip_2(WoodJoint& joint) {
    joint.name = "cr_c_ip_2";
    // 5 base polylines, no drills. Cut: 2×mill_project + 4×slice_projectsheer + 4×mill_project = 10
    std::vector<int> ct = {
        wood_cut::mill_project, wood_cut::mill_project,
        wood_cut::slice_projectsheer, wood_cut::slice_projectsheer, wood_cut::slice_projectsheer, wood_cut::slice_projectsheer,
        wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project, wood_cut::mill_project,
    };
    cr_c_ip_shared(joint, {}, 0.15, 0.6, 1.0, 1.0, 0, ct);
}
