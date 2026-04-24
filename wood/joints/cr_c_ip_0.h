// ─── cr_c_ip_0 ──────────────────────────────────────────────────────────────
// Verbatim port of wood_joint_lib.cpp:4583-4602. Trivial cross-joint stub:
// each face has 2 identical closed rectangles.
static void cr_c_ip_0(WoodJoint& joint) {
    joint.name = "cr_c_ip_0";
    double s = 1.0;
    joint.f_outlines[0] = {
        Polyline({Point(-0.5,0.5,s), Point(-0.5,-0.5,s), Point(-0.5,-0.5,0), Point(-0.5,0.5,0), Point(-0.5,0.5,s)}),
        Polyline({Point(-0.5,0.5,s), Point(-0.5,-0.5,s), Point(-0.5,-0.5,0), Point(-0.5,0.5,0), Point(-0.5,0.5,s)}),
    };
    joint.f_outlines[1] = {
        Polyline({Point(0.5,0.5,s), Point(0.5,-0.5,s), Point(0.5,-0.5,0), Point(0.5,0.5,0), Point(0.5,0.5,s)}),
        Polyline({Point(0.5,0.5,s), Point(0.5,-0.5,s), Point(0.5,-0.5,0), Point(0.5,0.5,0), Point(0.5,0.5,s)}),
    };
    joint.m_outlines[0] = {
        Polyline({Point(0.5,0.5,-s), Point(-0.5,0.5,-s), Point(-0.5,0.5,0), Point(0.5,0.5,0), Point(0.5,0.5,-s)}),
        Polyline({Point(0.5,0.5,-s), Point(-0.5,0.5,-s), Point(-0.5,0.5,0), Point(0.5,0.5,0), Point(0.5,0.5,-s)}),
    };
    joint.m_outlines[1] = {
        Polyline({Point(0.5,-0.5,-s), Point(-0.5,-0.5,-s), Point(-0.5,-0.5,0), Point(0.5,-0.5,0), Point(0.5,-0.5,-s)}),
        Polyline({Point(0.5,-0.5,-s), Point(-0.5,-0.5,-s), Point(-0.5,-0.5,0), Point(0.5,-0.5,0), Point(0.5,-0.5,-s)}),
    };
    for (int face = 0; face < 2; face++) {
        joint.f_cut_types[face] = { wood_cut::insert_between_multiple_edges, wood_cut::insert_between_multiple_edges };
        joint.m_cut_types[face] = { wood_cut::insert_between_multiple_edges, wood_cut::insert_between_multiple_edges };
    }
}
