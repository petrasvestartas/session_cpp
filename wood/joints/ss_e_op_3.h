// ─── ss_e_op_3 ──────────────────────────────────────────────────────────────
// Verbatim port of wood_joint_lib.cpp:1901-2003. Miter tenon-mortise with
// 4 female outlines (2 insert_between + 2 hole) and 2 male outlines
// (2 insert_between).
static void ss_e_op_3(WoodJoint& joint) {
    joint.name = "ss_e_op_3";
    joint.f_outlines[0] = {
        Polyline({Point(0.5,0.5,0.3), Point(0.5,-1.499975,0.3), Point(0.5,-1.499975,-0.3), Point(0.5,0.5,-0.3)}),
        Polyline({Point(0.5,0.5,0.3), Point(0.5,0.5,-0.3)}),
        Polyline({Point(0.5,-0.5,0.25), Point(0.5,0.5,0.25), Point(0.5,0.5,-0.25), Point(0.5,-0.5,-0.25), Point(0.5,-0.5,0.25)}),
        Polyline({Point(0.5,-0.5,0.25), Point(0.5,0.5,0.25), Point(0.5,0.5,-0.25), Point(0.5,-0.5,-0.25), Point(0.5,-0.5,0.25)}),
    };
    joint.f_outlines[1] = {
        Polyline({Point(-0.5,-0.499975,0.3), Point(-0.5,-1.49995,0.3), Point(-0.5,-1.49995,-0.3), Point(-0.5,-0.5,-0.3)}),
        Polyline({Point(-0.5,-0.499975,0.3), Point(-0.5,-0.5,-0.3)}),
        Polyline({Point(-0.5,-0.499975,0.25), Point(-0.5,0.500025,0.25), Point(-0.5,0.500025,-0.25), Point(-0.5,-0.499975,-0.25), Point(-0.5,-0.499975,0.25)}),
        Polyline({Point(-0.5,-0.499975,0.25), Point(-0.5,0.500025,0.25), Point(-0.5,0.500025,-0.25), Point(-0.5,-0.499975,-0.25), Point(-0.5,-0.499975,0.25)}),
    };
    joint.m_outlines[0] = {
        Polyline({Point(-0.5,-0.5,0.3), Point(0.5,-0.5,0.3), Point(0.5,-0.5,0.25), Point(-1,-0.5,0.25), Point(-1,-0.5,-0.25), Point(0.5,-0.5,-0.25), Point(0.5,-0.5,-0.3), Point(-0.5,-0.5,-0.3)}),
        Polyline({Point(-0.5,-0.5,0.3), Point(-0.5,-0.5,-0.3)}),
    };
    joint.m_outlines[1] = {
        Polyline({Point(0.5,0.5,0.3), Point(0.510075,0.5,0.3), Point(0.5,0.5,0.25), Point(-1,0.5,0.25), Point(-1,0.5,-0.25), Point(0.5,0.5,-0.25), Point(0.510075,0.5,-0.3), Point(0.5,0.5,-0.3)}),
        Polyline({Point(0.5,0.5,0.3), Point(0.5,0.5,-0.3)}),
    };
    for (int face = 0; face < 2; face++) {
        joint.f_cut_types[face] = { wood_cut::insert_between_multiple_edges, wood_cut::insert_between_multiple_edges, wood_cut::hole, wood_cut::hole };
        joint.m_cut_types[face] = { wood_cut::insert_between_multiple_edges, wood_cut::insert_between_multiple_edges };
    }
}
