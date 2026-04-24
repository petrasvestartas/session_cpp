// ─── ss_e_ip_3 ──────────────────────────────────────────────────────────────
// Verbatim port of wood_joint_lib.cpp:970-1116. Hardcoded mill+drill in-plane
// joint with 6 outlines per face (2 mill_project + 4 drill).
static void ss_e_ip_3(WoodJoint& joint) {
    joint.name = "ss_e_ip_3";
    joint.f_outlines[0] = {
        Polyline({Point(-1.25,-0.5,-0.5), Point(1,-0.5,-0.5), Point(1,-0.2,-0.5), Point(-1,0.2,-0.5), Point(-1,0.5,-0.5), Point(-1.25,0.5,-0.5), Point(-1.25,-0.5,-0.5)}),
        Polyline({Point(0,-0.5,0.5), Point(0,-0.5,-0.5)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
    };
    joint.f_outlines[1] = {
        Polyline({Point(-1.25,-0.5,0.5), Point(1,-0.5,0.5), Point(1,-0.2,0.5), Point(-1,0.2,0.5), Point(-1,0.5,0.5), Point(-1.25,0.5,0.5), Point(-1.25,-0.5,0.5)}),
        Polyline({Point(0,0.5,0.5), Point(0,0.5,-0.5)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
    };
    joint.m_outlines[0] = {
        Polyline({Point(1.25,0.5,-0.5), Point(-1,0.5,-0.5), Point(-1,0.2,-0.5), Point(1,-0.2,-0.5), Point(1,-0.5,-0.5), Point(1.25,-0.5,-0.5), Point(1.25,0.5,-0.5)}),
        Polyline({Point(0,-0.5,0.5), Point(0,-0.5,-0.5)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
    };
    joint.m_outlines[1] = {
        Polyline({Point(1.25,0.5,0.5), Point(-1,0.5,0.5), Point(-1,0.2,0.5), Point(1,-0.2,0.5), Point(1,-0.5,0.5), Point(1.25,-0.5,0.5), Point(1.25,0.5,0.5)}),
        Polyline({Point(0,0.5,0.5), Point(0,0.5,-0.5)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(-0.333333,-0.6,0), Point(-0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
        Polyline({Point(0.333333,-0.6,0), Point(0.333333,0.6,0)}),
    };
    for (int face = 0; face < 2; face++) {
        joint.f_cut_types[face] = { wood_cut::mill_project, wood_cut::mill_project, wood_cut::drill, wood_cut::drill, wood_cut::drill, wood_cut::drill };
        joint.m_cut_types[face] = { wood_cut::mill_project, wood_cut::mill_project, wood_cut::drill, wood_cut::drill, wood_cut::drill, wood_cut::drill };
    }
}
