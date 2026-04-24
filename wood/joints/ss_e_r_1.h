// ─── ss_e_r_1 (default case — miter tenon-mortise) ─────────────────────────
// Verbatim port of wood_joint_lib.cpp:2507-2720 default case. 39-pt circular
// arc profile in YZ-plane, replicated at x=0 and x=0.5 for the two faces.
// Cut types: f=conic, m=conic_reverse. unit_scale=true.
static void ss_e_r_1(WoodJoint& joint) {
    joint.name = "ss_e_r_1";
    // The 39 outline points share identical (y, z) for all four face outlines.
    // X is 0.5 for face index 0 of f/m[1], 0.0 for the others (matching wood).
    static const double yz[][2] = {
        {-0.825,  0.0         },
        {-0.825, -0.151041813 },
        {-0.825, -0.302083626 },
        {-0.825, -0.39066965  },
        {-0.764910275, -0.37364172  },
        {-0.619590501, -0.332461718 },
        {-0.474270727, -0.291281717 },
        {-0.328950953, -0.250101715 },
        {-0.183631179, -0.208921714 },
        {-0.038311405, -0.167741712 },
        { 0.078145959, -0.134740598 },
        { 0.097349939, -0.129031066 },
        { 0.106158874, -0.124374202 },
        { 0.11914747,  -0.117507751 },
        { 0.138265279, -0.101937742 },
        { 0.153962352, -0.082924124 },
        { 0.165630347, -0.061203771 },
        { 0.172817069, -0.037618462 },
        { 0.175,       -0.01554903  },
        { 0.175,       -3.4e-08     },
        { 0.175,        0.01554903  },
        { 0.172817069,  0.037618462 },
        { 0.165630347,  0.061203771 },
        { 0.153962352,  0.082924124 },
        { 0.138265279,  0.101937742 },
        { 0.11914747,   0.117507751 },
        { 0.106158839,  0.12437399  },
        { 0.09734984,   0.129030732 },
        { 0.078145959,  0.134740598 },
        {-0.038311405,  0.167741712 },
        {-0.183631179,  0.208921714 },
        {-0.328950953,  0.250101715 },
        {-0.474270727,  0.291281717 },
        {-0.619590501,  0.332461718 },
        {-0.764910275,  0.37364172  },
        {-0.825,        0.39066965  },
        {-0.825,        0.302083626 },
        {-0.825,        0.151041813 },
        {-0.825,        0.0         },
    };
    static const double yz_marker[][2] = {
        {-0.825,  0.39066965 },
        { 0.175,  0.39066965 },
        { 0.175, -0.39066965 },
        {-0.825, -0.39066965 },
        {-0.825,  0.39066965 },
    };
    auto make_poly = [&](double x, const double data[][2], size_t n) {
        std::vector<Point> pts;
        pts.reserve(n);
        for (size_t i = 0; i < n; ++i)
            pts.emplace_back(x, data[i][0], data[i][1]);
        return Polyline(pts);
    };
    joint.f_outlines[0] = { make_poly(0.5, yz, 39), make_poly(0.5, yz_marker, 5) };
    joint.f_outlines[1] = { make_poly(0.0, yz, 39), make_poly(0.0, yz_marker, 5) };
    joint.m_outlines[0] = { make_poly(0.0, yz, 39), make_poly(0.0, yz_marker, 5) };
    joint.m_outlines[1] = { make_poly(0.5, yz, 39), make_poly(0.5, yz_marker, 5) };
    joint.f_cut_types[0] = { wood_cut::conic, wood_cut::conic };
    joint.f_cut_types[1] = { wood_cut::conic, wood_cut::conic };
    joint.m_cut_types[0] = { wood_cut::conic_reverse, wood_cut::conic_reverse };
    joint.m_cut_types[1] = { wood_cut::conic_reverse, wood_cut::conic_reverse };
    joint.unit_scale = true;
}
