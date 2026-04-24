// ─── ts_e_p_5 ──────────────────────────────────────────────────────────────
// Verbatim port of wood_joint_lib.cpp:4342-4542. Parametric repeating
// tenon-mortise, `divisions` copies translated along Z.
// Male: all divisions concatenated into one polyline per face (22 pts × div).
// Female: `divisions` separate 5-pt rectangles + 1 bounding rectangle.
// f_cut_types = all hole; m_cut_types = {edge_insertion, edge_insertion}.
// unit_scale = true.
static void ts_e_p_5(WoodJoint& joint) {
    joint.name = "ts_e_p_5";

    // parameters that comes from the joint
    int divisions = std::max(1, joint.divisions);
    // scale down the edge, since wood_joint ->
    // bool joint::orient_to_connection_area()
    // make the distance between joint volumes
    // equal to 2nd joint volume edge
    double edge_length = joint.length * joint.scale[2];
    // normalization to the unit space,
    // joint_volume_edge_length is used for
    // parametrization
    double jv_len = 40.0;
    if (joint.joint_volumes_pair_a_pair_b[0]) {
        const auto& jv = *joint.joint_volumes_pair_a_pair_b[0];
        if (jv.point_count() >= 3) {
            Point p1 = jv.get_point(1);
            Point p2 = jv.get_point(2);
            double dx = p2[0]-p1[0], dy = p2[1]-p1[1], dz = p2[2]-p1[2];
            jv_len = std::sqrt(dx*dx + dy*dy + dz*dz);
        }
    }
    // movement vectors to translate the unit
    // joint to the end of the edge and then to
    // its middle
    double step = edge_length / (divisions * jv_len);
    double total = edge_length / jv_len;
    double z0 = total * 0.5 - step * 0.5; // unit-space Z shift for division 0

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Male default shape
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Hardcoded male profiles (22 pts, x=-0.5 and x=+0.5 faces).
    static const double m0[22][3] = {
        {-0.499996349848395,-0.499996349847159, 1.62789509252326},
        {-0.499996349848395,-3.40695187221018,  1.62789509252326},
        {-0.499996349848395,-3.40695187221018,  1.10464309849792},
        {-0.499996349848395,-0.0232556441796868,1.10464309849801},
        {-0.499996349848395,-0.0232556441796339,0.843017101485296},
        {-0.499996349848395, 0.49999634984571,  0.843017101485296},
        {-0.499996349848395, 0.49999634984571,  1.19185176416881},
        {-0.499996349848395, 0.6038871791861,   1.19185176416881},
        {-0.499996349848395, 1.01079104575736,  1.04650398805065},
        {-0.499996349848395, 1.01079104575736,  0.261625997012692},
        {-0.499996349848395,-3.40695187221018,  0.261625997012692},
        {-0.499996349848395,-3.40695187221018, -0.261625997012652},
        {-0.499996349848395, 1.01079104575736, -0.261625997012652},
        {-0.499996349848395, 1.01079104575736, -1.04650398805062},
        {-0.499996349848395, 0.6038871791861,  -1.19185176416877},
        {-0.499996349848395, 0.49999634984571, -1.19185176416877},
        {-0.499996349848395, 0.49999634984571, -0.843017101485257},
        {-0.499996349848395,-0.0232556441796339,-0.843017101485257},
        {-0.499996349848395,-0.0232556441796868,-1.10464309849797},
        {-0.499996349848395,-3.40695187221018, -1.10464309849788},
        {-0.499996349848395,-3.40695187221018, -1.62789509252322},
        {-0.499996349848395,-0.499996349847159,-1.62789509252322},
    };
    static const double m1[22][3] = {
        {0.499996349844421,-0.49999634984737,   1.62789509252334},
        {0.499996349844421,-3.40695187221039,   1.62789509252334},
        {0.499996349844421,-3.40695187221039,   1.10464309849799},
        {0.499996349844421,-0.0232556441798983, 1.10464309849809},
        {0.499996349844421,-0.0232556441798454, 0.843017101485375},
        {0.499996349844421, 0.499996349845499,  0.843017101485375},
        {0.499996349844421, 0.499996349845499,  1.19185176416889},
        {0.499996349844421, 0.603887179185889,  1.19185176416889},
        {0.499996349844421, 1.01079104575715,   1.04650398805073},
        {0.499996349844421, 1.01079104575715,   0.261625997012771},
        {0.499996349844421,-3.40695187221039,   0.261625997012771},
        {0.499996349844421,-3.40695187221039,  -0.261625997012573},
        {0.499996349844421, 1.01079104575715,  -0.261625997012573},
        {0.499996349844421, 1.01079104575715,  -1.04650398805054},
        {0.499996349844421, 0.603887179185889, -1.19185176416869},
        {0.499996349844421, 0.499996349845499, -1.19185176416869},
        {0.499996349844421, 0.499996349845499, -0.843017101485177},
        {0.499996349844421,-0.0232556441798454,-0.843017101485177},
        {0.499996349844421,-0.0232556441798983,-1.1046430984979},
        {0.499996349844421,-3.40695187221039,  -1.1046430984978},
        {0.499996349844421,-3.40695187221039,  -1.62789509252314},
        {0.499996349844421,-0.49999634984737,  -1.62789509252314},
    };
    // Hardcoded female profiles (5 pts, y≈-0.5 and y≈+0.5).
    static const double f0[5][3] = {
        {-0.499996349848395,-0.499996349847212, 1.10464309849788},
        {-0.499996349848395,-0.499996349847212,-1.104643098498},
        { 0.499996349844421,-0.499996349847317,-1.104643098498},
        { 0.499996349844421,-0.499996349847317, 1.10464309849788},
        {-0.499996349848395,-0.499996349847212, 1.10464309849788},
    };
    static const double f1[5][3] = {
        {-0.499996349848395, 0.499996349845604, 1.104643098498},
        {-0.499996349848395, 0.499996349845604,-1.10464309849788},
        { 0.499996349844421, 0.499996349845499,-1.10464309849788},
        { 0.499996349844421, 0.499996349845499, 1.104643098498},
        {-0.499996349848395, 0.499996349845604, 1.104643098498},
    };

    // Build male: concatenate all divisions into one polyline per face.
    std::vector<Point> m0_pts;
    std::vector<Point> m1_pts;
    m0_pts.reserve(22 * divisions);
    m1_pts.reserve(22 * divisions);
    for (int i = 0; i < divisions; i++) {
        double z_off = z0 - step * i;
        for (int k = 0; k < 22; k++) {
            m0_pts.emplace_back(m0[k][0], m0[k][1], m0[k][2] + z_off);
            m1_pts.emplace_back(m1[k][0], m1[k][1], m1[k][2] + z_off);
        }
    }
    joint.m_outlines[0] = {
        Polyline(m0_pts),
        Polyline(std::vector<Point>{m0_pts.front(), m0_pts.back()}),
    };
    joint.m_outlines[1] = {
        Polyline(m1_pts),
        Polyline(std::vector<Point>{m1_pts.front(), m1_pts.back()}),
    };

    // Build female: one 5-pt rectangle per division, then bounding rectangle.
    joint.f_outlines[0].reserve(divisions + 1);
    joint.f_outlines[1].reserve(divisions + 1);
    for (int i = 0; i < divisions; i++) {
        double z_off = z0 - step * i;
        std::vector<Point> fp0;
        std::vector<Point> fp1;
        fp0.reserve(5);
        fp1.reserve(5);
        for (int k = 0; k < 5; k++) {
            fp0.emplace_back(f0[k][0], f0[k][1], f0[k][2] + z_off);
            fp1.emplace_back(f1[k][0], f1[k][1], f1[k][2] + z_off);
        }
        joint.f_outlines[0].push_back(Polyline(fp0));
        joint.f_outlines[1].push_back(Polyline(fp1));
    }
    // Bounding rectangle spanning all divisions.
    joint.f_outlines[0].push_back(Polyline(std::vector<Point>{
        joint.f_outlines[0].front().get_point(0),
        joint.f_outlines[0].front().get_point(3),
        joint.f_outlines[0].back().get_point(2),
        joint.f_outlines[0].back().get_point(1),
        joint.f_outlines[0].front().get_point(0),
    }));
    joint.f_outlines[1].push_back(Polyline(std::vector<Point>{
        joint.f_outlines[1].front().get_point(0),
        joint.f_outlines[1].front().get_point(3),
        joint.f_outlines[1].back().get_point(2),
        joint.f_outlines[1].back().get_point(1),
        joint.f_outlines[1].front().get_point(0),
    }));

    // Cut types: all holes for female, edge_insertion×2 for male.
    joint.f_cut_types[0] = std::vector<int>(joint.f_outlines[0].size(), wood_cut::hole);
    joint.f_cut_types[1] = std::vector<int>(joint.f_outlines[1].size(), wood_cut::hole);
    joint.m_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.unit_scale = true;
}
