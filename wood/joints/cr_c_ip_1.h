// ─── cr_c_ip_1 ────────────────────────────────���─────────────────────────────
// Verbatim port of wood_joint_lib.cpp:4605-4717. Parametric cross-joint with
// 9 base polylines (center + 2 top sides + 2 bot sides + 4 corners), offset
// along normals, rotated for male, then duplicated 2×. 18 total outlines per
// face. Cut types: 6× mill_project + 12× slice = 18 entries.
static void cr_c_ip_1(WoodJoint& joint) {
    joint.name = "cr_c_ip_1";
    double s_param = std::max(std::min(joint.shift, 1.0), 0.0);
    s_param = 0.05 + (s_param - 0.0) * (0.4 - 0.05) / (1.0 - 0.0);

    double a = 0.5 - s_param;
    double b = 0.5;
    double c = 2.0 * (b - a);
    double z = 0.5;

    // 16 control points: center(4), center_offset(4), top(4), bottom(4)
    Point p[16] = {
        Point(a, -a, 0),          Point(-a, -a, 0),          Point(-a, a, 0),          Point(a, a, 0),
        Point(a+c, -a-c, 0),      Point(-a-c, -a-c, 0),      Point(-a-c, a+c, 0),      Point(a+c, a+c, 0),
        Point(b, -b, z),          Point(-b, -b, z),          Point(-b, b, z),          Point(b, b, z),
        Point(b, -b, -z),         Point(-b, -b, -z),         Point(-b, b, -z),         Point(b, b, -z),
    };

    // v0 = half-width offset along edge direction
    double inv_2a = (a > 1e-12) ? 1.0 / (a * 2.0) : 0.0;
    Vector d01(p[0][0]-p[1][0], p[0][1]-p[1][1], p[0][2]-p[1][2]);
    Vector v0(d01[0]*inv_2a*(0.5-a), d01[1]*inv_2a*(0.5-a), d01[2]*inv_2a*(0.5-a));

    // 9 base polylines for f[0]
    std::vector<Polyline> base(9);
    base[0] = Polyline({p[0]+v0, p[1]-v0, p[2]-v0, p[3]+v0, p[0]+v0});
    base[1] = Polyline({p[1]-v0, p[0]+v0, p[8]+v0, p[9]-v0, p[1]-v0});
    base[2] = Polyline({p[3]+v0, p[2]-v0, p[10]-v0, p[11]+v0, p[3]+v0});
    base[3] = Polyline({p[2], p[1], p[13], p[14], p[2]});
    base[4] = Polyline({p[0], p[3], p[15], p[12], p[0]});
    base[5] = Polyline({p[0], p[12], p[4], p[8], p[0]});
    base[6] = Polyline({p[13], p[1], p[9], p[5], p[13]});
    base[7] = Polyline({p[2], p[14], p[6], p[10], p[2]});
    base[8] = Polyline({p[15], p[3], p[11], p[7], p[15]});

    // Offset lengths per polyline (normal · length)
    double lengths[9] = { 0.5, 0.4, 0.4, 0.4, 0.4, 0.1, 0.1, 0.1, 0.1 };

    // rotation_in_xy_plane(0,1,0; 1,0,0; 0,0,-1) — swaps X↔Y and negates Z
    Xform xf_rot = Xform::from_axes(Vector(0,1,0), Vector(1,0,0), Vector(0,0,-1));

    // Build f[1] by offsetting f[0] along face normals, m[0]/m[1] by rotating
    std::vector<Polyline> f0 = base;
    std::vector<Polyline> f1(9);
    std::vector<Polyline> m0(9);
    std::vector<Polyline> m1(9);

    for (int i = 0; i < 9; i++) {
        f1[i] = base[i]; // copy
        // offset along cross-product normal
        Point pa = f1[i].get_point(0);
        Point pb = f1[i].get_point(1);
        Point pc = f1[i].get_point(2);
        Vector ab(pb[0]-pa[0], pb[1]-pa[1], pb[2]-pa[2]);
        Vector cb(pb[0]-pc[0], pb[1]-pc[1], pb[2]-pc[2]);
        Vector cross = ab.cross(cb);
        cross.normalize_self();
        for (size_t j = 0; j < f1[i].point_count(); j++) {
            Point pt = f1[i].get_point(j);
            f1[i].set_point(j, Point(pt[0]+cross[0]*lengths[i], pt[1]+cross[1]*lengths[i], pt[2]+cross[2]*lengths[i]));
        }

        m0[i] = base[i].transformed_xform(xf_rot);
        m1[i] = f1[i].transformed_xform(xf_rot);
    }

    // Duplicate each polyline 2× (wood pattern: each outline appears twice)
    joint.f_outlines[0].clear(); joint.f_outlines[1].clear();
    joint.m_outlines[0].clear(); joint.m_outlines[1].clear();
    for (int i = 0; i < 9; i++) {
        joint.f_outlines[0].push_back(f0[i]); joint.f_outlines[0].push_back(f0[i]);
        joint.f_outlines[1].push_back(f1[i]); joint.f_outlines[1].push_back(f1[i]);
        joint.m_outlines[0].push_back(m0[i]); joint.m_outlines[0].push_back(m0[i]);
        joint.m_outlines[1].push_back(m1[i]); joint.m_outlines[1].push_back(m1[i]);
    }

    // 18 cut types: 6× mill_project + 12× slice
    std::vector<int> ct(18);
    for (int i = 0; i < 6; i++) ct[i] = wood_cut::mill_project;
    for (int i = 6; i < 18; i++) ct[i] = wood_cut::slice;
    for (int face = 0; face < 2; face++) {
        joint.f_cut_types[face] = ct;
        joint.m_cut_types[face] = ct;
    }
}
