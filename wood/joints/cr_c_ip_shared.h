// ─── cr_c_ip_2..5 shared core ──────────────────────────────────────────────
// Port of wood_joint_lib.cpp:4720-5475. All four variants share the same
// 16-point layout, rotation, offset, duplication, and side-face
// reconstruction. Differences: drill polylines, extend factors, offset skip
// condition, cut types.
static void cr_c_ip_shared(WoodJoint& joint,
                            const std::vector<Polyline>& extra_drills,
                            double ext_side, double ext_vert,
                            double ext_side_factor_a, double ext_side_factor_b,
                            size_t offset_min_pts,
                            const std::vector<int>& cut_types) {
    double s_param = std::max(std::min(joint.shift, 1.0), 0.0);
    s_param = 0.05 + (s_param - 0.0) * (0.4 - 0.05) / (1.0 - 0.0);
    double a = 0.5 - s_param;
    double b = 0.5;
    double c = 2.0 * (b - a);
    double z = 0.5;

    Point p[16] = {
        Point(a, -a, 0),      Point(-a, -a, 0),      Point(-a, a, 0),      Point(a, a, 0),
        Point(a+c, -a-c, 0),  Point(-a-c, -a-c, 0),  Point(-a-c, a+c, 0),  Point(a+c, a+c, 0),
        Point(b, -b, z),      Point(-b, -b, z),      Point(-b, b, z),      Point(b, b, z),
        Point(b, -b, -z),     Point(-b, -b, -z),     Point(-b, b, -z),     Point(b, b, -z),
    };

    double inv_2a = (a > 1e-12) ? 1.0 / (a * 2.0) : 0.0;
    Vector d01(p[0][0]-p[1][0], p[0][1]-p[1][1], p[0][2]-p[1][2]);
    Vector v0(d01[0]*inv_2a*(0.5-a), d01[1]*inv_2a*(0.5-a), d01[2]*inv_2a*(0.5-a));

    // 5 base polylines (center + 2 top sides + 2 bot sides)
    std::vector<Polyline> base;
    base.push_back(Polyline({p[0]+v0, p[1]-v0, p[2]-v0, p[3]+v0, p[0]+v0}));
    base.push_back(Polyline({p[1]-v0, p[0]+v0, p[8]+v0, p[9]-v0, p[1]-v0}));
    base.push_back(Polyline({p[3]+v0, p[2]-v0, p[10]-v0, p[11]+v0, p[3]+v0}));
    base.push_back(Polyline({p[2], p[1], p[13], p[14], p[2]}));
    base.push_back(Polyline({p[0], p[3], p[15], p[12], p[0]}));

    // Extend bot-side rectangles (polylines 3 and 4)
    base[3].extend_segment_equally(0, ext_side * ext_side_factor_a, ext_side * ext_side_factor_b);
    base[3].extend_segment_equally(2, ext_side * ext_side_factor_a, ext_side * ext_side_factor_b);
    base[4].extend_segment_equally(0, ext_side * ext_side_factor_a, ext_side * ext_side_factor_b);
    base[4].extend_segment_equally(2, ext_side * ext_side_factor_a, ext_side * ext_side_factor_b);
    base[3].extend_segment_equally(1, ext_vert, ext_vert);
    base[3].extend_segment_equally(3, ext_vert, ext_vert);
    base[4].extend_segment_equally(1, ext_vert, ext_vert);
    base[4].extend_segment_equally(3, ext_vert, ext_vert);

    // Append extra drill polylines
    for (const auto& dr : extra_drills) base.push_back(dr);
    int n = (int)base.size();

    double lengths[5] = { 0.5, 0.4, 0.4, 0.4, 0.4 };
    Xform xf_rot = Xform::from_axes(Vector(0,1,0), Vector(1,0,0), Vector(0,0,-1));

    std::vector<Polyline> f0 = base;
    std::vector<Polyline> f1(n);
    std::vector<Polyline> m0(n);
    std::vector<Polyline> m1(n);

    for (int i = 0; i < n; i++) {
        f1[i] = base[i];
        if (base[i].point_count() > offset_min_pts && i < 5) {
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
        }
        m0[i] = base[i].transformed_xform(xf_rot);
        m1[i] = f1[i].transformed_xform(xf_rot);
    }

    // Duplicate 2×
    joint.f_outlines[0].clear(); joint.f_outlines[1].clear();
    joint.m_outlines[0].clear(); joint.m_outlines[1].clear();
    for (int i = 0; i < n; i++) {
        joint.f_outlines[0].push_back(f0[i]); joint.f_outlines[0].push_back(f0[i]);
        joint.f_outlines[1].push_back(f1[i]); joint.f_outlines[1].push_back(f1[i]);
        joint.m_outlines[0].push_back(m0[i]); joint.m_outlines[0].push_back(m0[i]);
        joint.m_outlines[1].push_back(m1[i]); joint.m_outlines[1].push_back(m1[i]);
    }

    // Side-face reconstruction for polylines at duplication indices 2 and 4
    for (int i = 0; i < 2; i++) {
        int id = (i + 1) * 2;
        auto& fo0 = joint.f_outlines[0]; auto& fo1 = joint.f_outlines[1];
        auto& mo0 = joint.m_outlines[0]; auto& mo1 = joint.m_outlines[1];
        Polyline side00({fo0[id].get_point(0), fo0[id].get_point(1), fo1[id].get_point(1), fo1[id].get_point(0), fo0[id].get_point(0)});
        Polyline side01({fo0[id].get_point(3), fo0[id].get_point(2), fo1[id].get_point(2), fo1[id].get_point(3), fo0[id].get_point(3)});
        fo0[id] = side00; fo1[id] = side01;
        fo0[id+1] = side00; fo1[id+1] = side01;
        Polyline mside00({mo0[id].get_point(0), mo0[id].get_point(1), mo1[id].get_point(1), mo1[id].get_point(0), mo0[id].get_point(0)});
        Polyline mside01({mo0[id].get_point(3), mo0[id].get_point(2), mo1[id].get_point(2), mo1[id].get_point(3), mo0[id].get_point(3)});
        mo0[id] = mside00; mo1[id] = mside01;
        mo0[id+1] = mside00; mo1[id+1] = mside01;
    }

    for (int face = 0; face < 2; face++) {
        joint.f_cut_types[face] = cut_types;
        joint.m_cut_types[face] = cut_types;
    }
}
