// b_0: beam slice joint. Port of wood_joint_lib.cpp:5867-5945.
// Computes a tween rectangle between the two joint volumes, extends it in
// X (scale[0]) and Y (scale[1]), then emits 4 world-space slice rectangles
// offset along the joint plane normal by ±0.25 (near) and ±(scale[2]+15) (far).
// No female geometry (beam joints are symmetric). Cut type: wood_cut::slice.
// orient is disabled (no_orient=true).
static void b_0(WoodJoint& joint) {
    joint.name = "b_0";
    joint.no_orient = true;

    if (!joint.joint_volumes_pair_a_pair_b[0] || !joint.joint_volumes_pair_a_pair_b[1])
        return;
    const Polyline& vol0 = *joint.joint_volumes_pair_a_pair_b[0];
    const Polyline& vol1 = *joint.joint_volumes_pair_a_pair_b[1];
    if (vol0.point_count() < 5 || vol1.point_count() < 5)
        return;

    // tween_two_polylines(vol0, vol1, 0.5)
    std::vector<Point> r(5);
    for (int i = 0; i < 5; ++i)
        r[i] = Point::lerp(vol0.get_point(i), vol1.get_point(i), 0.5);

    // extend_equally(r, 1, scale[0]) — extends edge [1,2]
    {
        double ex = r[2][0]-r[1][0], ey = r[2][1]-r[1][1], ez = r[2][2]-r[1][2];
        double len = std::sqrt(ex*ex + ey*ey + ez*ez);
        if (len > 1e-12) { double inv = 1.0/len;
            r[1] = Point(r[1][0]-ex*inv*joint.scale[0], r[1][1]-ey*inv*joint.scale[0], r[1][2]-ez*inv*joint.scale[0]);
            r[2] = Point(r[2][0]+ex*inv*joint.scale[0], r[2][1]+ey*inv*joint.scale[0], r[2][2]+ez*inv*joint.scale[0]);
        }
    }
    // extend_equally(r, 3, scale[0]) — extends edge [3,4]; closed: 4==0 so also update r[0]
    {
        double ex = r[4][0]-r[3][0], ey = r[4][1]-r[3][1], ez = r[4][2]-r[3][2];
        double len = std::sqrt(ex*ex + ey*ey + ez*ez);
        if (len > 1e-12) { double inv = 1.0/len;
            r[3] = Point(r[3][0]-ex*inv*joint.scale[0], r[3][1]-ey*inv*joint.scale[0], r[3][2]-ez*inv*joint.scale[0]);
            r[4] = Point(r[4][0]+ex*inv*joint.scale[0], r[4][1]+ey*inv*joint.scale[0], r[4][2]+ez*inv*joint.scale[0]);
            r[0] = r[4]; // closed: segment_id+1 == last → r[0] = r[4]
        }
    }

    // Y-axis: v = (r[1]-r[0]) * 0.5; shift corners toward midline, then extend outward
    double vx = (r[1][0]-r[0][0])*0.5, vy = (r[1][1]-r[0][1])*0.5, vz = (r[1][2]-r[0][2])*0.5;
    r[0] = Point(r[0][0]+vx, r[0][1]+vy, r[0][2]+vz);
    r[1] = Point(r[1][0]-vx, r[1][1]-vy, r[1][2]-vz);
    r[2] = Point(r[2][0]-vx, r[2][1]-vy, r[2][2]-vz);
    r[3] = Point(r[3][0]+vx, r[3][1]+vy, r[3][2]+vz);
    r[4] = Point(r[4][0]+vx, r[4][1]+vy, r[4][2]+vz);
    double vlen = std::sqrt(vx*vx + vy*vy + vz*vz);
    if (vlen > 1e-12) {
        double sc = (joint.scale[1] + 5.0) / vlen;
        r[0] = Point(r[0][0]+vx*sc, r[0][1]+vy*sc, r[0][2]+vz*sc);
        r[3] = Point(r[3][0]+vx*sc, r[3][1]+vy*sc, r[3][2]+vz*sc);
        r[4] = Point(r[4][0]+vx*sc, r[4][1]+vy*sc, r[4][2]+vz*sc);
    }

    // average normal (Newell) of r[0..3]
    double nx = 0, ny = 0, nz = 0;
    for (int i = 0; i < 4; ++i) {
        int j = (i+1) % 4;
        nx += (r[i][1]-r[j][1]) * (r[i][2]+r[j][2]);
        ny += (r[i][2]-r[j][2]) * (r[i][0]+r[j][0]);
        nz += (r[i][0]-r[j][0]) * (r[i][1]+r[j][1]);
    }
    double nlen = std::sqrt(nx*nx + ny*ny + nz*nz);
    if (nlen < 1e-12) return;
    nx /= nlen; ny /= nlen; nz /= nlen;

    // z_axis_offset_from_center = normal * 0.25; z_axis = normal * (scale[2] + 15)
    double ocx = nx*0.25, ocy = ny*0.25, ocz = nz*0.25;
    double fax = nx*(joint.scale[2]+15.0), fay = ny*(joint.scale[2]+15.0), faz = nz*(joint.scale[2]+15.0);

    // shift(r, 2): rotate starting index by 2, keeping closed
    std::rotate(r.begin(), r.begin()+2, r.end()-1);
    r[4] = r[0];

    // 4 translated copies
    auto tr = [&](const std::vector<Point>& pts, double dx, double dy, double dz) -> Polyline {
        std::vector<Point> out(pts.size());
        for (size_t i = 0; i < pts.size(); ++i)
            out[i] = Point(pts[i][0]+dx, pts[i][1]+dy, pts[i][2]+dz);
        return Polyline(out);
    };

    Polyline rect0 = tr(r,  ocx,  ocy,  ocz);
    Polyline rect1 = tr(r,  fax,  fay,  faz);
    Polyline rect2 = tr(r, -ocx, -ocy, -ocz);
    Polyline rect3 = tr(r, -fax, -fay, -faz);

    // m[0]={rect0,rect0,rect2,rect2}, m[1]={rect1,rect1,rect3,rect3}
    joint.m_outlines[0] = { rect0, rect0, rect2, rect2 };
    joint.m_outlines[1] = { rect1, rect1, rect3, rect3 };
    joint.m_cut_types[0] = { wood_cut::slice, wood_cut::slice, wood_cut::slice, wood_cut::slice };
    joint.m_cut_types[1] = { wood_cut::slice, wood_cut::slice, wood_cut::slice, wood_cut::slice };
}
