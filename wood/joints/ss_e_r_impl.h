// ─── ss_e_r_2/3 shared helpers ───────────────────────────────────────────────
// Both ss_e_r_2 and ss_e_r_3 share the same structure:
//   divisions copies along Z, each pushed twice per face (mill_project×2/div),
//   unit_scale=true, joint volumes resized to 120×shift square.
// `unit_scale_distance` must be pre-set to element thickness before calling.
static void ss_e_r_impl(WoodJoint& joint,
    const double m0[][3], int m0n,
    const double m1[][3], int m1n,
    const double f0[][3], int f0n,
    const double f1[][3], int f1n)
{
    int divisions = std::max(1, joint.divisions);
    double edge_length = joint.length * joint.scale[2];
    double jv_len = (joint.unit_scale_distance > 0) ? joint.unit_scale_distance : 40.0;
    double step = edge_length / (divisions * jv_len);
    double total = edge_length / jv_len;
    double z0 = total * 0.5 - step * 0.5;

    joint.m_outlines[0].reserve(2 * divisions);
    joint.m_outlines[1].reserve(2 * divisions);
    joint.f_outlines[0].reserve(2 * divisions);
    joint.f_outlines[1].reserve(2 * divisions);

    for (int i = 0; i < divisions; i++) {
        double z_off = z0 - step * i;
        // Translate shape copies
        auto make_poly = [&](const double pts[][3], int n) {
            std::vector<Point> v;
            v.reserve(n);
            for (int k = 0; k < n; k++)
                v.emplace_back(pts[k][0], pts[k][1], pts[k][2] + z_off);
            return Polyline(v);
        };
        Polyline pm0 = make_poly(m0, m0n);
        Polyline pm1 = make_poly(m1, m1n);
        Polyline pf0 = make_poly(f0, f0n);
        Polyline pf1 = make_poly(f1, f1n);
        // Push each twice (wood: emplace_back twice per division)
        joint.m_outlines[0].push_back(pm0);
        joint.m_outlines[0].push_back(pm0);
        joint.m_outlines[1].push_back(pm1);
        joint.m_outlines[1].push_back(pm1);
        joint.f_outlines[0].push_back(pf0);
        joint.f_outlines[0].push_back(pf0);
        joint.f_outlines[1].push_back(pf1);
        joint.f_outlines[1].push_back(pf1);
    }
    int n = 2 * divisions;
    joint.m_cut_types[0] = std::vector<int>(n, wood_cut::mill_project);
    joint.m_cut_types[1] = std::vector<int>(n, wood_cut::mill_project);
    joint.f_cut_types[0] = std::vector<int>(n, wood_cut::mill_project);
    joint.f_cut_types[1] = std::vector<int>(n, wood_cut::mill_project);
    joint.unit_scale = true;

    // Resize each joint volume to a fixed 120×shift square.
    double size = 120.0 * joint.shift;
    joint.unit_scale_distance = size;
    for (int vi = 0; vi < 4; vi++) {
        auto& opt = joint.joint_volumes_pair_a_pair_b[vi];
        if (!opt || opt->point_count() != 5) continue;
        Polyline& vol = *opt;
        Point p0 = vol.get_point(0);
        Point p1 = vol.get_point(1);
        Point p2 = vol.get_point(2);
        // center = midpoint(p0, p1)
        double cx = (p0[0]+p1[0])*0.5, cy = (p0[1]+p1[1])*0.5, cz = (p0[2]+p1[2])*0.5;
        // x_dir = (p1-p0) normalized × size/2
        double xdx=p1[0]-p0[0], xdy=p1[1]-p0[1], xdz=p1[2]-p0[2];
        double xl = std::sqrt(xdx*xdx+xdy*xdy+xdz*xdz);
        if (xl < 1e-12) continue;
        xdx *= size*0.5/xl; xdy *= size*0.5/xl; xdz *= size*0.5/xl;
        // y_dir = (p2-p1) normalized × size/2
        double ydx=p2[0]-p1[0], ydy=p2[1]-p1[1], ydz=p2[2]-p1[2];
        double yl = std::sqrt(ydx*ydx+ydy*ydy+ydz*ydz);
        if (yl < 1e-12) continue;
        ydx *= size*0.5/yl; ydy *= size*0.5/yl; ydz *= size*0.5/yl;
        // new vol: {center+x+2y, center-x+2y, center-x, center+x, center+x+2y}
        vol = Polyline(std::vector<Point>{
            Point(cx+xdx+2*ydx, cy+xdy+2*ydy, cz+xdz+2*ydz),
            Point(cx-xdx+2*ydx, cy-xdy+2*ydy, cz-xdz+2*ydz),
            Point(cx-xdx,       cy-xdy,        cz-xdz),
            Point(cx+xdx,       cy+xdy,        cz+xdz),
            Point(cx+xdx+2*ydx, cy+xdy+2*ydy, cz+xdz+2*ydz),
        });
    }
}
