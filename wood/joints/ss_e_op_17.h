// ─────────────────────────────────────────────────────────────────────────────
// ss_e_op_17: ss_e_op_0 (N-finger joint) with flat miter cap endings.
// Tenon count N = div/2 where div = clamp(joint.divisions, 2..20) rounded up
// to even. Caps: full cross (x=+0.5↔-0.5 or y=+0.5↔-0.5) on m0 and f1;
// near-dup on m1 and f0 — matching the original 3-finger convention.
// ─────────────────────────────────────────────────────────────────────────────
static void ss_e_op_17(WoodJoint& joint) {
    joint.name = "ss_e_op_17";
    auto P = [](double x, double y, double z) { return Point(x, y, z); };

    int div = std::max(2, std::min(20, joint.divisions));
    div += div % 2;  // force even → N = div/2 tenons, symmetric about z=0

    int N = div / 2;
    double d = 1.0 / (4 * N + 2);  // half-step: z levels at ±(2k-1)*d, k=1..N

    // Female f0 (x=+0.5): near-dup caps, y zigzag ascending z=-a..+a
    {
        std::vector<Point> pts;
        pts.reserve(4 + 4 * N);
        pts.push_back(P( 0.5,  0.5, -0.5));
        pts.push_back(P( 0.5,  0.5, -0.5));
        for (int k = 0; k < 2 * N; k++) {
            double z = -(2 * N - 1 - 2 * k) * d;
            if (k % 2 == 0) { pts.push_back(P( 0.5,  0.5, z)); pts.push_back(P( 0.5, -0.5, z)); }
            else             { pts.push_back(P( 0.5, -0.5, z)); pts.push_back(P( 0.5,  0.5, z)); }
        }
        pts.push_back(P( 0.5,  0.5,  0.5));
        pts.push_back(P( 0.5,  0.5,  0.5));
        Polyline f0_endpoints(std::vector<Point>{ P( 0.5, 0.5, -0.5), P( 0.5, 0.5, 0.5) });
        joint.f_outlines[0] = { Polyline(pts), f0_endpoints };
    }

    // Female f1 (x=-0.5): full cross caps, y zigzag ascending z=-a..+a
    {
        std::vector<Point> pts;
        pts.reserve(4 + 4 * N);
        pts.push_back(P(-0.5, -0.5, -0.5));
        pts.push_back(P(-0.5,  0.5, -0.5));
        for (int k = 0; k < 2 * N; k++) {
            double z = -(2 * N - 1 - 2 * k) * d;
            if (k % 2 == 0) { pts.push_back(P(-0.5,  0.5, z)); pts.push_back(P(-0.5, -0.5, z)); }
            else             { pts.push_back(P(-0.5, -0.5, z)); pts.push_back(P(-0.5,  0.5, z)); }
        }
        pts.push_back(P(-0.5,  0.5,  0.5));
        pts.push_back(P(-0.5, -0.5,  0.5));
        Polyline f1_endpoints(std::vector<Point>{ P(-0.5, -0.5, -0.5), P(-0.5, -0.5, 0.5) });
        joint.f_outlines[1] = { Polyline(pts), f1_endpoints };
    }

    // Male m0 (y=+0.5): full cross caps, x zigzag descending z=+a..-a
    {
        std::vector<Point> pts;
        pts.reserve(4 + 4 * N);
        pts.push_back(P( 0.5,  0.5,  0.5));
        pts.push_back(P(-0.5,  0.5,  0.5));
        for (int k = 0; k < 2 * N; k++) {
            double z = (2 * N - 1 - 2 * k) * d;
            if (k % 2 == 0) { pts.push_back(P(-0.5,  0.5, z)); pts.push_back(P( 0.5,  0.5, z)); }
            else             { pts.push_back(P( 0.5,  0.5, z)); pts.push_back(P(-0.5,  0.5, z)); }
        }
        pts.push_back(P(-0.5,  0.5, -0.5));
        pts.push_back(P( 0.5,  0.5, -0.5));
        Polyline m0_endpoints(std::vector<Point>{ P( 0.5, 0.5, 0.5), P( 0.5, 0.5, -0.5) });
        joint.m_outlines[0] = { Polyline(pts), m0_endpoints };
    }

    // Male m1 (y=-0.5): near-dup caps, x zigzag descending z=+a..-a
    {
        std::vector<Point> pts;
        pts.reserve(4 + 4 * N);
        pts.push_back(P(-0.5, -0.5,  0.5));
        pts.push_back(P(-0.5, -0.5,  0.5));
        for (int k = 0; k < 2 * N; k++) {
            double z = (2 * N - 1 - 2 * k) * d;
            if (k % 2 == 0) { pts.push_back(P(-0.5, -0.5, z)); pts.push_back(P( 0.5, -0.5, z)); }
            else             { pts.push_back(P( 0.5, -0.5, z)); pts.push_back(P(-0.5, -0.5, z)); }
        }
        pts.push_back(P(-0.5, -0.5, -0.5));
        pts.push_back(P(-0.5, -0.5, -0.5));
        Polyline m1_endpoints(std::vector<Point>{ P(-0.5, -0.5, 0.5), P(-0.5, -0.5, -0.5) });
        joint.m_outlines[1] = { Polyline(pts), m1_endpoints };
    }

    joint.f_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.f_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
}
