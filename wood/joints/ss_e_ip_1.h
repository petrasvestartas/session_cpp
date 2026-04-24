// ss_e_ip_1: side-to-side IN-PLANE PARAMETRIC zigzag joint (type=12, id=1).
// Verbatim port of wood_joint_lib.cpp:670-754. Walks `divisions` interpolated
// points along the central z-axis at (0, -0.5, *) and shifts them ±x per
// interleave to build a single zigzag polyline. The opposite face polyline
// (y=+0.5) is the same line offset by `v_o = (0, 1, 0)`. Both faces use the
// same outline for m and f (in-plane joints are symmetric).
static void ss_e_ip_1(WoodJoint& joint) {
    joint.name = "ss_e_ip_1";

    ////////////////////////////////////////////////////////////////////
    // Number of divisions
    // Input wood::joint line (its lengths)
    // Input distance for division
    ////////////////////////////////////////////////////////////////////
    int div = std::max(2, std::min(100, joint.divisions));
    div += div % 2; // force even

    //////////////////////////////////////////////////////////////////////////////////////////
    // Interpolate points
    //////////////////////////////////////////////////////////////////////////////////////////
    auto pts0 = Point::interpolate(Point(0, -0.5, 0.5), Point(0, -0.5, -0.5), div, 0);
    Vector v(0.5, 0, 0);
    double shift_ = Intersection::remap(joint.shift, 0, 1.0, -0.5, 0.5);
    Vector v_d(0, 0, -(1.0 / ((div + 1) * 2)) * shift_);

    std::vector<Point> pline0;
    pline0.reserve(pts0.size() * 2);
    pline0.push_back(pts0[0]);
    pline0.push_back(Point(pts0[0][0]-v[0]-v_d[0],
                            pts0[0][1]-v[1]-v_d[1],
                            pts0[0][2]-v[2]-v_d[2]));
    for (int i = 1; i + 1 < (int)pts0.size(); i++) {
        if (i % 2 == 1) {
            pline0.push_back(Point(pts0[i][0]-v[0]+v_d[0],
                                    pts0[i][1]-v[1]+v_d[1],
                                    pts0[i][2]-v[2]+v_d[2]));
            pline0.push_back(Point(pts0[i][0]+v[0]-v_d[0],
                                    pts0[i][1]+v[1]-v_d[1],
                                    pts0[i][2]+v[2]-v_d[2]));
        } else {
            pline0.push_back(Point(pts0[i][0]+v[0]+v_d[0],
                                    pts0[i][1]+v[1]+v_d[1],
                                    pts0[i][2]+v[2]+v_d[2]));
            pline0.push_back(Point(pts0[i][0]-v[0]-v_d[0],
                                    pts0[i][1]-v[1]-v_d[1],
                                    pts0[i][2]-v[2]-v_d[2]));
        }
    }
    Point last = pts0.back();
    pline0.push_back(Point(last[0]-v[0]+v_d[0],
                            last[1]-v[1]+v_d[1],
                            last[2]-v[2]+v_d[2]));
    pline0.push_back(last);

    // pline1 = pline0 offset by v_o = (0, 1, 0)
    Vector v_o(0, 1, 0);
    std::vector<Point> pline1;
    pline1.reserve(pline0.size());
    for (const Point& p : pline0)
        pline1.emplace_back(p[0]+v_o[0], p[1]+v_o[1], p[2]+v_o[2]);

    Polyline outline0(pline0);
    Polyline outline1(pline1);
    Polyline endpoints0(std::vector<Point>{pline0.front(), pline0.back()});
    Polyline endpoints1(std::vector<Point>{pline1.front(), pline1.back()});

    // Joint lines, always the last line or
    // rectangle is not a wood::joint but an
    // cutting wood::element
    joint.f_outlines[0] = { outline0, endpoints0 };
    joint.f_outlines[1] = { outline1, endpoints1 };
    joint.m_outlines[0] = { outline0, endpoints0 };
    joint.m_outlines[1] = { outline1, endpoints1 };
    // Wood: f/m boolean types = {edge_insertion, edge_insertion}
    // (wood_joint_lib.cpp:752-753).
    joint.f_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.f_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
}
