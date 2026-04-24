// ss_e_op_tutorial: single rectangular notch — tutorial example (type 11, id 18).
// See main_wood_03_new_joint.cpp for the step-by-step recipe to add variants like this.
// Unit cube axes: x=±0.5 (plate top/bottom), y=±0.5 (joint width), z (joint length).
static void ss_e_op_tutorial(WoodJoint& joint) {
    joint.name = "ss_e_op_tutorial";
    auto P = [](double x, double y, double z) { return Point(x, y, z); };
    // Female: rectangular slot on top face (x=+0.5) and bottom face (x=-0.5)
    joint.f_outlines[0] = {
        Polyline(std::vector<Point>{
            P( 0.5,  0.5, -0.25), P( 0.5, -0.5, -0.25),
            P( 0.5, -0.5,  0.25), P( 0.5,  0.5,  0.25)
        }),
        Polyline(std::vector<Point>{ P(0.5, 0.5, -0.5), P(0.5, 0.5, 0.5) })
    };
    joint.f_outlines[1] = {
        Polyline(std::vector<Point>{
            P(-0.5,  0.5, -0.25), P(-0.5, -0.5, -0.25),
            P(-0.5, -0.5,  0.25), P(-0.5,  0.5,  0.25)
        }),
        Polyline(std::vector<Point>{ P(-0.5, 0.5, -0.5), P(-0.5, 0.5, 0.5) })
    };
    // Male: matching tab on sides (y=+0.5 and y=-0.5)
    joint.m_outlines[0] = {
        Polyline(std::vector<Point>{
            P(-0.5, 0.5, -0.25), P( 0.5, 0.5, -0.25),
            P( 0.5, 0.5,  0.25), P(-0.5, 0.5,  0.25)
        }),
        Polyline(std::vector<Point>{ P(-0.5, 0.5, 0.5), P(-0.5, 0.5, -0.5) })
    };
    joint.m_outlines[1] = {
        Polyline(std::vector<Point>{
            P(-0.5, -0.5, -0.25), P( 0.5, -0.5, -0.25),
            P( 0.5, -0.5,  0.25), P(-0.5, -0.5,  0.25)
        }),
        Polyline(std::vector<Point>{ P(-0.5, -0.5, 0.5), P(-0.5, -0.5, -0.5) })
    };
    joint.f_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.f_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[0] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
    joint.m_cut_types[1] = { wood_cut::edge_insertion, wood_cut::edge_insertion };
}
