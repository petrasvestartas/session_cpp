#include "wood/wood_session.h"
#include "../src/session.h"
#include "../src/mesh.h"

using namespace session_cpp;
using namespace wood_session;

// One face of a butterfly tooth in unit-cube space [-0.5, +0.5]^3.
//   y_face        : -0.5 (face 0) or +0.5 (face 1) — the plate-thickness pin.
//                   NOT a scale knob; always at the volume face.
//   depth_x       : signed depth across the seam. Negative = male (carves -X),
//                   positive = female (carves +X). Magnitude = scale knob 1
//                   (in-plane width across the seam).
//   half_length_z : half the tooth's extent along the joint axis. Scale knob 2.
//   neck_ratio    : inner-neck / wing ratio. Default 0.2917 = canonical
//                   butterfly (0.1166666667 / 0.4). Lower = sharper bowtie.
static Polyline butterfly_tooth_face(double y_face,
                                     double depth_x,
                                     double half_length_z,
                                     double neck_ratio = 0.2917) {
    return Polyline(std::vector<Point>{
        Point(    0.0,  y_face,  half_length_z * neck_ratio),
        Point(depth_x,  y_face,  half_length_z),
        Point(depth_x,  y_face, -half_length_z),
        Point(    0.0,  y_face, -half_length_z * neck_ratio),
    });
}


int main() {

    // Load global wood parameters.
    globals::globals_yaml("hello");

    // Main Input - Polylines
    std::vector<Polyline> polylines = {
        // Set of Flat Plates
        Polyline({
            {-500,   0,   0},
            { 500,   0,   0},
            { 500, 500,   0},
            {-500, 500,   0},
            {-500,   0,   0},
        }),
        Polyline({
            {-500,   0, -15},
            { 500,   0, -15},
            { 500, 500, -15},
            {-500, 500, -15},
            {-500,   0, -15},
        }),
        Polyline({
            {-500, -500,   0},
            { 500, -500,   0},
            { 500,   0,   0},
            {-500,   0,   0},
            {-500, -500,   0},
        }),
        Polyline({
            {-500, -500, -15},
            { 500, -500, -15},
            { 500,   0, -15},
            {-500,   0, -15},
            {-500, -500, -15},
        }),
        // Set of Angled Plates
        Polyline({
            {1000,    0,   0},
            {2000,    0,   0},
            {2000,  500,   0},
            {1000,  500,   0},
            {1000,    0,   0},
        }),
        Polyline({
            {1000,    0, -15},
            {2000,    0, -15},
            {2000,  500, -15},
            {1000,  500, -15},
            {1000,    0, -15},
        }),
        Polyline({
            {1000, -500, 134},
            {2000, -500, 134},
            {2000,    0,   0},
            {1000,    0,   0},
            {1000, -500, 134},
        }),
        Polyline({
            {1000, -500, 119},
            {2000, -500, 119},
            {2000,    0, -15},
            {1000,    0, -15},
            {1000, -500, 119},
        }),
    };


    // Build WoodElements from the flat polyline list (even=bottom, odd=top).
    std::vector<WoodElement> elements;
    for (size_t i = 0; i + 1 < polylines.size(); i += 2)
        elements.emplace_back(polylines[i], polylines[i+1]); 


    //Custom joint geometry (id = 9, ss_e_ip_custom) set globally
    const double SEAM_DEPTH = 0.5*4;   // across-seam (world Y) — wider notch as you grow it
    const double TOOTH_HALF = 0.4*4;   // along-seam (world X) — taller individual tooth
    const double NECK_RATIO = 0.2917;  // bowtie pinch (0 = triangle, 1 = rectangle)

    globals::CUSTOM_JOINTS_SS_E_IP_MALE = {
        butterfly_tooth_face(-0.5, -SEAM_DEPTH, TOOTH_HALF, NECK_RATIO),
        butterfly_tooth_face( 0.5, -SEAM_DEPTH, TOOTH_HALF, NECK_RATIO),
    };
    globals::CUSTOM_JOINTS_SS_E_IP_FEMALE = {
        butterfly_tooth_face(-0.5, +SEAM_DEPTH, TOOTH_HALF, NECK_RATIO),
        butterfly_tooth_face( 0.5, +SEAM_DEPTH, TOOTH_HALF, NECK_RATIO),
    };


    // Run the joint-detection algorithm.
    std::vector<WoodJoint> joints = get_connection_zones(elements, face_to_face);

    // Session for visualization and export. 
    Session session(globals::DATA_SET_INPUT_NAME);

    // Merged Polylines
    std::shared_ptr<TreeNode> group_outlines = session.add_group("Outlines");
    for (const WoodElement& element : elements) {
        for (const Polyline& top : element.features.top)
            session.add_polyline(std::make_shared<Polyline>(top), group_outlines);

        for (const Polyline& bot : element.features.bottom)
            session.add_polyline(std::make_shared<Polyline>(bot), group_outlines);
    }

    // Joint areas
    std::shared_ptr<TreeNode> group_areas = session.add_group("Joint Areas");
    for (const WoodJoint& j : joints) {
        session.add_polyline(std::make_shared<Polyline>(j.joint_area), group_areas);
    }

    // Joint volumes
    std::shared_ptr<TreeNode> group_volumes  = session.add_group("Joint Volumes");
    for (const WoodJoint& j : joints) {   
        for (const std::optional<Polyline>& v : j.joint_volumes_pair_a_pair_b)
            if (v.has_value()) 
                session.add_polyline(std::make_shared<Polyline>(*v), group_volumes);
    }

    // Joint lines
    std::shared_ptr<TreeNode> group_lines  = session.add_group("Joint Lines");
    for (const WoodJoint& j : joints) {   
        for (const Line& l : j.joint_lines)
                session.add_line(std::make_shared<Line>(l), group_lines);
    }

    // Joint outlines, before merge
    std::shared_ptr<TreeNode> group_connectors = session.add_group("Connectors");
    for (const auto& j : joints) {
        if (!j.m_outlines[0].empty() && !j.m_outlines[1].empty()) {
            session.add_polyline(std::make_shared<Polyline>(j.m_outlines[0][0]), group_connectors);
            session.add_polyline(std::make_shared<Polyline>(j.m_outlines[1][0]), group_connectors);
        }
        if (!j.f_outlines[0].empty() && !j.f_outlines[1].empty()) {
            session.add_polyline(std::make_shared<Polyline>(j.f_outlines[0][0]), group_connectors);
            session.add_polyline(std::make_shared<Polyline>(j.f_outlines[1][0]), group_connectors);
        }
    }

    // Lofted volumes from merged outlines (bottom → top, with holes)
    std::shared_ptr<TreeNode> group_lofts = session.add_group("Lofts");
    for (const WoodElement& element : elements) {
        if (element.features.bottom.empty() || element.features.top.empty())
            continue;
        session.add_mesh(std::make_shared<Mesh>(Mesh::loft(element.features.bottom, element.features.top)), group_lofts);
    }

    // ── Rhino viewer (paste into Rhino 8 ScriptEditor, venv: session_py) ──────
    //
    //   #! python3
    //   # venv: session_py
    //
    //   import importlib
    //   import session_rhino.session
    //   importlib.reload(session_rhino.session)
    //   from session_rhino.session import Session
    //
    //   filepath = r"C:\brg\code_rust\session\session_data\wood_face_to_face.pb"
    //
    //   scene = Session.load(filepath)
    //   scene.draw(delete=True)
    session.pb_dump((internal::session_data_dir() / wood_session::globals::DATA_SET_OUTPUT_FILE).string());

    return 0;
}
