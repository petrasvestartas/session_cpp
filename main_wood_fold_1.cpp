#include "session.h"
#include "wood/wood_session.h"
#include <filesystem>
using namespace session_cpp;
using namespace wood_session;

int main() {
    auto data = std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data";

    // ── ReflexFold surface ────────────────────────────────────────────────────
    Polyline cross_section(std::vector<Point>{
        Point(-1.766628,    -1744.868541, 0),
        Point(230.700239,   -1744.868541, 578.230273),
        Point(964.66442,    -1744.868541, 738.362403),
        Point(1713.004411,  -1744.868541, 604.197646),
        Point(2035.432618,  -1744.868541, 4.784134),
    });
    Polyline profile(std::vector<Point>{
        Point(-1.766628,    -1744.868541, 0),
        Point(-78.85821,    -1834.648229, 0),
        Point(-20.962529,   -1924.427917, 0),
        Point(-91.269667,   -2014.207605, 0),
        Point(-27.678836,   -2103.987293, 0),
        Point(-91.431136,   -2193.766981, 0),
        Point(-21.238682,   -2283.546669, 0),
        Point(-79.251771,   -2373.326357, 0),
        Point(-2.584496,    -2463.106045, 0),
        Point(-56.551935,   -2552.885733, 0),
        Point(24.610977,    -2642.665422, 0),
        Point(-27.687968,   -2732.44511,  0),
        Point(53.791589,    -2822.224798, 0),
        Point(0.197576,     -2912.004486, 0),
        Point(77.400488,    -3001.784174, 0),
        Point(19.598326,    -3091.563862, 0),
        Point(89.696025,    -3181.34355,  0),
        Point(25.425603,    -3271.123238, 0),
        Point(88.064146,    -3360.902926, 0),
    });

    Mesh fold_surface = Mesh::reflex_fold(cross_section, profile);
    const double thickness = 10.0;
    auto plates = Mesh::miter_contours(fold_surface, thickness, 20.0, 20.0, false, 45.0);

    // ── Wood joints: out-of-plane at fold edges, in-plane at arch edges ──────
    // Threshold 150°: fold profile edges (~90-150° dihedral) → ss_e_op
    //                 arch cross-section edges (~160-175° dihedral) → ss_e_ip
    globals::globals_yaml("type_plates_name_side_to_side_edge_outofplane_folding");
    globals::DATA_SET_INPUT_NAME  = "wood_fold_1_mixed";
    globals::DATA_SET_OUTPUT_FILE = "wood_fold_1.pb";
    // ss_e_op (fold edges, ≤150°): 150mm div, id=17 (parametric miter endings)
    globals::JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 150.0;
    globals::JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.5;
    globals::JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 17.0;
    globals::JOINT_VOLUME_EXTENSION = {0.0, 0.0, -60.0, 0.0, 0.0};
    globals::LIMIT_MIN_JOINT_LENGTH = std::abs(globals::JOINT_VOLUME_EXTENSION[2]) * 2;

    // Use raw face corners (4-pt quads) so WoodElement computes correct thickness
    std::vector<WoodElement> elements;
    for (auto& [top, bot, top_raw, bot_raw, fn] : plates) {
        std::vector<Point> tc = top; tc.push_back(tc[0]);
        std::vector<Point> bc = bot; bc.push_back(bc[0]);
        elements.emplace_back(Polyline(bc), Polyline(tc));
    }

    auto joints = get_connection_zones(elements, face_to_face);

    Session session(globals::DATA_SET_INPUT_NAME);
    auto g_surf = session.add_group("Surface");
    session.add_mesh(std::make_shared<Mesh>(fold_surface), g_surf);
    fill_session(session, elements, joints, true);

    session.pb_dump((data / globals::DATA_SET_OUTPUT_FILE).string());

    return 0;
}
