#include "session.h"
#include "wood/wood_session.h"
#include <filesystem>
using namespace session_cpp;
using namespace wood_session;

int main() {
    auto data = std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data";

    // ── TranslationShell surface ──────────────────────────────────────────────
    Polyline C(std::vector<Point>{
        Point(4668.324796,  -1744.868541, 4.784134),
        Point(4562.643541,  -1744.868541, 126.748409),
        Point(4442.550969,  -1744.868541, 234.470158),
        Point(4306.753536,  -1744.868541, 321.402901),
        Point(4156.323594,  -1744.868541, 379.131783),
        Point(3996.588453,  -1744.868541, 399.552715),
        Point(3836.853312,  -1744.868541, 379.131783),
        Point(3686.423369,  -1744.868541, 321.402901),
        Point(3550.625939,  -1744.868541, 234.47016),
        Point(3430.533365,  -1744.868541, 126.748409),
        Point(3324.85211,   -1744.868541, 4.784134),
    });
    Polyline P(std::vector<Point>{
        Point(3324.85211,   -1744.868541, 4.784134),
        Point(3324.85211,   -1899.208521, 126.657309),
        Point(3324.85211,   -2066.485875, 230.012521),
        Point(3324.85211,   -2245.51187,  311.280689),
        Point(3324.85211,   -2433.939031, 367.343471),
        Point(3324.85211,   -2628.426542, 395.949972),
        Point(3324.85211,   -2825.003177, 395.949972),
        Point(3324.85211,   -3019.490677, 367.343473),
        Point(3324.85211,   -3207.917847, 311.280689),
        Point(3324.85211,   -3386.94384,  230.012522),
        Point(3324.85211,   -3554.221194, 126.657311),
        Point(3324.85211,   -3708.561177, 4.784134),
    });

    Mesh shell_surface = Mesh::translation_shell(C, P);
    auto plates = Mesh::miter_contours(shell_surface, 11.0, 1.0, 6.0, false);

    // ── Wood joints: in-plane (ss_e_ip) — translation shell ──────────────────
    // Translation shell dihedrals ~170-175° → all > 150° threshold → all ss_e_ip naturally
    globals::globals_yaml("type_plates_name_side_to_side_edge_inplane_2_butterflies");
    globals::DATA_SET_INPUT_NAME  = "wood_fold_2_inplane";
    globals::DATA_SET_OUTPUT_FILE = "wood_fold_2.pb";
    // Plate edges ~160-200mm, 60mm division → 2-3 teeth per edge
    globals::JOINTS_PARAMETERS_AND_TYPES[0*3+0] = 60.0;  // ss_e_ip division_length
    globals::JOINTS_PARAMETERS_AND_TYPES[0*3+2] = 1.0;   // ss_e_ip_1: parametric zigzag
    globals::JOINT_VOLUME_EXTENSION = {-7.0, 0.0, 0.0, 0.0, 0.0};

    // Use raw face corners (4-pt quads) so WoodElement computes correct thickness
    std::vector<WoodElement> elements;
    for (auto& [top, bot, top_raw, bot_raw, fn] : plates) {
        std::vector<Point> tc = top_raw; tc.push_back(tc[0]);
        std::vector<Point> bc = bot_raw; bc.push_back(bc[0]);
        elements.emplace_back(Polyline(bc), Polyline(tc));
    }

    auto joints = get_connection_zones(elements, face_to_face);

    Session session(globals::DATA_SET_INPUT_NAME);
    auto g_surf = session.add_group("Surface");
    session.add_mesh(std::make_shared<Mesh>(shell_surface), g_surf);
    fill_session(session, elements, joints, true);
    session.pb_dump((data / globals::DATA_SET_OUTPUT_FILE).string());

    return 0;
}
