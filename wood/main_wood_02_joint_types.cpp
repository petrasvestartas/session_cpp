// main_wood_02_joint_types.cpp — 5 annen datasets demonstrating joint type families.
//
// Runs ss_e_op (type 11) and ts_e_p (type 20) joints.
// Reads polylines from session_data/<name>.obj.
// Each dataset writes its own WoodF2F_<name>.pb.
//
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
//   filepath = r"C:\brg\code_rust\session\session_data\WoodF2F_annen_corner.pb"
//   # other outputs: WoodF2F_annen_box.pb  WoodF2F_annen_box_pair.pb
//   #                WoodF2F_annen.pb (grid_small)
//
//   scene = Session.load(filepath)
//   scene.draw(delete=True)
//
// ─────────────────────────────────────────────────────────────────────────
#include "wood/wood_session.h"
#include "../src/session.h"
using namespace session_cpp;

static void run_annen(const std::string& name, double div_len) {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::plates_exist(name)) {
        fmt::print("\n=== {}: dataset missing, skipping ===\n", name);
        return;
    }
    if (div_len > 0)
        JOINTS_PARAMETERS_AND_TYPES[1*3+0] = div_len;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 20;
    auto plates = internal::load_plates(name);

    Session session("WoodF2F");
    auto merged = get_connection_zones(plates, session, face_to_face);
    loft_merged_elements(session, merged);

    auto pb = (internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string();
    session.pb_dump(pb);
}

int main() {
    run_annen("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_corner",          0);
    run_annen("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box",           200);
    run_annen("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box_pair",      200);
    run_annen("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_small",    200);
    run_annen("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_full_arch",  0);
    return 0;
}
