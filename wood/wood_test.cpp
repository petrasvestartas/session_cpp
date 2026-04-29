// ─────────────────────────────────────────────────────────────────────────────
// wood/wood_test.cpp — 43 test-function implementations ported from
// `wood/cmake/src/wood/include/wood_test.cpp`.
//
// Each function:
//   1. Loads dataset-specific globals from `wood/config/<name>.yml`.
//   2. Loads the dataset as ElementPlates via internal::load_plates().
//   3. Calls get_connection_zones(plates, search_type) → vector<WoodJoint>.
//   4. Splats result into Session via fill_session(session, plates, joints).
//   4. Persists the result with session.pb_dump().
//
// Per-dataset parameter overrides (JOINTS_PARAMETERS_AND_TYPES, JVE,
// dihedral angle, etc.) live in YAML alongside this file and are loaded
// at runtime — edit them to retune a dataset without rebuilding.
// ─────────────────────────────────────────────────────────────────────────────
#include "wood_session.h"
#include "../src/session.h"

#include <fmt/core.h>

using namespace session_cpp;

// ── wood line 204 ──────────────────────────────────────────────────────────
bool type_plates_name_hexbox_and_corner() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_hexbox_and_corner");
    auto plates = internal::load_plates("type_plates_name_hexbox_and_corner");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_hexbox_and_corner]: {}\n", e.what());
        return false;
    }
}

// ── wood line 265 ──────────────────────────────────────────────────────────
bool type_plates_name_joint_linking_vidychapel_corner() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_joint_linking_vidychapel_corner");
    auto plates = internal::load_plates("type_plates_name_joint_linking_vidychapel_corner");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_joint_linking_vidychapel_corner]: {}\n", e.what());
        return false;
    }
}

// ── wood line 428 ──────────────────────────────────────────────────────────
bool type_plates_name_joint_linking_vidychapel_one_layer() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_joint_linking_vidychapel_one_layer");
    auto plates = internal::load_plates("type_plates_name_joint_linking_vidychapel_one_layer", 0.1);
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_joint_linking_vidychapel_one_layer]: {}\n", e.what());
        return false;
    }
}

// ── wood line 488 ──────────────────────────────────────────────────────────
bool type_plates_name_joint_linking_vidychapel_one_axis_two_layers() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_joint_linking_vidychapel_one_axis_two_layers");
    auto plates = internal::load_plates("type_plates_name_joint_linking_vidychapel_one_axis_two_layers");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_joint_linking_vidychapel_one_axis_two_layers]: {}\n", e.what());
        return false;
    }
}

// ── wood line 611 ──────────────────────────────────────────────────────────
bool type_plates_name_joint_linking_vidychapel_full() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_joint_linking_vidychapel_full");
    auto plates = internal::load_plates("type_plates_name_joint_linking_vidychapel_full");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_joint_linking_vidychapel_full]: {}\n", e.what());
        return false;
    }
}

// ── wood line 888 ──────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_inplane_2_butterflies() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_side_to_side_edge_inplane_2_butterflies");
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_inplane_2_butterflies");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_side_to_side_edge_inplane_2_butterflies]: {}\n", e.what());
        return false;
    }
}

// ── wood line 940 ──────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_inplane_hexshell() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_plates_name_side_to_side_edge_inplane_hexshell")) {
        fmt::print("\n=== inplane_hexshell: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_plates_name_side_to_side_edge_inplane_hexshell");
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_inplane_hexshell");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_side_to_side_edge_inplane_hexshell]: {}\n", e.what());
        return false;
    }
}

// ── wood line 998 ──────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_inplane_differentdirections() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_plates_name_side_to_side_edge_inplane_differentdirections")) {
        fmt::print("\n=== inplane_differentdirections: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_plates_name_side_to_side_edge_inplane_differentdirections");
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_inplane_differentdirections");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_side_to_side_edge_inplane_differentdirections]: {}\n", e.what());
        return false;
    }
}

// ── wood line 1129 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_outofplane_folding() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_side_to_side_edge_outofplane_folding");
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_outofplane_folding");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_side_to_side_edge_outofplane_folding]: {}\n", e.what());
        return false;
    }
}

// ── wood line 1384 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_outofplane_box() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_side_to_side_edge_outofplane_box");
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_outofplane_box");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_side_to_side_edge_outofplane_box]: {}\n", e.what());
        return false;
    }
}

// ── wood line 1440 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_outofplane_tetra() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_side_to_side_edge_outofplane_tetra");
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_outofplane_tetra");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_side_to_side_edge_outofplane_tetra]: {}\n", e.what());
        return false;
    }
}

// ── wood line 1497 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_outofplane_dodecahedron() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_side_to_side_edge_outofplane_dodecahedron");
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_outofplane_dodecahedron");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_side_to_side_edge_outofplane_dodecahedron]: {}\n", e.what());
        return false;
    }
}

// ── wood line 1555 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_outofplane_icosahedron() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_side_to_side_edge_outofplane_icosahedron");
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_outofplane_icosahedron");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_side_to_side_edge_outofplane_icosahedron]: {}\n", e.what());
        return false;
    }
}

// ── wood line 1613 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_outofplane_octahedron() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_side_to_side_edge_outofplane_octahedron");
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_outofplane_octahedron");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_side_to_side_edge_outofplane_octahedron]: {}\n", e.what());
        return false;
    }
}

// ── wood line 1671 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners");
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners]: {}\n", e.what());
        return false;
    }
}

// ── wood line 1729 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_combined() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_combined");
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_combined");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_combined]: {}\n", e.what());
        return false;
    }
}

// ── wood line 1787 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_different_lengths() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_different_lengths");
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_different_lengths");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_different_lengths]: {}\n", e.what());
        return false;
    }
}

// ── wood line 1849 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_inplane_hilti() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_plates_name_side_to_side_edge_inplane_hilti")) {
        fmt::print("\n=== inplane_hilti: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_plates_name_side_to_side_edge_inplane_hilti");
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_inplane_hilti");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_side_to_side_edge_inplane_hilti]: {}\n", e.what());
        return false;
    }
}

// ── wood line 1912 ─────────────────────────────────────────────────────────
bool type_plates_name_top_to_top_pairs() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_top_to_top_pairs");
    auto plates = internal::load_plates("type_plates_name_top_to_top_pairs");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_top_to_top_pairs]: {}\n", e.what());
        return false;
    }
}

// ── wood line 1965 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_outofplane_inplane_and_top_to_top_hexboxes() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_plates_name_side_to_side_edge_outofplane_inplane_and_top_to_top_hexboxes")) {
        fmt::print("\n=== hexboxes: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_plates_name_side_to_side_edge_outofplane_inplane_and_top_to_top_hexboxes");
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_outofplane_inplane_and_top_to_top_hexboxes");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_side_to_side_edge_outofplane_inplane_and_top_to_top_hexboxes]: {}\n", e.what());
        return false;
    }
}

// ── wood line 2037 ─────────────────────────────────────────────────────────
bool type_plates_name_hex_block_rossiniere() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_plates_name_hex_block_rossiniere")) {
        fmt::print("\n=== hex_block_rossiniere: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_plates_name_hex_block_rossiniere");
    auto plates = internal::load_plates("type_plates_name_hex_block_rossiniere");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_hex_block_rossiniere]: {}\n", e.what());
        return false;
    }
}

// ── wood line 2104 ─────────────────────────────────────────────────────────
bool type_plates_name_top_to_side_snap_fit() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_plates_name_top_to_side_snap_fit")) {
        fmt::print("\n=== top_to_side_snap_fit: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_plates_name_top_to_side_snap_fit");
    auto plates = internal::load_plates("type_plates_name_top_to_side_snap_fit");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_top_to_side_snap_fit]: {}\n", e.what());
        return false;
    }
}

// ── wood line 2163 ─────────────────────────────────────────────────────────
bool type_plates_name_top_to_side_box() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_plates_name_top_to_side_box")) {
        fmt::print("\n=== top_to_side_box: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_plates_name_top_to_side_box");
    auto plates = internal::load_plates("type_plates_name_top_to_side_box");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_top_to_side_box]: {}\n", e.what());
        return false;
    }
}

// ── wood line 2220 ─────────────────────────────────────────────────────────
bool type_plates_name_top_to_side_corners() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_plates_name_top_to_side_corners")) {
        fmt::print("\n=== top_to_side_corners: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_plates_name_top_to_side_corners");
    auto plates = internal::load_plates("type_plates_name_top_to_side_corners");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_top_to_side_corners]: {}\n", e.what());
        return false;
    }
}

// ── wood line 2279 ─────────────────────────────────────────────────────────
bool type_plates_name_top_to_side_and_side_to_side_outofplane_annen_corner() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_corner")) {
        fmt::print("\n=== annen_corner: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_corner");
    auto plates = internal::load_plates("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_corner");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_top_to_side_and_side_to_side_outofplane_annen_corner]: {}\n", e.what());
        return false;
    }
}

// ── wood line 2391 ─────────────────────────────────────────────────────────
bool type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box")) {
        fmt::print("\n=== annen_box: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box");
    auto plates = internal::load_plates("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box]: {}\n", e.what());
        return false;
    }
}

// ── wood line 2488 ─────────────────────────────────────────────────────────
bool type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box_pair() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box_pair");
    auto plates = internal::load_plates("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box_pair");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box_pair]: {}\n", e.what());
        return false;
    }
}

// ── wood line 2698 ─────────────────────────────────────────────────────────
// Special: keeps pb output file as "WoodF2F_annen.pb" for existing meta diff.
bool type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_small() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_small");
    auto plates = internal::load_plates("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_small");
    DATA_SET_OUTPUT_FILE = "WoodF2F_annen.pb";  // keep legacy name for existing ref
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_small]: {}\n", e.what());
        return false;
    }
}

// ── wood line 2763 ─────────────────────────────────────────────────────────
bool type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_full_arch() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_full_arch")) {
        fmt::print("\n=== annen_grid_full_arch: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_full_arch");
    auto plates = internal::load_plates("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_full_arch");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_full_arch]: {}\n", e.what());
        return false;
    }
}

// ── wood line 2829 ─────────────────────────────────────────────────────────
bool type_plates_name_vda_floor_0() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_plates_name_vda_floor_0")) {
        fmt::print("\n=== vda_floor_0: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_plates_name_vda_floor_0");
    auto plates = internal::load_plates("type_plates_name_vda_floor_0");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_vda_floor_0]: {}\n", e.what());
        return false;
    }
}

// ── wood line 2888 ─────────────────────────────────────────────────────────
bool type_plates_name_vda_floor_2() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_plates_name_vda_floor_2")) {
        fmt::print("\n=== vda_floor_2: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_plates_name_vda_floor_2");
    auto plates = internal::load_plates("type_plates_name_vda_floor_2");
    auto joints = get_connection_zones(plates, face_to_face);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_vda_floor_2]: {}\n", e.what());
        return false;
    }
}

// ── wood line 2955 ─────────────────────────────────────────────────────────
bool type_plates_name_cross_and_sides_corner() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_plates_name_cross_and_sides_corner")) {
        fmt::print("\n=== cross_and_sides_corner: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_plates_name_cross_and_sides_corner");
    auto plates = internal::load_plates("type_plates_name_cross_and_sides_corner");
    auto joints = get_connection_zones(plates, face_to_face_then_cross);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_cross_and_sides_corner]: {}\n", e.what());
        return false;
    }
}

// ── wood line 3016 ─────────────────────────────────────────────────────────
bool type_plates_name_cross_corners() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_cross_corners");
    auto plates = internal::load_plates("type_plates_name_cross_corners");
    auto joints = get_connection_zones(plates, cross_joint);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_cross_corners]: {}\n", e.what());
        return false;
    }
}

// ── wood line 3080 ─────────────────────────────────────────────────────────
bool type_plates_name_cross_vda_corner() {
    try {
    using namespace wood_session::globals;
    globals_yaml("type_plates_name_cross_vda_corner");
    auto plates = internal::load_plates("type_plates_name_cross_vda_corner");
    auto joints = get_connection_zones(plates, cross_joint);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_cross_vda_corner]: {}\n", e.what());
        return false;
    }
}

// ── wood lines 3144/3207/3270/3333/3396/3459 ──────────────────────────────
// cross_vda_* / cross_square_reciprocal_* — search_type=cross_joint, default globals.
#define SESSION_CROSS_STUB(FN, NAME)                                          \
    bool FN() {                                                               \
        try {                                                                 \
        using namespace wood_session::globals;                                \
        if (!internal::plates_exist(NAME)) {                                  \
            fmt::print("\n=== " NAME ": dataset missing, skipping ===\n");    \
            return false;                                                     \
        }                                                                     \
        globals_yaml(NAME);                                                   \
        auto plates = internal::load_plates(NAME);                            \
        auto joints = get_connection_zones(plates, cross_joint);              \
        Session session("WoodF2F");                                           \
        fill_session(session, plates, joints);                                \
        session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string()); \
        return true;                                                          \
        } catch (const std::exception& e) {                                   \
            fmt::print("  ERROR [" NAME "]: {}\n", e.what());                 \
            return false;                                                     \
        }                                                                     \
    }

SESSION_CROSS_STUB(type_plates_name_cross_vda_hexshell,             "type_plates_name_cross_vda_hexshell")
SESSION_CROSS_STUB(type_plates_name_cross_vda_hexshell_reciprocal,  "type_plates_name_cross_vda_hexshell_reciprocal")
SESSION_CROSS_STUB(type_plates_name_cross_vda_single_arch,          "type_plates_name_cross_vda_single_arch")
SESSION_CROSS_STUB(type_plates_name_cross_vda_shell,                "type_plates_name_cross_vda_shell")
SESSION_CROSS_STUB(type_plates_name_cross_square_reciprocal_two_sides, "type_plates_name_cross_square_reciprocal_two_sides")
SESSION_CROSS_STUB(type_plates_name_cross_square_reciprocal_iseya,  "type_plates_name_cross_square_reciprocal_iseya")
#undef SESSION_CROSS_STUB

// ── wood line 3522 ─────────────────────────────────────────────────────────
bool type_plates_name_cross_ibois_pavilion() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_plates_name_cross_ibois_pavilion")) {
        fmt::print("\n=== cross_ibois_pavilion: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_plates_name_cross_ibois_pavilion");
    auto plates = internal::load_plates("type_plates_name_cross_ibois_pavilion");
    auto joints = get_connection_zones(plates, face_to_face_then_cross);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_cross_ibois_pavilion]: {}\n", e.what());
        return false;
    }
}

// ── wood line 3588 ─────────────────────────────────────────────────────────
bool type_plates_name_cross_brussels_sports_tower() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_plates_name_cross_brussels_sports_tower")) {
        fmt::print("\n=== cross_brussels_sports_tower: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_plates_name_cross_brussels_sports_tower");
    auto plates = internal::load_plates("type_plates_name_cross_brussels_sports_tower");
    auto joints = get_connection_zones(plates, cross_joint);
    Session session("WoodF2F");
    fill_session(session, plates, joints);
    session.pb_dump((internal::session_data_dir() / DATA_SET_OUTPUT_FILE).string());
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_plates_name_cross_brussels_sports_tower]: {}\n", e.what());
        return false;
    }
}

// ── wood line 3670 ─────────────────────────────────────────────────────────
// Beam dataset — uses raw polylines as beam axes (not top/bottom plate pairs).
// Calls beam_volumes_pipeline instead of get_connection_zones.
bool type_beams_name_phanomema_node() {
    try {
    using namespace wood_session::globals;
    if (!internal::plates_exist("type_beams_name_phanomema_node")) {
        fmt::print("\n=== phanomema_node: dataset missing, skipping ===\n");
        return false;
    }
    globals_yaml("type_beams_name_phanomema_node");
    auto axes = internal::load_polylines("type_beams_name_phanomema_node");

    std::vector<std::vector<double>> segment_radii;
    segment_radii.reserve(axes.size());
    for (const auto& ax : axes) {
        std::vector<double> r;
        size_t nseg = ax.point_count() > 1 ? ax.point_count() - 1 : 0;
        r.assign(nseg, 150.0);
        segment_radii.push_back(std::move(r));
    }
    std::vector<std::vector<Vector>> segment_direction;

    std::vector<int> allowed_types{ 1 };
    double min_distance         = 20.0;
    double volume_length        = 500.0;
    double cross_or_side_to_end = 0.91;
    int    flip_male            = 1;

    beam_volumes_pipeline(
        axes, segment_radii, segment_direction,
        allowed_types,
        min_distance, volume_length, cross_or_side_to_end, flip_male);
    return true;
    } catch (const std::exception& e) {
        fmt::print("  ERROR [type_beams_name_phanomema_node]: {}\n", e.what());
        return false;
    }
}
