// ─────────────────────────────────────────────────────────────────────────────
// wood/wood_test.cpp — 43 test-function implementations ported from
// `wood/cmake/src/wood/include/wood_test.cpp`.
//
// Each function:
//   1. Resets globals to wood defaults.
//   2. Overrides dataset-specific globals (JOINTS_PARAMETERS_AND_TYPES, etc.).
//   3. Loads the dataset as ElementPlates via internal::load_plates().
//   4. Calls get_connection_zones(plates, session, search_type).
//   5. Persists the result with session.pb_dump().
//
// Functions are ordered by wood line number to make cross-file diffing
// straightforward.
// ─────────────────────────────────────────────────────────────────────────────
#include "wood_session.h"
#include "../src/session.h"

#include <fmt/core.h>

using namespace session_cpp;

// ── wood line 204 ──────────────────────────────────────────────────────────
bool type_plates_name_hexbox_and_corner() {
    try {

    using namespace wood_session::globals;
    reset_defaults();
    JOINTS_PARAMETERS_AND_TYPES[3*1+0] = 450;
    JOINTS_PARAMETERS_AND_TYPES[3*1+1] = 0.64;
    JOINTS_PARAMETERS_AND_TYPES[3*1+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[3*2+0] = 450;
    JOINTS_PARAMETERS_AND_TYPES[3*2+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[3*2+2] = 20;
    auto plates = internal::load_plates("type_plates_name_hexbox_and_corner");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 150;
    JOINT_VOLUME_EXTENSION[2] = -10;
    auto plates = internal::load_plates("type_plates_name_joint_linking_vidychapel_corner");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 50;
    JOINT_VOLUME_EXTENSION[2] = -15;
    auto plates = internal::load_plates("type_plates_name_joint_linking_vidychapel_one_layer", 0.1);
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 50;
    JOINT_VOLUME_EXTENSION = { 0,0,-200, 0,0,-200, 0,0,-20, 0,0,-20 };
    auto plates = internal::load_plates("type_plates_name_joint_linking_vidychapel_one_axis_two_layers");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 50;
    JOINT_VOLUME_EXTENSION[2] = -10;
    auto plates = internal::load_plates("type_plates_name_joint_linking_vidychapel_full");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_inplane_2_butterflies");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    if (!internal::plates_exist("type_plates_name_side_to_side_edge_inplane_hexshell")) {
        fmt::print("\n=== inplane_hexshell: dataset missing, skipping ===\n");
        return false;
    }
    JOINT_VOLUME_EXTENSION[0] = -20;
    JOINT_VOLUME_EXTENSION[2] = -10;
    JOINTS_PARAMETERS_AND_TYPES[0*3+0] = 40;
    JOINTS_PARAMETERS_AND_TYPES[0*3+2] = 1;
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_inplane_hexshell");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    if (!internal::plates_exist("type_plates_name_side_to_side_edge_inplane_differentdirections")) {
        fmt::print("\n=== inplane_differentdirections: dataset missing, skipping ===\n");
        return false;
    }
    JOINT_VOLUME_EXTENSION[0] = -10;
    JOINT_VOLUME_EXTENSION[2] = -75;
    JOINTS_PARAMETERS_AND_TYPES[0*3+0] = 40;
    JOINTS_PARAMETERS_AND_TYPES[0*3+2] = 1;
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_inplane_differentdirections");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    JOINT_VOLUME_EXTENSION[2] = -20;
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_outofplane_folding");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    JOINT_VOLUME_EXTENSION[2] = -100;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 17;
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_outofplane_box");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    JOINT_VOLUME_EXTENSION[2] = -100;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 17;
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_outofplane_tetra");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    JOINT_VOLUME_EXTENSION[2] = -250;
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 1000;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 14;
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_outofplane_dodecahedron");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    JOINT_VOLUME_EXTENSION[2] = -250;
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 1000;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 14;
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_outofplane_icosahedron");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    JOINT_VOLUME_EXTENSION[2] = -250;
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 1000;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 14;
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_outofplane_octahedron");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    JOINT_VOLUME_EXTENSION[2] = -20;
    JOINTS_PARAMETERS_AND_TYPES[0*3+2] = 2;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 12;
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    JOINT_VOLUME_EXTENSION[2] = -20;
    JOINTS_PARAMETERS_AND_TYPES[0*3+2] = 2;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 12;
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_combined");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    JOINT_VOLUME_EXTENSION[2] = -20;
    JOINTS_PARAMETERS_AND_TYPES[0*3+0] = 140;
    JOINTS_PARAMETERS_AND_TYPES[0*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[0*3+2] = 1;
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 140;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_different_lengths");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    if (!internal::plates_exist("type_plates_name_side_to_side_edge_inplane_hilti")) {
        fmt::print("\n=== inplane_hilti: dataset missing, skipping ===\n");
        return false;
    }
    JOINTS_PARAMETERS_AND_TYPES[5*3+0] = 500;
    JOINTS_PARAMETERS_AND_TYPES[5*3+1] = 1.0;
    JOINTS_PARAMETERS_AND_TYPES[5*3+2] = 55;
    FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ALL_TREATED_AS_ROTATED  = true;
    FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ROTATED_JOINT_AS_AVERAGE = true;
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_inplane_hilti");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    JOINT_VOLUME_EXTENSION[2] = -10;
    auto plates = internal::load_plates("type_plates_name_top_to_top_pairs");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    if (!internal::plates_exist("type_plates_name_side_to_side_edge_outofplane_inplane_and_top_to_top_hexboxes")) {
        fmt::print("\n=== hexboxes: dataset missing, skipping ===\n");
        return false;
    }
    JOINT_VOLUME_EXTENSION[2] = -50;
    DISTANCE_SQUARED *= 100;
    JOINTS_PARAMETERS_AND_TYPES[0*3+0] = 140;
    JOINTS_PARAMETERS_AND_TYPES[0*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[0*3+2] = 1;
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 140;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 21;
    JOINTS_PARAMETERS_AND_TYPES[4*3+0] = 100;
    JOINTS_PARAMETERS_AND_TYPES[4*3+1] = 150;
    JOINTS_PARAMETERS_AND_TYPES[4*3+2] = 43;
    auto plates = internal::load_plates("type_plates_name_side_to_side_edge_outofplane_inplane_and_top_to_top_hexboxes");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    if (!internal::plates_exist("type_plates_name_hex_block_rossiniere")) {
        fmt::print("\n=== hex_block_rossiniere: dataset missing, skipping ===\n");
        return false;
    }
    FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ALL_TREATED_AS_ROTATED = true;
    JOINT_VOLUME_EXTENSION[2] = -20;
    JOINTS_PARAMETERS_AND_TYPES[0*3+0] = 140;
    JOINTS_PARAMETERS_AND_TYPES[0*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[0*3+2] = 1;
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 140;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    auto plates = internal::load_plates("type_plates_name_hex_block_rossiniere");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    if (!internal::plates_exist("type_plates_name_top_to_side_snap_fit")) {
        fmt::print("\n=== top_to_side_snap_fit: dataset missing, skipping ===\n");
        return false;
    }
    JOINTS_PARAMETERS_AND_TYPES[2*3+0] = 300;
    JOINTS_PARAMETERS_AND_TYPES[2*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 25;
    auto plates = internal::load_plates("type_plates_name_top_to_side_snap_fit");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    if (!internal::plates_exist("type_plates_name_top_to_side_box")) {
        fmt::print("\n=== top_to_side_box: dataset missing, skipping ===\n");
        return false;
    }
    JOINTS_PARAMETERS_AND_TYPES[2*3+0] = 300;
    JOINTS_PARAMETERS_AND_TYPES[2*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 25;
    auto plates = internal::load_plates("type_plates_name_top_to_side_box");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    if (!internal::plates_exist("type_plates_name_top_to_side_corners")) {
        fmt::print("\n=== top_to_side_corners: dataset missing, skipping ===\n");
        return false;
    }
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.66;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[2*3+0] = 300;
    JOINTS_PARAMETERS_AND_TYPES[2*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 20;
    auto plates = internal::load_plates("type_plates_name_top_to_side_corners");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    if (!internal::plates_exist("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_corner")) {
        fmt::print("\n=== annen_corner: dataset missing, skipping ===\n");
        return false;
    }
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 20;
    auto plates = internal::load_plates("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_corner");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    if (!internal::plates_exist("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box")) {
        fmt::print("\n=== annen_box: dataset missing, skipping ===\n");
        return false;
    }
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 200;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 20;
    auto plates = internal::load_plates("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 200;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 20;
    auto plates = internal::load_plates("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box_pair");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 200;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 20;
    auto plates = internal::load_plates("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_small");
    DATA_SET_OUTPUT_FILE = "WoodF2F_annen.pb";  // keep legacy name for existing ref
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    if (!internal::plates_exist("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_full_arch")) {
        fmt::print("\n=== annen_grid_full_arch: dataset missing, skipping ===\n");
        return false;
    }
    auto plates = internal::load_plates("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_full_arch");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    if (!internal::plates_exist("type_plates_name_vda_floor_0")) {
        fmt::print("\n=== vda_floor_0: dataset missing, skipping ===\n");
        return false;
    }
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 8000;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.66;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[4*3+0] = 100;
    JOINTS_PARAMETERS_AND_TYPES[4*3+1] = 45;
    JOINTS_PARAMETERS_AND_TYPES[4*3+2] = 43;
    auto plates = internal::load_plates("type_plates_name_vda_floor_0");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    if (!internal::plates_exist("type_plates_name_vda_floor_2")) {
        fmt::print("\n=== vda_floor_2: dataset missing, skipping ===\n");
        return false;
    }
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 200;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.66;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[2*3+0] = 50;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 21;
    JOINT_VOLUME_EXTENSION[2] = -10;
    auto plates = internal::load_plates("type_plates_name_vda_floor_2");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face));
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
    reset_defaults();
    if (!internal::plates_exist("type_plates_name_cross_and_sides_corner")) {
        fmt::print("\n=== cross_and_sides_corner: dataset missing, skipping ===\n");
        return false;
    }
    JOINTS_PARAMETERS_AND_TYPES[0*3+2] = 3;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.66;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 12;
    JOINTS_PARAMETERS_AND_TYPES[2*3+0] = 500;
    JOINTS_PARAMETERS_AND_TYPES[2*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 25;
    auto plates = internal::load_plates("type_plates_name_cross_and_sides_corner");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face_then_cross));
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
    reset_defaults();
    JOINT_VOLUME_EXTENSION[1] = 2;
    auto plates = internal::load_plates("type_plates_name_cross_corners");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, cross_joint));
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
    reset_defaults();
    JOINT_VOLUME_EXTENSION[1] = 2;
    auto plates = internal::load_plates("type_plates_name_cross_vda_corner");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, cross_joint));
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
        reset_defaults();                                                     \
        if (!internal::plates_exist(NAME)) {                                  \
            fmt::print("\n=== " NAME ": dataset missing, skipping ===\n");    \
            return false;                                                     \
        }                                                                     \
        auto plates = internal::load_plates(NAME);                            \
        Session session("WoodF2F");                                           \
        loft_merged_elements(session, get_connection_zones(plates, session, cross_joint)); \
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
// cross_ibois_pavilion: search_type=face_to_face_then_cross, JVE[2]=-20.
bool type_plates_name_cross_ibois_pavilion() {
    try {

    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::plates_exist("type_plates_name_cross_ibois_pavilion")) {
        fmt::print("\n=== cross_ibois_pavilion: dataset missing, skipping ===\n");
        return false;
    }
    JOINT_VOLUME_EXTENSION[2] = -20;
    auto plates = internal::load_plates("type_plates_name_cross_ibois_pavilion");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, face_to_face_then_cross));
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
    reset_defaults();
    if (!internal::plates_exist("type_plates_name_cross_brussels_sports_tower")) {
        fmt::print("\n=== cross_brussels_sports_tower: dataset missing, skipping ===\n");
        return false;
    }
    JOINTS_PARAMETERS_AND_TYPES[3*3+2] = 35;
    OUTPUT_GEOMETRY_TYPE = 4;
    JOINT_VOLUME_EXTENSION[2] = 0;
    auto plates = internal::load_plates("type_plates_name_cross_brussels_sports_tower");
    Session session("WoodF2F");
    loft_merged_elements(session, get_connection_zones(plates, session, cross_joint));
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
    reset_defaults();
    if (!internal::plates_exist("type_beams_name_phanomema_node")) {
        fmt::print("\n=== phanomema_node: dataset missing, skipping ===\n");
        return false;
    }
    JOINT_VOLUME_EXTENSION[2] = 0;
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 150;
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
