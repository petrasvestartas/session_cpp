// ─────────────────────────────────────────────────────────────────────────────
// wood/wood_test.cpp — 43 test-function implementations ported from
// `wood/cmake/src/wood/include/wood_test.cpp`.
//
// Architecture mirrors the wood source 1:1: each function resets globals,
// loads the input OBJ, overrides the specific `wood::GLOBALS::*` fields the
// wood test overrides, and calls `get_connection_zones(...)`.
//
// Functions are ordered by wood line number to make cross-file diffing
// straightforward.
// ─────────────────────────────────────────────────────────────────────────────
#include "wood_session.h"
#include "polyline.h"
#include "vector.h"

#include <fmt/core.h>

using namespace session_cpp;

// ── wood line 204 ──────────────────────────────────────────────────────────
bool type_plates_name_hexbox_and_corner() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_hexbox_and_corner");

    JOINTS_PARAMETERS_AND_TYPES[3*1+0] = 450;
    JOINTS_PARAMETERS_AND_TYPES[3*1+1] = 0.64;
    JOINTS_PARAMETERS_AND_TYPES[3*1+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[3*2+0] = 450;
    JOINTS_PARAMETERS_AND_TYPES[3*2+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[3*2+2] = 20;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 265 ──────────────────────────────────────────────────────────
bool type_plates_name_joint_linking_vidychapel_corner() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_joint_linking_vidychapel_corner");

    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 150;  // division_length
    JOINT_VOLUME_EXTENSION[2] = -10;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 428 ──────────────────────────────────────────────────────────
bool type_plates_name_joint_linking_vidychapel_one_layer() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_joint_linking_vidychapel_one_layer",
        /*remove_duplicate_pts*/ true);

    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 50;
    JOINT_VOLUME_EXTENSION[2] = -15;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 488 ──────────────────────────────────────────────────────────
bool type_plates_name_joint_linking_vidychapel_one_axis_two_layers() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines,
        "type_plates_name_joint_linking_vidychapel_one_axis_two_layers");

    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 50;
    JOINT_VOLUME_EXTENSION = { 0,0,-200, 0,0,-200, 0,0,-20, 0,0,-20 };

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 611 ──────────────────────────────────────────────────────────
bool type_plates_name_joint_linking_vidychapel_full() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_joint_linking_vidychapel_full");

    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 50;
    JOINT_VOLUME_EXTENSION[2] = -10;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 888 ──────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_inplane_2_butterflies() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines,
        "type_plates_name_side_to_side_edge_inplane_2_butterflies");

    // No GLOBALS overrides in wood test.

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 940 ──────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_inplane_hexshell() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_plates_name_side_to_side_edge_inplane_hexshell")) {
        fmt::print("\n=== inplane_hexshell: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_side_to_side_edge_inplane_hexshell");

    JOINT_VOLUME_EXTENSION[0] = -20;
    JOINT_VOLUME_EXTENSION[2] = -10;
    JOINTS_PARAMETERS_AND_TYPES[0*3+0] = 40;
    JOINTS_PARAMETERS_AND_TYPES[0*3+2] = 1;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 998 ──────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_inplane_differentdirections() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_plates_name_side_to_side_edge_inplane_differentdirections")) {
        fmt::print("\n=== inplane_differentdirections: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines,
        "type_plates_name_side_to_side_edge_inplane_differentdirections");

    JOINT_VOLUME_EXTENSION[0] = -10;
    JOINT_VOLUME_EXTENSION[2] = -75;
    JOINTS_PARAMETERS_AND_TYPES[0*3+0] = 40;
    JOINTS_PARAMETERS_AND_TYPES[0*3+2] = 1;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 1129 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_outofplane_folding() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines,
        "type_plates_name_side_to_side_edge_outofplane_folding");

    JOINT_VOLUME_EXTENSION[2] = -20;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 1384 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_outofplane_box() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_side_to_side_edge_outofplane_box");

    JOINT_VOLUME_EXTENSION[2] = -100;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 17;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 1440 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_outofplane_tetra() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_side_to_side_edge_outofplane_tetra");

    JOINT_VOLUME_EXTENSION[2] = -100;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 17;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 1497 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_outofplane_dodecahedron() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines,
        "type_plates_name_side_to_side_edge_outofplane_dodecahedron");

    JOINT_VOLUME_EXTENSION[2] = -250;
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 1000;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 14;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 1555 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_outofplane_icosahedron() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines,
        "type_plates_name_side_to_side_edge_outofplane_icosahedron");

    JOINT_VOLUME_EXTENSION[2] = -250;
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 1000;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 14;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 1613 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_outofplane_octahedron() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines,
        "type_plates_name_side_to_side_edge_outofplane_octahedron");

    JOINT_VOLUME_EXTENSION[2] = -250;
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 1000;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 14;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 1671 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines,
        "type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners");

    JOINT_VOLUME_EXTENSION[2] = -20;
    JOINTS_PARAMETERS_AND_TYPES[0*3+2] = 2;   // ss_e_r
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 12;  // ss_e_op

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 1729 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_combined() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines,
        "type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_combined");

    JOINT_VOLUME_EXTENSION[2] = -20;
    JOINTS_PARAMETERS_AND_TYPES[0*3+2] = 2;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 12;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 1787 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_different_lengths() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines,
        "type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_different_lengths");

    JOINT_VOLUME_EXTENSION[2] = -20;
    JOINTS_PARAMETERS_AND_TYPES[0*3+0] = 140;
    JOINTS_PARAMETERS_AND_TYPES[0*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[0*3+2] = 1;
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 140;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 1849 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_inplane_hilti() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_plates_name_side_to_side_edge_inplane_hilti")) {
        fmt::print("\n=== inplane_hilti: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_side_to_side_edge_inplane_hilti");

    JOINTS_PARAMETERS_AND_TYPES[5*3+0] = 500;
    JOINTS_PARAMETERS_AND_TYPES[5*3+1] = 1.0;
    JOINTS_PARAMETERS_AND_TYPES[5*3+2] = 55;
    FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ALL_TREATED_AS_ROTATED  = true;
    FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ROTATED_JOINT_AS_AVERAGE = true;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 1912 ─────────────────────────────────────────────────────────
bool type_plates_name_top_to_top_pairs() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_top_to_top_pairs");

    JOINT_VOLUME_EXTENSION[2] = -10;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 1965 ─────────────────────────────────────────────────────────
bool type_plates_name_side_to_side_edge_outofplane_inplane_and_top_to_top_hexboxes() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_plates_name_side_to_side_edge_outofplane_inplane_and_top_to_top_hexboxes")) {
        fmt::print("\n=== hexboxes: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines,
        "type_plates_name_side_to_side_edge_outofplane_inplane_and_top_to_top_hexboxes");

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

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 2037 ─────────────────────────────────────────────────────────
bool type_plates_name_hex_block_rossiniere() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_plates_name_hex_block_rossiniere")) {
        fmt::print("\n=== hex_block_rossiniere: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_hex_block_rossiniere");

    FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ALL_TREATED_AS_ROTATED = true;
    JOINT_VOLUME_EXTENSION[2] = -20;
    JOINTS_PARAMETERS_AND_TYPES[0*3+0] = 140;
    JOINTS_PARAMETERS_AND_TYPES[0*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[0*3+2] = 1;
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 140;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 2104 ─────────────────────────────────────────────────────────
bool type_plates_name_top_to_side_snap_fit() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_plates_name_top_to_side_snap_fit")) {
        fmt::print("\n=== top_to_side_snap_fit: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_top_to_side_snap_fit");

    JOINTS_PARAMETERS_AND_TYPES[2*3+0] = 300;
    JOINTS_PARAMETERS_AND_TYPES[2*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 25;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 2163 ─────────────────────────────────────────────────────────
bool type_plates_name_top_to_side_box() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_plates_name_top_to_side_box")) {
        fmt::print("\n=== top_to_side_box: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_top_to_side_box");

    JOINTS_PARAMETERS_AND_TYPES[2*3+0] = 300;
    JOINTS_PARAMETERS_AND_TYPES[2*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 25;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 2220 ─────────────────────────────────────────────────────────
bool type_plates_name_top_to_side_corners() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_plates_name_top_to_side_corners")) {
        fmt::print("\n=== top_to_side_corners: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_top_to_side_corners");

    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.66;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[2*3+0] = 300;
    JOINTS_PARAMETERS_AND_TYPES[2*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 20;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 2279 ─────────────────────────────────────────────────────────
bool type_plates_name_top_to_side_and_side_to_side_outofplane_annen_corner() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_corner")) {
        fmt::print("\n=== annen_corner: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines,
        "type_plates_name_top_to_side_and_side_to_side_outofplane_annen_corner");

    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 20;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 2391 ─────────────────────────────────────────────────────────
bool type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box")) {
        fmt::print("\n=== annen_box: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines,
        "type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box");

    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 200;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 20;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 2488 ─────────────────────────────────────────────────────────
bool type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box_pair() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines,
        "type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box_pair");

    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 200;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 20;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 2698 ─────────────────────────────────────────────────────────
// Special: keeps pb output file as "WoodF2F_annen.pb" for existing meta diff.
bool type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_small() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines,
        "type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_small");
    DATA_SET_OUTPUT_FILE = "WoodF2F_annen.pb";  // keep legacy name for existing ref

    // wood_test.cpp:2720-2730
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 200;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 20;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 2763 ─────────────────────────────────────────────────────────
bool type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_full_arch() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_full_arch")) {
        fmt::print("\n=== annen_grid_full_arch: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines,
        "type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_full_arch");

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 2829 ─────────────────────────────────────────────────────────
bool type_plates_name_vda_floor_0() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_plates_name_vda_floor_0")) {
        fmt::print("\n=== vda_floor_0: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_vda_floor_0");

    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 8000;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.66;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[4*3+0] = 100;
    JOINTS_PARAMETERS_AND_TYPES[4*3+1] = 45;
    JOINTS_PARAMETERS_AND_TYPES[4*3+2] = 43;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 2888 ─────────────────────────────────────────────────────────
bool type_plates_name_vda_floor_2() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_plates_name_vda_floor_2")) {
        fmt::print("\n=== vda_floor_2: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_vda_floor_2");

    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 200;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.66;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 10;
    JOINTS_PARAMETERS_AND_TYPES[2*3+0] = 50;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 21;
    JOINT_VOLUME_EXTENSION[2] = -10;

    int search_type = 0;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 2955 ─────────────────────────────────────────────────────────
bool type_plates_name_cross_and_sides_corner() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_plates_name_cross_and_sides_corner")) {
        fmt::print("\n=== cross_and_sides_corner: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_cross_and_sides_corner");

    JOINTS_PARAMETERS_AND_TYPES[0*3+2] = 3;
    JOINTS_PARAMETERS_AND_TYPES[1*3+1] = 0.66;
    JOINTS_PARAMETERS_AND_TYPES[1*3+2] = 12;
    JOINTS_PARAMETERS_AND_TYPES[2*3+0] = 500;
    JOINTS_PARAMETERS_AND_TYPES[2*3+1] = 0.5;
    JOINTS_PARAMETERS_AND_TYPES[2*3+2] = 25;

    int search_type = 2;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 3016 ─────────────────────────────────────────────────────────
bool type_plates_name_cross_corners() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_cross_corners");

    JOINT_VOLUME_EXTENSION[1] = 2;

    int search_type = 1;
    std::vector<double> scale = { 1, 1.00, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 3080 ─────────────────────────────────────────────────────────
bool type_plates_name_cross_vda_corner() {
    using namespace wood_session::globals;
    reset_defaults();
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_cross_vda_corner");

    JOINT_VOLUME_EXTENSION[1] = 2;

    int search_type = 1;
    std::vector<double> scale = { 1, 1.00, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood lines 3144/3207/3270/3333/3396/3459/3522 ──────────────────────────
// cross_vda_* / cross_square_reciprocal_* / cross_ibois_pavilion
// — all search_type=1 with default globals.
#define SESSION_CROSS_STUB(FN, NAME)                                                                \
    bool FN() {                                                                                     \
        using namespace wood_session::globals;                                                      \
        reset_defaults();                                                                           \
        if (!internal::obj_exists(NAME)) {                                                          \
            fmt::print("\n=== " NAME ": OBJ missing, skipping ===\n");                              \
            return false;                                                                           \
        }                                                                                           \
        std::vector<std::pair<int,int>> input_polyline_pairs;                                       \
        std::vector<Polyline> polylines;                                                            \
        internal::set_file_path_for_input_xml_and_screenshot(                                       \
            input_polyline_pairs, polylines, NAME);                                                 \
        int search_type = 1;                                                                        \
        std::vector<double> scale = { 1, 1.00, 1 };                                                 \
        get_connection_zones(search_type, scale);                                                   \
        return true;                                                                                \
    }

SESSION_CROSS_STUB(type_plates_name_cross_vda_hexshell,             "type_plates_name_cross_vda_hexshell")
SESSION_CROSS_STUB(type_plates_name_cross_vda_hexshell_reciprocal,  "type_plates_name_cross_vda_hexshell_reciprocal")
SESSION_CROSS_STUB(type_plates_name_cross_vda_single_arch,          "type_plates_name_cross_vda_single_arch")
SESSION_CROSS_STUB(type_plates_name_cross_vda_shell,                "type_plates_name_cross_vda_shell")
SESSION_CROSS_STUB(type_plates_name_cross_square_reciprocal_two_sides, "type_plates_name_cross_square_reciprocal_two_sides")
SESSION_CROSS_STUB(type_plates_name_cross_square_reciprocal_iseya,  "type_plates_name_cross_square_reciprocal_iseya")
#undef SESSION_CROSS_STUB

// cross_ibois_pavilion: search_type=2, JVE[2]=-20 (wood_test.cpp:3522)
bool type_plates_name_cross_ibois_pavilion() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_plates_name_cross_ibois_pavilion")) {
        fmt::print("\n=== type_plates_name_cross_ibois_pavilion: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_cross_ibois_pavilion");
    JOINT_VOLUME_EXTENSION[2] = -20;
    int search_type = 2;
    std::vector<double> scale = { 1, 1.00, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 3588 ─────────────────────────────────────────────────────────
bool type_plates_name_cross_brussels_sports_tower() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_plates_name_cross_brussels_sports_tower")) {
        fmt::print("\n=== cross_brussels_sports_tower: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> input_polyline_pairs;
    std::vector<Polyline> polylines;
    internal::set_file_path_for_input_xml_and_screenshot(
        input_polyline_pairs, polylines, "type_plates_name_cross_brussels_sports_tower");

    JOINTS_PARAMETERS_AND_TYPES[3*3+2] = 35;
    OUTPUT_GEOMETRY_TYPE = 4; // merge_joints (was 3 = joint-outlines in wood, session has no type-3 mode)
    JOINT_VOLUME_EXTENSION[2] = 0;

    int search_type = 1;
    std::vector<double> scale = { 1, 1, 1 };
    get_connection_zones(search_type, scale);
    return true;
}

// ── wood line 3670 ─────────────────────────────────────────────────────────
// Beam dataset — mirrors wood's `type_beams_name_phanomema_node()` which
// dispatches to `wood::main::beam_volumes(...)` (not the plate pipeline).
// Beam axes are single-segment polylines; each one is an element on its
// own (no top/bottom pairing). Session's equivalent is
// `beam_volumes_pipeline`. Keep the wood tunables verbatim.
bool type_beams_name_phanomema_node() {
    using namespace wood_session::globals;
    reset_defaults();
    if (!internal::obj_exists("type_beams_name_phanomema_node")) {
        fmt::print("\n=== phanomema_node: OBJ missing, skipping ===\n");
        return false;
    }
    std::vector<std::pair<int,int>> dummy_pairs;
    std::vector<Polyline> axes;
    internal::set_file_path_for_input_xml_and_screenshot(
        dummy_pairs, axes, "type_beams_name_phanomema_node");

    // Wood: JOINT_VOLUME_EXTENSION[2] = 0, JOINTS_PARAMETERS_AND_TYPES[3] = 150.
    JOINT_VOLUME_EXTENSION[2] = 0;
    JOINTS_PARAMETERS_AND_TYPES[1*3+0] = 150;

    // Per-segment radii = 150 (wood sets polylines_segment_radii[i][j] = 150
    // for every axis/segment). Direction vectors are left empty → the
    // pipeline computes cross-section normals from the contact geometry.
    std::vector<std::vector<double>> segment_radii;
    segment_radii.reserve(axes.size());
    for (const auto& ax : axes) {
        std::vector<double> r;
        size_t nseg = ax.point_count() > 1 ? ax.point_count() - 1 : 0;
        r.assign(nseg, 150.0);
        segment_radii.push_back(std::move(r));
    }
    std::vector<std::vector<Vector>> segment_direction;

    // Wood constants for phanomema:
    //   allowed_types = {1}, min_distance = 20, volume_length = 500,
    //   cross_or_side_to_end = 0.91, flip_male = 1.
    std::vector<int> allowed_types{ 1 };
    double min_distance          = 20.0;
    double volume_length         = 500.0;
    double cross_or_side_to_end  = 0.91;
    int    flip_male             = 1;

    beam_volumes_pipeline(
        axes, segment_radii, segment_direction,
        allowed_types,
        min_distance, volume_length, cross_or_side_to_end, flip_male);
    return true;
}
