// ═══════════════════════════════════════════════════════════════════════════
// wood/wood_session.h — shared API surface between the wood pipeline
// (wood/wood_main.cpp) and the test harness (wood/wood_test.cpp).
//
// Mirrors the relevant declarations from wood's source tree:
//   wood::GLOBALS  → wood_session::globals   (wood_globals.cpp)
//   internal::     → internal::              (wood_internal.cpp)
//   wood::main::   → get_connection_zones    (wood_main.cpp)
//   wood_test.h    → 43 type_plates_name_*() (wood_test.cpp)
//
// Implementations are split across the translation units listed above.
// `main_5.cpp` is a near-empty entry point that only contains `int main()`.
//
// Further split candidates (deferred): wood/wood_joint.{h,cpp} (WoodJoint +
// orient helpers) and wood/wood_element.{h,cpp} (WoodElement + merge).
// Currently those pieces live inline in wood_main.cpp's anonymous namespace.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

// Forward-declare session types so this header stays lightweight.
namespace session_cpp { class Polyline; class Vector; }

// ═══════════════════════════════════════════════════════════════════════════
// wood_session::globals — mirror of wood's `wood::GLOBALS`. Definitions +
// `reset_defaults()` live in wood_globals.cpp. Each `type_plates_name_*()`
// wrapper calls `reset_defaults()` then overrides whichever entries the
// corresponding wood test overrides.
// ═══════════════════════════════════════════════════════════════════════════
namespace wood_session {
namespace globals {
    extern std::vector<double> JOINTS_PARAMETERS_AND_TYPES;  // wood_globals.cpp:49
    extern std::vector<double> JOINT_VOLUME_EXTENSION;       // wood_globals.cpp:25
    extern int    OUTPUT_GEOMETRY_TYPE;                      // wood_globals.cpp:32
    extern double FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_DIHEDRAL_ANGLE;
    extern bool   FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ALL_TREATED_AS_ROTATED;
    extern bool   FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ROTATED_JOINT_AS_AVERAGE;
    extern double DISTANCE_SQUARED;
    extern std::string DATA_SET_OUTPUT_FILE;
    extern std::string DATA_SET_INPUT_NAME;
    extern bool   REMOVE_DUPLICATE_PTS;

    void reset_defaults();
}} // namespace wood_session::globals

// ═══════════════════════════════════════════════════════════════════════════
// internal:: — mirror of wood's `internal::` helpers from wood_test.cpp.
// Implementations live in wood_internal.cpp.
// ═══════════════════════════════════════════════════════════════════════════
namespace internal {

// Absolute path to the `session_data/` directory at the repo root. All OBJ,
// .pb, .txt reads and writes resolve against this single anchor.
std::filesystem::path session_data_dir();

// Returns true iff `<short_name>.obj` exists in session_data/ for the given
// wood test function name. Used by the test wrappers to early-skip datasets
// whose OBJ hasn't been extracted yet.
bool obj_exists(const std::string& wood_name);

// Wood-parity helper: populates `polylines_out` from the dataset's OBJ,
// fills `pairs_out` as consecutive (top, bottom) index pairs, and wires
// `wood_session::globals::DATA_SET_INPUT_NAME` + `DATA_SET_OUTPUT_FILE`.
// Set `remove_duplicate_pts = true` to mirror wood_xml.cpp's consecutive-
// duplicate trim (wood does this only for vidychapel_one_layer).
void set_file_path_for_input_xml_and_screenshot(
        std::vector<std::pair<int,int>>& pairs_out,
        std::vector<session_cpp::Polyline>& polylines_out,
        const std::string& name,
        bool remove_duplicate_pts = false);

} // namespace internal

// ═══════════════════════════════════════════════════════════════════════════
// get_connection_zones — wood-style pipeline entry (wood_main.h equivalent).
// ═══════════════════════════════════════════════════════════════════════════
void get_connection_zones(
        int search_type = 0,
        const std::vector<double>& scale = {1.0, 1.0, 1.0});

// ═══════════════════════════════════════════════════════════════════════════
// beam_volumes_pipeline — beam (axis+radius) entry point. Equivalent of
// wood's `wood::main::beam_volumes`. Used by type_beams_name_* datasets
// (e.g. phanomema_node) that store beam axes rather than plate outlines.
// ═══════════════════════════════════════════════════════════════════════════
void beam_volumes_pipeline(
        const std::vector<session_cpp::Polyline>& axes,
        const std::vector<std::vector<double>>& segment_radii,
        const std::vector<std::vector<session_cpp::Vector>>& segment_direction,
        const std::vector<int>& allowed_types_per_polyline,
        double min_distance,
        double volume_length,
        double cross_or_side_to_end,
        int    flip_male);

// ═══════════════════════════════════════════════════════════════════════════
// wood_test.h — 43 test function declarations (wood line-number order).
// ═══════════════════════════════════════════════════════════════════════════
bool type_plates_name_hexbox_and_corner();                                            // 204
bool type_plates_name_joint_linking_vidychapel_corner();                              // 265
bool type_plates_name_joint_linking_vidychapel_one_layer();                           // 428
bool type_plates_name_joint_linking_vidychapel_one_axis_two_layers();                 // 488
bool type_plates_name_joint_linking_vidychapel_full();                                // 611
bool type_plates_name_side_to_side_edge_inplane_2_butterflies();                      // 888
bool type_plates_name_side_to_side_edge_inplane_hexshell();                           // 940
bool type_plates_name_side_to_side_edge_inplane_differentdirections();                // 998
bool type_plates_name_side_to_side_edge_outofplane_folding();                         // 1129
bool type_plates_name_side_to_side_edge_outofplane_box();                             // 1384
bool type_plates_name_side_to_side_edge_outofplane_tetra();                           // 1440
bool type_plates_name_side_to_side_edge_outofplane_dodecahedron();                    // 1497
bool type_plates_name_side_to_side_edge_outofplane_icosahedron();                     // 1555
bool type_plates_name_side_to_side_edge_outofplane_octahedron();                      // 1613
bool type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners();          // 1671
bool type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_combined(); // 1729
bool type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_different_lengths(); // 1787
bool type_plates_name_side_to_side_edge_inplane_hilti();                              // 1849
bool type_plates_name_top_to_top_pairs();                                             // 1912
bool type_plates_name_side_to_side_edge_outofplane_inplane_and_top_to_top_hexboxes(); // 1965
bool type_plates_name_hex_block_rossiniere();                                         // 2037
bool type_plates_name_top_to_side_snap_fit();                                         // 2104
bool type_plates_name_top_to_side_box();                                              // 2163
bool type_plates_name_top_to_side_corners();                                          // 2220
bool type_plates_name_top_to_side_and_side_to_side_outofplane_annen_corner();         // 2279
bool type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box();            // 2391
bool type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box_pair();       // 2488
bool type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_small();     // 2698
bool type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_full_arch(); // 2763
bool type_plates_name_vda_floor_0();                                                  // 2829
bool type_plates_name_vda_floor_2();                                                  // 2888
bool type_plates_name_cross_and_sides_corner();                                       // 2955
bool type_plates_name_cross_corners();                                                // 3016
bool type_plates_name_cross_vda_corner();                                             // 3080
bool type_plates_name_cross_vda_hexshell();                                           // 3144
bool type_plates_name_cross_vda_hexshell_reciprocal();                                // 3207
bool type_plates_name_cross_vda_single_arch();                                        // 3270
bool type_plates_name_cross_vda_shell();                                              // 3333
bool type_plates_name_cross_square_reciprocal_two_sides();                            // 3396
bool type_plates_name_cross_square_reciprocal_iseya();                                // 3459
bool type_plates_name_cross_ibois_pavilion();                                         // 3522
bool type_plates_name_cross_brussels_sports_tower();                                  // 3588
bool type_beams_name_phanomema_node();                                                // 3670
