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
// Structs: wood/wood_joint.h (WoodJoint) and wood/wood_element.h (WoodElement).
// Pipeline helpers (orient, merge) remain in wood_main.cpp's anonymous namespace.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

// Polyline is held by value inside CrossJoint → need the full type here.
#include "../src/polyline.h"
// ElementPlate is returned by value from load_plates and passed to get_connection_zones.
#include "../src/element.h"

namespace session_cpp { class Vector; class Plane; class Line; class Session; }

// ═══════════════════════════════════════════════════════════════════════════
// wood_session::CrossJoint + plane_to_face — side-to-side cross/lap joint
// detection between two plate elements. Wood-domain geometry (joint area /
// volumes / lines + plate face indices + type-30 code), so it lives here
// rather than in the general session_cpp::Intersection kernel.
// Implementations in wood_joint_detection.cpp.
// ═══════════════════════════════════════════════════════════════════════════
namespace wood_session {

struct CrossJoint {
    int type = 30;                                          ///< Joint type code (30 = side-to-side cross)
    std::pair<int,int> face_ids_a{-1,-1};                   ///< Two side-face indices of element A involved
    std::pair<int,int> face_ids_b{-1,-1};                   ///< Two side-face indices of element B involved
    session_cpp::Polyline joint_area;                       ///< Closed quad on the mid-plane (5 pts)
    std::array<session_cpp::Polyline,2> joint_lines;        ///< Two perpendicular centerlines of joint_area
    std::array<session_cpp::Polyline,2> joint_volumes;      ///< Two parallel quads bounding the joint volume
};

/// Cross/lap joint detection between two plate elements (side-to-side).
bool plane_to_face(
    const std::array<session_cpp::Polyline,2>& polylines_a,
    const std::array<session_cpp::Polyline,2>& polylines_b,
    const std::array<session_cpp::Plane,2>& planes_a,
    const std::array<session_cpp::Plane,2>& planes_b,
    CrossJoint& result,
    double angle_tol = 5.0,
    const std::array<double,3>& extension = {0.0, 0.0, 0.0});

/// Convenience overload taking ElementPlate pointers (extracts polylines/planes).
bool plane_to_face(
    session_cpp::ElementPlate* a,
    session_cpp::ElementPlate* b,
    CrossJoint& result,
    double angle_tol = 5.0,
    const std::array<double,3>& extension = {0.0, 0.0, 0.0});

/// Set the near-coplanar rejection threshold used internally by plane_to_face.
/// Wood reads from wood_session::globals::DISTANCE_SQUARED which some tests
/// (hexboxes) mutate. Caller syncs this before face_to_face iteration.
void set_cross_joint_distance_squared(double dist_sq);

} // namespace wood_session

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
    extern double DUPLICATE_PTS_TOL;

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

// Returns true iff the dataset for the given wood test function name exists
// in session_data/ (checks the underlying .obj file).
bool plates_exist(const std::string& wood_name);

// Load a named wood dataset from session_data/ and return one ElementPlate per timber plate.
// Consecutive polylines (even index = bottom, odd = top) are paired into plates.
// Also sets globals DATA_SET_INPUT_NAME and DATA_SET_OUTPUT_FILE as side effects.
// dataset_name — full wood test function name, e.g. "type_plates_name_hexbox_and_corner"
// duplicate_pts_tol — if > 0, removes consecutive duplicate points (vidychapel datasets)
std::vector<session_cpp::ElementPlate> load_plates(
        const std::string& dataset_name,
        double duplicate_pts_tol = 0.0);

// Load raw polylines from a named dataset (no top/bottom pairing).
// Used by beam datasets where each polyline is a beam axis.
// Also sets globals DATA_SET_INPUT_NAME and DATA_SET_OUTPUT_FILE as side effects.
std::vector<session_cpp::Polyline> load_polylines(
        const std::string& dataset_name,
        double duplicate_pts_tol = 0.0);

} // namespace internal

// ═══════════════════════════════════════════════════════════════════════════
// SearchType — controls which joint detection pass get_connection_zones runs.
// ═══════════════════════════════════════════════════════════════════════════
enum SearchType : int {
    face_to_face            = 0,  // coplanar face detection: ss_e_ip/op/r, ts_e_p
    cross_joint             = 1,  // crossing elements: plane_to_face (type-30)
    face_to_face_then_cross = 2,  // face-to-face first, then cross-joint fallback
};

// ═══════════════════════════════════════════════════════════════════════════
// get_connection_zones — 9-stage wood joint detection pipeline.
//
// Stages: build WoodElements → BVH adjacency → face_to_face_wood detection
//         → three-valence linking → joint geometry creation → orientation
//         → merge into plate outlines → fill session.
//
// elements    — timber plates (bottom polygon + top polygon per plate).
//               Load from a dataset with internal::load_plates(name), or
//               construct directly for custom geometry.
// session     — caller-owned; plates, joint volumes, merged outlines, and
//               loft meshes are appended. Caller calls session.pb_dump(path)
//               to persist the result.
// search_type — face_to_face (default), cross_joint, or face_to_face_then_cross.
// Returns: merged plate outline polylines per element
//          [element_id][outline_id]: [hole0_top, hole0_bot, ..., outer_top, outer_bot]
//          Use for lofting each element after this call.
// ═══════════════════════════════════════════════════════════════════════════
std::vector<std::vector<session_cpp::Polyline>> get_connection_zones(
        const std::vector<session_cpp::ElementPlate>& elements,
        session_cpp::Session& session,
        SearchType search_type = face_to_face);

// ═══════════════════════════════════════════════════════════════════════════
// loft_merged_elements — build a loft Mesh per element from the merged-outline
// result of get_connection_zones() and append them as a "MergedMeshes" group
// to the session. merged layout per element:
//   [hole0_top, hole0_bot, ..., outer_top, outer_bot]
// Callers decide when (or whether) to produce loft meshes; get_connection_zones
// no longer does this implicitly.
// ═══════════════════════════════════════════════════════════════════════════
void loft_merged_elements(
        session_cpp::Session& session,
        const std::vector<std::vector<session_cpp::Polyline>>& mc);

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
