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
// ElementPlate is still used by the cross-joint convenience overload below.
#include "../src/element.h"
// WoodElement / WoodJoint are passed by value/ref through this API surface.
#include "wood_element.h"

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
    // ── Joint algorithm tunables (pipeline reads these every run) ─────────
    extern std::vector<double> JOINTS_PARAMETERS_AND_TYPES;  ///< 7×3 family table: (division_length, shift, joint_type_id)
    extern std::vector<double> JOINT_VOLUME_EXTENSION;       ///< (width, height, length) ADDITIVE extension in mm, 5-entry default; +/- on volume edges
    extern std::array<double, 3> JOINT_SCALE;                ///< (sx, sy, sz) MULTIPLICATIVE scale fed into WoodJoint::scale; 1.0 = no change. Used by ss_e_ip_2, ss_e_r_*, ts_e_p_5.
    extern int    OUTPUT_GEOMETRY_TYPE;                      ///< 4 = merged outlines + lofts
    extern double FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_DIHEDRAL_ANGLE;       ///< degrees; rotated-joint threshold
    extern bool   FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ALL_TREATED_AS_ROTATED;///< force rotated geometry path
    extern bool   FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ROTATED_JOINT_AS_AVERAGE;///< averaged plane for rotated joints

    // ── Tolerances (heavy use across the kernel) ──────────────────────────
    extern double DISTANCE;                                  ///< inflate AABBs / point-merge tolerance (mm)
    extern double DISTANCE_SQUARED;                          ///< squared coplanarity tolerance (mm²)
    extern double ANGLE;                                     ///< angular tolerance, RADIANS (cos-tolerance)
    extern double DUPLICATE_PTS_TOL;                         ///< consecutive-duplicate-points removal in load_plates
    extern double LIMIT_MIN_JOINT_LENGTH;                    ///< filters out joints whose centerline is shorter

    // ── Clipper2 layer (unused in session port — Clipper2 was dropped) ───
    extern int64_t CLIPPER_SCALE;                            ///< upstream Clipper2 integer scale; informational
    extern double  CLIPPER_AREA;                             ///< upstream Clipper2 minimum polygon area; informational

    // ── Filesystem strings ────────────────────────────────────────────────
    extern std::string DATA_SET_INPUT_NAME;                  ///< short obj basename (set by load_plates)
    extern std::string DATA_SET_OUTPUT_FILE;                 ///< output .pb filename, written into session_data/
    extern std::string DATA_SET_OUTPUT_DATABASE;             ///< sqlite output path; informational, unused
    extern std::string PATH_AND_FILE_FOR_JOINTS;             ///< wood custom-joint-config file path; informational

    // ── Misc upstream-parity globals ──────────────────────────────────────
    extern std::vector<std::string> EXISTING_TYPES;          ///< upstream display table of joint variant names
    extern std::size_t RUN_COUNT;                            ///< upstream IMGUI loop counter; informational

    // ── Custom joint polylines (set at C++ runtime; YAML loader skips these) ─
    // Pairs (i, i+1) = (male, female) for one variant. Empty by default.
    // Wood's `wood_joint_lib.cpp` reads these to override the per-family
    // unit-cube geometry. No session-port consumer wired yet.
    extern std::vector<session_cpp::Polyline> CUSTOM_JOINTS_SS_E_IP_MALE;
    extern std::vector<session_cpp::Polyline> CUSTOM_JOINTS_SS_E_IP_FEMALE;
    extern std::vector<session_cpp::Polyline> CUSTOM_JOINTS_SS_E_OP_MALE;
    extern std::vector<session_cpp::Polyline> CUSTOM_JOINTS_SS_E_OP_FEMALE;
    extern std::vector<session_cpp::Polyline> CUSTOM_JOINTS_TS_E_P_MALE;
    extern std::vector<session_cpp::Polyline> CUSTOM_JOINTS_TS_E_P_FEMALE;
    extern std::vector<session_cpp::Polyline> CUSTOM_JOINTS_CR_C_IP_MALE;
    extern std::vector<session_cpp::Polyline> CUSTOM_JOINTS_CR_C_IP_FEMALE;
    extern std::vector<session_cpp::Polyline> CUSTOM_JOINTS_TT_E_P_MALE;
    extern std::vector<session_cpp::Polyline> CUSTOM_JOINTS_TT_E_P_FEMALE;
    extern std::vector<session_cpp::Polyline> CUSTOM_JOINTS_SS_E_R_MALE;
    extern std::vector<session_cpp::Polyline> CUSTOM_JOINTS_SS_E_R_FEMALE;
    extern std::vector<session_cpp::Polyline> CUSTOM_JOINTS_B_MALE;
    extern std::vector<session_cpp::Polyline> CUSTOM_JOINTS_B_FEMALE;

    // Reset every global above to wood baseline values. Used by the tutorial
    // mains (main_wood_01/02/03) that build geometry from scratch instead of
    // loading a named dataset. Test wrappers should prefer `globals_yaml(name)`.
    void reset_defaults();

    // Loads `wood/config/<dataset_name>.yml` and applies all keys to the globals
    // above. Replaces the old `reset_defaults()` + per-dataset hardcoded
    // overrides — users edit the YAML to retune a run without rebuilding.
    // Throws std::runtime_error if the YAML file is missing.
    void globals_yaml(const std::string& dataset_name);
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

// Load a named wood dataset from session_data/ and return one WoodElement per
// timber plate (planes, sides, thickness ready).
// Consecutive polylines (even index = bottom, odd = top) are paired.
// Also sets globals DATA_SET_INPUT_NAME and DATA_SET_OUTPUT_FILE as side effects.
// dataset_name — full wood test function name, e.g. "type_plates_name_hexbox_and_corner"
// duplicate_pts_tol — if > 0, removes consecutive duplicate points (vidychapel datasets)
std::vector<wood_session::WoodElement> load_plates(
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
// Stages: BVH adjacency → face_to_face_wood detection → three-valence linking
//         → joint geometry creation → orientation → merge into plate outlines.
//
// elements    — timber plates as WoodElements (built via the WoodElement
//               (bot, top) ctor, or returned by internal::load_plates(name)).
//               MUTATED: each
//               element's `features` field is populated with the merged
//               top/bottom outlines produced by the merge pass.
// search_type — face_to_face (default), cross_joint, or face_to_face_then_cross.
// Returns:    — every detected joint (per-pair), with type / area / lines /
//               volumes / male+female cut outlines populated.
//
// To visualize the result, build a Session and call fill_session(session,
// elements, joints, /*include_loft=*/true) — see below.
// ═══════════════════════════════════════════════════════════════════════════
std::vector<wood_session::WoodJoint> get_connection_zones(
        std::vector<wood_session::WoodElement>& elements,
        SearchType search_type = face_to_face);

// ═══════════════════════════════════════════════════════════════════════════
// fill_session — splat the result of get_connection_zones into a Session for
// visualization / .pb persistence. Recreates the legacy group layout:
//   "Elements"                               — input plates as ElementPlate
//   "JointAreas_SS_11" / "_TS_20" / "_Other" — per-type joint area polygons
//   "JointLines_SS_11" / "_TS_20" / "_Other" — per-type joint centerlines
//   "JointVols_SS_11"  / "_TS_20" / "_Other" — per-type joint volume quads
//   "element_<i>"                            — per-element merged outlines + cut polylines
//   "MergedMeshes"                           — loft of features.top/bottom per element
//                                              (only if include_loft = true)
// ═══════════════════════════════════════════════════════════════════════════
void fill_session(
        session_cpp::Session& session,
        const std::vector<wood_session::WoodElement>& elements,
        const std::vector<wood_session::WoodJoint>&   joints,
        bool include_loft = true);

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
