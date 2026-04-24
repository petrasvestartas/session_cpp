// ═══════════════════════════════════════════════════════════════════════════
// wood/wood_element.h — WoodJoint and WoodElement data structs.
//
// WoodJoint  mirrors the seven out-parameters of wood::main::face_to_face
//            plus all joinery-library fields consumed by joint_create_geometry
//            and joint_orient_to_connection_area in wood_main.cpp.
//
// WoodElement mirrors the plate model built by wood::main::get_elements:
//            polylines (all face outlines) and their planes.
//
// Implementations (constructors) in wood_element.cpp.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "../src/line.h"
#include "../src/plane.h"
#include "../src/point.h"
#include "../src/polyline.h"
#include "../src/vector.h"

namespace wood_session {

// ───────────────────────────────────────────────────────────────────────────
// WoodJoint — output record produced by face_to_face_wood, consumed by
// joint_create_geometry / joint_orient_to_connection_area /
// merge_joints_for_element. Mirrors wood's `joint` struct (wood_joint.h).
// ───────────────────────────────────────────────────────────────────────────
struct WoodJoint {
    WoodJoint();

    std::pair<int, int> el_ids;
    std::pair<std::array<int, 2>, std::array<int, 2>> face_ids;
    int joint_type;
    // Joint library name ("ss_e_ip_0", "ts_e_p_3", etc.) — mirrors wood's
    // `joint::name` at `wood_joint.h:50`. Set by each joint constructor in
    // `wood_joint_lib.h` via `joint.name = __func__`.
    std::string name;
    session_cpp::Polyline joint_area;
    std::array<session_cpp::Line, 2> joint_lines;
    std::array<std::optional<session_cpp::Polyline>, 4> joint_volumes_pair_a_pair_b;
    // Joinery library output
    std::array<std::vector<session_cpp::Polyline>, 2> m_outlines; // male [0]=top face, [1]=bottom face
    std::array<std::vector<session_cpp::Polyline>, 2> f_outlines; // female [0]=top, [1]=bottom
    // Per-polyline cut types (Stage 3). Each entry indexes 1:1 with
    // `m_outlines[face][i]` / `f_outlines[face][i]`. Empty vectors mean
    // "all entries are edge_insertion" (the legacy default before Stage 3
    // landed). See `wood_cut::cut_type` in wood_cut.h.
    std::array<std::vector<int>, 2> m_cut_types;
    std::array<std::vector<int>, 2> f_cut_types;
    int divisions;
    double shift;
    double length;
    double division_length; // wood's joint::division_length — the raw
                             // division_distance parameter passed to
                             // get_divisions; kept separate from the
                             // computed `length`. Used by tt_e_p_3.
    std::array<double, 3> scale;
    // Wood `joint::unit_scale` (`wood_joint.h:85`). When true,
    // `joint_orient_to_connection_area` rescales the joint volumes along
    // the Z axis (joint-line direction) so the unit-cube → world transform
    // doesn't stretch the joint geometry. Set to true by joint constructors
    // like `ss_e_op_2..6`, `ts_e_p_4..5`, `ss_e_r_3`, `ss_e_ip_5`. Annen
    // joints (`ss_e_op_0/1`, `ts_e_p_3`) leave it false → no rescale.
    bool unit_scale;
    double unit_scale_distance;
    // Joint linking (Vidy three-valence). The primary joint stores indices
    // of shadow joints it links to. `ss_e_op_5` uses these to generate
    // geometry for the linked joints simultaneously.
    std::vector<int> linked_joints;
    std::vector<std::vector<std::array<int, 4>>> linked_joints_seq;
    bool link;      // true for shadow joints created by three_valence_addition
    bool no_orient; // true for world-space joints (ss_e_r_0): skip orient step
    // Debug
    int dbg_coplanar;
    int dbg_boolean;
    std::string dbg_fail_reason;
};

// ───────────────────────────────────────────────────────────────────────────
// WoodElement — plate element: polylines and planes built exactly as
// wood::main::get_elements (wood_main.cpp:14-201) does it.
// ───────────────────────────────────────────────────────────────────────────
struct WoodElement {
    WoodElement();

    std::vector<session_cpp::Polyline> polylines;
    std::vector<session_cpp::Plane>    planes;
    std::vector<session_cpp::Vector>   insertion_vectors; // optional per-face assembly directions
    bool reversed;  // true if build_wood_element reversed the winding
    double thickness; // perpendicular distance between face planes 0 and 1
};

} // namespace wood_session
