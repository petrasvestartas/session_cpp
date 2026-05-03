// wood/wood_joint.h — joint orientation, linking, and element-aware constructors.
//
// Functions that need WoodElement geometry (plate planes/polylines) to build
// their cut outlines. These cannot live in wood_joint_lib.h (which is a
// type-only drop-in) and were previously inlined into wood_main.cpp.
//
// side_removal_ss_e_r_1_port: side-removal joint (type 13 / ss_e_r, id 58).
//   Extends adjacent-plate side-face rectangles by convex-corner test,
//   offsets them by plate normals, and emits mill_project cut outlines.
//   Port of wood_joint_lib.cpp:432 (simple side_removal variant).
//
// side_removal: general side-removal joint (cases 8, 28, 38, 57).
//   Full port of wood_joint_lib.cpp:432-631. Accesses element planes and
//   polylines to build mill_project cut outlines.
//
// tt_e_p_0..5: top-to-top drill joints (type 40, ids 40-42, 44-45).
//   Port of wood_joint_lib.cpp:5513-5834. Emit drill lines at points derived
//   from the joint_area geometry (centroid, boundary division, or grid).
//   All require element thickness for drill direction vectors.
#pragma once

#include "wood_element.h"

#include <vector>

namespace wood_session {

void apply_unit_scale(WoodJoint& joint);
void joint_orient_to_connection_area(WoodJoint& joint);
void merge_linked_joints(WoodJoint& joint, std::vector<WoodJoint>& all_joints);
void joint_get_divisions(WoodJoint& joint, double division_distance);

void side_removal_ss_e_r_1_port(WoodJoint& joint, const std::vector<WoodElement>& elements);
void side_removal(WoodJoint& joint, const std::vector<WoodElement>& elements, bool merge_with_joint = false);

void tt_e_p_0(WoodJoint& joint, const std::vector<WoodElement>& elements);
void tt_e_p_1(WoodJoint& joint, const std::vector<WoodElement>& elements);
void tt_e_p_2(WoodJoint& joint, const std::vector<WoodElement>& elements);
void tt_e_p_3(WoodJoint& joint, const std::vector<WoodElement>& elements);
void tt_e_p_4(WoodJoint& joint, const std::vector<WoodElement>& elements);
void tt_e_p_5(WoodJoint& joint, const std::vector<WoodElement>& elements);

} // namespace wood_session
