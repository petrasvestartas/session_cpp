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
// tt_e_p_3: top-to-top drill-grid joint (type 40 variant 3, id 43).
//   Offsets joint_area inward, divides edges into points, emits 2-pt drill
//   lines along the element thickness. Port of wood_joint_lib.cpp:5651-5710.
#pragma once

#include "wood_element.h"

#include <vector>

namespace wood_session {

void apply_unit_scale(WoodJoint& joint);
void joint_orient_to_connection_area(WoodJoint& joint);
void merge_linked_joints(WoodJoint& joint, std::vector<WoodJoint>& all_joints);
void joint_get_divisions(WoodJoint& joint, double division_distance);

void side_removal_ss_e_r_1_port(WoodJoint& joint, const std::vector<WoodElement>& elements);
void tt_e_p_3(WoodJoint& joint, const std::vector<WoodElement>& elements);

} // namespace wood_session
