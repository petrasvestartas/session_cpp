// wood/wood_merge.h — merge joint cut outlines into element plate polylines.
//
// merge_joints_for_element: stitches oriented joint cut outlines (from
//   joint_create_geometry / joint_orient_to_connection_area) into the
//   element's top and bottom plate polylines. Port of
//   wood_element.cpp:654-1424 (merge_joints).
//
// Output vector layout: [hole0_top, hole0_bot, ..., merged_top, merged_bot]
//   — holes (Phase A/B/C) come first, the outer boundary pair last.
#pragma once

#include "wood_element.h"

#include <utility>
#include <vector>

using JMF = std::vector<std::vector<std::vector<std::pair<int,bool>>>>;

std::vector<session_cpp::Polyline> merge_joints_for_element(
    const wood_session::WoodElement& el,
    const std::vector<std::vector<std::pair<int,bool>>>& el_jmf,
    std::vector<wood_session::WoodJoint>& joints,
    int ei);
