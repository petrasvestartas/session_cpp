// wood/wood_face_to_face.h — plate face-to-face joint detection.
//
// Stage 4 of 9 (face_to_face_wood): core joint topology detector. Classifies
//   one face-pair from two WoodElements as joint type 11/12/13/20/30/40 and
//   computes joint area, alignment lines, and volume rectangles.
//
// (Stage 1 — building a WoodElement from a bottom/top polyline pair — is now
//  a WoodElement constructor in wood_element.h.)
//
// Geometry helpers that moved to the session kernel:
//   polyline_two_rects_from_frame   → src/polyline.h
//   Intersection::line_line_classified → src/intersection.h
#pragma once

#include "wood_element.h"
#include "../src/point.h"
#include "../src/line.h"
#include "../src/polyline.h"
#include "../src/vector.h"

#include <utility>
#include <vector>

// Stage 4 of 9: Classify one face-pair from two elements as a wood joint.
// Returns true if a valid joint was found; populates out_joint and out_swap_planes_1.
// All parameters that the original wood reads from GLOBALS are explicit here so
// the function is pure and re-entrant.
bool face_to_face_wood(
    size_t joint_id,
    const wood_session::WoodElement& el0,
    const wood_session::WoodElement& el1,
    std::pair<int, int> el_ids_in,
    const std::vector<double>& joint_volume_extension,
    double limit_min_joint_length,
    double distance_squared,
    double coplanar_tolerance,
    double dihedral_angle_threshold,
    bool all_treated_as_rotated,
    bool rotated_joint_as_average,
    int  search_type,
    wood_session::WoodJoint& out_joint,
    bool& out_swap_planes_1);
