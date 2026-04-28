// main_5.cpp — entry point for the wood face-to-face port.
//
// All pipeline code lives in wood/wood_main.cpp; test harness in
// wood/wood_test.cpp; shared declarations in wood/wood_session.h.
// This file is just main() — a flat call list in wood_test.cpp order.
#include "wood/wood_session.h"

// ═══════════════════════════════════════════════════════════════════════════
// main() — flat call list matching wood_test.cpp:4684+ TEST() section order.
// The 43 test-function definitions live in wood/wood_test.cpp; declarations
// in wood/wood_session.h.
// ═══════════════════════════════════════════════════════════════════════════
int main() {
    type_plates_name_hexbox_and_corner();                                            // 204
    type_plates_name_joint_linking_vidychapel_corner();
    type_plates_name_joint_linking_vidychapel_one_layer();
    type_plates_name_joint_linking_vidychapel_one_axis_two_layers();
    type_plates_name_joint_linking_vidychapel_full();
    type_plates_name_side_to_side_edge_inplane_2_butterflies();
    type_plates_name_side_to_side_edge_inplane_hexshell();
    type_plates_name_side_to_side_edge_inplane_differentdirections();
    type_plates_name_side_to_side_edge_outofplane_folding();
    type_plates_name_side_to_side_edge_outofplane_box();
    type_plates_name_side_to_side_edge_outofplane_tetra();
    type_plates_name_side_to_side_edge_outofplane_dodecahedron();
    type_plates_name_side_to_side_edge_outofplane_icosahedron();
    type_plates_name_side_to_side_edge_outofplane_octahedron();
    type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners();
    type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_combined();
    type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_different_lengths();
    type_plates_name_side_to_side_edge_inplane_hilti();
    type_plates_name_top_to_top_pairs();
    type_plates_name_side_to_side_edge_outofplane_inplane_and_top_to_top_hexboxes();
    type_plates_name_hex_block_rossiniere();
    type_plates_name_top_to_side_snap_fit();
    type_plates_name_top_to_side_box();
    type_plates_name_top_to_side_corners();
    type_plates_name_top_to_side_and_side_to_side_outofplane_annen_corner();
    type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box();
    type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box_pair();
    type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_small();
    type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_full_arch();
    type_plates_name_vda_floor_0();
    type_plates_name_vda_floor_2();
    type_plates_name_cross_and_sides_corner();
    type_plates_name_cross_corners();
    type_plates_name_cross_vda_corner();
    type_plates_name_cross_vda_hexshell();
    type_plates_name_cross_vda_hexshell_reciprocal();
    type_plates_name_cross_vda_single_arch();
    type_plates_name_cross_vda_shell();
    type_plates_name_cross_square_reciprocal_two_sides();
    type_plates_name_cross_square_reciprocal_iseya();
    type_plates_name_cross_ibois_pavilion();
    type_plates_name_cross_brussels_sports_tower();
    type_beams_name_phanomema_node();
    return 0;
}
