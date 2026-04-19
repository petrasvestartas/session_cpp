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
    type_plates_name_joint_linking_vidychapel_corner();                              // 265
    type_plates_name_joint_linking_vidychapel_one_layer();                           // 428
    type_plates_name_joint_linking_vidychapel_one_axis_two_layers();                 // 488
    type_plates_name_joint_linking_vidychapel_full();                                // 611
    type_plates_name_side_to_side_edge_inplane_2_butterflies();                      // 888
    type_plates_name_side_to_side_edge_inplane_hexshell();                           // 940
    type_plates_name_side_to_side_edge_inplane_differentdirections();                // 998
    type_plates_name_side_to_side_edge_outofplane_folding();                         // 1129
    type_plates_name_side_to_side_edge_outofplane_box();                             // 1384
    type_plates_name_side_to_side_edge_outofplane_tetra();                           // 1440
    type_plates_name_side_to_side_edge_outofplane_dodecahedron();                    // 1497
    type_plates_name_side_to_side_edge_outofplane_icosahedron();                     // 1555
    type_plates_name_side_to_side_edge_outofplane_octahedron();                      // 1613
    type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners();          // 1671
    type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_combined(); // 1729
    type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_different_lengths(); // 1787
    type_plates_name_side_to_side_edge_inplane_hilti();                              // 1849
    type_plates_name_top_to_top_pairs();                                             // 1912
    type_plates_name_side_to_side_edge_outofplane_inplane_and_top_to_top_hexboxes(); // 1965
    type_plates_name_hex_block_rossiniere();                                         // 2037
    type_plates_name_top_to_side_snap_fit();                                         // 2104
    type_plates_name_top_to_side_box();                                              // 2163
    type_plates_name_top_to_side_corners();                                          // 2220
    type_plates_name_top_to_side_and_side_to_side_outofplane_annen_corner();         // 2279
    type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box();            // 2391
    type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box_pair();       // 2488
    type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_small();     // 2698
    type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_full_arch(); // 2763
    type_plates_name_vda_floor_0();                                                  // 2829
    type_plates_name_vda_floor_2();                                                  // 2888
    type_plates_name_cross_and_sides_corner();                                       // 2955
    type_plates_name_cross_corners();                                                // 3016
    type_plates_name_cross_vda_corner();                                             // 3080
    type_plates_name_cross_vda_hexshell();                                           // 3144
    type_plates_name_cross_vda_hexshell_reciprocal();                                // 3207
    type_plates_name_cross_vda_single_arch();                                        // 3270
    type_plates_name_cross_vda_shell();                                              // 3333
    type_plates_name_cross_square_reciprocal_two_sides();                            // 3396
    type_plates_name_cross_square_reciprocal_iseya();                                // 3459
    type_plates_name_cross_ibois_pavilion();                                         // 3522
    type_plates_name_cross_brussels_sports_tower();                                  // 3588
    type_beams_name_phanomema_node();                                                // 3670
    return 0;
}
