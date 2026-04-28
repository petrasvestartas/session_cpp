// main_wood_04_all_datasets.cpp — run all 43 wood datasets in sequence.
//
// Equivalent to main_5.cpp. Each dataset writes WoodF2F_<name>.pb
// to session_data/ — datasets missing their OBJ are silently skipped.
//
// ── Rhino viewer (paste into Rhino 8 ScriptEditor, venv: session_py) ──────
//
//   #! python3
//   # venv: session_py
//
//   import importlib
//   import session_rhino.session
//   importlib.reload(session_rhino.session)
//   from session_rhino.session import Session
//
//   filepath = r"C:\brg\code_rust\session\session_data\WoodF2F_annen_corner.pb"
//   # swap filepath to view any other dataset produced by this binary
//
//   scene = Session.load(filepath)
//   scene.draw(delete=True)
//
// ─────────────────────────────────────────────────────────────────────────
#include "wood/wood_session.h"

int main() {
    type_plates_name_hexbox_and_corner();
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
