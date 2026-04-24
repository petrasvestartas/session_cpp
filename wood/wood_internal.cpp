// ═══════════════════════════════════════════════════════════════════════════
// wood/wood_internal.cpp — internal helpers for the wood pipeline.
// Declarations in wood_session.h.
// ═══════════════════════════════════════════════════════════════════════════
#include "wood_session.h"
#include "../src/file_obj.h"
#include "../src/polyline.h"
#include "../src/point.h"
#include "../src/element.h"

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

using namespace session_cpp;

namespace internal {

// Absolute path to `session_data/` at the repo root. Centralises the
// `__FILE__.parent_path() × 3` walk so relocating this translation unit
// stays a one-line change.
std::filesystem::path session_data_dir() {
    return std::filesystem::path(__FILE__)
        .parent_path()   // session_cpp/wood/
        .parent_path()   // session_cpp/
        .parent_path()   // session/
        / "session_data";
}

// Map wood test function name → session OBJ basename.
static std::string wood_name_to_obj_short(const std::string& name) {
    static const std::unordered_map<std::string, std::string> alias = {
        {"type_plates_name_joint_linking_vidychapel_corner",                 "vidy_corner"},
        {"type_plates_name_joint_linking_vidychapel_one_layer",              "vidy_one_layer"},
        {"type_plates_name_joint_linking_vidychapel_one_axis_two_layers",    "vidy_one_axis_two_layers"},
        {"type_plates_name_joint_linking_vidychapel_full",                   "vidy_full"},
        {"type_plates_name_side_to_side_edge_inplane_2_butterflies",         "inplane_butterflies"},
        {"type_plates_name_side_to_side_edge_inplane_hexshell",              "inplane_hexshell"},
        {"type_plates_name_side_to_side_edge_inplane_differentdirections",   "inplane_differentdirections"},
        {"type_plates_name_side_to_side_edge_inplane_hilti",                 "inplane_hilti"},
        {"type_plates_name_side_to_side_edge_outofplane_folding",            "vidy_folding"},
        {"type_plates_name_side_to_side_edge_outofplane_box",                "outofplane_box"},
        {"type_plates_name_side_to_side_edge_outofplane_tetra",              "outofplane_tetra"},
        {"type_plates_name_side_to_side_edge_outofplane_dodecahedron",       "outofplane_dodecahedron"},
        {"type_plates_name_side_to_side_edge_outofplane_icosahedron",        "outofplane_icosahedron"},
        {"type_plates_name_side_to_side_edge_outofplane_octahedron",         "outofplane_octahedron"},
        {"type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners",                    "simple_corners"},
        {"type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_combined",           "simple_corners_combined"},
        {"type_plates_name_side_to_side_edge_inplane_outofplane_simple_corners_different_lengths",  "simple_corners_diff_lengths"},
        {"type_plates_name_side_to_side_edge_outofplane_inplane_and_top_to_top_hexboxes",           "hexboxes"},
        {"type_plates_name_top_to_side_and_side_to_side_outofplane_annen_corner",                   "annen_corner"},
        {"type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box",                      "annen_box"},
        {"type_plates_name_top_to_side_and_side_to_side_outofplane_annen_box_pair",                 "annen_box_pair"},
        {"type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_small",               "annen_grid_small"},
        {"type_plates_name_top_to_side_and_side_to_side_outofplane_annen_grid_full_arch",           "annen_grid_full_arch"},
    };
    auto it = alias.find(name);
    if (it != alias.end()) return it->second;

    static const std::string plate_prefix = "type_plates_name_";
    static const std::string beam_prefix  = "type_beams_name_";
    if (name.rfind(plate_prefix, 0) == 0) return name.substr(plate_prefix.size());
    if (name.rfind(beam_prefix,  0) == 0) return name.substr(beam_prefix.size());
    return name;
}

bool plates_exist(const std::string& wood_name) {
    auto path = session_data_dir() / (wood_name_to_obj_short(wood_name) + ".obj");
    return std::filesystem::exists(path);
}

// Load dataset → ElementPlates. Sets DATA_SET_INPUT_NAME + DATA_SET_OUTPUT_FILE globals.
std::vector<ElementPlate> load_plates(const std::string& dataset_name, double duplicate_pts_tol) {
    wood_session::globals::DUPLICATE_PTS_TOL = duplicate_pts_tol;

    const std::string obj_short = wood_name_to_obj_short(dataset_name);
    const std::string obj_path  = (session_data_dir() / (obj_short + ".obj")).string();
    auto polylines = file_obj::read_file_obj_polylines(obj_path);

    if (duplicate_pts_tol > 0.0)
        for (auto& pl : polylines) pl.remove_consecutive_duplicates(duplicate_pts_tol);

    wood_session::globals::DATA_SET_INPUT_NAME  = obj_short;
    wood_session::globals::DATA_SET_OUTPUT_FILE = "WoodF2F_" + obj_short + ".pb";

    std::vector<ElementPlate> plates;
    plates.reserve(polylines.size() / 2);
    for (size_t i = 0; i + 1 < polylines.size(); i += 2)
        plates.emplace_back(polylines[i], polylines[i + 1], "plate_" + std::to_string(i / 2));
    return plates;
}

// Load raw polylines without pairing — for beam datasets where each polyline is an axis.
std::vector<Polyline> load_polylines(const std::string& dataset_name, double duplicate_pts_tol) {
    wood_session::globals::DUPLICATE_PTS_TOL = duplicate_pts_tol;

    const std::string obj_short = wood_name_to_obj_short(dataset_name);
    const std::string obj_path  = (session_data_dir() / (obj_short + ".obj")).string();
    auto polylines = file_obj::read_file_obj_polylines(obj_path);

    if (duplicate_pts_tol > 0.0)
        for (auto& pl : polylines) pl.remove_consecutive_duplicates(duplicate_pts_tol);

    wood_session::globals::DATA_SET_INPUT_NAME  = obj_short;
    wood_session::globals::DATA_SET_OUTPUT_FILE = "WoodF2F_" + obj_short + ".pb";
    return polylines;
}

} // namespace internal
