// ═══════════════════════════════════════════════════════════════════════════
// wood/wood_internal.cpp — implementation of the `internal::` helpers from
// wood_test.cpp: OBJ path resolution, duplicate-point removal, and the
// `type_plates_name_*` → short-name alias table.
//
// Mirrors wood_test.cpp's anonymous `internal` namespace plus `wood_xml.cpp`
// bits that we inlined. Declarations in `wood_session.h`.
// ═══════════════════════════════════════════════════════════════════════════
#include "wood_session.h"
#include "../src/obj.h"
#include "../src/polyline.h"
#include "../src/point.h"

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
// Where session uses the same short name as wood, stripping the wood prefix
// (`type_plates_name_` / `type_beams_name_`) is sufficient; the explicit
// entries are only for datasets that were renamed on the session side (e.g.
// wood's vidychapel → session's vidy, wood's side_to_side_edge_inplane_2 →
// session's inplane_butterflies).
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

    // Fall back to prefix stripping for datasets whose session name is just
    // the wood name without the wood prefix.
    static const std::string plate_prefix = "type_plates_name_";
    static const std::string beam_prefix  = "type_beams_name_";
    if (name.rfind(plate_prefix, 0) == 0) return name.substr(plate_prefix.size());
    if (name.rfind(beam_prefix,  0) == 0) return name.substr(beam_prefix.size());
    return name;
}

bool obj_exists(const std::string& wood_name) {
    auto path = session_data_dir() / (wood_name_to_obj_short(wood_name) + ".obj");
    return std::filesystem::exists(path);
}

// Remove consecutive duplicate points from each polyline — wood runs the
// same cleanup inside wood_xml.cpp:135-138 when the caller requests it
// (currently only vidychapel_one_layer).
static void trim_consecutive_duplicates(std::vector<Polyline>& polys) {
    const double DIST_SQ = 0.01;
    for (auto& pl : polys) {
        auto pts = pl.get_points();
        std::vector<Point> cleaned;
        cleaned.reserve(pts.size());
        for (auto& p : pts) {
            if (cleaned.empty()) { cleaned.push_back(p); continue; }
            double dx = p[0] - cleaned.back()[0];
            double dy = p[1] - cleaned.back()[1];
            double dz = p[2] - cleaned.back()[2];
            if (dx*dx + dy*dy + dz*dz >= DIST_SQ) cleaned.push_back(p);
        }
        pl = Polyline(cleaned);
    }
}

void set_file_path_for_input_xml_and_screenshot(
        std::vector<std::pair<int,int>>& pairs_out,
        std::vector<Polyline>&           polylines_out,
        const std::string& name,
        bool remove_duplicate_pts) {
    wood_session::globals::REMOVE_DUPLICATE_PTS = remove_duplicate_pts;

    const std::string obj_short = wood_name_to_obj_short(name);
    const std::string obj_path  = (session_data_dir() / (obj_short + ".obj")).string();
    polylines_out = obj::read_obj_polylines(obj_path);

    if (remove_duplicate_pts) trim_consecutive_duplicates(polylines_out);

    pairs_out.clear();
    for (size_t i = 0; i + 1 < polylines_out.size(); i += 2)
        pairs_out.emplace_back((int)i, (int)(i + 1));

    wood_session::globals::DATA_SET_INPUT_NAME  = obj_short;
    wood_session::globals::DATA_SET_OUTPUT_FILE = "WoodF2F_" + obj_short + ".pb";
}

} // namespace internal
