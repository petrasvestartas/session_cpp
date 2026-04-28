// ═══════════════════════════════════════════════════════════════════════════
// wood/wood_globals.cpp — definitions + YAML-driven loader for
// `wood_session::globals`.
//
// Every tunable mirrors a `wood::GLOBALS::*` field from the upstream wood
// project (C:\brg\code_cpp\wood\cmake\src\wood\include\wood_globals.{h,cpp}).
// Tests call `globals_yaml("type_plates_name_X")` instead of patching values
// in code, so users can re-run without rebuilding by editing the YAML.
// ═══════════════════════════════════════════════════════════════════════════
#include "wood_session.h"
#include "yaml.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>

namespace wood_session {
namespace globals {

// Joint algorithm tunables.
std::vector<double> JOINTS_PARAMETERS_AND_TYPES;
std::vector<double> JOINT_VOLUME_EXTENSION;
std::array<double, 3> JOINT_SCALE = { 1.0, 1.0, 1.0 };
int    OUTPUT_GEOMETRY_TYPE                              = 4;
double FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_DIHEDRAL_ANGLE   = 150.0;
bool   FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ALL_TREATED_AS_ROTATED   = false;
bool   FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ROTATED_JOINT_AS_AVERAGE = false;

// Tolerances.
double DISTANCE                                          = 0.1;   // wood_globals.cpp:14
double DISTANCE_SQUARED                                  = 0.01;  // wood_globals.cpp:15
double ANGLE                                             = 0.11;  // wood_globals.cpp:16, RADIANS
double DUPLICATE_PTS_TOL                                 = 0.0;
double LIMIT_MIN_JOINT_LENGTH                            = 0.0;   // wood_globals.cpp:36

// Clipper2 layer (informational — Clipper2 is no longer linked).
int64_t CLIPPER_SCALE                                    = 1000000; // wood_globals.cpp:10
double  CLIPPER_AREA                                     = 0.01;    // wood_globals.cpp:11

// Filesystem strings.
std::string DATA_SET_INPUT_NAME;
std::string DATA_SET_OUTPUT_FILE;
std::string DATA_SET_OUTPUT_DATABASE;
std::string PATH_AND_FILE_FOR_JOINTS;

// Misc upstream-parity globals.
std::vector<std::string> EXISTING_TYPES;
std::size_t              RUN_COUNT = 0;

// Custom joint polylines (runtime-populated; YAML loader skips them).
std::vector<session_cpp::Polyline> CUSTOM_JOINTS_SS_E_IP_MALE;
std::vector<session_cpp::Polyline> CUSTOM_JOINTS_SS_E_IP_FEMALE;
std::vector<session_cpp::Polyline> CUSTOM_JOINTS_SS_E_OP_MALE;
std::vector<session_cpp::Polyline> CUSTOM_JOINTS_SS_E_OP_FEMALE;
std::vector<session_cpp::Polyline> CUSTOM_JOINTS_TS_E_P_MALE;
std::vector<session_cpp::Polyline> CUSTOM_JOINTS_TS_E_P_FEMALE;
std::vector<session_cpp::Polyline> CUSTOM_JOINTS_CR_C_IP_MALE;
std::vector<session_cpp::Polyline> CUSTOM_JOINTS_CR_C_IP_FEMALE;
std::vector<session_cpp::Polyline> CUSTOM_JOINTS_TT_E_P_MALE;
std::vector<session_cpp::Polyline> CUSTOM_JOINTS_TT_E_P_FEMALE;
std::vector<session_cpp::Polyline> CUSTOM_JOINTS_SS_E_R_MALE;
std::vector<session_cpp::Polyline> CUSTOM_JOINTS_SS_E_R_FEMALE;
std::vector<session_cpp::Polyline> CUSTOM_JOINTS_B_MALE;
std::vector<session_cpp::Polyline> CUSTOM_JOINTS_B_FEMALE;

namespace {

std::filesystem::path config_dir() {
    return std::filesystem::path(__FILE__).parent_path() / "config";
}

bool parse_bool(const std::string& s) {
    return s == "true" || s == "True" || s == "TRUE" || s == "1" || s == "yes";
}

std::vector<double> parse_doubles(std::vector<std::string>& xs) {
    std::vector<double> out;
    out.reserve(xs.size());
    for (auto& x : xs) out.push_back(std::stod(x));
    return out;
}

} // namespace

void reset_defaults() {
    JOINTS_PARAMETERS_AND_TYPES = {
        300, 0.5,  3,
        450, 0.64, 15,
        450, 0.5,  20,
        300, 0.5,  30,
          6, 0.95, 40,
        300, 0.5,  58,
        300, 1.0,  60,
    };
    JOINT_VOLUME_EXTENSION = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    JOINT_SCALE = { 1.0, 1.0, 1.0 };
    OUTPUT_GEOMETRY_TYPE                              = 4;
    FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_DIHEDRAL_ANGLE   = 150.0;
    FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ALL_TREATED_AS_ROTATED   = false;
    FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ROTATED_JOINT_AS_AVERAGE = false;
    DISTANCE                                          = 0.1;
    DISTANCE_SQUARED                                  = 0.01;
    ANGLE                                             = 0.11;
    DUPLICATE_PTS_TOL                                 = 0.0;
    LIMIT_MIN_JOINT_LENGTH                            = 0.0;
    CLIPPER_SCALE                                     = 1000000;
    CLIPPER_AREA                                      = 0.01;
    DATA_SET_INPUT_NAME.clear();
    DATA_SET_OUTPUT_FILE.clear();
    DATA_SET_OUTPUT_DATABASE.clear();
    PATH_AND_FILE_FOR_JOINTS.clear();

    // EXISTING_TYPES — verbatim from wood_globals.cpp:38-47.
    EXISTING_TYPES = {
        "JOINT_NAMES[1] = ss_e_ip_0;",     "JOINT_NAMES[2] = ss_e_ip_1;",  "JOINT_NAMES[3] = ss_e_ip_2;",     "JOINT_NAMES[8] = side_removal;",
        "JOINT_NAMES[9] = ss_e_ip_9;",     "JOINT_NAMES[10] = ss_e_op_0;", "JOINT_NAMES[11] = ss_e_op_1;",    "JOINT_NAMES[12] = ss_e_op_2;",
        "JOINT_NAMES[13] = ss_e_op_3;",    "JOINT_NAMES[14] = ss_e_op_4;", "JOINT_NAMES[15] = ss_e_op_5;",    "JOINT_NAMES[18] = side_removal;",
        "JOINT_NAMES[19] = ss_e_op_9;",    "JOINT_NAMES[20] = ts_e_p_0;",  "JOINT_NAMES[21] = ts_e_p_1;",     "JOINT_NAMES[22] = ts_e_p_2;",
        "JOINT_NAMES[23] = ts_e_p_3;",     "JOINT_NAMES[24] = ts_e_p_4;",  "JOINT_NAMES[25] = ts_e_p_5;",     "JOINT_NAMES[28] = side_removal;",
        "JOINT_NAMES[29] = ts_e_p_9;",     "JOINT_NAMES[30] = cr_c_ip_0;", "JOINT_NAMES[31] = cr_c_ip_1;",    "JOINT_NAMES[32] = cr_c_ip_2;",
        "JOINT_NAMES[38] = side_removal;", "JOINT_NAMES[39] = cr_c_ip_9;", "JOINT_NAMES[48] = side_removal;", "JOINT_NAMES[58] = side_removal_ss_e_r_1;",
        "JOINT_NAMES[59] = ss_e_r_9;",     "JOINT_NAMES[60] = b_0;",
    };
    RUN_COUNT = 0;
    CUSTOM_JOINTS_SS_E_IP_MALE.clear();   CUSTOM_JOINTS_SS_E_IP_FEMALE.clear();
    CUSTOM_JOINTS_SS_E_OP_MALE.clear();   CUSTOM_JOINTS_SS_E_OP_FEMALE.clear();
    CUSTOM_JOINTS_TS_E_P_MALE.clear();    CUSTOM_JOINTS_TS_E_P_FEMALE.clear();
    CUSTOM_JOINTS_CR_C_IP_MALE.clear();   CUSTOM_JOINTS_CR_C_IP_FEMALE.clear();
    CUSTOM_JOINTS_TT_E_P_MALE.clear();    CUSTOM_JOINTS_TT_E_P_FEMALE.clear();
    CUSTOM_JOINTS_SS_E_R_MALE.clear();    CUSTOM_JOINTS_SS_E_R_FEMALE.clear();
    CUSTOM_JOINTS_B_MALE.clear();         CUSTOM_JOINTS_B_FEMALE.clear();
}

void globals_yaml(const std::string& dataset_name) {
    reset_defaults();

    auto path = config_dir() / (dataset_name + ".yml");
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error("globals_yaml: missing config " + path.string());
    }

    TINY_YAML::Yaml y(path.string());

    // Every read is gated by y.has(...). Missing keys keep the value set by
    // reset_defaults() above — TinyYaml's operator[] dereferences a null
    // shared_ptr on miss (UB), so we must NOT touch absent keys.
    auto str  = [&](const char* k) -> std::string& { return y[k].getData<std::string>(); };
    auto list = [&](const char* k) -> std::vector<std::string>& { return y[k].getData<std::vector<std::string>>(); };

    if (y.has("joints_parameters_and_types")) {
        auto& jpt = list("joints_parameters_and_types");
        if (!jpt.empty()) JOINTS_PARAMETERS_AND_TYPES = parse_doubles(jpt);
    }
    if (y.has("joint_volume_extension")) {
        auto& jve = list("joint_volume_extension");
        if (!jve.empty()) JOINT_VOLUME_EXTENSION = parse_doubles(jve);
    }
    if (y.has("joint_scale")) {
        auto& jsc = list("joint_scale");
        if (jsc.size() >= 3) {
            std::vector<double> s = parse_doubles(jsc);
            JOINT_SCALE = { s[0], s[1], s[2] };
        }
    }

    if (y.has("output_geometry_type"))                              OUTPUT_GEOMETRY_TYPE = std::stoi(str("output_geometry_type"));
    if (y.has("face_to_face_side_to_side_joints_dihedral_angle"))   FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_DIHEDRAL_ANGLE = std::stod(str("face_to_face_side_to_side_joints_dihedral_angle"));
    if (y.has("face_to_face_side_to_side_joints_all_treated_as_rotated"))   FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ALL_TREATED_AS_ROTATED = parse_bool(str("face_to_face_side_to_side_joints_all_treated_as_rotated"));
    if (y.has("face_to_face_side_to_side_joints_rotated_joint_as_average")) FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ROTATED_JOINT_AS_AVERAGE = parse_bool(str("face_to_face_side_to_side_joints_rotated_joint_as_average"));
    if (y.has("distance"))                                          DISTANCE = std::stod(str("distance"));
    if (y.has("distance_squared"))                                  DISTANCE_SQUARED = std::stod(str("distance_squared"));
    if (y.has("angle"))                                             ANGLE = std::stod(str("angle"));
    if (y.has("duplicate_pts_tol"))                                 DUPLICATE_PTS_TOL = std::stod(str("duplicate_pts_tol"));
    if (y.has("limit_min_joint_length"))                            LIMIT_MIN_JOINT_LENGTH = std::stod(str("limit_min_joint_length"));
    if (y.has("clipper_scale"))                                     CLIPPER_SCALE = std::stoll(str("clipper_scale"));
    if (y.has("clipper_area"))                                      CLIPPER_AREA = std::stod(str("clipper_area"));
    if (y.has("data_set_input_name"))                               DATA_SET_INPUT_NAME = str("data_set_input_name");
    if (y.has("data_set_output_file"))                              DATA_SET_OUTPUT_FILE = str("data_set_output_file");
    if (y.has("data_set_output_database"))                          DATA_SET_OUTPUT_DATABASE = str("data_set_output_database");
    if (y.has("path_and_file_for_joints"))                          PATH_AND_FILE_FOR_JOINTS = str("path_and_file_for_joints");
    if (y.has("run_count"))                                         RUN_COUNT = static_cast<std::size_t>(std::stoull(str("run_count")));

    if (y.has("existing_types")) {
        auto& et = list("existing_types");
        if (!et.empty()) EXISTING_TYPES = et;
    }

    // CUSTOM_JOINTS_* — nested polyline lists are beyond tiny-yaml's flat schema.
    // Populate at C++ runtime if needed.
}

} // namespace globals
} // namespace wood_session
