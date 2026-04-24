// ═══════════════════════════════════════════════════════════════════════════
// wood/wood_globals.cpp — definitions for `wood_session::globals`.
//
// Mirrors `wood_globals.cpp` from the wood project: baseline values for
// `JOINTS_PARAMETERS_AND_TYPES`, `JOINT_VOLUME_EXTENSION`, and the other
// pipeline tunables that the test wrappers override per dataset.
// Declarations live in `wood_session.h`.
// ═══════════════════════════════════════════════════════════════════════════
#include "wood_session.h"

namespace wood_session {
namespace globals {

std::vector<double> JOINTS_PARAMETERS_AND_TYPES;
std::vector<double> JOINT_VOLUME_EXTENSION;
int    OUTPUT_GEOMETRY_TYPE                              = 4;
double FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_DIHEDRAL_ANGLE   = 150.0;
bool   FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ALL_TREATED_AS_ROTATED   = false;
bool   FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ROTATED_JOINT_AS_AVERAGE = false;
double DISTANCE_SQUARED                                  = 0.01;
std::string DATA_SET_OUTPUT_FILE;
std::string DATA_SET_INPUT_NAME;
double DUPLICATE_PTS_TOL                                 = 0.0;

// Defaults copied verbatim from wood_globals.cpp:25-67.
//
// The `JOINTS_PARAMETERS_AND_TYPES` table has 7 rows × 3 columns:
//   row 0 (ss_e_ip, type 12):   {300, 0.5,  3}
//   row 1 (ss_e_op, type 11):   {450, 0.64, 15}
//   row 2 (ts_e_p,  type 20):   {450, 0.5,  20}
//   row 3 (cr_c_ip, type 30):   {300, 0.5,  30}
//   row 4 (tt_e_p,  type 40):   {  6, 0.95, 40}
//   row 5 (ss_e_r,  type 13):   {300, 0.5,  58}
//   row 6 (b,       type 60):   {300, 1.0,  60}
// Columns are {division_length, shift, joint_type_id}. The id column is the
// wood-default constructor variant for the joint type.
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
    OUTPUT_GEOMETRY_TYPE                              = 4;
    FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_DIHEDRAL_ANGLE   = 150.0;
    FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ALL_TREATED_AS_ROTATED   = false;
    FACE_TO_FACE_SIDE_TO_SIDE_JOINTS_ROTATED_JOINT_AS_AVERAGE = false;
    DISTANCE_SQUARED                                  = 0.01;
    DATA_SET_OUTPUT_FILE.clear();
    DATA_SET_INPUT_NAME.clear();
    DUPLICATE_PTS_TOL                                 = 0.0;
}

} // namespace globals
} // namespace wood_session
