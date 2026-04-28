// ═══════════════════════════════════════════════════════════════════════════
// wood/wood_joint_lib.h — aggregator: one file per joint constructor.
//
// Each file in wood/joints/ is a plain header (no #pragma once, no #include
// of its own) designed to be included exactly once inside this file, which is
// itself included inside wood_main.cpp's anonymous namespace. All names
// needed by the joint constructors (Point, Vector, Polyline, WoodJoint,
// wood_cut::cut_type, Intersection, Xform) are already in scope at that site.
//
// Mirrors `cmake/src/wood/include/wood_joint_lib.cpp` from the wood project.
// The `joint_create_geometry` dispatcher lives in wood_main.cpp.
//
// Joint families:
//   group 0 (ss_e_ip, type 12): ss_e_ip_0..4
//   group 1 (ss_e_op, type 11): ss_e_op_0..5, ss_e_op_tutorial
//   group 2 (ts_e_p,  type 20): ts_e_p_0..3, ts_e_p_5
//   group 3 (cr_c_ip, type 30): cr_c_ip_0..5
//   group 5 (ss_e_r,  type 13): ss_e_r_0..3
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "wood_cut.h"
// NOTE: Joints reference `wood_session::globals::*` (e.g. CUSTOM_JOINTS_*).
// Do NOT include wood_session.h here — this aggregator is dropped into an
// anonymous namespace inside wood_joint.cpp, which would put the extern
// declarations in the wrong scope. Each consumer (wood_joint.cpp,
// wood_main.cpp) must include wood_session.h itself, before this file.

// joints/helpers.h was a thin wrapper around Point::interpolate(a, b, n, 0).
// Call sites now use Point::interpolate directly (no wrapper needed).

// MSVC C4505: each joint is `static` and only some are referenced by any one TU.
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4505)
#endif

// group 0 — ss_e_ip (side-side in-plane, type 12)
#include "joints/ss_e_ip_0.h"
#include "joints/ss_e_ip_1.h"
#include "joints/ss_e_ip_2.h"
#include "joints/ss_e_ip_3.h"
#include "joints/ss_e_ip_4.h"
#include "joints/ss_e_ip_custom.h"

// group 1 — ss_e_op (side-side out-of-plane, type 11)
// ss_e_op_4 must be included before ss_e_op_5 (5 calls 4).
#include "joints/ss_e_op_0.h"
#include "joints/ss_e_op_1.h"
#include "joints/ss_e_op_2.h"
#include "joints/ss_e_op_3.h"
#include "joints/ss_e_op_4.h"
#include "joints/ss_e_op_5.h"
#include "joints/ss_e_op_tutorial.h"

// group 2 — ts_e_p (top-side, type 20)
#include "joints/ts_e_p_0.h"
#include "joints/ts_e_p_1.h"
#include "joints/ts_e_p_2.h"
#include "joints/ts_e_p_3.h"
#include "joints/ts_e_p_5.h"

// group 3 — cr_c_ip (cross in-plane, type 30)
#include "joints/cr_c_ip_0.h"
#include "joints/cr_c_ip_1.h"
#include "joints/cr_c_ip_shared.h"
#include "joints/cr_c_ip_2.h"
#include "joints/cr_c_ip_3.h"
#include "joints/cr_c_ip_4.h"
#include "joints/cr_c_ip_5.h"

// group 5 — ss_e_r (side-side relief, type 13)
#include "joints/ss_e_r_0.h"
#include "joints/ss_e_r_impl.h"
#include "joints/ss_e_r_2.h"
#include "joints/ss_e_r_3.h"

#ifdef _MSC_VER
#pragma warning(pop)
#endif
