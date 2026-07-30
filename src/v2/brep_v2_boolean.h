#pragma once

// V2 BOOLEAN DRIVER — the piece that makes v2 an actual boolean rather than a set of
// components. Runs the pipeline in the fixed OCCT order and returns a BRep.
//
//   0  arena init                       (operands + their outward orientation, derived by
//                                        shell connexity, not by a weighted vote)
//   1  VV / VE / EE                     (v2int when its header and ours agree; see the .cpp)
//   2  VF / EF                          (idem)
//   3  FF section                       (v2sec when available; else ONE SSI per surface pair,
//                                        its pcurves fed to BOTH operands -> symmetric imprint)
//   4  PostTreat / same-domain          (brep_samedomain: coincident surface pairs are routed
//                                        to the SD path and NEVER to the section marcher)
//   5  split faces                      (v2sf when available; else the kernel's tested splitter
//                                        driven by the stage-3 pcurves)
//   6  classify ONCE  -> InParts        (guarantee I-10: selection performs no classification)
//   7  select                           (BuildBOP op-table, two fences — brep_v2_solid)
//   8  assemble                         (shells by shared-edge connexity, growth/hole, nesting,
//                                        solids — brep_v2_solid)
//
// The whole path is gated behind SESSION_V2: nothing in the existing kernel changes and
// BRep::boolean is untouched.
//
// NEW MODULE. Reads BReps, never mutates kernel state. namespace session_cpp::v2sol.

#include "brep.h"
#include "brep_v2_solid.h"
#include <string>
#include <vector>

namespace session_cpp {
namespace v2sol {

/// True when SESSION_V2 is set. The kernel's own boolean must keep working unchanged when it
/// is not; nothing in brep.cpp consults this yet.
bool v2_enabled();

struct V2BooleanOptions {
    double tolerance = 0.0;         ///< 0 -> derived from the operands' size
    bool verbose = false;           ///< also enabled by SESSION_V2_DBG
    bool keep_open_shells = false;  ///< G13: report only, never silently accept
};

/// Per-stage census; every give-up path is visible here (guarantee I-13).
struct V2BooleanReport {
    int n_surface_pairs = 0;
    int n_sd_surface_pairs = 0;   ///< coincident pairs routed away from the marcher
    int n_section_curves = 0;
    int n_faces_a = 0, n_faces_b = 0;      ///< face images after the split
    int n_in_a = 0, n_in_b = 0;            ///< faces classified IN the other operand
    int n_on = 0;                          ///< face images coincident with the other operand
    int n_sd_pairs = 0;                    ///< SD pairs fused to ONE arena face index
    int n_selected = 0;
    int n_shells = 0, n_solids = 0, n_holes = 0, n_open_shells = 0;
    int n_unused = 0;
    int fuzzy_steps = 0;   ///< >0: the tolerant closure fallback fired (approximate by design)
    std::vector<V2AlertRec> alerts;
    V2Verdict verdict;
    bool empty_result = false;
    std::string stage_fail;   ///< empty when every stage completed
    std::string str() const;
};

/// THE DRIVER. Returns the result BRep (possibly containing several solids — a multi-solid
/// result is CORRECT and is never collapsed). An empty BRep (0 faces) means "void result".
BRep v2_boolean(const BRep& A, const BRep& B, V2Op op, const V2BooleanOptions& opt = {},
                V2BooleanReport* report = nullptr);

/// Convenience wrappers matching BRep::boolean's vocabulary.
BRep v2_cut(const BRep& A, const BRep& B, double tol = 0.0, V2BooleanReport* r = nullptr);
BRep v2_common(const BRep& A, const BRep& B, double tol = 0.0, V2BooleanReport* r = nullptr);
BRep v2_fuse(const BRep& A, const BRep& B, double tol = 0.0, V2BooleanReport* r = nullptr);
BRep v2_cut21(const BRep& A, const BRep& B, double tol = 0.0, V2BooleanReport* r = nullptr);

/// Outward orientation of every face of a CLOSED operand, derived the OCCT way: shells by
/// shared-edge connexity, relative orientation by OrientFacesOnShell, global sense by the
/// per-shell growth/hole classification. +1 when the natural (Su x Sv) normal points OUT.
/// Replaces the weighted vote of BRep::face_outward_signs inside the v2 path only.
std::vector<double> v2_outward_signs(const BRep& b, int* shells = nullptr, int* holes = nullptr);

}  // namespace v2sol
}  // namespace session_cpp
