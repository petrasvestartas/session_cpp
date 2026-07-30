#pragma once

// P3 common-block subsystem — PARTIAL coincidence (OCCT port: BOPAlgo_Tools::PerformCommonBlocks
// + ComputeToleranceOfCB, BOPAlgo_Tools.cxx:107-356; data model BOPDS_CommonBlock/BOPDS_PaveBlock).
//
// WHY THIS EXISTS. Full coincidence can be handled by SUPPRESSION (drop the duplicate chain).
// Partial coincidence cannot: along part of a face a section legitimately must exist, so the
// shared boundary has to be SYNTHESISED as a common entity and the chain SPLIT at the boundary
// of the coincident region. Everything here is per-REGION; nothing accepts or rejects a whole
// chain. NEW module: reads BReps, never mutates kernel state.

#include "brep.h"
#include "brep_samedomain.h"
#include <vector>

namespace session_cpp {

/// Membership predicate used to decide whether a point belongs to an operand's shared region.
enum class CBOn {
    Face = 0,   ///< within band of the operand's TRIMMED boundary (2D-shared region test)
    Edge = 1    ///< within band of an existing EDGE of the operand (duplicate-chain test)
};

/// A pave on the source curve: a parameter where the coincidence state changes, or a range end.
/// OCCT's BOPDS_Pave analog; `kind` records provenance so an integrator can tell a synthesised
/// coincidence boundary from a pre-existing range end.
struct CBPave {
    double t = 0.0;
    int kind = 0;   ///< 0 = range end, 1 = coincidence transition
};

/// One member of a common block: an entity of an operand that the block runs along.
/// `dist` is the measured max distance over the block's samples (feeds ComputeToleranceOfCB).
struct CBMember {
    const BRep* brep = nullptr;
    int operand = 0;
    int face = -1;    ///< nearest trimmed face (CBOn::Face membership)
    int edge = -1;    ///< coincident existing edge, -1 when none (CBOn::Edge membership)
    double dist = 0.0;
    double tol = 0.0; ///< stored tolerance of the member entity (0 -- P5 gap, see cb_tolerance)
};

/// One interval of a chain with a CONSTANT coincidence state — the unit of per-region work.
/// Geometry is NEVER refitted: the block references the source curve plus its [t0,t1] range
/// (OCCT's "Range must be as it was" invariant).
struct CommonBlock {
    const NurbsCurve* curve = nullptr;   ///< source curve (not owned)
    double t0 = 0.0, t1 = 0.0;           ///< range of THIS block on the source curve
    bool coincident = false;             ///< shared by BOTH operands over the whole range
    int operands_mask = 0;               ///< bit0 = on A, bit1 = on B
    std::vector<CBMember> members;       ///< per-operand entities the block runs along
    std::vector<CBPave> paves;           ///< {t0, t1} with transition provenance
    double tolerance = 0.0;              ///< ComputeToleranceOfCB result
    int orient = 0;                      ///< 0 same / 1 opposite outward normals (coincident only)

    double length_param() const { return t1 - t0; }
    Point point_at_frac(double s) const;   ///< s in [0,1] over [t0,t1]
};

/// Result of splitting one chain against a pair of operands.
struct CBSplit {
    std::vector<CommonBlock> blocks;   ///< contiguous, ordered, covering [tmin,tmax]
    int n_coincident = 0;
    int n_transitions = 0;             ///< interior paves synthesised
};

/// Tuning for the splitting predicate.
struct CBParams {
    double band = 1e-6;      ///< coincidence band (tolA + tolB + fuzz at the call site)
    CBOn on = CBOn::Face;    ///< membership predicate
    int samples = 64;        ///< initial uniform samples along the chain
    int bisect = 40;         ///< bisection steps to localise each transition
    double micro_frac = 1e-4;///< intervals shorter than this fraction of the span are absorbed
};

///////////////////////////////////////////////////////////////////////////////////////////
// The splitting predicate (deliverable 2 — per-REGION granularity)
///////////////////////////////////////////////////////////////////////////////////////////

/// Is `p` within `band` of operand `b` under the given membership predicate?
/// `face`/`edge` receive the witness entity when the test succeeds.
bool cb_on_operand(const BRep& b, const Point& p, double band, CBOn on,
                   int* face = nullptr, int* edge = nullptr, double* dist = nullptr);

/// THE SPLITTING PREDICATE. Splits `curve` at every parameter where
/// `cb_on_operand(A) && cb_on_operand(B)` changes truth value, and returns the resulting
/// contiguous blocks. Transitions are localised by bisection, so a block boundary sits on the
/// true edge of the coincident region rather than on a sample. This is the per-parameter form
/// of a whole-chain accept/reject test: a chain that is coincident over only part of its span
/// yields >= 2 blocks instead of one verdict.
CBSplit cb_split_chain(const NurbsCurve& curve, const BRep& A, const BRep& B,
                       const CBParams& prm = {});

/// ComputeToleranceOfCB port (BOPAlgo_Tools.cxx:248-356): seed with the representative entity's
/// tolerance, then sample 11 interior points at dt = (t1-t0)/12 over the block's OWN range and
/// take max(tol, tol(mate) + distance) over every member edge and every attached face.
/// Returns the tolerance and stores it on the block.
double cb_tolerance(CommonBlock& blk, int samples = 11);

/// PerformCommonBlocks port (BOPAlgo_Tools.cxx:107-187): group coincident blocks that share a
/// member entity into connected components, merge their member/face lists (deduplicated, the
/// first member already carrying a group donates it), elect the min-index representative, and
/// recompute the tolerance of each group. Returns the group index per input block (-1 = its own
/// singleton). Groups of fewer than 2 blocks are skipped, exactly as OCCT does.
std::vector<int> cb_perform_common_blocks(std::vector<CommonBlock>& blocks);

///////////////////////////////////////////////////////////////////////////////////////////
// Region classification + op-table application (deliverable 3)
///////////////////////////////////////////////////////////////////////////////////////////

/// A sub-region of a face: the UV samples that belong to it plus its classified state.
struct CBRegion {
    const BRep* brep = nullptr;
    int operand = 0;
    int face = -1;
    std::vector<std::array<double, 2>> uvs;   ///< in-material UV samples of THIS region
    SDState state = SDState::Out;
    int orient = 0;                           ///< 0 same / 1 opposite (ON regions only)
};

/// Classify one region of a face against the other operand (areal rule over the region's own
/// samples). Fills `state` and, for ON regions, `orient` from the outward-normal relation.
SDState cb_classify_region(CBRegion& reg, const BRep& other, double tol,
                           const std::vector<double>& osign_S = {},
                           const std::vector<double>& osign_other = {});

/// Op-table applied to a REGION rather than a whole face. For an ON region this is exactly
/// sd_select_sd_face (the ON-same / ON-opposite rows) driven by the region's PAIR orientation;
/// for In/Out regions it is the ordinary sd_select_face row. The point of per-region granularity:
/// two regions of ONE face can now receive different verdicts, which is impossible whole-face.
SDVerdict cb_select_region(SDOp op, const CBRegion& reg);

/// Partition a face into coincident / non-coincident regions against `other` by classifying its
/// in-material samples individually, then grouping them by verdict. Returns one CBRegion per
/// distinct state present on the face. A face wholly In/Out/ON yields exactly one region; a
/// PARTIALLY coincident face yields two or more — the detection signal for partial coincidence.
std::vector<CBRegion> cb_partition_face(const BRep& S, int face, int operand, const BRep& other,
                                        double tol, int probes = 64,
                                        const std::vector<double>& osign_S = {},
                                        const std::vector<double>& osign_other = {});

} // namespace session_cpp
