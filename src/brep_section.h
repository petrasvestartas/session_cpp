#pragma once

#include "brep.h"
#include "point.h"
#include <vector>

namespace session_cpp {

// One section interval shared by both operands: an index-corresponded chain where
// p3[i] lies on BOTH surfaces and uvA[i]/uvB[i] are its exact parametric footprints.
// This is the OCCT BOPDS_Curve/pave-block analog: the 3D polyline IS the one section
// edge geometry both operands' faces will reference.
struct SectionSegment {
    int seg_id = -1;
    std::vector<Point> p3;    // shared 3D polyline (the ONE section edge geometry)
    std::vector<Point> uvA;   // (u,v,0) on A's surface, index-matched to p3
    std::vector<Point> uvB;   // (u,v,0) on B's surface, index-matched to p3
    int v_start = -1;         // scaffold vertex ids (v_start == v_end for closed)
    int v_end = -1;
    int surfA = -1;           // operand surface indices (pair provenance)
    int surfB = -1;
    bool closed = false;
};

struct SectionScaffold {
    std::vector<Point> vertices;                  // pave vertices, 3D-welded across ALL pairs
    std::vector<SectionSegment> segments;
    std::vector<std::vector<int>> segs_by_surfA;  // per A-surface index -> seg ids
    std::vector<std::vector<int>> segs_by_surfB;  // per B-surface index -> seg ids
    double tol3 = 0.0;                            // 3D weld / micro tolerance used
    // Self-diagnostics ([SCAF] gate): chain fidelity + pave census.
    int n_chains = 0;
    int n_paves_trimA = 0;
    int n_paves_trimB = 0;
    int n_paves_xing = 0;
    int n_paves_vertex = 0;
    int n_paves_closing = 0;
    int n_dropped_verdict = 0;
    int n_dropped_micro = 0;
    double max_devA = 0.0;                        // max |A(uvA[i]) - p3[i]| over all chains
    double max_devB = 0.0;                        // max |B(uvB[i]) - p3[i]|
};

// Build the shared section scaffold for an imported-freeform boolean pair (OCCT
// PaveFiller MakeBlocks analog). Runs ONE SSI per overlapping surface pair (with the
// swapped-order retry), canonicalizes every section triple into a Newton-corrected
// (3D, uvA, uvB) chain, paves it (trim-loop crossings on BOTH operands, chain-chain
// crossings, existing-vertex projections, closing paves), applies ONE shared
// keep-verdict per interval (midpoint inside A's trims AND B's trims), drops micro
// intervals symmetrically, and welds pave vertices in 3D across all pairs.
SectionScaffold build_section_scaffold(const BRep& A, const BRep& B, double tolerance);

} // namespace session_cpp
