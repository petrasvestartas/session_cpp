#pragma once

#include "brep.h"
#include "point.h"
#include <vector>
#include <map>

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
    double tol3_rep = 0.0;                        // representational tolerance (capture/
                                                  // micro/tube bands); decoupled from the
                                                  // junction weld radius (SESSION_REP_MULT)
    // Self-diagnostics ([SCAF] gate): chain fidelity + pave census.
    int n_chains = 0;
    int n_paves_trimA = 0;
    int n_paves_trimB = 0;
    int n_paves_xing = 0;
    int n_paves_vertex = 0;
    int n_paves_closing = 0;
    int n_dropped_verdict = 0;
    int n_dropped_micro = 0;
    int n_bridge_marched = 0;                     // valence-1 bridge: same-pair SSI re-marches appended
    int n_bridge_welded = 0;                      // valence-1 bridge: cross-pair junctions fused
    int n_bridge_residual = 0;                    // valence-1 vertices still dangling after the bridge
    double max_devA = 0.0;                        // max |A(uvA[i]) - p3[i]| over all chains
    double max_devB = 0.0;                        // max |B(uvB[i]) - p3[i]|
};

// BOP2 (SESSION_BOP2) — SHARED-EDGE POOL: the OCCT BOPDS_PaveBlock analog. Each section
// pave-block (a SectionSegment) is minted ONCE as a single topological edge whose 3D curve
// is the shared chain and whose endpoints are the scaffold's 3D-welded pave vertices, so both
// operands' face arrangements REFERENCE the same edge (2-trim by construction) instead of
// minting private copies that must be aliased/sewn afterward. `arena` owns only the shared
// vertices + section edges; the operand splits attach their own pcurves + trims to these edges.
struct SharedEdgePool {
    BRep arena;                                   // owns m_vertices/m_topology_vertices/m_curves_3d/m_topology_edges
    std::vector<int> vert_tv;                     // scaffold vertex id -> arena topology-vertex idx
    std::vector<int> seg_edge;                    // seg_id -> arena edge idx (block 0 / whole seg)
    std::map<std::pair<int,int>, int> block_edge; // (seg_id, block) -> arena edge idx
};

// Build the shared-edge pool from a finished scaffold (M1). One edge per segment, endpoints =
// scaffold welded pave vertices -> both operands land on the SAME topology vertex by construction.
SharedEdgePool build_shared_edge_pool(const SectionScaffold& scaf);

// Build the shared section scaffold for an imported-freeform boolean pair (OCCT
// PaveFiller MakeBlocks analog). Runs ONE SSI per overlapping surface pair (with the
// swapped-order retry), canonicalizes every section triple into a Newton-corrected
// (3D, uvA, uvB) chain, paves it (trim-loop crossings on BOTH operands, chain-chain
// crossings, existing-vertex projections, closing paves), applies ONE shared
// keep-verdict per interval (midpoint inside A's trims AND B's trims), drops micro
// intervals symmetrically, and welds pave vertices in 3D across all pairs.
SectionScaffold build_section_scaffold(const BRep& A, const BRep& B, double tolerance);

// PRESERVE-IDENTITY REFINEMENT (OCCT's "re-compare after splits" / PostTreatFF step).
// A face's UV arrangement can discover a crossing INTERIOR to a section segment -- a place
// the scaffold did not pave. The operand then emits a partial run there, and because the
// two operands' arrangements never agree on clip parameters, their partial copies carry
// different endpoints and no shared identity: the defect that Hausdorff sewing cannot fix.
// The cure is to make the SCAFFOLD authoritative: feed every discovered breakpoint back,
// split the shared segment there (both operands' footprints split at the SAME chain index,
// and the new endpoint becomes ONE welded scaffold vertex), and re-run the splits. After
// refinement every run spans a whole segment, so each section edge is keyed by segment
// identity and the operands' copies are the same edge by construction.
//
// breaks: seg_id -> fractional chain indices reported by either operand. Splits only open
// segments (closed/full-wrap segments are left intact and reported). Returns the number of
// new segments created; 0 means the scaffold already paved every discovered crossing.
int refine_scaffold_at_breaks(SectionScaffold& scaf,
                              const std::map<int, std::vector<double>>& breaks);

} // namespace session_cpp
