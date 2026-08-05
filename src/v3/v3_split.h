#pragma once

// v3 face splitting (BOPAlgo PaveFiller's MakeSplitEdges + WireSplitter +
// BuilderFace equivalent). Two stages:
//  1. split_edges_at_paves: cut a solid's edges at pave parameters, updating
//     every coedge/loop consistently (mating preserved — one entity per
//     feature, the shared Cur is never copied).
//  2. split_face: UV arrangement of the face's (already split) boundary
//     coedges plus section pcurves -> minimal cycles -> new faces.
// Section curves come from ssi() once per face PAIR and are shared by both
// operands (sec_edge ids are global), so result faces mate along identical
// section edges with no sewing.

#include "v3_topo.h"
#include "v3_ssi.h"
#include <map>
#include <vector>

namespace v3 {

// A section pcurve piece on a face, clipped to the face region.
struct SecPC {
    int sec_edge = -1;             // shared section edge id (global pool)
    std::vector<UV2> pc;           // polyline in the face's unwrapped chart
    int end_vtx[2] = {-1, -1};     // solid vertex ids at both ends (-1: floating)
    bool closed = false;           // closed section loop fully inside the face
    // pave attachment (filled by the boolean driver's attach pass): the
    // boundary edge + parameter this endpoint snaps to, if any
    int end_edge[2] = {-1, -1};
    double end_t[2] = {0, 0};
    // per-sample section edge id for chains MERGED from several sections
    // (empty: the whole chain is sec_edge). Junction arcs cut from a merged
    // closed chain must be re-split wherever this changes, so every arc keeps
    // the edge id its A-side counterpart uses.
    std::vector<int> pc_edge;
};

// Split every edge listed in `paves` (edge index -> curve parameters, assumed
// sorted/unsorted, deduped here) into sub-edges sharing the parent Cur.
// Vertices are created per (edge, t). Coedges referencing a split edge are
// replaced by the piece sequence in their loops. Returns per-edge piece ids
// [parent edge] -> [(t_start, t_end, new edge index)...] and, via vtx_of,
// the vertex id created for each (edge, t) pave.
std::map<int, std::vector<std::array<double, 3>>> split_edges_at_paves(
    Solid& S, std::map<int, std::vector<double>>& paves,
    std::map<std::pair<int, double>, int>* vtx_of = nullptr);

// UV arrangement split of one face. The face's boundary coedges must already
// be pave-split. Returns result face parts (same surface index, rev, src).
// New coedges/edges/vertices are appended to S (section-edge pieces already
// exist there with ids from the pool).
std::vector<Face> split_face(Solid& S, const Face& F,
                             const std::vector<SecPC>& secs, double tol3d);

// ---- section clipping helpers (used by the boolean driver) -------------------

// Clip a section curve's sample sequence to runs inside BOTH face regions.
// Transition points are refined by bisection (on inside-ness) and emitted as
// exact interpolated samples, so run endpoints lie ON the face boundary (not
// one sample step away from it).
struct SecRun {
    int i0 = 0, i1 = 0; // inclusive sample range
};
std::vector<SecRun> section_runs(const Solid& SA, const Face& FA,
                                 const Solid& SB, const Face& FB,
                                 const std::vector<SecPoint>& pts, double tol3d);

// refine a transition between samples a (inside) and b (outside): returns an
// interpolated boundary sample. `which`: 0 = A-side crossing, 1 = B-side.
SecPoint refine_boundary(const Solid& SA, const Face& FA, const Solid& SB,
                         const Face& FB, const SecPoint& a, const SecPoint& b,
                         bool has_exact, const Cur& c, int which, double tol3d);

// same for a whole SecCurve (uses its exact curve when present)
std::vector<SecRun> section_runs_refined(const Solid& SA, const Face& FA,
                                         const Solid& SB, const Face& FB,
                                         const SecCurve& sc,
                                         std::vector<SecPoint>& pts_out,
                                         double tol3d);

} // namespace v3
