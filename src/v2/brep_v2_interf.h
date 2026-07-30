#pragma once

// V2 INTERFERENCE STAGES — VV, VE, EE, VF, EF.
//
// Port of OCCT's BOPAlgo_PaveFiller stages 1..5
// (/home/petras/code/code_cpp/OCCT/src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_{1..5}.cxx
//  + TKBO/IntTools/{IntTools_Context, IntTools_EdgeEdge, IntTools_EdgeFace, IntTools_ShrunkRange}).
// Build documents: kb/port_02_vv_ve_ee.md (stages 1-3), kb/port_03_vf_ef.md (stages 4-5).
//
// NEW MODULE. It reads BReps, it writes only into a BdsArena (src/brep_bds.h), and nothing in the
// existing kernel calls it. src/brep_bds.* and src/brep_massprops.* are used, never modified.
//
// ---------------------------------------------------------------------------------------------
// WHY IT EXISTS
// ---------------------------------------------------------------------------------------------
// The v1 kernel has no EF stage at all (kb/port_03_vf_ef.md §4.3): where an edge PIERCES a face
// there is no vertex, so the pierced edge is never split, so the face arrangement downstream has
// no node to route through. Per-case analytic patching cannot cover rotation x topology x
// trimming. These five stages create the vertices and paves every later stage routes through.
//
// ORDER IS A DATA DEPENDENCY, NOT A CONVENTION (kb/port_02 §2.7, G11):
//   VV -> SD map + rebuilt pave blocks -> VE -> extra paves -> EE -> new vertices
//      -> VF -> face IN/ON sets -> EF -> piercing vertices + paves + FaceInfo entries.
// Each stage must be TOTAL before the next begins. perform_all() enforces this; the individual
// perform_xx() entry points exist so a test can run a prefix of the pipeline.
//
// ---------------------------------------------------------------------------------------------
// THE FIVE THINGS THIS FILE GUARANTEES (each has a cell in main_15.cpp)
// ---------------------------------------------------------------------------------------------
// P1. Vertex fusion is the OCCT BRepLib::BoundingVertex rule (exact 2-ball smallest enclosing
//     sphere for two, lexicographically-sorted mean + max radius for more), idempotent and
//     order-independent. The rule itself lives in bds_bounding_vertex(); VV only computes the
//     connected components and calls BdsArena::fuse_vertices() once per component.
// P2. A vertex lying on an edge becomes a PAVE on that edge, referring to the vertex BY ARENA
//     INDEX. No downstream stage may rediscover the incidence by distance.
// P3. An edge/edge crossing becomes a new vertex + a pave on each side; an OVERLAP becomes a
//     BdsCommonBlock, never a point set (kb/port_02 G9).
// P4. A vertex on a face becomes an entry in that face's IN set, found through a UV-RESTRICTED
//     projection and a STRICT in-classification (a foot exactly on the trim boundary is VE's
//     business, not VF's -- kb/port_03 §2.1.2).
// P5. Where an edge pierces a face there is a new vertex, a pave on the edge, and a FaceInfo
//     entry -- and TRANSVERSAL piercing is typed differently from TANGENTIAL contact and from
//     full COINCIDENCE. The type is decided by the sign of the surface-normal residual along the
//     edge, not inferred downstream from a distance.
//
// ---------------------------------------------------------------------------------------------
// DELIBERATE DIVERGENCES FROM OCCT (all measured, none accidental)
// ---------------------------------------------------------------------------------------------
// D1. IntTools_EdgeEdge's box-shrink recursion (kb/port_02 §2.6.6) has no termination proof.
//     We replace it by a bounded sampling + Newton scheme: sample, bracket, refine. Same
//     OUTCOMES (VERTEX parts for crossings, EDGE parts for overlaps, EDGE promotion only when
//     the common range covers essentially the whole of one side) with a hard sample budget.
// D2. IntTools_BeanFaceIntersector's marched tolerance tube (kb/port_03 §2.4) is replaced by the
//     SIGNED normal residual s(t) = (C(t) - S(u*,v*)) . N(u*,v*) along the edge. s is exactly
//     +-dist at a genuine projection foot, so a transversal piercing is a SIGN CHANGE of s -- a
//     bracketable, pose-independent event -- while a tangential contact is a minimum of |s| with
//     no sign change. This is what makes the transversal/tangential typing structural.
// D3. Per-entity tolerances do not exist in BRep (brep.h:27-58), so everything starts at
//     V2_CONFUSION and grows through BdsArena::absorb_tolerance, which records the growth.

#include "brep.h"
#include "brep_bds.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "point.h"
#include <array>
#include <map>
#include <string>
#include <vector>

namespace session_cpp {
// EVERYTHING this module defines lives in session_cpp::v2int. The kernel is built as a UNITY
// build (CMakeLists.txt:43) so every src/*.cpp shares one translation unit: a generic name at
// session_cpp scope collides with the other v2 modules (v2sec, v2sf, v2sol). This namespace is
// the ownership boundary.
namespace v2int {

/// OCCT Precision::Confusion() — a MODEL-SPACE length.
constexpr double V2_CONFUSION = 1e-7;
/// OCCT Precision::PConfusion() — a PARAMETER-space resolution. One of the five parameter-space
/// constants kb/port_03 §3 permits.
constexpr double V2_PCONFUSION = 1e-9;
/// OCCT Precision::Intersection() — the "is this contact real or only touching" gate (PF5:472).
constexpr double V2_INTERSECTION = 1e-9;
/// BOPTools_AlgoTools::DTolerance() — the floor added to a new EF vertex (AT2:254-271).
constexpr double V2_DTOLERANCE = 1e-12;
/// PF5:416 — "the piercing sits on a pave" decision tolerance, in parameter space.
constexpr double V2_TOL_TO_DECIDE = 5e-8;

///////////////////////////////////////////////////////////////////////////////////////////
// 1. Face view — surface + UV trim loops + the adaptor UV rectangle + a cached grid
///////////////////////////////////////////////////////////////////////////////////////////

/// Result of classifying a UV point against a face's trim loops.
/// OCCT's IntTools_Context::IsPointInFace is `state != OUT && state != ON` (strict IN, CTX:604);
/// IsPointInOnFace accepts ON (CTX:639). Both are needed, so the three states are distinct here.
enum class V2State : unsigned char { In = 0, On = 1, Out = 2 };

/// A face as the interference stages need it: the surface, the UV rectangle the projector is
/// restricted to (OCCT's BRepAdaptor_Surface bounds, CTX:253), the trim loops sampled in UV, and
/// a cached evaluation grid so a projection never costs a global surface search.
struct V2FaceView {
    const NurbsSurface* surface = nullptr;
    int    face_index = -1;                ///< arena index
    double umin = 0, umax = 1, vmin = 0, vmax = 1;
    std::vector<std::vector<std::array<double, 2>>> loops;   ///< sampled UV polygons
    /// Cached grid: gp[i * nv + j] == surface->point_at(gu[i], gv[j]).
    std::vector<double> gu, gv;
    std::vector<Point>  gp;
    int nu = 0, nv = 0;
    /// Periodicity of the underlying surface, cached: a foot that Newton drives past the u (or v)
    /// boundary of a CLOSED direction has to be retried from the opposite edge of the rectangle,
    /// or the projection sticks on the seam and reports a distance that is not the distance.
    bool closed_u = false, closed_v = false;

    bool ok() const { return surface != nullptr && !gp.empty(); }
    /// Even-odd crossing rule over ALL loops (outer and inner), with an ON band of (du,dv).
    V2State classify(double u, double v, double du, double dv) const;
};

/// Build the view. `loop_samples` is the number of samples per 2D trim curve span; `grid` is the
/// cached grid resolution per direction.
V2FaceView v2_make_face_view(const BRep& b, int local_face, int arena_face,
                             int loop_samples = 12, int grid = 33);

///////////////////////////////////////////////////////////////////////////////////////////
// 2. Projections — the two primitives every stage is built on
///////////////////////////////////////////////////////////////////////////////////////////

/// OCCT GeomAPI_ProjectPointOnCurve semantics (%GA%:131-153 over %EX%/Extrema_GGExtPC.hxx:503).
/// Returns the number of INTERIOR stationary points of |P - C(t)|^2 in (t0,t1) and fills
/// t/dist with the closest of them. Endpoints are added ONLY when |P - C(end)|^2 < 1e-14
/// (Extrema_GGExtPC.hxx:474-501). IT MUST NOT CLAMP: a clamping "closest parameter" here creates
/// a pave at an arc's end for a vertex that is nowhere near the arc, which yields a zero-length
/// pave block and welds two vertices that must stay distinct (kb/port_02 §2.3.3).
int v2_proj_pc(const NurbsCurve& c, double t0, double t1, const Point& p,
               double& t, double& dist);

/// UV-restricted projection onto a surface (OCCT ProjPS, CTX:247-265). Grid-seeded, then Newton
/// on (S-P).Su = (S-P).Sv = 0, clamped to the rectangle. `useed`/`vseed` < -1e299 means "no seed"
/// (use the grid alone); passing the previous sample's answer as the seed is what makes marching
/// along an edge cheap. Returns false only when the surface is unusable.
bool v2_proj_ps(const V2FaceView& f, const Point& p, double useed, double vseed,
                double& u, double& v, double& dist);

/// Unit surface normal at (u,v); returns false at a pole (|Su x Sv| below `eps`).
bool v2_surface_normal(const NurbsSurface& s, double u, double v, double& nx, double& ny,
                       double& nz, double eps = 1e-12);

///////////////////////////////////////////////////////////////////////////////////////////
// 3. Stage predicates
///////////////////////////////////////////////////////////////////////////////////////////

/// BOPTools_AlgoTools::ComputeVV (%BT%:1772-1794). 0 == INTERFERING. Strictly greater, so exact
/// touching counts as interfering.
int v2_compute_vv(const Point& p1, double tol1, const Point& p2, double tol2, double fuzz);

/// IntTools_Context::ComputeVE (%IT%:499-541).
/// 0 ok, -1 degenerate, -3 no interior projection, -4 distance too big.
/// tol_new = dist + tol(E)  — NOT + tol(V) (%IT%:534).
int v2_compute_ve(const Point& pv, double tolv, const NurbsCurve& c, double t0, double t1,
                  double tole, double fuzz, double& t, double& tol_new);

enum class V2VF : int { Ok = 0, NoProjection = -1, TooFar = -2, OutsideFace = -3 };

/// IntTools_Context::ComputeVF (CTX:545-590). The projector is UV-restricted, the containment
/// test is STRICT IN (a foot exactly ON the trim is rejected with -3 — that case belongs to VE),
/// and tol_new = dist + tol(F), not a max.
V2VF v2_compute_vf(const Point& p, double tolv, const V2FaceView& f, double tolf, double fuzz,
                   double& u, double& v, double& tol_new, double& dist);

///////////////////////////////////////////////////////////////////////////////////////////
// 4. Edge/edge and edge/face cores
///////////////////////////////////////////////////////////////////////////////////////////

enum class V2PartType : unsigned char { Vertex = 0, Edge = 1 };

/// One common part of two edges (IntTools_CommonPrt restricted to the EE case).
struct V2EEPart {
    V2PartType type = V2PartType::Vertex;
    double a0 = 0, a1 = 0;      ///< range on edge A
    double b0 = 0, b1 = 0;      ///< range on edge B
    double ta = 0, tb = 0;      ///< representative parameters (Vertex only)
    double dist = 0;            ///< 3D distance at (ta, tb)
    bool   tangential = false;  ///< IntTools_EdgeEdge's bTouchConfirm (%IT%:889)
};

/// IntTools_EdgeEdge port (kb/port_02 §2.6), sampling-based per divergence D1.
/// Returns false only for unusable data. `parts` is empty when the edges do not interfere.
bool v2_edge_edge(const NurbsCurve& ca, double a0, double a1, double tola,
                  const NurbsCurve& cb, double b0, double b1, double tolb,
                  double fuzz, std::vector<V2EEPart>& parts);

/// How an edge meets a face. This is the typing kb/port_03 G3 demands, and it is decided by the
/// SIGN of the normal residual, not by a distance threshold (divergence D2).
enum class V2EFKind : unsigned char {
    Transversal = 0,   ///< s(t) changes sign: the edge goes through the surface
    Tangential  = 1,   ///< |s| has a minimum below criteria with NO sign change: it touches
    Coincident  = 2    ///< dist <= criteria over essentially the whole range: it lies ON the face
};

/// One common part of an edge and a face (IntTools_CommonPrt restricted to the EF case).
struct V2EFPart {
    V2PartType type = V2PartType::Vertex;
    V2EFKind   kind = V2EFKind::Transversal;
    double t0 = 0, t1 = 0;      ///< range on the edge; a Vertex part still carries one (IEF:355)
    double tv = 0;              ///< the piercing/contact parameter (Vertex only)
    double u = 0, v = 0;        ///< the foot on the surface at tv
    double dist = 0;            ///< 3D distance at tv
    V2State state = V2State::In;///< classification of (u,v) against the face's trims at tv
};

/// IntTools_EdgeFace port (kb/port_03 §2.3/§2.4), signed-residual based per divergence D2.
/// criteria = tol(E) + tol(F) + fuzz (IEF:548).
bool v2_edge_face(const NurbsCurve& c, double t0, double t1, double tole,
                  const V2FaceView& f, double tolf, double fuzz,
                  std::vector<V2EFPart>& parts, int samples = 0);

///////////////////////////////////////////////////////////////////////////////////////////
// 5. The driver
///////////////////////////////////////////////////////////////////////////////////////////

struct V2InterfOptions {
    double fuzzy = V2_CONFUSION;      ///< clamped to >= V2_CONFUSION, as BOPAlgo_Options does
    int    ef_samples = 0;            ///< 0 = derive from the curve's span count
    bool   add_interfs = true;
    /// OCCT's MakeType emits a VERTEX common part for a tangential touch too, and PerformEF then
    /// mints a vertex from it. Keep that (the arrangement still needs the node), but the record
    /// carries kind == Tangential so no consumer can mistake it for a piercing.
    bool   vertex_for_tangential = true;
};

struct V2InterfStats {
    int vv_pairs = 0, vv_clusters = 0, vv_fused = 0;
    int ve_pairs = 0, ve_paves = 0, ve_rejected = 0;
    int ee_pairs = 0, ee_vertex_parts = 0, ee_edge_parts = 0;
    int ee_new_vertices = 0, ee_common_blocks = 0, ee_paves = 0;
    int vf_pairs = 0, vf_in = 0, vf_rejected_on = 0, vf_rejected_far = 0;
    int ef_pairs = 0, ef_transversal = 0, ef_tangential = 0, ef_coincident = 0;
    int ef_new_vertices = 0, ef_paves = 0, ef_on_pave = 0, ef_rejected_out = 0;
};

/// One EF interference, as the consumers (splitter, face builder) read it.
struct V2EFRecord {
    int      edge = -1, face = -1;   ///< arena indices
    V2EFKind kind = V2EFKind::Transversal;
    V2PartType type = V2PartType::Vertex;
    double   t = 0;                  ///< parameter on the edge (Vertex parts)
    double   t0 = 0, t1 = 0;         ///< the common part's range on the edge
    double   u = 0, v = 0;           ///< foot on the face
    int      new_vertex = -1;        ///< arena vertex created (or reused) for this piercing
    bool     on_pave = false;        ///< the piercing landed on one of the block's end vertices
};

struct V2VFRecord {
    int    vertex = -1, face = -1;
    double u = 0, v = 0, dist = 0;
};

struct V2EERecord {
    int    edge_a = -1, edge_b = -1;
    V2PartType type = V2PartType::Vertex;
    double ta = 0, tb = 0;
    int    new_vertex = -1;
    bool   tangential = false;
};

class V2Interf {
public:
    /// `ds` must already have been init()'d on `operands`, in the same order.
    V2Interf(BdsArena& ds, const std::vector<const BRep*>& operands,
             const V2InterfOptions& opt = V2InterfOptions());

    void perform_vv();
    void perform_ve();
    void perform_ee();
    void perform_vf();
    void perform_ef();
    /// The five stages in their mandatory order, with the pave-block rebuild between them.
    void perform_all();

    const V2InterfStats& stats() const { return m_st; }
    const std::vector<V2EFRecord>& ef_records() const { return m_ef; }
    const std::vector<V2VFRecord>& vf_records() const { return m_vf; }
    const std::vector<V2EERecord>& ee_records() const { return m_ee; }
    /// Total interference count — the quantity kb/port_02 G12 requires to be rigid-motion
    /// invariant.
    int interf_count() const;
    const V2FaceView& face_view(int arena_face);

    /// Cheap oracle-free sweep over everything the five stages produced. Returns false and fills
    /// `why` on the first violation.
    bool check_invariants(std::string* why = nullptr) const;

private:
    struct Cand { int a, b; };
    std::vector<Cand> candidates(BdsType ta, BdsType tb) const;
    bool  sub_shape_of(int container, int sub) const;
    bool  interferes_with_sub(int x, int container) const;
    /// BOPAlgo_Tools::IntersectVertices (%BA%:1119-1205): chains of vertices whose (grown)
    /// tolerance spheres overlap; every unpaired vertex is its own 1-element chain.
    void  chain_new_vertices(const std::vector<Point>& pts, const std::vector<double>& tols,
                             std::vector<std::vector<int>>& chains) const;

    BdsArena&                m_ds;
    std::vector<const BRep*> m_ops;
    V2InterfOptions          m_opt;
    V2InterfStats            m_st;
    std::vector<V2EFRecord>  m_ef;
    std::vector<V2VFRecord>  m_vf;
    std::vector<V2EERecord>  m_ee;
    std::map<int, V2FaceView> m_fv;
};

///////////////////////////////////////////////////////////////////////////////////////////
// 6. Verdict metric — validated against BRep::is_solid() before it is used for anything
///////////////////////////////////////////////////////////////////////////////////////////

/// Count of NAKED edges: edges with exactly one trim. ZERO-LENGTH DEGENERATE EDGES (a sphere's
/// poles, a cone's apex) and ORPHANED 0-trim records are EXCLUDED, exactly as BRep::is_solid()
/// excludes them (brep.cpp:1063-1100). Five measurement errors this session came from metrics
/// that counted a degenerate edge as naked; main_15 validates this against is_solid() on a
/// sphere, a cone and a cylinder BEFORE reporting any other number.
int v2_naked_edges(const BRep& b, int* non_manifold = nullptr, int* degenerate = nullptr);

/// true iff v2_naked_edges() == 0 and no edge carries more than two trims.
bool v2_closed(const BRep& b);

}  // namespace v2int
}  // namespace session_cpp
