#pragma once

// BDS ARENA — the shared intersection state of the v2 boolean kernel.
//
// Port of OCCT's BOPDS subsystem
// (/home/petras/code/code_cpp/OCCT/src/ModelingAlgorithms/TKBO/BOPDS/: BOPDS_DS, BOPDS_Pave,
// BOPDS_PaveBlock, BOPDS_CommonBlock, BOPDS_FaceInfo, BOPDS_ShapeInfo, BOPDS_Interf, BOPDS_Tools)
// plus BOPAlgo_Tools::PerformCommonBlocks / ComputeToleranceOfCB (BOPAlgo_Tools.cxx:107-356) —
// the subsystem kb/audit_occt_pavefiller-core.md flags as omission O1 from every older spec.
// Design: kb/ARCHITECTURE_v2.md §1; full port spec with file:line anchors: kb/port_01_bds_arena.md.
// NEW MODULE: it reads BReps, it never mutates them, and nothing in the existing kernel calls it yet.
//
// WHY IT EXISTS. The v1 kernel establishes EDGE IDENTITY by coordinate coincidence after a
// boundary snap (brep.cpp `vmap`/`emap`, quantised to 1e-6, preceded by two snapping layers), not
// by shared entities. Measured: splitting ONE operand alone on a padded UV domain yields 32 naked
// edges of 36 while the UV arrangement is verified identical to the working case. On an unpadded
// domain a face's trims land exactly on u0/u1/v0/v1, so snap_border forces adjacent faces'
// boundary points to be BIT-IDENTICAL and their edges weld by accident. Anything trimmed,
// rotated, imported or intersected loses that accident. This arena makes the defect
// unrepresentable: identity is an integer index, and there is no function here that finds an
// entity by proximity.
//
// ---------------------------------------------------------------------------------------------
// WHAT THIS PROVIDES THAT `SharedEdgePool` (brep_section.h:57, brep_section.cpp:2583) DOES NOT
// ---------------------------------------------------------------------------------------------
// SharedEdgePool is the partial ancestor of BdsPaveBlock — a pave-block arena, not a common-block
// model. It should be ABSORBED, not paralleled. Point by point:
//
//   SharedEdgePool has                     | BdsArena adds
//   ---------------------------------------+--------------------------------------------------
//   arena: BRep (section geometry only)    | every entity of BOTH operands in ONE index space,
//                                          |   plus everything created during intersection
//                                          |   (m_lines / BdsShape) — L1
//   (no ranges)                            | BdsIndexRange + rank(): "which operand owns this",
//                                          |   so cross-operand pruning is structural — L2
//   (no paves)                             | BdsPave: a vertex sitting at a parameter on an edge,
//                                          |   sorted, unique, always naming a real vertex — L3
//   block_edge[{seg, block}] -> edge index | BdsPaveBlock: an INTERVAL (pave1,pave2) of an edge,
//     a flat map, no parameter range       |   with shrunk range, ext-paves and provenance — L4
//   (no common blocks at all)              | BdsCommonBlock: members list, faces list, explicit
//                                          |   representative, per-block MEASURED tolerance — L5
//   (no per-face state)                    | BdsFaceInfo In/On/Sc — the place an EF result lands;
//                                          |   without it there is no IN set and no EF stage — L6
//   (no interference log)                  | BdsInterf VV/VE/EE/VF/EF/FF + has_interf() — L7
//   populated ONLY from a SectionScaffold  | populated from the operands at init(); non-empty for
//     => EMPTY for pure coincidence, the   |   pure coincidence, which is the case the pool is
//     case it is most needed for           |   most needed for and never sees — L8
//   (no tolerances anywhere)               | per-entity MODEL-SPACE tolerance on every shape and
//                                          |   every common block; growth recorded at merge — L9
//   (no orientation relation)              | BdsCommonBlock::orientation — a recorded slot for the
//                                          |   same/opposite relation. NOT populated here: the
//                                          |   detector lives in brep_samedomain (session B) and
//                                          |   writes it when the block is built.
//   (no member provenance)                 | every pave block records original_edge; every fused
//                                          |   vertex records origin; every CB records whether
//                                          |   its representative was forced (audit E3)
//
// Absorption map (kb/port_01_bds_arena.md §4.2): vert_tv -> the index returned by append_vertex;
// seg_edge -> the materialised edge of a section curve's first pave block; block_edge[{s,k}] ->
// that curve's k-th pave block, and the key disappears because a pave block IS the block.
//
// ---------------------------------------------------------------------------------------------
// THE SIX INVARIANTS THIS FILE MAKES UNREPRESENTABLE (each has a test in main_11.cpp)
// ---------------------------------------------------------------------------------------------
// I1. An edge is never mutated. BdsPaveBlock describes an INTERVAL; splitting is a view
//     materialised later. There is no setter for edge geometry after append, and
//     update_pave_blocks() is idempotent: running it twice does not double anything.
// I2. Two coincident edge pieces are ONE BdsCommonBlock referencing both pave blocks. There is
//     no second copy that can diverge; set_edge() names one edge for every member at once.
// I3. Vertex fusion is idempotent and order-independent. bds_bounding_vertex() is OCCT's
//     BRepLib::BoundingVertex rule (BRepLib.cxx:3013-3123): exact 2-ball smallest enclosing
//     sphere for two, lexicographically-sorted mean + max(dist+tol) for more. fuse_vertices()
//     memoises on the sorted member set, so fusing twice returns the SAME arena index.
// I4. Paves on an edge are sorted by parameter, unique, and every pave names a real vertex.
//     Dedup is by (vertex, parameter) exactly, as OCCT does (BOPDS_Pave::IsEqual), with an
//     additional MODEL-SPACE coincidence gate; paves() hard-asserts the tiling law.
// I5. Every tolerance stored here is a model-space distance. No parameter-space tolerance is
//     stored or compared anywhere in this file. Where a UV tolerance is needed, convert at the
//     point of use through the surface metric: bds_uv_tolerance() = tol3d / |dS/du|.
// I6. Identity is by arena index. There is deliberately NO function that finds an entity by
//     proximity — no coordinate hash, no quantiser, no weld radius, no nearest lookup. Two
//     vertices at identical coordinates stay DISTINCT until an explicit fuse_vertices() call.
//     bds_check_coincidence() is a predicate over two NAMED pave blocks; it returns bool, never
//     an entity, and so cannot be used to establish identity.

#include "brep.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "point.h"
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// 0. Scalars, type codes, boxes
///////////////////////////////////////////////////////////////////////////////////////////

/// OCCT Precision::Confusion(). A MODEL-SPACE length (Precision.hxx:165).
constexpr double BDS_CONFUSION = 1e-7;
/// Cycle cap on the same-domain redirection walk. OCCT has none and hangs on a 2-cycle
/// (BOPDS_DS.cxx:1244-1253); we cap and report.
constexpr int BDS_SD_MAX_HOPS = 64;
/// Number of interior samples used by compute_tolerance_of_cb (BOPAlgo_Tools.cxx:271).
constexpr int BDS_CB_SAMPLES = 11;

enum class BdsType : unsigned char { Vertex = 0, Edge = 1, Face = 2, Solid = 3 };

/// Dimension-ordered code mirroring TopAbs (SOLID 2 < FACE 4 < EDGE 6 < VERTEX 7): a LARGER
/// code means a LOWER dimension. BOPDS_Tools::TypeToInteger(BOPDS_Tools.lxx:86-123).
int bds_type_code(BdsType t);
/// BOPDS_Tools::HasBRep — only these three enter the candidate-pair search.
inline bool bds_has_brep(BdsType t) {
    return t == BdsType::Vertex || t == BdsType::Edge || t == BdsType::Face;
}
/// BOPDS_Tools::IsInterfering — HasBRep plus SOLID (self-intersection checker only).
inline bool bds_is_interfering(BdsType t) { return bds_has_brep(t) || t == BdsType::Solid; }

enum class BdsInterfType : signed char {
    None = -1, VV = 0, VE = 1, EE = 2, VF = 3, EF = 4, FF = 5
};
constexpr int BDS_NB_INTERF_TYPES = 6;

/// Exact port of the BOPDS_Tools::TypeToInteger(t1,t2) dispatch table (BOPDS_Tools.lxx:31-82),
/// restricted to the six types the boolean pipeline actually produces.
BdsInterfType bds_pair_type(BdsType t1, BdsType t2);

/// Axis-aligned box carrying its own gap, so a pair test is "is_out on boxes inflated by BOTH
/// gaps" — the sum-of-gaps semantics that yields tol(a) + tol(b) + fuzz. All model space.
struct BdsBox {
    double lo[3] = { 1e300,  1e300,  1e300};
    double hi[3] = {-1e300, -1e300, -1e300};
    double gap = 0.0;
    bool   is_void() const { return lo[0] > hi[0]; }
    void   add(const Point& p);
    /// Unions the OTHER box's INFLATED extents and leaves this->gap alone, so a later
    /// set_gap(get_gap() + add_tol) reproduces BOPDS_DS::prepareEdges exactly (BOPDS_DS.cxx:1688).
    void   add(const BdsBox& b);
    void   set_gap(double g) { gap = g; }
    double get_gap() const { return gap; }
    bool   is_out(const BdsBox& o) const;
};

/// One operand's ownership window in the flat arena. INCLUSIVE on both ends (BOPDS_IndexRange).
struct BdsIndexRange {
    int first = 0, last = -1;
    bool contains(int i) const { return i >= first && i <= last; }
    int  size() const { return last - first + 1; }
};

///////////////////////////////////////////////////////////////////////////////////////////
// 1. Arena slot and geometry records
///////////////////////////////////////////////////////////////////////////////////////////

/// One entity of either operand, or one entity created during intersection.
/// Port of BOPDS_ShapeInfo. `index` is its own arena index (G1: shape(i).index == i).
struct BdsShape {
    int      index = -1;
    BdsType  type = BdsType::Vertex;
    int      operand = -1;       ///< 0 = A, 1 = B, -1 = created after init()
    int      source = -1;        ///< index inside the operand BRep, or -1 when created
    int      geom = -1;          ///< index into m_vertex_geom / m_edge_geom / m_face_geom
    std::vector<int> subs;       ///< sub-shape ARENA indices (never local indices)
    double   tol = 0.0;          ///< MODEL-SPACE tolerance (I5)
    int      reference = -1;     ///< edge -> pave-block pool; face -> face-info pool
    int      flag = -1;          ///< >= 0 means "set"; 0 IS a valid value (BOPDS_ShapeInfo.lxx:19)
    BdsBox   box;

    bool has_reference() const { return reference >= 0; }
    bool has_flag()      const { return flag >= 0; }
    bool has_brep()      const { return bds_has_brep(type); }
    bool has_sub(int i)  const;
};

/// A fused geometric vertex: one per location, in model space (kb/ARCHITECTURE_v2.md §1).
/// `origin` is provenance: -1 for a source vertex, else the arena index of the first member of
/// the cluster it was fused from.
struct BdsVertex {
    Point  p;
    double tol = 0.0;
    int    origin = -1;
};

/// Geometry of an edge slot. Non-owning `curve` for a source edge (points into the operand's
/// BRep, valid for the arena's lifetime); `owned` for an edge created after init(). NEITHER is
/// ever modified after append — that is invariant I1, enforced by the absence of a setter.
struct BdsEdgeGeom {
    const NurbsCurve* curve = nullptr;
    std::shared_ptr<NurbsCurve> owned;
    double t0 = 0.0, t1 = 0.0;
    bool   closed = false;       ///< start vertex == end vertex (the seam rule)
};

/// Geometry of a face slot. Non-owning; the surface belongs to the operand's BRep.
struct BdsFaceGeom {
    const NurbsSurface* surface = nullptr;
};

///////////////////////////////////////////////////////////////////////////////////////////
// 2. Paves and pave blocks
///////////////////////////////////////////////////////////////////////////////////////////

/// A vertex sitting at a parameter on an edge (BOPDS_Pave).
/// OCCT documents (BOPDS_DS.cxx:1350-1355) that its `<` and `==` are inconsistent: `<` compares
/// the PARAMETER only, `==` compares (vertex, parameter) EXACTLY. We keep the same split —
/// sort by parameter, dedup by the pair — and never put a pave in a std::set.
struct BdsPave {
    int    vertex = -1;
    double t = 0.0;
    bool less(const BdsPave& o) const { return t < o.t; }
    bool same(const BdsPave& o) const { return vertex == o.vertex && t == o.t; }
};

/// One INTERVAL of one edge. The edge itself is never mutated (I1).
/// Reference-counted because the same block is aliased by the pool, the common-block map and two
/// faces' info sets — exactly OCCT's handle<BOPDS_PaveBlock> aliasing, and the mechanism by which
/// a section edge reaches both faces as ONE object.
class BdsPaveBlock {
public:
    int     original_edge = -1;  ///< arena index of the edge this interval lives on
    int     edge = -1;           ///< arena index of the MATERIALISED split edge; -1 until made
    BdsPave pave1, pave2;

    std::vector<BdsPave> ext;    ///< pending subdivisions, consumed by update()
    std::vector<int>     fence;  ///< vertex indices already in `ext` (BOPDS_PaveBlock::myMFence)

    double ts1 = 0.0, ts2 = 0.0; ///< shrunk range
    BdsBox shrunk_box;
    bool   has_shrunk = false;
    bool   splittable = false;

    void range(double& a, double& b) const { a = pave1.t; b = pave2.t; }
    void indices(int& v1, int& v2) const { v1 = pave1.vertex; v2 = pave2.vertex; }
    double length_param() const { return pave2.t - pave1.t; }
    bool has_edge()      const { return edge >= 0; }
    bool is_split_edge() const { return edge != original_edge; }
    bool is_to_update()  const { return !ext.empty(); }
    /// Order-insensitive match of the two vertex indices (BOPDS_PaveBlock::HasSameBounds).
    bool has_same_bounds(const BdsPaveBlock& o) const;

    /// Fenced by VERTEX index. False when that vertex is already pending.
    bool append_ext_pave(const BdsPave& p);
    /// Unfenced (the seam case). DIVERGENCE FROM OCCT, deliberate: we still record the vertex in
    /// `fence` so remove_ext_pave() can reach it. OCCT's AppendExtPave1 paves are unremovable
    /// (BOPDS_PaveBlock.cxx:184-202).
    void append_ext_pave1(const BdsPave& p);
    void remove_ext_pave(int vertex);
    /// Scans `ext` ONLY, never pave1/pave2 (BOPDS_PaveBlock::ContainsParameter).
    bool contains_parameter(double t, double tol, int& vertex_out) const;

    /// Subdivide into (n-1) blocks from n sorted paves, each inheriting ONLY original_edge:
    /// a freshly subdivided block is unmaterialised (edge = -1, no shrunk data). `use_own` is
    /// OCCT's theFlag: false is used only when pave1/pave2 are still unset.
    /// Always clears ext + fence. BOPDS_PaveBlock::Update(BOPDS_PaveBlock.cxx:249-312).
    void update(std::vector<std::shared_ptr<BdsPaveBlock>>& out, bool use_own = true);
};
using BdsPB = std::shared_ptr<BdsPaveBlock>;

/// Split one interval at one pave into exactly two intervals covering it. The two halves carry
/// BITWISE the original endpoints (a.pave1 == pb.pave1, b.pave2 == pb.pave2, a.pave2 == b.pave1),
/// so "covers exactly" is an equality test, not a tolerance test.
/// Returns false when `at` does not lie strictly inside (pave1.t, pave2.t).
bool bds_split_interval(const BdsPaveBlock& pb, const BdsPave& at, BdsPB& a, BdsPB& b);

///////////////////////////////////////////////////////////////////////////////////////////
// 3. Common block — coincidence is ONE object (I2)
///////////////////////////////////////////////////////////////////////////////////////////

class BdsCommonBlock {
public:
    std::vector<BdsPB> members;   ///< members[0] is the representative
    std::vector<int>   faces;     ///< arena face indices this block lies ON
    double             tol = 0.0; ///< compute_tolerance_of_cb result; MODEL SPACE
    int                orientation = 0;  ///< 0 same / 1 opposite, when known; provenance only
    /// True once set_representative() overrode the min-original-edge rule. OCCT's
    /// SetRealPaveBlock (BOPDS_CommonBlock.cxx:114-126) does this silently, which is why the
    /// "min-index representative" claim in older specs is broken (audit E3). We record it.
    bool               representative_forced = false;

    /// Keeps the minimal original_edge at the front (BOPDS_CommonBlock::AddPaveBlock:39-56).
    void add(const BdsPB& pb);
    void set_members(const std::vector<BdsPB>& v);
    void set_representative(const BdsPB& pb);
    const BdsPB& representative() const { return members.front(); }
    bool contains(const BdsPB& pb) const;       ///< pointer identity, never geometry
    bool contains_face(int f) const;
    BdsPB member_on_edge(int original_edge) const;   ///< null shared_ptr when absent
    /// Name ONE materialised edge for EVERY member at once. This is invariant I2 made literal.
    void set_edge(int e) { for (auto& m : members) m->edge = e; }
    int  edge() const { return members.empty() ? -1 : members.front()->edge; }
};
using BdsCB = std::shared_ptr<BdsCommonBlock>;

///////////////////////////////////////////////////////////////////////////////////////////
// 4. Face info — In / On / Sc. THERE IS NO "OUT" SET.
//    (corrects kb/ARCHITECTURE_v2.md §1, which lists _out; BOPDS_FaceInfo.hxx:129-137 has Sc)
///////////////////////////////////////////////////////////////////////////////////////////

/// Insertion-ordered set of pave blocks (OCCT NCollection_IndexedMap semantics). Order must be
/// stable and reproducible: the face builder walks these by index, and determinism of the whole
/// boolean depends on it never being a hash order.
class BdsPBSet {
public:
    bool add(const BdsPB& pb);
    bool contains(const BdsPB& pb) const;
    void clear();
    int  size() const { return (int)m_v.size(); }
    const BdsPB& operator[](int i) const { return m_v[i]; }
    void remove(const BdsPB& pb);
    std::vector<BdsPB>::const_iterator begin() const { return m_v.begin(); }
    std::vector<BdsPB>::const_iterator end()   const { return m_v.end(); }
private:
    std::vector<BdsPB> m_v;
    std::unordered_map<const BdsPaveBlock*, int> m_ix;
};

struct BdsFaceInfo {
    int index = -1;
    BdsPBSet pb_on, pb_in, pb_sc;
    std::vector<int> v_on, v_in, v_sc;   ///< sorted unique arena vertex indices
    /// OCCT's Clear() forgets Sc (BOPDS_FaceInfo.lxx:55-61). Ours clears all six.
    void clear();
};

///////////////////////////////////////////////////////////////////////////////////////////
// 5. Interference records
///////////////////////////////////////////////////////////////////////////////////////////

/// A typed interference between two arena entities, plus what it created.
struct BdsInterf {
    BdsInterfType type = BdsInterfType::None;
    int a = -1, b = -1;
    int new_vertex = -1;             ///< vertex created by this interference, or -1
    std::vector<int> new_curves;     ///< section curve slots (FF), or empty
    double ta = 0.0, tb = 0.0;       ///< VE: parameter on the edge; EE/EF: common-part range
    double u = 0.0, v = 0.0;         ///< VF/EF: parameters on the surface
    bool contains(int i) const { return i == a || i == b; }
    int  opposite(int i) const { return i == a ? b : (i == b ? a : -1); }
    bool has_new_vertex() const { return new_vertex >= 0; }
};

///////////////////////////////////////////////////////////////////////////////////////////
// 6. Free functions — geometry rules, used by the arena and directly testable
///////////////////////////////////////////////////////////////////////////////////////////

/// OCCT BRepLib::BoundingVertex (BRepLib.cxx:3013-3123), the vertex-fusion rule.
///  - n < 2: no change (caller keeps the single vertex).
///  - n == 2: exact smallest enclosing sphere of two balls. Let m be the larger-tolerance ball,
///    n the smaller, dR = R[m]-R[n], d = |P[m]-P[n]|. If d <= dR or d < RealEpsilon the big ball
///    already contains the small one, so the result IS the big ball. Otherwise
///    R = (R[m]+R[n]+d)/2 and C = (P[m]+P[n] - (P[n]-P[m])*(dR/d))/2.
///  - n > 2: points are sorted LEXICOGRAPHICALLY before summing (OCCT issue 0027540: a sum of
///    doubles depends on addition order), C = mean, R = max over members of (|C-P_i| + tol_i).
/// Order-independent by construction, which is half of invariant I3.
void bds_bounding_vertex(const std::vector<Point>& pts, const std::vector<double>& tols,
                         Point& center_out, double& tol_out);

/// Model-space -> UV conversion through the surface metric, computed AT THE POINT OF USE (I5).
/// du = tol3d / |dS/du|, dv = tol3d / |dS/dv|. Returns false when the metric is degenerate
/// (a pole), where no finite UV tolerance exists and the caller must handle the degeneracy by
/// name rather than by a lucky near-zero.
bool bds_uv_tolerance(const NurbsSurface& s, double u, double v, double tol3d,
                      double& du_out, double& dv_out);

/// Model-space -> curve-parameter conversion, same rule: dt = tol3d / |dC/dt|.
bool bds_t_tolerance(const NurbsCurve& c, double t, double tol3d, double& dt_out);

/// Fingerprint of a curve's (degree, knots, control points). Used to prove invariant I1: an
/// edge's geometry is bit-identical before and after every arena operation.
unsigned long long bds_curve_fingerprint(const NurbsCurve& c);

///////////////////////////////////////////////////////////////////////////////////////////
// 7. THE ARENA
///////////////////////////////////////////////////////////////////////////////////////////

class BdsArena {
public:
    BdsArena() = default;
    BdsArena(const BdsArena&) = delete;              ///< the arena is never copied
    BdsArena& operator=(const BdsArena&) = delete;

    //--- construction ---------------------------------------------------------------------
    /// Flatten `operands` into ONE index space: solid -> faces -> edges -> vertices, with
    /// sub-shape identity taken from the BRep's own indices. No coordinate is read to decide
    /// identity (I6). `fuzz` is a MODEL-SPACE length.
    void init(const std::vector<const BRep*>& operands, double fuzz = BDS_CONFUSION);
    void clear();
    /// True when init() had to assume BDS_CONFUSION because BRep carries no per-entity
    /// tolerance. Tests use it to distinguish "measured 1e-7" from "assumed 1e-7".
    bool tolerances_are_default() const { return m_tol_default; }

    //--- arena ----------------------------------------------------------------------------
    int  nb_shapes() const { return (int)m_lines.size(); }
    int  nb_source_shapes() const { return m_nb_source; }
    bool is_new_shape(int i) const { return i >= m_nb_source; }
    const BdsShape& shape(int i) const { return m_lines[i]; }
    BdsShape& change_shape(int i) { return m_lines[i]; }
    int  nb_ranges() const { return (int)m_ranges.size(); }
    const BdsIndexRange& range(int k) const { return m_ranges[k]; }
    int  rank(int i) const;                          ///< -1 for entities created after init()

    bool is_vertex(int i) const { return valid(i) && m_lines[i].type == BdsType::Vertex; }
    bool is_edge(int i)   const { return valid(i) && m_lines[i].type == BdsType::Edge; }
    bool is_face(int i)   const { return valid(i) && m_lines[i].type == BdsType::Face; }
    bool valid(int i)     const { return i >= 0 && i < (int)m_lines.size(); }

    /// IDENTITY LOOKUP — a table read, not a search. There is deliberately no by-proximity
    /// counterpart anywhere in this header (I6).
    int index_of_vertex(int operand, int local) const;
    int index_of_edge(int operand, int local) const;
    int index_of_face(int operand, int local) const;

    //--- append (entities created after init(); rank == -1) --------------------------------
    int append_vertex(const Point& p, double tol, int origin = -1);
    /// Takes ownership of a copy of `c`. v1/v2 are ARENA vertex indices. Inserts the two
    /// bounding paves at once, so every pave always names a real vertex (I4).
    int append_edge(const NurbsCurve& c, int v1, int v2, double tol);
    /// Reference an operand's existing curve without copying it.
    int append_edge_ref(const NurbsCurve* c, double t0, double t1, int v1, int v2, double tol);

    //--- geometry (uniform over source and created entities) -------------------------------
    const BdsVertex& vertex(int i) const;
    const Point& vertex_point(int i) const;
    const BdsEdgeGeom& edge_geom(int i) const;
    const NurbsCurve* edge_curve(int i) const;
    std::pair<double,double> edge_range(int i) const;
    const NurbsSurface* face_surface(int i) const;
    double tolerance(int i) const { return m_lines[i].tol; }
    /// tol := max(tol, d). Growth is RECORDED at the merge site; there is no global epsilon.
    void absorb_tolerance(int i, double d);

    //--- vertex fusion (I3) ----------------------------------------------------------------
    /// Fuse a cluster of vertex indices into one vertex under the BoundingVertex rule.
    /// IDEMPOTENT and ORDER-INDEPENDENT: the cluster is sorted and deduplicated, the result is
    /// memoised on that key, and every member is redirected through the same-domain map. Calling
    /// it twice with the same members — in any order — returns the SAME index, creates no second
    /// vertex, and yields bitwise the same point and tolerance.
    int fuse_vertices(std::vector<int> cluster);
    int nb_fusions() const { return (int)m_fusion_memo.size(); }

    //--- same domain -----------------------------------------------------------------------
    void add_shape_sd(int i, int i_sd);              ///< refuses i == i_sd
    bool has_shape_sd(int i, int& i_sd) const;
    /// Capped chain walk. Returns i itself when unbound. Sets m_sd_cycle and returns the last
    /// hop when the chain exceeds BDS_SD_MAX_HOPS — OCCT hangs here instead.
    int  resolve_sd(int i) const;
    bool sd_cycle_detected() const { return m_sd_cycle; }

    //--- paves and pave blocks (I1, I4) -----------------------------------------------------
    bool has_pave_blocks(int e) const { return valid(e) && m_lines[e].has_reference(); }
    const std::vector<BdsPB>& pave_blocks(int e) const;
    std::vector<BdsPB>& change_pave_blocks(int e);   ///< lazily builds the initial single block
    /// Add a pave to the edge's CURRENT block covering `t`. Model-space coincidence with an
    /// existing pave is REJECTED rather than silently accepted: two vertices at the same place
    /// must be fused first, which keeps paves unique (I4).
    enum class PaveStatus { Inserted, DuplicateVertex, CoincidentVertex, OutOfRange, BadEdge, BadVertex };
    PaveStatus add_pave(int e, double t, int vertex, double fuzz = 0.0);
    /// Rebuild the edge's block list from its pending paves. IDEMPOTENT (I1): a second call with
    /// no new paves changes nothing and creates nothing.
    int update_pave_blocks(int e);
    void update_pave_blocks();
    /// Sorted, deduplicated pave set of `e`. Hard-asserts the tiling law
    /// (paves == blocks + 1, strictly increasing) and returns false when it is violated.
    bool paves(int e, std::vector<BdsPave>& out) const;
    /// Do the blocks of `e` cover its range exactly, end to end, with no gap and no overlap?
    /// Compared BITWISE — a tolerance here would defeat the point.
    bool pave_blocks_cover_edge(int e) const;

    //--- common blocks (I2) ------------------------------------------------------------------
    bool  is_common_block(const BdsPB& pb) const;
    BdsCB common_block(const BdsPB& pb) const;       ///< null when none
    void  set_common_block(const BdsPB& pb, const BdsCB& cb);
    BdsPB real_pave_block(const BdsPB& pb) const;    ///< CB ? representative : pb
    bool  is_common_block_on_edge(const BdsPB& pb) const;
    /// BOPAlgo_Tools::PerformCommonBlocks overload A (BOPAlgo_Tools.cxx:107-187): groups of
    /// fewer than 2 are skipped; the FIRST member that already has a common block donates that
    /// object and every member CB's faces are merged into it, deduplicated; then the tolerance
    /// is recomputed. Returns the block, or null for a group smaller than 2.
    BdsCB make_common_block(const std::vector<BdsPB>& pbs);
    /// Overload B (BOPAlgo_Tools.cxx:191-244): one block per pave block, appending only faces
    /// not already present, then recomputing the tolerance.
    BdsCB make_common_block_on_faces(const BdsPB& pb, const std::vector<int>& faces);
    /// BOPAlgo_Tools::ComputeToleranceOfCB (BOPAlgo_Tools.cxx:248-356). tol starts at the
    /// tolerance of the REPRESENTATIVE'S ORIGINAL EDGE; early-exits when there are fewer than 2
    /// members and no faces; otherwise samples 11 STRICTLY INTERIOR points of the
    /// representative's pave range at dt = (t2-t1)/12, projects each onto every other member's
    /// original edge and every attached face, and takes max(tol, tol(other) + distance).
    double compute_tolerance_of_cb(const BdsCB& cb) const;

    //--- face info ---------------------------------------------------------------------------
    bool has_face_info(int f) const { return valid(f) && m_lines[f].has_reference(); }
    const BdsFaceInfo& face_info(int f) const;
    BdsFaceInfo& change_face_info(int f);            ///< lazily builds
    /// On is a VIEW of the face's own boundary, canonicalised through common blocks — it is
    /// derived, never authored (BOPDS_DS::FaceInfoOn:811-833). Recomputing is idempotent.
    void face_info_on(int f, BdsPBSet& pb_out, std::vector<int>& v_out) const;
    void update_face_info_on(int f);
    /// Drop from In every pave block also present in On (BOPDS_DS::RefineFaceInfoIn).
    void refine_face_info_in(int f);

    //--- interferences -------------------------------------------------------------------------
    /// Records the typed interference and marks the pair in the summary table.
    int  add_interf(BdsInterfType t, int a, int b);
    BdsInterf& change_interf(int i) { return m_interf[i]; }
    const std::vector<BdsInterf>& interfs() const { return m_interf; }
    bool has_interf(int i) const;
    bool has_interf(int a, int b) const;             ///< symmetric
    int  nb_interf(BdsInterfType t) const;

    //--- diagnostics ----------------------------------------------------------------------------
    /// Canonical, exactly reproducible textual dump of the whole arena. This is the golden
    /// tripwire: a known configuration must produce a byte-identical string.
    std::string signature() const;
    /// G1..G6 sweep. Cheap enough to run at the end of every corpus cell.
    bool check_invariants(std::string* why = nullptr) const;

private:
    int  append_shape(BdsType t, int operand, int source, double tol);
    void init_pave_blocks(int e);
    void init_face_info(int f);

    std::vector<const BRep*>   m_operands;
    std::vector<BdsShape>      m_lines;
    int                        m_nb_source = 0;
    bool                       m_tol_default = true;
    std::vector<BdsIndexRange> m_ranges;

    // identity tables: (operand, local) -> arena index. This IS OCCT's myMapShapeIndex, minus
    // the hash: our BRep is already an indexed arena, so identity is a subscript.
    std::vector<std::vector<int>> m_ix_vertex, m_ix_edge, m_ix_face;

    std::vector<BdsVertex>   m_vertex_geom;
    std::vector<BdsEdgeGeom> m_edge_geom;
    std::vector<BdsFaceGeom> m_face_geom;

    std::vector<std::vector<BdsPB>> m_pb_pool;   ///< indexed by BdsShape::reference
    std::vector<BdsFaceInfo>        m_fi_pool;   ///< indexed by BdsShape::reference
    std::unordered_map<const BdsPaveBlock*, BdsCB> m_pb_cb;

    std::map<std::vector<int>, int> m_fusion_memo;   ///< sorted cluster -> fused vertex index
    std::map<int,int>               m_sd;
    mutable bool                    m_sd_cycle = false;

    std::vector<BdsInterf>          m_interf;
    std::map<std::pair<int,int>,int> m_interf_tb;    ///< (min,max) -> index into m_interf
};

/// Coincidence PREDICATE over two NAMED pave blocks (BOPDS_DS::CheckCoincidence:1288-1331):
/// take one interior point of `a`, project it onto `b`'s original edge, and accept when the
/// distance is below tol(a's vertices) + tol(b's vertices) + max(fuzz, BDS_CONFUSION) AND the
/// projection parameter is STRICTLY inside b's range. It answers "are these two, which the
/// caller already named, one reality?" — it never returns an entity, so it cannot be used to
/// establish identity (I6).
bool bds_check_coincidence(const BdsArena& ds, const BdsPB& a, const BdsPB& b, double fuzz);

} // namespace session_cpp
