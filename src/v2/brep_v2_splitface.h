#pragma once

// FACE SPLITTER — one trimmed face + the section edges on it -> the correct set of new faces.
//
// Port of OCCT BOPAlgo_BuilderFace / BOPAlgo_WireSplitter / BOPTools_AlgoTools
// (/home/petras/code/code_cpp/OCCT/src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderFace.cxx,
//  BOPAlgo_WireSplitter.cxx, BOPAlgo_WireSplitter_1.cxx, BOPTools/BOPTools_AlgoTools.cxx),
// specified in kb/port_09_builder_face.md with file:line anchors; seam/pole rules from
// kb/port_08_seam_pole_periodic.md.  NEW MODULE: nothing in the existing kernel calls it.
//
// ============================================================================================
// WHY IT EXISTS — THE DEFECT THIS FILE MAKES UNREPRESENTABLE
// ============================================================================================
// In the v1 kernel edge identity is established by COORDINATE COINCIDENCE after a boundary
// snap: every UV run is lifted to a fresh 3D polyline and matched to its twin through a
// quantised coordinate key (brep.cpp `vmap`/`emap`, q6 = llround(x*1e6)), which only ever works
// because nurbssurface_trimmed.cpp `snap_border` forces every sample that lands near the
// surface's own domain border onto u0/u1/v0/v1 exactly, making two faces' copies BIT-IDENTICAL.
// Measured consequence: split ONE box alone — no boolean at all — on a surface whose domain is
// padded to [-0.04, 4.04] while the face's trims sit at {0, 4}, and the snap never fires: the
// last bits diverge, 32 of 36 edges go naked, while the UV arrangement is verified IDENTICAL to
// the working case (cuts=1 parts=2 per face both ways). Anything trimmed, rotated, imported or
// intersected loses that accident.
//
// This splitter cannot express that failure, for four structural reasons:
//
//  1. IT MINTS NOTHING. `sf_split_face` creates no vertex, no edge, no curve. Its output is a
//     list of INDICES INTO ITS OWN INPUT LIST. `SfReport::minted_edges/minted_vertices` are
//     asserted 0 and there is no code path that could raise them.
//  2. IDENTITY IS A SHARED ARENA ENTITY. An input element is an orientation VIEW of a
//     `BdsPB` (`std::shared_ptr<BdsPaveBlock>`). Face A and face B are handed the SAME
//     shared_ptr object, taken from `BdsArena::pave_blocks(edge)`. Adjacency is therefore
//     pointer identity — `sf_pave_block_id()` reports it as the pair (arena edge index,
//     position in that edge's pave-block pool), a table read.
//  3. THERE IS NO SEWING, WELDING, SNAPPING OR COORDINATE-MATCHING FUNCTION IN THIS API.
//     Their absence is the design. Grep this header for "weld", "sew", "snap", "quantise",
//     "find_near": nothing. `SfEmitter` materialises each distinct pave block as EXACTLY ONE
//     BRep edge, keyed by pave-block identity, created on first use and reused thereafter, so
//     two faces adjacent across an edge get the same BRep edge index BY CONSTRUCTION.
//  4. NO EPSILON IS A FRACTION OF THE UV DOMAIN. Every parametric epsilon here is a MODEL-SPACE
//     distance pushed through the surface metric at the point of use (`sf_uv_tolerance`), so
//     padding a domain by 4x changes nothing (kb/port_09_builder_face.md G7, divergence D15).
//
// ============================================================================================
// THE ANGULAR ORDER AT A MULTI-EDGE VERTEX — the crux, and why the metric enters
// ============================================================================================
// At a vertex where more than two edges meet, the wire walk must take the LEFTMOST TURN. That
// requires sorting the outgoing directions cyclically. Three candidate keys exist:
//
//   (a) the 3D tangent, or its projection into some global plane   -- WRONG.
//       The projection is not the surface's own frame; on a curved patch it can reverse the
//       sense of rotation (consider two edges leaving a point on the far side of a sphere).
//   (b) the RAW UV tangent, atan2(dv, du)                          -- correct order, bad key.
//       The surface differential J = [S_u | S_v] is a rank-2 linear map R^2 -> tangent plane
//       with det > 0 in the frame that defines the FORWARD normal, and a nonsingular linear map
//       of the plane preserves the CYCLIC ORDER of directions and the sense of rotation.
//       So in exact arithmetic raw UV gives the same winner as any metric-corrected key — this
//       is why OCCT can use it (BOPAlgo_WireSplitter_1.cxx:768-840). What it does not give is
//       a well-CONDITIONED key: J squashes angles by the anisotropy ratio |S_u|/|S_v|. Two
//       edges leaving a vertex at a genuine 90 degrees on a surface with |S_u|/|S_v| = 1e6
//       differ by ~2e-6 radians in raw UV, and by ~2e-12 at 1e12. The key then carries almost
//       no information and the winner is decided by float noise in the secant sample. That is
//       the "correct arrangement, wrong lift" signature.
//   (c) the UV tangent CORRECTED BY THE SURFACE METRIC                -- what this file uses.
//       Measure the angle of dS = du*S_u + dv*S_v in the orthonormal tangent frame
//       e1 = S_u/|S_u|, e2 = (S_v - (F/E) S_u)/|...|, which is Gram-Schmidt and hence
//       orientation-consistent with (S_u, S_v). With the first fundamental form
//       E = S_u.S_u, F = S_u.S_v, G = S_v.S_v and det = EG - F^2:
//
//              angle = atan2( dv * sqrt(det),  du * E + dv * F )
//
//       (derivation: dS.e1 = (du E + dv F)/sqrt(E), dS.e2 = dv sqrt(det)/sqrt(E); the common
//       positive factor 1/sqrt(E) drops out of atan2). For an orthonormal parameterisation
//       E = G = 1, F = 0 and this reduces exactly to atan2(dv, du), so (c) is (b) plus a
//       conditioning fix — same order, honest magnitudes. It is the TRUE angle on the surface.
//
// The metric is singular exactly at a POLE (det -> 0). There the key falls back to raw UV and
// `SfAlert::MetricDegenerate` is reported: degeneracy is TYPED, never a lucky near-zero
// (kb/port_09_builder_face.md G8).
//
// Near-ties are broken by SECANT ESCALATION, not by a hard-coded nudge: when two records at a
// node are within SF_ANGLE_TIE, only the tied records are re-measured with a longer secant
// (0.01, 0.05, 0.15, 0.40 of the pcurve span in turn), which is what distinguishes two curves
// that leave the vertex tangentially — their chords separate at a rate set by the curvature
// difference. This replaces OCCT's `RefineAngles`/`RefineAngle2D` ray-intersection scheme
// (WireSplitter_1.cxx:905-1125) and its +-Precision::Angular() last-resort nudge; the
// divergence is deliberate and is recorded in kb/v2_splitface_notes.md.
//
// ============================================================================================
// SEAMS AND POLES (kb/port_08_seam_pole_periodic.md S1-S5)
// ============================================================================================
// A SEAM edge is ONE pave block supplied as TWO input entries with DIFFERENT pcurves (u = umin
// and u = umax), opposite senses. That is literally BOPTools_AlgoTools3D::DoSplitSEAMOnFace
// (:200-231, BB.UpdateEdge(aSp, aC1, aC2, aF, aTol)). Because the two entries name the same
// pave block, the emitted face carries the edge twice with two pcurves, which no
// coordinate-based mating can produce.
// A POLE edge is a degenerate input (`SfInputEdge::degenerate`): a real edge with a real
// pcurve, one vertex used twice, no 3D extent. It is NEVER pruned as a valence-1 dangler
// (BOPAlgo_BuilderFace.cxx:200-203) and a wire may never consist of degenerate edges only
// (BOPAlgo_WireSplitter_1.cxx:437-445).
// One 3D vertex with several UV images (seam ends, poles) is handled by comparing UV positions
// of records that ALREADY SHARE ONE ARENA VERTEX INDEX. That is a disambiguation INSIDE a named
// entity; it never creates identity, and it is the only place a coordinate is compared at all.
//
// ============================================================================================
// SEAM COMPOSITION — regions that meet ACROSS a seam are ONE face  (primary requirement)
// ============================================================================================
// u = u0 and u = u1 are THE SAME 3D CURVE. A section that STRADDLES the seam therefore cuts the
// UV rectangle into more regions than the surface has faces: a circle straddling a sphere's
// meridian leaves a lune against u = u0 and a lune against u = u1 that are the two halves of
// ONE disk. Wire walking alone yields three areas where the answer is two, and the seam pieces
// between them stay 1-trim, i.e. naked. This is the measured curved-solid failure; it is
// orientation-CHAOTIC precisely because one degree of tilt moves a section across a seam, and
// it never touches box x box because a plane has no seam.
//
// THE RULE, applied after the walk and before the area stage: a SEAM pave block whose two
// occurrences land in TWO DIFFERENT wires is INTERIOR to their union -- splice the two wires
// into one and drop both occurrences. A seam pave block whose two occurrences land in the SAME
// wire is a genuine seam of that face and is left alone (that face wraps the chart, and the
// edge legitimately carries two trims belonging to one face). The spliced-in half is translated
// by the exact whole-period offset MEASURED BETWEEN THE TWO PCURVES OF THE SHARED BLOCK
// (`SfWire::shift_u/shift_v`), so the composed wire is continuous in UV: OCCT's
// AdjustPCurveOnSurf, derived from the shared entity instead of from coordinates. Composition
// is driven entirely by pave-block IDENTITY. Because the shift can carry a composed face
// outside the surface's stored parameter rectangle by up to one period, a consumer must
// evaluate the surface PERIODICALLY in that direction; `SfWire::seam_composed` flags it.

#include "brep.h"
#include "brep_bds.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include <string>
#include <unordered_map>
#include <vector>

namespace session_cpp {
namespace v2sf {

///////////////////////////////////////////////////////////////////////////////////////////
// 0. Constants — every one of them is either dimensionless or a MODEL-SPACE distance.
///////////////////////////////////////////////////////////////////////////////////////////

/// OCCT Precision::Confusion(); a model-space length (Precision.hxx:165).
constexpr double SF_CONFUSION = 1e-7;
/// OCCT Precision::PConfusion(); the floor on any curve-parameter step (Precision.hxx:334).
constexpr double SF_PCONFUSION = 1e-9;
/// Two angular keys closer than this at one node are a TIE and get secant escalation.
constexpr double SF_ANGLE_TIE = 1e-9;
/// Strict-improvement margin in the leftmost-turn comparison; the FIRST candidate in input
/// order wins a tie, which is the determinism anchor (OCCT Epsilon(1.), WireSplitter_1.cxx:380).
constexpr double SF_ANGLE_EPS = 2.220446049250313e-16;
/// Secant fractions used by the tie escalation, in order.
constexpr int SF_ESCALATION_LEVELS = 4;

///////////////////////////////////////////////////////////////////////////////////////////
// 1. Input — an ORIENTATION VIEW of a SHARED arena pave block
///////////////////////////////////////////////////////////////////////////////////////////

enum class SfSense : unsigned char { Forward = 0, Reversed = 1 };

/// One element of the face's edge set.
///
/// MULTIPLICITY IS THE INPUT'S JOB, exactly as in OCCT (BOPAlgo_Builder_2.cxx:363-494):
///   * a boundary edge of the face  -> ONE entry, with the loop's traversal sense;
///   * a section / IN / INTERNAL edge -> TWO entries, Forward and Reversed, same `pb`;
///   * a seam edge -> TWO entries, same `pb`, DIFFERENT `pcurve`, opposite senses.
/// `is_inside` (OCCT's "supplied with both senses") is DERIVED from that multiplicity here; it
/// is never a caller flag, so it cannot disagree with the list.
struct SfInputEdge {
    /// THE IDENTITY. The same shared_ptr object is handed to every face that touches this
    /// interval. Nothing in this file compares coordinates to decide that two entries are the
    /// same edge; they are the same edge iff `pb.get()` is the same pointer.
    BdsPB pb;
    /// Traversal sense relative to the pave block's natural direction (pave1 -> pave2).
    SfSense sense = SfSense::Forward;
    /// The pcurve of this occurrence on THIS face, always parameterised in the pave block's
    /// NATURAL direction: pcurve(t_first) is the UV of pave1.vertex, pcurve(t_last) that of
    /// pave2.vertex. The traversal direction lives in `sense`, never in the curve object — so
    /// two adjacent faces never hold two different curve objects for one edge (divergence D14).
    /// Owned by the caller; must outlive the call.
    const NurbsCurve* pcurve = nullptr;
    /// Pole row: a real edge with a real pcurve and no 3D extent. Never pruned as a dangler.
    bool degenerate = false;
    /// Caller provenance for diagnostics only. NEVER used for identity.
    int tag = -1;
};

///////////////////////////////////////////////////////////////////////////////////////////
// 2. Output
///////////////////////////////////////////////////////////////////////////////////////////

/// A wire: oriented input-edge views in traversal order. `inputs` holds INDICES INTO THE INPUT
/// LIST — the zero-minting law (G1) restated as a type.
struct SfWire {
    std::vector<int> inputs;
    /// PERIODIC SEAM COMPOSITION. Parallel to `inputs`: the whole-period UV translation to add
    /// to that occurrence's pcurve so the wire is CONTINUOUS in UV. Empty means all-zero.
    /// Non-zero only on a face assembled from regions that meet ACROSS a seam.
    std::vector<double> shift_u, shift_v;
    double area_uv = 0.0;     ///< signed UV area of the sampled wire polygon (shifts applied)
    bool   is_hole = false;   ///< area_uv < 0
    bool   degenerate_area = false;
    bool   seam_composed = false;  ///< assembled from more than one walk wire across a seam
};

struct SfFace {
    SfWire outer;
    std::vector<SfWire> holes;
    std::vector<SfWire> internals;   ///< unconsumed material placed inside this face (G9)
};

/// Nothing is silently dropped (kb/port_09_builder_face.md G9).
enum class SfAlert : unsigned char {
    NoPCurve, MicroEdge, DanglingPruned, DoubledHangingPruned, EdgeNotConsumed,
    WireDegenerateArea, HoleUnassigned, NoLoops, AngleTieUnresolved, WalkDeadEnd,
    WalkStepCap, MetricDegenerate, DegenerateOnlyWire
};
const char* sf_alert_name(SfAlert a);

struct SfReport {
    std::vector<std::pair<SfAlert, int> > events;   ///< (alert, input index or -1)
    /// STRUCTURALLY ZERO: this subsystem has no allocator for either.
    int minted_edges = 0;
    int minted_vertices = 0;
    int walk_steps = 0;
    int angle_escalations = 0;
    int seam_merges = 0;   ///< regions joined ACROSS a periodic seam into ONE face
    int nodes = 0;
    int max_valence = 0;
    void add(SfAlert a, int i) { events.push_back(std::make_pair(a, i)); }
    int  count(SfAlert a) const;
    std::string str() const;
};

struct SfOptions {
    /// MODEL-SPACE tolerance used when the arena has none for a vertex.
    double tol3d = SF_CONFUSION;
    /// OCCT myAvoidInternalShapes: when true the internal-wire stage is a no-op.
    bool avoid_internal = false;
    /// Hard blow-up guard. OCCT has none (kb/audit_occt_blowup-guards.md). 0 => 8*n + 64.
    int max_walk_steps = 0;
    /// Samples per pcurve when building a wire's UV polygon for area / classification.
    int wire_samples = 32;
};

struct SfResult {
    std::vector<SfFace> faces;
    SfReport report;
    /// Input indices that reached no wire and no internal wire. Must be empty on clean input.
    std::vector<int> unused;
};

///////////////////////////////////////////////////////////////////////////////////////////
// 3. Metric — the ONLY place a 3D tolerance becomes a UV epsilon, and the sorting key
///////////////////////////////////////////////////////////////////////////////////////////

/// First fundamental form at (u,v). Returns false when the metric is SINGULAR (a pole), where
/// no orthonormal tangent frame exists and the caller must handle the degeneracy by name.
bool sf_first_form(const NurbsSurface& s, double u, double v,
                   double& E, double& F, double& G, double& sqrt_det);

/// THE SORTING KEY: the angle of the UV direction (du,dv) measured in the surface's own
/// orthonormal tangent frame, folded into [0, 2*pi). Falls back to raw atan2(dv,du) at a pole
/// and sets `*degenerate` — see the header comment for the full derivation.
double sf_metric_angle(const NurbsSurface& s, double u, double v,
                       double du, double dv, bool* degenerate = nullptr);

/// Raw parametric angle, provided ONLY so tests can measure how much conditioning the metric
/// correction buys. Never used by the splitter.
double sf_raw_angle(double du, double dv);

/// MODEL-SPACE -> per-axis UV tolerance at (u,v):  du = tol3d/|S_u|, dv = tol3d/|S_v|.
/// False at a pole. No caller may divide a domain extent by a constant instead (G7).
bool sf_uv_tolerance(const NurbsSurface& s, double u, double v, double tol3d,
                     double& du, double& dv);

///////////////////////////////////////////////////////////////////////////////////////////
// 4. Pave-block identity — a table read, never a search
///////////////////////////////////////////////////////////////////////////////////////////

/// The canonical arena name of a pave block: (arena index of its original edge, its position in
/// that edge's pave-block pool). Two faces adjacent across an edge report the SAME pair.
struct SfPaveBlockId {
    int edge = -1;
    int block = -1;
    bool valid() const { return edge >= 0 && block >= 0; }
    bool operator==(const SfPaveBlockId& o) const { return edge == o.edge && block == o.block; }
    bool operator!=(const SfPaveBlockId& o) const { return !(*this == o); }
    bool operator<(const SfPaveBlockId& o) const {
        return edge != o.edge ? edge < o.edge : block < o.block;
    }
};
SfPaveBlockId sf_pave_block_id(const BdsArena& arena, const BdsPaveBlock* pb);

///////////////////////////////////////////////////////////////////////////////////////////
// 5. THE SUBSYSTEM ENTRY POINT
///////////////////////////////////////////////////////////////////////////////////////////

/// Split ONE trimmed face.
///   `arena`   : the shared arena that owns every vertex and edge named by `inputs`
///   `surface` : the face's surface, UNCHANGED (the splitter never re-parameterises)
///   `inputs`  : boundary edges (multiplicity 1) + section/IN/internal/seam (multiplicity 2)
/// Returns faces whose wires reference ONLY indices into `inputs`.
///
/// ORIENTATION IS INHERITED, NOT DERIVED (G4): every area is built FORWARD relative to the
/// surface; the caller re-applies the source face's orientation at emission.
SfResult sf_split_face(const BdsArena& arena, const NurbsSurface& surface,
                       const std::vector<SfInputEdge>& inputs,
                       const SfOptions& opt = SfOptions());

///////////////////////////////////////////////////////////////////////////////////////////
// 5b. Periodic continuation — so a face composed ACROSS a seam is actually EVALUABLE
///////////////////////////////////////////////////////////////////////////////////////////

/// A face assembled across a seam occupies a UV window that straddles u = u0 (or v = v0), i.e.
/// it reaches up to one period OUTSIDE the surface's stored parameter rectangle. OCCT does not
/// notice because Geom_SphericalSurface::Value() is analytic and periodic; our charts are
/// CLAMPED NURBS, where evaluating past the domain is not the periodic continuation. Any
/// consumer that integrates over the face (mass properties, tessellation, classification) would
/// then read a wrong surface. This function returns the surface CONTINUED periodically so that
/// its domain covers [lo, hi] in direction `dir` -- the same geometry, one or two more periods
/// of it.
///
/// Requires the chart to be closed in `dir` with a control net whose first and last rows
/// COINCIDE and whose knot pattern repeats by the period. That is exactly the structure every
/// quadric chart in this kernel has (Primitives::sphere/cylinder/cone/torus_surface: 9 control
/// points, knots {0,0,1,1,2,2,3,3,4,4}). Returns false, changing nothing, when it does not hold
/// -- the caller then knows the face is not evaluable rather than silently reading garbage.
bool sf_periodic_extend(const NurbsSurface& s, int dir, double lo, double hi, NurbsSurface& out);

///////////////////////////////////////////////////////////////////////////////////////////
// 6. Emission — the identity law made literal in the output BRep
///////////////////////////////////////////////////////////////////////////////////////////

/// Materialises split faces into a BRep. ONE `SfEmitter` serves a whole solid, so every face
/// that references a pave block gets the SAME BRep edge index. The maps are keyed by arena
/// index and by pave-block pointer; there is no coordinate key, no tolerance and no search.
class SfEmitter {
public:
    explicit SfEmitter(BRep& out) : m_out(out) {}

    /// POLE REPRESENTATION — a deliberate, stated choice, not an accident.
    /// A pole/apex is a pcurve singularity. Two conventions exist and both are watertight:
    ///   * true  (default) — OCCT / kb/port_08_seam_pole_periodic.md S5: the pole row is a real
    ///     EDGE RECORD with a real pcurve, one vertex used twice and ZERO 3D extent. Every
    ///     validity rule in this kernel (BRep::is_solid brep.cpp:1090-1102, v2v::v2_verdict,
    ///     sf_manifold) already excludes zero-length edges from the naked count, so such a
    ///     record is watertight by construction.
    ///   * false — the convention this kernel's own primitives use (BRep::create_sphere,
    ///     create_cone carry NO edge record at the singularity): the singular trim keeps its
    ///     pcurve and carries `edge_index = -1`, so no zero-length edge record is created.
    /// The SPLITTER itself is unaffected either way: it mints nothing, and the pole row must be
    /// present in the INPUT list as a degenerate `SfInputEdge` in both conventions -- without it
    /// the face's UV loop is open and the meridians dangle. Only the emitted record differs.
    void set_pole_edges(bool on) { m_pole_edges = on; }
    bool pole_edges() const { return m_pole_edges; }

    /// Emit the areas of one source face. Returns the number of BRep faces added.
    /// `face_reversed` is inherited from the source face and is written verbatim (G4).
    int emit(const BdsArena& arena, const NurbsSurface& surface,
             const std::vector<SfInputEdge>& inputs, const SfResult& res, bool face_reversed);

    /// BRep edge index of a pave block, or -1 if it was never emitted. Pointer identity.
    int edge_of(const BdsPaveBlock* pb) const;
    /// BRep topology-vertex index of an arena vertex, or -1. Index identity.
    int vertex_of(int arena_vertex) const;

    /// Faces whose composed wire needed a periodically continued surface, and faces where that
    /// continuation was not possible (the chart is not a repeating periodic net).
    int extended_surfaces() const { return m_extended; }
    int inextensible_faces() const { return m_inextensible; }

    int created_edges() const { return (int)m_pb_edge.size(); }
    int reused_edges() const { return m_reused; }
    int created_vertices() const { return (int)m_v_map.size(); }

private:
    int ensure_vertex(const BdsArena& arena, int arena_vertex);
    int ensure_edge(const BdsArena& arena, const BdsPB& pb, bool degenerate);
    int emit_wire(const BdsArena& arena, const std::vector<SfInputEdge>& inputs,
                  const SfWire& w, int face_idx, BRepLoopType type, bool internal);

    BRep& m_out;
    std::unordered_map<const BdsPaveBlock*, int> m_pb_edge;  ///< IDENTITY, not geometry
    std::unordered_map<int, int> m_v_map;                    ///< arena vertex -> BRep vertex
    int m_reused = 0;
    int m_extended = 0;
    int m_inextensible = 0;
    bool m_pole_edges = true;
};

///////////////////////////////////////////////////////////////////////////////////////////
// 7. Verdict metric — must agree with BRep::is_solid() (brep.cpp:1063-1119) before use
///////////////////////////////////////////////////////////////////////////////////////////

/// Manifold census of a BRep. A raw "trims != 2" count is NOT a validity verdict: a POLE edge
/// (sphere pole, cone apex) is watertight by construction and carries one trim, and a SEAM edge
/// appears ONCE in the edge list while being used TWICE by its own face's wires. `naked` counts
/// only GENUINE 1-trim edges with real 3D extent; `degenerate` and `orphan` are reported
/// separately so a verdict can never silently absorb them.
struct SfManifold {
    int edges = 0;
    int naked = 0;          ///< non-degenerate, 1 trim: a real hole in the boundary
    int nonmanifold = 0;    ///< non-degenerate, > 2 trims
    int degenerate = 0;     ///< zero 3D extent (pole/apex): excluded from the verdict
    int orphan = 0;         ///< 0 trims: a dead record, excluded (same scoping as is_solid)
    int seam_edges = 0;     ///< 2 trims that belong to the SAME face
    bool closed() const { return naked == 0 && nonmanifold == 0; }
};
SfManifold sf_manifold(const BRep& b);

}  // namespace v2sf
}  // namespace session_cpp
