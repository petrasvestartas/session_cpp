#pragma once

// P3 same-domain / coincidence subsystem (OCCT port: BOPAlgo_Builder_2 FillSameDomainFaces,
// BOPTools_AlgoTools::AreFacesSameDomain, BOPTools_Set, BOPAlgo_Builder::BuildBOP op-table).
// Standalone NEW module: reads BReps, never mutates kernel state. Integration hooks for the
// boolean pipeline are documented in kb/p3_integration_notes.md and applied by the P1 session.

#include "brep.h"
#include <array>
#include <cstdint>
#include <vector>

namespace session_cpp {

/// Boolean operation with distinct operand roles (object = A = operand 0, tool = B = 1).
/// Cut = A - B; Cut21 = B - A (OCCT BOPAlgo_CUT21).
enum class SDOp { Fuse = 0, Common = 1, Cut = 2, Cut21 = 3 };

/// Whole-face state of a face (fragment) vs the OTHER operand.
/// In/Out = strictly inside/outside; OnSame/OnOpposite = coincident with the other
/// operand's boundary with agreeing/opposing OUTWARD normals (the SD orientation relation).
enum class SDState { In = 0, Out = 1, OnSame = 2, OnOpposite = 3 };

/// Face selection verdict from the BuildBOP orientation op-table.
enum class SDVerdict { Drop = 0, Keep = 1, KeepReversed = 2 };

/// Canonical edge signature, orientation- and parameterization-phase-independent,
/// quantized on a grid of key_tol (BOPTools_Set element analog).
/// Open edge: {sorted endpoints (3+3), param-mid point (3)} — curve reversal maps the
/// domain midpoint to itself, so both traversals produce one signature.
/// Closed edge (rim circles etc.): {sample centroid x2, r_mean, r_spread, 0} — phase of the
/// parameterization (seam-twin cells: same circle rotated about its own axis) cancels out.
struct SDEdgeSig {
    std::array<int64_t, 9> k{};
    bool operator==(const SDEdgeSig& o) const { return k == o.k; }
    bool operator!=(const SDEdgeSig& o) const { return !(k == o.k); }
    bool operator<(const SDEdgeSig& o) const { return k < o.k; }
};

/// Canonical face key = SORTED edge signatures (BOPTools_Set analog: order/orientation
/// independent set identity of the face boundary). Degenerate edges are excluded.
struct SDFaceKey {
    std::vector<SDEdgeSig> sigs;
    bool operator==(const SDFaceKey& o) const { return sigs == o.sigs; }
    bool operator!=(const SDFaceKey& o) const { return !(sigs == o.sigs); }
    bool operator<(const SDFaceKey& o) const { return sigs < o.sigs; }
};

/// One face registered with the detector, plus its detection payload.
struct SDFaceRec {
    const BRep* brep = nullptr;
    int face = -1;      ///< face index in *brep
    int operand = 0;    ///< 0 = object (A), 1 = tool (B)
    int solid = -1;     ///< parent-solid id for the zero-thickness guard
    /// Per-face tolerance (0 -> detector key_tol). OCCT raises each face tolerance to the max
    /// tolerance of its non-degenerate EDGES before summing (BOPTools_AlgoTools.cxx:1173-1196);
    /// this kernel has no per-entity tolerances yet, so callers pass a face tolerance directly.
    /// This field is the P5 (tolerance model) hook: feed max(tolF, max tolE) once it exists.
    double tol = 0.0;
    // filled by detect():
    SDFaceKey key;
    Point pin = Point(0, 0, 0);     ///< interior point on the face material
    Vector nout = Vector(0, 0, 0);  ///< OUTWARD unit normal at pin (osign folded in)
    Point bmin = Point(0, 0, 0), bmax = Point(0, 0, 0);  ///< edge-sample bbox
    bool planar = false;
    bool bounded = true;
    int edge_count = 0;
};

/// Confirmed same-domain pair (i < j, detector indices).
/// orient: 0 = same outward orientation, 1 = opposite. via: 0 = planar shortcut
/// (bounded planar pair with equal keys, no geometric check — OCCT :707-718),
/// 1 = geometric AreFacesSameDomain confirm.
struct SDPairRec {
    int i = -1, j = -1;
    int orient = 0;
    int via = 1;
};

/**
 * @class SameDomain
 * @brief Detector for coincident (same-domain) faces across boolean operands.
 *
 * Pipeline (FillSameDomainFaces port): canonical face keys -> equal-key buckets
 * (+ tolerant bbox/bipartite fallback for quantization straddles) -> same-solid
 * zero-thickness guard -> planar shortcut / interior-point projection confirm ->
 * union-find equivalence classes with min-index representative.
 *
 * Candidacy is purely "identical edge set", exactly as in the Builder: no tangency flag is
 * consulted anywhere (OCCT's TangentFaces is read only by BOPAlgo_CheckerSI, on unsplit faces).
 *
 * DIVERGENCE 1 (load-bearing, read before integrating). OCCT can bucket by edge-set identity
 * only because PostTreatFF has ALREADY fused cross-pair coincident section edges into shared
 * TShapes, so the two operands' copies of one section edge ARE the same object. This kernel has
 * no shared entities at that point, so geometric bucketing at `key_tol` is the substitute and
 * its tolerance is load-bearing: too tight and coincident faces never bucket together (yielding
 * zero SD pairs and wrong COMMON/CUT), too loose and distinct rims collide. See
 * kb/p3_integration_notes.md before wiring HOOK 1.
 *
 * DIVERGENCE 2. The same-solid guard here is STRICTER than OCCT's: OCCT fills its face->solid
 * map only from TopAbs_SOLID sources (first bind wins), so faces of shell-only operands are
 * never guarded at all. Every face registered here carries a `solid` key and is always guarded.
 */
class SameDomain {
public:
    /// key_tol: quantization grid + coincidence band; fuzz: additive fuzzy tolerance.
    explicit SameDomain(double key_tol, double fuzz = 0.0);

    /// Register one face. solid < 0 -> solid = operand. Returns the detector index.
    int add_face(const BRep& b, int face, int operand, int solid = -1, double tol = 0.0);

    /// Register every face of an operand (solid < 0 -> solid = operand).
    void add_solid(const BRep& b, int operand, int solid = -1);

    /// Run detection over all registered faces.
    void detect();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Results
    ///////////////////////////////////////////////////////////////////////////////////////////

    int size() const { return (int)m_faces.size(); }
    const SDFaceRec& face(int i) const { return m_faces[i]; }
    const std::vector<SDPairRec>& pairs() const { return m_pairs; }

    /// Min-index representative of i's SD class (i itself when unmerged).
    int rep(int i) const;
    bool same_domain(int i, int j) const;

    /// Number of classes with >= 2 members.
    int group_count() const;

    /// All classes with >= 2 members, each sorted ascending (first = representative).
    std::vector<std::vector<int>> groups() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Geometric core (exposed for tests)
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// AreFacesSameDomain analog (BOPTools_AlgoTools.cxx:1139-1205). ONE-DIRECTIONAL: the
    /// interior point is taken from A.fa ONLY, so faces_same_domain(A,a,B,b) is not in general
    /// equal to faces_same_domain(B,b,A,a). FAIL-CLOSED: if no interior point can be found the
    /// answer is false ("cannot decide" == "not same domain"); there is deliberately no
    /// PointNearEdge fallback. TWO bands, mirroring IsValidPointForFace:
    ///   band       -- STRICT unsquared rejection of the projection distance (== band passes);
    ///   class_band -- the classifier band that accepts an ON-boundary point (OCCT uses the
    ///                 FACE tolerance of B.fb here, NOT the summed band).
    /// orient (optional): 0/1 relation of the NATURAL surface normals at the matched points
    /// (compose with outward signs for the BuildBOP relation).
    static bool faces_same_domain(const BRep& A, int fa, const BRep& B, int fb,
                                  double band, double class_band, int* orient = nullptr);

    /// OCCT's SD band (audit E3): the max tolerance of **F1's** non-degenerate edges raises
    /// BOTH face tolerances, then band = tolF1' + tolF2' + max(fuzz, 1e-7). F2's own edge
    /// tolerances never enter. Writes the classifier band (tolF2') to `class_band` if given.
    static double sd_band(const BRep& A, int fa, double tolF1, double tolF2, double fuzz,
                          double* class_band = nullptr);

    /// Max tolerance over a face's non-degenerate edges. Returns 0 today: this kernel stores no
    /// per-entity tolerances (phase P5). The P5 hook -- see sd_band.
    static double max_edge_tolerance(const BRep& b, int face);

    /// Canonical key of one face at grid key_tol (sorted edge signatures).
    static SDFaceKey face_key(const BRep& b, int face, double key_tol);

    /// Canonical signature of one topological edge at grid key_tol.
    /// Returns false for a degenerate edge (span below the grid — excluded from keys).
    static bool edge_sig(const BRep& b, int edge, double key_tol, SDEdgeSig& out);

    /// Interior point of a face on its material (UV even-odd vs sampled trim loops):
    /// domain-center probe, then grid scan. Returns false when no material point is found.
    static bool point_in_face(const BRep& b, int face, Point& p3, double& u, double& v);

    /// Up to `want` in-material UV samples spread over the face (areal coincidence probes).
    /// The first entry is point_in_face's probe. Returns {u, v} pairs; empty if the face has
    /// no material point. Used to make ON verdicts AREAL rather than point-wise: a single
    /// probe cannot separate true coincidence from measure-zero tangency (a face centroid can
    /// land exactly on a tangent line / on a smaller flush cap).
    static std::vector<std::array<double, 2>> samples_in_face(const BRep& b, int face,
                                                              int want = 16);

private:
    double m_key_tol;
    double m_fuzz;
    std::vector<SDFaceRec> m_faces;
    std::vector<SDPairRec> m_pairs;
    std::vector<int> m_rep;
    bool m_detected = false;

    int find(int i) const;
};

/// BuildBOP orientation op-table (OCCT BOPAlgo_Builder.cxx:641-737, verified on source).
/// isSameOriNeeded = (objState == toolsState): true for Fuse (OUT/OUT) and Common (IN/IN),
/// false for Cut (OUT/IN) and Cut21 (IN/OUT).
/// SD wall (OnSame/OnOpposite): kept ONCE iff the orientation relation matches the op;
/// the kept copy comes from the op's OUT-state side (object for Fuse/Cut, tool for Cut21;
/// object by convention for Common) — the other side's copy Drops. Otherwise both Drop.
/// Ordinary faces: keep iff takeIN(operand) == (state == In); IN-kept faces of the
/// IN-state side of Cut/Cut21 (tool for Cut, object for Cut21) are KeepReversed.
SDVerdict sd_select_face(SDOp op, int operand, SDState state);

/// Op-table verdict for a face KNOWN to be same-domain-paired with a face of the other operand.
/// `orient_same` is a property of the PAIR (SDPairRec::orient == 0), never a per-face reading.
/// This overload exists because OCCT does not carry an "ON" state at all: the survivor is decided
/// by TWO fences (BOPAlgo_Builder.cxx:682/696/714) -- aMFence, orientation-BLIND (IsSame), then
/// aMFenceOri, orientation-SENSITIVE (IsEqual = TShape+Location+Orientation). The ON-same vs
/// ON-opposite distinction exists ONLY because those two identities are kept separate, so a
/// caller must carry BOTH sightings of the wall (the pair), not a single face's measurement.
SDVerdict sd_select_sd_face(SDOp op, int operand, bool orient_same);

/// ON-classification of face `face` of S against solid `other` (section-block states).
/// AREAL verdict (Law 3: an ON state is a typed outcome, never a lucky near-zero): the face
/// is probed at up to `probes` in-material samples; ON requires EVERY sample to lie within
/// tol of `other`'s trimmed boundary (with the orientation relation taken by majority), so
/// measure-zero contact (line/point tangency) and partial overlap can never read ON.
/// Otherwise In/Out by majority signed side of the nearest boundary (contains_point_exact
/// doctrine). A face that is only PARTLY coincident reports its majority In/Out state --
/// exact per-region states require splitting first (see kb/p3_integration_notes.md).
/// osign_S / osign_other: face_outward_signs of each solid (empty -> computed internally;
/// pass cached vectors when classifying many faces).
SDState sd_classify_face(const BRep& S, int face, const BRep& other, double tol,
                         const std::vector<double>& osign_S = {},
                         const std::vector<double>& osign_other = {},
                         int probes = 16);

/// As sd_classify_face but over an EXPLICIT sample set -- the per-REGION form. `uvs` are UV
/// samples of `face` belonging to one region (a sub-area of the face, e.g. the coincident part
/// of a partially-shared face). The areal rule is unchanged: ON requires EVERY sample within
/// tol, so a region verdict is exact where a whole-face verdict would be a majority vote.
/// Empty `uvs` -> Out (fail-closed).
SDState sd_classify_samples(const BRep& S, int face,
                            const std::vector<std::array<double, 2>>& uvs,
                            const BRep& other, double tol,
                            const std::vector<double>& osign_S = {},
                            const std::vector<double>& osign_other = {});

/// Distance from `p` to the TRIMMED boundary of `b` (nearest trimmed face: interior projection
/// where the projection lands inside the trims, else the distance to the trim boundary).
/// Writes the nearest face index to `face` when given. Returns 1e300 for a BRep with no faces.
/// This is the membership primitive the common-block region predicate is built on.
double sd_distance_to_boundary(const BRep& b, const Point& p, int* face = nullptr);

/// Coincidence-only boolean assembly (test/reference driver): valid when every face of both
/// operands classifies as a WHOLE (no partial face overlaps) — A-op-A cells, flush stacks,
/// full shared walls. Selects faces per sd_select_face, assembles via subset/append, sews.
/// Returns an empty BRep (0 faces) for void results. The kernel integration path instead
/// hooks the detector into split_with fragments — see kb/p3_integration_notes.md.
BRep sd_boolean_coincident(const BRep& A, const BRep& B, SDOp op, double tol);

} // namespace session_cpp
