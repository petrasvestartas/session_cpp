#pragma once

// V2 SOLID ASSEMBLY — stage 10 of the v2 boolean pipeline (kb/port_10_builder_solid.md).
//
// Port of OCCT BOPAlgo_BuilderSolid.cxx (+ BOPAlgo_ShellSplitter, BOPTools_AlgoTools::
// MakeConnexityBlocks / OrientFacesOnShell / GetFaceOff, BRep_Tool::IsClosed, BRepClass3d)
// and of the BuildBOP selection loop (BOPAlgo_Builder.cxx:479-885).
//
// THE ONE RULE THIS FILE ENFORCES (guarantee I-1): two oriented face images are adjacent iff
// they reference the SAME EDGE INDEX in the shared arena. No distance, no tolerance, no
// Hausdorff comparison participates in shell grouping. `V2Topo` is built from a BRep's
// topology tables alone; replace every 3D curve with nonsense and the shell decomposition is
// bit-identical (main_16 test T2).
//
// EVERYTHING LIVES IN namespace session_cpp::v2sol. CMake builds src/*.cpp with UNITY_BUILD ON,
// so file-local helpers with generic names collide across the concatenated translation units;
// nothing here is declared outside this namespace and there is no anonymous namespace.
//
// NEW MODULE. It reads BReps and never mutates the existing kernel. Nothing in brep.cpp calls
// it; the entry point is brep_v2_boolean.h, gated behind SESSION_V2.

#include "brep.h"
#include "brep_massprops.h"
#include "mesh.h"
#include "point.h"
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace session_cpp {
namespace v2sol {

///////////////////////////////////////////////////////////////////////////////////////////
// 0. Constants (kb/port_10_builder_solid.md §2.14)
///////////////////////////////////////////////////////////////////////////////////////////

constexpr double V2_CONFUSION = 1e-7;    ///< Precision::Confusion()
constexpr double V2_ANGULAR = 1e-12;     ///< Precision::Angular()
constexpr double V2_PCONFUSION = 1e-9;   ///< Precision::PConfusion()
/// IntermediatePoint split; deliberately NOT the midpoint so symmetric configurations do not
/// land on a symmetry point (BOPTools_AlgoTools2D.cxx:407).
constexpr double V2_PAR_T = 0.43213918;

/// Vector helpers. Declared ONCE here (never as file-local statics) for the Unity-build reason
/// stated at the top of this file.
inline double v2_vlen(const Vector& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}
inline Vector v2_vunit(const Vector& v) {
    const double l = v2_vlen(v);
    return l > 0 ? Vector(v[0] / l, v[1] / l, v[2] / l) : Vector(0, 0, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////
// 1. Oriented references
///////////////////////////////////////////////////////////////////////////////////////////

/// (face, reversed) is the ORIENTED identity (OCCT IsEqual); `face` alone is the UNORIENTED
/// identity (OCCT IsSame). Both are needed — guarantee I-11.
/// ORIENTATION CONVENTION: the outward normal of (face, reversed) is
/// (reversed ? -1 : +1) * (Su x Sv) of the face's surface. BRepFace::reversed is NOT consulted
/// anywhere in v2; orientation is carried by this flag only.
struct V2OrientedFace {
    int face = -1;
    bool reversed = false;
    bool operator==(const V2OrientedFace& o) const {
        return face == o.face && reversed == o.reversed;
    }
    bool operator!=(const V2OrientedFace& o) const { return !(*this == o); }
    bool operator<(const V2OrientedFace& o) const {
        return face != o.face ? face < o.face : (int)reversed < (int)o.reversed;
    }
};

struct V2Box {
    double lo[3] = {1e300, 1e300, 1e300};
    double hi[3] = {-1e300, -1e300, -1e300};
    bool whole = false;
    void add(const Point& p);
    void add(const V2Box& o);
    bool is_void() const { return !whole && lo[0] > hi[0]; }
    bool is_out(const V2Box& o, double gap = 0.0) const;
    void set_whole() { whole = true; }
    Point center() const;
    double diagonal() const;
};

///////////////////////////////////////////////////////////////////////////////////////////
// 2. Diagnostics — every give-up path of §2.13 emits one of these (guarantee I-13)
///////////////////////////////////////////////////////////////////////////////////////////

enum class V2Alert {
    ShellSplitterFailed,       ///< G8: a connexity block produced no shells
    UnusedFaces,               ///< G10: faces that landed in no shell
    OpenShellAcceptedAsSolid,  ///< G13: only when the caller opts in
    ClassifierUndecided,       ///< G1/G2/G3: growth/hole search exhausted
    AngularTieUnresolved,      ///< G5: get_face_off could not separate two candidates
    HoleWithoutParent,         ///< G9: cavity shell contained in no growth solid
    NonManifoldEdge,           ///< an edge with more than 2 trims survived assembly
    SelectionDoubleBooking     ///< I-9 violation detected during selection
};

struct V2AlertRec {
    V2Alert kind = V2Alert::UnusedFaces;
    std::vector<int> faces;
    std::string detail;
};

const char* v2_alert_name(V2Alert a);

///////////////////////////////////////////////////////////////////////////////////////////
// 3. THE ARENA VIEW — incidence by index only
///////////////////////////////////////////////////////////////////////////////////////////

/// One occurrence of an edge inside a face (an OCCT "edge occurrence" == our trim).
struct V2EdgeUse {
    int trim = -1;
    int edge = -1;
    int face = -1;
    /// +1 when the trim traverses the edge from start_vertex to end_vertex, -1 otherwise.
    /// Purely the stored BRepTrim::reversed flag: no geometry is read.
    int dir = 1;
    bool degenerate = false;
};

/// Edge/face incidence of ONE BRep, built once, from the topology tables only.
/// This IS the arena: an "arena edge index" is a topology-edge index of `b`.
struct V2Topo {
    const BRep* b = nullptr;
    std::vector<V2EdgeUse> uses;                ///< one entry per trim, indexed by trim
    std::vector<std::vector<int>> face_uses;    ///< face -> use (trim) indices
    std::vector<std::vector<int>> edge_uses;    ///< edge -> use (trim) indices
    std::vector<char> edge_degenerate;          ///< pole / apex / zero-length edges
    std::vector<double> edge_length;
    double model_size = 1.0;

    /// Per-face tessellation, built lazily. Used ONLY by the growth/hole and nesting
    /// classifiers; no adjacency decision ever reads it.
    mutable std::vector<Mesh> face_mesh;
    mutable std::vector<double> face_mesh_sign;  ///< +1 when the mesh winding normal agrees
                                                 ///< with the surface's natural (Su x Sv) normal
    mutable bool meshes_built = false;
    void ensure_meshes() const;

    /// Exact per-face boundary integrals (brep_massprops), built lazily. This is the
    /// orientation/volume data the mesh path was an approximation of: FaceMassProps::flux is
    /// the natural-normal vector flux of each face, so an oriented shell volume is a signed
    /// sum, and it costs milliseconds where the CDT tessellation of 512-sample pcurve loops
    /// cost minutes (measured: 164 s -> 0.05 s on box x box rotated).
    mutable MassProps mp;
    mutable bool mp_built = false;
    void ensure_massprops() const;

    void build(const BRep& brep);
    int nb_faces() const { return b ? (int)b->m_faces.size() : 0; }
    int nb_edges() const { return b ? (int)b->m_topology_edges.size() : 0; }
    /// Direction in which the ORIENTED face traverses this edge occurrence.
    int use_dir(int use, bool face_reversed) const {
        return face_reversed ? -uses[use].dir : uses[use].dir;
    }
    /// Does `edge` occur more than once inside `face`? (the seam case)
    bool is_seam(int edge, int face) const;
};

///////////////////////////////////////////////////////////////////////////////////////////
// 4. Shells and solids
///////////////////////////////////////////////////////////////////////////////////////////

struct V2Shell {
    std::vector<V2OrientedFace> faces;
    bool closed = false;      ///< OCCT edge-parity test (BRep_Tool.cxx:1707-1729)
    bool manifold = false;    ///< the stronger 2-trim-everywhere condition
    bool is_hole = false;     ///< classify(infinite point) == IN  (BuilderSolid.cxx:823-831)
    bool internal = false;
    bool oriented_ok = false; ///< I-4 holds: every 2-face edge is traversed oppositely
    V2Box box;
};

struct V2SolidRec {
    int outer_shell = -1;
    std::vector<int> hole_shells;
    std::vector<int> internal_shells;
    V2Box box;
};

///////////////////////////////////////////////////////////////////////////////////////////
// 5. Free functions — each independently testable
///////////////////////////////////////////////////////////////////////////////////////////

/// Connexity block over shared EDGE INDEX only (BOPTools_AlgoTools.cxx:187-256).
/// `regular` is false when a face was supplied twice, or when some edge of the block has an
/// incident-face count != 2 inside the block.
struct V2ConnexityBlock {
    std::vector<V2OrientedFace> faces;
    bool regular = true;
};
std::vector<V2ConnexityBlock> v2_make_connexity_blocks(const V2Topo& topo,
                                                       const std::vector<V2OrientedFace>& faces);

/// Relative-orientation fixer (BOPTools_AlgoTools::OrientFacesOnShell, AlgoTools.cxx:363-507).
/// Reverses faces so that every non-seam edge shared by exactly two faces of the shell is
/// traversed oppositely (I-4). Fixes RELATIVE orientation only — it never globally flips.
void v2_orient_faces_on_shell(const V2Topo& topo, V2Shell& shell);

/// Edge-parity closure (BRep_Tool.cxx:1707-1729). NOT "every edge has two trims".
bool v2_shell_is_closed(const V2Topo& topo, const V2Shell& shell);
/// Stronger: every non-degenerate edge of the shell has exactly two occurrences in it.
bool v2_shell_is_manifold(const V2Topo& topo, const V2Shell& shell);
/// I-4 audit: number of 2-face edges whose two faces traverse them the SAME way.
int v2_shell_orientation_defects(const V2Topo& topo, const V2Shell& shell);

/// AngleWithRef (BOPTools_AlgoTools.cxx:1946-1975) — a monotone surrogate for the true angle,
/// copied verbatim so that `anAngleCriteria` keeps its meaning.
double v2_angle_with_ref(const Vector& d1, const Vector& d2, const Vector& dref);

/// The angular selector (BOPTools_AlgoTools::GetFaceOff, AlgoTools.cxx:994-1103).
/// `ref_use` is the reference face's occurrence of the edge; candidates are (use, face) pairs
/// that traverse the edge the OPPOSITE way. Returns false when the minimum is ambiguous within
/// `crit` (G5); the caller decides whether to trust `picked` anyway (SplitBlock does).
bool v2_get_face_off(const V2Topo& topo, int ref_use, V2OrientedFace ref,
                     const std::vector<std::pair<int, V2OrientedFace>>& candidates,
                     V2OrientedFace& picked, double crit = V2_CONFUSION);

/// Interior point of a face plus its NATURAL unit normal there. False when no material point
/// can be found. Used by growth/hole classification, never by adjacency.
bool v2_face_probe(const BRep& b, int face, Point& p, Vector& natural_normal, double* uu = nullptr,
                   double* vv = nullptr);

///////////////////////////////////////////////////////////////////////////////////////////
// 6. BuilderSolid
///////////////////////////////////////////////////////////////////////////////////////////

class V2BuilderSolid {
public:
    void set_topo(const V2Topo* t) { m_topo = t; }
    void set_faces(std::vector<V2OrientedFace> f) { m_input = std::move(f); }
    void set_avoid_internal_shapes(bool v) { m_avoid_internal = v; }
    /// When true a shell that fails the closure test is still turned into a solid (G13).
    void set_accept_open_shells(bool v) { m_accept_open = v; }

    void perform();   ///< BuilderSolid.cxx:76-125

    const std::vector<V2Shell>& shells() const { return m_shells; }
    const std::vector<V2Shell>& internal_shells() const { return m_internal; }
    const std::vector<V2SolidRec>& areas() const { return m_solids; }
    const std::vector<V2AlertRec>& alerts() const { return m_alerts; }
    const std::set<V2OrientedFace>& avoided() const { return m_avoid; }
    bool has_errors() const { return m_error; }

    /// Face images of the input that ended in no shell (reported, never silently dropped).
    const std::vector<V2OrientedFace>& unused() const { return m_unused; }

private:
    void perform_shapes_to_avoid();   ///< §2.3, BuilderSolid.cxx:129-219
    void perform_loops();             ///< §2.4, BuilderSolid.cxx:223-393
    void perform_areas();             ///< §2.7, BuilderSolid.cxx:397-598
    void split_block(const V2ConnexityBlock& blk, std::vector<V2Shell>& out);

    const V2Topo* m_topo = nullptr;
    std::vector<V2OrientedFace> m_input;
    std::set<V2OrientedFace> m_avoid;   ///< ORIENTED (BuilderArea.hxx:83)
    std::vector<V2Shell> m_shells;      ///< myLoops
    std::vector<V2Shell> m_internal;    ///< myLoopsInternal
    std::vector<V2SolidRec> m_solids;   ///< myAreas
    std::vector<V2OrientedFace> m_unused;
    std::vector<V2AlertRec> m_alerts;
    bool m_avoid_internal = false;
    bool m_accept_open = false;
    bool m_error = false;
};

/// Growth/hole decision for ONE shell, taken from that shell ALONE (guarantee I-5).
/// OCCT does this with PerformInfinitePoint + an exact ray/face intersector
/// (BRepClass3d_SClassifier.cxx:82-199). Our substitute keeps the same contract — one shell, no
/// other shell consulted — and replaces the random-ray search by the SIGN of the shell's own
/// enclosed volume, which is what "the infinite point is IN" means for a closed oriented shell.
/// Sets `undecided` (OCCT's G1 fallback, made visible) when the signed volume is degenerate.
bool v2_shell_is_hole(const V2Topo& topo, const V2Shell& shell, bool* undecided = nullptr);

/// Signed enclosed volume of an ORIENTED shell (divergence theorem over the tessellation).
/// Only its SIGN is consumed. That sign is robust to tessellation error in a way a near-surface
/// point classification is not: probing p +/- eps*n against a chordal mesh answers wrong
/// whenever the mesh sagitta exceeds eps, which is exactly what a curved cavity produces.
double v2_shell_signed_volume(const V2Topo& topo, const V2Shell& shell);

/// Is `inner` inside the enclosure of `outer`? Used for cavity nesting
/// (BuilderSolid.cxx:835-860). The two shells never intersect, so this is a far-field question.
bool v2_shell_inside(const V2Topo& topo, const V2Shell& inner, const V2Shell& outer);

///////////////////////////////////////////////////////////////////////////////////////////
// 7. SELECTION — the BuildBOP op-table (§2.10 / §3.5)
///////////////////////////////////////////////////////////////////////////////////////////

enum class V2Op { Fuse = 0, Common = 1, Cut = 2, Cut21 = 3 };

struct V2OpStates {
    bool objects_in = false;
    bool tools_in = false;
};
V2OpStates v2_op_states(V2Op op);   ///< BOPAlgo_Builder.hxx:214-250

/// Per-solid IN-face map computed ONCE in the split stage (guarantee I-10). Membership is by
/// ARENA FACE INDEX (unoriented).
struct V2InParts {
    std::set<int> faces;
    bool contains(int f) const { return faces.count(f) != 0; }
    void add(int f) { faces.insert(f); }
};

struct V2SelectionInput {
    std::vector<V2OrientedFace> object_faces;   ///< harvested images of A, orientation composed
    std::vector<V2OrientedFace> tool_faces;     ///< harvested images of B
    const V2InParts* in_objects = nullptr;      ///< faces lying IN a solid of A
    const V2InParts* in_tools = nullptr;        ///< faces lying IN a solid of B
};

/// The full 7-step loop of §2.10, transcribed from kb/port_10_builder_solid.md §3.5.
/// Performs ZERO point classifications (I-10): every state is a lookup.
std::vector<V2OrientedFace> v2_select_faces(const V2SelectionInput& in, V2OpStates st,
                                            std::vector<V2AlertRec>& alerts);

///////////////////////////////////////////////////////////////////////////////////////////
// 8. Verdict metric — DEGENERACY AWARE
///////////////////////////////////////////////////////////////////////////////////////////

/// Topological verdict of a finished BRep.
/// `naked` counts only NON-DEGENERATE edges with fewer than two occurrences. A pole/apex edge is
/// a zero-length 3D curve and is excluded; a seam edge is used twice by ONE face and therefore
/// counts 2. Five measurement errors in this campaign came from metrics that counted those as
/// naked.
struct V2Verdict {
    int faces = 0;
    int edges = 0;
    int naked = 0;         ///< non-degenerate edges with < 2 uses
    int nonmanifold = 0;   ///< edges with > 2 uses
    int degenerate = 0;    ///< zero-length / singular edges (excluded from `naked`)
    int seam = 0;          ///< edges used twice by ONE face
    int shells = 0;
    bool closed = false;   ///< naked == 0 && nonmanifold == 0
    std::string str() const;
};
V2Verdict v2_verdict(const BRep& b);

}  // namespace v2sol
}  // namespace session_cpp
