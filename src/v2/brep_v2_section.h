#pragma once

// V2 FACE/FACE SECTION STAGE — stage 6+7 of the v2 boolean pipeline.
//
// Port target: kb/port_04_ff_section.md (OCCT IntTools_FaceFace + BOPAlgo_PaveFiller::MakeBlocks
// + PostTreatFF) and kb/port_08_seam_pole_periodic.md §2.8 (seam crossings).
// NEW MODULE. It reads BReps and the BdsArena, it never mutates a BRep, and nothing in the
// existing v1 kernel calls it.
//
// WHAT IT DOES, in the four levels OCCT uses (kb/port_04_ff_section.md §2.0):
//
//   L1  surface x surface        -> the EXISTING analytic SSI (Intersection::surface_surface).
//                                   Never reimplemented here: those recognisers are exact and
//                                   rigid-motion equivariant.
//   L2  footprints               -> for every section curve, a PARAMETER-SYNCHRONISED trail of
//                                   V2PntOn2S (one 3D point, both UV footprints), built by
//                                   seeded continuation so a periodic direction is UNWRAPPED
//                                   (kb/port_08 S10: relative unwrap, never absolute fmod).
//   L3  wires                    -> paves at REAL ENTITIES ONLY: 3D intersections of the section
//                                   curve with the faces' boundary/seam EDGES, plus the curve's
//                                   own bounds and the closing pave of a closed curve. Each
//                                   interval between consecutive paves is kept or dropped WHOLE
//                                   on one 43.2%-point classification against both faces' wires
//                                   (V2_PAR_T, OCCT IntTools_Tools::IntermediatePoint). There is
//                                   no bisection of a classification predicate anywhere in this
//                                   file, so kb/port_04 G3 is structural.
//   L4  arena                    -> every surviving interval becomes a BdsPaveBlock on a carrier
//                                   edge in the BdsArena, and (post_treat_ff) coincident blocks
//                                   from DIFFERENT face pairs become ONE BdsCommonBlock naming
//                                   ONE arena edge, while coincident block endpoints become ONE
//                                   arena vertex through BdsArena::fuse_vertices.
//
// TOLERANCE RULE. Every tolerance here is a MODEL-SPACE distance (BdsArena invariant I5).
// Where a UV band is needed it is derived at the point of use through the surface metric, and
// the ON band of the wire classifier is measured as a 3D distance to the 3D image of the wire.
// No constant in this file is a function of range(u) or range(v) (kb/port_04 G6).
//
// CURVE TOLERANCE IS MEASURED, NEVER ASSIGNED (kb/port_04 G8): V2Curve::dev1/dev2 are the
// sampled max deviation of the 3D curve from each surface, and V2Curve::tol is derived from
// them.

#include "brep.h"
#include "brep_bds.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "point.h"

#include <memory>
#include <string>
#include <vector>

namespace session_cpp {
namespace v2sec {

///////////////////////////////////////////////////////////////////////////////////////////
// 0. Typed outcomes and scalars
///////////////////////////////////////////////////////////////////////////////////////////

/// kb/port_04 G9: a tangency is NEVER reported as "Ok with 0 curves".
enum class V2FFStatus : int {
    Ok = 0,                   ///< at least one section curve was produced
    Tangent = 1,              ///< the two surfaces touch tangentially over an area/line
    Empty = 2,                ///< the faces' boxes do not overlap: provably no section
    NoGeometricSolution = 3,  ///< boxes overlap but the SSI returned nothing usable
    Failed = 4                ///< bad input (invalid face / surface)
};

enum class V2State : int { In = 0, On = 1, Out = 2 };

/// Provenance of a pave. kb/port_04 G3/T9: there is deliberately no `Bisection` member.
enum class V2PaveOrigin : int {
    CurveBound = 0,     ///< a bound of the section curve
    Closing = 1,        ///< the closing pave of a closed section curve
    TrimCrossing = 2,   ///< 3D intersection with a boundary/mated edge of one of the faces
    SeamCrossing = 3,   ///< 3D intersection with a SEAM edge of a periodic face
    PoleTouch = 4,      ///< the curve reaches a singular (pole/apex) trim
    Fused = 5           ///< reserved; see V2Pave::fused -- origin is never overwritten
};

const char* v2_status_name(V2FFStatus s);
const char* v2_origin_name(V2PaveOrigin o);

/// SESSION_V2_SECDBG=1 — per-curve seam/pave diagnostics on stdout. Gated on (p && p[0]).
bool v2sec_dbg();

/// OCCT IntTools_Tools::IntermediatePoint, IntTools_Tools.cxx:254-259.
constexpr double V2_PAR_T = 0.43213918;
inline double v2_intermediate_point(double a, double b) {
    return (1.0 - V2_PAR_T) * a + V2_PAR_T * b;
}

///////////////////////////////////////////////////////////////////////////////////////////
// 1. A trimmed face, and the wire predicates
///////////////////////////////////////////////////////////////////////////////////////////

struct V2FaceRef {
    const BRep* brep = nullptr;
    int face = -1;
    bool valid() const;
    const NurbsSurface& surface() const;
};

/// The face's UV window: the surface's natural domain, plus the period of every closed
/// direction. kb/port_08 S12: for a seamed face this IS the natural domain.
struct V2UvRect {
    double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
    bool u_closed = false, v_closed = false;
    double u_period = 0, v_period = 0;
};
V2UvRect v2_uv_rect(const V2FaceRef& f);

/// Seeded closest-point inversion of `p` onto `s`. (u,v) is seed on entry, footprint on exit.
/// `seeded == false` runs a grid pre-search first. Returns the 3D distance, or -1 on failure.
/// Closed directions are wrapped, open ones clamped — so the answer is always inside the
/// surface's own rectangle and the returned distance is the honest distance to the PATCH.
double v2_invert(const NurbsSurface& s, const Point& p, double& u, double& v, bool seeded);

/// Per-face classification state: the face's wires chained into closed UV polygons together
/// with their 3D images, plus a seed grid for point inversion. Built ONCE per face; the whole
/// wire logic of L3 goes through it. OCCT analog: IntTools_Context's cached IntTools_FClass2d
/// and ProjPS per face (IntTools_Context.cxx:225-265).
class V2FaceClassifier {
public:
    explicit V2FaceClassifier(const V2FaceRef& f);
    const V2FaceRef& face() const { return m_f; }
    const V2UvRect& rect() const { return m_rect; }
    /// IntTools_FClass2d::Perform. `tol3d` is a MODEL-SPACE band.
    V2State state(double u, double v, double tol3d) const;
    bool in_on(double u, double v, double tol3d) const { return state(u, v, tol3d) != V2State::Out; }
    /// IntTools_Context::IsValidPointForFace: invert, reject on distance, then classify.
    bool valid_point(const Point& p, double tol3d, double* u_out = nullptr,
                     double* v_out = nullptr) const;
    /// Seeded inversion using this face's cached grid. Returns the 3D distance (-1 on failure).
    double invert(const Point& p, double& u, double& v, bool seeded) const;
    int loop_count() const { return (int)m_loops.size(); }
    bool ok() const { return m_ok; }

private:
    struct Poly {
        std::vector<double> u, v;
        std::vector<Point> p3;
    };
    V2FaceRef m_f;
    const NurbsSurface* m_s = nullptr;
    V2UvRect m_rect;
    std::vector<Poly> m_loops;
    std::vector<Point> m_grid;
    int m_gnu = 0, m_gnv = 0;
    bool m_ok = false;
};

/// OCCT IntTools_FClass2d::Perform / IntTools_Context::StatePointFace. Even-odd crossing over
/// every loop of the face, with an ON band measured as a 3D distance to the 3D image of the
/// wire (never a parameter epsilon).
V2State v2_state_point_face(const V2FaceRef& f, double u, double v, double tol3d);
/// IntTools_Context::IsPointInOnFace — ON counts as IN (IntTools_Context.cxx:639-643).
bool v2_is_point_in_on_face(const V2FaceRef& f, double u, double v, double tol3d);
/// IntTools_Context::IsValidPointForFace (IntTools_Context.cxx:647-673): project, reject when
/// the projection distance exceeds `tol3d`, then classify the footprint.
bool v2_is_valid_point_for_face(const V2FaceRef& f, const Point& p, double tol3d,
                                double* u_out = nullptr, double* v_out = nullptr);
bool v2_is_valid_point_for_faces(const V2FaceRef& a, const V2FaceRef& b, const Point& p,
                                 double tol3d);

/// Exact 3D intersections between two curves: adaptive tessellation to bracket, then a 2x2
/// Newton on ((A-B).A', (A-B).B') = 0. Accepted when the residual distance is <= tol.
/// This is the ENTITY behind every trim/seam pave (kb/port_04 G3).
std::vector<std::pair<double, double>> v2_curve_curve_points(const NurbsCurve& a,
                                                            const NurbsCurve& b, double tol);

/// Arclength of `c` over [t0,t1] by composite 5-point Gauss-Legendre on |C'(t)|.
double v2sec_arclen(const NurbsCurve& c, double t0, double t1);

///////////////////////////////////////////////////////////////////////////////////////////
// 2. Section curves, paves, blocks
///////////////////////////////////////////////////////////////////////////////////////////

/// One sample carrying BOTH parametric footprints for ONE 3D point (OCCT IntSurf_PntOn2S).
/// The only representation in which the two operands cannot disagree about where the section
/// is. u1/v1 are UNWRAPPED along the trail (kb/port_08 S10).
struct V2PntOn2S {
    double t = 0;
    Point p;
    double u1 = 0, v1 = 0, u2 = 0, v2 = 0;
};

struct V2Pave {
    double t = 0;
    int vertex = -1;                                   ///< arena vertex index
    /// PROVENANCE. Never overwritten: post_treat_ff sets `fused` instead, so the entity that
    /// created the pave stays visible after the node it names has been merged (kb/port_04 G3).
    V2PaveOrigin origin = V2PaveOrigin::CurveBound;
    bool fused = false;                                ///< vertex was merged by post_treat_ff
    int face = -1;                                     ///< arena face that produced it, or -1
    int edge = -1;                                     ///< arena/BRep edge that produced it, or -1
};

struct V2Block {
    double t0 = 0, t1 = 0;
    bool kept = false;
    int edge = -1;             ///< arena index of the minted section edge (-1 when dropped)
    int v0 = -1, v1 = -1;      ///< arena vertex indices of the two ends
    double length = 0;         ///< 3D length
    BdsPB pb;                  ///< the arena pave block (never null for a produced block)
};

struct V2Curve {
    int face1 = -1, face2 = -1;      ///< ARENA face indices
    int pair = -1;                   ///< sequential index of the face pair that produced it
    std::shared_ptr<NurbsCurve> c3d;
    std::vector<V2PntOn2S> trail;
    bool closed = false;
    double dev1 = 0, dev2 = 0;       ///< MEASURED max |c3d(t) - S_i(...)| over the KEPT blocks (G8)
    double dev_all = 0;              ///< same, over the WHOLE analytic carrier: diagnostic only
    double tol = 0;                  ///< max(dev1, dev2, tolF1, tolF2) + fuzz
    double tang_tol = 0;
    int carrier_edge = -1;           ///< arena edge carrying the WHOLE section curve
    std::vector<V2Pave> paves;       ///< sorted by t, unique
    std::vector<V2Block> blocks;     ///< tiles [t0,t1] exactly; `kept` selects the emitted ones
    int seam_paves = 0;
};

struct V2SectionParams {
    double fuzzy = 1e-7;        ///< OCCT myFuzzyValue floor; a MODEL-SPACE length
    int trail_samples = 192;    ///< initial footprint trail resolution per section curve
    bool seam_split = true;     ///< insert paves where the curve crosses a seam edge (S8)
    bool posttreat = true;      ///< run the cross-pair fusion
    bool verbose = false;

    // ADAPTIVE TRAIL (see V2Section::build_trail). The trail's UV footprints are what the
    // caller imprints on the face as a POLYGON, so the face boundary carries exactly the
    // sagitta of that polygon against the true section curve. A uniform sample count cannot
    // bound it: MEASURED on box x cone, the boolean is topologically perfect and volume-exact
    // at every pose but its closure residual sits at 1.5e-09..7.5e-09 against the 1e-09 gate,
    // and the residual falls as 1/n^2 in the TRAIL count (not in the imprint count). So the
    // trail is refined until the MEASURED sagitta is below trail_sag_rel * (model scale).
    double trail_sag_rel = 2e-10;   ///< target sagitta / operand-box diagonal; <=0 disables
    int trail_refine_passes = 10;   ///< bisection passes
    int trail_max_samples = 24000;  ///< hard cap on one curve's trail
};

struct V2SectionStats {
    int pairs = 0, ok = 0, tangent = 0, empty = 0, nogeom = 0, failed = 0;
    int curves = 0, blocks = 0, kept = 0, dropped = 0;
    int trim_paves = 0, seam_paves = 0, bound_paves = 0;
    int section_edges = 0;      ///< distinct arena edges after fusion
    int fused_groups = 0;       ///< vertex clusters collapsed by post_treat_ff
    int common_blocks = 0;      ///< BdsCommonBlocks built by post_treat_ff
    double max_dev = 0;         ///< worst measured curve/surface deviation over all curves
};

///////////////////////////////////////////////////////////////////////////////////////////
// 3. The stage
///////////////////////////////////////////////////////////////////////////////////////////

class V2Section {
public:
    /// `ds` must already have been init()ed with the SAME operand list, in the same order.
    V2Section(BdsArena& ds, const std::vector<const BRep*>& operands,
              const V2SectionParams& prm = V2SectionParams());

    /// One face pair, by ARENA face index. Appends to curves() on Ok.
    V2FFStatus perform_pair(int arena_face1, int arena_face2);
    /// Every cross-operand face pair whose boxes overlap. Returns the number of Ok pairs.
    int perform_all();
    /// Cross-pair fusion (OCCT PostTreatFF): one arena vertex per geometric node, one arena
    /// edge per geometric curve. Degrades to "no fusion", never to "no sections".
    void post_treat_ff();

    const std::vector<V2Curve>& curves() const { return m_curves; }
    const V2SectionStats& stats() const { return m_stats; }
    V2FaceRef face_ref(int arena_face) const;

    /// Distinct arena edges of all kept blocks, canonicalised through common blocks. Sorted.
    std::vector<int> section_edges() const;
    /// Distinct arena vertices at the ends of all kept blocks, resolved through SD. Sorted.
    std::vector<int> section_nodes() const;
    /// Distinct 3D locations of those nodes at `tol`. G4 holds iff this equals nodes().size().
    int distinct_node_locations(double tol) const;
    /// Connected components of the kept blocks, joined through shared node entities.
    int component_count() const;
    double kept_length() const;

    /// kb/port_04 G1: max over all kept blocks of max(d(c(t),S1), d(c(t),S2)).
    double g1_residual() const;
    /// Canonical, reproducible dump (G11 tripwire).
    std::string signature() const;

private:
    V2FFStatus section_for_pair(int f1, int f2, std::vector<V2Curve>& out);
    void build_trail(V2Curve& c, const V2FaceRef& r1, const V2FaceRef& r2);
    void collect_paves(V2Curve& c, const V2FaceRef& r1, const V2FaceRef& r2);
    void make_blocks(V2Curve& c, const V2FaceRef& r1, const V2FaceRef& r2);
    void measure_tolerance(V2Curve& c);

    const V2FaceClassifier& classifier(int arena_face) const;

    BdsArena& m_ds;
    std::vector<const BRep*> m_operands;
    V2SectionParams m_prm;
    std::vector<V2Curve> m_curves;
    V2SectionStats m_stats;
    std::vector<std::vector<int>> m_fused;   ///< the clusters post_treat_ff collapsed
    mutable std::vector<std::shared_ptr<V2FaceClassifier>> m_fc;   ///< by arena index
};

}  // namespace v2sec
}  // namespace session_cpp
