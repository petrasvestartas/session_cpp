#pragma once

#include "nurbssurface.h"
#include "nurbscurve.h"
#include "nurbssurface_trimmed.h"
#include "polyline.h"
#include "mesh.h"
#include "point.h"
#include "xform.h"
#include "color.h"
#include "guid.h"
#include "json.h"
#include <vector>
#include <string>
#include <functional>
#include <map>
#include <array>

namespace session_cpp {

class Plane;
class Line;

enum class BRepTrimType { Boundary = 0, Mated = 1, Seam = 2, Singular = 3 };
enum class BRepLoopType { Outer = 0, Inner = 1 };

struct BRepVertex {
    int point_index = -1;
    std::vector<int> edge_indices;
};

struct BRepEdge {
    int curve_3d_index = -1;
    int start_vertex = -1;
    int end_vertex = -1;
    std::vector<int> trim_indices;
};

struct BRepTrim {
    int curve_2d_index = -1;
    int edge_index = -1;
    int loop_index = -1;
    bool reversed = false;
    BRepTrimType type = BRepTrimType::Boundary;
};

struct BRepLoop {
    std::vector<int> trim_indices;
    int face_index = -1;
    BRepLoopType type = BRepLoopType::Outer;
};

struct BRepFace {
    int surface_index = -1;
    std::vector<int> loop_indices;
    bool reversed = false;
    Color facecolor = Color(0, 0, 0, 0);  // a=0 → not set
};

/**
 * @class BRep
 * @brief Boundary Representation solid model with indexed topology.
 *
 * Stores geometry pools (surfaces, 3D/2D curves, vertices) and topology tables
 * (faces, loops, trims, edges, vertices) that reference into the pools by index.
 * Supports construction from primitives, polylines, or NURBS curves with holes.
 */
class BRep {
public:
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    /// Clear the guid so a FRESH one mints lazily on next read — the duplicate/copy enabler.
    void refresh_guid() { _guid.clear(); }
    std::string name = "my_brep";
    double width = 1.0;
    Color surfacecolor = Color::black();
    Xform xform = Xform::identity();

    // Geometry pools
    std::vector<NurbsSurface> m_surfaces;
    std::vector<NurbsCurve> m_curves_3d;
    std::vector<NurbsCurve> m_curves_2d;
    std::vector<Point> m_vertices;

    // Topology
    std::vector<BRepVertex> m_topology_vertices;
    std::vector<BRepEdge> m_topology_edges;
    std::vector<BRepTrim> m_trims;
    std::vector<BRepLoop> m_loops;
    std::vector<BRepFace> m_faces;

public:
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Static Factory Methods
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Create an axis-aligned box centered at the origin with given half-extents.
    static BRep create_box(double sx, double sy, double sz);

    /// Create a cylinder along +Z with the given radius and height.
    static BRep create_cylinder(double radius, double height);

    /// Create a sphere centered at the origin with the given radius.
    static BRep create_sphere(double radius);

    /// Create a cone along +Z: base circle (radius) at z=0, apex at z=height.
    static BRep create_cone(double radius, double height);

    /// Create a torus centered at the origin in the XY plane (axis +Z).
    static BRep create_torus(double major_radius, double minor_radius);

    /// Create an axis-aligned box with a cylindrical hole through its center.
    static BRep create_block_with_hole(double sx, double sy, double sz, double hole_radius);

    /// Build a BRep from closed planar polylines; each polyline becomes a trimmed planar face.
    static BRep from_polylines(const std::vector<Polyline>& polylines);

    /// Build a BRep from closed NURBS curves with optional hole curves per face.
    static BRep from_nurbscurves(const std::vector<NurbsCurve>& curves, const std::vector<std::vector<NurbsCurve>>& holes = {});

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors & Destructor
    ///////////////////////////////////////////////////////////////////////////////////////////

    BRep();
    BRep(const BRep& other);
    BRep& operator=(const BRep& other);
    bool operator==(const BRep& other) const;
    bool operator!=(const BRep& other) const;
    ~BRep();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Accessors
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Return the number of faces in the BRep.
    int face_count() const;

    /// Return the number of topological edges.
    int edge_count() const;

    /// Return the number of topological vertices.
    int vertex_count() const;

    /// Return true if the BRep has valid topology and non-empty geometry pools.
    bool is_valid() const;

    /// Return true if the BRep forms a closed (watertight) solid.
    bool is_solid() const;

    /// OCCT-BRepCheck-style topology audit that is_solid() does not perform: counts naked
    /// (1-trim) and non-manifold (>2-trim) edges, duplicate vertices/edges, connected shell
    /// components (union-find over 2-trim adjacency), and the Euler characteristic V-E+F per
    /// shell (2-2g for a closed orientable shell). Returns a one-line-per-metric report; the
    /// bool out-param `valid_manifold` is true iff 0 naked, 0 non-manifold, and every shell is
    /// closed (Euler even). Diagnostic only -- does not mutate.
    std::string topology_report(bool* valid_manifold = nullptr) const;

    /// Validate the trim-orientation invariant (OCCT contract): within each loop the trims,
    /// traversed with their `reversed` flags applied, chain head-to-tail and close; where the
    /// loop does not jump a periodic seam, its signed UV area matches its type (outer CCW,
    /// inner CW). Returns the number of violations; prints them when `verbose`.
    int check_trim_orientation(bool verbose = false) const;

    /// Volume enclosed by the (closed) BRep via the divergence theorem over the
    /// tessellated boundary (signed tetrahedra). Exact for planar-faced solids;
    /// converges with mesh resolution for curved faces.
    double volume() const;

    /// True if `p` is strictly inside the (closed) solid, by ray-cast parity against the
    /// tessellated boundary. Robust for interior points (boolean fragment classification);
    /// matches OCCT BRepClass3d_SolidClassifier for IN/OUT.
    bool contains_point(const Point& p) const;

    /// As contains_point but reusing a precomputed boundary mesh (avoids re-tessellation
    /// when classifying many points against the same solid).
    bool contains_point(const Mesh& boundary, const Point& p) const;

    /// EXACT point-in-solid, mesh-free: find the closest point on the TRIMMED boundary
    /// (nearest surface point that lies inside its face's trim loops, over all faces) and
    /// return true iff `p` is on the inner side of that face's outward normal. Immune to the
    /// tessellation over/under-coverage that makes the winding-number/ray classifiers on a
    /// self-overlapping imported mesh answer wrong (rotated freeform chairs). `osign` is the
    /// per-face outward sign from face_outward_signs() (pass empty to compute internally).
    bool contains_point_exact(const Point& p, const std::vector<double>& osign) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Building
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Add a NURBS surface to the geometry pool. Returns the surface index.
    int add_surface(const NurbsSurface& srf);

    /// Add a 3D NURBS curve to the geometry pool. Returns the curve index.
    int add_curve_3d(const NurbsCurve& crv);

    /// Add a 2D trim curve to the geometry pool. Returns the curve index.
    int add_curve_2d(const NurbsCurve& crv);

    /// Add a vertex point to the geometry pool. Returns the vertex index.
    int add_vertex(const Point& pt);

    /// Add a topological edge referencing a 3D curve and two endpoint vertices.
    int add_edge(int curve_3d_idx, int start_vertex, int end_vertex);

    /// Add a trim referencing a 2D curve, its parent edge and loop, with orientation.
    int add_trim(int curve_2d_idx, int edge_idx, int loop_idx, bool reversed, BRepTrimType type);

    /// Add a loop (outer or inner) belonging to a face. Returns the loop index.
    int add_loop(int face_idx, BRepLoopType type);

    /// Add a face referencing a surface, with optional reversed orientation.
    int add_face(int surface_idx, bool reversed);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Splitting
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Split this BRep by a plane. Returns a new subdivided BRep.
    BRep split_by_plane(const Plane& plane, double tolerance = 0.0) const;

    /// Split this BRep by another surface. Returns a new subdivided BRep.
    BRep split_by_surface(const NurbsSurface& cutter, double tolerance = 0.0) const;

    /// Split this BRep by 3D curves pulled onto each face. New BRep.
    BRep split_by_curves(const std::vector<NurbsCurve>& curves, double tolerance = 0.0) const;

    /// Split this BRep by a line pulled onto each face. New BRep.
    BRep split_by_line(const Line& line, double tolerance = 0.0) const;

    /// Split this BRep by every face of another BRep. New BRep. imported_freeform enables
    /// the near-boundary cut-endpoint snap in the UV arrangement (freeform x freeform pairs).
    /// pre_cuts (optional): per-surface-index cut pcurves precomputed by the caller (one SSI
    /// per surface PAIR shared by both operands), bypassing the per-operand SSI here.
    /// scaf/scaf_is_A/sec_edges_out (optional, OCCT-adoption S2): cut with the SHARED section
    /// scaffold instead of per-operand SSI -- section edges lift from the scaffold's 3D chain
    /// (identical geometry on both operands by construction) and sec_edges_out reports
    /// result-edge-index -> {seg_id, v_start, v_end} so BRep::boolean can merge the two
    /// operands' copies into ONE edge at combine (no sewing of section edges).
    BRep split_by_brep(const BRep& cutter, double tolerance = 0.0, bool imported_freeform = false,
                       const std::vector<std::vector<NurbsCurve>>* pre_cuts = nullptr,
                       const struct SectionScaffold* scaf = nullptr, bool scaf_is_A = true,
                       std::map<int, std::array<int, 3>>* sec_edges_out = nullptr,
                       std::vector<int>* face_src_out = nullptr,
                       const std::vector<std::vector<NurbsCurve>>* extra_cuts = nullptr,
                       std::map<int, std::array<double, 3>>* sec_spans_out = nullptr,
                       const struct SharedEdgePool* pool = nullptr) const;

    /// OCCT pave-block normalization of scaffold section edges. spans: edge index ->
    /// {seg_id, fa, fb} (fractional chain-index range of the edge on its segment, recorded
    /// by split_by_brep for every chain-lifted run). Per segment, the union of all edges'
    /// range ends forms the pave set; every edge is split at interior paves (3D re-extracted
    /// from the shared chain, so pieces are bit-identical across copies) and same-block
    /// copies are merged into one edge (capped at 2 trims). This is the BOPDS common-block
    /// analog: partial runs whose clip params disagree across flanks/operands become
    /// identical per-block edges instead of relying on Hausdorff sewing. Updates spans and
    /// sec_edges (whole-segment keys re-validated, block edges keyed) in place. Returns the
    /// number of edges merged away.
    int normalize_section_blocks(const struct SectionScaffold& scaf,
                                 std::map<int, std::array<double, 3>>& spans,
                                 std::map<int, std::array<int, 3>>* sec_edges,
                                 const char* tag,
                                 const std::map<int, std::vector<double>>* shared_centers = nullptr);

    /// Recover span records for LEGACY-lifted section edges: an under-mated (< 2-trim) edge whose
    /// samples all lie on a scaffold segment's shared chain but which carries no {seg_id,fa,fb}
    /// span (it was dropped to a straight-chord / boundary edge by the operand's arrangement) is
    /// matched geometrically to that segment and given a span, so normalize_section_blocks pulls it
    /// into the identity-based pave-block merge instead of leaving it for Hausdorff sewing (which
    /// fails on staggered copies). Returns the number of spans recovered. Adds to `spans` in place.
    int recover_section_spans(const struct SectionScaffold& scaf,
                              std::map<int, std::array<double, 3>>& spans);

    /// Build a standalone BRep from a subset of this BRep's faces. edge_remap (optional):
    /// old topology-edge index -> new index for edges carried into the subset.
    BRep subset(const std::vector<int>& face_indices,
                std::map<int, int>* edge_remap = nullptr) const;

    /// Concatenate another BRep's faces/topology into this one (index-offset copy, no sewing).
    void append_brep(const BRep& other);

    /// Split by a plane and separate into the pieces on each side. One BRep per side.
    std::vector<BRep> split_by_plane_pieces(const Plane& plane, double tolerance = 0.0) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Booleans
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Boolean operation kind.
    enum class BooleanOp { Union, Difference, Intersection };

    /// Boolean of two solids via imprint -> classify -> select -> sew.
    /// Imprints both solids against each other (split_by_brep), classifies each face
    /// fragment as inside/outside the other solid (ray-cast parity), selects the fragments
    /// for the requested operation, and sews them (shared imprint edges merged by position).
    /// Scoped to the kernel's planar + quadric solids; matches OCCT BRepAlgoAPI_* on
    /// volume/face-count for those cases.
    BRep boolean(const BRep& other, BooleanOp op, double tolerance = 0.0) const;

    /// Imprint T-junctions: split any under-mated edge at interior points that coincide with
    /// another edge's endpoint vertex, so a long edge that spans several shorter coincident
    /// edges is broken to match them. Splits the edge's 3D curve and each trim's 2D pcurve,
    /// updating the owning loops. Run before sew_coincident_edges to resolve mismatched
    /// boundary segmentation between adjacent faces. Modifies in place.
    void imprint_edges(double tol = 0.0, bool mated_too = false);

    /// Merge edges whose 3D curves coincide (Hausdorff < tol) into single mated edges, and
    /// weld vertices within tol. Repairs a non-watertight BRep (e.g. after split/boolean) so
    /// that shared edges carry two trims. Modifies in place.
    void sew_coincident_edges(double tol = 0.0);

    /// Co-refine coincident section edges so they mate 1:1 before sewing. The A∩B section curve
    /// is imprinted independently on each operand, so one side may be a single CLOSED circle while
    /// the other is 2+ arcs (a periodic seam straddle), or two partially-overlapping arcs. This
    /// splits each under-mated edge at the endpoints of any DISTINCT under-mated edge that is
    /// coincident with (an arc of) it -- gated strictly on coincidence so non-shared edges are
    /// never spuriously split. Splits the 3D curve and each trim's 2D pcurve (exact, no refit),
    /// updates the owning loops. Run before sew_coincident_edges. Modifies in place.
    void co_refine_coincident_edges(double tol = 0.0);

    /// Snap under-mated section edges to the exact pair-once section curves: an edge whose BOTH
    /// endpoints lie on a section c3d (and whose body stays within a band of it) has its 3D
    /// curve replaced by the exact sub-arc — the two operands' copies of a section then carry
    /// identical geometry and sew mates them. Run between combine and co_refine. In place.
    void snap_section_edges(const std::vector<NurbsCurve>& sections, double tol = 0.0);

    /// BUILDSPEC P0 — shared section-edge backbone. The A&B section curve is computed ONCE
    /// (Intersection::surface_surface on the ORIGINAL operand surfaces A and B), and each
    /// under-mated section edge in this (already-combined) BRep is re-fitted to the EXACT
    /// sub-arc of that single section curve over its own endpoint span -- so the box-side and
    /// sphere-side arcs of a section reference identical exact geometry. The matching split
    /// vertices come from a single co-refine pass so the two operands agree on segment count;
    /// the trivially-coincident exact arcs are then merged into one shared edge (two trims,
    /// one per operand). Replaces co_refine_coincident_edges + the A&B branch of
    /// sew_coincident_edges when gated by SESSION_BOOL_SHARED_EDGES. Modifies in place.
    void make_shared_section_edges(const BRep& A, const BRep& B, double tol = 0.0);

    /// Outward-orientation sign per face: +1 when the surface's natural normal points OUT of
    /// the solid, -1 when it points in. Shell-orientation propagation: relative parity across
    /// every 2-trim edge (trim traversals normalized to material-left geometrically), one
    /// weighted-evidence flip per connected shell. Consumed by volume() and the STEP writer's
    /// ADVANCED_FACE same_sense flags. Optionally returns each face's interior point + normal.
    std::vector<double> face_outward_signs(std::vector<Point>* P3s = nullptr,
                                           std::vector<Vector>* Ns = nullptr) const;

    /// Does loop `li` traverse with face material on its LEFT in UV (majority over its trims,
    /// measured geometrically against the face's own region)? Stored trim conventions are not
    /// uniform across construction paths; consumers that need a definite winding (the STEP
    /// writer) measure it here instead of trusting flags.
    /// `dirs` (optional): per-trim traversal override as (trim_index, from_first) pairs --
    /// used by the STEP writer, whose loops are re-CHAINED head-to-tail and no longer follow
    /// the stored directions.
    bool loop_material_left(int li,
                            const std::vector<std::pair<int,char>>* dirs = nullptr) const;

    /// OCCT SameParameter-lite: for every 2-trim mixed (planar/curved) section edge, rebuild the
    /// PLANAR trim's pcurve from the edge's shared 3D curve by exact affine projection, so both
    /// faces integrate identical boundary geometry (the volume error is first-order in any
    /// per-face copy mismatch). Run after sew_coincident_edges. Modifies in place.
    void sameparameter_planar_pcurves();

    /// Rhino-style unify-same-domain for PLANAR faces: faces that are coplanar and share an
    /// edge merge into ONE face -- the shared edges vanish and the surviving trims re-chain
    /// into merged loops in a single chart. OCCT keeps these imprinted splits by default
    /// (its ShapeUpgrade_UnifySameDomain is the equivalent post-step), so the boolean gates
    /// compare RAW results; call this before presenting/exporting. Modifies in place.
    void merge_coplanar_faces(double tolerance = 0.0);

    /// A | B (fuse).
    BRep boolean_union(const BRep& other, double tolerance = 0.0) const {
        return boolean(other, BooleanOp::Union, tolerance);
    }
    /// A - B (cut).
    BRep boolean_difference(const BRep& other, double tolerance = 0.0) const {
        return boolean(other, BooleanOp::Difference, tolerance);
    }
    /// A & B (common).
    BRep boolean_intersection(const BRep& other, double tolerance = 0.0) const {
        return boolean(other, BooleanOp::Intersection, tolerance);
    }
    /// A ^ B (symmetric difference): (A - B) assembled with (B - A). The two cuts have
    /// DISJOINT interiors by construction and meet only along the shared section (measure
    /// zero), so xor is a disjoint assembly, never a second boolean: running the general
    /// union on boolean RESULTS stressed the same-domain machinery (chart-extent phantom
    /// cuts between result faces sharing surfaces) and shattered the output.
    BRep boolean_xor(const BRep& other, double tolerance = 0.0) const {
        BRep ab = boolean_difference(other, tolerance);
        BRep ba = other.boolean_difference(*this, tolerance);
        if (ab.face_count() == 0) return ba;    // A inside B -> xor = B - A
        if (ba.face_count() == 0) return ab;    // B inside A -> xor = A - B
        ab.append_brep(ba);
        return ab;
    }
    /// Split A by B (OCCT BOPAlgo_SPLITTER analogue for two solids): all three volume
    /// fragments {A - B, A & B, B - A}, empty fragments omitted. Their volumes partition
    /// fuse(A, B) exactly.
    std::vector<BRep> boolean_split(const BRep& other, double tolerance = 0.0) const {
        std::vector<BRep> out;
        for (BRep r : {boolean_difference(other, tolerance),
                       boolean_intersection(other, tolerance),
                       other.boolean_difference(*this, tolerance)})
            if (r.face_count() > 0) out.push_back(std::move(r));
        return out;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Meshing
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Generate a combined triangle mesh from all faces, respecting trim loops.
    Mesh mesh() const;
    std::vector<Mesh> face_meshes() const;
    std::vector<Mesh> face_meshes_q(bool has_quality, double max_angle_deg, double chord_factor) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Evaluation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Evaluate a 3D point on the given face at parameters (u, v).
    Point point_at(int face_idx, double u, double v) const;

    /// Evaluate the surface normal on the given face at parameters (u, v).
    Vector normal_at(int face_idx, double u, double v) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Apply the stored xform to all geometry and reset xform to identity.
    void transform();

    /// Return a copy with the stored xform applied to all geometry.
    BRep transformed() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to JSON object with fields in alphabetical order.
    nlohmann::ordered_json jsondump() const;

    /// Construct from a JSON object.
    static BRep jsonload(const nlohmann::json& data);

    /// Write JSON to a file.
    void file_json_dump(const std::string& filename) const;

    /// Read from a JSON file.
    static BRep file_json_load(const std::string& filename);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static BRep file_json_loads(const std::string& json_string);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serialize to a protobuf binary string.
    std::string pb_dumps() const;

    /// Deserialize from a protobuf binary string.
    static BRep pb_loads(const std::string& data);

    /// Write protobuf to a file.
    void pb_dump(const std::string& filename) const;

    /// Read from a protobuf file.
    static BRep pb_load(const std::string& filename);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // String Representation
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Simple string (type, face/edge/vertex counts).
    std::string str() const;

    /// Detailed string with topology and geometry pool sizes.
    std::string repr() const;

    /// Stream output operator (calls str()).
    friend std::ostream& operator<<(std::ostream& os, const BRep& brep);

private:
    mutable std::string _guid;

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Internal Helpers
    ///////////////////////////////////////////////////////////////////////////////////////////

    void deep_copy_from(const BRep& src);

    /// Shared splitter: subdivide each face by per-face cut pcurves, rebuild a new BRep.
    /// scaf/scaf_is_A/sec_edges_out: see split_by_brep (OCCT-adoption S2 scaffold path).
    /// face_src_out (optional): original face index for each result face -- lets the
    /// classifier inherit the operand's outward orientation per fragment (angle method).
    BRep split_with(double tolerance, const std::function<std::vector<NurbsCurve>(const NurbsSurface&)>& cut_for, bool imported_freeform = false,
                    const struct SectionScaffold* scaf = nullptr, bool scaf_is_A = true,
                    std::map<int, std::array<int, 3>>* sec_edges_out = nullptr,
                    std::vector<int>* face_src_out = nullptr,
                    const std::vector<std::vector<NurbsCurve>>* extra_cuts = nullptr,
                    std::map<int, std::array<double, 3>>* sec_spans_out = nullptr,
                    const struct SharedEdgePool* pool = nullptr) const;
};

} // namespace session_cpp
