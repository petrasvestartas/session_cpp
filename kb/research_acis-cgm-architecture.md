# Dassault Spatial ACIS + CGM — boolean/modeling architecture (public-source deep dive)

Research date: 2026-07-24. Sources: ACIS R10/R17 documentation mirrors (University of Arizona ISL mirror, q-solid.com R17 mirror), Spatial's own blog (incremental boolean workflow, healing, CGM), CAA V5 CGM documentation mirror (maruf.ca), patents, secondary histories. All claims cited. Written for the session_cpp boolean-kernel program (OCCT-aligned architecture; master plan P3 same-domain, P4 EE/EF interference, P5 tolerance model, P6 corpus validation). Companion file: `research_parasolid-architecture.md`.

Primary sources:
- ACIS R10 docs mirror (Arizona ISL): Booleans component TOC http://www-isl.ece.arizona.edu/ACIS-docs/HTM/SELECT/SUB/SB27_99.HTM ; Boolean Operations http://www-isl.ece.arizona.edu/ACIS-docs/HTM/DATA/BOOL/BOOL/01CMP/0001.HTM ; Intersection Graph …/0002.HTM ; Imprint …/0004.HTM ; Regularized/Nonregularized …/0005.HTM and …/0008.HTM ; Partial Booleans …/0010.HTM ; Tolerant Modeling chapter http://www-isl.ece.arizona.edu/ACIS-docs/HTM/DATA/KERN/KERN/06TMOD/0001.HTM (+0002 tolerant edges, 0003 tolerant vertices, 0004 options, 0005 applications) ; Cellular Topology component PDF http://www-isl.ece.arizona.edu/ACIS-docs/PDF/CT/01CMP.PDF
- ACIS R17 docs mirror (q-solid, http only): Tolerance Variables http://www.q-solid.com/ACIS_Docs_R17/online/SPAacisuserTechArticles/SPAacisuser_totol.htm ; Boolean Operations on Graphs http://www.q-solid.com/ACIS_Docs_R17/online/SPAacisuserTechArticles/SPAacisuser_grbool.htm ; glue_options http://www.q-solid.com/ACIS_Docs_R17/online/refman/generated/refman/SPAbool/class_glue_options.htm ; api_complete_intersection_graph http://www.q-solid.com/ACIS_Docs_R17/online/refman/generated/refman/SPAbool/function_api_complete_intersection_graph.htm ; api_detect_sliver_faces …/function_api_detect_sliver_faces.htm ; api_detect_short_edges …/function_api_detect_short_edges.htm ; api_fixup_intersection …/function_api_fixup_intersection.htm ; Tolerant Stitching http://www.q-solid.com/ACIS_Docs_R17/online/SPAacisuserTechArticles/SPAacisuser_mointrstitchto.htm
- Spatial blog: incremental boolean workflow https://blog.spatial.com/what-to-do-when-your-3d-modeling-boolean-operations-fail ; healing in interop https://blog.spatial.com/healing-in-3d-interoperability ; ACIS translated-data healing https://blog.spatial.com/acis-modeler-3d-translated-data ; CGM gap healing https://blog.spatial.com/cgm/healing-gaps-cgm ; CGM launch https://blog.spatial.com/news/cgm ; CGM product page https://www.spatial.com/solutions/3d-modeling/cgm-modeler
- CAA V5 CGM docs mirror (maruf.ca): Geometric objects (units/resolution/model size) https://www.maruf.ca/files/caadoc/CAAGobTechArticles/GeoObjects.htm ; Topological model https://www.maruf.ca/files/caadoc/CAATobTechArticles/TopoModel.htm ; Topology↔geometry association https://www.maruf.ca/files/caadoc/CAATobTechArticles/TopoCreate.htm ; Boolean operators https://www.maruf.ca/files/caadoc/CAATopTechArticles/TopoBoolean.htm ; Topological operators overview https://www.maruf.ca/files/caadoc/CAATopUseCases/CAATopOverview.htm ; Journal methodology https://www.maruf.ca/files/caadoc/CAATopTechArticles/JournalMethodology.htm ; CGM data checker https://www.maruf.ca/files/caadoc/CAACgmTechArticles/CAACgmDataChecker.htm ; Curves https://www.maruf.ca/files/caadoc/CAAGobTechArticles/Curves.htm
- Patents (selective/cellular booleans, Spatial-school): US7330771 emboss/engrave via ct-graph selective booleans https://patents.google.com/patent/US7330771 ; US7031790 sculpting with sheet bodies https://patents.google.com/patent/US7031790
- Secondary: engineering.com on ACIS vs CGM https://www.engineering.com/spatial-acis-cgm-and-the-future-of-geometric-modeling-kernels/ ; novedge boolean-robustness history https://novedge.com/blogs/design-news/design-software-history-boolean-modeling-in-cad-csg-origins-b-rep-breaking-points-and-robustness-solutions

---

## 1. Context

Two kernels, one company. **ACIS** (1989, Three-Space Ltd → Spatial, acquired by Dassault Systèmes 2000) is the classic open-architecture C++ B-Rep kernel (AutoCAD/Inventor heritage via a fork, HFSS, KeyCreator, many others). **CGM** ("Convergence Geometric Modeler") is the CATIA V5/V6/3DEXPERIENCE kernel, developed inside Dassault for V5 and componentized by Spatial for external licensing in 2012 ([blog.spatial.com/news/cgm](https://blog.spatial.com/news/cgm), [engineering.com](https://www.engineering.com/spatial-acis-cgm-and-the-future-of-geometric-modeling-kernels/)). They embody two different robustness philosophies:

- **ACIS**: exact modeler + *reactive* per-entity tolerances ("tolerant modeling", added mid-1990s to absorb imported data), boolean pipeline decomposed into resumable partial stages, and a *retry loop* (incremental boolean workflow) that mutates the operands when the boolean fails.
- **CGM**: watertight-*by-construction* shared-geometry model — an edge owns ALL of its representations (pcurve on face A, pcurve on face B, 3D curve) under one aggregate `CATEdgeCurve` object with an internal mapping, so faces can never disagree about their common boundary; plus mandatory operation journaling and a frozen-body/smart-duplication regime.

Both confirm the industry consensus (same as Parasolid/OCCT): tolerance-based modeling with per-entity gap budgets, never global exact arithmetic; robustness comes from *pipeline architecture and input conditioning*, not heroic numerics ([novedge history](https://novedge.com/blogs/design-news/design-software-history-boolean-modeling-in-cad-csg-origins-b-rep-breaking-points-and-robustness-solutions)).

---

## 2. ACIS tolerance doctrine

### 2.1 The four global resolutions

[Tolerance Variables, R17](http://www.q-solid.com/ACIS_Docs_R17/online/SPAacisuserTechArticles/SPAacisuser_totol.htm):

| Variable | Default | Meaning |
|---|---|---|
| `SPAresabs` | 1e-6 | "resolution absolute" — coincidence distance; also the smallest modelable feature |
| `SPAresnor` | 1e-10 | "resolution normalized" — ratio smallest/largest quantity ⇒ largest representable = resabs/resnor = 1e4 |
| `SPAresfit` | 1e-3 | fit tolerance for polynomial *approximations* of exact curves/surfaces (stored beside the exact geometry; used alone for cheap work, together with exact defs for precision) |
| `SPAresmch` | 1e-11 | machine epsilon guard: model span 1e4 × feature 1e-6 ⇒ need 1e-10, set to 1e-11 for headroom |

Notable: like Parasolid, ACIS is a **fixed absolute-tolerance kernel with a bounded model box** (span 1e4 units, resolution 1e-6 ⇒ dynamic range 1e10 with a "guard band" of one order of magnitude around resabs). Applications may rescale `SPAresabs`/`SPAresfit` "with great care"; `SPAresnor` never. The three-tier split (topology coincidence 1e-6 / approximation fit 1e-3 / numeric guard 1e-11) maps directly onto our TOLERANCE / mesh-deflection / eps ladder. `SPAresfit` being 1000× looser than `SPAresabs` is an explicit statement that **approximate geometry is a separate concern from topological coincidence** — approximations are cached companions, never the source of truth.

### 2.2 Tolerant modeling — the hot-edge/hot-vertex model

[Tolerant Modeling ch.6](http://www-isl.ece.arizona.edu/ACIS-docs/HTM/DATA/KERN/KERN/06TMOD/0001.HTM) (+[0002](http://www-isl.ece.arizona.edu/ACIS-docs/HTM/DATA/KERN/KERN/06TMOD/0002.HTM), [0003](http://www-isl.ece.arizona.edu/ACIS-docs/HTM/DATA/KERN/KERN/06TMOD/0003.HTM), [0004](http://www-isl.ece.arizona.edu/ACIS-docs/HTM/DATA/KERN/KERN/06TMOD/0004.HTM)):

- ACIS "was developed as an exact modeler"; tolerant modeling **"does not assume that geometry agrees with the topology"** and is applied **"only when required to maintain model topology integrity"** — i.e., tolerant entities are the exception, created on demand (import, healing failure, near-tangent local ops), not the default.
- **Representation flip**: on an exact edge, the EDGE's 3D curve is the *primary* geometry and coedges are secondary. On a tolerant edge (`TEDGE` derived from EDGE, `TCOEDGE` from COEDGE) **the primacy inverts: each TCOEDGE carries a mandatory pcurve as primary geometry, and the TEDGE's 3D curve is secondary ("lazy 3D curve")**. This is the deepest single idea: when faces disagree, the per-face parametric curves become the truth and the shared 3D curve becomes derived/deferred. (Same conclusion our mate/trim_dir work reached empirically; Parasolid instead keeps one SP-curve + tube radius.)
- **Edge tolerance semantics**: "maximum distance between any two equiparametric positions on any of its tolerant coedges" — an *equiparametric band* between the pcurves, not a distance-to-one-curve tube. A TEDGE with a single TCOEDGE has tolerance zero.
- **Coincidence rules**: two tolerant edges are coincident over an interval iff max-min distance of the bounded point sets < max of the two tolerances; tolerance below `SPAresabs` is clamped to `SPAresabs`. Tolerant-vs-exact uses max(tol, SPAresabs). **max(), not sum** — contrast Parasolid, which uses the *sum* of the two tolerances. (For us: OCCT also uses sum-ish combining; when we build P5, pick one rule and document it; max() is less inflationary.)
- **Vertex**: `TVERTEX` tolerance = max distance from vertex point to the ends of all adjacent edges terminating near it. Crucial fallback: "when solving for a tolerant vertex position, **a smart estimate of a solution position is used when curve/surface and curve/curve intersections fail** to determine the vertex location" — i.e., vertex-position solving is allowed to fail numerically and be replaced by an estimate + enlarged tolerance. This is a *designed-in failure mode*, exactly the pattern our forced-node/closure-weld caps implement.
- **Loop-closure relaxation**: "loops with tolerant edges and vertices are **not required to close to within SPAresabs**" — the closure criterion becomes the local tolerance. (Directly validates our closure-weld cap = max(join_tol, forced_node_eps).)
- **Near-tangency policy** ([options page](http://www-isl.ece.arizona.edu/ACIS-docs/HTM/DATA/KERN/KERN/06TMOD/0004.HTM)): global option `res_near_tangent` = largest angle (radians) treated as tangent, used system-wide; `lop_tolerant_hot` / `bl_tolerant_hot` allow local ops/shelling/blending to *create* tolerant edges for near-tangent contacts. Rationale verbatim: "By treating near-tangent edges as tangent, **difficult intersections that are likely to be slow and/or fail are avoided** between near coincident surfaces." Algorithms that *intend* tangency (blend/loft/sweep) set the edge convexity flag to "tangent" whether or not it is achieved within tolerance — intent is recorded on topology and trusted later. Removal path: non-tangent lateral tolerant edges can be re-intersected back to exact geometry later ("tweaking a face to its original surface"); tangent/near-tangent ones cannot.
- Tolerant entities interop: `EDGE` accepts `VERTEX` or `TVERTEX` freely; tolerant and exact entities mix in one body.

**Lesson for P5**: tolerances live on topology; exact stays default; tolerant status is *earned* by a detected disagreement; every consumer test must be tolerance-aware (max-rule); near-tangent is a *classification*, decided by one global angular threshold, that reroutes algorithms away from doomed intersections.

---

## 3. ACIS boolean pipeline

### 3.1 The four stages

[Boolean Operations](http://www-isl.ece.arizona.edu/ACIS-docs/HTM/DATA/BOOL/BOOL/01CMP/0001.HTM). Terms: **blank** = operand being machined, **tool** = operand doing the machining. Every boolean (unite/intersect/subtract) is:

1. **Intersect** all faces/edges of both bodies → build the **intersection graph** (a dedicated data structure, below).
2. **Imprint** the graph onto *both* bodies, splitting faces/edges.
3. **Classify** — decide keep/discard for faces, shells, edges of both bodies; discard.
4. **Join** (or un-join) along the intersection edges; reorganize shells and lumps.

Plus **chop** = simultaneous subtract + intersect sharing one intersection/imprint pass (cheaper than two booleans; leftovers optionally returned) — the OCCT analogue is our cut/common sharing SSI, which we already do; ACIS institutionalizes it as an API (`api_boolean_chop_body`).

### 3.2 The intersection graph as first-class citizen

[Intersection Graph](http://www-isl.ece.arizona.edu/ACIS-docs/HTM/DATA/BOOL/BOOL/01CMP/0002.HTM) — the most detailed public spec of a section-network data structure among the big kernels:

- A **wire body**: list of wires, each an independent connected component of the intersection; wires pairwise geometrically and structurally disjoint.
- Each intersection edge carries **one coedge per face of each body that passes through it** — counted **twice** if the face crosses (both sides), **once** if the intersection lies on the face's boundary. Blank-side and tool-side coedges are kept separate (tool coedges attached via an edge *attribute*, blank coedges via the normal pointer). Coedges of one edge are ring-linked as partners.
- **Dummy coedges**: whenever all coedges of an edge would have the same sense (sheet/incomplete bodies, boundary-running intersections), a converse-sense dummy coedge is appended so traversal invariants hold. "A dummy coedge may be present even if not strictly necessary."
- **Point intersections are zero-length edges**: "an intersection graph edge with no geometry and identical start and end vertices is used to represent an intersection point", with coedge counts encoding whether the point lies in a face interior, edge interior (2 coedges, one per edge portion), or on a vertex (one coedge per incident edge). ⇒ ACIS represents **vertex-only/tangential-touch interferences inside the same graph structure** rather than as a side channel — this is the P4 (EE/EF interference) architecture answer: unify point contacts into the section network with degenerate edges.
- Traversal contract at branch vertices (select coedge ending at vertex with non-NULL next → follow next → find opposite-sense partner with non-NULL next → repeat) — ordering at branches is explicitly *not* significant provided the traversal visits every edge.
- **Every graph entity carries attributes linking it back to the originating entities of both bodies** ("data essential to the operation of the later stages … must be constructed correctly") — ATTRIB_INTGRAPH/INTEDGE/INTVERT/INTCOED/FACEINT/EFINT classes in the TOC ([Booleans TOC](http://www-isl.ece.arizona.edu/ACIS-docs/HTM/SELECT/SUB/SB27_99.HTM)). Provenance is load-bearing, not debug info.

### 3.3 Partial booleans — the pipeline is resumable and steerable

[Partial Booleans](http://www-isl.ece.arizona.edu/ACIS-docs/HTM/DATA/BOOL/BOOL/01CMP/0010.HTM): the four stages are exposed as APIs so a caller with prior knowledge can skip work:

- `api_boolean_start` — init structures; `api_update_intersection` — **inject a known intersection curve** for a face pair (creates a surface-surface intersection record, bypassing SSI); `api_fixup_intersection` — repair sense/orientation of injected records ([ref](http://www.q-solid.com/ACIS_Docs_R17/online/refman/generated/refman/SPAbool/function_api_fixup_intersection.htm): update may reverse the curve, fixup re-anchors direction from positions stored in ATTRIB_FACEINT); `api_selectively_intersect` — intersect only a *given array of face pairs* instead of all×all; `api_complete_intersection_graph` / `api_bool_make_intersection_graph` — materialize the graph as a wire body ([ref](http://www.q-solid.com/ACIS_Docs_R17/online/refman/generated/refman/SPAbool/function_api_complete_intersection_graph.htm)); then finish with `api_slice_complete`, `api_imprint_complete`, `api_imprint_stitch_complete`, or `api_boolean_complete`.
- Declared use cases: feature patterns on a planar face; re-evaluating a feature when topology doesn't change. This is the "smart boolean" architecture: **cache/replay SSI results across reruns**. For us: our shared-SSI/bridge-march infrastructure is the same idea; worth formalizing an internal `update_intersection(faceA, faceB, curves)` seam so recognizers/oracles/replays can feed the splitter (P3/P6 lever, and the natural hook for a curve-cache between rotated reruns).
- **Imprint as a standalone op** ([Imprint](http://www-isl.ece.arizona.edu/ACIS-docs/HTM/DATA/BOOL/BOOL/01CMP/0004.HTM)): embeds the graph in both bodies; closed loop ⇒ new face; open loop ⇒ *spur* or *slit* in an existing face (they keep open imprints!); documented uses include "**debugging a Boolean operation**" — imprint-only mode is the official diagnostic of stage 1–2 vs stage 3–4 failures. (We should keep our imprint-only debug gate permanently.)

### 3.4 Glue booleans — the same-domain fast path (P3)

[class glue_options](http://www.q-solid.com/ACIS_Docs_R17/online/refman/generated/refman/SPAbool/class_glue_options.htm) + `api_boolean_glue`: a dedicated boolean for the case where **"the intersection of blank and tool is known to lie along a set of coincident faces"** — no SSI at all. The options object is a formal vocabulary of same-domain configurations:

- Caller supplies explicit arrays of pairwise **coincident faces** (blank[i] ↔ tool[i]).
- **Coincident patch** = maximal connected set of faces of one body with a well-defined onto coincidence mapping to a patch of the other.
- Precision flags (all default unset; wrong values = undefined behavior, "the glue operation will rely heavily on this information"): `patch_and_face_cover` (in every coincident pair, one face's point set ⊆ the other's; likewise patch-level), `blank_patches_strict_cover` (blank patch covers tool patch with tool ⊆ *interior* of blank), `single_face_patch` (each patch is exactly one face, all edges and vertices coincident — "geometrically and topologically identical"), `non_trivial` (solids guaranteed to actually interact: tool fully outside blank for glue-unite, fully inside for glue-subtract). `strict_cover` and `single_face_patch` are mutually exclusive.
- Reading for our P3: ACIS treats same-domain not as a perturbation of general booleans but as a **separate combinatorial regime keyed on a face-to-face coincidence mapping**, with an explicit taxonomy — equal / covered / strictly-interior — determining which edge-level cases (boundary crossing vs interior containment) can be skipped. Our planned same-domain subsystem should adopt the taxonomy: classify each coincident face pair as identical / A-covers-B / B-covers-A / partial-overlap before choosing an imprint strategy; the "partial overlap" cell is the only one that needs 2D boolean work in the shared surface's UV.

### 3.5 Regularized vs non-regularized

[Reg/Nonreg](http://www-isl.ece.arizona.edu/ACIS-docs/HTM/DATA/BOOL/BOOL/01CMP/0005.HTM), [Nonregularized Operations](http://www-isl.ece.arizona.edu/ACIS-docs/HTM/DATA/BOOL/BOOL/01CMP/0008.HTM):

- **Regularized** = operate on interior point sets, exclusive of boundaries; "resolves many modeling tangency issues" — boundary-coincidence ambiguities are *defined away* (lower-dimensional junk removed before returning).
- **Non-regularized** = same pipeline plus three extra keep-rules: (1) SINGLE_SIDED faces that become DOUBLE_SIDED BOTH_INSIDE (internal membranes) are kept; (2) any face-face coincident region is kept; (3) **no edge/vertex merging at the end**. Consequences spelled out: nonreg-unite keeps all face regions of both bodies (possibly split); nonreg-intersect of two blocks sharing a face keeps that face (a sheet result from solids); nonreg-subtract of a sheet from a sheet leaves an imprint. Implemented as an argument/mode on the same functions, not a separate engine.
- Also relevant: `api_regularise_entity` exists as a standalone post-op (strip unneeded entities), and `NO_MERGE_ATTRIB`/`api_set_no_merge_attrib` let callers *pin* specific edges against the final merge ([TOC](http://www-isl.ece.arizona.edu/ACIS-docs/HTM/SELECT/SUB/SB27_99.HTM)).
- For us: regularized-only is fine as product scope, but the **final merge must be a separable stage with per-edge opt-out** — that is what makes imprint/nonreg/selective modes fall out for free, and it is the natural home of our q6-weld/merge-coplanar logic.

### 3.6 Selective booleans (SBOOL) — cellular topology as the classification engine

[Booleans TOC](http://www-isl.ece.arizona.edu/ACIS-docs/HTM/SELECT/SUB/SB27_99.HTM) (`api_selective_boolean_stage1/stage2`, `api_selectively_imprint`, `api_selective_unite`, `bool:tube` [scheme doc](http://www-isl.ece.arizona.edu/ACIS-docs/HTM/DATA/SBOOL/SBOOL/02SC/0004.HTM)), [Cellular Topology component](http://www-isl.ece.arizona.edu/ACIS-docs/PDF/CT/01CMP.PDF), [graph booleans](http://www.q-solid.com/ACIS_Docs_R17/online/SPAacisuserTechArticles/SPAacisuser_grbool.htm), patents [US7330771](https://patents.google.com/patent/US7330771)/[US7031790](https://patents.google.com/patent/US7031790):

- **Cellular topology (CT)**: an auxiliary decoration on a body that organizes it into 3D (solid) and 2D (sheet) **cells** — the connected regions delimited by all faces including internal DOUBLE_SIDED BOTH_INSIDE membranes. A nonregularized unite produces exactly this: one body whose cells are {A only, B only, A∩B} pieces.
- **ct-graph**: cells become graph vertices labeled by provenance — tool cells "1", blank cells "2", shared "1,2"; adjacency = shared cell boundary. Standard booleans are then *graph vertex selections*: unite keeps all, subtract keeps only "2", intersect keeps only "1,2" (patent US7330771 phrasing). A **selective boolean** lets the caller pick any subset of vertices (with start/end position seeds in `bool:tube` — stage 1 builds the imprinted nonreg body + ct-graph once, stage 2 re-runs cheap selections many times).
- The graph-theory component supplies unite/intersect/subtract on abstract graphs, with subtract offered in **keep-boundary and lose-boundary variants** because "the result of subtraction can be ambiguous — an edge might be half in a graph" ([grbool](http://www.q-solid.com/ACIS_Docs_R17/online/SPAacisuserTechArticles/SPAacisuser_grbool.htm)).
- Architecture lesson (matches Parasolid's imprint–join–select and OCCT's BOPAlgo_Builder/CellsBuilder): **imprint once → enumerate cells with provenance → op = cheap selection**. Our per-op flood/classification should converge to this: build the imprinted arrangement once, tag every face/region with (fromA, fromB, inA, inB), and derive cut/common/fuse/xor/split as selections over one structure. This also gives chop, multi-result ops, and rerun-with-different-op for free — and it is the strongest antidote to our current per-op classification divergence (cut disagreeing with common on the same face set).

### 3.7 Failure recovery — the incremental boolean workflow

[Spatial blog](https://blog.spatial.com/what-to-do-when-your-3d-modeling-boolean-operations-fail) (R2017.1+ era feature, "prepare API"):

- Loop: run boolean → on failure the kernel reports **complexities** (named reasons: *short edges, sliver faces, tolerant vertices/collapsed features, improper intersections, near-coincident entities, near-tangent interactions, complicated intersections*) → run **prepare** = localized (not global) optimization on both operands targeted at the reported complexities (e.g., near-coincident faces are *made exactly coincident*) → retry.
- Stop conditions: no complexity reported; prepare fails; or **two successive attempts report the identical complexity set** (no progress). Claimed: 70% of failed booleans fixed with a *single* prepare iteration. Workflow is op-agnostic (works for regularized, selective, imprint, …) and user-extensible (custom routines/stop criteria).
- Key contrasts drawn by Spatial: healing = proactive + global; prepare = **reactive + local + in the context of this boolean pair**. And the fix is applied to the *inputs*, not patched into the half-built result.
- Supporting detection/mutation APIs (usable standalone): `api_detect_short_edges` (length < tol, default `SPAresfit`; optionally **replace with TVERTEX**) and `api_detect_sliver_faces` ([ref](http://www.q-solid.com/ACIS_Docs_R17/online/refman/generated/refman/SPAbool/function_api_detect_sliver_faces.htm): face with ≥1 short edge, ≤3 long edges, and max distance among long edges < tol; tol=-1 ⇒ auto from bounding box; optionally **replace whole face with a TEDGE**); `api_check_entity_ff_ints` (improper face-face intersection check); `api_split_edges_at_poles`, `api_split_periodic_faces`, `api_split_face_at_g_disc` (pre-splitting at poles/seams/G1 discontinuities — they sanitize periodic/singular geometry *before* booleans rather than special-casing inside).
- For us (maps to our campaign directly): (1) formalize our env-gated fixes into a **complexity report** (each gate = one named complexity class) + a **prepare pass** that applies only the relevant fix and retries — that converts our one-off SESSION_* toggles into the industry-standard control loop; (2) sliver *face*→tolerant-*edge* collapse and short-*edge*→tolerant-*vertex* collapse are the canonical dimensional-reduction repairs (our trim-snapped sliver dropping is a special case; make it a first-class op with the ACIS geometric criteria); (3) "make near-coincident exactly coincident, then re-run" is precisely the same-domain promotion we need in P3 — snap-to-shared-surface is a *prepare* action, not an in-splitter tolerance hack.

### 3.8 Healing / import conditioning (ACIS + 3D InterOp)

[healing-in-3d-interoperability](https://blog.spatial.com/healing-in-3d-interoperability), [acis-modeler-3d-translated-data](https://blog.spatial.com/acis-modeler-3d-translated-data), [tolerant stitching](http://www.q-solid.com/ACIS_Docs_R17/online/SPAacisuserTechArticles/SPAacisuser_mointrstitchto.htm):

- Three ordered phases: **stitch** (replace groups of coincident edges/vertices with single edges/vertices ⇒ topologically complete sheet/solid; tolerant stitching produces TEDGE/TVERTEX where gaps exceed SPAresabs), **geometry simplification** (spline → analytic where possible: "a simple cylinder … after translation all three surfaces may be represented as splines" — restores design intent, shrinks data, and re-enables analytic intersection fast paths), **gap tightening** (recompute intersections / adjust geometry to meet within tolerance; general gaps: extend + re-intersect surfaces; tangential analytic contacts: a **graph-based solver applying rigid/linear transformations, preferring to move higher-curvature surfaces**; spline-spline with isoparametric intersection: move control points).
- Cleanup ops: small-edge removal, sliver-face elimination, merge of adjacent coplanar/same-geometry faces. Units flagged as "one of the most underestimated sources of geometric corruption" (fix units before healing or tolerance scales are wrong).
- Order matters and is meaningful for P6 corpus work: simplify-to-analytic *before* the boolean gives exact recognizers a chance (our STEP analytic reader already proved this); gap-tighten *after* stitch so the topology tells you which pairs must meet.

---

## 4. CGM architecture

### 4.1 Object model, units, resolution

[GeoObjects](https://www.maruf.ca/files/caadoc/CAAGobTechArticles/GeoObjects.htm):

- Factory (`CATGeoFactory`) fixes units and validity ranges for all objects: **unit = mm**, angles in radians. **Model size: everything inside [-1000 m, +1000 m]** (1e6 mm; was ±100 m before R14). **Resolution = 1e-3 × unit = 1 µm**: below it two points "are considered to be geometrically at the same location". Dynamic range ⇒ 1e9 (vs ACIS 1e10, Parasolid ~1e11).
- **The tolerant escape hatch is doctrinal**: "the topology captures the design intent, and **the gap between the geometry of two faces sharing the same edge can be greater than the factory resolution: the modeler is tolerant**." I.e., CGM is exact-at-resolution for native data, with topology-level gap tolerance reserved for imported geometry.
- **All geometry must be C2**; objects failing this must be *split at discontinuities and reassembled topologically* — smoothness problems are pushed into topology (more edges) instead of being handled inside evaluators. (Matches ACIS `api_split_face_at_g_disc`; a standing argument for our splitting NURBS at knots of reduced continuity before SSI.)
- Persistent objects carry stable **tags** surviving stream/unstream; operators/params are transient. Surfaces include "procedural surfaces computing evaluations from other surfaces" (offsets etc. stay procedural/exact, not fitted).

### 4.2 Watertight by construction — the CATEdgeCurve aggregate

[TopoCreate](https://www.maruf.ca/files/caadoc/CAATobTechArticles/TopoCreate.htm), [Curves](https://www.maruf.ca/files/caadoc/CAAGobTechArticles/Curves.htm), [FAQ/data-checker](https://www.maruf.ca/files/caadoc/CAACgmTechArticles/CAACgmDataChecker.htm):

- A topological edge's geometry is a **`CATEdgeCurve`: one aggregate object holding *all* representations of the same curve** — when the edge bounds n faces it holds the n pcurves (plus possibly a 3D curve), with an internally computed **mapping between representations** (`CATEdgeCurveComputation` for `CATMergedCurve`; `CATIntersectionSurSur` directly produces `CATIntCurve`). Concrete types: `CATSimCurve` (similar curves), `CATIntCurve` (intersection result), `CATMergedCurve` (merged after the fact); the base class cannot be instantiated.
- A vertex's geometry is a **`CATMacroPoint`** aggregating one POEC (point-on-edge-curve) per incident edge (or point-on-surface/point for immersed vertices) — the vertex, too, is a bundle of per-representation positions with an internal gap budget.
- The **data checker** ([CAACgmDataChecker](https://www.maruf.ca/files/caadoc/CAACgmTechArticles/CAACgmDataChecker.htm)) enforces the watertight contract numerically: "the gap between the curves making up a CATIntCurve should be less than the resolution" (rule ICG_1); macro-point internal gap ≤ resolution (MPG_1); a POEC must lie on its edge curve (PCE_1); pcurve limits within surface bounds (PSS_*); an EdgeCurve must not contain identical pcurves twice (EIP_1); loops must not be "closed in 3D while open in 2D" (SCE_0); topologically-smooth edges must be geometrically smooth (ESH_1); no residual touched cells in a finished body (TRE_1); closed wire ≥ 2 edges (CWE_1).
- Net design: where ACIS/Parasolid record a *tolerance radius* around one chosen representation, **CGM stores all representations plus an explicit parametric mapping between them, and bounds their mutual gap** — an edge is never re-derived from face geometry at use time, so faces cannot drift apart. Imported models are the sanctioned exception (gap may exceed resolution ⇒ that is CGM's "tolerant modeling", per [GeoObjects](https://www.maruf.ca/files/caadoc/CAAGobTechArticles/GeoObjects.htm) and the CGM product page's "tolerant modeling ensures model validity for imported data" [spatial.com](https://www.spatial.com/solutions/3d-modeling/cgm-modeler)).
- **For us this is the highest-value structural idea in this file**: our booleans already produce both mates' pcurves from one marched section — P5 should make the *pair-with-mapping* the stored artifact (edge = {pcurve_A, pcurve_B, [3D], sync mapping, gap}), with a checker rule gap<tol, instead of one 3D polyline + per-face re-projection. Most of our rotated-chair alias/mate bugs (trim_dir half-circle, whole-seg alias keys) are exactly the failures this representation eliminates by construction.

### 4.3 Topological model — frozen bodies, smart duplication, touched cells

[TopoModel](https://www.maruf.ca/files/caadoc/CAATobTechArticles/TopoModel.htm), [CAATopOverview](https://www.maruf.ca/files/caadoc/CAATopUseCases/CAATopOverview.htm):

- Cells: vertex/edge/face/volume; domains: loop, wire, shell, lump, vertex-in-face/volume; orientation split into **CATOrientation** (cell vs geometry), **CATLocation** (outer/inner/full — "full" = immersed non-cutting topology, i.e., slits/spurs are first-class), **CATSide** (matter left/right/full). Validity rules for freezing include matter-left edge ordering, side/location compatibility, closed inner+outer loops with open immersed ones allowed, no cross-cutting shells.
- **Frozen bodies + smart duplication**: "once accepted, a body is never modified" — operators *always* create a new body that **shares all untouched cells/domains with the inputs** (`CATSmartBodyDuplicator`); only cells "touched" by the operation are duplicated; a valid finished body must contain no touched cells (checker rule TRE_1). Operators never modify inputs and are themselves transient, non-streamable, run under an explicit `CATSoftwareConfiguration` (versioning object: old documents replay old algorithm behavior — behavioral versioning is in the API contract).
- This is a persistent-data-structure regime: cheap history/rollback, safe replay, natural parallel reads — but CGM is **not thread-safe**; concurrency is via multiprocessing ([engineering.com](https://www.engineering.com/spatial-acis-cgm-and-the-future-of-geometric-modeling-kernels/); confirmed for InterOp parallel per-part healing [blog](https://blog.spatial.com/acis-modeler-3d-translated-data)).
- For us: our arena/pool phase-4 referencing is the same direction; the CGM refinement to adopt is the explicit **touched-cell discipline** (operator-scope marker that must be empty at commit — a cheap invariant assert) and result-shares-input rather than deep copy.

### 4.4 Boolean operators and same-domain doctrine

[TopoBoolean](https://www.maruf.ca/files/caadoc/CAATopTechArticles/TopoBoolean.htm):

- Operators: `CATDynBoolean` (union/intersection/removal), `CATDynSplit` (keep material on one side), `CATSewing` (skin onto body: "removing useless material and adding intersecting closed contours" — boolean with a sheet where the sheet wins), `CATTopologicalOperator::Trim` with per-face **keep/remove stamps** (selective boolean by explicit face marking — CGM's SBOOL analogue).
- The doc is unusually frank about overlaps: booleans "are **very sensitive to overlapping areas** of the boundaries of the input bodies". Ranked configurations: (a) same geometry + same topology on the overlap — favorable; (b) interpolated/refit surfaces in the overlap — creates numerous small elements; (c) multi-face patchwork overlaps — worst, "increasing the risk to create small elements".
- Prescribed mitigations (application-level contract, mirrors our P3 plan): **share geometry** ("a geometric element is shared if several cells directly refer it" — build B's face on *the same surface object* as A's, then coincidence is a pointer identity, no numeric test); **match faces logically** via transient attributes (declare the correspondence instead of detecting it); **modify operand shape** to remove the overlap when the result is unchanged; **define features logically** (constraints like "up to next face" rather than baked dimensions) so operands regenerate in a same-domain-safe way.
- For P3: priority order = pointer-identity same-surface > declared face-mapping > detected coincidence (with promotion-to-exact as a prepare step, per ACIS §3.7). Our recognizer/alias layer should output a declared face-mapping structure consumed by the same-domain subsystem, so detection happens once, upstream.

### 4.5 Journaling — operation provenance as an API contract

[JournalMethodology](https://www.maruf.ca/files/caadoc/CAATopTechArticles/JournalMethodology.htm), [CAATopOverview](https://www.maruf.ca/files/caadoc/CAATopUseCases/CAATopOverview.htm):

- Every topological operator fills a **topological journal** (`CATCGMJournalList` via `CATTopData`): events restricted to **creation, modification, deletion, subdivision, absorption, keep**, over *boundary* cells (faces/edges/vertices only). Copy-mode inputs default unmentioned cells to "survives"; no-copy-mode defaults to "gone unless kept" — defaults chosen to keep journals sparse.
- Journal validity is checkable (CAACheck): every result cell traceable to operands; only F/E/V reported; only bordering cells; parent+info collisions across dimensions forbidden. Journals feed **generic naming**: stable references of the form "cell dimension + parent feature + info tag" (info: 1=start, 2=end, 0=lateral) that survive rebuilds — the foundation of parametric update in CATIA.
- For us: our history layer (face provenance) is journal-lite. The CGM lesson is (a) fix a **closed event vocabulary** (created/modified/deleted/split/absorbed/kept) instead of ad-hoc tags; (b) make journal correctness a *checked* invariant in gates (every result face must trace to operand faces; split+absorb chains must compose); (c) journals are the natural substrate for P6 corpus diffing (compare journals, not just topology counts, across rotations — a face that changes provenance under rotation is a bug even when counts match).

### 4.6 CGM healing — gaps vs cracks

[healing-gaps-cgm](https://blog.spatial.com/cgm/healing-gaps-cgm):

- Vocabulary: **gap** = geometric spacing between topologically connected cells; **crack** = "open area surrounded by edges or vertices which are **not topologically connected**" (faces close in space, no adjacency). The **assembler** converts cracks→gaps (create topology when crack < tol); healing then closes gaps.
- CGM's checker flags gaps > **100 × resolution** (i.e., 0.1 mm); smaller gaps are declared "acceptable for downstream operators like Boolean or Offset" — a published **operator gap budget**: booleans must absorb up to 100× resolution. (For P5: define our own boolean input-gap contract as a multiple of model tolerance and test to it.)
- Two techniques: (1) **extend + re-intersect** around the gap, shape-preserving, best for simple/planar contexts; (2) **deform** surfaces/edges (global or local) with a user tolerance capping max shape change. Tolerance selection guidance: bigger than the largest crack to remove, smaller than the smallest feature/hole to preserve.

---

## 5. Synthesis: ACIS vs CGM vs (Parasolid/OCCT) on our four phases

| Concern | ACIS | CGM | Take for session_cpp |
|---|---|---|---|
| Tolerance substrate | fixed absolute ladder (1e-6/1e-3/1e-10/1e-11, span 1e4), tolerant TEDGE/TVERTEX on demand, max-rule | fixed resolution 1µm, span 1e9, native data exact-at-resolution, imported data tolerant, checker-enforced | P5: fixed ladder + per-entity gap earned by disagreement; publish an input-gap budget (CGM: 100×res) |
| Edge truth under disagreement | pcurves become primary, 3D lazy (TCOEDGE) | all representations + mapping in one CATEdgeCurve, gap<res checked | adopt CGM aggregate for boolean-born edges; ACIS flip explains import cases |
| Pipeline | intersect→graph→imprint→classify→join; resumable partial APIs; graph carries provenance attrs + degenerate point-edges | operators transient, frozen inputs, smart duplication, journal mandatory | keep 4-stage OCCT shape; add degenerate-edge point contacts (P4); imprint-once + cell-select for all ops |
| Same-domain | glue booleans: declared coincident-face arrays + cover taxonomy; prepare makes near-coincident exact | share geometry by pointer, declared face matching, overlap sensitivity ranked | P3: identity > declaration > detection; promote-to-exact as prepare step; cover taxonomy per face pair |
| Failure recovery | complexity report → local prepare → retry, fixed-point stop; sliver→tedge, short-edge→tvertex; imprint-only debug | validity pushed to checkers/journals; healing gaps-vs-cracks; behavioral versioning | formalize SESSION_* gates into complexity classes + prepare loop; keep imprint-only gate; add journal check to gates |
| Non-reg/selective | 3 keep-rule deltas on same engine; ct-graph selection; per-edge no-merge pins | Trim with keep/remove stamps; Location=Full immersed topology | make final merge separable + pinnable; classify once, select per op |

Historical note for calibration: CATIA V5 briefly shipped with ACIS before CGM matured ([engineering.com](https://www.engineering.com/spatial-acis-cgm-and-the-future-of-geometric-modeling-kernels/) — claim from that article; treat as secondary), and Spatial now sells both; CGM's componentized SDK gained ACIS-style conveniences (rollback, standalone interfaces) only after 2012. The two survive because the *by-construction* (CGM) and *by-repair* (ACIS) strategies are both necessary: native modeling wants the former, imported/degenerate data forces the latter. Our kernel, whose hardest corpus is imported STEP, needs the ACIS repair loop first and the CGM edge-aggregate as the internal representation goal.

---

## 6. Ranked actionable recommendations

1. **(P5, structural) Edge = representation bundle.** Store boolean-born edges as {pcurve_A, pcurve_B, optional 3D, parametric sync, gap}; checker rule gap < tol (CGM ICG_1/MPG_1 analogues). Kills the alias/mate/trim_dir class of rotation bugs by construction. [TopoCreate, CAACgmDataChecker]
2. **(now→P6) Complexity-report + prepare retry loop.** Name our failure classes (near-coincident, near-tangent, sliver, short-edge, improper-intersection, complicated-intersection), emit them from the boolean, apply the *matching* local fix, retry; stop on fixed-point of the complexity set. Converts SESSION_* gates into architecture. [Spatial blog]
3. **(P3) Same-domain as declared mapping + cover taxonomy.** Face-pair classification identical/cover/strict-interior/partial (glue_options vocabulary); promotion of near-coincident to exactly-coincident as a prepare action; pointer-shared surfaces where we control construction. [glue_options, TopoBoolean]
4. **(P4) Degenerate section edges.** Represent point/tangential contacts as zero-length edges with coedge multiplicity encoding face-interior/edge-interior/vertex incidence, inside the same section network. [Intersection Graph]
5. **(P3/P4) Dimensional-reduction repairs with ACIS criteria.** sliver face (≥1 short edge, ≤3 long edges, long-edge separation < tol) → tolerant edge; short edge → tolerant vertex; default tol = fit-tier (SPAresfit-like), auto-scale from bbox. [api_detect_sliver_faces/short_edges]
6. **(architecture) Imprint-once, classify-once, select-per-op** (ct-graph model): one imprinted arrangement with provenance tags; cut/common/fuse/xor/split/chop = selections. Eliminates per-op classification divergence. [SBOOL, US7330771]
7. **(P5) Publish tolerance contract**: fixed ladder (coincidence/fit/machine), bounded model box, input-gap budget (N× tolerance) that booleans must absorb; loop closure criterion = local tolerance, not global. [totol, tolerant modeling, healing-gaps-cgm]
8. **(P6) Journal checks in gates.** Closed event vocabulary (create/modify/delete/split/absorb/keep), every result face traceable; diff journals across rotation family, not just face counts. [JournalMethodology]
9. **(P4/P5) Near-tangent as classification**: single global angular threshold (res_near_tangent analogue) that reroutes near-tangent contacts away from generic SSI into tangency handling, optionally minting tolerant edges. [06TMOD options]
10. **(debug, keep) Imprint-only mode** as the permanent stage-1/2 vs stage-3/4 discriminator; per-edge no-merge pins on the final merge stage. [Imprint, NO_MERGE_ATTRIB]
