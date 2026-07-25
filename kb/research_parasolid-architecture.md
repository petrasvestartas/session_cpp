# Siemens Parasolid — boolean/modeling architecture (public-source deep dive)

Research date: 2026-07-24. Sources: PK Interface functional-description docs (V12.0 and V35.0 mirrors at q-solid.com), the founding tolerant-modelling paper (Jackson, Solid Modeling '95), the public XT Format Reference, patents, and secondary histories. All claims below are cited. Written for the session_cpp boolean-kernel program (OCCT-aligned architecture; master plan P3 same-domain, P4 EE/EF interference, P5 tolerance model, P6 corpus validation).

Primary sources:
- D. J. Jackson, "Boundary representation modelling with local tolerances", Proc. 3rd ACM Symp. Solid Modeling (SM'95), pp. 247–253 — EDS Parasolid, Cambridge. PDF: https://ftp.cs.wisc.edu/pub/users/prem/jackson-SM-95.pdf ; ACM: https://dl.acm.org/doi/10.1145/218013.218067
- Parasolid V35.0 Functional Description (chapter mirror): http://www.q-solid.com/Parasolid_Docs_V35/fd_index.html — esp. ch.16 "Session And Local Precision" (fd_chap.017), ch.30 "Checking" (fd_chap.031), ch.46 "Sewing and Knitting" (fd_chap.047), ch.50 "Imprinting and Curve Projection" (fd_chap.051), ch.51 "Introduction to Booleans" (fd_chap.052), ch.52 "Manifold Booleans" (fd_chap.053), ch.53 "General Booleans" (fd_chap.054), ch.90 "Importing Data" (fd_chap.091), ch.96/97 "Partitions"/"Rollback" (fd_chap.097/098), ch.113/114 threading (fd_chap.114/115), ch.14 "Model Structure" (fd_chap.015)
- Parasolid V12.0 docs: Booleans http://www.q-solid.com/Parasolid_Docs/chapters/fd_chap.10.html ; Partitions & Rollback http://www.q-solid.com/Parasolid_Docs/chapters/fd_chap.42.html
- Parasolid XT Format Reference (public): http://www.13thmonkey.org/documentation/CAD/Parasolid-XT-format-reference.pdf ; hand-decoded example: https://www.okino.com/solutions/sphere_parasolid_file_example_explained.htm
- Patents: US7031790B2 (Autodesk, cellular selective booleans — comparison point) https://patents.google.com/patent/US7031790 ; US11016470/US11921491 (Siemens, mesh→watertight B-rep, convergent modeling)
- Histories: https://novedge.com/blogs/design-news/design-software-history-romulus-to-parasolid-and-acis-kernel-lineages-that-defined-modern-cad ; product page https://plm.sw.siemens.com/en-US/plm-components/parasolid/

---

## 1. Context

Parasolid: Shape Data Ltd (Cambridge, founded by Braid/Grayer/Lang of Romulus lineage), first shipped 1988; owned successively by EDS → Unigraphics/UGS → Siemens (2007). It is the volume leader among licensed kernels (NX, Solid Edge, SOLIDWORKS, Onshape, many CAM/CAE). Its two signature architectural bets, both made in the early 1990s and still load-bearing:

1. **Tolerant modelling** — per-entity local tolerances on topology (not geometry), introduced ~V7/V8 era, published as Jackson SM'95. This is the direct ancestor of OCCT's per-entity tolerances (OCCT copied the doctrine; ACIS added it later as "tolerant hot vertices/edges").
2. **A single boolean engine** ("imprint–join–select") reused by everything: booleans, sewing (= boolean with FF/EF phases omitted), sectioning, imprinting, instancing, punching/fencing (= booleans with material-side reinterpretation).

The XT file format publicly documents the exact data-structure invariants the kernel promises (section 5), which makes it the best public spec of "what a valid Parasolid body is."

---

## 2. Tolerance doctrine

### 2.1 Session precision (the exact substrate)

[V35 ch.16, fd_chap.017]

- **Session linear precision** = 1.0e-8 units; distances below are zero, distances differing by less are equal. **Session angular precision** = 1.0e-11 rad. (`PK_SESSION_ask_precision`, `PK_SESSION_ask_angle_precision`.)
- **Size box**: all bodies must fit in a 1000×1000×1000 box centered at origin; recommended unit = meter (so ±500 m). Arc radii must be < 10× size-box dimension. So the effective relative precision is ~1e-11 of the model span — Parasolid is a *fixed absolute tolerance* kernel, unlike our diag-relative scheme.
- Guidance: keep unintended-distinct vertices > 100× linear precision apart, unintended-parallel directions > 100× angular precision apart.
- **Transitivity requirement**: precision must be transitive for validity — if d(1,2) < tol and d(2,3) < tol but d(1,3) > tol the body is declared *invalid*. This is a legality rule, not a runtime cluster heuristic: Parasolid pushes the coincidence-clustering ambiguity out of the algorithms and into the model-validity contract.
- Consequences documented: tangent curve/curve intersections whose separation < precision return one intersection point; cones with half-angle < angular precision degenerate to cylinders; scaling imported parts up/down can invalidate them (points off curves / points merging); "modest scaling up to 2× is unlikely to cause problems."
- Jackson gives the historic default global tolerance as **5.0e-9 m** ("chosen to be below the smallest realistic tolerance for most mechanical parts") — i.e. exactly *half the session precision*, which survives today as the tolerance imputed to "accurate" entities (below).

### 2.2 Tolerant modelling (local precision) — the Jackson doctrine

[Jackson SM'95; V35 ch.16 §16.3; ch.14 §14.8; XT format]

Core principles, verbatim-level:

- **Tolerances attach to topology, never to geometry.** Points/curves/surfaces (and derived objects: point-on-surface, trimmed curves) are always treated as exact. A tolerance is associated with each face, edge, vertex.
- **Regions**: vertex tolerance = radius of a sphere at the vertex point. Edge tolerance = radius of a tube centered on **one arbitrarily-chosen-but-fixed** parameter-space curve of the edge (all of the edge's other SP-curves must lie in this tube). Face tolerance = face "thickness" — *defined in the model but, in Parasolid, faces always carry the default tolerance* (declared "future work" in 1995 and still true in V35: only edges and vertices get local precision).
- **Monotonicity invariant**: vertex tolerance ≥ max tolerance of its edges; edge tolerance ≥ max tolerance of its faces. (Enforced: setting edge precision auto-raises its vertices; `PK_VERTEX_set_precision` refuses values below max of incident-edge precisions, refuses values so large an incident edge is fully swallowed.)
- **Test rule**: every coincidence/intersection test uses **the sum of the tolerances** of the two entities being compared.
- **Default = exact**: entities without local precision are "accurate" and carry a tolerance of **half the session precision** (XT: tolerance field null-double ⇒ ½ modeller resolution).
- **Validity of a tolerant model — the contractibility rules** (Jackson's key formalization):
  - Vertices must lie on connected edges and faces; edges must lie on connected faces (all "within summed tolerance").
  - Ends of an edge's parameter-space curves must lie inside the corresponding vertex spheres; loop curves need not meet end-to-end as long as they are within vertex tolerance.
  - **Edges may intersect only at vertices**: if the tolerance tubes of two edges intersect, each connected component of the intersection must intersect (be contractible onto) a *common vertex* of the two edges.
  - **Faces may intersect only at vertices/edges**: each connected component of the intersection of two face tolerance-regions must be contractible onto a collection of common edges/vertices.
  This gives a crisp self-intersection criterion under tolerances — "the intersection area is contractible onto shared boundary topology" — which is what `PK_FACE_state_self_int_c` / `PK_FACE_state_bad_face_face_c` check operationally.
- **Escalation rule**: when a topological operation (edge split, face add) cannot be performed at current tolerances because entities collide, *local tolerances are increased and redundant topology removed (merged) until a consistent model can be created*. Jackson calls the removal side "compression". Known cost: compression grows tolerances; containing tolerance growth ("models defined to a tolerance should unite to models of similar tolerance") is stated as an open problem.
- **Three declared motivations**: (1) importing foreign B-rep/surface data at whatever accuracy; (2) letting approximating constructions (blending!) use a loose local tolerance without polluting the rest of the model; (3) marking approximated regions so numerics are *not* run at inappropriately high accuracy (slow + unreliable).

### 2.3 Representation of tolerant edges (SP-curves)

[V35 ch.14 §14.8, ch.16 §16.3.1, ch.90 §90.3.1; XT format]

- Exact edge: one untrimmed 3-space curve on the edge; vertices within (vertex) tolerance of it; curve within modeller resolution of both adjacent surfaces (XT rule).
- Tolerant edge: the 3-space curve is **deleted** and replaced by an **SP-curve (surface-parameter-space B-spline) per fin**; on manifold edges that's two SP-curves. XT rule: every fin of a tolerant edge references a *trimmed* SP-curve on that face's surface; the curves must not deviate from each other by more than edge tolerance; ends within vertex tolerance of vertices. Trimmed-curve `point_1/point_2` lie exactly on the basis curve, within tolerance of the (possibly off-curve) tolerant vertex.
- Parasolid-generated SP-curves are **non-rational quadratic** B-splines. G1 in parameter space required (C2 optional via `PK_set_precision_c2_c`); an edge is split wherever the surface has parameter-space G1 discontinuities or singularities (SP-curves may *end at* but never *pass through* singularities; they may cross periodic seams freely — "no need for the application to artificially split curves here").
- **Nominal geometry**: optional mechanism to retain the original 3-space curve on a tolerant edge (for downstream export/reference). Checked by its own fault codes.
- Tolerance lifecycle API: `PK_EDGE_repair` (measures max deviation edge-curve↔surfaces / vertex↔edges, sets minimal sufficient precision — the recommended entry point), `PK_EDGE_set_precision_2` (explicit; lowering the number without rebuilding SP-curves can *fail checking* — documented foot-gun), `PK_EDGE_optimise` (shrink stored tolerance down to actual max curve deviation; short-edge special case: an edge is "short" if it lies entirely inside a vertex bubble), `PK_EDGE_reset_precision_2` (delete SP-curves, re-intersect adjacent surfaces to recover an exact edge; `method` = any coincident curve / prefer true intersection / only true intersection; documented to fail near tangency), `PK_LOOP_close_gaps`/`PK_FACE_close_gaps` (close loop gaps at tolerant vertices in 3-space and/or parameter space: minimise across periods, extend/trim SP-curves to `tol_in_2_space` default 1e-6, or as last resort **reparameterise the surface**).
- **Performance doctrine**: precision in the 1e-6..1e-8 range produces many-segment SP-curves and a corresponding slowdown — tolerant edges are meant to be *loose*; enquiries on exact geometry are explicitly faster. Don't set precision tighter than needed.

### 2.4 What this means vs our kernel (P5)

- Parasolid separates **model legality** (transitive clusters, contractibility, monotone vertex≥edge≥face tolerances) from **operation budgets** (boolean `default_tol`/`max_tol`, sewing `gap_width_bound`). We currently mix both in per-op epsilons. Adopting the three invariants (sum-of-tolerances tests, monotone hierarchy, contractibility as the self-intersection criterion) is the highest-leverage P5 item.
- Parasolid's answer to "tolerant edge geometry" is *pcurve-primary* (SP-curves per fin, 3D curve deleted); OCCT's is *3D-primary* (3D curve + pcurves + SameParameter). Ours is OCCT-style. The Parasolid lesson that transfers anyway: the designated-curve-plus-tube model (ONE canonical curve per edge, everything else within tube radius) and per-fin trimmed pcurves whose *ends need not hit the vertex* (only be within vertex tolerance).

---

## 3. Boolean architecture

### 3.1 The three-phase pipeline: imprint → join → select

[Jackson SM'95 §4 — still the only published description of the engine's internals; confirmed by V35 ch.50 §50.4.1 "the initial stage of a boolean operation is to intersect the target and tool entities and imprint edges and vertices…"]

Applies to conventional solid B-reps *and* mixed-dimension cellular non-manifold models (one algorithm for both).

**Phase 1 — Imprint.** Add topology so that each entity of one body meets the other body only *at topology* ("the interpretation is the same as if the two had been part of the same model"). Output: a list of topologies on one body with **matching topologies** on the other. A vertex may correspond to an edge, face, or even a 3-space cell if the tolerances so indicate. Order of comparisons is strictly **dimension-ascending** — "the algorithm compares low-dimension topologies before moving on to higher ones; this simplifies the capture of all geometric relationships":

  a) **vertex–vertex** — cluster coincident vertices (one-to-many allowed).
  b) **vertex–edge** — vertex inside an edge splits the edge; the new vertex *inherits the tolerance of the split edge*; splitting may delete nearby topology, including already-matched topology (re-match).
  c) **edge–edge** — intersect with **sum of edge tolerances**; intersections are returned as *areas of coincidence* (parametric ranges on each edge). Areas containing a common vertex are discarded (already handled by b). Others split *both* edges. **Contract on the curve–curve intersector: return an intersection for each area of coincidence, to tolerance** — interval/coincidence output, not point-only.
  d) **vertex–face** — interior vertex gets a corresponding vertex added to the face.
  e) **edge–face** — intersect edge curve with face surface for *intervals of coincidence*; because the edge was already compared with the face's edges, only interior coincidence/crossings remain. Coincident interval ⇒ add an edge to the face between existing vertices and pair it. Crossing ⇒ add paired vertices. **Any edge split here must be re-compared with other edges — smaller pieces may now be coincident when the whole was not** (bounded: only edges sharing a vertex need rechecking).
  f) **face–face** — intersect surfaces last. **Contract on the surface–surface intersector: return curves and point-contacts sufficient to (i) separate the areas of one surface lying on each side of the other and (ii) represent each contact/coincidence area — but it need NOT produce curves for contact areas already known common from edge–edge / edge–face phases. "This can be important, since computation of near-tangent intersection curves can be very unstable."** Trim section curves against both face boundaries; keep only portions lying on both faces; add them as paired edges. Again re-compare split edges.

  Post-condition: the boundary of each body is divided so each face lies inside, outside, or *on* the boundary of the other. Degeneracy cleanup is part of imprint: e.g. imprinting a line across a face with a short edge produces two coincident vertices → merge them and delete the thin face that degenerated to an edge.

**Phase 2 — Join.** Two jobs: (1) **compression** — when matched entities have different tolerances, take the larger; keep merging until there is a strict 1–1 correspondence between matched vertices/edges/faces of the two bodies (e.g. an edge fully inside a vertex sphere is removed and its ends combined); (2) topologically join into one (generally non-manifold, cellular) model and determine the resulting regions.

**Phase 3 — Select.** By operation (unite/subtract/intersect) and regularisation options, keep/delete regions, faces, edges, vertices via inside/outside classification — "possible because regions must be unambiguously inside or outside after the imprint phase." Options at this stage: drop lower-dimensional topology, split disconnected results, extract manifold sub-models.

**Sewing is the same engine minus e) and f)** (only vertex/edge matching), used to stitch trimmed surfaces known to abut; if faces don't meet at expected tolerance you get a thin hole, fixed by raising edge tolerances and re-sewing [Jackson §6.1; V35 ch.46].

Comparison note: OCCT's BOPAlgo pipeline (PaveFiller interference computation ascending VV→VE→EE→VF→EF→FF, then Builder) is the same dimension-ascending doctrine; Parasolid's published version adds two things OCCT lacks structurally — the *matched-topology correspondence lists* as a first-class output of imprint, and *tolerance-driven compression* in join.

### 3.2 The PK-level boolean surface (options doctrine)

[V35 ch.51 — full option table for `PK_BODY_boolean_2` (global) / `PK_FACE_boolean_2` (local); ch.52 manifold; ch.53 general]

- **Target/tool model**: target is modified, tools are deleted; tags of target entities persist as far as possible. Multiple tools: union of overlapping tools is computed first, then one boolean (target ∩ (tool1 ∪ tool2 ∪ …)) — *never* mutual intersection. Recommended: one multi-tool boolean instead of N sequential booleans.
- **Global vs local**: global compares *all* face pairs; **local boolean** (`PK_FACE_boolean_2`) compares only *selected* face pairs — faster, but "does not guarantee topological consistency of the results"; classification is *local* (inside/outside near the imprint loop). Local booleans support: `extend_face_list` (grow the face set to close incomplete imprint loops), `limit_target/tool_faces` (only listed faces may be trimmed), interior-overflow handling, region selectors. This is Parasolid's "fast path with explicit blast-radius" — the app owns correctness.
- **Two tolerance knobs on every boolean/imprint/section**:
  - `default_tol` (default **1.0e-5**): "the default tolerance to which the boolean operation, *if it has to*, may approximate entities or assume that entities are coincident." I.e. a per-operation *coincidence-assumption and approximation budget*, 3 orders looser than session precision.
  - `max_tol` (default 0.0 = unlimited): "maximum tolerance which may be applied to any entity involved" — a cap on tolerance escalation during the op. This pair (budget + cap) is the operational face of the escalation rule in §2.2.
- **Matched regions** (`PK_boolean_match_o_t`) — the tangency/coincidence subsystem, see §4.
- **Region/topolset selection**: `select_region` (local) / `selected_topolset` (global): include/exclude regions identified by a selector entity (region/face/edge/vertex) + optional `help_points` to disambiguate; `split_action` = fail | propagate if a selector is split. This generalizes cut/keep decisions beyond the three set-ops (⇒ our SESSION_OP-style variants map here).
- **Material-side options** (`target_material_side`/`tool_material_side` = default | inside | outside | none): locally *reinterpret* a body's solidity — treat sheet as solid (material behind normals or in front), solid as negated, solid as sheet. Yields punching, trimming, fencing (front = subtract with tool inside; back = subtract with tool outside), corner-making from two sheets, enclosure of solid regions by sheets — all *without* separate algorithms; there is a full truth table (unite/subtract/intersect × solid/void per operand → result solidity), and when the "outside" volume would end up solid, Parasolid negates the whole result to stay valid. Old `fence` option (none/front/back) is retained but deprecated in favour of material-side.
- **Imprint completion** (`imprint_complete_targ/tool`): tangentially extend imprinted edges from the tool's *laminar* (free) boundary so partial cuts still produce closed loops; overflow policies for where the extension lands: laminar boundary → `tangent` (default) / `ruled` (normal sheet) / `swept` (given direction) ± `laminar_walled` side faces; interior boundary (local booleans only) → `added` (pull more faces into the op) / `mixed` (concave: add face; convex: extend face) / `none` (fail with `PK_boolean_report_imprint_fail_c`). This subsystem is Parasolid's systematic answer to "section network doesn't close" — the same class of failure our scaffold/closure-weld fixes address, but resolved by *constructing* the missing imprint rather than welding gaps post hoc.
- **Merging**: `merge_imprinted` (merge mergeable imprinted edges after the op) + `selective_merge` (don't merge edges that were already mergeable before the boolean — preserve pre-existing scribes). Siemens **strongly recommends both ON** for manifold booleans. General-boolean equivalents: `merge_in_edge/face/solid` with dimension caps (`merge_in_*_dimension`), `impose_bodies` (target-wins / tools-win region inheritance), `merge_tools`, `prune_in_void`.
- **`imprint_overlapping`**: when target and tool faces share a surface, imprint the boundaries of the overlap area anyway (preserve scribed detail).
- **`keep_target_edges`**: deterministic survivor rule when target and tool edges coincide (default: older/smaller tag survives; option: target survives unless target edge is new and tool edge pre-existed; with GT on, target always survives). Note they *specify* the tie-break — determinism of coincident-entity survival is part of the contract.
- **`check_fa`** (default ON): check *only the faces adjacent to imprinted edges* after the boolean — built-in scoped validity checking, not a full body check.
- **`nm_edge_repair` + `blend_radius` (default 1e-5)**: if the result would be non-manifold along a common-extrusion-direction edge, auto-replace that edge with small blend faces so the manifold boolean can succeed. (Micro-blends as topology repair!)
- **`resulting_body_type`** (prefer solid/sheet/wire/general/original/simplest), `allow_disjoint`, `flag_no_effect`.
- **Tracking**: `tracking_type` basic (imprinted edge → target topology it lies on) or complete (+ tool topology that produced it); `track_regions` (region provenance records: derive-from-solid-parents vs created) — naming/history service for feature modelers, of the kind our oracle diffs would consume.
- **`update`**: version pinning — disable post-vN behaviour enhancements so old models rebuild identically. Institutionalized backward determinism.

### 3.3 Result/failure model

[V35 ch.51 §51.5, ch.30]

- Result struct returns bodies + a Parasolid report + a status: `success`, `no_clash` (operands don't interact — with defined semantics per op/material-side), `no_effect` (tool swallowed), `imprint` (succeeded but mergeable edges remain), `not_solid` (couldn't close to solid; sheets returned), `failed`.
- **Crucial contract**: `PK_boolean_result_failed_c` can come back with error code `PK_ERROR_no_errors`, and in that case **the model may be left in an invalid state; the documented recovery is: roll back the partition/session to the last mark**. Parasolid does *not* promise no-fail booleans or input restoration on failure — atomicity is delegated to the partition/rollback subsystem (§7). Fault tolerance = tolerant inputs (§2) + matched regions (§4) + rollback, not a never-fail guarantee.
- Manifold-boolean limitations: no T/X sheet cross-sections; subtract sheet-from-solid requires the sheet to cut completely (else `PK_ERROR_non_manifold`); sheet∩sheet only along coincident surfaces (overlap of target wins, normals from target).
- **Best-practice list** (V35 ch.52 §52.2, verbatim): align local coordinate systems with the model CS; use one multi-tool boolean; set `match_style` auto; **use oversize tools — "if the model is not exact and a tool that is considered to be exactly the correct size is used, coincident features can be missed"; for subtract the tool should protrude out of the target "to avoid problems with near coincidences"**; switch on `selective_merge`+`merge_imprinted`; use material-side instead of `fence`.
- Shared geometry after booleans is preserved and its propagation is specified (faces split from one face share its surface; sharing in the result mirrors sharing in the inputs).

---

## 4. Tangency & coincidence handling

Parasolid's strategy has three layers:

**Layer 1 — avoid computing near-tangent SSI at all.** The imprint phase's dimension-ascending order exists precisely so that contact areas discoverable from lower-dimension comparisons (edge–edge, edge–face coincidence intervals) are *excluded from the surface–surface intersector's obligations* — Jackson: near-tangent intersection curves are "very unstable", so the FF intersector must only separate sides and represent contacts *not already known*. Face tolerances (thickness) were proposed in 1995 specifically to declare approximately-coincident/tangent surface pairs as coincident instead of intersecting them ("intersecting them to an unsuitable tolerance will be very difficult as many complex curves may result") — never shipped; the shipped mechanism is matched regions.

**Layer 2 — matched regions** (`matched_region` substructure, on booleans, imprints, and sectioning) [V35 ch.51 §51.4.5]:
- `match_style`: `basic` (legacy default) / `auto` (**strongly recommended**: Parasolid auto-detects nearly-coincident regions) / `relax` (deprecated). With `auto`, optional `auto_match_tol` overrides `default_tol` for match detection.
- Application-supplied matches: pairs (edge–edge, face–edge, face–face, vertex–vertex) + `match_type` ∈ {exact (bounds match within tol), contains, overlap, imprinted} + per-pair `tolerance`. Recommendation: for FF/FE always use `overlap`.
- Documented rescue cases: (1) unite of a body with its mirror copy — "near coincident and tangential geometry meeting at the boundary" fails unmatched, succeeds with F1–F2 matched; (2) two sweeps along the same guide path — F1's boundary lies within tolerance of F2: match F2 + boundary edges E1,E2; (3) cone united with hemisphere sharing a base circle where the *neighbouring* faces intersect — only `auto` handles it (basic fails). Matched regions make booleans "produce a better result (coincident regions match as closely as possible) **and faster**."
- This is precisely our P3 same-domain subsystem: a declared/auto-detected same-domain relation consulted *before* SSI, with its own tolerance, unifying pcurve/section bookkeeping across the pair.

**Layer 3 — tolerant compression at join time.** Whatever coincidence survives to topology (vertex swallowing an edge, matched entities with different tolerances) is resolved by merging with the larger tolerance (§3.1 join). And at the session-precision floor, tangent crossings within precision are *defined* to be single-point (§2.1) — ambiguity legislated away rather than computed.

Sewing adds the empirical playbook for near-coincident boundaries [V35 ch.46]: gap_width_bound as the declared gap scale; refuse to sew a sheet on via a match *shorter* than the gap bound (a short match bounding an overlap can invert the sheet) unless *all* edges are short; short edges are allowed to degenerate into vertices and be engulfed; fused-edge tolerance is chosen to engulf both geometries and never below pre-existing tolerances; **incremental sewing** = multiple passes with increasing gap bounds (smallest first — Parasolid auto-generates the ladder if not supplied); "necklace of pearls" boundaries (many tiny edges with fat vertex bubbles) called out as "often lethal" — pre-merge them; µm-scale sliver filler sheets impair everything, sewing has limited ability to ignore 2–3-edge slivers; `reduce_edge_tolerance` post-pass shrinks sewn-edge tolerances where possible and reports where not. Sewing does **no** body check afterward — the app must check and resolve face-face inconsistencies.

---

## 5. Validity model & PK_BODY_check

[V35 ch.30 fd_chap.031; XT format rules]

Philosophy: **"Parasolid assumes that any bodies you are working with are valid. Most functions do not check the bodies you pass to them, and do not work properly if you hand them an invalid body."** Checking is explicit, expensive, and layered:

- **Checker suite**: `PK_BODY_check`, `PK_FACE_check`, `PK_EDGE_check`, `PK_FACE_check_pair`, `PK_GEOM_check`, `PK_ASSEMBLY_check`, `PK_TRANSF_check`.
- **Check groups run in sequence; each group runs only if all previous groups passed** — so not all faults are found in one call; per geometric entity only the most severe fault is reported (e.g. close-knots beats self-intersection), topology faults unlimited up to `max_faults`. `max_faults=0` turns any fault into a hard error (`PK_ERROR_check_error`) — "checks as assertions" mode. Recommended large value (e.g. 1000) to harvest faults.
- **Togglable check groups** (options of `PK_BODY_check_o_t`): `geom` (geometry validity), `bgeom` (B-geometry continuity), `mesh`, `top_geo` (topology–geometry consistency), `size_box`, `fa_x` (face self-intersection), `loops` (loop consistency), `fa_fa` (face–face inconsistency = shell-level self-intersection), `sh` (inside-out/inconsistent shells), `corrupt` (data structure), `nmnl_geom` (nominal geometry), `attribs`. Guidance: **fa_fa is "by far the most time-consuming for all but the simplest models and should only be used when self-intersection is suspected"** — ship two levels of check in the app UI (with/without fa_fa).
- **Fault taxonomy** (the public list is a de-facto spec of their invariants; highlights with meanings):
  - Geometry–topology: `PK_EDGE_state_bad_vertex_c` (vertex not on curve within tolerance), `PK_FACE_state_bad_vertex_c` / `bad_edge` (vertex/edge not on surface), `PK_EDGE_state_bad_spcurve_c` (fin SP-curves not within edge tolerance of each other), `vertices_touch` (edge's end spheres overlap illegally), `PK_TOPOL_state_not_G1_c`, `bad_closed` (closed but non-periodic geometry), degeneracy tokens (u/v, parametric/physical).
  - Tolerance-specific: `PK_EDGE_state_bad_tol_c` / `PK_VERTEX_state_bad_tol_c` (tolerance smaller than working precision), `PK_EDGE_state_open_c` (tolerances grew until an edge became a ring on an open curve — "tolerances consume features incorrectly"), `PK_FACE_state_redundant_c` (face redundant w.r.t. tolerances).
  - Self-intersection hierarchy: `PK_FACE_state_self_int_c` (edge/edge inconsistency within a face), `PK_FACE_state_bad_face_face_c` (face-pair inconsistency; body may still be usable — "many modeling operations work…although hidden-line may fail and mass properties may mislead"), `PK_SHELL_state_bad_*`, `PK_BODY_state_inside_out_c` (fix = `PK_BODY_reverse_orientation`).
  - Structural: corrupt tokens with `extra_faults` expansion into ~60 precise data-structure violations (fin ordering, region/shell containment, shared-geometry ordering `PK_TOPOL_state_bad_geom_share_c` — faces sharing a surface must be *ordered* on it, etc.).
  - `PK_TOPOL_state_check_fail_c` — the checker itself failed numerically; explicitly "should not be taken to mean the body is invalid." (Honest checker!)
- **Local checking**: an argument on mutating ops (sweep/spin, make_solid_bodies, attach_surf_fitting, transform, replace_surfs, delete...). Three stages: (1) each altered face self-intersection check — failure ⇒ error, roll back; (2) each altered face pairwise vs nearby faces — failure ⇒ `PK_local_check_failed_c` but *zero error* (acceptable intermediate state, app decides); (3) shell check with auto-negation (`PK_local_check_negated_c` when the body was inside-out and Parasolid flipped it — flip happens even with checking off). Local checking only validates what the op touched; a body invalid elsewhere stays invalid.
- Booleans' `check_fa` = the same idea inside the boolean: check only faces adjacent to imprinted edges, default ON.
- Session-level guards: `PK_SESSION_set_check_arguments` (rigorous argument checking, ON by default), `set_check_self_int` (B-geometry self-intersection screening at attach time; expensive; offsets of swept/spun/foreign always checked regardless), `set_check_continuity` (composite G1 checks at attach time). All are *attach-time gates* — geometry is vetted once, result cached in the data structure ("checks shall only be performed once, with the result stored").
- Intended uses, verbatim: debugging applications; validating results of modeling operations; **assisting construction of valid models from imported data; guiding the user attempting to repair an invalid model** — the checker is an interactive repair oracle, not only a gate. Recovery guidance per fault (e.g. rectify identifiers, attach fitting surface, negate body, reset tolerances) is in the doc table.

Mapping to us (P6): our gate battery (naked-edge count, volume flux, orientation) matches their fa_fa/sh/top_geo groups in spirit; what we lack is (a) the graded fault taxonomy with per-fault recovery guidance, (b) sequenced check groups with early-out, (c) scoped post-op checking (`check_fa` on faces adjacent to new section edges only) as the *default* in-boolean mode with full checks reserved for import/debug.

---

## 6. Local operations & related machinery

[V35 chapters 62–73 index; ch.30 §30.4; ch.33]

- Local ops (tweak/replace surface `PK_FACE_replace_surfs_3`, transform faces, delete faces with heal, taper, offset/hollow/thicken, generic `PK_FACE_change`) are first-class and share the local-checking contract above; they may legitimately traverse invalid intermediate states (checking OFF) with a final `PK_BODY_check` + rollback-on-fail. Face-face check arguments exist on taper/thicken/offset/hollow.
- Full Euler-operator set exposed publicly (`PK_EDGE_euler_split`, `PK_FIN_euler_glue`, zips, ring-face makers...) — the substrate the boolean's join/compression is built from, exposed so apps can do surgical topology edits under the same invariants.
- Redundant-topology control: `PK_TOPOL_delete_redundant_2` / `identify_redundant` with `scope` = merge_in/on/out (how far merging may leak into neighbours) and `propagate_redundancy` (entities that *become* redundant when others are removed). This is a standalone, reusable version of the select-phase cleanup.
- Imprinting family: `PK_BODY/FACE_imprint_*` returns **paired sets** of imprinted edges/vertices on target and tool (the correspondence is API-visible); `imprint_precision` = `auto` (exact projections — analytic-on-analytic, coincident inputs, B-curve-on-plane — give accurate edges; toleranced computations stamp their tolerance on the resulting edges/vertices) or `accurate` (edges always accurate, vertices may be tolerant to keep chains connected). Curve projection with normal/vector/perspective modes feeds it.
- Import/repair pipeline (ch.90): build faces via `PK_SURF_make_sheet_trimmed` from SP-curves/3D curves → `PK_EDGE_repair` all edges (compute deviations, set minimal tolerances or recover exact curves) → `PK_FACE_repair` (split at G1 discontinuities, cut out surface self-intersection regions) → `PK_BODY_repair_shells` (fix shell/region structure, topological clashes) → optionally `PK_EDGE_reset_precision_2` to tighten non-tangent edges back to exact. Self-intersection/degeneracy find-and-fix functions for raw geometry (`PK_CURVE/SURF_find/fix_self_int`, `fix_degens`).

---

## 7. Session, partitions, rollback

[V35 ch.96 fd_chap.097, ch.97 fd_chap.098; V12 fd_chap.42]

- **Partition** = self-contained collection of bodies/orphan geometry/transforms; no inter-partition references allowed (checked: `PK_PARTITION_state_xref_c`); one *current* partition receives new entities. **Pmark** = recorded state of a partition; **Mark** = session-level state; **Delta** = byte-stream diff (created/modified/deleted entities) between adjacent pmarks, stored *by the application* through the "frustrum" (host-provided storage callbacks, `PK_DELTA_register_callbacks`).
- The **pmark graph** supports branching rollback (alternative feature-history branches per body). Roll-to-pmark reports back exactly which entities were deleted/created/modified (with class filters and attribute-change reporting) so the app can fix its own references — the model-change notification channel. A **bulletin board** mechanism serves the same purpose per-operation.
- "Light partitions" drop the backward delta at the first non-initial pmark — cheap periodic save-state for *error recovery* specifically.
- Canonical failure protocol (stated in both boolean and error-handling chapters): failed/corrupting operation ⇒ **roll the partition back to the last pmark**; this also clears thread exclusion after serious errors (§8). Feature-update pattern: roll to the pmark before feature k, reapply modified feature, replay k+1….
- Session snapshot (`transmit`) + journaling exist alongside; XT transmit files are version-stable.

Mapping to us: this is the missing "atomicity layer" our pipeline currently improvises (private exe copies, regen guards). Even a minimal implementation — per-op entity-arena checkpoint + diff-record + roll-back-on-failed-gate — would let our boolean fail *cleanly* on rotated-chair frontier cells instead of leaving partial splits.

---

## 8. Threading model

[V35 ch.113 fd_chap.114 (application threads), ch.114 fd_chap.115 (SMP)]

Two orthogonal mechanisms:

**A. Application threads (thread-safe API).** Parasolid is thread-safe via an internal queue at the PK/KI boundary. Every PK function is classified **concurrent / exclusive / locally exclusive** (list published in the PK reference; all legacy KI functions exclusive). Rules: concurrent functions run simultaneously; an exclusive function waits until the kernel is empty and blocks everyone else; **locally exclusive** functions become concurrent when the calling thread has **locked partitions** (`PK_THREAD_lock_partitions`) — partition-granular parallelism: threads owning disjoint partitions can mutate concurrently. Fairness is guaranteed both ways (no starvation of exclusive waiters, no queue-jumping either). **Thread chaining** (`PK_THREAD_chain_start/stop`, with link `length`) amortizes lock/unlock across function sequences and provides atomic multi-call sections (exclusive chains); per-chain-local version controls allow different compatibility settings per thread. Error protocol: a serious error in one thread *excludes* all others from entering the kernel until rollback (which auto-clears the exclusion) or explicit `PK_THREAD_clear_exclusion`; `PK_THREAD_tidy` recovers state after longjmp-style handlers; per-thread error callbacks and memory callbacks.

**B. SMP worker threads.** `PK_SESSION_set_smp` (default when enabled: 1 thread/core, **hard cap 8 worker threads**, disabled on single-core). SMP is implemented *beneath* the API in low-level algorithms; documented SMP-enabled areas: **validity checking (parallel at face level), booleans (parallel in the face–face clashing portion)**, hidden-line/wireframe rendering, closest-approach, faceting (body level, opt-in flag), mass properties, spline fitting, isoclines, B-geometry array creation. Honest caveats: no linear scaling promised, ~2× temporary memory with 2 threads, locking overhead can make some functions slower, and if the OS can't create workers the work runs inline in the calling thread (reported via report records, operation still completes). **Determinism caveat**: with SMP on, order of returned entities and which result body inherits the target's tag are *not* guaranteed; `PK_DEBUG_shuffle_start/stop` exists to fuzz entity order for testing app robustness.

Mapping to us: the two-level split (thread-safe API with an op classification + internal parallelism confined to embarrassingly-parallel phases: FF clash detection and per-face checking) is a realistic template. Note what Parasolid does *not* parallelize: imprint/join topology mutation is serial. Parallel FF-candidate clash + parallel face checks are the two low-risk wins for our kernel; and their shuffle-debug tool is a cheap idea for hardening our result-ordering assumptions.

---

## 9. Patents & secondary literature

- **US7031790B2** — "Operator for sculpting solids with sheet bodies" (Autodesk, Kenneth J. Hill, filed 2003): cellular selective boolean: hollow the solid, unite all sheets into a non-manifold blank, cell decomposition against a universe body, cell-adjacency graph, even/odd distance from universe vertex classifies exterior/interior, per-sheet material-side labels select cells. Independent formalization of the same design space as Parasolid's material-side/fence and general-boolean cells — useful cross-check that *cell-graph + parity classification* is the standard robust approach for sheet-sculpting. https://patents.google.com/patent/US7031790
- **US11016470B2 / US11921491B2** — Siemens Industry Software: "Conversion of mesh geometry to watertight boundary representation" (convergent-modeling patents; mesh faces as first-class B-rep geometry — Parasolid's facet-geometry `partial support` notes throughout the boolean chapters are this feature surfacing). https://patents.google.com/patent/US11016470B2
- Shape Data-era algorithmic patents essentially don't exist publicly — UK software patentability + trade-secret culture; the tolerant-modelling doctrine was disclosed via SM'95 instead. The Jackson paper cites the adjacent robustness literature it rejected: exact rational arithmetic (Benouamer/Michelucci/Peroche), symbolic perturbation (Edelsbrunner–Mücke), consistency reasoning (Hoffmann/Hopcroft/Karasick; Stewart), tolerance-adaptive CSG/B-rep (Fang/Brüderlin/Zhu), polyhedral tolerance guarantees (Segal).
- Secondary: Novedge kernel-history series (ROMULUS→Parasolid/ACIS lineage; "tolerant modeling ethos avoids overzealous exactness, preferring predictable behavior under controlled epsilon… Booleans either succeed cleanly or regularize results to remove sub-tolerance shards") https://novedge.com/blogs/design-news/design-software-history-romulus-to-parasolid-and-acis-kernel-lineages-that-defined-modern-cad ; Parasolid overview PDF http://www.q-solid.com/Parasolid_Docs_V35/pdf/ov.pdf ; XT sphere walkthrough (Okino).

---

## 10. Actionable synthesis for our kernel (mapped to master-plan phases)

**P3 — same-domain subsystem** (= Parasolid matched regions):
1. Make matched regions a first-class *input and auto-detected* structure: pairs (FF/FE/EE/VV) + relation type (exact/contains/overlap) + per-pair tolerance, detected with an `auto_match_tol` distinct from and looser than the section tolerance. Consult it *before* SSI; matched FF pairs never enter the SSI walker.
2. Keep the dimension-ascending imprint contract: EE and EF coincidence intervals discovered first *subtract obligations* from FF — the FF intersector should only separate sides and report unknown contacts. This is the principled version of our shared-SSI gate and SEG-UNIFY work.
3. After any edge split, re-compare the pieces for coincidence (only against edges sharing a vertex) — Parasolid treats "smaller pieces may now be coincident when the whole was not" as a required fixpoint; our one-sided SEGLOST class is exactly this fixpoint missing.

**P4 — EE/EF interference**: intersectors must return *intervals of coincidence* (parametric ranges), not points, with the contract "one intersection per coincidence area, to (sum-of-)tolerance"; vertex-inheriting-split-edge-tolerance; degenerate-face removal (imprint creating two coincident vertices ⇒ merge + delete thin face) is part of imprint, not a post-pass.

**P5 — tolerance model**: adopt the four Parasolid invariants: (1) tolerances on topology only, geometry exact; (2) monotone hierarchy v≥e≥f with "accurate = ½ resolution" default; (3) all tests at sum of tolerances; (4) legality = transitive clustering + contractibility of tolerance-region intersections onto shared topology. Separate per-op budget (`default_tol`≈1e-5-equivalent, relative for us) and escalation cap (`max_tol`) from the model tolerances; escalate-then-compress (merge topology, larger tolerance wins) instead of failing; bound tolerance growth; provide `optimise` (shrink to measured deviation) and `close_gaps` (extend/trim pcurves, last-resort reparameterise) equivalents. Oversize-tool guidance and the short-edge rule ("edge entirely inside a vertex bubble") should inform our sliver classification.

**P6 — corpus validation**: reproduce the checker shape: sequenced check groups with early-out; graded fault taxonomy with machine-readable tokens + per-fault recovery hints; `max_faults=0` assertion mode for CI; scoped `check_fa` (faces adjacent to new section edges) as the cheap default after every boolean, full fa_fa reserved; a shuffle-debug mode to fuzz iteration order. Plus the failure contract: an op may return `failed` while leaving the arena dirty *only if* a checkpoint/rollback exists — add per-op arena checkpointing.

**Threading (later)**: classify ops concurrent/exclusive; parallelize only FF clash detection and per-face checking first; accept and *test for* nondeterministic result ordering.
