# session_cpp BRep Hardening Plan — Edge-Coherence Invariants

Ground truth files: `C:/pc/3_code/code_rust/session/session_cpp/src/brep.h` (H), `brep.cpp` (B), `file_step.cpp` (S). Model: OCCT BRep_TEdge/BRepLib::SameParameter + BOPAlgo PaveFiller section-edge discipline, adapted to our index-pool structs.

---

## 1. The Invariants (checkable predicates)

Notation: `E = m_topology_edges[e]`, `C = m_curves_3d[E.curve_3d_index]`, for trim `T` on face `F`: `PC = m_curves_2d[T.curve_2d_index]`, `S_F = m_surfaces[F.surface_index]`. `diag` = bbox diagonal. `CONF = max(1e-7, diag*1e-9)` (Precision::Confusion analog, scale-aware floor).

### I1 — SameRange: one parameter domain per edge
> **Predicate:** for every edge `e`: `E.first < E.last`, `C.domain() == [E.first, E.last]`, and for every trim `t ∈ E.trim_indices`: `PC.domain() == [E.first, E.last]` (within `1e-12` relative, OCCT CheckSameRange BL:145-179).

Requires new fields (§2). A trim's pcurve is a *representation of the edge over the edge's domain*, never a free curve with its own chord-length domain. Reversal is expressed ONLY via `T.reversed` (evaluate at `E.first + E.last − t`), never via a different domain.

### I2 — SameParameter: pointwise coincidence at equal t
> **Predicate:** for every edge `e` with `E.same_parameter == true`, for all `t ∈ [first,last]` (checked at 23 uniform samples): `|C(t) − S_F(PC_F(t*))| ≤ E.tolerance` for EVERY trim, where `t* = t` (or the reversal image). `same_parameter == true ⟹ same_range == true` (OCCT BRepCheck_Edge.cxx:91-94: SameParameter without SameRange is illegal).

This is the contract that kills the writer's entire compensation layer: the pcurve is the same curve re-expressed in UV, not an independently fitted twin (occt-sameparameter §THE INVARIANT).

### I3 — Measured, grow-only, ordered tolerances
> **Predicates:**
> - `E.tolerance == max(CONF, max over 23 samples of max_i |C(t) − S_i(PC_i(t))|)` — measured, not assumed (mirrors BRepCheck_Edge::Tolerance ×1.05, and FF ComputeTolReached3d).
> - `V.tolerance ≥ E.tolerance + 1e-12` for every incident edge (BOPTools DTolerance guard, AT:1663-1680), AND `V.tolerance ≥ |P_V − C(t_end)|` and `≥ |P_V − S(PC(t_end))|` for every rep endpoint of every incident edge (BRepCheck_Vertex; ON_BrepVertex rule).
> - No edge lies entirely inside the union of its two vertex spheres (`FindValidRange` rule PF6:907-930; Parasolid §16.3.2) — such an edge must not exist; fuse the vertices.

### I4 — Shared section edges: one curve bundle per FF pair
> **Predicate:** every 2-trim edge whose trims belong to faces originating from different operands has `T_A.edge_index == T_B.edge_index == e` referencing ONE `curve_3d`, and both pcurves were produced from the SAME intersection result over the same `[first,last]` — never two Hausdorff-reconciled independent fits.

OCCT: IntTools_Curve is born as {C3d, pcurve1, pcurve2, tol} with shared knots from one simultaneous fit (occt-bop-sections §1); the pave block is added to BOTH faces (PF6:1674-1675). Our `make_shared_section_edges` (H:268, B:4120-4381) is this — currently env-gated.

### I5 — Closed edges and seams are typed
> **Predicates:**
> - Seam edge (edge appearing twice in one face's loops on a closed surface): carries TWO pcurves, `pcurve2 = pcurve1 ± period`, forward occurrence uses pcurve1, reversed uses pcurve2 (BRep_Tool.cxx:338-341 convention). Splitting a seam edge preserves both pcurves period-offset (Builder_2.cxx:424-447).
> - Degenerate edge (pole/apex): `E.degenerated == true`, `E.curve_3d_index == -1`, pcurve only; `degenerated && curve_3d_index >= 0` is a violation (BRepCheck InvalidDegeneratedFlag).
> - Closed non-seam edge: `start_vertex == end_vertex`, and the vertex sits at a wire junction (not an arbitrary rebuild start).

### I6 — Topological coherence of loops
> **Predicates:**
> - One `BRepVertex` per geometric point: no two vertices with `|P_i − P_j| ≤ V_i.tol + V_j.tol`.
> - Within each loop, consecutive trims chain head-to-tail in UV: `|PC_i(end_i) − PC_{i+1}(start_{i+1})| ≤ uv_tol` mod period, where `uv_tol = max(1e-6, S.uv_resolution(E.tolerance))` (Parasolid loop-closure 1e-6; ShapeUpgrade CoordTol).
> - `check_trim_orientation()` (H:151) returns 0 — promoted from diagnostic to post-condition.

### I7 — Analytic identity carried, not re-recognized
> **Predicate:** every surface/curve created by primitives or booleans carries a type tag (`Plane/Cylinder/Sphere/Cone/Torus`, `Line/Circle/Ellipse`) + canonical frame; any derived sub-arc/split inherits the tag. `detect_analytic_srf` (S:2090) and the co_refine recognition lattice (B:3137-3210) become assertions, not derivations.

### I8 — Planar exception
> **Predicate:** trims on PLANAR faces may have pcurves regenerated at any time by exact normal projection (parameter-preserving, GeomLib::To3d inverse); on every NON-planar face the stored pcurve is mandatory and must satisfy I2. (BRep_Tool CurveOnPlane; STEP readers delete plane pcurves anyway — occt-step-write §5.)

---

## 2. Struct changes (H:25-56)

```cpp
struct BRepVertex {
    int point_index = -1;
    std::vector<int> edge_indices;
    double tolerance = 0.0;            // 0 = exact (Parasolid null convention); grow-only
};

struct BRepEdge {
    int curve_3d_index = -1;           // -1 legal ONLY when degenerated
    int start_vertex = -1;
    int end_vertex = -1;
    std::vector<int> trim_indices;
    double first = 0.0, last = 0.0;    // THE parameter domain, master for all reps
    double tolerance = 0.0;            // measured max same-t deviation; grow-only except in sameparameter()
    bool same_parameter = false;       // I2 certified
    bool same_range = false;           // I1 certified
    bool degenerated = false;          // I5
};

struct BRepTrim {
    int curve_2d_index = -1;
    int edge_index = -1;
    int loop_index = -1;
    bool reversed = false;
    BRepTrimType type = BRepTrimType::Boundary;
    int curve_2d_index_seam2 = -1;     // second pcurve for Seam trims (period-offset), else -1
};
```

Surface/curve type tags (I7): add to `NurbsSurface`/`NurbsCurve` or a side-table in BRep:
```cpp
enum class SurfKind { Freeform, Plane, Cylinder, Sphere, Cone, Torus };
struct SurfTag { SurfKind kind; Plane frame; double r1, r2; };   // parallel to m_surfaces
enum class CurveKind { Freeform, Line, Circle, Ellipse };
struct CurveTag { CurveKind kind; Plane frame; double r1, r2; }; // parallel to m_curves_3d
```
Serialization: all new fields JSON-alphabetical per project rules; default values keep old files loadable (flags false = "unverified", exactly OCCT's meaning).

---

## 3. `heal()` — the post-boolean pass that establishes I1-I8

```cpp
struct HealReport { int edges_refit, edges_bumped, vertices_fused, seams_rebuilt; double max_edge_tol, max_vertex_tol; bool ok; };
HealReport BRep::heal(const BRep* opA = nullptr, const BRep* opB = nullptr, double tol = 0.0);
```
Called at the end of `boolean()` (B:5193-5431) replacing the current co_refine→sew→(gated) chain. Ordered steps:

**Step 0 — shared section backbone (I4).** Promote `make_shared_section_edges(A,B,tol)` (B:4120-4381) to default; delete the `SESSION_BOOL_SHARED_EDGES` gate. Mirrors PaveFiller MakeBlocks: one IntTools_Curve bundle per FF pair, pave set shared, edge referenced by both faces (occt-bop-sections §3-4). Non-section coincident edges (same-operand splits) keep positional sew, but section edges never enter the Hausdorff path again.

**Step 1 — global vertex fusion (I3, I6).** One pass over ALL new vertices/edges (mirrors PostTreatFF's nested fuse, PF6:1161-1345): weld vertices within `max(V_i.tol + V_j.tol, q6)`; then micro-edge elimination via FindValidRange-equivalent — if `[first,last]` shrinks to nothing after clipping both vertex spheres, delete the edge and fuse its vertices, RECORDING the fused distance into the surviving vertex tolerance (fixes the silent-gap bug of micro-collapse at B:4462-4510).

**Step 2 — SameRange (I1).**
```cpp
void BRep::same_range_edge(int e);   // mirrors BRepLib::SameRange BL:183-262
```
For each edge: master range = 3D curve's `[first,last]` (3D rep wins, HXX:75-82); every trim pcurve affinely reparameterized onto it (`BSplCLib::Reparametrize` analog = knot-vector affine map — we already have knot rescale in NurbsCurve). Set `E.same_range = true`. Trimming/splitting an edge anywhere in the codebase must go through a single `set_edge_range(e, f, l)` that updates ALL reps at once (BRep_Builder::Range semantics, BRep_Builder.cxx:1064-1088) — retrofit `imprint_edges` and both split paths to use it.

**Step 3 — SameParameter (I2), generalizing `sameparameter_planar_pcurves`.**
```cpp
bool BRep::sameparameter_edge(int e, double tol);   // mirrors BRepLib::SameParameter BL:1183-1636
void BRep::sameparameter(double tol = 0.0);          // shape driver, mirrors InternalSameParameter BL:864-944
```
Per edge, per trim:
- a) measure `err = compute_tol(C, PC, S, 22)` — 23 uniform samples of `|C(t) − S(PC(t))|`, out-of-UV-domain overshoot penalized via surface resolution, result ×1.5, floor 1e-7 (ComputeTol BL:1006-1124).
- b) if `err ≤ tol`: done, `maxdist = max(maxdist, err)`.
- c) else build the monotone reparam map (Approx_SameParameter core, ASP:318-323): project 22 pcurve-image points onto `C` (Newton on `(P−C(t))·C′(t)=0`, 30 iters, global closest-point fallback — we have `NurbsCurve::closest_point`); REJECT non-monotone projections (`t_i > t_{i-1} + PConfusion`) — hard fail, this is the fold/faceted-boundary guard; interpolate a cubic 1D map `r: t3d → t2d` with end slopes `|C′|/|PC′|`; approximate `t ↦ PC(r(t))` as a 2D BSpline (maxdeg 11).
- d) **adopt only if the refit beats status quo** (`tolreached ≤ err`), else keep the old pcurve and absorb `err` into `maxdist` (the refit-vs-bump decision, BL:1554-1574).
- e) planar faces: keep the existing exact affine path from `sameparameter_planar_pcurves` (B:4646-4734) — exact, zero-tolerance, unconditional (I8); remove its deg-1×1-patch restriction by projecting in plane frame instead of patch CVs.
- f) analytic faces (I7 tags): pcurve = closed-form pullback of `C(t)` at its own parameters (we already have this machinery in the writer, S:2463-2545 — MOVE it into the kernel).
Finalize: `E.tolerance = max(maxdist, CONF)` — may SHRINK here, the one legal decrease (BL:1619-1631); `same_parameter = true` iff every trim passed; raise both vertex tolerances to `E.tolerance + 1e-12`.

**Step 4 — rep-upgrade atomicity (I2 hardening).** Rewrite sew's "Geometry rep upgrade" (B:4593-4620) as:
```cpp
void BRep::replace_edge_curve(int e, const NurbsCurve& c3, double f, double l);
```
which swaps the 3D curve AND immediately re-derives/re-verifies both trims' pcurves via `sameparameter_edge(e)` AND revalidates vertex containment — never "only the curve geometry moves". Same for co_refine's exact-arc rebuild (B:3494-3569). This deletes the co_refine "SameParameter guard" relift hack (B:3842-3901) — it becomes unreachable.

**Step 5 — seam/degenerate typing (I5).** Detect edges occurring twice in one face (seam) → populate `curve_2d_index_seam2`; detect zero-3D-extent pole runs → `degenerated = true`, null the 3D curve. Splits of closed edges re-install the second pcurve period-offset (DoSplitSEAMOnFace analog). Closed-edge vertex relocated to a wire junction here (kernel-side `edge_anchor`, absorbing S:2230-2236/3143-3161).

**Step 6 — tolerance ordering sweep (I3).** `InternalUpdateTolerances` analog: `V.tol = max(V.tol, incident E.tol + 1e-12, measured endpoint distances)`; verify no vertex-swallowed edges remain.

**Step 7 — loop closure check (I6).** Run `check_trim_orientation(false)` + UV chaining check; `HealReport.ok = (violations == 0)`.

Tolerance inputs to heal: working tol = `max(user tol, diag*1e-6)`; the diag-relative constants (`2e-3` lift B:2588, `5e-3` sew B:4401, `2e-4` pave, `on_eps` B:5288) survive only as SEARCH radii for matching, never as accepted geometric error — accepted error is always measured and stored.

---

## 4. BRepCheck-lite: `int BRep::check(bool verbose) const` — main_7 gate

Each check returns a violation count; the gate asserts total == 0 on every boolean/split result cell.

| # | Check | Predicate / tolerance | OCCT mirror |
|---|-------|----------------------|-------------|
| C1 | RangeValid | `first < last`; pcurve domains equal within `1e-12` rel | CheckSameRange BL:145-179 |
| C2 | SameParameterFlag | `same_parameter ⟹ same_range` | BRepCheck_Edge.cxx:91-94 |
| C3 | CurveOnSurface | 23 samples: `|C(t) − S(PC(t))| ≤ E.tol + prec_correction` where `prec_correction = max(curve_prec, surf_prec) ≈ CONF`; report `×1.00001` | BRepLib_ValidateEdge VE:29,50,67-76; BRepCheck_Edge InContext |
| C4 | MissingPCurve | non-planar face trim with no pcurve → violation; planar OK | BRepCheck_NoCurveOnSurface, BRep_Tool.cxx:346-349 |
| C5 | VertexContainment | for each incident edge & rep: `|P_V − rep(t_vertex)| ≤ max(V.tol, E.tol)` | BRepCheck_Vertex.cxx:137-232 |
| C6 | TolOrdering | `V.tol ≥ E.tol` for incidence; floor CONF on read | BRep_Builder.hxx:41-56 |
| C7 | MicroEdge | edge not inside union of vertex spheres: `chord_len(first,last) > V1.tol + V2.tol` | FindValidRange PF6:907-930 |
| C8 | DegeneratedFlag | `degenerated ⟹ curve_3d_index == -1`; converse for zero-extent | BRepCheck InvalidDegeneratedFlag |
| C9 | SeamPCurves | seam trims have both pcurves, offset = surface period within `uv_tol` | BRep_CurveOnClosedSurface |
| C10 | LoopClosureUV | consecutive trims: UV gap ≤ `max(1e-6, uv_resolution(E.tol))` mod period | Parasolid PK_LOOP_close_gaps; SUSD CoordTol |
| C11 | TrimOrientation | `check_trim_orientation() == 0` | BRepCheck_Wire Orientation |
| C12 | EdgeSharing | every interior edge has exactly 2 trims from 2 distinct faces; section edges have trims from both operands (when heal was given opA/opB) | BOPDS single pave-block index |
| C13 | VertexDedup | no vertex pair within summed tolerances | writer S:2222-2260 becomes a check |

main_7 wiring: after every boolean cell, `int v = result.check(false); record ok &= (v==0);` alongside the existing volume/face-count gates. During migration, run in WARN mode (count printed, non-fatal) per check until its enforcement step lands.

---

## 5. STEP writer deletions (S) and migration order

With I1-I8 held, `file_step.cpp` becomes a transcriber (occt-step-write §6): EDGE_CURVE = stored `C3d` verbatim; SURFACE_CURVE slots = stored pcurves (radian→degree on analytic surfaces retained); vertices = kernel vertices; loops = stored trim order.

**Deletable compensations (in dependency order):**
1. S:2646-2732 — trim-image rebuild of edge geometry + 9-sample kernel-curve trust test → replaced by trusting `C3d` (needs heal Step 4 + I2). *The single biggest one.*
2. S:2904-2915 — vertex-ignoring / built-curve-endpoint vertices → trust kernel vertices (needs I3/C5).
3. S:2463-2588 — export-time pcurve manufacture + kind-0 re-domaining fallback → serialize stored pcurves (needs I1+I2).
4. S:2222-2243 — location-based vertex dedup + edge_canon direction reconciliation → trust topology (needs C13, I6).
5. S:2743-2794 + 3143-3161 — edge_anchor rotation of closed curves → kernel places the vertex (heal Step 5).
6. S:2981-3176 PASS A re-chaining + trim_forward re-derivation (S:2325-2364) → emit stored loop order; keep `loop_material_left` only as a debug assert (needs I6/C11).
7. S:2073-2117 detect_analytic_srf + S:2795-2896 ISO circle re-derivation → read `SurfTag`/`CurveTag` (needs I7).
8. S:2044-2071 compress_curve at `diag*1e-7` — shrinks to near-no-op once rep curves are exact analytics/proper fits rather than 2000-CV polylines (I4+I7).
9. S:2386-2414 decimate_pullback — dies with (3).

**Keep (STEP-format-inherent, not compensations):** closed-edge split into two arcs (S:2634-2638, 2923-2973 — Rhino workaround, stricter than OCCT, correct per external-kernels D); SEAM_CURVE slot ordering (S:2607-2625); RadianToDegree; VERTEX_LOOP emission; plane-pcurve omission; degenerate-edge omission from EDGE_LOOP.

**Migration sequence (gates stay green throughout — matrix 60/60, battery 51/51, STEP re-import checks):**
each step = land kernel invariant → run `check()` in WARN → flip the corresponding writer path behind `SESSION_STEP_TRUST_KERNEL` env → A/B compare STEP output via the OCCT/Rhino oracle harness → make trust-path default → delete compensation.

---

## 6. Priority & effort

| # | Item | Effort | Payoff |
|---|------|--------|--------|
| P1 | I4: default-on `make_shared_section_edges`, delete Hausdorff reconciliation for section edges | **S** (code exists, B:4120-4381; needs battery pass to close gaps that made it gated) | Kills independent pcurve fits at the source; co_refine recognition lattice shrinks drastically |
| P2 | Struct fields (first/last, tolerances, flags, degenerated, seam2) + serialization + `set_edge_range`/`replace_edge_curve` plumbing | **M** | Enables everything else; mechanical but touches every construction path |
| P3 | I1+I2: `same_range_edge` + `sameparameter_edge` + shape driver; generalize planar/analytic pullbacks from writer into kernel | **L** (the reparam-map approximator is the one genuinely new algorithm) | Deletes writer items 1,3; deletes co_refine relift guard + param_near_3d searches |
| P4 | heal() orchestration + atomic rep-upgrade rewrite of sew/co_refine (Steps 1,4,6) | **M** | Deletes writer item 2; fixes silent micro-collapse gaps |
| P5 | I6 loop coherence as post-condition; writer PASS A removal | **M** | Deletes writer items 4,6 |
| P6 | BRepCheck-lite + main_7 gate | **S** (checks are 1:1 with predicates above; most measurement code exists) | Regression armor for P1-P5; do it in parallel with P2 |
| P7 | I5 seam/degenerate typing | **M** | Deletes writer item 5; fixes the Rhino open-loop class at the root |
| P8 | I7 analytic tags through booleans | **M** | Deletes writer item 7; removes double-recognition |

Recommended order: **P1 → P2+P6 → P3 → P4 → P5 → P7 → P8.** After P3 the kernel satisfies the same contract OCCT certifies with SameParameter=TRUE, and every remaining writer compensation is deletable on a green-gate schedule rather than a rewrite.

---

# Critique / amendments

Verified against current sources (brep.h/brep.cpp/file_step.cpp/intersection.cpp/main_7.cpp). Amendments below are concrete gaps or errors; everything not listed checks out.

## A. Missing pass entirely: merge_coplanar_faces (biggest omission)
1. `merge_coplanar_faces` (B:4736-5004) appears nowhere in the plan, yet main_7.cpp:262 runs it on the STEP-export copy of every matrix cell — the very files the §5 migration gates A/B-compare. It resamples trims at 256 samples, rebuilds every pcurve as deg-1 polylines (B:4969-4972), resets `reversed=false` (B:4976), and shoelace-classifies loops — destroying I1/I2/I5/I6/I7 *after* heal() ran. Amend: add a P-item to retrofit it SUSD-style (transport pcurves by exact plane frame change, preserve tags, re-run `sameparameter_edge` + `check()` on touched edges) or mandate heal() re-run after it; sequence against in-flight task #4 ("merge coplanar faces after boolean union").

## B. Freeform / marched SSI path
2. No contract on the intersector itself. `Intersection::surface_surface` (intersection.cpp:4637) already returns {C3d, pcA, pcB} triples, but nothing requires the three to share one domain/parameterization. The planar dispatch builds the planar-side pcurve via `Closest::surface_curve(a, c3)` (intersection.cpp:4668-4683) — an independent projection fit, exactly the bug class. Amend: add an SSI post-condition (all three curves same domain, same-t coincident within tol; marched fits emit all three from one sample chain, OCCT WLApprox-style) and certify bundles at birth; otherwise I4's "produced from the SAME intersection result" is unenforceable and P3's approximator runs on every section edge of every boolean.
3. SSI is computed 3× per face pair: A-split (B:2270, keeps only `get<1>`), B-split, and again inside `make_shared_section_edges` (B:4174). Marched curves are not reproducible across runs. Amend: PerformFF-analog — compute once before splitting, key bundles by (surfA,surfB), have `cut_for` and the pave engine consume the same stored bundle.
4. P1 payoff overstated: `make_shared_section_edges` refits section *3D* edges to shared sub-arcs but leaves both trims' pcurves from the two independent SSI runs. Amend P1: in Phase 4, install the bundle's pcA/pcB (sub-ranged by pave spans) onto the merged edge's two trims; otherwise I4/I2 on section edges wait for P3.
5. Step 0 wiring: current gated path *returns* before sew (B:5406-5409), and `secs.empty()` silently falls back to full Hausdorff sew (B:4198). Amend: reorder so same-operand edges still sew after the shared-section pass, and define behavior when SSI returns nothing for a genuinely intersecting freeform pair (fail loud vs legacy path).
6. Step 3c "non-monotone projection → hard fail" has no defined fallback. OCCT keeps the SameRange'd pcurve, IsSameP=0, bumps tolerance (BL:1575-1594). Amend: hard fail ⇒ `same_parameter=false` + measured tol recorded + WARN in check(), never a failed boolean.
7. Non-section 3D curves stay deg-1 chord polylines at devtol=diag*2e-3 (lift_loop B:2514-2557). Once C3d is master, that 2e-3 sag becomes the *certified edge tolerance* and writer-deletion #1 serializes 2000-CV polylines; §5 item 8's "compress becomes near-no-op" claim fails for freeform operands. Amend: add a BuildCurve3d-quality step — lift via pcurve∘surface approximation at working tol (deg≤14, GeomLib::BuildCurve3d analog) for non-analytic edges, or explicitly accept large tolerances and keep compress_curve.

## C. Seams / poles / periodic unwrapping
8. heal() has no pcurve UV-domain anchoring step (AdjustPCurveOnFace analog: ±period translation by *midpoint* test, cylinder special case dFi=tol/R, AT2D:254-407). The writer's `chart_lift_of`/`remap_samples` (S:2128-2220) compensate for exactly this and appear in neither the §5 deletable nor keep list. Amend: add heal Step 3.5 (anchor every pcurve into its face's UV window) and account for S:2128-2220 in the deletion schedule.
9. Seam handling only lands at heal Step 5 (post-boolean), but `imprint_edges` and both split paths split closed edges *during* the boolean; Step 2's `set_edge_range` retrofit doesn't mention re-installing pcurve2 period-offset on split (DoSplitSEAMOnFace, Builder_2.cxx:424-447). Amend: seam pcurve2 duplication belongs in `set_edge_range`/split plumbing (P2), not only heal Step 5.
10. Degenerate/pole typing must happen at primitive construction (`create_sphere`/`create_cone`, brep.h:99-103), with tags/flags/tolerances certified by construction — otherwise virgin primitives fail C8's converse and heal must rediscover poles each boolean.
11. Step 3 omits the OCC486 periodic clamp exception (don't clamp f3d/l3d into definition range when the basis is periodic, BL:1246-1252) — needed for cross-seam ranges.

## D. Predicate errors in the check table
12. C12 "exactly 2 trims from 2 distinct faces" is wrong for seam edges (2 trims, ONE face). Add seam exemption.
13. C7 uses `chord_len(first,last)` — zero for closed edges → false positive micro-edge on every closed edge. Use arc length or exempt `start_vertex==end_vertex` edges.
14. C1/C3/C8: spell out degenerated-edge exemptions (I1 references `C.domain()` unconditionally; C has no curve when `curve_3d_index==-1`).
15. C13/C12 fail legitimately on xor (disjoint two-shell assembly, coincident section vertices per shell) and multi-shell split output. Scope both per connected shell or exempt assembly results; wire heal+check into `boolean_split` fragments and xor (main_7.cpp:300-308), not just `boolean()`.

## E. Tolerances / vertices
16. Ordering bug in heal: Step 3 finalize and Step 6 GROW vertex tolerances after Step 1 fused; newly-overlapping spheres then violate C13/C7 with no re-fusion. OCCT closes this with CorrectToleranceOfSE (PF6:3907-4105, tolerance *shrink* to measured containment radius). Amend: add Step 6b (shrink pass or fixpoint re-run of Step 1's fuse/micro-edge check).
17. `BRep::transform()` (B:5843-5865) is untouched by the plan: must scale E.tol/V.tol by the map's max singular value (Rhino rule: tolerances are absolute model units), transform SurfTag/CurveTag frames+radii, demote analytic tags under non-uniform scale/shear, and state that affine maps preserve same_parameter flags.
18. Flag-invalidation discipline is absent: nothing says who clears `same_parameter/same_range` when a pcurve/curve/surface is mutated (add_curve_2d replacements, merge_coplanar, imprint splits, transform). Without it flags rot and Step 3's skip-if-certified (needed for perf, OCCT BL:1188) certifies stale state. Amend: enumerate mutators; only `set_edge_range`/`replace_edge_curve`/`sameparameter_edge` may set flags, every other mutation clears them.

## F. Backward compatibility / migration
19. Cross-language: brep exists in session_py (brep.py) and session_rust (brep.rs); project rules mandate identical structs/APIs ×3 plus session_proto schema change and to_proto/from_proto + file_json tests. §2 mentions JSON only — P2 is ~3× the stated effort, or the plan must declare an explicit C++-first exemption with py/rust field stubs so round-trips don't break.
20. Legacy data + STEP reader: `read_file_step_breps` exists (file_step.h:17). Old JSON and imported breps carry flags=false, tol=0 → C3 (floor CONF) fails on every legacy/imported shape. Define: unverified edges (flag false) are skipped by C2/C3 (OCCT semantics: flag false = no promise, not violation); check() gates only heal()-certified results; imports get a load-time sameparameter pass.
21. Volume-gate drift is why `sameparameter_planar_pcurves` is gated OFF today (B:5425-5429, "moves volume AWAY from truth"): heal Steps 3-5 mutate pcurves that `face_sample`/`mesh()`/`volume()` consume. §5's migration gates only writer flips; amend: each heal *step* gets its own env gate with matrix 60/60 + battery 51/51 A/B and an explicit acceptance rule (per-cell volume delta vs oracle must not regress) before default-on.

## G. Performance (plan is silent)
22. State the budget: heal is O(edges×23) samples + Newton reprojection only on failing edges — acceptable, but require (a) skip-if-certified via the flag discipline (#18), (b) SSI-once caching (#3) which removes the largest new cost and the 3rd marcher run, (c) check() reuses Step 3's measured deviations for C3 instead of resampling, (d) `make_shared_section_edges` O(facesA×facesB) AABB prefilter stays (already present, B:4168-4172).

Files: C:/pc/3_code/code_rust/session/session_cpp/src/brep.h, C:/pc/3_code/code_rust/session/session_cpp/src/brep.cpp, C:/pc/3_code/code_rust/session/session_cpp/src/file_step.cpp, C:/pc/3_code/code_rust/session/session_cpp/src/intersection.cpp (4637-4685), C:/pc/3_code/code_rust/session/session_cpp/main_7.cpp (249-308), C:/pc/3_code/code_rust/session/session_py/src/session_py/brep.py, C:/pc/3_code/code_rust/session/session_rust/src/brep.rs.
