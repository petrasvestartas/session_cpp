# PORT_ORDER — master implementation sequence for the rotated/curved/trimmed B-Rep boolean

2026-07-26. Decides: what is built in what order, by whom, gated on what, and what measurement
kills each step. Consumes the eleven subsystem specs (`kb/port_01..port_11`); does not restate
them. Read a spec only when you are implementing the increment that names it.

**Baseline being moved.** 224 in-memory primitive-pair cells under arbitrary rotation:
16 genuine passes, 15 of them box×box; every curved pair ≈ 0; box×torus and torus×torus hang.

**Two numbers, not one.** From INC 0 onward every report carries both:
- `N_gated` — passes with the repair chain live (`imprint_edges`, `snap_section_edges`,
  `co_refine_coincident_edges`, `run_xweld`, `sew_coincident_edges`, the fuzzy ladder,
  `SESSION_AUTO`). Today = 16.
- `N_clean` — the same battery with all of it compiled out. Today = unmeasured; predicted ≤ 6.

`N_clean` is the number this program optimises. A cell that passes only in `N_gated` passed by
coordinate accident and does not count as evidence for any increment. `N_gated − N_clean` is the
outstanding debt and must fall monotonically.

**Ordering principle.** Increments are ordered by *earliest measurable movement on curved cells*,
not by subsystem number and not by dependency depth. Foundation work that buys no cell is either
(a) pushed into a parallel lane, or (b) admitted at a numbered slot only when the next cell-moving
increment cannot be gated without it. Three increments (0, 1, 7) are explicitly labelled
"no cell movement expected" so nobody reads a flat number as a stall.

**Numbering vs calendar.** INC n is *merge order*. §4 gives the three-lane calendar; several
increments are worked concurrently and merge in this order.

---

## 0. Node inventory and one naming correction

| node | subsystem | sole producer of | sole consumer(s) |
|---|---|---|---|
| P01 | BDS arena, paves, pave blocks, common blocks, FaceInfo, iterator | integer identity; the place every later stage writes | P02, P03, P04·F6, P09, P10 |
| P02 | VV / VE / EE | fused vertices, paves, edge/edge common blocks | P03 (IntersectVE), P04·F6, P09 |
| P03 | VF / EF | *piercing vertices that are also paves*; FaceInfo In | P04·F8, P09 (mandatory nodes), P10 |
| P04 | FF section stage: restriction, block verdict, PostTreatFF | one section entity per geometric feature, on both faces | P09, P10 |
| P05 | analytic quadric SSI, arbitrary pose | exact conics/ALines for all 16 plane·cyl·cone·sphere pairs; typed verdicts; torus routing | P04 |
| P06 | numeric SSI (marching, WLine tooling, polyhedral seeding) — **slot reserved**, see below | section geometry for every pair P05 types `NoGeometricSolution`, and all non-coaxial torus | P04 |
| P07 | pcurves + same-parameter | the 2D image of every 3D curve, same-range, measured | P04, P08, P09, P10 (area/flux) |
| P08 | seams, poles, periodicity | seam pairs, degenerate edges, the four-clause closure rule, seam-split sections | P09, P10, P11(G3 partly) |
| P09 | face splitter | oriented face images that mint nothing | P10 |
| P10 | shell/solid assembly + selection | solids; the op table | result |
| P11 | tolerance model + classification | the metric, per-entity tolerance, tri-state IN/ON/OUT | everyone |

**Naming correction.** `kb/` holds ten spec files: `port_01..port_05`, `port_07..port_11`.
`port_06` is referenced from `port_01:2017` under the pre-renumber scheme and is not on disk as of
this writing. Its slot is fixed by *role*, not by number: the numeric marcher is the only producer
of section geometry for the pairs P05 declares `NoGeometricSolution` and for every non-coaxial
torus pose, and P04 is its only consumer. If the delivered `port_06` covers something else,
re-anchor it in §1 by its guarantees; nothing in §2 before INC 15 depends on it.
`port_02`'s summary was absent from the brief; the file exists and is used here (its I0–I7 ladder
is real and is scheduled at INC 7/9/11).

---

## 1. THE DEPENDENCY GRAPH

### 1.1 Hard edges, each with the guarantee that forces it

Read `X → Y` as "Y cannot be *gated* without X", not "Y cannot be typed".

| edge | forced by |
|---|---|
| P11·metric → P04, P07, P08, P09, P02 | every one of them converts a 3D tolerance to UV. P04·G6, P07·G8, P08·D7, P09·G7, P11·G1/G5 are the *same* guarantee stated five times. |
| P05·carried-descriptor → P07·chart → P08·periodicity | P07·G9 (typed degeneracy at poles) and P08·S12 (face UV bounds = surface natural bounds) both need the analytic frame that `RecogSurface` lacks (P07·D6: no reference X direction, so `u=0` is undefined). |
| P01 → P02 | P02·G5: a pave names a vertex *by index*. No arena, no index. |
| P01·step6 (FaceInfo) → P03 | P01·G8/D6: `FaceInfoIn` step 3 is the only place EF output can land. |
| P02·I1/I3 (VV+VE) → P03·E5 | P03·E5's `PerformNewVertices → IntersectVE → pave-block split`. EF's guarantee G1 clause (b) *is* a VE call. |
| P07·I0/I1 → P08·P4 | seam splitting root-finds `p_u(t) = u_seam` on the exact 3D curve (P07·I2). A fitted, rescaled polyline pcurve (P07·D2/D3) has no `p_u` to root-find. |
| P08·P0/P1/P2 → P08·P4/P5 → P09·P6 | P08's own S1–S4: a split of a seam must itself be a seam, which is only expressible once `seam_mate` exists. |
| P05 → P04 | P04·D12/D13: the dispatcher is P05's contract; P04·F7 *is* P05 restated. Do not implement F7 — implement P05 and delete F7. |
| P06 → P04·F5/F8 | domain-aware line construction and seeded walking are marcher features. |
| P07 → P09 | P09·G6: the sorting key is the *pcurve tangent*. There is no tangent on a sampled polyline that is not a chord (P09·D5). |
| P01·step8 → P01·step9 | P01's explicit warning: removing the coordinate weld without a shared arena takes box×box 15/20 → 0/20. |
| P01+P02+P03 → P09·step3/4 | P09·G1 (mint nothing) requires that every edge already exist somewhere. |
| P09 → P10 | P10·I-1: shells group by shared edge *index*. Minted edges have no shared index (P10's "32 naked of 36" root, declared unfixable inside P10). |
| P11·S7 (SolidClass3d) → P10·S2/S3/S5/S6 | selection is a lookup into a classification made once (P10·I-10). |
| P02 tolerance arithmetic → P11·S5 (per-entity tol) | P01's own OPEN item: `BRepVertex/Edge/Trim/Loop/Face` carry no tolerance field, so `tol(a)+tol(b)+fuzz` is currently unstateable. |

### 1.2 Four shared foundations that five specs each claim to own — MERGE THEM

This is the single largest decision in the file. Implemented as written, the specs produce three
divergent metrics and three divergent classifiers, after which no invariance gate is testable.

| foundation | file | absorbs | consequence of not merging |
|---|---|---|---|
| **F-A SurfaceChart** — carried analytic descriptor with frame, exact value/inverse/D1, `du_scale/dv_scale`, `u_resolution/v_resolution`, periodicity + pole table | `src/chart.{h,cpp}` (new) | P07·I0, P05·I0, P08·P0, P11·S1, P09·step1 (`BfMetric`), P02·I2 (curve half → `src/brep_curvetool`) | five metrics that disagree at the 1e-12 level; P08·E3, P09·T2, P11·T1.7 and P04·T6 are the *same* padded-domain test measured through four different converters |
| **F-B TolStore** — per-entity 3D tolerance, the three typed write kinds, harmonisation pass | `src/brep_tol.{h,cpp}` (new) + fields on `BRepVertex/Edge/Face` | P11·S5, P01's OPEN item, P02·G7/G10 growth rule, P07·G12 | P01 seeds every tolerance to a flagged 1e-7 forever; every "measured tolerance" guarantee stays aspirational |
| **F-C Classifiers** — `FaceClass2d` (tri-state IN/ON/OUT, band from F-A, exact-pcurve fallback) and `SolidClass3d` (edge/vertex ON pre-test, perpendicularity-scored ray, retry ladder) | `src/brep_classify2d.{h,cpp}`, `src/brep_classify3d.{h,cpp}` (new) | P11·S6/S7, P04·F3, P09·step9 `IsInside`, P10·S2, P03·E3's trimmed-face classifier (P03·G6 depends on it) | P03·G6 (no vertex on a face boundary) and P09's hole test disagree about ON; the CUT wall is kept twice or dropped twice |
| **F-D BdsArena** — P01 as written | `src/brep_bds.{h,cpp}` (header exists, 558 lines) | — | — |

Rule: **F-A and F-C are single-owner and merge before any consumer starts.** If a second copy
appears in a review diff, the copy is reverted, not reconciled.

### 1.3 Graph (hard edges only; F-A/F-B/F-C are foundations, not stages)

```
                      F-A SurfaceChart ──────┬────────────────┬──────────────┐
                             │               │                │              │
              F-B TolStore   │        P05 analytic SSI   P07 pcurves    P08 periodic
                    │        │               │                │              │
                    ├────────┴───────────────┤                ├──────────────┤
                    │                        │                │              │
   P01 arena ── P02 VV/VE/EE ── P03 VF/EF    └──── P04 FF ─────┘              │
        │             │            │                 │                       │
        └─────────────┴────────────┴─────────────────┴───────────┬───────────┘
                                                                 │
                                            F-C classifiers ─── P09 builder_face
                                                    │                │
                                                    └──────────── P10 builder_solid
                                                                     │
                                   P06 marcher ──→ P04 (torus / NoGeometricSolution only)
```

### 1.4 Non-dependencies worth stating (they are what makes the sequence short)

1. **EF is not required for sphere×sphere.** Every edge of a full sphere is its seam meridian; a
   point of that meridian lying on the other sphere lies on the section circle by construction.
   The EF piercing set *equals* the section's seam-crossing set, which P08·P4 produces
   structurally. This is the load-bearing fact behind §3.
2. **The arena is not required for the first curved win.** It is required for the win to be
   *structural* (see §3's weld-free variant) and for every later family.
3. **P05 (analytic completeness) is independent of the whole topology chain.** ALine, cyl×cyl
   semi-analytic and the coaxial arms can be built any time in lane A; they cash out only when
   INC 9 lands.
4. **P10 is independent of P05/P06/P07.** Replace every 3D curve with nonsense of the same
   topology and P10's decomposition must be bit-identical (P10·T2). It is gated on P09 alone.
5. **P06 (marcher) is on nobody's critical path.** It gates the torus rows and freeform only.
6. **P04·F7 is not work.** It is P05. P04·F1 is P07·I4. Delete both from P04's ladder.

---

## 2. THE IMPLEMENTATION SEQUENCE

### 2.0 Blocking matrix — which increment unblocks which family

| family (of the 224) | blocked by |
|---|---|
| box × box (15/20 today) | INC 8 |
| **sphere × sphere** | **INC 6** |
| box × sphere | INC 9, then INC 11 (six coplanar section circles must fuse into one node set on the sphere) |
| box × cylinder, box × cone | INC 6 (seam), INC 9, INC 10 |
| cylinder × cylinder (skew), cyl × cone, cone × cone | INC 14 (no dispatcher arm exists at all for cone×cone), INC 9 |
| cyl × sphere, cone × sphere (off-axis) | INC 14 (ALine), INC 9 |
| anything × torus | INC 4 (stop hanging), INC 15 (marcher), INC 10 (two seams) |
| every family | INC 2 (invariance), INC 8 (identity), INC 12 (assembly) |

### 2.1 Increments

Format: **BUILD / DRAWS ON / GATE / EFFECT (predicted delta, floor) / LANE**.
"Floor" is the delta below which the increment is killed (§5). All gates are oracle-free unless
marked ⊕ (analytically known constant, still oracle-free).

---

**INC 0 — Make it measurable and make it stop hanging.** *(no cell movement expected)*
- BUILD: 224-cell driver with per-cell wall-clock budget and typed outcomes
  {`SECTION_EMPTY`, `SECTION_PARTIAL`, `NAKED>0`, `NONMANIFOLD`, `VOLUME_MISMATCH`, `TIMEOUT`,
  `THROW`}; `measure_volume_ref` (refinement-converged divergence flux, cross-checked under a
  random rigid motion — the existing `volume()` integrates over the trim's UV *bounding box* and
  cannot be the oracle); the four standing invariants I1 partition identity, I2 rigid-motion
  equivariance, I3 A-op-A idempotence, I4 bbox containment; the `N_gated` / `N_clean` two-column
  report; FF provenance histogram; per-face mint counters; `SESSION_NO_SEW` naked count per cell.
- DRAWS ON: P11·S10 (+§5.0), P04·F0, P09·step0, P01·T7/step10.
- GATE: all 224 cells return a typed outcome inside budget; the 16 passes reproduce byte-identically
  across 20 runs; ⊕ `measure_volume_ref` matches analytic volume for box/cyl/cone/sphere/torus to
  1e-9 and is invariant to 1e-9 under 100 random rigid motions.
- EFFECT: `N_gated` = 16 confirmed; `N_clean` measured for the first time; torus rows become
  `TIMEOUT` rows. Every later delta becomes attributable.
- LANE: C.

**INC 1 — F-A SurfaceChart singleton.** *(no cell movement expected)*
- BUILD: `src/chart.{h,cpp}`; exact `Value/D1/Parameters` per chart kind; exact closed-form inverse
  with branch tracking; `du_scale/dv_scale`; `u_resolution/v_resolution`; `closed[]/period[]/pole_at[][]`;
  the **carried** descriptor (axis := R·a, apex := T·apex, radius unchanged) populated by the STEP
  reader and the six constructors; centre-onto-axis snapping at recognition time;
  `recognize_surface` demoted to fallback.
- DRAWS ON: P07·I0, P05·I0, P08·P0, P11·S1, P09·step1, P02·I2(curve half).
- GATE: `invert(value(u,v)) == (u,v)` to 1e-15 over 1e6 samples per kind modulo period;
  `Su·Sv == 0` to 1e-16; `du_scale` vs finite difference to 1e-9; ⊕ rotated cylinder's carried axis
  == R·a to 1e-15 (today: refitted at rtol 1e-3); ⊕ `u_resolution(1e-7)` on R=1000 == `2·asin(5e-11)`
  to 1e-15; knot-affine-map invariance to 1e-12; `closed/period/pole` match analytic truth for all six.
- EFFECT: none — no caller. This is the increment whose absence silently forks the program into
  five metrics.
- LANE: B (API frozen and published on day 1; A codes against the header).

**INC 2 — Model-space tolerances on the live path (ARCHITECTURE M0).**
- BUILD: retire every domain-relative constant, one call site per commit, in the order of P11·D2's
  table: `nurbssurface_trimmed.cpp:548-572`, `:1633-1647`, `:574`, `:569`, `:1653`, `:828`, `:848`,
  `:1425`, `:1650`; `brep.cpp:4262`, `:4280`, `:4297`, `:4350`, `:11373`, `:11574`;
  `brep_section.cpp:221`, `:1448-1449`, `:1889-1890`, `:2016-2017`. Each becomes a 3D quantity
  converted through F-A **per direction** at the point of use.
- DRAWS ON: P11·S2+S3, P09·step2, P04·F2, P07·I5, P08·P3(metric half).
- GATE: STEP round-trip box cell → 6 faces / naked 0 (today 2 / 8); one-operand padded split naked
  32-of-36 → 0; identical result at 1000× model scale; grep gate — zero literals multiplied by
  `range_u/range_v/(u1-u0)/(v1-v0)` outside `chart.cpp`; guards battery non-decreasing.
- EFFECT: curved faces stop being differentially mis-toleranced (a cylinder R=10 currently gets its
  V tolerance 10× too tight). Predicted +2..+6 on curved cells; **floor 0 on curved but the padded
  box cell is mandatory**.
- LANE: A. Must land before INC 3 — otherwise every later cell-level measurement is taken through
  a converter that is about to change.

**INC 3 — Exact functional pcurves.**
- BUILD: `PCurve::Pullback` — `value(t) = chart.invert(c3d.point_at(t))` with continuous branch
  tracking, `d1` by the orthogonal chain rule, domain **is** `[f,l]` (same-range by construction);
  route the boolean's plane pcurves through the exact affine control-point image; `compute_tol` +
  `validate_edge` wired as a **gate**, not a repair.
- DRAWS ON: P07·I1, I3, I4; P04·F1.
- GATE: `|C3d(t) − S(p(t))| ≤ 1e-13` at 64 samples for every analytic chart × every conic in every
  pose; zero `FitNurbs2d` provenance on any analytic surface; per-edge measured deviation recorded;
  deviation > 1.5·tol is a named hard failure, never a silent widen.
- EFFECT: removes the ~1e-4-relative pcurve error class from *every* curved face simultaneously
  (today: a non-rational cubic least-squares fit accepted against a UV-space criterion). Predicted
  +2..+8 — the curved cells whose section happens to avoid a seam. **Floor +1 curved cell**;
  if zero, the blocker is downstream and INC 6 must be pulled forward.
- LANE: A (engine in `src/pcurve.{h,cpp}`, B; wiring, A).

**INC 4 — Pose-free analytic sections + typed verdicts + bounded torus.**
- BUILD: P05·I1 shared primitives (`AxisRelation`, `dir_to_ax2`, `refine_dir`, the `qq::` constant
  block, six named epsilons replacing the single `kTol = 1e-6`); P05·I2 the four pose-free pairs
  (plane×plane, plane×sphere, plane×cylinder, **sphere×sphere**) on carried descriptors, with
  tangency typed (`Touch` + `Situation`) and `(trans1,trans2)` on every emitted curve; the routing
  half of P05·I7 — every torus pair and every `NoGeometricSolution` arm returns a *typed verdict*
  and, if it routes to the marcher, does so with a hard point cap and the
  `InfiniteSectionCurve` bail.
- DRAWS ON: P05·I1, I2, I7(routing), G1/G2/G5/G6/G8/G9.
- GATE: ⊕ emitted conics satisfy both implicit surface forms to 1e-13·scale² at 64 parameters;
  verdict type invariant and geometry covariant under 21 rigid poses; **box×torus and torus×torus
  complete every pose within budget with a typed verdict** (today: zero poses in 25 minutes);
  `handled == false` no longer exists as a state.
- EFFECT: sphere×sphere sections exact in every pose, tangency no longer silently empty; torus rows
  leave `TIMEOUT`. Predicted +0..+4 cells (the section is not the blocker for most rows yet), but
  ~28 torus cells become *reportable*. **Floor: termination is mandatory; cell floor 0.**
- LANE: A (`intersection.cpp`).

**INC 5 — Periodic representation: seams, poles, and an honest closure rule.**
- BUILD: `degenerate` flag on `BRepEdge`; poles become real degenerate edges in the four
  constructors and the STEP importer (`brep.cpp:464,474,530`, `file_step.cpp:1381`), deleting the
  four model-diagonal re-derivations (`brep.cpp:1091-1101, :1151, :7160-7190, :7355-7380`);
  `seam_mate` / `seam_pair_of` / `seam_pair_is_valid` populated at prepare time with S1–S4 asserted
  on **every input and every output BRep**; `is_solid` / `topology_report` rewritten to the
  four-clause rule (≥2 faces, or degenerate, or two pcurves on one face, or INTERNAL);
  `adjust_pcurve_to_face` + `adjust_periodic_interval` / `unwrap_near`, raw `fmod` banned.
- DRAWS ON: P08·P1, P2, P3.
- GATE: every corpus operand passes the input assertion; x4-padded domain gives bit-identical
  topology; full corpus non-decreasing.
- EFFECT: **the pass criterion becomes correct.** Expect the 16 to move by ±3 as cells that passed
  a length-threshold closure test are reclassified. Record the before/after per cell — this is a
  one-time deliberate baseline restatement, not a regression.
- LANE: A.

**INC 6 — Seam-split sections. ★ FIRST CURVED FAMILY ★**
- BUILD: exact seam crossings (root-find `p_u(t) = u_seam` on the 3D curve; assign period constants;
  one PCurve per period cell with its `(ku,kv)`); `split_pcurve_at_seams` on the **general** path in
  `src/brep_seam.cpp` — keep the closed flag and record the wrap count, take the period from F-A and
  not from the domain span, run on every section chain; `make_split_a_seam` variants A then B with
  the caller ladder — seam splits appended **twice**, degenerate splits **once** keeping orientation;
  the splitter emits `Seam` / `Singular` for the first time.
- DRAWS ON: P07·I2, P08·P4, P08·P5 (S2/S3/S4/S8).
- GATE: seam pcurve pair `Pf − Pr = (±T_u,0)` at 33 samples to 1e-12 relative; every split of a seam
  appears twice in the rebuilt wire; ⊕ sphere×sphere offset-along-axis 0/20 → 20/20; ⊕ sphere×sphere
  **arbitrary rotation ≥ 18/20 with naked == 0**; ⊕ partition identity `vol(A∪B)+vol(A∩B) =
  vol(A)+vol(B)` to 1e-12 on every completing sphere×sphere cell; ⊕ two-sphere union
  `2·(4π/3) − 5π/12`.
- EFFECT: the sphere×sphere row goes from ~0 to near-full across all four ops.
  Predicted **+8..+14**. **Floor +6.**
- LANE: A (engine B, wiring A).

**INC 7 — Arena + VV + VE + per-entity tolerance.** *(no cell movement expected)*
- BUILD: P01 steps 1–4 (`BdsBox/BdsIndexRange/BdsPair` + the full 81-entry type table; `BdsShape` +
  `init()` with ranges, boxes, wire→edge sub-shape rewrite; iterator, brute force first, LBVH behind
  it; paves, pave blocks, shrunk data, `release_pave_blocks`' three distinguishable states);
  P02·I0–I3 (VV with the exact two-ball enclosing sphere and the lexicographic n>2 sum; curve
  tooling — `curve_resolution`, `curve_box`, `proj_pc` with OCCT semantics and **no clamping**,
  `find_valid_range`; VE with SD-mapping at creation and the seam repeat-vertex rule);
  F-B `TolStore` populated read-only with the harmonisation pass and its assertions.
- DRAWS ON: P01·1/2/3/4, P02·I0/I1/I2/I3, P11·S5.
- GATE: 4×-padded box produces a **structurally identical arena**; box census (1 solid, 1 shell,
  6 faces, 12 edges, 8 vertices, every edge in exactly 2 faces, deg(v)==3); pave tiling exact
  (n blocks ⇔ n+1 paves, ranges sum to the edge range exactly, repeat-vertex append refused);
  ⊕ two-ball fusion `T = 0.5·(tol1+tol2+d)` internally tangent to 4 ulp; VE on a circular arc at
  `t = π/4` to 1e-12 with the pave's vertex index equal to the arena index; the non-clamping test
  (a vertex 1e-3 past the arc end must produce **no** pave).
- EFFECT: none — nothing calls it. Scheduled after the first curved win precisely so it is not on
  the critical path; it is lane-B work from day 1.
- LANE: B.

**INC 8 — The face splitter stops minting. ★ IDENTITY BECOMES STRUCTURAL ★**
- BUILD: absorb `SharedEdgePool` into the arena (`build_shared_edge_pool` → `append_vertex` /
  `append_edge` + `BdsFFCurve::blocks`), then replace `vmap`/`emap`/`bemap` identity for boundary
  runs; `BfInputEdge` assembly from the arena carrying **multiplicity** (boundary once in loop
  orientation, section/IN/internal/seam twice); `bf_emit_faces` looks up `input.edge` and deletes
  `lift_loop` / `find_or_add_vertex` / `q6`.
- DRAWS ON: P01·step8 then step9, P09·step3 then step4.
- GATE: minted edges == 0 and minted vertices == 0 on every corpus face; every result edge has
  exactly 2 trims; the instrumented counters on `sew_coincident_edges` / `run_xweld` /
  `imprint_edges` all read **zero**; `SESSION_NO_SEW=1` byte-identical to default;
  box×box `N_clean` reaches 20/20.
- EFFECT: every cell that passed by weld accident becomes either a structural pass or a visible
  failure; `N_clean` converges toward `N_gated`. Predicted **+5..+15** on `N_gated`, much larger on
  `N_clean`. **Floor: box×box `N_clean` ≥ 15.**
- LANE: A, on B's arena. **Step 8 strictly before step 9** (P01's own warning: step 9 alone takes
  box×box 15/20 → 0/20).

**INC 9 — VF/EF: piercings become paves. ★ LARGEST SINGLE JUMP ★**
- BUILD: `curve_surface` analytic dispatch (conic × {plane,cyl,cone,sphere,torus}) + append/transition;
  `ShrunkRange`/`FindValidRange`; VF with the UV-restricted projector, **strict-IN** classifier
  (F-C) and SD-collapse dedup; `EdgeFaceIntersector` exact-seed path, report-only first;
  the EF driver — per-pave-block loop, the VERTEX branch, `ForceInterfVF`, `CheckFacePaves`, the
  strict `IsPointInFace` gate, `PerformNewVertices` → `IntersectVE` → **pave-block split**;
  `UpdateFaceInfoIn` making `vertices_in` / `pave_blocks_in` **mandatory** arrangement nodes,
  which deletes `forced_boundary_nodes` and `scaf_forced_eps` outright.
- DRAWS ON: P03·E0, E2, E3, E4, E5, E8. Requires P01·step6 (FaceInfo) and P02·I3.
- GATE: ⊕ analytic piercing at `x = 1 + √0.07 = 1.2645751311064590` produces both a vertex **and** a
  pave that splits the block; the sampling sweep's "edges split" column non-zero for the first time
  at N=200 and N=2000; the strict-IN vs corner-ON discrimination; 4×-padded-domain interference
  records byte-identical; box×box stays 20/20; box×sphere leaves 0/20; the z15 `[SEGAUDIT]`
  asymmetric-segment count → 0.
- EFFECT: unlocks every curved pair with cross-piercing — box×sphere, box×cyl, box×cone, cyl×cone,
  cyl×sphere, cone×sphere. Predicted **+30..+80**. **Floor +20.**
- LANE: B builds `src/brep_curvesurface.*` and the driver; A wires the single call site.

**INC 10 — Periodicity inside the UV arrangement.**
- BUILD: hand `SurfacePeriodicity` to `split_face_by_wires`; replace 3D-position vertex bucketing
  (`nurbssurface_trimmed.cpp:1788-1811`) with exact parametric identification (two UV points
  identify iff separated by an integral period in a closed direction, or lie on the same collapsed
  iso); delete `seam_tol` and `seam_v[]`; stop dropping `Singular` trims
  (`brep.cpp:11477`, `:11674`) and exclude them only from the classification polygon.
- DRAWS ON: P08·P6.
- GATE: one-operand padded-domain split naked 32-of-36 → 0 **with every repair pass compiled out**;
  cylinder/cone/torus cells whose section crosses a seam close.
- EFFECT: predicted **+5..+15**. **Floor +3.**
- LANE: A.

**INC 11 — EE, common blocks, PostTreatFF: one entity per geometric feature.**
- BUILD: P01 steps 5–7 (CommonBlock + PB→CB map + measured `ComputeToleranceOfCB` over 11 strictly
  interior samples; FaceInfo In/On/Sc; the interference log with the same-domain hop cap that OCCT
  does not have); P02·I4–I6 (EE line/line, EE general, EE common blocks with the `has_same_bounds`
  gate) and I7 (`RepeatIntersection` + `ForceInterfEE`); P04·F4 (block-level verdict at the 43.2%
  point — delete `SESSION_CONN_STUB`, `SESSION_ON_QUORUM`, `SESSION_VERDICT_EPS_MULT`, `drop_iv`,
  and the 40-iteration bisection whose endpoints have no entity behind them), F5 (domain-aware line
  construction — delete the overshoot stubs `brep.cpp:4401-4450`), F6 (cross-pair fusion:
  `is_existing_pave_block` #1 and #2, the nested VV/VE/EE pass over section edges,
  `put_se_in_other_faces`; degrade to "no fusion" on failure rather than aborting the stage).
- DRAWS ON: P01·5/6/7, P02·I4/I5/I6/I7, P04·F4/F5/F6.
- GATE: no two emitted section edges lie within `tol1+tol2+fuzz` along their overlap, asserted over
  all 224 cells; zero `bisection` provenance tags; ⊕ two circles crossing transversally produce
  exactly two shared vertices; A-op-A stays green; every result edge in exactly two faces.
- EFFECT: multi-face pairs — box×sphere's six section circles meeting at the box's corner
  piercings, box×cone, cyl×cone. Predicted **+10..+30**. **Floor +8.**
- LANE: B (engines) + A (call sites).

**INC 12 — Assembly and selection by identity.**
- BUILD: `Shell`/`Solid`/`OrientedFace`; `make_connexity_blocks` on shared edge index only; closure
  by **edge parity** (not "2 trims everywhere"); `orient_faces_on_shell`; `SolidClass3d` (F-C) with
  the ON pre-test, the perpendicularity-scored ray and the retry ladder; `perform_infinite_point` /
  `IsHole` / innermost-parent nesting (delete `comp_cavity` and its O(components²) mesh work);
  `InPartsMap` computed once in the split stage; `select_faces` with **both** fences (unoriented and
  oriented), both avoid rules, and a real CUT21; the result becomes `vector<Solid>` over one arena.
- DRAWS ON: P10·S0, S1, S2, S3, S5, S6, S7; P11·S6/S7.
- GATE: replace every 3D curve with nonsense of the same topology — decomposition bit-identical;
  ⊕ box with a **spherical** cavity integrates `64 − 4π/3`; selection performs **zero** point
  classifications (assertion counter); the shared-wall four-op table including CUT vs CUT21 normal
  asymmetry; the rotated two-lump family yields exactly 2 closed manifold solids with no face image
  in two solids; grazing battery invariant under 100 random rigid motions.
- EFFECT: cavity and multi-lump curved cells; kills the "box cut inscribed sphere reads 97.5 instead
  of 30.5" class. Predicted **+10..+25**. **Floor +6.**
- LANE: B (all new files) + A (one call site).

**INC 13 — Angular key, path, areas, orientation inheritance.**
- BUILD: `Angle2D`/`Tolerance2D` from the pcurve tangent replacing the chord `atan2`; `Path` with
  backtracking and the two override rules; `RefineAngles`; `PerformAreas` with the signed-UV-area
  hole test and re-discretisation; `PerformInternalShapes`; face orientation inherited, not derived.
- DRAWS ON: P09·steps 5–11.
- GATE: no angular ties at valence-4 nodes; a curvature-matched exactly-tangent section splits the
  face; seam-crossing wires close; determinism across 20 shuffled input orders.
- EFFECT: tangency and high-valence junction cells. Predicted **+5..+15**. **Floor +3.**
- LANE: A.

**INC 14 — Analytic completeness: ALine, coaxial arms, semi-analytic cyl×cyl.**
- BUILD: P05·I3 (plane×cone incl. the apex branch, frame flip, the 1e9/2e6 extreme-conic give-up,
  apex-ray splitting); I4 (cyl×cone, cyl×sphere, cone×sphere, **cone×cone all five cases**,
  cyl×cyl both ladders); I5 (`ImplicitQuadric` + the trig root finder with the θ=π special case and
  the ×1e-4 rescaling loop + `AlineCurve` + `explore_curve`, wired into the four
  `NoGeometricSolution` arms); I6 (`CylCylCoeffs` semi-analytic per-U1 evaluator with the 1000-point
  cap and the `V-range > 1e5·R` bail).
- DRAWS ON: P05·I3, I4, I5, I6.
- GATE: ⊕ every emitted curve satisfies both implicit forms to 1e-13·scale² at 64 samples; the
  currently-251%-wrong cyl×cone case exact; cone×cone produces a curve at all; equivariance across
  21 poses; nappe purge by **axis parameter**, never by distance from the apex.
- EFFECT: the cone and skew-cylinder rows, which today have no dispatcher arm.
  Predicted **+20..+40**. **Floor +12.**
- LANE: A. Independent of INC 5–13; may be built any time and merged when INC 9 has landed.

**INC 15 — Marcher, torus, and the pole stage.**
- BUILD: P06 (polyhedral seeding, walking, WLine tooling, `DecompositionOfWLine`, ALine→WLine with
  pole/apex extension) behind P05's typed routing; P04·F8 seeded walking from EF points; P08·P7 the
  pole stage (`ProcessDE` with `FindPaveBlocks`/`FillPaves`/`AddSplitPoint`/`MakeSplitEdge`, plus
  `choose_pole_parameter` generalised from the tangent-plane construction — OCCT throws for any
  quadric other than sphere and cone, so freeform poles have no port to copy).
- GATE: ⊕ torus volumes to 1e-9; box×torus and torus×torus non-zero; sphere-through-pole yields
  closed shells; pole parameter assignment deterministic.
- EFFECT: the ~28 torus cells plus freeform. Predicted **+10..+25**. **Floor +6.**
- LANE: A + B.

**INC 16 — Delete the repair chain. ★ TERMINAL GATE ★**
- BUILD: remove `imprint_edges`, `snap_section_edges`, `co_refine_coincident_edges`, `run_xweld`,
  `sew_coincident_edges`, the fuzzy ladder (`brep.cpp:10912-11030`), `SESSION_EF_PAVES`,
  `SESSION_EF_MARCH`, `SESSION_AUTO`; demote `SESSION_EF_DIAG` into C's validator harness; delete
  the weighted-vote global shell flip and `SESSION_SHELL_ORIENT`.
- GATE: with all of it **compiled out**: base 3 ops exact, matrix 45/45, edge 54/54, rotated chairs,
  minitests green, and `N_clean == N_gated` on the 224 battery.
- EFFECT: the number becomes real. Any drop here is the accumulated weld debt made visible.
- LANE: A.

---

## 3. THE CRITICAL PATH TO THE FIRST CURVED SUCCESS

**Claim.** The first curved cell to produce a valid closed solid is **sphere × sphere CUT under
arbitrary rotation**, and the minimum set of increments before it is:

> **INC 1 → INC 2 → INC 3 → INC 5 → INC 6** (with INC 0 alongside, to know that it happened).
> INC 4 joins the path *only if* INC 0's provenance histogram shows the sphere×sphere section is
> not already exact.

Excluded from the path, deliberately: the arena (INC 7), no-mint (INC 8), VF/EF (INC 9), EE/common
blocks/PostTreatFF (INC 11), the solid builder (INC 12), the marcher (INC 15), analytic ALine
(INC 14).

**Defence.**

1. **Sphere × sphere is the only curved pair with no cross-piercing.** Both operands' entire edge
   set is one seam meridian (their poles are degenerate). Any point of A's seam that lies on B lies
   on both spheres, hence on the section circle. So the EF piercing set is *exactly* the set of
   points where the section circle crosses the seam — which INC 6 computes structurally, by
   root-finding `p_u(t) = u_seam` on the exact 3D circle, once, for a point that is then shared by
   both images of the seam. Every other curved primitive has non-seam boundary edges (cylinder and
   cone have cap circles; the box has twelve) whose piercings are not on any seam, and those cells
   cannot close before INC 9. Torus is likewise EF-free but needs the marcher.
2. **Its section needs no new intersector.** Sphere×sphere is one of the four pairs with no
   coaxiality gate in OCCT or in ours; it is a circle in closed form in every pose. No ALine, no
   semi-analytic evaluator, no marching, no seeding.
3. **It generates exactly one FF pair.** One face against one face; there is nothing for cross-pair
   fusion (P04·G4) to fuse, so INC 11 is not required. This is not true of box×sphere, where six
   coplanar section circles must resolve to one shared node set on the sphere.
4. **Its assembly is one shell per operand piece.** Each sphere splits into exactly two faces
   meeting along one section edge plus the two seam pieces; the result of a CUT is one closed shell.
   No cavity nesting, no growth/hole classification, no connexity blocks beyond the trivial —
   INC 12 is not required for closure, only for the general case.
5. **What is left is precisely what INC 1/2/3/5/6 build:** an exact chart, tolerances that do not
   depend on a UV domain that *is* the period on a periodic surface, an exact same-range pcurve
   whose seam crossing can be root-found at all, a representation in which a seam and a pole are
   first-class, and a splitter that splits a seam into seams. That is the shortest possible chain
   to a closed curved solid, and P08's own gate for it is quantitative: `sphere × sphere` arbitrary
   rotation ≥ 18/20 with `naked == 0`.

**The weld-free variant — read this before declaring victory.** With the repair chain live, INC 6
can produce a closed sphere×sphere because the two operands' independently lifted copies of the
section circle now agree to ~1e-13 and `sew_coincident_edges` binds them. That is still coordinate
identity; it is the accident this program exists to remove, and it will not survive box×sphere.
Therefore the success criterion is `N_clean`, not `N_gated`:

> **Structural critical path = 1 → 2 → 3 → 5 → 6 → 7 → 8.**
> INC 6 is declared successful only if sphere×sphere also passes with `SESSION_NO_SEW=1`. If it does
> not, INC 7+8 are on the path and are pulled in front of INC 9 immediately.

Both variants are worth having in view: 1–6 gives the *first measurable curved movement* (the point
of the ordering), 1–8 gives the *first curved result that cannot regress by accident*.

---

## 4. WHAT PROCEEDS IN PARALLEL — three sessions

**Ownership, restated as a non-overlap rule.**

| lane | owns | never touches |
|---|---|---|
| **A** | `brep.cpp`, `brep_section.*`, `intersection.*`, `nurbssurface_trimmed.*`, `main_7.cpp` — i.e. every wiring point and every deletion | new modules under `src/` owned by B; `validation/`, `bash/` |
| **B** | new files only: `chart`, `brep_tol`, `brep_bds`, `brep_curvetool`, `brep_curvesurface`, `pcurve`, `brep_seam`, `brep_classify2d/3d`, `brep_shell*`, `brep_inparts`, `brep_select`; already owns `brep_samedomain` (A-op-A green) and `brep_commonblock` (73/73) | `brep.cpp` and the four other A files — **B never edits the hot path** |
| **C** | `validation/`, `bash/`, corpus generation, the 224 driver, CI | `src/` entirely |

**The one interface rule that makes this work:** B ships engines as *pure functions over data*
(chains, arenas, point sets), A wires the single call site. B may not add a call from a new module
into `brep.cpp`; A may not implement an algorithm inside `brep.cpp` that a spec assigns to a module.

**Day-1 API freeze (before any implementation).** B publishes, as headers with stubs, on day 1:
`chart.h` (F-A), `brep_tol.h` (F-B), `brep_classify2d.h` / `brep_classify3d.h` (F-C),
and the `BdsArena` public surface. A codes INC 2/3/5 against those headers immediately. Without
this freeze A blocks on B for two increments.

**Concurrent schedule.**

| phase | A | B | C |
|---|---|---|---|
| 1 | (waits ~1 day on `chart.h`; meanwhile INC 4's routing/termination half, which needs nothing) | **INC 1** F-A, then F-B, then `pcurve` engine | **INC 0** driver, `measure_volume_ref`, invariants, two-column report |
| 2 | **INC 2** then **INC 3** (call sites) | **INC 7** arena + VV/VE (long, isolated) | corpus: 21-pose analytic battery, padded-domain cells, determinism harness |
| 3 | **INC 5**, **INC 6** ★ | INC 7 continues; `brep_seam` engine for INC 6; F-C classifiers | grazing battery, classifier-vs-oracle set, per-increment attribution report |
| 4 | **INC 8** (needs INC 7 merged), **INC 10** | **INC 9** engines (`brep_curvesurface`, EF driver) | EF acceptance battery (the analytic piercing constants), sampling sweep N=200/2000 |
| 5 | **INC 14** (fully independent, can start any time in idle slots), **INC 13** | **INC 11** engines, **INC 12** shell/solid/select | volume batteries for cavity and two-lump families |
| 6 | **INC 15**, **INC 16** | INC 12 finish, INC 15 marcher half | terminal gate: full corpus with repairs compiled out |

**Merge barriers — only three exist.**
- B1: `chart.h` frozen before A starts INC 2. (API only, not implementation.)
- B2: INC 7 merged before A starts INC 8. Violating this is the documented 15/20 → 0/20 failure.
- B3: INC 9's engines merged before A deletes `forced_boundary_nodes` / `scaf_forced_eps`.

**What must NOT be parallel.** F-A and F-C. Five specs independently specify the metric
(`BfMetric`, `u_resolution/v_resolution`, `uv_tol_at`, `chart.du_scale`, `curve_resolution`) and
four independently specify a classifier. Implemented concurrently by different sessions, the result
is four converters that disagree at 1e-12, at which point P04·T6, P08·E3, P09·T2 and P11·T1.7 — the
*same* padded-domain test — return four different answers and none of them is diagnostic. Single
owner, merged first, no exceptions.

**C is never blocked and never blocks.** Every gate in §2 is a query C's harness answers. If C
cannot express an increment's gate as a measurement, that increment is not ready to start.

---

## 5. KILL CRITERIA

Global rules, applied before the per-increment table:

- **K-A** An increment whose gate is met only with the repair chain enabled has *not* met its gate.
- **K-B** No increment may add a snapper, a weld, a quantizer, a coordinate hash, or a
  Hausdorff match. If the fix requires one, the increment is wrong at the design level — revert and
  re-derive; do not merge with a TODO.
- **K-C** No increment may introduce a threshold expressed in UV-domain extents. The grep gate from
  INC 2 runs in CI from INC 2 onward.
- **K-D** Any increment that lowers `N_clean` on *any* cell is reverted the same day, regardless of
  its effect on `N_gated`.
- **K-E** Below-floor delta (§2) = revert, not patch. The floors are predictions to be falsified; a
  miss means the causal model in this file is wrong for that family, and the correct response is to
  re-measure with INC 0's typed outcomes, not to add compensating code.

| INC | measurement that kills it | revert vs patch |
|---|---|---|
| 0 | `measure_volume_ref` cannot reach 1e-9 against analytic primitives, or is not rigid-motion invariant | **Stop the program.** Every acceptance gate downstream is an invariant on volumes. Fix the reference integrator first. |
| 1 | any consumer ships a second metric or a second periodicity table | revert the copy, never reconcile |
| 2 | padded box cell does not reach 6 faces / naked 0 | revert. M0's premise (domain-relative epsilons are the cause) is falsified for that cell; re-attribute with the mint counters before re-attempting |
| 3 | residual > 1e-13 for oblique conics on cyl/cone/sphere/torus | revert to fit **for that chart kind only** and fix the inverse's branch tracking. Never "snap" the pcurve to the surface |
| 4 | torus cells still exceed budget, or a previously-correct analytic pair changes its emitted geometry | revert. A typed give-up that terminates is a success; a faster wrong answer is not |
| 5 | more than ~3 of the current 16 passes are reclassified as failures | do not relax the four-clause rule globally — bisect which clause fires, and confirm against the operand's input assertion. If the operand itself fails S1–S4, the constructor is the bug |
| 6 | sphere×sphere passes only with the weld on (K-A) | not a kill, but a **re-route**: INC 7+8 move ahead of INC 9 |
| 6 | sphere×sphere `naked > 0` after seam splitting, or a seam split that is not itself a seam | revert P5, keep P4. P4 alone must still fix the axis-offset case 0/20 → 20/20; if it does not, the seam period is still coming from the domain span somewhere |
| 7 | 4×-padded arena is not structurally identical | do **not** tighten a quantiser — something in `init()` is still reading coordinates. Find it before writing any consumer |
| 8 | box×box `N_clean` < 15 | revert immediately. Most likely cause: FaceInfo On/In is empty so the splitter cannot find an edge it needs. Step 9 without step 8 is the known 0/20 |
| 9 | new vertices appear without a corresponding pave (the "edges split" column stays flat while the vertex count rises) | revert E5. G1 clause (b) is the whole point; a vertex with no pave is the current `SESSION_EF_PAVES` failure re-created |
| 9 | box×box regresses, or any cell's naked count rises | revert. EF is producing out-of-trim piercings — the strict-IN classifier (F-C) is not wired, or `IsPointInFace` is the permissive variant |
| 10 | padded one-operand split does not reach naked 0 **with repairs compiled out** | revert. The arrangement is still identifying UV points by 3D position somewhere |
| 11 | two section edges still lie within `tol1+tol2+fuzz` along an overlap on any cell | revert F6's fusion but keep F4/F5. A partial fusion is worse than none: it makes duplicate curves rarer and therefore harder to attribute |
| 11 | A-op-A goes non-green | revert. Same-domain (73/73) is the strongest existing evidence for the common-block model; breaking it means the CB representative or the SD map is now order-dependent |
| 12 | nonsense-geometry decomposition is not bit-identical | revert S0. Adjacency has re-acquired a geometric term — the shell walk is reading a curve |
| 12 | selection performs any point classification (assertion counter > 0) | revert S6. I-10 is what makes selection auditable; a re-classifying selector cannot be tested for double-booking |
| 13 | determinism fails across 20 shuffled input orders | revert. An order-dependent angular key makes every subsequent cell measurement noise |
| 14 | any emitted curve fails the implicit-form residual at 1e-13·scale², or equivariance fails across the 21 poses | revert that arm only. Arms are independent; a wrong cone×cone arm must not block a correct cyl×cyl |
| 15 | the marcher produces geometry that fails the same 1e-13 residual, or a torus cell exceeds budget again | revert to INC 4's typed give-up. A typed `Failed` is a shippable state; a hang is not |
| 16 | `N_clean` < `N_gated` after the repairs are compiled out | do not restore the repairs. The gap is the debt: attribute it per cell with the typed outcomes and open an increment per family. Restoring the chain re-hides exactly what this program removed |

---

## 6. Standing gauges (run at every increment, from INC 0)

1. `N_gated` and `N_clean` over the 224 cells, with typed outcome per cell.
2. `SESSION_NO_SEW` naked-edge count per cell — **monotonically non-increasing** across the whole
   program. This is the single most sensitive regression detector available and it is oracle-free.
3. Minted edges / minted vertices per source face — must reach 0 at INC 8 and stay there.
4. Repair-chain invocation counters — must reach 0 at INC 8 and stay there.
5. The four invariants: partition identity, rigid-motion equivariance, A-op-A idempotence, bbox
   containment.
6. The guards battery: base 3 ops exact, matrix 45/45, edge 54/54, minitests, chairs.
