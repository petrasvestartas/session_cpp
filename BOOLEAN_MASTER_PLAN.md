# BOOLEAN MASTER PLAN — the complete knowledge base and the endgame architecture
2026-07-23. Everything below is evidence-backed by a month of instrumented experiments, OCCT source
study, and oracle validation. This document supersedes patch-level plans (BOOLEAN_FIX_PLAN.md,
REMEDIATION_PLAN.md); it does not repeat their content, it distills what was PROVEN.

## 1. What we know (proven facts, not hypotheses)

### 1.1 The pipeline today (scaffold path, imported freeform × freeform)
1. `build_section_scaffold` (brep_section.cpp): ONE SSI per surface pair → canonicalized
   (p3, uvA, uvB) index-corresponded chains; pave census (trim crossings both operands,
   chain×chain crossings, vertex projections, closing paves); 3D-welded pave vertices;
   single keep-verdict per interval; valence-1 bridge (march/weld).
2. Per operand: `split_with` face loop → collect that operand's UV footprints of the shared
   chains (+ overshoot stubs, forced boundary nodes) → `split_by_uv_curves` UV arrangement →
   runs tagged {cidx, ta, tb} → `append_face` lifts section runs from the SHARED 3D chain
   sub-range; whole-segment runs keyed by seg_id; partial runs get span records.
3. Combine A2+B2: span-keyed alias merge of section edge copies; `normalize_section_blocks`
   (pave-block normalization); `co_refine_coincident_edges`; `sew_coincident_edges`
   (Hausdorff, tol=diag*5e-3); optional fuzzy ladder ×{1..48}; orphan collapse; island
   repair; capfill; wire repair; shell orient.
4. Classification: per-fragment sampled in/out of other operand (winding / contains_point_exact
   for freeform), angle method at section edges, connexity flood, seeded BFS parity,
   post-passes. Measured accuracy vs OCCT point oracle: 91–100% (100% on z15/x20/z30x20).
5. `face_outward_signs` tiers 1-4 (planar plane / cavity inversion / volume flux / ray parity)
   for orientation; volume() open-shell guard.

### 1.2 The three architectural roots (all failures trace here)
- **R1 — Section-network asymmetry.** The operands' section imprints DIFFER: one-sided
  emission (arrangement never lifts a seg the other emits — z90 segs 2,22), staggered
  partial runs (independent UV arrangements node crossings at different params), dangling
  valence-1 chain ends (junction undershoots, 0.07–0.47 gaps ≫ tol3), grazing divergence.
- **R2 — No shared-edge topology.** Section edge exists as TWO private copies matched
  post-hoc (span alias + Hausdorff sew). Failure poles: un-merged (naked edges) and
  over-merged (fuzzy ladder collapses distinct rims: z30 fuse 136→15.9). Orientation
  becomes unrecoverable post-hoc: at near-tangent section bands BOTH orientation signals
  are corrupt (geometric rel ambiguous |dp|<0.5, stored trim.reversed encodes pcurve dir
  not manifold traversal); 11 independent repair strategies each leave exactly 1 bad face
  (z90). The information is not locally present — only shared-edge radial coherence gives it.
- **R3 — No tangential contact path.** No TangentFaces analog; at grazing incidence the
  SSI marcher stalls/self-intersects (z90 SIW — fixed by NODESNAP for spurious crossings,
  but genuine tangency remains), x20-common is a 5e-4 lens, z15-common empty.

### 1.3 Approaches tried and their verdicts (do not re-try the dead ones)
- POST-HOC IDENTITY MERGE (13+ variants: qspan alias, normalize_section_blocks, SHAREDPAVE,
  span recovery, SD-weld, bridge, fuzzy ladder): reduces naked (6-32 → 0-10) but cannot
  close one-sided imprints (mate does not EXIST) and over-merges at high fuzzy. DEAD END as
  the cure; components remain useful as construction-time mechanisms.
- POST-HOC ORIENTATION (11 strategies incl. per-component robust re-vote): DEAD END,
  information-theoretic (ambiguous rel + corrupt flags + degenerate parity at section bands).
- POINT-CLASSIFIER UPGRADES (winding → exact trimmed-boundary classifier): fixed cone×cyl +
  z90 f2/f40; classification is now ~100% where the split is complete. NOT the residual.
- REFINE-AND-RESPLIT (FIX E): converges to 0 new segs — operands do not node chains at
  unpaved interior points; staggering lives at segment ENDS (~1% chord: stub clamps, trim
  snapping). Don't re-run.
- MESH-BOOLEAN CSG fallback (marching cubes SDF): works (chairs cut closed) but is not a
  B-Rep solution; keep as last-resort product feature, not the kernel path.
- Verdicts that MASK (orphan collapse pre-merge, micro-weld unguarded, forced closure):
  produce is_solid=1 but OCCT VALID=0. NEVER gate on is_solid; is_solid ≠ valid.

### 1.4 OCCT's answers (from source, validation/occt_oracle deps tree + C:\brg\compas_occt)
- PaveFiller computes ALL interferences (VV/VE/VF/EE/EF/FF) BEFORE any face is split;
  MakeSDVertices transitively welds coincident vertices; pave blocks are split at the
  union of all paves; a section pave-block is ONE edge with TWO pcurves referenced by
  both faces (CommonBlock). Dangling chords cannot exist by construction (every section
  endpoint is a welded valence≥2 node or a genuine open end pruned identically for both).
- BuilderFace prunes valence-1 chords exactly like our splitter — OCCT is not "more
  permissive"; it never SEES a false valence-1 because welding precedes splitting.
- Classification of split faces is point-based (ComputeState) like ours; orientation of
  the result is topological (radial edge-use / BuilderSolid), not probe-based.
- Fuzzy boolean = tolerance inflation at interference detection, not post-sew.

### 1.5 Validation doctrine (hard-won; violate = false confidence)
- GATE = external OCCT probe: `step_probe -c` (BRepCheck VALID), SOLIDS count vs
  OCCT_TRUTH.md (cut may legitimately be 2 solids: z15/z30/z45/z37/z63), volume vs OCCT
  within 1%, plus our topology_report (naked=0, manifold, single-shell where expected).
- OCCT_TRUTH.md is checked in (chairs dir); z15-common/x20-common are OCCT-degenerate —
  detect degeneracy, never gate on those volumes.
- Base regression set that must stay EXACT: base chairs cut/common/fuse 35/46.7943,
  <!-- provenance: the 46.8114 / 33.4951 / 127.0950 triple was produced by the boundary-insensitive bbox-Gauss integrator; BRep::volume() now uses the Green-reduced quadrature. Partition identity cut+common vs vol(A)=80.296862: 1.20e-4 old, 7.7e-7 new. -->
  25/33.5025, 50/127.0913; primitive matrix 45/45; SESSION_EDGE 54/54; minitests 760.
  <!-- provenance: the 46.8114 / 33.4951 / 127.0950 triple was produced by the boundary-insensitive bbox-Gauss integrator; BRep::volume() now uses the Green-reduced quadrature. Partition identity cut+common vs vol(A)=80.296862: 1.20e-4 old, 7.7e-7 new. -->
- Process: private exe copies (main_7_<tag>.exe), SESSION_NO_MERGE for chairs runs,
  parallel background runs with `wait`, tasklist not pgrep.

## 2. The architecture decision (authorized 2026-07-23)

**BOP2 — construction-time shared topology** (spec: chairs/BOP2_SPEC.md), replacing the
mint-twice + reconcile pipeline. The invariant: a surface–surface section is ONE edge =
one 3D curve + two pcurves, referenced by BOTH faces' wires; all junction vertices are
welded BEFORE any face is split. Mating and orientability hold by construction; the 6k
lines of reconciliation (alias/normalize/co_refine/sew/fuzzy/orphan/island/capfill/
wire-repair/shell-orient) are bypassed on this path and eventually deleted.

What BOP2 reuses (already built and validated): build_section_scaffold (SSI+pave+weld+
bridge), split_by_uv_curves (UV arrangement), the point classifiers (contains_point_exact,
inside_prim), face_outward_signs only as a cross-check, the oracle harness.

What BOP2 adds:
- Phase A (assembly): both operands split against the SAME pre-seeded shared arena
  (pool vertices + section edges). Section runs REFERENCE pool edges; operand-own
  edges land on pool vertices via seeded vertex map. One result BRep, no combine-merge.
- Phase B (pave completeness): two-pass split — pass 1 discovers each operand's clip
  params, ALL breakpoints become paves (cluster spansA∪spansB), pool is rebuilt with
  per-block edges, pass 2 re-splits so every run covers whole blocks. (Infrastructure:
  refine_scaffold_at_breaks + SHAREDPAVE shared_centers, both exist.)
- Phase C (classification): existing per-fragment point classification (measured ~100%)
  + section-block consistency: the four faces around each section block vote per operand;
  disagreement inside one operand = classification bug surfaced loudly, not masked.
- Phase D (orientation, the z90 cure): topological. Spanning-tree flood over the kept
  faces' shared-edge graph enforcing opposite edge-use per 2-trim edge; ONE global sign
  from signed volume. No geometric per-face probes.
- Phase E (tangency, last): TangentFaces analog for grazing pairs (x20/z15 lens cells) —
  detect near-parallel-normal spans pre-march, emit contact curve as shared boundary or
  declare degenerate-empty per oracle doctrine.

## 3. Milestones with gates (each gate blocks the next)
- **M0 — validation infra online.** step_probe currently READ_FAILs on ALL files including
  Rhino-authored chair0.stp (discovered 2026-07-23) → the oracle toolchain itself is broken
  (env/DLL/locale, needs root-cause). Rebuild/repair, verify against chair0.stp + OCCT_TRUTH
  regeneration spot-check. NOTHING can be claimed without this.
- **M1 — DONE.** Shared edge pool from scaffold (SESSION_BOP2), base cut unaffected.
- **M2 — base chairs via BOP2 assembly.** Cut/common/fuse EXACT (35/46.7943, 25/33.5025,
  <!-- provenance: the 46.8114 / 33.4951 / 127.0950 triple was produced by the boundary-insensitive bbox-Gauss integrator; BRep::volume() now uses the Green-reduced quadrature. Partition identity cut+common vs vol(A)=80.296862: 1.20e-4 old, 7.7e-7 new. -->
  50/127.0913) with NO sew/alias/normalize invoked on the BOP2 path. Proof the core works.
  <!-- provenance: the 46.8114 / 33.4951 / 127.0950 triple was produced by the boundary-insensitive bbox-Gauss integrator; BRep::volume() now uses the Green-reduced quadrature. Partition identity cut+common vs vol(A)=80.296862: 1.20e-4 old, 7.7e-7 new. -->
- **M3 — rotated battery closure.** All 10 rot configs × 3 ops: OCCT-VALID, solids per
  OCCT_TRUTH, vol within 1% where OCCT sound. Expected mechanism: pave completeness +
  shared edges kill the mating class; topological orientation kills the z90 class.
- **M4 — primitives + edge + minitests regression.** Matrix 45/45, edge 54/54, minitests
  green with BOP2 default-ON for the freeform path (primitives keep their exact recognizers).
- **M5 — random-rotation battery.** SESSION_ROT_RANDOM ≥ 20 seeds × 3 ops all valid-or-
  explained; step_crash_test set reads+validates in OCCT and Rhino.
- **M6 — deletion.** Remove the reconciliation pipeline + session gates from brep.cpp
  (target: brep.cpp shrinks by thousands of lines), port surviving architecture notes to
  the KB, update memory.

## 4. Failure playbook (when a gate fails)
- Naked edge on BOP2 path → dump the section block id + both operands' runs for that block;
  the defect is IN CONSTRUCTION (a run not covering a block / a missed pave), never in
  mating; fix the pave, never add a sew.
- Wrong volume → per-fragment verdict vs `step_probe --inside` point oracle (cls_audit.sh);
  if verdicts right and volume wrong → a fragment was never created (missing split) — check
  block coverage on that face.
- Bad orientation → cannot happen post-Phase-D by construction except through a non-manifold
  edge (trim count ≠ 2) — which is a construction defect; trace the block.
- Crash/hang → CUTCAP-class guards exist for coincident re-imprint floods; keep.

## 5. Current session state — 2026-07-24 (supersedes the 07-23 entry)
Milestone status: M0 DONE (oracle healthy; CLI order was the bug). M1 DONE. M2 DONE —
base cut/common/fuse EXACT (35/46.7943, 25/33.5025, 50/127.0913) with sew DISABLED.
  <!-- provenance: the 46.8114 / 33.4951 / 127.0950 triple was produced by the boundary-insensitive bbox-Gauss integrator; BRep::volume() now uses the Green-reduced quadrature. Partition identity cut+common vs vol(A)=80.296862: 1.20e-4 old, 7.7e-7 new. -->
M4 effectively DONE (matrix 0-FAIL, edge 0-FAIL, minitests green, NK-RESCUE default-on).
M3 is the active frontier; M5/M6 blocked on it.

Frontier after NK-RESCUE (cut, default no-sew): z30x20 0 | x20 9 | y30 13 | z63 13 |
z90 13 | z15 17 | x13y29 22 | z30 25 | z37 30 | z45 32. Sum 174 (was 196).

## 6. REMAINING PLAN (2026-07-24, evidence-ranked)

### 6.1 The one dominant defect class (fully diagnosed on x20)
The naked rims are per-operand realizations of ONE defect: the SSI marcher stalls
mid-face at grazing incidence (interior stop), leaving the chain end short of the
OTHER operand's TRIM boundary (x20: seg29 ends 0.128 from A's 16|18 border). Effects:
 (a) the end is never paved as a trim crossing on that operand;
 (b) the fed cut DANGLES inside that face -> the arrangement prunes the WHOLE segment
     -> one-sided imprint ([SEGLOST]: A lost segs 29/35/4/5; B never made seg21);
 (c) each operand closes the resulting open network its own way -> divergent bridges
     (~0.1-0.43 apart) that no weld may legally fuse.
Proven NOT the cause: classification (0/42 oracle), welding/tolerances (calibration
grid: no global optimum; knob gains always regress elsewhere), zero-span collapse
(ZEROFILL flat), scaffold dangles per se (bridge closes them: residual=0, naked stays).

### 6.2 Work queue (each with gate; measure 10-config battery + base 3 ops + matrix/edge)
1. **EXT_TRIM** (IN TEST, exe main_7_gk): extend interior-stopped chain ends along the
   true SSI (correct7 steps, forward-progress guard) until the A/B trim state flips;
   bisect the crossing; end the chain exactly there. Attacks (a)+(b) at the source.
   If positive -> default-on, re-audit frontier, re-run SEGLOST census (expect empty).
2. **Junction-knot fuse default**: bridge CASE-B (weld only, no CASE-A march) closes
   junction undershoots (x20 v28+v36 gap 0.0996 welds cleanly). Promote weld-only under
   its own gate if the 10-config battery is Pareto-clean (CASE-A march exposed z30/z37).
3. **Residual mate passes to defaults where Pareto-clean**: SYMLIFT (z63 -3, rest flat,
   guards green) and ON_QUORUM (z63 -4, base safe) after re-measuring ON TOP of 1+2.
4. **Re-audit every config post-1..3** with [SEGLOST]/[SEGAUDIT]/[NK]: the classes that
   remain get the x20 treatment (single-config full-trace root-cause, then mechanism).
5. **M3 close-out**: all 10 configs x 3 ops solid + OCCT BRepCheck VALID + vol vs
   OCCT_TRUTH within 1%; regenerate ALL inspection STEPs (user reviews every file).
6. **M5**: SESSION_ROT_RANDOM >= 20 seeds x 3 ops; step_crash_test reads+validates.
7. **Suite hard classes** (parked, distinct subsystems):
   - box x cyl coincidence family (ZC7/Z9/ZO2/ZN6/ZA8 + R-matrix cells): needs
     same-domain/ON-edge common blocks (OCCT MakeSD/PostTreatFF analog) — the biggest
     missing subsystem (see 6.3).
   - sphere pole-crossing empty result (bug25939): chart-singularity marching.
   - tangent-imprint face-count convention (class T); reimport-cone 2-naked pair.
   - R-cells (tilted-prim recognition): extend recognize_solid to rotated frames.
8. **M6 deletion**: retire sew/fuzzy/alias reconciliation from brep.cpp once M3+M5 hold
   with construction-only identity (target: -3000..5000 lines).

### 6.3 Industry-grade gap assessment (honest, 2026-07-24)
AT PARITY with OCCT on the studied domain: general-position transversal booleans of
analytic + freeform imported solids (matrix/edge batteries exact, base+z30x20 chairs
exact vs OCCT, point-classification oracle 0-mismatch, STEP round-trip exact).
ARCHITECTURALLY ALIGNED (the load-bearing part): shared section identity (pave/block/
pool), per-entity tolerance doctrine, construction-time mating, radial classification.
MISSING vs industry (OCCT/Parasolid), ranked by size:
 1. Same-domain subsystem: coincident/overlapping faces+edges as first-class common
    blocks (ON-classification, SD-vertex/edge/face merging). OCCT: ~10k lines. We have
    imprint hooks (extra_cuts) only. Blocks the coincidence suite family + flush caps
    beyond the recognizer path.
 2. First-class EE/EF interference stage (PerformEF/EE): we approximate via scaffold
    paves + EF diagnostics; increment 2 (EF-march) exists gated. Blocks tangency-heavy
    inputs and is the principled fix behind 6.2-1/2.
 3. Genuine tangential-contact path (TangentFaces analog) for extended grazing contact.
 4. Tolerance GROWTH model (fat vertices/edges accumulating per weld) — we have per-run
    bands but not persistent per-entity tolerance stored on the result.
 5. History (generated/modified/deleted maps), n-ary General Fuse, self-intersection
    handling, performance engineering (interference BVH partitioning, parallel PaveFiller).
 6. Corpus-scale validation: industry kernels are validated on 1e4..1e6 models; our
    battery is ~120 cells + chairs + primitives. The methodology (OCCT oracle diff)
    scales; the corpus needs building.
ESTIMATE: closing M3 (rotated chairs solid) is 1-3 mechanisms away (6.2-1..4). The
same-domain subsystem (6.3-1) is the largest single build (weeks at current pace).
Industry-grade breadth (arbitrary production models) additionally requires the corpus
campaign (6.3-6) — months, not days; but no further ARCHITECTURAL unknowns remain:
every remaining gap has a known OCCT design to port against.

## 7. THE INDUSTRY-GRADE PROGRAM (commissioned 2026-07-24: "even if it takes months")
Standing execution order. Each phase has a hard gate; a phase is entered only when the
previous gate holds. Every mechanism ships default-ON only after a Pareto-clean full
battery (10-config rot + base 3 ops + matrix + edge + minitests). The method that works
is LOCKED: one failing config -> full instrumentation trace ([SEGLOST]/[SEGAUDIT]/[NK]/
[SCAF-*]) -> name the mechanical cause -> smallest mechanism at the SOURCE -> measure ->
default or discard. No blind knob tuning (proven dead end, calibration grid 2026-07-24).

- **P1 = M3 (ACTIVE): rotated chairs all solid.** Queue in 6.2. Current: EXT_TRIM
  (in test), junction weld-only, SYMLIFT/ON_QUORUM promotion, per-config re-audit loop.
  Gate: 10 x 3 ops solid, OCCT VALID, vol<=1% vs OCCT_TRUTH, batteries green.
- **P2 = M5: generalization.** SESSION_ROT_RANDOM >= 20 seeds x 3 ops + reimport
  round-trip (result STEP -> read -> boolean again). Failures get the P1 loop.
  Gate: all valid-or-explained; zero unexplained.
- **P3: same-domain subsystem** (biggest build; OCCT refs: BOPAlgo_PaveFiller_4/5/6/7,
  MakeSD, BOPTools_AlgoTools coincidence, PostTreatFF). Design: detect coincident
  face pairs (orientation-aware overlap at tolerance), SD vertex/edge/face equivalence
  classes, ON-classification for section blocks (in/out/ON-same/ON-opposite), common-
  block edges carrying trims of BOTH operands' coincident faces, face-region booleans on
  the shared domain. Test bed exists: SESSION_OCCT2 coincidence family (ZC7/Z9/ZO2/ZN6/
  ZA8), A-op-A same-domain matrix cells, flush caps, box x cyl line contact.
  Gate: coincidence suite exact; no transversal regression.
- **P4: EE/EF interference first-class** (PerformEF/PerformEE analogs). Promote the
  EF Gauss-Newton point solver (proven machine-exact on x20 dangles) + EF-march from
  gated diagnostics into pipeline stages: EF paves added to chains BEFORE the verdict;
  EE vertices shared. Retire the interior-stop extension heuristics this obsoletes.
  Gate: z-family remnants + tangency battery; sphere pole cell (bug25939).
- **P5: tolerance model completion.** Persist per-entity tolerances ON THE RESULT
  (vertex sphere, edge tube, face band), grow at every weld (MakeSDVertices doctrine),
  fuzzy as an API parameter; reimport idempotence gate: boolean(STEP(boolean(X)))
  stable to 1e-6 volume.
- **P6: breadth + scale.** History maps (generated/modified/deleted), n-ary General
  Fuse + splitter, self-intersection guards, interference BVH pre-filter + parallel
  pave filling; CORPUS: generated randomized battery (1000s of cases: random prim
  pairs/triples x random rigid transforms x 3 ops, plus STEP zoo) diffed against the
  OCCT oracle nightly. Gate: >=99% corpus pass, no timeout class.
- **P7 = M6 + ports.** Delete the reconciliation pipeline (sew/fuzzy/alias/orphan
  repair) once P1..P4 hold construction-only; then port the kernel to Rust + Python
  per repo doctrine (C++ ground truth, identical APIs/tests); docs + KB consolidation.
Sessions are resumable: this section + memory project_bop2_construction_identity.md +
the charter memory ARE the program state. Restart recipe unchanged (private exe copies,
SESSION gates, scratchpad logs, OCCT oracle).

## 8. CONCRETE SUBSYSTEM SPECS (extracted 2026-07-24 from OCCT sources, kb/occt_*.md)
Seven function-level implementation specs live in kb/: pavefiller-core, interference-vef,
ff-posttreat-samedomain, builder-assembly, ssi-walking, tolerance-model, history-gf-scale.
Each has STAGE PIPELINE / DATA STRUCTURES / CONSTANTS / INVARIANTS / PITFALLS / PORT MAP.
The crosswalk that makes the phases concrete:

P1 (rotated-solid) — from kb/occt_ssi-walking.md + pavefiller-core:
- Stall completion is MINIMIZATION, not marching: IntWalk_PWalking::PutToBoundary ->
  SeekPointOnBoundary -> DistanceMinimizeByGradient (alternating 4-var gradient descent) +
  DistanceMinimizeByExtrema (per-surface Newton extremum) + domain clamp + locked-iso
  re-correction. REPLACE our stage-1b EXT_TRIM marching (correct7 fails at the stall by
  definition — that IS the stall). Snap band 1e-3*min(param range); hairpin delete.
- Graze continuation: ExtendLineInCommonZone — locked-iso stepping through the tangent
  zone, buffered commit, pi/4 zig-zag filter. Bad-point retry: rotate frozen iso (iso+1)%4
  up to 4x before halving step.
- SEGLOST adoption: IsExistingPaveBlock (midpoint+endpoint projection within tolR3D+fuzzy,
  thin cap min(0.001,10*tol), tangent |cos|>=0.9063) -> adopt existing edge + aPBFacesMap
  ownership + UpdateEdgeTolerance; then PutSEInOtherFaces (section edges pushed as IN
  edges into ALL faces containing them). This replaces ZEROFILL/SYMLIFT/NK-RESCUE band-aids.
- Never-drop rule: FillShrunkData/AnalyzeShrunkData — interval inside endpoint tolerance
  spheres => WELD endpoints (SD link), never emit, never silently drop.
- Junction acceptance: CheckArgumentsToExtend semantics — pi/6 triple angle gate,
  corrector-converged midpoint, seam/period/apex blocking.

P3 (same-domain) — from kb/occt_ff-posttreat-samedomain.md + builder-assembly +
history-gf-scale: BOPTools_Set edge-signature bucketing -> AreFacesSameDomain confirm
(interior-point projection, tol = tolF1'+tolF2'+fuzz) -> min-index representative
substituted into images -> BuildBOP orientation table (isSameOriNeeded = objState==
toolsState; CUT reverses tool faces; SD faces kept once iff orientation matches op).
PostTreatFF = nested pave-filler fuse over new section edges (existing edges in ONE
compound so they never re-intersect); aMEPB cache; aDMNewSD vertex map applied globally.

P4 (EE/EF first-class) — from kb/occt_interference-vef.md: IntTools_ShrunkRange end
bands; ComputeVE/VF accept dist <= tolV+tolX+max(fuzz,1e-7), growth = dist+tolX;
EdgeEdge golden-section box-tightening + CheckCoincidence + MergeSolutions;
EdgeFace MakeType (whole-range + chord > 2*criteria => EDGE else VERTEX);
ForceInterfEE/EF post-weld coincidence passes (angle gates cos 0.9063/0.4226);
PutPaveOnCurve + ExtendedTolerance (band grown to recorded common-part extent).

P5 (tolerance model) — from kb/occt_tolerance-model.md: fuzzy is threshold-only NEVER
stored (fuzz/2 per operand in pair tests); growth primitive UpdateTolerance =
max(current,new), monotone, geometry never moves; section-curve tol MEASURED
(ComputeTolReached3d = 1.00001*max |C3D-S(C2D)|, floor 5e-6 freeform); common-block tol
= max over 11 samples; final CorrectTolerances(0.05 cap) + hierarchy tolF<=tolE<=tolV.
Port: per-entity tol fields in the BOP2 pool; per-chain measured tol from correct7
residuals as the working radius; post-boolean correct_tolerances audit.

P6 (breadth/scale) — from kb/occt_history-gf-scale.md: intersection computed ONCE and
reused across ops (PerformWithFiller); tri-state image maps (unbound=identity, empty=
deleted, list=splits); classify-once myInParts + BuildBOP state table (xor/split free);
BOPDS_Iterator BVH self-join with same-argument pairs discarded; ArgumentAnalyzer
9-check preflight; CheckerSI self-intersection (analytic surfaces skipped);
history Modified/Generated/Deleted from image maps + result-membership filter.

Builder details (P1 wire assembly reference) — kb/occt_builder-assembly.md: IN/section
edges fed FORWARD+REVERSED (both-orientation tiling); WireSplitter min clockwise-angle
walk, back-edge=2pi, tie band 1e-14; boundary-vs-inside single-way override; valence-1
fixpoint pruning BEFORE walks; GetFaceOff radial mate (walk-refined bi-normal, MinStep3D
= 2*(tolE+tolF), ambiguity band Precision::Confusion -> classification fallback).
