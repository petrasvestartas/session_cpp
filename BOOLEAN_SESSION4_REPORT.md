# Boolean Pipeline — Session 4 Report (2026-07-21, ultracode)
Rotated-chair booleans (`serialization/boolean_steps/chairs`). Builds on Session 3
(BOOLEAN_PIPELINE_REPORT.md, BOOLEAN_FIX_PLAN.md). Working tree on top of 86bf50e.

## 0. Executive summary
The rotated-configuration failure decomposes into **two independent problems**, established by
six code/OCCT-research agents plus direct measurement:

1. **MATING** (naked edges): the pipeline traces each section curve **twice** (each operand runs
   an independent 2D arrangement on the shared chain), so staggered / one-sided / divergent
   section edges cannot mate. **Substantially fixed this session** (naked 6–32 → 0–10; z90's shell
   closes) via a unified valence-1 bridge (dangle closure) + an automatic fuzzy ladder (tolerant
   mating tolerance). The full architectural cure is one-shared-edge-two-pcurves (OCCT model).
2. **CLASSIFICATION** (which faces to keep): point-in-solid sampling is unreliable at grazing, and
   generalized winding is wrong on the rotated freeform mesh (which self-overlaps). Wrong faces are
   kept → wrong volume + orphan shells. **This is the remaining primary blocker** and is precisely
   diagnosed below with a concrete fix (KB law 7: angle-primary classification).

**"Fundamental / OCCT-unsound" is REFUTED.** OCCT computes each face-pair section ONCE as one edge
with two pcurves inserted into *both* faces' wires ("no second trace can diverge"), layered with a
fuzzy tolerance for near-tangency. Every failure class here is a signature of the opposite
(two-independent-traces + Hausdorff sew) architecture. The only genuinely ill-posed case is an
*exact transversal section at exact tangency*, answered by a shared/fat tolerant edge, not conceded.

## 1. Mating — measured (cut op, naked edges)
| cfg | baseline | +bridge | +bridge+ladder(cap16) | +fuzzy(single-shot high) |
|-----|----------|---------|-----------------------|--------------------------|
| z90 | 8 | 8 | **0** (ladder mult 2) | 0 |
| x20 | 9 | 6 | 1 | **0** (fz48, 35 faces, no over-merge) |
| x13y29 | 19 | 17 | 1 | 2 |
| y30 | 12 | 12 | 3 | 1 |
| z30 | 22 | 30* | 4 | 1 |
| z37 | 26 | 23 | 6 | 3 |
| z45 | 32 | 20 | 8 | 2 |
| z15 | 17 | 17 | 9 | 1 |
| z63 | 20 | 20 | 10 | 3 |
| z30x20 | 0 | 0 | 0 | 0 |
(*bridge alone regresses z30 by closing dangles that expose deeper one-sided imprints — see §3.)

The fuzzy ladder finds each config's **minimal** closing tolerance (z90 at mult 2 = ftol 0.142, not
the manual 16); face counts stay constant across the sweep → the mating merges are **not**
over-merging distinct faces.

## 2. Classification — the remaining blocker (evidence)
Naked=0 does **not** imply a correct solid. Two measured symptoms:
- **x20** closes (solid 1, naked 0, 35 faces) at high fuzzy but with **vol 51.5355**. x20 is
  near-tangent (its cut should remove almost nothing → ≈ 80), so removing ~29 units means the
  classifier keeps the **wrong faces** (it treats rotated B as still deeply interlocking).
- **z90** closes its main shell (naked 0) but leaves a **disconnected 1-face orphan shell**
  (`shell@38 V=0 E=0 F=1`) — a misclassified face bounded entirely by section edges, invisible to
  the island-repair (which excludes section edges).
Root: `classify()` is point-sampling-PRIMARY with the deterministic angle method demoted to a
correction (`brep.cpp:7433-7586`), and generalized winding (`contains_point:1046-1074`) is wrong on
the rotated self-overlapping mesh. Fix (KB law 7, agent-specced): make the angle method authoritative
at section edges, seed the parity phase from angle-certified fragments (not sampled score), demote
winding/exact-PIP to a redundant agree-not-outvote check. Plus the `oth_face_of_surf.emplace`
first-face-wins hazard (`:7445-7448`) for any operand with >1 face per surface (benign for chairs).

## 2b. Gating decision (2026-07-21)
All closure changes are **opt-in** so the default boolean stays byte-identical to baseline (no silent
approximate/possibly-misclassified closures): `SESSION_BRIDGE` (bridge), `SESSION_FUZZLADDER`
(fuzzy ladder, cap `SESSION_FUZZLADDER_CAP`=16), `SESSION_ZEROFILL`, `SESSION_SEGKEEP`. The
"aggressive tolerant-closure" mode = `SESSION_BRIDGE=1 SESSION_FUZZLADDER=1`. Classification
experiment: `SESSION_ANG_PRIMARY` (angle-primary parity seed, §2c). Verified: default tree base cut
35/naked 0, z90 naked 8 (= baseline); matrix/edge byte-identical by construction.

## 2c. Angle-primary classification attempt (SESSION_ANG_PRIMARY)
Implemented per KB law 7: the section-bounded-block parity phase is seeded from fragments with strong
ANALYTIC angle evidence (`ang_abs>0 && |ang_sum|/ang_abs>=0.5`) rather than the sampling-derived
`score`; only components with no angle evidence fall back to score. `brep.cpp` parity block.
**Result: SAFE but INEFFECTIVE (negative result).** base+angle cut = 46.8114 EXACT (does not break
base). But x20 cut vol = **51.5354 — UNCHANGED** (should be ≈80; near-tangent → cut ≈ A). x20's
parity already classifies without fallback (comps=1, done=35) and the orphan shell persists, so the
classification error is **NOT in the parity SEED**. It lies deeper: either the connexity-flood block
PARTITION unites fragments across an unrecognised separator (any phase then propagates the error), or
the angle evidence itself is ambiguous at grazing, or the orphan is a shell-walk ORIENTATION artifact.
Kept gated (`SESSION_ANG_PRIMARY`) as documented evidence. **The classification fix is therefore
larger than the KB-law-7 seed change — it needs correct block partitioning and/or an exact
point-classifier, not just angle-primary seeding.**

## 3. Implemented improvements (all gated opt-in; base/matrix/edge/minitests invariant)
- **Unified bridge** (`brep_section.cpp` phase 4b, default-on `SESSION_NO_BRIDGE`): CASE A same-pair
  local SSI re-march with verify-or-refuse (append symmetric `SectionSegment`, else leave residual);
  CASE B cross-pair-only midpoint fuse. Residual counter in `[SCAF] bridge(march/weld/resid)`.
- **SEGFALL identity guard** (`nurbssurface_trimmed.cpp:1315`) + **zero-span fill** (`brep.cpp:3182`):
  preserve section identity / emit the whole shared segment when an operand collapses it to a slit
  (Law 5). Marginal alone (most one-sided imprints are arrangement-verdict disagreement).
- **Automatic fuzzy ladder** (`brep.cpp` after the `SESSION_FUZZY` block, default-on
  `SESSION_NO_FUZZLADDER`, cap `SESSION_FUZZLADDER_CAP`=16): escalate `ftol=diag*5e-3*mult` through
  {1,2,4,8,12,16,24,32}, stop at the minimal mult that closes. No-op when already closed. OCCT's
  fuzzy-value auto-escalation. NOTE: closure is a *tolerant* solid (approximate by design); it does
  NOT fix classification, so a closed shell may still have the wrong faces (see §2).

Invariants held: base cut/common/fuse EXACT (35/46.8114, 25/33.4951, 50/127.0950); matrix 45/45 and
edge 54/54 byte-identical by construction (primitive cells never take the scaffold path); z30x20 stays
a closed solid; the ladder/bridge are no-ops on all of these (zero dangles / already closed).

## 4. Approaches explored (this session + prior) and why each is insufficient alone
pave-blocks (identity re-key), orphan-collapse, parity-classify, volume-guard, scaffold-bisection,
junction-weld, same-pair SSI re-march, cross-pair-restriction, exact-PIP classifier, island-repair,
SEGFALL guard, zero-span fill, P1 shared-edge engine, fuzzy fallback, **fuzzy ladder**. Each fixes
ONE mechanism; the residual is multi-mechanism (mating + classification), and the mating fixes are
repairs of a two-trace architecture rather than the single-shared-edge construction guarantee.

## 4b. Steps 1–3 investigation (2026-07-21)
**Step 1 — Flood audit: the flood is CLEAN.** Added `[FLOOD-AUDIT]`/`[FLOOD-BRIDGE]` (gated
`SESSION_FLOOD_AUDIT`, `brep.cpp` after the connexity flood): counts broad-recognised section edges
whose two fragments land in the SAME union-find block. x20: A blocks=4 sec2trim=28 **cross-block=0**
naked-sec=9; B blocks=5 cross-block=0 naked-sec=1. The flood does **not** bridge any mated section —
so the wrong volume is NOT a partition bug. The real signal is **naked-sec=9 on A2**: nine section
edges that are 1-trim (unmated *within* operand A) = faces the section **failed to split**.

**Step 2 — Classifier is NOT the primary cause.** An unsplit oversized fragment straddles BOTH inside
and outside B, so **no point classifier** (winding, exact-PIP, or ray-cast) can classify it correctly.
(No ray-surface primitive exists; per-face meshes suffer the same self-overlap `contains_point_exact`
already avoids, so a ray-cast is not an improvement.) `contains_point_exact` does have real grazing
bugs — `brep.cpp:1123` skips a face when the closest UV lands just outside its trims → picks a farther
face with wrong sign; `:1132` `dp<0` is a coin-flip when the probe grazes the boundary — but these are
SECONDARY. HARD EVIDENCE the classifier is not the cause: x20 cut vol = 51.5354 WITH exact-PIP and
51.5917 WITHOUT it (winding fallback) — **identical**; angle-primary (parity-seed change) also leaves
it unchanged. The wrong ~29-unit removal is the unsplit oversized fragments, not the point test.

**Step 3 — Shared section edge is the UNIFIED cure.** Step 1 unified the two problems: the result-
mating naked edges and the classification wrong-volume have ONE root — **incomplete section splitting
within each operand**. The scaffold-completeness flags (bridge/segkeep/zerofill, which act pre-
classification) do NOT reduce A2's naked-sec (still 9) — the incompleteness is in A2's **arrangement**
(`split_by_uv_curves`), which emits 1-trim section edges where a section grazes / only partially cuts
a face. The cure is the OCCT model: **one section edge with two pcurves, made authoritative over both
operands' arrangements** (symmetric total imprint), so every section is a 2-trim splitting edge in BOTH
operands — fixing mating and classification simultaneously. Concretely: `split_by_uv_curves` must
CONSUME the scaffold's shared section edges (forced interior + boundary nodes at the shared pave
params) rather than re-derive crossing nodes independently in each operand's UV. This is the largest,
highest-value change (touches the arrangement every cell uses) and needs its own validation cycle.

## 4c. Step-3 session (2026-07-21): cone×cyl regression fixed + boundary-reaching stub
Building a full regression harness (minitests+matrix+edge+base) surfaced that the WIP tree had
**regressed the primitive matrix**: `cone x cyl` cut/common/fuse FAIL (our cut 9.6866 vs OCCT 2.6180).
Bisected: passing on uc16/weld3, failing s5+. Cause isolated by env-toggle: **`SESSION_NO_EXACT_PIP`
fixes it** → the session-3 `contains_point_exact` (default-on) misclassifies cone×cyl (tilted cone3 is
unrecognized → routed through the freeform exact-PIP). Root = the exact bug the agent flagged
(`brep.cpp:1123`): when the surface-closest UV lands OUTSIDE a face's trims, the face is SKIPPED → a
farther face's wrong outward-normal sign is used.

**FIX (default-on, `SESSION_NO_EXACT_PIP2` to disable):** when the surface-closest point is off-trim,
fall back to the closest point on the face's **trim boundary** (`closest_on_trim`), two-pass (in-trim
first — byte-identical to legacy; then boundary-sample only off-trim faces closer than best, sorted so
it prunes fast; ns=24 coarse). **VALIDATED: matrix 45/45 (0 FAIL — cone×cyl recovered 2.6180/14.1372/
45.0295 exact), edge 0, minitests 760/760, base chairs 35/25/50 solid (topology preserved).** This
recovers a passing test AND is the correct robust classifier. It does NOT fix z90's orphan (still vol
62.04, shells 2) — z90's orphan is a shell-walk ORIENTATION artifact, a distinct issue.

**Boundary-reaching stub (Step 3, `SESSION_STUB_REACH` opt-in):** the 1-trim (unsplit-face) root cause
is a fixed-length overshoot stub that lands SHORT when the trim boundary is farther than `ov` (undershoot
band `ov<d_f≤5ov`) → the cut fails to cross → dangling-prune deletes it → face emitted UNCUT. Fix =
project onto the boundary (`foot_bnd_sc`) + overshoot past the foot; guard `d_f>ov` keeps `d_f≤ov`
byte-identical. Base chairs EXACT. Mixed on rotated configs (x20 mating 9→6 and closes with
bridge+fuzzladder; z37 stub-only 26→37 worse) → gated opt-in. Correction: x20's classification was
already 100% correct (x20 is OCCT-unsound, so its vol ~51.5 is fine); x20's residual is grazing-sliver
MATING (fuzzy), not classification.

## 4d. Architectural attempt: span-recovery (evidence-negative, decisive)
Implemented `recover_section_spans` (`brep.cpp`, `SESSION_RECOVER`): the seg_id-identity pave-block
merge (`normalize_section_blocks`) is *complete for chain-lifted edges but blind to legacy-lifted
section edges* (undershoot/SEGFALL/one-sided — no span record). Recovery gives those edges a span by
matching them geometrically to their scaffold segment, so *every* section edge enters the identity
merge. **Result: spans recovered (z90=7, z37=18, z30=4) but naked did NOT drop** (z37 +1). This is
**decisive evidence**: no post-combine merge can close the residual, because the residual edges are
**one-sided (the mate does not exist) or grazing-divergent (the mate diverges past any merge tol)** —
the merge needs the mate to *exist* and *coincide*, which only construction-time sharing provides.
Gated opt-in.

### The architectural cure, precisely characterized (OCCT BOPDS pave-block model)
Root of one-sided imprints: the scaffold clips each section to the **inside-both** region, but a
section segment whose endpoint is **interior to one operand's face** (the *other* operand's trim
clipped it) is a **dangling chord** on that operand → the arrangement's dangling-prune deletes it →
the face is emitted **uncut** (oversized fragment). OCCT avoids this by cutting each face with the
**full** section curve (clipped to that operand's own trim, not inside-both), splitting fully, then:
1. **Symmetric split**: split the ONE shared section chain at the **union** of BOTH operands' trim
   crossings (the scaffold already paves both, `n_paves_trimA + n_paves_trimB`); keep **per-operand**
   sub-segments (inside-A for A's cut, inside-B for B's), tagged. The **inside-both** sub-segments are
   in both → mate by identity; the inside-A-outside-B sub-segments split F_A but are operand-internal.
2. **Internal-edge removal**: after classification, remove section edges separating two
   **same-classification** pieces of the **same** operand (the inside-A-outside-B remnants) — they are
   interior to a kept region, not solid boundary.
This is a bounded but multi-part change to the scaffold verdict (per-operand keep flags on segments) +
`append_face` (lift per-operand segments) + a new post-classification internal-edge-removal pass. It is
the correct realization of "one shared section edge"; it is scoped, not blocked.

## 4e. OCCT source study + config closures (2026-07-21)
With the full OCCT source available (`validation/occt_oracle/build/deps/occt/.../TKBO/`), a detailed
study of `BOPAlgo_BuilderFace`, `BOPAlgo_PaveFiller_6` (PostTreatFF/MakeBlocks), `BOPDS_PaveBlock`,
`BOPAlgo_Tools` (MakeSDVertices) delivered the **decisive root cause**: OCCT's face splitter PRUNES
dangling chords too (`BOPAlgo_BuilderFace::PerformShapesToAvoid`) — it is NOT more permissive than the
target's prune. The reason OCCT faces still split: a section endpoint interior to a face is **never
valence-1** in the arrangement, because the DS-level `PostTreatFF`/`MakeSDVertices` runs BEFORE splitting
and **welds every coincident section endpoint (across all surface pairs) into one shared valence-≥2
node**. The target's dangling chord = an endpoint that should be a shared FF-junction (surface-pair A∩B
transitioning to A∩B' at a shared B-edge) but is un-welded (two marchers undershoot) → false valence-1 →
pruned.

**Ports implemented (this delivered config closures):**
- **SD weld** (`brep_section.cpp` 4a2, `SESSION_SDWELD`): OCCT `MakeSDVertices` — transitive union-find
  weld of **valence-1** section endpoints in 3D (`p3[]`), aDist/2 tolerance bridge, re-derive `uvA/uvB`
  by projecting the one shared 3D point onto each surface. (First attempt re-projected correct junctions
  too → base 35→127 corrupted; fixed to valence-1 only → base EXACT.) Helps the FF-junction dangle subset.
- **0-trim orphaned-edge skip in `is_solid`** (`brep.cpp`): an edge with 0 trims is a dead alias/merge
  record, not a manifold boundary edge — excluded from the 2-manifold check (like degenerate edges;
  no-op for correct solids, so base/matrix/edge/cone×cyl unaffected). This removed a false-negative that
  had z90 reading solid=0 despite naked=0.

**Result (HONEST — the fuzzy-forced `is_solid=1` closures are mostly FALSE POSITIVES):** with
`SESSION_SDWELD SESSION_BRIDGE SESSION_FUZZLADDER`, naked drops far (z90→0, y30→0 at cap 32) and
`is_solid` returns 1 — but `is_solid` only checks 2-trim edges, NOT connectivity, so a fuzzy over-merge
that self-closes a fragment passes falsely: **y30 reads solid 1 / naked 0 but vol 20.05 vs OCCT 46.96
(57% off) with shells=3 (a disconnected 10-face fragment)** — NOT a valid solid. **z90 reads solid 1,
naked 0, vol 62.04 vs 66.99 (7% off), shells=2** — a connected main shell plus one spurious empty face,
borderline. So only **z30x20 genuinely closes** (validated correct volume earlier). The OCCT-informed
mating fixes are real (naked reduced, cone×cyl recovered), but VALID closure of the rotated configs is
NOT achieved: the fuzzy naked=0 gives disconnected / wrong-volume results. The remaining configs sit at
**1–4 naked** and do NOT close even at cap 48 — the last edges are **genuine gaps** (missing section /
one-sided imprint with no mate), needing the edge-face (EF) section geometry the target does not compute
(OCCT's `PerformEF`). LESSON: `is_solid` must also require single-shell connectivity + a volume check to
avoid crediting fuzzy over-merge as closure.

## 5. Remaining work (ranked)
1. **Classification (KB law 7)** — angle-primary at section edges + one-representative-per-block +
   emplace face-pick. THE blocker for correct (not just closed) rotated solids. Freeform-only, so
   base/matrix/edge are protected. High value, medium risk.
2. **Single shared section edge (one edge, two pcurves)** — the load-bearing architectural cure for
   mating; makes bridge+fuzzy a construction guarantee. Large.
3. **Disconnected-orphan-shell removal** — drop spurious tiny closed sub-shells (cleans z90).
4. common/fuse ops for rotated configs (measured cut here; same machinery).

## 6. KB (memory/reference_brep_boolean_kb.md)
Laws 10 (topology-before-tracing: a march is handed its endpoints) and 11 (tangency is not a
transversal section → tolerant/fat edge; closed-form tangential tolerance
`aDt = TolF1·tan(π/2−angle) + TolF2/sin(angle)`). Single-shared-edge invariant is load-bearing;
fuzzy is the secondary tolerance layer.

## 7. Session 5 (2026-07-21): METRIC-FIRST foundation ("fix the validity metric BEFORE geometry")
Executed the user's step-1 directive in full. The campaign had been optimizing `is_solid` (a pure
per-edge 2-trim check) as if it were a *closure* signal — a necessary condition used as sufficient —
so fuzzy over-merges that self-closed disconnected/wrong-volume shells scored solid=1/naked=0 (false
GREEN on y30 vol 20 vs 47, z90 2-shell). Fix: gate every experiment on the OCCT oracle, never `is_solid`.

**7a. Checked-in OCCT ground truth** — `serialization/boolean_steps/chairs/OCCT_TRUTH.md` (from
`step_probe --cut/--common/--fuse`). It corrected two load-bearing wrong assumptions:
- **CUT is 1 OR 2 solids** — z15/z30/z45/z37/z63 cut legitimately DISCONNECT the chair into 2 solids;
  gating `solids==1` falsely fails half the battery. Gate `solids == OP_SOLIDS`.
- **x20 CUT is VALID** (single 80.30 solid), not unsound. The prior hardcoded `OCCT_UNSOUND=x20 z15`
  was wrong; only z15-common (empty) and x20-common (grazing 5e-4) are genuinely degenerate.
- **FUSE is uniformly 1 valid solid** across all 10 configs — the most tractable op; validate first.

**7b. Oracle gate** — `validate_oracle.sh <exe> "<flags>" "<cfgs>" <op>`: runs our boolean
(SESSION_FAST+ROT_STEP → exports result STEP, skips internal volume), BRepChecks OUR result via OCCT
(a disconnected result reads SOLIDS≥2, catching is_solid's false positives), compares to OCCT's own
boolean. PASS = OCCT-VALID ∧ solids==OP_SOLIDS ∧ |vol−OCCT|/OCCT<1%. Degeneracy DETECTED from OCCT's
OP_VALID/OP_VOLUME, never named.

**7c. Architecture audit** — the two fixes an independent 5-lens workshop re-proposed as "missing"
ALREADY EXIST: symmetric split = `refine_scaffold_at_breaks` (measured no-op: 0 new segments on
base/y30/z30x20 → operands don't node section chains at un-paved interior points, REFUTING the
pave-divergence hypothesis); interior pierce-point weld = `JWELD` (nurbssurface_trimmed.cpp:776);
`SectionScaffold` already keys section edges by seg_id from ONE shared chain (~70% BOPDS analog). True
residual: segment-END staggering (~1% chord, sub-refine-resolution), JWELD gap tuning, and genuinely
missing EF/VF for grazing (x20-/x13y29-common). Genuinely-new levers to try (oracle-gated): (1) L1/L2
decompose — prove section network closed (`scaf.n_bridge_residual==0`) before tuning classification;
(2) mate-of-scoped LOCAL weld (weld only DISTINCT-operand same-seg_id endpoints, never same-chain —
the exact y30 self-closure mechanism); (3) TangentFaces same-domain edge for grazing.

## 8. Session 6 (2026-07-22): z90 cut CLOSES to 1 solid + 7 gated mechanisms
BREAKTHROUGH: z90 cut goes from OPEN shell (solids=0) to **1 closed solid** (SHELLS 1, naked 0, ORIGIN
OUT correct) — first rotated config to close. Residual to OCCT-valid: 2 faces BadOrientationOfSubshape
(WIRE winding, not outward sign) + vol 66.26 vs 66.99 (1.1%), both caused by the fuzzy over-merge (ftol
0.142) reversing/merging trims. z30x20 stays PASS throughout.

Mechanisms added this session (ALL gated default-off; base cut stays 35/1/0):
- **SESSION_SHAREDPAVE** (normalize_section_blocks): both operands pave at one shared A∪B pave set →
  block boundaries align; z90 combine merges 40→48 section edges 2-trim by construction.
- **SESSION_SYMEMIT** (brep.cpp ~7380): curve-level extra_cuts for asymmetric emission — feed the operand
  that never LIFTS a section seg its uvA/uvB pcurve via a new onCutsB channel + second run_splits.
- **SESSION_CUTCAP=N / SESSION_CUTDEDUP** (nurbssurface_trimmed.cpp ~946): per-pair intersection
  cap / Hausdorff coincidence detector — the ROOT fix for the SYMEMIT explosion (a re-imprint coincident
  with the existing run floods `splits` → multi-GB). CUTCAP=24 also dedupes spurious cuts (z90 8→4).
  CUTDEDUP fixes z45/z63 fragmentation but hurts z90 → use CUTCAP for z90 (config-specific).
- **SESSION_CAPFILL** (brep.cpp ~8640): synthesize an ABSENT mating cap — chain naked edges into a closed
  loop (after a mate-of junction weld cap_diag*3e-3), project onto the other operand's surface, build a
  closed cap face. Works for FULLY-absent caps; z90's residual is a PARTIAL gap (open chain) so capfill
  emits 0 there and the fuzzladder sews it instead.
- **SESSION_SECPROTECT** (nurbssurface_trimmed.cpp:1057): protect section cuts from the dangling prune —
  no-op for z90 (A never lifts the missing segs, so nothing to protect).
- **SESSION_ORIENT_REPAIR** (face_outward_signs ~1665): relax relsgn to neighbour-majority — no-op for
  z90 (its errors are wire winding, not face sign).

KEY FINDINGS (evidence-backed):
- z90 naked loop = ABSENT B-cap fragment (SSI incompleteness), workflow-confirmed via an nt=0 orphaned
  edge (neither operand contributed a face). Not a classifier drop.
- No UNIVERSAL closing stack: the aggressive CUTCAP+CAPFILL+FUZZLADDER stack closes z90 but REGRESSES
  z30(3 shells)/z63(16 shells)/y30(3 shells) and EXPLODES on grazing z45/x20/z15/x13y29. Config-specific.
- Clean z90 closure needs proper cap-EXTENSION (attach the gap's naked chain to the existing adjacent cap
  face) to avoid the fuzzy over-merge that corrupts wire winding. That is the next concrete step.

## 9. Remaining work (ranked, concrete)
1. **Cap-extend** (clean z90): attach an open naked chain to the ADJACENT existing cap face's loop (extend
   it), instead of building a new closed face (capfill) or fuzzy-sewing (over-merge). Removes the wire
   corruption → z90 valid + vol exact.
2. **Wire-orientation repair**: after closure, fix per-face loop winding (material-left) for faces OCCT
   flags BadOrientationOfSubshape — helps z30/z37 too.
3. **Disconnection join** (y30/z63): mate-of-scoped sew of the 2 valid shells (NOT global fuzzy).
4. **Tangential SSI / TangentFaces** (z45/x20/z15/x13y29): the grazing configs that explode; P5.
