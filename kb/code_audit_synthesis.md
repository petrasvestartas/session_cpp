# Code audit synthesis (2026-07-24 @ HEAD 4544770f)

Synthesis of `audit_brep_cpp.md` (brep.cpp 11,285 ln), `audit_brep_section.md`
(brep_section.cpp 2,205 ln, mismatches M1–M15), `audit_test_infra.md` (batteries + gaps 1–12),
the 12 `occt2_*` research summaries, and the three naked-edge censuses
(x20: 9, y30: 13, z63: 13 — classes A SEGLOST=15, B divergent-bridge=12, C junction-undershoot=5,
D micro=1, F unmerged-mates=2, total 35).

"P1 relevance" below = relevance to the priority-1 frontier: closing the rotated-chairs naked-edge
classes by construction (scaffold valence closure + BOP2 identity), per the census mechanism table.

---

## 1. P7 DELETION PLAN — ordered by safety (line ranges vs brep.cpp @ 4544770f)

Doctrine anchor (occt2_topopebrep): the `CompleteDS` repair-cascade is the canonical anti-pattern;
every deletion below removes a distance-based or superseded repair whose invariant must instead
hold at creation. Never re-add a post-combine repair pass.

### Tier 0 — zero-risk hygiene (no behavior change possible)
| # | Target | Lines | Evidence |
|---|---|---|---|
| 0.1 | stale `SESSION_NO_FUZZLADDER` comment (gate is positive `SESSION_FUZZLADDER`) | 10160 | never read; comment-only |
| 0.2 | dead battery cell `ecylO` (defined, never run) | main_7.cpp edge_placements() | test-infra gap 7 — WIRE INTO edge_pairs() rather than delete |

### Tier 1 — no-risk deletions (opt-in, superseded, or measured-harmful; independent of BOP2)
| # | Target | Lines | Evidence |
|---|---|---|---|
| 1.1 | SESSION_CLS_FIX2 | 9344–9390 | "MEASURED HARMFUL … z30 23→35 naked"; finding recorded in KB |
| 1.2 | SYMEMIT block + p1c_onA/B channels | 8254–8333, 8126–8142, 8236–8241 | comment 8334: SYMLIFT is "the CORRECT replacement"; corrector "slid to the WRONG BRANCH" |
| 1.3 | SESSION_EF_PAVES | 3376–3426 | regressed cone×cyl 2→8 naked; superseded by scaffold paves |
| 1.4 | SESSION_RECOVER + recover_section_spans | 6105–6180, 10062–10064 | "residual lacks mateable copies" — measured ineffective |
| 1.5 | SESSION_SE_OTHER (PutSEInOtherFaces) | 4264–4332 | one-sided, y30 13→31 naked; symmetric successor = SYMLIFT/P4 |
| 1.6 | SESSION_ORIENT_REPAIR | 1677–1700 | weaker duplicate of SHELL_ORIENT (keep SHELL_ORIENT while rot frontier open) |
| 1.7 | SESSION_NO_ON gate (keep the ON handling itself) | 8814 | experiment concluded; ON handling is load-bearing |
| 1.8 | gated SESSION_BOOL_ELLIPSE block in co_refine | 5354–5409 | FINAL PASS 2 conic registry derives Steinmetz UNGATED (5932–5968) |
| 1.9 | SESSION_TRIM_SNAP/_OV bridges (legacy-only, `scaf ? nullptr`) | 4500–4557 | scaffold overshoot stubs 4055–4263 internalized the pattern |
| 1.10 | SESSION_STUB_REACH (after rotation campaign settles) | 4209–4241 | "mixed on rotated cfgs" |

### Tier 2 — debug consolidation (biggest single win, mechanical)
26 debug/print gates (audit_brep_cpp §2a) behind one `SESSION_BREP_DBG=<topics>` dispatcher.
~600–900 lines; debug printing ≈15% of boolean().

### Tier 3 — legacy-freeform lane retirement
PRECONDITION: scaffold gains seam handling for unrecognized analytic pairs (cone×cone currently
takes the S1 pre_cuts lane) — or intana port (§2 #10 runner-up) makes cone×cone exact.
- pre_cuts pair-SSI 8047–8076; cutsA/cutsB + sec_c3ds plumbing; snap_section_edges 6794–6871 +
  call 10078–10084 + NO_SECSNAP; legacy snap_bnd=0.05 branch 4571–4589 vicinity;
  `imported_freeform && !use_scaffold` conditionals throughout.

### Tier 4 — distance-based mating retirement (the construction-identity payoff)
PRECONDITION: BOP2 (pool referencing + P4 split + index alias) default-on AND `SESSION_NO_SEW`
runs show naked=0 across base/matrix/edge/rot batteries. Gate each deletion on re-running the
full battery set under NO_SEW (see §3 — thin-wall gap 3 must be filled FIRST: it is the only
class that would detect over-merge introduced by removing the sew ladder).
| # | Target | Lines |
|---|---|---|
| 4.1 | sew_coincident_edges P2 (2.5× relax) + P3 (mutual-best 8×) — keep P1 exact-Hausdorff + micro/orphan collapse | 7017–7049, 7050–7095 |
| 4.2 | FUZZY + FUZZLADDER + _CAP | 10116–10198 |
| 4.3 | make_shared_section_edges + BOOL_SHARED_EDGES branch (decouple s_trimcut at 2991 first) | 6531–6792, 10085–10093 |
| 4.4 | normalize_section_blocks A/B calls + SHAREDPAVE (keep R-pass initially, measure, then drop) | 6182–6529 callers, 8499–8547 |
| 4.5 | NK-RESCUE + span-alias (qspan) fallback at combine — BOP2 index alias supersedes | 9842–9911, 9596–9626 |
| 4.6 | CAPFILL, WIRE_REPAIR — delete or promote to unconditional ShapeFix-style validators (§2 #10) | 10199–10324, 10325–10380 |
| 4.7 | SYMLIFT — re-evaluate: unreachable once P4 emits every segment both sides | 8334–8441 |

### Must NOT delete (proven load-bearing)
co_refine circle/arc exact rebuild + conic registry (quadric-matrix volume exactness); sew P1 until
BOP2 covers primitives; winding contains_point; contains_point_exact; angle method; symmetric
connexity flood; parity; radial certification; island repair; micro-fragment collapse; xweld;
imprint_edges; face_outward_signs tiers 1–3.

Estimated full-P7 reduction: ~2,500–3,000 lines.

---

## 2. TOP 10 SPEC MISMATCHES — ranked by P1 relevance

| # | Mismatch (ours vs OCCT spec) | Spec source | Census evidence | Port action |
|---|---|---|---|---|
| 1 | **Invalid intervals DROPPED, endpoints never welded** (verdict L1200–1210, micro L1259–1285; M3). OCCT's four-layer micro defense NEVER silently discards — a swallowed range welds its two end vertices (pavefiller inv 5; FindValidRange). | pavefiller-core, breplib-sameparameter §FindValidRange, shapehealing FixGapsByRanges | Direct cause of class A+C = 25/35 naked edges; z63 SCAF-DROP ALL-OUT corridors starve segs 25/34/35/36; y30 "cuts=2 parts=1" SEGLOST | mandatory drop→weld: FindValidRange sphere-march+bisection (step Res(tolV)*1.01, eps max(Res(tolE)*0.1, PConfusion)) at stage-3; FixGapsByRanges no-worse acceptance before any whole-seg key drop |
| 2 | **Single-seed march misses connected components** — no exhaustive contact enumeration; branch budget unknown. OCCT: tri-tri SAT contact soup → chained components, EVERY component a section line; multi-seed schedule until walk matches branch extent; 4-shift ±1.5·defl pass for tangency. | intpolyh-seeding | y30 B14 lobe lost (segs 15,41 never marched); the (18,5) mini-branch class; z63 sA=16 corridor loss | NEW-BUILD polyhedral pre-pass (BVH+SAT couples→components) as mandatory branch budget for build_section_scaffold; multi-seed per branch clamped to branch UV bbox |
| 3 | **Interior stalls dangle; boundary completion MARCHES instead of constrained minimization** (Stage 1b, M4); no hairpin cleanup, no IsParallel guard, absolute epsilons. | ssi-walking (PutToBoundary band 1e-3·min(1,ranges), SeekPointOnBoundary → SqDist<1e-14) | Class C direct: y30 v7/v8 gap 0.4656 + v23/v27 gap 0.2512 → 10/13 naked; x20 e106 gap 0.1968 | REPLACE stage-1b stall path with gradient/extrema minimization; hairpin delete; make EXT_TRIM-equivalent default via minimization not march |
| 4 | **Bridges invent geometry** — CASE B raw midpoint + projection gate only; valence-1 bridge unconditional (M14). OCCT: corrector-converged midpoint + THREE pairwise π/6 angle gates + period rejection (ExtendTwoWLines); FixLacking bridges only 3d-merged-but-2d-open, forbids >0.9π back-going; ChoixUV angle-minimal continuation at pivots. | ssi-walking, shapehealing FixLacking, brepcheck ChoixUV | Class B = 12/35: x20 e86 diverges 0.41 from true 2-seg chain; z63 e103+e107 diverge 1.42; y30 e13 vs 4-edge true path 0.63 | REPLACE bridge/NK-RESCUE partner choice with ChoixUV + FixLacking decision tree; corrector-converge every junction point |
| 5 | **No SD map; greedy non-transitive weld; single global tol3=diag·2e-3; no tolerance honesty/growth feedback** (M5). OCCT: MakeSDVertices transitive components, re-keyed after EVERY stage; per-entity tol ≥ accepted distance; myIncreasedSS → re-intersection. | pavefiller-core inv 2/7, tolerance-model, breplib InternalUpdateTolerances | y30 JOINFAIL gapmax 6.194e-3 > tol 4.419e-3 (one hand-tuned cap) → pair B1 naked lens; z63 F-class e123/e131 d=0.0003 unmerged (rescue budget spent) | per-segment/vertex measured tolerance (ComputeTol w/ out-of-chart penalty ×1.5, floor 1e-7); transitive SD weld (promote SDWELD, extend past valence-1); derive weld caps/alias radii from measurement |
| 6 | **No section-edge adoption of coincident operand edges; whole-seg alias keys with 1e-2 dead-band** (M12; refine eps_f coupling). OCCT: IsExistingPaveBlock ×2 (growth cap 0.001) BEFORE minting; edge identity = shared object, not tolerance key. | pavefiller-core, breplib #1 (shared EdgeGeom), brepalgoapi-cells #3 (ownership rank A-wins), shapehealing FixShifted | fb=47.99997 trim-snapped sliver dropped keys (commit 4544770f); x20 one-sided seg retention 5/9 | NEW-BUILD shared `EdgeGeom{chain3d, per-chart run, tol, same_param}` referenced by BOTH faces at combine; FixShifted-style AdjustByPeriod key canonicalization; deterministic A-wins geometry rank |
| 7 | **Unconverged/clamped chain samples emitted; fixed-count resampling, no deflection control** (M1+M2). OCCT: clamp INSIDE Newton + re-converge, failures abort the point; sag ≤ fleche + ≥40 pts. tol3 is inflated to absorb sag OCCT prevents at source — the root inflation feeding #5's tolerance lies. | ssi-walking inv 1/4 | y30 pair B1: seg-2 PARTIAL run on A (fa=1.5893, whole=0) right after the JOINFAIL — chord sag vs mate tolerance | enforce converged-or-abort at L449–464; TestDeflection-style sag bound + SeekAdditionalPoints densification; then SHRINK tol3 |
| 8 | **First-order-only Newton in closest.cpp:311** — missing D2 terms (|Su|²+PPs·Suu…). Tangential stalls feed every projection verdict, partial runs, cut-node snap patch. | extrema-internals (FuncPSNorm::Values) | tangential newton_cc stall class (patched around in UV-arrangement cut-node snap, 4544770f); [SCAF-RUN] whole=0 partials | REPLACE Jacobian with full D2; ADOPT knot-aware span seeding + degenerate-iso densify; NEW-BUILD surface_points_all (multi-foot, 1e-9 UV dedup) for radial/NK/tube |
| 9 | **Verdict band UV-parametric (min-span·1e-3), not metric/resolution-scaled** (M9); graze intervals need ON_QUORUM gate. OCCT classifies middle+bound at 3D tolerance, all tests resolution-relative (inv 10). | ssi-walking inv 10, pavefiller IsValidBlockForFaces | z63 SCAF-DROP corridors correlate with ALL class-A losses; x20-common 5e-4 graze class | metric verdict at tol3-scaled 3D distance; keep-ON routing per GTopo ON-row (topopebrep) instead of drop |
| 10 | **Validity measured, never enforced; no hard in-pipeline checker** — stage-5 valence audit prints violations; even-valence, opposite-use orientation, 23-pt same-parameter all absent as gates (topopebrep Checker = NYI stub is the cautionary tale). | brepcheck-validity (Closed parity, opposite-use, ValidateEdge 22/23-pt), topopebrep lesson 3 | all 35 naked edges pass silently until step_probe; all-faces-flip class caught late by tier-3 flux | NEW-BUILD hard DS validation: even-valence + opposite-use gate post-combine; 23-pt same-parameter self-audit writing MEASURED deviation into edge tol (feeds #5) |

Runner-ups: whole-arc coincidence / tangent-zone routing (M7, Arcsol→RLine — SEGWHOLE done too
late); IntAna quad_quad_geo 15-pair case table + AxeOperator canonicalization (cone×cone, rotation
loss — unblocks Tier 3); one-scaffold-many-ops keep-mask lift (cells-builder #1 — replaces 3-run
boolean_split, fixes xor instability); simultaneous (C3d,pcA,pcB) shared-knot fit (approx-geomint
#1 — same-parameter by construction, Rhino trim rejection); pole decomposition in stage 1a2 (M15).

---

## 3. TEST-COVERAGE GAPS mapped to program phases

Phases: P1 = scaffold valence closure + rotation frontier (mismatches #1–#4); P2 = tolerance
honesty + identity (#5–#7); P4/BOP2 = construction identity default-on; P5 = battery expansion
(occt-test-mining reserves); P6 = external acceptance (Rhino/OCCT oracle); P7 = deletion (§1).

| Gap (audit_test_infra §6) | Phase it blocks/validates | Action |
|---|---|---|
| 2. no near-tangency epsilon sweep (1e-2..1e-9 tolerance cliff) | **P1** — the z45/x13y29 marcher failure class has no primitive repro; validates mismatch #2's 4-shift pass and #3 | build sweep family battery (offset-walk a tangency); gate on naked=0 + oracle vol |
| 1. freeform tangency/coincidence/same-domain/containment all quadric-only | **P1/P2** — scaffold coincidence handler (M7) and same-domain extra_cuts have zero battery drive | add freeform A op A, freeform flush-face, freeform containment cells (perturbed-sphere infra exists) |
| 4. pole-crossing / cone-apex never isolated | **P1** — stage-1a2 has no pole decomposition (M15) | cell forcing section through sphere/torus pole + apex-through-face |
| 8. common op second-class in validate_oracle/rot harnesses | **P1** — OCCT_TRUTH marks common hardest (multi-solid, degenerate) | default validate_* to all 3 ops |
| 9. main_9 no oracle hookup (gates on internal is_solid only) | **P1/P6** — exact false-positive class validate_oracle.sh was built to kill | add --probe mode wrapping step_probe |
| 12. SESSION_NO_ROT drops oriented cells from STEP-regen gate | **P1** hygiene | fix the hang, or gate rot cells separately |
| 3. thin-wall absent (result wall < sew/weld tol) | **P7 HARD PRECONDITION** — only class that detects over-merge when sew P2/P3 + FUZZY are deleted (§1 Tier 4) | box cut box offset 1e-3 family, gated |
| 5. self-intersection never asserted (only post-hoc step_probe -c) | **P2/P7** — validates mismatch #10's hard checker; BRepCheck never 3D-intersects faces, so a dedicated gate is needed beyond OCCT-VALID | assert no SelfIntersectingWire on every battery result |
| 6. xor/split gated only on 15 base pairs | **P4** — cells-builder keep-mask lift makes xor/split selections over ONE split; EDGE/OCCT2 xor is its acceptance test | extend SESSION_XOR_CHECK to EDGE + OCCT2 + chairs |
| 7. dead cell ecylO (parallel-axis proper-overlap cyls) | P0 hygiene (§1 Tier 0) | wire into edge_pairs() |
| 11. untranslatable OCCT-suite regions: truncated cone (214 files), prism/revol, angle-limited, multi-operand bbop | **P5** — blockers named in occt-test-mining: create_cone(r1,r2,h) ×3 langs + oracle cone3; arbitrary-axis xf_of via existing Xform::rotation; oracle AREA output | unlock reserves ZF6/ZM5/ZC5/gdmlB1 + SESSION_OCCT3 grid |
| 10. Rhino acceptance manual, never a gate | **P6** — known step_probe-VALID-but-Rhino-open class | wrap rhino_headless_probe.py as opt-in CI gate on chairs STEP exports |

Free invariant gates already available (adopt everywhere, zero oracle cost): S(fuse)+S(common)=
S(A)+S(B); tuc(A,B)==cut(B,A); ZD4/ZD5 seam-twin equality (seam-aliasing detector); empty-expected
cells must yield ZERO faces (sliver regression harness).
