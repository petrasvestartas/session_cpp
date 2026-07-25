# External Research Synthesis → Program Corrections

Synthesized 2026-07-24 from the 8 `kb/research_*.md` files (parasolid-architecture, acis-cgm-architecture, tolerance-doctrine-theory, academic-robust-booleans, ssi-literature, open-kernels-survey, validation-corpora, kernel-testing-methods). Mapped to master-plan phases P3 same-domain / P4 EE-EF / P5 tolerance model / P6 corpus.

---

## 1. What Parasolid/ACIS/CGM do differently from OCCT — tolerant entity model to adopt

OCCT copied Jackson's nesting hierarchy but stopped there. The three commercial kernels add seven load-bearing mechanisms OCCT lacks:

1. **Tolerances on topology only; geometry stays exact** (all three). Vertex = sphere, edge = tube, monotone hierarchy tol(V) ≥ tol(E) ≥ tol(F), auto-raised on construction. OCCT has this; what it lacks is the rest.
2. **Legality contract separate from operation budgets** (Parasolid). Model validity requires: (a) *transitive* precision clusters — if d(1,2)<tol, d(2,3)<tol but d(1,3)>tol the BODY IS INVALID (refuse the contradiction cycle, don't compute through it); (b) *contractibility* — every connected component of two tolerance-zone intersections must retract onto shared topology (the crisp self-intersection criterion under tolerance); (c) normative 100× minimum-feature-separation guidance. Per-op knobs are separate: `default_tol` (coincidence-assumption budget, ~1e-5, 3 orders looser than session precision) + `max_tol` (escalation cap). We currently mix legality and budgets in per-op epsilons — split them.
3. **One coincidence algebra everywhere**. Parasolid/OCCT/Segal: SUM of the two tolerances; ACIS: MAX. Either works; MIXING them manufactures contradiction cycles. Our diag-scaled per-call constants (alias 1e-2, sew diag*5e-3, on_eps asymmetry) are F9 epsilon-tweaking — collapse into one predicate layer consuming per-entity tolerances.
4. **Escalate-then-compress, never fail-and-leave-dirty** (Jackson join phase). Matched entities with different tolerances merge taking the LARGER; topology compressed to 1-1 correspondence; a vertex may legally correspond to an edge/face/cell. Failure recovery = partition rollback (Parasolid pmarks) — per-op arena checkpoint + roll-back-on-failed-gate is our missing atomicity layer.
5. **Edge representation under disagreement — pcurves become primary**. Parasolid tolerant edge: 3D curve DELETED, one SP-curve per fin, tube radius around one designated curve. ACIS TEDGE: pcurves primary, 3D curve lazy. CGM goes furthest and is the target: **`CATEdgeCurve` aggregate = ALL representations of the edge (pcurve_A, pcurve_B, 3D) + explicit parametric mapping + checked gap < resolution** — faces can never drift apart because the edge is never re-derived. Adopting this for boolean-born edges kills our trim_dir/alias/mate rotation-bug class by construction.
6. **Dimension-ascending imprint with re-compare fixpoint + FF obligation subtraction** (Jackson, = OCCT's PaveFiller order but with two extras): (a) after ANY edge split, re-compare pieces for coincidence (smaller pieces may be coincident when the whole was not — our SEGLOST class is this fixpoint missing); (b) the FF intersector must NOT produce curves for contacts already known from EE/EF — "near-tangent intersection curves are very unstable" (1995 statement of our shared-SSI/scaffold doctrine).
7. **Matched regions / glue booleans / declared face mapping** (P3 in all three vendors): declared or auto-detected coincident-pair structure (FF/FE/EE/VV + relation type exact/contains/overlap + per-pair tol) consulted BEFORE SSI; matched pairs never enter the SSI walker. ACIS glue_options taxonomy: identical / A-covers-B / B-covers-A / partial-overlap — only the last needs 2D UV boolean work. CGM priority order: pointer-shared surface > declared mapping > detection; ACIS "prepare" promotes near-coincident to exactly-coincident then retries. Plus ACIS complexity-report retry loop (named failure classes → local prepare fix → retry, stop at fixed-point; 70% fixed in one iteration) — the architectural form our SESSION_* gates should take.

Also: scoped post-op checking (`check_fa` = only faces adjacent to imprinted edges, default ON; full face-face check reserved), graded fault taxonomy with per-fault recovery hints, sequenced check groups with early-out, shuffle-debug entity-order fuzzing, imprint-only diagnostic mode, `keep_target_edges` deterministic survivor tie-break, oversize-tool best practice, version-pinned behavior (`update` / CATSoftwareConfiguration).

---

## 2. Academic mechanisms worth porting (name + 1-line design)

- **Segal `near`-value ledger (M3, P5 flagship)**: record every disjointness verdict with its measured margin; any tolerance growth past a stored margin re-opens exactly those verdicts (spatial index) — the re-verification OCCT and every shipping kernel lack.
- **Feature-size-linked growth caps (Sakkalis reach theorem + ACIS 10% envelope)**: budget β = κ·(local min feature separation); any tolerance-growth event exceeding β is an ERROR at the moment creep starts, not at collapse.
- **ε-stability contract (Fortune/Guibas–Salesin–Stolfi)**: kernel promises "result = exact boolean of inputs perturbed ≤ ε_out, ε_out reported" — makes creep observable at the API.
- **Indirect predicates / provenance keys (Attene)**: intersection points stored as defining entities (face-pair id + curve id + branch), never rounded coords; never round-then-reason (the fb=47.99997 alias-key drop is the anti-pattern).
- **Winding-vector cell classification (Zhou 2016 mesh arrangements)**: classify volumetric cells once with (w_A,w_B); cut/common/fuse/xor/split = selections over one imprinted arrangement — matches ACIS ct-graph/SBOOL; eliminates per-op classification divergence.
- **Radial face sort around section edges (Lévy exact Weiler model / Nef sphere map)**: interleave A/B faces by dihedral angle around each section edge as a combinatorial invariant — replaces quorum flood heuristics; local-cone classification at interference vertices for P4.
- **Degenerate section edges (ACIS intersection graph)**: point/tangential contacts = zero-length edges with coedge multiplicity, inside the same section network — P4 unification.
- **Three-valued predicates + decision memo (Fang–Brüderlin)**: hot ON/incidence tests return TRUE/FALSE/UNCERTAIN with margins; UNCERTAIN routes to exact fallback or recorded tie-break, never a silent coin-flip.
- **Dimension-ladder invariants with lowest-level repair (Mandoline)**: assert closure per level (paves→chains→loops→faces→shells); repair at the level that broke, never above (the closure-weld cap bug was an above-level repair).
- **Canonical deterministic arrangements (Lévy)**: UV splits a function of the input set only — lexicographic/quantized-key ordering, no iteration-order dependence.
- **Sliver/short-edge dimensional reduction (ACIS criteria)**: sliver face (≥1 short edge, ≤3 long, long-edge separation < tol) → tolerant edge; short edge → tolerant vertex.
- **4-state coincidence classes (SolveSpace + BRL-CAD NMG, independently converged)**: IN/OUT/COINC_SAME/COINC_OPP with per-op per-operand keep tables — the compact provably-single-copy same-domain answer; plus NMG fuse-first (weld coincidence into shared topology BEFORE SSI, classify by pointer identity) and forced A/B classification symmetry by copying shared verdicts.
- **Export watertightization (Urick/Marussig)**: post-boolean refit of patch boundaries so the section curve lies exactly on both surfaces — STEP/Rhino acceptance tier.
- **Mesh-proxy B-Rep booleans (Yang et al. TOG 2025)**: conservative mesh finds ALL intersection topology, bijective map back to B-Rep for true curves — the principled version of our scaffold; read in full.

---

## 3. SSI guaranteed branch capture — the literature answer to our missed mini-branch class

Consensus recipe (MIT school + Boeing + 2023–26 TOG papers), concretely:

1. **Boundary seeds**: all restriction-arc × other-surface roots via certified 1D interval rooting (OCCT IntStart model) — every open branch touches a boundary, so boundary seeds capture all non-loop components. Land march endpoints on these exact border roots (retires tol3 endpoint bridging).
2. **Collinear-normal solve (Sederberg–Christiansen–Katz 1989)**: any closed loop implies a line perpendicular to both patches; solve the 4-eq/4-unknown collinear-normal system globally (IPP/Bernstein all-roots solver), **subdivide both patches at the roots** — every loop becomes boundary-touching open arcs of sub-patches; boundary seeding is then complete. This is the classical guaranteed-capture recipe, and its roots double as tangency candidates (W3).
3. **Per-box no-loop certificate** where ambiguity remains — Bartoň–Elber–Hanniel NLT+SCT: build complementary tangent hypercones from normal-cone bounds in the 4D domain; if a hyperplane misses their intersection on S³, the curve is monotone in a direction ⇒ no loop in the box; then solve the 8 boundary systems — 0 hits ⇒ box PROVEN empty, 2 hits ⇒ exactly one monotone segment (trace it, cannot stray or close), ≥4 ⇒ subdivide. Gives a topology certificate: traced network must match boundary-hit counts; mismatch ⇒ subdivide and re-march instead of letting flood quorums absorb the error.
4. **Gold plating (TOG 2026 winding-number method)**: winding integral of a distance-gradient field on cell boundaries counts field zeros exactly — a loop cannot be missed no matter how small; gradient-variation subdivision localizes parallel-normal closest-point pairs, giving certified seeds ON tangential branches (solves W1 + tangential W3 in one mechanism).
5. **Tangency classification (Ye–Maekawa)**: at rank-drop, the II₁=II₂ quadratic's discriminant classifies contact — Δ>0 branch point (spawn 4 departures), Δ=0 tangential curve (march double-root direction with bordered/augmented Newton), Δ<0 isolated point (emit vertex), Δ≡0 same-domain candidate. Replaces minimization-completion of stalled marches.

Concrete kernel change: new `ssi_seed_certify(faceA, faceB)` — coarse UV×UV boxes; cheap cone-overlap no-loop test per box; failing boxes get the collinear-normal IPP solve; roots → extra seeds for the existing marcher; NLT-passing boxes with 0 boundary hits are proven empty. Kills the mesh-resolution dependency of scaffold seeding and the missed-tangent-circle class. Seeds computed on the pair (4D boxes) → one canonical seed list → drop the swapped-operand retry.

---

## 4. The 10k-case corpus plan

**Composition (10,000 nightly cases)**
| Tier | n | Source | Stresses |
|---|---|---|---|
| T0 | 500 | in-house primitives matrix + chairs rotation frontier + analytic pairs | exactness, recognizers, known frontier |
| T1 | 1,500 | OCCT tests/boolean (39 grids) + bugs/modalg harvest with embedded `checkprops`/`checknbshapes` oracles | pre-triaged adversarial, each a former bug (highest ROI/case) |
| T2 | 3,000 | 600 deduped valid ABC solids × 5 poses (aligned, z15, z30x20, tangent offset 0, ±10·tol) | freeform SSI, rotation, near-tangency |
| T3 | 2,000 | AutoMate mated pairs in mating pose (CC0; 1.29M mates) [+ Fusion 360 joints internal-only NC] | real coincident/coaxial contact → P3/P4 |
| T4 | 2,000 | DeepCAD replay: per-extrude (accumulated body, new extrusion, designer op) | sketch-on-face coplanarity at volume (P3) |
| T5 | 1,000 | synthetic generator: near-tangent quadric lattices, seam-crossing rotations, scale sweeps 1e-3..1e3, ±k·tol jitter ladders | P5 tolerance model, P4 events |

**Harvest methods**: ABC = mirror STEP chunks (start 5 chunks/50k), OCCT import, require single closed valid solid, unit-bbox normalize, B-Rep fingerprint dedup (topo counts + inertia tensor — AutoMate recipe); pairs guaranteed interfering via bbox + stl2 mesh-boolean prefilter; self-boolean `A op R(A)` generalizes the chair protocol. OCCT = tcl scraper restoring operand .breps → STEP + expected props. FreeCAD tracker = unzip .FCStd attachments → raw .brp operand pairs with issue-URL provenance. DeepCAD = pythonOCC replay dumping (A.step, B.step, op, props) per step. License buckets in directory layout (ours/lgpl/cc0/onshape-tou/nc-internal); manifest parquet with sha256 + provenance so a public cut is a filter, not a re-audit.

**Oracle (escalating cost ladder)**: (1) self-validity — closed shell, naked=0, BRepCheck-equivalent, orientation, tolerance bounds (invalid inputs → separate garbage-in track); (2) mass properties vs OCCT GProp (rel 1e-6 exact tier / 1e-4 freeform); (3) topology census vs `checknbshapes` with mapped-equivalence band; (4) point-membership differ ~1k stratified samples near section curves (catches flips volume misses); (5) mesh Hausdorff, triage-only. **N-version voting**: OCCT is fallible — disagreement goes to a mesh-boolean second oracle (Manifold / mesh-arrangements / our main_8 SDF path); 2-of-3 against us = confirmed fail; 3-way split = research specimen for the KB, often an OCCT bug. **Metamorphic layer on 100% of cases (no oracle needed)**: rigid-motion equivariance MR-T1 (the rotated-chair class), scale invariance, inclusion-exclusion V(A∪B)+V(A∩B)=V(A)+V(B), per-operand partition V(A\B)+V(A∩B)=V(A) (detects SEGLOST), idempotence A∪A≡A (sharpest P3 smoke test), disjoint-op exactness, flux-vs-identity volume (tier-3 classifier promoted to gate), jitter stability δ=1e-12..1e-9·diag, gap ladder g=k·tol single-monotone-switch, fuzzy sweep.

**Nightly runner**: per-case subprocess, 30 s timeout + RSS cap; verdicts PASS / FAIL(diff) / INVALID / CRASH / HANG / OCCT_FAIL; adopt OCCT's five-state ledger (OK/FAILED/BAD-known/IMPROVEMENT/SKIPPED) so CI stays green while the frontier is tracked; report = delta vs last green baseline; auto-minimization of new failures (drop sub-shapes, bisect pose) → named permanent regression in T0 (Parasolid zero-regression rule); parallel ~16 workers, <2 h wall; weekly extended ABC sweep (~100k pairs); perf+memory testdiff per battery. Build order: L2 invariant engine (fast profile: A1,A5,M1,M2,T2,S-gates) in the chairs loop first; MR-M2/M4 as gates; reader libFuzzer+ASan (CGAL CVE precedent); third-judge voting; rotation-grid pass-rate scalar; ABC-1k pilot.
