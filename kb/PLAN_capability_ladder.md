# Development plan: the capability ladder to industry-grade booleans

2026-07-25. Written after a day of measurement that reorganised the problem. **Provisional
on five investigations still running** (oriented-primitive root cause, freeform ladder,
interop, determinism, EF/VF gap) — the gates below are written so those results slot in
rather than invalidate.

---

## 1. The reframing: the axis is SURFACE TYPE x POSE, not chair config

The campaign has been organised around 10 rotated chair configs. The measurements do not
cluster that way. They cluster by what the surfaces ARE and how they are posed:

| geometry | pose | measured result |
|---|---|---|
| planar x planar | axis-aligned | exact |
| **planar x planar** | **rotated 30 deg** | **EXACT, rel 7.5e-16 / 4.0e-15 / 9.8e-16** (box x boxR) |
| analytic curved | axis-aligned | exact, rel <= 1e-14 (cone x cone), 45/45 cells |
| **analytic curved** | **rotated** | **FAIL, rel ~9.4e-05** (cyl x cylR, boxR x sph) |
| analytic curved (torus) | rotated | FAIL, rel 1.3e-02 .. 4.8e-02, face counts differ |
| freeform BSpline | any | **broken**: open shells, result exceeds operand bbox |
| trimmed multi-face (chairs) | rotated | 3 of 30 correct |
| coincident (A-op-A) | n/a | **FIXED today** (cut EMPTY, common = A, fuse = A) |

Two facts do the most work:

1. **Rotation alone does not break the kernel.** `box x boxR` is exact to 1e-16 with a
   30-degree rotation. So "rotation robustness" was never the real problem.
2. **Curvature + rotation breaks it, at a suspiciously uniform ~9.4e-05.** Four
   independent cells land at 9.39e-05, 9.18e-05, 9.45e-05, 9.69e-05. That is not
   accumulated round-off — round-off does not agree to two significant figures across
   different geometry. It is a **fixed sampling/deflection constant**, i.e. a numerical
   path being taken where an exact path should have been.

**The hypothesis this implies** (being tested right now by the oriented-primitive agent):
analytic intersection recognisers fire only in axis-aligned poses and silently fall back
to marching when rotated. OCCT does not have this problem because `IntAna` transforms
quadric pairs into a canonical frame and solves in closed form — the answer is exact in
ANY pose. If our recogniser *tests for* alignment instead of *transforming to* it, that
is a specific, bounded bug, and it plausibly explains the entire rotated-chair residue
(the chairs carry cylindrical faces).

The chairs are not a special problem. They are the first geometry in the corpus that is
simultaneously trimmed, multi-face, curved AND rotated — so every weakness lands at once.

---

## 2. The ladder

Each rung is a CAPABILITY with a falsifiable gate. Do not start a rung before the one
below it is green; do not claim a rung without the gate having been run.

### P0 — Measurement integrity (mostly landed today; finish it)
Nothing above this rung is meaningful without it.
- [done] non-manifold edges counted against a candidate in `metric()` (they scored ZERO
  before, so the AUTO ladder actively preferred non-manifold shells)
- [done] bbox containment invariant (cut within bbox(A); common within the intersection)
- [done] cross-op partition identity `vol(cut) + vol(common) = vol(A)` in the battery —
  the ONLY oracle-free check that catches y30's closed-but-wrong answer; every
  single-result invariant passes it
- [done] independent verification via an OCCT reader on the EXPORTED file, not on
  internal bookkeeping
- [todo] unmask the oriented battery permanently: coverage is **45/63**, not 45/45. A
  gate that silently skips its hardest cells reports green forever
- [todo] record the full `SESSION_*` flag set with every ledger row; silent flag drift is
  what hid the oriented battery
- **GATE**: every claim in a report traceable to a run whose flags and binary are
  recorded, and whose probe is verified present in that binary (a zero read from a binary
  lacking the print is not a measurement — this actually happened today)

### P1 — Analytic COVERAGE (revised 2026-07-25 after the hypothesis was refuted)

**CORRECTION — the original hypothesis was wrong and is retained here as a record.** I
wrote that "analytic recognisers fire only in axis-aligned poses and silently fall back
to marching when rotated". Session A refuted it with a rigid-motion equivariance sweep
(BOTH operands rotated by the same generic motion, 0-45 deg, compared to the angle-0
answer — no oracle needed):

    box x box2 (planar x planar)  worst rel 8.77e-15  EXACT
    cyl x cyl2 (curved x curved)  worst rel 8.30e-15  EXACT at 45 deg
    sph x cyl  (curved x curved)  worst rel 5.07e-14  EXACT
    box x cyl  (planar x curved)  1.89e-09, small break at 0.01 deg
    box x tor  (torus)            2.38e-02  BREAKS

Curvature + world rotation is FINE. The hypothesis was also structurally impossible:
under a SHARED rotation every RELATIVE angle is invariant, so a relative-angle predicate
cannot flip — and for box x tor the recogniser demonstrably never stops firing
(`[REC] A kind=1 B kind=4` identical at every angle including the broken ones).

**THE REAL AXIS IS RELATIVE POSE, AND THE DEFECT IS MISSING ANALYTIC BRANCHES.** The
failing oriented-battery cells (cyl x cylR 9.39e-05, boxR x sph 9.45e-05) differ in
RELATIVE configuration. Confirmed by reading our source: `ssi_cylinder_cylinder`
(`intersection.cpp:3952`) handles non-parallel axes ONLY when the radii are equal —
`|R1-R2|/Rmax > 1e-6` returns false and drops to the marcher. **Skew cylinders of
differing radii have no analytic branch at all.** The uniform ~9.4e-05 is the marcher's
sampling constant showing through the gap.

So P1 is: **complete the analytic case table**, not canonicalise poses.
- Build the coverage matrix — for all 15 quadric pair types x relative configurations
  (axes parallel/intersecting/skew, equal vs unequal radii, coaxial, tangent,
  concentric): does OUR code have a branch, does OCCT? Every cell where OCCT is analytic
  and we march is a gap to fill, ranked by real-world frequency (our chairs carry planar
  and CYLINDRICAL faces, so cyl-cyl and plane-cyl gaps rank highest)
- Implement the missing branches, closed form, with the audited constants
- Cover the quadric pairs: plane/cylinder/cone/sphere/torus. Torus is worst today
  (rel 4.8e-2) because it is quartic — treat it explicitly, per
  `kb/audit_occt_intana-analytic.md` (tangency is `IntSurf_Touch` + Situation; dead-bands
  are 0.0 / 1e-9 / 1e-7 / 1e-8; cyl-cyl coplanarity epsilon is 1e-7, NOT 1e-14)
- **GATE**: the 18 oriented-battery cells go from 15 FAIL / 3 OK to 18 OK at the same
  tolerance the axis-aligned matrix meets (1e-6 relative); rigid-motion equivariance holds
  to 1e-12 on a swept rotation angle, not just at one angle
- **WHY FIRST**: it is bounded, it is testable without the chairs, and if the hypothesis
  holds it removes the dominant error source from every curved-surface config at once

### P2 — Controlled approximation for the parametric path
When closed form does not exist (freeform x anything), the answer is approximate BY
NATURE. Industry practice is not to hide that — it is to BOUND it and RECORD it.
- Start-point discovery separated from refinement (Law 4): polyhedral seeding must
  guarantee every branch has a seed before any marching. `SESSION_SEED2` exists and is
  measured; per `kb/audit_occt_intpolyh-seeding.md` the spec's coverage guarantee is
  FALSE, its large/small refinement rule is INVERTED, and its retry loop as written HANGS
- Marching with a controlled step: OCCT's `UVMaxStep` 1e-3 floor is COMMENTED OUT
  (`IntPatch_Intersection.cxx:163`) and drops to 1e-4 near apexes/poles — a one-line
  guard worth trying first
- Approximation error must be written into the **per-entity tolerance** of the edge it
  produces, so downstream welds compare against `tolA + tolB + fuzz` rather than a global
  epsilon. Note the audited correction: monotone growth is the DESIGN INTENT, not an
  OCCT invariant — five shrink-capable writers exist; port per-site
- Cap before the approximator, not after: a failed approximation turns a 250k-point line
  into a 250k-pole curve (`GeomInt_IntSS::MakeBSpline`)
- **GATE**: freeform x box closes with volume within a STATED tolerance, and that
  tolerance is derived from the recorded approximation error rather than asserted

### P3 — Degenerate topology: seams, poles, single-face solids
The freeform blob is one periodic BSpline face with a seam and two zero-length pole
edges — sphere topology, the hardest legitimate input. Both operands verified closed and
valid, so the defect is entirely ours.
- The freeform difficulty ladder (L0 boxes -> L1 no-seam/no-pole -> L2 seam -> L3 poles
  multi-face -> L4 single periodic face -> L5 freeform x freeform), plus variants where
  the cut passes THROUGH a seam and THROUGH a pole, locates the exact failure rung (in
  flight)
- Whatever rung breaks first is the fix: seam handling, degenerate-edge (pole) handling,
  or single-face topology
- **GATE**: every rung of the ladder closes, with the cut-through-seam and
  cut-through-pole variants closing too

### P4 — Coincidence [LANDED TODAY]
A-op-A passes all three ops (cut EMPTY, common = A at 20 faces / 80.3011, fuse = A, no
timeout) via three mechanisms in order: skip SSI on identical surface pairs; suppress
common-block chains lying on an edge of BOTH operands; stand down the legacy ON-imprint.
Guards green with the gate on: base 3 ops exact, matrix 45/45, edge 54/54.
- Remaining: **partial** coincidence (a face only partly shared) genuinely needs
  common-block synthesis, since suppression is not available when a section must exist
  along part of the face. Build on `SharedEdgePool`; reference is
  `BOPAlgo_Tools::PerformCommonBlocks` + `ComputeToleranceOfCB`
- **GATE**: partial-coincidence cells from the stress design close; A-op-A stays green

### P5 — Interference completeness (EF/VF paving)
An architecture panel identified that this kernel appears to have **no edge-face /
vertex-face interference paving stage at all**, where OCCT spends two full source files
(`BOPAlgo_PaveFiller_4/5.cxx`), and attributed x20 and x13y29 to its absence. That claim
is being verified against our source now — it was asserted without checking.
- If confirmed: an EF stage computes where an edge of one operand pierces a face of the
  other and injects a pave/vertex there. Without it, those crossings simply have no
  vertex, and no amount of downstream repair can invent one
- **GATE**: for x20 and x13y29, every true edge-face piercing point (computable
  independently) has a corresponding vertex in our result

### P6 — Validation at scale, continuously
Already largely built by the corpus work; the job is to keep it honest.
- T0 ledger 132 cells with five verdicts, pinned to a kernel sha1
- Metamorphic invariants as the PRIMARY oracle: partition identity, rigid-motion
  equivariance, A-op-A idempotence, and (new) rotational-sweep continuity — volume must
  vary continuously with rotation angle; a discontinuity localises the exact angle where
  the kernel breaks, which no fixed placement can reveal
- The portable OCCT corpus: 371 distinct geometry pairs / 1484 cases carrying four
  independent expected-area assertions each, plus 237 self-contained bug regressions —
  the only public corpus that ships ground truth
- **Interop, which was never tested**: OCCT-produced B-Reps used as OPERANDS in our
  kernel, and our own output re-fed at chain depth 3. A kernel that only works on
  geometry it authored itself is not industry-grade
- **GATE**: nightly, on a fixed binary, with a declared noise band (prior runs varied by
  +-2 cells, so several "improvements" this campaign were inside the noise)

---

## 3. Sequencing, and why this order

**P0 then P1 first.** P1 is the highest-value rung available: bounded, testable without
the chairs, and if the pose-dependence hypothesis holds it removes the dominant error
from every curved config simultaneously. It also settles the architecture question
empirically — if fixing analytic pose-dependence moves the rotated frontier materially,
the splitter architecture was never the blocker.

**P3 in parallel** (different code, different owner): the freeform ladder is independent
of P1 and its result is a single actionable sentence.

**P5 verification in parallel**: cheap, and if EF paving really is absent it changes the
plan more than anything else here.

**P2 after P1**, because the approximation budget only makes sense once the exact path is
taken wherever an exact path exists. Building error control for cases that should have
been exact would institutionalise the wrong answer.

**Architecture decision deferred, deliberately.** The panel recommended completing the
pave-block path (a 2-4 month scope, deleting ~5000 of brep.cpp's 12045 lines) but its
decisive evidence — that patching has zero measured slope — was collected before
coincidence was fixed and before the pose-dependence pattern was found. The honest
sequence is: P0 + P1 (days, not months), then re-ask. If P1 moves the frontier, the
architecture is not the blocker. If it does not, the case for the rewrite is far stronger
than it is today, and will rest on measurement rather than doctrine.

---

## 4. What "industry grade" means here, stated concretely

Not "no failures" — 38% of OCCT's own 25-year bug history is silently-wrong boolean
output, and OCCT itself is self-inconsistent on z15 and fails BRepCheck on three of the
30 reference cells we generated. The realistic bar:

1. **Closure is structural, not achieved**: naked 0 AND non-manifold 0, verified on the
   EXPORTED file by an independent reader
2. **Correctness is checkable without an oracle**: partition identity, equivariance,
   idempotence, sweep continuity — all green
3. **Accuracy is stated, not hoped**: exact where closed form exists; elsewhere a bounded
   error recorded in per-entity tolerances
4. **Degeneracy is typed, not survived**: coincidence, tangency, seams and poles produce
   named outcomes rather than lucky near-zeros
5. **Foreign geometry is consumable**: OCCT/Parasolid-authored B-Reps work as operands,
   and our own output survives being fed back
6. **Regression is impossible to hide**: fixed corpus, pinned binary, declared noise
   band, no silently-skipped cells
