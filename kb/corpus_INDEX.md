# corpus_INDEX — routing index for the boolean test-corpus work (2026-07-25)

Five deliverables: three corpus mines (what external test material exists and what it costs),
one independent re-verification of our own shipped matrix results, one design for the stress
corpus we must author ourselves. Nothing here changed kernel code.

## Deliverables

- **`kb/corpus_occt_primitive_cases.md`** (1019 lines) — machine census of all 4093 OCCT
  `tests/boolean` case files plus full port tables for the 1581 self-contained primitive-only
  cases: the 371-pair core grid, DRAW→`BRep::create_*` translation semantics, `checkprops`
  assertion semantics, hard-cell classification, the seam-invariance families, TODO exclusions.
  **Finding:** 1484 of those cases collapse to 371 byte-identical geometry pairs × 4 ops (four
  independent oracles per pair), 1288 cases are constructible with today's primitives, and the
  single blocker on the remaining 196 is that all 64 `pcone` instances are truncated frustums —
  we need `create_cone(r1,r2,h)`.

- **`kb/corpus_occt_bug_cases.md`** (717 lines) — the 25-year OCCT bug history mined for booleans:
  1252 boolean cases in `tests/bugs`, the 237 fully self-contained ones tabled individually, an
  18-class defect taxonomy assigned 1:1, the 92 TODO cases clustered, and the external-data
  acquisition cost (1163 distinct files, none in the repo).
  **Finding:** a mature kernel's shipped boolean defects are dominated by silently-wrong output
  (WRONGRESULT 16.8%) and topology **metadata** (CONTAINER 15.2% — Closed flag lost, compsolid
  demoted, stray INTERNAL orientations), not crashes (5–7%); geometrically correct faces are
  routinely still a bug.

- **`kb/corpus_public_datasets.md`** (578 lines) — survey of 13 public corpora (ABC, Fusion 360
  Gallery, AutoMate, DeepCAD, BenchCAD, CC3D, MFCAD/MFCAD++, Thingi10K/ModelNet/ShapeNet, FreeCAD,
  CadQuery, academic benchmarks) with measured sizes, licences, a ranked acquisition plan, an
  oracle doctrine for truth-free corpora, and a parquet manifest schema.
  **Finding:** OCCT is the only public B-Rep boolean corpus that ships expected results (3754 of
  4093 cases carry a numeric `checkprops`, 3306 reproducible from free downloads) — every other
  dataset supplies models only, so for all of them the oracle must be metamorphic invariants plus
  OCCT-as-fallible-peer; Fusion 360 Gallery is licence-disqualifying for commercial use.

- **`kb/corpus_matrix_verification.md`** (268 lines) — independent FreeCAD/OCCT re-read of all 45
  shipped primitive-matrix STEP results: per-cell table (corrected vs naive naked edges, shipped
  vs in-memory volume, OCCT edge tolerance, polyline pole counts), failure classes, and an audit of
  whether our own gate could have caught them.
  **Finding:** topology is clean 45/45 (zero fragmentation, zero open shells, zero naked edges,
  solids and faces == OCCT), but only 23/45 shipped STEPs reproduce the oracle volume at 1e-6 while
  the in-memory gate passes 45/45 — the gate measures a different shape than the file contains,
  root-caused to result section edges written as degree-1 polylines up to 129 poles.

- **`kb/corpus_stress_matrix_design.md`** (585 lines) — the degenerate-boolean stress corpus we must
  author: 101 distinct pairs in 8 static classes (FC/EC/TG/PT/SP/MS/CN/SC) + 18 epsilon-sweep
  families × 17 offsets = 407 configurations / 1221 op-cells, each with a literal `main_7.cpp`
  `Place`, an analytic expected volume/solid count, and a 6-bit acceptance rule.
  **Finding:** each degeneracy class has its own volume-vanishing exponent (p=1 face, 1.5 curve,
  2 point/edge, 3 vertex/apex) and the kernel's chain-join band `tol3 = max(50·tol, diag·5e-4)` =
  2.83e-3 on a 4×4 face sits **above** the entire interesting epsilon range — the predicted cliff
  for every grazing family is ~3e-3, three decades before the sweeps bite.

## For session C (owns `validation/` + `corpus/`)

`corpus/runner.py` + `corpus/invariants.py` already run T0 (chairs base + 10 rotated + matrix +
edge) with five verdicts and three invariant families. Wiring order for the new material:

1. **Seam-invariance families first — free, no new oracle.** Six 4-member families (ZD4–ZD7,
   ZH5–ZH8, ZK1–ZK4, ZL2–ZL5, ZL6–ZL9, ZM7–ZN1) are the same coaxial equal-radius geometry with B
   spun 0/90/180/270° about its own axis; OCCT expects identical areas for all four. Encode as a
   fourth invariant family (seam equivariance) in `invariants.py` — 24 pairs / 96 cells, catches any
   seam-position dependence with zero external numbers.
2. **OCCT core corpus (322 pairs / 1288 cases).** Translation is mechanical: `create_box` is
   origin-centred with full side lengths, so every DRAW box needs a translate by
   `(x+dx/2, y+dy/2, z+dz/2)`; cylinder/sphere/torus map 1:1. Three prerequisites before this can
   run: (a) **a surface-area accessor** — every core assertion is `checkprops -s` = AREA and
   `src/brep.h:168` exposes `volume()` only; (b) treat the corpus oracle as **1% relative**
   (`depsilon 1e-2`), a coarse gate, keeping our own 1e-6 for analytic cells; (c) blacklist
   `bopfuse_simple/ZP6` (the only TODO in the portable corpus). The other 49 pairs / 196 cases stay
   parked behind `create_cone(r1,r2,h)`.
3. **Stress matrix, in the design's own risk order:** FC+CN (needs no SSI code, only same-domain
   typing) → GZ-F/GZ-J/GZ-E (the three p=1 sweeps, cheapest direct measurement of the tol3 cliff) →
   SP → EC/TG/PT → MS/SC. GZ needs a new verdict shape: acceptance is a **family** property (sign
   discipline, power law within 5%, monotonicity, topological stability) and the deliverable is the
   per-family `cliff` number, reported rather than pass/failed, with the per-family volumetric
   floors below which only topology is gated.
4. **The 237 self-contained bug cases** last among the runnable sets: their assertion mix is
   `checknbshapes` 135 / `checkshape` 132 / `checkprops` 106, so they need an entity-count
   comparator (V/E/W/F/Sh/So) and a container/Closed-flag check the ledger has no field for today —
   that comparator is what makes the 15% CONTAINER class testable at all. Exclude the 19 TODO cases
   as oracles.
5. **External data only after 1–4:** `opencascade-dataset-7.9.0.tar.xz` (98.7 MB) buys +1291
   oracle-carrying cases; MFCAD++/ABC/DeepCAD are volume without truth and must go through the
   invariant engine, never a dataset-supplied answer.

Engine/metric changes forced by `corpus_matrix_verification.md` (all four are its §5 list):
- Compare `shell_count_of()` against an expected solid count — multi-solid results are **correct**
  (4 matrix cells, chairs cut z15/z30/z45/z37/z63 = 2 solids, x13y29 common = 3); a harness that
  asserts 1 is wrong.
- Replace the naive naked-edge metric (`ancestorsOfType==1`) with wire-occurrence < 2 excluding
  degenerate edges: the naive one false-positives on pristine primitives (cyl 1, cone 2, tor 2,
  sph 3) and reports 126 phantom naked edges across the matrix.
- Round-trip the exported STEP and re-measure against the oracle; this alone converts today's
  silent 22/45 into a visible verdict.
- Run the partition identities on **exported** volumes — they flag 9 of 15 matrix pairs with no
  oracle at all — and tighten `PART_TOL` (`corpus/invariants.py:40`, currently 0.01) to 1e-3 for
  analytic-primitive cells.
- Reader detail that fabricates failures if missed: multi-solid results occupy `shapes[2:]`, not
  `shapes[2]`.

## For session A

Residue classes from `kb/p1_attack_plan.md` (A 71, B 70, C 18, F 9, G 5 over the 174-edge census;
current frontier 108) map onto the stress design as follows:

- **A (one-sided seg coverage) + F (unmerged identical mate pair)** → class **FC** (27 rows: face
  full/part/coplanar, equal-radius box×box / sph×sph / cyl×cyl / tor×tor, cyl flush) and **CN**.
  These are the cells where SSI returns no curve at all, so every edge must come from same-domain
  typing plus boundary imprinting — i.e. they test M1 SEG-ADOPT / NK-RESCUE directly, with no
  marcher in the loop to blame.
- **B (divergent invented seg=−1 closure) at near-coplanarity** → **GZ-J**: two 4-unit boxes flush
  at angle φ about z, overlap volume `V = 8·φ_rad` exactly (verified 7.999996–8.000138 over
  φ = 1e-9…1e-5°, 0 at φ=0). φ=0 is the passing control (the z30x20 analogue), φ>0 is where every
  rotated chairs config lives. Two primitives, no imported STEP, no freeform: if its cliff lands at
  ~3e-3 it is the same defect as the chairs frontier and is a 40-line repro of it.
- **C (junction undershoot at valence-1 scaffold pairs)** → class **SP** (15 seam/pole rows,
  all new) against the seam decomposition already in `brep_section`, plus **EC/TG/PT** for the
  tangency route. Note **TG10** (cylinder tangent to a cone along a generatrix) and its sweep GZ-Q
  are knowingly unreachable through the OCCT analytic ladder we are porting — OCCT itself returns
  nothing there — so they need a path of our own, not a port.
- Scale sensitivity: **SC** re-runs the same defects at other magnitudes; the absolute floor
  `50·Tolerance::ABSOLUTE = 5e-8` overtakes `diag·5e-4` once the bbox diagonal drops below 1e-4,
  which is the predicted total-collapse scale. One cell (**PT07**) needs the arbitrary-axis `xf_of`
  upgrade; everything else fits today's single-axis `Place`.

What the matrix verification means for the acceptance gate:

- The topology half of the 45/45 claim is **real and independently confirmed**: Class 0 is absent
  from the primitive matrix — zero fragmentation, zero open shells, zero genuinely naked edges,
  solids and faces == OCCT in 45/45 under a foreign reader, including the four cells whose correct
  answer is 2 disjoint solids. No time should be spent hunting fragmentation there.
- The gate itself is weaker than its number implies. `main_7.cpp:413` is
  `rel<1e-6 AND nf==occt_nf AND is_solid()`; `is_solid()` ("every edge has 2 trims") cannot
  distinguish 1 shell from 4 and prints `solid=1` for the four legitimately 2-solid cells, and
  `shell_count_of()` (`main_7.cpp:41`) is only wired into the rotated-chairs debug prints. A genuine
  fragmentation would have looked identical to a pass.
- On geometry the gate hides 3–3.5 orders of magnitude: 45/45 in-memory vs 23/45 exported at the
  same 1e-6 threshold against the same oracle, worst `box ∩ tor` 4.35e-3. The cause is downstream of
  the boolean — 16 cells write result section edges as degree-1 polylines (up to 129 poles, OCCT
  edge tol to 1.05e-3), proven by `box ∩ cone` whose answer *is* the cone: the operand's base circle
  round-trips as an exact STEP CIRCLE, the result's copy of the same circle comes back as a 129-pole
  polyline. Both operands round-trip to ≤8e-16, so the defect is in the result path only.
- Reference-free corroboration, usable in-session: the shipped volumes break
  `vol(A−B)+vol(A∩B)=vol(A)` on 9 of 15 pairs (worst 5.05e-4) while the OCCT cache satisfies the
  same identities to 4.3e-12 — the inconsistency is ours, no oracle required to see it.
- Practical consequence for the acceptance gate: keep 1e-6 in-memory (already met), but a matrix
  cell should not be called green until the **exported** STEP re-measures at 1e-6 and the shell
  count matches a reference; today that is 23/45, not 45/45. Separately, the SESSION_FREEFORM probe
  is genuinely broken and the matrix already reports it (open 8-face and 6-face shells with naked
  edges, fragmented fuse, rational-NURBS operand failing round-trip at 7.2e-3) — that one is not
  being hidden.
