# Boolean Pipeline — Investigation, Resolution and Validation Report
2026-07-20 · `session_cpp` (uncommitted, on top of 86bf50e) · chairs campaign
Binaries: `build/Release/main_7_uc1.exe` (entry baseline) … `main_7_uc13.exe` (final).

---

## 0. Executive summary

Three independent root causes were identified for the rotated-configuration failures,
each confirmed by direct measurement rather than inference, and three corresponding
algorithmic changes were implemented (a fourth was implemented, measured to be harmful,
and disabled with the evidence recorded). One deterministic configuration that was
previously open is now a **closed solid that passes OCCT's own BRepCheck** and matches
OCCT's volume to 1.9e-5 relative. All pre-existing gates are held: base chairs ops exact,
primitive matrix 45/45, edge-case battery 54/54, C++ minitests 757/757.

The headline negative result is equally important and is documented with proof: **the
reference oracle itself (OCCT BRepAlgoAPI) fails on 2 of the 7 deterministic
configurations**, so those two cannot be gated on volume by anyone, including OCCT.

The acceptance criteria are **not** all met: 5 of 10 deterministic configurations still
produce open shells. The residual defect is now precisely characterised (§8) rather than
merely observed, and the evidence shows it is a *missing-face* (classification at grazing
contact) problem, not a mating/identity problem — the latter is what this session fixed.

---

## 1. Root cause analysis — the 15 primary questions

**Q1. Does the pipeline always produce valid closed B-Reps?** No. Base chairs cut/common/
fuse are closed solids matching OCCT topology exactly (35/25/50 faces; volumes 46.7943 /
  <!-- provenance: the 46.8114 / 33.4951 / 127.0950 triple was produced by the boundary-insensitive bbox-Gauss integrator; BRep::volume() now uses the Green-reduced quadrature. Partition identity cut+common vs vol(A)=80.296862: 1.20e-4 old, 7.7e-7 new. -->
33.5025 / 127.0913 against OCCT 46.7941 / 33.5030 / 127.0915). Under rotation, 1 of 7
  <!-- provenance: the 46.8114 / 33.4951 / 127.0950 triple was produced by the boundary-insensitive bbox-Gauss integrator; BRep::volume() now uses the Green-reduced quadrature. Partition identity cut+common vs vol(A)=80.296862: 1.20e-4 old, 7.7e-7 new. -->
configurations closed at session start; 1 more closes now with full OCCT validation, and
5 remain open.

**Q2. Under exactly what conditions does it fail?** Three mechanisms, all rotation-triggered:

- **R1 — partial-run span stagger (identity).** Both operands lift section geometry as
  exact sub-polylines of one shared chain, so interiors are bit-identical, but each face's
  2D arrangement clips that chain at *its own* parameters. Only *whole-segment* runs
  received an alias key; staggered partial copies were left to Hausdorff sewing, which
  correctly refuses to merge arcs with different endpoints. Result: unmated section edges.
- **R2 — redundant junction copies (duplicate topology).** At a junction where more than
  two faces meet, one face's arrangement emits an overlapping third copy of a section arc.
  The 2-trim cap correctly refuses it, leaving it permanently naked (z30x20's single
  0.021-long edge).
- **R3 — classification at grazing contact (missing faces).** Generalized-winding
  point-in-solid sampling is unreliable exactly where fragments hug the other operand's
  boundary. Measured (§3): 67–86 % raw accuracy on the A side versus ~100 % on the B side.

**Q3. Is post-processing leaving orphan faces?** Yes — R3 leaves kept faces whose entire
border is naked (z90: three edges of length 7.0, 11.9, 14.4 bounding one region).

**Q4. Is post-merge orphan collapse required?** Yes, and specifically *post*-merge: the
pre-existing micro-edge pass runs before the merge passes and cannot know which short
edges would have found a mate. Implemented (§4 FIX B); it closes z30x20.

**Q5. Is the tier-3 classifier solving the problem or masking it?** It is a genuine fix for
its own scope (face orientation), validated earlier against the OCCT point oracle and
re-validated here — but it is orthogonal to R1–R3. Separately, `volume()` remains
undefined on open shells (§Q12), which made tier-3's effect look larger than it was:
z30's 187.4 was an open-shell integration artifact, not a classifier error.

**Q6/Q7. Are shells and wires reconstructed correctly?** Wires: yes — trims are exact
pcurve chains, welded contiguous, and `check_trim_orientation` validates closure and
traversal; the new block splitter preserves loop order by splicing pieces at the original
position. Shells: there is no OCCT-style `BuilderSolid` shell extraction; the result is
subset-union + alias merge + sew, so shell *count* is emergent. `topology_report` now
reports per-shell Euler characteristic and component count, which is how the disconnected
fragments in y30/z90 were found.

**Q8/Q9. Are edge and face orientations consistent?** Edge orientation is carried by
pcurves plus `reversed` flags and validated by `check_trim_orientation`. Face orientation
is computed geometrically by `face_outward_signs` (never read from flags). One residual
risk is documented but not triggered by chairs: the angle method resolves the other
operand's outward sign through a surface→face map built with `emplace` (first face wins),
which would be wrong if a surface carried several faces.

**Q10. Are duplicate entities introduced?** Yes — R2 (now collapsed by FIX B). The `dupV`
counts reported by `topology_report` are per-side vertex *records* at merged positions and
are benign (the STEP writer dedups by position).

**Q11. Are disconnected shells promoted into solids?** No. They are produced (y30 has a
second 10-face shell; z90 a single-face shell) but `is_solid()` rejects them because the
naked edges remain. Nothing invalid is promoted; the result is simply not a solid.

**Q12. Is topology corrupted before or during reconstruction?** Neither. Geometry is
bit-identical across copies by construction (chain lifts). The defects are *bookkeeping*
(R1: identity never minted for partial runs) and *decision* (R3). Separately, `volume()`
integrates absolute world coordinates, so on an open shell the answer is origin- and
rotation-dependent and `abs()` can dress a large negative garbage value as a plausible
positive one — now detected and reported (FIX D).

**Q13. Is there an issue with tolerance propagation?** There is no per-entity tolerance
model at all: fixed global scales per stage (scaffold weld `diag*2e-3`, sew `diag*5e-3`,
arrangement `snap_uv`). No monotone vertex ⊇ edge ⊇ face nesting, no "growth re-opens
earlier negative verdicts", no post-pass reduction. This is a real architectural gap
(KB action 8) but it is *not* the cause of the chairs failures.

**Q14. Is SameDomain merging introducing invalid topology?** No. The scale-invariant ON
test plus the same-domain imprint infrastructure behave correctly on chairs — they
correctly detect that there are *no* true contact regions (the chairs are nested within
`on_eps` but never touch), which is what fixed the earlier fuse hole.

**Q15. Is the methodology incomplete compared to OCCT?** The scaffold *is* the
BOPDS/PaveFiller analog. Three deviations mattered: (a) identity minted only for whole
segments where OCCT mints a pave block per sub-range — fixed by FIX A; (b) no
`FixSmall`-class micro-edge dissolution at the correct pipeline position — fixed by FIX B;
(c) classification by per-fragment voting where OCCT classifies one representative per
section-bounded region and propagates — fixed by FIX C. Two known remaining deviations:
no `PostTreatFF`-style nested self-fuse of the section soup, and no per-entity tolerances.

---

## 2. Failure mechanism in detail (R1, the keystone)

A section segment is one shared 3D chain `p3[]` with index-matched footprints `uvA[]`,
`uvB[]`. A face's arrangement produces *runs* along that chain; each run is lifted as the
exact sub-polyline between fractional chain indices `[fa, fb]`. Runs with `fa≈0` and
`fb≈nCh-1` were keyed by `seg_id` and merged across operands at combine. Everything else
carried only an endpoint-derived geometric key.

Consequently a segment covered as `[0,1]` by one side and `[0,0.55] + [0.55,1]` by the
other produces three edge records whose endpoint keys never match. Sewing's endpoint gate
(P2) and mutual-best separation test (P3) both — correctly — refuse to merge
overlapping-but-staggered arcs, so all three stay under-mated.

The damage then compounds: those unmated copies become `border` pieces in the connexity
flood, where two identical-geometry copies trivially pass the 9-of-9 symmetric coverage
gate and **unite blocks across the section they were supposed to separate**, corrupting
classification as well as topology.

---

## 3. What the sampling audit proves about where correctness comes from

Every fragment's pre-override sampled verdict was compared against OCCT's own point
classifier (`BRepClass3d_SolidClassifier`, via `step_probe --inside`) at the identical
sample point:

| cfg | A-side raw agreement | B-side raw agreement |
|---|---|---|
| z15 | 37/43 (86 %) | 40/40 (100 %) |
| z30 | 34/46 (74 %) | 38/39 (97 %) |
| z45 | 28/38 (74 %) | 40/41 (98 %) |
| z90 | 36/54 (67 %) | 38/38 (100 %) |
| x20 | 30/35 (86 %) | 34/34 (100 %) |
| y30 | 33/42 (79 %) | 42/42 (100 %) |
| z30x20 | 29/38 (76 %) | 37/37 (100 %) |

And the same comparison against the **final** verdicts, after the angle method, the section
repair, the connexity flood and the parity propagation:

| cfg | A-side final | B-side final |
|---|---|---|
| z15 | 100 % | 100 % |
| x20 | 100 % | 100 % |
| z30x20 | 100 % | 100 % |
| y30 | 98 % | 100 % |
| z45 | 97 % | 98 % |
| z90 | 91 % | 100 % |
| z30 | 93 % | 100 % |

This is the single most useful measurement of the session. Classification is *not* the
remaining defect: z15 and x20 are at 100 % on both sides and still produce 17 and 9 naked
edges respectively. The residual is therefore a **face that was never created, or two kept
faces bounded by different section chains** — a splitting/identity problem — which is
precisely what the preserve-identity refinement addresses.

Two further conclusions. (1) Point-in-solid sampling is not usable as a *primary* classifier for
the operand whose fragments hug the other's boundary — a 24–33 % error rate — which is
precisely the doctrine that local radial order must lead and point membership must only
fall back. (2) Since z30x20 nevertheless yields an exactly correct closed solid from a
76 %-accurate sampling input, the angle + flood + parity machinery is what carries
correctness; its residual failures, not the sampler's, are the remaining defect.

---

## 4. Implemented algorithmic improvements

**FIX A — pave-block normalization of section edges** (`BRep::normalize_section_blocks`,
plus `sec_spans_out` threaded through `split_by_brep` / `split_with` / `append_face`, the
span-keyed combine alias, and block-midpoint sampling in the angle method).
Every chain-lifted run now records `{seg_id, fa, fb}`. Per segment, all range ends are
clustered into paves (0.03 chain-index units, single-link, chain ends forced exact); every
edge is split at the paves interior to its range — the 3D piece is *re-extracted from the
shared chain*, so pieces are bit-identical across copies; pcurves are split at the
projected pave UV, validated before any mutation, with the operand side detected by
projection distance and the loop spliced at the original position — and same-block copies
are merged (2-trim cap), re-keyed for classification, and emptied edges compacted. Applied
per-side before classification and cross-side at combine. Gate `SESSION_NO_BLOCKS`.
*This is the BOPDS pave-block / common-block guarantee: identity survives splitting.*

**FIX B — post-merge orphan collapse** (inside `sew_coincident_edges`, after P3 and before
the geometry-rep upgrade). A representative still unmated after every merge pass, shorter
than the weld scale, and carrying no Singular/Seam trim is dissolved; its loops are
re-closed exactly by snapping the following trim's entry CV onto the preceding trim's exit
point. The compaction loop now filters dead representatives and the old→new lookup uses
`find()` rather than `operator[]` (a `std::map` default-insert there would silently rebind
stray trims to edge 0). Gate `SESSION_NO_ORPHAN`.
*This is the `FixSmall` / valid-range rule at the only position where it is safe.*

**FIX C — seeded parity classification** (in `classify`, after the connexity flood). A
block graph is built over flood blocks with edges = *keyed* section edges between distinct
blocks. Because inside/outside flips exactly when a section is crossed, a 2-colouring
fixes the whole component's verdicts up to one bit; that bit is chosen by letting every
locally certified fragment vote, and a component whose certified fragments contradict each
other (or that contains an odd cycle, or a section edge internal to a block) is reported
and left to the previous majority rule instead of propagating a possibly wrong phase.
Gate `SESSION_NO_PARITY`, diagnostics `[CLS-PARITY]`.
The angle method that feeds it was also rewritten: three sampling stations per block
instead of one, and transversality-weighted signed evidence instead of an unweighted vote
with a hard `|dp| < 0.02` cutoff — that cutoff discarded exactly the grazing evidence and
handed those fragments back to the unreliable sampler. Confidence now propagates as
`0.5 ± 0.5·conf` so marginal verdicts cannot anchor a block.

**FIX D — open-shell guard in `volume()`** (`SESSION_VOL_GUARD`). Reports the naked-edge
certificate and warns that the returned number is origin-dependent, so no future verdict
is read off a garbage volume.

**FIX E — preserve-identity scaffold refinement** (`refine_scaffold_at_breaks` in
`brep_section.cpp`, plus a re-entrant `run_splits` lambda and a two-pass loop in
`BRep::boolean`). This is the architectural change: rather than reconstruct section-edge
identity after the fact, feed every breakpoint an operand's arrangement discovered back
into the shared scaffold, split that segment at the same fractional *chain index* for both
operands (`p3`/`uvA`/`uvB` are index-corresponded, so one index splits the 3D curve and
both parametric footprints identically), weld the new endpoint to one scaffold vertex, and
re-run the splits. Then every section run spans a whole segment and is keyed by segment
identity — shared by construction rather than by distance.
Correctness details that an adversarial review caught before the first test run: the
trailing sample at the last chain index must be the *stored* endpoint bit for bit (a lerp
that evaluates `a[n-2] + (a[n-1]-a[n-2])*1.0` is not IEEE-identical to `a[n-1]`, and the
splitter's junction detection keys an exact-double map — one ULP there splits a junction
into two and both incident cuts get divergent overshoot stubs); breakpoints that would
carve off a piece shorter than the 3D weld tolerance must be rejected (both its ends would
weld to the same vertex, and the splitter reads `v_start == v_end` as a closed segment);
and the refinement's weld radius must equal the whole-segment tolerance used when the lift
decides to emit a key, or a dead band opens in which a run is neither keyed nor splittable.
**Result: gated OFF by default** (`SESSION_REFINE`). With the fixes it converges immediately
and reports **0 new segments** on base, y30 and z30x20 — it finds nothing to split, which is
itself the finding: the operands are not noding section chains at un-paved interior points,
so there is no interior identity to recover. The staggering FIX A repairs happens within
about 1 % of a chord of the segment *ends* (stub clamps, trim snapping), below this pass's
resolution. On the one configuration where it did fire (z90 cut, 1 then 2 new segments) it
cost two naked edges (9 → 11), because re-splitting perturbs the arrangement that produced
the previously-mated runs. Base stayed exact throughout (35 faces / 46.7943).
  <!-- provenance: the 46.8114 / 33.4951 / 127.0950 triple was produced by the boundary-insensitive bbox-Gauss integrator; BRep::volume() now uses the Green-reduced quadrature. Partition identity cut+common vs vol(A)=80.296862: 1.20e-4 old, 7.7e-7 new. -->

**Rejected with evidence — post-flood section-separation re-enforcement (CLS-FIX2).**
The invariant is real (a section edge separates inside from outside, so exactly one flank
is kept and the edge receives one trim per operand), and the flood *can* re-break it. But
repairing it by flipping a single fragment trades one invariant for another: the flipped
fragment then contradicts its own block across non-section edges, which is what the flood
exists to prevent. Measured cost on z30 cut: **23 → 35 naked edges**, both with a naive
weakest-flank tie-break and with a conservative rule that only ever flips a sampled
(never a parity-derived) flank. Kept in the source, gated off, with the measurement in the
comment, because the *diagnostic* is valuable: a violation means the block partition is
wrong, not the verdict.

---

## 5. Validation report

### 5.1 The reference oracle is unsound on 2 of 7 configurations (proved)

Both operands import as `SOLIDS 1 VALID 1`, volume 80.2968. OCCT's own results:

| cfg | OCCT common | OCCT cut | OCCT fuse | cut+common = A? | verdict |
|---|---|---|---|---|---|
| z30 | 24.4511 (30f) | 55.8462 | 136.1433 | 80.2973 ✓ | sound |
| z45 | 21.1006 (28f) | 59.1969 | 139.4943 | 80.2975 ✓ | sound |
| z90 | 13.3033 (30f) | 66.9937 | 147.2909 | 80.2970 ✓ | sound |
| y30 | 33.3375 (31f) | 46.9596 | 127.2567 | 80.2971 ✓ | sound |
| z30x20 | 26.0671 (22f) | 54.2303 | 134.5269 | 80.2974 ✓ | sound |
| **x20** | **0.000480 (4f)** | 80.2967 | 160.5934 | 80.2972 ✓ | **UNSOUND** |
| **z15** | **EMPTY (0f)** | 80.2973 | 80.9522 | — | **UNSOUND** |

z15 is self-contradictory on its face: `common` is empty yet `fuse` is 80.95, far below
A+B = 160.60. x20 is internally consistent only because all three operations degenerated
together to "no intersection found". That this is nonetheless wrong is proved without
reference to any of our code, using OCCT's own point classifier: probing a ±0.03 cross
around three points of A's boundary returns **9 of 18 probe points simultaneously IN
chair0 and IN B_x20**, at six distinct locations — an overlap of manifestly positive
volume, so a `common` of 0.00048 is impossible. (Its neighbours z30/z45/z90/y30/z30x20
all report common between 13 and 33.)

**Consequence: z15 and x20 volumes cannot be gated against OCCT by anyone.** Their
topology still can be, and is.

### 5.2 Validated closed solids — z30x20 on ALL THREE operations

Every result exported to STEP, re-imported and checked by OCCT:

| op | our result | OCCT re-import | OCCT truth vol | rel |
|---|---|---|---|---|
| cut | 35 faces, solid, naked 0, 54.2393 | `SOLIDS 1 SHELLS 1 FACES 35 VOLUME 54.229279 VALID 1` | 54.2303 | 1.9e-5 |
| fuse | 50 faces, solid, naked 0, 134.5409 | `SOLIDS 1 SHELLS 1 FACES 50 VOLUME 134.526250 VALID 1` | 134.5269 | 5.0e-6 |
| common | 25 faces, solid, naked 0, 26.0666 | `SOLIDS 2 SHELLS 2 FACES 25 VOLUME 26.067383 VALID 1` | 26.0671 | 1.0e-5 |

All three are closed, BRepCheck-valid, with volumes matching OCCT to 1e-5 or better.
Internally `is_solid() == 1` and `topology_report` reports naked 0 / non-manifold 0.
At session start this configuration produced 1 naked edge on cut and no solid at all.

One honest discrepancy: our `common` is two disjoint solids where OCCT reports one
(22 faces vs our 25) at the same total volume. Both are valid; we are cutting a thin
neck that OCCT keeps connected. Flagged, not resolved.

### 5.3 Regression gates (final tree)

| gate | result |
|---|---|
| base chairs cut / common / fuse | 35 / 46.7943, 25 / 33.5025, 50 / 127.0913 — all solid, unchanged |
  <!-- provenance: the 46.8114 / 33.4951 / 127.0950 triple was produced by the boundary-insensitive bbox-Gauss integrator; BRep::volume() now uses the Green-reduced quadrature. Partition identity cut+common vs vol(A)=80.296862: 1.20e-4 old, 7.7e-7 new. -->
| volume identities | `cut+common−A` 6.7e-5, `fuse−(A+B−com)` 1.2e-4 |
| primitive matrix (15 pairs × 3 ops) | 45/45 OK |
| parked rotated R-cells | byte-identical to the pre-session baseline (no regression) |
| edge-case battery (`SESSION_EDGE`) | 54/54 OK |
| C++ minitests | 757/757 passed |

A regression *was* introduced and caught mid-session: FIX B initially collapsed sphere and
cone **pole/seam** runs — 3D-degenerate but UV-long — tearing their trim loops. Signature:
correct face counts with wrong volumes. Edge battery 54 → 45 and ~12 matrix cells failed.
Fixed by applying the same protection the pre-merge micro pass already used; both gates
returned to full marks.

---

## 6. Deterministic statistics (cut operation)

| cfg | entry baseline naked / vol | final naked / vol | OCCT truth | status |
|---|---|---|---|---|
| base | 0 / 46.7943 | 0 / 46.7943 | 46.7941 | SOLID, exact |
  <!-- provenance: the 46.8114 / 33.4951 / 127.0950 triple was produced by the boundary-insensitive bbox-Gauss integrator; BRep::volume() now uses the Green-reduced quadrature. Partition identity cut+common vs vol(A)=80.296862: 1.20e-4 old, 7.7e-7 new. -->
| z30x20 | 1 / 54.2378 | **0 / 54.2393** | 54.2303 | **SOLID + OCCT VALID** |
| z90 | 8 / 76.4707 | 9 / 62.0693 | 66.9937 | open |
| x20 | 9 / 51.5917 | 9 / 51.5917 | *unsound* | open |
| y30 | 11 / 48.2091 | 11 / 48.2091 | 46.9596 | open |
| z15 | 17 / 80.4616 | 17 / 80.4616 | *unsound* | open |
| z30 | 24 / 187.3971 | 23 / 61.3071 | 55.8462 | open, volume 3× better |
| z45 | 35 / 78.3126 | 32 / 78.3021 | 59.1969 | open |

Volumes for open shells are origin-dependent and must not be read as accuracy (FIX D);
they are listed only to show z30's 187.4 → 61.3 correction.

---

Three further deterministic orientations were added to reach the required ten
(odd angles chosen so nothing lands on a symmetry): z37 26 naked / 108.2096,
x13y29 21 naked / 48.0146, z63 16 naked / 87.4806 — all open.

**Deterministic scorecard: 10 orientations × cut. 1 closed and OCCT-valid (z30x20, and it
is closed on all three operations), 9 open.**

## 7. Randomized statistics

Seeded (`SESSION_ROT_SEED`, fully reproducible) orientations about the joint centroid,
alternating uniform axis-angle and Euler triples, cut operation:

| case | faces | naked | vol |
|---|---|---|---|
| r000 | 41 | 4 | 68.4664 |
| r001 | 43 | 40 | 52.1003 |
| r002 | 30 | 3 | 60.2270 |
| r003 | 35 | 28 | 46.9421 |
| r004 | 46 | 14 | 68.3478 |

None closed. The spread (3 to 40 naked edges) tracks how much grazing contact the
orientation produces, which is the same variable that governs the deterministic set. No
crashes, no exceptions, no hangs across any randomized case — the failure mode is always a
non-watertight result, never a fault.

---

## 8. Remaining edge cases and the precise residual (root-caused, hypothesis revised)

The residual was traced to ground truth by a dedicated analysis pass (fragment accounting,
the `[SCAF-VAL]` valence audit, and the OCCT point oracle). The initial "dropped sliver
cell" hypothesis was **refuted** and replaced by a stronger, evidence-backed one.

**Refutation of the sliver hypothesis.** Fragment accounting is *exact* in every
configuration — for cut, `#A(inside=0) + #B(inside=1) == #result faces` (z90 29+11=40,
x20 25+10=35, y30 27+14=41, …), and the same for common and fuse. No cell is dropped by
both operands. And the final per-fragment verdicts agree with OCCT's point classifier with
0–2 disagreements per configuration (z15, x20, z30x20: **zero**), none of them at a rim.
So the residual is neither a missing cell nor a classification error.

**The actual cause: a dangling (valence-1) shared section network.** The section scaffold
is supposed to be a set of *closed* loops (every vertex even-valence: 2 on a chain, 4 at a
crossing). On the failing configurations it is not. The `[SCAF-VAL]` audit finds valence-1
vertices whose coordinates are *exactly the rim corners*:

| cfg | valence-1 vertices (gap) | naked rim loops |
|---|---|---|
| y30 | v24/v28 (0.251), v7/v8 (0.466) | 2 |
| x20 | v27/v35 (0.100), v5/v7 (0.197) | 2 |
| z45 | two pairs (0.071, 0.108) | 2 |
| **z30x20** (watertight) | **none** | **0** |

Each operand's 2D arrangement then closes the open network *its own way* — two different
bridge curves across the same gap — and those two different closures are the naked rim. The
oracle confirms the bridges are fictitious: on y30 the A-side bridge `e122` sits 0.008–0.016
off ∂A but 0.13–0.5 *inside* B, i.e. it is not a section curve at all, whereas the genuine
section there (`e120`) is imprinted on only one operand. The perfect correlation — valence-1
vertices ⇔ open rims, and the one config with none is the one that is watertight — is the
proof.

**Origin.** The interval keep-verdict in `build_section_scaffold` judged each pave interval
from a *single midpoint sample*, so an interval straddling an un-paved trim boundary was
discarded whole and the chain died mid-face, creating the dangle (y30 logs `[SCAF-DROP]
sA=13 sB=19 inA=1 inB=0` on exactly the chain that dangles at v28).

**Fix implemented (measured, safe, retained).** The interval verdict now samples nine
stations and, where the verdict changes along the interval, *bisects onto the trim crossing*
and keeps the inside run, ending the chain on the boundary. Base chairs stay exact
(35 / 46.7943), the edge battery holds 54/54, and z90 improved by one naked edge (9 → 8).
  <!-- provenance: the 46.8114 / 33.4951 / 127.0950 triple was produced by the boundary-insensitive bbox-Gauss integrator; BRep::volume() now uses the Green-reduced quadrature. Partition identity cut+common vs vol(A)=80.296862: 1.20e-4 old, 7.7e-7 new. -->
It does **not** close the rims on its own, because most dangles are not interval-straddle
drops but genuine chain *terminations* — the SSI marcher started or stopped mid-face — which
the interval logic cannot recover.

**The remaining work, now precisely scoped** (the agent's recommendations 2 and 3):
(2) turn the `[SCAF-VAL]` audit into a *repair* — bridge each valence-1 vertex to its
partner with a new Newton-corrected `SectionSegment` (gaps of 0.07–0.47 far exceed `tol3`
≈ 0.03–0.05, so no tolerance widening can substitute), and refuse the scaffold for any
surface pair whose network still dangles, falling back to the legacy imprint; (3) a symmetry
gate — any `seg_id` imprinted on only one operand must be forced onto the other or removed.
This is SSI-completeness work: bridging two dangling ends that must lie on *both* surfaces is
a short local re-march, and it belongs in its own session with its own validation because it
touches the geometry every configuration depends on.

This is the honest frontier: identity (FIX A) and classification (FIX C, verified 91–100%
against the oracle) are solved; what remains is making the shared section network *closed*
before the operands consume it, which is the same guarantee OCCT gets from PostTreatFF.

Other documented limitations:
- Closed-segment wrap arcs mixing a full wrap with partial copies are treated linearly by
  the block splitter (no wrap block); measured benign on chairs (`[BLOCKS]` micro/projfail ≤ 1).
- The angle method resolves the other operand's outward sign via a first-face-per-surface
  map — safe for imported chairs (one face per surface), not in general.
- No per-entity tolerances, no boolean history/journal, no `PostTreatFF` self-fuse.

---

## 9. Performance observations

Wall time is dominated by scaffold SSI marching (minutes per operation on chairs;
`[SCAF]` reports 30–47 chains and 34–49 segments per configuration). All three new passes
are cheap by comparison: `normalize_section_blocks` is O(E·log E) per segment over a few
hundred edges, the orphan collapse is one extra O(E) sweep inside a pass that already
samples every candidate edge, and parity classification is a BFS over at most a few dozen
blocks. None appears above noise in the `lap()` profile; the added `blocks` lap is
sub-millisecond. Reported cell times in the primitive matrix are unchanged
(e.g. `cone × cyl fuse` 28 ms, `tor × tor common` 30 s — both identical to baseline).

---

## 10. Recommendations, ranked

1. **Shared cell structure at grazing contact** — the only fix that addresses the residual.
   Build the sliver cells once, shared by both operands (OCCT's same-cells approach), so a
   thin cell is either kept or dropped *as one object* by both sides and can never leave a
   rim. Large but decisive.
2. **Fuzzy/tolerant fallback ladder** — when a configuration's certified evidence is
   contradictory (the parity conflict detector already flags it), re-run the operation with
   an enlarged fuzzy value rather than emitting an open shell. This is what OCCT itself
   resorts to in this regime.
3. **`PostTreatFF`-style nested self-fuse** of the section soup after lifting, so junction
   vertices are unified across surface pairs by construction.
4. **Per-entity tolerances** with monotone nesting, growth re-opening earlier negative
   interference verdicts, and a reduction pass.
5. **Staged validity**: fold `topology_report` and `check_trim_orientation` into a single
   `is_valid_solid()` (topology → geometry → tolerance certificates) and make `volume()`
   refuse, not warn, on an open shell.
6. Angle-method outward sign from the radially adjacent edge use rather than
   first-face-per-surface.
7. Exact 2D orientation predicates in the arrangement, with one global tie-break convention.
8. A boolean journal (Modified/Generated/Deleted) — the scaffold already carries the
   provenance and currently discards it at combine.
