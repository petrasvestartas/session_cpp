# Cross-pair corpus: ranked defect census

Binary under test: `build/main_7_xp`, **sha1 `ebe1cddcebfcc00c98662ef23b6251556e3e7327`**
-- a private copy of `build/main_7` taken once at 22:26 and used unchanged for every run in
this document, so nothing here is contaminated by the edits another session was making to
`src/` during the sweep.

Independent oracle: FreeCAD 1.1.1 headless (`Libs 1.1.1R44227 +647`), i.e. OCCT, run as
`/snap/bin/freecad.cmd`. **No verdict in this document uses the kernel's own reported
numbers.** Every "ours" figure is OCCT re-reading the STEP file our kernel wrote. Section 6
shows why that distinction is not pedantic.

Invocation, unchanged for every case:
`(ulimit -v 4194304; SESSION_NO_ROT=1 SESSION_OP=<op> SESSION_CHAIRS=<dir> timeout <cap> ./build/main_7_xp SKIPMATRIX)`

## 0. Bottom line

1. **The kernel is correct on the chairs configuration and on essentially nothing else.**
   The known-good control passes all three operations against OCCT to 1e-5 relative. Across
   the corpora that complete, **8 of 530 independently-scored operations pass (98%
   fail, 519/530)**, and the handful of passes are degenerate (both results empty, or a fuse
   where one operand contains the other).
2. **What hangs on the foreign-boolean corpora is `volume()`, not the boolean.** 127 op-runs
   on the hard and balanced chair-family sets hit the wall cap and never even printed their
   operand summary -- and that print is the first thing that calls `BRep::volume()`. With
   `SESSION_FAST=1`, which skips only `volume()`, the same pairs load instantly and their
   booleans complete; `pair_063` returns 75 faces, `is_solid = 0` and 47 naked edges. So
   `volume()` does not terminate in 900 s on a 32-face OCCT-authored NURBS solid, and it
   masked every boolean result in corpora A and B. 9 further op-runs timed out elsewhere.
   For reference OCCT performs the same cuts in 2.2-9.7 s.
3. **The provenance axis could not be isolated, because the baseline is already broken.**
   Previous-boolean-result operands fail at 96-97%, but primitive-vs-primitive operands
   fail at 99%. There is no interoperability defect to measure on top of a general-position
   boolean that does not work; fix the latter before the former can be seen.
4. **The most dangerous failures are silent.** 17 results are closed, OCCT-valid,
   free of naked edges and still the wrong solid, including one that is 16x too large and
   several where the kernel simply decided two overlapping boxes were disjoint and returned
   an operand unchanged.
5. **Three defects are pre-boolean.** A sphere ingests as `is_solid = 0, volume 0`; every
   cylinder and cone reports exactly one third of its true volume and a torus three times
   it; the freeform blob also ingests as a non-solid. These poison any downstream metric
   that reads `volume()`.
6. **OCCT is not a usable oracle on 44 of the 80 hard pairs**, including 3 fuses that return
   an empty shape and 11 op-results OCCT marks invalid itself. The balanced set is much
   healthier (8 of 60 untrustworthy), which is what it was built for.

## 1. What actually ran

| corpus | pairs | op-runs attempted | completed | TIMEOUT | crashed | not run | wall-clock cap |
|---|---:|---:|---:|---:|---:|---:|---|
| control: base chairs (known-good) | 1 | 3 | 3 | 0 | 0 | 0 | 900 s |
| A. hard cross-pairs pair_000..079 | 80 | 240 | 0 | 88 | 0 | 152 | 240 s (72 pairs) / 600 s (8 pairs) / 900 s (4 probes) |
| B. balanced cross-pairs bal_000..059 | 60 | 180 | 0 | 39 | 0 | 141 | 240 s |
| C. supplementary primitives spair_000..101 | 102 | 306 | 297 | 9 | 0 | 0 | 300 s |
| D. provenance set pvpair_000..056 | 57 | 171 | 171 | 0 | 0 | 0 | 300 s |
| E. analytic-vs-NURBS control nb | 10 | 30 | 30 | 0 | 0 | 0 | 300 s |
| F. box-box diagnostic battery dg | 11 | 33 | 33 | 0 | 0 | 0 | 300 s |

## 2. Control: the harness and the verification are sound

The same binary, the same invocation, the same export path and the same OCCT verification
were applied to the pair the kernel is known to get right. It passes all three operations
on the independent measure. Everything that follows therefore measures the kernel, not the
measuring apparatus.

| op | kernel's own printout | OCCT re-reading our exported STEP | OCCT ground truth | rel err of our geometry | verdict |
|---|---|---|---|---:|---|
| cut | 35f solid=1 naked=0 vol=46.8114 | 35f/1s closed=1/1 naked=0 valid vol=46.793117 | 35f/1s vol=46.794109 | 2.12e-05 | **PASS** |
| common | 25f solid=1 naked=0 vol=33.4951 | 25f/1s closed=1/1 naked=0 valid vol=33.503354 | 25f/1s vol=33.503026 | 9.79e-06 | **PASS** |
| fuse | 50f solid=1 naked=0 vol=127.0950 | 50f/1s closed=1/1 naked=0 valid vol=127.090340 | 50f/1s vol=127.091435 | 8.62e-06 | **PASS** |

Wall clock for the control: 413, 408, 508 s per operation on the same loaded machine.

## 3. The census

Scoring rule. PASS = the result our kernel wrote is, when re-read by OCCT, closed
(every result shell closed, zero 1-adjacent-face edges), OCCT-valid, volume within 1e-4
relative of the OCCT reference, and the solid count matches. PASS-CLOSURE = closed and
valid, but the pair has no trustworthy oracle (see section on oracle quality) so the
volume was deliberately not scored. FAIL = anything else that produced a result.
TIMEOUT is reported separately and is never folded into a pass rate.

### 3a. By geometry class (corpora that complete: primitives, blob, small foreign booleans)

| class | op-runs | PASS | PASS-CLOSURE | FAIL | TIMEOUT | not run | fail rate of runs that produced a result |
|---|---:|---:|---:|---:|---:|---:|---:|
| ii+iii freeform x analytic-curved | 18 | 0 | 0 | 16 | 1 | 1 | **100%** (16/16) |
| iii-b freeform x planar | 9 | 0 | 0 | 7 | 2 | 0 | **100%** (7/7) |
| ii  at least one analytic-curved | 390 | 2 | 0 | 388 | 0 | 0 | **99%** (388/390) |
| iii-c freeform blob (periodic seam + poles) | 36 | 1 | 0 | 32 | 3 | 0 | **97%** (32/33) |
| i   both planar | 84 | 5 | 3 | 76 | 0 | 0 | **90%** (76/84) |
| iii-a freeform x freeform (deg-3 NURBS) | 3 | 0 | 0 | 0 | 3 | 0 | n/a — nothing completed |

### 3b. By operand provenance

| class | op-runs | PASS | PASS-CLOSURE | FAIL | TIMEOUT | not run | fail rate of runs that produced a result |
|---|---:|---:|---:|---:|---:|---:|---:|
| iv-0 chair operands (authored NURBS) | 30 | 0 | 0 | 23 | 6 | 1 | **100%** (23/23) |
| iv-0 both primitives | 300 | 4 | 0 | 293 | 3 | 0 | **99%** (293/297) |
| iv-2 both are previous boolean results | 75 | 2 | 0 | 73 | 0 | 0 | **97%** (73/75) |
| iv-1 one is a previous boolean result | 135 | 2 | 3 | 130 | 0 | 0 | **96%** (130/135) |

### 3c. The two chair-family corpora, scored the same way

| class | op-runs | PASS | PASS-CLOSURE | FAIL | TIMEOUT | not run | fail rate of runs that produced a result |
|---|---:|---:|---:|---:|---:|---:|---:|
| iv-1 one is a previous boolean result | 108 | 0 | 0 | 0 | 20 | 88 | n/a — nothing completed |
| iv-2 both are previous boolean results | 312 | 0 | 0 | 0 | 107 | 205 | n/a — nothing completed |

## 4. Severity ranking: silent wrong answers

The most dangerous defect is not an open shell -- a downstream validity check catches
that. It is a result that is closed, OCCT-valid, has zero naked edges, and is still the
wrong solid. Those cases are listed here, worst first.

| rank | case | op | A | B | our volume | OCCT volume | rel err | our faces/solids | OCCT faces/solids |
|---:|---|---|---|---|---:|---:|---:|---|---|
| 1 | tet | common | tet_A | tet_B | 21.33333 | 1.33333 | 1.50e+01 | 8/2 | 4/1 |
| 2 | spair_065 | cut | c_box | chair1 | 125.00000 | 96.28540 | 2.98e-01 | 6/1 | 11/1 |
| 3 | spair_065 | fuse | c_box | chair1 | 205.29657 | 176.58222 | 1.63e-01 | 26/2 | 28/1 |
| 4 | tet | cut | tet_A | tet_B | 10.66667 | 9.33333 | 1.43e-01 | 4/1 | 6/1 |
| 5 | inside | cut | inside_A | inside_B | 65.00000 | 63.00000 | 3.17e-02 | 12/2 | 12/1 |
| 6 | sc1 | cut | sc1_A | sc1_B | 64.00000 | 63.18100 | 1.30e-02 | 6/1 | 9/1 |
| 7 | ax3 | cut | ax3_A | ax3_B | 64.00000 | 63.18100 | 1.30e-02 | 6/1 | 9/1 |
| 8 | sc0.1 | cut | sc0.1_A | sc0.1_B | 0.06400 | 0.06318 | 1.30e-02 | 6/1 | 9/1 |
| 9 | sc100 | cut | sc100_A | sc100_B | 64000000.00000 | 63181000.00000 | 1.30e-02 | 6/1 | 9/1 |
| 10 | sc10 | cut | sc10_A | sc10_B | 64000.00000 | 63181.00000 | 1.30e-02 | 6/1 | 9/1 |
| 11 | sc0.1 | fuse | sc0.1_A | sc0.1_B | 0.12800 | 0.12718 | 6.44e-03 | 12/2 | 12/1 |
| 12 | sc1 | fuse | sc1_A | sc1_B | 128.00000 | 127.18100 | 6.44e-03 | 12/2 | 12/1 |
| 13 | ax3 | fuse | ax3_A | ax3_B | 128.00000 | 127.18100 | 6.44e-03 | 12/2 | 12/1 |
| 14 | sc100 | fuse | sc100_A | sc100_B | 128000000.00000 | 127181000.00000 | 6.44e-03 | 12/2 | 12/1 |
| 15 | sc10 | fuse | sc10_A | sc10_B | 128000.00000 | 127181.00000 | 6.44e-03 | 12/2 | 12/1 |
| 16 | spair_052 | cut | boxoff | chair0 | 64.00000 | 63.92296 | 1.21e-03 | 6/1 | 11/1 |
| 17 | spair_052 | fuse | boxoff | chair0 | 144.29657 | 144.21972 | 5.33e-04 | 26/2 | 25/1 |

17 of 530 independently-scored results are silently wrong; the rest fail loudly (open shells / naked edges), which at least a downstream validity check would catch.

### Every PASS in the sweep, enumerated

Small enough to list in full, which is itself the finding. Not one pass involves two solids
whose boundaries genuinely cross: every one is either an empty-vs-empty agreement or a case
where one operand wholly contains the other, so the answer is an operand and no new
topology has to be built.

| corpus | case | op | A | B | why it passes |
|---|---|---|---|---|---|
| supp | spair_039 | cut | blob | sph | both our result and OCCT's are empty |
| supp | spair_047 | cut | box | ffres_s0 | both our result and OCCT's are empty |
| supp | spair_051 | fuse | box | wedge | one operand contains the other, so the union is an operand |
| supp | spair_095 | fuse | ffres_s0 | wedge | one operand contains the other, so the union is an operand |
| pv | pvpair_040 | fuse | R_box_cut_box2_s0 | R_box_cut_sph | one operand contains the other, so the union is an operand |
| pv | pvpair_043 | fuse | R_box_cut_box2_s0 | R_wed_cut_cyl | one operand contains the other, so the union is an operand |
| dg | inside | common | inside_A | inside_B | one operand contains the other |
| dg | inside | fuse | inside_A | inside_B | one operand contains the other, so the union is an operand |

## 4b. Where the chair-family corpora actually hang: `volume()`, not the boolean

Every run in section 1 on corpora A and B is recorded as a timeout, but the timeouts are
not where they appear to be. The kernel prints its operand summary immediately after
reading the two STEP files, and that print calls `A.volume()` and `B.volume()`. On corpora
A and B **the summary line never appeared at all**, at 240 s, at 600 s, or at the full
900 s cap -- so nothing had been measured about the boolean itself.

Re-running with `SESSION_FAST=1`, whose only effect is to skip `volume()` and the STEP
export, changes the picture completely. On `pair_063` (`occtref_y30_cut` 32 faces x
`occtref_z63_fuse` 53 faces) the milestones are: **STEP import + operand summary at 2 s**,
**cut complete at 185 s**, returning 75 faces, `is_solid = 0` and 47 naked edges. The same
pair with `volume()` enabled never reached the 2-second milestone in 900 s.

So the whole boolean costs about 183 s, while a single `BRep::volume()` call on a 32-face
OCCT-authored NURBS solid does not return in 900 s -- at least a 5x difference in the wrong
direction, on a call that exists only to print a diagnostic. The boolean does run on
foreign boolean-result geometry; it produces a badly open result; and `volume()` masked
that fact across all 140 pairs of corpora A and B.

That makes `volume()` the single highest-value defect in this report: it is unusable as a
metric (section 5, section 6), and on foreign NURBS input it does not return at all.

Measured time to each milestone with `SESSION_FAST=1`:

```
pair_063 operand_summary_at=2s cut_line_at=185s
pair_000 operand_summary_at=2s cut_line_at=215s
```

**A. hard set with `volume()` disabled** -- 0 of 240 op-runs produced a result; 0 exceeded the 420 s cap without one; 240 were not reached before this report was written (the sweep runs alphabetically, so `bal_*` precedes `pair_*`). 

**B. balanced set with `volume()` disabled** -- 27 of 180 op-runs produced a result; 0 exceeded the 420 s cap without one; 153 were not reached before this report was written (the sweep runs alphabetically, so `bal_*` precedes `pair_*`). Of the results obtained, **0 are closed solids (naked = 0 and is_solid = 1)** and **27 are open or not classified as solids**. 

Worst open results (kernel's own naked-edge count; no STEP is exported in this mode so these are not independently verified):

| pair | op | A | B | faces | is_solid | naked edges |
|---|---|---|---|---:|---:|---:|
| bal_005 | common | occtref_y30_fuse | occtref_z30_fuse | 97 | 0 | **67** |
| bal_005 | cut | occtref_y30_fuse | occtref_z30_fuse | 51 | 0 | **57** |
| bal_001 | cut | occtref_x20_fuse | occtref_z90_cut | 92 | 0 | **47** |
| bal_004 | cut | occtref_x20_fuse | occtref_z63_cut_s0 | 44 | 0 | **45** |
| bal_001 | common | occtref_x20_fuse | occtref_z90_cut | 42 | 0 | **42** |
| bal_004 | common | occtref_x20_fuse | occtref_z63_cut_s0 | 84 | 0 | **40** |
| bal_004 | fuse | occtref_x20_fuse | occtref_z63_cut_s0 | 24 | 0 | **31** |
| bal_005 | fuse | occtref_y30_fuse | occtref_z30_fuse | 11 | 0 | **29** |
| bal_000 | fuse | occtref_x20_cut | occtref_x20_fuse | 62 | 0 | **13** |
| bal_001 | fuse | occtref_x20_fuse | occtref_z90_cut | 52 | 0 | **7** |
| bal_000 | cut | occtref_x20_cut | occtref_x20_fuse | 10 | 0 | **5** |
| bal_000 | common | occtref_x20_cut | occtref_x20_fuse | 40 | 0 | **5** |

### Foreign boolean results our kernel refuses to accept as solids

Read from the `SESSION_FAST` sweep, where the operand summary is reachable. Face counts
match OCCT exactly for every operand, so the STEP reader itself is faithful; what differs
is the closure verdict. 12 distinct chair-family operands were observed and **2 of them are
classified `is_solid = 0`** although OCCT reads the same file as a closed, valid solid.
A boolean whose operand is not considered a solid cannot produce a correct result, so this
is upstream of everything else in corpora A and B.

| operand | OCCT faces | our faces | our `is_solid` | OCCT verdict |
|---|---:|---:|---:|---|
| `occtref_x20_fuse` | 68 | 68 | 0 | closed shell, `isValid` true |
| `occtref_z30_fuse` | 60 | 60 | 0 | closed shell, `isValid` true |
| `occtref_x13y29_fuse` | 50 | 50 | 1 | closed shell, `isValid` true |
| `occtref_x20_cut` | 38 | 38 | 1 | closed shell, `isValid` true |
| `occtref_y30_fuse` | 54 | 54 | 1 | closed shell, `isValid` true |
| `occtref_z45_fuse` | 55 | 55 | 1 | closed shell, `isValid` true |
| `occtref_z63_cut_s0` | 37 | 37 | 1 | closed shell, `isValid` true |
| `occtref_z90_cut` | 36 | 36 | 1 | closed shell, `isValid` true |
| `rotB_z30` | 20 | 20 | 1 | closed shell, `isValid` true |
| `rotB_z45` | 20 | 20 | 1 | closed shell, `isValid` true |
| `rotB_z63` | 20 | 20 | 1 | closed shell, `isValid` true |
| `rotB_z90` | 20 | 20 | 1 | closed shell, `isValid` true |

## 5. Operand ingestion: defects that exist before any boolean runs

Before any boolean is attempted, the kernel prints what it made of each imported operand.
These are OCCT-authored STEP primitives that OCCT itself reads back exactly. Planar solids
ingest perfectly; every curved one is wrong, and two of them are not even recognised as
solids, which by itself explains a whole block of downstream failures (an operand with
`is_solid = 0` and volume 0 is subtracted as if it were nothing, so `A cut B` returns `A`).

| operand | OCCT faces | OCCT volume | kernel faces | kernel is_solid | kernel volume | kernel/true |
|---|---:|---:|---:|---:|---:|---:|
| `tor` | 1 | 25.266187 | 1 | 1 | 75.7986 | **3.00000** |
| `c_tor` | 1 | 38.373022 | 1 | 1 | 111.2975 | **2.90041** |
| `c_sph` | 1 | 73.622177 | 1 | 0 | 0.0000 | **0.00000** |
| `sphoff` | 1 | 44.602238 | 1 | 0 | 0.0000 | **0.00000** |
| `sph` | 1 | 65.449847 | 1 | 0 | 0.0000 | **0.00000** |
| `b_sph` | 1 | 28.730912 | 1 | 0 | 0.0000 | **0.00000** |
| `chair0` | 20 | 80.296907 | 20 | 1 | 19.2149 | **0.23930** |
| `cone` | 2 | 16.755161 | 2 | 1 | 5.5810 | **0.33309** |
| `b_cyl` | 3 | 30.410617 | 3 | 1 | 10.1389 | **0.33340** |
| `cyl` | 3 | 42.411501 | 3 | 1 | 14.1403 | **0.33341** |
| `cylx` | 3 | 27.143361 | 3 | 1 | 9.0503 | **0.33343** |
| `c_cyl` | 3 | 55.417694 | 3 | 1 | 18.4998 | **0.33382** |
| `chair1` | 20 | 80.296902 | 20 | 1 | 52.1473 | **0.64943** |
| `blob` | 1 | 64.589785 | 1 | 0 | 64.1292 | **0.99287** |
| `wedge` | 6 | 37.333333 | 6 | 1 | 37.3333 | **1.00000** |
| `c_box` | 6 | 125.000000 | 6 | 1 | 125.0000 | **1.00000** |
| `ffres_s0` | 6 | 64.000000 | 6 | 1 | 64.0000 | **1.00000** |
| `boxoff` | 6 | 64.000000 | 6 | 1 | 64.0000 | **1.00000** |
| `box` | 6 | 64.000000 | 6 | 1 | 64.0000 | **1.00000** |
| `b_box` | 6 | 27.000000 | 6 | 1 | 27.0000 | **1.00000** |

## 6. Why the kernel's own numbers are not usable as evidence

On the control pair the kernel's self-reported volume disagrees with OCCT by up to 3.7e-4
relative, while the geometry it actually produced is accurate to 2.1e-5. Scoring the
kernel against a 1e-4 volume tolerance using its own printout would therefore have failed
the one case it gets right. That is why every verdict in this document is taken from OCCT
re-reading our exported STEP.

| quantity | kernel `volume()` | OCCT | rel err of the kernel's own reporter |
|---|---:|---:|---:|
| chair0 operand | 80.3011 | 80.296907 | 5.22e-05 |
| chair1 operand | 80.2988 | 80.296902 | 2.36e-05 |
| chairs cut result | 46.8114 | 46.794109 | 3.70e-04 |
| chairs common result | 33.4951 | 33.503026 | 2.37e-04 |
| chairs fuse result | 127.0950 | 127.091435 | 2.81e-05 |

The reporter is worse than that on curved geometry: see section 5, where `volume()` returns
exactly one third of the true value for every cylinder and cone, three times the true value
for a torus, and zero for a sphere. Any internal metric, gate, regression threshold or
`SESSION_AUTO` escalation decision that consumes `volume()` is reading a number that is
wrong by a factor of 3 as soon as a curved face is involved.

## 7. Two controlled experiments that localise the defect

**Experiment E -- analytic surfaces are not the cause.** The same five solid pairs were
exported twice: once carrying analytic STEP surfaces (`PLANE`, `CYLINDRICAL_SURFACE`,
`SPHERICAL_SURFACE`, `CONICAL_SURFACE`, `TOROIDAL_SURFACE`) and once with every face
converted to NURBS by `Shape.toNurbs()`. If the failures came from an untested analytic
import path, the NURBS variant would behave differently. It does not: `boxbox_analytic`
and `boxbox_nurbs` both return an open 12-naked-edge shell for all three operations, and
the other four cases fail in the same way in both variants. The defect is in the boolean,
not in the surface representation.

**Experiment F -- the box/box battery.** Eleven planar cases isolate what the boolean can
and cannot do. Full containment works exactly (`inside`: common and fuse match OCCT to the
last digit). Everything involving genuine face-face crossing fails, and it fails in two
distinct modes:

1. *Disjoint-miss* (`ax3`, `sc0.1`, `sc1`, `sc10`, `sc100`, `tet`): the kernel concludes
   the operands do not interfere. `cut` returns A unchanged, `fuse` returns A and B as two
   separate closed lumps, `common` returns nothing. The results are closed, OCCT-valid and
   have zero naked edges -- they are silently wrong. The scale sweep shows this is *not* a
   tolerance-versus-model-size problem: the identical relative error (1.296e-2 on cut,
   6.440e-3 on fuse) appears at 0.1x, 1x, 10x and 100x scale.
2. *Unstitched-patches* (`ax1`, `ax2`, `through`, `rot37`, and every primitive pair in
   corpus C): the section curves are found and the faces are split, but the pieces are
   never sewn. For `boxbox` the 12 naked edges are exactly the 12 original edges of box A,
   each of length 4.0 -- that is, the result is a loose bag of face patches with no shared
   topology at all.

## 8. Oracle quality: where OCCT itself cannot be used as truth

The corpus generator recorded OCCT's own partition identity per pair. Auditing it turns the
sweep into an accidental stress test of OCCT as well, and the pairs it fails cannot be used
to score us. Note one correction to a natural reading of the manifest: a `nan` residual is
usually **not** an OCCT defect -- it comes from an empty `cut`, which is the right answer
when A is contained in B. The genuine oracle defects are the gross residuals, the
self-declared invalid results, and the empty fuses.

### A. hard set (80 pairs)

- partition identity `cut+common == volA` holds to 1e-3 absolute on **36/80** pairs
- **13 pairs violate it grossly** (worst residuals below) -- these cannot be ground truth
- **11 op-results OCCT marks invalid itself**
- 31 ops return an empty CUT, which is the correct answer when A is contained in B (not an oracle defect, but it makes the residual undefined)
- **3 ops return an empty FUSE**, which is impossible for two interfering solids and is an outright OCCT failure
- **1 ops return a NEGATIVE volume** from OCCT: pair_019 cut = -0.027719 (occtref_z15_cut_s0 x occtref_z30x20_fuse)

| pair | A | B | residual | residual / volA | OCCT cut vol | OCCT common vol |
|---|---|---|---:|---:|---:|---:|
| pair_049 | occtref_x20_fuse | rotB_x13y29 | -154.8741 | 9.64e-01 | 5.672977 | 0.046248 |
| pair_004 | occtref_x13y29_fuse | occtref_x20_fuse | -121.9188 | 9.47e-01 | 6.810603 | 0.040466 |
| pair_056 | occtref_x20_fuse | rotB_z90 | -150.9581 | 9.40e-01 | 8.733127 | 0.902142 |
| pair_048 | occtref_x20_fuse | rotB_z63 | -131.0261 | 8.16e-01 | 29.53424 | 0.033017 |
| pair_047 | occtref_x20_fuse | rotB_z30x20 | -128.4876 | 8.00e-01 | 32.081889 | 0.023855 |
| pair_044 | occtref_x20_fuse | rotB_z30 | -79.4100 | 4.94e-01 | 81.183352 | 8e-06 |
| pair_046 | occtref_x20_fuse | rotB_z37 | -70.0956 | 4.36e-01 | 90.478249 | 0.019493 |
| pair_001 | occtref_x20_fuse | occtref_y30_fuse | -33.6359 | 2.09e-01 | 126.951299 | 0.006127 |
| pair_003 | occtref_x20_fuse | occtref_z30x20_fuse | -10.7685 | 6.71e-02 | 149.78696 | 0.037899 |
| pair_051 | occtref_y30_fuse | occtref_z30_fuse | -6.8086 | 5.35e-02 | 21.256267 | 99.19093 |
| pair_006 | occtref_z30_fuse | occtref_z45_fuse | -6.1876 | 4.54e-02 | 11.866566 | 118.177794 |
| pair_045 | occtref_x20_fuse | rotB_y30 | -6.7450 | 4.20e-02 | 153.842165 | 0.006127 |
| pair_038 | occtref_x13y29_fuse | occtref_y30_fuse | +0.0034 | 2.68e-05 | 16.801684 | 111.971653 |

Empty-fuse failures (OCCT returns 0 faces, 0 solids, and calls the result valid):

| pair | A | B | overlap frac | OCCT cut | OCCT common | OCCT fuse |
|---|---|---|---:|---|---|---|
| pair_046 | occtref_x20_fuse | rotB_z37 | 0.0002 | 90.478249 | 0.019493 | **0 faces / 0 solids** |
| pair_048 | occtref_x20_fuse | rotB_z63 | 0.0004 | 29.53424 | 0.033017 | **0 faces / 0 solids** |
| pair_051 | occtref_y30_fuse | occtref_z30_fuse | 0.7795 | 21.256267 | 99.19093 | **0 faces / 0 solids** |

### B. balanced set (60 pairs)

- partition identity `cut+common == volA` holds to 1e-3 absolute on **52/60** pairs
- **7 pairs violate it grossly** (worst residuals below) -- these cannot be ground truth
- **11 op-results OCCT marks invalid itself**
- 1 ops return an empty CUT, which is the correct answer when A is contained in B (not an oracle defect, but it makes the residual undefined)
- **6 ops return an empty FUSE**, which is impossible for two interfering solids and is an outright OCCT failure

| pair | A | B | residual | residual / volA | OCCT cut vol | OCCT common vol |
|---|---|---|---:|---:|---:|---:|
| bal_028 | occtref_x13y29_fuse | occtref_z30x20_fuse | -25.8947 | 2.01e-01 | 2.386836 | 100.48838 |
| bal_022 | occtref_x20_fuse | occtref_z30_cut_s0 | -19.4909 | 1.21e-01 | 129.74965 | 11.352773 |
| bal_011 | occtref_x20_fuse | occtref_z45_cut_s0 | -16.8241 | 1.05e-01 | 128.952949 | 14.816298 |
| bal_054 | occtref_x20_fuse | occtref_z30x20_cut | -15.5145 | 9.66e-02 | 136.103763 | 8.975023 |
| bal_005 | occtref_y30_fuse | occtref_z30_fuse | -6.8086 | 5.35e-02 | 21.256267 | 99.19093 |
| bal_029 | occtref_z30_fuse | rotB_y30 | +0.0041 | 3.00e-05 | 84.45987 | 51.77622 |
| bal_035 | occtref_x13y29_fuse | occtref_z90_fuse | -0.0011 | 8.69e-06 | 24.789065 | 103.979708 |

Empty-fuse failures (OCCT returns 0 faces, 0 solids, and calls the result valid):

| pair | A | B | overlap frac | OCCT cut | OCCT common | OCCT fuse |
|---|---|---|---:|---|---|---|
| bal_000 | occtref_x20_cut | occtref_x20_fuse | 0.3524 | 52.000501 | 28.295969 | **0 faces / 0 solids** |
| bal_005 | occtref_y30_fuse | occtref_z30_fuse | 0.7795 | 21.256267 | 99.19093 | **0 faces / 0 solids** |
| bal_011 | occtref_x20_fuse | occtref_z45_cut_s0 | 0.2507 | 128.952949 | 14.816298 | **0 faces / 0 solids** |
| bal_022 | occtref_x20_fuse | occtref_z30_cut_s0 | 0.2033 | 129.74965 | 11.352773 | **0 faces / 0 solids** |
| bal_028 | occtref_x13y29_fuse | occtref_z30x20_fuse | 0.7804 | 2.386836 | 100.48838 | **0 faces / 0 solids** |
| bal_054 | occtref_x20_fuse | occtref_z30x20_cut | 0.1654 | 136.103763 | 8.975023 | **0 faces / 0 solids** |

### C. supplementary set (102 pairs)

- partition identity `cut+common == volA` holds to 1e-3 absolute on **87/102** pairs
- **10 pairs violate it grossly** (worst residuals below) -- these cannot be ground truth
- **0 op-results OCCT marks invalid itself**
- 5 ops return an empty CUT, which is the correct answer when A is contained in B (not an oracle defect, but it makes the residual undefined)
- **0 ops return an empty FUSE**, which is impossible for two interfering solids and is an outright OCCT failure
- **1 ops return a NEGATIVE volume** from OCCT: spair_019 cut = -35.039230 (b_cyl x sph)
- **1 pairs report an overlap fraction > 1**, i.e. OCCT's `common` volume exceeds the smaller operand's own volume: spair_019 2.1522 (b_cyl x sph)

| pair | A | B | residual | residual / volA | OCCT cut vol | OCCT common vol |
|---|---|---|---:|---:|---:|---:|
| spair_040 | blob | sphoff | -0.4371 | 6.77e-03 | 31.688922 | 32.463803 |
| spair_037 | blob | cylx | -0.2605 | 4.03e-03 | 44.639806 | 19.689482 |
| spair_035 | blob | cone | -0.2596 | 4.02e-03 | 48.44135 | 15.888845 |
| spair_042 | blob | wedge | +0.2271 | 3.52e-03 | 28.67335 | 36.143492 |
| spair_033 | blob | box | -0.1743 | 2.70e-03 | 5.901263 | 58.514196 |
| spair_038 | blob | ffres_s0 | -0.1506 | 2.33e-03 | 5.913953 | 58.525194 |
| spair_034 | blob | boxoff | -0.1371 | 2.12e-03 | 39.808585 | 24.644076 |
| spair_041 | blob | tor | -0.0747 | 1.16e-03 | 44.058821 | 20.45624 |
| spair_036 | blob | cyl | -0.0552 | 8.54e-04 | 32.699439 | 31.83519 |
| spair_071 | c_sph | chair1 | -0.0011 | 1.49e-05 | 54.773581 | 18.847502 |

## 9. The ten hardest cases, with defect coordinates

Ranked by how badly a consumer would be misled, not by numeric size: a closed, valid,
wrong solid outranks an obviously broken one, and a hang outranks both when the reference
answers in seconds. All coordinates are in the operands' own model space and come from
OCCT reading our exported STEP.

**1. `pair_000 .. pair_079 (all 80) + bal_* — total hang on foreign boolean geometry` -- volume() does not terminate on foreign NURBS, hiding an open boolean result underneath**  
In the default configuration not one of the 140 pairs produced any output at all: 72 hard pairs capped at 240 s, 8 at 600 s, 4 probed at the full 900 s cap and 60 balanced pairs at 240 s, none of which even reached the operand-summary print. That print is the first call to BRep::volume(). Disabling only volume() with SESSION_FAST=1 shows the real picture: import and operand summary complete in 2 s and the cut finishes in 185 s, returning 75 faces, is_solid = 0 and 47 naked edges. So the severe verdict is twofold -- volume() does not terminate in 900 s on a 32-face OCCT-authored NURBS solid, and underneath it the boolean on foreign boolean-result geometry produces a badly open, non-solid result.  
`pair_063: import 2 s, cut 185 s, result 75 faces / is_solid 0 / 47 naked; volume() alone >900 s; OCCT does the same cut in 2.207-9.745 s (mean 4.625 s over 12 pairs)`  
Reproduce: `SESSION_NO_ROT=1 SESSION_OP=cut SESSION_CHAIRS=/home/petras/xpairs/pair_063 timeout 900 ./build/main_7_xp SKIPMATRIX`

**2. `dg/tet common` -- two tetrahedra: intersection returned as the union of both operands**  
Closed, OCCT-valid, zero naked edges, and 16x the correct volume. The kernel returned BOTH operands as the intersection (2 solids, 21.33333 = volA + volB) where the true common is a single 1.33333 solid. A consumer has no way to detect this: every validity check passes.  
`ours 8 faces / 2 solids / 21.33333; OCCT 4 faces / 1 solid / 1.33333; rel 1.50e+01`  
Reproduce: `SESSION_NO_ROT=1 SESSION_OP=common SESSION_CHAIRS=/home/petras/xp_work/dg/tet timeout 300 ./build/main_7_xp SKIPMATRIX`

**3. `dg/tet cut` -- two tetrahedra: difference returns operand A unchanged**  
Closed, OCCT-valid, zero naked edges, volume exactly volA. The subtraction did not happen at all, and nothing in the result says so.  
`ours 4 faces / 1 solid / 10.66667 (= volA); OCCT 6 faces / 1 solid / 9.33333; rel 1.43e-01`  
Reproduce: `SESSION_NO_ROT=1 SESSION_OP=cut SESSION_CHAIRS=/home/petras/xp_work/dg/tet timeout 300 ./build/main_7_xp SKIPMATRIX`

**4. `dg/ax3, dg/sc0.1, dg/sc1, dg/sc10, dg/sc100` -- corner-overlapping boxes declared disjoint, identically at four scales**  
Two axis-aligned 4x4x4 boxes overlapping in a 0.7 x 0.9 x 1.3 corner. cut returns A unchanged, fuse returns A and B as two separate closed lumps, common returns nothing. All closed, all OCCT-valid, all silently wrong. Scaling the whole configuration by 0.1, 1, 10 and 100 reproduces the SAME relative error to four digits, which rules out an absolute-tolerance-versus-model-size explanation.  
`cut ours 64.0 vs OCCT 63.181 (rel 1.296e-02) at every scale; fuse ours 128.0 vs 127.181 (rel 6.440e-03) at every scale; common ours empty vs OCCT 0.819`  
Reproduce: `SESSION_NO_ROT=1 SESSION_OP=cut SESSION_CHAIRS=/home/petras/xp_work/dg/ax3 timeout 300 ./build/main_7_xp SKIPMATRIX`

**5. `spair_009 / spair_024..032 / spair_070..071 — sphere ingestion` -- a sphere is not recognised as a solid, so it subtracts as nothing**  
Before any boolean, the kernel reads an OCCT-authored sphere (1 periodic face, 1 seam, 2 poles) as is_solid = 0 with volume 0.0000. Consequently `box cut sphere` returns the box unchanged and `sphere cut anything` returns an empty result. Four distinct spheres in the corpus behave identically. Every sphere pair in corpus C fails from this one cause.  
`sph: OCCT 65.449847, kernel is_solid=0 vol=0.0000; sphoff 44.602238 -> 0.0000; b_sph 28.730912 -> 0.0000; c_sph 73.622177 -> 0.0000`  
Reproduce: `SESSION_NO_ROT=1 SESSION_OP=cut SESSION_CHAIRS=/home/petras/xp_work/supp/spair_009 timeout 300 ./build/main_7_xp SKIPMATRIX`

**6. `cylinder / cone / torus volume reporter` -- volume() is off by exactly a factor of three on every analytic curved solid**  
Measured on operands the kernel has only imported, before any boolean. Four cylinders and one cone read as one third of their true volume; a torus reads as three times. Planar solids are exact. Any internal gate, sanity metric or escalation ladder that consumes volume() is being fed a number that is wrong by 3x the moment a curved face appears.  
`cyl 42.411501 -> 14.1403 (0.33341); cylx 27.143361 -> 9.0503 (0.33343); b_cyl 30.410617 -> 10.1389 (0.33340); c_cyl 55.417694 -> 18.4998 (0.33382); cone 16.755161 -> 5.5810 (0.33309); tor 25.266187 -> 75.7986 (3.00000)`  
Reproduce: `SESSION_NO_ROT=1 SESSION_OP=cut SESSION_CHAIRS=/home/petras/xp_work/supp/spair_006 timeout 300 ./build/main_7_xp SKIPMATRIX`

**7. `nb/boxbox_analytic and nb/boxbox_nurbs` -- two overlapping boxes come back as an unstitched bag of face patches**  
The simplest non-trivial boolean there is. All three operations return an open result whose naked edges are exactly the twelve original edges of box A, each of length 4.0 -- the faces were split but never sewn. Identical in the analytic-surface and the all-NURBS export of the same solids, so this is not an analytic import path defect.  
`cut ours 5 faces / 0 closed shells / 12 naked; OCCT 9 faces / 1 solid / 41.725`  
Defect locations (naked edges, longest first): len 4.0000 at (-2.000, 2.000, 0.000); len 4.0000 at (-2.000, 0.000, 2.000); len 4.0000 at (-2.000, -2.000, 0.000); len 4.0000 at (0.000, -2.000, -2.000)  
Reproduce: `SESSION_NO_ROT=1 SESSION_OP=cut SESSION_CHAIRS=/home/petras/xp_work/nb/boxbox_analytic timeout 300 ./build/main_7_xp SKIPMATRIX`

**8. `dg/inside cut` -- internal cavity is lost on export: one solid-with-void becomes two positive solids**  
The one case where the kernel's own topology is right (it reports 12 faces, is_solid, naked 0, volume 63.0000, which is correct) but the STEP it writes reads back as two separate positive closed solids of 64.0 and 1.0. The void nesting is not expressed, so any consumer of our STEP gets 65.0 instead of 63.0.  
`kernel volume() 63.0000 (correct); OCCT on our export 2 solids totalling 65.00000; OCCT truth 1 solid 63.00000`  
Reproduce: `SESSION_NO_ROT=1 SESSION_OP=cut SESSION_CHAIRS=/home/petras/xp_work/dg/inside timeout 300 ./build/main_7_xp SKIPMATRIX`

**9. `spair_002 / spair_033..042 — the freeform blob` -- single periodic BSpline face with seam and two poles: not a solid, results shred**  
The blob ingests as is_solid = 0 with volume 64.1292 against a true 64.589785 (0.71% low). Booleans against it return open results with many short naked edges clustered around the seam and pole region rather than a clean section curve.  
`b_box cut blob: ours 8 shells / 0 closed / 22 naked; OCCT 32 faces / 8 solids / 0.026038`  
Defect locations (naked edges, longest first): len 0.6521 at (-1.645, 1.259, 1.311); len 0.6503 at (-1.645, -1.260, -1.311); len 0.6217 at (1.128, -1.500, 1.495); len 0.6146 at (-1.500, 1.241, 1.500)  
Reproduce: `SESSION_NO_ROT=1 SESSION_OP=cut SESSION_CHAIRS=/home/petras/xp_work/supp/spair_002 timeout 300 ./build/main_7_xp SKIPMATRIX`

**10. `spair_050 box cut torus / spair_045 box cut cylinder` -- seam-carrying analytic surfaces leave the seam circles unattached**  
The naked edges land on the torus tube circles (length 8.79646 at y = +/-2.8) and on the cylinder's end circles (length 4.71239 at z = +/-3), i.e. exactly on the periodic seams and caps, together with box A's original edges. Note these lengths are far above the seam false-positive baseline: the shells are genuinely open (0 or 1 closed shell out of 3).  
`box cut tor: ours 3 faces / 1 of 3 shells closed / 12 naked / 25.2662; OCCT 7 faces / 1 solid / 48.518893. box cut cyl: ours 3 faces / 0 of 3 closed / 13 naked; OCCT 7 faces / 1 solid / 35.725666`  
Defect locations (naked edges, longest first): len 8.7965 at (0.000, -2.800, -0.000); len 8.7965 at (-0.000, 2.800, -0.000); len 4.0000 at (0.000, -2.000, -2.000); len 4.0000 at (2.000, 0.000, -2.000); len 6.0000 at (1.500, 0.000, 0.000); len 4.7124 at (0.000, -1.500, 3.000)  
Reproduce: `SESSION_NO_ROT=1 SESSION_OP=cut SESSION_CHAIRS=/home/petras/xp_work/supp/spair_050 timeout 300 ./build/main_7_xp SKIPMATRIX`

## 10. Crashes, hangs and timeouts

Every one of these is a hang against a reference kernel that answers the same question in
single-digit seconds, so each is recorded as a distinct and severe verdict, never folded
into the pass rate. Caveat on the 240 s cap: the known-good control pair (base chairs,
20+20 faces) itself needs 408-508 s per operation on this machine under the load present
during the sweep, so a 240 s timeout only establishes ">240 s". The meaningful
measurements are the 8 hard pairs run at 600 s and the 4 run at the full 900 s cap, all of
which also produced nothing. **Section 4b localises these hangs**: with `volume()` disabled
the same pairs load in 2 s and their booleans complete in 185-215 s, so what times out here
is `BRep::volume()` on the operands, not the boolean.

| corpus | op-runs that hit the wall cap | cap | example pairs |
|---|---:|---|---|
| A. hard cross-pairs pair_000..079 | 88 | 240s, 600s | pair_000:cut, pair_001:cut, pair_002:cut, pair_003:cut, pair_004:cut, pair_005:cut ... |
| B. balanced cross-pairs bal_000..059 | 39 | 240s | bal_000:cut, bal_001:cut, bal_004:cut, bal_005:cut, bal_008:cut, bal_009:cut ... |
| C. supplementary primitives spair_000..101 | 9 | 300s | spair_035:cut, spair_035:common, spair_035:fuse, spair_064:cut, spair_064:common, spair_073:fuse ... |

Segfaults / non-zero exits other than the timeout signal: **0**. C++ exceptions surfaced as `chairs <op> THREW`: **0**. No run exceeded the 4 GiB `ulimit -v` cap.

## 11. Per-pair results

Every "ours" figure is OCCT re-reading the STEP our kernel wrote, not the kernel's own
printout. Format: `faces/solids/naked-edges/validity/volume`. `naked` counts edges with
exactly one adjacent face; on results that contain a whole periodic surface this
over-counts seams (sphere 3, torus 2, cone 2, cylinder 1), so the closed-shell and
OCCT-valid columns, which are seam-immune, carry the verdict.

### control: base chairs (known-good)

| pair | A | B | cut | common | fuse |
|---|---|---|---|---|---|
| chairs | chair0 | chair1 | **PASS** ours 35f/1s/0naked/valid/46.7931 vs oracle 46.7941/35f/1s | **PASS** ours 25f/1s/0naked/valid/33.5034 vs oracle 33.5030/25f/1s | **PASS** ours 50f/1s/0naked/valid/127.0903 vs oracle 127.0914/50f/1s |

### F. box-box diagnostic battery dg

| pair | A | B | cut | common | fuse |
|---|---|---|---|---|---|
| ax1 | ax1_A | ax1_B | **FAIL** ours 2f/0s/8naked/INVALID/0.0000 vs oracle 52.8000/6f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/8naked/INVALID/0.0000 vs oracle 11.2000/6f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 8f/0s/12naked/INVALID/0.0000 vs oracle 116.8000/14f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| ax2 | ax2_A | ax2_B | **FAIL** ours 5f/0s/4naked/INVALID/0.0000 vs oracle 61.4800/8f/1s — open 1/1 shells;naked=4;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 1f/0s/4naked/INVALID/0.0000 vs oracle 2.5200/6f/1s — open 1/1 shells;naked=4;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 9f/0s/10naked/INVALID/0.0000 vs oracle 125.4800/14f/1s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| ax3 | ax3_A | ax3_B | **FAIL** ours 6f/1s/0naked/valid/64.0000 vs oracle 63.1810/9f/1s — vol rel=1.30e-02 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 0.8190/6f/1s — empty result, oracle has 1 solids | **FAIL** ours 12f/2s/0naked/valid/128.0000 vs oracle 127.1810/12f/1s — vol rel=6.44e-03;solids 2 vs 1 |
| inside | inside_A | inside_B | **FAIL** ours 12f/2s/0naked/valid/65.0000 vs oracle 63.0000/12f/1s — vol rel=3.17e-02;solids 2 vs 1 | **PASS** ours 6f/1s/0naked/valid/1.0000 vs oracle 1.0000/6f/1s | **PASS** ours 6f/1s/0naked/valid/64.0000 vs oracle 64.0000/6f/1s |
| rot37 | rot37_A | rot37_B | **FAIL** ours 4f/0s/10naked/INVALID/0.0000 vs oracle 29.4747/13f/1s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 8f/0s/10naked/INVALID/0.0000 vs oracle 34.5253/11f/1s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/10naked/INVALID/0.0000 vs oracle 93.4747/18f/1s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| sc0.1 | sc0.1_A | sc0.1_B | **FAIL** ours 6f/1s/0naked/valid/0.0640 vs oracle 0.0632/9f/1s — vol rel=1.30e-02 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 0.0008/6f/1s — empty result, oracle has 1 solids | **FAIL** ours 12f/2s/0naked/valid/0.1280 vs oracle 0.1272/12f/1s — vol rel=6.44e-03;solids 2 vs 1 |
| sc1 | sc1_A | sc1_B | **FAIL** ours 6f/1s/0naked/valid/64.0000 vs oracle 63.1810/9f/1s — vol rel=1.30e-02 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 0.8190/6f/1s — empty result, oracle has 1 solids | **FAIL** ours 12f/2s/0naked/valid/128.0000 vs oracle 127.1810/12f/1s — vol rel=6.44e-03;solids 2 vs 1 |
| sc10 | sc10_A | sc10_B | **FAIL** ours 6f/1s/0naked/valid/64000.0000 vs oracle 63181.0000/9f/1s — vol rel=1.30e-02 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 819.0000/6f/1s — empty result, oracle has 1 solids | **FAIL** ours 12f/2s/0naked/valid/128000.0000 vs oracle 127181.0000/12f/1s — vol rel=6.44e-03;solids 2 vs 1 |
| sc100 | sc100_A | sc100_B | **FAIL** ours 6f/1s/0naked/valid/64000000.0000 vs oracle 63181000.0000/9f/1s — vol rel=1.30e-02 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 819000.0000/6f/1s — empty result, oracle has 1 solids | **FAIL** ours 12f/2s/0naked/valid/128000000.0000 vs oracle 127181000.0000/12f/1s — vol rel=6.44e-03;solids 2 vs 1 |
| tet | tet_A | tet_B | **FAIL** ours 4f/1s/0naked/valid/10.6667 vs oracle 9.3333/6f/1s — vol rel=1.43e-01 | **FAIL** ours 8f/2s/0naked/valid/21.3333 vs oracle 1.3333/4f/1s — vol rel=1.50e+01;solids 2 vs 1 | **FAIL** ours 1f/0s/3naked/INVALID/0.0000 vs oracle 20.0000/11f/1s — open 1/1 shells;naked=3;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| through | through_A | through_B | **FAIL** ours 9f/0s/12naked/INVALID/0.0000 vs oracle 60.0000/10f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/0s/12naked/INVALID/0.0000 vs oracle 4.0000/6f/1s — open 3/3 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 5f/0s/12naked/INVALID/0.0000 vs oracle 69.0000/16f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |

### E. analytic-vs-NURBS control nb

| pair | A | B | cut | common | fuse |
|---|---|---|---|---|---|
| boxbox_analytic | boxboxA_analytic | boxboxB_analytic | **FAIL** ours 5f/0s/12naked/INVALID/0.0000 vs oracle 41.7250/9f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/0s/12naked/INVALID/0.0000 vs oracle 22.2750/6f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 5f/0s/12naked/INVALID/0.0000 vs oracle 105.7250/12f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| boxbox_nurbs | boxboxA_nurbs | boxboxB_nurbs | **FAIL** ours 6f/0s/12naked/INVALID/0.0000 vs oracle 41.7250/9f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/12naked/INVALID/0.0000 vs oracle 22.2750/6f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/12naked/INVALID/0.0000 vs oracle 105.7250/12f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| boxcone_analytic | boxconeA_analytic | boxconeB_analytic | **FAIL** ours 5f/0s/14naked/INVALID/0.0000 vs oracle 47.2448/10f/1s — open 2/2 shells;naked=14;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 4f/0s/12naked/INVALID/0.0000 vs oracle 16.7552/2f/1s — open 3/3 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/8naked/INVALID/0.0000 vs oracle 64.0000/10f/1s — open 1/1 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| boxcone_nurbs | boxconeA_nurbs | boxconeB_nurbs | **FAIL** ours 5f/0s/13naked/INVALID/0.0000 vs oracle 47.1132/10f/1s — open 2/2 shells;naked=13;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 3f/0s/13naked/INVALID/0.0000 vs oracle 16.9188/2f/1s — open 3/3 shells;naked=13;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 5f/0s/4naked/INVALID/0.0000 vs oracle 64.0000/10f/1s — open 1/1 shells;naked=4;OCCT-invalid [no trustworthy oracle] |
| boxcyl_analytic | boxcylA_analytic | boxcylB_analytic | **FAIL** ours 3f/0s/13naked/INVALID/0.0000 vs oracle 35.7257/7f/1s — open 3/3 shells;naked=13;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 5f/0s/13naked/INVALID/0.0000 vs oracle 28.2743/3f/1s — open 2/2 shells;naked=13;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/10naked/INVALID/0.0000 vs oracle 78.1372/10f/1s — open 3/3 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| boxcyl_nurbs | boxcylA_nurbs | boxcylB_nurbs | **FAIL** ours 3f/0s/13naked/INVALID/0.0000 vs oracle 35.4977/7f/1s — open 3/3 shells;naked=13;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 5f/0s/13naked/INVALID/0.0000 vs oracle 28.5328/3f/1s — open 2/2 shells;naked=13;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 4f/0s/12naked/INVALID/0.0000 vs oracle 78.2317/10f/1s — open 4/4 shells;naked=12;OCCT-invalid [no trustworthy oracle] |
| boxsph_analytic | boxsphA_analytic | boxsphB_analytic | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 32.6263/8f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 31.3737/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 77.2285/8f/1s — empty result, oracle has 1 solids |
| boxsph_nurbs | boxsphA_nurbs | boxsphB_nurbs | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 32.6331/8f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 31.3658/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 77.1219/8f/1s — empty result, oracle has 1 solids |
| boxtor_analytic | boxtorA_analytic | boxtorB_analytic | **FAIL** ours 3f/1s/12naked/INVALID/25.2662 vs oracle 48.5189/7f/1s — open 2/3 shells;naked=12;OCCT-invalid;vol rel=4.79e-01 | **FAIL** ours 5f/1s/12naked/INVALID/25.2662 vs oracle 15.4811/5f/1s — open 1/2 shells;naked=12;OCCT-invalid;vol rel=6.32e-01 | **FAIL** ours 2f/0s/8naked/INVALID/0.0000 vs oracle 73.7851/16f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| boxtor_nurbs | boxtorA_nurbs | boxtorB_nurbs | **FAIL** ours 3f/0s/10naked/INVALID/0.0000 vs oracle 48.5314/7f/1s — open 3/3 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 5f/0s/10naked/INVALID/0.0000 vs oracle 15.4677/5f/1s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 2f/0s/8naked/INVALID/0.0000 vs oracle 73.7856/16f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |

### C. supplementary primitives spair_000..101

| pair | A | B | cut | common | fuse |
|---|---|---|---|---|---|
| spair_000 | b_box | b_cyl | **FAIL** ours 2f/0s/9naked/INVALID/0.0000 vs oracle 15.5960/7f/1s — open 2/2 shells;naked=9;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/9naked/INVALID/0.0000 vs oracle 11.4040/3f/1s — open 2/2 shells;naked=9;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/8naked/INVALID/0.0000 vs oracle 46.0066/10f/1s — open 3/3 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_001 | b_box | b_sph | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 8.1725/11f/2s — empty result, oracle has 2 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 18.8275/6f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 36.9034/11f/1s — empty result, oracle has 1 solids |
| spair_002 | b_box | blob | **FAIL** ours 8f/0s/22naked/INVALID/0.0000 vs oracle 0.0260/32f/8s — open 8/8 shells;naked=22;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 8 | **FAIL** ours 7f/0s/19naked/INVALID/0.0000 vs oracle 26.9740/14f/1s — open 7/7 shells;naked=19;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 13f/0s/32naked/INVALID/0.0000 vs oracle 64.6063/25f/1s — open 2/2 shells;naked=32;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_003 | b_box | box | **FAIL** ours 1f/0s/4naked/INVALID/0.0000 vs oracle empty/0f/0s — open 1/1 shells;naked=4;OCCT-invalid;non-empty, oracle empty [no trustworthy oracle] | **FAIL** ours 7f/1s/4naked/INVALID/27.0000 vs oracle 27.0000/6f/1s — open 1/2 shells;naked=4;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 4f/0s/6naked/INVALID/0.0000 vs oracle 64.0000/6f/1s — open 1/1 shells;naked=6;OCCT-invalid [no trustworthy oracle] |
| spair_004 | b_box | boxoff | **FAIL** ours 5f/0s/14naked/INVALID/0.0000 vs oracle 15.4962/9f/1s — open 2/2 shells;naked=14;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/0s/14naked/INVALID/0.0000 vs oracle 11.5038/6f/1s — open 2/2 shells;naked=14;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 4f/0s/12naked/INVALID/0.0000 vs oracle 79.4962/12f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_005 | b_box | cone | **FAIL** ours 4f/0s/11naked/INVALID/0.0000 vs oracle 16.1171/10f/1s — open 3/3 shells;naked=11;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/11naked/INVALID/0.0000 vs oracle 10.8829/7f/1s — open 3/3 shells;naked=11;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/8naked/INVALID/0.0000 vs oracle 32.8723/12f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_006 | b_box | cyl | **FAIL** ours 2f/0s/9naked/INVALID/0.0000 vs oracle 6.1720/12f/2s — open 2/2 shells;naked=9;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 6f/0s/9naked/INVALID/0.0000 vs oracle 20.8280/6f/1s — open 2/2 shells;naked=9;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/8naked/INVALID/0.0000 vs oracle 48.5835/13f/1s — open 3/3 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_007 | b_box | cylx | **FAIL** ours 3f/0s/7naked/INVALID/0.0000 vs oracle 13.6228/12f/2s — open 2/2 shells;naked=7;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 7f/0s/7naked/INVALID/0.0000 vs oracle 13.3772/6f/1s — open 2/2 shells;naked=7;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/10naked/INVALID/0.0000 vs oracle 40.7661/13f/1s — open 3/3 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_008 | b_box | ffres_s0 | **FAIL** ours 1f/0s/4naked/INVALID/0.0000 vs oracle empty/0f/0s — open 1/1 shells;naked=4;OCCT-invalid;non-empty, oracle empty [no trustworthy oracle] | **FAIL** ours 7f/1s/4naked/INVALID/27.0000 vs oracle 27.0000/6f/1s — open 1/2 shells;naked=4;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 4f/0s/6naked/INVALID/0.0000 vs oracle 64.0000/6f/1s — open 1/1 shells;naked=6;OCCT-invalid [no trustworthy oracle] |
| spair_009 | b_box | sph | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 0.0233/32f/8s — empty result, oracle has 8 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 26.9767/14f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 65.4731/25f/1s — empty result, oracle has 1 solids |
| spair_010 | b_box | sphoff | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 9.4070/8f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 17.5931/6f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 54.0092/8f/1s — empty result, oracle has 1 solids |
| spair_011 | b_box | tor | **FAIL** ours 3f/1s/12naked/INVALID/25.2662 vs oracle 21.9070/11f/1s — open 2/3 shells;naked=12;OCCT-invalid;vol rel=1.53e-01 | **FAIL** ours 5f/1s/12naked/INVALID/25.2662 vs oracle 5.0930/5f/1s — open 1/2 shells;naked=12;OCCT-invalid;vol rel=3.96e+00 | **FAIL** ours 2f/0s/8naked/INVALID/0.0000 vs oracle 47.1732/12f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_012 | b_box | wedge | **FAIL** ours 4f/0s/12naked/INVALID/0.0000 vs oracle 4.1304/9f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/12naked/INVALID/0.0000 vs oracle 22.8696/9f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/12naked/INVALID/0.0000 vs oracle 41.4638/13f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_013 | b_cyl | b_sph | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 18.6558/6f/2s — empty result, oracle has 2 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 11.7548/3f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 47.3868/5f/1s — empty result, oracle has 1 solids |
| spair_014 | b_cyl | box | **FAIL** ours 5f/0s/4naked/INVALID/0.0000 vs oracle 15.2053/6f/2s — open 1/1 shells;naked=4;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 8f/1s/5naked/INVALID/30.4106 vs oracle 15.2053/3f/1s — open 1/2 shells;naked=5;OCCT-invalid;vol rel=1.00e+00 | **FAIL** ours 1f/0s/4naked/INVALID/0.0000 vs oracle 79.2053/10f/1s — open 1/1 shells;naked=4;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_015 | b_cyl | boxoff | **FAIL** ours 4f/0s/9naked/INVALID/0.0000 vs oracle 18.1413/6f/1s — open 2/2 shells;naked=9;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/8naked/INVALID/0.0000 vs oracle 12.2693/5f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/9naked/INVALID/0.0000 vs oracle 82.1413/10f/1s — open 2/2 shells;naked=9;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_016 | b_cyl | cyl | **FAIL** ours 3f/1s/1naked/valid/42.4115 vs oracle 7.6027/6f/2s — naked=1;vol rel=4.58e+00;solids 1 vs 2 | **FAIL** ours 6f/2s/2naked/valid/72.8221 vs oracle 22.8080/3f/1s — naked=2;vol rel=2.19e+00;solids 2 vs 1 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 50.0142/7f/1s — empty result, oracle has 1 solids |
| spair_017 | b_cyl | cylx | **FAIL** ours 3f/0s/5naked/INVALID/0.0000 vs oracle 23.2928/5f/1s — open 2/2 shells;naked=5;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 4f/0s/6naked/INVALID/0.0000 vs oracle 7.1179/4f/1s — open 2/2 shells;naked=6;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 2f/0s/4naked/INVALID/0.0000 vs oracle 50.4362/6f/1s — open 2/2 shells;naked=4;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_018 | b_cyl | ffres_s0 | **FAIL** ours 6f/0s/6naked/INVALID/0.0000 vs oracle 15.2053/6f/2s — open 2/2 shells;naked=6;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 7f/0s/7naked/INVALID/0.0000 vs oracle 15.2053/3f/1s — open 2/2 shells;naked=7;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 2f/0s/6naked/INVALID/0.0000 vs oracle 79.2053/10f/1s — open 2/2 shells;naked=6;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_019 | b_cyl | sph | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle -35.0392/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 65.4498/1f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 30.4106/3f/1s — empty result, oracle has 1 solids |
| spair_020 | b_cyl | sphoff | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 19.0250/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 11.3857/3f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 63.6271/4f/1s — empty result, oracle has 1 solids |
| spair_021 | b_cyl | tor | **FAIL** ours 3f/1s/8naked/INVALID/25.2662 vs oracle 30.3937/4f/1s — open 2/3 shells;naked=8;OCCT-invalid;vol rel=1.69e-01 | **FAIL** ours 2f/1s/9naked/INVALID/25.2662 vs oracle 0.0174/2f/1s — open 1/2 shells;naked=9;OCCT-invalid;vol rel=1.45e+03 | **FAIL** ours 2f/0s/4naked/INVALID/0.0000 vs oracle 55.6592/4f/1s — open 2/2 shells;naked=4;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_022 | b_cyl | wedge | **FAIL** ours 7f/0s/8naked/INVALID/0.0000 vs oracle 19.0060/6f/2s — open 3/3 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 6f/0s/9naked/INVALID/0.0000 vs oracle 11.4046/3f/1s — open 2/2 shells;naked=9;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 2f/0s/6naked/INVALID/0.0000 vs oracle 56.3394/10f/1s — open 2/2 shells;naked=6;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_023 | b_sph | blob | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 0.3246/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 28.4069/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 64.9396/2f/1s — empty result, oracle has 1 solids |
| spair_024 | b_sph | box | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 1.2708/7f/3s — empty result, oracle has 3 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 27.4601/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 65.2708/10f/1s — empty result, oracle has 1 solids |
| spair_025 | b_sph | boxoff | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 9.4536/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 19.2774/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 73.4536/8f/1s — empty result, oracle has 1 solids |
| spair_026 | b_sph | cone | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 19.7608/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 8.9701/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 1f/0s/3naked/INVALID/0.0000 vs oracle 36.5160/3f/1s — open 1/1 shells;naked=3;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_027 | b_sph | cyl | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 9.3251/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 19.4067/3f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 51.7373/5f/1s — empty result, oracle has 1 solids |
| spair_028 | b_sph | ffres_s0 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 1.2708/7f/3s — empty result, oracle has 3 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 27.4601/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 65.2708/10f/1s — empty result, oracle has 1 solids |
| spair_029 | b_sph | sph | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 0.0330/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 28.6980/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 65.4828/2f/1s — empty result, oracle has 1 solids |
| spair_030 | b_sph | sphoff | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 1.8502/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 26.8807/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 46.4524/2f/1s — empty result, oracle has 1 solids |
| spair_031 | b_sph | tor | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 20.7686/3f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 7.9623/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 46.0348/4f/1s — empty result, oracle has 1 solids |
| spair_032 | b_sph | wedge | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 11.7703/6f/2s — empty result, oracle has 2 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 16.9606/5f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 49.1037/8f/1s — empty result, oracle has 1 solids |
| spair_033 | blob | box | **FAIL** ours 2f/0s/4naked/INVALID/0.0000 vs oracle 5.9013/9f/4s — open 2/2 shells;naked=4;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 58.5142/5f/1s — empty result, oracle has 1 solids | **FAIL** ours 8f/1s/4naked/INVALID/64.0000 vs oracle 69.9365/11f/1s — open 2/3 shells;naked=4;OCCT-invalid [no trustworthy oracle] |
| spair_034 | blob | boxoff | **FAIL** ours 6f/0s/4naked/INVALID/0.0000 vs oracle 39.8086/4f/1s — open 1/1 shells;naked=4;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 24.6441/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 12f/1s/4naked/INVALID/64.0000 vs oracle 103.6353/7f/1s — open 1/2 shells;naked=4;OCCT-invalid [no trustworthy oracle] |
| spair_035 | blob | cone | **TIMEOUT** (oracle 48.4413/4f/2s) | **TIMEOUT** (oracle 15.8888/3f/1s) | **TIMEOUT** (oracle 65.2777/4f/1s) |
| spair_036 | blob | cyl | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 32.6994/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 31.8352/3f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 75.1182/5f/1s — empty result, oracle has 1 solids |
| spair_037 | blob | cylx | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 44.6398/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 19.6895/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 71.7836/5f/1s — empty result, oracle has 1 solids |
| spair_038 | blob | ffres_s0 | **FAIL** ours 2f/0s/4naked/INVALID/0.0000 vs oracle 5.9140/9f/4s — open 2/2 shells;naked=4;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 58.5252/5f/1s — empty result, oracle has 1 solids | **FAIL** ours 8f/1s/4naked/INVALID/64.0000 vs oracle 70.0034/11f/1s — open 2/3 shells;naked=4;OCCT-invalid [no trustworthy oracle] |
| spair_039 | blob | sph | **PASS** ours 0f/0s/0naked/INVALID/0.0000 vs oracle empty/0f/0s — both empty | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 64.5898/1f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 65.4401/1f/1s — empty result, oracle has 1 solids |
| spair_040 | blob | sphoff | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 31.6889/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 32.4638/3f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 76.3067/2f/1s — empty result, oracle has 1 solids |
| spair_041 | blob | tor | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 44.0588/3f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 20.4562/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 69.3387/4f/1s — empty result, oracle has 1 solids |
| spair_042 | blob | wedge | **FAIL** ours 1f/0s/2naked/INVALID/0.0000 vs oracle 28.6733/6f/1s — open 1/1 shells;naked=2;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 36.1435/7f/1s — empty result, oracle has 1 solids | **FAIL** ours 16f/1s/10naked/INVALID/37.3333 vs oracle 66.0131/10f/1s — open 2/3 shells;naked=10;OCCT-invalid [no trustworthy oracle] |
| spair_043 | box | boxoff | **FAIL** ours 5f/0s/12naked/INVALID/0.0000 vs oracle 41.7250/9f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/0s/12naked/INVALID/0.0000 vs oracle 22.2750/6f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 5f/0s/12naked/INVALID/0.0000 vs oracle 105.7250/12f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_044 | box | cone | **FAIL** ours 5f/0s/14naked/INVALID/0.0000 vs oracle 47.2448/10f/1s — open 2/2 shells;naked=14;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 4f/0s/12naked/INVALID/0.0000 vs oracle 16.7552/2f/1s — open 3/3 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/8naked/INVALID/0.0000 vs oracle 64.0000/10f/1s — open 1/1 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_045 | box | cyl | **FAIL** ours 3f/0s/13naked/INVALID/0.0000 vs oracle 35.7257/7f/1s — open 3/3 shells;naked=13;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 5f/0s/13naked/INVALID/0.0000 vs oracle 28.2743/3f/1s — open 2/2 shells;naked=13;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/10naked/INVALID/0.0000 vs oracle 78.1372/10f/1s — open 3/3 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_046 | box | cylx | **FAIL** ours 3f/0s/13naked/INVALID/0.0000 vs oracle 45.9044/7f/1s — open 3/3 shells;naked=13;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 5f/0s/13naked/INVALID/0.0000 vs oracle 18.0956/3f/1s — open 2/2 shells;naked=13;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/10naked/INVALID/0.0000 vs oracle 73.0478/10f/1s — open 3/3 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_047 | box | ffres_s0 | **PASS** ours 0f/0s/0naked/INVALID/0.0000 vs oracle empty/0f/0s — both empty | **PASS-CLOSURE** ours 6f/1s/0naked/valid/64.0000 vs oracle 64.0000/6f/1s — closed+valid; oracle untrustworthy so volume not scored | **PASS-CLOSURE** ours 6f/1s/0naked/valid/64.0000 vs oracle 64.0000/6f/1s — closed+valid; oracle untrustworthy so volume not scored |
| spair_048 | box | sph | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 9.5457/7f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 54.4543/7f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 74.9956/13f/1s — empty result, oracle has 1 solids |
| spair_049 | box | sphoff | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 32.6263/8f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 31.3737/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 77.2285/8f/1s — empty result, oracle has 1 solids |
| spair_050 | box | tor | **FAIL** ours 3f/1s/12naked/INVALID/25.2662 vs oracle 48.5189/7f/1s — open 2/3 shells;naked=12;OCCT-invalid;vol rel=4.79e-01 | **FAIL** ours 5f/1s/12naked/INVALID/25.2662 vs oracle 15.4811/5f/1s — open 1/2 shells;naked=12;OCCT-invalid;vol rel=6.32e-01 | **FAIL** ours 2f/0s/8naked/INVALID/0.0000 vs oracle 73.7851/16f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_051 | box | wedge | **FAIL** ours 8f/0s/8naked/INVALID/0.0000 vs oracle 26.6667/9f/1s — open 1/1 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/8naked/INVALID/0.0000 vs oracle 37.3333/6f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **PASS** ours 6f/1s/0naked/valid/64.0000 vs oracle 64.0000/7f/1s |
| spair_052 | boxoff | chair0 | **FAIL** ours 6f/1s/0naked/valid/64.0000 vs oracle 63.9230/11f/1s — vol rel=1.21e-03 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 0.0770/6f/1s — empty result, oracle has 1 solids | **FAIL** ours 26f/2s/0naked/valid/144.2966 vs oracle 144.2197/25f/1s — vol rel=5.33e-04;solids 2 vs 1 |
| spair_053 | boxoff | cone | **FAIL** ours 8f/1s/5naked/INVALID/64.0000 vs oracle 60.2431/7f/1s — open 2/3 shells;naked=5;OCCT-invalid;vol rel=6.24e-02 | **FAIL** ours 2f/0s/5naked/INVALID/0.0000 vs oracle 3.7569/3f/1s — open 2/2 shells;naked=5;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/1s/2naked/INVALID/64.0000 vs oracle 76.9983/8f/1s — open 1/2 shells;naked=2;OCCT-invalid;vol rel=1.69e-01 |
| spair_054 | boxoff | cyl | **FAIL** ours 3f/0s/8naked/INVALID/0.0000 vs oracle 45.2623/9f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 5f/0s/8naked/INVALID/0.0000 vs oracle 18.7377/6f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 4f/0s/9naked/INVALID/0.0000 vs oracle 87.6738/9f/1s — open 2/2 shells;naked=9;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_055 | boxoff | cylx | **FAIL** ours 5f/0s/8naked/INVALID/0.0000 vs oracle 49.0953/9f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/8naked/INVALID/0.0000 vs oracle 14.9047/5f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/9naked/INVALID/0.0000 vs oracle 76.2387/9f/1s — open 2/2 shells;naked=9;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_056 | boxoff | ffres_s0 | **FAIL** ours 7f/0s/12naked/INVALID/0.0000 vs oracle 41.7250/9f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/0s/12naked/INVALID/0.0000 vs oracle 22.2750/6f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 5f/0s/12naked/INVALID/0.0000 vs oracle 105.7250/12f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_057 | boxoff | sph | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 39.2590/7f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 24.7410/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 104.7088/7f/1s — empty result, oracle has 1 solids |
| spair_058 | boxoff | sphoff | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 32.1612/8f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 31.8388/5f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 76.7634/9f/1s — empty result, oracle has 1 solids |
| spair_059 | boxoff | tor | **FAIL** ours 5f/0s/4naked/INVALID/0.0000 vs oracle 54.6475/13f/1s — open 1/1 shells;naked=4;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 1f/0s/4naked/INVALID/0.0000 vs oracle 9.3525/10f/1s — open 1/1 shells;naked=4;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/1s/8naked/INVALID/25.2662 vs oracle 79.9137/10f/1s — open 1/2 shells;naked=8;OCCT-invalid;vol rel=6.84e-01 |
| spair_060 | boxoff | wedge | **FAIL** ours 9f/0s/10naked/INVALID/0.0000 vs oracle 53.8633/9f/1s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 5f/0s/10naked/INVALID/0.0000 vs oracle 10.1367/6f/1s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/8naked/INVALID/0.0000 vs oracle 91.1967/12f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_061 | c_box | c_cyl | **FAIL** ours 2f/0s/9naked/INVALID/0.0000 vs oracle 94.2124/7f/1s — open 2/2 shells;naked=9;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/9naked/INVALID/0.0000 vs oracle 30.7876/3f/1s — open 2/2 shells;naked=9;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/8naked/INVALID/0.0000 vs oracle 149.6301/10f/1s — open 3/3 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_062 | c_box | c_sph | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 51.8616/7f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 73.1384/7f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 125.4838/13f/1s — empty result, oracle has 1 solids |
| spair_063 | c_box | c_tor | **FAIL** ours 3f/1s/12naked/INVALID/38.3730 vs oracle 98.2168/7f/1s — open 2/3 shells;naked=12;OCCT-invalid;vol rel=6.09e-01 | **FAIL** ours 5f/1s/12naked/INVALID/38.3730 vs oracle 26.7832/5f/1s — open 1/2 shells;naked=12;OCCT-invalid;vol rel=4.33e-01 | **FAIL** ours 2f/0s/8naked/INVALID/0.0000 vs oracle 136.5898/16f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_064 | c_box | chair0 | **TIMEOUT** (oracle 87.6421/26f/2s) | **TIMEOUT** (oracle 37.3571/21f/1s) | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 167.9389/34f/1s — empty result, oracle has 1 solids |
| spair_065 | c_box | chair1 | **FAIL** ours 6f/1s/0naked/valid/125.0000 vs oracle 96.2854/11f/1s — vol rel=2.98e-01 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 28.7144/8f/1s — empty result, oracle has 1 solids | **FAIL** ours 26f/2s/0naked/valid/205.2966 vs oracle 176.5822/28f/1s — vol rel=1.63e-01;solids 2 vs 1 |
| spair_066 | c_cyl | c_sph | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 25.8460/6f/2s — empty result, oracle has 2 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 29.5717/3f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 99.4682/5f/1s — empty result, oracle has 1 solids |
| spair_067 | c_cyl | chair0 | **FAIL** ours 3f/1s/1naked/valid/55.4177 vs oracle 43.7119/11f/1s — naked=1;vol rel=2.68e-01 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 11.7056/12f/2s — empty result, oracle has 2 solids | **FAIL** ours 23f/2s/1naked/valid/135.7142 vs oracle 124.0088/26f/1s — naked=1;vol rel=9.44e-02;solids 2 vs 1 |
| spair_068 | c_cyl | chair1 | **FAIL** ours 3f/1s/1naked/valid/55.4177 vs oracle 39.0335/9f/2s — naked=1;vol rel=4.20e-01;solids 1 vs 2 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 16.3842/6f/1s — empty result, oracle has 1 solids | **FAIL** ours 23f/2s/1naked/valid/135.7143 vs oracle 119.3305/26f/1s — naked=1;vol rel=1.37e-01;solids 2 vs 1 |
| spair_069 | c_sph | c_tor | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 53.3616/3f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 20.2606/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 91.7346/4f/1s — empty result, oracle has 1 solids |
| spair_070 | c_sph | chair0 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 49.3378/12f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 24.2845/14f/1s — empty result, oracle has 1 solids | **NOVERIFY** (oracle 129.6351/24f/1s) |
| spair_071 | c_sph | chair1 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 54.7736/5f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 18.8475/6f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 135.0706/22f/1s — empty result, oracle has 1 solids |
| spair_072 | c_tor | chair0 | **FAIL** ours 6f/1s/20naked/INVALID/38.3730 vs oracle 25.8196/16f/1s — open 3/4 shells;naked=20;OCCT-invalid;vol rel=4.86e-01 | **FAIL** ours 5f/0s/16naked/INVALID/0.0000 vs oracle 12.5530/21f/3s — open 3/3 shells;naked=16;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 3 | **FAIL** ours 16f/1s/20naked/INVALID/38.3730 vs oracle 106.1166/28f/1s — open 1/2 shells;naked=20;OCCT-invalid;vol rel=6.38e-01 |
| spair_073 | c_tor | chair1 | **FAIL** ours 1f/1s/4naked/valid/38.3730 vs oracle 33.5299/7f/1s — naked=4;vol rel=1.44e-01 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 4.8426/8f/1s — empty result, oracle has 1 solids | **TIMEOUT** (oracle 113.8277/25f/1s) |
| spair_074 | chair0 | chair1 | **TIMEOUT** (oracle 46.7941/35f/1s) | **TIMEOUT** (oracle 33.5030/25f/1s) | **TIMEOUT** (oracle 127.0914/50f/1s) |
| spair_075 | cone | cyl | **FAIL** ours 3f/1s/1naked/valid/42.4115 vs oracle 2.6180/3f/1s — naked=1;vol rel=1.52e+01 | **FAIL** ours 6f/2s/4naked/INVALID/42.3913 vs oracle 14.1372/3f/1s — open 1/3 shells;naked=4;OCCT-invalid;vol rel=2.00e+00;solids 2 vs 1 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 45.0295/6f/1s — empty result, oracle has 1 solids |
| spair_076 | cone | cylx | **FAIL** ours 2f/0s/7naked/INVALID/0.0000 vs oracle 11.8901/5f/2s — open 2/2 shells;naked=7;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 3f/0s/10naked/INVALID/0.0000 vs oracle 4.8650/3f/1s — open 3/3 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/6naked/INVALID/0.0000 vs oracle 39.0335/6f/1s — open 3/3 shells;naked=6;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_077 | cone | ffres_s0 | **FAIL** ours 2f/0s/6naked/INVALID/0.0000 vs oracle empty/0f/0s — open 1/1 shells;naked=6;OCCT-invalid;non-empty, oracle empty [no trustworthy oracle] | **FAIL** ours 5f/1s/10naked/INVALID/-0.0294 vs oracle 16.7552/2f/1s — open 2/3 shells;naked=10;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 4f/0s/11naked/INVALID/0.0000 vs oracle 64.0000/10f/1s — open 2/2 shells;naked=11;OCCT-invalid [no trustworthy oracle] |
| spair_078 | cone | sph | **FAIL** ours 1f/0s/3naked/INVALID/0.0000 vs oracle 0.8376/3f/1s — open 1/1 shells;naked=3;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 15.9175/3f/1s — empty result, oracle has 1 solids | **FAIL** ours 1f/0s/3naked/INVALID/0.0000 vs oracle 66.2875/4f/1s — open 1/1 shells;naked=3;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_079 | cone | sphoff | **FAIL** ours 1f/0s/3naked/INVALID/0.0000 vs oracle 7.5037/3f/1s — open 1/1 shells;naked=3;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 9.2515/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 1f/0s/3naked/INVALID/0.0000 vs oracle 52.1059/3f/1s — open 1/1 shells;naked=3;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_080 | cone | wedge | **FAIL** ours 4f/0s/10naked/INVALID/0.0000 vs oracle 3.6922/8f/2s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 4f/0s/15naked/INVALID/0.0000 vs oracle 13.0629/5f/1s — open 2/2 shells;naked=15;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 4f/0s/10naked/INVALID/0.0000 vs oracle 41.0256/12f/1s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_081 | cyl | cylx | **FAIL** ours 3f/0s/9naked/INVALID/0.0000 vs oracle 30.8269/5f/1s — open 3/3 shells;naked=9;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 2f/0s/10naked/INVALID/0.0000 vs oracle 11.5846/4f/1s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 4f/0s/8naked/INVALID/0.0000 vs oracle 57.9718/6f/1s — open 4/4 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_082 | cyl | ffres_s0 | **FAIL** ours 6f/0s/12naked/INVALID/0.0000 vs oracle 14.1372/6f/2s — open 3/3 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 5f/0s/13naked/INVALID/0.0000 vs oracle 28.2743/3f/1s — open 2/2 shells;naked=13;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 4f/0s/12naked/INVALID/0.0000 vs oracle 78.1372/10f/1s — open 4/4 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_083 | cyl | sph | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 10.4720/6f/2s — empty result, oracle has 2 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 31.9395/3f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 75.9218/5f/1s — empty result, oracle has 1 solids |
| spair_084 | cyl | sphoff | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 22.6526/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 19.7590/3f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 67.2549/5f/1s — empty result, oracle has 1 solids |
| spair_085 | cyl | tor | **FAIL** ours 3f/1s/8naked/INVALID/25.2662 vs oracle 40.1522/5f/1s — open 2/3 shells;naked=8;OCCT-invalid;vol rel=3.71e-01 | **FAIL** ours 2f/1s/9naked/INVALID/25.2662 vs oracle 2.2593/2f/1s — open 1/2 shells;naked=9;OCCT-invalid;vol rel=1.02e+01 | **FAIL** ours 2f/0s/4naked/INVALID/0.0000 vs oracle 65.4184/6f/1s — open 2/2 shells;naked=4;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_086 | cyl | wedge | **FAIL** ours 6f/0s/10naked/INVALID/0.0000 vs oracle 22.3759/6f/1s — open 3/3 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 5f/0s/11naked/INVALID/0.0000 vs oracle 20.0356/4f/1s — open 2/2 shells;naked=11;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 4f/0s/10naked/INVALID/0.0000 vs oracle 59.7093/10f/1s — open 3/3 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_087 | cylx | ffres_s0 | **FAIL** ours 6f/0s/12naked/INVALID/0.0000 vs oracle 9.0478/6f/2s — open 3/3 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 5f/0s/13naked/INVALID/0.0000 vs oracle 18.0956/3f/1s — open 2/2 shells;naked=13;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/10naked/INVALID/0.0000 vs oracle 73.0478/10f/1s — open 3/3 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_088 | cylx | sph | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 6.4059/7f/2s — empty result, oracle has 2 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 20.7374/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 71.8558/5f/1s — empty result, oracle has 1 solids |
| spair_089 | cylx | sphoff | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 9.1685/7f/2s — empty result, oracle has 2 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 17.9749/5f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 53.7706/7f/1s — empty result, oracle has 1 solids |
| spair_090 | cylx | tor | **FAIL** ours 4f/2s/5naked/valid/52.4095 vs oracle 17.2954/5f/1s — naked=5;vol rel=2.03e+00;solids 2 vs 1 | **FAIL** ours 1f/1s/4naked/valid/25.2662 vs oracle 9.8477/5f/1s — naked=4;vol rel=1.57e+00 | **FAIL** ours 3f/1s/1naked/valid/27.1434 vs oracle 42.5616/6f/1s — naked=1;vol rel=3.62e-01 |
| spair_091 | cylx | wedge | **FAIL** ours 7f/0s/8naked/INVALID/0.0000 vs oracle 16.0291/6f/1s — open 3/3 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/9naked/INVALID/0.0000 vs oracle 11.1143/4f/1s — open 2/2 shells;naked=9;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/8naked/INVALID/0.0000 vs oracle 53.3624/10f/1s — open 3/3 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_092 | ffres_s0 | sph | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 9.5457/7f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 54.4543/7f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 74.9956/13f/1s — empty result, oracle has 1 solids |
| spair_093 | ffres_s0 | sphoff | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 32.6263/8f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 31.3737/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 77.2285/8f/1s — empty result, oracle has 1 solids |
| spair_094 | ffres_s0 | tor | **FAIL** ours 3f/1s/12naked/INVALID/25.2662 vs oracle 48.5189/7f/1s — open 2/3 shells;naked=12;OCCT-invalid;vol rel=4.79e-01 | **FAIL** ours 5f/1s/12naked/INVALID/25.2662 vs oracle 15.4811/5f/1s — open 1/2 shells;naked=12;OCCT-invalid;vol rel=6.32e-01 | **FAIL** ours 2f/0s/8naked/INVALID/0.0000 vs oracle 73.7851/16f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| spair_095 | ffres_s0 | wedge | **FAIL** ours 8f/0s/8naked/INVALID/0.0000 vs oracle 26.6667/9f/1s — open 1/1 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/8naked/INVALID/0.0000 vs oracle 37.3333/6f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **PASS** ours 6f/1s/0naked/valid/64.0000 vs oracle 64.0000/7f/1s |
| spair_096 | sph | sphoff | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 32.2400/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 33.2098/3f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 76.8423/2f/1s — empty result, oracle has 1 solids |
| spair_097 | sph | tor | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 45.0797/3f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 20.3701/2f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 70.3459/4f/1s — empty result, oracle has 1 solids |
| spair_098 | sph | wedge | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 31.2499/8f/2s — empty result, oracle has 2 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 34.2000/10f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 68.5832/16f/1s — empty result, oracle has 1 solids |
| spair_099 | sphoff | tor | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 33.6493/3f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 10.9532/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 58.9154/2f/1s — empty result, oracle has 1 solids |
| spair_100 | sphoff | wedge | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 28.1065/5f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 16.4957/5f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 65.4399/8f/1s — empty result, oracle has 1 solids |
| spair_101 | tor | wedge | **FAIL** ours 4f/0s/8naked/INVALID/0.0000 vs oracle 15.6606/11f/2s — open 1/1 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 5f/1s/12naked/INVALID/25.2662 vs oracle 9.6056/8f/1s — open 1/2 shells;naked=12;OCCT-invalid;vol rel=1.63e+00 | **FAIL** ours 2f/0s/8naked/INVALID/0.0000 vs oracle 52.9939/11f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |

### D. provenance set pvpair_000..056

| pair | A | B | cut | common | fuse |
|---|---|---|---|---|---|
| pvpair_000 | P_boxoff | R_box_com_sph | **FAIL** ours 6f/0s/14naked/INVALID/0.0000 vs oracle 20.0783/13f/1s — open 5/5 shells;naked=14;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 8f/0s/14naked/INVALID/0.0000 vs oracle 22.7967/10f/1s — open 5/5 shells;naked=14;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 4f/0s/18naked/INVALID/0.0000 vs oracle 71.1481/15f/1s — open 2/2 shells;naked=18;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_001 | P_boxoff | R_box_cut_box2_s0 | **FAIL** ours 7f/0s/12naked/INVALID/0.0000 vs oracle 17.7030/9f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/0s/12naked/INVALID/0.0000 vs oracle 25.1720/6f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 4f/0s/10naked/INVALID/0.0000 vs oracle 81.7030/12f/1s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_002 | P_boxoff | R_box_cut_box2_s1 | **FAIL** ours 4f/0s/10naked/INVALID/0.0000 vs oracle 23.3000/9f/1s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 8f/0s/10naked/INVALID/0.0000 vs oracle 19.5750/6f/1s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 4f/0s/10naked/INVALID/0.0000 vs oracle 50.3000/12f/1s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_003 | P_boxoff | R_box_cut_cyl | **FAIL** ours 10f/1s/8naked/INVALID/42.8750 vs oracle 33.1257/12f/1s — open 1/2 shells;naked=8;OCCT-invalid;vol rel=2.94e-01 | **FAIL** ours 4f/0s/8naked/INVALID/0.0000 vs oracle 9.7493/8f/1s — open 1/1 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/1s/4naked/INVALID/42.8750 vs oracle 68.8513/14f/1s — open 1/2 shells;naked=4;OCCT-invalid;vol rel=3.77e-01 |
| pvpair_004 | P_boxoff | R_box_cut_sph | **FAIL** ours 8f/0s/20naked/INVALID/0.0000 vs oracle 40.4997/15f/1s — open 2/2 shells;naked=20;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/20naked/INVALID/0.0000 vs oracle 2.3753/16f/3s — open 2/2 shells;naked=20;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 3 | **FAIL** ours 5f/0s/12naked/INVALID/0.0000 vs oracle 53.4299/13f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_005 | P_boxoff | R_box_fus_cyl | **FAIL** ours 8f/0s/16naked/INVALID/0.0000 vs oracle 13.8473/11f/1s — open 3/3 shells;naked=16;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 8f/0s/16naked/INVALID/0.0000 vs oracle 29.0277/9f/1s — open 3/3 shells;naked=16;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/20naked/INVALID/0.0000 vs oracle 106.1217/16f/1s — open 4/4 shells;naked=20;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_006 | P_boxoff | R_cyl_cut_cylx | **FAIL** ours 8f/0s/12naked/INVALID/0.0000 vs oracle 33.1568/12f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/0s/11naked/INVALID/0.0000 vs oracle 9.7182/12f/2s — open 2/2 shells;naked=11;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 3f/0s/6naked/INVALID/0.0000 vs oracle 77.7459/11f/1s — open 1/1 shells;naked=6;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_007 | P_boxoff | R_wed_cut_cyl | **FAIL** ours 8f/0s/18naked/INVALID/0.0000 vs oracle 41.2074/13f/1s — open 2/2 shells;naked=18;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 8f/0s/18naked/INVALID/0.0000 vs oracle 1.6676/11f/2s — open 2/2 shells;naked=18;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 6f/0s/14naked/INVALID/0.0000 vs oracle 58.5051/14f/1s — open 2/2 shells;naked=14;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_008 | P_cylz | R_box_com_sph | **FAIL** ours 8f/0s/8naked/INVALID/0.0000 vs oracle 9.8894/10f/2s — open 3/3 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 7f/0s/9naked/INVALID/0.0000 vs oracle 12.1018/7f/1s — open 2/2 shells;naked=9;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/6naked/INVALID/0.0000 vs oracle 60.9591/11f/1s — open 3/3 shells;naked=6;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_009 | P_cylz | R_box_cut_box2_s0 | **FAIL** ours 6f/0s/6naked/INVALID/0.0000 vs oracle 9.4248/6f/2s — open 2/2 shells;naked=6;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 7f/0s/7naked/INVALID/0.0000 vs oracle 12.5664/3f/1s — open 2/2 shells;naked=7;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/8naked/INVALID/0.0000 vs oracle 73.4248/10f/1s — open 3/3 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_010 | P_cylz | R_box_cut_box2_s1 | **FAIL** ours 7f/1s/2naked/INVALID/27.0000 vs oracle 13.0569/6f/1s — open 1/2 shells;naked=2;OCCT-invalid;vol rel=1.07e+00 | **FAIL** ours 8f/1s/3naked/INVALID/27.0000 vs oracle 8.9343/4f/1s — open 1/2 shells;naked=3;OCCT-invalid;vol rel=2.02e+00 | **FAIL** ours 1f/0s/2naked/INVALID/0.0000 vs oracle 40.0569/10f/1s — open 1/1 shells;naked=2;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_011 | P_cylz | R_box_cut_cyl | **FAIL** ours 9f/1s/6naked/INVALID/21.9911 vs oracle 19.1923/7f/1s — open 1/2 shells;naked=6;OCCT-invalid;vol rel=1.46e-01 | **FAIL** ours 6f/0s/5naked/INVALID/0.0000 vs oracle 2.7988/6f/1s — open 1/1 shells;naked=5;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 4f/1s/5naked/INVALID/21.9911 vs oracle 54.9180/10f/1s — open 1/2 shells;naked=5;OCCT-invalid;vol rel=6.00e-01 |
| pvpair_012 | P_cylz | R_box_cut_sph | **FAIL** ours 7f/0s/13naked/INVALID/0.0000 vs oracle 21.5265/9f/1s — open 2/2 shells;naked=13;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 8f/0s/12naked/INVALID/0.0000 vs oracle 0.4646/10f/2s — open 3/3 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 1f/0s/6naked/INVALID/0.0000 vs oracle 34.4568/10f/1s — open 1/1 shells;naked=6;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_013 | P_cylz | R_box_fus_cyl | **FAIL** ours 9f/0s/6naked/INVALID/0.0000 vs oracle 2.0991/12f/2s — open 1/1 shells;naked=6;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 12f/1s/7naked/INVALID/21.9911 vs oracle 19.8920/9f/1s — open 1/2 shells;naked=7;OCCT-invalid;vol rel=1.06e-01 | **FAIL** ours 1f/0s/4naked/INVALID/0.0000 vs oracle 94.3734/16f/1s — open 1/1 shells;naked=4;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_014 | P_cylz | R_cyl_cut_cylx | **FAIL** ours 6f/0s/6naked/INVALID/0.0000 vs oracle 10.2244/10f/1s — open 1/1 shells;naked=6;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 9f/1s/7naked/INVALID/21.9911 vs oracle 11.7668/11f/2s — open 1/2 shells;naked=7;OCCT-invalid;vol rel=8.69e-01;solids 1 vs 2 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 54.8136/8f/1s — empty result, oracle has 1 solids |
| pvpair_015 | P_cylz | R_wed_cut_cyl | **FAIL** ours 10f/1s/4naked/INVALID/38.1208 vs oracle 21.9302/7f/1s — open 2/3 shells;naked=4;OCCT-invalid;vol rel=7.38e-01 | **FAIL** ours 9f/1s/5naked/INVALID/38.1208 vs oracle 0.0610/5f/1s — open 1/2 shells;naked=5;OCCT-invalid;vol rel=6.24e+02 | **FAIL** ours 2f/0s/4naked/INVALID/0.0000 vs oracle 39.2279/11f/1s — open 2/2 shells;naked=4;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_016 | P_sphoff | R_box_com_sph | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 2.9066/6f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 21.5224/6f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 53.9763/8f/1s — empty result, oracle has 1 solids |
| pvpair_017 | P_sphoff | R_box_cut_box2_s0 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 1.9813/7f/3s — empty result, oracle has 3 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 22.4477/4f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 65.9813/10f/1s — empty result, oracle has 1 solids |
| pvpair_018 | P_sphoff | R_box_cut_box2_s1 | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 8.9405/5f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 15.4885/7f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 35.9405/11f/1s — empty result, oracle has 1 solids |
| pvpair_019 | P_sphoff | R_box_cut_cyl | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 17.1263/9f/3s — empty result, oracle has 3 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 7.3029/7f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 52.8518/11f/1s — empty result, oracle has 1 solids |
| pvpair_020 | P_sphoff | R_box_cut_sph | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 23.5037/6f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 0.9253/7f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 36.4340/8f/1s — empty result, oracle has 1 solids |
| pvpair_021 | P_sphoff | R_box_fus_cyl | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 1.5293/10f/3s — empty result, oracle has 3 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 22.8997/6f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 93.8037/15f/1s — empty result, oracle has 1 solids |
| pvpair_022 | P_sphoff | R_cyl_cut_cylx | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 17.5821/7f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 6.8475/6f/1s — empty result, oracle has 1 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 62.1717/7f/1s — empty result, oracle has 1 solids |
| pvpair_023 | P_sphoff | R_wed_cut_cyl | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 20.6225/9f/2s — empty result, oracle has 2 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 3.8065/9f/2s — empty result, oracle has 2 solids | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 37.9203/10f/1s — empty result, oracle has 1 solids |
| pvpair_024 | P_wedge | R_box_com_sph | **FAIL** ours 6f/0s/4naked/INVALID/0.0000 vs oracle 1.1624/10f/2s — open 1/1 shells;naked=4;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 12f/1s/4naked/INVALID/26.3640 vs oracle 25.2016/8f/1s — open 1/2 shells;naked=4;OCCT-invalid;vol rel=4.61e-02 | **FAIL** ours 1f/0s/2naked/INVALID/0.0000 vs oracle 52.2322/16f/1s — open 1/1 shells;naked=2;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_025 | P_wedge | R_box_cut_box2_s0 | **FAIL** ours 6f/1s/0naked/valid/64.0000 vs oracle empty/0f/0s — non-empty, oracle empty [no trustworthy oracle] | **PASS-CLOSURE** ours 12f/2s/0naked/valid/90.3640 vs oracle 26.3640/6f/1s — closed+valid; oracle untrustworthy so volume not scored | **FAIL** ours 0f/0s/0naked/INVALID/0.0000 vs oracle 64.0000/6f/1s — empty result, oracle has 1 solids |
| pvpair_026 | P_wedge | R_box_cut_box2_s1 | **FAIL** ours 7f/0s/10naked/INVALID/0.0000 vs oracle 11.1852/9f/1s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 9f/0s/10naked/INVALID/0.0000 vs oracle 15.1788/7f/1s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 2f/0s/6naked/INVALID/0.0000 vs oracle 38.1852/14f/1s — open 1/1 shells;naked=6;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_027 | P_wedge | R_box_cut_cyl | **FAIL** ours 10f/1s/9naked/INVALID/35.6984 vs oracle 16.9949/4f/1s — open 1/2 shells;naked=9;OCCT-invalid;vol rel=1.10e+00 | **FAIL** ours 10f/1s/9naked/INVALID/35.6984 vs oracle 9.3691/8f/1s — open 1/2 shells;naked=9;OCCT-invalid;vol rel=2.81e+00 | **FAIL** ours 3f/0s/8naked/INVALID/0.0000 vs oracle 52.7206/10f/1s — open 1/1 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_028 | P_wedge | R_box_cut_sph | **FAIL** ours 11f/1s/10naked/INVALID/3.7093 vs oracle 25.2016/8f/1s — open 1/2 shells;naked=10;OCCT-invalid;vol rel=8.53e-01 | **FAIL** ours 9f/1s/10naked/INVALID/80.0268 vs oracle 1.1624/10f/2s — open 2/3 shells;naked=10;OCCT-invalid;vol rel=6.78e+01;solids 1 vs 2 | **FAIL** ours 4f/0s/8naked/INVALID/0.0000 vs oracle 38.1318/14f/1s — open 1/1 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_029 | P_wedge | R_box_fus_cyl | **FAIL** ours 10f/0s/8naked/INVALID/0.0000 vs oracle empty/0f/0s — open 2/2 shells;naked=8;OCCT-invalid;non-empty, oracle empty [no trustworthy oracle] | **FAIL** ours 14f/0s/8naked/INVALID/0.0000 vs oracle 26.3640/6f/1s — open 2/2 shells;naked=8;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 2f/0s/6naked/INVALID/0.0000 vs oracle 92.2743/10f/1s — open 2/2 shells;naked=6;OCCT-invalid [no trustworthy oracle] |
| pvpair_030 | P_wedge | R_cyl_cut_cylx | **FAIL** ours 6f/0s/9naked/INVALID/0.0000 vs oracle 18.4348/10f/1s — open 2/2 shells;naked=9;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 10f/0s/9naked/INVALID/0.0000 vs oracle 7.9292/9f/2s — open 2/2 shells;naked=9;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 2f/0s/6naked/INVALID/0.0000 vs oracle 63.0240/13f/1s — open 2/2 shells;naked=6;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_031 | P_wedge | R_wed_cut_cyl | **FAIL** ours 7f/0s/8naked/INVALID/0.0000 vs oracle 16.9949/4f/1s — open 1/1 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 13f/1s/8naked/INVALID/26.3640 vs oracle 9.3691/8f/1s — open 1/2 shells;naked=8;OCCT-invalid;vol rel=1.81e+00 | **FAIL** ours 1f/0s/4naked/INVALID/0.0000 vs oracle 34.2927/11f/1s — open 1/1 shells;naked=4;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_032 | R_box_com_sph | R_box_cut_box2_s0 | **FAIL** ours 7f/0s/14naked/INVALID/0.0000 vs oracle empty/0f/0s — open 4/4 shells;naked=14;OCCT-invalid;non-empty, oracle empty [no trustworthy oracle] | **FAIL** ours 8f/0s/16naked/INVALID/0.0000 vs oracle 51.0697/7f/1s — open 2/2 shells;naked=16;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 3f/0s/6naked/INVALID/0.0000 vs oracle 64.0000/12f/1s — open 3/3 shells;naked=6;OCCT-invalid [no trustworthy oracle] |
| pvpair_033 | R_box_com_sph | R_box_cut_box2_s1 | **FAIL** ours 10f/0s/10naked/INVALID/0.0000 vs oracle 24.7575/13f/1s — open 2/2 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/0s/8naked/INVALID/0.0000 vs oracle 26.3122/10f/1s — open 3/3 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/0s/8naked/INVALID/0.0000 vs oracle 51.7575/19f/1s — open 2/2 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_034 | R_box_com_sph | R_box_cut_cyl | **FAIL** ours 7f/0s/17naked/INVALID/0.0000 vs oracle 28.0817/5f/1s — open 4/4 shells;naked=17;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 10f/0s/19naked/INVALID/0.0000 vs oracle 22.9880/6f/1s — open 3/3 shells;naked=19;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/6naked/INVALID/0.0000 vs oracle 63.8074/16f/1s — open 3/3 shells;naked=6;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_035 | R_box_com_sph | R_box_fus_cyl | **FAIL** ours 7f/0s/14naked/INVALID/0.0000 vs oracle empty/0f/0s — open 4/4 shells;naked=14;OCCT-invalid;non-empty, oracle empty [no trustworthy oracle] | **FAIL** ours 8f/0s/16naked/INVALID/0.0000 vs oracle 51.0697/7f/1s — open 2/2 shells;naked=16;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 7f/0s/12naked/INVALID/0.0000 vs oracle 92.2743/14f/1s — open 5/5 shells;naked=12;OCCT-invalid [no trustworthy oracle] |
| pvpair_036 | R_box_com_sph | R_cyl_cut_cylx | **FAIL** ours 5f/0s/24naked/INVALID/0.0000 vs oracle 34.9474/8f/1s — open 5/5 shells;naked=24;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 5f/0s/21naked/INVALID/0.0000 vs oracle 16.1223/7f/1s — open 2/2 shells;naked=21;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/0s/16naked/INVALID/0.0000 vs oracle 79.5367/9f/1s — open 6/6 shells;naked=16;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_037 | R_box_com_sph | R_wed_cut_cyl | **FAIL** ours 13f/0s/14naked/INVALID/0.0000 vs oracle 38.1173/13f/1s — open 6/6 shells;naked=14;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 10f/0s/16naked/INVALID/0.0000 vs oracle 12.9524/10f/1s — open 2/2 shells;naked=16;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 5f/0s/10naked/INVALID/0.0000 vs oracle 55.4151/17f/1s — open 5/5 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_038 | R_box_cut_box2_s0 | R_box_cut_box2_s1 | **FAIL** ours 11f/1s/4naked/INVALID/27.0000 vs oracle 37.0000/12f/1s — open 1/2 shells;naked=4;OCCT-invalid;vol rel=2.70e-01 | **FAIL** ours 7f/1s/4naked/INVALID/27.0000 vs oracle 27.0000/6f/1s — open 1/2 shells;naked=4;OCCT-invalid | **FAIL** ours 4f/0s/8naked/INVALID/0.0000 vs oracle 64.0000/6f/1s — open 1/1 shells;naked=8;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_039 | R_box_cut_box2_s0 | R_box_cut_cyl | **FAIL** ours 1f/0s/5naked/INVALID/0.0000 vs oracle 28.2743/3f/1s — open 1/1 shells;naked=5;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/1s/5naked/INVALID/64.0000 vs oracle 35.7257/7f/1s — open 1/2 shells;naked=5;OCCT-invalid;vol rel=7.91e-01 | **FAIL** ours 5f/0s/4naked/INVALID/0.0000 vs oracle 64.0000/8f/1s — open 1/1 shells;naked=4;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_040 | R_box_cut_box2_s0 | R_box_cut_sph | **FAIL** ours 5f/0s/22naked/INVALID/0.0000 vs oracle 51.0697/7f/1s — open 2/2 shells;naked=22;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 3f/0s/22naked/INVALID/0.0000 vs oracle 12.9303/7f/1s — open 3/3 shells;naked=22;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **PASS** ours 6f/1s/0naked/valid/64.0000 vs oracle 64.0000/12f/1s |
| pvpair_041 | R_box_cut_box2_s0 | R_box_fus_cyl | **FAIL** ours 2f/0s/10naked/INVALID/0.0000 vs oracle empty/0f/0s — open 2/2 shells;naked=10;OCCT-invalid;non-empty, oracle empty [no trustworthy oracle] | **FAIL** ours 8f/1s/10naked/INVALID/64.0000 vs oracle 64.0000/8f/1s — open 2/3 shells;naked=10;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 8f/1s/4naked/INVALID/64.0000 vs oracle 92.2743/10f/1s — open 2/3 shells;naked=4;OCCT-invalid [no trustworthy oracle] |
| pvpair_042 | R_box_cut_box2_s0 | R_cyl_cut_cylx | **FAIL** ours 6f/0s/13naked/INVALID/0.0000 vs oracle 47.6851/9f/1s — open 2/2 shells;naked=13;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/13naked/INVALID/0.0000 vs oracle 16.3149/5f/1s — open 2/2 shells;naked=13;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 5f/0s/12naked/INVALID/0.0000 vs oracle 92.2743/10f/1s — open 3/3 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_043 | R_box_cut_box2_s0 | R_wed_cut_cyl | **FAIL** ours 10f/0s/12naked/INVALID/0.0000 vs oracle 46.7022/11f/1s — open 1/1 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 8f/0s/12naked/INVALID/0.0000 vs oracle 17.2978/8f/1s — open 2/2 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **PASS** ours 6f/1s/0naked/valid/64.0000 vs oracle 64.0000/7f/1s |
| pvpair_044 | R_box_cut_box2_s1 | R_box_cut_cyl | **FAIL** ours 6f/0s/21naked/INVALID/0.0000 vs oracle 19.8852/7f/1s — open 4/4 shells;naked=21;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/21naked/INVALID/0.0000 vs oracle 7.1148/13f/2s — open 4/4 shells;naked=21;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 7f/0s/20naked/INVALID/0.0000 vs oracle 55.6109/11f/1s — open 2/2 shells;naked=20;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_045 | R_box_cut_box2_s1 | R_box_cut_sph | **FAIL** ours 8f/0s/22naked/INVALID/0.0000 vs oracle 26.3122/10f/1s — open 2/2 shells;naked=22;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 4f/0s/22naked/INVALID/0.0000 vs oracle 0.6878/16f/3s — open 2/2 shells;naked=22;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 3 | **FAIL** ours 9f/0s/20naked/INVALID/0.0000 vs oracle 39.2425/13f/1s — open 2/2 shells;naked=20;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_046 | R_box_cut_box2_s1 | R_box_fus_cyl | **FAIL** ours 3f/0s/12naked/INVALID/0.0000 vs oracle empty/0f/0s — open 3/3 shells;naked=12;OCCT-invalid;non-empty, oracle empty [no trustworthy oracle] | **FAIL** ours 7f/0s/12naked/INVALID/0.0000 vs oracle 27.0000/6f/1s — open 3/3 shells;naked=12;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 9f/0s/14naked/INVALID/0.0000 vs oracle 92.2743/10f/1s — open 2/2 shells;naked=14;OCCT-invalid [no trustworthy oracle] |
| pvpair_047 | R_box_cut_box2_s1 | R_cyl_cut_cylx | **FAIL** ours 6f/0s/11naked/INVALID/0.0000 vs oracle 18.9103/15f/1s — open 2/2 shells;naked=11;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/0s/11naked/INVALID/0.0000 vs oracle 8.0897/11f/1s — open 2/2 shells;naked=11;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 5f/0s/10naked/INVALID/0.0000 vs oracle 63.4995/15f/1s — open 3/3 shells;naked=10;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_048 | R_box_cut_box2_s1 | R_wed_cut_cyl | **FAIL** ours 5f/0s/16naked/INVALID/0.0000 vs oracle 24.1622/14f/1s — open 2/2 shells;naked=16;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/0s/16naked/INVALID/0.0000 vs oracle 2.8378/17f/3s — open 2/2 shells;naked=16;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 3 | **FAIL** ours 6f/0s/16naked/INVALID/0.0000 vs oracle 41.4599/14f/1s — open 2/2 shells;naked=16;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_049 | R_box_cut_cyl | R_box_cut_sph | **FAIL** ours 7f/0s/25naked/INVALID/0.0000 vs oracle 22.9880/6f/1s — open 2/2 shells;naked=25;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 4f/0s/24naked/INVALID/0.0000 vs oracle 12.7377/9f/1s — open 1/1 shells;naked=24;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/1s/1naked/valid/35.6984 vs oracle 35.9183/15f/1s — naked=1;vol rel=6.12e-03 |
| pvpair_050 | R_box_cut_cyl | R_box_fus_cyl | **FAIL** ours 3f/1s/5naked/INVALID/0.0000 vs oracle empty/0f/0s — open 1/2 shells;naked=5;OCCT-invalid;non-empty, oracle empty [no trustworthy oracle] | **FAIL** ours 6f/0s/4naked/INVALID/0.0000 vs oracle 35.7257/7f/1s — open 1/1 shells;naked=4;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 7f/0s/12naked/INVALID/0.0000 vs oracle 92.2743/10f/1s — open 3/3 shells;naked=12;OCCT-invalid [no trustworthy oracle] |
| pvpair_051 | R_box_cut_cyl | R_wed_cut_cyl | **FAIL** ours 7f/0s/20naked/INVALID/0.0000 vs oracle 18.4279/11f/1s — open 2/2 shells;naked=20;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 6f/0s/21naked/INVALID/0.0000 vs oracle 17.2978/8f/1s — open 4/4 shells;naked=21;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 8f/0s/16naked/INVALID/0.0000 vs oracle 35.7257/9f/1s — open 3/3 shells;naked=16;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_052 | R_box_cut_sph | R_box_fus_cyl | **FAIL** ours 6f/0s/16naked/INVALID/0.0000 vs oracle empty/0f/0s — open 3/3 shells;naked=16;OCCT-invalid;non-empty, oracle empty [no trustworthy oracle] | **FAIL** ours 9f/0s/18naked/INVALID/0.0000 vs oracle 12.9303/9f/1s — open 3/3 shells;naked=18;OCCT-invalid [no trustworthy oracle] | **FAIL** ours 8f/0s/14naked/INVALID/0.0000 vs oracle 92.2743/14f/1s — open 1/1 shells;naked=14;OCCT-invalid [no trustworthy oracle] |
| pvpair_053 | R_box_cut_sph | R_cyl_cut_cylx | **FAIL** ours 10f/0s/23naked/INVALID/0.0000 vs oracle 12.7377/9f/1s — open 2/2 shells;naked=23;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 9f/0s/21naked/INVALID/0.0000 vs oracle 0.1926/6f/2s — open 2/2 shells;naked=21;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 2 | **FAIL** ours 4f/0s/16naked/INVALID/0.0000 vs oracle 57.3269/14f/1s — open 1/1 shells;naked=16;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_054 | R_box_cut_sph | R_wed_cut_cyl | **FAIL** ours 10f/0s/22naked/INVALID/0.0000 vs oracle 8.5849/13f/1s — open 2/2 shells;naked=22;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 5f/0s/20naked/INVALID/0.0000 vs oracle 4.3454/10f/1s — open 2/2 shells;naked=20;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 10f/0s/12naked/INVALID/0.0000 vs oracle 25.8827/17f/1s — open 1/1 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_055 | R_box_fus_cyl | R_cyl_cut_cylx | **FAIL** ours 6f/0s/24naked/INVALID/0.0000 vs oracle 47.6851/9f/1s — open 2/2 shells;naked=24;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 10f/0s/26naked/INVALID/0.0000 vs oracle 44.5892/7f/1s — open 2/2 shells;naked=26;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/0s/14naked/INVALID/0.0000 vs oracle 92.2743/10f/1s — open 3/3 shells;naked=14;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 |
| pvpair_056 | R_box_fus_cyl | R_wed_cut_cyl | **FAIL** ours 13f/0s/12naked/INVALID/0.0000 vs oracle 74.9766/15f/1s — open 1/1 shells;naked=12;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 7f/0s/14naked/INVALID/0.0000 vs oracle 17.2978/8f/1s — open 2/2 shells;naked=14;OCCT-invalid;vol rel=1.00e+00;solids 0 vs 1 | **FAIL** ours 10f/1s/2naked/valid/92.2471 vs oracle 92.2743/11f/1s — naked=2;vol rel=2.96e-04 |

### A. hard cross-pairs pair_000..079

| pair | A | B | cut | common | fuse |
|---|---|---|---|---|---|
| pair_000 | occtref_x20_fuse | occtref_z30_fuse | **TIMEOUT** (oracle 160.5040/162f/2s/OCCT-INVALID) | **NOTRUN** (oracle 0.0893/9f/2s) | **NOTRUN** (oracle 7.3358/14f/2s) |
| pair_001 | occtref_x20_fuse | occtref_y30_fuse | **TIMEOUT** (oracle 126.9513/120f/1s/OCCT-INVALID) | **NOTRUN** (oracle 0.0061/5f/1s) | **NOTRUN** (oracle 121.8913/114f/2s/OCCT-INVALID) |
| pair_002 | occtref_x20_fuse | occtref_z90_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 0.2840/6f/1s) | **NOTRUN** (oracle 147.2902/92f/1s) |
| pair_003 | occtref_x20_fuse | occtref_z30x20_fuse | **TIMEOUT** (oracle 149.7870/131f/1s/OCCT-INVALID) | **NOTRUN** (oracle 0.0379/10f/2s) | **NOTRUN** (oracle 134.5270/108f/1s) |
| pair_004 | occtref_x13y29_fuse | occtref_x20_fuse | **TIMEOUT** (oracle 6.8106/38f/6s) | **NOTRUN** (oracle 0.0405/10f/2s) | **NOTRUN** (oracle 0.0008/5f/1s) |
| pair_005 | occtref_z30_fuse | occtref_z63_fuse | **TIMEOUT** (oracle 136.2313/151f/5s) | **NOTRUN** (oracle 0.0007/4f/1s) | **NOTRUN** (oracle 169.0187/101f/1s) |
| pair_006 | occtref_z30_fuse | occtref_z45_fuse | **TIMEOUT** (oracle 11.8666/65f/2s) | **NOTRUN** (oracle 118.1778/125f/1s/OCCT-INVALID) | **NOTRUN** (oracle 128.2892/110f/1s/OCCT-INVALID) |
| pair_007 | occtref_z15_fuse | occtref_z45_fuse | **TIMEOUT** (oracle 0.4985/8f/1s) | **NOTRUN** (oracle 80.4537/83f/1s) | **NOTRUN** (oracle 139.9918/78f/1s) |
| pair_008 | occtref_y30_fuse | occtref_z15_fuse | **TIMEOUT** (oracle 47.5073/66f/2s) | **NOTRUN** (oracle 79.7485/86f/1s) | **NOTRUN** (oracle 128.4591/79f/1s) |
| pair_009 | occtref_z15_fuse | occtref_z63_fuse | **TIMEOUT** (oracle 0.5525/8f/1s) | **NOTRUN** (oracle 80.3995/84f/1s) | **NOTRUN** (oracle 143.4723/76f/1s) |
| pair_010 | occtref_z15_fuse | occtref_z90_fuse | **TIMEOUT** (oracle 0.5701/8f/1s) | **NOTRUN** (oracle 80.3819/84f/1s) | **NOTRUN** (oracle 147.8605/74f/1s) |
| pair_011 | occtref_z15_cut_s0 | occtref_z45_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 80.2695/77f/1s) | **NOTRUN** (oracle 139.4933/76f/1s) |
| pair_012 | occtref_z15_fuse | occtref_z30x20_fuse | **TIMEOUT** (oracle 0.2746/11f/2s) | **NOTRUN** (oracle 80.6775/79f/2s) | **NOTRUN** (oracle 134.8295/78f/1s) |
| pair_013 | occtref_z30_cut_s0 | occtref_z45_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 55.8442/56f/1s) | **NOTRUN** (oracle 139.4932/77f/1s) |
| pair_014 | occtref_z37_cut_s0 | occtref_z45_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 57.4575/59f/1s) | **NOTRUN** (oracle 139.4931/76f/1s) |
| pair_015 | occtref_z15_cut_s0 | occtref_z63_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 80.2693/78f/1s) | **NOTRUN** (oracle 142.9198/74f/1s) |
| pair_016 | occtref_z15_cut_s0 | occtref_z90_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 80.2693/80f/1s) | **NOTRUN** (oracle 147.2905/73f/1s) |
| pair_017 | occtref_y30_fuse | occtref_z15_cut_s0 | **TIMEOUT** (oracle 47.7200/65f/2s) | **NOTRUN** (oracle 79.5359/83f/1s) | **NOTRUN** (oracle 127.9891/78f/1s) |
| pair_018 | occtref_y30_cut | occtref_z30_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 46.9590/50f/1s) | **NOTRUN** (oracle 136.2318/80f/1s) |
| pair_019 | occtref_z15_cut_s0 | occtref_z30x20_fuse | **TIMEOUT** (oracle -0.0277/5f/1s) | **NOTRUN** (oracle 80.2973/76f/2s) | **NOTRUN** (oracle 134.5272/77f/1s) |
| pair_020 | occtref_z45_cut_s0 | occtref_z45_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 59.1084/37f/1s) | **NOTRUN** (oracle 139.4931/55f/1s) |
| pair_021 | occtref_z45_fuse | occtref_z63_cut_s0 | **TIMEOUT** (oracle 76.8866/81f/2s) | **NOTRUN** (oracle 62.6064/55f/1s) | **NOTRUN** (oracle 139.4930/78f/1s) |
| pair_022 | occtref_z45_fuse | occtref_z63_fuse | **TIMEOUT** (oracle 19.7114/75f/3s) | **NOTRUN** (oracle 119.7812/120f/1s) | **NOTRUN** (oracle 162.6309/101f/1s) |
| pair_023 | occtref_z37_cut_s0 | occtref_z63_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 57.4574/60f/1s) | **NOTRUN** (oracle 142.9197/73f/1s) |
| pair_024 | occtref_z30_cut_s0 | occtref_z63_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 55.8441/59f/1s) | **NOTRUN** (oracle 142.9196/73f/1s) |
| pair_025 | occtref_x20_cut | occtref_z90_fuse | **TIMEOUT** (oracle -0.0000/2f/1s) | **NOTRUN** (oracle 80.2963/72f/1s) | **NOTRUN** (oracle 147.2905/68f/1s) |
| pair_026 | occtref_z30_cut_s0 | occtref_z90_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 55.8440/63f/1s) | **NOTRUN** (oracle 147.2905/74f/1s) |
| pair_027 | occtref_z37_cut_s0 | occtref_z90_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 57.4572/61f/1s) | **NOTRUN** (oracle 147.2903/73f/1s) |
| pair_028 | occtref_z45_fuse | occtref_z90_cut | **TIMEOUT** (oracle 72.4996/76f/2s) | **NOTRUN** (oracle 66.9931/52f/1s) | **NOTRUN** (oracle 139.4929/76f/1s) |
| pair_029 | occtref_x20_cut | occtref_y30_fuse | **TIMEOUT** (oracle 0.7332/12f/1s) | **NOTRUN** (oracle 79.5634/76f/1s) | **NOTRUN** (oracle 127.9893/70f/1s) |
| pair_030 | occtref_y30_fuse | occtref_z30_cut_s0 | **TIMEOUT** (oracle 72.0945/82f/1s) | **NOTRUN** (oracle 55.1614/62f/1s) | **NOTRUN** (oracle 127.9385/83f/1s) |
| pair_031 | occtref_y30_fuse | occtref_z37_cut_s0 | **TIMEOUT** (oracle 70.5172/77f/1s) | **NOTRUN** (oracle 56.7388/60f/1s) | **NOTRUN** (oracle 127.9746/80f/1s) |
| pair_032 | occtref_x13y29_fuse | occtref_z15_fuse | **TIMEOUT** (oracle 49.7632/64f/3s) | **NOTRUN** (oracle 79.0066/82f/1s) | **NOTRUN** (oracle 130.7152/76f/1s) |
| pair_033 | occtref_z63_cut_s0 | occtref_z63_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 62.6064/37f/1s) | **NOTRUN** (oracle 142.9196/53f/1s) |
| pair_034 | occtref_z45_cut_s0 | occtref_z63_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 59.1084/60f/1s) | **NOTRUN** (oracle 142.9196/71f/1s) |
| pair_035 | occtref_z45_cut_s0 | occtref_z90_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 59.1083/58f/1s) | **NOTRUN** (oracle 147.2904/69f/1s) |
| pair_036 | occtref_z63_cut_s0 | occtref_z90_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 62.6062/59f/1s) | **NOTRUN** (oracle 147.2902/73f/1s) |
| pair_037 | occtref_y30_fuse | occtref_z63_cut_s0 | **TIMEOUT** (oracle 65.3410/84f/1s) | **NOTRUN** (oracle 61.9150/62f/1s) | **NOTRUN** (oracle 127.9472/76f/1s) |
| pair_038 | occtref_x13y29_fuse | occtref_y30_fuse | **TIMEOUT** (oracle 16.8017/74f/6s) | **NOTRUN** (oracle 111.9717/101f/1s) | **NOTRUN** (oracle 144.0617/101f/1s) |
| pair_039 | occtref_y30_fuse | occtref_z45_cut_s0 | **TIMEOUT** (oracle 68.8605/78f/1s) | **NOTRUN** (oracle 58.3955/61f/1s) | **NOTRUN** (oracle 127.9688/75f/1s) |
| pair_040 | occtref_x13y29_cut | occtref_z30_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 48.4729/46f/1s) | **NOTRUN** (oracle 136.2319/75f/1s) |
| pair_041 | occtref_z90_cut | occtref_z90_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 66.9934/36f/1s) | **NOTRUN** (oracle 147.2905/53f/1s) |
| pair_042 | occtref_z63_fuse | occtref_z90_cut | **TIMEOUT** (oracle 75.9264/77f/1s) | **NOTRUN** (oracle 66.9930/56f/1s) | **NOTRUN** (oracle 142.9194/75f/1s) |
| pair_043 | occtref_y30_fuse | occtref_z90_cut | **TIMEOUT** (oracle 60.9959/81f/1s) | **NOTRUN** (oracle 66.2599/62f/1s) | **NOTRUN** (oracle 127.9890/78f/1s) |
| pair_044 | occtref_x20_fuse | rotB_z30 | **TIMEOUT** (oracle 81.1834/114f/2s/OCCT-INVALID) | **NOTRUN** (oracle 0.0000/4f/1s) | **NOTRUN** (oracle 103.9233/115f/1s/OCCT-INVALID) |
| pair_045 | occtref_x20_fuse | rotB_y30 | **TIMEOUT** (oracle 153.8422/177f/3s/OCCT-INVALID) | **NOTRUN** (oracle 0.0061/5f/1s) | **NOTRUN** (oracle 148.3058/104f/1s) |
| pair_046 | occtref_x20_fuse | rotB_z37 | **TIMEOUT** (oracle 90.4782/139f/2s/OCCT-INVALID) | **NOTRUN** (oracle 0.0195/4f/1s) | **NOTRUN** (oracle empty/0f/0s) |
| pair_047 | occtref_x20_fuse | rotB_z30x20 | **TIMEOUT** (oracle 32.0819/67f/4s) | **NOTRUN** (oracle 0.0239/5f/1s) | **NOTRUN** (oracle 91.2806/106f/1s) |
| pair_048 | occtref_x20_fuse | rotB_z63 | **TIMEOUT** (oracle 29.5342/60f/3s) | **NOTRUN** (oracle 0.0330/5f/1s) | **NOTRUN** (oracle empty/0f/0s) |
| pair_049 | occtref_x20_fuse | rotB_x13y29 | **TIMEOUT** (oracle 5.6730/49f/5s) | **NOTRUN** (oracle 0.0462/14f/3s) | **NOTRUN** (oracle 85.9694/109f/1s) |
| pair_050 | occtref_x13y29_fuse | occtref_z15_cut_s0 | **TIMEOUT** (oracle 49.9146/65f/3s) | **NOTRUN** (oracle 78.8552/78f/1s) | **NOTRUN** (oracle 130.1839/75f/1s) |
| pair_051 | occtref_y30_fuse | occtref_z30_fuse | **TIMEOUT** (oracle 21.2563/54f/3s) | **NOTRUN** (oracle 99.1909/115f/1s/OCCT-INVALID) | **NOTRUN** (oracle empty/0f/0s) |
| pair_052 | occtref_x20_fuse | occtref_y30_cut | **TIMEOUT** (oracle 155.6198/104f/2s) | **NOTRUN** (oracle 4.9734/27f/3s) | **NOTRUN** (oracle 78.8186/70f/1s) |
| pair_053 | occtref_y30_cut | occtref_z45_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 46.9590/46f/1s) | **NOTRUN** (oracle 139.4931/73f/1s) |
| pair_054 | occtref_z30_fuse | occtref_z63_common_s0 | **TIMEOUT** (oracle 118.5906/85f/1s) | **NOTRUN** (oracle 17.6414/46f/1s) | **NOTRUN** (oracle 136.2319/81f/1s) |
| pair_055 | occtref_z30x20_cut | occtref_z45_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 54.2580/59f/1s) | **NOTRUN** (oracle 139.4933/71f/1s) |
| pair_056 | occtref_x20_fuse | rotB_z90 | **TIMEOUT** (oracle 8.7331/29f/1s) | **NOTRUN** (oracle 0.9021/13f/2s) | **NOTRUN** (oracle 80.2966/67f/1s) |
| pair_057 | occtref_x13y29_cut | occtref_x20_fuse | **TIMEOUT** (oracle 44.2140/46f/2s) | **NOTRUN** (oracle 4.2591/33f/4s) | **NOTRUN** (oracle 125.0683/66f/1s) |
| pair_058 | occtref_y30_cut | occtref_y30_fuse | **TIMEOUT** (oracle empty/0f/0s) | **TIMEOUT** (oracle 46.9589/32f/1s) | **TIMEOUT** (oracle 127.2558/54f/1s) |
| pair_059 | occtref_x13y29_fuse | occtref_x20_cut | **TIMEOUT** (oracle 49.8874/65f/3s) | **NOTRUN** (oracle 78.8824/76f/1s) | **NOTRUN** (oracle 130.1838/70f/1s) |
| pair_060 | occtref_x13y29_fuse | occtref_z30_cut_s0 | **TIMEOUT** (oracle 74.0748/73f/2s) | **NOTRUN** (oracle 54.6952/56f/2s) | **NOTRUN** (oracle 129.9188/74f/1s) |
| pair_061 | occtref_x13y29_fuse | occtref_z37_cut_s0 | **TIMEOUT** (oracle 72.5553/75f/2s) | **NOTRUN** (oracle 56.2147/58f/2s) | **NOTRUN** (oracle 130.0129/74f/1s) |
| pair_062 | occtref_z63_fuse | occtref_z90_fuse | **TIMEOUT** (oracle 24.8551/77f/3s) | **NOTRUN** (oracle 118.0636/108f/1s) | **NOTRUN** (oracle 172.1449/100f/1s) |
| pair_063 | occtref_y30_cut | occtref_z63_fuse | **TIMEOUT** (oracle empty/0f/0s) | **TIMEOUT** (oracle 46.9590/49f/1s) | **TIMEOUT** (oracle 142.9196/72f/1s) |
| pair_064 | occtref_z30x20_cut | occtref_z63_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 54.2577/59f/1s) | **NOTRUN** (oracle 142.9196/68f/1s) |
| pair_065 | occtref_y30_cut | occtref_z90_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 46.9588/52f/1s) | **NOTRUN** (oracle 147.2904/73f/1s) |
| pair_066 | occtref_z30x20_cut | occtref_z90_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 54.2576/55f/1s) | **NOTRUN** (oracle 147.2904/67f/1s) |
| pair_067 | occtref_z30_fuse | occtref_z37_common_s0 | **TIMEOUT** (oracle 113.4930/85f/1s) | **TIMEOUT** (oracle 22.7390/43f/1s) | **TIMEOUT** (oracle 136.2320/78f/1s) |
| pair_068 | occtref_x13y29_fuse | occtref_z63_cut_s0 | **TIMEOUT** (oracle 67.4883/83f/2s) | **NOTRUN** (oracle 61.2816/58f/1s) | **NOTRUN** (oracle 130.0945/77f/1s) |
| pair_069 | occtref_x13y29_fuse | occtref_z45_cut_s0 | **TIMEOUT** (oracle 70.9313/77f/2s) | **NOTRUN** (oracle 57.8387/58f/2s) | **NOTRUN** (oracle 130.0397/75f/1s) |
| pair_070 | occtref_y30_fuse | occtref_z30x20_cut | **TIMEOUT** (oracle 73.7317/78f/1s) | **TIMEOUT** (oracle 53.5241/63f/1s) | **TIMEOUT** (oracle 127.9889/71f/1s) |
| pair_071 | occtref_x13y29_cut | occtref_z45_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 48.4730/46f/1s) | **NOTRUN** (oracle 139.4931/70f/1s) |
| pair_072 | occtref_y30_cut | occtref_z30x20_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 46.9589/43f/1s) | **NOTRUN** (oracle 134.5543/75f/1s) |
| pair_073 | occtref_y30_fuse | occtref_z90_fuse | **TIMEOUT** (oracle 24.8790/84f/3s) | **NOTRUN** (oracle 102.3765/112f/2s) | **NOTRUN** (oracle 172.1689/106f/1s) |
| pair_074 | occtref_x13y29_fuse | occtref_z90_cut | **TIMEOUT** (oracle 63.1906/80f/2s) | **NOTRUN** (oracle 65.5791/59f/1s) | **NOTRUN** (oracle 130.1838/76f/1s) |
| pair_075 | occtref_x13y29_cut | occtref_y30_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 48.4729/49f/1s) | **NOTRUN** (oracle 127.2559/64f/1s) |
| pair_076 | occtref_z15_cut_s0 | occtref_z15_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 80.2694/40f/1s) | **NOTRUN** (oracle 80.9521/43f/1s) |
| pair_077 | occtref_z45_common | occtref_z45_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 21.1009/28f/1s) | **NOTRUN** (oracle 139.4931/55f/1s) |
| pair_078 | occtref_z45_fuse | occtref_z90_fuse | **TIMEOUT** (oracle 30.1862/74f/3s) | **NOTRUN** (oracle 109.3066/104f/1s) | **NOTRUN** (oracle 177.4763/96f/1s) |
| pair_079 | occtref_x13y29_cut | occtref_z63_fuse | **TIMEOUT** (oracle empty/0f/0s) | **NOTRUN** (oracle 48.4729/47f/1s) | **NOTRUN** (oracle 142.9197/68f/1s) |

### B. balanced cross-pairs bal_000..059

| pair | A | B | cut | common | fuse |
|---|---|---|---|---|---|
| bal_000 | occtref_x20_cut | occtref_x20_fuse | **TIMEOUT** (oracle 52.0005/37f/1s) | **NOTRUN** (oracle 28.2960/22f/1s) | **NOTRUN** (oracle empty/0f/0s) |
| bal_001 | occtref_x20_fuse | occtref_z90_cut | **TIMEOUT** (oracle 139.0023/103f/3s) | **NOTRUN** (oracle 21.5907/33f/2s) | **NOTRUN** (oracle 76.5893/71f/1s) |
| bal_002 | occtref_y30_fuse | rotB_z45 | **NOTRUN** (oracle 87.5813/79f/1s) | **NOTRUN** (oracle 39.6748/54f/3s) | **NOTRUN** (oracle 167.8779/96f/1s) |
| bal_003 | occtref_z45_fuse | rotB_z90 | **NOTRUN** (oracle 97.1795/81f/1s) | **NOTRUN** (oracle 42.3135/59f/2s) | **NOTRUN** (oracle 177.4762/96f/1s) |
| bal_004 | occtref_x20_fuse | occtref_z63_cut_s0 | **TIMEOUT** (oracle 142.9587/115f/3s/OCCT-INVALID) | **NOTRUN** (oracle 17.6349/39f/3s) | **NOTRUN** (oracle 80.2926/73f/1s) |
| bal_005 | occtref_y30_fuse | occtref_z30_fuse | **TIMEOUT** (oracle 21.2563/54f/3s) | **NOTRUN** (oracle 99.1909/115f/1s/OCCT-INVALID) | **NOTRUN** (oracle empty/0f/0s) |
| bal_006 | occtref_y30_fuse | rotB_z63 | **NOTRUN** (oracle 90.0268/76f/1s) | **NOTRUN** (oracle 37.2291/50f/1s) | **NOTRUN** (oracle 170.3232/95f/1s) |
| bal_007 | occtref_y30_fuse | rotB_z30 | **NOTRUN** (oracle 83.7474/82f/1s) | **NOTRUN** (oracle 43.5088/56f/1s) | **NOTRUN** (oracle 164.0433/104f/1s) |
| bal_008 | occtref_x13y29_fuse | rotB_z45 | **TIMEOUT** (oracle 87.2318/80f/1s) | **NOTRUN** (oracle 41.5384/54f/2s) | **NOTRUN** (oracle 167.5282/98f/1s) |
| bal_009 | occtref_y30_fuse | occtref_z45_fuse | **TIMEOUT** (oracle 29.0971/83f/3s) | **NOTRUN** (oracle 98.1589/107f/3s) | **NOTRUN** (oracle 168.5904/100f/1s) |
| bal_010 | occtref_z30_fuse | rotB_z63 | **NOTRUN** (oracle 88.8119/91f/1s) | **NOTRUN** (oracle 47.4201/65f/3s) | **NOTRUN** (oracle 169.1081/104f/1s) |
| bal_011 | occtref_x20_fuse | occtref_z45_cut_s0 | **TIMEOUT** (oracle 128.9529/92f/1s/OCCT-INVALID) | **NOTRUN** (oracle 14.8163/36f/3s) | **NOTRUN** (oracle empty/0f/0s) |
| bal_012 | occtref_x20_fuse | rotB_x20 | **NOTRUN** (oracle empty/0f/0s) | **NOTRUN** (oracle 28.2960/22f/1s) | **NOTRUN** (oracle 80.2970/36f/1s) |
| bal_013 | occtref_x20_fuse | chair0 | **NOTRUN** (oracle 132.2976/52f/1s) | **NOTRUN** (oracle 28.2958/22f/1s) | **NOTRUN** (oracle 80.2969/38f/1s) |
| bal_014 | occtref_x13y29_fuse | rotB_z37 | **TIMEOUT** (oracle 85.4645/76f/1s) | **NOTRUN** (oracle 43.3055/51f/1s) | **NOTRUN** (oracle 165.7614/93f/1s) |
| bal_015 | occtref_z45_fuse | occtref_z90_fuse | **TIMEOUT** (oracle 30.1862/74f/3s) | **NOTRUN** (oracle 109.3066/104f/1s) | **NOTRUN** (oracle 177.4763/96f/1s) |
| bal_016 | occtref_y30_fuse | occtref_z30x20_fuse | **TIMEOUT** (oracle 29.0319/90f/4s) | **NOTRUN** (oracle 98.2240/108f/1s) | **NOTRUN** (oracle 163.5856/101f/1s) |
| bal_017 | occtref_y30_fuse | occtref_z63_fuse | **TIMEOUT** (oracle 28.0954/82f/4s) | **NOTRUN** (oracle 99.1607/104f/1s) | **NOTRUN** (oracle 171.0147/97f/1s) |
| bal_018 | occtref_x20_fuse | occtref_z37_cut_s0 | **TIMEOUT** (oracle 147.5707/118f/3s/OCCT-INVALID) | **NOTRUN** (oracle 13.0227/41f/5s) | **NOTRUN** (oracle 74.1386/75f/1s) |
| bal_019 | occtref_x13y29_fuse | occtref_z45_fuse | **TIMEOUT** (oracle 29.3044/83f/5s) | **NOTRUN** (oracle 99.4656/104f/2s) | **NOTRUN** (oracle 168.7977/102f/1s) |
| bal_020 | occtref_y30_common_s0 | occtref_z37_cut_s0 | **TIMEOUT** (oracle 17.1546/35f/2s) | **NOTRUN** (oracle 15.9668/45f/3s) | **NOTRUN** (oracle 74.6123/83f/1s) |
| bal_021 | occtref_x13y29_fuse | rotB_z30 | **TIMEOUT** (oracle 83.6129/81f/1s) | **NOTRUN** (oracle 45.1571/54f/1s) | **NOTRUN** (oracle 163.9094/101f/1s) |
| bal_022 | occtref_x20_fuse | occtref_z30_cut_s0 | **TIMEOUT** (oracle 129.7497/98f/1s/OCCT-INVALID) | **NOTRUN** (oracle 11.3528/42f/4s) | **NOTRUN** (oracle empty/0f/0s) |
| bal_023 | occtref_y30_common_s0 | occtref_z45_cut_s0 | **TIMEOUT** (oracle 15.8346/33f/1s) | **NOTRUN** (oracle 17.2869/44f/3s) | **NOTRUN** (oracle 74.9433/78f/1s) |
| bal_024 | occtref_y30_fuse | occtref_z90_fuse | **TIMEOUT** (oracle 24.8790/84f/3s) | **NOTRUN** (oracle 102.3765/112f/2s) | **NOTRUN** (oracle 172.1689/106f/1s) |
| bal_025 | occtref_x13y29_fuse | occtref_z63_fuse | **RUNNING** (oracle 28.2314/81f/5s) | **NOTRUN** (oracle 100.5381/106f/2s) | **NOTRUN** (oracle 171.1514/99f/1s) |
| bal_026 | occtref_x13y29_common_s0 | occtref_z37_cut_s0 | **TIMEOUT** (oracle 15.8398/32f/2s) | **NOTRUN** (oracle 15.4203/34f/1s) | **NOTRUN** (oracle 73.2975/77f/1s) |
| bal_027 | occtref_z30x20_fuse | occtref_z90_fuse | **TIMEOUT** (oracle 27.0471/69f/4s) | **NOTRUN** (oracle 107.5072/103f/1s) | **NOTRUN** (oracle 174.3377/97f/1s) |
| bal_028 | occtref_x13y29_fuse | occtref_z30x20_fuse | **RUNNING** (oracle 2.3868/18f/4s) | **NOTRUN** (oracle 100.4884/107f/1s/OCCT-INVALID) | **NOTRUN** (oracle empty/0f/0s) |
| bal_029 | occtref_z30_fuse | rotB_y30 | **NOTRUN** (oracle 84.4599/95f/1s/OCCT-INVALID) | **NOTRUN** (oracle 51.7762/71f/1s/OCCT-INVALID) | **NOTRUN** (oracle 157.4880/116f/1s) |
| bal_030 | occtref_z30x20_fuse | rotB_z63 | **TIMEOUT** (oracle 86.2607/89f/2s/OCCT-INVALID) | **NOTRUN** (oracle 48.2941/74f/2s/OCCT-INVALID) | **NOTRUN** (oracle 148.0613/109f/1s) |
| bal_031 | occtref_z90_fuse | rotB_x20 | **TIMEOUT** (oracle 98.0297/78f/1s) | **NOTRUN** (oracle 49.2600/56f/3s) | **NOTRUN** (oracle 178.3265/88f/1s) |
| bal_032 | occtref_z30x20_common | occtref_z63_cut_s0 | **TIMEOUT** (oracle 12.5862/28f/1s) | **NOTRUN** (oracle 13.4533/36f/5s) | **NOTRUN** (oracle 75.1923/72f/1s) |
| bal_033 | occtref_z63_common_s0 | occtref_z90_cut | **TIMEOUT** (oracle 9.8173/30f/1s) | **NOTRUN** (oracle 7.8240/40f/2s) | **NOTRUN** (oracle 76.8108/78f/1s) |
| bal_034 | occtref_z63_fuse | occtref_z90_fuse | **TIMEOUT** (oracle 24.8551/77f/3s) | **NOTRUN** (oracle 118.0636/108f/1s) | **NOTRUN** (oracle 172.1449/100f/1s) |
| bal_035 | occtref_x13y29_fuse | occtref_z90_fuse | **RUNNING** (oracle 24.7891/88f/5s) | **NOTRUN** (oracle 103.9797/115f/2s) | **NOTRUN** (oracle 172.0792/106f/1s) |
| bal_036 | occtref_y30_common_s0 | occtref_z30_cut_s0 | **TIMEOUT** (oracle 18.5448/36f/1s) | **NOTRUN** (oracle 14.5768/51f/4s) | **NOTRUN** (oracle 74.3887/84f/1s) |
| bal_037 | occtref_z90_fuse | rotB_z45 | **TIMEOUT** (oracle 97.1796/83f/1s) | **NOTRUN** (oracle 50.1106/61f/1s) | **NOTRUN** (oracle 177.4764/96f/1s) |
| bal_038 | occtref_x13y29_common_s0 | occtref_z45_cut_s0 | **TIMEOUT** (oracle 14.6771/32f/1s) | **NOTRUN** (oracle 16.5831/39f/2s) | **NOTRUN** (oracle 73.7857/76f/1s) |
| bal_039 | occtref_x13y29_common_s0 | occtref_z30_cut_s0 | **TIMEOUT** (oracle 17.1300/29f/1s) | **NOTRUN** (oracle 14.1301/35f/1s) | **NOTRUN** (oracle 72.9741/73f/1s) |
| bal_040 | occtref_z30_common | occtref_z63_cut_s0 | **TIMEOUT** (oracle 14.1350/28f/1s) | **NOTRUN** (oracle 10.2272/39f/3s) | **NOTRUN** (oracle 76.7413/76f/1s) |
| bal_041 | occtref_z15_fuse | chair1 | **TIMEOUT** (oracle 46.8222/56f/1s) | **NOTRUN** (oracle 34.1298/45f/1s) | **NOTRUN** (oracle 127.1193/71f/1s) |
| bal_042 | occtref_z63_fuse | rotB_x20 | **TIMEOUT** (oracle 90.9519/79f/1s) | **NOTRUN** (oracle 51.9677/64f/2s) | **NOTRUN** (oracle 171.2488/96f/1s) |
| bal_043 | occtref_z63_fuse | rotB_z15 | **RUNNING** (oracle 90.9331/81f/1s) | **NOTRUN** (oracle 51.9863/63f/1s) | **NOTRUN** (oracle 171.2298/98f/1s) |
| bal_044 | occtref_z63_fuse | rotB_x13y29 | **RUNNING** (oracle 90.8543/83f/1s) | **NOTRUN** (oracle 52.0652/65f/2s) | **NOTRUN** (oracle 171.1513/99f/1s) |
| bal_045 | occtref_z63_fuse | rotB_y30 | **RUNNING** (oracle 90.7180/76f/1s) | **NOTRUN** (oracle 52.2015/61f/1s) | **NOTRUN** (oracle 171.0148/97f/1s) |
| bal_046 | occtref_z45_common | occtref_z90_cut | **TIMEOUT** (oracle 8.5523/26f/1s) | **NOTRUN** (oracle 12.5486/37f/2s) | **NOTRUN** (oracle 75.5459/71f/1s) |
| bal_047 | occtref_z15_fuse | rotB_y30 | **TIMEOUT** (oracle 48.1623/65f/2s) | **NOTRUN** (oracle 32.7895/52f/2s) | **NOTRUN** (oracle 128.4593/79f/1s) |
| bal_048 | occtref_z45_fuse | rotB_x20 | **NOTRUN** (oracle 85.9224/80f/1s) | **NOTRUN** (oracle 53.5707/50f/2s) | **NOTRUN** (oracle 166.2193/91f/1s) |
| bal_049 | occtref_z90_fuse | chair1 | **RUNNING** (oracle 94.6757/73f/1s) | **NOTRUN** (oracle 52.6145/46f/1s) | **NOTRUN** (oracle 174.9727/92f/1s) |
| bal_050 | occtref_x13y29_fuse | rotB_z15 | **TIMEOUT** (oracle 77.5185/79f/1s) | **NOTRUN** (oracle 51.2514/55f/2s) | **NOTRUN** (oracle 157.8144/101f/1s) |
| bal_051 | occtref_z90_fuse | rotB_z30x20 | **NOTRUN** (oracle 94.0404/83f/2s) | **NOTRUN** (oracle 53.2499/55f/1s) | **NOTRUN** (oracle 174.3373/97f/1s) |
| bal_052 | occtref_z63_fuse | chair1 | **NOTRUN** (oracle 89.6642/86f/2s) | **NOTRUN** (oracle 53.2563/66f/1s) | **NOTRUN** (oracle 169.9613/98f/1s) |
| bal_053 | occtref_z15_cut_s0 | chair1 | **TIMEOUT** (oracle 46.7929/54f/1s) | **NOTRUN** (oracle 33.4764/42f/1s) | **NOTRUN** (oracle 127.0902/69f/1s) |
| bal_054 | occtref_x20_fuse | occtref_z30x20_cut | **RUNNING** (oracle 136.1038/87f/1s/OCCT-INVALID) | **NOTRUN** (oracle 8.9750/43f/4s) | **NOTRUN** (oracle empty/0f/0s) |
| bal_055 | occtref_x13y29_common_s0 | occtref_x20_fuse | **RUNNING** (oracle 7.2290/39f/3s) | **NOTRUN** (oracle 24.0310/32f/1s) | **NOTRUN** (oracle 35.5250/68f/1s) |
| bal_056 | occtref_y30_common_s0 | occtref_z63_cut_s0 | **TIMEOUT** (oracle 13.1840/37f/2s) | **NOTRUN** (oracle 19.9375/46f/3s) | **NOTRUN** (oracle 75.7904/79f/1s) |
| bal_057 | occtref_z63_fuse | rotB_z30 | **NOTRUN** (oracle 88.7219/87f/1s) | **NOTRUN** (oracle 54.1976/63f/2s) | **NOTRUN** (oracle 169.0185/101f/1s) |
| bal_058 | occtref_z15_cut_s0 | rotB_y30 | **TIMEOUT** (oracle 47.6923/64f/2s) | **NOTRUN** (oracle 32.5768/51f/2s) | **NOTRUN** (oracle 127.9893/78f/1s) |
| bal_059 | occtref_x20_cut | chair1 | **TIMEOUT** (oracle 46.7930/48f/1s) | **NOTRUN** (oracle 33.5034/42f/1s) | **NOTRUN** (oracle 127.0902/63f/1s) |

## 12. What this corpus does and does not cover

Two properties of the delivered corpus materially shape what could be concluded, and both
were measured rather than assumed.

**The chair family is freeform, not planar.** `chair0.stp` contains 20 `ADVANCED_FACE`
entities, every one of them a `B_SPLINE_SURFACE_WITH_KNOTS` of degree (3,3), with up to
18x21 control points; sampled face normals swing by as much as 1.49 rad across a single
face and principal curvatures reach 0.49. There is not one analytic surface in the file.
Every solid derived from the chairs -- the 10 rotated operands and all 34 usable OCCT
reference booleans -- inherits that. So the hard and balanced corpora are 100 %
freeform-x-freeform and contain no planar, analytic or seam-bearing geometry at all.

| source family | solids | faces | surface types | max planar deviation of a face |
|---|---:|---|---|---:|
| OCCT-authored primitives | 5 | 1-6 | Plane x9, Cone x1, Cylinder x1, Sphere x1, Toroid x1 | 0.0000 |
| chair0 / chair1 (the base joint operands) | 2 | 20-20 | BSplineSurface x40 | 2.0220 |
| ffbox_* (the freeform blob file) | 2 | 1-6 | Plane x6, BSplineSurface x1 | 0.0000 |
| occtref_* (OCCT boolean results of rotated chairs) | 34 | 4-68 | BSplineSurface x1076 | 2.0220 |
| rotB_* (rotated chair operands) | 10 | 20-20 | BSplineSurface x200 | 2.0220 |

**The difficulty ranking excluded every primitive.** The generator's score is
`curved*2 + (1-degeneracy)*3 + faces/40`, which the 46 chair-family solids dominate
(~8.75 versus ~4.7 for a primitive pair), so the top-80 selection contains 29 distinct
operands, all of them `occtref_*` or `rotB_*`. The analytic primitives, the freeform blob
and the planar boxes that were harvested never entered a single pair. Corpora C through F
in this document were built to cover those axes, since otherwise categories (i), (ii) and
(iii) of the requested census would have had zero data.

## 13. Proposed regression corpus

These are ordered by what they would unblock, and every one runs in seconds, which is what
makes them usable as a regression set. Directories already exist under
`/home/petras/xp_work/` and each is in the `chair0.stp` / `chair1.stp` layout that
`SESSION_CHAIRS` consumes, so no harness work is needed.

| # | case | directory | what it pins |
|---|---|---|---|
| 1 | `dg/ax3` + `dg/sc0.1` `dg/sc1` `dg/sc10` `dg/sc100` | `xp_work/dg/` | corner-overlap detection; the scale sweep makes a tolerance regression impossible to hide |
| 2 | `dg/inside` | `xp_work/dg/inside` | the one case that works today -- lock it, plus the void-on-export defect |
| 3 | `nb/boxbox_analytic` vs `nb/boxbox_nurbs` | `xp_work/nb/` | face stitching, and that analytic and NURBS inputs stay equivalent |
| 4 | `dg/ax1` `dg/ax2` `dg/through` `dg/rot37` | `xp_work/dg/` | one-, two- and three-axis crossings and a rotated operand |
| 5 | `dg/tet` | `xp_work/dg/tet` | non-axis-aligned planar faces; currently the worst silent error in the sweep |
| 6 | `spair_009` (box x sphere), `spair_045` (box x cylinder), `spair_050` (box x torus), `spair_005` (box x cone) | `xp_work/supp/` | ingestion and booleans for each analytic surface type with its seam |
| 7 | operand-ingestion assertions on `sph`, `cyl`, `cone`, `tor`, `blob` | `xp_work/supp/_prims` equivalents | `is_solid` and `volume()` before any boolean -- these fail today and are cheap to assert |
| 8 | `spair_002`, `spair_033` (blob x box) | `xp_work/supp/` | periodic seam + two poles |
| 9 | `pvpair_*` | `xp_work/pv/` | consuming another kernel's trimmed output at a size that finishes |
| 10 | `spair_052` (box x chair0), `spair_065` (box x chair1) | `xp_work/supp/` | the silent disjoint-miss on the chair geometry itself -- `cut` returns operand A unchanged and reports a closed valid solid |
| 11 | `pair_063` with and without `SESSION_FAST=1` | `/home/petras/xpairs/` | asserts that `BRep::volume()` returns at all on a 32-face foreign NURBS solid, and that the cut it currently hides is closed |
| 12 | `bal_000`, `bal_002` operand load only | `/home/petras/xpairs/` | `is_solid` on `occtref_x20_fuse` and `occtref_z30_fuse`, which OCCT reads as closed valid solids and we do not |

The control pair (`xp_work/ctl/chairs`) must stay in the set as the canary: if it ever
stops matching OCCT to 1e-5, a fix aimed at the cases above has regressed the one
configuration that currently works.

