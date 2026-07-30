# TIER 3a — the freeform difficulty ladder, both kernels

Session: T3a freeform. Driver: `main_22.cpp` (the only file I own; nothing under `src/`, no
`CMakeLists.txt`, no other `main_*.cpp` was touched). Build dir: `build_ff/` (my own).
Scoring: **only** `src/v2/v2_verdict.h` (`v2v::v2_verdict`) — no verdict code of my own.
Independent oracle: FreeCAD 1.1.1 / OCCT 7.8 headless, plus a tessellation integrator that is
independent of *both* kernels' volume code.

---

## 1. THE ANSWER, IN ONE SENTENCE

> **Both kernels first break at L1 — freeform per se.** The L0 planar control is exact for both
> (48 / 16 / 72, `closed=1`, residual 0), and the *first* rung that fails is the multi-face
> B-spline solid with **no seam and no pole**: the current kernel returns an open shell with
> **10 naked edges** and a cut volume of **16.8303** against OCCT's **25.4724** (−34 %), and v2
> returns an open shell with **19 naked edges** whose answer is not even mirror-symmetric on a
> mirror-symmetric input (cut = **71.09** at `+(1.3,0.7,0.4)` vs **13.11** at `−(1.3,0.7,0.4)`,
> an oracle-free proof of wrongness).

And the ladder does **not** get worse from there — it gets *better*, which is the second
headline:

> Seams and poles per se are **survivable**: the single periodic face with a seam and two poles
> (L4) at a generic box position is our **best** freeform cell — 0 naked edges, and its result
> solid is the **same solid OCCT produces to 0.003 %** (tessellated: ours 34.1643, OCCT's
> reference 34.1650 at the same tessellation deviation). What actually hurts is a **section
> loop that crosses the seam transversally** (L4o, L4s: 10–15 naked edges), not the existence
> of the seam (L2s, where the cutting plane *contains* the seam, is watertight to 2.8e−6).

So the implementers' order is: **(1) freeform multi-patch imprint (L1), (2) seam-crossing loop
closure, (3) nothing at the pole rung — poles are already fine.**

---

## 2. WHAT I CHANGED

* Created `main_22.cpp`. Modes: `probe` (one operand → one verdict line), `export` (write an
  operand to STEP for FreeCAD), `cell <v1|v2> <op> <A> <B>` (one boolean → one verdict line),
  `ssi <A> <B>` (per-face-pair surface–surface-intersection census). One cell per process, each
  wrapped in `timeout`, every completed cell ends in `T3DONE`.
* `CMakeLists.txt`'s target list stops at `main_17` and is owned by another session, so
  `main_22` is compiled and linked **by hand** with exactly the flags CMake emits for `main_17`
  in `build_ff` (`flags.make` + `link.txt`, driver object swapped).
* Created operands and scripts under `/home/petras/v2_freeform/` only.
* No file under `src/` was read-modified. Nothing was committed.

---

## 3. METHOD, AND EVERY REASON TO DISTRUST A NUMBER

**Binaries.** `session_core` + `session_v2` objects compiled **once**, 2026-07-26 16:39, from
HEAD `5bb685a` with a dirty working tree (other agents). Every cell number below comes from
driver sha1 **`3632028be5a62424d393a7a0b9f3f2f45ccecdc1`**; the later `ssi` diagnostics come
from `a3c71b54308fcd273a9d0b8ed3d66d2170faff01` — **only `main_22.cpp.o` differs between them,
the kernel objects are byte-identical**, so all kernel numbers are one coherent snapshot.
Files edited by other sessions *after* my 16:39 build and therefore **not** in any number here:
`src/brep.cpp` (17:10), `src/file_step.cpp` (17:07), `src/intersection.cpp` (17:02),
`src/v2/brep_v2_section.cpp` (16:51), `src/v2/v2_dump.cpp` (17:03).

**Environment.** No `SESSION_*` variable was set in the shell (verified with `env`). The only
env used are mine: `SESSION_T3_BUDGET` (default 100, see below), `SESSION_T3_REPORT=1` in run 2
(prints the v2 stage report; no behavioural change), `SESSION_T3_SAVE` for one result export.

**Quadrature budget.** `v2v::v2_verdict_options()` with `max_surface_evals` and
`min_face_evals` × 100 — the same raise `main_21` measured to be necessary, for the same
reason: at the default budget an intact operand can integrate several percent off with
`converged=0`, which measures the quadrature and not the kernel.

**Cells attempted vs completed.** run 1: **72 launched / 72 completed** (every log has both a
`T3DONE` and `EXIT=0`; zero timeouts, zero throws). run 2: **60 / 60**, same. Plus 6 one-off
diagnostic runs, all completed. **Zero cells failed to exit.**

**Control cell.** `L0 = box(4,4,4) × box(2,2,6)` is run in every sweep and is exact for both
kernels in all three ops (48.000000 / 16.000000 / 72.000000, analytic truth, `closed=1`,
residual 0.0e+00). The control never moved between runs.

**Trivially-exact cells are reported separately, never summed into successes.** `L1kb cut`
(the concentric box is entirely inside the pillow, so the true answer is the empty solid) is
the only such cell: both kernels return `EMPTY_RESULT`, which is correct, and it is excluded
from every count below.

**Two operand routes, on purpose.** A rung can fail because the STEP reader destroyed the
operand or because the boolean is wrong. So every rung was run twice: **in code** (`own:`,
the reader is not in the path at all) and **from STEP** (`step:`, FreeCAD/OCCT-authored).
**The in-code route carries the conclusion, because it is the one whose L0 control passes**
(§6 shows the STEP route's own L0 control fails for both kernels, so that route cannot
attribute anything above L0).

### 3a. A MEASURED CORRECTION: OCCT's own volume is wrong on periodic B-spline faces

This decides how L4 is graded, so it is established first, with an integrator that belongs to
neither kernel: tessellate the solid and sum signed tetrahedra (`tess_vol.py`, deviations
0.02 → 0.0025, inscribed so it converges from below).

| solid | our harness | OCCT `Shape.Volume` | tessellated 0.01 / 0.005 / 0.0025 | verdict |
|---|---|---|---|---|
| `sphere.step` (analytic control) | 65.449847 | 65.449847 | 65.4137 / 65.4391 / 65.4462 | both right (analytic 65.4498469) |
| `pillow.step` (non-periodic B-spline) | 80.469780 | 80.469667 | 80.3496 / 80.4191 / 80.4470 | both right |
| **`blob.step`** (periodic B-spline, 1 face) | **64.129200** | **64.589785** | 64.0279 / 64.0969 / **64.1168** | **ours right, OCCT +0.72 % high** |
| **L2 tube** (`dataset_ob/L2_seam_nopole/chair1.stp`) | **107.424331** | **109.160926** | 107.3818 / **107.4092** | **ours right, OCCT +1.6 % high** |

Both disputed cases are periodic B-spline faces; both analytic and non-periodic controls agree.
**Consequence: every OCCT reference volume for a cell whose operand carries a periodic B-spline
face is high by ~0.7–1.6 %, and the earlier kb tables that quote OCCT volumes as "true" for
`L2_*`, `L4_*` and the blob inherit that error.** Where it matters below I use the tessellation
oracle instead.

---

## 4. THE OPERANDS, AND THE PROOF THEY ARE GOOD

Built in code by this kernel, exported with `main_22 export`, then verified by FreeCAD
(`/home/petras/fc_inspect/validate_operands.py`, `FC_FILE=<step>`) **before** any of them was
used. All files live in `/home/petras/v2_freeform/`.

| operand | topology (ours) | our verdict | OCCT on our export |
|---|---|---|---|
| `box4` / `box226` / `box4_gen` / `box4_seam` / `box4_pole` / `box446_polein` | 6 planes, 12 edges | closed=1, resid 0, vol 64 / 24 / 64 / 64 / 64 / 96 | closed, valid, solid, vol identical |
| **`pillow`** (L1) | 6 bicubic patches, **seam 0 pole 0**, 12 shared straight edges | closed=1, resid 7.2e−18, vol 80.469780 | closed, valid, `{BSplineSurface: 6}`, vol 80.469667 |
| **`tube`** (L2) | 1 periodic B-spline + 2 planar caps, **seam 1 pole 0** | closed=1, resid 2.3e−17, vol 56.406267 | closed, valid, `uPeriodic=True`, vol 56.383702 |
| **`blob`** (L4) | 1 periodic B-spline face, **seam 1, poles 2** | closed=1, resid 2.3e−17, vol 64.129200 | closed, valid, deg (2,2), 8×5 poles, rational, uPeriodic, vol 64.589785 (see §3a) |
| `sphere` (L4x) | as blob, unperturbed | closed=1, vol 65.449847 = analytic | closed, valid, recognised as `Sphere` |

`validate_operands.py` reports the tube's seam and the blob's seam + 2 poles as "TRULY FREE"
because it counts `Face.Edges` (which de-duplicates); the periodicity print
(`uPeriodic=True uClosed=True`) and the two zero-length edges at `(0,0,±2.5)` identify them.
`isClosed() = True`, `isValid() = True`, `Part.Solid()` OK and positive volume for every one.

`L3` has **no in-code operand** — a pole-carrying multi-face sphere-topology solid cannot be
built without hand-assembling a BRep — so L3 is measured on the STEP route only (§6), with the
caveat that that route's control fails.

Cutting-plane variants per rung, as the brief asks: `_gen` = generic position
`@(1.3,0.7,0.4)` (nothing coincident), `_seam` = a box **face plane at y = 0, i.e. containing
the blob's/tube's seam meridian**, `_pole` = the box top face plane exactly at `z = 2.5`, **on
the pole**, `_polein` = a 4×4×6 box with **both poles strictly inside**, `o` = the shipped
`freeform_common_box` configuration (box and blob both centred, so the seam *crosses* a face).

---

## 5. THE LADDER (in-code operands — the route whose control passes)

`OURS` is `faces / solids / naked_real / volume / closed`, all from `v2v::v2_verdict`.
`naked_real` excludes degenerate edges; a seam is not naked. `closed` additionally requires the
mass-properties closure residual < 1e−9.

| rung | op | OCCT ref F/sol/vol | v1 (current kernel) | v2 |
|---|---|---|---|---|
| **L0** box × box | cut | 10/1/48.000000 | 10/1/**0**/48.000000/**1** | 10/1/**0**/48.000000/**1** |
| | common | 6/1/16.000000 | 6/1/**0**/16.000000/**1** | 6/1/**0**/16.000000/**1** |
| | fuse | 16/1/72.000000 | 16/1/**0**/72.000000/**1** | 16/1/**0**/72.000000/**1** |
| **L1** pillow × box gen | cut | 9/1/25.472406 | 9/0/**10**/16.830250/0 | 5/0/**19**/71.093169/0 |
| | common | 7/1/38.527583 | 7/0/**10**/47.169750/0 | 6/0/**19**/45.493169/0 |
| | fuse | 13/1/105.942100 | 11/0/**10**/97.300031/0 | 8/0/**18**/96.836183/0 |
| **L1x** mirror of L1 | cut | 9/1/25.472416 | 9/0/10/16.830250/0 | 7/0/**22**/**13.108475**/0 |
| | common | 7/1/38.527571 | 7/0/10/47.169750/0 | 6/0/22/50.891525/0 |
| | fuse | 13/1/105.942088 | 11/0/10/97.300031/0 | 8/0/22/93.578256/0 |
| **L1kb** pillow × concentric box | cut | empty (box ⊂ pillow) | EMPTY_RESULT ✔ | EMPTY_RESULT ✔ |
| | common | 6/1/64.000000 | **216**/0/0/**−0.148569**/0 | 6/1/**0**/64.000000/**1** ✔ |
| | fuse | 6/1/80.469667 | **204**/0/47/**−10.796152**/0 | 6/1/**0**/80.469780/**1** ✔ |
| **L1s** plane through the pillow's mid-plane | cut | 9/1/31.154827 | 10/0/7/29.841186/0 | identical to v1 |
| | common | 7/1/32.845144 | 17/0/7/34.148607/0 | identical |
| | fuse | 13/1/111.624457 | 24/0/15/110.314134/0 | identical |
| **L2** tube (seam, no pole) × box gen | cut | 8/1/37.040698 | 6/0/**6**/37.154278/0 | identical |
| | common | 6/1/26.900909 | 5/0/**6**/26.845546/0 | identical |
| | fuse | 9/1/93.508699 | 8/0/**6**/93.560544/0 | identical |
| **L2s** box face plane **contains the seam** | cut | 8/1/43.889441 | 8/0/**0**/43.882050/0 (resid 2.8e−6) | identical |
| | common | 4/1/20.110485 | 4/0/**0**/20.117950/0 (5.8e−6) | identical |
| | fuse | 10/1/100.395474 | 10/0/**0**/100.288317/0 (2.1e−6) | identical |
| **L4** blob (seam + 2 poles, ONE face) × box gen | cut | 7/1/29.599604¹ | 7/0/**0**/29.830726/0 (1.0e−5) | identical |
| | common | 5/1/34.394091¹ | 5/0/**0**/**34.169261**/0 (1.6e−5) | identical |
| | fuse | 8/1/94.115409¹ | 8/0/**0**/93.959926/0 (8.1e−6) | identical |
| **L4s** box face plane **contains the seam** | cut | 10/1/35.693280 | 7/0/**10**/56.403977/0 | 4/0/**15**/55.466667/0 |
| | common | 6/1/28.311936 | 5/0/**10**/7.596023/0 | 2/0/**15**/8.533333/0 |
| | fuse | 11/1/100.163230 | 9/0/**10**/120.533177/0 | 5/0/**15**/119.595867/0 |
| **L4p** box face plane **on the pole** | cut | 11/2/15.893190 | 11/0/**0**/15.849549/0 | identical |
| | common | 6/1/48.106359 | 6/0/**0**/48.096699/0 | identical |
| | fuse | 12/1/80.214777 | 12/0/**0**/80.032393/0 | identical |
| **L4pi** both poles **inside** the box | cut | 7/1/40.513975 | 7/0/**0**/41.478664/0 | identical |
| | common | 4/1/55.522677 | 4/0/**0**/54.521336/0 | identical |
| | fuse | 9/1/105.228359 | 8/0/**4**/89.607865/0 | identical |
| **L4o** the shipped `freeform_common_box` config | cut | 7/1/5.487351¹ | 8/0/**1**/15.102490/0 | 3/0/**6**/32.000000/0 |
| | common | 5/1/58.525194¹ | 6/0/**3**/46.968719/0 | 3/0/**6**/32.000000/0 |
| | fuse | 11/1/70.003399¹ | 14/0/**1**/79.231697/0 | 4/0/**6**/96.129200/0 |
| **L4x** exact NURBS sphere × box gen | cut | 9/1/29.264559 | 7/0/**6**/6.125462/0 | identical |
| | common | 6/1/34.735441 | 5/0/**0**/34.736470/0 | identical |
| | fuse | 9/1/94.714406 | 8/0/**6**/71.575309/0 | identical |
| **L5** pillow × blob (freeform × freeform) | cut | 11/2/25.719933¹ | 14/0/**0**/32.847837/0 | identical |
| | common | 6/1/54.778322¹ | 18/0/**6**/51.372387/0 | identical |
| | fuse | 12/1/90.227036¹ | 20/0/**0**/93.222656/0 | identical |

¹ OCCT reference volume is high by ~0.7 % on these cells (periodic B-spline operand, §3a).

### 5a. Reading the ladder

* **L0 passes, L1 fails, in five independent constructions.** In-code pillow at a generic
  position and at its mirror image, in-code pillow concentric, the FreeCAD-authored pillow
  (`S_L1b`, §6) and the FreeCAD-authored 6-patch lofted solid (`S_L1`, §6) — every one fails.
  L1 is where both kernels break.
* **Oracle-free proof for v2 at L1.** The pillow and the box are both centrally symmetric, so
  `box@+(1.3,0.7,0.4) × pillow` and `box@−(1.3,0.7,0.4) × pillow` are mirror images and must
  have equal volumes. OCCT: 25.472406 vs 25.472416. v1: 16.830250 vs 16.830250 (symmetric,
  consistently wrong). **v2: 71.093169 vs 13.108475** — a factor of 5.4 apart. No reference is
  needed to call that broken.
* **Oracle-free proof for v1 at L1** (STEP route, §6, same defect): `S_L1b common = 67.914335`
  against `vol(A) = 64.000000`. **An intersection cannot exceed an operand.**
* **The rungs above L1 are not monotonically worse — they are better.** L4 (one periodic face,
  seam + two poles) at a generic box position produces `naked = 0` in all three ops, and §5b
  shows its *solid* matches OCCT's to 0.003 %. L4p (plane exactly on a pole) and L4pi (both
  poles inside) are likewise `naked = 0` in 5 of 6 ops. **Poles are not the problem.**
* **The seam is only a problem when a section loop crosses it.** L2s and L4s put a box face
  plane *in* the seam: L2s is the cleanest freeform cell in the whole table (naked 0; volumes
  1.7e−4 / 3.7e−4 / 1.1e−3 relative of OCCT for cut / common / fuse, and OCCT is itself high on
  this operand, §3a), L4s is one of the worst (naked 10/15). The difference between them is
  that at L4s the *other* box faces still cut loops that run across the seam. The shipped
  `L4o` configuration is the same failure and reproduces at 46.968719 (kb `hunt_freeform`
  measured 47.1447 on an older binary).
* **v1 and v2 are the same kernel on freeform.** 46 of the 66 cells are **byte-identical**
  between the two kernels — same face count, same naked count, same volume to 6 decimals. They
  differ only at L1 (all configs), L4s, L4o, `S_L0` and `S_L1b`. Where they differ, **v2 is
  better exactly once** (L1kb, where v1 emits a 216-face shell with negative volume and v2 is
  exact and `closed=1`) and worse in every other case. The v2 stage report confirms v2 really
  runs its own pipeline (`pairs=0 sdsurf=0 sec=7 fA=7 fB=7 inA=2 inB=4 sel=5` on L1 cut), so
  the identity is not a fallback: **the defect is upstream of everything v2 replaces.**

### 5a-bis. The partition identity, and where it is blind

`vol(A cut B) + vol(A common B) = vol(A)` needs no reference. Measured on our own results
(`vol(A)` is a box and is exact for both kernels):

| rung | v1 sum / residual | v2 sum / residual | OCCT sum / residual |
|---|---|---|---|
| L0 | 64.0000 / +0.0000 | 64.0000 / +0.0000 | 64.0000 / +0.0000 |
| **L1** | **64.0000 / +0.0000** | **116.5863 / +52.5863** | 64.0000 / −0.0000 |
| L1kb | −0.1486 / −64.1486 | 64.0000 / +0.0000 | 64.0000 / +0.0000 |
| L1s | 63.9898 / −0.0102 | 63.9898 / −0.0102 | 64.0000 / −0.0000 |
| L2 / L2s | 63.9998 / −0.0002 · 64.0000 / +0.0000 | same | 63.9416 / −0.0584 · 63.9999 |
| L4 / L4s | 64.0000 / −0.0000 both | same | 63.9937 · 64.0052 |
| L4p | 63.9462 / −0.0538 | 63.9462 / −0.0538 | 63.9995 / −0.0005 |
| L4pi | 96.0000 / +0.0000 | 96.0000 / +0.0000 | 96.0367 / +0.0367 |
| **L4x** | **40.8619 / −23.1381** | same | 64.0000 / +0.0000 |
| L5 | 84.2202 / +3.7504 | 84.2202 / +3.7504 | 80.4983 / +0.0285 |

**The identity catches v2 at L1 (+52.6) and both kernels at L4x (−23.1) and L5 (+3.75) — but it
is exactly 0 for v1 at L1, where v1 is 34 % wrong.** v1 partitions the box consistently and
then assigns the material to the wrong side, so a partition gate alone will not catch it. The
mirror test (§5a) does. Use both.

### 5b. At L4 our result is the same solid as OCCT's — proven without either integrator

`SESSION_T3_SAVE` wrote our `L4 common`; `reference_cut.py FC_SAVE` wrote OCCT's. Both were
tessellated with the same deviations and integrated by signed tetrahedra:

| tessellation deviation | ours | OCCT reference |
|---|---|---|
| 0.01 | 34.124487 | 34.129387 |
| 0.005 | 34.154652 | 34.156238 |
| 0.0025 | **34.164310** | **34.165042** |
| the integrator's own claim | 34.169261 (our harness) | **34.386374** (OCCT `Volume`) |

The two solids agree to **0.0007 (0.002 %)**, and our harness's number (34.169261) is the one
consistent with the tessellation; OCCT's `Volume` for its *own* result is 0.65 % high. So at
L4-generic **both kernels are correct to ~0.01 %** and only the 1.6e−5 closure residual keeps
`v2v::closed()` from certifying it.

---

## 6. THE STEP ROUTE — and why it cannot carry the conclusion

Same rungs, operands authored by FreeCAD/OCCT (`/home/petras/hunt_freeform/dataset_ob/`,
built and gate-verified by the earlier `hunt_freeform` session).

**Reader fidelity first** (`main_22 probe`, operand only, no boolean). Truth is the tessellation
oracle where OCCT and we disagree (§3a):

| operand | our read | truth | verdict |
|---|---|---|---|
| `L0_box_box` B | 6 F, closed=1, 64.000000 | 64.000000 | exact |
| `L1b_pillow` B | 6 F, seam 0, closed=1, 76.576088 | 76.575864 | exact (2.9e−6) |
| `L1_noseam_nopole` B | 6 F, closed=1, 107.428409 | 107.429554 | exact (1.1e−5) |
| **`L2_seam_nopole` B** | **3 F, seam 1, closed=1, 107.424331** | **≈107.42 (tessellated)** | **exact — the reader has been FIXED since `hunt_freeform` measured 2.0404 here** |
| `L3_poles_noseam` B | 2 F, naked 0, 65.369077, **resid 6.4e−4 → closed=0** | 65.3855 / ≈65.39 | volume right, closure certificate fails |
| `L4_seam_poles` B, `L4x`, `L5` B | **1 F, 0 edges**, `is_solid=0` | — | **single periodic face still loses all topology on read** |

**The STEP route's own L0 control FAILS**, so nothing above it can be attributed on this route:

| cell | op | OCCT | v1 | v2 |
|---|---|---|---|---|
| `S_L0` box × box (offset) | cut | 9/1/36.379000 | 8/0/**6**/31.292333/0 | 7/0/**12**/11.273000/0 |
| | common | 6/1/27.621000 | 7/0/**6**/32.707667/0 | 4/0/**12**/11.273000/0 |
| | fuse | 12/1/100.379000 | 12/1/**0**/100.379000/**1** | 14/0/**8**/75.273000/0 |
| `S_L1b` FreeCAD pillow | cut | 12/1/4.761896 | 12/0/0/3.914335/0 | 6/0/12/70.288044/0 |
| | common | 12/1/59.238119 | 9/0/0/**67.914335** > vol(A)=64 | 6/0/12/70.288044/0 |
| | fuse | 12/1/81.337693 | 9/0/0/72.661753/0 | 6/0/12/70.288044/0 |
| `S_L1` 6-patch lofted | cut | 18/4/1.063837 | 12/0/10/34.386948/0 | identical |
| | common | 10/1/62.936172 | 4/0/10/29.613028/0 | identical |
| `S_L1s` plane at y=0 | common | 7/1/38.682143 | 2/0/7/11.074332/0 | identical |
| `S_L2` seam operand | cut | 18/4/1.063148 | EMPTY_RESULT | EMPTY_RESULT |
| | common | 10/1/62.936790 | 6/1/**0**/64.000000/**1** — *closed but WRONG: it is A whole* | identical |
| | fuse | 17/1/109.905522 | 3/0/0/107.418292/0 (= B alone) | identical |
| `S_L2s` | cut | 6/1/25.314618 | 6/1/**0**/64.000000/**1** — *closed but WRONG: A whole* | identical |
| | fuse | 8/1/132.876381 | 9/0/0/171.418292/0 (= 64 + 107.4, disjoint) | identical |
| **`S_L3` poles, no seam, 2 faces** | cut | 8/1/10.228673 | 6/0/**5**/50.448383/0 | identical |
| | common | 8/1/53.828724 | 3/0/**5**/13.551575/0 | identical |
| | fuse | 16/1/75.473279 | 9/0/**12**/92.174657/0 | identical |
| `S_L3pi` poles inside | common | 6/1/57.462483 | 2/0/**4**/8.914798/0 | identical |

Two things are worth carrying forward from this route even though its control is broken:

* **`closed = 1` is not sufficient evidence of a correct boolean.** `S_L2 common` and
  `S_L2s cut` both return `6 faces / 1 solid / 0 naked / 64.000000 / closed=1` — a perfect
  verdict for a result that is simply **operand A unchanged** (truth 62.94 and 25.31). Any gate
  built on `closed()` alone will pass these. Volume against a reference, or the partition
  identity, is mandatory.
* **The L3 rung (poles, no seam, multiple faces) fails badly on this route** (5 naked, common
  13.55 vs 53.83) — but its operand's own closure certificate already fails (6.4e−4), and the
  route's control fails, so this is *not* evidence that poles break the boolean. The in-code
  pole cells (L4p, L4pi) say the opposite.

---

## 7. WHAT THE SSI IS ACTUALLY DOING (mechanism, measured)

`main_22 ssi` runs `Intersection::surface_surface` on every face-pair and prints curve counts,
lengths and endpoints; `/home/petras/v2_freeform/ssi_ref.py` prints OCCT's `Face.section` for
the same pairs.

* **The pair sets agree exactly.** L1 (`box4_gen × pillow`): ours 7 non-empty pairs of 36,
  OCCT 7 — and they are the *same seven* `(0,3) (0,4) (1,1) (2,1) (2,3) (5,1) (5,4)`. L4
  (`box4_gen × blob`): ours 4 of 6, OCCT 4 of 6, same four. **The "our SSI drops half the
  plane × patch pairs" finding of `hunt_freeform` §1b does NOT reproduce on these operands** —
  at least not at a generic box position. Retract it as a general statement.
* **What differs is the extent: our section curves are not clipped to the cutting face.**
  Pair `(a=0, b=3)` at L1: A-face 0 is the plane `z = −1.6` spanning `x∈[−0.7,3.3]`,
  `y∈[−1.3,2.7]` (printed by `T3SURF`); our curve runs `(2,−2,−1.6) → (2,2,−1.6)`, i.e. from
  `y = −2`, **0.7 outside the face's own boundary**. OCCT's section of the same pair is
  length 3.307212 against our 4.010470 — and 3.307/3.3 = 1.002 = 4.010/4.0, so the excess is
  *exactly* the piece of the same curve beyond the face's `y` edge. Same at L4: our
  `(0,0)` curve is the whole 12.054861-long closed loop on the plane, OCCT's is the two arcs
  (5.012348) that lie inside the box face.
* This is a real property of the imprint and it is the natural suspect for L1's unclosed loops,
  but it is **not sufficient on its own** to explain the ladder: L4 has the same overshoot and
  still closes. State it as measured, not as the root cause.

---

## 8. WHAT TO FIX, IN ORDER

1. **The L1 imprint — multi-patch freeform.** Both kernels; 5 independent constructions; the
   cheapest regression cells are `main_22 cell v1 cut own:box@1.3,0.7,0.4 own:pillow` (must
   give 25.4724, naked 0) and its mirror `own:box@-1.3,-0.7,-0.4` (must give the **same**
   number — v2 is currently 5.4× apart between them). Both need no oracle.
2. **v2 at L1 specifically**: the mirror asymmetry is a bug that can be found without any
   reference solid, and v2's L1 result (19–22 naked edges) is *worse* than v1's (10). Whatever
   v2 does differently at the multi-patch imprint is a regression against the current kernel
   and blocks the "v2 ≥ current on every pair" kill criterion on this class.
3. **Seam-crossing loop closure** (L4s, L4o): 10–15 naked edges when a section loop crosses the
   seam transversally, 0 when the cutting plane contains it (L2s) and 0 when the loop stays off
   the seam (L4). This is `hunt_freeform` §8 item 5 and it is confirmed, but it is item 3 here,
   not item 1.
4. **Do not spend effort on poles.** L4p (plane exactly on the pole) and L4pi (both poles
   inside) are `naked = 0` in 5 of 6 ops, and the sole failure there is `L4pi fuse`.
5. **`L4x cut` (the exact NURBS sphere) is much worse than the perturbed blob** — 6 naked and
   6.125462 against 29.264559, while `L4x common` is exact to 3e−5 and the *perturbed* blob is
   fine in all three ops. A recogniser/analytic path is being taken for the exact sphere that
   the freeform path handles correctly. Cheap A/B: `own:sphere` vs `own:blob:0.001`.
6. **The single periodic face still does not survive the STEP reader** (`1 face, 0 edges,
   is_solid=0`), while the seam-only tube now reads exactly. That is the remaining reader
   defect and it is the reason L4/L5 cannot be tested on the STEP route at all.
7. **Grading rule for everyone**: never grade a periodic-B-spline cell against OCCT's
   `Shape.Volume` (§3a: +0.72 % on the blob, +1.6 % on the L2 tube). Use `tess_vol.py`, or the
   partition identity, or per-face sampling.

---

## 9. REPRODUCTION

```bash
ROOT=/home/petras/code/code_rust/session/session_cpp
# configure my build dir (protobuf/absl sources reused from build_k, no network)
cmake -S $ROOT -B $ROOT/build_ff -DCMAKE_BUILD_TYPE=Release \
  -DFETCHCONTENT_SOURCE_DIR_PROTOBUF=$ROOT/build_k/_deps/protobuf-src \
  -DFETCHCONTENT_SOURCE_DIR_ABSL=$ROOT/build_k/_deps/absl-src -DFETCHCONTENT_FULLY_DISCONNECTED=ON
cmake --build $ROOT/build_ff --target main_17 -j24      # builds session_core + session_v2
bash /home/petras/v2_freeform/scripts/build22.sh         # compiles+links main_22 by hand

cd $ROOT/build_ff
./main_22 probe own:pillow                              # operand verdict
./main_22 export own:blob /home/petras/v2_freeform/blob.step
./main_22 cell v1 cut "own:box@1.3,0.7,0.4" own:pillow  # L1, current kernel
./main_22 cell v2 cut "own:box@-1.3,-0.7,-0.4" own:pillow  # L1 mirror, v2
./main_22 ssi "own:box@1.3,0.7,0.4" own:pillow          # per-pair SSI census

# oracles (snap confinement: every path under /home/petras, passed by env var)
FC_FILE=/home/petras/v2_freeform/pillow.step /snap/bin/freecad.cmd /home/petras/fc_inspect/validate_operands.py
FC_LIST=/home/petras/v2_freeform/pairs.txt /snap/bin/freecad.cmd /home/petras/v2_freeform/refs.py
FC_FILE=/home/petras/v2_freeform/blob.step /snap/bin/freecad.cmd /home/petras/v2_freeform/tess_vol.py
FC_A=.../box4_gen.step FC_B=.../pillow.step /snap/bin/freecad.cmd /home/petras/v2_freeform/ssi_ref.py
```

Sweeps: `scripts/sweep.sh <outdir> <jobs> <timeout_s>` (12 cells × 2 kernels × 3 ops = 72) and
`scripts/sweep2.sh` (10 × 2 × 3 = 60); `scripts/table.py <outdir>` renders the ladder table and
the partition block. Everything is under `/home/petras/v2_freeform/`: operands (`*.step`),
oracle scripts (`refs.py`, `tess_vol.py`, `ssi_ref.py`, `mc_blob.py`), reference logs
(`refs.log`, `refs2.log`), build/sweep scripts (`scripts/`) and **all 132 raw per-cell logs**
(`logs/run1/`, `logs/run2/`).
