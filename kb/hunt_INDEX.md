# hunt_INDEX — ranked defect report across five isolated investigations

**Date:** 2026-07-26 · **Sources:** `hunt_occt_interop.md`, `hunt_oriented_primitives.md`,
`hunt_freeform_class.md`, `hunt_determinism.md`, `hunt_efvf_gap.md` (five hunts run without
knowledge of each other) · **Ranking axis:** how much each finding blocks *a valid solid boolean
B-Rep algorithm*. Not by interest, not by depth, not by how hard it was to find.

**Owner key:** **A** = kernel src (`src/`), **B** = same-domain / coincidence subsystem
(`brep_samedomain`, `brep_commonblock`), **C** = validation / corpus / harness.

---

## 0. The one thing that changed while writing this

Two hunts found the same defect from opposite ends and neither knew it. `hunt_freeform_class`
root-caused `src/file_step.cpp:1191-1193` and `:1473-1476` (a STEP `FACE_BOUND` entity is read as an
*inner/hole* loop, because `is_outer` is set only by `FACE_OUTER_BOUND`). `hunt_occt_interop`
independently reported, as its flagship symptom, that chair0 re-authored by OCCT's writer with zero
geometric change reads as **vol 19.2149 instead of 80.3011**, and built the control for it
(`/home/petras/hunt_interop/mkfob.sh` + 16 rewritten files in `/home/petras/hunt_interop/fob/`,
timestamped 23:52–23:53) — **and never ran it**. Its report does not contain the string `FACE_BOUND`.

Measured here, just now:

```
file                          bound entity          our reader
rd2/chair0_occt/chair0.stp    20x FACE_BOUND        faces 20 solid 1 vol 19.2149
   same file, sed -> FOB      20x FACE_OUTER_BOUND  faces 20 solid 1 vol 80.3011   <- the correct value
C1/chair0.stp (Rhino)         20x FACE_OUTER_BOUND  (the corpus original, 80.3011)

Q02  A = OCCT cylinder, B = OCCT box     (OCCT truth: A 28.274334, B 24.000000)
   as written   A vol  9.4269 (= 28.2743/3 EXACTLY)   cut 2f/open/naked 8/0.0000   common 5f/open/9.4248
   FOB rewrite  A vol 28.2743 (exact)                 cut 10f/open/naked 21/3.4812 common 13f/open/11.1608
   OCCT                                                cut 9f/closed/6.194964      common 8f/closed/22.079370
```

Binary `build/main_7_interop` md5 `2d4625f01c60c23ed21dcb403ed4a119`; flags
`SESSION_NO_ROT=1 SESSION_OP=<op> SESSION_CHAIRS=<dir>`, `ulimit -v 4194304`; N=1 (defensible for a
non-AUTO cell per `hunt_determinism`); the as-written value reproduces interop's 19.2149 digit for
digit, which pins binary and flags to theirs.

Three consequences, all load-bearing for everything below:

1. **Every OCCT/FreeCAD-authored operand in the interop hunt is 100% `FACE_BOUND`, 0% `FACE_OUTER_BOUND`**
   (verified across `prims/*.step`: box, box2, cbox, chbox, chcyl, chsph, con, cut1, cut2, cut3, cyl,
   far, sph, tor — 14/14 — and Q01–Q05, P01–P03). The `0 of 21` matrix and the "reader is
   dialect-dependent" verdict were both measured through a 10-line reader bug.
2. **"`volume()` on imported breps is wrong by 0.082x–4.7x, cylinder exactly 1/3 = integrated as a
   cone" is not a `volume()` defect.** The same cylinder reads 28.2743 exactly once the loop is
   labelled correctly. This re-attribution matters because `hunt_oriented_primitives` independently
   proved `volume()` exact on untrimmed primitives (box 64.000000000000, sphere 65.449846949788,
   cylinder 42.411500823462) and rigid-motion invariant to 1e-15. The two hunts are now consistent.
3. **The fix is necessary and not sufficient.** After the rewrite Q02's operands are exact and the
   result gets structurally closer (cut 2 faces -> 10, against OCCT's 9), but it is still open
   (naked 21) and still ~2x wrong. So expect the interop matrix to **reclassify**, not to repair:
   ingest failures become genuine boolean failures that can finally be diagnosed as such.

---

## 1. The ranked list

| # | defect | blocks | owner | cost of the next experiment |
|---|---|---|---|---|
| 1 | Outer-bound decision in the STEP reader | all foreign geometry + the whole authored corpus | A + C | done for the read; hours for the matrix |
| 2 | Kernel cannot decide validity (`is_solid`, naked, volume on results) | convergence on "valid" at all | A + C | ~1 day, no oracle |
| 3 | Assembly closure collapses as a chaotic function of *relative* pose | the 25 open rotated cells (the target class) | A | minutes/run, exact free oracle |
| 4 | Seam + pole handling, reader and splitter | ~half the legitimate topology space; likely the mechanism of #3 | A | hours |
| 5 | Freeform SSI branch discovery + junction continuation | correctness on any curved input | A | ~2 min for the decisive test |
| 6 | Classification keeps the wrong side / sees no interference | correct-not-merely-closed results | A | hours, oracle-free |
| 7 | Liveness: hangs and `bad_alloc` | a kernel must terminate; silently corrupts the ledger | A | 1 run with counters |
| 8 | Cross-session ledger is not comparable | knowing whether any change helped | C | protocol, no code |
| 9 | Export lossiness, multi-solid truncation, chain depth 2 | external verification + chained workflows | A + C | ~1 h, no oracle |
| 10 | EF/VF paving absence — **DEMOTED** | little; the panel's attribution is refuted | A | none; stop sizing on it |
| 11 | Latent nondeterminism hazards — **gated off** | nothing today; would block later | A | none; do not promote the flag |

---

### D1 — The STEP reader labels a face's outer loop as a hole
**Blocks:** every OCCT/FreeCAD/AP214-conformant file in existence, therefore the entire authored
validation corpus, therefore the grading of everything else. **Owner: A** (10 lines) **+ C** (re-measure).

**Evidence.** `src/file_step.cpp:1191-1193` and `:1473-1476`: `is_outer = has("FACE_OUTER_BOUND")`,
so a plain `FACE_BOUND` falls through to `BRepLoopType::Inner` at `:1333` / `:1643`.
`FACE_OUTER_BOUND` is an *optional* AP214 subtype; OCCT and FreeCAD write plain `FACE_BOUND`
universally. Rhino writes `FACE_OUTER_BOUND`, which is the only reason the chairs corpus escapes.
A third path, `read_file_step_nurbssurfaces_trimmed` (`:2190-2191`), does `if (!is_outer) continue;`
— such a face gets **no outer loop at all**.

Measured by `hunt_freeform_class` (FreeCAD box x cylinder): as written cut 3f/solid 0/naked 5/21.5032,
common 7/0/5/21.1634, fuse 2/0/5/21.3900; after the rewrite 7/1/0/**31.8301**, 3/1/0/**32.1699**,
10/1/0/**96.1699** — an exact OCCT match on all three ops. Measured here (section 0): the interop
hunt's flagship 19.2149 -> 80.3011, and the "cylinder integrates as a cone" artefact disappears.

**Cheapest next experiment.** The inputs already exist and have never been run: re-run the Q/P
primitive matrix and the P0x reference-operand cells against `/home/petras/hunt_interop/fob/` and
`/home/petras/hunt_freeform/dataset_ob/`. Zero new code. Then fix it properly — decide outer vs inner
**geometrically from 2D loop areas** as OCCT does, not from the entity name, so files with several
`FACE_BOUND`s per face (which the sed rewrite must skip) also work.

**Do not conclude from this that the interop verdict was wrong.** It was measured through a defect,
so its numbers must be retaken; the underlying claim ("we cannot consume foreign B-Reps") survives
my Q02 run, which is still open and ~2x off with exact operands.

---

### D2 — The kernel cannot decide whether its own output is a valid solid
**Blocks:** the goal directly. You cannot converge on a property your instrument cannot detect, and
this one reports success on demonstrable failure. **Owner: A** (predicates) **+ C** (verdict columns).

**Evidence, from three independent hunts.**
- `is_solid()` returned 1 for every reference solid OCCT counts **29–61 naked edges** on; the kernel
  printed `naked 0` for P09/P10 results OCCT measures at 48 and 30 (`hunt_occt_interop`).
- Chain depth 2 reports `faces 55 solid 1 naked 0 vol 49.4699`; OCCT reads the written file as **two
  untouched operands** (35f/46.792853 + 20f/80.296531). A self-report of "clean" on a no-op.
- `L1b_pillow` returns three watertight, `is_solid`-1, OCCT-confirmed-closed results that are simply
  the wrong solids — with `vol(common) = 67.5412 > vol(box operand) = 64`, and
  `vol(cut)+vol(common) = 71.68` against `vol(A) = 64` (`hunt_freeform_class`).
- `is_solid()` fails outright on a legitimate single-face sphere (seam + poles) while a torus
  (two seams, no poles) passes: the breaking element is the **degenerate pole edge**, one trim.
- The AUTO ladder's `metric()` counted only 1-trim edges, so a 3-trim edge scored 0 and non-manifold
  shells were actively *preferred*. **This one has landed**: `src/brep.cpp:8256-8268` now counts
  `nt > 2` against a candidate, uncommitted in the working tree.
- Naked counting itself is metric-dependent: wire-occurrence over `Wire.OrderedEdges` with degenerate
  edges excluded gives naked 0 for sphere/cyl/cone/torus where ancestor counting false-positives
  3/1/1/2; OCCT's own valid x20 cut reports naked 3 under the one-adjacent-face metric
  (`hunt_efvf_gap` calibration). Shell `isClosed()` echoes the `CLOSED_SHELL` declaration and is
  worthless as a verdict.

**Cheapest next experiment.** Wire `hunt_freeform_class`'s three oracle-free invariants into the
harness — they caught the pillow with no reference at all, and one of them is the only check that
catches y30's closed-but-wrong answer:
```
vol(A cut B) + vol(A common B) == vol(A)
vol(A common B) <= min(vol(A), vol(B))
no face of a result may be identical to an input face when the inputs provably intersect
```
Plus: wire-occurrence naked counting, `is_solid()` exempting degenerate pole edges, and stop
double-counting faces that carry them (a two-face lune reads **exactly 2x** the truth, 130.8815 vs
65.4498 — integrating over the full periodic domain instead of the trimmed half).

---

### D3 — Section assembly loses closure and correctness as a *chaotic* function of relative pose
**Blocks:** the target class. This is the 25 open rotated cells' symptom, reproduced in the simplest
fully-analytic geometry with an exact closed-form oracle. **Owner: A** (`brep_section.cpp` /
`brep.cpp` assembly, seam-relative).

**Evidence** (`hunt_oriented_primitives`, all with an exact oracle: sphere r=2.5 at origin INTERSECT
cylinder r=1.5 h=6 through the centre, `vol = 4pi/3 (R^3 - (R^2-r^2)^1.5) = 31.939525311496` at every
relative orientation; OCCT holds it to <= 4.8e-08 across 4 axes x 6 angles):
- The input curves are **provably exact at every angle**: `n=2 [deg=2 cv=9 rat=1 len=4.712388980385]`
  twice, byte-identical from 0 deg to 20 deg — `3pi/2` to 12 digits — *including where the answer is
  41% wrong*. No recogniser work can fix this.
- Rigid-motion equivariance is exact (max rel deviation 8.8e-15 / 8.3e-15 / 5.1e-14 over 5 pairs x
  10 angles x 3 ops, both operands rotated). World pose is not the variable; **relative** pose is.
- Cliff, not ramp: exact (2.0e-10) at 0.36 deg, 7.64e-05 wrong at 0.37 deg, linear error inside a
  band (-1.31e-02/deg), snapping back to exact at 1.90 deg. Open shell (`solid=0`) from 5 deg;
  18.849829 vs 31.939525 (**rel 4.1e-01**) at 20 and 30 deg.
- **Scale-invariant** (x0.1 and x10 put the cliff at the same angle with the same relative errors to
  9 digits) — therefore angular/relative, not an absolute length tolerance.
- **Azimuth-controlled**: tilt about (0,1,0), preserving mirror symmetry about the seam plane, is
  clean at every angle (rel <= 4e-09); (1,0,0) is already 4.1e-01 at 0.30 deg; (1,1,0) is chaotic at
  0.01 deg resolution (0.44 bad, 0.46 good, 0.47 bad, 0.48 bad, 0.50 bad).
- The 1e-4 cluster in the shipped battery is **not a sampling constant**: the marcher constants
  (`intersection.cpp:1578`, `:1666`, `:4903-4904`) are never executed in those cells, whose sections
  are exact conics. It is a fixed **absolute** volume defect of 2–4e-03 over results in the 20–40
  range. cyl x cylR is antisymmetric to 2.2e-06 (one shared boundary displaced, partition identity
  holds); boxR x sph **violates** partition identity by 1.196e-03 (cut and common bounded by
  different geometry).

**Cheapest next experiment.** Adopt sphere-INTERSECT-cylinder-through-centre as a permanent gate —
exact at arbitrary angular resolution at **zero oracle cost** — then bisect once at 0.36 vs 0.37 deg
and diff the `[SCAF-V]` / section dumps between the two runs. Two runs of seconds each, and the two
inputs are provably identical in every way that should matter. Read-only pointers named by the hunt:
`src/brep_section.cpp:804-904` (`split_seams`), `src/brep.cpp:6091` ("A's seam vs B's seam, ~1e-2
apart"), `src/brep.cpp:4029` (`whole_seg` alias index tolerance 1e-2) — 1e-2-scale seam tolerances,
the right order of magnitude for a 0.37 deg cliff.

---

### D4 — Seam and pole topology: destroyed by the reader, mishandled by the splitter
**Blocks:** roughly half the legitimate topology space cannot even be loaded; and this is the leading
candidate mechanism for D3. **Owner: A.**

**Evidence** (`hunt_freeform_class`, reader fidelity, our in-memory volume vs truth):
analytic survives any topology (cyl seam 64.3398 exact; torus 2 seams 59.2176 exact) and
seam-free/pole-free NURBS is fine (pillow exact); but **NURBS + seam is broken** (cyl_nurbs 42.8985
vs 64.9289; tor_nurbs 177.6529 vs 59.3611; L2 tube 2.0404 vs 109.1609), **poles are broken
independently** (lune exactly 2x), and single-face seam+poles fails `is_solid()`. Consequence: every
L4 cell's `cut = A`, `common = 0`, `fuse = A+B` is the **operand guard firing**, not a boolean result.

Splitter side (L4, built in-code, bypassing the reader entirely — `main_7.cpp:516-620`): our blob
face is **correct** (32.7684 retained vs 32.5443 true = 100.7%; OCCT's 49.9662 is 154% and wrong),
and the defect is confined to the box faces at seam crossings. `x=+2` keeps 2.9260 of a true 5.8178 —
**a ratio of 0.5029, exactly one half** — and `x=+2` is the one box face the seam meridian pierces.
`z=-2` is dropped entirely. **All eight naked-edge endpoints across cut/common/fuse lie on the seam
plane `y=0`** (max |y| = 0.0111); none is near a pole (nearest 1.58 away). The blob arrives as one
face with four separate wires and 15 edges. That is a **loop-closure failure at the periodic wrap** —
neither "classified in when out" nor "never trimmed".

**Cheapest next experiment.** Two separable, both cheap: (i) `is_solid()` pole exemption + the
periodic-face volume fix, verified on the lune (must go 130.8815 -> 65.4498) and the single-face
sphere (must go `solid 0` -> `solid 1`); (ii) compare the periodic-NURBS-from-STEP reconstruction
path (clamped poles + `u_closed=.T.`) against the analytic path that is exact on the same shape —
the two paths disagree and one of them is right.

---

### D5 — Freeform SSI silently drops branches, and junctions are one branch short
**Blocks:** correctness on any curved input; and this is the *discovery-failure* evidence Step 1 was
designed to gather. **Owner: A** (`intersection.cpp` seeding; `brep_section.cpp` continuation gate).

**Evidence.**
- `L1b_pillow` — a 4^3 cube whose six planar faces are bulged bicubic patches; **seam 0, pole 0,
  closed, valid, read exactly by our kernel** (OCCT measures our round-tripped operand at 76.5759 vs
  the true 76.575864). **Exactly 3 of the 6 plane x patch SSI pairs return no intersection curve at
  all.** The `-x/-y/-z` patches pass through byte-for-byte untrimmed — area 14.9911 each, which is
  precisely the whole patch (89.946641 / 6 = 14.9911068), wires 1, edges 4, their own original
  straight boundary — and the three opposing box planes are then classified wholly-out and dropped.
  The `+x/+y/+z` three work and agree with OCCT to 0.2%. **The pillow is symmetric under sign flip**,
  so this is a seeding/marching-direction asymmetry in the algorithm, not a property of the data. The
  same three pairs fail in all three ops. The control that isolates it: box x analytic cylinder
  (which *has* a seam), same harness, same reader, same binary — **exact on all three ops**.
- Junction continuation (`hunt_efvf_gap`, x20 cut): OCCT's result has **val4 = 16** at edge-face
  piercing points where ours has **val4 = 1**. Our junctions are systematically one branch short —
  the section does not continue through the junction onto the adjacent face. `SESSION_EF_MARCH`
  **never fires** on x20 across all nine AUTO ladder passes (`bridge(march=0 weld=0|2)`): either
  `find_cont` (`brep_section.cpp:1861`) or the pi/6 continuation gate (`:1906`) rejects all four
  dangles, and all four dangles are within 2e-5 of a true piercing point.

**Cheapest next experiment (~2 minutes).** Mirror the pillow (negate one axis) and re-run. If the
failing set follows the mirror, the cause is in the data; if it stays `-x/-y/-z`, it is the marcher's
direction convention. Confirm with an operand-order swap — `src/brep.cpp:8446-8452` states outright
that "the freeform marcher is order-sensitive (seeds come from the first argument's cells)" and
`build_section_scaffold(A,B)` is still built from A first, so order dependence is expected nonzero
and has **never been measured** (`hunt_determinism` phase 4, queued and unfinished).

---

### D6 — Classification keeps the wrong side, or concludes there is no interference
**Blocks:** correct results as distinct from closed results. **Owner: A.**

**Evidence.** `x20 common`: OCCT's answer is **4 faces / 4 verts / vol 0.0005**; ours is **21 faces /
48 verts / 13 naked / open**, kernel-reported vol 26.3426 — *while holding 32 of the 34 true
edge-face piercing points*. We are not missing the interference geometry; we are keeping the wrong
side of it. Interop's Q06 (two axis-aligned overlapping boxes, the simplest boolean in solid
modelling, both read back with exact volume 1000) returned `fuse = A(1000)`, `common = B(1000)`,
`cut = A u B(2000)` against OCCT's 1875/125/875: **B classified as inside A**, intersection never
found. L4's `x=+2` keeps exactly 0.5029 of the true area. Chain depth 2 hands back both operands
untouched and calls it a solid.

**Cheapest next experiment.** Grid-sample the truth: `hunt_freeform_class` established that per-box-
face grid sampling of "is this point inside the other solid" is a cheap exact-enough oracle needing no
reference file, and used it to prove OCCT's own L4 reference wrong (OCCT omits both `y = +/-2` cuts,
14.63 of face area; Monte-Carlo with 40000 points puts true common at 53.7600 +/- 0.3519 against
OCCT's 58.5252, **13 sigma out**). Dump our per-face keep verdict for `x20 common` against that
sampled truth. Oracle-free, hours.

---

### D7 — Liveness: unbounded loops and allocation blow-ups
**Blocks:** the kernel must terminate; and a hang silently corrupts the ledger, because
`timeout N` -> rc=124 -> a truncated log the harness records as a *result*. **Owner: A.**

**Evidence.** `coneR x cyl` produces **no output within 600 s twice** on cut, no output in 600 s on
fuse, and `std::bad_alloc` on common under `ulimit -v 4194304` — OCCT does all three in milliseconds
(12.3073989871567 / 4.44774457132759 / 54.7193868517866, 3/2/5 faces). `box x torus` throws
`bad_alloc` in cut in **both** analytic and NURBS form; `I_iso_f2` common throws `bad_alloc`.
`SESSION_SYMEMIT`'s own comment records 459 runs / 19k edges / **8.3 GB** -> `bad_alloc` on z37.
Inside the AUTO ladder a `bad_alloc` is caught at `src/brep.cpp:8364` and silently becomes "variant
discarded" — so `ulimit -v` is part of the experiment and changes which variant wins, deterministically
(session A used 8 GB, the determinism probe used 4 GB: different experiments).

**Cheapest next experiment.** One run of `coneR x cyl` cut under a 60 s budget with the existing
iteration counters printed — `g_segrun_cap` (`brep.cpp:3935-3941`), `pair_budget=20000`
(`intersection.cpp:4907`), `max_iterations` (`intersection.cpp:934/964`) — to identify which loop is
unbounded. Then a hard cap that throws a **diagnosable** failure instead of allocating.

---

### D8 — The cross-session ledger is not comparable, so the campaign cannot tell whether it is improving
**Blocks:** every decision about what to fix next. **Owner: C.**

**Evidence** (`hunt_determinism`, whose kernel verdict is the good news: **deterministic to the
byte**). Full stdout+stderr md5s were byte-identical across repeats for y30/z37/z90/x20 cut
(`a5ea49e042`, `2e011fdb94`, `f21b2cf911`, `383b5b1e06`) under load average 28.7–58.1 and a 57%
wall-time spread. Zero RNG, zero threads, zero unordered containers in `brep.cpp`,
`brep_section.cpp`, `brep_samedomain.cpp`, `intersection.cpp`. The single nondeterministic input —
`/proc/self/statm` RSS at `brep.cpp:44/173`, armed only by AUTO at 3000 MB — did not fire (peak RSS
~33 MB). **The variance was never in the kernel.** What is corrupt:
- The frontier numbers are **Windows numbers**. Commit `aa49805` calls `_putenv` unconditionally;
  `g++ -c` on that is a hard error on Linux. The tree has since moved: **747 inserted / 39 deleted
  lines** across `brep.cpp` + `brep_section.cpp` + `intersection.cpp`, plus **4034 untracked lines**
  in `brep_bds`, `brep_commonblock`, `brep_massprops`, `brep_samedomain` (verified today at `5bb685a`).
  `CMakeLists.txt:84` uses `-march=native`.
- AUTO vs non-AUTO were compared as if they were the same algorithm ("z15 measures 17" is an **exact**
  match to `kb/census_z15.md`, run without AUTO; "z90 measures 0" is *better* than the frontier's 8).
- The y30 SD anomaly (23.96 vs 47.70) is a **real, deterministic** SD effect — `src/brep.cpp:8587`
  puts `!s_sd_mode` in the *loop condition*, deleting the entire legacy ON-imprint pass — and it was
  mis-cleared because the counter used to rule it out (`brep.cpp:8773`) sits **inside the branch
  `SESSION_SD` disables**. A zero read from a disabled probe is not a measurement.

**Cheapest next experiment.** None needed; adopt the protocol: pin the binary by md5 and never
measure against one another session is rebuilding; record the full `SESSION_*` set, platform,
`ulimit -v`, `git rev-parse HEAD` **and** `git status --short src/`; compare `md5(stdout+stderr)` with
timing stripped; treat rc=124 as *no measurement*; timeout = 3x measured baseline; **tolerance band
zero** for two runs of the same pinned binary; N=2 byte-identical suffices for non-AUTO cells, N=5 plus
peak-RSS logging required for any AUTO cell.

This **retires** DECISION Step 0.3 ("run T0 three times, declare the noise band"): the band is zero,
and the 2/132 cells that flipped verdict class were timeouts, not noise.

---

### D9 — Our own STEP export is lossier than the effects under study; chains and multi-solid results are unusable
**Blocks:** external verification at the precision the campaign needs, and every file-based workflow.
**Owner: A** (writer + multi-solid read) **+ C.**

**Evidence.** OCCT reads our exported `box_cut_sph` at **9.51456162197675** where our own verified
internal value is 9.545719199340 — a **3.3e-03 relative loss on export**. Any "measure our result with
OCCT via STEP" experiment is contaminated at 3e-03 and **cannot resolve the 1e-4 cluster it is being
used to grade**. Spheres are dropped entirely on export. The reader takes only `as[0]`, so the 6 of 30
shipped references that are multi-solid compounds cannot be fed back at all; chained booleans break at
depth 2 and report themselves clean (D2).

This directly threatens the DECISION's own safety net — "route all 132 cells through `step_probe`,
~40 lines in `runner.py`". Done today, it would manufacture false failures at the 1e-4 scale and
declare the export loss to be boolean error.

**Cheapest next experiment.** Write-read-write idempotence on our own exporter across the 30
references: `vol(read(write(X))) == vol(X)` to 1e-9. No oracle, one line per cell, and it fails today.

---

### D10 — EF/VF interference paving: **demoted**, the panel's attribution is refuted
**Blocks:** measurably little. Listed because `ARCHITECTURE_v2` M3 and `PLAN_capability_ladder` P5
currently rank it as a stage-sized deliverable. **Owner: A.**

**Evidence** (`hunt_efvf_gap`). The census is complete and independent (FreeCAD/OCCT,
`edge.section(face)` both directions): x20 has 34 true EF points, x13y29 has 42, **VF is zero in
both** (nearest operand vertex to the other surface: 0.042859 and 0.007938). Every vertex of all six
OCCT results is either an original operand vertex or an EF point — UNEXPLAINED = 0 in all six. **Our
results already contain 33/34 (worst error 3.2e-4) and 42/42 (worst 3.8e-3), with the pierced edges
split at them** (valence >= 3). Missing EF paving accounts for **3 of 9** naked edges in x20 cut,
**2 of 18** in x13y29 cut, and **0 of 13** in x20 common.

So `ARCHITECTURE_v2` M3's gate — "every true edge-face piercing has a vertex" — is **already green**,
and `PLAN` P5's gate is already met. Building the stage as specified would consume weeks and move
nothing. What remains is genuinely small: one doubly-represented EF point in x13y29 (V7 d=0.0058
val=3 and V6 d=0.0283 val=1, from two chains, never unified) and one absent point in x20 at d=0.127
that coincides with a stray 1-face shell carrying 3 of the 9 naked edges. Three of x13y29's naked
edges are **long** (10.39, 7.04, 3.35 units) and sit 2.1–4.1 from any EF point: a whole missing face
far outside the intersection zone — a different defect entirely.

A real EF stage is still worth having later (unconditional coverage independent of the SSI marcher,
single-source vertex identity, a branch-completeness invariant, and the edge-in-face common-block case
we lack entirely, OCCT `PaveFiller_5.cxx:545-565`). Realistic size: 250–400 lines in
`brep_section.cpp` plus a `curve_surface` API, roughly doubled if the EDGE-type common-block half is
included. **Not a campaign.**

---

### D11 — Two latent nondeterminism hazards, currently gated off
**Blocks:** nothing today. Listed so they are not promoted by accident. **Owner: A.**

`src/brep.cpp:1391-1396` caches a `Mesh` keyed on the raw `this` pointer plus face/vertex counts — a
classic ABA: free a BRep, allocate a new one at the same address with the same counts, and the
previous solid's mesh is reused for a point-in-solid test. Address-dependent, hence ASLR-dependent,
hence genuinely nondeterministic. Inert only because Tier-2 is behind `SESSION_PIP_GUARD`. **This is
exactly the flag DECISION Step 0.4 proposes to measure** — measure it, but do not promote it to
default until the key is a content hash. Second: `src/brep.cpp:8199-8210` + `8311-8314` cannot
distinguish an unset env var from one set to empty (`saved[k] = v ? v : ""`, and `put_env` maps empty
to `unsetenv`), so a variable exported as `SESSION_M3=` is **deleted** by the first restore.

---

## 2. Cross-hunt collisions — claims in the current plans that these findings overturn

| document | claim | status |
|---|---|---|
| `PLAN_capability_ladder` P1 | "the uniform ~9.4e-05 is the marcher's sampling constant showing through the gap"; P1 is "complete the analytic case table" | **Refuted.** The marcher constants are not executed in those cells; the sections are exact conics. The cluster is a fixed *absolute* 2–4e-03 defect divided by results in the 20–40 range. The coverage gap (skew unequal-radius cylinders have no branch, `intersection.cpp:4011`) is real but is **not** what produces the failing numbers. |
| `PLAN` P5 / `ARCHITECTURE_v2` M3 | EF/VF paving is missing and x20 + x13y29 are attributed to it; "Stage 5 is the one whose absence is measurable" | **Refuted.** Gate already green at 33/34 and 42/42; VF is zero. See D10. |
| `ARCHITECTURE_v2` §6 | "foreign geometry consumable" as an acceptance bar | Now **measured**: 0/21, and the dominant term is 10 lines in the reader (D1). Promote it from a bar to a rung. |
| `DECISION` "the one thing not in doubt: the geometry is right ... ~17,000 lines of geometry are not on the table" | | **Holds for analytic, fails for freeform.** Equivariance 1e-15 and byte-identical exact recognisers in every pose confirm the analytic half emphatically. But plane x bicubic NURBS drops 3 of 6 pairs (D5), so `intersection.cpp` is partly on the table after all. |
| `DECISION` safety net: route all 132 cells through `step_probe` | | **Blocked by D9.** Our exporter loses 3.3e-03; the probe would grade export loss as boolean error. Export idempotence first. |
| `DECISION` Step 0.3: run T0 three times, declare the noise band | | **Answered and retired.** Band is zero under a pinned binary; the flipping cells were rc=124 timeouts. Replace with the pinning protocol (D8). |
| `DECISION` Step 0.1: replace `metric()` | | **Landed** (uncommitted): `src/brep.cpp:8256-8268` now counts `nt > 2` against a candidate. |
| `DECISION` Step 0.4: free `SESSION_PIP_GUARD` measurement | | Still worth doing, with the D11 caveat attached. |

---

## 3. What this changes about the architecture decision (a vs b)

**Verdict: STRENGTHENS Step 0 (and enlarges it from half a day to three or four), REDIRECTS Step 1
from a two-bucket to a four-bucket partition run on different cells, and DELAYS the 2–4 month commit
by about a week — but does not move a single point toward (a).**

**(a) is not rehabilitated; it is further isolated.** Of the eleven ranked defects above, **zero are
owned by session B** (same-domain / coincidence). Five hunts, four of them looking at geometry classes
chosen precisely because they were unexplored, found not one blocking coincidence case. The 25 open
cells, the sph x cyl cliff, the pillow, the interop matrix, the x20/x13y29 junctions — none is a
same-domain problem. `DECISION`'s reasoning that (a) cannot move the open cells is unchanged and now
has four more independent data points behind it.

**(b)'s premise is confirmed at its core and its ladder is misordered.**
- *Confirmed, strongly:* `hunt_oriented_primitives` is the cleanest evidence (b) has received. The
  input curves are exact to 12 digits, the pipeline is rigid-motion equivariant to 1e-15, the
  recognisers fire identically in every pose — and the result is an **open shell** at a 0.3 deg
  relative tilt of a sphere and a cylinder. The defect is provably in how the arrangement is
  assembled, which is exactly the layer (b) proposes to make structural. `hunt_efvf_gap` says the
  same thing from the other side: we already hold the interference points and we are one branch short
  at 16 junctions. Neither is a geometry problem, a tolerance problem, or a coincidence problem.
- *Misordered:* M3 (VF/EF) is already green and should shrink to about a day (unify the doubly
  represented vertex; segment splitting at the anchor). M4's stated work (fill analytic coverage
  gaps) will not achieve M4's stated gate, because the oriented battery's wrongness is not recogniser
  coverage. Ingest (D1) and seam/pole (D4) have **no rung at all** and belong before M1 — a shared
  intersection graph built from an operand whose outer loops are labelled as holes is a faithfully
  shared wrong answer.
- *One sufficiency caveat, and it is the reversal trigger `DECISION` already wrote down:* v2 makes
  one-sided loss and divergent duplicates unrepresentable. It does **not** make the pillow's three
  missing SSI branches, or the sph x cyl cliff, impossible. "A shared graph of the branches you found
  is still missing the branch you never traced" now has a measured instance at the simplest possible
  curved input.

**Step 1 must be redirected, not merely rescheduled.** Three reasons:
1. **The dichotomy is not exhaustive.** We now have measured, unambiguous instances of *four* classes:
   **ingest** (Q02, rd2 — the operand was never right), **discovery** (pillow: 3 of 6 SSI pairs return
   nothing; x20: the branch through the junction is never marched), **agreement/assembly** (sph x cyl:
   sections provably exact, result open and 41% wrong), and **classification** (x20 common: 21 faces
   vs 4 while holding 32/34 EF points; L4: a box face keeps exactly half). A two-bucket partition run
   today would mis-file at least the first and fourth into whichever bucket the analyst preferred.
2. **Some of the 25 cells may be ingest artefacts.** The chair operands are Rhino-authored and escape
   D1 — but the `REFERENCE_*.step` results are OCCT-authored (P01's chair0: 36 `FACE_BOUND`), 6 of 30
   are multi-solid compounds the reader truncates to `as[0]`, and the round-trip control that was used
   to clear the corpus measured *geometry preservation*, not *interpretation*, so it could not have
   caught D1. Partition before re-measuring and the partition inherits the bug.
3. **The decision rule cannot fire cleanly.** ">= 60% agreement -> proceed; >= half discovery -> stop,
   the project is Law 4" presumes the classes are close to disjoint. They are not, and both are
   already confirmed present. Restate it as a per-class budget: how many cells does each class own,
   and what does each class cost.

**Revised Step 0+, three to four days, every item valuable under every outcome:**

| day | work | why it is unconditional |
|---|---|---|
| 1 | D1: outer-bound decided geometrically from 2D loop areas; re-run the 21-cell interop matrix and the freeform ladder against inputs that **already exist** (`hunt_interop/fob/`, `hunt_freeform/dataset_ob/`) | 10 lines; two "critical" verdicts are currently measured through it; ~half the corpus operands are affected |
| 1 | D2: three oracle-free invariants + wire-occurrence naked + pole-exempt `is_solid()` + periodic-face volume; add `nonman`/shells/naked verdict columns | nothing above this is meaningful; the invariants caught the pillow with no oracle at all |
| 2 | Three free micro-oracles as permanent gates: sphere-x-cylinder-through-centre (closed form, any angle), pillow partition identity, per-box-face grid sampling | seconds per run against 15–18 min per T0; this is what makes `DECISION` Step 5's ~25–30 hours of gate time affordable |
| 2 | D9: export write-read-write idempotence, before any `step_probe` wiring | otherwise the safety net grades our exporter's 3.3e-03 loss as boolean error |
| 2 | D8: pinning protocol adopted; frontier re-baselined on Linux with a pinned md5 | the current frontier is Windows numbers from a tree that cannot compile here |
| 3–4 | Four-class partition of the **re-measured** open cells, plus the three micro-cells | same 1–2 days as before, on data that is not damaged, with a taxonomy that fits what is there |

**Then decide.** The extra day is repaid on day one by not grading against damaged operands.

**What would reverse this.** If the interop matrix stays at 0/21 with exact operands after D1, the
ingest story is finished and **D6 (classification) jumps to rank 1** — a kernel that reads geometry
perfectly and still classifies B as inside A is a different project from one that reads it wrong. My
Q02 run is already a partial datapoint in that direction: exact operands, structurally closer result
(2 faces -> 10 against OCCT's 9), still open at naked 21 and still ~2x off. Expect reclassification,
not repair.

---

## 4. Provenance

| source | date | kernel state | notes |
|---|---|---|---|
| `hunt_occt_interop.md` | 07-26 01:32 | `main_7_interop` md5 `2d4625f01c60c23ed21dcb403ed4a119` | 6 cells NOT MEASURED (880 s cap, load 28); OCCT-result operands confounded by round-trip (sections 3, 4, 6 are not) |
| `hunt_oriented_primitives.md` | 07-25 23:59 | `main_7_oriented` | do not quote box x torR reference digits past 1e-4 (cache vs FreeCAD spread 1.5e-04); no conclusion uses the STEP-export route |
| `hunt_freeform_class.md` | 07-25 23:27 | FreeCAD 1.1.1 harness | retracts its own brief's bbox "proof" (FreeCAD `BoundBox` bounds the control net); shows OCCT's L4 reference is itself wrong by 13 sigma; `I_iso_*` variants are not pole-free and should not be over-trusted |
| `hunt_determinism.md` | 07-25 23:02 | N=2 serialized | phases 2–6 (concurrency wave, operand-order A,B vs B,A, AUTO 5x5, ASLR/UB probes, volume repeatability) NOT MEASURED |
| `hunt_efvf_gap.md` | 07-25 22:49 | base path, AUTO off except x20 cut | fuse not run for either config; common only for x20; EF matching crosses a 5e-05 reader fidelity gap |
| **this file's section 0** | 07-26 | same binary and flags as interop, N=1 | as-written value reproduces interop's 19.2149 exactly, pinning binary and flags |
