# TIER 3b — FOREIGN GEOMETRY AND CHAINED BOOLEANS (measured)

**GATE NOT MET, and (d) says why.** Test (d) — round-trip identity — was run FIRST, as instructed,
because it gates the interpretation of everything else. It fails on 2 of 5 OCCT-authored
primitives and on 40 of 40 imported OCCT/FreeCAD boolean results. Both defects are in
`src/file_step.cpp`, which is outside this session's ownership line, so they are **reported with
the evidence that isolates them, not fixed**.

The headline is not "our booleans are worse on foreign operands" (they are, by 2-4 orders of
magnitude). It is that **on foreign operands a closure-only verdict reports SUCCESS on an answer
that is 83 % wrong**, because a silently-empty import turns `A op B` into `A`.

---

## 0. Provenance — every number below carries this

| item | value |
|---|---|
| driver | `session_cpp/main_23.cpp` (new; this session's only file in the repo) |
| driver binary sha1 | `41f5badeff84cd9d59393024adc30163f94f969d` (`/home/petras/v2_interop/build/main_23`) |
| OCCT tool sha1 | `2de995b91c672b12cbad203510f918c06f5fa0ec` (`/home/petras/v2_interop/occt_build/occt_interop`) |
| verdict | `src/v2/v2_verdict.h` → `session_cpp::v2v::v2_verdict` ONLY. No verdict code was written for this task. |
| `SESSION_*` env | **none set**, in every run (`env \| grep -c '^SESSION_'` == 0). All kernel defaults. |
| repo HEAD | `5bb685a`, with the v1 agent's uncommitted edits present in the tree |
| source sha1 of the binary's inputs | `out/source_sha1.txt` — `file_step.cpp b93713b0`, `brep.cpp 41f25195`, `intersection.cpp d5d1b7ed`, `brep_section.cpp 1ddf125e`, `brep_v2_boolean.cpp ea1139df`, `v2_verdict.cpp bd7ed3aa` |
| compiler | g++ 15.2.0, `-O3 -march=native`, Release |
| OCCT | the repo's own static install, `validation/occt_oracle/build/deps/occt/install` (V8_0_0_rc2), i.e. the same kernel that produced the trace corpus |
| FreeCAD | 1.1.1 (snap), `Part.makeBox/makeSphere/...`, used to confirm the reader gap reproduces on the tool the brief names |

Everything is under `/home/petras/v2_interop/`: `occt_interop.cpp` (OCCT side), `run_[abcde]_*.sh`
(the sweeps), `analyze.py` (log → table), `out/*.log` (raw), `step/` (every operand file).

### What I changed
* Added `session_cpp/main_23.cpp` — a **cell-oriented** driver: one process invocation = one
  boolean = one `key=value` line. A hang costs one cell, not the run, and "attempted vs
  completed" is countable from the log.
* Wired it from `/home/petras/v2_interop/proj/CMakeLists.txt`, which `add_subdirectory()`s
  `session_cpp` and adds only the `main_23` target. **`session_cpp/CMakeLists.txt` was not
  touched** (another agent owns it); the two paths that file reads relative to `CMAKE_SOURCE_DIR`
  (`generated/`, `../session_proto`) are symlinked inside `/home/petras/v2_interop/`. Build dir is
  `/home/petras/v2_interop/build` — nobody else's.
* Nothing else in the repo was modified.

### Two measurement errors I made and corrected mid-run — both would have inverted a conclusion
1. **`printf` to a redirected log is block-buffered.** The first (c) run reported "v2 never
   reached depth 1" — actually depth 1 had completed in 15.8 s and `SIGTERM` from `timeout`
   discarded the buffer. Every `printf` in `main_23.cpp` is now followed by `fflush(stdout)`;
   the sweeps were restarted from zero. *Do not count a missing line as a failure until the
   writer flushes.*
2. **`read_file_step_breps` returns a vector, and these files have up to 3 solids.** Probing
   only `[0]` made `REFERENCE_z30_cut` look like "our reader drops 4 of 42 faces". It does not:
   38 + 4 = 42, exactly OCCT's count. §4 re-measures every file over *all* its solids.

### One hypothesis I formed and then killed with a measurement
"None of the 30 REFERENCE files contains a single `PCURVE` or `SURFACE_CURVE` (verified: 0 in
all 30), so our reader must project the 3D edges and that is where closure is lost."
**False.** I re-exported four of them through OCCT *with* pcurves (228-519 each) and our reader
produced **identical verdicts** — same faces, same edges, same `resid`, same volume to every
printed digit. (The reader does have a pcurve path — `file_step.cpp:1012`, and :1439 "bind the
file's own pcurves (or projected samples)" — so the null result means that path did not change
the answer, not that pcurves are ignored.) Killed a second, independent way: FreeCAD's
primitives carry 0 pcurves and import exactly.

---

## 1. (d) ROUND-TRIP IDENTITY — run first, and it FAILS

`import → v2_verdict → export → import → v2_verdict`, no boolean anywhere.
Log: `out/d_roundtrip.log`, 13 attempted, 11 completed (2 exited 4 = the re-import found no
solid, which is itself the result).

| source | in: closed/faces/vol | out: closed/faces/vol | rel Δvol |
|---|---|---|---|
| `own:box` | 1 / 6 / 64 | 1 / 6 / 64 | 0 |
| `own:sphere` | 1 / 1 / 65.449847 | 1 / 1 / 65.449847 | 0 |
| `own:cylinder` | 1 / 3 / 35.342917 | 1 / 3 / 35.342917 | 4.0e-16 |
| `own:cone` | 1 / 2 / 20.943951 | 1 / 2 / 20.943951 | 5.1e-16 |
| `own:torus` | 1 / 1 / 19.344425 | 1 / 1 / 19.344425 | 2.2e-15 |
| OCCT `box` | 1 / 6 / 64 | 1 / 6 / 64 | 0 |
| **OCCT `sphere`** | **0 / 1 / 0** | **re-import found no solid** | — |
| OCCT `cylinder` | 1 / 3 / 35.342917 | 1 / 3 / 35.342917 | 2.0e-16 |
| OCCT `cone` | 1 / 2 / 20.943951 | 1 / 2 / 20.943951 | 3.4e-16 |
| **OCCT `torus`** | 1 / 1 / 19.344425 | **0 / 1 / 9.6722123** | **0.5** |
| `REFERENCE_z30_cut` #0 | 0 / 38 / 78.32326 | 0 / 38 / 70.930494 | 0.094 |
| `REFERENCE_z30_common` | 0 / 27 / 21.793991 | 0 / 27 / 44.742078 | 1.05 |
| `REFERENCE_x20_fuse` | 0 / 68 / 338.4372 | re-import found no solid | — |

**Our own primitives round-trip exactly, 5/5.** Our STEP *writer* is also sound: OCCT reads all
nine primitive exports back as `valid=1, nsolid=1` with volume and area matching to every printed
digit. The failures are two specific, isolated defects.

### Defect D1 — `VERTEX_LOOP` is not implemented in the reader
A full sphere has no edges, so OCCT and FreeCAD bound its single face with a STEP `VERTEX_LOOP`
(one vertex, meaning "the whole surface"). `occt_sphere.step` in full:

```
#17 = ADVANCED_FACE('',(#18),#22,.T.);
#18 = FACE_BOUND('',#19,.T.);
#19 = VERTEX_LOOP('',#20);          <-- reader has no branch for this
#22 = SPHERICAL_SURFACE('',#23,2.5);
```

`file_step.cpp:1203` and `:1516` do `lent->second.find("EDGE_LOOP")` and `continue` when it is
absent. Result: a 1-face, **0-edge, 0-volume** BRep. `v2_verdict.has_topology` is false, so the
verdict is honest at the operand — but a boolean does not check that (§2).

Reproduces on the tool the brief names. FreeCAD `Part.makeSphere(2.5)`, exported and re-imported:

| FreeCAD primitive | VERTEX_LOOP | our import |
|---|---|---|
| box | 0 | closed=1, vol 64, resid 0 |
| **sphere** | **1** | **closed=0, faces=1, edges=0, vol 0** |
| cylinder | 0 | closed=1, vol 35.34291735, resid 3.6e-17 |
| cone | 0 | closed=1, vol 20.94395102, resid 4.6e-17 |
| torus | 0 | closed=1, vol 19.34442463, resid 5.0e-17 |

Forcing OCCT's explicit-sweep constructor (`BRepPrimAPI_MakeSphere(r, 2π)`) does **not** avoid it —
still one `VERTEX_LOOP`. There is no way to hand our reader an OCCT-authored full sphere.

### Defect D2 — the writer collapses a doubled seam to one side of the period
Our export of the *imported* OCCT torus writes the seam edge's two pcurves at the **same** `u`:

```
own:torus  (round-trips fine)            imported OCCT torus (re-reads as half)
#54=CARTESIAN_POINT('',(6.28318,0.));    #54=CARTESIAN_POINT('',(6.28318,6.28318));
#59=CARTESIAN_POINT('',(9.5e-33,0.));    #59=CARTESIAN_POINT('',(6.28318,6.28318));
     ^ u=2π and u=0: the two aliases          ^ both at u=2π: the u=0 alias is gone
```

OCCT's reader heals this (it reads our file back as an exact `vol=19.34442463`, `valid=1` torus);
ours does not, and resolves the face to half its period → `vol=9.6722123`, `resid=0.36`. So the
loss is real information loss on write, masked by OCCT's `ShapeFix` on read.

**Consequence for the rest of this document:** sphere-as-an-OCCT-operand cannot be measured at
all, and any result whose operand came back through our own writer carries D2. Every table below
states which.

---

## 2. (a) OCCT-AUTHORED PRIMITIVES AS OPERANDS

Method: OCCT authors **both** operands at their final poses and writes them to STEP — no
transform is applied on our side, so the "translating an import moves its volume" pathology
cannot confound the cell. Then, on the identical shapes:

* **truth** — OCCT's own boolean (`BRepAlgoAPI_Cut/Common/Fuse`) on the same two shapes;
* **foreign** — our kernels on `step:A.step` / `step:B.step`;
* **control** — our kernels on `own:` primitives built to the *same* dimensions (verified
  identical: `own:cylinder@1,0,1` and the OCCT spec both give `vol=35.34291735 area=61.26105675`).

The control is the control cell in the brief's sense: it isolates the interop axis from "this
pair is just hard". Log `out/a_primitives.log`, 120 cells, timeout 300 s each.

```
pair          op           OCCT vol |    cur/for    relerr c |    cur/own    relerr c |     v2/for    relerr c |     v2/own    relerr c
---------------------------------------------------------------------------------------------------------------------------------------
box_sphere    cut          34.94027 |         64  8.32e-01 1 |    34.9498  2.74e-04 0 |         64  8.32e-01 1 |    34.9498  2.74e-04 0 
box_sphere    common       29.05973 |          0  1.00e+00 0 |    29.0471  4.33e-04 0 |          0  1.00e+00 0 |    29.0471  4.33e-04 0 
box_sphere    fuse         100.3901 |         64  3.62e-01 1 |    104.716  4.31e-02 0 |         64  3.62e-01 1 |    104.716  4.31e-02 0 
box_cyl       cut          38.82315 |    38.8314  2.13e-04 0 |     38.826  7.47e-05 0 |    40.5175  4.36e-02 0 |     38.826  7.47e-05 0 
box_cyl       common       25.17685 |    25.1645  4.89e-04 0 |    25.1699  2.77e-04 0 |    26.8712  6.73e-02 0 |    25.1699  2.77e-04 0 
box_cyl       fuse         74.16607 |    74.1743  1.11e-04 0 |     74.169  3.91e-05 0 |    64.0794  1.36e-01 0 |     74.169  3.91e-05 0 
box_cone      cut          49.39638 |    52.4337  6.15e-02 0 |    49.3988  4.85e-05 0 |    52.4337  6.15e-02 0 |    49.3988  4.85e-05 0 
box_cone      common       14.60362 |    39.3084  1.69e+00 0 |    14.4313  1.18e-02 0 |    39.3084  1.69e+00 0 |    14.4313  1.18e-02 0 
box_cone      fuse         70.34033 |    46.6807  3.36e-01 0 |    70.1333  2.94e-03 0 |    46.6807  3.36e-01 0 |    70.1333  2.94e-03 0 
box_torus     cut           55.2472 |    58.8184  6.46e-02 0 |    55.3312  1.52e-03 0 |    58.8184  6.46e-02 0 |    55.3312  1.52e-03 0 
box_torus     common       8.752802 |     5.1815  4.08e-01 0 |    8.61595  1.56e-02 0 |     5.1815  4.08e-01 0 |    8.61595  1.56e-02 0 
box_torus     fuse         74.59161 |    62.2499  1.65e-01 0 |    69.9239  6.26e-02 0 |    62.2499  1.65e-01 0 |    69.9239  6.26e-02 0 
sph_cyl       cut          41.30659 |          0  1.00e+00 0 |         --        -- - |          0  1.00e+00 0 |         --        -- - 
sph_cyl       common       24.14326 |          0  1.00e+00 0 |         --        -- - |          0  1.00e+00 0 |         --        -- - 
sph_cyl       fuse          76.6495 |    35.3429  5.39e-01 1 |    957.849  1.15e+01 0 |    35.3429  5.39e-01 1 |    957.849  1.15e+01 0 
sph_sph       cut          51.83628 |          0  1.00e+00 0 |    51.8435  1.39e-04 1 |          0  1.00e+00 0 |    51.8363  4.24e-09 1 
sph_sph       common       13.61357 |          0  1.00e+00 0 |    13.6064  5.29e-04 1 |          0  1.00e+00 0 |    13.6136  5.14e-09 1 
sph_sph       fuse         117.2861 |          0  1.00e+00 0 |    117.293  6.14e-05 1 |          0  1.00e+00 0 |    117.286  1.71e-09 1 
cyl_cyl       cut          27.69417 |    19.2678  3.04e-01 0 |    19.2674  3.04e-01 0 |    19.2678  3.04e-01 0 |    19.2674  3.04e-01 0 
cyl_cyl       common       7.648748 |    7.64553  4.21e-04 0 |    7.64559  4.12e-04 0 |    7.64553  4.21e-04 0 |    7.64559  4.12e-04 0 
cyl_cyl       fuse         63.03709 |    51.5071  1.83e-01 0 |    54.6103  1.34e-01 0 |    51.5071  1.83e-01 0 |    54.6103  1.34e-01 0 
cone_cone     cut          15.42167 |     13.295  1.38e-01 0 |    11.4278  2.59e-01 0 |     13.295  1.38e-01 0 |    11.4278  2.59e-01 0 
cone_cone     common       5.522278 |    5.46259  1.08e-02 0 |    13.6139  1.47e+00 0 |    5.46259  1.08e-02 0 |    13.6139  1.47e+00 0 
cone_cone     fuse         36.36562 |    34.3118  5.65e-02 0 |    25.8954  2.88e-01 0 |    34.3118  5.65e-02 0 |    25.8954  2.88e-01 0 
torus_torus   cut          14.46306 |    6.93701  5.20e-01 0 |    8.78014  3.93e-01 0 |    6.93701  5.20e-01 0 |    8.78014  3.93e-01 0 
torus_torus   common       4.880927 |    4.82321  1.18e-02 0 |    4.82321  1.18e-02 0 |    4.82321  1.18e-02 0 |    4.82321  1.18e-02 0 
torus_torus   fuse         33.80615 |    22.6936  3.29e-01 0 |    25.2696  2.53e-01 0 |    22.6936  3.29e-01 0 |    25.2696  2.53e-01 0 
box_box       cut                43 |         43  0.00e+00 1 |         43  0.00e+00 1 |   0.333333  9.92e-01 0 |         43  0.00e+00 1 
box_box       common             21 |         21  0.00e+00 1 |         21  0.00e+00 1 |    21.6667  3.17e-02 0 |         21  0.00e+00 1 
box_box       fuse              107 |        107  0.00e+00 1 |        107  0.00e+00 1 |    64.3333  3.99e-01 0 |        107  0.00e+00 1 

  cur foreign: cells=30 closed=6 vol_exact=3 SUCCESS(closed&rel<1e-6)=3 FALSE_PASS(closed&rel>1e-3)=3 throw=0 empty=2 missing/timeout=0
  cur control: cells=28 closed=6 vol_exact=3 SUCCESS(closed&rel<1e-6)=3 FALSE_PASS(closed&rel>1e-3)=0 throw=0 empty=0 missing/timeout=2
  v2  foreign: cells=30 closed=3 vol_exact=0 SUCCESS(closed&rel<1e-6)=0 FALSE_PASS(closed&rel>1e-3)=3 throw=0 empty=2 missing/timeout=0
  v2  control: cells=28 closed=6 vol_exact=6 SUCCESS(closed&rel<1e-6)=6 FALSE_PASS(closed&rel>1e-3)=0 throw=0 empty=0 missing/timeout=2
  cur == v2 (identical status/closed/faces/naked/vol), foreign: 24/30
  cur == v2 (identical status/closed/faces/naked/vol), control: 25/28
```

**Reading it.**

* `box_sphere` foreign, cut and fuse: `closed=1`, `solids=1`, and the answer is **the whole box**
  (64 vs 34.94 and 100.39). The sphere operand imported as a 0-edge shell (D1) and the boolean
  quietly returned `A`. `sph_cyl` foreign fuse is the same failure with the operands swapped:
  `closed=1`, `vol=35.3429` — exactly the cylinder's own volume — against a truth of 76.65.
  **This is the industry-grade finding: on foreign geometry, closure is not a success
  criterion.** A sweep scored on `closed()` alone would have called all three PASS. They are
  counted here as FALSE_PASS and never as successes.
* `sph_cyl` / `sph_sph` foreign cut and common: `status=EMPTY` or `vol=0` — same cause (D1),
  opposite symptom. 2 EMPTY plus 4 zero-volume cells, reported separately, never as successes.
* `box_cone` / `box_torus`: the OCCT operands import **exactly** (§1: `vol` to 10 digits,
  `resid` ≤ 5e-17) and yet the boolean is 1-2 orders of magnitude worse than on our own
  identical-volume primitive. That is the interop axis with no import defect in it.

### The isolating experiment: authorship vs our own round-trip
Same pair, same op, three provenances of one operand:

| box × cone, `common` (OCCT truth 14.60362) | vol | rel err | naked | faces |
|---|---|---|---|---|
| our native `own:cone` | 14.4312957 | 1.2e-2 | 0 | 4 |
| our own cone through **our** writer+reader | 15.2388580 | 4.4e-2 | 3 | 4 |
| **OCCT-authored cone** | **39.3083641** | **1.69** | **29** | 33 (26 shells) |

| box × torus, `common` (OCCT truth 8.752802) | vol | rel err | naked | faces |
|---|---|---|---|---|
| our native `own:torus` | 8.61594632 | 1.6e-2 | 0 | 5 |
| our own torus through our writer+reader | 8.61594632 | 1.6e-2 | 0 | 5 |
| **OCCT-authored torus** | **5.18150257** | **0.41** | 0 | 5 |

Our own STEP round trip is nearly neutral (exactly neutral for the torus). **The damage comes
from OCCT's authoring of an analytically identical surface**, not from passing through a file.
The STEP entity types are the same on both sides (`CONICAL_SURFACE`+2 `PLANE`,
`TOROIDAL_SURFACE`+`PLANE` — checked), so what differs is the *parametrisation*: seam origin,
reference direction, trim-curve form. Our booleans are sensitive to it; OCCT's are not.

---

## 3. (b) OCCT BOOLEAN RESULTS AS OPERANDS — termination and closure only

30 files in `/home/petras/fc_inspect/REFERENCE/`. Cutter: an OCCT-authored box at each operand's
own bbox centre, half its bbox size, so a real overlap is guaranteed; identical operands for both
kernels. Operand = solid `#0` of the file (several files hold 2-3 solids; §4 covers all of them).

**Why volume is not scored here, restated with the measurement.** The brief says their volumes are
meaningless as truth. §4 shows the actual situation is sharper and the blame is ours: OCCT reads
all 30 files as `valid=1` solids, and *we* import them with the face and edge counts exactly right
but the trimmed **areas inflated by a median factor of 1.77**. So the operand is already wrong
before any boolean runs, and only termination and closure of the result mean anything.

```
  cur: OK=87  cells=87  naked>0=87  nonmanifold=72  open=87
       naked_real min/median/max = 4/67/138   closure_residual min/median/max = 0.0005757/0.3219/0.9405
  v2 : OK=87  cells=87  naked>0=87  nonmanifold=72  open=87
       naked_real min/median/max = 4/67/138   closure_residual min/median/max = 0.0005757/0.3219/0.9405
  cur and v2 produced the IDENTICAL result (status/closed/faces/naked/vol): 87/87 pairs
  process exits: {'rc0': 174, 'notcompleted': 12}
  OCCT on the identical operands: {'cells': 87, 'occt_valid': 87}
```

`REFERENCE_z15_common.step` contains **0 `ADVANCED_FACE` and 0 `MANIFOLD_SOLID_BREP`** — OCCT's
own answer for that pair was empty. It is a trivially-empty operand and is reported here, never
summed into any success count.

---

## 4. Reader fidelity on all 30 files — no boolean involved

This is the (d) gate extended to the whole foreign corpus, and it is the cleanest number in this
document. Log `out/e_import.log`, report `out/report_e.txt`.

```
file                            nsolid  faces occt/ours  edges occt/ours   area occt   area ours  ratio    vol occt    vol ours  resid max  closed
REFERENCE_x13y29_common          3/3        31/31            75/75           81.0341     229.745   2.84     31.8241     96.7304     0.7514 0/3
REFERENCE_x13y29_cut             1/1        29/29            90/90           158.315     205.047   1.30     48.4728     155.782     0.2948 0/1
REFERENCE_x13y29_fuse            1/1        50/50           155/155          272.971     369.256   1.35      128.77     289.746     0.1764 0/1
REFERENCE_x20_common             1/1         4/4              6/6          0.0590158     0.39905   6.76 0.000480411    0.167398     0.8428 0/1
REFERENCE_x20_cut                1/1        38/38           103/103          176.994     313.931   1.77     80.2964     39.6936     0.2371 0/1
REFERENCE_x20_fuse               1/1        68/68           173/173          353.946     508.154   1.44     160.593     338.437       0.26 0/1
REFERENCE_y30_common             2/2        31/31            81/81           86.2181     244.498   2.84     33.3379     11.9984     0.4031 0/2
REFERENCE_y30_cut                1/1        32/32            96/96           158.301     239.129   1.51     46.9589     193.816     0.2536 0/1
REFERENCE_y30_fuse               1/1        54/54           164/164          267.787     404.767   1.51     127.256     176.716     0.1652 0/1
REFERENCE_z15_common                                   EMPTY FILE (0 ADVANCED_FACE) — OCCT's answer was empty
REFERENCE_z15_cut                2/2        44/44           119/119          178.021     315.308   1.77     80.2971     291.567     0.3832 0/2
REFERENCE_z15_fuse               1/1        43/43           118/118          177.638     318.305   1.79     80.9521     260.077     0.2723 0/1
REFERENCE_z30_common             1/1        27/27            76/76           72.0479     240.617   3.34      24.362      21.794      0.157 0/1
REFERENCE_z30_cut                2/2        42/42           120/120          179.764     341.293   1.90     55.9351     80.8526      0.293 0/2
REFERENCE_z30_fuse               1/1        60/59           166/164          282.285     385.542   1.37     136.232     103.785     0.1326 0/1
REFERENCE_z30x20_common          1/1        20/20            55/55           68.4425     227.699   3.33     26.0394     137.317     0.3647 0/1
REFERENCE_z30x20_cut             1/1        32/32            97/97           182.098     278.298   1.53     54.2577     239.867     0.2896 0/1
REFERENCE_z30x20_fuse            1/1        52/51           147/145          285.863     333.608   1.17     134.555     24.7645     0.2223 0/1
REFERENCE_z37_common             2/2        30/30            79/79           70.5789     210.098   2.98     22.7394     151.428     0.6465 0/2
REFERENCE_z37_cut                2/2        42/42           120/120          179.878     357.925   1.99     57.5577     75.6158     0.3693 0/2
REFERENCE_z37_fuse               1/1        56/56           163/161          283.425      353.55   1.25     137.854     140.592     0.1564 0/1
REFERENCE_z45_common             1/1        28/28            78/78           70.2515     222.265   3.16     21.1009     239.014     0.4092 0/1
REFERENCE_z45_cut                2/2        41/41           117/117          180.289     331.339   1.84     59.1962     42.5488     0.4475 0/2
REFERENCE_z45_fuse               1/1        55/55           156/156          283.753       390.3   1.38     139.493     215.187     0.1888 0/1
REFERENCE_z63_common             2/2        32/32            84/84           68.1232     258.654   3.80     17.6742     46.1752     0.7248 0/2
REFERENCE_z63_cut                2/2        41/41           117/117          178.936     333.134   1.86     62.6227     105.314     0.3956 0/2
REFERENCE_z63_fuse               1/1        53/53           156/156          285.881     329.322   1.15      142.92     171.585     0.1686 0/1
REFERENCE_z90_common             2/2        30/30            78/78           64.1473     175.518   2.74     13.3036     104.743     0.5266 0/2
REFERENCE_z90_cut                1/1        36/36           108/108          176.575     274.445   1.55     66.9934     92.7104    0.09015 0/1
REFERENCE_z90_fuse               1/1        53/53           159/159          289.857     392.178   1.35      147.29     16.0108     0.1343 0/1

  area ratio ours/OCCT: min=1.15  max=6.76  median=1.77
  imported solids closed(): 0/40
```

**Defect D3 — trimmed regions of imported bicubic B-spline faces are too large.**

* Solid count, face count and edge count match OCCT **exactly** on 27 of 29 non-empty files
  (`z30_fuse` 59/60 and `z30x20_fuse` 51/52 are the only face-count misses). `naked_real` and
  `nonmanifold` also match — including OCCT's own `nonmanifold=25` on `x20_fuse`. **The topology
  is read faithfully.**
* The **area** of the same faces is 1.15-6.76× too large, median 1.77×. For the 19 single-solid
  files this is a direct like-for-like comparison of the same face set, with our own writer
  nowhere in the loop.
* Independent confirmation without our area integrator: OCCT reads our re-export of
  `REFERENCE_z30_cut` #0 with bbox `z ∈ [-4.7239, 7.76605]` where the original file's whole
  bbox is `z ∈ [-3.74642, 7.76605]`. **Our imported faces occupy space outside the true solid.**
* Consequence: `closure_residual` 0.090-0.843 and **0 of 40 imported solids are `closed()`**.
  The volume is not translation-invariant — `REFERENCE_z30_cut#0` reads `vol=78.32326` at the
  origin and `2000.36581` translated by (100,0,0), with `area` unchanged at 327.238 — which is
  exactly what a non-closed boundary does under the divergence theorem.

Surface *type* is not the cause: an OCCT box, cylinder, cone and torus put through
`BRepBuilderAPI_NurbsConvert` (so every face is `B_SPLINE_SURFACE_WITH_KNOTS`) import at
`vol=64/35.307/20.912/19.344`, `resid` 0 to 8e-4; and an OCCT `cut` of two boxes, NURBS-converted,
imports at `vol=43, resid=0, closed=1` — identical to the analytic version. All 42 surfaces in a
REFERENCE file are bicubic (3,3) with exactly one bound per face. What those faces have that the
primitives do not is a trim loop that is a genuine free-form sub-region of a larger patch.

---

## 5. (c) CHAINED BOOLEANS TO DEPTH 3

Our own output is the left operand of the next step, **in memory** — no STEP round trip inside
the loop, so D2 cannot be blamed for a chain failure. OCCT truth for the same chains is in
`out/c_truth.log`. Log `out/c_chain.log`, timeout 600 s per chain (900 s for the three chains
re-run after the harness reaped the first sweep's launcher).

```
chain                  k    op                              d1                         d2                         d3  last_closed
C3_common              cur  common      OK cl=0 f=  6 v=  44.3573    OK cl=0 f=  7 v=  20.8928              (not reached)   -
C1_box_cut             cur  cut         OK cl=1 f=  9 v=       43    OK cl=0 f= 11 v=  27.6667    OK cl=0 f= 11 v=       12   1
C2_mixed_cut           cur  cut         OK cl=0 f= 10 v=  34.9498    OK cl=0 f= 13 v=  19.9846              (not reached)   -
C4_foreign_cut         cur  cut         OK cl=1 f=  9 v=       43    OK cl=0 f= 11 v=  27.6667    OK cl=0 f= 12 v=       12   1
C5_foreign_all_cut     cur  cut         OK cl=0 f=  8 v=  38.8314    OK cl=0 f= 25 v=  36.8926              (not reached)   -
C1_box_fuse            cur  fuse        OK cl=1 f= 12 v=      107    OK cl=0 f= 19 v=  143.667    OK cl=0 f= 24 v=  185.333   1
C2_mixed_fuse          cur  fuse        OK cl=0 f= 10 v=  104.716    OK cl=0 f= 14 v=  120.039    OK cl=0 f= 26 v=  21.6008   0
C3_common              v2   common      OK cl=0 f=  6 v=  44.3573    OK cl=0 f=  7 v=  20.8928              (not reached)   -
C1_box_cut             v2   cut         OK cl=1 f=  9 v=       43    OK cl=0 f= 11 v=  27.6667    OK cl=0 f= 11 v=       12   1
C2_mixed_cut           v2   cut         OK cl=0 f= 10 v=  34.9498    OK cl=0 f= 13 v=  19.9846              (not reached)   -
C4_foreign_cut         v2   cut         OK cl=0 f=  8 v=       46              (not reached)              (not reached)   -
C4_foreign_cut         v2   cut         OK cl=0 f=  8 v=       46    OK cl=0 f=  9 v=  30.6667    OK cl=0 f=  9 v=       15   0
C5_foreign_all_cut     v2   cut         OK cl=0 f= 10 v=  40.5175              (not reached)              (not reached)   -
C1_box_fuse            v2   fuse        OK cl=1 f= 12 v=      107              (not reached)              (not reached)   -
C2_mixed_fuse          v2   fuse        OK cl=0 f= 10 v=  104.716    OK cl=0 f= 14 v=  120.039    OK cl=0 f= 26 v=  21.6008   0
```

Cells: (a) **120 attempted / 116 completed** (4 timeouts, all `CONTROL_sph_cyl` cut+common).
(b) **186 attempted / 174 completed** (12 timeouts). (c) **14 chain runs attempted / 7
completed** (7 timeouts). Everything not completed is a `timeout` (`rc=124`), never a silent
drop, and is listed as `NOTCOMPLETED` in the logs; a timed-out chain still reports every depth
it finished, because every line is `fflush`ed. `C4_foreign_cut v2` appears twice because the
first run was killed mid-chain when the harness reaped its launcher; the second, complete run is
the one to read.

**The depth at which each kernel breaks: DEPTH 2, both of them, on every chain.**

* `C1_box_cut` (four axis-aligned boxes, the easiest chain that exists): both kernels are exact
  and closed at depth 1 (43, OCCT truth 43), then at depth 2 return `27.6667` with **6 naked
  edges** where the truth is 29, and at depth 3 `12` with 14 naked where the truth is 14.
* `C1_box_fuse`: exact and closed at depth 1 (107 = truth); depth 2 `143.667` vs truth 145,
  6 naked; depth 3 `185.333` vs 185.25, 21 naked + 1 non-manifold.
* Chains that start curved (`C2`, `C3`, `C5`) never produce a closed solid at all — they are
  already open at depth 1, so "depth 3" is not reached in any meaningful sense.
* OCCT completes **all 15** truth chain steps as `valid=1` solids (`out/c_truth.log`).
* `C4_foreign_cut` is the same chain as `C1_box_cut` with the first operand replaced by the
  OCCT-authored box STEP. **cur reproduces C1 exactly** (43 / 27.6667 / 12) — so for the
  *current* kernel a foreign planar operand is free. **v2 does not**: 46 / 30.6667 / 15, open at
  depth 1, where it is exact on the identical own-authored box. Same signature as `box_box`
  in (a).

Every chain step is in memory, so no `main_23` STEP write is involved and defect D2 cannot be
the cause.

---

## 6. WHAT THIS MEANS

### The gate
**Not met, and not meetable from inside this ownership line.** (d) fails; the two defects that
make it fail live in `src/file_step.cpp`, which belongs to the v1 agent. I stopped at
characterising them to the line number rather than editing the file.

### Is v2 >= current on the interop axis?  NO.
The plan's kill criterion is that v2 must be >= current on EVERY pair. On foreign operands it is
not, and the counter-example is the *easiest* pair in the matrix:

| box × box, OCCT-authored operands, truth 43 / 21 / 107 | cur | v2 |
|---|---|---|
| cut | **43, closed=1** (exact) | **0.333333, closed=0** (99.2 % error) |
| common | **21, closed=1** (exact) | 21.6667, closed=0 |
| fuse | **107, closed=1** (exact) | 64.3333, closed=0 |

The identical geometry authored by *us* gives 43 / 21 / 107 closed in **both** kernels, and both
operands import with `vol=64, resid=0, closed=1`. So this is purely the interop axis, on six
planes. `C4_foreign_cut` in (c) reproduces it independently.

Scored over the whole (a) matrix, with SUCCESS = `closed() && |Δvol|/vol < 1e-6`:

| | cells | closed | SUCCESS | FALSE_PASS | empty | timeout |
|---|---|---|---|---|---|---|
| cur, foreign | 30 | 6 | **3** | 3 | 2 | 0 |
| cur, control | 28 | 6 | **3** | 0 | 0 | 2 |
| v2, foreign | 30 | 3 | **0** | 3 | 2 | 0 |
| v2, control | 28 | 6 | **6** | 0 | 0 | 2 |

**v2 is better than cur on our own geometry (6 vs 3) and worse on OCCT's (0 vs 3).** Where v2
wins it wins big — `sphere × sphere` control: v2 `4.2e-9 / 5.1e-9 / 1.7e-9` relative error
against cur's `1.4e-4 / 5.3e-4 / 6.1e-5`, five orders of magnitude, consistent with the
`3.03e-11` residual in the brief. Where it loses, it loses on the two easiest foreign pairs:
`box_box` (six planes) and `box_cyl` (`4.4e-2 / 6.7e-2 / 1.4e-1` against cur's
`2.1e-4 / 4.9e-4 / 1.1e-4`).

`cur == v2` (identical status/closed/faces/naked/vol) in **25/28** control and **24/30** foreign
cells in (a), and **87/87** pairs in (b). That is `v2_boolean` delegating: when its front end
refuses it calls `BRep::boolean` (`brep_v2_boolean.cpp:1255`, `stage_fail =
"v2-front-refused(delegated-to-kernel)"`, disabled by `SESSION_V2_NODELEGATE`, which was NOT
set here). So most of this pose family does not exercise v2's own pipeline; where it does
(`box_cyl` foreign, `box_box` foreign, `sph_sph` control) the numbers above are v2's own. These
poses are translation-only and are **not** the 8-pair × 20-rigid-motion ladder, so nothing here
supersedes the ladder's 104/83/132.

### Foreign operands, summarised in one line each
* **(a)** OCCT primitives: 3/30 cells succeed with cur, 0/30 with v2. The sphere operand cannot
  be read at all (D1), which kills all 9 sphere-bearing foreign cells: 3 become silent
  FALSE_PASS, 6 return empty or zero volume. Cone and torus import *exactly* and still cost
  1.7-40× the error our own primitives cost.
* **(b)** OCCT boolean results: **174/174 completed cells terminate, 0 close.** `naked_real`
  median 67 (max 138), `closure_residual` median 0.32 (max 0.94). OCCT itself is `valid=1` on
  87/87. cur and v2 are byte-identical on all 87 pairs.
* **(c)** chained booleans: both kernels stop being closed at **depth 2** on the simplest planar
  chain; OCCT is valid at all three depths.
* **(d)** round-trip: our own primitives 5/5 exact; OCCT primitives 3/5; OCCT boolean results
  0/40.

### What would move the needle first
An `else if (VERTEX_LOOP)` branch in the reader is worth more than any boolean work on this
axis: it converts 3 silently-wrong FALSE_PASS cells and 6 empty/zero-volume cells per kernel
into real measurements, and it is the difference between "we can read what OCCT and FreeCAD
write" and "we cannot read a sphere".

### The four defects, ranked by what they cost

1. **D1 `VERTEX_LOOP` (reader).** Costs correctness *silently*: an OCCT/FreeCAD sphere becomes a
   no-op operand and the boolean returns `A` with `closed=1, solids=1`. One `else if` on the loop
   type in `file_step.cpp` (≈ line 1203 and its twin at 1516) removes it, plus the pole/seam
   synthesis for a full-period face. Highest value per line of code in this report.
2. **D3 over-large trims on imported free-form faces (reader).** Costs every foreign boolean
   result: 0/40 imported solids close. Topology is already correct, so the fix is confined to
   how the 2D trim region is built for a `B_SPLINE_SURFACE_WITH_KNOTS` face.
3. **Parametrisation sensitivity of the booleans.** OCCT-authored cone/torus operands import
   exactly and still degrade our answer by 1.7-40×. Not a reader bug; the marcher/splitter
   depends on seam origin and trim form. This is the real "industry grade" gap.
4. **D2 seam-alias collapse (writer).** Costs least — OCCT heals it — but it makes our own
   export→import chain lossy for any face imported with a doubled seam.

### Method notes for whoever repeats this
* Score with `v2v::v2_verdict` **and** a volume comparison against OCCT on the identical operands.
  Closure alone produced 3 FALSE_PASS cells per kernel here (6 of 60 foreign cells), with 36 %,
  54 % and 83 % volume error.
* Report trivially-empty cells separately (`REFERENCE_z15_common`, and every `status=EMPTY`).
* Make each cell its own process with a `timeout`, and `fflush` every line, or a killed cell
  reports as "never started".
* `read_file_step_breps` returns **all** solids in the file. Score them all.
