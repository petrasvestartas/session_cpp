# OCCT interop hunt — can our kernel consume foreign B-Reps?

**Verdict up front: NO.** Our kernel can read and re-write a *small* class of foreign STEP
solids losslessly, but it cannot boolean them. Every OCCT-authored operand tested — including
a 6-face axis-aligned box, and including our own kernel's STEP output fed back — produced an
open, non-solid, wrong-volume result, while the *identical geometry* built by our own
`create_*` code booleans exactly. Two independent defects were isolated, both upstream of the
boolean algorithm; one of them (edge-topology shredding on import) is visible with **no boolean
at all**.

The sharpest single number: take `chair0.stp`, the model the kernel gets exactly right, and pass
it through OCCT's STEP writer with **no geometric change**. `chair0 cut chair1` goes from
`35 faces / solid / 0 naked / 46.8114` (exact) to `31 faces / not solid / 40 naked / 23.4416`.
Nothing changed but the file's author.

Session: interop battery, `/home/petras/hunt_interop/`. Binary: `build/main_7_iop`
(byte copy of `build/main_7` as of 2026-07-26 00:20; `src/` untouched).

---

## 0. Method and metrics

- **Kernel driver**: `SESSION_CHAIRS=<dir> ./build/main_7_iop SKIPMATRIX` with
  `SESSION_NO_ROT=1`, reading `<dir>/chair0.stp` and `<dir>/chair1.stp` as operands A and B.
  `SESSION_OP=none` skips all three booleans and yields a pure read+`volume()` report.
  All runs capped with `ulimit -v 4194304` and `timeout` ≤ 880 s.
- **Independent measurement**: FreeCAD 1.1.1 headless (= OCCT), scripts in
  `/home/petras/hunt_interop/` (`fclib.py`, `fcprobe.py`, `split3.py`, `shells.py`,
  `measure_all.py`, `occt_refs.py`).
- **Naked-edge metric**: wire-occurrence counting over `Wire.OrderedEdges`, with degenerate
  edges excluded. `OrderedEdges` lists a seam edge twice, so seams reach occurrence 2 and are
  **not** false-positived. Verified on the OCCT primitives: sphere/cylinder/cone/torus all
  report `naked=0` with this metric versus `naked=3/1/1/2` with ancestor counting (`nakedA`,
  also reported for comparison with the older scripts). Wire-occurrence is what every table
  below uses.
- **`isClosed()` caveat**: OCCT's `Shell.isClosed()` echoes the `CLOSED_SHELL` declaration in
  the file rather than recomputing it. Our writer always emits `CLOSED_SHELL`, so shells we
  wrote can report `closed=True` while carrying dozens of naked edges (e.g. §1 P10 shell0:
  `closed=True naked=39`). **Naked count and `isValid()` are the load-bearing metrics; ignore
  `closed` on our own files.**
- **Kernel-reported volume is not evidence.** `BRep::volume()` disagrees with OCCT on every
  imported analytic solid tested (§2), so all volume verdicts below are OCCT's measurement of
  the written STEP, not the kernel's printout. Kernel printouts are shown separately and
  labelled.

---

## 1. ROUND-TRIP IDENTITY (test 4) — run first, because it confounds everything else

Import an OCCT solid, export it from our kernel with **no boolean applied**, re-import with
OCCT. The driver always writes operand A and operand B into its 3-body output STEP, so pairing
each subject with a disjoint dummy box gives a clean read→write round-trip.

`shell0` below is our kernel's re-export of operand A. Nothing but the STEP reader and writer
touched it.

| operand | author | OCCT original: faces / edges / naked / vol | our re-export: faces / edges / naked / vol | verdict |
|---|---|---|---|---|
| `chair0.stp` | (project's own real-world model) | 20 / 54 / 0 / 80.296907 | 20 / 54 / **0** / 80.296832 | **PASS** (bbox drift 0, valid) |
| `prims/box.step` | OCCT (`Part.makeBox`) | 6 / 12 / 0 / 1000.000000 | 6 / 12 / **0** / 1000.000000 | **PASS** (bbox drift 0) |
| `prims/cyl.step` | OCCT (`makeCylinder`) | 3 / 3 / 0 / 565.486678 | 3 / 5 / **0** / 565.486678 | **PASS** (bbox drift 0) |
| `prims/con.step` | OCCT (`makeCone`) | 3 / 3 / 0 / 454.483737 | 3 / 5 / **0** / 454.483737 | **PASS** (bbox drift 4.4e-15) |
| `prims/tor.step` | OCCT (`makeTorus`) | 1 / 2 / 0 / 473.741011 | 1 / 4 / **0** / 473.741011 | **PASS** (bbox drift 0) |
| `prims/sph.step` | OCCT (`makeSphere`) | 1 / 3 / 0 / 904.778684 | **no body written at all** | **LOST** |
| `REFERENCE_z90_cut.step` | OCCT boolean result | 36 / 108 / 0 / 66.993404 | 36 / **181** / **49** / n/a (open) | **FAIL** |
| `REFERENCE_z30x20_cut.step` | OCCT boolean result | 32 / 97 / 0 / 54.257681 | 32 / **141** / **31** / n/a (open) | **FAIL** |
| `REFERENCE_z45_common.step` | OCCT boolean result | 28 / 78 / 0 / 21.100948 | 28 / **95** / **29** / 2.024570 | **FAIL** |
| `REFERENCE_y30_cut.step` | OCCT boolean result | 32 / 96 / 0 / 46.958863 | 32 / **108** / **39** / 27.389135 | **FAIL** |
| `REFERENCE_x20_cut.step` | OCCT boolean result | 38 / 103 / 0 / 80.296449 | 38 / **131** / **39** / 137.393765 | **FAIL** |
| `REFERENCE_x13y29_cut.step` | OCCT boolean result | 29 / 90 / 0 / 48.472832 | 29 / **142** / **61** / 146.029528 | **FAIL** |
| `REFERENCE_z30x20_common.step` | OCCT boolean result | 20 / 55 / 0 / 26.039372 | 20 / **79** / **34** / 30.690633 | **FAIL** |

**This is the headline defect.** No boolean ran. The face count survives exactly; the **edge
count inflates** (108→181, 97→141, 96→108, 103→131, 90→142, 78→95, 55→79) and the shell comes
back with 29–61 naked edges. The signature is a reader that fails to *share* edges between
adjacent faces —
it creates per-face edge copies that OCCT then sees as free boundaries. Our kernel's own
`is_solid()` returns 1 for all of these, i.e. our closure test does not detect the breakage.

Two secondary reader defects, both from the same probe:

- **`SPHERICAL_SURFACE` is not survivable.** All three spheres tested (`sph`, `cut3`, `chsph`)
  import as a 1-face **non-solid** with `volume()==0`, and are dropped entirely on export.
- **Planar-face surfaces are inflated on import.** Reading `prims/box.step` (a
  [0,10]³ box) and writing it back yields `PLANE` origins such as
  `CARTESIAN_POINT('',(0.,10.1,-0.1))` — the underlying surface patch is padded ~1 % beyond
  the face boundary (0.04 on the 4-unit control box, 0.1 on the 10-unit box). Reported as a
  measured file-level observation, not a proven cause.

### 1b. The reader is dialect-dependent, not geometry-dependent

The sharpest single experiment in this report. Take `chair0.stp` — the model our kernel handles
best — read it with OCCT and write it straight back out with **no geometric change whatsoever**.
OCCT measures the re-authored file as byte-equivalent in every metric: 20 faces, 54 edges,
0 naked, valid, `vol=80.296907` (identical to the original to all printed digits). Only the STEP
file's authoring tool changed. Then read both files with our kernel (`SESSION_OP=none`, no
boolean):

| file | OCCT's measurement of the file | our kernel's `volume()` |
|---|---|---|
| `chair0.stp` (as shipped) | 20 f / 54 e / 0 naked / 80.296907 | **80.3011** |
| `chair0.stp` re-authored by OCCT | 20 f / 54 e / 0 naked / 80.296907 | **19.2149** |
| `chair1.stp` (as shipped) | 20 f / 54 e / 0 naked / 80.296902 | **80.2988** |
| `chair1.stp` re-authored by OCCT | 20 f / 54 e / 0 naked / 80.296902 | **52.1473** |

Same geometry, same face count, same topology as OCCT sees it — our kernel's reading changes by
a factor of 4. **A pure format normalisation is enough to break the reader.** Every "our kernel
handles the chairs" result in the project rests on one specific STEP dialect.

And it carries straight into the boolean. `ctrl/Cchair` is the project's flagship exact case —
`chair0 cut chair1` — with **only** operand A swapped for its OCCT-re-authored twin:

| operand A | operand B | result: faces / solid / naked / vol (kernel) | result, OCCT-measured |
|---|---|---|---|
| `chair0.stp` as shipped | `chair1.stp` | 35 / **1** / **0** / 46.8114 | 35 f / **0 naked** / valid / 46.793117 |
| `chair0.stp` re-authored by OCCT | `chair1.stp` (same file) | 31 / **0** / **40** / 23.4416 | 11 f / **40 naked** / open |

Note the re-authored operand *itself* still round-trips cleanly through our reader and writer
(OCCT measures our re-export at 20 f / 54 e / 0 naked / 80.296640). The geometry survives; the
kernel's interior interpretation of it does not — `volume()` says 19.2149 and the boolean loses
the solid. This is the same signature as the box in §3 and the chain in §6.

**Consequence for the rest of this report — stated plainly because it matters:** interop results
on the OCCT *reference* solids (§5) **are confounded**. Those operands are already damaged before
any boolean starts, so §5 measures reader + boolean together and cannot apportion blame between
them. Interop results on the OCCT *primitives* (§4) and the box controls (§3) are **not**
confounded: box, cylinder, cone and torus round-trip bit-exactly through our reader and writer,
so their boolean failures are boolean failures. The chain (§6) is likewise unconfounded at
depths 2 and 3 — both operands round-trip cleanly there.

---

## 2. Reader fidelity: `volume()` on imported solids

Pure read, `SESSION_OP=none`, no boolean. Kernel printout vs OCCT truth.

| operand | OCCT vol | kernel `volume()` | kernel `is_solid()` | ratio |
|---|---:|---:|:---:|---:|
| box | 1000.000000 | 1000.0000 | 1 | 1.0000 |
| box2 | 1000.000000 | 1000.0000 | 1 | 1.0000 |
| cut1 (box) | 320.000000 | 320.0000 | 1 | 1.0000 |
| chbox (box) | 216.000000 | 216.0000 | 1 | 1.0000 |
| chair0.stp | 80.296907 | 80.3011 | 1 | 1.00005 |
| cyl | 565.486678 | 188.6087 | 1 | **0.3336** |
| cut2 (cyl) | 251.327412 | 83.8701 | 1 | **0.3337** |
| chcyl | 274.889357 | 91.7082 | 1 | **0.3336** |
| con (frustum) | 454.483737 | 64.9262 | 1 | **0.1429** |
| tor | 473.741011 | 1379.3351 | 1 | **2.9116** |
| sph | 904.778684 | 0.0000 | **0** | **0** |
| cut3 (sph) | 113.097336 | 0.0000 | **0** | **0** |
| chsph (sph) | 268.082573 | 0.0000 | **0** | **0** |
| REFERENCE_z90_cut | 66.993404 | 98.5444 | 1 | **1.471** |
| REFERENCE_z30x20_cut | 54.257681 | 40.4825 | 1 | **0.746** |
| REFERENCE_z30x20_common | 26.039372 | 122.2875 | 1 | **4.696** |
| REFERENCE_y30_cut | 46.958863 | 40.4179 | 1 | **0.861** |
| REFERENCE_z45_common | 21.100948 | 64.9297 | 1 | **3.077** |
| REFERENCE_x20_cut | 80.296449 | 111.4040 | 1 | **1.387** |
| REFERENCE_z90_fuse | 147.290289 | 55.6507 | 1 | **0.378** |
| REFERENCE_x13y29_cut | 48.472832 | 101.5997 | 1 | **2.096** |
| chair0.stp re-authored by OCCT | 80.296907 | 19.2149 | 1 | **0.239** |
| chair1.stp re-authored by OCCT | 80.296902 | 52.1473 | 1 | **0.649** |
| our own depth-1 chair result (§6) | 46.793247 | 3.8138 | 1 | **0.082** |

The cylinder ratio is exactly 1/3 (πr²h/3 vs πr²h) and the kernel's *boolean result* volume for
a no-op cut on the cylinder printed 188.4956 = π·9·20/3 exactly — the imported cylinder is being
volume-integrated as a cone. Yet the same brep exports to STEP with OCCT-exact geometry (§1),
so this is a `volume()` defect, not a geometry defect, for cyl/con/tor. For the OCCT *reference*
solids the geometry is broken too, so both apply.

**Never quote a kernel volume on imported geometry as evidence.**

---

## 3. THE DECISIVE CONTROL — identical geometry, internal vs imported

The kernel's own primitive matrix builds `box` (side 4, centred) and `box2` (side 2, centred,
translated +2 x). I authored the *same two boxes* in FreeCAD (`Part.makeBox(4,4,4,(-2,-2,-2))`
and `Part.makeBox(2,2,2,(1,-1,-1))`), exported to STEP, and fed them in through the STEP path.
OCCT's answer on those files, our internal answer, and our imported answer:

| variant | operand read (A/B vol) | cut faces / solid / naked / vol | common | fuse |
|---|---|---|---|---|
| **OCCT truth** | 64 / 8 | 11 / solid / 0 / **60.000000** | 6 / solid / 0 / **4.000000** | 11 / solid / 0 / **68.000000** |
| **built internally** (`main_7_iop box`, no STEP) | — | 11 / solid / — / **60.0000** rel 2.3e-15 | 6 / solid / **4.0000** | 11 / solid / **68.0000** |
| **imported, OCCT `PLANE` file** (`ctrl/Csame`) | 64.0000 / 8.0000 (exact) | 7 / **not solid** / **10 naked** / 53.3333 | 3 / not solid / 10 / 10.6667 | 9 / not solid / 10 / 61.3333 |
| **imported, NURBS-converted file** (`ctrl/Cnurbs`, `toNurbs()`) | 64.0000 / 8.0000 (exact) | 10 / **not solid** / **8 naked** / 57.3333 | 6 / not solid / 8 / 14.6667 | 6 / not solid / 8 / 57.3333 |
| **imported, our own kernel's export of the same box** (`ctrl/Cown`) | 64.0000 / 8.0000 (exact) | 7 / **not solid** / **10 naked** / 53.3333 | 3 / not solid / 10 / 10.6667 | 9 / not solid / 10 / 61.3333 |

Independently confirmed by OCCT on the written results (`measure_all.py`): every imported
variant's result body is a set of **open shells** (`Csame_cut` 7 faces / 2 shells / 2 open /
10 naked; `Cown_cut` byte-identical numbers).

Three things this control establishes:

1. The failure is **not** the geometry, the configuration, or the operand quality — the
   operands read back with *exact* volumes (64.0000 / 8.0000) and the internal build of the
   same boxes is exact to 2e-15.
2. The failure is **not** the analytic-vs-NURBS surface entity. Converting every `PLANE` to
   `B_SPLINE_SURFACE_WITH_KNOTS` before export changes the numbers but not the outcome.
3. The failure is **not** escaped by using our own kernel as the author. `ctrl/Cown` is our
   kernel's own STEP export of the same box, fed back in; it reproduces the imported failure
   exactly, digit for digit (53.3333 / 10.6667 / 61.3333, 10 naked). *Our kernel cannot consume
   its own STEP output.* (Caveat: that file passed through OCCT's writer during extraction from
   the 3-body output — what it isolates is that the geometry as **our reader reconstructs it**
   is unusable, whichever tool then serialises it.)

Note this does **not** contradict §1b. For the box, both dialects fail; for the chairs, the
shipped dialect works and the OCCT dialect does not. The reader's behaviour depends on the
input dialect *and* is wrong for the whole analytic-primitive class regardless of dialect.

---

## 4. OCCT PRIMITIVES AS OPERANDS (test 3)

All operands are OCCT-authored (`Part.make*` → STEP). OCCT's answer on the identical files is
the reference. Our result body was isolated from the 3-body output and measured by OCCT.

| case | pair | op | OCCT ref: faces / solids / naked / vol | ours (OCCT-measured): faces / shells(open) / naked / vol | verdict |
|---|---|---|---|---|---|
| Q06 | box × box2 | cut | 9 / 1 / 0 / 875.000000 | 12 / 2(0) / 0 / **2000.000000** | WRONG (returned A∪B) |
| Q06 | | common | 6 / 1 / 0 / 125.000000 | 6 / 1(0) / 0 / **1000.000000** | WRONG (returned B) |
| Q06 | | fuse | 12 / 1 / 0 / 1875.000000 | 6 / 1(0) / 0 / **1000.000000** | WRONG (returned A) |
| Q01 | box × sph | cut | 7 / 1 / 0 / 782.706508 | 6 / 1(0) / 0 / 1000.000000 | WRONG (B lost on import) |
| Q01 | | common | 5 / 1 / 0 / 217.293492 | **EMPTY** | WRONG |
| Q01 | | fuse | 7 / 1 / 0 / 1687.485192 | 6 / 1(0) / 0 / 1000.000000 | WRONG |
| Q02 | box × cyl | cut | 7 / 1 / 0 / 717.256661 | 2 / 2(**2**) / 8 / n/a | OPEN |
| Q02 | | common | 3 / 1 / 0 / 282.743339 | 6 / 2(**2**) / 8 / n/a | OPEN |
| Q02 | | fuse | 10 / 1 / 0 / 1282.743339 | 3 / 3(**3**) / 8 / n/a | OPEN |
| Q03 | cyl × con | cut | 7 / 2 / 0 / 272.271363 | 5 / 3(**2**) / 4 / n/a | OPEN |
| Q03 | | common | 4 / 1 / 0 / 293.215314 | 4 / 2(**1**) / 4 / n/a | OPEN |
| Q03 | | fuse | 6 / 1 / 0 / 726.755101 | 2 / 2(**2**) / 4 / n/a | OPEN |
| Q04 | sph × tor | cut | 3 / 1 / 0 / 767.538805 | **EMPTY** | WRONG (A lost on import) |
| Q04 | | common | 4 / 1 / 0 / 137.239774 | **EMPTY** | WRONG |
| Q04 | | fuse | 2 / 1 / 0 / 1241.280276 | 1 / 1(0) / 0 / 473.741011 | WRONG (returned B) |
| Q05 | box × tor | cut | 11 / 1 / 0 / 847.432257 | 3 / 3(**2**) / 8 / n/a | OPEN |
| Q05 | | common | 5 / 1 / 0 / 152.567533 | 5 / 2(**1**) / 10 / n/a | OPEN |
| Q05 | | fuse | 12 / 1 / 0 / 1321.173269 | 2 / 2(**2**) / 8 / n/a | OPEN |
| Q07 | sph × cyl | cut | 3 / 1 / 0 / 867.993385 | **EMPTY** | WRONG (A lost on import) |
| Q07 | | common | 3 / 1 / 0 / 36.784780 | **EMPTY** | WRONG |
| Q07 | | fuse | 4 / 1 / 0 / 1433.480194 | 3 / 1(0) / 0 / 565.486678 | WRONG (returned B) |

**0 of 21 cells correct.** Not one OCCT-authored primitive pair produced the right answer. Every
sphere case degenerates to "one operand does not exist"; every other case produces open shells
or the untouched operand.

Q06 deserves a note because it is the simplest possible boolean in solid modelling — two
axis-aligned overlapping boxes, both read back with exact volume. Our kernel returned
`fuse = A`, `common = B`, `cut = A ∪ B` (as two disjoint solids), i.e. it classified
`B ⊂ A` and never found the intersection at all. B is `[5,15]³`, A is `[0,10]³`.

---

## 5. OCCT RESULT AS OPERAND (test 1)

Operands are OCCT's own boolean results on the chair operands, from
`/home/petras/fc_inspect/REFERENCE/`. **Read §1 first: these operands do not survive our STEP
reader**, so these rows measure the compounded failure, not the boolean in isolation.

Reference answers computed with FreeCAD on the identical files (`refs/occt_refs_ALL.json`).

| case | A | B | oracle | op | OCCT ref: faces / solids / naked / vol | ours (kernel print): faces / solid / naked / vol |
|---|---|---|---|---|---|---|
| P01 | REFERENCE_z90_cut | chair0.stp | A ⊂ B exactly | cut | **EMPTY** | 37 / **0** / **58** / 328.5486 |
| | | | | common | 36 / 1 / 0 / 66.993404 | 19 / **0** / **55** / 152.1062 |
| | | | | fuse | 46 / 1 / 0 / 80.296732 | *no result in 880 s* |
| P02 | REFERENCE_z30x20_cut | REFERENCE_z30x20_common | A ⊔ B partition chair0 | cut | 32 / 1 / 0 / 54.257681 | 26 / **0** / **33** / 79.5213 |
| | | | | common | **EMPTY** | 6 / **0** / **31** / 37.7980 |
| | | | | fuse | 36 / 1 / 0 / **80.296956** (= chair0) | *no result in 880 s* |
| P03 | REFERENCE_z30x20_cut | chbox (OCCT box) | — | cut | 40 / 2 / 0 / 28.734421 | **0 / 0 / 0 / 0.0000 (empty)** |
| | | | | common | 31 / 2 / 0 / 25.523384 | 32 / **0** / 0 / 40.4825 (= A untouched) |
| | | | | fuse | 39 / 1 / 0 / 244.734327 | 6 / 1 / 0 / 216.0000 (= B untouched) |
| P04 | REFERENCE_y30_cut | chair0.stp | A ⊂ B exactly | cut | **EMPTY** | 31 / **0** / **83** / 16.6494 |
| | | | | common | 32 / 1 / 0 / 46.958863 | 25 / **0** / **81** / 130.9388 |
| | | | | fuse | 38 / 1 / 0 / 80.296561 | *no result in 880 s* |
| P05 | REFERENCE_z45_common | chair0.stp | A ⊂ B exactly | cut | **EMPTY** | 33 / **0** / **53** / 49.3659 |
| | | | | common | 28 / 1 / 0 / 21.100948 | *no result in 880 s* |
| | | | | fuse | 43 / 1 / 0 / 80.297478 | *no result in 880 s* |
| P10 | REFERENCE_x20_cut | chcyl (OCCT cyl) | — | cut | 38 / 3 / 0 / 50.854908 | **0 / 0 / 0 / 0.0000 (empty)** |
| | | | | common | 37 / 1 / 0 / 29.439362 | 38 / **0** / 0 / 24.5477 |
| | | | | fuse | 38 / 1 / 0 / 325.739456 | 3 / 1 / 0 / 91.6298 (= B untouched) |
| P06 | REFERENCE_z90_fuse | chair0.stp | B ⊂ A exactly | cut | 47 / 2 / 0 / 66.993897 | 54 / **0** / **90** / 231.1985 |
| | | | | common / fuse | 46 / 53 faces, 80.296513 / 147.290289 | *no result in 880 s* |
| P07 | REFERENCE_x13y29_cut | chair1.stp | — | cut | 51 / 2 / 0 / 40.405162 | 30 / **0** / 0 / 101.6143 |
| | | | | common | 34 / 3 / 0 / 8.067970 | 1 / **0** / **3** / 0.0146 |
| | | | | fuse | 61 / 1 / 0 / 120.702449 | 53 / **0** / **18** / 124.5472 |
| P08 | REFERENCE_z30x20_common | REFERENCE_z45_common | — | cut | 38 / 5 / 0 / 10.202602 | 21 / **0** / **32** / 208.6167 |
| | | | | common / fuse | 30 / 76 faces, 15.836727 / 31.303482 | *no result in 880 s* |
| P09 | REFERENCE_z90_cut | chsph (OCCT sph) | — | cut | 37 / 3 / 0 / 27.757069 | 37 / **0** / 0 / 94.7604 |
| | | | | common | 37 / 3 / 0 / 39.236546 | 1 / **0** / 0 / 0.0000 |
| | | | | fuse | 32 / 1 / 0 / 295.836947 | 36 / **0** / 0 / 94.7604 |

P09's operand B is a sphere, which does not survive import at all (§1), so P09 measures the
sphere defect rather than the boolean. The kernel prints `naked 0 solid 0` for its P09 and P10
results while OCCT measures the corresponding shells at 48 and 30 naked edges — the kernel's
own naked counter also disagrees with an independent reader on imported geometry.

The oracle-free identities (P01, P02, P04, P05) are the cleanest statements available and all
fail:

- **P01/P04/P05**: A is by construction a subset of chair0, so `A cut chair0` must be empty and
  `A fuse chair0` must be exactly chair0 (80.2969). OCCT confirms both on the same files. Ours
  returns 31–37-face open shells with 53–83 naked edges and volumes 16.6–328.5.
- **P02**: `REFERENCE_z30x20_cut` and `REFERENCE_z30x20_common` partition chair0
  (54.257681 + 26.039372 = 80.297053). OCCT's fuse of the two files reconstructs
  **80.296956** and their common is **empty** — the oracle is self-verifying. Our kernel
  produced 33 and 31 naked edges and never finished the fuse.

Cells marked *no result in 880 s* did not complete inside the timeout on a machine averaging
load 28 (32 cores, three other sessions running). They are **not measured** — neither a pass
nor a fail.

Every P-case operand's own round-trip is in §1: shell0 of each output file is our re-export of
operand A, and it carries 29–61 naked edges in every case. Reader-level volumes for the
remaining operands: REFERENCE_z90_fuse 55.6507 (truth 147.290289),
REFERENCE_x13y29_cut 101.5997 (truth 48.472832), REFERENCE_z30x20_common 122.2875
(truth 26.039372), REFERENCE_x20_cut 111.4040 (truth 80.296449).

---

## 6. CHAINED BOOLEANS (test 2)

Chain: `chair0 cut chair1` → `cut chair1_tx` → `cut chair1_ty`, where `chair1_tx/ty` are
`chair1.stp` translated (1.5,0,0) and (0,1.5,0), authored via OCCT. At each depth our result
body is extracted and re-imported as the next operand. Pure-OCCT reference chain computed on
the same operand sequence (`occt_chain.py`).

| depth | operation | OCCT reference on the same operands: faces / solids / naked / vol | ours, kernel print: faces / solid / naked / vol | ours, OCCT-measured: faces / naked / vol | verdict |
|---|---|---|---|---|---|
| 1 | chair0 cut chair1 | 35 / 1 / 0 / **46.794109** | 35 / 1 / 0 / 46.8114 | 35 / **0** / **46.793117** (valid) | **PASS**, rel 2.1e-5 |
| 2 | D1 cut chair1_tx | 46 / 3 / 0 / **35.516115** | 55 / 1 / 0 / 49.4699 | 55 (=35+20) / **0** / **127.089384** | **FAIL** (silent) |
| 3 | D2 cut chair1_ty | 49 / 3 / 0 / 32.637384 | 35 / 1 / 0 / 15.9058 | 35 / **0** / **46.792501** (= A) | **FAIL**, no-op again |

**Depth 1 is essentially exact** — 35 faces, closed, OCCT-valid, volume within 2.1e-5 relative
of OCCT's answer on the same two files. This is the case the kernel is tuned for, and it holds.

**Depth 2 fails silently — the worst failure mode in the report.** The kernel reports
`faces 55 solid 1 naked 0 vol 49.4699`: closed, manifold, plausible. OCCT's measurement of the
written result shows what it actually is — two shells of 35 and 20 faces with volumes
46.792853 and 80.296531, i.e. **operand A and operand B, both untouched**, total 127.089384.
The boolean was a no-op dressed up as a solid. OCCT's answer on the identical operand files is
46 faces / 3 solids / 35.516115.

The operands themselves round-trip *cleanly* at this depth (OCCT measures our re-exports at
35 f / 110 e / 0 naked / 46.792626 and 20 f / 54 e / 0 naked / 80.296643) — but the kernel's
internal reading of them is wrong:

```
A: faces 35 solid 1 vol 3.8138 | B: faces 20 solid 1 vol 53.2837
```

3.8138 against a true 46.7931 (12× low), and 53.2837 against a true 80.2969 (34 % low). This
is the same signature as Q06 in §4: geometry ingested correctly, interior classification wrong,
so the boolean concludes there is nothing to intersect and hands back its inputs.

Depth 3 was run for completeness. It repeats the pattern: operands read as
`A 35 f vol 15.9058 | B 20 f vol 48.3284` (true 46.79 and 80.30) and the cut returns
`faces 35 solid 1 naked 0 vol 15.9058` — operand A handed straight back, again reported as a
clean solid. It also exposes a further interop limit: the depth-2 output is a **2-solid**
compound and the driver's reader takes only `as[0]`, so half the model was silently dropped
before the boolean even started. Multi-solid boolean results — which OCCT produces routinely
(6 of the 30 shipped references are compounds with 2–3 solids) — cannot be fed back as a
single operand at all.

**Chain-depth failure point: 2.** Depth 1 exact, depth 2 wrong and undetectable from inside the
kernel, depth 3 wrong the same way. An **in-process** chain (no STEP round-trip between steps)
was not part of this battery and is the obvious follow-up: the defect isolated here lives in
the file-interchange path, so an in-memory chain may well go deeper.

---

## 7. What is and is not broken

| capability | status | evidence |
|---|---|---|
| read + re-write a simple OCCT analytic solid (box/cyl/cone/torus) | **works**, OCCT-exact | §1 |
| read + re-write the project's own `chair0.stp` | **works**, rel 9.3e-7 | §1 |
| read the same `chair0.stp` after OCCT re-authors the file | **wrong** — `volume()` 19.2149 vs 80.3011 | §1b |
| boolean the flagship `chair0 cut chair1` with A re-authored by OCCT | **35 f solid → 11 f open, 40 naked** | §1b |
| read a `SPHERICAL_SURFACE` solid | **lost entirely** (non-solid, vol 0, dropped on export) | §1, §2 |
| read + re-write a multi-face OCCT boolean result | **shredded** — edge count inflates 1.2–1.7×, 29–61 naked edges | §1, §5 |
| `volume()` on an imported analytic solid | **wrong** (cyl 1/3×, cone 0.14×, torus 2.9×) | §2 |
| `volume()` on an imported OCCT boolean result | **wrong** by 0.38×–4.7× | §2, §5 |
| `is_solid()` on an imported broken solid | **false negative** — returns 1 on shells OCCT counts 29–61 naked edges on | §1 |
| kernel's own naked-edge counter on imported results | **false negative** — prints `naked 0` where OCCT counts 30–48 | §5 |
| boolean two OCCT-authored primitives | **0/21 cells correct** | §3, §4 |
| boolean two of our own STEP exports | **fails identically to OCCT input** | §3 |
| boolean the same geometry built internally | **exact** (`main_7_iop box` filter: 15/15 cells OK) | §3 |
| chain depth 1 (STEP round-trip of our own chair result) | **exact**, 2.1e-5 | §6 |
| chain depth 2 | **silently wrong** — returns A ∪ B, reports `solid 1 naked 0` | §6 |
| feed a multi-solid result back as one operand | **impossible** — driver reads only `as[0]` | §6 |

## 8. Verdict

**The kernel only works on geometry it authored itself, plus one imported file family it has
been tuned against.** Every boolean on a foreign operand tested here failed, and the failure is
reproducible with the simplest possible input — two overlapping axis-aligned boxes whose volumes
the kernel reads back exactly. The same two boxes built by `create_box` boolean exactly, so this
is not a boolean-algorithm limitation; it is an interchange-path defect sitting upstream of it.

Because the same failure appears when the kernel is fed **its own STEP output**, this is not
"OCCT compatibility" — it is a self-consistency failure. Any workflow that writes an
intermediate to disk and reads it back is broken today, which includes every realistic CAD
pipeline and the chained-boolean use case at depth 2.

The most alarming property is not that it is wrong but that it is **quietly** wrong. At chain
depth 2 the kernel reported `faces 55 solid 1 naked 0` for a result that is literally its two
inputs stacked side by side; on the P09/P10 references it reported `naked 0` for shells OCCT
counts 30–48 naked edges on. The internal validity gates cannot see the damage they exist to
catch, so no amount of running the existing suites would have surfaced any of this — which is
exactly why the gap the user identified went unnoticed.

Priority order implied by the measurements:

1. **Edge sharing on import.** Faces survive, edges do not (108→181 on a 36-face solid; 90→142
   on a 29-face solid). Visible with no boolean running; almost certainly the root of §3–§6.
2. **Interior classification on imported breps.** Where the geometry *does* round-trip exactly
   (box, cylinder, cone, torus, our own depth-1 chair result), the kernel still reads the wrong
   volume and the boolean concludes "no interference". `volume()` reporting exactly πr²h/3 for
   an imported cylinder points at the same machinery the boolean's point-in-solid test uses.
3. **`SPHERICAL_SURFACE` import.** A whole surface type is silently dropped.
4. **Validity gates.** `is_solid()` and the kernel's naked-edge counter both return clean on
   demonstrably open results. Until these agree with an independent reader, no interop result
   from inside the kernel can be trusted — including any future fix's own regression numbers.

Suggested first regression test, because it runs in seconds and fails today:
`SESSION_OP=none SESSION_CHAIRS=/home/petras/hunt_interop/rd2/chair0_occt` must report
`vol 80.2969`, not 19.2149. Second: `SESSION_CHAIRS=/home/petras/hunt_interop/ctrl/Csame` must
give cut 60 / common 4 / fuse 68, matching what `./build/main_7 box` already gives for the same
two boxes built internally.

## 9. Reproduction

```
# reader fidelity (seconds per solid, no boolean)
SESSION_OP=none SESSION_NO_ROT=1 SESSION_CHAIRS=/home/petras/hunt_interop/rd/cyl \
  ./build/main_7_iop SKIPMATRIX

# the decisive control
SESSION_NO_ROT=1 SESSION_CHAIRS=/home/petras/hunt_interop/ctrl/Csame ./build/main_7_iop SKIPMATRIX
./build/main_7_iop box            # same geometry, built internally -> 15/15 OK

# independent measurement
cd /home/petras/fc_inspect
FC_FILES=<a>:<b> /snap/bin/freecad.cmd /home/petras/hunt_interop/fcprobe.py
FC_FILES=<our-3-body-step> /snap/bin/freecad.cmd /home/petras/hunt_interop/shells.py
```

Artefacts: `/home/petras/hunt_interop/{prims,pairs,ctrl,rt,rd,rd2,chain2,refs,reports}/`.
