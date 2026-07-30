# v2 performance, liveness, memory and scaling — measured

**Verdict up front, all measured, nothing inferred:**

1. **The liveness gate PASSES.** Every one of the 20 historical-defect cells and all 144 ladder
   cells **terminated**. No hang, no `std::bad_alloc`, no non-termination anywhere, in either
   kernel. `coneR x cyl` — the case that produced *no output in 600 s twice* and OOMed under a
   4 GiB cap — now completes all three ops in **5.4–10.4 s** (and in **50.7–52.8 s** with
   `SESSION_AUTO=1`), with a **47 MB** peak RSS and no throw under `ulimit -v 4194304`.
2. **Memory is fine.** Peak RSS never exceeded **427 MB**, and that was the extreme 130×130-face
   synthetic; every ladder cell stayed under **77 MB**. The 8.4 GB AUTO regression is not present.
3. **Speed is the problem, and it is severe.** Over the 8-pair ladder × 3 poses × 3 ops, v2's
   median cell is **3.07 s** against the current kernel's **13.7 ms** — a **225× median
   slowdown**, worst cell **18.4 s**. On the real 20-face chairs, v2 takes **23.1–29.2 s** where
   the current kernel takes **2.4–2.7 s** and OCCT is quoted at 2.2–9.7 s. **v2 is 3–13× slower
   than the OCCT reference scale on the only real-world model measured.**
4. **Root cause of the slowdown is located and quantified** (40/40 stack samples + gdb breakpoint
   timing + a direct A/B measurement): v2's shell orientation stage calls the **display
   triangulator** (`BRep::face_meshes()` → `NurbsSurfaceTrimmed::mesh()` → Delaunay) on the
   *imprinted* operands, 5 times per boolean; and v2's imprint leaves trim loops ~30× denser than
   the current kernel's, which makes that triangulation ~10<sup>4</sup>× more expensive. Measured
   on `box x box` cut: identical 9-face results, but current-kernel result = 84 trim control
   points → mesh in **0.36 ms**, v2 result = 2412 trim control points → mesh in **4146 ms**.
5. **The O(n²) candidate-pair generation already matters.** Two prisms that **do not touch at all**
   (100 units apart, zero intersections) still cost **16.4 s (current) / 23.7 s (v2)** at 130+130
   faces, growing as n^1.9±0.2 — ≈**1.0–1.4 ms per candidate pair**. At 68 faces that pure
   no-op broad phase is **36 %** of the current kernel's total time for the interfering case.

---

## 0. What is mine, what I changed

Created **`main_24.cpp`** (the only file I own) and nothing else. No file under `src/`, no
`CMakeLists.txt`, no other `main_*.cpp` was touched — `CMakeLists.txt`'s target list stops at
`main_17` and is owned by another session, so `main_24` is compiled and linked **by hand** with
exactly the flags CMake emits for `main_16` (`build_perf/CMakeFiles/main_16.dir/flags.make` +
`link.txt`, driver object swapped). Build script:
`…/scratchpad/build24.sh`. Build dir: **`build_perf/`** (my own; configured 13:44, objects built
13:51).

`main_24` modes: `info base ladder one spin mp mesh splitmesh resmesh stages live livemp scale
scaledisj chairs`. Everything is one-cell-per-process where a per-cell peak RSS is wanted.

---

## 1. Method, and every reason to distrust a number

**Machine.** 13th Gen Intel Core i9-13900HX, 32 logical cores, 30 GiB RAM, Linux 7.0.0-28-generic,
g++ (Ubuntu) 15.2.0, `-O3 -DNDEBUG -march=native -std=gnu++23` (identical to every other target).

**Co-tenancy is real and is reported with every number.** Other agents build and sweep on this
box; observed load average ranged **2.2 → 68.1** during this session. Every cell therefore records
**both** wall time (`steady_clock`) and **process CPU time** (`CLOCK_PROCESS_CPUTIME_ID`), and
every run prints `/proc/loadavg`.
- In the primary ladder run (load 3.2 at start) `max(wall/cpu) = 1.0011` over 144 cells: wall time
  there **is** CPU time, the numbers are not queueing artefacts.
- But CPU time itself is not load-immune: `prism32 v2 cut` measured **20.6 s CPU** at load 28 and
  **11.6 s CPU** at load 16. Memory-bandwidth/LLC contention inflates CPU time up to ~2× on this
  machine. **Cross-run ratios are only safe when the control cell agrees.**

**Control cell** (`box x box` / cut / current kernel), run first in every sweep:
7.020 ms (ladder-1) vs 7.311 ms (ladder-3) warm median — 4 % apart, so those two runs are
comparable. The later single-shot control read 14.4–16.8 ms; that is **not** drift, it is the
cold/warm gap (see below).

**Cold vs warm.** Each `ladder`/`scale` cell reports `first_ms` (the first call in a fresh process
— cold allocator, cold I-cache, cold statics) and `median_ms` over repeats taken until 400 ms of
work accumulate (≤20 reps). Measured cold/warm ratio for the control: **≈2×** (15.6 ms cold vs
7.3 ms warm). All headline ladder numbers below are **warm medians**; all `one`/`live`/`chairs`
numbers are **cold single shots** and are labelled as such.

**Scoring.** No verdict code of my own: `src/v2/v2_verdict.h` (`v2v`) is used where a verdict is
needed, and its cost is always timed in a *separate* column from the boolean, precisely because a
previous session mistook `BRep::volume()` cost for a boolean hang.

**Empty/trivial cells.** None of the ladder pairs is trivially empty (every pair interferes); the
one deliberately non-interfering experiment is the `scaledisj` family and it is reported in its
own section, never summed with anything.

**Binaries.** `session_core` + `session_v2` objects were compiled **once**, 2026-07-26 13:51:39–46,
and never recompiled; only `main_24.cpp.o` was recompiled as modes were added, so the measured
kernel code is byte-identical across every number here. Driver sha1 per dataset:

| dataset | `build_perf/main_24` sha1 |
|---|---|
| `ladder 1`, first stack samples | `046f6ae3…` / `0d8cf17a…` |
| `mesh`, `stages` | `2e610771…`, `c3da7990…` |
| **`ladder 3` (headline timings), liveness, memory, chairs, scaling pass 1** | **`c3da79905167cc7a876300452bb6b32a1ce75ebd`** |
| `splitmesh`, `resmesh`, scaling pass 2 (`main_24_pinned`) | `716a927a…`, **`6fbf96026fcd7773d0c1cc6f67dcf2e43dc429fd`** |
| `scaledisj` | `bbf0bea24faf4a22f9c17b3d7e47f21a0f88372e` |

**Source state.** HEAD `5bb685a`, working tree dirty (other agents). My objects predate later
edits by other sessions: `src/brep.cpp` was rewritten at **14:30** and `src/v2/v2_dump.cpp` was
added at **14:21**, i.e. *after* my 13:51 build — **neither is in any number here.** v2 sources as
of my build produce `sha1(cat src/v2/*.cpp src/v2/*.h)` = `5dee69b1…` **at the time of writing**;
this is a post-hoc read and is *not* a certificate of what I compiled, so treat the mtime list as
the authority.

**Cells attempted vs completed.** ladder 144/144 · liveness 20/20 · memory 17/17 · chairs 6/6 ·
scaling crossed 12/12 · scaling disjoint 12/12. **Zero cells failed to exit.**

---

## 2. (a) Wall time per operation, 8-pair ladder × 3 poses × 3 ops × 2 kernels

`main_24 ladder 3`, sha1 `c3da7990…`, load at start **3.24**, all 144 cells `ok` (no throw, no
empty result). Poses are `main_16.cpp motion(k)` for k = 0,1,2 applied to **both** operands.
Warm medians; min/med/max are over the 9 cells (3 poses × 3 ops) of each pair.

| pair | CURRENT min/med/max (ms) | V2 min/med/max (ms) | v2/cur (median) |
|---|---|---|---|
| box x box | 7.2 / **7.6** / 8.0 | 8875 / **9197** / 9711 | **1212×** |
| sphere x sphere | 11.6 / **14.6** / 18.1 | 985 / **2165** / 9780 | 148× |
| sphere x cylinder | 3.2 / **3.4** / 7.0 | 98 / **4929** / 6288 | **1431×** |
| box x sphere | 7.7 / **35.4** / 37.2 | 105 / **579** / 3075 | 16× |
| cylinder x cylinder | 22.7 / **25.5** / 28.3 | 475 / **2425** / 3281 | 95× |
| box x cone | 11.7 / **11.8** / 13.0 | 53 / **2558** / 3068 | 217× |
| cone x cone | 326.7 / **341.7** / 350.2 | 1551 / **5370** / 7780 | 16× |
| torus x torus | 4.2 / **20.6** / 28.5 | 8356 / **14159** / 18377 | 689× |
| **all 72 cells / kernel** | 3.2 / **13.7** / 350.2 | 53 / **3072** / **18377** | **225×** |

Sum over all 72 cells: **current 4.03 s, v2 370.2 s (91.8×)**.
By operation (median over 24 cells each): cut 11.7 ms → 3178 ms · common 15.7 ms → 3091 ms ·
fuse 16.4 ms → 2969 ms. **The op does not matter; the pair does.**

**Pose sensitivity is large for v2 and small for the current kernel.** `sphere x cylinder` v2
ranges 98 ms → 6288 ms (64×) across the same 9 cells whose analytic answer is pose-invariant;
`box x box` v2 varies only 8875–9711 ms. Any single-pose v2 timing is therefore not
representative — this is why the table gives min/med/max, not a mean.

### Real-world reference: the chairs (20 faces each, STEP)

`main_24 chairs corpus/work/chairs …`, cold single shots, one process per cell, load ≈19–21.
STEP read excluded from the boolean time (it costs 0.20–0.35 s).

| op | CURRENT | V2 | v2/cur | OCCT reference (given) |
|---|---|---|---|---|
| cut | 2.529 s (35 faces) | **23.135 s** | 9.1× | 2.2–9.7 s, mean 4.6 (n=12) |
| common | 2.387 s (25 faces) | **29.225 s** | 12.2× | — |
| fuse | 2.651 s (50 faces) | **24.731 s** | 9.3× | — |

Peak RSS: current 35.5–36.1 MB, v2 44.9–46.1 MB.
**The current kernel sits inside the OCCT band (at its fast end). v2 sits 2.4–13× outside it.**

---

## 3. (b) The historical liveness defects, re-checked

One process per cell, `/usr/bin/time -v` for RSS, a gdb backtrace taken automatically if the cell
was still alive at the sample point, hard kill at the bound. Bounds used: 120 s / 180 s (default
env), 600 s (`SESSION_AUTO=1`), 300 s (4 GiB cap). Geometry is `main_7.cpp placements()` verbatim:
`coneR = cone(r2,h4) @ (0,-2.8,0) rot-x −90°`, `cyl = cylinder(r1.5,h6) @ (0,0,−3)`,
`box = box(4,4,4)`, `torR = torus(2,0.8) rot-x 45°`, `tor/tor2 = torus(2,0.8)` at 0 and (2,0,0).
`tilt21.3` = sphere r2.5 × cylinder r1 h8 through the centre, tilted 21.3° about Y.

| case | op | kernel | result | in-process ms | peak RSS |
|---|---|---|---|---|---|
| coneR × cyl | cut | current | **completes**, 4 faces | 6 842 | 46.3 MB |
| coneR × cyl | cut | v2 | completes, 5 faces | 10 442 | 19.9 MB |
| coneR × cyl | common | current | **completes**, 3 faces | 6 716 | 45.6 MB |
| coneR × cyl | common | v2 | completes, 4 faces | 10 155 | 21.4 MB |
| coneR × cyl | fuse | current | **completes**, 4 faces | 5 366 | 45.3 MB |
| coneR × cyl | fuse | v2 | completes, 3 faces | 7 675 | 19.5 MB |
| cyl × coneR (swapped) | cut | current / v2 | completes / completes | 6 074 / 9 267 | 35.1 / 33.8 MB |
| box × torR | cut | current / v2 | completes, 7 / 7 faces | 3 899 / 6 790 | 23.7 / 33.7 MB |
| box × torR | common | current / v2 | completes, 3 / 3 faces | 4 050 / 7 575 | 23.4 / 33.4 MB |
| torus × torus | cut | current / v2 | completes, 9 / 9 faces | 3 791 / 8 330 | 25.3 / 26.3 MB |
| torus × torus | fuse | current / v2 | completes, 6 / 6 faces | 4 343 / 8 776 | 25.7 / 26.1 MB |
| sph × cyl @ 21.3° | cut | current / v2 | completes, 2 / 2 faces | **19.2** / 8 700 | 11.4 / 50.4 MB |
| sph × cyl @ 21.3° | common | current / v2 | completes, 3 / 3 faces | **19.2** / 6 681 | 11.9 / 50.3 MB |

**Under `SESSION_AUTO=1`** (the variant-selection ladder, current kernel, 600 s bound):
coneR × cyl cut **50.7 s**, common **52.8 s**, fuse **51.5 s**, box × torR cut **3.9 s** —
all complete, peak RSS 46.8–47.0 MB. AUTO costs ≈7.5× on this pair and **still terminates**.

**Under `ulimit -v 4194304` (4 GiB, the original OOM experiment)**: coneR × cyl common completes
in both kernels (current 10.2 s / v2 14.6 s), **no `std::bad_alloc`**, peak RSS 45.2 / 21.4 MB.

### What this means for the four reported defects

| reported defect | status now |
|---|---|
| `coneR × cyl` hangs > 600 s on cut and fuse, `bad_alloc` on common under 4 GiB | **GONE, in both kernels.** Worst case measured 52.8 s (AUTO), 10.4 s (default). |
| `box × torus` / `torus × torus` "appeared to hang, was actually `BRep::volume()`" | **Confirmed as not-a-boolean-hang.** Both booleans complete in 3.8–8.8 s. |
| the 21.3° tilt hang "reported GONE in v2" | **Confirmed gone — in *both* kernels.** Current 19.2 ms, v2 6.7–8.7 s. Note the direction: at this tilt v2 is 350–450× *slower* than the current kernel, but it does terminate. |

**GATE (b): PASSED.** 20/20 liveness cells exited. No severity-one non-termination defect exists
in either kernel on any case I could construct, including the historical ones, including under
`SESSION_AUTO`, including under a 4 GiB address-space cap.

*Scope statement:* this clears the cases that were **reported**. It is not a proof of termination.
The longest single operation observed anywhere in this campaign is **469.8 s** (v2, 130+130-face
synthetic prisms), which is finite but would read as a hang to any harness with a 300 s budget.

---

## 4. (c) Memory: peak RSS per operation

`/usr/bin/time -v`, one process per boolean, pose 0, cut, cold. Load ≈22 throughout, so the times
in this table are inflated ≈1.6× against §2 — the RSS numbers are unaffected.
Process floor (`main_24 base`, no geometry at all): **7 320 KB**.

| pair | CURRENT peak RSS | V2 peak RSS | v2 − floor |
|---|---|---|---|
| box x box | 11 568 KB | 45 296 KB | 38.0 MB |
| sphere x sphere | 11 460 KB | **77 132 KB** | 69.8 MB |
| sphere x cylinder | 11 480 KB | 12 964 KB | 5.5 MB |
| box x sphere | 11 288 KB | 12 152 KB | 4.7 MB |
| cylinder x cylinder | 12 024 KB | 13 220 KB | 5.8 MB |
| box x cone | 11 728 KB | 12 772 KB | 5.3 MB |
| cone x cone | 12 292 KB | 15 696 KB | 8.2 MB |
| torus x torus | 11 400 KB | 70 572 KB | 61.9 MB |

Largest RSS observed anywhere in the campaign:

| workload | peak RSS |
|---|---|
| v2 cut, prism_128 × prism_128 (130+130 faces, 469.8 s) | **426 828 KB (417 MB)** |
| v2 cut, prism_66 × prism_66 (68+68 faces) | 143 564 KB |
| v2 cut, prism_32 × prism_32 | 73 772 KB |
| chairs, v2 fuse | 46 116 KB |
| coneR × cyl, current, `SESSION_AUTO=1` | 47 008 KB |

**Nothing balloons.** The prior 8.4 GB AUTO regression does not reproduce; the worst case here is
**417 MB**, and it is 20× larger than any realistic cell. The three v2 cells with elevated RSS
(sphere×sphere 77 MB, torus×torus 71 MB, box×box 45 MB) are exactly the three cells with the
densest imprint trims — see §6.

---

## 5. (d) Scaling with face count

**Family.** `prism(n)` = a regular n-gon extruded along +Z, built through `BRep::from_polylines`
as exactly **n+2 planar faces**; verified closed and manifold by `v2_verdict_topology_only`
(naked_real 0, nonmanifold 0, shells 1) for n = 4, 8, 16, 32, 66, 128. `prism(66)` is the
**68-face reference solid**. Two copies, the second rotated 90° about X so they interpenetrate.
All faces planar, so per-face geometric difficulty is held constant and only *count* varies.

`main_24 scale n <k> cut`, one process per cell. Rows n ≤ 68 are one coherent sweep with the
pinned driver `6fbf9602…` at load 15.8–16.9. The n = 130 row is **not** from that sweep: current
came from the earlier `c3da7990…` sweep at load ≈20 and v2 from a later driver (kernel objects
byte-identical, only `main_24.cpp.o` differs) finishing at load ≈16–20. **The n=130 row is the
least trustworthy row in this document** and is the only one whose growth exponent jumps. An
independent reason to distrust it: `prism32 v2` measured 22.7 s CPU at load 28 and 11.6 s CPU at
load 16 — a 1.95× swing from co-tenancy alone on the identical cell.

| faces each | CURRENT cut | V2 cut | v2/cur | result faces | v2 peak RSS |
|---|---|---|---|---|---|
| 6 | 0.115 s | 0.487 s | 4.2× | 14 | 15.7 MB |
| 10 | 0.335 s | 1.391 s | 4.2× | 26 | 23.9 MB |
| 18 | 1.336 s | 3.830 s | 2.9× | 46 | 40.0 MB |
| 34 | 3.783 s | 11.628 s | 3.1× | 90 | 73.8 MB |
| **68** | **16.107 s** | **39.094 s** | 2.4× | 198 | 143.6 MB |
| 130 | 297.800 s | 469.822 s | 1.6× | 529 | 426.8 MB |

Local log-log growth exponent (time ∝ faces^p), consecutive pairs:
- current: 2.09, 2.35, 1.64, 2.09, **4.50**
- v2: 2.06, 1.72, 1.75, 1.75, **3.84**

So **both kernels are ≈O(n²) up to 68 faces**, and both show a *steeper-than-quadratic* jump on
the last step to 130 faces. v2's **relative** penalty shrinks with size (4.2× → 1.6×): v2's
overhead is largely a **per-boolean constant** (see §6), which is why it dominates 2-face
primitives and is merely a factor on a 130-face model.

### Is the O(n²) candidate-pair generation the problem? Measured: partly, and already.

**Experiment `scaledisj`** (sha1 `bbf0bea2…`): the *same* prisms, second one translated 100 units
away, so **no pair of faces intersects and no section, split or classification work exists**.
Everything that remains is candidate-pair generation over bounding boxes plus fixed overhead.

| faces each | CURRENT disjoint | V2 disjoint | as % of the interfering case (cur / v2) |
|---|---|---|---|
| 6 | 0.039 s | 0.039 s | 34 % / 8 % |
| 10 | 0.118 s | 0.122 s | 35 % / 9 % |
| 18 | 0.445 s | 0.426 s | 33 % / 11 % |
| 34 | 1.571 s | 1.451 s | 42 % / 12 % |
| 68 | 5.783 s | 5.858 s | **36 %** / 15 % |
| 130 | **16.394 s** | **23.685 s** | 5.5 % / 5.0 % |

Growth exponents: current 2.18, 2.26, 1.98, 1.88, 1.61 · v2 2.22, 2.12, 1.93, 2.01, 2.16 —
**quadratic, cleanly, with no interference work in the loop at all.**

Quantified: at 130×130 faces there are 16 900 candidate pairs and the disjoint boolean costs
16.4 s (current) / 23.7 s (v2) ⇒ **0.97 ms / 1.40 ms per candidate pair**. That is not a
bounding-box rejection cost; a real AABB-overlap test on 16 900 pairs is microseconds total. So
the measured quadratic term is **O(n²) candidate pairs each doing ≈1 ms of real work before being
discarded**, which is exactly the failure mode a BVH (or any early box reject that actually
rejects) is meant to remove.

**Answer to "does it matter yet": yes.** On the 68-face reference solid, **36 % of the current
kernel's entire runtime is spent on face pairs that cannot possibly interfere.** Both kernels
share this cost — it is *not* a v2 regression — and it is the single cheapest large win available
for many-face models.

---

## 6. Where v2's time actually goes (the 225× is one stage)

`perf` is unavailable on this machine (`kernel.perf_event_paranoid = 4`) and
`yama/ptrace_scope = 1`, so `main_24` calls `prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY)` and gdb
attach is used as the sampler.

**Step 1 — sampling.** `main_24 spin v2 box_x_box cut 0`, 40 gdb backtraces spread over the run:

```
40 / 40 samples inside  session_cpp::BRep::face_meshes_q
leaf histogram: 25 NurbsCurve::is_valid, 3 find_span, 2 divide_by_count, 2 basis_functions, …
full path (every sample):
  v2_cut -> v2_boolean -> v2_outward_signs -> V2BuilderSolid::perform_areas
    -> v2_shell_is_hole -> v2_shell_signed_volume -> V2Topo::ensure_meshes
    -> BRep::face_meshes() -> NurbsSurfaceTrimmed::mesh() -> Delaunay2D::insert
```

i.e. **v2 computes a shell's signed volume by triangulating every face with the *display* mesher**
(`brep_v2_solid.cpp:487,512,531` → `ensure_meshes` → `b->face_meshes()`).

**Step 2 — call counting** (gdb breakpoints, `box x box` cut): **5** `BRep::face_meshes_q` calls
and **18** `NurbsSurfaceTrimmed::mesh_q` calls per boolean, grouped 6 / 6 / 0 / 0 / 6.
Per-call wall time from breakpoint timestamps: **0.65 – 1.46 s per `mesh_q`** — on faces of a
**box**.

**Step 3 — the A/B that explains it.** `main_24 resmesh 0` runs both kernels on the same pair and
then times `face_meshes()` on each *result* and counts every 2-D trim curve's control points:

| pair | kernel | faces | mesh time | triangles | trim CVs |
|---|---|---|---|---|---|
| box x box | current | 9 | **0.362 ms** | 18 | **84** |
| box x box | **v2** | 9 | **4146.0 ms** | 9 408 | **2 412** |
| box x box | pristine operand A | 6 | 0.029 ms | — | 48 |
| sphere x sphere | current | 2 | 9.5 ms | 880 | 262 |
| sphere x sphere | **v2** | 2 | **3768.5 ms** | 16 006 | **1 184** |
| torus x torus | current | 3 | 58.6 ms | 2 102 | 24 |
| torus x torus | **v2** | 2 | **3193.9 ms** | 29 600 | **796** |
| sphere x cylinder | current / v2 | 2 / 2 | 32.3 / 32.3 ms | 1 248 / 1 248 | 16 / 16 |
| box x sphere | current / v2 | 7 / 7 | 9.6 / 6.8 ms | 427 / 427 | 65 / 65 |
| cylinder x cylinder | current / v2 | 6 / 6 | 39.0 / 40.8 ms | 1 192 / 1 192 | 532 / 532 |
| box x cone | current / v2 | 3 / 3 | 17.9 / 18.1 ms | 350 / 350 | 26 / 26 |
| cone x cone | current / v2 | 5 / 5 | 39.0 / 40.5 ms | 1 013 / 1 013 | 63 / 63 |

The correlation is exact and it is the whole story:
**the three pairs where v2's imprint densifies the trim loops (box×box 84→2412, sphere×sphere
262→1184, torus×torus 24→796) are precisely the three pairs where v2 is 148–1212× slower and
where its RSS reaches 45–77 MB.** Where the trim CV counts are identical (sphere×cylinder,
box×sphere, cylinder×cylinder, box×cone, cone×cone), the mesh times are identical to three
digits — and those are v2's *cheap* pairs (16–217×, and 98 ms at best).

**Step 4 — the amplifier in shared v1 code.** `NurbsCurve::point_at` (`src/nurbscurve.cpp:2104`)
begins with `if (!is_valid())`, and `NurbsCurve::is_valid` (`:824`) scans **every** control value
(`for i in m_cv: isfinite`) plus the whole knot vector. So a single point evaluation is
**O(cv_count)** instead of O(order), and tessellating a curve at P points costs O(P·n) instead of
O(P·order). This is why a 30× denser trim loop costs ~10<sup>4</sup>× more to mesh, and it is why
25 of 40 stack samples land in `is_valid`. *(Read-only observation — `nurbscurve.cpp` is not mine
and I changed nothing.)*

**Consequences, in priority order, for whoever owns these files:**
1. `v2_shell_signed_volume` should not need a display triangulation at all; `brep_massprops`
   already computes an exact signed volume by the Green/divergence route with a bounded budget,
   and it is what `v2_verdict` uses. Replacing the mesh-based shell volume removes 100 % of the
   sampled time on the worst cells.
2. Failing that, the mesh should be built once per boolean, not 5 times, and at classification
   quality rather than display quality.
3. `NurbsCurve::point_at` hoisting its `is_valid()` out of the hot path is a one-line change with
   a measured 10<sup>4</sup> lever on trimmed-face meshing — but it is shared v1 code and needs
   its own owner and its own regression run.
4. v2's imprint producing 30× denser trim loops than the current kernel's is a *correctness-
   adjacent* fact worth a separate look: same face count, same result, 30× the data.

---

## 7. Gate statement

| gate | result |
|---|---|
| **no operation may hang** | **PASS.** 144/144 ladder cells, 20/20 liveness cells, 6/6 chairs cells, 24/24 scaling cells exited. Zero timeouts, zero `bad_alloc`, zero non-termination. |
| bound that THROWS is acceptable, infinite loop is not | No cell threw and no cell looped. |
| memory does not balloon | **PASS.** Worst 417 MB (130-face synthetic); worst realistic cell 77 MB. |
| v2 usable on speed | **FAIL as it stands.** 225× median slowdown vs the current kernel; 9–12× vs the current kernel and 2.4–13× outside the OCCT reference band on the only real model measured (chairs). Root cause is one stage and is quantified in §6. |

**Severity-one defects found: none** (no non-termination). **Severity-two: the v2 shell-orientation
stage's use of the display triangulator**, which alone accounts for 100 % of the sampled time on
v2's worst ladder cells and turns a 7.6 ms boolean into a 9.2 s one.

## 8. Reproduce

```bash
cd session_cpp
cmake -B build_perf -DCMAKE_BUILD_TYPE=Release
cmake --build build_perf --target main_16 main_17 -j 12     # builds session_core + session_v2
bash <scratchpad>/build24.sh                                # compiles+links main_24 by hand
./build_perf/main_24 info
./build_perf/main_24 ladder 3          # §2
./build_perf/main_24 resmesh 0         # §6 step 3
./build_perf/main_24 scale 66 v2 cut   # §5
./build_perf/main_24 scaledisj 128 cur cut
/usr/bin/time -v ./build_perf/main_24 one v2 torus_x_torus cut 0      # §4
timeout -k 5 600 ./build_perf/main_24 live coneRxcyl cur cut          # §3
./build_perf/main_24 chairs corpus/work/chairs v2 cut
```
Raw logs, gdb backtraces and `/usr/bin/time -v` files:
`/tmp/claude-1000/-home-petras-code-code-rust-session-session-cpp/4b6574b4-867c-482c-816a-9c3762a2bd3e/scratchpad/`
(`ladder1.log`, `ladder3.log`, `live/`, `mem/`, `scale/`, `scale2/`, `prof_box_x_box_v2_cut/`).
These are session-scratch paths and will not survive; every number in this document is also
printed by the commands above.
