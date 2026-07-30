# brep_massprops — integration notes for session A

New files, nothing else touched:

- `src/brep_massprops.h` / `src/brep_massprops.cpp` — volume, surface area, centroid for
  trimmed-NURBS B-Reps, with a hard work bound, an error estimate on every value, and a
  closure certificate.
- `main_12.cpp` — the test driver (analytic truth for every case; timing gate).
- `CMakeLists.txt:258` — the single shared token: `foreach(MAIN_ID … 12)`.
  `src/*.cpp` is already GLOBbed into `session_core`, so no other build change is needed.

Build/run:

```
cmake -S session_cpp -B session_cpp/build_e -DCMAKE_BUILD_TYPE=Release
cmake --build session_cpp/build_e --target main_12 -j 12
session_cpp/build_e/main_12                 # full suite
session_cpp/build_e/main_12 --dump a.step   # one file, per-face breakdown
session_cpp/build_e/main_12 --mem sphere    # one in-memory primitive
SESSION_MP_DBG=1 …                          # per-face/per-trim diagnostics on stderr
```

---

## 1. The API and how to swap the call sites

```cpp
#include "brep_massprops.h"

MassProps m = brep_massprops(brep);          // everything at once
double v = brep_volume(brep);                // convenience wrappers
double a = brep_area(brep);
Point   c = brep_centroid(brep);
```

`MassProps` carries what a caller has to gate on:

| field | meaning |
|---|---|
| `volume`, `area`, `centroid` | the values |
| `volume_error`, `area_error` | absolute error estimates (quadrature + closure defect) |
| `converged` | **false** ⇒ the work bound, not the tolerance, ended refinement |
| `closed`, `closure_residual` | geometric closure certificate — see §4 |
| `naked_edges` | topological closure (edges with ≠ 2 trims) |
| `shell_count`, `shell_volumes` | signed per-shell volumes; cavities are negative |
| `surface_evals`, `seconds` | measured cost |
| `faces[i]` | per-face `area`, `flux`, `traversal`, `outward`, `path`, `chain_gap`, `evals` |

**Swap procedure.** `BRep::volume()` returns a bare double and has no failure channel, so a
call site that only wants a number becomes:

```cpp
- double v = R.volume();
+ double v = brep_volume(R);                 // never hangs
```

but the call sites that *gate* on the number should take the struct so they can refuse a
value that is not trustworthy:

```cpp
MassProps m = brep_massprops(R);
if (!m.closed || m.closure_residual > 1e-9) { /* volume is origin-dependent — do not gate */ }
if (!m.converged)                            { /* value is only accurate to m.volume_error */ }
```

Call sites, by file (23 in the mains + 27 in `brep_test.cpp`):

- `main_9.cpp:55,56,71` — operand summary + per-op result. **These are the three lines that
  masked 127 boolean results**: `A.volume()` is evaluated inside the `printf` argument list
  *before* the summary prints, so a non-terminating `volume()` swallowed the whole run.
  Replacing them with `brep_volume(...)` removes the hang class entirely.
- `main_7.cpp` — 29 sites, mostly `v = r.volume()` after an op; direct substitution.
- `main_10.cpp:363,378,418,432,442,447` (session B) and `main_8.cpp:22-24,43` — same.
- `src/brep.cpp:2172` — the current `BRep::volume()`. Simplest integration: keep the method
  and make its body `return brep_volume(*this);`. That is a one-line change inside the file
  you own; `brep_massprops.h` includes only `brep.h`/`point.h`, so there is no cycle.
- `src/brep_test.cpp` — the minitest assertions can stay on `BRep::volume()` if it forwards.

Nothing in `brep_massprops` mutates the BRep and nothing calls back into `BRep::volume()`,
`face_outward_signs()`, `mesh()` or the CDT, so it is safe to call while `brep.cpp` is being
edited.

---

## 2. Why it always terminates

The old path built a 384×384×5×5 masked Gauss grid per curved face and ran point-in-polygon
against the sampled trim polygons at every one of the 3.7 M Gauss points — on a face whose
pcurves carry thousands of CVs that is ~10¹⁰ predicate evaluations per face, which is the
non-termination.

The new path never builds a grid. Two structural guarantees:

1. **Green reduction, not masking.** The 2D integral over the trimmed UV region D is reduced
   to a 1D integral along the trim boundary — the same reduction OCCT uses in
   `BRepGProp_VinertGK` / `BRepGProp_TFunction` / `BRepGProp_UFunction`
   (`/home/petras/code/code_cpp/OCCT/src/ModelingAlgorithms/TKTopAlgo/BRepGProp/`):

   ```
   ∫∫_D f(u,v) du dv = ∮_{∂D, CCW} H(u,v) dv,     H(u,v) = ∫_{u_ref}^{u} f(s,v) ds
   ```

   The value does not depend on `u_ref` (two choices differ by a closed-loop integral of an
   exact 1-form), so `u_ref` is set to the trim's `umin` — which makes every isoparametric
   trim drop out for free: a trim with `dv ≡ 0` contributes nothing, and a trim at `u = u_ref`
   has `H ≡ 0`. An untrimmed quadric face therefore costs **one** 1D integral, not a 2D sweep.
   There is no point-in-polygon test anywhere, and no staircase error: the trim curves *are*
   the domain of integration. Degenerate (pole) trims are ordinary curves in UV and need no
   special case.

2. **Bounded refinement.** Every 1D integral is globally adaptive Gauss–Kronrod 7/15 (QUADPACK
   `qk15` constants; K15 is exact for polynomials of degree ≤ 22 and the embedded G7 gives the
   error estimate free). The refinement loop is

   ```
   while (!done && panels < cap) { if (budget_left <= 0) …; if (converged()) …; split_worst(); }
   ```

   `cap` is a constant, one panel is added per iteration, every panel has `depth < max_depth`,
   and every integrand call decrements `budget_left`. The loop therefore executes at most
   `cap` times **whatever the geometry does**. There is no `while (err > tol)` in the file.
   When a bound rather than the tolerance stops it, the value is still returned with its error
   estimate and `converged = false`.

**Hard bound.** `MassPropsOptions::max_surface_evals` (default 60 000 000) caps
`NurbsSurface::evaluate` calls for the whole BRep; the per-face share is
`max(min_face_evals, max_surface_evals / faces)`. Measured throughput on this machine is
1.7 M evaluations/s, so the worst case is **≈ 35 s**, and it is a *bound*, not a behaviour:
the whole 40-solid OCCT corpus runs at 0.10–4.33 s per solid.

One non-obvious detail worth keeping: the **initial** panel list of every integral is always
evaluated in full, before any budget check. Skipping initial panels when the budget runs out
does not lose accuracy, it silently drops whole chunks of the integral — that produced a
systematic −3e-4 on a cylinder before it was fixed. Only refinement is budget-gated.

---

## 3. Accuracy — what makes it exact, and the one trap

- **Planar faces** take a closed-form path: vector area `A = ½∮ S × dS`, flux
  `= (P₀·N̂)(A·N̂)` since `S·N̂` is the constant plane offset, and the first moments Green-reduce
  in the plane frame to `∮ G_k db` with `G_k` a cubic polynomial. All 1D, all polynomial, so
  K15 is exact. When the trim's edge carries a valid 3D curve whose image matches the pcurve
  (endpoints + midpoint, quarter point for closed edges), the **3D edge curve is used instead
  of the pcurve** — that is what makes `occ_cylinder.step` exact instead of 9.4e-6 off, since
  the reader stores its cap pcurves as 48-segment polylines but the edge curves exactly.
- **Curved faces** take the Green path. Break points are the pcurve knots (uniformly merged
  down to `max_init_intervals` when a boolean-produced degree-1 pcurve has thousands of
  spans), and the inner integral splits at the surface's u-knots.
- **Orientation** never reads `BRepFace::reversed` or `BRepTrim::reversed` as ground truth.
  Per-face traversal sense is the **sign of the raw area integral** (the true area is positive
  by definition), trim order/direction inside a loop is recovered by chaining head-to-tail in
  UV, hole loops are re-signed against the dominant loop, relative face signs are propagated
  across every 2-trim edge using the shared 3D edge curve's own parametrisation, each shell is
  normalised to positive enclosed volume, and cavity shells (bbox nesting, odd depth) go
  negative.

**The trap, if you tune tolerances.** A component whose exact value is zero by symmetry — every
first moment of a centred solid — makes a purely relative convergence test compare roundoff
against roundoff and never converge. That alone burned the whole 60 M budget: the sphere took
4.8 M evaluations and 3.8 s. Adding per-component **absolute floors derived from the model's
own scale** (`rtol·span²` for area, `rtol·far·span²` for flux, `rtol·far²·span²` for the
moments) dropped the same sphere to 22 170 evaluations and 0.016 s with the value unchanged at
1e-14. Do not remove `face_scale_atol`.

Nested quadrature has a noise floor: the inner integral's roundoff (~1e-13 relative) is the
outer integrand's noise, so the outer tolerance is clamped at 1e-11 and the inner at 1e-13
internally regardless of `rel_tolerance`. Chasing tighter than that only spends budget.

---

## 4. The closure certificate — read this before gating on a volume

`∮ n dA = 0` for any closed shell, so the net outward vector area is an exact, cheap closure
test. The three components ride along in the same quadrature, and

```
closure_residual = |Σ_faces outward·(∫∫ Su×Sv)| / total area
```

is dimensionless: 0 for a closed shell, and when it is not small the divergence-theorem flux
is **origin-dependent** — the number changes if you translate the model. `brep_massprops` sets
`closed = false` above 1e-6 and inflates `volume_error` by `closure_residual · area · diag / 3`,
the actual uncertainty. `MassProps::max_chain_gap` reports the worst UV gap when chaining a
face's trims, normalised by the domain size; Green's theorem needs a *closed* boundary, so a
large value says the input's trim loops are not loops.

Measured, on solids built by this codebase: `closure_residual ≈ 1e-17`, `max_chain_gap = 0`,
and translating by (37, −11, 53) changes the volume by 2e-16 relative.

On the 40 OCCT boolean solids in `/home/petras/fc_inspect/REFERENCE/`: `closure_residual`
0.08–0.85 and `max_chain_gap` 0.48–0.96 for **all 40**. Translating `REFERENCE_y30_cut`
changes its volume from 203.2 to 505.8. Those volumes are not meaningful for any
implementation — the imported shells do not close — which is why `main_12` uses that corpus as
a *termination and timing* gate and reports clos/gap per solid rather than pretending to an
accuracy claim.

---

## 5. Measured results (real runs, this machine, Release)

`main_12` — **28 passed, 0 failed, 2 xfail (STEP-reader defects), worst case 4.28 s**
(stable across runs; 4.325 / 4.278 / 4.267 s on three). Every PASS asserts, simultaneously:
volume, area and centroid against analytic truth; translation invariance;
`closure_residual`; `converged`; and the time bound.

Degenerate input returns rather than crashing: an empty `BRep` gives V = A = 0 with 0 faces,
and a deliberately open shell (one face of a box) still gives the exact area 16.0000000000
while reporting `closure_residual = 1.0` and `closed = false`.

| case | volume | rel err | area rel err | time |
|---|---|---|---|---|
| mem/box | 64.0000000000 | 1.1e-16 | 0 | 0.001 s |
| mem/sphere | 65.4498469498 | 0 | 0 | 0.025 s |
| mem/cylinder | 42.4115008235 | 3.3e-16 | 2.0e-16 | 0.002 s |
| mem/cone | 16.7551608191 | 0 | 0 | 0.002 s |
| mem/torus | 25.2661872668 | 1.4e-16 | 0 | 0.029 s |
| step-rt/box … torus (our STEP round-trip) | — | ≤ 1.7e-15 | ≤ 9.0e-16 | ≤ 0.19 s |
| mem/block_with_hole (trimmed, hole loop) | 60.5960186675 | 2.4e-16 | 1.2e-16 | 0.002 s |
| occA/occ_box | 64.0000000000 | 1.1e-16 | 0 | 0.000 s |
| occA/occ_cylinder | 42.4115008235 | 1.7e-16 | 2.0e-16 | 0.002 s |
| occA/occ_torus | 37.8992809003 | 3.8e-12 | 5.7e-12 | 0.160 s |
| occA/occ_sphere_hole (napkin ring, trimmed) | 94.7815026807 | 4.5e-16 | 2.2e-14 | 0.095 s |
| occA/occ_cyl_halfcut (plane-cut cylinder) | 31.4159265359 | 1.1e-16 | 0 | 0.001 s |
| occN/occ_sphere (B-spline, no pcurves) | 65.4498469498 | 1.1e-13 | 7.2e-14 | 0.016 s |
| occN/occ_torus | 37.8992809002 | 3.6e-13 | 1.8e-13 | 0.020 s |
| occN/occ_sphere_hole | 94.7815026808 | 5.1e-13 | 1.6e-13 | 0.020 s |
| occN/occ_cylinder, occ_cone, occ_cyl_halfcut | — | ≤ 1.0e-3 | ≤ 9.4e-4 | ≤ 0.044 s |
| mem/box_with_cavity (2 shells, void) | 182.4896783617 | 1.6e-16 | 0 | 0.013 s |
| mem/two_disjoint_solids | 249.5103216383 | 0 | 0 | 0.013 s |

The `occN/*` residuals at 1e-3 are the reader's 48-segment polyline pcurves: an inscribed
48-gon under-measures a circle by `1 − (48/2π)·sin(2π/48) = 2.9e-3`, so that is the ceiling
the *data* allows, not the quadrature.

For reference, OCCT's own `Shape.Volume` on the same `toNurbs()` sphere reports
**65.4212** (4.4e-4 low); `brep_massprops` returns **65.449846949780** against the analytic
65.449846949787.

**Termination gate**, 40 OCCT boolean solids (21–68 faces, includes two 32-face solids), plus
`/home/petras/xpairs/`:

| | |
|---|---|
| solids completed | 40 / 40 |
| 32-face solids | 2, both complete |
| worst single solid | **4.28 s** (`REFERENCE_x20_fuse`, 68 faces) |
| total for all 40 | 76.4 s |
| bound asserted | 60 s/solid — 0 over |
| `xpairs/bal_000/chair0.stp` / `chair1.stp` | 2.18 s / 4.30 s |

Before: no return at 240 s, 600 s or the 900 s cap on the 32-face solid.

The three defect signatures from the original report are gone: **cylinder and cone no longer
report ⅓** (both exact), **the torus no longer reports 3×** (exact), and the sphere's 0 is not
a mass-properties result at all — see §6.

---

## 6. STEP-reader defects found (these are yours, not mass properties)

Root-caused while building the fixtures. All three are in the *reader*; no mass-properties
implementation can produce a right answer from this data.

1. **Bare `SPHERICAL_SURFACE` face imports empty.** `occ_sphere.step` (one
   `SPHERICAL_SURFACE`, one face) yields a `NurbsSurface` with `degree −1`, `0×0` CVs, empty
   domain, and the BRep has 0 loops / 0 trims / 0 edges. Same in
   `/home/petras/xpairs/_prims/occtsph.step`. This is the "sphere reports 0 with is_solid=0"
   in the original report — there is literally no geometry.
2. **`CONICAL_SURFACE` → NURBS spans 1.5 turns.** `occ_cone.step` imports as `deg 2×1`,
   `13×2` CVs, u-knots `0,0,1,1,…,6,6`, u-domain `[0,6]`. Sampling shows `u = 0` and `u = 4`
   are the same 3D point: **one turn is `[0,4]`, the patch covers 540°.** The seam trim then
   runs diagonally from `(6, apex)` to `(4, base)` instead of straight up at `u = 4`, so the
   imported face self-overlaps a wedge. Integrating that (correct) region gives
   19.5403488074 against 16.7551608191 — and the extra is exactly the wedge:
   `(L²/8)∫₄⁶ c(u)(u−4)² du = 0.1667 × the true lateral area`, which matches the measured
   32.7702 vs 28.0993 to 5 digits.
3. **Periodic seams collapse to one parametric value.** In the `toNurbs()` files a fully
   periodic face's two seam copies both land at `u = 0` (instead of 0 and 2π), so the loop
   encloses **zero** UV area — the cylinder's lateral face, the torus, the cone's lateral face.
   `brep_massprops` detects a degenerate trim region (sampled UV polygon area ≤ 1e-9 of the
   domain) and falls back to the whole parameter rectangle, which is the only defensible
   reading of a periodic face without real trims — that is why those cases now come out exact.
   Fixing the seam u-values in the reader would let the normal trimmed path handle them.
4. **`BREP_WITH_VOIDS` is not read.** `read_file_step_breps` returns 0 BReps for a box with a
   spherical void (7 `ADVANCED_FACE`, 1 `BREP_WITH_VOIDS`, 1 `ORIENTED_CLOSED_SHELL`). The
   cavity test therefore builds its two shells in memory (`merge_breps` in `main_12.cpp`).
5. **Boolean-result trim loops do not chain.** On all 40 `REFERENCE_*.step`, the trims of a
   face do not join head-to-tail in UV (`max_chain_gap` 0.48–0.96 of the domain diagonal) and
   the shells do not close (`closure_residual` 0.08–0.85). Concretely, `REFERENCE_x20_common`
   face 0 has three trims — `(2.3069, 3.4227)→(2.3069, 0)`, `(2.3949, 2.0878)→(2.3069,
   2.2805)`, `(2.3069, 1.8878)→(2.3949, 2.0877)` — where the first spans the *whole* v-domain
   at the patch boundary and cannot belong to the same lens-shaped region as the other two.
   OCCT reads the same file as V = 0.000480, A = 0.059016; we import a shell whose area is
   0.399. `max_chain_gap` and `closure_residual` are now reported on every result so this is
   visible instead of silent.

Fixtures used are all under `/home/petras/massprops_fx/` (snap confinement — nothing in
`/tmp`), regenerated by `gen.py` / `gen2.py` there with `MP_OUT=<dir> /snap/bin/freecad.cmd
<script>` (paths only via env vars: `freecadcmd` treats extra argv as files to open).
`ref_truth.txt` holds OCCT's own volume/area for the 40 reference solids.
