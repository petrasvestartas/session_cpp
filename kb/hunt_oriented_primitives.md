# Hunt: the masked oriented-primitive battery

**Verdict up front.** The stated hypothesis is **refuted by direct measurement**. The analytic
recognisers fire *identically* in rotated poses — the intersection curves in every failing cell
(except the torus one) are byte-identical exact rational conics whose lengths match closed form to
12 digits, at every orientation, including in a case that comes out **41 % wrong**. The pipeline is
rigid-motion equivariant to 1e-15. The failure is **relative-pose dependent, downstream of the
SSI**, it is a **cliff** (not a ramp), the governing predicate is **scale-invariant**, and it
depends on the **azimuth of the relative tilt** — i.e. on where the exact section curve falls
relative to the operands' own seam/pole parameterisation.

Everything below is measured. Binary: `build/main_7_oriented` (my own copy of `build/main_7`, taken
22:59) plus `/home/petras/hunt_oriented/sweep`, a standalone full-precision driver that links the
prebuilt `session_core` objects. **No file under `src/`, `main_*.cpp`, `validation/`, `corpus/` or
`serialization/` was modified.** (Session A rebuilt `build/main_7` at 23:37 mid-session; every
number here comes from the frozen 22:59 snapshot, so the whole report is internally consistent
against one source state. `validation/occt_cache.txt` mtime is unchanged — no oracle re-shelling
occurred.) Independent oracle: FreeCAD 1.1.1 (OCCT), primitives built
*natively* in FreeCAD from the same numeric placements — no STEP round trip, no dependence on our
kernel.

---

## 1. All 18 cells, reproduced and independently verified

`SESSION_NO_ROT` is checked at **main_7.cpp:685** (comment at :680) — `pr[1].back()=='R' ||
pr[2].back()=='R'` skips the whole oriented battery. Unmasked:

| pair | op | ours (12 dp) | OCCT cache | OCCT indep (FreeCAD) | rel | faces ours/occt | solid |
|---|---|---|---|---|---|---|---|
| cyl × cylR | cut | 20.966024594273 | 20.964056898232 | 20.9640568977396 | **9.39e-05** | 10 / 8 | 1 |
| cyl × cylR | common | 21.445474041042 | 21.447443924628 | 21.4474439245833 | **9.18e-05** | 7 / 7 | 1 |
| cyl × cylR | fuse | 59.851415142961 | 63.375557720735 | 63.375557720801 | **5.56e-02** | 10 / 8 | **0** |
| box × boxR | cut | 37.939310229206 | 37.939310229206 | 37.9393102292058 | 7.49e-16 | 9 / 9 | 1 ✅ |
| box × boxR | common | 26.060689770794 | 26.060689770794 | 26.0606897707942 | 3.95e-15 | 7 / 7 | 1 ✅ |
| box × boxR | fuse | 101.939310229206 | 101.939310229206 | 101.939310229206 | 9.76e-16 | 13 / 13 | 1 ✅ |
| boxR × sph | cut | 38.650883144024 | 38.647230233894 | 38.6472302461638 | **9.45e-05** | 11 / 11 | 1 |
| boxR × sph | common | 25.350312995746 | 25.352769766058 | 25.3527697491653 | **9.69e-05** | 7 / 7 | 1 |
| boxR × sph | fuse | 104.100727435617 | 104.097077183682 | 104.097077283311 | **3.51e-05** | 10 / 10 | 1 |
| box × torR | cut | 43.626360258283 | 44.565552438205 | 44.565403065163 | **2.11e-02** | 7 / 7 | 1 |
| box × torR | common | 20.373639740239 | 19.434447562061 | 19.4345997235752 | **4.83e-02** | 3 / 7 | 1 |
| box × torR | fuse | 68.892550116576 | 69.831739704673 | 69.8317355015926 | **1.34e-02** | 12 / 16 | 1 |
| **coneR × cyl** | cut | **NO RESULT** | 12.307413316136 | 12.3073989871567 | — | — / 3 | — |
| **coneR × cyl** | common | **THREW std::bad_alloc** | 4.447747524071 | 4.44774457132759 | — | — / 2 | — |
| **coneR × cyl** | fuse | **NO RESULT** | 54.718914131241 | 54.7193868517866 | — | — / 5 | — |
| **boxR × cyl** | cut | 52.639529326297 | 52.636295533589 | 52.6362955335918 | **6.14e-05** | 9 / 9 | 1 |
| **boxR × cyl** | common | 11.360805390094 | 11.363704466409 | 11.3637044664082 | **2.55e-04** | 5 / 5 | 1 |
| **boxR × cyl** | fuse | 95.051030149756 | 95.047796357050 | 95.0477963570541 | **3.40e-05** | 10 / 10 | 1 |

The two missing pairs:

* **boxR × cyl** — runs fine, all three ops FAIL the 1e-6 gate but keep exact face counts and
  `is_solid=1`. Worst cell of the whole battery on the relative scale: common **2.55e-04**.
* **coneR × cyl** — **does not complete**. `cut`: no output within 600 s (twice, 10-min cap).
  `fuse`: no output within 600 s. `common`: `std::bad_alloc` under `ulimit -v 4194304`
  (4 GiB). This cell is a hang/OOM, not a precision failure. OCCT does it in milliseconds
  (3 / 2 / 5 faces).

Oracle-quality caveat, measured: for the torus and cone cells the cached oracle and my independent
FreeCAD run **disagree with each other** — box × torR cut 44.565552438 (cache) vs 44.565403065
(FreeCAD), a 1.5e-4 absolute / 3.4e-6 relative spread; coneR × cyl fuse spreads 4.7e-4. So OCCT's
own answer on the torus cell is only good to ~1e-4 absolute. Our box × torR error is **0.94** —
four decades larger, so the verdict is safe, but do not quote box × torR reference digits past 1e-4.

---

## 2. THE CENTRAL EXPERIMENT

### 2a. Rigid-motion equivariance — rotate BOTH operands together

`SESSION_POSE_SWEEP=1`, 5 pairs × 10 angles (0, 1e-6, 1e-4, 1e-2, 0.1, 1, 5, 15, 30, 45°) × 3 ops,
generic rotation axis (0.4082, 0.8165, 0.4082). A rigid motion cannot change the answer, so any
deviation from angle 0 is pure world-frame pose dependence.

Full log: `/home/petras/hunt_oriented/pose_sweep.txt`. Result: **flat**.

| pair | max rel deviation over the whole sweep |
|---|---|
| box × box2 (planar × planar) | 8.8e-15 |
| cyl × cyl2 (curved × curved) | 8.3e-15 |
| sph × cyl (curved × curved) | 5.1e-14 |
| box × cyl (planar × curved) | 1.9e-09 (one blip at 0.01°, 1e-14 elsewhere) |
| box × tor (torus) | constant −2.41e-09 at every non-zero angle; two sporadic `common` blips (2.4e-2 at 1e-4°, 2.8e-8 at 0.01° and 1.0°) that are **not monotone in angle** |

**No cliff and no ramp. Face counts never change.** There is no world-frame orientation dependence
to find — the answer at 45° equals the answer at 0° to machine precision. That kills the framing
"rotated poses degrade the answer" and forces the question onto **relative** pose.

### 2b. Relative pose, with an EXACT closed-form oracle

Configuration: **A = sphere r = 2.5 at the origin; B = cylinder r = 1.5, h = 6, centred on the
origin**, rotated by θ about an axis **through the origin**. The cylinder's axis therefore always
passes through the sphere centre, and its caps (|z| = 3) never reach the sphere (2.5), so the
intersection is **congruent at every relative orientation**:

```
vol(common) = 4π/3 · (R³ − (R²−r²)^{3/2}) = 31.939525311496   (exact, all θ)
vol(cut)    = 4/3πR³ − that                = 33.510321638291   (exact, all θ)
```

OCCT agrees: rel ≤ **4.8e-08** across 4 rotation axes × 6 angles, always 3 faces
(`/home/petras/hunt_oriented/occt_invariant.py`). So this is an oracle-free instrument of
unlimited angular resolution. Ours, rotating **only** the cylinder (`./sweep sph cyl <ax> <ay> <az>
<deg> B`):

**axis (1,2,1)/√6** — `runs/sphcyl_invariant.txt`, `runs/sphcyl_bisect_s1.txt`

| deg | our common | rel err | solid |
|---|---|---|---|
| 0 → 0.30 | 31.9395252–31.9395253 | 3.7e-09 … 1.4e-09 | 1 |
| 0.34 | 31.939525291533 | 6.3e-10 | 1 |
| **0.36** | 31.939525305075 | **2.0e-10** | 1 ← last good |
| **0.37** | 31.937084930838 | **7.64e-05** | 1 ← **CLIFF** |
| 0.38 | 31.937019770526 | 7.84e-05 | 1 |
| 0.44 | 31.936629685050 | 9.06e-05 | 1 |
| 0.48 | 31.936370462722 | 9.87e-05 | 1 |
| 0.50 | 31.932878489872 | 2.08e-04 | 1 |
| 0.75 | 31.929574668126 | 3.12e-04 | 1 |
| 1.00 | 31.926284029301 | 4.15e-04 | 1 |
| 1.25 | 31.923006671500 | 5.17e-04 | 1 |
| 1.50 | 31.919742673286 | 6.20e-04 | 1 |
| **1.85** | 31.915195629025 | **7.62e-04** | 1 ← last bad |
| **1.90** | 31.939551255523 | **8.1e-07** | 1 ← **CLIFF back** |
| 2.0 / 3.0 | 31.9395528 / 31.9395682 | 8.6e-07 / 1.3e-06 | 1 |
| 5 / 10 | 31.9395252 / 31.9395250 | ~1e-08 | **0 (open shell)** |
| **20 / 30** | **18.849829 / 18.848938** | **4.1e-01** | **0** |
| 45 | 31.936817607615 | 8.5e-05 | **0** |
| 60 / 90 | 31.9395219 | 1.1e-07 | **0** |

**It is a CLIFF, twice.** Exact (1e-10) at 0.36°, 7.6e-05 wrong at 0.37°, and inside the band
[0.37°, 1.87°] the error is **linear in θ** (−1.30e-02 per degree in the 0.5–1.85 stretch),
snapping back to exact at 1.90°. That is the signature of a tolerance-gated *discrete branch*: the
mishandled feature is a sliver whose volume grows ∝ θ.

**Scale control** (`SWEEP_SCALE`, the whole configuration scaled uniformly):

| scale | 0.36° | 0.38° | 0.5° | 1.0° |
|---|---|---|---|---|
| ×0.1 | 0.031939528810 | 0.031937023461 | 0.031932878991 | 0.031926284029 |
| ×1 | 31.939525305075 | 31.937019770526 | 31.932878489872 | 31.926284029301 |
| ×10 | 31939.525305075 | 31937.019770526 | 31932.878489872 | 31926.284029301 |

The cliff sits at the **same angle** and the **same relative error to 9 digits** across three
decades of size. The gate is therefore **scale-invariant (angular / relative)**, not an absolute
length tolerance.

**The cliff angle depends on the tilt AZIMUTH, not just the magnitude:**

| rotation axis | tilt direction | behaviour |
|---|---|---|
| (0,1,0) | toward +x (the sphere's seam meridian) | **clean**: rel ≤ 4e-09 at 0.30, 0.32, 0.33, 0.335, 0.34, 0.35, 0.36, 0.40° |
| (1,2,1) | azimuth −26.6° | clean ≤ 0.36°, broken band [0.37°, 1.87°], clean again, open shell ≥ 5° |
| (1,0,0) | azimuth −90° | **18.849556 (rel 4.1e-01), solid = 0 already at 0.30°**, same at 0.5° and 1.0° |
| (1,1,0) | azimuth −45° | **chaotic at 0.01° resolution**: 0.44 → 18.8496 (bad), 0.46 → 31.9395254 (good), 0.47 → 31.9287 (bad), 0.48 → 18.8496, 0.50 → 18.8496, 0.52 → 18.8368 |

So the variable is not "how much is it rotated" but "where the (exact) section curve lands relative
to the operands' own seam/pole structure". Tilting *toward* the seam preserves a mirror symmetry
about the seam plane and is clean; every other azimuth is not.

---

## 3. The recogniser is NOT the culprit — measured at the SSI level

`sweep` prints, per surface pair, the section curve's **degree / CV count / rational flag /
length**. An exact analytic conic is `deg=2 cv=9 rat=1`; a marched section is a `deg=3 rat=0`
B-spline with dozens of CVs. This is a direct read-out of which branch fired.

**sph × cyl across the entire cliff** (`runs/sphcyl_ssi_signature.txt`), at
0, 0.30, 0.36, 0.37, 0.38, 0.40, 0.5, 1.0, 1.85, 1.90, 2.0, 5.0, **20.0°**:

```
SSI A0 x B0: n=2 [deg=2 cv=9 rat=1 len=4.712388980385] [deg=2 cv=9 rat=1 len=4.712388980385]
```

**Identical at every single angle**, including at 20° where the answer is 41 % wrong.
`4.712388980385 = 3π/2 = π·1.5` — the exact half-circumference of the r = 1.5 intersection circle,
to 12 digits. The analytic branch fires, and it is exact, in every pose.

Same story at the battery's shipped poses (`runs/ssi_signatures.txt`):

| cell | sections |
|---|---|
| cyl × cylR | all `deg=2 cv=9 rat=1`; len 8.943031803978 and 4.908441480166 = half the perimeters of the two exact Steinmetz ellipses (a = R/sin(α/2) = 3.919877, b = 1.5 → P/2 = 8.94303; a = R/cos(α/2) = 1.623587 → P/2 = 4.90844, Ramanujan, to the 5 digits computed by hand) |
| boxR × sph | 5 sections, all `deg=2 cv=9 rat=1` |
| boxR × cyl | 2 exact circles `deg=2 cv=9 rat=1` + 2 exact rulings `deg=1 cv=2 rat=0 len=6.6` |
| box × boxR | all `deg=1 cv=2 rat=0`, lengths exact |
| box × sph (passing baseline) | 6 × `deg=2 cv=9 rat=1 len=4.712388980385` — *the same signature as the failing boxR × sph* |

**The hypothesis is refuted.** Recognition is not orientation-dependent; the 1e-4 cluster is
generated entirely downstream of the intersection curves.

### The one genuine orientation-gated recogniser predicate (reported, but not the cause)

**`src/intersection.cpp:3126`**, in `ssi_plane_torus`:

```cpp
if (std::abs(std::abs(wn) - 1.0) > 1e-7) return false;  // non-perpendicular -> marcher
```

`wn` = plane normal · torus axis. Only planes **exactly perpendicular** to the torus axis are
analytic; the implied cliff is `1 − cos θ > 1e-7`, i.e. **θ > 0.0256°**. Real, and worth fixing —
but **measured not to be the differentiator here**: for *both* box × tor (upright, rel 3.0e-07) and
box × torR (45° tilt, rel 2.11e-02), every plane × torus section comes back marched
(`deg=3 rat=0`, cv = 72 / 75 / 18 / 48). No box face is ever perpendicular to the torus axis in
either configuration, so the analytic branch is never reached in either pose. The tilted cell fails
for a different reason (its topology also collapses: common 3 faces vs 7, fuse 12 vs 16).

### Complete inventory of analytic-coverage gates and their constants (read-only grep)

| file:line | predicate | constant |
|---|---|---|
| `intersection.cpp:3126` | `ssi_plane_torus`: plane ⟂ torus axis | `\|\|wn\|−1\| > 1e-7` → marcher |
| `intersection.cpp:2759` | `ssi_plane_cylinder`: oblique ellipse case | `\|wn\| < 1e-7` → hand off |
| `intersection.cpp:2783` | `ssi_plane_cylinder_lines`: axis-parallel rulings | `\|wn\| >= 1e-7` → not this case |
| `intersection.cpp:4011` | `ssi_cylinder_cylinder`: crossed cylinders need **equal radii** | `\|R1−R2\|/Rmax > 1e-6` → marcher |
| `intersection.cpp:4013` | …and **intersecting axes** (`lines_closest_point`) | `kTol = 1e-6`; skew axes → marcher |
| `intersection.cpp:3920, 3935, 3949, 3977, 4030, 4045, 4070, 4102, 4177` | coaxial / on-axis gate of every quadric×quadric handler (cyl×sph, cyl×cone, cone×sph, cyl×tor, cone×tor, sph×tor, tor×tor) | `kTol = 1e-6` |
| `intersection.cpp:4201` | recognition fit tolerance | `rtol = max(tolerance, 1e-7) * 1e4` → **1e-2** at the boolean's 1e-6 |

Net analytic coverage: plane × {plane, sphere, cylinder, cone, torus-⟂}, sphere × sphere, and
**coaxial-only** quadric × quadric. Two cylinders with unequal radii or skew axes always march.
That is a real coverage hole, orientation-independent in nature, and *not* what these cells hit.

---

## 4. Is the 1e-4 cluster a known sampling constant? **No.**

The marcher's constants are `intersection.cpp:1578` (`nu = (spans−1)*4` seed grid),
**`intersection.cpp:1666` `step = min(du,dv) * 0.25`** (RK2 marching step; `max_steps = nu*nv*32`;
adaptive ×0.25 / ×0.5 at tangent dots `0.95` / `0.985`), and `intersection.cpp:4903-4904`
(`h_init = min cell diagonal * 0.25`, `conv_tol = max(tol, h_init*1e-7)`).

**None of them is executed in the 1e-4 cells** — those cells' sections are exact rational conics
(§3). There is no sampling in that code path, so a fixed deflection constant cannot be the
explanation. The uniformity of 9.39e-05 / 9.18e-05 / 9.45e-05 / 9.69e-05 is not a shared constant;
it is what a *fixed absolute* volume defect (~2–4e-03) looks like when divided by results that all
happen to sit in the 20–40 range.

What the cluster actually is, measured:

* **cyl × cylR**: cut error **+1.96770e-03**, common error **−1.96988e-03** — antisymmetric to
  **−2.2e-06**. The partition identity holds; one *shared* boundary is displaced.
* **boxR × sph**: cut **+3.653e-03**, common **−2.457e-03** → partition identity **violated by
  +1.196e-03**. The cut and common results are bounded by *different* geometry — an inconsistent
  trim, i.e. a genuine construction error, not a symmetric flux bias.
* **sph × cyl invariant sweep**: `cut` stays right to **≤ 8.7e-07 at every angle in the band**
  (33.510292–33.510318 vs exact 33.510321638) while `common` is wrong by up to 7.6e-04 and later
  41 %. The defect is specific to the intersection *assembly*, and it is not symmetric between the
  two ops.
* Our `BRep::volume()` is **exact on untrimmed primitives** — box 64.000000000000, sphere
  65.449846949788 (exact 65.44984694978736), cylinder 42.411500823462 (exact 42.41150082346221),
  torus 25.266187266789 — and rigid-motion invariant to 1e-15 (§2a). It is **not** the 1e-4 source.
* Its residual error on trimmed curved faces is ~1e-7: `cyl × cyl2 common` = 17.999996669944
  against the exact Steinmetz 18 (rel 1.85e-07) — that is the "1.4e-7" of the shipped matrix, three
  decades below the cluster.

**Instrumentation warning for anyone repeating this:** our STEP writer is far lossier than any of
these effects. OCCT reads our exported `box_cut_sph` result as **9.51456162197675** where our own
(verified-correct, rel 5.6e-07 vs OCCT) internal value is **9.545719199340** — a **3.3e-03**
relative loss on export. Any "measure our result with OCCT via STEP" experiment is contaminated at
3e-03 and **cannot** resolve 1e-4. I did not use that route for any conclusion above.

---

## 5. What this means for the rotated chairs

Directly measured evidence that this is the same disease:

1. It is **not** world-frame orientation — rigid equivariance is exact to 1e-15. Anything that
   "rotating the chair" breaks must break through **relative** pose.
2. The dominant symptom in the invariant sweep is the **open shell** (`solid = 0`) at generic
   relative tilts of a *fully analytic, exactly-intersected* sphere/cylinder pair — the same
   symptom as the rotated chair configs (multiple shells, naked edges), reproduced here in the
   simplest possible geometry with an exact oracle.
3. The controlling variable is the **azimuth of the relative tilt**, i.e. where the exact section
   curve lands relative to the operands' own **seam/pole** parameterisation — not the angle
   magnitude. Chairs have cylindrical faces with seams; rotating a chair sweeps its section curves
   across those seams.

So the rotated residue is a strong candidate for a **seam/parameterisation-relative section
assembly** defect, **not** for analytic recogniser coverage. Places to look (read-only pointers,
nothing edited):

* `src/brep_section.cpp:804-904` — `split_seams`, the only seam-specific stage (chains split at
  periodic-seam jumps with corrector-pinned crossings). It is a no-op for the (non-periodic)
  chairs but is exactly the stage the sph × cyl invariant exercises.
* `src/brep.cpp:6091` — comment "the two sides were split at different events (A's seam vs B's
  seam, ~1e-2 apart)" and `src/brep.cpp:4029` — `whole_seg` alias index tolerance `1e-2`. These are
  1e-2-scale seam tolerances in the assembly, the right order of magnitude for a 0.37° cliff.

**Recommended permanent gate:** add sphere ∩ cylinder-through-centre as a regression test. Its
answer is `4π/3·(R³−(R²−r²)^{3/2})` at *every* relative orientation, so it can be swept at arbitrary
angular resolution with **zero oracle cost**, and it currently fails at 0.3° on one axis and 41 % at
20° on another.

**Recommended unmasking:** `SESSION_NO_ROT` (main_7.cpp:685) hides six pairs. Five of them run and
are merely wrong; one (**coneR × cyl**) hangs > 600 s on cut and fuse and OOMs on common. That last
one is a genuine liveness bug and should be tracked separately from the accuracy ones.

---

## Artefacts

All under `/home/petras/hunt_oriented/`:

| file | contents |
|---|---|
| `pose_sweep.txt` | rigid-motion equivariance sweep, 5 pairs × 10 angles × 3 ops |
| `relpose_sweep.txt` | partition-identity vs relative angle, 6 pairs × 12 angles |
| `runs/sphcyl_invariant.txt` | exact-oracle sweep, axis (1,2,1), 23 angles |
| `runs/sphcyl_bisect_s1.txt` | both cliff edges bisected |
| `runs/sphcyl_ssi_signature.txt` | SSI degree/cv/rational/length across the cliff |
| `runs/sphcyl_partition.txt` | cut + common across the cliff |
| `runs/fullprec_deg0.txt` | 12-dp our-volumes for the whole battery + baselines |
| `runs/occt_matrix_full.txt` | independent FreeCAD/OCCT recomputation, all 21 pairs × 3 ops |
| `occt_matrix.py`, `occt_invariant.py`, `measure_steps.py` | the FreeCAD oracles |
| `mx_*.txt` | raw unmasked matrix runs per pair |
