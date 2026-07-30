# Gap 1 — THE INTERSECTION LAYER: what OCCT computes for surface/surface sections that we do not

Scope: everything between "two surfaces" and "the set of 3D section curves handed to the splitter".
Everything below is either (a) a record from an OCCT trace, (b) a number I measured today, or
(c) a `file:line` citation, marked as source-derived. Claims with neither are labelled
**no measured impact yet**.

## 0. Provenance of every number in this file

| instrument | what | when |
|---|---|---|
| `validation/occt_trace/build/occt_trace` | OCCT 8.0.0.rc2, existing 38-case corpus in `traces/` **plus 6 new cases I ran today** (`coneR_cyl_cut`, `mx_box_torR`, `mx_box_tor`, `mx_cyl_cylR`, `mx_boxR_sph`, `mx_boxR_cyl`) built to reproduce the exact `main_7.cpp placements()` geometry | today |
| `session_cpp/build/main_7` | our kernel, binary dated **2026-07-26 08:21**. `src/intersection.cpp` was edited by another session at 16:16/16:34 today, so the binary predates those edits. Every "ours" number below carries that caveat. | today |
| `src/intersection.cpp` line numbers | as of the 16:34 snapshot; the file is under concurrent edit, so function names are the durable anchor | today |

New trace commands (reproducible verbatim):

```
./build/occt_trace --op cut --a "cone,r1=2,r2=0,h=4,rotx=-90,ty=-2.8" --b "cylinder,r=1.5,h=6,center" --name coneR_cyl_cut  --out …
./build/occt_trace --op cut --a "box,dx=4,dy=4,dz=4,center"           --b "torus,r1=2,r2=0.8,rotx=45"  --name mx_box_torR   --out …
./build/occt_trace --op cut --a "cylinder,r=1.5,h=6,center"           --b "cylinder,r=1.5,h=6,roty=45,tx=-1,tz=-1.8" --name mx_cyl_cylR --out …
./build/occt_trace --op cut --a "box,dx=4,dy=4,dz=4,center,rotz=30,tx=2,ty=1" --b "sphere,r=2.5"       --name mx_boxR_sph  --out …
./build/occt_trace --op cut --a "box,dx=4,dy=4,dz=4,center,rotz=30,tx=2,ty=1" --b "cylinder,r=1.5,h=6,center" --name mx_boxR_cyl --out …
```

**Measurement caveat that must be read before any length comparison.** Our `exact_circle` /
`exact_ellipse` (`intersection.cpp:2297`, `:2316`) build a 9-CV rational conic and then
`set_domain(0,1)`; `NurbsCurve::length()` on the result reports **exactly half** the true
perimeter. Proof by three independent instances: box×sph reports `len=4.7124` for a circle of
radius 1.5 (true circumference 9.42478); cyl×cylR reports `len=4.9084` and `8.9430` for the two
Steinmetz ellipses whose Ramanujan perimeters are 9.8169 and 17.887; `kb/hunt_oriented_primitives.md`
§3 independently calls 4.712388980385 "the exact **half**-circumference". Those cells are
volume-exact (box×sph 8.52e-07, sph×cyl 3.73e-09), so this is a `length()`/domain reporting
artifact, **not** a geometry defect — but it means **`len=` from `SESSION_PAIR_SSI` is comparable to
OCCT's `SEC len=` only for MARCHED curves** (which are cubic fits and do report full length: our
box×torR `5.8288` == OCCT's `5.82880714`).

---

# 1. WHAT OCCT DOES

## 1.1 Routing — which pairs are even offered to the closed-form solver

Source, `IntPatch_Intersection.cxx:1264-1292`: `ts = 1` unconditionally for **Plane, Cylinder,
Sphere, Cone**; `ts = bGeomGeom` for **Torus**. `ts1 == ts2 == 1 && isGeomInt` → `GeomGeomPerfom`
→ `IntPatch_ImpImpIntersection` (closed form); otherwise `ParamParamPerfom` (polyhedral seeding +
walking). Consequences:

* **All 10 quadric pairs that contain no torus go to closed form at EVERY relative pose.**
  Dispatch at `IntPatch_ImpImpIntersection.cxx:2570-2757`
  (`IntPP/IntPCy/IntPSp/IntPCo/IntCyCy/IntCyCo/IntCySp/IntCoCo/IntCoSp/IntSpSp`).
  Three escapes: `TreatAsBiParametric` for degenerate cones (`|semiangle| < 0.02 || > 1.55`,
  `IntPatch_Intersection.cxx:1117-1141`); `isGeomInt == false` from the plane×cylinder needle test
  (`IntTools_FaceFace.cxx:249-320`); and ImpImp failing (`IntPatch_Intersection.cxx:1790-1797`),
  which falls through to the walker.
* **The 5 torus pairs reach closed form only for specific axis relations**
  (`IntPatch_Intersection.cxx:1163-1232`): plane ⟂ torus axis, **or** plane parallel to the axis
  *and passing through the axis location within `Precision::Confusion()`*; sphere centre on the
  torus axis; cyl/cone/torus axis parallel to **and coincident with** the torus axis. Everything
  else marches. Even inside ImpImp, `TreatResultTorus` (`:9648-9710`) accepts only `Empty` or
  `Circle`; anything else returns false → walker.

Confirmed by measurement, not inferred: `mx_box_tor` (upright torus, box side faces parallel to
the axis but 2 units off it) and `mx_box_torR` (45°-tilted torus) both produce **10 section curves,
every one `type=BSpline tol=1e-06`** — the loosest tolerance anywhere in the 44-case corpus. OCCT
marches plane×torus in both poses.

## 1.2 What the closed-form solver actually delivers (measured curve types and tolerances)

| pair, pose | OCCT `SEC type` | `SEC tol` | source case |
|---|---|---|---|
| plane × sphere, oblique (rotated box) | `Circle` | 1e-07 … 1.433e-07 | `mx_boxR_sph` |
| plane × cylinder, oblique | `Line` + `Circle` | 1e-07 | `mx_boxR_cyl` |
| plane × cone, oblique 30° | `Ellipse`, `Line` | 1e-07 … 1.779e-07 | `box_cone_p2_cut` |
| cyl × cyl, **equal** radii, 45° crossing axes | `Ellipse` ×8 arcs | 1e-07 | `mx_cyl_cylR` |
| cyl × cyl, **unequal** radii, ⟂ crossing axes | `BSpline` ×2 | 1e-07 | `cyl_cyl_cut` |
| **cone × cylinder, ⟂ intersecting axes** | `BSpline` ×1, closed | **4.00816023e-07** | `coneR_cyl_cut` (new) |
| cone × cone, ⟂ and oblique | `BSpline` ×6 / ×7 | 1e-07 … 4.705e-06 | `cone_cone_p1/p2` |
| plane × torus, any non-special pose | `BSpline` ×10 | **1e-06** | `mx_box_tor`, `mx_box_torR` |
| sphere × cylinder, axis through centre | `Circle` | 1.0099e-07 … 1.702e-05 | `sph_cyl_roty*` |

Two things follow that matter more than the analytic/marched label:

1. **OCCT carries a per-curve tolerance and propagates it verbatim into the section edge.**
   Findings Q5.2: `RESEDGE tol` equals the DS `etol` of its pave block, which equals `SEC tol`
   (`sph_cyl_roty45_cut`: 1.29109247e-07 / 1.14703676e-07 / 1.1236843e-07 on both sides). The
   number is *measured from the curve*, not assigned.
2. **A `BSpline` result does not prove marching.** ALines (`IntAna_IntQuadQuad`) are approximated to
   BSplines too. The trace cannot distinguish them; I do not claim it can. What the trace *does*
   prove is the **delivered accuracy**, and that is the only thing our splitter consumes.

## 1.3 OCCT restricts the section to the face domain and splits it there — measured

`mx_boxR_sph`, face pair `f1=2` (one rotated box side, a plane, u,v ∈ [-2,2]) × `f2=36` (sphere):

```
SEC c=0 type=Circle t0=0.723268559 t1=0.93309883  len=0.522311017 p0=-0.732050808,1.73205081,1.64745307 …
SEC c=1 type=Circle t0=2.20849382  t1=2.60078056  len=0.976482954 …
SEC c=2 type=Circle t0=3.68240475  t1=4.07469148  len=0.976482954 …
SEC c=3 type=Circle t0=5.35008648  t1=5.55991675  len=0.522311017 …
```

Four arcs of **one** circle, with gaps in `t`: the circle leaves and re-enters the square face four
times, and OCCT emits only the in-face portions. The endpoints are exact algebraic points on the
rotated box edge (`-0.732050808 = 1 − √3`, `1.73205081 = √3`). This is `PutPointsOnLine` /
`ProcessSegments` / `ProcessRLine` (`IntPatch_ImpImpIntersection.cxx:2910-2926`) intersecting the
analytic line with each face's *restriction arcs* before anything downstream runs.

`mx_cyl_cylR`, lateral × lateral, is the same story plus seams: the small Steinmetz ellipse comes
back as **4 quarter-arcs** (t = 0, π/2, π, 3π/2 — the four points where it crosses one or the other
cylinder's seam meridian), and the large ellipse as **4 arcs with a gap** at t ∈ (1.972, 4.311),
the part that lies outside the finite operands. 8 curves on one face pair.

## 1.4 Seam and pole splitting is done inside `PerformFF`, not later — measured

Findings Q2 (established, re-confirmed today). `mx_box_torR`, torus face `f2=36`, `SEC2D`:

```
f1=2  c=0 face=36 v: 0 → π/2 → 0            c=1 face=36 v: 2π → 3π/2 → 2π
f1=12 c=0 u∈[0,0.775]      v∈[0,π/2]        c=1 u∈[5.508,2π] v∈[0,π/2]
      c=2 u∈[5.508,2π]     v∈[4.712,2π]     c=3 u∈[0,0.775]  v∈[4.712,2π]
```

The −X box face's section loop crosses the torus **v**-seam twice → **2** curves. The +X face's
loop crosses **both** the u- and the v-seam → **4** curves. The ±Y/±Z faces' loops stay strictly
inside the domain → **1** curve each. Total 10. The rule is mechanical: one `BOPDS_Curve` per
seam-delimited arc, decided on the 2D image, at FF time (`SEC tag=afterFF` count == `SEC tag=final`
count in every one of the 44 cases).

The same rule governs the sphere: at tilt 0–23° the section circle *encircles the pole*, its sphere
pcurve spans exactly one full u-period, that is legal, and OCCT emits **2** curves; from
23.578178478201835° (= asin(1/2.5), the exact pole tangency) upward the circle no longer encircles
the pole, crosses the seam twice, and OCCT emits **3** curves (`sph_cyl_roty23_*` → `roty24_*`).
**This 2→3 transition is the single most load-bearing fact in this document** (§4.1).

## 1.5 The fallback path — what OCCT's discovery stage guarantees

`IntPatch_PrmPrmIntersection::Perform`, `IntPatch_PrmPrmIntersection.cxx:2500-2620`:

1. Sample both surfaces: `Adaptor3d_TopolTool::ComputeSamplePoints` gives **2×2 for a plane,
   15×15 for cylinder/cone/sphere/torus/revolution/extrusion, NbKnots×Degree (≥4) for a BSpline,
   capped at 50** (`Adaptor3d_TopolTool.cxx`, `ComputeSamplePoints`).
2. `IntPolyh_Intersection` triangulates both grids and computes the **full triangle–triangle
   intersection**, returning `NbSectionLines()` — *ordered polylines* of start points — and
   `NbTangentZones()`.
3. The section lines are **sorted by point count, descending** (`:2582-2600`) so the largest branch
   is walked first.
4. For each line, `TabPtDep[]` marks which of *that line's* points a walk has consumed; the walker
   is **restarted from the next unconsumed point** until the line is covered. Every polyhedral
   branch is therefore guaranteed at least one walk, and a branch that a walk only partly covers is
   re-seeded.
5. `SeuildPointLigne = 15·Increment²` is the same-point threshold; tangent zones become separate
   solutions rather than terminating anything.

The guarantees are: **branch coverage keyed to a topological object (a polyhedral section line),
not to a distance threshold**; **re-seeding inside a branch**; and **a typed tangent-zone output**.

## 1.6 Tangency — measured, and it is not what the name suggests

Findings Q1, re-read against the traces:

* `BOPDS_InterfFF::TangentFaces()` is **0** at the exact pole tangency. Across the whole corpus it
  is set only for **fully coincident** face pairs (`box_box_touch_fuse`: all 5 `tangent=1` records
  have `ncurves=0`). It is a same-domain flag, not a geometric tangency test.
* The tangency is absorbed by **inflating the two existing pole vertices in place**:
  DS vertex 5 goes 1e-07 → 2.14903822e-06, vertex 7 goes 1e-07 → **1.70243165e-05**, both keeping
  `sd = -1` (widened, not replaced). The inflation is exactly `curve tolerance + 1.0e-12`
  (measured twice).
* It is the **only** case of 38 that runs the re-intersection pass:
  `Init VV VE EE VF EF [VV VE VF] FF`.
* OCCT's own answer is **worse just below tangency than at it**: at 23° common = 15.0649961
  (error +3.2e-3) versus 15.0617954 (exact) at 23.578°.

---

# 2. WHAT WE DO

## 2.1 The analytic coverage matrix, all 15 quadric pairs

`analytic_ssi`, `intersection.cpp:4670-4741`. Kinds from `recognize_surface_impl` (`:2675`), fitted
from an 8×8 sample grid at `rtol = max(tolerance, 1e-7) * 1e4` (= 1e-2 at the boolean's 1e-6).

| # | pair | OCCT closed form | ours | our gate (`intersection.cpp`) |
|---|---|---|---|---|
| 1 | plane × plane | any pose | any pose (clipped to both UV rects) | `ssi_plane_plane:3143`; `|n1×n2| < 1e-9` → marcher |
| 2 | plane × sphere | any pose | **any pose** | `ssi_plane_sphere:2743` |
| 3 | plane × cylinder | any pose | **any pose** (ellipse + rulings) | `ssi_plane_cylinder:2755`, `ssi_plane_cylinder_lines:2778` |
| 4 | plane × cone | any pose (2/4 rays through apex) | **any pose** (circle/ellipse/parabola/hyperbola/rays) | `ssi_plane_cone:3040` |
| 5 | plane × torus | plane ⟂ axis, **or** plane containing the axis | **plane ⟂ axis only** | `ssi_plane_torus:3122`, `abs(abs(wn)-1) > 1e-7 → false` |
| 6 | sphere × sphere | any pose | **any pose**; nothing emitted inside a `(r1+r2)·1e-9` tangency band | inline `:4695-4715` |
| 7 | sphere × cylinder | any pose (ALine when off-axis, `IntCySp:8266`) | **centre ON the cylinder axis only** | `ssi_cylinder_sphere:4390`, `point_axis_dist > 1e-6 → false` |
| 8 | sphere × cone | any pose (ALine, `IntCoSp:9351`) | **centre on the cone axis only** | `ssi_cone_sphere:4419` |
| 9 | sphere × torus | centre on torus axis | centre on torus axis + ring torus | `ssi_sphere_torus:4540` |
| 10 | cyl × cyl | any pose | parallel (any radii) **or** crossing with `|R1−R2|/Rmax ≤ 1e-6` **and** axes meeting within 1e-6 | `ssi_cylinder_cylinder:4446` |
| 11 | cyl × cone | any pose (ALine, `IntCyCo`) | **coaxial only** | `ssi_cylinder_cone:4405`, `axes_coaxial(…,1e-6)` |
| 12 | cyl × torus | coaxial | coaxial + ring | `ssi_cylinder_torus:4500` |
| 13 | **cone × cone** | any pose (`IntCoCo`, incl. common-generatrix and 2/4-ray apex splits) | **NO DISPATCHER ARM AT ALL** → `else { return res; }` at `:4741` | — |
| 14 | cone × torus | coaxial | coaxial + ring | `ssi_cone_torus:4515` |
| 15 | torus × torus | coaxial, or `Same` | coaxial + ring; else parallel-axis **equal-R-and-r, coplanar-centres** spiric with three topology gates | `ssi_torus_torus:4647`, `ssi_torus_torus_spiric:4572` |

Score: OCCT attempts closed form on **10/15 pairs at any pose** and on 5/15 (the torus family) only
in special poses. We attempt it at any pose on **6/15** (rows 1,2,3,4,6, plus cyl×cyl-parallel), in
degenerate poses only on 8/15, and **not at all on cone×cone**.

## 2.2 The marcher (everything the dispatcher declines)

`Intersection::surface_surface`, `intersection.cpp:5242-5995`.

* **Seeds** (`:5460-5486`): cells = `(spans−1)·4` per direction (`:5338`), AABBs inflated by
  `2·sag + tolerance`; every *overlapping AABB pair* gives ONE minimum-norm Gauss–Newton start from
  the two **cell centres**; deduped when the corrected 3D points are within
  `seed_tol_3d = max(min-cell-diagonal(A), min-cell-diagonal(B))` (`:5460`); hard cap
  `pair_budget = 20000` (`:5461`) which `break`s the loop silently.
* **March** (`:5512-5638`): predictor–corrector, `h_init = min cell diagonal · 0.25` (`:5399`),
  `conv_tol = max(tolerance, h_init·1e-7)`, step halving on corrector failure or a direction dot
  below 0.985, `max_steps = (a_nu·a_nv + b_nu·b_nv)·32`.
* **Tangency = stop** (`:5505`, `:5533`): `tangent_3d` returns false when
  `|n_a × n_b| < 1e-4·|n_a|·|n_b|` (normals within ~1e-4 rad); the trace then reuses the previous
  direction at most 3 times and otherwise terminates with reason `"tangency"`.
* **Trace dedup** (`:5700`): `dup_tol = h_init·2`, with an in-code note that the previous `h_init·6`
  merged 3 of the 4 Steinmetz arcs into one.
* **Accuracy** (`:5871`, `:5986`): points are refined until the chord deviation is below
  `refine_tol = max(tolerance·100, 5e-6)` = **1e-4** at the boolean's 1e-6, then fitted to
  `max(tolerance·10, 1e-7)` = **1e-5**. Pcurves are fitted in *parameter* space to
  `min(du,dv)·1e-4`.
* **Point sections are deleted**: `drop_point_sections:5235` removes any triple whose 3D length is
  below `max(tolerance·10, 1e-9)`.

## 2.3 What the analytic path hands downstream

`analytic_ssi:4746-4790`. For each exact 3D conic it tries `analytic_pcurve` (exact and rational
only for a PLANE — affine inversion preserving weights, `:3222-3247`; degree-1 exact `v = const`
lines for a *full-wrap, axis-perpendicular* circle on cylinder/sphere/cone), then
`analytic_torus_pullback` / `analytic_sphere_pullback` / `analytic_cone_pullback`, then
`Closest::surface_curve`. Two properties:

```cpp
// intersection.cpp:4752-4776  (all six of these lines have the same shape)
if (!pa.is_valid() && ra.kind == K::SPHERE) { auto v = analytic_sphere_pullback(a, ra, cc3); if (!v.empty()) pa = v[0]; }
```

* **`pa = v[0]`.** The pullbacks *seam-split* — that is their documented job
  (`:3955-3960` "…and seam-splits on the u-seam") — and we keep **only the first segment** and
  discard the rest, while the 3D curve keeps its full extent. The triple `(c3, pa, pb)` is then
  internally inconsistent.
* `emit_pullback_curve:3693-3727` ends in `return NurbsCurve::create(false, 1, pts);` — a **degree-1
  polyline in UV** — unless `SESSION_PB_FIT` / `SESSION_PB_SMOOTH` are set, and both default OFF.
  So the *exact-conic* path produces a worse pcurve than the marched path (which fits cubics).

## 2.4 v2 changes what matters

`v2/brep_v2_section.cpp:1155-1180`: `section_for_pair` calls `Intersection::surface_surface` and
consumes **only `std::get<0>(tr)`** — the 3D curve — then rebuilds both UV trails itself
(`build_trail`), paves at seam crossings (`V2PaveOrigin::SeamCrossing`,
`v2/brep_v2_section.h:75`) and *measures* the curve tolerance (`dev1`/`dev2`,
`brep_v2_section.h:38-40`). **v2 is structurally immune to §2.3's two defects.** What v2 still
inherits from this layer is exactly two things: **which 3D curves exist**, and **how accurate they
are**.

---

# 3. THE GAP — concrete implementable deltas

**G1. The analytic path silently truncates a seam-split section to its first segment.**
Delta: at `intersection.cpp:4752-4776`, when a pullback returns `n > 1` segments, emit `n` triples
(splitting `c3` at the same parameters), not one. OCCT's equivalent is "one `BOPDS_Curve` per
seam-delimited arc, decided at FF time" (§1.4). Alternative, cheaper, and the v2-compatible route:
keep one triple but carry the *full* multi-branch pcurve and let the consumer pave at the seam —
which is what v2 already does, so under v2 this reduces to "never drop `v[1..]`".

**G2. Marched 3D curves are built to a 1e-4 chord budget / 1e-5 fit tolerance; OCCT delivers
1e-07 (79 of 142 corpus curves), 1.5e-07 (29), 1e-06 worst-case for plane×torus.**
Delta: drive `refine_tol` and the `fit_track` target from the *boolean* tolerance, not
`tolerance·100` / `tolerance·10`; and attach the achieved deviation to the returned curve so the
consumer can use OCCT's rule (edge tol = curve tol; vertex tol = curve tol + 1e-12, findings Q5.3)
instead of a global constant. `intersection.cpp:5871`, `:5986`.

**G3. No domain-restriction step. Our analytic sections are whole unclipped conics.**
Delta: intersect each analytic conic with both faces' boundary arcs in closed form and emit only the
in-domain sub-arcs, with vertices at the exact crossings — OCCT's `PutPointsOnLine` /
`ProcessSegments` (`IntPatch_ImpImpIntersection.cxx:2910-2926`). Measured contrast in §1.3
(OCCT 4 exact arcs vs our 1 full circle on `boxR × sph`).

**G4. `cone × cone` has no dispatcher arm.** Delta: add the `IntCoCo` case table — coaxial circles,
the common-generatrix branch, and the 2/4 rays through the apex when the apexes coincide
(`IntAna_QuadQuadGeo.cxx:1433-1884`; audit E6 for the ray splitting).

**G5. `plane × torus` closed form only for plane ⟂ axis.** Delta: add OCCT's second branch — plane
*containing* the torus axis gives two exact circles of radius `aRMin` (= the minor radius), normal =
the plane normal, centred at `torusCentre ± aRMaj·(torusAxis × planeNormal)`
(`IntAna_QuadQuadGeo.cxx:2230-2247`, verified by reading; gate at
`IntPatch_Intersection.cxx:1185-1191`). Guard: the branch demands
`Pln.Distance(torusCentre) <= myEPSILON_DISTANCE (1e-14)` (`:2232-2236`), an exact-construction
requirement, so snap the plane onto the axis at recognition time.

**G6. Off-axis `sphere×cylinder`, `sphere×cone`, `cylinder×cone`, general `cylinder×cylinder`:
OCCT solves these as exact ALines, we march.** Delta: port `IntAna_IntQuadQuad`. See §4.4 before
budgeting this — the measured payoff is currently zero.

**G7. Seed discovery is distance-thresholded, not branch-keyed.** Delta: replace the
overlapping-AABB-cell-centre seeding with a polyhedral triangle–triangle pass that returns *ordered
section lines*, walk the largest first, and re-seed from each line's unconsumed points
(`IntPatch_PrmPrmIntersection.cxx:2532-2620`). Concretely this removes three unbounded risks:
`seed_tol_3d` merging two branches that pass within one cell of each other (`:5460`), `dup_tol`
merging two traces that stay within `h_init·2` (`:5700`, already documented to have mis-merged
Steinmetz arcs at `h_init·6`), and the silent `pair_budget = 20000` cut-off (`:5461`).

**G8. Tangency is a stop condition and a delete; OCCT has neither.** Delta: (i) `tangent_3d`
should not terminate a trace at `|n_a×n_b| < 1e-4` — OCCT walks tangent zones and reports them
typed (`NbTangentZones()`); (ii) a point-tangency section must not be silently deleted by
`drop_point_sections:5235` — OCCT keeps the contact and pays for it by *inflating the existing
vertex tolerance to the curve tolerance + 1e-12* and re-running VV/VE/VF; (iii) our `sph×sph`
`(r1+r2)·1e-9` dead band emits nothing at exact tangency where OCCT emits a `Touch` line with an
inside/outside `Situation` (audit E2 — do **not** model that as "Undecided").

---

# 4. MEASURED IMPACT — ranked

Our matrix state as I measured it today with `build/main_7` @08:21 (`SESSION_PAIR_SSI=1`, full
63-cell run): **45/45 non-oriented cells OK; every failure is in the 18-cell oriented battery.**
That does not match the brief's "34 OK / 29 FAIL", which must come from a different build state; I
report what I measured, and the failing cells are the same ones the brief names.

| cell | our vol | OCCT vol | rel | our/OCCT faces | solid |
|---|---|---|---|---|---|
| cyl × cylR cut | 20.9660 | 20.9641 | 9.39e-05 | 10 / 8 | 1 |
| cyl × cylR common | 21.4455 | 21.4474 | 9.18e-05 | 7 / 7 | 1 |
| cyl × cylR fuse | 59.8514 | 63.3756 | **5.56e-02** | 10 / 8 | **0** |
| boxR × sph cut/common/fuse | — | — | 9.45e-05 / 9.69e-05 / 3.51e-05 | 11/11, 7/7, 10/10 | 1 |
| box × torR cut | 43.6264 | 44.5656 | **2.11e-02** | 7 / 7 | 1 |
| box × torR common | 20.3736 | 19.4344 | **4.83e-02** | **3 / 7** | 1 |
| box × torR fuse | 68.8926 | 69.8317 | **1.34e-02** | **12 / 16** | 1 |
| boxR × cyl cut/common/fuse | — | — | 6.14e-05 / 2.55e-04 / 3.40e-05 | exact | 1 |
| box × boxR (3 cells) | — | — | ≤ 9.8e-16 | exact | 1 (OK) |

## 4.1 G1 is the top gap — the sphere/cylinder pole-crossing instrument

`sphere r=2.5 ∩ cylinder r=1 h=8, cylinder axis through the sphere centre, tilted about Y`. The
answer is **tilt-independent by symmetry**: `common = 4π/3(R³ − (R²−r²)^{3/2}) = 15.0617954`,
`cut = 50.3880515`. OCCT is exact to 8 digits at every angle in the trace corpus with `valid=1
naked=0`. Ours (`SESSION_SPHCYL=1 SESSION_SPHCYL_R=1.0`):

| tilt ° | our cut (f/naked) | our vol_cut | our common (f/naked) | our vol_common | OCCT vol_common |
|---|---|---|---|---|---|
| 0 | 2/0 | 50.388045 | 3/2 | 15.061795 | 15.0617955 |
| 20 | 2/0 | 50.388046 | 3/2 | 15.061795 | 15.0618286 |
| 23 | 2/0 | 50.387577 | 3/2 | 15.062264 | 15.0649961 |
| 23.400 | 2/0 | 50.387300 | 3/2 | 15.062540 | — |
| **23.500** | 2/0 | 50.387165 | 3/2 | 15.062676 | — |
| **23.550** | **3/2** | **NaN** | **2/2** | **12.330717** | — |
| 23.578178478201835 (exact tangency) | 2/0 | **15.061776** | 4/1 | 15.061794 | 15.0617954 |
| 23.600 / 23.700 / 23.900 | 2/2 | **NaN** | 4/0 | 15.058474 / 15.058771 / 15.059169 | 15.0617954 |
| 24.5 | 2/2 | **NaN** | 4/0 | 15.061794 | 15.0617954 |
| **26 / 28 / 30 / 31** | 2/3 | **NaN** | 3/1 | **15.908478** | 15.0617954 |
| 32 / 33 / 34 / 35 / 40 / 45 | 2/2 | **NaN** | 4/0 | 15.061794 | 15.0617954 |

Three measured facts:

1. **The `cut` is broken at EVERY tilt from 23.55° to 45°** — naked edges and a NaN volume at all
   of them except the exact-tangency angle itself, where it closes (`naked=0`) but returns
   `15.061776`, i.e. **the `common` region instead of the `cut` region**. The onset bracket is
   `(23.500°, 23.550°]`, and `asin(1/2.5) = 23.5781785°` is the exact angle at
   which the section circles stop encircling the sphere's pole, i.e. **the exact angle at which
   OCCT's section-curve count goes 2 → 3 because the pcurve must now break at the seam**
   (`sph_cyl_roty23_cut seccurves=2` → `sph_cyl_roty24_cut seccurves=3`, findings Q2). The onset of
   our failure and the onset of the required seam split are the same event.
2. **`common` is wrong by 15.908478 − 15.061795 = +0.846683 (5.62e-02) at 26/28/30/31°**, with
   `is_solid=0`, `naked=1`, 3 faces vs OCCT's 4. `15.908478` is the *exact* value this repo already
   documents for "one of the two section circles was dropped because its pcurve could not be built"
   (`intersection.cpp:4756-4766`, the `SESSION_SSI_KEEP` comment). The mitigation described there is
   default-ON, yet the broken value still reproduces, and `SESSION_NO_SSI_KEEP=1` changes **nothing**
   (identical to 6 digits at 25/30/35/40/45°) — consistent with both routes ending in `pa = v[0]`.
3. At 23° **OCCT is 7× worse than us** (OCCT +3.2e-3, ours +4.7e-4). Near-tangency is OCCT's own
   weakest band; correctness, not parity, is the target.

Secondary evidence for G1, `box × torR` (all-marched cell): per-face section **counts** disagree
with OCCT while per-face **total length** agrees to 4 decimals.

| box face | OCCT | ours |
|---|---|---|
| −X | 2 × 4.68285333 | **1** × 9.3657 |
| +X | 4 × 2.34142666 | **3** × {2.3414, 2.3414, 4.6829} |
| ±Y, ±Z | 1 × 5.82880714 each | 1 × 5.8288 each |

6 curves against OCCT's 10; the 4.6829 piece is exactly two OCCT arcs fused. The cell fails cut
2.11e-2 / common 4.83e-2 with **3 result faces against OCCT's 7**. (Honest caveat: our torus's seam
*phase* may differ from OCCT's, which would make some of this deficit legitimate. Distinguishing
"missed a crossing" from "no crossing to miss" needs a per-curve pcurve dump the harness does not
yet have; named as an audit item in §5.)

## 4.2 G2 — partially attributable

`box × torR` and `box × tor` are the only two all-marched cells. OCCT's plane×torus sections are its
loosest at `tol=1e-06`; ours are built to a 1e-4 chord budget. `box × tor` (upright) passes at
2.59e-07 and `box × torR` (tilted) fails at 2.11e-2, so **looseness alone is not sufficient to
fail** — it is a co-factor with G1, not an independent cause. No cell is attributable to G2 alone.

## 4.3 The 9.4e-05 cluster is **NOT** in this layer — measured, with three controls

`cyl × cylR`, `boxR × sph`, `boxR × cyl` all fail in the 3.4e-05 … 2.55e-04 band with **exact face
counts** and `is_solid = 1`. Their sections are exact rational conics
(`kb/hunt_oriented_primitives.md` §3: `deg=2 cv=9 rat=1`, lengths matching closed form to 12
digits, identical at every orientation). I ran three controls on all three cells, all three ops:

| control | effect on volume |
|---|---|
| `SESSION_PB_SMOOTH=1` (interpolated instead of degree-1 pullback pcurve) | **bit-identical** in 9/9 cells |
| `SESSION_PB_FIT=1` (fitted pullback pcurve, target 1e-9) | **bit-identical** in 9/9 cells |
| `SESSION_NO_SSI_KEEP=1` (different pcurve route entirely) | **bit-identical** in 9/9 cells |

So the degree-1 *section* pcurve of §2.3 has **no measured impact** on the 1e-4 cluster. That is
consistent with the brief's root cause (the *split face's boundary* pcurve, a different object) and
with `kb/hunt_oriented_primitives.md` §4. **Do not spend intersection-layer budget on the 1e-4
cluster.**

## 4.4 The flagship `coneR × cyl` is **NOT** an intersection-layer defect — measured

Geometry: cone(r=2,h=4) rotated −90° about X and translated to (0,−2.8,0), so its axis is the +Y
line `{x=0, z=0}`; cylinder(r=1.5,h=6) on the +Z axis. The axes are **perpendicular and
intersecting at the origin** (not skew). Our `ssi_cylinder_cone:4405` requires coaxial → the pair
falls to the marcher.

**New OCCT trace, `coneR_cyl_cut`:**

```
SEC tag=afterFF f1=2 f2=13 c=0 type=BSpline len=8.04334209 tol=4.00816023e-07
    p0 = p1 = 1.10641839,-1.01283678,0     (closed)
    box = -1.10641882,-1.5000497,-1.35007982 ; 1.10641879,-1.01283635,1.3500004
SUMMARY … FF=1 seccurves=1 secpb=1 res_face=3 res_naked=0 res_vol=12.307399 res_valid=1
```

One closed quartic loop, no closed form (the algebraic extremes check out exactly:
`1.25y² − 0.6y − 1.89 = 0` → `y ∈ [−1.5, −1.01283…]`, `x = ±1.10641839` at `z = 0`,
`z = ±1.35` at `x = 0`). OCCT itself only delivers an approximated BSpline here.

**Ours** (`SESSION_PAIR_SSI=1`, same geometry): `[PSSI] A0 x B0: 1 curves: len=8.0433`. The
marcher finds the same single closed branch, and the length agrees with OCCT's `8.04334209` to
every digit printed. Only one surface pair intersects at all (the cone's base disc at `y = −2.8`
and the cylinder's caps at `z = ±3` are out of reach), and OCCT confirms it: `FF=1`.

**Conclusion: the missing `cylinder × cone` analytic branch costs us nothing on this cell. The
coneR × cyl catastrophe (cut 43.1452 vs 12.3074, `is_solid=0` on all three ops, as reported in the
brief) is produced downstream of the section.** Note the run also took >2 min in-process for one
`cut` while OCCT's whole trace takes milliseconds; that cost is also downstream — the SSI printed
its single curve immediately.

## 4.5 Everything else: no measured impact yet

| gap | status |
|---|---|
| **G3** domain restriction | No isolated failure. `boxR × sph` has exact face counts and its residue is disproved as pcurve-related (§4.3). Correctness risk is real (our crossing points come from the UV arrangement at fit tolerance, OCCT's are exact conic∩arc roots), but unquantified. |
| **G4** cone × cone missing arm | The matrix's coaxial `cone × cone` cell is **OK at 5.63e-14** through the marcher. Untested poses exist in the trace corpus (`cone_cone_p1_cut` OCCT 15.7076198 / 4 faces; `cone_cone_p2_cut` OCCT 6.60028262 / 7 faces / **2 solids**) — neither is in any of our batteries. |
| **G5** plane × torus through-axis | No battery pose puts a plane through a torus axis. |
| **G6** ALine port (`IntAna_IntQuadQuad`) | Zero measured payoff: §4.4 for cyl×cone; and OCCT itself marches unequal-radius crossing cylinders (`cyl_cyl_cut` → BSpline). |
| **G7** seeding | No cell proven to lose a branch. The `box × torR` count deficit (§4.1) is either G1 or G7. |
| **G8** tangency | **The 54-cell edge/tangency battery is 54/54 OK** under our drop-the-tangency policy — `sph tanext`, `sph tanint`, `sph inscr` (6 simultaneous tangencies), `cyl tanline`, `tor linked` all exact to ≤5.1e-14 with correct face counts and `is_solid=1`. The one hard case (exact pole tangency at 23.578178478201835°) fails, but the bisection in §4.1 shows the failure band starts at 23.55° and continues at 23.6/23.7/23.9/24.5/26/…/45°, i.e. it is G1 (seam), not the tangency. **The tangency policy has no measured impact today.** |

Where OCCT is not the target: it is 7× worse than us at 23° (§4.1); its own cached vs FreeCAD
answers for `box × torR` disagree at 3.4e-6 relative and for `coneR × cyl fuse` at 4.7e-4
(`kb/hunt_oriented_primitives.md` §1), so do not quote those references past 1e-4; and it keeps
result edges of length 1.5e-05 (`cone_cone_p1_cut RESEDGE i=5`) while reporting `valid=1`.

---

# 5. IMPLEMENTATION ORDER — smallest shippable increment first

**I1 — stop discarding `v[1..]` in `analytic_ssi` (G1).** ~10 lines at
`intersection.cpp:4752-4776`: when a pullback returns `n > 1` segments, split `c3` at the same
3D points and push `n` triples. Under v2 this is even smaller (v2 reads only `get<0>`), so the v2
form is "emit `n` 3D sub-curves". *Acceptance*: `SESSION_SPHCYL=1 SESSION_SPHCYL_R=1.0
SESSION_SPHCYL_ANGLES=0,20,23,23.5,23.55,23.578178478201835,24,25,30,45` gives, at **every** angle,
`cut = 50.3880515 ± 1e-6`, `common = 15.0617954 ± 1e-6`, `is_solid=1`, `naked=0`. Oracle is closed
form, so no OCCT call is needed. This test currently fails at 10 of 10 angles on `cut`.

**I2 — a per-curve measured tolerance on the returned triple (G2 enabler).** Return the achieved
max deviation of the fitted 3D curve from both surfaces alongside the curve. No behaviour change;
it makes OCCT's propagation rule implementable (edge tol = curve tol; vertex tol = curve tol +
1.0e-12, findings Q5.3). *Acceptance*: for the 8 `sph_cyl_roty*` cases and `coneR_cyl_cut`, our
reported per-curve tolerance is within 2× of the OCCT `SEC tol` in the corresponding trace
(1.0099e-07 … 1.702e-05, and 4.00816023e-07).

**I3 — tighten the marched-curve budget (G2).** `refine_tol` from `max(tol·100, 5e-6)` to
`max(tol, 1e-7)`, `fit_track` target from `max(tol·10,1e-7)` to `max(tol, 1e-7)`
(`intersection.cpp:5871`, `:5986`). *Acceptance*: I2's measured tolerance for `box × torR` and
`box × tor` drops to ≤ 1e-06 (OCCT's own plane×torus number), with `box × tor` still OK at ≤1e-6
relative and no cell regressing. Watch the runtime: `box × tor` already costs 2.3 s per op.

**I4 — instrument section segmentation, then fix or close out G1's second half.** Add a
`SESSION_SSI_SEG` dump printing, per face pair, each returned curve's UV bbox on **both** surfaces
and its seam-crossing count. *Acceptance*: reproduce the `mx_box_torR` table of §4.1 and decide by
measurement whether our 6-vs-10 deficit is a missed crossing or a different seam phase. Only if it
is a missed crossing does more work land here.

**I5 — cone × cone dispatcher arm (G4).** Port `IntAna_QuadQuadGeo::Perform(Cone,Cone)`
(`IntAna_QuadQuadGeo.cxx:1433-1884`) plus the apex ray splitting (audit E6: tangency → 2 GLines,
interior → 4, each a separate line carrying the apex as a vertex) and the negative-nappe purge
(`param >= paramapex`, audit §2.1 trap 3). *Acceptance*: two new matrix cells built from the trace
corpus — `cone(r1=2,r2=0,h=5,center) × same rotx=90` (OCCT cut 15.7076198, 4 faces, 1 solid;
common 5.2542877, 5 faces) and `× same roty=35,tx=1` (OCCT cut 6.60028262, 7 faces, **2 solids**;
common 14.3436692, 6 faces) — pass at ≤1e-6 relative with exact face and solid counts. Run them
against the marcher first: if the marcher already passes, this arm is a performance change only.

**I6 — analytic domain restriction (G3).** Intersect each emitted conic with both faces' boundary
arcs in closed form; emit only in-domain sub-arcs with exact crossing vertices. *Acceptance*: for
`mx_boxR_sph` face pair (box side × sphere) we emit **4** arcs whose 8 endpoints match OCCT's to
1e-9 (they are `(1−√3, √3, ±1.64745307)` and `(0.94193295, −1.16737411, ±2)`); the cell's volume
error must not regress.

**I7 — branch-keyed seeding (G7).** Replace cell-centre AABB seeding with a triangle–triangle
polyhedral pass returning ordered section lines; walk largest-first with per-line re-seeding; drop
`seed_tol_3d`, `dup_tol` and `pair_budget` as branch-selection mechanisms. *Acceptance*: (a) no
change on the 45 currently-OK non-oriented cells; (b) a synthetic two-branch case whose branches
pass within `0.5·h_init` of each other returns 2 curves, where today it returns 1.

**I8 — typed tangency instead of delete (G8).** Return a tangency outcome from the marcher rather
than terminating, keep point contacts instead of `drop_point_sections`-ing them, and let the caller
absorb them by inflating the coincident vertex to `curve tol + 1e-12` (OCCT's measured rule).
*Acceptance*: the 54-cell edge battery stays 54/54, **and** a new cell `cyl(r=1) tangent to
cone(half-angle α) along a generator` — the class the audit flags as unreachable even in OCCT
(`IntAna_IntQuadQuad` drops isolated tangent points, audit §2.1) — produces a non-empty typed
result rather than silence. Last, because it has no measured impact today.

**Explicitly deprioritised**, with the measurement that justifies it: the `IntAna_IntQuadQuad`
ALine port (§4.4 — the marcher already reproduces OCCT's `coneR × cyl` section to every printed
digit); the section-pcurve degree (§4.3 — three controls, 9/9 cells bit-identical); and
`plane × torus`-through-axis (no battery pose reaches it).
