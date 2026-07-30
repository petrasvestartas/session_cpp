# FACE SPLITTER — integration notes for the assembly stage

`src/v2/brep_v2_splitface.h` / `.cpp` are NEW and self-contained: nothing in the existing kernel
calls them and they mutate nothing. Namespace `session_cpp::v2sf`. Test driver `main_14.cpp`,
**62/62 PASS** on an actual run
(`cmake -S session_cpp -B session_cpp/build_g -DCMAKE_BUILD_TYPE=Release && ./build_g/main_14`),
byte-identical output across 5 consecutive runs (`md5sum` of stdout equal 5/5). The only
shared-file edit was appending `14` to `foreach(MAIN_ID ...)` at `CMakeLists.txt:258`; the whole
tree builds green (`cmake --build session_cpp/build_g -j32`, exit 0, unity build ON).

Spec: `kb/port_09_builder_face.md` (guarantees G1–G10, OCCT `file:line` anchors),
`kb/port_08_seam_pole_periodic.md` (S1–S13), arena contract `kb/bds_integration_notes.md`.

---

## 1. What the subsystem is

```cpp
SfResult sf_split_face(const BdsArena& arena, const NurbsSurface& surface,
                       const std::vector<SfInputEdge>& inputs,
                       const SfOptions& opt = SfOptions());
```

One trimmed face + the arena pave blocks lying on it → the correct set of new faces.

| stage | what it does | OCCT analogue |
|---|---|---|
| `classify_multiplicity` | derives `is_inside` / `is_seam` from the INPUT LIST | `WireSplitter_1.cxx:148-151, :309` |
| `build_nodes` | vertex → records, keyed by **arena vertex index** | `mySmartMap` |
| `prune_to_fixpoint` | dangling + doubled-hanging prune, degenerate-exempt | `BuilderFace.cxx:152-235` |
| `compute_angles` | metric-corrected UV angle + secant tie escalation | `Angle2D` + `RefineAngles` |
| `walk` / `path` | stack machine, closure scan, truncate-and-continue | `WireSplitter_1.cxx:358-617` |
| **`compose_across_seams`** | **regions meeting across a seam become ONE face** | *(no OCCT analogue — see §4)* |
| `areas` | wire role from signed UV area; tightest-owner holes | `PerformAreas` :387-614 |
| `internals` | unconsumed material → INTERNAL wires, or reported | `PerformInternalShapes` :618-778 |

`SfEmitter` materialises the result into a `BRep`.

---

## 2. The contract you must satisfy when you call it

**Multiplicity is the caller's job**, exactly as in OCCT (`BOPAlgo_Builder_2.cxx:363-494`):

| input kind | entries | `pcurve` | `sense` |
|---|---|---|---|
| face boundary edge | **1** | its pcurve on this face | the loop's traversal sense |
| section / IN / INTERNAL edge | **2** | the SAME pcurve object | Forward + Reversed |
| **seam** edge | **2** | **two DIFFERENT pcurve objects** (u = u0 and u = u1) | opposite |
| pole / degenerate row | 1 (or 2 if internal) | its UV row | loop sense, `degenerate = true` |

Everything else is derived, so a caller cannot contradict the list.

Three rules that are easy to get wrong and that `main_14` exercises directly:

1. **`pcurve` is always parameterised in the pave block's NATURAL direction** — `pcurve(t_first)`
   is the UV of `pb->pave1.vertex`, `pcurve(t_last)` that of `pave2.vertex`. Direction lives in
   `sense`, never in the curve object. (v1 physically reverses the pcurve and always writes
   `reversed = false`, so the shared-edge relation is invisible in the data — divergence D14.)
2. **The arena builds pave-block pools LAZILY.** `BdsArena::pave_blocks(e)` returns an empty
   vector until something calls `change_pave_blocks(e)` / `add_pave(e, …)`. Materialise every
   edge of the operand once after `init()`, otherwise faces silently receive no boundary at all
   (this cost a full debug cycle: the box split returned 0 faces).
3. **A pole row must be present in the input.** Without it the face's UV loop is open, the
   meridians are valence-1 and the fixpoint prune eats the face. See §6 for the two conventions.

---

## 3. What it guarantees, and the measurement behind each claim

All numbers below are from the run above, not from a design intent.

### G1 — zero minting

`sf_split_face` creates no vertex, no edge, no curve. Output wires are **indices into the input
list**. `SfReport::minted_edges/minted_vertices` are asserted 0 and there is no code path that
could raise them.

### THE DECISIVE MEASUREMENT — canonical vs padded UV domain

A box split by its mid-plane, once on surfaces whose domain is exactly the face (`[0,1]²`, trims
at `{0,1}` — border-snappable) and once on surfaces that extend 1 % beyond the face and carry the
domain `[-0.04, 4.04]` with the trims at `{0,4}` — the STEP round-trip shape that makes a
boundary snap impossible.

```
box CANONICAL[0,1]        faces=10 edges=20 naked=0 nonman=0 solid=1
box PADDED[-0.04,4.04]    faces=10 edges=20 naked=0 nonman=0 solid=1   (v1 kernel here: 32 naked of 36)
topology IDENTICAL index-for-index   (354-char signature of every wire's (edge, block, sense) sequence)
adjacency: 20 distinct pave blocks, ALL shared by exactly 2 faces, 0 bad, 20 emitter reuses
mass properties: A=24.000000000000 V=8.000000000000 on BOTH domains
shared verdict v2v: naked_real=0 nonman=0 closure=0.00e+00 vol=8.0000000000 valid=1 on BOTH
```

This is oracle-free and it is the whole point. It cannot regress by accident: the padded run
executes the same code with a different `domain()`, and the only way the two could diverge is a
domain-relative epsilon, of which there are none.

### Adjacency is an INDEX, never a coordinate

`sf_pave_block_id(arena, pb)` returns `(arena edge index, position in that edge's pave-block
pool)` — a table read. `main_14` asserts the two faces on either side of every one of the 20
split edges report the **same pair**, and that the pave-block → face map is identical between the
canonical and padded runs. A pave block from a *different arena* with bit-identical geometry
resolves to `(-1,-1)`: geometry cannot mint identity.

### No sewing / welding / snapping API exists

`main_14` greps the header's declarations for `sew`, `weld`, `snap`, `quantis`, `find_near`,
`nearest`, `coincident_edge`, `merge_by` — zero hits. Their absence is the design.
`SfEmitter` materialises each distinct pave block as **exactly one** BRep edge, keyed by
pave-block identity, created on first use and reused thereafter.

---

## 4. THE ANGULAR ORDER — why the metric, and how much it buys

The sorting key at a multi-edge vertex is

```
angle = atan2( dv * sqrt(EG - F²),  du * E + dv * F )      E = Su·Su, F = Su·Sv, G = Sv·Sv
```

i.e. the angle of `dS = du·Su + dv·Sv` measured in the surface's own orthonormal tangent frame.

* **Not the 3D tangent, and not a projection of it.** A projection is not the surface's frame and
  can reverse the sense of rotation on a curved patch.
* **Not the raw UV tangent either — but for a subtle reason.** `J = [Su|Sv]` is nonsingular with
  `det > 0` in the frame that defines the FORWARD normal, so it preserves the CYCLIC ORDER of
  directions: in exact arithmetic raw UV picks the same winner. That is why OCCT can use it
  (`BOPAlgo_WireSplitter_1.cxx:768-840`). What raw UV does not give is a well-CONDITIONED key.
  Measured in `main_14` at anisotropy `|Su|/|Sv| = 1e6`, on four directions exactly 90° apart on
  the surface:

  ```
  min pairwise gap:  metric = 1.570796 rad     raw UV = 2.000e-06 rad     ratio 7.85e+05
  ```

  Raw UV loses ~6 decimal digits of the only information the walk has; at 1e12 anisotropy it
  loses 12 and the winner is decided by float noise in the secant sample. That is exactly the
  "correct arrangement, wrong lift" signature.
* **It reduces EXACTLY to `atan2(dv,du)` when `E=G=1, F=0`** — asserted to 1e-14 over 32
  directions. So it is raw UV plus a conditioning fix, not a different rule.

End-to-end: an X-crossing (valence-4 node, 8 records) on a *curved* parabolic strip with
`|Su|/|Sv| = 1e6` yields 4 faces with UV areas `2.4999999998e-07, 2.5000000000e-07, 0.499999750,
0.499999750` against the analytic `tri = 2.5e-07, side = 0.499999750`, unchanged under 8
arbitrary rotations of the whole configuration (signature stable).

**Degeneracy is typed.** At a pole `det → 0`; the key falls back to raw UV and reports
`SfAlert::MetricDegenerate`. `sf_uv_tolerance` returns false there. Both are asserted.

**Ties** are broken by SECANT ESCALATION, not a nudge: only the tied records are re-measured at
0.01 / 0.05 / 0.15 / 0.40 of the pcurve span, which is what separates two curves leaving a vertex
tangentially. A "tie" is only ever two OUTGOING records at the SAME UV image of a vertex with
different pave blocks — the only pair the leftmost-turn comparison ever compares. (Getting that
scoping wrong reports `AngleTieUnresolved` on every seam node, where a seam below and a seam above
are collinear by construction and are not an ambiguity.) This replaces OCCT's `RefineAngles` /
`RefineAngle2D` (`WireSplitter_1.cxx:905-1125`) and its ±`Precision::Angular()` last-resort nudge.
`AngleTieUnresolved` count on the whole suite: **0**.

**No epsilon is a fraction of the UV domain** (G7). Every parametric epsilon is
`sf_uv_tolerance(surface, u, v, tol3d)` = `tol3d / |Su|`, `tol3d / |Sv|`, computed at the point of
use. Asserted: the same geometric plane on `[0,1]²` and on `[-0.04,4.04]²` gives UV tolerances in
the ratio 4.080000000 with `du·|Su| == tol3d` in both.

---

## 5. SEAM COMPOSITION — the primary requirement, and how it works

`u = u0` and `u = u1` are THE SAME 3D CURVE. A section that **straddles** the seam cuts the UV
rectangle into more regions than the surface has faces: a circle straddling a sphere's meridian
leaves a lune against `u = u0` and a lune against `u = u1` that are the two halves of ONE disk.
Wire walking alone gives three areas where the answer is two, and the seam pieces between the
lunes stay 1-trim — naked. This is orientation-CHAOTIC (one degree of tilt moves a section across
a seam) and never touches box × box, because a plane has no seam.

**The rule** (`compose_across_seams`, run after the walk, before the area stage):

> A SEAM pave block whose two occurrences land in TWO DIFFERENT wires is INTERIOR to their union
> → splice the two wires into one and drop both occurrences.
> A SEAM pave block whose two occurrences land in the SAME wire is a genuine seam of that face
> and is left alone (that face wraps the chart; the edge legitimately carries two trims belonging
> to one face).

The splice is `chain[0..p-1] ++ rotate(other, j+1) ++ chain[p+1..]`, and the spliced-in half is
translated by the **exact whole-period offset measured between the two pcurves of the shared
block** — `pc1(mid) − pc2(mid)`, stored per occurrence in `SfWire::shift_u / shift_v`. That is
OCCT's `AdjustPCurveOnSurf`, derived from the SHARED ENTITY instead of from coordinates.
Composition is driven entirely by pave-block identity; no coordinate is matched. It iterates, so
a component of three wires (torus) composes in two splices.

Measured:

```
straddle sphere    faces=2  seam_merges=1  composed_wires=1  alerts=0
                   composed wire closes in UV to 5.511e-17
                   UV areas partition the chart: 8.000000000000 / 8.000000000000
                   edges=6 naked=0 nonman=0 degen=2 seam=2 orphan=0 solid=1
                   v2v: naked_real=0 nonman=0 closure=1.14e-16 vol=4.1887902048 valid=1   (= 4/3 pi)
                   invariant over 12 arbitrary rotations, signature stable
straddle cylinder  faces=2  seam_merges=1  composed_wires=1  alerts=0  uv gap 4.578e-16
                   capped: edges=6 naked=0 nonman=0 seam=2 orphan=0 solid=1
                   invariant over 12 rotations
torus (1,1) cut crossing BOTH seams
                   faces=1  seam_merges=2  alerts=0   uv gap 0.000e+00
                   composed UV area = 16.000000000000 = the whole chart
                   edges=4 naked=0 nonman=0 seam=4 solid=1
                   v2v: naked_real=0 nonman=0 closure=1.87e-13 vol=14.2122303376 valid=1
                        (= 2 pi^2 R r^2 for R=2, r=0.6)
                   invariant over 8 rotations
```

The torus result is the strongest structural statement in the suite: cutting a torus along a
(1,1) curve leaves a CONNECTED cylinder, so the answer is **one** face, and the composition finds
it from three walk wires by composing in **both** parameters.

### Two consequences the assembly stage must know

1. **The seam piece between two straddle crossings is never materialised.** It is interior to the
   composed face and belongs to nothing else, so `SfEmitter` never creates an edge record for it.
   Sphere straddle: 6 edges, not 7, and **0 orphans**. Do not expect it and do not look for it.
2. **A composed face reaches up to one period OUTSIDE the chart's stored rectangle**, and our
   charts are CLAMPED NURBS — evaluating past the domain is *not* the periodic continuation.
   `SfEmitter` therefore emits a periodically CONTINUED surface for such a face:

   ```cpp
   bool sf_periodic_extend(const NurbsSurface& s, int dir, double lo, double hi, NurbsSurface& out);
   ```

   It repeats the control net by one period per call (the net's first and last rows must coincide
   and the knot pattern must repeat — true of every quadric chart `Primitives::` builds: 9 CVs,
   knots `{0,0,1,1,2,2,3,3,4,4}`) and returns **false**, changing nothing, when that does not
   hold. `SfEmitter::extended_surfaces()` / `inextensible_faces()` report both outcomes.
   Without this the composed face is topologically right and geometrically unreadable: the same
   shapes score `closure_residual = 1.16e-01 / 2.20e-01` and a wrong volume. With it they score
   `1.14e-16 / 1.87e-13` and the exact volume. **If you re-emit or re-parameterise faces
   downstream, preserve this.**

---

## 6. Poles — the convention is stated, not accidental

None of this kernel's primitives carries a zero-length EDGE record at a singularity
(`create_sphere` = 1 edge, `create_cone` = 2). Two conventions exist and both are watertight:

* `SfEmitter::set_pole_edges(true)` (default) — OCCT / `port_08` S5: the pole row is a real edge
  record with a real pcurve, one vertex used twice and zero 3D extent. Every validity rule here
  (`BRep::is_solid` `brep.cpp:1090-1102`, `v2v::v2_verdict`, `sf_manifold`) already excludes
  zero-length edges from the naked count.
* `SfEmitter::set_pole_edges(false)` — the primitives' convention: the singular trim keeps its
  pcurve and carries `edge_index = -1`; no zero-length record is created.

The SPLITTER is unaffected either way — it mints nothing, and the pole row must be in the INPUT
list as a degenerate `SfInputEdge` in both conventions, otherwise the UV loop is open and the
meridians dangle. Measured, on the same split sphere:

```
with_pole_edges { edges=5  degen=2  naked_real=0  vol=4.1887902048 }
edgeless        { edges=3  degen=0  naked_real=0  vol=4.1887902048 }
```

Identical verdict, identical volume, two fewer edge records. **Pick one and keep it**; an
inconsistency here is what produces "watertight but wrong".

---

## 7. Scoring — use the SHARED harness

`main_14` scores through `session_cpp::v2v::v2_verdict` (`src/v2/v2_verdict.h`) and asserts on
every shape that it AGREES with this file's own cheap census `sf_manifold`, and with
`BRep::is_solid()`. `sf_manifold` exists only as a topology-only inner-loop diagnostic; it is
never the verdict.

Before scoring anything, `main_14` validates the metric against `is_solid()` on four known-good
un-split solids (sphere, cone, cylinder, box) **and** on a split sphere that actually carries
pole edges — the one shape where the trap bites:

```
verdict split_sphere_WITH_pole_edges   is_solid=1  sf_closed=1  edges=5 naked=0 degen=2 seam=2
verdict naive_nt!=2_count_WOULD_HAVE_LIED   naive says 2 naked; sf_manifold says 0
```

Never score with a bare `trims != 2` count: it reads a watertight sphere as open (2 poles), and a
seam edge appears ONCE in the edge list while being used TWICE by its own face's wires.
**And never score on volume alone** — unmerged regions across a seam can sum to the right volume
while leaving naked edges. Require `naked_real == 0 && nonmanifold == 0 && closure_residual small`.

---

## 8. Determinism

Node order is the first-appearance order of OUT ends in the input list; records inside a node keep
input order; the leftmost-turn comparison is strict-improvement with a one-ulp margin
(`SF_ANGLE_EPS = Epsilon(1.)`), so the FIRST candidate in list order wins a tie. `main_14` asserts
byte-identical results on a repeated run and the stdout `md5sum` is equal across 5 consecutive
process runs.

---

## 9. Known gaps — what this deliverable does NOT do

1. **It does not build the input list.** Assembling boundary + IN + section + seam entries with
   the right multiplicity, and producing each occurrence's pcurve, is the caller's job
   (`BOPAlgo_Builder_2.cxx:233-507` is the model). `main_14` does it by hand for every test.
2. **`brep_v2_section` was not available**; every test drives the splitter with section edges
   constructed directly in the arena. When the section stage lands, only the input assembly
   changes — no splitter code.
3. **Internal wires (`PerformInternalShapes`) are implemented but untested** — no test in the
   suite produces a dangling section. `SfAlert::EdgeNotConsumed` / `DoubledHangingPruned` fire and
   `SfResult::unused` reports; the containment classification uses the same UV point-in-polygon as
   the hole stage.
4. **Hole → owner uses the sampled UV polygon**, not `IntTools_FClass2d`'s re-discretisation loop
   (`IntTools_FClass2d.cxx:458-534`). Wire polygons ARE sampled at the pcurve's own knots, so a
   degree-1 pcurve (what a real imprint produces) is reproduced exactly; a high-degree pcurve with
   large sag could still mis-sign a very thin wire. `SfAlert::WireDegenerateArea` is the tripwire.
5. **`sf_periodic_extend` only handles a repeating periodic control net.** A trimmed or re-fitted
   periodic surface whose end rows do not coincide bitwise returns false and the face is counted in
   `inextensible_faces()`.
6. **Observation for session A, not a splitter issue:** `brep_massprops` integrates a PLANAR face
   bounded by a SINGLE closed trim over the whole parameter rectangle (`path = FullDomain`) instead
   of over the disk — measured on the hand-built cylinder caps in `main_14` (`area = 4` instead of
   `pi` per cap, `closure_residual = 3.89e-01`). Any capped cylinder/cone in the corpus will score
   geometrically open for that reason alone.
