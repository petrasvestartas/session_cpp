# v2_section_notes — the FF SECTION STAGE, and the API the next stages consume

Module: `src/v2/brep_v2_section.{h,cpp}`, namespace `session_cpp::v2sec`.
Driver: `main_13.cpp` (target `main_13`, links `session_v2`).
Port target: `kb/port_04_ff_section.md` (OCCT `IntTools_FaceFace` + `BOPAlgo_PaveFiller::MakeBlocks`
+ `PostTreatFF`) and `kb/port_08_seam_pole_periodic.md` §2.8 (seam crossings).

Measured status, from the run that printed it (`./main_13`, 18.2 s): **76 checks passed, 0 failed.**
Nothing below is a projection; every number is reproduced by that binary.

---

## 1. WHAT THE STAGE IS

Four levels, the same split OCCT uses (`port_04` §2.0), so that the wire logic never leaks into
the marcher and the marcher's tolerances never leak into the wire logic.

| level | what | where |
|---|---|---|
| L1 | surface × surface | the EXISTING analytic SSI, `Intersection::surface_surface`. Not reimplemented, not modified. |
| L2 | footprints | `V2Curve::trail` — a parameter-synchronised `V2PntOn2S` list: one 3D point, both UV footprints, UNWRAPPED in periodic directions. |
| L3 | wires | paves at real entities only, then ONE 43.2 % classification per interval; kept or dropped whole. |
| L4 | arena | every surviving interval is a `BdsPaveBlock` on a carrier edge in `BdsArena`; `post_treat_ff` fuses across face pairs into one vertex / one edge per geometric feature. |

---

## 2. THE API

```cpp
#include "src/v2/brep_v2_section.h"
using namespace session_cpp::v2sec;

BdsArena ds;
const std::vector<const BRep*> ops = {&A, &B};
ds.init(ops, 1e-7);                    // SAME operand list, SAME order
V2Section S(ds, ops, V2SectionParams{});
S.perform_all();                       // every cross-operand face pair; runs post_treat_ff
```

### Entry points

| call | meaning |
|---|---|
| `V2FFStatus perform_pair(int arenaF1, int arenaF2)` | one face pair, by ARENA face index. Appends to `curves()` on `Ok`. |
| `int perform_all()` | every rank-0 × rank-1 face pair whose arena boxes overlap; returns the number of `Ok` pairs; runs `post_treat_ff()` when `params.posttreat`. |
| `void post_treat_ff()` | cross-pair fusion. Idempotent enough to call once; degrades to "no fusion", never to "no sections". |

For more than two operands, call `perform_pair` yourself for the pairs you want (`perform_all`
only walks ranks 0 and 1) and then `post_treat_ff()` once. `main_13` T7 does exactly this with
three operands.

### Typed outcome (`port_04` G9 — a tangency is never "Ok with 0 curves")

```cpp
enum class V2FFStatus { Ok, Tangent, Empty, NoGeometricSolution, Failed };
```
* `Empty` — the two arena boxes do not overlap. Decided before any SSI call.
* `Tangent` — the SSI produced nothing and the surfaces touch with parallel normals
  (`|n1·n2| > 1 - 1e-7`). Hand this pair to the same-domain stage (`brep_samedomain`).
* `NoGeometricSolution` — boxes overlap, SSI produced nothing, not tangent. **This is a
  diagnostic, not a silent zero** (OCCT `BOPAlgo_PaveFiller_6.cxx:545-553` emits a warning here).
* `Failed` — invalid face or surface.

### What you read afterwards

```cpp
struct V2Curve {                 // one geometric section curve of one face pair
    int face1, face2;            // ARENA face indices
    std::shared_ptr<NurbsCurve> c3d;
    std::vector<V2PntOn2S> trail;   // {t, p, u1,v1, u2,v2} — UNWRAPPED footprints
    bool closed;
    double dev1, dev2;           // MEASURED deviation over the KEPT blocks (G8)
    double tol;                  // max(face tolerance, measured deviation * (1+1e-5))
    int carrier_edge;            // arena edge carrying the WHOLE curve
    std::vector<V2Pave> paves;   // sorted, unique, each naming a real entity
    std::vector<V2Block> blocks; // tiles [t0,t1] EXACTLY; `kept` selects the emitted ones
    int seam_paves;
};

struct V2Block { double t0, t1; bool kept; int edge; int v0, v1; double length; BdsPB pb; };
struct V2Pave  { double t; int vertex; V2PaveOrigin origin; bool fused; int face; int edge; };
```

* `V2Block::edge` is the ARENA index of the minted section edge (a trimmed copy of `c3d`).
  After `post_treat_ff` every member of a common block carries the SAME index — that is the
  Law-1 guarantee, made literal by `BdsCommonBlock::set_edge`.
* `V2Block::pb` is the `BdsPaveBlock` in the arena. `ds.common_block(pb)` gives the fused group,
  `ds.real_pave_block(pb)` the representative.
* Vertex indices must be read through `ds.resolve_sd(v)` after fusion; `section_nodes()` does
  that for you.
* `V2Pave::origin` is **never overwritten**. When `post_treat_ff` merges the node it sets
  `fused = true` and leaves the provenance intact.

### Reporting helpers (all measured, none cached)

```cpp
std::vector<int> section_edges() const;      // distinct arena edges of kept blocks
std::vector<int> section_nodes() const;      // distinct arena vertices, SD-resolved
int    distinct_node_locations(double tol) const;   // G4 holds iff == section_nodes().size()
int    component_count() const;              // kept blocks joined through shared node ENTITIES
double kept_length() const;
double g1_residual() const;                  // max over kept blocks of max(d(c,S1), d(c,S2))
std::string signature() const;               // G11 tripwire, byte-comparable
const V2SectionStats& stats() const;
```

### Free functions the other v2 stages can reuse

```cpp
V2UvRect v2_uv_rect(const V2FaceRef&);
double   v2_invert(const NurbsSurface&, const Point&, double& u, double& v, bool seeded);
class    V2FaceClassifier;                   // per-face wires + seed grid; build ONCE per face
V2State  v2_state_point_face(const V2FaceRef&, double u, double v, double tol3d);
bool     v2_is_point_in_on_face(...);        // ON counts as IN
bool     v2_is_valid_point_for_face(...);    // project, reject on distance, then classify
bool     v2_is_valid_point_for_faces(...);
std::vector<std::pair<double,double>> v2_curve_curve_points(const NurbsCurve&, const NurbsCurve&, double tol);
double   v2sec_arclen(const NurbsCurve&, double t0, double t1);
inline double v2_intermediate_point(double a, double b);   // the 43.2 % point
```

`V2FaceClassifier` is the one to hold onto: constructing it tessellates the face's wires and
builds an inversion seed grid. `V2Section` caches one per arena face internally.

---

## 3. INVARIANTS THIS STAGE ESTABLISHES (and how they are checked)

* **G1 — on both surfaces.** `g1_residual()` ≤ **8.9e-16** on every analytic case measured
  (T1–T7). It is a max over kept blocks at 33 samples each, computed by inversion, not assumed.
* **G2 — inside both trims.** One `is_valid_point_for_faces` call at the 43.2 % point per
  interval. Kept or dropped WHOLE.
* **G3 — real entities only.** `V2PaveOrigin` has no `Bisection` member and no code path
  bisects a classification predicate. T10 measured 95 paves across three cases:
  `closing=20 curve_bound=8 seam_crossing=3 trim_crossing=64`, 0 untraceable.
* **G4 — one entity per feature.** T4: 24 arcs whose 48 endpoint instances resolve to
  **24 arena vertices at 24 distinct locations**; without `post_treat_ff` the same run reports
  48 entities at 24 locations. T7: the same curve arriving from two identical operands gives
  **48 blocks → 24 arena edges, 24 common blocks**.
* **G5 — no dangling ends.** `deg1 == 0` in every case; T4 additionally asserts min degree 2.
* **G7 — pose invariance.** T1: 21/21 rigid poses exact and structurally identical.
  T6(b): 9/9 poses, worst total-length drift **1.07e-14**.
* **G8 — measured tolerance.** `V2Curve::dev1/dev2` are measured over the KEPT blocks only and
  `tol` is derived from them; T9 measured worst deviation **6.6e-16** and 0 curves whose
  tolerance failed to cover their own footprints.
* **G11 — determinism.** T4 runs the whole stage twice and compares `signature()`.

---

## 4. FIVE THINGS THAT WERE WRONG AND HAD TO BE MEASURED, NOT REASONED

Recorded because each one silently produced a *plausible* wrong answer.

1. **Tolerance from the whole carrier.** Seeding the L3 verdict with the deviation measured over
   the WHOLE analytic curve folds in the distance from the parts that lie off the trimmed patch.
   On box × sphere that made `tol = 1.2`, and **every** block passed the trim test: 82 blocks
   kept instead of 24. The verdict tolerance must be the FACE tolerance
   (OCCT's `tolR3D`); the curve's own deviation is measured afterwards, over kept blocks only.
2. **Unwrapped seeds fed back into the evaluator.** The footprint trail must carry the unwrapped
   parameter but SEED the next inversion with the wrapped one. Feeding the unwrapped value back
   evaluates the surface out of range; the inversion then never moves and reports a "deviation"
   that is really the sampling step (measured: 2.81e-2 = exactly one trail step).
3. **A bracket built on the requested sag instead of the achieved one.** A refinement that hits
   its point budget silently widens the bracket it needs. Driving the tessellation from `tol`
   (1e-7 → ~14000 chords on a unit conic) guaranteed the budget was hit, and the gate then
   missed **every** seam crossing. Fix: measure the sag, gate on the measurement, and pick the
   sag for isolation only — Newton supplies the precision.
4. **Quadrature panels straddling knots.** `|C'(t)|` of a rational conic has a kink at every span
   joint. Uniform panels across them cost ~1e-7 relative, which showed up as section arclengths
   disagreeing with the closed form at 1e-6 and as false pose-variance. Integrate each knot span
   separately.
5. **The stored seam edge is not the seam.** `create_sphere`'s stored seam meridian misses its own
   sphere by **3.4e-4** and misses the true section by **4.0e-3**; intersecting it finds nothing.
   `NurbsSurface::iso_curve()` ignores the rational weights and returns radius **1.1497** for the
   unit sphere. The seam therefore has to be solved directly against the surface evaluator —
   `v2seam_refine` does a Gauss-Newton on `|C(t) - S(seam, w)|`, bracketed by the unwrapped
   footprint trail. (Both defects are in v1 files this session does not own; they are reported
   here, not patched.)

---

## 5. WHAT IS POSE-INVARIANT AND WHAT IS NOT — read this before writing an assertion

A rigid motion of BOTH operands cannot change anything. A rigid motion of ONE operand relative to
the other **can** change the number of blocks, and asserting otherwise is a false failure:

* invariant: number of section CURVES, number of connected COMPONENTS, total arclength, `deg1`,
  `nodes == distinct locations`, `g1_residual`;
* **not** invariant: number of blocks, number of paves, number of seam paves. Whether a section
  crosses a seam is a property of the relative pose. T2 measures exactly this: 2 curves and 2
  components at every tilt from 0° to 45°, total arclength `12.566370614` (= 4π, worst drift
  **3.55e-15**), while the block count moves between 2, 4 and 6 as the sections cross the
  sphere's and the cylinder's seams differently.

A section whose start parameter lands exactly ON the seam needs no seam pave: its closing pave is
already there and its single block spans one period seam-to-seam. Assert the invariant ("no
block's pcurve wraps"), not the pave count.

---

## 6. MEASURED RESULTS (`./main_13`, all 76 checks green)

| case | measured |
|---|---|
| T0 shared verdict harness | `v2v::v2_verdict` agrees with `is_solid()` on sphere/cone/cylinder and on two boolean results. Recorded finding: `box - cone` is topologically closed but has `closure_residual = 9.66e-02` — geometrically open, a class `is_solid()` cannot see. |
| T1 sphere × sphere, 21 poses | 21/21 exact. 1 curve, 3 blocks, 1 component. Worst \|r−√3/2\| **5.32e-12**, worst out-of-plane **5.30e-12**, worst \|Δlen\| **5.41e-12**, worst G1 **7.26e-12**. (The 1e-12 floor is the upstream recogniser re-FITTING the quadric — `port_04` §4.4 D12 — not this stage.) |
| T2 sphere × cylinder, 9 tilts 0…45° | 9/9. 2 curves and 2 components at every tilt, total arclength 4π with worst drift **3.55e-15**, worst G1 **8.67e-16**, every block inside one seam window. |
| T3 the 45° cut | 2 curves, 2 components, 4 blocks, 0 dangling ends, arclength 4π, G1 **8.4e-16**. Context (not a gate): the v1 boolean on the same inputs reports `naked=3 … closed=0`. |
| T4 box × sphere R=1.5 | 6 circles → **24 arcs** (4 per face), arclength `17.266946674` = the closed form, **8 loops** (one per box corner), every node degree 2. Fusion: 48 entities/24 locations → **24/24**. |
| T5 box × cylinder | 2 circles, arclength `7.539822369` = 2·2π·0.6, 2 loops, seam paved. |
| T5b sphere seam | 1 wrapping circle → **2 blocks**, 1 loop, exactly one SPHERE-seam pave, arclength 2π·√3/2 exact. |
| T6a box × cone (inside) | 2 circles, arclength `5.026548246` = 2π(r₁+r₂) exact. |
| T6b box × cone (through the sides), 9 poses | 9/9 pose-invariant, worst \|Δlen\| **1.07e-14**, worst G1 **3.55e-15**, 0 dangling ends. |
| T7 cross-pair EDGE fusion | 48 blocks from two identical operands → **24 arena edges, 24 common blocks, 24 nodes**; without fusion 48 edges and 96 nodes at 24 locations. |
| T8 typed outcomes | external tangency → `Tangent`, 0 curves, not `Ok`. Disjoint boxes → `Empty` with no SSI call. |
| T9 measured tolerance | worst deviation **6.568e-16**, 0 tolerance violations. |
| T10 provenance | 95 paves, 0 untraceable. |

---

## 7. WHAT IS NOT DONE HERE

* `is_existing_pave_block` (`port_04` §2.12 (5)/(3), G10): a section that runs along an EXISTING
  edge of either operand is not yet redirected onto that edge. `v2_curve_curve_points` already
  detects the "running along" case (it returns empty above 64 accepted roots) but does not act on
  it.
* `put_se_in_other_faces` (`BOPAlgo_PaveFiller_6.cxx:4277-4304`): section edges are not attached
  to faces that did not create them.
* EF-seeded walking (`port_04` F8) — there is no EF stage yet, and the analytic SSI does not take
  start points.
* Poles are handled only as classification boundaries: `V2PaveOrigin::PoleTouch` exists but no
  pave is created at a singular trim (degenerate edges are excluded as interference operands per
  `port_08` S6).
* Everything is `O(n²)` in the number of section blocks inside `post_treat_ff`. Fine at the
  corpus sizes measured (108 blocks worst); it needs a BVH before a large assembly.
