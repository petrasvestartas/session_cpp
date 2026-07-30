# Gap 5 — THE DEGENERACY LAYER

**Question answered:** what must be implemented in our kernel so that coincident, tangent and
near-degenerate configurations produce correct booleans.

**Method.** Everything below is measured. Ground truth is the instrumented tracer
`validation/occt_trace/` (OCCT 8.0.0.rc2-a66b3fd6, `SetRunParallel(false)`, `SetUseOBB(false)`,
default fuzzy). The existing 38-case corpus contains only **three** degenerate configurations
(`box_box_touch_fuse`, `box_box_touch_common`, `box_box_half_fuse`) plus the pole-tangency sweep,
so **17 new traces were run for this analysis** covering partial coincidence on a curved face,
point tangency, line tangency on and off a seam, coplanar containment and apex contact. The same
cases were then run through our own v2 pipeline with `main_20` (the v2 tracer, identical CLI), so
every gap below carries a *paired* number.

Reproduce the new OCCT traces (`validation/occt_trace/`, binary already built at `build/occt_trace`):

```
B=./build/occt_trace
$B --op fuse   --a cylinder,r=1,h=4,center --b cylinder,r=1,h=4,center,tz=2   --name cylsame_fuse
$B --op common --a cylinder,r=1,h=4,center --b cylinder,r=1,h=4,center,tz=2   --name cylsame_common
$B --op cut    --a cylinder,r=1,h=4,center --b cylinder,r=1,h=4,center,tz=2   --name cylsame_cut
$B --op cut    --a box,dx=4,dy=4,dz=4,center --b box,dx=4,dy=4,dz=4,center,tx=4 --name box_box_touch_cut
$B --op cut    --a sphere,r=1,tz=3        --b box,dx=4,dy=4,dz=4,center       --name sphtang_box_cut
$B --op fuse   --a sphere,r=1,tz=3        --b box,dx=4,dy=4,dz=4,center       --name sphtang_box_fuse
$B --op common --a sphere,r=1,tz=3        --b box,dx=4,dy=4,dz=4,center       --name sphtang_box_common
$B --op fuse   --a sphere,r=1             --b sphere,r=1,tx=2                 --name sphsph_tang_fuse
$B --op common --a sphere,r=1             --b sphere,r=1,tx=2                 --name sphsph_tang_common
$B --op cut    --a cylinder,r=2,h=6,center --b cylinder,r=1,h=6,center,tx=1   --name cyltang_in_cut
$B --op common --a cylinder,r=2,h=6,center --b cylinder,r=1,h=6,center,tx=1   --name cyltang_in_common
$B --op cut    --a cylinder,r=2,h=6,center --b cylinder,r=1,h=6,center,tx=-1  --name cyltang_off_cut
$B --op common --a cylinder,r=2,h=6,center --b cylinder,r=1,h=6,center,tx=-1  --name cyltang_off_common
$B --op fuse   --a cylinder,r=1,h=6,center --b cylinder,r=1,h=6,center,tx=2   --name cyltang_out_fuse
$B --op fuse   --a cone,r1=2,r2=0,h=5     --b cone,r1=2,r2=0,h=5,rotx=180,tz=10 --name coneapex_fuse
$B --op common --a cone,r1=2,r2=0,h=5     --b cone,r1=2,r2=0,h=5,rotx=180,tz=10 --name coneapex_common
```

Same specs through our kernel: `session_cpp/build_v2diff/main_20 --op … --a … --b … --out …`.

---

## 0. THE HEADLINE, RANKED BY MEASURED IMPACT

| # | Gap | Measured cost | Where |
|---|---|---|---|
| D1 | **Partial coincidence on a curved face is not detectable at all.** `SameDomain` buckets faces by *identical edge set* and its tolerant fallback additionally requires *equal edge count and equal bbox*; two partly-overlapping faces satisfy neither. | `cylsame` (coaxial overlapping cylinders): all three ops wrong by **exactly 2π/3 = 2.0943951** — fuse +11.11 %, common −33.33 %, cut +33.33 % — and all three come back **open, 5 naked edges, `res_solid=0`**. OCCT: exact on all three, `naked=0 valid=1`. | §A |
| D2 | **No edge-on-face (EF) imprint feeds the face splitter**, so the *other* operand's boundary edges never cut a coincident face. This is the mechanism OCCT uses for every partial-coincidence case; our only route is the FF section, which is empty for a coincident pair. | Same three cells as D1, plus the `cyltang_off` coplanar-disc-in-disc region. In OCCT the imprint arrives as `IEF … ctype=EDGE` → `CB … nfaces=1` → `FaceInfo.In`. Our v2 `secpb=0` on those pairs — i.e. we discard the only curve we had. | §A |
| D3 | **The ON state is decided by ONE probe** at the largest CDT triangle's *equal-weight* centroid, with a straddle band of `diag·2e-3` (≈ 4 orders looser than OCCT's `tolF1+tolF2+Confusion`). The asymmetric-probe fix exists but is **gated off by default** (`SESSION_PROBE_FIX`). | Historical, from the code's own measured notes: the legacy rim-only ON test fires **20/20 on A-op-A** (all spurious) and cost y30 cut **23.9619 vs 46.9596 reference** (−49 %); it is still the default (`SESSION_ONIMP_FIX` off). | §A, §D |
| D4 | **Tangency is a diagnosis, not an action.** `V2FFStatus::Tangent` is produced only when the SSI already returned nothing, and no consumer reads it. The marcher's contract says outright: *"Marching terminates at tangencies (n_a ∥ n_b); tangential intersections are unsupported."* | `cyltang_off_cut` (line tangency off-seam): **56.5448431 vs 56.5486678 = 18π**, −3.82e-3 abs / −6.76e-5 rel, **open, 6 naked edges**. OCCT exact, closed. | §B |
| D5 | **The pole/apex degenerate edge is destroyed whenever the face is split.** Away from tilt 0 our sphere-cylinder results carry `ndegen=0` where OCCT carries 2 or 3, and our own `is_solid()` then rejects a shell whose volume is right to 7 digits. | sph×cyl common sweep: `valid=0` at 20/23/24/30/45° with `naked=0` and volume error ≤ 6e-5. At the exact tangency the same defect becomes a *volume* error: **15.0591515 vs 15.0617954, −2.6439e-3 (−1.76e-4 rel)** — our worst point in the sweep and OCCT's best. | §C |
| D6 | **No same-domain hook drives selection.** Both `SESSION_SD` hooks in `brep.cpp` print and return; `sd_boolean_coincident` is a test driver. The v2 fence (`v2sol_build_sections`) *does* consult `SameDomain`, but only to suppress the SSI — nothing rebuilds the shared wall. | `x20 cut` deletes 12 of A's 36 fragments whose probe points sit at distance **0.00e+00** from B's boundary while our ON-detector reports `on=0` for every one (recorded at `brep.cpp:8919-8921`). | §A |

Two things our kernel already gets **right** and must not regress:

* **Point tangency.** `sphtang_box_fuse` v2 = **68.1887902**, OCCT = **68.1887902**, both `res_solid=2`,
  `naked=0`, `valid=1`. We reproduce OCCT's regularized convention (measure-zero contact never
  merges lumps) exactly.
* **The BuildBOP orientation op-table.** `sd_select_face`
  (`src/brep_samedomain.cpp:572-590`) was previously verified only against source. It now agrees
  with measurement on every ON row available: touch-fuse (ON-opposite, Fuse → drop both),
  touch-common (ON-opposite, Common → drop both, empty result), touch-cut (ON-opposite, Cut → keep
  once from A), cylsame fuse/common (ON-same → keep once from A), cylsame cut (ON-same, Cut → drop
  both). See §A.4.

---

## A. COINCIDENCE / SAME-DOMAIN

### A.1 What OCCT does

**A.1.1 Candidacy is edge-set identity, and it is only sound because the edges are already shared.**

`BOPAlgo_Builder::FillSameDomainFaces` (`BOPAlgo_Builder_2.cxx:580-800`):

* Only faces that appear in an `InterfFF` record **and** have `FaceInfo` are considered
  (`:657-681`).
* Each face's *images* (or the face itself if unsplit) are hashed by `AddEdgeSet`
  (`:562`, `:726`, `:735`) into a `BOPTools_Set` of its EDGES. Buckets of fewer than two are
  skipped (`:758`).
* A pair inside a bucket is discarded if both faces trace back to the **same source solid**
  (`:769-779`) — the zero-thickness guard.
* **Planar + bounded pairs are declared same-domain with no geometric test at all** (`:780-785`).
* Everything else goes to `AreFacesSameDomain`.

This works because `PostTreatFF` has already fused the two operands' copies of every section edge
into one `TShape`, so "identical edge set" is an *identity* test, not a geometric one.

**A.1.2 `AreFacesSameDomain` is one-directional and fail-closed.**
`BOPTools_AlgoTools.cxx:1139-1205`:

* one interior point from **F1 only** (`:1153`); `iErr != 0` ⇒ **return false** (`:1155-1159`) —
  "cannot decide" is reported as "not same domain", and there is deliberately no
  `PointNearEdge` fallback;
* the band is `aTolF1 + aTolF2 + max(fuzz, Confusion)` (`:1199`), where the **max non-degenerate
  edge tolerance of F1** raises *both* `aTolF1` and `aTolF2` (`:1173-1196`) — F2's own edge
  tolerances never enter;
* the verdict is `IsValidPointForFace` (`IntTools_Context.cxx:647-673`): project onto F2's
  surface, reject if `LowerDistance > aTol` (strict `>`, so `== aTol` passes), then
  `IsPointInOnFace` — an ON classification counts as inside.

**A.1.3 The interior point deliberately avoids symmetric loci.**
`BOPTools_AlgoTools3D::PointInFace` (`BOPTools_AlgoTools3D.cxx:906-937`) shoots a vertical hatch
line at `aUx = IntTools_Tools::IntermediatePoint(aUMin, aUMax)`, and
`IntTools_Tools::IntermediatePoint` (`IntTools_Tools.cxx:254-259`) is

```
constexpr double PAR_T = 0.43213918;               // documented as 10*e^(-pi)
return (1. - PAR_T) * aFirst + PAR_T * aLast;
```

If the hatcher fails it retries at the **mirror** `aUx = aUMax - (aUx - aUMin)` = 0.56786082 of
the span (`:932`) — a second asymmetric value, never 0.5. Inside the hatch domain the V is picked
the same way (`:1053-1055`). The same constant governs `IsValidBlockForFace`
(`IntTools_Context.cxx:695-706`) and every "intermediate point of a block" in the BO package.

**A.1.4 The BuildBOP orientation op-table.** `BOPAlgo_Builder.cxx:479-737`:

* `isSameOriNeeded = (theObjState == theToolsState)` (`:641`) — true for FUSE (OUT/OUT) and
  COMMON (IN/IN), false for CUT (OUT/IN) and CUT21 (IN/OUT);
* filtering: FUSE avoids every IN face (`:670`); CUT avoids faces IN for **both** groups (`:676`);
* the SD wall is decided by **two fences**: `aMFence` (unoriented `IsSame`, `:682`) and
  `aMFenceOri` (oriented `IsEqual` = TShape + Location + Orientation, `:696`/`:714`).
  `isSameOri = !aMFenceOri.Add(aFIm)`; the wall is kept once iff
  `isSameOriNeeded == isSameOri` (`:697`), else it is added to `aMFToAvoid` (`:708`) and *both*
  copies die.
* ordinary faces: kept iff `bTakeIN == isINOpposite` (`:719`); an IN face kept by CUT/CUT21 is
  added **reversed** (`:722-729`).

There is no "ON" state anywhere in OCCT. The ON-same/ON-opposite distinction exists *only*
because those two fences are kept separate — which is why the decision must be carried on the
**pair**, never read off one face.

**A.1.5 Full coincidence, measured.** `box_box_touch_fuse` (two 4³ boxes flush at x=+2):

```
IVV  15↔39 → 68 | 16↔40 → 69 | 18↔42 → 70 | 20↔44 → 71        (8 source verts → 4 NEW verts)
IEE  14↔38 ctype=EDGE r1=0:4 r2=0:4  (×4)
CB   id=0 tol=1e-07 edge=-1 npb=2 pbs=14:0:4|38:0:4 nfaces=0   (×4, edge-edge flavour)
IFF  12↔36 tangent=1 ncurves=0 npoints=0                       (the coincident wall)
IMGFACE a=0 f=2 split=1 nimg=1 nkept=0 out=0                   (A's +X face DROPPED)
IMGFACE a=1 f=1 split=1 nimg=1 nkept=0 out=0                   (B's -X face DROPPED)
RES nsolid=1 nface=10 vol=128 naked=0 valid=1
```

Note the coincident vertices are **not** widened; they are replaced by four brand-new DS vertices
(indices 68-71 ≥ `NbSourceShapes` = 68) — the opposite of the tangency rule in A/§B.2. Note also
that the four *coplanar but non-overlapping* side-face pairs each get `tangent=1` and are **not**
merged: `res_face=10`, eight faces of area 16 each. Coplanar-face unification is
`ShapeUpgrade_UnifySameDomain`, a separate post-step, not part of the boolean.

The CUT of the same pair (`box_box_touch_cut`, new trace) keeps the wall once and proves the
SD-fusion survives into the result:

```
IMGFACE a=0 f=2 split=1 nimg=1 nkept=1 out=2
IMGFACE a=1 f=1 split=1 nimg=1 nkept=1 out=2     <- SAME output face index
RES nsolid=1 nface=6 vol=64 naked=0 valid=1
```

`box_box_touch_common` returns an **empty compound, `valid=1`, no error** — the regularized
answer, not a failure.

**A.1.6 PARTIAL coincidence, planar — `box_box_half_fuse`** (4³ box × 4×4×2 box at tx=4; the
shared strip is 4×2 inside a 4×4 face). The overlap boundary is produced entirely **without an FF
section between the coincident pair**:

```
IFF 12↔36 tangent=1 ncurves=0                       (coincident pair: no curves, ever)
IVE 39↔14 t=3 | 40↔14 t=1 | 42↔19 t=3 | 44↔19 t=1   (B's verts split A's edges at t=1,3)
IEE 14↔38 ctype=EDGE r1=1:3 r2=0:2                  (PARTIAL edge coincidence)
CB  id=0 npb=2 pbs=14:1:3|38:0:2 nfaces=0           (edge-edge, sub-range of A = whole of B)
IEF 41↔12 ctype=EDGE r1=1e-07:3.9999999             (B's edge lies ON A's face)
CB  id=2 npb=1 pbs=41:0:4 nfaces=1 faces=12         (EDGE-ON-FACE flavour)
FI  f=12 nIn=2 nOn=6   In = {41, 45}                (B's two edges are IN-edges of A's face)
IMGFACE a=0 f=2 split=1 nimg=3 nkept=2 out=0,6,8    (A's face -> 3 pieces, 2 kept)
IMGFACE a=1 f=1 split=1 nimg=1 nkept=0 out=0        (B's face dropped whole)
RES nface=12 vol=96 naked=0 valid=1
```

Read the mechanism off that: **the coincident region's boundary is delivered by EF (edge-on-face
CommonBlock → `FaceInfo.In`) and by partial EE/VE, and only then by FF for the *non*-coincident
neighbours.** At `tag=final` the CommonBlock has elected a representative edge (`edge=38`) and
A's `[1,3]` block resolves through it, so both operands index the *same* edge.

**A.1.7 PARTIAL coincidence, CURVED and PERIODIC — `cylsame` (new).** Two r=1 h=4 cylinders,
one at tz=2, so their lateral faces lie on the *same infinite cylinder* and overlap over z∈[0,2].

```
STAGE: Init VV VE EE VF EF [VV VE VF] FF            <- RepeatIntersection fires
IFF 2↔15 tangent=1 ncurves=0                       <- coincident lateral pair, no curves
IEF 21↔2  ctype=EDGE r1=1e-07:6.28318521           <- B's bottom rim ON A's lateral
IEF 4↔15  ctype=EDGE r1=1e-07:6.28318521           <- A's top rim ON B's lateral
CB  id=2 npb=1 pbs=21:0:6.28318531 nfaces=1 faces=2
CB  id=0 npb=1 pbs=4:0:6.28318531  nfaces=1 faces=15
IVE 20↔6 t=2 | 5↔19 t=2                            <- each seam vertex inside the other seam
IEE 6↔19 ctype=EDGE r1=2:4 r2=0:2                  <- PARTIAL seam-line coincidence
CB  id=1 npb=2 pbs=6:2:4|19:0:2 nfaces=0
FI  f=2  nIn=1 nOn=3   In={21}                     <- B's rim is an IN-edge of A's lateral
FI  f=15 nIn=1 nOn=3   In={4}
IFF 2↔24 ncurves=1 npb=0 | IFF 9↔15 ncurves=1 npb=0  <- disc x lateral circles, DISCARDED
IMGFACE a=0 f=1 split=1 nimg=2 nkept=2 out=1,2
IMGFACE a=1 f=1 split=1 nimg=2 nkept=2 out=1,3     <- out=1 shared by BOTH: SD-fused face
RES nsolid=1 nface=5 vol=18.8495559 naked=0 valid=1
```

Three rules fall straight out:

1. **A coincident face pair never produces a section curve** — planar or curved, full or partial
   (`tangent=1 ncurves=0` in every one of the four such pairs in the corpus).
2. **The split of a partly-coincident face comes from the other operand's EDGES via EF**, deposited
   in `FaceInfo.In`. On a periodic face this includes the *seam*, split by VE at the partner's seam
   vertex and paired by a partial-range EE CommonBlock.
3. **The coincident sub-face is then fused to ONE face** (`out=1` is an image of both operands'
   lateral face) and the op-table decides how many copies survive.

**A.1.8 Coplanar containment — `cyltang_off` (new).** B's r=1 top disc sits strictly inside A's
r=2 top disc, same plane, same outward normal. `IFF 9↔22 tangent=1 ncurves=0`;
`IMGFACE a=0 f=2 split=1 nimg=2 nkept=1` → the big disc becomes an **annulus**
(`RESFACE i=3 surf=Plane nwire=2 nedge=4 area=9.42477796 = 3π`), the inner copy is dropped for
CUT. The imprint again arrives through EF: `CB id=0/1 pbs=17:0:π and 17:π:2π nfaces=1 faces=9`.

**A.1.9 `TangentFaces()` cannot be used as a decision input.** Measured over the whole corpus:

| configuration | `fftangent` |
|---|---|
| full coplanar overlap + 4 coplanar edge-sharing pairs (`box_box_touch_*`) | 5 |
| partial coplanar overlap + 2 coplanar edge-sharing (`box_box_half_fuse`) | 3 |
| partial coaxial *cylindrical* overlap (`cylsame_*`) | 1 |
| cone-to-cone **apex point** contact (`coneapex_*`) | 1 |
| coplanar disc-inside-disc, 2 pairs (`cyltang_off_*`) | 2 |
| **line tangency** cylinder-in-cylinder (`cyltang_in_*`) | **0** |
| **exact pole tangency** sphere×cylinder 23.578178478° | **0** |
| **point tangency** sphere/plane (`sphtang_box_*`) | **0** |
| **point tangency** sphere/sphere (`sphsph_tang_*`) | **0** |
| all 23 transverse cases | 0 |

It is set for coplanar faces whose interiors are **disjoint** and unset at three genuine
tangencies. It is neither a tangency test nor a same-domain test; the observed meaning is "the SSI
produced no walkable transversal branch".

**A.1.10 OCCT is not self-consistent on redundant section curves.** In `box_box_half_fuse` the
geometrically symmetric pairs 12×56 and 12×60 give `ncurves=0` and `ncurves=1` respectively; in
`box_box_touch_common` two of the four perimeter pairs emit a curve and two do not. Every such
curve carries `npb=0` and is discarded. **A correct kernel must therefore treat "the section curve
lies wholly on an existing edge" as a first-class discard case and must never depend on the curve
being emitted at all.**

### A.2 What we do

* `src/brep_samedomain.h/.cpp` — a faithful, well-documented port of `FillSameDomainFaces`,
  `AreFacesSameDomain`, `BOPTools_Set` and the BuildBOP op-table. Candidacy
  (`brep_samedomain.cpp:304-354`) is: exact quantized-edge-set buckets, then a tolerant fallback
  that requires **`edge_count` equality** (`:335`) **and bbox equality within band** (`:339-343`)
  and a bipartite match of *every* signature (`:344-352`). The planar shortcut is at `:371-378`,
  the zero-thickness guard at `:362`, the interior probe at `:502-530` using
  `SD_PAR_T = 0.43213918` (`:20`, `:510`), the band at `:434-443`.
* `src/brep_commonblock.h/.cpp` — the partial-coincidence machinery: `cb_split_chain`
  (`:58`), `cb_tolerance` (ComputeToleranceOfCB port), `cb_perform_common_blocks`,
  `cb_partition_face` (`:256`), `cb_select_region` (`:253`).
* `main_10.cpp` (73/73) exercises all of it — `run_partial()` at `main_10.cpp:476-666` has 14
  partial-coincidence cells (P1-P14) including a half-face split, an inner region, a cylinder
  lateral strip, a box face on a cylinder cap, tolerance and grouping.
* Pipeline wiring:
  * v1 (`brep.cpp`): **HOOK 1-PRE** at `:8631-8645` and **HOOK 1** at `:8924-8942` both run
    `SameDomain::detect()` and `fprintf` the result. Neither mutates anything. Both are behind
    `SESSION_SD`.
  * v1's actual coincidence handler is the legacy **ON-imprint** at `brep.cpp:8708-8896`, whose own
    comment (`:8730-8748`) lists four ways it is unsound.
  * v1's ON *state* is decided at `brep.cpp:9564-9599`: one sample from `face_sample`, a straddle
    test at `on_eps = diag·2e-3` (`:9456`), plus a scale-invariance retry at `on_eps/16`
    (`:9579-9589`).
  * v2 (`v2/brep_v2_boolean.cpp`): `v2sol_sd_surface_pairs` (`:207-227`) runs `SameDomain` on the
    **original** faces and `v2sol_build_sections` (`:159-204`) uses the result as a **fence** —
    coincident surface pairs are skipped entirely, no section. That is correct as far as it goes.

### A.3 The gap — concrete delta

1. **Partial-coincidence candidacy.** `SameDomain` can structurally never see a partial overlap:
   equal edge sets, equal edge counts and equal bboxes are all violated. Needed: a second detector
   that pairs faces by **surface identity + areal overlap**, not by boundary identity —
   `same_surface(SA, SB, tol)` (v2 already has `v2sol_same_surface`,
   `v2/brep_v2_boolean.cpp:141-155`, but it demands identical domains and CV structure) followed by
   `cb_partition_face` on both sides. `cb_partition_face` already returns the right thing
   (`brep_commonblock.cpp:256`); nothing calls it from the pipeline.

2. **An EF imprint channel into the splitter.** Every OCCT partial-coincidence case is resolved by
   putting the *other* operand's edges into `FaceInfo.In` of a face and letting `BuildSplitFaces`
   cut with them. We have the interference stage (v2 `main_15` 29/29, 297 piercings) but no path
   from an EF/edge-on-face result to `split_with`'s cut-pcurve list. Delta: for every
   `(edge e of B, face f of A)` whose distance stays within band over the whole of `e`, pull `e`
   back to `f`'s UV and append it to `f`'s cut set — i.e. exactly what `extra_cuts`
   (`brep.cpp:4637-4646`) already accepts, but sourced from a *measured* EF relation instead of
   from the unsound rim-only ON test.

3. **Seam handling inside a coincident region.** `cylsame` shows the shared strip's boundary
   includes a *partial* seam-line CommonBlock (`pbs=6:2:4|19:0:2`) created by VE at the partner's
   seam vertex. A partial-coincidence implementation that only imprints rim circles will leave the
   strip's seam un-split and the face un-closable. Delta: run the VE/EE partial-range pass on seam
   edges before the region split.

4. **A face-level SD fusion step.** After splitting, the two operands' coincident sub-faces must
   become **one** entity, then be selected once by the op-table. v2 has
   `SameDomain`-based image fusion at `v2/brep_v2_boolean.cpp:1112-1120` and `:1346-1360`
   (post-split, "fuse same-domain face images to ONE arena face index"); it will now fire once
   candidacy (delta 1) is fixed, because after the split the two halves *do* have identical edge
   sets. This is the smallest change with the largest reach.

5. **`extra_cuts` must not be fed by the rim-only test.** `brep.cpp:8730-8748` documents the
   corrected predicate (`SESSION_ONIMP_FIX`: areal, fail-closed, projection must land inside a real
   face, `|cos| ≥ 0.9`). It is **off by default**. Turn it on and delete the legacy branch, once
   deltas 1-2 supply the imprints it used to fake.

### A.4 Measured impact

**cylsame — partial coincidence on a curved face (new case).**

| op | v2 (`main_20`) | OCCT | truth | v2 error | v2 `valid`/`naked`/`nsolid` | OCCT |
|---|---|---|---|---|---|---|
| fuse | 20.943951 | 18.8495559 | 6π | **+2.0943951 (+11.11 %)** | 0 / 5 / 0 | 1 / 0 / 1 |
| common | 4.1887902 | 6.28318531 | 2π | **−2.0943951 (−33.33 %)** | 0 / 5 / 0 | 1 / 0 / 1 |
| cut | 8.37758041 | 6.28318531 | 2π | **+2.0943951 (+33.33 %)** | 0 / 5 / 0 | 1 / 0 / 1 |

All three errors are the **same number, 2π/3**, with the sign pattern of a single misplaced
boundary: `fuse = A+B−common` and `cut = A−common` are both exactly satisfied by
`common = 4π/3`. So the failure is one thing — the coincident strip's extent is computed as
height **4/3 instead of 2** — not accumulated noise. Our section stage matches OCCT exactly here
(`seccurves=2`, `fftangent=1`, `secpb=0` on both sides); the loss is entirely downstream of it.

**Full and partial *planar* coincidence pass on volume, but by shattering.**

| case | v2 vol | OCCT vol | v2 `dsshapes` | OCCT `dsshapes` | v2 `secpb` | OCCT `secpb` | v2 `cb` | OCCT `cb` |
|---|---|---|---|---|---|---|---|---|
| box_box_touch_fuse | 128 ✓ | 128 | **1350** | 84 | **512** | 0 | **256** | 4 |
| box_box_touch_common | empty ✓ | empty | 1350 | 84 | 512 | 0 | 256 | 4 |
| box_box_half_fuse | 96 ✓ | 96 | **964** | 72 | **384** | 0 | **128** | 4 |

Right answer, 13-16× the entities, and 384-512 section pave blocks where OCCT needs **none**. That
is not a correctness failure today, but it is the same machinery that produces D1 on a curved
face, and it is the reason a real assembly with a dozen flush contacts is unaffordable.

**Historical, from the code's own measured notes** (`brep.cpp:8730-8748`, `:8919-8921`):

* legacy ON-imprint fires **20/20 on A-op-A**, all spurious, re-shattering the whole boundary;
* on `y30` (no coincidence at all, `SD groups=0`) disabling it alone moved cut from
  **23.9619 → 47.6964** against a **46.9596** reference — the legacy predicate was costing ~49 %;
* `x20 cut` deletes **12 of A's 36 fragments** whose probe points sit at distance **0.00e+00**
  from B's boundary while our ON-detector reports `on=0` for every one of them.

**Op-table: no gap.** `sd_select_face` reproduces every ON row observed:

| case | wall relation | op | OCCT | `sd_select_face` |
|---|---|---|---|---|
| box_box_touch_fuse | ON-opposite | Fuse | both dropped | Drop / Drop ✓ |
| box_box_touch_common | ON-opposite | Common | both dropped, empty result | Drop / Drop ✓ |
| box_box_touch_cut | ON-opposite | Cut | kept once, image shared | Keep(A) / Drop(B) ✓ |
| cylsame_fuse | ON-same | Fuse | kept once, image shared | Keep(A) / Drop(B) ✓ |
| cylsame_common | ON-same | Common | kept once | Keep(A) / Drop(B) ✓ |
| cylsame_cut | ON-same | Cut | both dropped | Drop / Drop ✓ |

---

## B. TANGENCY

### B.1 What OCCT does — measured

**B.1.1 A point tangency is a section POINT plus a V-something interference, never a curve.**

`sphtang_box_cut` (unit sphere at z=3 resting on the top face of a 4³ box; contact at (0,0,2),
which is the sphere's own south-pole vertex):

```
IVF 7↔41 u=2 v=2 new=-1                       <- the EXISTING pole vertex pierces the box face
IFF 2↔41 tangent=0 ncurves=0 npoints=1        <- tangent FALSE at a genuine tangency
FFP f1=2 f2=41 p=0 xyz=0,0,2 v=-1             <- FF point, NOT promoted to a DS vertex
FI  f=41 vIn=7                                <- alone vertex of the box's top face
IMGFACE a=1 f=6 split=1 nimg=1 nkept=0        <- the box face IS rebuilt around the alone vertex
cut:    nsolid=1 nface=1 vol=4.1887902 (= the whole sphere) naked=0 valid=1
common: EMPTY compound, valid=1
fuse:   nsolid=2 vol=68.1887902 naked=0 valid=1
```

`sphsph_tang_fuse` (two unit spheres touching at (1,0,0), which is *not* a vertex of either):

```
IEF 6↔11 ctype=VERTEX r1=6.28263766:6.28373296 new=18    <- A's SEAM edge x B's face -> NEW vertex
DSVERT i=18 p=1,0,0 tol=2.00001e-07 new=1                <- tol = 1e-7 + 1e-7 + 1e-12
IFF 2↔11 tangent=0 ncurves=0 npoints=1
FFP xyz=1,0,0 v=-1
RES nsolid=2 nface=2 nvert=5 ndegen=4 vol=8.37758041 naked=0 valid=1
```

**B.1.2 The contact set has non-zero PARAMETER width, and the width obeys the square-root law.**
This is the single most useful measurement in this file. A tangency is a double root, so the
distance grows quadratically and a tolerance band of `τ` is entered over a half-width
`s = sqrt(2τ / |κ_rel|)` in arc length, where `κ_rel` is the relative normal curvature of the two
entities at the contact.

| record | measured half-width (arc) | `κ_rel` | `sqrt(2·3e-7/κ_rel)` | agreement |
|---|---|---|---|---|
| `sphsph_tang` `IEF 6↔11 r1=6.28263766:6.28373296` | 5.4765e-04 | 2 (external, 1+1) | 5.4772e-04 | 4 s.f. |
| `cyltang_off` `IEE 4↔17 r1=3.14104314:3.14214216` (r=2 circle) | 1.09902e-03 | 0.5 (internal, 1−½) | 1.09545e-03 | 0.3 % |
| `cyltang_off` `IEE 4↔17 r2=3.14049393:3.14269138` (r=1 circle) | 1.098725e-03 | 0.5 | 1.09545e-03 | 0.3 % |

with `τ = tolA + tolB + Precision::Confusion = 3e-7`. **At unit scale the contact region is ~3.5
orders of magnitude wider than the tolerance.** Any predicate that tests "is this contact a point?"
by comparing a *parameter* interval against `τ` will be wrong by that factor. This is why a
tangency must be a named outcome and not a small number.

**B.1.3 A line tangency produces a section curve whose RANGE is wrong, corrected later by paves.**
`cyltang_off_cut` (r=2 and r=1 cylinders internally tangent along x=−2, tangency *not* on either
seam):

```
afterFF: SEC f1=2 f2=15 type=Line t0=-6e-07 t1=6.0000006 len=6.0000012 npb=0
final:   SEC ... same geometry ... npb=1  edge=36
         FI f=2  set=Sc orig=-1 t0=0 t1=6 edge=36
         FI f=15 set=Sc orig=-1 t0=0 t1=6 edge=36
```

The SSI overshoots the true `[0,6]` by **6e-7 = 6·Confusion at each end**, the geometry is *never
refitted*, and the honest range is restored by the paves supplied by
`IEE 4↔17 new=26` / `IEE 8↔21 new=27` and by `IVF 18↔9`, `IVF 20↔11`. Result:

```
RESEDGE i=3 curve=Line len=6 nface=4        <- FOUR faces on the tangent edge
RES nsolid=1 nface=6 vol=56.5486678 (=18π) naked=0 valid=1
```

**A line tangency legitimately yields a 4-valent edge and OCCT calls the result valid.** Any
non-manifold gate we apply must permit `nface=4` at a tangency.

**B.1.4 When the tangency happens to lie on both operands' seams, OCCT never needs the section
curve at all.** `cyltang_in_*` (same radii, tangency at x=+2 = both seams): `seccurves=7` and
**`secpb=0` — every one of the seven curves is discarded** — while the tangent generatrix is
carried by `IEE 6↔19 ctype=EDGE r1=0:6 r2=0:6` → `CB id=0 pbs=6:0:6|19:0:6`. Results still exact
(common = 6π, cut = 18π, both `naked=0 valid=1`). The lesson is not "seams are lucky"; it is that
**OCCT has two independent routes to the same contact geometry (FF-section and EE/EF-coincidence)
and needs only one to succeed.** We have one.

**B.1.5 Exact surface-tangency at a pole absorbs into vertex tolerance in place.**
`sph_cyl_roty23578` (already in `occt_trace_findings.md` §Q1): three section curves; DS vertex 5
goes 1e-07 → **2.14903822e-06**, DS vertex 7 goes 1e-07 → **1.70243165e-05**, both keeping
`sd=-1`; vertex tol = curve tol + exactly 1.0e-12. This is the *opposite* of the coincidence rule
(A.1.5), where coincident vertices are replaced by new ones and never widened.

**B.1.6 Re-intersection.** `BOPAlgo_PaveFiller::RepeatIntersection`
(`BOPAlgo_PaveFiller.cxx:359-421`, called at `:291`) re-runs VV → VE → VF, each followed by
`UpdatePaveBlocksWithSDVertices`, **iff some source vertex's tolerance was increased**
(`myIncreasedSS`, `:373-390`); it then runs `ForceInterfEE` (`:298`) and `ForceInterfEF` (`:305`)
before `PerformFF`. Measured to fire in **three** corpus cases: `sph_cyl_roty23578_*` (exact pole
tangency, widening visible in the dump), `cylsame_*` and `sphtang_box_*` (widening not visible at
`afterFF`, so the trigger there is not fully observable through the public API). It does **not**
fire in `box_box_half_fuse` even though EF created two CommonBlocks there.

**B.1.7 Apex-to-apex contact.** `coneapex_*` (two cones nose to nose): `VV=1`, `sd=2`,
`newverts=1`, `fftangent=1`, **`seccurves=0` and `secpoints=0`** — no FF output whatsoever; the
contact is carried purely by the VV fusion of the two apex vertices. `fuse` → 2 solids,
vol 41.887902 = 2·(π·4·5/3); `common` → empty. So `tangent=1 ncurves=0 npoints=0` is *also* the
signature of a point contact between two cones — indistinguishable from full coincidence by that
flag alone. Confirmation that the flag is unusable (A.1.9).

### B.2 What we do

* `V2FFStatus::Tangent` exists (`v2/brep_v2_section.h:59-66`) with the doctrine "a tangency is
  NEVER reported as Ok with 0 curves", and `v2tangent_faces`
  (`v2/brep_v2_section.cpp:859-886`) implements a surface-level test. But it is only reached
  **after the SSI has already returned nothing** (`v2/brep_v2_section.cpp:1159-1163`), and the
  status is counted (`m_stats.tangent`) and otherwise **discarded** — no consumer reads it.
* The marcher's own contract, `intersection.h:486-487`: *"Marching terminates at tangencies
  (n_a parallel n_b); tangential intersections are unsupported."* `intersection.cpp:1741`
  halves the step "near-tangency: Newton diverges at full step" — a recovery, not a classification.
* There is no vertex-tolerance model at all: `SameDomain::max_edge_tolerance`
  (`brep_samedomain.cpp:425-432`) returns **0** with the comment "this kernel stores no per-entity
  tolerances (phase P5)". So B.1.5's tolerance-inflation mechanism has nowhere to write.
* There is no section-POINT type. `V2FFStatus` has no `Point` member; `secpoints` is always 0 in
  every v2 trace taken here.
* `brep.cpp:9579-9589` has a scale-invariance retry on the ON straddle (probe at `on_eps/16` must
  agree) — the only place in the kernel where a degenerate verdict is stress-tested.

### B.3 The gap — concrete delta

1. **A typed `Tangent` outcome that is produced *when the SSI succeeds too*, and that is
   consumed.** Today a tangency reaches the splitter as either (a) nothing, or (b) a section
   curve with a wrong range and a doubled/spurious crossing. Delta: after every successful SSI,
   test each returned branch for `|n1 · n2| > 1 − ε` along its whole length; classify the branch
   as `Transversal | TangentCurve | TangentPoint` and route:
   * `TangentPoint` → emit a **section point**, not a curve; create/reuse a vertex; do **not**
     split any face by it (OCCT: `FFP … v=-1`, face rebuilt only because of the alone vertex).
   * `TangentCurve` → keep the curve, but take its **range from paves only**, never from the SSI
     (B.1.3: OCCT's own range was off by 6e-7 on a length-6 line and the paves fixed it).

2. **A contact-width predicate based on the square-root law.** Implement
   `tangency_halfwidth(κ_rel, τ) = sqrt(2τ / |κ_rel|)` (B.1.2, agrees with OCCT to 0.3-0.01 %)
   and use it as the merge radius when deciding whether two nearby roots are **one** tangency or
   **two** transversal crossings. Today "a tangency is a double root that perturbation splits into
   two" is handled nowhere; a linear `τ`-test is off by ~3.5 decades.

3. **Vertex tolerance inflation in place.** Add a per-vertex tolerance to the BRep and implement
   OCCT's two measured rules: (i) tangency → **widen the existing vertex in place**, new tol =
   contact tol + 1.0e-12, keep `sd=-1`; (ii) coincidence → **replace by a new fused vertex**, do
   not widen. Without (i) there is nothing to absorb a tangency into and it must become geometry.

4. **A re-intersection pass.** Once (3) exists, re-run VV/VE/VF for every vertex whose tolerance
   grew, then force EE/EF (`BOPAlgo_PaveFiller.cxx:291-309`). Measured necessary in 3 of 55 traced
   cases — always the degenerate ones.

5. **Permit `nface = 4` at a tangency in the validity gate.** B.1.3 shows OCCT ships a 4-valent
   edge with `valid=1 naked=0`. Our `nonmanifold` counter must special-case an edge whose four
   trims are two co-tangent pairs.

### B.4 Measured impact

| case | v2 vol | OCCT vol | truth | v2 err | v2 `valid`/`naked` | OCCT |
|---|---|---|---|---|---|---|
| `sphtang_box_fuse` (point tangency, existing vertex) | 68.1887902 | 68.1887902 | 68.1887902 | **0** | 1 / 0, `nsolid=2` | 1 / 0, `nsolid=2` |
| `cyltang_off_common` (line tangency off-seam) | 18.8495559 | 18.8495559 | 6π | **0** | 1 / 0 | 1 / 0 |
| `cyltang_off_cut` (line tangency off-seam) | **56.5448431** | 56.5486678 | 18π | **−3.8247e-03 (−6.76e-05 rel)** | **0 / 6**, `nsolid=0` | 1 / 0, `nsolid=1` |
| `sph_cyl` exact pole tangency, common | **15.0591515** | 15.0617954 | 15.0617954 | **−2.6439e-03 (−1.76e-04 rel)** | 1 / 0 | 1 / 0 |

Point tangency: **no gap** — we match OCCT bit-for-bit including the two-solid convention.
Line tangency: **cut fails, common passes** — the tangent generatrix is representable but the
4-valent edge is not assembled, leaving 6 naked edges. Exact surface tangency: see §C.

`coneapex`, `sphsph_tang`, `cyltang_in`, `cyltang_out`: **no measured impact yet** — not yet run
through `main_20`.

---

## C. NEAR-DEGENERATE STABILITY

### C.1 What OCCT does, and where it is worse than us

The sphere r=2.5 × cylinder r=1 sweep, `common`, converged truth **15.0617954** (tilt-independent
by symmetry). Our numbers are from `build_v2diff/main_20`, run for this analysis.

| tilt | OCCT vol | OCCT err | v2 vol | v2 err | v2 `valid` | v2 `ndegen` | OCCT `ndegen` |
|---|---|---|---|---|---|---|---|
| 0° | 15.0617955 | +1e-07 | 15.0617955 | **+1e-07** | 1 | 2 | 2 |
| 20° | 15.0618286 | +3.3e-05 | 15.0617947 | **−7e-07** | **0** | **0** | 2 |
| 23° | 15.0649961 | **+3.2007e-03** | 15.0617505 | **−4.49e-05** | **0** | **0** | 2 |
| **23.578178478°** (exact) | 15.0617954 | **0** | **15.0591515** | **−2.6439e-03** | 1 | **0** | 3 |
| 24° | 15.0617954 | 0 | 15.0618552 | +6.0e-05 | **0** | 0 | 0 |
| 30° | 15.0617954 | 0 | 15.061796 | +6e-07 | **0** | 0 | 0 |
| 45° | 15.0617954 | 0 | 15.0617955 | +1e-07 | **0** | 0 | 0 |

Two facts, both measured:

* **At 23° (0.578° below tangency) we are 71× more accurate than OCCT** (4.49e-05 vs 3.2007e-03).
  (The project brief records "7×"; the current build measures 71.3×. Flagging the discrepancy
  rather than reconciling it — I did not run the configuration that produced the 7× figure.)
* **At the exact tangency we are at our worst and OCCT is exact.** Our error there is 59× our 23°
  error. The neighbourhood is therefore *not* monotone for either kernel, and neither kernel's
  behaviour can be predicted from the other's.

The error at the exact tangency is not topological noise. Face areas:

| face | v2 | OCCT | relative deficit |
|---|---|---|---|
| north lobe 1 | 1.63842824 | 1.63922141 | **4.8387e-04** |
| north lobe 2 | 1.63842824 | 1.63922141 | **4.8387e-04** |
| south cap | 3.27685647 | 3.2784428 | **4.8386e-04** |
| cylinder wall | 28.7931723 | 28.7931723 | **0** |

**Every spherical piece is short by the identical relative amount and the cylindrical piece is
exact.** That is a boundary-geometry deficit confined to the operand carrying the pole, not a
classification error. It correlates exactly with the missing degenerate edges: v2 has
`nedge=5 ndegen=0 nvert=6`, OCCT has `nedge=8 ndegen=3 nvert=3`. With no degenerate pole edge the
wire closes across the pole rather than along the `v = ±π/2` line, and each face loses the same
lune.

The same defect is visible at 20-23° without a volume cost: `ndegen=0` where OCCT has 2, `nedge=5`
vs 7 — and our own `is_solid()` then returns 0 for a shell with `naked=0`,
`closure=4.78400647e-09`, `nonmanifold=0`. Five of seven sweep cells report `valid=0` on a
correct-to-7-digits volume.

### C.2 What a correct kernel should do near a degeneracy

Parity is the wrong target: OCCT is 3.2e-03 wrong at 23° and exact at 23.578°. The targets are:

1. **Continuity of the answer, not of the algorithm.** The exact result here is tilt-independent.
   A correct kernel's *volume* must be continuous through the degeneracy even though its
   *topology* is not (2 section curves below, 3 at and above). Our 23°→23.578° jump of 2.6e-03 is
   a discontinuity of the answer and is therefore a defect regardless of OCCT.
2. **The degeneracy must be entered deliberately, not stumbled into.** Detect it by the
   square-root law (B.3.2): if two roots are closer than `sqrt(2τ/|κ_rel|)` they are one tangency;
   snap to the exact configuration and take the tangency branch. Never let a perturbation decide.
3. **Never perturb the input.** A "symbolic perturbation" that splits a tangency into two
   crossings changes the answer by O(1) in topology and cannot be undone downstream.

### C.3 Oracle-free invariants that detect a wrong answer at a degeneracy

Every one of these is computable from our own result and would have caught at least one failure
measured here:

| invariant | catches |
|---|---|
| `vol(A∩B) + vol(A\B) == vol(A)` and `vol(A∪B) == vol(A)+vol(B)−vol(A∩B)` | **cylsame**: our three ops satisfy this *exactly* with a common of 4π/3 — so it does **not** catch D1 by itself. It does prove the failure is a single misplaced boundary, which is diagnostically decisive. |
| `is_solid()` / `naked == 0` | **cylsame** (5 naked, all 3 ops), **cyltang_off_cut** (6 naked). Catches D1 and D4. |
| **Symmetry/invariance sweep**: run the same pair under N rigid motions; every volume is invariant | the entire §C table — the 2.6e-03 spike at 23.578° is invisible without an oracle *unless* you sweep the parameter. This is the only invariant that catches C. |
| **Degenerate-edge conservation**: a pole/apex contained in a kept region must still carry a degenerate edge | **all 5 `valid=0` sweep cells**, and the exact-tangency volume error. |
| **Tangency contact width**: a contact whose measured parameter width exceeds `sqrt(2τ/κ)` by more than ~2× is not a tangency; below it, two "crossings" are one | not yet wired; would catch a marcher that emits two piercings for one tangency. |
| **Area sanity per face**: analytic area of the trimmed patch vs the integrated boundary | the exact-tangency case, where all three spherical faces are short by the *same* 4.8387e-04 — a uniform relative deficit is a signature no random error produces. |

### C.4 Measured impact

D5 (degenerate-edge destruction): **5 of 7** cells in the sweep report `valid=0` with correct
volume; **1 of 7** (the exact tangency) has a **−1.76e-04 relative volume error** and a
**−8.97e-05 relative area error** traceable to the same cause.

---

## D. THE SYMMETRIC-PROBE TRAP

### D.1 The rule

**Never probe at a symmetric parameter.** A midpoint, a domain centre, an equal-weight centroid or
a uniform grid fraction lands *exactly* on a symmetry plane, a seam, a pole meridian or a shared
edge in symmetric geometry — and there the quantity being probed is **undefined**, not merely
small: distances tie at 0, normals come out perpendicular, an even-odd test straddles. OCCT
encodes the rule as a constant: `PAR_T = 0.43213918` (`IntTools_Tools.cxx:254-259`), with a
*second* asymmetric fallback at `1 − PAR_T = 0.56786082` (`BOPTools_AlgoTools3D.cxx:932`). Our
whole corpus is symmetric — boxes centred on the origin, coaxial cylinders, spheres at the origin,
A-op-A — so the trap is not hypothetical here. `brep.cpp:9299-9306` records three separate bugs
in this project traced to it.

Two constants are needed, not one: a square face has a **diagonal** symmetry that a single `PAR_T`
applied to both `u` and `v` does not break.

### D.2 Where it still applies in our code

| site | probe | status |
|---|---|---|
| `brep.cpp:9312-9313` `face_sample` fallback | `u0 + (u1−u0)·0.5`, `v0 + (v1−v0)·0.5` | **default symmetric** — the fix (`PAR_T`, `PAR_T2 = 0.61803399`) exists but is behind `SESSION_PROBE_FIX` |
| `brep.cpp:9375-9377` `face_sample` barycentric | `(1/3, 1/3, 1/3)` of the largest CDT triangle | **default symmetric** — same gate |
| `brep.cpp:9433-9435` `face_samples` | `(1/3, 1/3, 1/3)` for all K samples | **default symmetric** — same gate |
| `brep.cpp:9569-9589` ON straddle (`on_eps` at `:9456`) | one sample from `face_sample`, band `diag·2e-3` | **single probe, default-symmetric point, band 4 decades looser than OCCT's `tolF1+tolF2+Confusion`** |
| `brep_samedomain.cpp:518-528` `point_in_face` grid fallback | `iu/nn` for `nn ∈ {8,16,32}` — contains exactly 0.5, 0.25, 0.75 | symmetric; only reached when the `SD_PAR_T` probe falls outside the trims, but then it is the whole answer |
| `brep_samedomain.cpp:376-377` planar-shortcut orientation | `normal_at(0.5, 0.5)` on both surfaces | symmetric, **low risk today** (the branch is gated on `is_planar`, where the normal is constant) but wrong by construction |
| `v2/brep_v2_section.cpp:861-875` `v2tangent_faces` | 13×13 grid `i/12`, `k/12` — includes the exact domain centre (6/12) and all four corners | symmetric; on a periodic face the corners are seam points |
| `v2/brep_v2_boolean.cpp:148-155` `v2sol_same_surface` | 6×6 grid `i/5` — corners and edges of the domain, no centre | symmetric at the seam; and the predicate additionally demands *identical domains and CV counts*, so a coincident plane authored differently never matches |

Clean sites, for contrast: `brep_samedomain.cpp:509-510` (`SD_PAR_T` for both `u` and `v` — but
the *same* constant for both, so diagonal symmetry survives) and
`brep_samedomain.cpp:546-552` `samples_in_face` (irrational offsets `+0.13212`, `+0.31121`,
different per axis — the only fully correct probe in the kernel).

### D.3 The gap — concrete delta

1. **Delete the `SESSION_PROBE_FIX` gate** and make the asymmetric weights the only behaviour
   (`brep.cpp:9312`, `:9375`, `:9433`). Two *different* constants per axis, as
   `samples_in_face` already does.
2. **Replace the single ON probe with `samples_in_face` + the areal rule** already implemented in
   `sd_classify_samples` (`brep_samedomain.cpp:611-678`): ON requires **every** probe within band.
3. **Replace `on_eps = diag·2e-3` with `tolA + tolB + max(fuzz, Confusion)`** — currently the ON
   band is roughly `1e-2` on a unit-scale model against OCCT's `3e-7`, i.e. it will call
   "coincident" anything within 1 % of the model diagonal.
4. **Fix the grid fallbacks** (`brep_samedomain.cpp:521`, `v2/brep_v2_section.cpp:863-866`) to use
   `(i + PAR_T)/n` rather than `i/n`.

### D.4 Measured impact

The corrected predicate's own recorded measurement (`brep.cpp:8783-8792`): with the areal test
enabled but the grid symmetric, **all 27 y30 candidates reported `probes=0`** — the areal test
was silently behaving as a wholesale disable for every thin fragment, and the thin-fragment
fallback had to be added to keep it from rejecting by artifact. That is a symmetric-grid failure,
not a geometric one.

Beyond that: **no measured impact yet** for items D.3.1 and D.3.4 in isolation — no one has run
the battery with `SESSION_PROBE_FIX=1` and reported the delta. That is the cheapest measurement in
this whole document and should be taken first (see E.0).

---

## E. IMPLEMENTATION ORDER

Smallest shippable increment first; every step has an acceptance test that is a *number*, not an
opinion.

### E.0 — MEASURE FIRST (hours, no code)

Run the 63-cell oriented battery and the sph×cyl sweep twice: default, and with
`SESSION_PROBE_FIX=1 SESSION_ONIMP_FIX=1`. Both fixes are already implemented and gated off.

* **Acceptance:** a table of per-cell deltas. If the fixes are net-positive, delete the gates and
  the legacy branches in the same commit. If any cell regresses, the regression is a *finding*
  and belongs in this file.
* **Why first:** it is the only step whose cost is zero and whose information changes the order of
  everything below.

### E.1 — Degenerate-edge conservation (D5) — highest measured value per line

A pole/apex whose region survives must keep its degenerate edge through the split. Today the edge
survives only at tilt 0.

* **Acceptance A:** `sph_cyl` common at 20°, 23°, 30°, 45° → `ndegen == 2` (0 at ≥24° where the
  pole is outside the region — match OCCT's `res_degen` column), `is_solid() == true`.
  Currently 0/4.
* **Acceptance B:** `sph_cyl` common at 23.578178478201835° → `ndegen == 3`, `nvert == 3`,
  volume within **1e-6** of 15.0617954 (currently 15.0591515, error 2.6439e-03), total area
  within 1e-6 of 35.3500579 (currently 35.3468852).
* **Acceptance C:** `sph_box_cut` → `ndegen == 2`; `sph_sph_p2_cut` → `ndegen == 3`
  (`traces/_summary.txt`).
* Guard: a degenerate edge has `nface == 1` and must be excluded from every naked-edge count
  (`occt_trace_findings.md` §Q4).

### E.2 — Tangency as a typed, consumed outcome (D4)

Classify each SSI branch as `Transversal | TangentCurve | TangentPoint` **after** a successful
SSI, not only after an empty one. Take a `TangentCurve`'s range from paves only. Emit a section
**point** for `TangentPoint` and do not split by it.

* **Acceptance A:** `cyltang_off_cut` → volume within 1e-9 of 18π = 56.5486678
  (currently 56.5448431, err −3.8247e-03), `naked == 0`, `is_solid() == true`
  (currently 6 naked, `nsolid=0`), and the tangent generatrix edge present with **4** trims.
* **Acceptance B:** the validity gate accepts that 4-valent edge (`nonmanifold == 0`).
* **Acceptance C:** no regression — `cyltang_off_common` stays at exactly 18.8495559,
  `sphtang_box_fuse` stays at exactly 68.1887902 with `nsolid == 2`.
* **Acceptance D (unit):** `tangency_halfwidth(κ_rel, τ) = sqrt(2τ/|κ_rel|)` reproduces the three
  measured OCCT contact widths of B.1.2 to within 1 %:
  `(κ=2, τ=3e-7) → 5.477e-4` vs measured 5.4765e-4; `(κ=0.5, τ=3e-7) → 1.0954e-3` vs
  measured 1.09902e-3 and 1.098725e-3.

### E.3 — EF imprint channel into the face splitter (D2)

For every `(edge e of B, face f of A)` pair whose distance to `f` stays within
`tol_e + tol_f + max(fuzz, Confusion)` over the whole of `e`, pull `e` back onto `f`'s UV and
append it to `f`'s cut set. Feed it through the existing `extra_cuts` parameter
(`brep.cpp:3267`, consumed at `:4641-4646`) rather than through the legacy ON-imprint.

* **Acceptance A:** on `cylsame`, A's lateral face splits at exactly `z = 0` and B's at exactly
  `z = 2` — measure the split circle's z to 1e-9, and the two sub-face areas to
  `2π·1·2 = 12.5663706` each (OCCT `RESFACE i=1/i=2/i=3` areas).
* **Acceptance B:** on `cyltang_off_cut`, A's top disc becomes an annulus:
  `nwire == 2`, area `== 3π = 9.42477796` (OCCT `RESFACE i=3`).
* **Acceptance C:** on `box_box_half_fuse` the imprint replaces the current shattering:
  `dsshapes` drops from **964** toward OCCT's 72; volume stays exactly 96.
* **Acceptance D:** the channel is **silent** on transverse cases — `y30` cut must not move
  (this is precisely where the legacy path cost 49 %).

### E.4 — Partial-coincidence candidacy + region split (D1)

Add a surface-identity + areal-overlap detector alongside the edge-set detector, and drive
`cb_partition_face` from it. Then let the existing post-split SD fusion
(`v2/brep_v2_boolean.cpp:1346-1360`) merge the coincident sub-faces, and select once with
`sd_select_face`.

* **Acceptance A:** `cylsame` all three ops exact — fuse **18.8495559**, common **6.28318531**,
  cut **6.28318531** (currently 20.943951 / 4.1887902 / 8.37758041); `naked == 0`,
  `nsolid == 1`, `nface == 5 / 3 / 3` (OCCT).
* **Acceptance B:** the shared strip is **one** face claimed by both operands' image maps
  (OCCT `IMGFACE a=0 f=1 out=1,2` and `a=1 f=1 out=1,3`).
* **Acceptance C:** partial coincidence on the *seam*: the seam edge of the shared strip is one
  edge with a partial-range common block (OCCT `CB id=1 pbs=6:2:4|19:0:2`).
* **Acceptance D:** no regression on full coincidence — `box_box_touch_{fuse,common,cut}` stay at
  128 / empty / 64.
* **Acceptance E (unit, `main_10`):** a new cell that *builds a BRep* for the `cylsame`
  configuration. Every existing partial-coincidence cell (P1-P14) is predicate-level only; not one
  of them produces a solid, which is exactly why 73/73 coexists with a 33 % volume error.

### E.5 — Per-entity tolerance + inflation + re-intersection (B.3.3-4)

Add a tolerance to vertices and edges. Implement the two measured rules: tangency widens an
existing vertex **in place** to `contact_tol + 1.0e-12` keeping `sd=-1`; coincidence **replaces**
by a new fused vertex without widening. Then re-run VV/VE/VF for every widened vertex and force
EE/EF.

* **Acceptance A:** `sph_cyl_roty23578` reproduces OCCT's vertex tolerances:
  north pole **2.14903822e-06**, south pole **1.70243165e-05**, both `sd == -1`; and the section
  curve tolerances **2.14903722e-06** / **1.70243155e-05** (differences exactly 1.0e-12).
* **Acceptance B:** `box_box_touch_fuse` produces **4 new** fused vertices with tol unchanged at
  1e-07 and `sd` set on all 8 sources — i.e. the *opposite* rule, correctly distinguished.
* **Acceptance C:** the re-intersection pass fires on exactly the cases OCCT fires it on among the
  traced set (`sph_cyl_roty23578`, `cylsame`, `sphtang_box`) and on none of the others.
* **Acceptance D:** `SameDomain::sd_band` stops returning a 0 edge-tolerance term
  (`brep_samedomain.cpp:425-432`) and the SD band becomes the real
  `max(tolF1, maxTolE(F1)) + max(tolF2, maxTolE(F1)) + max(fuzz, Confusion)`.

### E.6 — Wire the SD hooks to selection (D6)

Promote `HOOK 1` (`brep.cpp:8924-8942`) from `fprintf` to a real stage: build the SD classes, fuse
the coincident face images, and apply `sd_select_face` / `cb_select_region` per region. Delete the
legacy ON-imprint (`brep.cpp:8708-8896`) once E.3 supplies its imprints.

* **Acceptance A:** A-op-A for all three ops on the chairs: fuse == A, common == A, cut == empty,
  with `SD groups` equal to the face count and **zero** section segments between coincident
  surfaces (currently 481 segments and 312 fragments — `brep.cpp:8624-8628`).
* **Acceptance B:** `x20 cut` — the 12 A-fragments at distance 0.00e+00 from B's boundary are
  classified ON, not deleted (`brep.cpp:8919-8921`).
* **Acceptance C:** the whole 63-cell battery does not regress; report the delta per cell.

---

## F. WHAT COULD NOT BE ESTABLISHED

* **The exact trigger of `RepeatIntersection` in two of three cases.** The source condition is
  unambiguous (`myIncreasedSS` non-empty, `BOPAlgo_PaveFiller.cxx:373-390`), and in
  `sph_cyl_roty23578` the widening is visible in the dump. In `sphtang_box` and `cylsame` no
  vertex tolerance above 1.00000001e-07 appears at `tag=afterFF`, so *which* vertex was widened
  is not observable through the public API. Do not infer.
* **Why OCCT emits a redundant section curve for some symmetric face pairs and not others**
  (A.1.10). Observed, harmless (`npb=0`), unexplained.
* **The mechanism behind `cylsame`'s exact 4/3-instead-of-2 strip height.** The arithmetic
  signature is unambiguous (all three ops off by exactly 2π/3, and `fuse = A+B−common`,
  `cut = A−common` both satisfied), but I did not instrument the split to find where 4/3 comes
  from. That is one `SESSION_SPLIT_DBG` run away.
* **The "7× better than OCCT at 23°" figure** in the project brief. The current
  `build_v2diff/main_20` measures **71.3×** (4.49e-05 vs 3.2007e-03). I did not identify the
  configuration that produced 7×.
* **`coneapex`, `sphsph_tang`, `cyltang_in`, `cyltang_out` through our kernel.** OCCT ground truth
  captured; our side not yet run. Their expected values are 41.887902 / empty, 8.37758041 / empty,
  56.5486678 / 18.8495559, and 37.6991118 with `nsolid=2`.
* **Whether the D.3 probe fixes change any battery cell.** Both are implemented and gated off; the
  measurement has never been taken (E.0).
