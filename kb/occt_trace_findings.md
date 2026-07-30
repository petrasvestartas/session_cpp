# OCCT pipeline trace — measured findings

Tool: `validation/occt_trace/` (`occt_trace.cpp` + `CMakeLists.txt` + `run_traces.sh`).
OCCT **8.0.0.rc2-a66b3fd6**, static install at
`validation/occt_oracle/build/deps/occt/install`, Release, `SetRunParallel(false)`,
`SetUseOBB(false)`, default fuzzy value.
Corpus: 38 traces, 1.3 MB, in `validation/occt_trace/traces/`; one-line-per-case digest in
`traces/_summary.txt`.

Everything below is **measured from the dumps**, not inferred from source. Where a stage
could not be observed through the public API it is stated explicitly.

---

## 0. How the tracer drives the pipeline (and what it cannot reach)

`BOPAlgo_PaveFiller::PerformVV/VE/VF/EE/EF/FF` and `Init` are `protected virtual`, so
`occt_trace` subclasses `BOPAlgo_PaveFiller` and overrides them (call base, then snapshot
the DS). That yields a real per-stage trace of the six intersection stages.

**Not reachable through the public API** (stated rather than guessed):

* `MakeSplitEdges`, `MakeBlocks`, `MakePCurves`, `ProcessDE`, `RemoveMicroEdges`,
  `RepeatIntersection`, `ForceInterfEE/EF` are `protected` but **non-virtual** — no
  snapshot can be taken between them. The DS between the end of `PerformFF` and the end of
  `Perform()` is observable only as one lump ("final" tag in the dumps).
* `BOPAlgo_Builder`'s face-image construction (`BuildSplitFaces`, `FillSameDomainFaces`,
  `FillImagesSolids`, `BuildSplitSolids`) is likewise protected non-virtual. Only the
  finished `Images()` / `Origins()` / `ShapesSD()` maps are public — which output face came
  from which input face IS observable; the intermediate wire assembly is not.
* `BOPDS_CommonBlock::Edge()` is declared non-const; reading it needs a `const_cast`.

**Trap worth knowing:** `BOPDS_DS::HasPaveBlocks(i)` and `HasFaceInfo(i)` are both just
`ShapeInfo(i).HasReference()`. The reference slot is *shared* between the pave-block pool
and the face-info pool and is discriminated only by shape type. Calling `PaveBlocks(i)` on a
FACE (or `FaceInfo(i)` on an EDGE) indexes the wrong pool and segfaults. Found the hard way.

### Measured stage order

Every one of the 38 cases produced exactly:

```
Init -> VV -> VE -> EE -> VF -> EF -> FF
```

**EE runs before VF**, not the VV/VE/VF/EE/EF/FF order the header's declaration sequence
suggests. Exactly one case in the corpus ran extra passes:

```
sph_cyl_roty23578_*  Init VV VE EE VF EF [VV VE VF] FF
```

i.e. the exact pole-tangency case is the only one that triggers the re-intersection pass
(`RepeatIntersection`) — because EF/VF enlarged a vertex tolerance and the VV/VE/VF results
had to be recomputed against the enlarged vertex.

---

## Q1. sphere r=2.5 × cylinder r=1 at the exact pole-tangency angle

Angle: `asin(1/2.5) = 23.578178478201835°` (cylinder axis through the sphere centre, tilted
about Y; at this angle the cylinder surface passes exactly through both sphere poles).

Sweep (`--op cut` and `--op common`, values identical for both since the same PaveFiller
feeds both):

| tilt Y | section curves | VV | VE | VF | EE | EF | new DS verts | common vol | error |
|---|---|---|---|---|---|---|---|---|---|
| 0°       | 2 | 0 | 0 | 0 | 2 | 0 | 2 | 15.0617955 | +1e-7 |
| 20°      | 2 | 0 | 0 | 0 | 2 | 0 | 2 | 15.0618286 | +3.3e-5 |
| 23°      | 2 | 0 | 0 | 0 | 2 | 0 | 2 | 15.0649961 | **+3.2e-3** |
| 23.578…° | **3** | 0 | **1** | **1** | 1 | 0 | **1** | 15.0617954 | 0 |
| 24°      | 3 | 0 | 0 | 0 | 1 | 2 | 3 | 15.0617954 | 0 |
| 25/30/45°| 3 | 0 | 0 | 0 | 1 | 2 | 3 | 15.0617954 | 0 |

(The true answer is tilt-independent by symmetry; the converged value is 15.0617954,
cut = 50.3880515, sum = 65.4498469 = 4/3·π·2.5³ exactly.)

**Answer: THREE section curves at exact tangency, and there is no typed tangency record.**

* `BOPDS_InterfFF::TangentFaces()` is **0** at the tangency. That flag is *not* about
  point/curve tangency at all: across the corpus it is set only for **fully coincident
  (same-domain) face pairs** — in `box_box_touch_fuse` all 5 `tangent=1` records have
  `ncurves=0` and correspond to the shared face and the 4 coplanar side faces. So OCCT has
  no "tangency" record in the sense we assumed.
* The 3 curves are: the **north circle split into two half-arcs**, plus the **south circle
  as a single closed curve**. Measured:
  * `c=0` Circle, t∈[0, π], len 3.14159265, p0 = **(0,0,2.5) = the north pole**,
    p1 = (1.83303028, 0, 1.7); sphere pcurve u∈[4.71238761, 2π], v∈[0.747762635, **π/2**].
    tol = 2.14903722e-06. Pave block v1 = **DS 5** (the sphere's own pole vertex, a *source*
    shape, not a new one), v2 = DS 22 (new).
  * `c=1` Circle, t∈[π, 2π], p0 = (1.83303028,0,1.7), p1 = (0,0,2.5); sphere pcurve
    u∈[0, 1.5707977], v∈[0.747762635, π/2]. tol = 2.14901286e-06.
  * `c=2` Circle, t∈[π, 3π], len 6.28318531 (full circle), p0 = p1 = **(0,0,−2.5) = the
    south pole**; sphere pcurve u∈[1.57078533, 4.71239997], v∈[−π/2, −0.747762629].
    tol = **1.70243155e-05**. Both pave-block ends are DS 7 (the sphere's south pole vertex).
* Why 2 vs 1: the cut rule is *"cut the section curve wherever its 2D image on a periodic
  face must break"*. The north circle passes through the pole **and** crosses the seam
  meridian at (1.833,0,1.7) → 2 cut points → 2 arcs. The south circle passes through the
  pole but never reaches the seam (its u stays in [π/2, 3π/2]) → 1 cut point → 1 closed arc.
* The tangency is absorbed by **tolerance inflation of the two existing pole vertices**,
  not by a special record: DS vertex 5 goes 1e-07 → 2.14903822e-06, DS vertex 7 goes
  1e-07 → **1.70243165e-05**. Both keep `sd=-1` (they are *not* replaced by a same-domain
  vertex; they are widened in place).
* Supporting interferences at the tangency only: `IVF i1=5 i2=11 u=π v=6.29128785`
  (north pole vertex × cylinder lateral face), `IVE i1=7 i2=15 t=1.70871215` (south pole
  vertex × a cylinder edge), `IEE i1=6 i2=15 ctype=VERTEX new=22`.
* The curve count is already 3 at the `after_FF` snapshot and stays 3 at `final` — across
  the whole corpus `SEC tag=afterFF` count == `SEC tag=final` count in every file. **The
  seam/pole splitting is done inside `PerformFF` (the SSI itself), not later in
  `MakeBlocks`.**
* OCCT's own answer is *worse just below tangency than at it*: at 23° OCCT is off by
  3.2e-3 in volume (2e-4 relative), at 23.578° it is exact to 8 digits. The near-miss
  (section circle passing 0.58° from the pole) is numerically harder for OCCT than the
  exact hit. Result is still `valid=1, naked=0` at every angle.

---

## Q2. Section curve crossing a periodic seam

**OCCT splits the section curve at the seam, at the `BOPDS_Curve` level, inside
`PerformFF`.** One `BOPDS_Curve` per seam-delimited arc; each arc normally carries exactly
one pave block.

Evidence from the sphere × cylinder sweep (sphere seam = the u=0/2π meridian):

* **tilt 0–23°** — the section circle *encircles the pole*, so its sphere pcurve winds once
  through the full period: `umin=0, umax=6.28318531` with v varying. It crosses the seam
  once and OCCT emits **ONE curve** for it (a pcurve spanning exactly one full period is
  legal, so no split). 2 curves total.
* **tilt ≥ 23.578°** — the circle no longer encircles the pole, so it crosses the seam
  **twice**; OCCT emits **TWO curves**, one with pcurve u∈[…, 2π] and one with u∈[0, …].
  E.g. at 45°: `c=0` u∈[5.6820329, 2π], `c=1` u∈[0, 0.601152388]. 3 curves total.
* The cylinder side of the same pair is never split: its pcurves happily run outside
  [0,2π] (`umin=6.28318531 umax=12.5663706` at tilt 0) because the cut is driven by the
  *sphere's* seam, and both faces then receive the same set of pave blocks.

Pave blocks per curve across the corpus (142 `SEC` records): **127 curves have npb=1,
6 have npb=2, 2 have npb=4, 7 have npb=0.** So the seam split is expressed as *separate
curves*, and extra pave blocks appear only where an existing vertex lands inside an arc
(cone×cone). The npb=0 curves are all from the coincident-face cases
(`box_box_touch_*`, `box_box_half_fuse`): a section curve that lands on already-existing
geometry is computed but yields no pave block and therefore no section edge.

Cross-pair fusion is exactly symmetric: the `Sc` set of both faces of an FF pair contains
the *same* pave blocks. From `sph_cyl_roty45_cut`:

```
FI f=2  nOn=3 nSc=3   FIPB f=2  set=Sc orig=-1 edge=33 / 32 / 31
FI f=11 nOn=3 nSc=3   FIPB f=11 set=Sc orig=-1 edge=33 / 32 / 31
```

`orig=-1` marks a section pave block (no original edge). The `On` sets hold the split pave
blocks of each face's *own* edges that lie on the other shape.

---

## Q3. One face or two, when split regions meet across the seam

**Measured answer: it depends on whether the region wraps the full period.**

**Case A — region wraps the full u period → ONE face, seam reinstated.**
`sph_box_common` (sphere r=2.5 ∩ box 4×4×4 centred): **res_face = 7** — 1 spherical face +
6 planar discs, `valid=1`, `naked=0`, vol 54.4542756. The single sphere face:

```
RESFACE i=1 surf=Sphere u0=0 u1=6.28318531 v0=-0.927295218 v1=0.927295218
        nwire=4 nedge=9 nseam=2
```

Its outer wire has 8 oriented edge occurrences; the *seam meridian is used twice*, once at
u=0 and once at u=2π, and the seam-straddling hole (the +X box face's disc, centred on the
seam) is **opened into the outer wire** rather than kept as an inner loop
(`RESFEDGE` records, uv values abbreviated):

```
w=0 e=1 REV  uv0=0:0.9273        uv1=6.2832:0.9273      (top boundary circle)
w=0 e=2 FWD  seam uv0=6.2832:0.6435 uv1=6.2832:0.9273   (upper seam fragment, u=2pi)
w=0 e=3 REV  uv0=6.2832:0.6435  uv1=6.2832:-0.6435      (+X hole, right half)
w=0 e=4 FWD  seam uv0=6.2832:-0.9273 uv1=6.2832:-0.6435 (lower seam fragment, u=2pi)
w=0 e=5 FWD  uv0=0:-0.9273      uv1=6.2832:-0.9273      (bottom boundary circle)
w=0 e=4 REV  seam uv0=0:-0.9273 uv1=0:-0.6435           (same edge, u=0)
w=0 e=6 REV  uv0=0:-0.6435      uv1=0:0.6435            (+X hole, left half)
w=0 e=2 REV  seam uv0=0:0.6435  uv1=0:0.9273            (same edge, u=0)
w=1..3  three closed hole circles at u = pi/2, pi, 3pi/2
```

The seam is 2 distinct edges because the +X hole interrupts it. Same structure in
`sph_cyl_roty45_cut`: the sphere face is ONE face with `u0=0 u1=2π v0=-π/2 v1=π/2`,
`nwire=2 nedge=7 nseam=2`, and *both pole degenerate edges in the same outer wire*.

**Case B — region straddles the seam but does NOT wrap → TWO faces.**
`sph_box_cut`: the +X spherical cap straddles the seam and becomes two faces

```
RESFACE i=3 surf=Sphere u0=5.63085983 u1=6.28318531 area=3.92699082 nedge=2
RESFACE i=4 surf=Sphere u0=0          u1=0.652325479 area=3.92699083 nedge=2
```

(3.92699082 + 3.92699083 = 7.85398 = one whole cap, cf. the un-straddling caps i=8/10/12
each area 7.85398). They share the seam-meridian fragment `e4`, which carries a pcurve at
u=2π on face 3 and one at u=0 on face 4; both faces sit in the same closed shell
(`RESSHELL i=2 nface=3 faces=3,4,5 closed=1`, naked=0). OCCT does **not** shift the pcurve
into negative u to keep one face.

Same behaviour on cones: `cone_cone_p1_cut` splits the second cone's lateral face into
`RESFACE i=2 (u 0.4115…2π)` and `RESFACE i=3 (u 0…0.4115)`, each `nseam=1`.

**So: the "seam merge" is only ever a *wrap* merge.** OCCT merges regions across the seam
exactly when the region is periodic-closed in u (then it emits one face and re-inserts the
seam edge twice with opposite orientation); otherwise it emits two faces. That is what
produces 7 faces for sphere ∩ box(4³), and 13 faces (7 spherical + 6 planar) for
sphere − box(4³).

Caveat when reading the dumps: `BRep_Tool::IsClosed(E, F)` is evaluated per
*(surface, location)*, not per face, so it returns 1 for `e4` on **both** halves of a split
cap even though neither face is periodically closed. Use `nwire`/`u0..u1` and the wire
listing (`RESFEDGE`), not `nseam`, to decide.

---

## Q4. Does OCCT create an edge record at a sphere pole / cone apex?

**Yes — a DEGENERATED edge, in the input and in the result. The pole/apex is never
edgeless.**

Input (`BRepPrimAPI_MakeSphere(2.5)`), 1 face / 3 edges / 2 vertices:

```
AFACE  surf=Sphere u0=0 u1=6.28318531 v0=-pi/2 v1=pi/2 uper=1 vper=0
AEDGE  i=1 curve=Degenerated t0=0 t1=6.28318531 tol=1e-07 degen=1 v1=1 v2=1   (north pole)
AEDGE  i=2 curve=Circle      t0=4.71238898 t1=7.85398163 tol=1e-07            (seam meridian)
AEDGE  i=3 curve=Degenerated t0=0 t1=6.28318531 tol=1e-07 degen=1 v1=2 v2=2   (south pole)
AVERT  i=1 p=0,0,2.5  tol=1e-07
AVERT  i=2 p=0,0,-2.5 tol=1e-07
```

Input (`BRepPrimAPI_MakeCone(2, 0, 5)`), 2 faces / 3 edges / 2 vertices — identical
convention:

```
AFACE a=1 i=1 surf=Cone u0=0 u1=6.28318531 v0=0 v1=5.38516481 uper=1
AEDGE a=1 i=1 curve=Degenerated t0=0 t1=6.28318531 degen=1 v1=1 v2=1   (apex)
AEDGE a=1 i=2 curve=Line   t0=0 t1=5.38516481                          (seam)
AEDGE a=1 i=3 curve=Circle t0=0 t1=6.28318531                          (base)
AVERT a=1 i=1 p=0,0,2.5  (apex)
```

Properties of the degenerate edge: it has a **full-period parameter range [0, 2π]**, tol
1e-07, both vertices are the same pole vertex, it carries a pcurve on the face (the
constant-v line v=±π/2, or v=apex on the cone), and it appears **once** in the wire (unlike
the seam, which appears twice).

In the **result** it survives and is *split like any other edge*:

* `sph_box_cut` face 1 (north cap): `RESFEDGE f=1 w=0 e=3 REV seam=0 degen=1 pc=1
  uv0=0:1.5708 uv1=6.2832:1.5708` — one whole degenerate pole edge inside the face's wire.
* `sph_cyl_roty23578_common` (exact tangency): the north pole edge is **split into two
  degenerate edges**, `e3` with pcurve u∈[0, 1.5708] on face 1 and `e7` with pcurve
  u∈[4.7124, 2π] on face 3, plus `e8` (south pole) with pcurve u∈[1.5708, 4.7124]. Three
  degenerate edges in the result (`res_degen=3`), each with `nface=1`.
* `cone_cone_p1_cut`: `RESEDGE i=10 curve=Degenerated tol=1e-07 nface=1` — the apex edge of
  the surviving cone.
* Degenerate edges legitimately have `nface=1` and must be excluded from any naked-edge
  count; every result in the corpus reports `naked=0` under that rule.

Contrast: a cylinder has **no** degenerate edge (`sph_cyl` arg 1: edges are Circle / Line /
Circle, `degen=0` throughout), and its seam is a Line.

---

## Q5. Section-edge tolerances and where they come from

Census over the 142 `SEC` records of the corpus (`tol` = `BOPDS_Curve::Tolerance()`):

| tol | count | note |
|---|---|---|
| 1e-07 | 79 | `Precision::Confusion` — the floor |
| 1.5e-07 | 29 | 23 of them with `tantol` 2.12132034e-07 ( = 1.5e-7·√2 ) |
| 1.00987792e-07 … 1.77869085e-07 | 18 | analytic sphere/cylinder/cone sections, just above confusion |
| 1.50003136e-07 / 1.50361662e-07 | 8 | approximated (BSpline) cone×cone sections |
| 2.14901286e-06 / 2.14903722e-06 | 4 | sphere×cylinder at **exact pole tangency**, north arcs |
| 4.70466939e-06 | 2 | cone×cone p1, the long BSpline section |
| **1.70243155e-05** | 2 | sphere×cylinder at exact pole tangency, **south circle** |

`IntTools_Curve::TangentialTolerance()` is 1.5e-07 for almost every curve (a handful:
2.12132034e-07, 3e-07, 4.98826429e-07, 5.79555496e-07).

Measured propagation rules:

1. **Section edge tolerance = the pave block's own tolerance, recomputed per sub-range —
   it is not simply inherited from the parent `BOPDS_Curve`.** When a curve has one pave
   block, `etol == BOPDS_Curve::Tolerance()` exactly. When it has several, they differ:
   `cone_cone_p1_cut` c=4 has `tol=1.50003136e-07` but its four pave blocks get
   `1.5e-07, 1.5e-07, 1.50003136e-07, 1.5e-07` — only the sub-range containing the worst
   deviation keeps the parent value; the rest fall back to 1.5e-07.
2. **`BOPAlgo_BOP` does not change it.** Every `RESEDGE tol=` equals the DS `etol` of the
   pave block it came from (e.g. `sph_cyl_roty45_cut`: RESEDGE 2/5/7 tol =
   1.29109247e-07 / 1.14703676e-07 / 1.1236843e-07 = the three `SEC tol` values verbatim).
3. **Vertex enlargement = curve tolerance + exactly 1.0e-12.** Measured twice at the
   tangency: curve tol 2.14903722e-06 → vertex tol 2.14903822e-06; curve tol
   1.70243155e-05 → vertex tol 1.70243165e-05. Both existing pole vertices were widened
   *in place* (`sd=-1`); no same-domain replacement vertex was created.
4. **Same-domain (VV) fusion works the other way**: in `box_box_touch_fuse` the 8
   coincident source vertices are *not* widened, they are mapped to **4 brand-new DS
   vertices** (`SD i=15 sd=68`, `i=39 sd=68`, …, indices 68–71 ≥ `NbSourceShapes`=68).
5. Untouched input geometry keeps tol 1e-07 end to end; OCCT never lowers a tolerance.

---

## Other ground truth captured (for diffing our kernel)

`traces/_summary.txt` has one `SUMMARY` line per case with DS counts, interference counts
per type, section-curve/pave-block counts and the result's solid/shell/face/edge/vertex/
naked/degenerate counts, volume, area and validity. Every case in the corpus is
`valid=1 naked=0 pf_err=0 bop_err=0`. Highlights:

| case | sec curves | res faces | res solids | volume |
|---|---|---|---|---|
| box_box_cut (control) | 8 | 10 | 1 | 48 |
| box_box_common | 8 | 6 | 1 | 16 |
| sph_sph_p1_cut | 2 | 2 | 1 | 58.5285258 |
| sph_sph_p2_cut | 3 | 2 | 1 | 50.0149375 |
| cyl_cyl_cut | 2 | 4 | 1 | 47.6817989 |
| cyl_cyl_common | 2 | 3 | 1 | 8.86713168 |
| box_cone_p1_cut | 2 | 7 | 1 | 48.7528037 |
| box_cone_p2_cut | 6 | 8 | 1 | 46.5479313 |
| cone_cone_p1_cut | 6 | 4 | 1 | 15.7076198 |
| cone_cone_p2_cut | 7 | 7 | **2** | 6.60028262 |
| sph_box_common | 7 | **7** | 1 | 54.4542756 |
| sph_box_cut | 7 | 13 | **6** | 10.9955738 |
| sph_box_fuse | 7 | 13 | 1 | 74.9955738 |
| box_box_touch_common | 2 | **0** | 0 | 0 (empty compound, valid, no error) |

Two further facts that matter for us:

* **OCCT keeps micro-edges.** `cone_cone_p1_cut` ships result edges of length
  1.50881598e-05, 2.69613843e-05 and 3.61989887e-05 (`RESEDGE i=5/4/3`) rather than
  collapsing them, and still reports `valid=1 naked=0`.
* **`BOPDS_CommonBlock` only ever appears for coincident geometry.** Zero common blocks in
  all 35 transverse cases. They appear only in the added touching-box cases:
  `box_box_touch_fuse` → 4 common blocks, each with 2 pave blocks on the two coincident
  edges (`pbs=14:0:4|38:0:4`), `faces=-`; `box_box_half_fuse` → 2 of that kind plus 2 of the
  edge-on-face kind (`npb=1 nfaces=1 faces=12`). Both flavours are dumped.

---

## Record schema (grep keys)

```
TRACE / SPEC                 header, op, OCCT version, operand specs
ARG / AFACE / AFEDGE /
  AEDGE / AVERT              per-operand input topology + geometry + tolerances
STAGE name=after_XX          per-stage counts (shapes, SD, PB, CB, faceinfo, VV..FF, curves)
DS / RANGE / SI              data-structure header, per-operand index ranges, ShapeInfo
DSVERT / SD                  every DS vertex (xyz, tol, new?, same-domain) and the SD map
PAVE / PB / CB               paves per edge; pave blocks (t0,t1,v1,v2,edge,etol,shrunk); common blocks
FI / FIPB                    BOPDS_FaceInfo In/On/Sc, vertices and pave blocks
IVV IVE IVF IEE IEF IFF      interference records (sorted, so runs diff cleanly)
SEC / SEC2D / SECPB / SECTV  section curve, its 2D footprint on both faces, its pave blocks
FFP                          FF section points
RES / RESSOLID / RESSHELL /
  RESFACE / RESFEDGE /
  RESEDGE / RESVERT          result topology; RESFEDGE lists oriented wire occurrences + pcurve uv
IMGFACE / IMGEDGE            BOPAlgo_Builder::Images() — input face/edge -> result indices
SUMMARY                      one comparable line per run
```

Tags: `tag=afterFF` = state right after `PerformFF`; `tag=final` = after `Perform()`
completed. Floats are `%.9g` with |x| < 1e-12 snapped to 0; every list-valued record is
sorted, so two runs (and two builds) diff cleanly.

Rerun with `validation/occt_trace/run_traces.sh` (build first:
`cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 &&
cmake --build build -j`). Single case:

```
./build/occt_trace --op cut --a sphere,r=2.5 \
    --b cylinder,r=1,h=8,center,roty=23.578178478201835 --name x --out x.trace
```

Operands: `sphere,r=` `cylinder,r=,h=` `box,dx=,dy=,dz=` `cone,r1=,r2=,h=` `torus,r1=,r2=`
`step,file=<path>`, plus `center`, `rotx=`, `roty=`, `rotz=`, `tx=`, `ty=`, `tz=` (applied
in that order, about the global origin, with `BRepBuilderAPI_Transform(copy=true)` so the
geometry is genuinely rotated rather than carrying a `TopLoc_Location`). The STEP path is
exercised and works (`schoring_head_0.step` × box: 10 solids / 75 faces in, pipeline clean).
