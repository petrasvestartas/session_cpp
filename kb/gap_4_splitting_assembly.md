# GAP 4 — THE SPLITTING AND ASSEMBLY LAYER

**Question answered:** what must be implemented, in the layer that turns sections into faces and
faces into solids, to make our booleans correct — ranked by *measured* impact.

**Method.** Everything below is either (a) a record from an OCCT trace, (b) a number I measured by
running our own kernel, or (c) an OCCT `file:line`. Nothing is inferred from OCCT source where a
trace could be run instead. Claims that could not be measured are marked
**"no measured impact yet"** and are ranked last, not omitted.

---

## 0. PROVENANCE OF EVERY NUMBER IN THIS FILE

**OCCT side.** Existing corpus `validation/occt_trace/traces/` (38 cases) plus **6 new cases I ran**
with `validation/occt_trace/build/occt_trace` (OCCT 8.0.0.rc2-a66b3fd6):

```
--op {cut,common,fuse} --a "cone,r1=2,r2=0,h=4,rotx=-90,ty=-2.8" --b "cylinder,r=1.5,h=6,tz=-3"
--op {cut,common}      --a "box,dx=4,dy=4,dz=4,center"           --b "torus,r1=2,r2=0.8,rotx=45"
--op cut               --a "box,dx=4,dy=4,dz=4,center"           --b "box,dx=2,dy=2,dz=2,center"
--op cut               --a "cone,r1=2,r2=0,h=4,tz=-2.8"          --b "cylinder,r=1.5,h=6,rotx=-90,ty=-3"
```

The first is the matrix cell `coneR x cyl` reproduced exactly: OCCT returns cut 12.307399,
common 4.44774457, fuse 54.7193869 — the values the brief quotes. The last is that same cell
rigidly rotated by −90° about X (a control for pose dependence).

**Our side.** `session_cpp/build_v2diff/main_20` (the v2 tracer, binary dated 2026-07-26 14:23),
which emits the *same record format* as `occt_trace`, driven with the same operand strings;
`SESSION_V2_NOFRONT=1` forces the v1 kernel, `SESSION_V2_SFDBG=1` dumps the per-face-image
classification, `SESSION_SD_TRACE=1` dumps every classification probe.

**Caveat that must not be lost:** `_gate_matrix.txt` and the 63-cell battery in `main_7` were
produced by *other* builds. Absolute volumes are only comparable within one binary. I could not
re-run the 63-cell battery (a filtered `./build/main_7 "coneRx cyl"` run exceeded 300 s and was
killed), so the *census* of the 29 failing cells is taken from the brief; the *mechanisms* below
are measured first-hand on two representative cells (`coneR x cyl`, `sphere x box`).

**Which kernel actually answers.** Measured over the 14 cases I ran through `main_20`: the v2 front
end **accepted 5** (`coneR x cyl` ×3, its rotated control, `sphere x cylinder@45`) and **refused 9**
(`BOPERR v2-front-refused(delegated-to-kernel)`) — including every case containing a box *and*
the coaxial `cone x cylinder`. So today **v1's splitter still answers the majority of the matrix**,
and v1's defects dominate the 29 failing cells. Both kernels are covered below.

---

## 1. WHAT OCCT DOES

### 1.1 The splitter consumes ORIENTED OCCURRENCES of SHARED edges; identity is the shape, never a coordinate

`BOPAlgo_WireSplitter_1.cxx:136-195` fills `mySmartMap`, keyed by the **vertex shape itself**; each
edge contributes one `BOPAlgo_EdgeInfo` per vertex occurrence with `IsIn = (orientation ==
REVERSED)`. Multiplicity is the *input's* job: a boundary edge appears once, a section/IN edge twice
(FWD+REV), a seam twice with two pcurves. The "is this edge interior" flag is derived, not passed:
`bIsClosed = Degenerated(E) || IsClosed(E,F)` (`:146`), and an edge is removed from the fence map
`aMS` only when supplied twice **and not closed** (`:148-151`), so `IsInside = supplied twice AND
not a seam AND not degenerate` (`:309`).

**Fast path (`:199-295`):** if every vertex has exactly one In and one Out *and* no edge appears
twice, the whole set becomes ONE wire with **no angular computation at all**. Angles are only ever
computed for genuinely branching arrangements.

### 1.2 The angular rule at a multi-edge vertex on a curved surface — VERIFIED

The brief asked whether our metric key matches OCCT. **It does not, and that is safe.** Measured
from source:

* `Angle2D` (`BOPAlgo_WireSplitter_1.cxx:768-840`) is the **raw 2D angle**: it evaluates the pcurve
  at the vertex and at one step `dt`, forms `gp_Vec2d`, and takes
  `Angle(gp_Dir2d)` = angle to the 2D X axis (`:836-837`, `:844-855`). **No first fundamental form
  appears anywhere in the WireSplitter or in BOPTools_AlgoTools.** The surface enters only through
  the *step size*: `dt = max(Resolution(2·Tolerance2D(V)), PConfusion())`, enlarged by
  `acos(R/(R+tol2d))` for curved pcurves so a large radius does not get a microscopic step
  (`:792-808`), then capped at `0.05·(last-first)`, or `min(5e-5, span/2)` if that is smaller
  (`:809-820`). `Tolerance2D` (`:859-881`) is `max(UResolution(tolV3D), VResolution(tolV3D))`,
  floored at `tolV3D` and multiplied by 1.1 on BSpline surfaces — a **model-space** tolerance pushed
  through the surface, exactly the discipline our `sf_uv_tolerance` follows.
* An IN record measures the direction **toward** the vertex, an OUT record **away** (`:834`).
* `ClockWiseAngle` (`:621-659`) measures from the *reversed* incoming direction; a difference
  `dA <= 1e-14` is snapped to `2π`, i.e. a tie is pushed to LAST.
* The winner (`:527-611`): one-way-out shortcut (`:557-562`); the arriving edge itself scores `2π`
  (`:564-567`); on a vertex flagged closed, candidates whose 2D image is farther than `2·Tolerance2D`
  from the arrival point are skipped (`:571-582`); strict improvement by `eps = Epsilon(1.)`
  (`:595`), so the FIRST candidate in list order wins a tie; and the override that decides the
  hard cases: **if the walk arrived on a boundary edge and exactly one unpassed *interior* edge
  leaves this vertex, take it whatever the angle says** (`:602-605`).

**Our metric key `atan2(dv·√(EG−F²), du·E + dv·F)` gives the same winner as OCCT's raw key in exact
arithmetic.** Proof: the key is the angle of `M·(du,dv)` with
`M = [[E, F],[0, √(EG−F²)]]/√E`, and `det M = √(EG−F²) > 0`; an orientation-preserving linear map of
the plane preserves the cyclic order of directions, hence preserves both the clockwise ordering and
the minimum. It is a **conditioning** improvement (it undoes the anisotropy squash `|S_u|/|S_v|`),
not a behavioural change. Consequence for ranking: **the angular key cannot be the cause of any
failing cell** (see §4, item 8).

`RefineAngles` (`:905-1125`) refines converging pcurves by intersecting them with the face's
bounding curves; we replaced it with secant escalation
(`brep_v2_splitface.cpp:452-502`). Divergence acknowledged; no measured impact either way.

### 1.3 Outer vs inner wire, and how images share edges

`BOPAlgo_BuilderFace.cxx:387-614` (`PerformAreas`):

* Each wire becomes a draft face; **growth vs hole is decided first by a topological shortcut** —
  `IsGrowthWire`: if the wire contains any edge already used by a known hole face, it is a growth
  (`:441`) — and only then by `IntTools_FClass2d::IsHole()` (`:445-447`), a real UV classifier that
  knows the surface's periodicity. **It is not a signed-area test.**
* Holes are attached to the **innermost** containing growth face: BVH over UV boxes (`:468-511`),
  containment by `IsInside` (a point-in-face classification), and when a hole already has an owner
  the new candidate wins iff it is itself inside the old owner (`:524-535`).
* A hole with no owner is only rehomed into a fresh face when the original face is **infinite**
  (`:557-581`); otherwise it stays unowned.
* `PerformShapesToAvoid` (`:152-235`) prunes to a fixpoint: a vertex with exactly one incident edge
  kills that edge — **except a degenerate edge (`:200-203`) or an INTERNAL vertex (`:204-207`)** —
  and a vertex whose two incident entries are the same edge kills it, unless the edge's two vertices
  are the same (a closed edge, `:216-222`).
* Unconsumed edges become INTERNAL wires inside the containing area (`:618-778`), and anything still
  unclassified raises `BOPAlgo_AlertFaceBuilderUnusedEdges` (`:776-777`). **Nothing is dropped
  silently.**
* Neighbouring images reference the **same edge entity**: across the whole corpus every
  non-degenerate `RESEDGE` reports `nface=2`.

### 1.4 SEAMS AND POLES — the measured rules

| rule | trace evidence |
|---|---|
| region **wraps** the full period → **ONE face**, seam edge reinstated **twice** (u=u0 and u=u1) | `coneR_cyl_cut` `RESFACE i=1 surf=Cone u0=0 u1=6.28318531 nwire=1 nedge=3 nseam=1` with `RESFEDGE e=2 ori=FWD uv=6.2832:*` and `e=2 ori=REV uv=0:*`; `sph_box_cut` faces 1 and 6; `sph_box_common` face 1 (`u0=0 u1=2π`, `nwire=4 nedge=9 nseam=2`) |
| region wraps in **both** periods (torus) → ONE face, **both** seams reinstated twice | `box_torR_common` `RESFACE i=2 surf=Torus u0=0 u1=2π v0=0 v1=2π nwire=5 nedge=13 nseam=3`; outer wire uses u-seam `e4` at u=0 and u=2π and v-seam pieces `e6`,`e7` at v=0 and v=2π; the four section circles are four **inner** wires |
| region **straddles** the seam without wrapping → **TWO faces**, sharing the seam fragment | `sph_box_cut` `RESFACE i=3 (u 5.63086…2π, area 3.92699082)` + `i=4 (u 0…0.652325, area 3.92699083)`, sum = one whole cap 7.85398; the shared fragment `RESEDGE i=4 … nface=2` |
| a section curve whose 2D image spans **exactly one period** is **not** split | `coneR_cyl_cut` `SEC c=0 … npb=1` with `SEC2D face=2 umin=0 umax=6.28318531`; sph×cyl at tilt 0–23° |
| a section crossing the seam **twice** is split into two `BOPDS_Curve`s | sph×cyl ≥23.578°; `sph_box_cut` edges 59/60 (`t0=0..π`, `t0=π..2π`) |
| the **seam edge itself is split** by EF vertices, and each fragment keeps TWO pcurves | `coneR_cyl_cut` `FIPB f=2 set=On k=0 orig=6 t0=0 t1=1.99810922 edge=25` / `k=1 … edge=26`, created by `IEF i1=6 i2=13 ctype=VERTEX new=24`; `sph_box_cut` splits the sphere seam into **5** pave blocks from 4 EF vertices. Mechanism: `BOPTools_AlgoTools3D::DoSplitSEAMOnFace` (`:236-327`) projects the split's midpoint onto both original pcurves, translates a copy by the period, and calls `BRep_Builder::UpdateEdge(aSp, aC1, aC2, F, tol)`; the tangent-dot test (`:317-327`) decides which copy is the FORWARD pcurve |
| a **pole** carries a DEGENERATED edge over the full period, appearing **ONCE** in the wire | `sph_box_cut` `RESFEDGE f=1 w=0 e=3 ori=REV degen=1 uv0=0:1.5708 uv1=6.2832:1.5708`; `coneR_cyl_common` `RESEDGE i=3 curve=Degenerated … nface=1` |
| poles are **never merged** across, only split | `sph_cyl_roty23578_common`: the north pole edge becomes two degenerate edges, `res_degen=3` |

### 1.5 Shell and solid assembly

* **Connexity by shared edge only.** `BOPAlgo_ShellSplitter::Perform` →
  `BOPTools_AlgoTools::MakeConnexityBlocks(shapes, EDGE, FACE)` (`ShellSplitter.cxx:142`).
* **Free-edge peeling to a fixpoint** before the walk (`ShellSplitter.cxx:185-222`): any face with an
  edge used once (non-degenerate, non-INTERNAL) is removed, repeatedly.
* **The walk** (`:253-360`): iterate the shell as it grows; skip an edge that already has ≥1 face in
  this shell (`:283-292`), skip INTERNAL and degenerate edges; a candidate must contain the edge with
  the opposite orientation (`GetEdgeOff`); if the current face is a *boundary* face (supplied once)
  and exactly one candidate is *not* a boundary face, take it (`aNbWaysInside == 1`); otherwise pick
  the minimal dihedral angle via `GetFaceOff`.
* **Growth vs hole, from that shell alone**: `IsHole` = `BRepClass3d_SolidClassifier::
  PerformInfinitePoint` (`BuilderSolid.cxx:823-833`). Nesting: innermost parent wins
  (`:497-548`); a hole shell inside no growth becomes its own solid (`:578-589`).
* **One solid may own several shells.** `box_inside_cut`: `RESSOLID i=1 nface=12` with
  `RESSHELL i=1 nface=6 closed=1` (outer) and `RESSHELL i=2 nface=6 closed=1` (the cavity), vol 56.
* **Multi-solid results are normal and correct.** `sph_box_cut` = **6 solids / 6 shells / 13 faces**,
  `valid=1 naked=0`; `cone_cone_p2_cut` = 2 solids; `box_box_touch_common` = an empty compound,
  `valid=1`, no error.
* **Selection** (`BOPAlgo_Builder.cxx:479-885`): two states `(objState, toolState)`;
  `bAvoidIN = !objIN && !toolIN` (FUSE), `bAvoidINforBoth = objIN != toolIN` (CUT),
  `isSameOriNeeded = (objState == toolState)`. **Two fences**: `aMFence` unoriented and
  `aMFenceOri` oriented. A face image met a second time and *also present in the opposite group* is
  a same-domain wall: it is taken **without any classification** iff
  `isSameOriNeeded == isSameOri`, else added to `aMFToAvoid` (`:682-712`). Otherwise
  `bTakeIN == isINOpposite` decides, with the IN case emitting the image **both ways** and the
  CUT case emitting it reversed (`:719-735`).
* **Two inputs that selection depends on, and where they come from:**
  1. **Image orientation** — `BOPTools_AlgoTools::IsSplitToReverse(aFIm, aF)`
     (`Builder.cxx:594`, implementation `BOPTools_AlgoTools.cxx:1324-1436`): find an interior point
     of the *split* face, take the surface normal there, apply the face's own orientation, project
     that point onto the *original* face, take its normal there, compare the dot product. **Purely
     local. It never assembles a shell and never computes a volume.**
  2. **IN membership** — `myInParts`, filled by `BOPAlgo_Builder::FillIn3DParts`
     (`Builder_3.cxx:97-263`) → `BOPAlgo_Tools::ClassifyFaces` → `BOPAlgo_FillIn3DParts::Perform`
     (`BOPAlgo_Tools.cxx:1334-1518`). Three properties matter:
     * face images are grouped into **connexity blocks that never cross the solid's own edges**
       (`MakeConnexityBlock(aFP, aMSE, …)`, `:1465`), and the whole block takes ONE verdict;
     * the verdict is `BOPTools_AlgoTools::IsInternalFace` (`AlgoTools.cxx:807-891`), which **first
       tries the ANGLE method around an edge shared with the solid** (`:828-874`) — the dihedral
       wedge test — and only falls back to `ComputeState` (a point classifier) when no shared edge
       exists or the angle method returns "undecidable" (`:881-890`);
     * classification is against the **draft solid** (the solid rebuilt from its own split faces,
       `BuildDraftSolid`, `Builder_3.cxx:267+`), not against the original operand.

### 1.6 Where OCCT is itself imperfect (do not copy blindly)

* **Micro-edges are kept.** `cone_cone_p1_cut` ships result edges of length 1.50881598e-05,
  2.69613843e-05, 3.61989887e-05 and still reports `valid=1 naked=0`.
* **Near-tangency is worse than exact tangency**: sphere×cylinder at 23° is off by 3.2e-3 in volume;
  at the exact pole tangency 23.578178478° it is exact to 8 digits.
* **The straddle answer (2 faces) is a representation choice, not a theorem.** One face over a
  periodically continued chart is equally watertight. But it is the answer that needs no periodic
  evaluation anywhere downstream, and it is the one every consumer of ours already supports.
* Per the brief, OCCT omits a legitimate lump of A\B in two chair cells. Parity is not the target;
  correctness is.

---

## 2. WHAT WE DO

### 2.1 v1 — the kernel that still answers most of the matrix

**There is no wire walk.** `NurbsSurfaceTrimmed::split_by_uv_curves`
(`src/nurbssurface_trimmed.cpp:539`) is a **UV polyline arrangement**:

* every cutter is sampled to a polyline, adaptively refined to `samp_tol = 2e-5 · max(range_u,
  range_v)` (`:577`, `:599-637`);
* every sample is clamped to the domain and **snapped to the domain border** within `snap_uv`
  (`:581-590`, `:641-642`) — this snap is the only reason two faces' copies of one edge ever come out
  bit-identical (the failure it hides is quantified in `brep_v2_splitface.h:14-23`: splitting ONE box
  on a padded domain gives **32 naked edges of 36** while the UV arrangement is verified identical);
* regions are the arrangement's cycles; outer/inner by signed area with point-in-cycle nesting
  (`:1300-1321`);
* **every emitted loop is a degree-1 pcurve** (`:1691`, `:2331`, inner loops `:2416`).

**Every split edge's 3D curve is also a degree-1 polyline.** `lift_loop` (`src/brep.cpp:3588-3733`)
lifts the pcurve by adaptive chord refinement and ends with
`c3d = NurbsCurve::create(false, 1, pts3)`.

**Identity is coordinate-based.** `q6(x) = llround(x·1e6)` (`brep.cpp:2996`); vertices weld through
`vmap` on the quantised key with a 3×3×3 neighbour scan (`:3846-3860`); edges match either through
`bemap` — keyed by *(original edge, welded vertex pair)* and accepted when the two copies'
arc-length midpoints agree within `1.5·(devtol_a + devtol_b) + 0.1·bemap_tol` (`:4268-4286`) — or
through `emap`, keyed by *(v_lo, v_hi, q6(midpoint))* (`:4288-4296`).

**Seam handling.** The wrap-only merge exists and — measured against the traces — implements the
**right** rule: it unions two regions across a border only if a full-period march at a shared seam
parameter stays inside their union (`:1390-1414`), and it explicitly refuses the pole merge
(`:1416-1425`, with the sph_cyl_roty23578 evidence written into the comment). Two problems:
it is **env-gated OFF** (`s_seam_merge = getenv("SESSION_SEAM_MERGE") != nullptr`, `:1339-1342`), and
the merged face is represented as **one face with several OUTER wires**
(`brep.cpp:4977-4988`), which OCCT never produces and which the mass-properties integrator has to be
told about.

**Poles** match OCCT: `create_sphere`/`create_cone` carry a degenerate pole edge with a full-period
range (`brep.cpp:509-540`), opt-out `SESSION_NO_POLE_EDGE`.

**Assembly.** `BRep` has **no shell and no solid records at all** (`src/brep.h`): the result is faces,
loops, trims, edges, vertices. Shell structure is recomputed by connexity wherever a metric needs it
(e.g. `main_7.cpp:177-199`). Orientation is `face_outward_signs` (tiers: planar plane / cavity
inversion / volume flux / ray parity).

### 2.2 v2 — a real port of BuilderFace + WireSplitter

`src/v2/brep_v2_splitface.cpp`: `classify_multiplicity` (`:247`, OCCT's `IsInside` derivation
reproduced exactly), `build_nodes` (`:286`; node identity is the **resolved arena vertex index**,
`:307`), `prune_to_fixpoint` (`:372`, with the degenerate and closed-edge exemptions),
`compute_angles` (`:452`, metric key + secant escalation), `path` (`:506`, OCCT's stack machine with
closure scan, slit rejection and truncate-and-continue), `areas` (`:794`, signed-area role with the
shared-pave-block exclusion), `internals` (`:861`). `SfEmitter` (`:1089-1188`) materialises one BRep
edge per pave block keyed by pointer identity — **no coordinate key anywhere**.

Splitter input (`src/v2/brep_v2_boolean.cpp:840-896`): boundary pave blocks get an **exact** sub-pcurve
(`NurbsCurve::trim` of the stored curve; the degree-1 resample at `:250-259` is only a fallback when
`trim` fails). Section blocks (`:902-930`) get a **cubic interpolant** through ≤192 trail samples
(`:295-345`); the "degree-1 chord polygon" comment at `:907` is stale relative to `:340`.

Seam composition (`:656-733`): a seam pave block whose two occurrences land in **two different
wires** is treated as interior and the two wires are **spliced into one face**, with the spliced half
translated by the period difference measured between the two pcurves.

Assembly (`src/v2/brep_v2_solid.cpp`): `perform_shapes_to_avoid` (`:577`), `perform_loops` (`:608`,
connexity by shared edge index only), `split_block` (`:665`, a faithful port of ShellSplitter's walk
including free-edge peeling, `GetEdgeOff` direction test and the boundary "only way inside"
override), `perform_areas` (`:760`, growth/hole then innermost-parent nesting).
`v2_select_faces` (`:858-940`) is a faithful, line-referenced transcription of BuildBOP including
both fences and the same-domain branch.

Two inputs to selection are **ours, not OCCT's**:

* orientation: `v2_outward_signs` (`brep_v2_boolean.cpp:93-123`) = build `V2Topo` → run
  `V2BuilderSolid` → per-shell signed volume sign. **It needs a successful assembly to produce the
  orientation that the assembly needs.**
* IN membership: `sd_classify_face` / `sd_classify_samples`
  (`src/brep_samedomain.cpp:603-673`) = sample points in the face, `nearest_on_solid(other, p)`,
  decide by the sign of `(p−q)·n_out`, with a tessellated ray-parity fallback when the hit is on a
  trim boundary or `|dp|` is tiny.

The output BRep is the arena subset (`brep_v2_boolean.cpp:1199-1211`); `V2BuilderSolid`'s shells and
solids feed the **report only** (`:1217-1231`).

---

## 3. THE GAP — as concrete implementable deltas

**G1 — Image orientation must be LOCAL.** Replace `v2_outward_signs` as the source of
`V2OrientedFace::reversed` with a port of `IsSplitToReverse(split_face, source_face)`: interior point
of the image (we already have `v2_face_probe`), surface normal there, source-face normal at the
projected point, compare dot. Keep `v2_outward_signs` only as a diagnostic. This removes the
circular dependency assembly→orientation→assembly, and makes orientation independent of whether a
shell closed.

**G2 — IN membership must be a connexity-block verdict, angle-first.** Three changes, in order of
value:
  (a) group face images into blocks that never cross an edge of the other operand's split, and give
      the whole block ONE verdict (OCCT `MakeConnexityBlock` with `aMSE` = the solid's edges);
  (b) decide the block with the **dihedral angle around a shared edge** when one exists — we already
      have `v2_angle_with_ref` and `v2_get_face_off` (`brep_v2_solid.cpp:345`, `:375`), so this is
      a re-use, not new mathematics;
  (c) only fall back to a point classifier, and when doing so classify against the **draft solid**
      (the other operand rebuilt from its own images) rather than the raw operand.

**G3 — The seam-composition rule is inverted; delete the merge.** `compose_across_seams`
(`brep_v2_splitface.cpp:656-733`) merges exactly the case the traces say OCCT splits (straddle:
two occurrences in two different wires) and leaves alone exactly the case OCCT merges (wrap: two
occurrences in one wire — which the walk already assembles correctly, verified by
`coneR_cyl_cut RESFACE i=1` and by main_14's `cylinder seam_edge_appears_twice_in_one_wire`). Delta:
remove the splice; let the two straddle regions stay two faces; materialise the seam fragment as one
shared edge with two trims (its 2 uses then come from the two halves, as in `sph_box_cut RESEDGE
i=4 nface=2`). This also deletes the requirement for `sf_periodic_extend`, i.e. it removes the
obligation on *every* downstream consumer to evaluate a chart outside its stored domain.

**G4 — v1's split faces must carry exact geometry.** Two sub-deltas, in increasing cost:
  (a) boundary runs: instead of re-sampling the arrangement's polyline, `trim` the face's stored
      pcurve and its stored 3D curve to the run's parameter window (v2 already does this at
      `brep_v2_boolean.cpp:876-882`). This is where the rational conic survives; `NurbsCurve::create`
      cannot express weights, so re-sampling a circle *always* loses them;
  (b) section runs: fit the marched trail with a cubic (or better) instead of
      `NurbsCurve::create(false, 1, …)` in `lift_loop` (`brep.cpp:3724`).

**G5 — v1's wrap merge must be on by default, and represented as ONE wire.** Ungate
`SESSION_SEAM_MERGE` (`nurbssurface_trimmed.cpp:1339-1342`) and change the emission from "extra
OUTER wires" (`brep.cpp:4977-4988`) to OCCT's form: one wire that uses the seam edge twice, at u=u0
and u=u1. The wrap test at `:1390-1414` already computes the right predicate.

**G6 — The result must carry shell and solid records.** `BRep` has none, so "1 solid with a cavity"
and "2 solids" are the same object today. Add shell/solid records fed from `V2BuilderSolid::areas()`
(the data already exists at `brep_v2_boolean.cpp:1217-1224` and is thrown away).

**G7 — Micro pave blocks must be unified, not split.** In `coneR x cyl` our arena paves the cone's
seam at `t=1.99810918` (an EF probe vertex) *and* at `t=1.99810922` (the section's own endpoint),
producing a pave block of parameter length 4e-8 whose two vertices are **5.43e-4 apart in 3D**
(`RESVERT i=2 p=1.1064183,-1.01283688,0.000542923558` vs `i=3 p=1.10641839,-1.01283678,0`). OCCT has
ONE vertex there (`SECPB … v1=24 v2=24`, `npb=1`). Delta: OCCT's `SplitPaveBlocks` rule — a block
with no valid range **unifies its endpoint vertices** rather than being emitted.

**G8 — Small, no measured impact yet:** the WireSplitter fast path (`:199-295`); an
`FClass2d`-grade hole test to replace the signed-area role assignment (`brep_v2_splitface.cpp:816`);
`IsGrowthWire`'s topological shortcut; OCCT's `RefineAngles`.

---

## 4. MEASURED IMPACT — ranked

### 1. G1+G2 (selection input) — the entire `coneR x cyl` row, and it is the worst cell in the matrix

**v2, measured.** On `coneR x cyl` the v2 pipeline **computes the right faces and then selects the
complementary set**. The three ops come out permuted:

| requested | v2 volume | equals | OCCT truth for the request | error |
|---|---|---|---|---|
| cut | 54.7189165 | OCCT **fuse** 54.7193869 | 12.307399 | **+345 %** |
| common | 37.9637557 | **cut21** (42.4115008 − 4.44774457 = 37.9637562) | 4.44774457 | **+753 %** |
| fuse | 12.3074157 | OCCT **cut** 12.307399 | 54.7193869 | **−77.5 %** |

The permutation is exactly "the tool's IN/OUT membership is inverted" in the BuildBOP table.
`SESSION_V2_SFDBG=1` confirms it face by face — all four cylinder images are classified backwards
while all three cone images are right:

```
[SFDBG] SPLIT A{F=3 shells=1 solids=1 naked=0 vol=16.7551608 closed=1}  want_vol=16.7551608191
[SFDBG] SPLIT B{F=4 shells=1 solids=1 naked=0 vol=42.4115008 closed=1}  want_vol=42.4115008235
[SFDBG] SPLIT faithful=1
[SFDBG] cls k=0 op=0 st=Out  area=17.565850   <- cone lateral outside cylinder   CORRECT
[SFDBG] cls k=1 op=0 st=In   area=10.533409   <- cone apex cap inside cylinder   CORRECT
[SFDBG] cls k=2 op=0 st=Out  area=12.566371   <- cone base                       CORRECT
[SFDBG] cls k=3 op=1 st=In   area=51.421725   <- cylinder remainder              WRONG (is OUT)
[SFDBG] cls k=4 op=1 st=Out  area=5.126943    <- cylinder patch inside cone      WRONG (is IN)
[SFDBG] cls k=5 op=1 st=In   area=7.068583    <- cylinder cap                    WRONG (is OUT)
[SFDBG] cls k=6 op=1 st=In   area=7.068583    <- cylinder cap                    WRONG (is OUT)
```

`SESSION_SD_TRACE=1` gives the mechanism: **every** probe on a cylinder image returns nearest face
`fb0` (the cone's lateral face) with `dp < 0`, including points far outside the cone —
`p(0.9386,1.1701,2.4834) -> fb0 d=2.444 dp=-2.025` (the cone's apex is at (0,1.2,0), so this point is
2.65 outside). The nearest-point-plus-signed-normal classifier degenerates near a **pole**: the
nearest point collapses onto the apex, where the normal is undefined, and the sign is wrong over a
whole half-space. OCCT never asks that question — it uses the dihedral angle around the shared
section edge (§1.5) and only classifies a point when there is no shared edge.

**Pose control:** rotating the whole cell by −90° about X reproduces the same wrong answer
(54.7189163), so this is a property of the *configuration* (cone apex inside the tool), not of the
rotated chart. The coaxial `cone x cylinder` is answered by v1 and is exact (2.61799388 = 5π/6).

**v1, measured** (`SESSION_V2_NOFRONT=1`, same binary) — the numbers in the brief reproduce exactly:

| op | v1 | OCCT | faces | naked | shells | is_solid |
|---|---|---|---|---|---|---|
| cut | 43.1452184 | 12.307399 | 4 | **3** | 2 | 0 |
| common | 26.3900583 | 4.44774457 | 3 | **3** | 2 | 0 |
| fuse | 29.0080514 | 54.7193869 | 4 | **3** | 3 | 0 |

and the v1 cut result shows *both* failure modes at once: the section edge exists as **two copies**
(`RESEDGE i=3 BSpline len=8.04279908 nface=1` and `i=7 BSpline len=8.043342 nface=2` — 5.4e-4 apart,
the same micro-gap as G7) and the face set contains both the cylinder remainder (a fuse piece) and
the cylinder patch (a cut piece).

**Impact:** 3 of the 29 failing cells directly, with the largest volume errors in the matrix
(the brief's "worst structural cell"). Fixing G1+G2 is the only change that can move them: the
splitting layer is already producing the correct faces for this cell.

### 2. G4 (accuracy class) — the 11 improved cells and the residual on every curved cell

Measured with the same binary on `sphere x box` (answered by v1 after the v2 front refused):

| op | ours | OCCT | signed error |
|---|---|---|---|
| cut | 10.9834957 | 10.9955738 | −0.0120781 (−0.11 %) |
| fuse | 74.9834957 | 74.9955738 | −0.0120781 (−0.016 %) |
| common | 35.6167954 | 54.4542756 | **−18.8375 (−34.6 %)** |

The **identical absolute deficit 0.0120781** in cut and fuse is the chord/sagitta error of the
polyline pcurves and polyline 3D curves — the accuracy class, exactly what G4 removes (the brief
measures 54–128× improvements and `box x tor common` 2.25e-4 → 8.70e-12 when the degree is raised).
Face structure is otherwise right: our sphere face is one wrap face with 4 wires and area 31.4304209
against OCCT's 31.4159297 (4.6e-4 relative — again the chord error), and all six discs are exact.

The **common** cell is a different animal and must not be attributed to G4: the deficit 18.8375 is,
to 0.06 %, exactly `2 × 2 × (1/3)·2·7.06858` — i.e. **two of the six planar discs contribute to the
divergence integral with the wrong sign**. Topology is perfect (`7 faces, naked=0, nonmanifold=0,
closed shell, valid=1`), so **the verdict cannot see it**. This is an orientation defect in the v1
assembly, invisible to every metric we currently gate on. (No per-face sign dump exists on the v1
path, so the "exactly two discs" attribution is an inference from the measured deficit, not a direct
observation — but the arithmetic is exact and the alternative explanations, e.g. a missing face, are
excluded by `res_face=7` and `res_area=73.84`.)

**Consequence for ranking:** G4 fixes the ~1e-4 class; the 34 % class needs the orientation work of
G1 plus an orientation-coherence term in the verdict.

### 3. G7 (micro pave block) — 1 cell, currently masked

In v2's `coneR x cyl` the closed section curve is split into **2 pave blocks** where OCCT has 1
(`SECPB k=0 t0=0 t1=6.74997173e-05 len=0.000542923578`), and the cone's lateral image carries **7**
oriented edge occurrences with **two** distinct seam edges each used twice (`nseam=4`), where OCCT
has 4 occurrences and one seam used twice. The excursion has zero area, so the face area is still
right (17.5658497 vs 17.5657752) — the defect is currently invisible in the volume but it is a
non-manifold-in-waiting and it is what stops `res_solid` from being 1 (`nsolid=0, valid=0` on all
three v2 coneR results even though `naked=0, closure=6.3e-08`).

### 4. G3 (seam composition inverted) — **no measured impact yet**, but it is wrong today

The straddle path is exercised only by main_14's synthetic block (`main_14.cpp:1377-1442`), which
**asserts the wrong answer**: `o.faces == 2 && o.merges == 1 && o.composed_wires == 1`. OCCT's answer
for the same configuration is one face more, with the seam fragment materialised
(`sph_box_cut` faces 3+4, `RESEDGE i=4 nface=2`). No matrix cell measures it because the v2 front end
refuses every box-containing case today (9 of my 14 sample runs delegated to v1). It will fire on
every sphere/cylinder/torus × box cell the moment the front end accepts planar operands, and v1's
own history is the warning: merging straddles drove `tor x tor cut` to 8.5029 against 18.7296 and
cost 10 matrix cells their face-count parity (recorded at `nurbssurface_trimmed.cpp:1396-1397`).

### 5. G5 (v1 wrap merge off by default) — impact bounded but unmeasured

The predicate is implemented and correct; it is simply not switched on, and its representation (extra
outer wires) is not what any integrator expects. No cell can be attributed to it without a battery
re-run, which I could not complete. Cheap to measure once: run the 63-cell matrix with and without
`SESSION_SEAM_MERGE=1`.

### 6. G6 (no shell/solid records) — **no measured impact yet**

Both structures we can check come out right without them: `box(4) cut box(2)-inside` gives
`res_solid=1 res_shell=2 res_face=12 vol=56 valid=1` — bit-identical to OCCT's
`RESSOLID i=1 nface=12` + two closed shells; `sph_box_cut` gives `res_solid=6 res_shell=6
res_face=13`, matching OCCT's 6/6/13. The gap is representational (STEP export, downstream
consumers, "is this one solid with a void or two solids?"), not numerical, today.

### 7. Same-domain / two-fence handling — **no measured impact yet**

`sd=0` and `n_sd_pairs=0` in every v2 run I made; no coincident-face cell reaches the selector.
`v2_select_faces` (`brep_v2_solid.cpp:894-909`) is a faithful transcription of BuildBOP `:682-712`,
and the OCCT ground truth for when it does matter is already captured
(`box_box_touch_fuse`: 5 `tangent=1` FF records with `ncurves=0`, 4 common blocks;
`box_box_touch_common`: empty compound, valid, no error).

### 8. The angular key — **no measured impact, and provably none available**

Our metric key and OCCT's raw key select the same edge (proof in §1.2). The only true divergences are
the tie-break policy (`SF_ANGLE_EPS = 2.22e-16` first-wins vs OCCT's `Epsilon(1.)` first-wins with
`dA<=1e-14` pushed last) and secant escalation vs `RefineAngles`. No failing cell reports
`AngleTieUnresolved`. **Do not spend effort here.**

### What is measurably CORRECT in this layer already (so it is not re-litigated)

* v2's splitter is exact on every cell it accepts: `sphere x cylinder@45 cut = 50.3880514` against
  OCCT's 50.3880515 (2e-9 relative), 2 faces, `naked=0`, and the sphere's wrap face carries the seam
  twice; `coneR x cyl` splits both operands into closed shells whose volumes reproduce the operands
  to 9 digits (`16.7551608191`, `42.4115008235`) with per-face areas within 4e-6 of OCCT.
* The wrap rule, the pole rule and the "one-period section is not split" rule are all reproduced by
  v2's walk without any special-casing — they fall out of supplying the seam twice with two pcurves.
* v1 reproduces the cavity case exactly (56.000000, 2 shells) and the 6-solid case exactly.

---

## 5. IMPLEMENTATION ORDER — smallest shippable increment first

**S1. Local image orientation (G1).** Port `IsSplitToReverse` as
`v2sol::v2_split_to_reverse(const BRep& img, int face, const BRep& src, int src_face)` and use it
where `osign` is read at `brep_v2_boolean.cpp:1163-1176`.
*Acceptance:* (i) on `coneR x cyl` the sign of every image equals the sign `v2_outward_signs` gives
on the cells that already pass (`sph_cyl45`), i.e. no regression; (ii) a new invariance test: rotate
both operands by 12 random rigid motions — every image's `reversed` flag must be identical, which
`v2_outward_signs` cannot guarantee because it depends on tessellated volume signs.

**S2. Angle-first IN membership on connexity blocks (G2).** New
`v2sol::v2_fill_in_parts(const V2Topo&, images, other_split)` implementing §1.5 (2): block by shared
edges avoiding the other solid's edges → `v2_get_face_off`-based dihedral verdict → point fallback.
*Acceptance:* the `SFDBG cls` table for `coneR x cyl` reads `k3=Out k4=In k5=Out k6=Out`, and the
three ops return 12.3074 / 4.4477 / 54.7189 within 1e-5 relative. This alone is expected to move all
three cells to OK.

**S3. Micro-block unify (G7).** In the arena's pave-block construction, when a block's parameter
range is below `PConfusion` *or* its two vertices are within `max(tol(v1),tol(v2))`, unify the
vertices instead of emitting the block.
*Acceptance:* `coneR x cyl` v2 trace shows `SEC … npb=1`, one `RESVERT` at the section closure, the
cone seam with exactly 2 pave blocks, and the cone image's wire with 4 occurrences / `nseam=1` —
i.e. byte-comparable to OCCT's `RESFACE i=1` line. `res_solid=1`, `valid=1`.

**S4. Delete the straddle merge (G3).** Remove `compose_across_seams`; re-baseline main_14's straddle
block to the OCCT answer.
*Acceptance:* the straddle block asserts `faces == 3, merges == 0`, the seam fragment edge has 2
trims, `naked == 0`, `is_solid == 1`, and the composed-wire UV-gap assertion is replaced by "both
half-faces lie inside the stored domain". Additionally `sf_periodic_extend` must have zero callers.

**S5. Exact boundary geometry in v1 (G4a).** In `append_face`'s boundary path
(`brep.cpp:4246-4260`), when a run is a whole trim or a parameter sub-range of one, `trim` the stored
pcurve and the stored 3D curve instead of calling `lift_loop`.
*Acceptance:* `box x tor common` ≤ 1e-9 relative; the 11 cells the brief measures as 54–128×
improved stay improved; `box x box` unchanged bit-for-bit.

**S6. Ungate and re-represent the v1 wrap merge (G5).** Default `SESSION_SEAM_MERGE` on; emit the
merged region as ONE wire using the seam edge twice.
*Acceptance:* `sphere x box common` gives 7 faces with the sphere face carrying `nseam=2` in one
wire; the 63-cell battery does not regress (this is the one item that requires the full battery, so
schedule it after S1–S5 so the battery run is worth its cost).

**S7. Shell/solid records (G6).** Add `m_shells`/`m_solids` to `BRep`, filled from
`V2BuilderSolid::areas()`.
*Acceptance:* `box(4) cut box(2)-inside` reports 1 solid / 2 shells through the public API (not just
through a connexity recount), and a STEP round-trip preserves the cavity; `sph_box_cut` reports 6
solids.

**S8. The small items (G8).** WireSplitter fast path; `IsGrowthWire` shortcut; an `FClass2d`-grade
hole test.
*Acceptance:* no cell changes value — these are determinism and cost items, and any change they cause
is a bug in them.

---

## 6. RESIDUAL UNKNOWNS

* **Per-cell attribution across all 29 failing cells.** I measured 2 cells first-hand
  (`coneR x cyl`, `sphere x box`); the remaining 27 are attributed from the brief's census. A single
  filtered `main_7` run per pair (with a per-cell timeout, since `box x tor` cells cost 40–54 s each)
  would close this.
* **Why the v2 front end refuses.** 9 of 14 sample cells delegated to v1, including cases with no box
  operand (coaxial `cone x cylinder`, `cylinder x coneR`). The refusal reason is not in the dump.
  Until that is fixed, v2's correct splitter is unreachable for most of the matrix and every
  splitting-layer improvement in v2 is invisible in the battery.
* **The two inverted discs in `sphere x box common`.** Inferred from an exact arithmetic match, not
  observed; the v1 path has no per-face outward-sign dump.
* **Whether the wrap rule holds for a region that wraps u and is bounded in v while also containing
  a pole** (the `sph_cyl_roty23578` family). The traces show OCCT keeps the two pole lobes separate;
  our v2 has never been run on that case end to end.
