# occt2_topopebrep-lessons

The OLD OCCT boolean generation (Matra Datavision 1993-1999, Jean Yves LEBEY; patched 1998-2002
by `xpu`, `NIZHNY-MZV`, `NIZHNY-PKV`). Sources surveyed at
`C:/brg/compas_occt/external/occt/src/occt/src/ModelingAlgorithms/TKBool/`
packages `TopOpeBRep` (82 files), `TopOpeBRepBuild` (113), `TopOpeBRepDS` (115), `TopOpeBRepTool` (74).

Status in the current tree: **no boolean entry point uses it anymore** (the old
`BRepAlgo_BooleanOperation/Fuse/Cut/Common/Section` classes are deleted). The only remaining
consumers are fillet/chamfer (`TKFillet/ChFi3d/ChFi3d_Builder*.cxx` uses `TopOpeBRepBuild_HBuilder`
+ `TopOpeBRepDS_HDataStructure` for corner-solid construction) and Draw test commands. Booleans were
rewritten as BOPAlgo/BOPDS by the same group that patched this code (NIZHNY-PKV = pave-filler
author) — see `kb/occt_pavefiller-core.md`, `kb/occt_builder-assembly.md` for the successor; this
file covers what the successor kept, what it discarded, and why.

---

## STAGE PIPELINE

Two-phase architecture: **DSFiller** (fill an interference data structure) then **Builder**
(split + assemble from the DS). The builder never re-intersects; every decision reads the DS.

### Phase A — DSFiller (`TopOpeBRep`)

1. **`TopOpeBRep_DSFiller::Insert(S1,S2,HDS)`** (`TopOpeBRep_DSFiller.cxx:428`) =
   `InsertIntersection` + `Complete`.
2. **`InsertIntersection`** (`:443`):
   - `FBOX_Prepare()` / `FC2D_Prepare(S1,S2)` — global bounding-box tool + 2d-curve cache.
   - `TopOpeBRep_ShapeIntersector::RejectedFaces` — box rejection; rejected faces recorded in DS
     (`ChangeMapOfRejectedShapesObj/Tool`) so the builder can pass them through untouched.
   - Scan loop `InitIntersection` / `MoreIntersection` / `NextIntersection`: the ShapeIntersector
     yields intersecting GeomShape couples (S1 SCANNED via `TopOpeBRep_ShapeScanner` boxes, S2
     EXPLORED via `TopOpeBRepTool_ShapeExplorer`). Dispatch by type pair:
     - FACE×FACE → **`TopOpeBRep_FacesFiller::Insert`** (`TopOpeBRep_FacesFiller.cxx`)
     - EDGE×EDGE → **`TopOpeBRep_EdgesFiller::Insert`** (2d intersection in a face's UV)
     - FACE×EDGE / EDGE×FACE → **`TopOpeBRep_FaceEdgeFiller::Insert`**
   - Face-face same-domain protocol: `FillShapesSameDomain` optimistically during scan, then
     `BREP_UnfillSameDomain` (`:282`) when the NEXT couple in scan order is not EDGE×EDGE
     (`islFFsamdom` flag machine, `:585-697`) — see PITFALLS.
   - Post passes: `BREP_sortonparameter` (sort interferences by edge parameter),
     `BREP_correctgbound` (`:173` — repair the "geometry vertex is an edge bound" flag),
     `BREP_mergePDS` (`TopOpeBRep_mergePDS.cxx` — merge coincident DS points across pairs),
     `HDS->AddAncestors`, `FDSCNX_Prepare` (edge→face connexity map, `TopOpeBRepDS_connex.cxx`),
     `FDSSDM_prepare` (same-domain map, `TopOpeBRepDS_samdom.cxx`).
3. **`TopOpeBRep_FacesFiller::Insert`** — one SSI per face pair:
   - `TopOpeBRep_FacesIntersector::Perform` (`TopOpeBRep_FacesIntersector.cxx:226`) wraps
     `IntPatch_Intersection`. Lines typed by `TopOpeBRep_TypeLineCurve`:
     `ANALYTIC / RESTRICTION / WALKING / LINE / CIRCLE / ELLIPSE / PARABOLA / HYPERBOLA`.
   - Each line (`TopOpeBRep_LineInter`) carries **VPoints** (`TopOpeBRep_VPointInter` = wrapped
     `IntPatch_Point` + shape indices + `IntSurf_Transition` on each surface + on-domain arcs +
     keep flag). `VP_Position` / `TopOpeBRep_VPointInterClassifier::VPointPosition` classifies each
     VPoint IN/ON/OUT of each face; un-kept VPoints are dropped (`VP.Keep()` checks in
     `TopOpeBRep_FacesFiller_1.cxx:589,1238`).
   - `ProcessLine` / `ProcessRLine` / `FillLineVPonR` / `FillLine` then **`AddShapesLine`**:
     compute 3d curve + both pcurves, add `TopOpeBRepDS_Curve` and face/curve +
     curve/point interferences to the DS. Restriction lines re-use the existing edge (section edge)
     instead of creating a curve.
4. **`TopOpeBRep_DSFiller::Complete`** (`:723`), in order:
   - `GapFiller` (`TopOpeBRepDS_GapFiller.cxx`) — closes point-identity gaps between interferences,
   - `CompleteDS` (`:754`) — **the patch cascade**: `FUN_ds_PointToVertex`, `FUN_ds_redusamsha`,
     `FUN_ds_PURGEforE9`, `FUN_ds_completeforSE8`, `SE1`, `SE2`, `SE3`, `SE4`, `SE5`, `SE6`,
     `E7`, `SE9`, `complete1dForSESDM` (all in `TopOpeBRepDS_EXPORT.hxx/.cxx`, each dated 1998,
     each fixing one regression family; SE7 is commented out),
   - `Filter` (`TopOpeBRepDS_Filter` — drop redundant/contradictory edge & curve interferences),
   - `Reducer` (`TopOpeBRepDS_Reducer` — "reduce interferences", collapse compatible transitions),
   - `RemoveUnsharedGeometry` (`:817` — unfill same-domain faces that only share a vertex,
     `FUN_shareNOG` tangency test),
   - `Checker` (`:882`) — **empty, NYI**. DS validity was never enforced.

### Phase B — Builder (`TopOpeBRepBuild`)

5. **`TopOpeBRepBuild_Builder::Perform(HDS)`** (`TopOpeBRepBuild_Builder.cxx:116`):
   `Clear` → `BuildVertices` (vertex per kept DS point) → `SplitEvisoONperiodicF` →
   `BuildEdges` (edges per kept DS curve, split at curve-point interferences;
   `TopOpeBRepBuild_BuildEdges.cxx`) → `BuildFaces` → `InitSection` → `SplitSectionEdges`
   (parts ON of section edges, both pcurves attached) → `TopOpeBRepDS_Filter
   ::ProcessFaceInterferences(mySplitON)` → `TopOpeBRepDS_Reducer::ProcessFaceInterferences`.
   `Perform(HDS,S1,S2)` (`:138`) additionally sets `myIsKPart = FindIsKPart()`.
6. **`MergeShapes(S1,TB1,S2,TB2)`** (`TopOpeBRepBuild_Merge.cxx:174`) — TB = state to keep
   (cut = OUT/IN, common = IN/IN, fuse = OUT/OUT via `Opec12/Opec21/Opecom/Opefus`):
   - **KPart shortcut** (`:203`): if `IsKPart()` → `MergeKPart` and return (see PITFALLS #7).
   - `Reverse(TB1,TB2)` — orientation reversal per operand (cut reverses tool faces),
   - `SplitShapes(ex,TB1,TB2,SFS,RevOri)` (`Builder.cxx:1713`) recursion:
     `SplitSolid` (`:1535`) → `SplitFace1/2` (`:1171/:1304`) → `SplitEdge1/2` (`:941/:1083`),
     filling one global `TopOpeBRepBuild_ShellFaceSet SFS`. Splitting an edge = iterate its DS
     point interferences into a `PaveSet` and run `EdgeBuilder`; splitting a face = collect split
     edges of its wires + `AddIntersectionEdges` (`Builder.cxx:150` — new edges from DS curves with
     orientation from `TopOpeBRepDS_CurveIterator::Orientation(ToBuild)`) into a `WireEdgeSet` and
     run `FaceBuilder`; the `GSplitFace`/`GFillFaceWES`/`GFillCurveTopologyWES` ladder in
     `TopOpeBRepBuild_Grid.cxx/GridFF.cxx/fctwes.cxx/ffwesk.cxx` is the same logic driven by the
     `GTopo` state matrix.
7. **Loop assembly** (shared machine for all dimensions — `TopOpeBRepBuild_AreaBuilder`):
   - 1D: `PaveSet` + `PaveClassifier` → `EdgeBuilder` → `GPVSMakeEdges`/`GEDBUMakeEdges`
   - 2D: `WireEdgeSet` + `WireEdgeClassifier` → `FaceBuilder` → `GWESMakeFaces`/`GFABUMakeFaces`
   - 3D: `ShellFaceSet` + `ShellFaceClassifier` → `SolidBuilder` → `GSFSMakeSolids`
   - Under all three: `TopOpeBRepBuild_ShapeSet` (elements + start elements + neighbour map keyed
     by connecting subshape type), `BlockBuilder` (connexity flood into blocks),
     `AreaBuilder::CompareLoopWithListOfLoop` (`AreaBuilder.cxx` — classify each Loop
     IN/OUT/ON against accumulated loops to nest wires-in-faces / shells-in-solids),
     `Atomize` (UNKNOWN state → raise or force a default).
8. **Post**: `RegularizeFaces/RegularizeSolids` (`FREGU.cxx/SREGU.cxx` →
   `TopOpeBRepTool_REGUW/REGUS` — re-split result faces/shells whose wires touch at more than one
   connexity), `CorrectFace2d`, `FuseFace`; section access `SectionCurves`/`SectionEdges`/`Section`
   (`TopOpeBRepBuild_Section.cxx`); history via `TopOpeBRepBuild_HBuilder`
   (`EdgeCurveAncestors`, `EdgeSectionAncestors`, `NewVertex/NewEdges/NewFaces`, `IsSplit/Splits`,
   `IsMerged/Merged`).
9. **Builder1 override path** (`TopOpeBRepBuild_Builder1*.cxx`, Maxim ZVEREV 1999 + NIZHNY 2000) —
   the transitional rewrite, gated by mutable global `GLOBAL_USE_NEW_BUILDER`
   (`Builder1.cxx:~55`: "new algo can not be used in LocOpe and Mechanical Features... that's why
   we use new algo only in BRepAlgoAPI_BooleanOperation"):
   - **`PerformShapeWithStates`** (`Builder1_1.cxx:183,363`) — precompute IN/OUT/ON parts of EVERY
     subshape of each operand w.r.t. the other, stored as `TopOpeBRepDS_ShapeWithState`
     (partIn/partOut/partOn + state + isSplitted) in the DS; `StatusEdgesToSplit` (`:556`);
     classification helper `ClassifyEdgeToSolidByOnePoint` (`:1117`).
   - `GFillFaceSameDomSFS/GFillFaceNotSameDomSFS` etc. (`Builder1.cxx:303-1150`) — separate
     same-domain vs not-same-domain fill paths; `PerformONParts` (`:1151`),
     `PerformPieceIn2D` (`:1415`), `PerformPieceOn2D` (`:1531`), `TwoPiecesON` (`:1656`),
     `IsSame2d` (`:2115`), `OrientateEdgeOnFace` (`:2227`).
   - This is the architectural bridge to BOPAlgo: precomputed states = the future BOPDS
     `FaceInfo/state cache`; the ON-parts machinery = the future common blocks.

---

## DATA STRUCTURES

- **`TopOpeBRepDS_DataStructure`** (`TopOpeBRepDS_DataStructure.hxx`): indexed pools of new
  geometry — `TopOpeBRepDS_Surface/Curve/Point` (each with tolerance and keep flag, curves carry
  both pcurves and the two `SurfaceCurveInterference` links via `CurveData`) — plus an indexed
  shape map `TopoDS_Shape → TopOpeBRepDS_ShapeData`. Per shape: interference list, same-domain
  list + `SameDomainRef` (representative index) + `SameDomainOri` (`Config`) + `SameDomainInd` +
  `AncestorRank` (operand 1 or 2), keep flag. Also: section-edge pool (`AddSectionEdge`),
  rejected-shape maps, `ShapeWithState` maps (Builder1), `Isfafa` flag (all-faces-tangent mode).
- **Interference hierarchy**: `TopOpeBRepDS_Interference` = { `Transition`, `SupportType/Support`
  (Kind+index of carrier), `GeometryType/Geometry` (Kind+index of cut geometry) }. Subclasses:
  `CurvePointInterference` (+parameter), `ShapeShapeInterference` (+`GBound` bool: geometry vertex
  is a bound of the support edge), `EdgeVertexInterference`, `FaceEdgeInterference`,
  `SurfaceCurveInterference` (+pcurve), `SolidSurfaceInterference`.
  `TopOpeBRepDS_Kind`: POINT, CURVE, SURFACE, VERTEX, EDGE, WIRE, FACE, SHELL, SOLID, …
- **`TopOpeBRepDS_Transition`** (`TopOpeBRepDS_Transition.hxx`): `(StateBefore, StateAfter)` ∈
  {IN,OUT,ON,UNKNOWN}² + `ShapeBefore/After` + `IndexBefore/After` (which operand shape defines
  the state). `Orientation(S)`: leave S → REVERSED, enter S → FORWARD, stay in → INTERNAL,
  stay out → EXTERNAL. This is the algebra that turns interferences into oriented boundary.
- **`TopOpeBRepDS_Config`**: `UNSHGEOMETRY / SAMEORIENTED / DIFFORIENTED` — same-domain relative
  orientation, the disambiguator for ON-classification (our SD/DIFF cell logic).
- **`TopOpeBRepBuild_GTopo`** (`GTopo.hxx`): 3×3 boolean matrix `mycases[state1][state2]` over
  IN/ON/OUT — "keep the piece whose (state vs operand1, state vs operand2) cell is true" — plus
  shape types, `Config1/Config2`, reverse flags. Built per op by `TopOpeBRepBuild_GTool`
  (`GFusSame/GCutSame/GComSame/GFusUnsh/GCutUnsh/GComUnsh/GFusDiff/GCutDiff/GComDiff` — one
  matrix per op × same/unshared/diff-oriented same-domain config). The whole boolean-op semantics
  is these tables.
- **Intersection carriers**: `TopOpeBRep_LineInter` (typed line + VPoint array + restriction
  `Arc()`), `TopOpeBRep_VPointInter` (see above), `TopOpeBRep_Point2d` + `TopOpeBRep_P2Dstatus`
  (EE 2d points: `P2DUNK/P2DINT/P2DSGF/P2DSGL/P2DNEW`), `TopOpeBRep_Bipoint`.
- **Builder result maps**: `mySplitIN/ON/OUT` and `myMergedIN/ON/OUT`
  (`shape → TopOpeBRepDS_ListOfShapeOn1State`, split flag included), `myNewVertices` (point index →
  vertex), `myNewEdges` (curve index → edges), `myNewFaces` (surface index → faces).
- **Set/assembly**: `ShapeSet` (StartElements = seeds, neighbour map through subshape),
  `Pave` (vertex + parameter + orientation as a Loop), `PaveSet/PaveClassifier` (V1 inside V2 iff
  on the kept side of V2 along the curve), `WireEdgeSet/WireEdgeClassifier`,
  `ShellFaceSet/ShellFaceClassifier`, `Loop/LoopSet/LoopClassifier`, `BlockBuilder/BlockIterator`,
  `AreaBuilder` (+ `Area1d/2d/3dBuilder`, `FaceAreaBuilder`, `SolidAreaBuilder`).
- **KPart result codes** (`TopOpeBRepBuild_kpresu.hxx`): `RESUNDEF -100, RESNEWSHA2 -12,
  RESNEWSHA1 -11, RESNEWCOM -3, RESNEWSOL -2, RESNEWSHE -1, RESNULL 0, RESSHAPE1 1, RESSHAPE2 2,
  RESSHAPE12 3, RESFACE1 11, RESFACE2 12`; shell-classification codes `SHEUNDEF/-100 … SHEGARDTOUS/6`.

---

## CONSTANTS & TOLERANCES (exact values)

- `GLOBAL_tolFF = 1.e-7` default, mutable global; overwritten per face pair with
  `max(myTol1,myTol2)` (`TopOpeBRep_FacesIntersector.cxx:37,212`).
- `FTOL_FaceTolerances` (`TopOpeBRepTool_tol.cxx`): pair tolerance `myTol1 = myTol2 =
  tolF1 + tolF2` (sum of the two face tolerances); callers cap it:
  `myTol1 = (myTol1 > 1.e-4) ? 1.e-4 : myTol1` (both FF 3d, `FacesIntersector.cxx:206-207`,
  and EE 2d, `EdgesIntersector.cxx:145-146`). **Absolute cap 1e-4 regardless of model scale.**
- Walking parameters: `Deflection = 0.01 * dx` (dx = max bbox span of the two faces, capped at
  `1000000.0`), then clamped to `[1e-3, 0.1]`; `MaxUV = 0.01` clamped to `[1e-3, 0.01]`
  (`TopOpeBRepTool_tol.cxx`). Unset boxes default to unit box `(0,0,0)-(1,1,1)`.
- `PAR_T = 0.43213918` — "parameter division number as 10*e^(-PI)" (`Builder1.cxx:49`); the
  parameter at which `ClassifyEdgeToFaceByOnePoint` / `ClassifyEdgeToSolidByOnePoint` sample an
  edge for its ONE classification point (irrational-ish to dodge symmetric midpoint coincidences).
- `TopOpeBRep_EdgesIntersector::ComputeSameDomain` (`EdgesIntersector.cxx:708`): edges same-domain
  iff a 2d overlap segment exists AND same 2d curve type AND (type==Line → true; type==Circle →
  `|r1-r2| < Precision::Confusion()` and 3d center distance `<= tolE1+tolE2`, with the comment
  `"tolerance a revoir"` = tolerance to revisit; any other type → **false, NYI**).
- `FUN_shareNOG` tangency test: `tola = Precision::Angular()` (1e-12) on
  `|1-|dot(tangents)||`; normal-side test `dot(xxF1,xxF2) > 0` ⇒ shared domain
  (`TopOpeBRep_DSFiller.cxx:304-424`).
- Curve-on-arc projection: `Geom2dInt_TheProjPCurOfGInter::FindParameter(..., 1.e-7)`
  (`FacesIntersector.cxx:733+`).
- `EdgesIntersector` tolerances initialized to `0.` with commented-out
  `Precision::PConfusion()/PIntersection()` (`EdgesIntersector.cxx:76-77`).

---

## INVARIANTS

1. **Fill-then-build with a hard wall**: after `DSFiller::Complete` the DS is frozen; the Builder
   only reads it. All geometry (curves, points, pcurves) exists before any topology is assembled.
2. **Transition ⇒ orientation**: an interference's `(before,after)` states against the kept state
   determine the split edge/face orientation in the result via `Transition::Orientation`
   (FORWARD=enter, REVERSED=leave, INTERNAL/EXTERNAL=stay). One algebra reused in 1D/2D/3D.
3. **Same-domain graph consistency**: every same-domain family has one `SameDomainRef`; interference
   geometry indices are rewritten onto the ref vertex (`BREP_correctgbound` second loop) so all
   pairs speak about the same representative.
4. **`GBound` correctness**: a `ShapeShapeInterference` must record whether its geometry vertex is
   a boundary vertex of the support edge; wrong GBound flips pave semantics (bound paves cannot
   split). Enforced by a *repair* pass — i.e. the invariant was violated at creation.
5. **Section edges are first-class**: `SplitSectionEdges` runs before merging; every ON part of a
   section edge carries pcurves on BOTH faces (`Builder::SSE` throws
   `"SSE EG without C3D"` / `"SSE EG without PC on FS"` if not — construction-time asserts).
6. **Loop closure**: `AreaBuilder` consumes every start element; classification of a loop against
   the area's existing loops must return a definite state — `Atomize` converts UNKNOWN into either
   an exception (`myUNKNOWNRaise=true`) or a forced default. Result shells/wires are then
   regularized (REGUW/REGUS) so that no wire/shell has multiple connexity components.
7. **Rejected shapes bypass**: faces rejected by box tests are recorded and passed through
   unmodified — the general machinery never touches non-interfering geometry.

---

## PITFALLS (negative knowledge — mistakes not to repeat)

1. **Post-hoc patch cascade instead of source-of-truth fixes.** `CompleteDS` is 12+ ordered passes
   (`FUN_ds_completeforSE1..SE9/E7/PURGEforE9/redusamsha/complete1dForSESDM`), each added for one
   regression (dated comments `xpu160398`, `xpu020998`, …), each dependent on the previous pass's
   output. 79 files in the two intersection/build packages carry `xpu`/`NIZHNY`/`NYI` markers;
   `FacesFiller` alone cites 46 customer test IDs (cto/PRO/BUC). When interference transitions are
   created wrong, no sequence of DS repair passes converges. **Rule for us: a keep-verdict or
   transition bug is fixed where it is computed (build_section_scaffold keep-verdict), never by
   appending a repair pass after `Complete`.**
2. **Single-sample classification.** `ClassifyEdgeToFaceByOnePoint` / `...ToSolidByOnePoint`
   sample one point at magic `PAR_T=0.43213918`. Tangent/grazing/ON-ambiguous pieces misclassify
   with no recourse. Our winding+radial classification with quorum (SESSION_ON_QUORUM work) is the
   correct replacement; never regress to one probe per piece.
3. **Mutable global state.** `GLOBAL_tolFF`, `GLOBAL_USE_NEW_BUILDER`, static file-scope maps
   `mySDEdgeMap`, `processedEdges`, `theUsedVertexMap`, `theUnkStateVer` (`Builder1*.cxx`) —
   non-reentrant, phases coupled through hidden channels, behavior switches by global bool.
4. **Contradictory tolerance model.** Pair tolerance = `tolF1+tolF2` **capped at absolute 1e-4**;
   walking deflection scales with bbox (`0.01*span`) but UV step is absolute (`0.01`); EE
   tolerances zero-initialized; no per-entity tolerance growth (nothing like BOPAlgo
   `UpdateVertex`), no fuzzy option. Same-domain circle test uses `Precision::Confusion()` on
   radii with an in-code admission it's wrong (`"tolerance a revoir"`). Scale-dependence bugs are
   structural, not incidental. Extend `kb/occt_tolerance-model.md` rules instead.
5. **Scan-order-dependent same-domain protocol.** FF same-domain is *filled optimistically* during
   the scan and *unfilled* when the following couple is not EDGE×EDGE (`islFFsamdom` flag in
   `InsertIntersection`) — semantics depend on iterator ordering of intersection couples. Detect
   same-domain from geometry (coincidence of split results), never from event order.
6. **Same-domain detection NYI for general curves** (`ComputeSameDomain`: only Line and Circle 2d
   types; everything else silently "not same-domain") — tangent NURBS overlaps fall through to the
   general path and produce sliver geometry. Silent NYI fallbacks are worse than errors.
7. **KPart special-case forest.** `FindIsKPart` dispatches 5 hard-coded particular cases
   (`iskole`=glued-on-face, `iskoletge`=tangent-glued, `isdisj`=disjoint, `isfafa`=all-faces-same-
   domain, `issoso`=solid/solid trivial) each with its own merge path (`MergeKPartiskole…`)
   bypassing the general algorithm, with French mnemonic result codes. Each special path is a
   divergence class (our tor×tor recognize_solid hijack was exactly this failure mode). Special
   paths must be result-identical to the general path and covered by the same gates.
8. **Validity checking never implemented.** `DSFiller::Checker` is an empty stub (`// NYI`) — the
   DS was never validated, so corrupt interference sets flowed silently into the builder. Our
   equivalent: keep gate checks (naked-edge count, closure) INSIDE the pipeline, not only in tests.
9. **History grafted on afterwards.** `HBuilder` ancestor queries reverse-engineer the maps; BOPAlgo
   made images/origins first-class from the start (`kb/occt_history-gf-scale.md`).
10. **Two algorithms in one class hierarchy behind a bool.** `Builder1` ("extension … dedicated to
    avoid bugs in Rebuilding Result") shipped alongside the old path because LocOpe/features
    couldn't use it; `GLOBAL_USE_NEW_BUILDER` selected per call site. The half-migration froze both
    paths and the whole generation was eventually discarded wholesale.
11. **Periodic-surface handling as repair.** UV-disconnected pcurves on periodic faces are fixed
    after the fact by translating pcurves by the period (`TopOpeBRepTool_CORRISO`,
    `SplitEvisoONperiodicF`, `CorrectFace2d`) rather than splitting at the seam up front — the same
    lesson as our seam-decomposition work (`reference_seam_split_cuts`): handle seams in the
    splitter, not with post-hoc pcurve surgery.

**Why it was replaced (synthesis of in-tree evidence).** The three core bets failed together:
(a) transition-coded interferences can be repaired into consistency after filling (spawned the
completeforSE cascade), (b) classification can be deferred to assembly time with single-point
probes (spawned KParts and Builder1's precomputed states), (c) tolerance is a per-pair capped
scalar (spawned scale-dependent failures). BOPAlgo — written by the same NIZHNY-PKV who patched
this code — inverted all three: dimension-ordered pave filling with per-entity tolerance updates,
pave blocks/common blocks as the single splitting currency computed *before* assembly, and one
general-fuse path with op selection by cell state instead of special cases.

**What survived into BOPAlgo** (map to `kb/occt_pavefiller-core.md` / `kb/occt_builder-assembly.md`):
- DS-centric two-phase design: `TopOpeBRepDS_DataStructure` → `BOPDS_DS`.
- Pave concept: `TopOpeBRepBuild_Pave/PaveSet/PaveClassifier` → `BOPDS_Pave/PaveBlock`.
- Builder1's `ShapeWithState` precomputation → BOPDS state caching / `FaceInfo` On/In/Sc sets.
- Section edges as first-class citizens → BOPDS section edges + `BOPAlgo_Section`.
- Loop assembly by connexity blocks + nesting classifier: `AreaBuilder/BlockBuilder` →
  `BOPAlgo_BuilderFace/BuilderSolid` (WireSplitter/ShellSplitter) — the one piece kept nearly as-is.
- Same-domain with orientation config (`SAMEORIENTED/DIFFORIENTED`) → `myShapesSD` + SD face logic.
- Box rejection of non-interfering faces → `BOPDS_Iterator` + pass-through images.

---

## PORT MAP (OCCT mechanism → our anchor → action)

| OCCT (old gen) | Our anchor | Action |
|---|---|---|
| `DSFiller::InsertIntersection` scan + typed fillers (FF/EE/FE) | `brep_section.cpp build_section_scaffold` (SSI chains, paves) | **adopt (have)** — one fill pass writes the whole section DS; never add repair passes after it (Pitfall 1). |
| `Transition` (before/after state) algebra → edge orientation | winding+radial classification in `brep.cpp` | **replace** — we classify pieces after splitting (flux/winding quorum); do not port transition bookkeeping, it is the failure core; keep `Transition::Orientation`'s 4-case table only as reference semantics when orienting lifted loops. |
| `GTopo` 3×3 IN/ON/OUT keep matrix per op × `Config` (SAMEORIENTED/DIFFORIENTED) | `combine` keep tables (cut/common/fuse) + same-domain cells | **adopt** — make our keep table explicit per op including the ON row split by relative orientation; this is the cleanest artifact of the old design. |
| `PaveSet/PaveClassifier` (1D pave splitting) | scaffold paves + valence-1 bridge + welded vertices | **adopt (have)** — identical concept; keep GBound rule: a pave at an existing edge bound must never split (Pitfall/Invariant 4). |
| `ShapeSet/BlockBuilder/AreaBuilder` connexity blocks + loop nesting | `split_with` UV arrangement + shared-chain run lifting; `combine` exact weld + tube merge | **adopt partial** — our arrangement flood = BlockBuilder; add AreaBuilder's explicit UNKNOWN policy: NK-RESCUE stays loud/logged (their `Atomize` silently defaulted when `myUNKNOWNRaise=false`). |
| Builder1 `PerformShapeWithStates` (precomputed IN/ON/OUT parts per subshape) | keep-verdict caching / symmetric-coverage flood | **adopt** — compute verdicts once, share across both operands and all ops (matches our shared-closure bridge march); never re-classify the same piece two ways. |
| `FTOL_FaceTolerances` (sum, cap 1e-4, deflection 0.01·span) | our tol ladder (join_tol, whole-seg alias key 1e-2, forced_node_eps) | **replace** — per-entity growing tolerances (see `kb/occt_tolerance-model.md`); never introduce an absolute cap like 1e-4; keep walking step relative to span but clamp both ends explicitly like their `[1e-3,0.1]`. |
| Scan-order same-domain fill/unfill (`islFFsamdom`) | same-domain imprint (`extra_cuts`), SEG-UNIFY dedup | **new-build** — detect same-domain from split-result coincidence (BOPAlgo `myShapesSD` style), never from event ordering. |
| KPart dispatch (`FindIsKPart` → 5 special merges) | `recognize_solid` analytic fast paths | **caution** — keep, but every special path must pass the same gates as the general path (tor×tor hijack was our KPart moment); prefer rejection-only shortcuts (their `KPisdisj` = disjoint assembly is safe, our xor does this). |
| `BREP_correctgbound` repair pass | welded-vertex / cut-node bookkeeping in scaffold | **adopt invariant, not the pass** — set "pave is at an existing vertex" correctly at creation; a repair pass over the finished DS is the smell to avoid. |
| `CORRISO`/`SplitEvisoONperiodicF` periodic pcurve repair | seam decomposition in `brep_section` (chains split at periodic-seam jumps) | **adopt (have)** — split at seams up front; do not port pcurve-translation repair. |
| `Checker` (NYI stub) | closed-solid / naked-edge gates in pipeline | **new-build** — in-pipeline DS validation with hard failure, the piece they never wrote. |
| `HBuilder` ancestor queries | (missing) history in our kernel | **new-build later** — follow `kb/occt_history-gf-scale.md` (images/origins first-class), not HBuilder's retrofit. |
| `RegularizeFaces/REGUW` multi-connexity re-split | `combine` NK-RESCUE + micro-piece filter | **adopt idea** — final faces/shells must be single-connexity; run as a structured post-step with its own gate, not silent. |
