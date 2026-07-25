# ff-posttreat-samedomain

OCCT implementation spec: face-face intersection -> section edges -> cross-pair unification (PostTreatFF) -> same-domain detection. Extracted from OCCT sources at `C:/brg/compas_occt/external/occt/src/occt/src/ModelingAlgorithms/TKBO/` (BOPAlgo, IntTools, BOPTools). Line refs are approximate against that tree.

---

## STAGE PIPELINE

Ordered by execution inside `BOPAlgo_PaveFiller::PerformInternal` (FF stages only). Stage N consumes state produced by stage N-1 via `BOPDS_DS` (the shared interference DS).

### Stage 0 — PerformFF: run one SSI per overlapping face pair
`BOPAlgo_PaveFiller_6.cxx`, `BOPAlgo_PaveFiller::PerformFF` (~L285-622), helper class `BOPAlgo_FaceFace` (~L138-277), `ToleranceFF` (~L3922), `CheckPlanes` (~L3639), `GetEFPnts` (~L2608), `IsClosedFF`/`IsPlaneFF` (~L84-134), `BOPAlgo_Tools::TrsfToPoint` (`BOPAlgo_Tools.cxx` ~L1912).

- **When**: after V/V, V/E, E/E, V/F, E/F interference stages (their results are inputs here).
- Iterate FF pairs from `myIterator` (bbox-filtered). `UpdateFaceInfoOn/In` for every touched face first.
- **EE-vertex map**: collect all E/E interferences that produced a new vertex into `aEEMap : (nE1,nE2) -> list<newVertexIdx>`.
- **Plane x plane cheap gate** `CheckPlanes` (~L3639): the pair is intersected only if the two faces share >= 2 vertices in each other's VerticesIn/On sets; otherwise an empty `BOPDS_InterfFF` is appended (bounded coplanar-crossing planes always cross via boundary, producing shared vertices earlier).
- **Seam-shift correction** (~L400-486): for non-plane pairs, walk edge pairs (E1 of F1, E2 of F2) where at least one edge is closed on its surface (`IsClosedFF`: has a curve-on-closed-surface representation, or closed on triangulation). If that edge pair has an EE new vertex: project the vertex point exactly onto both edges (`ProjPC`); if the distance between the two on-edge points exceeds the vertex tolerance, translate the face owning the *closed* edge by exactly that vector (`aFShifted->Move`), and remember `aShiftValue` = distance. Purpose (comment in source): the SSI curves must reach the boundary; a seam misalignment otherwise truncates them.
- **Pair tolerance**: `aTolFF = max(aShiftValue, ToleranceFF(BAS1,BAS2))`; `ToleranceFF` = `max(tolF1,tolF2)`, floored at **5.e-6** if either surface is not one of {Plane, Cylinder, Cone, Sphere, Torus}.
- **Seed points**: `GetEFPnts` (~L2608) converts every E/F interference vertex whose edge and face both belong to this pair into an `IntSurf_PntOn2S` (UV on both faces, via existing pcurve where available else two surface projections) — passed to the intersector with `SetList`. Exact point transfer between interference stages.
- **Origin centering** `BOPAlgo_FaceFace::Perform` (~L202): `TrsfToPoint(box1,box2,trsf)` — if the union box center is farther than **1e5** from origin and boxSize/dist < 1/1e5, both faces are moved to the origin before intersection; curves/points are transformed back afterwards (`ApplyTrsf`). Numeric conditioning only.
- All pairs run in parallel (`BOPTools_Parallel::Perform`), then results are harvested:
  - failed/errored pair -> empty interf + warning;
  - `PrepareLines3D(bSplitCurve=false)`, `ApplyTrsf`;
  - each `IntTools_Curve` passes `IntTools_Tools::CheckCurve` (validity + bnd box), box enlarged by `aTolFF + max vertex tolerance of both faces` ("bounding box used in sharing"), stored as `BOPDS_Curve` with `tol = max(IC.Tolerance(), aTolFF)`;
  - intersection points stored as `BOPDS_Point`.
- **Output**: `myDS->InterfFF()` array; each entry = (nF1,nF2, TangentFaces flag, Curves[], Points[]).

### Stage 1 — IntTools_FaceFace: geometric SSI + curve/pcurve construction
`IntTools_FaceFace.cxx`: `Perform` (~L330-609), `MakeCurve` (~L695-1846), `ComputeTolReached3d` (~L613-691), statics: `PerformPlanes` (~L2426), `ClassifyLin2d` (~L2574), `CorrectSurfaceBoundaries` (~L2017), `CorrectPlaneBoundaries` (~L3093), `isTreatAnalityc` (~L249), `ApproxWithPCurves` (~L2357), `ApproxParameters` (~L2736), `Tolerances` (~L2787), `SortTypes`/`IndexType` (~L2826/2844), `ParameterOutOfBoundary` (~L2213), `IsCurveValid` (~L2308), `CheckPCurve` (~L3010), `FindMaxDistance` (~L2899-2988).

- **Canonical order**: `SortTypes` orders the pair by surface-type rank Plane(0) < Cylinder(1) < Cone(2) < Sphere(3) < Torus(4) < Bezier(5) < BSpline(6) < Revolution(7) < Extrusion(8) < Offset(9) < Other(10); on swap, seed UVs and pcurve-request flags are swapped too, and pcurves are swapped back at the end.
- **Tolerance model**: `myTolF1 = tolF1 + fuzz/2`, `myTolF2 = tolF2 + fuzz/2`, `myTol = myTolF1+myTolF2`; `TolArc = TolTang = myTol`.
- **Plane x plane** -> `PerformPlanes` (~L2426): `IntAna_QuadQuadGeo(pln1,pln2, TolAng=1e-8, TolTang)`; `IntAna_Same` -> `TangentFaces=true` (the same-domain trigger for planes). Line projected into both UV charts, `ClassifyLin2d` clips against each face's (10%-expanded) UV box, keep common param interval `[max(P11,P21), min(P12,P22)]` if longer than TolTang. Curve tol = `max(TolF1,TolF2)`; **tangential tolerance** = `sqrt(aDt^2 + TolF1^2)` with `aDt = IntTools_Tools::ComputeIntRange(TolF1,TolF2,angle)` — the only exactly-computed tangential tolerance; all other cases use max face tol.
- **Domain conditioning**: plane charts expanded 10% each side (`CorrectPlaneBoundaries`); other surfaces expanded by `myTol*2` in non-periodic directions for {Bezier,BSpline,Extrusion,Revolution,Cylinder} and *clamped* in periodic directions by the box of the seam-edge pcurves (`CorrectSurfaceBoundaries` ~L2116-2205: only if all closed edges are 2D lines parallel to an axis).
- **Special tolerances** (`Tolerances` ~L2787): cylinder x torus with `|Rcyl - RminorTorus| < Precision::Confusion()` -> `TolTang *= 0.1` (near-tangent family). Same condition drops max approx degree to 6 (`ApproxParameters`); cyl x cyl gets 1 approx iteration.
- Walker step: `UVMaxStep = IntPatch_Intersection::DefineUVMaxStep(...)`, `Deflection = 0.1` (0.01 for BSpline x BSpline).
- Analytic torus pairs discard seed points (~L492-500). `isTreatAnalityc` (~L249): bounded plane x cylinder producing an ellipse with `major > 1e5 * minor` -> force walker instead of analytic (huge-sliver ellipse guard).
- `myIntersector.Perform(HS1,dom1,HS2,dom2,TolArc,TolTang,listOfPnts,isGeomInt)`; `TangentFaces()` from intersector = same-domain signal for non-planes.
- **MakeCurve per line** (~L695):
  - `IntPatch_Restriction` lines bypass the line constructor (`bAvoidLineConstructor=true`); handled by `GeomInt_IntSS::TreatRLine` + `TrimILineOnSurfBoundaries`, pcurves trimmed and box-checked against both UV charts (~L1742-1841).
  - `Lin/Parabola/Hyperbola` (~L781): trim to line-constructor parts; pcurves via `GeomInt_IntSS::BuildPCurves(fprm,lprm,Tolpc,S,newc,C2d)`; infinite parts kept only if a probe point (offset **dT=100**) classifies not-OUT in both domains (`Parameters` = surface projection, `dom->Classify`).
  - `Circle/Ellipse` (~L906): parts crossing parameter 0 are normalized into [0,2pi] and **split into two intervals**; near-degenerate remainders (< Tolpc) checked by endpoint distance > myTol and nudged off boundary-ON states by `ParameterOutOfBoundary` (step `0.1*myTol`, <= 11 iterations); full-circle fallback samples **18 points (2pi/17)** and keeps the curve if any sample classifies in both domains.
  - `Walking` (~L1175): `IntTools_WLineTool::DecompositionOfWLine` splits WLines crossing seams/degenerate points of closed surfaces (bIsDecomposited; each piece re-approximated with `aTolC=Precision::Confusion()` base tol). Approximation `GeomInt_WLApprox`: degrees 4..8, 30 segments, parametrization type from `ApproxInt_KnotTools::DefineParType`. If one surface is a plane: approximate in that plane's 2D and lift poles by `ElSLib::Value` (exact-on-plane 3D curve). BSpline x BSpline may reject surfaces for approx (`NotUseSurfacesForApprox`). `myHS1==myHS2` (self-intersection) forces no-surface mode.
  - **Approx failure fallback**: raw `GeomInt_IntSS::MakeBSpline` polyline-interpolating BSpline through the WLine points + `MakeBSpline2d` for pcurves (degree-1-ish exact lift, no smoothing).
  - **Pcurve QA loop**: `GeomLib_CheckBSplineCurve/2d` `FixTangent(TOLCHECK=1e-7, TOLANGCHECK=1e-6)`; `IsCurveValid` (2D self-intersection, tol 1e-10) and `CheckPCurve` (23 samples per CN interval must stay within UV bounds + 1% margin, with period renormalization) — failure triggers `goto reapprox` with `myTolApprox=1e-5` and/or no-surfaces mode; if approximated pcurves are out of bounds but raw MakeBSpline pcurves are in bounds, the raw version wins (~L1595-1625).
- **ComputeTolReached3d** (~L613): per curve, `tolC = max(tolC, max_t |C3D(t) - S_i(C2Di(t))|)` for both pcurves (`IntTools_Tools::ComputeTolerance`), or `FindMaxDistance` (golden-section 0.618 over 11 seed intervals, eps=1e-4*dt, distance = projection LowerDistance) where a pcurve is absent. Tangential tolerance floored at `max(myTolF1,myTolF2)`.
- **Output**: `mySeqOfCurve` of `IntTools_Curve {C3D, C2D1, C2D2, Tolerance, TangentialTolerance}` + valid intersection points (each checked `IsPointInOnFace` on both faces).

### Stage 2 — MakeBlocks: paving section curves into pave blocks / section edge construction
`BOPAlgo_PaveFiller_6.cxx`, `MakeBlocks` (~L649-1137) + helpers `PutPavesOnCurve` (~L2372), `PutPaveOnCurve` (~L2959), `ExtendedTolerance` (~L2542), `FilterPavesOnCurves` (~L2437), `PutStickPavesOnCurve` (~L2748), `PutEFPavesOnCurve` (~L2692), `GetStickVertices` (~L2847), `GetFullShapeMap` (~L2909), `RemoveUsedVertices` (~L2928), `PutBoundPaveOnCurve` (~L2308) + `getBoundPaves` (~L2255), `PutClosingPaveOnCurve` (~L3500), `IsExistingVertex` (~L1950), `IsExistingPaveBlock` x2 (~L1988, ~L2047), `ProcessExistingPaveBlocks` x2 (~L3072, ~L3171), `PreparePostTreatFF` (~L3609), `RemoveMicroSectionEdges` (~L4308), `MakeSDVerticesFF` (~L1141), `UpdateSavedTolerance` (~L629); `BOPTools_AlgoTools::MakeEdge` (`BOPTools_AlgoTools.cxx` ~L1729), `MakeSectEdge` (`_2.cxx` ~L102), `MakePCurve` (~L1657).

Per FF interference `i` (loop can grow: see recheck queue):

1. **Context collection**: `myDS->SubShapesOnIn(nF1,nF2, aMVOnIn, aMVCommon, aMPBOnIn, aMPBCommon)` — vertices and pave blocks lying ON/IN either face (Common = on/in both); `SharedEdges(nF1,nF2) -> aLSE`. `aTolFF = max(tolF1,tolF2)` (current face tolerances, not the SSI one).
2. **Points**: `IsExistingVertex(P, aTolFF, aMVOnIn)` — bbox prefilter then `ComputeVV(V, P, tolCheck = aTolFF + fuzzy)` where `ComputeVV` adds `tolV1 + Precision::Confusion()`; if no existing vertex, `MakeNewVertex(P, aTolFF)` and register in `aMSCPB` with (interf idx, point idx).
3. **Pave placement on each curve** (order matters):
   - `GetStickVertices`: every *new* vertex created by VV/VE/EE/VF/EF interferences whose two parents both belong to the union of subshapes of nF1 and nF2 (`aMI`); EF subset kept separately (`aMVEF`). SD-resolved (`HasShapeSD`).
   - `PutPavesOnCurve`: EF vertices first (`iCheckExtend=2`), then ON/IN vertices — non-common vertices must be new shapes and pass curve-bbox filter (comment cites test `bugs modalg_6 bug26789_1`: avoid extending vertex tolerance while putting). `iCheckExtend=1`.
   - `PutPaveOnCurve` (~L2959): `IsVertexOnLine(V, tolV, IC, tolR3D + fuzzy) -> aT`. On failure and `iCheckExtend`, `ExtendedTolerance` (~L2542) re-tries with tolerance grown to the max distance from the vertex to the endpoints of the originating EE/EF common-part range (type 1: EE only; type 2: EF only); on success the effective tolV becomes the actual distance point-curve. Then:
     - if the carrier pave block already contains parameter `aT` within `aPTol = GAC.Resolution(max(tolR3D,tolV))` -> record vertex into SD group `aDMVLV[nVUsed] += nV` (deferred fuse), remember original tolerances in `aMVTol`;
     - else append ext pave `(nV, aT)` and, if needed, grow the vertex tolerance to `dist + DTolerance() (=1e-12)` (bnd box updated, original tol saved in `aMVTol` for rollback).
   - `FilterPavesOnCurves` (~L2437): a vertex paved on several curves keeps only near ones: remove pave when `sqDist > 100 * max(tolR3D^2, minSqDist)` **and** `sinAngle < 0.5` (comment cites `bug27761`: do not remove if projection angle is large); after removal, vertex tol reduced to `sqrt(maxDistKept) + Confusion` if it was saved.
   - `PutStickPavesOnCurve` (~L2748): only if a curve end lacks a vertex; candidate "stick" vertices within `aDT2 = 2e-7` (squared distance) of the curve end, and the surfaces' normals at that end satisfy `1 - |n1.n2| <= aDScPr = 5e-9` (creasing criteria) — i.e. the curve *vanishes* tangentially; put pave with tol = actual distance.
   - `PutEFPavesOnCurve` (~L2692): only when the pair produced exactly **one** curve (comment: test `904/F7`) and the curve is Bezier/BSpline: project leftover EF vertices onto the curve and pave at LowerDistance.
   - `PutBoundPaveOnCurve` (~L2308): compare existing extreme paves against curve endpoints with `tol = curveTol_max + Confusion` (`getBoundPaves` + `ComputeVV`); for an uncovered end whose point `IsValidPointForFaces(P, F1, F2, tolR3D)` (projection distance <= tol && classifier not-OUT on both) make a **new bound vertex** (`MakeNewVertex(P, tolR3D)` + `UpdateVertex(IC, t, V)`), append ext pave, track in `aMVBounds`/`aDMBV`. Closed curve (endpoint distance < Confusion): only one bound, and skipped entirely if either end already has a vertex.
   - `PutClosingPaveOnCurve` (~L3500): curve with bounds, geometrically closed within `tolV + (max(tol,tangTol)+Confusion)`, having a pave at exactly one end parameter (PConfusion match): add the *same vertex* as pave at the opposite end parameter; vertex tol raised to `distVP + 1e-12` if `FindValidRange` still leaves a valid range.
4. **BVH over existing ON/IN pave blocks** (`aPBTree`, boxes of their split edges; degenerate originals skipped via `HasFlag`).
5. **Section-edge creation loop** per curve `j`: `aPB1->Update(aLPB)` turns ext paves into candidate pave blocks; for each block `(nV1,t1)-(nV2,t2)`:
   - skip if `|t1-t2| < Precision::PConfusion()`;
   - **keep-verdict** `myContext->IsValidBlockForFaces(t1,t2,IC,F1,F2,tolR3D)` (`IntTools_Context.cxx` ~L717): midpoint at `IntermediatePoint` (0.43213918 blend) classified per face via **its pcurve if present** (pure 2D classification) else 3D projection + classification with tolR3D;
   - **reuse pass A — shared edges** `IsExistingPaveBlock(PB,NC,aLSE,...)` (~L1988): block midpoint vs each edge already shared by both faces, `ComputePE(Pm, max(tolE, max(tolV1,tolV2)) + fuzzy)`; hit -> `UpdateEdgeTolerance(nEOut, dist)` + `UpdateSavedTolerance` for its vertices, block dropped (edge already exists);
   - **micro gate** `BRepLib::FindValidRange(...)`: block entirely inside the tolerance spheres of its vertices -> record in `aMicroPB` (vertices will be fused in PostTreat; the block itself dies) unless a bound vertex is involved;
   - **reuse pass B — ON/IN pave blocks** `IsExistingPaveBlock(PB,NC,tolR3D,aMPBOnIn,aPBTree,aMPBCommon,...)` (~L2047): BVH select around first endpoint box (enlarged by tolV); per candidate: endpoint identity flags `iFlag1/2 = 2` when vertices shared; base band `aTolCheck = tolR3D + fuzzy`; **common-block bonus**: if the candidate is a common block already, `aRealTol = max(aTolCheck, tolV1, tolV2)`, x2 if it is common with a *face*; **thin-face heuristic** when both endpoints shared but no common block: cap `aMaxTolAdd = min(0.001, 10*aTolCheck)`, probe with `aTolAdd = 2*min(aMaxTolAdd, max(aRealTol, tolVs))`, require tangent alignment `|cos| >= 0.9063` (~25 deg) between curve tangent and edge tangent at the projection — then `aRealTol = aTolAdd`, distance doubled (`aCoeff=2`) for the tolerance update; skip when exactly one of the two blocks is closed (nV1==nV2 xor nV21==nV22). Midpoint + non-shared endpoints must all project within `aRealTol` (`ComputePE`); the closest hit wins (`theTolNew = coeff * dist`).
     On reuse: if the found PB is not yet registered ON/IN **both** faces, raise its edge tolerance to `max(theTolNew, aNC.Tolerance())` if larger; record the missing face in `aPBFacesMap[PB]`; the new block's vertices that are not the found block's vertices go to `aVertsOnRejectedPB` (to be fused); `PreparePostTreatFF` (~L3609) pushes the existing edge into `aMSCPB` + curve's PB list (once per PB, fenced by `aMPBAdd`).
   - **create**: `BOPTools_AlgoTools::MakeEdge(IC, V1,t1, V2,t2, tolR3D, aES)` (~L1729): vertices' tol raised to `tolR3D + 1e-12`, `MakeSectEdge` = `BRepBuilderAPI_MakeEdge(C3D,V1,V2,t1,t2)` + `BB.Range(t1,t2)` ("Range must be as it was!"), edge tol = tolR3D. Then `BOPTools_AlgoTools::MakePCurve(aES, F1, F2, IC, bPC1, bPC2, ctx)` (~L1657): for each face, **reuse the SSI pcurve** (`IC.FirstCurve2d/SecondCurve2d`) if non-null, else build by projection (`BuildPCurveForEdgeOnFace` -> `Make2D` -> `MakePCurveOnFace`, `BOPTools_AlgoTools2D.cxx` ~L48/432/501: `ProjLib_ProjectedCurve` with escalating tolerance tiers `aTR=1e-7 -> min(aMaxTol=1e-4, 0.1*TolReached2d)` etc., surface slightly U/V-extended by 1% for boundary-hugging projections); the pcurve is period-normalized onto the face chart (`AdjustPCurveOnFace/AdjustPCurveOnSurf` ~L209-400: half-parameter probe point, `GeomInt::AdjustPeriodic`, cylinder guard `dFi = maxEdgeTol/R`, V-period midpoint preference, `BRepClass_FaceClassifier` double-check when chart span exceeds the period, then whole-curve `Translate(du,dv)`); finally `BRepLib::SameParameter(aE)`. Register `(aES -> CoupleOfPaveBlocks(i,j,PB))` in `aMSCPB`, vertices in `aMVI`, drop their `aMVTol` rollback entries (they are used now).
   - `ProcessExistingPaveBlocks` overload 1 (~L3072): BVH select ON/IN PBs whose boxes meet the new edge's box; PB registered in both faces -> straight to post-treat (common edge); else consult `myDistances[(originalEdge, otherFace)]` (recorded by the EF stage): overlapping param range and `dist <= tolES + tolE` -> post-treat + `aPBFacesMap`.
6. Per-interference epilogue: rollback tolerances of unused paved vertices from `aMVTol` (direct `BRep_TVertex::Tolerance` + bnd box rebuild + `aDMVLV` unbind); `ProcessExistingPaveBlocks` overload 2 (~L3171): for each new **bound** vertex, BVH-find ON/IN PBs passing through it (`ComputeVE`), send them to post-treat and record missing faces.
7. **Recheck queue** (~L719-733, L879, L1067-1071): an interference processed *early* whose curves produced no pave-block splits is appended to `aFFToRecheck` and re-processed after all others ("necessary to recheck ... to avoid missing section edges" — FF ordering dependence).
8. After the loop: `RemoveMicroSectionEdges` (~L4308): section edges failing `IsMicroEdge` (`BOPTools_AlgoTools.cxx` ~L2075: degenerated/non-geometric or `IntTools_ShrunkRange` fails) are removed from curve PB lists; their PBs go to `theMicroPB`.
9. `MakeSDVerticesFF` (~L1141): each `aDMVLV` group -> `MakeSDVertices` -> `aDMNewSD[old] = new`.

### Stage 3 — PostTreatFF: cross-pair unification of coincident section entities
`BOPAlgo_PaveFiller_6.cxx`, `PostTreatFF` (~L1165-1669).

- **Unused-vertex sweep** (~L1203-1231): per FF pair, stick vertices not used by any curve pave (`RemoveUsedVertices`) collected into `VertsUnused` (kept only if unused in exactly one pair — the toggle `IndMap.Add/RemoveKey` logic).
- **Fast path** (~L1237): exactly 1 shape, no micro PBs, no rejected verts, no unused verts -> append the vertex/edge directly to the DS (existing-edge case binds `aDMExEdges[PB1] = {PB1}`), done.
- **Argument assembly** (~L1280-1385):
  - all new section edges and new vertices from `aMSCPB` go into list `aLS`; **already-existing edges are packed into a single compound** `anExistingEdges` so the nested filler will NOT intersect them among themselves — only against the new entities (geometry preservation);
  - SD images (`aDMNewSD`) of involved vertices added so they participate in the fuse;
  - **micro-PB vertex pairs** (~L1324-1358): their two (SD-resolved) vertices are added, and if their tolerance spheres do not overlap, both tolerances are inflated by half the gap (`aDist -= tolV1+tolV2; tol += aDist/2`) — guarantees the nested fuse unites them;
  - vertices on rejected PBs and unused stick vertices added (SD-resolved, fenced).
- **Nested fuse** (~L1389): a fresh secondary `BOPAlgo_PaveFiller aPF(args=aLS)` (non-primary, inherits NonDestructive) is run — this general V/V+V/E+E/E fuse is the actual unification engine for coincident section edges from *different* FF pairs and for all the vertex classes above.
- **Read-back** (~L1407-1656):
  - vertex: image = SD shape in `aPDS` if any; appended to main DS if new; if it was an intersection *point*, the FF interference's `BOPDS_Point.SetIndex(iV)`; else `aDMNewSD[nSx]=iV` + `myDS->AddShapeSD`.
  - edge: fetch its `CoupleOfPaveBlocks (iX interf, iC curve, aPB1)`; `bOld = aPB1->HasEdge()` (was an existing edge);
    - image has no pave blocks: old -> `aDMExEdges[aPB1] += aPB1`; new -> append edge shape to DS, `aPB1->SetEdge(iE)`;
    - image became **micro** (`aNbLPBx==0`, or 1 block without shrunk data): remove `aPB1` from the curve's PB list, push its vertices back into `aLS` for SD;
    - otherwise, for each image pave block `aPBx` (through `aPDS->RealPaveBlock` — common-block representative): remap pave vertices into the main DS (append if new); **vertex SD bookkeeping**: if the old pave vertex differs from the image at the same parameter -> `aDMNewSD` + `AddShapeSD`; if parameters differ, accept only when the small filler itself has that SD connection (~L1574-1596);
    - image edge appended to main DS; if the image is a common block, `BOPAlgo_Tools::ComputeToleranceOfCB` (`BOPAlgo_Tools.cxx` ~L248: max over 11 interior samples of `tol(other edge/face) + lowerDistance` to it, cached per CB) may **raise the curve tolerance** `aNC.SetTolerance`;
    - `aMEPB` cache guarantees **one** main-DS pave block per new edge index (different curves referencing the same fused edge share the PB) — this is the aliasing that makes a cross-pair-coincident section edge single in the DS;
    - old edges collect their split PBs in `aDMExEdges[aPB1]`; new edges' PBs are appended to the curve's PB list (replacing `aPB1`).
  - transitive closure of `aDMNewSD` chains (~L1659-1668).

### Stage 4 — MakeBlocks tail: tolerance correction, face-info update, SD application, SE spreading
`BOPAlgo_PaveFiller_6.cxx`: `CorrectToleranceOfSE` (~L4072), `UpdateFaceInfo` (~L1673), `UpdateExistingPaveBlocks` (~L3278), `UpdatePaveBlocks` (~L3679), `RemovePaveBlocks` (~L3815), `PutSEInOtherFaces` (~L4277), `UpdateBlocksWithSharedVertices` (~L3946, NonDestructive only), `RemoveMicroEdges` (~L4388).

- `CorrectToleranceOfSE`: for pure section edges (`OriginalEdge < 0`) where `curveTol < tangentialTol` and `curveTol < edgeTol`: **reduce** edge tolerance to curve tolerance (the tangential tolerance covers the flat common zone but the edge doesn't need it); then per touched vertex compute the max distance to the pave points of all adjacent edges (+edge tol) and reduce the vertex tolerance to that, floored by the max adjacent edge tolerance; skipped when the gain is < 0.1%.
- `UpdateFaceInfo`:
  - curve PBs bound in `aDMExEdges` (existing edges rebuilt by PostTreat) are re-split via `UpdateExistingPaveBlocks` (~L3278): old PB(s)/common-block removed from the original edges' PB lists; for common blocks new per-original-edge PBs are built with parameters transferred by vertex identity, `ComputeVE` projection, or nearest-boundary fallback (~L3355-3416); then **face attachment**: for each face recorded in `aPBFacesMap`, run `IntTools_EdgeFace` on the split edge restricted to the PB range; a single `TopAbs_EDGE` common part -> the PB joins/creates a common block, `aCB->AddFace(nF)`, and enters that face's `PaveBlocksIn` (~L3448-3494);
  - remaining curve PBs enter both faces' `PaveBlocksSc`; PBs of the same DS edge coming from several curves are united into a **new common block** (~L1767-1858, merging faces and PBs of any pre-existing CBs);
  - FaceInfo vertex maps and On/In/Sc PB maps rewritten through `aDMNewSD` / `aDMExEdges` with `RealPaveBlock` dedup (~L1874-1945).
- `UpdatePaveBlocks`: applies `aDMNewSD` to every pave of every PB (section + pool); a rebuilt PB whose two vertices collapsed into one and which has no shrunk data becomes a **micro edge** and is removed globally (`RemovePaveBlocks`: pool, curve lists, face-info maps); otherwise the edge is re-split (`SplitEdge`).
- `PutSEInOtherFaces` (~L4277): collect ALL section PBs of all curves and run `ForceInterfEF(aMPBScAll, false)` — "Treat possible common zones by trying to put each section edge into all faces, not participated in creation of that edge, as IN edge".

### Stage 5 — Same-domain faces (Builder stage, consumes FF data)
`BOPAlgo_Builder_2.cxx`: `FillSameDomainFaces` (~L580-925), `BOPAlgo_PairOfShapeBoolean::Perform` (~L104); `BOPTools_AlgoTools.cxx`: `AreFacesSameDomain` (~L1139-1205), `Sense` (~L1209), `IsSplitToReverse(face)` (~L1324); also used by `BOPAlgo_CheckerSI` (~L327).

- **When**: after `BuildSplitFaces` (faces already split by section edges), before building the result.
- Candidates: all faces appearing in any FF interference (fenced, sorted by DS index).
- **Edge-set bucketing**: for every face (or each of its splits) build a `BOPTools_Set` of its edges (`AddEdgeSet`); only faces with **identical edge sets** are SD candidates — the geometry check runs on pairs within a bucket.
- **Same-solid guard** (~L595-649): faces (or splits — propagated through `myImages`) whose parent solid is the same are never SD ("zero-thickness interior in a single operand").
- **Planar shortcut** (~L692-717, 780-785): two *bounded* planar faces (bbox not open in any direction) with equal edge sets are SD **without** geometric check.
- Remaining pairs run `AreFacesSameDomain` in parallel: find an interior point of F1 (`BOPTools_AlgoTools3D::PointInFace` — hatcher-based), tolerance `aTol = tolF1' + tolF2' + max(fuzzy, Confusion)` where each `tolF'` is maxed with the largest non-degenerate edge tolerance of F1 ("faces should have the same boundaries, thus it does not matter which face to explore"); verdict = `IsValidPointForFace(P, F2, aTol)` (`IntTools_Context.cxx` ~L647: projection LowerDistance <= aTol AND UV classifier not-OUT).
- SD pairs -> connexity blocks (`BOPAlgo_Tools::MakeBlocks` over the back-forth map); **representative** = face with minimal DS index among original (unsplit) faces of the group, else the first; every member bound in `myShapesSD`, `myImages` rewritten to the representative, `myOrigins` reverse-filled. Original faces in a group get a self-image ("consider it being split").
- Orientation of the representative vs consumers resolved later via `IsSplitToReverse(faceSp, faceSr)` (~L1324: same-surface handle -> orientation compare; else PointInFace + normal projection compare) / `Sense` (~L1209: normals compared at a shared non-degenerate, non-closed edge, `SenseFlag`).

---

## DATA STRUCTURES

- **`BOPDS_InterfFF`** — per face pair: `(nF1,nF2)`, `TangentFaces` flag (same-domain surfaces signal), `Curves : array<BOPDS_Curve>`, `Points : array<BOPDS_Point>`. TangentFaces=true means *no* curves — the pair is handled by the SD-face stage instead.
- **`BOPDS_Curve`** — one section curve: `IntTools_Curve` (below), `Bnd_Box` (enlarged by tolFF + max vertex tol — the box used for *sharing* queries), `Tolerance` (mutable: raised by CB tolerance in PostTreat), `TangentialTolerance`, `PaveBlock1` (carrier of ext paves = the paving workspace), `PaveBlocks` (final section pave blocks). Both tolerances feed `aTolR3D = max(tol, tangTol)` everywhere.
- **`IntTools_Curve`** — `{C3D, C2D1, C2D2, Tolerance, TangentialTolerance}`. Pcurves may be null (then classification falls back to 3D projection). Index correspondence 3D<->2D guaranteed by construction (same knots/params for approx output; SameRange for restriction lines).
- **`BOPDS_Pave`** `{vertex index, parameter}`; ordered on curve.
- **`BOPDS_PaveBlock`** `{Pave1, Pave2, ExtPaves (unsplit), Edge (split-edge DS idx), OriginalEdge (-1 for section edges — the marker CorrectToleranceOfSE keys on), ShrunkData}`. `Update(aLPB)` = sort ext paves and emit sub-blocks.
- **`BOPDS_CommonBlock`** `{PaveBlocks (coincident PBs from different edges), Faces (face indices the block lies IN), Tolerance, PaveBlock1 = representative}` — the persistent "these edges are geometrically one" record; `RealPaveBlock` resolves a PB to its CB representative.
- **`BOPDS_CoupleOfPaveBlocks`** `{IndexInterf, Index (curve or point idx), PaveBlock1}` — back-pointer that lets PostTreatFF write fuse results into the right FF/curve slot.
- **`aMSCPB`** : IndexedDataMap shape->CoupleOfPaveBlocks — every entity going into the nested fuse.
- **`aDMExEdges`** : PB -> list<PB> — how an *existing* edge's block decomposed in PostTreat; consumed by `UpdateFaceInfo`/`UpdateExistingPaveBlocks`.
- **`aDMNewSD`** : int->int vertex SD substitution map, applied globally by `UpdatePaveBlocks`; transitively closed.
- **`aMVTol`** : vertex -> original tolerance (rollback for unused paves); **`aDMVLV`** : vertex -> SD group (paves landing on an occupied parameter).
- **`aPBFacesMap`** (`BOPAlgo_DataMapOfPaveBlockListOfInteger`) : PB -> faces it must be attached to (deferred `IntTools_EdgeFace` verification).
- **`aMicroPB`**, **`aVertsOnRejectedPB`**, `VertsUnused` — vertex-fusion feedstock for the nested filler.
- **`aEEMap`** : (nE1,nE2)->new EE vertices — seam-shift detection input.
- **`myDistances`** : (originalEdge, face) -> `EdgeRangeDistance {First, Last, Distance}` — EF near-miss distances recorded by the EF stage, reused by `ProcessExistingPaveBlocks` as a coincidence oracle.
- **`BOPDS_FaceInfo`** — `VerticesOn/In/Sc`, `PaveBlocksOn/In/Sc`: the per-face boundary description the Builder splits faces with; `Sc` = section entities.
- **`BOPTools_Set`** — order/orientation-independent canonical set of a face's edges; hash key of the SD bucketing.
- **`aFaceToParent`** — face->owning solid (with split propagation) for the same-solid SD guard.

---

## CONSTANTS & TOLERANCES

| Value | Where | Meaning |
|---|---|---|
| `Precision::Confusion() = 1e-7` | everywhere | base 3D coincidence |
| `Precision::PConfusion() = 1e-9` | param comparisons (zero-span block, closing pave, SameRange) | parametric coincidence |
| `Precision::Angular() = 1e-12` | iso direction test, seam-line parallel test | |
| `BOPTools_AlgoTools::DTolerance() = 1e-12` | additive when growing vertex/edge tols (`MakeEdge`, `PutPaveOnCurve`, `UpdateVertex`) | anti-borderline epsilon |
| `myTolFF default 1e-7`; `ToleranceFF = max(tolF1,tolF2)`, floor `5e-6` if any non-analytic surface | PerformFF ~L3922 | SSI acceptance band |
| `aTolFF(MakeBlocks) = max(BRep tolF1, tolF2)` | MakeBlocks ~L750 | vertex-existence band |
| `myTol = (tolF1+fuzz/2)+(tolF2+fuzz/2)`; `TolArc=TolTang=myTol` | IntTools_FaceFace::Perform ~L381-387 | walker tolerances |
| `TolAng plane-plane = 1e-8` | Perform ~L405, PerformPlanes | |
| cyl-torus `|Rc-Rt| < 1e-7` -> `TolTang *= 0.1`, `iDegMax=6` | `Tolerances` ~L2787, `ApproxParameters` ~L2736 | tangent family damping |
| `Deflection 0.1` (0.01 bspl x bspl); `UVMaxStep = DefineUVMaxStep` | Perform ~L483-489 | walker step |
| `myTolApprox = 1e-7` (ctor), fallback `aTolApproxImp = 1e-5` on reApprox | MakeCurve ~L1237, 1727 | approx tolerance ladder |
| `TOLCHECK=1e-7, TOLANGCHECK=1e-6` | MakeCurve ~L709 | FixTangent |
| chart expansion: plane 10% per side; others `myTol*2` non-periodic | `CorrectPlaneBoundaries` ~L3093, `CorrectSurfaceBoundaries` ~L2017 | curves must overshoot trims |
| `isTreatAnalityc`: ellipse `major > 1e5 * minor` -> walker | ~L318 | sliver ellipse guard |
| infinite-line probe `dT=100`; circle sampling `2pi/17`, 18 samples | MakeCurve ~L854, L1106 | keep-verdict probes |
| `ParameterOutOfBoundary` step `0.1*myTol` (min Confusion), <= 11 iters | ~L2229 | walk off boundary-ON |
| `CheckPCurve`: 23 pts/interval, margin `max(1% UV span, Confusion)` | ~L3010 | pcurve in-bounds QA |
| `IsCurveValid` self-int tol `1e-10` | ~L2315 | 2D loop guard |
| `MakePCurveOnFace` tiers: `aTR=1e-7`, `aMaxTol=1e-4`, escalation `0.1*TolReached2d`, `1e3*aMaxTol` cap, surface extension 1% | AlgoTools2D ~L525-585 | projection ladder |
| `IsExistingVertex`: `tolR3D + fuzzy` (+`tolV + 1e-7` inside ComputeVV) | ~L1960 | |
| `IsExistingPaveBlock` B: `aTolCheck = tolR3D + fuzzy`; `aMaxTolAdd = min(0.001, 10*aTolCheck)`; probe `2*min(aMaxTolAdd, max(...))`; CB bonus x2; tangent `|cos| >= 0.9063` (25 deg); reuse tol = `coeff(1 or 2) * dist` | ~L2104-2247 | same-domain edge reuse band |
| `FilterPavesOnCurves`: drop pave iff `sqD > 100*max(tol^2, minSqD)` && `sin < 0.5` | ~L2515 | multi-curve arbitration |
| `PutStickPavesOnCurve`: `aDT2=2e-7` (sq dist), `aDScPr=5e-9` on `1-|n1.n2|` | ~L2793 | vanishing-curve ends |
| `PutPaveOnCurve` pave dedup: `aPTol = GAC.Resolution(max(tolR3D,tolV))` | ~L3002 | param-space merge radius |
| `getBoundPaves` end match: `max(tol,tangTol) + Confusion` | ~L2292 | |
| micro: `BRepLib::FindValidRange` / `IntTools_ShrunkRange` | ~L937, IsMicroEdge ~L2075 | tolerance-sphere containment |
| micro-PB fuse guarantee: inflate both tols by `gap/2` | PostTreatFF ~L1352-1357 | |
| `ComputeToleranceOfCB`: 11 interior samples, `tol_other + lowerDistance`, max | BOPAlgo_Tools ~L248 | CB tolerance |
| `CorrectToleranceOfSE`: only if gain >= 0.1% | ~L4225 | tol reduction hysteresis |
| `AreFacesSameDomain`: `tol = tolF1' + tolF2' + max(fuzz, 1e-7)`, `tolF' = max(faceTol, max edgeTol)` | AlgoTools ~L1168-1199 | SD face band |
| plane-plane tangential tol: `sqrt(dt^2 + TolF1^2)`, `dt = ComputeIntRange(TolF1,TolF2,angle)` | PerformPlanes ~L2552 | flat-zone half-width |
| `TrsfToPoint` criteria `1e5` | BOPAlgo_Tools.hxx ~L237 | far-from-origin recentering |
| `IntermediatePoint` blend `PAR_T = 0.43213918` (=10*e^-pi) | AlgoTools2D ~L407 | irrational midpoint (avoids symmetric coincidences) |
| `FindMaxDistance`: golden 0.6180339887, 11 seeds, eps `1e-4*dt` | FaceFace ~L2899 | curve-surface deviation search |
| `AttachExistingPCurve` reject: `aTolSP > 10*tolE && aTolSP > 0.1` | AlgoTools2D_1 ~L115 | pcurve transplant guard |

---

## INVARIANTS

Downstream stages (BuildSplitFaces, BuilderFace, BuilderSolid) may assume:

1. **One DS edge per geometric section locus.** Coincident sub-curves — whether from the same pair, different pairs, or coinciding with a pre-existing model edge — are represented by a single edge: pass A (shared edges), pass B (ON/IN pave blocks with tangent test), the `aMEPB` cache, and the nested PostTreatFF fuse together enforce this. Edges shared by several curves are additionally tied by common blocks.
2. **Every section edge is SameParameter'd with pcurves on both generating faces**, and its tolerance covers the true 3D/2D deviation (ComputeTolReached3d before, ComputeTolerance/SameParameter after).
3. **Tolerance covers reality, in both directions**: every vertex tolerance covers its distance to every curve/edge it paves (grown with +1e-12 margin at grow time); tolerances are *reduced* afterwards only when a computed bound proves it safe (FilterPavesOnCurves, CorrectToleranceOfSE with 0.1% hysteresis, rollback of unused paves).
4. **No micro entities**: every surviving pave block has a valid shrunk range outside its vertices' tolerance spheres; collapsed blocks were removed and their vertices fused.
5. **Keep-verdict**: every section PB's interior lies In/On both faces (IsValidBlockForFaces), and bound vertices were only created at points valid for both faces.
6. **SD maps are globally applied**: after UpdatePaveBlocks no pave references a fused-away vertex; FaceInfo maps are rewritten consistently (RealPaveBlock canonicalization).
7. **Section edges are known to every face that contains them**, not just the generating pair (PutSEInOtherFaces/ForceInterfEF; aPBFacesMap + IntTools_EdgeFace attachment) — common (tangential-overlap) zones appear as IN edges everywhere relevant.
8. **Input geometry is never re-approximated**: existing edges enter the nested fuse inside one compound and are not intersected among themselves; CopyEdge/MakeSplitEdge reuse the original curve, only ranges/vertices change.
9. **SD faces have exactly one representative** in images/origins, chosen deterministically (min DS index), and two faces of one solid are never declared SD.
10. **TangentFaces flag** partitions the problem: a pair is either transversal (curves) or tangential (SD candidate) — never both.

---

## PITFALLS

Corner cases the source explicitly handles (comments / dedicated branches):

- **Seam misalignment between closed surfaces** (PerformFF ~L393-486): EE vertex not exactly on the closed edge -> whole face translated by the exact residual; residual added to the pair tolerance. Without it section curves stop short of the boundary.
- **FF processing order can lose section edges** (~L719-733): pairs whose curves yield no blocks on first pass are re-queued (`aFFToRecheck`) and re-run after all pairs have contributed their paves/vertices.
- **`bugs modalg_6 bug26789_1`** (~L801-807): non-common vertices are bbox-checked against the curve before paving to avoid spurious vertex-tolerance growth.
- **`bug27761`** (FilterPavesOnCurves ~L2503): a vertex projecting onto a curve at a steep angle (sin >= 0.5) is *kept* even at large distance — removing it would open the loop.
- **`904/F7`** (~L822-826): EF paves are added mid-curve only when the pair produced exactly one curve; otherwise they belong to the other curve.
- **Thin-face / near-tangent existing edge** (IsExistingPaveBlock B ~L2110-2198): both endpoints shared but no common block -> the section curve may run alongside an existing edge within a *large* band; capped at `min(0.001, 10*tolCheck)` and requires 25-degree tangent alignment; skipped when exactly one of the blocks is closed (nV==nV on one side only).
- **Circle/ellipse period handling** (~L926-1011): intervals crossing t=0 are split in two; sub-tolerance remainders survive only if their chord exceeds myTol, with `ParameterOutOfBoundary` stepping the parameter off boundary-ON states (<= 11 steps of 0.1*tol).
- **Pcurve out of face bounds after approx** (~L1595-1625): fall back to non-approximated WLine BSpline lift; if that is also out of bounds, keep the approximated one anyway.
- **2D self-intersecting pcurve** (IsCurveValid) and **failed FixTangent QA** -> `goto reapprox` with degraded tolerance 1e-5 / no-surface mode; `myHS1==myHS2` always no-surface (`rejectSurface`).
- **Torus seeds dropped** (~L492-500) for analytic pairs; plane x cylinder giant ellipse -> walker (`isTreatAnalityc`).
- **Closed curve paving**: `PutBoundPaveOnCurve` puts only one bound on a closed curve and none if an end vertex exists; `PutClosingPaveOnCurve` re-uses the same vertex at the opposite parameter and checks `FindValidRange` before growing its tolerance.
- **Micro-PB vertex fusion guarantee** (PostTreatFF ~L1344-1358): tolerances of the two vertices are mutually inflated to touch, otherwise the nested filler might not unite them.
- **Vertices of rejected blocks / unused stick vertices** are still sent through the fuse — they may coincide with kept vertices and must not survive as duplicates.
- **Existing edges in one compound** (PostTreatFF ~L1280-1316): prevents the nested filler from re-intersecting model edges with each other (that would create spurious splits of untouched geometry).
- **Pave transfer across original edges of a common block** (UpdateExistingPaveBlocks ~L3355-3416): parameters transferred by vertex identity, else `ComputeVE`, else nearest-end fallback; "still closed" case handled separately.
- **AdjustPCurveOnSurf** (`AlgoTools2D` ~L247-400): period translation decided at the *half-parameter* point; cylinder uses `dFi = maxEdgeTol/R` angular band; V-period prefers the chart-midpoint-closest branch; a `BRepClass_FaceClassifier` re-check protects charts wider than one period.
- **UpdateClosedPCurve** (`AlgoTools2D_1` ~L164-285): an edge closed on a face needs *two* pcurves; the translation vector is taken from the two existing seam pcurves, its sign fixed by which seam copy the new curve hugs, the pcurve order by tangent dot product.
- **AttachExistingPCurve** rejects transplants that would blow the tolerance (`>10x tolE && >0.1`) and works on a temporary edge so failures don't corrupt the target.
- **Same-solid SD guard** and **planar-bounded shortcut** in FillSameDomainFaces (zero-thickness guard; classifier-free SD for bounded planes with identical edge sets).
- **Degenerated edges** are excluded everywhere (`HasFlag()`, `BRep_Tool::Degenerated`), incl. the BVH of existing PBs and SD tolerance scan.
- **CorrectToleranceOfSE** applies only to pure section edges (`OriginalEdge < 0`): small dihedral angles make `TangentialTolerance` huge; the 3D curve tolerance is the honest bound for the *edge*, the tangential one only for *classification*.

---

## PORT MAP

Anchors: `session_cpp/src/brep_section.cpp` (`build_section_scaffold`, stage 1b PutToBoundary, paves, keep-verdict, micro filter, valence-1 bridge), `session_cpp/src/brep.cpp` (`split_with`/`split_by_uv_curves`, whole-segment seg_id runs, zero-span collapse, `combine`: 1e-7 weld + tube merge + NK-RESCUE + collapse + classification). Tolerances: `tol3 = diag*2e-3`, `tol3_rep`.

| # | OCCT mechanism | Our anchor | Action | Design (for new build) |
|---|---|---|---|---|
| 1 | One SSI per pair, seeded by prior-stage exact points (`GetEFPnts`/`SetList`, `myListOfPnts`) | `build_section_scaffold` ONE SSI per pair | **already-equivalent / extend** | We already run one SSI per pair; extend by seeding the marcher's predictor with our already-computed trim-loop crossing points (both operands), so chains start *on* boundary crossings instead of stalling near them — OCCT's exact-point transfer is why their curves reach the boundary. |
| 2 | `PutBoundPaveOnCurve` + `IsValidPointForFaces` gate + `DecompositionOfWLine` | stage 1b PutToBoundary (CHART bounds only; gated SESSION_EXT_TRIM) | **adopt** | Make trim-crossing extension the default, but gate each extension exactly like OCCT: extend only if the target point projects within `tolR3D` of BOTH surfaces and classifies not-OUT in both trims; closed chains get at most one bound + a closing pave reusing the same vertex at the opposite end (our closure-weld cap is the analogue — key it on the same-vertex rule). |
| 3 | `IsExistingPaveBlock` A (shared edges) + B (ON/IN BVH, 25-deg tangent test, `min(0.001,10*tol)` cap, CB x2 bonus) | `split_with` whole-segment seg_id runs + zero-span collapse; `combine` NK-RESCUE (0.15*tol3) | **new build** (top priority) | Before emitting a section run as a new edge, BVH-query the operand's *existing* boundary edges: test run midpoint + both endpoints by projection within `max(tolE, tolV) + band`, require tangent alignment `|cos|>=0.9063`; on hit, alias the run to the existing edge id (no new edge) and raise that edge's stored tolerance to the measured distance. This is the missing same-domain-edge subsystem and directly targets the SEGLOST/boundary-hugging class we currently patch with zero-span collapse + NK-RESCUE after the fact. |
| 4 | PostTreatFF nested fuse: cross-pair unification of coincident section edges + vertex SD map, `aMEPB` one-PB-per-edge aliasing, existing-edges-in-compound rule | `combine` exact 1e-7 weld + vertex-pair-keyed tube merge + NK-RESCUE | **replace / move earlier** | Run ONE global unification pass over all pairs' chains BEFORE the per-operand UV split: (a) weld vertices across pairs into an SD map (we do 3D weld already — make the map explicit and apply it to chain endpoints); (b) fuse coincident sub-chains from different pairs into one global seg_id (generalize the per-pair alias index); (c) never re-march or re-fit chains against each other's *operand boundary* edges (OCCT's compound rule). Combine then sees canonical edges instead of rescuing mates within 0.15*tol3. |
| 5 | Per-entity tolerance model: `aNC.SetTolerance(ComputeTolReached3d)`, `UpdateEdgeTolerance`, `ComputeToleranceOfCB` (11-sample max), `CorrectToleranceOfSE` reduction, `aMVTol` rollback | fixed `tol3 = diag*2e-3`, `tol3_rep`; no growth model (listed MISSING) | **new build** | Per-chain tolerance = max corrector residual over samples (we have the Gauss-Newton residuals for free); per-vertex tolerance = actual weld distance + 1e-12, grown on weld, rolled back if the pave goes unused; classification and NK-RESCUE bands become `max(chain_tol, vertex_tol)` instead of global tol3; add the 0.1%-gain reduction pass after combine. |
| 6 | `IsValidBlockForFaces`: midpoint classified via pcurve when present (pure 2D), else 3D projection | per-interval keep-verdict (9 samples, bisected ends) | **already-equivalent (keep ours), adopt detail** | Our 9-sample verdict is stronger; adopt the pcurve shortcut: classify uvA/uvB midpoints by UV winding directly (we carry index-corresponded uvA/uvB) — cheaper and seam-robust vs 3D projection; keep 3D fallback for extension segments lacking UV. |
| 7 | Micro machinery: `FindValidRange` gate pre-creation, `RemoveMicroSectionEdges`, post-SD collapse detection in `UpdatePaveBlocks` (nV1==nV2 && no shrunk data) | micro filter (scaffold) + micro-edge collapse (combine) | **adopt detail** | Add the post-weld recheck: after the vertex SD map is applied, re-scan all runs for blocks whose two endpoints became the same vertex and whose span is inside the weld band; delete and re-point adjacent faces. We only collapse at operand split today, so cross-pair welds can still produce degenerate stubs. |
| 8 | `PutSEInOtherFaces` (`ForceInterfEF` on all section PBs) + `aPBFacesMap`/`IntTools_EdgeFace` face attachment + CommonBlock face lists | nothing (chains only fed to the generating pair's faces); `extra_cuts` same-domain imprint infrastructure exists | **new build** | After scaffold, test every section chain against every OTHER overlapping face of both operands (BVH + 11-sample projection <= tolE+tolF); on coincidence, imprint the chain into that face's UV arrangement via extra_cuts and record the face on the chain's merged tube; classification flood must treat a tube as boundary for ALL attached faces (targets the asymmetric span-crossing-union failures). |
| 9 | `AreFacesSameDomain` + `FillSameDomainFaces` (edge-set bucket, same-solid guard, planar shortcut, min-index representative, `Sense`/`IsSplitToReverse` orientation) | MISSING entirely | **new build** | After split_with: bucket result faces by canonical boundary key (sorted set of merged-edge ids); skip same-operand pairs; for bucket pairs, sample an interior point of A (our winding-contains gives one), project onto B's surface, accept if dist <= tolA+tolB+band and inside B's trims; group by union-find, keep min-id representative, orientation by normal dot at a shared edge midpoint. Required for tangential-contact fuse/common and xor cells; also the correct home for TangentFaces pairs our SSI currently reports as empty/degenerate. |
| 10 | `MakePCurve`: reuse SSI pcurves, else `ProjLib` ladder; `AdjustPCurveOnFace` period normalization; `BRepLib::SameParameter` | chains index-corresponded (p3, uvA, uvB) by construction | **already-equivalent (stronger), adopt detail** | Our exact correspondence beats projection; adopt only the period-normalization discipline when lifting runs into charts: decide the +-period translation at the half-parameter probe with a classifier double-check when the chart spans >= one period (matches our UV-arrangement cut-node snap fixes). |
| 11 | `ToleranceFF` non-analytic floor 5e-6; `aTolFF = max(shift, faces)`; seam-shift value folded into tolerance | SSI acceptance uses tol3/tol3_rep only | **adopt** | Floor the per-pair acceptance/weld band for imported freeform pairs at `max(tol3_rep, scale*5e-6)`; if we ever apply a seam-alignment correction, fold its magnitude into that pair's band as OCCT does. |
| 12 | Recheck queue `aFFToRecheck` (order-dependence guard) | valence-1 bridge re-march CASE A | **adopt policy** | Re-run the marcher for any pair whose chains produced zero kept intervals after ALL pairs contributed paves/welds (not just at valence-1 endpoints) — cheap second pass with the enriched pave set. |
| 13 | Pave arbitration: `FilterPavesOnCurves` (min-distance keep, 100x band + sin<0.5 drop), `PutStickPavesOnCurve` (2e-7 / 5e-9 crease), pave dedup by curve resolution radius | paves = trim crossings + chain-chain crossings + vertex projections | **adopt** | When one welded vertex projects onto multiple chains, keep the nearest chain's pave and drop paves with sqDist > 100x band unless the projection angle is steep (sin >= 0.5); dedup paves within `resolution(max(band, tolV))` in parameter space. Prevents duplicate junction paves that currently force micro-weld rescues. |
| 14 | `TrsfToPoint` origin recentering (criteria 1e5) | none | **adopt (cheap)** | If the pair's joint bbox center exceeds ~1e5x its size from origin, translate both surfaces to origin for the march and translate chains back — pure conditioning for the corrector. |
| 15 | `MakeSectEdge` exact-range rule ("Range must be as it was"), `CopyEdge`/`MakeSplitEdge` reuse original geometry | runs lift EXACT subranges of the shared 3D chain | **already-equivalent** | Keep: our exact-subrange lifting is the same invariant (never re-fit a split's geometry). |

---
*Sources read in full or in the cited ranges: `BOPAlgo_PaveFiller_6.cxx` (4435 L), `IntTools_FaceFace.cxx` (3111 L), `BOPTools_AlgoTools.cxx` (2416 L), `BOPTools_AlgoTools_2.cxx`, `BOPTools_AlgoTools2D.cxx` (700 L), `BOPTools_AlgoTools2D_1.cxx` (311 L), `BOPAlgo_Tools.cxx` (relevant fns), `BOPAlgo_Builder_2.cxx` (FillSameDomainFaces), `IntTools_Context.cxx` (validity fns), `Precision.hxx`.*
