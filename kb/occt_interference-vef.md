# interference-vef

OCCT boolean-kernel subsystem: Vertex-Edge (VE), Edge-Edge (EE), Vertex-Face (VF), Edge-Face (EF)
interference detection, common-part classification (point vs overlap range), pave placement with
extended tolerance, and conversion of results into paves + common blocks.

Sources read (OCCT, `C:/brg/compas_occt/external/occt/src/occt/src/ModelingAlgorithms/TKBO/`):
`BOPAlgo/BOPAlgo_PaveFiller.cxx` (driver), `_2.cxx` (VE), `_3.cxx` (EE), `_4.cxx` (VF), `_5.cxx` (EF),
`_6.cxx` (PutPaveOnCurve/ExtendedTolerance, FF-side pave placement), `_10.cxx` (UpdateVertex),
`IntTools/IntTools_EdgeEdge.cxx`, `IntTools_EdgeFace.cxx`, `IntTools_BeanFaceIntersector.cxx`,
`IntTools_CommonPrt.hxx`, `IntTools_Range.hxx`, `IntTools_Context.cxx`, `IntTools_ShrunkRange.cxx`,
`BOPTools/BOPTools_AlgoTools_2.cxx` (MakeNewVertex/CorrectRange), `BOPTools_AlgoTools.cxx` (ComputeVV),
`IntTools_Tools.cxx` (IsOnPave1/IsInRange/VertexParameters/ComputeIntRange).

Global stage order (`BOPAlgo_PaveFiller.cxx:254-328`, `PerformInternal1`):

```
PerformVV -> PerformVE -> UpdatePaveBlocksWithSDVertices -> PerformEE -> SD-update
          -> PerformVF -> SD-update -> PerformEF -> SD-update
          -> RepeatIntersection (re-runs VV/VE/VF for vertices whose tolerance grew; :359-422)
          -> ForceInterfEE -> ForceInterfEF -> PerformFF (SSI stage, consumes all tables above)
```

Every interference is recorded twice: a typed record (`InterfVE/VF/EE/EF` arrays in BOPDS_DS, with
`IndexNew` = fused vertex it produced) AND a pair entry (`myDS->AddInterf(n1,n2)`) so every later
stage can skip pairs already decided (`HasInterf`, `HasInterfShapeSubShapes`).

## STAGE PIPELINE

### 0. FillShrunkData / IntTools_ShrunkRange — precondition for every stage
- Files: `BOPAlgo_PaveFiller_3.cxx:727-762` (`FillShrunkData(PB)`), `:766-824` (`AnalyzeShrunkData`),
  `IntTools_ShrunkRange.cxx:107-191` (`Perform`).
- Purpose: for each pave block (PB) compute the sub-range of the edge NOT covered by the tolerance
  spheres of its two bounding vertices ("shrunk range"), its bounding box, and the `IsSplittable` flag.
- Mechanism: `BRepLib::FindValidRange(curve, tolE, T1, P1, tolV1+1e-7, T2, P2, tolV2+1e-7, TS1, TS2)`
  with `tolVi := max(tolVi, tolE)`. Micro edge if `TS2-TS1 < PConfusion` or arclength `< Confusion`.
  `IsSplittable = (length > 2*tolE + 2*Confusion)` (room for one splitting vertex + two minimal edges).
  Box: `BndLib_Add3dCurve::Add(curve, TS1, TS2, tolE + Confusion)`; PaveFiller adds `+ myFuzzyValue/2`
  gap (`_3.cxx:822`). Failure emits `AlertTooSmallEdge` / `AlertNotSplittableEdge` / `AlertBadPositioning`
  and stores a degenerate shrunk record with `bIsSplittable=false`.
- Runs: at the head of PerformVE/EE/EF (`FillShrunkData(type,type)`), after every PB split, and lazily
  in ForceInterfEF.
- Output consumed as: the pave "end band" `[T, TS]` on each side — the parametric zone owned by the
  bounding vertex. All on-pave decisions below test against these bands.

### 1. PerformVE — vertex-edge interference
- File: `BOPAlgo_PaveFiller_2.cxx:141-208` (`PerformVE`), `:212-395` (`IntersectVE`), solver class
  `BOPAlgo_VertexEdge::Perform` `:104-121` -> `IntTools_Context::ComputeVE` (`IntTools_Context.cxx:499-541`).
- When: after PerformVV; also re-entered by PerformEE/PerformEF via `PerformNewVertices -> IntersectVE`
  (with `theAddInterfs=false`) to put freshly created intersection vertices on their parent edges.
- Pair filtering (`:156-205`): skip if vertex is a sub-shape of the edge, edge is degenerate (`HasFlag`),
  pair already has interference, edge has no PBs, or the single PB is a micro edge (`!IsSplittable`).
  Vertices grouped per PB, then SD-resolved (`HasShapeSD`) and deduplicated by `BOPDS_Pair(nVSD,nE)`
  fence map `aDMVSD` (`:235-291`); vertices that are already pave bounds of any PB of the edge are
  skipped (`aMVPB`, `:247-265`).
- Solver `ComputeVE(V,E, ->T, ->TolVNew, fuzz)`: project vertex point with cached `ProjPC(E)`;
  returns 0 ok; -1 degenerated; -2 no 3D curve; -3 projection failed; -4 `dist > tolV+tolE+max(fuzz,Confusion)`.
  Always outputs `T = LowerDistanceParameter` and the growth suggestion `TolVNew = dist + tolE`.
- Analysis (`:315-388`): only flag==0 accepted. `UpdateVertex(nV, aTolVNew)` (growth-only, `_10.cxx:105-160`;
  non-destructive mode copies the vertex into a new DS entry). Find the PB whose range STRICTLY contains T
  (`aT > aT1 && aT < aT2`, `:345-354`) — a parameter at a pave bound is never re-paved. `AppendExtPave`
  (extra pave on PB), record `BOPDS_InterfVE{nVOld, nE, T, IndexNew}` for every original vertex in the SD
  group, `AddInterf`.
- `SplitPaveBlocks(theMEdges)` (`_2.cxx:419-626`): for every modified edge, `PB->Update(newPBs)` splits at
  ext paves; per new PB: `UpdatePaveBlockWithSDVertices`, `FillShrunkData`; if no valid range — or PB
  unsplittable AND its two vertices interfere (`BOPTools_AlgoTools::ComputeVV(...)==0`,
  `BOPTools_AlgoTools.cxx:1772-1794`) — the two vertices are unified (`MakeSDVertices`) instead of keeping
  a zero-length edge (`:464-507`). Common-block preservation: new PBs are regrouped by bounding-vertex
  pair into new CommonBlocks (`MakeNewCommonBlock` `:401-415`); for CLOSED common blocks (nV1==nV2) the
  vertex pair is ambiguous, so membership is decided geometrically: midpoint of the first PB tested against
  each candidate PB with `ComputePE(PM, tolEFirst+tolE+fuzzy, E, ->TOut, ->Dist)` and `TOut` inside the PB
  range (`:571-616`, `IntTools_Context::ComputePE` at `IntTools_Context.cxx:437-495`).

### 2. PerformEE — edge-edge interference
- File: `BOPAlgo_PaveFiller_3.cxx:145-590`; solver `BOPAlgo_EdgeEdge` (`:53-137`) wraps `IntTools_EdgeEdge`.
- Pair prep (`:181-267`): per PB-vs-PB (not edge-vs-edge!) with box rejection. `GetPBBox` (`:914-955`):
  PB range must exceed `PConfusion`; box = shrunk box if present, else curve box over full range with
  `tolE + Confusion` gap, memoized in `aDMPBBox`. `bExpressCompute` when the two PBs have identical
  bounding-vertex index sets -> `UseQuickCoincidenceCheck`. Before `IntTools_EdgeEdge::Perform`, if the
  pair of boxes is far from origin `BOPAlgo_Tools::TrsfToPoint` translates BOTH edges to the origin to
  regain double precision (`:100-109`); common parts are re-anchored to the original edges after (`:120-128`).
- Solver `IntTools_EdgeEdge::Perform` (`IntTools_EdgeEdge.cxx:185-243`):
  1. `Prepare()` (`:89-181`): order curves by type complexity `TypeToInteger` (Line=0 < Hyp/Parab=1 <
     Circle/Ellipse=2 < Bezier/BSpline=3 < Other=4; ties broken by `CurveDeflection` — 10-sample summed
     tangent-turn, `:1611-1638`), possibly swapping (mySwap). Tolerances `myTol1=tolE1+fuzz/2`,
     `myTol2=tolE2+fuzz/2`, `myTol=myTol1+myTol2`. Per-curve resolution coefficients
     (`ResolutionCoeff` `:1486-1557`: circle `1/(2R)`, ellipse `1/Rmaj`, offset/other: min dt/dist over 30
     samples) and `Resolution(curve,type,coeff,tol3D)` (`:1561-1607`: line `tol`, circle `2*asin(coeff*tol)`,
     B-spline native `Resolution()`). Parametric tol `myPTol = 5e-13`, or `5e-16*max|T|` when `max|T| > 999`.
  2. Line/Line -> `ComputeLineLine` (`:902-1056`), fully analytic: coincident if angle `< Precision::Angular`
     and cross-distance `<= myTol^2` at both ends -> EDGE common part with clipped ranges (`AllNullFlag`
     set when edge1 fully inside edge2's range); else skew/cross distance test `|distLL| > myTol` -> none;
     point solution -> VERTEX with half-widths `aDt_i = IntTools_Tools::ComputeIntRange(tol_i, tol_j, angle)`
     (`IntTools_Tools.cxx:783-804`: `tol2` if angle≈90°, else `tol1*tan(pi/2-a) + tol2/sin(a)`).
  3. QuickCoincidence (`IsCoincident` `:247-286`): 24 samples (aNbSeg=23) of curve1 projected on curve2;
     if `>50%` land within `myTol` -> single EDGE common part over BOTH FULL RANGES; return.
  4. Line-vs-conic fast reject: `BRepExtrema_DistShapeShape` min distance `> 1.1*myTol` -> no solution (`:217-233`).
  5. `FindSolutions` (`:290-549`): closed-curve handling (split curve2 in 2, curve1 in 2 if closed, after a
     `CheckCoincidence` pre-test); recursive box-tightening kernel (`:353-549`): alternately clamp each
     curve's parametric range to the other's bounding box via `FindParameters` (`:553-671`) — a marching +
     golden-section (`aCf=0.618034`) bound refinement using `Resolution(...,dist)` as adaptive step (with
     step-growth heuristic `k*=2` when distance stagnates within 10%); stop when both ranges shrink by less
     than `max(range/250, res)`; a range is "thin" when `span < res` or its box is thin (`IsXThin&&IsYThin&&
     IsZThin` vs `myTol`). Thin ranges are validated by midpoint projection `<= myTol` then appended as a
     candidate range pair. Non-thin stalls run `CheckCoincidence` (`:1150-1206`: 10-segment express
     endpoint `DistPC` probes then golden-section `FindDistPC` max-dist per segment vs `myTol`; returns
     0=coincident -> range pair emitted as overlap) and `IsIntersection` (`:1060-1146`: endpoint pairing
     `dist^2 < (coef*myTol)^2` with coef 5000/adaptive, plus tangent-angle criterion 5e-3 rad, plus
     min-distance confirmation) before splitting range1 into 3 and recursing.
  6. `MergeSolutions` (`:675-776`): merge candidate range pairs whose gaps `< 20*Resolution(myTol)`;
     if a merged range spans (within `myRes`) the WHOLE of either input range -> the result becomes a
     single `TopAbs_EDGE` common part and all point solutions are discarded (`:758-769`). Otherwise each
     merged pair becomes `TopAbs_VERTEX` via `AddSolution` (`:780-822`, un-swap applied) and
     `FindBestSolution` (`:826-898`) computes the representative parameters: golden-section minimum of
     point-curve distance over 10 sub-segments, `aSolCriteria=5e-16`, touch detection `5e-13` — a
     confirmed touch interval collapses to its midpoint.
- Result analysis back in `PerformEE` (`:285-556`), per `IntTools_CommonPrt`:
  - **TopAbs_VERTEX** (`:369-527`): requires BOTH PBs splittable. `IntTools_Tools::VertexParameters`
    (`IntTools_Tools.cxx:593-611`) gives (T1,T2) — vertex parameter if inside Range1/Ranges2(1), else range
    midpoints. On-pave test with `aTol=Precision::Confusion()`: `bIsOnPave[j] = IsOnPave1(T, endBand, tol)
    || IsOnPave1(bandEdge, commonRange, tol)` for the 4 end bands `aR11=(T11,TS11), aR12=(TS12,T12),
    aR21, aR22` (`IsOnPave1` `IntTools_Tools.cxx:627-646`: inside-or-within-tol). If the point is on a pave
    of BOTH edges -> skip entirely (existing vertices already model it) (`:399-403`). If on a pave of ONE
    edge -> `ForceInterfVE(nV, otherPB, aMEdges)` (`_3.cxx:828-910`): re-runs `ComputeVE` accepting
    `iFlag==0 || iFlag==-4` (i.e. FORCES the interference even when the distance exceeds the tol sum,
    because EE evidence says the geometry crosses there), records InterfVE, grows the vertex, appends ext
    pave, warns `AlertSelfInterferingShape` if vertex and edge share rank. If any forced vertex exists
    (`isVExists`): distinguish touch vs crossing — points `C1(T1)` vs `C2(T2)` farther than
    `Precision::Intersection()` apart -> touching, do nothing; else grow each on-pave existing vertex by its
    actual distance to the new point (`UpdateVertex(nV[j], aDistPP)`) and add it to
    `myVertsToAvoidExtension` (`:423-452`), and DO NOT create a new vertex.
    Otherwise create `MakeNewVertex(E1,T1,E2,T2)` (`BOPTools_AlgoTools_2.cxx:224-250`: midpoint of the two
    curve points, tol = `max(tolE1,tolE2) + 0.5*dist`). Analytic Line/Circle pairs get a tolerance floor of
    half the common-range chord ("increase tolerance ... but do not update the vertex till its intersection
    with some other shape", `:455-466`). LXBR guard (`:468-510`): if the new point is within
    `sqrt(100)*(tolVnew+tolVx) = 10*(tolVnew+tolVx)` of a vertex SHARED by the two PBs -> drop (the shared
    vertex already represents the intersection). Survivors: append `BOPDS_InterfEE{nE1,nE2,CommonPart}`,
    `AddInterf`, and stash the candidate vertex in `aMVCPB` keyed by shape with a
    `BOPDS_CoupleOfPaveBlocks{PB1,PB2,iInterf,tolVnew}`.
  - **TopAbs_EDGE** (overlap) (`:529-551`): accepted only when it is the ONLY common part
    (`aNbCPrts == 1`) and `aPB1->HasSameBounds(aPB2)` (both PBs already bounded by the same two vertices —
    the VE/VV stages must have run first to make this true). Records InterfEE with the common part and
    links `PB1<->PB2` into `aMPBLPB` for common-block creation. NOTE: partial overlaps are NOT handled
    here — they only become common blocks after vertex unification lets both PBs share bounds (later
    ForceInterfEE pass, or next RepeatIntersection round).
- Post-treatment (`:558-589`): `BOPAlgo_Tools::PerformCommonBlocks(aMPBLPB)` fuses linked PB chains into
  `BOPDS_CommonBlock`s; `UpdateVerticesOfCB` (`:959-993`) pushes each CB's tolerance into its bounding
  vertices; `PerformNewVertices(aMVCPB, ...)` (below); finally `SplitPaveBlocks` for edges modified only
  by ForceInterfVE.

### 3. PerformNewVertices + TreatNewVertices — fusing EE/EF point results into DS vertices
- File: `BOPAlgo_PaveFiller_3.cxx:594-688` / `:692-723`. Runs at the end of PerformEE (EE mode) and
  PerformEF (EF mode, `bIsEEIntersection=false`).
- `TreatNewVertices`: cluster all candidate vertices by proximity — `BOPAlgo_Tools::IntersectVertices
  (verts+tols, myFuzzyValue, chains)` — and build ONE new vertex per chain (`MakeVertex`: barycenter,
  tolerance covering all members). This is transitive chaining, not grid welding.
- Each fused vertex is appended to the DS with box gap `tol + myFuzzyValue/2` (`:637-639`); every member's
  `CoupleOfPaveBlocks` gets the new index, and the originating InterfEE/InterfEF record gets
  `SetIndexNew(iV)` (`:641-652`).
- Then `aMPBLI: PB -> list of new vertices` is built and `IntersectVE(aMPBLI, theAddInterfs=false)` puts
  each new vertex as an ext pave on BOTH parent PBs (re-using the whole VE machinery: projection,
  strict-interior test, PB split, vertex unification for invalid ranges) (`:655-687`).

### 4. PerformVF — vertex-face interference
- File: `BOPAlgo_PaveFiller_4.cxx:139-301`; solver `BOPAlgo_VertexFace::Perform` `:102-119` ->
  `IntTools_Context::ComputeVF` (`IntTools_Context.cxx:545-590`).
- When: after PerformEE (so EE-created vertices exist). GlueFull mode: skip solving, only init FaceInfo.
- Pair filtering (`:181-232`): skip sub-shapes, pairs with interference, pairs whose sub-shapes already
  interfere (`HasInterfShapeSubShapes` — a vertex found on an edge of the face makes VF redundant);
  SD-resolve vertex; fence map keyed `BOPDS_Pair(nVSD,nF)` collecting the SD group.
- `ComputeVF(V,F, ->U,->V, ->TolVNew, fuzz)`: project point with cached `ProjPS(F)` (whole UV bounds,
  projection tol 1e-12, `Extrema_ExtFlag_MIN`); -1 not projectable; -2 `dist > tolV+tolF+max(fuzz,Confusion)`;
  -3 point ON SURFACE but classified OUT of the face (`IsPointInFace` -> `IntTools_FClass2d`, state must be
  IN, not ON — `IntTools_Context.cxx:604-608`); 0 ok. `TolVNew = dist + tolF`.
- Analysis (`:249-298`): record `BOPDS_InterfVF{nV,nF,U,V,IndexNew}` per SD member, `AddInterf`,
  `UpdateVertex(nV, aTolVNew)`, and register the vertex in `FaceInfo.VerticesIn` — the registry consumed
  by PerformEF's express checks and PerformFF's pave placement.
- `TreatVerticesEE` (`_4.cxx:305-390`): sweep all vertices CREATED by EE (`InterfEE.IndexNew`) against all
  faces (box-filtered `BOPDS_SubIterator`); those not already in `FaceInfo.VerticesOn` get a `ComputeVF`;
  success -> InterfVF + `VerticesIn`. Rationale: those vertices did not exist when the VF iterator ran.

### 5. PerformEF — edge-face interference
- File: `BOPAlgo_PaveFiller_5.cxx:165-592`; solver `BOPAlgo_EdgeFace` (`:55-157`) wraps `IntTools_EdgeFace`;
  geometry core `IntTools_BeanFaceIntersector`.
- Pair prep (`:219-307`): skip degenerate edges; per PB of the edge: skip PBs already registered as
  `PaveBlocksOn` the face (via `RealPaveBlock` — CB representative); box reject vs face box.
  `bExpressCompute` when BOTH PB vertices are among the face's `VerticesIn ∪ VerticesOn`.
  Range hygiene: BOTH the shrunk range and the full PB range are pulled inward by
  `BOPTools_AlgoTools::CorrectRange(E,F,...)` (`BOPTools_AlgoTools_2.cxx:364-434`): each end moves inward
  by the parametric image of `tolF` (divided by local D1 magnitude for B-spline/Bezier/offset/other, else
  `Resolution(tolF)`); reverted if the span would drop below `PConfusion`. This keeps boundary touching
  points owned by the vertices out of the solver. The corrected shrunk range is stored as `NewSR`.
  Every prepared pair is remembered in `myFPBDone[nF] += PB` so ForceInterfEF will not redo it.
  Same `TrsfToPoint` far-origin translation as EE (`:119-128`).
- Solver `IntTools_EdgeFace::Perform` (`IntTools_EdgeFace.cxx:504-685`):
  1. `myCriteria`: `tolE' + tolF'` where each includes `+fuzz/2`; EXCEPT B-spline/Bezier edges:
     `1.5*tolE' + tolF'`, and if the two tolerances differ by more than 100x -> `max(tolE', tolF')`
     (bug-5112 guard) (`:529-549`).
  2. QuickCoincidence (`IsCoincident` `:62-163`): 24 samples (only 3 for Line/Plane) of the edge projected
     onto the face; sample fails hard if `dist > 100*myCriteria` (early false); counts `dist <= myCriteria`;
     classification (`FClass2d`) of first/middle/last projectable samples must not be OUT; coincident when
     `>50%` of samples are on -> single EDGE common part covering `myRange`; done.
  3. `IntTools_BeanFaceIntersector` on the corrected PB range (`:565-570`;
     `IntTools_BeanFaceIntersector.cxx:288-379`): Line/Plane analytic (`ComputeLinePlane` `:820-906`,
     `Tolang=1e-9`; in-plane -> whole range; crossing -> range `t ± ComputeIntRange(beanTol,faceTol,angle)`
     clipped); `FastComputeAnalytic` (`:692-816`) closed-form coincidence/no-intersection for
     plane-vs-conic, cylinder-vs-line/circle, sphere-vs-line; `TestComputeCoinside` sampling; localized
     B-spline subdivision path (`ComputeLocalized`) when surface is Bezier/other/BSpline of degree>2;
     general path = `ComputeAroundExactIntersection` (`:564-688`: `IntCurveSurface_HInter` exact
     points/segments seed ranges; if >1 exact point, `myCriteria` is DROPPED to `3*Confusion` so distinct
     intersection points do not merge into one range; periodic surfaces re-adjust U/V by
     `GeomInt::AdjustPeriodic` before use) + `ComputeUsingExtremum` (`Extrema_ExtCS` per unresolved range)
     + `ComputeNearRangeBoundaries`; ranges flagged 2 are merged when the parametric gap `< PConfusion`.
     Also tracks `MinimalSquareDistance` for the no-solution case.
  4. Each result range is kept only if its intermediate point `IsProjectable` (projection within
     `myCriteria` AND classified in-or-on the face — `IsValidPointForFace`, `IntTools_Context.cxx:647-673`).
     Bounding points recorded.
  5. **Type decision** `MakeType` (`:304-359`): `AllNullFlag` -> EDGE. Else: chord between range end points
     `df1 > 2*myCriteria` AND range == whole edge range (within `Resolution(myCriteria)`) -> EDGE;
     otherwise VERTEX at `tm` (touch parameter from `CheckTouch` (`:363-500`): `Extrema_ExtCS` minimum on
     the range, boundary/midpoint `DistanceFunction` fallbacks, accepted if within `myCriteria` and interior);
     whole-range midpoint sanity `aPF.Distance(C(tm)) > 2*myCriteria` -> EDGE.
  6. Tangency reclassification (`:609-683`): Line/Cylinder and non-coplanar non-radius Circle/Plane pairs:
     EDGE results are re-tested with `CheckTouch` and demoted to VERTEX at the touch parameter (grazing
     line along a cylinder yields a point, not an overlap); VERTEX results refined by `CheckTouchVertex`
     (`:689-784`, interior-touch extremum with `aEpsT=8e-5`, `9e-5` for lines).
- Result analysis in `PerformEF` (`:324-571`):
  - **No common parts**: if solver's `MinimalDistance` is meaningful and `> tolE+tolF`, record
    `EdgeRangeDistance{T1,T2,dist}` in `myDistances[(nE,nF)]` — later consumed as gap evidence
    (RepeatIntersection / warnings).
  - **TopAbs_VERTEX** (`:406-543`): `VertexParameter` -> T; `MakeNewVertex(E,T,F)` (point on curve, tol
    `tolE+tolF+1e-12`). First, `ReduceIntersectionRange` (`:685-768`) shrinks the shrunk band using
    existing EE results: if an EE interference created a vertex equal to one of this PB's bounding vertices
    and its other edge belongs to this face, the EE common range on this edge is subtracted from
    `[TS1,TS2]` (prevents EF from re-finding an EE vertex near a pave). On-pave test with
    `aTolToDecide = 5e-8` against bands `aR1=(T1,TS1)`, `aR2=(TS2,T2)` via `IsInRange`
    (`IntTools_Tools.cxx:650-666`). Special promotion: point in BOTH bands — or Line/Plane and in one
    band — while BOTH PB vertices are already on/in the face (`CheckFacePaves` `:596-601`) -> the "point"
    is really a thin overlap: clone the common part, `SetType(TopAbs_EDGE)`, record InterfEF and link
    PB->face for a common block (`:421-439`). Single-sided on-pave: `ForceInterfVF(nV,nF)` (`:631-681`,
    accepts `iFlag==0 || iFlag==-2` — forces the VF interference even beyond tolerance, growing the
    vertex; adds to `VerticesIn`; self-interference warning). If still on-pave: touch-vs-cross test — real
    intersection iff projection distance of the new point onto F `< Precision::Intersection()`; crossing
    grows the existing pave vertex by `aDistPP` capped by `aMaxDist = 1e4*tolV` (and `min(...,0.1)` when
    `tolV < 0.01`), marks `myVertsToAvoidExtension`; either way no new vertex (`:459-503`).
    Otherwise: reject if an equal vertex already lies on the face (`CheckFacePaves(Vnew, VerticesOn)` via
    `ComputeVV`); `tolVnew = max(tolVnew, tolE, tolF)`; Line/Plane tolerance floor = half common range;
    final classification `IsPointInFace(P, F, tolVnew)`; survivors recorded as InterfEF + couple
    `(PB,PB,iX,tolVnew)` into `aMVCPB`.
  - **TopAbs_EDGE** (`:545-566`): record InterfEF; if either PB vertex is NOT among the face's on/in
    vertices the common part is dropped (interference kept, no common block — inconsistent CB protection);
    else `SetCommonPart` and PB->face linked into `aMPBLI`.
- Post (`:573-591`): `PerformCommonBlocks(aMPBLI)` — each PB obtains a CB carrying `Faces()` list;
  `UpdateVerticesOfCB`; `PerformNewVertices(..., false)`; `myDS->UpdateFaceInfoIn(aMIEFC)` refreshes the
  faces' In-sets with the new pave blocks/vertices.

### 6. RepeatIntersection — convergence loop
- File: `BOPAlgo_PaveFiller.cxx:359-422`. Re-runs PerformVV, PerformVE, PerformVF restricted to vertices
  whose tolerance grew during EE/EF (`myIncreasedSS`), with SD updates between. Guarantees the
  interference tables are transitively closed after tolerance growth.

### 7. ForceInterfEE — coincidence-only EE pass with extended fuzzy
- File: `BOPAlgo_PaveFiller_3.cxx:997-1333`. When: after RepeatIntersection, before FF.
- Purpose: "Now that we have vertices increased and unified, try to find additional common blocks among
  the pairs of edges" — ONLY overlaps (`TopAbs_EDGE`), points are done.
- Mechanism: init PBs for all vertices that took part in interferences; map `BOPDS_Pair(nV1,nV2) -> PBs`
  over REAL pave blocks (CB representatives); for every vertex-pair bucket with >= 2 PBs, all cross pairs
  are candidates. Same-rank pairs allowed only when the sharing of vertices was ACQUIRED during the
  operation (`:1149-1160`). Skip pairs already in one CB. Extended fuzzy:
  `aTolAdd = 2*max(tolV1,tolV2)` (self-intersection check mode: plain `myFuzzyValue`), but DISABLED when
  the tangents of the two edges at the mid-point differ by more than 25 deg (`|cos| < 0.9063`,
  non-line pairs, via `ProjPC` projection of the midpoint) — "may lead to undesired unification of edges"
  (`:1176-1205`). Runs `IntTools_EdgeEdge` with `UseQuickCoincidenceCheck(true)`. Accepts ONLY results
  that are a SINGLE common part of type EDGE (`:1281-1291`). Same-rank results emit
  `AlertAcquiredSelfIntersection`. New CBs merge with pre-existing CB members (`:1313-1329`).

### 8. ForceInterfEF — coincidence-only EF pass with computed extra fuzzy
- File: `BOPAlgo_PaveFiller_5.cxx:772-827` (driver) + `:831-1199` (worker).
- Candidates: every REAL PB (BVH tree over shrunk boxes) vs every face whose box overlaps and whose
  vertex set (`VerticesOn ∪ VerticesIn ∪ VerticesSc` plus pave vertices of the face's PB sets) CONTAINS
  BOTH PB vertices (`:955-964`). PB must not already be On/In/Sc of the face; edge and face must come from
  different arguments (unless the PB has a split edge). Mid-point screening: projection distance
  `<= aTolCheck + fuzzy` where `aTolCheck = 2*max(tolV1,tolV2)` (SI mode: fuzzy); UV must classify in-face.
  Angle guard: vector (projection->point) vs curve tangent must be within 25 deg of perpendicular
  (`|cos| <= 0.4226`), else no additional tolerance (`:1038-1052`). Extra fuzzy
  `aTolAdd = max(dist(C(TS_i), F)) - (tolE + tolF)` clamped `>= 0`, computed at the two shrunk ends
  (`:1054-1085`). Pairs with `aTolAdd == 0` that were already processed in PerformEF (`myFPBDone`) are
  skipped. Runs `IntTools_EdgeFace` with `fuzzy + aTolAdd`, quick-coincidence on, full PB range. Accepts
  ONLY a single EDGE common part; PB is added to the face's `PaveBlocksIn`, and (when `theAddInterf`)
  InterfEF + PB->face CBs are created (`:1147-1198`).

### 9. FF-side pave placement — PutPaveOnCurve / ExtendedTolerance (consumer of VE/EE/EF tables)
- File: `BOPAlgo_PaveFiller_6.cxx` (PerformFF/MakeBlocks call chain at `:808-825`).
- `PutPavesOnCurve` (`:2372-2421`): for each section curve NC of a face pair, with
  `aTolR3D = max(NC.Tolerance(), NC.TangentialTolerance())`: EF-created vertices (aMVEF) are placed FIRST
  with `iCheckExtend=2`; then all On/In vertices of the two faces — non-common vertices must be NEW shapes
  and box-overlap the curve — with `iCheckExtend=1`.
- `PutPaveOnCurve` (`:2959-3068`): `IsVertexOnLine(V, tolV, IC, aTolR3D + myFuzzyValue, ->T)`
  (`IntTools_Context.cxx:775-982`: tolSum = `2*(tolV+tolC)` floored at `1e-5` for B-spline/Bezier curves,
  `1e-6` otherwise; curve-end points preferred with `Extrema_LocateExtPC` refinement and global
  `Extrema_ExtPC` fallback, guarded against drifting past mid-curve; interior via `ProjPT`). On failure and
  `iCheckExtend != 0` and vertex not in `myVertsToAvoidExtension`:
  **`ExtendedTolerance`** (`:2542-2604`): only for NEW vertices; scans InterfEE (`iCheckExtend==1`) or
  InterfEF (`iCheckExtend==2`) records whose `IndexNew == nV` and whose BOTH parents belong to the shape
  set of this face pair; the vertex tolerance is extended to
  `max(dist(Pv, C1(Range1.First)), dist(Pv, C1(Range1.Last)))` — i.e. to the full extent of the recorded
  COMMON PART on edge 1. Retry `IsVertexOnLine` with the extended tol; on success the effective `tolV`
  becomes the ACTUAL distance point-to-curve (`:2977-2990`). Then: pave dedup
  `aPB->ContainsParameter(T, aPTol, ->nVUsed)` with `aPTol = GeomAdaptor_Curve(IC).Resolution(
  max(aTolR3D, aTolV))` -> reuse the existing pave and record the SD pair in `aDMVLV` (both vertices will
  be unified later); else `AppendExtPave` and grow the vertex to `dist + DTolerance() (=1e-12)`, extending
  its DS box by `+Confusion` (`:2992-3067`).
- `PutEFPavesOnCurve` (`:2692-2744`): only for Bezier/BSpline section curves: EF vertices not used
  anywhere yet are projected on the curve (`ProjPT`) and placed with band = actual distance.
- `PutStickPavesOnCurve` (`:2748-2843`): for curve ends WITHOUT bound vertices: "stick" vertices (all
  interference-created vertices of the pair, `GetStickVertices` `:2847-2905`) within
  `aDT2 = 2e-7` (squared) of a curve endpoint, where the two surface normals at the endpoint UV coincide
  (`1-|n1.n2| <= 5e-9` — the curve is a vanishing/tangential crease) are placed at the endpoint with
  band = distance.
- `FilterPavesOnCurves` (`:2437-2538`): a vertex paved onto several section curves keeps only curves with
  `sqDist <= 100*max(tolCurve^2, minSqDist)`; closer-but-oblique projections (`sin >= 0.5`) are kept
  anyway (bug 27761); if paves were removed, the vertex tolerance is REDUCED back to
  `sqrt(maxKeptSqDist) + Confusion` (direct write to `BRep_TVertex`).

## DATA STRUCTURES

- **`IntTools_Range`** (`IntTools_Range.hxx`): `{myFirst, myLast}` doubles. The universal currency of the
  subsystem: PB ranges, shrunk ranges, end bands, common-part ranges.
- **`IntTools_CommonPrt`** (`IntTools_CommonPrt.hxx:117-127`): THE result record of EE/EF solvers.
  Fields: `myEdge1, myEdge2` (EF uses only Edge1); `myType` — `TopAbs_VERTEX` = point intersection,
  `TopAbs_EDGE` = overlap/coincidence range (the entire point-vs-common-part distinction of the task);
  `myRange1` (range on edge1); `myRanges2` (SEQUENCE of ranges on edge2 — closed-curve solutions can
  produce two); `myVertPar1/myVertPar2` (representative point parameters for VERTEX type);
  `myAllNullFlag` (edge1 completely inside the solution — forces EDGE type); `myPnt1/myPnt2` bounding
  points (EF sets these for later use).
- **`BOPDS_Pave`**: `{Index (DS vertex), Parameter}`; ordered by parameter within a PB.
- **`BOPDS_PaveBlock`**: `{Pave1, Pave2, OriginalEdge, Edge (split result), ExtPaves list, ShrunkData
  (TS1,TS2,Bnd_Box,IsSplittable)}`. Key methods: `AppendExtPave`, `RemoveExtPave`, `IsToUpdate`,
  `Update(newPBs)` (split at ext paves), `ContainsParameter(T, PTol, ->nV)` (pave dedup),
  `HasSameBounds(other)` (gate for EE overlap acceptance), `IsSplittable`.
  The PB — not the edge — is the unit of intersection in EE/EF.
- **`BOPDS_CommonBlock`**: `{PaveBlocks list (one per edge sharing the geometry), Faces list (faces the
  block lies IN), representative PaveBlock1(), Tolerance()}`. Built by `PerformCommonBlocks` from linked
  maps `PB<->PB` (EE) or `PB->face` (EF). This is the "shared section edge" primitive: downstream, one
  edge represents all coincident PBs, and `UpdateVerticesOfCB` pushes the CB tolerance into its vertices.
- **`BOPDS_InterfVE/VF/EE/EF`** (`BOPDS_Interf.hxx`): typed rows `{Index1, Index2, IndexNew, payload}` —
  VE: parameter T; VF: (U,V); EE/EF: full `IntTools_CommonPrt`. `IndexNew` = DS index of the vertex the
  interference produced (after fusion) — this is what `ExtendedTolerance`, `TreatVerticesEE`,
  `ReduceIntersectionRange`, and `GetStickVertices` traverse.
- **`BOPDS_CoupleOfPaveBlocks`**: `{PB1, PB2, IndexInterf, Index, Tolerance}` — staging record binding a
  candidate new vertex (map key = TopoDS vertex) to its parent PBs and its interference row, so that after
  fusion the row's IndexNew and the PB ext paves can be fixed up.
- **`BOPDS_FaceInfo`**: `{VerticesOn, VerticesIn, VerticesSc; PaveBlocksOn, PaveBlocksIn, PaveBlocksSc}`
  (On = from the face's own boundary; In = interior by interference; Sc = section results). The registry
  that makes EF/FF incremental: express checks, duplicate suppression, ForceInterfEF candidate filter.
- **`IntTools_Context`** (`IntTools_Context.cxx`): per-operation cache of stateful solvers keyed by shape:
  `ProjPS(face)` (surface projector, tol 1e-12, min-only), `ProjPC(edge)` / `ProjPT(curve)` (curve
  projectors), `FClass2d(face)` (2D classifier), `SurfaceAdaptor`, `Hatcher`, `BndBox`, `OBB`,
  `SolidClassifier`. Shared across all parallel solvers of a stage — the reason identical queries are cheap.
- **`IntTools_ShrunkRange`**: `{TS1, TS2, BndBox, IsDone, IsSplittable, Length}` — see stage 0.
- **`EdgeRangeDistance`** + `myDistances: (nE,nF) -> list` (PaveFiller member): minimal-distance evidence
  for EF pairs that produced NO common part (gap diagnosis / later repair).
- **`myVertsToAvoidExtension`** (PaveFiller member): vertices whose tolerance was already grown to cover a
  near-pave intersection; excluded from `ExtendedTolerance` retries (prevents double growth).
- **`myFPBDone: nF -> set<PB>`**: EF pairs already solved in PerformEF; ForceInterfEF re-solves them only
  with a positive computed `aTolAdd`.
- **`aMVTol: nV -> original tol`** and **`aDMVLV: nV -> SD list`** (FF stage locals): rollback of vertex
  tolerances when a pave turns out unused, and vertex-unification worklist from pave dedup.
- **`myIncreasedSS`**: vertices whose tolerance grew — drives RepeatIntersection.

## CONSTANTS & TOLERANCES

Precision (OCCT `Precision`): `Confusion()=1e-7`, `PConfusion()=1e-9`, `Intersection()=1e-9`
(Confusion/100... actually `Confusion()*0.01`), `Angular()=1e-12`, `SquareConfusion()=1e-14`;
`gp::Resolution()` = smallest positive double (~1e-308). `BOPTools_AlgoTools::DTolerance() = 1e-12`.

Solver acceptance bands (all include the fuzzy value):
- `ComputeVE`: accept `dist <= tolV + tolE + max(fuzz, 1e-7)`; suggested new vertex tol `= dist + tolE`
  (`IntTools_Context.cxx:532-535`).
- `ComputeVF`: accept `dist <= tolV + tolF + max(fuzz, 1e-7)`; new tol `= dist + tolF`; plus FClass2d
  in-face (`:573-589`).
- `ComputePE`: `tolP + tolE + 1e-7`; off-curve fallback checks edge vertices with their own tols (`:460-489`).
- `ComputeVV` (`BOPTools_AlgoTools.cxx:1772-1794`): `dist^2 <= (tolV1 + tolV2 + max(fuzz,1e-7))^2`.
- EdgeEdge: `myTol_i = tolE_i + fuzz/2`, `myTol = myTol1 + myTol2`; parametric tol `myPTol = 5e-13`
  (`5e-16 * max|T|` when `max|T| > 999`) (`IntTools_EdgeEdge.cxx:149-180`).
- EdgeFace: `myCriteria = (tolE + fuzz/2) + (tolF + fuzz/2)`; B-spline/Bezier edge: `1.5*tolE + tolF`;
  tol ratio > 100 -> `max(tolE, tolF)` (`IntTools_EdgeFace.cxx:529-549`).
- BeanFaceIntersector: `myCriteria = beanTol + faceTol` (+`1e-7` in the Edge/Face Init overload);
  multi-exact-point mode drops it to `3 * 1e-7` (`IntTools_BeanFaceIntersector.cxx:582`);
  `ComputeLinePlane` `Tolang = 1e-9`; result-range merge gap `PConfusion`.

Classification / decision constants:
- Coincidence sampling (both `IsCoincident`s): `aNbSeg = 23` (24 samples; Line/Plane EF: 2 -> 3 samples),
  acceptance ratio `aTresh = 0.5`; EF hard reject per sample at `100 * myCriteria`; EF boundary shift
  `1% of range` before sampling; classification indices `[25%, 75%]` window
  (`IntTools_EdgeEdge.cxx:247-286`, `IntTools_EdgeFace.cxx:62-163`).
- EE on-pave: `Precision::Confusion()` (`_3.cxx:382`); EF on-pave: `aTolToDecide = 5e-8` (`_5.cxx:416`).
- EE new-vertex-vs-shared-vertex guard: `dist^2 < 100 * (tolVnew + tolVx)^2` (`_3.cxx:497`).
- EE touch-vs-cross: `Precision::Intersection()` between the two curve points (`_3.cxx:432`);
  EF touch-vs-cross: projection distance `< Precision::Intersection()` (`_5.cxx:472`).
- EF existing-vertex growth cap: `aMaxDist = 1e4 * tolV`, additionally `min(aMaxDist, 0.1)` when
  `tolV < 0.01` (`_5.cxx:490-494`).
- Analytic tolerance floors: Line/Circle EE and Line/Plane EF vertex tol >= half the common-range span
  (`_3.cxx:455-466`, `_5.cxx:513-519`).
- MakeNewVertex: (E,T,E,T) and (V,V): midpoint, `tol = maxTol + 0.5*dist`; (E,T,F):
  `tol = tolE + tolF + 1e-12` (`BOPTools_AlgoTools_2.cxx:187-271`).
- EdgeEdge search: golden ratio `0.6180339887498948`; stagnation-step growth `k *= 2` while
  `|dPrev-d|/dPrev < 0.1` and step < 1% of range; termination shrink threshold `range/250`;
  `IsIntersection` endpoint criterion `aCoef in {5000, adaptive tail, 1} * myTol`, tangent-angle criterion
  `5e-3` rad; `CheckCoincidence` express segments 10; `FindBestSolution` `aSolCriteria = 5e-16`,
  `aTouchCriteria = 5e-13`; `MergeSolutions` gap `20 * Resolution(myTol)`; whole-range-coincidence
  detection within `myRes_i`.
- Line/Line: coincidence angle `Precision::Angular()`; vertex half-width
  `ComputeIntRange(tol1,tol2,angle) = tol1*tan(pi/2-a) + tol2/sin(a)` (or `tol2` at ~90 deg)
  (`IntTools_Tools.cxx:783-804`).
- EdgeFace MakeType: EDGE iff chord `> 2*myCriteria` over the whole range; `CheckTouchVertex`
  `aEpsT = 8e-5` (`9e-5` for lines).
- CorrectRange inward pull: EF `res = tolF`; EE `res = 2*(tolE1+tolE2)`; converted to parametric by
  `res/|D1|` (irregular curves, guard `|D1| > 1e-12`) or `Resolution(res)`; span floor `PConfusion`.
- ShrunkRange: splittable iff `length > 2*tolE + 2*1e-7`; box gap `tolE + 1e-7` `+ fuzz/2` (PaveFiller).
- ForceInterfEE: extra fuzzy `2 * max(tolV1, tolV2)`; tangent-alignment gate `|cos| >= 0.9063` (25 deg).
- ForceInterfEF: mid distance gate `2*max(tolV1,tolV2) + fuzz`; perpendicularity gate `|cos| <= 0.4226`
  (65-115 deg window); `aTolAdd = max(endDist) - (tolE + tolF)`, clamped >= 0.
- PutPaveOnCurve family: `aTolR3D = max(curveTol, tangentialTol)`; `IsVertexOnLine` band
  `2*(tolV + tolC)` floored at `1e-5` (B-spline/Bezier) / `1e-6`; pave dedup parametric band
  `Resolution(max(aTolR3D, tolV))`; vertex growth to `dist + 1e-12`; stick-pave endpoint band
  `sqrt(2e-7)`, crease normal criterion `1 - |n1.n2| <= 5e-9`; pave filter factor `100 *` square distance,
  oblique-keep `sin >= 0.5`.
- New-vertex fusion: chain clustering fuzz = `myFuzzyValue`; DS box gap `+ fuzz/2`; `UpdateVertex` box gap
  `+ 1e-7`.
- EE far-pair fast reject: `BRepExtrema` distance `> 1.1 * myTol` (line-vs-analytic only).

## INVARIANTS

1. **Order dependency**: VE runs after VV, EE after VE, VF after EE, EF after VF+EE. Each stage may assume
   every lower-dimensional interference is already recorded and every affected vertex already SD-unified
   (`UpdatePaveBlocksWithSDVertices` between stages). EE's overlap acceptance (`HasSameBounds`) is only
   correct because VV/VE already identified shared endpoints.
2. **Paves are strictly interior**: an ext pave is only appended when its parameter is strictly inside the
   PB range; intersections at/near pave bounds are ALWAYS represented by growing an existing vertex, never
   by a new coincident vertex. Downstream never sees two vertices closer than their summed tolerances on
   one edge (LXBR guard + ContainsParameter dedup).
3. **Tolerances only grow during intersection** (`UpdateVertex` takes max; growth recorded in
   `myIncreasedSS` and re-propagated by RepeatIntersection). The single exception is
   `FilterPavesOnCurves`, which may shrink a vertex back AFTER pave placement is final.
4. **Boxes are conservative**: every DS box gets `+Confusion` (+`fuzz/2`) gap after any vertex/edge
   update, so box-rejection tests never produce false negatives w.r.t. the tolerance model.
5. **Every EDGE-type common part becomes a CommonBlock**; after PerformCommonBlocks each geometric locus
   has ONE representative PB (`RealPaveBlock`), and CB tolerance has been pushed into the bounding
   vertices. FF (and Builder) may treat coincident edges as a single edge.
6. **Point results are globally fused before entering the DS** (`TreatNewVertices` transitive chains with
   fuzzy), and every InterfEE/EF row's `IndexNew` points at the fused vertex — so `ExtendedTolerance` and
   friends can reconstruct "which common part created this vertex".
7. **Every new PB has valid shrunk data or was eliminated** (vertices unified). No zero-length or
   sub-tolerance pave blocks survive `SplitPaveBlocks`.
8. **VF/EF results are face-interior**: `ComputeVF` and the EF VERTEX path classify against the trimmed
   face (FClass2d / `IsPointInFace`), so `VerticesIn`/`PaveBlocksIn` never contain boundary duplicates
   (those are `VerticesOn` via the face's own topology).
9. **Determinism/idempotence of pair processing**: fence maps at every level (pair maps, `myFPBDone`,
   `aMPBFence`, SD-keyed maps) guarantee each geometric pair is solved exactly once per pass, and
   interference tables make later passes skip solved pairs.

## PITFALLS

- **Far-from-origin geometry**: EE and EF solvers translate both operands to the origin
  (`BOPAlgo_Tools::TrsfToPoint`) before intersecting and re-anchor results after — double precision decays
  with distance from origin (`_3.cxx:100-109`, `_5.cxx:119-128`).
- **Closed edges / closed common blocks**: vertex-pair keys are ambiguous (nV1==nV2); SplitPaveBlocks
  falls back to geometric midpoint coincidence (`ComputePE`) to regroup CB fragments (`_2.cxx:555-616`).
  EdgeEdge splits closed curves into 2 segments before the box recursion (and checks whether curve2's seam
  point lies in curve1's box) (`IntTools_EdgeEdge.cxx:298-349`).
- **Micro edges**: unsplittable PBs are excluded from VE and from EE vertex creation; if a split would
  create a range shorter than the tolerance spheres, the bounding vertices are UNIFIED instead
  (`_2.cxx:464-507`) — the source of OCCT's "vertices swallow tiny edges" behavior.
- **Touch vs crossing at a pave**: both EE and EF explicitly distinguish a tangential touch (do nothing)
  from a true crossing near an existing vertex (grow that vertex, `myVertsToAvoidExtension`), using
  `Precision::Intersection()` as the discriminator (`_3.cxx:423-452`, `_5.cxx:459-503`).
- **Tangential analytic pairs**: Line/Circle (EE) and Line/Plane (EF) intersections carry a tolerance floor
  of half the common range, deliberately NOT written into the vertex until corroborated by another
  interference ("do not update the vertex till its intersection with some other shape").
- **Line/Cylinder and Circle/Plane EF**: EDGE results are re-audited by `CheckTouch` and demoted to VERTEX
  for grazing contact; VERTEX results refined by `CheckTouchVertex` (`IntTools_EdgeFace.cxx:609-683`).
- **EF thin-sliver promotion**: a VERTEX result sitting in BOTH pave bands (or one band for line/plane)
  while both PB vertices are already on the face is re-typed to EDGE — the "point" was a collapsed overlap
  (`_5.cxx:421-439`).
- **EF EDGE with un-registered vertex**: interference recorded but common part dropped when a PB vertex is
  not on the face — prevents a CommonBlock whose bounding vertex is not actually shared (`_5.cxx:549-559`).
- **Forced interferences beyond tolerance**: `ForceInterfVE` accepts `ComputeVE == -4` and `ForceInterfVF`
  accepts `ComputeVF == -2` (distance beyond tol sum) when EE/EF evidence demands the incidence — the
  tolerance model bends to topological evidence, with self-interference warnings when both operands come
  from the same argument.
- **`ReduceIntersectionRange`**: EF must subtract EE common ranges near paves or it re-discovers EE
  vertices as new EF vertices (`_5.cxx:685-768`).
- **Unprojectable middle points** in coincidence sampling: classification widened to the [25%,75%] index
  window because the exact midpoint may fail to project (`IntTools_EdgeFace.cxx:122-141`).
- **Periodic surfaces**: exact intersection UVs re-adjusted by `GeomInt::AdjustPeriodic` before range
  seeding (`IntTools_BeanFaceIntersector.cxx:598-648`).
- **Angle guards against false unification**: EE-coincidence extra fuzzy disabled beyond 25 deg tangent
  divergence; EF-coincidence extra fuzzy disabled outside the 65-115 deg tangent-vs-normal window;
  `FilterPavesOnCurves` keeps oblique projections (`sin >= 0.5`) even when far. All three exist because
  extended tolerances otherwise weld nearly-parallel but distinct geometry.
- **Multiple exact EF points**: BeanFace drops `myCriteria` to `3*Confusion` when the exact intersector
  returns >1 point, so adjacent transversal points do not merge into a bogus overlap range.
- **B-spline vs analytic tolerance asymmetry** (EF criteria 1.5x / ratio-100 rule) — protects pairs with
  wildly mismatched tolerances from producing garbage overlap verdicts.
- **`IsVertexOnLine` endpoint preference**: curve ends are checked before interior projection, with local
  extrema refinement that is explicitly guarded against drifting past mid-curve — vertices near section
  curve ends must snap to the end, not to an interior projection.
- **`aEEs`/`aEFs` growth invalidation**: interference rows are stored by index (`IndexInterf` in the
  couple) precisely because the arrays are append-only during a stage.

## PORT MAP

Anchors: `session_cpp/src/brep_section.cpp` (`build_section_scaffold`: one SSI per pair, correct7
corrector, chains (p3,uvA,uvB), stage-1b PutToBoundary, paves = trim-loop crossings + chain-chain
crossings + vertex projections, per-interval keep-verdict, micro filter, valence-1 bridge, 3D weld);
`session_cpp/src/brep.cpp` (`split_with`: per-operand UV arrangement, runs lift exact subranges,
whole-segment runs keyed seg_id, zero-span collapse; `combine`: 1e-7 weld, vertex-pair-keyed common-block
tube merge, NK-RESCUE 0.15*tol3, micro-edge collapse, winding/radial/flood/parity classification,
face_outward_signs). Tolerances: `tol3 = diag*2e-3` weld band, `tol3_rep` representation band, no growth
model. Missing today: same-domain/coincidence subsystem, first-class EE/EF stages, per-entity tolerances,
history, n-ary fuse.

| # | OCCT mechanism | Our anchor | Action | Design (for new builds) |
|---|---|---|---|---|
| 1 | Stage ordering VV->VE->EE->VF->EF->(repeat)->Force*->FF, with interference tables gating later stages | `build_section_scaffold` runs SSI directly; paves computed inside one pass | **adopt** (restructure) | Split pave sourcing into ordered sub-passes with a shared interference registry (pair -> record + new-vertex id) consulted before creating any pave; enables skip/dedup semantics we currently approximate with 3D welds. |
| 2 | `ComputeVE` return-code discipline + growth suggestion `tolNew = dist + tolE` (`IntTools_Context.cxx:499-541`) | vertex projections in `build_section_scaffold` pave step | **replace** | Replace fixed tol3-band accept with `dist <= tolV+tolE+max(fuzz,eps)` and carry `dist+tolE` as the vertex's new tolerance instead of discarding the distance. |
| 3 | ShrunkRange (`IntTools_ShrunkRange.cxx:107-191`): end bands `[T,TS]`, `IsSplittable = len > 2*tolE+2*eps` | per-interval keep-verdict (bisected ends) in `build_section_scaffold`; micro filter | **adopt** | Compute per-interval shrunk range (subtract endpoint tolerance spheres via arc-length march) BEFORE the 9-sample keep-verdict; classify only the shrunk core; mark unsplittable intervals so no pave is ever placed inside an endpoint band. |
| 4 | Pave-on-bound discipline: `IsOnPave1` bands + snap-to-existing-vertex + touch-vs-cross via `Precision::Intersection` (`_3.cxx:369-452`, `_5.cxx:406-503`) | chain-chain crossing paves + 3D weld | **new build** | When a crossing parameter falls inside an interval's end band (band width = shrunk gap, floor 1e-7): do NOT emit a pave; test cross-vs-touch by 3D distance of the two chain points; crossing -> grow the existing weld-vertex tol to the actual distance and flag it no-further-extension; touch -> drop. Direct guard against duplicate near-endpoint vertices that currently force NK-RESCUE. |
| 5 | EE VERTEX/EDGE classification: recursive box-tightening + `CheckCoincidence` + whole-range detection in `MergeSolutions` + 23-sample `IsCoincident` (`IntTools_EdgeEdge.cxx`) | MISSING first-class EE stage; nearest: SEG-UNIFY / whole-seg alias keyed seg_id in `split_with`/`combine` | **new build** | Chain-vs-chain overlap solver: 24-sample projection of chain A onto chain B with `tol = tolA+tolB(+fuzz)`; ratio > 0.5 -> whole-overlap -> alias both to one seg_id (existing alias machinery); partial -> range-merge with gap `20*res` and emit either a shared subrange (common block) or a crossing pave via golden-section min-distance (best-solution 5e-16/touch 5e-13 discipline). This is the OCCT-shaped fix for the SEGLOST one-sided whole-segment loss. |
| 6 | CommonBlock: PB list + faces + ONE representative + CB tolerance pushed into vertices (`_2.cxx:401-415`, `_3.cxx:558-563,959-993`) | vertex-pair-keyed common-block tube merge in `combine` | **already-equivalent (partial) / adopt growth** | Keep tube merge; add CB tolerance = max mate deviation across merged tubes, propagated into the two endpoint vertices (growth-only), replacing the flat 0.15*tol3 NK-RESCUE band with evidence-based per-tube tolerance. |
| 7 | `PutPaveOnCurve` + `ExtendedTolerance` retry (band grown to the recorded common-part extent; success re-measures actual distance) (`_6.cxx:2959-3068, 2542-2604`) | vertex projections onto chains, 3D-welded across pairs | **new build** | Two-phase pave placement: phase 1 strict band `2*(tolV+tolC)` floored 1e-6/1e-5; phase 2 only for vertices CREATED by our own crossing/EF-analog records: extend band to the stored common-part span end-distance, retry, and on success set band = actual distance. Requires mechanism #1's registry (records must remember their Range1). |
| 8 | Pave dedup by parametric resolution: `ContainsParameter(T, Resolution(max(tolC,tolV)))` -> reuse + SD-unify (`_6.cxx:3004-3039`) | exact 1e-7 vertex weld in `combine` | **replace** | On each chain, dedup paves parametrically with band = chain-local `Resolution(max(tol3_rep, tolV))` (arc-length/param scale), unifying the vertices (SD list) instead of relying on the global 3D 1e-7 weld to catch them. |
| 9 | `TreatNewVertices` transitive fuzzy chaining + barycentric covering vertex (`_3.cxx:692-723`) | exact 1e-7 weld | **replace** | Replace grid/exact weld for NEW intersection vertices with chain clustering at `tolA+tolB+fuzz` per pair and covering-tolerance barycenter; keep 1e-7 exact weld only for operand-input vertices. |
| 10 | `SplitPaveBlocks` no-valid-range -> vertex unification (`MakeSDVertices`) (`_2.cxx:464-507`) | zero-span collapse (SESSION_ZEROFILL) + micro-edge collapse | **adopt** | When a lifted run subrange collapses below tolerance, UNIFY its two endpoint vertices (rewrite all runs referencing either) rather than dropping the run — removes the SEGLOST hole class caused by one-sided drops. |
| 11 | EF solver: BeanFace range manager + `MakeType` (chord > 2*criteria & whole-range -> EDGE; else VERTEX via CheckTouch) + tangency demotion (`IntTools_EdgeFace.cxx:304-683`) | MISSING first-class EF stage; nearest: SESSION_EXT_TRIM trim-crossing extension | **new build** | Trim-edge-vs-opposite-face marcher: sample trim edge against opposite surface with `criteria = tolE+tolF`, build ON-ranges (range manager = flag array over param intervals, merge gap = param eps); classify each range: whole-range + chord > 2*criteria -> feed as whole-segment run (seg_id alias, mechanism #5); short/touch -> single pave at extremum parameter. Reuse correct7 as the local distance minimizer. |
| 12 | FaceInfo VerticesOn/In + PaveBlocksOn/In registry consulted by EF/FF (`_4.cxx:294-297`, `_5.cxx:237-241`) | per-operand UV arrangement knows trims only implicitly | **new build** | Per-face registry {vertices-on (from trim loops), vertices-in (from projections/EF)} filled once, consulted before pave creation (mechanism #4's "already represented" test) and by the EF marcher's express path (both endpoints registered -> quick-coincidence sampling first). |
| 13 | `ComputeVF` = project + in-face classify, with distinct "on surface but out of face" verdict (`IntTools_Context.cxx:545-590`) | vertex projections + winding contains in `combine` | **already-equivalent** | Our winding contains covers classification; keep the -2/-3 distinction (distance fail vs classification fail) in the return value for diagnostics. |
| 14 | `ReduceIntersectionRange`: EE common ranges shrink the EF acceptance window near paves (`_5.cxx:685-768`) | UV-arrangement cut-node crossing snap | **adopt** | Before accepting an EF-marcher touch point near an interval end, subtract any recorded chain-chain common range attached to that endpoint's vertex; snap into the existing vertex instead. |
| 15 | ForceInterfEE: post-weld coincidence hunt among edge pairs sharing BOTH endpoint vertices, extra fuzzy `2*max(tolV)`, 25-deg tangent gate, accept single-EDGE only (`_3.cxx:997-1333`) | MISSING same-domain subsystem; nearest: cross-pair SEG-UNIFY | **new build (TOP PRIORITY)** | After combine's vertex weld: bucket all section/trim edges by (v1,v2) welded-vertex pair; for each bucket pair not already aliased, run mechanism #5's 24-sample coincidence with band `tol3 + 2*max(tolV)` gated by mid-point tangent alignment `|cos| >= 0.9063`; positives alias to one seg_id (and merge tubes). This is precisely the missing pass for one-sided whole-segment loss (task SEGLOST). |
| 16 | ForceInterfEF: PB-vs-face with both vertices registered on face, computed `aTolAdd = endDist-(tolE+tolF)`, perpendicularity gate (`_5.cxx:831-1199`) | MISSING | **new build** | Same bucket idea vs faces: edge whose both endpoint vertices lie in a face's registry (mechanism #12) but which is not aliased into the face -> run EF marcher with fuzzy + computed end-distance excess; accept only whole-edge ON verdicts; register run as face-interior segment. Catches near-tangent shared walls the SSI march misses. |
| 17 | `UpdateVertex` growth-only per-entity tolerance + `myIncreasedSS` + RepeatIntersection convergence loop (`_10.cxx:105-160`, `PaveFiller.cxx:359-422`) | no growth model (flat tol3/tol3_rep) | **new build** | Add `tol` field to combine's vertex records (init = weld band actually used); every mechanism above grows it monotonically; a second pave/classification pass re-runs only entities whose tol grew. Also feed vertex tol into STEP writer dedup. |
| 18 | `FilterPavesOnCurves`: per-vertex cross-curve distance audit, drop paves > 100x band unless oblique (sin>=0.5), then SHRINK vertex tol to max kept distance (`_6.cxx:2437-2538`) | vertices 3D-welded across pairs can attach to multiple chains | **new build** | After pave placement across all pairs: for each vertex on >1 chain, compute distance to each chain at its pave parameter; remove paves beyond `10x` the min unless the projection is oblique; shrink the vertex tol back to the max kept distance. Prevents tolerance inflation from spurious cross-pair attachments. |
| 19 | `MakeNewVertex` midpoint + `tol = maxTol + 0.5*dist` (`BOPTools_AlgoTools_2.cxx:224-250`) | CASE B cross-pair junction weld (valence-1 bridge) | **adopt** | Use the OCCT formula for junction vertices created from two chains: position = midpoint of the two chain points, tol = max(chain reps) + half separation — replaces implicit tol3 assumption. |
| 20 | `CorrectRange` inward pull by parametric image of tolF before EF solving (`BOPTools_AlgoTools_2.cxx:364-434`) | stage-1b PutToBoundary + SESSION_EXT_TRIM trim-crossing extension | **adopt** | When extending chains to trim crossings, stop the extension short of the crossing by the parametric image of tolF (res/|D1|) and let the vertex band own the remainder — avoids boundary-touch duplicate paves at trim loops. |
| 21 | `myDistances` near-miss record for EF pairs with no common part (`_5.cxx:350-360`) | SESSION_WIREGAP audit (manual) | **adopt (diagnostic)** | During the EF marcher and SSI march, record min distance for pairs that produce nothing but come within `k*tol3`; surface in the wire-gap audit to explain naked edges. |
| 22 | Analytic special-casing: Line/Line closed form, Line/Circle & Line/Plane half-range tolerance floors, Line/Cylinder & Circle/Plane tangency demotion | our analytic recognizer (recognize_solid kinds) handles surface pairs; curve-level analytics absent | **adopt (selective)** | In mechanisms #5/#11, branch analytic curve types first (line/circle vs line/plane/cylinder): closed-form intersection with `ComputeIntRange` half-widths; apply the do-not-commit tolerance-floor rule for tangential cases. |
| 23 | TrsfToPoint far-origin translation (`_3.cxx:100-109`) | correct7/SSI work in world coords | **adopt (conditional)** | If far-origin STEP models show corrector stalls: translate both surfaces by the shared box centroid before the march, add it back on output. Cheap, zero-risk. |
| 24 | Fuzzy value plumbing: user fuzz split `fuzz/2` per operand into every band | tol3 fixed `diag*2e-3` | **replace** | Introduce an explicit fuzz parameter (default derived from diag) added as `max(fuzz, eps)` into every acceptance band from mechanisms #2-#16, decoupling "user slack" from "weld representation band" (tol3_rep stays). |

**Priority order for our kernel** (highest leverage first): #15 (ForceInterfEE-style same-domain pass —
direct SEGLOST fix), #17+#6 (per-entity growth-only tolerance + CB tolerance), #4+#8 (pave-on-bound
snap + parametric pave dedup), #5 (EE overlap classifier feeding seg_id alias), #11+#16 (EF marcher +
forced EF), #3 (shrunk-range end bands), #10 (unify-instead-of-drop on collapse).
