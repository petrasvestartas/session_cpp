# OCCT spec: IntPolyh seeding — polyhedral pre-intersection for PrmPrm marching

Sources read (OCCT @ C:/brg/compas_occt/external/occt/src/occt/src/ModelingAlgorithms/TKGeomAlgo):
`IntPolyh/IntPolyh_Intersection.{hxx,cxx}`, `IntPolyh/IntPolyh_MaillageAffinage.{hxx,cxx}` (3997 ln),
`IntPolyh/IntPolyh_Tools.cxx`, `IntPolyh/IntPolyh_Triangle.cxx`, `IntPolyh/IntPolyh_StartPoint.{hxx,cxx}`,
`IntPolyh/IntPolyh_Couple.hxx`, `IntPatch/IntPatch_PrmPrmIntersection.cxx` (consumer, ln 2488–3198),
`IntPatch/IntPatch_Intersection.cxx` (defaults), `IntCurveSurface/IntCurveSurface_ThePolyhedronOfHInter.cxx` (deflection).

Related specs (referenced, not duplicated): `kb/occt_ssi-walking.md` (PWalking internals — this spec stops at
`PW.Perform`), `kb/occt_pavefiller-core.md` (downstream consumption of WLines), `kb/occt_tolerance-model.md`.

This is the mechanism by which OCCT **never misses a section branch**: an exhaustive triangle-triangle
interference soup enumerates every connected component of the intersection set BEFORE any Newton march;
each component contributes its own seed(s). Our single-seed-per-SSI-pair strategy has no such enumerator —
the (18,5) mini-branch class is exactly a second connected component that our one seed never reaches.

---

## STAGE PIPELINE

Ordered; caller is `IntPatch_PrmPrmIntersection::Perform(S1,D1,S2,D2,TolTangency,Epsilon,Deflection,Increment,ClearFlag)`
(`IntPatch_PrmPrmIntersection.cxx:2488`). Applies when both surfaces are parametric (no analytic recognizer) —
i.e. our imported_freeform x imported_freeform class.

**S0. Sampling parameters** — `IntPatch_PrmPrmIntersection.cxx:2498-2515`
- `D1->SamplePnts(Deflection, 10, 10)`; `D2->SamplePnts(...)` — Adaptor3d_TopolTool computes a
  deflection-driven, knot-aware sample count per direction (min 10x10). U/V parameter arrays are pulled with
  `D1->UParameters/VParameters` — NON-uniform (denser at knots/curvature).
- If `NbU1*NbV1 > 2500` or `NbU2*NbV2 > 2500`: bail to legacy `PointDepart` grid path (ln 3200+), no IntPolyh.
- `IntPolyh_Intersection` is built with the precomputed arrays unless `D1->IsUniformSampling()` (then counts only).

**S1. Deflection tolerance per surface** — `IntPolyh_Intersection::Perform` (`IntPolyh_Intersection.cxx:156`)
- `IntPolyh_Tools::ComputeDeflection(surf, UPars, VPars)` (`IntPolyh_Tools.cxx:143`) =
  `IntCurveSurface_ThePolyhedronOfHInter(surf,UPars,VPars).DeflectionOverEstimation()` — max distance of
  surface mid-points to triangle planes over the whole net, multiplied by 1.2 internally
  (`IntCurveSurface_ThePolyhedronOfHInter.cxx:129-130`: `ComputeMaxDeflection * 1.2`).

**S2. Standard mesh + intersection** — `IntPolyh_Intersection::PerformStd -> PerformMaillage` (`IntPolyh_Intersection.cxx:244,351`)
- `new IntPolyh_MaillageAffinage(S1,nbU1,nbV1,S2,nbU2,nbV2,0)`
- `FillArrayOfPnt(1/2, UPars, VPars, &deflTol)` (`IntPolyh_MaillageAffinage.cxx:348`): evaluate all nodes,
  set per-point `(x,y,z,u,v)`; flag degenerate iso rows/cols (poles) via `DegeneratedIndex/IsDegenerated`
  (3-point iso probe, dist^2 <= MyTolerance^2); accumulate `Bnd_Box`, then inflate box by
  `deflTol*1.2` per axis plus `Enlarge(MyTolerance)`.
- static `ComputeIntersection(theMaillage)` (`IntPolyh_Intersection.cxx:508`):
  1. `CommonBox()` (`IntPolyh_MaillageAffinage.cxx:503`) — intersect the two inflated boxes, expand by 10%
     per axis, mark every point with a 6-bit outcode `SetPartOfCommon` (1|2 x, 4|8 y, 16|32 z; 0 = inside).
  2. `FillArrayOfTriangles(1/2)` (ln 960) — 2 triangles per grid cell, fixed diagonal (UV, U V+1, U+1 V+1) /
     (UV, U+1 V+1, U+1 V); a triangle whose 3 points share a nonzero outcode bit on every edge pair gets
     `SetIntersectionPossible(false)` (outside common zone rejection).
  3. `FillArrayOfEdges(1/2)` (ln 770) — full winged-edge table: `NbEdges = 3*nbU*nbV - 2*(nbU+nbV) + 1`,
     each edge knows FirstTriangle/SecondTriangle (-1 at boundary); triangles get edge indices + orientations
     via `SetEdgeAndOrientation`.
  4. `TrianglesDeflectionsRefinementBSB()` — see S3.
  5. `TriangleCompare()` — see S4.
- If 0 couples: enlarge-zone retry (`PerformMaillage`, `IntPolyh_Intersection.cxx:373-390`):
  `IntPolyh_Tools::IsEnlargePossible` (BSpline/Bezier, not closed, not periodic, finite) -> resample with
  domain widened by 1% per side (`EnlargeZone`, `IntPolyh_Tools.cxx:62`), recompute. Catches intersections
  that graze the trimmed boundary.

**S3. Deflection refinement near intersection zones** — `IntPolyh_MaillageAffinage::TrianglesDeflectionsRefinementBSB` (`IntPolyh_MaillageAffinage.cxx:1286`)
- `ComputeDeflections(1/2)`: per triangle `IntPolyh_Triangle::ComputeDeflection` (`IntPolyh_Triangle.cxx:54`) =
  distance from surface point at UV barycenter to triangle plane; track FlecheMin/FlecheMax.
- Critical deflection = `FlecheMin*0.2 + FlecheMax*0.8` (80% quantile — only the worst 20% get refined).
- Disproportion test: `diag_i = (box diagonal)^2 / (nbU_i*nbV_i)`; if the *smaller* surface's critical
  deflection >= the larger's weighted diag, one surface is much larger:
  `LargeTrianglesDeflectionsRefinement` (ln 1197) — refine only large-surface triangles whose box overlaps
  the small surface's box; criterion `CritereAffinage = 0.5 * min-side of opposite box`;
  `MultipleMiddleRefinement` (`IntPolyh_Triangle.cxx:555`) recursively bisects with cap
  `MaxNbTT = 2*NbT + 1000`.
- Normal case: `TrianglesDeflectionsRefinement` (ln 1119) — `GetInterferingTriangles` (BVH pair traversal,
  see S4) to find couples; **non-interfering triangles are demoted** `SetIntersectionPossible(false)` on both
  meshes (never touched again); interfering triangles above their critical deflection get
  `MiddleRefinement` (`IntPolyh_Triangle.cxx:254`) — longest-edge bisection: new point at parametric middle
  (`IntPolyh_Point::Middle` evaluates the surface), 4 new triangles (2 own + 2 in the adjacent triangle across
  the split edge), full winged-edge patch-up (`NewEdge/OldEdge/LinkEdges2Triangle`); old triangles are killed
  in place (`deflection=-1, IsIntersectionPossible=false`), arrays only grow.

**S4. Triangle-triangle interference soup** — `IntPolyh_MaillageAffinage::TriangleCompare` (`IntPolyh_MaillageAffinage.cxx:3141`)
- `GetInterferingTriangles` (ln 185): two `BVH_BoxSet<double,3,int>` built with `BVH_LinearBuilder(10)` over
  per-triangle `Bnd_Box`es (`IntPolyh_Triangle::BoundingBox` caches box with gap
  `deflection + Precision::Confusion()`); skip `!IsIntersectionPossible || IsDegenerated`;
  `BVH_PairTraverse` selector collects box-overlap pairs, sorted, grouped into map T1 -> list<T2>.
- For each candidate pair: `TriContact(P1,P2,P3,Q1,Q2,Q3,Angle)` (ln 1466) — exact SAT test, 17 axes via
  `project6` (n1, m1, 9 edge-cross axes e_i x f_j, 3 in-plane outward axes per triangle); non-strict overlap
  (`mn1 > mx2` rejects) so *touching* triangles count as contact. Outputs `Angle` = cos of the two normals.
- Hit -> `IntPolyh_Couple(i_S1, i_S2, CoupleAngle)` appended to `TTrianglesContacts` list; both triangles
  flagged `SetIntersection(true)`.

**S5. Advanced 4-shift pass for tangential cases** — `IntPolyh_Intersection::Perform` + `IsAdvRequired` (`IntPolyh_Intersection.cxx:187,463`)
- Trigger: standard pass has 0 couples (and not parallel), OR `<= 10` couples with any `|cos| > 0.996` (~5 deg
  between triangle normals = near-tangency).
- `PerformAdv` (ln 266): compute `IntPolyh_ArrayOfPointNormal` per surface
  (`IntPolyh_Tools::FillArrayOfPointNormal`, normalized D1 cross); build FOUR meshes with nodes shifted
  **along the normal by `1.5 * deflTol`**, signs (S1 fwd,S2 rev), (rev,fwd), (fwd,fwd), (rev,rev)
  (`FillArrayOfPnt` advanced overload, `IntPolyh_MaillageAffinage.cxx:413`, `aPN.Normal * 1.5*theDeflTol`,
  reversed when `!isShiftFwd`). A tangential contact between offset shells becomes a transversal crossing in
  at least one of the four sign combinations.
- `MergeCouples` (ln 432): global `NCollection_Map<IntPolyh_Couple>` fence (hash = unordered index pair)
  keeps first instance across the 4 lists — valid because all 4 meshes share identical grid topology.
- If advanced found couples: chain start points from ALL FOUR maillages; else fall back to std couples.
- Parallel-surface guard `AnalyzeIntersection` (ln 536): `> 200` couples and count(|cos|>0.996) >= NbTriangles
  of either mesh -> `myIsParallel = true`, couples cleared (walking would be hopeless; caller sees IsParallel).

**S6. Start points + chaining into section lines** — `IntPolyh_MaillageAffinage::StartPointsChain` (`IntPolyh_MaillageAffinage.cxx:3333`)
- Loop over the couples list; skip `IsAnalyzed`. Each *unanalyzed* couple opens a NEW `IntPolyh_SectionLine`
  (reusing the last one if still empty — eap fix) and computes its intersection points:
- `StartingPointsResearch(T1,T2,SP1,SP2)` (ln 1767): 6 x `TriangleEdgeContact` — 3 edges of T2 vs plane of
  T1, then 3 edges of T1 vs plane of T2. `TriangleEdgeContact` (ln 2550):
  - coplanar sub-case (both edge ends on the plane within MyConfusionPrecision):
    `CalculPtsInterTriEdgeCoplanaires` — 2D projections on `n x cote`, up to 2 points;
  - transversal: `lambda = (pe1-pt1)/(pe1-pe2)` along the edge, then solve barycentric `(alpha,beta)` with
    hand-picked equation pairs; accept `0<=alpha<=1`, `0<=beta<=alpha` (inside-triangle in the e12/e23 frame);
  - every produced `IntPolyh_StartPoint` carries `(x,y,z, u1,v1, u2,v2, t1,e1,lambda1, t2,e2,lambda2)` —
    UVs on BOTH surfaces, the mesh edge crossed on each mesh (`e = -1` vertex hit, `-2` interior/none), and
    the barycentric position on that edge; edge snapped to -1 when lambda within MyConfusionPrecision of 0/1.
  - `TestNbPoints` (ln 1677) merges results to at most 2 distinct SPs (`CheckSameSP`: same edge id + same
    lambda within MyConfusionPrecision, or same UV1 for vertex points); >2 distinct => coplanar couple,
    NbPoints=3 (goes to tangent zones).
- Chaining (`NbPoints==2` generic case): append SP1 then extend forward, prepend SP2 then extend backward
  (`Prepend=true` — eap fix making both halves correctly ordered in ONE polyline).
  `GetNextChainStartPoint(SP, SPNext, ...)` (ln 3617): the crossed mesh edge `SP.E1()`/`SP.E2()` names the
  ADJACENT triangle via winged-edge (`TEdges[e].FirstTriangle()!=T ? First : Second`); the couple
  (nextTri, otherTri) must exist in the soup (`CheckCoupleAndGetAngle` — also marks it analyzed);
  `NextStartingPointsResearch` recomputes the 2 points of the next couple *excluding the incoming edge* and
  returns the one that is not SPInit. Both-edges case (`E1>=0 && E2>=0`, point on edges of both meshes)
  uses `CheckCoupleAndGetAngle2` to mark the two diagonal couples (T1,nextT2), (nextT1,T2) analyzed too —
  prevents the same crossing from re-seeding.
- `CheckNextStartPoint` (ln 3276): vertex points (`e==-1`) are diverted to `TTangentZones` (dedup by UV1+UV2
  within MyConfusionPrecision) instead of the section line — chains do not fork at mesh vertices; other
  points are appended/prepended. Termination: no next couple, or next point rejected.
- `NbPoints==1` (vertex/edge-edge graze): stored, then chain attempted across both candidate edges.
- `NbPoints in (2,7)` i.e. 3..6 = coplanar contact: single point stored via CheckNextStartPoint (-> tangent zone).
- KEY PROPERTY: the outer loop continues over REMAINING unanalyzed couples — every connected component of
  the tri-tri contact graph yields at least one section line. Branch enumeration is exhaustive by construction.

**S7. Consumption: seeds for the marching walker** — `IntPatch_PrmPrmIntersection::Perform` (`IntPatch_PrmPrmIntersection.cxx:2560-2942`)
- `nbLigSec = Interference.NbSectionLines()`; lines sorted by point count DESC (bubble sort on `TabL`) —
  longest branch walked first so dedup kills short echoes, not long originals.
- Per line `ls`:
  1. UV bounds `[UminLig1..UmaxLig1] x [VminLig1..VmaxLig1]` and same for S2 = min/max over ALL points of
     the polyhedral line (`GetLinePoint` returns x,y,z,u1,v1,u2,v2,incidence). These bound the walk.
  2. Multi-seed schedule (`NombreDePointsDeDepartDuCheminement`, ln 2679): attempt k picks point index
     `nbps2` = (1) `nbp/2`, (2) `1`, (3) `nbp-1`, (4) `3*nbp/4`, (5) `nbp/4`, then (6+) linear scan
     `k-3` — each index tried once (`TabPtDep` fence). Loop `while (nbp>5 && (k < 5 || !lignetrouvee) &&
     (k-3 < nbp || !lignetrouvee))` — i.e. at least 5 spread seeds, and if NO WLine was produced keep going
     through EVERY point of the line before giving up.
  3. Seed = the point's 4 parameters `(U1,V1,U2,V2)` -> `PW.PerformFirstPoint(StartParams, StartPOn2S)`
     (Newton onto the true intersection; see kb/occt_ssi-walking.md).
  4. Pre-walk dedup: refined start point checked against every existing WLine with
     `IsPointOnLine(StartPOn2S, testwline, Deflection)`; threshold logic uses
     `SeuildPointLigne = 15 * Increment^2` (skip seed if it lies on an already-marched line).
  5. `PW.Perform(StartParams, UminLig1..VmaxLig2)` — walk clamped to the line's UV boxes.
  6. Post-walk: `PW.PutToBoundary(Surf1,Surf2)`; if `NbPoints < 40`: `PW.SeekAdditionalPoints(...,40)`;
     duplicate-line rejection: if endpoints match an existing WLine's endpoints within `TolTangency`
     (either orientation), probe the existing line's MIDDLE point distance to the new polyline's segments —
     duplicate iff `aDx <= 2*Epsilon` (then `DublicateOfLinesProcessing`).
  7. Accepted: transitions from `tgline.DotCross(norm2,norm1)`; `IntPatch_RstInt::PutVertexOnLine` against
     both domains; synthetic endpoint vertices if none; `SeveralWlinesProcessing` (period stitching);
     `AddWLine(SLin, wline, Deflection)`; `lignetrouvee = true`.
- Tangent zones second (ln 2959-3189): ONE global UV box over all TZ points; each TZ point tried as a seed
  the same way (dedup, walk, reject-by-endpoint+midpoint, AddWLine). Isolated contacts thus still get a
  chance to open a full walked line.
- Finally `AdjustOnPeriodic(Surf1, Surf2, SLin)`.

---

## DATA STRUCTURES

- `IntPolyh_Point`: `x,y,z,u,v` + `myPOC` outcode int (common-box bits 1|2,4|8,16|32; 0 = inside) +
  `myDegenerated` flag. Arrays are `IntPolyh_Array<T>` vectors — grow-only, index-stable (refinement appends).
- `IntPolyh_Edge`: point1, point2, triangle1, triangle2 (winged edge; -1 = boundary/none).
- `IntPolyh_Triangle`: 3 point indices, 3 edge indices + 3 orientations (+1/-1), `myDeflection`,
  `myIsIntersectionPossible` (default true; false = culled), `myHasIntersection`, `myIsDegenerated`,
  cached `Bnd_Box` (gap = deflection + Precision::Confusion). Edge convention: e1=p1->p2, e2=p2->p3, e3=p3->p1.
- `IntPolyh_Couple`: `(myIndex1, myIndex2, myAnalyzed, myAngle)`; equality/hash symmetric in the pair
  (`IntPolyh_Couple.hxx:81-115`) — enables the 4-mesh fence map. Stored in `NCollection_List` (sequential
  scan + in-place `SetAnalyzed`).
- `IntPolyh_StartPoint` (`IntPolyh_StartPoint.hxx:107-122`): `x,y,z, u1,v1, u2,v2, lambda1,lambda2, angle,
  t1,e1, t2,e2, chainlist`. Edge code semantics: `>=0` mesh-edge index crossed, `-1` mesh VERTEX hit,
  `-2` none/interior. `angle` = couple's normal-cos, exported as `incidence` to the consumer.
- `IntPolyh_SectionLine`: growable array of StartPoints with `Prepend` (backward chain half);
  `IntPolyh_ArrayOfSectionLines` Init(1000); `IntPolyh_ArrayOfTangentZones` Init(10000) of StartPoints.
- Consumer sees only: `NbSectionLines / NbPointsInLine / GetLinePoint(x,y,z,u1,v1,u2,v2,incidence)` and
  `NbTangentZones / GetTangentZonePoint` — i.e. a pure `(3D + UVxUV)` polyline soup. Triangle identity does
  not leave IntPolyh.

---

## CONSTANTS & TOLERANCES (exact values)

| constant | value | where |
|---|---|---|
| `MyTolerance` | `10.0e-7` (= 1e-6) | IntPolyh_MaillageAffinage.cxx:51, Triangle.cxx:20 |
| `MyConfusionPrecision` | `10.0e-12` (= 1e-11) | ibid:52 — ALL lambda/UV same-point tests |
| `SquareMyConfusionPrecision` | `10.0e-24` | ibid:53 — degenerate normal test |
| default sampling | 10 x 10 nodes per surface | IntPolyh_Intersection.cxx:39-42; PrmPrm min via SamplePnts |
| sampling cell limit | `Limit = 2500` per surface | IntPatch_PrmPrmIntersection.cxx:2498 |
| deflection tol of net | `ComputeMaxDeflection * 1.2` | IntCurveSurface_ThePolyhedronOfHInter.cxx:129 |
| surface box inflation | `+deflTol*1.2` each axis, then `Enlarge(1e-6)` | MaillageAffinage.cxx:401-408 |
| common box expansion | 10% per axis (0-width axes borrow 10% of another) | MaillageAffinage.cxx:622-669 |
| advanced-shift magnitude | `1.5 * deflTol` along unit normal, +/- | MaillageAffinage.cxx:448 |
| IsAdvRequired | couples==0 (and !parallel), OR couples<=10 with any `abs(cos)>0.996` (~5 deg) | IntPolyh_Intersection.cxx:475-499 |
| parallel guard | couples>200 AND count(cos>.996) >= NbTri(1) or NbTri(2) | IntPolyh_Intersection.cxx:545-565 |
| refine criterion | `FlecheMin*0.2 + FlecheMax*0.8` | MaillageAffinage.cxx:1300,1315 |
| large-vs-small criterion | `0.5 * min(dx,dy,dz)` of the small surface's box | MaillageAffinage.cxx:1251-1253 |
| MultipleMiddleRefinement cap | `2*NbTriangles + 1000` | IntPolyh_Triangle.cxx:568 |
| enlarge-zone margin | `0.01 * |domain|` per side (BSpline/Bezier, open, finite) | IntPolyh_Tools.cxx:73-83 |
| BVH leaf size | 10 (`BVH_LinearBuilder(10)`) | MaillageAffinage.cxx:193 |
| triangle box gap | `deflection + Precision::Confusion()` | IntPolyh_Triangle.cxx:637 |
| degenerate iso probe | 3 points, dist^2 <= (1e-6)^2 | MaillageAffinage.cxx:3920-3996 |
| pole triangle cull | >= 2 degenerated points | IntPolyh_Triangle.cxx:66-71 |
| seed-on-line threshold | `SeuildPointLigne = 15 * Increment^2` (default Increment 0.01 -> 1.5e-3) | PrmPrm.cxx:2562 |
| walker min points | `aMinNbPoints = 40` (`SeekAdditionalPoints`) | PrmPrm.cxx:2774 |
| duplicate-line probe | endpoints within `TolTangency` both ways, then middle-point `aDx <= 2*Epsilon` | PrmPrm.cxx:2806-2834 |
| seeds per line | 5 spread attempts (`NbDePointsDeDepartDuChmLimit`), then EXHAUSTIVE per-point while no line found | PrmPrm.cxx:2677-2936 |
| IntPatch defaults | `myFleche` (Deflection) 0.01 clamp [1e-3,10]; `myUVMaxStep` (Increment) 0.01 clamp <=0.5 | IntPatch_Intersection.cxx:159-190 |

---

## INVARIANTS

1. **Every start point is a full 4-parameter seed** `(u1,v1,u2,v2)` + 3D — the walker's Newton
   (`PerformFirstPoint`) starts from a consistent pair of surface parameters; no closest-point projection,
   no cross-surface guessing.
2. **Exhaustive branch coverage**: every tri-tri contact couple is either consumed by a chain (marked
   analyzed while chaining across shared mesh edges) or OPENS A NEW section line. Connected components of the
   contact graph == section lines (+ tangent-zone singletons). Nothing is dropped before the walker sees it.
3. **Chains are walks in the triangle adjacency graph**: next couple = flip across the winged-edge; a chain
   step consumes the crossing couple(s) exactly once (including the two diagonal couples in the edge-edge
   case), so chains never loop and never re-walk a branch.
4. **Chains grow both directions** from the initial couple (append + prepend), so seed position inside the
   branch does not truncate it.
5. **Refinement is local**: only triangles whose BVH boxes interfere are refined; all others are culled
   (`IsIntersectionPossible=false`) *before* the O(couples) exact tests. Refinement preserves index stability
   (kill-in-place + append).
6. **The 4 shifted meshes share grid topology** with the base mesh -> triangle indices are comparable ->
   couple dedup across shifts is a plain unordered-pair fence map.
7. **Walk confinement**: each march is clamped to the UV bbox of ITS polyhedral line (+ the TZ box for
   tangent-zone seeds) — a walk cannot wander onto another branch and mask it (dedup would then delete
   the other branch's line as "already found").
8. **Failure of one seed does not lose the branch**: the schedule retries mid/first/last/3-quarter/quarter
   and then every remaining point of the line until `lignetrouvee`.
9. Vertex-grazing contact points never enter section lines (would create false forks at mesh nodes); they are
   quarantined in TangentZones and retried later as independent seeds.

---

## PITFALLS

- **10x10 uniform is not the real net**: PrmPrm feeds knot-aware `SamplePnts` arrays; uniform-only sampling
  is precisely what misses thin features between samples. Density adapts to `Deflection` (default 0.01).
- The near-tangent class kills naive seeding: near tangency, base meshes either miss contact (0 couples) or
  produce quasi-parallel triangles whose intersection segments are noise. The **+/-1.5-deflection 4-shift
  pass** is the load-bearing fix — a tangential contact becomes a clean transversal crossing of offset shells
  in at least one sign combination. Angle `|cos|>0.996` is the trigger AND the parallel-guard metric.
- `TriContact` uses non-strict SAT rejection (`>` not `>=`) — touching counts as contact (conservative:
  more seeds, never fewer).
- Coplanar triangle/edge configurations are handled by a dedicated 2D path
  (`CalculPtsInterTriEdgeCoplanaires`), NOT by perturbation; results still dedup by UV within 1e-11.
- Chaining depends on couples being findable un-analyzed (`CheckCoupleAndGetAngle`); the edge-edge crossing
  case must mark BOTH diagonal couples analyzed or the same crossing re-seeds a duplicate line.
- Longest-line-first sort matters: dedup (`IsPointOnLine` / endpoint+midpoint test) rejects the NEW line, so
  short fragments must be walked after their long parents.
- Sub-`Deflection` micro-branches: a section line with `nbp <= 5` gets a single seed attempt (schedule sets
  k = limit immediately when `nbp < 3`); OCCT relies on the line EXISTING to seed at all — enumeration first,
  march second. (Our (18,5) class fails at enumeration, not at march robustness.)
- Enlarge-zone retry applies only to BSpline/Bezier open finite surfaces — analytic and periodic surfaces
  never get the 1% widening (they cannot lose grazing contacts to trimming the same way).
- `MiddleRefinement` on a boundary edge (no adjacent triangle) creates only 2 triangles with `-1` winged
  slots — the chain simply terminates at the mesh boundary (walker's `PutToBoundary` finishes the job in 3D).
- `AnalyzeIntersection` clears ALL couples when parallel-overlap is detected — the caller must treat
  `IsParallel()` as "coincident-face path", not "no intersection".

---

## PORT MAP

Our anchors: `brep_section.cpp build_section_scaffold` (SSI chains, paves, keep-verdict, valence-1 bridge,
welded vertices), `brep.cpp split_with` (UV arrangement, shared-chain run lifting, whole-seg keys),
`combine` (exact weld + tube merge + NK-RESCUE), winding+radial classification. Marching = our newton_cc
SSI walker (see kb/occt_ssi-walking.md port map for step control; this table covers only SEEDING).

| # | OCCT mechanism | Our anchor | Action | Design (1 line) |
|---|---|---|---|---|
| 1 | Tri-tri interference soup (`TriangleCompare` + `GetInterferingTriangles` BVH) | brep_section.cpp SSI seed generation (single seed per surface pair) | **NEW-BUILD** | Pre-march polyhedral pass: sample both surfaces (existing mesh_q or raw grid), BVH box pairs, SAT `TriContact`, emit couple list — this is the branch ENUMERATOR we lack. |
| 2 | `StartPointsChain` winged-edge chaining -> one section line per connected component | build_section_scaffold chain assembly | **ADOPT** | Chain the couple soup across shared mesh edges into polyline components BEFORE marching; component count = mandatory branch budget; a marched result that covers fewer components than enumerated = missing branch (the (18,5) mini-branch becomes its own component with its own seed). |
| 3 | `StartPoint = (xyz, uv1, uv2, edge, lambda)` full 4-param seeds | our seeds (3D point + one-surface UV, other projected) | **ADOPT** | Carry BOTH surfaces' UVs from the triangle barycentrics through to newton_cc start — kills projection-onto-wrong-sheet failures at seed time. |
| 4 | Multi-seed schedule nbp/2, 1, nbp-1, 3nbp/4, nbp/4, exhaustive + per-line UV walk box | single-seed newton_cc march per SSI pair | **REPLACE** | Retry the march from spread points of the SAME polyhedral line until the marched chain matches the line's extent; clamp march to the line's UV bbox so one walk cannot swallow/mask a sibling branch. |
| 5 | 4-shift advanced pass (`PerformAdv`, +/-1.5*defl normal offsets, `MergeCouples` fence) | tangential newton_cc stalls (cut-node crossing snap class) | **ADOPT** | When the std polyhedral pass yields 0 or <=10 near-parallel couples, redo it on 4 normal-offset node sets and dedup couples by unordered pair — recovers tangent circles / grazing contacts as seedable transversal crossings. |
| 6 | Deflection refinement in interfering zones only (`TrianglesDeflectionsRefinementBSB`, 80% criterion, longest-edge `MiddleRefinement`) | fixed-density sampling in section probing | **NEW-BUILD (deferred)** | Refine only BVH-interfering triangles above `min*0.2+max*0.8` deflection; disproportionate-size fallback (0.5*min-box-side criterion) for small-cutter x big-base cases. |
| 7 | Seed/line dedup: `IsPointOnLine` (15*Inc^2), endpoint TolTangency + midpoint 2*Epsilon probe | combine exact weld + NK-RESCUE duplicate handling | **ADOPT** | Use the two-stage verdict (endpoints both ways, then midpoint-to-segment distance) to decide "same branch" before welding a re-marched chain — cheaper and stricter than whole-chain Hausdorff. |
| 8 | TangentZones = quarantined vertex/coplanar contacts retried as seeds | welded vertices + valence-1 bridge in build_section_scaffold | **ADOPT** | Isolated contact points that fail to chain should not be dropped: park them, and after all chains are marched, seed a march from each surviving one (dedup via #7); complements valence-1 bridging for point-contact branches. |
| 9 | Longest-line-first ordering before marching | our chain processing order (unordered) | **ADOPT** | Sort enumerated components by point count desc so dedup always discards fragments, never parents. |
| 10 | Enlarge-zone 1% retry on 0 couples | trim-boundary grazing losses (trim-snapped sliver class) | **ADOPT (cheap)** | If the polyhedral pass finds nothing for a pair whose boxes overlap, retry with the sample domain widened 1% per open finite side. |

Priority for the (18,5) mini-branch class: **#1 + #2** (enumerate components — the miss is structural),
then **#4** (multi-seed with UV confinement), then **#5** (tangential robustness).
