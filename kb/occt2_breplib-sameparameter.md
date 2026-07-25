# breplib-sameparameter

The same-parameter doctrine: how OCCT makes an edge with one 3D curve + N pcurves
parameter-consistent, computes edge tolerance from measured deviation, subtracts
endpoint tolerance spheres to find an edge's valid range, and the
multi-representation edge model our kernel lacks.

Sources read (line numbers refer to these files):
- `ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib.cxx` (3295 L: SameParameter L1011-1740, UpdateTolerances L1744-1979, UpdateInnerTolerances L1983, BuildCurve3d L301, UpdateEdgeTol L493, GetEdgeTol L719, BoundingVertex L3013)
- `ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_1.cxx` (FindValidRange L173/L262, findNearestValidPoint L31)
- `ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_ValidateEdge.cxx/.hxx`
- `ModelingData/TKGeomBase/Approx/Approx_SameParameter.cxx/.hxx` (the reparameterization engine SameParameter delegates to)
- `ModelingData/TKBRep/BRep/BRep_TEdge.{hxx,lxx,cxx}`, `BRep_CurveRepresentation.hxx`, `BRep_GCurve.hxx`, `BRep_CurveOnSurface.hxx`, `BRep_CurveOnClosedSurface.hxx`, `BRep_CurveOn2Surfaces.hxx`, `BRep_TVertex.hxx`, `BRep_Builder.cxx` (UpdateCurves L54-405, UpdateEdge L635-1008, Range L1091-1155, UpdateVertex L1203-1312), `BRep_Tool.cxx` (Curve L172, CurveOnSurface L301/L488, CurveOnPlane L379, Range L931, Parameter L1340, Tolerance L881/L1314)

Relation to existing specs (reference, do not re-read for these topics):
- `kb/occt_tolerance-model.md` — where SameParameter/ValidateEdge sit inside the BOP
  pipeline (MakePCurve L1724 call site, CorrectEdgeTolerance audit, fuzzy model).
- `kb/occt_ff-posttreat-samedomain.md` — section-edge creation calls
  `BRepLib::SameParameter(aE)` after MakePCurve; FindValidRange as micro gate.
- `kb/occt_pavefiller-core.md`, `kb/occt_interference-vef.md` — FindValidRange call
  sites (shrunk range, micro pave blocks).
This spec covers the *mechanisms themselves*, which those specs only invoke.

---

## STAGE PIPELINE

### 0. Edge model prerequisite — the multi-representation list
`BRep_TEdge` (BRep_TEdge.hxx L39-82) holds `myTolerance`, `myFlags`
(ParameterMask=1, RangeMask=2, DegeneratedMask=4 — BRep_TEdge.cxx L27-29) and
`myCurves : NCollection_List<handle<BRep_CurveRepresentation>>`. One edge = at most
ONE `BRep_Curve3D` + one `BRep_CurveOnSurface` per adjacent (surface, location)
chart + `BRep_CurveOnClosedSurface` (TWO pcurves) when the edge is a seam of a
closed surface + optional `BRep_CurveOn2Surfaces` (regularity only) + polygon reps.
Constructor (BRep_TEdge.cxx L33-39) sets tol=RealEpsilon, **SameParameter=true,
SameRange=true** — a fresh edge *claims* consistency.

### 1. Shape-level driver: `BRepLib::SameParameter(S, tol, forced)` → `InternalSameParameter` (BRepLib.cxx L913-1008)
1. For every unique edge: if `forced` and (SameRange || SameParameter), clear both
   flags first (L931-946), then call the per-edge `BRepLib::SameParameter(edge,
   tol, newTol, useOldEdge)` (L948). If `newTol > 0`, bind it into a
   shape→tolerance map for BOTH end vertices via `UpdTolMap` (max-merge, L796-831).
2. Plane pass (L971-1002): for every edge of every PLANAR face, `GetEdgeTol`
   (L719-793) — because pcurves on planes are **not stored**, the per-edge stage
   never measured them: project the 3D curve on the plane
   (`ProjLib_ProjectedCurve`), sample **23** points, `theEdTol = 1.05 * sqrt(max
   square deviation)` with per-sample floating-noise floor
   `Epsilon(max(|Pc3d|^2,|Pcons|^2))`; bind into the same map.
3. `UpdShTol` (L837-909): apply the map (`UpdateFace`/`UpdateEdge` grow-only;
   vertex via `TVertex::UpdateTolerance`, or hard `Tolerance()` when forced),
   empty-copy + reshaper replace when input is immutable.
4. `InternalUpdateTolerances(S, false, ...)` (L1007) — the V>=E>=F harmonization
   audit (stage 6).

### 2. Per-edge core: `BRepLib::SameParameter(theEdge, theTolerance, theNewTol&, IsUseOldEdge)` (BRepLib.cxx L1251-1740)
- Early out if flag already set (L1256). `GetCurve3d` (L1192): first
  `BRep_Curve3D` rep gives C3d, [f3d,l3d], location, and the live rep list CList.
  Null C3d → return null edge (degenerated edges never same-parametered).
- Trimmed-periodic guard (NIZHNY-OCC486, L1303-1332): clamp [f3d,l3d] to the basis
  curve's parameter range ONLY if the basis curve of a `Geom_TrimmedCurve` is NOT
  periodic — a trimmed periodic curve may legitimately cross its period.
- `Prec_C3d = BRepCheck::PrecCurve(GAC)` (L1339) — numeric noise floor of the 3D
  curve at its extreme coordinate values.
- Loop over every `BRep_GCurve` rep that `IsCurveOnSurface()` (L1352-1718), for
  `PC[0] = PCurve()` and, when `IsCurveOnClosedSurface()`, also `PC[1] = PCurve2()`
  (both pcurves of a seam edge measured independently, same iteration i=0,1):
  1. **SameRange normalization** (L1387-1392): if edge not SameRange,
     `GeomLib::SameRange(TolSameRange, PC[i], GCurve->First(), GCurve->Last(),
     f3d, l3d, curPC)` — affine (or re-approximated) reparameterization of the
     pcurve onto the 3D range. `TolSameRange = max(GAC.Resolution(theTolerance),
     Precision::PConfusion())` (L1378).
  2. **Deviation measurement** `error = ComputeTol(HC, HC2d, HS, NCONTROL=22)`
     (L1070-1188): 23 uniform samples of `|C3d(u) − S(PC(u))|²`; **out-of-chart
     parametric penalty**: if PC leaves the surface UV domain by more than 1% of
     the span (du=0.01*(ul-uf)) on a non-periodic axis, count `dapp =
     DSdu * overshoot` where `DSdu = 1/UResolution(1.)` (parametric→3D conversion)
     and skip the 3D sample; infinite surface value → `Precision::Infinite()`.
     Outlier analysis (L1147-1185): bucket square distances at 1.0; if the big
     bucket is nonzero but `<10%` of populated samples (`N3 = 100*N2/(N1+N2) < 10`)
     treat the big ones as broken evaluations and take the max of the small bucket
     (`ana` path). Return `1.5 * d2` (or `1.5*sqrt(D2)` on the ana path), floored
     at `1.e-7`. `error > BigError=1.e10` → record and break (L1398).
  3. **C0-pcurve repair** (only `GeomAbs_BSplineCurve` with `Continuity()==C0`,
     L1404-1623): `Geom2dConvert::C0BSplineToC1BSplineCurve(bs2d, TolConf2d)` with
     `TolConf2d = max(min(UResolution(tol), VResolution(tol)), PConfusion)`;
     if periodic and the origin moved by > PConfusion, restore it by scanning
     knots for the original origin point and `SetOrigin(Index)` (IFV Jan 2000,
     twice: L1419-1442 and L1471-1494). If still C0: `EvalTol` (L1034-1066: 5
     interior samples at t=i/6, `Extrema_LocateExtPC` projection, need >2
     successes; returns max projection distance as `tolbail`) → retry conversion
     with `Tol2dbail = max(min(min(UResbail,VResbail), 0.1*sqrt(min pole gap²)),
     TolConf2d)`; still C0 → keep original, `repar=false`. On success
     `BSplCLib::Reparametrize(fC0, lC0, Knots)` — to the ORIGINAL C0 curve range,
     not [f3d,l3d] (bug history in commented line L1514) — then re-measure
     `error1 = ComputeTol(...)`; if worse, restore saved `bs2dsov` and set
     `isANA=true` (L1522-1534). **Bad-knot check** (L1537-1582): if continuity
     >C0 and `error > max(1e-3, tol)`, ratio of adjacent knot intervals >
     `critratio=10` (and `dtmin < Resolution(max(1e-3,tol))`) marks the pcurve
     "bad" → re-approximate by curvilinear (arc-length-like) parameter:
     `Approx_CurvilinearParameter(HC2d, HS, max(1.e-3, tol), cont<=C2, maxdeg
     (14 if degree was 1), 10)`, then Reparametrize back to [fC0,lC0] if the
     range drifted by > TolSameRange (L1598-1619).
  4. **Reparameterization** (L1625-1696): `Approx_SameParameter SameP(HC, HC2d,
     HS, aTol)` with `aTol = (isANA && isBSP) ? 1.e-7 : theTolerance`.
     - `IsSameParameter()` → keep pcurve, `maxdist = max(maxdist, TolReached())`.
     - `IsDone()` → the reparameterized pcurve `SameP.Curve2d()` replaces the
       stored one ONLY if `tolreached <= error` (never accept a worse curve);
       `maxdist = max(maxdist, min(tolreached, error))`; write back via
       `GCurve->PCurve(curPC)` / `PCurve2(curPC)` (i-indexed).
     - failed → `GeomLib::SameRange` fallback write-back and `IsSameP = false`.
  5. **OCC5898 numeric rescue** (L1704-1714): if `!IsSameP`, accept anyway when
     `anEdgeTol + max(Prec_C3d, BRepCheck::PrecSurface(HS)) >= error` — the
     deviation is inside representation noise; `maxdist = max(maxdist, anEdgeTol)`.
- Epilogue (L1719-1737): `B.Range(aNE, f3d, l3d)` (pushes the 3D range into ALL
  reps), `B.SameRange(aNE, true)`; if `IsSameP` and at least one pcurve existed:
  `maxdist = max(maxdist, Precision::Confusion())`, `theNewTol = maxdist`,
  `aNTE->Tolerance(maxdist)` — **hard SET, the one sanctioned tolerance
  reduction in OCCT** (comment L1723-1727: allowed because every representation
  was just measured; vertices can NOT be shrunk the same way), then
  `B.SameParameter(aNE, true)`. Caller propagates `theNewTol` to end vertices
  (`UpdateVTol` L1222 / `UpdTolMap`), grow-only.

### 3. Reparameterization engine: `Approx_SameParameter::Build(Tolerance)` (Approx_SameParameter.cxx L318-544)
1. `BuildInitialDistribution` (L547): `myNbSamples = 22` uniform samples on BOTH
   parameter ranges ("to be consistent with checkshape", hxx L161); if pcurve
   continuity < C1, refine to one sample per C1 interval
   (`IncreaseInitialNbSamples` L589).
2. `CheckSameParameter` (L655): sample `|C3d(t3d_i) − ConS(t2d_i)|²`; also
   projects to build the corrected `t3d→t2d` table. Already same-parameter →
   `myTolReached = ComputeTolReached(c3d, cons, 2*22)` and done. If more than
   `aPercentOfBadProj = 0.3` of projections failed → not done, bail (L353-373).
3. `ComputeTangents` at extremities (required for the interpolation, L380).
4. Loop: interpolate a **cubic 1D B-spline reparameterization function**
   t3d→t2d through the `nbp+1` sample pairs (`num_poles = nbp+3`, `num_knots =
   nbp+7`, degree 3); `Check` monotonicity/deviation on `2*nbp` midpoints; then
   approximate the composite `PC(reparam(t))` as a real 2D curve with
   `AdvApprox_ApproxAFunction` (tolerances `{UResolution(besttol),
   VResolution(besttol)}`, continuity min(C1, input), `aMaxDeg=11`,
   `aMaxSeg=1000`); accept iff `TolReached < 250 * besttol` (L456 — "to be
   tolerant with discrete tolerance"); else `IncreaseNbPoles` (bisection insert at
   worst interval) up to `myMaxArraySize = 1000` points.
5. Failure fallback (L490-540): compute deviation of BOTH the original pcurve and
   the forced approximation, keep whichever is smaller; still `myDone = true`
   with honest `myTolReached`.
- `ComputeTolReached` (L152-190): max deviation over `2*22+1` uniform samples,
  `* 1.05`, floored at `Precision::Confusion()`. Exceptions during evaluation →
  Infinite.

### 4. Validation: `BRepLib_ValidateEdge` (BRepLib_ValidateEdge.cxx)
Measures max distance between reference 3D curve and one `Adaptor3d_CurveOnSurface`.
- `processApprox` (L103-230): `myControlPointsNumber = 22`. If same-parameter AND
  equal ranges (within PConfusion): direct 23-point same-parameter compare.
  Else: compare both endpoint pairs, then for the 21 interior points project each
  curve's point onto the OTHER curve (`Extrema_LocateExtPC` seeded at the
  proportional parameter, both directions); any projection failure → `IsDone =
  false`. Optional early-exit gate `SetExitIfToleranceExceeded`.
- `processExact` (L234): `GeomLib_CheckCurveOnSurface` (global minimization) —
  only when same-parameter.
- `correctTolerance` (L68): checked tolerance += `max(BRepCheck::PrecCurve(ref),
  BRepCheck::PrecSurface(surf))`. `GetMaxDistance`/`UpdateTolerance` (L49-64)
  return `myCalculatedDistance * 1.00001` (grow-only update).

### 5. Valid range: `BRepLib::FindValidRange` (BRepLib_1.cxx)
Edge-level overload (L262-309): needs a 3D curve; `aTolV[i] =
BRep_Tool::Tolerance(V_i) + Precision::Confusion()` ("to have correspondence with
intersection precision"); null vertex at a finite end → sphere of `tolE` at the
curve endpoint. Curve-level (L173-258):
- degenerate range `theParV2 − theParV1 < PConfusion` → false.
- `anEps = max(Curve.Resolution(tolE)*0.1, Epsilon(maxAbsPar), PConfusion)` (L201).
- Per end, `findNearestValidPoint` (L31-169) = **endpoint-sphere subtraction**:
  1. end point must be INSIDE its sphere (`dist² <= tolV²`), else the vertex does
     not cover its curve end → false;
  2. march inward with step `Curve.Resolution(theTol) * 1.01` (min anEps) until a
     sample leaves the sphere; Bezier/BSpline local-singularity acceleration:
     when local `|D1|² < (0.01/Resolution(1.))²` double the step (L108-138);
     marching past the far end with everything inside → whole edge inside sphere
     → false (micro edge);
  3. binary search the in/out bracket down to `anEps`, return midpoint.
- Reject if remaining span `< anEps` from either side, or `theFirst > theLast`
  (spheres overlap through the edge) → false. **No valid range = micro edge law**
  (see occt_pavefiller-core.md stage 5 for the BOP consequence: weld endpoints,
  never emit the edge).

### 6. Tolerance harmonization: `InternalUpdateTolerances` (BRepLib.cxx L1744-1962)
1. Optional `IsVerifyTolerance` face floor: surface-type minimum `Confusion ×
   {1 plane/cylinder/cone, 2 sphere/torus, 4 other}` scaled by the face bounding
   box max dimension, capped at 0.99 (L1753-1823).
2. Edge pass: `tol(E) = max(tol(F_parents))` — bind only if it GROWS the edge
   (L1827-1856).
3. Vertex pass (L1858-1959): for each vertex, `tol = max(tol(E_incident))`; then
   for every incident SameRange edge evaluate EVERY representation at the vertex
   parameter (`BRep_Tool::Parameter(V,E)`): 3D curve value and `S->Value(PC(par))`
   for the pcurve AND `PCurve2` of closed reps, all transformed by `Eloc * crLoc`;
   `tol = max(tol, sqrt(maxSquareDist)) + 2*Epsilon(tol)`; update grow-only
   (forced set only under IsVerifyTolerance bookkeeping).
Also `UpdateInnerTolerances` (L1983-2083): per edge, 23 samples (2 if not
same-parameter — endpoints only, matched by parameter proportion) across ALL
`BRepAdaptor_Curve(E,F)` reps vs the free curve, `dist + 2*Epsilon(dist)`;
endpoint distances also pushed into vertex tolerances.

### 7. Vertex fusion primitive: `BRepLib::BoundingVertex` (BRepLib.cxx L3013-3125)
n=2: minimal enclosing sphere of two tolerance spheres — if `D <= Rmax − Rmin`
keep the bigger one; else center `= 0.5*(Pm + Pn − dir*(dR/D))`, radius
`= 0.5*(Rm + Rn + D)`. n>2: coordinate-sorted barycenter (issue 0027540
determinism), radius = max over members of `dist + tolV_i`, `+ eps` margin.

### 8. Reverse direction: `BRepLib::BuildCurve3d` (BRepLib.cxx L301-456)
When an edge has pcurves but no 3D curve: plane rep → exact `GeomLib::To3d`
with tolerance 0; else approximate first pcurve-on-surface via
`GeomLib::BuildCurve3d` (MaxSegment default `30 + max(surface knots, pcurve
knots)`, `evaluateMaxSegment` L273), edge tol = `max(existing, requested)`; if it
was the ONLY pcurve the edge is same-parameter by construction (L441-448).
`UpdateEdgeTol` (L493-687): re-measures an edge against ALL its other reps with
QuasiUniformDeflection sampling (`MinTol*100` deflection, clamp 30..90 points,
`safe_factor 1.4`), using `EvalMaxParametricDistance` when same-parameter else
`EvalMaxDistanceAlongParameter`; hard-SETs the tolerance.

---

## DATA STRUCTURES

```
BRep_TEdge : TopoDS_TEdge
  myTolerance : double            # 3D cover radius of ALL representations
  myFlags     : int               # SameParameter=1 | SameRange=2 | Degenerated=4
  myCurves    : List<handle<BRep_CurveRepresentation>>

BRep_CurveRepresentation            (abstract; myLocation relative to edge location)
 ├─ BRep_GCurve                     (myFirst, myLast — the range; virtual D0)
 │   ├─ BRep_Curve3D                (myCurve : Geom_Curve)
 │   ├─ BRep_CurveOnSurface         (myPCurve, mySurface, myUV1, myUV2 cached chart endpoints)
 │   │   └─ BRep_CurveOnClosedSurface (myPCurve2, myContinuity, myUV21, myUV22)  # seam: 2 pcurves, same surface
 │   └─ (polygon reps: Polygon3D, PolygonOnSurface, PolygonOnTriangulation, + Closed variants)
 └─ BRep_CurveOn2Surfaces           (regularity/continuity only, no geometry)

BRep_TVertex : TopoDS_TVertex
  myPnt, myTolerance
  myPoints : List<handle<BRep_PointRepresentation>>   # parameter of V on each curve/pcurve
```

Semantics that make the list a coherent multi-representation model:
- **Uniqueness**: `BRep_Builder::UpdateEdge` goes through static `UpdateCurves`
  (BRep_Builder.cxx L58-374) which REPLACES the existing rep for the same
  `(surface, location)` key (`IsCurveOnSurface(S,l)`) and removes it when the new
  pcurve handle is null; the 3D range found on the Curve3D rep is copied onto the
  new pcurve rep (L154-164) so ranges stay shared.
- **Location algebra**: every public Builder/Tool entry stores/queries with
  `L.Predivided(E.Location())` and returns `E.Location() * GC->Location()` — reps
  are stored relative to the edge frame; instancing-safe.
- **Seam addressing**: `BRep_Tool::CurveOnSurface(E, S, L, ...)` (BRep_Tool.cxx
  L327-373) returns `PCurve2()` when the rep `IsCurveOnClosedSurface()` and the
  edge orientation in the face is REVERSED — the edge's two occurrences in the
  seam-face's wire address the two chart images by orientation.
- **Plane fallback**: same function falls back to `CurveOnPlane` (L379-450 —
  on-the-fly `ProjLib_ProjectedCurve` projection, `theIsStored=false`): pcurves on
  planes are deliberately never stored.
- **Range authority**: `BRep_Tool::Range(E)` (L931) prefers the Curve3D rep, else
  the first curve-on-surface; `BRep_Builder::Range(E, f, l, Only3d)` (L1091)
  writes into ALL GCurve reps (or only 3D).
- **Vertex parameter**: `BRep_Tool::Parameter(V,E)` (L1340) resolves via vertex
  orientation in the edge (FORWARD→First, REVERSED→Last), falling back to the
  vertex's stored `BRep_PointRepresentation` list; closed-curve disambiguation by
  vertex orientation when `Pf≈Pl` within tolV.

---

## CONSTANTS & TOLERANCES (exact values)

| constant | value | where |
|---|---|---|
| NCONTROL sample count (edge SameParameter) | 22 (→23 points) | BRepLib.cxx L1294 |
| Approx_SameParameter samples | myNbSamples = 22, "consistent with checkshape" | Approx_SameParameter.hxx L161 |
| ComputeTolReached margin / floor | ×1.05, floor Precision::Confusion (1e-7) | Approx_SameParameter.cxx L184-188 |
| ComputeTol safety / floor | ×1.5, floor 1e-7 | BRepLib.cxx L1185-1186 |
| ComputeTol out-of-chart band | 1% of UV span (du=0.01*(ul-uf)) | L1083 |
| ComputeTol outlier bucket / gate | dist² threshold 1.0; ana iff N3=100*N2/(N1+N2) < 10 (and N1>N2, N2≠0) | L1153-1174 |
| BigError bail | 1.e10 | L1349 |
| TolSameRange | max(GAC.Resolution(tol), PConfusion=1e-9) | L1378 |
| C0→C1 conversion tol | TolConf2d = max(min(URes(tol),VRes(tol)), PConfusion) | L1406-1409 |
| EvalTol samples / quorum | 5 at t=i/6, need ok>2 | L1048-1065 |
| Tol2dbail pole-gap guard | d = 0.1*min adjacent pole distance; Tol2dbail = max(min(Tol2dbail,d),TolConf2d) | L1455-1467 |
| bad-knot ratio / trigger | critratio = 10, only when error > max(1e-3, tol) | L1541-1576 |
| Approx_CurvilinearParameter | tol max(1e-3, tol), cont ≤ C2, maxdeg 14 (if deg 1), 10 segs | L1593-1603 |
| ANA∧BSP special SameP tol | 1.e-7 | L1628 |
| final edge tol floor | Precision::Confusion (1e-7) | L1731 |
| Approx interpolation degree / sizes | 3; poles nbp+3, flat knots nbp+7 | Approx cxx L398-401 |
| Approx 2D re-approx | tol1d = {URes(besttol), VRes(besttol)}, maxdeg 11, maxseg 1000 (fallback 40) | L444-452, 517 |
| Approx discrete accept ratio | TolReached < 250 × besttol | L456 |
| Approx bad-projection quota | 0.3 of samples | L353 |
| Approx max samples | myMaxArraySize = 1000 | hxx L162 |
| ValidateEdge points / margin | 22; ×1.00001; +max(PrecCurve, PrecSurface) | ValidateEdge cxx L30, L51, L68-76 |
| FindValidRange step | Curve.Resolution(tolV)*1.01, min anEps | BRepLib_1.cxx L61 |
| FindValidRange eps | max(Resolution(tolE)*0.1, Epsilon(maxPar), PConfusion) | L201 |
| BSpline slow-D1 accel | aD1Mag = (0.01/Resolution(1.))², step ×2 | L78-82, L119 |
| FindValidRange vertex margin | tolV + Precision::Confusion | L284-290 |
| GetEdgeTol (plane pass) | 23 samples, ×1.05, Epsilon(coord²) noise floor | BRepLib.cxx L770-792 |
| UpdateInnerTolerances samples | 23 same-param / 2 endpoints otherwise; dist + 2·Epsilon(dist) | L2028-2044 |
| InternalUpdateTolerances vertex margin | + 2·Epsilon(tol) | L1942 |
| face floor by surface type | Confusion×{1: plane/cyl/cone, 2: sphere/torus, 4: else} × boxDim, cap 0.99 | L1774-1819 |
| UpdateEdgeTol sampling | deflection MinTol×100, clamp [30, 90] pts, safe_factor 1.4 | L498-503 |
| BuildCurve3d default | tol 1e-5; MaxSegment 30 + max knots | L463, L295 |
| tolerance read floor | BRep_Tool::Tolerance(E/V) returns max(stored, Confusion) | BRep_Tool.cxx L881-894, L1314-1333 |
| BRep_TEdge ctor | tol = RealEpsilon, SameParameter=true, SameRange=true | BRep_TEdge.cxx L33-39 |

---

## INVARIANTS

1. **Same-parameter contract**: flag true ⟺ for every stored pcurve P_k on chart
   S_k: `|C3d(t) − S_k(P_k(t))| <= Tol(E)` for the SAME parameter t across the
   shared range — consumers may evaluate any representation at any t
   interchangeably, no projection needed.
2. **SameRange ⊂ SameParameter**: SameParameter(true) is only ever set after
   Range+SameRange(true) (BRepLib.cxx L1719-1736); a not-SameRange edge is first
   affinely renormalized (GeomLib::SameRange).
3. **Tolerance monotonicity with one exception**: every `Update*` path is
   grow-only (`UpdateTolerance` = max); the ONLY sanctioned shrink is
   `TE->Tolerance(maxdist)` at SameParameter success (all reps just measured) and
   `UpdateEdgeTol`'s equivalent hard set. Vertex tolerances are NEVER shrunk by
   this machinery (explicit comment L1723-1727).
4. **Hierarchy direction**: Tol(V) >= Tol(E) >= Tol(F) is restored *bottom-up
   after* any same-parameter run (edges take max of parent faces, vertices take
   max of incident edges plus measured endpoint scatter).
5. **Vertex covers all representation endpoints**: after harmonization,
   `dist(Pnt(V), rep(par_V))` <= Tol(V) for the 3D curve and every pcurve
   (including PCurve2) of every incident edge.
6. **Representation uniqueness**: at most one rep per (surface, location) key per
   edge; inserting a new pcurve replaces the old and inherits the 3D range.
7. **Seam edges**: a closed-surface edge carries exactly two pcurves in ONE rep
   (CurveOnClosedSurface); selection is by edge orientation within the face.
8. **Degenerated edges** have no 3D curve (Builder::Degenerated nulls it, L1073);
   ranges live on the pcurves; excluded from same-parameter and from
   UpdateEdgeTol.
9. **Valid range**: `FindValidRange` output [first,last] excludes both endpoint
   tolerance spheres; no valid range ⟹ the edge is micro (entirely inside its
   vertices' spheres) and must not survive as an independent edge.
10. **Never move geometry**: the entire doctrine adjusts pcurve
    *parameterizations* and *tolerances*; 3D curve and surface geometry are
    untouched (cf. occt_tolerance-model.md law 1).

---

## PITFALLS

- **Clamped evaluation hides divergence**: without ComputeTol's parametric
  penalty (`DSdu * overshoot`), a pcurve drifting outside a bounded chart
  evaluates at the clamped border and can measure near-zero deviation. Any port
  that samples `S(uv)` must detect uv outside the domain (beyond the 1% band) and
  convert the excess to 3D via 1/Resolution.
- **One broken sample poisons the tolerance**: surface evaluation at poles/seams
  can blow up a single sample; ComputeTol's <10% outlier vote keeps the tolerance
  honest. Symmetric hazard: don't auto-discard big samples when they are >10% —
  that is real divergence.
- **C0 origin drift**: `C0BSplineToC1BSplineCurve` on a periodic pcurve may move
  the curve origin; OCCT restores it by scanning knots for the original origin
  point (done twice). Skipping this silently rotates the seam parameterization.
- **Reparametrize target**: knots go back to `[fC0, lC0]` (the pcurve's own
  range), NOT `[f3d, l3d]` — the commented-out lines L1514/L1614 are bug history.
- **Reparameterization can degrade**: always re-measure after C0→C1 + Reparametrize
  and RESTORE the saved original if error grew (isANA path).
- **Accept-if-not-worse**: Approx_SameParameter's result replaces the pcurve only
  when `tolreached <= error`; and its internal accept gate is deliberately loose
  (`< 250×besttol`) because TolReached is a 45-sample discrete measure — demanding
  `<= tol` livelocks.
- **Numeric-noise rescue (OCC5898)**: for far-from-origin geometry, deviation may
  be below `PrecCurve/PrecSurface` representable noise; without the rescue the
  edge is unfixable and the flag flaps false forever.
- **Fresh-edge lie**: BRep_TEdge ctor sets SameParameter=true. Any constructor
  that attaches unvalidated pcurves must clear the flag or run the algorithm —
  otherwise every consumer trusts a contract nobody checked.
- **`forced` semantics**: InternalSameParameter with forced=true FIRST clears
  SameRange+SameParameter then re-runs; without forced, a set flag short-circuits
  everything (L1256).
- **Trimmed periodic ranges**: don't clamp [f3d,l3d] into the basis curve's
  nominal range when the basis is periodic — seam-crossing edges legitimately
  have l3d > basis LastParameter (NIZHNY-OCC486).
- **Planes are invisible**: stored-rep iteration never sees plane pcurves; the
  shape-level driver needs the dedicated planar-face pass (GetEdgeTol), and
  `BRep_Tool::CurveOnSurface` computes them on the fly (`theIsStored=false`).
- **EmptyCopied invalidates pointers**: after `EmptyCopied()` the rep list and
  handles are new objects — re-fetch (L1281-1283) or you edit the original.
- **FindValidRange end-not-covered**: if a curve endpoint is NOT inside its own
  vertex sphere, the function returns false — callers must distinguish "micro"
  from "vertex doesn't cover its end" (both false here; OCCT callers pre-grow
  tolV by Confusion to absorb intersection noise).
- **Micro slow-out**: near-zero derivative regions (BSpline singularities) make
  the fixed step crawl; the D1-magnitude step-doubling exists exactly for this —
  a naive port loops ~forever on collapsed-parameterization curves.
- **Tolerance read vs stored**: `BRep_Tool::Tolerance` floors at Confusion but
  the stored value may be smaller (ctor RealEpsilon); persist/compare stored,
  reason with floored.

---

## PORT MAP

Our pipeline: `brep_section.cpp build_section_scaffold` (SSI chains, paves,
keep-verdict, valence-1 bridge, welded vertices) → `brep.cpp split_with` (UV
arrangement, shared-chain run lifting, whole-seg keys) → `combine` (exact weld +
tube merge + NK-RESCUE) → winding+radial classification. Our section chains carry
index-corresponded `(p3, uvA, uvB)` triplets — a *discrete* same-parameter
representation by construction.

| # | OCCT mechanism | our anchor | action + 1-line design |
|---|---|---|---|
| 1 | `BRep_TEdge` multi-rep edge: ONE 3D curve + N pcurves + tol + SameParameter/SameRange flags | edges exist only implicitly: per-face UV trims + combine whole-seg alias keys (tol 1e-2) name shared identity | **new-build**: a shared `EdgeGeom{chain3d, per-chart pcurve/run, tol, same_param}` record emitted by split_with and referenced (not copied) by both faces at combine — the alias key becomes a pointer, making tube merge and NK-RESCUE lookups identity-based instead of key-tolerance-based. |
| 2 | Functional pcurve + cubic reparam (`Approx_SameParameter::Build`) | marched chains index-aligned in (p3, uvA, uvB) | **already-equivalent (stronger) — replace not needed**: exact correspondence by construction beats reprojection; adopt it ONLY at STEP import if imported pcurves disagree with 3D curves (validate per #8, refit the CHAIN, not a spline). |
| 3 | `ComputeTol`: 23-sample deviation + out-of-chart parametric penalty + <10% outlier vote + ×1.5 floor 1e-7 | split_with run lifting; UV-arrangement cut-node crossing snap (tangential stalls left 0.008 spurious splits) | **adopt**: after lifting each run, measure `max_i |S(uv_i) − p3_i|` with the out-of-chart penalty (uv beyond 1% span → 3D-equivalent via 1/resolution) and the outlier vote; store as the run's deviation instead of trusting global tol3. |
| 4 | Edge tol = measured maxdist (hard SET, floor Confusion); vertex tols grow to cover (`UpdTolMap`) | global `tolerance` + hand-tuned constants (whole-seg key tol 1e-2, closure-weld cap max(join_tol, forced_node_eps)) | **adopt**: per-edge measured tolerance (the [SCAF-RUN] dev(A/B) numbers already computed) recorded on the EdgeGeom of #1; derive weld caps and alias-key radii from it (×1.05 margin) instead of fixed exponents — shrink where exact, grow where marched. |
| 5 | `FindValidRange` endpoint-sphere subtraction (march ×1.01 step + bisection to eps) | scaffold micro-piece filter + keep-verdict interval drops; trim-snapped sliver class (fb=47.99997 dropped keys) | **adopt exact algorithm**: replace endpoint-parametric epsilon tests with the sphere march+bisection per pave interval — kills slivers wholly inside pave-weld spheres AND rescues short-but-valid blocks whose endpoints merely graze the sphere. |
| 6 | `InternalUpdateTolerances`: V>=E>=F harmonization; vertex covers every rep endpoint evaluation +2·Epsilon | combine exact weld + NK-RESCUE naked-edge audit | **new-build (final audit pass)**: after combine, for each welded vertex evaluate every incident edge chain end (both charts) and grow the vertex's weld-radius record to cover the scatter; drive the naked-edge audit off these per-vertex radii instead of fixed tol multiples. |
| 7 | Seam model: `BRep_CurveOnClosedSurface` = ONE edge, TWO pcurves, orientation-addressed | brep_section seam decomposition (chains split at periodic-seam jumps); SESSION_SCAFFOLD_ALL blocked on seam trims in the splitter | **adopt model**: a seam-crossing section edge on a periodic chart must carry BOTH UV images on that chart keyed by traversal side (uv, uv2), not be split into two unrelated edges — this is the missing piece for scaffold-on-analytic with seam trims. |
| 8 | `BRepLib_ValidateEdge`: 22-point audit, ×1.00001, +max(PrecCurve,PrecSurface), early-exit gate | [SCAF-RUN] MATED debug prints (diagnostic only) | **adopt as permanent gate**: cheap post-split validator per lifted run (same-param path = direct index compare since runs are aligned); failure = partial-lift detection (the 0.008 spurious-split class) BEFORE faces are built, with the early-exit trick for speed. |
| 9 | `GeomLib::SameRange` affine range normalization + SameRange flag | chain-index parameterization shared across both operands by construction | **already-equivalent**: our runs are same-range by construction; keep; only relevant if #1 ever stores resampled pcurves per chart. |
| 10 | `BoundingVertex` minimal enclosing sphere (2-sphere formula; sorted barycenter n>2) | scaffold closure-weld + valence-1 bridge weld (midpoint/first-wins today) | **adopt formula**: weld center `0.5(Pm+Pn − dir·dR/D)`, radius `0.5(Rm+Rn+D)` for pair welds; coordinate-sorted barycenter + covering radius for junction clusters — deterministic and minimal, removes weld-order dependence. |
| 11 | Plane pcurves never stored; on-the-fly `CurveOnPlane` projection + dedicated `GetEdgeTol` planar pass | our planar faces store trims like any surface | **skip (document)**: storing planar trims is fine (simpler than OCCT); but do NOT port any OCCT logic that assumes plane reps are absent (e.g. GetEdgeTol's early-return when a stored rep exists). |
| 12 | `BuildCurve3d` (pcurve→3D reverse direction, exact on planes) | section chains ARE the 3D curve; STEP writer compresses to cubics separately | **skip for booleans / adopt for import repair**: STEP files with missing 3D curves (pcurve-only edges are legal STEP) should rebuild via chart evaluation of the pcurve + our two-tier cubic fit. |
