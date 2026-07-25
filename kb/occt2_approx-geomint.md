# OCCT spec: ApproxInt + GeomInt — walked-line -> NURBS approximation (the coupled 3D+pcurve fit)

Sources read (2026-07-24, OCCT @ C:/brg/compas_occt/external/occt/src/occt/src):
- ModelingAlgorithms/TKGeomAlgo/ApproxInt: ApproxInt_Approx.gxx, ApproxInt_KnotTools.{hxx,cxx}, ApproxInt_MultiLine.gxx, ApproxInt_MultiLineTool.lxx, ApproxInt_ImpPrmSvSurfaces.gxx, ApproxInt_PrmPrmSvSurfaces.gxx
- ModelingAlgorithms/TKGeomAlgo/GeomInt: GeomInt_IntSS.cxx, GeomInt_IntSS_1.cxx, GeomInt_WLApprox.hxx (+ _0.cxx instantiation stubs)
- ModelingData/TKGeomBase/Approx: Approx_ComputeLine.gxx, Approx_BSplComputeLine.gxx, Approx_MCurvesToBSpCurve.cxx
- ModelingData/TKGeomBase/AppParCurves: AppParCurves_LeastSquare.gxx (Error), AppParCurves_Gradient.gxx

Related specs (do not re-read here): kb/occt_ssi-walking.md (produces the IntPatch_WLine consumed here),
kb/occt_tolerance-model.md (what TolReached feeds), kb/occt_pavefiller-core.md, kb/occt_builder-assembly.md.
This file covers ONLY: walked polyline -> (C3d, pcurve1, pcurve2) BSpline triple with verified tolerance.

Generic-class decoder (the `_0.cxx` files bind gxx templates):
- `GeomInt_TheComputeLineOfWLApprox`       = Approx_BSplComputeLine.gxx  (single BSpline, C2 path)
- `GeomInt_TheComputeLineBezierOfWLApprox` = Approx_ComputeLine.gxx      (per-knot-interval Bezier path)
- `Approx_MyGradient` etc.                 = AppParCurves_Gradient.gxx / AppParCurves_LeastSquare.gxx
- `TheSvSurfaces`                          = ApproxInt_ImpPrmSvSurfaces (quadric x param) or ApproxInt_PrmPrmSvSurfaces (param x param)

---

## STAGE PIPELINE (ordered)

1. **GeomInt_IntSS::InternalPerform** (GeomInt_IntSS.cxx)
   TolArc = TolTang = user Tol; Deflection = 0.1, divided by 10 iff BOTH surfaces BSpline.
   Runs IntPatch intersector, then per line -> MakeCurve.

2. **GeomInt_IntSS::MakeCurve** (GeomInt_IntSS_1.cxx) — the orchestrator for `IntPatch_Walking`:
   a. `GeomInt_LineTool::DecompositionOfWLine(WL, HS1, HS2, aTolSS=2e-7, ...)` — split WLine into parts near boundaries.
   b. `ApproxInt_KnotTools::DefineParType(WL, ifprm, ilprm, approxXYZ, approxU1V1, approxU2V2)` — choose parametrization per part.
   c. `theapp3d.SetParameters(myTolApprox=1e-7, tol2d=1e-7, DegMin=4, DegMax=8, NbIterMax=0, NbPntMax=30, ApproxWithTangency=(HS1!=HS2), aParType)`.
   d. Plane shortcut: if a surface is a plane -> `Perform(..., ApproxXYZ=false, thatPlaneUV=true, ...)`; fit runs in 2D only,
      3D poles are lifted pole-by-pole with `ElSLib::Value(u,v,Pln)`; `myTolReached3d = myTolReached2d`.
   e. `theapp3d.Perform(HS1, HS2, WL, ApproxXYZ, ApproxU1V1, ApproxU2V2, ifprm, ilprm)`.
   f. If `!theapp3d.IsDone()` -> **fallback**: `MakeBSpline` / `MakeBSpline2d` = degree-1 BSpline through the raw walked
      points (knots 0..n-1, end mults 2). A result is ALWAYS produced.
   g. Success: per MultiBSpCurve build Geom_BSplineCurve + Geom2d_BSplineCurve from the SAME knots/mults/degree
      (channel 1 = 3D poles, channels 2/3 = UV poles); `GeomLib_CheckBSplineCurve/Check2dBSplineCurve::FixTangent(true,true)`
      with myTolCheck=1e-7, myTolAngCheck=1e-6; closure snap (see INVARIANTS 7); `AdjustUPeriodic` translates the pcurve by
      period multiples (sample at t1+0.467*(t2-t1)); TolReached2d/3d aggregated by max.

3. **ApproxInt_Approx::Perform(Surf1, Surf2, theline, ...)** (ApproxInt_Approx.gxx) — surface-type dispatch:
   if either surface is Plane/Cylinder/Sphere/Cone -> `IntSurf_Quadric` + `ApproxInt_TheImpPrmSvSurfaces` (analytic-param,
   `SetUseSolver(false)` initially — corrector off for plain tangency queries, on for refinement);
   else `ApproxInt_ThePrmPrmSvSurfaces` (param-param, Int2S corrector, TOLTANGENCY=1e-10).
   Then: `prepareDS` -> `myBezierApprox = (indicemax-indicemin >= aMinNbPointsForApprox=5)` -> `fillData` -> `buildKnots`
   -> `myComputeLine/.Bezier.Init(4, 8, tol3d, tol2d, nbIter, cut=myBezierApprox, parametrization)` -> `buildCurve`.

4. **ApproxInt_Approx::fillData** — conditioning translation: `ComputeTrsf3d/ComputeTrsf2d` compute
   Xo=-minX, Yo=-minY, Zo=-minZ, U1o=-minU1, ... over the line; the MultiLine adds these offsets on every `Value()`;
   after fitting, `buildCurve` calls `Transform(indice3d, -Xo, 1, -Yo, 1, -Zo, 1)` / `Transform2d` to shift poles back.

5. **ApproxInt_Approx::Parameters(Line, firstP, lastP, Par, TheParameters)** — THE coupled parametrization:
   one scalar chord per step, `dist = sqrt( sum_j |P3d_i - P3d_{i-1}|^2 + sum_j |P2d_i - P2d_{i-1}|^2 )` over ALL channels
   (3D + both UV, translated coords). ChordLength: cumulative dist; Centripetal: cumulative sqrt(dist); normalized to [0,1].
   IsoParametric: uniform by index.

6. **ApproxInt_KnotTools::DefineParType** (ApproxInt_KnotTools.cxx) — parametrization choice from curvature:
   2 points -> IsoParametric. Else build the D-dim coord array (D = 3*XYZ + 2*U1V1 + 2*U2V2, max 7), chord-length params,
   `BuildCurvature`; default ChordLength; switch to Centripetal iff `maxCurv/midCurv > aCritRat=500` AND
   (ratio > 5*500 OR `MaxParamRatio(aPars) > aCritParRat=100`). Linear (maxCurv < PConfusion) or infinite -> ChordLength.

7. **ApproxInt_Approx::buildKnots** -> **ApproxInt_KnotTools::BuildKnots(PntsXYZ, PntsU1V1, PntsU2V2, pars, ..., theMinNbPnts=myNbPntMax=30, theKnots)**
   Knots are point INDICES into the walked line. Algorithm (Razdan, "Knot Placement for B-Spline Curve Approximation"):
   - `BuildCurvature`: discrete curvature at each point in D-dim: 3-point Lagrange (PLib::EvalLagrange, degree 2) gives V1,V2;
     `EvalCurv = |V1 ^ V2| / |V1|^3` with the outer product generalized to D dims (sum of squared 2x2 minors);
     |V1|^2 < 1/Precision::Infinite() -> return Infinite (knot at singularity).
   - `ComputeKnotInds`: (I) knots at curvature EXTREMA: d1*d2>0 with |d1|,|d2|>eps=1e-9, or plateau-jump
     (|d1|<eps && |d2|>eps1=1e-6, or symmetric); (II) `InsKnotBefI` fills monotone intervals: insert where
     curvature ratio to interval start > aLimitCurvatureChange=3.0, or at the curvature midpoint if the turning angle
     test passes: `mp > aSinCoeff2 * m1 * m2` with aSinCoeff2 = (3-sqrt(5))/8 = 0.09549150281252627 (sin^2, i.e. turn > ~18deg);
     (III) re-check feature points with the same angular criterion, adding knots on the steeper side.
   - `FilterKnots`: max points per interval = aMaxPntCoeff=15 * theMinNbPnts (bisect longer); merge intervals with
     < theMinNbPnts points (several rebalancing cases, aMinNbStep = theMinNbPnts/2); last knot forced to last index.
   - Post rule in Perform: if only 2 knots survive but range > 2*myNbPntMax -> force a middle knot ("At least 3 knots for BrepApprox").

8. **ApproxInt_Approx::buildCurve** — per knot interval [myKnots(k), myKnots(k+1)] build an `ApproxInt_TheMultiLine`
   window and run `myComputeLineBezier.Perform` (Approx_ComputeLine.gxx); append every produced MultiCurve into
   `Approx_MCurvesToBSpCurve myBezToBSpl`; un-translate poles; loop; finally `myBezToBSpl.Perform()` concatenates the Bezier
   segments into ONE MultiBSpCurve (Convert_CompBezierCurvesToBSplineCurve — all channels share the resulting knot vector).
   Non-Bezier path (<5 pts): single `myComputeLine.Perform` (Approx_BSplComputeLine with `SetContinuity(2)` from the ctor).

9. **Approx_ComputeLine::Perform / Compute** (Approx_ComputeLine.gxx) — degree ladder + tolerance verification loop:
   - `Compute(Line, fpt, lpt, Para, tol3d, tol2d, indbad)`: for deg = min(nbp-1, degmin=4) .. Mdegmax
     (Mdegmax clamped to nbp-5 when cutting): run `Approx_MyGradient` (= AppParCurves_Gradient.gxx):
       LSQ solve for poles at fixed params (AppParCurves_ParLeastSquare — ONE system, shared collocation matrix A over all
       channels; constraints via Uzawa) -> ONE Rogers&Fog/Hoschek projection step on interior parameters
       (DU = (C-P).C' / |C'|^2 summed over ALL channels, clamped to +-5e-2) -> optional BFGS (Eps=1e-7, NbIterations) —
       **NbIterMax=0 in the IntSS/BOP configuration, so the BFGS is skipped**; error re-measured after correction.
     Accept iff `TheTol3d <= mytol3d && TheTol2d <= mytol2d` (both, separately); else keep best-so-far
     (currenttol3d/2d monotone), try next degree. Monotonicity guard: if the projection step broke param ordering,
     restore saved params ("restau").
   - On accept, `CheckMultiCurve`: pole-turn scan, reject if consecutive pole vectors have dot < MinScalProd=-0.9 AND the
     walked points themselves don't corroborate a real loop AND max/min step ratio >= 4 -> returns indbad ->
     `LineTool::MakeMLOneMorePoint` inserts one corrector-verified midpoint, recursive `Perform` on the new line.
   - Cutting loop (mycut=true): on failure bisect `mylastpt = (myfirstpt+mylastpt)/2`; when nbp <= degmax+5:
     if `WhatStatus == Approx_PointsAdded` (SvSurfaces available) -> `MakeMLBetween` re-walk with more points (recursive
     Perform on it, myMultiLineNb depth cap 3); if no points can be added and nbp <= degmax+1 -> `ComputeCurve` interpolation:
     nbp==2 -> straight multiline poles; else deg = nbp+1 Bezier LSQ with `AppParCurves_TangencyPoint` end constraints,
     end tangents from SvSurfaces or a 3-point parabola LSQ (`FirstTangencyVector`/`LastTangencyVector`), tangent
     magnitudes scaled by `SearchFirstLambda/SearchLastLambda` (lambda = |P1P2|/(|V|*(u2-u1)), signed, /deg).
     Every stored piece appends its own (tol3d, tol2d) into Tolers3d/2d.

10. **Approx_BSplComputeLine::Compute** (Approx_BSplComputeLine.gxx) — the C2 single-BSpline path:
    degree ladder degmin..degmax; interior mults = max(1, deg - mycont) with mycont=2 -> C2 for deg>=3; end mults deg+1;
    if nbpoles (+1 per Tangency/Curvature end constraint) > nbp -> `Interpol`: cubic with knots = point params
    (interpolation, nbpoles = nbp+2), end tangents from parabola (nbp 3..4) or a <=9-point Bezier LSQ probe, periodic lines
    average V1/V2. Otherwise `Approx_MyBSplGradient` (BSpline LSQ + gradient, lambda seeds from
    `Approx_BSpParLeastSquareOfMyBSplGradient::FirstLambda/LastLambda` at degmin) or `Approx_MyGradientbis` when the knot
    vector is Bezier-like (nbpoles == deg+1). No-cut variant increments interior knot count (uniform in existing params)
    until tolerance reached: `nbknots++` per outer loop.

11. **Error accounting** — `AppParCurves_LeastSquare::Error(F, MaxE3d, MaxE2d)` (AppParCurves_LeastSquare.gxx:1214):
    per point i, per channel k: Fi = squared residual; MaxE3d = sqrt(max over 3D channels/points), MaxE2d likewise for the
    2D channels; F = total sum (all channels). One matrix, separate maxima.
    `ApproxInt_Approx::UpdateTolReached` = max over Bezier segments of `myComputeLineBezier.Error(ICur, Tol3D, Tol2D)`.
    Public `TolReached3d() = myTolReached3d * RatioTol` (RatioTol=1.5); symmetric on input: `myTol3d = Tol3d / 1.5`.
    `IsDone()` = (Bezier path: >=1 MultiCurve produced) OR (single path: `IsToleranceReached()`).

12. **Tangency/refinement backends** (the corrector that makes added points EXACT, not interpolated):
    - `ApproxInt_ImpPrmSvSurfaces::Compute` (ApproxInt_ImpPrmSvSurfaces.gxx): math_FunctionSetRoot on the implicit-zero
      function over the parametric surface (Tolerance 1e-8, bounds = surface UV box, `FillInitialVectorOfSolution` handles
      period translation + inward nudge by UResolution(Confusion)); REJECT if the corrector moved u or v by > 0.001;
      3D point = barycenter of the two surface evaluations; quadric U unwrapped by +-2PI to stay near the previous value;
      3D tangent = cross of unit normals; 2D tangents via `NonSingularProcessing`: from Tg = DU*x + DV*y,
      x = +-sqrt(|Tg^DV|^2/|DU^DV|^2) signed by (Tg^DV).N, y likewise (exact, no least squares); `SingularProcessing`
      handles |DU|~0 / |DV|~0 / DU||DV (aNullValue = Precision::Approximation()^2 = 1e-12, anAngTol = Precision::Angular()=1e-12).
      Two-deep result cache (`MyHasBeenComputed/bis`) keyed on exact (u1,v1,u2,v2).
    - `ApproxInt_PrmPrmSvSurfaces::Compute` (ApproxInt_PrmPrmSvSurfaces.gxx): IntImp Int2S corrector
      (TOLTANGENCY=1e-10); tangent = Int2S direction (reject when `IsTangent()`); 2D tangents from the 2x2 Gram system
      DeltaU = (Tg.TU*TVTV - Tg.TV*TUTV)/DIS, DIS = TUTU*TVTV - TUTV^2, reject |DIS| < Precision::Angular().
    - `ApproxInt_MultiLine::Tangency` (ApproxInt_MultiLine.gxx): thin adapter; returns false (-> constraint downgrade)
      when no SvSurfaces or the corrector fails.

13. **Point-insertion services** (ApproxInt_MultiLine.gxx):
    - `MakeMLBetween(Low, High, nbToInsert)`: constant 3D-arc-length resampling (AC = cumulative |P_{i-1}P_i|,
      ds = AC(High)/(NbPnts-1), skip if new point closer than dsmin = 0.3*ds to an existing one); original walked points are
      KEPT interleaved; each new point solved by `SvSurfaces::Compute` (true SSI corrector, UseSolver forced on);
      a-posteriori "virage" check: for each consecutive UV triple on both surfaces, deviation from linear extrapolation
      d^2 > 0.25*prevStep^2 -> reject the WHOLE resample; also reject if count < nbToInsert+High-Low+1;
      SvSurfaces pointer propagated to the new MultiLine only when High-Low > 10 (else refinement recursion stops).
    - `MakeMLOneMorePoint(Low, High, indbad, out)`: single corrector-verified midpoint (`SeekPoint`) inserted before indbad;
      rejected if mid==cur within 1e-8 in UV, if the new 3D point is within Precision::SquareConfusion of a neighbor, or if
      the corrector displacement exceeds the half-step it was seeded with.

14. **Bezier -> BSpline concatenation** — `Approx_MCurvesToBSpCurve::Perform` (Approx_MCurvesToBSpCurve.cxx):
    single segment -> knots {0,1} mults deg+1; multiple -> `Convert_CompBezierCurves(2d)ToBSplineCurve` on channel 1,
    remaining channels converted pole-parallel; all channels share the final knot vector.

15. **Restriction lines** — `GeomInt_IntSS::TreatRLine` + `BuildPCurves` (GeomInt_IntSS_1.cxx): 3D curve re-approximated
    from the arc pcurve by `Approx_CurveOnSurface(anAHC2d, aGAHS, tf, tl, Precision::Confusion())` with
    Perform(aMaxSeg=1000, aMaxDeg=8, GeomAbs_C1); missing pcurve on the OTHER surface projected by
    `GeomProjLib::Curve2d`; degenerate-edge guard `isDegenerated` (ends + midpoint within Confusion^2);
    range < 2e-9 -> 2-point degree-1 pcurve from Extrema_ExtPS with midpoint same-parameter distance added into Tol;
    BSpline pcurve knots re-pinned to [f,l] by `BSplCLib::Reparametrize` when off by > PConfusion.

## DATA STRUCTURES

- `IntPatch_WLine` / `IntSurf_LineOn2S` of `IntSurf_PntOn2S` = {gp_Pnt, (u1,v1), (u2,v2)} — the walked-triple record.
  EVERYTHING downstream keeps the triple together; there is no independent re-projection of pcurve points.
- `ApproxInt_MultiLine` (ApproxInt_MultiLine.gxx): windowed view [indicemin..indicemax] over the WLine +
  translation offsets Xo..V2o + `void* PtrOnmySvSurfaces` + nbp3d in {0,1}, nbp2d in {0,1,2}, p2donfirst.
  `WhatStatus()` = Approx_PointsAdded iff SvSurfaces present (drives whether refinement is possible).
- `AppParCurves_MultiPoint / MultiCurve / MultiBSpCurve`: N-channel pole containers — one 3D channel + up to two 2D
  channels sharing degree/knots/params. THE structural embodiment of the simultaneous fit.
- `GeomInt_WLApprox::Approx_Data` (GeomInt_WLApprox.hxx): {myBezierApprox, Xo..V2o, ApproxXYZ/U1V1/U2V2,
  indicemin/max, myNbPntMax=30, parametrization (default ChordLength)}.
- `myKnots : NCollection_DynamicArray<int>` — knots as walked-point indices, not parameters.
- `AppParCurves_ConstraintCouple` (index, AppParCurves_Constraint in {NoConstraint, PassPoint, TangencyPoint, CurvaturePoint}).
- `Approx_MCurvesToBSpCurve` — Bezier segment accumulator (Reset/Append/Perform/Value).

## CONSTANTS & TOLERANCES (exact values)

| where | constant | value |
|---|---|---|
| GeomInt_IntSS_1.cxx MakeCurve | myTolApprox (3D), tol2d | 1e-7, 1e-7 |
| GeomInt_IntSS_1.cxx MakeCurve | aTolSS (DecompositionOfWLine) | 2e-7 |
| GeomInt_IntSS_1.cxx MakeCurve | DegMin, DegMax, NbIterMax, NbPntMax | 4, 8, **0**, 30 |
| GeomInt_IntSS.lxx | myTolCheck, myTolAngCheck (FixTangent) | 1e-7, 1e-6 |
| GeomInt_IntSS.cxx | Deflection | 0.1 (0.01 iff BSpl x BSpl) |
| ApproxInt_Approx.gxx | aMinNbPointsForApprox (Bezier-cut threshold) | 5 |
| ApproxInt_Approx.gxx | RatioTol (internal = user/1.5, reported = internal*1.5) | 1.5 |
| ApproxInt_Approx.gxx ctor | default degmin/degmax/tol3d/tol2d/itermax, continuity | 4/8/0.001/0.001/5, C2 |
| ApproxInt_KnotTools.cxx | aSinCoeff2 (angular knot criterion, sin^2 ~18deg) | (3-sqrt5)/8 = 0.09549150281252627 |
| ApproxInt_KnotTools.cxx | aMaxPntCoeff (max pts/interval = 15*min) | 15 |
| ApproxInt_KnotTools.cxx | curvature-extrema eps, eps1 | 1e-9, 1e-6 |
| ApproxInt_KnotTools.cxx InsKnotBefI | aLimitCurvatureChange | 3.0 |
| ApproxInt_KnotTools.cxx DefineParType | aCritRat, hard switch, aCritParRat | 500, 2500, 100 |
| ApproxInt_ImpPrmSvSurfaces.gxx | corrector tolerance (both UV) | 1e-8 |
| ApproxInt_ImpPrmSvSurfaces.gxx | max corrector drift accepted | 0.001 (u and v separately) |
| ApproxInt_ImpPrmSvSurfaces.gxx | aNullValue, anAngTol | Approximation()^2 = 1e-12, Angular() = 1e-12 |
| ApproxInt_ImpPrmSvSurfaces.gxx | boundary fuzz in FillInitialVectorOfSolution | 1e-10 |
| ApproxInt_PrmPrmSvSurfaces.gxx | TOLTANGENCY (Int2S) | 1e-10 |
| ApproxInt_PrmPrmSvSurfaces.gxx | Gram-determinant singular reject | < Precision::Angular() = 1e-12 |
| ApproxInt_MultiLine.gxx MakeMLBetween | dsmin, virage coeff, SvSurf keep-depth | 0.3*ds, 0.25*step^2, High-Low>10 |
| ApproxInt_MultiLine.gxx MakeMLOneMorePoint | UV degeneracy tol, 3D dedup | 1e-8, Precision::SquareConfusion()=1e-14 |
| Approx_ComputeLine.gxx CheckMultiCurve | MinScalProd (pole fold), step-ratio coeff | -0.9, 4.0 |
| Approx_ComputeLine.gxx Compute | Mdegmax clamp when cutting | nbp-5 |
| Approx_ComputeLine.gxx Perform | add-points window, interpolate window | nbp <= degmax+5, nbp <= degmax+1 |
| AppParCurves_Gradient.gxx | Hoschek step clamp, BFGS Eps | 5e-2, 1e-7 |
| Approx_BSplComputeLine.gxx Compute | param sanity band for saving | (-1e-6, 1+1e-6) |
| GeomInt_IntSS_1.cxx (closure) | closed-endpoint threshold | dist^2 < 2*Epsilon(max end |P|^2) |
| GeomInt_IntSS_1.cxx AdjustUPeriodic | sample fraction, aEps, aEpsilon | 0.467, PConfusion()=1e-9, Epsilon(10.)~1.77e-15 |
| GeomInt_IntSS_1.cxx TreatRLine | aMaxSeg, aMaxDeg, continuity | 1000, 8, GeomAbs_C1 |
| GeomInt_IntSS_1.cxx BuildPCurves | tiny-range threshold | (l-f) <= 2e-9 -> line-segment pcurve |
| GeomInt_IntSS_1.cxx TrimILineOnSurfBoundaries | boundary intersection tol | 10*Confusion() = 1e-6 |

## INVARIANTS

1. **Same-parameter by construction**: 3D curve and both pcurves are solved as channels of ONE least-squares system with a
   single shared parameter vector, one collocation matrix, one knot vector. There is no reprojection step for WLine curves;
   same-parameter is not repaired afterwards, it never breaks.
2. **Coupled metric everywhere**: chord parametrization, Hoschek parameter correction, knot curvature, and loop checks all
   operate on the concatenated (up to 7-dim) coordinate vector. A feature in UV (seam approach, pole) shapes the 3D fit
   and vice versa.
3. **Tolerance is verified, never assumed**: accept only when measured MaxE3d <= tol3d AND MaxE2d <= tol2d; on failure the
   ladder continues (degree -> split -> densify -> interpolate); the best-so-far curve is retained with its measured error;
   the caller reads TolReached3d/2d (x1.5) and inflates edge tolerances accordingly (see occt_tolerance-model.md).
4. **Degree ladder, not degree guess**: try deg 4..8; degree is capped by point count (deg <= nbp-1; when cutting nbp-5).
   Interpolation is the floor (few points), approximation the norm.
5. **Refinement returns to the true intersection**: every inserted point (MakeMLBetween/MakeMLOneMorePoint/SeekPoint)
   is solved by the SSI corrector — the walked polyline is a seed, never the ground truth.
6. **End constraints degrade gracefully**: TangencyPoint requested; `FindRealConstraints`/Tangency failure downgrades to
   PassPoint per end. Endpoints are always interpolated (PassPoint minimum) — segment joins are exact C0, G1 when tangents exist.
7. **Closure snap**: if a WLine 3D curve's endpoints coincide within 2*Epsilon(scale) but IsClosed() fails, both end poles
   are set to their average (deg-1 2-pole curves discarded instead).
8. **Knots-from-indices**: knot placement selects walked-point indices (curvature features), each interval fit
   independently as a Bezier, minimum 3 knots when the line is long (>2*30 points); a knot interval always contains
   >= NbPntMax/2 and <= 15*NbPntMax points.
9. **Conditioning translation**: all channels are shifted to min=0 before fitting and shifted back after — pole math is done
   near the origin.
10. **Result always exists**: approximation failure produces the degree-1 polyline BSpline over the raw walked triple —
    downstream must accept C0.

## PITFALLS

- **NbIterMax = 0 in production**: the BOP/IntSS configuration disables the BFGS refinement entirely; the fit quality comes
  from LSQ + ONE Hoschek projection step per degree. Porting the gradient machinery is wasted effort; porting the projection
  step is mandatory.
- **RatioTol=1.5 double-speak**: internal tolerances are user/1.5 and the report is internal*1.5 — a "reached" 1e-7 request
  may legitimately report up to 1.5e-7. OCCT comments say "should be removed in the future". Do not copy blindly into a
  system that compares TolReached against the original request.
- **Plane path swaps metrics**: with a plane operand ApproxXYZ=false — the 3D error is NEVER measured, it is 2D error times
  plane isometry (exact). `myTolReached3d = myTolReached2d` is only valid because the plane lift is an isometry.
- **Constraint downgrade is silent**: a failed tangency at one walked point (tangent zone, corrector drift > 0.001) makes
  that Bezier end PassPoint -> possible G1 kink at a knot join with zero diagnostics.
- **Curvature knots on the combined curve**: near-seam UV velocity spikes generate knots even when the 3D trace is a clean
  arc — this is intentional (pcurve tolerance needs them). A 3D-only knot criterion under-resolves pcurves.
- **MakeMLBetween all-or-nothing**: one bad "virage" (UV extrapolation deviation > 0.25*step^2) rejects the entire densified
  line, falling back to bisection. Arc-length is measured in 3D only (UV abscissa was tried and abandoned — see the #if 0).
- **SvSurfaces caches by exact FP equality** and mutates its inputs (corrector snap-back writes into u1..v2, quadric U
  unwrapped by +-2PI). Two-deep cache swap means query order affects which cached value is returned first.
- **MakeBSpline fallback is degree-1 with mult-2 ends** — consumers (sewing, sameparameter, STEP writers) see polylines;
  our own step-writer sliver problem (see project memory) is exactly this class of output.
- **Params re-normalized per sub-interval** in the cutting loop ((t-pfirst)/(plast-pfirst)) — segment params are local [0,1];
  concatenation re-scales via Convert_CompBezier knot ratios, NOT via original chord lengths.
- **CheckMultiCurve only guards channel 1** (and simple cases nbp3d<=1): folds in the second pcurve of a 3-channel fit are
  caught only via the shared indbad heuristics.
- **BuildPCurves never overwrites**: it is a no-op when C2d already exists — order of TreatRLine operations matters.
- **AdjustUPeriodic samples one interior point** (0.467 fraction): a pcurve legitimately crossing the period boundary is
  translated as a whole, never split — correctness relies on DecompositionOfWLine having split at seams first.

## PORT MAP (OCCT mechanism -> our anchor -> action)

| OCCT mechanism | our anchor | action |
|---|---|---|
| Coupled 7-dim chord/centripetal parametrization (ApproxInt_Approx::Parameters) | step-writer section-polyline -> cubic fit (two-tier, endpoints pinned); brep_section chain sampling | **adopt**: parametrize compression fits by sqrt(sum 3D^2 + sum UV^2) chords so the SAME fit serves 3D edge and pcurve. |
| Simultaneous multi-channel LSQ, one knot vector (AppParCurves_MultiCurve/LeastSquare) | brep.cpp split_with run lifting: 3D chains and UV runs currently lifted/fit separately | **new-build**: single fit producing (C3d, pcu_A, pcu_B) with shared knots — kills same-parameter drift at STEP export and Rhino trim rejection. |
| Curvature knot seeding + FilterKnots (ApproxInt_KnotTools::BuildKnots) | step-writer ncv ladder (8-> gate 12/64 CVs) | **replace**: seed knot count/placement from discrete-curvature features (aSinCoeff2, ratio 3.0, 15x cap) instead of doubling CV counts; keep our exact/section tolerance tiers as the accept test. |
| Verified-tolerance ladder: accept only if MaxE3d<=tol && MaxE2d<=tol, else degree++/split/densify (Approx_ComputeLine::Compute/Perform) | step-writer two-tier accept (diag*1e-7 then diag*2e-5) | **adopt**: add per-channel (2D) error to the accept test and bisect-on-failure instead of falling back to raw polyline immediately. |
| Hoschek projection parameter step, clamp 5e-2 (AppParCurves_Gradient ctor) | our cubic fits use fixed chord params | **adopt**: one projection pass measurably drops max error at zero architectural cost; skip BFGS (OCCT itself runs 0 iterations). |
| SvSurfaces corrector-backed point insertion (MakeMLBetween / MakeMLOneMorePoint / SeekPoint) | brep_section.cpp build_section_scaffold: newton_cc pave refinement, valence-1 bridge march | **adopt** (pattern exists): drive densification by FIT FAILURE (which interval missed tolerance), not by uniform step; keep 0.3*ds dedup and the whole-or-nothing virage check. |
| Tangency from corrector (ImpPrm cross-normals + NonSingularProcessing exact 2D decomposition; PrmPrm Gram system) | our SSI marcher already computes tangents for stepping; not exposed to any fit | **adopt**: expose (T3d, Tuv_A, Tuv_B) per chain point; use as Bezier end constraints; copy the downgrade-to-PassPoint rule verbatim. |
| TolReached bookkeeping (UpdateTolReached max, x1.5 report) -> caller inflates tolerance | combine (exact weld + tube merge + NK-RESCUE): weld tol currently independent of fit error | **adopt**: carry fit TolReached3d per section curve into combine weld cap: weld_tol = max(join_tol, tol_reached); drop the 1.5 factor (report the measured max directly). |
| Degree-1 polyline fallback always produced (MakeBSpline) | our sections are already polylines; step writer falls back to deg-1 | **keep** (already equivalent) — never let a failed fit delete a section curve. |
| Pole-fold loop detection (CheckMultiCurve, dot < -0.9) + one-point insert | step-writer cubic compression; scaffold closure-weld | **new-build**: cheap guard on fitted CVs; a fold means the marched polyline under-sampled a turn — insert a corrector midpoint and refit. |
| Centripetal switch at curvature ratio 500 (DefineParType) | section chains near tangential crossings (our newton_cc stall class) | **adopt**: near-singular chains (curvature spike ratio) fit with centripetal params — directly targets the tangential-SSI sliver class from the 4544770f round. |
| Plane 2D-fit + exact lift (MakeCurve plane branch, ElSLib::Value) | our planar faces use exact arcs/lines already | **keep**; extend rule "never approximate in 3D what is exact in a plane" to scaffold outputs on planar operands. |
| AdjustUPeriodic post-fit period translation | split_with UV arrangement seam handling (seam-decomposition groundwork, f633bf12) | **adopt** with caveat: only valid AFTER seam splitting; translate whole pcurve by period using one interior sample (0.467), never per-point rewrap. |
| Min-3-knots rule + 30-point interval floor | brep_section chain segmentation | **adopt** as sanity floor when converting long marched chains (>60 pts) to single curves. |
