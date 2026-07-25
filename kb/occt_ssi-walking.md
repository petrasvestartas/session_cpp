# ssi-walking

OCCT surface-surface intersection by marching ("walked lines"), extracted from source at
`C:/brg/compas_occt/external/occt/src/occt/src/ModelingAlgorithms/TKGeomAlgo/` (IntPatch, IntWalk, IntStart, IntSurf, ApproxInt).
This subsystem is the direct OCCT counterpart of our `build_section_scaffold` SSI march and the
authoritative reference for the graze-stall defect class (stalled/tangent marches completed to
boundaries by *minimization*, not by more marching).

---

## STAGE PIPELINE

Ordered as executed for a freeform x freeform (Prm-Prm) pair; Imp-Prm and post stages included where they run.

### Stage 0 — Dispatch, tolerance setup, surface classification
- **File/fn**: `IntPatch_Intersection.cxx` — `IntPatch_Intersection::Perform` (~L1066–1372), `SetTolerances` (~L134), `DefineUVMaxStep` (~L2372–2408), `CheckSingularPoints` (~L2286–2368).
- **Purpose**: choose Geom-Geom (`GeomGeomPerfom` ~L1778) / Geom-Param (`GeomParamPerfom` ~L1909) / Param-Param (`ParamParamPerfom` ~L1659) intersector; set the two global walk parameters:
  - `myFleche` (3D deflection target, default **0.01** if unset, ~L1078);
  - `myUVMaxStep` ("Increment", default **0.01** here; callers use `DefineUVMaxStep` = **0.001**, reduced to **0.0001** when one operand has a parametric singular boundary whose D1-norm < `Precision::Confusion()` and the singular point sits within `(1e-7, 1e-5)` of the other surface — the sphere-pole-graze mitigation).
- **Inputs**: two `Adaptor3d_Surface` + `Adaptor3d_TopolTool` domains, `TolArc`, `TolTang` (typ. 1e-7 from BOP).
- **Outputs**: `slin` (sequence of `IntPatch_Line`: GLine/ALine/RLine/WLine), `spnt` (isolated tangency points).
- **When**: entry point of every SSI. Cone/torus near-degenerate cases forcibly re-typed to `GeomAbs_BezierSurface` to be sent to the *parametric* walker (~L1244–1260) — OCCT deliberately walks analytic surfaces when the analytic path is ill-conditioned.
- **Post**: after all intersectors, every WLine with `IsPurgingAllowed()` is replaced by `IntPatch_WLineTool::ComputePurgedWLine` (~L1346–1371).

### Stage 1 — Start-point seeding (Prm-Prm)
- **File/fn**: `IntPatch_PrmPrmIntersection.cxx`, three live variants:
  1. **Polyhedron interference** — `Perform(Surf1,Poly1,D1,Surf2,Poly2,D2,TolTangency,Epsilon,Deflection,Increment)` (~L369–928). `IntPatch_InterferencePolyhedron` intersects the two sampling polyhedra → `nbLigSec` polygonal section lines + tangent zones. Section lines sorted by descending point count (~L419–434). Per line: UV min/max over all its points define the **walk clamp box** passed to the walker (this is why OCCT walks in a *shrunk* box, then extends: see Stage 5). Start point = the section-line point with **minimal incidence** (most transversal crossing) searched from the middle outward in `[nbp/4, 3nbp/4]`, each better candidate must beat `0.9*CurrentIncidence` (~L508–544); fallback schedule: middle, 1, last, then all points.
  2. **IntPolyh grid** — `Perform(...,ClearFlag)` (~L2488–3100). `D1->SamplePnts(Deflection,10,10)` adaptive sampling (≤ `Limit=2500` nodes), `IntPolyh_Intersection` gives section lines with per-point `(x,y,z,U1,V1,U2,V2,incidence)`; start-point schedule per line: `nbp/2`, `1`, `nbp-1`, `3nbp/4`, `nbp/4`, then every point (~L2679–2709). **This is the variant that calls `PutToBoundary` + `SeekAdditionalPoints`** (~L2767–2777).
  3. **Pre-seeded points** — `Perform(...,NCollection_List<IntSurf_PntOn2S>& LOfPnts)` (~L1827–2172): seeds come from the caller (IntTools BOP reuses previous run's points). **Seam duplication**: for U/V-closed surfaces every seed sitting on a period boundary (within `Precision::PConfusion()`) is cloned to the opposite bound (~L1866–1959) so a chain starting on the seam can be walked in both charts. The UV clamp box is the natural bounds *extended by the seeds*.
- **Rejection before walking**: `PW.PerformFirstPoint` (Stage 2), then the converged start point is discarded if `IsPointOnLine(StartPOn2S, existingWLine, Deflection)` (~L3993–4083: point inside the segment-tube of any already-built line) — dedup happens **before** marching, keyed `dminiPointLigne` vs `SeuildPointLigne = 15*Increment²` (~L397).

### Stage 1-alt — Boundary/interior seeding (Imp-Prm; the model for trim-aware seeding)
- **File/fn**: `IntStart_SearchOnBoundaries.gxx` — `Perform` (~L1135), `BoundedArc` (~L229–800), `PointProcess` (~L880), `TreatLC` (~L1020), `ComputeBoundsfromInfinite` (~L804); `IntStart_SearchInside.gxx` (interior seeds); consumed by `IntPatch_ImpPrmIntersection.cxx::Perform` (~L617–960) via `solrst`/`solins`.
- **Purpose**: find every crossing of a *restriction arc* (trim curve) with the implicit surface: the scalar function F(t) = quadric distance along the arc is sampled `NbEchant = max(Func.NbSamples(),100)` and rooted with `math_FunctionAllRoots(EpsX=1e-10, maxdist, maxdist)` where `maxdist = TolBoundary + TolTangency` (fallback `TolBoundary`, ~L349–372). Cheap interval **rejection** pre-pass with 6 samples of (F, F') (~L292–333). For canonic arc x real quadric an **exact** `IntCurveSurface_HInter` is used instead (~L374–472).
- **Tangent (graze) intervals**: consecutive roots whose midpoint also satisfies |F|<maxdist are collapsed to **one point of minimal |F|** scanned at pitch `aTol=min(1000*TolBoundary, 0.001)` (~L599–689) — a grazing arc yields a single clean vertex, not a root cluster. Whole-arc-solution (`Arcsol`) is detected and emitted as a **segment** (future `IntPatch_RLine`) — the coincidence/run concept.
- **Outputs**: `IntStart_SequenceOfPathPoint` (points with arc + parameter + domain-resolved vertex identity) and `IntStart_SequenceOfSegment`.
- **Departure directions**: `IntPatch_ImpPrmIntersection.cxx::ComputeTangency` (~L221–470) converts path points into `IntSurf_PathPoint` seeds carrying an *inward* marching direction (multiple directions when the crossing is tangential); `IntPatch_TheSearchInside` supplies interior seeds for closed loops that touch no boundary.

### Stage 2 — First point
- **File/fn**: `IntWalk_PWalking.cxx` — `PerformFirstPoint` (~L622–651); engine `IntWalk_TheInt2S` = `IntImp_Int2S<...>` with `math_FunctionSetRoot` on the 3-equation/4-unknown system, one parameter locked (`IntImp_ConstIsoparametric`).
- **Purpose**: converge the approximate 4-tuple seed to an exact intersection point; returns tangency status and the marching direction (`Direction()`, `DirectionOnS1/S2()` 2D tangents).
- **When**: once per candidate seed; a failure or `IsTangent()` at t=0 aborts that seed (`Perform` ~L812–826). `IsTangentExtCheck` (~L687–745) additionally rejects seeds sitting in a *tangent zone*: normals with `cos² > 0.9998` AND all 8 probe points (±step in each param) within `2*TolTang` of the other surface.

### Stage 3 — The march (predictor / corrector / step control)
- **File/fn**: `IntWalk_PWalking.cxx` — constructor (~L219–414), `ComputePasInit` (~L38–106), `Perform(ParDep, u1min..v2max)` main loop (~L749–1821), `TestDeflection` (~L3407–3916), `TestArret` (~L3918–4096), `RepartirOuDiviser` (~L3297–3397).
- **State init** (constructor): `pasMax = 0.2*Increment`; `pasuv[i] = pasMax*range_i` clamped ≤ 10; resolutions `Reso* = Resolution(Precision::Confusion())` rescaled by max |param| (NEWRESO, ~L261–311) then forced ≤ `1e-5*pasuv[i]`; `myStepMin[i] = 100*Reso_i` and ≥ `2*Resolution(tolconf)`; **periodic-domain widening**: for a periodic direction narrower than a period, both bounds are pushed apart by `min(KELARG=20 * pasuv[i], half of the gap to a full period)` (~L335–397) — marching is allowed past the seam so closure/seam crossings do not stall at the chart edge.
- **Predictor** (~L897–958): `f = max(|previousd{1,2}.{X,Y}| along the locked iso, 0.1)`; `dP_i = sens * pasuv[i] * previousd_i / f`; if the locked-iso component `|dP| < 1e-7` it is boosted by `5*IncKey` (IncKey grows every failed cycle, cap 5000) — escape from degenerate direction.
- **Corrector** (~L965–1018): `myIntersectionOn2S.Perform(Param, Rsnld, aBestIso)` — 4-param Newton with the chosen parameter frozen. Border snap first: each converged param within `aTol[i]=Epsilon(range_i)` of a bound is clamped exactly to it (~L981–1005). **Bad-point retry**: if the corrector lands back on the previous point (all |ΔP| < Reso, and status ≠ PasTropGrand) the locked iso is rotated `(aBestIso+1)%4`, up to 4 retries (~L1006–1018). Out-of-bounds → handled by TestArret framing path.
- **Step control** — `TestDeflection` returns one of `IntWalk_StatusDeflection` {`PasTropGrand`, `StepTooSmall`, `PointConfondu`, `ArretSurPointPrecedent`, `ArretSurPoint`, `OK`}:
  - tangent at current point → `ArretSurPoint` (stop-on-point; triggers Stage 4);
  - **inflexion**: `cos(T_prev,T_cur) < 0` → halve all 4 steps, `STATIC_PRECEDENT_INFLEXION += 3` (~L3452–3482);
  - **confused points**: 3D dist² < `Precision::SquareConfusion()` → raise `pasInit` to ≥ `5*Reso`, multiply steps 1.5×, plus OCC26717 local-resolution rescue: measure real chord `RefDist` of one `pasuv` offset and force `pasuv[choixIso] ≥ 2*pasuv*tolconf/RefDist` (~L3488–3548);
  - all four UV deltas < Reso → `ArretSurPointPrecedent` (no progress);
  - **2D angular gates** (OCC431): `Cosi² < CosRef*Duv` with `CosRef = cos(π/9)/tolCoeff`, `tolCoeff = exp(min(sqrt(ResoUV/Duv)*tolArea, 7))`, `tolArea = 100` (200 when Reso < PConfusion); and `Ang > (π/2)*tolCoeff` → halve steps, `PasTropGrand` (~L3579–3674);
  - **3D deflection estimate**: `FlecheCourante = sqrt(|T_prev − T_cur|² * dist²)/8`; `< fleche/2` → grow step by `Ratio = 0.5*fleche/FlecheCourante` (bounded by pasInit); `> fleche` → shrink by `fleche/FlecheCourante`, `PasTropGrand` (~L3684–3798);
  - **osculating-circle hysteresis** (exact): with `d²=dist²`, `sinB2 = 1 − 2/(1+0.25*d²/tolconf²)`; deflection ≥ tolconf → `PasTropGrand` (halve); ≤ tolconf/2 (and previous status wasn't PasTropGrand — anti-cycling, bug 0029682) → `StepTooSmall` (steps ×1.5) (~L3800–3903);
  - final clamp `pasuv[i] = max(myStepMin[i], min(Ratio*|Du_i|, pasuv[i], pasInit[i]))`.
- **Main-loop status handling** (~L1201–1315): `PasTropGrand` → restore saved Param, after 5 fruitless iterations shrink `pasInit` by 25%; `PointConfondu` → count; >5 → force `ArretSurPoint`; step-regrowth after 5 consecutive OK: `pasuv += (pasInit−pasuv)*0.25` (cap `0.1*pasuv`), and if already at pasInit, grow `pasMax` ×1.1 up to 0.1 (~L1118–1194).
- **Domain exit / framing** — `TestArret`: if any param (predicted or corrected) leaves `[Uvd−Reso, Uvf+Reso]`: clamp all four to bounds and pick the **best iso to lock** by the progress metric `Duv = (DPc*DPb+dv²)²/((DPc²+dv²)(DPb²+dv²))` (largest wins) (~L3944–4062); then the loop re-corrects *on the boundary* (block ~L1460–1802): acceptance box widened only by `aTol2D = 1e-11`; a **backtrack test** rejects the framed point if either 2D direction reverses (`dot < 0`, ~L1583–1604); if the previous point was already tangent, an anti-orbit test kills the march when both surface-angles to the last two points exceed `MaxAngle = 3π/4` (~L1638–1698), and a **singular-direction test** stops when the directional derivative of the section curve is ~0 in both surfaces (sphere-pole case, ~L1700–1749).
- **Closure**: 2D sign test in `TestArret` (`close2dS1 && close2dS2`, ~L4066–4089) plus a 3D catch in the main loop: once the walker has left the start (`bTestFirstPoint=false`), returning within `aSQDistMax = 1e-14` of `pf` closes the line and re-appends point 1 (~L1400–1411).
- **Failure → division/reversal** — `RepartirOuDiviser`: halve all steps; when all `pasuv*0.5 < Reso`: if not yet done, **reverse the line** (`line->Reverse()`, `sensCheminement=-1`, restore first-point tangents reversed, restart marching from the other end with steps re-estimated from the last chord) — every line is walked in BOTH directions before being declared finished; else `Arrive=true`. Guards: `LevelOfIterWithoutAppend > 20` aborts, `RejectIndexMAX = 250000` caps points, IncKey cap 5000.

### Stage 4 — Tangent-zone extension (stall continuation *through* a graze)
- **File/fn**: `IntWalk_PWalking.cxx` — `ExtendLineInCommonZone(theChoixIso, theDirectionFlag)` (~L1831–2385). Called from the main loop only when the march arrived tangent (`myIntersectionOn2S.IsTangent()`) and the previous point was NOT tangent (~L1779–1802) — i.e. exactly at graze entry.
- **Mechanism**: refuse if current point already within Reso of any chart bound (~L1861–1893). Otherwise keep stepping with the **same locked iso** (`theChoixIso`), collecting converged points into `aSeqOfNewPoint` (not yet the line); per-step handling mirrors the main loop but with gentler step recovery (`pasInit −= 10%` on PasTropGrand, `pasuv += 10%` toward pasInit on PointConfondu); stop when the corrector becomes non-tangent (**out of tangent zone**) or `TestArret` fires; guards `nbIterWithoutAppend > 20`, `nbEqualPoints > 20` (points equal within Reso), first-point coincidence cap `dIncKey == 5000`.
- **Acceptance** (~L2220–2376): extension is committed if (a) the last point reached a chart bound (any param within Reso), else (b) status is OK **and** the collected polyline passes a zig-zag filter: when the tangent-zone direction is > π/4 off the locked-iso direction, all successive sub-direction pairs must stay within π/4 (`piquota = M_PI*0.25`) — a hairpin means the "extension" is orbiting, so it is rejected. On success all collected points are appended and the main loop resumes (`Arrive=false`).

### Stage 5 — Completion of stalled ends to the boundary (the graze-stall answer)
- **File/fn**: `IntWalk_PWalking.cxx` — `PutToBoundary(theASurf1, theASurf2)` (~L2951–3155), `SeekPointOnBoundary` (~L2716–2947), `DistanceMinimizeByGradient` (~L2394–2522), `DistanceMinimizeByExtrema` (~L2532–2583), `HandleSingleSingularPoint` (~L2587–2712), helper `IsParallel` (~L118–186), `AdjustToDomain` (~L193–215).
- **Called from**: `IntPatch_PrmPrmIntersection.cxx` ~L2058 (LOfPnts variant) and ~L2767 (grid variant) — immediately after `PW.Perform`, before dedup/vertex work. **Key architectural fact: OCCT never asks the marcher to finish a stalled end; it snaps the end to the boundary by constrained minimization.**
- **PutToBoundary**: band `aTol = 1e-3 * min(1.0, all four param ranges)` (degenerate ranges bail out when `aTol ≤ 2*Precision::Confusion()`); `IsParallel` samples ≤ 23 line points; if the line's U (or V) swing on a surface is < aTol the line is boundary-parallel in that direction and the corresponding snap is **skipped** (prevents sliding a boundary-parallel curve sideways). For each end point and each of the 4 params: if `aTolMin < dist_to_bound < aTol` (strictly *not already on* the bound: `aTolMin = Precision::Confusion()`), overwrite that param with the exact bound value and call `SeekPointOnBoundary(..., isTheFirst)`.
- **SeekPointOnBoundary**: solution tolerance `a3DTol = max(PConfusion/UResolution(1.0), PConfusion/VResolution(1.0))` over both surfaces, `aTol = max(Precision::Confusion(), a3DTol)` (i.e. metric-aware). Loop ≤ 20: `DistanceMinimizeByGradient` (4-var gradient descent on |P1−P2|², per-var step 1e-6 growing ×1.2 on success, minimum add clamped to `max(Epsilon(param), 1/Precision::Infinite())`, ≤ 60 iterations, success when SqDist < 1e-14) then two `DistanceMinimizeByExtrema` (2-var Newton on point-to-surface extremum, ≤ 10 iterations, same 1e-14 target); after each, `AdjustToDomain` clamps params back into the box (PConfusion band) — converged-and-inside terminates. Then **`HandleSingleSingularPoint`**: for every param sitting on a bound (PConfusion), rerun the Int2S corrector with that iso locked at the bound and snap results to the bound when the 3D move is within `the3DTol` scaled by local D1 magnitude — this is what lands the point *exactly* on pole/seam/corner. Accept iff midpoint of (P1,P2) is within `aTol` of both. Insert as new first/last point, after a **hairpin cleanup**: repeatedly delete existing end points whose direction w.r.t. the new boundary point reverses (`dot ≤ 0`), and delete an end point coincident with the insert (Precision::SquareConfusion) (~L2829–2944). Test cases named in comments: `bugs modalg_5 bug24585_1`, `boolean bcut_complex G7`, `bugs moddata_2 bug469`.
- **SeekAdditionalPoints**: if the walked line has < `aMinNbPoints = 40` points, midpoints of consecutive pairs are converged by the same gradient/extrema pair (accept both half-distances < 1e-14) and inserted, repeated until 40 or no progress — guarantees the approximator has enough samples even for short grazes.

### Stage 6 — Line acceptance, dedup, transitions, paves
- **File/fn**: `IntPatch_PrmPrmIntersection.cxx` — main accept block (e.g. ~L2053–2165), `IsPointOnLine` (~L3993), `DublicateOfLinesProcessing` (~L278–312), `SeveralWlinesProcessing` (~L65–272), `AdjustOnPeriodic` (~L2329), `AddWLine` (~L4085).
- Reject unless `PW.NbPoints() > 2`. Reject if end point lies on an existing line (`IsPointOnLine`, tube-of-segments with Deflection radius) or both ends match an existing line's ends within `TolTangency`; on rejection `DublicateOfLinesProcessing` keeps whichever of the two coincident lines has **more points** (or greater polyline length on tie) — densest survivor wins.
- **Transitions**: `tgline.DotCross(norm2, norm1) > 0 → trans1=Out, trans2=In` else swapped, evaluated at `PW.TangentAtLine(indextg)` (the tangent stored at `myTangentIdx`) (~L2094–2115).
- **Paves**: `IntPatch_RstInt::PutVertexOnLine(wline, Surf1, D1, Surf2, OnFirst, TolTang)` for BOTH operands — inserts vertices where the walked line crosses each operand's restriction (trim) arcs; may reduce point count; a line left with < 2 points is dropped. If no vertex was created, synthetic vertices are placed at both ends (`ParameterOnLine` = point index) (~L2123–2150).
- **Cross-WLine vertex welding**: `SeveralWlinesProcessing` — for each vertex of the new WLine, find the nearest vertex among existing WLines within `UResolution/VResolution(distance) < maxStep` on both surfaces, and **replace the point** with the existing vertex's point (period-aware via `MakeNewPoint`), removing an adjacent point when the vertex spacing allows — network vertices become exactly shared 4-tuples across walked lines.

### Stage 7 — WLine purge / join / extend (post-processing)
- **File/fn**: `IntPatch_WLineTool.cxx`:
  - `ComputePurgedWLine` (~L1468–1601): (I) delete near-equal consecutive points in a look-ahead window of 5: equal if 3D `IsEqual(gp::Resolution)` or either UV pair differs by < `1e-16*max|param|`; abort purge entirely (return as-is) when any surface is C0 or `nb < aNbSingleBezier = 30`; (II) `DeleteOuterPoints` (~L222–353): trim only the *ends* against the face domains (`TopolTool::Classify(pnt, Precision::Confusion())`), skipped wholesale for periodic surfaces, first/last vertex geometry moved to the first kept point (`MovePoint` clamps into bounds); (III) `DeleteByTube` (~L434–617): a point is deleted when it stays inside the 2D tubes (radius `UResolution/VResolution(Precision::Approximation())²) on both surfaces AND the 3D tube (radius `Precision::Approximation()²`) around the previous chord, with parametrization-evenness guard `min/max step ratio ≥ 0.99²` and step-jump guard `aMaxSqrRatio = 15²` (non-planar only); backstops keep ≥ 2 interior points and re-seed 8 evenly spaced points when `15 < n < 30` (bad distribution before single-Bezier fit).
  - `JoinWLines` (~L1605–1838): (call site: cylinder x cylinder, `GeomGeomPerfom` ~L1847) merge WLines whose ends coincide within `Precision::Confusion()`; refuse across seam or natural bounds (`IsSeamOrBound` ~L636: any of the two end segments crossing a period/bound line, or midpoint exactly on one); refuse a kink: `CheckArgumentsToJoin` (~L1074) requires intersection-curve curvature radius > `1e-3*min(cyl radii)` or, if radius unknown, end-tangent angle ≤ `myMaxConcatAngle = π/6` AND perpendicular sag < `0.01*|chord|` (as `Cross² < 1e-4*|chord|⁴`). Also absorbs isolated tangency points (`theSPnt`) that duplicate WLine ends.
  - `ExtendTwoWLines` (~L1874–2153): for every WLine pair (only lines whose vertex 1 / last vertex sit exactly on point 1 / point N), compute for each end-pair FF/FL/LF/LL: `aVec1` = end tangent of WL1, `aVec2` = (reversed) end tangent of WL2, `aVec3` = gap vector. `CheckArgumentsToExtend` (~L918–1067): if gap ≤ tol3D → `Common` (ends already touch: still require angle(aVec1,aVec2) ≤ π/6); else require all three pairwise angles ≤ **π/6**; then build the candidate junction as the **corrector-converged midpoint** (`IsIntersectionPoint` ~L734: Int2S on 0.5*(P1+P2)); reject if out of both UV boxes (period-aware); if the segment to the new point would cross a period boundary → reject, if it merely lands on one → `Singular`. Action `ExtendFirst`/`ExtendLast` (~L810–889): the SAME converged point is appended to BOTH lines (each keeps its own UV rep; if the new point's params differ by more than the vertex tolerance the line is only *marked*, not extended); unless `Singular`, the two lines are then physically concatenated and re-vertexed. Extension is *blocked* near critical points (cone apex, sphere poles collected in `GeomGeomPerfom` ~L1874–1894) and when a mid-vertex sample of the WLine is out of the UV domain (`IsNeedSkipWL` ~L1842).

### Stage 8 — Seam / pole special points
- **File/fn**: `IntPatch_SpecialPoints.cxx` (`AddCrossUVIsoPoint`, `AddPointOnUorVIso`, `AddSingularPole`, `ContinueAfterSpecialPoint`, `AdjustPointAndVertex`); consumers: `IntPatch_ImpPrmIntersection.cxx::DecomposeResult` (~L47, body ~L2600+; splits every walked line of a quadric operand at seam crossings and poles, re-emitting exact on-seam points with both U=0 and U=2π representations) and `IntPatch_ALineToWLine.cxx` (`IsPoleOrSeam` ~L72, pole insertion during sampling).
- **Purpose**: walked lines never terminate *near* a seam/pole with an ambiguous chart: the crossing is solved exactly, the line is cut there, and marching state (`aPrePointExist` = SPntSeamU/SeamV/SeamUV/Pole) suppresses the chart jump on the next point.

### Stage 9 — Imp-Prm walking (implicit x parametric; same defect class, 2-var walker)
- **File/fn**: `IntPatch_ImpPrmIntersection.cxx::Perform` (~L617–1960): `solrst` (Stage 1-alt) → `ComputeTangency` → `solins` (interior) → `IntPatch_TheIWalking iwalk(TolTang, Fleche, aLocalPas)` with `GetLocalStep` (~L545: shrink step for tiny/huge parametric spans) → per line: transitions from `tgline.DotCross(norm2,norm1)`, quadric params recomputed and V clamped with `TolV=1e-14`, seam readjust `AdjustLine` (~L2226), `DecomposeResult`. Whole-arc solutions become `IntPatch_RLine` (restriction lines) — first-class *coincidence* results.
- `IntWalk_IWalking.gxx` (2-variable marching on the parametric operand): constants `CosRef3D = 0.98` (≈11.5°) 3D-turn gate, `CosRef2D = 0.88` (25°) 2D-turn gate, `MaxDivision = 60` step-halvings cap (~L36–39); `ComputeOpenLine` (~L1414) / `ComputeCloseLine` (~L2007): step `PasC = pas*min((UM−Um)/|d2dx|, (VM−Vm)/|d2dy|)` scaled to the box; `Cadrage` (~L413) frames the step onto the border exactly; `TestArretPassage` (~L593/777) detects passing near start / other departure points (multiplicity bookkeeping so every boundary seed is consumed exactly once); open lines from interior seeds are walked, reversed (`StepSign = -1`), and walked again — bidirectionality again.

### Stage 10 — Approximation to curves
- **File/fn**: `ApproxInt_Approx.gxx` — ctor (~L166: `myDegMin=4, myDegMax=8, myTol3d=myTol2d=0.001, myNbIterMax=5`, continuity C2, `RatioTol=1.5`, `aMinNbPointsForApprox=5` below which interpolation is used), `Perform` (~L184/226), `Parameters` (~L88: ChordLength/Centripetal over the *combined* 3D+2D squared distances — the 2D reps participate in parametrization), `buildKnots`/`buildCurve` with `ApproxInt_KnotTools` (curvature-driven knot placement; ≥3 knots forced when one span would exceed `myNbPntMax`); conditioning translations `ComputeTrsf3d/2d` (~L35/57) shift data to origin before fitting. `ApproxInt_PrmPrmSvSurfaces` / `ApproxInt_ImpPrmSvSurfaces` evaluate exact points/tangents during fitting (the approximator re-corrects onto the surfaces rather than fitting the raw polyline — tangent supplied by `IntImp_ComputeTangence`).
- **When**: BRepApprox/GeomInt run this on each final WLine; reached tolerances (`TolReached3d/2d`) are propagated to the resulting edge tolerance (growth model input).

---

## DATA STRUCTURES

- **`IntSurf_PntOn2S`** — {gp_Pnt pt; u1,v1,u2,v2}. THE invariant record: every walked sample is index-corresponded (3D + both UV reps) — identical to our `(p3, uvA, uvB)` triple. `IsSame(other, tol3D[, tol2D])` used everywhere for identity.
- **`IntSurf_LineOn2S`** (`IntSurf_LineOn2S.cxx`, 227 lines) — `NCollection_Sequence<IntSurf_PntOn2S>` + three lazy bounding boxes (3D `myBxyz`, 2D `myBuv1`, `myBuv2`), each auto-enlarged by 1% of max extent on build (`IsOutBox` ~L78) and invalidated by `RemovePoint`. Ops: `Add`, `InsertBefore` (appends when index past end), `RemovePoint`, `Split(Index)`, `Reverse`, `SetUV(Index,OnFirst,U,V)` (chart fix-up without touching 3D). The boxes power the cheap `IsPointOnLine` seed rejection.
- **`IntWalk_PWalking`** (hxx ~L242–288) — the marcher state that matters:
  - `pasuv[4]` current steps / `pasInit[4]` cruise steps / `pasSav[4]` saved / `myStepMin[4]` floors — all per-parameter, all mutated by TestDeflection; the four-way independence is what lets one surface's fine direction not throttle the other.
  - `Um1..VM2` — *widened* domain bounds (periodic KELARG margin), distinct from the clamp box passed to `Perform`.
  - `ResoU1..ResoV2` — magnitude-rescaled parametric resolutions; every "no progress" and "on boundary" test is against these, never absolute epsilons.
  - `previousPoint/previoustg/previousd/previousd1/previousd2` — corrector output of last accepted point (3D tangent + both 2D tangents) = predictor input.
  - `myTangentIdx`, `tgdir` — index+direction used later for transition computation (`TangentAtLine`); maintained under `RemoveAPoint`.
  - `sensCheminement` (+1/−1), `choixIsoSav`, `close/tgfirst/tglast`, `STATIC_BLOCAGE_SUR_PAS_TROP_GRAND`, `STATIC_PRECEDENT_INFLEXION` (hysteresis counters).
- **`IntPatch_WLine`** — LineOn2S + ordered vertices (`IntPatch_Point`) + `trans1/trans2` + creating-way tag + `EnablePurging` flag. Vertex `ParameterOnLine()` is a *point index* stored as double — all join/purge code rebuilds these indices when points are removed.
- **`IntPatch_Point`** (vertex/pave) — 3D value + tolerance + (u1,v1,u2,v2) + optional arc/parameter identity on each domain (`SetArc`, `SetVertex`) — the pave record binding a walked-line point to a trim arc crossing.
- **`IntSurf_PathPoint`** — boundary seed: point + uv + one or more departure directions + tangency flag + multiplicity (`IntStart` output, `ComputeTangency` enriched).
- **`IntWalk_StatusDeflection`** — {PasTropGrand, StepTooSmall, PointConfondu, ArretSurPointPrecedent, ArretSurPoint, OK}; the previous status is *carried into* the next TestDeflection call (anti-oscillation).
- **`IntImp_ConstIsoparametric`** — which of the 4 params the corrector freezes {UIsoCaro1, VIsoCaro1, UIsoCaro2, VIsoCaro2}; chosen by `TestArret`'s progress metric, rotated on bad-point retry.

---

## CONSTANTS & TOLERANCES

Precision basis: `Precision::Confusion()=1e-7`, `SquareConfusion()=1e-14`, `PConfusion()=1e-9` (=Confusion/100... actually Parametric(Confusion)), `Angular()=1e-12`, `Approximation()=1e-6`, `Intersection()=1e-9`.

| Constant | Value / formula | Where applied |
|---|---|---|
| `myFleche` default | 0.01 | Intersection::Perform ~L1080 |
| `myUVMaxStep` (Increment) | 0.001; **0.0001 near singular boundary** (dist∈(1e-7,1e-5)) | `DefineUVMaxStep` ~L2377–2402 |
| `pasMax` | `0.2 * Increment` | PWalking ctor ~L244 |
| initial `pasuv[i]` | `pasMax * range_i`, clamp ≤ 10 | ctor ~L313–316, L406–413 |
| `ComputePasInit` | `pasuv[i]=max(2*pasMax*max(boxΔ, 0.01*rangeΔ), pasuv[i])` | ~L43–90 |
| Reso rescale | `Reso *= max|bound|` if result < 10; then `Reso ≤ 1e-5*pasuv[i]` | ctor ~L261–333 |
| `myStepMin[i]` | `max(100*Reso_i, 2*Resolution(tolconf))` | ctor ~L399, ComputePasInit ~L97–100 |
| KELARG | 20 (periodic widening: `min(20*pasuv, half period slack)`) | ctor ~L242, 335–397 |
| predictor floor `f` | `max(|iso comp|, 0.1)` | ~L917 |
| tiny-dP boost | `|dP|<1e-7 → dP *= 5*IncKey`, IncKey cap 5000 | ~L932–952 |
| border snap in corrector | `aTol[i] = Epsilon(range_i)` | ~L873–876, 986–996 |
| bad-point detection | all ΔP < Reso → rotate iso, ≤ 4 tries | ~L1006–1018 |
| `aSQDistMax` (3D closure) | 1e-14 | ~L759, 1383/1404 |
| `RejectIndexMAX` | 250000 | ~L829 |
| `LevelOfIterWithoutAppend` cap | 20 (then halve or abort) | ~L885–895, 1084–1105 |
| `LevelOfEmptyIn...` reset | >10 → restore pasSav | ~L1062–1068 |
| `LevelOfPointConfondu` | >5 → ArretSurPoint | ~L1112–1116 |
| step regrowth | after 5 OK: `+=(pasInit−pasuv)*0.25` cap `0.1*pasuv`; pasMax ×1.1 while < 0.1 | ~L1120–1193 |
| TestDeflection: `CosRef2D` | `cos(π/9) ≈ 0.9397`, `AngRef2D = π/2`, `d = 7` | ~L3402–3404 |
| `tolArea` | 100 (200 if any Reso < PConfusion) | ~L3579–3584 |
| `tolCoeff` | `exp(min(sqrt(ResoUV²/Duv)*tolArea, 7))` | ~L3600–3615 |
| deflection estimate | `sqrt(|ΔT|²·dist²)/8` vs `fleche` (half/full hysteresis) | ~L3684 |
| osculating-circle gate | `sinB2 = 1−2/(1+0.25·d²/tolconf²)`; cmp `cos(T1,T2)` | ~L3862–3878 |
| confused-pt rescue | `pasInit ≥ 5*Reso`; local resol `pasuv ≥ 2*pasuv*tolconf/RefDist` (OCC26717) | ~L3488–3544 |
| framing accept band | `aTol2D = 1e-11` | ~L1577 |
| tangent-zone orbit gate | `MaxAngle = 3π/4` | ~L1651 |
| ExtendLineInCommonZone guards | 20 iter no-append, 20 equal-points, 5000 coincidence; zigzag `π/4` | ~L1899, 2308–2366 |
| PutToBoundary band | `aTol = 1e-3*min(1, ranges)`; skip if `≤ 2*Confusion`; snap iff `Confusion < δ < aTol` | ~L2967–2976, 2988+ |
| IsParallel | ≤ 23 samples; swing < aTol ⇒ parallel | ~L124 |
| SeekPointOnBoundary tol | `aTol = max(Confusion, PConfusion/U-,V-Resolution(1.0))` both surfaces | ~L2737–2742 |
| Gradient/Extrema targets | SqDist < **1e-14**; grad: 60 iters, step 1e-6 ×1.2, min-add `max(Epsilon(p), 1/Inf)`; extrema: 10 Newton iters | ~L2399–2583 |
| SeekAdditionalPoints | `aMinNbPoints = 40`, accept SqDist < 1e-14 | ~L2774 (call), 3163 |
| Seed dedup | `SeuildPointLigne = 15*Increment²` | PrmPrm ~L397/2001/2562 |
| line-pair rejection | ends within `TolTangency` | ~L2079, 2806 |
| Purge equal-point | window 5; `1e-16*max|param|` or 3D `gp::Resolution` | WLineTool ~L1505–1541 |
| Purge skip | C0 continuity or `nb < 30` (`aNbSingleBezier`); redistribution if `15 < n < 30` (`aMinNbBadDistr`) | ~L1583, 402–403, 602–613 |
| Tube radii | `U/VResolution(Precision::Approximation())²` (2D), `Approximation²` (3D); evenness `0.99²`; jump `15²` | ~L466–477 |
| Join gates | ends `Precision::Confusion`; kink: curvature radius > `1e-3*min(radius)` or angle ≤ `π/6` + sag `Cross² < 1e-4·chord⁴` | ~L1617, 1690, 1074–1118 |
| `myMaxConcatAngle` | **π/6** | WLineTool.cxx L27 |
| ExtendTwoWLines junction | corrector-converged midpoint; angles ≤ π/6 (3 pairwise) | ~L918–1067 |
| IntStart | `EpsX=1e-10`; `NbEchant ≥ 100`; `maxdist = TolBoundary+TolTangency`; regularity re-run `5e-4`; tangent-cluster pitch `min(1000·TolBoundary, 0.001)` | gxx ~L261–371, 496, 631 |
| IWalking | `CosRef3D=0.98`, `CosRef2D=0.88`, `MaxDivision=60` | gxx ~L36–39 |
| ALineToWLine | 200 pts/WLine; `aTol=2*myTol3D+Confusion`; `aPrmTol=max(1e-4·range, PConfusion)`; degenerate radius `5e-6`; step `sqrt(eps(2R+eps))/|T|`, bisection ≤ 50 | ~L417–418, 451, 513, 1042 |
| ApproxInt | deg 4–8, tol3d=tol2d=0.001, 5 iters, C2, `RatioTol=1.5`, interpolate below 5 pts | gxx ~L28–31, 166–180 |

---

## INVARIANTS

1. **Every emitted point is corrector-converged** (a true intersection point within TolTangency); the predictor result is never stored. Every point carries index-corresponded (3D, uv1, uv2).
2. **Ends are resolved**: an accepted WLine either (a) ends on a natural-domain boundary of at least one surface (after PutToBoundary, *exactly* on it), (b) is closed (`point[N] == point[1]`, appended), or (c) ends at a tangency, flagged `tgfirst/tglast`. Downstream (RstInt, approximator, BOP pave-filler) may assume no dangling interior end that isn't a tangency.
3. **Bidirectional coverage**: every line has been marched in both directions from its seed (RepartirOuDiviser reversal) before being finalized — a stall in one direction cannot lose the other half.
4. **Deflection bound**: consecutive-point sag ≤ fleche (tolconf) by the osculating-circle criterion; approximation can therefore trust chord-based parametrization.
5. **Monotone ends**: PutToBoundary/SeekPointOnBoundary never create a hairpin — offending end points are deleted before insertion; ExtendLineInCommonZone rejects zig-zag extensions.
6. **No duplicate lines**: a seed on an existing line is never walked; two walks with coincident ends keep only the denser one (DublicateOfLinesProcessing). Network vertices shared between WLines are welded to identical 4-tuples (SeveralWlinesProcessing).
7. **Vertices bracket the line**: after PutVertexOnLine + synthetic end vertices, vertex 1 is at point 1 and the last vertex at point N; vertex `ParameterOnLine` = integer point index (renumbered by every purge/join).
8. **Purge conservatism**: purging never runs on C0 surfaces or < 30 points, never deletes trim-crossing vertices (hash = −1), keeps ends (DeleteOuterPoints only clamps them into the domain), keeps ≥ 2 points.
9. **Seam/pole points are exact**: lines are decomposed at seam/pole with points emitted exactly on the seam (both chart representations), so no consumer ever interpolates across a period jump.
10. **All "no progress"/"on boundary" tests are resolution-relative** (`Reso*`, `U/VResolution(tol)`), never absolute parametric epsilons — invariance under reparametrization of huge/tiny charts.

---

## PITFALLS (explicitly handled in source)

- **PasTropGrand ↔ StepTooSmall limit cycle** (bug 0029682): TestDeflection refuses StepTooSmall right after PasTropGrand; the main loop nullifies `LevelOfIterWithoutAppend` only when the point count actually grew (~L1090–1105, 1300–1310).
- **Corrector re-converges to the same point** (graze!): detected via all-|ΔP|<Reso, answered by rotating the locked iso through all 4 choices (~L1006–1018), not by shrinking the step.
- **Predictor step vanishing along the locked iso**: `|dP|<1e-7 → ×5·IncKey` escalation (~L932).
- **Walking around one point in a border tangent zone**: 3-point angle test > 3π/4 aborts (~L1638–1698); `nbEqualPoints > 20` aborts extension.
- **Singular direction (sphere pole)**: directional derivative of the section curve ~0 on both surfaces → stop (comment cites sphere at (0, π/2) with dir {1,0}) (~L1700–1749).
- **Tangent-zone seeds**: `IsTangentExtCheck` — normals aligned (cos² > 0.9998) + 8 probes within 2·tol ⇒ refuse to walk at all (~L687–745). OCCT walks *across* grazes it meets, but never *starts* inside one.
- **Boundary-parallel lines**: PutToBoundary's `IsParallel` skips snapping in the direction the line runs parallel to — otherwise the whole end drifts along the border (~L2979–2982).
- **Near-coincident neighbor points break angle math** in SeekPointOnBoundary: points with "good" distances are selected before computing direction dots (comment block ~L2804–2827, tests bug24585_1, bcut_complex G7, bug469).
- **Deleting vs keeping the boundary point priority**: "intersection point on the surface boundary has highest priority over middle points" — middle points are sacrificed to keep insertion monotone (~L2818–2822).
- **DeleteOuterPoints skipped on periodic surfaces** (~L234–237): classification against the domain in the presence of periodicity is unreliable.
- **Purge would destroy C0 or sparse lines** (bug24731): skip conditions + forced re-distribution of 8 points for 15..30-point lines (~L1579–1588, 598–614).
- **JoinWLines across a seam** would create a 2D-discontinuous line: `IsSeamOrBound` rejects any candidate whose end segments cross or touch a period/bound iso (~L636–733).
- **ExtendTwoWLines at poles/apex**: critical-point proximity (Precision::Confusion in 3D) disables extension of that end (`GeomGeomPerfom` builds the critical list; ~L1970–2007).
- **Extension across a period boundary**: candidate junction whose parametric interval to either line crosses a period line → NotConnected; landing exactly on it → Singular (extend both, don't merge) (~L992–1058).
- **Closed-surface seeds on the seam**: PrmPrm LOfPnts variant duplicates every seed at both period bounds and dedups (~L1866–1959) — a chain that should start on the far chart copy isn't lost.
- **Duplicate walks of the same curve from different seeds**: end-pair + tube rejection with "denser wins" arbitration (~L2079–2090, 278–312).
- **First point == start of an existing line but line continues elsewhere**: IncKey/5000 spin guard before declaring closure (~L1383–1398).
- **Huge lines**: `RejectIndexMAX = 250000` hard cap.
- **ALine sampling near degenerate section radius < 5e-6**: 2D params are unreliable → point flagged degenerate, first/last such points get corrected later (`CorrectEndPoint` ~L254), middle ones dropped (~L513–524).
- **IntStart quasi-tangent root clusters**: merged to the single parameter of minimal |F| (pitch ≤ 1e-3), preventing pave spam on grazing trims (~L599–689).

---

## PORT MAP

Anchors: `session_cpp/src/brep_section.cpp` (`build_section_scaffold`, stage 1b PutToBoundary, paves, keep-verdict, valence-1 bridge), `session_cpp/src/brep.cpp` (`split_with`, `split_by_uv_curves`, combine/classification), tolerances `tol3 = diag*2e-3`, `tol3_rep`.

1. **Constrained-minimization boundary completion** — `IntWalk_PWalking::PutToBoundary` + `SeekPointOnBoundary` + `DistanceMinimizeByGradient`/`ByExtrema` + `HandleSingleSingularPoint` → **anchor**: `brep_section.cpp` stage 1b PutToBoundary (currently marching-based, CHART bounds only; interior stalls left) → **action: REPLACE (this is the graze-stall fix)**. Design: keep our marching extension as phase A; phase B for any chain end that is (i) within `1e-3*min(range)` of a chart bound in some param but not on it, or (ii) an interior stall flagged tangent: clamp the offending param to the bound and run alternating 4-var gradient descent (step 1e-6 ×1.2, SqDist target scaled from `PConfusion/Resolution(1.0)`, ≤60 iters) + per-surface 2-var Newton extremum + domain clamp, ≤20 rounds; then a locked-iso re-correction pass to land exactly on seam/pole; insert with hairpin cleanup (delete end points with reversed direction dot ≤ 0). Never march to the boundary — minimize to it.
2. **Tangent-zone continuation** — `IntWalk_PWalking::ExtendLineInCommonZone` → **anchor**: `build_section_scaffold` corrector (`correct7`) stall handling / SESSION_EXT_TRIM gate → **action: NEW BUILD**. Design: when correct7 reports rank-deficiency/tangency at a chain end whose previous point was transversal, continue stepping with ONE parameter locked (the pre-stall choix iso), buffering points; commit only if the buffer exits the tangent zone or reaches a chart bound AND passes a π/4 zig-zag filter; guards 20 no-append / 20 equal-points.
3. **Bad-point iso rotation in the corrector** — PWalking corrector retry loop (~L1006–1018) → **anchor**: `correct7` 4-param Gauss-Newton → **action: ADOPT**. Design: when the Newton returns to the previous point within resolution, re-run with a different frozen parameter, cycling `(iso+1)%4` up to 4 times, before halving the step — grazes usually stall because the *default* frozen parameter is the tangent direction.
4. **Bidirectional restart on stall** — `RepartirOuDiviser` (reverse line, `sens=-1`, resume from the other end with chord-estimated steps) → **anchor**: `build_section_scaffold` predictor loop → **action: ADOPT** (verify we always re-march the reverse direction *after* a mid-chain stall, not only from the seed). One reversal flag per chain (`DejaReparti`), reverse exactly once.
5. **Osculating-circle step control with hysteresis** — `TestDeflection` (sinB2 formula, fleche/2..fleche band, per-axis pasuv, 5-OK regrowth, inflexion halving, PointConfondu local-resolution rescue) → **anchor**: our fixed predictor step in `build_section_scaffold` → **action: REPLACE**. Design: carry per-axis steps `pasuv[4]` with floors `100*Reso`, previous-status hysteresis (no grow right after shrink), and the exact `1−2/(1+0.25·d²/tol²)` gate; expose `fleche = diag*1e-3`-ish and `Increment=0.001` analogs.
6. **Seed scheduling + pre-walk dedup** — PrmPrm incidence-based start choice + `IsPointOnLine` tube rejection + `DublicateOfLinesProcessing` densest-wins → **anchor**: our ONE-SSI-per-pair scaffold (single seed) → **action: ADOPT (partial)**. Design: when a pair's chain network is incomplete (valence-1 ends), re-seed from additional polyhedral-interference points (mid/1/last/3n4/n4 schedule) but reject any seed inside the tube of an existing chain (radius = deflection) before marching; if two chains share both ends keep the denser.
7. **Min-point densification** — `SeekAdditionalPoints(minNb=40)` → **anchor**: chain sampling before our cubic fit / STEP compression tiers → **action: ADOPT**. Design: converge midpoints by the same gradient+extrema pair until ≥ N points (N ~ 40 or arc-length-scaled) so short graze chains don't reach the fitter with 3 points.
8. **Purge (equal points + tube)** — `ComputePurgedWLine`/`DeleteByTube` → **anchor**: our micro-piece filter + zero-span collapse in split/combine → **action: ADOPT (tube criterion)**. Design: pre-fit thinning with the 3-tube test (both 2D tubes at `Resolution(1e-6)` + 3D tube at 1e-6, evenness ratio ≥ 0.98, jump ratio ≤ 225) — never remove pave-bound points, never below 30 points, never on C0.
9. **End-join / extend with corrector-converged junction** — `JoinWLines` + `ExtendTwoWLines` + `CheckArgumentsToExtend` → **anchor**: valence-1 bridge (CASE A same-pair re-march / CASE B cross-pair junction weld) → **action: REPLACE the acceptance test**. Design: gate CASE A/B on the OCCT triple: angle(tangent1, tangent2) ≤ π/6 AND both ≤ π/6 to the gap vector; junction = Gauss-Newton–converged midpoint (our correct7 on 0.5*(P1+P2)), rejected if it leaves either operand's UV box or its parametric join interval crosses a seam; "Singular" outcome = extend both chains to the same junction point but do NOT merge (period-boundary landing).
10. **Vertex welding across chains** — `SeveralWlinesProcessing` (resolution-scaled, period-aware `MakeNewPoint`) → **anchor**: `build_section_scaffold` "vertices 3D-welded across pairs" → **action: ALREADY-EQUIVALENT (upgrade)**: adopt the *resolution-of-distance < max-step* acceptance (per-surface `U/VResolution(dist)` vs the pair's max step) instead of a flat 3D tol, and rewrite the welded point into the chain (not only the vertex table).
11. **Trim-arc seeding & graze-cluster collapse** — `IntStart_SearchOnBoundaries::BoundedArc` (FunctionAllRoots, `maxdist=TolBoundary+TolTangency`, tangent clusters → single min-|F| point, whole-arc → segment) → **anchor**: our paves (trim-loop crossings both operands) in `brep_section.cpp` → **action: ADOPT (two pieces)**: (a) when several section-x-trim paves cluster within the 1e-3-pitch graze window, collapse to the single minimal-distance pave; (b) whole-trim-arc-on-surface detection feeding our whole-segment runs (`seg_id` keys in `split_with`) — this is OCCT's RLine, our SEGWHOLE, and the missing same-domain/coincidence hook.
12. **Seam/pole decomposition with dual-chart points** — `IntPatch_SpecialPoints` + `DecomposeResult` + ALineToWLine `IsPoleOrSeam` → **anchor**: our seam-decomposition groundwork (chains split at periodic-seam jumps, corrector-pinned crossings) → **action: ADOPT (finish)**: emit the crossing point in BOTH chart representations (U=0 and U=period copies) and cut the chain there; carry a `PrePoint` state so the next corrector call is chart-pinned (no snap-back).
13. **Periodic chart widening (KELARG)** — PWalking ctor widens periodic bounds by `min(20*pasuv, half period slack)` → **anchor**: our CHART bounds used by stage 1b and the UV arrangement in `split_by_uv_curves` → **action: NEW BUILD (march-only)**: march in a widened window for periodic directions, then wrap points back at emission — kills the class of stalls exactly AT the seam.
14. **Singular-aware global step** — `DefineUVMaxStep` (0.001 → 0.0001 when a degenerate boundary of one operand nearly touches the other) → **anchor**: scaffold march init per pair → **action: ADOPT** (cheap pre-check per pair: probe degenerate iso boundaries, if D1-norm ≈ 0 and distance to other surface ∈ (1e-7, 1e-5), reduce base step ×10).
15. **Tangent-zone seed refusal** — `IsTangentExtCheck` → **anchor**: scaffold eligibility / same-domain missing subsystem → **action: ADOPT as classifier input**: normal-alignment (cos² > 0.9998) + 8-probe proximity ⇒ route the pair to a coincidence/overlap handler (to be built) instead of marching a graze into micro-fragments.
16. **Transition tags at creation** — `tgline.DotCross(norm2, norm1)` at the stored tangent index → **anchor**: combine classification (winding + radial pre-pass + connexity flood) → **action: ADOPT (upstream)**: stamp In/Out transition per section chain at march time (cheap, exact at a transversal point) and feed it to the flood as a prior, reducing dependence on downstream sampling verdicts.
17. **Reached-tolerance propagation** — ApproxInt `TolReached3d/2d` → edge tolerance → **anchor**: MISSING per-entity tolerances (`tol3_rep` only) → **action: NEW BUILD (minimal)**: record max fit deviation + max corrector residual per section chain and store on the resulting edges; combine's NK-RESCUE band becomes per-edge instead of global `0.15*tol3`.
18. **Approximation parametrization/knots** — `ApproxInt_Approx::Parameters` (chord-length over combined 3D+2D metric), `ApproxInt_KnotTools` curvature knots, deg 4–8 C2, conditioning translation → **anchor**: our two-tier cubic compression in the STEP writer + exact-subrange lifting in `split_with` → **action: ADOPT (knot seeding)**: seed the cubic-fit knot ladder from discrete-curvature extrema of the chain instead of uniform ncv ladder; parametrize with the combined 3D+2D chord metric so surface-uv reps stay in sync with the 3D curve.
19. **Densest-wins duplicate arbitration** — `DublicateOfLinesProcessing` → **anchor**: cross-pair segment unification (SEG-UNIFY) → **action: ALREADY-EQUIVALENT (keep; add the same-point-count → longer-polyline tiebreak)**.
20. **2-var walker turn gates** — IWalking `CosRef3D=0.98`, `CosRef2D=0.88`, `MaxDivision=60` → **anchor**: any future analytic-x-freeform marching path (cone x cone gap noted in memory) → **action: ADOPT constants** if/when we add an implicit-operand walker; they are field-tuned values ("rule by tests in U4").
