# audit_occt_blowup-guards — every bound OCCT puts on intersection/splitting blowup

Source of truth: `/home/petras/code/code_cpp/OCCT` @ `37dd5686f2` (8.0.1.dev). All paths below are relative to that root.
Feeds session A task #9 (SYMEMIT 4.7–5.5 GB balloon).

File aliases used in citations:
```
PF    src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller.cxx
PF3   .../BOPAlgo_PaveFiller_3.cxx      PF5  .../BOPAlgo_PaveFiller_5.cxx
PF6   .../BOPAlgo_PaveFiller_6.cxx      PF10 .../BOPAlgo_PaveFiller_10.cxx
PF11  .../BOPAlgo_PaveFiller_11.cxx     ALR  .../BOPAlgo_Alerts.hxx
OPT   .../BOPAlgo_Options.cxx           CSI  .../BOPAlgo_CheckerSI.cxx
TLS   src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Tools.cxx
FF    src/ModelingAlgorithms/TKBO/IntTools/IntTools_FaceFace.cxx
ITT   src/ModelingAlgorithms/TKBO/IntTools/IntTools_Tools.cxx
IPI   src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Intersection.cxx
IPP   .../IntPatch/IntPatch_PrmPrmIntersection.cxx
III   .../IntPatch/IntPatch_ImpImpIntersection.cxx
A2W   .../IntPatch/IntPatch_ALineToWLine.cxx
POLY  .../IntPatch/IntPatch_Polyhedron.cxx
HIT   .../IntPatch/IntPatch_HInterTool.cxx
WLT   .../IntPatch/IntPatch_WLineTool.cxx
PW    src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_PWalking.cxx
MRP   src/FoundationClasses/TKernel/Message/Message_Report.cxx
```

## 1. VERDICT

No spec was supplied (source-only hunt), so there is nothing to falsify; this file is the source-derived
baseline. Three findings contradict what the existing kb implies or omits, and matter for #9:

- **There is no memory guard anywhere in the FF pass.** Every bound is a *count/iteration/step* bound in the
  walkers; BOPAlgo itself has zero caps on number of section curves, pave blocks, or concurrently live
  intersectors. The only BOPAlgo-level abort paths are `UserBreak` (cooperative), `Standard_Failure` catch, and
  `HasErrors()` gates. Peak RSS in OCCT is bounded *indirectly*, by (a) a hard 250 k-point ceiling per walking
  line, (b) a hard 30×30 seed grid, (c) a 30-segment approximation cap.
- **OCCT holds every face-pair result simultaneously.** `BOPTools_Parallel::Perform(myRunParallel, aVFaceFace)`
  (PF6:529) dispatches *all* pairs with `OSD_Parallel::For(0, Length())` (BOPTools_Parallel.hxx:157-161) — no
  chunking, no streaming. Each `BOPAlgo_FaceFace` keeps its `IntPatch_Intersection` (`slin`/`spnt`) **and** its
  `mySeqOfCurve` alive until the treatment loop finishes at PF6:621. This is the structural analogue of a
  SYMEMIT balloon and is a *deliberate* OCCT design (results are consumed in a second serial pass), not an
  oversight — porting it 1:1 reproduces the balloon.
- **`kb/occt_ssi-walking.md:208` ("RejectIndexMAX = 250000 hard cap") is right but incomplete**: `RejectIndex`
  is initialised once (PW:835) and **never reset** — not by `RepartirOuDiviser`, not on reversal — so it is a
  *global per-`Perform` budget across both walk directions*, and it is bypassed entirely by the
  tangent-zone path (PW:2381, see trap T6).

## 2. PORT-CRITICAL DETAILS (guard, trigger, constant, file:line)

### 2.1 BOPAlgo FF pass — abort / re-entrancy / fence

| guard | trigger | constant | site |
|---|---|---|---|
| empty-work early out | `myIterator->ExpectedLength()==0` for FACE/FACE | — | PF6:315-319 (after `UpdateFaceInfoOn/In`, so face info is still refreshed) |
| plane/plane rejection | both surfaces `GeomAbs_Plane` and `CheckPlanes` finds ≤1 shared vertex | `iCnt > 1` | PF6:380-390, PF6:3639-3675 → `aFF.Init(0,0)`, pair never intersected |
| per-pair exception firewall | `Standard_Failure` inside one pair | — | PF6:209-240 `OCC_CATCH_SIGNALS` + `AddError(BOPAlgo_AlertIntersectionFailed)` **on the per-pair object**, not on the filler |
| failed-pair degradation | `!IsDone() \|\| HasErrors()` on the pair | — | PF6:545-552 → empty `Init(0,0)` + `AlertIntersectionOfPairOfShapesFailed` warning; the algorithm **continues** |
| cooperative abort | `Message_ProgressScope::UserBreak()` | — | PF6:205, 367, 530, 539, 593; PF6 `MakeBlocks` 727; OPT:110-118 adds `BOPAlgo_AlertUserBreak` as **error** |
| curve sanity filter | `Bnd_Box::IsThin(3*Precision::Confusion())` on the section curve | `3e-7` | PF6:599 → ITT `CheckCurve` (2 vertices at Confusion + Confusion gap) — degenerate curves dropped before any pave block exists |
| zero-span pave block | `fabs(aT1-aT2) < Precision::PConfusion()` | `1e-9` | PF6:906 |
| micro pave block | `BRepLib::FindValidRange` fails for the block | tol = `max(tolR3D, tolV_i)` | PF6:937-960 → block *not* kept, pushed to `aMicroPB` for vertex fusion |
| micro section edge | `BOPTools_AlgoTools::IsMicroEdge(E, ctx, false)` (shrunk range undone) | — | PF6:4348 in `RemoveMicroSectionEdges` |
| micro split edge | `nV1==nV2` and no shrunk data | `aLPB.Extent() < 2` skip | PF6:4401-4432 `RemoveMicroEdges` |
| far-from-origin conditioning | `\|P−boxCentre\| ≥ 1e5` **and** `boxDiag/dist ≤ 1e-5` | `theCriteria = 1.e+5` | PF6:215 → TLS `TrsfToPoint` (BOPAlgo_Tools.hxx:233-237); faces are *moved to the origin*, intersected, results transformed back (PF6:244-265) |
| re-entrancy of the FF recheck loop | `aNbC>0 && i < aNbFFPrev` and no pave block produced | ≤ 2·aNbFFPrev iterations | PF6:720-724, 733, 879, 894-897, 1067-1071 |
| nested-filler flag | `PostTreatFF` builds a **second** `BOPAlgo_PaveFiller` | `SetIsPrimary(false)` | PF6:1195-1197 |
| `!myIsPrimary` disables | `ForceInterfEF(range)` | whole pass skipped | PF5:775 |
| `!myIsPrimary` disables | `SetNonDestructive()` auto-detect | keeps caller's flag | PF10:43 |
| **not** disabled by `!myIsPrimary` | `ForceInterfEE` | — | PF3:997 (asymmetry is real, verified) |
| nested-filler termination | nested arguments are edges/vertices only → FF iterator empty → `PerformFF` returns at PF6:315, `MakeBlocks` returns at PF6:660-663 | — | data-driven, **no depth counter** |
| post-treat failure | nested filler `HasErrors()` | — | PF6:1393-1397 → `AlertPostTreatFF` (error) → `MakeBlocks` returns at PF6:1121-1124 |
| single-shot fast path | `aNbS==1 && micro==0 && rejected==0 && VertsUnused.IsEmpty()` | — | PF6:1237-1276: no nested filler at all |
| EF memoisation fence | pair (face, pave block) already intersected | `myFPBDone` | PF5:300-305 (fill), PF5:1090-1092 (skip in forced pass) |
| EF distance cache | `MinimalDistance()` when no common part found | `myDistances[(nE,nF)]` list of (t1,t2,dist) | PF5:352-360, consumed PF6:3122-3146 to avoid re-projection |
| tolerance-extension veto | vertex sits on a real E/E or E/F intersection point | `myVertsToAvoidExtension` | PF3:449, PF5:498, honoured PF6:2977 |
| repeat-intersection single shot | only if some sub-shape tolerance grew | `myIncreasedSS` non-empty | PF:359-424 — VV/VE/VF only, **never loops**; `myIterator->IntersectExt(map)` restricts the re-run to affected shapes |
| stage gating | after every stage | `if (HasErrors()) return;` | PF:239-354 (11 gates) |

Fuzzy value is floored, never zeroed: `myFuzzyValue = max(theFuzz, Precision::Confusion())` (OPT:105-108),
default `Precision::Confusion()` (OPT:53), `myUseOBB=false` (OPT:54).

### 2.2 Tolerance floors that decide how much geometry gets created

| quantity | value | site |
|---|---|---|
| `BOPAlgo_FaceFace::myTolFF` default | `1.e-7` | PF6:148 |
| `ToleranceFF` | `max(tolF1,tolF2)`, **floored to `5.e-6` if either face is non-analytic** | PF6:3922-3941 |
| shift compensation | `aTolFF = max(aShiftValue, ToleranceFF(...))` where `aShiftValue` = seam-edge E/E mismatch distance | PF6:470-478, 495 |
| approximation tolerance handed to IntTools | `anApproxTol = 1.e-7`, `bSplitCurve = false` | PF6:329-331, 506 |
| section-curve box padding | `aTolFF + max(MaxTolerance(F1,VERTEX), MaxTolerance(F2,VERTEX))` | PF6:580-588, `aBox.Enlarge` PF6:605 |
| curve tolerance stored | `max(aIC.Tolerance(), aTolFF)` | PF6:607 |
| `BOPTools_AlgoTools::DTolerance()` | `1.e-12` (added whenever a vertex tolerance is grown to reach a curve) | BOPTools_AlgoTools.hxx:70; used PF6:2999, 3052-3054, 3573 |
| existing-edge reuse band | `aTolCheck = tolR3D + fuzzy`; thin-face probe capped `aMaxTolAdd = min(0.001, 10*aTolCheck)`; probe `2*min(aMaxTolAdd, max(aRealTol, tolV))`; tangent gate `\|cos\| ≥ 0.9063` (≈25°); common-block bonus `×2`; accepted distance scaled `aCoeff=2` | PF6:2106-2112, 2149-2191, 2245 |
| pave filtering on curves | drop a pave if `d² > 100·max(tol², d²_min)` **and** `sin(angle) < 0.5` | PF6:2485, 2515-2519; kept-vertex tolerance re-raised to `sqrt(maxKept)+Confusion` PF6:2533 |
| "stick" pave criteria | rich `d² ≤ 2e-7`; crease `1−\|n1·n2\| ≤ 5e-9` | PF6:2793-2794 |
| tolerance-reduction skip | `aTolV − aMaxTol < 0.001·aTolV` | PF6:4225 |

### 2.3 IntPatch dispatcher — clamps on the walker's inputs (the real memory dial)

`IntPatch_Intersection::SetTolerances` (IPI:134-172) clamps **all four** walker inputs:
`TolArc, TolTang ∈ [1e-8, 0.5]`; `Fleche ∈ [1e-3, 10]`; `UVMaxStep ≤ 0.5` — **the UVMaxStep floor is commented
out** at IPI:163 (`// if(myUVMaxStep<1.0e-3) myUVMaxStep=1e-3;`). Defaults when unset: `Fleche = 0.01`,
`UVMaxStep = 0.01` (IPI:184-191, 1078-1085, 1389-1396 — three separate copies of the same default block).

`IntTools_FaceFace` supplies them (FF:483-489): `TolArc = TolTang = tolF1+tolF2+fuzzy` (FF:381-387),
`Deflection = 0.1`, **`/10` → `0.01` when both surfaces are BSpline**, and
`UVMaxStep = IntPatch_Intersection::DefineUVMaxStep(...)` = `0.001`, dropping to **`0.0001` when a singular
point of one surface lies within `(Confusion, 1e-5)` of the other** (IPI:2372-2406, singularity probe
`aNbBndPnts = 5` at IPI:2298). `UVMaxStep` becomes `Increment` in `IntWalk_PWalking` ⇒ `pasMax = 0.2·Increment`
(PW:244/442) ⇒ near-singular pairs walk with steps 10× smaller and produce ~10× more points.

Infinite-domain trimming multipliers: `tp = 1000·TP` (IPI:401), `FUN_TrimBothSurf(..., 1.e+8, ...)` in
Prm/Prm (IPI:1747) vs `1.e+5` in Geom/Prm (IPI:1961).

### 2.4 Seeding discretisation — the only *hard* grid cap in the stack

| guard | constant | site |
|---|---|---|
| polyhedron grid per direction | `NBMAXUV 30` → grid ≤ 31×31 = 961 points, ≤ 1922 triangles, arrays `(nbU+1)(nbV+1)+1` of `gp_Pnt`+2 doubles | POLY:33, 43, 52, 82 |
| deflection over-estimation | `tol = ComputeMaxDeflection(...) * DEFLECTION_COEFF 1.1` | POLY:32, 128-131 |
| degenerate triangle edge | `LONGUEUR_MINI_EDGE_TRIANGLE 1e-14` | POLY:31 |
| sample counts before the cap | Plane 2; Cylinder/Cone/Sphere/Rev/Extrusion U=10 V=15; Torus U=20 V=15; Bezier `3+NbPoles`; **BSpline `NbKnots*Degree*(2 if non-rational)`, floor 4** | HIT:37-112 |
| infinite-parameter clamp | `±1e5` (both infinite) / `±2e5` span (one infinite) | HIT:133-159 |
| arc sampling | Line 2, conic 10, Bezier `NbPoles`, BSpline `2+NbKnots*Degree`, default 10 | HIT:232-261 |

A degree-3 BSpline with 200 knots asks for 1200 samples and gets **30** — the cap is what keeps
`IntPatch_Polyhedron` from being O(knots²) memory. It also means OCCT *under-seeds* dense NURBS and relies on
the marcher to find the rest.

### 2.5 Prm/Prm seeding loop — bounded restart, no unbounded retry

- Seed dedup before marching: `SeuildPointLigne = 15.0·Increment²` (IPP:397, 957); a converged first point
  within that of an existing line is discarded (`IsPointOnLine`, IPP:3993-4083).
- Restart ladder per polyhedral section line, `TabPtDep[]` fence marks used seed indices (IPP:441-446, 575-577);
  loop condition `while (nbp > 5 && ((NbStarts < 3 || !lignetrouvee) && ((NbStarts-3 < nbp) || lignetrouvee)))`
  (IPP:721-723) ⇒ **at most `nbp+3` seeds per section line**, and it exits as soon as one line is found and 3
  seeds have been tried.
- Duplicate-line arbitration keeps the *longer* line: `DublicateOfLinesProcessing` compares `NbPnts`, then total
  chord length (IPP:278-312); `AddWLine` removes any previously stored line whose midpoint **and all vertices**
  lie on the new one (IPP:4085-4141).
- Densification floor after walking: `aMinNbPoints = 40` — `PW.SeekAdditionalPoints` is called only when
  `NbPoints() < 40` (IPP:2774-2777, 3067-3070), and the bisection stops when the point count stops changing
  (PW:3183-3185 `aNbPoints != aNbPointsPrev`).

### 2.6 IntWalk_PWalking — the counters that actually terminate a runaway march

| counter / guard | fires when | constant | site |
|---|---|---|---|
| **`RejectIndexMAX`** | total appended regular points | **250000** → `Arrive = true` | PW:829, 1416, 1516, 1756 (never reset — global per `Perform`) |
| `LevelOfIterWithoutAppend` | iterations that appended nothing | `> 20` → `RepartirOuDiviser` once, then break | PW:886-895 |
| deflection test disabled | `LevelOfIterWithoutAppend ≥ 10` | steps halved instead of tested (bug #0029682 guard: only when `aStatus != IntWalk_StepTooSmall`) | PW:1084-1105 |
| `LevelOfEmptyInmyIntersectionOn2S` | consecutive corrector failures | `> 10` → restore `pasSav` | PW:1060-1068 |
| `LevelOfPointConfondu` | consecutive coincident points | `> 5` → `IntWalk_ArretSurPoint` | PW:1112-1116, 1241-1244 |
| `IncKey` / `dIncKey` | new point keeps landing on the first point | `== 5000` → `return` (abandon line) | PW:1386, 1498, 1620; ExtendLine 2094, 2192 |
| `NbPasOKConseq` | consecutive good steps | `≥ 5` → try to grow the step back toward `pasInit` (`t = (pasInit−pasuv)*0.25`, ≥ `0.1*pasInit`) | PW:1120-1135 |
| `STATIC_BLOCAGE_SUR_PAS_TROP_GRAND` | step-growth requests | `> 5` → force `PasTropGrand` | PW:3767 |
| `STATIC_PRECEDENT_INFLEXION` | tangent reversal (`cos < 0`) | `+= 3`, halve steps; if all `pasuv < Reso` → `ArretSurPointPrecedent` | PW:3453-3477 |
| absolute step clamp | after `ComputePasInit` | `pasuv[i] > 10 → 10` | PW:408, 787 |
| step floor | `myStepMin[i] = max(myStepMin[i], 2·Resolution(tolconf))`, then `pasuv = max(myStepMin, pasuv)` | — | PW:97-104 |
| box-estimate floor | UV box reduction limited to 1 % of natural range | `aRangePart = 0.01`, `Increment = 2·pasMax` | PW:43-44, 56-89 |
| domain enlargement | walking box padded | `KELARG = 20` × `pasuv` | PW:242, 440, 517-577 |
| closure detection | `pf.SquareDistance(pl) < 1e-14` after having left the start | `aSQDistMax = 1.0e-14` | PW:759, 1400-1412 |
| `ExtendLineInCommonZone` | tangent-zone continuation | `nbIterWithoutAppend > 20 \|\| nbEqualPoints > 20` | PW:1853-1854, 1897-1900 |
| Newton/gradient minimisers | inner solves | `aNbIterMAX = 60` (gradient PW:2399), `10` (extrema PW:2546), `20` (singular point PW:2752), `5` (boundary PW:3238) | — |
| parallelism test sampling | line sampled for U/V-parallel check | `aNbPointsMAX = 23` | PW:124 |

`TestDeflection` (PW:3407+) is the step controller: `tolArea = 100.0` (×2 if any resolution < PConfusion,
PW:3579-3584); angular refs `CosRef2D = cos(π/9)`, `AngRef2D = π/2`, damping `d = 7.0`,
`tolCoeff = exp(min(sqrt(Reso²/Duv²)·tolArea, 7))` (PW:3402-3404, 3596-3617); arc-vs-chord deflection compared
against `tolconf` and `tolconf/2` through `aSinB2Max = 1 − 2/(1+0.25·d²/tolconf²)` (PW:3862-3877).
Coincident-point recovery **raises** `pasInit` to `5·Reso` (PW:3492-3495) — the only place a step floor is
enlarged mid-walk.

### 2.7 Analytic branches

- **ALine → WLine**: target `aNbPointsInALine = 200` per WLine (IPI:1807, `IntPatch_ALineToWLine` ctor
  A2W:147-151); step `aStep = (LPar−Par)/(N−1)`, break if `< Epsilon(LPar)` (A2W:451-455); near a vertex the
  step is cut to `max((aLPar−aParameter)/5, 1e-5)` (A2W:476-479); bracket `[0.1·aStep, 10·aStep]`
  (A2W:486, 869-870); `StepComputing` bisection `aNbIterMax = 50` and returns false past it
  (A2W:1018, 1050-1076); deflection classification `CheckDeflection` returns −1 when `2·dist < maxDefl`
  (A2W:972-994); vertex proximity `aPrmTol = max(1e-4·(LPar−FPar), PConfusion)`, merge band `2·myTol3D +
  Confusion` (A2W:417-418).
- **Cyl/Cyl (ImpImp)**: `aNbMaxPoints = 1000`, `aNbMinPoints = 200`, tightened to `200/50` when the intersection
  is "good"; candidate count also capped by `20·R1`; final `aNbPoints = min(max(aNbMinPoints, aNbPts),
  aNbMaxPoints)`; `aStepMin = max(aTol2D, PConfusion)`, `aStepMax = range/aNbPoints`
  (III:6675-6694). Exactly `aNbWLines = 2` branches are tracked (III:6766) and at most
  `aNbCritPointsMax = 12` critical U-values (III:6724).
- **ImpImp densification**: `SeekAdditionalPoints(..., theMinNbPoints)` inserts midpoints only while
  `|U1l−U1f| > aMinDeltaParam` with `aMinDeltaParam = max(|u2−u1|/theMinNbPoints, theTol2D)` and stops on the
  fixpoint `aNbPoints == aNbPointsPrev` (III:6034-6156); callers pass `3` (III:7579, 7610) — i.e. this path is
  a repair, not a densifier.
- **Purging** (`IntPatch_WLineTool::ComputePurgedWLine`, WLT:1478+, called IPI:1361-1370, 2088-2096): only when
  `aWL->IsPurgingAllowed()`; Geom/Geom explicitly **disables** purging on walking lines
  (`EnablePurging(false)`, IPI:1828-1831). `DeleteByTube` keeps ≥2 points (WLT:443-447, 587-595) and, when the
  survivor count lands in `(15, 30)` (`aMinNbBadDistr`, `aNbSingleBezier`, WLT:402-403), force-keeps 8 evenly
  spaced points so the result is not converted into one bad Bezier (WLT:601-614).

### 2.8 Approximation (where the point cloud becomes a BSpline)

- `theapp3d.SetParameters(myTolApprox, tol2d, degMin, degMax, nbIter, **30**, withSurfaces, parType)` — the 6th
  argument is **max segments = 30** in every call (FF:1332, 1339, 1371, 1381, 1347).
- `ApproxParameters`: `iDegMin=4, iDegMax=8, iNbIter=0`; `iDegMax=6` for cylinder/torus with |Rc−Rt| < Confusion;
  `iNbIter=1` only for cylinder/cylinder (FF `ApproxParameters` body).
- `Tolerances`: cylinder/torus with equal radii ⇒ `TolTang *= 0.1` (FF `Tolerances` body).
- Approximation retry is **single-shot**: `goto reapprox` is guarded by `!rejectSurface && !reApprox`
  (FF:1467-1473, 1493-1499, 1553-1559, 1579-1585, 1727-1730); label at FF:717. `myTolApprox` is dropped to
  `aTolApproxImp = 1.e-5` on retry (FF:1256, 1727).
- If the approximator fails, OCCT does **not** fail the pair — it emits the raw polyline as a BSpline through
  `GeomInt_IntSS::MakeBSpline(WL, ifprm, ilprm)` (FF:1388-1407), i.e. a 250 k-point WLine becomes a
  250 k-pole curve.

### 2.9 Self-interference guards (all warnings, never fatal)

| check | condition | alert | site |
|---|---|---|---|
| V/E same rank | `Rank(nV) == Rank(nE) ≥ 0` | `AlertSelfInterferingShape` (warning) | PF3:897-907 |
| V/F same rank | `Rank(nV) == Rank(nF) ≥ 0` | same | PF5:668-678 |
| common block spanning ≥2 edges of one argument | `aLE.Extent() > 1` | `AlertAcquiredSelfIntersection` | PF11:131-146 |
| one vertex/edge shared by ≥2 faces of one argument | `aMCS.Extent() > 1` | same | PF11:200-218 |
| whole check skipped | single-argument (self-check) mode | — | PF11:30-34 |
| checker level | `myLevelOfCheck = NbInterfTypes()-1`, settable in `[0, NbInterfTypes)` | — | CSI:105, 113-124 |
| face self-intersection | only if `myLevelOfCheck ≥ 5`; skips Plane/Cylinder/Cone/Sphere and any torus with `R_major > R_minor + Confusion` | — | CSI:415-456 |
| checker input | more than one argument | `AlertMultipleArguments` (error) | CSI:158-163 |

`CheckSelfInterference()` runs **after** `MakeBlocks` (PF:336) and is not gated by `HasErrors()`.

### 2.10 Alert plumbing / allocator strategy

- `DEFINE_SIMPLE_ALERT` types **merge** (`Message_Alert::SupportsMerge()` returns true, Message_Alert.cxx:30)
  ⇒ at most one instance per type. `DEFINE_ALERT_WITH_SHAPE` types **do not merge**
  (TopoDS_AlertWithShape.cxx:31-34) ⇒ one live alert *per event*, each pinning a `TopoDS_Compound` of the
  offending shapes (PF2:630-641 `AddIntersectionFailedWarning`).
- `Message_Report::myLimit = -1` (MRP:35) and **nothing in OCCT ever calls `SetLimit`**; the trim branch
  (MRP:57-62) is only reached for merged alerts, so the report is effectively unbounded.
- `MakeBlocks` runs three allocators on purpose (PF6:669-718): an `IncAllocator` for cross-iteration data, a
  second `IncAllocator` reset per FF pair via `aTmpAllocator->Reset(false)` (PF6:768) so bucket arrays are
  reused, and `CommonBaseAllocator` for the collections that call `Remove/UnBind` (because
  `IncAllocator::Free()` is a no-op). `aFFs.SetIncrement(iSize)` (PF6:324) sizes the interference array growth.
- Progress weights encode the expected cost distribution: FF is `30·aFFSize`, MakeBlocks `5·aFFSize`, EF
  `10·aEFSize` (PF:464-476) — useful as a first-order model of where the memory goes too.

## 3. PORTING TRAPS

- **T1 — There is no memory budget; copying OCCT's structure copies the balloon.** `aVFaceFace` holds every
  pair's full intersection state until PF6:621. If our port already balloons at 4.7 GB, the fix is *not* in
  OCCT's guard set: it must be a streaming/chunked FF pass (consume + release per pair), which OCCT does not do.
  Chunking is safe: nothing in the treatment loop is order-dependent except `aFFs` append order (indices are
  stored in `BOPDS_CoupleOfPaveBlocks::IndexInterf`, PF6:787, 1039).
- **T2 — `RejectIndexMAX` is a *point* budget, not a *line* budget, and it leaks.** It is never reset across
  reversal (PW:835 vs `RepartirOuDiviser` PW:3297), so a line that is walked forward 200 k points then reversed
  gets only 50 k more. A port that resets it per direction doubles the worst-case line length (~40 MB/line at
  ~80 B per `IntSurf_PntOn2S` sequence node: `gp_Pnt` 24 B + 4 doubles 32 B + node overhead).
- **T3 — The commented-out `UVMaxStep` floor (IPI:163) is load-bearing.** `DefineUVMaxStep` returns `1e-4`
  whenever a cone apex / sphere pole sits within `(Confusion, 1e-5)` of the other surface (IPI:2380-2404). That
  single branch multiplies the walker's point count by ~10 for the *whole* pair. If our SYMEMIT case involves a
  near-apex or near-pole configuration, this is the first thing to instrument.
- **T4 — `Fleche` (deflection) has a floor of `1e-3` but the caller may bypass `SetTolerances`.** OCCT's own
  defaults path (IPI:184-191, 1078-1085, 1389-1396) sets `Fleche = 0.01` *without* the clamp block; only
  `SetTolerances` clamps. `IntTools_FaceFace` always goes through `SetTolerances` (FF:489) — a port that calls
  the intersector directly loses both the `[1e-3, 10]` deflection clamp and the `TolArc/TolTang ∈ [1e-8, 0.5]`
  clamp.
- **T5 — The FF "recheck" loop mutates the loop bound while iterating.** `aNbFF` is incremented inside the body
  (PF6:1070) while `for (i = 0; i < aNbFF; ++i)`; the only thing preventing unbounded growth is `i < aNbFFPrev`
  in the `isToRecheck` predicate (PF6:879). The `Message_ProgressScope` was constructed with the *original*
  `aNbFF` (PF6:659), so progress overruns — harmless in OCCT, but a port that derives work estimates from the
  scope will mis-size buffers.
- **T6 — Tangent-zone continuation bypasses the point budget.** `ExtendLineInCommonZone` collects into
  `aSeqOfNewPoint` and flushes with `AddAPoint` at PW:2381 **without touching `RejectIndex`**; its only bounds
  are `nbIterWithoutAppend > 20` and `nbEqualPoints > 20` (PW:1899). Tangential/near-tangential face pairs —
  precisely the SYMEMIT class — take this path.
- **T7 — Failure is degradation, not abort.** A failed pair yields `aFF.Init(0,0)` and a *warning*
  (PF6:545-552); a failed nested post-treat yields an *error* that aborts only `MakeBlocks` (PF6:1121). Porting
  "alert ⇒ stop" changes results on models OCCT completes. Conversely, `UserBreak` is recorded as an **error**
  (OPT:112-117), so `HasErrors()` cannot distinguish cancellation from geometric failure without inspecting the
  alert type (`HasError(STANDARD_TYPE(BOPAlgo_AlertUserBreak))`).
- **T8 — The nested `BOPAlgo_PaveFiller` in `PostTreatFF` has no recursion counter.** Termination relies on the
  nested argument list containing no faces (so `PerformFF` returns at PF6:315 and `MakeBlocks` at PF6:660). A
  port that feeds faces into the post-treat filler recurses forever. `SetIsPrimary(false)` only disables
  `ForceInterfEF` (PF5:775) and `SetNonDestructive` (PF10:43) — **not** `ForceInterfEE` (PF3:997).
- **T9 — `NBMAXUV = 30` is a *silent* accuracy/memory trade.** Dense NURBS get 30×30 seeds regardless of knot
  count (POLY:33 vs HIT:86-98 asking for `NbKnots*Degree*2`). Raising it to "be safe" grows the polyhedron
  quadratically *and* multiplies `Intf_Interference` section lines, each of which becomes a walking seed
  (IPP:409-436) — a direct blowup lever in the wrong direction.
- **T10 — 30-segment approximation cap silently degrades to raw polylines.** When `theapp3d` cannot fit within
  30 segments / degree 8, OCCT does not retry with more segments; it either takes the single `reApprox` shot at
  `tol = 1e-5` (FF:1727-1730) or dumps the WLine points straight into a BSpline (FF:1388-1407). A 250 k-point
  line then survives into the topology as a 250 k-pole curve — the memory is *moved*, not freed. Any port that
  keeps both the WLine and the fitted curve alive doubles it.
- **T11 — Alert-with-shape accumulation is itself a leak vector.** One `AlertIntersectionOfPairOfShapesFailed`
  per failed pair (PF6:551, PF5:334/1157, PF4:263, PF2:327, PF3:299/1277), each holding a compound of the two
  shapes, never merged (TopoDS_AlertWithShape.cxx:31), never trimmed (MRP:35). In a pathological run this is
  thousands of retained shape handles. Cap it in the port, or emit indices instead of shapes.
- **T12 — `CheckPlanes` is a correctness-relevant shortcut, not an optimisation.** Two planes with ≤1 shared
  vertex are declared non-interfering *before* intersection (PF6:380-390) — no line is produced even if the
  planes genuinely cross. Omitting it changes results (extra section edges); implementing it wrongly (counting
  ON+IN vertices twice) suppresses real intersections.
- **T13 — `TrsfToPoint` changes the numbers.** Faces further than `1e5` from the origin (with a small
  size/distance ratio) are intersected *at the origin* and the curves transformed back (PF6:215-235, 244-265).
  Reproducing OCCT's curve coefficients bit-for-bit requires reproducing this move, including the fact that the
  start-point list (`myListOfPnts`) is deliberately **not** transformed because it carries UV only
  (PF6:223-230).
