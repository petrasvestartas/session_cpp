# port_07 — PCURVES AND SAME-PARAMETER

**Subsystem**: giving a 3D intersection curve its exact 2D representation on a *curved trimmed
surface*, and making the 3D curve and every 2D representation agree at the same parameter.

This is a build document. Every algorithmic claim carries a `file:line` into the real OCCT tree at
`/home/petras/code/code_cpp/OCCT` (V8 layout: `occ::handle`, `NCollection_*`, `bool/int/double`) or
into `/home/petras/code/code_rust/session/session_cpp/src`. Sections marked **[BEYOND OCCT]** are
closed-form results OCCT does *not* implement (it approximates instead); they are derived here and
each carries the OCCT line that shows the fallback being taken, so the reader can verify that OCCT
really gives up there.

Files read in full for this document:

```
ModelingData/TKGeomBase/ProjLib/ProjLib_ProjectedCurve.cxx        (1179 L)
ModelingData/TKGeomBase/ProjLib/ProjLib_Plane.cxx  _Cylinder.cxx  _Cone.cxx  _Sphere.cxx  _Torus.cxx
ModelingData/TKGeomBase/ProjLib/ProjLib_Projector.cxx  ProjLib.cxx  ProjLib_ComputeApprox.cxx (1545 L)
ModelingData/TKGeomBase/Approx/Approx_SameParameter.cxx/.hxx
ModelingData/TKGeomBase/Approx/Approx_ComputeCLine.gxx  Approx_FitAndDivide2d.hxx
ModelingData/TKGeomBase/AppCont/AppCont_LeastSquare.cxx
ModelingData/TKGeomBase/GeomLib/GeomLib.cxx  (SameRange)
ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib.cxx  BRepLib_1.cxx  BRepLib_ValidateEdge.cxx
ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck.cxx  (PrecCurve/PrecSurface)
ModelingData/TKBRep/BRep/BRep_Tool.cxx  BRep_CurveOnSurface.{cxx,hxx}  BRep_CurveOnClosedSurface.hxx
ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools2D.cxx  BOPTools_AlgoTools.cxx
ModelingAlgorithms/TKBO/IntTools/IntTools_Tools.cxx  IntTools_EdgeFace.cxx
FoundationClasses/TKMath/ElSLib/ElSLib.cxx        ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_Surface.cxx
FoundationClasses/TKernel/Precision/Precision.hxx
```

Relation to existing kb: `kb/occt2_breplib-sameparameter.md` covers the same-parameter *doctrine*
and is broadly correct; `kb/audit_occt_tolerance-model.md` §2.2–2.6 audited it and its corrections
(hard-SET of `tolE`, the `1.5×` factor, `1.5*tolE+tolF`) are **confirmed here against source**. What
neither covers, and what this document adds, is **the pullback itself** — how the 2D curve is
produced, which cases are closed-form, and exactly where OCCT drops to a fit.

---

# 1. WHAT THIS SUBSYSTEM MUST GUARANTEE

Each invariant is stated so a test can decide it without an oracle. `E` = edge, `F` = face,
`S` = F's surface, `C3d` = E's 3D curve on `[f,l]`, `p` = the pcurve of E on S,
`Σ(u,v)` = the surface point map, `tolE` = E's tolerance.

**G1 — SAME PARAMETER.** For all `t ∈ [f,l]`: `|C3d(t) − Σ(p(t))| ≤ tolE`.
The *same* `t`. No projection, no re-parameterization, no nearest-point search is permitted at any
consumer. Testable: sample 23 parameters `t_i = f + i(l−f)/22`, assert the max.
(OCCT contract: `BRepLib.cxx:1721-1737` sets the flag only after measuring exactly this.)

**G2 — SAME RANGE.** `p` is *defined on* `[f,l]`, the identical interval as `C3d`, not on `[0,1]`
and not on its own chord length. Testable: `p.domain() == C3d.domain()` bitwise.
(`BRepLib.cxx:1719-1720` — `B.Range(aNE,f3d,l3d); B.SameRange(aNE,true)` is unconditional, even on
failure; `GeomLib::SameRange` at `GeomLib.cxx:842-...` is the renormalizer.)
G2 is a *precondition* of G1, never a consequence.

**G3 — CHART CONTAINMENT.** `p(t)` lies inside `[uf,ul]×[vf,vl]` for every non-periodic direction,
up to a band of 1% of the span. On a periodic direction, `p` may leave, but only by whole periods,
and only where the edge genuinely crosses the seam.
Testable: for non-periodic axes, `max_t (uf − p_u(t))⁺` and `max_t (p_u(t) − ul)⁺` are 0.
(`BRepLib.cxx:1083` `du = 0.01*(ul-uf)`; the excess is converted to a 3D distance and *charged as
deviation*: `dapp = max(dapp, DSdu*(uf − Puv.X()))`, `BRepLib.cxx:1096-1121`.)

**G4 — ENDPOINT IDENTITY, NOT COINCIDENCE.** `p(f)` and `p(l)` are the *stored* UV images of E's
two vertices as this face sees them. Two trims of the same wire that share a vertex must produce
*bitwise identical* UV endpoints, because they read the same stored value, not because two
independently computed numbers happened to be within a tolerance.
(`BRep_CurveOnSurface::Update()`, `BRep_CurveOnSurface.cxx:100-114`, caches `myUV1 = PC->D0(First)`,
`myUV2 = PC->D0(Last)`; `BRep_CurveOnSurface.hxx:74-75`.)
Testable: for every wire, `Σ_i |p_i(l_i) − p_{i+1}(f_{i+1})| == 0.0` exactly.

**G5 — ONE EDGE, N PCURVES, ONE 3D CURVE.** An edge shared by two faces holds *one* `C3d` and one
pcurve per (surface, location). Neither face owns a private 3D copy.
(`BRep_TEdge::myCurves` list; uniqueness enforced in `BRep_Builder::UpdateCurves`, replacement keyed
by `IsCurveOnSurface(S,l)` — note `BRep_CurveOnSurface.cxx:59-63` compares the surface handle by
**pointer identity**, `(S == mySurface) && (L == myLocation)`, not by geometric equality.)
Testable: every section edge has exactly 1 curve-3d index and exactly 2 trims.

**G6 — SEAM EDGES CARRY TWO IMAGES.** An edge lying on the seam of a periodic surface is ONE edge
with TWO pcurves in ONE representation, selected by the edge's orientation in the face.
(`BRep_CurveOnClosedSurface.hxx:33-88`; selection at `BRep_Tool.cxx:354-357`:
`if (GC->IsCurveOnClosedSurface() && Eisreversed) return GC->PCurve2();`)
Testable: no periodic face has two distinct edge records whose 3D curves coincide.

**G7 — ORIENTED, REGULAR, NON-SELF-CROSSING IN THE CELL.** `p` is C0 and has a well-defined
non-zero tangent except at chart poles; within one period cell it does not wrap more than once. The
2D arrangement that splits the face sorts edges around nodes by that tangent; a pcurve with a
vanishing or ambiguous tangent produces a wrong face partition, not a warning.

**G8 — TOLERANCE IS MEASURED, MODEL-SPACE, AND DERIVED FROM G1's RESIDUAL.**
`tolE = max over representations of (safety × measured 3D deviation)`, floored at `Confusion`.
Never a constant, never derived from a UV span.
Testable: rescale the surface's UV domain by 4× (a pure reparameterization, same point set) and
assert `tolE` is bit-identical.

**G9 — DEGENERACY IS TYPED.** A pcurve running into a pole (sphere ±π/2, cone apex, a degenerate
NURBS iso) is *named* as such and repaired by a prescribed rule (trim + straight-line extension to
the pole boundary), not left to a projector that returns an arbitrary `u`.
(`ProjLib_ProjectedCurve.cxx:130-159` `TrimC3d`, `:163-238` `ExtendC2d`.)

**G10 — NO FIT WHERE A CLOSED FORM EXISTS.** If the surface is one of the five analytic types and
the 3D curve is a conic or lies exactly on the surface, the pcurve is produced by the closed-form
chart inverse, exactly; a least-squares fit is used *only* for a genuinely free-form surface.
Testable: `pcurve_provenance(E,F) ∈ {EXACT_AFFINE, EXACT_CHART, FIT}` is recorded, and `FIT` never
appears for an analytic surface.

**G11 — INVERTIBILITY OF THE 3D↔2D BINDING.** `Σ(p(t))` reproduces `C3d(t)` and, conversely,
`π_S(C3d(t))` reproduces `p(t)` modulo period. There is exactly one binding, so the two faces of a
section edge integrate the *same* boundary. (Our measured failure of this is the first-order volume
error documented at `src/brep.cpp:7482-7486`.)

**G12 — TOLERANCE HIERARCHY AFTER THE PASS.** `tol(V) ≥ tol(E) ≥ tol(F)` for every incidence, and
`dist(P(V), rep(t_V)) ≤ tol(V)` for the 3D curve *and* every pcurve of every incident edge.
(`InternalUpdateTolerances`, `BRepLib.cxx:1744-1979`, vertex loop `:1858-1959`, margin `+2·Epsilon(tol)`
at `:1942`.)

### How a wrong pcurve manifests (all observed in our kernel)

| pcurve defect | downstream symptom |
|---|---|
| fitted cubic, ~1e-4 relative | UV-arrangement crossings land at wrong parameters ⇒ the two operands cut the *same* section curve at *different* points ⇒ two edge records, no shared identity |
| pcurve domain rescaled to chord length (G2 broken) | every consumer that assumes `t` is shared must re-project; the "same parameter" contract silently becomes "nearest point", and near-tangential regions pick the wrong branch |
| endpoints not bitwise shared (G4 broken) | wire closure decided by a coordinate tolerance; a padded UV domain inflates that tolerance and the wire opens — *32 naked edges of 36 on a single-operand split with a verified-identical UV arrangement* |
| pcurve sags between samples (deg-1 polyline) | face area and boundary flux integrate the sag ⇒ volume error first order in the sag; on a torus, `cv ≈ 4000` polylines (`src/brep.cpp:1589-1592`) |
| exact conic replaced by a spline | downstream conic recognizers fail (`src/brep.cpp:5306-5312` explicitly notes lift vertices sit ~1e-4 off the section circle) |
| pcurve leaves the chart and is silently clamped | deviation *measures as zero* because `Σ` was evaluated at the clamped border — the reason OCCT's `ComputeTol` charges out-of-chart excursions as 3D distance (`BRepLib.cxx:1096-1121`) |

---

# 2. OCCT'S ALGORITHM

## 2.0 Where a pcurve lives

```
BRep_TEdge  { myTolerance; myFlags{SameParameter=1,SameRange=2,Degenerated=4};
              myCurves : List<handle<BRep_CurveRepresentation>> }

BRep_CurveRepresentation                       (myLocation, relative to the edge location)
 └ BRep_GCurve            (myFirst, myLast)    — the shared parameter range
    ├ BRep_Curve3D        (myCurve)
    └ BRep_CurveOnSurface (myPCurve, mySurface, myUV1, myUV2)
       └ BRep_CurveOnClosedSurface (myPCurve2, myContinuity, myUV21, myUV22)
```

- Evaluation: `BRep_CurveOnSurface::D0(U,P)` = `P = mySurface->Value(myPCurve->Value(U))`, then
  `P.Transform(myLocation)` — `BRep_CurveOnSurface.cxx:42-48`. **The pcurve parameter IS the edge
  parameter.** That single line is the whole same-parameter contract.
- Endpoint cache refresh: `Update()` — `BRep_CurveOnSurface.cxx:100-114`, guarded against infinite
  ends. This is G4's storage.
- Lookup and seam addressing: `BRep_Tool::CurveOnSurface(E,S,L,f,l,theIsStored)` —
  `BRep_Tool.cxx:327-373`. Rep matched by `cr->IsCurveOnSurface(S, L.Predivided(E.Location()))`;
  `PCurve2()` returned when the rep is on a closed surface and the edge occurrence is REVERSED
  (`:354-357`).
- **Planes store nothing.** If no rep matches, `theIsStored` is set false and the pcurve is computed
  on the fly by `CurveOnPlane` (`BRep_Tool.cxx:379-450`): project the trimmed 3D curve onto the plane
  with `GeomProjLib::ProjectOnPlane` (`:431-435`), then `ProjLib_ProjectedCurve` (`:440`) and
  `Geom2dAdaptor::MakeCurve` (`:441`). `BRepLib::BuildPCurveForEdgeOnPlane`
  (`BRepLib_1.cxx:330-339`) is nothing but "call that, and tell me whether it was stored".
  Consequence to port: **any loop that iterates stored reps is blind to planar pcurves**; that is why
  `InternalSameParameter` needs a dedicated planar-face pass (`BRepLib.cxx:971-1002` → `GetEdgeTol`).
- Index overload `BRep_Tool::CurveOnSurface(E,C,S,L,f,l,Index)` — `BRep_Tool.cxx:488-538` — counts a
  closed-surface rep as **two** indices (`:512-521`).

## 2.1 The pullback dispatcher — `ProjLib_ProjectedCurve::Perform`

`ProjLib_ProjectedCurve.cxx:369-886`. This is the single entry point for "give me the 2D image of
this 3D curve on this surface".

Setup (`:371-387`): `myTolerance = max(myTolerance, Precision::Confusion())`; `eps = 0.01`;
`dt = (LastPar − FirstPar) * eps`; `TolConf = Precision::Confusion()`; chart bounds `U1,U2,V1,V2`
read from the adaptor.

Branch on surface type:

| surface | lines | what happens |
|---|---|---|
| Plane | 391-396 | `ProjLib_Plane P(Plane); Project(P, curve)` |
| Cylinder | 398-403 | `ProjLib_Cylinder` |
| Cone | 405-410 | `ProjLib_Cone` |
| Sphere | 412-445 | `ProjLib_Sphere`; on success `SetInBounds(FirstParameter)` (`:419`); **on failure** compute `TolConf = max(R·1e-5·π, |R − maxdist|)` (`:435`) and `TrimC3d` the curve away from each pole (`:437-441`) so the later approximation does not evaluate at the singularity |
| Torus | 447-452 | `ProjLib_Torus` |
| Bezier/BSpline surface | 454-537 | *not analytic*: detect degenerate isos with `IsoIsDeg` (`:468-494`), `TrimC3d` off each pole, then `ProjLib_ComputeApproxOnPolarSurface` (`:496-502`), then `ExtendC2d` straight-line extensions back to the pole and `GeomLib::SameRange` to restore the original range (`:509-532`) |
| anything else (revolution, extrusion, offset) | 540-714 | axis-singularity trims for revolution (`:546-612`), then a **numerical** projection: `ProjLib_HCompProjectedCurve(surface, curve, aTolU, aTolV, aMaxDist)` (`:623-624`) with `aMaxDist = 100·myTolerance` (`:618`), approximated by `Approx_CurveOnSurface` with `MaxDegree = 14` (`:647`), `MaxSeg = 16` (`:652`), continuity `C1` (or `C0` if `myBndPnt == PassPoint`, `:642-646`), `Only2d = true` (`:641`) |

`Project(ProjLib_Projector&, curve)` (`:242-270`) dispatches on **curve** type: Line, Circle,
Ellipse, Hyperbola, Parabola get a `Project` overload; `BSplineCurve / BezierCurve / OffsetCurve /
OtherCurve` **fall through doing nothing** (`:262-266`) — the comment says "try the approximation".

**The analytic fallback** (`:717-786`): `if (!myResult.IsDone() && isAnalyticalSurf)` run
`ProjLib_ComputeApprox` (§2.4). This is where every non-chart-aligned conic on a curved analytic
surface ends up. If the approximation also fails, `Perform` simply returns with `myResult` not done
(`:726-729`) and the caller gets a null pcurve.

**Periodic re-framing** (`:788-885`), run whenever the result is done and the surface is periodic in
u or v: for a BSpline result, sample `deg+1` points inside every knot span, histogram them into
period cells `k = floor((coord − first)/period)` (`:830-836`), find the most populated cell
(`:849-858`), and translate the whole curve by `−k·period` (`:859-865`). For a Line result use
`UFrame`/`VFrame` (`ProjLib_Projector.cxx:179-223`), which put the *first* point into
`[first, first+period)` via `ElCLib::InPeriod`.

## 2.2 The exact analytic arms OCCT implements — and exactly what they refuse

### Plane — `ProjLib_Plane.cxx`
`EvalPnt2d(P) = ((P−O)·Xd, (P−O)·Yd)` (`:87-92`), `EvalDir2d(D) = (D·Xd, D·Yd)` (`:94-97`).
- Line → `gp_Lin2d` (`:101-106`)
- Circle → `gp_Circ2d` with mapped axes, `myIsPeriodic = true` (`:110-123`)
- Ellipse → `gp_Elips2d` (`:127-139`), Parabola → `gp_Parab2d(Ax, Focal)` (`:143-154`),
  Hyperbola → `gp_Hypr2d` (`:158-169`)

For splines, `ProjLib_ComputeApprox::Perform` short-circuits entirely and **projects the control
points**, preserving degree, knots, multiplicities, weights and periodicity —
`ProjLib_ComputeApprox.cxx:1197-1228` (BSpline) and `:1229-1252` (Bezier), using the batched
`PlaneProjector` (`:55-89`). **Plane pullback is exact for every curve type. No fit ever.**

### Cylinder — `ProjLib_Cylinder.cxx`
`EvalPnt2d(P,Cy)` (`:74-91`): local `X,Y,Z` by dot products, `U = atan2(Y,X)` unless both
`|X|,|Y| ≤ PConfusion` in which case `U = 0`; returns `(U, Z)`.
- `Project(gp_Lin)` (`:95-118`): **refuses** unless the line is parallel to the axis —
  `CrossSquareMagnitude > Angular²` returns with `isDone` false (`:99-103`). Then the pcurve is
  `gp_Lin2d(P2d, (0, ±1))`, sign from `L.Direction()·axis` (`:112-114`).
- `Project(gp_Circ)` (`:122-156`): **refuses** unless the circle's normal is parallel to the axis
  (`:128-132`). Then `U = angle(Xcyl, Xcirc)` about `Zcyl` (`:138`), `V = (Ccirc−Ocyl)·Zcyl` (`:141`),
  direction `(±1, 0)` (`:145-152`).
- `Project(gp_Elips)` (`:161-165`): **empty body.** Verbatim comment:
  *"Due to widespread issues with poorly handled periodicity, the projection of an ellipse onto a
  cylinder is delegated to approximation."*
- Parabola/Hyperbola (`:167-175`) forward to `ProjLib_Projector::Project`, which just sets
  `myType = GeomAbs_OtherCurve` (`ProjLib_Projector.cxx:165-175`) — i.e. **give up**.

> **This is the single most important fact in this document.** The generic plane section of a
> cylinder — the ellipse that every rotated box × cylinder produces — is handled by OCCT with a
> least-squares B-spline fit, not a closed form. §2.5 gives the closed form.

### Cone — `ProjLib_Cone.cxx`
- `Project(gp_Lin)` (`:69-104`): if the line's location is the apex, step one unit along the
  direction and remember `aDeltaV = 1.0` (`:76-81`); invert with `ElSLib::ConeParameters` (`:83`);
  compute `Vv = ∂P/∂v` (`:88`); **refuse** unless `Dv ∥ L.Direction()` within `Precision::Angular()`
  (`:91`). Result `gp_Lin2d((U, V − aDeltaV·sign), (0, sign))` (`:96-100`).
- `Project(gp_Circ)` (`:108-165`): **refuse** unless the circle axis is parallel to the cone axis
  (`:115-119`). Then, inlining `ElSLib` to avoid a `gp_Trsf` round trip:
  `x = Xcone·Xcirc`, `y = Ycone·Xcirc`, `z = (Ccirc − Ocone)·Zcone` (`:125-127`);
  `U = 0` if `|x| ≤ Angular && |y| ≤ Angular` (`:133-136`), else `U = atan2(−y,−x)` when
  `−RefRadius > z·tan(SemiAngle)` (the far nappe, `:137-140`), else `atan2(y,x)`; wrap to `[0,2π)`
  (`:145-148`). **`V = z / cos(SemiAngle)`** (`:150`). Direction `(±1,0)` (`:154-161`).
- Ellipse/Parabola/Hyperbola (`:167-180`) → base class → `GeomAbs_OtherCurve`. **Give up.**

### Sphere — `ProjLib_Sphere.cxx`
`EvalPnt2d(vec, Sp)` (`:65-93`): `U = InPeriod(atan2(Y,X), 0, 2π)` unless both `|X|,|Y| ≤ PConfusion`
(then `U = 0`); `Z` clamped into `[−1,1]` (`:82-89`) and `V = asin(Z)`.
- `Project(gp_Circ)` (`:97-179`) recognizes exactly two cases, both with `Tol = Confusion`:
  - **isIsoU** (`:118`): `Zcirc ⟂ Zsphere` **and** the circle centre equals the sphere centre — a
    great circle through the poles. Endpoints from `EvalPnt2d(Xc)` and `EvalPnt2d(Yc)`; three
    fix-ups: pole-hit (`|V ∓ π/2| < PConfusion` ⇒ take `U` from the second point, `:131-138`),
    antipodal-`U` (`||U1−U2| − π| < PConfusion` ⇒ force `U2 = U1` and mirror `V2` about `±π`,
    `:139-153`), else `U2 = U1` (`:154-157`). Result is a `gp_Lin2d` (vertical in the chart).
  - **isIsoV** (`:119`): `Xc ⟂ Zs` and `Yc ⟂ Zs` — a parallel. `U = angle(Xs, Xc)` about `Xs×Ys`
    (`:167`), `V = asin(((Cc − Os)·Zs)/R)` (`:172-173`), direction `(±1, 0)` (`:175`).
  - **Everything else**: `isDone` stays false; `myLin` is written from uninitialised `P2d1`/`D2d`
    (`:178`) but never consumed because `IsDone()` is false.
- Line/Ellipse/Parabola/Hyperbola (`:181-199`) → base class → **give up**.
- `SetInBounds(U)` (`:203-248`), applied by `ProjectedCurve` at `:419`: bring `V(U)` into `[−π,π]`
  (`:208-211`); if the line passes beyond a pole (`V − π/2 > 1e-7`, or `|V − π/2| < 1e-7` with
  direction `+Y`; symmetric for the south pole) mirror the line about the `V = ±π/2` axis
  (`:220-240`), translate `U` by `π` (`:242`), then re-place `U` in `[0, 2π)` (`:245-247`).
  `Tol = 1.e-7` hard-coded at `:216`. This is the "a meridian great circle continues on the far side
  of the pole with `u` shifted by π" rule.

### Torus — `ProjLib_Torus.cxx`
`EvalPnt2d(v, To)` (`:59-77`): `U = atan2(v·Yt, v·Xt)` (or 0), `V = 0`.
- `Project(gp_Circ)` (`:81-173`), two branches:
  - **iso-V** if `|OC| < Confusion` **or** the circle axis ∥ torus axis (`:96-97`):
    `Z = ((Cc − Ot)·Zt)/r` clamped, `V = asin(Z)` (`:102-118`); if `Rcirc < MajorRadius` then
    `V = π − V`, else if `V < 0` then `V += 2π` (`:120-127`); the two sample `U`s give the direction,
    reversed if the seam was crossed (`|U1 − U2| > π`, `:131-136`).
  - **iso-U** otherwise (`:145-171`): `U = angle(Xt, OC)` about `Xt×Yt` (`:148`),
    `V1 = angle(OC, Xc)` about `OC×Zt` (`:155`), direction `±DY2d` from the sign of
    `(OC×Zt)·(Xc×Yc)` (`:164-168`).
  - **`isDone = true` unconditionally at `:172`** — there is *no* verification that the circle really
    is an iso. A circle on a torus that is neither a parallel nor a meridian (e.g. a Villarceau
    circle) produces a wrong straight line, silently. **Do not port this without the check.**
- Line/Ellipse/Parabola/Hyperbola (`:175-193`) → base → **give up**.

### Summary of OCCT's exact coverage

| surface \ curve | Line | Circle | Ellipse | Parab | Hypr | BSpline/Bezier |
|---|---|---|---|---|---|---|
| Plane | exact | exact | exact | exact | exact | **exact (pole projection)** |
| Cylinder | only ∥ axis | only ⟂ axis | **fit** | **fit** | **fit** | fit |
| Cone | only through apex | only ⟂ axis | **fit** | **fit** | **fit** | fit |
| Sphere | **fit** | only great-circle-through-poles or parallel | **fit** | **fit** | **fit** | fit |
| Torus | **fit** | iso-U / iso-V *unverified* | **fit** | **fit** | **fit** | fit |
| any free-form | fit | fit | fit | fit | fit | fit |

"fit" = `ProjLib_ComputeApprox` (§2.4) for analytic surfaces,
`ProjLib_ComputeApproxOnPolarSurface` / `Approx_CurveOnSurface` for the rest.

## 2.3 The exact chart inverse `π_S` — the primitive everything is built on

`ElSLib::*Parameters` is a **closed-form point inversion for every analytic surface**, valid even
when the point is not exactly on the surface (it returns the parameters of the nearest chart point in
the relevant sense). All work in the surface's local frame via `gp_Trsf::SetTransformation(Pos)`.

| surface | forward `Σ(u,v)` | inverse `π_S(P)`, local `(x,y,z)` | source |
|---|---|---|---|
| Plane | `O + uX + vY` | `u = x`, `v = y` | `ElSLib.cxx:1547-1554` |
| Cylinder R | `O + R(cos u·X + sin u·Y) + v·Z` | `u = atan2(y,x)` normalized, `v = z` | `:1558-1570` |
| Cone (R₀, α) | `O + (R₀ + v sinα)(cos u·X + sin u·Y) + v cosα·Z` | `u = 0` if `|x|,|y| < gp::Resolution`; `u = atan2(−y,−x)` if `−R₀ > z·tanα`; else `atan2(y,x)`; normalized. **`v = sinα(x cos u + y sin u − R₀) + cosα·z`** | `:1574-1611` |
| Sphere R | `C + R cos v(cos u·X + sin u·Y) + R sin v·Z` | `l = √(x²+y²)`; if `l < gp::Resolution` then `v = ±π/2, u = 0`; else **`v = atan(z/l)`**, `u = atan2(y,x)` normalized | `:1615-1645` |
| Torus (R, r) | `C + (R + r cos v)(cos u·X + sin u·Y) + r sin v·Z` | `u = atan2(y,x)`, plus a `R < r` disambiguation comparing the tube residuals at `u` and `u+π` (`:1664-1684`); then `ρ⃗ = (x − R cos u, y − R sin u, z)`, `v = ∠(d̂x, ρ̂)` about `d̂x × Ẑ`, `v = 0` if `|ρ⃗| ≤ gp::Resolution` | `:1649-1701` |

Two properties of these charts that the whole subsystem depends on:

1. **Orthogonality.** For all five, `Σ_u · Σ_v = 0` everywhere.
   *Cone check:* `Σ_u = (R₀+v sinα)(−sin u, cos u, 0)`, `Σ_v = (sinα cos u, sinα sin u, cosα)`;
   the dot is `(R₀+v sinα)·sinα·(−sin u cos u + cos u sin u) = 0`. Same for the others.
   This is exactly why OCCT's derivative helper is restricted to those five types and returns
   `false` for anything else: `Function_D1`, `ProjLib_ComputeApprox.cxx:183-235`, computes
   `dU = T·Σ_u/|Σ_u|²`, `dV = T·Σ_v/|Σ_v|²` (`:216-226`) — the correct chain rule **only** for an
   orthogonal chart — and bails when either `|Σ_u|² < Epsilon(1.)` or `|Σ_v|² < Epsilon(1.)`
   (`:219-222`), and for any non-analytic type at `:230-232`.
2. **Metric scale in closed form**, needed for every model-space→UV tolerance conversion:
   `|Σ_u| = 1, |Σ_v| = 1` (plane); `R, 1` (cylinder); `R₀ + v sinα, 1` (cone);
   `R cos v, R` (sphere); `R + r cos v, r` (torus).
   OCCT's coarse global version is `GeomAdaptor_Surface::UResolution/VResolution`
   (`GeomAdaptor_Surface.cxx:1818-1958`), which is the *chord-to-arc* inverse at the largest radius:
   `Res = R3d/(2R)` then `2·asin(Res)`, capped at `2π` (`:1890-1895`), with
   `R = radius` (sphere, cylinder), `R = major+minor` (torus U), `R = minor` (torus V),
   `R = max(VIso radius at Vfirst, Vlast)` and a plain `R3d/R` for the cone (`:1855-1868`),
   and `R3d` (plane, cylinder V, cone V, extrusion V) (`:1928-1933`). Unbounded cone
   (`myVLast − myVFirst > 1e10`) returns `Precision::Parametric(R3d) = 0.01·R3d` (`:1856-1860`).

## 2.4 The fit path OCCT actually takes — `ProjLib_ComputeApprox`

`ProjLib_ComputeApprox.cxx:1171-1492`. Used for every analytic surface × non-chart-aligned curve.

**Step A — the function to approximate.** `ProjLib_Function` (`:1026-1116`) is an `AppCont_Function`
with `myNbPnt = 0`, `myNbPnt2d = 1` (`:1047-1048`), i.e. a 1-branch 2D function of the *3D curve's*
parameter — so **the fit is same-parameter by construction**; it approximates `t ↦ π_S(C3d(t))`.
- `Function_Value(t)` (`:103-179`): `ElSLib::Parameters(surface, C3d(t), S, T)` for the five types
  (`:118-148`, `throw Standard_NoSuchObject` otherwise at `:146-147`), then seam folding:
  `if (UCouture && (S<U1||S>U2)) S = ElCLib::InPeriod(S,U1,U2)` (`:150-157`); for a sphere with
  `VCouture`, if `|S − U1| > π` then `T = π − T; S = π + S` (`:158-165`) — the "over the pole"
  reflection — then `T` folded into `[V1,V2]` (`:172-175`).
- `Function_D1` (`:183-235`) as in §2.3.
- `Function_SetUVBounds` (`:262-1019`) decides `myU1,myU2,myV1,myV2,UCouture,VCouture` before any
  sampling. It is 750 lines of seam bookkeeping and it is where most of OCCT's cylinder/sphere/cone
  pcurve bugs historically lived. Key mechanics:
  - `Function_ComputeStep(curve, R)` (`:239-258`): `nbp = len/(R·π/4) + 1`, at least 3;
    `Step = (W2−W1)/(nbp−1)`, capped at `Step0 = 0.1`.
  - Cylinder, general curve (`:522-589`): march with that step, accumulate an unwrapping `Delta`
    whenever `|U − U_prev| > π` (`:539-552`), track `myU1/myU2` and `dmax`, then pad the extremes by
    `dmax·0.5` if the extremum was interior (`:567-576`), and set `UCouture` iff the resulting span
    leaves `[0, 2π]` (`:578-588`).
  - Cylinder × ellipse (`:467-521`): closed vs open ellipse decides whether the chart window is
    `[U−π, U+π]` or a full `2π` window on one side, chosen by the sign of `T·Σ_u` at the start
    (`:503-519`).
  - Sphere × circle (`:596-813`): OCCT *solves the seam-crossing count algebraically* — the number
    of solutions of {sphere ∩ circle-plane ∩ (y = 0, x > 0)} — with `Tol = 1e-10`, `:614-676`;
    1 solution ⇒ open pcurve on a `2π` window; 0 or 2 ⇒ closed pcurve on `[Uc−π, Uc+π]`
    (`:735-771`). `VCouture` is set when the circle plane is tangent to the sphere
    (`||D/C| − R| ≤ 1e-10`, `:774-806`) — and then immediately neutralised by the
    "box+sphere" patch `myV1 = −1e100; myV2 = 1e100` (`:808-811`).
  - Cone (`:287-461`), Torus (`:890-1010`): same march-and-unwrap scheme, torus in both directions.

**Step B — the fit.** (`:1284-1329`)
```
Deg1 = 5                                              (:1285)   // min degree
Deg2 = simplecase ? 8 : 10                            (:1286-1293)
aMaxSegments = 1000                                   (:1304)
aTolU = UResolution(myTolerance) capped at 0.01·UPeriod   (:1120-1129, :1318)
aTolV = VResolution(myTolerance) capped at 0.01·VPeriod   (:1133-1142, :1319)
aTol2d = max( sqrt(aTolU² + aTolV²), Precision::PConfusion() )   (:1320-1321)
Approx_FitAndDivide2d Fit(Deg1, Deg2, myTolerance, aTol2d, /*cutting*/true, aFirstC, aLastC)  (:1323)
Fit.SetMaxSegments(1000); if (simplecase) Fit.SetHangChecking(false);   (:1324-1328)
```
`simplecase` (`:1181-1195`) = analytic surface **and** analytic curve; for a BSpline/Bezier curve it
additionally requires `degree ≤ 2 && nbKnots ≤ 2`.
Default `myTolerance = Precision::PApproximation() = 1e-8` (`:1147`), raised by the caller.

**The fitting engine**, `Approx_ComputeCLine.gxx` (instantiated as `Approx_FitAndDivide2d` via
`Approx_FitAndDivide2d_0.cxx:17-25`):
- `MAXSEGM = 1000` (`:26`).
- `Perform` (`:95-230`): `TolU = max(span·1e-3, Confusion)` when hang-checking is on, else
  `max(span·1e-5, PApproximation)` (`:104-111`). Bisection loop: `Compute` on `[myfirstU, mylastU]`;
  on failure halve `mylastU` (`:205`, `aNbCut++`); on success advance to the next interval
  (`:141-158`). Hang guard: after `aNbComp = 10` cuts, if cuts exceeded improvements by more than 1,
  stop cutting and accept the best kept curve (`:189-195`, `:200-219`). Termination is *guaranteed*:
  either `|myfirstU − mylastU| ≤ TolU`, or `aMaxSegments ≥ myMaxSegments − 1`, or the hang guard.
- `Compute` (`:262-380`): `NbPointsMax = 24`, `aMinRatio = 0.05`, `aMaxDeg = 8` (`:264-266`).
  Degrees are scanned **from `degreemax` down to `degreemin`** (inverse order is the default,
  `:52-53`, `:284`), except that for a sub-interval shorter than 5% of the whole and
  `degreemax > 8` the order is flipped back (`:272-279`). For each degree,
  `NbPoints = min(2·deg + 1, 24)` (`:287`), and one `AppCont_LeastSquare` is solved on that Bezier
  segment. Acceptance: `TheTol3d ≤ mytol3d && TheTol2d ≤ mytol2d` (`:292`); the *lowest* degree that
  passes is kept (`:294-311`).
- `AppCont_LeastSquare` (`AppCont_LeastSquare.cxx`): Gauss–Legendre collocation —
  `math::GaussPoints(myNbPoints, GaussP); math::GaussWeights(...)` (`:150-152`), weighted normal
  equations on the Bezier poles. **Error criterion** `Error(F, MaxE3d, MaxE2d)` (`:551-607`):
  residual at each Gauss point, `MaxE2d = sqrt(max over Gauss points of (e₁²+e₂²))` (`:594-606`).
  **This is a discrete error at ≤24 Gauss points per segment — not a certified bound.**

**CV budget.** Per accepted segment: `deg+1` poles, `deg ∈ [5, 8]` (analytic pair) or `[5, 10]`.
`Convert_CompBezierCurves2dToBSplineCurve2d` (`:1338-1352`) welds the Bezier segments into one
`Geom2d_BSplineCurve`, raising continuity to C1 where possible. Hard caps: `NbPoles` and `NbKnots`
must be in `(0, 100000]` else `Perform` returns with a null result (`:1356-1363`).

**Step C — range and seam repair.**
- `BSplCLib::Reparametrize(C->FirstParameter(), C->LastParameter(), NewKnots)` (`:1372`), then
  `NewKnots(NbKnots) = C->LastParameter()` exactly, "to avoid problems if trim is used" (`:1374-1376`).
  **That is G2 being restored explicitly.**
- If a `PassPoint` constraint was used, smooth by removing interior knots at
  `aSmoothTol = max(Confusion, aNewTol2d)` (`:1385-1395`).
- "Return curve home" (`:1417-1490`): evaluate the true chart parameters `(u,v)` of the 3D curve at
  the mid parameter, compare against the fitted curve's value there, and translate by whole `π`
  multiples: `du = u − P2d.X()`, nudged by `±PConfusion`, `modf(du/π, &aNbPer)`, `du = number·π`
  (`:1470-1477`); `dv` from `ElCLib::InPeriod` bookkeeping (`:1456-1467`); for the sphere V-couture
  case also mirror about the X axis (`:1458-1463`, applied `:1486-1489`).

**Step D — tolerance back-conversion** (`:1408-1415`):
```
myTolerance *= (aNewTol2d / aTol2d);
```
with the explicit comment that this assumes `aTolU/aTolV` keep their ratio. So the *reported* 3D
tolerance of the pullback is the achieved 2D error rescaled by the requested 2D/3D ratio — an
estimate, not a measurement. The measurement happens later (§2.7).

### The free-form path
`ProjLib_ComputeApproxOnPolarSurface` (called at `ProjLib_ProjectedCurve.cxx:496-502`) for
Bezier/BSpline surfaces, and `ProjLib_HCompProjectedCurve` + `Approx_CurveOnSurface` (`:623-659`) for
everything else. Both are genuine numerical projections (per-point Newton on the surface) followed
by an approximation; `MaxDegree = 14`, `MaxSeg = 16`, `aMaxDist = 100·tol` — **a point whose
projection is farther than `aMaxDist` terminates the projected branch**, which is how a curve that
leaves the face is truncated rather than mis-projected.

## 2.5 **[BEYOND OCCT]** The closed-form pullback for oblique curves

Everything here is exact, needs no fit, and removes the ~1e-4 error class entirely. The construction
has two layers.

### Layer 1 — the *functional pcurve*: exact for any curve on any analytic surface

For any 3D curve `C(t)` whose points lie on an analytic surface `S`, define

```
p(t) = π_S(C(t))            evaluated by the closed forms of §2.3
```

with a **continuous branch**: on a periodic axis, accumulate the unwrapping offset so that
`|p_u(t_{i+1}) − p_u(t_i)| < period/2` along the curve, and split at seam crossings (§2.6).

Properties, all provable and testable:
- `Σ(p(t)) = C(t)` to machine precision — G1 with `tolE` at the level of `Confusion`, independent of
  curve type, surface pose, or UV domain choice.
- G2 by construction: `p` is a function *of `t`*, so its range is `C3d`'s range.
- Exact first derivative without finite differences, by the orthogonal-chart chain rule (§2.3):
  `p'(t) = ( C'(t)·Σ_u / |Σ_u|² , C'(t)·Σ_v / |Σ_v|² )` — this is `Function_D1`,
  `ProjLib_ComputeApprox.cxx:209-227`, used here as *the pcurve's own derivative* rather than as a
  fitting aid.
- Exact seam crossing parameters: `p_u(t) = u_seam` is solved on the *3D* curve
  (`(C(t)−O)·Y = 0` and `(C(t)−O)·X > 0` for the `u=0` seam of a cylinder/cone/sphere/torus), i.e.
  a scalar root find on an exact function, so both operands compute the *same* crossing.

A functional pcurve is not a NURBS. That is the point: a NURBS pcurve of an oblique cylinder section
**does not exist** (see below), so any implementation that insists on NURBS pcurves is choosing an
approximation before it starts. Store the function; convert to NURBS only when writing STEP.

### Layer 2 — closed-form *analytic* pcurves for planar sections

For the important case "3D curve = intersection of the surface with a plane
`A·x + B·y + C·z + D = 0` expressed in the surface's local frame", the chart image has a closed
algebraic form in `u`. Write `K(u) = A cos u + B sin u`, and use the standard identity
`P cos θ + Q sin θ = √(P²+Q²)·cos(θ − ψ)` with `ψ = atan2(Q, P)`.

**Cylinder radius R.** `x = R cos u, y = R sin u, z = v`.
```
A R cos u + B R sin u + C v + D = 0
⇒  v(u) = −( D + R·K(u) ) / C                     (C ≠ 0)
⇒  v(u) = v₀ + A_amp · cos(u − φ),
        v₀ = −D/C,  A_amp = −R√(A²+B²)/C,  φ = atan2(B, A)
```
**The oblique plane section of a cylinder pulls back to a pure sinusoid in the (angle, height)
chart.** Chart-aligned specialisations: `A = B = 0` ⇒ `v = const` (OCCT's `Project(gp_Circ)`);
`C = 0` ⇒ the section degenerates to two axis-parallel lines `K(u) = −D/R`, i.e. `u = const`
(OCCT's `Project(gp_Lin)`).
A sinusoid is **not** a rational function of `u`, therefore not a NURBS — which is precisely why
`ProjLib_Cylinder::Project(const gp_Elips&)` is an empty body with an apology
(`ProjLib_Cylinder.cxx:161-165`).

**Cone (R₀, α).** Local radius `r(v) = R₀ + v sinα`, height `z(v) = v cosα`.
```
r(v)·K(u) + C·v cosα + D = 0
⇒  v(u) = −( D + R₀·K(u) ) / ( sinα·K(u) + C cosα )
```
A ratio of first-degree trigonometric polynomials. Under the Weierstrass substitution
`w = tan(u/2)` both numerator and denominator become quadratics in `w`, so `v` is a rational
quadratic function of `w` — but `u = 2·atan w` is transcendental, so `(u(w), v(w))` is still not a
NURBS. Poles of `v(u)` occur where `sinα·K(u) + C cosα = 0`: those are exactly the generators
parallel to the cutting plane (parabola/hyperbola cases) and must be handled as `v → ±∞` branch
splits.
Chart-aligned specialisations: `A = B = 0` ⇒ `v = −D/(C cosα)` const (OCCT `Project(gp_Circ)`,
matching its `V = z/cos(SemiAngle)` at `ProjLib_Cone.cxx:150`); a plane through the apex gives
`D = −R₀·K(u₀)` with `u` const on each generator (OCCT `Project(gp_Lin)`).

**Sphere radius R.** `x = R cos v cos u, y = R cos v sin u, z = R sin v`.
```
R cos v·K(u) + R C sin v + D = 0
⇒  K(u) cos v + C sin v = −D/R
⇒  v(u) = ψ(u) ± acos( S / √(K(u)² + C²) ),   ψ(u) = atan2(C, K(u)),  S = −D/R
```
The `±` are the two arcs of the small circle; they meet where the acos argument reaches ±1, i.e. at
```
K(u)² + C² = S²   ⇒  the u-extremes of the circle
```
which is a closed-form quadratic in `(cos u, sin u)` — solve it once and you have the exact
turning-point parameters. Chart-aligned specialisations: `A = B = 0` ⇒ `K ≡ 0`, `ψ = π/2`,
`v = π/2 ± acos(S/|C|)` = const — OCCT's `isIsoV`; `C = 0, D = 0` ⇒ `S = 0`, `ψ = 0`,
`v = ±π/2` at the K-zeros — OCCT's `isIsoU` great circle.

**Torus (R, r).**
```
(R + r cos v)·K(u) + C r sin v + D = 0
⇒  r K(u) cos v + C r sin v = −D − R K(u)
⇒  v(u) = atan2(C·r, r·K(u)) ± acos( (−D − R·K(u)) / (r·√(K(u)² + C²)) )
```
This is the exact chart form of the entire **spiric of Perseus** family, including the
Cassini/lemniscate and Villarceau degeneracies. The acos argument leaving `[−1,1]` marks the `u`
intervals where the plane misses the tube — that is the exact, closed-form answer to "where does the
section loop start and stop", and it is why a torus boolean does not need a marcher at all for
plane cuts. Chart-aligned specialisation `A = B = 0`: `v = π/2 ± acos(−D/(rC))`, two constants — the
two parallels.

**Non-planar section curves** (cylinder×cylinder quartic, sphere×cylinder, torus×anything) have no
closed `v(u)`; use Layer 1 (functional pcurve). Layer 1 is already exact; Layer 2 only adds the
ability to answer "where are the extremes / seam crossings / branch splits" analytically instead of
by root-finding.

### What Layer 1 costs, and why it is admissible
A functional pcurve cannot be stored in a STEP `PCURVE` directly. That is not a problem for the
boolean: the boolean needs `p(t)`, `p'(t)`, exact seam crossings, and exact endpoint values —
all of which the functional form provides better than any spline. Approximation is deferred to
export, where a documented tolerance is legitimate, and where OCCT's own machinery (§2.4) is the
right tool.

## 2.6 Seam handling and periodic placement (required for G3/G6)

Two mechanisms, both mandatory:

1. **Splitting.** A section curve that crosses a seam becomes *n* pcurve pieces, one per period
   cell, each with the crossing `u` set to the cell boundary **exactly** (assign the boundary
   constant; never let the computed value stand). OCCT does the equivalent by shifting the whole
   fitted pcurve into the most-populated period cell
   (`ProjLib_ProjectedCurve.cxx:812-865`) and by `AdjustPCurveOnSurf` (below).
2. **Placement.** `BOPTools_AlgoTools2D::AdjustPCurveOnSurf`
   (`BOPTools_AlgoTools2D.cxx:247-400`) decides the translation `(du, dv)` for a finished pcurve:
   - `aDelta = Precision::PConfusion()` (`:263`); the pcurve is probed at the mid parameter
     `aT = 0.5(aFirst+aLast)` (`:265-271`);
   - U: snap `u2` to `UMin` or `UMin + UPeriod` within `aDelta` (`:281-288`), then
     `GeomInt::AdjustPeriodic` (`:290`); if that yields `du == 0` and the surface is a **cylinder**,
     use an angular slack derived from the *edges' tolerances*:
     `dFi = MaxToleranceEdge(F)/R`, at least `aDelta` (`:296-305`), and shift by a full period if
     `u2` is outside by more than `dFi` (`:307-312`);
   - V: shift by `±VPeriod` if outside by more than `aDelta` (`:324-330`); if the face's V span is
     less than a period, prefer whichever of `v2`, `v2+dv` is nearer the mid of `[VMin,VMax]`
     (`:332-343`);
   - final arbitration by **point-in-face classification**: if the shifted mid point classifies
     `TopAbs_OUT`, shift back by a period (`:346-387`, `BRepClass_FaceClassifier`).
   - Then, and only then, translate a *copy* of the pcurve (`:389-397`).

Note the asymmetry that must be preserved: U uses a tolerance-derived angular slack on cylinders;
V uses a bare `PConfusion`.

## 2.7 SAME-PARAMETER

### 2.7.1 What it enforces
G1 and G2, by *changing the pcurve's parameterization* (never its point set in 3D, never the surface,
never the 3D curve), and then *recording the residual as the edge tolerance*.

### 2.7.2 Shape-level driver — `InternalSameParameter` (`BRepLib.cxx:913-1008`)
1. For each unique edge (`:924-969`):
   - if `IsForced` and (`SameRange` or `SameParameter` is set) **clear both flags first**
     (`:931-946`), empty-copying the edge when the input is immutable;
   - `aResEdge = BRepLib::SameParameter(aNE, theTol, aNewTol, UseOldEdge)` (`:948`);
   - if `aNewTol > 0`, merge it into a shape→tolerance map for **both end vertices** via `UpdTolMap`
     (`:954-966`), which is **max-only** (`:799-831`).
2. **Planar-face pass** (`:971-1002`): for every non-degenerate edge of every *plane* face,
   `GetEdgeTol` (`:719-793`). This exists because plane pcurves are never stored, so step 1 never
   measured them. `GetEdgeTol`:
   - returns immediately if a stored rep for `(S, l)` exists (`:728-736`);
   - builds the plane pcurve on the fly with `ProjLib_ProjectedCurve` (`:765-766`);
   - **23 samples** `t = i/23, i = 0..23` on `[First,Last]` (`:770-777`);
   - per-sample floating-noise floor: `eps = Epsilon(max(|Pc3d|², |Pcons|²))`; squared distances
     below `eps` are zeroed (`:779-785`);
   - `theEdTol = 1.05 · sqrt(max squared distance)` (`:791`).
3. `UpdShTol(map, ..., theVForceUpdate = false)` (`:1005`) applies the map: faces via `UpdateFace`
   (a **set**), edges via `UpdateEdge` (max), vertices via `UpdateTolerance` (max) or hard
   `Tolerance()` when forced (`:877-902`).
4. `InternalUpdateTolerances(theSh, false, ...)` (`:1007`) — the V≥E≥F harmonization.
   **This step is part of `SameParameter(shape)` but NOT of `SameParameter(edge)`.**

### 2.7.3 Per-edge core — `BRepLib::SameParameter(edge, tol, newTol&, useOldEdge)` (`BRepLib.cxx:1251-1740`)

**Guard (`:1256-1259`)**: if the edge already carries `SameParameter`, return immediately, doing
nothing. A port that marks constructed edges same-parameter by default disables this whole function
(audit trap #15).

**Setup**
- `GetCurve3d` (`:1192-1218`) takes the first `BRep_Curve3D` rep: `C3d`, `[f3d,l3d]`, location,
  and the live rep list. Null `C3d` ⇒ return a null edge (degenerate edges are never processed).
- `NCONTROL = 22` (`:1294`) ⇒ 23 sample points.
- **Trimmed-periodic guard** (NIZHNY-OCC486, `:1303-1332`): clamp `[f3d,l3d]` into the curve's own
  parameter range **only if** the curve is not a `Geom_TrimmedCurve` over a periodic basis. A
  seam-crossing trimmed periodic curve legitimately has `l3d > basis LastParameter`.
- Apply the location to `C3d` (`:1333-1336`); `GAC.Load(C3d, f3d, l3d)` (`:1337`).
- `Prec_C3d = BRepCheck::PrecCurve(GAC)` (`:1339`) — `RealEpsilon()` except for an **ellipse**, where
  it is `max_i Epsilon(|x_i|)` over centre coordinates and both radii (`BRepCheck.cxx:70-99`).
- `anEdgeTol = BRep_Tool::Tolerance(aNE)` (`:1345`); `BigError = 1.e10` (`:1349`).

**Per representation** (`:1352-1718`), for `PC[0] = PCurve()` and, when
`IsCurveOnClosedSurface()`, also `PC[1] = PCurve2()` (`:1363-1375`) — both measured independently in
the same `i = 0,1` loop:

1. `TolSameRange = max(GAC.Resolution(theTolerance), Precision::PConfusion())` (`:1378`).
2. **SameRange normalisation** (`:1387-1392`): if the edge is not SameRange,
   `GeomLib::SameRange(TolSameRange, PC[i], GCurve->First(), GCurve->Last(), f3d, l3d, curPC)`.
   `GeomLib::SameRange` (`GeomLib.cxx:842-...`) is: identity if both ends already match within the
   tolerance (`:854-859`); if only the *length* matches, translate a line along its direction
   (`:864-871`), rotate a circle about its centre by `dU` with a sign depending on
   `Circ2d().IsDirect()` (`:872-889`), recurse through a trimmed curve (`:890-901`), else convert to
   B-spline and `BSplCLib::Reparametrize` (`:908-922`); if the lengths differ, segment first, then
   convert and reparametrize (`:924-...`).
3. **Deviation** `error = ComputeTol(HC, HC2d, HS, 22)` (`:1396`). If `error > 1e10`, record it and
   break out of this representation (`:1398-1402`).
4. **C0 pcurve repair** — only for a `GeomAbs_BSplineCurve` pcurve whose `Continuity() == GeomAbs_C0`
   (`:1404-1623`):
   - `TolConf2d = max(min(UResolution(tol), VResolution(tol)), PConfusion)` (`:1406-1409`);
   - remember the origin point `bs2d->D0(fC0, OriginPoint)` (`:1415`) and
     `Geom2dConvert::C0BSplineToC1BSplineCurve(bs2d, TolConf2d)` (`:1416`);
   - **periodic origin restoration** (IFV Jan 2000, `:1419-1442`): if the origin moved by more than
     `PConfusion` in either coordinate, scan the knots for one whose point equals the old origin and
     `SetOrigin(Index)`;
   - still C0? `EvalTol(curPC, S, GAC, tol, tolbail)` (`:1447` → `:1034-1066`): 5 interior samples at
     `t = i/6`, project each `S(PC(t))` back on the 3D curve with `Extrema_LocateExtPC`, require
     `> 2` successes, return the max projection distance. Then retry the conversion with
     `Tol2dbail = max( min( min(UResolution(tolbail), VResolution(tolbail)), 0.1·min adjacent pole
     distance ), TolConf2d )` (`:1450-1467`), repeating the origin restoration (`:1471-1494`). Still
     C0 ⇒ keep the original curve, `repar = false` (`:1496-1501`); `EvalTol` failure ⇒
     `goodpc = false` (`:1503-1506`).
   - **Reparametrize** (`:1511-1535`): `BSplCLib::Reparametrize(fC0, lC0, Knots)` — back to the
     *pcurve's own* original range, **not** `[f3d,l3d]` (the commented-out line at `:1514` is bug
     history). Re-measure `error1 = ComputeTol(...)` (`:1522`); if it got worse, restore the saved
     `bs2dsov` and set `isANA = true` (`:1523-1530`).
   - **Bad-knot detection** (`:1537-1582`): if continuity > C0 and `error > max(1e-3, tol)`, compute
     the ratio of adjacent knot intervals; `critratio = 10` (`:1545`); any ratio above it marks the
     pcurve bad — unless `dtmin ≥ bs2d->Resolution(max(1e-3,tol))` (`:1576-1580`), which un-marks it
     "to avoid failures in `Approx_CurvilinearParameter`".
   - **Arc-length reparameterization** (`:1584-1621`): `Approx_CurvilinearParameter(HC2d, HS,
     max(1e-3, tol), min(cont, C2), maxdeg, 10)` with `maxdeg = bs2d->Degree()`, forced to **14** if
     the degree was 1 (`:1593-1603`); on success, if the resulting range drifted from `[fC0,lC0]` by
     more than `TolSameRange`, reparametrize back (`:1610-1619`).
5. **The reparameterization itself** (`:1625-1696`):
   ```
   aTol = (isANA && isBSP) ? 1.e-7 : theTolerance                     (:1628)
   Approx_SameParameter SameP(HC, HC2d, HS, aTol);                    (:1631)
   ```
   - `SameP.IsSameParameter()` ⇒ nothing to change; `maxdist = max(maxdist, SameP.TolReached())`
     (`:1633-1647`).
   - `SameP.IsDone()` ⇒ **accept the new pcurve only if it is not worse**:
     `if (tolreached <= error) { curPC = SameP.Curve2d(); maxdist = max(maxdist, tolreached); }`
     else `maxdist = max(maxdist, error)` and the original pcurve is kept (`:1648-1672`).
   - failed ⇒ fall back to `GeomLib::SameRange` and set `IsSameP = false` (`:1673-1696`).
   Write-back is `GCurve->PCurve(curPC)` for `i == 0`, `GCurve->PCurve2(curPC)` for `i == 1`.
6. **OCC5898 numeric rescue** (`:1703-1714`): if `!IsSameP`, and
   `anEdgeTol + max(Prec_C3d, BRepCheck::PrecSurface(HS)) ≥ error`, declare success anyway with
   `maxdist = max(maxdist, anEdgeTol)`. `PrecSurface` is `RealEpsilon()` except for a **cone**, where
   it is `max_i Epsilon(|x_i|)` over the apex coordinates and `RefRadius` (`BRepCheck.cxx:103-130`).
   At 1e6-scale coordinates this is ~1e-10, not 2e-16 — that is the point of the rescue.

**Epilogue** (`:1719-1737`)
```
B.Range(aNE, f3d, l3d);                 // unconditional, pushes the range into ALL reps
B.SameRange(aNE, true);                 // unconditional, even on failure
if (IsSameP) {
   if (YaPCu) {                         // at least one pcurve existed
      maxdist   = max(maxdist, Precision::Confusion());
      theNewTol = maxdist;
      aNTE->Modified(true);
      aNTE->Tolerance(maxdist);         // *** HARD SET — this can SHRINK tolE ***
   }
   B.SameParameter(aNE, true);
}
```
The comment at `:1723-1727` states the reasoning: all representations were just measured, so the
edge tolerance may be reduced; **vertices may not be**, and are raised separately by `UpdateVTol`
(`:1222-1233`) / `UpdTolMap`, both max-only.

Consequence to port faithfully or not at all: a section edge created with
`tolE = aTolR3D = 3e-4` can leave `SameParameter` with `tolE = 1e-7`. OCCT survives this only
because `PostTreat`'s `CorrectShapeTolerances` later re-raises `tolE ≥ tolF` and `tolV ≥ tolE`
(audit trap #2, `kb/audit_occt_tolerance-model.md` §3.2).

### 2.7.4 `ComputeTol` — the deviation measurement (`BRepLib.cxx:1070-1188`)

```
uf,ul,vf,vl = surface chart bounds
du = 0.01*(ul-uf) ;  dv = 0.01*(vl-vf)                                (:1083)
DSdu = 1/UResolution(1.) ;  DSdv = 1/VResolution(1.)                  (:1085)
dapp = -1
for i = 0..nbp:                                                       (:1090)
    t = i/nbp ;  u = first*(1-t) + last*t
    Pc3d = c3d->Value(u) ;  Puv = c2d->Value(u)
    if !UPeriodic and Puv.X() < uf-du:  dapp = max(dapp, DSdu*(uf-Puv.X())); continue   (:1096-1102)
    if !UPeriodic and Puv.X() > ul+du:  dapp = max(dapp, DSdu*(Puv.X()-ul)); continue   (:1103-1107)
    ... same for V with DSdv                                          (:1109-1121)
    Pcons = surf->Value(Puv)
    if any coordinate infinite: d2 = Precision::Infinite(); break      (:1123-1128)
    dist(i+1) = Pc3d.SquareDistance(Pcons) ;  d2 = max(d2, that)       (:1129-1133)
if infinite: return d2                                                 (:1136-1139)
d2 = sqrt(d2)
if dapp > d2: return dapp                                              (:1142-1145)
# outlier vote
N1 = #{dist(i) in (0,1)} ;  N2 = #{dist(i) >= 1}                       (:1153-1166)
if N1 > N2 and N2 != 0: N3 = 100*N2/(N1+N2)                            (:1168-1171)
if 0 < N3 < 10:  ana = true;  D2 = max{ dist(i) : 0 < dist(i) < 1 }    (:1172-1182)
d2 = ana ? 1.5*sqrt(D2) : 1.5*d2                                       (:1185)
return max(d2, 1.e-7)                                                  (:1186)
```
Three things a port must not simplify:
- the **out-of-chart parametric penalty** (`DSdu × overshoot`) — without it, evaluating a
  clamped `Σ(uv)` reports near-zero deviation for a pcurve that has left the face;
- the **<10% outlier vote** — a single blown-up sample at a pole must not poison the tolerance, but
  a genuinely diverging 20% must;
- `1.5×`, floor `1e-7`. Not `1.00001`, not `1.05`. Those belong to other functions (§2.9).

### 2.7.5 `Approx_SameParameter` — the reparameterization engine

`Approx_SameParameter.cxx`, header constants `myNbSamples = 22` ("to be consistent with checkshape",
`.hxx:161`), `myMaxArraySize = 1000` (`.hxx:162`), `myDeltaMin = Precision::PConfusion()`
(ctor init `:274`, `:290`, `:306`).

`Build(Tolerance)` (`:318-543`):
1. `BuildInitialDistribution` (`:547-580`): 22 uniform parameters on the 2D range and on the 3D
   range, plus the last point ⇒ 23 pairs. If the pcurve continuity is below C1, replace the
   distribution by one sample per C1 interval, merged with the uniform ones
   (`IncreaseInitialNbSamples`, `:588-649`); returns false if that exceeds `myMaxArraySize − 1`.
2. `CheckSameParameter(data, sqDist)` (`:653-766`): compare `|Σ(p(t_i)) − C3d(t_i)|²` against
   `Tol²` at each sample; a sample already within tolerance *and* separated from the previous by
   `myDeltaMin` is kept as-is (`:685-698`); otherwise project `Σ(p(t_i))` onto the 3D curve with
   `Extrema_LocateExtPC` (`:706-712`), then `ProjectPointOnCurve` (a 30-iteration Newton on
   `(C(t)−P)·C'(t) = 0`, `:106-149`, division guard `1e-12` at `:137`), then a global
   `Extrema_ExtPC` (`:729-758`), building the corrected `t3d ↦ t2d` table. Monotonicity
   (`curp > previousp + myDeltaMin`) and `curp < myC3dPL − myDeltaMin` are required at every step.
   - Already same-parameter ⇒ `myTolReached = ComputeTolReached(c3d, cons, 2*22)`, done (`:356-361`).
   - **Bad-projection bail**: `aPercentOfBadProj = 0.3`; if fewer than `70%` of the samples survived,
     return `myDone = false` with the *original* pcurve and an honest `myTolReached` (`:352-374`).
3. `ComputeTangents` (`:770-809`): `tangent = |C3d'| / |Σ(p)'|` at both ends; fails if either
   `|Σ(p)'| ≤ 1e-12` (`:774`, `:784-791`, `:799-806`).
4. **Loop over the number of poles** (`:400-480`):
   - `Interpolate` (`:813-853`): a **cubic (degree 3) 1D B-spline reparameterization function**
     `t3d ↦ t2d` through the `nbp+1` sample pairs, with tangent constraints at both ends
     (`ContactOrder(2) = ContactOrder(num_poles−1) = 1`, `:825`);
     `num_poles = nbp + 3`, `num_knots = nbp + 7` (`:403-404`), quadruple end knots (`:827-829`),
     solved by `BSplCLib::Interpolate` (`:845-851`); returns false on `inversion_problem`.
   - `Check` (`:192-266`): evaluate at `2·nbp` parameters spread over
     `[3·pc3d[0] − 2·pc3d[n−1], 3·pc3d[n−1] − 2·pc3d[0]]` clipped to the 2D range
     (`:209-221` — the comment admits this interval "has no sensible grounds" but fixes OCC5898);
     require the reparameterized `tcons` to be **monotone increasing** and inside the range, else
     `tol = Infinite` and fail (`:238-242`); measure `tol = max |C3d(t) − Σ(p(reparam(t)))|`;
     also require the interpolated poles to be sorted (`:254-263`); accept if
     `tol ≤ besttol || tol > 0.8·oldtol` (`:265` — the second clause is a *stagnation* accept).
   - Approximate the composite `p∘reparam` as a real 2D curve with `AdvApprox_ApproxAFunction`
     (`:434-445`): `tol1d = { UResolution(besttol), VResolution(besttol) }` (`:428-430`),
     continuity `min(C1, pcurve continuity)` (`:392-396`), `aMaxDeg = 11`, `aMaxSeg = 1000` (`:433`),
     evaluated through `Approx_SameParameter_Evaluator` which chains the cubic reparam with the
     pcurve, including the derivative `p'(reparam)·reparam'` (`:64-102`).
   - `myTolReached = ComputeTolReached(c3d, cons, 2*22)` (`:454`).
   - **Accept iff `myTolReached < 250 · besttol`** (`aMult = 250.0`, `:456`, comment "To be tolerant
     with discrete tolerance"). Otherwise `IncreaseNbPoles` (`:857-992`): insert a projected midpoint
     wherever the cubic reparam is non-monotone on that interval (`:874-927`), else double the sample
     count (`:939-980`), capped at `myMaxArraySize`.
5. **Failure fallback** (`:482-540`): measure the ORIGINAL pcurve's `TolReached`; build the forced
   approximation once more with `aMaxSeg = 40` (`:518`); keep whichever has the smaller
   `TolReached` (`:532-538`); set `myDone = true` regardless, with an honest `myTolReached`.

`ComputeTolReached(c3d, cons, nbp)` (`:153-188`): `nbp = 2·22 = 44` ⇒ 45 samples;
`aDeviation = 1.05 · sqrt(max squared distance)` (`:184-185`), floored at `Precision::Confusion()`
(`:186`); any evaluation exception or infinite point ⇒ `Precision::Infinite()` (`:165-180`).

### 2.7.6 Validation — `BRepLib_ValidateEdge`

`BRepLib_ValidateEdge.cxx`. `myControlPointsNumber = 22` (ctor, `:32`).
- `Process()` (`:89-98`): exact method (`GeomLib_CheckCurveOnSurface`, a global minimisation) only
  if `myIsExactMethod && mySameParameter`; otherwise `processApprox`.
- `processApprox` (`:102-…`): `anIsProjection` is true if not same-parameter **or** either range
  endpoint differs by more than `PConfusion` (`:113-118`).
  - not a projection ⇒ direct index-to-index comparison at 23 shared parameters (`:121-139`);
  - a projection ⇒ endpoints compared directly, then each of the 21 interior points projected onto
    the other curve **in both directions** with `Extrema_LocateExtPC`; a single non-converged
    extremum sets `myIsDone = false` and aborts with a partial distance.
- `correctTolerance(t) = t + max(BRepCheck::PrecCurve(ref), BRepCheck::PrecSurface(surf))`
  (`:68-77`); `CheckTolerance(t)` returns `correctTolerance(t) > myCalculatedDistance` (`:42-45`).
- `GetMaxDistance()` / `UpdateTolerance()` return `myCalculatedDistance * 1.00001` (`:48-64`),
  grow-only for the update.
- `SetExitIfToleranceExceeded` (`:81-85`) makes the computed distance a **lower bound only**; never
  feed it into a tolerance.

**The pad asymmetry is load-bearing**: `CheckTolerance` pads the *threshold*; `UpdateTolerance` pads
the *distance*. Writing `if (dist > tol) tol = dist*1.00001;` is not the same predicate.

### 2.7.7 The exact measurement used by the boolean

`IntTools_Tools::ComputeTolerance` (`IntTools_Tools.cxx:737-779`): builds
`Adaptor3d_CurveOnSurface(pcurve, surface)` and runs `GeomLib_CheckCurveOnSurface` (a global
optimizer over C1 intervals, parallelizable), then
```
anEps = 1.0 + 1.0e-5 ;  theMaxDist = anEps * aCS.MaxDistance() ;  theMaxPar = aCS.MaxParameter()
```
(`:773-776`), with the explicit comment that the margin exists because a later trim can find a more
precise minimum and would otherwise invalidate the tolerance.

### 2.7.8 Boolean entry points

- `BOPTools_AlgoTools2D::MakePCurveOnFace(F, C3d, t1, t2, C2d&, TolReached2d&, ctx)`
  (`BOPTools_AlgoTools2D.cxx:501-644`):
  ```
  aTR     = Precision::Confusion()  (1e-7)                (:525)
  aMaxTol = 1.e3 * aTR   (1e-4)                           (:526)
  isAnaSurf = ProjLib::IsAnaSurf(surface)                 (:527)
  ```
  - surface of revolution ⇒ direct `ProjLib_ProjectedCurve(surf, curve, max(aTR, TolReached2d))`
    (`:530-540`);
  - else **escalation ladder** (`:548-565`):
    - `10·aTR ≤ TolReached2d ≤ aMaxTol` (or analytic): `aTR = min(aMaxTol, 0.1·TolReached2d)`,
      `aMaxSegments = 100`, `aMaxDist = 1e3·TolReached2d`, and `aBndPnt = PassPoint` when the
      surface is not analytic or `TolReached2d > 1`;
    - `TolReached2d > aMaxTol`: `aTR = min(TolReached2d, 1e3·aMaxTol)`, `aMaxDist = 1e2·aTR`,
      `aMaxSegments = 100`;
    - in both cases the **surface is extended by 1% in each non-periodic direction**
      (`UTrim/VTrim` with `dt = 0.01·span`, `:566-585`) so the projection has room at the borders;
  - `aProjCurv.Perform(curve)` → `ProjLib::MakePCurveOfType` (`:591-592`), which *throws
    `Standard_NotImplemented`* for `GeomAbs_BezierCurve` / `GeomAbs_OtherCurve`
    (`ProjLib.cxx:207-211`);
  - null result ⇒ one retry at `aTR = max(TolReached2d, aMaxTol)` (`:596-602`), then
    `throw Standard_ConstructionError` (`:604-607`);
  - `AdjustPCurveOnSurf` (`:613`);
  - **range repair** (`:616-631`): if the pcurve's own range does not cover `[aT1,aT2]`,
    `GeomLib::SameRange(PConfusion, ...)` onto `[aT1,aT2]` — G2 again;
  - **final measurement** `IntTools_Tools::ComputeTolerance(C3d, C2d, S, aT1, aT2, aTolR, aT)`,
    and `TolReached2d = max(TolReached2d, aTolR)` (`:633-643`).
- `BOPTools_AlgoTools::MakePCurve(E, F1, F2, IC, bPC1, bPC2, ctx)`
  (`BOPTools_AlgoTools.cxx:1657-1724`): prefers the pcurves the FF intersector already produced
  (`aIC.FirstCurve2d()` / `SecondCurve2d()`, `:1687-1698`) and only calls
  `BuildPCurveForEdgeOnFace` when they are null (`:1700-1709`); adjusts for periodicity
  (`:1711-1718`); `UpdateEdge(aE, aC2DA, aFFWD, aTolE)`; and finally, once for the edge,
  **`BRepLib::SameParameter(aE)`** (`:1724`), whose default tolerance is `1.0e-5`
  (`BRepLib.hxx:161-162`).
- The edge/face acceptance predicate that consumes the resulting tolerances is **asymmetric**:
  `IntTools_EdgeFace.cxx:529-549` —
  ```
  aTolF = tol(F) + fuzz/2 ;  aTolE = tol(E) + fuzz/2
  if (edge curve is BSpline or Bezier):
      if (aTolE/aTolF > 100 || aTolF/aTolE > 100)  criteria = max(aTolE, aTolF)
      else                                          criteria = 1.5*aTolE + aTolF
  else                                              criteria = aTolE + aTolF
  ```

## 2.8 `BuildCurve3d` — the reverse direction (pcurve ⇒ 3D curve)

`BRepLib.cxx:301-456`. Needed at import: a STEP edge may legally carry pcurves and no 3D curve.
- If a 3D curve exists, return true immediately (`:321-325`).
- If the edge is not same-range, run `SameRange` first (`:330-333`).
- Search the reps for a **plane** (`:342-357`); if found, the 3D curve is **exact**:
  `GeomLib::To3d(P->Position().Ax2(), PC)` (`:362`), and the edge is updated with tolerance
  **`0.0`** (`:371`), then the range is restored (`:372-373`).
- Otherwise take the first pcurve+surface, build `Adaptor3d_CurveOnSurface`, and approximate with
  `GeomLib::BuildCurve3d(Tolerance, cons, f, l, C&, maxDev, avgDev, Continuity, MaxDegree,
  evaluateMaxSegment(MaxSegment, cons))` (`:421-430`), where `evaluateMaxSegment` (`:273-297`)
  returns `30 + max(surface knots, pcurve knots)` when 0 was passed.
  The stored tolerance is `max(BRep_Tool::Tolerance(E), Tolerance)` — note `max_deviation` from the
  approximation is **discarded** (`:434-435`, the original line is commented out).
- If the edge had exactly one pcurve, it is declared `SameParameter(true)` by construction
  (`:441-448`).
- `BuildCurves3d(S, tol)` default tolerance `1.0e-5` (`:463`).

`BRepLib::UpdateEdgeTol(E, MinToleranceRequested, MaxToleranceToCheck)` (`:493-687`) re-measures an
edge against **all** its other representations: `GCPnts_QuasiUniformDeflection` sampling at
deflection `MinToleranceRequested·100` (`factor = 100.0`, `:501`), clamped to
`[30, 90]` points (`:499`, `:591-598`); `EvalMaxParametricDistance` when same-parameter, else
`EvalMaxDistanceAlongParameter` (`:651-676`); `max_distance *= safe_factor = 1.4` (`:503`, `:677`);
final `TE->Tolerance(edge_tolerance)` — again a **hard set** (`:685`).

## 2.9 Constants — every value, with its site

| constant | value | site |
|---|---|---|
| `Precision::Confusion` | `1e-7` | `Precision.hxx:165` |
| `Precision::Angular` | `1e-12` | `Precision.hxx:123` |
| `Precision::Intersection` | `1e-9` | `Precision.hxx:220` |
| `Precision::Approximation` | `1e-6` | `Precision.hxx:235` |
| `Precision::PConfusion` | `1e-9` | `Precision.hxx:334` |
| `Precision::PApproximation` | `1e-8` | `Precision.hxx:346` |
| `Precision::Parametric(P)` | `0.01·P` | `Precision.hxx:328` |
| ProjLib pole-trim fraction `eps` | `0.01` of the curve span | `ProjLib_ProjectedCurve.cxx:380` |
| sphere-fallback `TolConf` | `max(R·1e-5·π, |R − maxdist|)` | `:425`, `:435` |
| `ComputeTolU/V` periodic cap | `min(Resolution(tol), 0.01·period)` | `:59`, `:72`; `ComputeApprox:1125,1138` |
| `IsoIsDeg` sampling | 10 steps along the iso | `:95-124` |
| `ExtendC2d` parallel test | `100·Precision::Angular()` | `:213` |
| free-form projector `aMaxDist` | `100·myTolerance` (or `myMaxDist`) | `:618-622` |
| `Approx_CurveOnSurface` (revolution/other) | MaxDegree 14, MaxSeg 16, C1 (C0 if PassPoint) | `:642-659` |
| `ProjLib_ComputeApprox` degrees | min 5, max 8 (simple) / 10 | `:1285-1293` |
| `ProjLib_ComputeApprox` max segments | 1000 | `:1304` |
| `aTol2d` | `max(√(TolU²+TolV²), PConfusion)` | `:1320-1321` |
| pole/knot sanity caps | `(0, 100000]` | `:1356-1363` |
| `Function_ComputeStep` | `nbp = L/(R·π/4)+1`, ≥3; `Step ≤ 0.1` | `:239-258` |
| sphere seam-count tolerance | `1e-10` | `:615`, `:774` |
| `Approx_ComputeCLine` MAXSEGM | 1000 | `Approx_ComputeCLine.gxx:26` |
| hang-check `TolU` | `max(span·1e-3, Confusion)`; else `max(span·1e-5, PApproximation)` | `:104-111` |
| hang-check window | 10 cuts, stop if cuts > improvements+1 | `:113`, `:190-195` |
| least-squares points | `min(2·deg+1, 24)` Gauss points | `:264`, `:287`, `:355` |
| inverse-degree-order flip ratio | 0.05 of the whole span, `aMaxDeg = 8` | `:265-279` |
| `NCONTROL` (SameParameter) | 22 ⇒ 23 samples | `BRepLib.cxx:1294` |
| `ComputeTol` out-of-chart band | `0.01·(ul−uf)` | `BRepLib.cxx:1083` |
| `ComputeTol` outlier gate | bucket at `1.0`; `ana` iff `N1>N2, N2≠0, 0 < 100·N2/(N1+N2) < 10` | `:1153-1182` |
| `ComputeTol` safety / floor | `×1.5`, floor `1e-7` | `:1185-1186` |
| `BigError` | `1e10` | `:1349` |
| `TolSameRange` | `max(GAC.Resolution(tol), PConfusion)` | `:1378` |
| `TolConf2d` | `max(min(URes(tol), VRes(tol)), PConfusion)` | `:1406-1409` |
| `EvalTol` | 5 samples at `i/6`, need `ok > 2` | `:1048-1065` |
| `Tol2dbail` | `max(min(min(UResbail,VResbail), 0.1·min pole gap), TolConf2d)` | `:1450-1467` |
| bad-knot `critratio` | 10, only when `error > max(1e-3, tol)` | `:1541-1576` |
| `Approx_CurvilinearParameter` | tol `max(1e-3,tol)`, cont ≤ C2, maxdeg 14 if deg 1, 10 segments | `:1593-1603` |
| ANA∧BSP special tol | `1e-7` | `:1628` |
| final edge tol floor | `Precision::Confusion` | `:1731` |
| `GetEdgeTol` | 23 samples, `×1.05`, floor `Epsilon(coord²)` | `:770-792` |
| `UpdateEdgeTol` | deflection `MinTol·100`, 30..90 points, `safe_factor 1.4` | `:499-503`, `:677` |
| `BuildCurve3d` defaults | tol `1e-5`; MaxSegment `30 + max knots` | `:463`, `:295` |
| `Approx_SameParameter` samples | 22 (⇒23 pairs); tolReached over `2·22+1 = 45` | `.hxx:161`; `.cxx:358` |
| `Approx_SameParameter` max array | 1000 | `.hxx:162` |
| `myDeltaMin` | `Precision::PConfusion` | `.cxx:274` |
| bad-projection quota | 0.3 | `.cxx:353` |
| interpolation sizes | degree 3; poles `nbp+3`, flat knots `nbp+7` | `.cxx:403-404` |
| composite re-approx | `tol1d = {URes, VRes}(besttol)`, maxdeg 11, maxseg 1000 (fallback 40) | `.cxx:428-433`, `:518` |
| discrete accept ratio | `TolReached < 250 · besttol` | `.cxx:456` |
| `ComputeTolReached` | `×1.05`, floor Confusion | `.cxx:184-186` |
| Newton in `ProjectPointOnCurve` | 30 iterations, divide guard `1e-12` | `.cxx:146`, `:137` |
| `ComputeTangents` guard | `1e-12` | `.cxx:774` |
| `ValidateEdge` | 22 control points; `×1.00001`; `+max(PrecCurve, PrecSurface)` | `ValidateEdge.cxx:32,48-64,68-77` |
| `IntTools_Tools::ComputeTolerance` | `×(1 + 1e-5)` | `IntTools_Tools.cxx:773-774` |
| `MakePCurveOnFace` ladder | `aTR=1e-7`, `aMaxTol=1e-4`, maxSeg 100, maxDist `1e3·Tol` / `1e2·aTR`, surface extension 1% | `AlgoTools2D.cxx:525-585` |
| `AdjustPCurveOnSurf` | `aDelta = PConfusion`; cylinder `dFi = maxTolEdge/R` | `AlgoTools2D.cxx:263`, `:296-305` |
| E/F criterion (spline edge) | `1.5·tolE + tolF`, escape `max` when ratio > 100 | `IntTools_EdgeFace.cxx:529-549` |
| `BRepCheck::PrecCurve` | `RealEpsilon()`, except ellipse: `max Epsilon(|centre|, Rmaj, Rmin)` | `BRepCheck.cxx:70-99` |
| `BRepCheck::PrecSurface` | `RealEpsilon()`, except cone: `max Epsilon(|apex|, RefRadius)` | `BRepCheck.cxx:103-130` |

## 2.10 Where OCCT itself gives up (documented, not invented)

1. **Oblique conic on a cylinder / cone / sphere / torus** → least-squares fit.
   `ProjLib_Cylinder.cxx:161-165` (empty body + apology), `ProjLib_Cone.cxx:167-180`,
   `ProjLib_Sphere.cxx:181-199`, `ProjLib_Torus.cxx:175-193` — all route to
   `ProjLib_Projector::Project`, which sets `GeomAbs_OtherCurve`
   (`ProjLib_Projector.cxx:144-175`), and `ProjLib_ProjectedCurve.cxx:717-786` then runs
   `ProjLib_ComputeApprox`.
2. **Spline/Bezier curve on any non-plane surface** → fit. `ProjLib_ProjectedCurve.cxx:262-266`.
3. **Torus circle validity is not verified.** `ProjLib_Torus.cxx:172` sets `isDone = true`
   unconditionally in both branches.
4. **`ProjLib::MakePCurveOfType` throws** on `GeomAbs_BezierCurve` and `GeomAbs_OtherCurve`
   (`ProjLib.cxx:207-211`); `BOPTools_AlgoTools2D::MakePCurveOnFace` converts a null result into a
   `Standard_ConstructionError` (`:604-607`). There is no graceful degradation below that.
5. **`Approx_SameParameter` accepts 250× its own target** (`:456`) and, on total failure, keeps
   whichever of {original pcurve, forced approximation} deviates less and reports `myDone = true`
   (`:482-540`). The honest output is `TolReached`, not a guarantee.
6. **`Check`'s validity interval "has no sensible grounds"** — OCCT's own comment,
   `Approx_SameParameter.cxx:204-208`.
7. **`ProjLib_ComputeApprox`'s tolerance back-conversion is an assumption**, stated as such at
   `:1410-1414`.
8. **`BuildCurve3d` discards the measured approximation deviation** and stores
   `max(existing tol, requested tol)` instead (`BRepLib.cxx:433-435`).
9. **`ComputeTol`'s sphere V-couture bookkeeping is patched out**: `Function_SetUVBounds` computes
   `myV1/myV2` carefully and then overwrites them with `±1e100` under a comment
   `// box+sphere` (`ProjLib_ComputeApprox.cxx:808-811`).
10. **Plane pcurves are recomputed on every query** (`BRep_Tool.cxx:368-372` → `CurveOnPlane`), so the
    "cache" is a projection each time; only the stored-rep path is O(1).

---

# 3. DATA STRUCTURES AND C++ DECLARATIONS FOR OUR PORT

New files: `src/chart.h/.cpp` (analytic charts + exact inversion), `src/pcurve.h/.cpp` (the
functional pcurve), `src/sameparameter.h/.cpp` (measurement + repair). Nothing below stores a
sampled polyline as the primary representation.

```cpp
// ─── src/chart.h ──────────────────────────────────────────────────────────────
namespace session_cpp {

enum class ChartKind : uint8_t { Plane, Cylinder, Cone, Sphere, Torus, Freeform };

// An analytic surface WITH its canonical frame. This is the missing half of
// RecogSurface (src/intersection.cpp:2373-2379): a chart needs an X reference
// direction, or "u = 0" is undefined and every pullback must table-invert.
struct Chart {
    ChartKind kind = ChartKind::Freeform;
    Point     loc;                 // plane origin / cyl axis point / cone apex-ref / sphere centre / torus centre
    Vector    xd, yd, zd;          // orthonormal, right-handed; yd = zd × xd
    double    r0        = 0.0;     // cylinder R; cone RefRadius at v=0; sphere R; torus MajorRadius
    double    r1        = 0.0;     // torus MinorRadius
    double    half_angle= 0.0;     // cone semi-angle (signed)
    // Chart window actually used by the face (periods handled separately).
    double    u0=0, u1=0, v0=0, v1=0;
    bool      u_periodic=false, v_periodic=false;
    double    u_period=0, v_period=0;

    // Exact forward map.                      ElSLib::*Value
    Point  value(double u, double v) const;
    void   d1(double u, double v, Point& P, Vector& Su, Vector& Sv) const;

    // Exact closed-form inversion.            ElSLib::*Parameters
    // `hint_u` disambiguates the periodic branch (pass the previous point's u).
    void   invert(const Point& P, double& u, double& v, double hint_u = 0.0) const;

    // |Su|, |Sv| in closed form — the ONLY sanctioned uv<->3d metric conversion.
    double du_scale(double u, double v) const;   // plane 1, cyl r0, cone r0+v*sin(a), sphere R*cos v, torus R+r*cos v
    double dv_scale(double u, double v) const;   // plane 1, cyl 1,  cone 1,            sphere R,      torus r

    // uv tolerance from a 3D tolerance, AT A POINT. Never domain-relative.
    double u_tol(double tol3d, double u, double v) const { return tol3d / du_scale(u,v); }
    double v_tol(double tol3d, double u, double v) const { return tol3d / dv_scale(u,v); }

    bool   orthogonal() const { return kind != ChartKind::Freeform; }  // Su·Sv == 0, proven in §2.3
};

// Recognition happens ONCE, at ingest, and the result is stored on the surface.
// (Today it is re-derived by fitting on every call: src/intersection.cpp:2668-2720.)
bool recognize_chart(const NurbsSurface& s, double tol3d, Chart& out);

} // namespace session_cpp
```

```cpp
// ─── src/pcurve.h ─────────────────────────────────────────────────────────────
namespace session_cpp {

enum class PCurveKind : uint8_t {
    Line2d,          // exact: plane×line, chart-aligned circles/generators (OCCT's cases)
    Conic2d,         // exact: plane×{circle,ellipse,parabola,hyperbola}
    Nurbs2d,         // exact: plane×spline (control-point image), or an imported pcurve
    CylSinusoid,     // exact [BEYOND OCCT]: v(u) = v0 + A cos(u - phi)
    ConeRational,    // exact [BEYOND OCCT]: v(u) = -(D + r0*K(u)) / (sin(a)*K(u) + C*cos(a))
    SphereBranch,    // exact [BEYOND OCCT]: v(u) = psi(u) +- acos(S / hypot(K(u), C))
    TorusBranch,     // exact [BEYOND OCCT]: v(u) = psi(u) +- acos((-D - R*K(u)) / (r*hypot(K(u), C)))
    Pullback,        // exact pointwise: p(t) = invert(C3d(t)); the general functional form
    FitNurbs2d       // approximate; ONLY legal when chart.kind == Freeform
};

enum class PCurveProvenance : uint8_t { ExactAffine, ExactChart, ExactPullback, Fit, Imported };

struct PCurve {
    PCurveKind       kind = PCurveKind::Pullback;
    PCurveProvenance prov = PCurveProvenance::ExactPullback;

    // Parameter domain — ALWAYS the owning edge's [f,l]. G2 is structural, not checked.
    double f = 0.0, l = 0.0;

    // Closed-form payloads (one is live per kind).
    // Line2d/Conic2d/Nurbs2d/FitNurbs2d:
    NurbsCurve c2d;
    // CylSinusoid: v0, amp, phi.  ConeRational/Sphere/TorusBranch: plane (A,B,C,D) in chart frame
    // plus `branch` in {+1,-1} and the u-interval on which the branch is real.
    double A=0,B=0,C=0,D=0, v0=0, amp=0, phi=0;
    int    branch = +1;
    // Pullback: reference to the shared 3D curve + the period offsets of THIS piece.
    const NurbsCurve* c3d = nullptr;
    int    ku = 0, kv = 0;             // period-cell indices; the piece lives in cell (ku,kv)

    const Chart* chart = nullptr;

    // Exact evaluation and derivative. Never sampled.
    gp2  value(double t) const;
    gp2  d1(double t) const;           // chain rule via chart->d1 (orthogonal charts only)

    // Endpoints are STORED, not recomputed (G4): these are the values other trims must reuse.
    gp2  uv_first, uv_last;

    // Export only. `tol3d` is the accepted deviation of the spline from the exact curve.
    NurbsCurve to_nurbs(double tol3d, double& tol_reached) const;
};

// Exact seam crossings of a pullback: parameters t where p_u(t) hits a period boundary.
// Solved on the 3D curve (scalar root find on an exact function), NOT on a sampled polyline,
// so both operands obtain bitwise identical values.
std::vector<double> seam_crossings(const Chart& ch, const NurbsCurve& c3d, double f, double l);

// Split a pullback at its seam crossings; each piece gets its exact cell indices and its
// boundary endpoints ASSIGNED to the period constant (never left as the computed value).
std::vector<PCurve> split_at_seams(const Chart& ch, const NurbsCurve& c3d, double f, double l);

} // namespace session_cpp
```

```cpp
// ─── src/sameparameter.h ──────────────────────────────────────────────────────
namespace session_cpp {

// One representation of an edge on one chart. Mirrors BRep_CurveOnSurface /
// BRep_CurveOnClosedSurface: the seam case is ONE record with TWO pcurves.
struct EdgeRep {
    int    surface_index = -1;
    PCurve pc;                       // primary image
    bool   closed_surface = false;
    PCurve pc2;                      // second image, valid iff closed_surface  (G6)
};

// The shared edge. ONE 3D curve, N representations, one tolerance, two flags.
struct EdgeGeom {
    NurbsCurve           c3d;
    double               f = 0.0, l = 0.0;
    double               tol = 0.0;         // measured, model space (G8)
    bool                 same_parameter = false;   // FALSE until measured (never a default true)
    bool                 same_range     = false;
    int                  v_start = -1, v_end = -1;
    std::vector<EdgeRep> reps;
};

// ── measurement ───────────────────────────────────────────────────────────────
struct Deviation {
    double dist3d      = 0.0;   // max |C3d(t) - S(p(t))| over the samples
    double out_of_chart= 0.0;   // 3D-equivalent of any excursion beyond the 1% band
    int    n_outliers  = 0;     // samples voted spurious
    bool   infinite    = false;
};

// Port of BRepLib.cxx:1070-1188, verbatim: 23 samples, 1% out-of-chart band converted
// through 1/resolution, <10% outlier vote, x1.5, floor 1e-7.
Deviation compute_tol(const NurbsCurve& c3d, double f, double l,
                      const PCurve& p, const Chart& ch, int nbp = 22);
double    compute_tol_value(const Deviation& d);          // 1.5 * value, floor 1e-7

// Port of BRepLib_ValidateEdge: 22 control points, index-to-index when same-parameter,
// two-way projection otherwise; distance x1.00001; threshold padded by prec(curve/surface).
struct ValidateResult { double max_dist = 0.0; bool done = false; bool early_exit = false; };
ValidateResult validate_edge(const EdgeGeom& e, const EdgeRep& r, const Chart& ch,
                             double exit_if_exceeded = -1.0);

// ── repair ────────────────────────────────────────────────────────────────────
// Affine (or spline) renormalisation of a pcurve onto [f,l]. Port of GeomLib::SameRange.
bool same_range(PCurve& p, double from_f, double from_l, double to_f, double to_l, double tol);

// Cubic reparameterization engine. Port of Approx_SameParameter::Build. ONLY needed for
// IMPORTED pcurves; our own pullbacks are same-parameter by construction.
struct ReparamResult {
    bool   already_same = false;
    bool   done         = false;
    double tol_reached  = 0.0;
    PCurve curve2d;
};
ReparamResult reparameterize_same(const NurbsCurve& c3d, double f, double l,
                                  const PCurve& p, const Chart& ch, double tol);

// The driver. Returns the new edge tolerance, or -1 if nothing was measurable.
// Semantics deliberately match OCCT: on success tol is SET (may shrink); vertices are
// only ever GROWN by the caller.
double same_parameter(EdgeGeom& e, const std::vector<Chart>& charts, double tol);

// The other half of OCCT's contract, without which the SET above is unsound.
void   harmonize_tolerances(BRep& shape);   // tol(V) >= tol(E) >= tol(F), + 2*Epsilon(tol)

} // namespace session_cpp
```

**Storage rule for our `BRep`**: `m_curves_2d[i]` becomes a `PCurve`, not a `NurbsCurve`. A `PCurve`
of kind `Nurbs2d`/`FitNurbs2d` still holds a `NurbsCurve`, so the change is additive; every consumer
that today calls `pc.point_at(t)` gets `pc.value(t)` with the same meaning **except** that `t` now
means the *edge* parameter (G2), which is the behavioural change the port is for.

---

# 4. WHAT OUR CODE DOES TODAY, AND EXACTLY WHERE IT DIVERGES

## 4.1 The pullback

| what | our site | OCCT / target | divergence |
|---|---|---|---|
| Plane × spline pullback | `src/brep.cpp:916-938` `project_curve_to_uv` — affine inverse applied to control points, rational weights preserved | `ProjLib_ComputeApprox.cxx:1197-1252` | **equivalent and correct**, but it is only reachable from `BRep::from_nurbscurves` (`:899`), i.e. construction, not from the boolean |
| Plane × conic | `src/intersection.cpp:3222-3248` (PLANE arm of `analytic_pcurve`) | `ProjLib_Plane.cxx:101-169` | equivalent (control-point image preserves the exact rational conic) |
| Cylinder | `src/intersection.cpp:3253-3280` | `ProjLib_Cylinder.cxx:122-156` | requires **constant height** (`hmax-hmin > 1e-5·|h1-h0|` ⇒ bail, `:3272`) **and a full wrap** (`:3274`); otherwise returns an invalid curve |
| Sphere | `:3285-3312` | `ProjLib_Sphere.cxx:162-177` (isoV only) | requires **constant latitude** (`:3300`) and full wrap (`:3301`); the iso-U great-circle case has no arm at all |
| Cone | `:3314-3338` | `ProjLib_Cone.cxx:108-165` | requires a coaxial circle (`:3328`) and full wrap (`:3329`) |
| Torus | `:3345-3395` | `ProjLib_Torus.cxx:81-173` | requires constant minor angle (`:3367`) and full wrap (`:3368`); the iso-U meridian case is explicitly noted as falling back (`:3343-3344`) |
| **oblique everything** | falls to `analytic_sphere_pullback` (`:3406-3559`), `analytic_cone_pullback` (`:3567-3675`), `analytic_torus_pullback` (`:3682-…`), else `Closest::surface_curve` | OCCT: `ProjLib_ComputeApprox` fit | **§2.5 Layer 2 says no fit is needed at all** |

Our "analytic pullbacks" are closer to right than OCCT's fit, but they are still approximations,
for three structural reasons:

1. **They emit degree-1 polylines.** `NurbsCurve::create(false, 1, seg)` at
   `src/intersection.cpp:3557`, `:3666`, `:3673` (and the sphere/torus equivalents). The pcurve is a
   sampled chord chain at `n = max(cv_count·8, 120)` (`:3618`) — a sag error, and 4000-CV polylines
   downstream (`src/brep.cpp:1589-1592`).
2. **They table-invert the NURBS parameterization.** `:3435-3444` (sphere), `:3594-3616` (cone) build
   a 128-entry `u ↦ longitude` table and binary-search it, then Newton-polish twice (`:3629-3643`).
   This exists only because the surface is stored as a NURBS whose `u` is the rational-quadratic
   circle parameter, not the angle. With a `Chart` (§3) the inverse is `atan2`, exactly, with no
   table, no Newton, no polish.
3. **They destroy SameRange.** `NurbsCurve::create` rescales the domain to the **arc length** of the
   control polygon (`src/nurbscurve.cpp:29-48`) — for a pcurve, the UV chord length. So the pcurve's
   parameter has no relation whatsoever to the 3D curve's parameter. `Closest::surface_curve` is
   worse: it ends with `pcurve.set_domain(0.0, 1.0)` (`src/closest.cpp:713`). **G2 is violated by
   construction for every pcurve our kernel produces**, which is why nothing downstream can rely on
   G1 and everything re-projects.

## 4.2 The fit fallback — where the exact conic is thrown away

`Closest::surface_curve` (`src/closest.cpp:356-718`), reached from
`Intersection::cut_curves_on_surface` at `:5661`, `:5668`, `:5674`, `:5685`:

- sampling `n0 = max(16, 4·span_count)` (`:467`), warm-started point inversion per sample
  (`:471-495`), adaptive bisection to depth 8 / 4096 samples (`:501-535`);
- tolerances: `step = min(du,dv)·0.25` (`:398`), `fit_tol = tolerance>0 ? tolerance : step·(…)`
  (`:399`), `reject_tol = 100·fit_tol` (`:400`), `on_surf_tol = 0.05·corner_diag` (`:405-407`) —
  all **UV/domain relative**;
- the fit: `target_cvs = max(8, total_turning/0.5 + 6)` (`:686`), then
  **`NurbsCurve::create_fitted(pts_uv, target_cvs, 3, piece_loop)`** (`:691`) — a **non-rational
  cubic least-squares fit** (`src/nurbscurve.cpp:353+`), doubled up to 5 times until
  `max_dev < fit_tol_uv = step` (`:689-701`);
- fallbacks: `create_interpolated` (`:703-706`), then a degree-1 polyline (`:710`);
- finally `set_domain(0.0, 1.0)` (`:713`).

A non-rational cubic **cannot represent a circle**. Its best uniform approximation to a quarter
circle has relative error ~2.7e-4 with 4 CVs; refining `target_cvs` reduces it but the *criterion*
is a UV deviation against the sampled polyline, not a 3D deviation against the exact conic — so the
measured error is bounded by the sampling, not by the truth. That is precisely the reported symptom:
*exact circles computed, then a fitted cubic used, ~1e-4 relative error independent of geometry.*

## 4.3 Domain-relative tolerances (breaks G8)

| site | expression | effect of a 4× padded UV domain |
|---|---|---|
| `src/nurbssurface_trimmed.cpp:574` | `samp_tol = max(range_u, range_v) * 2e-5` | 4× coarser trim sampling |
| `src/nurbssurface_trimmed.cpp:565-569` | `snap_uv = max(1e-9, tolerance/uv_to_3d)` or `min(range_u,range_v)*1e-7` | boundary snap radius changes |
| `src/nurbssurface_trimmed.cpp:1653` | `samp_tol = max(range_u, range_v) * 1e-3` | 4× |
| `src/brep.cpp:4350` | `eps_border = min(du_span, dv_span) * 2e-3` | 4× |
| `src/brep.cpp:4280` | `scaf_forced_eps = clamp(min_rangeF*1e-2, …)` | 4× |
| `src/closest.cpp:398-407` | `step`, `fit_tol`, `reject_tol`, `on_surf_tol` | 4× |
| `src/brep_section.cpp:221` | `samp_tol_lp = max(1e-9, max(bx1-bx0, by1-by0)*2e-4)` | 4× |

All of these must become `chart.u_tol(tol3d, u, v)` / `chart.v_tol(tol3d, u, v)` evaluated at the
point of use. `Chart::du_scale/dv_scale` (§3) makes that a closed-form division, so there is no cost
argument for keeping the domain-relative form.

## 4.4 Same-parameter

- **There is no same-parameter subsystem.** The only artefact is
  `BRep::sameparameter_planar_pcurves()` (`src/brep.cpp:7480-7568`), which rebuilds *the planar
  side* of a 2-trim mixed planar/curved section edge by affinely projecting the *curved* trim's
  lifted boundary. Its own header comment states the problem exactly: the two trims "still integrate
  their OWN pcurve copies, which differ at the section-fit tolerance (~2e-4): the volume error is
  FIRST order in that mismatch" (`:7482-7484`). It is **gated off by default**
  (`src/brep.cpp:11318-11322`, `SESSION_SAMEPARAM`), with the note that it only helps once *all*
  section arcs are pullback-quality on both surfaces.
  Additional restrictions that make it a special case, not a mechanism: exactly one planar side
  (`:7500`), curved pcurve must be a dense non-rational degree-1 polyline with ≥8 CVs (`:7516`), the
  lifted points must lie within `5e-3·surface_size` of the plane (`:7534`), the plane patch must be
  affine degree-1×1 (`:7545-7548`), and endpoints must land within `2e-2·span_uv` of the old ones
  (`:7564`).
- The closest thing to a *measurement* is `scaf.max_devA/max_devB`
  (`src/brep_section.cpp:1197-1198`): `max |A(uvA[i]) − p3[i]|` over chain nodes. This is the right
  quantity but it is (a) computed only at chain nodes, (b) global to the scaffold rather than
  per-edge, (c) diagnostic only — nothing consumes it as a tolerance.
- The "SameParameter guard" at `src/brep.cpp:6028-6100` is a *repair for a different failure*: it
  detects an edge whose 3D curve does not span what its trims' pcurves span and **re-lifts the 3D
  curve from the pcurve** — i.e. it moves the 3D geometry to match a pcurve, which is the opposite
  of OCCT's law (§2.7.1: never move geometry; adjust parameterization).

## 4.5 The shared-chain representation we already have (and should keep)

`SectionSegment` (`src/brep_section.h:14-24`) carries `p3[i]`, `uvA[i]`, `uvB[i]`
index-corresponded, built by sampling A's pcurve, seeding B's at the same normalized fraction, and
Newton-correcting the 7-unknown system (`src/brep_section.cpp:483-507`, `correct7` at `:491`).
**This is a discrete same-parameter representation and it is stronger than OCCT's reprojection** —
at the chain nodes. The port keeps it as the *pave/topology* layer and replaces the geometry layer
underneath: `p3` becomes the exact 3D curve, `uvA/uvB` become exact `PCurve`s, and the node
correspondence becomes exact for all `t`, not just at nodes.

## 4.6 Analytic identity is re-derived by fitting, on every call

`recognize_surface_impl` (`src/intersection.cpp:2675-2720`) fits a plane/sphere/cylinder/cone/torus
from a 5×5 sample grid each time it is called, and `RecogSurface`
(`src/intersection.cpp:2373-2379`) stores **no reference X direction** — so there is no canonical
`u = 0`, and every pullback must table-invert the NURBS parameterization (§4.1 point 2). This
violates ARCHITECTURE_v2 law 3 ("Analytic identity survives I/O and transforms… Never re-derive by
fitting what is already known exactly", `kb/ARCHITECTURE_v2.md:30-32`) and is the root cause that
makes exact pcurves impossible in the current representation.

---

# 5. ACCEPTANCE TESTS

Every test is oracle-free: the invariant is checkable from the operands alone. Where an analytic
answer exists it is given, so the test can also assert the *value*.

**T1 — Plane pullback is exactly affine.**
Operands: a circle of radius 5 in an arbitrarily rotated plane; the plane face.
- assert `pcurve.kind == Conic2d`, `provenance == ExactAffine`;
- assert the 2D conic is a circle of radius 5 (radius error `≤ 1e-15·5`);
- **invariant**: `max over 1000 t of |Σ(p(t)) − C(t)| ≤ 4·eps·|C|`;
- **invariant (G2)**: `p.f == C.f && p.l == C.l` bitwise.

**T2 — Oblique cylinder section is a sinusoid, not a fit.**
Cylinder `R = 1`, axis `+Z`; plane `z = 0.3 + 0.5x`. Section is an ellipse with semi-axes
`1` and `√1.25`.
- analytic answer: `v(u) = 0.3 + 0.5·cos u`;
- assert `pcurve.kind == CylSinusoid`, `provenance == ExactChart`;
- assert `max_u |p_v(u) − 0.3 − 0.5 cos u| ≤ 1e-15`;
- **invariant**: `max_t |Σ(p(t)) − C(t)| ≤ 1e-13`;
- **regression bound**: the current kernel returns a `FitNurbs2d` with residual ≥ 1e-5 here
  (`src/intersection.cpp:3272` bails, `src/closest.cpp:691` fits).

**T3 — Oblique sphere circle (not a parallel).**
Sphere `R = 1` at the origin; plane `x + y + z = 0.5`.
- analytic: the section is a circle of radius `√(1 − 0.25/3)` centred at `(1/6)(1,1,1)`;
- with `A=B=C=1/√3, D=−0.5/√3` in the chart frame,
  `v(u) = atan2(C, K(u)) ± acos( (−D)/√(K(u)²+C²) )`, `K(u) = A cos u + B sin u`;
- assert `pcurve.kind == SphereBranch`; assert both branch arcs are produced and meet at the two
  `u` roots of `K(u)² + C² = D²` computed independently in closed form;
- **invariant**: residual `≤ 1e-13`; the two arcs' shared endpoints are **bitwise equal**.

**T4 — Cone conic families.**
Cone semi-angle 30°, apex at origin, axis `+Z`, `R₀ = 0`.
Three planes chosen so the section is an ellipse, a parabola and a hyperbola.
- assert `v(u) = −(D + R₀K(u))/(sinα·K(u) + C cosα)`; for the parabola/hyperbola cases assert the
  branch split occurs exactly at the roots of `sinα·K(u) + C cosα = 0`;
- **invariant**: residual `≤ 1e-13` on every emitted branch; no branch crosses its own pole.

**T5 — Torus spiric.**
Torus `R = 3, r = 1`, axis `+Z`; plane `x = 1.5` (perpendicular to the tube, off-axis).
- analytic: `v(u) = atan2(C r, r K(u)) ± acos((−D − R K(u))/(r√(K(u)²+C²)))`; the section exists only
  where the acos argument is in `[−1,1]` — assert the emitted `u` intervals equal the closed-form
  ones to 1e-14;
- **invariant**: residual `≤ 1e-13`; each loop closes exactly (first/last UV bitwise equal).

**T6 — Rotation invariance (the headline defect).**
For each of the 5 chart types × {plane cut, cylinder cut, sphere cut}: apply 20 random rigid motions
(random axis, random angle) to *both* operands.
- **invariant**: the pcurve residual and the derived `tolE` are identical across all 20 poses to
  `1e-15` relative. Any dependence on pose means a recogniser or a chart frame is pose-sensitive.

**T7 — UV-domain padding invariance (breaks today).**
Take any operand; reparameterize its surface so the UV domain is 4× larger (pure knot rescale, same
point set). Re-run the split.
- **invariant**: identical face count, identical naked-edge count, identical `tolE` per edge, and
  the pcurve residuals bit-identical.
- Today: a STEP round-trip that returns `u ∈ [−0.04, 4.04]` instead of `[0,1]` inflates
  `samp_tol`, `eps_border`, `snap_uv`, `scaf_forced_eps` 4× (§4.3) and the box cell degrades from
  6 faces / 0 naked to 2 faces / 8 naked (`kb/ARCHITECTURE_v2.md:150`).

**T8 — Single-operand split, edge identity (breaks today: 32 naked of 36).**
Split ONE operand alone on a padded UV domain with a verified-identical UV arrangement.
- **invariant**: `naked_edges == 0`; and every shared wire junction satisfies G4 (bitwise identical
  UV endpoints) — assert with `==` on doubles, not with a tolerance. This is the test that
  distinguishes "identity by shared entity" from "identity by coordinate coincidence".

**T9 — Seam splitting is exact and symmetric.**
A circle on a cylinder crossing the `u = 0` seam, and the same circle approached from the other
operand.
- **invariant**: exactly 2 pcurve pieces; the shared crossing `u` equals the period constant
  bitwise; and the crossing parameter `t` computed from operand A equals the one from operand B
  bitwise (both solve the same scalar root on the same exact 3D curve).

**T10 — Pole degeneracy.**
A great circle on a sphere passing through both poles; a generator line through a cone apex.
- **invariant**: the wire closes (G4) and the pcurve reaches `v = ±π/2` (resp. the apex) with a `u`
  taken from the *tangent direction*, not from `atan2(0,0)`; face area computed from the pcurve
  equals the analytic hemisphere/sector area to `1e-12` relative.

**T11 — Same-parameter enforcement on a foreign pcurve.**
Construct an edge with a correct 3D circle and a deliberately mis-parameterized pcurve
(`t ↦ p(t²)` on `[0,1]`). Run `same_parameter`.
- **invariant**: after the pass, `max over 23 shared t of |C3d(t) − Σ(p(t))| ≤ tolE`, and
  `tolE ≤ 1.5 × measured deviation + 1e-7` (the `ComputeTol` bound), and `p.f/p.l == C3d.f/C3d.l`.
- **invariant**: `same_parameter` is idempotent — a second run changes nothing.

**T12 — Tolerance is measured, not assumed.**
Build an edge whose pcurve deviates by exactly `d = 3.7e-6` (constructed offset).
- **invariant**: `tolE ∈ [1.5d, 1.5d·(1+1e-9)]`, floored at `1e-7`; and after
  `harmonize_tolerances`, `tol(V) ≥ tol(E) ≥ tol(F)` at every incidence, with
  `dist(P(V), Σ(p(t_V))) ≤ tol(V)`.

**T13 — Provenance audit (G10).**
Over the full 224-cell primitive-pair sweep:
- **invariant**: `count(provenance == Fit) == 0` for every pair whose two surfaces are both analytic;
  and `count(pcurve.kind == FitNurbs2d && chart.kind != Freeform) == 0`.
This is the single number that says whether the subsystem is done.

**T14 — Out-of-chart detection.**
Construct a pcurve that leaves the chart by 5% of the U span on a non-periodic direction.
- **invariant**: `compute_tol` returns a deviation ≥ `DSdu × overshoot`, i.e. it does **not** report
  a small number obtained by clamping. (Direct port test of `BRepLib.cxx:1096-1121`.)

**T15 — Volume closure via shared boundary (G11).**
For every closed result of the primitive sweep: compute the volume twice, once integrating each
face's boundary flux from its own pcurves.
- **invariant**: the partition residual `|vol(A∪B) + vol(A∩B) − vol(A) − vol(B)|` is at the level of
  the *3D curve* tolerance, not the pcurve fit tolerance. Today the mismatch is first order in the
  ~2e-4 pcurve difference (`src/brep.cpp:7482-7484`).

---

# 6. IMPLEMENTATION ORDER

Each increment is independently shippable, independently measurable, and does not require the next.

**I0 — `Chart`: exact forward map, exact inverse, exact metric.** (`src/chart.h/.cpp`)
Port `ElSLib::{Plane,Cylinder,Cone,Sphere,Torus}{Value,D1,Parameters}` verbatim
(`ElSLib.cxx:71-88`, `:126-160`, `:1547-1701`) plus `du_scale/dv_scale` from §2.3, and
`recognize_chart` that fills the frame (`xd, yd, zd`) — the piece `RecogSurface` lacks.
*Gate*: for 10⁶ random `(u,v)` per chart type, `invert(value(u,v)) == (u,v)` to 1e-15 (modulo period);
`Su·Sv == 0` to 1e-16; `du_scale` matches a finite-difference `|Su|` to 1e-9.
*No behaviour change; nothing calls it yet.*

**I1 — `PCurve::Pullback`, functional, same-range by construction.** (`src/pcurve.h/.cpp`)
`value(t) = chart.invert(c3d.point_at(t))` with continuous branch tracking; `d1(t)` by the
orthogonal chain rule (`ProjLib_ComputeApprox.cxx:209-227`); domain **is** `[f,l]`.
*Gate*: T1, T6, T14. Residual `≤ 1e-13` for every analytic surface × every conic, in every pose.
*This alone removes the 1e-4 class* — before any closed-form branch work.

**I2 — Exact seam crossings and seam splitting.** (`seam_crossings`, `split_at_seams`)
Root-find `p_u(t) = u_seam` on the exact 3D curve; assign period constants to the split endpoints;
emit one `PCurve` per period cell with its `(ku,kv)`.
*Gate*: T9. Both operands produce bitwise identical crossing parameters.

**I3 — Plane exactness end to end.**
Route the boolean's plane pcurves through the affine control-point image
(the code already exists at `src/brep.cpp:916-938`; lift it into `PCurve` construction) instead of
through `analytic_pcurve`'s PLANE arm plus a later resample.
*Gate*: T1 plus "no plane face in the corpus carries a `FitNurbs2d`".

**I4 — Measurement: `compute_tol` + `validate_edge`.** (`src/sameparameter.cpp`)
Verbatim ports of `BRepLib.cxx:1070-1188` and `BRepLib_ValidateEdge.cxx`. Wire them as a *gate*,
not a repair: every emitted `PCurve` is measured, the deviation is recorded on the `EdgeGeom`, and
a deviation above `1.5×tol` is a hard failure with a named diagnostic.
*Gate*: T12, T14. Replaces the diagnostic-only `scaf.max_devA/max_devB`
(`src/brep_section.cpp:1197-1198`) with a per-edge, consumed number.

**I5 — Model-space tolerances everywhere.**
Replace the seven domain-relative sites of §4.3 with `chart.u_tol/v_tol` at the point of use.
*Gate*: T7 (padded-domain invariance), and the ARCHITECTURE_v2 M0 gate — STEP round-trip box cell
back to 6 faces / 0 naked.

**I6 — `EdgeGeom` as the shared entity; G4 and G5 structural.**
One record per section edge, referenced (not copied) by both operands' faces; endpoints stored, not
recomputed; seam edges as `closed_surface = true` with `pc` and `pc2`.
Absorb `SharedEdgePool` (`src/brep_section.h:57-62`) into it.
*Gate*: T8 (single-operand padded split, naked = 0, bitwise wire closure), T9.

**I7 — Closed-form branch pcurves (§2.5 Layer 2).**
`CylSinusoid`, `ConeRational`, `SphereBranch`, `TorusBranch`, each with its exact branch intervals
and turning points. Strictly an *upgrade* over I1's `Pullback` — same residual, but analytic
extremes, analytic branch splits, and an exact answer to "does this plane cut this torus, and
where".
*Gate*: T2, T3, T4, T5.

**I8 — `same_parameter` for imported geometry.**
Port `GeomLib::SameRange` (§2.7.3 step 2), `Approx_SameParameter::Build` (§2.7.5), the C0 repair
chain and the OCC5898 rescue. Apply **only** to edges whose pcurves came from a file
(`provenance == Imported`); our own pcurves are same-parameter by construction and must take the
`IsSameParameter()` early-out.
Match OCCT's write semantics exactly: `tolE` is a **SET**, vertices are **grown**, and
`harmonize_tolerances` runs afterwards — port both halves or neither
(`kb/audit_occt_tolerance-model.md` §3.2).
*Gate*: T11, T12.

**I9 — Fit, confined.**
`FitNurbs2d` becomes legal *only* when `chart.kind == Freeform`. Implement it as the OCCT ladder —
Gauss-point least squares, degrees 5..10 scanned high-to-low, `min(2·deg+1, 24)` points, interval
bisection to `MAXSEGM = 1000`, hang guard, `Convert_CompBezierCurves2dToBSplineCurve2d`,
`BSplCLib::Reparametrize` back onto `[f,l]` — and delete `Closest::surface_curve`'s use as a pcurve
source (`src/closest.cpp:691`, `:713`) from the boolean path.
*Gate*: T13 (`count(Fit on analytic) == 0`), T15 (partition residual at 3D-curve tolerance).

**I10 — Degeneracy typing.**
`TrimC3d`/`ExtendC2d` analogues for sphere poles, cone apex and degenerate NURBS isos
(`ProjLib_ProjectedCurve.cxx:130-238`), each producing a *named* outcome.
*Gate*: T10.

**Ordering rationale.** I1 is the whole win and depends on nothing but I0; everything after it is
either correctness plumbing (I2, I3, I6, I10), measurement (I4), invariance (I5, I7), or
containment of the legacy path (I8, I9). If only two increments ever land, land I0+I1 and I5.
