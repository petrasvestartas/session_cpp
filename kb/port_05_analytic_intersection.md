# port_05 — Analytic surface/surface intersection for ARBITRARY relative pose

**Subsystem**: closed-form (and OCCT's own semi-closed-form) intersection of every ordered pair of
`{plane, cylinder, cone, sphere, torus}` in *arbitrary* relative pose. This is stage 6 (FF) of
`kb/ARCHITECTURE_v2.md` for the quadric×quadric case. It is the stage whose coverage gaps produce
the measured 0/20 on box×sphere, 0/20 on sphere×sphere, 0 on every cone pair, and the hangs on
every torus pair.

**Ground truth read for this document** (tree `/home/petras/code/code_cpp/OCCT`, all paths below are
relative to that root; every line number was opened and read, not recalled):

| file | role |
|---|---|
| `src/ModelingData/TKGeomBase/IntAna/IntAna_QuadQuadGeo.cxx` (2919 lines) | the case table — 15 `Perform` overloads |
| `src/ModelingData/TKGeomBase/IntAna/IntAna_QuadQuadGeo.hxx` | result accessors, tolerance fields |
| `src/ModelingData/TKGeomBase/IntAna/IntAna_ResultType.hxx` | the 10 result types |
| `src/ModelingData/TKGeomBase/IntAna/IntAna_Quadric.cxx/.hxx` | implicit-form coefficients + frame change |
| `src/ModelingData/TKGeomBase/IntAna/IntAna_IntQuadQuad.cxx` | the exact ALine solver (cyl×quadric, cone×quadric) |
| `src/ModelingData/TKGeomBase/IntAna/IntAna_Curve.cxx` | ALine evaluator |
| `src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.cxx` (9715 lines) | consumption layer, 15 `IntXxYy` arms, cyl×cyl semi-analytic solver |
| `src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Intersection.cxx` | routing: which pairs even reach the analytic layer |
| `src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Line.cxx` | the three tangency/transition encodings |
| `src/FoundationClasses/TKMath/math/math_TrigonometricFunctionRoots.cxx` | root finder under the ALine solver |
| `src/ModelingAlgorithms/TKBO/IntTools/IntTools_FaceFace.cxx` | boolean-side analytic eligibility gate |

Corrections in `kb/audit_occt_intana-analytic.md` were re-verified against the source before being
restated here; where this document and the older `kb/occt2_intana-analytic.md` /
`kb/spec_analytic_ssi_canonical.md` disagree, **this document is the one that was read against
source line by line** — see §2.9 for the list of corrections.

---

## 1. WHAT THIS SUBSYSTEM MUST GUARANTEE

Each invariant is written so it can be asserted in a test with no external oracle.

**G1 — Totality.** For every ordered pair of the five quadric kinds, in every relative pose, the
subsystem returns a *typed verdict*, never "unhandled". The verdict alphabet is exactly:
`Empty | Same | Point(s) | Line(s) | Circle(s) | Ellipse(s) | Parabola | Hyperbola(s) |
PointAndCircle | Analytic(ALine) | Numeric(WLine) | InfiniteSectionCurve`.
There is no code path that silently drops to "caller marches with no seed". Test: iterate the 15
kind-pairs × N random rigid motions; assert `verdict != Unhandled` for all.

**G2 — No pose privilege.** The verdict *type* is invariant under any rigid motion applied to both
operands simultaneously, and the emitted geometry transforms covariantly:
`section(T·A, T·B) == T·section(A, B)` to `1e-12·scale`. This is the invariant our recognisers
violate today (they only fire coaxial/concentric). Test: for a fixed pair, sample 200 random
`T ∈ SE(3)`; assert type stability and covariance of every emitted point/radius/axis.

**G3 — Analytic identity is carried, never refitted.** The input to this subsystem is
`(kind, exact parameters)` obtained from the STEP reader or from the construction API, transformed
by `T` as `axis := R·axis, apex := T·apex, radius := radius`. This subsystem must **never** sample a
NURBS and least-squares-fit a quadric. Test: build a cylinder, apply `T`, assert the descriptor's
axis equals `R·a` bit-for-bit up to the matrix product (not up to a fit tolerance).

**G4 — Degeneracy is typed, and each type has one named tolerance.** Coincidence (`Same`),
tangency (`Touch` + `Situation`), apex contact, concentricity, pole contact, and empty each have a
distinct return, and the predicate that selects them names its epsilon. No branch may be reached by
"the sqrt came out at 1e-17". Test: drive each degenerate configuration from both sides of its
epsilon and assert the type flips exactly once, at the documented threshold.

**G5 — Tangency carries a side, not just a flag.** A tangential contact must report, per operand,
whether the *other* operand lies `Inside` or `Outside` at the contact. A boolean "tangent" flag is
insufficient: downstream keep/purge for grazing arcs needs the side. OCCT encodes this as
`tS1 = tS2 = IntSurf_Touch` plus `sit1/sit2 ∈ {Inside, Outside}`
(`IntPatch_Line.cxx:36-49`). `IntSurf_Undecided` (`IntPatch_Line.cxx:51-62`) is a *different*
contract meaning "transversal, sign unresolvable" and must not be conflated.
Test: two externally tangent spheres — assert `Touch` + `(Outside, Outside)`; internally tangent —
assert `Touch` + one `Inside`.

**G6 — Every emitted curve carries a transverse orientation.** A transversal section curve carries
`(trans1, trans2) ∈ {(In,Out),(Out,In),(Undecided,Undecided)}` derived from
`tangent · (N2 × N1)` at a sampled point. Downstream classification propagation (ARCHITECTURE_v2 §4)
consumes this instead of re-classifying. Test: for a sphere cut by a plane, assert the circle's
`trans1` flips when the plane normal is reversed.

**G7 — Nothing is emitted on the wrong nappe.** Cone contacts on the negative nappe are purged.
The predicate is the *axis parameter*, not a distance: keep iff
`ElCLib::LineParameter(cone.Axis(), P) >= LineParameter(cone.Axis(), apex)`
(`IntPatch_ImpImpIntersection.cxx:8858` for cone×cone, `:9245` for cone×sphere).
Test: cone×sphere with the sphere centred behind the apex — assert 0 curves, not 2.

**G8 — Bounded work.** No branch may loop unboundedly. Every retry ladder has a hard count
(OCCT: the tangent-probe walk is `kount > 5`, `IntPatch_ImpImpIntersection.cxx:8534-8543`) and the
numeric cyl×cyl path has a point cap (`aNbMaxPoints = 1000`, `:6675`) and an explicit
"section curve is effectively infinite" bail
(`IntStatus_InfiniteSectionCurve`, `:6603-6608`). Test: torus×torus at 20 random poses completes in
bounded wall time and returns a typed verdict (today: 0 poses complete in 25 minutes).

**G9 — Result geometry is exact where the mathematics is exact.** When the verdict is
`Circle/Ellipse/Line/Parabola/Hyperbola`, the emitted conic must satisfy both implicit surface
equations to `1e-13 · scale²` at 64 sampled parameters. This is an oracle-free residual test.

**G10 — The analytic path is the default, the marcher is the exception.** For any pair drawn from
`{plane, cylinder, cone, sphere}` (16 ordered pairs), the result is either a closed-form conic set
or an exact `ALine` (implicit algebraic curve with closed-form point evaluation) — **never** a
marched WLine, except for cyl×cyl in general position, where OCCT itself uses a semi-analytic
per-`U1` closed-form evaluator (§2.7), which is still not a marcher. Torus pairs are the only ones
where OCCT genuinely marches (§2.8).

---

## 2. OCCT'S ALGORITHM, IMPLEMENTABLE

### 2.0 Shape of the whole thing

```
IntPatch_Intersection::Perform            (routing: is this pair eligible for the analytic path?)
  └─ GeomGeomPerfom
       └─ IntPatch_ImpImpIntersection::Perform          (consumption layer; 15 IntXxYy arms)
            ├─ IntAna_QuadQuadGeo::Perform(A, B, ...)   (THE CASE TABLE → IntAna_ResultType)
            │     └─ returns Empty | Same | Point | Line | Circle | PointAndCircle
            │               | Ellipse | Parabola | Hyperbola | NoGeometricSolution
            ├─ on NoGeometricSolution, for cyl/cone × {plane,cyl,cone,sphere}:
            │     IntAna_IntQuadQuad(Cyl|Cone, IntAna_Quadric)  → up to 12 IntAna_Curve (ALines)
            ├─ on NoGeometricSolution, for cyl × cyl:
            │     ComputationMethods + WorkWithBoundaries      → 2 WLines, closed-form per U1
            └─ on NoGeometricSolution, for any torus pair: return false → caller marches
```

`IntAna_ResultType` (`IntAna_ResultType.hxx:12-23`), in declaration order:
`IntAna_Point, IntAna_Line, IntAna_Circle, IntAna_PointAndCircle, IntAna_Ellipse, IntAna_Parabola,
IntAna_Hyperbola, IntAna_Empty, IntAna_Same, IntAna_NoGeometricSolution`.

Dispatch key (`IntPatch_ImpImpIntersection.cxx:3058-3091`, `:2555-2567`):
`SetQuad` maps Plane→1, Cylinder→2, Cone→3, Sphere→4, Torus→5; `iTT = iT1*10 + iT2`;
`bReverse = (iT1 > iT2)`. Every arm therefore sees its operands in canonical kind order and knows
whether the caller's (1,2) were swapped. **Copy this exactly** — every `Reversed` branch in the
transition code depends on it.

The single entry-level angular tolerance is `Tolang = 1.e-8`
(`IntPatch_ImpImpIntersection.cxx:2551`). It is *not* the same as the tolerances used inside
`IntAna_QuadQuadGeo` (§2.2) nor the ones used by the cone×cone recursions (§2.3.10).

### 2.1 Shared primitives (implement these first, verbatim)

#### 2.1.1 `AxeOperator` — the axis-relation oracle
`IntAna_QuadQuadGeo.cxx:69-208`. Constructed as
`AxeOperator(A1, A2, theEpsDistance = 1.e-14, theEpsAxesPara = Precision::Angular() = 1.e-12)`
(`:72-75`).

```
V1 = A1.Direction(); V2 = A2.Direction();
RefineDir(V1); RefineDir(V2);                                     // :139-140
thecoplanar = false; thenormal = false;
theparallel = V1.IsParallel(V2, myEPSILON_AXES_PARA);             // :145
perp        = A1.Direction() × A2.Direction();                    // :147  NOTE: RAW dirs, not refined
if (theparallel) thedistance = gp_Lin(A1).Distance(A2.Location());// :150-151
else             thedistance = |unit(perp) · (P2 - P1)|;          // :155
if (thedistance < myEPSILON_DISTANCE) {                           // :159   <-- gate
    D33 = det[ V1 ; V2 ; P1-P2 ];                                 // :161-169
    if (|D33| <= myEPSILON_DISTANCE) thecoplanar = true;          // :170-173
}
thenormal = |V1·V2| < myEPSILON_AXES_PARA;                        // :176
if (thecoplanar && !theparallel) ptintersect = <2x2 solve, largest-minor pivot>;  // :179-203
else                             ptintersect = (0,0,0);           // :206
```
Accessors (`:79-91`):
`Same() = theparallel && (thedistance < myEPSILON_DISTANCE)`;
`Intersect() = thecoplanar && !theparallel`;
`Coplanar()`, `Parallel()`, `Normal()`, `Distance()`, `PtIntersect()`.

**Load-bearing consequence**: `thecoplanar` is only ever computed when `thedistance < eps`.
Therefore `Intersect()` is *not* "coplanar and non-parallel" — it is "the axes actually meet, to
within `eps`". With the default `eps = 1e-14` this demands axes constructed to intersect. Every
`Perform` except cyl×cyl uses the default; **cyl×cyl overrides it to `Precision::Confusion() = 1e-7`**
(`:1054-1057`).

The pivoted 2×2 solve (`:184-201`), verbatim:
```
smx,smy,smz = P2 - P1
Det1 = V1.Y*V2.X - V1.X*V2.Y
Det2 = V1.Z*V2.Y - V1.Y*V2.Z
Det3 = V1.Z*V2.X - V1.X*V2.Z
if (Det1 != 0 && |Det1| >= |Det2| && |Det1| >= |Det3])  A = (smy*V2.X - smx*V2.Y)/Det1
else if (Det2 != 0 && |Det2| >= |Det1| && |Det2| >= |Det3]) A = (smz*V2.Y - smy*V2.Z)/Det2
else                                                       A = (smz*V2.X - smx*V2.Z)/Det3
ptintersect = P1 + A*V1
```

`AxeOperator::Distance(dist, Param1, Param2)` (`:212-231`) additionally returns the signed common
perpendicular length and the two foot parameters via three 3×3 determinants; it is used only by the
cyl×cyl external-tangency branch (`:1269-1290`). It silently does nothing when `D == 0`.

#### 2.1.2 `RefineDir` — exact-axis snapper
`IntAna_QuadQuadGeo.cxx:2867-2918`. It fires **only** if some component is *exactly* `±1.0`
(`m > 0`) **and** some other component is nonzero and not `±1` (`n > 0`) (`:2876-2888`). Then it
finds the first component within `[1-RealEpsilon(), 1+RealEpsilon()]` in magnitude, sets it to
`±1`, and zeroes the other two (`:2896-2915`).
**It will not clean up a rotated frame** — after a generic rotation no component is exactly `±1`, so
`m == 0` and the function is a no-op. Do not treat it as a canonicaliser. If you want axis snapping,
do it at recognition time on known-exact parameters (G3), before entering the case table.

#### 2.1.3 `DirToAx2` — deterministic conic phase
`IntAna_QuadQuadGeo.cxx:237-257`. Given a point `P` and a unit direction `D = (x,y,z)`, returns
`gp_Ax2(P, D, X)` where `X` is chosen from the **smallest-magnitude component**, with exact-zero
short-circuits:
```
if (|x| == 0 || (|x| < |y| && |x| < |z|))  X = normalize(( 0, -z,  y))
else if (|y| == 0 || (|y| < |x| && |y| < |z|)) X = normalize((-z,  0,  x))
else                                        X = normalize((-y,  x,  0))
```
Every `Circle(n)` accessor routes through this (`:2743,2745,2753,2757,2761,2765`). Two
mathematically identical circles produced by different branches therefore get **different parameter
origins**. Never key a weld or a dedup on a conic parameter; key on 3D points.

#### 2.1.4 `EstimDist` — minimal apex-to-solution distance for two cones
`IntAna_QuadQuadGeo.cxx:263-325`. Preconditions (comment `:265-266`): axes coplanar and
`distance > Precision::Confusion()`.
```
aPln = plane through (apex1, apex2, apex1 + axis1_dir)          // :271, gce_MakePln
if (!aMkPln.IsDone()) return Precision::Infinite();             // :273-275
project each cone axis into aPln (ProjLib::Project) → anAx12d, anAx22d
Lines1[0,1] = anAx12d rotated by ±SemiAngle1                    // :283-284
Lines2[0,1] = anAx22d rotated by ±SemiAngle2                    // :290-291
for each of the 4 (i,j): IntAna2d_AnaIntersection; take |ParamOnFirst|, |ParamOnSecond|
aMinDist[0] = min over all |ParamOnFirst|;  aMinDist[1] = min over all |ParamOnSecond|
return max(aMinDist[0], aMinDist[1])                            // :323-324
```

### 2.2 The tolerance table — actual values

`IntAna_QuadQuadGeo::InitTolerances()` (`IntAna_QuadQuadGeo.cxx:351-359`); `Precision::Confusion() =
1e-7`, `Precision::Angular() = 1e-12`, `Precision::PConfusion() = Confusion()*0.01 = 1e-9`
(`src/FoundationClasses/TKernel/Precision/Precision.hxx:123,165,334`).

| field | value | used by |
|---|---|---|
| `myEPSILON_DISTANCE` | `1.0e-14` | `AxeOperator` default; plane×torus perpendicular gate `:2234`; cyl/cone/sphere×torus axis-distance gates `:2299,2378,2514`; torus×torus `:2606` |
| `myEPSILON_ANGLE_CONE` | `Precision::Angular()` = `1e-12` | cone×cone `aTolAng` floor `:1458,1471`; cone×cone `Same` sub-branch `:1486` |
| `myEPSILON_MINI_CIRCLE_RADIUS` | `0.01 * Confusion()` = `1e-9` | sphere×cone `PointAndCircle` collapse `:1971,1988`; sphere×sphere point collapse `:2119` |
| `myEPSILON_CYLINDER_DELTA_RADIUS` | `1.0e-13` | cyl×cyl equal-radius test (RELATIVE) `:1224`; plane×torus numeric tolerance `:2197` |
| `myEPSILON_CYLINDER_DELTA_DISTANCE` | `Precision::Confusion()` = **`1e-7`** | **only** the cyl×cyl `AxeOperator` `:1054-1057` |
| `myEPSILON_AXES_PARA` | `Precision::Angular()` = `1e-12` | `AxeOperator` parallel/normal; torus-pair `IsParallel/IsNormal` `:2182,2183,2298,2377,2606` |

**Audited constant, restated**: cyl×cyl coplanarity epsilon is `Precision::Confusion() = 1e-7`,
**not** `1e-14`. The signature default is explicitly overridden at `:1054-1057`. Consequently two
equal-radius cylinders whose axes miss by `1e-9` *do* get exact Steinmetz ellipses. `1e-14` applies
only to the default-constructed operators in cyl×cone `:1327`, cyl×sphere `:1377`,
cone×cone `:1456`, sphere×cone `:1923`.

**Transition dead-bands are non-uniform.** This is not a stylistic detail; the boundaries differ by
eight orders of magnitude and change which curves get `Undecided`:

| dead band | sites |
|---|---|
| `> 0.0` (none) | `IntPP:3136`; `IntPCy:3261,3277,3304,3325`; `IntPSp:3419`; `IntPTo:3833`; `IntCySp` 1st circle `:8227`; `IntCoCo` 2-line `:8803` |
| `1e-8` | `IntPCo:3731,3736,3761,3766`; `CyCyAnalyticalIntersect:4997,5002,5052,5057,5128,5133`; `IntCyCo:8445,8450,8548,8553`; `IntCoCo:8878,8883,8917,8922,8955,8960,8992,8997`; `IntSpSp:9530,9535`; `TreatResultTorus:9686,9691` |
| `1e-7` | `IntCySp` 2nd circle `:8243,8248` |
| `1e-9` (`Precision::PConfusion()`) | `IntCoSp` `PointAndCircle` `:9312,9317,9329,9334` |
| `1e-9` (literal `0.000000001`) | `IntCoSp` ALine `:9416,9421`; `IntCoCo` ALine `:9095,9100` |

Additional ALine-only guard: `IntCoCo` rejects a tangent probe whose derivative is degenerate,
`tgvalid.SquareMagnitude() < 1e-14 → tgfound = false` (`:9080-9084`). `IntCySp`/`IntCyCo`/`IntCoSp`
have no such guard.

### 2.3 The case table, pair by pair

Notation: `n` = plane unit normal, `d` = plane signed distance function `A x + B y + C z + D`,
`a` = axis unit direction, `V` = cone apex, `β` = cone semi-angle, `R` = radius, `Rmaj/Rmin` = torus
major/minor.

---

#### 2.3.1 plane × plane — `Perform(P1,P2,TolAng,Tol)`, `:389-512`
Canonicalisation: none; both operands stay in world coordinates.

```
P1.Coefficients(A1,B1,C1,D1); P2.Coefficients(A2,B2,C2,D2)
vd    = n1 × n2 ;  aMVD = |vd|
dist1 = A2*P1.Loc + D2 ;  dist2 = A1*P2.Loc + D1
if (aMVD <= TolAng)                                        // :413
    typeres = (|dist1| <= Tol && |dist2| <= Tol) ? Same : Empty      // :416
else
    denom  = n1·n2 ;  ddenom = 1 - denom²
    denom  = (|ddenom| <= 1e-16) ? 1e-16 : ddenom           // :423,428  aEps = 1e-16
    par1   =  dist1/denom ;  par2 = -dist2/denom            // :430-431
    inter1 = n1 × vd ; inter2 = n2 × vd                     // :433-434
    X1 = P1.Loc + par1*inter1 ; X2 = P2.Loc + par2*inter2
    pt1 = 0.5*(X1 + X2) ; dir1 = unit(vd) ; typeres = Line ; nbint = 1   // :443-446
```
Origin refinement for near-parallel planes (`:458-509`): thresholds `aTreshAng = 2e-6`
(≈ 1e-4 degree), `aTreshDist = 1e-12`. If `aMVD < 2e-6` **and** `pt1` is farther than `1e-12` from
either plane, `pt1` is recomputed by two successive `IntAna_IntConicQuad` line/plane hits: first the
line through `pt1` along `n1` against `P1`, then the line through that point along `dir1 × n1`
against `P2` (`:476-507`). Any `!IsDone()` or a parallel verdict aborts with `done` still `false`
(`:481-503`) — a documented give-up.

Emission (`IntPP`, `:3106-3150`): `Same → SameSurf = true` (no curve). Otherwise one
`IntPatch_GLine(line, /*Tang=*/false, trans1, trans2)` with
`discri = dir·(N2(loc) × N1(loc))`, `discri > 0.0 → (Out,In)` else `(In,Out)` (`:3133-3145`),
**no dead band**.

---

#### 2.3.2 plane × cylinder — `Perform(P, Cl, Tolang, Tol, H)`, `:543-722`
`H` is the cylinder face's finite V-extent, supplied by the caller as `VMax - VMin`, or `0` if
either is infinite (`IntPatch_ImpImpIntersection.cxx:2579-2587`).

Step 1 — **parallelism relaxation** (`:571-598`). This is the only place OCCT lets face extent
influence a surface-level decision:
```
dA = |angle(axis, n)| ;  if (dA > π/4) {
    dangle = | |angle(axis,n)| - π/2 |
    if (dangle > Tolang) {
        sinda = |sin(dangle)| ;  dif = |sinda - Tol|
        if (dif < Tol  ||  (H > 0 && sinda*H < 2*Tol)) {       // :588   TWO disjuncts
            tolang  = 2*sinda
            toltang = max(Tol, sinda*H*1.01)
            newparams = true
        } } }
```
Note the first disjunct `|sinda - Tol| < Tol` fires with **no** face-height information at all.

Step 2 — `IntAna_IntConicQuad inter(axis_line, P, tolang, toltang, H)` (`:601`).

Step 2a — **`inter.IsParallel()`** (`:603-684`) → `typeres = Line`:
`omega = axisLoc - dist·n` (foot of the axis origin on the plane), `dist = d(axisLoc)`.
- `| |dist| - radius | < Tol` → 1 tangent line at `omega`  (`:611-636`)
- `|dist| < radius` → 2 lines at `omega ∓ h·axey`, `h = sqrt(R² - dist²)`,
  `axey = axisDir × n` (already unit) (`:637-674`)
- else → `Empty` (`:680-683`)

When `newparams` is set, the line **directions** are not the cylinder axis. They are recomputed by
translating the axis origin by **100 units** along the axis, re-projecting, and taking the direction
between the two foot points (`:618-631` for the 1-line case, `:646-668` for the 2-line case, where
the offset uses `ht = sqrt(max(0, R² - distt²))`, `:653-654`). Otherwise `dir1 = dir2 = axisDir`.

Step 2b — **not parallel** (`:685-719`) → exactly one axis/plane point `pt1 = inter.Point(1)`:
```
axey = n × axisDir ;  sint = |axey|
if (sint < Tol/radius)      // :695   NOTE: a linear tolerance divided by a radius
    typeres = Circle ;  dir1 = axisDir ;  dir2 = Cl.XDirection() ;  param1 = radius
else
    typeres = Ellipse ;  cost = |axisDir·n| ;  axex = axey × n
    dir1 = n ;  dir2 = axex
    param1 = radius/cost ;  param1bis = radius                 // major, minor
```

Emission (`IntPCy`, `:3157-3345`): `Line` with `NbSol == 1` → **tangency**, see §2.6.2.
`Line` with `NbSol == 2` → two transversal GLines, dead band `> 0.0` (`:3261,3277`).
`Circle` → `AdjustToSeam(Cy, cirsol)` (`:3300`) then transversal, dead band `> 0.0` (`:3304`).
`Ellipse` → transversal, dead band `> 0.0` (`:3325`). `default: return false` (`:3340`).

---

#### 2.3.3 plane × sphere — `Perform(P, S)`, `:980-1021` (no tolerance arguments at all)
```
dist = d(sphereCentre)
if (| |dist| - radius | < Epsilon(radius))          // :997  ≈ 2.22e-16 * radius
    typeres = Point ; pt1 = centre - dist*n
else if (|dist| < radius)
    typeres = Circle ; pt1 = centre - dist*n
    dir1 = P.Axis().Direction();  if (!P.Direct()) dir1.Reverse()      // :1011-1015
    dir2 = P.Position().XDirection()
    param1 = sqrt(radius² - dist²)
else  typeres = Empty                                // default set at :989
```
**The tangency band is `Epsilon(radius)`, an ULP, not a model tolerance.** For `R = 100` that is
`1.4e-14`; a plane `1e-9` inside tangency yields a circle of radius `≈ 4.5e-4`, never a point. If
you want a usable tangency verdict you must widen this yourself and document that you diverge.

Emission (`IntPSp`, `:3352-3439`): `Point` → `IntPatch_Point::SetValue(psol, TolTang, /*tangent=*/true)`
(`:3404`). `Circle` → `AdjustToSeam(Sp, cirsol, Tolang)` (`:3413`) then transversal, dead band
`> 0.0` (`:3419`). `default: return false` (`:3434`).

---

#### 2.3.4 plane × cone — `Perform(P, Co, Tolang, Tol)`, `:752-953`
Canonical quantities (`:767-796`):
```
apex = Co.Apex() ;  dist = d(apex)                    // SIGNED
normp = P.Axis().Direction() ;  if (!P.Direct()) normp.Reverse()      // :774-778
axey = normp × coneAxisDir ;  axex = axey × normp
angl = Co.SemiAngle() ;  cosa = cos(angl) ;  sina = |sin(angl)|
sint = |axey| ;  cost = |coneAxisDir · normp|
costa = cost*cosa - sint*sina                          // = sin((π/2 - t) - angl)
```

**Branch A — plane through the apex**, `|dist| < Tol` (`:799`):
- `|costa| < Tolang` → 1 **Line** through the apex (`:805-816`). Direction is built by taking
  `ptonaxe = apex + 10·axisDir`, projecting it onto the plane, and using `ptonaxe - apex`.
- `cost < sina` (plane cuts the interior) → 2 **Lines** from the apex,
  `dh = sqrt(sina² - cost²)/cosa`, `dir1 = axex + dh·axey`, `dir2 = axex - dh·axey` (`:817-826`).
- else → 1 **Point** at the apex (`:827-832`).

**Branch B — plane misses the apex** (`:834-919`):
- `cost < Tolang` (plane contains the axis direction) → **Hyperbola**, `nbint = 2`:
  `pt1 = pt2 = apex - dist·normp`, `dir1 = normp`, `dir2 = axex`,
  `param1 = param2 = |dist/tan(angl)|`, `param1bis = param2bis = |dist|` (`:840-852`).
- otherwise compute `center = IntAna_IntConicQuad(axisLine, P, Tolang).Point(1)`,
  `distance = |apex - center|` (`:856-864`), and **flip the frame** if
  `inter.ParamOnConic(1) + Co.RefRadius()/tan(angl) < 0` → `axex.Reverse(); axey.Reverse()`
  (`:866-870`);
  - `|costa| < Tolang` → **Parabola**, `deltacenter = distance/(2·cosa)`, `axex.Normalize()`,
    `pt1 = center - deltacenter·axex`, `dir1 = normp`, `dir2 = axex`,
    `param1 (focal) = deltacenter·sina²` (`:872-882`);
  - `sint < Tolang` → **Circle**, `pt1 = center`, `dir1 = cone axis dir`,
    `dir2 = cone XDirection`, `param1 = |apex-center|·|tan(angl)|` (`:883-891`);
  - `cost < sina` → **Hyperbola**, `nbint = 2`, `axex.Normalize()`,
    `deltacenter = sint·sina²·distance/(sina² - cost²)`, `pt1 = pt2 = center - deltacenter·axex`,
    `param1 = param2 = cost·sina·cosa·distance/(sina² - cost²)`,
    `param1bis = param2bis = cost·sina·distance/sqrt(sina² - cost²)` (`:892-905`);
  - else (`cost > sina`) → **Ellipse**, `axex.Normalize()`,
    `major = cost·sina·cosa·distance/(cost² - sina²)`,
    `deltacenter = sint·sina²·distance/(cost² - sina²)`,
    `pt1 = center + deltacenter·axex` (**plus**, unlike the parabola/hyperbola branches),
    `param1 = major`, `param1bis = cost·sina·distance/sqrt(cost² - sina²)` (`:906-918`).

**Extreme-conic give-up** (`:925-950`): `EllipseLimit = 1.0e+9`, `HyperbolaLimit = 2.0e+6`.
If `Ellipse && (|param1| > 1e9 || |param1bis| > 1e9)`, or `Hyperbola && (|param2| > 2e6 ||
|param2bis| > 2e6)`, or `Hyperbola && (|param1| > 2e6 || |param1bis| > 2e6)`, then `done = false;
return;`. `IntPCo` then returns `false`, `ImpImp::Perform` returns with `myDone = Fail`, and
`GeomGeomPerfom` falls back to `ParamParamPerfom` (`IntPatch_Intersection.cxx:1792-1797`). **This is
a documented OCCT fallback, not a bug.**

Emission (`IntPCo`, `:3446-3786`), and this is where the audited apex-split correction lives:
- `Point` → point with `isTangent = false` (`:3497`).
- `Line, NbSol == 1` (tangency) → **2 GLines**, one per ray from the apex, each with
  `Touch` + `Situation`, each carrying the apex as vertex with `SetFirstPoint(1)` (`:3515-3596`).
- `Line, NbSol == 2` → **4 GLines** (2 algebraic lines × 2 rays), transversal, `Multpoint = true`,
  apex added as a multiple vertex (`:3598-3676`).
- `Circle` → `AdjustToSeam(Co, cirsol)` (`:3683`), transversal, dead band `> 0.0` (`:3689`).
- `Ellipse` → transversal, dead band `> 0.0` (`:3710`).
- `Parabola` → transversal from `YAxis` direction, dead band `1e-8` (`:3731,3736`), else `Undecided`.
- `Hyperbola` → two GLines, probe point `ElCLib::Value(MajorRadius, XAxis)`, dead band `1e-8`
  (`:3761,3766`).
- `default: return false` (`:3781`).

---

#### 2.3.5 plane × torus — `Perform(Pln, Tor, Tol)`, `:2163-2249`
```
if (Rmin >= Rmaj)  → NoGeometricSolution                              // :2171-2175
bParallel = torusAxis.IsParallel(planeAxis, 1e-12)                    // :2182
bNormal   = !bParallel && torusAxis.IsNormal(planeAxis, 1e-12)        // :2183
if (!bNormal && !bParallel) → NoGeometricSolution                     // :2184-2188
```
**This is the whole pose gate.** Any other relative orientation is refused.

Parallel branch (`:2193-2229`), `aTolNum = myEPSILON_CYLINDER_DELTA_RADIUS = 1e-13`:
```
aDist = d(torusCentre) ;  aDR = |aDist| - Rmin
if (aDR >  aTolNum) → Empty
if (|aDR| < aTolNum) aDist = sign(aDist)*Rmin                        // :2210-2213 snap
typeres = Circle
pt1 = torusCentre - aDist*n ;  aDt = sqrt(|Rmin² - aDist²|)
param1 = Rmaj + aDt ;  dir1 = TORUS axis direction ;  nbint = 1      // :2217-2221
if (aDR < -aTolNum && aDt > Tol) { pt2 = pt1 ; param2 = Rmaj - aDt ; dir2 = dir1 ; nbint = 2 }
```
Perpendicular branch (`:2231-2248`):
```
if (Pln.Distance(torusCentre) > myEPSILON_DISTANCE (1e-14)) → NoGeometricSolution   // :2234
typeres = Circle ; param1 = param2 = Rmin ; dir1 = dir2 = PLANE axis direction ; nbint = 2
aDir = torusAxisDir ^ dir1
pt1 = torusCentre + Rmaj*aDir ;  pt2 = torusCentre - Rmaj*aDir
```
Note the circles' normals differ between branches: torus axis in the parallel branch (`:2220`),
plane normal in the perpendicular branch (`:2242`).

Emission (`IntPTo`, `:3790-3857`): `AdjustToSeam(aTorus, aC)` is applied **only when the plane axis
is NOT normal to the torus axis** (`:3827-3830`). Dead band `> 0.0` (`:3833`).
`NoGeometricSolution → bRet = false` (`:3850-3853`) → whole `ImpImp` fails → marcher.

---

#### 2.3.6 cylinder × cylinder — `Perform(Cyl1, Cyl2, Tol)`, `:1050-1297`
The only `Perform` with an overridden `AxeOperator` epsilon:
```
AxeOperator A1A2(Cyl1.Axis(), Cyl2.Axis(),
                 myEPSILON_CYLINDER_DELTA_DISTANCE /* = 1e-7 */,
                 myEPSILON_AXES_PARA               /* = 1e-12 */);    // :1054-1057
RmR = |R1 - R2| ;  RmR_Relative = RmR / max(R1,R2) ;  DistA1A2 = A1A2.Distance()
```

**Parallel ladder** (`:1070-1221`), in exact order:
1. `DistA1A2 <= Tol` → `Same` iff `RmR <= Tol`, else `Empty` (`:1072-1082`).
2. Project `Cyl2.Location()` onto the base plane of `Cyl1` → `P2` (`:1085-1096`).
3. `DistA1A2 > R1+R2+Tol` → `Empty` (`:1099-1103`).
4. `(R1+R2 - DistA1A2) <= RealSmall()` (**≈ 2.2e-308**) → 1 tangent **Line** at
   `P1 + R1/(R1+R2)·(P2-P1)` (`:1104-1115`). *This branch is effectively unreachable*; external
   tangency is actually caught by the chord collapse inside branch 5.
5. `DistA1A2 > RmR` → 2 **Lines** (`:1116-1197`):
   ```
   aCos  = 0.5*(R1² - R2² + Dist²)/(R1*Dist)
   aSin2 = 1 - aCos²
   isTangent = (4*R1²*aSin2) < Tol²                        // :1157  chord-length collapse
   if isTangent → nbint = 1, pt1 = P1 + R1*aCos*unit(P2-P1)     // :1162-1168
   else {
       aSin = sqrt(aSin2)
       (aDx,aDy) = unit(P2-P1) in Cyl1's (XDirection, YDirection) basis
       pt1 = P1 + (aDx*aCos - aDy*aSin)*R1*Xdir + (aDy*aCos + aDx*aSin)*R1*Ydir
       pt2 = P1 + (aDx*aCos + aDy*aSin)*R1*Xdir + (aDy*aCos - aDx*aSin)*R1*Ydir
   }
   dir1 = dir2 = Cyl1 axis direction
   ```
6. `DistA1A2 > RmR - Tol` → 1 internal-tangent **Line** at `P1 + (R1/RmR)·(P2-P1)`, with
   `R1/RmR` **negated when `R1 < R2`** (`:1198-1214`).
7. else → `Empty` (`:1215-1219`).

**Non-parallel** (`:1222-1296`):
- `RmR_Relative <= 1e-13` **and** `A1A2.Intersect()` → 2 **Ellipses** (Steinmetz), `:1224-1266`:
  ```
  pt1 = pt2 = A1A2.PtIntersect()
  A = angle(dir1cyl, dir2cyl)
  B = |sin(0.5*(π - A))| ;  A = |sin(0.5*A)|
  if (A == 0 || B == 0) → Same, return                                 // :1239-1243
  dir1 = unit(d1 + d2) ;  dir2 = unit(d1 - d2)
  param2 = R1/A ;  param1 = R1/B ;  param1bis = param2bis = R1
  then swap so that param1 >= param1bis and param2 >= param2bis        // :1253-1265
  ```
  Recall `Intersect()` requires `thedistance < 1e-7` **and** `|D33| <= 1e-7`, i.e. the axes must
  actually meet within `1e-7`; and `RmR_Relative` is the **relative** radius difference.
- else if `|DistA1A2 - R1 - R2| < Tol` → **Point** (external tangency of skew cylinders),
  `:1269-1290`: uses `A1A2.Distance(d, p1, p2)` to get the common-perpendicular feet `P1, P2`, then
  `pt1 = P1 + R1·unit(P2-P1)`.
- else → **`NoGeometricSolution`** (`:1293`).

**Therefore: OCCT has NO closed-form solution for skew cylinders with unequal radii.** That case
lands in the semi-analytic solver of §2.7. This is a *documented fallback*, and it is not a marcher.

Emission (`CyCyAnalyticalIntersect`, `:4845-5189`):
`Empty`, `Same`, `Point` (tangent flag `true`, `:4887`);
`Line, NbSol == 1` → tangency with the **two-radius-vector** situation rule (§2.6.2);
`Line, NbSol > 1` → transversal per line, dead band `1e-8` (`:4997,5002`);
`Ellipse` → the two Steinmetz ellipses, with vertices at `0`, `2π` and multiple points at
`0.5π, 1.5π` on the first; the **first** ellipse's GLine is constructed with **swapped**
transitions `(trans2, trans1)` because they were sampled at parameter 0 but apply at π
(`:5067-5069`); the second ellipse's multiple-point parameters are assigned by comparing
`ElCLib::Parameter` of the two tangency points and the transition is then sampled at π or 0
accordingly (`:5107-5124`).
`Parabola`/`Hyperbola` → **throws** `Standard_Failure("IntCyCy(): Wrong intersection type!")`
(`:5176-5178`).
`Circle` and `NoGeometricSolution` → `return false` (`:5180-5185`), and `IntCyCy` then **clears
`slin`/`spnt` and the three flags** before entering the numeric path (`:7924-7931`).

---

#### 2.3.7 cylinder × cone — `Perform(Cyl, Con, Tol)`, `:1324-1344`
```
AxeOperator A1A2(Cyl.Axis(), Con.Axis());       // DEFAULT eps = 1e-14
if (A1A2.Same()) {                              // parallel AND distance < 1e-14
    dist = Cyl.Radius() / tan(Con.SemiAngle())
    pt1 = apex + dist*cylAxisDir ;  pt2 = apex - dist*cylAxisDir
    dir1 = dir2 = cylAxisDir ;  param1 = param2 = Cyl.Radius()
    nbint = 2 ; typeres = Circle
} else typeres = NoGeometricSolution
```
That is the **entire** analytic content. The `Tol` argument is unnamed and unused (`:1324`).
Note both nappes are emitted (`+dist` and `-dist`), unconditionally.

Emission (`IntCyCo`, `:8374-8602`): `Circle` → loop `j = 1..2` (hard-coded, `:8440`), transversal,
dead band `1e-8` (`:8445,8450`).
`Point` → **dead code and buggy**: `Perform(Cyl,Con)` can never produce `IntAna_Point`, and the arm
calls `Quad1.Parameters` twice (`:8426-8427`) instead of `Quad1`/`Quad2`.
`NoGeometricSolution` → `IntAna_IntQuadQuad anaint(Cy, Co, Tol)` (`:8468`; `gp_Cone` converts
implicitly to `IntAna_Quadric`) → §2.5, then per curve `ExploreCurve(Co, aC, 10*Tol, aLC)` splits at
apex crossings (`:8512`, definition `:8608-8675`), then the tangent-probe ladder (`:8534-8543`) and
`ProcessBounds`.
`default: return false` (`:8596`).

**Consequence for our 251%-wrong and hanging cyl×cone**: OCCT's answer for a skew cylinder/cone is
an exact `ALine` from `IntAna_IntQuadQuad`, split at the apex. It is not marched and it is not
skipped.

---

#### 2.3.8 cylinder × sphere — `Perform(Cyl, Sph, Tol)`, `:1373-1405`
```
AxeOperator A1A2(Cyl.Axis(), Sph.Position().Axis());   // DEFAULT eps = 1e-14
if ((A1A2.Intersect() && sphCentre.Distance(A1A2.PtIntersect()) == 0.0)   // :1378  EXACT == 0.0
    || A1A2.Same()) {
    if (Sph.Radius() < Cyl.Radius()) → Empty
    else {
        dist = sqrt(Rs² - Rc²) ;  dir = cylAxisDir
        typeres = Circle ; pt1 = centre + dist*dir ; param1 = Rc ; nbint = 1
        if (dist > RealEpsilon())    // 2.22e-16
            pt2 = centre - dist*dir ; param2 = Rc ; nbint = 2
    }
} else typeres = NoGeometricSolution
```
The `== 0.0` test is exact. Combined with `Intersect()` (which already demands `< 1e-14`), this
in practice requires the sphere centre to be *constructed on* the cylinder axis. **Snap centres onto
axes at recognition time; this is confirmed load-bearing for rotated frames.**

Emission (`IntCySp`, `:8107-8370`): `Point` tangent-flagged `true` (`:8160`).
`Circle, NbSol == 1` → tangency (§2.6.2). `Circle, NbSol == 2` → first circle dead band `> 0.0`
(`:8227`), **second** circle dead band `1e-7` (`:8243,8248`).
`NoGeometricSolution` → `IntAna_IntQuadQuad anaint(Cy, Sp, Tol)` (`:8266`) → ALines, no
`ExploreCurve` (spheres have no apex), tangent-probe ladder `:8308-8317`, dead band `1e-8`
(`:8322,8327`), `ProcessBounds` with both `procf`/`procl` forced `false` (`:8342-8343`).
`default: return false` (`:8365`).

---

#### 2.3.9 sphere × sphere — `Perform(Sph1, Sph2, Tol)`, `:2034-2136`
Fully general, no pose gate. Canonicalisation: the line of centres.
```
dO1O2 = |O1 - O2| ;  Rmin = min(R1,R2) ; Rmax = max(R1,R2) ;  typeres = Empty
if (dO1O2 <= Tol && |R1-R2| <= Tol) → Same                            // :2057-2060
if (dO1O2 <= Tol) → return (Empty, concentric different radii)         // :2063-2066
Dir = unit(O2 - O1)
t = Rmax - dO1O2 - Rmin
if (0 <= t <= Tol) {                        // INTERNAL tangency        // :2079-2094
    typeres = Point ; nbint = 1
    t2 = (R1 == Rmax) ? (R1 + R2 + dO1O2)*0.5 : (-R1 + dO1O2 - R2)*0.5
    pt1 = O1 + t2*Dir
} else if (dO1O2 > R1+R2+Tol || Rmax > dO1O2+Rmin+Tol) → Empty         // :2105-2108
else {
    Alpha = 0.5*(R1² - R2² + d²)/d ;  Beta = sqrt(max(0, R1² - Alpha²))
    if (Beta <= myEPSILON_MINI_CIRCLE_RADIUS /*1e-9*/) {               // :2119
        typeres = Point ; Alpha = (R1 + dO1O2 - R2)*0.5               // EXTERNAL tangency
    } else { typeres = Circle ; dir1 = Dir ; param1 = Beta }
    pt1 = O1 + Alpha*Dir ;  nbint = 1
}
```
Emission (`IntSpSp`, `:9473-9554`): `Point` tangent-flagged `true` (`:9517`); `Circle` transversal,
dead band `1e-8` (`:9530,9535`); `default: return false` (`:9549`).

This pair is the cleanest evidence that our 0/20 on sphere×sphere is not a mathematics problem —
OCCT solves it in 100 lines with no pose restriction whatsoever.

---

#### 2.3.10 cone × cone — `Perform(Con1, Con2, Tol)`, `:1433-1890`
```
tg1 = tan(β1) ;  tg2 = tan(β2) ;  if (tg1*tg2 < 0) tg2 = -tg2         // :1443-1449
TOL_APEX_CONF = 1.e-10                                                 // :1440
aTol2 = Tol² ;  aDA1A2 = |apex1 - apex2|²                              // squared distance
AxeOperator A1A2(Con1.Axis(), Con2.Axis());        // DEFAULT eps = 1e-14
aTolAng = myEPSILON_ANGLE_CONE (1e-12)
if (|tg1 - tg2| < Tol && A1A2.Parallel() && A1A2.Distance() > 100*Tol) {   // :1459-1462
    aMinSolDist = EstimDist(Con1, Con2)
    aTolAng = (aMinSolDist < Epsilon(1.)) ? Tol
                                          : min(max(1e-12, Tol/aMinSolDist), Tol)  // :1465-1473
}
```

**Case 1 — `A1A2.Same()` (coaxial)** (`:1478-1522`), `d = axis1 · (apex2 - apex1)`:
- `|tg1 - tg2| > 1e-12`:
  - `|d| < 1e-10` → **Point** at apex1 (`:1488-1494`);
  - else 2 **Circles**: `x = d·tg2/(tg1+tg2)` and `x = d·tg2/(tg2-tg1)`,
    `pt = apex1 + x·axis1`, `param = |x·tg1|`, `dir1 = dir2 = axis1` (`:1495-1504`).
- `|tg1 - tg2| <= 1e-12`:
  - `|d| < 1e-10` → **`Same`** (`:1508-1511`);
  - else 1 **Circle** at `x = d/2`, radius `|x·tg1|` (`:1513-1520`).

**Case 2 — parallel, equal semi-angle within `aTolAng`** (`:1524-1606`):
Build the plane that must contain the solution conic, then **recurse into plane × cone**:
```
DA1 = axis1 dir ;  O1O2 = apex2 - apex1 ;  O1O2n = unit(O1O2)
DB1 = unit(O1O2n - (DA1·O1O2n) DA1)               // in-plane perpendicular
yO1O2 = O1O2 · DA1 ;  ABSTG1 = |tg1|
X2 = (DistA1A2/ABSTG1 - yO1O2)*0.5 ;  X1 = X2 + yO1O2
P1 = apex1 + X1*(DA1 + ABSTG1*DB1)
MO1O2 = midpoint(apex1, apex2)
OrthoPln = (DA1 × DB1) × unit(P1 - MO1O2)
IntAna_QuadQuadGeo INTER_QUAD_PLN(gp_Pln(P1, OrthoPln), Con1, Tol, Tol);   // :1555
```
**`Tolang = Tol` in the recursion** — ~10× looser than the `1e-8` the direct plane×cone path gets
from `IntPatch_ImpImpIntersection.cxx:2551`. Circle/parabola/hyperbola boundaries shift accordingly.
The switch then copies out `Ellipse` / `Circle` / `Hyperbola` (`nbint = 2`) / `Line` (`nbint = 2`,
both from `Line(1)`, params zeroed), `default → NoGeometricSolution` (`:1558-1604`).

**Case 3 — coincident apices, `aDA1A2 < aTol2`** (`:1608-1758`):
A 2D pre-analysis in the plane spanned by the two axes decides the count:
```
aHalfPI = π/2 ;  aD1 = 1 ;  aPA1 = (1,0) ;  aP0 = (0,0)
aGamma = angle(axis1, axis2) ;  if (aGamma > π/2) aGamma = π - aGamma
aTgBeta1 = |tan(β1)| ;  aTgBeta2 = |tan(β2)|
aR1 = aD1*aTgBeta1 ;  aP1 = (aD1, aR1)
aVAx2 = (cos aGamma, sin aGamma) ;  aPA2 = aP0 + (aVAx2·(aP1-aP0))·aVAx2
aR2 = |aPA2 - aP0| * aTgBeta2 ;  aRD2 = |aPA2 - aP1|
if (aRD2 > aR2 + Tol) → Empty, return                                  // :1667-1671
iRet = (aRD2 < aR2 - Tol) ? 2 : 1                                      // :1673-1677
```
Then the 3D construction (`:1681-1757`):
```
aQApex1 = apex1 ;  aQA1 = aQApex1 + 1.0*axis1dir
if (axis1dir · axis2dir < 0) axis2 := axis2.Reversed()                 // :1693-1697
aD2 = sqrt((1 + aTgBeta1²)/(1 + aTgBeta2²))
aQA2 = aQApex1 + aD2*axis2dir
aPln1 = plane(aQA1, axis1dir) ;  aPln2 = plane(aQA2, axis2dir)
aIntr.Perform(aPln1, aPln2, Tol, Tol);                                 // :1709  Tolang = Tol
if (!IsDone() || NbSolutions()==0) → NoGeometricSolution, return       // :1710-1714
aLin = aIntr.Line(1) ;  aOrig = aLin.Location() ;  aVLin = aLin.Direction()
aQX = aOrig + ((aOrig - aQA1)·aVLin) * aVLin                           // :1720-1722
typeres = Line ;  param1 = param2 = param1bis = param2bis = 0
if (iRet == 1) { nbint = 1 ; pt1 = apex1 ; dir1 = unit(aQX - apex1) }
else           { nbint = 2 ; aDa = |aQA1 - aQX| ; aDx = sqrt(aR1² - aDa²)
                 aQX1 = aQX + aDx*aVLin ; aQX2 = aQX - aDx*aVLin
                 pt1 = pt2 = apex1 ; dir1 = unit(aQX1 - apex1) ; dir2 = unit(aQX2 - apex1) }
```
The `aQX` formula looks like a sign error (the perpendicular foot from `aQA1` would be
`aOrig + ((aQA1 - aOrig)·d) d`). **It is harmless here and must be copied as written**: both plane
locations `aQA1` and `aQA2` are perpendicular to `d` through the same point (because
`d ⊥ axis1` and `d ⊥ axis2`, and the apices coincide), so `(aOrig - aQA1)·d = 0` and
`aQX = aOrig` — which *is* the correct foot. A port that "fixes" the sign gets the same answer here
and a different one if this helper is reused elsewhere.

**Case 4 — `A1A2.Intersect()` (axes meet within 1e-14) with a common generatrix** (`:1760-1884`):
```
ElSLib::Parameters(Con2, apex1, u,v) ; if (|apex1 - ElSLib::Value(u,v,Con2)|² > Tol²)
    → NoGeometricSolution                                              // :1764-1770
ElSLib::Parameters(Con1, apex2, u,v) ; if (|apex2 - ElSLib::Value(u,v,Con1)|² > Tol²)
    → NoGeometricSolution                                              // :1772-1778
myCommonGen = true                                                     // :1781
aGen  = line(apex1, unit(apex2 - apex1))
myPChar = ElCLib::Value(ElCLib::Parameter(aGen, A1A2.PtIntersect()), aGen)   // :1790-1791
aGen1 = aGen rotated π about Con1.Axis() ;  aGen2 = aGen rotated π about Con2.Axis()
aD1 = aGen1.Direction() ;  aD2 = aD1 × aGen.Direction()
if (aD1 ∥ aGen2.Direction() within Precision::Angular())  aN = aD1 × aD2      // :1803-1806
else if (aGen1.SquareDistance(aGen2) > Tol²) → NoGeometricSolution            // :1807-1812
else { <closed-form line/line intersection point aPGint>                      // :1815-1824
       aD1 = unit(myPChar - aPGint) ;  aN = aD1 × aD2 }
IntAna_QuadQuadGeo INTER_QUAD_PLN(gp_Pln(myPChar, aN), Con1, Tol, Tol);       // :1832
switch → Ellipse | Circle | Parabola | Hyperbola ; default → NoGeometricSolution
```

**Case 5 — everything else** → **`NoGeometricSolution`** (`:1886-1889`).
That is: two cones with skew axes, or intersecting axes without a common generatrix, or non-parallel
non-coaxial. **This is the general position, and OCCT solves it with `IntAna_IntQuadQuad`.**

Emission (`IntCoCo`, `:8679-9175`):
- `Line, NbSol == 1` → 2 GLines (both rays), `Touch` + situation from
  `aDot = N1(ptbid)·N2(ptbid)`: `aDot < 0 → (Outside, Outside)`; else compare the two axis
  distances `aR1 = dist(ptbid, axis1)`, `aR2 = dist(ptbid, axis2)`: base `(Inside, Outside)`,
  flipped to `(Outside, Inside)` when `aR1 > aR2` (`:8732-8788`).
- `Line, NbSol == 2` → 4 GLines, transversal, dead band `> 0.0` (`:8803`), `Multpoint = true`.
- `Point` → **nappe purge**: keep with `isTangent = false` when the contact is within `Tol` of
  *both* apices; keep with `isTangent = true` when `param1 >= paramapex1 && param2 >= paramapex2`;
  **otherwise silently drop** (`:8835-8866`).
- `Circle` / `Ellipse` / `Hyperbola` / `Parabola` → transversal, dead band `1e-8`, and if
  `inter.HasCommonGen()` the characteristic point `PChar` is added as a vertex with
  **`SetParameter(0.)` regardless of its true conic parameter** (`:8893-8902, 8932-8941, 8970-8979,
  9008-9017`).
- `NoGeometricSolution` → `IntAna_IntQuadQuad anaint(Co1, Co2, Tol)` (`:9024`) → ALines; note the
  extra degenerate-derivative guard `tgvalid.SquareMagnitude() < 1e-14` (`:9080-9084`) and the
  `1e-9` dead band (`:9095,9100`).
- After the switch, if `HasCommonGen()`, the shared generatrix itself is emitted as an extra
  tangent GLine with `IntSurf_Undecided, IntSurf_Undecided` (`:9149-9172`).
- `default: return false` (`:9142`).

---

#### 2.3.11 sphere × cone — `Perform(Sph, Con, Tol)`, `:1917-2005`
```
AxeOperator A1A2(Con.Axis(), Sph.Position().Axis());     // DEFAULT eps = 1e-14
if ((A1A2.Intersect() && sphCentre.Distance(A1A2.PtIntersect()) == 0.0) || A1A2.Same()) {  // :1926
    dApexSphCenter = |sphCentre - apex|
    ConDir = (dApexSphCenter > RealEpsilon()) ? unit(sphCentre - apex) : coneAxisDir  // :1931-1938
    tga = tan(β) ;  tgatga = tga²
    math_DirectPolynomialRoots Eq(1 + tgatga,  2*tgatga*dApexSphCenter,
                                  -R² + dApexSphCenter²*tgatga);          // :1947-1949
    if (!Eq.IsDone()) { done = false; return; }                           // :1996-1999  give-up
    nbsol = Eq.NbSolutions()
    if (nbsol == 0) → Empty
    else { typeres = Circle
           for k in 1..min(nbsol,2):
               x = Eq.Value(k) ;  s = dApexSphCenter + x
               pt_k = apex + s*ConDir ;  param_k = |tga*s| ;  dir_k = ConDir ; nbint = k
               if (param_k <= myEPSILON_MINI_CIRCLE_RADIUS /*1e-9*/) {
                   typeres = PointAndCircle ;  param_k = 0.0 }            // :1971-1975, 1988-1992
    }
} else typeres = NoGeometricSolution
```
`PointAndCircle` is decided **per root**; if both roots degenerate, `param1 == param2 == 0` and the
accessors (`Point` keys on `param1 == 0.0`, `:2686`; `Circle` keys on `param2 == 0.0`, `:2741`)
return a zero-radius circle.

Emission (`IntCoSp`, `:9179-9470`):
- `Point` → nappe purge, same predicate as cone×cone but single-cone: contact within `Tol` of the
  apex → `isTangent = false`; else `param >= paramapex` → `isTangent = true`; else **dropped**
  (`:9226-9253`).
- `Circle` → transversal, dead band `1e-8` (`:9269,9274`).
- `PointAndCircle` → the apex is emitted as a point with `isTangent = false` (`:9301`), and the
  circle's transition **sense flips by nappe**: if the circle centre's axis parameter is `>=` the
  apex parameter, `qwe > PConfusion → (Out,In)`; otherwise the two are **swapped**
  (`:9310-9343`). Dead band `Precision::PConfusion() = 1e-9`.
- `NoGeometricSolution` → `IntAna_IntQuadQuad anaint(Co, Sp, Tol)` (`:9351`) → ALines,
  tangent-probe ladder `:9402-9411`, dead band `1e-9` (`:9416,9421`).
- `default: return false` (`:9464`).

---

#### 2.3.12 cylinder × torus — `Perform(Cyl, Tor, Tol)`, `:2278-2330`
```
if (Rmin >= Rmaj) → NoGeometricSolution                                // :2286-2290
if (!torusAxis.IsParallel(cylAxis, 1e-12) || dist(torusAxisLine, cylLoc) > 1e-14)
    → NoGeometricSolution                                              // :2298-2303   COAXIAL ONLY
if (Rc + Tol < Rmaj - Rmin  ||  Rc - Tol > Rmaj + Rmin) → Empty        // :2308-2312
typeres = Circle
aDist = sqrt(|Rmin² - (Rc - Rmaj)²|)
dir1 = torusAxisDir ;  pt1 = torusLoc + aDist*dir1 ;  param1 = Rc ;  nbint = 1
if (aDist > Tol && Rc > Rmaj-Rmin && Rc < Rmaj+Rmin)
    pt2 = torusLoc - aDist*dir1 ;  param2 = Rc ;  nbint = 2
```

#### 2.3.13 cone × torus — `Perform(Con, Tor, Tol)`, `:2357-2469`
```
if (Rmin >= Rmaj) → NoGeometricSolution
if (!torusAxis.IsParallel(coneAxis, 1e-12) || dist(torusAxisLine, apex) > 1e-14)
    → NoGeometricSolution                                              // :2377-2382   COAXIAL ONLY
aPN  = torusLoc + Rmaj*Tor.YAxis().Direction()
aDN  = unit(aPN - torusLoc) ;  anAxCLRot = axis(apex, aDN)
aConL = torusAxisLine rotated by SemiAngle about anAxCLRot             // a cone generatrix
aDL  = aConL.Direction() ;  aXDir = Tor.XAxis().Direction()
typeres = Empty
for i in {0,1}:                                                        // :2401-2435
    if (i) aXDir.Reverse()
    aPCT = torusLoc + Rmaj*aXDir                                       // tube centre on this side
    aDist = aConL.Distance(aPCT) ;  if (aDist > Rmin + Tol) continue
    typeres = Circle
    aPh = aPCT - aDist * aConL.Normal(aPCT).Direction()
    aDt = sqrt(|Rmin² - aDist²|)
    aP  = aPh + aDt*aDL ; aParam[nbint] = dist(torusAxisLine, aP)
    aPt[nbint] = aP - aParam[nbint]*aXDir ; aDir[nbint] = torusAxisDir ; ++nbint
    if (aDist < Rmin && aDt > Tol) {                                   // strict <
        aP = aPh - aDt*aDL ; aParam[nbint] = dist(torusAxisLine, aP)
        aPt[nbint] = aP - aParam[nbint]*aXDir ; aDir[nbint] = aDir[nbint-1] ; ++nbint }
then the switch at :2437-2468 fills (pt1..pt4, param1..4, dir1..4)     // up to 4 circles
```

#### 2.3.14 sphere × torus — `Perform(Sph, Tor, Tol)`, `:2496-2561`
```
if (Rmin >= Rmaj) → NoGeometricSolution
if (dist(torusAxisLine, sphCentre) > 1e-14) → NoGeometricSolution      // :2514-2518  ON-AXIS ONLY
aXDir   = Tor.XAxis().Direction()
aTorLoc = torusAxisLoc + Rmaj*aXDir                                    // tube centre
aDist   = |sphCentre - aTorLoc|
if (aDist - Tol > Rmin + Rs  ||  aDist + Tol < |Rmin - Rs|) → Empty    // :2529-2533
typeres = Circle
anAlpha = 0.5*(Rmin² - Rs² + aDist²)/aDist ;  aBeta = sqrt(|Rmin² - anAlpha²|)
aDir12 = unit(sphCentre - aTorLoc)
aPh = aTorLoc + anAlpha*aDir12 ;  aDC = Tor.YAxis().Direction() ^ aDir12
aP  = aPh + aBeta*aDC ; param1 = dist(torusAxisLine, aP)
pt1 = aP - param1*aXDir ; dir1 = torusAxisDir ; nbint = 1
if (aDist < Rs+Rmin && aDist > |Rs-Rmin| && aBeta*|aDC| > Tol)         // :2553
    aP = aPh - aBeta*aDC ; param2 = dist(torusAxisLine, aP)
    pt2 = aP - param2*aXDir ; dir2 = dir1 ; nbint = 2
```

#### 2.3.15 torus × torus — `Perform(Tor1, Tor2, Tol)`, `:2588-2666`
```
if (!axis1.IsParallel(axis2, 1e-12) || dist(axis1Line, loc2) > 1e-14)
    → NoGeometricSolution                                              // :2606-2610  COAXIAL ONLY
if (loc1.IsEqual(loc2, Tol) && |Rmin1-Rmin2| <= Tol && |Rmaj1-Rmaj2| <= Tol) → Same  // :2612-2617
if (Rmin1 >= Rmaj1 || Rmin2 >= Rmaj2) → NoGeometricSolution            // :2619-2623
aP1 = loc1 + Rmaj1*Tor1.XAxis().Dir ;  aP2 = loc2 + Rmaj2*Tor1.XAxis().Dir   // NOTE: Tor1's XAxis both
aDist = |aP2 - aP1|
if (aDist - Tol > Rmin1+Rmin2 || aDist + Tol < |Rmin1-Rmin2|) → Empty  // :2634-2638
<same circle construction as sphere×torus, with Rmin1/Rmin2>           // :2640-2665
```
**Order quirk**: the `Same` test (`:2612-2617`) precedes the ring-validity test (`:2619-2623`), so
two identical self-intersecting tori still report `Same`.

---

### 2.4 Where OCCT itself gives up — the complete, verified list

A "give-up" here means: the analytic layer returns something the caller must handle by falling back.
Documenting these is part of the port; inventing solutions for them is not.

| site | condition | what happens next |
|---|---|---|
| `IntAna_QuadQuadGeo.cxx:925-950` | plane×cone conic exceeds `EllipseLimit = 1e9` / `HyperbolaLimit = 2e6` | `done = false` → `IntPCo` returns `false` → `ImpImp` Fail → `ParamParamPerfom` |
| `IntAna_QuadQuadGeo.cxx:481-503` | plane×plane origin refinement fails or is parallel | `done` stays `false` → `IntPP` returns `false` → Fail |
| `IntAna_QuadQuadGeo.cxx:1996-1999` | sphere×cone quadratic `math_DirectPolynomialRoots` not done | `done = false` → `IntCoSp` returns `false` → Fail |
| `IntAna_QuadQuadGeo.cxx:1710-1714`, `:1768,1776,1810` | cone×cone recursion produces no line / apex not on other cone / generatrices too far | `NoGeometricSolution` → `IntAna_IntQuadQuad` ALine path |
| `IntAna_IntQuadQuad.cxx:440-444` | `|Qzz| < myEpsilonCoeffPolyNull = 1e-8` (cylinder path) | `done = false` → caller returns `false` → Fail |
| `IntAna_IntQuadQuad.cxx:462,901,920,994` | any `TrigonometricRoots` not done | `done = false` → Fail |
| `IntAna_IntQuadQuad.cxx:1079-1083` | cone path: `|Theta2 - Theta1| <= myEpsilon (1e-8)` | `done = false` → **kills the whole pair** (the cylinder path merely skips that interval) |
| `IntAna_IntQuadQuad.cxx:956-959` | cone path, `PolZ2` and `PolZ1` both infinite-root and `|Q1| <= 1e-8` | `done = false` |
| `IntPatch_ImpImpIntersection.cxx:5176-5178` | `CyCyAnalyticalIntersect` sees `Parabola`/`Hyperbola` | **throws** `Standard_Failure` |
| `IntPatch_ImpImpIntersection.cxx:5180-5185` | `CyCyAnalyticalIntersect` sees `Circle` or `NoGeometricSolution` | `false` → `IntCyCy` clears state (`:7924-7931`) → numeric cyl×cyl (§2.7) |
| `IntPatch_ImpImpIntersection.cxx:6603-6608` | numeric cyl×cyl: `V`-range > `1e5 · radius` | `IntStatus_InfiniteSectionCurve` |
| `IntPatch_ImpImpIntersection.cxx:9707-9710` | `TreatResultTorus` sees anything but `Empty`/`Circle` | `false` → Fail → `ParamParamPerfom` (**this is the torus marcher**) |
| `IntPatch_ImpImpIntersection.cxx:3340,3434,3781,8596,9464,9549,9142` | the `default:` arm of `IntPCy/IntPSp/IntPCo/IntCyCo/IntCoSp/IntSpSp/IntCoCo` | `false` → Fail |
| `IntPatch_ImpImpIntersection.cxx:8534-8543` etc. | tangent probe fails 6 times | ALine built with the 2-arg constructor → `IntSurf_Undecided` transitions |

Two more structural give-ups worth naming:
- `IntAna_IntQuadQuad::NbPnt()` is **always 0**. The cylinder path's isolated-tangent-point
  extraction is commented out (`IntAna_IntQuadQuad.cxx:626-631`) and the cone path never extracts
  points at all. So *cylinder tangent to cone* yields nothing: `Perform(Cyl,Con)` cannot return
  `IntAna_Point`, and the ALine solver drops the isolated tangency. This is the grazing-residue class.
- `InternalSetNextAndPrevious()` (end-to-end ALine chaining) is called only from the **cone** path
  (`IntAna_IntQuadQuad.cxx:1395`); the cylinder path returns at `:825` without chaining, so
  cyl×quadric ALines never get `next/previouscurve` links.

### 2.5 The exact ALine solver — `IntAna_IntQuadQuad`

This is the general-position answer for cyl×{cone,sphere,plane} and cone×{cone,sphere,cylinder,plane}.
It is **not** a marcher: it produces a closed-form algebraic curve `θ ↦ (u, v)` with an exact
evaluator.

**Implicit operand** — `IntAna_Quadric` (`IntAna_Quadric.cxx`):
```
f(x,y,z) = CXX x² + CYY y² + CZZ z² + 2(CXY xy + CXZ xz + CYZ yz) + 2(CX x + CY y + CZ z) + CCte
```
Constructors: plane (`:58-74`, with `CX,CY,CZ` halved and all quadratic terms zero), cylinder
(`:79-87`), cone (`:97-102`), sphere (`:107-117`). **There is no torus constructor** — the implicit
operand can only be plane/cylinder/cone/sphere. `mySpecialPoints` accumulates the cone apex
(`ElSLib::Value(0, -RefRadius/sin(SemiAngle), Cone)`, `:100-101`) and both sphere poles
(`v = ±π/2`, `:110-111`); it is **never cleared**, so a reused `IntAna_Quadric` accumulates.

`NewCoefficients(..., gp_Ax3 Axis)` (`:148-248`) rewrites the coefficients in the frame `Axis` by
inverting `gp_Trsf::SetTransformation(Axis)` and substituting; copy the 10 expressions verbatim
(`:203-236`).

**Parametric operand — cylinder path** (`IntAna_IntQuadQuad.cxx:375-825`):
```
Quad.NewCoefficients(..., Cyl.Position())            // :413
if (|Qzz| < 1e-8) { done = false; return; }          // :440-444
R2 = RCyl²
C_1  = Qz² - Qzz*Q1
C_SS = R2*(Qyz² - Qyy*Qzz) ;  C_CC = R2*(Qxz² - Qxx*Qzz)
C_S  = RCyl*(Qyz*Qz - Qy*Qzz) ; C_C  = RCyl*(Qxz*Qz - Qx*Qzz)
C_SC = R2*(Qxz*Qyz - Qxy*Qzz)                        // :452-457
MTF  = MyTrigonometricFunction(C_CC, C_SS, C_SC, C_C, C_S, C_1)
PolDIS = TrigonometricRoots(C_CC - C_SS, C_SC, 2*C_C, 2*C_S, C_1 + C_SS, 0, 2π)   // :460
```
Then, by root count of the discriminant:
- `InfiniteRoots` → 2 curves over `[0, 2π]`, `UN_SEUL_Z_PAR_THETA`, `Z_POSITIF`/`Z_NEGATIF`
  (`:468-503`).
- `nbsolDIS == 0` → if `MTF.Value(π) >= -RealEpsilon()` 2 curves over `[0, 2π]`, else 0 curves
  (`:511-563`).
- `nbsolDIS == 1` → if `MTF.Value(root + π) >= -RealEpsilon()` 2 curves over `[0, 2π]`, else 0
  (isolated tangent point **discarded**) (`:571-632`).
- `nbsolDIS >= 2` (`:634-819`):
  - a pre-pass **only when `nbsolDIS == 2`** (`:660`): if `|Theta2 - Theta1| <= RealEpsilon()`
    (2.2e-16), probe at `Theta1 ∓ 0.1` (`:673-679`); if `MTF >= 0`, emit 2 curves over
    `[Theta1, Theta1 + 2π]` after `AddSpecialPoints`, and set `UnPtTg`;
  - otherwise, for each root interval `[Theta_i, Theta_{i+1}]` (wrapping the last to
    `Theta_1 + 2π`): skip if `|Δ| <= 1e-12` (`:731`); the positivity test is a **3-sample SUM**
    `MTF(0.5(T1+T2)) + MTF(0.4T1+0.6T2) + MTF(0.6T1+0.4T2) >= 0` (`:738-740`) — a narrow negative
    dip is outvoted; then if the *next* root is within `5e-8` (`:757`) emit **two** single-`Z`
    curves over `[T1,T2]`, else **one** `DEUX_Z_PAR_THETA` curve (`:796-816`).

**Parametric operand — cone path** (`:841-1396`):
```
tAx3 = Cone.Position() with Location := Cone.Apex()     // :868-869
Quad.NewCoefficients(..., tAx3)
TgAngle = 1/tan(SemiAngle)                              // :872
A(t): Z2CC=Qxx, Z2SS=Qyy, Z2Cte=Qzz*TgAngle², Z2SC=Qxy, Z2C=TgAngle*Qxz, Z2S=TgAngle*Qyz
      PolZ2 = TrigonometricRoots(Z2CC-Z2SS, Z2SC, 2*Z2C, 2*Z2S, Z2Cte+Z2SS, 0, 2π)    // :900
B(t): Z1Cte=2*TgAngle*Qz, Z1S=Qy, Z1C=Qx, rest 0
      PolZ1 = TrigonometricRoots(...)                                                  // :919
Discriminant: C_1  = TgAngle²*(Qz² - Qzz*Q1)
              C_SS = Qy² - Qyy*Q1 ;  C_CC = Qx² - Qxx*Q1
              C_S  = TgAngle*(Qy*Qz - Qyz*Q1) ; C_C = TgAngle*(Qx*Qz - Qxz*Q1)
              C_SC = Qx*Qy - Qxy*Q1                                                    // :986-991
              Pol  = TrigonometricRoots(...)                                           // :993
```
Control flow: `PolZ2.InfiniteRoots()` (A ≡ 0) → linear-in-`Z` special cases (`:928-967`);
`Pol.InfiniteRoots()` → 2 curves over `[0, 2π]` (`:1003-1042`);
`!nbsol && MTF(π) < 0` → nothing (`:1047-1051`);
`!nbsol` otherwise → `DiscriminantConstantPositif = true`, `nbsol = 1` (`:1056-1061`), and then
**`Theta1 = 0, Theta2 = 2π - 1e-8`** (`:1070-1071`) — a closed curve is delivered 1e-8 short of
closure, so its chain never links;
`|Theta2 - Theta1| <= 1e-8` → **`done = false; return;`** (`:1079-1083`) — the cone path *aborts*
where the cylinder path merely skips;
the 3-sample-sum positivity test again (`:1085-1090`);
then the interval is subdivided at every root of `A(t)` inside it (`:1102-1259`), and each
sub-interval produces a `Z_NEGATIF` and a `Z_POSITIF` curve, with `SetIsFirstOpen` / `SetIsLastOpen`
decided by `sign(B)` at the endpoint (`:1236-1254`, `:1301-1322`, `:1366-1387`);
finally `InternalSetNextAndPrevious()` (`:1395`).

`AddSpecialPoints` (`:61-118`): widens `[T1,T2]` by the max out-of-domain `u`-delta of any special
point whose `ElSLib` round-trip satisfies `SquareDistance < Precision::SquareConfusion()` (`:83`),
then **caps the width at `2π`** (`:113-116`).

`TrigonometricRoots` wrapper (`:199-285`): roots normalised into `[0,2π]` *before* sorting
(`:230-241`); a **residual reject** `|F(root)| > 1e-8` sets `done = false` for the whole solve
(`:245-256`); infinite-roots detection is `Σ|coef| < 1e-10 && |Cte| < 1e-10` (`:275-284`).

`math_TrigonometricFunctionRoots` (Weierstrass `t = tan(θ/2)`), quartic coefficients
`ko = {A-C+E, 2D-4B, 2E-2A, 4B+2D, A+C+E}` (`math_TrigonometricFunctionRoots.cxx:358-362`).
`Eps = 1.5e-12` on raw coefficients (`:93`); Newton `Tol1 = 1e-15`, `Nit = 10` (`:87`), rejected if
it moves more than 1% of the interval (`SupmInfs100`, `:439,470-478`).
**θ = π is not representable** by the substitution; it is re-inserted by a trailing special case only
when `|A - C + E| <= Eps` (`:506-540`), which can append π up to 4 times, deduplicating only on the
first iteration (`:526-529`).
**Coefficient-rescaling loop** (`:402-434`): if two sorted roots differ by `< 1.5e-12` but the
quartic derivative there exceeds `1.5e-12`, all five coefficients are multiplied by `1e-4` and the
solve is repeated until the test passes. Root sets for grazing configurations are therefore
magnitude-dependent; reproduce the loop or normalise by `max|coef|` and know you are off-parity.

**ALine evaluator — `IntAna_Curve`** (`IntAna_Curve.cxx`):
`SetConeQuadValues` (`:95-155`) and `SetCylinderQuadValues` (`:161-217`) store 18 coefficients
`Z{0,1,2}{Cte,Sin,Cos,SinSin,CosCos,CosSin}` (note: **no** factor 2 on the `Cos`/`Sin`/`CosSin`
terms, comment `:125-130`), plus `TwoCurves`, `TakeZPositive`, `[DomainInf, DomainSup]`, and
`myLastParameter = TwoCurves ? 2*DomainSup - DomainInf : DomainSup`.

`InternalUVValue(θ)` (`:279-374`):
```
aDT = 100 * Epsilon(2*DomainSup - DomainInf)                              // :292
out-of-domain (with relative slack Epsilon(1)) → throw Standard_DomainError  // :297-302
if (|θ - DomainSup| < aDT) θ = DomainSup                                  // :304-308
else if (θ > DomainSup) { θ = 2*DomainSup - θ ; SecondSolution = true }   // :309-313
A = Z2Cte + sint(Z2Sin + sint*Z2SinSin) + cost(Z2Cos + cost*Z2CosCos) + Z2CosSin*sin2t
B, C analogous with Z1*, Z0*
aDA = cost*Z2Sin - sint*Z2Cos + sin2t*(Z2SinSin - Z2CosCos) + cos2t*(Z2CosSin * Z2CosSin)  // :330-331
aDB = Z1Sin*cost - Z1Cos*sint + sin2t*(Z1SinSin - Z1CosCos) + cos2t*(Z1CosSin + Z1CosSin)
aDC = Z0Sin*cost - Z0Cos*sint + sin2t*(Z0SinSin - Z0CosCos) + cos2t*(Z0CosSin + Z0CosSin)
disc = B² - 4AC
aTolD = 2*aDT*|B*aDB - 2*(A*aDC + C*aDA)|                                 // :351
if (disc < aTolD) disc = 0.0                                              // :353-356  ONE-SIDED
if (|A| <= PConfusion) { Param2 = (|B| <= PConfusion) ? 0.0 : -C/B }      // :358-368
else { SigneSqrtDis = ±sqrt(disc) ; Param2 = (-B + SigneSqrtDis)/(2A) }
```
Three things to copy exactly:
1. **`aDA` uses `Z2CosSin * Z2CosSin`** where `aDB`/`aDC` use `(x + x)` (`:331` vs `:337,343`).
   Squared instead of doubled. It affects only `aTolD`, and only the cone path (`Z2CosSin = Qxy`;
   identically 0 for cylinders, `:206`).
2. **The discriminant snap is one-sided.** *Any* negative discriminant, however large, becomes 0, so
   `Value()` is total: it silently returns the tangency vertex `-B/(2A)` instead of failing. Raising
   on negative discriminant shortens domains vs OCCT; snapping symmetrically lengthens them.
3. **The linear fallback silently returns `V = 0.0`** when both `|A|` and `|B|` are below
   `PConfusion` (`:360-363`) — a point on the axis/apex circle, not an error.

`InternalValue(U, V)` (`:535-568`): **`V` is clamped to `±100000`** *before* evaluation (`:539-546`),
and the cone remap is `(V - RCyl)/sin(Angle)` (`:557`) even though the comment says `tan`.

`D1u(θ)` (`:397-431`): fails exactly in the tangent zone, `|A| < 1e-7 || |SigneSqrtDis| < 1e-10`
(`:412`); otherwise the derivative is a **finite difference** with
`dθ = (DomainSup - DomainInf)·1e-6`, sign-flipped near the far end (`:418-425`).

`FindParameter(P, params)` (`:440-531`): candidate set is
`{DomainInf, DomainSup, θ, 2*DomainSup-θ, 2*DomainSup-DomainInf}`; **`std::sort(aParams, aParams +
aMaxPar - 1)` sorts only the first 4 of 5** (`:493`); acceptance is a **squared** tolerance used as
if linear — `InternalPrecision = 1e-8` compared against `SquareDistance` (effective `1e-4` in 3D)
for the projected candidates, `Precision::SquareConfusion()` (effective `1e-7` in 3D) for the
boundary ones (`:442-445,516-529`).

### 2.6 Output encoding

#### 2.6.1 The three line encodings (`IntPatch_Line.cxx`)
| ctor | `tg` | `tS1/tS2` | `sit1/sit2` | meaning |
|---|---|---|---|---|
| `(Tang, Trans1, Trans2)` `:21-34` | as given | as given (`In`/`Out`/`Undecided`) | `Unknown` | transversal; `Undecided` = sign unresolvable |
| `(Tang, Situ1, Situ2)` `:36-49` | as given | **`IntSurf_Touch`** both | as given (`Inside`/`Outside`) | **tangency with a side** |
| `(Tang)` `:51-62` | as given | `IntSurf_Undecided` both | `Unknown` | transitions unknown |

`SituationS1()`/`SituationS2()` throw unless `tS == Touch`. **Do not collapse these three into one
tangent flag** — that loses the inside/outside verdict OCCT computed from a curvature-side test, and
it must then be re-derived from flux/quorum, which is exactly the residue class we are fighting.

#### 2.6.2 The situation (curvature-side) rules, per pair
- **plane × cylinder** (`:3205-3244`): `TestCurvature = (lineOrigin → Cy.Location())`;
  `Normp·TestCurvature > 0 → situcyl = Outside` else `Inside`; then
  `situp = (Normp·Normcyl > 0) ? opposite(situcyl-ish) : same` per the exact table at `:3221-3244`.
- **plane × cone** (`:3521-3566`): same shape, with `TestCurvature = (ptbid → apex + 5·axisDir)`,
  and it additionally computes the *other-side* pair for the second ray.
- **cylinder × sphere** (`:8174-8213`): `TestCurvature = (ptref → Sp.Location())`, probe is
  `Normcyl`.
- **cylinder × cylinder** (`:4906-4982`): uses **both** radius vectors `crb1 = (ptref → Cy1.Loc)`,
  `crb2 = (ptref → Cy2.Loc)`. If `crb1·crb2 < 0` (opposed curvature centres) each cylinder's
  situation is decided by the **other's** normal; otherwise the smaller-radius cylinder decides.
- **cone × cone** (`:8746-8773`): if `N1·N2 < 0` both are `Outside`; else the cone whose axis is
  **farther** from the probe point is `Outside`.

`IntSurf_Touch` is only ever produced on the **degenerate-count** branch (`NbSol == 1`). If a port
emits two nearly coincident curves instead of collapsing to one, it never enters the situation
branch and emits two transversal curves where OCCT emits one tangent curve. The collapse thresholds
are asymmetric (§2.3.6 branch 5) and must be copied.

#### 2.6.3 Same-domain by restriction coverage
`tgte` is set at exactly one place (`IntPatch_ImpImpIntersection.cxx:2865-2869`):
`SameSurf || (all1 && all2)`, where `allN = solrst.AllArcSolution() && typs1 == typs2`
(`:2795,2834`), and `allN` is cancelled when that face has no restrictions at all (`:2813-2816`,
`:2855-2858`). When it fires, `slin` and `spnt` are **cleared** (`:2870-2871`) and
`oppo = N1(Pref)·N2(Pref) < 0` with `Pref` a per-type canonical sample (`:2875-2901`).
So a pair whose case table says *not* `Same` still becomes tangent-same if every restriction arc of
both faces is a solution and the surface types match. Routing keyed only off the analytic `Same`
verdict misses this class.

#### 2.6.4 Vertices and parameters
- `ProcessBounds` receives `procf = !firstp = IsFirstOpen` (`:8574-8575`): **open (asymptotic) ends
  are treated as already processed and never get a vertex**. A closed ALine (`|ptf - ptl| <= Tol`)
  gets one *multiple* vertex duplicated at both ends (`:4794-4804`).
- GLines with no vertices get two synthesised ones at `ElCLib::Value(0.0, ...)`, parameters `0` and
  `2π` — **both at the same 3D point** (`:3000-3052`). Circles and ellipses only.
- `PChar` (cone×cone common generatrix) is added with `SetParameter(0.)` regardless of its true
  conic parameter (`:8900,8939,8977`).
- Vertex dedup tolerances are hard-coded and ignore the passed tolerance:
  `IntPatch_GLine.cxx:421` ignores its `Tol` argument and uses `1000*Precision::PConfusion() = 1e-6`
  (`:442`); `IntPatch_ALine.cxx:70` uses `1e-5`.
- `AdjustToSeam` re-seats a circle's X-direction onto the host quadric's seam
  (`:3863-3925`; `SeamPosition` = `gp_Ax2(circleLoc, quadric.Direction(), quadric.XDirection())`).
  Called from `IntPCy:3300`, `IntPSp:3413` (takes `Tolang`, and only acts if the circle axis is
  parallel to the sphere axis, `:3885-3890`), `IntPCo:3683`, `IntPTo:3827-3830` (**only when the
  plane axis is not normal to the torus axis**), `TreatResultTorus:9680-9683` (**only when both
  quadrics are the same type**).

### 2.7 Cylinder × cylinder in general position — OCCT's semi-analytic solver

This is the answer to "cyl×cyl skew with unequal radii". It is in
`IntPatch_ImpImpIntersection.cxx`, not in `IntAna`, and it is **closed-form per `U1`** — there is no
marching, no step control on a 3D curve, no Newton on a 2-surface system.

**Derivation** (`:3949-3990`). Writing both cylinders parametrically and eliminating, the system
reduces to
```
V1 = K11 sin U1 + K21 sin U2 + L11 cos U1 + L21 cos U2 + M1
V2 = K12 sin U1 + K22 sin U2 + L12 cos U1 + L22 cos U2 + M2      (4)
cos(U2 - FI2) = B·cos(U1 - FI1) + C                              (5)
```
so for any `U1`, `U2 = FI2 ± acos(B cos(U1-FI1) + C)` — **two** branches, i.e. two WLines.

**Coefficient construction** `stCoeffsValue::stCoeffsValue` (`:4063-4250`):
```
A1 = -R1*Cyl1.XAxis().Dir ;  A2 =  R2*Cyl2.XAxis().Dir
B1 = -R1*Cyl1.YAxis().Dir ;  B2 =  R2*Cyl2.YAxis().Dir
C1 =  Cyl1.Axis().Dir     ;  C2 = -Cyl2.Axis().Dir
D  =  Cyl2.Location() - Cyl1.Location()
pick the 2 of 3 rows with the largest 2×2 minor of [C1 C2]     // :4083-4115, COE12/COE23/COE13
if (|aDetV1V2| < Precision::Angular())  throw Standard_Failure  // :4123-4126
permute all seven vectors' components to put the chosen rows first  // :4128-4203
K21 = (C2(2)B2(1) - C2(1)B2(2))/det ;  K11 = (C2(2)B1(1) - C2(1)B1(2))/det
L21 = (C2(2)A2(1) - C2(1)A2(2))/det ;  L11 = (C2(2)A1(1) - C2(1)A1(2))/det
M1  = (C2(2)D(1)  - C2(1)D(2))/det                                     // :4207-4215
K22 = (C1(1)B2(2) - C1(2)B2(1))/det ;  K12 = (C1(1)B1(2) - C1(2)B1(1))/det
L22 = (C1(1)A2(2) - C1(2)A2(1))/det ;  L12 = (C1(1)A1(2) - C1(2)A1(1))/det
M2  = (C1(1)D(2)  - C1(2)D(1))/det                                     // :4220-4228
ShortCosForm(L11,K11 → K1,FIV1) ; (L21,K21 → L1,PSIV1) ;
             (L12,K12 → K2,FIV2) ; (L22,K22 → L2,PSIV2)                // :4231-4234
aA1 = C1(3)K21 + C2(3)K22 - B2(3) ;  aA2 = C1(3)L21 + C2(3)L22 - A2(3)
aB1 = B1(3) - C1(3)K11 - C2(3)K12 ;  aB2 = A1(3) - C1(3)L11 - C2(3)L12
C   = D(3) - C1(3)M1 - C2(3)M2
ShortCosForm(aB2,aB1 → B,FI1) ;  ShortCosForm(aA2,aA1 → aA,FI2)
B /= aA ;  C /= aA                                                      // :4245-4249
```
`ShortCosForm(cosFactor, sinFactor, coeff, angle)` (`:5197-...`) writes
`cosFactor·cos A + sinFactor·sin A = coeff·cos(A - angle)`, with `aNulValue = 1.0e-11` as the
"treat as zero" threshold (`:3930`).

**Domain of validity** `BoundariesComputing` (`:6166-...`): from `-1 <= B cos(U1-FI1) + C <= 1`, a
five-way case split on the sign of `B` and the position of `|C|` relative to `1 ± B`, yielding one
or two `U1` ranges expressed as `acos((1-C)/B)` / `acos(-(C+1)/B)` offsets from `FI1`, with the acos
arguments clamped to `[-1,1]` (`:6200-6208, 6221-6229, 6243-6261`). `return false` means no solution.

**Evaluation** `CylCylComputeParameters` (`:5323-5444`):
```
aTol0 = min(10*Epsilon(1.0)*B, aNulValue) ;  aTol = 1 - aTol0
anArg = B*cos(U1 - FI1) + C
if (anArg >=  aTol) anArg =  1.0 ;  else if (anArg <= -aTol) anArg = -1.0
U2 = FI2 + (branch ? -1 : +1) * acos(anArg)
V1, V2 from (4)
```
The `theDelta` output is the propagated `acos` error estimate
`aTol0 / sqrt(aDelta*(2 - aDelta))` with `aDelta = min(1-anArg, 1+anArg)` (`:5390-5393`).

**Sampling policy** `CyCyNoGeometric` (`:6573-...`): `aNbMaxPoints = 1000`, `aNbMinPoints = 200`
(`:6675-6676`); the "good intersection" heuristic (perpendicular axes within 10°, radius ratio > 3,
axis distance < Rmax/2) reduces those to 200/50 with a deflection-derived `du` (`:6613-6666`);
`aStepMin = max(Tol2D, PConfusion())`, `aStepMax` from the `U1` span (`:6691-6694`);
the infinite-curve bail is `V`-range `> 1e5·radius` (`:6603-6608`).

Both WLines are then produced by stepping `U1` over the valid ranges and **evaluating** — the point
positions are exact, only their *distribution* is heuristic. That is the property to preserve.

### 2.8 Routing: which pairs reach the analytic layer at all

`IntPatch_Intersection::Perform` (`IntPatch_Intersection.cxx:1090-1339`).

`bToCheck` is the **degeneracy** flag, not an eligibility flag:
- cones: `bToCheck = (|SemiAngle| < 0.02) || (|SemiAngle| > 1.55)` (`:1119-1120`), OR'd with the
  second cone's when both are cones (`:1122-1127`); with a quasi-planar exception for two
  `> 1.55` coaxial cones whose apices are coplanar (`:1128-1140`);
- tori: `bToCheck = MajorRadius > MinorRadius`, and for torus×torus the **second assignment
  overwrites the first** (`:1150-1158`) — torus1's validity is discarded.

The whole block that can set `bGeomGeom = 1` runs **only if `bToCheck`** (`:1166`). A normal cone
therefore never re-qualifies through it — and does not need to, because
`ts = 1` unconditionally for Plane/Cylinder/Sphere/**Cone** (`:1264-1291`); `ts = bGeomGeom` applies
**only** to `GeomAbs_Torus`.

Routing (`:1298-1339`): `ts1 == ts2 == 1 && isGeomInt → GeomGeomPerfom`;
`ts1 != ts2 → GeomParamPerfom`; `ts1 == ts2 == 0 → ParamParamPerfom`.

Consequences, stated plainly:
- **Every plane/cyl/cone/sphere pair reaches `IntPatch_ImpImpIntersection`.** OCCT never marches
  those. Neither should we.
- **A torus pair reaches it only when `bGeomGeom == 1`**, i.e. only in the coaxial (or
  plane-parallel / plane-through-centre-normal) configurations that `IntAna_QuadQuadGeo` can
  actually solve (`:1172-1227`). **In every other pose OCCT marches the torus.** That is OCCT's
  documented answer, and matching it is a legitimate port target.

`isGeomInt` comes from `IntTools_FaceFace::isTreatAnalityc`
(`src/ModelingAlgorithms/TKBO/IntTools/IntTools_FaceFace.cxx:249-324`): it inspects **only**
plane×cylinder-with-finite-`V` — everything else returns `true` early (`:271,307`) — and for that
one case rejects needle ellipses via `aMajorR < 100000·aMinorR` (`:318`); a non-`Ellipse` result
returns `inter.IsDone()` (`:322`).

`GeomGeomPerfom` fallback (`IntPatch_Intersection.cxx:1789-1797`):
`!interii.IsDone()` — and `IsDone() == (GetStatus() != IntStatus_Fail)`
(`IntPatch_ImpImpIntersection.lxx:21-24`) — triggers `ParamParamPerfom`. Afterwards
`aNbPointsInALine = 200` for ALine→WLine conversion (`:1807,1816`); `JoinWLines` is applied **only**
for cyl×cyl (`:1845-1848`); `ExtendTwoWLines` only when some ALine was converted (`:1850`).

### 2.9 Corrections this document carries (verified against source, this session)

1. **cyl×cyl coplanarity epsilon is `1e-7`**, not `1e-14`: the signature default is overridden at
   `IntAna_QuadQuadGeo.cxx:1054-1057`.
2. **Tangency is `Touch` + `Situation`, not `Undecided`** (`IntPatch_Line.cxx:36-49` vs `:51-62`).
3. **Transition dead bands are non-uniform**: `0.0 / 1e-9 / 1e-8 / 1e-7`, table in §2.2.
4. **plane×cone through the apex emits 2 or 4 GLines, not 1 or 2** (`:3570-3596`, `:3617-3675`);
   same for cone×cone (`:8774-8787`, `:8810-8827`).
5. **`RefineDir` needs an *exact* `±1` component** (`:2876-2888`); it is not a rotated-frame
   canonicaliser.
6. **`IntAna_IntQuadQuad` is reached by cyl×sphere (`:8266`), cyl×cone (`:8468`), cone×cone
   (`:9024`) and cone×sphere (`:9351`)** — not only by cyl×cone. Non-coaxial cylinder×sphere and
   cone×sphere produce exact ALines, never marched WLines.
7. **`IntAna_IntQuadQuad::NbPnt()` is always 0** on both paths.
8. **ALine chaining runs only on the cone path** (`IntAna_IntQuadQuad.cxx:1395`).
9. **`tgte` can be set without the case table** via `all1 && all2` (`:2865-2869`).
10. **Cone×cone in general position is not unhandled in OCCT** — it is `NoGeometricSolution` →
    ALine. Our claim that "cone×cone has no arm" is true of *our* code, not of OCCT.

---

## 3. DATA STRUCTURES AND C++ DECLARATIONS FOR OUR PORT

New files: `src/quadric.h` / `src/quadric.cpp` (descriptors + case table),
`src/quadric_aline.h` / `.cpp` (the ALine solver), `src/quadric_cylcyl.h` / `.cpp` (the
semi-analytic cyl×cyl solver). None of these may include `tolerance.h` (project rule); all
tolerances are explicit parameters or named constants in the `.cpp`.

```cpp
// ---------------------------------------------------------------- quadric.h
enum class QuadKind { Plane = 1, Cylinder = 2, Cone = 3, Sphere = 4, Torus = 5 };

// EXACT parameters, carried from the STEP reader / construction API, transformed
// covariantly. NEVER produced by fitting a NURBS (ARCHITECTURE_v2 law 3, G3 above).
struct Quadric {
    QuadKind kind;
    Point    loc;        // plane: a point on it; cyl/cone/torus: axis location; sphere: centre
    Vector   axis;       // unit; plane: normal; cone: axis (apex at `loc`); torus: axis
    Vector   xdir;       // unit, orthogonal to axis -- the SEAM direction. Load-bearing: it fixes
                         // the parameter origin of every emitted circle (see AdjustToSeam).
    double   r1 = 0;     // cyl/sphere radius; cone semi-angle (radians); torus major radius
    double   r2 = 0;     // torus minor radius
    bool     direct = true;   // gp_Ax3 handedness; plane normal is reversed when false
    Vector ydir() const;      // axis ^ xdir
    Point  apex() const;      // cone only == loc
    double ref_radius() const;// cone only: radius at loc's axial station
};

enum class QuadResult {           // 1:1 with IntAna_ResultType
    Point_, Line_, Circle_, PointAndCircle, Ellipse_, Parabola_, Hyperbola_,
    Empty, Same, NoGeometricSolution
};

enum class Trans { In, Out, Touch, Undecided };      // == IntSurf_TypeTrans
enum class Situation { Inside, Outside, Unknown };   // == IntSurf_Situation

// One emitted section element. Mirrors IntPatch_GLine/IntPatch_ALine's payload.
struct SectionElem {
    enum class Geo { Point, Line, Circle, Ellipse, Parabola, Hyperbola, Analytic, Walking } geo;
    // conic frame: centre + (zdir, xdir) exactly as DirToAx2 / the case table produced them
    Point  centre; Vector zdir, xdir;
    double major = 0, minor = 0;      // circle: major only; parabola: focal in `major`
    Vector dir;                       // Line only
    bool   tangential = false;        // tg
    Trans  t1 = Trans::Undecided, t2 = Trans::Undecided;
    Situation s1 = Situation::Unknown, s2 = Situation::Unknown;   // valid iff t1 == Touch
    std::vector<SectionVertex> vertices;
    int    aline_index = -1;          // index into AlineSet when geo == Analytic
    int    wline_index = -1;          // index into WlineSet when geo == Walking
};

struct SectionVertex {
    Point  p; double tol; double param;      // param on the element
    double u1, v1, u2, v2;                   // params on each operand
    bool   is_tangent = false;               // IntPatch_Point::SetValue 3rd arg
    bool   is_multiple = false;
};

// The complete verdict for one pair. `type` is G1's typed alphabet.
struct QuadQuadResult {
    QuadResult type = QuadResult::Empty;
    int  nb = 0;                       // nbint
    bool done = false;                 // IsDone()
    bool same_surface = false;         // Same / tgte
    bool opposite = false;             // oppo, valid iff same_surface
    bool has_common_gen = false;       // cone x cone
    Point pchar;                       // cone x cone characteristic point
    std::vector<SectionElem> elems;
    // raw case-table slots, kept so the accessors can reproduce OCCT's exact phase choices
    Point  pt[4]; Vector dir[4];
    double param[4]; double param_bis[2];
};

// --- shared primitives, §2.1
struct AxisRelation {                          // == AxeOperator
    AxisRelation(const Point& p1, const Vector& d1,
                 const Point& p2, const Vector& d2,
                 double eps_distance = 1e-14, double eps_axes_para = 1e-12);
    bool   same() const;        // parallel && distance < eps_distance
    bool   intersect() const;   // coplanar && !parallel  (implies distance < eps_distance)
    bool   parallel() const, coplanar() const, normal() const;
    double distance() const;
    Point  pt_intersect() const;
    void   common_perp(double& dist, double& param1, double& param2) const;
    /* fields exactly as §2.1.1 */
};
void   refine_dir(Vector& d);                                    // §2.1.2, verbatim
void   dir_to_ax2(const Point& p, const Vector& d, Vector& z, Vector& x);   // §2.1.3, verbatim
double estim_dist(const Quadric& cone1, const Quadric& cone2);   // §2.1.4

// --- the case table: 15 overloads, one per unordered kind pair, in OCCT's argument order
QuadQuadResult qq_plane_plane   (const Quadric& p1, const Quadric& p2, double tol_ang, double tol);
QuadQuadResult qq_plane_cylinder(const Quadric& p,  const Quadric& cy, double tol_ang, double tol,
                                 double h /*face V-extent, 0 if infinite*/);
QuadQuadResult qq_plane_sphere  (const Quadric& p,  const Quadric& sp);           // no tolerances
QuadQuadResult qq_plane_cone    (const Quadric& p,  const Quadric& co, double tol_ang, double tol);
QuadQuadResult qq_plane_torus   (const Quadric& p,  const Quadric& to, double tol);
QuadQuadResult qq_cyl_cyl       (const Quadric& c1, const Quadric& c2, double tol);
QuadQuadResult qq_cyl_cone      (const Quadric& cy, const Quadric& co, double tol);
QuadQuadResult qq_cyl_sphere    (const Quadric& cy, const Quadric& sp, double tol);
QuadQuadResult qq_cyl_torus     (const Quadric& cy, const Quadric& to, double tol);
QuadQuadResult qq_cone_cone     (const Quadric& c1, const Quadric& c2, double tol);
QuadQuadResult qq_sphere_cone   (const Quadric& sp, const Quadric& co, double tol);
QuadQuadResult qq_cone_torus    (const Quadric& co, const Quadric& to, double tol);
QuadQuadResult qq_sphere_sphere (const Quadric& s1, const Quadric& s2, double tol);
QuadQuadResult qq_sphere_torus  (const Quadric& sp, const Quadric& to, double tol);
QuadQuadResult qq_torus_torus   (const Quadric& t1, const Quadric& t2, double tol);

// --- the dispatcher: kind codes 1..5, iTT = k1*10 + k2, reverse when k1 > k2 (§2.0)
struct SectionResult {
    enum class Status { Ok, Empty, SameSurface, InfiniteSectionCurve, Fail } status;
    bool tangent_faces = false, opposite_faces = false;
    std::vector<SectionElem> elems;
    AlineSet alines; WlineSet wlines;
};
SectionResult intersect_quadrics(const Quadric& a, const Quadric& b,
                                 const UvBox& dom_a, const UvBox& dom_b,
                                 double tol_arc, double tol_tang);

// ------------------------------------------------------- quadric_aline.h  (§2.5)
struct ImplicitQuadric {                          // == IntAna_Quadric
    double cxx, cyy, czz, cxy, cxz, cyz, cx, cy, cz, ccte;
    std::vector<Point> special_points;            // cone apex / sphere poles; NEVER auto-cleared
    static ImplicitQuadric from(const Quadric& q);            // plane/cyl/cone/sphere ONLY
    ImplicitQuadric in_frame(const Point& o, const Vector& z, const Vector& x) const;
};

struct AlineCurve {                               // == IntAna_Curve
    double z0[6], z1[6], z2[6];       // {Cte, Sin, Cos, SinSin, CosCos, CosSin}
    bool   two_curves = false, take_z_positive = false;
    double tolerance = 0, dom_inf = 0, dom_sup = 0;
    bool   first_open = false, last_open = false;
    QuadKind host;                    // Cylinder or Cone
    Point  ax_loc; Vector ax_z, ax_x; double rcyl = 0, angle = 0;
    double first_param = 0, last_param = 0;
    void   uv_value(double theta, double& u, double& v,
                    double& A, double& B, double& C, double& sqrt_dis) const;  // §2.5, verbatim
    Point  value(double theta) const;
    bool   d1u(double theta, Point& p, Vector& t) const;
    void   find_parameter(const Point& p, std::vector<double>& params) const;
    void   set_domain(double f, double l);
};

struct AlineSet { std::vector<AlineCurve> curves; bool done = false; };
AlineSet aline_solve_cylinder(const Quadric& cyl, const ImplicitQuadric& q);   // §2.5 cyl path
AlineSet aline_solve_cone    (const Quadric& cone, const ImplicitQuadric& q);  // §2.5 cone path
// Splits an ALine at every cone-apex crossing, tolerance 10*tol.  == ExploreCurve
std::vector<AlineCurve> explore_curve(const Quadric& cone, const AlineCurve& c, double tol);

// ------------------------------------------------------ quadric_cylcyl.h  (§2.7)
struct CylCylCoeffs {                             // == ComputationMethods::stCoeffsValue
    double a1[3], a2[3], b1[3], b2[3], c1[3], c2[3], d[3];
    double k11,k21,l11,l21,m1, k12,k22,l12,l22,m2;
    double k1,l1,k2,l2, fiv1,psiv1,fiv2,psiv2;
    double B, C, fi1, fi2;
    CylCylCoeffs(const Quadric& cy1, const Quadric& cy2);   // throws on |det| < 1e-12
};
bool cylcyl_u1_ranges(const CylCylCoeffs& k, double period, Range out[2]);  // BoundariesComputing
bool cylcyl_eval(double u1, int branch /*0|1*/, const CylCylCoeffs& k,
                 double& u2, double* delta = nullptr);
bool cylcyl_eval_v(double u1, double u2, const CylCylCoeffs& k, double& v1, double& v2);

// ------------------------------------------------------------------ named constants
namespace qq {
    inline constexpr double EPS_DISTANCE               = 1.0e-14;
    inline constexpr double EPS_ANGLE_CONE             = 1.0e-12;   // Precision::Angular
    inline constexpr double EPS_MINI_CIRCLE_RADIUS     = 1.0e-9;    // 0.01*Confusion
    inline constexpr double EPS_CYLINDER_DELTA_RADIUS  = 1.0e-13;   // RELATIVE
    inline constexpr double EPS_CYLINDER_DELTA_DIST    = 1.0e-7;    // Confusion -- cyl x cyl only
    inline constexpr double EPS_AXES_PARA              = 1.0e-12;
    inline constexpr double TOL_APEX_CONF              = 1.0e-10;   // cone x cone
    inline constexpr double ELLIPSE_LIMIT              = 1.0e+9;
    inline constexpr double HYPERBOLA_LIMIT            = 2.0e+6;
    inline constexpr double ENTRY_TOLANG               = 1.0e-8;    // ImpImp entry
    inline constexpr double DEAD_BAND_NONE             = 0.0;
    inline constexpr double DEAD_BAND_PCONF            = 1.0e-9;
    inline constexpr double DEAD_BAND_STD              = 1.0e-8;
    inline constexpr double DEAD_BAND_CYSP2            = 1.0e-7;
    inline constexpr double ALINE_PROBE_MAX            = 5;         // kount > 5
    inline constexpr double ALINE_V_CLAMP              = 1.0e5;
    inline constexpr int    WLINE_MAX_POINTS           = 1000;
    inline constexpr int    WLINE_MIN_POINTS           = 200;
    inline constexpr double WLINE_INFINITE_FACTOR      = 1.0e5;     // V-range > f*R -> bail
}
```

**Tolerance contract for the port** (ARCHITECTURE_v2 law 2): the two tolerances that enter the case
table are `tol_ang` (dimensionless) and `tol` (a **model-space distance**). Everything else in the
table above is a fixed absolute constant of OCCT's, and each must be spelled out at its use site,
never folded into a global epsilon. **No tolerance in this subsystem may be derived from a UV
domain range.**

---

## 4. WHAT OUR CODE DOES TODAY, AND EXACTLY WHERE IT DIVERGES

All paths below are under `/home/petras/code/code_rust/session/session_cpp/src`.

### 4.1 The shape of what exists
`intersection.cpp:2265-4290` is a single-file analytic SSI layer:
- `RecogSurface` descriptor, `intersection.cpp:2373-2379` — five kinds, `p1/p2/r/r2`.
- Fitters: `fit_cylinder :2381`, `fit_cone :2442`, `fit_sphere :2516`, `fit_torus :2555`.
- `recognize_surface_impl :2675-2720` — plane, then sphere, then cylinder, then cone, then torus.
- Handlers: `ssi_plane_sphere :2743`, `ssi_plane_cylinder :2755`, `ssi_plane_cylinder_lines :2778`,
  `ssi_plane_cone :3040`, `ssi_plane_torus :3122`, `ssi_plane_plane :3143`,
  `ssi_cylinder_sphere :3919`, `ssi_cylinder_cone :3934`, `ssi_cone_sphere :3948`,
  `ssi_cylinder_cylinder :3975`, `ssi_cylinder_torus :4029`, `ssi_cone_torus :4044`,
  `ssi_sphere_torus :4069`, `ssi_torus_torus_spiric :4101`, `ssi_torus_torus :4176`.
- Dispatcher `analytic_ssi :4199-4290`; `handled == false` means "recognised but not handled" and
  the caller marches (`:4274`).

### 4.2 Divergence D1 — **descriptors are FITTED, not carried** (violates G3)
`intersection.cpp:2675-2720` calls `fit_sphere/fit_cylinder/fit_cone/fit_torus`, each of which
samples the NURBS on a 5×5 or 8×5 grid (`:2386-2395`, `:2448-2460`) and least-squares-solves.
`analytic_ssi:4201` then sets the fit tolerance to `rtol = max(tolerance, 1e-7) * 1e4` — i.e.
**1e-3 in model units**. Every downstream axis, apex and radius therefore carries ~1e-3 error before
any case-table decision is made, while the case table's own gates run at `1e-14` (§2.2).
That is a four-order-of-magnitude mismatch and by itself explains why the coaxiality gates can
never be satisfied for a rotated operand.
*Fix*: `Quadric` comes from the STEP reader / constructor and is transformed, never fitted.

### 4.3 Divergence D2 — **every curved pair is gated on a special pose** (violates G2, G1)
| our handler | our gate | line | OCCT's requirement |
|---|---|---|---|
| `ssi_cylinder_sphere` | `point_axis_dist(P, w, C) > 1e-6 → return false` | `:3923` | same requirement (`:1378`), but on **exact** parameters, and the `false` case goes to the **ALine solver** (`:8266`), not to a marcher |
| `ssi_cylinder_cone` | `!axes_coaxial(...) → return false` | `:3938` | same (`:1328`), and the `false` case goes to the ALine solver (`:8468`) |
| `ssi_cone_sphere` | `point_axis_dist(apex, a, C) > 1e-6 → false` | `:3952` | same (`:1926`), `false` → ALine solver (`:9351`) |
| `ssi_cylinder_cylinder` | non-parallel requires `|R1-R2|/Rmax <= 1e-6` **and** `lines_closest_point` within `1e-6` | `:4011-4013` | same (`:1224`), but `false` → the **semi-analytic** cyl×cyl solver (§2.7), not a marcher |
| `ssi_cylinder_torus` | `!axes_coaxial → false` | `:4034` | same (`:2298`) — **and OCCT also gives up**, marching. Parity. |
| `ssi_cone_torus` | `!axes_coaxial → false` | `:4049` | same (`:2377`). Parity. |
| `ssi_sphere_torus` | `point_axis_dist(C, w, S) > 1e-6 → false` | `:4074` | same (`:2514`). Parity. |
| `ssi_torus_torus_spiric` | parallel **and** `R1==R2` **and** `r1==r2` **and** zero axial offset | `:4107-4110` | OCCT requires coaxial + on-axis (`:2606`) — our gate is *narrower on radii*, *wider on offset* |
| `ssi_plane_torus` | `| |w·n| - 1 | > 1e-7 → false` | `:3126` | OCCT also accepts the **parallel** case (`:2193-2229`), which we do not implement at all |
| **cone × cone** | **no arm** — falls through to `:4270-4272 return res` | `:4270` | OCCT: 5 analytic cases (`:1478-1884`) **plus** the ALine solver for general position (`:9024`) |

So: of the 10 curved pairs, 6 are gated to a measure-zero pose set, 1 (cone×cone) is absent, and
3 (the torus pairs other than plane×torus) match OCCT's own give-up. **The measured 0/20 on
box×sphere, sphere×sphere and every cone pair is entirely explained by D1 + D2.**

### 4.4 Divergence D3 — **`handled == false` means "march"; OCCT has no such state**
`intersection.cpp:4274` — `if (!handled) return res;` with `res.status` left at its default, and the
caller (`:4757`) then falls to the marcher. OCCT's `NoGeometricSolution` is not a fallthrough: it is
a **dispatch to a second exact solver** (§2.5) for every plane/cyl/cone/sphere pair. Our code has no
equivalent of `IntAna_IntQuadQuad` at all. This is the single largest structural gap.

### 4.5 Divergence D4 — **one flat tolerance `kTol = 1e-6` for everything**
`intersection.cpp:3920, 3935, 3949, 3977, 4030, 4045, 4070, 4102` all declare
`const double kTol = 1e-6;` and use it for axis coincidence, radius comparison, tangency collapse,
discriminant sign and emptiness alike. OCCT uses six different named constants plus four different
transition dead bands (§2.2), and the *distinctions* are load-bearing: e.g. cyl×cyl equal-radius is
**relative** at `1e-13` while cyl×cyl coplanarity is **absolute** at `1e-7`. Collapsing them means
the type boundaries are in the wrong places by up to seven orders of magnitude, in both directions.

### 4.6 Divergence D5 — **no tangency type at all**
Nothing in `intersection.cpp` emits anything like `Touch` + `Situation`. `ssi_cylinder_sphere:3927`
handles the tangent case by emitting a plain circle; `ssi_cone_sphere:3962` emits a single circle
from a double root; `analytic_ssi:4242-4243` handles sphere/sphere tangency by **excluding it**
(`tan_tol = (r1+r2)*1e-9`, then `dist < r1+r2-tan_tol && dist > |r1-r2|+tan_tol`), so a tangency
produces *no output at all* rather than a typed contact. That violates G5 and forces downstream
keep/purge to re-derive the side by flux/quorum.

### 4.7 Divergence D6 — **no transverse orientation on any emitted curve**
Every handler pushes bare `NurbsCurve`s into `std::vector<NurbsCurve>& out`
(`ssi_cylinder_sphere:3930`, `ssi_cone_sphere:3971`, `ssi_cylinder_cylinder:4021-4022`, …). There is
no `(trans1, trans2)`, so nothing downstream can inherit a side from the section's orientation as
ARCHITECTURE_v2 §4 requires. Violates G6.

### 4.8 Divergence D7 — **no nappe purge**
`ssi_cone_sphere:3966-3967` filters on `sAx < kTol` (a *distance* along the axis from the apex),
which is close to but not the same as OCCT's `param >= paramapex` (`:9245`), and there is no
equivalent anywhere for cone×cone. Violates G7.

### 4.9 Divergence D8 — **conic phase is arbitrary and not OCCT's**
`ortho_basis(n)` (`intersection.cpp:2281-2294`) picks the reference axis from the
**smallest-magnitude** component, which is the same *rule* as `DirToAx2` — but it then builds
`u = a × n` and `v = n × u`, whereas `DirToAx2` returns `gp_Ax2(P, D, X)` with `X` one of
`(0,-z,y) / (-z,0,x) / (-y,x,0)` **unnormalised before `gp_Dir`**. The resulting phases differ, and
we never apply anything like `AdjustToSeam`, so circles emitted by different handlers for the same
geometry get different parameter origins. This is survivable *only* because nothing downstream keys
on the conic parameter — and that must remain true (see §2.6.4).

### 4.10 Divergence D9 — **finite-extent leakage into surface-level decisions**
`ssi_plane_cylinder_lines:2788-2800` and `ssi_plane_cone:3047` (`cone_axial_extent`) clip the
analytic answer to the *face's* sampled extent, and `ssi_cylinder_cylinder:3996-3998` intersects the
two cylinders' axial spans. OCCT does exactly one thing of this kind — the plane×cylinder `H`
relaxation (`:571-598`) — and it affects only the *parallelism* decision, never the emitted geometry
for the non-parallel case. Clipping at the surface level makes the section curve depend on the
operand's trim, which stage 6 must not do (the trim is applied in stage 9).

### 4.11 What is right today and must be kept
- `exact_circle :2297` and `exact_ellipse :2316` produce exact 9-CV rational NURBS conics — G9 is
  achievable with these unchanged.
- `ssi_plane_plane :3143` correctly refuses the parallel case rather than fabricating a line.
- `ssi_plane_cone :3040-3116` is the closest thing we have to a faithful port: it reproduces
  OCCT's `costa = cost*cosa - sint*sina` discriminant (`:3055`) and the four-way
  circle/parabola/hyperbola/ellipse split (`:3084-3089`). Its tolerances (`ang = 1e-6`,
  `distTol = 1e-6*max(1,H)`, `:3057-3058`) are the divergence, not its structure.
- `analytic_pcurve :3217` and `analytic_torus_pullback :3682` already do closed-form pull-back; they
  are the right consumers for the richer output this spec produces.

---

## 5. ACCEPTANCE TESTS

Every test is stated with concrete operands and an **oracle-free** invariant. `T` denotes a random
rigid motion (random axis, random angle, random translation); `scale` denotes the model's
characteristic length. Residual budgets are absolute in model units unless stated.

### T-group A — covariance and totality (G1, G2)
**A1 (type stability).** For each of the 15 kind pairs, fix one intersecting configuration; sample
200 random `T`; assert `intersect_quadrics(T·a, T·b).status` and `.type` are identical for all 200.
*Oracle-free*: the answer is compared only against itself at other poses.
**A2 (geometry covariance).** Same sampling; for each emitted `SectionElem`, assert
`elem(T·a, T·b).centre` equals `T · elem(a,b).centre` to `1e-12·scale`, radii equal to `1e-12·scale`,
and `zdir` equals `R·zdir` to `1e-12`.
**A3 (no unhandled).** Same sampling extended to non-intersecting and tangent configurations; assert
`status != Fail` for every plane/cyl/cone/sphere pair, and that torus pairs return either a typed
analytic verdict or `Status::Ok` with `Walking` elements — never a silent empty result with
`type == NoGeometricSolution`.

### T-group B — closed-form residual (G9)
**B1 (implicit residual).** For every emitted conic, sample 64 parameters uniformly; assert
`|f_a(P)| <= 1e-13·scale²` and `|f_b(P)| <= 1e-13·scale²` where `f` is the implicit quadric form of
§2.5. Applies to Circle/Ellipse/Line/Parabola/Hyperbola.
**B2 (ALine residual).** For every emitted `AlineCurve`, sample 200 `θ` in `[dom_inf, last_param]`;
assert both implicit residuals `<= 1e-11·scale²`. Looser than B1 because the ALine is evaluated
through a square root.
**B3 (WLine residual).** cyl×cyl general position: assert every WLine point satisfies both cylinder
equations to `1e-12·scale²`. This is the property that distinguishes OCCT's semi-analytic evaluator
from a marcher and it must hold exactly.

### T-group C — the cases with known closed-form answers
Each of these has an answer derivable by hand; assert the *derived* value, not a reference file.

**C1 — sphere × sphere, arbitrary pose.** `S1 = (centre 0, R=5)`, `S2 = (centre (6,0,0), R=4)`.
Truth: one circle, `Alpha = (25-16+36)/12 = 3.75`, centre `(3.75,0,0)`, radius
`sqrt(25 - 14.0625) = sqrt(10.9375) = 3.30719...`. Apply 200 random `T`. Assert 1 circle, radius to
`1e-13`, centre to `1e-13`, and that both spheres' implicit residuals vanish (B1).
*Today: 0/20.*

**C2 — sphere × sphere, external tangency.** `R1=5, R2=4, |O1O2| = 9`. Truth: `type == Point_`,
`nb == 1`, point at `O1 + 5·u`. Assert also `SectionVertex::is_tangent == true`. Then perturb
`|O1O2|` to `9 + 2·Tol` and `9 - 2·Tol` and assert the type flips to `Empty` and `Circle_`
respectively — the G4 single-flip test. (Note OCCT's threshold here is `t = Rmax - d - Rmin ∈
[0, Tol]`, `:2079`, plus the `Beta <= 1e-9` collapse at `:2119`.)

**C3 — plane × sphere, arbitrary pose.** `S = (0, R=3)`, plane at signed distance `d = 1.2`.
Truth: circle radius `sqrt(9-1.44) = 2.749545...`, centre at the foot. 200 random `T`.
*Today: passes only because our plane×sphere is already pose-free — use it as the control.*

**C4 — cylinder × sphere, sphere centre ON the axis.** `Cyl (axis z, R=2)`, `Sph (centre (0,0,0),
R=3)`. Truth: 2 circles of radius 2 at `z = ±sqrt(5)`. 200 random `T`.
**C4b — cylinder × sphere, centre OFF the axis by 0.5.** Truth: no closed-form conic; assert
`type == NoGeometricSolution` **and** that an `AlineSet` with `curves.size() >= 1` is produced, and
apply B2. *Today: `ssi_cylinder_sphere:3923` returns `false` and the pair is marched.*

**C5 — cylinder × cone, coaxial.** `Cyl (axis z, R=2)`, `Cone (apex 0, axis z, β=30°)`.
Truth: 2 circles of radius 2 at `z = ±2/tan30° = ±3.4641016...`. 200 random `T`.
**C5b — cylinder × cone, skew (the 251%-wrong case).** `Cyl (axis z through (3,0,0), R=1)`,
`Cone (apex 0, axis x, β=25°)`. Truth: `type == NoGeometricSolution`, `AlineSet` non-empty,
B2 holds, and `explore_curve` splits at the apex whenever the curve passes within `10·tol` of it.
Additional oracle-free invariant: the ALine's endpoints must lie on **both** surfaces to `1e-11`.
*Today: hangs or returns a 251%-wrong volume.*

**C6 — cone × sphere, centre on axis.** `Cone (apex 0, axis z, β=30°)`, `Sph (centre (0,0,4), R=1)`.
Truth: quadratic `(1+t²)x² + 2t²·4·x + (16t² - 1) = 0` with `t = tan30°`; two circles at
`s = 4 + x_k`, radii `t·s`. Assert both roots' radii to `1e-13`. 200 random `T`.
**C6b — cone × sphere, centre off-axis.** Assert `NoGeometricSolution` + non-empty `AlineSet` + B2.
**C6c — nappe purge (G7).** `Cone (apex 0, axis +z, β=30°)`, `Sph (centre (0,0,-4), R=1)`.
Truth: the algebraic solution lies entirely on the negative nappe; assert **0** emitted elements
and 0 points. This is an oracle-free test: the sphere lies strictly in `z < 0` and the cone occupies
`z >= 0`, so any emitted geometry is provably wrong.

**C7 — cone × cone, coaxial, unequal angles.** `C1 (apex 0, axis +z, β1=20°)`,
`C2 (apex (0,0,6), axis +z, β2=35°)`. Truth (`:1495-1501`): with `d = 6`,
`x_a = d·tg2/(tg1+tg2)`, `x_b = d·tg2/(tg2-tg1)`; two circles at `z = x_a, x_b` with radii
`|x·tg1|`. Assert both to `1e-13`. 200 random `T`.
**C7b — cone × cone, common apex, transversal.** `C1 (apex 0, axis +z, β=30°)`,
`C2 (apex 0, axis (0,sin40°,cos40°), β=30°)`. Truth: 2 lines through the apex; assert `type ==
Line_`, `nb == 2`, both directions satisfy both cone equations to `1e-13`, and 4 GLines are emitted
(2 lines × 2 rays, §2.3.10).
**C7c — cone × cone, common apex, tangency.** Rotate `C2` until the 2D test `aRD2 == aR2` (§2.3.10
case 3): the two cones touch along one generatrix. Assert `nb == 1`, and the two emitted GLines
carry `Touch` with `(Outside, Outside)` when `N1·N2 < 0` (`:8749-8753`).
**C7d — cone × cone, general position (the missing arm).** `C1 (apex 0, axis +z, β=25°)`,
`C2 (apex (4,1,2), axis (1,1,0)/√2, β=35°)` — skew axes, distinct apices, no common generatrix.
Truth: `type == NoGeometricSolution`, `AlineSet` non-empty, B2 holds. *Today: no dispatcher arm.*

**C8 — cylinder × cylinder, parallel, 2 lines.** `C1 (axis z at origin, R=3)`,
`C2 (axis z at (4,0,0), R=3)`. Truth: `aCos = 0.5·(9-9+16)/(3·4) = 2/3`, `aSin = sqrt(5)/3`; two
lines at `(3·(2/3), ±3·sqrt(5)/3, ·) = (2, ±sqrt(5), ·)`. Assert both to `1e-13`. 200 random `T`.
**C8b — cylinder × cylinder, perpendicular, equal radii (Steinmetz).** `R = 2`, axes `z` and `x`
meeting at the origin. Truth: two ellipses, semi-axes `(2√2, 2)`, major directions `(z±x)/√2`.
Assert `type == Ellipse_`, `nb == 2`, semi-axes to `1e-13`. **Also assert** the first ellipse's
GLine carries the **swapped** transitions relative to the sampled sign (`:5067-5069`), the vertices
at `0` and `2π`, and the multiple points at `0.5π, 1.5π`.
**C8c — cylinder × cylinder, skew, unequal radii.** `C1 (axis z, R=3)`,
`C2 (axis through (2,0,1) along (0,1,1)/√2, R=1)`. Truth: `type == NoGeometricSolution` from the
case table, then **two** WLines from §2.7 with B3 holding, and `status != Fail`. Also assert
`cylcyl_u1_ranges` returns non-empty ranges whose union is the exact set
`{U1 : |B cos(U1-FI1) + C| <= 1}` — checkable by direct sampling, no oracle.
**C8d — cylinder × cylinder, coaxial equal radius.** Assert `type == Same`.
**C8e — cylinder × cylinder, coplanarity epsilon (the audited constant).** Two equal-radius
cylinders (`R=2`) with perpendicular axes whose closest approach is `1e-9`. Truth: OCCT's
`Intersect()` succeeds (`1e-9 < 1e-7`) → **two Steinmetz ellipses**. Assert `type == Ellipse_`.
Then set the miss distance to `1e-6` and assert `type == NoGeometricSolution`. This test fails
outright if the port uses `1e-14`.

**C9 — plane × cone, all five conic branches.** Cone `(apex 0, axis +z, β=30°)`; plane normals
chosen so that `cost` takes the values `1` (circle), `cos30°` exactly (parabola, `costa = 0`),
`0.4` (hyperbola, `cost < sina`… note `sina = 0.5`), `0.9` (ellipse), and `0` (axis-containing
hyperbola). For each, assert the emitted conic type, and B1. Then repeat with the plane translated
to pass through the apex, asserting the `Line_` (1), `Line_` (2), `Point_` triple of §2.3.4 branch A,
and that the `NbSol==1` case emits **2** GLines with `Touch`.
**C9b — extreme-conic give-up.** Choose a plane nearly parallel to a generatrix so the hyperbola's
`param1` exceeds `2e6`; assert the port reports the same give-up (`done == false` →
`Status::Fail`) rather than emitting a garbage conic. This is a *parity* test with a documented
OCCT fallback.

**C10 — plane × cylinder, parallel ladder.** `Cyl (axis z, R=2)`; planes containing the `z`
direction at signed distances `0`, `1`, `2`, `3`. Truth: 2 lines at `x=±2`… (`d=0`); 2 lines at
`±sqrt(3)` (`d=1`); 1 tangent line (`d=2`); `Empty` (`d=3`). Assert types and positions to `1e-13`,
and that the `d=2` case emits `Touch` with a situation.
**C10b — plane × cylinder, generic.** Plane normal at 60° to the axis: ellipse with
`major = R/cos60° = 4`, `minor = 2`. 200 random `T`.

**C11 — plane × torus, both accepted poses.** Torus `(centre 0, axis z, Rmaj=5, Rmin=2)`.
(a) plane `z = 1` (parallel to axis normal? — here the plane axis is **parallel** to the torus axis):
truth 2 circles of radii `5 ± sqrt(3)`; (b) plane `y = 0` (plane axis **normal** to the torus axis,
plane through the centre): truth 2 circles of radius 2 centred at `(±5, 0, 0)` with the **plane**
normal as axis. Assert both. (c) plane at 30° to the axis: assert
`type == NoGeometricSolution` and that the pair is routed to the marcher — **parity with OCCT**.
*Today: (a) is missing entirely (`ssi_plane_torus:3126` refuses anything but perpendicular).*

**C12 — the four coaxial torus pairs.** cyl×torus, cone×torus, sphere×torus, torus×torus, each in
its coaxial configuration with hand-computed circle radii from §2.3.12–15. 200 random `T` each.
**C12b — non-coaxial torus pairs terminate.** For each of the five torus pairs, 20 random poses;
assert each call returns within a fixed wall-clock budget with a typed verdict.
*Today: box×torus and torus×torus complete **zero** poses in 25 minutes — this is the G8 test.*

### T-group D — degeneracy single-flip (G4)
For each degenerate predicate in §2.2's table, drive the configuration from `threshold·(1-δ)` to
`threshold·(1+δ)` with `δ = 10` and assert the returned `type` changes **exactly once** and at the
documented value. Predicates to cover, minimum: sphere/sphere tangency (`Tol`, then `1e-9`);
cyl/cyl equal-radius (**relative** `1e-13`); cyl/cyl coplanarity (`1e-7`); cone/cone equal-angle
(`aTolAng` from `EstimDist`); cone/cone coincident apices (`Tol²` on the **squared** distance);
sphere/cone mini-circle (`1e-9`); plane/torus parallel snap (`1e-13`); torus axis coincidence
(`1e-14`).

### T-group E — tangency side (G5)
**E1.** Externally tangent spheres → `Touch`, `(Outside, Outside)`.
**E2.** Internally tangent spheres (`Rmax = d + Rmin`) → `Touch`, one `Inside`.
**E3.** Plane tangent to a cylinder → `Touch`; assert `situcyl` flips when the plane normal is
reversed, per `:3221-3244`.
**E4.** Two cylinders tangent along a line, same-side curvature centres, `R1 < R2` → the
smaller-radius cylinder decides, per `:4942-4961`.
**E5.** Two cones tangent along a generatrix with `N1·N2 >= 0` → the cone whose axis is farther from
the probe point is `Outside`, per `:8765-8772`.

### T-group F — integration with stage 6
**F1 (partition residual).** Re-run the 224-cell primitive-pair battery. Gate: box×sphere,
sphere×sphere, box×cone, cone×cone, cyl×cone, sphere×cone all reach `>= 18/20` genuinely correct
(partition residual `< 1e-12`), and box×torus / torus×torus complete all 20 poses with a typed
verdict.
**F2 (no regression).** box×box stays at `>= 15/20`; base chairs cut/common/fuse stay exact
(35/46.8114, 25/33.4951, 50/127.0950); matrix 45/45; edge 54/54.
**F3 (independence from UV domain).** Re-run F1 with every operand's UV domain padded 4× (the STEP
round-trip case). Gate: byte-identical section geometry. This is the direct test that no tolerance
in this subsystem is domain-relative.

---

## 6. IMPLEMENTATION ORDER

Each increment is independently landable, independently measured, and gated. `SESSION_QQ` selects
the new path; the existing `analytic_ssi` remains the default until increment 7.

**I0 — `Quadric` descriptors carried, not fitted.** (fixes D1, enables everything else)
Add `Quadric` and a `Quadric` field on the surface/face representation, populated by the STEP reader
and the primitive constructors, transformed covariantly. `recognize_surface` becomes a *fallback*
used only when the carried descriptor is absent.
*Gate*: build a rotated cylinder, assert its carried axis equals `R·a` to `1e-15` (not `1e-3`);
F2 unchanged (the new field is unused by the old path).
*Ships alone. This is the largest single lever and touches no algorithm.*

**I1 — shared primitives.** `AxisRelation`, `refine_dir`, `dir_to_ax2`, `estim_dist`, the `qq::`
constant block. Unit tests only.
*Gate*: `AxisRelation` reproduces the pivoted 2×2 solve on 1000 random axis pairs (compare against a
least-squares closest-point computation to `1e-12`); `dir_to_ax2` is deterministic and orthonormal
for 10 000 random directions; `refine_dir` is a no-op for any direction with no exact `±1` component.

**I2 — the four pose-free pairs, exactly.** plane×plane, plane×sphere, plane×cylinder,
sphere×sphere. These have no coaxiality gate in OCCT, so they are pure transcription and they give
the first measurable win (sphere×sphere is 0/20 today).
*Gate*: C1, C2, C3, C10, C10b, plus A1/A2/B1 restricted to these four pairs. F2 unchanged.

**I3 — plane×cone and the conic emitter.** Full §2.3.4 including the apex branch, the frame flip at
`:866-870`, and the extreme-conic give-up. Introduce `SectionElem` with `(t1, t2)` and the
`Touch`/`Situation` encoding, and the apex-ray splitting (2 or 4 GLines).
*Gate*: C9, C9b, E3-analogue for the cone tangency; A1/A2/B1 extended. Wire the transitions through
to the (unchanged) downstream classifier behind `SESSION_QQ` and assert F2 unchanged with the gate
off.

**I4 — the coaxial curved pairs, exactly.** cyl×cone, cyl×sphere, cone×sphere, cone×cone (all five
cases of §2.3.10), cyl×cyl (both ladders). Everything still returns `NoGeometricSolution` in general
position — but now it *returns it as a typed verdict*, not as "unhandled".
*Gate*: C4, C5, C6, C6c, C7, C7b, C7c, C8, C8b, C8d, C8e; T-group D; T-group E. This is where the
audited `1e-7` constant is first proved (C8e).

**I5 — the ALine solver.** `ImplicitQuadric`, `AlineCurve`, `aline_solve_cylinder`,
`aline_solve_cone`, `explore_curve`, and the tangent-probe ladder. Wire it into the four
`NoGeometricSolution` arms (cyl×sphere, cyl×cone, cone×cone, cone×sphere).
*Sub-order inside I5*: (a) `ImplicitQuadric` + `in_frame` (unit-tested by evaluating the form at
100 surface points and asserting `< 1e-13·scale²`); (b) the trig root finder including the θ=π
special case and the ×1e-4 rescaling loop (unit-tested against direct sampling of the trig
polynomial); (c) `AlineCurve::uv_value` / `value` / `d1u` (tested by B2 on a hand-built curve);
(d) the cylinder path; (e) the cone path; (f) `explore_curve`.
*Gate*: C4b, C5b, C6b, C7d; B2; A3 now passes for all 16 plane/cyl/cone/sphere pairs.
**This is the increment that closes the "everything else marches" hole.**

**I6 — the semi-analytic cyl×cyl solver.** `CylCylCoeffs`, `cylcyl_u1_ranges`, `cylcyl_eval`, the
sampling policy and the infinite-curve bail.
*Gate*: C8c; B3; A3 now passes for cyl×cyl in general position.

**I7 — torus pairs at parity with OCCT.** The five coaxial solvers of §2.3.12–15 plus the missing
plane×torus **parallel** branch, and — critically — the routing rule of §2.8 that sends every
non-coaxial torus pair to the marcher **with a bounded budget and a typed verdict** rather than
letting it hang.
*Gate*: C11, C12, C12b (the G8 termination test), F1, F2, F3. `SESSION_QQ` becomes the default; the
old `analytic_ssi` handlers are deleted.

**Explicitly out of scope for this subsystem** (owned by neighbouring port specs): the ALine→WLine
conversion (`IntPatch_ALineToWLine`, 200 points), `JoinWLines`/`ExtendTwoWLines`, the restriction
solver `solrst` that feeds §2.6.3, `ComputeVertexParameters`, and the projection of section curves
into UV. This spec's output contract stops at `SectionResult`.

---

### Appendix — one-line summary of OCCT's pose requirement per pair

| pair | closed form in general pose? | pose gate | general-position answer |
|---|---|---|---|
| plane × plane | **yes** | none | Line / Same / Empty |
| plane × cylinder | **yes** | none | Ellipse / Circle / 1–2 Lines / Empty |
| plane × sphere | **yes** | none | Circle / Point / Empty |
| plane × cone | **yes** | none | Circle/Ellipse/Parabola/Hyperbola/Lines/Point (+ 2e6/1e9 give-up) |
| plane × torus | **no** | parallel **or** normal-through-centre | marcher |
| cylinder × cylinder | partial | parallel; or equal-radius **and** axes meet within `1e-7` | **semi-analytic WLine** (§2.7) |
| cylinder × cone | no | coaxial within `1e-14` | **exact ALine** (§2.5) |
| cylinder × sphere | no | centre exactly on axis | **exact ALine** |
| cylinder × torus | no | coaxial | marcher |
| cone × cone | partial | coaxial; or parallel+equal-angle; or coincident apices; or common generatrix | **exact ALine** |
| cone × sphere | no | centre exactly on axis | **exact ALine** |
| cone × torus | no | coaxial | marcher |
| sphere × sphere | **yes** | none | Circle / Point / Same / Empty |
| sphere × torus | no | centre on axis | marcher |
| torus × torus | no | coaxial | marcher |

Five pairs are unconditional. Six more are exact via the ALine solver or the cyl×cyl semi-analytic
solver in *every* pose. Only the four torus-with-curved pairs and plane×torus fall to a marcher, and
that is OCCT's own documented answer.
