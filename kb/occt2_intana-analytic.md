# intana-analytic

OCCT exact analytic quadric-quadric intersection, extracted from source at
`C:/brg/compas_occt/external/occt/src/occt/src/ModelingData/TKGeomBase/IntAna/`
(IntAna_QuadQuadGeo, IntAna_IntQuadQuad, IntAna_Quadric, IntAna_Curve, IntAna_IntConicQuad)
plus its consumers `ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.cxx`,
`IntPatch_Intersection.cxx` and `ModelingAlgorithms/TKBO/IntTools/IntTools_FaceFace.cxx`.

This is the subsystem that decides WHEN a surface pair gets an exact conic/line/point
answer instead of a marched curve, and HOW every degenerate configuration (tangent line,
double line, point contact, Same) is classified. The marching side is already specified in
`kb/occt_ssi-walking.md` (do not re-read that here; this spec covers everything upstream of
it on the analytic branch). Tolerance semantics of `Precision::*` are in
`kb/occt_tolerance-model.md`.

---

## STAGE PIPELINE

Ordered as executed for a natural-quadric x natural-quadric face pair inside a boolean.

### Stage 0 — Analytic-eligibility routing (analytic vs marching decision #1)
- **File/fn**: `IntPatch_Intersection.cxx` — `IntPatch_Intersection::Perform` (~L1090–1338).
- Each surface gets `ts = 1` (Geom) or `0` (Param):
  - Plane / Cylinder / Sphere / Cone → always `ts = 1`.
  - Torus → `ts = bGeomGeom`, computed ONLY for the coaxial-family cases (see gates below); any
    general-position torus is demoted to the parametric walker.
  - Everything else → `ts = 0`.
- **Cone degeneracy gate**: any cone with `|SemiAngle| < 0.02` rad or `> 1.55` rad sets
  `TreatAsBiParametric = true` (~L1116–1143) → surfaces retyped to `GeomAbs_BezierSurface`
  → Prm-Prm marching (plane x cone keeps Imp-Prm). Exception kept analytic: BOTH cones
  quasi-planar (`>1.55`), axes parallel (`Precision::Angular()`), apexes coplanar within
  `Precision::Confusion()` (~L1128–1140).
- **Torus gates** (`bGeomGeom = 1` required to stay analytic, ~L1150–1233):
  - vs Plane: torus axis parallel to plane normal, OR normal to it AND plane passes through
    torus center (`Distance < Precision::Confusion()`).
  - vs Sphere: sphere center on torus axis (`aL1.Distance(loc) < Confusion`).
  - vs Cylinder/Cone/Torus: axes parallel (`Precision::Angular()`) AND coaxial
    (`aL1.Distance(axisLoc) <= Confusion`).
  - Additionally requires `MajorRadius > MinorRadius` (self-intersecting torus never analytic).
- **Plane x cone special**: analytic unless `SemiAngle < 0.02` AND axis nearly in the plane
  (`|axis . planeNormal| < 0.015`) (~L1177–1184).
- Infinite domains force `TreatAsBiParametric = false` (~L1237).
- Routes: Geom-Geom → `GeomGeomPerfom` (~L1778); Geom-Param → `GeomParamPerfom`; else
  `ParamParamPerfom` (marching, see occt_ssi-walking.md).

### Stage 1 — GeomGeomPerfom wrapper (analytic vs marching decision #2)
- **File/fn**: `IntPatch_Intersection.cxx` — `GeomGeomPerfom` (~L1778–1900).
- Runs `IntPatch_ImpImpIntersection`. If `!IsDone()` → **fallback to ParamParamPerfom**
  (marching). This is the master rule: the analytic machinery may simply give up
  (`done=false`) and OCCT silently re-runs the pair through the walker.
- ALines (trig-parametrized exact curves from IntQuadQuad) are converted to WLines via
  `IntPatch_ALineToWLine AToW(theS1, theS2, aNbPointsInALine=200)` — the exact curve is
  SAMPLED into a 200-point walking line for downstream topology (approx recovers a curve fit).
- Cyl x cyl only: `IntPatch_WLineTool::JoinWLines`; if any WLine came from an ALine:
  `ExtendTwoWLines` with a critical-point list = cone apexes + sphere poles of the operands.

### Stage 2 — ImpImp dispatch table
- **File/fn**: `IntPatch_ImpImpIntersection.cxx` — `Perform` (~L2517–2780), key `iTT = iT1*10+iT2`
  with 1=Plane 2=Cylinder 3=Cone 4=Sphere 5=Torus:
  - 11 `IntPP`, 12/21 `IntPCy`, 13/31 `IntPCo`, 14/41 `IntPSp`, 15/51 `IntPTo`,
    22 `IntCyCy`, 23/32 `IntCyCo`, 24/42 `IntCySp`, 25/52 `IntCyTo`,
    33 `IntCoCo`, 34/43 `IntCoSp`, 35/53 `IntCoTo`, 44 `IntSpSp`, 45/54 `IntSpTo`, 55 `IntToTo`.
- Local `Tolang = 1.e-8` for all plane cases. Plane x cylinder passes the cylinder's finite
  V-extent `H = VMax - VMin` (0 if infinite) down to QuadQuadGeo — the near-parallel
  ellipse-degeneracy guard needs the face height.
- Cyl x cyl computes `a2DTol = min(1.0e-4, min(S1->UResolution(TolTang), S2->UResolution(TolTang)))`
  and calls `IntCyCy`; if the result is a WLine, post-processing (restriction arcs) is SKIPPED.

### Stage 3 — IntAna_QuadQuadGeo: the geometric case table
- **File/fn**: `IntAna_QuadQuadGeo.cxx`, one `Perform` per pair. Result = `IntAna_ResultType`
  (`Point, Line, Circle, PointAndCircle, Ellipse, Parabola, Hyperbola, Empty, Same,
  NoGeometricSolution`) + up to 4 (pt, dir, param) solution slots.
  `NoGeometricSolution` = "a curve exists but is not a conic → go to IntQuadQuad / walker".

Complete case table (all formulas in source, cited by line):

**Plane x Plane** (`Perform(P1,P2,TolAng,Tol)` L389):
`|N1 x N2| <= TolAng` → `Same` iff both cross-plane distances `<= Tol` else `Empty`.
Else `Line` (1): dir = N1 x N2, origin = midpoint of the two per-plane projections.
Origin refinement when `|N1 x N2| < 2.e-6` rad and residual plane distance `> 1.e-12`:
two successive `IntAna_IntConicQuad` line-plane projections re-seat the origin (L458–509).
Denominator guard `aEps = 1.e-16` on `1 - (N1.N2)^2`.

**Plane x Cylinder** (L543, takes `H`):
- Near-parallel relaxation (L571–598): if axis/plane angle is within `PI/4` of parallel and
  `dangle > Tolang` but `sin(dangle)*H < 2*Tol` (deviation over the actual face height under
  tolerance) → force-parallel with `tolang = 2*sinda`, `toltang = max(Tol, sinda*H*1.01)`
  (`newparams` mode; line directions then re-derived from a 100-unit axis translation so the
  "lines" follow the tilted axis).
- `IntAna_IntConicQuad inter(axis, P, tolang, toltang, H)`; if `IsParallel()`:
  `||dist| - radius| < Tol` → 1 tangent `Line` (dir = axis);
  `|dist| < radius` → 2 `Line`s at `omega ± sqrt(r^2 - d^2) * (axisDir x planeNormal)`;
  else `Empty`.
- Else 1 intersection point of axis = conic center: `sint = |normp x axisDir|`;
  `sint < Tol/radius` → `Circle` (radius r, dir1 = axis, dir2 = XDirection);
  else `Ellipse`: `cost = |axisDir . normp|`, center pt1, dir1 = plane normal, dir2 = major
  axis `axex = (normp x axis) x normp`, `param1 = radius/cost` (major), `param1bis = radius` (minor).

**Plane x Sphere** (L980):
`dist = signed distance(center, plane)`. `||dist| - r| < Epsilon(radius)` → `Point`
(projection of center). `|dist| < r` → `Circle` radius `sqrt(r^2 - d^2)`, center = projection,
dir1 = plane normal (reversed if `!P.Direct()`). Else `Empty`. (Machine-epsilon tangency,
NOT Tol — the only pair using `Epsilon(r)`.)

**Plane x Cone** (L752): `dist` = signed apex-plane distance; `sint,cost` from axis vs normal;
`sina,cosa` from SemiAngle; `costa = cost*cosa - sint*sina` (= cos(axisAngle + semiangle) —
zero iff plane parallel to a generatrix).
- Apex ON plane (`|dist| < Tol`):
  `|costa| < Tolang` → 1 `Line` (generatrix through apex, dir = projection of a +axis point);
  `cost < sina` (plane pierces interior) → 2 `Line`s: `dir = axex ± dh*axey`,
  `dh = sqrt(sina^2 - cost^2)/cosa`;
  else → `Point` (apex).
- Apex OFF plane:
  `cost < Tolang` (plane contains axis direction) → `Hyperbola` (2 branches),
  center `apex - dist*normp`, `param1 = |dist/tan(angl)|` (major), `param1bis = |dist|` (minor);
  else compute conic center = axis-plane intersection via `IntAna_IntConicQuad`; the local x
  axis flips when `ParamOnConic(1) + RefRadius/tan(angl) < 0` (which nappe);
  `|costa| < Tolang` → `Parabola`: `deltacenter = distance/(2*cosa)`, vertex
  `center - deltacenter*axex`, focal `param1 = deltacenter*sina^2`;
  `sint < Tolang` → `Circle`, radius `apexDist*|tan(angl)|`;
  `cost < sina` → `Hyperbola`: `deltacenter = sint*sina^2*distance/(sina^2-cost^2)`,
  `param = cost*sina*cosa*distance/(sina^2-cost^2)`,
  `parambis = cost*sina*distance/sqrt(sina^2-cost^2)`;
  else → `Ellipse`: `param1 = cost*sina*cosa*distance/(cost^2-sina^2)` (major),
  `deltacenter = sint*sina^2*distance/(cost^2-sina^2)` (center offset along +axex),
  `param1bis = cost*sina*distance/sqrt(cost^2-sina^2)` (minor).
- Extreme-conic rejection (L925–950): `Ellipse` with `param > 1.0E+9` or `Hyperbola` with
  `param > 2.0E+6` → `done = false` → whole pair falls to the biparametric walker.

**Cylinder x Cylinder** (L1050): `AxeOperator A1A2(axis1, axis2,
myEPSILON_CYLINDER_DELTA_DISTANCE, myEPSILON_AXES_PARA)`.
- Parallel axes: `DistA1A2 <= Tol` → `Same` if `|R1-R2| <= Tol` else `Empty`.
  Else: `Dist > R1+R2+Tol` → `Empty`;
  `(R1+R2 - Dist) <= RealSmall()` → 1 tangent `Line` at ratio `R1/(R1+R2)` along P1P2;
  `Dist > |R1-R2|` → 2 `Line`s = circle-circle intersection in the base plane
  (`aCos = (R1^2 - R2^2 + d^2)/(2 R1 d)`), with sub-tolerance tangency collapse:
  `4*R1^2*(1-aCos^2) < Tol^2` → 1 `Line` (chord AB shorter than Tol);
  `Dist > |R1-R2| - Tol` → 1 internal-tangent `Line` (sign flips if R1 < R2);
  else `Empty`.
- Skew/crossing axes: **equal radius** (`|R1-R2|/Rmax <= myEPSILON_CYLINDER_DELTA_RADIUS`)
  AND axes truly intersect → 2 `Ellipse`s through the axis intersection point,
  dirs `D1+D2` and `D1-D2`, `param = R/|sin(halfAngle)|` per branch, minor `= R` both
  (Steinmetz); degenerate `sin = 0` → `Same`.
  Else external point-tangency `| Dist - R1 - R2 | < Tol` → `Point` on the common
  perpendicular at distance R1 from axis1.
  Else → `NoGeometricSolution` (→ `CyCyNoGeometric` walking-on-formula, Stage 5).

**Cylinder x Cone** (L1324): coaxial (`A1A2.Same()`: parallel + inter-axis distance
`< 1e-14`) → 2 `Circle`s radius `RCyl` at `apex ± (RCyl/tan(SemiAngle)) * axisDir`.
ANY other configuration → `NoGeometricSolution` (Tol unused). All non-coaxial cyl-cone
conics come from Stage 4.

**Cylinder x Sphere** (L1373): sphere center EXACTLY on cylinder axis
(`Pt.Distance(A1A2.PtIntersect()) == 0.0` — exact, or `A1A2.Same()`):
`RSph < RCyl` → `Empty`; else `Circle`(s) radius `RCyl` at
`center ± sqrt(RSph^2 - RCyl^2) * axisDir` (2 iff the offset `> RealEpsilon()`).
Else `NoGeometricSolution`.

**Cone x Cone** (L1433): `TOL_APEX_CONF = 1.e-10`; `tg1,tg2 = tan(SemiAngle)` sign-normalized
to `tg1*tg2 >= 0`. Adaptive angle tolerance (L1458–1475): if `|tg1-tg2| < Tol`, axes parallel,
`DistA1A2 > 100*Tol` → `aTolAng = clamp(Tol / EstimDist(Con1,Con2), Precision::Angular(), Tol)`
where `EstimDist` intersects the 4 projected silhouette lines in the common axis plane
(2D `IntAna2d_AnaIntersection`) and returns the min apex-to-solution distance — the further
the solution from the apexes, the looser the "equal-angle" test may be.
1. Coaxial (`A1A2.Same()`): different angles: apex offset `|d| < 1e-10` → `Point` (apex);
   else 2 `Circle`s at `x = d*tg2/(tg1+tg2)` (between-apex) and `x = d*tg2/(tg2-tg1)`,
   radius `|x*tg1|`. Equal angles: `|d| < 1e-10` → `Same`; else 1 `Circle` at `d/2`.
2. Parallel axes + equal semiangle (within `aTolAng`): the two cones share a symmetry plane;
   build the orthogonal plane through the computed characteristic point and recurse
   `IntAna_QuadQuadGeo(gp_Pln, Con1, Tol, Tol)` → re-emit its `Ellipse/Circle/Hyperbola/Line`
   verbatim (`Line` case = touching cones → 2 coincident line slots, nbint=2).
3. Coincident apexes (`apexDist^2 < Tol^2`): 2D unit-sphere test: `aRD2 > aR2+Tol` → `Empty`;
   touch (`>= aR2-Tol`) → 1 `Line` through apex; intersect → 2 `Line`s (constructed via
   plane-plane line + `± sqrt(R1^2 - aDa^2)` offset).
4. Axes intersect AND each apex lies ON the other cone (`ElSLib::Parameters` +
   re-eval + `SquareDistance <= Tol^2`): common-generatrix case → `myCommonGen = true`,
   `myPChar` = characteristic point (projection of axis-intersection onto the generatrix);
   build the "maximal plane" (through myPChar, normal from rotated generatrices) and recurse
   plane x cone → `Ellipse/Circle/Parabola/Hyperbola`. Consumers (GeomInt/IntTools) read
   `HasCommonGen()` to add the degenerate touch-point to the curve decomposition.
5. Else → `NoGeometricSolution` (→ Stage 4 cone-quadric trig solve).

**Sphere x Cone** (L1917): sphere center exactly on cone axis (same `== 0.0` test as CySp):
`ConDir` = apex→center (or axis if apexes coincide); solve
`(1+tga^2) x^2 + 2 tga^2 d x + (d^2 tga^2 - R^2) = 0` (`math_DirectPolynomialRoots`);
0 roots → `Empty`; else 1–2 `Circle`s centered on axis at `d+x`, radius `|tga*(d+x)|`;
any radius `<= myEPSILON_MINI_CIRCLE_RADIUS` (1e-9) → type `PointAndCircle` with that
`param = 0` (apex point contact + real circle). Else `NoGeometricSolution`.

**Sphere x Sphere** (L2034):
`d <= Tol && |R1-R2| <= Tol` → `Same`; `d <= Tol` (concentric) → `Empty`.
Internal tangency `0 <= Rmax - d - Rmin <= Tol` → `Point` (explicit two-sided formula).
`d > R1+R2+Tol` or `Rmax > d+Rmin+Tol` → `Empty`.
Else radical circle: `Alpha = (R1^2 - R2^2 + d^2)/(2d)`, `Beta = sqrt(max(0, R1^2-Alpha^2))`;
`Beta <= 1e-9` → `Point` at `(R1 + (d - R2))/2`; else `Circle` center `O1 + Alpha*Dir`,
radius `Beta`, dir1 = center line.

**Plane x Torus** (L2163): requires `RMin < RMaj` else `NoGeometricSolution`.
- Axis parallel to plane normal: `aDR = |dist| - RMin`; `aDR > 1e-13` → `Empty`;
  `|aDR| < 1e-13` → snap `dist = ±RMin`; `Circle`(s) center = projected torus center,
  radii `RMaj ± sqrt(|RMin^2 - dist^2|)`; 2 circles iff `aDR < -1e-13` and `aDt > Tol`.
- Axis normal to plane normal AND plane through center (`dist > myEPSILON_DISTANCE=1e-14` →
  `NoGeometricSolution`): 2 `Circle`s radius `RMin`, centers `loc ± RMaj*(axisDir x planeNormal)`
  (the two tube sections). Any oblique plane → `NoGeometricSolution` (Villarceau NOT handled).

**Cylinder x Torus** (L2278): coaxial required (parallel within `Angular` AND axis-line
distance `> 1e-14` → `NoGeometricSolution`). `RCyl+Tol < RMaj-RMin` or `RCyl-Tol > RMaj+RMin`
→ `Empty`. Else `Circle`(s) radius `RCyl` at `z = ±sqrt(|RMin^2 - (RCyl-RMaj)^2|)`; 2 iff
`aDist > Tol` and strictly inside the annulus.

**Cone x Torus** (L2357): coaxial required (apex on torus axis, same gates). Rotate the axis
line by `SemiAngle` about the tube-center normal → generator line `aConL`; for BOTH sides
(`aXDir`, `-aXDir`): `aDist = aConL.Distance(tubeCenter)`; `> RMin+Tol` → skip; else generator
pierces the tube: 1–2 `Circle`s per side (up to 4 total — the only 4-circle case; pt3/pt4,
param3/param4, dir3/dir4 slots), each center back-projected onto the axis, radius = distance
to axis.

**Sphere x Torus** (L2496): center on torus axis required. Meridian-plane circle-circle
(tube circle `RMin` at `(RMaj,0)` vs sphere section): `(aDist-Tol) > RMin+RSph` or
`(aDist+Tol) < |RMin-RSph|` → `Empty`; else `Alpha/Beta` chord solve → 1–2 `Circle`s of
revolution (radius measured back to the axis); 2 iff strict overlap AND `|Beta*aDC| > Tol`.

**Torus x Torus** (L2588): coaxial required (else `NoGeometricSolution`).
Same location + both radii within `Tol` → `Same`. Either `RMin >= RMaj` →
`NoGeometricSolution`. Else meridian circle-circle identical to Sphere x Torus with
`(RMin1, RMin2)` → `Empty` / 1–2 `Circle`s.

### Stage 4 — IntAna_IntQuadQuad: exact non-conic curves (ALines)
- **File/fn**: `IntAna_IntQuadQuad.cxx` — `Perform(gp_Cylinder, IntAna_Quadric, Tol)` (L375),
  `Perform(gp_Cone, IntAna_Quadric, Tol)` (L841). Only invoked by `IntCyCo` (and legacy
  paths); implicit operand = ANY quadric via its 10 polynomial coefficients
  (`IntAna_Quadric::Coefficients` then `NewCoefficients(..., Cyl.Position() /
  cone-apex frame)` rewrites them in the explicit surface's frame — full 3x4 congruence
  transform, `IntAna_Quadric.cxx` L148–248).
- Cylinder path: substitute `X=R cosT, Y=R sinT, Z=z` → `Qzz z^2 + B(T) z + C(T) = 0`.
  `|Qzz| < myEpsilonCoeffPolyNull (1e-8)` → **`done = false` — gives up** (a linear-in-z
  intersection is NOT handled here; caller falls back to marching).
  Discriminant `DIS(T)` is a degree-2 trigonometric polynomial with coefficients
  `C_1 = Qz^2 - Qzz*Q1`, `C_SS = R^2(Qyz^2 - Qyy*Qzz)`, `C_CC = R^2(Qxz^2 - Qxx*Qzz)`,
  `C_S = R(Qyz*Qz - Qy*Qzz)`, `C_C = R(Qxz*Qz - Qx*Qzz)`, `C_SC = R^2(Qxz*Qyz - Qxy*Qzz)`.
  Roots via `math_TrigonometricFunctionRoots` (wrapped in local `TrigonometricRoots`:
  normalizes to `[0,2PI]`, residual check `|F(root)| > 1e-8` → `done=false`, bubble-sort,
  all-coeff `< 1e-10` + cte `< 1e-10` → infinite roots).
  Case analysis on root count of DIS:
  - infinite roots → curve is a perfect square: 2 curves (Z+ branch, Z− branch) on `[0,2PI]`;
  - 0 roots: `DIS(PI) >= -RealEpsilon()` → 2 full-period curves, else no intersection;
  - 1 root: tangency; `DIS >= 0` elsewhere → 2 full-period curves (curve passes through the
    tangent point), else isolated tangent point (currently DROPPED — commented out, NbCurves=0);
  - >= 2 roots: per interval `[Ti, Ti+1]` with positivity established by 3-point sampling
    `qwet = DIS(mid) + DIS(0.4/0.6 mixes) >= 0`:
    double root (`|T2-T1| <= RealEpsilon()`) → tangent point on an otherwise-positive period:
    2 single-Z curves over `[T1, T1+2PI]` with `AddSpecialPoints` domain widening;
    near-coincident next root (`T3-T2 < 5.e-8`) → 2 single-Z curves on `[T1,T2]` (splits at
    the pinch so no curve passes through a double tangent point);
    else ONE curve with `DEUX_Z_PAR_THETA` (both z branches encoded in one param range —
    mirror parametrization, `myLastParameter = 2*DomainSup - DomainInf`).
- Cone path: frame at apex, `x = z tanB cosT` etc; `A(T) z^2 + B(T) z + C(T) = 0` where now
  ALL THREE coefficient sets are trig polynomials (`SetConeQuadValues`, `IntAna_Curve.cxx`
  L95: `Z2* = {Qzz/tan^2, 2Qyz/tan, 2Qxz/tan, Qxx, Qyy, Qxy}`, `Z1* = {2Qz/tan, 2Qy, 2Qx}`,
  `Z0Cte = Q1`).
  Solves `A(T)=0` (PolZ2 = asymptote directions), `B(T)=0` (PolZ1), and the discriminant.
  `A ≡ 0`: `B` no roots → 1 line-type curve; `B ≡ 0` too and `|Q1| <= 1e-8` → `done=false`.
  Discriminant constant-positive is simulated as a double root at 0. Positive intervals are
  SPLIT at every root of `A(T)` inside them (`to` values): each sub-interval gets a Z− and Z+
  single-Z curve, and the curve on the unbounded side of the asymptote is flagged
  `SetIsFirstOpen/SetIsLastOpen` by the sign of `B(T)` at the split (`MTFZ1.Value(to) < 0` →
  infinity on Z+ branch). Open ends = infinite branches (hyperbola-like arms).
- `AddSpecialPoints` (L61–118): widens `[T1,T2]` so that cone apex / sphere poles
  (`IntAna_Quadric::SpecialPoints()`: cone apex appended at `SetQuadric(gp_Cone)`, both poles
  at `SetQuadric(gp_Sphere)`) that ARE true intersection points
  (`SquareDistance < Precision::SquareConfusion()`) fall inside the curve domain —
  compensates boundary roots computed slightly inside.
- `InternalSetNextAndPrevious` (L1409): links curve endpoints into chains:
  params equal within `aEps = 1.e-7` AND 3D points within `aEPSILON_DISTANCE = 1.e-10`;
  sign encodes same-or-opposite orientation (`nextcurve/previouscurve` arrays, +/-(index+1)).
  Open (asymptotic) ends never link.

### Stage 5 — Consumption in IntPatch (GLine/ALine/WLine creation + transitions)
- **File/fn**: `IntPatch_ImpImpIntersection.cxx`.
- Conic results → `IntPatch_GLine(conic, isTangent, trans1, trans2)`; transition computed at
  one reference point: `qwe = Tgt.DotCross(Quad2.Normale(ptref), Quad1.Normale(ptref))`;
  `qwe > 1e-8` → `(Out, In)`, `< -1e-8` → `(In, Out)`, else `Undecided` (tangent).
  Ellipse pairs (cyl x cyl) get multiple-point vertices at the two tangent points
  (`Multpoint = true`), each ellipse gets vertices at param `0`/`2PI` (seam closure) plus
  the two mult points at `PI/2`, `3PI/2` (`CyCyAnalyticalIntersect`, L4841–5178).
  Parabola/Hyperbola from a cyl x cyl → `throw` ("Wrong intersection type").
- `IntCyCo` (L8374): QuadQuadGeo first; `NoGeometricSolution` → `IntAna_IntQuadQuad(Cy, Co, Tol)`;
  each `IntAna_Curve` is split at the cone apex by `ExploreCurve(Co, aC, 10*Tol, aLC)`
  (L8608: `FindParameter(apex)` → `SetDomain` sub-curves whenever the curve passes within
  `10*Tol` of the apex — an ALine through the apex is otherwise a single self-touching curve).
  Tangent vector for transition obtained by probing `D1u` at `para = (1.123*first + para)/2.123`
  up to 5 times (D1u fails in tangent zones); after 5 failures the ALine is kept Undecided.
  `ProcessBounds` closes/associates endpoints of consecutive ALines.
- `IntCyCy` (L7881): `CyCyAnalyticalIntersect` handles `Point/Line/Ellipse/Same/Empty`.
  On `NoGeometricSolution`: **analytic marching-free WLine construction** `CyCyNoGeometric`
  — NOT the IntWalk walker: `ComputationMethods::stCoeffsValue(aCyl1, aCyl2)` closed-form
  U2/V1/V2 as functions of U1, `WorkWithBoundaries::BoundariesComputing` → up to 2 valid
  U1-ranges; both operand orders are measured and the cylinder with the LARGER visible
  U-range sum becomes the driving parameter (`isToReverse = aSumRange[1] > aSumRange[0]`,
  L8057) because more range = more WLine points = better precision.
- Torus family `IntCyTo/IntCoTo/IntSpTo/IntToTo` (L9564–9644) → `TreatResultTorus` (L9648):
  ONLY `Empty`/`Circle` accepted; `NoGeometricSolution` → `return false` → ImpImp fails →
  Stage 1 falls back to full marching. Torus x torus circles are re-seamed
  (`AdjustToSeam`) when both quadrics are tori.

### Stage 6 — BOP-level pre-filter (IntTools)
- **File/fn**: `IntTools_FaceFace.cxx` — `isTreatAnalityc` (L249–324): for plane x cylinder
  with finite height, runs `IntAna_QuadQuadGeo(pln, cyl, 1e-8, tol, H)` BEFORE the real
  intersection; if the result is an `Ellipse` with `MajorRadius >= 100000 * MinorRadius`
  the pair is declared non-analytic (grazing plane → needle ellipse → approx/parametrized
  treatment instead). Also `PerformPlanes` uses `IntAna_QuadQuadGeo(pln1, pln2, 1e-8, TolTang)`
  directly (L2441).

---

## DATA STRUCTURES

- **`IntAna_ResultType`** (IntAna_ResultType.hxx): `Point, Line, Circle, PointAndCircle,
  Ellipse, Parabola, Hyperbola, Empty, Same, NoGeometricSolution`. `PointAndCircle` is
  cone x sphere only: one root circle degenerated to the apex (its `param == 0` marks which
  slot is the point).
- **`IntAna_QuadQuadGeo`** (hxx L252–281): `done, nbint (0..4), typeres`; solution slots
  `pt1..pt4 : gp_Pnt`, `dir1..dir4 : gp_Dir`, `param1..param4` (radius / major),
  `param1bis, param2bis` (minor radius for ellipse/hyperbola); 6 instance tolerances
  (below); `myCommonGen : bool` + `myPChar : gp_Pnt` (cone-cone common generatrix
  characteristic point). Accessors build `gp_Circ` via `DirToAx2(pt, dir)` — the X
  direction is synthesized from the smallest component of dir (L237–257), NOT stored:
  circle phase is arbitrary.
- **`AxeOperator`** (QuadQuadGeo.cxx L69–231): axis-pair relation oracle:
  `Parallel()` (via `IsParallel`, eps `Angular`), `Distance()` (common-perpendicular),
  `Coplanar()` (det33 < 1e-14), `Intersect()` (coplanar && !parallel), `Same()`
  (parallel && distance < eps), `Normal()`, `PtIntersect()`; `RefineDir` (L2867) snaps
  near-axis directions with components within `RealEpsilon()` of `±1` to EXACT unit axes.
- **`IntAna_Quadric`** (hxx L97–108): 10 coefficients `CXX..CCte` of
  `f = CXX x^2 + CYY y^2 + CZZ z^2 + 2(CXY xy + CXZ xz + CYZ yz) + 2(CX x + CY y + CZ z) + CCte`
  (note the FACTOR-2 convention on mixed and linear terms; plane sets `CX..CZ = A/2..C/2`);
  `mySpecialPoints : list<gp_Pnt>` (cone apex, sphere poles).
  `NewCoefficients(..., gp_Ax3)`: exact congruence rewrite into another frame.
- **`IntAna_IntQuadQuad`** (hxx L121–132): `TheCurve[12] : IntAna_Curve`,
  `previouscurve[12], nextcurve[12] : int` (signed chain links), `NbCurves, Nbpoints`,
  `Thepoints[2]`, `myNbMaxCurves = 12`, `myEpsilon = 1e-8`, `myEpsilonCoeffPolyNull = 1e-8`.
- **`IntAna_Curve`** (hxx L134–169): 18 trig coefficients (3 z-degrees x 6 basis
  {Cte, Sin, Cos, SinSin, CosCos, CosSin}); `TwoCurves` (mirror-encoded second branch),
  `TakeZPositive` (which quadratic root); `DomainInf/Sup` + trim `myFirstParameter/
  myLastParameter` (mirror domain = `[Inf, 2*Sup-Inf]` when `TwoCurves`);
  `firstbounded/lastbounded` = OPEN (asymptotic) end flags; `typequadric`
  (Cylinder/Cone), `RCyl`, `Angle`, `Ax3` for `ElSLib` point evaluation.
  `Value(T)`: quadratic solve with derivative-scaled discriminant snap; V hard-clamped to
  `±100000`; cone V remap `(V - RCyl)/sin(Angle)`.
  `D1u`: finite difference `dT = (Sup-Inf)*1e-6`, FAILS (returns false) when `|A| < 1e-7`
  or `|sqrtDis| < 1e-10` (tangent zone). `FindParameter(P)`: candidate params
  `{DomInf, DomSup, proj T, mirror T, mirror end}` filtered by 3D re-eval distance.
- **`TrigonometricRoots`** (IntQuadQuad.cxx L124): sorted roots in `[0,2PI]` of
  `CC cos^2 + 2 SC sincos + C cos + S sin + Cte`; `IsARoot(u)` modulo-2PI eps `RealEpsilon`;
  `InfiniteRoots` flag. **`MyTrigonometricFunction`**: value/derivative of the full
  degree-2 trig polynomial.
- **IntPatch line types**: `IntPatch_GLine` (gp conic + transitions + vertices),
  `IntPatch_ALine` (IntAna_Curve + transitions), `IntPatch_WLine` (point chain). ALine →
  WLine sampling count = 200 (`GeomGeomPerfom`).

---

## CONSTANTS & TOLERANCES

`IntAna_QuadQuadGeo::InitTolerances` (QuadQuadGeo.cxx L351–359):
| name | value | used for |
|---|---|---|
| `myEPSILON_DISTANCE` | `1.0e-14` | axis coincidence / coplanarity dets; plane-through-center tests (torus) |
| `myEPSILON_ANGLE_CONE` | `Precision::Angular()` = `1e-12` | equal-semiangle test cone-cone |
| `myEPSILON_MINI_CIRCLE_RADIUS` | `0.01 * Precision::Confusion()` = `1e-9` | circle → point collapse (sph-sph, sph-cone `PointAndCircle`) |
| `myEPSILON_CYLINDER_DELTA_RADIUS` | `1.0e-13` | equal-radius cyl-cyl (RELATIVE: `RmR/Rmax`); plane-torus `aTolNum` |
| `myEPSILON_CYLINDER_DELTA_DISTANCE` | `Precision::Confusion()` = `1e-7` | cyl-cyl AxeOperator distance eps |
| `myEPSILON_AXES_PARA` | `Precision::Angular()` = `1e-12` | all axis-parallel gates |

Other exact values:
- `AxeOperator` defaults: `theEpsDistance = 1.e-14`, `theEpsAxesPara = Precision::Angular()`.
- Pln-pln: denom guard `aEps = 1.e-16`; origin refinement gates `aTreshAng = 2.e-6` rad,
  `aTreshDist = 1.e-12`.
- Pln-cyl near-parallel: trigger `sinda * H < 2*Tol`; relaxed `tolang = 2*sinda`,
  `toltang = max(Tol, sinda*H*1.01)`.
- Pln-sphere tangency: `Epsilon(radius)` (machine ULP of r — not Tol!).
- Conic rejection: `EllipseLimit = 1.0E+9` (OCC513), `HyperbolaLimit = 2.0E+6` (OCC537).
- Cone-cone: `TOL_APEX_CONF = 1.e-10`; adaptive `aTolAng` triggered at `DistA1A2 > 100*Tol`,
  clamped to `[Precision::Angular(), Tol]`, formula `Tol / EstimDist`.
- Cyl-cyl parallel tangency: outer `(R1+R2-Dist) <= RealSmall()` (denormal-min ~2.2e-308);
  2-line chord collapse `4 R1^2 sin^2 < Tol^2`; inner `Dist > RmR - Tol`.
- Cyl-sph / sph-cone axis test: `Pt.Distance(PtIntersect()) == 0.0` — EXACT zero.
- IntQuadQuad: `myNbMaxCurves = 12`, `myEpsilon = 1e-8`, `myEpsilonCoeffPolyNull = 1e-8`;
  TrigonometricRoots residual reject `|F(root)| > 1e-8`; null-poly detect `1e-10`;
  near-double root split `(T3 - T2) < 5.e-8`; interval-sign sampling at
  `{0.5, 0.4/0.6, 0.6/0.4}` blends; chain link eps `1e-7` (param) / `1e-10` (3D).
- IntAna_Curve: discriminant snap `aTolD = 2*aDT*|B*dB - 2(A*dC + C*dA)|` with
  `aDT = 100*Epsilon(2*DomSup - DomInf)`; V clamp `±100000`; `D1u` fail gates `|A| < 1e-7`,
  `|sqrtDis| < 1e-10`, step `(Sup-Inf)*1e-6`; `FindParameter` `anEpsAng = 1e-8`,
  `InternalPrecision = 1e-8` (squared-dist tol for projected candidates),
  `Precision::SquareConfusion()` for boundary candidates; linear fallback `|A| <=
  Precision::PConfusion()` → `V = -C/B`.
- IntPatch: ImpImp local `Tolang = 1.e-8`; transition threshold `qwe| > 0.00000001` (1e-8);
  cone degeneracy gates `0.02` / `1.55` rad; plane-cone axis-in-plane `ps < 0.015`;
  cyl-cyl `a2DTol = min(1.0e-4, UResolution(TolTang))`; ALine sampling 200 points;
  `ExploreCurve` apex tube `10 * Tol`.
- IntTools_FaceFace: needle-ellipse reject `MajorR >= 100000 * MinorR`; `Tolang = 1.e-8`.

---

## INVARIANTS

1. **Analytic-first, marching-fallback, never both**: every pair tries the geometric case
   table first; `NoGeometricSolution` routes to the trig solver (cyl/cone explicit only) or
   to walking; `done = false` ANYWHERE (extreme conic, Qzz≈0, root-validation failure)
   silently reroutes the whole pair to Prm-Prm marching. There is no partial mixing of
   analytic and marched output for one pair.
2. **Only cylinders and cones are ever explicit parametrization hosts** in IntQuadQuad;
   sphere/torus never (sphere handled by case table or as implicit operand; torus only by
   case table or marching). An ALine is always `T ∈ [0,2PI]-ish` on a cyl/cone.
3. **Result conics are frame-canonical**: circle/ellipse axes are constructed from the
   configuration (axis dirs, cross products), the phase (`XDirection`) is arbitrary
   (`DirToAx2`), so consumers must never rely on conic parameter origin — OCCT re-derives
   vertex params via `ElCLib::Parameter` (e.g. ellipse tangent-point ordering in
   `CyCyAnalyticalIntersect`).
4. **Tangency collapses are two-sided**: every "circle/2-lines" case has a paired
   "point/1-line" collapse under `Tol` (chord < Tol, radius < 1e-9, `|dist ± r| < Tol`),
   and the collapse produces the CONTACT element, not Empty — point contacts are kept as
   `IntPatch_Point` (spnt), tangent lines as single lines. Empty requires clearance > Tol.
5. **Same-surface detection lives here**: `Same` from the case table (pln-pln, cyl-cyl,
   sph-sph, cone-cone, tor-tor) is what sets `TangentFaces` upstream — booleans branch to
   same-domain face-face treatment (see kb/occt_ff-posttreat-samedomain.md) purely off this
   verdict.
6. **Mirror parametrization invariant** (IntAna_Curve): when `TwoCurves`, params in
   `[Sup, 2*Sup-Inf]` map to `2*Sup - T` on the OTHER quadratic root; the fold point
   `T = Sup` is the null-discriminant (tangency) point and `Value` snaps it exactly.
7. **Chain topology is endpoint-exact**: ALine chains (`next/previouscurve`) require BOTH
   param proximity (1e-7) and 3D proximity (1e-10); open (asymptotic) ends are excluded —
   an ALine chain never silently bridges across an asymptote.
8. **Special points must lie inside domains**: apex/pole intersection points are pulled into
   curve domains (`AddSpecialPoints`) and curves are SPLIT at the apex (`ExploreCurve`) —
   the apex is a curve endpoint, never an interior point, in the delivered decomposition.
9. **Driving-parameter choice is coverage-greedy** (CyCyNoGeometric): the operand whose
   visible U-range is larger drives; result is order-independent only for the analytic
   (conic) outcomes.
10. **All gates are dimensionless or caller-Tol-scaled** except the hard-coded absolute ones
    (1e-14 axis distances, 1e-13 relative radius, 1e-9 mini-circle) — OCCT assumes
    model-scale coordinates (mm-ish); these do NOT scale with the model.

---

## PITFALLS

1. **The exact-zero axis tests**: cyl-sphere and cone-sphere require
   `Pt.Distance(PtIntersect()) == 0.0` — bitwise. Any FP noise in the sphere center
   (e.g. after rotation) silently demotes a coaxial case to `NoGeometricSolution` →
   marching → the tangent-circle family degrades. OCCT survives because its primitives
   are constructed in canonical frames; imported (STEP) geometry with recomputed centers
   will miss these branches. Our reader-recognized quadrics have the same hazard —
   snapping centers onto axes during recognition is load-bearing.
2. **Cone x cone `Tol^2` apex test but `Tol` line tests**: case 3 uses squared distance vs
   `aTol2 = Tol*Tol`; with Tol=1e-7, apexes 1e-5 apart are "distinct" but the 2D
   touch test uses plain `±Tol` — inconsistent sensitivity, known source of jitter between
   1-line and 2-line outcomes for near-apex-coincident cones.
3. **`IntQuadQuad` cylinder path refuses `Qzz ≈ 0`** (implicit quadric linear in the
   cylinder's z): e.g. cylinder vs a PLANE fed through the generic path → done=false.
   Not a real gap (plane pairs never route here) but any port must preserve the routing
   assumption: IntQuadQuad's implicit operand is never a plane through the cyl axis frame.
4. **Isolated tangent point is DROPPED on the cylinder path** (1-root case, else-branch
   L621–632: the point computation is commented out, `NbCurves = 0`) — a cylinder
   externally tangent to an implicit quadric along a point yields NOTHING from IntQuadQuad;
   the contact survives only if QuadQuadGeo classified it first (`IntAna_Point`). Parity
   check: our pipeline must emit point contacts from the case table, never expect them from
   the trig fallback.
5. **Ellipse `param1`/`param1bis` are not ordered**: `Ellipse(n)` swaps major/minor at
   accessor time (hxx contract requires major >= minor for gp_Elips); hyperbola accessor
   does NOT swap and branch 2 reverses `dir2`. Copying param slots without the accessor
   swap logic produces invalid gp_Elips (throws) for `cost` near the circle transition.
6. **Plane x cylinder near-parallel "newparams" mode changes the LINE DIRECTIONS**: the
   emitted tangent/secant lines follow the projected (tilted) axis, not the true cylinder
   rulings — intentional (face-height-limited parallelism), but a port that reuses the
   axis direction for the lines will diverge from OCCT by up to `sinda` over the face.
7. **`RefineDir` snapping is asymmetric**: only components within one ULP of ±1 are snapped
   (then the OTHER two zeroed). Directions like (0.5, 0.5, 0.7071...) are untouched; do not
   expect general axis cleanup from it.
8. **Trig-root residual rejection kills the whole solve**: one root of the discriminant
   polynomial with residual > 1e-8 → `TrigonometricRoots.done = false` → `IntQuadQuad`
   `done = false` → marching. Large quadric coefficients (big radii, far-from-origin
   frames) inflate residuals; OCCT does NOT normalize the implicit quadric before the
   residual check. Port should normalize coefficients (divide by max |coeff|) before the
   1e-8 test or inherit scale fragility.
9. **`InternalValue` V-clamp ±100000** silently flattens far asymptotic branches — ALine
   points beyond that distance are WRONG (clamped); only harmless because BOP boxes clip
   first. A port evaluating ALines outside a bounded region must carry the clamp or bound
   the domain first.
10. **Torus analytic coverage is coaxial-only** — any tilted torus pair (even
    tangent-circle configurations rotated off-axis) routes to marching. This matches our
    z15/z30 rotated-chair regressions: rotation exits the entire analytic family. OCCT
    accepts this and leans on the walker + approx; parity does not require tilted-torus
    conics.
11. **`PointAndCircle` slot convention**: which of pt1/pt2 is the point is encoded by
    `param == 0.0`, checked in `Point()`/`Circle()` accessors — an easy port bug.
12. **Cyl-cyl equal-radius test is RELATIVE (1e-13)** but the axis-intersect requirement
    uses coplanarity dets vs ABSOLUTE 1e-14 — two crossing cylinders of equal radius whose
    axes miss by 1e-9 fall to `NoGeometricSolution` and come back as WLines, not the exact
    Steinmetz ellipses. Deliberate: near-miss ellipses would be wrong; the walker finds the
    true near-elliptic curves.

---

## PORT MAP

Anchors: `brep.cpp` `recognize_solid` (L7761), boolean gate + S1 shared-SSI routing
(L8007–8036), conic registry / FINAL PASS 2 exact-arc rebuild (L5285, L5780–5900),
`split_with` (UV arrangement), `combine` (exact weld), winding+radial classification
(L8557); `brep_section.cpp` `build_section_scaffold`.

1. **IntAna_QuadQuadGeo case table → `quad_quad_geo()` (NEW-BUILD, highest value)**.
   Our conic registry currently derives only sphere-sphere radical circles and equal-radius
   Steinmetz ellipses (brep.cpp L5780 comment). Port the full table (15 pairs incl. all 9
   torus-family coaxial circle cases, plane-cone 5-conic split, cone-cone 5 branches) as a
   free function returning `{type, up-to-4 conic slots, common_gen, pchar}` and make the
   registry consult it FIRST for every recognized-pair imprint; keeps our exact rational
   arcs (we already rebuild bit-identical arcs for both mates) but widens coverage from 2
   configurations to the whole table. Adopt `InitTolerances` values verbatim.
2. **Analytic-eligibility gates → scaffold routing (ADOPT into existing gate)**.
   Our S1 gate is `imported_freeform && !scaffold_eligible` (L8020–8036); OCCT's inverse
   gate is principled: cone semiangle `<0.02 / >1.55`, torus non-coaxial, needle-ellipse
   `major >= 1e5*minor`, `EllipseLimit 1e9 / HyperbolaLimit 2e6` → route to marching even
   for analytic surfaces. Adopt these numeric gates in `scaffold_eligible` so cone x cone /
   tilted-torus pairs (our known no-recognizer family) are routed by the same criteria OCCT
   uses, instead of by surface-degree heuristics.
3. **IntQuadQuad + IntAna_Curve → exact trig ALine for cyl-cone/cyl-quadric sections
   (NEW-BUILD, replaces marched chains where applicable)**. `build_section_scaffold`
   marches ALL non-conic quadric sections today. Port `SetCylinderQuadValues/
   SetConeQuadValues` + `Value/D1u` (18 trig coeffs, mirror parametrization, discriminant
   snap `aTolD`) to evaluate exact section curves for cylinder/cone x quadric; sample them
   like OCCT (200 pts) into our chain format so paves/keep-verdict/welds are unchanged.
   Include `ExploreCurve`-style apex splitting — mirrors our seam-decomposition chain
   splitting (chains split at periodic-seam jumps) and prevents self-touching chains at the
   apex.
4. **AxeOperator + RefineDir → recognizer canonicalization (ADOPT)**. In `recognize_solid`
   and the STEP-reader quadric recognizer, snap axis dirs (`RefineDir`) and DECIDE
   coaxial/parallel/intersect with AxeOperator's exact tolerance set (1e-14 / Angular),
   then SNAP centers onto axes when within tolerance — this is what makes OCCT's `== 0.0`
   coaxial branches reachable and is directly the fix class for our rotated-frame
   recognition losses (pitfall 1).
5. **Tangency-collapse ladder → keep-verdict parity (ADOPT semantics)**. Our winding/radial
   classification treats tangent contacts via quorum/flux; OCCT resolves them UPSTREAM:
   chord `< Tol` → single line, radius `< 1e-9` → point, `PointAndCircle` for apex-through
   contact, `Undecided` transitions (`|qwe| <= 1e-8`) for tangent curves. Emit contact
   elements (point/line) from stage-3 classification instead of letting sub-Tol slivers
   reach the arrangement — this is the same defect family as our SEGWHOLE/sliver key drops.
6. **`Same` verdict → same-domain path (REFERENCE)**. Our same-domain imprint infra
   (extra_cuts) should key off a stage-1-style `Same` classification (all five same-type
   pairs, exact criteria above) rather than per-face coincidence sampling; see
   kb/occt_ff-posttreat-samedomain.md for the downstream contract.
7. **ALine→WLine sampling + ExtendTwoWLines critical points (REFERENCE, already covered)**:
   endpoint completion to boundaries and apex/pole critical-point handling are specified in
   kb/occt_ssi-walking.md Stage 5; the only addition from this spec is WHERE the 200-point
   sampling constant and the critical-point list (cone apex, sphere poles ±PI/2) originate.
