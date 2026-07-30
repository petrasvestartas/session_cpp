# audit: occt2_intana-analytic.md vs OCCT V8 source

Source tree audited: `/home/petras/code/code_cpp/OCCT` @ `37dd5686f2` (8.0.1.dev). All citations below are
paths relative to that root. Note: `IntAna` lives in `src/ModelingData/TKGeomBase/IntAna/`, not in
`TKGeomAlgo` (the task prompt's path is wrong; the spec's own header path is right).

---

## 1. VERDICT

**Substantially faithful on the case table, materially wrong on the consumption layer.**
Every `IntAna_QuadQuadGeo` line number in the spec resolves to the correct function in the real file
(L351/389/543/752/980/1050/1324/1373/1433/1917/2034/2163/2278/2357/2496/2588 all confirmed), the
tolerance table is exact, and the IntQuadQuad trig-solver description is accurate in structure. The
spec is **not** reliable on: (a) how tangency is *represented* downstream — it collapses three distinct
encodings into one; (b) which pairs reach the ALine solver; (c) transition thresholds, which are not
uniform; (d) the cyl×cyl coplanarity tolerance; (e) apex line splitting. Eight material errors:

- **E1 — ALine solver reach.** Spec: `IntAna_IntQuadQuad` is "only invoked by `IntCyCo` (and legacy
  paths)". False. `IntCySp` calls it at `IntPatch_ImpImpIntersection.cxx:8266` and `IntCoSp` at
  `:9351`. Non-coaxial cylinder×sphere and cone×sphere produce exact ALines, never marched WLines.
- **E2 — Tangency is NOT `Undecided`.** Spec: tangent curves get `Undecided` transitions when
  `|qwe| <= 1e-8`. Real: a tangent conic is built with a *different* `IntPatch_Line` constructor which
  sets `tS1 = tS2 = IntSurf_Touch` plus `IntSurf_Situation` (`Inside`/`Outside`) —
  `IntPatch_Line.cxx:35-48`. `Undecided` (`IntPatch_Line.cxx:50-61`, or explicit `trans1=trans2=
  IntSurf_Undecided`) means "transition *unknown*", a different downstream contract
  (`SituationS1()` throws unless `tS == Touch`, `IntPatch_Line.lxx:52-58`).
- **E3 — transition thresholds are not uniform.** `IntPP:3136`, `IntPCy:3261,3304,3325`,
  `IntPSp:3419`, `IntPTo:3833`, `IntCySp` first circle `:8227`, `IntCoCo` 2-line `:8803` use bare
  `> 0.0` with **no** dead band at all. `1e-8` is used in `IntPCo:3731,3761`, `CyCyAnalytical
  :4997,5052,5128`, `IntCyCo:8445,8548`, `IntCoCo:8878,8917,8955`, `IntSpSp:9530`,
  `TreatResultTorus:9686`. `1e-7` in `IntCySp:8243`. `Precision::PConfusion()` (1e-9) in
  `IntCoSp` PointAndCircle `:9312,9317`. `1e-9` literal in `IntCoSp` ALine `:9416`.
- **E4 — `Same`/TangentFaces.** Spec invariant 5 says `Same` from the case table sets `TangentFaces`.
  Incomplete: `tgte` is set at exactly one place, `IntPatch_ImpImpIntersection.cxx:2865-2869`, by
  `SameSurf || (all1 && all2)`, where `allN = solrst.AllArcSolution() && typs1 == typs2`
  (`:2795,2834`). A pair whose case table says *not* Same still becomes tangent-same if every
  restriction arc of both faces is a solution — and `slin`/`spnt` are then **cleared** (`:2870-2871`).
- **E5 — cyl×cyl coplanarity eps (spec PITFALL 12 is wrong).** For cyl×cyl the `AxeOperator` is built
  with `myEPSILON_CYLINDER_DELTA_DISTANCE = Precision::Confusion() = 1e-7`
  (`IntAna_QuadQuadGeo.cxx:1054-1057`), so *both* `thedistance < eps` and `|D33| <= eps` use **1e-7**,
  not 1e-14 (`:159-174`). Two equal-radius cylinders whose axes miss by 1e-9 **do** get exact
  Steinmetz ellipses, contrary to the spec's claim. 1e-14 applies only to the default-constructed
  operators in CyCo/CySp/CoCo/SpCo (`:1327,1377,1456,1923`).
- **E6 — plane×cone through the apex emits 2 or 4 GLines, not 1 or 2.** Each algebraic line is split
  at the apex into two opposite rays, each a separate `IntPatch_GLine` carrying the apex as a vertex:
  tangency → 2 lines (`:3570-3596`), interior → 4 lines (`:3617-3675`). Same for cone×cone
  (`:8774-8787` and `:8810-8827`).
- **E7 — `RefineDir` precondition.** Spec: "components within one ULP of ±1 are snapped". Real
  (`IntAna_QuadQuadGeo.cxx:2874-2888`): it runs **only** if some component is *exactly* `±1.0` (`m`)
  **and** some other is nonzero-non-unit (`n`). `(0.9999999999999999, 1e-9, 0)` is left untouched.
- **E8 — ALine chaining.** `InternalSetNextAndPrevious()` is called only from the **cone** path
  (`IntAna_IntQuadQuad.cxx:1395`); the cylinder `Perform` returns at `:825` without chaining, so
  cyl×quadric ALines never get `next/previouscurve` links.

Minor: spec says `IntQuadQuad` drops the isolated tangent point "on the cylinder path" — in fact
`Nbpoints` is never incremented on **either** path (cone path has no point extraction at all), so
`IntAna_IntQuadQuad::NbPnt()` is always 0.

---

## 2. PORT-CRITICAL DETAILS THE SPEC MISSED

### 2.1 Tangency: representation, keep/purge, situation computation

- **Three encodings**, all reachable in one boolean:
  1. `IntPatch_Point::SetValue(P, Tol, isTangent)` — 3rd arg is the tangency flag.
  2. tangent curve: `IntPatch_GLine(c, /*Tang=*/true, Situation1, Situation2)` →
     `tg=true, tS1=tS2=IntSurf_Touch, sit1/sit2` (`IntPatch_Line.cxx:35-48`).
  3. transversal curve with unresolvable sign: `tS1=tS2=IntSurf_Undecided`, `tg=false`.
- **Which pairs emit `Touch`+Situation** (i.e. the analytic tangency ladder):
  `IntPCy` NbSol==1 (`:3203-3254`), `IntPCo` NbSol==1 (`:3515-3596`), `IntCySp` NbSol==1
  (`:8172-8223`), `CyCyAnalyticalIntersect` NbSol==1 (`:4900-4985`), `IntCoCo` NbSol==1
  (`:8732-8787`).
- **Situation is a curvature-side test, not a cross-product sign.** Canonical form
  (`IntPCy:3205-3244`): `TestCurvature = vec(tangency point → cylinder axis location)`; if
  `Normp·TestCurvature > 0` the cylinder is `Outside` else `Inside`; the plane's situation then
  follows from `sign(Normp·Normcyl)`. `IntCySp:8174-8213` uses `vec(ptref → sphere centre)` with
  `Normcyl` as the probe. `CyCyAnalyticalIntersect:4906-4982` uses both radius-vectors: if
  `crb1·crb2 < 0` (opposed curvature centres) each cylinder's situation is decided by the *other's*
  normal; otherwise the smaller-radius cylinder decides. `IntCoCo:8746-8773`: if `N1·N2 < 0` both
  are `Outside`; else the cone whose axis is farther from the probe point is `Outside`.
- **Nappe purge for cone contacts (not in spec at all).** A cone×cone or cone×sphere contact point is
  **discarded** unless it lies on the positive nappe: `param1 >= paramapex1 && param2 >= paramapex2`
  (`IntCoCo:8858`), `param >= paramapex` (`IntCoSp:9245`), where `param = ElCLib::LineParameter(
  Co.Axis(), ptcontact)`. Contacts coincident with the apex(es) within `Tol` are kept but flagged
  `isTangent = false` (`:8851-8857`, `:9238-9244`); genuine nappe tangency is flagged `true`.
- **`PointAndCircle` transition sense flips by nappe** (`IntCoSp:9310-9343`): if the circle centre's
  axis parameter is below the apex parameter, `(Out,In)` and `(In,Out)` are swapped; dead band is
  `Precision::PConfusion()`, not 1e-8. The apex point is emitted with `isTangent = false` (`:9301`).
- **Cyl×cone external tangency is unreachable analytically.** `Perform(Cyl,Con)` returns only
  `Circle` or `NoGeometricSolution` (`IntAna_QuadQuadGeo.cxx:1324-1344`), so the `IntAna_Point`
  branch in `IntCyCo:8423-8432` is dead code (and buggy — it calls `Quad1.Parameters` twice,
  `:8426-8427`). The tangency then falls to `IntAna_IntQuadQuad`, which drops isolated tangent
  points → **cylinder-tangent-to-cone yields nothing**. This is exactly the grazing-residue class.
- **ALine tangent-zone probing** (`IntCyCo:8534-8543`, `IntCoSp:9402-9411`): `para` is walked by
  `para = (1.123*first + para)/2.123` up to 5 times until `D1u` succeeds; on the 6th failure the ALine
  is built with the 2-arg constructor → `Undecided` (`IntPatch_ALine.cxx:54-63`).
- `ProcessBounds` receives `procf = !firstp = IsFirstOpen` (`:8574-8575`): **open (asymptotic) ends
  are treated as already processed and never get a vertex**; a closed ALine (`ptf.Distance(ptl) <=
  Tol`) gets one *multiple* vertex duplicated at both ends (`:4789-4801`).

### 2.2 Grazing numerics — where a near-tangency becomes a double root or vanishes

- `math_TrigonometricFunctionRoots` uses the Weierstrass substitution `t = tan(θ/2)`; quartic
  coefficients `ko = {A−C+E, 2D−4B, 2E−2A, 4B+2D, A+C+E}`
  (`src/FoundationClasses/TKMath/math/math_TrigonometricFunctionRoots.cxx:357-361`). **θ = π is not
  representable**; it is re-inserted by a trailing special case only when `|A − C + E| <= Eps`
  (`:513-516`), and that loop can append π up to 4 times, deduplicating only on its first iteration
  (`:505-535`). A tangency at θ=π therefore surfaces as a *repeated* root — precisely what feeds
  IntQuadQuad's `UnPtTg` branch.
- Degeneracy tests are **absolute**: `Eps = 1.5e-12` on raw coefficients (`:92`), Newton tol
  `Tol1 = 1e-15`, `Nit = 10` (`:87,459`). Newton result is rejected if it moves more than 1% of the
  interval (`SupmInfs100`, `:454,470-478`).
- **Coefficient-rescaling loop** (`:401-432`): if two sorted roots differ by < 1.5e-12 but the
  quartic derivative there exceeds 1.5e-12, OCCT declares numerical error, multiplies **all five**
  coefficients by `1e-4`, and re-solves — repeating until the test passes. Root sets for grazing
  configurations are therefore magnitude-dependent.
- `TrigonometricRoots` wrapper (`IntAna_IntQuadQuad.cxx:199-285`): roots normalized into `[0,2π]`
  *before* sorting (`:230-241`); residual reject `|F(root)| > 1e-8` sets `done=false` for the whole
  solve (`:245-256`); infinite-roots detection is `Σ|coef| < 1e-10 && |Cte| < 1e-10` (`:275-284`).
- **Interval positivity is a 3-sample SUM**, not a min: `MTF(0.5(T1+T2)) + MTF(0.4T1+0.6T2) +
  MTF(0.6T1+0.4T2) >= 0` (`:738-739` cylinder, `:1085-1086` cone). A narrow negative dip is
  outvoted.
- Cylinder path double-root thresholds are **three different numbers**: `RealEpsilon()` (2.2e-16) in
  the `nbsolDIS == 2` pre-pass (`:670`), `1e-12` in the general interval loop (`:731`), and `5e-8`
  for "next root is nearly coincident → split at the pinch" (`:757`). The pre-pass runs **only** when
  `nbsolDIS == 2` (`:660`) and probes the sign at `Theta1 ± 0.1` rad (`:673-679`).
- **Cone path aborts where the cylinder path skips**: `|Theta2 − Theta1| <= myEpsilon (1e-8)` →
  `done = false; return` (`:1079-1083`), killing the whole pair; the cylinder path merely drops that
  interval. Also `DiscriminantConstantPositif` sets `Theta2 = 2π − 1e-8` (`:1071`) — a closed curve
  is delivered 1e-8 short of closure, so its chain never links.
- `IntAna_Curve::InternalUVValue` discriminant snap is **one-sided**:
  `if (aDiscriminant < aTolD) aDiscriminant = 0.0` with
  `aTolD = 2*aDT*|B*aDB − 2(A*aDC + C*aDA)|`, `aDT = 100*Epsilon(2*DomainSup − DomainInf)`
  (`IntAna_Curve.cxx:292,345-356`). *Any* negative discriminant, however large, becomes 0 → `Value()`
  silently returns the tangency vertex `−B/(2A)` instead of failing.
- Real bug in the same block: `aDA` uses `aCos2t * (Z2CosSin * Z2CosSin)` where `aDB`/`aDC` use
  `(x + x)` (`IntAna_Curve.cxx:331` vs `:337,343`). Squared instead of doubled; affects only `aTolD`,
  and only the cone path (`Z2CosSin = Qxy`; identically 0 for cylinders, `:206`).
- `D1u` fails exactly in the tangent zone: `|A| < 1e-7 || |sqrtDis| < 1e-10` (`:412`); step
  `dtheta = (DomainSup − DomainInf)*1e-6` with sign flip near the far end (`:418-425`).
- Linear fallback: `|A| <= Precision::PConfusion()` → `V = −C/B`, and if also `|B| <= PConfusion` →
  **`V = 0.0`** silently (`:358-368`), producing a point on the axis/apex circle rather than an error.
- `FindParameter` acceptance is a **squared** tolerance used as if linear: `InternalPrecision = 1e-8`
  compared against `SquareDistance` → effective 1e-4 in 3D for projected candidates; boundary
  candidates use `Precision::SquareConfusion()` → 1e-7 in 3D (`:442-445,516-529`). And
  `std::sort(aParams, aParams + aMaxPar - 1)` sorts only the first 4 of 5 entries (`:493`).

### 2.3 Case-table details with load-bearing detail the spec omits

- **plane×cylinder relaxation** (`IntAna_QuadQuadGeo.cxx:571-598`): gate is
  `|angle(axisDir, planeNormal)| > π/4`; `dangle = ||angle| − π/2|`; trigger is
  `dif = |sinda − Tol| < Tol` **OR** `(H > 0 && sinda*H < 2*Tol)` — the spec has only the second
  disjunct, but the first fires with no face-height information at all. Then `tolang = 2*sinda`,
  `toltang = max(Tol, sinda*H*1.01)`. In `newparams` mode the emitted line directions come from
  re-projecting the axis translated by **100 units** (`:619,649`), and the 2-line offset uses
  `ht = sqrt(max(0, R² − distt²))` (`:653-654`).
- **`IntAna_IntConicQuad::Perform(Lin, Pln, Tolang, Tol, Len)`** (`IntAna_IntConicQuad.cxx:436-492`)
  is the actual parallel oracle: `parallel = |N·D| < Tolang`; then, if `Len != 0 && Direc != 0`, the
  point `aP1 + Len*D` (aP1 = origin projected onto the plane) must stay within `Tol` of the plane or
  parallelism is **revoked** (`:464-475`). When parallel, `inquadric = |Dis| < Tolang` — a distance
  compared against an angular tolerance (`:479`).
- **plane×sphere** (`:997`): tangency band is `Epsilon(radius)` ≈ 2.2e-16·R. For R = 100 that is
  1.4e-14, so a plane 1e-9 inside tangency yields a **circle of radius ≈ 4.5e-4**, never a point.
  `dir2 = P.Position().XDirection()` and `dir1` is reversed when `!P.Direct()` (`:1011-1016`).
- **cyl×cyl parallel ladder, exact order** (`:1070-1220`): `Dist ≤ Tol` → `Same` iff `|R1−R2| ≤ Tol`
  else `Empty`; `Dist > R1+R2+Tol` → `Empty`; `(R1+R2−Dist) ≤ RealSmall()` → 1 tangent line;
  `Dist > |R1−R2|` → 2 lines *with* the chord collapse `4R1²(1−aCos²) < Tol²` → 1 line at
  `P1 + R1·aCos·û` (`:1154-1168`); `Dist > |R1−R2| − Tol` → 1 internal tangent line (ratio negated
  when `R1 < R2`); else `Empty`. **`RealSmall()` ≈ 2.2e-308** makes the outer-tangency branch
  effectively unreachable — external tangency is actually caught by the chord test inside the 2-line
  branch.
- **cyl×cyl crossing, equal radius**: `A = |sin(½·angle)|`, `B = |sin(½(π−angle))|`,
  `param2 = R/A`, `param1 = R/B`, minors `= R`, **already swapped to major-first inside `Perform`**
  (`:1250-1265`); `A == 0 || B == 0` → `Same` (`:1239-1243`).
- **cone×cone adaptive `aTolAng`** (`:1458-1475`): only when `|tg1−tg2| < Tol && Parallel &&
  DistA1A2 > 100·Tol`; if `EstimDist < Epsilon(1.)` then `aTolAng = Tol` (spec missed this
  sub-branch), else `clamp(Tol/EstimDist, Angular, Tol)`. `EstimDist` returns
  `Precision::Infinite()` when the 3-point plane fails (`:273-275`) → `aTolAng = Angular`.
- **cone×cone recursion uses `Tol` as the ANGULAR tolerance**:
  `IntAna_QuadQuadGeo(gp_Pln(...), Con1, Tol, Tol)` at `:1555` (parallel-equal-angle branch) and
  `:1832` (common-generatrix branch), and `Perform(aPln1, aPln2, Tol, Tol)` at `:1709`. The recursive
  plane×cone conic classification therefore uses `Tolang = 1e-7`, ~10× looser than the direct
  plane×cone path which gets `1e-8` from `IntPatch_ImpImpIntersection.cxx:2551`. Circle/parabola/
  hyperbola boundaries shift accordingly. `:1710-1714`: `!IsDone() || NbSolutions()==0` →
  `NoGeometricSolution`.
- **cone×cone common generatrix** (`:1760-1884`) requires `A1A2.Intersect()` **and** each apex to lie
  on the other cone (`ElSLib::Parameters` round-trip, `SquareDistance <= Tol²`), then builds the
  plane normal either from `aGen1 ∥ aGen2` (`Precision::Angular()`, `:1803`) or from the generatrix
  intersection; `aGen1.SquareDistance(aGen2) > tol²` → `NoGeometricSolution` (`:1807-1812`).
  Consumers add `PChar` as a vertex with **`SetParameter(0.)` regardless of its true conic
  parameter** (`IntPatch_ImpImpIntersection.cxx:8900,8939,8977`).
- **sph×cone `PointAndCircle`** is decided per root (`:1971-1975`, `:1988-1992`); if both roots
  degenerate, `param1 == param2 == 0` and the accessors (`Point` keys on `param1 == 0.0` `:2686`,
  `Circle` keys on `param2 == 0.0` `:2741`) return a zero-radius circle.
- **cone×torus** (`:2401-2435`): the 2nd circle per side needs `aDist < aRMin` **strict** and
  `aDt > Tol`; up to 4 circles fill `pt1..pt4/param1..4/dir1..4` through the switch at `:2437-2468`.
- **torus×torus**: the `Same` test (`:2612-2617`) precedes the `RMin >= RMaj` validity test
  (`:2619-2623`) — two identical self-intersecting tori still report `Same`.
- **plane×torus**: parallel branch circles carry the **torus** axis as normal (`:2220`), perpendicular
  branch carries the **plane** normal (`:2242`); the latter demands
  `Pln.Distance(torusCentre) <= myEPSILON_DISTANCE (1e-14)` (`:2234`).

### 2.4 Routing / eligibility corrections

- `bToCheck` in `IntPatch_Intersection.cxx:1120-1157` is the *degeneracy* flag. For cones it is
  `(|semiangle| < 0.02) || (> 1.55)`; the whole axis-configuration block that can set `bGeomGeom = 1`
  runs **only if `bToCheck`** (`:1163`), i.e. a normal cone is never re-qualified through it.
  For tori it is `MajorRadius > MinorRadius`, and for torus×torus the second assignment
  **overwrites** the first (`:1152-1156`) — torus1's validity is discarded.
- `ts = bGeomGeom` applies **only** to `GeomAbs_Torus`; plane/cyl/sphere/cone always get `ts = 1`
  (`:1264-1292`). Routing: `ts1==ts2==1 && isGeomInt` → `GeomGeomPerfom`, else `ParamParamPerfom`
  (`:1301-1338`). `isGeomInt` comes from `IntTools_FaceFace::isTreatAnalityc`.
- `isTreatAnalityc` (`src/ModelingAlgorithms/TKBO/IntTools/IntTools_FaceFace.cxx:249-320`) inspects
  **only** plane×cylinder-with-finite-V; everything else returns `true` early (`:271,306`), and a
  non-Ellipse result returns `inter.IsDone()` (`:319`). Needle test `aMajorR < 100000*aMinorR`
  (`:318`).
- `GeomGeomPerfom` fallback is `!interii.IsDone()` where `IsDone() == (GetStatus() != IntStatus_Fail)`
  (`IntPatch_ImpImpIntersection.lxx:21-24`) → `ParamParamPerfom`
  (`IntPatch_Intersection.cxx:1790-1797`). `aNbPointsInALine = 200` (`:1807`); `JoinWLines` only for
  cyl×cyl (`:1845-1848`); `ExtendTwoWLines` only when some ALine was converted (`:1850`).
- Complete list of "analytic gives up" exits: extreme conic
  (`IntAna_QuadQuadGeo.cxx:925-950`), `math_DirectPolynomialRoots` failure in sph×cone (`:1996-1999`),
  `|Qzz| < 1e-8` (`IntAna_IntQuadQuad.cxx:440-444`), `PolDIS/PolZ1/PolZ2/Pol !IsDone`
  (`:462,901,920,994`), cone-path root spacing (`:1079-1083`), `TreatResultTorus` on anything but
  `Empty`/`Circle` (`IntPatch_ImpImpIntersection.cxx:9707-9710`), and the `default: return false`
  arms of `IntPCy/IntPSp/IntPCo/IntCyCo/IntCoSp/IntSpSp` (`:3340,3434,3781,8596,9464,9549`).
- `IntCyCy` also drops to the numeric path when `CyCyAnalyticalIntersect` *returns false on a
  geometric result* — notably `IntAna_Circle` (`:5180-5185`) — clearing `slin/spnt` first
  (`:7924-7931`). `IntAna_Parabola/Hyperbola` from cyl×cyl **throws** (`:5176-5178`).

### 2.5 Frame / parameter conventions

- `Ellipse(1)` uses `gp_Ax2(pt1, dir1, dir2)`, `Ellipse(2)` uses `gp_Ax2(pt2, **dir2, dir1**)` —
  main direction and X direction swap between the two Steinmetz branches
  (`IntAna_QuadQuadGeo.cxx:2791,2804`). `Hyperbola(2)` reverses `dir2` (`:2847`).
- `DirToAx2` picks the reference axis from the **smallest-magnitude** component, with exact-zero
  short-circuits (`:237-257`) — circle phase is arbitrary but *deterministic*; a port that picks
  differently will produce different vertex parameters for identical geometry.
- `AdjustToSeam` re-seats circle X-directions onto the host quadric's seam
  (`IntPatch_ImpImpIntersection.cxx:3863-3930`); called from `IntPCy:3300`, `IntPSp:3413` (takes
  `Tolang`), `IntPCo:3683`, `IntPTo:3827-3830` (**only when the plane axis is not normal to the torus
  axis**), `TreatResultTorus:9680-9683` (**only when both quadrics are the same type**).
- `CyCyAnalyticalIntersect` builds the first ellipse's GLine with **swapped** transitions
  `(trans2, trans1)` because they were sampled at parameter 0 but apply at π (`:5067-5069`). The
  second ellipse's multiple-point parameters are assigned by comparing `ElCLib::Parameter` of the two
  tangency points, and the transition is then sampled at π or 0 accordingly (`:5107-5124`).
- Vertex dedup tolerances are hard-coded and ignore the passed tolerance: `IntPatch_GLine.cxx:421`
  ignores its `Tol` argument and uses `PrecisionPConfusion = 1000*Precision::PConfusion() = 1e-6`
  (`:442`); `IntPatch_ALine.cxx:70` uses `PCONFUSION = 1e-5`.
- `IntAna_Quadric::SetQuadric(gp_Cone)` appends the apex as
  `ElSLib::Value(0, −RefRadius/sin(SemiAngle))` (`IntAna_Quadric.cxx:100-101`); sphere appends both
  poles at `v = ±π/2` (`:110-111`); `mySpecialPoints` is **never cleared**, so re-using an
  `IntAna_Quadric` accumulates. There is **no torus constructor** — the implicit operand of
  `IntAna_IntQuadQuad` can only be plane/cylinder/cone/sphere.
- `AddSpecialPoints` widens `[T1,T2]` by the max out-of-domain delta and then **caps the width at 2π**
  (`IntAna_IntQuadQuad.cxx:109-117`); a point qualifies only if its `ElSLib` round-trip
  `SquareDistance < Precision::SquareConfusion()` (`:83`).
- `ExploreCurve` splits at every `FindParameter` hit whose 3D point is within `10*Tol` of the apex
  (`IntPatch_ImpImpIntersection.cxx:8608-8674`); sub-curves share all coefficients and differ only in
  `SetDomain`.

---

## 3. PORTING TRAPS

1. **Do not model tangency as "Undecided".** `Touch` + `Situation{Inside,Outside}` carries which solid
   is on which side of a tangential contact; `Undecided` carries nothing. Downstream keep/purge for a
   grazing arc (z37) depends on that distinction. Porting both into one "tangent" flag loses the
   inside/outside verdict that OCCT computed from the curvature-centre test, and you will have to
   re-derive it from flux/quorum — which is exactly the residue we are fighting.
2. **`IntSurf_Touch` is only ever produced for the *degenerate-count* branch** (NbSol == 1). If your
   port emits two nearly-coincident lines/circles instead of collapsing to one, you will never enter
   the situation-based branch and will emit two transversal curves where OCCT emits one tangent
   curve. The collapse thresholds are asymmetric (see §2.3 cyl×cyl ladder) and must be copied exactly.
3. **Cone contacts on the negative nappe are purged, silently.** A port that keeps every algebraic
   contact will produce extra vertices in cone-bearing sections. The predicate is the *axis*
   parameter, not a distance (`ElCLib::LineParameter >= paramapex`).
4. **The discriminant snap is one-sided.** `disc < aTolD → 0` means `IntAna_Curve::Value` is total: it
   returns a plausible point everywhere, including well outside the real curve. If your ALine
   evaluator raises/returns-none on negative discriminant, your domains will be shorter than OCCT's;
   if you snap symmetrically (`|disc| < tol`), they will be longer. Neither matches.
5. **`V` is clamped to ±100000 inside `InternalValue`** (`IntAna_Curve.cxx:539-546`) *before* the
   `ElSLib` evaluation, and the cone remap is `(V − RCyl)/sin(Angle)` (`:557`) even though the comment
   says `tan`. Both must be copied verbatim or far branches diverge.
6. **`aTolD`'s `aDA` term is wrong in OCCT** (`Z2CosSin * Z2CosSin`). Copy the bug if you want
   bit-parity on cone ALines; fixing it changes which points snap to the tangency vertex.
7. **θ = π is a special case of the root finder, not of the geometry.** Any reimplementation using a
   direct trig solve (or a different substitution) will produce a *different multiplicity* at π,
   which flips IntQuadQuad between the "double root → 2 single-Z curves over [T1, T1+2π]" branch and
   the "one curve with DEUX_Z_PAR_THETA" branch. Curve count, not just parametrization, changes.
8. **The absolute epsilons in the root finder are unnormalized.** `1.5e-12` on quartic coefficients,
   `1e-8` on the residual, `1e-10` on the null-polynomial sum. Discriminant coefficients scale as
   `R²·(quadric coefficients)²`; a 1000-unit model inflates them by ~10¹², a 0.001-unit model
   deflates by the same. OCCT partially compensates with the `×1e-4` rescaling loop
   (`math_TrigonometricFunctionRoots.cxx:424-431`) — reproduce that loop or normalize by
   `max|coef|` before the tests, and know you are then off-parity.
9. **Cone path aborts, cylinder path skips.** Porting one control-flow shape for both surfaces will
   either lose valid cylinder curves or keep cone curves OCCT would have rerouted to the walker.
10. **`Tolang` is not one number.** `1e-8` at the ImpImp entry (`:2551`), `Tol` (≈1e-7) in the
    cone×cone recursions, `2*sinda` in the plane×cylinder relaxation, `Precision::Angular()` (1e-12)
    for axis parallelism, `Epsilon(radius)` for plane×sphere tangency. Any single global "angular
    tolerance" reproduces none of the boundaries.
11. **The `== 0.0` axis tests are real** (`IntAna_QuadQuadGeo.cxx:1378`, `:1926`) but the surrounding
    `Intersect()`/`Same()` already ran at 1e-14, so the exact test only rejects a centre that is
    off-axis by more than 0 yet the axes still coplanar — in practice it demands the sphere centre be
    *constructed* on the axis. Snap centres onto axes at recognition time; this is confirmed
    load-bearing for the rotated-frame cases.
12. **`RefineDir` will not clean up a rotated frame** (needs an exact ±1 component). Do not rely on it
    as a canonicalizer; do the snapping yourself before calling the case table.
13. **`all1 && all2` can declare tangency without the case table.** If your same-domain routing keys
    only off the analytic `Same` verdict, you will miss the class OCCT catches by "every restriction
    arc of both faces is a solution, and the surface types match".
14. **Apex/vertex parameters are placeholders.** `PChar` vertices are added with parameter 0 and the
    conic vertex fill-in at `IntPatch_ImpImpIntersection.cxx:3000-3052` adds two vertices *both* at
    `ElCLib::Value(0.0, ...)` with parameters 0 and 2π. Re-derive real parameters downstream (OCCT
    does, in `ComputeVertexParameters`) rather than trusting the emitted numbers.
15. **Circle/ellipse phase is synthesized, never stored.** `DirToAx2` fabricates the X direction from
    the normal; two mathematically identical circles produced by different branches (e.g. plane×torus
    parallel vs `AdjustToSeam`-corrected) will have different parameter origins. Never key an exact
    weld on the conic parameter — key on 3D points, as our FINAL PASS 2 rebuild already does.
