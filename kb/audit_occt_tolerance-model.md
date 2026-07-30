# audit_occt_tolerance-model

Audit of `kb/occt_tolerance-model.md` against real OCCT V8 source at `/home/petras/code/code_cpp/OCCT`.
All `file:line` below are **relative to that root**. Line numbers are from the V8 tree (renamed types
`occ::handle`, `NCollection_*`, `bool/int/double`), so they do **not** match the spec's V7 numbers;
every citation here was read, not inferred.

---

## 1. VERDICT

The spec is **structurally faithful and mostly numerically correct**, but it states three doctrine
claims that the source contradicts, and it omits the entire *provisional-growth / rollback* layer,
which is the single most important M4 mechanism to port.

**Material errors**

| # | Spec claim | Reality |
|---|---|---|
| E1 | INVARIANT 1: "tolerance is monotone non-decreasing for the whole operation"; "the only two mutators" are `UpdateVertex`/`UpdateEdgeTolerance` | False. There are **five** hard-set (shrink-capable) writers: `BRepLib::SameParameter` `aNTE->Tolerance(maxdist)` (BRepLib.cxx:1734), `BRepLib::UpdateEdgeTol` `TE->Tolerance(edge_tolerance)` (BRepLib.cxx:685), `UpdShTol` vertex force-branch `aTV->Tolerance(aTol)` (BRepLib.cxx:889-892), `BRep_Builder::UpdateFace` → `TF->Tolerance(Tol)` (ModelingData/TKBRep/BRep/BRep_Builder.cxx:604), and the boolean's own per-FF **rollback** `TV->Tolerance(aTol)` (BOPAlgo_PaveFiller_6.cxx:1084) plus `FilterPavesOnCurves` `TV->Tolerance(aRealTol)` (BOPAlgo_PaveFiller_6.cxx:2531). Only `BRep_Builder::UpdateEdge/UpdateVertex` are max-monotone. |
| E2 | Stage 5 / constants table: E/F criterion = `tolE + fuzz/2 + tolF + fuzz/2` | Only for analytic edges. For **BSpline/Bezier** edges `IntTools_EdgeFace.cxx:532-547`: if `tolE/tolF > 100 \|\| tolF/tolE > 100` → `criteria = max(tolE,tolF)`; else `criteria = 1.5*tolE + tolF` — asymmetric 1.5× weight on the edge. |
| E3 | Stage 4 self-interference: "fuzzy for the pass is **replaced by** `2*max(tolV1,tolV2)`" | It is **added**: `anEdgeEdge.SetFuzzyValue(myFuzzyValue + aTolAdd)` (BOPAlgo_PaveFiller_3.cxx:1217), and only when a tangency gate passes (`\|cos\| >= 0.9063`, :1200). Same for EF: `myFuzzyValue + aTolAdd` (BOPAlgo_PaveFiller_5.cxx:1103) where `aTolAdd` is a *measured* endpoint-to-face distance minus `tolE+tolF` (:1063-1084). |
| E4 | Stage 4: "if the contact is a real crossing (`dist(P1,P2) > Precision::Intersection()` → touching; else)" | Inverted/garbled. `BOPAlgo_PaveFiller_3.cxx:429-434`: `if (aPOnE1.Distance(aPOnE2) > Precision::Intersection()) continue;` — a real intersection requires the two curve points to **coincide within 1e-9**. EF uses a *different* metric: distance from the new point to the **surface** (`aProjPS.LowerDistance() < Precision::Intersection()`, BOPAlgo_PaveFiller_5.cxx:466-478). |
| E5 | Stage 5: EF on-pave growth "grows existing vertex by `aDistPP` **capped by** `min(1e4·tolV, 0.1)`" | Not a cap — a **reject**: `if (aDistPP < aMaxDist) { UpdateVertex(...) }` (BOPAlgo_PaveFiller_5.cxx:494-499). Beyond the limit nothing is grown *and* the interference is dropped (`continue`, :500). |
| E6 | INVARIANT 1: "Geometry is never moved" | One exception: for faces with closed (seam) edges whose seam vertices mismatch, one **face copy is translated** by `gp_Vec(aP1,aP2)` before FF intersection (`BOPAlgo_PaveFiller_6.cxx:470-481`), results transformed back by `aFaceFace.ApplyTrsf()` (:546), and `aTolFF` floored at the shift distance (:495). |
| E7 | Stage 11 / PORT MAP 7: pcurve/same-parameter maintenance described as pure growth | `BOPTools_AlgoTools::MakePCurve` ends with `BRepLib::SameParameter(aE)` (BOPTools_AlgoTools.cxx:1724) whose **default tolerance is 1.0e-5** (BRepLib.hxx:161-162), and which **overwrites** `tolE` with the newly measured `maxdist` (floored only at `Precision::Confusion()`). A section edge that entered with `tolE = aTolR3D = 3e-4` can leave with `tolE = 1e-7`. |

**Material omissions** (each expanded in §2): the per-FF **tolerance rollback** map `aMVTol`;
`FilterPavesOnCurves` (pave rejection + tolerance reduction); `aCoeff = 2.` doubling in
`IsExistingPaveBlock`; the `BRepLib_ValidateEdge` asymmetry between *check* padding and *update*
padding; the `1.5×`/`1.05×`/`1.4×`/`1.01×` family of per-function safety factors in BRepLib
(the spec only recorded `1.00001` and `1+1e-5`); `BRepLib::UpdateTolerances` face-minimum table;
`BoundingVertex`'s exact 2-vertex minimal-enclosing-sphere formula; and the fact that
non-destructive **copy-on-write fires unconditionally**, not only when growth is needed.

Everything else the spec asserts that I checked (fuzzy clamping, half-fuzz box inflation,
`ComputeVV/VE/VF/PE` formulas, `DTolerance()=1e-12`, `ToleranceFF` 5e-6 floor, `IsVertexOnLine`
`2·(tolV+tolC)` with 1e-5/1e-6 floors, `ComputeToleranceOfCB` 11 samples, `ShrunkRange`
splittable rule, micro-block `+dist/2` fuse, `PostTreat(0.05)`, `CorrectWires` 1.01×/0.3,
`CheckEdge` `sqrt(d²)+0.1·tol`, `ComputeIntRange`) is **verbatim correct**.

---

## 2. PORT-CRITICAL DETAILS THE SPEC MISSED

### 2.1 Provisional growth and rollback — the missing M4 layer

The spec models growth as write-once-and-keep. OCCT treats *pave-placement* growth as a **bet that
is unwound if the pave is not consumed**.

- `BOPAlgo_PaveFiller_6.cxx:699` — `NCollection_DataMap<int,double> aMVTol` (per-FF-pair), cleared at
  the top of every FF iteration (:761).
- `BOPAlgo_PaveFiller_6.cxx:3029-3060` — `PutPaveOnCurve` binds the vertex's **pre-growth** tolerance
  into `aMVTol` *before* calling `BRep_Builder().UpdateVertex(aV, aDist + 1e-12)`.
- `BOPAlgo_PaveFiller_6.cxx:2974` — the acceptance test itself reads the **saved** tolerance:
  `double aTolV = (aMVTol.IsBound(nV) ? aMVTol(nV) : BRep_Tool::Tolerance(aV));`
  So growth caused by curve *k* of an FF pair does **not** widen acceptance for curve *k+1*.
  Without this the pipeline self-amplifies (each accepted pave enlarges the next search radius).
- `BOPAlgo_PaveFiller_6.cxx:1047-1048` — when a section edge is actually built from the pave block,
  its two vertices are `aMVTol.UnBind(...)` → growth is **committed**.
- `BOPAlgo_PaveFiller_6.cxx:1073-1095` — end of the FF pair: every vertex still in `aMVTol` is
  **reset**: `TV->Tolerance(aTol)` (hard set, not max), its DS box is **rebuilt from scratch**
  (`aBoxDS = Bnd_Box(); BRepBndLib::Add(aV,aBoxDS); SetGap(+Confusion)` :1088-1091 — a plain
  `BRepBndLib::Add` would only union and never shrink), and its SD group is dropped
  (`aDMVLV.UnBind(nV1)` :1093).
- `UpdateSavedTolerance` (BOPAlgo_PaveFiller_6.cxx:629-645): when an *existing* edge is grown instead
  of building a section edge, the saved tolerances of that edge's vertices are raised too, so the
  rollback cannot undo growth that a committed edge depends on.

Second, weaker reduction: `FilterPavesOnCurves` (BOPAlgo_PaveFiller_6.cxx:2437-2534) runs between
pave placement and block building:
- per (vertex, curve) it records `SquareDist` and `sin` of the angle between `V→C(t)` and `C'(t)`
  (:2463-2469);
- a pave is **removed** iff `SquareDist > 100·max(tolR3D², minDistOverAllCurves)` **and**
  `sinAngle < 0.5` (30°) (:2485-2518) — "do not remove a vertex if it is projected on the curve with
  quite large angle";
- if anything was removed, the vertex tolerance is **reduced** to
  `max(savedTol, sqrt(maxDistKept) + Confusion)` by hard set (:2525-2532).

### 2.2 `BRepLib::SameParameter` — what actually happens to `tolE`

`BRepLib.cxx:1251-1740`, entered from `BOPTools_AlgoTools::MakePCurve` (BOPTools_AlgoTools.cxx:1724)
and `BOPTools_AlgoTools2D_1.cxx:131`.

- **No-op guard**: returns immediately if the edge already carries the `SameParameter` flag
  (:1256-1259). Section edges built by `BRepBuilderAPI_MakeEdge` do carry it in the common case —
  so on many section edges `SameParameter` does nothing at all.
- `NCONTROL = 22` control points (:1294).
- Per (pcurve, surface) representation, and for both pcurves of a closed surface (:1372-1379):
  - `TolSameRange = max(GAC.Resolution(theTolerance), Precision::PConfusion())` (:1378);
  - `error = ComputeTol(HC,HC2d,HS,22)` (:1396) — see §2.3; `error > 1.e10` aborts (:1398-1402);
  - C0 BSpline pcurves get a `Geom2dConvert::C0BSplineToC1BSplineCurve` pass at
    `TolConf2d = max(min(UResolution(tol),VResolution(tol)), PConfusion)` (:1406-1416), with a
    fallback pass at `Tol2dbail = max(min(min(UResbail,VResbail), 0.1·minPoleGap), TolConf2d)`
    (:1447-1469);
  - knot-ratio sanity: if `continuity > C0 && error > max(1e-3, tol)` and any adjacent knot span
    ratio exceeds `critratio = 10`, the pcurve is re-parametrised by arc length via
    `Approx_CurvilinearParameter(max(1e-3,tol), cont, maxdeg, 10)` (:1541-1621);
  - the approximation tolerance handed to `Approx_SameParameter` is
    `aTol = (isANA && isBSP) ? 1.e-7 : theTolerance` (:1628) — a hard 1e-7 in the
    "reparametrisation made it worse, fall back to original" case;
  - `maxdist = max(maxdist, SameP.TolReached())` (:1635, :1655), or `max(maxdist, error)` when the
    approximation reached worse than the sampled error (:1659).
- **OCC5898 rescue** (:1704-1713): if a representation failed, but
  `anEdgeTol + max(BRepCheck::PrecCurve, BRepCheck::PrecSurface) >= error`, the edge is declared
  same-parameter anyway and `maxdist = max(maxdist, anEdgeTol)`.
- **Unconditional** `B.Range(aNE, f3d, l3d); B.SameRange(aNE, true)` (:1719-1720) — the range is
  rewritten even on failure.
- **The write** (:1728-1735), only if at least one pcurve existed:
  `maxdist = max(maxdist, Precision::Confusion()); theNewTol = maxdist; aNTE->Tolerance(maxdist);`
  — a *set*. Comment at :1723-1727: *"Reduce eventually the tolerance of the edge... The same cannot
  be done with vertices"*. Vertices are then raised separately by
  `UpdateVTol` → `BRep_Builder::UpdateVertex` (max) at :1222-1233 / :1245.
- Shape-level `SameParameter` (:913-1008) additionally: in `forced` mode clears `SameRange` and
  `SameParameter` flags first (:931-946); after the edge loop, for **planar faces only**, recomputes
  a per-edge tolerance with `GetEdgeTol` (:973-1002); merges everything through `UpdTolMap`
  (max-only, :799-831) then `UpdShTol(..., theVForceUpdate=false)` (:1005); and finally calls
  `InternalUpdateTolerances(theSh, false, ...)` (:1007) — i.e. **V≥E≥F harmonisation is part of
  `SameParameter(shape)`**, but *not* of `SameParameter(edge)`.
- `GetEdgeTol` (:719-793): 24 samples (`nn = 23`), squared distance zeroed below
  `Epsilon(max(|Pc3d|²,|Pcons|²))`, result `d = 1.05 * sqrt(d2)` (:791).

### 2.3 `ComputeTol` — the sampled deviation used by SameParameter (BRepLib.cxx:1070-1188)

Not mentioned in the spec at all; it is the function that decides whether a pcurve is acceptable.
- `nbp+1 = 23` samples; per-sample out-of-domain penalty for non-periodic directions: if
  `Puv.X() < uf - 0.01·(ul-uf)` the sample is skipped and
  `dapp = max(dapp, (1/UResolution(1))·(uf - Puv.X()))` — a 2D overshoot converted to 3D by the
  inverse resolution (:1083-1121). `dapp > d2` short-circuits the return (:1142-1145).
- Infinite `Pcons` ⇒ `d2 = Precision::Infinite()` and immediate return (:1123-1128, :1136-1139).
- **Outlier suppression** (:1147-1185): distances are bucketed as `<1.0` (N1) and `>=1.0` (N2);
  if `N1 > N2 && N2 != 0` and `N3 = 100·N2/(N1+N2) < 10`, the ≥1.0 samples are treated as spurious
  and only the `<1.0` maximum `D2` is used.
- Final: `d2 = 1.5 * (ana ? sqrt(D2) : d2)`, floored at `1.e-7` (:1185-1186). **A 1.5× safety
  factor**, not 1.00001.
- `EvalTol` (:1034-1066), used only in the C0-BSpline fallback: 5 interior samples at `i/6`,
  succeeds if ≥3 projections converge.

### 2.4 `BRepLib::UpdateTolerances` / `InternalUpdateTolerances` (BRepLib.cxx:1744-1979)

Never called from TKBO (verified: no `BRepLib::UpdateTolerances` in `TKBO/`), but it is the reference
implementation of the containment invariant and is reachable via `SameParameter(shape)`.

- **Face minima** (only when `verifyFaceTolerance`, :1753-1823): base `tol` by surface type —
  Plane/Cylinder/Cone `= Confusion`, Sphere/Torus `= 2·Confusion`, everything else `= 4·Confusion`
  (:1776-1789) — then **scaled by the face bounding-box extent** `tol *= dMax` (:1814) and clamped
  to `0.99` (:1816-1819). This is the only place OCCT makes a minimum tolerance *model-scale
  relative*; the boolean pipeline never does.
- **Edges**: `tolE := max over incident faces` but only recorded `if (tol > BRep_Tool::Tolerance(EK))`
  (:1847-1855). Comment at :1849-1850: "Update can only increase tolerance".
- **Vertices** (:1865-1959): `tol` starts as `max` of incident-edge tolerances; edges with
  `tol > BigTol = 1.e10` (:1859) or `!SameRange` (:1881-1884) are skipped for the geometric part;
  for every curve representation of every incident edge the vertex point is compared against
  `C3D(par)` **and** `S(PC(par))` **and** `S(PC2(par))` for closed surfaces (:1895-1937);
  then `tol = max(tol, sqrt(aMaxDist))` and **`tol += 2.*Epsilon(tol)`** (:1941-1942).
- The write goes through `UpdShTol(..., theVForceUpdate = **true**)` (:1961) → vertices are
  **hard-set** (`aTV->Tolerance(aTol)`, :889-892), i.e. this pass can *shrink* a vertex tolerance;
  edges and faces in the same map go through `UpdateEdge`(max)/`UpdateFace`(set) (:880-886).
- `UpdShTol` also implements the reshaper copy-on-write: when the input is immutable it
  `EmptyCopied()` the shape, re-adds children, and copies the six topology flags
  `Free/Checked/Orientable/Closed/Infinite/Convex` (:860-874) before replacing.

### 2.5 `BRepLib::UpdateInnerTolerances` (BRepLib.cxx:1983-2083)

The "inner point" audit the spec's PORT MAP item 7 wants, and it exists ready-made:
- `NbSamples = SameParameter(edge) ? 23 : 2` (:2028); when not same-parameter, only the two
  representation endpoints are compared (:2037-2040) — parameters are **not** assumed shared.
- Per (3D curve, each curve-on-surface) sample: `aDist += 2.*Epsilon(aDist)` (:2044), edge grown to
  `MaxDist` via `UpdateEdge` (:2065).
- Vertices grown twice: to the *first/last sample of every representation* (:2051-2062) and then to
  `max(dist(V, C(fpar)), TolEdge)` / `max(dist(V, C(lpar)), TolEdge)` (:2067-2081) — the second
  write is what enforces `tolV ≥ tolE` locally.

### 2.6 `BRepLib_ValidateEdge` — check padding ≠ update padding

- `myControlPointsNumber = 22` (BRepLib_ValidateEdge.cxx:30).
- `UpdateTolerance` proposes `dist * 1.00001`, max-merged into the caller's accumulator (:57-64).
- `CheckTolerance(t)` returns `correctTolerance(t) > dist` where
  `correctTolerance(t) = t + max(BRepCheck::PrecCurve, BRepCheck::PrecSurface)` (:42-45, :68-77).
  `PrecCurve` = `RealEpsilon()` except for **ellipses**, where it is `Epsilon(max(|center|, Rmaj, Rmin))`
  (ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck.cxx:70-99); `PrecSurface` likewise except for
  **cones** (`Epsilon(max(|apex coords|, RefRadius))`, :101-129). ULP-scale, but coordinate-scaled —
  at 1e6 mm coordinates it is ~1e-10, not 2e-16.
- Two distinct sampling modes (:113-118): if `SameParameter` **and** both parameter ranges agree
  within `PConfusion`, points are compared **index-to-index** at 23 shared parameters (:121-139).
  Otherwise it is a **two-way projection**: endpoints compared directly, then for each of 21 interior
  indices `Extrema_LocateExtPC` is run *both* reference→other and other→reference, seeded at
  `Resolution(Precision::Confusion())` (:169-227). A single non-converged extremum sets
  `myIsDone = false` and **aborts with a partial distance** (:201-206, :221-226).
- `SetExitIfToleranceExceeded` (:81-85) makes `myCalculatedDistance` a **lower bound only** —
  `GetMaxDistance()` after an early exit is not the true maximum.
- Exact mode (`GeomLib_CheckCurveOnSurface`) only runs when `myIsExactMethod && mySameParameter`
  (:91-98). `CorrectEdgeTolerance` never enables it, so the boolean's final audit uses the
  22-point approximation.

### 2.7 `BRepLib::FindValidRange` — how the "shrunk range" is really computed

`BRepLib_1.cxx:31-258`. Used for micro-block detection (BOPAlgo_PaveFiller_6.cxx:937-946),
closing-pave validation (:3573-3585) and `IntTools_ShrunkRange` (:147).
- Span guard `theParV2 - theParV1 < Precision::PConfusion()` (:184).
- `anEps = max(max(theCurve.Resolution(theTolE)*0.1, Epsilon(maxAbsPar)), Precision::PConfusion())`
  (:201-202) — the bisection resolution; note `Epsilon(par)` makes it **parameter-magnitude scaled**.
- `findNearestValidPoint` (:31-169): march from the end with
  `aStep = theCurve.Resolution(theTol) * 1.01` (:61), clamped to `>= anEps`; for Bezier/BSpline
  (including the basis curve of an offset curve, :72-77) a singularity escape doubles the step while
  `|C'|² < (0.01/Resolution(1))²` (:80-81, :108-137); then bisection to `anEps` (:151-166).
- **Three distinct false returns**: the corresponding end is *not* inside its own tolerance sphere
  (:50-53); the whole range is inside the sphere (:103, :133); the two found parameters overlap
  (`theFirst > theLast`, :251-255) or leave a span `< anEps` (:221-224, :244-247).
- `FindValidRange(edge)` (:262-309) adds `Precision::Confusion()` to each vertex tolerance, and uses
  `tolE` in place of a missing vertex tolerance (:284-297).

### 2.8 `BRepLib::BoundingVertex` — the VV fusion geometry (BRepLib.cxx:3013-3123)

The spec says "barycenter, covering tol". Correct only for `n > 2`.
- `n == 2` (:3025-3072): exact minimal enclosing sphere. With `m` = larger-tolerance vertex,
  `dR = R_m - R_n`, `D = |P_m - P_n|`: if `D <= dR || D < RealEpsilon()` the result is
  **`(P_m, R_m)`** — the bigger sphere swallows the smaller and nothing moves; else
  `R = 0.5·(R_m + R_n + D)`, `C = 0.5·(P_m + P_n - (P_m→P_n)·(dR/D))` (:3063-3064).
- `n > 2` (:3074-3122): points are **sorted lexicographically** (`BRepLib_ComparePoints`, :96-113,
  applied :3090) before summing — issue 0027540, floating-point-order determinism — then
  `C = mean`, `R = max_i(|C - P_i| + tol_i)` (:3102-3118).

### 2.9 Fuzzy plumbing: complete read-site census

Clamped at three independent places (each `max(theFuzz, Precision::Confusion())`):
`BOPAlgo_Options.cxx:107`, `IntTools_EdgeEdge.lxx:163`, `IntTools_FaceFace.cxx:232`.
Defaults `Precision::Confusion()` at `BOPAlgo_Options.cxx:53,65`, `IntTools_EdgeFace.cxx:53`,
`IntTools_FaceFace.cxx:152`, `IntTools_EdgeEdge.lxx:24,48,76`.

| Consumer | Site | Form |
|---|---|---|
| DS box inflation | BOPDS_DS.cxx:312 | `max(fuzz,Confusion)*0.5`; vertex `SetGap(tol+add)` (**set**, :1605), edge/face `SetGap(GetGap()+add)` (:1688, :1775) |
| broad-phase OBB | BOPDS_Iterator.cxx:345-346, BOPDS_IteratorSI.cxx:117-118 | `theCtx->OBB(shape, fuzz)` — OBB inflated by *full* fuzz, AABB by half |
| V/V | BOPAlgo_PaveFiller_1.cxx:93 → BOPTools_AlgoTools.cxx:1783 | `tolV1+tolV2+max(fuzz,Confusion)` |
| V/E | BOPAlgo_PaveFiller_2.cxx:115, _3.cxx:867, _6.cxx:3246,3386 → IntTools_Context.cxx:532 | `tolV+tolE+max(fuzz,Confusion)`; out `theTol = dist + tolE` (:534) written **before** the accept test |
| V/F | _4.cxx:113,374, _5.cxx:641, Builder_2.cxx:198 → IntTools_Context.cxx:574 | `tolV+tolF+max(fuzz,Confusion)`; out `theTol = dist + tolF` |
| P/E | IntTools_Context.cxx:461, :481 | `tolP+tolE+Confusion` (no fuzz inside); fallback branch tests distance to **vertices** with `tolP+tolV+Confusion` |
| E/E | IntTools_EdgeEdge.cxx:149-152 | `tol_i = curveTol_i + fuzz/2`, criterion `myTol = myTol1+myTol2` |
| E/F | IntTools_EdgeFace.cxx:529-547 | `+fuzz/2` each, then the **1.5× BSpline rule** (E2 above) |
| F/F | IntTools_FaceFace.cxx:381-387 | `myTolF_i = tolF_i + fuzz/2`; `TolArc = TolTang = myTolF1+myTolF2` |
| CB coincidence | BOPDS_DS.cxx:1318 | `MaxTolerance(E1,VERTEX)+MaxTolerance(E2,VERTEX)+max(fuzz,Confusion)` **plus** projection parameter strictly inside PB2's range (:1321-1324) |
| new-vertex fusion | BOPAlgo_PaveFiller_3.cxx:711 → BOPAlgo_Tools.cxx:1094,1135 | pair test `tolV1+tolV2+fuzz`; BVH box gap `tol + fuzz/2` |
| existing-edge search | BOPAlgo_PaveFiller_6.cxx:1960, 2032, 2104-2106, 2127 | `tolR3D + fuzz`, `max(tolE,tolV)+fuzz`, `max(tolV1,tolV2)+fuzz` |
| vertex-on-curve | BOPAlgo_PaveFiller_6.cxx:2976, 2982 | radius argument `aTolR3D + myFuzzyValue` |
| forced EE/EF | _3.cxx:1217/1221, _5.cxx:1026/1103 | `fuzz + aTolAdd`; EF pre-gate `LowerDistance() > aTolCheck + fuzz` |
| same-domain faces | Builder_2.cxx:104, CheckerSI.cxx:327 | `AreFacesSameDomain(..., fuzz)` |
| VFI internal-vertex | Builder_2.cxx:977 | `aVFI.SetFuzzyValue(fuzz)` |
| result simplification | BRepAlgoAPI_BuilderAlgo.cxx:185 | `ShapeUpgrade_UnifySameDomain::SetLinearTolerance(fuzz)` — fuzzy doubles as the *unify* tolerance |
| **dead** | BOPAlgo_Builder.cxx:758 | `BOPAlgo_BuilderSolid::SetFuzzyValue(fuzz)` — `BuilderSolid`/`BuilderArea` contain **no read of `myFuzzyValue`**; solid assembly is fuzzy-blind |
| **not propagated** | BOPAlgo_PaveFiller_6.cxx:1196-1197 | `PostTreatFF`'s nested `BOPAlgo_PaveFiller aPF` receives `SetIsPrimary(false)` and `SetNonDestructive(...)` but **no `SetFuzzyValue`** → section-edge self-intersection runs at default `Confusion` regardless of the user's fuzzy |

### 2.10 `IsExistingPaveBlock` (2nd overload) — the rescue path, in full

`BOPAlgo_PaveFiller_6.cxx:2054-2251`. The spec reduces this to "search radius `tolR3D + fuzz`".
- Candidate selection is by BVH over the **first-point box** only, enlarged by `tolV11` (:2065-2081).
- `aTolCheck = theTolR3D + myFuzzyValue` (:2104); `aMaxTolAdd = min(0.001, 10.·aTolCheck)` (:2109-2111).
- `aRealTol` escalates: common block → `max(aTolCheck, max(tolV1,tolV2))` (:2148); common block
  **with a face** → `*= 2.` (:2152); both endpoints shared and non-line/non-line → probe with
  `aTolAdd = 2·min(aMaxTolAdd, max(aRealTol, max(tolV1,tolV2)))` and accept only if the tangents
  agree within `|cos| >= 0.9063` (≈25°), in which case `aRealTol = aTolAdd` **and `aCoeff = 2.`**
  (:2166-2190).
- The reported growth is `theTolNew = aCoeff * aDistToSp` (:2245) — up to **twice** the measured
  deviation. The closest candidate wins (:2242).
- The 1st overload (:1986-2036) is much cruder: midpoint only, `ComputePE(aPm, max(tolE, max(tolV1,tolV2)) + fuzz, ...)`,
  `theTolNew = aDist` with **no delta pad** (:2029).
- Both feed `UpdateEdgeTolerance(nEOut, aTolNew)` (:926, :980) which propagates to *all* the edge's
  vertices (BOPAlgo_PaveFiller_10.cxx:95-100).

### 2.11 Non-destructive copy-on-write fires unconditionally

`BOPAlgo_PaveFiller_10.cxx:105-162`. For an **old** vertex in non-destructive mode there is **no
`aTolV < aTolNew` test before the copy** — a new vertex at the same point with
`max(tolV, aTolNew)` is created, appended to the DS, registered as SD and fenced in
`myVertsToAvoidExtension` (:130-154) even when `aTolNew` is smaller than the current tolerance.
`myIncreasedSS` is updated only if growth was real (:156-159).
`UpdateCommonBlocksWithSDVertices` exploits exactly this: it calls
`UpdateVertex(nV, Precision::Confusion())` (:196, :214-215) purely to *force SD image creation*.
`UpdateEdgeTolerance` (:63-101) refuses early if the edge is an original **or any of its vertices is
an original without an SD image** (:69-85) — the refusal is per-edge, all-or-nothing.
Underneath, `BRep_Builder::UpdateEdge/UpdateVertex/UpdateFace` **throw `TopoDS_LockedShape`** on
locked TShapes (BRep_Builder.cxx:1002-1005, 1446-1449, 600-603); `Locked()` is also what
`SetNonDestructive` detects (BOPAlgo_PaveFiller_10.cxx:51-58).

### 2.12 Assorted exact values the spec rounded or skipped

- `IntTools_ShrunkRange::Perform` (IntTools_ShrunkRange.cxx:107-190): `myLength` is **arc length**
  (`GCPnts_AbscissaPoint::Length`) computed with `aPTolE = min(Resolution(tolE), (T2-T1)/100)`
  (:159-168); micro-edge if `myLength < Precision::Confusion()` (:170-175) or the shrunk parametric
  span `< PConfusion` (:152-156); splittable iff `myLength > 2·tolE + 2·Confusion` (:181-185);
  box gap `tolE + Confusion` (:189).
- `MakeNewVertex(E1,t1,F1)` (BOPTools_AlgoTools_2.cxx:254-270) gives the EF vertex
  **`tolE + tolF + 1e-12`** — additive, *before* the `max(tolVnew, max(tolE,tolF))` at
  BOPAlgo_PaveFiller_5.cxx:508-509. `MakeNewVertex(E1,t1,E2,t2)` (:224-250) gives
  `max(tolE1,tolE2) + 0.5·dist` at the midpoint.
- `PutBoundPaveOnCurve` (BOPAlgo_PaveFiller_6.cxx:2308-2367): closure is tested with a **hard
  `Precision::Confusion()`**, not a tolerance (`aP[1].IsEqual(aP[0], 1e-7)`, :2413-2414); the
  "skip" applies **only** when the curve is closed *and* an end already has a vertex (:2415-2418);
  each new bound vertex must first pass `IsValidPointForFaces(P, F1, F2, aTolR3D)` (:2340).
- `getBoundPaves` (BOPAlgo_PaveFiller_6.cxx:2255-2303): end-vertex matching uses
  `ComputeVV(V, P_end, max(curveTol, tangTol) + Confusion)`.
- `PutClosingPaveOnCurve` (BOPAlgo_PaveFiller_6.cxx:3556-3604): not closed if
  `dist(PV, P_opp) > tolV + (max(curveTol, tangTol) + Confusion)`;
  `aNewTolV = max(tolV, dist + 1e-12)`; **`FindValidRange` is re-run with `aIC.Tolerance()` (not
  `aTolR3D`) and `aNewTolV` on both ends** and the pave is abandoned if no range survives (:3572-3586).
- `ComputeToleranceOfCB` (BOPAlgo_Tools.cxx:248-355): seeded with the *representative original
  edge's* tolerance (:261); early return when `aLPB.Extent() < 2 && aLFI.IsEmpty()` (:266-269);
  step `aDt = (T2-T1)/(11+1)` so the 11 samples are **strictly interior**, endpoints never sampled
  (:277-278); `aTolMax = max(tol_member + LowerDistance)` over edges (:316) and faces (:346).
- `IntTools_FaceFace::ComputeTolReached3d` (IntTools_FaceFace.cxx:613-690): tolerance only ever
  **grows** from `aIC.Tolerance()`; per-pcurve via `IntTools_Tools::ComputeTolerance(..., Precision::PConfusion(), parallel)`
  (:648-661) = `1.00001 * GeomLib_CheckCurveOnSurface::MaxDistance()` (IntTools_Tools.cxx:774-776);
  missing pcurve ⇒ `FindMaxDistance(C3d, f, l, F, ctx)` (:668-673); tangential tolerance is raised to
  `max(myTolF1,myTolF2)` only if currently smaller (:685-689).
- FF curve box: `Enlarge(aTolFF + max(MaxTolerance(F1,VERTEX), MaxTolerance(F2,VERTEX)))`, the vertex
  term added only when the pair produced curves (BOPAlgo_PaveFiller_6.cxx:575-582, :599-606).
- `CorrectEdgeTolerance` (BOPTools_AlgoTools_1.cxx:761-1000): guards are
  `!SameRange && SameParameter` (:784-787), `unique == 0` / `unique > 1` 3D curves (:803-810),
  `myCref.IsNull() && Degenerated` mismatch (:826-829), `Last <= First` (:838-843),
  and per-representation `SameRange && (f != First || l != Last)` (:894-897).
  Crucially `aNewTol` **accumulates across representations** (:915,:928,:987) while the acceptance
  test always compares against the **original** `Tol` captured at :874 — so one bad pcurve raises the
  edge for all of them.
- `UpdateEdges` (BOPTools_AlgoTools_1.cxx:1027-1062): the direct-vertex-under-face branch compares
  `aTolV < aTolE` where `aTolE` carries over from the **last edge processed** in the same loop
  (initialised to `aTolF` at :1033) — order-dependent, then writes `aTolF` anyway.
- `UpdateShape` (BOPTools_AlgoTools_1.cxx:1066-1089) handles **only** EDGE and VERTEX. Faces are
  never touched by the audit; `tolF` is an input-side constant for the whole boolean.
- Degenerate edges: rebuilt at `Precision::Confusion()` (BOPAlgo_PaveFiller_8.cxx:115) or a literal
  `aTol = 1.e-7` (:343, :356).

---

## 3. PORTING TRAPS

1. **Do not implement growth as monotone-only.** A faithful port needs a two-phase tolerance:
   a committed value and a provisional value with a per-FF-pair undo log (§2.1). Making
   `PutPaveOnCurve`-equivalent growth permanent is the classic way to get progressive tolerance
   inflation across many section curves — and it also silently changes *later* acceptance tests,
   because OCCT deliberately reads the **saved** tolerance in the predicate
   (BOPAlgo_PaveFiller_6.cxx:2974), not the grown one.

2. **`tolE` after same-parameter is a set, not a max.** If you port `MakePCurve` → `SameParameter`,
   note the edge tolerance is *replaced* by the measured `maxdist` (floored at 1e-7) and can drop far
   below the section-curve tolerance the edge was built with (BRepLib.cxx:1731-1734). OCCT tolerates
   this only because `PostTreat`'s `CorrectShapeTolerances` later re-raises `tolE ≥ tolF` and
   `tolV ≥ tolE`. Port both halves or neither.

3. **The `1.00001` margin is not the only pad, and the pads are not interchangeable.** Measured
   deviations are inflated by `1.00001` (`BRepLib_ValidateEdge`, `IntTools_Tools::ComputeTolerance`),
   `1.5` (`ComputeTol`, BRepLib.cxx:1185), `1.4` (`UpdateEdgeTol` `safe_factor`, BRepLib.cxx:503,677),
   `1.05` (`GetEdgeTol`, BRepLib.cxx:791), `1.01` (`CorrectWires`, BOPTools_AlgoTools_1.cxx:751),
   `+0.1·tol` (`CheckEdge`, :477,:504), `+1e-12` (`DTolerance`), and `+2·Epsilon(tol)`
   (`InternalUpdateTolerances`, BRepLib.cxx:1942; `UpdateInnerTolerances`, :2044). Using one global
   pad reproduces none of the acceptance boundaries.

4. **`CheckTolerance` pads the threshold; `UpdateTolerance` pads the distance.** Writing
   `if (dist > tol) tol = dist * 1.00001;` is wrong: the real gate is
   `dist >= tol + max(PrecCurve, PrecSurface)` (BRepLib_ValidateEdge.cxx:42-45, :68-77). With
   `SetExitIfToleranceExceeded` in play the returned distance is a lower bound, so *never* feed
   `GetMaxDistance()` from an early-exited run into a tolerance.

5. **E/F is not symmetric.** `1.5*tolE + tolF` for spline edges, with a `100×` ratio escape to
   `max(tolE,tolF)` (IntTools_EdgeFace.cxx:532-547). A symmetric `tolE+tolF+fuzz` port will both
   over-accept (ratio case) and under-accept (spline case) relative to OCCT.

6. **Half-fuzz vs full-fuzz is inconsistent by design and must be copied exactly.** AABB gaps get
   `fuzz/2` per operand; OBBs get the **full** fuzz (BOPDS_Iterator.cxx:345-346); pair narrow-phase
   tests get `fuzz/2` per operand; point tests get `+fuzz` once; the EF force-pass adds `+fuzz`
   *on top of* a sum (BOPAlgo_PaveFiller_5.cxx:1026). Normalising these "for consistency" changes
   which interferences exist.

7. **Fuzzy stops at the intersection phase.** `BuilderSolid` receives a fuzzy value it never reads
   (§2.9), and `PostTreatFF`'s nested PaveFiller never receives one at all
   (BOPAlgo_PaveFiller_6.cxx:1196-1197). A port that threads fuzzy into shelling/classification will
   diverge from OCCT under large fuzzy values.

8. **"Real intersection" has two different definitions.** EE: the two *curve* points must coincide
   within `Precision::Intersection()` (BOPAlgo_PaveFiller_3.cxx:429-434). EF: the new point must lie
   within `Precision::Intersection()` of the *surface* (BOPAlgo_PaveFiller_5.cxx:466-478). Using one
   for both mis-classifies tangential EF contacts as crossings.

9. **The analytic (Line/Circle, Line/Plane) widened tolerance is a *clustering radius*, never a
   stored tolerance.** It travels `aCPB.SetTolerance(aTolVnew)` (BOPAlgo_PaveFiller_3.cxx:523,
   _5.cxx:541) → `aVerts` map (BOPAlgo_PaveFiller_3.cxx:698-701) → `IntersectVertices`, where it only
   raises the BVH box gap (BOPAlgo_Tools.cxx:1105-1108) and the pair predicate (:1076-1091). The
   final fused vertex tolerance comes from `BRepLib::BoundingVertex` over the members' **real**
   tolerances. Writing it onto a vertex reproduces exactly the runaway the comment warns against
   ("do not update the vertex till its intersection with some other shape", _3.cxx:452-453).

10. **`FindValidRange` returning false has three meanings**, and the boolean depends on
    distinguishing them: "end not covered by its vertex" (drop the block as invalid),
    "entire block inside the spheres" (demote to a micro-block and force-fuse the vertices),
    "range degenerate" (drop silently). A single boolean return that always demotes to micro-fuse
    over-merges; one that always drops loses section edges.

11. **`aCoeff = 2.` in the existing-edge rescue** (BOPAlgo_PaveFiller_6.cxx:2245) means the reused
    edge is grown to *twice* the measured deviation when the tangency branch fired. This is the
    mechanism that makes reused trim edges reliably absorb section curves; halving it (the "obvious"
    fix) reintroduces dropped section edges.

12. **`aMaxTol = 0.05` is absolute, not model-relative** (BOPAlgo_Builder.cxx:472), and it is a
    *reject* threshold: `if (aNewTol < aMaxTol) UpdateShape(...)`. A deviation of 0.06 leaves the
    shape invalid rather than tolerant. On a metre-scale model in mm this is 50 µm; on a
    micron-scale model it is effectively unbounded. `BRepLib::UpdateTolerances` is the only place
    OCCT scales a tolerance by model size (`tol *= dMax`, BRepLib.cxx:1814) — copy that idea if the
    port's units differ.

13. **OCCT bug to not replicate blindly**: `IntersectCurves2d` computes
    `aLen2 = MapEdgeLength(*theEData1.Edge, ...)` (BOPTools_AlgoTools_1.cxx:624) — edge 1 twice —
    so the `0.3·min(len1,len2)` vertex-coverage cap is really `0.3·len(E1)`. Matching OCCT bit-for-bit
    requires the bug; matching its *intent* requires `theEData2`.

14. **Box refresh after growth must be a rebuild, not a union, wherever shrinking is possible.**
    Growth paths use `BRepBndLib::Add(shape, box); box.SetGap(box.GetGap()+Confusion)`
    (BOPAlgo_PaveFiller_10.cxx:120-123, :146-148) which only unions; the rollback path must first
    assign a fresh `Bnd_Box()` (BOPAlgo_PaveFiller_6.cxx:1086-1091). Also note `prepareVertices`
    uses `SetGap(tol+add)` (**set**, BOPDS_DS.cxx:1605) while edges/faces use
    `SetGap(GetGap()+add)` (**accumulate**, :1688, :1775).

15. **`BRepLib::SameParameter(edge)` silently no-ops on an edge whose `SameParameter` flag is
    already set** (BRepLib.cxx:1256-1259). If a port marks every constructed edge as
    same-parameter (the natural thing to do for exactly-marched section curves), the whole
    re-approximation and the tolerance recomputation are skipped and `tolE` keeps whatever the
    construction assigned.

16. **`FilterPavesOnCurves`'s obliqueness gate is load-bearing.** A pave is only discarded if it is
    both far (`>100×` the tolerance/min-distance band) **and** obliquely projected (`sin < 0.5`)
    (BOPAlgo_PaveFiller_6.cxx:2485-2518). Dropping the angle term deletes legitimate paves at
    near-perpendicular crossings — the regression OCCT cites as `bugs modalg_6 bug27761`.
