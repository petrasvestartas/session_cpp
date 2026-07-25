# OCCT Extrema Internals — Point-Surface / Curve-Curve / Curve-Surface Extrema

Sources read (OCCT master @ C:/brg/compas_occt/external/occt/src/occt/src):
- `ModelingData/TKGeomBase/Extrema/` — `Extrema_ExtPS.cxx`, `Extrema_GenExtPS.cxx`, `Extrema_FuncPSNorm.cxx`, `Extrema_GenLocateExtPS.cxx`, `Extrema_ExtCS.cxx`, `Extrema_GenExtCS.cxx`, `Extrema_ExtCC.cxx`, `Extrema_GGenExtCC.hxx` (= `Extrema_ECC`), `Extrema_ExtPElS.cxx`, `Extrema_POnSurfParams.hxx`, `Extrema_ElementType.hxx`
- `FoundationClasses/TKMath/math/` — `math_FunctionSetRoot.cxx`, `math_NewtonFunctionSetRoot.hxx`, `math_GlobOptMin.hxx`
- `ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ProjectPointOnSurf.cxx`
- `ModelingAlgorithms/TKBO/IntTools/IntTools_Context.cxx`

NOTE: the Extrema package lives in **ModelingData/TKGeomBase**, not TKGeomAlgo.
Related specs (do not re-read here): `kb/occt_tolerance-model.md` (tolerance ladder),
`kb/occt_ssi-walking.md` (SSI marching), `kb/occt_pavefiller-core.md` (where ProjPS verdicts feed VE/EE/EF interference).

---

## STAGE PIPELINE

### A. Point-Surface (`Extrema_ExtPS` -> `Extrema_GenExtPS` -> `math_FunctionSetRoot`)

1. **`Extrema_ExtPS::Initialize`** (`Extrema_ExtPS.cxx:202`)
   - Clamp infinite UV bounds to `±1e10`.
   - Pick grid density: `nbU = nbV = 44` if BSpline/Bezier, else `32`.
   - **Degenerate-iso probe** `IsoIsDeg` (`Extrema_ExtPS.cxx:32`): walk the 4 border isos with 10 samples; if `max |D1|` along a border iso `<= 1e-9`, that direction is degenerate (cone apex, sphere pole) -> bump that direction's density to **300**.
   - Forwards to `myExtPS.Initialize(S, nbU, nbV, ...)` (the GenExtPS engine).

2. **`Extrema_ExtPS::Perform`** (`Extrema_ExtPS.cxx:269`) — type dispatch:
   - `Cylinder/Plane/Cone/Sphere/Torus` -> **analytic** `Extrema_ExtPElS::Perform(P, quad, Precision::Confusion())` (closed-form feet: 2 for cylinder/sphere, up to 4 for torus; axis-distance degeneracy guarded by `Tol`).
   - `SurfaceOfExtrusion` -> `Extrema_ExtPExtS` (reduce to point-curve on basis curve).
   - `SurfaceOfRevolution` -> `Extrema_ExtPRevS`.
   - default (BSpline/Bezier/offset/other) -> `Extrema_GenExtPS::Perform`.
   - Every raw solution passes **`Extrema_ExtPS::TreatSolution`** (`:97`): `ElCLib::InPeriod` into `[uinf, uinf+period]`, then re-shift by `±period` if beyond `usup+mytolu` / below `uinf-mytolu` (trimmed-surface handling), then accept iff inside `[uinf-tolu, usup+tolu] x [vinf-tolv, vsup+tolv]`.

3. **`Extrema_GenExtPS::BuildGrid`** (`Extrema_GenExtPS.cxx:528`) — seed construction (Grad algo, the default):
   - **Knot-aware params first**: `GetGridPoints` (`:371`) -> `fillParams` (`:321`): for BSpline/Bezier (and revolution/extrusion basis curves), per knot span emit `k = 1..degree` samples with step `span/max(degree,2)`, dedup by `Precision::PConfusion()`; used only if the count `>= requested sample count`, else fall back to uniform.
   - **Uniform fallback**: step `PasU = (range - range/nbU/100)/(nbU-1)`, first sample at `umin + range/nbU/200` — i.e. the grid is **inset from the borders** by half of 1% of a step (never sits exactly on the border/seam).
   - Batch-evaluate all nodes via `GeomGridEval_Surface::EvaluateGrid` (span-cached).
   - Sentinel borders: `myPoints` ring (indices 0 and n+1) gets `SqrDistance = -1` (for max search); `myFacePntParams` ring gets `RealLast()` (for min search) — removes all boundary tests from the scan loops.

4. **Distance field, three levels** (`BuildGrid` steps 1–3):
   - nodes: `sqDist(P, grid_ij)`;
   - edges: **`ComputeEdgeParameters`** (`:460`) — project P onto the chord `P0P1`: `ratio = (P-P0)·(P1-P0)/|P0P1|²`; if `|d0²-d1²| >= |P0P1|² - aDiffTol` the foot is a node, else interpolate UV linearly at `ratio`, evaluate the surface there, tag `Extrema_UIsoEdge/VIsoEdge`. `aDiffTol = mytolu + mytolv`.
   - faces: if both opposite edge pairs say "interior", take the UV midpoint of the 4 edge feet, evaluate, tag `Extrema_Face`; else take the best edge foot.

5. **`Extrema_GenExtPS::Perform`** (`:968`) — seed selection + polish:
   - MIN: scan face cells; a cell launches Newton iff its foot is `Extrema_Face`, OR its foot is a boundary edge/node of the grid, OR it is the **canonical owner** of a shared edge/node foot (compare indices with the down/left neighbor cell — prevents launching Newton twice for the same basin).
   - MAX: node is a seed iff its distance `>=` all 8 neighbors.
   - **`FindSolution`** (`:933`): `math_FunctionSetRoot S(myF, Tol=(tolu,tolv)); S.Perform(myF, UV, UVinf, UVsup)` — bounded quasi-Newton on `F = ((P-S)·Su, (P-S)·Sv) = 0`.
   - Alternative `Extrema_ExtAlgo_Tree` (`BuildTree` `:856`): UB-tree of `Bnd_Sphere` over grid points; BSpline density `max(nbU, UDegree*NbUKnots)` capped at 300; `CorrectNbSamples` (`:793`) fixes anisotropy (iso-length ratio > 10 -> multiply NbV by `int(log(ratio))`); nearest/farthest node by tree selector, ONE Newton polish. Used by `Extrema_ExtAlgo_Tree` callers only; BO uses Grad.

6. **Solution accumulation: `Extrema_FuncPSNorm::GetStateNumber`** (`Extrema_FuncPSNorm.cxx:139`)
   - THE multiple-solution mechanism: every time `math_FunctionSetRoot` finishes (or lands exactly), the function object appends the current `(U,V,P)` to internal `myPoint/mySqDist` **unless** an existing solution is within `PConfusion()² = 1e-18` of it in UV squared distance. All Newton launches share one accumulating store; `NbExt()` is its length. Duplicate basin hits from adjacent seeds are absorbed here.
   - Newton function values (`Value` `:83` / `Values` `:112`): `F1 = PPs·Su, F2 = PPs·Sv`; **full second-order Jacobian** from `D2`: `Df11 = |Su|² + PPs·Suu`, `Df12 = Su·Sv + PPs·Suv`, `Df22 = |Sv|² + PPs·Svv`.

7. **`math_FunctionSetRoot::Perform`** (`math_FunctionSetRoot.cxx:796`) — the polish engine:
   - Start clamped into bounds. Direction = Newton (Gauss/SVD when pivot small); on failure switch to gradient. Step **capped at 1/4 of the domain** per unknown (`InvLengthMax = 1/max((sup-inf)/4, 1e-9)`).
   - Line search: quadratic interpolation then `math_BrentMinimum` fallback (100 iters, tol1d from per-axis `Tol(i)/|Dir(i)|`).
   - Bounds handled by projection + direction recompute when a variable is pinned.
   - Stop: `F2 <= 1e-32`, or gradient norm `<= 1e-64`, or `|Dy| <= 1e-32`, or progress `(F2 - PreviousMinimum) <= Epsilon(F2)`, or `Itermax = 100`.

8. **Local (seeded) variant: `Extrema_GenLocateExtPS::Perform`** (`Extrema_GenLocateExtPS.cxx:122`)
   - Used when a good UV seed exists (walkers, warm starts).
   - `CorrectTol` (`:29`): if `Epsilon(U0) > Epsilon(1)` (huge UV values), escalate the UV tolerance by `10^n`, `n = int(0.43429*ln(eps_u/eps_1)+1)+1` — otherwise FSR cannot terminate.
   - Order: `math_FunctionSetRoot` first; if not done (or tol was escalated) retry `math_NewtonFunctionSetRoot(F, aTol, Precision::Confusion())` (pure Newton, 100 iters). Distance-criteria mode: `math_BFGS` on `Extrema_FuncPSDist` (rel tol `1e-8`), fallback `math_FRPR`.
   - **`IsMinDist`** (`:63`): 3x3 stencil verification that a converged root is a true local MIN — steps `du = max(UResolution(10*Confusion), 10*PConfusion)` = max(model 1e-6 mapped to UV, 1e-8). Callers use this to reject saddle/max roots.

### B. Curve-Surface (`Extrema_ExtCS` -> `Extrema_GenExtCS` (PSO) -> FSR)

1. **`Extrema_ExtCS::Perform`** (`Extrema_ExtCS.cxx:109`): seeds `NbT=12, NbU=NbV=10`; `13` if periodic (odd count so seeds straddle the seam) or circle/BSpline curve.
2. Analytic pairs (line x plane/cyl/sphere; circle x plane/cyl/sphere; hyperbola x plane) -> `Extrema_ExtElCS`. If analytic extrema all fall outside trims: **fallback projects the two curve endpoints** by `ElSLib::Parameters` and keeps the nearer (or both when equal within `Confusion`).
3. Line x general surface: clamp the line parameter range to the surface `Bnd_Box` (8 corners projected on the line via `ElCLib::Parameter`); if the clamped range collapses (`<= Confusion`), reduce to a single `Extrema_ExtPS` MIN call at the midpoint.
4. General: `Extrema_GenExtCS` (`Extrema_GenExtCS.cxx`) — grid inset by `range/1e4` (`aBorderDivisor`), then **particle-swarm global search** (`math_PSO`, `aNbParticles = 48`) over `(t,u,v)` (3-var), conic-restricted 2-var, or quadric-restricted 1-var functionals (`Extrema_GlobOptFuncCS/ConicS/CQuadric`), then `math_FunctionSetRoot` polish on `Extrema_FuncExtCS`. Hyperbola t clamped to `[-20, 20]` (overflow guard).
5. **Sharp points**: after the generic pass, every interior `C1`-discontinuity parameter of the curve is projected with a cached `Extrema_ExtPS` and the best foot appended; then of these extra solutions only the global min and max survive (`Perform` tail `:452-526`).
6. **`AddSolution`** (`:576`): period-normalize T/U/V, accept within `±tolC/±tolS` of the ranges, dedup by `|T-Tj|<=tolC && |U-Uj|<=tolS && |V-Vj|<=tolS`.

### C. Curve-Curve (`Extrema_ExtCC` -> `Extrema_ECC` = `Extrema_GGenExtCC` -> `math_GlobOptMin`)

1. **`Extrema_ExtCC::Perform`** (`Extrema_ExtCC.cxx:177`): compute the 4 endpoint-pair squared distances first (`mydist11..22`, exposed via `TrimmedSquareDistances` — BO uses these as cheap bounds). Dispatch: line x (line..parabola) and circle x circle -> analytic `Extrema_ExtElC`; everything else -> `myECC.Perform()`.
2. **`Extrema_GGenExtCC::Perform`** (`Extrema_GGenExtCC.hxx:453`) — global search:
   - Interval decomposition at `C2` continuity; if `nInt1*nInt2 > 100` degrade to `C1`. Closed curve with 1 interval -> force 3. Length-ratio balancing: if one curve's per-interval arc length exceeds the other's by `mult = 20x`, subdivide it (`ChangeIntervals`, target `<= 100` cells).
   - Lipschitz constant for `math_GlobOptMin`: start `aLC = 100`, clamp by `maxDer = max(1/Resolution(1)) * sqrt(2)`, hard max `1e4`; refined by a **21x21 gradient sampling** of the functional; various lock conditions (line operands, ratio < 0.001).
   - `math_GlobOptMin aFinder(func, low, upp, aLC)`; `SetTol(aDiscTol = 1e-2, aSameTol = myCurveMinTol/1e-2)`; `SetFunctionalMinimalValue(0)`; run **per interval-product cell** via `SetLocalParams`.
   - Keep only cells whose min is within `aSameTol * aValueTol` (`1e-2`) of the running best; dedup solutions with an `NCollection_CellFilter` of cell size `max(range) * PConfusion/(2*sqrt2)`.
3. **Parallel-case detection** (`:707-791`): sort solutions lexicographically; parallel iff (a) functional at midpoints of consecutive solutions equals best F within `Precision::Confusion`, (b) solution parameters advance monotonically in the same direction on both curves, (c) endpoint cross-projections (`Extrema_GGenExtCC_ProjPOnC`) reproduce the same distance. Three independent confirmations.
4. **`Extrema_ExtCC::PrepareParallelResult`** (`:397`): line-line -> project ends of C1 on C2, intersect ranges: overlap `> Confusion` = truly parallel (1 sqdist, no points); touching at a point = midpoint solution pair; disjoint = min of the 4 endpoint pairs. circle-circle -> project the arc, 3 iterations of `Bnd_Range` intersection with `PI` shifts, precision `max(Epsilon(R1), Epsilon(R2))`.
5. **`PrepareResults`** (`:832,:905`): `InPeriod` normalize, accept within `[Ut - RealEpsilon, Ut + RealEpsilon]` of the trimmed ranges (NO tolerance slack here, unlike PS/CS!).

### D. BO usage (`GeomAPI_ProjectPointOnSurf` + `IntTools_Context`)

1. **`GeomAPI_ProjectPointOnSurf::Init`** (`GeomAPI_ProjectPointOnSurf.cxx:105-173`): wraps one `Extrema_ExtPS` with `TolU = TolV = Tolerance` (default `Precision::Confusion()`; bounds-variant default `PConfusion()`); `Init()` (`:81`) scans all extrema and stores the index of the **lowest square distance — first hit wins ties** (deterministic only if enumeration order is reproduced).
2. **`IntTools_Context::ProjPS`** (`IntTools_Context.cxx:247`): ONE projector **cached per face**; initialized with the **face's restriction UV bounds** (`UVBounds(aF)`, not the surface bounds), `myPOnSTolerance = 1e-12`, and `Extrema_ExtFlag_MIN` (skips the max search entirely).
3. **Verdict functions** — distance and containment are separate questions:
   - `ComputeVE` (`:499`): project vertex point; not done -> `-1`; `LowerDistance() > aTolV + aTolF + max(fuzz, Confusion)` -> `-2`; `(U,V)` classified OUT by `FClass2d` -> `-3`; else 0.
   - `IsValidPointForFace` (`:647`): `LowerDistance() > aTol` -> false; else `IsPointInOnFace` (`state != TopAbs_OUT` — **ON counts as in**).
   - `IsPointInFace(gp_Pnt)` (`:612`): same but strict (`!OUT && !ON`).
   - `SetPOnSTolerance` (`:1006`) clears the projector cache (tolerance is baked in at Init).

---

## DATA STRUCTURES

- **`Extrema_POnSurf`**: `(U, V, gp_Pnt)`.
- **`Extrema_POnSurfParams`** (`Extrema_POnSurfParams.hxx`): extends POnSurf with `mySqrDistance`, `myElementType` (`Extrema_Node | Extrema_UIsoEdge | Extrema_VIsoEdge | Extrema_Face`), grid indices `myIndexU/myIndexV`. The element type + indices are what make seed dedup possible.
- **Grid arrays** (GenExtPS): `myPoints(0..nbU+1, 0..nbV+1)` nodes with sentinel ring; `myUEdgePntParams(1..nbU-1, 1..nbV)`; `myVEdgePntParams(1..nbU, 1..nbV-1)`; `myFacePntParams(0..nbU, 0..nbV)` with `RealLast` ring. `myUParams/myVParams`: the (possibly knot-aware) parameter arrays.
- **`Extrema_FuncPSNorm`**: function object = solution store (`NCollection_Sequence` of POnSurf + sqdist). Cleared on `SetPoint`/`Initialize`, appended by `GetStateNumber`.
- **UB-tree of `Bnd_Sphere`** (Tree algo): sphere per grid point carrying `(NoU, NoV)` indices, min/max selectors shrink the search radius as better nodes are found.
- **`NCollection_CellFilter` + `PointsInspector`** (GGenExtCC): 2D UV cell hash for O(1) solution dedup.
- **`IntTools_Context` maps**: `myProjPSMap: TopoDS_Face -> GeomAPI_ProjectPointOnSurf*` (placement-new from an incremental allocator), same pattern for ProjPC (edge), ProjPT (curve), FClass2d, SurfaceAdaptor, Hatcher, SolidClassifier.

---

## CONSTANTS & TOLERANCES

Precision base values (`TKernel/Precision/Precision.hxx`): `Confusion = 1e-7`, `PConfusion = 1e-9`, `Intersection = 1e-9`, `Angular = 1e-12`, `SquareConfusion = 1e-14`, `Infinite = 2e100`.

| Constant | Value | Where |
|---|---|---|
| PS grid density, analytic-ish surfaces | 32 x 32 | `ExtPS::Initialize` |
| PS grid density, BSpline/Bezier | 44 x 44 | `ExtPS::Initialize` |
| PS grid density, degenerate border iso | 300 (that direction) | `ExtPS::Initialize` |
| degenerate-iso test | `max|D1| <= 1e-9` over 10 samples | `IsoIsDeg` |
| infinite-bound clamp | `±1e10` | `ExtPS::Initialize` |
| analytic quadric foot tolerance | `Confusion = 1e-7` | `ExtPS::Perform` -> `ExtPElS` |
| grid border inset | half of `range/nb/100` (0.5% of a step) | `BuildGrid`, `BuildTree` |
| knot-aware samples per span | `max(degree, 2)` intervals, dedup `PConfusion` | `fillParams` |
| Tree BSpline density | `UDegree*NbUKnots`, cap 300 | `BuildTree` |
| anisotropy fix | iso-length ratio > 10 -> `NbV *= int(log(ratio))`; probe with `min(23, Nb)` pts; min iso length `1e-3` | `CorrectNbSamples` |
| grid element classification tol | `aDiffTol = tolU + tolV` (on SQUARED dists) | `BuildGrid`/`ComputeEdgeParameters` |
| FSR internal | `EpsSqrt 1e-16, Eps 1e-32, Eps2 1e-64, Progres 0.005`, Itermax 100, step cap = range/4, progress stop `Epsilon(F2)` | `math_FunctionSetRoot::Perform` |
| PS solution dedup | UV sq-dist `<= PConfusion² = 1e-18` | `FuncPSNorm::GetStateNumber` |
| range acceptance PS | `± mytolu/mytolv` (caller tol) | `ExtPS::TreatSolution` |
| GenLocateExtPS tol escalation | `10^n`, `n = int(0.43429*ln(epsU/eps1)+1)+1`, base `PConfusion` | `CorrectTol` |
| GenLocateExtPS BFGS rel tol | `1e-8`; Newton fallback residual `Confusion` | `GenLocateExtPS::Perform` |
| IsMinDist stencil | `du = max(UResolution(1e-6), 1e-8)` | `GenLocateExtPS::IsMinDist` |
| CS seeds | T=12 (13 circle/BSpline), U=V=10 (13 periodic) | `ExtCS::Perform` |
| CS hyperbola clamp | `t in [-20, 20]` | `ExtCS::Perform` |
| GenExtCS border inset divisor | `1e4`; PSO particles 48 | `Extrema_GenExtCS.cxx` |
| CS accept/dedup | `±tolC`, `±tolS`; dedup same tols | `ExtCS::AddSolution` |
| CC interval cap | `nInt1*nInt2 <= 100` (C2 -> C1); closed min 3; length-ratio `mult = 20` | `GGenExtCC::Perform` |
| CC Lipschitz | default 100, `maxDer = sqrt2/Resolution(1)`, cap 1e4; 21x21 gradient probe | `GGenExtCC::Perform` |
| CC GlobOptMin tols | `aDiscTol = 1e-2`, `aValueTol = 1e-2`, `aSameTol = tol/1e-2` (default `1e-7`) | `GGenExtCC::Perform` |
| CC dedup cell | `max(range)*PConfusion/(2*sqrt2)`, floor `PConfusion` | `GGenExtCC::Perform` |
| CC parallel value test | `Precision::Confusion`; circle precision `max(Epsilon(R1),Epsilon(R2))` | `GGenExtCC` / `PrepareParallelResult` |
| CC range acceptance | `± RealEpsilon()` (no slack!) | `ExtCC::PrepareResults` |
| GeomAPI default projector tol | `Confusion 1e-7` (full), `PConfusion 1e-9` (bounded ctor) | `GeomAPI_ProjectPointOnSurf::Init` |
| BO projector tol | `myPOnSTolerance = 1e-12`, flag MIN | `IntTools_Context::ProjPS` |
| BO distance acceptance | `tolV + tolF + max(fuzz, 1e-7)` | `IntTools_Context::ComputeVE` |

---

## INVARIANTS

1. **Sampling hypothesis** (documented at `Extrema_GenExtPS.cxx:173`): if the surface has N extrema w.r.t. P, the grid has N extrema too — grid density must resolve every basin. Everything else (knot-aware seeds, degenerate-iso 300, anisotropy fix) exists to keep this hypothesis true.
2. **All Newton launches share one accumulating solution store** (FuncPSNorm); dedup happens in UV (not 3D), so the result set is exhaustive over local minima AND maxima with no duplicates — a seed escaping into an already-found basin is harmless.
3. **Seeds never sit on the domain border**; border extrema are reached by the *clamped* Newton (bounds projection inside FSR), not by border samples. Corner distances are additionally exposed raw via `TrimmedSquareDistances`.
4. Newton is always **bounded** (clamp each iteration) and **step-capped** (quarter domain) — it can pin variables to a bound and continue in the reduced space.
5. **Analytic surfaces never touch the grid path** — quadric feet are closed-form with `Confusion` degeneracy guards; parity with OCCT on quadrics requires the analytic dispatch, not a denser grid.
6. **Period normalization before range filtering, with tolerance slack, then clamp** (`TreatSolution`): a foot at `usup + 0.5*tolu` is accepted (kept as-is, not snapped). CC is the exception: `RealEpsilon` only.
7. **Distance verdict and containment verdict are independent**: the projector answers "distance to the carrier within face UV bounds"; whether the foot is *in the face* is answered separately by the 2D classifier on `(U,V)`. Acceptance distance (`tolV+tolF+fuzz`) is decoupled from projector precision (`1e-12`).
8. Parallel curve-curve status requires three independent confirmations (equal-F midpoints + monotone direction coincidence + endpoint cross-projection); a parallel verdict *removes* point solutions and returns only the distance.
9. Projector objects are **cached per topological entity** and reused across thousands of queries (BO); cache key includes the restriction bounds, so face and surface projections are distinct.

---

## PITFALLS

- **Gauss-Newton vs full Newton**: `FuncPSNorm::Values` includes the second-order terms `PPs·Suu / Suv / Svv` in the Jacobian. Omitting them (pure first-order) stalls near high-curvature feet and converges to tangential offsets — exactly the `newton_cc` stall class we hit in the UV-arrangement cut-node snap (commit 4544770f).
- **Uniform seeding on BSplines misses inner-knot basins** — a C0/C1 knot can hide a closer foot between two uniform samples; OCCT refuses uniform grids whenever knot-aware sampling produces enough points.
- **Degenerate isos silently break the sampling hypothesis**: near a cone apex `|D1|→0`, so 32 samples cluster in 3D and whole basins vanish. The 1e-9/300 bump is load-bearing, not cosmetic.
- **UV-space dedup at 1e-9 can merge distinct 3D feet** on closed surfaces when two minima wrap to nearly equal UV *before* `InPeriod` normalization (order: dedup in FuncPSNorm first, period-normalize in TreatSolution second). OCCT tolerates this; a port that normalizes first changes the solution count.
- **FSR converges to any stationary point** (min, max, saddle) — the equations are `grad = 0`. When a MIN is required, verify with the `IsMinDist` 3x3 stencil or use the MIN flag path (which only affects *seed selection*, not the polish).
- **Huge UV domains break fixed tolerances**: at `U ~ 1e6`, `Epsilon(U) ≈ 1.2e-10 > PConfusion`; without `CorrectTol` escalation FSR's stop test can never fire. Imported STEP surfaces regularly have such domains.
- **Line vs surface without bbox clamping** seeds a 12-point grid over `±1e10` — useless. Clamp the curve range to the surface AABB first.
- **Parallel/overlap CC must be split into overlap / touch / disjoint**: returning "one distance" for disjoint collinear trims loses the endpoint minimum; returning points for a true overlap invents arbitrary parameters.
- **Tie-break by enumeration order**: `GeomAPI...::Init` keeps the FIRST of equal-distance feet. Any parity harness must reproduce the seed scan order, or compare feet as sets.
- **Face-vs-surface bounds**: BO projects inside the face's restriction UV box. Projecting on the full closed surface can wrap to the other side of a cylinder and produce a different (valid!) foot with the same distance — classification then diverges.
- `SetAlgo` invalidates the grid (`myInit = false`) but `SetFlag` does not; grids are reused across `Perform` calls with different query points (only distances are recomputed, geometry evaluation is cached).

---

## PORT MAP

Our anchors: `Closest::surface_point` (`session_cpp/src/closest.cpp:248`), `Closest::curve_point` (`:17`), `Closest::curve_curve` (`:108`), consumers `brep_section.cpp build_section_scaffold` (SSI chains/paves at `:452,:1560-1579,:1933`), `brep.cpp split_with` UV lifting (`:5588-5689`), `combine`/NK-RESCUE (`:8195-8223`), radial classification (`:10291-10309`), warm-window inverter `invert_near` (`closest.cpp:427`).

| OCCT mechanism | Our anchor | Action |
|---|---|---|
| Full-D2 Newton Jacobian (`FuncPSNorm::Values`: `|Su|²+PPs·Suu` etc.) | `surface_point` Newton loop `closest.cpp:311-349` (currently first-order only: `duu = Su·Su`) | **REPLACE**: add the three second-derivative dot terms from `surface.evaluate(u,v,2)`; fixes tangential-stall feet that forced the cut-node crossing snap. |
| Knot-aware seeding (`fillParams`: `max(degree,2)` intervals per knot span) | `surface_point` seed grid `closest.cpp:277-303` (order-based uniform, window-scaled) | **ADOPT**: derive seeds from `get_span_vector` per window (spans intersecting `[u0,u1]` x degree samples), keep the 3-seed floor for warm windows. |
| Degenerate-iso densify (`IsoIsDeg` -> 300) | `surface_point` full-domain calls on cones/spheres of imported chairs | **ADOPT** (cheap probe: `|D1|<=1e-9` on 10 border samples -> raise that direction's seed count); apex/pole feet currently rely on Newton luck. |
| Multi-solution accumulation (`GetStateNumber` UV-dedup store) | single-best return breaks tube merge + radial classification when two feet are equidistant (periodic surfaces) | **NEW-BUILD** `surface_points_all` variant: run Newton from every locally-minimal grid cell, dedup at `1e-9` UV, return sorted by distance; keep `surface_point` = first entry. |
| Analytic quadric dispatch (`ExtPElS`, tol `1e-7`) | we already recognize plane/cyl/cone/sphere/torus (`recognize_solid` kinds 0-4) but still grid-project on them | **ADOPT**: closed-form feet for recognized kinds inside `surface_point` front-end; exact UV for the radial classifier and scaffold paves. |
| `TreatSolution` accept-within-`±tol`-then-report semantics | `split_with` UV lifting clamps raw (`:5682-5689`), losing feet just past a trim | **ADOPT**: accept `±tol` outside the window before clamping; aligns with whole-seg alias key tol 1e-2 rationale. |
| Per-face cached projector, MIN flag (`IntTools_Context::ProjPS`) | `build_section_scaffold` re-seeds a full grid per pave refinement call; `invert_near` warm-window is a partial equivalent | **ADOPT**: one persistent projector state per (surface, restriction-window) during a boolean run; carries the seed grid + span cache across the thousands of `Closest::surface_point` calls in `brep.cpp:1381-1491` flood tests. |
| CC global search (interval products + GlobOptMin, Lipschitz-guided) | `curve_curve` `:108` — single dense grid + one Newton, single solution | **KEEP for now** (our only CC consumer `brep.cpp:6653` wants min distance of section chains); **NEW-BUILD only if** shared-chain run lifting starts needing *all* CC extrema — then port the interval-product outer loop, not PSO. |
| CC endpoint distances (`TrimmedSquareDistances`) as cheap pre-verdict | valence-1 bridge decisions in `build_section_scaffold` | **ADOPT**: compare 4 endpoint pairs before running any iteration; matches OCCT's early-out and our bridge's "nearest open end" heuristic. |
| `CorrectTol` epsilon escalation for large UV | `surface_point` fixed `step_tolerance = range*1e-10` | **ADOPT**: floor the stop tolerance at `10*Epsilon(max|u|)`; imported STEP chairs have mm-scale models but 1e2-1e4 UV domains after transform. |
| `IsMinDist` 3x3 stencil min-verification | NK-RESCUE / keep-verdict feet (`brep.cpp:8195`) where a saddle foot poisons the nearest-surface vote | **ADOPT** as a post-check on suspicious feet (distance within tol of a rival surface). |
| Distance/containment decoupling (`ComputeVE` codes -1/-2/-3) | our keep-verdict conflates "close to carrier" and "inside face" in one tolerance | **REFERENCE** `kb/occt_tolerance-model.md`; expose the split verdict from `surface_point` consumers: distance test at `tolV+tolF+fuzz`, then UV classification separately. |

---

## Cross-references
- `kb/occt_tolerance-model.md` — where `1e-12` projector precision vs `tolV+tolF+fuzz` acceptance fits the global ladder.
- `kb/occt_ssi-walking.md` — walkers consume `Extrema_GenLocateExtPS`-style seeded polish; the warm-window `invert_near` is our equivalent.
- `kb/occt_pavefiller-core.md` — `ComputeVE`/`IsValidPointForFaces` verdict codes feed the VE/EF interference tables described there.
