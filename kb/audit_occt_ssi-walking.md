# audit — occt_ssi-walking.md vs real OCCT source

Audited against `/home/petras/code/code_cpp/OCCT` (V8 layout, clang-formatted). All line numbers below are
**verified in that tree**; paths are relative to the OCCT root. Focus: `PutToBoundary` minimization internals,
step control, invocation conditions, boundary stall, tangent-zone detection, marching aborts.

Primary files:
- `src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_PWalking.cxx` (4096 lines) — hereafter **PW**
- `src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_PWalking.hxx` — **PWh**
- `src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PrmPrmIntersection.cxx` — **PP**
- `src/ModelingAlgorithms/TKGeomAlgo/IntImp/{IntImp_Int2S.gxx, IntImp_ZerParFunc.gxx, IntImp_ZerParFunc.lxx, IntImp_ComputeTangence.cxx}`

---

## 1. VERDICT

The spec is **structurally faithful and unusually accurate on line numbers** (they match this tree within a few
lines) — the stage decomposition, the constant table, and the PORT MAP are usable as-is. It is, however,
**wrong or dangerously incomplete on five points that hit session A's next port directly**:

1. **ERROR — `DistanceMinimizeByGradient` is a no-op for non-NURBS surfaces.** PW:2413-2422 returns `true`
   (success!) without touching the point when either surface is not `GeomAbs_BezierSurface`/`GeomAbs_BSplineSurface`.
   For plane/cyl/cone/sphere/torus/revolution/extrusion/offset operands the gradient stage never runs and
   reports success. The spec presents gradient descent as the workhorse of `SeekPointOnBoundary`. It is not:
   for analytic pairs the entire boundary landing is done by `HandleSingleSingularPoint` (the locked-iso Int2S
   corrector), and `SeekAdditionalPoints` degenerates to "insert the parametric midpoint iff it happens to be
   within 2e-7" (usually false → no densification at all).
2. **ERROR — order of operations in `SeekPointOnBoundary`.** `HandleSingleSingularPoint` is not a refinement
   *of* the minimized point: `aSingularPnt` is a copy of the **pristine input** taken before the loop (PW:2750),
   it is always attempted (PW:2779) and, on success, **overwrites** the minimization result (PW:2782). The
   minimizer output is used only when the singular-point path fails.
3. **ERROR — "walk clamp box".** Spec Stage 1 says the per-section UV min/max box is "passed to the walker" and
   OCCT "walks in a shrunk box, then extends". False: `Perform(ParDep,u1min..v2max)` uses the box **only** in
   `ComputePasInit(u1max-u1min, …)` (PW:783) to size the initial steps. The marching domain is `Um1..VM2` from
   the constructor (natural bounds, periodically widened) and is never narrowed. Porting the "shrunk box" idea
   would produce spurious boundary framings.
4. **OMISSION — two constructors with different domain policy.** PW:219 (6-arg, the one every PrmPrm call site
   uses: PP:395, 955, 1509, 2010, 2219, 2571, 3228) has the **non-periodic KELARG widening commented out**
   (PW:337, 353, 369, 385) and caps the resolution rescale at `NEWRESO < 10` (PW:272). PW:418 (10-arg, with
   seed params; used by `ChFi3d_Builder_0.cxx:4083`) **does** widen non-periodic bounds by `KELARG*pasuv`
   (PW:517-518, 534-535, 551-552, 568-569), has no `<10` cap (PW:469), and applies the `Reso ≤ 1e-5*pasuv`
   clamp *after* `pasInit/pasSav` are frozen (PW:584-604). The spec describes a blend of the two.
5. **OMISSION — `PutToBoundary` can shrink the line below usability.** The hairpin cleanup deletes end points;
   the grid caller re-checks `if (PW.NbPoints() < 3) continue;` right after (PP:2770-2772). The LOfPnts caller
   (PP:2058) does not, and both ignore the return value.

Minor spec inaccuracies: transitions use `>= 0.` not `> 0` (PP:2103); `IsTangentExtCheck` uses 4 probes per
surface (8 total, spec says "8 probe points" — correct, but they are *local extrema*, not evaluations);
`SeuildPointLigne = 15*Increment²` confirmed (PP:397).

---

## 2. PORT-CRITICAL DETAILS THE SPEC MISSED

### 2.1 `PutToBoundary` (PW:2951-3155)

- **Band**: `aTol = min(1.0, ΔU1, ΔU2, ΔV1, ΔV2) * 1e-3` (PW:2967-2971 — the `*1e-3` binds to the whole chained
  min, so the band is capped at 1e-3 absolute regardless of chart size). Bail-out `aTol <= 2*Precision::Confusion()`
  (PW:2973) means charts whose smallest range is ≤ 2e-4 are **never** snapped.
- **Snap test is strict on both ends and lower-bound-first**: `(aTolMin < aDelta) && (aDelta < aTol)` with
  `aTolMin = Precision::Confusion()` (PW:2954, 2991). The upper bound is only tried in the `else` branch
  (PW:2996-3003) — a param cannot be snapped to both bounds, and a param already *on* a bound (δ ≤ 1e-7) is
  deliberately skipped.
- **Parallelism gating is cross-mapped**: `isV1parallel` (small **U** swing, PW:184) gates the **u1** snap
  (PW:2988); `isU1parallel` (small V swing, PW:185) gates **v1** (PW:3026). Same for surface 2 (PW:3007, 3045).
  Reading the flag names naively inverts the guard.
- **Degenerate `IsParallel`**: `aNbPointsMAX = 23` (PW:124); **`NbPoints < 3` returns with both flags `true`**
  (PW:134-139) → the whole snap is disabled for very short lines. Sampling index is `RealToInt(aNPoint)`
  (truncation) with `aStep = NbPoints / min(NbPoints,23)` (PW:141-156).
- **Flags are computed once, before any insertion** (PW:2981-2982) and reused for the last-point pass even
  though the first pass may have deleted points.
- **Only the last call's status is returned** (PW:3149-3154): a successful first-point insert followed by an
  untriggered last-point pass returns `false`. Both call sites ignore it anyway.
- **Bounds used are the surfaces' natural bounds** (`FirstUParameter`…, PW:2958-2965), *not* the walker's
  widened `Um1..VM2`. On a periodic operand the marched end may legitimately sit outside `[First,Last]`; then
  `aDelta` is negative and no snap occurs.

### 2.2 `SeekPointOnBoundary` (PW:2716-2947)

- Tolerance: `a3DTol = max over both surfaces of max(PConfusion/UResolution(1.0), PConfusion/VResolution(1.0))`,
  `aTol = max(Precision::Confusion(), a3DTol)` (PW:2737-2742). `PConfusion() = Confusion()*0.01 = 1e-9`
  (`Precision.hxx:334`). Effect: 1e-7 for normal-sized geometry, growing linearly for large charts.
- Loop: `aNbIter = 20`, decremented once per round (PW:2752-2776). Each round runs gradient → extrema(S1) →
  extrema(S2). **Break only when a stage returned success AND `AdjustToDomain` reported no change**
  (PW:2758, 2765, 2772). If the stage succeeded but the point had to be clamped, the loop continues; the
  `while (!aStatus && aNbIter>0)` condition then still exits if the *third* stage succeeded.
- `AdjustToDomain` (PW:193-215) clamps with a `Precision::PConfusion()` dead band and reports whether it moved
  anything; it is applied to 4 params after gradient, to params 1-2 after extrema(S1), 3-4 after extrema(S2).
- **Acceptance** (PW:2790-2798): `aPInt = 0.5*(P1+P2)`, accept iff `|aPInt-P1|² ≤ aTol²` ⇒ `|P1-P2| ≤ 2*aTol`.
  The stored point is the **midpoint with the two (unequal) parameter pairs** (PW:2801-2802) — exactly our
  `(p3, uvA, uvB)` triple with a residual gap baked in.
- **Hairpin cleanup, exact algorithm** (PW:2829-2885 first / 2886-2944 last):
  1. scan from the end for the first point `aP1` with `|aP1-aPInt|² > SquareConfusion`;
  2. continue for the next point `aP2` with `|aP2-aP1|² > SquareConfusion`;
  3. if no such pair exists (`aPInd > aNbPnts`, PW:2861) → **return false, insert nothing**;
  4. accept if `(aP1-aPInt)·(aP2-aP1) > 0`, else **`RemoveAPoint(1)`** — it deletes the *end* point, not the
     scanned index (PW:2874/2932) — and repeat while `NbPoints > 1`;
  5. finally delete the end point if it coincides with the insertion (PW:2877-2881/2936-2940), then
     `InsertBefore(1,·)` / `Add(·)`.
  The commented-out blocks at PW:2843-2849/2901-2907 are the older "forbid insertion" behaviour — do not
  resurrect them.
- `RemoveAPoint` (PWh:156-172) decrements `myTangentIdx` when the removed index ≤ it; **`InsertBefore` does
  not increment it**, so after a first-point insertion the tangent index used for transition computation
  (PP:2099-2107) refers to the neighbouring point. Harmless in OCCT (tgdir is stored separately) but a porter
  who recomputes the tangent from the index will get the wrong point.

### 2.3 `DistanceMinimizeByGradient` (PW:2394-2522)

- `aNbIterMAX = 60`, `aTol = 1e-14`, `aTolNul = 1/Precision::Infinite()` (PW:2399-2401).
- **Type gate** PW:2413-2422 (see VERDICT 1).
- Gradient of ½|P1-P2|²: `aGradFu = -P12·D1u`, `aGradFv = -P12·D1v`, `aGradFU = +P12·D2U`, `aGradFV = +P12·D2V`
  with `P12 = P2-P1` (PW:2434-2439).
- Update `p -= copysign(max(|grad*step|, aMinAddVal), grad)` (PW:2456-2466) with
  `aMinAddVal = max(Epsilon(param), aTolNul)` (PW:2406-2409). **The minimum add is applied even when the
  gradient is ~0**, so the point always drifts by ≥ 1 ulp per iteration.
- **`aNbIter` is decremented only on non-improving iterations** (PW:2491). Improving iterations are unlimited
  and multiply all four steps by 1.2 (PW:2484-2487); on failure the gradient is recomputed at the current best
  and the steps reset to 1e-6 (PW:2497-2516). So this is "60 line-search failures", not "60 iterations".
- Returns `aSQDistPrev < 1e-14` **evaluated only inside the improvement branch** (PW:2483): if the very first
  trial step fails 60 times, the return is `false` even if the input was already a perfect intersection point.
- `theStep0` is never supplied by any OCCT caller (both call sites pass the default `nullptr`).

### 2.4 `DistanceMinimizeByExtrema` (PW:2532-2583)

- 2-var Newton on ∂/∂u,∂/∂v of |S(u,v)-P0|², `aTol = 1e-14`, `aNbIter = 10` decremented only after an
  improving step (PW:2563), loop breaks the moment the distance stops decreasing (PW:2555-2558).
- **The Hessian off-diagonal is wrong on purpose/by accident**: `aDf1v = aD2Su.Dot(aD1Sv)` (PW:2574) where the
  exact term is `D2uv·vec + D1u·D1v`; `aDf2u` is then set equal to it. Only `aDf1u`/`aDf2v` are exact
  (`D2u·vec + D1u·D1u`). `aD2SuvTemp` is fetched from `D2` and never used. **Port this literally** — "fixing"
  it changes the iteration path and therefore which points get inserted.
- Step signs: `aU -= step0[0]*(aDf2v*aF1 - aDf1v*aF2)/aDet`, `aV += step0[1]*(aDf2u*aF1 - aDf1u*aF2)/aDet`
  (PW:2578-2579) — consistent with Cramer's rule for the above (asymmetric-looking but correct).
- No determinant guard: `aDet == 0` at a degenerate point yields inf/NaN and the subsequent `aSQDist >= aSQDistPrev`
  comparison is false → the loop exits with the last good `(theU0,theV0)`. NaN-safety is accidental; a port with
  a different comparison order can hang or propagate NaN.
- `theU0/theV0` are only updated on improvement, so the function never worsens its input.

### 2.5 `HandleSingleSingularPoint` (PW:2587-2712) — the real boundary lander

- Builds a **fresh** `IntWalk_TheInt2S(theASurf1, theASurf2, the3DTol)` and `math_FunctionSetRoot`
  (PW:2607-2608): the corrector tolerance here is the metric-scaled `aTol` from `SeekPointOnBoundary`, **not**
  the walker's `myTolTang`.
- For each of the 4 params whose value is within `Precision::PConfusion()` of either natural bound (PW:2612-2613)
  it re-runs the corrector with **that** iso locked (`aLockedDir[]` = U1,V1,U2,V2 in enum order, PW:2601-2604).
- After convergence, per surface and per direction: `aTolU = the3DTol/|Du|`; if the converged param is within
  `aTolU` of a bound **and** `|S(bound,v) - S(u,v)|² < the3DTol²`, the param is written exactly to the bound
  (PW:2642-2684). This is the metric-aware snap that lands on poles/seams.
- Success requires **all four** params inside `[low-PConf, upp+PConf]` (PW:2694-2702); returns on the first
  param that yields such a point. `thePnt` is mutated even on failure paths.

### 2.6 `SeekAdditionalPoints` (PW:3159-3295)

- Early `return true` when `NbPoints > theMinNbPoints` (**strictly** greater, PW:3165). Callers pre-check
  `PW.NbPoints() < 40` (PP:2774-2778, PP:3067-3071).
- Outer loop repeats until the count reaches the target or a full pass inserts nothing (PW:3185).
- The seed is the **componentwise parametric midpoint of all four params**, each clamped to its surface's
  natural bounds (PW:3197-3235).
- Inner solve: up to 5 rounds, **break at the first stage that returns success** (PW:3237-3264) — different
  from `SeekPointOnBoundary`, which additionally requires "no domain adjustment". **No `AdjustToDomain` at all**
  here, so the extrema stages may return a point outside the domain and it is accepted.
- Acceptance: both `|aPInt-P1|² < 1e-14` and `|aPInt-P2|² < 1e-14` (PW:3271-3273) ⇒ `|P1-P2| < 2e-7`. On
  rejection `lp--` so the pair index does not advance (PW:3288).
- **Consequence for the caller**: `wline->EnablePurging(!hasBeenAdded)` (PP:2882, PP:3143) — a line that was
  densified is exempted from `ComputePurgedWLine` forever. Port this coupling or the thinner will undo the
  densification.

### 2.7 Invocation topology (when minimization runs vs marching)

- `PutToBoundary` is called from exactly two places, both immediately after `PW.Perform` and only when
  `PW.IsDone() && PW.NbPoints() > 2`: PP:2058 (pre-seeded `LOfPnts` variant, `Perform` at PP:1827) and
  PP:2767 (IntPolyh grid variant, `Perform` at PP:2488). The polyhedron-interference variant (PP:369), the
  self-intersection variant (PP:932) and PP:2176 **never** call it. No other OCCT subsystem calls it
  (`ImpPrm`, `ImpImp`, `IWalking` have no equivalent).
- `SeekAdditionalPoints` is called only in the grid variant, twice: after `PutToBoundary` on the polyhedral
  seed branch (PP:2777) and on the tangent-zone / second-pass seed branch (PP:3070) where `PutToBoundary` is
  **not** called.
- Tangent-zone seeds are actually tried: PP:3009-3020 walks from `Interference.GetTangentZonePoint(...)`.
  They are rejected inside `Perform` by `IsTangentExtCheck` (PW:858-861), not before.

### 2.8 Marching side — details that change results

- **Early `return` = whole line discarded.** `Perform` sets `done = true` only at PW:1820. Every early `return`
  (first-point not done/empty/tangent PW:812-826; `IsTangentExtCheck` PW:860; `IncKey == 5000` PW:1388, 1500,
  1622) leaves `done == false`, so `PW.IsDone()` is false and the caller drops **all** collected points.
- **Corrector**: `IntImp_Int2S::Perform(Param,Rsnld,ChoixIso)` freezes one param and solves 3 unknowns;
  acceptance is `|P1-P2|² ≤ TolTangency²` (`IntImp_Int2S.gxx:99-100`, `tol` set at `:34`), and the emitted point
  is the **midpoint** `(pntsol1+pntsol2)/2` (`IntImp_ZerParFunc.lxx:27-30`). The three free params are
  box-constrained by `math_FunctionSetRoot` to the **natural** surface bounds `ua0..vb1`
  (`IntImp_ZerParFunc.gxx:248-311`), default 100 iterations (`math_FunctionSetRoot.hxx:55-56`). Therefore the
  KELARG widening only lets the *locked* param and PWalking's own accept tests leave the natural box.
- **Tangency criterion** (`IntImp_ComputeTangence.cxx:70-124`), i.e. what "graze" means to this kernel:
  any `|D|² ≤ 1e-32` → tangent; either normal with `|N|² < 1e-32` → tangent; else tangent iff all four
  `|Tgduv[i]| ≤ EpsUV[i]*|D|` with cross-paired norms (`Tgduv[0]` vs `Eps_u*|Dv1|`, `Tgduv[1]` vs `Eps_v*|Du1|`,
  …); else tangent iff `|N1·N2| > 0.999999999` (≈1.5e-5 rad). Iso ranking = ascending sort of
  `|Tgduv[1]|/|Du1|, |Tgduv[0]|/|Dv1|, |Tgduv[3]|/|Du2|, |Tgduv[2]|/|Dv2|` in parallel with
  `staticChoixRef[] = {U1,V1,U2,V2}` (`IntImp_ComputeTangence.cxx:19-24, 133-150`).
- `IsTangentExtCheck` (PW:687-745): degenerate normal (`|N|² < RealSmall()`) ⇒ **returns tangent** (PW:705-708);
  cos² gate `aDP² < 0.9998*|N1|²*|N2|²` ⇒ not tangent (PW:710); probes are `±pasuv[i]` offsets evaluated by
  `Extrema_GenLocateExtPS` (local extremum from the current uv, PW:665-679) against `4*myTolTang²`.
- `TestDeflection` details the spec flattens:
  - inflexion → halve all steps, `STATIC_PRECEDENT_INFLEXION += 3`, and if the halved steps are all below Reso
    return `ArretSurPointPrecedent` else `PasTropGrand` (PW:3461-3473); the **next three calls return
    `IntWalk_OK` immediately** (PW:3477-3482), skipping every other test.
  - `PointConfondu` rescue: `pasuv[i] = max(pasuv[i], min(1.5*pasuv[i], pasInit[i]))` (PW:3499); the OCC26717
    local-resolution branch runs **only if `|pasuv[choixIso]-pasInit[choixIso]| ≤ Confusion`** (PW:3502) and sets
    **both** `pasuv` and `pasInit` to `2*LocalResol` (PW:3543). `choixIso` is used as an array index — the enum
    order is U1,V1,U2,V2 (`IntImp_ConstIsoparametric.hxx`).
  - no-progress → steps are **reset to Reso** before returning `ArretSurPointPrecedent` (PW:3571-3575).
  - first 2D gate (vs previous tangents) halves the steps, and if still above Reso **halves them again**
    (quarter total) before `PasTropGrand` (PW:3629-3644); the second gate (vs current tangents + `Ang > AngRef`)
    halves once and returns `ArretSurPoint` — not `…Precedent` — when below Reso (PW:3665-3672).
  - the two deflection branches **return early**: `Flèche ≤ fleche/2` grows steps (Ratio capped by
    `min(pasInit[i]/pasuv[i])`, PW:3741-3764) and returns `PasTropGrand` once
    `STATIC_BLOCAGE_SUR_PAS_TROP_GRAND > 5` (PW:3767-3771), else returns the current status (PW:3778);
    `Flèche > fleche` scales and returns `PasTropGrand` (PW:3782-3791). **The osculating-circle test only runs
    in the `fleche/2 < Flèche ≤ fleche` band**, with `Ratio = 0.75*fleche/Flèche` (PW:3796).
  - osculating gate: `anInvSqAbsArcDeflMax = 0.25*d²/tolconf²`, `aSinB2Max = 1-2/(1+that)`,
    `PasTropGrand` iff `aSinB2Max ≥ 0 && cos(T1,T2) ≤ 2*aSinB2Max²-1` (PW:3862-3868); the `StepTooSmall` test
    uses `4*anInvSqAbsArcDeflMax` and is suppressed when the previous status was `PasTropGrand` (PW:3871-3878).
    `StepTooSmall` raises `pasuv` **and `pasInit`** to the achieved |Δ| (PW:3892-3900).
- `TestArret` (PW:3918-4096): out-of-bounds is tested on **both** the predicted `Param(i)` and the corrected
  `SolParam[i]` (PW:3971-3972, 3990-3992); on framing **all four** params are overwritten by `ParC`, where
  in-range entries hold the **predicted** value (PW:4014-4015, 4031) — the corrected point is discarded;
  `ChoixIso = ChoixRef(argmax Duv)` with the fallback ladder when every `Duv == -1` (PW:4042-4059);
  `close = false` is forced on any framing (PW:4061); closure detection runs only when `!DejaReparti` (PW:4066).
- `RepartirOuDiviser` (PW:3297-3397): on reversal the steps are re-estimated from the **last chord of the
  reversed line** (PW:3331-3339 / 3374-3382), not reset to `pasInit`; `tglast = true` is set when the step
  underflows and the previous point was not tangent (PW:3347-3350).
- `ExtendLineInCommonZone` (PW:1831-2385): the entry guard compares against the **natural** bounds with
  one-sided Reso margins (PW:1861-1893). `PasTropGrand` inside the extension shrinks `pasInit` by 10 % of the
  `pasInit-pasSav` gap **and un-counts the iteration** (`nbIterWithoutAppend--`, PW:2011-2019). The zig-zag
  filter uses `piquota = π/4` and skips sub-steps shorter than `gp::Resolution()` (PW:2306-2365). It is invoked
  twice per stall — before and after `RepartirOuDiviser` (PW:1781-1789, 1793-1802).

---

## 3. PORTING TRAPS

1. **"Minimize to the boundary" is only true for NURBS×NURBS.** With any analytic operand the sequence is:
   clamp the offending param → `HandleSingleSingularPoint` (locked-iso Newton with a metric tolerance) →
   midpoint acceptance. Implement the corrector path first; the gradient/extrema pair is the NURBS fallback.
   A port that always runs gradient descent will behave differently from OCCT on every cylinder/plane case.
2. **The inserted boundary point is not an exact intersection point.** It is the midpoint of two surface points
   up to `2*aTol` apart, carrying two independent uv pairs. Downstream code must tolerate `|S1(uv1)-S2(uv2)| > 0`
   at chain ends; asserting exactness there will reject valid OCCT-equivalent output.
3. **`HandleSingleSingularPoint` wins over the minimizer.** If you run them in the naive order (minimize, then
   snap), grazing ends land in a different place than OCCT's.
4. **Iteration counters are failure counters.** `aNbIterMAX = 60` (gradient) and `aNbIter = 10` (extrema) are
   decremented only on non-improving / improving steps respectively (PW:2491, 2563). Treating them as plain
   loop bounds changes both cost and outcome.
5. **`IsParallel` flag names are inverted relative to the axis they gate**, and it declares short lines (<3
   points) parallel in both directions, disabling the whole snap (PW:134-139, 184-185).
6. **`PutToBoundary` deletes points.** Re-check the chain length (and any index-based vertex/tangent bookkeeping)
   after it; OCCT itself only re-checks at one of its two call sites (PP:2770).
7. **Don't fix the Hessian in `DistanceMinimizeByExtrema`** (PW:2574) and don't add a determinant guard that
   changes the exit condition — both alter which midpoints get accepted.
8. **The corrector's free variables are hard-clamped to the natural surface box**, so periodic widening
   (`KELARG = 20`) only affects the locked parameter and PWalking's own accept tests
   (`IntImp_ZerParFunc.gxx:248-311` vs PW:335-397). A port that widens the box *inside* the Newton solve will
   walk off the chart.
9. **Which constructor you copy matters**: the PrmPrm path has non-periodic widening disabled and a `<10` cap
   on the resolution rescale (PW:272, 337); the fillet path does not (PW:469, 517).
10. **Early returns discard the line.** Mirror the `done` flag semantics or a stalled walk will silently return
    a partial chain that OCCT would have thrown away (PW:812-826, 860, 1388).
11. **Densification and purging are coupled**: `EnablePurging(!hasBeenAdded)` (PP:2882, 3143).
12. **`SeekAdditionalPoints` has no domain clamp inside the solve**, only on the initial midpoint; its
    acceptance (`|P1-P2| < 2e-7`) is absolute, not metric-scaled — on large-coordinate models it essentially
    never fires. If our chains are in model units of 10³+, port the acceptance as a scaled tolerance or accept
    that the stage is a no-op (which is what OCCT does in practice there).
13. **Tangency is a property of the corrector, not of the caller.** Reuse the exact
    `IntImp_ComputeTangence` predicate (four cross-paired component tests + `|cos| > 0.999999999`) for
    "graze entry"; a normals-only test will trigger `ExtendLineInCommonZone` in the wrong places.
14. **`ExtendLineInCommonZone` appends its buffer even when it returns `false`** (PW:2373-2384 — the return
    value is `bOutOfTangentZone`, not "committed"). Treating `false` as "nothing happened" double-counts or
    drops points.
