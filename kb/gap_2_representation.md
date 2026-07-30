# Gap 2 — THE REPRESENTATION LAYER: curves, pcurves, tolerances, same-parameter

**What must be implemented in our kernel to produce correct boolean results, in this layer,
ranked by measured impact.**

Everything below is either (a) read out of `validation/occt_trace/traces/`, (b) measured by a
probe I wrote and ran against the same OCCT 8.0.0.rc2 static install the tracer uses, or (c) cited
to `file:line`. Where a claim is geometric reasoning rather than measurement it says so.

**Instruments used (reproducible, both outside the repo, nothing under `src/` touched):**

* `validation/occt_trace/traces/*.trace` — 38 existing dumps, read only.
* `<scratchpad>/pcprobe.cpp` — new. Two modes.
  `--proj` drives `ProjLib_ProjectedCurve` directly on a canonical pullback battery and reports
  the result *type, degree, pole count, rationality, reached tolerance* and the measured
  `max|C3d(t) − S(C2d(t))|`.
  `--bool` runs six real `BRepAlgoAPI` booleans and dumps, for every (face, edge) pair of the
  result, the 3D curve type, the pcurve type/degree/poles/rationality, `BRep_Tool::Tolerance`,
  the `SameParameter`/`SameRange` flags, both parameter ranges, and the same-parameter deviation
  at 401 samples.
  Build: `g++ -std=c++17 -O2 pcprobe.cpp -isystem <occt>/my_include -Wl,--start-group <24 .a>
  -Wl,--end-group -lpthread -ldl`.
* `<scratchpad>/sag.py` — closed-form error scaling of a degree-1 vs a cubic pcurve for the
  sphere-side pullback of the `sph×cyl@45°` section. No OCCT involved; pure geometry.

---

## 1. WHAT OCCT DOES

### 1.1 The stored curve-on-surface is a TYPED object, and 87 % of the time it is EXACT

Probe `--bool`, 6 booleans (`sph−cyl@45`, `sph−cyl@0`, `sph−box`, `boxR30−cyl`, `cone−cone`,
`tor∩box`), **82 (face, edge) pcurves**:

| stored 2D type | count | share |
|---|---:|---:|
| `Geom2d_Line` | 60 | 73 % |
| `Geom2d_Circle` | 8 | 10 % |
| `Geom2d_Ellipse` | 3 | 4 % |
| `Geom2d_BSplineCurve` | 12 | 15 % |

Cross-tabulated against the 3D curve:

| 3D curve | 2D pcurve | count | measured `max‖C3d(t) − S(C2d(t))‖` |
|---|---|---:|---|
| Line | Line | 35 | 0 … 7.0e-16 |
| Circle | Line | 25 | 0 … 1.4e-15 |
| Circle | Circle | 8 | 2.5e-16 … 3.1e-16 |
| Ellipse | Ellipse | 3 | 4.7e-16 … 9.2e-16 |
| Circle | BSpline | 8 | 8.85e-08 … 1.29e-07 |
| Ellipse | BSpline | 3 | 3.32e-09 … 7.75e-08 |
| Hyperbola | BSpline | 1 | 1.33e-08 |

**71 of 82 pcurves (87 %) are exact to machine epsilon.** The 12 BSpline ones are *all* degree 8,
*all* non-rational, 9–72 poles.

`SameParameter=1` and `SameRange=1` on **82 of 82**, with no exception, and in every record
`range2 == range3` bit-for-bit. There is no such thing in OCCT's boolean output as a pcurve on a
different parameterisation from its 3D curve.

The same picture in the 3D curves of the trace corpus (`RESEDGE curve=` over 38 traces):
157 `Line`, 128 `Circle`, 32 `Degenerated`, 28 `BSpline`, 14 `Ellipse` — and in the section
curves themselves (`SEC tag=final geom=` over 142 records): 77 `Geom_TrimmedCurve(Circle)`,
29 `Geom_TrimmedCurve(Line)`, 14 `Geom_TrimmedCurve(Ellipse)`, 22 `Geom_BSplineCurve`.
**84 % of OCCT's section curves are exact analytic conics, not approximations.**

### 1.2 The exact-pullback dispatcher — the complete enumeration

`ProjLib_ProjectedCurve::Perform` (`ModelingData/TKGeomBase/ProjLib/ProjLib_ProjectedCurve.cxx:369`)
switches on **surface type** (`:389–452`) into a per-surface projector, and each projector
switches on **curve type** (`Project`, `:242–270`). Every projector sets `isDone=false` unless it
recognises an exact case; `:717` then routes everything not done to approximation.

Measured with `--proj` (`tolreached` is `ProjLib_ProjectedCurve::GetTolerance()`):

```
PROJ plane x circle-in-plane      type=Circle   tolreached=1e-07 devsameparam=0
PROJ cyl   x circle-perp-axis     type=Line     tolreached=1e-07 devsameparam=0
PROJ cyl   x generator-line       type=Line     tolreached=1e-07 devsameparam=0
PROJ sph   x latitude-circle      type=Line     tolreached=1e-07 devsameparam=0
PROJ sph   x meridian-great-circle type=Line    tolreached=1e-07 devsameparam=1.27e-15
PROJ cyl(tilted) x same-circle    type=Line     tolreached=1e-07 devsameparam=4.97e-16
PROJ cone  x base-circle          type=Line     tolreached=1e-07 devsameparam=0
PROJ torus x parallel-circle      type=Line     tolreached=1e-07 devsameparam=9.80e-16
PROJ cyl   x ellipse-45deg-cut    type=BSpline deg=8 poles=23 knots=4 rat=0 tol=9.27e-08 dev=1.34e-07
PROJ sph   x tilted-circle-45     type=BSpline deg=8 poles=65 knots=10 rat=0 tol=9.50e-08 dev=1.33e-07
PROJ sph   x tilted-circle-20     type=BSpline deg=8 poles=100 knots=15 rat=0 tol=9.93e-08 dev=9.26e-08
```

The exact set, with source:

| surface | curve | condition | result | source |
|---|---|---|---|---|
| Plane | Line | always | `gp_Lin2d` | `ProjLib_Plane.cxx:101` |
| Plane | Circle | always | `gp_Circ2d` | `ProjLib_Plane.cxx:110` |
| Plane | Ellipse | always | `gp_Elips2d` | `ProjLib_Plane.cxx:127` |
| Plane | Parabola / Hyperbola | always | `gp_Parab2d` / `gp_Hypr2d` | `ProjLib_Plane.cxx:143,158` |
| Plane | Bezier / BSpline | always | project the **poles**, keep knots, mults, degree **and weights** | `ProjLib_ComputeApprox.cxx:1197–1252` |
| Cylinder | Line | `dir ∥ axis` (`Precision::Angular`) | `gp_Lin2d` u=const | `ProjLib_Cylinder.cxx:95–118` |
| Cylinder | Circle | circle normal ∥ axis | `gp_Lin2d` v=const | `ProjLib_Cylinder.cxx:122–156` |
| Cylinder | Ellipse | **never** — deliberately delegated | — | `ProjLib_Cylinder.cxx:161–165` |
| Cone | Line | line ∥ a v-isoline (a generator) | `gp_Lin2d` u=const | `ProjLib_Cone.cxx:69–104` |
| Cone | Circle | circle axis ∥ cone axis | `gp_Lin2d` v=const | `ProjLib_Cone.cxx:108–165` |
| Sphere | Circle, iso-U | circle axis ⟂ sphere axis **and** centres coincide (a meridian great circle) | `gp_Lin2d` | `ProjLib_Sphere.cxx:118,124–161` |
| Sphere | Circle, iso-V | circle X and Y both ⟂ sphere axis (a latitude circle) | `gp_Lin2d` | `ProjLib_Sphere.cxx:119,162–177` |
| Sphere | anything else | **never** | — | `ProjLib_Sphere.cxx:181–199` → base class, not done |
| Torus | Circle | iso-V branch (`|OC|<Confusion` or circle axis ∥ torus axis), else iso-U | `gp_Lin2d` | `ProjLib_Torus.cxx:81–173` |

Two things fall out of this table that matter to us:

* **The pullback is exact exactly when it is an ISO-LINE or an affine image.** On a plane the map
  UV→3D is affine, so *every* curve pulls back exactly, weights and all. On a quadric of
  revolution only iso-U / iso-V lines are exact. There is no third case.
* `ProjLib_Sphere::Project` handles **only** `gp_Circ`; `Project(gp_Lin/gp_Elips/gp_Parab/gp_Hypr)`
  forward to the base class and never set `isDone`. A tilted circle on a sphere — the section of
  a sphere with any cylinder whose axis is not the sphere axis — is *not* a NURBS at any degree:
  the pullback is `v(u) = arcsin(a·cos(u − u₀) + b)`. It has to be approximated.

**Where OCCT is wrong here (cite this, do not copy it):** `ProjLib_Torus::Project(const gp_Circ&)`
(`ProjLib_Torus.cxx:81`) sets `isDone = true` unconditionally at `:172` — there is **no validity
check** on the circle. Measured: a circle of radius 1 centred at (3,0,0) with axis (0.3,0.2,1) on
a torus R=3 r=1 returns `type=Line` with `devsameparam = 1.27`. That pcurve is wrong by 1.27 model
units and OCCT will happily store it. Our torus recogniser must gate on the two iso conditions and
fall through otherwise.

### 1.3 The transcendental fallback, and exactly what it costs

`ProjLib_ProjectedCurve.cxx:717–786`: when the analytic projector is not done and the surface is
analytic, it runs `ProjLib_ComputeApprox`. Parameters (`ProjLib_ComputeApprox.cxx:1285–1329`):

* degree band **[5, 8]** when both surface and curve are analytic (`simplecase`), **[5, 10]**
  otherwise (`:1285–1293`).
* 3D tolerance `myTolerance`, set by the caller; in the BOP path it is
  `Precision::Confusion() = 1e-7` (`ProjLib_ProjectedCurve.hxx:58–59`;
  `BOPTools_AlgoTools2D.cxx:525`).
* 2D tolerance derived from the surface metric:
  `aTol2d = max(√(TolU² + TolV²), Precision::PConfusion())`, `TolU/TolV = ComputeTolU/V(S, tol3d)`
  (`:1318–1321`).
* `MaxSegments = 1000` (`:1304`), continuity `AppParCurves_TangencyPoint` (C1) (`:1309`).
* `Approx_FitAndDivide2d Fit(Deg1, Deg2, myTolerance, aTol2d, true, …)` (`:1323`).
* The reached tolerance is rescaled back to 3D: `myTolerance *= (aNewTol2d / aTol2d)` (`:1415`).

Result: **degree 8, non-rational, adaptive knot count.** Measured pole counts scale with how hard
the pullback is, at fixed 1e-7:

| case | poles | knots | reached tol | measured dev |
|---|---:|---:|---|---|
| ellipse on cylinder (45° cut) | 23 | 4 | 9.27e-08 | 1.34e-07 |
| tilted circle on sphere, 45° | 65 | 10 | 9.50e-08 | 1.33e-07 |
| tilted circle on sphere, 20° | 100 | 15 | 9.93e-08 | 9.26e-08 |

The 20° case needs 100 poles where 45° needs 65: the closer the section circle passes to the
sphere pole, the higher the UV curvature and the more knots the fit takes. This is the same
regime in which the traces show OCCT's own volume answer degrading (23° tilt: OCCT off by 3.2e-3,
`kb/occt_trace_findings.md` Q1).

`ProjLib_ComputeApprox` can also **fail silently**: driven on a curve that does not lie on the
surface it returned `poles=7002 knots=1001 tolreached=0.0633 dev=1.39` — it hit `MaxSegments=1000`
and returned a curve two decades outside tolerance, still `IsDone`. Any port must check the
reached tolerance, not the done flag.

### 1.4 Where a pcurve comes from inside the boolean

Two sources, in priority order (`BOPTools_AlgoTools::MakePCurve`,
`BOPTools_AlgoTools.cxx:1657–1725`):

1. **The intersector supplies it.** `IntTools_Curve::FirstCurve2d()/SecondCurve2d()` — the SSI
   computes the 3D curve and both 2D footprints together (`:1687,1694`). Every `SEC` record in
   the corpus has `c2d1=1 c2d2=1`. This is the normal path.
2. **Projection**, only when the SSI's 2D curve is null: `BuildPCurveForEdgeOnFace`
   (`BOPTools_AlgoTools2D.cxx:48–70`) → `CurveOnSurface` → `Make2D` (`:432`) →
   `MakePCurveOnFace` (`:501`).

Then, unconditionally, `BRepLib::SameParameter(aE)` at `BOPTools_AlgoTools.cxx:1724`.

Also worth copying: `BOPTools_AlgoTools2D.cxx:566–585` **extends the surface by 1 % of its
parameter span in the non-periodic directions** before projecting, so the projection of a curve
that runs along the patch boundary is not clipped by the domain.

### 1.5 The tolerance model, entity by entity

**Every shape carries a model-space tolerance; it is a MEASURED containment radius, not a policy
constant.** The `--bool` probe proves this directly — for the 12 approximated pcurves,
`BRep_Tool::Tolerance(E)` tracks the measured same-parameter deviation:

| poles | measured dev | stored `tolE` | ratio |
|---:|---|---|---:|
| 30 | 1.14699464e-07 | 1.14703676e-07 | 1.0000368 |
| 37 | 1.29029198e-07 | 1.29109247e-07 | 1.00062 |
| 65 | 1.12263091e-07 | 1.1236843e-07 | 1.00094 |
| 72 | 9.76962031e-08 | 1e-07 | floor |
| 23 | 3.79312588e-08 | 1e-07 | floor |
| 9 | 3.32070637e-09 | 1e-07 | floor |

`tolE = max(1e-7, measured_deviation × small_margin)`. Nothing else.

The rules, with source and with the trace record that proves each:

1. **Constants** (`FoundationClasses/TKernel/Precision/Precision.hxx`):
   `Confusion = 1e-7` (`:165`), `PConfusion = 1e-9` (`:334`), `Angular = 1e-12` (`:123`),
   `Intersection = 1e-9` (`:220`), `Approximation = 1e-6` (`:235`), `PApproximation = 1e-8`
   (`:346`). The 1e-7 floor in the traces is `Confusion`.

2. **Containment invariant.** After building a pcurve, `MakePCurveOnFace` measures
   `IntTools_Tools::ComputeTolerance(C3d, C2d, S, t1, t2, …)`
   (`BOPTools_AlgoTools2D.cxx:637–643`), which runs `GeomLib_CheckCurveOnSurface` and returns
   `theMaxDist = (1 + 1.0e-5) × MaxDistance()` (`IntTools_Tools.cxx:774–775`). The 1e-5 margin is
   documented in place as protection against the curve being trimmed later and a *closer* minimum
   being found — i.e. the tolerance must stay valid under sub-ranging.

3. **Section-edge seed tolerance** = `max(BOPDS_Curve::Tolerance(), TangentialTolerance())`
   (`BOPAlgo_PaveFiller_6.cxx:886`), passed to
   `BOPTools_AlgoTools::MakeEdge(…, aTolR3D, aES)` (`:1024`). `TangentialTolerance` is 1.5e-7 for
   almost every curve in the corpus, which is why 29 of 142 `SEC` records sit at exactly 1.5e-7.

4. **Tolerance is recomputed PER PAVE-BLOCK SUB-RANGE, not inherited.** Verbatim from
   `traces/cone_cone_p1_cut.trace`:
   ```
   SEC   tag=final f1=2 f2=13 c=4 type=BSpline len=1.97972734 tol=1.50003136e-07 npb=4
   SECPB c=4 k=0 t0=0            t1=7.84036495e-06 edge=39 etol=1.5e-07
   SECPB c=4 k=1 t0=7.84036e-06  t1=0.999980837    edge=40 etol=1.5e-07
   SECPB c=4 k=2 t0=0.999970673  t1=1              edge=38 etol=1.50003136e-07
   SECPB c=4 k=3 t0=0.999980837  t1=0.999993124    edge=41 etol=1.5e-07
   ```
   Only the sub-range that actually contains the worst deviation keeps the parent value.
   A sub-edge inherits **its own** measured error, never the parent's.

5. **Vertex enlargement at a section endpoint = curve tolerance + exactly 1.0e-12.**
   `BOPTools_AlgoTools::DTolerance() { return 1.e-12; }` (`BOPTools_AlgoTools.hxx:70`), applied at
   `BOPTools_AlgoTools.cxx:1738–1741`. Measured twice at the pole-tangency case: curve tol
   2.14903722e-06 → vertex tol 2.14903822e-06; 1.70243155e-05 → 1.70243165e-05
   (`kb/occt_trace_findings.md` Q5.3).

6. **Distance-based growth at every merge.** `BB.UpdateVertex(aV, aDist + DTolerance())` at
   `BOPTools_AlgoTools_2.cxx:51,73,96`; the VV/VE/EE acceptance band is
   `tol(a) + tol(b) + max(fuzzy, Confusion)` (`BOPTools_AlgoTools.cxx:1757,1782`,
   `BOPTools_AlgoTools_2.cxx:267`). Acceptance and growth use the same number — a merge accepted at
   distance *d* always leaves both entities with tolerance ≥ *d*.

7. **"Monotone" is a design intent, not an invariant — and the audit that established this is
   right.** (`kb/audit_occt_tolerance-model.md` E1.) Confirmed at source: five hard-set,
   shrink-capable writers exist —
   `BRepLib.cxx:1734` (`aNTE->Tolerance(maxdist)` in `SameParameter`),
   `BRepLib.cxx:685` (`UpdateEdgeTol`),
   `BRepLib.cxx:889–892` (`UpdShTol` vertex force branch),
   `BRep_Builder.cxx:604` (`UpdateFace`),
   plus the boolean's own two: the per-FF **rollback** `TV->Tolerance(aTol)` at
   `BOPAlgo_PaveFiller_6.cxx:1073–1090` (restores the pre-FF tolerance of every vertex the FF pass
   inflated but did not end up using, *and* resets its bounding box by rebuild, not union) and
   `FilterPavesOnCurves` `TV->Tolerance(aRealTol)` at `BOPAlgo_PaveFiller_6.cxx:2527–2535`
   (reduces a vertex tolerance to the largest distance to a curve it is actually kept on).
   Only `BRep_Builder::UpdateEdge/UpdateVertex` are max-monotone.
   **Consequence for us: any bounding structure keyed on tolerance must be rebuilt, not unioned,
   after a tolerance write.**

8. **OCCT never lowers a tolerance below 1e-7**: `maxdist = max(maxdist, Precision::Confusion())`
   at `BRepLib.cxx:1731`. Untouched input geometry keeps 1e-7 end to end.

### 1.6 SAME-PARAMETER: what is enforced, how failure is detected, what is done about it

`BRepLib::SameParameter(const TopoDS_Edge&, tol, &newTol, useOldEdge)`,
`BRepLib.cxx:1251–1740`.

**The invariant enforced.** For an edge with 3D curve `C3d` on `[f3d, l3d]` and each pcurve
`C2d` on surface `S`: `C3d(t)` and `S(C2d(t))` must agree **at the same parameter t** across the
whole range, within the edge's tolerance. Not "the pcurve traces the same point set" — the same
point at the same parameter. That is why `range2 == range3` on all 82 probed pcurves.

**How failure is detected.** `ComputeTol(HC, HC2d, HS, NCONTROL=22)`, `BRepLib.cxx:1070–1188`,
called at `:1396`:

* 23 samples, `t = f + (l−f)·i/22`, `i = 0..22` (`:1090–1093`).
* `d = max‖C3d(t) − S(C2d(t))‖` (`:1122–1133`).
* **Out-of-domain penalty:** if `C2d(t)` leaves the surface's non-periodic parameter range by more
  than 1 % of the span, the excess is converted to model space through `1/UResolution(1)` and
  folded into the answer (`:1096–1121`). A pcurve that runs off its own patch is a tolerance
  failure, not silently clamped.
* An outlier filter: if fewer than 10 % of samples exceed 1.0 model unit, those are discarded and
  the max is taken over the rest (`:1147–1185`).
* Final: `d = 1.5 × d`, then `d = max(d, 1e-7)` (`:1185–1186`).

**What is done about failure**, in order (`:1625–1714`):

1. `Approx_SameParameter SameP(HC, HC2d, HS, aTol)` (`:1631`) — **reparametrises the 2D curve** so
   that it is same-parameter with the 3D curve. This is the real repair: it does not move the
   geometry, it re-fits the pcurve's parameterisation.
2. If `IsSameParameter()` already: keep, `maxdist = max(maxdist, TolReached())` (`:1633–1647`).
3. If `IsDone()` but not already same-parameter: accept the new 2D curve only if
   `TolReached() ≤ error` (the pre-existing measured error) — otherwise keep the old curve and
   carry `error` as the tolerance (`:1648–1672`). *Never* accept a repair that is worse than what
   it replaces.
4. If the approximation fails outright: fall back to `GeomLib::SameRange` (a pure affine
   reparameterisation of the pcurve onto `[f3d, l3d]`) and set `IsSameP = false` (`:1673–1696`).
5. Escape hatch (`:1703–1714`): if `edgeTol + max(PrecCurve(C3d), PrecSurface(S)) ≥ error`, declare
   success anyway — i.e. if the disagreement is within the intrinsic numerical precision of the
   curve and surface evaluators, it is not a real failure.
6. Outcome (`:1719–1737`): `B.Range(f3d,l3d)`, `B.SameRange(true)`, and **if same-parameter holds,
   the edge tolerance is HARD-SET to `max(maxdist, 1e-7)`** — this both grows and shrinks — and
   `B.SameParameter(true)`.

Two upstream helpers used before the fit:

* `Geom2dConvert::C0BSplineToC1BSplineCurve(bs2d, TolConf2d)` when the pcurve is a C0 BSpline
  (`:1404–1417`), with `TolConf2d = max(min(UResolution(tol), VResolution(tol)), PConfusion)`.
* `Approx_CurvilinearParameter` re-parameterisation when the pcurve's knot spacing is pathological
  — detected by a knot-interval **ratio** test (`:1560–1621`).

---

## 2. WHAT WE DO

### 2.1 We store no tolerance on any entity in the v1 BRep

`src/brep.h:27–58`:

```cpp
struct BRepVertex { int point_index; std::vector<int> edge_indices; };
struct BRepEdge   { int curve_3d_index; int start_vertex; int end_vertex;
                    std::vector<int> trim_indices; };
struct BRepTrim   { int curve_2d_index; int edge_index; int loop_index;
                    bool reversed; BRepTrimType type; };
struct BRepFace   { int surface_index; std::vector<int> loop_indices; bool reversed; Color; };
```

There is no tolerance field on a vertex, an edge, a trim, or a face. `tolerance` appears in
`brep.h` only as a *function argument* (`:220–401`) — a single global epsilon threaded through the
call. **The approximation error of every curve and pcurve we build is computed and then thrown
away.** Nothing downstream can ask "how far off is this edge".

The v2 arena does better: `src/brep_bds.h:168,184,277` carry a per-entity `double tol`, with
`tolerance(i)` / `absorb_tolerance(i,d)` accessors (`:432–434`) and a `tolerances_are_default()`
flag (`:394`) so tests can tell a measured 1e-7 from an assumed one. That infrastructure exists
and is unused by the pcurve construction (§2.4).

### 2.2 We have no exact-pullback dispatcher for curved surfaces

There is exactly one exact-pullback path in the kernel, and it is **planar only**:
`brep.cpp:5966–5999`. For a degree-1×1 parallelogram-CV patch it inverts the affine frame
directly and maps each control point with its weight unchanged:

```cpp
bool ok2 = nc>=2 && pc.create(3, arc3.is_rational(), ord, nc) && ...
for (int i=0;i<nc && ok2;++i){ Point Pe=arc3.get_cv(i); double w=arc3.weight(i);
    double uu, vv; if (affine) to_uv(Pe, uu, vv); ...
    ok2 = pc.set_cv_4d(i, uu*w, vv*w, 0.0, w); }
```

This is exactly OCCT's `ProjLib_Plane` + `ProjLib_ComputeApprox.cxx:1197–1252` pole-projection
rule, and it is correct. **There is no analogue for a cylinder, cone, sphere or torus.** For a
curved face, `brep.cpp:5896–5964` takes the *stored* pcurve and splits it — which is right when
the stored pcurve is right, and inherits whatever error it has when it is not.

### 2.3 Every split face's 3D edge is rebuilt as a degree-1 polyline

`brep.cpp:3588–3627`, `lift_loop`, the only 3D-edge constructor on the split path
(single call site, `brep.cpp:4250`):

```cpp
// Lift a 2D pcurve onto the surface as a 3D polyline edge, refining ADAPTIVELY by 3D
// chord deviation ...
if (dev > devtol && depth < 9) { rec(...); acc.push_back(pmid); rec(...); }
...
c3d = NurbsCurve::create(false, 1, pts3);
```

with `devtol = sd * 2e-3` where `sd` is the **surface bounding-box diagonal** (`brep.cpp:3976`).

For the battery's sphere (r=2.5, bbox diagonal 8.66) that is `devtol = 1.73e-2`. Chord sagitta
`≈ Rθ²/8`, so the recursion stops at `θ ≈ 0.235 rad` — **~27 chords for a full section circle**,
whose inscribed-polygon area deficit is `θ²/6 ≈ 9.2e-3 relative`. The code says so itself at
`brep.cpp:5596–5600`:

> the lift stores every 3D edge as a chord polyline (devtol ~2e-3 of the face size), so a split
> section CIRCLE — and every pcurve rebuilt from its pieces — INSCRIBES the true circle: O(1/N²)
> area/volume deficit (box × sphere ~1.5e-3).

A **post-hoc rescue** exists: `brep.cpp:5596–5620` fits a circle/ellipse to the polyline CVs and,
on success, rebuilds each piece as an exact rational arc via `exact_arc_3d` (`brep.cpp:5394–5420`,
correct rational-quadratic construction with `w = cos(φ/2)`). It fires only when
`C.degree()==1 && !C.is_rational()` and the fit succeeds. This is recognition-after-destruction:
the exact conic was known at the intersector and was thrown away, then guessed back.

The section curves themselves are also degree-1: `brep_section.cpp:2597`
`NurbsCurve c3d = NurbsCurve::create(false, 1, s.p3);` for every shared section edge.

### 2.4 v2's section pcurve: 192 samples, cubic interpolation, error never recorded

`v2/brep_v2_boolean.cpp:296–344`, `v2sol_trail_pcurve` — the pcurve of every section block:

```cpp
static const bool s_lin = getenv("SESSION_V2_SEC_LINEAR");
if (!s_lin && pts.size() >= 4) {
    const NurbsCurve c = NurbsCurve::create_interpolated(pts);
    if (c.is_valid()) return c;
}
return NurbsCurve::create(false, 1, pts);
```

sampled at `SESSION_V2_SEC_N`, default **192** (`v2/brep_v2_boolean.cpp:907–911`). The comment at
`:331–334` records the measurement that motivated the cubic: sphere × sphere closure residual
1.19e-04 at n=32, 6.59e-06 at 128, 4.12e-07 at 512 — clean O(h²) for degree 1.

The **boundary** trims are handled correctly — `v2/brep_v2_boolean.cpp:872–882` trims the stored
pcurve instead of re-sampling it, with the right reason given in place. It is only the *section*
pcurves that are sampled.

The tolerance that v2 does compute, `V2Section::measure_tolerance`
(`v2/brep_v2_section.cpp:946–967`), is the right *shape* — it even copies OCCT's margin,
`c.tol = max(c.tol, max(dev1,dev2) * (1.0 + 1e-5))`, matching `IntTools_Tools.cxx:774` — but it
measures the **wrong distance and at the wrong time**:

* it measures `C.invert(P, u, v)`, the *closest-point* distance from the 3D curve to the surface,
  which is a lower bound on the same-parameter deviation, not the deviation;
* it runs at `brep_v2_section.cpp:1177`, **before** `v2sol_trail_pcurve` builds the pcurve at
  `brep_v2_boolean.cpp:296`. The pcurve's own approximation error therefore cannot be, and is not,
  in `c.tol`. The tolerances written into the arena at `brep_v2_section.cpp:1094–1135` are all
  `c.tol`, i.e. all pre-pcurve.

### 2.5 We have no same-parameter concept at all

`grep -rn "SameParameter|same_param|SameRange" src/*.{cpp,h} src/v2/*` returns hits only in
`file_step.cpp` (STEP round-trip, where the *writer* has to satisfy OCCT's reader) and one
declaration in `brep.h:356–360`:

```
/// OCCT SameParameter-lite: for every 2-trim mixed (planar/curved) section edge, rebuild the
/// PLANAR trim's pcurve from the edge's shared 3D curve by exact affine projection ...
void sameparameter_planar_pcurves();
```

That is a repair for one specific configuration (mixed planar/curved section edges), not an
invariant. Nothing in the kernel:

* asserts that `C3d(t)` and `S(C2d(t))` agree at the same `t`;
* measures the disagreement;
* reparameterises a pcurve to restore agreement;
* records that an edge failed to achieve it.

Two independently sampled pcurves for the two faces of one section edge are the *normal* case in
our pipeline, and nothing checks that they are the same curve.

### 2.6 Rational curves are expressible, but the constructor in use cannot express them

`nurbscurve.h:64` — `create(bool periodic, int degree, const std::vector<Point>&)` has no weight
parameter and is what every construction site listed above calls. But `nurbscurve.h:83–88`
`create_from_parameters(points, weights, knots, mults, degree, periodic)` **does** take weights,
`set_weight`/`set_cv_4d` exist, and `exact_arc_3d` (`brep.cpp:5394`) already builds exact rational
conics with them. **The representation is not the blocker; the construction path is.**

---

## 3. THE GAP — concrete implementable deltas

Numbered `R1…R7`, in the order they should be built (§5 restates as increments with tests).

**R1. An exact-pullback dispatcher `pullback_exact(surface, curve3d) -> optional<NurbsCurve2d>`.**
Mirror of §1.2. Nine recognisers:

| # | surface | curve | condition | 2D result |
|---|---|---|---|---|
| 1 | plane | any NURBS | always | map every CV through the inverse affine frame, **keep weights, knots, mults, degree** |
| 2 | cylinder | line | `‖dir × axis‖ ≤ Angular` | degree-1, 2 CVs, `u = u₀`, `v = ±t + v₀` |
| 3 | cylinder | circle | `‖n_circ × axis‖ ≤ Angular` | degree-1, 2 CVs, `u = ±t + u₀`, `v = v₀` |
| 4 | cone | line | `dir ∥ ∂S/∂v` at the foot | degree-1, `u = u₀` |
| 5 | cone | circle | `n_circ ∥ axis` | degree-1, `v = z/cos(semiangle)` const |
| 6 | sphere | circle | `n_circ ⟂ axis` **and** centres coincide | degree-1, `u = u₀` (mind the ±π/2 fold, `ProjLib_Sphere.cxx:203–248`) |
| 7 | sphere | circle | `X_circ ⟂ axis && Y_circ ⟂ axis` | degree-1, `v = asin(z/R)` const |
| 8 | torus | circle | `|OC| < Confusion` or `n_circ ∥ axis` | degree-1, `v` const |
| 9 | torus | circle | otherwise, **only if** the circle's centre lies on the torus axis-circle and its radius equals the minor radius | degree-1, `u` const |

Recogniser 9 is where we must be **stricter than OCCT** — `ProjLib_Torus.cxx:172` has no gate and
returns a pcurve 1.27 units wrong (§1.2). Add the explicit test; return nullopt otherwise.

Every one of these returns a **degree-1 2-CV NURBS in UV** whose parameterisation is the 3D
curve's own — i.e. same-parameter by construction, exact to machine epsilon (measured: 0 …
1.4e-15). Recogniser 1 additionally preserves rationality, which is what makes exact conics on
planes work.

The dispatcher must be called **for the operand's own trims too**, not only for section curves —
a split cylinder cap circle is recogniser 3.

**R2. Carry the section curve's ANALYTIC TYPE forward instead of recognising it back.**
Our section stage produces the trail; where the surface pair is one of the analytic-dispatcher
cases the true curve is a circle / ellipse / line and we already know it. Emit
`exact_arc_3d(...)` (which exists, `brep.cpp:5394`) as the 3D curve and R1's exact pullback as
both pcurves. Retire the polyline → `circle_through_points` → rebuild round trip
(`brep.cpp:5596–5620`), keeping it only as a fallback for imported freeform.

**R3. Replace `lift_loop`'s degree-1 lift with (a) exact when R1 fires, (b) a degree-5
approximation with a MEASURED tolerance otherwise.** `devtol = 2e-3 × bbox_diagonal`
(`brep.cpp:3976`) is 1.7e-2 on the battery's sphere; that is five orders of magnitude looser than
the answer we are trying to produce. The tolerance must be an absolute model-space target
(1e-7-class), and the degree must be ≥ 3 so the sample count needed to hit it is finite (§4).

**R4. A per-entity tolerance field on `BRepVertex`, `BRepEdge`, `BRepTrim`, `BRepFace`, with a
containment invariant and a measurement function.** v2's arena already has the field
(`brep_bds.h:168,184,277`); v1 has none. The measurement function is
`same_parameter_deviation(C3d, C2d, S, t0, t1)` = OCCT's `ComputeTol` (`BRepLib.cxx:1070`):
23 samples, out-of-domain penalty through the surface metric, ×1.5, floored at 1e-7. Writers:
`absorb(i, d)` for max-monotone growth at merges, and an explicit `set(i, d)` for the
shrink-capable sites (SameParameter, rollback). Bounding boxes keyed on tolerance are **rebuilt**
after a write, never unioned (§1.5 rule 7).

**R5. Tolerance recomputed per sub-range.** When an edge is split at a pave, each piece's
tolerance is re-measured on **its own** sub-range, not inherited. Direct port of the measured
behaviour in §1.5 rule 4. Vertex enlargement at a section endpoint is `curve_tol + 1e-12`
(`BOPTools_AlgoTools.hxx:70`).

**R6. `same_parameter(edge)` — enforce, detect, repair.**
Enforce: `‖C3d(t) − S(C2d(t))‖ ≤ tol(edge)` for every pcurve of the edge, at the same `t`, with
`domain(C2d) == domain(C3d)`.
Detect: R4's measurement function.
Repair, in OCCT's order (`BRepLib.cxx:1625–1714`): (i) affine range alignment
(`GeomLib::SameRange` equivalent) — we have `NurbsCurve::set_domain`; (ii) re-fit the pcurve's
**parameterisation** against the 3D curve (`Approx_SameParameter` equivalent); (iii) accept the
re-fit only if it beats the pre-existing error; (iv) on failure, set the edge tolerance to the
measured error and flag the edge `same_parameter=false` rather than pretending.
Run it: once after every section-edge construction, and once as a whole-shape pass before the
verdict, exactly as OCCT does at `BOPTools_AlgoTools.cxx:1724`.

**R7. Both faces of a section edge share ONE pcurve object per face, derived from ONE 3D curve.**
v2 already does this for section blocks (`brep_v2_boolean.cpp:919–931` pushes the same
`NurbsCurve*` for the Forward and Reversed entries). The invariant to add is that the *two
faces'* pcurves are built from the *same* 3D curve by R1/R3, so the closure residual is bounded by
each pcurve's own tolerance rather than by the difference between two independent samplings.

---

## 4. MEASURED IMPACT

### 4.1 The core number: what pcurve degree costs, on a case from the battery

`sag.py`, sphere r=2.5 × the tilted section circle of `sph×cyl@45°` (a genuinely transcendental
pullback — recogniser 7 does not fire, `ProjLib_Sphere` does not fire, this is the honest
worst case). Max 3D deviation of the reconstructed boundary from the true circle:

| samples `n` | degree 1 | cubic | ratio |
|---:|---|---|---:|
| 32 | 9.303726e-03 | 4.551987e-04 | 20 |
| 64 | 2.340621e-03 | 4.069906e-05 | 58 |
| 128 | 5.860465e-04 | 4.834359e-06 | 121 |
| **192** | **2.605197e-04** | **1.423559e-06** | **183** |
| 256 | 1.465489e-04 | 5.990608e-07 | 245 |
| 384 | 6.513305e-05 | 1.771421e-07 | 368 |
| 512 | 3.663781e-05 | 7.469585e-08 | 490 |

Degree 1 is O(h²); cubic is O(h⁴). The 183× ratio at n=192 is the same order as the reported
"cubic moved cells 54–128×" — this is the mechanism, quantified.

Reading off the three representations at a **1e-7 target**:

| representation | control points needed |
|---|---:|
| OCCT: degree 8, `Approx_FitAndDivide2d` | **65** (measured, `--proj`) |
| cubic interpolation | ~490 (extrapolated from the table, O(h⁴)) |
| degree-1 polyline | ~9 800 (extrapolated, O(h²)) |

And on the **other side of the same section** — the cylinder — the pullback is a `gp_Lin2d` with
`devsameparam = 4.97e-16` (measured). **Zero control points of approximation, exactly, on one of
the two faces sharing that edge.** Our pipeline samples both.

Our current v2 default (cubic @ 192) sits at **1.42e-6** on that boundary; OCCT sits at
**1.12e-7** with 65 poles. We are 13× worse than OCCT after the cubic fix, and we pay 192 CVs to
be worse.

### 4.2 Which of the 63 cells this layer explains

The 21 pairs are `main_7.cpp:236–247`, placements `main_7.cpp:207–229`. Classifying each pair's
sections by whether R1 fires on each side (this is **geometric reasoning**, with the
representative cases in each class measured through `--proj` in §1.2):

| # | pair | section geometry | R1 side A | R1 side B |
|---|---|---|---|---|
| 1 | box × box2 | lines | ✅ affine | ✅ affine |
| 2 | box × sph | circles; ±Z faces ⟂ sphere axis, ±X/±Y not | ✅ affine | ⚠️ 2 of 6 exact |
| 3 | box × cone | circles / hyperbolas | ✅ affine | ⚠️ circles only |
| 4 | box × cyl | circles ⟂ axis only (walls miss, r=1.5 < 2) | ✅ | ✅ |
| 5 | box × tor | **spiric quartics** (planes ∥ axis, off-axis) | ✅ affine | ❌ |
| 6 | sph × sph2 | circle ⟂ X, neither sphere's axis | ❌ | ❌ |
| 7 | sph × cone | coaxial → circles ⟂ Z | ✅ | ✅ |
| 8 | sph × cyl | coaxial → circles ⟂ Z | ✅ | ✅ |
| 9 | sph × tor | coaxial → circles ⟂ Z | ✅ | ✅ |
| 10 | cone × cone2 | coaxial → circles | ✅ | ✅ |
| 11 | cone × cyl | coaxial → circles | ✅ | ✅ |
| 12 | cone × tor3 | coaxial → circles | ✅ | ✅ |
| 13 | cyl × cyl2 | equal-radius orthogonal → 2 ellipses at 45° | ❌ | ❌ |
| 14 | cyl × tor | coaxial → circles | ✅ | ✅ |
| 15 | tor × tor2 | parallel offset axes → quartic | ❌ | ❌ |
| 16 | cyl × cylR | skew 45°, equal radii → ellipses | ❌ | ❌ |
| 17 | box × boxR | lines | ✅ affine | ✅ affine |
| 18 | boxR × sph | ±Z faces ⟂ axis; 4 side faces not | ✅ affine | ⚠️ 2 of 6 |
| 19 | box × torR | torus tilted 45° → quartics | ✅ affine | ❌ |
| 20 | coneR × cyl | skew quadrics → quartic space curve | ❌ | ❌ |
| 21 | boxR × cyl | rotation about Z keeps side faces ∥ and caps ⟂ the cylinder axis → lines ∥ axis + circles ⟂ axis | ✅ affine | ✅ |

**36 cells (12 pairs) are FULLY exact-pullback-eligible on both sides. 9 more (3 pairs) are exact
on one side and partly on the other. 18 cells (6 pairs) are genuinely transcendental.**

### 4.3 Ranked impact

**Rank 1 — R1 + R2 + R3, exact pullbacks and exact section curves. Attributable failing cells:
`boxR × cyl` cut/common/fuse (3), plus the accuracy floor of every one of the 36 exact-eligible
cells.**

`boxR × cyl` is the decisive evidence. Its complete section set is *lines parallel to the
cylinder axis* and *circles perpendicular to it* — recognisers 2 and 3, both measured at
`devsameparam = 0` (§1.2). Both are exact on the plane side too (recogniser 1). Every pcurve in
that cell can be a 2-CV degree-1 UV segment with zero error. Measured error today
(`kb/hunt_oriented_primitives.md` §1): cut **6.14e-5**, common **2.55e-4**, fuse **3.40e-5** —
the worst relative error of the whole oriented battery. That error cannot be attributed to
transcendence, to the marcher's convergence, or to topology: the geometry is exactly
representable and we do not represent it. Compare the axis-aligned sibling `box × cyl`
(only the ⟂ caps cut, no ∥-axis lines) at **8.92e-15**.

Same argument, weaker only because I do not have per-cell current numbers for them: pairs 7–12
and 14 are coaxial, so every section is a circle ⟂ both axes, exact on both sides — and the
axis-aligned gate still shows `sph × cone cut 1.26e-07`, `sph × cyl cut 1.12e-07`,
`sph × tor cut 1.87e-07`, `sph × sph fuse 8.54e-08` (`validation/_gate_matrix.txt:17–28`).
Those are **not zero**, and for exact-eligible geometry they should be ~1e-15. Every one of those
residuals is representation error.

**Rank 2 — R3's `devtol`. Attributable: the accuracy class as a whole, and it is a one-line
change.** `devtol = sd × 2e-3` (`brep.cpp:3976`) is 1.73e-2 on a sphere of radius 2.5. That
admits a 27-chord polyline for a full section circle, a 9.2e-3 relative area deficit, which the
code's own comment records as "box × sphere ~1.5e-3" (`brep.cpp:5599`). This is the largest single
loosely-set constant in the layer and it sets the floor under everything the conic-recovery pass
does not rescue.

**Rank 3 — R6, same-parameter. Attributable: no single cell yet — mark as measurement-blocked, not
as no-impact.** We cannot attribute cells to it because **we do not measure the quantity**.
What is established: OCCT holds `SameParameter=1, SameRange=1` on 82/82 result pcurves with
`range2 == range3` bitwise, and its edge tolerance *is* the measured same-parameter deviation
(§1.5 table). Our pipeline builds the two faces' pcurves of a section edge by two independent
samplings of a trail (`v2sol_trail_pcurve` called once per face,
`v2/brep_v2_boolean.cpp:905–931`) and never compares them. The v2 closure-residual measurement
already quoted (`brep_v2_boolean.cpp:331–334`: 1.19e-4 → 4.12e-7 as n goes 32 → 512) *is* a
same-parameter deviation measured indirectly. Implementing R4+R6 converts that from an anecdote
into a per-edge number, which is the precondition for attributing any remaining failure.

**Rank 4 — R4/R5, the tolerance model. Attributable: `coneR × cyl` and the structural class,
indirectly.** No accuracy cell is caused by the missing tolerance field. But the structural class
— open shells, 47–490 % volume error, `coneR × cyl` at cut 43.1452 vs 12.3074 — is where welds,
adoptions and junction closures decide topology on distance comparisons, and every one of those
comparisons currently uses a global epsilon. `kb/p1_attack_plan.md` §M4 attributes 4 of 174
chairsROT naked edges *directly* to flat bands and names per-entity growth as infrastructure for
M1 (121 edges). This layer supplies that infrastructure. The measured OCCT rule to copy is
§1.5 rules 2, 4, 5, 6 verbatim, including the shrink-capable writers (rule 7) — a growth-only
model will not reproduce OCCT and will over-inflate.

**Rank 5 — the transcendental 18 cells (pairs 6, 13, 15, 16, 19, 20).** R1 cannot help these; only
R3's degree and tolerance can. Current worst: `box × torR` common **4.83e-2**, cut **2.11e-2**;
`cyl × cylR` fuse **5.56e-2** with `is_solid=0`. Note the honest caveat already established in
`kb/hunt_oriented_primitives.md` §1: **OCCT's own torus answers disagree with an independent
FreeCAD/OCCT run by 1.5e-4 absolute**, so on the torus cells the target is analytic truth, not
OCCT parity, and reference digits past 1e-4 must not be quoted.

**Explicitly no measured impact yet:**

* Rational pcurves for transcendental cases. OCCT's approximated pcurves are **all
  non-rational** (measured: `rat2=0` on 12 of 12). Weights buy nothing here. They matter only for
  recogniser 1 (exact conics on planes), which we already do.
* Degree 8 vs degree 5 for the approximation. OCCT uses `[5,8]`; I measured degree 8 output but
  did not measure whether degree 5 would need materially more poles for the same tolerance.
* `Approx_CurvilinearParameter` re-parameterisation (`BRepLib.cxx:1598`). It fires only on
  pathological knot spacing, which nothing in our corpus has been shown to produce.

---

## 5. IMPLEMENTATION ORDER — smallest shippable increment first

**Increment 1 — `pullback_exact()`, recognisers 1–3 only (plane, cylinder × line, cylinder ×
circle).** New free function + unit test; no pipeline change yet.
*Acceptance:* a table-driven test over the §1.2 battery asserting that each recogniser returns a
2-CV degree-1 UV curve and that `max|C3d(t) − S(C2d(t))|` over 401 samples is `< 1e-14`, and that
each non-matching input returns nullopt. Reproduce the three `--proj` rows
`plane × circle-in-plane`, `cyl × circle-perp-axis`, `cyl × generator-line` at `dev = 0`.

**Increment 2 — wire recognisers 1–3 into the split path** (`brep.cpp:4250` / `lift_loop`, and
`v2/brep_v2_boolean.cpp:872–931`): when the trim's 3D curve and the face's surface match a
recogniser, emit the exact pcurve and skip the sampled lift entirely.
*Acceptance:* `boxR × cyl` cut/common/fuse move from 6.14e-5 / 2.55e-4 / 3.40e-5 to `< 1e-12`;
`box × cyl` and `box × box` stay at their current 1e-14/1e-15; face counts unchanged on all 63
cells; `main_14` stays 62/62.

**Increment 3 — recognisers 4–9 (cone, sphere, torus), with the torus gate OCCT lacks.**
*Acceptance:* the remaining `--proj` exact rows reproduce at `dev < 1e-14`; the adversarial
`torus × arbitrary-circle` case (`dev = 1.274` in OCCT) returns **nullopt** from our dispatcher.
Cells: the seven coaxial pairs (7–12, 14) drop from ~1e-7 to `< 1e-12`; `box × sph` and
`boxR × sph` improve on their ±Z caps only.

**Increment 4 — tighten `devtol` and raise the fallback degree.** Replace
`devtol = sd * 2e-3` (`brep.cpp:3976`) with an absolute model-space target
`max(1e-7, 1e-9 × sd)`, and make the non-exact lift a degree-5 fit rather than a degree-1 chord
polyline. This is what makes the sample count finite: at degree 1 a 1e-7 target on the battery's
sphere needs ~9 800 points; at cubic ~490; at degree 5 fewer still.
*Acceptance:* the 18 transcendental cells improve monotonically and no cell regresses; runtime on
`box × tor` (currently 4.5e7 µs, `validation/_gate_matrix.txt:14`) does not more than double.

**Increment 5 — per-entity tolerance fields + `same_parameter_deviation()`.**
Add `double tol` to `BRepVertex/BRepEdge/BRepTrim/BRepFace` (`brep.h:27–58`), mirroring
`brep_bds.h:168`. Implement `ComputeTol` exactly: 23 samples, out-of-domain penalty through
`1/UResolution(1)`, ×1.5, floor 1e-7 (`BRepLib.cxx:1070–1188`). Write it at every pcurve
construction site.
*Acceptance:* a new diagnostic prints, per result edge, `tol` and `max_sameparam_dev`, and asserts
`dev ≤ tol` on **every** edge of all 63 cells — the containment invariant. Cells where it fails
are then, for the first time, attributable.

**Increment 6 — per-sub-range recompute + `curve_tol + 1e-12` vertex enlargement.**
*Acceptance:* reproduce the `cone_cone_p1_cut c=4` pattern qualitatively — four sub-blocks of one
curve must not all carry the parent's tolerance; only the sub-range containing the worst deviation
may. And the containment invariant of Increment 5 must still hold after splitting.

**Increment 7 — `same_parameter(edge)` repair.** Range alignment, then parameterisation re-fit,
then accept-only-if-better, then flag-and-record on failure (§3 R6).
*Acceptance:* every result edge of all 63 cells reports `same_parameter = true` with
`domain(C2d) == domain(C3d)`; where it cannot, the edge is flagged and its tolerance equals the
measured deviation. Target parity with the measured OCCT state: **82/82 `samep=1 samer=1`.**

**Increment 8 — one 3D curve, two derived pcurves, per section edge (R7).**
*Acceptance:* for every 2-trim section edge, the two faces' pcurves are byte-identical functions of
the same 3D curve; the closure residual reported by the v2 verdict harness drops below each
edge's own tolerance rather than tracking the sample count.

---

## Appendix — reproducing the measurements

```
# pcurve census + same-parameter deviation over 6 real booleans
<scratchpad>/pcprobe --bool

# canonical pullback battery through ProjLib_ProjectedCurve
<scratchpad>/pcprobe --proj

# degree-1 vs cubic error scaling for the sphere-side tilted-circle pullback
python3 <scratchpad>/sag.py
```

Existing trace records quoted: `traces/sph_cyl_roty45_cut.trace:198–209` (three exact `Circle`
section curves with their per-block tolerances), `traces/cone_cone_p1_cut.trace` (`SEC c=4` +
four `SECPB` — the per-sub-range recompute), `traces/_summary.txt` (per-case digests).
