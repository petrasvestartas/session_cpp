# PORT SPEC 04 — STAGE 6: FACE/FACE SECTION CURVES BETWEEN TWO TRIMMED CURVED FACES

Build document. Everything below is grounded in the OCCT tree at
`/home/petras/code/code_cpp/OCCT`; paths are relative to `src/`. Our code is at
`/home/petras/code/code_rust/session/session_cpp/src`. Read-only on both trees.

Scope: given two **trimmed** faces `F1`, `F2` in **arbitrary relative pose**, produce the
set of section curves that lie on **both trimmed faces**, converted into edges with
correct tolerances, and **fused** so that a curve produced for pair `(F1,F2)` and the same
curve produced for pair `(F1,F3)` is ONE edge entity.

Companion audits that this document assumes and does not repeat:
`kb/audit_occt_ff-samedomain.md` (PostTreatFF fusion mechanics, SD verdict — trust it over
`kb/occt_ff-posttreat-samedomain.md`), `kb/audit_occt_tolerance-model.md`,
`kb/audit_occt_ssi-walking.md`, `kb/audit_occt_intana-analytic.md`.

Naming below: **FF** = the `IntTools_FaceFace` stage (surface-level, rectangle-trimmed);
**MakeBlocks** = the `BOPAlgo_PaveFiller` stage that turns section curves into real edges
(this is where the *wire* trimming actually happens); **PostTreatFF** = the cross-pair
fusion.

---

## 1. WHAT THIS SUBSYSTEM MUST GUARANTEE

Each invariant is stated so it can be asserted in code with no oracle. `tol(x)` is a
model-space (3D) distance. `A`,`B` are the two operands.

**G1 — On both surfaces.** Every point of every emitted section curve is within
`tol(curve)` of *both* underlying surfaces:
`max_t max(d(c(t),S1), d(c(t),S2)) <= tol(curve)`.
Sampled at ≥ 64 parameters plus every knot. This is checkable without knowing the answer.

**G2 — Inside both trims (blocks, not curves).** The curve is cut into *pave blocks* by
paves. For every emitted block `[t0,t1]`, the point at
`t = 0.568·t0 + 0.432·t1` classifies **IN or ON** against `F1`'s wires and against `F2`'s
wires. No emitted block has its 43.2 % point OUT of either face.
(OCCT: `IntTools_Context::IsValidBlockForFaces`, `IntTools_Context.cxx:717-755`;
constant `PAR_T = 0.43213918`, `IntTools_Tools.cxx:254-259`.)

**G3 — Blocks are delimited by real entities.** Every block endpoint is either (a) an
existing vertex of A or B that lies on the curve, (b) a vertex created by an earlier
interference stage (VV/VE/EE/VF/EF), (c) a bound of the curve that classified valid on
both faces, or (d) a closing pave of a closed curve. **Never** an ad-hoc numeric
bisection of a classification predicate. If a block endpoint has no entity behind it, the
subsystem is wrong even when the geometry looks right.

**G4 — One entity per geometric feature (Law 1).** If the same 3D curve arises from two
different face pairs sharing a face (`(F1,F2)` and `(F1,F3)`), or from a pair and an
existing model edge, the result is exactly ONE edge object referenced by all touching
faces. Test: build the set of emitted section edges; no two of them are within
`tol1+tol2+fuzz` of each other along their whole overlap. Equivalently: `#edges ==
#geometrically distinct curves`.

**G5 — Section edges reach the boundary.** In each face's 2D arrangement (wires ∪ section
pcurves), the planar graph has **no vertex of degree 1**. A dangling section end means the
curve stopped inside the face — either the trimming dropped a block it should have kept,
or a boundary crossing was not paved.

**G6 — Domain invariance (Law 2).** Re-parameterising either face's UV domain by any
affine map (`u' = a·u + b`, `a>0`), or padding the domain (the STEP round-trip case
`u∈[0,1] → u∈[-0.04,4.04]`), changes the emitted 3D section set by less than `1e-12`
Hausdorff and changes the block count not at all. No tolerance in this subsystem may be a
function of `range(u)` or `range(v)`.

**G7 — Pose invariance.** Applying a rigid motion `R` to both operands produces the
transformed section set: same curve count, same block count, same topology, 3D positions
within `1e-12 · diag` of `R·(original)`.

**G8 — Measured tolerance.** `tol(curve) >=` the maximum deviation between the 3D curve
and each of its two pcurves evaluated on the respective surface. An edge whose tolerance
does not cover its own pcurves is invalid by construction.
(OCCT: `IntTools_FaceFace::ComputeTolReached3d`, `IntTools_FaceFace.cxx:613-691`.)

**G9 — Typed degeneracy (Law 4).** The pair result carries a discriminated outcome:
`OK(curves,points)`, `TANGENT(opposite: bool)`, `EMPTY`, `FAILED`. Tangency is never
reported as "0 curves, OK". OCCT: `TangentFaces()` short-circuits before curve production
(`IntTools_FaceFace.cxx:536-540`).

**G10 — No curve is invented at a graze.** A section that coincides with an existing edge
of either operand must resolve to that existing edge, not to a second, nearly-identical
one. (OCCT: `BOPAlgo_PaveFiller::IsExistingPaveBlock`, two overloads,
`BOPAlgo_PaveFiller_6.cxx:1988-2043` and `:2047-2251`.)

**G11 — Determinism.** Same inputs ⇒ byte-identical output. Any map iteration that feeds
vertex identity must be ordered (this is the one place OCCT itself is order-dependent; see
`kb/audit_occt_ff-samedomain.md` traps 12/13 — we do better, deliberately).

---

## 2. OCCT'S ALGORITHM

### 2.0 The two-level structure (this is the single most important fact)

OCCT **does not** trim the section curve to the faces' wires inside the FF intersector.
It trims to the faces' **UV rectangles**, and the wire-level trimming happens one stage
later, in `MakeBlocks`, on *pave blocks* rather than on the raw curve:

| level | who | domain used | granularity |
|---|---|---|---|
| L1 surface | `IntPatch_*` intersectors | rectangle `[u0,u1]×[v0,v1]` via `Adaptor3d_TopolTool` restriction lines | line vertices |
| L2 rectangle | `GeomInt_LineConstructor` | same rectangle, `Classify != OUT` | parameter spans |
| L3 wires | `IntTools_Context::IsValidBlockForFaces` inside `MakeBlocks` | `IntTools_FClass2d` on the real wires | pave block |
| L4 arrangement | `BOPAlgo_BuilderFace` | 2D arrangement of wires ∪ section pcurves | face piece |

`IntTools_TopolTool` (`IntTools/IntTools_TopolTool.cxx`) overrides **only** the sampling
(`ComputeSamplePoints`, `:78-394`); it does **not** override `Classify`. So the classifier
used at L1/L2 is `Adaptor3d_TopolTool::Classify` (`ModelingData/TKG3d/Adaptor3d/
Adaptor3d_TopolTool.cxx:280-...`), a pure rectangle in/on/out test with `nbRestr==4`
short-circuit at `:287-299`.

Port consequence: **do not** try to make the marcher wire-aware. Make it rectangle-aware,
and put the wire logic in L3 where OCCT puts it. Our kernel currently has neither L1
domain awareness nor L3 (see §4).

### 2.1 Entry, face pre-conditioning, and the UV rectangle

Entry point `IntTools_FaceFace::Perform(F1,F2,runParallel)`,
`IntTools/IntTools_FaceFace.cxx:330-609`.

**(a) Far-from-origin recentring.** Before `Perform`, the BOP wrapper translates both
faces near the origin when they are far away:
`BOPAlgo_FaceFace::Perform`, `BOPAlgo/BOPAlgo_PaveFiller_6.cxx:202-241`, using
`BOPAlgo_Tools::TrsfToPoint` (`BOPAlgo/BOPAlgo_Tools.cxx:1912-1937`): translate iff
`|centre - origin| >= 1e5` **and** `bboxDiag / |centre| <= 1e-5`. The inverse transform is
applied to the produced curves and points afterwards (`ApplyTrsf`, `:244-265`). Port this:
it is a pure accuracy win and costs nothing.

**(b) Type canonicalisation.** `SortTypes/IndexType`, `IntTools_FaceFace.cxx:2826-2895`.
Index: Plane 0, Cylinder 1, Cone 2, Sphere 3, Torus 4, Bezier 5, BSpline 6,
SurfaceOfRevolution 7, SurfaceOfExtrusion 8, Offset 9, Other 10 (default 11).
`bReverse = index(t1) < index(t2)`; when true, faces, types, the start-point list
(u/v pairs swapped, `:359-371`) and `myApprox1/myApprox2` are swapped (`:351-376`).
**Net effect: after this, `index(type(F1)) >= index(type(F2))`, i.e. the *simpler* surface
is always `F2`, and a plane paired with anything else is always `F2`.** The swap is undone
on the pcurves at `:550-563` (and at `:423-435` on the plane/plane path).

**(c) Tolerance seeds** (`:378-387`):
```
aFuzz  = myFuzzyValue / 2               // myFuzzyValue >= Precision::Confusion() = 1e-7, :230-233
myTolF1 = tol(F1) + aFuzz
myTolF2 = tol(F2) + aFuzz
myTol   = myTolF1 + myTolF2
TolArc  = TolTang = myTol
```

**(d) The UV rectangle.** `IntTools_Context::UVBounds` (`IntTools/IntTools_Context.cxx:
1028-1039`) returns `BRepAdaptor_Surface(face, Restriction=true)` bounds
(`IntTools_Context.cxx:327-339`), which is `BRepTools::UVBounds(F)` — **the bounding box
of the face's pcurves**, not the surface's natural domain
(`ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_Surface.cxx:56-81`, `:71-76`).

Then it is padded, three different ways (`IntTools_FaceFace.cxx:440-473`):

* plane paired with cylinder/cone/torus → `CorrectPlaneBoundaries` (`:3093-3111`):
  `u ± 0.1·(umax-umin)`, `v ± 0.1·(vmax-vmin)`, skipped for infinite bounds.
* everything else → `CorrectSurfaceBoundaries(face, myTol*2, u,v)` (`:2017-2206`):
  - returns immediately for doubly-nested `RectangularTrimmedSurface`/`OffsetSurface`
    (`:2036-2058`);
  - `enlarge` only for Bezier, BSpline, SurfaceOfExtrusion, SurfaceOfRevolution,
    **Cylinder** (`:2064-2069`) — *not* sphere, cone, torus, plane;
  - non-periodic direction: `umin -= delta` if `umin - uinf > delta`, else `umin = uinf`
    (clamped to the surface's natural bounds), symmetric for max (`:2072-2113`);
  - periodic direction: **shrink** to the box of the closed (seam) edges' pcurves
    (`:2116-2205`), requiring every closed edge to have a `Geom2d_Line` pcurve parallel to
    an axis, otherwise the correction is abandoned.

  **Unit wart, port honestly:** `delta = theTolerance = 2·myTol` is a **3D** tolerance
  added directly to **parameter** values (`:2031`, `:2077`). OCCT mixes units here. Our
  port must convert: `delta_u = 2·myTol / |dS/du|` at the patch centre (Law 2). This is a
  deliberate divergence and is listed in §6/M2.

**(e) Sampling.** `IntTools_TopolTool::ComputeSamplePoints`
(`IntTools/IntTools_TopolTool.cxx:78-394`), capped at 50 per direction (`:85`):
plane 10×10; cylinder/cone/sphere/torus: `nbsu = (usup-uinf)/aMaxAngle` with
`aMaxAngle = 2·acos(1 - 0.01/R)` (deflection 1e-2, `:148-152`, `:193-199`, `:257-279`),
floors 2 (cylinder) or 10 (cone/sphere/torus); BSpline: `nbKnots·degree`, min 4 then min
10, plus a pole-curvature sign-change analysis `Analyse` (`:454-564`) and an anisotropy
doubling when `((usup-uinf)/UResolution(1)) / ((vsup-vinf)/VResolution(1))` is ≥10 or ≤0.1
(`:345-358`).

**(f) Walker parameters** (`:482-490`):
```
UVMaxStep  = IntPatch_Intersection::DefineUVMaxStep(S1,dom1,S2,dom2)   // 0.001, or 0.0001
Deflection = 0.1        (0.01 when both surfaces are BSpline)
myIntersector.SetTolerances(TolArc, TolTang, UVMaxStep, Deflection)
```
`DefineUVMaxStep` (`IntPatch/IntPatch_Intersection.cxx:2372-2408`): 0.0001 iff a **singular
boundary** of one surface (a boundary along which `|dS/du|` or `|dS/dv|` stays below
`Precision::Confusion()`, detected by `CheckSingularPoints`, `:2286-2368`, 5 probes per
boundary) lies at distance in `(1e-7, 1e-5)` from the other surface. This is the
cone-apex / sphere-pole guard.

**(g) Cyl/torus tolerance shrink.** `Tolerances`, `:2787-2822`: if the pair is
cylinder×torus and `| R_cyl - r_minor | < 1e-7`, then `TolTang *= 0.1`.

**(h) Torus start-point purge.** `:492-500`: when **both** surfaces are analytic
(not BSpline/Bezier/Other) and either is a torus, the EF seed point list is cleared.

**(i) EF seed points.** `BOPAlgo_PaveFiller::GetEFPnts`
(`BOPAlgo/BOPAlgo_PaveFiller_6.cxx:2608-2688`) collects, for the pair `(nF1,nF2)`, every
edge/face interference point whose edge and opposite face both belong to the pair's
sub-shape closure, and expresses it as an `IntSurf_PntOn2S` (UV on both faces; the pcurve
value on the owning face, the projection on the opposite face). These are handed to the
PrmPrm walker as **guaranteed starting points** (`IntTools_FaceFace::SetList`, `:244-247`;
consumed at `IntPatch_Intersection::ParamParamPerfom`, `IntPatch_Intersection.cxx:
1674-1680`). **This is how OCCT guarantees the section reaches the face boundary in the
freeform case.** Our kernel has no EF stage at all, so it has no such seeds — the direct
cause of sections that stop short.

### 2.2 The three-way dispatch

`IntPatch_Intersection::Perform(S1,D1,S2,D2,TolArc,TolTang,ListOfPnts,isGeomInt,…)`,
`IntPatch/IntPatch_Intersection.cxx:1376-1655`.

**Step 1 — the `isGeomInt` gate** is computed by the caller:
`isTreatAnalityc(aBAS1,aBAS2,myTol)`, `IntTools_FaceFace.cxx:249-324`. It returns `true`
(= "analytic allowed") in every case except one:
* neither face is a plane → `true` (`:271`);
* the non-plane partner is not a cylinder → `true` (`:307`);
* the cylinder's V-range is infinite → `true` (`:280-283`, `:296-299`);
* otherwise it runs `IntAna_QuadQuadGeo(plane, cylinder, Tolang=1e-8, theTol, aHigh)`
  where `aHigh = Vmax-Vmin`; if the result is an **ellipse**, it returns
  `majorR < 100000 · minorR` (`:312-319`) — i.e. an ellipse with eccentricity beyond 1e5
  is refused and the pair is sent to PrmPrm; else it returns `inter.IsDone()`.

**Step 2 — cone/torus degeneracy pre-analysis** (`IntPatch_Intersection.cxx:1411-1546`),
only when one of the two is a Cone or a Torus. Produces two flags:
`TreatAsBiParametric` (force PrmPrm) and `bGeomGeom` (allow ImpImp for a torus).
* Cone present: `bToCheck = |semiAngle| < 0.02 || |semiAngle| > 1.55` (`:1429-1430`); if
  both are cones the same test is OR-ed for the second (`:1434-1436`); **exception**: two
  quasi-planar cones (`both |a| > 1.55`) with parallel axes whose apexes lie in a common
  plane normal to the axis (distance ≤ `Precision::Confusion()`) reset `bToCheck=false`
  (`:1438-1450`). `TreatAsBiParametric = bToCheck` (`:1453`).
* Torus present: `bToCheck = MajorRadius > MinorRadius`; torus×torus additionally passes
  when both radii pairs agree within `TolTang` (`:1462-1470`).
* If `bToCheck`, the *partner* surface decides `bGeomGeom` (`:1478-1539`):
  plane → `bGeomGeom=1` for a cone (with an extra guard: a near-cylindrical cone,
  `|semiAngle| < 0.02`, whose axis is nearly perpendicular to the plane normal,
  `|axis·planeNormal| < 0.015`, resets it to 0, `:1489-1496`); for a torus only when the
  axis is parallel to the plane normal or normal to it **and** the plane passes through the
  axis location; sphere → `bGeomGeom=1` iff the sphere centre lies on the cone/torus axis
  (`:1510-1516`); cylinder/cone/torus → `bGeomGeom=1` iff axes are parallel **and**
  coaxial within `Precision::Confusion()` (`:1532-1538`).
  `bGeomGeom==1` ⇒ `TreatAsBiParametric = false` (`:1541-1544`).
* Any infinite domain ⇒ `TreatAsBiParametric = false` (`:1549-1552`).
* If `TreatAsBiParametric`, both types are **relabelled `GeomAbs_BezierSurface`**
  (`:1554-1559`) purely to force the PrmPrm branch below.

**Step 3 — the type score.** (`:1562-1592`)
```
ts(type) = 1 for Plane, Cylinder, Sphere, Cone
         = bGeomGeom for Torus            // 0 unless the degeneracy analysis allowed it
         = 0 otherwise (Bezier, BSpline, Revolution, Extrusion, Offset, Other)
```

**Step 4 — dispatch** (`:1598-1622`):
```
!isGeomInt          -> ParamParamPerfom   (PrmPrm)
ts1 != ts2          -> GeomParamPerfom    (ImpPrm; the parametric one is the one with ts==0)
ts1 == ts2 == 0     -> ParamParamPerfom   (PrmPrm)
ts1 == ts2 == 1     -> GeomGeomPerfom     (ImpImp)
```

**Step 5 — WLine purge** (`:1629-1654`): for every produced WLine with purging enabled,
`IntPatch_WLineTool::ComputePurgedWLine` replaces it (removes near-duplicate and looping
points). ImpImp explicitly **disables** purging on its own WLines (`:1830`).

### 2.3 Path A — ImpImp (`IntPatch_ImpImpIntersection`)

`IntPatch/IntPatch_ImpImpIntersection.cxx:2517-2769`. `SetQuad` (`:3058-3091`) assigns
Plane 1, Cylinder 2, Cone 3, Sphere 4, Torus 5; `iTT = 10·iT1 + iT2`, `bReverse = iT1>iT2`.
The switch has **15 distinct arms** (all pairs of {plane,cyl,cone,sphere,torus}):

| iTT | routine | typical output |
|---|---|---|
| 11 | `IntPP` (`:3106-…`) | 1 `IntPatch_Lin`, or `SameSurf` |
| 12/21 | `IntPCy` | 1–2 lines, 1 circle, or 1 ellipse |
| 13/31 | `IntPCo` | circle / ellipse / parabola / hyperbola / 2 lines / apex point |
| 14/41 | `IntPSp` | 1 circle, or a tangency point |
| 15/51 | `IntPTo` | 1–2 circles (axis-parallel or axis-containing planes), else ALine |
| 22 | `IntCyCy` (`:7902…`) | 1–2 lines, 1–2 ellipses, or **WLine** (see below) |
| 23/32 | `IntCyCo` (`:8405…`) | conics when coaxial; else ALine |
| 24/42 | `IntCySp` (`:8137…`) | circles when the sphere centre is on the axis; else ALine |
| 25/52 | `IntCyTo` (`:9574…`) | circles when coaxial; else ALine |
| 33 | `IntCoCo` (`:8697…`) | 2 lines / conic / **ALine** in general pose |
| 34/43 | `IntCoSp` (`:9208…`) | circles when centred on the axis; else ALine |
| 35/53 | `IntCoTo` (`:9592…`) | ALine |
| 44 | `IntSpSp` (`:9489…`) | 1 circle |
| 45/54 | `IntSpTo` (`:9610…`) | circles; else ALine |
| 55 | `IntToTo` (`:9628…`) | circles (coaxial/equal); else ALine (spiric) |

**Key facts for the port:**
* Every arm goes through `IntAna_QuadQuadGeo` with `Tolang = 1e-8` (`:2551`) and
  `TolTang` as the distance tolerance.
* `IntCyCy` is the only arm that can return a **WLine directly** (numerically walked in
  the implicit×implicit setting); when it does, `isPostProcessingRequired = false`
  (`:2666-2674`) and the whole restriction pass below is skipped.
* **Restriction pass.** Unless `SameSurf`, for each surface the *other* quadric is
  intersected with that surface's domain boundary arcs:
  `AFunc.SetQuadric(quad2); AFunc.Set(S1); solrst.Perform(AFunc, D1, TolArc, TolTang)`
  (`:2786-2811`), and symmetrically (`:2823-2858`). The results are `pnt1/edg1`,
  `pnt2/edg2`. This is what produces `IntPatch_Point`s carrying `IsOnDomS1/IsOnDomS2` and
  `TransitionLineArc1/2` — **the entry/exit transitions the LineConstructor peels**
  (§2.6, Restriction branch).
* **Tangency.** `SameSurf || (all1 && all2)` where `allN = solrst.AllArcSolution() &&
  typs1==typs2` ⇒ `empt=false; tgte=true; slin.Clear(); spnt.Clear();`, plus
  `oppo = N1(P)·N2(P) < 0` at a type-dependent reference point (`:2865-2904`).
  `IntTools_FaceFace::Perform` then returns immediately with zero curves
  (`IntTools_FaceFace.cxx:536-540`), and `PerformFF` records an `InterfFF` with the flag
  but deliberately skips `myDS->AddInterf` (`BOPAlgo_PaveFiller_6.cxx:568-575`). The pair
  is then handled by the same-domain face stage (see `kb/audit_occt_ff-samedomain.md`).
* **ALine → WLine.** ALines never reach `MakeCurve`; `GeomGeomPerfom`
  (`IntPatch_Intersection.cxx:1816-1825`) converts every `IntPatch_Analytic` line with
  `IntPatch_ALineToWLine(S1,S2, aNbPointsInALine = 200)` (`:1807`, `:1816`).
  `IntPatch_ALineToWLine` constants (`IntPatch/IntPatch_ALineToWLine.cxx:150-153`):
  `myNbPointsInWline = 200`, `myTolOpenDomain = 1e-9`, `myTolTransition = 1e-8`,
  `myTol3D = Precision::Confusion() = 1e-7`. `MakeWLine` shrinks an open domain by
  `myTolOpenDomain` at each end (`:361-377`); vertex-merge tolerance
  `aTol = 2·myTol3D + Confusion`, parameter tolerance
  `aPrmTol = max(1e-4·(L-F), PConfusion)` (`:417-418`); step
  `(LPar - param)/(myNbPointsInWline - 1)` (`:451`). The resulting WLines are tagged
  `IntPatch_WLImpImp` (`:904`, `:909`, `:936`).
  `MakeCurve`'s `case IntPatch_Analytic:` is therefore an empty `break`
  (`IntTools_FaceFace.cxx:1171-1173`).
* **Cylinder×cylinder WLine join.** `IntPatch_WLineTool::JoinWLines(slin,spnt,S1,S2,
  TolTang)` for that pair only (`IntPatch_Intersection.cxx:1845-1848`).
* **Extend after ALine conversion.** `IntPatch_WLineTool::ExtendTwoWLines`
  (`:1896-1903`) with the two UV boxes enlarged by `PConfusion` (`:1852-1867`), the four
  periods (`:1869-1872`), and a list of *critical points*: the cone apex and/or the two
  sphere poles of each surface (`:1876-1894`). This is what closes the gap an ALine leaves
  at a pole/apex.

### 2.4 Path B — ImpPrm (`IntPatch_ImpPrmIntersection`)

`IntPatch_Intersection::GeomParamPerfom`, `IntPatch_Intersection.cxx:1909-2000`.
* Optional start point set on the parametric side (`:1918-1928`).
* Both domains infinite ⇒ try `FUN_PL_Intersection` (collinear plane/extrusion special
  case, `:1932-1956`), else trim both surfaces to a 1e+5 box via `FUN_TrimBothSurf`
  (`:1959-1962`); otherwise call directly (`:1967`):
  `interip.Perform(S1,D1,S2,D2, myTolArc, myTolTang, myFleche, myUVMaxStep)`.
* Output: `IntPatch_WLine`s tagged `IntPatch_WLImpPrm`
  (`IntPatch/IntPatch_ImpPrmIntersection.cxx:1074`, `:2973`, `:3589`), plus points; the
  **non-Walking lines are appended before the Walking ones** (`:1977-1993`).
* **Known limitation OCCT documents in-source (do not "fix" it silently):** the ImpPrm
  intersector "does not respect the domain of the quadric surface (it takes into account
  the domain of the parametric surface only)", so a 2-point WLine can have one point
  outside the quadric's domain — see the comment and the compensating rule at
  `GeomInt/GeomInt_LineConstructor.cxx:183-225` (fix #29972).

### 2.5 Path C — PrmPrm (`IntPatch_PrmPrmIntersection`)

`IntPatch_Intersection::ParamParamPerfom`, `IntPatch_Intersection.cxx:1659-1774`.
* Both domains finite (the normal B-Rep case):
  if the start-point list is non-empty, run once **with** the points and `ClearFlag=false`,
  then run again without (`:1673-1680`) — i.e. seeded walking first, then the generic
  seeding, accumulating.
  `interpp.Perform(S1,D1,S2,D2, TolTang, TolArc, myFleche, myUVMaxStep [, ListOfPnts])`.
* One domain infinite: build a trimmed surrogate of the infinite one from the finite one's
  3D box (`FUN_GetMinMaxXYZPnt` / `FUN_TrimInfSurf`, `:1682-1709`).
* Both infinite: `FUN_PL_Intersection` collinear check, else `FUN_TrimBothSurf(…,1e+8,…)`
  (`:1712-1749`).
* Output: `IntPatch_WLine` tagged `IntPatch_WLPrmPrm`; again **non-Walking first, then
  Walking** (`:1758-1772`). Failure mode: `interpp.IsDone()==false` leaves `done=false`,
  and `IntTools_FaceFace::Perform` returns with `myIsDone=false`; `PerformFF` then records
  an empty `InterfFF` and emits `AddIntersectionFailedWarning`
  (`BOPAlgo_PaveFiller_6.cxx:545-553`). **A failed FF pair is a warning, not an error, and
  the boolean continues with the section missing.** Port the diagnostic, not the silence.

### 2.6 Line types, carriers, and how each is trimmed to the rectangle

`IntPatch_IType`: `Lin`, `Circle`, `Ellipse`, `Parabola`, `Hyperbola` (all carried by
`IntPatch_GLine`), `Analytic` (`IntPatch_ALine`), `Walking` (`IntPatch_WLine`),
`Restriction` (`IntPatch_RLine`).

`GeomInt_LineConstructor::Perform(L)`,
`GeomInt/GeomInt_LineConstructor.cxx:114-670`. **One tolerance for the whole routine:**
`Tol = Precision::PConfusion()·35 = 3.5e-8` (`:118`). It fills `seqp` with pairs
`(first,last)` — the kept spans. Four branches:

**(1) `IntPatch_Analytic`** (`:121-151`) — for each consecutive vertex pair with
`firstp != lastp`, evaluate the ALine at the midpoint `(f+l)/2`, get UV on both quadrics by
`IntSurf_Quadric::Parameters` (`:820-862`), `AdjustPeriodic` (`:737-816`, period hard-coded
`2π` for cyl/cone/sphere U and torus U,V), then keep iff
`dom1->Classify(u1,v1,Tol) != OUT && dom2->Classify(u2,v2,Tol) != OUT`.

**(2) `IntPatch_Walking`** (`:152-329`) — per consecutive vertex pair (indices into the
point array):
  * if `lastp != firstp + 1`: classify the **integer midpoint** point
    `WLine->Point((int)((firstp+lastp)/2))` (`:164-179`);
  * if `lastp == firstp + 1` (a 2-point block) **and** `GetCreatingWay() == WLImpPrm`:
    classify the **interpolated average** of the two endpoints' UV (`:183-225`) — the
    #29972 rule;
  * otherwise (2-point block, ImpImp or PrmPrm): **both** endpoints must classify
    non-OUT (`:226-252`).
  * Post-pass (`:265-326`): if `>1` part **and** the pair is plane × (extrusion|revolution),
    the parts are merged by an XOR-on-index trick that collapses N touching parts into one
    (comment: "one resulting curve consists of 7 segments … the other reason is
    `TolReached3D=49`").

**(3) GLine, non-Restriction** (`:332-386`):
  * `Circle`/`Ellipse` → `TreatCircle` (`:674-733`): reject micro conics
    (`RejectMicroCircle`, `:895-915`: radius (circle) or major radius (ellipse) `< Tol`),
    put every vertex parameter into `[0,2π)` (`ElCLib::InPeriod`, `:60`), sort, append a
    wrap-around copy of the first vertex at `+2π` (`:694-695`), drop duplicates within
    `1000·PConfusion = 1e-6` (`RejectDuplicates`, `:926-983`, marking rejects with
    `RealLast()`), re-sort, then classify each interval's **midpoint**.
  * `Lin`/`Parabola`/`Hyperbola` → classify each vertex-interval midpoint when
    `|firstp-lastp| > PConfusion` (`:350-374`). If **no** interval was testable
    (`!intrvtested`) the whole line is kept a priori (`:376-382`) — the decision is
    deferred to `MakeCurve`'s ±100 probe (§2.7).

**(4) `IntPatch_Restriction`** (`:388-669`) — the only branch that uses **transitions**
rather than midpoints. Each vertex contributes an orientation pair
`(or1,or2)` from `TransitionLineArc1/2().TransitionType()`:
`IntSurf_In → FORWARD`, `IntSurf_Out → REVERSED`, `Touch|Undecided → INTERNAL`; a vertex
not on that domain gives `INTERNAL` (`:403-451`). Vertices closer than `Tol` in parameter
are merged, contradicting orientations collapsing to `INTERNAL` (`:453-505`). Then the
in/out state is initialised from the first non-INTERNAL orientation on each side
(`dansS1 = (or1 != FORWARD)`, `:514-523`; symmetric for S2), with a fallback that
classifies the first vertex not on that domain and gives up if it is OUT (`:525-543`,
`:556-573`). If no vertex carries a transition at all (`!trim`), the whole line is kept
(`:575-581`). Otherwise the sequence is peeled: while both `dansS1 && dansS2`, a
`REVERSED` closes the current span at that parameter; while not both, a `FORWARD` reopens
it (`:589-656`); a trailing open span is closed at `thelast` (`:659-668`).
`nbvtx == 0` ⇒ keep everything (`:391-398`).

**Boundary graze at this level:** `Classify` returns `TopAbs_ON` when the point is within
`Tol` of the rectangle border, and every test is `!= TopAbs_OUT`. **A section running
exactly along the padded UV border is kept.** That is deliberate; the drop, if any, happens
at L3.

### 2.7 `MakeCurve` — converting a kept span into an `IntTools_Curve`

`IntTools_FaceFace::MakeCurve(Index, dom1, dom2, theToler)`,
`IntTools/IntTools_FaceFace.cxx:695-1846`. Locals: `TOLCHECK = 1e-7`,
`TOLANGCHECK = 1e-6` (`:709-710`), `Tolpc = myTolApprox` (`= 1e-7`, set by `PerformFF` at
`BOPAlgo_PaveFiller_6.cxx:329`). `bAvoidLineConstructor` is set **only** for
`IntPatch_Restriction` (`:748-751`); the `IntPatch_Walking` block at `:724-744` computes
`P1.SquareDistance(P2) < 1e-14` and then assigns `bAvoidLineConstructor = false` — a
**dead assignment** in current OCCT; do not port it.

**(a) Lin / Parabola / Hyperbola** (`:781-901`). Carrier: `Geom_Line`, `Geom_Parabola`,
`Geom_Hyperbola` built from the `IntPatch_GLine`. For each kept part `[fprm,lprm]`:
  * both finite → `Geom_TrimmedCurve(newc,fprm,lprm)`; parabola gets
    `tol = IntTools_Tools::CurveTolerance(aCT3D, myTol)` (`IntTools_Tools.cxx:430-464`,
    which scales `myTol` by `sqrt(0.5·x/focal)` at each end and takes the max —
    `ParabolaTolerance`, `:473-544`). pcurves via
    `GeomInt_IntSS::BuildPCurves(fprm,lprm,Tolpc,surface,newc,C2d)` trimmed to the same
    range; **if a requested pcurve comes back null the whole part is skipped**
    (`:826-846`).
  * one or both infinite → probe at `lprm-100`, `fprm+100`, or
    `IntermediatePoint(-100,100)` (`:854-869`); for extrusion/offset/revolution surfaces
    the untrimmed curve is kept unconditionally with no pcurves (`:873-882`); otherwise
    keep iff both `dom->Classify(uv, Precision::Confusion())` are non-OUT (`:884-897`).

**(b) Circle / Ellipse** (`:906-1169`). Carrier `Geom_Circle` / `Geom_Ellipse`.
  * Each part crossing 0 is split into `[fprm,2π]` and `[0,lprm]` after normalising into
    `[0,2π]` (`:926-1011`). A piece shorter than `Tolpc` is dropped **unless** its two
    endpoints are more than `myTol` apart in 3D, in which case
    `ParameterOutOfBoundary` (`:2213-2304`) walks the parameter in steps
    `adelta = max(0.1·myTol, Precision::Confusion())` away from the boundary until the
    projected point is no longer `TopAbs_ON` for either face, capped at **11 iterations**
    (`:2276-2281`), clamped to the other endpoint (`:2288-2301`).
  * For each resulting `[fprm,lprm]`: if it is *not* the full `[0,2π]`
    (`|fprm|>RealEpsilon || |lprm-2π|>RealEpsilon`), emit a `Geom_TrimmedCurve` and build
    pcurves with the 8-argument `BuildPCurves` that passes the surface's UV box
    (`:1030-1063`). If it *is* the full period and it is the only part, emit the whole
    circle (`:1073-1101`). Otherwise fall back to **18 probes** at `j·2π/17`, `j=0..17`,
    keeping the whole (untrimmed) conic on the first probe that classifies non-OUT on both
    domains at `Tol = Precision::Confusion()` (`:1104-1165`).

**(c) Analytic** — empty (`:1171-1173`); ALines were converted in §2.3.

**(d) Walking** (`:1175-1740`).
  * `!myApprox` (never in the BOP pipeline; `PerformFF` sets `bApprox` from the section
    attribute): raw degree-1 BSpline through the WLine points,
    `GeomInt_IntSS::MakeBSpline/MakeBSpline2d` (`GeomInt/GeomInt_IntSS_1.cxx:1452-1501`).
  * `myApprox` (the normal path): 
    - `aTolApproxImp = 1e-5`, `tol2d = myTolApprox` (`:1237-1238`).
    - cylinder×sphere pcurve refusal: `ApproxWithPCurves(cyl,sph)` (`:2357-2422`) returns
      false when a sphere apex lies on the cylinder within `1e-7` of `R_cyl²`
      (`:2362-2389`), or when `R_cyl >= 2·R_sph` and the sphere centre is within `0.2·R_sph`
      of the cylinder wall on the "wrong" side (`:2392-2420`). Refusal ⇒
      `myTolApprox = 1e-5`, no pcurves (`:1254-1261`).
    - `IntTools_WLineTool::DecompositionOfWLine(...)` — see §2.8.
    - approximation parameters: `ApproxParameters` (`:2736-2783`) gives
      `degMin=4, degMax=8, nbIter=0`; cylinder×torus with `|R_cyl - r_minor| < 1e-7` →
      `degMax=6`; cylinder×cylinder → `nbIter=1`. `myHS1==myHS2` (self-intersection) or a
      re-approx pass uses `(4,8,0,30,false)` (`:1330-1349`). Parametrisation type from
      `ApproxInt_KnotTools::DefineParType` (`:1328-1329`).
    - **plane special case**: when either surface is a plane, the 3D approximation is
      switched off and the curve is *lifted* from the 2D approximation on the plane via
      `ElSLib::Value(u,v,Pln)` (`:1354-1361`, `:1427-1449`, `:1513-1529`).
    - BSpline×BSpline: `SetParameters(...,4,8,0,30,true,…)`, and if
      `IntTools_WLineTool::NotUseSurfacesForApprox` says an end point lies in a degenerated
      zone of either face (`IntTools_WLineTool.cxx:227-248`), retry with the surfaces
      disabled (`:1367-1383`).
    - Validity checks and the **re-approximation loop**: `IsCurveValid(pcurve)`
      (`:2308-2353`, a 2D self-intersection test with `tolint = 1e-10`) and
      `CheckPCurve(pcurve, face, ctx)` (`:3010-3089`: 23 samples per continuity interval,
      `tolU = max(0.01·(umax-umin), Confusion)`, periodic re-centring on the mid point);
      failure sets `myTolApprox = 1e-5; tol2d = myTolApprox; reApprox = true; goto reapprox`
      (`:1725-1731`). **The loop runs at most twice**, because `reApprox` is checked before
      every `goto` (`:1467`, `:1493`, `:1553`, `:1579`, `:1671`, `:1706`). Port it as a
      bounded `for (int pass = 0; pass < 2; ++pass)`, not as a goto — an unbounded
      transcription hangs.
    - When a plane-side pcurve fails `CheckPCurve`, OCCT falls back to the *unapproximated*
      degree-1 BSpline pair and, if that also fails, keeps the approximated one anyway
      (`:1593-1629`). Documented fallback; port it verbatim.
    - `GeomLib_CheckBSplineCurve` / `GeomLib_Check2dBSplineCurve` with
      `(TOLCHECK=1e-7, TOLANGCHECK=1e-6)` + `FixTangent(true,true)` on every produced
      spline (`:1450-1451`, `:1463-1464`, `:1489-1490`, `:1535-1536`, `:1549-1550`,
      `:1650-1651`, `:1668-1669`, `:1703-1704`).
    - Curve tolerance seed `aTolC`: `Precision::Confusion()` when the WLine was decomposed
      (`:1277-1281`), else 0; raised to `1e-6` for plane×torus (`:1411-1421`).

**(e) Restriction** (`:1742-1841`). `GeomInt_IntSS::TreatRLine` produces `(c3d,c2d1,c2d2,
tolReached)`. The curve's parameter set starts as `{first,last}` and is enriched by
`GeomInt_IntSS::TrimILineOnSurfBoundaries(c2d1,c2d2,box1,box2,params)`
(`GeomInt/GeomInt_IntSS_1.cxx:1333-1448`), which intersects each pcurve with the **four
boundary lines** of its surface's UV box at `anIntTol = 10·Precision::Confusion() = 1e-6`
(`:1441`) and sorts the parameters. The boxes are then enlarged by `theToler` (= `TolArc`)
(`:1781-1782`), and each consecutive parameter pair is kept iff its **midpoint** pcurve
values are inside both enlarged boxes (`:1802-1834`). Sub-`PConfusion` intervals are
skipped with a coalescing trick (`:1791-1800`).

### 2.8 `DecompositionOfWLine` — splitting a WLine at seam/boundary touches

`IntTools/IntTools_WLineTool.cxx:492-1312`. Purpose: a walked line that touches a periodic
seam must be cut there, and the touching points must be re-computed exactly, otherwise the
approximation smears across the seam.
* `aTol = 0.5·Precision::Confusion() = 5e-8` (`:533-534`).
* Pass 1 (`:541-642`): classify every WLine point as "on a boundary" — for each surface and
  each **periodic** direction, adjust the parameter into `[min,max]`
  (`GeomInt::AdjustPeriodic`) and test `|p - min| < UResolution(aTol)` or
  `|p - max| < …` (`IsPointOnBoundary`, `:258-286`). Runs of equal flag become segments
  (`anArrayOfLines`, `anArrayOfLineType`). Fewer than 2 runs ⇒ return false, no
  decomposition (`:639-642`).
* Pass 2 (`:646-1095`): for each *interior* run, recompute its two end points. Counts how
  many boundaries the neighbouring point sits on (`nbboundaries`), including a
  "near-boundary" band `min(100·resolution, 0.1·span)` (`:759-769`). Depending on the
  count it either mirrors the parameter across the period (`anotherPar`, `:799-803`) with
  an angle test (`|angle| < π/4` and positive dot, `:875-901`), or calls `FindPoint` to
  intersect the chord with the UV box (`:1006`). The candidate is validated by projecting
  its 3D image onto the **other face** (`aContext->ProjPS`) and requiring
  `LowerDistance < theTol` (`= myTol`) (`:1012-1027`), then re-adjusted by period against
  the neighbour (`AdjustByNeighbour`, `:292-336`), with a second projection round-trip if
  it still falls outside the box (`:1050-1075`).
* Pass 3 (`:1098-1308`): re-emit the runs, clipped to each LineConstructor part
  `[ifprm,ilprm]`, inserting the recomputed boundary points at the ends. Each emitted
  `IntPatch_WLine` inherits `SetCreatingWayInfo(theWLine->GetCreatingWay())`.

### 2.9 `ComputeTolReached3d` — the measured curve tolerance

`IntTools_FaceFace.cxx:613-691`. For each produced `IntTools_Curve`:
```
tolC = aIC.Tolerance()
for side in {1,2}:
    if pcurve[side] exists:
        IntTools_Tools::ComputeTolerance(c3d, pcurve, surface, first, last, d, t,
                                         Precision::PConfusion(), runParallel)
        tolC = max(tolC, d)
    else:
        tolC = max(tolC, FindMaxDistance(c3d, first, last, face[side], ctx))
aIC.SetTolerance(tolC)
aIC.SetTangentialTolerance(max(aIC.TangentialTolerance(), max(tolF1,tolF2)))
```
* `ComputeTolerance` (`IntTools_Tools.cxx:737-779`) wraps
  `GeomLib_CheckCurveOnSurface` and multiplies the answer by `1 + 1e-5` as a margin
  (`:774-775`).
* `FindMaxDistance` (`IntTools_FaceFace.cxx:2899-2933`): 11 equal sub-intervals, and on
  each a **golden-section maximisation** (`:2937-2988`, `Cf = 0.6180339887…`) of the
  point-to-face projection distance, converging to `anEps = 1e-4 · dt`.
* Comment at `:682-689`: tangential tolerance is only really computed for plane/plane; for
  everything else it is just `max(tolF1,tolF2)`. Honest fallback — port it and note it.

### 2.10 `PerformPlanes` — the plane/plane fast path (the tolerance model to copy)

`IntTools_FaceFace.cxx:2426-2558`. `IntAna_QuadQuadGeo(pln1,pln2,TolAng=1e-8,TolTang)`;
`IntAna_Same` ⇒ tangent; `IntAna_Empty` ⇒ nothing. Otherwise project the 3D line onto both
planes (`ProjLib_Plane`), clip each 2D line against that plane's **padded UV rectangle**
with `ClassifyLin2d` (`:2574-2731`, an explicit 4-edge walk with `INTER`/`COINC` predicates
at `:2562-2572`), intersect the two parameter ranges, require `pmax-pmin > TolTang`.
Then:
```
tol         = max(TolF1, TolF2)
aDt         = IntTools_Tools::ComputeIntRange(TolF1, TolF2, angle(n1,n2))
tangentTol  = sqrt(aDt² + TolF1²)
```
`ComputeIntRange` (`IntTools_Tools.cxx:783-804`):
`|π/2 - θ| < Precision::Angular()` ⇒ `tol2`; else with `θ' = min(θ, π-θ)`,
`tol1·tan(π/2-θ') + tol2/sin(θ')`. **This is the correct model for how far a section is
"uncertain" along itself at a shallow angle** — port it and use it for curved pairs too
(OCCT does not, and that is a known gap, see `:682-689`).

### 2.11 `PrepareLines3D` and the FF hand-off

`IntTools_FaceFace.cxx:1932-2013`, called by `PerformFF` with `bToSplit = false`
(`BOPAlgo_PaveFiller_6.cxx:331`, `:558`). So in the boolean pipeline:
* closed curves are **not** split in half (`IntTools_Tools::SplitCurve`,
  `IntTools_Tools.cxx:191-250`, is dead in this path — but `PutClosingPaveOnCurve` later
  handles closure instead);
* plane×cone producing exactly 4 curves whose first is a `GeomAbs_Line` runs
  `IntTools_Tools::RejectLines` (`:106-160`), which keeps only the first line plus the
  first line whose direction is not "coincident" with it
  (`IsDirsCoinside`, `dLim = 2e-4` on the direction-point distance, `:164-173`).

`PerformFF` then, per produced curve (`BOPAlgo_PaveFiller_6.cxx:591-609`):
```
IntTools_Tools::CheckCurve(aIC, aBox)          // IntTools_Tools.cxx:549-575
    -> 3D curve non-null, box built with max(tol, tangTol),
       rejected iff aBox.IsThin(3·Precision::Confusion() = 3e-7)
aBox.Enlarge(aBoxExpandValue)                  // aTolFF + max vertex tolerance of both faces
aNC.SetTolerance(max(aIC.Tolerance(), aTolFF))
```
with `aTolFF = max(shiftValue, ToleranceFF(bas1,bas2))`, `ToleranceFF`
(`BOPAlgo_PaveFiller_6.cxx:3922-3942`) `= max(tolF1,tolF2)`, floored at **5e-6** when
either surface is *not* one of plane/cyl/cone/sphere/torus. `shiftValue` comes from the
seam-shift heuristic (`:393-486`): when a **closed (seam) edge** of one face and an edge of
the other face have an EE intersection whose exact points on the two edges differ by more
than the vertex tolerance, one face is *translated* by that difference before intersecting,
and the shift magnitude becomes the floor of the curve tolerance.

### 2.12 The real wire trimming: `MakeBlocks`

`BOPAlgo_PaveFiller::MakeBlocks`, `BOPAlgo_PaveFiller_6.cxx:649-1137`. Per FF pair:

**(1) Gather the face pair's boundary state** (`:770-771`):
`SubShapesOnIn(nF1,nF2, aMVOnIn, aMVCommon, aMPBOnIn, aMPBCommon)` — the vertices and pave
blocks that are ON or IN either face; `SharedEdges(nF1,nF2,aLSE)`.

**(2) Points.** Each isolated section point becomes a new vertex unless
`IsExistingVertex(P, aTolFF, aMVOnIn)` (`:1950-1984`; `ComputeVV` with
`tol = tolFF + fuzz`).

**(3) Paves.** In this exact order (`:794-851`):
   * `GetStickVertices(nF1,nF2, aMVStick, aMVEF, aMI)` (`:2847-2905`) — every new vertex
     produced by VV/VE/EE/VF (into `aMVStick`) and by EF (into both `aMVStick` and
     `aMVEF`), restricted to the pair's sub-shape closure.
   * `PutPavesOnCurve` (`:2372-2421`): EF vertices first with `iCheckExtend=2`, then all
     ON/IN vertices with `iCheckExtend=1` (non-common ones pre-filtered by bounding box and
     required to be `IsNewShape`).
     `PutPaveOnCurve` (`:2959-3068`): `IsVertexOnLine(V, tolV, curve, tolR3D + fuzz, t)`
     (`IntTools_Context.cxx:775-…`: the sum tolerance is **doubled** and floored at `1e-5`
     for BSpline/Bezier curves, `1e-6` otherwise, `:793-808`); on failure and
     `iCheckExtend`, retry with `ExtendedTolerance` (`:2542-2604`, the radius of the EE/EF
     common part). Accepted paves either reuse an existing pave within
     `Resolution(max(tolR3D,tolV))` (grouping the two vertices for later SD fusion in
     `aDMVLV`) or append a new ext-pave and **grow the vertex tolerance** to
     `dist + DTolerance (1e-12)` (`:3040-3065`).
   * `FilterPavesOnCurves` (`:2437-2538`): a vertex bound to several curves is removed from
     those where `dist² > 100·max(tol², minDist²)` **and** the projection angle sine is
     below `0.5`; the vertex tolerance is then reduced to the largest kept distance.
   * `PutStickPavesOnCurve` (`:2748-2843`): only for curve ends without a vertex; a stick
     vertex within `sqrt(2e-7)` of a curve end whose two surface normals are parallel to
     `1 - |n1·n2| <= 5e-9` (the "creasing criteria") is paved there.
   * `PutEFPavesOnCurve` (`:2692-2744`): only when the pair produced exactly one curve and
     the curve is a BSpline/Bezier; projects unused EF vertices onto it.
   * `PutBoundPaveOnCurve` (`:2308-2368`): for each curve end that has no vertex yet, and
     **only if `IsValidPointForFaces(P, F1, F2, tolR3D)`** (a real wire test, §2.13), create
     a new vertex with tolerance `tolR3D`, grown by `UpdateVertex` to cover the curve
     (`BOPTools_AlgoTools_2.cxx:80-98`). A closed curve gets only one bound
     (`:2324-2339`).
   * `PutClosingPaveOnCurve` (`:3500-3605`): if a pave sits at one end within `PConfusion`
     and the opposite bound point is within `tolV + tolP` of it, a second pave with the same
     vertex index is added at the opposite parameter — provided
     `BRepLib::FindValidRange` still yields a non-empty range with the inflated tolerance.

**(4) Blocks.** `aPB1->Update(aLPB,false)` builds blocks **between consecutive ext paves
only** (`BOPDS/BOPDS_PaveBlock.cxx:249-311`, invoked at `:892`). **The curve's own ends are
not paves** — so a curve with fewer than 2 ext paves yields **no** section edge, and the
parts outside the extreme paves are discarded, never extrapolated. That situation sets
`isToRecheck` and the FF pair is queued for a **second pass** (`:879`, `:894-897`,
`:1067-1071`) after other pairs have created more vertices.

**(5) Per-block acceptance** (`:899-1063`), in order:
   1. `|t1-t2| < PConfusion` ⇒ skip (`:906-909`).
   2. **`IsValidBlockForFaces(t1,t2,curve,F1,F2,tolR3D)`** ⇒ the wire test (`:914-918`).
   3. `IsExistingPaveBlock(pb, curve, aLSE, nEOut, tolNew)` (`:1988-2043`) — the block's
      43.2 % point projects onto a **shared edge** of the two faces within
      `max(tolE, max(tolV1,tolV2)) + fuzz`; then that edge's tolerance is raised and the
      block is dropped (no new edge). This is graze case (i).
   4. `BRepLib::FindValidRange` with `tolR3D` and each vertex's tolerance; failure ⇒ the
      block is a **micro** block: it is recorded in `aMicroPB`, its two vertices are queued
      for fusion, and no edge is made (`:936-960`).
   5. `IsExistingPaveBlock(pb, curve, tolR3D, aMPBOnIn, tree, aMPBCommon, aPBOut, tolNew)`
      (`:2047-2251`) — the block coincides with an **existing ON/IN pave block of either
      face**. The search is a BVH over those blocks' boxes; the acceptance uses the
      43.2 % point plus both ends, with `aRealTol = tolR3D + fuzz`, doubled for common
      blocks with a face (`:2147-2156`), and, when both ends already share vertices with
      the candidate, extended to `2·min(0.001, 10·tolCheck)` if the tangents agree to
      `|cos| >= 0.9063` (25°) (`:2157-2198`). The nearest candidate wins. The existing edge
      is then attached to the face that did not own it (`aPBFacesMap`), its tolerance is
      raised, the section block's own vertices are queued for fusion, and
      `PreparePostTreatFF` registers the *existing* edge as if it were a section edge
      (`:962-1021`). This is graze case (ii) — and it is exactly guarantee **G10**.
   6. Otherwise: **mint the edge** —
      `BOPTools_AlgoTools::MakeEdge(curve, V1, t1, V2, t2, tolR3D, E)`
      (`BOPTools/BOPTools_AlgoTools.cxx:1729-1746`): both vertices are first grown to
      `tolR3D + DTolerance(1e-12)`, the edge is built by
      `BRepBuilderAPI_MakeEdge(c3d,V1,V2,t1,t2)` with the range forcibly restored
      (`BOPTools_AlgoTools_2.cxx:102-120`), then `UpdateEdge(E, tolR3D)`.
      Then `BOPTools_AlgoTools::MakePCurve(E, F1, F2, curve, bPC1, bPC2, ctx)`
      (`BOPTools_AlgoTools.cxx:1657-1725`): use the `IntTools_Curve` pcurve if present, else
      build one by projection (`BOPTools_AlgoTools2D::BuildPCurveForEdgeOnFace`), then
      `AdjustPCurveOnFace` (periodic-aware), `UpdateEdge`, and finally **one**
      `BRepLib::SameParameter(E)` for the whole edge (`:1724`).
   7. `ProcessExistingPaveBlocks(...)` (`:3072-3167`) then pulls in any ON/IN pave block of
      one face whose distance to the new section edge is `<= tolE_section + tolE_other`
      (using pre-computed edge-range distances `myDistances`), so that it participates in
      the fusion too.

**(6) End of the curve loop:** `aLPBC.RemoveFirst()` removes the workspace pave block
(`:1065`). Unused paves' vertex tolerances are rolled back (`:1075-1095`).

### 2.13 The wire predicates (this is "in/on/out" for real)

`IntTools/IntTools_Context.cxx`:
* `FClass2d(F)` (`:225-243`): `IntTools_FClass2d(F.Forward(), BRep_Tool::Tolerance(F))`,
  cached, orientation-insensitive key.
* `IntTools_FClass2d::Perform(P2d, RecadreOnPeriodic)`
  (`IntTools/IntTools_FClass2d.cxx:637-804`): first a fast winding test over the
  pre-tessellated wires (`TabClass(n).SiDans`, `:688-724`); when that is indecisive it falls
  back to `BRepClass_FClassifier` on a `BRepClass_FaceExplorer` with
  `aFCTol = min(surf->UResolution(tolF), surf->VResolution(tolF))` when the point is inside
  (or outside) both UV ranges, else the resolution of the direction that is out of range
  (`:729-756`). **The classifier band is a 3D tolerance converted through the surface
  metric — never a raw parameter epsilon.** Periodic faces retry over period shifts
  (`:758-802`).
* `StatePointFace(F,P2d)` → that state (`:594-600`).
* `IsPointInFace` (`:604-608`): `state != OUT && state != ON` — **ON is OUT**.
* `IsPointInOnFace` (`:639-643`): `state != OUT` — **ON is IN**.
* `IsValidPointForFace(P3d,F,tol)` (`:647-673`): `ProjPS(F).Perform(P)`; not done ⇒ false;
  `LowerDistance > tol` ⇒ false (strict `>`, so `dist == tol` passes); then
  `IsPointInOnFace` on the projection parameters. `ProjPS` is built once per face over the
  **face's UV box** with projector tolerance `myPOnSTolerance = 1e-12` and
  `Extrema_ExtFlag_MIN` (`:247-265`, `:65`).
* `IsValidPointForFaces` (`:677-691`): both faces, short-circuit.
* `IsValidBlockForFaces(t1,t2,curve,F1,F2,tol)` (`:717-755`):
  ```
  tm = IntermediatePoint(t1,t2)          // 0.43213918 of the way, NOT the midpoint
  P  = c3d->Value(tm)
  for side in {1,2}:
      if pcurve[side] exists:  ok = IsPointInOnFace(F[side], pcurve[side]->Value(tm))
      else:                    ok = IsValidPointForFace(P, F[side], tol)
  ```
  **The pcurve shortcut is the whole reason this is cheap and exact**: when the section
  carries a pcurve, no projection happens at all — the 2D point is classified directly
  against the wires.

**Boundary graze, complete answer:**
1. surface tangency ⇒ typed `TANGENT`, zero curves, handled by the SD-face stage
   (`IntTools_FaceFace.cxx:536-540`; `BOPAlgo_PaveFiller_6.cxx:568-575`);
2. curve along the padded UV rectangle border ⇒ **kept** at L2 (`Classify != OUT`,
   `Tol = 3.5e-8`);
3. block whose 43.2 % point lands exactly on a wire ⇒ **kept** at L3
   (`IsPointInOnFace` accepts `TopAbs_ON`);
4. block that coincides with a shared edge ⇒ that edge is reused, its tolerance raised, no
   new edge (`IsExistingPaveBlock` #1);
5. block that coincides with any ON/IN pave block of either face ⇒ that block is reused and
   attached to the second face (`IsExistingPaveBlock` #2);
6. block shorter than its own vertices' tolerance spheres ⇒ dropped as micro, the two
   vertices fused in PostTreatFF;
7. block whose 43.2 % point is OUT of either face ⇒ dropped, **and nothing is bisected**:
   the surviving neighbours' endpoints are unchanged, because they are real paves.

### 2.14 PostTreatFF — the cross-pair fusion (Law 1 for section curves)

`BOPAlgo_PaveFiller_6.cxx:1165-1669`. Full mechanics, traps and constants are in
`kb/audit_occt_ff-samedomain.md` §2E/§3 — **read that; it is audited and this document does
not restate it**. The five facts that matter for the port:

1. **The mechanism is a nested boolean.** All new section edges (`aLS`) plus the existing
   edges (packed into ONE compound so they are not intersected against each other,
   `:1280-1316`) plus the SD-candidate vertices, micro-block vertices, rejected-block
   vertices and "unused" vertices are fed to a **second `BOPAlgo_PaveFiller`**
   (`:1195-1197`, `:1390-1398`). That filler performs VV/VE/EE among the section edges
   themselves and returns a DS in which coincident section edges are one common block and
   coincident endpoints are one SD vertex.
2. **Read-back rewires the main DS** (`:1408-1656`): each original section pave block is
   replaced by the nested DS's pave blocks; the nested DS's edge indices are appended to the
   main DS once (`aMEPB`, `:1405`, `:1628-1641`), so **one DS edge ⇒ one pave block object
   shared by every curve that produced it**. That is the fusion.
3. **Fast path** (`:1237-1276`): exactly one section shape, no micro blocks, no rejected
   blocks, no unused vertices ⇒ skip the nested filler entirely.
4. **Failure is fatal to the whole section stage** (`:1393-1397` → `:1121-1124`). Our port
   must degrade to "no cross-pair fusion" instead.
5. **Downstream ordering is fixed** (`:1110`→`:1113`→`:1114`→`:1126`→`:1129`→`:1131`→
   `:1136`): `RemoveMicroSectionEdges` → `MakeSDVerticesFF` → `PostTreatFF` →
   `CorrectToleranceOfSE` → `UpdateFaceInfo` → `UpdatePaveBlocks` → `PutSEInOtherFaces`.
   `UpdateFaceInfo` (`:1673-1946`) adds every surviving section pave block to **both**
   generating faces' `PaveBlocksSc` (`:1733-1734`) and merges blocks sharing a DS edge into
   a common block (`:1767-1858`). `PutSEInOtherFaces` (`:4277-4304`) then intersects every
   section edge against the faces that did **not** create it — this is how a section from
   `(F1,F2)` gets attached to `F3`.

---

## 3. DATA STRUCTURES AND C++ DECLARATIONS FOR OUR PORT

New files: `src/ff_section.h/.cpp` (L1/L2 + MakeCurve), `src/ff_blocks.h/.cpp`
(L3 + fusion). Both sit under `brep_bds.h` from `kb/ARCHITECTURE_v2.md` §1 and use its
`BdsVertex/BdsPave/BdsPaveBlock/BdsCommonBlock/BdsFaceInfo`.

```cpp
#pragma once
#include "point.h"
#include "vector.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include "brep.h"
#include <vector>
#include <optional>

namespace session_cpp {

// ---------------------------------------------------------------- L1 carriers

enum class SectionLineType { Line, Circle, Ellipse, Parabola, Hyperbola,
                             Walking, Restriction };

enum class WLineOrigin { Unknown, ImpImp, ImpPrm, PrmPrm };   // drives the L2 rule

enum class DomTransition { In, Out, Touch, Undecided };

// One walked/analytic sample carrying BOTH parametric footprints. Index-corresponded
// with the 3D point: this is the only representation in which the two operands can
// never disagree about where the section is (OCCT IntSurf_PntOn2S).
struct PntOn2S {
    Point  p;                  // 3D
    double u1, v1, u2, v2;     // footprints on S1 and S2
};

// A vertex of an L1 line: a point where the line meets a DOMAIN boundary of S1 or S2,
// with the transition type. This is the entity OCCT's LineConstructor peels; without it
// there is no way to end a section exactly ON a domain edge (OCCT IntPatch_Point).
struct LineVertex {
    double        t = 0.0;             // parameter (or 1-based index for Walking)
    bool          on_dom1 = false, on_dom2 = false;
    DomTransition tr1 = DomTransition::Undecided;
    DomTransition tr2 = DomTransition::Undecided;
    double        u1 = 0, v1 = 0, u2 = 0, v2 = 0;
    double        tol = 0.0;
};

// Exact analytic carrier for the GLine types (Law 3: never re-fit what is known).
struct ConicCarrier {
    Point  origin;
    Vector xdir, ydir, zdir;           // right-handed frame
    double r1 = 0.0, r2 = 0.0;         // radius / major+minor / focal
};

struct SectionLine {
    SectionLineType         type = SectionLineType::Walking;
    WLineOrigin             origin = WLineOrigin::Unknown;
    ConicCarrier            conic;     // valid for Line/Circle/Ellipse/Parabola/Hyperbola
    std::vector<PntOn2S>    pts;       // valid for Walking (and converted Analytic)
    std::vector<LineVertex> vertices;  // sorted by t
    bool                    purge_allowed = true;
    double                  first_t = 0.0, last_t = 0.0;
};

// ---------------------------------------------------------------- L2 domains

// The face's UV rectangle: pcurve bbox, padded per §2.1(d) with METRIC padding.
struct UvRect {
    double u0, u1, v0, v1;
    bool   u_periodic = false, v_periodic = false;
    double u_period = 0.0, v_period = 0.0;
};

enum class RectState { In, On, Out };
RectState classify_rect(const UvRect& r, double u, double v, double tol_uv);

// Returns the kept parameter spans of `L` against both rectangles. tol_uv is
// 35*PConfusion in OCCT; we pass a metric-derived value (see §6/M2).
std::vector<std::pair<double,double>>
line_constructor(const SectionLine& L,
                 const UvRect& d1, const UvRect& d2,
                 const NurbsSurface& s1, const NurbsSurface& s2,
                 double tol_uv);

// ---------------------------------------------------------------- FF output

struct SectionCurve {                    // = IntTools_Curve
    NurbsCurve c3d;
    NurbsCurve pc1, pc2;                 // may be !is_valid()
    double     tol      = 0.0;           // measured (G8)
    double     tang_tol = 0.0;
    bool has_pc1() const { return pc1.is_valid(); }
    bool has_pc2() const { return pc2.is_valid(); }
};

struct SectionPoint { Point p; double u1,v1,u2,v2; };

enum class FFStatus { Ok, Tangent, Empty, Failed };

struct FaceFaceResult {
    FFStatus                  status = FFStatus::Failed;
    bool                      opposite = false;          // only when Tangent
    std::vector<SectionCurve> curves;
    std::vector<SectionPoint> points;
    bool                      reversed = false;          // SortTypes swap was applied
};

struct FFParams {
    bool   approx3d   = true;
    bool   pcurve_on1 = true;
    bool   pcurve_on2 = true;
    double approx_tol = 1e-7;      // OCCT anApproxTol
    double fuzzy      = 1e-7;      // >= Confusion
};

FaceFaceResult face_face(const BRep& A, int faceA,
                         const BRep& B, int faceB,
                         const FFParams& prm);

// ---------------------------------------------------------------- L3 blocks

// 43.2% point: (1-PAR_T)*a + PAR_T*b, PAR_T = 0.43213918.
inline double intermediate_point(double a, double b) {
    constexpr double PAR_T = 0.43213918;
    return (1.0 - PAR_T) * a + PAR_T * b;
}

// The real wire predicates. `face` is (BRep, face index).
struct FaceRef { const BRep* brep; int face; };

RectState  state_point_face   (const FaceRef&, double u, double v);   // FClass2d analog
bool       is_point_in_on_face(const FaceRef&, double u, double v);   // ON counts as IN
bool       is_valid_point_for_face (const FaceRef&, const Point& p, double tol3d);
bool       is_valid_point_for_faces(const FaceRef&, const FaceRef&, const Point& p, double tol3d);
bool       is_valid_block_for_faces(double t0, double t1, const SectionCurve&,
                                    const FaceRef&, const FaceRef&, double tol3d);

// A section curve inside the shared DS, with its paves and blocks.
struct BdsCurve {                        // = BOPDS_Curve
    SectionCurve      c;
    std::array<double,6> box;            // enlarged by tolFF + max vertex tol
    double            tol = 0.0;         // max(c.tol, tolFF)
    std::vector<BdsPave> ext_paves;      // sorted by t, unique
    std::vector<int>     blocks;         // BdsPaveBlock ids, filled by make_blocks
    int face1 = -1, face2 = -1;          // DS face indices (arena)
};

// Stage 6 driver: one call per overlapping face pair, results appended to the DS.
void perform_ff(BdsDS& ds, const FFParams& prm);

// Stage 7 driver: paving + wire trimming + edge minting + cross-pair fusion.
void make_blocks(BdsDS& ds);
void post_treat_ff(BdsDS& ds);           // must not be able to fail the whole stage

} // namespace session_cpp
```

**Notes on the declarations**

* `PntOn2S` is non-negotiable: it is the only structure in which the two operands' UV
  footprints are index-corresponded to one 3D point. Our `SectionSegment`
  (`src/brep_section.h:14-24`) already has this shape (`p3/uvA/uvB`) — keep it, but move it
  into the DS so that it is not per-pair.
* `LineVertex` is the missing entity in our kernel. Without domain transitions the marcher
  cannot end a line ON a boundary, and every downstream stage has to guess.
* `SectionCurve::tol` must be *measured*, never assigned a constant.
* `FFStatus::Tangent` must be a distinct outcome from `Empty` (Law 4).

---

## 4. WHAT OUR CODE DOES TODAY, AND WHERE IT DIVERGES

All paths under `/home/petras/code/code_rust/session/session_cpp/src`.

### 4.1 There is no L1 domain awareness

`Intersection::surface_surface(a, b, tolerance)` — `intersection.cpp:4746-…` — takes two
**bare `NurbsSurface`s**. It has no face, no wires, no UV rectangle, no `TopolTool`
equivalent. Seeds come from overlapping AABBs of a half-resolution sample grid
(`cell_boxes`, `intersection.cpp:4855-4891`; seeding loop from `:4961`), the corrector is a
minimum-norm or tangent-pinned Newton (`correct`, `:4914-4959`) with
`conv_tol = max(tolerance, h_init·1e-7)` (`:4904`), and open domains are clamped
(`clamp_open`, `:4906-4911`) while closed ones wrap (`make_wrap`, `:4812-4826`).

Divergences:

* **D1 — no `IntPatch_Point` / no transitions.** Nothing in the marcher records *why* a walk
  stopped. There is no analog of `IntPatch_ImpImpIntersection`'s restriction pass
  (`IntPatch_ImpImpIntersection.cxx:2786-2858`) nor of `Adaptor3d_TopolTool`'s four
  restriction lines (`Adaptor3d_TopolTool.cxx:57-209`). Consequence: a section can only
  end where Newton stalls, never exactly on a boundary. This is the root of G5 failures.
* **D2 — no `LineConstructor`.** There is no L2 at all. The full untrimmed section is
  returned and trimming is deferred to polygon tests in `brep_section.cpp` (§4.2).
* **D3 — no start points.** `Intersection::surface_surface` has no parameter for seeded
  starting points, so the EF-derived seeds of `GetEFPnts`
  (`BOPAlgo_PaveFiller_6.cxx:2608-2688`) cannot be supplied even once we build stage 5.
  The signature must grow a `const std::vector<PntOn2S>&` argument.

### 4.2 Trimming is a UV polygon test on sampled trim curves

`brep_section.cpp`:
* `face_loops_uv(X, si)` (`:191-257`) tessellates every trim pcurve into a polyline. Sample
  count `min(max(cv_count*4,16),256)` (`:209`), refined until the chord sag is under
  `samp_tol_lp = max(bbox_u, bbox_v) * 2e-4` (`:221`), max 6 passes / 4096 points
  (`:232-244`).
* `point_in_poly_uv` (`:275-285`) is a raw crossing-number test; `dist_to_poly_uv`
  (`:259-273`) is the polyline distance; `in_faces_uv` (`:289-299`) is
  "inside outer OR within eps of outer, and not strictly inside a hole".
* The keep verdict (`:1491-1497`) is
  `in_faces_uv(loopsA, uA,vA, epsA*mult) && in_faces_uv(loopsB, uB,vB, epsB*mult)` with
  `epsA = min(range_u_A, range_v_A) * 1e-3`, `epsB` likewise (`:1448-1449`).

Divergences:

* **D4 — parameter-space classification band (Law 2 violation).** `epsA/epsB` at
  `brep_section.cpp:1448-1449` are `min(uv range) · 1e-3`. OCCT's band is
  `min(UResolution(tolF), VResolution(tolF))` (`IntTools_FClass2d.cxx:729-745`) — a 3D
  tolerance pushed through the surface metric. A 4× padded domain inflates ours 4×; OCCT's
  is unchanged. **Directly breaks G6.**
* **D5 — sampled boundary (sag-dependent classification).** `samp_tol_lp` at
  `brep_section.cpp:221` is `max(bbox extents)·2e-4` — again parameter-space, and again 4×
  on a padded domain. OCCT never tessellates a wire to classify a point; `FClass2d` uses
  the pcurve adaptors directly and its winding pre-pass is only a *fast path* that falls
  back to the exact classifier when indecisive (`IntTools_FClass2d.cxx:716-756`).
* **D6 — the verdict is a bisected predicate, not a block test.** `brep_section.cpp:
  1508-1563`: 9 samples per interval, and when the verdict is mixed the code *bisects the
  predicate 40 times* (`bisect`, `:1544-1550`) and creates new endpoints at the crossing.
  Those endpoints have **no entity behind them** — they are the numeric root of a
  classification function. Two operands' arrangements can and do produce different roots.
  **This is the direct violation of G3, and the mechanism behind "edge identity comes from
  coordinate coincidence".** OCCT never bisects: block endpoints are paves, and a block is
  kept or dropped whole on one 43.2 % test (`IntTools_Context.cxx:717-755`).
* **D7 — whole-interval drop with no recovery entity.** `brep_section.cpp:1531-1541`
  (`nin == 0` ⇒ `drop_iv`, `n_dropped_verdict++`) severs the section network wherever the
  section leaves the trims between two paves; the surrounding machinery
  (`SESSION_CONN_STUB`, `:1570-1589`; `SESSION_ON_QUORUM`, `:1498-1530`;
  `SESSION_VERDICT_EPS_MULT`, `:1486-1490`) exists to paper over that. In OCCT the same
  situation cannot arise because the trim crossing **is** a pave, created by the EF/VF/EE
  stages we do not have.
* **D8 — `ON` handling is a distance band, not a state.** `in_faces_uv`
  (`brep_section.cpp:289-299`) folds ON into IN by a *distance to the sampled polyline*,
  in parameter units. OCCT gets ON from the classifier state
  (`IntTools_Context.cxx:639-643`) at a metric band.

### 4.3 Section identity is coordinate welding, per-pair

* `weld_vertex` (`brep_section.cpp:1418-1430`) is a nearest-match linear scan over
  `scaf.vertices` with radius `weld_tol` derived from the joint bbox diagonal
  (`brep_section.cpp:412-417` and following). Two sections of the same geometric feature
  arriving from different pairs are unified **only if their computed endpoints land within
  `weld_tol`**.
* `build_shared_edge_pool` (`brep_section.cpp:2583-…`, declared `brep_section.h:66`) mints
  **one arena edge per `SectionSegment`**, and segments are per surface pair
  (`SectionSegment::surfA/surfB`, `brep_section.h:21-22`).
* It is opt-in: `use_bop2 = use_scaffold && getenv("SESSION_BOP2")`, `brep.cpp:8495`.

Divergences:

* **D9 — no PostTreatFF. G4 is not enforced anywhere.** The curve for `(A_i, B_j)` and the
  curve for `(A_i, B_k)` are two `SectionSegment`s and become two arena edges. There is no
  nested filler that intersects section edges against each other
  (`BOPAlgo_PaveFiller_6.cxx:1390-1398`), no `aMEPB` one-block-per-edge aliasing
  (`:1628-1641`), and no `PutSEInOtherFaces` (`:4277-4304`). **This is the requested Law 1
  gap, stated exactly.**
* **D10 — no reuse of existing edges.** There is no analog of either `IsExistingPaveBlock`
  overload (`BOPAlgo_PaveFiller_6.cxx:1988-2043`, `:2047-2251`). A section that runs along
  an existing edge of either operand produces a *second* curve. G10 fails.
* **D11 — `refine_scaffold_at_breaks` is the wrong direction of information flow.**
  `brep_section.h:91-92` / `brep_section.cpp:2427-…` feed breakpoints *discovered by an
  operand's arrangement* back into the shared scaffold. That is a repair for D6/D7. With
  paves from real interference entities, no arrangement can discover a crossing the
  scaffold does not already have, and this API disappears.

### 4.4 Dispatch coverage

`analytic_ssi` — `intersection.cpp:4199-4290`:
* Recogniser tolerance `rtol = max(tolerance,1e-7) · 1e4` (`:4201`) ⇒ **1e-3 absolute** for
  a default tolerance. `recognize_surface` (`:2668-2675`) *fits* a quadric
  (`fit_cylinder :2381`, `fit_cone :2442`, `fit_sphere :2516`, `fit_torus :2555`).
* Arms present: plane×{plane,sphere,cylinder,cone,torus}, sphere×sphere,
  cylinder×{sphere,cone,cylinder,torus}, cone×{sphere,torus}, sphere×torus, torus×torus
  (`:4215-4269`).
* **There is no `cone × cone` arm.** Control falls to `else { return res; }` (`:4270-4272`)
  ⇒ marcher. OCCT has `iTT == 33 → IntCoCo` (`IntPatch_ImpImpIntersection.cxx:2709-2716`).
* Several arms only fire in special poses: `ssi_cylinder_sphere` (`:3919`),
  `ssi_cylinder_cone` (`:3934`), `ssi_cone_sphere` (`:3948`), `ssi_cylinder_torus`
  (`:4029`), `ssi_cone_torus` (`:4044`), `ssi_sphere_torus` (`:4069`) all return `handled =
  false` outside coaxial/concentric/centre-on-axis configurations, sending the pair to the
  marcher (`:4274`).
* Planar dispatch re-derives the plane by sampling the surface centre
  (`plane_from`, `:4764-4771`), rather than carrying analytic identity.

Divergences:

* **D12 — Law 3 violated at the dispatcher.** OCCT gets the quadric from the surface's own
  type (`SetQuad`, `IntPatch_ImpImpIntersection.cxx:3058-3091`); we re-fit at 1e-3. A
  rotated cylinder read from STEP is *known* to be a cylinder with axis `R·a`; fitting it
  can and does fail on skinny patches.
* **D13 — no cone×cone arm at all**, and no `TreatAsBiParametric`/`bGeomGeom` policy
  (`IntPatch_Intersection.cxx:1411-1546`): our fallback is "marcher", OCCT's fallback is
  "PrmPrm **with** typed reasons and with the ALine→WLine machinery and pole/apex
  extension" (`:1816-1903`).
* **D14 — no ALine.** OCCT's generic-pose quadric×quadric answer is an `IntPatch_ALine`
  converted to a 200-point WLine with pole/apex extension. We have no analytic-line
  representation, so every generic-pose curved pair degrades to a bare marcher with no
  knowledge of where the apex/pole is.

### 4.5 Tolerances and edge construction

* **D15 — no `ComputeTolReached3d`.** Nothing measures the 3D↔2D deviation of a produced
  section. `SectionScaffold` carries only `max_devA/max_devB` diagnostics
  (`brep_section.h:47-48`), computed but not turned into an edge tolerance. G8 fails.
* **D16 — domain-relative constants throughout the consumer.**
  `nurbssurface_trimmed.cpp:574` `samp_tol = max(range_u,range_v)*2e-5`;
  `nurbssurface_trimmed.cpp:1653` `samp_tol = max(range_u,range_v)*1e-3`;
  `brep.cpp:4350` `eps_border = min(du,dv)*2e-3`;
  `brep.cpp:4280` `scaf_forced_eps = clamp(min_range*1e-2 … min_range*1.3e-1)`;
  `brep.cpp:4262` `ov = min_range*1e-2`. All inflate 4× under a 4× padded domain. G6 fails.
  Note `brep.cpp:4269-4275` already computes a metric factor `uv3dF = max(|dS/du|,|dS/dv|)`
  at the patch centre — the conversion machinery exists; it is used to *clamp* a
  parameter-space constant instead of *replacing* it.
* **D17 — overshoot stubs.** `brep.cpp:4401-4450` extends section cut ends past the trim
  boundary by `ov` to force an arrangement crossing. OCCT never does this; it does not need
  to, because the crossing point is a pave and a forced node by construction. Every stub is
  a fabricated geometric entity and a G3 violation.

---

## 5. ACCEPTANCE TESTS

All in-memory, no file I/O. Each test is run at `N = 20` random rigid motions
(`R` uniform over SO(3), translation uniform in a box of 10× the operand size) plus the
identity, and each invariant must hold at every pose (G7). "exact" below means the stated
absolute tolerance on a unit-scale operand.

### T1 — sphere × sphere (analytically known)
Spheres `R1=1` at origin, `R2=1` at `(1,0,0)`; apply `R`.
Known answer: one circle, centre `R·(0.5,0,0)`, radius `√3/2`, plane normal `R·(1,0,0)`.
Asserts: `curves.size()==1`; curve closed; `|radius - √3/2| < 1e-12`; G1 at 1e-9;
one pave block (a closing pave, not two halves); the section edge count after fusion is 1.
*This is the case that currently returns 2 faces / 2 naked for every rotation.*

### T2 — cylinder × cylinder, equal radii, perpendicular crossing axes (Steinmetz)
`R=1` along `Z` and along `X`, both through the origin.
Known answer: exactly 2 planar ellipses, semi-axes `1` and `√2`, in the planes
`x = ±z`. Asserts: `curves.size()==2`; each planar to 1e-12; each closed;
`|arclen(c) - E(√2/2)·4·√2| < 1e-9`; G1 at 1e-9; G7.
Oracle-free variant: total arclength is invariant under `R` to 1e-12.

### T3 — cone × cone, distinct apexes, skew axes (no dispatcher arm today)
Cones semi-angle `π/6`, apexes at `(0,0,0)` and `(2,0.3,0.7)`, axes `Z` and
`(0.2,0.9,0.4)` normalised.
No closed form. Asserts (all oracle-free): G1 at 1e-7; every emitted curve either closed or
terminating on a face boundary (G5); `curves.size()` and total block count identical at all
21 poses (G7); no two curves within `tol1+tol2` along any overlap (G4);
**status is never `Failed`** and runtime < 200 ms.
*Today this pair has no dispatcher arm (`intersection.cpp:4270`) and falls to the marcher.*

### T4 — box × sphere, sphere centred on a box edge (the fusion test)
Unit box `[0,1]³`; sphere `R=0.4` centred at `(1,1,0.5)` — it cuts faces `x=1` and `y=1`
and their shared edge. Faces `F_x` and `F_y` are two different FF pairs against the same
sphere face.
Asserts: the arc on `F_x` and the arc on `F_y` **share both endpoint vertices as the same
entity** (pointer/index equality in the DS, not coordinate proximity) — this is G4/G3;
each endpoint lies on the box edge to 1e-12; the box edge is split into exactly 3 blocks;
in each face's 2D arrangement no vertex has degree 1 (G5).
*Today: two independent segments, welded only if `weld_tol` happens to cover the gap
(`brep_section.cpp:1418-1430`).*

### T5 — cross-pair fusion proper (the Law-1 test)
Operand B is a box whose face `y=0` has been **pre-split** into two coplanar faces
`F2 = {x<0.5}` and `F3 = {x>0.5}` sharing an interior edge. Operand A is a cylinder
`R=0.3`, axis `Y`, through `(0.5, ·, 0.5)` — its lateral face `F1` intersects both `F2` and
`F3`, and the true section is ONE circle.
Asserts: after `post_treat_ff`, exactly **one** section edge exists for that circle
(not two arcs); it is referenced by `F1`, `F2` and `F3`; it carries a pave at the
`F2|F3` shared edge; `|radius - 0.3| < 1e-12`. Running the same test with the box face
un-split must produce the identical edge geometry to 1e-12.
*This test cannot pass today: `SectionSegment` is keyed by `(surfA,surfB)`
(`brep_section.h:21-22`), so there are two segments by construction.*

### T6 — domain invariance (G6)
Take T1 and T2. Re-parameterise each operand's surfaces by
`u' = 4u - 0.04, v' = 4v - 0.04` (the measured STEP round-trip pattern) **without changing
the geometry** (re-knot, re-scale the pcurves accordingly).
Assert: identical curve count, identical block count, and 3D Hausdorff distance between the
two section sets `< 1e-12`.
*Today: `epsA/epsB` (`brep_section.cpp:1448-1449`), `samp_tol_lp` (`:221`), `eps_border`
(`brep.cpp:4350`) and `scaf_forced_eps` (`brep.cpp:4280`) all scale by 4.*

### T7 — single-operand identity (the 32-of-36-naked reproducer)
Split ONE operand alone (`A` against `B`, discard `B`'s result) on a 4× padded UV domain,
then again on the natural domain.
Assert: the two splits have identical edge counts, identical naked-edge counts (which must
be 0 for a closed operand), and identical face counts.
*Measured today: 32 naked of 36 on the padded domain, 0 on the natural one, while the UV
arrangement is verified identical.*

### T8 — boundary graze, typed
(a) Cylinder `R=1` axis `Z` and a plane `x=1` (tangent along a line).
(b) Same, but the plane is trimmed so the tangency line **is** one of its trim edges.
Assert: (a) status is `Tangent` with `opposite` set correctly, `curves.size()==0`, and the
pair is handed to the same-domain stage; (b) exactly one section edge is produced and it
**is the existing trim edge object** (G10), with its tolerance raised, not a second curve.

### T9 — pave provenance (G3)
For every emitted block endpoint in T1–T8, assert that it carries a non-null provenance tag
from `{existing_vertex, vv, ve, ee, vf, ef, curve_bound, closing}`. Any endpoint tagged
`bisection` fails the test. *This is a structural assert; it makes D6 unrepresentable.*

### T10 — no dangling section ends (G5)
For every face touched in T1–T8, build the 2D arrangement of (wires ∪ section pcurves) and
assert that no vertex has degree 1. This is computable per face with no oracle and catches
the whole "lost regions" family without needing a reference result.

### T11 — curve tolerance is honest (G8)
For every emitted `SectionCurve` with pcurves, assert
`max_t |c3d(t) - S_i(pc_i(t))| <= tol` for `i∈{1,2}`, sampled at 200 parameters plus knots.
This is exactly what `ComputeTolReached3d` guarantees
(`IntTools_FaceFace.cxx:613-691`); today nothing does.

### T12 — the 224-cell matrix, FF-local
The existing primitive-pair sweep, but scored **only** on G1/G4/G5/G7/G9 at the FF+blocks
level (no build, no volume). Baseline to record before any change; the gate for each
increment in §6 is "non-decreasing".

---

## 6. IMPLEMENTATION ORDER

Smallest shippable increment first. Every increment is independently revertable, gated by
the guards battery (`kb/ARCHITECTURE_v2.md` §5: base 3 ops exact, matrix 45/45, edge 54/54,
minitests C++ 760, A-op-A) **plus** its own gate below. Nothing in this list requires the
v2 DS to exist first except F6 and F7.

**F0 — instrument (no behaviour change).**
Add T12 as a standing measurement; record the baseline for G1/G4/G5/G7/G9 over the 224
cells and over the chairs corpus. Add T9's provenance tag to `SectionSegment` (currently
untagged) and print the histogram.
*Gate: numbers recorded, zero behaviour delta (byte-identical outputs).*

**F1 — measured curve tolerance (G8).**
Implement `compute_tol_reached_3d`: for each produced `(c3d, pc1, pc2)` compute the max
deviation to each surface (11 sub-intervals × golden-section, `IntTools_FaceFace.cxx:
2899-2988`), with the `1+1e-5` margin (`IntTools_Tools.cxx:774-775`), and store it on the
section. Nothing consumes it yet.
*Gate: T11 green; guards battery unchanged.*

**F2 — metric tolerance conversion (G6). This is `ARCHITECTURE_v2.md` M0 for this
subsystem.**
Introduce `uv_tol(surface, u, v, tol3d) = tol3d / max(|dS/du|,|dS/dv|)` and replace, in
order: `brep_section.cpp:1448-1449` (`epsA/epsB`), `brep_section.cpp:221` (`samp_tol_lp`),
`brep.cpp:4350` (`eps_border`), `nurbssurface_trimmed.cpp:574` and `:1653` (`samp_tol`),
`brep.cpp:4262`/`:4280` (`ov`, `scaf_forced_eps` — the metric factor `uv3dF` is already
computed at `brep.cpp:4269-4275`, use it as the source, not the clamp).
*Gate: T6 and T7 green; guards battery non-decreasing.*

**F3 — exact wire classification (D4/D5/D8).**
Replace `in_faces_uv` (`brep_section.cpp:289-299`) with a two-tier classifier mirroring
`IntTools_FClass2d::Perform`: a tessellated winding fast path that returns
`In/Out/Indecisive`, and an exact fallback that ray-casts against the **pcurves**
(Newton-refined crossings), band `uv_tol(surface, tol_face)`. Return a tri-state
`RectState`, not a bool; `is_point_in_on_face` accepts `On`, `is_point_in_face` does not
(`IntTools_Context.cxx:604-608` vs `:639-643`).
*Gate: T6 green with the tessellation deliberately coarsened 4× (the classification must
not move); guards battery non-decreasing.*

**F4 — block-level verdict, no bisection (G2/G3, kills D6/D7).**
Replace the 9-sample/bisect verdict (`brep_section.cpp:1508-1563`) with:
one `is_valid_block_for_faces(t0,t1,…)` call per interval between consecutive paves, using
`intermediate_point` (43.2 %) and the pcurve shortcut when the section carries pcurves.
Delete `SESSION_CONN_STUB`, `SESSION_ON_QUORUM`, `SESSION_VERDICT_EPS_MULT` and the
`drop_iv` recovery machinery. Add the provenance tag from F0 as a hard assert.
**Prerequisite:** intervals must be delimited by real paves, which for the trim-crossing
case they already are (the `refine_trim_pave` Newton solve,
`brep_section.cpp:330-371`, is the correct entity — keep it, tag it `trim_crossing`).
*Gate: T9 green (zero `bisection` tags); T10 non-decreasing; guards battery
non-decreasing. Expect regressions here that F5 fixes — land F4 and F5 together if so.*

**F5 — L1/L2: domain-aware sections (G5, kills D1/D2/D17).**
   1. Give the marcher a domain: pass each surface's `UvRect` (padded per §2.1(d) with
      metric padding) into `Intersection::surface_surface`, and make the walker **stop on
      the rectangle boundary**, emitting a `LineVertex` with the crossing parameter and the
      transition sense (`In`/`Out` from the sign of the step against the boundary normal).
   2. Implement `line_constructor` (§2.6 branches 1–3; branch 4 only once RLines exist).
      For `Walking`, port the three sub-rules including the `WLImpPrm` interpolated-midpoint
      rule (`GeomInt_LineConstructor.cxx:183-225`) — our marcher is a PrmPrm analog, so use
      the strict both-endpoints rule (`:226-252`) and tag `origin = PrmPrm`.
   3. Delete the overshoot stubs (`brep.cpp:4401-4450`) and the forced-node machinery that
      exists to compensate for sections that stop short.
*Gate: T10 green (no degree-1 vertices) on the chairs corpus and on the 224 cells;
T7 green; guards battery non-decreasing.*

**F6 — PostTreatFF: cross-pair fusion (G4/G10, kills D9/D10).**
   1. Move section curves out of the per-pair scaffold into the DS: `BdsCurve` keyed by
      `(face1,face2)` but stored in one arena.
   2. `is_existing_pave_block` #1 — a block whose 43.2 % point projects onto a **shared
      edge** of the two faces within `max(tolE, maxTolV) + fuzz` reuses that edge and raises
      its tolerance (`BOPAlgo_PaveFiller_6.cxx:1988-2043`).
   3. `is_existing_pave_block` #2 — BVH over the faces' ON/IN blocks, acceptance on the
      43.2 % point plus both ends at `tolR3D + fuzz`, doubled for face-common blocks, and
      the 25° tangent extension (`:2047-2251`). The reused block is attached to the second
      face.
   4. `post_treat_ff`: run a nested VV/VE/EE pass over the **section edges themselves**
      (this is where the `SharedEdgePool` from `brep_section.h:57-62` is absorbed, per
      `ARCHITECTURE_v2.md` §1), producing one DS edge per geometric curve and one pave
      block object shared by every `BdsCurve` that produced it. On failure, degrade to "no
      fusion", never to "no sections" (contra `BOPAlgo_PaveFiller_6.cxx:1393-1397`).
   5. `put_se_in_other_faces` (`:4277-4304`): intersect every section edge against the faces
      that did not create it.
   6. Delete `refine_scaffold_at_breaks` (`brep_section.h:91-92`) — it becomes unreachable.
*Gate: T4 and T5 green; G4 asserted over all 224 cells; guards battery non-decreasing.*

**F7 — dispatch completeness (kills D12/D13/D14).**
   1. Carry analytic identity from the reader instead of fitting: `recognize_surface`
      (`intersection.cpp:2668`) becomes a *fallback* behind a stored analytic tag (Law 3).
   2. Add the missing `cone × cone` arm (`IntPatch_ImpImpIntersection.cxx:2709-2716`
      → `IntCoCo`), and complete the generic-pose arms currently returning
      `handled=false` (`intersection.cpp:3919-4199`).
   3. Add an `ALine` carrier + a 200-point `ALine→WLine` conversion with the
      pole/apex `ExtendTwoWLines` behaviour (`IntPatch_Intersection.cxx:1816-1903`,
      `IntPatch_ALineToWLine.cxx:150-153`, `:361-451`).
   4. Port the `TreatAsBiParametric` / `bGeomGeom` policy verbatim
      (`IntPatch_Intersection.cxx:1411-1592`) so that near-degenerate cones and
      major≤minor tori are sent to the parametric walker **for a stated reason**, not by
      accident.
   5. Port `isTreatAnalityc` (`IntTools_FaceFace.cxx:249-324`) including the
      `majorR < 1e5·minorR` ellipse refusal.
*Gate: T1, T2, T3, T8 green at all 21 poses; the 224-cell matrix G1/G4/G5/G7/G9 score
strictly improved; guards battery non-decreasing.*

**F8 — seeded walking from EF (needs stage 5 from `ARCHITECTURE_v2.md` M3).**
Add the `const std::vector<PntOn2S>&` start-point argument to the marcher and feed it from
the EF interference points (`BOPAlgo_PaveFiller_6.cxx:2608-2688`), with the torus purge
(`IntTools_FaceFace.cxx:492-500`) and the two-phase seeded-then-generic run
(`IntPatch_Intersection.cxx:1673-1680`).
*Gate: T3 and T10 on the freeform corpus; chairs corpus non-decreasing.*

**F9 — the remaining OCCT behaviours, each independently gated.**
Far-from-origin recentring (`BOPAlgo_Tools.cxx:1912-1937`); the seam-shift heuristic
(`BOPAlgo_PaveFiller_6.cxx:393-486`); `DecompositionOfWLine`
(`IntTools_WLineTool.cxx:492-1312`) once periodic seams are in the marcher; the plane/cone
`RejectLines` rule (`IntTools_FaceFace.cxx:1974-2003`); `ComputeIntRange` tangential
tolerance for curved pairs (`IntTools_Tools.cxx:783-804`) — note OCCT itself only applies it
to plane/plane (`IntTools_FaceFace.cxx:682-689`), so this one is an intentional improvement
and must be measured, not assumed.

### Where OCCT itself gives up (do not invent a better answer silently)

* Tangential tolerance is computed only for plane/plane; everything else gets
  `max(tolF1,tolF2)` (`IntTools_FaceFace.cxx:682-689`, explicit source comment).
* A failed FF pair is a **warning**, and the boolean proceeds with that section missing
  (`BOPAlgo_PaveFiller_6.cxx:545-553`).
* A `PostTreatFF` failure aborts the **entire** section stage
  (`:1393-1397` → `:1121-1124`).
* The ImpPrm intersector does not respect the implicit surface's domain; OCCT compensates
  with a special two-point classification rule rather than fixing the intersector
  (`GeomInt_LineConstructor.cxx:183-225`, issue #29972).
* `CorrectSurfaceBoundaries` pads a **parameter** domain by a **3D** tolerance
  (`IntTools_FaceFace.cxx:449`, `:2031`, `:2077`) — a unit error OCCT lives with.
* A curve with fewer than two ext paves produces **no** section edge, and the pieces outside
  the extreme paves are discarded, never extrapolated
  (`BOPDS_PaveBlock.cxx:249-311`; `BOPAlgo_PaveFiller_6.cxx:892`); the only recovery is the
  single `aFFToRecheck` second pass (`:879`, `:1067-1071`).
* `aDMNewSD`'s SD closure is a **single pass** (`:1659-1668`), so a 3-deep chain can leave a
  stale mapping (`kb/audit_occt_ff-samedomain.md` E5/trap 13). A union-find is a deliberate
  improvement; verify nothing downstream depended on the stale value.
