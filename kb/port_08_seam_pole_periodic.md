# port_08_seam_pole_periodic — SEAMS, POLES AND DEGENERATE EDGES ON PERIODIC SURFACES

Port specification for the topology of closed/periodic curved faces: seam edges, degenerate
(pole) edges, periodic parameter normalisation, and what the boolean pipeline must do when a
section curve crosses a seam or terminates at a pole.

Written to be implementable without further reference to OCCT. Every algorithmic claim carries
`file:line`. Where OCCT itself gives up or falls back it is marked **[OCCT FALLBACK]** or
**[OCCT GIVES UP]** — those are answers, not gaps.

**OCCT ground truth** (read 2026-07-26, tree `/home/petras/code/code_cpp/OCCT`, 8.0.1.dev). All
paths below are relative to `/home/petras/code/code_cpp/OCCT/src/`.

| role | file |
|---|---|
| edge record + flags | `ModelingData/TKBRep/BRep/BRep_TEdge.hxx/.cxx` |
| **the two-pcurve representation** | `ModelingData/TKBRep/BRep/BRep_CurveOnClosedSurface.cxx/.hxx` |
| pcurve selection by orientation | `ModelingData/TKBRep/BRep/BRep_Tool.cxx` |
| seam/degenerate writers | `ModelingData/TKBRep/BRep/BRep_Builder.cxx` |
| **normative face constructor** | `ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeFace.cxx` |
| independent confirmation | `ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_OneAxis.cxx`, `BRepPrim_Builder.cxx`, `BRepPrim_Sphere.cxx` |
| UV bounds / closedness | `ModelingData/TKBRep/BRepTools/BRepTools.cxx` |
| surface closedness test | `ModelingData/TKGeomBase/GeomLib/GeomLib.cxx:2693-2850` |
| **seam pcurve pairing in BOP** | `ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools3D.cxx:58-327` |
| legacy seam pairing (3rd impl) | `ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools2D_1.cxx:164-287` |
| pcurve periodic adjust | `ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools2D.cxx:247-400` |
| BOP face assembly | `ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder_2.cxx:387-464, 1100-1189` |
| degenerate-edge stage | `ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_8.cxx` (whole file, 393 lines) |
| DS flags for degenerate | `ModelingAlgorithms/TKBO/BOPDS/BOPDS_DS.cxx:440-500, 1614-1779` |
| WLine seam decomposition | `ModelingAlgorithms/TKBO/IntTools/IntTools_WLineTool.cxx:44-1120` |
| section restriction to domain | `ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_LineConstructor.cxx` |
| seam/bound detection on WLine | `ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_WLineTool.cxx:619-725` |
| **pole/apex special points** | `ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_SpecialPoints.cxx:801-1128` |
| pole classification on ALine | `ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ALineToWLine.cxx:63-141` |
| period normalisation | `FoundationClasses/TKMath/ElCLib/ElCLib.cxx:115-149`, `ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt.cxx:21-48` |
| validity rules | `ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Edge.cxx:80-160`, `BRepCheck_Wire.cxx:2085-2170` |
| closed-shell rule | `ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BOP.cxx:1474-1510` |
| 2D classifier | `ModelingAlgorithms/TKBO/IntTools/IntTools_FClass2d.cxx:160-230` |
| face UV window | `ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_Surface.cxx:56-81` |
| constants | `FoundationClasses/TKernel/Precision/Precision.hxx` |

**Our code** (READ ONLY): `src/brep.cpp`, `src/brep.h`, `src/brep_section.cpp`,
`src/nurbssurface_trimmed.cpp`, `src/closest.cpp`, `src/file_step.cpp`, `src/primitives.cpp`.

---

## 0. THE ONE-PARAGRAPH SUMMARY

A closed periodic surface has no boundary in 3D but its parameter domain is a rectangle. The
rectangle's two opposite sides map to the *same* 3D curve. A B-Rep face on such a surface is
topologically complete only if that curve is present **once** as an edge and **twice** as a
co-edge (trim), the two co-edges carrying two pcurves that differ by exactly one period, with
opposite orientations. That edge is the **seam**. Where an iso-line of the surface collapses to
a single 3D point (sphere pole, cone apex), the corresponding side of the rectangle is a
**degenerate edge**: a real topological edge, with a real pcurve, with **no 3D curve**, with one
vertex used twice. Both constructs exist so that the face's UV boundary is a closed 2D loop —
without them, the loop is open and every downstream algorithm (classification, wire walking,
Euler count, shell closure) is operating on a torn domain.

---

## 1. WHAT THIS SUBSYSTEM MUST GUARANTEE

Each guarantee is stated so it can be asserted in code with no reference kernel.

### S1 — SEAM EXISTENCE

> For every face `F` whose surface `S` is closed in `u` and whose face UV window spans the full
> `u` period (within `Precision::PConfusion()`), the outer wire of `F` contains an edge `e`
> **exactly twice**, once FORWARD and once REVERSED, whose two pcurves lie on `u = umin` and
> `u = umax`. Symmetrically for `v`.

Test: `count_occurrences(e, F) == 2 && orientation(occ0) != orientation(occ1)`. This is OCCT's
`BRepTools::IsReallyClosed` — `BRep_Tool::IsClosed(E,F)` **and** exactly two occurrences
(`BRepTools.cxx:1204-1220`).

### S2 — SEAM PCURVE PAIR RELATION (exact, not tolerant)

> Let `Pf` be the pcurve returned for the FORWARD occurrence and `Pr` for the REVERSED
> occurrence of a seam edge on face `F` **taken FORWARD**. Both are parametrised on the same
> parameter interval `[t1,t2]` (the edge range). Then for all `t`:
> `Pf(t) - Pr(t) = (±T_u, 0)` for a u-seam, `(0, ±T_v)` for a v-seam, where `T` is the period.

Test: sample 33 parameters, require `max_t |Pf(t) - Pr(t) - delta| < 1e-12 * |delta|` and
`|delta|` equal to the period to 1e-12 relative. This is the invariant that makes the seam a
seam rather than two nearby curves; it is exact in OCCT because both pcurves are constructed by
translating one copy (`BOPTools_AlgoTools3D.cxx:201-207`, `BRepLib_MakeFace.cxx:655-668`).

### S3 — SEAM ORIENTATION INVARIANT (the sign that encodes which side is which)

> With `delta = Pf(t) - Pr(t)` and `Tf = Pf'(t)`:
> `cross2d(delta, Tf) = delta.x*Tf.y - delta.y*Tf.x > 0`.

Equivalently: walking the FORWARD occurrence keeps the face interior on its left in UV, and so
does the REVERSED occurrence. This single scalar reproduces all four OCCT cases:

| case | Pf located at | Tf | delta | cross |
|---|---|---|---|---|
| u-seam, edge runs +v | `u = umax` | `(0,+1)` | `(+T,0)` | `+T` |
| u-seam, edge runs −v | `u = umin` | `(0,−1)` | `(−T,0)` | `+T` |
| v-seam, edge runs +u | `v = vmin` | `(+1,0)` | `(0,−T)` | `+T` |
| v-seam, edge runs −u | `v = vmax` | `(−1,0)` | `(0,+T)` | `+T` |

Rows 1 and 3 are literally what `BRepLib_MakeFace` emits (`:689` u-seam `UpdateEdge(eumin,
Lumax, Lumin, …)`, `:753` v-seam `UpdateEdge(evmin, Lvmin, Lvmax, …)`), and what
`BRepPrim_OneAxis` emits (`:435-438` u-seam `SetPCurve(ESTART, F, lin@myAngle, lin@0)`;
`:392-395` v-seam `SetPCurve(ETOP, F, lin@myVMin, lin@myVMax)`). Rows 2 and 4 are what
`BOPTools_AlgoTools3D::DoSplitSEAMOnFace` produces when the split's tangent runs the other way
(`:209-230`). **Caveat**: if the face is stored REVERSED, `BRep_Tool::CurveOnSurface(E,F)`
flips the edge first (`BRep_Tool.cxx:310-313`), so the invariant must always be evaluated with
the face taken FORWARD.

### S4 — SEAM SPLITS ARE SEAMS

> If a seam edge is split into pieces by paves, **every** piece is itself a seam on that face:
> two pcurves satisfying S2/S3, appearing twice in the rebuilt wire.

OCCT enforces this at exactly two sites, both of which run before the wire is assembled:
`BOPAlgo_Builder_2.cxx:429-455` and `:1161-1166`. A split that fails to be made closed raises
`BOPAlgo_AlertUnableToMakeClosedEdgeOnFace` (`:444`) — **[OCCT FALLBACK]**, see §2.11.

### S5 — DEGENERATE EDGE REPRESENTATION

> A degenerate edge `d` has: `degenerate == true`; **no 3D curve**; at least one pcurve on the
> face; exactly one vertex, present twice (FORWARD and REVERSED); a parameter range equal to the
> full non-degenerate extent of its pcurve (e.g. `[0, 2π]` for a sphere pole).

Test: `d.curve3d == none && d.degenerate && vertices(d) == {v, v} && range(d).length > 0`.
Validity: OCCT reports `BRepCheck_InvalidDegeneratedFlag` if a degenerate edge **has** a 3D
curve (`BRepCheck_Edge.cxx:145-147`), and `BRepCheck_No3DCurve` if a non-degenerate edge has
none (`:120-124`).

### S6 — DEGENERATE EDGES ARE NEVER INTERFERENCE OPERANDS

> No degenerate edge participates in VE, EE, EF or FF interference; it is split only by the
> dedicated pole stage; it is never a candidate for a naked-edge or non-manifold report.

OCCT: `BOPDS_DS::prepareEdges` flags degenerate edges (`BOPDS_DS.cxx:1630, 1674`) and
`prepareFaces` re-flags them with their face (`:1743-1746`); every stage then early-outs on
`aSI.HasFlag()` — VE `BOPAlgo_PaveFiller_2.cxx:171`, EE `PaveFiller_3.cxx:190,195,1047`, EF
`PaveFiller_5.cxx:228,804`, FF pave-block tree `PaveFiller_6.cxx:868`, `:4407`,
`PaveFiller_7.cxx:410`, `PaveFiller_9.cxx:95`, `PaveFiller_11.cxx:64`.

### S7 — CLOSURE COUNTS SEAMS AND POLES CORRECTLY

> A shell is closed iff every edge satisfies at least one of: (a) referenced by ≥2 faces;
> (b) degenerate; (c) `IsClosed(e, F)` — a seam, i.e. it carries two pcurves on the single face
> that owns it; (d) orientation INTERNAL.

Verbatim from `BOPAlgo_BOP.cxx:1474-1505`. This is the rule our `is_solid()`/`topology_report()`
must implement — with the seam clause coming from **topology** (two pcurves), not from a
3D-length threshold.

### S8 — SECTION CURVES ARE SPLIT AT SEAM CROSSINGS

> No pcurve stored on a face may contain a parameter jump of more than half a period in a
> periodic direction; a section curve that wraps is delivered as ≥2 pcurves, each entirely
> inside the face UV window, meeting at a pair of points that are 3D-identical and UV-separated
> by exactly one period.

Test: for every stored pcurve, sample densely and assert
`max |Δu| < T_u/2 && max |Δv| < T_v/2`; and for every consecutive pair produced from one section
curve, assert `|P3D(end_k) - P3D(start_{k+1})| < tol3d`.

### S9 — PCURVES LIVE IN THE FACE'S UV WINDOW

> After construction, every pcurve's mid-parameter point lies inside
> `[Umin - δ, Umax + δ] × [Vmin - δ, Vmax + δ]` where `Umin..Vmax` are the **face's** UV bounds
> (`BRepTools::UVBounds`) and `δ = Precision::PConfusion()`; if not, the whole pcurve is
> translated by an integral number of periods until it is.

OCCT: `BOPTools_AlgoTools2D::AdjustPCurveOnSurf` (`:247-400`), applied unconditionally at the
end of every pcurve construction (`BOPTools_AlgoTools2D.cxx:611-614`).

### S10 — MARCHING UNWRAP IS RELATIVE, NEVER ABSOLUTE

> Along an intersection polyline, consecutive parameter values in a periodic direction differ by
> at most half a period. Normalisation into a canonical window happens **once, at the end**, per
> piece — never per sample.

OCCT: `IntPatch_SpecialPoints::AdjustPointAndVertex` (`:1082-1128`) shifts each new point by
whole periods until it is within half a period of the reference point. This is the only correct
rule; an absolute `fmod` per sample manufactures the jumps S8 then has to clean up.

### S11 — POLE PARAMETRISATION IS CHOSEN, NOT COMPUTED

> At a pole the surface has a one-parameter family of UV pre-images of one 3D point and no
> normal. The pole's free parameter must be **assigned** from the direction of approach of the
> curve, and the assignment must be recorded, not re-derived.

OCCT: `IntPatch_SpecialPoints::AddSingularPole` sets `aUquad = 0`, `aVquad = ±π/2` (sphere) or
`-R/sin(semiangle)` (cone) and then *chooses* `aUquad` from the tangent plane of the other
surface (`:828-840`, `:901-922`); on continuing past the pole the U-parameter is adjusted with a
**quarter-period** `π/2`, not `2π` (`:1063`). See §2.12.

### S12 — FACE UV BOUNDS OF A SEAMED FACE SPAN THE FULL PERIOD

> For a full periodic face, `UVBounds(F)` returns exactly the surface's natural bounds; adding a
> seam must not enlarge them, and a face with no pcurves at all falls back to natural bounds.

OCCT: `BRepTools::AddUVBounds(F, B)` (`:126-157`), with the void-box fallback at `:141-153`;
per-edge clamping to natural bounds happens only in **non-periodic** directions (`:270-281`,
`:351-362`).

### S13 — POLE VERTEX TOLERANCE ABSORBS THE DEGENERACY RADIUS

> The vertex at a pole carries tolerance ≥ the measured collapse radius of the degenerate iso.

OCCT: `BRepLib_MakeFace.cxx:597-620` builds the corner vertices with
`max(uminTol, vminTol)` etc., where those tolerances come out of `IsDegenerated` (`:392-459`) as
the circle radius or the max pole spread. A pole is not a mathematical point in a tolerant
kernel; it is a small ball, and the ball's radius must be stored.

---

## 2. OCCT'S ALGORITHM

### 2.1 The representation objects

`BRep_TEdge` (`BRep_TEdge.hxx:37-80`) holds a *list* of curve representations plus three bit
flags in `myFlags`: `SameParameter`, `SameRange`, `Degenerated` (accessors `:54-64`).

Curve representations relevant here:

- `BRep_CurveOnSurface` — one pcurve + surface + location + range + the two UV end points
  (`myUV1`, `myUV2`).
- `BRep_CurveOnClosedSurface` — **derives from** `BRep_CurveOnSurface` and adds `myPCurve2`,
  `myContinuity`, and a second pair of UV end points `myUV21`, `myUV22`
  (`BRep_CurveOnClosedSurface.cxx:30-39`). `IsCurveOnClosedSurface()` returns true (`:43-46`).
  `Surface2()`/`Location2()` return the *same* surface and location as the first pcurve
  (`:81-91`) — a seam's two pcurves are always on **one** surface.
  `Update()` evaluates `myPCurve2` at `First()`/`Last()` into `myUV21/myUV22` and then defers to
  the base class for the first pcurve (`:123-134`).

**Selection rule** — the whole orientation semantics in six lines
(`BRep_Tool.cxx:334, 347-364`):

```
CurveOnSurface(E, S, L, first, last):
    Eisreversed = (E.Orientation() == REVERSED)
    for cr in E.Curves():
        if cr.IsCurveOnSurface(S, L):
            cr.Range(first, last)
            if cr.IsCurveOnClosedSurface() and Eisreversed: return cr.PCurve2()
            else:                                           return cr.PCurve()
```

and, one level up (`BRep_Tool.cxx:301-315`), `CurveOnSurface(E, F, …)` **reverses the edge**
first when the face is REVERSED. `UVPoints` mirrors this exactly (`:1005-1090`, the
`UVPoints2` branch at `:1023-1028`). Iteration by index counts the second pcurve as its own
index (`:488-538`, the `++i` at `:518`).

**Writers** (`BRep_Builder.cxx`):
- one pcurve: `UpdateEdge(E, C, S, L, Tol)` → `UpdateCurves(lcr, C, S, L)` (`:655-672`, helper
  `:104-167`), which *removes* any existing representation on `(S,L)` first (`:130-137`).
- two pcurves: `UpdateEdge(E, C1, C2, S, L, Tol)` → `UpdateCurves(lcr, C1, C2, S, L)`
  (`:702-720`, helper `:251-307`) constructing
  `new BRep_CurveOnClosedSurface(C1, C2, S, L, GeomAbs_C0)` (`:290-291`). The range is inherited
  from the 3D curve if one exists (`:292-304`).
- degeneracy: `Degenerated(E, D)` sets the flag (`:1073-1081`).
- **`C1` is PCurve1 = the FORWARD pcurve; `C2` is PCurve2 = the REVERSED pcurve.** There is no
  other convention anywhere in the tree.

### 2.2 The normative constructor: `BRepLib_MakeFace::Init` (natural bounds)

`BRepLib_MakeFace.cxx:463-869`. This is the reference implementation of "build a complete face
on a periodic/degenerate surface". Pseudocode with every constant:

```
Init(S, UMin, UMax, VMin, VMax, TolDegen):                 # TolDegen default = Precision::Confusion() = 1e-7
  eps = Precision::PConfusion()                            # = 1e-9                      (:491)
  BS = (S is RectangularTrimmed) ? S.BasisSurface() : S
  BS.Bounds(umin, umax, vmin, vmax)                                                      (:493)

  if S.IsUPeriodic(): ElCLib::AdjustPeriodic(umin, umax, eps, UMin, UMax)                (:520-523)
  elif UMin > UMax:   swap; if out of [umin,umax] by > eps -> error ParametersOutOfRange (:524-534)
  ... same for V                                                                          (:536-550)

  uclosed = S.IsUClosed() and |UMin-umin| < eps and |UMax-umax| < eps                     (:559-560)
  vclosed = S.IsVClosed() and |VMin-vmin| < eps and |VMax-vmax| < eps                     (:562-563)

  # --- degeneracy of each of the four boundary isos ---
  Cumin = S.UIso(UMin); Dumin = IsDegenerated(Cumin, TolDegen, uminTol)                   (:573-577)
  Cumax = S.UIso(UMax); Dumax = ...                                                       (:578-582)
  Cvmin = S.VIso(VMin); Dvmin = ...                                                       (:583-587)
  Cvmax = S.VIso(VMax); Dvmax = ...                                                       (:588-592)

  # --- four corner vertices, then identification ---
  V00 = Vertex(S(UMin,VMin), max(uminTol, vminTol))   # and V01, V10, V11                 (:597-620)
  if uclosed: V10 = V00; V11 = V01                                                        (:622-626)
  if vclosed: V01 = V00; V11 = V10                                                        (:628-632)
  if Dumin:   V00 = V01     # the whole u=UMin side is one point                          (:634-637)
  if Dumax:   V10 = V11                                                                   (:638-641)
  if Dvmin:   V00 = V10                                                                   (:642-645)
  if Dvmax:   V01 = V11                                                                   (:646-649)

  # --- the four boundary pcurves are straight lines, ALWAYS in +V / +U direction ---
  Lumin = Line(Pnt2d(UMin,0), Dir2d::Y)   # u = UMin, runs +V                             (:653-656)
  Lumax = Line(Pnt2d(UMax,0), Dir2d::Y)                                                   (:657-660)
  Lvmin = Line(Pnt2d(0,VMin), Dir2d::X)   # v = VMin, runs +U                             (:661-664)
  Lvmax = Line(Pnt2d(0,VMax), Dir2d::X)                                                   (:665-668)

  F = MakeFace(S, Precision::Confusion())                                                 (:672)

  # --- u = UMin edge (becomes the SEAM when uclosed) ---
  eumin = Dumin ? MakeEdge()          # NO 3D curve
                : MakeEdge(Cumin, uminTol)                                                (:679-686)
  if uclosed: UpdateEdge(eumin, Lumax, Lumin, F, max(uminTol,umaxTol))   # PC1=Lumax!      (:687-690)
  else:       UpdateEdge(eumin, Lumin,        F, uminTol)                                 (:691-694)
  Degenerated(eumin, Dumin)                                                                (:695)
  Add(eumin, V00.Oriented(FORWARD));  Add(eumin, V01.Oriented(REVERSED))                   (:696-705)
  Range(eumin, VMin, VMax)                                                                 (:706)

  # --- u = UMax edge: SHARED with eumin when closed ---
  if uclosed: eumax = eumin                                                                (:711-714)
  else:       build separately with Lumax, Dumax, V10/V11, Range(VMin,VMax)                (:715-738)

  # --- v = VMin edge (becomes the SEAM when vclosed) ---
  evmin = Dvmin ? MakeEdge() : MakeEdge(Cvmin, vminTol)                                    (:743-750)
  if vclosed: UpdateEdge(evmin, Lvmin, Lvmax, F, max(vminTol,vmaxTol))   # PC1=Lvmin!      (:751-754)
  else:       UpdateEdge(evmin, Lvmin,        F, vminTol)                                  (:755-758)
  Degenerated(evmin, Dvmin); Add V00 FORWARD, V10 REVERSED; Range(evmin, UMin, UMax)       (:759-770)
  if vclosed: evmax = evmin                                                                (:775-777)

  # --- the wire: exactly one CCW loop of the UV rectangle ---
  eumin.Orientation(REVERSED)                                                              (:806)
  evmax.Orientation(REVERSED)                                                              (:807)
  W = Wire(eumin, evmin, eumax, evmax)   # in that order                                   (:838-855)
  Add(F, W); W.Closed(all four finite); F.Closed(uclosed and vclosed)                      (:855-857)
```

Read the wire: `eumin` REVERSED at `u=UMin` runs `(UMin,VMax) → (UMin,VMin)`; `evmin` FORWARD at
`v=VMin` runs `(UMin,VMin) → (UMax,VMin)`; `eumax` FORWARD at `u=UMax` runs
`(UMax,VMin) → (UMax,VMax)`; `evmax` REVERSED at `v=VMax` runs `(UMax,VMax) → (UMin,VMax)`.
That is the rectangle traversed **counter-clockwise**, and since (when `uclosed`)
`eumax` *is* `eumin`, the FORWARD occurrence sits at `UMax` and the REVERSED at `UMin` — which
is exactly `UpdateEdge(eumin, Lumax, Lumin, …)`, PC1 = `Lumax`. S2 and S3 fall straight out.

**`IsDegenerated`** (`BRepLib_MakeFace.cxx:392-459`) — the only degeneracy test OCCT uses at
construction time. It is *type-based*, not sampling-based:

```
IsDegenerated(C, maxTol, &actTol):
  if C is Circle:   if radius > maxTol: return false
                    actTol = max(radius, Precision::Confusion()); return true              (:401-410)
  if C is BSpline:  for i in 2..NbPoles: if |P_i - P_1|^2 > maxTol^2: return false
                    actTol = max(1.000001*sqrt(maxSpread2), Confusion()); return true      (:411-433)
  if C is Bezier:   identical to BSpline                                                   (:434-456)
  return false                                                                             (:458)
```

Note the constant `1.000001` (`:431, :454`) — a deliberate 1 ppm inflation so the recorded
tolerance strictly covers the measured spread. **[OCCT GIVES UP]**: for any other curve type
(line, ellipse, analytic, offset, trimmed) `IsDegenerated` returns `false` outright; a collapsed
iso of such a type is simply not detected here. `IntTools_FClass2d` re-detects it later by
sampling (§2.9).

### 2.3 Independent confirmation: the primitive builders

`BRepPrim_OneAxis` builds every surface of revolution primitive and reaches the same
representation by a different route — useful as a cross-check that the rule is not an accident
of `MakeFace`.

- `MeridianOnAxis(V) := |MeridianValue(V).X()| < Precision::Confusion()` (`:230-233`) — "this
  end of the meridian sits on the axis", i.e. **this side is a pole**.
- `MeridianClosed()` := `MeridianValue(VMin).IsEqual(MeridianValue(VMax), Confusion())`
  (`:237-248`) — the torus case, both v-ends coincide.
- `HasSides() := 2π − myAngle > Precision::Angular()` (`:333`) — a *partial* revolution has real
  start/end faces and **no seam**; a full revolution has `ESTART == EEND` (`:976-979`,
  `:1028-1031`) i.e. one shared edge.
- Lateral face pcurves (`:389-439`):
  - meridian closed (torus): `SetPCurve(ETOP, F, lin(0,myVMin)·X, lin(0,myVMax)·X)` — v-seam,
    **PC1 at VMin** (`:392-395`).
  - full revolution: `SetPCurve(ESTART, F, lin(myAngle,−offset)·Y, lin(0,−offset)·Y)` — u-seam,
    **PC1 at UMax = myAngle** (`:435-438`).
  - partial revolution: two separate single-pcurve edges at `u=0` and `u=myAngle` (`:424-431`).
- Lateral wire (`:660-684`): `AddWireEdge(TopEdge, false)`, `AddWireEdge(EndEdge, true)`,
  `AddWireEdge(BottomEdge, true)`, `AddWireEdge(StartEdge, false)`, where the third argument is
  `direct` and `direct == false` means **reverse** (`BRepPrim_Builder.cxx:184-192`). So the
  stored orientations are: TOP REVERSED, END(=seam) FORWARD, BOTTOM FORWARD, START(=same seam)
  REVERSED — the same CCW rectangle, same PC1-at-UMax assignment.
- `SetPCurve(E, F, L1, L2)` forces the edge to FORWARD before writing and then marks the pair
  `GeomAbs_CN` continuous across itself (`BRepPrim_Builder.cxx:102-118`).
- Degenerate edges: `MakeDegeneratedEdge(E)` = `MakeEdge(E)` (no curve) + `Degenerated(E,true)`
  (`BRepPrim_Builder.cxx:73-77`). Used for the pole ring when `MeridianOnAxis(VMax)`
  (`BRepPrim_OneAxis.cxx:1210-1213`) and for the axis edge of a zero-height wedge (`:929-936`).
  The pole edge gets its single vertex twice with range `[0, myAngle]` via
  `AddEdgeVertex(E, V, 0., myAngle)` (`:1218`), whose body adds `V` FORWARD **and** REVERSED and
  sets the range (`BRepPrim_Builder.cxx:159-170`).

Consequence — the canonical counts:

| primitive | faces | edges | vertices | co-edges (trims) |
|---|---|---|---|---|
| full cylinder | 3 (lateral + 2 caps) | 3 (top circle, bottom circle, **seam**) | 2 | 6 (4 lateral + 1 + 1) |
| full sphere | 1 | 3 (**2 degenerate poles** + **seam**) | 2 | 4 |
| full cone | 2 (lateral + base) | 2 (base circle + **seam**) + 1 degenerate apex | 2 | 5 |
| full torus | 1 | 2 (**u-seam** + **v-seam**) | 1 | 4 |

### 2.4 Detecting closedness of an existing face

Three independent detectors, used in different places; port all three because they answer
different questions.

**(a) Is this *surface* closed?** `GeomLib::IsClosed(S, Tol, &isU, &isV)`
(`GeomLib.cxx:2693-2850`) — type-dispatched:
- Plane → both false (`:2713-2715`).
- Cylinder / SurfaceOfExtrusion → compare `S(u1,v1)` with `S(u2,v1)`, `v1` clamped to 0 if
  infinite (`:2716-2733`).
- Cone → pick the `v` **farthest from the apex**, compare `S(u1,v)` with `S(u2,v)`
  (`:2734-2755`).
- Sphere → pick `v = 0` if the v-range straddles the equator, else the end with larger `|v|`
  (i.e. the largest circle), compare (`:2756-2773`).
- Torus → `isU = (u2-u1) >= UPeriod - UResolution(Tol)`, `isV` likewise (`:2774-2781`).
- BSpline/Bezier → `IsBSplUClosed` / `IsBzUClosed` on the CV net (`:2782-2793`).
- Revolution / Offset / Other → 23-sample sweep in `v` comparing `S(u1,t)` with `S(u2,t)` against
  `Tol²`, with the sample count raised when `dt` falls below `UResolution(Tol)` (`:2794-2850`).

**(b) Is this *edge* a seam of this face?** `BRep_Tool::IsClosed(E, F)` (`:795-805`) →
`IsClosed(E, S, L)` (`:814-841`): returns true iff some representation on `(S,L)` reports
`IsCurveOnClosedSurface()`. **Planes short-circuit to false** (`:819-822`) — a plane can never
carry a seam. There is also a triangulation variant (`:849-874`).

**(c) Is it *really* a seam?** `BRepTools::IsReallyClosed(E, F)` (`:1204-1220`) = (b) **and**
`E` occurs exactly twice in `F`. `BRepTools::DetectClosedness(F, &uClosed, &vClosed)`
(`:1224-1251`) then decides the *direction* by comparing the two pcurves at the same parameter:
`IsUiso = |P1.X − P2.X| > |P1.Y − P2.Y|` (`:1240`) — the pair is separated in whichever
coordinate moves more.

BOP uses a fourth composite in `BOPAlgo_Builder_2.cxx:387-404`: `GeomLib::IsClosed(surface,
tol(edge))` for the surface, `BRep_Tool::IsClosed(E,F)` for the edge, and
`BOPTools_AlgoTools2D::IsEdgeIsoline(E, F, &isU, &isV)` to check the seam runs along the right
iso, then `bIsClosed = (isUClosed && isUIso) || (isVClosed && isVIso)`. `IsEdgeIsoline`
(`BOPTools_AlgoTools2D.cxx:669-700`) evaluates the pcurve tangent at the mid-parameter,
normalises it, and tests `crossMagnitude(T, (0,1)) <= Precision::Angular()` for U-iso and
`crossMagnitude(T, (1,0)) <= Precision::Angular()` for V-iso, with `Precision::Angular() = 1e-12`
(`:693`).

### 2.5 Face UV bounds in the presence of seams and periodicity

`BRepTools::AddUVBounds(F, B)` (`:126-157`): union the boxes of all edges' pcurves; **if the
result is void** (a face with no pcurves at all) fall back to the surface's natural bounds
(`:141-153`). This is the reason a naked periodic surface still has a usable UV window.

`BRepTools::AddUVBounds(F, E, B)` (`:172-367`) is the per-edge worker and contains the only
place OCCT tries to *recover* periodicity that the surface flag does not declare:

- Get the pcurve box `[aXmin,aXmax]×[aYmin,aYmax]` via `BndLib_Add2dCurve::Add(C2D, t1, t2, 0.,
  box)` (`:185`).
- If `!S->IsUPeriodic()` **and** `S` is a `Geom_BSplineSurface` **and** the pcurve leaves
  `[umin,umax]`: test whether the surface is *effectively* U-periodic (`:210-268`):
  1. if `!IsUClosed()`, compare `S(umin,v)` with `S(umax,v)` at `v = vmin` and `v = vmax` against
     `aTol2 = 100 * Precision::Confusion()²` (`:212`, i.e. distance 1e-6);
  2. then compare 3 or 6 probe points *outside* the domain with their `±period` images against
     the same `aTol2` (`:231-267`).
- If not periodic in that direction, **clamp** the box to the natural bounds (`:270-280`, and
  `:351-361` for V). If periodic, leave the out-of-range extent alone.

`BRepTools::Update(F)` → `UpdateFaceUVPoints(F)` (`:383-390`, `:488-523`) recomputes the cached
UV end points of every edge's representation on that face by calling `GC->Update()`, which for a
`BRep_CurveOnClosedSurface` updates **both** pairs (`BRep_CurveOnClosedSurface.cxx:123-134`).
Call this after any seam edit.

`BRepAdaptor_Surface::Initialize(F, Restriction=true)` loads the surface **restricted to
`BRepTools::UVBounds(F)`** (`:56-81`). Every subsequent `FirstUParameter()`/`LastUParameter()`
therefore reports the *face's* window while `IsUPeriodic()`/`UPeriod()` still report the
*surface's* period (`GeomAdaptor_Surface.cxx:946-956`). All of §2.7's arithmetic depends on that
split.

### 2.6 Degenerate edges — every place the algorithms special-case them

1. **DS preparation.** `BOPDS_DS::prepareEdges` (`:1614-1692`): non-degenerate edges get
   infinite-parameter vertices synthesised (`:1630-1671`); degenerate edges instead get
   `SetFlag(edgeIndex)` (`:1672-1675`). `prepareFaces` (`:1696-1779`) overwrites that flag with
   the *face* index for every degenerate edge it meets (`:1743-1746`) — the flag doubles as
   "which face owns this pole".
2. **Pave blocks.** `BOPDS_DS::InitPaveBlocks` (`:440-500`): for a degenerate edge the vertex
   pave is appended with `AppendExtPave1` **unconditionally** (`:467-471`), bypassing the
   normal duplicate check. Immediately after, the *closed-edge* rule: if the edge has exactly
   one vertex, a second pave is appended for the **reversed** vertex occurrence
   (`:476-484`) — this is how a seam circle or a pole ring gets two paves from one vertex.
3. **Interference stages** all skip flagged edges — see S6 for the eight call sites.
4. **The dedicated stage `ProcessDE`** — `BOPAlgo_PaveFiller_8.cxx:54-131`, described in §2.10.
5. **Face assembly.** `BOPAlgo_Builder_2.cxx:406, 413-418`: a degenerate edge's splits are
   appended **once**, keeping the original orientation — never twice as a seam would be. In the
   draft-face path, `:1155-1159` likewise. `HasMultiConnected` is skipped for degenerate edges
   (`:1122, :1143`).
6. **Wire splitting.** `BOPAlgo_WireSplitter_1.cxx:146`:
   `bIsClosed = Degenerated(E) || IsClosed(E, myFace)` — a degenerate edge is treated as "closed"
   so that appearing once with both ends on the same vertex is legal. `:439`: a wire may not be
   assembled from degenerate edges alone (`bHasEdge = !Degenerated(aEPrev)`).
7. **Face building.** `BOPAlgo_BuilderFace.cxx:198-203`: a vertex with only one incident edge is
   normally pruned into `myShapesToAvoid`; **not** if that edge is degenerate. `:864-868`:
   degenerate edges are skipped when classifying a wire against a face.
8. **2D classification.** `IntTools_FClass2d.cxx:167-170`: degenerate **or** seam edges are
   excluded from the classification polygon. `:176-187`: an edge with a null vertex is also
   treated as degenerate.
9. **Closure.** `BOPAlgo_BOP.cxx:1479-1484`: degenerate edges are skipped in the closed-shell
   check (S7).
10. **Validity.** `BRepCheck_Edge.cxx:131-147` (no 3D curve ⟺ degenerate),
    `:534` (`nbconnection < 2` is not an error for a degenerate edge), `:622` (SameParameter is
    not checked); `BRepCheck_Wire.cxx:775-780, :810-816` (a wire whose first oriented edge has
    neither FORWARD nor REVERSED vertex is `InvalidDegeneratedFlag`), `:866, :879, :1964`.
11. **Vertex parameters.** `BRep_Builder::UpdateVertex` (`:1249-1252` and `:1351-1354`): if the
    edge has no vertices *yet* and is degenerate, the orientation is taken from the vertex
    itself — the special case that lets a degenerate edge be built vertex-last.
    `BRep_Tool::Parameter` (`:1604`) accepts a degenerate edge as a valid host even though
    `Curve(E)` is null.

### 2.7 Adjusting a pcurve onto a periodic face — `AdjustPCurveOnSurf`

`BOPTools_AlgoTools2D.cxx:247-400`. Input: the face-restricted `BRepAdaptor_Surface` (so
`UMin..VMax` are the **face's** bounds, §2.5), the 3D parameter interval `[aFirst,aLast]`, the
freshly projected pcurve. Output: a copy translated by whole periods.

```
aDelta = Precision::PConfusion()                                    # 1e-9              (:263)
aT = 0.5*(aFirst + aLast);  (u2,v2) = C2D(aT)                                          (:265-271)

du = 0
if surface IsUPeriodic:                                                                 (:275)
    T = UPeriod()
    if |u2 - UMin| < aDelta:            u2 = UMin                                       (:281-284)
    elif |u2 - UMin - T| < aDelta:      u2 = UMin + T                                   (:285-288)
    GeomInt::AdjustPeriodic(u2, UMin, UMax, T, u2, du, 0.)                               (:290)
    if du == 0 and surface is Cylinder:                # special case, see below         (:292-314)
        dFi = max(maxEdgeTol(F)/R, aDelta)
        if UMin - u2 > dFi:  du = +T
        elif u2 - UMax > dFi: du = -T

dv = 0
if surface IsVPeriodic:                                                                 (:319)
    T = VPeriod()
    if VMin - v2 > aDelta: dv = +T   elif v2 - VMax > aDelta: dv = -T                    (:324-330)
    if (VMax - VMin < T) and dv != 0:                  # partial v-window: pick nearer   (:332-343)
        if |v2 - mid| < |v2+dv - mid|: dv = 0

# final safety: if the face window is WIDER than one period, the shift may have pushed
# the point outside the real trimmed area -> classify and undo
u = u2+du; v = v2+dv
if IsUPeriodic and (UMax-UMin-2*aDelta) > T:                                            (:351-367)
    if u > UMin+aDelta+T or u < UMax-aDelta-T:
        if BRepClass_FaceClassifier(F, (u,v), aDelta).State() == OUT:
            du += (u > UMin+aDelta+T) ? -T : +T
... same for V                                                                          (:370-386)

if du != 0 or dv != 0: C2DA = C2D.Copy().Translate(gp_Vec2d(du,dv))                      (:391-397)
```

Two things to carry over verbatim:
- the **cylinder-only** rescue at `:294-313` uses an angular tolerance derived from the largest
  edge tolerance on the face divided by the cylinder radius (`MaxToleranceEdge`,
  `BOPTools_AlgoTools2D.cxx:648-665`) — a *model-space* tolerance converted to UV at the point
  of use, exactly the doctrine in `kb/ARCHITECTURE_v2.md §3`;
- the classifier re-check at `:346-387` only fires when the face's UV window is *wider* than one
  period, which happens after a STEP round-trip.

`GeomInt::AdjustPeriodic` (`GeomInt.cxx:21-48`) is the primitive:

```
AdjustPeriodic(par, parMin, parMax, period, &newPar, &offset, eps=0):
  offset = 0; newPar = par
  bMin = (parMin - par > eps);  bMax = (par - parMax > eps)
  if bMin or bMax:
     dp = bMin ? (parMax - par) : (parMin - par)
     modf(dp/period, &nPer)                      # TRUNCATE toward zero
     offset = nPer*period;  newPar = par + offset
  return offset > 0
```

Note the return value is `offset > 0`, **not** `offset != 0` — a negative shift reports `false`.
Callers that only test the boolean will believe nothing happened. Port the out-parameters, not
the return value.

Where the pcurve is *built*: `BOPTools_AlgoTools2D::MakePCurveOnFace` (`:501-644`) projects the
3D curve with `ProjLib_ProjectedCurve` and then, at `:611-614`, **always** runs
`AdjustPCurveOnSurf`. It then repairs the parameter range with `GeomLib::SameRange` if the
projected pcurve's domain does not cover `[aT1,aT2]` (`:616-631`), and finally recomputes the
edge tolerance with `IntTools_Tools::ComputeTolerance` (`:633-643`). Surfaces of revolution take
a simplified branch (`:530-540`); everything else may extend the surface by 1% in the
**non-periodic** directions before projecting (`:566-585` — note `:570` and `:576` explicitly
refuse to extend a direction that already spans a full period).

`BOPTools_AlgoTools2D::BuildPCurveForEdgeOnFace` (`:48-70`) is the entry point: if the edge
already has a pcurve on the face, **do nothing** (`:57-61`) — existing pcurves are authoritative
and are never re-projected.

### 2.8 A section curve that CROSSES a seam

OCCT splits it in three independent places; all three are needed.

**(1) Restriction to the face domain — the split happens because the seam *is* a boundary.**
`GeomInt_LineConstructor::Perform` (`GeomInt_LineConstructor.cxx:114-…`) walks the intersection
line vertex-to-vertex, takes the midpoint of each interval, calls the local `AdjustPeriodic`
(`:737-…`) to bring `(u,v)` of both surfaces into their windows, and classifies with
`myDom1/myDom2->Classify(p2d, Tol)` where `Tol = Precision::PConfusion()*35 = 3.5e-8` (`:118`).
Only intervals classified `!OUT` on **both** surfaces are kept (`:137-147`). The local
`AdjustPeriodic` hard-codes `2π` as the period for Cylinder/Cone/Sphere/Torus (`:749-…`) and
treats every other type as non-periodic ("Case of periodic biparameters is processed upstream").
**[OCCT FALLBACK]** — a periodic B-spline surface gets no period here.

**(2) `IntTools_WLineTool::DecompositionOfWLine`** (`IntTools_WLineTool.cxx:492-1120`) — the
explicit seam decomposer. Structure:

```
aTol = 0.5 * Precision::Confusion()                     # = 5e-8                       (:533-534)
# Pass 1: tag every WLine point as ON-boundary or not                                  (:541-629)
for each point:
   for surface in {S1, S2}:
      for dir in {U, V}:
         if not periodic in dir: continue                                              (:567-571)
         res = (dir==U) ? UResolution(aTol) : VResolution(aTol)                        (:575-587)
         GeomInt::AdjustPeriodic(par, lo, hi, period, &adj, &off)                       (:590-595)
         if |adj - lo| < res or |adj - hi| < res:  point is ON boundary                (:598-608)
   # cut the point list wherever the ON/OFF flag flips
   if onBoundary != prevOnBoundary: close current run, start a new one                 (:617-627)
if number_of_runs <= 1: return false        # nothing to decompose                     (:639-642)
# Pass 2: for each OFF-boundary run, recompute its two end points so they land exactly
#         on the boundary, choosing the branch nearest the neighbour                   (:649-1010)
```

The end-point recomputation is the interesting part:
- `nbboundaries == 2` (a corner) → recompute unconditionally (`:775-780`).
- `nbboundaries == 1` on a periodic direction → compute the *mirror* parameter
  `anotherPar = isFirst ? (upper - dist) : (lower + dist)` plus the recorded offset
  (`:799-803`), then decide between the point and its mirror by comparing distances to the
  neighbour **and** by an angle test: accept the mirror only if the 2D vector to it makes an
  angle `< π/4` with the previous segment and has positive dot product (`:875-901`), and only
  if the resulting 3D points on both surfaces agree within `aTol` (`:882-894`). The distance
  gate is `adist2 < period/4` (`:826`, `:837`).
- "near boundary" → `anEpsilon = 100*resolution`, capped at 10% of the span (`:760-762`), then
  `AdjustByNeighbour` (`:292-336`) picks, among `{p, p±period}` in each periodic direction, the
  copy closest to the neighbour point; if that lands outside the rectangle, `FindPoint`
  (`:378-488`) intersects the segment with the four sides and returns the crossing.
- If a recomputed point projects onto the other surface farther than `theTol`, the point is
  **dropped** (`:1023-1026`) — **[OCCT FALLBACK]**, the run simply loses its end.

**(3) Detecting that a *segment* crosses a seam at all.**
`IntPatch_WLineTool::IsSeamOrBound` (`IntPatch_WLineTool.cxx:636-725`) works on the 4-vector of
parameters `(u1,v1,u2,v2)` with `theArrPeriods[4]`:
- for each of the 4 parameters build `Bnd_Range` over `[parF, parL]` and test intersection with
  the first/last bound **modulo the period** (`:650-664`);
- **the decisive heuristic**: `if 2*|parL − parF| > period → "Most likely, seam is intersected"`
  (`:674-679`);
- also test intersection with `0.0` modulo the period (`:681-684`);
- finally test the *mid* point against the same three positions (`:702-722`) to catch a segment
  that straddles without either end being near.

Then the pcurves for each piece are produced by `MakePCurveOnFace` (§2.7), which translates each
piece independently onto the face window. **The two pieces produced at a seam crossing meet at
two UV points exactly one period apart with the same 3D point** — which is S8.

**Making the split of an existing seam edge a seam again** is a separate operation, §2.9.

### 2.9 A section (or split) curve that runs **ALONG** a seam

This is the `DoSplitSEAMOnFace` family — the single most important routine in this document to
port, because it is the only code that *creates* a two-pcurve representation after the fact.

**Variant A — geometric** (`BOPTools_AlgoTools3D.cxx:58-232`). Input: a split edge `aSplit` that
already carries **one** pcurve on `aF`, and the face.

```
aSp = aSplit.Oriented(FORWARD); aTol = Tolerance(aSp)                                   (:74-77)
aS = Surface(aF); aS->Bounds(aUmin,aUmax,aVmin,aVmax)                                   (:79-81)
bIsUPeriodic = aS->IsUClosed();  bIsVPeriodic = aS->IsVClosed()                         (:84-85)
anUPeriod = aUmax-aUmin (if U);  anVPeriod = aVmax-aVmin (if V)                         (:87-94)

if neither:                                                                             (:97-147)
    aS must be Geom_RectangularTrimmedSurface else return FALSE                         (:100-106)
    take BasisSurface; use IsUPeriodic/IsVPeriodic and their UPeriod/VPeriod            (:112-120)
    else if basis IsUClosed and the trim spans the basis bounds within aTol -> periodic (:123-139)
    if still neither: return FALSE                                                      (:142-145)

C2D1 = CurveOnSurface(aSp, aF, a, b)                                                    (:150)
aT   = IntermediatePoint(a, b)        # = 0.56786082*a + 0.43213918*b                   (:152)
C2D1->D1(aT, aP2D, aVec2D);  aDir2D1 = dir(aVec2D)                                      (:153-154)
dU = GeomAdaptor_Surface(aS).UResolution(aTol);  dV = VResolution(aTol)                 (:162-164)

anU1 = anU; anV1 = anV
if U-periodic: if |anU-aUmin| < dU: bIsLeft=true,  anU1 = anU + anUPeriod                (:166-172)
               elif |anU-aUmax| < dU: bIsLeft=false, anU1 = anU - anUPeriod              (:173-177)
if V-periodic: if |anV-aVmin| < dV: bIsLeft=true,  anV1 = anV + anVPeriod                (:180-185)
               elif |anV-aVmax| < dV: bIsLeft=false, anV1 = anV - anVPeriod              (:186-191)
if nothing moved: return FALSE          # the split is NOT on the seam                   (:194-197)

aScPr = (anU1 == anU) ? aDir2D1 · (-1,0)      # V-seam: minus the U-component
                      : aDir2D1 · ( 0,1)      # U-seam: the V-component                  (:199)

aC1 = TrimmedCurve(copy(C2D1), a, b)                                                     (:201-202)
aC2 = TrimmedCurve(copy(C2D1), a, b).Translate(anU1-anU, anV1-anV)                       (:204-207)

if !bIsLeft:   UpdateEdge(aSp, (aScPr<0 ? aC2 : aC1), (aScPr<0 ? aC1 : aC2), aF, aTol)   (:209-219)
else:          UpdateEdge(aSp, (aScPr<0 ? aC1 : aC2), (aScPr<0 ? aC2 : aC1), aF, aTol)   (:220-230)
return TRUE
```

Work the four combinations through and you get exactly the table in S3. `IntermediatePoint(f,l)
= (1-PAR_T)*f + PAR_T*l` with `PAR_T = 0.43213918` (`BOPTools_AlgoTools2D.cxx:404-411`) — an
irrational-looking constant chosen so the probe never lands on a knot or a symmetry point.

**Variant B — projection onto the original seam** (`BOPTools_AlgoTools3D.cxx:236-327`), tried
when Variant A returns false:

```
if !IsClosed(theEOrigin, theFace): return FALSE                                          (:240-243)
if  IsClosed(theESplit,  theFace): return TRUE      # already done                       (:245-248)
aC2DSplit = CurveOnSurface(split.FORWARD, face.FORWARD)                                  (:256-261)
aC2D1 = CurveOnSurface(origin.FORWARD, face);  aC2D2 = CurveOnSurface(origin.REVERSED, face)  (:263-267)
aPMid, aVTgt = aC2DSplit->D1(IntermediatePoint(aTS1,aTS2))                               (:269-272)
project aPMid on aC2D1 and on aC2D2                                                      (:275-285)
if both distances > Precision::PConfusion(): return FALSE                                (:287-290)
aNewPnt = value of the OTHER original pcurve at the winning parameter                    (:292-294)
aC2 = copy(aC2DSplit).Translate(aPMid -> aNewPnt)                                        (:296-303)
aDot = aVTgt · tangent(winning original pcurve)                                          (:305-316)
if ((aDist1 < aDist2) == (aDot > 0)): UpdateEdge(split, aC1, aC2, ...)                   (:318-321)
else:                                 UpdateEdge(split, aC2, aC1, ...)                   (:322-325)
```

i.e. "PCurve1 of the split must sit on the same side, and run the same way, as PCurve1 of the
original seam".

**Variant C — legacy** `UpdateClosedPCurve` (`BOPTools_AlgoTools2D_1.cxx:164-287`), reached from
`BOPTools_AlgoTools2D::AttachExistingPCurve`. It derives the translation vector directly as
`aV2DS12 = aP2DS1 → aP2DS2` (the vector between the two original pcurves at a common parameter,
`:204-210`), decides U- vs V-closedness by `|aD2DS12 · DX| < aTol` (`:214-229`), reverses the
vector if the new pcurve sits on the *second* side (`:250-259`), and orders the pair by
`aScPr = aV2D · aV2DS1 < 0 → swap` (`:266-278`). Same rule, third derivation.

**Callers** (`BOPAlgo_Builder_2.cxx`):
```
if bIsClosed:                                                                     (:429)
    if the split is not already closed on the face:                               (:433)
        if !DoSplitSEAMOnFace(aSp, aF):                                           (:435)
            if !DoSplitSEAMOnFace(aE, aSp, aF):                                   (:438)
                AddWarning(AlertUnableToMakeClosedEdgeOnFace)     # [OCCT FALLBACK] (:440-445)
    append aSp FORWARD and aSp REVERSED                                           (:449-452)
```
and the same three-step ladder at `:1163-1166` for the draft-face path. Note the fence map
`aMFence` (`:431`) — each split is turned into a seam **once**, no matter how many times it
appears.

**What OCCT does NOT do**: it never creates a *new* section edge lying along a seam. A section
curve coincident with an existing edge is absorbed into the existing pave block as a common
block (`BOPAlgo_PaveFiller_6.cxx:922, 963` `IsExistingPaveBlock`, and
`ProcessExistingPaveBlocks` `:3072`, `:3171-3260`, which validates coincidence with
`myContext->ComputeVE(aV, aE, aT, dummy, myFuzzyValue)` at `:3245`). So "section along the seam"
degenerates to "the seam is already there, reuse it" — which is only representable if seams are
first-class entities in the DS.

### 2.10 The dedicated pole stage — `ProcessDE`

`BOPAlgo_PaveFiller_8.cxx:54-131`. Runs after all interference stages. For every edge whose DS
flag is set (i.e. degenerate) and whose flag value `nF` names a **face**:

```
nV = the degenerate edge's single vertex (mapped through same-domain)                   (:73-77)
FindPaveBlocks(nV, nF, &aLPBOut)     # every pave block of face nF touching nV           (:84, 135-159)
if aLPBOut not empty:
    aPBD = the degenerate edge's own (single) pave block
    FillPaves(nV, edge, nF, aLPBOut, aPBD)                                               (:98)
    myDS->UpdatePaveBlock(aPBD)                                                          (:100)
MakeSplitEdge(edge, nF)                                                                  (:103)
```

`FillPaves` (`:224-331`) is where the pole's *free parameter* is recovered:

```
aTolV  = Tolerance(pole vertex)                                                          (:238)
aURes  = SurfaceAdaptor(aDF).UResolution(aTolV);  aVRes = VResolution(aTolV)             (:245-247)
aTolInt = max(Precision::PConfusion(), max(aURes, aVRes))     # 2D intersection tol      (:243-249)
aC2DDE = CurveOnSurface(degenerate edge, face)                                           (:257)
bUDir  = |aC2DDE(t1).Y - aC2DDE(t2).Y| < Precision::PConfusion()   # the pole line is U-directed (:264-265)
aTolCmp = max(Precision::PConfusion(), bUDir ? aURes : aVRes)                            (:254-267)
for each pave block through the vertex:
    aC2D = its pcurve on the face                                                        (:284)
    Geom2dInt_GInter(adaptor(aC2DDE), adaptor(aC2D), aTolInt, aTolInt)                   (:303)
    if points found: for each, AddSplitPoint(aPBD, pave@ParamOnFirst, aTolCmp)           (:304-314)
    else:            project the pave-block end point onto the degenerate pcurve and use
                     the projection parameter                    # [OCCT FALLBACK]        (:315-329)
```

`AddSplitPoint` (`:368-393`) rejects a parameter within `aTolCmp` of either end of the block
(`:377-380`) or duplicating an existing one (`:383-387`), else appends it as an **extra pave**.

`MakeSplitEdge` (`:163-214`) then materialises the pieces with `MakeSplitEdge1` (`:335-358`):

```
E = aE.EmptyCopy();  Add(E, aV1); Add(E, aV2);  Range(E, aF, aP1, aP2);
Degenerated(E, true);  UpdateEdge(E, 1.e-7)          # hard-coded tolerance              (:343, :354-356)
```

Two facts worth transcribing exactly: the split degenerate edge's range is set **on the face**
(`Range(E, aF, p1, p2)`, `:352`) because there is no 3D curve to range against; and the tolerance
is the literal `1.e-7` (`:343`), not `Precision::Confusion()` — numerically equal today but
written as a literal.

If the flagged partner `nF` is an **edge** rather than a face (a degenerate edge sitting on
another edge), OCCT builds a brand new degenerate edge holding only the new vertex and rebinds
the pave block to it (`:105-121`).

### 2.11 Building the pcurve of a curve that ENDS at a pole

At a pole, `dS/du × dS/dv = 0`: there is no normal, no unique UV pre-image, and projection is
ill-conditioned. OCCT's answer, in order:

1. **Detect the pole zone.** `IntTools_WLineTool::IsDegeneratedZone(p2d, S, iDir)`
   (`:50-122`): step `dX = dY = 1e-5` in parameter on either side of the point (clamped to the
   surface bounds) and declare the zone degenerate iff **both** 3D displacements are below
   `dD = 1e-12` (`:66-68`, `:87-92`, `:113-118`). `IsPointInDegeneratedZone` (`:128-223`) applies
   it at each of the four boundary parameters of each surface with `aDelta = 1e-7` and
   `UResolution(aDelta)` (`:149-221`). If either end of a WLine sub-range is in a degenerate
   zone, `NotUseSurfacesForApprox` returns true (`:227-248`) and the approximation is run
   **without** the surfaces — i.e. the pcurves are not fitted through the pole.
   **[OCCT FALLBACK]** — this is an explicit decision to stop using the surface near a pole.
2. **Place the pole as an exact special point.** `IntPatch_SpecialPoints::AddSingularPole`
   (`:806-959`):
   ```
   aUquad = 0.0                                                                          (:828)
   sphere: aVquad = copysign(π/2, aVquad)                                                (:830-833)
   cone:   aVquad = -RefRadius / sin(SemiAngle)                                          (:834-840)
   other:  throw Standard_TypeMismatch          # only sphere and cone have poles        (:841-845)
   aPQuad = QSurf.D0(aUquad, aVquad)                                                     (:847)
   if required and |aPQuad - vertex|^2 >= tol^2: return false                            (:848-852)
   if !IsPointOnSurface(PSurf, aPQuad, tol, aP0, aU0, aV0): return false                 (:854-857)
   addedPoint = midpoint(aP0, aPQuad) with params (aUquad,aVquad) / (aU0,aV0)            (:862-869)
   # now CHOOSE aUquad from the other surface's tangent plane, transformed into the
   # quadric's own frame:
   PSurf.D1(aU0, aV0, _, Du, Dv); transform Du,Dv by Trsf(QSurf.Position())              (:888-899)
   sphere: ProcessSphere(refIso, Du, Dv, reversed, &aVquad, &aUquad, &isIsoChoosen)      (:903-909)
   cone:   ProcessCone  (refIso, Du, Dv, Cone, reversed, &aUquad, &isIsoChoosen)         (:910-922)
   if !isIsoChoosen: AdjustPointAndVertex(refPoint, periods, addedPoint)                 (:939-952)
   ```
3. **Continue past the pole.** `ContinueAfterSpecialPoint` (`:990-1078`). The documented rule
   (comment `:967-988`): crossing a sphere pole flips the quadric's U by `±π`; crossing a cone
   apex either flips by `±π` or requires recomputing `U` from the second intersection line of
   the tangent plane. Then:
   ```
   aPeriod = (type == SPntPole) ? M_PI_2 : 2π                                            (:1063)
   ```
   with the comment (`:1047-1062`) stating plainly that using `π/2` "does not have any
   mathematical idea but allows creating WLine with more or less uniform distributed points" —
   i.e. it forbids jumps larger than `π/4` between neighbouring walking points.
   **[OCCT FALLBACK]** and an admitted hack; transcribe it as such. The comment also records an
   unimplemented case: a second surface that is only C0 at the pole "is not implemented but will
   be able to be done after the corresponding demand" — **[OCCT GIVES UP]**.
4. **Classify a vertex as pole/seam.** `IntPatch_ALineToWLine.cxx:72-141`:
   ```
   for surface in {S1, S2}:
      Sphere or Cone -> try AddSingularPole(...) -> IntPatch_SPntPole                    (:93-105)
      Torus          -> try AddCrossUVIsoPoint(...) -> IntPatch_SPntSeamUV               (:108-121)
      Cylinder (and fallthrough from the above) -> AddVertexPoint -> IntPatch_SPntSeamU   (:123-126)
   ```
   The enumeration itself (`IntPatch_SpecPntType.hxx:22-30`) is the taxonomy to port:
   `SPntNone, SPntPole, SPntSeamU, SPntSeamV, SPntSeamUV, SPntPoleSeamU`.

### 2.12 Periodic parameter normalisation — the two rules

**Rule N1 — canonical window (used once, per finished piece).**
`ElCLib::AdjustPeriodic(UFirst, ULast, Preci, &U1, &U2)` (`ElCLib.cxx:115-149`):

```
if either bound infinite: leave as is                                                   (:121-126)
period = ULast - UFirst;  if period < Epsilon(ULast): leave as is                       (:128-137)
U1 -= floor((U1 - UFirst)/period) * period            # U1 in [UFirst, UFirst+period)   (:139)
if ULast - U1 < Preci: U1 -= period                   # snap off the top bound          (:140-143)
U2 -= floor((U2 - U1)/period) * period                # U2 in [U1, U1+period)           (:144)
if U2 - U1 < Preci: U2 += period                      # a full turn, not a null turn    (:145-148)
```

Post-condition: `U1 ∈ [UFirst, UFirst+period)` and `U2 ∈ (U1, U1+period]`, so `U2 - U1 ∈
(0, period]` — a wrapping interval never collapses to zero. This is the answer to "how U is
brought into `[0,2π)` consistently", and the `if U2 - U1 < Preci: U2 += period` line is the trap:
without it, a full wrap reads as an empty interval.

**Rule N2 — relative unwrap (used everywhere along a curve).**
`IntPatch_SpecialPoints::AdjustPointAndVertex` (`:1082-1128`):

```
for i in 0..3:                       # {u1,v1,u2,v2}
   period = theArrPeriods[i]; if period == 0: continue
   ref = the corresponding parameter of the reference point
   d = ref - par[i];  incr = copysign(period, d)
   while |d| > period/2: par[i] += incr; d = ref - par[i]
```

Post-condition: every parameter is within half a period of the reference. `AdjustByNeighbour`
(`IntTools_WLineTool.cxx:292-336`) is the same rule expressed as an explicit 3-candidate search
over `{p-T, p, p+T}` minimising squared 2D distance to the neighbour, applied independently in U
and V.

**The trap this exists to avoid**: applying N1 per sample. Two consecutive samples at
`u = 2π-ε` and `u = 2π+ε` normalise to `2π-ε` and `ε`, a jump of one full period, and every
downstream consumer sees a curve that traverses the whole domain backwards in one step. Apply N2
during the walk, N1 once per finished piece.

**Period source.** `GeomAdaptor_Surface::IsUPeriodic()/UPeriod()` forward to the underlying
`Geom_Surface` (`:946-956`); the *face* window comes from `BRepTools::UVBounds` via
`BRepAdaptor_Surface::Initialize(F, true)` (`:71-76`). They are different numbers and both are
needed: the period for the arithmetic, the window for the target interval.

### 2.13 Constant table (actual values)

| symbol | value | source |
|---|---|---|
| `Precision::Confusion()` | `1e-7` | `Precision.hxx:165` |
| `Precision::PConfusion()` | `1e-9` (= `Confusion()*0.01`) | `Precision.hxx:334` |
| `Precision::Angular()` | `1e-12` | `Precision.hxx:123` |
| `Precision::Intersection()` | `1e-9` (= `Confusion()*0.01`) | `Precision.hxx:220` |
| `Precision::Approximation()` | `1e-6` (= `Confusion()*10`) | `Precision.hxx:235` |
| default `TolDegen` | `Precision::Confusion()` = `1e-7` | `BRepLib_MakeFace.cxx:288-337` |
| degeneracy tolerance inflation | `1.000001` | `BRepLib_MakeFace.cxx:431, 454` |
| BSpline effective-periodicity probe | `100 * Confusion()²` = `1e-12` (dist `1e-6`) | `BRepTools.cxx:212, 293` |
| `IntermediatePoint` blend `PAR_T` | `0.43213918` | `BOPTools_AlgoTools2D.cxx:407` |
| degenerate-zone probe step | `dX = dY = 1e-5`, threshold `dD = 1e-12` | `IntTools_WLineTool.cxx:66-68` |
| degenerate-zone boundary window | `aDelta = 1e-7` | `IntTools_WLineTool.cxx:149` |
| WLine decomposition tolerance | `0.5 * Confusion()` = `5e-8` | `IntTools_WLineTool.cxx:533-534` |
| near-boundary epsilon | `100 * resolution`, capped at `0.1 * span` | `IntTools_WLineTool.cxx:760-762` |
| mirror-acceptance distance gate | `period/4` | `IntTools_WLineTool.cxx:826, 837` |
| mirror-acceptance angle gate | `π/4` and positive dot | `IntTools_WLineTool.cxx:877` |
| pole continuation pseudo-period | `π/2` | `IntPatch_SpecialPoints.cxx:1063` |
| line-constructor classify tolerance | `Precision::PConfusion()*35` = `3.5e-8` | `GeomInt_LineConstructor.cxx:118` |
| split degenerate edge tolerance | literal `1.e-7` | `BOPAlgo_PaveFiller_8.cxx:343` |
| FClass2d degeneracy re-detection | `0.25*Confusion()²` over 11 samples | `IntTools_FClass2d.cxx:206-227` |
| seam-crossing heuristic | `2*|Δpar| > period` | `IntPatch_WLineTool.cxx:674-679` |

Every one of these is a **model-space** or **dimensionless** quantity except the parametric ones,
which are always obtained through `UResolution(tol3d)` / `VResolution(tol3d)` from a 3D
tolerance. There is no `min(uv_range)*constant` anywhere in this subsystem.

### 2.14 Where OCCT gives up — the honest list

| # | situation | OCCT's behaviour | citation |
|---|---|---|---|
| F1 | split of a seam edge cannot be made closed by either variant | emits `BOPAlgo_AlertUnableToMakeClosedEdgeOnFace` warning and continues with a **one-pcurve** edge → the resulting face has a torn UV loop | `BOPAlgo_Builder_2.cxx:440-445` |
| F2 | `IsDegenerated` on a non-{circle,BSpline,Bezier} iso | returns `false`; degeneracy not detected at construction | `BRepLib_MakeFace.cxx:458` |
| F3 | UV of a vertex on a seam or pole | `BRep_Tool::Parameters` returns whichever pcurve end it finds first; the source comment says "Ambiguity (natural) for degenerated edges" | `BRep_Tool.cxx:1698` |
| F4 | pole on a surface that is not a sphere or a cone | `AddSingularPole` throws `Standard_TypeMismatch` | `IntPatch_SpecialPoints.cxx:841-845` |
| F5 | second surface only C0 at the pole | "not implemented but will be able to be done after the corresponding demand" | `IntPatch_SpecialPoints.cxx:1050-1054` |
| F6 | crossing a pole with high curvature | uses a **fake** period `π/2` "does not have any mathematical idea" | `IntPatch_SpecialPoints.cxx:1055-1063` |
| F7 | periodic B-spline in the line constructor | treated as non-periodic ("processed upstream") | `GeomInt_LineConstructor.cxx:761-766` |
| F8 | a recomputed WLine end point that does not project back onto the other surface within tolerance | the point is dropped, the run loses its end | `IntTools_WLineTool.cxx:1023-1026` |
| F9 | `GeomInt::AdjustPeriodic` return value | reports `false` for a *negative* shift; only the out-parameters are trustworthy | `GeomInt.cxx:47` |
| F10 | a pave block whose 2D intersection with the pole line fails | falls back to projecting the block end point onto the degenerate pcurve | `BOPAlgo_PaveFiller_8.cxx:315-329` |

---

## 3. DATA STRUCTURES AND C++ DECLARATIONS FOR OUR PORT

Fits `kb/ARCHITECTURE_v2.md §1`: the periodicity descriptor is per-surface, computed once at
prepare time (stage 0) and stored in the arena; seam/pole facts are per-(edge,face) and live in
the trim.

```cpp
// ---- src/brep_periodic.h  (new) -------------------------------------------------------

enum class ParamDir { U = 0, V = 1 };

// Computed ONCE per surface at DS prepare time. Never re-derived.
struct SurfacePeriodicity {
    bool   closed[2]   = {false, false};  // GeomLib::IsClosed analogue, model-space tol
    double period[2]   = {0.0, 0.0};      // valid iff closed[d]; == natural domain span
    double dom_lo[2]   = {0.0, 0.0};      // NATURAL domain of the surface (not the face window)
    double dom_hi[2]   = {0.0, 0.0};
    // Degenerate boundary isos: pole_at[d][0] = the d-min side collapses, [1] = d-max side.
    bool   pole_at[2][2] = {{false,false},{false,false}};
    double pole_radius[2][2] = {{0,0},{0,0}};   // measured collapse radius, MODEL space
    bool   any_pole() const {
        return pole_at[0][0]||pole_at[0][1]||pole_at[1][0]||pole_at[1][1];
    }
};

// Metric conversion at a point -- the ONLY way parametric tolerances are produced.
struct SurfaceMetric {
    double u_res;   // |du| that moves 1 model-space unit  == 1/|dS/du|
    double v_res;
};
SurfaceMetric surface_metric(const NurbsSurface& S, double u, double v);
inline double u_resolution(const SurfaceMetric& m, double tol3d) { return tol3d * m.u_res; }
inline double v_resolution(const SurfaceMetric& m, double tol3d) { return tol3d * m.v_res; }

SurfacePeriodicity analyse_periodicity(const NurbsSurface& S, double tol3d);
```

```cpp
// ---- additions to src/brep.h ----------------------------------------------------------

struct BRepEdge {
    int  curve_3d_index   = -1;   // -1 IS LEGAL AND MEANINGFUL when degenerate == true
    int  start_vertex     = -1;
    int  end_vertex       = -1;
    std::vector<int> trim_indices;
    bool degenerate       = false;   // NEW: S5. A pole. No 3D curve. start_vertex == end_vertex.
    double tol            = 0.0;     // NEW: model-space edge tolerance
};

// A trim is one CO-EDGE. A seam edge owns exactly two of them on one face.
struct BRepTrim {
    int  curve_2d_index = -1;
    int  edge_index     = -1;   // NEVER -1 for a Singular trim after this port (see D3)
    int  loop_index     = -1;
    bool reversed       = false;
    BRepTrimType type   = BRepTrimType::Boundary;
    int  seam_mate      = -1;   // NEW: index of the other trim of the same seam edge on the
                                // same face; -1 if this trim is not a seam co-edge.
};
```

```cpp
// ---- src/brep_seam.h  (new) -----------------------------------------------------------

// S2/S3 as a computable predicate. Returns the signed period vector delta = Pf - Pr,
// or std::nullopt when the pair does not satisfy the invariant.
struct SeamPair {
    int      trim_forward;      // the FORWARD co-edge
    int      trim_reversed;
    ParamDir dir;               // U-seam or V-seam
    double   delta;             // signed: +period or -period; Pf = Pr + delta*axis(dir)
};
std::optional<SeamPair> seam_pair_of(const BRep& b, int face, int edge);

// Verify S2 and S3 on an existing pair. n_samples default 33.
bool seam_pair_is_valid(const BRep& b, const SeamPair& sp, int n_samples = 33,
                        double rel_tol = 1e-12);

// THE porting target: BOPTools_AlgoTools3D::DoSplitSEAMOnFace, variants A then B.
// `split_trim` currently carries ONE pcurve. On success the split edge owns two trims on
// `face` with correct order and orientation, and both trim.type == Seam, both seam_mate set.
enum class SeamMakeResult { MadeGeometric, MadeByProjection, NotOnSeam, Failed };
SeamMakeResult make_split_a_seam(BRep& b, int face, int split_trim,
                                 int original_seam_edge /* -1 if unknown */,
                                 double tol3d);

// S9: translate a pcurve by whole periods onto the FACE's UV window.
// Port of BOPTools_AlgoTools2D::AdjustPCurveOnSurf, including the cylinder rescue and the
// wider-than-a-period classifier re-check.
NurbsCurve adjust_pcurve_to_face(const BRep& b, int face, const NurbsCurve& pc,
                                 double t_first, double t_last, double tol3d);

// S8: cut a UV polyline / pcurve at every seam crossing. Each output piece lies inside the
// face window; consecutive pieces share a 3D point and are one period apart in UV.
struct SeamCut { NurbsCurve pc; double t_lo, t_hi; ParamDir crossed; };
std::vector<SeamCut> split_pcurve_at_seams(const SurfacePeriodicity& per,
                                           const NurbsCurve& pc_unwrapped);

// N1: canonical window. Post: lo in [dom_lo, dom_lo+T), hi in (lo, lo+T].
void adjust_periodic_interval(double dom_lo, double dom_hi, double preci,
                              double& lo, double& hi);
// N2: relative unwrap against a reference. Post: |ref - par| <= T/2.
double unwrap_near(double par, double ref, double period);
```

```cpp
// ---- src/brep_pole.h  (new) -----------------------------------------------------------

enum class SpecialPointKind { None, Pole, SeamU, SeamV, SeamUV, PoleSeamU };  // IntPatch taxonomy

// IsDegenerated: type-first, sampling as the documented fallback (closes OCCT gap F2).
struct DegeneracyVerdict { bool degenerate; double radius3d; };
DegeneracyVerdict iso_is_degenerate(const NurbsCurve& iso3d, double max_tol);

// Materialise a pole as a real degenerate edge + one trim (S5).
// Returns the new edge index. pcurve MUST be the full-extent iso line of the collapsed side.
int add_degenerate_edge(BRep& b, int loop, const NurbsCurve& pcurve, int pole_vertex,
                        double pole_radius3d);

// Choose the free parameter of a curve arriving at a pole from its approach direction.
// Port of AddSingularPole/ProcessSphere/ProcessCone; `approach_dir3d` is the curve tangent
// just before the pole, `other_du/dv` the other surface's derivatives at the pole.
bool choose_pole_parameter(const SurfacePeriodicity& per, ParamDir collapsed,
                           const Vector& approach_dir3d,
                           const Vector& other_du, const Vector& other_dv,
                           double& free_param /*in: guess, out: chosen*/);
```

**Stage-0 obligations** (`BdsShape` preparation, `ARCHITECTURE_v2 §2` stage 0):
- compute `SurfacePeriodicity` for every surface once;
- set `BRepEdge::degenerate` for every edge whose 3D extent is below `tol3d` **and** which is a
  boundary iso of a periodic/degenerate surface — never from a domain-relative threshold;
- set `BRepTrim::seam_mate` for every seam pair and assert S1/S2/S3;
- refuse to proceed (typed error, not a warning) if a face's surface is closed in `d`, its UV
  window spans the full period in `d`, and no seam pair exists — that face is torn.

---

## 4. WHAT OUR CODE DOES TODAY, AND WHERE IT DIVERGES

All paths under `/home/petras/code/code_rust/session/session_cpp/src/`.

### 4.1 The direct answer to the create_sphere / create_cylinder question

**The premise in the brief is wrong for the seam and right for the pole.** Read the
constructors:

- `brep.cpp:338-424` `create_cylinder` — builds `ei_seam` once (`:360`) and references it from
  **two** trims of the body loop: `:387` `add_trim(c2d_sr, ei_seam, li_body, /*reversed=*/false,
  BRepTrimType::Seam)` with pcurve `(u_max,v_min) → (u_max,v_max)`, and `:397`
  `add_trim(c2d_sl, ei_seam, li_body, /*reversed=*/true, BRepTrimType::Seam)` with pcurve
  `(u_min,v_max) → (u_min,v_min)`.
- `brep.cpp:426-486` `create_sphere` — same pattern at `:469` and `:479`.
- `brep.cpp:488-554` `create_cone` — `:527` and `:533`.
- `brep.cpp:556-609` `create_torus` — four seam trims, `:593` (v-seam FORWARD at `v_min`),
  `:596` (u-seam FORWARD at `u_max`), `:599` (v-seam REVERSED at `v_max`), `:602` (u-seam
  REVERSED at `u_min`).

Reversing the stored pcurve of the REVERSED occurrence (our trims store the pcurve already in
loop-traversal order; OCCT stores both in edge parametrisation) gives, for the cylinder,
`reverse(P_r): (u_min,v_min) → (u_min,v_max)` and `P_f: (u_max,v_min) → (u_max,v_max)`, i.e.
`P_f = reverse(P_r) + (period, 0)`. **That is exactly S2 and S3, and exactly what
`BRepLib_MakeFace.cxx:689` and `BRepPrim_OneAxis.cxx:435-438` emit.** Our torus likewise matches
`BRepPrim_OneAxis.cxx:392-395` for the v-seam (PC1 at `VMin`, FORWARD, running `+U`).

The canonical OCCT counts from §2.3 for a full cylinder are F=3, E=3, V=2, co-edges=6. Our
`create_cylinder` produces exactly F=3, E=3 (`ci_bot`, `ci_top`, `ci_seam`), V=2, trims=6
(`:382, 387, 392, 397, 413, 417`). **The in-memory cylinder is topologically complete and
canonical.**

The observed `T 6→10, E 3→5, V 2→4` after a STEP round-trip is **produced by our own exporter**,
not by the round-trip discovering a missing seam. `file_step.cpp:3194-3199`:

> "Closed edges (built start == end) are SPLIT into TWO arcs sharing a new halfway vertex: Rhino
> refuses to trim a face whose loop contains ONE closed EDGE_CURVE with identical start/end
> VERTEX_POINT …"

Each of the two cap circles becomes two arcs → +2 edges, +2 vertices, +4 trims (2 on the body,
2 on the caps): `3→5`, `2→4`, `6→10`. Arithmetic matches exactly. The seam edge is not split
(its ends are already distinct vertices) and survives as one edge with two `Seam` trims — the
importer picks the second SEAM_CURVE pcurve for the REVERSED use at `file_step.cpp:1020`
(`pick = (is_seam && mine.size() > 1 && !forward_use) ? 1 : 0`), which is our port of
`BRep_Tool.cxx:354-357`.

**Verdict:** the round-tripped form is *legal but non-canonical* OCCT topology (nothing forbids
an extra vertex on a closed edge; `BRepLib_MakeFace` simply never creates one). The canonical
form is ours. Neither is "missing a seam".

**However — the sphere and the cone ARE topologically incomplete**, for the pole, not the seam:

- `brep.cpp:464` and `:474` (sphere south/north), `brep.cpp:530` (cone apex) create the polar
  side of the UV rectangle as `add_trim(..., /*edge_index=*/-1, ..., BRepTrimType::Singular)` —
  **a trim with no edge**.
- The STEP importer reproduces the same shape: `file_step.cpp:1379-1383` synthesises a
  `Singular` trim with `edge_index = -1` whenever consecutive loop pcurves have a UV gap whose
  two 3D images coincide.

Under OCCT's rules that is invalid on five counts:
1. `BRepLib_MakeFace.cxx:679-707` builds a **real edge** for the degenerate side (`MakeEdge()`
   with no curve), flags it `Degenerated` (`:695`), gives it the pole vertex twice
   (`:696-705`) and a range (`:706`).
2. `BRepPrim_OneAxis.cxx:1210-1218` does the same for the sphere pole ring.
3. Because our pole carries no edge, the pole **vertex has no incidence** through the pole:
   `add_trim` only records a trim on an edge when `edge_idx >= 0` (`brep.cpp:2867-2868`), so
   nothing in the topology says "these two seam-ends are the same point".
4. Every OCCT algorithm that special-cases poles keys off `BRep_Tool::Degenerated(E)` (the 11
   sites in §2.6). With no edge there is nothing to key off, so a pole is invisible to
   classification, to the wire splitter's `bIsClosed` rule
   (`BOPAlgo_WireSplitter_1.cxx:146`), to `BOPAlgo_BuilderFace`'s valence-1 pruning exemption
   (`:198-203`) and to the closed-shell rule (`BOPAlgo_BOP.cxx:1479-1484`).
5. `ProcessDE` (`BOPAlgo_PaveFiller_8.cxx`) — the stage that splits a pole ring when section
   curves land on it — has no operand at all.

So: **create_cylinder and create_torus are correct; create_sphere and create_cone are
topologically incomplete faces**, missing the degenerate pole edges. That is the ingest-level
defect behind "a sphere does not even ingest as a solid" in the pole-bearing cases, and it is
independent of the seam.

### 4.2 Divergences, ranked by blast radius

**D1 — the splitter never emits a seam.** Every `add_trim` call in the boolean path passes
`BRepTrimType::Boundary` or `Mated`: `brep.cpp:3986`, `:4044`, `:4202`, `:9011`, `:9017`,
`:10540`, `:11151`. `BRepTrimType::Seam` appears **only** inside the six primitive constructors
(`brep.cpp:387, 397, 469, 479, 527, 533, 593, 596, 599, 602, 729, 739`) and in the STEP importer
(`file_step.cpp:1365`). Consequence: after any split of a periodic face, S1/S2/S3/S4 are false by
construction, and there is no analogue of `DoSplitSEAMOnFace` anywhere in the tree. This is the
single structural cause of "two spheres, any rotation: cut = 2 faces / 2 naked".

**D2 — the splitter never emits a pole.** `BRepTrimType::Singular` likewise appears only in the
constructors and the importer. Any split of a sphere face loses both poles.

**D3 — poles are trims without edges.** `brep.cpp:464, 474, 530`; `file_step.cpp:1381`
(`add_trim(..., -1, loop_idx, false, BRepTrimType::Singular)`). See §4.1.

**D4 — there is no `degenerate` flag; degeneracy is re-derived by sampling, with a
model-diagonal-relative tolerance.** `brep.cpp:1071-1073` in `is_solid()`:
`deg_tol = max(diag*1e-7, 1e-12)` where `diag` is the **whole model's** bounding-box diagonal;
`brep.cpp:1091-1101` then samples the 3D curve at 5 parameters and treats extent `< deg_tol` as
degenerate. The identical computation is repeated in `topology_report` (`:1129-1145, :1151`),
in the micro-edge guard (`:7160-7190`) and in the orphan guard (`:7355-7380`). Three
consequences: (a) a genuinely tiny but non-degenerate edge on a large model is silently
reclassified as a pole; (b) the same edge changes class when the model grows; (c) a pole on a
small feature inside a large assembly is *not* recognised. OCCT stores the flag once at
construction and never re-derives it.

**D5 — the closed-shell rule is length-based, not topology-based.** `brep.cpp:1075-1117`
`is_solid()` accepts an edge if it has exactly 2 trims, or 0 trims (`:1082-1086`), or is
3D-short (`:1091-1102`). It has no clause for "one face, two pcurves" (S7 clause (c)) and no
clause for INTERNAL. A seam edge happens to pass because it has 2 trims, but only by accident
of the constructors; a *split* seam that mated only on one side is reported as naked with no
indication that the seam rule was the missing one.

**D6 — the UV arrangement does not know the surface is periodic.** `srf.is_closed(...)` is
consulted in exactly **one** place in the entire splitter: `brep.cpp:4566`
(`bool cu2 = srf.is_closed(0), cv2 = srf.is_closed(1);`), and only to *skip* two operations —
the clip of cut pcurves to the chart rectangle (`:4562-4600`) and the drop of pcurves running
along a chart border (`:4638-4660`). The arrangement itself
(`nurbssurface_trimmed.cpp:1589-2100`, `split_face_by_wires`) never receives the periodicity: it
compensates by keying graph vertices on **3D position** (`:1788-1811`, `vert_id` buckets
`floor(P/snap3)` and merges within `snap3`), which collapses the two seam sides and the whole
pole row into single vertices, then tries to re-separate them with a UV heuristic
`seam_tol = max(snap_uv*8, min(range_u,range_v)*1e-6)` (`:1650`) and a `seam_v[]` guard
(`:1930-1968`). This is exactly the "edge identity from coordinate coincidence" defect named in
the mission, applied to the one place where the identity is *known analytically* (two UV points
one period apart are the same point, always, exactly).

**D7 — every parametric tolerance in the arrangement is domain-relative.**
`nurbssurface_trimmed.cpp:574` `samp_tol = max(range_u, range_v) * 2e-5`; `:1653`
`samp_tol = max(range_u, range_v) * 1e-3`; `:1650` `seam_tol` as above; `brep.cpp:4350`
`eps_border = min(du, dv) * 2e-3`; `brep.cpp:4280` `scaf_forced_eps` mixes
`min_rangeF * 1e-2` with a model-space term. A periodic surface's domain span **is** its period,
so these tolerances scale with the period — a sphere parametrised on `[0,4]` (our convention,
`primitives.cpp:307-336`) and one on `[0,2π]` get tolerances differing by 57%, for identical
geometry. OCCT's equivalents are always `UResolution(tol3d)` / `VResolution(tol3d)`.

**D8 — seam decomposition exists but only on the scaffold path, and destroys closure.**
`brep_section.cpp:803-905` is a genuine port of `DecompositionOfWLine`'s cutting step: it detects
`|Δ| > period*0.5` (`:823`), computes the exact crossing fraction (`:864-869`), pins the crossing
parameter to the bound and re-converges the other three with `correct7_pinned` (`:872-878`), and
emits the mirrored start of the next piece at the opposite bound (`:886-894`). Three gaps:
(a) it runs only inside the scaffold section builder, not on the general path; (b) a **closed**
chain is rotated and turned into an open one (`:838` `ch.closed = false`) — the wrap identity is
discarded rather than recorded; (c) it uses `dom[d].second - dom[d].first` as the period
(`:822`), which is only correct when the face window is exactly one period.

**D9 — the pull-back is seam-correct but its result is thrown into a seam-blind consumer.**
`closest.cpp:356-770` `Closest::surface_curve` does this properly: `wrap_u/wrap_v` (`:409-424`),
a windowed inversion that tries `±period` branch centres (`:426-452`), `unwrap_to` implementing
N2 exactly (`:454-465`), and a piece splitter that cuts at every seam line `u0 + k*range_u`
(`:541-620`) including the `closure_crosses_seam` distinction (`:552-557`). This is the one part
of our periodic handling that matches OCCT's intent. Its output pieces then enter
`split_face_by_wires`, which re-merges their endpoints by 3D coincidence (D6) — the information
is computed and immediately discarded.

**D10 — edge identity after splitting is coordinate-keyed.** `brep.cpp:4188-4197`:
`ekey = (min(va,vb), max(va,vb), q6(pm.x), q6(pm.y), q6(pm.z))` with
`q6(x) = llround(x*1e6)` (`brep.cpp:2897`) — a 1 µm absolute grid; and `brep.cpp:4168-4187`, the
`bemap` path, matches by 3D midpoint within
`1.5*(devtol + pr.dev) + bemap_tol*0.1` where `bemap_tol = max(1e-9, model_diag*5e-4)`
(`brep.cpp:3838-3846`). For a seam this "works" only because both occurrences lift to the
identical 3D curve; nothing in the data structure states that they must.

**D11 — `Singular` trims are silently dropped when a face is converted to a trimmed surface.**
`brep.cpp:11477` and `:11674`: `if (trim.type == BRepTrimType::Singular) continue;` while
building `NurbsSurfaceTrimmed::m_outer_segments`. The pole side of the UV rectangle is therefore
absent from the loop that the classifier and the arrangement see; the loop is open by exactly
the pole span. OCCT keeps the degenerate edge's pcurve in the wire and excludes it only from the
*classification polygon* (`IntTools_FClass2d.cxx:167-170`) — a different and much narrower
exclusion.

**D12 — no `AdjustPCurveOnSurf` equivalent.** No routine in our tree translates a finished
pcurve by whole periods onto the face's UV window; the closest thing is the per-loop chain shift
in the STEP importer, and that one is explicitly disabled for non-projected loops precisely
because it would collapse a seam (`file_step.cpp:1344-1352`).

### 4.3 What is already right and must be kept

- The four primitive constructors' seam representation (§4.1) — it is OCCT-canonical; the port
  must generalise it, not replace it.
- `Closest::surface_curve`'s unwrap and seam splitting (`closest.cpp:409-620`).
- `brep_section.cpp:803-905`'s crossing computation (pin one parameter to the bound, re-converge
  the rest) — this is better than OCCT's `FindPoint` because it re-solves the intersection
  system instead of interpolating.
- The STEP importer's SEAM_CURVE pcurve-order rule (`file_step.cpp:1020`) — it is exactly
  `BRep_Tool.cxx:354-357`.
- `NurbsSurface::is_closed(dir)` (`nurbssurface.cpp:507-528`) — CV-net based, correct for our
  clamped-closed primitives; extend it with the sampling test of `GeomLib::IsClosed` for
  imported freeform surfaces.

---

## 5. ACCEPTANCE TESTS

All are oracle-free: the expected value is either analytic or an invariant of the output alone.
`T` denotes the period in the relevant direction.

### A. Representation tests (no boolean involved)

| id | operand | assertion |
|---|---|---|
| **A1** | `create_cylinder(1, 2)` | F=3, E=3, V=2, trims=6. Seam edge occurs twice in the body loop with opposite `reversed`. |
| **A2** | `create_cylinder(1, 2)` | S2: sample 33 params; `max_t |Pf(t) − Pr(t) − (T,0)| < 1e-12·T` with `T = u_max − u_min`. |
| **A3** | `create_cylinder(1, 2)` | S3: `cross2d(Pf − Pr, Pf') > 0` at 33 params. |
| **A4** | `create_torus(3, 1)` | Both seams satisfy A2/A3, one in U (`delta = (+T_u,0)`) and one in V (`delta = (0,−T_v)`). |
| **A5** | `create_sphere(1)` | **currently fails.** Each pole is a real edge with `degenerate == true`, `curve_3d_index == -1`, `start_vertex == end_vertex`, exactly one trim, pcurve spanning the full u range. |
| **A6** | `create_cone(1, 2)` | as A5 for the apex; the base circle is non-degenerate. |
| **A7** | any of the above | S7: `is_solid()` returns true using **only** clauses (a) ≥2 faces, (b) `degenerate`, (c) seam-pair present, (d) INTERNAL — with the 3D-length clause deleted. |
| **A8** | `create_sphere(1)` | Euler: V−E+F = 2−3+1 = 0 counting only non-degenerate edges and the seam once; V−E+F = 2−3+1 with poles counted = 0. Assert the chosen convention is stable across a scale of 1e−3 and 1e+3 (D4 regression). |
| **A9** | `create_sphere(1)` scaled ×1e6 and ×1e−6 | identical topology counts and identical `degenerate` flags. This is the direct test for D4. |
| **A10** | `create_sphere(1)` on domain `[0,4]` vs an equivalent sphere on `[0,2π]` | identical trim/edge/vertex counts and identical seam `delta/T` ratio. Direct test for D7. |
| **A11** | any periodic face | S12: `uv_bounds(F)` equals the surface's natural bounds to 1e-12. |

### B. Seam-crossing tests (splitter, no boolean semantics)

| id | operands | assertion |
|---|---|---|
| **B1** | cylinder R=1 H=2 axis Z; cutting plane `z = 1` | the section is one circle. Its pullback wraps U once. Output: **2** pcurve pieces on the body face if the cut starts off-seam is not required — required is S8: no stored pcurve has a `|Δu| > T/2`, and the two pieces' shared points are 3D-identical to 1e-12. |
| **B2** | B1, then rotate the whole assembly by a random axis/angle (100 poses) | every pose yields the same piece count and the same total 3D arc length `2π` to 1e-9. Rotation must not change the seam behaviour, because the seam is a property of the parametrisation, not of world space. |
| **B3** | cylinder cut by plane `x = 0` (a plane **through the axis**, section runs ALONG two generatrices, one of which is the seam) | the section coincident with the seam produces **no new edge**; the existing seam edge is reused (§2.9 "what OCCT does not do"). Assert `edges_after == edges_before + 1` (only the far generatrix is new). |
| **B4** | split a cylinder body face by a section that lands exactly on the seam within `u_resolution(1e-7)` | the split seam pieces each satisfy S2/S3 (`make_split_a_seam` variant A path). |
| **B5** | as B4 but with the section 1e-4 away from the seam | variant A must return `NotOnSeam`, variant B must not be invoked, and no seam is fabricated. |

### C. Pole tests

| id | operands | assertion |
|---|---|---|
| **C1** | sphere R=1, cutting plane `z = 0.5` | one section circle of radius `√3/2`; the polar cap face retains its pole edge; the cap's UV loop is closed (pcurve chain head-to-tail within `u_resolution(tol)`). |
| **C2** | sphere R=1, plane `x = 0` (**through both poles**) | the section passes through both poles. Assert: a vertex exists at each pole; the pole edge is split into exactly 2 pieces per pole (`ProcessDE` analogue), each still `degenerate`; no NaN in any stored pcurve. |
| **C3** | cone R=1 H=2, plane through the apex | the section terminates at the apex. Assert the apex vertex tolerance ≥ the measured apex collapse radius (S13), and the section's last pcurve point has `v` equal to the apex `v` to 1e-12. |
| **C4** | sphere, section curve approaching the pole from direction `θ` (sweep θ over 16 values) | the chosen pole `u` parameter equals `θ` to within `u_resolution(1e-7)` — the S11 assignment is *recorded*, and re-running the split reproduces it bit-for-bit (determinism). |
| **C5** | sphere ∩ sphere, centres offset along Z by `0.5R`, equal radii | the section is one circle at constant `v` on both spheres — it never crosses a seam and never touches a pole. This is the **easiest** curved case and must pass before anything else: cut = 2 faces per operand, naked = 0, `is_solid` true. |

### D. Boolean-level, analytically known answers

| id | operands | expected |
|---|---|---|
| **D1** | two unit spheres, centres `(0,0,0)` and `(d,0,0)` with `d = 1.0`, arbitrary rigid motion applied to both | union volume `= 2·(4π/3) − V_lens` with `V_lens = (π/12)(4R+d)(2R−d)² = (π/12)(5)(1)² = 5π/12`. Assert to 1e-9 relative, over 100 random poses. Also: `naked == 0`, `is_solid == true`, faces per operand ≥ 2. |
| **D2** | same operands, CUT | volume `= 4π/3 − 5π/24` (half the lens). Assert to 1e-9 over the same 100 poses. |
| **D3** | sphere R=1 ∪ cylinder R=0.5 H=4 coaxial-Z, then rotated | volume by divergence theorem on the output shell must equal the sum of analytic pieces to 1e-9; independently, `Σ_faces ∮ (F·n) dA` over the output must equal the same number (**partition residual** test — no oracle). |
| **D4** | any pair, any op | **partition identity**: `vol(A∪B) + vol(A∩B) = vol(A) + vol(B)` to 1e-12 relative, where all four are computed by the same integrator on the four outputs. This is the strongest oracle-free test and it exercises seam/pole correctness indirectly but completely. |
| **D5** | freeform blob (one periodic BSpline face, seam + two poles) ∩ box | output shell closed; every stored pcurve satisfies S8; the seam of the blob appears in the output as a seam pair on each surviving piece. |
| **D6** | torus ∩ box | both seams survive; `naked == 0`; genus of the output shell computed from V−E+F matches the analytic genus of the result. |

### E. Regression guards for the specific defects

| id | check |
|---|---|
| **E1** | grep-equivalent assertion: after the port, every `add_trim` in the boolean path that references an edge with a seam mate uses `BRepTrimType::Seam`; a debug post-condition counts `seam_trims % 2 == 0` per face. |
| **E2** | no `Singular` trim anywhere has `edge_index == -1` (D3). |
| **E3** | no parametric tolerance in the seam/pole subsystem is computed from `range_u`/`range_v`; enforced by a unit test that runs the whole A- and B-batteries on a face whose UV domain has been artificially padded ×4 and asserts bit-identical topology (this is the STEP-round-trip pathology from `ARCHITECTURE_v2 §3`). |
| **E4** | `is_solid()` on `create_sphere(1)` with the 3D-length clause removed still returns true — i.e. it passes on the seam/degenerate clauses, not on a length threshold (D5). |

---

## 6. IMPLEMENTATION ORDER (smallest shippable increment first)

Each step is independently revertable and independently measured. Gates are oracle-free.

**P0 — `SurfacePeriodicity` and the metric (no behaviour change).**
Add `src/brep_periodic.h/.cpp`: `analyse_periodicity` (port `GeomLib::IsClosed`
`GeomLib.cxx:2693-2850`, type-dispatched with the sampling fallback), `surface_metric`,
`u_resolution`/`v_resolution`. Compute once per surface at prepare time; store it. Nothing reads
it yet.
*Gate*: unit test — for all six primitive surfaces the detected `closed[]`, `period[]` and
`pole_at[][]` match the analytic truth; `u_resolution(1e-7)` on a unit cylinder equals `1e-7`
(radius 1 ⇒ arc-length = angle) to 1e-15.

**P1 — `degenerate` flag on `BRepEdge`; poles become real edges.**
Add the flag; set it in the four constructors and in the STEP importer; replace the
`edge_index = -1` `Singular` trims (`brep.cpp:464, 474, 530`; `file_step.cpp:1381`) with real
degenerate edges via `add_degenerate_edge`. Delete the 3D-length re-derivation in `is_solid`
(`brep.cpp:1091-1101`), `topology_report` (`:1151`), the micro-edge guard (`:7160-7190`) and the
orphan guard (`:7355-7380`), replacing every one with `edge.degenerate`.
*Gate*: A5, A6, A9, E2, E4. Full corpus non-decreasing.

**P2 — seam pairs become first-class.**
Add `BRepTrim::seam_mate`; implement `seam_pair_of` and `seam_pair_is_valid`; populate at
prepare time; assert S1–S4 as a debug post-condition on every input BRep and on every output
BRep. Rewrite `is_solid()`/`topology_report()` to the four-clause rule of S7
(`BOPAlgo_BOP.cxx:1474-1505`).
*Gate*: A1–A4, A7, A8, A11. Every corpus operand passes the input assertion (this alone will
name every malformed operand we have).

**P3 — `adjust_pcurve_to_face` (S9).**
Port `BOPTools_AlgoTools2D::AdjustPCurveOnSurf` (`:247-400`) in full, including the cylinder
rescue (`:294-313`) and the wider-than-a-period classifier re-check (`:346-387`). Route every
pcurve produced by the splitter through it before storage. Port
`adjust_periodic_interval` (N1, `ElCLib.cxx:139-148`) and `unwrap_near` (N2,
`IntPatch_SpecialPoints.cxx:1111-1119`) as the two named primitives; forbid raw `fmod` in the
subsystem.
*Gate*: A10, E3 — the ×4-padded-domain test passes with bit-identical topology.

**P4 — `split_pcurve_at_seams` on the general path (S8).**
Lift `brep_section.cpp:803-905` out of the scaffold path into `src/brep_seam.cpp`, fix the three
gaps: keep the closed flag and record the wrap count instead of dropping it (`:838`); take the
period from `SurfacePeriodicity` rather than the domain span (`:822`); run it on **every**
section chain, not only scaffold ones. Feed `Closest::surface_curve`'s already-correct pieces
(`closest.cpp:541-620`) straight through instead of re-merging them.
*Gate*: B1, B2, C1, C5. `sphere × sphere` offset-along-axis must go from 0/20 to 20/20.

**P5 — `make_split_a_seam` (S4) and seam-aware assembly.**
Port `DoSplitSEAMOnFace` variant A (`BOPTools_AlgoTools3D.cxx:58-232`) then variant B
(`:236-327`), and the caller ladder of `BOPAlgo_Builder_2.cxx:429-455`: detect closedness with
`GeomLib::IsClosed` + `IsClosed(E,F)` + `IsEdgeIsoline`; on a split of a closed edge, make it a
seam, then append it **twice** (FORWARD and REVERSED) into the face's edge list; append a
degenerate split **once** keeping its orientation (`:413-418`). Emit
`BRepTrimType::Seam`/`Singular` from the splitter.
*Gate*: B3, B4, B5, E1. `sphere × sphere` arbitrary rotation ≥ 18/20 with `naked == 0`.

**P6 — periodicity into the UV arrangement (kills D6).**
Give `split_face_by_wires` the `SurfacePeriodicity`. Replace 3D-position vertex bucketing
(`nurbssurface_trimmed.cpp:1788-1811`) with **exact** parametric identification: two UV points
identify iff they differ by an integral multiple of a period in a closed direction, or lie on the
same collapsed iso. Replace `seam_tol` (`:1650`) and the `seam_v[]` heuristic (`:1930-1968`) with
that exact rule. Stop dropping `Singular` trims when building trimmed-surface loops
(`brep.cpp:11477, 11674`); keep them in the loop and exclude them only from the classification
polygon, as `IntTools_FClass2d.cxx:167-170` does.
*Gate*: C2, D6, and the freeform blob D5. The one-operand padded-domain test
(32 naked of 36) must reach 0 naked.

**P7 — the pole stage `ProcessDE` (S11).**
Port `BOPAlgo_PaveFiller_8.cxx` in full: `FindPaveBlocks` (`:135-159`), `FillPaves` (`:224-331`)
with `aTolInt`/`aTolCmp` derived from the pole vertex tolerance through `UResolution`/
`VResolution` (`:243-267`), `AddSplitPoint` (`:368-393`), `MakeSplitEdge`/`MakeSplitEdge1`
(`:163-214, 335-358`) with the range set **on the face**. Add `choose_pole_parameter`
(port of `AddSingularPole` `:888-922` and `ContinueAfterSpecialPoint` `:990-1078`, **including**
the `π/2` pseudo-period at `:1063` and its comment verbatim as a documented hack).
*Gate*: C2, C3, C4. Sphere-through-pole cases produce closed shells; C4 determinism holds.

**P8 — full curved battery.**
*Gate*: D1–D4 over 100 random poses each; the 224-cell primitive matrix with box×sphere,
sphere×sphere, box×cone, cone×cone, cyl×cone, sphere×cone all non-zero and `naked == 0`;
partition identity (D4) to 1e-12 on every cell that completes.

**Ordering rationale.** P0–P2 are pure representation and cost nothing at runtime, but they turn
every subsequent defect into an assertion failure at the point of creation instead of a naked
edge three stages later. P3 removes the domain-relative tolerance class from this subsystem
before any new code depends on it. P4 is the smallest change that fixes a measurable cell
(sphere × sphere coaxial). P5 is the change that makes rotated sphere cases representable at
all. P6 removes the last coordinate-coincidence identity in the subsystem. P7 is last because a
pole only matters once the seam is correct — a section that reaches a pole has already crossed a
seam.
