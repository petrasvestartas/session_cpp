# OCCT BRepCheck: the validity contract (spec extracted from sources)

Sources read: `src/ModelingAlgorithms/TKTopAlgo/BRepCheck/` — `BRepCheck_Analyzer.{hxx,cxx}`, `BRepCheck.cxx`,
`BRepCheck_Result.cxx`, `BRepCheck_Vertex.cxx`, `BRepCheck_Edge.cxx`, `BRepCheck_Wire.cxx` (2177 lines),
`BRepCheck_Face.cxx`, `BRepCheck_Shell.cxx`, `BRepCheck_Solid.cxx`, `BRepCheck_Status.hxx`, plus
`TKTopAlgo/BRepLib/BRepLib_ValidateEdge.cxx` (the edge tolerance-containment engine).

Related existing specs (referenced, not duplicated): `kb/occt_tolerance-model.md` (Precision constants, tolerance
inflation laws), `kb/occt_builder-assembly.md` (how BOPAlgo assembles shapes that must pass these checks),
`kb/occt_interference-vef.md` (V/E/F interference — the *construction-time* analog of these *audit-time* checks).

This is the audit OCCT runs when we call `BRepCheck_Analyzer(shape).IsValid()` — our acceptance oracle
(`validation/` harness, "OCCT-VALID" verdicts). Everything below is what VALID actually means.

---

## STAGE PIPELINE

Ordered, exact file + function names. Two phases: **Minimum** (standalone, run at construction of each
`BRepCheck_Result`) and **InContext** (subshape checked against a parent, run by the analyzer), then
**IsValid** aggregation.

### 0. Setup — `BRepCheck_Analyzer::Init` / `Put` / `Perform` (BRepCheck_Analyzer.cxx)

- `Init(S, GeomControls=true)` → `Put(S, B)` recursively walks `TopoDS_Iterator`, creating exactly ONE
  `BRepCheck_Result` per unique subshape into `myMap : IndexedDataMap<TopoDS_Shape, Handle(BRepCheck_Result)>`.
  Each Result constructor calls `Init(shape)` → `Minimum()` immediately. `GeomControls(B)` set on Edge/Wire/Face;
  `SetExactMethod(myIsExact)` on Edge (default false; exact = `GeomLib_CheckCurveOnSurface` global extrema).
- `Perform()` chunks the map into tasks (min 10 shapes/task, `nbThreads*10` tasks) and runs
  `BRepCheck_ParallelAnalyzer::operator()` per shape type:
  - **VERTEX**: nothing (Minimum was everything).
  - **EDGE**: `BRepCheck_Edge::CheckPolygonOnTriangulation(edge)`; then for each unique vertex of the edge:
    `BRepCheck_Vertex::InContext(edge)`.
  - **WIRE**: nothing here (wire-in-face is driven from the FACE case).
  - **FACE**: for each unique vertex: `Vertex::InContext(face)`; for each unique edge: `Edge::InContext(face)` —
    while doing so, if any edge yields `NoCurveOnSurface | InvalidCurveOnSurface | InvalidRange |
    InvalidCurveOnClosedSurface`, set `performwire=false`; for each wire: `Wire::InContext(face)` — if any wire
    status != NoError, `orientofwires=false`. Finally: if `performwire && orientofwires` →
    `BRepCheck_Face::OrientationOfWires(true)` (which chains `ClassifyWires` → `IntersectWires`), else
    `BRepCheck_Face::SetUnorientable()` (status `BRepCheck_UnorientableShape`).
  - **SHELL**: nothing here (shell-in-solid driven from SOLID case).
  - **SOLID**: for each shell: `BRepCheck_Shell::InContext(solid)`.
  - Any `Standard_Failure` during a check → `SetFailStatus` = `BRepCheck_CheckFail` on the shape (still invalid).

### 1. `BRepCheck_Vertex::Minimum` (BRepCheck_Vertex.cxx)
Appends `NoError` unconditionally. A lone vertex is always valid.

### 2. `BRepCheck_Edge::Minimum` (BRepCheck_Edge.cxx)
Representation existence & parameter-range sanity on `BRep_TEdge->Curves()`:
- `!SameRange && SameParameter` → `InvalidSameParameterFlag` (flag coherence: SameParameter implies SameRange).
- No `IsCurve3D()` rep → `No3DCurve`; more than one → `Multiple3DCurve`.
- Reference curve `myCref` = first 3D curve, else (if `Degenerated`) first curve-on-surface. 3D curve present on a
  Degenerated edge → `InvalidDegeneratedFlag`.
- Range `[First,Last]` of `myCref`: `Last <= First` → `InvalidRange`. Against basis-curve domain `[f,l]`
  (unwrapping `Geom_TrimmedCurve`): periodic → require `Last-First <= period + eps`; non-periodic → require
  `First >= f-eps && Last <= l+eps`; violation → `InvalidRange`. `eps = Precision::PConfusion()` (=1e-9).
  Same test applied to the pcurve rep if `myCref` is a curve-on-surface.
- Builds `myHCurve` (GeomAdaptor_Curve or Adaptor3d_CurveOnSurface) as the reference for all later containment.

### 3. `BRepCheck_Wire::Minimum` (BRepCheck_Wire.cxx)
- Zero edges → `EmptyWire`.
- >= 2 edges: vertex-edge incidence map `myMapVE`, BFS `Propagate(mapVE, firstEdge, mapE)`; any edge not reached
  → `NotConnected`. (Pure connexity — orientation ignored here.)

### 4. `BRepCheck_Face::Minimum` (BRepCheck_Face.cxx)
`TF->Surface().IsNull()` → `NoSurface`. Nothing else standalone.

### 5. `BRepCheck_Shell::Minimum` (BRepCheck_Shell.cxx)
- Zero faces → `EmptyShell`.
- >= 2 faces: edge→faces map `myMapEF`, width-first `Propagate(mapEF, firstFace, mapF)`; `mapF.Extent() != nbface`
  → `NotConnected`.

### 6. `BRepCheck_Solid::Minimum` (BRepCheck_Solid.cxx — Kurnev 2014; solid has NO InContext)
- Same face reachable twice across the solid's shells → `InvalidImbricationOfShells`.
- Direct child that is not a SHELL must be orientation INTERNAL, else `BadOrientationOfSubshape`.
- All shells internal (no growth) → `BadOrientationOfSubshape`.
- For >= 2 closed non-internal shells, per shell build a probe solid; `BRepCheck_ToolSolid::Init` classifies the
  infinite point (`PerformInfinitePoint(::RealSmall())`; state IN → shell is a hole) and picks an inner probe
  point on the first non-degenerated edge at `t = (1-aPAR_T)*t1 + aPAR_T*t2`, `aPAR_T = 0.43213918` (10*e^-PI).
  - More than ONE non-hole shell → `BRepCheck_EnclosedRegion` ("too many growths").
  - Any shell's probe point classified OUT of another shell → `SubshapeNotInShape`.

### 7. `BRepCheck_Vertex::InContext` (BRepCheck_Vertex.cxx)
Parent EDGE: `Tol^2` compare with `Tol = max(tolV, tolE)`. For every curve representation of the edge:
- every `BRep_PointRepresentation` `IsPointOnCurve(C,L)`: `|TV->Pnt() - C(pr->Parameter())| > Tol` →
  `InvalidPointOnCurve`.
- if vertex oriented FORWARD (or multiple): `C(GC->First())` must be within Tol; REVERSED (or multiple):
  `C(GC->Last())` within Tol → else `InvalidPointOnCurve`. **This is the vertex-position <-> curve-endpoint weld
  contract: vertex 3D point must sit within max(tolV,tolE) of the 3D curve evaluated at the edge's range ends.**
- curve-on-surface reps: `S(pc(param))` within Tol (both pcurves if closed surface) → else
  `InvalidPointOnCurveOnSurface`.
Parent FACE: point-on-surface reps within `max(tolV, tolF)` → else `InvalidPointOnSurface`.

### 8. `BRepCheck_Edge::InContext` (BRepCheck_Edge.cxx)
Parent must contain the edge (`SubshapeNotInShape` else).
Parent FACE (the same-parameter / pcurve containment contract; skipped if `myCref` null):
- `!SameParameter` → `InvalidSameParameterFlag`; `!SameRange` → `InvalidSameRangeFlag`; either one aborts the rest.
- For each pcurve rep on THIS surface (`cr->IsCurveOnSurface(Su, L)`):
  1. Range agreement: pcurve `Range(f,l)` (transformed) vs reference `[First,Last]`:
     `|ff-First| > eps || |ll-Last| > eps` → `InvalidSameRangeFlag` AND `InvalidSameParameterFlag`
     (`eps = Precision::PConfusion()`).
  2. Domain containment (same periodic/non-periodic rule as Minimum) → `InvalidRange` (returns immediately).
  3. Geometric containment (only if `myGctrl`): `BRepLib_ValidateEdge(myHCurve, ACS, SameParameter)` with
     `SetExitIfToleranceExceeded(Tol)` where `Tol = BRep_Tool::Tolerance(edge)`:
     `!IsDone() || !CheckTolerance(Tol)` → `InvalidCurveOnClosedSurface` if seam rep else
     `InvalidCurveOnSurface`, plus always `InvalidSameParameterFlag`.
  4. Seam rep (`IsCurveOnClosedSurface`): SECOND pcurve `PCurve2` gets its own ValidateEdge run at same bounds →
     failure = `InvalidCurveOnClosedSurface` (+`InvalidSameParameterFlag` if SameParameter).
- No pcurve found on this surface: legal ONLY if the surface is a plane (possibly under
  `Geom_RectangularTrimmedSurface`) — then the 3D curve is projected on the fly
  (`GeomProjLib::ProjectOnPlane` + `ProjLib_ProjectedCurve`) and ValidateEdge-checked (fail →
  `InvalidCurveOnSurface`). Non-plane without pcurve → `NoCurveOnSurface`.
Parent SOLID (edge manifoldness):
- `nbconnection` = number of (face, edge-use) incidences over all faces of the solid (each occurrence in each
  face wire counts — a seam used twice by one face counts 2).
- `nbconnection < 2 && !Degenerated` → `FreeEdge`; `> 2` → `InvalidMultiConnexity`. **This is OCCT's "naked
  edge" and "non-manifold edge" gate at solid level.**

#### 8b. `BRepLib_ValidateEdge` (BRepLib_ValidateEdge.cxx) — the distance engine
- `myControlPointsNumber = 22` → 23 sample points `i=0..22`, `t_i = ((22-i)*First + i*Last)/22`.
- Non-projection fast path (SameParameter and ranges equal within PConfusion): direct
  `|C3dref(t_i) - S(pc(t_i))|` — SAME parameter, no reprojection. Max distance kept.
- Else (not SameParameter or ranges differ): endpoints compared directly, interior points via
  `Extrema_LocateExtPC` both ways (point of one curve projected on the other); Extrema failure → `!IsDone()`.
- `CheckTolerance(tol)`: `correctTolerance(tol) > myCalculatedDistance`, where `correctTolerance` adds
  `max(BRepCheck::PrecCurve(ref), BRepCheck::PrecSurface(surf))` — `RealEpsilon()` normally; for Ellipse/Cone
  it is the max machine `Epsilon()` of the defining data (center coords, radii) — a scale-aware epsilon.
- `UpdateTolerance`/`GetMaxDistance` report `dist * 1.00001`.
- Exact mode (`myIsExactMethod && mySameParameter`): `GeomLib_CheckCurveOnSurface` global extremum — stronger
  than 23-point sampling. Default is approx.

### 9. `BRepCheck_Wire::InContext(FACE)` (BRepCheck_Wire.cxx) — ordered, short-circuit
Order (first failure recorded, rest skipped): `SelfIntersect` (only if `myGctrl`) → `Closed` → `Orientation` →
`Closed2d`.

- **`Closed(Update)`** — combinatorial parity on ORIENTED edges only (`IsOriented` = FORWARD or REVERSED;
  INTERNAL/EXTERNAL ignored):
  - edge occurring >= 3 times, or exactly twice with the SAME orientation → `RedundantEdge`
    (twice with opposite orientations is legal — a seam).
  - connexity of oriented sub-graph via `Propagate` → `NotConnected`.
  - every oriented vertex must be met an EVEN number of times over oriented edge uses (`myMapVE(i).Extent() % 2`)
    → else `NotClosed`. (Even, not exactly 2 — a figure-8 vertex passes here; Orientation disambiguates.)

- **`Orientation(F, Update)`** — head-to-tail chain walk: from a seed edge, repeatedly find edges whose FORWARD
  vertex IsSame the current chain-end vertex VL (or REVERSED matching VF when walking backwards; direction
  flips once when the wire is open). Candidate list `ledge` filtered by `ChoixUV(pivot, theEdge, F, ledge)`:
  - `ChoixUV` evaluates the UV tangent of the current edge at the pivot (`CurveDirForParameter`; if D1 is null
    it climbs derivatives D2..D100), reverses per orientation, then keeps the single candidate with the
    minimal CCW angle if `F.Orientation()==FORWARD` (maximal if REVERSED); candidates whose UV endpoint is not
    within `IsDistanceIn2DTolerance` of the pivot point are discarded; if the angular filter kept nothing but
    exactly one candidate remains, it is accepted only if 2D-close AND 3D-close
    (`IsDistanceIn3DTolerance(pEdg, pEFound, tolV)`), and neither edge Degenerated.
  - after ChoixUV: `nbconnex >= 2` → `BadOrientationOfSubshape`; `nbconnex == 0` → `NotClosed` (or
    `BadOrientationOfSubshape` if the wire was combinatorially closed); missing FORWARD/REVERSED vertices on
    the seed → `InvalidDegeneratedFlag`.
  - extra 2D closure probe at Index==1 (wire combinatorially closed): the start vertex must have an incoming
    (REVERSED) edge chosen by ChoixUV, else `NotClosed`.

- **`Closed2d(F, Update)`** — geometric closure in UV (runs for ALL faces since OCC234, not only periodic):
  - `BRepTools_WireExplorer` over the wire on the face must traverse exactly the number of oriented edges
    (else `NotClosed` — the explorer itself gives up on 2D gaps).
  - Gap check between last edge's end UV point and first edge's start UV point (`BRep_Tool::UVPoints`):
    - `IsDistanceIn2DTolerance(surf, p, pRef, aTol3d)` with `aTol3d = max(tol(firstVertex), tol(lastVertex))`:
      PASS if `du < 0.01*Uspan && dv < 0.01*Vspan` (1%-of-domain fast accept!); else
      `aTol2d = 2*max( max(UResolution(aTol3d), aTol3d/|D1U|), max(VResolution(aTol3d), aTol3d/|D1V|) )`
      evaluated at the UV midpoint; PASS if `max(du,dv) < aTol2d`. FAIL → `NotClosed`.
    - `IsDistanceIn3DTolerance(pnt(firstV), pnt(lastV), aTol3d)`: `dist < aTol3d` (strict) → else `NotClosed`.
  - Periodic-face seam special case `IsClosed2dForPeriodicFace`: if the shared vertex lies on a seam edge
    (edge `BRep_Tool::IsClosed(edge,face)` present twice), the two UV images of the vertex across the seam are
    computed; required `distUV(p1,p2guess) <= max(0.01 * seamJump, sqrt(UResol^2+VResol^2))` — i.e. the 2D gap
    must be far smaller than the period jump. FAIL → `NotClosed`.

- **`SelfIntersect(F, retE1, retE2, Update)`** — all pcurve pairs of the wire (and each pcurve against itself)
  via `Geom2dInt_GInter` with domain tol `tolint = 1e-10`:
  - Missing pcurve on first edge → immediately `SelfIntersectingWire`; later edges → `NoCurveOnSurface` /
    `InvalidRange` returned directly.
  - 2D boxes (`BndLib_Add2dCurve`, gap `Precision::PConfusion()`) prune pairs; `E1.IsSame(E2)` skipped in the
    pair loop (self-test done separately per curve).
  - An intersection POINT is an error only if either transition is `IntRes2d_Middle` (interior crossing).
    Escape: the 3D point (from the 3D curve, else surface eval of the pcurve point) lying within vertex
    tolerance of ANY vertex of E1 (self case, `tolvtt^2` compare) / within `1.1*tolvtt` (squared) of a COMMON
    vertex of E1,E2 for BOTH 3D points (pair case) → legal.
  - Params outside either edge's 3D range (by PConfusion) are skipped ("gka protect").
  - "Maximum yawn" secondary escape (pair case with common vertex): build the 3D line from nearest common vertex
    to the intersection point; sample each edge at `u = VPara + k*0.1*(IPPara - VPara)`, `k=2..8`; every sample's
    distance to the line must be `<= 2.0 * tol(edge)`; then localok (tangential touch within tolerance wedge).
  - Intersection SEGMENTS (overlaps): error unless at least one segment endpoint has both transitions
    non-Middle, with a line-line coincidence special case (parallel within `Precision::Angular()` at sample
    `aPAR_T=0.43213918` → still error), or an endpoint 3D-close (1.1*tolvtt) to a common vertex.
  - Any failure → `SelfIntersectingWire` with the offending edge pair reported.

### 10. `BRepCheck_Face` geometric chain (BRepCheck_Face.cxx) — called as `OrientationOfWires(true)`
`OrientationOfWires` → `ClassifyWires` → `IntersectWires` (each memoized, each aborts the chain on error):
- **`IntersectWires`**: duplicate wire in face → `RedundantWire`. All edge pairs across each wire pair
  (2D boxes pruned, box built per edge once) via `Geom2dInt_GInter`, `Inter2dTol = 1e-10`:
  any intersection point/segment NOT coincident with a common vertex of the two wires → `IntersectingWires`.
  Coincidence test: 3D surface points of the intersection vs common-vertex 3D point within
  `tolv` (segments) / `tolv + 1e-8` (points, squared compare). `!Inter.IsDone()` → error too.
- **`ClassifyWires`**: each wire alone on an `EmptyCopied` FORWARD face; `BRepTopAdaptor_FClass2d(newFace,
  Precision::PConfusion()).PerformInfinitePoint() != TopAbs_OUT` → wire encloses infinity = it is a HOLE as
  oriented, so it is re-keyed reversed. Every other wire tested `IsInside` (midpoint of a non-tiny edge
  classified by FClass2d; skip edges with param span < PConfusion or 3D chord < Precision::Confusion).
  Requirement: exactly ONE wire contains all others and no other wire contains anything → else
  `InvalidImbricationOfWires` (also when no outer wire exists on a finite face).
- **`OrientationOfWires`**: the outer wire's orientation must equal its as-found-in-face orientation
  (else `BadOrientationOfSubshape`, with a thin-face escape `CheckThin`: exactly-2-edge wire whose pcurves run
  antiparallel is tolerated); every hole wire must appear in the face with orientation OPPOSITE to its
  classification key → else `BadOrientationOfSubshape`.

### 11. `BRepCheck_Shell::InContext(SOLID)` (BRepCheck_Shell.cxx): `Closed()` then (if orientable) `Orientation()`
- **`Closed(Update)`** — parity over ORIENTED faces:
  - `aMEToAvoid`: any edge occurring with INTERNAL/EXTERNAL orientation in any oriented face is excluded from
    the count entirely.
  - duplicate oriented face → `RedundantFace`.
  - face connexity via `Propagate` over `myMapEF` → `NotConnected`.
  - per counted edge, face-incidence `nboc`: `nboc == 1` and edge not Degenerated → **`NotClosed`** (THE naked
    edge criterion); `nboc == 0 || nboc >= 3` → run `NbConnectedSet` (splits the shell at multi-edges into
    connected face sets); more than 1 set → `InvalidMultiConnexity`.
  - Analyzer nuance: at SOLID level the status is recorded only if `(Closed()==NotClosed && S.Closed())` — i.e.
    a shell flagged closed but geometrically open — or any other non-NoError status.
- **`Orientation(Update)`** — 2-manifold orientation coherence:
  - duplicate face key → `RedundantFace`.
  - For each non-degenerated edge with `lface.Extent() <= 2`: take both faces WITH their shell orientation
    applied (`Fcur.Orientation(orf)`); locate the edge occurrence in each face; if the edge is "closed"
    (same face twice — seam) advance to the SECOND occurrence in that face; the two uses must have OPPOSITE
    orientations → else `BadOrientationOfSubshape`.
  - For edges with > 2 faces: count FORWARD uses vs REVERSED uses across all faces; `numF != numR` →
    `BadOrientationOfSubshape`.
  - If `BadOrientationOfSubshape` was found: reorientability flood from `Fref` — BFS over faces flipping
    `MapOfShapeOrientation` entries to fix mismatched edges; needing to flip an already-visited face →
    `UnorientableShape` (Moebius). Missing map entries anywhere → `SubshapeNotInShape`.

### 12. `BRepCheck_Edge::CheckPolygonOnTriangulation` (BRepCheck_Edge.cxx; analyzer EDGE case)
For each polygon-on-triangulation rep (only if a 3D curve also exists): with parameters — every node
`|C3d(param_i) - node_i| <= (deflection + tolE)` squared → else `InvalidPolygonOnTriangulation`; without
parameters — 23 curve samples (`NCONTROL`) must fall inside the node bounding box enlarged by that tol.

### 13. Aggregation — `BRepCheck_Analyzer::IsValid(S)` / `ValidSub` (BRepCheck_Analyzer.cxx)
- `IsValid(S)`: S's own standalone status list must have `NoError` first (the `BRepCheck::Add` law below makes
  list[0] != NoError iff any error exists); recurse over ALL children (`TopoDS_Iterator`, so free-standing and
  INTERNAL subshapes too); then context statuses:
  - EDGE: every VERTEX's status-in-context-of-this-edge must be all NoError (`ValidSub(S, TopAbs_VERTEX)`);
  - FACE: `ValidSub` over WIRE, EDGE, and VERTEX contexts;
  - SOLID: `ValidSub` over SHELL contexts;
  - SHELL standalone: only its own list + children (context in solid is checked from the solid).
- `BRepCheck::Add(lst, stat)` (BRepCheck.cxx): removes `NoError` when appending a real error; dedups statuses.
  So VALID <=> literally every status list of every subshape and every (subshape, parent) context is `[NoError]`.

## DATA STRUCTURES

- `BRepCheck_Result` (base, BRepCheck_Result.cxx): `myShape`; `myMap : DataMap<TopoDS_Shape,
  Handle(List<BRepCheck_Status>)>` — key == myShape → standalone ("Minimum") statuses; key == parent shape →
  "status on shape in context". `myMin`, `myBlind` flags; `myMutex` + `myIsParallel` for the parallel analyzer.
  `SetFailStatus(S)` appends `BRepCheck_CheckFail` (exception happened during that check).
- `BRepCheck_Analyzer::myMap : IndexedDataMap<TopoDS_Shape, Handle(BRepCheck_Result)>` — one Result per unique
  TShape+Location; results are SHARED between all occurrences of a subshape.
- `BRepCheck_Wire::myMapVE : IndexedDataMap<vertex, List<edge>>` — oriented incidence (parity check).
- `BRepCheck_Shell::myMapEF : IndexedDataMap<edge, List<face>>` — face incidence per edge (naked/manifold check).
- `BRepCheck_Face::myMapImb : DataMap<wire, List<wire>>` — wires contained inside key wire (imbrication).
- `BRepCheck_Status` (BRepCheck_Status.hxx) — full taxonomy, 37 values, in enum order:
  `NoError, InvalidPointOnCurve, InvalidPointOnCurveOnSurface, InvalidPointOnSurface, No3DCurve,
  Multiple3DCurve, Invalid3DCurve, NoCurveOnSurface, InvalidCurveOnSurface, InvalidCurveOnClosedSurface,
  InvalidSameRangeFlag, InvalidSameParameterFlag, InvalidDegeneratedFlag, FreeEdge, InvalidMultiConnexity,
  InvalidRange, EmptyWire, RedundantEdge, SelfIntersectingWire, NoSurface, InvalidWire, RedundantWire,
  IntersectingWires, InvalidImbricationOfWires, EmptyShell, RedundantFace, InvalidImbricationOfShells,
  UnorientableShape, NotClosed, NotConnected, SubshapeNotInShape, BadOrientation, BadOrientationOfSubshape,
  InvalidPolygonOnTriangulation, InvalidToleranceValue, EnclosedRegion, CheckFail`.
  (`Invalid3DCurve`, `InvalidWire`, `BadOrientation`, `InvalidToleranceValue` are declared but never emitted by
  the checkers read here — `InvalidToleranceValue` marked NYI in the Analyzer header.)

## CONSTANTS & TOLERANCES (exact values)

| Constant | Value | Where / role |
|---|---|---|
| `NCONTROL` | 23 | BRepCheck_Edge.cxx:61 — samples for `Tolerance()` estimator and polygon bbox check |
| `myControlPointsNumber` | 22 (23 points) | BRepLib_ValidateEdge ctor — same-parameter containment sampling |
| param eps | `Precision::PConfusion()` = 1e-9 | all range/flag comparisons (Edge Minimum/InContext, ValidateEdge projection trigger) |
| `tolint` | 1.e-10 | Wire::SelfIntersect — 2D intersector domain + confusion tol |
| `Inter2dTol` | 1.e-10 | Face Intersect (wire x wire) |
| vertex escape inflation | `1.1 * tolV`, squared | SelfIntersect pair case (self case uses 1.0*tolV) |
| yawn tolerance | `2.0 * tol(edge)`, 7 samples k=2..8 at 10% steps | SelfIntersect chord-line test |
| point-coincidence pad | `tolv + 1e-8` | Face Intersect point-vs-common-vertex |
| 2D closure fast accept | `du < 0.01*Uspan && dv < 0.01*Vspan` | IsDistanceIn2DTolerance first branch |
| 2D closure real tol | `aTol2d = 2*max(URes(tol3d), tol3d/|D1U|, VRes(tol3d), tol3d/|D1V|)` | IsDistanceIn2DTolerance |
| 3D closure | `dist < aTol3d = max(tolV_first, tolV_last)` strict | IsDistanceIn3DTolerance |
| seam 2D tol | `max(0.01 * seamJumpUV, sqrt(URes^2 + VRes^2))` | IsClosed2dForPeriodicFace |
| inner-point param | `aPAR_T = 0.43213918` (10*e^-PI) | Solid probe point; SelfIntersect line-coincidence sample |
| tolerance estimator margin | `sqrt(maxDist2) * 1.05` | Edge::Tolerance, Vertex::Tolerance ("5% de marge") |
| ValidateEdge report margin | `dist * 1.00001` | GetMaxDistance / UpdateTolerance |
| epsilon correction | `+ max(PrecCurve, PrecSurface)`; `RealEpsilon()` generally, `Epsilon(defining data)` for Ellipse/Cone | correctTolerance |
| FClass2d tol | `Precision::PConfusion()` | ClassifyWires wire classifier |
| infinite-point tol | `::RealSmall()` | Solid PerformInfinitePoint |
| parallel task size | >= 10 shapes | Analyzer::Perform |
| polygon tol | `deflection + tolE`, squared | CheckPolygonOnTriangulation |

## INVARIANTS

1. **VALID = universally NoError.** Every subshape's standalone list AND every (subshape, ancestor) context list
   must be exactly `[NoError]`. One error anywhere (including `CheckFail` from a thrown exception) fails the root.
2. **Edge tolerance is the containment budget.** Reference = the 3D curve. Every other representation
   (each pcurve lifted through its surface, the seam's second pcurve, the plane on-the-fly projection, the
   triangulation polygon with deflection added) must stay within `tol(edge)` (+machine epsilon correction) of it
   at 23 sampled parameters — at the SAME parameter when SameParameter is set (no reprojection).
3. **Vertex welds are metric.** Vertex 3D point within `max(tolV, tolE)` of curve at range endpoints (and of every
   stored point representation), within `max(tolV, tolF)` of surface points. This is the whole vertex contract.
4. **Closedness is combinatorial parity, then 2D/3D metric.** Wire: every oriented vertex used an even number of
   times; Shell: every counted non-degenerate edge used by exactly 2 (1 = naked = NotClosed; >= 3 splits into
   connected sets = InvalidMultiConnexity). Metric closure only re-enters through Closed2d (UV + 3D endpoint gap).
5. **Orientation coherence is the opposite-use rule.** A shared edge must occur FORWARD in one face and REVERSED
   in the other (with shell orientations applied; seams = same face twice with both orientations; >2 faces:
   #FWD == #REV). Face level: outer wire as-oriented, holes reversed. Fixable by flipping faces → only
   `BadOrientationOfSubshape`; unfixable (Moebius flood) → `UnorientableShape`.
6. **Interior crossings are the only illegal intersections.** Self/mutual pcurve intersections are legal exactly
   when they happen at vertices (within 1.1*tolV in 3D) or inside the tangential "yawn" wedge (2*tolE from the
   chord). Same law for wire-x-wire on a face (common vertex escape).
7. **Imbrication is single-rooted.** Face: exactly one outer wire containing all holes, holes contain nothing.
   Solid: exactly one growth shell, holes inside it, internal shells exempt.
8. **Degenerated edges are parity-exempt.** No 3D curve allowed (else InvalidDegeneratedFlag); exempt from
   FreeEdge, shell NotClosed, and shell orientation checks; skipped by ChoixUV acceptance.
9. **INTERNAL/EXTERNAL is invisible.** Non-oriented edges/faces are excluded from parity, closure, and
   orientation checks everywhere (`IsOriented` filters, `aMEToAvoid`).
10. **Error statuses cascade downward, never upward-mask.** A bad pcurve makes the face merely
    `UnorientableShape` while the real cause sits on the edge's context list — IsValid still fails via ValidSub.

## PITFALLS

- **One Result per unique subshape**: a defect detected on a shared edge poisons every occurrence; conversely a
  context list is only created ONCE per (shape, parent) pair — `InContext` early-returns if already bound.
- **Wire::InContext short-circuits**: SelfIntersect error hides Closed/Orientation/Closed2d results (only the
  first failing status is recorded at wire-in-face level). Do not expect complete diagnostics from one run.
- **`InvalidCurveOnSurface` always drags `InvalidSameParameterFlag`** (skv 2004 change) — matching OCCT status
  lists requires emitting both.
- **The 1%-of-UV-span fast accept in `IsDistanceIn2DTolerance` is LOOSE**: on a face whose parametric domain is
  huge (long extrusion), a sizable UV gap passes wire 2D closure. OCCT VALID is weaker than intuition here.
- **23-point sampling can miss spikes**: default (approx) mode only samples; `theIsExact` (GeomLib global
  extremum) is opt-in. An OCCT-VALID verdict from the default analyzer is a sampled claim, not a proof.
- **BRepCheck never intersects faces in 3D**: no face-face interference inside a shell, no shell-shell clash
  except point-in-solid probes at Solid::Minimum. A self-intersecting (but topologically coherent) solid is
  OCCT-VALID. Our oracle acceptance therefore does NOT certify absence of geometric self-intersection —
  matches what we saw in the boolean campaign (kb/occt_interference-vef.md covers the construction-time guard
  that compensates inside BOPAlgo).
- **Edge FreeEdge counting counts USES, not faces**: a seam edge used twice by ONE face is 2 connections — not
  free. Any 3rd use → InvalidMultiConnexity even if it is the same face again.
- **Wire evenness, not two-ness**: vertex valence 4 in a wire passes `Closed()`; the Orientation walk with
  `ChoixUV` is what rejects genuinely forked wires (`nbconnex >= 2` after angular filtering →
  BadOrientationOfSubshape). Port both or the gate is leaky.
- **Shell closure records at solid level only when `(NotClosed && S.Closed())`** — an open shell inside a solid
  whose Closed flag is unset does NOT fail the solid via this path (it fails FreeEdge via Edge::InContext(SOLID)
  instead, if edges are naked). Flag semantics matter when emitting shapes for the oracle: set `Closed(true)`
  on shells we claim are closed, or Rhino/OCCT read different verdicts.
- **`Closed2d` needs `BRepTools_WireExplorer` to succeed**: the explorer silently drops edges it cannot chain in
  2D, which converts orientation bugs into NotClosed — statuses are not always the "true" root cause.
- **Exceptions become `CheckFail`, not crashes** — every Perform branch is wrapped in `OCC_CATCH_SIGNALS`; a
  throwing surface evaluator yields an invalid-but-diagnosed shape.
- **Tolerance asymmetries**: vertex-on-curve uses `max(tolV,tolE)`; wire 3D closure uses max of two VERTEX tols
  only (edge tol not consulted); self-intersect escape uses vertex tol inflated 1.1x; yawn uses 2x EDGE tol.
  These are four different budgets — mirroring them exactly matters for borderline slivers.

## PORT MAP

(OCCT mechanism → our anchor → action, 1-line design.)

1. Shell naked/manifold parity (`BRepCheck_Shell::Closed`: 1-use non-degenerate edge = NotClosed, >=3 =
   multiconnexity, INTERNAL excluded) → `BRep::topology_report` (brep.cpp) → **adopt**: our naked/non-manifold
   counters already match; add the `aMEToAvoid` rule (exclude any edge that ALSO appears non-oriented) and the
   degenerated-edge exemption so counts equal OCCT on seam/apex-bearing solids.
2. Opposite-use orientation rule + reorientability flood (`BRepCheck_Shell::Orientation`) →
   `combine` (brep.cpp: exact weld + tube merge + NK-RESCUE) → **new-build**: post-weld gate asserting every
   welded edge is used FWD once + REV once (seams: twice by one face with both orientations); run BEFORE trusting
   winding+radial classification — catches the all-faces-flip failure class (chairs z15/z30 tier-3 fix) cheaply.
3. Same-parameter containment (`BRepCheck_Edge::InContext(FACE)` + `BRepLib_ValidateEdge`, 23 pts, tol =
   edge tol + eps) → `brep.cpp split_with` lifted chains / `brep_section.cpp` section pcurves → **new-build**:
   self-audit sampling `|C3d(t) - S(uv(t))|` at 23 params on every emitted (3D chain, UV run) pair; feed max
   distance into the edge tolerance we write (OCCT law: tol must contain it, report dist*1.00001) instead of a
   fixed section tolerance.
4. Wire vertex-parity closure (`BRepCheck_Wire::Closed` even-valence + RedundantEdge same-orientation rule) →
   `split_with` loop assembly / scaffold closure-weld (brep_section.cpp) → **adopt**: verify per-vertex even
   incidence per rebuilt loop rather than endpoint-chaining alone; flags the SEGLOST/holes class (face losing
   its section keys) as a parity error before the oracle sees it.
5. `ChoixUV` angle-minimal continuation (min UV angle on FORWARD face, max on REVERSED; derivative ladder
   D1..D100 for degenerate tangents) → valence-1 bridge + NK-RESCUE candidate choice → **replace**: use
   angle-minimal continuation at the pivot vertex in UV to pick the bridge/rescue partner deterministically;
   our current nearest-endpoint choice can pick the wrong branch at 4-valent junction vertices.
6. Vertex weld contract (`BRepCheck_Vertex::InContext`: endpoint within `max(tolV,tolE)`) → welded vertices in
   `build_section_scaffold` → **adopt**: set written vertex tolerance to measured max endpoint deviation * 1.05
   (OCCT's own estimator margin, `Tolerance()` functions) so reimported solids pass the vertex checks exactly.
7. Interior-crossing-only self-intersection law (Middle transition + 1.1*tolV vertex escape + 2*tolE yawn
   wedge) → UV arrangement in `split_with` (cut-node crossing snap) → **adopt** the escape thresholds: a
   detected UV crossing within 1.1*vertex-tol of an existing node is a legal touch to be snapped, not a split —
   directly the tangential newton_cc stall class (0.008 spurious splits) with OCCT-calibrated constants.
8. Face imbrication + hole orientation (`ClassifyWires`/`OrientationOfWires`: infinite-point test, one outer
   wire, holes reversed) → per-face loop emission in `split_with` → **adopt**: after run lifting, classify loops
   by signed UV area (our analog of PerformInfinitePoint), require exactly one outer + holes opposite-signed;
   currently implicit in winding classification — make it a hard per-face gate with these exact statuses.
9. Growth/hole solid audit (`BRepCheck_Solid::Minimum`: one growth, probes at aPAR_T, EnclosedRegion,
   imbrication of shells) → multi-shell boolean outputs (xor = disjoint assembly; fuse of disjoint bodies) →
   **new-build (small)**: after combine, when >1 closed shell results, classify shells in/out with our winding
   contains_point at OCCT's probe recipe (edge point at t=0.43213918 blend) and assert single-growth.
10. `IsValid` aggregation + `BRepCheck::Add` NoError-removal law → validation/ oracle interpretation →
    **reference**: OCCT VALID == all lists NoError, first-element test is sufficient; and VALID does NOT test
    3D face-face self-intersection — never cite OCCT-VALID as proof of non-self-intersection (keep volume/flux
    and Rhino headless probes as the geometric backstop).
11. Exact mode (`GeomLib_CheckCurveOnSurface` via `BRepCheck_Analyzer(..., theIsExact=true)`) → oracle harness →
    **adopt (oracle-side)**: run the analyzer with exact=true in validation scripts for frontier cells; default
    sampling has passed shapes whose pcurves spike between the 23 samples.
