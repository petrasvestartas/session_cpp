# tolerance-model

OCCT boolean-kernel tolerance doctrine, extracted from source at
`C:/brg/compas_occt/external/occt/src/occt/src/ModelingAlgorithms/TKBO/` (BOPAlgo, BOPTools, IntTools, BOPDS)
plus the two out-of-toolkit primitives it relies on (`BRep_Builder`/`BRep_TVertex`/`BRep_TEdge` in TKBRep, `BRepLib_ValidateEdge` in TKTopAlgo).

The doctrine in one sentence: **geometry is never moved — every incidence decision is a distance-vs-sum-of-tolerances test, fuzzy is an additive threshold that is never stored, and whenever a measured distance exceeds the stored tolerance the tolerance (not the point) grows to `distance + delta`, monotonically, with a final whole-shape audit pass.**

---

## STAGE PIPELINE

Ordered as executed by `BRepAlgoAPI_BooleanOperation` → `BOPAlgo_PaveFiller::Perform` → `BOPAlgo_Builder::PerformInternal1`.

### 0. Fuzzy value intake and clamping
- **Purpose**: single user-facing "slop" knob; clamp so it can never be below kernel precision.
- **Where**: `BOPAlgo/BOPAlgo_Options.cxx` — ctor (`myFuzzyValue = Precision::Confusion()`, L49–57) and `SetFuzzyValue` (L105–108): `myFuzzyValue = std::max(theFuzz, Precision::Confusion())`.
- **Flow**: `BOPAlgo_Builder::Perform` (`BOPAlgo_Builder.cxx` L185) → `pPF->SetFuzzyValue(myFuzzyValue)`; read back in `PerformInternal1` (L316). Every intersection object receives it: `SetFuzzyValue` on `IntTools_EdgeEdge` (PaveFiller_3 L264), `IntTools_EdgeFace` (PaveFiller_6 L3475), `IntTools_FaceFace` (PaveFiller_6 L507), `BOPAlgo_VertexEdge` (PaveFiller_2 L290).
- **In/out**: user double → clamped member; never persisted on shapes.

### 1. DS init: half-fuzz box inflation
- **Purpose**: make the broad-phase (bounding-box tree) see everything the fuzzy-widened narrow phase might accept.
- **Where**: `BOPAlgo_PaveFiller.cxx` `Init` L200 → `BOPDS_DS::Init(theFuzz)` (`BOPDS/BOPDS_DS.cxx` L285–324): `anAdditionalTolerance = std::max(theFuzz, Precision::Confusion()) * 0.5` (L312), then
  - `prepareVertices` (L1589): vertex box gap = `BRep_Tool::Tolerance(V) + add` (L1605);
  - `prepareEdges` (L1614): edge box = curve box + vertex boxes, then `SetGap(GetGap() + add)` (L1688);
  - `prepareFaces` (L1696): `SetGap(GetGap() + add)` (L1775).
- **Why half**: each *pair test* sums two operand tolerances; giving each box fuzz/2 makes box overlap consistent with `tolA + fuzz/2 + tolB + fuzz/2 = tolA + tolB + fuzz`.
- **When**: once, before any interference stage. Same convention on shrunk-range boxes: PaveFiller_3 L822 `aBox.SetGap(aBox.GetGap() + myFuzzyValue / 2.)`.

### 2. V/V interference — fuzzy-widened coincidence, fused vertex covers the cluster
- **Where**: `BOPAlgo_PaveFiller_1.cxx` L93: `BOPTools_AlgoTools::ComputeVV(aV1, aV2, myFuzzyValue)`.
- **Predicate** (`BOPTools/BOPTools_AlgoTools.cxx` L1772–1794): `aTolSum = tolV1 + tolV2 + max(fuzz, Confusion)`; coincident iff `dist² ≤ aTolSum²`. Point-vs-vertex variant L1750–1768 uses `tolV1 + tolP2 + Confusion`.
- **Fusion**: `MakeVertex` (L1798–1813) → `BRepLib::BoundingVertex`: NEW vertex at cluster barycenter with tolerance covering all member tolerance spheres. This is the *only* place "geometry moves" — and it is a **new** vertex; the originals are untouched and become SD (same-domain) images of it.

### 3. V/E and V/F interference — tolerance growth = measured distance + representation tolerance
- **Where**: `BOPAlgo_PaveFiller_2.cxx` (VE) and `_4.cxx` (VF); predicates in `IntTools/IntTools_Context.cxx`:
  - `ComputeVE` (L499–541): accept iff `dist ≤ tolV + tolE + max(fuzz, Confusion)`; **outputs `theTol = dist + tolE`** — the tolerance the vertex must have to legitimately contain its projection onto the edge.
  - `ComputeVF` (L545–590): accept iff `dist ≤ tolV + tolF + max(fuzz, Confusion)`; `theTol = dist + tolF`; then 2D point-in-face classification (`IsPointInFace`).
  - `ComputePE` (L437–495): point-edge, `aTolSum = aTolP1 + aTolE2 + Confusion`.
- **Growth site**: PaveFiller_2 L337–338 `aTolVNew = aVESolver.VertexNewTolerance(); UpdateVertex(nV, aTolVNew)`; PaveFiller_4 L286 same after `ComputeVF`. Also forced variants `ForceInterfVE` (PaveFiller_3 L867–883) and `ForceInterfVF` (PaveFiller_5 L631–657).
- **The growth primitive** is stage 10's `BOPAlgo_PaveFiller::UpdateVertex`.

### 4. E/E interference — fuzz split per operand; new vertex covers common part
- **Where**: `IntTools/IntTools_EdgeEdge.cxx` L149–152: `aTolAdd = myFuzzyValue / 2; myTol1 = tolC1 + aTolAdd; myTol2 = tolC2 + aTolAdd; myTol = myTol1 + myTol2` (criterion).
- **Vertex creation**: `BOPAlgo_PaveFiller_3.cxx` VERTEX case L369–530:
  - `BOPTools_AlgoTools::MakeNewVertex(aE1,aT1,aE2,aT2,...)` (`BOPTools_AlgoTools_2.cxx` L224–250): NEW point = midpoint of the two curve points, tol = `max(tolE1, tolE2) + 0.5*dist`.
  - If the intersection lands on an existing pave (`bIsOnPave`), do NOT create a vertex; instead if the contact is a real crossing (`aPOnE1.Distance(aPOnE2) > Precision::Intersection()` → touching; else) grow the existing vertex: `UpdateVertex(nV[j], aDistPP)` where `aDistPP = |P_existing − P_new|` (L440–451), and add it to `myVertsToAvoidExtension`.
  - **Analytic guard** (L455–466): for Line/Circle pairs the tolerance is widened to half the common-part parametric span (`(aCR.Last()-aCR.First())/2`) but *not yet written to the vertex* — "increase tolerance ... but do not update the vertex till its intersection with some other shape". Stored in the couple: `aCPB.SetTolerance(aTolVnew)` L524.
  - LXBR proximity test L468–499: candidate new vertex is dropped in favor of an existing shared endpoint if `dist² < 100·(tolVnew + tolVx)²` — a 10× tolerance-radius capture band.
- **Self-interference mode** (ForceInterfEE, L1111–1222): fuzzy for the pass is replaced by `2*max(tolV1, tolV2)` — vertex tolerances act as the fuzzy when hunting common blocks between pave blocks that share both end vertices.

### 5. E/F interference — same doctrine
- **Where**: `IntTools/IntTools_EdgeFace.cxx` L529–531: `aFuzz = myFuzzyValue/2; aTolF = tolF + aFuzz; aTolE = tolE + aFuzz` → criteria = sum.
- **Vertex creation**: `BOPAlgo_PaveFiller_5.cxx` VERTEX case L440–542: new vertex tol = `max(tolVnew, max(tolE, tolF))` (L510–512); Line/Plane guard widens by half common-part range (L513–519); on-pave crossing grows existing vertex by `aDistPP` capped by `aMaxDist = min(1e4·tolV, 0.1)` when `tolV < 0.01` (L482–501).
- New vertices fused across EF results via `BOPAlgo_Tools::IntersectVertices(aVerts, myFuzzyValue, aChains)` (PaveFiller_3 `PerformNewVertices` L607–717, `aTolAdd = myFuzzyValue/2`).

### 6. Common blocks — tolerance = max sampled projection distance
- **Purpose**: when several pave blocks (edge pieces, possibly on faces) are geometrically coincident, they merge into one `BOPDS_CommonBlock` whose tolerance must cover ALL members.
- **Where**: `BOPAlgo/BOPAlgo_Tools.cxx` `ComputeToleranceOfCB` (L248–356): take representative pave block's edge curve, sample **11 interior points** (`aNbPnt = 11`), project each onto every other member edge (`ProjPC`) and every member face (`ProjPS`); `aTolCB = max(tol_member + LowerDistance)` over all samples and members. Set at both `PerformCommonBlocks` overloads (L184–185, L241–242).
- **Propagation**: `UpdateVerticesOfCB` (PaveFiller_3 L959–993): `UpdateVertex(Pave1, aTolCB); UpdateVertex(Pave2, aTolCB)`. On split, the real edge gets it: PaveFiller_7 L453–454 `ComputeToleranceOfCB` → `UpdateEdgeTolerance(nE, aTol)`; L541 `UpdateEdgeTolerance(nSp, aBSE.Tolerance())`.
- **Coincidence re-check when merging CBs**: `BOPDS_DS::UpdateCommonBlock`/`CheckCoincidence` (`BOPDS_DS.cxx` L557–632, L1288–1326): project midpoint of PB1 onto edge of PB2; coincident iff `dist < MaxTolerance(E1 incl. vertices) + MaxTolerance(E2 incl. vertices) + max(fuzz, Confusion)`.

### 7. F/F intersection — section-curve tolerance is MEASURED, then floored
- **Fuzz intake**: `IntTools/IntTools_FaceFace.cxx` `Perform` L381–387: `aFuzz = myFuzzyValue/2; myTolF1 = tolF1 + aFuzz; myTolF2 = tolF2 + aFuzz; myTol = myTolF1 + myTolF2` → `TolArc = TolTang = myTol` fed to the intersector.
- **Initial curve tolerance**: `BOPAlgo_PaveFiller_6.cxx` `ToleranceFF` (L3922–3942): `aTolFF = max(tolF1, tolF2)`, and **if either surface is non-analytic** (not Plane/Cyl/Cone/Sphere/Torus) **floor at 5.e-6**. Applied L495–496 (`max(shiftValue, ToleranceFF)` for closed-edge shifted faces) and stored on each curve: `aNC.SetTolerance(max(aIC.Tolerance(), aTolFF))` L607. Curve boxes enlarged by `aTolFF` (possibly raised to max vertex tolerance of the two faces, L580–607).
- **Measured tolerance** (`IntTools_FaceFace::ComputeTolReached3d`, L613–691): for each section curve, for each of the two pcurves: `IntTools_Tools::ComputeTolerance(C3d, C2d, S, f, l, ...)` (`IntTools_Tools.cxx` L737–779) = max distance between the 3D curve and the surface-composed pcurve (via `GeomLib_CheckCurveOnSurface`), multiplied by margin `(1.0 + 1.0e-5)` ("obtaining precise result is impossible ... we must provide some margin"); if a pcurve is absent, `FindMaxDistance(C3d, surface)` instead. Curve tol = max of all. **Tangential tolerance**: default `max(myTolF1, myTolF2)` (L686–689); exact only for Plane/Plane (L2541–2555): `aDt = IntTools_Tools::ComputeIntRange(TolF1, TolF2, angle)` = `TolF2` at right angle else `TolF1·tan(π/2−α) + TolF2/sin(α)` (`IntTools_Tools.cxx` L783–804); `tangTol = sqrt(aDt² + TolF1²)`.
- **Working radius everywhere downstream**: `aTolR3D = max(aNC.Tolerance(), aNC.TangentialTolerance())` (PaveFiller_6 L886, L2317, L2384, L4023).

### 8. Paves on section curves — snap by projection, grow vertex to cover the gap
- **`PutPaveOnCurve`** (PaveFiller_6 L2959–3068): vertex-on-curve test `myContext->IsVertexOnLine(aV, aTolV, aIC, aTolR3D + myFuzzyValue, aT)` (L2976). `IsVertexOnLine` (`IntTools_Context.cxx` L775+): `aTolSum = 2·(aTolV + aTolC)`, floored at `1e-5` for B-spline/Bezier curves, `1e-6` otherwise — an intentional 2× + absolute-floor slack for curve-end matching. Parametric pave dedup radius: `aPTol = aGAC.Resolution(max(aTolR3D, aTolV))` (L3002). If the pave is new and `tolV < dist + DTolerance`: `BRep_Builder().UpdateVertex(aV, aDist + aDTol)` where `aDTol = BOPTools_AlgoTools::DTolerance() = 1e-12` (L3048–3054) and DS box refreshed with `+Confusion` gap.
- **`ExtendedTolerance`** (L2542–2604): only for NEW vertices not in `myVertsToAvoidExtension` — search radius may be extended to the farthest end of the EE/EF **common part range** that created the vertex (`aD = max(|PV−P11|, |PV−P12|)`); if the retry succeeds the vertex tolerance becomes the actual distance to the curve (L2982–2988).
- **`PutBoundPaveOnCurve`** (L2308–2368): curve-end vertices: `MakeNewVertex(P_end, aTolR3D)` then `BOPTools_AlgoTools::UpdateVertex(aIC, aT, aVn)` (`BOPTools_AlgoTools_2.cxx` L80–98: if `dist(PV, C(T)) > tolV` → `UpdateVertex(V, dist + 1e-12)`). End-vertex matching L2287–2303 uses `tol = max(NC.Tolerance(), NC.TangentialTolerance()) + Confusion` via `ComputeVV(V, P, tol)`.
- **`PutClosingPaveOnCurve`** (L3555–3604): closed-curve seam: closed iff `dist(PV, P_opposite) ≤ tolV + (max(tolC, tangTol) + Confusion)`; then `aNewTolV = max(tolV, distVP + DTolerance)`; `FindValidRange` must survive with that tolerance; `UpdateVertex(nV, aNewTolV)` (L3591).

### 9. Section-edge construction and rejection — every acceptance grows something
- **Where**: `BOPAlgo_PaveFiller_6.cxx` `MakeBlocks` L882–1019.
- Degenerate span dropped at `|aT1−aT2| < Precision::PConfusion()` (L906). 2D validity via `IsValidBlockForFaces(..., aTolR3D)` (L914).
- **Existing-edge rescue**: if the candidate block coincides with an already-existing edge (`IsExistingPaveBlock` — search radius `aTolCheck = theTolR3D + myFuzzyValue`, L1956–1960 / L2104–2127 with per-pair `max(tolV..) + myFuzzyValue`), the section edge is NOT built; instead the existing edge is grown: `UpdateEdgeTolerance(nEOut, aTolNew)` (L926, and L976–984 with `aTolNew = max(aTolNew, aNC.Tolerance())` "use real tolerance of intersection").
- **Micro pave block**: `BRepLib::FindValidRange(curve, aTolR3D, T1, P(V1), max(aTolR3D, tolV1), T2, P(V2), max(aTolR3D, tolV2))` (L937–946) — if the block is entirely inside its vertices' tolerance spheres it becomes vertices-only; later (PostTreatFF L1339–1358) the two vertices are FORCED to fuse: if `dist − (tolV1 + tolV2) > 0`: `UpdateVertex(V1, tolV1 + dist/2); UpdateVertex(V2, tolV2 + dist/2)` — each vertex grows half the residual gap so `ComputeVV` must succeed.
- **Edge build**: `BOPTools_AlgoTools::MakeEdge` (`BOPTools_AlgoTools.cxx` L1729–1746): `aNeedTol = theTolR3D + DTolerance(); UpdateVertex(V1, aNeedTol); UpdateVertex(V2, aNeedTol); ... UpdateEdge(E, theTolR3D)` — a section edge's vertices always carry at least the curve tolerance + 1e-12.
- `UpdateBlocksWithSharedVertices` (L3946–4051): old vertices shared by both faces are put on curves (`EstimatePaveOnCurve` with `aTolR3D`) and re-anchored via `UpdateVertex(nV, aTolV)` (L4045) to force SD-image creation in non-destructive mode.

### 10. The growth primitives (the only two mutators)
- **`BOPAlgo_PaveFiller::UpdateVertex(nV, aTolNew)`** (`BOPAlgo_PaveFiller_10.cxx` L105–162):
  - if vertex is new / has SD image / destructive mode: `if (tolV < aTolNew) BB.UpdateVertex(V, aTolNew)`, refresh DS box (`+Confusion` gap), record in `myIncreasedSS`.
  - **non-destructive + old vertex: copy-on-write** — make a NEW vertex at the SAME point `aPV` with `tol = max(tolV, aTolNew)`, append to DS, register SD map `AddShapeSD(nV, nVNew)`, add to `myVertsToAvoidExtension`.
- **`BOPAlgo_PaveFiller::UpdateEdgeTolerance(nE, theTol)`** (L63–101): refuses in non-destructive mode if the edge or any of its vertices is an unprotected original; otherwise `BB.UpdateEdge(E, theTol)`, box refresh, then `UpdateVertex(nV, theTol)` for every edge vertex — **edge growth always propagates to its vertices**.
- Underneath: `BRep_Builder::UpdateVertex/UpdateEdge` → `BRep_TVertex::UpdateTolerance` / `BRep_TEdge::UpdateTolerance` — documented "Sets the tolerance to the max of `<T>` and the current tolerance" (`TKBRep/BRep/BRep_TVertex.hxx` L43–45, `BRep_TEdge.hxx` L50–52). **Monotone non-decreasing by construction.**

### 11. PCurves / same-parameter maintenance after splitting
- **Where**: `BOPAlgo_PaveFiller_7.cxx` `MakePCurves` L600–801: pcurves are projected for IN/ON pave-block edges and section edges; result written by `BRep_Builder().UpdateEdge(edge, newPC, face, aMPC.GetNewTolerance())` (L797).
- **`UpdateVertices(aE, aF)`** (L808–846): at both edge ends compare `C3D(t)` vs `S(C2D(t))`; if `d² > tolV²` → `UpdateVertex(V, d + DTolerance)` — vertices absorb 3D-vs-pcurve end mismatch.
- `BOPTools_AlgoTools::MakePCurve` (`BOPTools_AlgoTools.cxx` L1657–1725): adjusts/creates the pcurve then `BRepLib::SameParameter(aE)` (L1724) — canonical same-parameter re-approximation for the section edge.
- `Prepare` (L850–932): pcurves on planes are precomputed (destructive mode only) at unchanged `tolE` (L928–929).
- Degenerate-edge splits: new degenerate edges get `UpdateEdge(aE, Precision::Confusion())` (`PaveFiller_8.cxx` L115) or fixed `1e-7` (`MakeSplitEdge1`, L343/356).

### 12. Final audit — `PostTreat`: CorrectTolerances + CorrectShapeTolerances
- **Call site**: `BOPAlgo_Builder.cxx` `PostTreat` L450–475:
  - `aMA` ("map to avoid") = all source VERTEX/EDGE/FACE shapes, filled **only in non-destructive mode** (L455–468) — those shapes' tolerances may not be touched.
  - `BOPTools_AlgoTools::CorrectTolerances(myShape, aMA, 0.05, myRunParallel)` then `BOPTools_AlgoTools::CorrectShapeTolerances(myShape, aMA, myRunParallel)`.
  - Same pair also used on intermediate face sets: `BOPAlgo_Tools.cxx` L794–795.
- **`CorrectTolerances`** (`BOPTools/BOPTools_AlgoTools_1.cxx` L309–317) = `CorrectPointOnCurve` + `CorrectCurveOnSurface`, `aMaxTol = 0.05` cap:
  - `CorrectPointOnCurve` (L322–344) → per-edge `CheckEdge` (L430–517): for every vertex of every edge, compare vertex point vs (a) each stored point-on-curve representation and (b) the 3D curve evaluated at the vertex's end parameter; if `d² > max(tolV, tolE)²` → `newTol = sqrt(d²) + 0.1·max(tolV,tolE)`; applied only if `newTol < aMaxTol` via `UpdateShape` (L1066–1089; skips shapes in `aMapToAvoid`).
  - `CorrectCurveOnSurface` (L348–385) → per-face `CorrectWires` + per-(edge,face) `CorrectEdgeTolerance`:
    - `CorrectWires` (L665–757): at each face vertex, distance from vertex point to `S(C2D(t_V))` for every incident edge, AND to 2D self-intersections of incident-edge pcurve pairs (`IntersectCurves2d` L557–661, `aTol2d = 1e-10`, intersection accepted only within half-range of the vertex parameter and capped by `(0.3·min(len1,len2))²` — "MaxEdgePartCoveredByVertex = 0.3"); if `d²max > tolV²` → `UpdateShape(V, 1.01·sqrt(d²max))`.
    - `CorrectEdgeTolerance` (L761–1001): re-runs same-parameter validation per (edge, face) — `BRepLib_ValidateEdge` on 3D curve vs each curve-on-surface representation (both pcurves of closed surfaces, L924–940; on-the-fly plane projection when no pcurve, L945–998); `UpdateTolerance` (`TKTopAlgo/BRepLib_ValidateEdge.cxx` L57–63) proposes `maxDist · 1.00001`; if it exceeds current tolerance and `< aMaxTol`: `UpdateShape(edge, newTol)` + `CorrectVertexTolerance(edge)`.
- **`CorrectShapeTolerances`** (L389–423) — enforces the containment hierarchy:
  - `CorrectVertexTolerance` (L1005–1023): every vertex of an edge gets `tolV ≥ tolE`.
  - `UpdateEdges` (L1027–1062): every edge of a face gets `tolE ≥ tolF`; face-level vertices get `tolV ≥ tolF`.
- **When**: last step of the boolean; result invariant: valid `BRepCheck` shape without moving any geometry.

---

## DATA STRUCTURES

- **`BOPAlgo_Options`** (`BOPAlgo_Options.hxx/.cxx`) — `myFuzzyValue` (clamped ≥ `Precision::Confusion()`); inherited by PaveFiller, Builder, BOP, Splitter, CellsBuilder, BuilderSolid. One knob, plumbed everywhere, never stored on shapes.
- **`BOPDS_ShapeInfo`** (BOPDS) — per-shape record: `Shape()`, `ChangeBox()` (Bnd_Box whose **gap** carries `tol + fuzz/2`), `SubShapes()`, flags. Tolerance growth must refresh the box (`BRepBndLib::Add` + `SetGap(+Confusion)`) or later broad-phase misses the widened entity — done in `UpdateVertex`/`UpdateEdgeTolerance` (PaveFiller_10 L90–92, L120–123).
- **`BOPDS_Pave` / `BOPDS_PaveBlock`** — pave = (vertex index, parameter); pave block = edge subrange between two paves; carries **shrunk data** `(TS1, TS2, BndBox, IsSplittable)` — the part of the range outside the end vertices' tolerance spheres. `ContainsParameter(aT, aPTol, nVUsed)` is the parametric dedup gate for new paves.
- **`BOPDS_CommonBlock`** — list of coincident pave blocks + face indices + **`Tolerance()`**: the sampled max deviation (stage 6). Its tolerance is the band that makes "these edges are the same edge" true; propagated to the surviving edge and end vertices.
- **`BOPDS_Curve`** (per FF section curve) — `IntTools_Curve` + box + `Tolerance()` + `TangentialTolerance()`. Working radius is always `max` of the two.
- **`IntTools_Curve`** (`IntTools_Curve.hxx` L120–126) — 3D curve, two pcurves, `myTolerance` (measured 3D deviation), `myTangentialTolerance` (lateral uncertainty of tangential contact). Two DIFFERENT tolerances with different semantics — key design point.
- **`IntTools_ShrunkRange`** (`IntTools_ShrunkRange.cxx` L107–191) — computes `[TS1, TS2]` = range minus vertex spheres, using `tolV_i := max(tolV_i, tolE) + Confusion`; `myIsSplittable = length > 2·tolE + 2·Confusion` ("minimal diameter of tolerance sphere of splitting vertex" + "minimal length of the new edges"); box gap `tolE + Confusion` (+ fuzz/2 at call site).
- **`IntTools_Context`** — cache of projectors (`ProjPC`, `ProjPS`, `ProjPT`), classifiers (`FClass2d`), surface adaptors; all tolerance predicates (`ComputeVE/VF/PE`, `IsVertexOnLine`, `IsValidBlockForFaces`, `IsPointInFace`) live here so every stage tests incidence with the SAME formulas.
- **PaveFiller bookkeeping sets** (`BOPAlgo_PaveFiller.hxx`):
  - `myVertsToAvoidExtension` — vertices whose tolerance was already grown from a real intersection; `ExtendedTolerance`/`PutPaveOnCurve` must not extend them further (anti-runaway).
  - `myIncreasedSS` — indices of entities whose tolerance was increased (drives warnings and re-iteration).
  - SD map (`AddShapeSD/HasShapeSD`) — old→new vertex aliases from fusion or copy-on-write growth; every interference lookup resolves through it (`UpdateInterfsWithSDVertices`, `UpdatePaveBlocksWithSDVertices`).
- **`aMapToAvoid`** (`BOPTools_AlgoTools_1.cxx`, threaded through all Correct* helpers) — the non-destructive fence: `UpdateShape` (L1070) silently refuses to touch fenced shapes.

---

## CONSTANTS & TOLERANCES

| Constant | Value | Where / role |
|---|---|---|
| `Precision::Confusion()` | `1e-7` | universal additive floor; default fuzzy; box-gap refresh delta |
| `Precision::PConfusion()` | `1e-9` (`Confusion·0.01`) | parametric degeneracy: pave-block span, range checks |
| `Precision::Intersection()` | `1e-9` (`Confusion·0.01`) | "real crossing vs touching" test (PaveFiller_3 L432, _5 L472) |
| `Precision::Angular()` | `1e-12` | plane-angle right-angle test in `ComputeIntRange` |
| `BOPTools_AlgoTools::DTolerance()` | `1e-12` | "delta tolerance ... slightly bigger than the actual distances ... avoid numerical instability" (`BOPTools_AlgoTools.hxx` L64–70); added to every `dist → tol` write |
| fuzzy default / clamp | `Confusion()`; `max(theFuzz, Confusion())` | `BOPAlgo_Options.cxx` L53, L107 |
| box inflation | `max(fuzz, Confusion)·0.5` | `BOPDS_DS::Init` L312; shrunk-range box `+fuzz/2` (PaveFiller_3 L822) |
| pair criteria | `tolA + fuzz/2 + tolB + fuzz/2` | EE (`IntTools_EdgeEdge` L149–152), EF (`IntTools_EdgeFace` L529–531), FF (`IntTools_FaceFace` L381–384) |
| point criteria | `tolV + tolX + max(fuzz, Confusion)` | `ComputeVV` L1782, `ComputeVE` L532, `ComputeVF` L573 |
| VV fused vertex | barycenter, covering tol (`BRepLib::BoundingVertex`) | `MakeVertex` L1798 |
| EE new vertex | midpoint, `max(tolE1,tolE2) + 0.5·dist` | `MakeNewVertex` (`_2.cxx` L224–250) |
| EF new vertex | `max(tolVnew, tolE, tolF)` | PaveFiller_5 L510–512 |
| analytic common-part guard | `(range.Last − range.First)/2` | Line/Circle (PaveFiller_3 L455–466), Line/Plane (PaveFiller_5 L513–519) |
| EE dup-capture band | `dist² < 100·(tolVnew + tolVx)²` (10× radius) | PaveFiller_3 L497 |
| EF on-pave growth cap | `min(1e4·tolV, 0.1)` when `tolV < .01` | PaveFiller_5 L490–494 |
| FF curve tol floor (freeform) | `5.e-6` if either surface non-analytic | `ToleranceFF` L3937–3939 |
| FF measured-tol margin | `×(1.0 + 1.0e-5)` | `IntTools_Tools::ComputeTolerance` L774; same in `BRepLib_ValidateEdge` (`×1.00001`) |
| plane/plane tangential tol | `sqrt(aDt² + TolF1²)`, `aDt = TolF1·tan(π/2−α) + TolF2/sin(α)` (or `TolF2` at α=π/2) | FaceFace L2541–2555, `ComputeIntRange` L783–804 |
| working section radius | `aTolR3D = max(curveTol, tangTol)`; searches use `aTolR3D + fuzz` | PaveFiller_6 L886, L1960, L2032, L2976 |
| vertex-on-curve-end slack | `2·(tolV + tolC)`, floor `1e-5` (bspline/bezier) / `1e-6` (analytic) | `IsVertexOnLine` L789–808 |
| parametric pave dedup | `Resolution(max(aTolR3D, tolV))` | PaveFiller_6 L3002 |
| micro-block vertex fuse | each vertex `+ dist_residual/2` | PostTreatFF L1348–1357 |
| CB tolerance sampling | 11 interior points, `max(tol_member + projDist)` | `ComputeToleranceOfCB` L271–356 |
| final audit cap | `aMaxTol = 0.05` (absolute) | `PostTreat` L472 |
| CheckEdge growth pad | `sqrt(d²) + 0.1·tol` | `_1.cxx` L452, L477 |
| CorrectWires growth | `1.01·sqrt(d²max)`; 2D tol `1e-10`; vertex may cover ≤ `0.3` of shortest edge | `_1.cxx` L751–754, L572, L627–628 |
| shrunk-range splittable | `length > 2·tolE + 2·Confusion` | `IntTools_ShrunkRange` L184 |
| self-interference EE fuzzy | `2·max(tolV1, tolV2)` instead of user fuzzy | PaveFiller_3 L1116–1118 |

---

## INVARIANTS

1. **Geometry is never moved.** No existing vertex point, curve, or surface is repositioned anywhere in TKBO. The only 'moved' geometry is *newly created* vertices (VV barycenter, EE/EF midpoint), which immediately supersede the originals via the SD map. Growth writes go through `BRep_TVertex/TEdge::UpdateTolerance = max(current, new)` — tolerance is monotone non-decreasing for the whole operation.
2. **Fuzzy is a threshold, never a state.** `myFuzzyValue` widens acceptance tests (split fuzz/2-per-operand in pair tests, whole-fuzz in point tests, fuzz/2 in box gaps) but is never added to a stored tolerance. Result tolerances derive only from **measured distances** (+`1e-12` or ×1.00001 pads).
3. **Every accepted incidence is covered.** Whenever a test accepts `dist ≤ Σtol + fuzz` with `dist > tol_entity`, some entity's tolerance is grown to ≥ `dist (+ DTolerance)` before downstream stages run — so downstream may assume: *anything recorded as an interference is inside the tolerance spheres of the entities it references, without fuzzy*.
4. **Tolerance hierarchy** on the final result: `tolF ≤ tolE ≤ tolV` for containment chains (enforced by `CorrectShapeTolerances`; incrementally by `UpdateEdgeTolerance` propagating to vertices and `CorrectVertexTolerance`).
5. **Boxes always cover tolerances.** Any tolerance growth refreshes the DS bounding box and re-adds a `Confusion` gap; broad phase never under-reports after growth.
6. **Section edge ⊃ curve tolerance.** A built section edge has `tolE ≥ aTolR3D` and its vertices `≥ aTolR3D + 1e-12` (`MakeEdge`); an *existing* edge reused as section edge is grown to `max(existing, measured, curve tol)`.
7. **Non-destructive mode**: input shapes are bitwise untouched — growth on old vertices is copy-on-write via SD images; `PostTreat`'s map-to-avoid fences source V/E/F from the audit pass; `UpdateEdgeTolerance` refuses old edges entirely.
8. **Same-parameter validity**: after `PostTreat`, every (edge, face) pcurve representation deviates from the 3D curve by less than `tolE` (that is what `BRepLib_ValidateEdge` re-measures, growing `tolE` if needed, capped 0.05).
9. **A vertex participating in one real intersection is fenced** (`myVertsToAvoidExtension`) against speculative tolerance extension by later stages — growth is evidence-driven, once.

---

## PITFALLS

Corner cases the source explicitly handles (comments/branches):

- **Fuzzy must be halved in symmetric tests, not doubled**: pair tests add fuzz/2 to *each* operand so pair-sum equals point-test's single `+fuzz`; mixing conventions double-counts slop (consistent in EE/EF/FF vs VV/VE/VF).
- **Runaway growth**: EF on-pave growth is capped (`min(1e4·tolV, 0.1)`), analytic Line/Circle & Line/Plane vertex tolerances are widened *locally* "but do not update the vertex till its intersection with some other shape" (PaveFiller_3 L457–458) — otherwise a tangential analytic contact inflates a vertex globally.
- **Touching vs crossing**: before growing an existing on-pave vertex, verify a *real* crossing with `Precision::Intersection()` (PaveFiller_3 L426–436, PaveFiller_5 L461–478: "If it is a touching point, do nothing").
- **Vertex swallowing an edge**: `CorrectWires`' 2D-self-intersection growth is bounded by `0.3·min(edge lengths)` squared ("MaxEdgePartCoveredByVertex = 0.3", `_1.cxx` L627) — a vertex may never cover more than 30% of an incident edge; out-of-range and far-end intersections are skipped (L638–649).
- **Micro entities**: pave blocks fully inside vertex spheres are demoted to vertex fusions (with forced `+dist/2` growth so the fuse actually succeeds); shrunk-range "micro edge" and "no valid range" branches (`IntTools_ShrunkRange` L148–156, L171–175) prevent splitting edges shorter than their own tolerance.
- **Measured-tolerance margin**: both `IntTools_Tools::ComputeTolerance` and `BRepLib_ValidateEdge` multiply the numerically-found max deviation by `1+1e-5` — the comment (L767–773) explains a *later, more precise* minimum search could otherwise find a bigger deviation and invalidate the edge after trimming.
- **Closed curves/seams**: `PutBoundPaveOnCurve` puts only one bound pave on a closed curve (L2335–2338) and skips entirely if either end already has a vertex; `PutClosingPaveOnCurve` re-tests closure with the *found vertex's* tolerance, not a global epsilon, and verifies a valid range remains before growing.
- **Curve-end projection traps** (`IsVertexOnLine` L813–870): local extrema search near a curve end can converge to the wrong branch — three explicit rejection conditions (`aT > mid-range`, distance > tolSum, converged back to the end point → clamp `aT = aFirst`), with a global-extrema fallback when `Extrema_LocateExtPC` fails.
- **B-spline curve-end slack floor 1e-5**: pure tolerance sums are too tight for approximated section curves; without the floor, end paves are missed and curves stay open (L793–800).
- **Freeform FF floor 5e-6**: approximated (non-analytic) intersection curves cannot be trusted below approximation accuracy; flooring the initial curve tolerance avoids rejecting good paves before the measured tolerance exists (`ToleranceFF` L3937–3939).
- **Degenerated edges**: skipped in splitting (`HasFlag`, PaveFiller_7 L410–414), rebuilt with fixed `Confusion`/`1e-7` tolerance and 2D-only ranges (`BB.Range(E, aF, aP1, aP2)`, PaveFiller_8) — no 3D tolerance semantics exist for them.
- **`CorrectEdgeTolerance` guards**: bails on `!SameRange && SameParameter`, multiple 3D curves, degenerate+curve mismatch, inverted range (`Last ≤ First`) — the audit only judges edges whose representation set is coherent.
- **Non-destructive audit fence**: without `aMapToAvoid` the final `CorrectTolerances` pass would mutate input-shape tolerances (they are shared by handle) — the fence is the entire non-destructive guarantee of the audit stage.
- **Edge growth without box refresh is a latent bug** — OCCT always pairs `UpdateEdge/UpdateVertex` with `BRepBndLib::Add` + `SetGap(+Confusion)` inside the DS (PaveFiller_10); growing a tolerance without re-gapping the box makes later broad-phase misses.

---

## PORT MAP

Anchors: `session_cpp/src/brep_section.cpp` (`build_section_scaffold`), `session_cpp/src/brep.cpp` (`split_with`, `combine`, classification). Current bands: `tol3 = diag*2e-3` (weld), `tol3_rep` (representation), NK-RESCUE `0.15*tol3`; no growth model, no per-entity tolerances.

1. **Per-entity tolerance storage + only-grow `update_vertex`/`update_edge`** (`BRep_TVertex/TEdge::UpdateTolerance`, `BOPAlgo_PaveFiller_10.cxx` UpdateVertex/UpdateEdgeTolerance)
   → anchor: `combine`'s vertex/edge arena (BOP2 pool records) → **new build**: add a `double tol` field to pooled vertices/edges (init `1e-7`-scale or STEP-carried), with `update_vertex(v, t) { tol = max(tol, t); }` and `update_edge` that propagates to both end vertices; all weld/merge/classify bands read `tolA + tolB + fuzz` instead of the single global `tol3`.

2. **Evidence-driven growth on weld** (`PutPaveOnCurve` L3048–3054, `UpdateVertices` _7 L830–845: tol := dist + 1e-12 whenever an accepted incidence exceeds current tol)
   → anchor: `combine` exact 1e-7 weld + NK-RESCUE tolerant mate-pair pass → **replace**: keep the weld, but record the actual mate gap into both vertices' `tol` at weld time; NK-RESCUE's `0.15*tol3` heuristic becomes "accept iff `gap ≤ tolV1 + tolV2 + fuzz`, then grow both to cover" — the rescue band self-calibrates per junction instead of being a global fraction.

3. **Measured section-curve tolerance** (`IntTools_FaceFace::ComputeTolReached3d` + `ToleranceFF` floor `5e-6` + margin `×1.00001`; working radius `aTolR3D = max(tol, tangTol)`)
   → anchor: `build_section_scaffold` marched chains (predictor + correct7 Gauss-Newton, index-corresponded p3/uvA/uvB) → **new build**: per-chain `tol_chain = 1.00001 × max_i max(|p3_i − SA(uvA_i)|, |p3_i − SB(uvB_i)|)` (residuals are already available from correct7), floored at `5e-6·scale` for freeform pairs; store on the chain and use it (not `tol3_rep`) for pave snapping, keep-verdict ON-bands, and `split_with` run lifting.

4. **Pave/vertex-on-curve snapping thresholds** (`IsVertexOnLine`: `2·(tolV + tolC)` with absolute floors `1e-5`/`1e-6`; parametric dedup `Resolution(max(tolR3D, tolV))`; search radius `tolR3D + fuzz`)
   → anchor: `build_section_scaffold` paves (trim-loop crossings, chain-chain crossings, vertex projections) → **adopt**: replace fixed-band vertex-projection acceptance with `2·(tolV + tol_chain)` + bspline floor; convert 3D dedup distance to parameter space via local `|C'|` (Resolution) — fixes chains whose paves alias under rotation (our trim-snapped `fb=47.99997` class).

5. **Common-block tolerance by sampling** (`BOPAlgo_Tools::ComputeToleranceOfCB`: 11 samples, `max(tol_member + projDist)`; propagate to surviving edge + vertices)
   → anchor: `combine` vertex-pair-keyed common-block tube merge → **adopt**: after tube merge, sample 11 points of the survivor against each merged mate (and both faces) and set survivor `tol_edge`; grow end vertices to the same. Gives SEGWHOLE/alias-key merges a principled band instead of whole-seg alias key tol `1e-2`.

6. **Micro-block demotion + forced vertex fuse** (`FindValidRange` micro test; PostTreatFF `+dist/2` symmetric growth)
   → anchor: `combine` micro-edge collapse → **replace**: before collapsing an edge shorter than its vertices' spheres, grow both endpoint tols by `(dist − tolV1 − tolV2)/2` when positive, then collapse; the collapse can then never re-open under a later exact test (our zero-span/boundary-hugging drops become tolerance-covered instead of deleted-and-hoped).

7. **Final audit pass** (`BOPAlgo_Builder::PostTreat` → `CorrectTolerances(0.05)` + `CorrectShapeTolerances`: CheckEdge `sqrt(d²)+0.1·tol`, CorrectWires `1.01·d`, ValidateEdge `d×1.00001`, then hierarchy `tolF ≤ tolE ≤ tolV`)
   → anchor: end of `brep.cpp` boolean (after `combine`, before STEP dump / naked-edge audit) → **new build**: `correct_tolerances(result, max_tol = 0.05·scale)`: (a) per edge, evaluate 3D chain vs both faces' charts at endpoints and interior samples (we hold uvA/uvB exactly — no projection needed), grow `tol_edge`; (b) per vertex vs each incident edge end, grow `tol_v = d + 0.1·tol`; (c) enforce hierarchy. Our "tolerance-insensitive naked" classes (x20, wire-gap slivers) become sub-tolerance closed instead of naked because the audit *records* the deviation the pipeline actually produced.

8. **Fuzzy as user threshold** (`BOPAlgo_Options::SetFuzzyValue` clamp; half-fuzz box inflation `BOPDS_DS::Init` L312; threshold-only semantics)
   → anchor: boolean entry point in `brep.cpp` (+ SSI candidate-pair AABB overlap test in `build_section_scaffold`) → **new build**: one `fuzzy` parameter (default `1e-7`-scale), clamped, added as `fuzz/2` to each operand's AABB gap for pair candidacy and as `+fuzz` in every point-incidence predicate; never added into stored `tol` fields. Replaces the ad-hoc family of SESSION_* tolerance gates with a single doctrine-consistent knob.

9. **Shrunk range for verdicts** (`IntTools_ShrunkRange`: range minus vertex spheres; splittable iff `len > 2·tolE + 2·Confusion`)
   → anchor: `build_section_scaffold` per-interval keep-verdict (9 samples, bisected ends) → **already-equivalent in intent, adopt the formula**: derive the sampled subrange by subtracting endpoint tolerance spheres (`FindValidRange` analog) instead of fixed end bisection; refuse to split intervals shorter than `2·tol_chain + 2e-7` (kills the micro-piece filter's magic constant).

10. **Touch-vs-cross gate before growth** (`Precision::Intersection()` test PaveFiller_3 L432 / _5 L472; growth caps `min(1e4·tolV, 0.1)`; `myVertsToAvoidExtension` fence; `CorrectWires` 0.3-edge-coverage cap)
    → anchor: `combine` classification pre-pass + valence-1 bridge (CASE A/B) → **adopt**: when a bridge/weld candidate sits near an existing junction, grow only if the contact is a genuine crossing (surface-distance < 1e-9·scale), cap growth at 30% of the shortest incident edge, and fence a junction after its first evidence-driven growth — prevents the asymmetric span-crossing over-merges that broke base cut 35→43.

11. **Existing-edge rescue instead of duplicate section edges** (`IsExistingPaveBlock` with `tolR3D + fuzz` search + `UpdateEdgeTolerance(nEOut, max(measured, curveTol))`, PaveFiller_6 L920–985)
    → anchor: `split_with` whole-segment runs keyed by `seg_id` + zero-span collapse → **adopt**: when a lifted run coincides with an operand trim edge within `tol_edge + tol_chain + fuzz`, do not emit the run — grow the trim edge's `tol` to the measured deviation and alias the key (turns the SEGLOST/one-sided whole-segment loss class into a tolerance-recorded reuse instead of a dropped run).

12. **Non-destructive copy-on-write + map-to-avoid** (PaveFiller_10 `UpdateVertex` old-vertex branch; `PostTreat` fence)
    → anchor: our pipeline copies operand data into the BOP2 pool before splitting → **already-equivalent** (inputs are never mutated); no action beyond keeping audit-pass growth confined to result-owned records.

13. **Tangential tolerance as a separate channel** (`IntTools_Curve::myTangentialTolerance`; plane/plane `sqrt(aDt² + TolF1²)`; default `max(tolF1, tolF2)`)
    → anchor: `build_section_scaffold` tangential/grazing chains (newton_cc stall class, cone×cone tangent circle) → **new build (small)**: per-chain `tang_tol = max(tolF1, tolF2)` default, `ComputeIntRange` formula when the local crossing angle α is available from correct7 jacobians (`tolA·tan(π/2−α) + tolB/sin(α)`); use `max(tol_chain, tang_tol)` as the working radius for pave snapping on grazing chains — directly widens exactly where marching is least certain, instead of globally.
