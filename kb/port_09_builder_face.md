# port_09_builder_face — FACE SPLITTING: trimmed face + section edges → new faces

Port specification for the subsystem that turns ONE curved trimmed face plus a set of section
edges into the correct set of NEW FACES, with edge identity preserved by construction.

**OCCT ground truth** (read 2026-07-26, tree `/home/petras/code/code_cpp/OCCT`, 8.0.1.dev):

| unit | file |
|---|---|
| `BOPAlgo_BuilderFace` | `src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderFace.cxx/.hxx` |
| `BOPAlgo_BuilderArea` | `src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderArea.hxx` |
| `BOPAlgo_WireEdgeSet` | `src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_WireEdgeSet.hxx/.lxx` |
| `BOPAlgo_WireSplitter` | `src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_WireSplitter.cxx/.hxx/.lxx` |
| `BOPAlgo_WireSplitter` core | `src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_WireSplitter_1.cxx` |
| `BOPAlgo_EdgeInfo` | declared inside `BOPAlgo_WireSplitter.lxx:22-69` |
| input assembly | `src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder_2.cxx` |
| `BOPTools_AlgoTools` | `src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools.cxx` |
| `BOPTools_AlgoTools::MakeSplitEdge` | `src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools_2.cxx:138-183` |
| seam pcurve pairing | `src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools3D.cxx:58-234` |
| hole/inside classification | `src/ModelingAlgorithms/TKBO/IntTools/IntTools_FClass2d.cxx` |
| polygon classifier | `src/FoundationClasses/TKMath/CSLib/CSLib_Class2d.cxx` |
| metric conversion | `src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_Surface.cxx:1818-1896` |
| pcurve metric | `src/ModelingData/TKG2d/Geom2dAdaptor/Geom2dAdaptor_Curve.cxx:1186-1219` |
| constants | `src/FoundationClasses/TKernel/Precision/Precision.hxx` |

**Our code** (read-only): `src/brep.cpp`, `src/nurbssurface_trimmed.cpp`, `src/brep.h`.

Everything below that makes an algorithmic claim carries `file:line`. Where OCCT gives up or
falls back, it is marked **[OCCT FALLBACK]** or **[OCCT GIVES UP]**.

---

## 1. WHAT THIS SUBSYSTEM MUST GUARANTEE

These are testable statements about the output of `split_face(face, edge_set) -> vector<Face>`.
Each is checkable with no reference kernel.

### G1 — ZERO MINTING (the identity law)

> The face splitter creates **no vertex and no edge**. Every edge referenced by every output
> face is an index that was present in the input edge set. Every vertex is an index that was
> present on those edges.

Testable: `minted_edges == 0 && minted_vertices == 0`, asserted inside the splitter as a
post-condition, and re-checkable from outside as
`set(edge_idx of all output trims) ⊆ set(input edge indices)`.

This is what makes welding/sewing/coordinate matching structurally unnecessary. It is violated
today: `brep.cpp:4195` `result.add_edge(...)`, `brep.cpp:3757` `result.add_vertex(...)` both run
inside the per-face lift.

### G2 — HALF-EDGE CONSERVATION

> Let the input edge set list edge `e` with multiplicity `m(e)` (1 for a boundary edge in its
> loop orientation, 2 — FORWARD and REVERSED — for a section edge, an IN edge, an INTERNAL
> edge, or a seam edge). Then across all output wires, `e` is traversed exactly `m(e)` times,
> each traversal in a distinct orientation, unless `e` was pruned into `myShapesToAvoid`.

Corollary and the direct test for our defect: **a section edge appears in exactly two output
faces, and it is the same index in both.** OCCT: multiplicity set at
`BOPAlgo_Builder_2.cxx:469-494`; consumption enforced by the `Passed` flag,
`BOPAlgo_WireSplitter.lxx:39-42` and `BOPAlgo_WireSplitter_1.cxx:405,543`.

### G3 — PARTITION

> `Σ area3d(output faces) == area3d(input face)` to lift tolerance, and the output faces have
> pairwise-disjoint interiors.

Checkable without an oracle by UV quadrature over each output face's wires against the input
face's wires.

### G4 — ORIENTATION INHERITANCE, NOT DERIVATION

> No output face's orientation is computed from geometry. All areas are built FORWARD relative
> to the surface; the caller re-applies the source face's orientation.

OCCT: `BOPAlgo_BuilderFace.cxx:79-84` (`SetFace` stores `myOrientation`, forces the working face
FORWARD), all areas built by `aBB.MakeFace(aFace, aS, aLoc, aTol)`
(`BOPAlgo_BuilderFace.cxx:438,566`) which is FORWARD, and `BOPAlgo_Builder_2.cxx:535-552`
re-applies REVERSED iff `anOriF == TopAbs_REVERSED`.

### G5 — WIRE ROLE FROM SIGNED UV AREA ONLY

> Outer vs hole is decided by the sign of the signed area of the wire's UV polygon, on the same
> face's own parameterisation. Never by 3D, never by bounding box, never by edge count.

OCCT: `IntTools_FClass2d.cxx:454-456` (`Poly::PolygonProperties`), `:548-565` (`aS > 0` ⇒
`myIsHole = false`; `aS < 0` ⇒ hole; `|aS| < Precision::SquareConfusion()` ⇒ **bad wire**,
`TabOrien = -1`). `Poly::PolygonProperties` documents positive = counter-clockwise
(`Poly.hxx:156-158`).

### G6 — ANGULAR ORDER IS COMPUTED IN THE FACE'S OWN UV PLANE

> The sorting key at a vertex where >2 edges meet is `atan2` of the **pcurve tangent** in the
> face's `(u,v)` plane. Not the chord to the next arrangement node. Not a 3D direction. Not a
> projection into any global plane.

OCCT: `BOPAlgo_WireSplitter_1.cxx:768-840` (`Angle2D`), `:844-855` (`Angle`). See §2.6 for
exactly why UV is not only acceptable but required, and where the surface metric does enter.

### G7 — TOLERANCES ENTER AS 3D DISTANCES CONVERTED THROUGH THE SURFACE METRIC

> No constant in this subsystem is a fraction of the UV domain size. Every parametric epsilon is
> `metric_convert(3d_tolerance)`.

OCCT: `BOPAlgo_WireSplitter_1.cxx:859-901` (`Tolerance2D`/`UTolerance2D`/`VTolerance2D` built
from `BRep_Tool::Tolerance(V)` through `BRepAdaptor_Surface::UResolution/VResolution`).
`GeomAdaptor_Surface.cxx:1818-1896` shows those are per-surface-type closed forms
(`2·asin(R3d/2R)` for sphere/cylinder/torus U). Violated today at 6 sites — §4.2.

### G8 — DEGENERACY IS TYPED

> Degenerate edges, seam edges, poles, and dangling sections have named handling paths and
> named outcomes. No branch depends on a near-zero surviving a comparison by luck.

OCCT sites: degenerate edge never pruned as a valence-1 dangler
(`BOPAlgo_BuilderFace.cxx:200-203`); a wire may not consist of degenerate edges only
(`BOPAlgo_WireSplitter_1.cxx:437-445`); an infinite vertex parameter yields angle 0
(`:780-784`); a seam edge carries **two** pcurves on the same face
(`BOPTools_AlgoTools3D.cxx:200-231`, `BB.UpdateEdge(aSp, aC1, aC2, aF, aTol)`).

### G9 — EVERY INPUT EDGE HAS A DOCUMENTED FATE

> Each input edge ends in exactly one of: (a) traversed into a wire; (b) `shapes_to_avoid` and
> later added as an INTERNAL wire to a face that contains it; (c) `shapes_to_avoid` and
> unclassifiable → reported as a warning with the offending compound. Nothing is silently
> dropped.

OCCT: (a) `BOPAlgo_BuilderFace.cxx:277-283`; (b) `:618-741`; (c) `:743-777`
(`BOPAlgo_AlertFaceBuilderUnusedEdges`).

### G10 — DETERMINISM

> Two runs on the same input produce the same faces in the same order, and the choice at every
> angular tie is fixed by input list order, not by hash order or floating-point noise.

OCCT's tie rule is a strict-improvement comparison with a one-ulp margin,
`BOPAlgo_WireSplitter_1.cxx:380` (`eps = Epsilon(1.)`) and `:595`
(`if (anAngle < aMinAngle - eps)`), so the FIRST candidate in list order wins a tie.

---

## 2. OCCT'S ALGORITHM, IN FULL

### 2.0 The five-stage skeleton

`BOPAlgo_BuilderFace::Perform` (`BOPAlgo_BuilderFace.cxx:117-148`), progress weights in brackets:

```
CheckData()                    // :102-113  null face -> AlertNullInputShapes; lazily make context
PerformShapesToAvoid()  [ 1]   // :152-235  prune danglers + doubled-hanging edges, to fixpoint
PerformLoops()          [10]   // :239-383  WireSplitter -> myLoops; leftovers -> myLoopsInternal
PerformAreas()          [80]   // :387-614  wires -> faces; hole classification; hole attachment
PerformInternalShapes() [ 9]   // :618-778  avoided edges -> INTERNAL wires inside the right face
```

State lives in `BOPAlgo_BuilderArea` (`BOPAlgo_BuilderArea.hxx:78-84`):

```
myContext            // IntTools_Context: caches surface adaptors + FClass2d per face
myShapes             // INPUT: the oriented edge list
myLoops              // wires that bound areas
myLoopsInternal      // wires made of avoided edges
myAreas              // OUTPUT faces
myShapesToAvoid      // IndexedMap of edges excluded from the wire walk
myAvoidInternalShapes// user flag; when true PerformInternalShapes is a no-op (:620-625)
```

`myFace` is the source face forced FORWARD; `myOrientation` remembers the original
(`:79-84`).

### 2.1 How the input edge set is assembled (`BOPAlgo_Builder_2.cxx:233-507`)

This is the part that creates edge identity. The list `aLE` is built **from shared entities in
the DS arena** — every element is a `TopoDS_Shape` handle onto a `TShape` that already exists and
is referenced by other faces too. Nothing is copied and nothing is re-created.

Skip conditions in order (`:260-296`):
1. not a FACE;
2. `!myDS->HasFaceInfo(i)`;
3. `!aNbPBIn && !aNbPBOn && !aNbPBSc && !aNbAV` — no IN pave blocks, no ON pave blocks, no
   section pave blocks, no alone vertices ⇒ this face is untouched, no image at all.

Draft fast path (`:298-351`), entered only when `!aNbPBIn && !aNbPBSc`: if there are also no
alone vertices, no wire of the face is in `myImages`, and no wire's **first** edge is INTERNAL
(`:317-334` — note it inspects only the first edge of each wire), the face is skipped entirely.
Otherwise `BuildDraftFace` (`:1052-1189`) rebuilds the wires from edge images without any
splitting, and returns a **null face** — forcing the full BuilderFace path — if it finds an
INTERNAL edge (`:1104-1110`), a multi-connected vertex (`:1122-1125,1143-1146`), or evidence of
edge unification (`:1128-1131,1149-1152`).

Then the four contributions to `aLE`:

**(1.1) Bounding edges** — `:363-465`. For each edge of the FORWARD-oriented face:
- no image (`!myImages.IsBound(aE)`), orientation INTERNAL ⇒ append **twice**, FORWARD and
  REVERSED (`:371-378`); otherwise append **once** with its loop orientation (`:381`).
- has images: for each split `aSp`:
  - degenerate original ⇒ `aSp.Orientation(anOriE)`, append once (`:413-418`);
  - original was INTERNAL ⇒ append twice, F and R (`:420-427`);
  - original was a **seam** (`bIsClosed`, computed at `:387-404` as
    `GeomLib::IsClosed(surface, BRep_Tool::Tolerance(aE), isU, isV)` **and**
    `BRep_Tool::IsClosed(aE, aF)` **and** `BOPTools_AlgoTools2D::IsEdgeIsoline` agreeing on the
    axis) ⇒ dedupe by fence, ensure the split also carries **two** pcurves via
    `DoSplitSEAMOnFace` (`:435-446`), then append twice, F and R (`:449-452`);
  - otherwise `aSp.Orientation(anOriE)` and reverse if
    `BOPTools_AlgoTools::IsSplitToReverseWithWarn(aSp, aE, ...)` (`:457-462`; the tangent-dot
    test is `BOPTools_AlgoTools.cxx:1440-1531`), append once.

**(1.2) IN edges** — `:469-480`. Each `PaveBlocksIn` block's edge appended **twice**, F and R.

**(1.3) Section edges** — `:484-494`. Each `PaveBlocksSc` block's edge appended **twice**, F and
R. These are the FF section curves already materialised as shared edges by PostTreatFF.

**(1.4) pcurves** — `:496-500`: `BRepLib::BuildPCurveForEdgesOnPlane(aLE, aFF)` when the filler
is not in NonDestructive mode. In general, a pcurve for an edge on a face is attached **to the
edge**, not to a per-face copy: `BOPTools_AlgoTools2D::BuildPCurveForEdgeOnFace`
(`BOPTools_AlgoTools2D.cxx:48-70`) ends in `aBB.UpdateEdge(aE, aC2D, aF, aTolFact)`.

Then `aBF.SetFace(aF); aBF.SetShapes(aLE);` (`:502-506`) and the split faces are collected back
into `myImages` (`:527-552`).

> **This is the whole answer to "how does the same edge entity end up in both adjacent faces".**
> A section edge is one `TShape`. It is placed into the input list of face `A` twice and into the
> input list of face `B` twice. Each list entry is an *orientation view* of the same `TShape`.
> Wire construction (`BRep_Builder::Add(wire, edge)`) stores the handle. Face construction stores
> the wire. So `TopExp::MapShapesAndAncestors(result, VERTEX→EDGE / EDGE→FACE)` with
> `TopTools_ShapeMapHasher` — which compares by `IsSame`, i.e. TShape + Location, orientation
> **ignored** (`TopTools_ShapeMapHasher.hxx:30-38`) — sees ONE edge with TWO face ancestors.
> There is no coordinate comparison anywhere in this chain.

The per-face pcurves for that one edge live on the edge as a list of curve representations; the
face's surface selects which one. A seam edge holds two pcurves for the **same** surface and the
edge's orientation selects between them (`BOPTools_AlgoTools3D.cxx:200-231`).

### 2.2 `PerformShapesToAvoid` — pruning, to a fixpoint (`BOPAlgo_BuilderFace.cxx:152-235`)

```
loop forever:
  bFound = false
  aMVE.Clear()
  for each edge aE in myShapes with aE not in myShapesToAvoid:
      TopExp::MapShapesAndAncestors(aE, VERTEX, EDGE, aMVE)     // :174-182
  for each vertex aV in aMVE:                                    // :186-228
      aLE = incident edge records at aV
      if aLE.Extent() == 0: continue
      aE1 = aLE.First()
      if aLE.Extent() == 1:
          if BRep_Tool::Degenerated(aE1): continue                // :200-203  degenerate never pruned
          if aV.Orientation() == INTERNAL: continue               // :204-207
          bFound = true; myShapesToAvoid.Add(aE1)                 // dangling prune
      else if aLE.Extent() == 2:
          aE2 = aLE.Last()
          if aE2.IsSame(aE1):                                     // both records are the SAME TShape
              TopExp::Vertices(aE1, aV1x, aV2x)
              if aV1x.IsSame(aV2x): continue                      // closed edge, legitimate
              bFound = true; add aE1 and aE2 to myShapesToAvoid   // :223-226
  if !bFound: break
```

Semantics of the two branches:
- **valence 1** = a chain end that leads nowhere → the whole chain unravels over iterations.
- **valence 2, both records the same TShape** = a section edge listed F+R whose far end is a free
  end: both orientations dead-end at this vertex → prune both. This is OCCT's dangling-section
  rule, and it operates on the **oriented-multiplicity** graph, which is why it does not eat a
  section that genuinely reaches the boundary.

Note the incidence map is rebuilt from scratch every iteration (`:173-182`), so removal cascades.

### 2.3 `PerformLoops` (`BOPAlgo_BuilderFace.cxx:239-383`)

```
aWES.SetFace(myFace)
for each aE in myShapes not in myShapesToAvoid:  aWES.AddStartElement(aE)   // :258-266
aWSp.SetWES(aWES); aWSp.SetContext(myContext); aWSp.Perform()               // :268-275
myLoops = aWES.Shapes()                                                     // :277-283
```

**Post-treatment** (`:284-321`) — mandatory:
```
aMEP = { every edge appearing in any wire of myLoops }        // :288-298
aMEP += myShapesToAvoid                                        // :305-310
for each aE in myShapes not in aMEP: myShapesToAvoid.Add(aE)   // :313-321
```
i.e. an edge the walk never consumed is reclassified as internal material.

**Internal wires** (`:327-382`): the avoided edges are flooded into connexity blocks by shared
vertex and each block wrapped in one `TopoDS_Wire`. The flood is a breadth-first walk over
`aVEMap` guarded by `aMAdded`; the wire is **not** ordered and **not** oriented — `aBB.Add(aW,
aEx)` in discovery order; `aW.Closed(BRep_Tool::IsClosed(aW))` at the end (`:380`). `bFlag`
short-circuits once every avoided edge has been consumed (`:372-375`).

### 2.4 `BOPAlgo_WireSplitter` — connexity blocks (`BOPAlgo_WireSplitter.cxx:90-219`)

```
MakeConnexityBlocks(myWES->StartElements(), VERTEX, EDGE, myLCB)   // :107-110
MakeWires()                                                         // :116
```

`BOPTools_AlgoTools::MakeConnexityBlocks(list, VERTEX, EDGE, LCB)`
(`BOPTools_AlgoTools.cxx:187-256`):
- a shape that appears more than once in the input list goes into `aMNRegular` and is added to
  the compound only once (`:199-211`);
- flood by shared vertex (`:105-154`) gives raw blocks;
- for each block a `BOPTools_ConnexityBlock` is filled; a member in `aMNRegular` is pushed
  **twice** (FORWARD, REVERSED) and sets `bRegular = false` (`:231-238`);
- otherwise the member is pushed once and, while still regular, the block stays regular only if
  **every** vertex of that member has exactly 2 incident edges in the connection map
  (`:242-249`).

`MakeWires` (`BOPAlgo_WireSplitter.cxx:163-219`):
- regular block ⇒ `MakeWire(block.Shapes(), aW)` directly, no angular work (`:185-191`);
- irregular block ⇒ queued into a vector and processed by `SplitBlock`, in parallel
  (`:194-206`), then all resulting loops harvested (`:208-218`).

`MakeWire` (`BOPAlgo_WireSplitter.lxx:78-89`) is a bare `MakeWire` + `Add` of every edge, then
`aWire.Closed(BRep_Tool::IsClosed(aWire))`. It preserves the incoming orientation of each edge.

### 2.5 `SplitBlock` — the wire walk (`BOPAlgo_WireSplitter_1.cxx:112-354`)

**Step 1: build `mySmartMap : vertex → list<EdgeInfo>`** (`:136-195`).

```
for each edge aE in the block:
    if !BOPTools_AlgoTools2D::HasCurveOnSurface(aE, myFace): continue        // :141-144
    bIsClosed = BRep_Tool::Degenerated(aE) || BRep_Tool::IsClosed(aE, myFace)
    if (!aMS.Add(aE) && !bIsClosed) aMS.Remove(aE)                           // :148-151
    for i, aV in TopoDS_Iterator(aE):        // cumOri=true: composes edge orientation
        aLEI = mySmartMap[aV]                // key hashed by IsSame -> orientation-blind
        aEI.SetEdge(aE)
        aEI.SetInFlag(aV.Orientation() == TopAbs_REVERSED)                   // :169-171
        aLEI.Append(aEI)
        if i == 0: aV1 = aV  else: bIsClosed |= aV1.IsSame(aV)
        aVertMap[aV] = bIsClosed (sticky-true)                               // :183-193
```

Three facts a port must not lose:
- `HasCurveOnSurface` (`BOPTools_AlgoTools2D.cxx:188-205`) additionally rejects an edge whose 3D
  range is shorter than `Precision::PConfusion()` = 1e-9 (`:196-199`).
- `TopoDS_Iterator` with `cumOri = true` (default, `TopoDS_Iterator.hxx:50-65`) composes the
  edge's orientation into its vertices. Therefore a FORWARD edge yields
  `(Vstart FORWARD, Vend REVERSED)` ⇒ OUT at start, IN at end; a REVERSED edge yields
  `(Vstart REVERSED, Vend FORWARD)` ⇒ IN at start, OUT at end. This is precisely the half-edge
  model, obtained without a half-edge structure.
- `aMS` is an `NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher>` (IsSame). An edge given
  twice is added then removed, so `aMS.Contains(e)` is false for it. `SetIsInside(!aMS.Contains(aE))`
  (`:309`) therefore means **`IsInside == "this edge was supplied with both orientations"`**, i.e.
  it is a section / IN / INTERNAL edge, not a face boundary edge.

**Step 2: two "nothing to do" tests.**
- (a) every vertex has exactly one IN and one OUT (`:199-223`);
- (b) no edge appears twice in the block (`:229-284`; the second `for` gives `bFlag=false` for
  any bucket of size 2 whose members `IsSame`, or size > 2).
If both hold, the block is emitted as a single wire and `SplitBlock` returns (`:285-295`).

**Step 3: angles** (`:298-318`).
```
aBAS = theContext->SurfaceAdaptor(myFace)         // cached BRepAdaptor_Surface
for each vertex aV, for each EdgeInfo aEI:
    aEI.SetIsInside(!aMS.Contains(aEI.Edge()))
    aVV = aV with orientation REVERSED if aEI.IsIn() else FORWARD
    aEI.SetAngle(Angle2D(aVV, aEI.Edge(), myFace, aBAS, aEI.IsIn(), theContext))
```

**Step 4: refinement** — `RefineAngles(myFace, mySmartMap, theContext)` (`:323`), see §2.7.

**Step 5: launch paths** (`:331-353`). For every vertex in `mySmartMap` insertion order, for every
EdgeInfo in list order, if it is OUT and not yet Passed, run `Path(...)`. Insertion order is the
edge order of the input list — this is the determinism anchor (G10).

### 2.6 `Angle2D` — THE SORTING KEY (`BOPAlgo_WireSplitter_1.cxx:768-840`)

```cpp
aTV = BRep_Tool::Parameter(aV, anEdge, myFace);        // :780
if (Precision::IsInfinite(aTV)) return 0.;             // :781-784   [OCCT FALLBACK]

BOPTools_AlgoTools2D::CurveOnSurface(anEdge, myFace, aC2D, aFirst, aLast, aToler, ctx);  // :786
double tol2d = 2. * Tolerance2D(aV, aGAS);             // :787
Geom2dAdaptor_Curve aGAC2D(aC2D);
double dt = max(aGAC2D.Resolution(tol2d), Precision::PConfusion());     // :792  (PConfusion=1e-9)

if (aGAC2D.GetType() != GeomAbs_Line) {                                 // :794-808
    GeomLProp_CLProps2d LProp(aC2D, aTV, 2, Precision::PConfusion());
    if (LProp.IsTangentDefined()) {
        double R = LProp.Curvature();
        if (R > Precision::PConfusion()) {
            R = 1./R;
            double cosphi = R/(R + tol2d);
            dt = max(dt, acos(cosphi));      // do not sample inside the curve's own tolerance band
        }
    }
}
double aTX = 0.05 * (aLast - aFirst);                                   // :810
if (aTX < 5.e-5) aTX = min(5.e-5, (aLast - aFirst)/2.);                 // :811-814
if (dt > aTX) dt = aTX;                                                 // :815-820

aTV1 = (fabs(aTV-aFirst) < fabs(aTV-aLast)) ? aTV + dt : aTV - dt;      // :822-829
aGAC2D.D0(aTV1, aPV1); aGAC2D.D0(aTV, aPV);
gp_Vec2d aV2D = bIsIN ? gp_Vec2d(aPV1, aPV) : gp_Vec2d(aPV, aPV1);      // :834
return Angle(gp_Dir2d(aV2D));                                            // :836-837
```

`Angle` (`:844-855`) is `atan2` against `+X`, folded into `[0, 2π)`.

`Tolerance2D` (`:859-881`):
```cpp
aTolV3D = BRep_Tool::Tolerance(aV);
anUr = aGAS.UResolution(aTolV3D);  aVr = aGAS.VResolution(aTolV3D);
aTol2D = max(anUr, aVr);
if (aTol2D < aTolV3D) aTol2D = aTolV3D;                 // floor at the 3D value
if (aGAS.GetType() == GeomAbs_BSplineSurface) aTol2D *= 1.1;
```
`UTolerance2D`/`VTolerance2D` (`:885-901`) return the *unmixed* per-axis values, used only for
the anisotropic closure test in `Path`.

`GeomAdaptor_Surface::UResolution` (`GeomAdaptor_Surface.cxx:1818-1896`) closed forms:
`Torus: Res = R3d/(2(Rmaj+Rmin))`, `Sphere: R3d/(2R)`, `Cylinder: R3d/(2R)`, cone via its VIso
circle radius, then `return Res<=1 ? 2*asin(Res) : 2π` (`:1890-1895`); everything else, including
BSpline surfaces, falls to `Precision::Parametric(R3d) = R3d * 0.01` (`Precision.hxx:328`)
**[OCCT FALLBACK — a crude constant, not a metric]**.

`Geom2dAdaptor_Curve::Resolution` (`Geom2dAdaptor_Curve.cxx:1186-1219`): `Line: Ruv`;
`Circle: 2*asin(Ruv/2R)` (or `2π`); `Ellipse: Ruv/MajorRadius`; Bezier/BSpline delegate to the
curve's own `Resolution`; default `Precision::Parametric(Ruv)`.

#### Why raw UV — and where the surface metric actually matters

The angle **value** is computed in raw parameter space with no first-fundamental-form correction.
That is not an oversight, and a port must not "fix" it:

- At a regular point the surface differential `J = [S_u | S_v]` is a rank-2 linear map from
  `R²` to the tangent plane. A nonsingular linear map preserves the **cyclic order** of
  directions around the origin, and preserves the sense of rotation exactly when `det > 0` — which
  holds by definition for the frame `(S_u, S_v)`, the same frame whose cross product defines the
  FORWARD face normal. Therefore *leftmost turn in UV* ≡ *leftmost turn on the surface viewed from
  the FORWARD side*. The metric distorts the angle magnitudes; it cannot reorder them.
- Consequently sorting in 3D (or in a projection of 3D) is **wrong**, because the projection is
  not the surface's own frame and can reverse the sense on a curved patch.

The metric enters at exactly three places, all of them *distances*, never the angle:

1. **Step size.** `tol2d = 2·Tolerance2D(V)` is the vertex's 3D tolerance pushed into UV via
   `UResolution/VResolution`; `dt` is then that UV distance pushed into curve parameter via
   `Geom2dAdaptor_Curve::Resolution`. The sample point must lie *outside* the vertex's tolerance
   ball, otherwise the direction is noise. On a cylinder of radius 100 the U step for a 1e-7
   tolerance is 1e-9 rad while the V step is 1e-7 — a single isotropic UV epsilon is wrong by the
   radius factor.
2. **Closure test.** `aTol2D = 2·Tolerance2D(aVb, aGAS)` and the per-axis `aTolU/aTolV`
   (`:421-422,459-462`) decide whether two UV points are the same physical vertex.
3. **Candidate filter at a closed vertex.** `Coord2dVf(aE, myFace)` vs `aPb` within `aTol2D`
   (`:571-582`) — the seam disambiguation.

Where the argument breaks: at a **pole** `J` is singular. OCCT covers this by never letting a
degenerate edge be pruned as a dangler (`BOPAlgo_BuilderFace.cxx:200-203`), never building a wire
out of degenerate edges alone (`BOPAlgo_WireSplitter_1.cxx:437-445`), and returning angle 0 for an
infinite parameter (`:781-784`). At a **seam** one 3D vertex has two UV images; that is handled by
`aVertMap` + the 2D closure test, not by the angle.

### 2.7 `RefineAngles` — convergent pcurves at a node (`BOPAlgo_WireSplitter_1.cxx:905-1125`)

Per vertex (`:925-1029`):
```
count boundary records (IsInside == false) and interior records
aA1 = angle of the boundary record that is OUT
aA2 = angle of the boundary record that is IN
if (#boundary != 2) return                                   // :965-968
aDelta = ClockWiseAngle(aA2, aA1)                            // :970  the material wedge
for each record that is interior AND OUT:
    aDA = ClockWiseAngle(aA2, aEI.Angle())
    if (aDA < aDelta) continue                               // :985-989  already inside the wedge
    if (RefineAngle2D(...)) bind refined angle
    else if (#interior == 2) aA = (aA <= aA1) ? aA1 + Precision::Angular()
                                              : aA2 - Precision::Angular();   // :996-1000
if nothing bound: return
apply: interior IN records get (refined + π)                 // :1009-1028
```
`Precision::Angular() = 1e-12` (`Precision.hxx:123`).

`RefineAngle2D` (`:1033-1125`) — constants `aCf = 0.01`, `aTolInt = 1e-10`,
`MaxDT = 0.3*(aT2-aT1)` (`:1053-1065`):
for `i` in `{0,1}`, build the 2D ray from the vertex along `aA1` (i=0) or `aA2 + π` (i=1),
intersect it with the pcurve (`Geom2dInt_GInter`), take the intersection with the greatest
parameter on the ray subject to `|t1 - tV| < MaxDT` (`:1089-1100`), step `aCf` of the way from
there toward the far end of the pcurve (`:1104-1111`), and recompute the angle from the vertex to
that point. Accept if the new angle now falls inside the wedge (`:1116-1121`); otherwise
**[OCCT GIVES UP]** and returns false.

Purpose in one sentence: when a section pcurve leaves the vertex *tangentially* to a boundary
pcurve, the differential angle is meaningless, so OCCT replaces it with a **secant** angle taken
far enough along the curve to be decidable, and if even that fails, with a hard nudge just inside
the wedge (only when exactly two interior records exist).

### 2.8 `Path` — loop extraction with backtracking (`BOPAlgo_WireSplitter_1.cxx:358-617`)

Stacks, all parallel: `aLS` (edges), `aVertVa` (the vertex the edge was entered from),
`aCoordVa` (that vertex's UV on that edge), `anInfoSeq` (the EdgeInfo pointers).
`eps = Epsilon(1.)` = one ulp of 1.0 ≈ 2.22e-16 (`:380`).

```
loop:
  # A. Do not escape through the edge you entered by
  if aLS.Length() == 1 and aLS(1).IsSame(aEOuta): return                   # :396-403

  # B. Push
  anEdgeInfo->SetPassed(true); aLS.Append(aEOuta); aVertVa.Append(aVa);
  anInfoSeq.Append(anEdgeInfo)                                             # :405-408
  pVa = aVa oriented FORWARD
  aPa = Coord2d(pVa, aEOuta, myFace); aCoordVa.Append(aPa)                 # :410-413
  aVb = GetNextVertex(pVa, aEOuta)                                         # :415
  aPb = Coord2d(aVb, aEOuta, myFace)                                       # :417
  aLEInfo = mySmartMap[aVb]
  aTol2D = 2*Tolerance2D(aVb, aGAS);  aTol2D2 = aTol2D^2                   # :421-422
  bIsClosed = aVertMap[aVb]                                                # :424

  # C. Closure scan, from the top of the stack downward
  aBuf = {}; bHasEdge = false
  for i = aLS.Length() down to 1:                                          # :430-523
      aBuf.Append(aLS(i))
      if !bHasEdge:
          bHasEdge = !Degenerated(aLS(i)); if !bHasEdge: continue          # :438-445
      anIsSameV = aVertVa(i).IsSame(aVb)
      anIsSameV2d = anIsSameV
      if anIsSameV and bIsClosed:                                          # :449-467
          anIsSameV2d = aCoordVa(i).SquareDistance(aPb) < aTol2D2
          if anIsSameV2d:
              if |du| > 2*UTolerance2D(aVb) or |dv| > 2*VTolerance2D(aVb):
                  anIsSameV2d = false
      if anIsSameV and anIsSameV2d:
          iPriz = 1
          if aBuf.Extent()==2 and aBuf.First().IsSame(aBuf.Last()): iPriz = 0   # :474-480
          if iPriz: MakeWire(aBuf, aW); aCB.ChangeLoops().Append(aW)            # :481-486
          if i-1 < 1: clear all stacks; return                                  # :488-497
          truncate all stacks to i-1 entries                                    # :499-517
          aVb = aVertVa(i)   (as read BEFORE truncation)                        # :503
          aEOuta = aLS.Last(); anEdgeInfo = anInfoSeq.Last()
          break

  # D. Choose the next outgoing edge at aVb
  anAngleIn = AngleIn(aEOuta, aLEInfo)          # angle of the IN record for the arriving edge
  aMinAngle = 100.;  iCnt = NbWaysOut(aLEInfo)  # unpassed OUT records
  isBoundary = !anEdgeInfo->IsInside()
  aNbWaysInside = 0; pOnlyWayIn = null; pEdgeInfo = null
  for each anEI in aLEInfo with (OUT && !Passed):                          # :537-601
      if iCnt == 0: return                                                 # :551-555
      if iCnt == 1: pEdgeInfo = &anEI; break                               # :557-562
      if anEI.Edge().IsSame(aEOuta): anAngle = 2π                          # :564-567
      else:
          if bIsClosed and Coord2dVf(anEI.Edge(), myFace).SquareDistance(aPb) > aTol2D2:
              continue                                                     # :570-582
          anAngle = ClockWiseAngle(anAngleIn, anEI.Angle())                # :584-586
      if isBoundary and anEI.IsInside(): ++aNbWaysInside; pOnlyWayIn = &anEI   # :589-593
      if anAngle < aMinAngle - eps: aMinAngle = anAngle; pEdgeInfo = &anEI  # :595-599
  if aNbWaysInside == 1: pEdgeInfo = pOnlyWayIn                            # :602-605
  if !pEdgeInfo: return                                                     # :607-611
  aVa = aVb; aEOuta = pEdgeInfo->Edge(); anEdgeInfo = pEdgeInfo
```

`ClockWiseAngle(aIn, aOut)` (`:621-659`):
```
A1 = (aIn mod 2π) + π, folded into [0,2π)
A2 = aOut mod 2π
dA = A1 - A2
if dA <= 0.   dA += 2π
else if dA <= 1.e-14  dA = 2π          // :654-657  exact-tie promotion, hard constant 1e-14
return dA
```
`A1` is the **reversed incoming direction**; minimising `dA` selects the outgoing edge with the
smallest clockwise sweep from it, i.e. **the leftmost turn**, which traces regions with material
on the left ⇒ bounded wires positive-area, holes negative-area.

Helper semantics:
- `Coord2d(V, E, F)` = `BRep_Tool::Parameter(V, E, F)` evaluated on the pcurve (`:663-674`).
  `BRep_Tool::Parameter(V,E,S,L)` (`BRep_Tool.cxx:1532-1581`) matches by `IsSame` on the
  FORWARD-oriented edge, disambiguates a doubled vertex by orientation, and swaps `f`/`l` when the
  vertex is doubled **and** the edge is REVERSED (`:1553,1571-1581`).
- `Coord2dVf(E, F)` (`:678-697`) = the UV of the FORWARD-oriented vertex of `E`; returns
  `(99, 99)` if there is none — a sentinel that guarantees the `bIsClosed` filter rejects it.
- `GetNextVertex(V, E, V1)` (`:749-764`) returns the first sub-vertex that is **not `IsEqual`**
  (TShape + orientation + location) to `V`; falls back to `V` itself.
- `NbWaysOut` (`:701-720`), `AngleIn` (`:724-745`; returns 0 if no IN record is found — a silent
  default a port should assert on instead).

The `aNbWaysInside == 1` override (`:602-605`) is load-bearing: arriving on a **boundary** edge at
a vertex that has exactly one unpassed outgoing **interior** (section) edge forces the walk onto
the section, whatever the angle says. Without it a section that meets the boundary tangentially is
skipped and the face does not split.

Worked example, square face cut by a section from left-edge point `P` to right-edge point `Q`
(boundary already split by the paves at `P` and `Q`; edges CCW `B, R1, R2, T, L1, L2`; section `S`
listed as `S_F: P→Q` and `S_R: Q→P`):
- Path 1 from `B`: `B → R1 →` at `Q`, `isBoundary = true` and the only interior OUT is `S_R`
  ⇒ forced ⇒ `S_R: Q→P`; at `P` `isBoundary = false`, angular rule picks `L2`; at `BL` the
  closure scan matches stack entry 1 ⇒ wire `{B, R1, S_R, L2}`; stacks empty ⇒ return.
- Path 2 from `R2`: `R2 → T → L1 →` at `P` forced onto `S_F: P→Q`; at `Q` closure matches
  ⇒ wire `{R2, T, L1, S_F}`.
- `S_F` and `S_R` consumed once each; the same edge `TShape` sits in both wires (G2).

A closed section circle with a single vertex `V` also works: at `V` the two copies give 2 OUT and
2 IN records; each Path pushes one copy, `GetNextVertex` returns the same `V` (different
orientation), the closure scan matches at `i = 1` with `aBuf.Extent() == 1`, so `iPriz = 1` and a
one-edge wire is emitted; the two wires have opposite signed area.

### 2.9 `PerformAreas` — wires to faces, holes to owners (`BOPAlgo_BuilderFace.cxx:387-614`)

```
aS   = BRep_Tool::Surface(myFace, aLoc)
aTol = BRep_Tool::Tolerance(myFace)

if myLoops.IsEmpty():                                             // :401-414
    if myContext->IsInfiniteFace(myFace):                         // IntTools_Context.cxx:216-221
        emit a face with NO wires (NaturalRestriction copied)
    return                                                        // [OCCT GIVES UP] face vanishes

for each wire aWire in myLoops:                                   // :427-459
    aFace = MakeFace(aS, aLoc, aTol); Add(aFace, aWire)           // one wire per draft face
    bIsGrowth = IsGrowthWire(aWire, aMHE)                         // fast path, :441
    if !bIsGrowth: bIsGrowth = !myContext->FClass2d(aFace).IsHole()
    if bIsGrowth: aNewFaces.Append(aFace)
    else: aHoleFaces.Add(aFace); MapShapes(aWire, EDGE, aMHE)

if aHoleFaces.IsEmpty(): myAreas = aNewFaces; return              // :461-466

build Bnd_Box2d for every hole face via BRepTools::AddUVBounds -> BOPTools_Box2dTree   // :471-484
for each growth face aFace:                                       // :494-537
    select candidate holes whose UV box overlaps aFace's UV box
    for each candidate aHole:
        if !IsInside(aHole, aFace, ctx): continue
        if aHoleFaceMap already has aHole:
            if IsInside(aFace, previous_owner): previous_owner = aFace   // tighter owner wins
        else aHoleFaceMap[aHole] = aFace

invert to aFaceHolesMap                                            // :540-555

if some holes unassigned:                                          // :558-581
    if the ORIGINAL face's 3D Bnd_Box is open in any of 6 directions:
        create a bare face from the surface and give it all unassigned holes
    else: the unassigned holes are DROPPED                         // [OCCT GIVES UP]

for each growth face: add its hole wires; re-Init its FClass2d      // :585-613
myAreas = the growth faces
```

`IsGrowthWire` (`:898-913`): if the wire shares **any** edge with an already-recorded hole face,
it is a growth without classification. Order-dependent by construction — it only sees holes
discovered earlier in the `myLoops` iteration.

`IsInside(wire_or_edge, face, ctx)` (`:842-894`): map the face's edges; take the **first**
non-degenerate sub-edge of the wire that has a pcurve on the face; if the face already contains
that edge return false immediately (`:870-875`); otherwise classify the pcurve's **mid-parameter**
point with `IntTools_FClass2d::Perform` and return `state == IN`; then **break** — one sample
decides (`:886-891`).

`IntTools_FClass2d::Init` (`IntTools_FClass2d.cxx:77-621`), the hole test:
- walk each wire with `BRepTools_WireExplorer` (connected order), skip non-FORWARD/REVERSED
  orientations (`:152-155`), bail out entirely if any pcurve is null (`:157-161`);
- `degenerated` is set when the edge is degenerate, closed on the face, has a null vertex, **or**
  when 11 samples of the 3D curve all lie within `0.5*Precision::Confusion()` of the mid point
  (`:199-220`) — the "forgot to flag it degenerate" guard;
- sample count `nbs = Geom2dInt_Geom2dCurveTool::NbSamples(C)`, `×4` if `> 2` (`:229-233`);
  for `nbs == 2` the parameters are `{first, first + 0.0025*(last-first), last}` (`:254-260`);
- accumulate `SeqPnt2d`, track `Umin/Umax/Vmin/Vmax` and the chordal sags `FlecheU/FlecheV`
  (`:345-364`);
- `Poly::PolygonProperties(SeqPnt2d, aS, aPer)` (`:456`); if the sag exceeds
  `max(2|aS|/aPer, 1e-7)` the wire is **re-discretised** with `GCPnts_QuasiUniformDeflection` at
  a tighter deflection, looping while `aDefl > anExpThick && aDiscrDefl > 1e-7` (`:458-534`) —
  the self-intersection guard;
- verdict (`:548-565`): `|aS| < Precision::SquareConfusion()` ⇒ `TabOrien = -1` (bad wire, forces
  the exact classifier); `aS > 0` ⇒ not a hole, `TabOrien = 1`; `aS < 0` ⇒ hole, `TabOrien = 0`;
- if the surface is cone/cylinder/torus/sphere/revolution, a periodic recentring window
  `U1 = Umin - (2π-(Umax-Umin))/2`, `U2 = U1 + 2π` is recorded (`:588-604`), and likewise in V for
  a torus (`:605-615`).

`IntTools_FClass2d::Perform` (`:637-804`): polygon fast path via `CSLib_Class2d::SiDans`
(`:684-724`); any `Uncertain` result or a bad wire drops to the exact
`BRepClass_FClassifier` with tolerance `min(URes, VRes)` when the point is inside both parametric
ranges (or the single relevant resolution otherwise) (`:726-756`); with `RecadreOnPeriodic` the
whole test is retried on period-shifted copies until IN/ON or the range is exhausted (`:758-802`).

`CSLib_Class2d` normalises the polygon to `[0,1]²` and treats `myTolU * (myUMax-myUMin)` as the
boundary band (`CSLib_Class2d.cxx:152-189`), re-testing the four corners of the tolerance box and
returning `Uncertain` if they disagree (`:174-186`).

> Note: `BOPTools_AlgoTools::IsHole` (`BOPTools_AlgoTools.cxx:1535-1604`) also exists, uses the
> trapezoid form `Σ(y0+y1)(x1-x0) > 0 ⇒ hole` (opposite sign convention), and is **not** used by
> `BuilderFace`. Do not port it here.

### 2.10 `PerformInternalShapes` (`BOPAlgo_BuilderFace.cxx:618-778`)

```
if myAvoidInternalShapes: return                       // :620-625
if myLoopsInternal.IsEmpty(): return                   // :627-631
index every edge of myLoopsInternal, box it in UV (BRepTools::AddUVBounds(myFace, aE, aBoxE))
  and insert into a BOPTools_Box2dTree                 // :643-666
for each area face aF:                                 // :675-741
    select edges whose UV box overlaps aF's UV box
    for each unclassified selected edge: if IsInside(aE, aF, ctx): collect + mark done
    MakeInternalWires(collected, aLSI); add each wire to aF
    if all edges done: return
leftovers -> MakeInternalWires -> compound{myFace, wires} -> AddWarning(
    BOPAlgo_AlertFaceBuilderUnusedEdges)               // :743-777   [OCCT GIVES UP, but reports]
```

`MakeInternalWires` (`:782-838`) floods by shared vertex and sets every edge's orientation to
`TopAbs_INTERNAL` before adding it to the wire (`:810,829`).

### 2.11 Table of every hard constant in the subsystem

| constant | value | site |
|---|---|---|
| progress split | 1 / 10 / 80 / 9 | `BuilderFace.cxx:129-147` |
| `Precision::Confusion()` | 1e-7 | `Precision.hxx:165` |
| `Precision::PConfusion()` | 1e-9 | `Precision.hxx:334` |
| `Precision::Angular()` | 1e-12 | `Precision.hxx:123` |
| `Precision::Parametric(P)` | `P * 0.01` | `Precision.hxx:328` |
| min edge 3D range | `PConfusion()` = 1e-9 | `AlgoTools2D.cxx:176-179,196-199` |
| angle step cap | `0.05 * (last-first)`, floor `min(5e-5, span/2)` | `WireSplitter_1.cxx:810-814` |
| tie-promotion in `ClockWiseAngle` | `1.e-14` | `WireSplitter_1.cxx:654` |
| walk tie margin | `Epsilon(1.)` ≈ 2.22e-16 | `WireSplitter_1.cxx:380,595` |
| `aMinAngle` sentinel | `100.` | `WireSplitter_1.cxx:530` |
| `Coord2dVf` sentinel | `(99., 99.)` | `WireSplitter_1.cxx:680-681` |
| BSpline surface tol2d inflation | `× 1.1` | `WireSplitter_1.cxx:875-878` |
| closure test radius | `2 × Tolerance2D` (and `2 × U/VTolerance2D` per axis) | `WireSplitter_1.cxx:421,459-460` |
| `RefineAngle2D` step fraction | `aCf = 0.01` | `WireSplitter_1.cxx:1054` |
| `RefineAngle2D` intersection tol | `aTolInt = 1.e-10` | `WireSplitter_1.cxx:1055` |
| `RefineAngle2D` param window | `MaxDT = 0.3*(aT2-aT1)` | `WireSplitter_1.cxx:1065` |
| refine nudge | `± Precision::Angular()` | `WireSplitter_1.cxx:998` |
| FClass2d degeneracy probe | 11 samples, radius `0.5*Confusion()` | `IntTools_FClass2d.cxx:206-219` |
| FClass2d `nbs == 2` interior param | `0.0025` of span | `IntTools_FClass2d.cxx:256-259` |
| FClass2d sample multiplier | `×4` when `nbs > 2` | `IntTools_FClass2d.cxx:230-233` |
| FClass2d re-discretisation floor | `1e-7` | `IntTools_FClass2d.cxx:458-462` |
| FClass2d bad-wire area threshold | `Precision::SquareConfusion()` = 1e-14 | `IntTools_FClass2d.cxx:548` |
| `mySmartMap` initial buckets | 100 | `WireSplitter_1.cxx:129` |

---

## 3. DATA STRUCTURES AND C++ DECLARATIONS FOR OUR PORT

New file pair `src/brep_builderface.h/.cpp`. It depends on the arena (`brep_bds` from
`kb/ARCHITECTURE_v2.md` §1) and on `BRep`'s existing tables (`brep.h:24-58`), and it **never
allocates into the arena**.

```cpp
#pragma once
#include "brep.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include <vector>
#include <cstdint>

// ---------------------------------------------------------------------------
// Input: an ORIENTATION VIEW of an arena edge. The face splitter is handed a
// list of these and may not create any others.
// ---------------------------------------------------------------------------
enum class EdgeSense : uint8_t { Forward = 0, Reversed = 1 };

struct BfInputEdge {
    int       edge = -1;          // index into the SHARED arena (BRep::m_topology_edges)
    EdgeSense sense = EdgeSense::Forward;
    int       pcurve = -1;        // index into BRep::m_curves_2d, valid on THIS face.
                                  // A seam edge supplies two entries with different pcurve
                                  // and opposite sense (OCCT DoSplitSEAMOnFace analogue).
    bool      degenerate = false; // 3D length below tol; pole row
    // provenance, for diagnostics only -- never for identity
    int       source = -1;        // original operand edge, or -1 for a section edge
};

// ---------------------------------------------------------------------------
// Per-(vertex, oriented-edge) record. Mirrors BOPAlgo_EdgeInfo
// (BOPAlgo_WireSplitter.lxx:22-69) plus the two flags OCCT derives implicitly.
// ---------------------------------------------------------------------------
struct BfEdgeInfo {
    int    input = -1;      // index into the BfInputEdge list
    bool   is_in = false;   // true: this record is the ARRIVING end at the vertex
    bool   is_inside = false;// true: the edge was supplied with BOTH senses (section/internal)
    bool   passed = false;  // consumed by a walk
    double angle = -1.0;    // Angle2D, radians in [0, 2pi)
    double t_at_vertex = 0.0;   // pcurve parameter at the vertex
    double t_other = 0.0;       // pcurve parameter at the far end (sign of the step)
    Point  uv_at_vertex;        // (u, v, 0)
};

struct BfVertexNode {
    int vertex = -1;                    // arena topology-vertex index
    std::vector<BfEdgeInfo> records;    // insertion order == input list order (determinism)
    bool closed = false;                // OCCT aVertMap: some incident edge closes here
    double tol3d = 0.0;                 // arena vertex tolerance, 3D
    double tol_u = 0.0, tol_v = 0.0;    // = UResolution(tol3d), VResolution(tol3d)
    double tol_uv = 0.0;                // = max(tol_u, tol_v), floored at tol3d
};

// ---------------------------------------------------------------------------
// A wire produced by the walk: an ordered list of oriented input-edge views.
// ---------------------------------------------------------------------------
struct BfWire {
    std::vector<int> inputs;   // indices into the BfInputEdge list, in traversal order
    double signed_area_uv = 0.0;
    bool   is_hole = false;
    bool   is_internal = false;
};

struct BfArea {
    BfWire              outer;
    std::vector<BfWire> holes;
    std::vector<BfWire> internals;   // INTERNAL-oriented wires added by stage 5
};

// ---------------------------------------------------------------------------
// Surface metric adapter -- the ONLY place a 3D tolerance becomes a UV epsilon.
// Analytic closed forms first (GeomAdaptor_Surface.cxx:1818-1896), numeric
// fallback via |dS/du|, |dS/dv| for a general NURBS patch.
// ---------------------------------------------------------------------------
struct BfMetric {
    const NurbsSurface* srf = nullptr;
    // exact for the recognised analytic types; numeric otherwise
    double u_resolution(double tol3d, double u, double v) const;
    double v_resolution(double tol3d, double u, double v) const;
    // pcurve parameter step whose UV image is >= tol_uv, curvature-capped
    double curve_resolution(const NurbsCurve& pc, double t, double tol_uv) const;
};

// ---------------------------------------------------------------------------
// Diagnostics: nothing is silently dropped (G9).
// ---------------------------------------------------------------------------
enum class BfAlert : uint8_t {
    NullFace, NoPCurveOnFace, MicroEdge, DanglingPruned, DoubledHangingPruned,
    EdgeNotConsumed, WireDegenerateArea, HoleUnassigned, InfiniteFaceNoLoops,
    AngleRefineFailed, WalkDeadEnd
};
struct BfReport {
    std::vector<std::pair<BfAlert,int>> events;   // (alert, input-edge or wire index)
    int minted_edges = 0;      // MUST stay 0
    int minted_vertices = 0;   // MUST stay 0
};

// ---------------------------------------------------------------------------
// The subsystem entry point.
//   `arena`   : the shared BRep holding vertices/edges/curves for BOTH operands
//   `surface` : the face's surface (unchanged; the splitter never re-parameterises)
//   `inputs`  : boundary edges (multiplicity 1) + section/IN/internal/seam (multiplicity 2)
//   `face_reversed` : inherited from the source face; NOT derived here (G4)
// Returns areas whose wires reference ONLY indices found in `inputs`.
// ---------------------------------------------------------------------------
struct BfOptions {
    double face_tol3d = 1e-7;      // model space
    bool   avoid_internal = false; // OCCT myAvoidInternalShapes
    int    max_walk_steps = 0;     // 0 => 4 * inputs.size(), a hard blow-up guard
};

std::vector<BfArea> bf_split_face(const BRep&                       arena,
                                  const NurbsSurface&               surface,
                                  const std::vector<BfInputEdge>&   inputs,
                                  bool                              face_reversed,
                                  const BfOptions&                  opt,
                                  BfReport&                         report);

// Materialise areas into `out` (which must already share the arena's vertex/edge tables).
// Adds faces, loops and trims ONLY. Asserts report.minted_edges == 0.
void bf_emit_faces(BRep&                          out,
                   const NurbsSurface&            surface,
                   const std::vector<BfInputEdge>& inputs,
                   const std::vector<BfArea>&     areas,
                   bool                           face_reversed);
```

Two representation rules that make G1 true and are different from what we do today:

1. **`BRepTrim::reversed` carries the traversal sense.** The pcurve object is stored once, in its
   natural direction; the trim's `reversed` flag says whether this wire runs along it backwards.
   Today `nurbssurface_trimmed.cpp:1367` physically reverses the pcurve and `brep.cpp:4202` always
   writes `reversed = false`, so the shared-edge relation is invisible in the data.
2. **A seam edge is two `BfInputEdge` entries with the same `edge` and different `pcurve`.**
   `bf_emit_faces` writes two trims pointing at the same `edge` index. This is exactly
   `BB.UpdateEdge(aSp, aC1, aC2, aF, aTol)` (`BOPTools_AlgoTools3D.cxx:200-231`).

---

## 4. WHAT OUR CODE DOES TODAY, AND EXACTLY WHERE IT DIVERGES

### 4.1 Where face splitting happens today

`BRep::split_with` (`src/brep.cpp:3456-4886`) is the whole subsystem. Per source face it:
1. collects the face's boundary pcurves (`brep.cpp:4223-4243`);
2. builds cut pcurves (scaffold or per-operand SSI) (`brep.cpp:4250-4760`);
3. calls `NurbsSurfaceTrimmed::split_by_uv_curves` (`brep.cpp:4786-4790`, implementation
   `nurbssurface_trimmed.cpp:536-1588`) — or the gated `split_face_by_wires`
   (`nurbssurface_trimmed.cpp:1603-...`, only when `SESSION_WIRESPLIT` is set,
   `brep.cpp:4771`);
4. for each resulting region calls the lambda `append_face` (`brep.cpp:3859-4206`), which lifts
   every UV run to a fresh 3D polyline and **mints** vertices and edges.

### 4.2 Divergence table

| # | OCCT | ours | file:line | consequence |
|---|---|---|---|---|
| D1 | edges are shared arena entities; the splitter creates none | every run is lifted to a new 3D polyline and a new edge index is minted | `brep.cpp:3489-3553` (`lift_loop`), `:4160` (`add_curve_3d`), `:4186`,`:4195` (`add_edge`) | **the identity defect**: adjacency must be re-discovered by geometry |
| D2 | vertex identity = arena index | vertex identity = quantised coordinate: `q6(x)=llround(x*1e6)` plus a 3×3×3 cell scan accepting `d² ≤ 1e-12` | `brep.cpp:2897`, `:3747-3761` | two copies of one corner that differ by >1e-6 mint two vertices; everything downstream is unmatable |
| D3 | edge identity = TShape | edge identity = `(vlo, vhi, q6(midpoint))` or, for boundary runs, `(orig_edge, vlo, vhi)` + tolerant midpoint | `brep.cpp:4012` (section), `:4169-4187` (`bemap`), `:4189-4199` (`emap`) | edge mating depends on two independent lifts agreeing to 1e-6 |
| D4 | the *only* reason D2/D3 ever work here is a **boundary snap**: every sampled UV point is clamped to the domain and snapped onto `u0/u1/v0/v1` within `snap_uv`, making two faces' copies bit-identical | — | `nurbssurface_trimmed.cpp:578-587` (`snap_border`), `:638-639` (clamp + snap applied to every sample); same pair at `:1657-1662`,`:1711-1712` | **exactly the measured failure**: on a padded domain (`u∈[-0.04,4.04]` from a STEP round-trip) the trims no longer touch the domain border, `snap_border` never fires, the last bits diverge, and 32 of 36 edges go naked while the UV arrangement is identical |
| D5 | angular key = pcurve **tangent** at the vertex, stepped by a metric-converted tolerance, curvature-capped, span-capped | angular key = `atan2` of the **chord to the next arrangement node** | `nurbssurface_trimmed.cpp:1188-1192` | on a curved section the key is a secant over an arbitrary sampling step; two nearly-tangent edges at a node sort by sampling density, not by geometry |
| D6 | `RefineAngles` / `RefineAngle2D` recover the order for convergent pcurves | absent | — (no analogue anywhere) | tangential section-meets-boundary nodes route wrongly |
| D7 | `aNbWaysInside == 1` forces the walk onto the single interior edge when arriving on a boundary edge | absent | `nurbssurface_trimmed.cpp:1196-1206` picks purely by angle | a tangentially-arriving section is skipped ⇒ the face does not split |
| D8 | `IsInside` flag = "supplied with both senses" | not represented at all: `split_by_uv_curves` gives every edge two half-edges regardless of multiplicity | `nurbssurface_trimmed.cpp:1180-1183` | the complement region is traceable and must be filtered out by area sign + a `touches_border` heuristic (`:1270-1280`) |
| D9 | boundary edges have multiplicity 1, so the exterior cycle cannot be traced | both senses always exist | same as D8 | extra cycles, and the `border_vids` heuristic decides which to drop |
| D10 | dangling prune runs on the oriented-multiplicity graph, and never prunes a degenerate edge or an INTERNAL-oriented vertex | prune runs on the undirected graph, no degenerate/internal exemptions, and is bypassed wholesale by an env flag | `nurbssurface_trimmed.cpp:1122-1147` (`SESSION_SECPROTECT`) | pole rows and legitimate sections get pruned; the workaround is a global switch |
| D11 | hole/outer from the signed UV area of the wire, with a re-discretisation loop guarding self-intersection, and a *typed* bad-wire state | signed area vs `snap_uv²`, no re-discretisation, no bad-wire state | `nurbssurface_trimmed.cpp:1224-1232`, `:1266-1281` | a sagging polyline can flip the sign of a thin wire |
| D12 | hole→owner by 2D classification of a mid-curve point against the candidate face, with tightest-owner refinement | ray-cast `point_in_cycle` against the **polyline**, tightest by area, plus a same-vertex-set exclusion | `nurbssurface_trimmed.cpp:1242-1251`, `:1292-1318` | polyline-vs-pcurve mismatch near the boundary |
| D13 | face orientation inherited from the source face | `add_face(si, false)` — always FORWARD, never inherited | `brep.cpp:3863` | orientation is re-derived later by flux/normal tests |
| D14 | one pcurve per (edge, face) stored on the edge; direction is the trim's orientation | pcurve is physically reversed to encode direction; `reversed` is always written `false` | `nurbssurface_trimmed.cpp:1367`, `brep.cpp:4202` | two adjacent faces hold two different curve objects for one edge |
| D15 | every parametric epsilon is `metric_convert(tol3d)` | domain-relative: `samp_tol = max(range)*2e-5`; `eps_border = min(range)*2e-3`; `snap_uv = tol/uv_to_3d` (a *single* isotropic scalar from one mid-domain finite difference); `scaf_forced_eps ≥ min_range*1e-2`; `bemap_tol = diag*5e-4`; `devtol = surface_diag*2e-3`; `join_tol ≥ min_range*1.5e-3` | `nurbssurface_trimmed.cpp:574`, `brep.cpp:4350`, `nurbssurface_trimmed.cpp:559-569`, `brep.cpp:4280`, `brep.cpp:3845`, `brep.cpp:3877`, `nurbssurface_trimmed.cpp:1424-1426` | a 4× padded domain inflates all of them 4×; anisotropic surfaces get one averaged scale |
| D16 | a run that cannot be trimmed is an error | a run that cannot be trimmed silently degrades to a straight UV chord and loses its provenance unless `SESSION_SEGKEEP` is set | `nurbssurface_trimmed.cpp:1373-1399` (`SEGFALL`) | a section imprints on one operand only |
| D17 | leftovers become INTERNAL wires inside the correct face, or a reported warning | no internal-wire stage | — | edges that the walk did not consume disappear |
| D18 | no post-hoc repair exists or is needed | the whole downstream repair chain: `imprint_edges`, `snap_section_edges`, `co_refine_coincident_edges`, `run_xweld`, `sew_coincident_edges`, then a fuzzy ladder | `brep.cpp:10912`, `:10918`, `:10935`, `:10941`, `:10950`, `:10965-11030` | this chain exists only to undo D1–D4; it is what the port deletes |

### 4.3 The gated partial port already in the tree

`NurbsSurfaceTrimmed::split_face_by_wires` (`nurbssurface_trimmed.cpp:1603-...`, enabled only by
`SESSION_WIRESPLIT`) already has four of the right pieces and should be the starting point, not a
rewrite from zero:
- `end_angle` (`:1873-1907`) is a faithful `Angle2D`: curvature-aware `dt`, `aTX = 0.05*(t1-t0)`
  with the `5e-5` floor, `bIsIn` reversal, folded to `[0,2π)`;
- `clockwise_angle` (`:1943-1950`) matches `ClockWiseAngle` including the `1e-14` promotion;
- vertices are keyed by **3D position** (`:1792-1810`), which collapses seam and pole images;
- a seam guard filters candidates whose start UV does not match the arrival UV (`:1962-1966`).

What it still lacks, all of which this spec adds: `is_inside` multiplicity (D8), the
`aNbWaysInside` override (D7), `RefineAngles` (D6), `Path` backtracking (it uses a plain
permutation orbit, `:1955-1987`, which cannot emit the several wires a single start edge can
produce), the internal-wire stage (D17), and — decisively — it still returns UV curves to
`append_face`, so D1–D4 are untouched.

---

## 5. ACCEPTANCE TESTS

All operands are constructed in memory. Every test states an **oracle-free invariant**: something
checkable from the output alone, without a reference kernel.

Shared harness invariants, asserted on every test:
- **I-MINT**: `report.minted_edges == 0 && report.minted_vertices == 0`.
- **I-SUBSET**: every `edge` index referenced by an output trim is in the input list.
- **I-MULT**: for each input edge `e`, `#{trims referencing e} == m(e)` where `m(e)` is its
  input multiplicity, unless `e` was reported `DanglingPruned` / `DoubledHangingPruned`.
- **I-MANIFOLD**: every edge with `m(e) == 2` has exactly 2 trims; every edge with `m(e) == 1`
  has exactly 1.
- **I-AREA**: `|Σ area3d(outputs) − area3d(input face)| ≤ 1e-9 · area3d(input face)`.
- **I-SIGN**: each output face's outer wire has positive signed UV area; each hole wire negative.
- **I-DET**: two runs give byte-identical output (face order, wire order, trim order).

### T1 — planar square, one straight chord (baseline)
Face: plane patch `u,v ∈ [0,1]`, 4 boundary edges. Section: the segment `v = 0.5`, supplied twice.
Boundary edges pre-split at `(0,0.5)` and `(1,0.5)`.
Analytic answer: 2 faces, each area 0.5.
Invariant: I-MULT gives the chord exactly 2 trims **with opposite `reversed` flags**, and the two
trims belong to different faces.

### T2 — T1 on a padded UV domain (the regression test for the measured defect)
Identical geometry, but the surface is parameterised on `u ∈ [-3, 7]`, `v ∈ [-3, 7]` with the
trims at `u,v ∈ [0,1]` (i.e. exactly the STEP round-trip shape that produced 32 naked of 36).
Invariant: **the output topology is index-for-index identical to T1's.** Same face count, same
wire sizes, same `edge` indices in the same order. Anything domain-relative fails this; nothing
metric-based can.
Second form: re-run with `u ∈ [0,1]` but the surface scaled 1000× in world units — outputs must
again be topologically identical, proving the epsilons are 3D distances, not parametric ones.

### T3 — cylinder face cut by a plane through the axis (seam crossing)
Cylinder `R = 1`, `z ∈ [0,2]`, periodic in `u`, seam at `u = 0`. Cutting plane `y = 0` gives two
section lines at `u = 0` and `u = π`. Section at `u = π` is transversal; the one at `u = 0`
coincides with the seam.
Analytic answer: 2 faces, each a half-cylinder of area `π·2 = 2π`.
Invariants: I-AREA against `2π` each; the seam edge appears with **two pcurves** (`u = 0` and
`u = 2π`) on the SAME edge index; no vertex is duplicated across the seam (I-MINT plus
`#vertices == 4`).

### T4 — cylinder face cut by an oblique plane (ellipse, no analytic sub-case)
Plane through `(0,0,1)` with normal `(0.3, 0, 1)` normalised. The section is a closed curve that
wraps the seam once.
Analytic answer: 2 faces; the lower face's area is `2πR·1 = 2π` exactly (the mean height of the
oblique cut over a full period is the axis intercept).
Invariants: I-AREA against the closed form; the section edge has exactly 2 trims; the wire that
crosses the seam contains the seam edge twice (once per pcurve) — `#{trims on the seam edge} == 2`
within a single wire, which no coordinate-based mating can produce.

### T5 — sphere face cut by a plane not through the centre (pole + degenerate edges)
Sphere `R = 1`, full parameterisation with degenerate edges at both poles. Plane `z = 0.5`.
Analytic answer: 2 faces; the cap area is `2πR(R − 0.5) = π` exactly.
Invariants: I-AREA against `π` and `3π`; the degenerate pole edges survive
(`DanglingPruned` must NOT be reported for them — the G8/`BuilderFace.cxx:200-203` rule); no wire
consists only of degenerate edges.

### T6 — section circle strictly interior (hole classification)
Planar face `u,v ∈ [0,1]`; section = a circle of radius 0.25 at `(0.5,0.5)`, closed, one vertex,
supplied twice.
Analytic answer: 2 faces — a disk of area `π/16` and an annulus of area `1 − π/16`.
Invariants: exactly one output face has 2 wires; the hole wire's signed UV area is the negative of
the disk's outer wire; the circle edge has exactly 2 trims, one per face; I-AREA.

### T7 — nested holes (owner selection)
Planar face with two concentric section circles `r = 0.4` and `r = 0.2`.
Analytic answer: 3 faces — outer annulus, middle annulus, inner disk.
Invariant: each hole is attached to the **tightest** containing face — checkable oracle-free as
"no output face contains another output face's outer wire strictly inside one of its own holes".

### T8 — section tangent to the boundary (the `aNbWaysInside` test)
Planar face `u,v ∈ [0,1]`; section = an arc from `(0, 0.5)` that leaves the left boundary
**tangentially** (initial direction `(0, +1)`, i.e. collinear with the boundary) and returns to
`(0.5, 1.0)`.
Analytic answer: 2 faces.
Invariant: face count == 2 and I-MULT holds. Today this is the exact configuration that yields a
"slit" (the cut is traversed twice by one wire and nothing splits) — see the angular-tie note at
`brep.cpp:4393-4397`. A run that produces 1 face fails.

### T9 — section dead-ending inside the face (dangling)
Planar face; section from `(0, 0.5)` to `(0.5, 0.5)` — one free end.
Analytic answer: 1 face, containing the section as an INTERNAL wire.
Invariants: `#faces == 1`; `report` contains exactly one `DoubledHangingPruned` for that edge;
the edge appears in the output as an internal wire (I-MULT relaxed for internal wires, which have
multiplicity 1); I-AREA. Nothing may vanish (G9).

### T10 — cone × cone generic pose (the currently-uncovered dispatcher arm)
Two cones in a generic relative pose so the section is a genuine quartic space curve, supplied to
the splitter as a pre-noded section edge chain.
Analytic answer: unknown in closed form — so this test checks **only** structural invariants.
Invariants: I-MINT, I-SUBSET, I-MULT, I-MANIFOLD, I-DET, and `Σ area3d(outputs) == area3d(input)`.
This is the test that says "the splitter is correct even where the intersector is approximate".

### T11 — torus with a section that self-touches at two points (multi-valence node)
Torus `Rmaj = 2`, `Rmin = 0.5`; the section is a spiric curve with two nodes of valence 4.
Invariants: at each valence-4 node the four incident records sort into a strict cyclic order with
no two angles within `Epsilon(1.)`; I-MULT; I-MANIFOLD; I-DET. If two angles do tie, the
`RefineAngles` path must have fired and the report must say so (`AngleRefineFailed` absent).

### T12 — parameterisation-reversal invariance
Take T6 and reverse the surface's `v` parameterisation (`v → 1 − v`), keeping the same 3D geometry
and the same section.
Invariant: the same face count and the same 3D areas, with all wire signed areas negated
consistently. This proves the outer/hole decision is made in the face's own frame and the face
orientation is inherited rather than derived (G4, G5).

### T13 — anisotropy stress
Cylinder `R = 1000`, `z ∈ [0, 0.001]`, cut by a plane. The U and V metrics differ by 1e6.
Invariant: face count 2 and I-AREA. Any single isotropic UV epsilon
(`nurbssurface_trimmed.cpp:559-569`) fails on one axis or the other; only per-axis
`u_resolution`/`v_resolution` survives.

### T14 — repair-chain-off assertion
Run the full boolean on the base chairs corpus with `imprint_edges`, `co_refine_coincident_edges`,
`sew_coincident_edges` and `xweld` **all disabled**.
Invariant: naked-edge count == 0. This is the single test that says the port actually replaced the
repair chain rather than being layered on top of it.

---

## 6. IMPLEMENTATION ORDER

Each step is independently shippable, independently revertable, and independently measured. The
guards battery from `kb/ARCHITECTURE_v2.md` §5 gates every step.

### Step 0 — instrument, do not change behaviour
Add `BfReport` counters to the existing path: count edges and vertices minted per source face in
`append_face`, and dump `(source face, minted edges, minted vertices, naked after combine)`.
Gate: numbers reproduce the measured 32-of-36 on the padded-domain cell; no behaviour change.
**Ship value:** the regression metric for every later step exists before any of them lands.

### Step 1 — `BfMetric`, and only that
Implement `u_resolution`/`v_resolution`/`curve_resolution` with the analytic closed forms
(plane, cylinder, cone, sphere, torus) and a numeric fallback from `|dS/du|`, `|dS/dv|` at the
point of use. No caller changes yet; unit-test against the closed forms and against finite
differences.
Gate: T13's metric values, checked directly; `u_resolution(1e-7)` on `R = 1000` equals
`2·asin(5e-11)` to 1e-15.

### Step 2 — retire the domain-relative epsilons, still on the old path
Replace, one at a time, each site in D15 with a `BfMetric` conversion of a 3D tolerance. Keep
everything else identical.
Gate: T2 (padded domain and 1000× scale) must reach the SAME naked count as the unpadded case,
even if that count is not yet 0. Corpus non-decreasing. This is the `M0` row of
`ARCHITECTURE_v2.md` §5 and can land alone.

### Step 3 — `BfInputEdge` assembly, replacing the pcurve-list interface
Build the input list from the arena: boundary edges once with their loop orientation, section /
IN / internal / seam edges twice. Feed it to the **existing** `split_by_uv_curves` by projecting
back to a pcurve list, so behaviour is unchanged, but multiplicity and `edge` indices now travel
with every run.
Gate: byte-identical output to Step 2 on the whole corpus. Multiplicity assertions (I-MULT on the
input side) hold on every corpus face.

### Step 4 — `bf_emit_faces`: stop minting (the identity fix)
Rewrite the emission half of `append_face` to look up `input.edge` instead of lifting and hashing.
`lift_loop`, `find_or_add_vertex`, `vmap`, `emap`, `bemap` are deleted from the section- and
boundary-run paths. Each output trim gets `curve_2d_index = input.pcurve` and
`reversed = (traversal sense != input sense)`.
Gate: **I-MINT == 0** on every corpus face; T1, T2, T6 pass; T14 passes on the base chairs cell
(the three repair passes disabled). This is the step that closes the reported defect.

### Step 5 — `PerformShapesToAvoid`, oriented and typed
Port `BuilderFace.cxx:152-235` exactly, on the oriented-multiplicity graph, with the degenerate
and INTERNAL-vertex exemptions. Retire `SESSION_SECPROTECT`
(`nurbssurface_trimmed.cpp:1122-1147`) — the correct rule makes the workaround unnecessary.
Gate: T5 (poles survive), T9 (dangling section pruned exactly once and reported).

### Step 6 — `Angle2D` + `Tolerance2D` as the sorting key
Replace the chord `atan2` (`nurbssurface_trimmed.cpp:1188-1192`) with the tangent form from
`split_face_by_wires::end_angle` (`:1873-1907`), now fed by `BfMetric` instead of the isotropic
`snap_uv`. Keep the walk otherwise unchanged.
Gate: T11 (no angular ties at valence-4 nodes); corpus non-decreasing; T4.

### Step 7 — `Path` with backtracking, and the two override rules
Replace the permutation-orbit extraction with the OCCT `Path` stack machine
(`WireSplitter_1.cxx:358-617`): the closure scan with per-axis 2D tolerance, the truncate-and-
continue rule, the `iCnt == 1` short circuit, the `IsSame(aEOuta) ⇒ 2π` demotion, the `bIsClosed`
`Coord2dVf` filter, and the `aNbWaysInside == 1` override. Add `max_walk_steps` as a hard
blow-up guard (OCCT has none — see `kb/audit_occt_blowup-guards.md`).
Gate: T8 (tangential section splits the face), T3, T4 (seam-crossing wires), I-DET.

### Step 8 — `RefineAngles` / `RefineAngle2D`
Port `WireSplitter_1.cxx:905-1125` including the `aCf = 0.01`, `aTolInt = 1e-10`,
`MaxDT = 0.3·span` constants and the `±Precision::Angular()` last-resort nudge, and report
`AngleRefineFailed` when both rays fail.
Gate: T8 with the section made *exactly* tangent (curvature-matched) at the boundary; T11.

### Step 9 — `PerformAreas`: signed-area hole test + owner assignment
Replace the `snap_uv²` area threshold and the polyline ray-cast with: (a) the FClass2d wire
sampling and re-discretisation loop (`IntTools_FClass2d.cxx:229-565`) producing
`{outer, hole, bad}`; (b) UV-box overlap prefilter; (c) `IsInside` by mid-parameter classification
of a candidate wire edge against the candidate face; (d) tightest-owner refinement; (e) the
unassigned-hole rule, with `HoleUnassigned` reported rather than dropped silently.
Gate: T6, T7, T12.

### Step 10 — `PerformInternalShapes`
Port `BuilderFace.cxx:618-778` and `MakeInternalWires` (`:782-838`). Emit `EdgeNotConsumed` for
anything still unplaced.
Gate: T9; G9 holds on the whole corpus (`Σ report.EdgeNotConsumed == 0` on clean cells).

### Step 11 — orientation inheritance
`bf_emit_faces` writes `BRepFace::reversed = face_reversed`, inherited from the DS source face
(`BOPAlgo_Builder_2.cxx:535-552`); delete the downstream flux/normal re-derivation for split faces.
Gate: T12; the oriented-primitive battery.

### Step 12 — delete the repair chain
Remove `imprint_edges`, `snap_section_edges`, `co_refine_coincident_edges`, `run_xweld`,
`sew_coincident_edges` and the fuzzy ladder from the boolean pipeline
(`brep.cpp:10912-11030`).
Gate: T14 on the entire corpus — base 3 ops exact, matrix 45/45, edge 54/54, rotated chairs, the
224-cell primitive sweep. Naked-edge count 0 with every repair pass compiled out.

---

## 7. WHERE OCCT ITSELF GIVES UP (do not invent a fix; reproduce the documented outcome)

1. **No loops and a bounded face** — `PerformAreas` returns with `myAreas` empty; the face
   disappears from the result with no alert (`BuilderFace.cxx:401-414`).
2. **Unassigned holes on a bounded face** — dropped silently; only an *infinite* face gets the
   synthetic carrier face (`BuilderFace.cxx:558-581`).
3. **`RefineAngle2D` failure with more than two interior records** — the angle is left unrefined
   and the walk proceeds on a key known to be unreliable (`WireSplitter_1.cxx:991-1000`: the
   `±Precision::Angular()` fallback fires only when `iCntInt == 2`).
4. **`AngleIn` with no matching IN record** returns `0.` silently (`WireSplitter_1.cxx:743-744`) —
   the walk then measures every clockwise angle from `+X`. Our port should assert here.
5. **`Path` dead ends** — `return` with the partial stack discarded, no alert
   (`WireSplitter_1.cxx:551-555, 607-611`). The edges involved are later swept up by the
   `PerformLoops` post-treatment (`BuilderFace.cxx:313-321`) into `myShapesToAvoid`, so they
   surface as internal shapes or as the unused-edges warning — that is the only reason the loss is
   observable at all.
6. **`UResolution`/`VResolution` for a BSpline surface** is `R3d * 0.01`
   (`GeomAdaptor_Surface.cxx:1886-1888` → `Precision.hxx:328`) — a constant, not a metric. Our
   `BfMetric` numeric fallback (`tol3d / |dS/du|`) is strictly better and is a deliberate
   divergence; record it as such.
7. **No blow-up guard.** There is no iteration cap in `Path`, no cap on wires per block, and no
   memory guard anywhere in this subsystem. `max_walk_steps` in `BfOptions` is ours, not OCCT's.
