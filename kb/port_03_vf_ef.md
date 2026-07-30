# port_03 — Interference stages 4 and 5: Vertex/Face and Edge/Face

**Scope.** A complete, implementable specification of OCCT's VF (stage 4) and EF (stage 5)
interference stages, written so this subsystem can be built without opening OCCT again.
Every algorithmic claim carries a `file:line` citation into
`/home/petras/code/code_cpp/OCCT/src/`. Where OCCT gives up, approximates, or falls back,
that is stated as such — a documented fallback is part of the specification.

**OCCT source roots used** (paths abbreviated below as noted):

| abbreviation | real path (under `/home/petras/code/code_cpp/OCCT/src/`) |
|---|---|
| `PF4` | `ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_4.cxx` (391 lines, VF) |
| `PF5` | `ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_5.cxx` (1200 lines, EF) |
| `PF3` | `ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_3.cxx` |
| `PF9` | `ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_9.cxx` |
| `PF10` | `ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_10.cxx` |
| `CTX` | `ModelingAlgorithms/TKBO/IntTools/IntTools_Context.cxx` (1040 lines) |
| `IEF` | `ModelingAlgorithms/TKBO/IntTools/IntTools_EdgeFace.cxx` (876 lines) |
| `BFI` | `ModelingAlgorithms/TKBO/IntTools/IntTools_BeanFaceIntersector.cxx` (2640 lines) |
| `MRS` | `ModelingAlgorithms/TKBO/IntTools/IntTools_MarkedRangeSet.cxx` |
| `SHR` | `ModelingAlgorithms/TKBO/IntTools/IntTools_ShrunkRange.cxx` |
| `TOOLS` | `ModelingAlgorithms/TKBO/IntTools/IntTools_Tools.cxx` (805 lines) |
| `AT2` | `ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools_2.cxx` |
| `DS` | `ModelingAlgorithms/TKBO/BOPDS/BOPDS_DS.cxx` |
| `ICS` | `ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_Inter.pxx` |
| `ICSU` | `ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_InterUtils.pxx` |

**Note on OCCT layout.** `IntTools` lives under `TKBO`, *not* `TKGeomAlgo`, in this checkout.
`IntCurveSurface` has been refactored from `.gxx` templates into `.pxx` header templates
(`IntCurveSurface_Inter.pxx`, `IntCurveSurface_InterUtils.pxx`); the classical
`IntCurveSurface_Inter.gxx` no longer exists.

**Our source** is read-only for this document; citations are `src/<file>:<line>` under
`/home/petras/code/code_rust/session/session_cpp/`.

---

## 1. WHAT THIS SUBSYSTEM MUST GUARANTEE

These are the invariants. Each is stated so it can be turned into an assertion or a test.
`A` and `B` are the two operands; `tol(x)` is the model-space tolerance of entity `x`;
`fuzz` is the user fuzzy value (default `0`, floored at `Precision::Confusion()` = `1e-7`).

### G1 — Piercing completeness (the load-bearing one)

> For every edge `e` of `A` and every face `f` of `B`, and for every parameter `t` in the
> **shrunk range** of any pave block of `e` such that `dist(C_e(t), f) <= tol(e)+tol(f)+fuzz`
> and the foot point projects strictly inside `f`'s trimmed UV region, the result contains a
> vertex `v` with `dist(v, C_e(t)) <= tol(v)`, **and** `e` carries a pave at `t` that splits
> its pave block there.

Two halves, both testable independently: (a) the vertex exists in the result; (b) the *edge*
was actually split at it. Our current kernel can satisfy (a) by accident (see §4) and never
satisfies (b) by construction.

### G2 — Symmetry

> The EF stage is run twice with the roles exchanged (`A.edges × B.faces` and
> `B.edges × A.faces`) and the two runs write into the *same* shared arena. A vertex minted by
> `A.e × B.f` and a vertex minted by `B.e × A.f` at the same location must be **one** vertex
> object, not two coincident ones.

This is a structural requirement on the arena (`kb/ARCHITECTURE_v2.md` §1, law 1), not a
tolerance question.

### G3 — Typed contact (transversal vs tangential vs coincident)

> Every edge/face contact is classified into exactly one of three named outcomes, and each
> emits a different record:
>
> | outcome | test | emits |
> |---|---|---|
> | **transversal piercing** | isolated common part, `type = VERTEX` | new vertex + pave on the edge + `InterfEF` with `IndexNew` |
> | **tangential touch** | isolated common part, `type = VERTEX`, but the extremum is a local minimum of distance rather than a sign change | same records as piercing (OCCT does *not* distinguish them in what it emits — see §2.5.6); the distinction is internal, used only to place `t` correctly |
> | **coincidence (edge lies ON face)** | common part `type = EDGE` | **no new vertex**; an edge/face *common block*, and the pave block is added to `FaceInfo.PaveBlocksIn(f)` |
>
> No contact may fall through unclassified.

### G4 — Face IN/ON sets are complete before splitting

> Before face `f` is split, `FaceInfo(f)` holds:
> - `VerticesOn(f)` — every vertex that is a sub-shape of `f`, plus both paves of every pave
>   block of `f`'s own edges;
> - `VerticesIn(f)` — every vertex lying strictly inside `f` (VF hits, EF-born vertices,
>   EE-born vertices re-tested against `f`, and `f`'s own internal vertices);
> - `PaveBlocksIn(f)` — every foreign pave block that coincides with `f`;
> - `PaveBlocksOn(f)` — `f`'s own pave blocks.
>
> The splitter reads only these sets. Anything not in them cannot appear in the split.

### G5 — Tolerance growth is recorded, never silent

> If a VF or EF interference is accepted at distance `d > 0`, the participating vertex's
> tolerance grows to at least `d + tol(f)` (VF) or `max(tol(e), tol(f))` (EF), and that growth
> is recorded against the vertex so downstream stages compare against the grown value.

### G6 — No vertex may be created on a face boundary

> An EF vertex whose projection lands **ON** the face's boundary wire (state `ON`, not `IN`)
> is rejected. That contact is EE or VE territory and must have been produced by stage 3 or 2.
> Creating it here produces two vertices at one location and a non-manifold edge.

### G7 — Domain independence

> None of the tolerances in this subsystem may be derived from a UV domain extent. Every
> comparison is a 3D distance. UV appears only as (i) the *restriction rectangle* for the
> projector (`UVBounds`), and (ii) the argument to point-in-face classification. A face
> re-imported on `u ∈ [-0.04, 4.04]` instead of `u ∈ [0,1]` must produce byte-identical
> interference records.

### G8 — Idempotence / no double-counting

> Running the stage twice on the same arena creates no new entities. Each (vertex, face) and
> (pave-block, face) pair is processed at most once; SD (same-domain) vertex groups are
> processed once for the whole group.

---

## 2. OCCT'S ALGORITHM

### 2.0 Constants, with their actual values

Collected here because they are scattered across six files. All are literal in the source.

| symbol | value | site |
|---|---|---|
| `Precision::Confusion()` | `1.e-7` | `FoundationClasses/TKernel/Precision/Precision.hxx:165` |
| `Precision::SquareConfusion()` | `1.e-14` | `Precision.hxx:169` |
| `Precision::Angular()` | `1.e-12` | `Precision.hxx:123` |
| `Precision::Intersection()` | `Confusion()*0.01` = `1.e-9` | `Precision.hxx:220` |
| `Precision::PConfusion()` | `Confusion()*0.01` = `1.e-9` | `Precision.hxx:334` |
| `Precision::Approximation()` | `Confusion()*10` = `1.e-6` | `Precision.hxx:235` |
| `BOPTools_AlgoTools::DTolerance()` | `1.e-12` | `BOPTools/BOPTools_AlgoTools.hxx:70` |
| `IntTools_Context::myPOnSTolerance` | `1.e-12` (projector tolerance) | `CTX:65`, `CTX:84` |
| VF: `aTolSum` | `tol(V) + tol(F) + max(fuzz, 1e-7)` | `CTX:573` |
| EF `myCriteria` (generic) | `tol(E) + tol(F) + fuzz` | `IEF:548` with `aFuzz = fuzz/2` at `IEF:529-531` |
| EF `myCriteria` (BSpline/Bezier curve) | `1.5*tol(E) + tol(F)`, or `max(tolE,tolF)` if the ratio exceeds 100 | `IEF:532-545` |
| `IsCoincident` sample count | `23` segments (`2` for Line×Plane) | `IEF:73-77` |
| `IsCoincident` accept fraction | `> 0.5` | `IEF:79`, `IEF:161-162` |
| `IsCoincident` hard reject factor | `100 × myCriteria` | `IEF:110-113` |
| `IsCoincident` boundary shift | `0.01 × (t2-t1)` at each end | `IEF:86-90` |
| EF "on pave" decision tolerance | `5.e-8` | `PF5:416` |
| EF existing-vertex extension cap | `min(1.e4 × tol(V), 0.1)` when `tol(V) < 0.01`, else `1.e4 × tol(V)` | `PF5:490-494` |
| EF real-intersection test | `LowerDistance < Precision::Intersection()` = `1e-9` | `PF5:472` |
| `CheckTouchVertex` end-exclusion | `aEpsT = 8.e-5`, or `9.e-5` for a Line | `IEF:700-704` |
| `ForceInterfEF` normal/tangent gate | `|cos| > 0.4226` disables the extra tolerance (≈25° from perpendicular) | `PF5:1046-1050` |
| `ForceInterfEF` distance criterion | `2 × max(tol(V1), tol(V2))` (or `fuzz` in SI-check mode) | `PF5:1022-1024` |
| `IntCurveSurface` Newton tolerance | `THE_TOLTANGENCY = 1.e-8` | `ICS:49` |
| `IntCurveSurface` angular tolerance | `THE_TOLERANCE_ANGULAIRE = 1.e-12` | `ICS:50` |
| `IntCurveSurface` conic sample counts | circle 32, ellipse 32, parabola 16, hyperbola 32 | `ICS:52-55` |
| `IntCurveSurface` polyhedron sampling caps | `nbsu,nbsv ≤ 40`; for the line fallback, `≥ 20` | `ICS:208-215`, `ICS:681-688` |
| `IntCurveSurface` start-point dedup | `10 × PConfusion()` = `1e-8` | `ICSU:1481`, `ICSU:1388` |
| `IntCurveSurface` UV clamp | `±1.0e50` | `ICSU:1616` |
| `BeanFaceIntersector` exact-mode criteria override | `3 × Confusion()` = `3e-7` when >1 exact point found | `BFI:582` |
| `BeanFaceIntersector` march min step | `myCurveResolution × 0.5`, capped by `0.1 × (t2-t1)` | `BFI:1189-1195` |
| `BeanFaceIntersector` march loop cap | `loopcounter <= 10` | `BFI:1221` |
| `BeanFaceIntersector` extrema tolerance | `1.e-10` (both directions) | `BFI:1227` |
| `BeanFaceIntersector` localization min cell | `10 × PConfusion()` UV, `10 × myCurveResolution` param | `BFI:1824-1825`, `BFI:1897` |
| `BeanFaceIntersector` `TestComputeCoinside` samples | `23` | `BFI:2132` |
| `ShrunkRange` splittability threshold | length `> 2·tol(E) + 2·Confusion()` | `SHR:184-187` |

### 2.1 Stage 4 — VF (`PerformVF`, `PF4:139-301`)

#### 2.1.1 The pair loop

`myIterator->Initialize(TopAbs_VERTEX, TopAbs_FACE)` (`PF4:141`) enumerates candidate
vertex/face pairs; the iterator is bounding-box driven and only yields pairs from *different*
operands or, in self-interference mode, from the same one.

Skips, in order (`PF4:181-232`):

1. `myDS->IsSubShape(nV, nF)` (`PF4:189`) — the vertex belongs to the face. Not an
   interference.
2. `myDS->HasInterf(nV, nF)` (`PF4:194`) — already recorded.
3. `myDS->ChangeFaceInfo(nF)` (`PF4:199`) — **side effect, not a skip**: this *creates*
   `FaceInfo` for the face if absent, seeding `VerticesIn` with the face's own internal
   vertices (`DS:751-769`) and `VerticesOn`/`PaveBlocksOn` from its own edges
   (`DS:811-833`). Every face that appears in *any* VF candidate pair gets its `FaceInfo`
   initialised here, even if the intersection then fails.
4. `myDS->HasInterfShapeSubShapes(nV, nF)` (`PF4:200`) — the vertex already interferes with
   some sub-shape of the face (an edge or a vertex of it). That is VE/VV territory.
5. SD collapse: if `nV` has a same-domain representative `nVSD`, use `nVSD` (`PF4:205-209`).
   A `(nVx, nF)` pair is intersected **once**; every original `nV` in the SD group is recorded
   against the single result (`PF4:211-221`, replayed at `PF4:272-293`). This is invariant G8.

The surviving pairs are pushed into a vector and intersected in parallel (`PF4:242`).

#### 2.1.2 The kernel: `IntTools_Context::ComputeVF` (`CTX:545-590`)

```
int ComputeVF(V, F, out U, out V_param, out theTol, fuzz):
    aP = BRep_Tool::Pnt(V)                                     # CTX:555

    # 1. project onto the *restricted* surface
    proj = ProjPS(F)                                           # CTX:558  (cached per face)
    proj.Perform(aP)
    if !proj.IsDone(): return -1                               # CTX:561-564

    # 2. distance gate
    aDist  = proj.LowerDistance()                              # CTX:568
    aTolV  = BRep_Tool::Tolerance(V)
    aTolF  = BRep_Tool::Tolerance(F)
    aTolSum = aTolV + aTolF + max(fuzz, Precision::Confusion())   # CTX:573
    theTol  = aDist + aTolF                                    # CTX:574   <-- the NEW vertex tol
    proj.LowerDistanceParameters(U, V_param)                   # CTX:575
    if aDist > aTolSum: return -2                              # CTX:577-581

    # 3. UV containment gate
    if !IsPointInFace(F, gp_Pnt2d(U, V_param)): return -3      # CTX:583-588
    return 0
```

Three things must be ported exactly:

- **The projector is UV-restricted.** `ProjPS(F)` (`CTX:247-265`) initialises
  `GeomAPI_ProjectPointOnSurf` with `UVBounds(F, Umin, Usup, Vmin, Vsup)` (`CTX:253`), which
  comes from the *face's* `BRepAdaptor_Surface` bounds — `FirstUParameter … LastVParameter`
  (`CTX:1028-1039`) — not the underlying surface's natural domain. The projector tolerance is
  `myPOnSTolerance = 1.e-12` (`CTX:65`) and the extrema flag is `Extrema_ExtFlag_MIN`
  (`CTX:260`), i.e. only minima are sought. **This is the "UV-box restriction" the brief asks
  about**: it is a *rectangular* pre-restriction of the projection search, applied before any
  trim-loop classification. Its purpose is to stop the projector from returning a foot on a
  distant lobe of a periodic or self-overlapping surface.
- **`theTol = aDist + aTolF`, not `max`.** The returned tolerance is the *candidate* new
  tolerance; `UpdateVertex` applies it only if it is larger than the current one
  (`PF10:117-125`). So a vertex accepted at distance `d` from a face ends up with tolerance
  `≥ d + tol(F)` — it swallows the gap and the face's own uncertainty. Invariant G5.
- **The containment test is strict IN.** `IsPointInFace(F, P2d)` returns
  `state != TopAbs_OUT && state != TopAbs_ON` (`CTX:604-608`). A vertex whose projection lands
  exactly on the face's boundary wire is **rejected with `-3`**. The `ON` case belongs to VE.
  (Contrast `IsPointInOnFace`, `CTX:639-643`, which accepts `ON`; that one is used by
  `IsValidPointForFace`, `CTX:670`, i.e. by the EF range filter, not by VF.)

The classification itself is `IntTools_FClass2d::Perform` (`CTX:597-599`), which first tries a
cheap per-wire polygon test and falls back to `BRepClass_FClassifier` with a metric-converted
tolerance `min(UResolution(Toluv), VResolution(Toluv))`
(`IntTools/IntTools_FClass2d.cxx:637-756`, tolerance computation at `:728-746`). Note the
tolerance conversion at `:732-745`: the UV tolerance is derived from the 3D face tolerance
through the surface's own `UResolution`/`VResolution` — i.e. the metric conversion demanded by
G7, done at the point of use.

#### 2.1.3 What VF creates (`PF4:272-297`)

For each original `nV` in the SD group:

1. `BOPDS_InterfVF` appended, carrying `(nV, nF)` and the `(U,V)` parameters (`PF4:279-281`;
   `SetUV` at `BOPDS/BOPDS_Interf.hxx:355`).
2. `myDS->AddInterf(nV, nF)` — registers the pair in the interference map (`PF4:283`).
3. `nVx = UpdateVertex(nV, aTolVNew)` (`PF4:286`) — see §2.1.4.
4. If `UpdateVertex` minted a new shape, `aVF.SetIndexNew(nVx)` (`PF4:289-292`).

Then, **once per (SD-vertex, face) pair**: `FaceInfo(nF).ChangeVerticesIn().Add(nVx)`
(`PF4:295-297`).

**VF creates no paves on any edge.** It only records vertex-in-face incidence and grows the
vertex tolerance. This is the single most commonly mis-stated fact about the stage.

#### 2.1.4 `UpdateVertex` (`PF10:105-162`)

```
int UpdateVertex(nV, aTolNew):
    nVNew = nV
    if IsNewShape(nVNew) or HasShapeSD(nV, nVNew) or !myNonDestructive:
        # allowed to mutate in place
        if tol(shape(nVNew)) < aTolNew:
            BRep_Builder().UpdateVertex(shape(nVNew), aTolNew)
            recompute the DS bounding box, gap += Confusion()
            myIncreasedSS.Add(nV)
        return nVNew
    # non-destructive mode on an argument vertex: mint a copy
    aVNew = MakeVertex(Pnt(V), max(tol(V), aTolNew))
    nVNew = DS.Append(aVNew)
    box(nVNew) = bbox(aVNew), gap += Confusion()
    DS.AddShapeSD(nV, nVNew)                      # old -> new alias
    myVertsToAvoidExtension.Add(nVNew)
    if tol(V) < aTolNew: myIncreasedSS.Add(nV)
    return nVNew
```

The alias `AddShapeSD(nV, nVNew)` is what makes G8 work across stages: every later stage
resolves `nV` through the SD map before comparing.

#### 2.1.5 `TreatVerticesEE` (`PF4:305-390`) — the second VF pass

After the main loop (and also as the *only* action when there are no VF candidate pairs at
all, `PF4:163-170`), vertices **born in stage 3 (EE)** are tested against every face:

1. Collect `IndexNew()` of every `BOPDS_InterfEE` (`PF4:319-332`). If none, return.
2. Collect every face index (`PF4:340-347`). If none, return.
3. `BOPDS_SubIterator` over (faces × those vertices) (`PF4:356-362`) — this is a bbox-driven
   sub-iteration, not a full product.
4. For each pair, if the vertex is **not** already in `FaceInfo(nF).VerticesOn()` (`PF4:370`),
   run `ComputeVF` (`PF4:374`) and on success append `InterfVF`, `AddInterf`, and add to
   `VerticesIn` (`PF4:378-386`).

Note the asymmetry with the main loop: here there is **no `UpdateVertex` call**. The tolerance
out-parameter is discarded into `dummy` (`PF4:374`). An EE vertex that lands on a face does not
grow. This is deliberate — the EE vertex already carries a tolerance sized by the EE common
part.

#### 2.1.6 `ForceInterfVF` (`PF5:631-681`) — VF called *from* EF

EF calls this when a piercing point lands on one of the pave block's end vertices and that
vertex is not yet known to the face. Its acceptance rule is *looser* than `PerformVF`'s:

```
iFlag = ComputeVF(V, F, U, V_param, aTolVNew, fuzz)
if iFlag == 0 or iFlag == -2:                     # PF5:642  <-- -2 accepted!
    append InterfVF(nV,nF) with (U,V); AddInterf
    nVx = UpdateVertex(nV, aTolVNew)              # PF5:657   grows tol to aDist + tolF
    if IsNewShape(nVx): aVF.SetIndexNew(nVx)
    FaceInfo(nF).ChangeVerticesIn().Add(nVx)      # PF5:664-666
    if Rank(nV) == Rank(nF) >= 0: warn self-interfering  # PF5:669-678
    return true
return false
```

`-2` means *"the distance exceeded `tol(V)+tol(F)+fuzz`"*. Accepting it means: **force the
vertex to reach the face by growing its tolerance to `aDist + tol(F)`**. `-1` (unprojectable)
and `-3` (projects outside the trimmed face) are still rejected. This is a deliberate,
localised violation of the normal distance gate, and it exists so that a near-miss piercing at
an edge end becomes a vertex-on-face rather than a spurious new vertex a hair away from an
existing one.

---

### 2.2 Stage 5 — EF: the driver (`PerformEF`, `PF5:165-592`)

#### 2.2.1 Prologue

`FillShrunkData(TopAbs_EDGE, TopAbs_FACE)` (`PF5:167`, implementation `PF9:65-129`+): for every
edge appearing in an edge/face candidate pair, and for every pave block of that edge, compute
the **shrunk range** if not already valid. Degenerate edges (`aSIE.HasFlag()`) are skipped
(`PF9:95-98`).

The shrunk range (`SHR:107-191`) is the part of the pave block's parameter interval **not
covered by the tolerance spheres of its two end vertices**:

```
aTolV1 = max(tol(V1), tol(E)) + Confusion()          # SHR:129-141
aTolV2 = max(tol(V2), tol(E)) + Confusion()
BRepLib::FindValidRange(BAC, tol(E), t1, P1, aTolV1, t2, P2, aTolV2, out tS1, out tS2)  # SHR:147
if !found                       -> not done          # SHR:147-151
if (tS2 - tS1) < PConfusion()   -> micro edge, not done   # SHR:152-156
length = GCPnts_AbscissaPoint::Length(BAC, tS1, tS2, aPTolE)   # SHR:170
   where aPTolE = min(BAC.Resolution(tol(E)), (t2-t1)/100)     # SHR:162-169
if length < Confusion()         -> micro edge, not done   # SHR:171-175
myIsDone = true
myIsSplittable = length > 2*tol(E) + 2*Confusion()   # SHR:184-187
bndBox = bbox(BAC, tS1, tS2, gap = tol(E) + Confusion())  # SHR:190
```

`IsSplittable` is the gate that later decides whether a piercing may create a new vertex at
all (`PF5:369`, `PF5:442-445`). An edge too short to be split once is never split.

The **glue-full** short-circuit (`PF5:179-192`): in `BOPAlgo_GlueFull` mode no intersection is
performed at all; the loop only ensures `FaceInfo` exists for each face. Not needed for our
port unless we add a glue mode.

#### 2.2.2 The pave-block loop (`PF5:219-307`)

EF works **per pave block**, not per edge. An edge already split by VV/VE/EE is handled
piecewise, and each piece is intersected against the face independently.

For each candidate `(nE, nF)`:

- skip if `ShapeInfo(nE).HasFlag()` — degenerate edge (`PF5:227-231`);
- `aFI = myDS->ChangeFaceInfo(nF)` (`PF5:237`) — again the side effect that materialises
  `FaceInfo`;
- read `aMPBF = aFI.PaveBlocksOn()`, `aMVIn = aFI.VerticesIn()`, `aMVOn = aFI.VerticesOn()`
  (`PF5:238-241`).

For each pave block `aPB` of `nE` (`PF5:246-306`):

1. `aPBR = RealPaveBlock(aPB)` — the common-block representative if any. If
   `aMPBF.Contains(aPBR)` the block is already known to be **on** the face; skip (`PF5:256-260`).
2. `GetPBBox(...)` (`PF5:263`, impl `PF3:914-955`) returns the PB range `(aT1,aT2)`, the shrunk
   range `(aTS1,aTS2)` and a bounding box. Returns `false` — and the block is skipped — when
   `aT2-aT1 <= PConfusion()` (`PF3:925-929`). If shrunk data exists it is used; otherwise the
   shrunk range is set equal to the full range and the box is built with
   `BndLib_Add3dCurve::Add(BAC, tS1, tS2, tol(E)+Confusion(), box)` and cached (`PF3:939-953`).
3. Bounding-box reject: `if (aBBF.IsOut(aBBE)) continue` (`PF5:268-271`).
4. **Quick-coincidence eligibility.** `bV1 = aMVIn.Contains(nV1) || aMVOn.Contains(nV1)` and
   likewise `bV2`; `bExpressCompute = bV1 && bV2` (`PF5:273-276`). Passed to
   `UseQuickCoincidenceCheck` (`PF5:287`). The documented precondition
   (`IntTools_EdgeFace.hxx:93-98`) is: both vertices lie on the face **and** the edge does not
   cross the face's boundary within the range. Only the first half is actually checked; the
   second is assumed. This is a known soft spot in OCCT.
5. **Range correction, twice** (`PF5:289-297`):
   - `anewSR` from the shrunk range `(aTS1,aTS2)` via `BOPTools_AlgoTools::CorrectRange(E, F, …)`;
   - `aPBRange` from the full PB range `(aT1,aT2)` via the same call; this becomes the
     intersector's working range (`SetRange`).

   `CorrectRange(E, F, aSR, aNewSR)` (`AT2:364-434`) trims `aRes` off each end where
   `aRes = BAC.Resolution(tol(F))` for analytic curves, or `tol(F) / |C'(t)|` for
   BSpline/Bezier/Offset/Other when `|C'| > 1e-12` (`AT2:391-418`). If the trim would make the
   range shorter than `PConfusion()`, the original range is restored (`AT2:429-432`). **This is
   a 3D-tolerance-to-parameter conversion done through the curve's own derivative — the metric
   conversion of G7, at the point of use.**
6. Register the pair in `myFPBDone[nF]` (`PF5:300-305`) so the later `ForceInterfEF` pass does
   not redundantly re-intersect it.

Then all prepared `BOPAlgo_EdgeFace` objects are run in parallel (`PF5:317`).

#### 2.2.3 The worker's conditioning trick (`PF5:105-147`)

Before calling `IntTools_EdgeFace::Perform`, the worker checks
`BOPAlgo_Tools::TrsfToPoint(box1, box2, aTrsf)` (`PF5:120`) and, if the shapes are far from the
origin, **moves both edge and face to the origin** (`PF5:124-127`), intersects there, then
restores the original shapes and re-stamps the common parts' edge (`PF5:136-145`). Purely a
floating-point conditioning measure; port it, it is cheap and it matters for models placed at
large coordinates.

Failures are caught (`PF5:132-135`) and turned into a per-pair warning, not a global failure
(`PF5:331-336`).

---

### 2.3 Stage 5 — EF: the solver `IntTools_EdgeFace::Perform` (`IEF:504-685`)

```
Perform():
    CheckData()                                  # IEF:512  -> myErrorStatus 2 (degenerate)
                                                 #             or 3 (non-geometric edge); abort
    myC.Initialize(myEdge)                       # IEF:524  BRepAdaptor_Curve
    aFuzz = myFuzzyValue / 2                     # IEF:529
    aTolF = tol(F) + aFuzz ; aTolE = tol(E) + aFuzz
    if curve is BSpline or Bezier:               # IEF:532-545
        if max(aTolE/aTolF, aTolF/aTolE) > 100: myCriteria = max(aTolE, aTolF)
        else:                                    myCriteria = 1.5*aTolE + aTolF
    else:                                        myCriteria = aTolE + aTolF
    myS = SurfaceAdaptor(F)                      # IEF:551

    if myQuickCoincidenceCheck and IsCoincident():   # IEF:553-563
        emit ONE common part, type = EDGE, range = myRange; done; return

    anIntersector = IntTools_BeanFaceIntersector(myC, myS, aTolE, aTolF)   # IEF:565
    anIntersector.SetBeanParameters(myRange.First(), myRange.Last())       # IEF:566
    anIntersector.Perform()                                               # IEF:570
    myMinDistance = sqrt(anIntersector.MinimalSquareDistance())           # IEF:572-575
    if !anIntersector.IsDone(): return                                    # IEF:577-580

    for each result range R:                                              # IEF:582-591
        if IsProjectable(IntermediatePoint(R.First(), R.Last())):
            emit common part with Range1 = R

    for each common part: set bounding points C(t1), C(t2); MakeType()    # IEF:595-608

    # analytic post-corrections, see 2.3.4
    Line x Cylinder  : IEF:620-646
    Circle x Plane   : IEF:650-682
    myIsDone = true
```

#### 2.3.1 `IsCoincident` (`IEF:62-163`) — the quick coincidence check

Used only when both PB vertices are known to lie on the face.

```
aNbSeg = 23                                     # IEF:73
if curve is Line and surface is Plane: aNbSeg = 2    # IEF:74-77
aTresh = 0.5                                    # IEF:79
aTreshIdxF = round((aNbSeg+1)*0.25)             # IEF:80
aTreshIdxL = round((aNbSeg+1)*0.75)             # IEF:81
t1 = myRange.First() + 0.01*(t2-t1)             # IEF:86-89   shift off the ends
t2 = myRange.Last()  - 0.01*(t2-t1)
dT = (t2-t1)/aNbSeg
iCnt = 0 ; isClassified = false
for i in 0..aNbSeg:
    P = C(t1 + i*dT)
    proj = ProjPS(F); proj.Perform(P)
    if !proj.IsDone(): continue                 # IEF:101-104   sample simply skipped
    aD = proj.LowerDistance()
    if aD > myCriteria:
        if aD > 100*myCriteria: return false    # IEF:110-113   hard reject
        else: continue                          # IEF:114-117   soft skip
    iCnt++
    # classify only i == 0, one index in [aTreshIdxF, aTreshIdxL], and i == aNbSeg
    if (0<i<aTreshIdxF) or (aTreshIdxL<i<aNbSeg): continue      # IEF:133-136
    if isClassified and i != aNbSeg: continue                   # IEF:138-141
    state = FClass2d(F).Perform(proj.LowerDistanceParameters())
    if state == TopAbs_OUT: return false                        # IEF:150-153
    if i != 0: isClassified = true                              # IEF:155-158
return (iCnt / (aNbSeg+1)) > aTresh             # IEF:161-162
```

**This is a majority vote over 24 samples with only three UV classifications.** It is
explicitly a heuristic, not a proof of coincidence. OCCT protects itself by only enabling it
when both endpoints are already known to lie on the face. Port it with exactly that guard.

#### 2.3.2 `IsProjectable` (`IEF:181-190`) — the result filter

```
IsProjectable(t) = myContext->IsValidPointForFace(C(t), F, myCriteria)
```

`IsValidPointForFace` (`CTX:647-673`): project; if `LowerDistance > aTol` return false;
otherwise classify the foot with **`IsPointInOnFace`** — i.e. `state != OUT`, so `ON` is
accepted (`CTX:670`, `CTX:639-643`). A common part whose midpoint projects onto the face's
boundary wire survives here. The rejection of boundary points happens later, at `PF5:523`,
using the strict `IsPointInFace`. That two-stage asymmetry is deliberate and must be preserved:
the range filter is permissive, the vertex-creation filter is strict (invariant G6).

#### 2.3.3 `MakeType` (`IEF:304-359`) — VERTEX or EDGE

```
if aCommonPrt.AllNullFlag(): type = EDGE; return          # IEF:310-315
(af1, al1) = Range1
PF = C(af1) ; PL = C(al1) ; df1 = |PF - PL|
isWholeRange = |af1 - myRange.First()| < myC.Resolution(myCriteria)
            && |al1 - myRange.Last()|  < myC.Resolution(myCriteria)   # IEF:326-330
if df1 > 2*myCriteria and isWholeRange:  type = EDGE                  # IEF:332-335
else:
    if isWholeRange:
        tm = (af1+al1)/2
        if |PF - C(tm)| > 2*myCriteria: type = EDGE; return           # IEF:338-347
    if !CheckTouch(cp, tm): tm = (af1+al1)/2                          # IEF:349-352
    type = VERTEX ; VertexParameter1 = tm ; Range1 = (af1, al1)       # IEF:353-355
```

So: a common part spanning the **whole** working range whose endpoints are more than
`2·myCriteria` apart in 3D is an EDGE (coincidence). Everything else is a VERTEX, whose
parameter is `CheckTouch`'s answer if it succeeded, else the midpoint of the range.

Note that a VERTEX common part **retains a range** `(af1, al1)`, not just a point. That range
is what `PF5` later compares against the pave-block end ranges.

#### 2.3.4 `CheckTouch` (`IEF:363-500`) — where the piercing parameter actually comes from

This is the heart of "how the piercing parameter is found for a curved face".

```
(aTF, aTL) = cp.Range1()
aCR = myC.Resolution(myCriteria)
if |aTF - myRange.First()| < aCR and |aTL - myRange.Last()| < aCR: return false   # IEF:372-377
Tol = PConfusion() = 1e-9
Curve   = BRep_Tool::Curve(edge)            trimmed to (aTF, aTL)      # IEF:382, 390
Surface = BRep_Tool::Surface(face)          on the ADAPTOR's UV bounds # IEF:383-391
anExtrema = Extrema_ExtCS(TheCurve, TheSurface, Tol, Tol)              # IEF:393
aDist2 = 1e100
if anExtrema.IsDone():
    if anExtrema.IsParallel(): return false        # IEF:449-452  <-- GIVE-UP, documented
    if NbExt() > 0:
        pick the extremum with the smallest square distance            # IEF:405-422
        aDist2 = that square distance ; aTx = its parameter on the curve
    else:
        # fallback: exact curve/surface intersection                   # IEF:426-446
        anExactIntersector = IntCurveSurface_HInter
        anExactIntersector.Perform(GeomAdaptor_Curve(TheCurve), GeomAdaptor_Surface(TheSurface))
        for each intersection point with aTF <= W <= aTL:
            aDist2 = 0 ; aTx = W                   # last one wins — no ordering
# then three explicit candidate parameters, each tested against aDist2:
for aP in {aTF, aTL, (aTF+aTL)/2}:                                     # IEF:455-477
    d = DistanceFunction(aP) + myCriteria
    if d*d < aDist2: aDist2 = d*d ; aTx = aP
if aDist2 > myCriteria^2: return false                                 # IEF:479-482
if |aTx - aTF| < PConfusion(): return true                             # IEF:484-487
if |aTx - aTL| < PConfusion(): return true                             # IEF:489-492
if aTF < aTx < aTL:            return true                             # IEF:494-497
return false
```

`DistanceFunction(t)` (`IEF:194-235`) is `dist(C(t), F) - myCriteria`, computed by the cached
projector, with an analytic shortcut: `IsEqDistance` (`IEF:240-299`) returns the exact radius
when the point sits on the axis of a cylinder (`IEF:249-261`), on the axis of a cone
(`IEF:263-280`, `R = |P-apex|·tan(semiangle)`), or on the core circle of a torus
(`IEF:282-297`), each within `1e-7` of the degenerate locus. Without these, the projector
returns an arbitrary point of a whole circle of equidistant feet. **Port these three shortcuts
— they are the only thing that makes distance-to-quadric well-defined on the axis.**

If the projector fails, `DistanceFunction` sets `myErrorStatus = 4` and returns `99.`
(`IEF:224-228`) — a sentinel large value, not an exception.

#### 2.3.5 `CheckTouchVertex` (`IEF:689-784`)

A stricter variant used only in the Line×Cylinder and Circle×Plane post-corrections. Differences
from `CheckTouch`:

- baseline distance is taken at the range midpoint only (`IEF:706-708`);
- the surface is trimmed to the **natural** surface bounds `Surface->Bounds(...)` (`IEF:715`),
  not the face's adaptor bounds — a real inconsistency with `CheckTouch:385-388`, preserved
  here because behaviour depends on it;
- `IsParallel` or `NbExt()==0` → return false (`IEF:722-735`), no exact-intersector fallback;
- if the best extremum is *farther* than the midpoint distance, the answer is the midpoint
  (`IEF:751-755`);
- end-exclusion uses `aEpsT = 8.e-5` (`9.e-5` for a Line) rather than `PConfusion()`
  (`IEF:700-704`, `IEF:768-776`) — three to four orders of magnitude looser.

#### 2.3.6 The two analytic post-corrections (`IEF:609-683`)

- **Line × Cylinder** (`IEF:620-646`): for each common part typed EDGE, run `CheckTouch`; if it
  reports a touch, demote to VERTEX at `aTx`. For each typed VERTEX, run `CheckTouchVertex` and
  refine the vertex parameter. Reason: a line tangent to a cylinder produces a long
  near-coincident range that `MakeType` would call an EDGE.
- **Circle × Plane** (`IEF:650-682`): same treatment, but only when the circle is **neither**
  coplanar with the plane (`IsCoplanar`, `IEF:788-811`, axis directions coincide within
  `IntTools_Tools::IsDirsCoinside`) **nor** at radius distance (`IsRadius`, `IEF:815-840`,
  `|dist(centre, plane) - R| < myCriteria`, i.e. the tangency configuration).

No other surface type gets a post-correction. Cone, sphere, torus and all free-form surfaces
rely entirely on `MakeType` + `CheckTouch`.

---

### 2.4 The engine: `IntTools_BeanFaceIntersector::Perform` (`BFI:288-379`)

This is the algorithm that produces the parameter *ranges* on the edge where the edge is within
`myCriteria` of the face. Its result is a `NCollection_Sequence<IntTools_Range>`.

```
Perform():
  if curve is Line and surface is Plane: ComputeLinePlane(); return        # BFI:299-303
  if FastComputeAnalytic():             myIsDone = true; return           # BFI:306-311
  myRangeManager.SetBoundaries(t1, t2, flag=0)                            # BFI:314
  if TestComputeCoinside():                                                # BFI:317-323
      result = [ (t1, t2) ]; done; return
  bLocalize = all four UV bounds finite                                    # BFI:328-330
           && (surface is Bezier | Other | (BSpline with degree>2 in u or v
                                            and >2 knots in both))         # BFI:331-336
  isLocalized = bLocalize && ComputeLocalized()                            # BFI:338
  if !isLocalized:
      ComputeAroundExactIntersection()                                     # BFI:343
      ComputeUsingExtremum()                                               # BFI:345
      ComputeNearRangeBoundaries()                                         # BFI:347
  myIsDone = true
  # harvest: every range with flag == 2, merging adjacent ones             # BFI:353-378
```

The **marked range set** (`MRS`) is a partition of `[t1,t2]` with an integer flag per cell:
`0` = unknown, `1` = proved empty, `2` = solution. `InsertRange(a,b,flag)` splits cells at `a`
and `b` and stamps the covered cells (`MRS:65-124`). Harvest keeps flag-2 cells, merging cells
whose endpoints agree within `PConfusion()` (`BFI:363-372`).

#### 2.4.1 `ComputeLinePlane` (`BFI:820-906`) — the exact planar case

```
Tolang = 1.e-9                                          # BFI:822
plane coefficients (A,B,C,D); line origin O, direction (Al,Bl,Cl)
Direc = A*Al + B*Bl + C*Cl ; Dis = A*Ox + B*Oy + C*Oz + D
if |Direc| < Tolang:  parallel; inplane = |Dis| < myCriteria             # BFI:840-844
else: inplane = (|d(P(t1))| <= myCriteria and |d(P(t2))| <= myCriteria)  # BFI:846-863
if inplane: result = [(t1,t2)]; return                                   # BFI:865-870
if parallel: return (empty)                                              # BFI:872-875
t = -Dis / Direc
if t outside [t1,t2]: return                                             # BFI:878-881
(u,v) = ElSLib::Parameters(plane, P(t))
if (u,v) outside the UV rectangle: return                                # BFI:886-890
anAngle = |pi/2 - angle(line dir, plane normal)|                         # BFI:898
aDt = IntTools_Tools::ComputeIntRange(tolE, tolF, anAngle)               # BFI:900
result = [ (max(t1, t-aDt), min(t2, t+aDt)) ]                            # BFI:902-905
```

`ComputeIntRange` (`TOOLS:783-804`) is the **grazing-angle range widening** and is essential:

```
if |pi/2 - theta| < Angular(): aDt = tol2
else:
    a = (theta > pi/2) ? (pi - theta) : theta
    aDt = tol1 * tan(pi/2 - a) + tol2 / sin(a)
```

At a near-tangential crossing (`a → 0`) this blows up as `tol1/a + tol2/a`, correctly widening
the intersection *interval* on the edge to cover everything within tolerance of the plane. A
port that returns a bare point here will produce spurious extra vertices on grazing cuts.

#### 2.4.2 `FastComputeAnalytic` (`BFI:692-816`) — cheap coincidence / no-intersection proofs

Returns `true` only when **no further computation is needed** — i.e. either full coincidence
was proved (and the whole range was appended as the result, `BFI:810-813`) or intersection was
proved impossible.

- Immediately `false` for Bezier, BSpline, Offset, Other curves (`BFI:695-700`) — **documented
  give-up, falls through to the generic path.**
- **Plane × (Circle | Ellipse | Hyperbola | Parabola)** (`BFI:708-750`): if the conic's axis is
  not parallel to the plane normal within `Precision::Angular()`, return `false`. Otherwise the
  conic is *in a parallel plane*: `hasIntersection = false`, and `isCoincide = dist(conic
  location, plane) < myCriteria`.
- **Cylinder × Line** (`BFI:760-772`): if the line is not parallel to the axis within
  `Angular()`, return `false`. Else `isCoincide = | dist(line, axis point) - R | < myCriteria`
  and `hasIntersection = false`.
- **Cylinder × Circle** (`BFI:774-793`): axes must be parallel within `Angular()`. Then
  `aDistLoc = dist(cyl axis line, circle centre)`,
  `isCoincide = (aDistLoc + |Rc - Rcyl|) < myCriteria`; if not coincident,
  `hasIntersection = (aDistLoc - (Rc+Rcyl)) < myCriteria && (|Rc-Rcyl| - aDistLoc) < myCriteria`.
- **Sphere × Line** (`BFI:797-807`): `hasIntersection = (dist(line, centre) - R) < myCriteria`.
  Never proves coincidence.
- Everything else falls through with `hasIntersection = true`, `isCoincide = false` → returns
  `false`.

**Coverage gap to record:** there is no fast path for Cone×anything, Torus×anything,
Sphere×Circle, or any free-form surface. Those always take the generic route.

#### 2.4.3 `TestComputeCoinside` (`BFI:2129-2176+`)

```
nbSeg = 23
if Distance(t1, out U, out V) > myCriteria: return false
ComputeRangeFromStartPoint(increasing=true, t1, U, V)
if the cell containing t2 already has flag 2: return true
if Distance(t2, U, V) > myCriteria: return false
ComputeRangeFromStartPoint(increasing=false, t2, U, V)
for i in 1..nbSeg-1:
    if Distance(t1 + i*dT, U, V) > myCriteria: return false
    ...
```

Unlike `IsCoincident`, this one requires **all** samples within tolerance (no majority vote) and
does no UV classification at all — it is purely a distance test against the *untrimmed*
restricted surface. The trim check is applied later by `IsProjectable` in `IEF:586`.

#### 2.4.4 `Distance(t)` and `Distance(t, out u, out v)` (`BFI:397-560`)

Primary: `ProjPS(face).Perform(C(t))` → `LowerDistance()` (`BFI:401-407`, `BFI:477-485`).

**Documented fallback when the projector fails or returns nothing** (`BFI:409-459`): for each
of the four iso-curves at `Umin, Umax, Vmin, Vmax`, if the iso is not degenerate (its three
sample points are not all equal within `myCriteria`, `BFI:429-433`), project the point onto the
iso-curve (`GeomAPI_ProjectPointOnCurve`, `BFI:437-449`) and keep the minimum; otherwise fall
back to the distance to the iso's two corner points (`BFI:452-458`). The returned `(u,v)` in the
second overload is finally clamped into the UV rectangle (`BFI:554-557`).

This fallback is an **approximation**: it measures distance to the boundary of the UV patch, not
to the patch interior. It exists so the marching loop always has a finite number to work with.

#### 2.4.5 `ComputeAroundExactIntersection` (`BFI:564-688`) — the exact seed stage

```
anExactIntersector = IntCurveSurface_HInter
anExactIntersector.Perform(BRepAdaptor_Curve(myCurve), BRepAdaptor_Surface(mySurface))  # BFI:571
if IsDone():
    if NbPoints() > 1:
        myCriteria = 3 * Precision::Confusion()      # = 3e-7        # BFI:582
        myCurveResolution = myCurve.Resolution(myCriteria)           # BFI:583
        # rationale (BFI:579-581): keep distinct intersection points from
        # merging into one range
    for each point P:
        if P.W() outside [t1, t2]: skip                              # BFI:590
        U = P.U(); V = P.V()
        if U outside [Umin,Umax]:
            if surface is U-periodic: GeomInt::AdjustPeriodic(...)   # BFI:607-624
            else: solution invalid, skip
        likewise for V                                               # BFI:627-648
        n0 = myRangeManager.Length()
        ComputeRangeFromStartPoint(false, P.W(), U, V)               # BFI:658
        ComputeRangeFromStartPoint(true,  P.W(), U, V)               # BFI:659
        if no new cell was created: SetEmptyResultRange(P.W(), mgr)  # BFI:661-664
             # -> insert a ZERO-LENGTH range [W,W] with flag 2       # MRS / BFI:1344-1365
        else: myMinSqDistance = 0
    for each segment S (a 1-D family of intersections):              # BFI:672-686
        InsertRange(max(t1,S.P1.W()), min(t2,S.P2.W()), flag 2)
        ComputeRangeFromStartPoint(false, P1.W(), P1.U(), P1.V())
        ComputeRangeFromStartPoint(true,  P2.W(), P2.U(), P2.V())
```

`SetEmptyResultRange` (`BFI:1344-1365`) is how a **transversal piercing at a single point**
survives: the marching found no neighbourhood within tolerance, so a degenerate range `[W,W]`
is stamped with flag 2, provided no overlapping cell already has flag 2. That degenerate range
is what `MakeType` later turns into a VERTEX common part.

#### 2.4.6 `ComputeRangeFromStartPoint` (`BFI:1150-1340`) — the tolerance-tube march

Given a known point `(t, u, v)` within tolerance, walk in one direction as far as the curve
stays within `myCriteria` of the surface.

```
aMinDelta = min(myCurveResolution*0.5, 0.5 * 0.1*(t2-t1))   # BFI:1189-1195
aDelta    = myCurveResolution                               # BFI:1198
aCurPar   = t ± aDelta   (clamped into the current cell)    # BFI:1200-1211
loopcounter = 0
while aDelta >= aMinDelta and loopcounter <= 10:            # BFI:1221
    P = C(aCurPar)
    anExtrema = Extrema_GenLocateExtPS(mySurface, 1e-10, 1e-10)   # BFI:1227
    anExtrema.Perform(P, U, V)                                    # BFI:1228   warm start!
    if IsDone() and SquareDistance() < myCriteria^2:
        (U,V) = anExtrema.Point().Parameter(); pointfound = true
    elif !IsDone():
        pointfound = (Distance(aCurPar) < myCriteria)             # BFI:1241
    if pointfound: aPrevPar = aCurPar ; anotherSolutionFound = true
    else:          aDeltaRestrictor = aDelta
    aDelta = pointfound ? aDelta*2 : aDelta*0.5                   # BFI:1262
    aDelta = min(aDelta, aDeltaRestrictor)
    aCurPar = aPrevPar ± aDelta
    if aCurPar == aPrevPar: break              # BFI:1270-1273 (underflow guard)
    ... cell-boundary bookkeeping, may advance aValidIndex ...    # BFI:1275-1326
if anotherSolutionFound: InsertRange(t, aPrevPar, flag 2)   (ordered)   # BFI:1329-1339
```

**The warm-started local extremum `Extrema_GenLocateExtPS(surface, 1e-10, 1e-10)` seeded from
the previous `(U,V)` is the whole trick.** It is a local Newton on
`min ||C(t) - S(u,v)||²` in `(u,v)` with `t` fixed. It converges in a few iterations because the
seed is the previous step's answer. Everything else is step-doubling with a restrictor.

#### 2.4.7 `ComputeUsingExtremum` (`BFI:910-1081`) — filling the unknown cells

For each cell still flagged `0`:

- degenerate cell (`< PConfusion()` long) adjacent to a flag-2 cell → flag it `1` and continue
  (`BFI:928-937`);
- build `Extrema_ExtCS` over that parameter sub-range and the full UV rectangle, tolerance
  `PConfusion()` both ways (`BFI:939-954`);
- **parallel branch** (`BFI:960-1039`): `SquareDistance(1)` is the constant distance. If it is
  `< myCriteria²`:
  - both ends within criteria → stamp the whole cell flag 2 (`BFI:972-977`);
  - one end within → march from that end (`BFI:979-988`);
  - neither end within → **bisection search** for any interior point within criteria
    (`BFI:991-1020`), loop condition `(b - a) > myCurveResolution`, moving the endpoint with
    the *smaller* distance inward. If found, march both ways; **if not found, flag the cell `1`
    (empty)** — a documented give-up (`BFI:1027-1030`).
  - if the parallel distance is `≥ myCriteria` → flag `1` (`BFI:1035-1038`).
- **non-parallel branch** (`BFI:1040-1072`): for each extremum with
  `SquareDistance < myCriteria²`, march both ways from it; if none qualified, flag the cell `1`.
- After either branch, if the range manager grew, skip the newly inserted cells
  (`BFI:1073-1078`).

#### 2.4.8 `ComputeNearRangeBoundaries` (`BFI:1085-1142`)

Final sweep: for each remaining flag-0 cell whose predecessor is also flag-0, if
`Distance(cell.First()) < myCriteria`, march both ways from `cell.First()`; if nothing was
inserted, stamp the degenerate range. Then the same for the very last cell's `Last()`
(`BFI:1126-1141`). This catches contacts that begin exactly at a cell boundary, which the
extremum pass can miss.

#### 2.4.9 `ComputeLocalized` / `LocalizeSolutions` (`BFI:1369-2125`)

The recursive box-subdivision path, used only for Bezier / Other / high-degree BSpline surfaces
(gate at `BFI:328-336`). Sketch, since it is a performance path rather than a correctness one:

- surface grid data cached per face in `IntTools_Context::SurfaceData` (`CTX:418-433`),
  constructed as `IntTools_SurfaceRangeLocalizeData(3, 3, 10*PConfusion(), 10*PConfusion())` —
  3×3 sampling, minimum cell `1e-8` in each UV direction;
- `LocalizeSolutions` recursively splits the curve range into 3 and the UV rectangle into 3×3
  (`BFI:1488-1490`), rejecting pairs by bounding box (`BFI:1562-1567`, `BFI:1572-1576`,
  `BFI:1646-1658`), refusing to split further when the requested cell would fall below the
  minimum (`CheckSampling`, `BFI:1429-1446`), and balancing subdivision so a box 10× larger than
  its partner is split first (`BFI:1704-1734`);
- surviving (curve-cell, surface-cell) pairs are merged (`MergeSolutions`, `BFI:1939`) and each
  is solved with `Extrema_GenExtCS` initialised with a 10×10 sampling grid (`BFI:2033-2034`);
- each extremum within criteria is periodic-adjusted, clamped into the UV rectangle
  (`BFI:2054-2084`), and used as a march start point (`BFI:2087-2093`);
- cells with no extremum are re-inserted with flag `0` (`BFI:2099`), and curve ranges proved out
  by boxes are flagged `1` (`BFI:2110-2119`);
- finally `ComputeNearRangeBoundaries()` runs (`BFI:2120`).

`LocalizeSolutions` can return `false` (`BFI:1749-1752`, `BFI:1762-1764`, `BFI:1785-1787`),
which makes `ComputeLocalized` return `false` (`BFI:1932-1934`) and **the whole generic
three-stage path runs instead** (`BFI:341-348`). A documented fallback.

---

### 2.5 The underlying curve/surface intersection: `IntCurveSurface_HInter`

Used by `ComputeAroundExactIntersection` (`BFI:566-571`) and by `CheckTouch`'s fallback
(`IEF:426-431`). The template implementation is `ICS` + `ICSU`.

#### 2.5.1 Top-level decomposition (`ICS:69-86`)

`Perform` decomposes the **surface** into `C2` intervals in U and V
(`DecomposeSurfaceIntervals`, `ICSU:1549-1610`) and calls `PerformBounds` on each rectangle.
This is why a multi-patch BSpline surface is intersected patch by patch.

#### 2.5.2 Dispatch by curve type (`ICS:105-182`)

- UV parameters clamped to `±1e50` first (`ICS:117`, `ICSU:1614-1633`).
- Line / Circle / Ellipse / Parabola / Hyperbola → `PerformConicSurf*` (`ICS:123-137`).
- Anything else (BSpline, Bezier, Offset, Other):
  - if the surface is **not** Plane/Cylinder/Cone/Sphere: split the curve into `C2` intervals
    (`ICS:139-147`); for each, sample it with `CurveTool::SamplePars(curve, u1, u2, defl=0.1,
    NbMin=10)` (`ICS:153-157`) into a `Polygon`, and call the polygon/polyhedron path;
  - if the surface **is** a quadric: `theInternalPerformQuadric` → `PerformCurveQuadric`
    (`ICS:178`, `ICSU:1243-1274`), which uses the exact `IntCurveSurface_QuadCurvExactHInter`
    root finder and reports `NbRoots()` parameters directly.

#### 2.5.3 Conic × quadric — fully analytic (`ICS:532-966`)

| curve | surface | solver | site |
|---|---|---|---|
| Line | Plane | `IntAna_IntConicQuad(line, plane, 1e-12)` | `ICS:549-553` |
| Line | Cylinder | `IntAna_IntConicQuad(line, cyl)` | `ICS:555-559` |
| Line | Sphere | `IntAna_IntConicQuad(line, sph)` | `ICS:560-564` |
| Line | Torus | `IntAna_IntLinTorus` (`ICSU:1283-1315`) | `ICS:565-583` |
| Line | Cone | `IntAna_IntConicQuad(line, cone)` **only if** `|semiangle| < pi/2 - 1e5·Angular()` | `ICS:584-597` |
| Circle/Ellipse | Plane/Cylinder/Cone/Sphere | `IntAna_IntConicQuad` | `ICS:731-753`, `ICS:782-804` |
| Parabola/Hyperbola | Plane/Cylinder/Cone/Sphere | `IntAna_IntConicQuad` | `ICS:834-855`, `ICS:912-933` |

Any case not in this table sets `isAnaProcessed = false` (`ICS:598-600`) and falls to the
polyhedron path. **A degenerate cone (semi-angle at or beyond 90°) and a failed line/torus
solve are the two documented analytic give-ups.**

For the line fallback the surface is sampled into a polyhedron with `nbsu,nbsv` forced to at
least 20 (`ICS:681-688`), the line is clipped against the polyhedron's bounding box by
`Intf_Tool::LinBox` (`ICS:691-693`), and each resulting segment is polygonised with 2 points
(`ICS:704`). Infinite-domain surfaces get their limits estimated first — extrusion
(`ICSU:148-371`), revolution (`ICSU:376-494`), offset (`ICSU:499-734`), and a generic clamp to
`±1e10` otherwise (`ICSU:137-143`).

#### 2.5.4 Analytic result post-processing (`ProcessIntAna`, `ICSU:1188-1228`)

```
if !theIntAna.IsDone(): return false
if IsInQuadric() or IsParallel(): theIsParallel = true; return true   # ICSU:1202-1206
for i in 1..NbPoints():
    P = Point(i) ; w = ParamOnConic(i)
    (u,v) = ComputeParamsOnQuadric(surface, P)      # ElSLib::Parameters, ICSU:900-925
    if ComputeAppendPoint(curve, w, surface, u, v, out pt): append pt
```

`AppendIntAna` (`ICS:974-1002`) turns `theIsParallel` into a flag on the intersector rather than
a point list — i.e. **the "curve lies in the quadric" case is reported, not enumerated.**

#### 2.5.5 `ComputeAppendPoint` (`ICSU:1116-1176`) — the acceptance gate

```
W0,W1 = curve first/last ; U0,U1,V0,V1 = surface bounds
if curve is periodic or Circle or Ellipse: w = ElCLib::InPeriod(w, W0, W0+Period)  # ICSU:1135-1138
if (W0 - w) >= 1e-8 or (w - W1) >= 1e-8: reject                                    # ICSU:1140-1143
if surface U-periodic or Cylinder/Cone/Sphere: u = InPeriod(u, U0, U0+UPeriod)     # ICSU:1146-1150
if surface V-periodic: v = InPeriod(v, V0, V0+VPeriod)                             # ICSU:1152-1155
if (U0 - u) >= 1e-8 or (u - U1) >= 1e-8: reject                                    # ICSU:1157-1160
if (V0 - v) >= 1e-8 or (v - V1) >= 1e-8: reject                                    # ICSU:1161-1164
ComputeTransitions(...)                                                            # ICSU:1167
point = IntCurveSurface_IntersectionPoint(C(w), u, v, w, transition)
```

The reject threshold is `THE_TOLTANGENCY = 1e-8` in **parameter** units, applied uniformly to
both curve parameter and both surface parameters — one of the very few places OCCT uses a bare
parametric constant. Note it is a *one-sided* comparison, so a point up to `1e-8` outside a
bound is accepted and reported with its out-of-range parameter unchanged.

#### 2.5.6 `ComputeTransitions` (`ICSU:855-895`) — transversal vs tangential

```
S.D1(u,v) -> Psurf, D1U, D1V ; NSurf = D1U × D1V
C.D1(w)   -> P, T
Norm = |NSurf|
if Norm > 1e-12 and |T|² > 1e-12:
    T.Normalize() ; CosDir = (NSurf · T) / Norm
    if -CosDir >  1e-12: transition = IntCurveSurface_In
    elif CosDir >  1e-12: transition = IntCurveSurface_Out
    else:                 transition = IntCurveSurface_Tangent
else:                     transition = IntCurveSurface_Tangent
```

**This is the transversal/tangential discriminator**, and its threshold is
`THE_TOLERANCE_ANGULAIRE = 1e-12` on the *normalised* dot product — effectively "any non-zero
crossing angle counts as transversal".

**Important, and easy to get wrong:** the transition is stored on the
`IntCurveSurface_IntersectionPoint` but **`IntTools_BeanFaceIntersector` never reads it**
(`BFI:586-668` uses only `W()`, `U()`, `V()`). Consequently `IntTools_EdgeFace` never sees it
either, and `PF5` emits **the same records for a transversal piercing and a tangential touch**.
The distinction survives only in the *shape* of the common part: a transversal piercing gives a
degenerate range `[W,W]` (via `SetEmptyResultRange`), while a tangential touch usually gives a
short non-degenerate range whose midpoint `CheckTouch` relocates to the true extremum. Our port
should keep the transition on the record (it is free, and the builder can use it), but must not
make emission depend on it, or behaviour will diverge from OCCT.

#### 2.5.7 Start points from the polygon/polyhedron interference (`ICSU:1345-1519`)

For non-analytic pairs:

1. `CollectInterferencePoints` (`ICSU:1345-1376`): every `Intf_SectionPoint` of the
   polygon/polyhedron interference, plus every point of every `Intf_TangentZone`, is converted to
   `(u, v, w)` by `SectionPointToParameters` (`ICSU:739-848`). The surface parameters come from
   barycentric interpolation inside the hit triangle (`ICSU:773-796`), with a degenerate-triangle
   fallback that projects onto the longest edge (`ICSU:797-832`). The curve parameter is
   `Polygon.ApproxParamOnCurve(SegIndex, param)` (`ICSU:845`).
2. `SortStartPoints` (`ICSU:1380-1447`): bubble sort by `W`, then `U`, then `V`, snapping values
   within `ptol = 10·PConfusion() = 1e-8` of the previous one to be exactly equal.
3. `ProcessSortedPoints` (`ICSU:1460-1519`): for each start point that differs from the previous
   accepted one by more than `ptol` in any of `u,v,w` (`ICSU:1495`), run
   `theExactInter.Perform(u, v, w, rsnld, U0, U1, V0, V1, Winf, Wsup)` — a
   `math_FunctionSetRoot` Newton on the 3-equation system `C(w) - S(u,v) = 0` with tolerance
   `THE_TOLTANGENCY = 1e-8` (`ICS:321`, `ICS:380`) — and, on success, take
   `ParameterOnCurve()` / `ParameterOnSurface()` and run `ComputeAppendPoint`.

**Give-up to record:** the `ptol` dedup at `ICSU:1495` compares against the *previous* sorted
point only. Two genuinely distinct intersections closer than `1e-8` in all three parameters are
collapsed to one. That is by design (they cannot be resolved), but it is a real limit.

---

### 2.6 Stage 5 — EF: consuming the common parts (`PF5:324-571`)

For each finished `BOPAlgo_EdgeFace`:

- `!IsDone() || HasErrors()` → warning only, continue (`PF5:331-336`).
- **No common parts** (`PF5:348-362`): if `MinimalDistance()` is finite and
  `> tol(E) + tol(F)`, record `(t1, t2, distance)` in `myDistances[(nE,nF)]`. This cache is used
  much later by `ProcessExistingPaveBlocks` to decide whether growing tolerances have brought an
  edge into contact with a face. **Port it; it is cheap and it is the mechanism that recovers
  contacts created by later tolerance growth.**
- Otherwise: read `anewSR` (the corrected shrunk range), the PB range `(aT1,aT2)`, the PB's
  vertices `nV[0], nV[1]`, and `bIsPBSplittable` (`PF5:364-371`).
- If the **first** common part is of type VERTEX, call `ReduceIntersectionRange(nV0, nV1, nE,
  nF, aTS1, aTS2)` (`PF5:373-380`).
- Define the two **end ranges** `aR1 = (aT1, aTS1)` and `aR2 = (aTS2, aT2)` (`PF5:382`) — the
  parts of the pave block covered by its end vertices' tolerance spheres.
- `bLinePlane = (curve is Line) && (surface is Plane)` (`PF5:388-394`).

#### 2.6.1 `ReduceIntersectionRange` (`PF5:685-768`)

Purpose: stop EF from creating a vertex right next to a vertex that stage 3 (EE) already
created.

```
if neither theV1 nor theV2 is a NEW shape: return                # PF5:692-695
if !HasInterfShapeSubShapes(theE, theF): return                  # PF5:697-700
if no EE interferences at all: return                            # PF5:702-707
aMFE = { edge sub-shapes of theF }                               # PF5:713-723
for each BOPDS_InterfEE with an IndexNew:
    nV = IndexNew ; skip unless nV == theV1 or theV2             # PF5:733-738
    (nE1, nE2) = the EE pair
    skip unless theE is one of them AND the other is in aMFE     # PF5:742-746
    (aTR1, aTR2) = the EE common part's range on theE            # PF5:749-751
    if nV == theV1: theTS1 = max(theTS1, aTR2)                   # PF5:753-758
    else:           theTS2 = min(theTS2, aTR1)
```

I.e. the working range is pulled inward past the parameter span already consumed by an
edge/edge crossing with one of the face's own edges.

#### 2.6.2 Common part of type VERTEX (`PF5:406-544`)

```
IntTools_Tools::VertexParameter(aCPart, aT)                      # PF5:412 ; TOOLS:615-623
    #   aT = 0.5*(R1.First()+R1.Last()), overridden by VertexParameter1
    #   if that value lies inside R1
BOPTools_AlgoTools::MakeNewVertex(aE, aT, aF, aVnew)             # PF5:413 ; AT2:254-271
    #   point   = C(aT)
    #   tolerance = tol(E) + tol(F) + DTolerance() (= 1e-12)

aR = aCPart.Range1()
aTolToDecide = 5.e-8                                             # PF5:416
bIsOnPave[0] = IntTools_Tools::IsInRange(aR1, aR, aTolToDecide)  # PF5:418 ; TOOLS:650-666
bIsOnPave[1] = IntTools_Tools::IsInRange(aR2, aR, aTolToDecide)  # PF5:419
    #   IsInRange(ref, r, tol) = (r.First() in [ref.First()-tol, ref.Last()+tol])
    #                         || (r.Last()  in the same widened interval)

# --- CASE A: the common part touches BOTH end ranges (or, for Line/Plane, either) ---
if (bIsOnPave[0] && bIsOnPave[1]) || (bLinePlane && (bIsOnPave[0] || bIsOnPave[1])):   # PF5:421
    if both PB vertices are already in FaceInfo.VerticesOn ∪ VerticesIn:               # PF5:423-425
        # the whole pave block is effectively ON the face
        aCP = aCPart with type forced to EDGE                                          # PF5:427-428
        append InterfEF(nE,nF) with that common part ; AddInterf                       # PF5:429-433
        aMIEFC.Add(nF)                                                                 # PF5:435
        BOPAlgo_Tools::FillMap(aPB, nF, aMPBLI, alloc)   # queue for common-block       # PF5:437
        break   # done with this common part

# --- CASE B: not splittable ---
if !bIsPBSplittable: continue                                                          # PF5:442-445

# --- CASE C: the point sits on one of the ends ---
for j in 0,1:
    if bIsOnPave[j] and vertex nV[j] not already known to the face:
        bIsOnPave[j] = ForceInterfVF(nV[j], nF)          # §2.1.6                      # PF5:447-457
if bIsOnPave[0] || bIsOnPave[1]:
    # Do NOT create a new vertex here. Decide whether the contact is real.
    aProjPS = ProjPS(aF) ; aProjPS.Perform(Pnt(aVnew))                                 # PF5:467-469
    aMinDistEF = IsDone && NbPoints ? LowerDistance() : Infinite                       # PF5:470-471
    if aMinDistEF >= Precision::Intersection()   (= 1e-9): continue   # touching only   # PF5:472-478
    for j in 0,1 with bIsOnPave[j]:                                                    # PF5:482-501
        aDistPP = |Pnt(nV[j]) - Pnt(aVnew)|
        aTol    = tol(nV[j])
        aMaxDist = 1.e4 * aTol ; if aTol < .01: aMaxDist = min(aMaxDist, 0.1)
        if aDistPP < aMaxDist:
            UpdateVertex(nV[j], aDistPP)          # grow the EXISTING vertex
            myVertsToAvoidExtension.Add(nV[j])
    continue

# --- CASE D: a genuine interior piercing ---
if CheckFacePaves(aVnew, FaceInfo(nF).VerticesOn()): continue    # PF5:505-508
    #   CheckFacePaves(V, map) returns TRUE (=> skip) if V coincides
    #   (BOPTools_AlgoTools::ComputeVV) with ANY vertex in the map   # PF5:605-627
aTolVnew = max(tol(aVnew), max(tol(E), tol(F)))                  # PF5:510-511
BRep_Builder().UpdateVertex(aVnew, aTolVnew)                     # PF5:512
if bLinePlane:                                                   # PF5:513-519
    aTolVnew = max(aTolVnew, (aCR.Last() - aCR.First()) / 2)
    # NOTE: applied to the local variable only, NOT written into aVnew --
    # the comment at PF5:514-515 says the vertex is not updated until it
    # intersects something else.
if !myContext->IsPointInFace(Pnt(aVnew), aF, aTolVnew): continue # PF5:523-526
    #   the gp_Pnt overload, CTX:612-635: project, require dist < aTolVnew,
    #   then STRICT IsPointInFace (state != OUT && != ON)  -- invariant G6
aMIEFC.Add(nF)                                                   # PF5:528
append InterfEF(nE, nF) with the common part ; iX = its index     # PF5:530-533
AddInterf(nE, nF)                                                # PF5:535
aCPB = CoupleOfPaveBlocks(aPB, aPB) ; SetIndexInterf(iX) ; SetTolerance(aTolVnew)  # PF5:537-541
aMVCPB.Add(aVnew, aCPB)                                          # PF5:542
```

`ComputeVV(V1, V2)` (`TOOLS:354-372`) returns `0` (coincident) when
`|P1-P2|² < (tol1+tol2)²`. Note the missing square on the right in the *sum* — OCCT squares the
sum, so the test is `dist < tol1 + tol2`. Correct as written.

#### 2.6.3 Common part of type EDGE (`PF5:545-566`)

```
aMIEFC.Add(nF)
append InterfEF(nE, nF)                                          # PF5:549-551
bV[0] = CheckFacePaves(nV[0], VerticesOn, VerticesIn)            # PF5:553
bV[1] = CheckFacePaves(nV[1], VerticesOn, VerticesIn)            # PF5:554
if !bV[0] || !bV[1]:
    AddInterf(nE, nF); break     # record the interference, but NO common block  # PF5:555-559
aEF.SetCommonPart(aCPart)
AddInterf(nE, nF)
BOPAlgo_Tools::FillMap(aPB, nF, aMPBLI, alloc)                   # PF5:564
```

So an edge that coincides with a face becomes a **common block with that face** only if both of
its bounding vertices are already known to be on/in the face. Otherwise the interference is
recorded but the pave block is not shared — the contact will be revisited by `ForceInterfEF`
after tolerances have grown.

#### 2.6.4 Post-treatment (`PF5:573-591`)

```
BOPAlgo_Tools::PerformCommonBlocks(aMPBLI, alloc, myDS, myContext)   # PF5:576
UpdateVerticesOfCB()                                                  # PF5:577
PerformNewVertices(aMVCPB, alloc, range, /*bIsEEIntersection=*/false) # PF5:578
myDS->UpdateFaceInfoIn(aMIEFC)                                        # PF5:585
```

`PerformNewVertices` (`PF3:594-688`) is where the paves are actually inserted:

1. `TreatNewVertices` (`PF3:692-723`) fuses the queued new vertices:
   `BOPAlgo_Tools::IntersectVertices(verts, myFuzzyValue, chains)` groups vertices whose
   tolerance spheres overlap, and `BOPTools_AlgoTools::MakeVertex(chain, aVNew)` builds one
   vertex per chain. **This is invariant G2 in OCCT's implementation**: the A-side and B-side
   piercings at the same location fuse here because both are queued into the same `aMVCPB`.
2. Each fused vertex is appended to the DS with box gap `tol(V) + fuzz/2` (`PF3:631-639`), and
   every source vertex's `CoupleOfPaveBlocks` gets `SetIndex(iV)` plus
   `aInt->SetIndexNew(iV)` on the corresponding `BOPDS_InterfEF` (`PF3:641-652`).
3. Build `aMPBLI : PaveBlock -> list of vertex indices` (`PF3:656-685`).
4. `IntersectVE(aMPBLI, range, /*bAddInterfs=*/false)` (`PF3:687`) — this projects each vertex
   onto its pave block's edge, creates the `BOPDS_Pave`, appends it as an *ext pave*, and
   **splits the pave block** (`SplitPaveBlocks`, declared `BOPAlgo_PaveFiller.hxx:209-210`).

`UpdateFaceInfoIn(aMIEFC)` (`DS:894-950`) rebuilds `VerticesIn` and `PaveBlocksIn` for every
touched face from scratch:
- clear both sets; re-add the face's own internal vertices (`InitFaceInfoIn`, `DS:751-769`);
- add `Index1` of every `InterfVF` whose `Index2` is the face, resolved through SD (`DS:914-922`);
- for every `InterfEF` on the face: if it has an `IndexNew`, add that vertex to `VerticesIn`;
  otherwise walk the edge's pave blocks and add every common block that contains the face to
  `PaveBlocksIn` (`DS:925-949`).

**That last rule is the link between EF and the splitter.** An EF interference contributes
either a vertex (piercing) or a pave block (coincidence) to the face's IN sets, never both.

#### 2.6.5 `ForceInterfEF` — the second pass (`PF5:772-1199`)

Runs after all vertices have been grown and unified (`myIsPrimary` guard at `PF5:775-778`), and
looks **only for coincidence** (`PF5:1141-1171` keeps only `TopAbs_EDGE` common parts).

1. Collect every real pave block of every non-degenerate edge (`PF5:787-822`).
2. Build a BVH of their shrunk bounding boxes (`PF5:843-874`).
3. For each face with `FaceInfo`, select overlapping pave blocks, and keep only those whose
   **both** vertices are already in the face's `VerticesOn ∪ VerticesIn ∪ VerticesSc` or are
   endpoints of the face's own pave blocks (`PF5:912-964`).
4. Skip pairs from the same operand unless the pave block has no edge yet (`PF5:966-981`).
5. Evaluate the edge at the midpoint of its shrunk range, project onto the face
   (`PF5:998-1012`), and require
   `LowerDistance <= aTolCheck + fuzz` where
   `aTolCheck = 2·max(tol(V1), tol(V2))` (or `fuzz` in self-interference mode) (`PF5:1016-1029`).
6. Require the foot to be strictly inside the face (`IsPointInFace(F, (U,V))`, `PF5:1033-1036`).
7. **Direction gate** (`PF5:1038-1052`): unless (plane, line), compute
   `aVFNorm = P_on_surface -> P_on_edge` and the curve tangent; if
   `|cos(angle)| > 0.4226` (≈ more than 25° away from perpendicular) set `bUseAddTol = false`.
   Rationale in the comment at `PF5:986-990`: without this, an edge merely passing near the face
   would be unified with it.
8. Additional fuzzy tolerance (`PF5:1054-1085`): the max of the two shrunk-range endpoint
   distances to the face that are below `aTolCheck`, minus `tol(E)+tol(F)`, floored at 0.
9. Intersect if `aTolAdd > 0` or the pair was not already done in `myFPBDone[nF]`
   (`PF5:1087-1092`), with `SetFuzzyValue(fuzz + aTolAdd)` and
   `UseQuickCoincidenceCheck(true)` (`PF5:1103-1105`).
10. For each result that is a single `TopAbs_EDGE` common part: append `InterfEF`, `AddInterf`,
    add the pave block to `FaceInfo(nF).ChangePaveBlocksIn()` (`PF5:1186`) and queue a common
    block (`PF5:1188-1191`).

---

## 3. DATA STRUCTURES AND C++ DECLARATIONS FOR OUR PORT

New files: `src/brep_interf_vf.h/.cpp`, `src/brep_interf_ef.h/.cpp`, plus
`src/brep_curvesurface.h/.cpp` for the curve/surface intersector. They depend on the arena from
`kb/ARCHITECTURE_v2.md` §1 (`src/brep_bds.h`). Names below use that arena's vocabulary.

```cpp
// ---------------------------------------------------------------- src/brep_curvesurface.h
namespace session_cpp {

/// Transition of a curve through a surface at an intersection point.
/// Port of IntCurveSurface_TransitionOnCurve (ICSU:855-895).
enum class CSTransition { In, Out, Tangent };

struct CurveSurfacePoint {
    double w = 0.0;              ///< parameter on the curve
    double u = 0.0, v = 0.0;     ///< parameters on the surface
    Point  p;                    ///< C(w)
    CSTransition transition = CSTransition::Tangent;
};

struct CurveSurfaceSegment {    ///< a 1-D family (curve lies in the surface over [w0,w1])
    double w0 = 0.0, w1 = 0.0;
    CurveSurfacePoint p0, p1;
};

struct CurveSurfaceResult {
    bool done = false;
    bool curve_in_surface = false;   ///< IntAna IsInQuadric / IsParallel  (ICSU:1202-1206)
    std::vector<CurveSurfacePoint>   points;
    std::vector<CurveSurfaceSegment> segments;
};

/// Exact curve x surface intersection. Port of IntCurveSurface_HInter.
/// - decomposes the surface into C2 UV intervals (ICSU:1549-1610);
/// - analytic branch for conic x quadric (ICS:532-966);
/// - polygon/polyhedron seeding + 3x3 Newton for everything else
///   (ICS:308-414, ICSU:1345-1519);
/// - acceptance gate ComputeAppendPoint with THE_TOLTANGENCY = 1e-8 (ICSU:1116-1176).
CurveSurfaceResult curve_surface(const NurbsCurve& c, double w0, double w1,
                                 const NurbsSurface& s,
                                 double umin, double umax, double vmin, double vmax);
} // namespace session_cpp
```

```cpp
// -------------------------------------------------------------------- src/brep_interf_ef.h
namespace session_cpp {

/// Port of IntTools_Range.
struct ParamRange { double first = 0.0, last = 0.0;
                    double length() const { return last - first; } };

/// Port of IntTools_CommonPrt, restricted to the edge/face case.
enum class CommonPartType { Vertex, Edge };

struct EFCommonPart {
    CommonPartType type = CommonPartType::Vertex;
    ParamRange range;            ///< always present, even for Vertex (IEF:355)
    double     vertex_param = 0.0;   ///< meaningful when type == Vertex
    Point      p_first, p_last;      ///< C(range.first), C(range.last)   (IEF:602-605)
    bool       all_null = false;     ///< AllNullFlag  (IEF:310-315)
};

/// Port of IntTools_MarkedRangeSet. Flags: 0 unknown, 1 proved empty, 2 solution.
class MarkedRangeSet {
public:
    void set_boundaries(double a, double b, int flag);           // MRS:35-45
    bool insert_range(double a, double b, int flag);             // MRS:65-124
    bool insert_range(double a, double b, int flag, int index);  // MRS:128-172
    void set_flag(int i, int f);                                 // MRS:181-184
    int  flag(int i) const;                                      // MRS:186-189
    int  length() const;
    ParamRange range(int i) const;
    int  get_index(double t, bool forward) const;
    std::vector<int> get_indices(double t) const;
private:
    std::vector<double> m_bounds;   // length n+1
    std::vector<int>    m_flags;    // length n
};

/// Port of IntTools_ShrunkRange (SHR:107-191). All tolerances are 3D.
struct ShrunkRange {
    double t1 = 0.0, t2 = 0.0;      ///< shrunk parameters
    double length = 0.0;            ///< arc length over [t1,t2]
    Aabb   box;
    bool   done = false;
    bool   splittable = false;      ///< length > 2*tol(E) + 2*CONFUSION   (SHR:184-187)
};
ShrunkRange compute_shrunk_range(const NurbsCurve& c, double t1, double t2,
                                 double tol_e, const Point& p1, double tol_v1,
                                 const Point& p2, double tol_v2);

/// Port of IntTools_BeanFaceIntersector. Produces parameter ranges on the edge where
/// the edge is within `criteria` of the (UV-restricted) surface.
class BeanFaceIntersector {
public:
    void set_curve(const NurbsCurve& c, double t1, double t2);
    void set_surface(const NurbsSurface& s, double umin, double umax,
                     double vmin, double vmax);
    void set_tolerances(double tol_bean, double tol_face);   ///< criteria = sum (BFI:222)
    void perform();                                          ///< BFI:288-379
    bool   is_done() const { return m_done; }
    double min_distance() const;                             ///< sqrt(min sq distance)
    const std::vector<ParamRange>& result() const { return m_results; }
private:
    // stages, in the order BFI:299-348 runs them
    void compute_line_plane();                               // BFI:820-906
    bool fast_compute_analytic();                            // BFI:692-816
    bool test_compute_coinside();                            // BFI:2129+
    bool compute_localized();                                // BFI:1819-2125
    void compute_around_exact_intersection();                // BFI:564-688
    void compute_using_extremum();                           // BFI:910-1081
    void compute_near_range_boundaries();                    // BFI:1085-1142
    void march_from(bool increasing, double t, double u, double v, int cell = -1);
                                                             // BFI:1150-1340
    double distance(double t) const;                         // BFI:397-461
    double distance(double t, double& u, double& v) const;   // BFI:465-560
    MarkedRangeSet m_mgr;
    std::vector<ParamRange> m_results;
    double m_criteria = 0.0, m_curve_resolution = 0.0, m_min_sq_distance = 1e300;
    bool   m_done = false;
};

/// Port of IntTools_EdgeFace.
class EdgeFaceIntersector {
public:
    void set_edge(int edge_index, const NurbsCurve& c);
    void set_face(int face_index, const BRepFaceRef& f);     ///< surface + trim loops
    void set_range(double t1, double t2);                    ///< the corrected PB range
    void set_fuzzy_value(double f);                          ///< floored at CONFUSION
    void use_quick_coincidence_check(bool b);
    void perform();                                          ///< IEF:504-685
    bool is_done() const;
    int  error_status() const;      ///< 0 ok, 1 not started, 2/3 bad data, 4 projection failed
    double minimal_distance() const;
    const std::vector<EFCommonPart>& common_parts() const;
private:
    bool is_coincident();                                    // IEF:62-163
    bool is_projectable(double t) const;                     // IEF:181-190
    double distance_function(double t);                      // IEF:194-235
    static bool is_eq_distance(const Point& p, const NurbsSurface& s,
                               double tol, double& d);       // IEF:240-299
    void make_type(EFCommonPart& cp);                        // IEF:304-359
    bool check_touch(const EFCommonPart& cp, double& tx);    // IEF:363-500
    bool check_touch_vertex(const EFCommonPart& cp, double& tx); // IEF:689-784
    double m_criteria = 0.0;
};
} // namespace session_cpp
```

```cpp
// -------------------------------------------------------------------- src/brep_interf_vf.h
namespace session_cpp {

/// Return codes of ComputeVF, identical to CTX:545-590.
enum class VFStatus : int {
    Ok          =  0,   ///< inside the trimmed face, within tolerance
    NoProjection= -1,   ///< projector failed                        CTX:561-564
    TooFar      = -2,   ///< distance > tolV + tolF + max(fuzz, 1e-7) CTX:577-581
    OutsideFace = -3    ///< projects OUT or exactly ON the boundary  CTX:583-588
};

struct VFResult { VFStatus status; double u, v; double new_vertex_tol; double distance; };

/// Port of IntTools_Context::ComputeVF (CTX:545-590).
/// `uv_box` is the FACE's adaptor UV rectangle, used to restrict the projector (CTX:253).
VFResult compute_vf(const Point& p, double tol_v,
                    const BRepFaceRef& f, double tol_f,
                    double fuzz);
} // namespace session_cpp
```

Arena additions required (extending `kb/ARCHITECTURE_v2.md` §1):

```cpp
struct BdsFaceInfo {                       // BOPDS_FaceInfo.hxx:33-138
    std::set<int> pave_blocks_in, pave_blocks_on, pave_blocks_sc;
    std::set<int> vertices_in,     vertices_on,   vertices_sc;
};

struct BdsInterfVF {                       // BOPDS_Interf.hxx:307-360
    int vertex = -1, face = -1;
    double u = 0, v = 0;
    int index_new = -1;                    // SetIndexNew / GetIndexNew
};

struct BdsInterfEF {                       // BOPDS_Interf.hxx:440-490
    int edge = -1, face = -1;
    EFCommonPart common_part;
    int index_new = -1;
};

/// PF5:354-359 — distance cache for pairs with no intersection, keyed by (edge, face).
struct EdgeRangeDistance { double first, last, distance; };
std::map<std::pair<int,int>, std::vector<EdgeRangeDistance>> distances;

/// PF5:300-305 — pairs already intersected, so ForceInterfEF does not redo them.
std::map<int, std::set<int>> fpb_done;      // face -> set of pave-block ids

/// PF5:498, PF10:154 — vertices already extended to swallow a nearby EE/EF point;
/// they must not be extended again to reach a section curve.
std::set<int> verts_to_avoid_extension;
std::set<int> increased_ss;                 // PF10:124, :158
```

**Tolerance rules for the port, restated to satisfy G7.** All of these are 3D distances:

```
tol_sum_vf   = tol(V) + tol(F) + max(fuzz, CONFUSION)                      // CTX:573
new_tol_vf   = dist + tol(F)                                               // CTX:574
criteria_ef  = tol(E) + tol(F) + fuzz                       (generic)      // IEF:548
             = 1.5*tol(E) + tol(F)                          (spline curve) // IEF:543
             = max(tol(E), tol(F))            (ratio > 100, spline curve)  // IEF:539
new_tol_ef   = max(tol(E), tol(F))                                         // IEF/PF5:510-511
```

The only parameter-space constants permitted are the ones OCCT itself uses as such, listed in
§2.0: `PConfusion() = 1e-9`, `aTolToDecide = 5e-8`, `aEpsT = 8e-5 / 9e-5`,
`THE_TOLTANGENCY = 1e-8`, and the `1e-10` extrema tolerances. **Every other tolerance is a 3D
distance converted through `BAC.Resolution(tol3d)` or `|C'(t)|` (`AT2:391-418`) or
`UResolution/VResolution` (FClass2d `:732-745`) at the point of use.**

---

## 4. WHAT OUR CODE DOES TODAY, AND WHERE IT DIVERGES

### 4.1 There is no curve/surface intersection primitive at all

`src/intersection.h` exposes `curve_plane` (`:364`), `curve_plane_points` (`:377`),
`curve_plane_bezier_clipping` (`:391`), `curve_plane_algebraic` (`:405`),
`curve_plane_production` (`:419`), `curve_closest_point` (`:433`), `surface_plane` (`:453`),
`surface_plane_uv` (`:471`), `surface_surface` (`:489`), `cut_curves_on_surface` (`:500`).

**There is no `curve_surface`.** Every `curve_*` entry point takes a `Plane`, i.e. an infinite
analytic plane, not a `NurbsSurface`. `src/closest.h` has `Closest::surface_point` (`:45`) and
`Closest::surface_curve` (`:54`), which are minimum-distance queries, not intersections.

**Divergence D1.** The entire §2.5 subsystem (`IntCurveSurface_HInter`) has no counterpart.
Without it there is no way to obtain a piercing parameter on a curved face, and §2.4's exact-seed
stage (`BFI:564-688`) cannot be ported at all.

### 4.2 There is no VF stage, and no face IN/ON sets

Grep of `src/brep.h`, `src/brep_section.h`, `src/nurbssurface_trimmed.h` finds no `FaceInfo`,
no `vertices_in` / `vertices_on`, and no per-face vertex incidence structure of any kind. There
is no `ComputeVF`, no `IsPointInFace`, and no trimmed-face point classifier callable on a
`(face, uv)` pair.

**Divergence D2.** Invariant G4 cannot be stated, let alone tested, against the current code:
the splitter reads UV pcurves and a UV domain rectangle, not a set of IN/ON entities. The
arrangement in `NurbsSurfaceTrimmed::split_by_uv_curves`
(`src/nurbssurface_trimmed.h:74`, call site `src/brep.cpp:4788-4790`) receives
`forced_boundary_nodes` — a bare `std::vector<Point>` of UV points — and nothing else. There is
no notion of "this vertex is IN this face".

**Divergence D3.** Because `ComputeVF`'s strict-IN test (`CTX:604-608`) has no counterpart, G6
is not enforced anywhere: a point that lands on a face's own boundary wire is treated exactly
like an interior point.

### 4.3 The EF surrogates that exist

Three, all partial, two of them off by default.

#### 4.3.1 `SESSION_EF_PAVES` — `src/brep.cpp:3565-3612`

Not a solver. It takes the **endpoints of already-computed SSI cut curves** (`cut_for(srf2)`,
`src/brep.cpp:3581`) and keeps those that lie within `pave_cap_tol` of one of this operand's
edges, tested by sampling the edge at 33 points (`src/brep.cpp:3600-3606`).

Divergences:

- **D4 — it is a filter over the FF result, not an E×F intersection.** A piercing that does not
  happen to be a cut-curve endpoint is invisible to it. In particular, an edge that pierces a
  face *in the interior of a section curve* — the common case when the section curve is closed —
  produces nothing: closed cuts are explicitly skipped (`src/brep.cpp:3588-3593`).
- **D5 — `pave_cap_tol = max(1e-9, diag*2e-3)`** (`src/brep.cpp:3575`). Model-diagonal-relative,
  not entity-tolerance-relative. Two operands of different sizes get the same absolute tolerance;
  a 1000-unit model gets a 2-unit capture radius.
- **D6 — the "lies on an edge" test is a 33-point polyline sample** (`src/brep.cpp:3600-3606`),
  which misses a piercing on a curved edge between samples and false-positives on a near edge.
- **D7 — it is gated behind an env var and documented as a regression** (`src/brep.cpp:3566-3568`:
  "regressed the proven quadric path (reimport cone × cyl 2→8 naked)").
- **D8 — it only runs on the non-scaffold branch** (`else if` at `src/brep.cpp:3565`, after
  `if (scaf)` at `:3562`).

#### 4.3.2 `SESSION_EF_DIAG` — `src/brep_section.cpp:2165-2247`

This *is* a real edge × surface solver, and it is the closest thing we have to
`IntCurveSurface`:

- coarse 33-sample scan of each edge (`src/brep_section.cpp:2176-2185`);
- a local-minimum trigger `d < weld_tol*2.0 && d <= prevd` (`src/brep_section.cpp:2184`);
- a 3×3 Gauss–Newton on `C(t) - S(u,v) = 0` with Jacobian columns `[C', -S_u, -S_v]`, solved by
  Cramer, 12 iterations, step-norm exit `1e-14` (`src/brep_section.cpp:2186-2213`);
- acceptance `|C(t) - S(u,v)| < conv_tol*100` (`src/brep_section.cpp:2219`), where
  `conv_tol = max(tolerance, diag*1e-9)` (`src/brep_section.cpp:423`);
- dedup at `weld_tol*0.1` (`src/brep_section.cpp:2220`).

Divergences:

- **D9 — it injects nothing.** `efpts` is a local (`src/brep_section.cpp:2228`) used only to
  print `[EFDIAG] dangle vN dEF=...` (`src/brep_section.cpp:2244`). No vertex, no pave, no
  interference record.
- **D10 — no trim classification.** It intersects the edge with the **untrimmed surface**
  (`SURFOP.m_surfaces[si]`, `src/brep_section.cpp:2169`). There is no `IsPointInFace`. A
  piercing of the surface outside the face's trim is accepted identically to one inside. This is
  invariant G6 and the second half of G1 both violated at once.
- **D11 — the seed strategy cannot find tangential contacts.** The trigger requires
  `d <= prevd`, i.e. a *monotone decreasing* run of the 33 samples, and `d < weld_tol*2`. A
  tangential graze whose minimum distance is below tolerance but whose sampled distance is not
  monotone near the minimum is missed. OCCT's equivalent (`BFI:564-688` + `BFI:910-1081`) uses
  an exact intersector plus `Extrema_ExtCS` over every unknown cell, precisely so that no
  contact depends on a sampling grid.
- **D12 — the pave-block structure is ignored.** It iterates
  `EDGEOP.m_topology_edges` over the full curve domain (`src/brep_section.cpp:2172-2176`). OCCT
  works per pave block over the *shrunk* range (`PF5:246-306`), so a piercing near an edge end
  is distinguishable from the end vertex. We have no shrunk range at all — grep finds no
  `FindValidRange` equivalent.
- **D13 — `weld_tol = diag*2e-3` and `conv_tol = diag*1e-9`** (`src/brep_section.cpp:421-423`)
  are model-diagonal-relative, violating G7 in the same way as D5.

#### 4.3.3 `SESSION_EF_MARCH` — `src/brep_section.cpp:1832-1990`

Consumes EF junctions, does not discover them: for a chain end that is already dangling, it
looks for an other-operand edge through it and marches the section onto the next face, gated by
a π/6 continuation-angle test. Off by default; reachable only through the `SESSION_AUTO` ladder
(`src/brep.cpp:8348-8360`).

### 4.4 The path that *does* produce EF points — and its exact boundary

`build_section_scaffold` (`src/brep_section.cpp:405+`) computes, for every section chain, the
crossings of the chain's UV image with the **face trim loops of both operands**
(`src/brep_section.cpp:1228-1281`), Newton-refined onto the exact section ∩ trim point by
`refine_trim_pave`. A crossing of the `(FA × FB)` section with `FA`'s trim boundary **is** the
point where the edge bounding `FA` pierces face `FB` — an EF interference obtained by a
different route.

These become scaffold vertices, then the shared pave set for both operands' splits
(`src/brep.cpp:3558-3564`), then result topology vertices via `mint_pave_tv`
(`src/brep.cpp:3681-3690`).

**Divergence D14 — the route only exists when the scaffold is eligible.**
`src/brep.cpp:8436-8438`:

```
bool scaffold_eligible =
    ((imported_freeform && (has_freeform(*this) || has_freeform(other))) || s_scaffold_all)
    && !s_scaffold_off;
```

with `imported_freeform = (prA0.kind == 0 && prB0.kind == 0)` (`src/brep.cpp:8406`) and
`has_freeform` requiring a surface of degree ≥ 3 (`src/brep.cpp:8433-8437`). **Every primitive
pair in the 224-cell battery fails this test**: box × sphere, sphere × sphere, box × cone,
cone × cone, cyl × cone, sphere × cone, box × torus, torus × torus are all recognized or
degree-2 rational. For all of them the EF stage does not exist in any form, by construction.
That is the precise scope of the gap.

**Divergence D15 — the route produces points, not paves.** The scaffold's EF-equivalent points
reach the arrangement only as `forced_boundary_nodes` (`src/brep.cpp:4789`), i.e. as UV points
snapped into the *boundary polylines* within `scaf_forced_eps` (`src/brep.cpp:4280-4281`):

```
scaf_forced_eps = min( max(min_rangeF*1e-2, 0.6*scaf->tol3 / uv3dF), min_rangeF*1.3e-1 )
```

Both the floor and the cap are **fractions of the UV domain extent** `min_rangeF`
(`src/brep.cpp:4261`). This is exactly the domain-relative tolerance family the mission brief
names: a face re-imported on a 4× padded domain gets a 4× larger forced-node radius and a 4×
larger cap. G7 violated. The middle term `0.6*tol3/uv3dF` *is* a proper metric conversion
(`uv3dF` is estimated from `|dS/du|`, `|dS/dv|` at the domain midpoint, `src/brep.cpp:4269-4275`)
— but it is bracketed between two domain-relative bounds, so it only governs in a middle band.

**Divergence D16 — no vertex identity, only coordinate capture.** `find_or_add_vertex`
(`src/brep.cpp:3692+`) and `mint_pave_tv` (`src/brep.cpp:3681`) assign result vertices by
*distance to a pave point*, with radii `pave_tol2 = pave_cap_tol²` and
`orig_tol2 = min(diag*5e-4, min_pair_distance/3)²` (`src/brep.cpp:3643-3660`). OCCT never does
this: a pave carries a vertex **index**, and the split inherits the index
(`PF3:641-652`, `PF3:687`). This is invariant G2 as a data-structure property versus G2 as a
numerical coincidence.

### 4.5 What EF absence does and does not explain (honest attribution)

The mission brief names z15 (missing 18 reference faces / 53.351 of area) as a prime suspect for
the EF gap. The measured evidence in this repository does **not** support that attribution, and
this document will not repeat it:

- `kb/census_z15.md` traces all 17 naked edges of z15 to **one** mechanism: operand A lost the
  entire 9-segment section span `{32..39, 41}` (`[SEGAUDIT] keptA=0` for all nine; seg 37 absent
  from the A2 arrangement entirely). The hole is between A's invented chord path and B's true
  section path. That is class R2 (one-sided section-run loss) in
  `kb/hunt_trimmed_splitter.md:820`, not a missing node.
- `kb/hunt_efvf_gap.md:100-115` measured, against FreeCAD/OCCT, that x20 has 34 true EF points
  and **0** VF interferences, x13y29 has 42 and **0**; and that 33/34 and 42/42 respectively are
  already present as vertices in our shipped result. For the chairs configs the EF points are
  *there*.

What EF absence **does** explain, and what this port must fix:

1. **The 208 non-box×box cells have no EF stage at all** (D14). box × sphere 0/20,
   sphere × sphere 0/20, every cone and torus family 0 — none of these operands can reach the
   scaffold, so no piercing of a sphere face by a box edge is ever computed. This is the
   measurable consequence and it is total, not marginal.
2. **Even where the scaffold runs, EF points are not paves** (D15, D16). A pave point that falls
   outside `scaf_forced_eps` of the boundary polyline is silently not inserted as a node
   (`src/nurbssurface_trimmed.h:74`: "Points farther than a small tolerance from any boundary are
   ignored"). The arrangement then has no vertex where the section ends, the run is
   valence-1, and the dangling-prune removes it — which is precisely the `[SEGLOST]` mechanism
   z15 exhibits. So EF *does* touch z15, not as a missing intersection but as a missing
   **node-insertion contract**: OCCT's EF guarantees the pave exists *and splits the pave
   block*; ours guarantees only that a UV point was offered to a polyline snapper.
3. **There is no ON-set, so coincidence is not representable.** OCCT's EF `type == EDGE` branch
   (`PF5:545-566`) turns an edge lying on a face into a common block. Our kernel has
   `src/brep_commonblock.cpp` (293 lines) but nothing routes an *edge/face* coincidence into it —
   grep finds no caller that pairs a pave block with a face.

### 4.6 Why a missing EF vertex produces lost regions — the mechanism, concretely

Take face `f` of operand B, a cylindrical side, and edge `e` of operand A that pierces it
transversally at `P`, well inside `f`'s trim. Suppose `e` also lies in the plane of the section
curve `s = f ∩ (some face of A)`, so `s` passes through `P` and terminates there.

With EF:
- `P` becomes a vertex `v` with tolerance `max(tol(e), tol(f))`.
- `e`'s pave block is split at `t(P)`; both halves reference `v`.
- `v` is added to `FaceInfo(f).VerticesIn`.
- When `f` is split, `v` is a **node** of the UV arrangement, `s`'s pcurve terminates *at that
  node*, and the two arrangement edges leaving `v` along `f`'s own trim are `e`'s two halves'
  pcurves. `v` has valence ≥ 3. The wire walk finds a closed loop on each side.

Without EF:
- No vertex at `P`.
- `s`'s pcurve on `f` ends at a free end somewhere near `P` (wherever the marcher stopped).
- The arrangement sees an open cutter that does not reach the trim boundary and does not cross
  another cutter. `split_by_uv_curves` documents exactly this outcome
  (`src/nurbssurface_trimmed.h:70-72`): "Dangling open cutters that do not reach the border or
  another cutter are **discarded**."
- The cut is discarded, `f` is emitted **whole** (or split by only the surviving cutters), and
  the region of `f` that should have been on the other side of `s` is either present when it
  should be absent or absent when it should be present.
- On the *other* operand the same section produced a run that *was* kept (because on A's side
  the section ended on a trim crossing that happened to be within snap tolerance). Now A has an
  edge with 1 trim and B has none: a naked edge, long, following the true section path — the
  exact signature `kb/hunt_trimmed_splitter.md:820` records for the lost-region family
  ("many naked edges, individually long; reference faces with no counterpart at all").

The chain is: **no EF vertex → no arrangement node → dangling cutter → cutter discarded → region
never produced → long naked edges on the operand that did keep its run.** Each arrow is a
mechanism in our source, not a supposition: the discard is `src/nurbssurface_trimmed.h:70-72`,
and the one-sided survival is `[SEGAUDIT] keptA=0 keptB=1` in `kb/census_z15.md`.

The VF analogue is milder but real: a vertex of A lying exactly on a face of B, with no VF
record, means that vertex is not in `FaceInfo(f).VerticesIn`, so when `f` is split there is no
node there, and the two section runs that meet at that vertex are two independent free ends
instead of one junction. `kb/hunt_efvf_gap.md:105-107` measured 0 VF interferences in the chairs
configs (nearest approach 0.043 and 0.0079), so VF is not implicated *there* — but a box sharing
a corner with a sphere, or any coplanar/tangent primitive pose, generates them by the dozen, and
those are exactly the poses the 224-cell battery contains.

---

## 5. ACCEPTANCE TESTS

Every test below has an analytically known answer and an oracle-free invariant. "Oracle-free"
means the assertion can be evaluated from our own result alone, without OCCT.

### T1 — Box edge pierces sphere face (transversal, curved face)

Operands: unit box `[0,1]³`; sphere centre `(0.5, 0.5, 0.5)`, radius `0.3`.
Then rotate the sphere by a random axis/angle — a sphere is rotation-invariant about its centre,
so the **answer does not change**, which makes this a determinism test as well.

Analytic answer: the sphere is strictly inside the box; **no box edge pierces the sphere**.
Expected EF interference count: 0. Expected VF: 0.

Now translate the sphere to `(0.9, 0.5, 0.5)`. The plane `x = 1` cuts the sphere in a circle of
radius `sqrt(0.3² − 0.1²) = 0.28284271247461903`. The box edges at `x = 1` are the four lines
`{x=1, y∈[0,1], z∈{0,1}}` and `{x=1, z∈[0,1], y∈{0,1}}`; none reaches within `0.3` of
`(0.9,0.5,0.5)`, so still **0 EF piercings of the sphere by box edges**, but **4 EF piercings of
the box's `x=1` face by the sphere's… none** (the sphere has no edges except its seam).

Use instead: sphere centre `(1.0, 0.5, 0.5)`, radius `0.3`. Now the box edge
`{x=1, y=0.5±, z=0.5}` — construct a box `[0,1]×[0,1]×[0,1]` and additionally a second box
`[1,2]×[0.4,0.6]×[0.4,0.6]`; its edge `{y=0.4, z=0.4, x∈[1,2]}` pierces the sphere
`|P−(1,0.5,0.5)| = 0.3` at `x = 1 + sqrt(0.3² − 0.1² − 0.1²) = 1 + sqrt(0.07) =
1.2645751311064590`.

**Invariants:**
- `I1a` (oracle-free): the result contains a vertex within `1e-9` of
  `(1.2645751311064590, 0.4, 0.4)`.
- `I1b` (oracle-free): the edge `{y=0.4,z=0.4}` in the result is split into exactly two edges
  meeting at that vertex; neither has zero length.
- `I1c` (oracle-free, **G1 half b**): that vertex has valence ≥ 3 in the result's edge graph.
- `I1d` (oracle-free, **G2**): exactly **one** vertex exists within `1e-6` of that point, not two.

### T2 — Box edge lies ON a cylinder face (coincidence, `type == EDGE`)

Cylinder: axis `z`, centre `(0,0,0)`, radius `1`, `z ∈ [0,2]`.
Box: `[−1, 1] × [−0.0000001, 0.0000001] × [0.5, 1.5]` — its edge `{x=1, y=0, z∈[0.5,1.5]}` lies
on the cylinder surface to within `1e-7`.

Analytic answer: that edge is **coincident** with the cylinder face over its whole length.

**Invariants:**
- `I2a`: the EF result for that (pave block, face) pair has exactly one common part, of type
  `Edge`, spanning the whole corrected range.
- `I2b` (**G3**): **no** new vertex is created on that edge by the EF stage.
- `I2c` (**G4**): that pave block appears in `FaceInfo(cylinder face).PaveBlocksIn`.
- `I2d` (oracle-free): in the final result the edge carries exactly 2 trims (one from the box
  face, one from the cylinder face) — i.e. it is a shared edge, not a duplicated pair.

### T3 — Tangential edge/cylinder contact

Cylinder: axis `z`, radius `1`, `z ∈ [0,2]`.
Edge: the line `{y = 1, z = 1, x ∈ [−2, 2]}`. It touches the cylinder at exactly one point,
`(0, 1, 1)`, tangentially.

This is precisely the case `IEF:620-646` (Line × Cylinder post-correction) exists for.

**Invariants:**
- `I3a`: the common part is typed `Vertex` (not `Edge`), with `vertex_param` giving
  `C(t) = (0,1,1)` to within `1e-9`. Rationale: `MakeType` would produce a long near-coincident
  range; `CheckTouch` must demote it.
- `I3b` (**G3**): the transition recorded on the intersection point is `Tangent`
  (`ICSU:876-889`).
- `I3c` (oracle-free): the result contains exactly one vertex within `1e-6` of `(0,1,1)`; the
  edge is split there into two pieces of length 2 each.
- `I3d` (regression guard): raising the fuzzy value to `1e-3` must not turn this into an `Edge`
  common part; only `myCriteria` grows, and `MakeType`'s `isWholeRange` test still fails because
  `|C(t_first) − C(t_last)| = 4 > 2·myCriteria`.

### T4 — Piercing exactly at a pave (the `aTolToDecide = 5e-8` branch)

Box A: `[0,1]³`. Box B: `[0.5, 1.5]³`.
A's edge `{y=1, z=1, x∈[0,1]}` and B's face `{x = 0.5}` meet at `(0.5, 1, 1)`, which is also a
**vertex** of the arrangement created by EE (A's edge × B's edge `{x=0.5,z=1}`).

**Invariants:**
- `I4a`: `ReduceIntersectionRange` (`PF5:685-768`) pulls the working range inward, so EF does
  **not** create a second vertex at `(0.5,1,1)`.
- `I4b` (**G6/G2**): exactly one vertex exists within `1e-6` of `(0.5,1,1)`.
- `I4c`: if the EE vertex is absent (simulate by disabling stage 3), EF must instead take the
  `bIsOnPave` branch and either force a VF interference or grow the existing end vertex — never
  mint a vertex closer than `Precision::Confusion()` to an existing one.

### T5 — Vertex exactly on a face (VF, and the strict-IN rule)

Box A: `[0,1]³`. Box B: `[1,2] × [0.25,0.75] × [0.25,0.75]`.
B's vertex `(1, 0.25, 0.25)` lies exactly on A's face `{x = 1}`, strictly inside its trim
(the face spans `y,z ∈ [0,1]`).

**Invariants:**
- `I5a`: `compute_vf` returns `Ok` with `distance == 0`, `(u,v)` reproducing the point to `1e-12`.
- `I5b` (**G4**): that vertex is in `FaceInfo(A's x=1 face).VerticesIn`.
- `I5c` (**G5**): the vertex's tolerance is at least `0 + tol(face)`.
- `I5d` (**G6**): now move B to `[1,2] × [0,0.5] × [0,0.5]`. Its vertex `(1,0,0)` is on the
  *corner* of A's face, i.e. state `ON`, and `compute_vf` must return `OutsideFace` (`-3`), not
  `Ok`. The interference belongs to VV.

### T6 — Domain independence (G7)

Take T1's configuration. Re-parameterise every surface of both operands by an affine change of
UV domain: `u' = a·u + b` with `a = 4`, `b = −0.04` (the STEP-round-trip pattern recorded in
`kb/ARCHITECTURE_v2.md` §3). Geometry is bit-identical; only the domains differ.

**Invariant `I6` (oracle-free):** the number of EF interferences, the number of VF
interferences, and every piercing point's 3D coordinates must be identical to within `1e-12`.
Any dependence on the domain fails the test. *(This test currently fails by construction —
`scaf_forced_eps` is a fraction of `min_rangeF`, `src/brep.cpp:4280-4281`.)*

### T7 — Symmetry (G2)

For any operand pair, run the EF stage in both orders (`A.edges × B.faces` then
`B.edges × A.faces`, and the reverse) and compare the resulting vertex sets.

**Invariant `I7` (oracle-free):** the two vertex sets are identical as *sets of indices in the
shared arena*, not merely as point clouds. Concretely: `count(vertices with index_new set) `
must be equal, and every vertex must be referenced by pave blocks from both operands where the
geometry says so.

### T8 — Completeness sweep (G1), the master test

For a given operand pair, independently of the boolean:

1. For every edge `e` of A and face `f` of B (and vice versa), sample `e` at `N = 200` points
   over its shrunk range, project each onto `f` (restricted projector), and collect every
   parameter interval where `dist <= tol(e)+tol(f)+fuzz` **and** the foot classifies strictly IN.
2. Refine each interval's minimum by the same Gauss–Newton used in production.
3. Every isolated such point must have a corresponding vertex in the result within
   `max(tol(e), tol(f))`, and the edge must be split there.

**Invariant `I8`:** `missing == 0`. This is a *sampling* oracle, so it can produce false
negatives (a piercing narrower than `1/200` of the range) but never false positives — anything
it finds is real. Run it at `N = 200` and `N = 2000` and require the same answer; a difference
means the geometry has features below the sampling scale and the test must be tightened for
that pair.

The recommended reporting form, matching `kb/hunt_efvf_gap.md`'s census table:

```
| pair | true EF (sampled) | vertices present | edges split | missing |
```

`vertices present` measures G1(a); `edges split` measures G1(b). Our current kernel scores
non-zero on the first column and **zero on the third for every pair**, because no pave is ever
created.

### T9 — Curved-on-curved, the target case

Sphere `S1` centre `(0,0,0)` radius `1`; sphere `S2` centre `(1.5, 0, 0)` radius `1`.
Rotate both by the same random rotation (answer invariant), then rotate `S2` alone by a random
rotation about its own centre (answer still invariant — a sphere is rotation-invariant, only its
*seam edge* moves).

The seam edge of `S2` is a meridian half-circle. After a general rotation it pierces `S1`'s
surface at 0, 1 or 2 points, computable analytically: the seam lies in a plane through `S2`'s
centre; intersect that plane with `S1` (a circle), then intersect that circle with the sphere
`|P − C2| = 1`.

**Invariants:**
- `I9a`: every analytically computed seam piercing has a vertex in the result within `1e-9`.
- `I9b` (**G1b**): `S2`'s seam edge is split at each of them.
- `I9c` (oracle-free): the `cut` result is a closed shell — 0 naked edges — for **all 20 random
  poses**. This is the cell currently scoring 0/20.

---

## 6. IMPLEMENTATION ORDER

Each increment is independently shippable and independently gated. Gates are oracle-free unless
marked. No increment may regress the box × box 15/20 baseline.

### Increment E0 — `curve_surface` primitive, analytic cases only *(no boolean change)*

Build `src/brep_curvesurface.{h,cpp}` with the analytic dispatch of `ICS:532-966`:
Line/Circle/Ellipse/Parabola/Hyperbola × Plane/Cylinder/Cone/Sphere/Torus, using our existing
`IntAna`-equivalent quadric machinery, plus `ComputeAppendPoint` (`ICSU:1116-1176`) and
`ComputeTransitions` (`ICSU:855-895`). Report `curve_in_surface` for the parallel/in-quadric
cases; do **not** enumerate them.

*Gate:* unit tests on the closed-form cases — line × sphere (0/1/2 roots, discriminant known),
line × cylinder, circle × plane, line × torus (quartic). Every root must satisfy
`|C(w) − S(u,v)| < 1e-12` and reproduce the analytic value to `1e-12`. Zero effect on `boolean`.

### Increment E1 — `curve_surface` generic case

Add the polygon/polyhedron seeding path: `C2` decomposition of curve and surface
(`ICSU:1549-1610`, `ICS:139-174`), `SamplePars(defl=0.1, NbMin=10)`, polyhedron with
`nbsu,nbsv ∈ [1,40]` (min 20 for the line fallback), section-point-to-parameter conversion
(`ICSU:739-848`), sort + dedup at `1e-8` (`ICSU:1380-1447`), and the 3×3 Newton
(`ICSU:1460-1519`) with `THE_TOLTANGENCY = 1e-8`.

*Gate:* NURBS curve × NURBS surface cases whose answer is known by construction (build the
surface as a sweep of a curve that provably crosses a plane at a known parameter). Plus: for
every analytic case in E0, E1's generic path must agree with E0's analytic answer to `1e-9` when
forced on. That cross-check is the strongest available oracle-free validation.

### Increment E2 — `ShrunkRange`

Port `SHR:107-191` including `FindValidRange`, `GCPnts_AbscissaPoint::Length` with
`aPTolE = min(Resolution(tol(E)), (t2−t1)/100)`, and the `IsSplittable` rule.

*Gate:* for every edge of every corpus operand, `t1 < tS1 < tS2 < t2`, `length > 0`, and
`splittable` false exactly for edges shorter than `2·tol(E) + 2e-7`. Deterministic across runs.

### Increment E3 — VF stage *(first stage wired into the arena)*

`compute_vf` (`CTX:545-590`) with the UV-restricted projector, the strict-IN classifier, the
three return codes, `UpdateVertex` (`PF10:105-162`), the SD-collapse dedup (`PF4:205-221`), and
`FaceInfo.VerticesIn` population. Plus `TreatVerticesEE` (`PF4:305-390`).

Requires: a trimmed-face point classifier `state(face, uv) ∈ {IN, ON, OUT}`. We have trim loops
(`NurbsSurfaceTrimmed::m_outer_loop`, `m_inner_loops`); the classifier is a winding/crossing
count on the loop polylines with the tolerance converted through `UResolution/VResolution` as in
FClass2d `:728-746`. **This classifier is also the missing piece for G6 in EF, so it is on the
critical path.**

*Gate:* T5 (`I5a`–`I5d`). Plus: on all 224 battery cells, `count(VF) ` is stable across 20 random
poses of a rotation-invariant pair (T1's first configuration). Zero effect on the boolean result
until E5 consumes the sets — this increment is additive to the arena only.

### Increment E4 — `EdgeFaceIntersector`, exact-seed path only

`BeanFaceIntersector` with `ComputeLinePlane` (`BFI:820-906`, including `ComputeIntRange`),
`FastComputeAnalytic` (`BFI:692-816`), `MarkedRangeSet` (`MRS`),
`ComputeAroundExactIntersection` (`BFI:564-688`) on top of E0/E1,
`ComputeRangeFromStartPoint` (`BFI:1150-1340`), and `SetEmptyResultRange`
(`BFI:1344-1365`). Then `IntTools_EdgeFace::Perform` (`IEF:504-685`) with `myCriteria`,
`IsProjectable`, `MakeType`, `CheckTouch` (including the three `IsEqDistance` shortcuts),
and the Line×Cylinder / Circle×Plane post-corrections.

Skip for now: `ComputeUsingExtremum`, `ComputeNearRangeBoundaries`, `ComputeLocalized`,
`IsCoincident`, `TestComputeCoinside`.

*Gate:* T1 (`I1a`), T3 (`I3a`, `I3b`, `I3d`). Report only; nothing is injected into the boolean.
Diff against `SESSION_EF_DIAG`'s output on the chairs configs: the new solver must find every
point `EF_DIAG` finds, plus the ones it misses by D10/D11 — and must reject the ones outside the
trim, which `EF_DIAG` currently accepts.

### Increment E5 — EF driver: vertices and paves *(first behaviour change)*

`PerformEF` (`PF5:165-592`) up to and including `PerformNewVertices`:
per-pave-block loop, `GetPBBox`, `CorrectRange` ×2, `myFPBDone`, the VERTEX branch
(`PF5:406-544`) with `aTolToDecide = 5e-8`, `IsInRange`, `ForceInterfVF`, the
`Precision::Intersection()` real-intersection re-check, the `1e4·tol / 0.1` extension cap,
`CheckFacePaves`, the strict `IsPointInFace` gate, `aMVCPB`, then `PerformNewVertices`
(`PF3:594-688`) → `IntersectVE` → pave-block split. Plus `ReduceIntersectionRange`
(`PF5:685-768`) and `myDistances` (`PF5:348-362`).

Gate it behind `SESSION_V2_EF` initially.

*Gate:* T1 (`I1a`–`I1d`), T4 (`I4a`–`I4c`), T8 (`I8` — the third column becomes non-zero for the
first time). Corpus: box × box must stay at 15/20 genuine; box × sphere must move off 0/20.

### Increment E6 — `ComputeUsingExtremum` + `ComputeNearRangeBoundaries`

The two remaining generic stages of `BFI:288-379`. This is what makes tangential and
near-tangential contacts reliable rather than sampling-dependent, and it is what E4 deliberately
omitted.

Requires a curve×surface extremum solver (`Extrema_ExtCS` equivalent) that reports
`IsParallel`. `src/closest.cpp:248` (`Closest::surface_point`) is the point×surface half; the
curve×surface half must be built, including the parallel branch and the bisection give-up
(`BFI:991-1030`).

*Gate:* T3 (`I3c`), T9 (`I9a`, `I9b`). sphere × sphere must reach `I9c` (0 naked, 20/20).

### Increment E7 — EDGE common parts and the ON set

The `TopAbs_EDGE` branch (`PF5:545-566`), `IsCoincident` (`IEF:62-163`),
`TestComputeCoinside` (`BFI:2129+`), `PaveBlocksIn` population, and routing edge/face
coincidence into the existing `src/brep_commonblock.cpp`.

*Gate:* T2 (`I2a`–`I2d`). Same-domain corpus (A-op-A) must stay green.

### Increment E8 — `UpdateFaceInfoIn` and the splitter contract

Make the face splitter consume `FaceInfo` instead of a bare UV point list:
`vertices_in` become **mandatory nodes** of the UV arrangement (not points offered to a snapper),
`pave_blocks_in` become mandatory interior cutters, and a cutter that terminates at a mandatory
node is **never** discarded as dangling.

This is the increment that closes the mechanism in §4.6. It replaces
`forced_boundary_nodes` / `scaf_forced_eps` (`src/brep.cpp:4280-4281`,
`src/nurbssurface_trimmed.h:74`) entirely.

*Gate:* T6 (`I6` — domain independence, currently failing). z15: `[SEGAUDIT]` asymmetric-segment
count must go to 0; the 9-segment span `{32..39,41}` must be kept by A. Corpus: no regression on
any currently-passing cell.

### Increment E9 — `ForceInterfEF` second pass

`PF5:772-1199`: BVH over pave blocks, same-vertex pairing, the midpoint distance check with
`aTolCheck = 2·max(tol(V1),tol(V2))`, the `0.4226` direction gate, the `aTolAdd` computation,
and EDGE-only harvesting.

*Gate:* the tangent-primitive families (cylinder tangent to plane, sphere tangent to plane) must
produce a single shared face pair rather than two near-coincident ones. Oracle-free check:
`count(faces with exactly 2 trims) == count(faces)` for the tangent cells.

### Increment E10 — retire the surrogates

Delete `SESSION_EF_PAVES` (`src/brep.cpp:3565-3612`), demote `SESSION_EF_DIAG`
(`src/brep_section.cpp:2165-2247`) to a validator that compares the production EF stage against
its own sampling scan (i.e. turn it into T8's harness), and remove `SESSION_EF_MARCH`
(`src/brep_section.cpp:1832-1990`) once E5+E8 make the dangling-end case unreachable.

*Gate:* full 224-cell battery, no regression; `SESSION_AUTO` ladder no longer needed for any
cell it currently rescues.

---

## 7. WHERE OCCT ITSELF GIVES UP — the complete list for this subsystem

Recorded so a port does not invent behaviour that OCCT does not have.

| # | site | give-up / fallback |
|---|---|---|
| 1 | `IEF:449-452` | `CheckTouch`: if `Extrema_ExtCS` reports `IsParallel`, return "no touch" — the common part keeps its `MakeType` classification. |
| 2 | `IEF:722-735` | `CheckTouchVertex`: not done, parallel, or zero extrema → return false, no fallback intersector. |
| 3 | `IEF:224-228` | `DistanceFunction`: projector failure → `myErrorStatus = 4` and return the sentinel `99.`. The caller does not check the status. |
| 4 | `IEF:101-104` | `IsCoincident`: an unprojectable sample is silently skipped, lowering the denominator's effective coverage but not the denominator itself. |
| 5 | `IEF:161-162` | `IsCoincident` is a **majority vote** (`> 0.5`), not a proof. |
| 6 | `BFI:695-700` | `FastComputeAnalytic`: unsupported for Bezier/BSpline/Offset/Other curves — always falls to the generic path. |
| 7 | `BFI:797-807` | `FastComputeAnalytic`: Sphere × Line can only prove *no* intersection, never coincidence. No fast path at all for Cone, Torus, Sphere×Circle, or free-form. |
| 8 | `BFI:409-459` | `Distance`: when the projector fails, falls back to projecting onto the four boundary iso-curves, and then to the four corner points. An approximation, not the true distance to the patch. |
| 9 | `BFI:1027-1030` | `ComputeUsingExtremum`, parallel branch: if bisection over `(b−a) > myCurveResolution` finds no point within criteria, the cell is flagged **empty**. A genuine narrow contact can be lost here. |
| 10 | `BFI:1068-1071` | Non-parallel branch: if no extremum is within criteria, the cell is flagged **empty** — correct only if `Extrema_ExtCS` found all extrema. |
| 11 | `BFI:1221`, `BFI:1270-1273` | The march is capped at 10 boundary-crossing iterations and breaks on parameter underflow. A very long coincident run can be truncated. |
| 12 | `BFI:1749-1764`, `BFI:1932-1934` | `LocalizeSolutions` can fail; `ComputeLocalized` then returns false and the generic three-stage path runs instead. |
| 13 | `BFI:582-583` | When more than one exact intersection point is found, `myCriteria` is **silently reduced** to `3e-7` for the rest of the run, changing the meaning of every subsequent comparison. |
| 14 | `ICS:584-596` | Line × Cone: a cone with `|semiangle| >= pi/2 − 1e5·Angular()` is not solved analytically; falls to the polyhedron path. |
| 15 | `ICS:565-583` | Line × Torus: if `IntAna_IntLinTorus` is not done, falls to the polyhedron path. |
| 16 | `ICSU:1495` | `ProcessSortedPoints` dedups against the **previous** sorted start point only, at `1e-8` in all three parameters. Distinct intersections closer than that are collapsed. |
| 17 | `ICSU:1140-1164` | `ComputeAppendPoint` rejects on a bare parametric threshold `1e-8`, one-sided; a point marginally outside a bound is accepted with its out-of-range parameter. |
| 18 | `ICSU:797-832` | `SectionPointToParameters`: a degenerate polyhedron triangle falls back to projecting onto its longest edge, and finally to vertex A's parameters. |
| 19 | `PF5:331-336`, `PF4:260-265` | A failed pair intersection produces a **warning**, not an error; the pair is simply absent from the result. |
| 20 | `PF5:442-445` | A non-splittable pave block silently drops every VERTEX common part. |
| 21 | `PF5:555-559` | An EDGE common part whose pave-block vertices are not yet known to the face records the interference but creates **no common block** — recovery is deferred to `ForceInterfEF`. |
| 22 | `PF5:1046-1050` | `ForceInterfEF` disables its extra tolerance when the edge tangent is more than ~25° from perpendicular to the face normal. A genuinely coincident but obliquely-parameterised edge can be missed. |
| 23 | `IEF:385-388` vs `IEF:715` | `CheckTouch` uses the **face's adaptor** UV bounds; `CheckTouchVertex` uses the **surface's natural** bounds. An inconsistency in OCCT, preserved because results depend on it. |
| 24 | `PF4:374` | `TreatVerticesEE` discards the new-tolerance output of `ComputeVF` — EE-born vertices never grow from VF. |
| 25 | `PF5:513-519` | The `bLinePlane` tolerance increase is applied to a local variable and **not** written into the vertex; the comment (`PF5:514-515`) states this is deliberate. |

---

## 8. CROSS-REFERENCES

- Target architecture and arena: `kb/ARCHITECTURE_v2.md` (stages 4–5 marked **MISSING** in the
  pipeline table, §2).
- Prior measurement of the gap, including the FreeCAD/OCCT EF census for x20 and x13y29:
  `kb/hunt_efvf_gap.md`. Its verdict — that the *chairs* configs' EF points are already present —
  stands, and §4.5 of this document does not contradict it.
- Defect classification and the lost-region family: `kb/hunt_trimmed_splitter.md:820` (class R2),
  `kb/census_z15.md`.
- Tolerance doctrine, including the audited correction that monotone tolerance growth is design
  intent rather than an OCCT invariant: `kb/audit_occt_tolerance-model.md`.
- PaveFiller stage order and DS semantics: `kb/audit_occt_pavefiller-core.md`.
