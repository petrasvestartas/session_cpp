# PORT SPEC 02 — INTERFERENCE STAGES 1–3: vertex/vertex, vertex/edge, edge/edge

Build document. Everything below is read out of the OCCT tree at
`/home/petras/code/code_cpp/OCCT` (V8 dev tree, same tree as `kb/audit_occt_pavefiller-core.md`).
Path aliases used throughout:

| alias | directory |
|---|---|
| `%BA%` | `src/ModelingAlgorithms/TKBO/BOPAlgo` |
| `%DS%` | `src/ModelingAlgorithms/TKBO/BOPDS` |
| `%BT%` | `src/ModelingAlgorithms/TKBO/BOPTools` |
| `%IT%` | `src/ModelingAlgorithms/TKBO/IntTools` |
| `%TA%` | `src/ModelingAlgorithms/TKTopAlgo/BRepLib` |
| `%GA%` | `src/ModelingAlgorithms/TKGeomAlgo/GeomAPI` |
| `%EX%` | `src/ModelingData/TKGeomBase/Extrema` |

`PF` = `%BA%/BOPAlgo_PaveFiller.cxx`, `PF_n` = `%BA%/BOPAlgo_PaveFiller_n.cxx`.
**IntTools lives in TKBO, not TKGeomAlgo** (the mission brief's path is wrong; the files are at
`%IT%/`).

Our code is at `/home/petras/code/code_rust/session/session_cpp/src`; cited read-only.

Scope: stages VV, VE, EE. VF/EF/FF are other port specs. Everything VV/VE/EE does is **3D
metric**: not one predicate in this subsystem looks at a UV domain. That is the whole point —
these three stages are the layer that makes edge identity come from shared entities instead of
from coordinate coincidence.

---

## 1. WHAT THIS SUBSYSTEM MUST GUARANTEE

Stated so each can be checked by a test with no reference kernel. `tol(x)` is the entity's
stored 3D tolerance; `fuzz` is the operation's fuzzy value, clamped to `>= 1e-7`
(`%BA%/BOPAlgo_Options.cxx:107`).

**G1 — Vertex uniqueness.** After VV, no two vertices `a != b` in the arena satisfy
`|P(a)-P(b)| < tol(a)+tol(b)+fuzz`. Equivalently: the "interferes" relation on vertices has no
edges left inside a component.

**G2 — Fusion containment and minimality.** For every fused vertex `V` built from a cluster
`{v_i}`: `|P(V)-P(v_i)| + tol(v_i) <= tol(V)` for all `i` (containment), and for a cluster of
exactly two, `tol(V)` equals the radius of the *smallest* sphere containing both balls
(`%TA%/BRepLib.cxx:3025-3072`). For clusters of three or more OCCT deliberately over-estimates
(mean centre, max radius); the guarantee there is containment only (`%TA%/BRepLib.cxx:3074-3122`).

**G3 — Fusion is a total, order-independent map.** Fusing is a connected-component computation,
so the result must not depend on the order the pairs were discovered. For `n > 2` OCCT sorts the
cluster's points lexicographically before summing precisely to make the floating-point result
order-independent (`%TA%/BRepLib.cxx:3078-3097`, comment cites OCCT issue 0027540).

**G4 — Idempotence.** Re-running VV/VE/EE on their own output changes nothing: no new fused
vertex, no new pave, no new common block.

**G5 — Every incidence is a shared index, never a coordinate match.** If a vertex `V` lies on an
edge `E` within tolerance, then after VE there is a pave `(E, t, V)` referring to `V` **by
index**. No downstream stage may re-discover this by distance. This is the structural fix for
"edge identity comes from coordinate coincidence".

**G6 — Pave-block partition.** For each edge, the paves (two bounding + all extra) sort by
parameter into a contiguous, non-overlapping cover of the edge's range; each pave block's range
is `(t_i, t_{i+1})` with `t_i < t_{i+1}`; a block whose range degenerates is not emitted and its
two vertices are fused instead (`PF_2.cxx:457-508`).

**G7 — Pave containment.** For every pave `(E, t, V)`: `dist(P(V), C_E(t)) <= tol(V)`. VE
enforces this by *raising* `tol(V)` to `dist + tol(E)` (`%IT%/IntTools_Context.cxx:534`).

**G8 — Pave parameters are interior.** Every *extra* pave's parameter lies strictly inside the
range of the pave block it is attached to (`PF_2.cxx:345-358`, the `aT > aT1 && aT < aT2` test).
An interference whose parameter falls on or outside every block's range is **dropped entirely** —
no pave, no interference record.

**G9 — Crossings become vertices, overlaps become common blocks.** An EE result of type
`TopAbs_VERTEX` creates (or grows) exactly one vertex and one extra pave on each side. An EE
result of type `TopAbs_EDGE` creates **no** vertex; it links the two pave blocks into one
`CommonBlock` (`PF_3.cxx:369-551`).

**G10 — A common block is one geometric reality.** All pave blocks of a common block span the
**same two vertex indices** (`HasSameBounds`, `%DS%/BOPDS_PaveBlock.cxx:148-159`, gate at
`PF_3.cxx:535-540`). The block carries one tolerance, computed as the max over 11 interior
samples of `tol(mate) + dist(sample, mate)` (`%BA%/BOPAlgo_Tools.cxx:248-356`), and that
tolerance is pushed into both bounding vertices (`PF_3.cxx:959-993`).

**G11 — Stage order is a data dependency, not a convention.**
VV → (SD map + rebuilt pave blocks) → VE → (SD rewrite of every pave) → EE → (SD rewrite again).
EE's coincidence gate is *index equality* of bounding vertices; without VV+VE having run it can
never fire. See §2.7.

**G12 — Invariance.** The arena produced by VV/VE/EE is invariant under (a) any reparametrisation
of a face's UV domain — these stages never read UV; (b) any rigid motion, up to the same rigid
motion applied to vertex positions, with tolerances **bit-identical** and curve parameters
**bit-identical** (parameters are intrinsic to the curve, not to the pose).

**G13 — Bounded output.** For any pair of edges the number of common parts is finite and each
part's range is a sub-range of the pave block's range. No stage may loop unboundedly; every
recursion has an explicit shrink guard (§2.6.6).

---

## 2. OCCT'S ALGORITHM

### 2.0 Orchestration and where these stages sit

`BOPAlgo_PaveFiller::PerformInternal` (`PF:234-355`):

```
Init(...)                                   PF:238   → DS, IntTools_Context, Iterator
Prepare(...)                                PF:248   (non-destructive bookkeeping only)
PerformVV(...)                              PF:254   ← STAGE 1
PerformVE(...)                              PF:260   ← STAGE 2
UpdatePaveBlocksWithSDVertices()            PF:266   ← MANDATORY between VE and EE
PerformEE(...)                              PF:268   ← STAGE 3
UpdatePaveBlocksWithSDVertices()            PF:273
PerformVF / PerformEF / …                   PF:275,282
RepeatIntersection(...)                     PF:291   re-runs VV, VE, VF for grown vertices
ForceInterfEE(...)                          PF:298   late EE-coincidence recovery
```

`Init` (`PF:175-213`) builds `BOPDS_DS` with the fuzzy value (`PF:200`), one
`IntTools_Context` (`PF:203`) shared by every worker, and the `BOPDS_Iterator`
(`PF:206-209`).

`RepeatIntersection` (`PF:359-424`): scan all source vertices; any whose tolerance was raised
(`myIncreasedSS`, populated by `UpdateVertex`, `PF_10.cxx:124,158`) or whose SD image was raised
goes into `anExtraInterfMap`; `myIterator->IntersectExt(map)` re-selects pairs against the grown
boxes (`%DS%/BOPDS_Iterator.cxx:363-458`); then **VV, VE, VF run again** (`PF:402,409,416`). So
VV and VE must be re-entrant and must be no-ops when nothing changed (G4).

### 2.1 Candidate-pair generation (shared by all three stages)

`BOPDS_Iterator::Intersect` (`%DS%/BOPDS_Iterator.cxx:270-357`):

1. Build one BVH over the boxes of every source shape with a BRep (`:277-291`). Vertex boxes are
   `SetGap(tol(V) + addTol)` then `Add(point)` (`%DS%/BOPDS_DS.cxx:1604-1606`); edge boxes are the
   curve's box unioned with its vertices' boxes, then `SetGap(GetGap() + addTol)`. `addTol =
   max(fuzz, 1e-7) * 0.5` (`%DS%/BOPDS_DS.cxx:312`).
2. `BOPTools_BoxPairSelector` with `SetSame(true)` selects all interfering box pairs, then
   `Sort()` (`:294-299`).
3. Pairs **within one argument's index range are discarded** (`:311-323`) — no self-interference
   pairs at this stage.
4. A shape is never paired with its own sub-shape (`:337-342`).
5. Optional OBB re-check (`:344-353`).
6. The pair is filed under `TypeToInteger(type1,type2)` as `BOPDS_Pair(min,max)` (`:355-356`).

`Initialize(t1,t2)` **`std::stable_sort`s the pair list** before iterating (`:200`) — this is the
*only* source of determinism in the whole subsystem; there is no index-ordering rule anywhere
else (audit E4). `Value(n1,n2)` swaps so the **lower-dimension** shape comes first
(`:226-243`; `TopAbs` has `FACE=4 < EDGE=6 < VERTEX=7`, so the test `iT1 < iT2` puts the *higher*
enum, i.e. the lower dimension, second — callers read `Value(nV,nE)` at `PF_2.cxx:163` and
`Value(nE,nF)` at `PF_5.cxx:225`).

### 2.2 STAGE 1 — VV (`PF_1.cxx:45-233`)

#### 2.2.1 Candidates and pruning

`PerformVV` (`PF_1.cxx:45-132`):

```
myIterator->Initialize(VERTEX, VERTEX)                       :50
aSize = ExpectedLength(); if (!aSize) return                 :51-56
for each pair (n1,n2):                                       :69
    if (myDS->HasInterf(n1,n2)) { FillMap(n1,n2,...); continue; }   :77-81
    n1SD = SD(n1); n2SD = SD(n2)                             :84-88
    if (BOPTools_AlgoTools::ComputeVV(V(n1SD), V(n2SD), myFuzzyValue) == 0)
        FillMap(n1, n2, aMILI, alloc)                        :93-97
MakeBlocks(aMILI, aMBlocks, alloc)                           :101
for each block: MakeSDVertices(block)                        :104-113
for each key in myDS->ShapesSD(): InitPaveBlocksForVertex(key)  :117-127
```

Notes that matter:
- The already-interfering shortcut at `:77-81` still calls `FillMap`, i.e. a pair recorded as
  interfering by an *earlier* pass (this is `RepeatIntersection`'s second run) is re-merged into
  the same component. Skipping the `FillMap` breaks re-entrancy.
- The predicate runs on the **SD images** (`:90-91`), never on the raw vertices.
- `FillMap` is the symmetric template at `%BA%/BOPAlgo_Tools.hxx:83-102`: it appends `n2` to
  `map[n1]` **and** `n1` to `map[n2]`, so the BFS in `MakeBlocks` can never dereference a missing
  key.
- `MakeBlocks` (`%BA%/BOPAlgo_Tools.hxx:44-80`) is plain BFS connected components over that
  symmetric adjacency, with a fence map; components are emitted in first-seen key order.

#### 2.2.2 The predicate

`BOPTools_AlgoTools::ComputeVV(V1, V2, fuzz)` — `%BT%/BOPTools_AlgoTools.cxx:1772-1794`:

```
fuzz1   = max(fuzz, Precision::Confusion())          // 1e-7
tolSum  = tol(V1) + tol(V2) + fuzz1
return  (P1.SquareDistance(P2) > tolSum*tolSum) ? 1 : 0     // 0 == interfering
```

Strictly greater, so exact touching counts as interfering. There is a point overload
`ComputeVV(V1, P2, tolP2)` at `:1750-1768` using `tol(V1)+tolP2+1e-7`; it is used by the
EE/EF vertex machinery.

The equivalent used by `BOPAlgo_Tools::IntersectVertices` (the *new*-vertex fuser, §2.7.4) is
`BOPAlgo_PairVerticesSelector::Accept` (`%BA%/BOPAlgo_Tools.cxx:1072-1105`), which uses
`max(tol(V), requestedTol)` per side and `+ myFuzzyValue` (no extra Confusion) and tests
`aD2 < aTolSum2` — **strictly less**. The two predicates therefore differ at the exact boundary;
port both as written.

#### 2.2.3 What VV creates: `MakeSDVertices` (`PF_1.cxx:136-233`)

```
for nX in cluster:                                            :143
    if HasShapeSD(nX, nSD1):                                  :146
        if (nSD == -1) { aVSD = Shape(nSD1); nSD = nSD1; }     :149-153
        else aLV.Append(Shape(nSD1))                           :156
    aLV.Append(Shape(nX))                                      :159-160
BOPTools_AlgoTools::MakeVertex(aLV, aVn)                       :162
if (nSD != -1):                                                :164
    // MUTATE THE EXISTING SD VERTEX'S TShape IN PLACE
    BRep_TVertex* t = static_cast<BRep_TVertex*>(aVSD.TShape().get());   :167
    t->Pnt(BRep_Tool::Pnt(aVn));                               :168
    t->Tolerance(BRep_Tool::Tolerance(aVn));                   :169
    aVn = aVSD; nV = nSD;                                      :170-171
else:
    append new ShapeInfo(VERTEX, aVn) to DS → nV               :175-179
box = Box(nV); box.Add(P(aVn)); box.SetGap(tol(aVn) + Precision::Confusion())   :181-184
for n1 in cluster:                                             :193
    myDS->AddShapeSD(n1, nV)                                   :197
    for n2 in cluster after n1:                                :202
        if (Rank(n1) >= 0 && Rank(n1) == Rank(n2)) warn self-interference   :208-219
        if (theAddInterfs && myDS->AddInterf(n1,n2)) {         :221-229
            InterfVV& vv = aVVs.Appended(); vv.SetIndices(n1,n2); vv.SetIndexNew(nV);
        }
return nV
```

Three facts a port must not lose:

- **The weld target is not min-index.** It is the SD image of the *first* cluster member that
  already has one (`:145-153`). Determinism comes from the iterator's `stable_sort`, nothing else.
- **In-place `TShape` mutation** (`:167-171`) is how every already-referencing pave, interference
  and edge silently sees the new position/tolerance. A port that instead allocates a new vertex
  must re-point every reference — which is exactly why our arena should store vertices by index
  and mutate the arena record (see §3).
- **Box gap is `tol + 1e-7`** here (`:184`), *not* `tol + fuzz/2`. Two gap policies coexist in
  OCCT (audit E6): `+Confusion` for `MakeSDVertices` (`PF_1.cxx:184`), `UpdateVertex`
  (`PF_10.cxx:123,148`); `+ fuzz/2` for the new EE/EF vertices (`PF_3.cxx:639`) and for shrunk-range
  boxes (`PF_3.cxx:822`).

`AddShapeSD(i, iSD)` is a **no-op when `i == iSD`** (`%DS%/BOPDS_DS.cxx:1219-1226`);
`HasShapeSD` walks the chain to a fixpoint (`:1229-1240`), as does `GetSameDomainIndex`
(`:1245-1254`).

`AddInterf(i1,i2)` (`%DS%/BOPDS_DS.lxx:94-103`) inserts an **unordered** `BOPDS_Pair` into
`myInterfTB` and marks both indices in `myInterfered`; returns false if already present.
`BOPDS_Pair::IsEqual` is order-insensitive (`%DS%/BOPDS_Pair.hxx:69-73`) while `operator<` is
ordered on `(i1,i2)` (`%DS%/BOPDS_Pair.hxx:61-66`) — a pair used as a map key must therefore be
normalised `(min,max)` at construction, which the iterator does (`%DS%/BOPDS_Iterator.cxx:356`).

#### 2.2.4 The fusion geometry: `BRepLib::BoundingVertex`

`BOPTools_AlgoTools::MakeVertex(list, Vnew)` — `%BT%/BOPTools_AlgoTools.cxx:1798-1813`:
`n == 1` ⇒ alias the single vertex (no copy); `n > 1` ⇒ `BRepLib::BoundingVertex` then
`BRep_Builder::MakeVertex`. `n == 0` leaves `Vnew` null.

`BRepLib::BoundingVertex` — `%TA%/BRepLib.cxx:3013-3123`:

```
n < 2 : return, outputs UNTOUCHED                                          :3019-3023

n == 2 :                                                                   :3025-3072
    eps = RealEpsilon()                       // 2.220446049250313e-16     :3032
    m = index of LARGER tolerance, n = index of smaller                    :3040-3046
    dR = R[m] - R[n]                          // >= 0                      :3048
    VD = gp_Vec(P[m], P[n]); D = |VD|                                      :3049-3050
    if (D <= dR || D < eps):                  // small ball inside large    :3052
        C = P[m];  T = R[m]
    else:
        T = 0.5 * (R[m] + R[n] + D)                                        :3063
        C = 0.5 * (P[m] + P[n] - VD * (dR / D))                            :3064

n > 2 :                                                                    :3074-3122
    points[] = P(v_i)
    std::sort(points, BRepLib_ComparePoints())   // lexicographic x,y,z    :3090
    C = (sum of sorted points) / n                                         :3092-3099
    T = max over i of ( |C - P(v_i)| + R(v_i) )                            :3104-3118
```

`BRepLib_ComparePoints` is at `%TA%/BRepLib.cxx:96-113`: compare `Coord(1..3)` in order,
strictly less / strictly greater, else false.

The `n == 2` branch is the exact smallest enclosing sphere of two balls. Verification (write it
into the port's unit test): with `u = VD/D` (unit vector from `P[m]` toward `P[n]`),
`C - P[m] = 0.5*(D - dR)*u`, so `|C-P[m]| + R[m] = 0.5D - 0.5(R[m]-R[n]) + R[m] = T`, and
`|C-P[n]| + R[n] = 0.5(D+dR) + R[n] = T`. Both balls are internally tangent to the result.

The `n > 2` branch is **not** minimal — it is mean-centre plus max-radius. Port it verbatim
including the sort; the sort is the only reason the result is reproducible.

### 2.3 STAGE 2 — VE (`PF_2.cxx:141-395`, worker `PF_2.cxx:40-134`)

#### 2.3.1 Candidates and the pruning ladder

`PerformVE` (`PF_2.cxx:141-208`):

```
FillShrunkData(VERTEX, EDGE)                                    :143   ← see §2.5
myIterator->Initialize(VERTEX, EDGE)                            :145
for each (nV, nE):                                              :156
    if ShapeInfo(nE).HasSubShape(nV)          continue          :166-169
    if ShapeInfo(nE).HasFlag()                continue          :171-174   (degenerate edge)
    if myDS->HasInterf(nV, nE)                continue          :176-179
    if myDS->HasInterfShapeSubShapes(nV, nE)  continue          :181-184
    aLPB = PaveBlocks(nE); if empty          continue           :186-190
    aPB = aLPB.First();
    if (!aPB->IsSplittable())                 continue          :192-197   (micro edge)
    aMVEPairs[aPB].Append(nV)                                   :199-204
IntersectVE(aMVEPairs, ...)                                     :207
```

- `HasFlag()` is `myFlag >= 0`; **flag value 0 is valid** (`%DS%/BOPDS_ShapeInfo.lxx`) — using 0 as
  "no flag" disables degenerate handling for shape index 0.
- `HasInterfShapeSubShapes(nV, nE)` (`%DS%/BOPDS_DS.cxx:356-375`, `theAnyInterference` defaults
  true) is `any_of(subshapes(nE), s => HasInterf(nV, s))`: **if the vertex already interferes with
  one of the edge's own vertices, VE is skipped**. This is the VV→VE dependency in code form.
- The map key is only `aLPB.First()` — the *first* pave block of the edge, even if the vertex will
  eventually land in a different block. The real block is picked later, geometrically.

#### 2.3.2 `IntersectVE` (`PF_2.cxx:212-395`)

Preparation loop (`:238-292`):

```
for each (aPB, list<nV>) in theVEPairs:
    nE = aPB->OriginalEdge()
    aMVPB = { every Pave1().Index() and Pave2().Index() of every PB of nE }   :247-254
    for nV in list:
        nVSD = SD(nV)                                                          :262-263
        if (aMVPB.Contains(nVSD)) continue      // already a pave of this edge :265-268
        key = BOPDS_Pair(nVSD, nE)
        if (aDMVSD has key) { aDMVSD[key].Append(nV); continue; }              :271-277
        aDMVSD[key] = {nV}                                                     :279-280
        enqueue solver(nVSD, nE, V(nVSD), E(nE), aPB, fuzz)                    :285-290
```

So one *solve* per (SD vertex, edge) pair; the resulting interference is later replayed for every
original vertex that maps to that SD image (`:366-387`).

Solve (`PF_2.cxx:104-121`, parallel at `:304`): `myContext->ComputeVE(V, E, aT, aTolVNew, fuzz)`;
a thrown `Standard_Failure` is caught and turned into a warning, and the pair is dropped
(`:117-120`, `:322-330`).

Result handling (`:315-388`):

```
if (flag != 0) { warn if errors; continue }                     :322-330
aT       = solver.Parameter()
aTolVNew = solver.VertexNewTolerance()
nVx = UpdateVertex(nV, aTolVNew)                                :338      ← may MINT a vertex
// find the pave block whose OPEN range contains aT
for aPB in PaveBlocks(nE):
    aPB->Range(aT1,aT2); if (aT > aT1 && aT < aT2) break        :345-354
if (no such block) continue                                     :355-358  ← INTERFERENCE DROPPED
aPave.SetIndex(nVx); aPave.SetParameter(aT); aPB->AppendExtPave(aPave)   :360-363
aMEdges.Add(nE)                                                 :364
if (theAddInterfs):                                             :366
    for nVOld in aDMVSD[BOPDS_Pair(nV,nE)]:                     :370-372
        InterfVE& ve = aVEs.Appended(); ve.SetIndices(nVOld,nE); ve.SetParameter(aT)  :376-378
        myDS->AddInterf(nVOld, nE)                              :380
        if (myDS->IsNewShape(nVx)) ve.SetIndexNew(nVx)          :382-385
SplitPaveBlocks(aMEdges, theAddInterfs)                         :394
```

`AppendExtPave` is **fenced by vertex index** (`%DS%/BOPDS_PaveBlock.cxx:167-174`): a second pave
with the same vertex index on the same block is ignored regardless of parameter. `AppendExtPave1`
(`:177-181`) is unfenced and is used only for degenerate edges and seams
(`%DS%/BOPDS_DS.cxx:467-484`). `RemoveExtPave(vertNum)` removes **all** paves of that vertex
(`:184-203`) — a port fencing by `(vertex, parameter)` breaks that.

#### 2.3.3 The VE predicate: `IntTools_Context::ComputeVE`

`%IT%/IntTools_Context.cxx:499-541`:

```
if (BRep_Tool::Degenerated(E))  return -1                              :505-508
if (!BRep_Tool::IsGeometric(E)) return -2                              :509-512
P = BRep_Tool::Pnt(V)                                                  :517
proj = ProjPC(E);  proj.Perform(P)                                     :519-520
if (proj.NbPoints() == 0) return -3                                    :522-526   ← GIVES UP
aDist   = proj.LowerDistance()                                         :528
aTolSum = tol(V) + tol(E) + max(fuzz, Precision::Confusion())          :530-532
theTol  = aDist + tol(E)                                               :534       ← NOT + tol(V)
theT    = proj.LowerDistanceParameter()                                :535
if (aDist > aTolSum) return -4                                         :536-539
return 0
```

`ProjPC(E)` (`%IT%/IntTools_Context.cxx:269-286`) is a **cached** `GeomAPI_ProjectPointOnCurve`
initialised on `BRep_Tool::Curve(E, f, l)` — the **edge's own parameter range**, not a pave
block's. Caching is per-`TopoDS_Edge`, so the port needs the same cache or it will pay the
Extrema setup cost per query.

**Failure mode `-3` is real and common on curved edges.** `GeomAPI_ProjectPointOnCurve::Perform`
(`%GA%/GeomAPI_ProjectPointOnCurve.cxx:131-153`) sets `myIsDone = myExtPC.IsDone() &&
NbExt() > 0`, and `Extrema` for an **analytic** curve (Line, Circle, Ellipse, Hyperbola, Parabola)
returns only true perpendicular feet whose parameter lies inside `[uinf, usup]`
(`%EX%/Extrema_GGExtPC.hxx:503-525`). Curve **endpoints are appended only** for
BSpline/Offset/Other curves and only when the endpoint's squared distance is below
`Precision::SquareConfusion()` = 1e-14 (`%EX%/Extrema_GGExtPC.hxx:474-501`). Consequence for a
circular arc: a vertex sitting just outside the arc's angular span has **no** foot on the arc, so
`ComputeVE` returns `-3` and no VE interference exists. OCCT relies on VV to have caught it
(the arc's own end vertex is nearby). **A port that uses a clamping "closest parameter" here
silently creates a pave at the arc's end**, which yields a zero-length pave block, which triggers
the SD-weld cascade of §2.4 and fuses two vertices that should stay distinct.

`ComputePE` (`%IT%/IntTools_Context.cxx:437-495`) is the *point* analogue used by common-block
grouping. It **does** have the endpoint fallback: if `NbPoints() == 0` it iterates the edge's
FORWARD/REVERSED vertices, accepts the nearest with
`aTolSum = tolP1 + tol(V) + Precision::Confusion()` and takes `aT = BRep_Tool::Parameter(V, EFwd)`
(`:468-493`); returns `-3` only if none qualifies, and `-4` when a foot exists but
`aDist > tolP1 + tol(E) + Confusion` (`:463-466`).

#### 2.3.4 `UpdateVertex` — how a vertex's tolerance grows (`PF_10.cxx:105-162`)

```
nVNew = nV
if (IsNewShape(nVNew) || HasShapeSD(nV,nVNew) || !myNonDestructive):     :112
    V = Shape(nVNew)
    if (tol(V) < aTolNew):                                               :117
        BRep_Builder().UpdateVertex(V, aTolNew)        // in-place       :119
        Box(nVNew) = BRepBndLib::Add(V);  gap += Precision::Confusion()  :120-123
        myIncreasedSS.Add(nV)                                            :124
    return nVNew
// non-destructive AND nV is an original vertex: never touch it
Vnew = MakeVertex(P(V), max(tol(V), aTolNew))                            :134-136
nVNew = DS.Append(ShapeInfo(VERTEX, Vnew))                               :139-142
Box(nVNew) = BRepBndLib::Add(Vnew); gap += Precision::Confusion()        :145-148
myDS->AddShapeSD(nV, nVNew)                                              :151
myVertsToAvoidExtension.Add(nVNew)                                       :154
if (tol(V) < aTolNew) myIncreasedSS.Add(nV)                              :156-159
```

`myIncreasedSS` is what drives `RepeatIntersection` (`PF:365-390`).

#### 2.3.5 `SplitPaveBlocks` — turning extra paves into blocks (`PF_2.cxx:419-626`)

```
for nE in theMEdges:                                                     :432
  for aPB in ChangePaveBlocks(nE):                                       :438
    if (!aPB->IsToUpdate()) { next; }        // no ext paves             :443-447
    aCB = CommonBlock(aPB)                                               :449
    aPB->Update(aLPBN)                       // theFlag defaults TRUE    :453
    for aPBN in aLPBN:                                                   :457
        myDS->UpdatePaveBlockWithSDVertices(aPBN)                        :461
        FillShrunkData(aPBN)                                             :462
        bHasValidRange = aPBN->HasShrunkData()                           :464
        bCheckDist     = bHasValidRange && !aPBN->IsSplittable()         :468
        if (!bHasValidRange || bCheckDist):
            aPBN->Indices(nV1,nV2)
            if (nV1 == nV2) continue         // DROPPED, no weld         :473-477
            if (bCheckDist && ComputeVV(V1,V2,fuzz) == 0)
                bHasValidRange = false                                   :480-489
            if (!bHasValidRange):
                if (aMPairs.Add(BOPDS_Pair(nV1,nV2))):                   :495
                    MakeSDVertices({nV1,nV2}, theAddInterfs)             :500
                    aMVerticesToInitPB += {nV1, nV2}                     :503-504
                continue                     // sub-block NOT appended   :506
        aLPB.Append(aPBN)                                                :511
        if (aCB) aMCBNewPB[aCB].Append(aPBN)                             :513-523
    aLPB.Remove(old aPB)                                                 :526
… CB regroup (see below) …                                               :531-618
for v in aMVerticesToInitPB: InitPaveBlocksForVertex(v)                  :621-625
```

`BOPDS_PaveBlock::Update` (`%DS%/BOPDS_PaveBlock.cxx:249-312`):
`aNb = extPaves + (theFlag ? 2 : 0)`; **if `aNb <= 1`, clear the ext paves and the fence and emit
nothing** (`:262-268`) — this is how an edge ends up with an *empty* pave-block list, which
downstream reads as "deleted". Otherwise collect all paves into an array, `std::sort` by
`BOPDS_Pave::operator<` = parameter only (`%DS%/BOPDS_Pave.hxx:63-65`), and emit consecutive
pairs as new pave blocks carrying the same `OriginalEdge` (`:276-311`). `std::sort` is unstable;
equal-parameter paves therefore produce zero-length blocks that the loop above filters.

CB regroup (`:531-618`): the new sub-blocks of a common block are grouped by
`BOPDS_Pair(pave1.Index, pave2.Index)` (`:538-553`). For an **open** representative
(`aCB->PaveBlock1()` has `nV1 != nV2`), each group becomes one new common block inheriting the
parent's face list (`MakeNewCommonBlock`, `:401-415`, `:564-568`). For a **closed**
representative, groups are further clustered geometrically (`:572-616`): take the first member,
its mid-parameter point `aPMFirst` and `aTolEFirst = BRep_Tool::MaxTolerance(E_first, VERTEX)`;
a candidate joins when
`ComputePE(aPMFirst, aTolEFirst + MaxTolerance(E_cand,VERTEX) + fuzz, E_cand, aTOut, dist)`
succeeds **and** `aTOut` is strictly inside the candidate's own pave range (`:602-610`).

### 2.4 `IntTools_ShrunkRange` — the splittability oracle

Used by VE (`PF_2.cxx:143,462`), EE (`PF_3.cxx:147`) and EF. `%IT%/IntTools_ShrunkRange.cxx:107-191`:

```
if (myT2 - myT1 < Precision::PConfusion())     return   // 1e-9, not done  :117-120
tolE = tol(E); tolV1 = max(tol(V1), tolE); tolV2 = max(tol(V2), tolE)      :124-135
tolV1 += Precision::Confusion(); tolV2 += Precision::Confusion()           :139-140
if (!BRepLib::FindValidRange(BAC, tolE, myT1,P1,tolV1, myT2,P2,tolV2, myTS1,myTS2))
    return                                     // no valid range            :147-151
if (myTS2 - myTS1 < Precision::PConfusion())   return   // micro edge       :152-156
pTolE = min( BAC.Resolution(tolE), (myT2-myT1)/100 )                        :161-167
myLength = GCPnts_AbscissaPoint::Length(BAC, myTS1, myTS2, pTolE)           :170
if (myLength < Precision::Confusion())         return   // micro edge       :171-175
myIsDone = true                                                             :177
myIsSplittable = (myLength > 2*tolE + 2*Precision::Confusion())             :185-187
BndLib_Add3dCurve::Add(BAC, myTS1, myTS2, tolE + Precision::Confusion(), myBndBox)  :190
```

`BRepLib::FindValidRange` (`%TA%/BRepLib_1.cxx:173-247`) trims the parameter range to the part of
the curve **outside both vertex tolerance spheres**:

```
if (t2 - t1 < PConfusion()) return false                                    :177-180
anEps = max( max(Curve.Resolution(tolE)*0.1, Epsilon(maxAbsPar)), PConfusion() )  :195-196
first = findNearestValidPoint(curve, t1, t2, isFirst=true,  P1, tolV1, anEps)     :210-219
    if failed return false; if (t2 - first < anEps) return false            :220-224
last  = findNearestValidPoint(curve, t1, t2, isFirst=false, P2, tolV2, anEps)     :233-242
    if failed return false; if (last - t1 < anEps) return false             :243-246
if (first > last) return false      // overlapping spheres                  :249-254
```

`findNearestValidPoint` (`%TA%/BRepLib_1.cxx:31-164`):
1. If the start end is already **outside** the vertex sphere, return false (`:48-54`) — the vertex
   does not cover its own curve end, so the data is inconsistent.
2. March with `aStep = Curve.Resolution(tol) * 1.01`, floored at `anEps` (`:61-65`), until a point
   is outside the sphere. For Bezier/BSpline (or an offset of one) a small-derivative accelerator
   doubles the local step whenever `|C'| ² < aD1Mag = (1/Resolution(1.))*0.01` squared
   (`:68-82`, `:107-136`). If the far end is reached still inside, return false (all inside).
3. Bisect between the last inside and the first outside parameter until the bracket is below
   `theEps`; return the midpoint (`:147-163`).

This is the mechanism that makes "micro edge" a *measured* property (arc length outside the
vertex balls) rather than a parametric guess. **Our kernel has no equivalent at all.**

### 2.5 `FillShrunkData` batch form (`PF_9.cxx:65-137`)

Called at the head of VE (`PF_2.cxx:143`), EE (`PF_3.cxx:147`), EF (`PF_5.cxx:167`). It walks the
iterator's pairs of the requested type combination, and for every EDGE index not yet visited,
recomputes shrunk data for every pave block where `!HasShrunkData() || !IsValidShrunkData(aPB)`
(`:104-108`). The solves run in parallel (`:130`); `AnalyzeShrunkData` runs serially afterwards
(`:132-137`).

`AnalyzeShrunkData` (`PF_3.cxx:766-824`):
- Not done or not splittable ⇒ classify: `bWholeEdge = (PBfirst <= Efirst && PBlast >= Elast)`
  (`:775-778`); emit `AlertTooSmallEdge` / `AlertNotSplittableEdge` (whole edge) or
  `AlertBadPositioning` (a sub-range) — all **warnings**, the pipeline continues (`:793-817`).
- Not done ⇒ `SetShrunkData(TS1, TS2, Bnd_Box() /*void*/, false)` and return (`:803-806`); a void
  box makes `HasShrunkData()` false (`%DS%/BOPDS_PaveBlock.cxx:317-320`).
- Otherwise `box.SetGap(box.GetGap() + myFuzzyValue/2)` then
  `SetShrunkData(TS1, TS2, box, IsSplittable())` (`:819-823`).

### 2.6 STAGE 3 — EE (`PF_3.cxx:145-590`, engine `%IT%/IntTools_EdgeEdge.cxx`)

#### 2.6.1 Candidates and pruning (`PF_3.cxx:181-267`)

```
FillShrunkData(EDGE, EDGE)                                             :147
for each (nE1, nE2) from the iterator:                                 :181
    skip if either ShapeInfo HasFlag()   (degenerate)                  :189-198
    skip if either pave-block list is empty                            :200-210
    for aPB1 in PB(nE1):                                               :215
        if (!GetPBBox(E1, aPB1, cache, aT11,aT12, aTS11,aTS12, aBB1)) continue   :226
        for aPB2 in PB(nE2):                                           :233
            if (!GetPBBox(...aBB2)) continue                           :240
            if (aBB1.IsOut(aBB2)) continue                             :245-248
            bExpressCompute = same unordered bounding-vertex pair       :252
            enqueue EdgeEdge(aPB1,aPB2, E1[aT11,aT12], E2[aT21,aT22],
                             boxes, fuzz, UseQuickCoincidenceCheck(bExpressCompute))  :254-264
```

`GetPBBox` (`PF_3.cxx:914-955`): range validity is `last - first > Precision::PConfusion()`
(`:925`); if the block has shrunk data, its stored shrunk range and box are used; otherwise the
full range and a freshly built `BndLib_Add3dCurve` box with `tol(E) + Precision::Confusion()`
(`:939-953`), memoised per pave block.

Note the ranges handed to the engine are the **full** pave-block ranges `aT11..aT12`
(`:261-262`), while the *boxes* are the shrunk ones. The shrunk parameters are used later only to
build the four "collar" ranges (`:339`).

The worker `BOPAlgo_EdgeEdge::Perform` (`PF_3.cxx:87-129`) optionally **translates both edges to
the origin** when `BOPAlgo_Tools::TrsfToPoint(box1, box2, trsf)` says the pair is far from the
origin (`:100-109`). `TrsfToPoint` (`%BA%/BOPAlgo_Tools.cxx:1912-1937`, default
`thePoint = (0,0,0)`, `theCriteria = 1e+5`): unite the boxes; if the centre is closer than `1e5`
to the target point, or if `sqrt(box.SquareExtent())/dist > 1/1e5`, do nothing; otherwise
translate by `CornerMin → thePoint`. Afterwards the original edges are restored on the common
parts (`:118-128`).

#### 2.6.2 `IntTools_EdgeEdge::Prepare` — role assignment and the tolerance→parameter bridge

`%IT%/IntTools_EdgeEdge.cxx:89-181`.

```
myCurve1.Initialize(E1); myCurve2.Initialize(E2)                        :94-95
if range == (0,0) use the curve's own first/last                        :97-107
iCT1 = TypeToInteger(type1); iCT2 = TypeToInteger(type2)                :112-113
if (iCT1 == iCT2 && iCT1 != 0):                                         :115-130
    aC2 = CurveDeflection(curve2, range2)
    aC1 = (aC2 > Precision::Confusion()) ? CurveDeflection(curve1, range1) : 1.
    if (aC1 < aC2) --iCT1                     // force the swap below
if (iCT1 < iCT2) swap edges/curves/ranges; mySwap = true                :132-147
aTolAdd = myFuzzyValue / 2.                                             :149
myTol1 = curve1.Tolerance() + aTolAdd                                   :150
myTol2 = curve2.Tolerance() + aTolAdd                                   :151
myTol  = myTol1 + myTol2                                                :152
if (iCT1 != 0 || iCT2 != 0):                                            :154
    myGeom1/2 = BRep_Tool::Curve(...)                                   :158-159
    myResCoeff1 = ResolutionCoeff(curve1, range1)                       :161
    myResCoeff2 = ResolutionCoeff(curve2, range2)                       :162
    myRes1 = Resolution(geom1, type1, myResCoeff1, myTol1)              :164
    myRes2 = Resolution(geom2, type2, myResCoeff2, myTol2)              :165
    myPTol1 = 5.e-13; if (max|range1| > 999.) myPTol1 = 5.e-16*max|range1|   :167-172
    myPTol2 = 5.e-13; if (max|range2| > 999.) myPTol2 = 5.e-16*max|range2|   :174-179
```

`TypeToInteger` (`:1456-1482`): `Line = 0`; `Hyperbola, Parabola = 1`; `Circle, Ellipse = 2`;
`Bezier, BSpline = 3`; everything else `= 4`.

`CurveDeflection` (`:1611-1638`): total turning angle of the tangent over 10 uniform samples
(`aNbP = 10`), skipping samples with sub-`gp::Resolution()` derivative.

**Net effect of the swap rule: `myCurve1` ends up being the more complex / more strongly curved
curve.** Curve 1 is the one that gets *subdivided* and *sampled*; curve 2 is the *projection
target*. That is the curvature-driven role assignment, and it is the reason line/curve pairs are
handled with the line as curve 2.

`ResolutionCoeff` (`:1486-1557`) — the per-curve constant that converts a 3D distance into a
parameter step:

| curve type | coefficient |
|---|---|
| Circle | `1 / (2R)` (`:1495-1497`) |
| Ellipse | `1 / MajorRadius` (`:1498-1500`) |
| OffsetCurve on Line | `0` (`:1506-1509`) |
| OffsetCurve on Circle | `1 / (2*(offset + R))` (`:1510-1514`) |
| OffsetCurve on Ellipse | `1 / (offset + MajorRadius)` (`:1515-1519`) |
| Hyperbola, Parabola, Other, offset-of-anything-else | `min over 30 sampled sub-intervals of (dt / |ΔP|)` — i.e. the reciprocal of the maximum sampled speed; `kMin` starts at 10 (`:1524-1551`) |
| Line, Bezier, BSpline | `0` (unused; `Resolution` handles them) |

`Resolution(curve, type, coeff, R3D)` (`:1561-1607`):

| type | parametric resolution |
|---|---|
| Line | `R3D` (unit-speed line) |
| Circle | `aDt = coeff*R3D`; `res = (aDt <= 1) ? 2*asin(aDt) : 2π` — the **exact** chord-to-angle inversion, saturating at a full turn |
| Bezier | `Geom_BezierCurve::Resolution(R3D, res)` |
| BSpline | `Geom_BSplineCurve::Resolution(R3D, res)` |
| OffsetCurve on Line | `R3D`; on Circle: the `2*asin` form |
| default (Hyperbola, Parabola, Ellipse, Other) | `coeff * R3D` |

**This is the model-space → parameter-space conversion our kernel is missing.** `myRes1`/`myRes2`
are the parametric images of `myTol1`/`myTol2` through each curve's own metric, and every
downstream parametric comparison in the engine is expressed in those units.

`myPTol1/2` are pure floating-point parameter resolutions for the bisection in
`FindParameters`, **not** geometric tolerances.

#### 2.6.3 `Perform` — the dispatcher (`:185-243`)

```
CheckData();  if (myErrorStatus) return                                  :188-192
Prepare()                                                                :195
if (type1 == Line && type2 == Line) { ComputeLineLine(); return; }        :198-202
if (myQuickCoincidenceCheck && IsCoincident()):                           :204-215
    AddSolution(full range1, full range2, TopAbs_EDGE);  return
if (both types <= Parabola && one of them is Line):                       :217-233
    d = BRepExtrema_DistShapeShape(E1, E2, Extrema_ExtFlag_MIN).Value()
    if (d > 1.1 * myTol) return                    // provably disjoint
FindSolutions(ranges1, ranges2, bSplit2)                                  :239
MergeSolutions(ranges1, ranges2, bSplit2)                                 :242
```

`CheckData` (`%IT%/IntTools_EdgeEdge.lxx:189-208`): null edge ⇒ status 1; degenerate ⇒ 2;
non-geometric ⇒ 3. `IsDone()` is `myErrorStatus == 0` (`lxx:182-185`). `PerformEE` treats
`!IsDone() || HasErrors()` as a **warning** and skips the pair (`PF_3.cxx:294-301`).

`SetFuzzyValue` clamps to `>= Precision::Confusion()` (`lxx:161-164`); the default member value
is `Precision::Confusion()` (`lxx:24`).

#### 2.6.4 `IsCoincident` — the quick coincidence heuristic (`:247-286`)

```
aTresh = 0.5;  aNbSeg = 23
proj.Init(myGeom2, aT21, aT22)
dT = (aT12 - aT11)/aNbSeg
iCnt = count over i = 0..aNbSeg of ( proj(geom1(aT11 + i*dT)) has points
                                     && LowerDistance() < myTol )
return (double(iCnt) / (aNbSeg + 1)) > aTresh
```

24 samples, accept on a bare majority. This is explicitly a heuristic and is gated behind
`bExpressCompute` — i.e. only used when the two pave blocks already share both bounding vertices
(`PF_3.cxx:252-256`) or when `ForceInterfEE` forces it on (`PF_3.cxx:1209`). On acceptance the
whole of both ranges is emitted as a single `TopAbs_EDGE` common part (`:206-214`).

#### 2.6.5 `FindSolutions` (top level) — closed-curve handling (`:290-349`)

```
bIsClosed2 = IsClosed(myGeom2, aT21, aT22, myTol2, myRes2)               :302
if (bIsClosed2):
    box1 = Bnd(curve1 over [aT11,aT12], myTol1)
    bIsClosed2 = !box1.IsOut(myGeom2->Value(aT21))    // seam must be near curve1  :306-310
if (!bIsClosed2):
    FindSolutions(range1, box1, range2, box2, out1, out2);  return       :312-318
if (!CheckCoincidence(aT11,aT12,aT21,aT22, myTol, myRes1)):              :320-325
    // fully coincident: return the whole ranges, merge will promote to EDGE
    out1 += range1; out2 += range2; return
aNb1 = IsClosed(myGeom1, aT11, aT12, myTol1, myRes1) ? 2 : 1             :330
aNb2 = 2                                                                 :331
aNb1 = SplitRangeOnSegments(aT11,aT12,myRes1,aNb1,seg1)                  :333
aNb2 = SplitRangeOnSegments(aT21,aT22,myRes2,aNb2,seg2)                  :334
for i,j: FindSolutions(seg1[i], box(seg1[i]), seg2[j], box(seg2[j]), out1, out2)   :336-346
bSplit2 = (aNb2 > 1)                                                     :348
```

`IsClosed(curve, t1, t2, tol, res)` (`:1642-1659`): false if `|t1-t2| < res`; else
`|C(t1) - C(t2)| < tol`.

`SplitRangeOnSegments(t1, t2, resolution, nbSeg, out)` (`:1366-1406`): if the span is below
`resolution` or `nbSeg == 1`, emit one segment; otherwise `dt = span/nbSeg`, and if `dt` would be
below `resolution`, recompute `nbSeg = int(span/resolution) + 1`. Returns the number emitted.

#### 2.6.6 `FindSolutions` (recursive core) — the box shrink (`:353-549`)

```
loop:                                                                     :377
    remember (aTB11,aTB12,aTB21,aTB22) = current ranges                   :379-382
    if (box1.IsOut(box2)) { out; break }                                  :385-389
    thin = (aT12-aT11 < myRes1) || (box1 thin in X,Y,Z at myTol)          :391-392
    ok = FindParameters(curve2, aTB21,aTB22, myTol2,myRes2,myPTol2,myResCoeff2,
                        box1, aT21,aT22)                                  :394-403
    if (!ok || thin) break                                                :404-407
    box2 = Bnd(curve2 over [aT21,aT22], myTol2)                           :411
    if (box1.IsOut(box2)) { out; break }                                  :412-416
    thin = (aT22-aT21 < myRes2) || (box2 thin in X,Y,Z at myTol)          :418-419
    ok = FindParameters(curve1, aTB11,aTB12, myTol1,myRes1,myPTol1,myResCoeff1,
                        box2, aT11,aT12)                                  :421-430
    if (!ok || thin) break                                                :432-435
    step1 = max((aTB12-aTB11)/250, myRes1);  step2 = max((aTB22-aTB21)/250, myRes2)   :438-448
    if (all four ends moved by less than their step) { bStop = true }      :450-454
    else box1 = Bnd(curve1 over [aT11,aT12], myTol1)                       :456-458
while (!bStop)

if (out) return                                                            :462-466
if (!thin):
    iCom = CheckCoincidence(aT11,aT12,aT21,aT22, myTol, myRes1)            :471
    if (iCom == 0) thin = true            // coincident ⇒ accept as a part :472-475
if (thin):
    if (iCom != 0):                        // not proven coincident: probe  :480-513
        aT1 = (aT11+aT12)/2;  P1 = geom1(aT1)
        proj.Init(myGeom2, aT21, aT22); proj.Perform(P1)
        bSol = proj.NbPoints() ? (proj.LowerDistance() <= myTol)
                               : P1.IsEqual(geom2((aT21+aT22)/2), myTol)
        if (!bSol) return
    out1 += (aT11,aT12);  out2 += (aT21,aT22);  return                     :515-519
if (!IsIntersection(aT11,aT12,aT21,aT22)) return                           :522-525
// split curve1 into 3 and recurse
box1 = Bnd(curve1 over [aT11,aT12], myTol1);  parentExtent = box1.SquareExtent()   :533-534
box2 = Bnd(curve2 over [aT21,aT22], myTol2)                                :537
aNb1 = SplitRangeOnSegments(aT11, aT12, myRes1, 3, seg1)                   :539
for each seg: box = Bnd(curve1 over seg, myTol1)
    if (!box.IsOut(box2) && (aNb1 == 1 || box.SquareExtent() < parentExtent))
        FindSolutions(seg, box, (aT21,aT22), box2, out1, out2)             :540-548
```

**The only termination guarantee is the `SquareExtent() < parentExtent` test at `:544` plus the
`myRes` floors inside `SplitRangeOnSegments`.** OCCT does not prove termination; port the guard
verbatim and add an explicit depth cap in our version (see §3, `EEParams::max_depth`) — that is a
deliberate, documented divergence, not a reproduction.

`FindParameters` (`:553-671`) — find the sub-range of a curve whose points are within `theTol` of
a box:

```
aCf = 0.6180339887498948482045868343656            // golden ratio        :572
aCBx = theCBox; aCBx.SetGap(gap + theTol)                                 :573-574
aMaxDt = (aT2 - aT1) * 0.01                                               :578
for i in {0 (forward from aT1), 1 (backward from aT2)}:                   :580
    aTB = (i==0 ? aT1 : aT2); aT = (i==0 ? aT2 : aTB1); aC = (i==0 ? +1 : -1)
    aDt = theRes; aDistP = 0; k = 1
    while (aC*(aT - aTB) >= 0):                    // march into the range :590
        aDist = PointBoxDistance(theCBox, curve(aTB))                      :592-593
        if (aDist > theTol):
            if (aDistP > 0):
                if (|aDistP - aDist| / aDistP < 0.1):        // slow approach
                    aDt = Resolution(curve, type, theResCoeff, k*aDist)
                    if (aDt < aMaxDt) { grow: k *= 2 }        // accelerate  :599-606
                else { k = 1; aDt = Resolution(curve, type, theResCoeff, aDist) }   :608-612
            aTB += aC*aDt                                                  :614
        else { found = true; break }                                       :617-619
        aDistP = aDist
    if (!found):
        if (i == 0) return false                   // whole curve outside   :626-630
        else { found = true; aTB = aTB1; aDt = aT2 - aTB1 }                :632-636
    if (aTB != (i==0 ? aT1 : aT2)):                // bisect the boundary   :639-660
        aTIn = aTB; aTOut = aTB - aC*aDt
        while (|aTIn - aTOut| > thePTol):
            aTB = aTOut + (aTIn - aTOut)*aCf
            if (aCBx.IsOut(curve(aTB))) aTOut = aTB; else aTIn = aTB
    (i==0) ? aTB1 = aTB : aTB2 = aTB
return true
```

The step is `Resolution(curve, type, coeff, aDist)` — **the parametric image of the current 3D
distance to the box**. On a circle that is `2*asin(dist/2R)`; on a BSpline it is the curve's own
resolution bound. Marching in raw parameter would be either intolerably slow or would skip the
target on high-curvature curves. `PointBoxDistance` (`:1423-1452`) is the ordinary
point-to-AABB Euclidean distance.

`CheckCoincidence(t11, t12, t21, t22, criteria, curveRes1)` (`:1150-1206`):

```
proj.Init(myGeom2, t21, t22)
aNb  = 10
aNb1 = SplitRangeOnSegments(t11, t12, curveRes1, aNb, ranges)              :1168
// 1. express pass: check the RIGHT end of ranges 1..aNb1-1
for i in 1..aNb1-1:  iErr = DistPC(ranges[i].Last(), geom1, criteria, proj, dmax, t2max)
                     if (iErr) return iErr                                 :1169-1179
if (aNb1 < aNb) return iErr        // ranges already below the resolution   :1183-1186
// 2. deep pass: golden-section MAXIMISATION over ranges 2..aNb1-1
for i in 2..aNb1-1:  iErr = FindDistPC(a, b, geom1, criteria, curveRes1, proj, dmax, t1max, t2max)
                     if (iErr) return iErr                                 :1189-1200
return iErr
// 0 = coincident, 1 = a point of C1 cannot be projected on C2, 2 = distance too big
```

`DistPC` (`:1332-1362`): project `C1(t1)` on `C2`; `NbPoints()==0 ⇒ iErr = 1`; otherwise
`aD = LowerDistance()`, `aT2 = LowerDistanceParameter()`, and
`iErr = 2` when `iC*(aD - criteria) > 0` (`iC = +1` when searching for a maximum, `-1` for a
minimum). `FindDistPC` (`:1210-1297`) is a golden-section search (`aGS` = same 0.61803… constant)
with `anEps = max(theEps, Epsilon(max(|a|,|b|))*10)` (`:1260`); when minimising and the bracket
returns "distance too small" it evaluates the bracket midpoint before returning (`:1282-1286`).

`IsIntersection(t11,t12,t21,t22)` (`:1060-1146`) — "do these two nearly-coincident sub-ranges
actually cross?":

```
aCoef = 1e5
if ((t12-t11) > aCoef*myRes1 && (t22-t21) > aCoef*myRes2) aCoef = 5000
else { aCoef = max(1., min((t12-t11)/myRes1, (t22-t21)/myRes2) / 100.) }   :1072-1085
aCriteria = (aCoef*myTol)^2                                                :1086-1087
compute the four endpoint square distances                                 :1089-1097
if ((d11_21 && d12_22 small) || (d11_22 && d12_21 small)):                 :1104
    if (aCoef == 1.) return true
    anAngleCriteria = 5.e-3                                                :1114
    take the two tangent angles of the matched pairs                       :1115-1130
    if (either angle is within 5e-3 of 0 or of π):                         :1132-1133
        proj.Init(myGeom2, t21, t22)
        iErr = FindDistPC(t11, t12, myGeom1, myTol, myRes1, proj, d, t1m, t2m, /*max*/false)
        return (iErr == 2)     // "distance too small" ⇒ a real intersection :1141-1142
return true
```

#### 2.6.7 `MergeSolutions` — how an overlap becomes an EDGE part (`:675-776`)

```
aRes1 = Resolution(curve1, type1, myResCoeff1, myTol)       // note: myTol, not myTol1  :695
aRes2 = Resolution(curve2, type2, myResCoeff2, myTol)                                   :696
dTR1 = 20*aRes1;  dTR2 = 20*aRes2                                                       :700-701
aType = TopAbs_VERTEX
for i in 1..n (skipping already-merged indices):
    (ai11,ai12) = ranges1[i]; (ai21,ai22) = ranges2[i]
    for j > i not merged:
        bCond = |ai12 - aj11| < dTR1  ||  aj11 in (ai11,ai12)  ||  ai11 in (aj11,aj12)
                || (bSplit2 && |aj12 - ai11| < dTR1)                                    :733-734
        if (bCond && bSplit2):
            bCond = ( (max(ai22,aj22) - min(ai21,aj21)) - ((ai22-ai21)+(aj22-aj21)) < dTR2 )
                    || aj21 in (ai21,ai22) || ai21 in (aj21,aj22)                       :737-740
        if (bCond) { union the two ranges on both curves; mark j merged }               :743-750
        else if (!bSplit2) { i = j; break }                                             :751-755
    if ( (|t11 - ai11| < myRes1 && |t12 - ai12| < myRes1)
      || (|t21 - ai21| < myRes2 && |t22 - ai22| < myRes2) ):                            :758-759
        aType = TopAbs_EDGE;  myCommonParts.Clear()                                     :761-762
    AddSolution(ai11, ai12, ai21, ai22, aType)                                          :765
    if (aType == TopAbs_EDGE) break                                                     :766-769
    if (bSplit2) ++i                                                                    :771-774
```

**The EDGE promotion rule is: the merged common range covers essentially the WHOLE of range1 or
the WHOLE of range2** (within `myRes`). A partial overlap of two edges — where neither edge is
fully inside the other — therefore produces a `VERTEX` part, not an `EDGE` part. The only way
partial overlap becomes a common block is by VE first splitting both edges at each other's
endpoints so that the middle pave blocks *do* span the full sub-range and share both vertices.
This is the single most important order dependency in the subsystem (see acceptance test T7).

Note the EDGE promotion **clears all previously found vertex parts** (`:762`) — documented
information loss.

`AddSolution` (`:780-822`) writes the part with the original (un-swapped) edge order and, for
`TopAbs_VERTEX`, calls `FindBestSolution` to pick the representative parameters.

`FindBestSolution(t11,t12,t21,t22, out t1, out t2)` (`:826-898`):

```
aSolCriteria   = 5.e-16                                                   :840
aTouchCriteria = 5.e-13                                                   :841
aRes1 = Resolution(curve1, type1, myResCoeff1, myTol)                     :845
aNbS  = SplitRangeOnSegments(t11, t12, 3*aRes1, 10, ranges)               :846-847
proj.Init(myGeom2, t21, t22)
for each sub-range:                                                       :854
    aD = myTol
    iErr = FindDistPC(a, b, myGeom1, aSolCriteria, myPTol1, proj, aD, t1m, t2m, /*max*/false)
    if (iErr != 1):
        if (aD < aDMin) { t1 = t1m; t2 = t2m; aDMin = aD; isSolFound = true }   :862-870
        if (aD < aTouchCriteria):                                          :872
            if (bTouch) { t12Touch = t1m; t22Touch = t2m; bTouchConfirm = true }
            else        { t11Touch = t1m; t21Touch = t2m; bTouch = true }
if (!isSolFound || bTouchConfirm):                                         :889
    t1 = (t11Touch + t12Touch) * 0.5                                       :891
    iErr = DistPC(t1, myGeom1, aSolCriteria, proj, aD, t2, -1)             :892
    if (iErr == 1) t2 = (t21Touch + t22Touch) * 0.5                        :893-896
```

`bTouchConfirm` is the **tangency** case: two distinct sub-ranges achieved a distance below
5e-13, so the "intersection" is a contact interval; the reported point is its midpoint.

#### 2.6.8 `ComputeLineLine` — the fully analytic special case (`:902-1056`)

```
aTol = myTol^2                                                             :904
anAngle = D1.Angle(D2);  IsCoincide = anAngle < Precision::Angular()  (1e-12)  :912-913
if (IsCoincide && L1.SquareDistance(L2.Location()) > aTol) return           :915-919
P11 = L1(t11); P12 = L1(t12)
if (!IsCoincide):                                                           :929
    O2 = L2((t21+t22)/2) (or L2.Location() when a bound is infinite)        :931-935
    V1 = vec(O2,P11) × D2;  V2 = vec(O2,P12) × D2                           :937-938
    IsCoincide = (|V1|² <= aTol && |V2|² <= aTol)                           :943
    if (!IsCoincide && V1·V2 > 0) return       // both endpoints on one side :945-949
if (IsCoincide):                                                            :956
    t21' = param of P11 on L2;  t22' = param of P12 on L2                   :958-959
    if both projections are wholly outside [t21,t22] return                  :961-965
    order them; then                                                         :967-970
    if (t21' >= t21):
        if (t22' <= t22)  { Range1 = (t11,t12); AllNullFlag = true; Range2 = (t21',t22') }
        else              { Range1 = (t11, t12-(t22'-t22)); Range2 = (t21', t22) }
    else                  { Range1 = (t11+(t21-t21'), t12); Range2 = (t21, t22') }
    type = TopAbs_EDGE; return                                               :972-993
// transversal
aCross = D1 × D2;  aDistLL = vec(L1.Loc, L2.Loc) · normalize(aCross)         :996-998
if (|aDistLL| > myTol) return                                                :999-1002
// FAST OUT: the edges share a sub-shape (a common vertex) ⇒ no new intersection
for v1 in E1: for v2 in E2: if (v1.IsSame(v2)) return                        :1004-1016
aSqSin = |aCross|²
t2 = ((D1 * (O1O2·D1)) - O1O2) · D2 / aSqSin;  if outside [t21,t22] return   :1018-1026
t1 = vec(L1.Loc, L2(t2)) · D1;                 if outside [t11,t12] return   :1028-1035
if (|L1(t1) - L2(t2)|² > aTol) return                                        :1037-1044
dt1 = IntTools_Tools::ComputeIntRange(myTol1, myTol2, anAngle)               :1047
dt2 = IntTools_Tools::ComputeIntRange(myTol2, myTol1, anAngle)               :1048
Range1 = (t1-dt1, t1+dt1); Range2 = (t2-dt2, t2+dt2); type = VERTEX;
VertexParameter1 = t1; VertexParameter2 = t2                                 :1050-1055
```

`IntTools_Tools::ComputeIntRange(tol1, tol2, angle)` (`%IT%/IntTools_Tools.cxx:783-804`):

```
if (|π/2 - angle| < Precision::Angular()) return tol2
a = (angle > π/2) ? (π - angle) : angle
return tol1 * tan(π/2 - a) + tol2 / sin(a)
```

This is the *only* place in the subsystem where a crossing angle is converted into a parametric
half-width. It diverges as the lines become parallel — deliberately: the intersection is smeared
along the line. There is **no curved analogue**; for curved pairs the common part's range comes
from the box shrink, and the tolerance from `MakeNewVertex` (§2.6.9).

#### 2.6.9 What EE creates: `PerformEE`'s result loop (`PF_3.cxx:285-556`)

```
aPB1->Range(aT11,aT12);  shrunk data or fallback → aTS11,aTS12, bIsPBSplittable1   :310-322
aPB2->Range(aT21,aT22);  likewise                                                  :324-336
collars: aR11=(aT11,aTS11) aR12=(aTS12,aT12) aR21=(aT21,aTS21) aR22=(aTS22,aT22)   :339
bAnalytical = (type(E1)==Line && type(E2)==Circle) || (Circle && Line)             :341-353
for each common part:
```

**`case TopAbs_VERTEX` (`:369-527`):**

```
if (!bIsPBSplittable1 || !bIsPBSplittable2) continue      // micro edge, no vertex  :370-373
IntTools_Tools::VertexParameters(aCPart, aT1, aT2)                                  :381
aTol = Precision::Confusion()          // 1e-7, used in PARAMETER space             :382
aCR1 = aCPart.Range1();  aCR2 = aCPart.Ranges2()(1)                                 :383-384
bIsOnPave[0] = IsOnPave1(aT1, aR11, aTol) || IsOnPave1(aR11.First(), aCR1, aTol)    :387-388
bIsOnPave[1] = IsOnPave1(aT1, aR12, aTol) || IsOnPave1(aR12.Last(),  aCR1, aTol)    :389-390
bIsOnPave[2] = IsOnPave1(aT2, aR21, aTol) || IsOnPave1(aR21.First(), aCR2, aTol)    :391-392
bIsOnPave[3] = IsOnPave1(aT2, aR22, aTol) || IsOnPave1(aR22.Last(),  aCR2, aTol)    :393-394
nV[0..1] = aPB1->Indices();  nV[2..3] = aPB2->Indices()                             :396-397
if (on a pave of BOTH blocks) continue         // it is already a shared vertex      :399-403
for j in 0..3 with bIsOnPave[j]:
    bIsOnPave[j] = ForceInterfVE(nV[j], (j<2 ? aPB2 : aPB1), aMEdges); isVExists |= …:406-417
BOPTools_AlgoTools::MakeNewVertex(E1, aT1, E2, aT2, aVnew);  aPnew = Pnt(aVnew)     :419-420
if (isVExists):                                                                     :422
    if (|BRepAdaptor_Curve(E1).Value(aT1) - BRepAdaptor_Curve(E2).Value(aT2)|
            > Precision::Intersection())   continue        // 1e-9: only touching    :430-436
    for j with bIsOnPave[j]:  UpdateVertex(nV[j], |P(nV[j]) - aPnew|);
                              myVertsToAvoidExtension.Add(nV[j])                     :440-451
aTolVnew = BRep_Tool::Tolerance(aVnew)                                              :454
if (bAnalytical):                                                                    :455
    aTolMin = (type(E1)==Line) ? (aCR1.Last()-aCR1.First())/2
                               : (aCR2.Last()-aCR2.First())/2                        :459-461
    aTolVnew = max(aTolVnew, aTolMin)                                                :462-465
// reject a new vertex too close to a vertex SHARED by both pave blocks
for each nV[2],nV[3] that also appears in {nV[0],nV[1]}:                             :468-509
    if (aPnew.SquareDistance(P(nVx)) < 100*(aTolVnew + tol(nVx))^2) → skip the part
InterfEE& ee = aEEs.Appended(); ee.SetIndices(nE1,nE2); ee.SetCommonPart(aCPart)     :513-516
myDS->AddInterf(nE1, nE2)                                                            :518
CoupleOfPaveBlocks cpb(aPB1, aPB2); cpb.SetIndexInterf(iX); cpb.SetTolerance(aTolVnew)
aMVCPB.Add(aVnew, cpb)                                                               :520-525
```

`aTolMin` always takes the range **on the line** (if `E1` is the line, use `Range1`; otherwise
`E1` is the circle so `E2` is the line, use `Ranges2()(1)`). A `Geom_Line` is unit-speed, so this
is a 3D length. Getting this backwards yields an angle in metres.

`IsOnPave1(tR, range, tol)` (`%IT%/IntTools_Tools.cxx:627-646`): true if `tR` is inside
`[first,last]`, else true if it is within `tol` of either bound.

`VertexParameters(part, t1, t2)` (`%IT%/IntTools_Tools.cxx:593-611`): midpoint of each range,
overridden by the stored `VertexParameter1/2` when it lies inside the range.

`MakeNewVertex(E1, t1, E2, t2, Vnew)` (`%BT%/BOPTools_AlgoTools_2.cxx:224-250`):

```
P1 = C1(t1); P2 = C2(t2);  d = |P1-P2|
tol = max(tol(E1), tol(E2)) + 0.5*d
P   = 0.5*(P1 + P2)
```

`ForceInterfVE(nV, aPB, theMEdges)` (`PF_3.cxx:828-910`) — the **explicit give-up-on-geometry
fallback**:

```
guards: edge HasSubShape(nV) / HasInterf(nV,nE) / HasInterfShapeSubShapes / nV is a pave of aPB
        ⇒ return true (already connected)                                          :837-856
nVx = SD(nV)
iFlag = ComputeVE(V(nVx), E(nE), aT, aTolVNew, fuzz)                                :867
if (iFlag == 0 || iFlag == -4):        // -4 == "distance too big"!                 :868
    record InterfVE, AddInterf                                                      :873-880
    nVx = UpdateVertex(nV, aTolVNew)   // grow the vertex to cover                  :883
    AppendExtPave({nVx, aT}); theMEdges.Add(nE)                                     :890-894
    if (Rank(nV) == Rank(nE)) warn self-interference                                :897-906
    return true
return false
```

Accepting `-4` means: the vertex is *further* from the edge than tolerance allows, and OCCT
attaches it anyway, inflating the vertex tolerance to `dist + tol(E)` so the model stays
consistent. Document this as a deliberate topology-over-geometry fallback.

**`case TopAbs_EDGE` (`:529-551`):**

```
if (aNbCPrts > 1) break                       // more than one part: not a clean overlap
if (!aPB1->HasSameBounds(aPB2)) break         // ← THE GATE
InterfEE& ee = aEEs.Appended(); ee.SetIndices(nE1,nE2); ee.SetCommonPart(aCPart)
myDS->AddInterf(nE1, nE2)
BOPAlgo_Tools::FillMap(aPB1, aPB2, aMPBLPB, alloc)    // symmetric adjacency
```

`HasSameBounds` (`%DS%/BOPDS_PaveBlock.cxx:148-159`) is unordered **index** equality of the two
paves. When the gate fails the coincidence is silently discarded at EE time; only
`ForceInterfEE` (`PF_3.cxx:997-1333`) may recover it later.

Post-treatment (`:558-589`):

```
BOPAlgo_Tools::PerformCommonBlocks(aMPBLPB, alloc, myDS, myContext)      :561
UpdateVerticesOfCB()                                                     :563
PerformNewVertices(aMVCPB, alloc, range)                                 :565
if (aMEdges non-empty):
    remove from aMEdges every edge that owns a pave block in aMVCPB      :571-582
    SplitPaveBlocks(aMEdges, /*theAddInterfs=*/false)                    :584
```

#### 2.6.10 Common-block construction

`BOPAlgo_Tools::PerformCommonBlocks(PB → list<PB>)` (`%BA%/BOPAlgo_Tools.cxx:107-187`):

```
MakeBlocks(aMPBLPB, blocks, alloc)                    // BFS components   :123
for each block:
    if (block.Extent() < 2) continue                                      :134-137
    aCB = the CB of the FIRST member that already has one (else new)      :146-172
    merge the face lists of ALL member CBs, deduplicated                  :150-166
    aCB->SetPaveBlocks(block);  aCB->SetFaces(faces)                      :176-177
    for each member: pDS->SetCommonBlock(member, aCB)                     :178-181
    aCB->SetTolerance(ComputeToleranceOfCB(aCB, pDS, ctx))                :184-185
```

`ComputeToleranceOfCB` (`%BA%/BOPAlgo_Tools.cxx:248-356`):

```
aPBR = theCB->PaveBlock1();  nE = aPBR->OriginalEdge()
aTolMax = BRep_Tool::Tolerance(E(nE))                                     :258-262
if (PaveBlocks().Extent() < 2 && Faces().IsEmpty()) return aTolMax        :266-269
aNbPnt = 11;  aPBR->Range(aT1,aT2);  aDt = (aT2-aT1)/(aNbPnt+1)           :271-278
for each member PB != aPBR:                                               :288-300
    proj = ctx->ProjPC(E(member->OriginalEdge()))
    for i in 1..11:  aT = aT1 + i*aDt;  proj.Perform(C(aT))
        if (proj.NbPoints()) aTolMax = max(aTolMax, tol(memberEdge) + proj.LowerDistance())
for each attached face: the analogous ProjPS loop                          :322-…
```

Sampling is over the **representative's pave range**, at 11 strictly interior points.

`UpdateVerticesOfCB` (`PF_3.cxx:959-993`): walk every pave block; for each distinct common block,
if `Tolerance() > 0`, `UpdateVertex(PaveBlock1()->Pave1().Index(), tolCB)` and the same for
`Pave2()` (`:985-990`). This is the mechanism that converts a coincidence *band* into vertex
tolerance, which then feeds `myIncreasedSS` → `RepeatIntersection` → `ForceInterf*`.

`BOPDS_CommonBlock` invariants (`%DS%/BOPDS_CommonBlock.hxx:33-140`): the first pave block is the
representative, normally the one with the minimal original-edge index — but
`SetRealPaveBlock` (`:130-133`) deliberately re-orders, so "min index" is a convention, not an
invariant (audit E3).

#### 2.6.11 New-vertex fusion after EE: `PerformNewVertices` (`PF_3.cxx:594-688`)

```
aTolAdd = myFuzzyValue / 2.                                               :607
TreatNewVertices(theMVCPB, aImages)                                       :612
for each fused image (aV, list of originals):                             :622
    iV = myDS->Append(ShapeInfo(VERTEX, aV))                              :631-634
    Box(iV).Add(Pnt(aV));  Box(iV).SetGap(tol(aV) + aTolAdd)              :637-639   ← fuzz/2!
    for each original aVx: theMVCPB[aVx].SetIndex(iV);
                           interference[IndexInterf()].SetIndexNew(iV)    :641-652
aMPBLI : pave block → list of new vertex indices                          :656-685
IntersectVE(aMPBLI, range, /*theAddInterfs=*/false)                       :687
```

`TreatNewVertices` (`PF_3.cxx:692-723`): build `map<vertex, requestedTol>` from the couples
(`:700-707`), call `BOPAlgo_Tools::IntersectVertices(map, fuzz, chains)` (`:711`), and
`BOPTools_AlgoTools::MakeVertex(chain, Vnew)` once per chain (`:715-722`).

`IntersectVertices` (`%BA%/BOPAlgo_Tools.cxx:1119-1205`):

```
if (aNbV <= 1) { emit the single vertex as a 1-element chain; return }     :1124-1132
aTolAdd = theFuzzyValue / 2.                                               :1135
BVH over boxes: gap = max(tol(V), requestedTol) + aTolAdd                  :1141-1155
BOPAlgo_PairVerticesSelector (predicate at :1072-1105, see §2.2.2)         :1160-1165
FillMap over the accepted pairs; MakeBlocks; one chain per component       :1172-1194
every vertex in no pair becomes its own 1-element chain                    :1197-1204
```

**Crucially, the new EE vertices are routed back through `IntersectVE` (`:687`)** so that the same
pave-placement and pave-block-splitting machinery handles them. A port must therefore have
`IntersectVE` as a reusable entry point taking `(paveblock → list<vertex>)`, not only the
iterator-driven form.

### 2.7 ORDER DEPENDENCIES — what stage N creates that stage N+1 needs

| producer | artefact | consumer | mechanism |
|---|---|---|---|
| VV | SD map `shape → fused shape` | VE | `HasShapeSD(nV,nVSD)` before solving (`PF_2.cxx:262-263`); the solve runs on the SD vertex (`PF_2.cxx:282`) |
| VV | `myInterfTB` entries | VE | `HasInterf(nV,nE)` and `HasInterfShapeSubShapes(nV,nE)` prefilters (`PF_2.cxx:176-184`) |
| VV | rebuilt pave blocks | VE, EE | `InitPaveBlocksForVertex` for every SD key (`PF_1.cxx:117-127`); `InitPaveBlocks` SD-maps every pave at creation (`%DS%/BOPDS_DS.cxx:465,495`) |
| VE | extra paves → split pave blocks | EE | `PerformEE` enumerates `PaveBlocks(nE)` (`PF_3.cxx:200-210`), so a missing VE pave means EE never sees the sub-block that could match |
| VE | shared vertex **indices** on both edges | EE | `bExpressCompute` (`PF_3.cxx:252`) and `HasSameBounds` (`PF_3.cxx:536`) are index equality — the EDGE/common-block path is *unreachable* without VE |
| VE + `SplitPaveBlocks` | possibly NEW SD vertices (`PF_2.cxx:500`) | EE | `UpdatePaveBlocksWithSDVertices()` at `PF:266` rewrites both paves of every block through the SD chain |
| EE | new vertices, grown tolerances | VE (re-entrant) | `PerformNewVertices` → `IntersectVE(..., false)` (`PF_3.cxx:687`) |
| EE | common-block tolerances | VV/VE (second pass) | `UpdateVerticesOfCB` (`PF_3.cxx:563`) → `myIncreasedSS` → `RepeatIntersection` (`PF:291`) re-runs VV and VE |
| EE + VF + EF | grown vertex tolerances | `ForceInterfEE` | runs only after everything else (`PF:298`), and only for pave-block pairs sharing both bounding vertices (`PF_3.cxx:1093-1120`) |

### 2.8 WHERE OCCT GIVES UP OR FALLS BACK — inventory

Every one of these is a documented outcome; a port must reproduce the *outcome*, not invent a
better one.

1. `ComputeVE` returns `-3` when the point has no perpendicular foot on the edge's curve; no
   fallback (`%IT%/IntTools_Context.cxx:522-526`). Curved edges hit this routinely (§2.3.3).
2. `ComputePE` falls back to the edge's vertices, then returns `-3`
   (`%IT%/IntTools_Context.cxx:468-493`).
3. `PerformVE` silently drops an interference whose parameter is not strictly inside any pave
   block's range (`PF_2.cxx:355-358`).
4. `PerformVE` skips edges whose first pave block is not splittable (micro edges) — no
   interference at all (`PF_2.cxx:192-197`).
5. `AnalyzeShrunkData` emits warnings (`AlertTooSmallEdge`, `AlertNotSplittableEdge`,
   `AlertBadPositioning`) and continues with a void shrunk box (`PF_3.cxx:793-823`).
6. `SplitPaveBlocks` drops a sub-block whose two paves have the same vertex index without welding
   anything (`PF_2.cxx:473-477`).
7. `IntTools_EdgeEdge::CheckData` refuses degenerate / non-geometric edges; the pair is skipped
   with a warning (`%IT%/IntTools_EdgeEdge.lxx:189-208`, `PF_3.cxx:294-301`).
8. Any exception inside the VE or EE worker becomes `BOPAlgo_AlertIntersectionFailed` and the pair
   is skipped (`PF_2.cxx:117-120`, `PF_3.cxx:113-116`).
9. `IsCoincident` is a 24-sample majority vote (`%IT%/IntTools_EdgeEdge.cxx:284-285`).
10. `MergeSolutions` discards every vertex part when it promotes to EDGE
    (`%IT%/IntTools_EdgeEdge.cxx:761-763`).
11. `PerformEE`'s EDGE case requires exactly one common part **and** identical bounding vertices,
    otherwise the coincidence is thrown away at EE time (`PF_3.cxx:529-540`).
12. `ForceInterfVE` accepts `iFlag == -4` — the geometric predicate said "too far" and OCCT
    attaches the vertex anyway, growing its tolerance (`PF_3.cxx:868`).
13. `BRepLib::BoundingVertex` for `n > 2` is mean-centre + max-radius, not a minimal enclosing
    sphere (`%TA%/BRepLib.cxx:3074-3122`).
14. `BRepLib::BoundingVertex` for `n < 2` returns leaving the outputs **untouched**
    (`%TA%/BRepLib.cxx:3019-3023`) — an uninitialised-output trap for a naive port.
15. `FindSolutions` recursion has no depth limit; termination rests on the
    `SquareExtent() < parent` guard and the `myRes` floors
    (`%IT%/IntTools_EdgeEdge.cxx:544`, `:1373`).
16. `IsIntersection`'s near-tangent branch returns `true` (keep recursing) whenever `FindDistPC`
    fails to project (`iErr == 1`), i.e. it errs toward more work
    (`%IT%/IntTools_EdgeEdge.cxx:1141-1142`).
17. The four `IsOnPave1` tests compare **parameters** against `Precision::Confusion()` = 1e-7, a
    3D-flavoured constant (`PF_3.cxx:382-394`) — an acknowledged unit mismatch that is safe only
    because those collars are already parametrically scaled.
18. Two different bounding-box gap policies coexist: `tol + Confusion` and `tol + fuzz/2`
    (§2.2.3).

---

## 3. DATA STRUCTURES AND C++ DECLARATIONS FOR OUR PORT

New files. Nothing here mutates `BRep`; the arena is built *from* BReps and consumed by later
stages. Names extend the `Bds*` sketch in `kb/ARCHITECTURE_v2.md:47-62`.

### 3.1 `src/brep_bds.h` — the arena

```cpp
#pragma once
#include "point.h"
#include "nurbscurve.h"
#include "brep.h"
#include <vector>
#include <array>
#include <map>
#include <set>

namespace session_cpp {

enum class BdsType { Vertex = 0, Edge = 1, Face = 2, Solid = 3 };

// ---- the flat arena -------------------------------------------------------
struct BdsShape {
    BdsType type = BdsType::Vertex;
    int operand   = -1;      // 0 = A, 1 = B, -1 = synthesised
    int source     = -1;     // index into the operand BRep's own table (-1 for new shapes)
    std::vector<int> subs;   // sub-shape arena indices (edge -> its vertices)
    int flag       = -1;     // >= 0 means "degenerate"; 0 IS a valid flag  (%DS%/BOPDS_ShapeInfo.lxx)
    int reference  = -1;     // edge: index into pave_block_pool           (-1 = none)
    std::array<double,6> box{};   // AABB, already inflated by `gap`
    double gap = 0.0;
    bool has_box = false;
};

// ---- geometry carriers ----------------------------------------------------
struct BdsVertex {                 // arena vertices are OWNED here, never in BRep
    Point  p;
    double tol = 0.0;
};

struct BdsEdgeGeom {
    NurbsCurve c;                  // 3D curve, arbitrary parameterisation
    double t0 = 0.0, t1 = 0.0;     // the edge's own range
    double tol = 0.0;
    bool   degenerate = false;
    // analytic identity when known (Law 3): -1 unknown, else a CurveKind
    int    kind = -1;              // Line=0 Circle=1 Ellipse=2 Hyperbola=3 Parabola=4 Bezier=5 BSpline=6 Other=7
    double radius = 0.0;           // Circle: R;  Ellipse: major radius
};

// ---- paves and pave blocks ------------------------------------------------
struct BdsPave {
    int    vertex = -1;            // arena vertex index
    double t      = 0.0;           // parameter on the ORIGINAL edge
    bool operator<(const BdsPave& o) const { return t < o.t; }   // parameter ONLY
};

struct BdsPaveBlock {
    int  original_edge = -1;       // arena index of the edge this block belongs to
    int  edge          = -1;       // arena index of the materialised split edge, -1 until built
    BdsPave p1, p2;                // bounding paves, p1.t < p2.t
    std::vector<BdsPave> ext;      // extra paves waiting to be applied
    std::set<int> ext_fence;       // fence BY VERTEX INDEX (%DS%/BOPDS_PaveBlock.cxx:167-174)
    // shrunk data
    double ts0 = -99.0, ts1 = -99.0;
    std::array<double,6> sbox{};
    bool   has_shrunk   = false;   // == "sbox is not void"
    bool   is_splittable = false;
    int    common_block = -1;      // index into the common-block pool, -1 if none

    void range(double& a, double& b) const { a = p1.t; b = p2.t; }
    void indices(int& a, int& b) const { a = p1.vertex; b = p2.vertex; }
    bool has_same_bounds(const BdsPaveBlock& o) const {
        return (p1.vertex == o.p1.vertex && p2.vertex == o.p2.vertex)
            || (p1.vertex == o.p2.vertex && p2.vertex == o.p1.vertex);
    }
    bool is_to_update() const { return !ext.empty(); }
    void append_ext_pave(const BdsPave& pv) {           // fenced
        if (ext_fence.insert(pv.vertex).second) ext.push_back(pv);
    }
    void append_ext_pave_unfenced(const BdsPave& pv) { ext.push_back(pv); }
    void remove_ext_pave(int vertex);                   // removes ALL paves of that vertex
    // OCCT BOPDS_PaveBlock::Update. with_bounds == theFlag.
    // Returns false and CLEARS ext when the total pave count is <= 1 (annihilation).
    bool update(std::vector<BdsPaveBlock>& out, bool with_bounds = true);
};

struct BdsCommonBlock {
    std::vector<int> pave_blocks;  // indices into the pave-block pool; [0] is the representative
    std::vector<int> faces;        // arena face indices
    double tol = 0.0;
};

// ---- interferences --------------------------------------------------------
enum class BdsInterfKind { VV, VE, EE, VF, EF, FF };

struct BdsInterf {
    BdsInterfKind kind;
    int a = -1, b = -1;            // arena indices, unordered for the table key
    int new_vertex = -1;           // fused/created vertex, -1 if none
    double t_a = 0.0, t_b = 0.0;   // VE: parameter on b; EE: the two vertex parameters
    // EE common part
    int  part_type = -1;           // -1 none, 0 VERTEX, 1 EDGE
    double r_a0 = 0, r_a1 = 0, r_b0 = 0, r_b1 = 0;
};

// unordered pair key
struct BdsPair {
    int i = -1, j = -1;
    BdsPair() = default;
    BdsPair(int a, int b) : i(a < b ? a : b), j(a < b ? b : a) {}
    bool operator<(const BdsPair& o) const { return i != o.i ? i < o.i : j < o.j; }
    bool operator==(const BdsPair& o) const { return i == o.i && j == o.j; }
};

// ---- the DS ---------------------------------------------------------------
class BdsDS {
public:
    std::vector<BdsShape>    shapes;
    std::vector<BdsVertex>   vertices;      // parallel to shapes where type == Vertex
    std::vector<BdsEdgeGeom> edges;         // parallel to shapes where type == Edge
    std::vector<std::vector<BdsPaveBlock>> pave_block_pool;   // per edge `reference`
    std::vector<BdsCommonBlock> common_blocks;
    std::vector<BdsInterf>   interfs;

    int    nb_source_shapes = 0;            // shapes appended after this index are NEW
    double fuzzy = 1e-7;                    // clamped to >= 1e-7 at set time

    // same-domain map, chain-walked to a fixpoint (%DS%/BOPDS_DS.cxx:1229-1254)
    std::map<int,int> shapes_sd;
    std::set<BdsPair> interf_table;
    std::set<int>     interfered;
    std::set<int>     increased_ss;            // vertices whose tolerance grew
    std::set<int>     verts_to_avoid_extension;

    bool is_new_shape(int i) const { return i >= nb_source_shapes; }
    bool has_shape_sd(int i, int& sd) const;   // walks the chain, returns false if none
    int  same_domain(int i) const;             // fixpoint, returns i when unmapped
    void add_shape_sd(int i, int sd) { if (i != sd) shapes_sd[i] = sd; }
    bool add_interf(int a, int b);             // false if already present
    bool has_interf(int a, int b) const { return interf_table.count(BdsPair(a,b)) != 0; }
    bool has_interf(int i) const { return interfered.count(i) != 0; }
    // any_of over shapes[b].subs
    bool has_interf_shape_subshapes(int a, int b) const;
    int  operand_of(int i) const { return shapes[i].operand; }

    std::vector<BdsPaveBlock>&       pave_blocks(int edge);        // creates on demand
    const std::vector<BdsPaveBlock>& pave_blocks(int edge) const;
    void init_pave_blocks(int edge);
    void init_pave_blocks_for_vertex(int vertex);                  // via a vertex->edges map
    void update_pave_block_with_sd_vertices(BdsPaveBlock& pb);
    void update_pave_blocks_with_sd_vertices();
};

} // namespace session_cpp
```

### 3.2 `src/brep_curvetool.h` — the geometric primitives the stages need

These four functions are the entire geometric contract. **Every one of them is model-space in and
model-space out; none takes a UV domain.**

```cpp
#pragma once
#include "nurbscurve.h"
#include "point.h"
#include <array>

namespace session_cpp {

// OCCT ResolutionCoeff + Resolution (%IT%/IntTools_EdgeEdge.cxx:1486-1607).
// Returns the parameter step that corresponds to a 3D distance `r3d` on `c` over [t0,t1].
//   Line   : r3d
//   Circle : (r3d/(2R) <= 1) ? 2*asin(r3d/(2R)) : 2*PI
//   Ellipse: r3d / major_radius
//   NURBS  : r3d / max|C'| over the range  (== OCCT's Geom_BSplineCurve::Resolution bound)
//   other  : r3d * min over 30 samples of (dt / |dP|)
double curve_resolution(const NurbsCurve& c, int kind, double radius,
                        double t0, double t1, double r3d);

// Conservative 3D box of c over [t0,t1], inflated by `gap`.
// MUST be an enclosure, not a sample hull: use the control-polygon AABB of the trimmed
// curve (valid for positive weights) unioned with the two endpoints.
std::array<double,6> curve_box(const NurbsCurve& c, double t0, double t1, double gap);

// OCCT GeomAPI_ProjectPointOnCurve semantics (%GA%/GeomAPI_ProjectPointOnCurve.cxx:131-153
// over %EX%/Extrema_GGExtPC.hxx:503-525).
// Returns the number of INTERIOR stationary points of |P - C(t)|^2 in (t0,t1); fills
// `t`/`dist` with the closest of them. Returns 0 when there is none — it MUST NOT clamp
// to an endpoint. Endpoints are added only when |P - C(end)|^2 < 1e-14, mirroring
// %EX%/Extrema_GGExtPC.hxx:474-501.
// Implementation: sample g(t) = (C(t)-P).C'(t) on a grid of step curve_resolution(...,tol),
// bracket sign changes, refine with safeguarded Newton to |dt| < 1e-13.
int proj_pc(const NurbsCurve& c, double t0, double t1, const Point& p,
            double& t, double& dist);

// Total turning of the tangent over 10 uniform samples (%IT%/IntTools_EdgeEdge.cxx:1611-1638).
double curve_deflection(const NurbsCurve& c, double t0, double t1);

// BRepLib::FindValidRange (%TA%/BRepLib_1.cxx:173-247) + findNearestValidPoint (:31-164).
bool find_valid_range(const NurbsCurve& c, double tol_e,
                      double tv0, const Point& p0, double tolv0,
                      double tv1, const Point& p1, double tolv1,
                      double& first, double& last);

// Arc length of c over [a,b] to the given parametric accuracy (GCPnts_AbscissaPoint::Length).
double curve_length(const NurbsCurve& c, double a, double b, double ptol);

} // namespace session_cpp
```

### 3.3 `src/brep_interf.h` — the three stages

```cpp
#pragma once
#include "brep_bds.h"
#include <map>
#include <vector>

namespace session_cpp {

// ---------- shared predicates ----------------------------------------------
// %BT%/BOPTools_AlgoTools.cxx:1772-1794.  0 == interfering.
int  bds_compute_vv(const BdsVertex& a, const BdsVertex& b, double fuzz);

// %TA%/BRepLib.cxx:3013-3123.  n<2 leaves the outputs UNTOUCHED — callers must handle n==1.
void bds_bounding_vertex(const std::vector<BdsVertex>& cluster, Point& centre, double& tol);

// %IT%/IntTools_Context.cxx:499-541.  Returns 0, -1 (degenerate), -2 (non-geometric),
// -3 (no projection), -4 (distance too big).  `tol_new` = dist + tol(edge).
int  bds_compute_ve(const BdsVertex& v, const BdsEdgeGeom& e,
                    double& t, double& tol_new, double fuzz);

// %IT%/IntTools_Context.cxx:437-495.  Endpoint fallback included.
int  bds_compute_pe(const Point& p, double tol_p, const BdsEdgeGeom& e,
                    const BdsVertex& v0, const BdsVertex& v1,
                    double t_v0, double t_v1, double& t, double& dist);

// ---------- stage 1 ---------------------------------------------------------
struct BdsVVResult { int n_pairs = 0, n_clusters = 0, n_fused = 0; };
BdsVVResult bds_perform_vv(BdsDS& ds, const std::vector<BdsPair>& candidates);

// The fusion primitive: mutates the arena vertex IN PLACE when the cluster already has an SD
// image (OCCT's BRep_TVertex mutation, PF_1.cxx:167-171); otherwise appends a new arena vertex.
// Returns the surviving arena index.
int  bds_make_sd_vertices(BdsDS& ds, const std::vector<int>& cluster, bool add_interfs = true);

// %BA%/BOPAlgo_Tools.cxx:1119-1205.  Chains of interfering vertices with per-vertex requested
// tolerances; every non-interfering vertex is its own 1-element chain.
void bds_intersect_vertices(const std::vector<int>& verts,
                            const std::vector<double>& requested_tol,
                            const BdsDS& ds, double fuzz,
                            std::vector<std::vector<int>>& chains);

// PF_10.cxx:105-162.  Returns the (possibly new) arena vertex index.
int  bds_update_vertex(BdsDS& ds, int v, double tol_new, bool non_destructive = false);

// ---------- shrunk range ----------------------------------------------------
// %IT%/IntTools_ShrunkRange.cxx:107-191 + PF_3.cxx:766-824.
void bds_fill_shrunk_data(BdsDS& ds, BdsPaveBlock& pb);
void bds_fill_shrunk_data(BdsDS& ds, BdsType t1, BdsType t2,
                          const std::vector<BdsPair>& candidates);

// ---------- stage 2 ---------------------------------------------------------
struct BdsVEResult { int n_pairs = 0, n_paves = 0, n_dropped_no_block = 0, n_welds = 0; };
BdsVEResult bds_perform_ve(BdsDS& ds, const std::vector<BdsPair>& candidates);

// Re-entrant core, also used by EE's new vertices (PF_3.cxx:687).
// Key: a pave-block LOCATOR (edge arena index, block position); value: vertex arena indices.
struct BdsPBRef { int edge = -1; int block = -1; };
BdsVEResult bds_intersect_ve(BdsDS& ds,
                             const std::vector<std::pair<BdsPBRef,std::vector<int>>>& pairs,
                             bool add_interfs = true);

// PF_2.cxx:419-626.  Splits the flagged edges' blocks by their extra paves, recomputes shrunk
// data, welds the vertices of any block that lost its valid range, and regroups common blocks.
void bds_split_pave_blocks(BdsDS& ds, const std::vector<int>& edges, bool add_interfs);

// PF_3.cxx:828-910.  Accepts flag 0 AND flag -4 (deliberate topology-over-geometry fallback).
bool bds_force_interf_ve(BdsDS& ds, int v, BdsPBRef pb, std::vector<int>& touched_edges);

// ---------- stage 3 ---------------------------------------------------------
struct BdsCommonPart {
    int    type = 0;                       // 0 VERTEX, 1 EDGE
    double a0 = 0, a1 = 0;                 // range on edge A
    double b0 = 0, b1 = 0;                 // range on edge B
    double ta = 0, tb = 0;                 // representative parameters (VERTEX only)
    bool   all_null = false;               // ComputeLineLine's AllNullFlag
};

struct BdsEEParams {
    double fuzzy = 1e-7;
    bool   quick_coincidence = false;      // UseQuickCoincidenceCheck
    int    max_depth = 64;                 // OUR addition: OCCT has no depth cap (§2.8.15)
};

// The IntTools_EdgeEdge port.  `ka`/`kb` are BdsEdgeGeom::kind.
// Returns false when the data is unusable (degenerate / non-geometric) — the caller emits a
// warning and skips the pair, exactly as PF_3.cxx:294-301 does.
bool bds_edge_edge(const BdsEdgeGeom& ea, double a0, double a1,
                   const BdsEdgeGeom& eb, double b0, double b1,
                   const BdsEEParams& prm,
                   std::vector<BdsCommonPart>& parts);

struct BdsEEResult { int n_pairs = 0, n_vertex_parts = 0, n_edge_parts = 0,
                     n_new_vertices = 0, n_common_blocks = 0, n_failed = 0; };
BdsEEResult bds_perform_ee(BdsDS& ds, const std::vector<BdsPair>& candidates);

// %BA%/BOPAlgo_Tools.cxx:107-187 and :248-356.
void   bds_perform_common_blocks(BdsDS& ds,
                                 const std::vector<std::pair<BdsPBRef,BdsPBRef>>& adjacency);
double bds_compute_tolerance_of_cb(const BdsDS& ds, const BdsCommonBlock& cb);
void   bds_update_vertices_of_cb(BdsDS& ds);          // PF_3.cxx:959-993

} // namespace session_cpp
```

### 3.4 Notes binding the declarations to OCCT behaviour

- `BdsPaveBlock::update` must return the **annihilation** case (`aNb <= 1` ⇒ clear and emit
  nothing, `%DS%/BOPDS_PaveBlock.cxx:262-268`) as a distinguishable outcome, because downstream
  reads "edge with an empty block list" as *deleted*.
- Sorting paves uses **parameter only** (`%DS%/BOPDS_Pave.hxx:63-65`). Use `std::stable_sort` so
  equal parameters keep insertion order; OCCT uses `std::sort` and tolerates the instability only
  because equal-parameter blocks are filtered afterwards. Documented, intentional divergence.
- The interference table key is unordered; the *candidate list* key must be normalised `(min,max)`
  and the candidate list must be **stably sorted** before iteration — that is OCCT's only
  determinism source (`%DS%/BOPDS_Iterator.cxx:200`).
- Two box-gap policies: `tol + 1e-7` for SD/updated vertices, `tol + fuzzy/2` for new EE/EF
  vertices and shrunk-range boxes. Keep both; do not unify.
- `bds_bounding_vertex` for `n > 2` must sort points lexicographically before summing.

---

## 4. WHAT OUR CODE DOES TODAY, AND EXACTLY WHERE IT DIVERGES

All paths relative to `/home/petras/code/code_rust/session/session_cpp/src`.

### 4.0 The structural fact

`brep.h:27-30` and `brep.h:32-37`:

```cpp
struct BRepVertex { int point_index = -1; std::vector<int> edge_indices; };
struct BRepEdge   { int curve_3d_index = -1; int start_vertex = -1; int end_vertex = -1;
                    std::vector<int> trim_indices; };
```

**Neither carries a tolerance.** Every predicate in OCCT's VV/VE/EE is
`dist <= tol(a) + tol(b) + fuzz`. With no per-entity tolerance our kernel cannot express any of
them, so every one of them has been replaced by a global or model-relative epsilon. That is the
root of the divergences below, and it is why the port needs its own arena (§3.1) rather than
extra fields on `BRep`.

### 4.1 Stage 1 (VV) — **absent**; replaced by coordinate hashing

There is no vertex-fusion stage. Vertex identity is minted inside the splitter by
`find_or_add_vertex`, `brep.cpp:3692-3761`:

1. `brep.cpp:3693-3722` — snap to the operand's own topology vertices if within `orig_tol2`.
2. `brep.cpp:3723-3741` — else snap to a scaffold "pave" point if within `pave_tol2`.
3. `brep.cpp:3742-3757` — else quantise with `q6` (`brep.cpp:2897`,
   `llround(x * 1e6)` — a **fixed 1e-6 world grid**), scan the 3×3×3 neighbouring cells and accept
   the first seed whose squared distance is `<= 1e-12` (i.e. a **hard-coded 1e-6 absolute
   radius**), else mint a new vertex.

Divergences:

- **D1.1 (no cluster, no fusion).** OCCT computes connected components and one fused vertex per
  component (`PF_1.cxx:101-113`). Ours accepts the *first* seed found in scan order
  (`brep.cpp:3752-3755`) and never moves it. Two vertices 1.4e-6 apart may or may not fuse
  depending on which cell they land in; three vertices in a chain can fuse pairwise into two
  different survivors.
- **D1.2 (no smallest-enclosing-sphere).** Nothing computes `BRepLib::BoundingVertex`. The
  surviving vertex keeps its own position, so G2's containment guarantee simply does not hold —
  and cannot, because there is no tolerance to hold it in.
- **D1.3 (absolute 1e-6 grid).** `q6` at `brep.cpp:2897` is scale-free: on a 1 mm feature it is a
  1e-3 relative tolerance; on a 10 m model it is 1e-7. OCCT's threshold is always
  `tol(a)+tol(b)+fuzz`, both of which travel with the entity.
- **D1.4 (model-relative fallbacks).** `orig_tol2` is `(diag * 5e-4)²` capped at `(min_pair/3)²`
  (`brep.cpp:3644-3660`); `pave_cap_tol` is `0.7 * scaf->tol3` on the scaffold path
  (`brep.cpp:3564`) or `max(1e-9, diag * 2e-3)` on the quadric path (`brep.cpp:3575`). All three
  are functions of the *model bounding box*, so adding a far-away second solid changes vertex
  identity of the first.
- **D1.5 (no SD map).** There is no `shapes_sd` equivalent, so nothing downstream can ask "which
  vertex did this one become". `orig_to_pave` (`brep.cpp:3663-3678`) is the closest analogue and
  is a one-shot nearest-pave lookup, not a chain.

### 4.2 Stage 2 (VE) — **absent**; two partial substitutes, both post-hoc repairs

**(a) `BRep::imprint_edges(double tol, bool mated_too)` — `brep.cpp:5038`.**

- Default `tol = diag * 1e-6` (`brep.cpp:5049`) — model-relative, not per-entity.
- Only **under-mated** edges are considered (`brep.cpp:5075-5077` comment; the guard mirrors
  `co_refine`'s `cand`), so an edge that already has two trims is never split even when a vertex
  genuinely lies on it.
- `pave_tol = max(tol, edge_bbox_diag * 2e-4)` (`brep.cpp:5100-5101`) — again a size-relative
  epsilon, and it is used *both* as an end-guard and as a dedup radius.
- The projection is `C.closest_parameter(V)` (`brep.cpp:5109`), i.e. a **clamping** closest point,
  followed by `C.point_at(tc).distance(V) > tol` and a fractional guard
  `frac <= 1e-6 || frac >= 1-1e-6` (`brep.cpp:5111-5113`).

  Divergence **D2.1**: OCCT's `ComputeVE` returns `-3` and creates nothing when there is no
  interior perpendicular foot (`%IT%/IntTools_Context.cxx:522-526`); ours clamps to the nearest
  end and then rejects on a *fractional* guard. On an arc, a vertex slightly past the arc's span
  is clamped to the arc end, the distance test may pass, and only the `frac` guard saves us — a
  guard expressed in normalised parameter, which for a knot-vector-dependent NURBS
  parameterisation has no geometric meaning.
- **D2.2**: no pave is recorded anywhere. The edge is physically split and new topology vertices
  are minted; nothing records "vertex V lies on edge E at parameter t". Consequently the EE stage
  (§4.3) has no index-based coincidence to test — G5 fails by construction.
- **D2.3**: no shrunk range, no `is_splittable`. OCCT refuses VE on an edge whose valid range is
  too short to split (`PF_2.cxx:192-197`) and *welds* the endpoints of a sub-block that loses its
  valid range (`PF_2.cxx:491-507`). We have neither; a split that produces a micro fragment simply
  produces a micro fragment.
- **D2.4**: no vertex-tolerance feedback. OCCT sets `tol(V) := max(tol(V), dist + tol(E))`
  (`%IT%/IntTools_Context.cxx:534` → `PF_2.cxx:338`), guaranteeing G7. We move the *vertex* instead
  (the snapping in `find_or_add_vertex`), which changes geometry rather than recording uncertainty.

**(b) `edge_paves`, `brep.cpp:3782-3800`.** Per original edge, the scaffold pave points that lie on
it, used to subdivide boundary runs so copies of one physical edge share endpoints. This is the
closest thing to a pave list we have, but it is keyed by 3D position with `ptol = pave_cap_tol`
and is built only when a scaffold exists (`brep.cpp:3785`), i.e. it is empty for pure coincidence
and for the quadric path.

### 4.3 Stage 3 (EE) — **no transversal stage at all**; one sampled-coincidence repair

- **D3.1 — There is no edge/edge *crossing* computation anywhere in the kernel.**
  `Closest::curve_curve` (`closest.cpp:108-168`) exists and returns the single global minimum via
  a `max(40, cv_count*8)` grid seed plus 64 Newton steps, but nothing in `brep.cpp` calls it for
  interference. It cannot serve as an EE solver regardless: it returns **one** extremum, whereas
  two circles cross at two points (acceptance test T6) and a line can cut a NURBS at many.
- **D3.2 — The only coincidence pass is sampled and one-directional.**
  `BRep::co_refine_coincident_edges(double tol)`, `brep.cpp:5388`:
  - `tol` defaults to `diag * 5e-3` (`brep.cpp:5399`) — a *five-per-mille of the model* tolerance,
    six orders of magnitude looser than OCCT's `tol(E1)+tol(E2)+fuzz`.
  - candidates are only edges with `< 2` trims (`brep.cpp:5420-5422`) — a repair filter, not an
    interference stage.
  - `NS = 96` uniform samples per edge (`brep.cpp:5429`), and `subset_of` accepts when *every*
    sample of `ej` is within `tol` of `ei`'s **polyline** (`brep.cpp:5442-5443`). This is a
    directed Hausdorff on chords: it conflates true coincidence with sampling sag, which is why
    the comment at `brep.cpp:5424-5428` has to argue about the 0.034 sag of a spiric.
  - it splits `ei` at the *endpoints* of coincident open arcs and never creates a common block or
    a shared entity.
- **D3.3 — Coincidence is resolved by merging trims after the fact.** The within-operand pass at
  `brep.cpp:4894-4990` groups 1-trim edges by their **welded vertex-index pair**
  (`brep.cpp:4906-4914`), samples 5 interior points of one against the 16-point polyline of the
  other, and accepts when all are inside `tube = 1.5*(dev_a + dev_b) + 1e-9`
  (`brep.cpp:4944-4952`), then repoints trims (`brep.cpp:4968-4970`). This is structurally the
  closest thing we have to OCCT's `HasSameBounds` + `PerformCommonBlocks`, and it shows the design
  is reachable — but `edge_dev` (`brep.cpp:4788-4791` region) is a *lift deviation* recorded
  ad hoc, not an entity tolerance, and the whole pass is gated on `!pave_pts.empty()`
  (`brep.cpp:4900`), i.e. it does not run without a scaffold.
- **D3.4 — Final fallback is a global Hausdorff sew.** `BRep::sew_coincident_edges`,
  `brep.cpp:7074`, `tol = diag * 5e-3` (`brep.cpp:7092`), `NS = 64` (`brep.cpp:7097`), candidates
  = edges with `< 2` trims (`brep.cpp:7105`). ARCHITECTURE_v2 already rules this out
  (`kb/ARCHITECTURE_v2.md:129`, "No sewing. No tolerance-based matching. No fuzzy joins.").
- **D3.5 — `brep_commonblock` is not wired in.** `brep_commonblock.h/.cpp` implements
  `cb_split_chain`, `cb_tolerance` (a faithful `ComputeToleranceOfCB` port,
  `brep_commonblock.cpp:150`) and `cb_perform_common_blocks`
  (`brep_commonblock.cpp:186`), but `grep` finds **no caller** outside the module and its own
  header declares it "reads BReps, never mutates kernel state" (`brep_commonblock.h:10`). It is
  chain-vs-operand, not pave-block-vs-pave-block, so it cannot implement G9/G10 as written; its
  `cb_tolerance` is directly reusable by `bds_compute_tolerance_of_cb`.

### 4.4 Domain-relative tolerances that VV/VE/EE must make irrelevant

For the record, since these stages are the place where the fix becomes structural — none of the
following may appear in any VV/VE/EE code path:

| constant | site | why it is wrong here |
|---|---|---|
| `q6` = `llround(x*1e6)` | `brep.cpp:2897` | absolute world grid; vertex identity depends on model units |
| `1e-12` squared accept | `brep.cpp:3754` | hard-coded 1e-6 absolute radius |
| `orig_tol2 = (diag*5e-4)²` | `brep.cpp:3644-3660` | model-bbox relative |
| `pave_cap_tol = 0.7*tol3` / `diag*2e-3` | `brep.cpp:3564`, `:3575` | model-bbox relative |
| `eps_border = min(uv range)*2e-3` | `brep.cpp:4350` | **UV-domain relative** — a 4× padded domain inflates it 4× |
| `scaf_forced_eps` | `brep.cpp:4280` | UV-domain relative, with a UV-range cap `min_range*1.3e-1` |
| `tol = diag*5e-3` | `brep.cpp:5399`, `:7092` | model-bbox relative |
| `tol = diag*1e-6` | `brep.cpp:5049` | model-bbox relative |
| `pave_tol = edge_diag*2e-4` | `brep.cpp:5100-5101` | edge-size relative |

Replacements: `tol(a) + tol(b) + fuzz` in 3D everywhere (G1, G7, §2.2.2, §2.3.3), and
`curve_resolution(...)` (§3.2) wherever a 3D distance must become a parameter step.

---

## 5. ACCEPTANCE TESTS

Every test states operands with analytically known answers and an invariant checkable without a
reference kernel. Unless stated, `fuzz = 1e-7`, vertex tolerance `1e-7`, edge tolerance `1e-7`.

### T1 — VV two-ball fusion is the exact smallest enclosing sphere

**T1a (containment case).** `V1 = (0,0,0)` tol `1e-3`; `V2 = (5e-4,0,0)` tol `2e-3`.
`dR = 1e-3`, `D = 5e-4 <= dR` ⇒ expect `C = (5e-4,0,0)`, `tol = 2e-3` exactly.
**Invariant:** `C == P(V2)` bitwise and `tol == tol(V2)` bitwise.

**T1b (general case).** `V1 = (0,0,0)` tol `1e-3`; `V2 = (1,0,0)` tol `2e-3`.
Analytic: `T = 0.5*(2e-3 + 1e-3 + 1) = 0.5015`, `C = (0.5005, 0, 0)`.
**Invariants (all oracle-free):** (i) `|C-P1| + tol1 == T` and `|C-P2| + tol2 == T` to within 4
ulp — both balls internally tangent; (ii) no sphere of radius `T - 1e-12` centred anywhere
contains both balls (check by a 1-D scan along the axis).

**T1c (n > 2).** Three vertices at `(0,0,0)`, `(1,0,0)`, `(0,1,0)`, tolerances `1e-3, 2e-3, 3e-3`.
Expect `C = (1/3, 1/3, 0)` and `T = max_i(|C-Pi| + Ri)`.
**Invariant:** containment for all three, plus the result is **bitwise identical** for all 6 input
permutations (this is what the lexicographic sort at `%TA%/BRepLib.cxx:3090` buys).

### T2 — VV determinism and idempotence

Five vertices forming a chain, consecutive gaps `1.5e-7` with tolerances `1e-7` so that each
consecutive pair interferes (`1.5e-7 < 1e-7+1e-7+1e-7`) but the ends do not.
**Invariants:** (i) exactly one fused vertex (one connected component, G3); (ii) re-running VV on
the arena produces zero new fusions (G4); (iii) the result is identical under any permutation of
the candidate pair list *after* the stable sort.

### T3 — VE on a circular arc, exact hit (curved focus)

Edge: circle radius `R = 2`, centre origin, XY plane, arc parameter range `[0, π/2]`, `tol(E)=1e-7`.
Vertex `V = (√2, √2, 0)` (exactly `θ = π/4`), `tol(V)=1e-7`.
**Expected:** one pave at `t = π/4`; `tol_new = dist + tol(E) = 0 + 1e-7`; two pave blocks
`[0, π/4]` and `[π/4, π/2]`.
**Invariants:** (i) `|t - π/4| < 1e-12`; (ii) `dist(P(V), C(t)) <= tol(V)` (G7); (iii) the paves
of the edge sort to `{0, π/4, π/2}` with no duplicates and cover the range contiguously (G6);
(iv) the pave's vertex index equals the arena index of `V` — not a copy (G5).

### T4 — VE must NOT clamp: vertex just past the arc end

Same arc. `V = (2, -1e-3, 0)`, `tol(V) = 1e-7`. The perpendicular foot on the **full circle** is at
`θ ≈ -5.0e-4`, outside `[0, π/2]`.
**Expected:** `bds_compute_ve` returns `-3` (no interior foot) — no pave, no interference. VV also
does nothing: distance to the arc's start vertex `(2,0,0)` is `1e-3 > 1e-7+1e-7+1e-7`.
**Invariants:** (i) the edge's pave list is unchanged; (ii) no interference record with `kind==VE`
mentions this pair. **This test fails immediately if `proj_pc` clamps** — the clamped foot would
be `t = 0`, `dist = 1e-3`, and the fractional guard is the only thing between that and a
zero-length pave block. It is the direct regression test for D2.1.

### T5 — VE tolerance arithmetic and the growth rule

Same arc. `V` at `(2·cos(π/4) + 1e-5·cos(π/4), 2·sin(π/4) + 1e-5·sin(π/4), 0)` — radially offset by
`1e-5`. `tol(V)=tol(E)=1e-7`.
**Run A, `fuzz = 1e-7`:** `aTolSum = 3e-7 < 1e-5` ⇒ return `-4` ⇒ **no pave**.
**Run B, `fuzz = 2e-5`:** accepted; `tol_new = 1e-5 + 1e-7`; the vertex tolerance becomes
`max(1e-7, 1.001e-5)`.
**Invariants:** (i) Run A leaves the arena untouched; (ii) after Run B, `dist(P(V), C(t)) <= tol(V)`
(G7) and `t == π/4` to 1e-12; (iii) `tol(V)` is monotone across the two runs when B follows A.

### T6 — EE transversal crossing of two curved edges (curved focus)

Circle A: unit circle in the XY plane, full range `[0, 2π)` split at `0` (one closed edge).
Circle B: unit circle in the XZ plane, same centre. They meet **transversally** at `(1,0,0)` and
`(-1,0,0)` (tangents `(0,1,0)` and `(0,0,1)` there — perpendicular).
**Expected:** exactly **two** `TopAbs_VERTEX` common parts, at `θ_A ∈ {0, π}` and `θ_B ∈ {0, π}`.
**Invariants:** (i) `n_parts == 2`; (ii) for each part, `|C_A(ta) - C_B(tb)| <= tol_A + tol_B + fuzz`;
(iii) each `ta` is within `1e-9` of `0` or `π` (analytic); (iv) after the stage, both edges carry
two extra paves each, and the two created vertices are within `1e-9` of `(±1,0,0)`; (v) the two
vertices are distinct (they are 2 apart, far beyond any tolerance).
This is the test the current kernel cannot pass at all: `Closest::curve_curve`
(`closest.cpp:108-168`) returns one extremum (D3.1).

### T7 — EE partial overlap becomes a common block ONLY through VE (the order test)

Two edges on the **same** unit circle in the XY plane: `E1` over `[0, π/2]`, `E2` over
`[π/4, 3π/4]`, each with its own two vertices. `tol(E)=1e-7`, `tol(V)=1e-7`.

**Stage-by-stage expectation:**
- VV: `E1`'s vertices are at `θ=0, π/2`; `E2`'s at `θ=π/4, 3π/4`. `|P(E1.end) - P(E2.start)|
  = 2 sin(π/8) ≈ 0.765` — nothing fuses.
- VE: `E1`'s end vertex (`θ=π/2`) lies exactly on `E2` ⇒ pave on `E2` at `π/2` **referring to
  E1's end vertex index**. `E2`'s start vertex (`θ=π/4`) lies exactly on `E1` ⇒ pave on `E1` at
  `π/4` referring to `E2`'s start vertex index.
  Resulting blocks: `E1 → [0,π/4], [π/4,π/2]`; `E2 → [π/4,π/2], [π/2,3π/4]`.
- EE: the middle blocks have identical bounding vertex indices ⇒ `has_same_bounds` true; the
  engine returns one `TopAbs_EDGE` part covering the whole of both ⇒ **one common block**
  containing exactly those two blocks, tolerance `= max(tol(E1), tol(E2)) = 1e-7` (samples are
  exactly coincident, so `ComputeToleranceOfCB` adds 0).

**Invariants:** (i) total distinct arena vertices `== 4`; (ii) total pave blocks `== 4`;
(iii) exactly **one** common block, with exactly two members; (iv) for every pair of pave blocks
whose sampled max mutual distance is `<= tol1+tol2+fuzz`, both are in the same common block —
computable directly from the arena; (v) **with VE disabled the common block count is 0** — the
EE-only run must produce a `TopAbs_VERTEX` part at the overlap midpoint instead (§2.6.7). Assert
both branches; this pins G11.

### T8 — EE line/circle analytic tolerance floor

Circle radius `1` in the XY plane; line along `y = 0.5`, `z = 0`, parameterised by arc length,
range `[-2, 2]`. Analytic crossings at `x = ±√0.75 ≈ ±0.8660254`.
**Expected:** two `TopAbs_VERTEX` parts; `bAnalytical` is true; each new vertex's tolerance is at
least half the common part's range **on the line** (`PF_3.cxx:459-461`).
**Invariants:** (i) `|x_found ∓ 0.8660254037844386| < 1e-9`; (ii) for each new vertex `V`,
`|P(V) - C_line(t_line)| <= tol(V)` and `|P(V) - C_circle(t_circ)| <= tol(V)`;
(iii) the tolerance floor is taken from the **line**'s range — verify by swapping which edge is
passed first and asserting the tolerance is unchanged (catches the units bug of §2.6.9).

### T9 — EE near-tangent line/circle (degeneracy typing, not luck)

Circle radius `1`; line `y = 1 - 1e-9`, range `[-0.1, 0.1]`. Analytic crossings at
`x = ±√(2·1e-9 - 1e-18) ≈ ±4.4721e-5`, i.e. `8.944e-5` apart.
**Invariants:** (i) the number of parts is 1 or 2, never 0 and never > 2; (ii) the union of the
parts' ranges on the line, expanded by the reported vertex tolerances, **contains both analytic
crossings**; (iii) if one part is returned, its vertex tolerance is `>= 4.4721e-5` (it must cover
both); (iv) the outcome is *typed*: the result carries a "tangential" flag derived from
`bTouchConfirm` (`%IT%/IntTools_EdgeEdge.cxx:889`), not inferred downstream.

### T10 — Micro edge: no vertex, and endpoint welding

Edge `E1`: straight segment from `(0,0,0)` to `(1.5e-7, 0, 0)`, `tol(E)=1e-7`, vertex tolerances
`1e-7`. Its shrunk range is empty (`FindValidRange` fails: the spheres overlap), so
`has_shrunk == false` and `is_splittable == false`.
Edge `E2`: a segment crossing it transversally at its midpoint.
**Invariants:** (i) VE never proposes a pave on `E1` (`PF_2.cxx:192-197`);
(ii) EE's `TopAbs_VERTEX` branch creates **no** vertex because `is_splittable` is false
(`PF_3.cxx:370-373`); (iii) if the pair is instead routed through `SplitPaveBlocks`, `E1`'s two
vertices are fused into one (`PF_2.cxx:491-505`) and `E1`'s pave-block list becomes empty
(G6 + `%DS%/BOPDS_PaveBlock.cxx:262-268`).

### T11 — Shared-corner fast-out

Two segments sharing one endpoint vertex, meeting at 60°.
**Invariant:** no new vertex is created at the shared corner
(`%IT%/IntTools_EdgeEdge.cxx:1004-1016` for line/line;
`PF_2.cxx:166-169` `HasSubShape` for VE). Assert vertex count unchanged and no `VE`/`EE`
interference recorded between the shared vertex and either edge.

### T12 — UV-domain invariance (structural regression for our defect)

Take a box, then re-express one planar face's surface on a padded UV domain
(`u ∈ [-0.04, 4.04]` instead of `[0,1]`, matching the measured STEP round-trip). Run stages 1–3 on
box × box.
**Invariant:** the arena is **identical** to the unpadded run: same vertex count, positions equal
to 1e-15, tolerances bitwise equal, same pave count, and every pave's 3D point
`C_E(t)` equal to 1e-15. This must hold *by construction* because no VV/VE/EE predicate reads a
UV domain — the test's job is to catch a port that reintroduces one.

### T13 — Rigid-motion equivariance

Any of T3, T6, T7, T8 under 20 random rigid motions (random axis, random angle, random
translation up to 100× the model size).
**Invariants:** (i) vertex positions transform by the same motion to within 1e-12 relative;
(ii) vertex and common-block **tolerances are bitwise identical** (a rigid motion is an isometry
and all tolerances are distances); (iii) **pave parameters are bitwise identical** (parameters are
intrinsic to the curve); (iv) the combinatorics — vertex count, pave count per edge, common-block
membership — are identical. This directly attacks "analytic recognisers only fire in special
relative poses".

### T14 — Global arena invariants, asserted after every stage (run on the whole corpus)

Cheap, oracle-free, and they are the real gate:

- **A1** (G1): no two arena vertices with `|Pa-Pb| < tol_a + tol_b + fuzz`.
- **A2** (G6): for every edge, paves sort strictly increasing after dedup by vertex index, and the
  pave blocks partition `[t0,t1]` with no gaps or overlaps.
- **A3** (G7): every pave satisfies `dist(P(V), C_E(t)) <= tol(V)`.
- **A4** (G8): every extra pave's parameter is strictly inside the block it was attached to.
- **A5** (G10): all pave blocks of a common block have identical unordered bounding vertex index
  pairs, and the block's tolerance is `>=` the sampled max mutual distance over 11 interior points.
- **A6** (G4): running the three stages again changes nothing (compare a canonical serialisation).
- **A7** (G13): every `bds_edge_edge` call returns in bounded time — assert `max_depth` was never
  hit, and if it was, that the pair is reported as failed rather than silently truncated.

---

## 6. IMPLEMENTATION ORDER — smallest shippable increment first

Each increment is independently revertable and independently measured. The gate for every
increment includes T14/A1–A7 on whatever the arena contains at that point, plus the guards battery
already required by `kb/ARCHITECTURE_v2.md:161-165`.

**I0 — Arena + candidate iterator.** `src/brep_bds.h/.cpp`: `BdsShape`, `BdsVertex`,
`BdsEdgeGeom`, `BdsDS` with `shapes_sd`/`interf_table`, box construction with the two gap policies
(§2.2.3), and a BVH pair selector reproducing `%DS%/BOPDS_Iterator.cxx:270-357` including the
same-argument-range exclusion, the sub-shape exclusion, and the **stable sort** at `Initialize`.
*Gate:* on 10 corpus cells, the pair set equals a brute-force `O(n²)` box-overlap enumeration;
the sorted pair sequence is byte-identical across 20 runs and across `-O0`/`-O2`.

**I1 — VV.** `bds_compute_vv`, symmetric `FillMap` + BFS `MakeBlocks`, `bds_bounding_vertex`
(both branches, with the lexicographic sort), `bds_make_sd_vertices` with in-place arena mutation,
`bds_update_vertex`.
*Gate:* T1a/T1b/T1c, T2, T13(i–ii). A1 holds on every corpus cell.

**I2 — Curve tooling.** `src/brep_curvetool.h/.cpp`: `curve_resolution` (all seven type rows of
§2.6.2), `curve_box` (control-polygon enclosure), `proj_pc` with **OCCT semantics**,
`curve_deflection`, `curve_length`, `find_valid_range`.
*Gate:* unit tests — `curve_resolution(circle R, d) == 2*asin(d/(2R))` to 1 ulp and saturates at
`2π`; `curve_box` contains 10⁵ evaluated samples for 50 random NURBS; `proj_pc` returns 0 for T4's
configuration and returns *both* feet for a point equidistant from two arms of a parabola;
`find_valid_range` reproduces the analytic answer for a segment (`[t0 + tolV0, t1 - tolV1]`).

**I3 — VE.** `BdsPaveBlock` (+`update`, +fence), `init_pave_blocks` with SD-mapping at creation
and the seam rule (`aVertexIndices.Length() == 1`, `%DS%/BOPDS_DS.cxx:479-483`),
`bds_fill_shrunk_data`, `bds_compute_ve`, `bds_perform_ve` / `bds_intersect_ve`,
`bds_split_pave_blocks` including the weld branch, `bds_force_interf_ve`.
*Gate:* T3, T4, T5, T10, T11(VE half), T12. A2/A3/A4 hold corpus-wide.
**Landmark:** at this point the "one operand alone on a padded domain yields 32 naked edges of 36"
measurement must be re-run: the arena's vertex/pave sets must be identical padded vs unpadded
(T12).

**I4 — EE, line/line only.** `bds_edge_edge` implementing only `ComputeLineLine` (§2.6.8) plus
`CheckData`; `bds_perform_ee`'s result loop with the `TopAbs_VERTEX` branch, `MakeNewVertex`,
`bds_intersect_vertices`, and the re-entry into `bds_intersect_ve`.
*Gate:* T8's line/line analogue (two coplanar segments), T11, and **box × box unchanged**
(15/20 must not regress; the target is 20/20 once the rest lands).

**I5 — EE general.** `Prepare` (type ladder + deflection swap + resolutions + `myPTol`),
`FindSolutions` (both levels, with `max_depth`), `FindParameters`, `CheckCoincidence`,
`FindDistPC`/`DistPC`, `IsIntersection`, `MergeSolutions`, `FindBestSolution`, `IsCoincident`.
*Gate:* T6, T9, T13 on T6. A7 corpus-wide. Measure: sphere × sphere and box × sphere must now
produce a non-empty, index-shared vertex set at every crossing — that is the first curved
milestone.

**I6 — EE common blocks.** The `TopAbs_EDGE` branch with the `has_same_bounds` gate,
`bds_perform_common_blocks` (BFS + CB donation + face merge), `bds_compute_tolerance_of_cb`
(reuse `cb_tolerance` from `brep_commonblock.cpp:150`), `bds_update_vertices_of_cb`, and the
closed-representative regroup in `bds_split_pave_blocks` (`PF_2.cxx:572-616`).
*Gate:* T7 (both branches — with and without VE), A5 corpus-wide, A-op-A stays green.

**I7 — Re-entrancy: `RepeatIntersection` + `ForceInterfEE`.** The `increased_ss` scan, the extended
candidate selection, the re-run of VV+VE, then `ForceInterfEE` with its
`2·max(tol(V1),tol(V2))` extra fuzz and the `|cos| < 0.9063` (25°) tangent gate
(`PF_3.cxx:1116-1118`, `:1178-1205`).
*Gate:* A6 (idempotence) corpus-wide; the coincident-arc corpus; no regression on any prior gate.

**Deferred, out of this spec:** VF/EF (stages 4–5), FF, split-image materialisation. Note that
`bds_perform_ee`'s `aMEdges` bookkeeping (`PF_3.cxx:571-584`) and `PerformNewVertices` are already
shaped to accept EF's vertices (`theIsEEIntersection` flag, `PF_3.cxx:594-600`), so I4–I6 should
keep those parameters even though only the EE path is exercised.
