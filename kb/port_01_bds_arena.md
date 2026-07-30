# port_01 — BOPDS: the shared arena, paves, pave blocks, common blocks, face info, iterator

**Scope.** The shared data structure every other boolean subsystem hangs on. This is a build
document: a competent C++ engineer must be able to implement `src/brep_bds.h/.cpp` from this file
alone, without opening OCCT.

**Sources read in full for this document** (every claim below carries a `file:line`):

- `%DS%` = `/home/petras/code/code_cpp/OCCT/src/ModelingAlgorithms/TKBO/BOPDS`
  — `BOPDS_DS.{hxx,cxx,lxx}`, `BOPDS_Pave.{hxx,lxx}`, `BOPDS_PaveBlock.{hxx,cxx}`,
  `BOPDS_CommonBlock.{hxx,cxx}`, `BOPDS_FaceInfo.{hxx,lxx}`, `BOPDS_ShapeInfo.{hxx,lxx}`,
  `BOPDS_IndexRange.{hxx,lxx}`, `BOPDS_Interf.hxx`, `BOPDS_Iterator.{hxx,cxx}`,
  `BOPDS_SubIterator.cxx`, `BOPDS_IteratorSI.cxx`, `BOPDS_Tools.{hxx,lxx}`, `BOPDS_Pair.hxx`,
  `BOPDS_Curve.hxx`, `BOPDS_Point.hxx`, `BOPDS_CoupleOfPaveBlocks.hxx`
- `%BA%` = `/home/petras/code/code_cpp/OCCT/src/ModelingAlgorithms/TKBO/BOPAlgo`
  — `BOPAlgo_PaveFiller.cxx`, `BOPAlgo_PaveFiller_{4,5,6,7}.cxx`, `BOPAlgo_Tools.cxx`,
  `BOPAlgo_Builder_{1,2}.cxx`
- `%BT%` = `/home/petras/code/code_cpp/OCCT/src/ModelingAlgorithms/TKBO/BOPTools`
  — `BOPTools_BoxTree.hxx`, `BOPTools_BoxSelector.hxx`, `BOPTools_PairSelector.hxx`
- `Bnd_Tools.hxx` (`FoundationClasses/TKMath/Bnd`), `Precision.hxx`
  (`FoundationClasses/TKernel/Precision`), `TopTools_ShapeMapHasher.hxx`, `TopoDS_Shape.hxx`
  (`ModelingData/TKBRep`), `BRepBndLib.cxx` (`ModelingAlgorithms/TKTopAlgo/BRepBndLib`)
- `%US%` = `/home/petras/code/code_rust/session/session_cpp/src` (read-only)

Where OCCT gives up, falls back, or is internally inconsistent, this document says so explicitly
and marks it **[OCCT-GAP]** or **[OCCT-QUIRK]**. Nothing is invented.

This document is consistent with `kb/audit_occt_pavefiller-core.md` (independently re-verified
here: E1 iterator swap, E3 `SetRealPaveBlock`, O1 `BOPAlgo_Tools` centralization all confirmed).
It **corrects `kb/ARCHITECTURE_v2.md` §1** on one point: `BOPDS_FaceInfo` has **no OUT set** —
it has In / On / **Sc** (see §2.7.1).

---

# 1. WHAT THIS SUBSYSTEM MUST GUARANTEE

Each guarantee is stated so it can be asserted in code and tested without an oracle. `bds` is the
arena instance. All indices are `int`, 0-based, and `-1` means "none".

### G1 — Single arena, single index space
Every vertex, edge, face, shell and solid of **both** operands, plus every entity created during
intersection, has exactly one integer index in one flat array. There is no per-operand index space.
Nothing downstream ever identifies an entity by anything other than that integer.

> **Test:** `bds.shape(i).index == i` for all `i`; `bds.index_of(shape_key) == i` round-trips.

### G2 — Identity is structural, never geometric
Two faces reference the same edge **iff** they hold the same edge index. The arena never compares
coordinates to decide identity. There is no coordinate hash, no quantizer, no weld radius, and no
Hausdorff match anywhere in the identity path.

> **Test (the one that fails today):** take one operand; multiply its every surface's UV domain by
> 4 (a pure reparameterization, no geometric change); flatten into the arena. The edge count, the
> per-edge trim count, and the vertex→edge incidence lists must be **bit-identical** to the
> unpadded run. §5.T1.

### G3 — Ranges partition, and rank is total
`NbRanges()` equals the number of distinct argument shapes. The ranges are contiguous, ascending,
non-overlapping, and cover `[0, NbSourceShapes())`. `rank(i)` returns the argument ordinal for
every source index and `-1` for every index created after `Init`.

> **Test:** `sum(range[k].last - range[k].first + 1) == NbSourceShapes()`;
> `range[k].last + 1 == range[k+1].first`; `rank(i) >= 0 ⟺ i < NbSourceShapes()`.

### G4 — Original edges are immutable; splitting is a view
An edge's 3D curve, its parameter range and its `BRep` record are never modified by the
intersection stages. All subdivision is expressed as a list of `PaveBlock`s (parameter intervals)
attached to the edge. Materialising split edges is a **terminal** operation.

> **Test:** hash every source edge's curve control net + knot vector before `Init` and after the
> last interference stage; the hashes must be equal.

### G5 — Paves on an edge are sorted, unique-by-vertex, and tile the edge
For an edge with `n` pave blocks, the pave set has exactly `n+1` members, strictly increasing in
parameter, the first at the edge's `t_min` and the last at `t_max`; block `k` runs `[p_k, p_{k+1}]`.
Two paves on one edge never share a vertex index (exception: a closed/seam edge, whose first and
last pave legitimately carry the same vertex — §2.5.4).

> **Test:** for every edge, `paves(e).size() == pave_blocks(e).size() + 1`; parameters strictly
> increasing; `paves(e).front().t == range(e).first` and `.back().t == range(e).second`.

### G6 — Coincidence is one object, not two copies
When two pave blocks (from different operands, or from the same operand) are geometrically
coincident, they are members of one `CommonBlock`. The `CommonBlock` has exactly one representative
pave block and, once materialised, exactly one edge index shared by all members. There is never a
second edge for the same geometry that some later pass must reconcile.

> **Test:** after materialisation, `for each CB: all members have the same .edge index != -1`; and
> the count of distinct edge indices referenced by all trims equals the count of live edges.

### G7 — Common-block tolerance is measured, not assumed
`CommonBlock::tol` is the maximum over: the representative edge's own tolerance; for every other
member edge `E`, `tol(E) + max_k dist(C_rep(t_k), E)`; for every face `F` in the block's face list,
`tol(F) + max_k dist(C_rep(t_k), F)`; sampled at 11 strictly interior parameters of the
representative's pave-block range.

> **Test:** T5 — build a cylinder and a coaxial cylinder offset by `d` in radius, force them into a
> common block; `CB.tol` must be within 1e-12 of `tol_edge + d`.

### G8 — FaceInfo is derived, never authored
`FaceInfo(f).on` is exactly the set of real pave blocks of the face's own boundary edges — it is
recomputed from the topology, not accumulated. `FaceInfo(f).in` is exactly the set produced by
VF and EF interferences plus the face's internal vertices. Recomputing either from scratch must be
idempotent.

> **Test:** `update_face_info_on(f); S1 = on(f); update_face_info_on(f); S2 = on(f); assert S1==S2.`

### G9 — Candidate generation is complete for the box test and prunes nothing else
The iterator emits every cross-operand pair `(i,j)` of BRep-carrying entities whose tolerance-
inflated boxes overlap, minus (a) same-range pairs, (b) pairs where one is a sub-shape of the
other, (c) optionally pairs whose OBBs are disjoint. It emits each unordered pair once. The order
is deterministic (sorted).

> **Test:** brute-force `O(n²)` box overlap against the BVH result on 500 random boxes — sets must
> be equal. Re-run the same input 100× — the emitted sequence must be byte-identical.

### G10 — Every tolerance is a model-space length
No constant in this subsystem is expressed in parameter space. Box gaps, coincidence bands,
common-block tolerances and vertex fusion radii are 3D distances. Conversion to UV, where needed
downstream, happens at the point of use through the surface metric.

> **Test:** T1 (§5) is the enforcement test; additionally, `grep` the implementation for
> `domain(` used in the same expression as a tolerance must return nothing.

### G11 — Same-domain redirection is a terminating chain
`same_domain[i] = j` redirects index `i` to `j`. `resolve_sd(i)` walks the chain to a fixed point.
The chain must be acyclic and short.

> **Test:** `resolve_sd` must be implemented with a visited-set or hop cap and must assert on
> cycles. **[OCCT-GAP]** OCCT does not: `%DS%/BOPDS_DS.cxx:1229-1240` (`HasShapeSD`) and
> `:1244-1253` (`GetSameDomainIndex`) are unbounded `while`/`for` loops over `myShapesSD.Seek`,
> guarded only by `AddShapeSD` refusing `i == j` (`:1219-1225`). A 2-cycle hangs. Our port must
> cap.

### G12 — Mutation is in place, through explicit `change_*` accessors
The arena is a long-lived mutable object shared by all stages. Read accessors return `const&`;
mutating accessors are separately named and may **construct on demand** (lazy pave-block and
face-info initialization). No stage ever copies the arena.

> **Test:** the arena type is non-copyable (`= delete` on copy ctor/assign) — a compile-time test.

---

# 2. OCCT'S ALGORITHM, IN FULL

## 2.0 Index model, flags, and the two type predicates

**All DS indices are 0-based.** `myLines` is a `NCollection_DynamicArray<BOPDS_ShapeInfo>`
constructed with lower bound 0 (`%DS%/BOPDS_DS.cxx:102`). `NbShapes() == myLines.Length()`
(`:186-189`). `Rank()` scans ranges from 0 and returns `-1` on miss (`:214-224`).
`IsNewShape(i) == (i >= NbSourceShapes())` (`:228-231`).

Two type predicates decide who participates:

```
BOPDS_Tools::HasBRep(t)      == (t==VERTEX || t==EDGE || t==FACE)   %DS%/BOPDS_Tools.lxx:17-20
BOPDS_Tools::IsInterfering(t)== HasBRep(t) || t==SOLID              %DS%/BOPDS_Tools.lxx:24-27
```

`ShapeInfo::HasBRep()`/`IsInterfering()` forward to these (`%DS%/BOPDS_ShapeInfo.lxx:135-145`).
**Only `HasBRep` shapes enter the boolean iterator's BVH** (`%DS%/BOPDS_Iterator.cxx:281-284`);
`IsInterfering` (which admits SOLID) is used only by the self-intersection checker
(`%DS%/BOPDS_IteratorSI.cxx:50-53`) and by `IntersectExt` (`%DS%/BOPDS_Iterator.cxx:379-382`, where
SOLID is then explicitly excluded again).

**`TypeToInteger` — the interference-type dispatch table** (`%DS%/BOPDS_Tools.lxx:86-123`,
`:31-82`). Single-type codes: COMPOUND 0, COMPSOLID 1, SOLID 2, SHELL 3, FACE 4, WIRE 5, EDGE 6,
VERTEX 7, SHAPE 8, default 9. Pair code `iX = iT2*10 + iT1`, mapped:

| `iX` | result | meaning |
|---|---|---|
| 77 | 0 | VV |
| 76, 67 | 1 | VE |
| 66 | 2 | EE |
| 74, 47 | 3 | VF |
| 64, 46 | 4 | EF |
| 44 | 5 | FF |
| 72, 27 | 6 | VZ |
| 62, 26 | 7 | EZ |
| 42, 24 | 8 | FZ |
| 22 | 9 | ZZ |
| any other | **-1** | not an interference |

`NbInterfTypes() == 10` (`%DS%/BOPDS_DS.lxx:87-90`).

**[OCCT-GAP] Types 6–9 (VZ, EZ, FZ, ZZ) are never produced by the boolean pipeline.** They are
written only by `%BA%/BOPAlgo_CheckerSI.cxx:359,375,387,399` and
`%BA%/BOPAlgo_CheckerSI_1.cxx:252,379,445,446` — the self-intersection checker. Solids are excluded
from the boolean iterator's BVH by the `HasBRep` filter. Our port needs storage for them only if we
port the checker; the boolean itself needs VV, VE, EE, VF, EF, FF.

**Flags.** `ShapeInfo::myFlag` defaults to `-1` and `HasFlag() == (myFlag >= 0)`
(`%DS%/BOPDS_ShapeInfo.lxx:19-25`, `:149-152`). **Flag value 0 is a legal flag.** Two different
meanings share the field:

- On an **edge**: "degenerate". `prepareEdges` sets `SetFlag(edgeIndex)` for a degenerate edge
  (`%DS%/BOPDS_DS.cxx:1674`), then `prepareFaces` **overwrites** it with the owning face index
  (`:1745`). Edges are prepared before faces (`:314-315`), so the surviving value is the face index.
- On a **vertex**: "synthetic, created at an infinite curve parameter" (`:1658`, `SetFlag(1)`).
  `InitPaveBlocks` branches on this to use the unsafe `ComputeParameter` helper instead of
  `BRep_Tool::Parameter` (`:462-463`).

`Reference` defaults to `-1`, `HasReference() == (myReference >= 0)`
(`%DS%/BOPDS_ShapeInfo.lxx:114-117`). It is **type-punned**: on an edge it indexes
`myPaveBlocksPool`, on a face it indexes `myFaceInfoPool`. `HasPaveBlocks(i)` and `HasFaceInfo(i)`
are literally the same predicate on the same field (`%DS%/BOPDS_DS.cxx:405-408` and `:706-709`), so
a shape that is neither edge nor face must never be asked.

## 2.1 Fields of `BOPDS_DS` (`%DS%/BOPDS_DS.hxx:479-502`)

```
myAllocator      handle<NCollection_BaseAllocator>
myArguments      List<TopoDS_Shape>                         the operands, in order
myNbShapes       int                                        == myLines.Length()
myNbSourceShapes int                                        frozen at end of Init()
myRanges         DynamicArray<BOPDS_IndexRange>             one per distinct argument
myLines          DynamicArray<BOPDS_ShapeInfo>              THE ARENA
myMapShapeIndex  DataMap<TopoDS_Shape,int,ShapeMapHasher>   shape -> arena index
myPaveBlocksPool DynamicArray<List<handle<PaveBlock>>>      one list per touched edge
myMapPBCB        DataMap<handle<PaveBlock>,handle<CommonBlock>>
myFaceInfoPool   DynamicArray<BOPDS_FaceInfo>               one per touched face
myShapesSD       DataMap<int,int>                           same-domain redirection
myMapVE          DataMap<int,List<int>>                     vertex index -> edge indices
myInterfTB       Map<BOPDS_Pair>                            "these two interfered" set
myInterfVV..ZZ   DynamicArray<BOPDS_InterfXX>               10 typed interference logs
myInterfered     Map<int>                                   union of all interfered indices
```

`Clear()` resets all of the above and both counters (`%DS%/BOPDS_DS.cxx:135-161`).

## 2.2 `BOPDS_ShapeInfo` — one arena slot (`%DS%/BOPDS_ShapeInfo.hxx:124-131`)

```
TopoDS_Shape     myShape
TopAbs_ShapeEnum myType       default TopAbs_SHAPE
Bnd_Box          myBox
List<int>        mySubShapes  indices, meaning depends on type (see 2.4)
int              myReference  -1
int              myFlag       -1
```

`HasSubShape(i)` is a linear `mySubShapes.Contains(i)` (`%DS%/BOPDS_ShapeInfo.lxx:107-110`) —
O(deg), used on the hot path of the iterator's self-pair rejection.

## 2.3 `BOPDS_IndexRange` — operand separation (`%DS%/BOPDS_IndexRange.hxx:69-72`)

Two ints, `myFirst`/`myLast`, defaulting to `(0,0)` (`%DS%/BOPDS_IndexRange.lxx:17-21`).
`Contains(i) == (i >= myFirst && i <= myLast)` — **inclusive on both ends**
(`%DS%/BOPDS_IndexRange.lxx:77-80`).

Ranges are built in `Init` (`%DS%/BOPDS_DS.cxx:295-307`): `i1 = 0`; for each argument that is not
already bound in `myMapShapeIndex`, append it and recursively append its whole sub-tree, then
record `[i1, NbShapes()-1]` and set `i1 = last + 1`. **Duplicate arguments are skipped entirely and
get no range** (`:298-301`) — so `NbRanges()` can be smaller than `myArguments.Length()`.

Because argument sub-trees are appended contiguously and depth-first, and shared sub-shapes bind
once, **`Rank(i)` is exactly "which operand owns index `i`"** for source shapes. Anything appended
after `Init` (new vertices, split edges, section edges) is outside every range, so `Rank` returns
`-1` — which is also the definition of `IsNewShape`.

## 2.4 `Init` — flattening both operands into the arena (`%DS%/BOPDS_DS.cxx:285-324`)

```
Init(fuzz = Precision::Confusion()):
  if myArguments empty: return
  myRanges.SetIncrement(myArguments.Length())
  myLines.SetIncrement(500)                            // THE_INITIAL_LINES_INCREMENT, :57
  i1 = 0
  for each argument S:
      if myMapShapeIndex.IsBound(S): continue          // duplicate argument, no range
      InitShape(Append(S), S)                          // recursive flatten
      myRanges.Append(IndexRange(i1, NbShapes()-1))
      i1 = NbShapes()
  myNbSourceShapes = NbShapes()

  addTol = max(fuzz, Precision::Confusion()) * 0.5     // :312   Confusion() == 1e-7
  prepareVertices(addTol)
  nE = prepareEdges(addTol)
  nF = prepareFaces(addTol)
  prepareSolids()
  buildVertexEdgeMap()
  myPaveBlocksPool.SetIncrement(nE)
  myFaceInfoPool.SetIncrement(nF)
```

### 2.4.1 `InitShape` — the identity mechanism (`%DS%/BOPDS_DS.cxx:328-352`)

```
InitShape(idx, S):
  info = ChangeShapeInfo(idx); info.SetShapeType(S.ShapeType())
  subs = info.ChangeSubShapes()
  seen = set(subs)                                    // idempotent re-entry
  for each direct child C of S (TopoDS_Iterator):
      existing = myMapShapeIndex.Seek(C)
      j = existing ? *existing : Append(C)            // <<-- IDENTITY HAPPENS HERE
      InitShape(j, C)
      if seen.Add(j): subs.Append(j)
```

`Append` binds `myMapShapeIndex[shape] = index` (`:235-251`). The map's equality is
`TopTools_ShapeMapHasher::operator()(S1,S2) → S1.IsSame(S2)`
(`TopTools_ShapeMapHasher.hxx:35-38`), and `TopoDS_Shape::IsSame` is
`myTShape == other.myTShape && myLocation == other.myLocation`
(`TopoDS_Shape.hxx:268-271`) — **pointer identity of the underlying `TShape` plus location**,
orientation ignored. The hash is the TShape pointer combined with the location hash
(`TopoDS_Shape.hxx:332-341`).

**This is the whole answer to "how do two faces come to reference the same edge".** In a valid
`TopoDS_Shape`, two faces that share an edge literally hold handles to the same `TopoDS_TEdge`
object. `InitShape` walks face → wire → edge → vertex; the second face's walk reaches the *same
pointer*, finds it already bound, and reuses its arena index. **No coordinates are read, no
tolerance is consulted, no weld is performed.** Edge identity is inherited from the input topology
and is exact by construction.

The corollary that matters to us: a coordinate-coincidence weld is never needed **because the
splitter never mints two copies in the first place**. The weld in our kernel is not a bad
implementation of identity — it is a symptom of having lost identity earlier, in the splitter.

### 2.4.2 `prepareVertices` (`%DS%/BOPDS_DS.cxx:1589-1610`)

For each source shape of type VERTEX: `box.SetGap(BRep_Tool::Tolerance(V) + addTol)` then
`box.Add(BRep_Tool::Pnt(V))`. Half-extent in each axis is exactly `tol(V) + addTol`. Returns the
count (unused by `Init`).

### 2.4.3 `prepareEdges` (`%DS%/BOPDS_DS.cxx:1614-1692`)

For each source EDGE:
1. If **not** degenerate: fetch the FORWARD-oriented edge's `Geom_Curve` and its `[f,l]`. If
   `Precision::IsNegativeInfinite(f)`, evaluate `C(f)`, `BRep_Builder::MakeVertex(V, P, tol(E))`,
   orient FORWARD, create a `ShapeInfo` with `SetFlag(1)` (the "infinite/synthetic" flag), append it
   and push its index onto the edge's sub-shape list. Same for `IsPositiveInfinite(l)` (`:1646-1670`).
   These synthetic vertices are **not** topologically connected to the edge, which is why
   `InitPaveBlocks` must not call `BRep_Tool::Parameter` on them.
2. Else (degenerate): `SetFlag(edgeIndex)` (`:1674`) — later overwritten by `prepareFaces`.
3. `BRepBndLib::Add(E, box)` — this already enlarges by `BRep_Tool::Tolerance(E)`
   (`BRepBndLib.cxx:149`, `:170`, `:194`, `:201`).
4. Add every sub-vertex's box (`:1682-1686`).
5. `box.SetGap(box.GetGap() + addTol)` — **additive on the existing gap, not assignment**
   (`:1688`).

### 2.4.4 `prepareFaces` (`%DS%/BOPDS_DS.cxx:1696-1779`)

For each source FACE: `BRepBndLib::Add(F, box)` (already includes `tol(F)`, `BRepBndLib.cxx:99`,
`:109`, `:131`). Then **the sub-shape list is rewritten**: on entry it holds WIRE indices; the code
walks wire → edge, unions each edge box into the face box, adds the edge index to a set, marks
degenerate edges with `SetFlag(faceIndex)`, adds every vertex of the edge to the set; then adds any
direct VERTEX children of the face (internal vertices); then clears the wire list and refills it
from the set. **After `Init`, a face's `SubShapes()` is {edges} ∪ {vertices}, wires gone**
(`:1710-1773`). Finally `box.SetGap(box.GetGap() + addTol)` (`:1775`).

Note the set is a `NCollection_Map<int>` so **the resulting order is hash order, not topological
order** — anything that iterates a face's sub-shapes must not depend on order.

### 2.4.5 `prepareSolids` (`%DS%/BOPDS_DS.cxx:1783-1852`)

**[OCCT-QUIRK] Returns 0 immediately unless `myArguments.Length() == 1`** (`:1789-1792`). In a
two-operand boolean, solids therefore keep **void** bounding boxes and their sub-shape lists stay as
SHELL indices. This is harmless only because `HasBRep` excludes SOLID from the iterator BVH. When
there is exactly one argument (the checker path), the solid's box is built by `BuildBndBoxSolid` and
its sub-shape list is rewritten from shells to {faces} ∪ {edges}.

`BuildBndBoxSolid(i, box, checkInverted=true)` (`%DS%/BOPDS_DS.cxx:1390-1445`): union the boxes of
all faces of all shells; if any face box `IsOpen()`, or `BOPTools_AlgoTools::IsOpenShell(shell)`, the
result is `box.SetWhole()`; else if `checkInverted && BOPTools_AlgoTools::IsInvertedSolid(solid)`,
also `SetWhole()`.

### 2.4.6 `buildVertexEdgeMap` (`%DS%/BOPDS_DS.cxx:1856-1886`)

For every source EDGE and each of its sub-vertices `v`: append the edge index to `myMapVE[v]`,
with a linear uniqueness check (`std::none_of`). Used by `InitPaveBlocksForVertex(nV)`
(`:1487-1499`), which forces pave-block construction on every edge touching a given vertex.

## 2.5 Paves and pave blocks — an interval of an edge without mutating the edge

### 2.5.1 `BOPDS_Pave` (`%DS%/BOPDS_Pave.hxx:75-78`, `.lxx`)

```
int    myIndex      = -1     // arena index of the VERTEX sitting here
double myParameter  = 99.    // parameter on the edge's 3D curve
```
`operator<` compares **parameter only** (`%DS%/BOPDS_Pave.lxx:68-71`).
`operator==` compares **index AND parameter, exactly** (`:75-78`).
**[OCCT-QUIRK]** OCCT documents that these two are inconsistent
(`%DS%/BOPDS_DS.cxx:1350-1355`): `!(a<b) && !(b<a)` does *not* imply `a==b`. It therefore refuses to
use `std::set` and hand-rolls dedup-by-`==` plus sort-by-`<`. Our port must keep the same split:
**sort by parameter, dedup by (vertex, parameter)**.

Hash: `hash_combine(hash(index), hash(parameter))` (`%DS%/BOPDS_Pave.hxx:82-93`).

### 2.5.2 `BOPDS_PaveBlock` (`%DS%/BOPDS_PaveBlock.hxx:182-193`)

```
handle<Allocator> myAllocator
int        myEdge        = -1     // arena index of the MATERIALISED split edge, -1 until made
int        myOriginalEdge= -1     // arena index of the edge this block lives on
Pave       myPave1, myPave2       // the interval's ends
List<Pave> myExtPaves             // pending subdivisions, consumed by Update()
double     myTS1, myTS2  = -99.   // shrunk range (see 2.5.6)
Bnd_Box    myShrunkBox
Map<int>   myMFence               // vertex indices already present in myExtPaves
bool       myIsSplittable = false
```

Accessors of note:
- `Range(t1,t2)` = `(myPave1.Parameter(), myPave2.Parameter())` (`%DS%/BOPDS_PaveBlock.cxx:132-136`).
- `Indices(n1,n2)` = the two vertex indices (`:140-144`).
- `HasSameBounds(other)`: true if the index pairs match **in either order** (`:148-160`).
- `IsSplitEdge()` = `myEdge != myOriginalEdge` (`:97-100`). Note the reuse path
  (`%BA%/BOPAlgo_PaveFiller_7.cxx:460`) deliberately sets `myEdge = myOriginalEdge`, making this
  false for an untouched edge.
- `HasEdge()` = `myEdge >= 0` (`:68-71`).
- `IsToUpdate()` = `!myExtPaves.IsEmpty()` (`:220-223`).
- `ContainsParameter(t, tol, &vertexIdx)`: linear scan of **`myExtPaves` only** (not `myPave1/2`),
  first with `|p.Parameter() - t| < tol` wins (`:227-245`).

### 2.5.3 The two append variants (`%DS%/BOPDS_PaveBlock.cxx:167-180`)

```
AppendExtPave(p) : if myMFence.Add(p.Index()) then myExtPaves.Append(p)   // fenced by VERTEX index
AppendExtPave1(p):                                 myExtPaves.Append(p)   // unfenced
```
**[OCCT-QUIRK]** `RemoveExtPave(v)` only acts when `myMFence.Contains(v)` (`:184-202`), so paves
added through `AppendExtPave1` are **unremovable**. Our port should either fence both or make
removal independent of the fence; document whichever we choose.

### 2.5.4 `InitPaveBlocks` — a topological edge becomes paves (`%DS%/BOPDS_DS.cxx:437-501`)

Called lazily from `ChangePaveBlocks(i)` when the edge has no reference (`:425-433`).

```
InitPaveBlocks(nE):
  info = ChangeShapeInfo(nE); E = Edge(info.Shape())
  V    = info.SubShapes()
  if V.IsEmpty(): return                     // :444-447  NO pave block, NO reference set
  PB = new PaveBlock; PB->SetOriginalEdge(nE)

  if E.Orientation() != INTERNAL:
      for each vertex index nv in V:
          vi = ShapeInfo(nv); Vtx = Vertex(vi.Shape())
          t  = vi.HasFlag() ? ComputeParameter(Vtx, E)      // synthetic infinite vertex
                            : BRep_Tool::Parameter(Vtx, E)
          nv = GetSameDomainIndex(nv)                        // :465  SD-mapped AT CREATION
          if info.HasFlag():  PB->AppendExtPave1(Pave(nv,t)) // degenerate edge: unconditional
          else:               PB->AppendExtPave (Pave(nv,t)) // fenced by vertex index
          if V.Length() == 1:                                // :479-483  SEAM RULE
              Vtx.Reverse()
              PB->AppendExtPave1(Pave(nv, BRep_Tool::Parameter(Vtx, E)))
  else:                                                      // INTERNAL-oriented edge
      for each vertex child of E (TopoDS_Iterator(E,false,true)):
          nv = Index(Vtx); vi = ShapeInfo(nv)
          t  = vi.HasFlag() ? ComputeParameter(Vtx,E) : BRep_Tool::Parameter(Vtx,E)
          PB->AppendExtPave1(Pave(GetSameDomainIndex(nv), t))

  PB->Update(myPaveBlocksPool.Appended(), /*theFlag=*/false)
  info.SetReference(myPaveBlocksPool.Length() - 1)
```

Points a reimplementer must not miss:
- **The seam rule is `V.Length() == 1`, not "the curve is closed"** (`:479`). A closed edge with one
  distinct vertex gets a second, unfenced pave at the reversed vertex's parameter — which is the
  other end of the parameter range.
- **Paves are same-domain-resolved at creation** (`:465`, `:495`), so a fused vertex never shows up
  under its pre-fusion index.
- `ComputeParameter(V, E)` (`%DS%/BOPDS_DS.cxx:66-86`) is documented in-source as **"generally
  unsafe … left to preserve previous behavior"**: it tests only the curve's start and end parameter
  against `dist² < tol(E)²` and **returns `0.` when neither matches**. **[OCCT-GAP]** — a documented
  fallback that can silently produce a wrong pave parameter. Our port should return an error instead;
  we do not have infinite edges.

### 2.5.5 `Update` — turning ext-paves into blocks (`%DS%/BOPDS_PaveBlock.cxx:249-312`)

```
Update(out list LPB, theFlag = true):
  n = myExtPaves.Extent() + (theFlag ? 2 : 0)
  if n <= 1: myExtPaves.Clear(); myMFence.Clear(); return    // :263-268  produces NOTHING
  P = array[1..n]
  if theFlag: P[1]=myPave1; P[2]=myPave2; i=3 else i=1
  copy myExtPaves into P[i..]
  myExtPaves.Clear(); myMFence.Clear()
  std::sort(P)                                               // by PARAMETER only
  for k = 2..n:
      PB = new PaveBlock
      PB->SetOriginalEdge(myOriginalEdge)
      PB->SetPave1(P[k-1]); PB->SetPave2(P[k])
      LPB.Append(PB)
```

- `theFlag=false` is used exactly once, by `InitPaveBlocks` (`%DS%/BOPDS_DS.cxx:499`), where
  `myPave1/myPave2` are still the `(-1, 99.)` defaults and must not participate.
- New blocks inherit **only** `myOriginalEdge`. `myEdge` stays `-1`, shrunk data is not copied,
  `myIsSplittable` resets to `false`. That is deliberate: a freshly subdivided block is unmaterialised.
- **No dedup by parameter.** Two paves with equal parameters produce a zero-length block. The fence
  in `AppendExtPave` prevents duplicates only by *vertex index*.

`UpdatePaveBlocks()` (`%DS%/BOPDS_DS.cxx:505-526`) sweeps the whole pool, replacing any block with
pending ext-paves by its subdivision. `UpdatePaveBlock(PB)` (`:530-553`) does one block, located by
pointer identity within its original edge's list.

### 2.5.6 Shrunk data (`%DS%/BOPDS_PaveBlock.cxx:317-346`, `%DS%/BOPDS_DS.cxx:1547-1585`)

`SetShrunkData(tS1, tS2, box, isSplittable)` records the sub-range of the block that lies **outside**
both end vertices' tolerance spheres, plus that sub-range's box, plus a flag saying whether the block
is long enough to split at all. `HasShrunkData() == !myShrunkBox.IsVoid()` (`:317-320`).

`BOPDS_DS::IsValidShrunkData(PB)` re-validates it after vertex tolerances change (`:1547-1585`):

```
eps = BRep_Tool::Tolerance(originalEdge) * 0.01
for i in {0,1}:
    V   = Shape(PB.Indices()[i]);  tolV = Tolerance(V) + Precision::Confusion()
    d   = dist( Pnt(V), BRepAdaptor_Curve(E).Value(tS_i) )
    if (tolV - d > eps): return false      // shrunk end is now INSIDE the vertex sphere
return true
```

### 2.5.7 `Paves(edge)` — the sorted, deduplicated pave set (`%DS%/BOPDS_DS.cxx:1346-1386`)

```
LPB = PaveBlocks(nE); if empty: return
P   = array[1 .. LPB.Length()+1];  i = 1;  seen = Map<Pave>          // hash = (index,parameter)
for each PB in LPB: for p in {PB.Pave1(), PB.Pave2()}:
      if seen.Add(p): P[i++] = p
assert(LPB.Length()+1 == seen.Length())                              // :1378 "Abnormal number of paves"
std::sort(P)                                                          // by parameter
emit P in order
```

**[OCCT-QUIRK]** `P` is sized `LPB.Length()+1` but filled by a conditional counter. If the pave
census is wrong the assert fires only in debug; in release, fewer unique paves leave default
`(-1, 99.)` entries that sort to the end, and *more* unique paves overrun the array. Our port must
size the container dynamically and hard-assert G5.

## 2.6 `BOPDS_CommonBlock` — coincident pave blocks unified

### 2.6.1 Structure (`%DS%/BOPDS_CommonBlock.hxx:140-142`)

```
List<handle<PaveBlock>> myPaveBlocks   // members; FIRST is the representative
List<int>               myFaces        // faces the block lies ON
double                  myTolerance    // = ComputeToleranceOfCB, default 0.0
```

The class comment (`:28-34`) states the contract: a common block stores pave blocks that have
geometric coincidence (within a tolerance) with other pave block(s) **and/or** with face(s), and the
first pave block — the *real* pave block — is "always a pave block with the minimal index of the
original edge".

**[OCCT-QUIRK, confirms audit E3]** That is only conditionally true. `AddPaveBlock` prepends when
`aPB->OriginalEdge() < myPaveBlocks.First()->OriginalEdge()`, else appends
(`%DS%/BOPDS_CommonBlock.cxx:39-56`), and `SetPaveBlocks` funnels through `AddPaveBlock` (`:60-68`)
— so a single pass gives min-index-first **only** because every candidate is compared against the
current head; a sequence like (5, 3, 4) yields head 3, list [3,5,4]. And
`SetRealPaveBlock(PB)` (`:114-126`) deliberately moves an arbitrary member to the front; it is used
by the "don't split, reuse the existing edge" path at `%BA%/BOPAlgo_PaveFiller_7.cxx:450`. **The
representative is therefore not a pure function of the member set.** Our port must make the
representative an explicit, recorded choice.

Queries: `Contains(PB)` is pointer identity over members (`:221-245`); `Contains(faceIdx)` is a
linear scan of `myFaces` (`:249-264`); `IsPaveBlockOnEdge(e)` matches any member's `OriginalEdge()`
(`:173-191`); `PaveBlockOnEdge(e)` returns the member on that edge, or a **static null handle**
(`:130-148` — note the `static` local, not thread-safe for writing).

`SetEdge(nSp)` assigns the same materialised edge index to **every** member (`:195-205`); `Edge()`
reads it off the representative (`:209-217`). **This is where G6 becomes literal:** one call makes
all coincident pieces name one edge.

### 2.6.2 Who creates common blocks — all four sites funnel through `BOPAlgo_Tools`

**Overload A**, for pave-block/pave-block coincidence
(`%BA%/BOPAlgo_Tools.cxx:107-187`). Input: `IndexedDataMap<PB, List<PB>>` — an adjacency map "this
PB coincides with these PBs", accumulated by EE / ForceEE.

```
MakeBlocks(adjacency) -> list of connected components               // :123
for each component LPB with >= 2 members:                           // :134-137 (singletons skipped)
    collect faces from every member that ALREADY has a CB, dedup     // :146-169
    reuse the FIRST such CB object if any, else new CommonBlock      // :164-174
    CB->SetPaveBlocks(LPB); CB->SetFaces(collectedFaces)             // :176-177
    for each member: DS->SetCommonBlock(member, CB)                  // :178-181
    CB->SetTolerance(ComputeToleranceOfCB(CB, DS, ctx))              // :184-185
```

**Overload B**, for pave-block/face coincidence
(`%BA%/BOPAlgo_Tools.cxx:191-244`). Input: `IndexedDataMap<PB, List<int>>` — "this PB lies on these
faces", accumulated by EF (`%BA%/BOPAlgo_PaveFiller_5.cxx:564` via `FillMap`) and ForceEF.

```
for each (PB, faceList):
    CB = DS->IsCommonBlock(PB) ? DS->CommonBlock(PB) : (new CommonBlock, CB->AddPaveBlock(PB))
    append the faces of faceList not already in CB->Faces()          // :216-238
    DS->SetCommonBlock(PB, CB)
    CB->SetTolerance(ComputeToleranceOfCB(CB, DS, ctx))              // :241-242
```

`FillMap(PB, nF, map, alloc)` (`:91-103`) is the trivial multimap insert.

### 2.6.3 `ComputeToleranceOfCB` — what a common block's tolerance MEANS (`%BA%/BOPAlgo_Tools.cxx:248-356`)

```
if CB is null: return 0
PBR = CB->PaveBlock1();  Eor = Shape(PBR->OriginalEdge())
tolMax = BRep_Tool::Tolerance(Eor)
if (CB->PaveBlocks().Extent() < 2 && CB->Faces().IsEmpty()): return tolMax   // :266-269

nPnt = 11                                                      // :271
C3D  = BRep_Tool::Curve(Eor, _, _)
(t1, t2) = PBR->Range()                                        // :277  overwrites the curve range
dt   = (t2 - t1) / (nPnt + 1)                                  // :278  => /12

if members > 1:                                                // :287
  for each member PB != PBR:
      E = Shape(PB->OriginalEdge());  tol = Tolerance(E)
      proj = ctx->ProjPC(E)                                    // point-on-curve projector, cached
      t = t1
      repeat 11 times: t += dt; C3D->D0(t, P); proj.Perform(P)
          if proj.NbPoints(): tolMax = max(tolMax, tol + proj.LowerDistance())

if faces non-empty:                                            // :327
  for each nF in CB->Faces():
      F = Shape(nF);  tol = Tolerance(F)
      proj = ctx->ProjPS(F)                                    // point-on-surface projector, cached
      t = t1
      repeat 11 times: t += dt; C3D->D0(t, P); proj.Perform(P)
          if proj.NbPoints(): tolMax = max(tolMax, tol + proj.LowerDistance())
return tolMax
```

Sampling parameters are `t1 + k·dt` for `k = 1..11`, i.e. **strictly interior** — the endpoints are
never sampled (they are vertices with their own tolerances). The value is
`max(own tolerance, other_entity_tolerance + measured_deviation)`: exactly "how fat does this edge
have to be to be honestly shared by everything in this block".

The tolerance is written back onto the materialised edge:
`%BA%/BOPAlgo_PaveFiller_7.cxx:135` computes it inside the parallel split task, `:541` calls
`UpdateEdgeTolerance(nSp, aBSE.Tolerance())`, and `:453-454` does the same on the "reuse the
existing edge" path.

### 2.6.4 `UpdateCommonBlock` — splitting a common block (`%DS%/BOPDS_DS.cxx:557-654`)

Invoked when the members carry pending ext-paves. Early-out if the **representative** has none
(`:562-565`).

```
for each member PB:
    find PB in its original edge's pool list
    PB->Update(newBlocks)                             // subdivide
    for each new block B:
        append B to the pool list
        key = Pair(B.Indices())                       // (vertexIdx1, vertexIdx2)
        multimap[key].append(B)
    remove PB from the pool list

for each multimap bucket LPBx:                        // :611
    while LPBx non-empty:
        seed = pop first
        group = [seed]
        for each remaining B in LPBx:
            if CheckCoincidence(B, seed, fuzz): move B into group
        CB' = new CommonBlock
        CB'->SetPaveBlocks(group); CB'->SetFaces(CB->Faces())
        for each B in group: SetCommonBlock(B, CB')
```

Note the new sub-blocks inherit the **parent's whole face list**, and no new tolerance is computed
here.

`CheckCoincidence(PB1, PB2, fuzz)` (`%DS%/BOPDS_DS.cxx:1288-1331`) — the only geometric predicate
inside BOPDS:

```
(f1,l1) = PB1->Range();  tm = IntTools_Tools::IntermediatePoint(f1, l1)
P       = BOPTools_AlgoTools::PointOnEdge(Edge(PB1->OriginalEdge()), tm)
E2      = Edge(PB2->OriginalEdge());  C2 = BRep_Tool::Curve(E2, a, b)
proj    = GeomAPI_ProjectPointOnCurve(C2, a, b);  proj.Perform(P)
if proj.NbPoints() == 0: return false
tol = BRep_Tool::MaxTolerance(E1, VERTEX) + BRep_Tool::MaxTolerance(E2, VERTEX)
    + max(fuzz, Precision::Confusion())
return proj.LowerDistance() < tol
    && PB2.Range().first < proj.LowerDistanceParameter() < PB2.Range().second   // STRICT
```

**One interior point only.** Coincidence here is a cheap regrouping test after a known-coincident
block was subdivided, not a general coincidence detector. The tolerance is the sum of both edges'
**vertex** tolerances plus fuzz — a model-space length.

### 2.6.5 `RealPaveBlock` and friends (`%DS%/BOPDS_DS.cxx:658-687`)

```
RealPaveBlock(PB)      = CommonBlock(PB) ? CB->PaveBlock1() : PB
IsCommonBlockOnEdge(PB)= CB && CB->PaveBlocks().Length() > 1
IsCommonBlock(PB)      = myMapPBCB.IsBound(PB)
CommonBlock(PB)        = myMapPBCB.Seek(PB) ? *it : null
SetCommonBlock(PB, CB) = myMapPBCB.Bind(PB, CB)
```
`RealPaveBlock` is the canonicalisation used everywhere a pave block is put into a set —
`FaceInfoOn` (`:825`), `SharedEdges` (`:1173`, `:1200`), `FillImagesEdges`
(`%BA%/BOPAlgo_Builder_1.cxx:101`).

## 2.7 `BOPDS_FaceInfo` — the per-face intersection state

### 2.7.1 Structure (`%DS%/BOPDS_FaceInfo.hxx:129-137`)

```
handle<Allocator>            myAllocator
int                          myIndex        = -1   // the face's arena index
IndexedMap<handle<PaveBlock>> myPaveBlocksIn
Map<int>                      myVerticesIn
IndexedMap<handle<PaveBlock>> myPaveBlocksOn
Map<int>                      myVerticesOn
IndexedMap<handle<PaveBlock>> myPaveBlocksSc
Map<int>                      myVerticesSc
```

**There is no OUT set.** The three sets are:

| set | contents | who fills it |
|---|---|---|
| **On** | the real pave blocks of the face's **own boundary edges**, and every vertex those blocks touch, plus the face's direct vertex children | `FaceInfoOn`, derived from topology only |
| **In** | pave blocks lying strictly **inside** the face (from EF interferences via a common block whose face list contains this face), and vertices inside it (VF interferences, EF new vertices, internal vertices) | `FaceInfoIn` / `UpdateFaceInfoIn`, derived from the interference logs |
| **Sc** | pave blocks of **section** curves on this face, and section **points** | written directly by `PerformFF`/`UpdateFaceInfo` |

`Clear()` (`%DS%/BOPDS_FaceInfo.lxx:55-61`) clears **In and On only — not Sc**. **[OCCT-QUIRK]**
Our port should either clear all six or rename the method.

`FaceInfoPool` slots are created lazily by `ChangeFaceInfo(i)` → `InitFaceInfo(i)`
(`%DS%/BOPDS_DS.cxx:726-747`), which appends a slot, sets the shape's `Reference`, sets
`aFaceInfo.SetIndex(i)`, then runs `InitFaceInfoIn(i)` and `UpdateFaceInfoOn(i)`.

### 2.7.2 `FaceInfoOn` — On is a *view of the boundary* (`%DS%/BOPDS_DS.cxx:811-833`)

```
for each sub-shape index s of face f:
    if ShapeInfo(s).ShapeType() == EDGE:
        for each PB in PaveBlocks(s):
            (v1,v2) = PB->Indices();  MVP.Add(v1); MVP.Add(v2)
            MPB.Add(RealPaveBlock(PB))
    else:                                   // VERTEX
        MVP.Add(GetSameDomainIndex(s))
```

So "On" is not a classification result — it is literally "the face's own boundary, expressed in
pave blocks, canonicalised through common blocks". Two faces sharing an edge get the **same
`handle<PaveBlock>` objects** in their On sets, which is how `SubShapesOnIn` can find shared entities
by pointer.

`UpdateFaceInfoOn(i)` clears On and re-runs `FaceInfoOn` (`:792-807`); the set-valued overload
(`:954-971`) does the same for a batch, creating the pool slot if missing.

`RefineFaceInfoOn()` (`:975-991`) re-derives On for every pooled face, then **removes every pave
block that has no materialised edge** (`!PB->HasEdge()`) — called once, after `MakeBlocks`, at
`%BA%/BOPAlgo_PaveFiller.cxx:340`.

### 2.7.3 `FaceInfoIn` (`%DS%/BOPDS_DS.cxx:837-890`)

```
1. every direct VERTEX child of the face   -> verticesIn (SD-resolved)
2. every InterfVF containing f             -> the opposite index, SD-resolved, into verticesIn
3. every InterfEF containing f:
     if the interference produced a new vertex (GetIndexNew()):  that vertex -> verticesIn
     else: for each PB of the interfering edge:
              CB = CommonBlock(PB)
              if CB && CB->Contains(f):  paveBlocksIn.Add(CB->PaveBlock1())
```

Line 3 is the crucial one: **an edge lands in a face's IN set only through a common block whose
face list contains that face.** That is the EF stage's output (§2.6.2 overload B). Without an EF
stage there is no IN set, hence no interior boundary, hence no correct face split — which is
exactly `kb/hunt_efvf_gap.md`'s finding about our kernel.

`UpdateFaceInfoIn(i)` (`:773-788`) clears In and re-runs `FaceInfoIn`. The batch overload
(`:894-950`) is a rewritten, single-pass version: clear all requested faces, re-add internal
vertices, then one sweep over `InterfVF` and one over `InterfEF`. **[OCCT-QUIRK]** the batch version
tests `aVF.Index2()` / `aEF.Index2()` against the face set rather than `Contains()`; it relies on the
iterator's ordering convention (§2.8) putting the face second in VF and EF pairs.

`RefineFaceInfoIn()` (`:995-1024`) removes from In every pave block that is also in On. Called once
at `%BA%/BOPAlgo_PaveFiller.cxx:320`, *before* `MakeSplitEdges`.

### 2.7.4 Sc — section entities (`%BA%/BOPAlgo_PaveFiller_6.cxx:1694-1765`)

`UpdateFaceInfo` walks every `InterfFF`; for each intersection curve's pave blocks that were not
absorbed into an existing edge, it does `aFI1.ChangePaveBlocksSc().Add(aPB)` and
`aFI2.ChangePaveBlocksSc().Add(aPB)` (`:1733-1734`) — **the same handle into both faces' Sc sets**.
For each intersection point with a valid vertex index, `aFI1/2.ChangeVerticesSc().Add(nV1)`
(`:1759-1760`).

That single shared handle is the entire mechanism by which a section edge is *one* edge: when
`MakeBlocks` later appends the section edge to the arena and calls `aPB1->SetEdge(iE)`
(`%BA%/BOPAlgo_PaveFiller_6.cxx:1272`, `:1495`, `:1640`), both faces' Sc entries — being the same
object — see the same edge index. `BOPAlgo_Builder::BuildSplitFaces` then reads `aPB->Edge()`
(`%BA%/BOPAlgo_Builder_2.cxx:487`) and hands the same `TopoDS_Edge` to both faces' 2D arrangement.

### 2.7.5 Derived queries

`AloneVertices(f, out)` (`%DS%/BOPDS_DS.cxx:1028-1062`): collect all endpoint vertex indices of
`PaveBlocksIn ∪ PaveBlocksSc`; then every vertex in `VerticesIn ∪ VerticesSc` with index `>= 0` that
is not among them is "alone" (a section point with no section edge through it).

`SubShapesOnIn(f1, f2, &MVOnIn, &MVCommon, &PBOnIn, &commonPBs)` (`:1066-1143`): union the four
`PaveBlocksOn/In` maps into `PBOnIn` and their endpoint vertices into `MVOnIn`; a pave block of `f1`
that also appears in `f2`'s On or In map (**pointer identity**) goes into `commonPBs` and its
vertices into `MVCommon`; likewise vertices present in both faces' `VerticesOn/In`.

`SharedEdges(f1, f2, out, alloc)` (`:1147-1208`): for each edge sub-shape of `f1`, add
`RealPaveBlock(PB)->Edge()` for every PB (or the raw edge index if the edge has no pave blocks) to a
set; then for `f2`, emit every such index found in that set. **This returns materialised edge
indices, so it is only meaningful after `MakeSplitEdges`.**

## 2.8 `BOPDS_Iterator` — candidate pair generation

### 2.8.1 Fields (`%DS%/BOPDS_Iterator.hxx:117-129`)

```
handle<Allocator>                     myAllocator
int                                   myLength       // pairs of the currently-initialised type
BOPDS_PDS                             myDS           // raw pointer, not owned
DynamicArray<DynamicArray<BOPDS_Pair>> myLists       // 10 buckets, one per interference type
DynamicArray<BOPDS_Pair>::Iterator    myIterator
bool                                  myRunParallel
DynamicArray<DynamicArray<BOPDS_Pair>> myExtLists    // 4 buckets (VV,VE,EE,VF)
bool                                  myUseExt
```
`NbExtInterfs() == 4` (`:107`), documented as "VV, VE, VF interfering pairs; E/E is also
initialized but never filled".

### 2.8.2 `Prepare` and `Intersect` (`%DS%/BOPDS_Iterator.cxx:247-359`)

```
Prepare(ctx, checkOBB, fuzz):
    clear all 10 buckets; myLength = 0
    if myDS == null: return
    Intersect(ctx, checkOBB, fuzz)

Intersect(ctx, checkOBB, fuzz):
    tree = BOPTools_BoxTree();  tree.SetSize(NbSourceShapes())
    for i in [0, NbSourceShapes()):
        if !ShapeInfo(i).HasBRep(): continue            // :282-285  V/E/F only, no solids
        tree.Add(i, Bnd_Tools::Bnd2BVH(ShapeInfo(i).Box()))
    tree.Build()                                        // BVH_LinearBuilder (LBVH)

    sel = BOPTools_BoxPairSelector()
    sel.SetBVHSets(&tree, &tree);  sel.SetSame(true);  sel.Select();  sel.Sort()

    iPair = 0
    for iR in [0, NbRanges()):
        R = Range(iR)
        for (; iPair < nPairs; ++iPair):
            p = pairs[iPair]
            if !R.Contains(p.ID1): break                // :314-318  advance to next range
            if  R.Contains(p.ID2): continue             // :320-324  same operand -> drop
            SI1 = ShapeInfo(p.ID1);  SI2 = ShapeInfo(p.ID2)
            iT1 = TypeToInteger(SI1.ShapeType());  iT2 = TypeToInteger(SI2.ShapeType())
            if ((iT1 < iT2 && SI1.HasSubShape(p.ID2)) ||
                (iT1 > iT2 && SI2.HasSubShape(p.ID1))): continue    // :336-340
            if checkOBB && ctx->OBB(SI1.Shape(), fuzz).IsOut(ctx->OBB(SI2.Shape(), fuzz)): continue
            iX = TypeToInteger(SI1.Type(), SI2.Type())
            myLists(iX).Append(Pair(min(ID1,ID2), max(ID1,ID2)))    // :354-356
```

Two facts make the range loop correct:
1. `BOPTools_PairSelector::RejectElement` rejects `theID1 >= theID2` when `SetSame(true)`
   (`%BT%/BOPTools_PairSelector.hxx:87-90`), and elements are added to the tree in increasing DS
   index order, so **every emitted pair has `ID1 < ID2` in DS index terms**.
2. `Sort()` is `std::sort` on `PairIDs::operator<` = lexicographic `(ID1, ID2)`
   (`%BT%/BOPTools_PairSelector.hxx:38-44`, `:61`), and ranges ascend contiguously.

Hence: the loop walks ranges in order; within range `k` it consumes all pairs whose `ID1` is in
range `k`, dropping those whose `ID2` is also in range `k` (same-operand) and keeping the rest
(cross-operand). For the last range every pair has both IDs inside it, so nothing is emitted —
correct, since those pairs were already seen from the earlier range's side.

### 2.8.3 What the bounding structure actually prunes, and with which tolerances

The BVH is `BOPTools_BoxSet<double,3,int>` built by `BVH_LinearBuilder`
(`%BT%/BOPTools_BoxTree.hxx:25-45`). `Bnd_Tools::Bnd2BVH` copies `Bnd_Box::Get()` **including the
gap** into an axis-aligned `BVH_Box<double,3>` (`Bnd_Tools.hxx:35-41`). Node rejection is
`BVH_Box(min1,max1).IsOut(min2,max2)`; element rejection is
`myBVHSet1->Box(i).IsOut(myBVHSet2->Box(j))` (`%BT%/BOPTools_PairSelector.hxx:77-90`).

**There is no extra tolerance in the selector.** All tolerance lives in the box gaps set during
`Init`:

| entity | box | effective half-extent |
|---|---|---|
| vertex | `SetGap(tol(V) + addTol)`, `Add(P)` — `%DS%/BOPDS_DS.cxx:1605-1606` | `tol(V) + addTol` |
| edge | `BRepBndLib::Add(E)` (already `+tol(E)`) ∪ vertex boxes, then `SetGap(gap + addTol)` — `:1679-1688` | curve extent `+ tol(E) + addTol` |
| face | `BRepBndLib::Add(F)` (already `+tol(F)`) ∪ edge boxes, then `SetGap(gap + addTol)` — `:1717-1775` | surface extent `+ tol(F) + addTol` |

with

```
addTol = max(fuzz, Precision::Confusion()) * 0.5        %DS%/BOPDS_DS.cxx:312
Precision::Confusion() = 1.0e-7                          Precision.hxx:165
```

So an overlap test between two shapes admits pairs separated by up to
**`tol(A) + tol(B) + max(fuzz, 1e-7)`** — each side contributes `addTol = fuzz/2`. That is the
*only* geometric constant in the whole candidate-generation path, and it is a model-space length
(guarantee **G10**).

### 2.8.4 `Initialize`, `More`, `Next`, `Value` (`%DS%/BOPDS_Iterator.cxx:192-243`)

```
Initialize(T1, T2):
    myLength = 0
    iX = TypeToInteger(T1, T2)
    if iX >= 0:
        pairs = (myUseExt && iX < 4) ? myExtLists(iX) : myLists(iX)
        std::stable_sort(pairs)                  // :203  determinism, EVERY time
        myIterator.Init(pairs);  myLength = pairs.Length()

Value(&i1, &i2):
    (n1, n2) = myIterator.Value().Indices()
    iT1 = (int)ShapeInfo(n1).ShapeType();  iT2 = (int)ShapeInfo(n2).ShapeType()
    i1 = n1; i2 = n2
    if (iT1 < iT2): i1 = n2; i2 = n1          // :238-242
```

**Confirms audit E1.** The comparison is on the raw `TopAbs_ShapeEnum` where `FACE=4 < EDGE=6 <
VERTEX=7`, so `iT1 < iT2` means shape 1 is the *higher*-dimensional one, and the swap puts the
**lower-dimensional shape first**. Callers read `Value(nV, nE)`
(`%BA%/BOPAlgo_PaveFiller_2.cxx:163`), `Value(nV, nF)` (`%BA%/BOPAlgo_PaveFiller_4.cxx:153`,
`:187`), `Value(nE, nF)` (`%BA%/BOPAlgo_PaveFiller_5.cxx:185`, `:225`),
`Value(nF1, nF2)` (`%BA%/BOPAlgo_PaveFiller_6.cxx:298`). Getting this backwards inverts every
interference stage.

`ExpectedLength()` returns `myLength` (`:167-170`); `BlockLength()` returns
`max(1, (int)(0.5 * myLength))` (`:174-188`) and is a parallel-chunking hint only.

### 2.8.5 `IntersectExt` — the re-intersection pass (`%DS%/BOPDS_Iterator.cxx:363-463`)

Called by `RepeatIntersection` (`%BA%/BOPAlgo_PaveFiller.cxx:398`) after vertex tolerances have
grown. Builds a tree over all interfering non-solid shapes, but adds *inflated* boxes for the
indices in `theIndices` (taking the SD image's box), runs one `BoxTreeSelector` per inflated shape,
then keeps cross-rank pairs (`myDS->Rank(i) != myDS->Rank(j)`, `:435-438`), applies the same
sub-shape rejection, dedups with a fence map, and appends to `myExtLists` for types `< 4`. Sets
`myUseExt = true`, so subsequent `Initialize(VV/VE/EE/VF)` reads the extra lists instead.

### 2.8.6 `BOPDS_SubIterator` (`%DS%/BOPDS_SubIterator.cxx`)

A two-set variant: `SetSubSet1(list)`, `SetSubSet2(list)`, `Prepare()`, `Initialize()`. Builds one
BVH per subset (`:110-124`), `SetSame` **not** set, so mirrored pairs are possible — hence an
explicit `aPair.ID1 == aPair.ID2` skip (`:142-145`) and a `Map<BOPDS_Pair>` fence keyed on the
sorted pair (`:147-151`). Same sub-shape rejection (`:162-167`). `Value` has the identical
lower-dimension-first swap (`:61-78`). Used by `TreatVerticesEE`
(`%BA%/BOPAlgo_PaveFiller_4.cxx:356-363`) to test newly-created EE vertices against all faces.

### 2.8.7 `BOPDS_IteratorSI` (`%DS%/BOPDS_IteratorSI.cxx:39-...`)

Self-intersection variant: admits `IsInterfering()` (so SOLID enters the tree) and **omits the
range loop entirely**, so same-operand pairs are kept. `UpdateByLevelOfCheck(level)` clears buckets
`level+1 .. 9`. Not part of the boolean pipeline.

## 2.9 Interference records (`%DS%/BOPDS_Interf.hxx`)

Base `BOPDS_Interf` (`:34-205`):
```
int myIndex1 = -1, myIndex2 = -1, myIndexNew = -1
Indices(&i1,&i2)  SetIndices(i1,i2)  Index1()  Index2()
OppositeIndex(i)  -> the other index, or -1 if i is neither      :105-119
Contains(i)       -> i == myIndex1 || i == myIndex2              :129
SetIndexNew(i)  IndexNew()  HasIndexNew()  GetIndexNew() -> optional<int>   :137-176
```
Derived payloads:
- `InterfVV` — none.
- `InterfVE` — `double myParameter` (parameter of the vertex on the edge), `:302-303`.
- `InterfVF` — `double myU, myV`, `:378-380`.
- `InterfEE` — `IntTools_CommonPrt myCommonPart`, `:435-436`.
- `InterfEF` — `IntTools_CommonPrt myCommonPart`, `:497-498`.
- `InterfFF` — `bool myTangentFaces`, `DynamicArray<BOPDS_Curve> myCurves`,
  `DynamicArray<BOPDS_Point> myPoints`, plus `Init(nC, nP)` presizing, `:602-605`.
- `InterfVZ/EZ/FZ/ZZ` — none (checker-only, §2.0).

`BOPDS_Curve` (`BOPDS_Curve.hxx:33-...`) holds `IntTools_Curve myCurve`,
`List<handle<PaveBlock>> myPaveBlocks`, `List<int> myTechnoVertices`, `Bnd_Box myBox`,
`double myTolerance`. `BOPDS_Point` holds `gp_Pnt myPnt`, `gp_Pnt2d myPnt2D1/2`, `int myIndex`.

The summary table (`%DS%/BOPDS_DS.lxx:94-125`):
```
AddInterf(i1,i2): if myInterfTB.Add(Pair(i1,i2)) { myInterfered.Add(i1); myInterfered.Add(i2); return true }
                  else return false
HasInterf(i)     : myInterfered.Contains(i)
HasInterf(i1,i2) : myInterfTB.Contains(Pair(i1,i2))
```
`BOPDS_Pair::IsEqual` is order-insensitive and its hash sorts the pair
(`%DS%/BOPDS_Pair.hxx:70-99`), so `(i,j)` and `(j,i)` are the same key.

`HasInterfShapeSubShapes(i1, i2, any=true)` (`%DS%/BOPDS_DS.cxx:356-375`): `any_of`/`all_of` over
`ShapeInfo(i2).SubShapes()` testing `HasInterf(i1, sub)`.
`HasInterfSubShapes(i1, i2)` (`:379-385`): any sub-shape of `i1` interferes with any sub-shape of
`i2`.

## 2.10 Same-domain redirection (`%DS%/BOPDS_DS.cxx:1212-1253`)

```
ShapesSD()            -> DataMap<int,int>&                       (mutable, callers write directly)
AddShapeSD(i, iSD)    : if (i != iSD) myShapesSD.Bind(i, iSD)    // self-bind refused, cycles NOT
HasShapeSD(i, &iSD)   : walk the chain; true if at least one hop
GetSameDomainIndex(i) : walk the chain to a fixed point; returns i if no binding
```
Both walkers are unbounded loops (**[OCCT-GAP], see G11**).

`UpdatePaveBlockWithSDVertices(PB)` (`:1462-1473`) rewrites both paves' vertex indices through
`GetSameDomainIndex`. `UpdatePaveBlocksWithSDVertices()` (`:1449-1458`) sweeps the pool;
`UpdateCommonBlockWithSDVertices(CB)` (`:1477-1483`) does one block's members. The PaveFiller calls
the sweep after **every** interference stage (`%BA%/BOPAlgo_PaveFiller.cxx:266, 273, 280, 287, 328`).

## 2.11 `ReleasePaveBlocks` — marking untouched vs deleted edges (`%DS%/BOPDS_DS.cxx:1503-1543`)

```
for each pool list L:
    if L.Length() != 1: continue
    PB = L.First()
    if IsCommonBlock(PB): continue
    (n1, n2) = PB->Indices()
    if !IsNewShape(n1) && !IsNewShape(n2):        // both ends are original vertices
        ChangeShapeInfo(PB->OriginalEdge()).SetReference(-1)   // "untouched"
        L.Clear()
```
The in-source comment (`:1505-1513`) is explicit about the encoding, and it matters:

- **reference `-1`** → untouched edge → `FillImagesEdges` skips it
  (`%BA%/BOPAlgo_Builder_1.cxx:84-87`) and `BuildResult` emits the original shape
  (`%BA%/BOPAlgo_Builder_1.cxx:145-153`).
- **reference `>= 0` with an empty list** → the edge is *deleted* (it was too small to build even
  one pave block); `FillImagesEdges` binds it to an empty image list, so it never appears in the
  result.

## 2.12 The pipeline that drives all of this (`%BA%/BOPAlgo_PaveFiller.cxx:175-355`)

```
Init:      myDS = new BOPDS_DS(alloc);  myDS->SetArguments(args);  myDS->Init(fuzz)   :198-200
           myContext = new IntTools_Context                                            :203
           myIterator = new BOPDS_Iterator(alloc);  SetDS;  Prepare(ctx, useOBB, fuzz) :206-209
           SetNonDestructive()                                                         :212
PerformInternal:
   Prepare                                                                             :248
   PerformVV                                                                           :254
   PerformVE                                                                           :260
   UpdatePaveBlocksWithSDVertices                                                       :266
   PerformEE                                                                           :268
   UpdatePaveBlocksWithSDVertices                                                       :273
   PerformVF                                                                            :275
   UpdatePaveBlocksWithSDVertices                                                       :280
   PerformEF                                                                            :282
   UpdatePaveBlocksWithSDVertices;  UpdateInterfsWithSDVertices                          :287-288
   RepeatIntersection  (IntersectExt + VV + VE + VF on grown vertices)                   :291
   ForceInterfEE                                                                         :298
   ForceInterfEF                                                                         :305
   PerformFF                                                                             :312
   UpdateBlocksWithSharedVertices     [no-op unless NonDestructive, PF_6:3946-3951]      :318
   myDS->RefineFaceInfoIn()                                                              :320
   MakeSplitEdges                                                                        :322
   UpdatePaveBlocksWithSDVertices                                                        :328
   MakeBlocks                                                                            :330
   CheckSelfInterference; UpdateInterfsWithSDVertices                                    :336-338
   myDS->ReleasePaveBlocks();  myDS->RefineFaceInfoOn()                                  :339-340
   RemoveMicroEdges                                                                      :342
   MakePCurves                                                                           :344
   ProcessDE                                                                             :350
```
Note `myDS->Init(fuzz)` happens **before** the iterator is built, and `Prepare()` (the stage) is
distinct from `myIterator->Prepare()`.

## 2.13 Materialisation — how one edge index reaches two faces

This closes the edge-identity loop and is the part our kernel is missing.

**`MakeSplitEdges`** (`%BA%/BOPAlgo_PaveFiller_7.cxx:~390-549`) walks the pave-block pool:

- Degenerate edges (`aSIE.HasFlag()`) are skipped (`:410-414`).
- A common block is processed **once**, guarded by a fence map of CB handles (`:416-421`).
- **The no-split fast path** (`:425-468`): if neither pave vertex is a new shape and either
  (a) the block is a common block and some member's original edge has exactly one pave block —
  then `aCB->SetRealPaveBlock(thatPB)`, `aCB->SetEdge(nE)` (reusing the *original* edge index for the
  whole block!) and `UpdateEdgeTolerance(nE, ComputeToleranceOfCB(...))` (`:450-454`); or
  (b) the edge has exactly one pave block — then `aPB->SetEdge(nE)` (`:457-461`).
  Either way no new shape is appended.
- Otherwise a `BOPAlgo_SplitEdge` task is queued with `(E, V1, t1, V2, t2)` taken from the
  **representative** pave block when a CB is present (`:471-496`). The task computes
  `myTol = ComputeToleranceOfCB(myCB, myDS, myContext)` and
  `BOPTools_AlgoTools::MakeSplitEdge(E, V1, t1, V2, t2, ESp)` (`:135-138`).
- Collection (`:515-548`): a fresh `BOPDS_ShapeInfo` of type EDGE is appended with the split edge,
  its box, and sub-shapes `{Pave1().Index(), Pave2().Index()}`; then

```
  if (CB non-null): UpdateEdgeTolerance(nSp, task.Tolerance());  CB->SetEdge(nSp)     // :539-543
  else:             PB->SetEdge(nSp)                                                   // :544-547
```

`CB->SetEdge(nSp)` writes `nSp` into **every member pave block** — the members belong to different
original edges, possibly from different operands. One index, many owners. **That is G6, literally.**

**`SplitEdge(nE, nV1, t1, nV2, t2)`** (`%BA%/BOPAlgo_PaveFiller_7.cxx:553-585`) is the serial helper
used by the SD-vertex rebuild path (`%BA%/BOPAlgo_PaveFiller_6.cxx:3793-3801`), with the same
CB/PB dichotomy.

**`FillImagesEdges`** (`%BA%/BOPAlgo_Builder_1.cxx:71-126`):
```
for each source EDGE i with a reference:
    images[E] = []
    for each PB in PaveBlocks(i):
        PBR  = RealPaveBlock(PB)                      // canonicalise through the common block
        nSpR = PBR->Edge()
        images[E].append(Shape(nSpR));  origins[Shape(nSpR)].append(E)
        if IsCommonBlockOnEdge(PB): shapesSD[Shape(PB->Edge())] = Shape(nSpR)
```

**`BuildSplitFaces`** (`%BA%/BOPAlgo_Builder_2.cxx:~270-507`) then assembles each face's edge list:
- boundary edges via `myImages` (`:363-465`) — every face touching an original edge gets the *same*
  split-edge shapes out of the same image list;
- IN edges via `aMPBIn(j)->Edge()` (`:469-480`);
- section edges via `aMPBSc(j)->Edge()` (`:484-494`);
and hands the list to `BOPAlgo_BuilderFace`. **No coordinate matching anywhere in this path.**

## 2.14 Mutable-in-place and the `Change*` discipline

Every accessor comes in two forms: `X() const -> const T&` and `ChangeX() -> T&`. The mutating form
may **construct on demand**:

- `ChangePaveBlocks(i)` calls `InitPaveBlocks(i)` if the edge has no reference
  (`%DS%/BOPDS_DS.cxx:425-433`).
- `ChangeFaceInfo(i)` calls `InitFaceInfo(i)` if the face has no reference (`:726-734`).
- The `const` counterparts return a **static empty** object instead
  (`PaveBlocks`: `:412-421`; `FaceInfo`: `:713-722`).

The reason the structure is mutable in place is not convenience:

1. **Handle identity is the identity model.** Pave blocks are `Standard_Transient` handles held
   simultaneously by the pool, by `myMapPBCB`, by two faces' `FaceInfo` sets, and by a `BOPDS_Curve`.
   `SetEdge` on one of them is *observed by all of them*. Copying the DS would fork those aliases and
   destroy G6.
2. **Stages are strictly ordered and each is total** (§2.12). Stage `n+1` reads what stage `n` wrote
   through the same object; there is no message passing.
3. **Lazy pools.** Pave blocks and face infos exist only for edges/faces that something actually
   touched, so the arena stays proportional to the interference, not to the model.
4. **`myMapPBCB` is the only owner-of-record of common blocks.** `SetCommonBlock` rebinds; there is
   no back-pointer on the pave block. Everything else goes through the DS.

---

# 3. DATA STRUCTURES AND C++ DECLARATIONS FOR OUR PORT

New files `src/brep_bds.h` / `src/brep_bds.cpp`, per `kb/ARCHITECTURE_v2.md` §1. Names follow that
document (`Bds*`), corrected where OCCT disagrees (no `out` set in `BdsFaceInfo`).

Our `BRep` is **already an indexed arena** (`%US%/brep.h:77-87`): `m_surfaces`, `m_curves_3d`,
`m_curves_2d`, `m_vertices`, `m_topology_vertices`, `m_topology_edges`, `m_trims`, `m_loops`,
`m_faces`. Two faces of one operand that share an edge already reference the same
`m_topology_edges[i]` — verified for `create_box` (`%US%/brep.cpp:281-282` creates 12 edges;
`:284-...` builds 6 faces × 4 trims over them). **So the flatten step is a pure index shift; we get
OCCT's `IsSame`-based identity for free, per operand, and we never need a shape→index hash.**

```cpp
// src/brep_bds.h
#pragma once
#include "brep.h"
#include "point.h"
#include <array>
#include <vector>
#include <map>
#include <unordered_map>
#include <memory>

namespace session_cpp {

//////////////////////////////////////////////////////////////////////////////////////////
// 0. Scalars and small value types
//////////////////////////////////////////////////////////////////////////////////////////

constexpr double BDS_CONFUSION = 1e-7;      // OCCT Precision::Confusion(), Precision.hxx:165
constexpr int    BDS_SD_MAX_HOPS = 64;      // G11 cycle cap; OCCT has none

enum class BdsType : unsigned char { Vertex = 0, Edge = 1, Face = 2, Shell = 3, Solid = 4 };

/// Dimension-ordered rank used by the pair-type table. Mirrors TopAbs ordering
/// (SOLID 2 < FACE 4 < EDGE 6 < VERTEX 7) so the "lower dimension first" rule (2.8.4)
/// reads the same way: LARGER code == LOWER dimension.
inline int bds_type_code(BdsType t) {
    switch (t) { case BdsType::Solid:  return 2; case BdsType::Shell: return 3;
                 case BdsType::Face:   return 4; case BdsType::Edge:  return 6;
                 case BdsType::Vertex: return 7; }
    return 9;
}
inline bool bds_has_brep(BdsType t) {                     // BOPDS_Tools::HasBRep
    return t == BdsType::Vertex || t == BdsType::Edge || t == BdsType::Face;
}
inline bool bds_is_interfering(BdsType t) {               // BOPDS_Tools::IsInterfering
    return bds_has_brep(t) || t == BdsType::Solid;
}

enum class BdsInterfType : signed char {
    None = -1, VV = 0, VE = 1, EE = 2, VF = 3, EF = 4, FF = 5, VZ = 6, EZ = 7, FZ = 8, ZZ = 9
};
constexpr int BDS_NB_INTERF_TYPES = 10;                   // BOPDS_DS::NbInterfTypes()

/// Exact port of BOPDS_Tools::TypeToInteger(t1,t2), BOPDS_Tools.lxx:31-82.
BdsInterfType bds_pair_type(BdsType t1, BdsType t2);

/// Axis-aligned box carrying its own gap, so pair tests are `IsOut` on inflated boxes.
struct BdsBox {
    double lo[3] = { 1e300,  1e300,  1e300};
    double hi[3] = {-1e300, -1e300, -1e300};
    double gap = 0.0;
    bool   is_void() const { return lo[0] > hi[0]; }
    void   add(const Point& p);
    void   add(const BdsBox& b);              // unions extents; takes max of gaps? NO: see .cpp
    void   set_gap(double g) { gap = g; }
    double get_gap() const { return gap; }
    bool   is_out(const BdsBox& o) const;     // inflated by BOTH gaps
};

/// Half-open ownership window for one operand, INCLUSIVE on both ends (BOPDS_IndexRange).
struct BdsIndexRange {
    int first = 0, last = -1;
    bool contains(int i) const { return i >= first && i <= last; }
    int  size()          const { return last - first + 1; }
};

//////////////////////////////////////////////////////////////////////////////////////////
// 1. Arena slot
//////////////////////////////////////////////////////////////////////////////////////////

/// One entity of either operand, or one entity created during intersection.
/// Port of BOPDS_ShapeInfo. `shape` is a *reference into the source BRep*, never a copy:
/// (brep, local index). For entities created after Init, brep == nullptr and the geometry
/// lives in `bds.made` (§ BdsMadeGeometry below).
struct BdsShape {
    BdsType  type = BdsType::Vertex;
    const BRep* brep = nullptr;      ///< owning operand, or nullptr for a created entity
    int      local = -1;             ///< index into brep->m_topology_{vertices,edges}/m_faces
    int      made  = -1;             ///< index into bds.made_* when brep == nullptr

    BdsBox   box;
    double   tol = 0.0;              ///< MODEL-SPACE tolerance of this entity (G10)

    std::vector<int> subs;           ///< sub-shape arena indices; see the table below
    int      reference = -1;         ///< edge -> pave_block_pool idx; face -> face_info_pool idx
    int      flag = -1;              ///< >= 0 means "set"; 0 IS a valid value (§2.0)

    bool has_reference() const { return reference >= 0; }
    bool has_flag()      const { return flag >= 0; }
    bool has_brep()      const { return bds_has_brep(type); }
    bool has_sub(int i)  const;      ///< linear, like BOPDS_ShapeInfo::HasSubShape
};
```

`subs` contents, matching §2.4.3/2.4.4 exactly:

| type | after `init()` |
|---|---|
| Vertex | empty |
| Edge | its two vertex indices (one, for a seam/closed edge; plus synthetic ones we never create) |
| Face | {edge indices} ∪ {vertex indices} — **wires are gone** |
| Shell | face indices |
| Solid | shell indices; rewritten to {faces} ∪ {edges} only in the single-argument case |

```cpp
//////////////////////////////////////////////////////////////////////////////////////////
// 2. Paves and pave blocks
//////////////////////////////////////////////////////////////////////////////////////////

/// A vertex sitting at a parameter on an edge (BOPDS_Pave).
struct BdsPave {
    int    vertex = -1;      ///< arena index of the vertex
    double t      = 0.0;     ///< parameter on the edge's 3D curve

    /// Sort key is the PARAMETER ONLY (BOPDS_Pave::IsLess).
    bool less(const BdsPave& o) const { return t < o.t; }
    /// Identity is (vertex, parameter) EXACTLY (BOPDS_Pave::IsEqual).
    bool same(const BdsPave& o) const { return vertex == o.vertex && t == o.t; }
};

/// One INTERVAL of one edge. The edge is never mutated (G4).
/// Reference-counted because it is aliased by the pool, the CB map, two FaceInfo sets and
/// a section curve — exactly OCCT's handle<BOPDS_PaveBlock> aliasing (§2.14).
class BdsPaveBlock {
public:
    int     original_edge = -1;   ///< arena index of the edge this interval lives on
    int     edge          = -1;   ///< arena index of the MATERIALISED split edge; -1 until made
    BdsPave pave1, pave2;

    std::vector<BdsPave> ext;     ///< pending subdivisions, consumed by update()
    std::vector<int>     fence;   ///< vertex indices already in `ext` (BOPDS_PaveBlock::myMFence)

    double ts1 = 0.0, ts2 = 0.0;  ///< shrunk range
    BdsBox shrunk_box;            ///< void until set_shrunk_data
    bool   has_shrunk = false;
    bool   splittable = false;

    void range(double& t1, double& t2) const { t1 = pave1.t; t2 = pave2.t; }
    void indices(int& v1, int& v2)      const { v1 = pave1.vertex; v2 = pave2.vertex; }
    bool has_edge()        const { return edge >= 0; }
    bool is_split_edge()   const { return edge != original_edge; }
    bool is_to_update()    const { return !ext.empty(); }
    bool has_same_bounds(const BdsPaveBlock& o) const;   ///< order-insensitive index pair match

    /// Fenced by VERTEX index. Returns false when the vertex is already present.
    bool append_ext_pave(const BdsPave& p);
    /// Unfenced. NOTE (§2.5.3): our port also records the vertex in `fence` so that
    /// remove_ext_pave() works for these too -- OCCT's cannot. Documented divergence.
    void append_ext_pave1(const BdsPave& p);
    void remove_ext_pave(int vertex);
    bool contains_parameter(double t, double tol, int& vertex_out) const;  ///< scans `ext` only

    /// Subdivide. `use_own` == OCCT's theFlag. Produces (n-1) blocks from n sorted paves,
    /// each inheriting ONLY original_edge. Always clears ext + fence.
    void update(std::vector<std::shared_ptr<BdsPaveBlock>>& out, bool use_own = true);
};
using BdsPB = std::shared_ptr<BdsPaveBlock>;

//////////////////////////////////////////////////////////////////////////////////////////
// 3. Common block
//////////////////////////////////////////////////////////////////////////////////////////

/// A set of pave blocks that are ONE geometric reality, plus the faces they lie on.
class BdsCommonBlock {
public:
    std::vector<BdsPB> members;   ///< members[0] is the representative; see set_representative
    std::vector<int>   faces;     ///< arena face indices this block lies ON
    double             tol = 0.0; ///< ComputeToleranceOfCB result (G7); MODEL SPACE

    /// Insert keeping the minimal original_edge at the front (BOPDS_CommonBlock::AddPaveBlock).
    void add(const BdsPB& pb);
    void set_members(const std::vector<BdsPB>& v);   ///< clears, then add() each
    /// Deliberate override of the min-index rule (BOPDS_CommonBlock::SetRealPaveBlock).
    /// Records `representative_forced = true` so the choice is auditable (audit E3).
    void set_representative(const BdsPB& pb);
    bool representative_forced = false;

    const BdsPB& representative() const { return members.front(); }
    bool  contains(const BdsPB& pb)  const;   ///< pointer identity
    bool  contains_face(int f)       const;
    BdsPB member_on_edge(int orig_edge) const;   ///< null shared_ptr when absent (not a static!)

    /// Assign ONE materialised edge index to EVERY member. This is guarantee G6.
    void set_edge(int e) { for (auto& m : members) m->edge = e; }
    int  edge() const { return members.empty() ? -1 : members.front()->edge; }
};
using BdsCB = std::shared_ptr<BdsCommonBlock>;

//////////////////////////////////////////////////////////////////////////////////////////
// 4. Face info -- In / On / Sc. THERE IS NO "OUT" SET (corrects ARCHITECTURE_v2 §1).
//////////////////////////////////////////////////////////////////////////////////////////

/// Insertion-ordered set of pave blocks (OCCT NCollection_IndexedMap semantics: the
/// downstream face builder walks these by index, so order must be stable and reproducible).
class BdsPBSet {
public:
    bool add(const BdsPB& pb);                 ///< false if already present
    bool contains(const BdsPB& pb) const;
    void clear();
    int  size() const { return (int)m_v.size(); }
    const BdsPB& operator[](int i) const { return m_v[i]; }
    void remove_at(int i);                     ///< O(n); used by refine_face_info_*
    std::vector<BdsPB>::const_iterator begin() const { return m_v.begin(); }
    std::vector<BdsPB>::const_iterator end()   const { return m_v.end(); }
private:
    std::vector<BdsPB> m_v;
    std::unordered_map<const BdsPaveBlock*, int> m_ix;
};

struct BdsFaceInfo {
    int      index = -1;              ///< arena index of the face
    BdsPBSet pb_on,  pb_in,  pb_sc;
    std::vector<int> v_on, v_in, v_sc;   ///< sorted unique arena vertex indices

    /// OCCT's Clear() forgets to clear Sc (BOPDS_FaceInfo.lxx:55-61). Ours clears ALL SIX
    /// and the caller that wanted the old behaviour must say so explicitly.
    void clear();
};

//////////////////////////////////////////////////////////////////////////////////////////
// 5. Interference records
//////////////////////////////////////////////////////////////////////////////////////////

struct BdsInterf {
    int i1 = -1, i2 = -1;
    int new_index = -1;               ///< vertex/edge created by this interference, or -1
    bool contains(int i)      const { return i == i1 || i == i2; }
    int  opposite(int i)      const { return i == i1 ? i2 : (i == i2 ? i1 : -1); }
    bool has_new()            const { return new_index >= 0; }
};
struct BdsInterfVV : BdsInterf {};
struct BdsInterfVE : BdsInterf { double t = 0.0; };
struct BdsInterfVF : BdsInterf { double u = 0.0, v = 0.0; };
struct BdsInterfEE : BdsInterf { /* common part: filled by port_03 (EE) */ int common_kind = 0;
                                 double t1a=0, t1b=0, t2a=0, t2b=0; };
struct BdsInterfEF : BdsInterf { int common_kind = 0; double te_a=0, te_b=0; };
struct BdsInterfFF : BdsInterf {
    bool tangent = false;
    std::vector<int> curves;          ///< indices into bds.ff_curves
    std::vector<int> points;          ///< indices into bds.ff_points
};

/// A section curve of one FF pair, with its pave blocks -- the section analog of an edge.
struct BdsFFCurve {
    int  interf = -1;                 ///< index into bds.interf_ff
    NurbsCurve c3d;                   ///< the section geometry, computed ONCE per pair
    NurbsCurve pc1, pc2;              ///< pcurves on face 1 / face 2 (may be empty)
    std::vector<BdsPB> blocks;        ///< pave blocks laid on this curve
    std::vector<int>   techno_vertices;
    BdsBox box;
    double tol = 0.0;
};
struct BdsFFPoint { Point p; double u1=0,v1=0,u2=0,v2=0; int vertex = -1; };

/// Unordered pair key for the interference table.
struct BdsPair {
    int a = -1, b = -1;
    BdsPair() = default;
    BdsPair(int i, int j) : a(std::min(i,j)), b(std::max(i,j)) {}
    bool operator==(const BdsPair& o) const { return a == o.a && b == o.b; }
    bool operator< (const BdsPair& o) const { return a != o.a ? a < o.a : b < o.b; }
};

//////////////////////////////////////////////////////////////////////////////////////////
// 6. Geometry created after Init (new vertices, split edges, section edges)
//////////////////////////////////////////////////////////////////////////////////////////

struct BdsMadeVertex { Point p; };
struct BdsMadeEdge   { NurbsCurve c3d; int v1 = -1, v2 = -1; };   ///< arena vertex indices

//////////////////////////////////////////////////////////////////////////////////////////
// 7. THE ARENA
//////////////////////////////////////////////////////////////////////////////////////////

class BdsArena {
public:
    BdsArena() = default;
    BdsArena(const BdsArena&)            = delete;   ///< G12: never copied
    BdsArena& operator=(const BdsArena&) = delete;

    //--- construction -------------------------------------------------------------------
    /// Flatten `operands` into one index space and build boxes, sub-shape lists and the
    /// vertex->edge map. `fuzz` is a MODEL-SPACE length.
    void init(const std::vector<const BRep*>& operands, double fuzz = BDS_CONFUSION);
    void clear();

    //--- arena ---------------------------------------------------------------------------
    int  nb_shapes()        const { return (int)m_lines.size(); }
    int  nb_source_shapes() const { return m_nb_source; }
    bool is_new_shape(int i) const { return i >= m_nb_source; }
    const BdsShape& shape(int i)        const { return m_lines[i]; }
    BdsShape&       change_shape(int i)       { return m_lines[i]; }

    int  nb_ranges()          const { return (int)m_ranges.size(); }
    const BdsIndexRange& range(int k) const { return m_ranges[k]; }
    int  rank(int i) const;                          ///< -1 for created shapes

    /// Identity lookup for a SOURCE entity. `operand` is the index into the ctor's vector.
    /// No coordinates are read: this is a table lookup, the port of myMapShapeIndex.
    int  index_of_vertex(int operand, int local) const;
    int  index_of_edge  (int operand, int local) const;
    int  index_of_face  (int operand, int local) const;

    int  append_vertex(const Point& p, double tol);          ///< new shape, rank -1
    int  append_edge  (const NurbsCurve& c, int v1, int v2, double tol);

    //--- geometry access (uniform over source and created entities) ----------------------
    Point             vertex_point(int i)  const;
    const NurbsCurve& edge_curve  (int i)  const;
    std::pair<double,double> edge_range(int i) const;
    const NurbsSurface& face_surface(int i) const;

    //--- pave blocks ----------------------------------------------------------------------
    bool has_pave_blocks(int e) const { return shape(e).has_reference(); }
    const std::vector<BdsPB>& pave_blocks(int e) const;      ///< static empty if none
    std::vector<BdsPB>&       change_pave_blocks(int e);     ///< lazily inits (§2.14)
    void update_pave_blocks();                               ///< sweep the whole pool
    void update_pave_block(const BdsPB& pb);
    /// Sorted, deduplicated pave set of `e`. Asserts G5.
    void paves(int e, std::vector<BdsPave>& out) const;
    void init_pave_blocks_for_vertex(int v);                 ///< via the vertex->edge map
    void release_pave_blocks();                              ///< §2.11 untouched/deleted marking
    bool is_valid_shrunk_data(const BdsPB& pb) const;

    //--- common blocks ---------------------------------------------------------------------
    bool  is_common_block(const BdsPB& pb) const;
    BdsCB common_block(const BdsPB& pb) const;               ///< null when none
    void  set_common_block(const BdsPB& pb, const BdsCB& cb);
    BdsPB real_pave_block(const BdsPB& pb) const;            ///< CB ? CB->representative() : pb
    bool  is_common_block_on_edge(const BdsPB& pb) const;    ///< CB && members.size() > 1
    void  update_common_block(const BdsCB& cb, double fuzz); ///< §2.6.4
    bool  check_coincidence(const BdsPB& a, const BdsPB& b, double fuzz) const;  ///< §2.6.4

    //--- face info ---------------------------------------------------------------------------
    bool has_face_info(int f) const { return shape(f).has_reference(); }
    const BdsFaceInfo& face_info(int f) const;               ///< static empty if none
    BdsFaceInfo&       change_face_info(int f);              ///< lazily inits
    void face_info_on(int f, BdsPBSet& pb, std::vector<int>& v) const;
    void face_info_in(int f, BdsPBSet& pb, std::vector<int>& v) const;
    void update_face_info_on(int f);
    void update_face_info_in(int f);
    void update_face_info_on(const std::vector<int>& faces);
    void update_face_info_in(const std::vector<int>& faces);
    void refine_face_info_on();                              ///< drop PBs with edge == -1
    void refine_face_info_in();                              ///< drop In members also in On
    void alone_vertices(int f, std::vector<int>& out) const;
    void sub_shapes_on_in(int f1, int f2,
                          std::vector<int>& v_on_in, std::vector<int>& v_common,
                          BdsPBSet& pb_on_in, std::vector<BdsPB>& pb_common) const;
    void shared_edges(int f1, int f2, std::vector<int>& out) const;

    //--- same domain -------------------------------------------------------------------------
    void add_shape_sd(int i, int i_sd);                      ///< refuses i == i_sd
    bool has_shape_sd(int i, int& i_sd) const;
    int  resolve_sd(int i) const;                            ///< capped walk; asserts on cycle
    void update_pave_blocks_with_sd_vertices();
    void update_pave_block_with_sd_vertices(const BdsPB& pb);
    void update_common_block_with_sd_vertices(const BdsCB& cb);

    //--- interferences ------------------------------------------------------------------------
    std::vector<BdsInterfVV>& interf_vv() { return m_vv; }
    std::vector<BdsInterfVE>& interf_ve() { return m_ve; }
    std::vector<BdsInterfEE>& interf_ee() { return m_ee; }
    std::vector<BdsInterfVF>& interf_vf() { return m_vf; }
    std::vector<BdsInterfEF>& interf_ef() { return m_ef; }
    std::vector<BdsInterfFF>& interf_ff() { return m_ff; }
    std::vector<BdsFFCurve>&  ff_curves() { return m_ff_curves; }
    std::vector<BdsFFPoint>&  ff_points() { return m_ff_points; }

    bool add_interf(int i1, int i2);                         ///< false if already recorded
    bool has_interf(int i) const;
    bool has_interf(int i1, int i2) const;
    bool has_interf_shape_subshapes(int i1, int i2, bool any = true) const;
    bool has_interf_subshapes(int i1, int i2) const;

    //--- diagnostics -----------------------------------------------------------------------
    void dump(std::FILE* f) const;
    bool check_invariants(std::string* why = nullptr) const; ///< G1..G12, cheap enough for tests

private:
    void init_shape(int idx);                                ///< recursive sub-shape flatten
    void prepare_vertices(double add_tol);
    void prepare_edges(double add_tol);
    void prepare_faces(double add_tol);
    void prepare_solids();
    void build_vertex_edge_map();
    void init_pave_blocks(int e);
    void init_face_info(int f);
    void init_face_info_in(int f);

    std::vector<const BRep*>   m_operands;
    std::vector<BdsShape>      m_lines;
    int                        m_nb_source = 0;
    std::vector<BdsIndexRange> m_ranges;

    // identity tables: (operand, local) -> arena index. One per entity kind.
    std::vector<std::vector<int>> m_ix_vertex, m_ix_edge, m_ix_face, m_ix_shell, m_ix_solid;

    std::vector<BdsMadeVertex> m_made_v;
    std::vector<BdsMadeEdge>   m_made_e;

    std::vector<std::vector<BdsPB>> m_pb_pool;      ///< indexed by BdsShape::reference
    std::vector<BdsFaceInfo>        m_fi_pool;      ///< indexed by BdsShape::reference
    std::unordered_map<const BdsPaveBlock*, BdsCB> m_pb_cb;

    std::map<int,int>               m_sd;
    std::map<int,std::vector<int>>  m_map_ve;

    std::vector<BdsInterfVV> m_vv;  std::vector<BdsInterfVE> m_ve;
    std::vector<BdsInterfEE> m_ee;  std::vector<BdsInterfVF> m_vf;
    std::vector<BdsInterfEF> m_ef;  std::vector<BdsInterfFF> m_ff;
    std::vector<BdsFFCurve>  m_ff_curves;  std::vector<BdsFFPoint> m_ff_points;

    std::vector<BdsPair>     m_interf_tb_sorted;    ///< or a hash set; must be deterministic
    std::vector<char>        m_interfered;          ///< bitmap over arena indices
};

//////////////////////////////////////////////////////////////////////////////////////////
// 8. Candidate pair iterator
//////////////////////////////////////////////////////////////////////////////////////////

class BdsIterator {
public:
    void set_arena(BdsArena* ds) { m_ds = ds; }
    /// Build the BVH over HasBRep shapes and fill the 10 buckets. `fuzz` model space.
    void prepare(double fuzz = BDS_CONFUSION, bool check_obb = false);
    /// Re-intersect after tolerances grew, filling the 4 extra buckets (§2.8.5).
    void intersect_ext(const std::vector<int>& grown_indices);

    void initialize(BdsType t1, BdsType t2);       ///< stable-sorts the bucket, resets cursor
    bool more() const { return m_cursor < (int)m_cur->size(); }
    void next()       { ++m_cursor; }
    /// LOWER-DIMENSION SHAPE FIRST (§2.8.4). Callers read value(nV,nE), value(nE,nF), ...
    void value(int& i1, int& i2) const;
    int  expected_length() const { return m_cur ? (int)m_cur->size() : 0; }

private:
    void intersect(double fuzz, bool check_obb);
    BdsArena* m_ds = nullptr;
    std::array<std::vector<BdsPair>, BDS_NB_INTERF_TYPES> m_lists;
    std::array<std::vector<BdsPair>, 4>                   m_ext_lists;
    bool m_use_ext = false;
    const std::vector<BdsPair>* m_cur = nullptr;
    int  m_cursor = 0;
};

/// Two-subset variant (BOPDS_SubIterator): pairs drawn from two explicit index lists.
class BdsSubIterator {
public:
    void set_arena(BdsArena* ds) { m_ds = ds; }
    void set_subsets(const std::vector<int>& s1, const std::vector<int>& s2);
    void prepare();
    void initialize();
    bool more() const;  void next();
    void value(int& i1, int& i2) const;            ///< same lower-dimension-first rule
private:
    BdsArena* m_ds = nullptr;
    std::vector<int> m_s1, m_s2;
    std::vector<BdsPair> m_list;
    int m_cursor = 0;
};

} // namespace session_cpp
```

## 3.1 Implementation notes that are not obvious from the declarations

**`init()` — the exact order (must match §2.4).**
```
1. m_operands = operands
2. for k, operand in operands:
       i1 = nb_shapes()
       append every SOLID of the operand, then recursively its shells/faces/edges/vertices
       through init_shape(), reusing indices via m_ix_* (identity by (operand, local))
       m_ranges.push_back({i1, nb_shapes()-1})
   m_nb_source = nb_shapes()
3. add_tol = max(fuzz, BDS_CONFUSION) * 0.5
4. prepare_vertices(add_tol); prepare_edges(add_tol); prepare_faces(add_tol); prepare_solids()
5. build_vertex_edge_map()
```
`init_shape` is the only place identity is established, and it never touches coordinates.
Because our `BRep` is index-based, `m_ix_edge[operand][local]` **is** `myMapShapeIndex`; there is no
hash and no `IsSame`. This is strictly stronger than OCCT (no pointer aliasing hazard) and is the
whole point of guarantee **G2**.

**Per-entity tolerances.** `BdsShape::tol` must be populated at `init()`:
- vertex: the operand's stored vertex tolerance if we have one, else `BDS_CONFUSION`;
- edge: the max deviation recorded when the edge was built, else `BDS_CONFUSION`;
- face: same, from the surface.

Our `BRep` currently stores **no** per-entity tolerance (`%US%/brep.h:27-56`: `BRepVertex`,
`BRepEdge`, `BRepTrim`, `BRepLoop`, `BRepFace` have no tolerance field). Until port_02 adds them,
`init()` seeds every tolerance to `BDS_CONFUSION` and **records that fact** in a
`tolerances_are_default` flag, so the acceptance tests can distinguish "measured 1e-7" from
"assumed 1e-7". Do not silently substitute a domain-relative value.

**`BdsBox::add(const BdsBox&)`.** OCCT's `Bnd_Box::Add(Bnd_Box)` unions the *inflated* extents and
keeps the receiver's gap. Our port must do the same: expand `lo/hi` by `o.lo - o.gap` / `o.hi +
o.gap`, leave `this->gap` alone. Then `prepare_edges` doing `set_gap(get_gap() + add_tol)` after the
union reproduces OCCT exactly.

**`BdsBox::is_out`.** `for k in 0..2: if (hi[k] + gap < o.lo[k] - o.gap) return true; if (o.hi[k] +
o.gap < lo[k] - gap) return true; return false;` — the sum-of-gaps semantics that yields
`tol(A) + tol(B) + fuzz` (§2.8.3).

**`resolve_sd`.** `int j = i; for (int h = 0; h < BDS_SD_MAX_HOPS; ++h) { auto it = m_sd.find(j); if
(it == m_sd.end()) return j; j = it->second; } assert(!"same-domain cycle"); return j;` — closes
**[OCCT-GAP] G11**.

**Determinism.** `initialize()` must `std::stable_sort` the bucket every time, as OCCT does
(`%DS%/BOPDS_Iterator.cxx:203`). All `BdsPBSet`s are insertion-ordered. No iteration order anywhere
may depend on pointer values or hash-table layout — that is what makes the "run 100×, byte-identical"
test in G9 meaningful.

**Threading.** The arena is single-writer. If the FF stage runs in parallel, each task writes into
its own `BdsFFCurve` and the merge back into the arena is serial, exactly as
`%BA%/BOPAlgo_PaveFiller_7.cxx:508-548` does for split edges.

---

# 4. WHAT OUR CODE DOES TODAY, AND WHERE IT DIVERGES

All paths below are under `%US%` = `/home/petras/code/code_rust/session/session_cpp/src`.

## 4.1 There is no arena

There is no shared data structure. `BRep::boolean` (`%US%/brep.cpp:8196`) splits each operand
independently — `split_by_brep` for B (`%US%/brep.cpp:8570`) and for A (`:8795`) — and then
reconciles the two results numerically. Every structure in §3 is new.

The closest existing thing is `SectionScaffold` (`%US%/brep_section.h:26-49`), which is a shared
*section* description (welded pave vertices + index-corresponded chains) but not an arena: it holds
no operand entities, no ranges, no interference log, no face info.

## 4.2 `SharedEdgePool` — what it provides, what it lacks

Declaration `%US%/brep_section.h:57-62`:
```cpp
struct SharedEdgePool {
    BRep arena;                                   // owns m_vertices/m_topology_vertices/
                                                  //      m_curves_3d/m_topology_edges
    std::vector<int> vert_tv;                     // scaffold vertex id -> arena topology-vertex idx
    std::vector<int> seg_edge;                    // seg_id -> arena edge idx (block 0 / whole seg)
    std::map<std::pair<int,int>, int> block_edge; // (seg_id, block) -> arena edge idx
};
```
Construction `%US%/brep_section.cpp:2583-2606`:
- one arena topology-vertex per scaffold pave vertex (`:2586-2591`);
- one arena edge per segment, degree-1 polyline through the shared chain, endpoints
  `vert_tv[v_start] / vert_tv[v_end]` (`:2593-2604`).

Consumption in `split_with` (`%US%/brep.cpp:3469-3474`): the result BRep is **pre-seeded** with the
pool's vertices, curves and edges (with `trim_indices` cleared), so identical indices exist in both
operands' results; `%US%/brep.cpp:3627-3629` seeds `pave_tv` from `pool->vert_tv`;
`%US%/brep.cpp:3975-3984` makes a whole-segment run *reference* `pool->seg_edge[seg_id]` instead of
minting.

**What it genuinely provides (keep this):**
- **P1.** A single edge object per section segment, created once, referenced by both operands →
  a working instance of G6 for section edges.
- **P2.** Welded pave vertices shared by both operands by index → a working instance of G2 for
  section endpoints.
- **P3.** Index-aligned pre-seeding, so combine can unify by index rather than by geometry.

**What it lacks (measured against §2):**

| missing | OCCT counterpart | consequence |
|---|---|---|
| L1. Any operand entity — it holds only section geometry | `myLines` (`%DS%/BOPDS_DS.hxx:484`) | original edges and faces have no shared identity at all; only sections do |
| L2. Ranges / rank | `myRanges`, `Rank()` (`%DS%/BOPDS_DS.cxx:207-224`) | no way to ask "which operand owns this"; cross-operand pruning is impossible |
| L3. Paves — an edge cannot be subdivided without re-minting | `BOPDS_Pave`, `InitPaveBlocks` | partial runs must be reconciled afterwards, see L5 |
| L4. Pave blocks as intervals | `BOPDS_PaveBlock` | `block_edge[{seg,block}]` is a flat map with no parameter range, no shrunk data, no ext-paves |
| L5. Common blocks: no members list, no faces list, no per-block tolerance, no representative | `BOPDS_CommonBlock` + `ComputeToleranceOfCB` (`%BA%/BOPAlgo_Tools.cxx:107-356`) | coincidence must be rediscovered geometrically later (`sew_coincident_edges`, `%US%/brep.cpp:7074`) |
| L6. FaceInfo In/On/Sc | `BOPDS_FaceInfo` | no IN set ⇒ no EF stage ⇒ `kb/hunt_efvf_gap.md` |
| L7. Interference log | `myInterfVV..FF` | nothing can ask "did these two already interfere" |
| L8. It is only ever populated from a section scaffold | — | **empty for pure coincidence**, the case it is most needed for; `%US%/brep.cpp:8494-8505` builds it only when `use_bop2` |
| L9. Per-entity tolerance | `BdsShape::tol` / `CommonBlock::tol` | every comparison downstream must invent a tolerance |

**Verdict: absorb, do not parallel.** `SharedEdgePool` is `BdsArena` restricted to (created
vertices, created section edges) with the pave-block layer flattened away. Concretely:

- `SharedEdgePool::arena.m_vertices` / `m_topology_vertices` → `BdsArena::m_made_v` plus arena slots
  of type `Vertex` with `rank == -1`.
- `SharedEdgePool::arena.m_curves_3d` / `m_topology_edges` → `BdsArena::m_made_e` plus arena slots of
  type `Edge` with `rank == -1`.
- `vert_tv` → the arena index returned by `append_vertex`; the scaffold keeps that index directly.
- `seg_edge` → `BdsFFCurve::blocks[0]->edge`.
- `block_edge[{seg, k}]` → `BdsFFCurve::blocks[k]->edge` — and the key becomes unnecessary, because
  a pave block *is* the block.

## 4.3 The divergences, with file:line

### D1 — Edge identity is coordinate coincidence after a boundary snap
`split_with` mints result edges through a position-keyed map. Declaration
`%US%/brep.cpp:3478-3479`:
```cpp
std::map<std::tuple<long long, long long, long long>, int> vmap;
std::map<std::tuple<int, int, long long, long long, long long>, int> emap;
```
`vmap` is keyed on `q6(x), q6(y), q6(z)` where `q6(x) = llround(x * 1e6)`
(`%US%/brep.cpp:2897`). The lookup scans the 3×3×3 neighbourhood and accepts within
`1e-12` squared, i.e. 1e-6 absolute (`%US%/brep.cpp:3745-3755`), then mints on miss (`:3756-3761`).
`emap` is keyed on `(lo_vertex, hi_vertex, q6(midpoint))` (`%US%/brep.cpp:4013`), looked up at
`:4015-4016`, inserted at `:4024`; the boundary-run variant is at `:4190-4196`.

Before that, two *snapping* layers run so that the coordinate key can succeed at all:
`pave_tv` capture within `pave_cap_tol` (`%US%/brep.cpp:3722-3741`), and `orig_tv` capture within a
diagonal-relative `5e-4` capped at a third of the closest original-vertex pair
(`%US%/brep.cpp:3646-3666`).

**Divergence from §2.4.1:** OCCT establishes identity by `IsSame` on the input topology — zero
coordinate reads. We establish it by quantised coordinate matching *after* two snaps. Any geometry
whose trims do not land exactly where the snap expects loses identity, which is exactly the measured
"32 naked of 36 on a padded UV domain".

**Fix:** `BdsArena::index_of_edge(operand, local)`. Once the splitter emits *references* to arena
edges instead of minted copies, `vmap`/`emap` have nothing left to do.

### D2 — Splitter tolerances are domain-relative
```
%US%/brep.cpp:4350   double eps_border = min(du.second-du.first, dv.second-dv.first) * 2e-3;
%US%/brep.cpp:4280   scaf_forced_eps = min(max(min_rangeF * 1e-2, 0.6 * scaf->tol3 / uv3dF),
                                           min_rangeF * 1.3e-1);
%US%/brep.cpp:4271   double ov = min_rangeF * 1e-2;                    // overshoot stub length
%US%/brep.cpp:3575   pave_cap_tol = max(1e-9, dg2 * 2e-3);             // diagonal-relative (OK-ish)
%US%/brep.cpp:2897   q6 quantizer: a fixed 1e-6 grid in MODEL units, but applied to values whose
                     meaning depends on the model scale
```
`min_rangeF` is the surface's UV domain extent — an arbitrary exporter choice. A 4× padded domain
inflates `eps_border`, `ov` and the floor of `scaf_forced_eps` 4×.

**Divergence from §2.8.3 / G10:** every constant in BOPDS is a model-space length
(`tol(V) + addTol`, `tol(E) + addTol`, `max(fuzz, 1e-7)`), and the only place OCCT converts to
parameter space is inside the intersectors, at the point of use.

**Fix:** `BdsShape::tol` in model space; `uv_tol = tol3d / |dS/du|` computed at the call site.

### D3 — There is no rank, so cross-operand pruning is done by geometry
`split_by_brep` builds `cutter_bbs` (`%US%/brep.cpp:3159-3161`) and tests surface AABBs, but there is
no notion of "these two entities belong to the same operand, skip". The same-operand pairs are
implicitly excluded because each operand is split against the *other* BRep — which also means
self-intersection is invisible.

**Divergence from §2.3 / §2.8.2 / G3.**

### D4 — No pave layer: subdivision re-mints geometry
Section runs are re-extracted from the scaffold chain per face and keyed by
`(lo, hi, q6(midpoint))` (`%US%/brep.cpp:4013-4024`); partial runs are reconciled afterwards by
`normalize_section_blocks` (declared `%US%/brep.h:~510`), which "splits at interior paves (3D
re-extracted from the shared chain)". That is a post-hoc rebuild of §2.5.5 `Update`, done on
materialised edges instead of on intervals.

**Divergence from §2.5 / G4.** OCCT never re-extracts: paves accumulate on the block, `Update`
subdivides the *interval list*, and geometry is produced exactly once at the end.

### D5 — Common blocks exist as a separate module, not as arena state
`brep_commonblock.{h,cpp}` is a faithful port of the *predicate* (`CBParams::band` is documented as
"tolA + tolB + fuzz at the call site", `%US%/brep_commonblock.h:71`) and of `ComputeToleranceOfCB`,
and it explicitly honours the "range must be as it was" invariant (`%US%/brep_commonblock.h:44-46`).
But `CommonBlock` there references a `const NurbsCurve*` and a `[t0,t1]` (`:47-49`), not pave blocks,
and nothing in `BRep::boolean` binds a `CommonBlock` to the edges that will be built. `CBMember::tol`
is documented as a P5 gap (`%US%/brep_commonblock.h:40`).

**Divergence from §2.6 / G6+G7.** In OCCT the common block *is* the thing that later receives
`SetEdge`, which is what makes coincident pieces literally one edge.
**Fix:** keep the predicate and the tolerance computation; re-host the container as
`BdsCommonBlock` holding `BdsPB` members.

### D6 — No FaceInfo, so no IN set and no EF stage
Nothing in `%US%` corresponds to `BOPDS_FaceInfo`. The splitter's per-face work is
`scaf->segs_by_surfA[face.surface_index]` (`%US%/brep.cpp:4352-4353`) — a section list, not an
In/On/Sc classification. Consequently there is no place for an edge-pierces-face vertex to be
recorded, matching the "no edge-face or vertex-face interference stage anywhere" defect.

**Divergence from §2.7.** `FaceInfoIn` step 3 (`%DS%/BOPDS_DS.cxx:867-889`) is precisely the missing
plumbing.

### D7 — Identity is repaired at the end by three successive geometric welds
```
%US%/brep.cpp:10364-10386  run_xweld(): quantize to 1e7, 3x3x3 neighbourhood, accept <= 1e-7,
                           rewrite edge endpoints
%US%/brep.cpp:10951        sew_coincident_edges(): Hausdorff match of 3D curves
%US%/brep.cpp:10961-10986  SESSION_FUZZY: re-run co_refine + sew with an inflated
                           diag*5e-3*mult tolerance, explicitly "approximate BY DESIGN"
```
Plus `imprint_edges` (`%US%/brep.cpp:5038`), `co_refine_coincident_edges` (`:5388`),
`make_shared_section_edges` (`:6732`), `snap_section_edges` (`:6995`).

**Divergence from §2.13 / G2+G6.** OCCT's builder has no equivalent stage: `FillImagesEdges`
(`%BA%/BOPAlgo_Builder_1.cxx:71-126`) and `BuildSplitFaces` (`%BA%/BOPAlgo_Builder_2.cxx:363-494`)
read edge indices out of pave blocks and hand the same `TopoDS_Edge` to both faces. Every weld in our
kernel is repair work for identity that was destroyed earlier.

The `SESSION_NO_SEW` gate (`%US%/brep.cpp:10951`) already exists to measure exactly this residual —
it is the natural acceptance harness for the port (§5.T7).

### D8 — Interference log absent
No structure records "V17 and F3 interfered". Every stage that would want `has_interf(i,j)` re-does
the geometry.

### D9 — `volume()` and solid ingestion
Out of scope for this file, but they bear on §2.4.5: `prepare_solids`/`BuildBndBoxSolid` is where
OCCT detects an open shell (`SetWhole`) and an inverted solid. Our port should surface the same two
verdicts as named results (`ShellOpen`, `SolidInverted`) so the "a sphere does not even ingest as a
solid" defect fails loudly at `init()` rather than silently downstream.

---

# 5. ACCEPTANCE TESTS

Each test names concrete operands, an analytically known answer, and the **oracle-free** invariant
it checks. All run in memory. `bds` is a `BdsArena`.

### T1 — Reparameterization invariance (**the defect this subsystem exists to kill**)
**Operands:** `BRep::create_box(2,2,2)`; and a copy `Bpad` in which every surface's UV domain is
rescaled by 4 (a pure knot-vector affine change; the surface point set is identical).
**Run:** `bds.init({&box}); bds2.init({&Bpad});`
**Invariant (oracle-free):** the two arenas are structurally identical —
`nb_shapes()` equal; the multiset of `(type, subs.size())` equal; for every edge, the sorted list of
faces referencing it equal; `m_map_ve` equal.
**Why:** G2 + G10. Today `split_by_brep` on `Bpad` yields 32 naked edges of 36; a correct arena
cannot see the difference at all.
**Fail signature to watch for:** any use of a `domain(` extent inside a tolerance expression.

### T2 — Arena census for a box (analytically known)
**Operand:** `BRep::create_box(sx,sy,sz)`.
**Known answer:** 1 solid, 1 shell, 6 faces, 12 edges, 8 vertices.
**Invariants:** `nb_source_shapes() == 28`; exactly one range `[0,27]`; every face's `subs` has 4
edges and 4 vertices after `prepare_faces`; every edge appears in the `subs` of exactly 2 faces;
`m_map_ve[v].size() == 3` for all 8 vertices; `rank(i) == 0` for all `i < 28`.

### T3 — Range partition for two operands
**Operands:** box(2,2,2) and sphere(r=1) (as soon as sphere ingests; else box + cylinder).
**Invariants:** `nb_ranges() == 2`; `range(0).first == 0`; `range(0).last + 1 == range(1).first`;
`range(1).last == nb_source_shapes()-1`; `rank(i)` is 0 exactly on range 0 and 1 exactly on range 1;
`rank(append_vertex(p, 1e-7)) == -1` and `is_new_shape` is true for it.
**Duplicate-argument case:** `init({&box, &box})` must yield `nb_ranges() == 1` and
`nb_source_shapes() == 28` (§2.3, `%DS%/BOPDS_DS.cxx:298-301`).

### T4 — Paves tile an edge (analytically known parameters)
**Operand:** a single straight edge from `(0,0,0)` to `(10,0,0)`, parameter `[0,10]`.
**Action:** append vertices at `x = 2, 5, 7`; `append_ext_pave` each onto the edge's single block;
`update_pave_blocks()`.
**Known answer:** 4 blocks with ranges `[0,2] [2,5] [5,7] [7,10]`.
**Invariants (G5):** `pave_blocks(e).size() == 4`; `paves(e).size() == 5`; parameters strictly
increasing and equal to `{0,2,5,7,10}` to 1e-15; `sum(t2-t1) == 10` exactly; the original edge's
curve control net hash is unchanged (G4).
**Degenerate sub-case:** append the same vertex index twice — `append_ext_pave` must return false the
second time and the block count must stay 4.

### T5 — Common-block tolerance is exactly the measured offset (**G7**)
**Operands:** edge `E1` = line `(0,0,0)→(10,0,0)` with `tol = 1e-7`; edge `E2` = line
`(0,d,0)→(10,d,0)` with `tol = t2`, for `d = 1e-4` and `t2 = 3e-6`.
**Action:** put both single pave blocks into one `BdsCommonBlock`, run `compute_tolerance_of_cb`.
**Known answer:** `tol = max(1e-7, t2 + d) = 3e-6 + 1e-4 = 1.03e-4`.
**Invariant:** `|CB.tol - 1.03e-4| < 1e-12`; and the 11 sample parameters are strictly inside
`(0,10)` (assert none equals 0 or 10).
**Face variant:** `E1` plus the plane `z = 0` offset to `z = d`: `tol == tol(face) + d`.

### T6 — One edge, two faces, no coordinates (**G2 + G6**)
**Operands:** box(2,2,2) and box(2,2,2) translated by `(1,0,0)` — the classic 15/20 case, planar so
it must be exact.
**Action:** run the arena `init`, iterator `prepare`, and (once the later ports land) the full
pipeline through materialisation.
**Invariants (oracle-free):**
1. Every result edge has exactly 2 trims (`trim_indices.size() == 2`) — no naked edges, no
   non-manifold edges.
2. **Zero calls** to any coordinate-weld routine: instrument `sew_coincident_edges`,
   `run_xweld`, `imprint_edges` with counters and assert all are 0.
3. `SESSION_NO_SEW=1` produces a byte-identical result to the default run.
4. Partition residual: `|vol(A∪B) + vol(A∩B) - vol(A) - vol(B)| < 1e-15 · vol(A)`.

### T7 — The residual gauge (regression harness, no oracle)
For each cell of the 224-cell primitive-pair matrix, run with `SESSION_NO_SEW=1` and record
`naked_edge_count`. **Invariant:** the count is monotonically non-increasing across ports. The port
of BOPDS alone will not fix curved cells — but it must not regress the 16 currently-genuine passes,
and it must reduce the box×box `SESSION_NO_SEW` naked count to 0 (today the sew is load-bearing even
there).

### T8 — Iterator completeness and determinism (**G9**)
**Input:** 500 random axis-aligned boxes assigned to two ranges, each with a random tolerance in
`[1e-9, 1e-3]`, plus a fuzz of `1e-5`.
**Invariants:**
1. The BVH result equals brute-force `O(n²)`: for all `i<j` with `rank(i) != rank(j)` and both
   `has_brep()`, `pair (i,j)` is emitted **iff** `!box(i).is_out(box(j))` and neither is a sub-shape
   of the other.
2. Each unordered pair appears at most once, and `a < b` in every emitted `BdsPair`.
3. 100 repeat runs produce byte-identical bucket contents.
4. `value()` returns the **lower-dimension** shape first: for a (Vertex, Face) pair, `i1` is the
   vertex. Test all six type combinations.

### T9 — Sub-shape self-pair rejection
**Operand:** box(2,2,2), single argument, so every pair is same-range and the iterator must emit
nothing at all for the boolean path. Then force two ranges by `init({&box, &box_translated_by_0})`
(coincident boxes): every face of A overlaps every face of B by box test, but a face must never pair
with its own edges/vertices. **Invariant:** for every emitted pair `(i,j)`,
`!shape(i).has_sub(j) && !shape(j).has_sub(i)`.

### T10 — FaceInfo On is derived and idempotent (**G8**)
**Operand:** box(2,2,2).
**Action:** `change_face_info(f)` for each face; then `update_face_info_on(f)` twice.
**Invariants:** `pb_on.size() == 4` for every face (one block per boundary edge);
`v_on.size() == 4`; the second `update_face_info_on` produces an identical set (compare by pave-block
pointer sequence); and the pave-block pointers in face `f1`'s `pb_on` for a shared edge are the
**same objects** as in face `f2`'s `pb_on` (`&*pb1 == &*pb2`) — this is the pointer-identity property
`SubShapesOnIn` relies on (§2.7.5).

### T11 — Same-domain chain terminates (**G11**)
**Action:** `add_shape_sd(5, 7); add_shape_sd(7, 9);` → `resolve_sd(5) == 9`.
`add_shape_sd(5, 5)` → refused, no binding created.
**Cycle:** `add_shape_sd(5, 7); add_shape_sd(7, 5);` → `resolve_sd(5)` must assert/abort within
`BDS_SD_MAX_HOPS`, not hang. (OCCT hangs here — `%DS%/BOPDS_DS.cxx:1244-1253`.)

### T12 — `release_pave_blocks` encodes untouched vs deleted (§2.11)
**Operands:** two boxes far apart (no interference) plus a third box that intersects one of them.
**Invariants:** every untouched edge has `reference == -1` **and** an empty pool list; every touched
edge has `reference >= 0` and a non-empty list; an edge that was too small to build a block has
`reference >= 0` and an **empty** list. These three states must be distinguishable by the caller,
because they map to "emit original", "emit images", "emit nothing".

### T13 — Immutability of source edges (**G4**)
Hash every source edge's `(degree, knots, control points)` after `init()`; run the full interference
pipeline; re-hash. **Invariant:** all hashes unchanged. Any stage that mutates a source curve fails
here immediately.

### T14 — Box gap arithmetic (**G10**, §2.8.3)
Two vertices at distance `d`, tolerances `t1`, `t2`, fuzz `f`.
**Invariant:** the iterator emits the pair **iff** `d <= t1 + t2 + max(f, 1e-7)`. Sweep `d` across
the threshold in 1e-12 steps and check the transition point to within one step. Repeat with an
edge/face pair to confirm `BRepBndLib`-equivalent tolerance inclusion.

---

# 6. IMPLEMENTATION ORDER

Smallest shippable increment first. Each step is independently testable and leaves the tree green;
nothing here changes any default behaviour until step 8.

| # | deliverable | files | gate |
|---|---|---|---|
| **1** | `BdsBox`, `BdsIndexRange`, `bds_type_code`, `bds_has_brep`, `bds_pair_type`, `BdsPair` | new `brep_bds.h/.cpp` | unit: the full `TypeToInteger` table (§2.0) reproduced for all 81 type pairs; T14 box arithmetic |
| **2** | `BdsShape` + `BdsArena::init()` (flatten, ranges, boxes, sub-shape rewrite, vertex→edge map) — no paves, no face info | `brep_bds.cpp` | **T1** (reparameterization invariance), **T2** (box census), **T3** (ranges), T13 immutability |
| **3** | `BdsIterator` (BVH or, for step 3, a brute-force `O(n²)` with the identical filter chain) + `BdsSubIterator` | `brep_bds.cpp` | **T8**, **T9**. Ship brute force first; swap in the BVH in step 3b behind the same test |
| **3b** | LBVH replacing brute force | `brep_bds.cpp` | T8 must still pass **set-equal** against brute force on 500 boxes |
| **4** | `BdsPave`, `BdsPaveBlock`, the pool, `init_pave_blocks`, `update`, `paves()`, `release_pave_blocks`, shrunk data | `brep_bds.cpp` | **T4**, **T12**; assert G5 in `paves()` |
| **5** | `BdsCommonBlock`, `m_pb_cb`, `real_pave_block`, `check_coincidence`, `update_common_block`; re-host `brep_commonblock`'s predicate and `compute_tolerance_of_cb` onto pave blocks | `brep_bds.cpp`, reads `brep_commonblock.h` | **T5**; plus: `set_edge` on a 3-member block sets all three `edge` fields (direct assertion of G6) |
| **6** | `BdsFaceInfo`, `BdsPBSet`, `face_info_on/in`, `update_*`, `refine_*`, `alone_vertices`, `sub_shapes_on_in`, `shared_edges` | `brep_bds.cpp` | **T10**; `refine_face_info_in` removes exactly the On∩In intersection on a hand-built case |
| **7** | Interference records + `add_interf` / `has_interf` / `has_interf_*_subshapes`, same-domain map with the cycle cap | `brep_bds.cpp` | **T11**; `add_interf` idempotence; `has_interf(i,j) == has_interf(j,i)` |
| **8** | **Absorb `SharedEdgePool`.** `build_shared_edge_pool` becomes `bds.append_vertex` / `bds.append_edge` + `BdsFFCurve::blocks`; `split_with`'s `pool` parameter becomes `const BdsArena*`; `seg_edge`/`block_edge` lookups become `ff_curves()[c].blocks[k]->edge` | `brep_section.{h,cpp}`, `brep.{h,cpp}` — gated by `SESSION_V2` | byte-identical corpus results with the gate off; with the gate on, the current BOP2 corpus numbers must not regress |
| **9** | Replace `vmap`/`emap` identity in `split_with` with arena references for **boundary** runs (section runs already reference the pool after step 8) | `brep.cpp` under `SESSION_V2` | **T6** clauses 1–3 on box×box; `SESSION_NO_SEW=1` box×box naked count → 0 |
| **10** | `check_invariants()` wired into the test harness; `dump()` | `brep_bds.cpp` | runs at the end of every corpus cell under `SESSION_V2`; G1–G12 all assert |

**Dependency notes.**
- Steps 1–7 touch **no existing file** — pure addition, zero regression risk, mergeable one at a time.
- Step 4 must precede step 5 (a common block holds pave blocks).
- Step 6 must precede any EF/VF work (port_04/05): `FaceInfoIn` step 3 is where EF results land.
- Step 8 is the first step that edits `%US%/brep.cpp`; it must be gated.
- **Do not attempt step 9 before step 8.** Removing the coordinate weld without a shared arena to
  replace it would take box×box from 15/20 to 0/20.

**What this subsystem does NOT fix.** The arena alone does not make a single curved cell pass. It
makes the *identity* defects unrepresentable and gives the later ports (VF, EF, FF, builder) a place
to write. The curved-surface failures are in port_02 (tolerance/geometry), port_04/05 (VF/EF) and
port_06 (FF/SSI coverage). What this port must deliver is: **after it, no downstream stage is allowed
to ask "are these two edges the same?" — it can only ask "is this the same index?"**
