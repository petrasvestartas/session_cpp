# pavefiller-core

Implementation spec extracted from OCCT sources at
`C:/brg/compas_occt/external/occt/src/occt/src/ModelingAlgorithms/TKBO/{BOPAlgo,BOPDS}`.
Files read verbatim: `BOPAlgo_PaveFiller.cxx/.hxx`, `_1.cxx`, `_7.cxx`, `_8.cxx`, plus targeted reads of
`_2/_3/_6/_10.cxx`, and `BOPDS_DS.cxx/.hxx`, `BOPDS_PaveBlock.*`, `BOPDS_CommonBlock.*`, `BOPDS_Pave.hxx`,
`BOPDS_FaceInfo.hxx`, `BOPDS_Curve.hxx`, `BOPDS_Point.hxx`, `BOPDS_Iterator.cxx`.

The PaveFiller is the **intersection phase** of all OCCT booleans: it computes every pairwise
sub-shape interference, splits edges at paves, welds same-domain (SD) vertices transitively,
builds common blocks for coincident edge pieces, and fills per-face FaceInfo (On/In/Sc sets)
that the downstream Builder (BOPAlgo_Builder / BOP) consumes to split faces and classify.

---

## STAGE PIPELINE

Master orchestration: `BOPAlgo_PaveFiller::PerformInternal` (`BOPAlgo_PaveFiller.cxx:234-355`).
`Perform` (`:217`) is just a try/catch wrapper adding `BOPAlgo_AlertIntersectionFailed`.
Every stage bails out on `HasErrors()`. Order is **strictly bottom-up by dimension**:
VV -> VE -> EE -> VF -> EF -> (repeat) -> (force EE/EF) -> FF -> split edges -> make blocks -> pcurves -> degenerated edges.

### 0. Init — `Init()` (`BOPAlgo_PaveFiller.cxx:175-213`)
- Purpose: build the DS and the pair iterator.
- Creates `BOPDS_DS`, `myDS->SetArguments`, `myDS->Init(myFuzzyValue)`; creates `IntTools_Context`;
  creates `BOPDS_Iterator`, `myIterator->Prepare(ctx, myUseOBB, myFuzzyValue)`; `SetNonDestructive()`
  (auto-on if any argument sub-shape is Locked).
- `BOPDS_DS::Init` (`BOPDS_DS.cxx:285-324`): appends every argument shape and recursively all
  sub-shapes into the indexed pool `myLines` (`InitShape` `:328`), assigns one `BOPDS_IndexRange`
  **per argument** (= rank), then per-type preparation:
  - `prepareVertices` (`:1589`): vertex box gap = `BRep_Tool::Tolerance(V) + addTol`.
  - `prepareEdges` (`:1614`): edge boxes (+vertex boxes), degenerated edges get `SetFlag(edgeIndex)`;
    **infinite curves** get synthetic vertices created at infinite parameters with `SetFlag(1)`.
  - `prepareFaces` (`:1696`): face sub-shape lists are FLATTENED — wires replaced by unique edge+vertex
    indices; degenerated edges get `SetFlag(faceIndex)` (flag == owning face).
  - `prepareSolids` (`:1783`): shells flattened to faces+edges; solid boxes via `BuildBndBoxSolid`
    (`:1390`) — open shell or inverted solid => `theBox.SetWhole()`.
  - `buildVertexEdgeMap` (`:1856`): vertex index -> unique list of edges containing it (`myMapVE`),
    used by `InitPaveBlocksForVertex`.
- Iterator `Prepare/Intersect` (`BOPDS_Iterator.cxx:247-359`): BVH box tree over all shapes with BRep;
  pair selection; keeps only **cross-rank** pairs (same-range pairs skipped `:320`), skips
  shape-vs-own-subshape (`:336`), optional OBB rejection (`:342`), buckets pairs per type-pair into
  `myLists(TypeToInteger(t1,t2))`. `Initialize` (`:192`) `std::stable_sort`s the pair list — the
  intersection order is deterministic and downstream results depend on it.

### 1. Prepare — `Prepare()` (`BOPAlgo_PaveFiller_7.cxx:850-932`)
- Purpose: pre-build pcurves of edges on **planar** faces involved in any interference
  (`BRepLib::BuildPCurveForEdgeOnPlane` in parallel via `BOPAlgo_BPC`), so later stages never race on it.
- Skipped entirely in non-destructive mode (must not modify inputs).

### 2. PerformVV — `_1.cxx:45-132`
- Iterate VERTEX/VERTEX pairs; if already interfering, just add to adjacency; else
  `BOPTools_AlgoTools::ComputeVV(V1SD, V2SD, myFuzzyValue)` (note: compares the **SD images**).
- Interfering pairs form a graph; `BOPAlgo_Tools::MakeBlocks(aMILI, aMBlocks)` computes **connected
  components**; each component welded by `MakeSDVertices` (below). Then for every SD-mapped vertex,
  `myDS->InitPaveBlocksForVertex(n1)` (`BOPDS_DS.cxx:1487`) forces pave-block creation on all edges
  through it (via `myMapVE`), so the new vertex index lands in their paves.

### 2a. MakeSDVertices — `_1.cxx:136-233` (the transitive weld primitive)
- Input: list of vertex indices (one connected component). Gathers all their shapes **plus existing SD
  images**; `BOPTools_AlgoTools::MakeVertex(aLV, aVn)` builds one vertex covering all (point+tolerance).
- If some input already had an SD image: that old vertex is **updated in place**
  (`BRep_TVertex::Pnt/Tolerance`, `:167-171`) and reused as the target index — keeps prior references
  valid. Else the new vertex is appended to DS.
- Box: `aBox.SetGap(BRep_Tool::Tolerance(aVn) + Precision::Confusion())` (`:184`).
- Every input `n1` gets `myDS->AddShapeSD(n1, nV)`; all pairs `(n1,n2)` get a `BOPDS_InterfVV` with
  `IndexNew = nV`; same-rank pairs additionally raise `BOPAlgo_AlertSelfInterferingShape` (`:208-219`).
- SD lookup is a **chain walk**: `HasShapeSD`/`GetSameDomainIndex` (`BOPDS_DS.cxx:1229-1253`) follow
  `myShapesSD` links until fixpoint — welding is transitive even across separate weld events.

### 3. PerformVE — `_2.cxx:141` + `IntersectVE` (`:212`)
- Vertex-on-edge: computes parameter, adds `InterfVE`, appends the vertex as an **extra pave**
  on the covering pave block; then `SplitPaveBlocks` (below) splits.
- Followed in `PerformInternal` by `UpdatePaveBlocksWithSDVertices()` (re-keys every pave to its SD
  image; `BOPDS_DS.cxx:1449-1473`). This call is repeated **after every stage** (lines 266, 273, 280,
  287, 328 in PaveFiller.cxx) — the standing invariant is "paves always reference SD representatives".

### 3a. SplitPaveBlocks — `_2.cxx:419-626` (the edge-split kernel)
- For each edge with `IsToUpdate()` pave blocks: `aPB->Update(aLPBN)` (sort paves, make sub-blocks;
  `BOPDS_PaveBlock.cxx:249-312`); each new PB gets `UpdatePaveBlockWithSDVertices` + `FillShrunkData`.
- **No valid shrunk range** (or unsplittable and its two vertices interfere by `ComputeVV`):
  the two end vertices are **unified** via `MakeSDVertices` (fence map on the vertex pair), and pave
  blocks are (re-)initialized for them (`:491-507`, `:620-625`). This is where "interval shorter than
  vertex tolerance spheres => weld endpoints" lives.
- If the split PB belonged to a CommonBlock: new PBs are regrouped into **new CommonBlocks keyed by
  vertex pair** (`aMInds`, `:538-553`); for **closed** CBs (nV1==nV2) grouping additionally verifies
  geometric coincidence via `myContext->ComputePE(midPoint, tolE1+tolE2+fuzzy, E, tOut, dist)` with the
  projection parameter required to fall inside the candidate's range (`:572-616`) — two halves of a
  closed edge must not be fused blindly by vertex-pair key (they share both vertices!).

### 4. PerformEE — `_3.cxx:145-...`
- Edge/edge via `IntTools_EdgeEdge` on **shrunk ranges**; produces `InterfEE` with common points
  (new vertices) or common parts (=> CommonBlocks of the two PBs). New vertices go through
  `PerformNewVertices` (`:594`)/`TreatNewVertices` (`:692`): cluster by tolerance-box intersection,
  one vertex per cluster (same weld pattern as VV). `FillShrunkData`/`AnalyzeShrunkData`
  (`:727-824`): shrunk range = PB range pulled in by vertex tolerance spheres via
  `IntTools_ShrunkRange`; failure => warnings (`TooSmallEdge` when whole edge, `BadPositioning`
  otherwise); box gap += `myFuzzyValue/2` (`:822`).

### 5. PerformVF — `_4.cxx:139` — vertex-in-face interferences (`InterfVF`, point-in-face classify),
  feeds FaceInfo VerticesIn later. `TreatVerticesEE` (`:305`) puts EE-new vertices into faces.

### 6. PerformEF — `_5.cxx:165`
- Edge/face on shrunk ranges; out: new vertices (`InterfEF` with `IndexNew`) or common parts =>
  **CommonBlock(PB, face)** — a PB lying IN a face records that face in the CB's face list.
- `ReduceIntersectionRange` (`:685`) shrinks the EF intersection range by already-known EE common
  ranges to avoid spawning near-duplicate vertices. `ForceInterfVE` (`_3.cxx:828`) / `ForceInterfVF`
  (`_5.cxx:631`) bump vertex tolerances to accept near-miss interferences.
- After EF: `UpdateInterfsWithSDVertices` (`_10.cxx:248`) re-keys `IndexNew` of all stored
  interferences through the SD map.

### 7. RepeatIntersection — `BOPAlgo_PaveFiller.cxx:359-424`
- Collect all vertices whose tolerance was increased during the run (`myIncreasedSS`, checked on the
  original index **and** its SD image); `myIterator->IntersectExt(set)` (`BOPDS_Iterator.cxx:363-463`)
  re-selects pairs against the enlarged boxes (cross-rank only, fence map, results into `myExtLists`);
  then re-runs **PerformVV, PerformVE, PerformVF** (not EE/EF) with SD updates between.

### 8. ForceInterfEE / ForceInterfEF — `_3.cxx:997` / `_5.cxx:772`
- After tolerance increases: look for **additional common blocks** among edge pairs (EE) and
  edge/face pairs (EF) that share bounding vertices but produced no interference earlier —
  catches coincidence that box-pair selection with original tolerances missed.
  (`_5.cxx:493`: extension distance clamped `aMaxDist = min(aMaxDist, 0.1)`.)

### 9. PerformFF — `_6.cxx:285-622`
- First refresh FaceInfo On/In for **all faces in FF pairs plus any face already having FaceInfo**
  (`myDS->UpdateFaceInfoOn/In(aMIFence)`, `:290-313`).
- Plane/plane pre-filter `CheckPlanes` (`:3639`): skip pairs with no interfering sub-shapes.
- **Seam-shift trick** (`:393-486`): if an EE intersection vertex between a closed (seam) edge of one
  face and an edge of the other projects to two points more than tolV apart on the two edges, one
  face is temporarily translated by that gap vector before SSI, and the shift value becomes a floor
  for `TolFF` (`:495`) — makes SSI curves reach the boundary. Result curves get `ApplyTrsf` back.
- `GetEFPnts` (`:2608`) feeds known EF intersection points into the face-face intersector as
  mandatory points (`aFaceFace.SetList`).
- SSI runs in parallel (`BOPAlgo_FaceFace`, `IntTools_FaceFace` with `bApprox`, approx tol `1.e-7`).
- Per result: `InterfFF` gets `BOPDS_Curve[]` (validated by `IntTools_Tools::CheckCurve`; box
  enlarged by `TolFF + max vertex tolerance of both faces` `:580-605`; curve tolerance =
  `max(IC.Tolerance(), aTolFF)`) and `BOPDS_Point[]`.

### 10. UpdateBlocksWithSharedVertices — `_6.cxx:3946` then `myDS->RefineFaceInfoIn()` (`BOPDS_DS.cxx:995-1024`)
- RefineFaceInfoIn: any PB present in both In and On of a face is removed from **In** (On wins).

### 11. MakeSplitEdges — `_7.cxx:371-549`
- Purpose: materialize one TopoDS edge per pave block / common block.
- `UpdateCommonBlocksWithSDVertices()` first (`_10.cxx:173`: in non-destructive mode also refreshes
  CB vertices with `UpdateVertex(nV, Precision::Confusion())`).
- Walk the whole `PaveBlocksPool`; skip degenerated edges (`aSIE.HasFlag()`); process each
  CommonBlock **once** (fence map `aMCB`).
- **Split avoidance** (`:423-468`): if neither pave vertex is new: for a CB, find a member edge whose
  PB list has exactly 1 PB — reuse that edge unsplit: `aCB->SetRealPaveBlock(thatPB)`,
  `aCB->SetEdge(nE)`, `UpdateEdgeTolerance(nE, ComputeToleranceOfCB(...))`; for a plain PB whose edge
  has a single PB — `aPB->SetEdge(nE)` (edge passes through untouched). Non-destructive mode restricts this.
- Otherwise split: for CBs always split the **representative** `aCB->PaveBlock1()`. Parallel
  `BOPAlgo_SplitEdge::Perform` (`:128-139`): `ComputeToleranceOfCB` then
  `BOPTools_AlgoTools::MakeSplitEdge(E, V1^FORWARD, t1, V2^REVERSED, t2)`, box + `Precision::Confusion()`.
- Post: append new edge to DS with `SubShapes = {pave1.Index, pave2.Index}` (`:529-537`);
  for CB: `UpdateEdgeTolerance(nSp, cbTol)` and `aCB->SetEdge(nSp)` (sets the edge on **every** member
  PB, `BOPDS_CommonBlock.cxx:195-205`); else `aPB->SetEdge(nSp)`.
- `SplitEdge(nE,nV1,t1,nV2,t2)` (`_7.cxx:553-585`) is the single-edge synchronous variant.

### 12. MakeBlocks — `_6.cxx:649-1137` (section-edge factory; the FF post-processing)
Per FF interference i (order matters; a recheck queue `aFFToRecheck` re-processes FF pairs whose
curves yielded no blocks on first pass, `:719-733`, `:1067-1071`):
1. `myDS->SubShapesOnIn(nF1,nF2, aMVOnIn, aMVCommon, aMPBOnIn, aMPBCommon)` (`BOPDS_DS.cxx:1066`):
   union of On+In PBs and vertices of both faces; "common" = PB/vertex present in both faces.
   `SharedEdges` (`:1147`) = split-edge images shared by both faces.
2. Points: `IsExistingVertex(P, tolFF, aMVOnIn)` (`:1950`, box + distance test) else make new vertex,
   queue in `aMSCPB` for PostTreatFF.
3. Curves — pave placement on each `BOPDS_Curve`:
   - `PutPavesOnCurve` (`:2372`) all On/In vertices (common vertices skip the bnd-box prefilter);
     internally `PutPaveOnCurve` (`:2959`) projects vertex onto curve, may **extend vertex tolerance**
     (`ExtendedTolerance` `:2542`) and records old tol in `aMVTol` for possible rollback.
   - `FilterPavesOnCurves` (`:2437`): a vertex put on several curves keeps only the best distance;
     kept tol = `max(saved, sqrt(maxDistKept)+Precision::Confusion())` (`:2533`).
   - `PutStickPavesOnCurve` (`:2748`) for near-tangent "stick" vertices (all vertices from any
     interference between subshapes of the two faces, `GetStickVertices` `:2847`);
     `PutEFPavesOnCurve` (`:2692`) only when the FF pair produced exactly 1 curve (`:823`).
   - `PutBoundPaveOnCurve` (`:2308`): paves at curve bounds (vertex tol floor `Precision::Confusion()`);
     bound vertices tracked in `aMVBounds`. `PutClosingPaveOnCurve` (`:3500`): for a closed 3D curve
     (period match within `PConfusion`), the start pave is mirrored to the end parameter.
4. BVH `aPBTree` over boxes of On/In pave-block edges (`:854-875`).
5. `aPB1->Update(aLPB, false)` splits the curve's proto pave block at all paves; per candidate block:
   - reject param span `< Precision::PConfusion()` (`:906`);
   - `IsValidBlockForFaces` — classify middle+bound points 2D-inside both faces (`:914`);
   - `IsExistingPaveBlock(vs SharedEdges)` (`:2047`): block coincides with an already-shared edge =>
     keep the edge, `UpdateEdgeTolerance(nEOut, aTolNew)`, no new section edge;
   - `BRepLib::FindValidRange` (`:937`): block fully inside its vertices' tolerance spheres => "micro"
     pave block; its vertices are queued for fusion in PostTreatFF (`aMicroPB`, `:952-959`);
   - `IsExistingPaveBlock(vs aMPBOnIn + BVH)` (`:1988`, max allowed tol growth `0.001` `:2110`):
     block coincides with an existing On/In PB of either face => **adopt** that PB: raise its edge
     tolerance to `max(aTolNew, curve tol)`, register the *other* face in `aPBFacesMap` (PB must be
     added to that face's In set later), collect its stray vertices in `aVertsOnRejectedPB`, and push
     the PB through `PreparePostTreatFF` (`:3609`) so it participates in section-edge unification;
   - else **make the section edge**: `BOPTools_AlgoTools::MakeEdge(IC, V1,t1, V2,t2, tolR3D, aES)` +
     `MakePCurve` on both faces (`:1024-1032`), append PB to the curve, register in `aMSCPB`;
     `ProcessExistingPaveBlocks` (`:3072`) additionally adopts existing On/In PBs that intersect the
     new edge's box (E-F distance pre-saved in `myDistances`).
6. Rollback of unused tolerance extensions from `aMVTol` (`:1075-1095`).
Post-loop: `RemoveMicroSectionEdges` (`:4308`); `MakeSDVerticesFF` (`:1141`, welds the vertex groups
collected in `aDMVLV` => `aDMNewSD`); **`PostTreatFF`** (`:1165`) — global unification: all section
edges + adopted PBs + micro-PB vertices go through a mini GF (vertices and edges fused; every
resulting PB re-keyed; ex-edges map `aDMExEdges`); `CorrectToleranceOfSE` (`:4072`) reduces section
edge/vertex tolerances back where possible (skip if reduction `< 0.1%` `:4225`); `UpdateFaceInfo`
(`:1673`) rebuilds FaceInfo On/In and inserts section PBs into **Sc** + `aPBFacesMap` Ins;
`UpdatePaveBlocks(aDMNewSD)` (`:3679`, below); `PutSEInOtherFaces` (`:4277`) — every section edge is
tested against all faces not involved in its creation, and if ON the face, added to that face's In
set (common-zone catch-all).

### 12a. UpdatePaveBlocks(aDMNewSD) — `_6.cxx:3679-3811` (post-weld PB rebuild)
- Every PB (pool + all curve PBs), CB members represented by `PaveBlock1`, once each: substitute SD
  images into pave1/pave2. If changed: re-split via `SplitEdge`; CB => `aCB->SetEdge(nSp)`.
- **Micro-edge detection**: if the PB was regular (two distinct vertices), is not degenerated, and
  now has nV1==nV2, run `FillShrunkData`; no shrunk data => the edge is micro: collected and removed
  by `RemovePaveBlocks` (`:3815`) from (1) the pool, (2) all FF curve PB lists, (3) all FaceInfo
  In/On/Sc maps.

### 13. CheckSelfInterference — `_11.cxx:28` (warning-only analysis) then
`UpdateInterfsWithSDVertices`; `myDS->ReleasePaveBlocks()` (`BOPDS_DS.cxx:1503-1543`): untouched
edges (single PB, both vertices original, no CB) get their PB-list reference removed so the Builder
reuses the original edge; small edges keep an **empty PB list = "deleted" marker**;
`myDS->RefineFaceInfoOn()` (`:975`): removes from On any PB that never got an edge (rejected pieces).

### 14. RemoveMicroEdges — `_6.cxx:4388` — final sweep of micro edges (checked via shrunk range).

### 15. MakePCurves — `_7.cxx:589-801`
- For every FaceInfo: build/attach pcurves for **In** PBs' edges on the face and for **On** PBs
  lacking one (for CB members, `AttachExistingPCurve` from a mate PB that has one, `:649-695`).
- For section edges (if `PCurveOnS1/2` requested): pcurves already built in MakeBlocks; here
  `UpdateVertices` (`:808-846`) bumps vertex tolerances to cover 3D-vs-2D end mismatch.
- Parallel `BOPAlgo_MPC`; periodic surfaces: `AdjustPCurveOnSurf` on a copy (thread-safe, `:263-282`).
- Failures raise `BOPAlgo_AlertBuildingPCurveFailed` with the offending E+F compound.

### 16. ProcessDE — `_8.cxx:54-131` (degenerated edges, LAST stage)
- For every edge flagged degenerated with owner face nF: take its (SD-image) vertex nV,
  `FindPaveBlocks(nV, nF)` (`:135`) = all PBs through nV in the face's **In+On+Sc** maps;
  `FillPaves` (`:224-331`): intersect the DE's 2D curve with each PB edge's 2D curve on the face
  (`Geom2dInt_GInter`, tol = resolution of vertex tol, below); fallback = 2D projection of the PB's
  end point. Valid interior points (`AddSplitPoint` `:368-393`: strictly inside range by tolCmp, not
  duplicate) become **extra paves**; `myDS->UpdatePaveBlock` then `MakeSplitEdge(nE,nF)` (`:163-214`)
  emits per-PB degenerated split edges (range set via `BB.Range(E, aF, t1, t2)`, still degenerated).
  If the DE references an edge (not a face), an empty-copy degenerated edge is emitted (`:105-123`).

---

## DATA STRUCTURES

### BOPDS_DS (`BOPDS_DS.hxx:64-503`) — the single source of truth
- `myLines : DynamicArray<BOPDS_ShapeInfo>` — indexed pool of ALL shapes (source first, then every
  shape created during the run). Index into this array is the universal currency of the algorithm.
  `myNbSourceShapes` splits source from new: `IsNewShape(i) = i >= NbSourceShapes()` (`:228`).
- `BOPDS_ShapeInfo` — shape, `ShapeType`, `Box`, flattened `SubShapes` list, `Flag` (degenerated
  edge => owner face; synthetic infinite vertex => 1), `Reference` — **type-punned pointer**: for an
  edge = index into `myPaveBlocksPool`, for a face = index into `myFaceInfoPool` (-1 = none; cleared
  reference + empty list marks a deleted small edge).
- `myRanges : DynamicArray<BOPDS_IndexRange>` — per-argument index ranges; `Rank(i)` = which operand
  a shape belongs to. Same-rank pairs are never intersected (self-intersection is CheckerSI's job).
- `myPaveBlocksPool : DynamicArray<List<Handle(BOPDS_PaveBlock)>>` — per source edge, the current
  partition of the edge into pave blocks (list mutates during the run).
- `myMapPBCB : DataMap<Handle(PaveBlock), Handle(CommonBlock)>` — PB -> CB membership.
  `RealPaveBlock(PB)` (`:658`) = CB ? CB->PaveBlock1() : PB — the canonical representative used
  everywhere identity matters (FaceInfo, SharedEdges).
- `myFaceInfoPool : DynamicArray<BOPDS_FaceInfo>`.
- `myShapesSD : DataMap<int,int>` — same-domain (weld) links, always followed transitively
  (`GetSameDomainIndex` `:1244`). `AddShapeSD` refuses self-links (`:1219`).
- `myMapVE : DataMap<int, List<int>>` — vertex -> edges through it (for pave-block (re)init on weld).
- `myInterfTB : Map<BOPDS_Pair>` — summary interference table (unordered index pair);
  `myInterfVV/VE/VF/EE/EF/FF/VZ/EZ/FZ/ZZ` — typed interference arrays. `InterfFF` **owns** the section
  curves/points. `myInterfered` — set of shapes with any interference.

### BOPDS_Pave (`BOPDS_Pave.hxx:27-93`)
- `{ myIndex /*vertex*/, myParameter /*t on edge*/ }`. `operator<` compares **parameter only**;
  `operator==` compares index AND parameter — deliberately inconsistent (comment `BOPDS_DS.cxx:1349`);
  sorting uses `<`, uniqueness uses `==`.

### BOPDS_PaveBlock (`BOPDS_PaveBlock.hxx:33-194`)
- `myOriginalEdge` — source edge index; `myEdge` — split-edge image index (-1 until MakeSplitEdges;
  `IsSplitEdge()` = differs from original). `myPave1/myPave2` — bounding paves (pave1.param < pave2.param).
- `myExtPaves + myMFence` — accumulator of future split points; `AppendExtPave` fences **by vertex
  index** (one pave per vertex), `AppendExtPave1` appends unconditionally (needed for closed/seam
  edges where one vertex owns two parameters, and for degenerated edges). `IsToUpdate()` = has extras.
- `Update(theLPB, theFlag)` (`BOPDS_PaveBlock.cxx:249`): gather (pave1,pave2 if theFlag)+extras,
  sort by parameter, emit consecutive-pair PBs inheriting `myOriginalEdge`. Used both for edge PBs
  (flag=true) and for curve proto-PBs (flag=false: bounds come from bound paves already in extras).
- Shrunk data: `myTS1/myTS2/myShrunkBox/myIsSplittable` — the range minus vertex tolerance spheres;
  `HasShrunkData` = box not void. `IsSplittable` = range long enough to host a split.
- `HasSameBounds` (`:148`) — unordered vertex-pair equality; the key for CB regrouping.

### BOPDS_CommonBlock (`BOPDS_CommonBlock.hxx/.cxx`) — coincidence class
- `myPaveBlocks : List<Handle(PaveBlock)>` — one PB **per edge** participating in the coincidence;
  `AddPaveBlock` keeps the PB with the **minimal OriginalEdge index first** (`:39-56`) so
  `PaveBlock1()` is a deterministic representative. `SetRealPaveBlock` promotes a chosen PB (unsplit
  reuse case). `myFaces : List<int>` — faces the block lies IN (from EF common parts).
- `SetEdge(theEdge)` writes the image edge into **all** member PBs (`:195-205`) — after which every
  operand edge shares literally the same split edge (this is how shared/sewn topology emerges).
- `myTolerance` — CB tolerance; computed by `BOPAlgo_Tools::ComputeToleranceOfCB` = max deviation of
  the representative edge from all mate edges/faces (used to fatten the image edge in MakeSplitEdges).

### BOPDS_FaceInfo (`BOPDS_FaceInfo.hxx:33-138`) — per-face state consumed by the Builder
- `myPaveBlocksOn : IndexedMap<Handle(PB)>` + `myVerticesOn : Map<int>` — PBs/vertices **on the
  boundary** of the face (from the face's own edges; PBs stored as `RealPaveBlock`, i.e. CB
  representatives; vertices SD-mapped). Built by `FaceInfoOn` (`BOPDS_DS.cxx:811`).
- `myPaveBlocksIn / myVerticesIn` — PBs/vertices **inside** the face: internal vertices of the face,
  V-F interference vertices, E-F IndexNew vertices, and `CommonBlock->PaveBlock1()` for every CB that
  `Contains(face)` (`FaceInfoIn` `:837-890`, `UpdateFaceInfoIn(map)` `:894-950`).
- `myPaveBlocksSc / myVerticesSc` — **section** PBs/vertices (filled by MakeBlocks/UpdateFaceInfo).
- Lifecycle: `InitFaceInfo` (`:738`) = In(internal vertices) + On; `RefineFaceInfoIn` drops In∩On;
  `RefineFaceInfoOn` drops On-PBs without an edge; `AloneVertices` (`:1028`) = In/Sc vertices not used
  by any In/Sc PB (the Builder inserts them as internal vertices of split faces).

### BOPDS_Curve (`BOPDS_Curve.hxx:33-119`)
- `myCurve : IntTools_Curve` (3D + both 2D pcurves + tolerance + tangential tolerance);
  `myPaveBlocks` — section PBs on this curve, `PaveBlock1` = proto-PB carrying the accumulated paves;
  `myTechnoVertices` — bound vertices to be removed/fused later; `myBox`; `myTolerance`
  (= max(IC tol, TolFF)). `TangentialTolerance` used in `aTolR3D = max(tol, tangTol)` (`_6.cxx:886`).

### BOPDS_Point (`BOPDS_Point.hxx:29-75`)
- `myPnt`, `myPnt2D1`, `myPnt2D2`, `myIndex` (vertex index once materialized).

### BOPDS_Iterator (`BOPDS_Iterator.hxx/.cxx`)
- Per type-pair lists of `BOPDS_Pair` (unordered min/max index pairs) from BVH selection; stable-sorted
  at `Initialize`; `Value` returns indices ordered so the **higher-dimension shape is first**
  (`:226-243`). `myExtLists`/`myUseExt` — extra pairs from `IntersectExt` (repeat-intersection mode).

### PaveFiller session state (`BOPAlgo_PaveFiller.hxx:638-661`)
- `myFPBDone` — fence map of face x PB intersections already done; `myIncreasedSS` — sub-shapes whose
  tolerance grew (drives RepeatIntersection); `myVertsToAvoidExtension` — vertices created near E/E,
  E/F intersections that must not be tolerance-extended again to reach section curves;
  `myDistances : DataMap<BOPDS_Pair, List<EdgeRangeDistance>>` — cached minimal E-F distances for
  pairs with no real intersection (reused by `ProcessExistingPaveBlocks` when tolerances grow).

---

## CONSTANTS & TOLERANCES

- **Fuzzy padding of ALL boxes**: `addTol = max(myFuzzyValue, Precision::Confusion()) * 0.5`
  (`BOPDS_DS.cxx:312`); vertex box gap = `tolV + addTol`; edge/face box gap += `addTol`.
- New/updated vertex box gap: `tol + Precision::Confusion()` (`_1.cxx:184`, `_10.cxx:123,148`,
  `_6.cxx:1089,3064`; split-edge box gap likewise `_7.cxx:138,581`).
- VV weld predicate: `BOPTools_AlgoTools::ComputeVV(V1,V2,myFuzzyValue)` — distance vs
  `tolV1+tolV2+fuzzy` on SD images (`_1.cxx:93`, `_2.cxx:484`).
- CB coincidence (`BOPDS_DS::CheckCoincidence` `:1288-1331`): project midpoint of PB1 on curve of E2;
  accept if `dist < MaxTolerance(E1,VERTEX)+MaxTolerance(E2,VERTEX)+max(fuzzy,Precision::Confusion())`
  AND the projection parameter is strictly inside PB2's range.
- Closed-CB regroup coincidence: `ComputePE(midPt, maxTolE1+maxTolE2+myFuzzyValue, ...)`, parameter
  strictly inside candidate range (`_2.cxx:604-605`).
- Shrunk-range validity re-check `IsValidShrunkData` (`BOPDS_DS.cxx:1547-1585`):
  `eps = BRep_Tool::Tolerance(edge)*0.01`; invalid when `tolV + Precision::Confusion()` exceeds the
  vertex-to-shrunk-end distance by more than eps. Shrunk box gap += `myFuzzyValue/2` (`_3.cxx:822`).
- **ToleranceFF** (`_6.cxx:3922-3941`): `max(tolF1, tolF2)`; if either surface is NOT analytic
  (plane/cylinder/cone/sphere/torus): floor at `5.e-6`. SSI approx tolerance fixed `1.e-7`
  (`:329`; `myTolFF` default `1.e-7` `:148`). TolFF floor additionally raised to the seam-shift value.
- Section curve: tolerance = `max(IC.Tolerance(), aTolFF)`; box enlarged by
  `aTolFF + max(MaxTolerance(F1,VERTEX), MaxTolerance(F2,VERTEX))` (`:580-607`).
  Block validity span cut: `Precision::PConfusion()` (`:906`). `aTolR3D = max(tol, tangentialTol)` (`:886`).
- Adopting existing PB for section edge: max allowed edge tolerance growth `aMaxTolAdd = 0.001`
  (**absolute**, model units; `_6.cxx:2110`).
- `CorrectToleranceOfSE`: skip reduction when `< 0.1%` of current value (`aTolV - aMaxTol < 0.001*aTolV`,
  `:4225`). Vertex tol updates elsewhere add `BOPTools_AlgoTools::DTolerance()` (= 1.e-12 headroom,
  `:2999, :3573`, `_7.cxx:843`).
- ForceInterfEF distance clamp: `aMaxDist = min(aMaxDist, 0.1)` (`_5.cxx:493`).
- ProcessDE 2D tolerances (`_8.cxx:243-267`): `aTolInt = max(Precision::PConfusion(),
  max(UResolution(tolV), VResolution(tolV)))`; boundary compare tol `aTolCmp` = same but only along
  the DE's direction (U-iso vs V-iso decided by `|Y(t1)-Y(t2)| < PConfusion`). Degenerated split edge
  built with hard `aTol = 1.e-7` (`:343`).
- Progress weights (cost model, `fillPISteps` `BOPAlgo_PaveFiller.cxx:438-477`): VV=1, VE=2, EE=5,
  VF=5, EF=10, FF=30 (glue:1), MakeSplitEdges=EE, MakeBlocks=5*FF, Repeat=0.2*(VV+VE+VF),
  ForceEE=2*EE, ForceEF=2*EF, MakePCurves=0.2*(EE+EF), ProcessDE=0.1*EE — a faithful ranking of
  where time goes.

---

## INVARIANTS

1. **Dimension monotonicity**: when stage (d1,d2) runs, all lower-dimensional interferences are
   final; EF sees all EE vertices, FF sees all EE+EF vertices and common blocks; MakeBlocks sees the
   complete pave partition of every operand edge.
2. **SD closure**: after every stage, `UpdatePaveBlocksWithSDVertices` (+`UpdateInterfsWithSDVertices`
   after EF/MakeBlocks) guarantees no pave/interference references a welded-away vertex; SD lookups
   chain to fixpoint, so welds compose transitively regardless of order.
3. **Pave ordering**: within a PB, pave1.param <= pave2.param; `Update()` output PBs are contiguous,
   sorted, non-overlapping, and exactly cover the parent range.
4. **Representative discipline**: every consumer of coincident geometry goes through
   `RealPaveBlock`/`CommonBlock::PaveBlock1` (min original edge index; deterministic); after
   MakeSplitEdges, all member PBs of a CB carry the **same** image edge index -> downstream Builder
   automatically produces shared (sewn) topology across operands.
5. **Every split edge covers its vertices**: shrunk-data check ensures a PB whose range is swallowed
   by vertex tolerance spheres never becomes an edge — its vertices get welded instead (SplitPaveBlocks,
   UpdatePaveBlocks(aDMNewSD), RemoveMicroSectionEdges, RemoveMicroEdges — four layers of defense).
6. **FaceInfo consistency at exit**: On = real boundary split-PBs with edges (RefineFaceInfoOn),
   In∩On = ∅ (RefineFaceInfoIn), Sc = section PBs valid 2D-inside both parents, every section edge
   ON any third face is registered in that face's In (PutSEInOtherFaces). Builder may assume any
   PB in FaceInfo has an edge and 2D validity.
7. **Tolerance honesty**: any geometry accepted at distance d gets tolerance >= d
   (`UpdateEdgeTolerance` cascades edge tol to its vertices `_10.cxx:63-101`); every tolerance
   increase is recorded (`myIncreasedSS`) and re-triggers pair discovery (RepeatIntersection +
   ForceInterf*), so acceptance bands never silently exceed detection bands.
8. **Determinism**: iterator pairs stable-sorted; CB representative = min index; MakeBlocks
   FF-processing order fixed + recheck queue compensates order-dependent misses.
9. **Non-destructive mode**: input shapes are never mutated; any tolerance/geometry update on an
   original shape first creates an SD copy (`UpdateVertex` `_10.cxx:105-162`).

## PITFALLS (explicitly handled in source)

- **Closed/seam edges**: single vertex, two parameters — `InitPaveBlocks` adds both paves with
  `AppendExtPave1` (unfenced) (`BOPDS_DS.cxx:479-484`); closed-CB regrouping must use geometric
  coincidence, not vertex-pair keys (`_2.cxx:557-616`); FF applies the face-shift trick when seam-edge
  intersections are offset (`_6.cxx:393-486`).
- **Degenerated edges**: flag = owner face; excluded from MakeSplitEdges; paves computed in 2D
  against pcurves of PBs through the apex vertex, with resolution-scaled tolerances; unconditional
  pave append (fence would collapse the two paves of the same vertex).
- **Infinite curves**: synthetic vertices fabricated at infinite parameters (`prepareEdges`), flagged,
  parameter recovered by point matching (`ComputeParameter` `BOPDS_DS.cxx:66-86` — explicitly noted
  "generally unsafe, preserved behavior").
- **Vertex over-extension**: `myVertsToAvoidExtension` prevents a vertex already extended for an E/E
  or E/F point from being extended again to reach a section curve; `aMVTol` rollback restores
  tolerances (and bnd boxes) of vertices whose extension ended up unused (`_6.cxx:1073-1095`).
- **E-F vertex on two curves**: `FilterPavesOnCurves` removes the worse placement — otherwise a
  vertex glued to a far curve bends the section topology ("bugs modalg_6 bug26789_1" comment `:802`).
- **Missing section edges due to FF processing order**: recheck queue (`aFFToRecheck`) re-runs FF
  pairs whose curves produced no blocks.
- **Section edge coincides with existing edge**: two distinct existence checks (vs shared edges, vs
  On/In PBs) with tolerance-growth cap 0.001; adopted PBs recorded in `aPBFacesMap` and later added
  to the other face's In set + pcurve-projected (`UpdateExistingPaveBlocks` `_6.cxx:3278`) — skipping
  this yields duplicate overlapping edges (our SEGWHOLE/SEGLOST analog).
- **Micro edges after welding**: an edge whose ends weld to the same vertex may legitimately be a
  closed loop — `UpdatePaveBlocks` distinguishes by shrunk-range success before declaring it micro
  (`_6.cxx:3781-3792`).
- **Small untouched edges**: `ReleasePaveBlocks` empty-list convention marks edges too small to ever
  get a PB as deleted, while releasing normal untouched edges for identity reuse.
- **Pave sort comparator**: `<` on parameter only, `==` on (index,parameter) — using a std::set would
  be wrong; DS code keeps both operations separate (`BOPDS_DS.cxx:1346-1386`).
- **pcurve races in parallel MakePCurves**: pcurve built on a **copy** edge, committed serially after
  the parallel phase; periodic surfaces re-adjust rather than rebuild (`_7.cxx:225-292`).

---

## PORT MAP

Anchors: `session_cpp/src/brep_section.cpp::build_section_scaffold` (SSI + chains + paves +
keep-verdict), `session_cpp/src/brep.cpp::split_with` (UV arrangement + run lifting),
`combine` (weld + common-block tube merge + classification). Our tolerances: `tol3 = diag*2e-3`,
`tol3_rep`; exact 1e-7 vertex weld.

1. **SD map with transitive chain-walk** (`BOPDS_DS::AddShapeSD/GetSameDomainIndex`)
   -> combine's exact 1e-7 vertex weld + NK-RESCUE -> **new build**: replace one-shot weld with a
   persistent `sd: map<vid,vid>` walked to fixpoint; every later pass (tube merge, NK-RESCUE,
   micro-collapse) writes links instead of rewriting geometry, and all run endpoints re-key through
   `sd` after each pass (mirrors `UpdatePaveBlocksWithSDVertices` called after every stage).
2. **Connected-component vertex welding** (`MakeSDVertices` on `BOPAlgo_Tools::MakeBlocks`
   components; one representative vertex covering all, old SD target updated in place)
   -> combine vertex weld -> **replace**: weld pairwise-interfering vertices as whole connected
   components (union-find over pairs within band), representative kept stable across passes;
   fixes order-dependence of the current pairwise 0.15*tol3 NK-RESCUE mate pass.
3. **ExtPave accumulate-then-split** (`BOPDS_PaveBlock::AppendExtPave/Update`; fence by vertex index,
   unfenced variant for seam/degenerate) -> split_with per-operand UV arrangement -> **adopt**:
   collect ALL split params per trim/chain first (trim crossings, chain-chain crossings, vertex
   projections), then one sort + consecutive-pair rebuild; unfenced duplicates only for periodic
   seam points. Our arrangement already approximates this; make the fence rule explicit.
4. **CommonBlock with face membership + deterministic representative + CB tolerance**
   (`BOPDS_CommonBlock`, `ComputeToleranceOfCB`, `SetEdge` writing all members)
   -> combine vertex-pair-keyed tube merge -> **adopt**: give merged tubes (a) a faces list (which
   faces the piece lies IN — feeds same-domain/imprint), (b) a deterministic representative
   (min seg_id), (c) a stored tolerance = max mate deviation, applied to the surviving edge instead
   of the global tol3 band. Closed tubes must re-verify coincidence by midpoint projection, not by
   endpoint-pair key (OCCT closed-CB rule) — our alias tube merge currently keys on vertex pairs only.
5. **Shrunk range / FindValidRange gate** (`FillShrunkData`, `AnalyzeShrunkData`,
   `BRepLib::FindValidRange` in MakeBlocks; no-valid-range => weld endpoints, never emit edge)
   -> our micro filter + micro-edge collapse -> **replace**: define interval validity as
   "range minus endpoint tolerance spheres non-empty"; if invalid, WELD the endpoints (SD link) rather
   than dropping the interval — dropping creates the wire gaps we chase (z30x20 sliver class);
   OCCT has four layers of this gate, we have one heuristic micro filter.
6. **FaceInfo On/In/Sc registry** (`BOPDS_FaceInfo`, `FaceInfoOn/In`, `RefineFaceInfoIn/On`,
   `SubShapesOnIn`, `AloneVertices`) -> implicit in split_with + classification flood -> **new
   build**: per-face explicit registry {On: trim runs, In: coincident/interior runs, Sc: section
   runs, verticesIn}; enforce In∩On=∅ (On wins) and On-only-with-edge; classification flood and
   face_outward_signs consume the registry instead of re-deriving membership; `AloneVertices`
   analog seeds isolated intersection vertices into split faces.
7. **Existing-edge adoption for section runs** (`IsExistingPaveBlock` x2 + `aPBFacesMap` +
   `UpdateExistingPaveBlocks` + `PutSEInOtherFaces`; growth cap 0.001) -> whole-segment alias keys
   (seg_id, tol 1e-2) + SEGLOST fix -> **adopt (highest value)**: when a marched section run
   coincides with an operand trim run (midpoint distance test with per-entity tol, not global band),
   do NOT emit a new run — adopt the trim run, record the *other* operand face as owner
   (aPBFacesMap analog), and raise that run's tolerance; afterwards test every section run against
   all non-parent faces and register ON hits as In-runs (PutSEInOtherFaces) — this is precisely the
   missing same-domain/coincidence subsystem entry point.
8. **Pave placement battery on section curves** (`PutPavesOnCurve` common-vs-noncommon,
   `FilterPavesOnCurves` best-curve-wins, `PutStickPavesOnCurve`/`PutEFPavesOnCurve` tangent cases,
   `PutBoundPaveOnCurve`, `PutClosingPaveOnCurve`) -> our paves = trim crossings + chain-chain
   crossings + vertex projections; PutToBoundary -> partially **already-equivalent** (bound + trim
   crossings); **adopt**: (a) stick-vertex projection — project ALL vertices arising from any
   interference of the two faces onto the chain with extended tolerance (fixes tangential stalls);
   (b) best-curve filter when one vertex projects onto several chains; (c) closing pave for periodic
   chains (formalizes our closure-weld cap as a pave, not a post-weld).
9. **Tolerance growth feedback loop** (`myIncreasedSS` + `RepeatIntersection` + `IntersectExt` +
   `ForceInterfEE/EF`, `UpdateEdgeTolerance` cascading edge->vertices) -> MISSING (no growth model)
   -> **new build**: track per-entity tol; whenever a weld/adoption raises a tolerance, add the
   entity to an `increased` set; after combine's first pass, re-run pair discovery (box test with
   enlarged boxes) restricted to that set and re-march/re-weld only affected pairs. Cheap version:
   one fixed re-pass, matching OCCT's single RepeatIntersection.
10. **Per-entity tolerance model** (`ToleranceFF` floor 5e-6 freeform; curve tol = max(IC tol,
    TolFF); vertex tol >= real distance + DTolerance; `CorrectToleranceOfSE` reduction)
    -> `tol3 = diag*2e-3` global weld band -> **replace incrementally**: keep tol3 as the *detection*
    band but store per-run/per-vertex *result* tolerances (corrector residual, adoption distance);
    combine acceptance then uses entity tolerances, and a final CorrectToleranceOfSE-style pass
    shrinks them (>=0.1% rule) — prerequisite for STEP export fidelity and Rhino trim acceptance.
11. **Deterministic ordering + recheck queue** (`std::stable_sort` of pairs, min-index CB rep,
    `aFFToRecheck`) -> our rotation-robustness campaign (order-sensitive symmetric-coverage floods)
    -> **adopt**: canonical sort of surface-pair processing order; re-queue pairs whose SSI chains
    yielded zero kept intervals for a second pass after all other pairs contributed paves — direct
    antidote for the partial-run classes seen at rotated configs.
12. **Degenerate-UV handling** (`ProcessDE`/`FillPaves`: split degenerate edges by 2D intersection of
    pcurves at apex/pole, resolution-scaled tolerances) -> our UV-arrangement cut-node crossing snap
    -> **already-equivalent in spirit**; adopt the resolution-based tolerance
    (`max(PConfusion, U/VResolution(tolV))`) instead of fixed UV epsilons for pole/apex rows.
13. **Seam-shift before SSI** (`PerformFF:393-486`: translate one face by the measured seam offset,
    floor TolFF by it, transform curves back) -> our predictor/corrector marcher on periodic
    surfaces (seam decomposition groundwork) -> **adopt as diagnostic**: when chain endpoints near a
    seam differ across operands by more than vertex tol, treat the offset as a tolerance floor for
    that pair's runs instead of welding blind (prevents half-circle trim_dir class regressions).
14. **Untouched-edge identity + deleted-edge marker** (`MakeSplitEdges` no-split path,
    `ReleasePaveBlocks` empty-list convention) -> split_with zero-span collapse -> **adopt**: when an
    operand trim acquires no interior paves, reuse the original run object (identity, not a rebuilt
    copy) so downstream welds are exact; mark too-small runs deleted explicitly instead of dropping
    them silently (distinguishes SEGLOST from legitimate collapse).
