# audit: occt_pavefiller-core

Audited against real OCCT V8 (`37dd5686f2`, "Bump version to 8.0.1.dev").
Paths below are relative to `/home/petras/code/code_cpp/OCCT`; two directory aliases are used:

- `%DS%` = `src/ModelingAlgorithms/TKBO/BOPDS`
- `%BA%` = `src/ModelingAlgorithms/TKBO/BOPAlgo`

so `%DS%/BOPDS_DS.cxx:312` etc. `PF_n` = `%BA%/BOPAlgo_PaveFiller_n.cxx`, `PF` = `%BA%/BOPAlgo_PaveFiller.cxx`.

---

## 1. VERDICT

**Substantially faithful; line anchors verified correct** (this V8 tree is the same tree the spec was written
from — `PF:266/273/280/287/328`, `PF_2.cxx:419-626`, `%DS%/BOPDS_PaveBlock.cxx:249-312`,
`%DS%/BOPDS_DS.cxx:1503-1543` all land exactly where the spec claims). The stage pipeline, the SD chain-walk,
the CommonBlock representative idea, the four-layer micro-edge defense and the tolerance-feedback loop are all
correctly described. **Six material errors and one large structural omission**:

- **E1 (wrong, breaks porting).** `BOPDS_Iterator::Value` does **not** put the higher-dimension shape first.
  `%DS%/BOPDS_Iterator.cxx:226-243` swaps when `iT1 < iT2` where `iT` is the raw `TopAbs_ShapeEnum`
  (`FACE=4 < EDGE=6 < VERTEX=7`), i.e. the **lower**-dimension shape comes first. That is why callers read
  `Value(nV,nE)` (`PF_2.cxx:163`) and `Value(nE,nF)` (`PF_5.cxx:225`).
- **E2 (wrong).** `UpdateBlocksWithSharedVertices` is a **no-op unless non-destructive**:
  `PF_6.cxx:3946-3951` returns immediately when `!myNonDestructive`. The spec lists it as an unconditional
  stage-10 and folds `RefineFaceInfoIn` into it; `RefineFaceInfoIn` is a separate call at `PF:320`.
- **E3 (half-wrong).** "CB representative = minimal original edge index, deterministic" holds for
  `AddPaveBlock`/`SetPaveBlocks` (`%DS%/BOPDS_CommonBlock.cxx:39-56`, `:60-68`) but is **deliberately broken by**
  `SetRealPaveBlock` (`:114-126`), used by the unsplit-reuse path (`PF_7.cxx:450`). Invariant 4 of the spec is
  therefore conditional, not absolute.
- **E4 (wrong).** The SD weld target is **not** min-index: `MakeSDVertices` reuses the SD image of the **first
  list element that already has one** (`PF_1.cxx:145-161`) and mutates its `BRep_TVertex` in place
  (`:167-171`). Determinism comes only from the `stable_sort` of iterator pairs, not from an index rule.
- **E5 (mis-attributed).** `aMaxDist = min(aMaxDist, 0.1)` at `PF_5.cxx:493` belongs to **PerformEF**'s
  "update existing vertex to cover the intersection point" branch (`aMaxDist = 1.e4*tolV`, clamped to 0.1 only
  when `tolV < 0.01`), not to `ForceInterfEF`. `ForceInterfEF` has entirely different gates (see §2.9).
- **E6 (wrong constant).** New EE/EF vertices get box gap `tol + myFuzzyValue/2`
  (`PF_3.cxx:607`, `:639`), **not** `tol + Precision::Confusion()`. The `+Confusion` form is used by
  `MakeSDVertices` (`PF_1.cxx:184`), `UpdateVertex` (`PF_10.cxx:123,148`) and the split-edge boxes
  (`PF_7.cxx:138,581`). Two different gap policies coexist.
- **O1 (structural omission).** The spec never mentions that **CommonBlock construction is centralized in
  `BOPAlgo_Tools`**: `PerformCommonBlocks` (two overloads, `%BA%/BOPAlgo_Tools.cxx:107-187` and `:191-244`) and
  `ComputeToleranceOfCB` (`:248-356`). All four CB-creating sites (EE, EF, ForceEE, ForceEF) funnel through
  them. This is the Law-1 primitive to port and it is absent from the spec.

Everything else checked matched. The additions in §2 are things a reimplementer cannot derive from the spec.

---

## 2. PORT-CRITICAL DETAILS THE SPEC MISSED

### 2.1 Index model and flags (get this wrong and nothing else works)

- **All DS indices are 0-based.** `myLines` starts at 0, `Rank()` scans `for (aRangeIndex = 0; ...)` and returns
  `-1` on failure (`%DS%/BOPDS_DS.cxx:214-224`); every source-shape loop is `for (i = 0; i < NbSourceShapes(); ++i)`.
- **`ShapeInfo::myFlag` defaults to `-1`, and `HasFlag()` is `myFlag >= 0`** (`%DS%/BOPDS_ShapeInfo.lxx:7-13`,
  `%DS%/BOPDS_ShapeInfo.lxx` `HasFlag`). Flag value **0 is a valid flag** (face index 0 owns a degenerate edge;
  edge index 0 is a degenerate edge). Using 0 as "no flag" silently disables degenerate handling for the first face.
- `Reference` defaults to `-1`; `HasReference()`/`HasPaveBlocks()`/`HasFaceInfo()` are the same predicate on the
  same field (`%DS%/BOPDS_DS.cxx:405-408`, `:706-709`) — an edge's Reference indexes `myPaveBlocksPool`, a face's
  indexes `myFaceInfoPool`. Type-punned as the spec says, but note both are the *same* accessor, so a shape that is
  neither edge nor face must never be asked.
- `prepareEdges` marks degenerate edges `SetFlag(edgeIndex)` (`%DS%/BOPDS_DS.cxx:1674`); `prepareFaces`
  **overwrites** it with the owning face index (`:1745`). Order matters: edges are prepared before faces
  (`:313-316`), so the final value is the face.
- **`prepareSolids` does nothing unless there is exactly one argument** (`%DS%/BOPDS_DS.cxx:1789-1792`). In a normal
  two-operand boolean, solids have void boxes at PaveFiller time; this is harmless only because
  `BOPDS_Tools::HasBRep` is true for V/E/F only (`%DS%/BOPDS_Tools.lxx`), so solids never enter the iterator BVH.

### 2.2 Init constants

`%DS%/BOPDS_DS.cxx:57` `THE_INITIAL_LINES_INCREMENT = 500`; `:292-293` ranges sized by argument count;
`:298-301` **duplicate arguments are skipped** (no second range); `:312`
`anAdditionalTolerance = max(theFuzz, Precision::Confusion()) * 0.5`; `:322-323` pools presized by edge/face counts.
Vertex box: `SetGap(tolV + addTol)` then `Add(pnt)` (`:1605-1606`). Edge box: own box + all vertex boxes, then
`SetGap(GetGap() + addTol)` (`:1678-1688`) — additive on the existing gap, not assignment. Face box likewise `:1775`.

### 2.3 `InitPaveBlocks` — how a topological edge becomes paves (`%DS%/BOPDS_DS.cxx:437-501`)

1. Empty sub-shape list ⇒ **no pave block at all, no reference set** (`:444-447`).
2. Non-INTERNAL edges: for each vertex, parameter via `BRep_Tool::Parameter`, **except** vertices flagged as
   synthetic-infinite, which use the unsafe `ComputeParameter` helper (`:462-463`, helper at `:66-86`, returns
   `0.` when the point matches neither curve end — documented as "generally unsafe … preserved behavior").
3. **Paves are SD-mapped at creation**: `aVertexIndex = GetSameDomainIndex(aVertexIndex)` (`:465`, `:495`).
4. Degenerate edge ⇒ `AppendExtPave1` (unfenced); otherwise `AppendExtPave` (fenced by vertex index) (`:467-474`).
5. **The seam rule is `aVertexIndices.Length() == 1`, not "closed curve"** (`:479-483`): when the edge has exactly
   one distinct vertex, a second unfenced pave is added at `BRep_Tool::Parameter(V.Reversed(), E)`.
6. INTERNAL-oriented edges take a separate branch iterating `TopoDS_Iterator(anEdge, false, true)` and use
   `AppendExtPave1` for every vertex (`:486-497`).
7. Final: `Update(pool.Appended(), /*theFlag=*/false)` — bounds come **only** from the ext-paves (`:499`).

### 2.4 `PaveBlock::Update` silently annihilates single-pave blocks (`%DS%/BOPDS_PaveBlock.cxx:249-312`)

`aNb = extPaves + (theFlag ? 2 : 0)`; **if `aNb <= 1` the ext paves and fence are cleared and *nothing* is
emitted** (`:263-268`). For `theFlag=false` (InitPaveBlocks, curve proto-PB) this is the path that produces an
*empty* pave-block list for an edge — which downstream code reads as "deleted". `std::sort` uses
`Pave::operator<` = parameter-only (`%DS%/BOPDS_Pave.lxx` `IsLess`), so paves with identical parameters keep
insertion order (std::sort is not stable — an unstable sort here is tolerated because equal-parameter paves
produce zero-length blocks that are filtered later).

`RemoveExtPave(vertNum)` (`:184-202`) removes **all** paves of that vertex and the fence entry — used by
`FilterPavesOnCurves`. A port that fences by (vertex,parameter) instead of vertex alone breaks this.

### 2.5 `PerformVE` guard ladder and the SD fence (`PF_2.cxx:141-395`)

Prefilter, in order (`:165-197`): edge already `HasSubShape(nV)`; edge `HasFlag()` (degenerate);
`HasInterf(nV,nE)`; `HasInterfShapeSubShapes(nV,nE)` (the vertex already interferes with *one of the edge's own
vertices* — `%DS%/BOPDS_DS.cxx:356-385`, `any_of`); PB list empty; **`aLPB.First()->IsSplittable()` false ⇒ micro
edge, skip** (`:192-197`). Only the *first* PB is used as the map key.

In `IntersectVE` (`:212-395`): `aMVPB` collects **every** pave index of the edge; a vertex whose SD image is
already a pave is skipped (`:247-268`). The solve runs on the **SD vertex**, and the resulting interference is
replayed for every original vertex that maps to it via `aDMVSD` (`:270-291`, `:366-387`).
`nVx = UpdateVertex(nV, aTolVNew)` may create a *new* vertex in non-destructive mode.

**The pave is attached to the PB whose open range strictly contains `aT` (`aT > aT1 && aT < aT2`, `:350`); if no
PB qualifies, the entire interference is dropped with `continue` (`:355-358`)** — no interference is recorded.

`FillShrunkData(VERTEX, EDGE)` runs *before* the loop (`:143`); `PerformEE` and `PerformEF` do the same
(`PF_3.cxx:147`, `PF_5.cxx:167`). Shrunk data is recomputed for a PB whenever
`!HasShrunkData() || !IsValidShrunkData()` (`PF_9.cxx:105`), computed in parallel, then analyzed serially (`:130-137`).

### 2.6 `SplitPaveBlocks` — the exact weld decision (`PF_2.cxx:419-626`)

For each new sub-PB: `UpdatePaveBlockWithSDVertices` then `FillShrunkData` (`:461-462`).
- `bHasValidRange = aPBN->HasShrunkData()`
- `bCheckDist = bHasValidRange && !aPBN->IsSplittable()` (`:468`)
- If `nV1 == nV2` ⇒ `continue` — **the sub-block is dropped and no weld happens** (`:473-477`). The spec's
  "no valid range ⇒ weld endpoints" is not universal.
- If `bCheckDist`, `ComputeVV(V1,V2,myFuzzyValue) == 0` demotes it to invalid (`:480-489`).
- Invalid ⇒ fence on `BOPDS_Pair(nV1,nV2)` (unordered hash, `%DS%/BOPDS_Pair.hxx:60-75`), `MakeSDVertices({nV1,nV2},
  theAddInterfs)`, remember both vertices for `InitPaveBlocksForVertex` **after the whole loop** (`:491-507`, `:620-625`).
  The sub-block is **not** appended.
- CB regroup (`:531-618`): key `BOPDS_Pair(pave1.Index, pave2.Index)`. Closed CB
  (`aCB->PaveBlock1()` has `nV1==nV2`) ⇒ greedy clustering by
  `ComputePE(midPointOfFirstMember, maxTolV(E_first) + maxTolV(E) + myFuzzyValue, E, tOut, dist)` with
  `tOut` **strictly inside** the candidate's own range (`:604-605`). Faces are inherited from the parent CB
  (`MakeNewCommonBlock`, `:401-415`).

### 2.7 EE: coincidence is only accepted when the pave partitions already agree (`PF_3.cxx:529-550`)

`case TopAbs_EDGE:` requires **`aNbCPrts == 1`** *and* **`aPB1->HasSameBounds(aPB2)`** — identical unordered
vertex pair. Otherwise `break`: the coincidence is thrown away at EE time and only recovered later by
`ForceInterfEE`. This is the load-bearing "watertight by construction" rule: shared topology is only created
between pave blocks that already span the same two global vertices.

Vertex-type common parts (`:369-526`) carry more machinery than the spec states:
- both PBs must be splittable (`:370-373`);
- four `IsOnPave1` flags against the un-shrunk end ranges with `aTol = Precision::Confusion()` (`:381-394`);
  if the hit is on paves of *both* blocks ⇒ skip (`:399-403`);
- otherwise `ForceInterfVE` is attempted for each on-pave end (`:406-417`), and if any existing vertex was hit,
  a *real* intersection is required (`aPOnE1.Distance(aPOnE2) <= Precision::Intersection()` = 1e-9, `:432`)
  before the existing vertex is grown by `UpdateVertex(nV[j], aDistPP)` and locked with
  `myVertsToAvoidExtension.Add` (`:440-451`);
- Line×Circle pairs get a tolerance floor of **half the common-part range** (`bAnalytical`, `:341-353`, `:455-466`);
- a new vertex is rejected if it lies within `100·(tolVnew+tolVx)²` (squared) of a vertex shared by both PBs
  (`:490-509`).

New EE/EF vertices are fused by `BOPAlgo_Tools::IntersectVertices` (BVH over boxes gapped by
`max(tolV, requestedTol) + fuzzy/2`, then connected components — `%BA%/BOPAlgo_Tools.cxx:1119-1200`), one
`MakeVertex` per chain (`PF_3.cxx:692-723`), appended with box gap `tol + myFuzzyValue/2` (`:639`), and finally
routed back through `IntersectVE(..., theAddInterfs=false)` (`:687`) so the *same* split machinery places them.

### 2.8 CommonBlock construction (the omitted subsystem)

`BOPAlgo_Tools::PerformCommonBlocks(PB → list<PB>)` (`%BA%/BOPAlgo_Tools.cxx:107-187`):
connected components via the BFS template `MakeBlocks` (`BOPAlgo_Tools.hxx:44-79`; `FillMap` is symmetric so the
BFS never dereferences a missing key, `hxx:82-100`); **groups of fewer than 2 PBs are skipped** (`:134-137`);
**the first member that is already in a CB donates that CB object**, and the faces of *all* member CBs are merged
into it, deduplicated (`:146-177`); then `SetPaveBlocks` (restores min-original-edge-first) and
`SetTolerance(ComputeToleranceOfCB(...))` (`:183-185`).

`PerformCommonBlocks(PB → list<face>)` (`:191-244`): one CB per PB, appends only faces not already present, then
recomputes the CB tolerance.

`ComputeToleranceOfCB` (`:248-356`): `tol = Tolerance(representative's ORIGINAL edge)`; **early exit if
`<2 PBs and no faces`** (`:266-269`); otherwise sample `aNbPnt = 11` interior points at
`dt = (t2-t1)/12` over the **representative's pave range** (`:271-278`), project each onto every mate edge
(`ProjPC`) and every attached face (`ProjPS`), and take
`tol = max(tol, Tolerance(mate) + LowerDistance())` (`:316-319`, `:345-348`).

`UpdateVerticesOfCB` (`PF_3.cxx:959-993`), called right after both EE and EF common-block creation
(`PF_3.cxx:563`, `PF_5.cxx:577`), pushes the CB tolerance into **both bounding vertices** of the representative.
That is the mechanism converting a coincidence band into vertex tolerance, which then drives
`myIncreasedSS` → `RepeatIntersection` → `ForceInterf*`.

`CommonBlock::SetEdge` writes into every member (`%DS%/BOPDS_CommonBlock.cxx:195-205`); `Edge()` reads only the
first member (`:209-217`).

### 2.9 EF and the Force* passes

**`PerformEF` (`PF_5.cxx:165-592`)**
- Per-PB skip if the face's `PaveBlocksOn()` already contains `RealPaveBlock(aPB)` (`:256-260`).
- Ranges are corrected by `BOPTools_AlgoTools::CorrectRange(E,F,...)` for both the shrunk and the full range
  (`:289-297`).
- **`myFPBDone[nF] += aPB` for every attempted pair, whether or not it intersects** (`:300-305`) — this is the
  memo that later suppresses redundant `ForceInterfEF` work.
- No common part but a finite minimal distance greater than `tolE+tolF` ⇒ cached in
  `myDistances[BOPDS_Pair(nE,nF)]` as `(t1,t2,dist)` (`:350-360`).
- On-pave decision uses `aTolToDecide = 5.e-8` (`:416`); `bLinePlane` (line vs plane) makes a single on-pave hit
  sufficient (`:421`).
- **A PB→face common block is only created when `CheckFacePaves` succeeds for BOTH pave vertices**, i.e. both
  endpoints are already in the face's `VerticesOn ∪ VerticesIn` (`:423-439` for the promoted-vertex case,
  `:553-559` for the genuine EDGE case). Otherwise only `AddInterf` is recorded. This is the EF analogue of the
  EE `HasSameBounds` rule.
- New EF vertex: `tolVnew = max(tolVnew, max(tolE,tolF))` (`:511`), line/plane floor of half the common range
  (`:513-519`), and a final `IsPointInFace(aPnew, aF, aTolVnew)` gate (`:523-526`).
- Ends with `myDS->UpdateFaceInfoIn(aMIEFC)` for every face that acquired an EF common part (`:585`).

**`ForceInterfEE` (`PF_3.cxx:997-1332`)**
- First re-initializes pave blocks for **every source vertex having any interference** (`:1009-1023`).
- Index: `BOPDS_Pair(nV1,nV2)` of the **RealPaveBlock**, one entry per Real PB (fence map), groups < 2 skipped.
- Same-rank pairs are allowed only when the shared vertices were *acquired*: skipped if either bounding vertex
  is an original shape of that same rank (`:1150-1160`).
- Already in the same CB ⇒ skip (`:1163-1169`).
- Extra fuzzy `2·max(tolV1,tolV2)` (`:1116-1118`), suppressed when the mid-point tangents deviate by more than
  ~25° (`|cos| < 0.9063`, `:1178-1205`); in self-interference mode (`myArguments.Extent()==1`) only `myFuzzyValue`.
- Accepts only `CommonParts().Length()==1 && Type()==TopAbs_EDGE` (`:1281-1291`).
- Before creating the CB it **expands each side into all members of its existing CB** (`:1313-1329`) so CBs merge
  transitively; then `PerformCommonBlocks` **without a context** (`:1332`) — `ComputeToleranceOfCB` then builds a
  throw-away `IntTools_Context` (`%BA%/BOPAlgo_Tools.cxx:280-284`).

**`ForceInterfEF` (`PF_5.cxx:772-1199`)** — also reused by `PutSEInOtherFaces` with `theAddInterf=false`
(`PF_6.cxx:4303`) and gated off entirely when `!myIsPrimary` (`:775-778`).
- BVH is built over **shrunk-range boxes**, recomputing shrunk data when stale (`:849-871`).
- A PB is a candidate only if the face's `VerticesOn ∪ VerticesIn ∪ VerticesSc` *plus the endpoints of all its
  On/In/Sc pave blocks* contains **both** PB vertices (`:915-964`).
- Rank check only applies when the PB has no edge yet (`:966-981`).
- Mid-point projection must satisfy `LowerDistance() <= 2·max(tolV1,tolV2) + myFuzzyValue` (`:1022-1029`) and
  land inside the face (`:1033-1036`).
- Angle gate: `|cos(∠(P_surf→P_edge, edge tangent))| > 0.4226` (25° off perpendicular) disables the extra
  tolerance (`:1044-1051`).
- `aTolAdd` = max over the two shrunk-range endpoints of their distance to the face, capped by `aTolCheck`,
  then reduced by `(tolE + tolF)` and floored at 0 (`:1064-1084`).
- If `aTolAdd == 0` the pair is intersected **only if it was not already attempted in PerformEF** (`myFPBDone`,
  `:1087-1092`).
- On a single EDGE common part the PB is added **directly to `FaceInfo::PaveBlocksIn`** (`:1186`) and only then,
  if `theAddInterf`, a CB is built (`:1187-1197`).

### 2.10 `MakeBlocks` details that change results (`PF_6.cxx:649-1137`)

- **`aTolFF` here is re-derived as `max(Tolerance(F1), Tolerance(F2))` (`:750`)** — it is *not* the `ToleranceFF`
  used for SSI (which has the 5e-6 free-form floor and the seam-shift floor, `PF_6.cxx:3922-3942`, `:495`).
  It is used for `IsExistingVertex` and for creating FF-point vertices (`:782-789`).
- `aNC.InitPaveBlock1()` (`:800`) puts the proto pave block at the head of the curve's PB list; after the block
  loop **`aLPBC.RemoveFirst()` (`:1065`)** drops it. Forgetting this leaves a bogus PB in every section curve.
- **Recheck queue semantics** (`:719-733`, `:879`, `:894-897`, `:1067-1071`): `isToRecheck` is armed when the FF
  has curves and this is a first-pass index; it is disarmed as soon as **any** curve yields ≥1 sub-block from
  `Update`. So the queue fires only for FF pairs whose curves carried fewer than 2 paves at processing time.
  `aNbFF` grows inside the loop and rechecked indices are read from `aFFToRecheck`.
- Order inside one FF: points → `GetStickVertices` → `PutPavesOnCurve` for all curves → `FilterPavesOnCurves`
  (across all curves) → per curve `PutStickPavesOnCurve`, `PutEFPavesOnCurve` (only if `aNbC == 1`),
  `PutBoundPaveOnCurve` → `PutClosingPaveOnCurve` for all curves → build the BVH of On/In PB edge boxes → make
  section edges. Splitting `FilterPavesOnCurves` per-curve would change results.
- BVH content: only On/In PBs that **have an edge** and whose original edge is not degenerate (`:854-875`);
  boxes come from `ShapeInfo(aPB->Edge()).Box()`.
- Per candidate block, in order: span `< Precision::PConfusion()` (`:906`) → `IsValidBlockForFaces` (`:914`) →
  `IsExistingPaveBlock(vs shared edges)` (`:922`) → `BRepLib::FindValidRange` (`:937`) →
  `IsExistingPaveBlock(vs On/In + BVH)` (`:962`) → make edge + pcurves (`:1024-1032`) →
  `ProcessExistingPaveBlocks` (`:1051`).
- `FindValidRange` is called with `max(aTolR3D, Tolerance(V))` per end (`:941`, `:944`); on failure the block is
  dropped and, **only if neither end is a bound vertex** (`aMVBounds`), the PB is queued in `aMicroPB` with both
  vertices (`:952-959`).
- Adoption bookkeeping (`:964-1021`): raise the adopted edge's tolerance to `max(aTolNew, aNC.Tolerance())`
  (`:976-985`); `aPBFacesMap[aPBOut] += theFaceThatLacksIt`; the *rejected* block's vertices are queued in
  `aVertsOnRejectedPB` unless they are already the adopted PB's own ends or bound vertices (`:1003-1012`);
  `PreparePostTreatFF` registers the adopted PB's **edge** in `aMSCPB` (`:1014-1018`).
- When a real section edge is made, its two vertices are **removed from the rollback map**
  (`aMVTol.UnBind`, `:1047-1048`).
- **Rollback** (`:1075-1095`) writes the saved tolerance straight into `BRep_TVertex` and **rebuilds the DS box
  from scratch** (`aBoxDS = Bnd_Box(); BRepBndLib::Add(...); SetGap(gap + Confusion)`), and un-binds the vertex's
  SD group from `aDMVLV` — a naive "restore the gap" implementation leaves the box inflated.

### 2.11 Existence tests, verbatim (`PF_6.cxx:1988-2251`)

**vs shared split edges** (`:1988-2043`): midpoint of the candidate block; box enlarged by `max(tolV1,tolV2)`;
per shared edge `ComputePE(aPm, max(tolE, max(tolV1,tolV2)) + myFuzzyValue, aE, tx, dist)`. On success the block is
dropped entirely (no PB anywhere) and the shared edge's tolerance is raised to the measured `dist` (`:922-930`),
with `UpdateSavedTolerance` propagating it into the rollback map (`:928`, helper at `:629-645`).

**vs On/In pave blocks** (`:2047-2251`):
- BVH pre-selection uses the **first** point's box only (`:2066-2080`) — a block whose first endpoint is far from
  every On/In edge is never adopted regardless of its midpoint.
- `aTolCheck = theTolR3D + myFuzzyValue`; `aMaxTolAdd = min(0.001, 10·aTolCheck)` (`:2106-2112`).
- `iFlag1/iFlag2` start at 2 when the vertex indices coincide, else 1 (or 0 ⇒ skip when the last point's box
  misses the edge's box) (`:2133-2138`).
- If the candidate is a CB: `aRealTol = max(aTolCheck, max(tolV1,tolV2)+fuzzy)`, **doubled** when the PB is common
  to both faces (`:2147-2155`).
- Else, when both ends coincide by index (`iFlag1==iFlag2==2`), and not the closed-vs-open mismatch
  `((nV11==nV12) != (nV21==nV22))` (`:2163-2164`), a tangency probe runs: `aTolAdd = 2·min(aMaxTolAdd,
  max(aRealTol, max(tolV1,tolV2)))`, `ComputePE` at the midpoint, and if the tangents agree
  (`|cos| >= 0.9063`) then `aRealTol = aTolAdd` and the reported tolerance is doubled (`aCoeff = 2`, `:2169-2196`).
- The **closest** matching PB wins; `theTolNew = aCoeff · aDistToSp` (`:2240-2248`).

### 2.12 Pave placement on curves — the constants

- `PutPavesOnCurve` (`:2372-2421`): EF vertices first with `iCheckExtend = 2`, then all On/In vertices with
  `iCheckExtend = 1`; **non-common vertices are additionally required to be new shapes** (`:2413-2416`) as well as
  passing the box test — the spec mentions only the box prefilter.
- `PutPaveOnCurve` (`:2959-3068`): `IsVertexOnLine(aV, aTolV, aIC, aTolR3D + myFuzzyValue, aT)`; on failure and
  when the vertex is not in `myVertsToAvoidExtension`, retry with `ExtendedTolerance` (`:2977-2990`).
  Duplicate detection uses `aPTol = GeomAdaptor_Curve::Resolution(max(aTolR3D, aTolV))` against existing ext
  paves (`:3002-3004`); a hit merges the two vertices into an `aDMVLV` group instead of adding a pave
  (`:3005-3039`). A miss appends the pave and grows the vertex to `dist + DTolerance()` where
  `DTolerance() = 1.e-12` (`%BA%/../BOPTools/BOPTools_AlgoTools.hxx:70`), saving the old tolerance in `aMVTol`.
- `ExtendedTolerance` (`:2542-2604`) only fires for **new** shapes (`:2548-2551`) and only when both indices of the
  originating EE/EF interference belong to the two faces (`aMI`, `:2583`); the extension is the larger distance
  from the vertex to the two ends of the stored common part (`:2587-2597`).
- `FilterPavesOnCurves` (`:2437-2538`): removal requires **both** `SquareDist > 100·max(tol², minSquareDist)`
  **and** `SinAngle < 0.5` (`:2515-2520`); if anything was removed, the vertex tolerance becomes
  `max(savedTol, sqrt(maxSquareDistKept) + Precision::Confusion())` (`:2533`).
- `PutStickPavesOnCurve` (`:2748-2843`): skipped when **both** curve ends already have bound vertices
  (`:2762-2766`); requires both 2D curves; `aDT2 = 2e-7` is a **squared** distance to a curve *end*, and
  `aDScPr = 5.e-9` is the crease criterion on `1 - |n1·n2|` (`:2793-2834`). It is an end-of-curve/crease device,
  not a general "project all interference vertices" pass.
- `PutEFPavesOnCurve` (`:2692-2744`) additionally requires the curve to be **Bezier or BSpline** (`:2708-2711`).
- `PutBoundPaveOnCurve` (`:2308-2368`): `getBoundPaves` (`:2255-2304`) finds the extreme ext-paves and nulls them
  when they are farther than `max(curveTol, tangentialTol) + Confusion` from the curve ends; closedness is
  `aP[1].IsEqual(aP[0], Precision::Confusion())`; if closed and either end already has a vertex, **nothing is
  added** (`:2325-2328`); new bound vertices require `IsValidPointForFaces` (`:2340`).
- `PutClosingPaveOnCurve` (`:3500-3605`): the end pave is matched by `|t - bound| < PConfusion`; closedness by
  `dist(V, oppositeBound) <= tolV + max(curveTol, tangTol) + Confusion`; then a **`BRepLib::FindValidRange` gate**
  with `aNewTolV = max(tolV, dist + DTolerance())`, and `UpdateVertex` may *replace* the vertex index before the
  mirrored pave is appended (`:3589-3604`).

### 2.13 `MakeSplitEdges` (`PF_7.cxx:371-549`)

- `UpdateCommonBlocksWithSDVertices()` first (`:392`). In destructive mode it is just
  `UpdatePaveBlocksWithSDVertices` (`PF_10.cxx:175-179`); in non-destructive mode it walks CBs once, bumps both
  vertices by `Precision::Confusion()` via `UpdateVertex` (creating SD copies), then re-keys (`PF_10.cxx:198-220`).
- No-split path requires `!IsNewShape(nV1) && !IsNewShape(nV2)` **and** `(!myNonDestructive || !bCB)`
  (`:426-432`) — a CB in non-destructive mode is always split.
- For a CB it looks for a member whose *original edge* has exactly one pave block (`:437-445`); on success
  `SetRealPaveBlock` + `SetEdge(nE)` + `UpdateEdgeTolerance(nE, ComputeToleranceOfCB(...))` (`:450-454`).
  For a plain PB the condition is `aLPB.Extent() == 1` ⇒ `aPB->SetEdge(nE)` (`:457-461`) — the PB's edge is the
  *original* edge index, so `IsSplitEdge()` is false afterwards.
- Split path: for a CB it always switches to `aCB->PaveBlock1()` and its original edge/vertices (`:471-476`).
  `V1` is forced FORWARD, `V2` REVERSED (`:482-486`).
- Worker (`:128-139`): `ComputeToleranceOfCB` then `MakeSplitEdge`, box + `Precision::Confusion()`.
- Commit (`:515-548`): new `ShapeInfo` gets `SubShapes = {pave1.Index, pave2.Index}`; CB ⇒
  `UpdateEdgeTolerance(nSp, cbTol)` **then** `aCB->SetEdge(nSp)` (all members); else `aPB->SetEdge(nSp)`.
- `UpdateEdgeTolerance` (`PF_10.cxx:63-101`) in non-destructive mode **refuses** to touch an original edge or an
  edge any of whose vertices is original and unaliased (`:69-85`); otherwise it grows the edge, re-adds its box
  with `+Confusion`, and cascades `UpdateVertex(nV, theTol)` to every sub-vertex.

### 2.14 `PostTreatFF` (`PF_6.cxx:1165-1669`) — the global unification

- Short-circuit: exactly one entry, no micro PBs, no rejected-PB vertices and no unused vertices ⇒ the single
  vertex/edge is appended to the DS directly with no fuse (`:1237-1276`).
- "Unused" vertices are computed per FF as `GetStickVertices` minus `RemoveUsedVertices`, and a vertex seen in
  **two or more** FF pairs is *dropped* from the unused set (`:1204-1231`).
- **All already-edged section blocks are packed into one `TopoDS_Compound` argument** (`:1280-1316`) so the
  nested PaveFiller's same-rank rule prevents them from being intersected against each other.
- Micro-PB vertex pairs are force-fused by inflating both tolerances by half the residual gap
  `dist - (tol1+tol2)` (`:1344-1358`).
- Nested filler: `BOPAlgo_PaveFiller aPF; aPF.SetIsPrimary(false); aPF.SetNonDestructive(myNonDestructive)`
  (`:1195-1197`). `IsPrimary(false)` disables `ForceInterfEF` (`PF_5.cxx:775-778`) and `SetNonDestructive()`
  auto-detection (`PF_10.cxx:43-46`).
- Result mapping (`:1407-1656`): every fused vertex writes **both** `aDMNewSD.Bind` and `myDS->AddShapeSD`
  (`:1451-1452`, `:1579-1592`); an edge whose nested image has zero pave blocks — or one PB without shrunk data
  — is treated as a micro edge, removed from the curve's list, and its vertices are appended to the work list for
  a second fuse pass (`:1507-1529`); CB tolerances from the nested DS are lifted into `aNC.SetTolerance`
  (`:1612-1626`); one `BOPDS_PaveBlock` is created per resulting edge index and memoized in `aMEPB` (`:1628-1641`).
- Final loop `:1659-1668` collapses `aDMNewSD` chains to a fixpoint and mirrors them into `myDS`.

### 2.15 `UpdateFaceInfo` (`PF_6.cxx:1673-1946`)

- Section PBs go into **both** faces' `PaveBlocksSc`; adopted (ex-edge) PBs are removed from the curve list and
  routed through `UpdateExistingPaveBlocks` (`:1712-1731`).
- `anEdgeLPB` groups PBs by resulting **edge index**; any group with ≥2 members becomes a new (or extended)
  CommonBlock, absorbing the members of any pre-existing CB and the union of their faces (`:1767-1858`). This is
  the step that makes two operands literally share one section edge.
- Vertex re-keying uses `Remove`+`Add` on the On/In maps (`:1893-1901`), so a mapping to an already-present index
  silently merges.
- On/In/Sc are then **rebuilt through `RealPaveBlock` with a global fence across all three maps** (`:1908-1944`):
  a PB that is representative for two of the maps lands in only the first one visited (order: On, In, Sc).
- `UpdateExistingPaveBlocks` (`:3278-...`) removes the old PBs from the pool for every CB member, then rebuilds
  per-member pave blocks, computing the parameters of foreign vertices by `ComputeVE` projection onto that
  member's original edge (`:3380-3387`) and preserving closedness explicitly (`:3357-3365`).

### 2.16 `UpdatePaveBlocks` / `RemovePaveBlocks` / `RemoveMicroEdges`

- `UpdatePaveBlocks(aDMNewSD)` (`:3679-3811`) collects **section-curve PBs first, then pool PBs** (`:3695-3727`),
  represents CBs by `PaveBlock1`, fences by PB. Substitution keeps the original **parameters** and only replaces
  the vertex index (`:3749-3769`).
- Micro detection fires only for `wasRegularEdge && !isDegEdge && nV[0]==nV[1]` (`:3781`); the edge index used is
  `aPB->Edge()`, falling back to `OriginalEdge()` when unset (`:3773-3779`).
- **`RemovePaveBlocks` matches on `aPB->Edge()` only** (`:3831`, `:3860`, `:3895`). A micro PB registered under its
  *original* edge index (because `Edge() == -1`) is therefore never removed — a real behavioural corner.
  Removal covers three places: the pool, all FF curve PB lists, and all three FaceInfo maps.
- `RemoveMicroEdges` (`:4388-4435`) only looks at edges with ≥2 pave blocks, skips degenerate edges, uses
  `RealPaveBlock` + fence, and only tests PBs whose two vertex indices are equal.
- `RemoveMicroSectionEdges` (`:4308-4384`) runs **before** `PostTreatFF`, uses
  `BOPTools_AlgoTools::IsMicroEdge(E, ctx, /*bCheckSplittable=*/false)`, skips entries that already have an edge
  (adopted PBs), removes the PB from the curve and hands it to `theMicroPB`.

### 2.17 Exit-state helpers

- `ReleasePaveBlocks` (`%DS%/BOPDS_DS.cxx:1503-1543`): only lists of **exactly one** PB are considered
  (`:1517-1520`); CB members are skipped; both vertices original ⇒ `SetReference(-1)` **and** `Clear()`. The
  "deleted" marker is therefore *reference kept + empty list* (an edge whose `Update` produced nothing), while
  *reference cleared* means "untouched, reuse the original edge".
- `RefineFaceInfoOn` (`:975-991`) first **rebuilds** On from the current pave blocks (`UpdateFaceInfoOn(index)`)
  and only then drops PBs without an edge — it is not a pure filter.
- `RefineFaceInfoIn` (`:995-1024`) iterates source shapes only, requires `HasReference`, and skips faces whose In
  or On map is empty.
- `SharedEdges` (`:1147-1208`): an edge with an **empty** pave-block list contributes its **own index**
  (`:1165-1168`, `:1189-1194`); otherwise it contributes `RealPaveBlock(pb)->Edge()`. The resulting list can mix
  original-edge and split-edge indices.
- `AloneVertices` (`:1028-1062`) unions In+Sc pave-block endpoints, then reports In+Sc vertices not in that set,
  skipping negative indices.
- `CheckSelfInterference` (`PF_11.cxx:28-...`) is skipped entirely in single-argument mode (`:30-34`) and only
  reports warnings.

### 2.18 `ProcessDE` (`PF_8.cxx:54-393`)

DE vertex = `SubShapes().First()` then SD-mapped (`:73-77`). `FindPaveBlocks` scans **In, On and Sc**
(`:155-158`). `aTolInt = max(PConfusion, max(UResolution(tolV), VResolution(tolV)))` (`:243-249`); the DE's
direction is decided by `|Y(t1) - Y(t2)| < PConfusion` (`:264-265`) and
`aTolCmp = max(PConfusion, bUDir ? aURes : aVRes)` (`:267`). Non-line 2D curves are loaded with their range,
lines without (`:293-301`). Intersection failure falls back to a 2D projection of the PB endpoint (`:317-328`).
`AddSplitPoint` requires `aT - aTD1 >= theTol && aTD2 - aT >= theTol` and no duplicate parameter within `theTol`,
then `AppendExtPave1` (`:368-392`). `MakeSplitEdge` (`:163-214`) splits only when
`IsNewShape(nV1) || aNbPB > 1`; otherwise it **clears the reference and the PB list** (`:207-212`).
`MakeSplitEdge1` uses a hard `aTol = 1.e-7`, `BB.Range(E, aF, p1, p2)` and `BB.Degenerated(E, true)` (`:335-358`).

### 2.19 `MakePCurves` (`PF_7.cxx:589-801`)

Whole function returns early when `myAvoidBuildPCurve || (!PCurveOnS1 && !PCurveOnS2)` (`:592`) — the default
`BOPAlgo_SectionAttribute` has all three flags true, only `BOPAlgo_CheckerSI` sets `SetAvoidBuildPCurve(true)`.
In PBs assert `nE >= 0` ("Face information is not up to date", `:625`). For On PBs already carrying a pcurve
nothing is done (`:641-645`); for CB members the *mate* PB's original edge and its pave range/vertices are handed
to the worker as `aEz` (`:649-695`) so the pcurve is copied rather than recomputed. Section-edge pcurves are only
re-touched through `anEFPairs` deduplication with `SetFlag(true)` (`:744-753`). `UpdateVertices` (`:808-846`)
grows a vertex by `sqrt(d²) + DTolerance()` when the 3D and 2D endpoints disagree.

### 2.20 Verified constant table (values, not names)

`Precision::Confusion()=1e-7`, `PConfusion()=1e-9`, `Intersection()=1e-9`, `Angular()=1e-12`,
`BOPTools_AlgoTools::DTolerance()=1e-12`.

| where | value |
|---|---|
| `%DS%/BOPDS_DS.cxx:57` | lines-array increment 500 |
| `%DS%/BOPDS_DS.cxx:312` | `addTol = max(fuzz, 1e-7) * 0.5` |
| `%DS%/BOPDS_DS.cxx:1318` | CB coincidence tol `maxTolV(E1)+maxTolV(E2)+max(fuzz,1e-7)` |
| `%DS%/BOPDS_DS.cxx:1567,1572,1579` | `eps = tolE*0.01`; invalid if `tolV + 1e-7 - dist > eps` |
| `PF_1.cxx:184` | SD vertex box gap `tolV + 1e-7` |
| `PF_3.cxx:382,387-403` | on-pave decision tol `1e-7` |
| `PF_3.cxx:432` | real-intersection test `> 1e-9` ⇒ reject |
| `PF_3.cxx:497` | duplicate-vertex reject `d² < 100·(tolVnew+tolVx)²` |
| `PF_3.cxx:607,639` | EE/EF new vertex box gap `tol + fuzz/2` |
| `PF_3.cxx:822` | shrunk box gap `+= fuzz/2` |
| `PF_3.cxx:950` | PB bbox curve tol `tolE + 1e-7` |
| `PF_3.cxx:1116-1118,1200` | ForceEE add-fuzz `2·max(tolV)`; angle gate `|cos| < 0.9063` |
| `PF_5.cxx:416` | EF on-pave `aTolToDecide = 5.e-8` |
| `PF_5.cxx:490-494` | vertex-cover cap `1e4·tolV`, clamped to `0.1` when `tolV < 0.01` |
| `PF_5.cxx:1022-1026,1047` | ForceEF `2·max(tolV)+fuzz`; angle gate `|cos| > 0.4226` |
| `PF_6.cxx:329` | SSI approx tolerance `1.e-7` |
| `PF_6.cxx:3939` | `ToleranceFF` free-form floor `5.e-6` |
| `PF_6.cxx:495` | `aTolFF = max(seam-shift, ToleranceFF)` |
| `PF_6.cxx:585-587,605` | curve box enlarge `aTolFF + max vertex tol of both faces` |
| `PF_6.cxx:750` | MakeBlocks-local `aTolFF = max(tolF1,tolF2)` |
| `PF_6.cxx:906` | block span reject `< 1e-9` |
| `PF_6.cxx:2110-2112` | `aMaxTolAdd = min(0.001, 10·(tolR3D+fuzz))` |
| `PF_6.cxx:2154,2176,2188,2191` | CB bonus `×2`; probe tol `2·min(...)`; tangency `|cos| >= 0.9063`; `aCoeff=2` |
| `PF_6.cxx:2485,2515,2533` | filter `sin < 0.5`, `d² > 100·max(tol², minD²)`, kept tol `sqrt(maxD²)+1e-7` |
| `PF_6.cxx:2793-2794` | stick `aDT2 = 2e-7` (squared), crease `aDScPr = 5.e-9` |
| `PF_6.cxx:4225` | tolerance-reduction skip `tolV - maxTol < 0.001·tolV` |
| `%BA%/BOPAlgo_Tools.cxx:271,278` | `ComputeToleranceOfCB`: 11 samples, `dt = (t2-t1)/12` |
| `PF_8.cxx:343` | degenerate split edge tolerance `1.e-7` |
| `PF:464-476` | PI weights VV=1,VE=2,EE=5,VF=5,EF=10,FF=30(glue 1), split=EE, blocks=5·FF(glue 0), repeat=0.2(VV+VE+VF), forceEE=2·EE, forceEF=2·EF, pcurves=0.2(EE+EF), DE=0.1·EE |

---

## 3. PORTING TRAPS

1. **Iterator pair orientation is inverted from the spec (E1).** If you order pairs "higher dimension first" every
   `Value()` consumer reads (edge,vertex) instead of (vertex,edge). Cite `%DS%/BOPDS_Iterator.cxx:238-242`.

2. **The pair-selection loop is a merge-walk, not a filter.** `%DS%/BOPDS_Iterator.cxx:307-357` advances a single
   `iPair` cursor across ranges and `break`s out of the inner loop the moment `ID1` leaves the current range. It is
   only correct because `aPairSelector.Sort()` (`:298`) orders pairs by `ID1`. Re-implementing it as an unordered
   "skip same-rank pairs" filter changes which pairs survive when a shape's box overlaps several ranges.

3. **`BOPDS_Pair` has ordered `<` but unordered `==`/hash** (`%DS%/BOPDS_Pair.hxx:60-75`, `:82-100`). Used as a map
   key it is unordered (weld fences, `myDistances`, `ForceInterfEE` grouping); used in `stable_sort` it is ordered.
   Implementing one semantics for both silently changes the intersection order.

4. **`BOPDS_Pave` `<` compares parameter only, `==` compares (index, parameter)** — the spec says this, but the
   consequence it does not spell out is in `%DS%/BOPDS_DS.cxx:1363-1379`: `Paves()` de-duplicates with a `Map`
   (hash on both fields) and *then* sorts with `<`, and asserts `nbPB + 1 == nbUniquePaves`. A port that
   de-duplicates by parameter loses the two seam paves of a closed edge; one that de-duplicates by vertex loses the
   legitimate "same vertex, two parameters" case.

5. **A pave block with one pave produces zero blocks and clears its extras** (`%DS%/BOPDS_PaveBlock.cxx:263-268`).
   This is not an error path; it is the normal way an edge acquires an *empty* pave-block list, which downstream is
   read as "deleted". Returning the parent block unchanged instead breaks `ReleasePaveBlocks`, `SharedEdges` and
   `PostTreatFF`'s micro-edge test, all of which key off `Extent()==0`.

6. **`nV1 == nV2` sub-blocks are dropped without welding** (`PF_2.cxx:473-477`). The spec's rule "no valid range ⇒
   weld the endpoints" would create a self-SD link; `AddShapeSD` refuses self-links anyway
   (`%DS%/BOPDS_DS.cxx:1221`), so a port that welds here loses the block *and* records nothing.

7. **EE coincidence needs identical bounding vertices (`HasSameBounds`) before it can create a CommonBlock**
   (`PF_3.cxx:536-540`); EF coincidence needs both endpoints already registered with the face (`CheckFacePaves`,
   `PF_5.cxx:553-559`). Skipping these gates produces CommonBlocks over misaligned partitions and the shared split
   edges will not close. The recovery for genuinely-coincident-but-misaligned pairs happens later in
   `ForceInterfEE`/`ForceInterfEF`, after vertices have been unified — the two-phase structure is essential.

8. **`ForceInterf*` are not "retry with bigger tolerance".** They are *coincidence-only* passes over pairs that
   already share both bounding vertices, with an angle gate (`0.9063` for EE, `0.4226` for EF) that switches the
   extra tolerance off for non-tangential pairs, and they accept only a single `TopAbs_EDGE` common part.
   Implementing them as a plain tolerance bump unifies edges that merely cross.

9. **`ComputeToleranceOfCB` samples the *representative's* range, not the mates'** (`%BA%/BOPAlgo_Tools.cxx:277-278`)
   and adds the *mate's* tolerance to each measured distance (`:316`, `:345`). It also returns
   `Tolerance(originalEdge)` unchanged when the CB has <2 PBs and no faces (`:266-269`) — so a CB(PB, face) with one
   PB *does* get face-based sampling, while a lone-PB CB with no faces does not.

10. **CB tolerance must be pushed into the bounding vertices immediately** (`UpdateVerticesOfCB`, `PF_3.cxx:959-993`),
    otherwise `myIncreasedSS` never fills and `RepeatIntersection` (`PF:359-424`) becomes a no-op — the entire
    tolerance-feedback loop the spec's item 9 asks for silently disappears.

11. **`UpdateVertex` has two completely different behaviours** (`PF_10.cxx:105-162`). Destructive (or already-new or
    already-aliased): mutate in place, return the same index. Non-destructive on an original vertex: **create a new
    vertex, add an SD link, and add it to `myVertsToAvoidExtension`**, returning a *different* index. Every caller
    must use the returned index (`PF_2.cxx:338`, `PF_3.cxx:883`, `PF_5.cxx:657`, `PF_6.cxx:3591`). Ignoring the
    return value writes paves that point at the pre-alias vertex.

12. **`myIncreasedSS` is keyed on the index passed in, not on the index modified** (`PF_10.cxx:124`, `:158`), which
    is why `RepeatIntersection` checks both `i` and its SD image (`PF:373-389`).

13. **The rollback of unused tolerance extensions rebuilds the bounding box from scratch** (`PF_6.cxx:1085-1089`).
    Restoring only the tolerance, or only the gap, leaves an inflated box that keeps selecting spurious pairs in
    every later BVH query.

14. **The curve's proto pave block is a list element.** `InitPaveBlock1` inserts it, `Update(aLPB,false)` reads its
    ext-paves, and `aLPBC.RemoveFirst()` (`PF_6.cxx:1065`) evicts it. Everything that later iterates
    `BOPDS_Curve::PaveBlocks()` (UpdateFaceInfo, UpdatePaveBlocks, PutSEInOtherFaces, MakePCurves) assumes it is gone.

15. **`aTolFF` is two different numbers.** SSI uses `max(shiftValue, ToleranceFF(...))` with the 5e-6 free-form
    floor (`PF_6.cxx:495`); `MakeBlocks` recomputes `max(tolF1,tolF2)` for vertex existence tests (`:750`); the
    per-block geometric tolerance is `max(curve.Tolerance(), curve.TangentialTolerance())` (`:886`). Collapsing
    these into one number changes both which vertices are reused and which blocks survive `IsValidBlockForFaces`.

16. **The recheck queue is triggered by "no sub-blocks", not by "no kept blocks"** (`PF_6.cxx:894-897`). A curve that
    produced ten candidate blocks all of which were rejected does *not* re-queue its FF pair.

17. **`PostTreatFF` relies on the same-rank rule to avoid intersecting already-built section edges with each other**
    (`PF_6.cxx:1280-1316`): they are wrapped in a single compound so they share one `BOPDS_IndexRange`. A port that
    feeds them as separate arguments will re-split every adopted edge.

18. **`RemovePaveBlocks` keys on `PaveBlock::Edge()`** (`PF_6.cxx:3831`), while `UpdatePaveBlocks` may have collected
    the *original* edge index for a PB whose `Edge()` is still `-1` (`:3773-3779`). Reproduce this literally or you
    will remove pave blocks OCCT keeps (and vice versa).

19. **`RefineFaceInfoOn` rebuilds before it filters** (`%DS%/BOPDS_DS.cxx:975-991`). Implementing it as a pure
    "drop PBs without an edge" pass leaves the On map stale relative to post-`MakeBlocks` splits.

20. **`UpdateFaceInfo`'s final rebuild uses one fence across On, In and Sc** (`PF_6.cxx:1908-1944`), visited in that
    order. A representative PB that qualified for two maps ends up in exactly one. This is what makes the spec's
    "In ∩ On = ∅" invariant hold *without* a second `RefineFaceInfoIn` call.

21. **`SetRealPaveBlock` breaks the min-index representative** (`%DS%/BOPDS_CommonBlock.cxx:114-126`). If your port
    recomputes `PaveBlock1()` as `min(originalEdge)` on demand, the unsplit-reuse path in `MakeSplitEdges`
    (`PF_7.cxx:450-454`) silently reverts and you get a rebuilt copy instead of the original edge — losing exact
    identity with the input topology.

22. **`prepareSolids` only runs for single-argument input** (`%DS%/BOPDS_DS.cxx:1789-1792`). Do not port
    `BuildBndBoxSolid`'s `SetWhole()` behaviour into the two-operand path expecting it to matter; it does not run
    there, and solids are not in the iterator BVH at all (`BOPDS_Tools::HasBRep` covers V/E/F only).

23. **`Prepare` mutates the input edges** (adds pcurves on planar faces, `PF_7.cxx:917-931`) and is therefore skipped
    entirely in non-destructive mode (`:852-856`). Any port that builds pcurves lazily inside the parallel EF/FF
    stages will race exactly where OCCT decided not to.

24. **`MakeSDVertices` mutates a shared `BRep_TVertex` in place** (`PF_1.cxx:167-171`) — point *and* tolerance. All
    handles to the old SD vertex see the new geometry instantly; that is the mechanism by which "prior references
    stay valid". A port using value semantics must re-key every prior reference explicitly.

25. **`UpdatePaveBlocksWithSDVertices` is called after VE, EE, VF, EF and MakeSplitEdges (`PF:266,273,280,287,328`)
    but *not* after VV and *not* after FF.** After VV it is unnecessary because `InitPaveBlocks` SD-maps at
    creation (`%DS%/BOPDS_DS.cxx:465`); after FF the re-keying is done by `UpdatePaveBlocks(aDMNewSD)` instead
    (`PF_6.cxx:1131`), which additionally *re-splits* the edges. Inserting a plain re-key after FF would leave
    edges whose geometry no longer matches their paves.
