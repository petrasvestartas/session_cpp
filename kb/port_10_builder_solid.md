# port_10 — SHELL AND SOLID ASSEMBLY + BOOLEAN SELECTION

**Status**: build document. Everything below is grounded in the OCCT checkout at
`/home/petras/code/code_cpp/OCCT` (8.0.1.dev). Every algorithmic claim carries `file:line`.
Path shorthands used throughout:

| shorthand | real path |
|---|---|
| `BOPAlgo/` | `src/ModelingAlgorithms/TKBO/BOPAlgo/` |
| `BOPTools/` | `src/ModelingAlgorithms/TKBO/BOPTools/` |
| `IntTools/` | `src/ModelingAlgorithms/TKBO/IntTools/` |
| `BRepClass3d/` | `src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/` |
| `TopoDS/`, `TopTools/`, `BRep/` | `src/ModelingData/TKBRep/...` |
| `Precision/` | `src/FoundationClasses/TKernel/Precision/` |

Our source citations are relative to `/home/petras/code/code_rust/session/session_cpp/`.

This subsystem is **stage 10** of `kb/ARCHITECTURE_v2.md` ("Build — selection, not
re-classification"). It consumes the split images produced by stage 9 and produces solids.
It assumes, and cannot substitute for, the entity-sharing guarantees of stages 0–9: if two
coincident face pieces are two arena entries instead of one, no amount of assembly logic
recovers. Section 3 states that precondition formally.

---

## 1. WHAT THIS SUBSYSTEM MUST GUARANTEE

Each guarantee is written so it can be asserted in code with no reference oracle. `I-n` tags
are referenced by the acceptance tests in §5.

**I-1 — Adjacency is entity identity, never proximity.**
Two oriented face images are adjacent iff they reference the *same edge index in the shared
arena*. No distance, no tolerance, no Hausdorff comparison participates in shell grouping.
Testable: run the whole assembly with every 3D curve replaced by a nonsense curve of the
same topology; the shell decomposition must be bit-identical.

**I-2 — Every input face image lands in exactly one shell, or is reported.**
Partition of the input face-image multiset over shells ∪ internal-shells ∪ *reported unused*.
No image is silently dropped and none is used twice within one shell. Testable: multiset sum.

**I-3 — A shell declared closed has even edge parity.**
For every non-degenerate, non-INTERNAL, non-EXTERNAL edge referenced by the shell, the number
of trims of that edge belonging to faces of the shell is even (OCCT uses exactly this test —
`BRep/BRep_Tool.cxx:1707-1729`). "2-trim everywhere" is a *stronger* condition we also want
but is not what closure means in the presence of seams; see §2.6.

**I-4 — Relative orientation inside a shell is consistent.**
For every edge shared by exactly two faces of a shell (excluding seams, where one face uses
the edge twice), the two faces traverse that edge with *opposite* orientation.
Testable per-edge, no geometry.

**I-5 — Outer/cavity status of a shell is decided by a point-in-solid classification of the
shell alone, independent of every other shell.**
`IsHole(shell) := classify(infinite point, solid(shell)) == IN`
(`BOPAlgo/BOPAlgo_BuilderSolid.cxx:823-831`). A shell must never need to see another shell's
geometry to know whether it is a cavity. Testable: classify each shell in isolation, compare
with classification in the assembled model; must agree.

**I-6 — Nesting picks the innermost containing growth solid.**
If cavity shell H is inside growth solids S1 ⊃ S2, H becomes a cavity of S2
(`BuilderSolid.cxx:517-528`). Testable on nested-box towers with analytically known volumes.

**I-7 — A cavity shell inside no growth solid still becomes a solid.**
It is not discarded (`BuilderSolid.cxx:578-597`). Its bounding box is set to *whole*.

**I-8 — The number of result solids is the number of connected components of the kept
oriented-face set under I-1 adjacency, after closure and nesting. It is not clamped, not
compared to an expected count, and not "repaired" toward 1.**
Two closed lumps are two solids; that is the answer, not a symptom.

**I-9 — Selection uses each face image exactly once per result, and the three
partition results together consume each image exactly once.**
For operands A,B: every image of A is used exactly once across `{A\B, A∩B}`; every image of
B exactly once across `{A∩B, B\A}`; same-domain walls are the single exception and are
governed by the explicit SD rows of §2.10. This is the invariant that catches the
double-booking defect noted in the mission brief (a lump omitted from `A\B` while also
present in `A∩B` passes a volume-sum identity but fails this).

**I-10 — Selection is a lookup, not a re-classification.**
The IN/OUT state of a face image w.r.t. the *other* operand's solids is read from a map
computed once, in the split stage (`myInParts`, `BOPAlgo/BOPAlgo_Builder_3.cxx:243-261`).
The selection step performs zero point classifications.

**I-11 — Same-domain decisions require two distinct identities.**
An *unoriented* identity (arena face index) and an *oriented* identity (index + orientation).
Collapsing them into one collapses the ON-same and ON-opposite rows and loses the CUT wall
(`BOPAlgo/BOPAlgo_Builder.cxx:682` vs `:696`/`:714`; hasher semantics
`TopTools/TopTools_ShapeMapHasher.hxx:34-37` vs `TopoDS/TopoDS_Shape.hxx:276,282`).

**I-12 — Provenance is total.**
For every input face/edge/vertex of either operand, the result records exactly one of:
Modified→{list of surviving images}, Generated→{list of new entities}, Deleted. No input is
unaccounted for (`BOPAlgo/BOPAlgo_Builder_4.cxx:187-251`).

**I-13 — Failure is loud.**
Every give-up path (§2.13) emits a typed diagnostic carrying the offending shapes. A result
built along a fallback path is distinguishable from one built along the main path.

---

## 2. OCCT'S ALGORITHM

### 2.0 Where this sits

`BOPAlgo_BOP::PerformInternal1` (`BOPAlgo/BOPAlgo_BOP.cxx:422-579`) runs, in order:
FillImagesVertices → BuildResult(VERTEX) → FillImagesEdges → BuildResult(EDGE) →
FillImagesContainers(WIRE) → FillImagesFaces → BuildResult(FACE) →
FillImagesContainers(SHELL) → **FillImagesSolids** (`:525`) → BuildResult(SOLID) →
FillImagesContainers(COMPSOLID) → FillImagesCompounds → **BuildShape** (`:564`) →
**PrepareHistory** (`:571`) → PostTreat (`:578`).

`FillImagesSolids` (`BOPAlgo/BOPAlgo_Builder_3.cxx:60-93`) is:
`FillIn3DParts` (`:80`) → `BuildSplitSolids` (`:86`) → `FillInternalShapes` (`:92`).
`BuildSplitSolids` is what runs `BOPAlgo_BuilderSolid` per source solid.

`BuildShape` (`BOP.cxx:871-1093`) then selects. There are **three** distinct selection
paths, and a port must know which is which:

| path | when | file:line |
|---|---|---|
| `BuildBOP` (state-based, faces → BuilderSolid) | 3D∩3D **and** any argument solid is not closed | `BOP.cxx:875-897`, impl `BOPAlgo_Builder.cxx:479-885` |
| `BuildRC` + `BuildSolid` | FUSE with `myDims[0]==3` | `BOP.cxx:900-906`, `BOP.cxx:1097-1378` |
| `BuildRC` + container rebuild | everything else (COMMON/CUT/CUT21 on closed solids, and all lower-dimensional ops) | `BOP.cxx:900`, `:914-1092` |

`BuildRC` (`BOP.cxx:583-867`) selects **whole split solids** by set membership; `BuildBOP`
selects **faces** by IN/OUT state and re-runs `BuilderSolid`. §2.10 specifies `BuildBOP`,
because it is the general rule; §2.11 specifies `BuildRC` because it is what runs in the
common closed-solid case and because its cell logic is the cheap path we want as a
fast route.

### 2.1 Input contract of `BOPAlgo_BuilderSolid`

Set by `SetShapes(list of TopoDS_Shape)` (`BOPAlgo/BOPAlgo_BuilderArea.hxx:46`). The list is
a list of **oriented faces**; the same face may appear twice with opposite orientations.
Class doc, `BOPAlgo/BOPAlgo_BuilderSolid.hxx:32-51`: *"The given faces should be
non-intersecting, i.e. all coinciding parts of the faces should be shared among them."*
That sentence is the entire precondition and is exactly guarantee I-1.

Who fills it, in the split path (`BOPAlgo/BOPAlgo_Builder_3.cxx:491-518`):
* `aSFS` = every face of the **draft solid** `aSD` (`:494-499`) — i.e. the source solid's
  shells with their faces replaced by images, INTERNAL faces removed (§2.2);
* plus every face in `myInParts[solid]` inserted **twice**, once FORWARD and once REVERSED
  (`:502-511`). This doubling is load-bearing: an interior wall can bound material on either
  side, so both orientations must be available and the shell walk picks exactly one per side.

`BuilderSolid::Perform` (`BuilderSolid.cxx:76-125`):
```
if myShapes empty            -> return                                     (:82-85)
if myContext null            -> myContext = new IntTools_Context           (:87-90)
myBoxes.Clear()                                                            (:92)
PerformShapesToAvoid( 1% of progress)                                      (:106)
PerformLoops        (10%)                                                  (:112)
PerformAreas        (80%)                                                  (:118)
PerformInternalShapes(9%)                                                  (:124)
```
Note `myShapesToAvoid` is `NCollection_IndexedMap<TopoDS_Shape>` — **default (oriented)
hasher** (`BOPAlgo/BOPAlgo_BuilderArea.hxx:83`). `F` and `F.Reversed()` are distinct entries.

### 2.2 `BuildDraftSolid` — what the shells look like before splitting

`BOPAlgo/BOPAlgo_Builder_3.cxx:267-368`. Per source solid:
* copy the solid's orientation onto the draft (`:280-281`);
* for each direct child that is a SHELL (`:286-290`, non-shells — internal edges/vertices —
  are skipped), make a new shell with the *same stored orientation* (`:292-294`);
* for each face `aF` of that shell with stored orientation `aOrF`:
  * if `myImages` has splits of `aF`: for each split `aFx`
    * **if `myShapesSD.IsBound(aFx)`** (the split is same-domain with a face of the other
      operand, so the surviving representative may have the *other* operand's surface and
      hence the other normal sense):
      * `aOrF == INTERNAL` → `aFx.Orientation(INTERNAL)`, push to `theLIF` (`:314-318`);
      * else run the geometric test
        `BOPTools_AlgoTools::IsSplitToReverseWithWarn(aFx, aF, ...)` and reverse if true
        (`:321-327`), then add to the draft shell.
    * **else** (ordinary split): `aFx.Orientation(aOrF)` verbatim — **no geometric test at
      all** (`:334`). INTERNAL → `theLIF`, otherwise into the shell (`:335-343`).
  * if `aF` has no images: same INTERNAL/else split, face added as-is (`:348-359`).
* the shell is added to the draft solid only if it received at least one non-INTERNAL face
  (`iFlag`, `:362-366`), and `aShD.Closed(BRep_Tool::IsClosed(aShD))` is recorded (`:364`).

Two consequences a port must reproduce:
1. **Ordinary splits inherit the parent's orientation with no test.** The audit
   (`kb/audit_occt_builder-assembly.md` §1 E1) confirms the earlier spec was wrong here.
2. The reason 1 is safe is the surface-identity fast path of `IsSplitToReverse(Face,Face)`
   (`BOPTools/BOPTools_AlgoTools.cxx:1336-1341`): if the two faces share the *same*
   `Geom_Surface` handle, the answer is just `orientation1 != orientation2`. If your splitter
   rebuilds surfaces per split, that fast path never fires; see §2.13 trap T6.

### 2.3 Stage A — `PerformShapesToAvoid`

`BuilderSolid.cxx:129-219`. Removes faces that cannot participate in a closed shell, and
iterates because removal creates new free edges.

```
myShapesToAvoid.Clear()                                                    (:138)
for (;;) {                                                                 (:142)
    bFound = false
    aMEF.Clear()
    for each face F in myShapes not in myShapesToAvoid:                    (:152-160)
        MapShapesAndAncestors(F, EDGE, FACE, aMEF)     // edge key IsSame-hashed
    for i in 1..aMEF.Extent():                                             (:164)
        E = aMEF.FindKey(i)
        if Degenerated(E)                     continue                     (:167-170)
        LF = aMEF.ChangeFromKey(E); nF = LF.Extent()
        if nF == 0                            continue                     (:174-177)
        aOrE = E.Orientation()                                             (:179)
        F1 = LF.First()
        if nF == 1:                                                        (:182)
            if aOrE == INTERNAL                continue                    (:184-187)
            bFound = true; myShapesToAvoid.Add(F1)                         (:188-189)
        else if nF == 2:                                                   (:191)
            F2 = LF.Last()
            if F2.IsSame(F1):                  // same face, both orientations
                if BRep_Tool::IsClosed(E, F1)  continue   // seam edge      (:196-199)
                if aOrE == INTERNAL            continue                    (:201-204)
                bFound = true
                myShapesToAvoid.Add(F1); myShapesToAvoid.Add(F2)           (:206-208)
    if (!bFound) break                                                     (:213-216)
}
```
`myShapesToAvoid` is *oriented*, so `Add(F1)` and `Add(F2)` at `:207-208` insert the two
orientations separately. `LF` holds oriented faces; `IsSame` compares ignoring orientation.
So the `nF==2` case fires exactly on **a face that is alone on that edge but was supplied
twice with opposite orientations** — the class doc's "alone faces given twice with different
orientation" (`BuilderSolid.hxx:38-39`).

### 2.4 Stage B — `PerformLoops`: shells by shared-edge connexity

`BuilderSolid.cxx:223-393`.

**B.1 infinite faces bypass everything.** `myContext->IsInfiniteFace(F)`
(`IntTools/IntTools_Context.cxx:216-221`: the face's bounding box is open in any of the 6
directions) → wrap that single face in its own shell and append directly to `myLoops`
(`BuilderSolid.cxx:242-251`). Never enters the splitter.

**B.2 the rest go to `BOPAlgo_ShellSplitter`** (`:253-260`).

**B.3 `ShellSplitter::Perform`** (`BOPAlgo/BOPAlgo_ShellSplitter.cxx:137-149`):
```
BOPTools_AlgoTools::MakeConnexityBlocks(myStartShapes, EDGE, FACE, myLCB)   (:142)
MakeShells(...)                                                             (:148)
```

**B.4 `MakeConnexityBlocks`** (`BOPTools/BOPTools_AlgoTools.cxx:187-256`, delegating to
`:105-154`). This is the **grouping by shared-edge connexity** and it is purely topological:
* start elements are put into a compound; a *duplicate* start element (same face already
  seen, `IsSame`) is recorded in `aMNRegular` (`:203-210`);
* `MakeConnexityBlocks(compound, EDGE, FACE, blocks, connMap)` (`:105-154`) —
  `TopExp::MapShapesAndAncestors(S, EDGE, FACE, connMap)` (`:114`) then a BFS over faces:
  for each unvisited face, explore its EDGEs, and for each edge pull `connMap[edge]` and add
  every unvisited ancestor face (`:131-150`). **Nothing but map lookup by edge identity.**
* per block, `bRegular` starts true and is falsified by (a) a duplicated start element
  (`:231-238`, which additionally pushes the face into the block twice, FORWARD and
  REVERSED), or (b) any edge of any block face whose ancestor list size ≠ 2 (`:245-248`).

**B.5 `MakeShells`** (`ShellSplitter.cxx:621-679`):
* **regular block** → `MakeShell(faces, shell)` (`:683-698`) which simply adds all faces then
  calls `BOPTools_AlgoTools::OrientFacesOnShell(shell)`; `shell.Closed(true)` is asserted
  unconditionally (`:647`).
* **non-regular block** → queued for `SplitBlock`, run in parallel (`:651-654`, `:664`); the
  resulting loops are appended with `Closed(true)` (`:666-678`).

**B.6 `SplitBlock`** (`ShellSplitter.cxx:153-421`) — the non-manifold walk:

*Free-edge peeling* (`:185-222`): repeat { rebuild `aEFMap` over surviving faces; for each
non-degenerate, non-INTERNAL edge whose ancestor list has exactly 1 face, remove that face }
until the surviving count stops changing or reaches 0. If nothing survives → **return with no
shells at all** (`:224-227`).

*Boundary faces* (`:230-245`): a face is a "boundary face" iff it appears **once** in the
block's face list (`aBoundaryFaces.Add` returning false → `Remove`, `:240-243`). Faces
supplied twice (both orientations) are therefore *not* boundary faces.

*The walk* (`:251-369`). For each unprocessed face `aFF`:
```
new shell, add aFF                                                          (:261-263)
aMEFP = edge->faces map of THIS shell only                                  (:265-266)
iterate faces of the growing shell (TopoDS_Iterator over the shell):        (:270)
  isBoundary = aBoundaryFaces.Contains(aF)                                  (:274)
  for each edge aE of aF:                                                   (:277)
     if aMEFP already lists >1 face for aE  -> continue  // edge closed in this shell (:283-291)
     if aE.Orientation() == INTERNAL        -> continue                     (:292-297)
     if Degenerated(aE)                     -> continue                     (:298-302)
     candidates = aEFMap[aE]   (all faces of the block on this edge)        (:305)
     aLCSOff = {}; aNbWaysInside = 0
     for each candidate aFL:                                                (:319)
        if aFL.IsSame(aF) or already added   -> continue                    (:322-325)
        if !GetEdgeOff(aE, aFL, aEL)         -> continue                    (:328-331)
        if isBoundary && !aBoundaryFaces.Contains(aFL):
              ++aNbWaysInside; aSelF = aFL                                  (:333-337)
        aLCSOff.Append({aEL, aFL})                                          (:338-340)
     if aLCSOff empty -> continue                                           (:343-347)
     if !isBoundary || aNbWaysInside != 1:                                  (:351)
        if 1 candidate  -> aSelF = that one                                 (:353-356)
        else            -> GetFaceOff(aE, aF, aLCSOff, aSelF, ctx)          (:359)
     if aSelF valid and newly added:                                        (:363)
        add to shell; extend aMEFP                                          (:365-366)
RefineShell(shell, aMEFP, aLShSp)                                           (:373)
for each piece: closed -> myLoops; not closed -> aLShNC                     (:378-392)
if all faces used -> break                                                  (:394-398)
if the shell produced exactly one piece -> continue (no point retrying)      (:400-405)
else release the faces of the non-closed pieces back to the pool            (:411-419)
```

`GetEdgeOff(E1, F2, E2)` (`BOPTools_AlgoTools.cxx:1107-1135`) finds, among the edges of `F2`
that are `IsSame` with `E1`, the one whose orientation is exactly `Reverse(E1.Orientation())`.
This is a pure topological filter: a candidate face qualifies only if it traverses the shared
edge the opposite way — guarantee I-4 enforced at construction time.

**B.7 `RefineShell`** (`ShellSplitter.cxx:443-617`) splits a walked shell at *branch edges*.
`aMEStop` collects edges where:
* the shell's own edge→face map lists **> 2** faces (`:464-468`); or
* it lists exactly 2 faces and both traverse the edge with the **same** orientation
  (`:470-483`, via `FindShape(aE, aFi)` at `:425-439` which recovers the oriented occurrence);
  or
* counting each face twice when the edge is INTERNAL in it gives > 2 (`:487-512`).

If `aMEStop` is empty, the shell is emitted whole (`:514-518`). Otherwise a flood fill that
refuses to cross stop edges, INTERNAL edges and degenerate edges (`:554-567`) produces one
sub-shell per component (`:528-616`).

**B.8 `OrientFacesOnShell`** (`BOPTools_AlgoTools.cxx:363-507`) — relative orientation fixing:
* build `aEFMap` (edge → faces) over the shell (`:377`);
* de-duplicate: a seam edge lists the same face twice; collapse each list to distinct faces
  (`:381-403`);
* pass 1 (`:406-479`): for each non-degenerate edge whose distinct-face list has **exactly 2**
  entries `F1,F2`:
  * skip if both already processed (`:426-429`);
  * if neither processed, accept `F1` as-is and mark it processed (`:431-436`);
  * fetch the *processed* copy of each (orientation may already have been flipped)
    (`:438-450`);
  * `anOrE1 = Orientation(E, F1x)`, `anOrE2 = Orientation(E, F2x)` (helper at `:511-527`:
    the orientation of the occurrence of `E` inside that face; `INTERNAL` if absent);
  * if the unprocessed face's edge orientation **equals** the processed one, reverse the
    unprocessed face — but only if the edge is a seam in *neither* face
    (`!IsClosed(aE,aF1) && !IsClosed(aE,aF2)`, `:457-463` / `:469-475`);
  * mark it processed and add to the new shell.
* pass 2 (`:482-505`): faces touched only by edges with ≠2 faces are appended unchanged.
* `aShell = aShellNew` (`:506`).

This fixes **relative** orientation only. **OCCT never globally flips a shell.** The
outer/cavity question is answered later by classification (§2.7), not by reversal. That is a
design decision a port must copy: a globally-inverted shell is *reported as a cavity*, and
whether it belongs to a solid is a containment question.

**B.9 post-treatment and internal shells** (`BuilderSolid.cxx:285-392`):
* collect every face that ended in some loop (`:295-305`);
* add the avoid-set faces (`:312-317`);
* any face of `myShapes` that is neither in a loop nor already avoided is **added to
  `myShapesToAvoid`** (`:320-331`) — i.e. faces the splitter could not use are demoted, not
  dropped;
* build `myLoopsInternal`: BFS over the avoid-set using their own edge→face map, one shell per
  connected group, `aShell.Closed(BRep_Tool::IsClosed(aShell))` (`:338-392`).

### 2.5 `GetFaceOff` — the angular selector (this is the curved-surface core)

`BOPTools/BOPTools_AlgoTools.cxx:994-1103`. Given the reference edge `theE1` (oriented as it
appears in the reference face `theF1`) and a list of (edge-occurrence, candidate-face) pairs,
pick the candidate whose *material side* is angularly closest, walking around the edge.

```
aAngleMin = 100.;  aTwoPI = 2*PI                                            (:1012-1013)
aC3D = Curve(theE1, t1, t2)
aT   = BOPTools_AlgoTools2D::IntermediatePoint(t1, t2)                      (:1015)
     // = (1-PAR_T)*t1 + PAR_T*t2, PAR_T = 0.43213918
     //   (BOPTools_AlgoTools2D.cxx:404-411; identical constant in
     //    IntTools/IntTools_Tools.cxx:254-259). Deliberately NOT the midpoint:
     //   avoids symmetric configurations landing on a symmetry point.
aC3D->D0(aT, aPx)                                                           (:1016)
EdgeTangent(theE1, aT, aVTgt); aDTgt = dir(aVTgt); aOr = theE1.Orientation()(:1018-1020)
aPL = Geom_Plane(aPx, aDTgt)          // plane normal to the edge at aPx     (:1022)
aProjPL.Init(aPL, ...)                                                      (:1023-1024)
aDt3D = MinStep3D(theE1, theF1, theLCSOff, aPx, ctx, bSmallFaces)           (:1027)
GetFaceDir(theE1, theF1, aPx, aT, aDTgt, bSmallFaces, aDN1, aDBF, ...)      (:1028-1029)
aDTF = aDN1 ^ aDBF                    // reference axis for signed angles    (:1038)
anAngleCriteria = Precision::Confusion()  // 1e-7                            (:1042)
bRet = true
for each (aE2, aF2) in theLCSOff:                                            (:1046)
    aDTgt2 = (aE2.Orientation() == aOr) ? aDTgt : aDTgt.Reversed()           (:1052)
    bIsComputed = GetFaceDir(aE2, aF2, aPx, aT, aDTgt2, ..., aDN2, aDBF2,...)(:1053-1054)
    aAngle = AngleWithRef(aDBF, aDBF2, aDTF)                                 (:1063)
    if |aAngle| < Precision::Angular():        // 1e-12                      (:1065)
        if aF2 == theF1  (IsEqual: same orientation)   -> aAngle = PI        (:1067-1070)
        elif aF2.IsSame(theF1) (opposite orientation)  -> aAngle = 2*PI      (:1071-1074)
        elif !bIsComputed                              -> aAngle = 2*PI      (:1075-1082)
    if |aAngle| < anAngleCriteria || ||aAngle| - aAngleMin| < anAngleCriteria:
        bRet = false                   // "the minimal angle cannot be found"(:1085-1089)
    if aAngle < 0: aAngle += 2*PI                                            (:1091-1094)
    if aAngle < aAngleMin: aAngleMin = aAngle; theFOff = aF2                 (:1096-1100)
return bRet
```

`AngleWithRef(D1, D2, DRef)` (`:1946-1975`) is *not* `atan2`; it is a monotone surrogate:
```
sinus   = |D1 x D2|;  cosinus = D1·D2
beta    = (PI/2)*(1 - cosinus)          // since sinus >= 0 always, the else branch is dead
if ((D1 x D2)·DRef) < 0: beta = -beta
```
So `beta ∈ [0, PI]` mapped monotonically from `cosinus ∈ [1,-1]`, signed by the reference
axis. It is *not* the true angle — it is a strictly monotone reparametrisation of it, which
is all the "minimum" needs. A port may substitute `atan2(sin, cos)` **only if** it also
changes `anAngleCriteria` consistently; safest is to copy `AngleWithRef` verbatim.

`GetFaceDir` (`:2118-2160`) computes the in-face *bi-normal* — the direction inside the face,
perpendicular to the edge:
```
GetNormalToFaceOnEdge(aE, aF, aT, aDN, ctx); if aF REVERSED -> aDN.Reverse() (:2133-2137)
aTolE = Tolerance(aE);  aDB = aDN ^ aDTgt                                    (:2139-2140)
bFound = !theSmallFaces && FindPointInFace(aF, aP, aDB, aPx, ..., aDt, aTolE)(:2145-2146)
if !bFound:
    bFound = GetApproxNormalToFaceOnEdge(aE, aF, aT, aDt, aPx, aDN, ctx)     (:2150-2151)
    aProjPL.Perform(aPx); aPx = nearest;  aDB = vector(aP -> aPx)            (:2152-2156)
```

`FindPointInFace` (`:2168-2239`) — the step-into-the-face iteration, and the place where a
curved face differs from a planar one:
```
aDTol = Precision::Angular()                       // 1e-12                  (:2182)
aPM   = |aP|; if aPM > 1000: aDTol = 5.e-16 * aPM  // absolute-coordinate scaling (:2183-2187)
aNbItMax = 15;  anEps = Precision::SquareConfusion()  // 1e-14               (:2189-2190)
aPS = project(aP onto face surface) then project onto the normal plane       (:2194-2202)
aPS += 2*aTolE * aDB                                // clear the edge band   (:2204)
aPS = project onto surface, then onto the normal plane                       (:2205-2212)
do {
    aP1 = aPS + aDt * aDB                          // aDt from MinStep3D
    aPOut = project(aP1 onto surface); aDist = lower distance                (:2218-2224)
    aPOut = project(aPOut onto the normal plane)                             (:2226-2227)
    aV = aPOut - aPS;  if |aV|^2 < anEps -> FAIL                             (:2229-2233)
    aDB = dir(aV)                                                            (:2234)
} while (aDist > aDTol && --aNbItMax);
return aDist < aDTol
```
The loop is a fixed-point iteration that walks a point of the **circle of radius `aDt`
centred at `aPx` in the plane normal to the edge** onto the surface, then back onto the
plane, until the surface distance drops below `aDTol`. On a plane it converges in one step;
on a sphere/cylinder/torus it is the reason `aDt` must be chosen carefully — hence:

`MinStep3D` (`:2243-2354`) — the step size:
```
aTolE = Tolerance(theE1); aDtMax = -1; aDtMin = 5.e-6                        (:2260-2262)
for each candidate face aF (including theF1, appended at :2253-2258):
    aDt = 2*(aTolE + Tolerance(aF)); aDtMax = max(aDtMax, aDt)               (:2270-2275)
    switch (SurfaceAdaptor(aF).GetType()):                                   (:2281)
        Cylinder: aR = Radius                                                (:2283-2286)
        Cone:     aR = distance(aP, cone axis line)                          (:2287-2291)
        Sphere:   aDtMin = max(aDtMin, 5.e-4); aR = Radius                   (:2292-2296)
        Torus:    aR = MajorRadius                                           (:2297-2300)
        default:  aDtMin = max(aDtMin, 5.e-4)      // NURBS / freeform!      (:2301-2303)
    if aR > 100:
        d = 10*Precision::PConfusion()             // 1e-8
        aDtMin = max(aDtMin, sqrt(d*d + 2*d*aR))   // chord for sagitta d    (:2306-2310)
aDtMax = max(aDtMax, aDtMin)                                                 (:2313-2316)
// "too big for any face" test:
for each candidate face aF:                                                  (:2319)
    UVBounds(aF, uMin,uMax,vMin,vMax)                                        (:2328)
    if (uMax-uMin) > 0 and 2*UResolution(aDtMax) > (uMax-uMin) -> break       (:2330-2338)
    if (vMax-vMin) > 0 and 2*VResolution(aDtMax) > (vMax-vMin) -> break       (:2340-2348)
theSmallFaces = (loop broke early)                                           (:2351)
return aDtMax
```
`theSmallFaces == true` disables `FindPointInFace` entirely (`:2145-2146`) and forces the
hatcher path. **Note**: the `default:` arm — every NURBS/freeform surface — gets
`aDtMin = 5e-4` unconditionally. On a model whose features are smaller than 5e-4 the step
overshoots the face and the hatcher fallback is what actually decides. That is a documented
OCCT limitation, not something to "improve" silently: record it and gate on it.

### 2.6 Shell closedness

`BRep/BRep_Tool.cxx:1707-1729`, called as `BRep_Tool::IsClosed(shell)`:
```
map = {}
for each EDGE of shell.Oriented(FORWARD):
    if Degenerated(E) or E.Orientation() in {INTERNAL, EXTERNAL}: continue
    hasBound = true
    if !map.Add(E): map.Remove(E)          // toggle; map is IsSame-keyed
return hasBound && map.IsEmpty()
```
This is **edge parity**, not "every edge has two trims". A seam edge appears twice in one
face and cancels; an edge shared by four faces also cancels. So `IsClosed` is *necessary but
not sufficient* for manifoldness. Guarantee I-3 states the OCCT condition; a port should
additionally assert the stronger 2-trim condition and *report* (not silently accept) any
edge with parity 0 but count 4.

### 2.7 Stage C — `PerformAreas`: growth vs hole, and nesting

`BuilderSolid.cxx:397-598`.

**C.1 Classify every loop shell** (`:412-442`):
```
for each shell in myLoops:
    bIsGrowth = IsGrowthShell(shell, aMHF)                                   (:422)
    if !bIsGrowth: bIsGrowth = !IsHole(shell, myContext)                     (:426)
    if bIsGrowth: make a solid of it, append to aNewSolids                   (:430-436)
    else:         aHoleShells.Add(shell); MapShapes(shell, FACE, aMHF)       (:437-441)
```
`IsGrowthShell(shell, MHF)` (`:864-879`) is a **pure shortcut**: if any face of this shell is
already a face of some previously-found hole shell, the shell is a growth. It exists only to
avoid a classifier call and is a no-op when `MHF` is empty. It is *order dependent* — a
correctness-neutral optimisation only because a face cannot legitimately be in two shells
under I-2.

`IsHole(shell, ctx)` (`:823-831`):
```
BRepClass3d_SolidClassifier& clsf = ctx->SolidClassifier(*(TopoDS_Solid*)&shell);
clsf.PerformInfinitePoint(::RealSmall());        // RealSmall() = DBL_MIN = 2.2250738585072014e-308
return clsf.State() == TopAbs_IN;
```
Yes — the shell is reinterpreted as a solid by pointer cast (legal in OCCT's handle model
because `TopoDS_Solid` and `TopoDS_Shell` are both `TopoDS_Shape` with a type tag; the
classifier only ever explores SHELL children, and `SolidExplorer::InitShell` does
`Init(myShape, TopAbs_SHELL)` (`BRepClass3d/BRepClass3d_SolidExplorer.cxx:1000-1003`), which
finds the shell itself). A port with real types just wraps the shell in a temporary solid.

**C.2 no holes → early exit** (`:444-458`): every new solid is appended to `myAreas` and its
bounding box bound into `myBoxes`.

**C.3 nesting** (`:460-576`):
```
BVH over hole-shell boxes (BOPTools_BoxTree)                                 (:463-478)
for each growth solid S:                                                     (:484)
    box(S) -> myBoxes                                                        (:494-497)
    select candidate holes whose box interferes                              (:499-504)
    for each candidate hole H:                                               (:506)
        if !IsInside(H, S, ctx): continue                                    (:511-514)
        if H already assigned to S_prev:
             if IsInside(S, S_prev): reassign H to S     // keep the INNERMOST(:517-524)
        else assign H -> S                                                   (:525-528)
invert to solid -> [holes]                                                   (:533-548)
for each solid: add its hole shells; reload its cached classifier            (:551-576)
for each hole never assigned: make it a solid on its own,                    (:578-594)
    and bind an INFINITE (SetWhole) box for it                               (:590-593)
myBoxes.UnBind(hole) for every hole                                          (:596)
```
`IsInside(S1, S2, ctx)` (`:835-860`):
```
if S1 has no FACE:                                                           (:844-845)
    clsf = ctx->SolidClassifier(S2); clsf.PerformInfinitePoint(RealSmall());  (:847-848)
    state = clsf.State()
else:
    bounds = all EDGEs of S2                                                 (:853-854)
    F = the FIRST face of S1 (TopExp_Explorer order)                         (:855)
    state = BOPTools_AlgoTools::ComputeState(F, S2, Precision::Confusion(), bounds, ctx) (:856-857)
return state == TopAbs_IN
```
`ComputeState(Face, Solid, tol, bounds, ctx)` (`BOPTools_AlgoTools.cxx:660-715`):
```
for each non-degenerate EDGE E of F:                                         (:672-686)
    if E not in bounds:  return ComputeState(E, ref, tol, ctx)               (:681-685)
// all edges of the face lie on the solid -> need an interior point
iErr = BOPTools_AlgoTools3D::PointInFace(F, aP3D, aP2D, ctx)                 (:692)
if iErr != 0:                                                                (:693)
    for each non-degenerate edge: iErr = PointNearEdge(E, F, aP2D, aP3D, ctx) (:696-707)
if iErr == 0: return ComputeState(aP3D, ref, tol, ctx)                       (:709-712)
return TopAbs_UNKNOWN
```
`ComputeState(Edge, ...)` (`:734-786`) evaluates the curve at
`IntTools_Tools::IntermediatePoint(t1,t2)` (`:778`) — the same 0.43213918 split — with
special cases for semi-infinite (`t = t2 - 10` / `t1 + 10`, `dT=10` at `:759`) and
bi-infinite (`t = 0`) ranges, and falls back to the first vertex for a null curve (`:746-755`).
`ComputeState(Pnt, ...)` (`:790-803`) is the direct `SolidClassifier::Perform(P, tol)`.

`PointInFace(F, ...)` (`BOPTools/BOPTools_AlgoTools3D.cxx:906-938`) shoots a **vertical
2D line** at `u = IntermediatePoint(uMin,uMax)` through the face's 2D hatcher
(`Geom2dHatch_Hatcher`, `:992-1035+`), takes a domain, and picks a `v` inside it; on failure
it retries once with the mirrored abscissa `uMax - (uₓ - uMin)` (`:929-934`).

### 2.8 The point-in-solid classifier and grazing rays

This is the machinery behind every `IsHole`/`IsInside`/`IsInternalFace` call.

`IntTools_Context::SolidClassifier(solid)` (`IntTools/IntTools_Context.cxx:312-323`) caches
one `BRepClass3d_SolidClassifier` per solid, IsSame-keyed.

`BRepClass3d_SolidClassifier::Perform(P, Tol)`
(`BRepClass3d/BRepClass3d_SolidClassifier.cxx:171-211`) forwards straight to
`BRepClass3d_SClassifier::Perform(explorer, P, Tol)` (`:209`) — the `MARCHEPASSIUNESEULEFACE`
box-rejection shortcut is compiled **out** (`:17`, `#define MARCHEPASSIUNESEULEFACE 0`).

**`SolidExplorer::InitShape`** (`BRepClass3d_SolidExplorer.cxx:898-982`):
* one `IntCurvesFace_Intersector(Face, Precision::Confusion(), /*restriction*/true,
  /*UseBoundaries*/false)` per face (`:924`) — this is the exact ray/face intersector: it
  intersects a `gp_Lin` with the **real surface** (not a tessellation) and classifies the hit
  in the face's 2D domain;
* `myParamOnEdge = 0.512345` (`:905`);
* `myMapEV` collects the vertices and edges of **non-INTERNAL, non-EXTERNAL faces**, skipping
  INTERNAL/EXTERNAL and degenerate edges (`:938-967`), and a `NCollection_UBTree<int,Bnd_Box>`
  is filled with their bounding boxes (`:970-981`).

**`SClassifier::Perform(SolidExplorer, P, Tol)`** (`BRepClass3d_SClassifier.cxx:203-523`):

1. `Reject(P)` — true only if the solid has no faces at all → `myState = 3` (IN), i.e. an
   empty solid is the whole space (`:207-212`, `SolidExplorer.cxx:990-993`).
2. **Vertex/edge proximity pre-test** (`:214-230`): select the UB-tree with a point selector;
   any hit → `myState = 2` (**ON**) and return. This is the tolerance-band test and it runs
   *before* any ray is cast.
3. `mapEF` = edge → faces over the whole shape (`:232-234`).
4. **Ray loop** `while (isFaultyLine)` (`:257-516`):
   * `Segment(P,L,Par)` for the first iteration (which is `myFirstFace = 0; OtherSegment(...)`,
     `SolidExplorer.cxx:1095-1101`), `OtherSegment` afterwards (`:261-266`);
   * `aCurInd = GetFaceSegmentIndex()` must be **strictly greater** than the previous, else
     `myState = 1` (Faulty) and return (`:268-278`);
   * `iFlag == 1` → ON (point on an infinite face) return (`:280-285`);
     `iFlag == 2` → OUT return (`:286-290`);
     `iFlag == 3` → point on the surface but outside the face: **skip this face, retry**
     (`:292-298`);
   * `parmin = RealLast()`, `NearFaultPar = RealLast()`;
   * **line vs edges/vertices** (`:303-361`): a line selector over the same UB-tree. Every
     vertex hit records `NearFaultPar = min |LP|`. Every edge hit whose two endpoints were
     *not* themselves hit is resolved by `GetTransi(f1,f2,E,param,L,tran)`:
     `GetTransi` (`:654-724`) takes the two adjacent face normals at that edge parameter and
       - returns −1 (faulty) if either normal is missing, or if `|L.Direction · n| <
         Precision::Angular()` for either face — **this is the grazing rejection**
         (`:677-683`);
       - if the two normals are parallel, decides by the sign of `n1·L` (`:685-701`);
       - otherwise projects `L` into the plane spanned by the two normals and requires both
         `n1·projL` and `n2·projL` to have the **same** sign beyond `Precision::Angular()`;
         mixed signs → return 0 = "skip" (`:703-722`).
     A usable transition with `|Lpar| < |parmin|` sets `parmin` and `Trans(parmin, tran,
     myState)` (`:351-355`); otherwise it becomes another `NearFaultPar` candidate.
   * **face intersections** (`:363-509`): for each face,
     ```
     addW = max(10*Tol, 0.01*Par)                                            (:380)
     AddW = addW
     if the face box is finite:  addW = max(addW, GetAddToParam(L, Par, box)) (:383-393)
     minW = -AddW;  maxW = min(Par*10, Par + addW)                            (:395-396)
     Intersector3d.Perform(L, minW, maxW)                                     (:397)
     ```
     `GetAddToParam` (`:569-602`) returns the largest line parameter among the 8 box corners,
     so the segment is guaranteed to reach past the face; any infinite corner returns 1e20.
     * If the intersector finds **no** point and reports `IsParallel()`, an `Extrema_ExtPS`
       from `P` to the surface decides: distance ≤ `Tol` and the UV point classified IN/ON →
       `myState = 2` (ON), `parmin = 0` (`:400-443`).
     * Otherwise, for each hit with `|w| < |parmin| - Precision::PConfusion()` (`:446-447`):
       - `|parmin| <= Tol` → **ON**, record face, break (`:451-456`);
       - hit state `TopAbs_IN` (strictly inside the face) and transition ≠ `Tangent` →
         `Trans(parmin, tran, myState)` (`:458-474`); a `Tangent` transition is *ignored*
         (`:464-470`) — the second grazing rejection;
       - hit state `TopAbs_ON` (on the face boundary) → `isFaultyLine = true`, **restart the
         whole ray loop with a different ray** (`:477-481`) — the third and main grazing
         rejection.
   * after all faces: if `NearFaultPar` was set and `|parmin| >= |NearFaultPar| -
     Precision::PConfusion()`, the ray passed closer to a suspicious feature than to its
     decisive hit → `isFaultyLine = true`, retry (`:511-515`).
5. `State()` maps `myState`: 2→ON, 3→IN, 4→OUT, **anything else (0 = untouched, 1 = faulty)
   → OUT** (`:525-542`, comment at `:540`).

`Trans(parmin, tran, state)` (`:728-746`): if `parmin < 0` flip the transition; then
`Out → state 3 (IN)`, otherwise `state 4 (OUT)`. Rationale: the ray leaves the material at
the nearest crossing ⟺ `P` was inside.

**`OtherSegment(P, L, Par)`** (`SolidExplorer.cxx:493-787`) — the ray generator, and the
place where grazing is *prevented* rather than detected:
```
for (;;) {
  ++myFirstFace
  for each face from index myFirstFace onward:
     surf.Initialize(face, aRestr); U1,V1,U2,V2 = parameter bounds
     eps = Precision::PConfusion()                                            (:559)
     epsU = max(eps*max(|U2|,|U1|), eps);  epsV likewise                      (:560-561)
     if |U2-U1| < epsU or |V2-V1| < epsV -> return 2   // degenerate face -> OUT (:562-565)
     Extrema_ExtPS(P, surface, TolU=TolV=Precision::PConfusion(), MIN)        (:576)
     pick the nearest extremum whose (u,v) is inside the bounds               (:580-600)
     if Dist2Min < 1.e-24:                                                    (:602-604)
         if the face is UV-infinite -> return 1 (ON)                          (:606-609)
         else classify (u,v) in the face:
             IN or ON -> return 1 (ON);  else -> return 3 (skip this face)    (:612-631)
     if the face is UV-infinite: L = line(P -> nearest extremum); return 0    (:634-642)
     (u,v) := nearest extremum parameters                                     (:644-647)
     do {
        PointInTheFace(face, APoint, u, v, myParamOnEdge, ++IndexPoint, ...)  (:653-665)
        V = APoint - P;  Par = |V|
        if Par > gp::Resolution() and both surface derivatives nonzero:
            Norm = D1U x D1V;  tt = |Norm·V| / (|Norm| * Par)                 (:673-677)
            if tt > maxscal:
                maxscal = tt;  L = line(P, V);  _Par = Par                    (:678-682)
                if maxscal > 0.2 -> return 0        // <== the grazing gate   (:684-688)
     } while (IndexPoint < 200 && NbPointsOK < 16)                            (:693)
     if maxscal > 0.2 -> return 0                                             (:696-699)
     ...
  if no faces at all: myReject = true; return 0                               (:724-732)
  if some ray was found (ptfound): return 0                                   (:734-737)
  myFirstFace = 0
  myParamOnEdge ladder: 0.512345 -> 0.4 -> 0.6 -> 0.3 -> 0.7 -> 0.2 -> 0.8
                        -> 0.1 -> 0.9 -> then halve each round                (:739-784)
  if myParamOnEdge < 0.0001: L = line(P, P + (1,0,0)); Par = 1; return 0      (:775-783)
}
```
**`maxscal > 0.2` is the grazing gate**: the ray must strike its target face at
`|cos(angle between ray and surface normal)| > 0.2`, i.e. at least ≈ 11.5° away from
tangency. If no candidate point on any face reaches 0.2, OCCT re-seeds the point search with
a different `myParamOnEdge` (which changes where on each boundary edge the interior-point
search starts) and tries again — up to the ladder above, and finally falls back to the
**+X axis ray** (`:714-719`, `:777-782`).

`PointInTheFace` (`:241-425`) generates the target points: a 6×6 grid over each quadrant of
the UV box starting at the centre and marching outward (`du=(U2-U1)/6`, `dv=(V2-V1)/6`,
clamped at 1e-12, `:255-263`; four quadrant sweeps `:299-367`), then a 37×37 grid over the
whole box (`:369-396`), then the centre (`:397-409`). Each candidate is accepted only if
`ClassifyUVPoint` returns `TopAbs_IN`, where `ClassifyUVPoint` (`:221-237`) *first* rejects
points whose 3D image is inside the vertex/edge tolerance boxes (returns ON) and only then
consults the face's 2D classifier. `IndexPoint` is a resume cursor so successive calls yield
successive points. If the face is not in `myMapOfInter`, it falls back to
`FindAPointInTheFace` (`:74-187`): step off a boundary edge by `TolInit = 0.00001` along the
inward 2D normal, ray-cast in 2D to the nearest other edge, take `ParamInit *= 0.41234` of
that distance, verify with `BRepTopAdaptor_FClass2d` that the point is `TopAbs_IN`, and
require a non-degenerate surface cross product (`:156-184`).

**`SClassifier::PerformInfinitePoint(SolidExplorer, Tol)`**
(`BRepClass3d_SClassifier.cxx:82-199`) — used by `IsHole` and by
`BOPTools_AlgoTools::IsInvertedSolid`:
```
if Reject(origin) -> myState = 3 (IN)   // no faces: whole space              (:96-100)
myFace.Nullify(); myState = 2           // default = ON if nothing decides     (:110-111)
faces = every face of every shell                                              (:114-121)
for itry in 0..9:                       // NB_MAX_POINTS_PER_FACE = 10         (:124-125)
  for each face aF:                                                            (:127)
    aParam = 0.1 + 0.8*BullardRandom()   // random in [0.1, 0.9]               (:134)
    if !FindAPointInTheFace(aF, aPoint, u, v, aParam) -> continue              (:135-139)
    if !FaceNormal(aF, u, v, aDN)        -> continue                           (:136-139)
    aLin = gp_Lin(aPoint, -aDN)          // shoot BACKWARD along the outward normal (:141)
    parmin = RealLast()
    for every face of every shell:  Intersector3d.Perform(aLin, -RealLast(), parmin)
        keep the hit with the smallest w; record its state and transition       (:143-180)
    if the winning hit is TopAbs_IN (strictly inside a face):                   (:182)
        transition Out -> myState = 3 (IN)  and return                          (:184-190)
        transition In  -> myState = 4 (OUT) and return                          (:191-195)
```
`FaceNormal` (`:606-627`) reverses the normal when the face is REVERSED (`:622-625`). Note
that the ray starts *on* face `aF` and travels along `-aDN`; if the first thing it meets is
the inside of the shell, the shell encloses the region on the normal's negative side and the
solid is "a hole in space". If ten random points on every face never produce a decisive hit,
`myState` remains 2 → `State()` returns `TopAbs_ON` → **`IsHole` returns false** → the shell
is treated as a **growth**. That is OCCT's documented fallback for an undecidable shell.

`BRepClass3d_SolidClassifier::PerformInfinitePoint` also caches
`isaholeinspace = (State() != TopAbs_OUT)` (`SolidClassifier.cxx:222`), which is dead under
the current `MARCHEPASSIUNESEULEFACE 0` compile setting.

`RejectShell(gp_Lin)` and `RejectFace(gp_Lin)` always return `false`
(`SolidExplorer.cxx:1037-1040`, `:1084-1087`) — the shell/face pre-rejection hooks exist but
are unimplemented. Everything is intersected.

### 2.9 Stage D — `PerformInternalShapes`

`BuilderSolid.cxx:602-759`. Skipped entirely when `myAvoidInternalShapes` is set (`:604-608`)
— which `BOPAlgo_BOP::BuildSolid` does set (`BOP.cxx:1237`) but `BuildBOP` does not.
```
collect all faces of myLoopsInternal into aMFs                                (:619-629)
if myAreas empty:                                                             (:633)
    make ONE solid out of MakeInternalShells(aMFs) and return                 (:636-650)
BOPAlgo_Tools::ClassifyFaces(aLFaces, myAreas, ..., aMSLF, myBoxes, {}, ...)  (:673-681)
for each solid with a non-empty IN list:                                      (:689-722)
    MakeInternalShells(those faces) -> shells, each added to the solid        (:713-721)
any face not classified into any solid -> aMFUnUsed                           (:726-735)
if aMFUnUsed non-empty:
    MakeInternalShells(aMFUnUsed) -> one shape (single shell or compound)
    AddWarning(BOPAlgo_AlertSolidBuilderUnusedFaces(that shape))              (:737-758)
```
`MakeInternalShells` (`:763-819`) builds connexity blocks by shared edge and forces every
face to `TopAbs_INTERNAL` (`:791`, `:810`), recording `Closed(BRep_Tool::IsClosed(shell))`
(`:816`).

**Unused faces never enter the result** — comment at `:724-725`: *"Do not put such faces into
result as they will form not closed solid."* They are reported only.

`BOPAlgo_Tools::ClassifyFaces` (`BOPAlgo/BOPAlgo_Tools.cxx:1622-1747`) is the same routine
used by `FillIn3DParts`; per solid it runs `BOPAlgo_FillIn3DParts::Perform` (`:1334-1519`):
BVH box selection (`:1345-1354`), exclusion of the solid's own faces (`:1384-1393`),
connexity blocks stopped at the solid's own edges (`:1465`, `MakeConnexityBlock` at
`:1555-1615`), whole-block bbox pre-reject over vertex boxes (`:1467-1491`), then **one**
`BOPTools_AlgoTools::IsInternalFace(representativeFace, solid, aMEFDS,
Precision::Confusion(), ctx)` per block (`:1505-1509`); the whole block inherits the verdict
(`:1510-1517`). A solid with **no faces at all** (`bIsEmpty`) classifies every candidate as
IN with no test (`:1405-1417`).

`IsInternalFace(face, solid, MEF, tol, ctx)` (`BOPTools_AlgoTools.cxx:807-891`) tries the
**angle method** first: find an edge of the face that is also in the solid's edge→face map;
with 1 ancestor and an INTERNAL occurrence, or with 2 ancestors, call the 4-argument
`IsInternalFace(face, edge, F1, F2, ctx)` (`:939-990`) which builds the `theLCSOff` list
{(edge,face), (edgeOff,F2)} and asks `GetFaceOff` whether `theFace` is the selected one
(`:977-987`); return codes 0 = not internal, 1 = internal, 2 = undecidable. Only on 2, or
when no suitable edge exists, does it fall through to the ray classifier
`ComputeState(face, solid, tol, bounds, ctx)` (`:884-890`).

### 2.10 SELECTION — `BuildBOP` (the general rule)

`BOPAlgo/BOPAlgo_Builder.cxx:479-885`. Entry from `BOP.cxx:885-897` when any argument solid
is open. `BuildBOP(objects, tools, operation, ...)` maps the op to two states
(`BOPAlgo/BOPAlgo_Builder.hxx:214-250`):

| operation | objects state | tools state |
|---|---|---|
| `COMMON` | `IN` | `IN` |
| `FUSE` | `OUT` | `OUT` |
| `CUT` (A−B) | `OUT` | `IN` |
| `CUT21` (B−A) | `IN` | `OUT` |

Anything else → `UNKNOWN` and the state-form rejects it (`Builder.cxx:500-505`).

**Harvest** (`:559-629`). Four maps per group, and **the identity used is the whole
mechanism**:

| map | declared | identity |
|---|---|---|
| `aMObjFacesOri`, `aMToolFacesOri` | `:559` | **oriented** (`IsEqual`: TShape+Location+Orientation) |
| `aMObjFaces`, `aMToolFaces` | `:561` | unoriented (`IsSame`) |
| `anINObjects`, `anINTools` | `:563` | unoriented |
| `aMResFacesOri` | `:643` | **oriented** |
| `aMResFacesFence` | `:644` | unoriented |
| `aMFence`, `aMFToAvoid` | `:646` | unoriented |
| `aMFenceOri` | `:648` | **oriented** |

(`TopTools/TopTools_ShapeMapHasher.hxx:34-37` → `IsSame`; default hasher →
`TopoDS_Shape::operator==` → `IsEqual`, `TopoDS/TopoDS_Shape.hxx:268-282`.)

Harvest body:
```
for group i in {objects, tools}:
  for each SOLID of each input shape:
    for each FACE aF (TopExp_Explorer -> orientation COMPOSED with the shell's):  (:575-582)
       if aF.Orientation() not in {FORWARD, REVERSED}: continue                   (:583-586)
       if myImages has splits of aF:
          for each split aFImRef:
              if IsSplitToReverse(aFImRef, aF, ctx): add aFImRef.Reversed()
              else                                   add aFImRef                  (:593-606)
          (added to BOTH the oriented and the unoriented map)
       else add aF to both maps                                                   (:608-612)
    union myInParts[solid] into anINObjects / anINTools                           (:616-626)
```
`myInParts[solid]` (`BOPAlgo/BOPAlgo_Builder_3.cxx:243-261`) = the faces of *other* shapes
classified IN this solid, followed by the solid's own INTERNAL faces. It is bound only when
one of those lists is non-empty (`:244`), so a missing entry means "nothing inside".
**This is the IN/OUT state source — I-10.**

**Derived flags** (`:634-641`):
```
isObjectsIN     = (objState  == IN)
isToolsIN       = (toolState == IN)
bAvoidIN        = (!isObjectsIN && !isToolsIN)          // FUSE only
bAvoidINforBoth = (isObjectsIN != isToolsIN)            // CUT and CUT21 only
isSameOriNeeded = (objState == toolState)               // true for FUSE and COMMON
```

**The selection loop** (`:650-737`), per group `i`, per **oriented** face `aFIm` of
`aMap = (i ? aMToolFacesOri : aMObjFacesOri)`, in map order:

1. `isIN = anINMap.Contains(aFIm)` (own group), `isINOpposite = anOppositeINMap.Contains(aFIm)`
   (other group) — both unoriented (`:666-667`).
2. `if (bAvoidIN && (isIN || isINOpposite)) continue;` — **FUSE drops every interior wall**
   (`:670-673`).
3. `if (bAvoidINforBoth && isIN && isINOpposite) continue;` — **CUT/CUT21 drop doubly-interior
   walls** (`:676-679`).
4. `if (!aMFence.Add(aFIm))` — this is the **second unoriented sighting** of this face
   (`:682`). Two sub-cases:
   * **not in `anOppositeMap`** → the two sightings are both from *this* group: a wall between
     two solids of the same operand. `if (bTakeIN != isSameOriNeeded) aMFToAvoid.Add(aFIm);`
     and then **fall through** to step 5 — there is no `continue` here (`:684-692`).
     FUSE (`bTakeIN=false`, `needed=true`) ⇒ avoided (two touching operand solids merge).
     COMMON (`true`,`true`) ⇒ kept, and each sighting contributes its own orientation, so the
     wall survives two-sided.
   * **in `anOppositeMap`** → the **same-domain wall shared by both operands**:
     ```
     isSameOri = !aMFenceOri.Add(aFIm)      // true iff this exact orientation was seen before (:696)
     if (isSameOriNeeded == isSameOri):
         if (aMResFacesFence.Add(aFIm)) aMResFacesOri.Add(aFIm);   // keep ONCE       (:697-704)
     else:
         aMFToAvoid.Add(aFIm);                                                        (:706-709)
     continue;                                                                        (:711)
     ```
     The `aMResFacesFence` guard means: **if the other group already emitted this face through
     the ordinary path at step 6, this add is suppressed and the surviving orientation is the
     other group's.**
5. `if (!aMFenceOri.Add(aFIm)) continue;` — one traversal per *oriented* face (`:714`).
6. Keep test and reversal (`:719-735`):
   ```
   if (bTakeIN == isINOpposite) {
       if (isIN)                              { add aFIm; add aFIm.Reversed(); }   (:721-725)
       else if (bTakeIN && !isSameOriNeeded)  { add aFIm.Reversed(); }             (:726-729)
       else                                   { add aFIm; }                        (:730-733)
       aMResFacesFence.Add(aFIm);                                                  (:734)
   }
   ```
   The `else if` at `:726` **is** the CUT tool-face reversal (and the CUT21 object-face
   reversal). `isIN` outranks it: a face that is IN a solid of its own group is emitted in
   **both** orientations, unreversed.
7. Emit `aMResFacesOri` minus everything in `aMFToAvoid` (`:740-749`). Because `aMFToAvoid`
   is **unoriented**, marking one orientation kills all orientations of that face.

**The same-domain rows, resolved.** An ON (same-domain) face is never in either IN map,
because `ClassifyFaces` excludes a solid's own faces (`BOPAlgo_Tools.cxx:1389-1392`). So it
reaches step 4's second branch. Combining that with which group passed step 6 first:

| op | states | `isSameOriNeeded` | ON, normals AGREE | ON, normals OPPOSE |
|---|---|---|---|---|
| FUSE | OUT/OUT | true | **keep once**, objects' orientation | drop both |
| COMMON | IN/IN | true | **keep once**, objects' orientation | drop both |
| CUT (A−B) | OUT/IN | false | drop both | **keep once**, objects' orientation (objects passed step 6: `bTakeIN=false == isINOpposite=false`) |
| CUT21 (B−A) | IN/OUT | false | drop both | **keep once**, *tools'* orientation (objects fail step 6 because `bTakeIN=true ≠ isINOpposite=false`, leaving `aMResFacesFence` free for the tools' sighting) |

**After selection** (`:750-885`):
```
BOPAlgo_BuilderSolid aBS;  SetShapes(aResFaces); SetContext; SetFuzzyValue; Perform (:754-759)
keep a solid only if it contains at least one face from aMObjFacesOri or
    aMToolFacesOri  (ORIENTED containment)                                         (:774-788)
unused result faces -> connexity blocks -> shell -> OrientFacesOnShell -> solid,
    with NO closedness check                                                       (:796-833)
if (!bAvoidIN) BOPAlgo_Tools::FillInternals(...)   // never for FUSE               (:835-871)
myShape = compound of aResSolids;  PrepareHistory                                  (:873-884)
```

### 2.11 SELECTION — `BuildRC` (the whole-solid cell rule) and `BuildSolid`

`BOP.cxx:583-867`. This is the path taken for closed operands.

* **FUSE**: `myRC` = every distinct shape of type `TypeToExplore(myDims[0])` found in
  `myShape` (the general-fuse compound) (`:594-609`). Then `BuildSolid` (`:902-906`).
* **COMMON / CUT / CUT21** (`:615-867`):
  ```
  aMArgs, aMTools  = the building elements of the arguments/tools (per-dimension type) (:622-645)
  aMArgsIm, aMToolsIm = their images (or themselves if untouched)                      (:655-705)
      untouched SOLIDs also get a BOPTools_Set face-signature entry in aMSetArgs/Tools  (:694-702)
  aMIt      = bCut21 ? aMToolsIm : aMArgsIm       // the side we iterate               (:715-716)
  aMCheck   = bCut21 ? aMArgsIm  : aMToolsIm      // the side we test membership in    (:717-718)
  for COMMON only: expand aMIt down to iDimMin (sub-shapes of each dimension)          (:724-738)
  always: expand aMCheck down to iDimMin                                               (:744-755)
  for each S in aMItExp:
      bContains = aMCheckExp.Contains(S)                     // IsSame                 (:762)
      if !bContains and S is a SOLID: retry via BOPTools_Set face signature            (:763-768)
      COMMON: keep iff bContains;   CUT/CUT21: keep iff !bContains                     (:770-783)
  COMMON post-filter: from dimension 3 down to iDimMin, keep only maximal shapes       (:787-809)
  degenerate-edge re-injection: a DE whose vertex is in the result, is not a new
      shape, and has no interference, is added back                                    (:821-864)
  ```
  So `BuildRC`'s rule for solids is: **a split solid of A survives A−B iff it is not also a
  split solid of B**, with `IsSame` identity first and an unordered face-set signature
  (`BOPTools/BOPTools_Set.cxx`) as fallback. This is exactly guarantee I-9's "each image used
  once" in its cheapest form, and it produces multi-solid results naturally.

`BuildSolid` (`BOP.cxx:1097-1378`, FUSE-3D only): map FACE→SOLID over the *arguments* to find
faces shared between operand solids (`:1112-1151`); then `MapFacesToBuildSolids` over the RC
solids (`:1754-1784`), which stores a face under its **unoriented** key and appends a second
owner **only if the second sighting's orientation differs** (`:1777-1782`); faces whose owner
list has exactly one entry form the face set for a fresh `BuilderSolid` run with
`SetAvoidInternalShapes(true)` (`:1218-1237`). That "unoriented key + oriented comparison"
pair is the second occurrence of the two-fence idiom (I-11).

### 2.12 History / provenance

`BOPAlgo_Builder::PrepareHistory` (`BOPAlgo/BOPAlgo_Builder_4.cxx:164-252`):
```
if !HasHistory() return                                                            (:166-169)
myHistory = new BRepTools_History
myMapShape = every sub-shape of myShape                                            (:175-176)
for i in 0..myDS->NbSourceShapes()-1:
    aS = myDS->Shape(i)
    if !BRepTools_History::IsSupportedType(aS) continue                            (:192-195)
    isModified = false
    pLSp = LocModified(aS)          // = myImages.Seek(aS)   (:157-160)
    for each split aSp in *pLSp:
        if myMapShape.Contains(aSp):
            if aSp is VERTEX or SOLID:  aSp.Orientation(aS.Orientation())          (:217-221)
            else if IsSplitToReverse(aSp, aS, ctx): aSp.Reverse()                  (:222-225)
            myHistory->AddModified(aS, aSp); isModified = true                     (:227-228)
    for each aG in LocGenerated(aS):
        if myMapShape.Contains(aG): myHistory->AddGenerated(aS, aG)                (:234-243)
    if (!isModified && !myMapShape.Contains(aS)) myHistory->Remove(aS)             (:247-250)
```
`LocGenerated` (`:27-153`): for an EDGE, the new vertices of every EE and EF interference it
participates in; for a FACE, the new vertices of its EF interferences **plus** every section
edge in `FaceInfo(nS).PaveBlocksSc()` and every section vertex in `VerticesSc()`
(`:124-150`). Only entities present in the result are reported.

The three provenance maps themselves:
* `myImages : source shape -> list of images` — **IsSame-keyed**, so orientation must be
  re-derived at every use site. An **empty bound list** means "deliberately deleted"
  (e.g. micro edges, `Builder_1.cxx:95`); an **absent** binding means "untouched, use the
  source". Three consumers branch on that distinction (`Builder_1.cxx:145`,
  `Builder_2.cxx:720`, `Builder_3.cxx:131`).
* `myOrigins : image -> list of sources`, filled for solids at `Builder_3.cxx:604-609`.
* `myShapesSD : same-domain member -> representative`, including the **self-binding**
  `myShapesSD[rep] = rep` (`Builder_2.cxx:876-881`), which is a branch selector at
  `Builder_3.cxx:311`, not bookkeeping.

### 2.13 Where OCCT gives up, falls back, or is wrong

These are facts about OCCT; none is invented, and a port must decide consciously for each.

| # | site | behaviour |
|---|---|---|
| G1 | `SClassifier::PerformInfinitePoint` (`BRepClass3d_SClassifier.cxx:110-198`) | after 10 random points × every face with no decisive transition, `myState` stays 2 → `State()` = `ON` → `IsHole` returns **false** → the shell is treated as a growth. |
| G2 | `SClassifier::State` (`:525-542`) | `myState` 0 (never touched) and 1 (Faulty) both map to **`TopAbs_OUT`**. A failed classification is silently reported as OUT. |
| G3 | `SClassifier::Perform` (`:268-278`) | if `OtherSegment` does not advance the face index, `myState = 1` → OUT (G2). This is the ray-search exhaustion path. |
| G4 | `SolidExplorer::OtherSegment` (`:714-719`, `:775-783`) | last-resort ray is the **+X axis** through `P`, with `Par = 1`. |
| G5 | `GetFaceOff` (`BOPTools_AlgoTools.cxx:1085-1089`) | returns `false` ("minimal angle cannot be found") when two candidates tie within `Precision::Confusion()`. `IsInternalFace` maps that to code 2 and falls back to the ray classifier (`:978-982`, `:884-890`). **`ShellSplitter::SplitBlock` ignores the return value entirely (`ShellSplitter.cxx:359`) and uses the ambiguous pick.** |
| G6 | `GetFaceDir` (`:2145-2157`) | if the circle-intersection point search fails, falls back to the hatcher; if that fails too, `bIsComputed = false` and `GetFaceOff` forces the angle to `2π` (`:1075-1082`) so the unreliable face loses. |
| G7 | `MinStep3D` (`:2301-2303`) | every non-quadric surface gets `aDtMin = 5e-4` unconditionally. Features smaller than that are decided by the hatcher fallback. |
| G8 | `ShellSplitter::SplitBlock` (`:185-227`) | faces with free edges are peeled iteratively; if all are peeled the block yields **no shells at all** and the faces are silently returned to the caller as "not in any loop" (they end up in `myShapesToAvoid`, `BuilderSolid.cxx:320-331`). |
| G9 | `BuilderSolid::PerformAreas` (`:578-597`) | a cavity shell contained in no growth solid becomes a solid on its own with an **infinite** bounding box. |
| G10 | `BuilderSolid::PerformInternalShapes` (`:724-758`) | faces classified into no solid are **excluded from the result** and packaged into `BOPAlgo_AlertSolidBuilderUnusedFaces`. |
| G11 | `BOP::BuildShape` (`:875-897`) | if any argument solid is open, the entire image-based path is abandoned and the result is rebuilt from face states, **losing solid history** (comment `:880-884`). |
| G12 | `BuildBOP` (`:790-793` + `BOP.cxx:890`) | a `BuilderSolid` failure returns without adding any alert, and the caller treats an alert-free report as success. **Genuine defect — do not port**; emit an explicit failure. |
| G13 | `BuildBOP` (`:796-833`) | unused faces become solids with no closedness check. **Gate on shell closure unless you want open "solids".** |
| G14 | `BOP::BuildSolid` (`:1139-1140`) | `for (i = 1; i < aNb; ++i)` over `aMFS.Extent()` never visits the last entry. **Off-by-one; use `i <= aNb`.** |
| G15 | `SolidExplorer::RejectShell/RejectFace` (`:1037-1040`, `:1084-1087`) | always `false`; the pre-rejection hooks are unimplemented, so every face is intersected on every ray. |
| G16 | `ClassifyFaces` / `FillIn3DParts` (`BOPAlgo_Tools.cxx:1405-1417`) | a solid with no faces classifies **every** candidate face as IN with no test. |

### 2.14 Every constant, with its value

| constant | value | site |
|---|---|---|
| `Precision::Confusion()` | `1.e-7` | `Precision/Precision.hxx:165` |
| `Precision::SquareConfusion()` | `1.e-14` | `:169` |
| `Precision::Angular()` | `1.e-12` | `:123` |
| `Precision::PConfusion()` | `1.e-9` | `:334` |
| `RealSmall()` | `DBL_MIN` = `2.2250738585072014e-308` | `Standard/Standard_Real.hxx:132` |
| `PAR_T` (intermediate-point split) | `0.43213918` | `BOPTools_AlgoTools2D.cxx:407`, `IntTools_Tools.cxx:257` |
| `GetFaceOff` `aAngleMin` init | `100.` | `BOPTools_AlgoTools.cxx:1012` |
| `GetFaceOff` `anAngleCriteria` | `Precision::Confusion()` = `1e-7` | `:1042` |
| `FindPointInFace` max iterations | `15` | `:2189` |
| `FindPointInFace` `aDTol` | `Precision::Angular()`, or `5.e-16 * |P|` when `|P| > 1000` | `:2182-2187` |
| `FindPointInFace` edge clearance | `2 * Tolerance(edge)` | `:2204` |
| `MinStep3D` `aDtMin` floor | `5.e-6` | `:2262` |
| `MinStep3D` sphere/other floor | `5.e-4` | `:2293`, `:2302` |
| `MinStep3D` large-radius rule | `R > 100` ⇒ `aDtMin ≥ sqrt(d²+2dR)`, `d = 10·PConfusion() = 1e-8` | `:2306-2310` |
| `MinStep3D` base step | `2·(tol(edge) + tol(face))` | `:2271` |
| `MinStep3D` "small face" test | `2·UResolution(aDtMax) > uMax-uMin` (or V) | `:2334`, `:2344` |
| `SolidExplorer` initial `myParamOnEdge` | `0.512345` | `SolidExplorer.cxx:905` |
| `myParamOnEdge` ladder | `0.512345, 0.4, 0.6, 0.3, 0.7, 0.2, 0.8, 0.1, 0.9`, then ×0.5 | `:739-784` |
| `myParamOnEdge` give-up threshold | `< 0.0001` | `:710`, `:775` |
| **grazing gate** `maxscal` | `> 0.2` (≈ 11.5° off tangency) | `:684`, `:696` |
| point-search budget | `IndexPoint < 200 && NbPointsOK < 16` | `:693` |
| "point is on the surface" squared distance | `1.e-24` | `:602` |
| `FindAPointInTheFace` `TolInit` | `0.00001` | `:110` |
| `FindAPointInTheFace` step fraction | `ParamInit *= 0.41234` | `:158` |
| `PointInTheFace` coarse grid | `(U2-U1)/6`, `(V2-V1)/6`, floor `1e-12` | `:255-263` |
| `PointInTheFace` fine grid | `/37.0`, floor `1e-12` | `:369-378` |
| `PerformInfinitePoint` tries per face | `NB_MAX_POINTS_PER_FACE = 10` | `SClassifier.cxx:124` |
| `PerformInfinitePoint` random param | `0.1 + 0.8·rand` ∈ `[0.1, 0.9]` | `:134` |
| `SClassifier` segment prolongation | `addW = max(10·Tol, 0.01·Par)`, extended to the face box | `:380-393` |
| `SClassifier` segment window | `[-AddW, min(10·Par, Par+addW)]` | `:395-396` |
| `IntCurvesFace_Intersector` tolerance | `Precision::Confusion()` | `SolidExplorer.cxx:924` |
| `IsInside` classification tolerance | `Precision::Confusion()` | `BuilderSolid.cxx:857` |
| `IsHole` classification tolerance | `RealSmall()` | `BuilderSolid.cxx:828` |
| `IsInvertedSolid` tolerance | `1.e-7` (literal) | `BOPTools_AlgoTools.cxx:2412` |
| `FillInternalShapes` state tolerance | `1.e-11` (literal) | `Builder_3.cxx:836` |
| `ClassifyFaces`/`IsInternalFace` tolerance | `Precision::Confusion()` | `BOPAlgo_Tools.cxx:1508` |
| `AreFacesSameDomain` tolerance | `tol(F1) + tol(F2) + max(fuzz, Confusion())`, each face tolerance raised to the max non-degenerate edge tolerance of F1 | `BOPTools_AlgoTools.cxx:1168-1199` |
| `PostTreat` tolerance cap | `0.05` | `BOPAlgo_Builder.cxx:472` |
| `BOPTools_Set` hash modulus | `myUpper = 432123` | `BOPTools_Set.cxx:34` |

---

## 3. DATA STRUCTURES AND C++ DECLARATIONS FOR OUR PORT

### 3.1 The precondition this subsystem cannot fix

Everything below is index-based over a **shared arena** produced by stages 0–9 of
`kb/ARCHITECTURE_v2.md`. The contract:

> **P-1.** A coincident geometric feature has exactly **one** arena index. Two coincident face
> pieces are one `face` index; two coincident edge pieces are one `edge` index. Faces from A
> and B that are same-domain resolve to the *same* `face` index through the SD representative
> map, and both operands' shells reference that index.
>
> **P-2.** `edge.faces` is exact and derived from the arena, never from geometry comparison.
>
> **P-3.** Any face image that can bound material on either side is present in the input list
> **twice**, once with `reversed=false` and once with `reversed=true`
> (mirrors `BOPAlgo/BOPAlgo_Builder_3.cxx:502-511`).

If P-1 fails, §2's algorithm degrades to exactly the failure mode we have today. Assert P-1
before running: every pair of faces with identical `(surface, trim-loop)` identity must share
an index; every edge appearing twice geometrically must be one index.

### 3.2 New file `src/brep_shell.h`

```cpp
#pragma once
#include "brep.h"
#include <array>
#include <vector>

namespace session_cpp {

// ---------------------------------------------------------------------------
// Oriented references. `face`/`edge` are indices into the arena BRep. The pair
// (face, reversed) is the ORIENTED identity (OCCT IsEqual); `face` alone is the
// UNORIENTED identity (OCCT IsSame). Both are needed -- guarantee I-11.
// ---------------------------------------------------------------------------
struct OrientedFace {
    int  face     = -1;
    bool reversed = false;
    bool operator==(const OrientedFace& o) const { return face == o.face && reversed == o.reversed; }
    bool operator<(const OrientedFace& o) const {
        return face != o.face ? face < o.face : (int)reversed < (int)o.reversed;
    }
};
struct OrientedFaceHash {
    size_t operator()(const OrientedFace& f) const {
        return (size_t)f.face * 2u + (size_t)f.reversed;
    }
};

struct Box3 {
    std::array<double,3> mn{ 1e300, 1e300, 1e300};
    std::array<double,3> mx{-1e300,-1e300,-1e300};
    bool whole = false;
    void add(const Point& p);
    bool is_void()  const { return !whole && mn[0] > mx[0]; }
    bool is_out(const Box3& o, double gap = 0.0) const;   // false if `whole`
    void set_whole() { whole = true; }
};

// ---------------------------------------------------------------------------
// A shell: a maximal set of oriented faces connected through SHARED EDGE INDICES.
// `closed` is the OCCT edge-parity test (BRep_Tool.cxx:1707-1729), NOT "2 trims".
// `manifold` is the stronger condition we additionally record.
// `is_hole` is set by classify_infinite_point == IN (BuilderSolid.cxx:823-831).
// ---------------------------------------------------------------------------
struct Shell {
    std::vector<OrientedFace> faces;
    bool closed   = false;
    bool manifold = false;
    bool is_hole  = false;
    bool internal = false;       // built from unused faces; all faces INTERNAL
    Box3 box;
};

struct Solid {
    int              outer_shell = -1;
    std::vector<int> hole_shells;
    std::vector<int> internal_shells;
    Box3             box;
};

// ---------------------------------------------------------------------------
// Diagnostics. Every give-up path in section 2.13 emits one of these (I-13).
// ---------------------------------------------------------------------------
enum class AssemblyAlert {
    ShellSplitterFailed,       // G8: a connexity block produced no shells
    UnusedFaces,               // G10: faces classified into no solid
    OpenShellAcceptedAsSolid,  // G13: only if the caller opts in
    ClassifierUndecided,       // G1/G2/G3: ray search exhausted
    AngularTieUnresolved,      // G5: GetFaceOff could not separate two candidates
    HoleWithoutParent          // G9: cavity shell that contains no growth solid
};
struct Alert {
    AssemblyAlert     kind;
    std::vector<int>  faces;   // arena face indices carrying the evidence
    std::string       detail;
};

// ---------------------------------------------------------------------------
// Port of BOPAlgo_BuilderSolid. `arena` supplies geometry AND the edge->face
// incidence; `input` is the oriented-face list (P-3 doubling applied by caller).
// ---------------------------------------------------------------------------
class BuilderSolid {
public:
    void set_arena(const BRep* arena)                       { m_arena = arena; }
    void set_faces(std::vector<OrientedFace> input)         { m_input = std::move(input); }
    void set_avoid_internal_shapes(bool v)                  { m_avoid_internal = v; }

    void perform();                       // -> BuilderSolid.cxx:76-125

    const std::vector<Shell>& shells()  const { return m_shells; }
    const std::vector<Solid>& areas()   const { return m_solids; }
    const std::vector<Alert>& alerts()  const { return m_alerts; }
    bool  has_errors() const { return m_error; }

private:
    void perform_shapes_to_avoid();       // 2.3, BuilderSolid.cxx:129-219
    void perform_loops();                 // 2.4, BuilderSolid.cxx:223-393
    void perform_areas();                 // 2.7, BuilderSolid.cxx:397-598
    void perform_internal_shapes();       // 2.9, BuilderSolid.cxx:602-759

    const BRep*               m_arena = nullptr;
    std::vector<OrientedFace> m_input;
    std::set<OrientedFace>    m_avoid;        // ORIENTED (BuilderArea.hxx:83)
    std::vector<Shell>        m_shells;       // == myLoops
    std::vector<Shell>        m_internal;     // == myLoopsInternal
    std::vector<Solid>        m_solids;       // == myAreas
    bool                      m_avoid_internal = false;
    bool                      m_error = false;
    std::vector<Alert>        m_alerts;
    SolidClassifierCache*     m_cls = nullptr;
};

} // namespace session_cpp
```

### 3.3 New file `src/brep_shell_split.h` — the connexity + walk

```cpp
namespace session_cpp {

// Connexity block over shared EDGE INDEX only. `regular` iff every edge touched
// by the block has exactly two incident oriented faces IN THE BLOCK, and no face
// was supplied twice.  -> BOPTools_AlgoTools.cxx:187-256
struct ConnexityBlock {
    std::vector<OrientedFace> faces;
    bool regular = true;
};

std::vector<ConnexityBlock>
make_connexity_blocks(const BRep& arena, const std::vector<OrientedFace>& faces);

// Relative-orientation fixer. Reverses faces so that every non-seam edge shared
// by exactly two faces of the shell is traversed oppositely (I-4).
// -> BOPTools_AlgoTools::OrientFacesOnShell, AlgoTools.cxx:363-507
void orient_faces_on_shell(const BRep& arena, Shell& shell);

// Edge-parity closure test.  -> BRep_Tool.cxx:1707-1729
bool shell_is_closed(const BRep& arena, const Shell& shell);
// Stronger: every non-degenerate edge has exactly two incident trims in the shell.
bool shell_is_manifold(const BRep& arena, const Shell& shell);

// Non-manifold walk: free-edge peeling, boundary-face preference, angular pick,
// branch-edge refinement.  -> BOPAlgo_ShellSplitter.cxx:153-421 and :443-617
struct ShellSplitter {
    const BRep*                arena = nullptr;
    std::vector<OrientedFace>  start;
    std::vector<Shell>         out;
    std::vector<Alert>         alerts;
    void perform();
};

// The angular selector.  -> BOPTools_AlgoTools::GetFaceOff, AlgoTools.cxx:994-1103
// Returns false when the minimum is ambiguous within `angle_criteria` (G5); the
// caller decides whether to trust `picked` anyway (SplitBlock does; IsInternalFace
// does not).
bool get_face_off(const BRep&  arena,
                  int          edge,
                  OrientedFace reference,
                  const std::vector<std::pair<int /*edge occurrence*/, OrientedFace>>& candidates,
                  OrientedFace& picked,
                  double        angle_criteria = 1e-7);

} // namespace session_cpp
```

### 3.4 New file `src/brep_classify3d.h` — the point-in-solid classifier

```cpp
namespace session_cpp {

enum class State3d { In, Out, On, Unknown };

// Port of BRepClass3d_SolidClassifier / SClassifier / SolidExplorer.
// Holds per-solid caches: face intersectors, the vertex/edge bounding-box tree,
// and the ray-search cursor (myFirstFace / myParamOnEdge).
class SolidClassifier {
public:
    void load(const BRep& arena, const Solid& s);     // -> SolidExplorer::InitShape :898-982
    void load(const BRep& arena, const Shell& s);     // shell-as-solid, for IsHole

    State3d perform(const Point& p, double tol);      // -> SClassifier::Perform :203-523
    State3d perform_infinite_point(double tol);       // -> SClassifier::PerformInfinitePoint :82-199

    bool undecided() const { return m_undecided; }    // G1/G2/G3 made VISIBLE

private:
    struct Ray { Point origin; Vector dir; double par; };
    // -> SolidExplorer::Segment / OtherSegment :493-787
    // returns 0 ok, 1 point-on-face, 2 degenerate-face-OUT, 3 skip-this-face
    int  next_segment(const Point& p, Ray& out);
    bool find_point_in_face(int face, double param, Point& p3, double& u, double& v);

    const BRep* m_arena = nullptr;
    std::vector<int> m_faces;
    double m_param_on_edge = 0.512345;
    int    m_first_face    = 0;
    bool   m_undecided     = false;
    // vertex/edge tolerance boxes for the ON pre-test  -> SClassifier.cxx:214-230
    struct EVBox { Box3 box; int entity; bool is_vertex; };
    std::vector<EVBox> m_ev;
};

// Cache keyed by solid/shell identity -> IntTools_Context::SolidClassifier :312-323
class SolidClassifierCache {
public:
    SolidClassifier& get(const BRep& arena, const Solid& s);
    SolidClassifier& get(const BRep& arena, const Shell& s);
};

// The three ComputeState overloads  -> BOPTools_AlgoTools.cxx:660-803
State3d compute_state(const BRep& arena, const Point& p,  const Solid& ref, double tol, SolidClassifierCache&);
State3d compute_state(const BRep& arena, int edge,        const Solid& ref, double tol, SolidClassifierCache&);
State3d compute_state(const BRep& arena, OrientedFace f,  const Solid& ref, double tol,
                      const std::set<int>& ref_edges, SolidClassifierCache&);

} // namespace session_cpp
```

### 3.5 New file `src/brep_select.h` — the op table

```cpp
namespace session_cpp {

enum class BopState { In, Out };
struct OpStates { BopState objects; BopState tools; };

// -> BOPAlgo_Builder.hxx:214-250
constexpr OpStates op_states(BRep::BooleanOp op, bool swap_operands) {
    switch (op) {
        case BRep::BooleanOp::Intersection: return { BopState::In,  BopState::In  };  // COMMON
        case BRep::BooleanOp::Union:        return { BopState::Out, BopState::Out };  // FUSE
        case BRep::BooleanOp::Difference:
            return swap_operands ? OpStates{ BopState::In,  BopState::Out }   // CUT21
                                 : OpStates{ BopState::Out, BopState::In  };  // CUT
    }
    return { BopState::Out, BopState::Out };
}

// Per-solid IN-face map computed ONCE in the split stage (I-10).
// -> BOPAlgo_Builder_3.cxx:243-261 / BOPAlgo_Tools::ClassifyFaces :1622-1747
struct InPartsMap {
    // solid index -> set of arena face indices lying IN that solid (UNORIENTED)
    std::map<int, std::set<int>> in_faces;
    bool contains(int solid, int face) const;
};

struct SelectionInput {
    const BRep*                arena;
    std::vector<int>           object_solids;    // split solids of A
    std::vector<int>           tool_solids;      // split solids of B
    std::vector<OrientedFace>  object_faces;     // harvested, orientation composed
    std::vector<OrientedFace>  tool_faces;
    const InPartsMap*          in_objects;
    const InPartsMap*          in_tools;
};

// The full 7-step loop of section 2.10. Returns the oriented face list to feed
// BuilderSolid, plus the alerts.
std::vector<OrientedFace> select_faces(const SelectionInput& in, OpStates states,
                                       std::vector<Alert>& alerts);

} // namespace session_cpp
```

Reference implementation skeleton of `select_faces` — this is the part that must be
transcribed exactly, so it is given in full:

```cpp
std::vector<OrientedFace> select_faces(const SelectionInput& in, OpStates st,
                                       std::vector<Alert>& alerts)
{
    const bool objIN  = (st.objects == BopState::In);
    const bool toolIN = (st.tools   == BopState::In);
    const bool bAvoidIN        = (!objIN && !toolIN);            // FUSE
    const bool bAvoidINforBoth = (objIN != toolIN);              // CUT / CUT21
    const bool isSameOriNeeded = (st.objects == st.tools);       // FUSE, COMMON

    std::set<int>          unorientedOfObjects, unorientedOfTools;   // IsSame maps
    for (auto f : in.object_faces) unorientedOfObjects.insert(f.face);
    for (auto f : in.tool_faces)   unorientedOfTools.insert(f.face);

    std::vector<OrientedFace> resOri;                 // aMResFacesOri   (ORDERED)
    std::set<int>             resFence;               // aMResFacesFence (UNORIENTED)
    std::set<int>             fence, toAvoid;         // aMFence, aMFToAvoid (UNORIENTED)
    std::set<OrientedFace>    fenceOri;               // aMFenceOri      (ORIENTED)
    auto addOri = [&](OrientedFace f) {
        if (std::find(resOri.begin(), resOri.end(), f) == resOri.end()) resOri.push_back(f);
    };

    for (int grp = 0; grp < 2; ++grp) {
        const auto& faces        = grp ? in.tool_faces        : in.object_faces;
        const auto& oppUnoriented= grp ? unorientedOfObjects  : unorientedOfTools;
        const auto& inOwn        = grp ? *in.in_tools         : *in.in_objects;
        const auto& inOpp        = grp ? *in.in_objects       : *in.in_tools;
        const auto& ownSolids    = grp ? in.tool_solids       : in.object_solids;
        const auto& oppSolids    = grp ? in.object_solids     : in.tool_solids;
        const bool  bTakeIN      = grp ? toolIN : objIN;

        for (OrientedFace fim : faces) {
            bool isIN = false, isINOpp = false;
            for (int s : ownSolids) if (inOwn.contains(s, fim.face)) { isIN    = true; break; }
            for (int s : oppSolids) if (inOpp.contains(s, fim.face)) { isINOpp = true; break; }

            if (bAvoidIN        && (isIN || isINOpp)) continue;           // :670-673
            if (bAvoidINforBoth && isIN && isINOpp)   continue;           // :676-679

            if (!fence.insert(fim.face).second) {                          // :682
                if (!oppUnoriented.count(fim.face)) {
                    // duplicate WITHIN one group -- fall through, NO continue  :684-692
                    if (bTakeIN != isSameOriNeeded) toAvoid.insert(fim.face);
                } else {
                    // same-domain wall shared with the other operand           :693-712
                    const bool isSameOri = !fenceOri.insert(fim).second;   // :696
                    if (isSameOriNeeded == isSameOri) {
                        if (resFence.insert(fim.face).second) addOri(fim); // :700-703
                    } else {
                        toAvoid.insert(fim.face);                          // :708
                    }
                    continue;                                              // :711
                }
            }
            if (!fenceOri.insert(fim).second) continue;                    // :714

            if (bTakeIN == isINOpp) {                                      // :719
                if (isIN) {                                                // :721-725
                    addOri(fim);
                    addOri(OrientedFace{fim.face, !fim.reversed});
                } else if (bTakeIN && !isSameOriNeeded) {                  // :726-729  CUT/CUT21 reversal
                    addOri(OrientedFace{fim.face, !fim.reversed});
                } else {                                                   // :730-733
                    addOri(fim);
                }
                resFence.insert(fim.face);                                 // :734
            }
        }
    }

    std::vector<OrientedFace> out;                                          // :740-749
    for (auto f : resOri) if (!toAvoid.count(f.face)) out.push_back(f);
    return out;
}
```

Two transcription traps, both from `kb/audit_occt_builder-assembly.md` §3:
* the *within-one-group duplicate* branch **must not** `continue`;
* `toAvoid` is **unoriented**, so marking one orientation kills both.

### 3.6 Provenance

```cpp
struct History {
    // source entity index -> surviving images present in the result
    std::map<int, std::vector<OrientedFace>> modified;
    // source entity index -> newly created entities present in the result
    std::map<int, std::vector<int>>          generated;
    std::set<int>                            deleted;
};
// -> BOPAlgo_Builder_4.cxx:164-252.  An EMPTY `modified[i]` vector means
// "deliberately deleted"; an ABSENT key means "untouched, use the source".
// That distinction is load-bearing (Builder_1.cxx:145, Builder_2.cxx:720,
// Builder_3.cxx:131) and must be preserved by the map type, not by convention.
void prepare_history(const BRep& arena, const BRep& result,
                     const std::map<int,std::vector<OrientedFace>>& images,
                     History& out);
```

---

## 4. WHAT OUR CODE DOES TODAY, AND EXACTLY WHERE IT DIVERGES

### 4.1 There is no shell and no solid entity

`src/brep.h:27-58` declares `BRepVertex`, `BRepEdge`, `BRepTrim`, `BRepLoop`, `BRepFace`;
`src/brep.h:80-90` declares the pools. **There is no `BRepShell` and no `BRepSolid`.** A
"solid" in our kernel is a `BRep` whose edges happen to be 2-trim.

Consequences, each of which is a divergence from a *named* OCCT stage:

| OCCT stage | our equivalent | divergence |
|---|---|---|
| `PerformShapesToAvoid` (`BuilderSolid.cxx:129-219`) | none | free-edge faces and doubled-orientation faces are never removed before assembly |
| `MakeConnexityBlocks` + `ShellSplitter` (`AlgoTools.cxx:187-256`, `ShellSplitter.cxx:153-421`) | `brep_shell_count` (`src/brep.cpp:63-84`) | our version *does* group by shared edge index correctly, but it is **diagnostic only** — it is invoked at `src/brep.cpp:8308` behind `SESSION_NT_DBG` and its result is explicitly *not* gated on (comment `src/brep.cpp:8300-8307`) |
| `OrientFacesOnShell` (`AlgoTools.cxx:363-507`) | `BRep::face_outward_signs` (`src/brep.cpp:1560-2170`) | see §4.2 |
| `IsHole` / `IsInside` / nesting (`BuilderSolid.cxx:397-598`) | `comp_cavity` inside `face_outward_signs` (`src/brep.cpp:1937-1960`) | see §4.3 |
| `PerformInternalShapes` (`BuilderSolid.cxx:602-759`) | none | unused faces are neither placed nor reported |
| `PrepareHistory` (`Builder_4.cxx:164-252`) | none | `remapA`/`remapB` (`src/brep.cpp:10215`) are subset index remaps discarded after combine |

### 4.2 Orientation: relative propagation is right, the global flip is a vote

`BRep::face_outward_signs` (`src/brep.cpp:1560-2170`) does implement the OCCT idea in its
first half: the comment at `src/brep.cpp:1668-1677` states the invariant, and relative signs
are propagated across every 2-trim edge. That part corresponds to
`OrientFacesOnShell` and is sound.

The divergence is the **global** flip. OCCT does not flip shells at all (§2.4 B.8): it
classifies. Ours computes a per-connected-component score from flux and ray-parity evidence
(`src/brep.cpp:2040-2100`) and flips the whole component if the score is negative
(`src/brep.cpp:2093-2100`). Because that score is a weighted vote of heuristics, it is
"a face sign can only be wrong if the evidence outvotes the WHOLE shell" — which is a
probability statement, not an invariant. The `SESSION_SHELL_ORIENT` override
(`src/brep.cpp:2092+`) exists precisely because the aggregate lands wrong on fragmented
components (`ncomp=7` for z90 cut, per the comment at `src/brep.cpp:2085-2090`).

**Port action**: delete the vote. Replace with (a) `orient_faces_on_shell` for relative
orientation and (b) `SolidClassifier::perform_infinite_point` per shell for the
outer/cavity decision, which is a *per-shell* question requiring no other shell (I-5).

### 4.3 Cavity detection needs other components; OCCT's does not

`src/brep.cpp:1937-1960`: `comp_cavity[c]` is decided by building a mesh per component
(`subset(comp_faces[c]).mesh()`), then, for each face of the component, casting 3 rays from
its interior sample point against the **other** components' meshes and taking a per-face
majority, then a per-component majority.

OCCT: `IsHole(shell) := PerformInfinitePoint(shell-as-solid) == IN`
(`BuilderSolid.cxx:823-831`). One shell. No other shell is consulted. Containment is a
*separate* question answered afterwards by `IsInside` + BVH (`BuilderSolid.cxx:460-576`),
and the answer is used to nest, never to re-orient.

Our formulation cannot distinguish "cavity of the neighbour" from "solid lump inside the
neighbour's bounding region", and it is O(components²) mesh work.

### 4.4 Point-in-solid: we classify against a tessellation

`BRep::contains_point(const Mesh&, const Point&)` (`src/brep.cpp:1238-1268`) is a generalized
winding number over `boundary.to_vertices_and_faces()`, accepted when `|omega| > 2π`
(`src/brep.cpp:1267`). `BRep::contains_point(const Point&)` (`src/brep.cpp:1269-1271`) calls
`mesh()` to build that tessellation.

Divergences from §2.8:
1. **The tessellation is the accuracy floor.** OCCT intersects the ray with the exact surface
   via `IntCurvesFace_Intersector(face, Precision::Confusion(), true, false)`
   (`SolidExplorer.cxx:924`) and classifies the hit in the face's real 2D domain. On curved
   faces our answer is limited by mesh density and by whatever the mesher does at poles and
   seams — which is exactly the family the mission brief measures as 0/20.
2. **No ON state.** OCCT returns IN/OUT/**ON**, and ON short-circuits the whole ray search via
   the vertex/edge tolerance-box tree (`SClassifier.cxx:214-230`). Ours has only a "p is
   exactly a mesh vertex" degenerate escape (`src/brep.cpp:1257`) which returns *true*.
3. **No grazing handling.** OCCT's three defences — the `maxscal > 0.2` ray-acceptance gate
   (`SolidExplorer.cxx:684`), the `TopAbs_ON` hit → retry-with-a-different-ray loop
   (`SClassifier.cxx:477-481`), and the `NearFaultPar` guard (`:511-515`) — have no analog.
   The winding number is claimed to be "crack-robust" (`src/brep.cpp:1239-1245`), which is
   true for *small* cracks but says nothing about a mis-oriented triangle fan at a pole.
4. **No tolerance model.** The `2π` threshold at `src/brep.cpp:1267` is a bare constant; there
   is no entity tolerance anywhere in the path.

`contains_point_exact` (`src/brep.cpp:1273+`) is a closest-point-on-trimmed-boundary variant
and is the closer analog, but it is not what the shell/solid decisions use.

### 4.5 Adjacency comes from coordinates

`BRep::sew_coincident_edges` (`src/brep.cpp:7074-7200`) decides edge identity by a two-way
point-to-polyline Hausdorff test at 64 samples (`src/brep.cpp:7097`) with
`tol = diag * 5e-3` (`src/brep.cpp:7092`). Additionally, the boolean aliases A-side and
B-side section edges by a **quantized float key** `qspan(f) = llround(f*32)`
(`src/brep.cpp:10228`, used at `:10234`, `:10307`).

OCCT's shell walk never compares coordinates: `aEFMap` is built by
`TopExp::MapShapesAndAncestors(F, EDGE, FACE, aEFMap)` (`ShellSplitter.cxx:195`,
`BuilderSolid.cxx:158`, `:348`) and two faces are adjacent iff they literally share an edge
object. **This is the single defect the mission brief names first** ("32 naked edges of 36
… while the UV arrangement is verified identical") and it is unfixable inside this
subsystem: it must be fixed by P-1 in stages 0–9.

### 4.6 Selection: our op table vs OCCT's

Ours, in `BRep::boolean`'s `classify` lambda:

```
src/brep.cpp:9442-9451   ON faces:
    Union        : keep = is_first &&  same_orient
    Intersection : keep = is_first &&  same_orient
    Difference   : keep = is_first && !same_orient
src/brep.cpp:10019-10027 non-ON faces:
    Union        : keep = !inside
    Intersection : keep =  inside
    Difference   : A -> keep = !inside ;  B -> keep = inside, rev_v = true
```
`same_orient` is computed at `src/brep.cpp:9446-9447` from ±eps probes
(`own_p ? in_p : in_m`), i.e. from geometry, not from an orientation fence.
The reversal for the tool side is applied later at `src/brep.cpp:10218-10219`.

Divergence list, each against §2.10:

* **D1 — one identity, not two.** There is no unoriented fence and no oriented fence; the
  same-domain decision is a per-face geometric probe. Violates I-11. Consequence: a
  same-domain wall present in the arena twice with the *same* orientation is
  indistinguishable from two independent walls, so the ON-same and ON-opposite rows collapse.
* **D2 — CUT21 does not exist.** `BRep::boolean_xor` (`src/brep.h:386-393`) assembles
  `A−B` and `B−A` as two independent booleans and unions the results structurally. OCCT's
  CUT21 row keeps the same-domain wall with the **tools'** orientation (§2.10 table); ours
  always prefers the first operand (`is_first` at `src/brep.cpp:9448-9450`), so the CUT21
  wall normal is mirrored.
* **D3 — the ON rows are not OCCT's.** OCCT: FUSE/COMMON keep-once when normals **agree**
  and drop both when they **oppose**; CUT/CUT21 the reverse. Ours keeps
  `is_first && same_orient` for Union *and* Intersection and `is_first && !same_orient` for
  Difference — the *sense* matches, but "keep once" is enforced only by `is_first`, i.e. by
  operand order, not by the `aMResFacesFence` mechanism, so the surviving orientation is
  always A's.
* **D4 — no `bAvoidIN` / `bAvoidINforBoth`.** There is no multi-solid interior-wall
  suppression, because there is no `myInParts` map: we have no face-vs-solid classification
  stage at all. A fused pair of touching operand solids keeps its shared wall.
* **D5 — the IN/OUT state is recomputed per face, not looked up.** `inside_v[fi]` comes from
  `vote_inside(fi, K, &score[fi])` (`src/brep.cpp:10009`-region) plus a chain of repairs:
  the angle method, the section repair, the connexity flood, the parity propagation and the
  island/hole repair (`src/brep.cpp:10028-10075`). Violates I-10. Each repair is a patch on
  the *previous* verdict rather than a single seeded-and-propagated state.
* **D6 — no `aMFToAvoid`.** There is no way to say "this face is excluded in every
  orientation", so a wall killed on one side can survive on the other.

### 4.7 Assembly: concatenate two subsets and hope

`src/brep.cpp:10216-10217` builds `subA = A2.subset(keptA)` and `subB = B2.subset(keptB)`;
`:10218-10219` flips `reversed` on the B faces marked by `revB`; `:10275` onward concatenates
the pools with index offsets and aliases the section edges by span key. There is **no**
shell construction, **no** closure step, **no** cavity nesting and **no** solid container at
the end. What comes out is a face-and-edge soup whose validity is later *measured* by
`is_solid()` (`src/brep.cpp:1063-1119`) — "every non-degenerate, non-orphan edge has exactly
two trims" — which cannot distinguish one solid from N interpenetrating closed shells, cannot
detect a globally inverted shell, and cannot detect a cavity attached to the wrong lump.

`src/brep.cpp:8218-8310` (the `SESSION_AUTO` ladder) is the compensating mechanism: it runs
the pipeline in several variants and picks the one with the fewest naked + non-manifold
edges, subject to a bounding-box containment check and a per-op volume sanity bracket. The
comment at `src/brep.cpp:8300-8307` is the honest statement of the gap: *shell count is
reported, never gated on*, because with the current architecture "several shells" cannot be
distinguished from "one open shell plus a chunk". Once §3 exists, that distinction is
structural and the ladder can gate on it.

### 4.8 Volume

`BRep::volume` (`src/brep.cpp:2172+`) integrates the divergence theorem with per-face outward
signs from `face_outward_signs`, so every defect of §4.2/§4.3 propagates into it. The
open-shell guard (`src/brep.cpp:2284-2320`) only *warns*, and only under `SESSION_VOL_GUARD`.
With §3 in place, volume must instead be computed **per solid**, with cavity shells
contributing negatively by construction, and must refuse to answer for a non-closed shell.

---

## 5. ACCEPTANCE TESTS

Every test states operands with an analytically known answer **and** the oracle-free
invariant it checks. No test compares against OCCT output.

### T1 — single box, identity assembly
Operands: `create_box(1,1,1)` alone through `BuilderSolid`.
Expect: 1 shell, `closed && manifold`, 6 faces, 1 solid, 0 hole shells, 0 alerts.
Invariants: I-2 (6 images in, 6 placed), I-3 (every edge parity 2), I-4, I-5
(`perform_infinite_point` = OUT ⇒ not a hole).
Analytic: volume 8.0 exactly.

### T2 — topology-only shell decomposition (proves I-1)
Take the T1 arena; replace every `m_curves_3d` entry with a straight segment between
arbitrary distinct points, leaving the topology tables untouched. Re-run
`make_connexity_blocks` + closure.
Expect: **bit-identical** shell decomposition to T1. Any dependence on coordinates fails
here. (This test is impossible to pass with `sew_coincident_edges` in the path — that is the
point.)

### T3 — box with a concentric box cavity
Outer box side 4 (volume 64), inner box side 2 (volume 8) supplied with all six faces
reversed.
Expect: 2 shells; the inner shell has `perform_infinite_point == IN` ⇒ `is_hole`; 1 solid
with `outer_shell` = the big one and one entry in `hole_shells`.
Analytic: volume 56.0 exactly.
Invariants: I-5 (classify the inner shell in isolation — same answer), I-6 trivially.

### T4 — box with a **spherical** cavity (the curved-shell version of T3)
Outer box side 4, inner sphere r = 1 reversed.
Expect: same structure as T3.
Analytic: `64 − 4π/3 = 59.8112...`; assert to 1e-9 with an exact quadric volume path, or to
the stated mesh tolerance with an explicitly reported error bound.
This is the test our current `comp_cavity` vote fails per the comment at
`src/brep.cpp:2050-2055` ("box-cut-inscribed-sphere read 97.5 = box + sphere instead of
30.5"); it must pass by construction, not by tuning.

### T5 — nested tower (proves I-6, innermost parent)
Box side 8 (512) ⊃ reversed box side 4 (cavity) ⊃ box side 2 (solid, 8).
Expect: 2 solids. Solid 1 = outer shell (side 8) + one hole shell (side 4), volume
512 − 64 = 448. Solid 2 = the side-2 box, volume 8.
Invariant: the cavity must attach to the side-8 solid, and the side-2 solid must **not**
receive it. Swap the construction order of the growth solids and assert the result is
unchanged (`BuilderSolid.cxx:517-524` is order-independent by the `IsInside(S, S_prev)`
refinement; a port that keeps first-found fails this).

### T6 — orphan cavity (proves I-7)
A single reversed box, no growth shell anywhere.
Expect: 1 solid whose shell is a cavity; `box.whole == true`; one `HoleWithoutParent` alert.
Nothing dropped.

### T7 — torus, genus 1 (proves closure ≠ Euler 2)
`create_torus(3, 1)`.
Expect: 1 shell, `closed == true`, `manifold == true`, χ = V − E + F = 0, not a hole.
Invariant: I-3 is edge parity; a port that asserts χ == 2 fails here.
Analytic: volume `2π²Rr² = 59.2176...`.

### T8 — two disjoint boxes fused (proves I-8)
Boxes at (0,0,0) and (10,0,0), both side 1.
Expect: 2 solids, each closed, volumes 8 and 8. No merging, no "repair toward 1".

### T9 — two boxes sharing exactly one face (the same-domain wall rows)
Boxes side 2 sharing the plane x = 1; the shared wall is ONE arena face index (P-1), present
with both orientations.
Run all four ops and assert against §2.10's table:
* FUSE → 1 solid, volume 16, the wall **dropped** (`bAvoidIN`);
* COMMON → empty result (the wall is not a solid);
* CUT (A−B) → 1 solid, volume 8, the wall **kept once with A's orientation**;
* CUT21 (B−A) → 1 solid, volume 8, the wall **kept once with B's orientation**.
The CUT vs CUT21 wall-normal asymmetry is the specific consequence of the
`aMResFacesFence` suppression (`BOPAlgo_Builder.cxx:700`); assert the normal direction, not
just the volume.

### T10 — partition disjointness (proves I-9, catches double booking)
For any A, B: compute `A\B`, `A∩B`, `B\A`. Build the multiset
`M_A = { (image, orientation) : used in A\B } ⊎ { … used in A∩B }` restricted to images whose
source is A, and likewise `M_B`.
Assert: **every image of A occurs exactly once in `M_A`**, and every image of B exactly once
in `M_B`, with the same-domain walls accounted for by the §2.10 table and nothing else.
This is the test the mission brief's counter-example needs: an OCCT reference that omits a
lump of `A\B` while double-booking it into `A∩B` satisfies
`vol(A\B) + vol(A∩B) + vol(B\A) = vol(A∪B)` but fails here.
Secondary assertion (cheap, keep both): `vol(A\B) + vol(A∩B) = vol(A)` and
`vol(A∩B) + vol(B\A) = vol(B)` — *separately*, not only their sum.

### T11 — grazing-ray battery
Box side 2. Classify 10 000 points on a lattice that includes points exactly on face planes,
exactly on edges, exactly at vertices, and points whose ray to every face-interior sample is
within 1e-9 of tangency to some other face.
Assert: (a) no `SolidClassifier::undecided()`; (b) the IN/OUT verdict is invariant under
rotating the whole model by 100 random rigid motions; (c) points on faces/edges/vertices
return `On`, not IN or OUT.
Then repeat with a sphere and a torus — this is where the mesh-based winding number of
`src/brep.cpp:1238-1268` is expected to break, and the exact intersector must not.

### T12 — the rotated 2-lump family (proves I-8 on real data)
The cells the campaign already measures as legitimately multi-solid: rotated-cylinder cut
from box at z30 / z45 / z37 / z63.
Assert: exactly 2 solids, both `closed && manifold`, `hole_shells` empty for both, and no
face image appears in more than one solid (I-2 across solids).
Do **not** assert against any oracle's solid count; assert the invariants.

### T13 — face conservation under selection
For every corpus cell: `|input images| == |selected| + |dropped by op table| + |reported
unused|`, with the dropped set explained by exactly one of the numbered steps of §2.10.
No image may be unaccounted for (I-2, I-13).

### T14 — provenance totality
For every cell, assert I-12: each input face/edge/vertex appears in exactly one of
`modified` (non-empty), `modified` (empty ⇒ deliberately deleted), `deleted`, or is present
verbatim in the result. Assert also that an absent `modified` key and an empty
`modified` vector are distinguishable (§3.6).

### T15 — determinism
Every test above, run 20× with a shuffled input face order.
Assert byte-identical shells, solids, selection and history. OCCT's only determinism anchor
is `std::sort` on DS indices (`Builder_2.cxx:687`); ours must sort on arena indices at the
same points — connexity block seeds, hole/growth iteration order, selection order.

---

## 6. IMPLEMENTATION ORDER

Smallest shippable increment first. Each step lands independently, behind `SESSION_V2`, and
is judged by the gate in its row. No step lands without the standing guards battery (base
chairs 3 ops exact, matrix 63 cells, edge 54/54, C++ minitests, A-op-A).

| step | deliverable | files | gate |
|---|---|---|---|
| **S0** | `Shell`, `Solid`, `OrientedFace`, `Box3`, `Alert` structs; `make_connexity_blocks`; `shell_is_closed` (edge parity); `shell_is_manifold`. Pure topology, no geometry. | new `src/brep_shell.h/.cpp`, new `src/brep_shell_split.h/.cpp` | **T1, T2, T7, T8**. T2 is the hard gate: the decomposition must be coordinate-independent. |
| **S1** | `orient_faces_on_shell` (relative orientation, seam-aware). Wire it in place of the *relative* half of `face_outward_signs`; leave the global flip alone for now. | `brep_shell_split.cpp` | I-4 holds on every corpus cell; base chairs and the matrix byte-identical. |
| **S2** | `SolidClassifier`: exact ray/face intersection (reuse `intersection.cpp`'s surface-ray path), the vertex/edge tolerance-box pre-test, `next_segment` with the `maxscal > 0.2` gate and the `myParamOnEdge` ladder, the `TopAbs_ON` retry loop, `NearFaultPar`. `undecided()` exposed. | new `src/brep_classify3d.h/.cpp` | **T11**. Also: agreement with the existing radial test on its 42-case set, and 0 `undecided()` on the whole corpus. |
| **S3** | `perform_infinite_point`; `IsHole`; `IsInside` via `compute_state(face,…)`; BVH nesting with the innermost-parent refinement; orphan-hole rule. `perform_areas` complete. Delete `comp_cavity`. | `brep_shell.cpp` | **T3, T4, T5, T6**. T4 is the gate that proves curved cavities work. |
| **S4** | `ShellSplitter::perform`: free-edge peeling, boundary-face preference, `get_face_off` + `AngleWithRef` + `GetFaceDir` + `FindPointInFace` + `MinStep3D`, `RefineShell`. `perform_shapes_to_avoid`. | `brep_shell_split.cpp` | corpus cells that currently report non-manifold edges (>2 trims) produce the right shell count with no ambiguity alerts; `AngularTieUnresolved` alerts counted and reported, never silent. |
| **S5** | `InPartsMap`: classify every face image against every solid **once**, in the split stage, via `SolidClassifier` + connexity-block batching (the `FillIn3DParts` structure). | new `src/brep_inparts.h/.cpp` | I-10: the selection step performs zero point classifications. Verified by an assertion counter. |
| **S6** | `select_faces` exactly as §3.5, both fences, both avoid rules, the CUT/CUT21 reversal, `toAvoid` unoriented. Replace `src/brep.cpp:9442-9451` and `:10019-10027`. Add a real CUT21. | new `src/brep_select.h/.cpp` | **T9, T10, T13**. A-op-A stays green. |
| **S7** | `perform_internal_shapes` + the unused-face alert; `BuilderSolid::perform` end-to-end; the boolean's result becomes `std::vector<Solid>` over one arena instead of a concatenated soup. Retire `sew_coincident_edges` from the boolean path. | `brep_shell.cpp`, `src/brep.cpp` boolean | **T12**; every corpus cell reports its solids explicitly; no face silently lost. |
| **S8** | `prepare_history`. | `brep_shell.cpp` | **T14**. |
| **S9** | Per-solid volume with cavity shells contributing negatively by construction; refuse to answer on a non-closed shell. Re-gate `SESSION_AUTO` on shell/solid structure instead of naked-edge count. | `src/brep.cpp` `volume` | analytic volumes for box/cyl/cone/sphere/torus exact to 1e-9; the four constant-factor errors named in the mission brief resolved. |
| **S10** | `orient_faces_on_shell` becomes the *only* orientation mechanism; delete the weighted vote at `src/brep.cpp:2040-2100` and the `SESSION_SHELL_ORIENT` override. | `src/brep.cpp` | **T15** plus the full standing battery non-decreasing. |

Ordering rationale: S0–S1 are pure topology and can land against the existing pipeline
without changing any result. S2–S3 replace classification, which is where the curved-surface
failures live, and are gated by analytic volumes rather than by any oracle. S4 is the only
step that needs the angular machinery, and it is needed only for non-manifold blocks — most
cells never reach it. S5–S6 are the selection rewrite and depend on S3 for the classifier.
S7 is the structural change to the result type. S8–S10 are cleanup that becomes safe only
once the structure is in place.

**Do not port**: G12 (failure reported as success), G13 unguarded (open shells as solids),
G14 (the off-by-one at `BOP.cxx:1139`). **Do port with an explicit alert**: G1, G2, G3, G5,
G8, G9, G10 — each is a real OCCT fallback and each must be visible in our output.
