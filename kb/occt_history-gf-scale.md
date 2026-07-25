# history-gf-scale

Implementation spec extracted from OCCT sources (read 2026-07-24) at
`C:/brg/compas_occt/external/occt/src/occt/src/ModelingAlgorithms/TKBO/{BOPAlgo,BOPDS,BOPTools,BRepAlgoAPI}`
and `ModelingData/TKBRep/BRepTools/BRepTools_History.hxx`.
Scope: n-ary General Fuse (GF) architecture, history maps (Generated/Modified/IsDeleted),
input validity analysis (ArgumentAnalyzer), self-intersection checking (CheckerSI),
BVH interference candidate iterator (BOPDS_Iterator), parallelization boundaries
(BOPTools_Parallel / BOPAlgo_ParallelAlgo), and the public API contract (BRepAlgoAPI).

---

## STAGE PIPELINE

The GF architecture is a strict two-phase design: **PaveFiller** (intersection; not in scope
here) fills a shared data structure `BOPDS_DS`; **Builder** (this spec) consumes it to build
splits ("images") of every argument sub-shape bottom-up by dimension, then result, then history,
then tolerance post-treatment. The API layer (BRepAlgoAPI) wraps both and owns the user contract.

### 0. API entry — argument collection and dispatch
- **Purpose**: collect n-ary arguments, dedupe, select builder, drive progress, expose history.
- **Where**:
  - `BRepAlgoAPI_BuilderAlgo::Build` — `BRepAlgoAPI_BuilderAlgo.cxx:81-101`. `NotDone(); Clear();` then `IntersectShapes(myArguments, 70%)` then `myBuilder = new BOPAlgo_Builder; SetArguments; BuildResult(30%)`.
  - `BRepAlgoAPI_BuilderAlgo::IntersectShapes` — `:107-134`. Creates `BOPAlgo_PaveFiller`, forwards options (`SetRunParallel`, `SetFuzzyValue`, `SetNonDestructive`, `SetGlue`, `SetUseOBB`), virtual `SetAttributes()` hook for FF-intersection options, `myDSFiller->Perform`, then `GetReport()->Merge(myDSFiller->GetReport())`.
  - `BRepAlgoAPI_BuilderAlgo::BuildResult` — `:138-164`. `myBuilder->SetCheckInverted; SetToFillHistory(myFillHistory); PerformWithFiller(*myDSFiller)`; merge report; on success `Done(); myShape = myBuilder->Shape();` and if history: `myHistory = new BRepTools_History; myHistory->Merge(myBuilder->History())`.
  - `BRepAlgoAPI_BooleanOperation::Build` — `BRepAlgoAPI_BooleanOperation.cxx:122-223`. Binary BOP wrapper over the SAME n-ary core: errors `BOPAlgo_AlertTooFewArguments` (empty args/tools) and `BOPAlgo_AlertBOPNotSet` (op unknown); concatenates `myArguments + myTools` into ONE list for intersection (`:174-193`); SECTION → `BOPAlgo_Section`, else `BOPAlgo_BOP` with `SetTools/SetOperation`; env `CSF_DEBUG_BOP` dumps failing args + DRAW script (`BRepAlgoAPI_DumpOper::Dump :229-330`).
- **Inputs/outputs**: TopoDS shapes → result compound + `BRepTools_History`.
- **When**: user-facing entry; intersection phase can be skipped entirely if a prebuilt `BOPAlgo_PaveFiller` was passed to the ctor (`myIsIntersectionNeeded=false`, `BRepAlgoAPI_BuilderAlgo.cxx:38-47`) — intersection is REUSABLE across operation types.

### 1. Builder setup — arguments, checks
- `BOPAlgo_Builder::AddArgument` — `BOPAlgo_Builder.cxx:103-109`: fence map `myMapFence.Add(theShape)` guarantees argument uniqueness in the n-ary list.
- `BOPAlgo_Builder::Perform` — `:167-194`: owns a fresh PaveFiller (`myEntryPoint=1` → Builder deletes it); `PerformWithFiller` — `:198-208`: borrows caller's filler (`myEntryPoint=0`) and COPIES its effective options back (`myNonDestructive/myFuzzyValue/myGlue/myUseOBB` from the filler — filler's options win).
- `BOPAlgo_Builder::PerformInternal` — `:212-227`: try/`OCC_CATCH_SIGNALS`/catch wrapper → any `Standard_Failure` becomes `BOPAlgo_AlertBuilderFailed` (never crashes the caller).
- `BOPAlgo_Builder::CheckData` — `:129-139`: n < 2 arguments → `BOPAlgo_AlertTooFewArguments`; `CheckFiller` — `:143-151`: no filler → `BOPAlgo_AlertNoFiller`, else `GetReport()->Merge(myPaveFiller->GetReport())` (intersection failures propagate as builder errors).
- `BOPAlgo_Builder::Prepare` — `:155-163`: result = empty compound.

### 2. PerformInternal1 — the ordered image-filling ladder
`BOPAlgo_Builder::PerformInternal1` — `BOPAlgo_Builder.cxx:310-446`. Grabs `myDS`, `myContext`
from the filler, then runs, aborting on `HasErrors()` after EVERY stage:
1. `FillImagesVertices` + `BuildResult(TopAbs_VERTEX)`
2. `FillImagesEdges` + `BuildResult(TopAbs_EDGE)`
3. `FillImagesContainers(TopAbs_WIRE)` + `BuildResult(WIRE)`
4. `FillImagesFaces` + `BuildResult(FACE)`
5. `FillImagesContainers(TopAbs_SHELL)` + `BuildResult(SHELL)`
6. `FillImagesSolids` + `BuildResult(SOLID)`
7. `FillImagesContainers(TopAbs_COMPSOLID)` + `BuildResult(COMPSOLID)`
8. `FillImagesCompounds` + `BuildResult(COMPOUND)`
9. `PrepareHistory`
10. `PostTreat`
Progress weights: `fillPIConstants` — `:278-289` (history = 5% of whole, PostTreat = 3%);
`fillPISteps` — `:293-306` (faces weighted ×20, solids ×50 per unit; vertices/edges/etc ×1);
normalization in `BOPAlgo_Algo::analyzeProgress` — `BOPAlgo_Algo.cxx:57-98`.

### 3. FillImagesVertices — vertex SD images
- `BOPAlgo_Builder_1.cxx:40-67`. Iterates `myDS->ShapesSD()` (map nV → nVSD computed by PaveFiller).
  Maintenance rule per pair: `myImages.Bound(aV)->Append(aVSD)`; `myShapesSD.Bind(aV, aVSD)`;
  `myOrigins(aVSD)` list appends aV (an SD vertex has MULTIPLE origins — origins is a list).

### 4. FillImagesEdges — edge splits from pave blocks
- `BOPAlgo_Builder_1.cxx:71-126`. For each source EDGE with initialized pave blocks:
  image list = `myDS->RealPaveBlock(aPB)->Edge()` shapes for every pave block. Key rules:
  - **Small edges having no pave blocks get an EMPTY image list** — "thus, will be avoided in the result" (comment `:92-94`). Empty list ≠ unbound: unbound = untouched, empty = deleted.
  - Common-block members: `if myDS->IsCommonBlockOnEdge(aPB)` then `myShapesSD.Bind(aSp, aSpR)` — the non-representative split is registered SD to the representative ("real") edge.
  - `myOrigins` appended per split (again a list — coincident edges from different args share one image).

### 5. BuildResult — per-type result assembly
- `BOPAlgo_Builder_1.cxx:130-168`. For each ARGUMENT (not source sub-shape) of the given type:
  unbound in `myImages` → add argument itself; else add each image; fence map dedupes.
  Result of GF = compound of images of all n arguments, type by type.

### 6. FillImagesContainers / FillImagesCompound — wires/shells/compsolids/compounds
- `FillImagesContainers` — `BOPAlgo_Builder_1.cxx:172-193`; `FillImagesContainer` — `:221-276`:
  **reuse-if-unchanged**: if no sub-shape has an image differing from itself, the container is NOT
  rebuilt (no image bound → passes through identity). Else new container built from splits; splits
  re-oriented via `BOPTools_AlgoTools::IsSplitToReverseWithWarn`; closedness recomputed
  (`aCIm.Closed(BRep_Tool::IsClosed(aCIm))`).
- `FillImagesCompound` — `:280-342`: recursive over nested compounds with fence map `theMFP`;
  child orientation is transplanted onto each image (`aSXIm.Orientation(aOrX)`).

### 7. FillImagesFaces = BuildSplitFaces → FillSameDomainFaces → FillInternalVertices
- `BOPAlgo_Builder::FillImagesFaces` — `BOPAlgo_Builder_2.cxx:215-229` (weights 9 / 0.5 / 0.5).
- `BuildSplitFaces` — `:233-555`:
  - Skip faces without `FaceInfo`; skip faces with no IN/ON-section/section pave blocks and no alone vertices (`:293-296` "not complete").
  - **Draft-face fast path** (`:298-351`): if no IN and no section edges — i.e. only boundary re-splits — and no INTERNAL wires and no modified wires: face passes unchanged; else `BuildDraftFace` (`:1052-1189`) rebuilds wires from edge images WITHOUT the full BuilderFace machinery. Draft bails out (returns null → full BuilderFace) on: INTERNAL edge (`:1105-1110`), multi-connected vertex (>2 edges at a vertex, `HasMultiConnected :1014-1045`), edge unification (same split appears twice in wires, `:1128,1149`).
  - Full path assembles the edge set `aLE`: bounding-edge images (INTERNAL edges appended twice FORWARD+REVERSED `:371-378`; degenerated keep orientation; **closed-face seam handling**: a split of a seam edge that lost its second pcurve gets `BOPTools_AlgoTools3D::DoSplitSEAMOnFace` with a fallback second attempt and warning `BOPAlgo_AlertUnableToMakeClosedEdgeOnFace` `:429-455`), IN pave-block edges twice-oriented (`:469-480`), section pave-block edges twice-oriented (`:484-494`); planar pcurve batching `BRepLib::BuildPCurveForEdgesOnPlane` only when destructive (`:496-500`).
  - Each face becomes a `BOPAlgo_SplitFace` task (subclass of `BOPAlgo_BuilderFace`) in a vector; run through `BOPTools_Parallel::Perform(myRunParallel, aVBF)` (`:521`); harvested serially: `aFacesIm.Add(myDS->Index(aBF.Face()), aBF.Areas())` and `myReport->Merge(aBF.GetReport())` (`:527-532`).
  - Images bound with the ORIGINAL face orientation re-applied to splits (`:534-552`).
- `FillSameDomainFaces` — `:580-925` (the SD subsystem):
  - Only faces involved in FF interference are candidates (`myDS->InterfFF()` drives it, `:584`).
  - **Same-solid exclusion** (`:596-649`): faces (and their split pieces, propagated through `myImages`) are mapped to their parent SOLID; two faces of one solid can never be SD — "that would imply a zero-thickness interior in a single operand" (comment `:596-597`).
  - Face indices collected and `std::sort`ed (`:687`) for deterministic representative election.
  - **Edge-set hashing**: `AddEdgeSet` (`:562-576`) keys each face/split by `BOPTools_Set` of its edges → map edge-set → faces. Only groups with ≥2 faces continue.
  - **Planar fast path** (`:707-718, 780-785`): a planar face with a finite bbox that shares the exact edge set with another planar candidate is SD WITHOUT the geometric `AreFacesSameDomain` check.
  - Remaining pairs go to parallel `BOPAlgo_PairOfShapeBoolean` tasks calling `BOPTools_AlgoTools::AreFacesSameDomain(aFj, aFk, ctx, myFuzzyValue)` (`:94-105`), context-per-thread via `BOPTools_Parallel::Perform(..., myContext)` (`:805`).
  - Positive pairs → union-find style `BOPAlgo_Tools::FillMap` + `MakeBlocks` (`:815-826`) → SD groups. Representative election (`:837-874`): the ORIGINAL face (still present in DS) with the MINIMAL DS index; an original face in a group gets a self-image `myImages(aF)={aF}` ("consider it being split", `:853-858`); every group member binds `myShapesSD(member)=representative`.
  - Image substitution pass (`:886-921`): every face image that has an SD entry is REPLACED in `myImages` by the representative, and `myOrigins(representative)` accumulates ALL source faces. This is what makes coincident faces of different arguments collapse to one face in the result.
- `FillInternalVertices` — `:929-1008`: alone vertices (`myDS->AloneVertices`) classified against every split via parallel `BOPAlgo_VFI` tasks (`myContext->ComputeVF(myV, myF, ..., myFuzzyValue)`, internal iff flag==0 `:198-199`); positives added to faces with orientation INTERNAL.

### 8. FillImagesSolids = FillIn3DParts → BuildSplitSolids → FillInternalShapes
- `BOPAlgo_Builder::FillImagesSolids` — `BOPAlgo_Builder_3.cxx:60-93` (weights 4/5/1); early-out if no SOLID among source shapes.
- `FillIn3DParts` — `:97-263`:
  - Collect ALL faces (splits where images exist, originals + cached `aSI.Box()` otherwise).
  - Per solid: lazily `myDS->BuildBndBoxSolid(i, aBoxS, myCheckInverted)` (`:179-183`) — this is where `myCheckInverted` (holes-in-space solids) is honored; `BuildDraftSolid` (`:267-368`) rebuilds shells from face images: SD-bound splits are re-oriented via `IsSplitToReverseWithWarn` before adding (`:311-331`), INTERNAL faces are set aside into `theLIF`, shells keep only flagged (non-empty) content, closedness recomputed (`:364`).
  - `BOPAlgo_Tools::ClassifyFaces(aLFaces, aLSolids, myRunParallel, myContext, anInParts, aShapeBoxMap, aSolidsIF)` (`:201-208`) classifies every candidate face against every draft solid (box-filtered) → IN lists.
  - Results (`:211-262`): a solid with no IN faces AND unmodified shells is skipped entirely (identity pass-through); else `theDraftSolids.Bind(solid, draft)` and `myInParts.Bound(solid)` ← IN faces + own INTERNAL faces. **`myInParts` is the persistent classification table later reused by `BuildBOP`** (see stage 12).
- `BuildSplitSolids` — `:413-618`:
  - Phase 0 (`:432-461`): every NON-interfered solid registers its `BOPTools_Set` of faces into `aMST` — the SD-solid detection set.
  - Phase 1: for each interfered solid, shell-face set = draft solid faces + IN faces twice-oriented (`:491-511`); parallel `BOPAlgo_SplitSolid` (subclass of `BOPAlgo_BuilderSolid`) tasks (`:513-517, 532`).
  - Harvest (`:539-577`): BuilderSolid ERRORS are DOWNGRADED to warnings on the main report, wrapped with the offending solid in a compound (`TopoDS_AlertWithShape`, comment `:544-547`).
  - Image/SD/origin maintenance (`:580-617`): each result solid is keyed by its face-set; `aMST.Added(aST)` dedupes: if an identical face-set solid already exists (from another argument), the EXISTING one becomes the image and `myShapesSD.Bind(aSR, aSx)` records the SD-solid relation; `myOrigins(aSx)` accumulates all source solids.
- `FillInternalShapes` — `:622-887`: pushes free vertices/edges/wires from the arguments and internal sub-shapes of source solids into the split solids they now fall inside: candidate set built from args + `OwnInternalShapes` (`:891-905`, everything directly under a solid that is not a SHELL); shapes already tied to split faces/edges excluded via ancestor maps (`:790-809`); classification `BOPTools_AlgoTools::ComputeStateByOnePoint(aSI, aSd, 1.e-11, myContext)` (`:836`); an untouched original solid that receives an internal gets a NEW solid image (copy + add INTERNAL, `:844-869`) so inputs are never mutated (non-destructive contract at solid level).

### 9. PrepareHistory — the history stage
- `BOPAlgo_Builder::PrepareHistory` — `BOPAlgo_Builder_4.cxx:164-252`; guarded by `HasHistory()` (`myFillHistory`, default TRUE — `BOPAlgo_BuilderShape.hxx:122`).
- `myHistory = new BRepTools_History`; `myMapShape` ← `TopExp::MapShapes(myShape)` (result membership filter).
- Per source shape (only `BRepTools_History::IsSupportedType` = VERTEX/EDGE/FACE/SOLID — `BRepTools_History.hxx:145-150`):
  - **Modified** (`:204-231`): splits from `LocModified(aS)` (= `myImages.Seek(theS)` — `:157-160`, virtual so CellsBuilder-style ops can redefine) that are CONTAINED IN THE RESULT; orientation fixed before recording: VERTEX/SOLID inherit source orientation, EDGE/FACE via `BOPTools_AlgoTools::IsSplitToReverse` → `myHistory->AddModified(aS, aSp)`.
  - **Generated** (`:233-243` + `LocGenerated` `:27-153`): only EDGE and FACE can generate. Scans `myDS->InterfEE()` and `myDS->InterfEF()` for interferences that `Contains(nS)` and `HasIndexNew()` → the new vertex (SD-resolved via `myDS->HasShapeSD(nVNew,nVNew)`, fence map deduped) is Generated IF in result (`:80-114`). For FACE additionally: `myDS->FaceInfo(nS).PaveBlocksSc()` section edges and `.VerticesSc()` section vertices, filtered by result membership (`:120-151`). EE interference scan is skipped for faces (`:78-80`).
  - **Deleted** (`:245-250`): `!isModified && !myMapShape.Contains(aS)` → `myHistory->Remove(aS)`. A shape can be Deleted AND have Generated elements (BRepTools_History contract).
- API surface: `BOPAlgo_BuilderShape::Modified/Generated/IsDeleted/HasModified/HasGenerated/HasDeleted/History` — `BOPAlgo_BuilderShape.hxx:52-110`; `History()` returns a fresh empty history if the algorithm failed before filling (`:96-103`), and NULL if filling was disabled ("may be partially filled for internal needs", `:105-109`).
- `BRepTools_History` axioms (`BRepTools_History.hxx:27-88`): G(S)∩M(S)=∅; Removed ⇒ not an output and M(S)=∅ (Generated from removed IS allowed); every output is an input or generated/modified from one; merge laws for chained histories H13=H23∘H12: `G12→(G23∪M23)=G13`, `M12→G23=G13`, `M12→M23=M13`.

### 10. PostTreat — tolerance correction of the result
- `BOPAlgo_Builder::PostTreat` — `BOPAlgo_Builder.cxx:450-475`.
- In non-destructive mode, all source V/E/F are collected into `aMA` (MapToAvoid) so their stored tolerances are never touched (`:455-469`).
- `BOPTools_AlgoTools::CorrectTolerances(myShape, aMA, 0.05, myRunParallel)` then `CorrectShapeTolerances(myShape, aMA, myRunParallel)` — validates/raises tolerances of result sub-shapes so that geometry containment invariants (curve within edge tol, edge within face tol, vertex covers edge ends) hold; 0.05 is the max correction cap (declaration `BOPTools_AlgoTools.hxx:439-471`, default cap otherwise 0.0001).

### 11. Input validity analysis — BOPAlgo_ArgumentAnalyzer
- `BOPAlgo_ArgumentAnalyzer::Perform` — `BOPAlgo_ArgumentAnalyzer.cxx:130-257`. Whole battery in try/`OCC_CATCH_SIGNALS`; any escape → single `BOPAlgo_CheckUnknown` result. Ten toggleable checks (`bool&` mode accessors, `BOPAlgo_ArgumentAnalyzer.hxx:59-91`), `StopOnFirstFaulty()` short-circuits. Result: list of `BOPAlgo_CheckResult` each carrying (shape1, shape2, faulty-subshape lists, `BOPAlgo_CheckStatus`).
  1. `Prepare` (`:115-126`): emptiness via `BOPTools_AlgoTools3D::IsEmptyShape`.
  2. `TestTypes` (`:275-352`): null/empty args → `BOPAlgo_BadType`; **dimension compatibility table** (`:330-350`) via `BOPTools_AlgoTools::Dimensions` (min/max dim of each arg): FUSE requires homogeneous equal dims both sides; CUT requires `iDimMax[0] <= iDimMin[1]`; CUT21 mirrored; COMMON accepts anything.
  3. `TestSelfInterferences` (`:356-445`): per argument, runs a full `BOPAlgo_CheckerSI` (`SetNonDestructive(true)`, level default). Each interference pair whose BOTH shapes are original (`!aDS.IsNewShape`) becomes a `BOPAlgo_SelfIntersect` result; a CheckerSI hard error becomes `BOPAlgo_OperationAborted`.
  4. `TestSmallEdge` (`:449-567`): every non-degenerated edge with `BOPTools_AlgoTools::IsMicroEdge(anEdge, aCtx)` → `BOPAlgo_TooSmallEdge`; SECTION-only refinement (`:480-539`): result kept only if a vertex of the micro edge lies ON the other shape within summed tolerances (`BRepExtrema_DistShapeShape` + support-shape tol accumulation) — a floating micro edge far from the tool is harmless for SECTION.
  5. `TestRebuildFace` (`:571-672`, skipped for SECTION/UNKNOWN): re-runs `BOPAlgo_BuilderFace` on each face's own edges (INTERNAL edges duplicated F+R); face is `BOPAlgo_NonRecoverableFace` if the rebuild yields ≠1 area or consumes a different edge count than the input.
  6. `TestTangent` (`:676-679`): **not implemented** (documented placeholder).
  7/8. `TestMergeVertex`/`TestMergeEdge` → `TestMergeSubShapes` (`:683-878`): cross-argument coincidence matrix; vertices equal iff `dist <= tol(V1)+tol(V2)` (`:766-776`); edges equal iff `IntTools_EdgeEdge` finds a common part of type EDGE (`:777-800`); faces "not yet implemented" (`:801-804`). A sub-shape of one argument matching **more than one** (`nbs > 1`) of the other → `BOPAlgo_IncompatibilityOfVertex/Edge` — checked in BOTH directions (`:752-877`). (Exactly one match is fine — the fuzzy/SD machinery will merge it.)
  9. `TestContinuity` (`:896-958`): edges/faces with `GeomAbs_C0` continuity → `BOPAlgo_GeomAbs_C0` warning-class result.
  10. `TestCurveOnSurface` (`:962-1015`): per face×edge, `BOPTools_AlgoTools::ComputeTolerance(aF, aE, aD, aT)`; if actual 3D-curve↔pcurve-on-surface deviation `aD > BRep_Tool::Tolerance(aE)` → `BOPAlgo_InvalidCurveOnSurface` with max distance and parameter recorded.
- Status enum: `BOPAlgo_CheckStatus.hxx:18-32` (CheckUnknown, BadType, SelfIntersect, TooSmallEdge, NonRecoverableFace, IncompatibilityOfVertex/Edge/Face, OperationAborted, GeomAbs_C0, InvalidCurveOnSurface, NotValid).

### 12. Self-intersection checker — BOPAlgo_CheckerSI
- `BOPAlgo_CheckerSI` derives from `BOPAlgo_PaveFiller` (`BOPAlgo_CheckerSI.hxx:36`); ctor (`BOPAlgo_CheckerSI.cxx:102-108`): `myLevelOfCheck = BOPDS_DS::NbInterfTypes()-1` (=9, all), `myNonDestructive = true` (forced), `SetAvoidBuildPCurve(true)` (no pcurve construction — check only).
- Level ladder (`.hxx:47-59`): 0=V/V … 5=+F/F, 6=+V/S, 7=+E/S, 8=+F/S, 9=+S/S.
- `Init` (`:129-148`): builds own DS from the ONE argument, `IntTools_Context`, and a `BOPDS_IteratorSI` (`Prepare(ctx, myUseOBB, myFuzzyValue)`, then `UpdateByLevelOfCheck` truncates candidate lists above the level).
- `Perform` (`:152-209`): exactly one argument or `BOPAlgo_AlertMultipleArguments`; run base `BOPAlgo_PaveFiller::Perform` (sub-shape intersections V/V…F/F); then `CheckFaceSelfIntersection`; then solid stages `PerformVZ/EZ/FZ/ZZ`; then `PostTreat`.
- `CheckFaceSelfIntersection` — `:413-508` (level ≥ 5): a FACE vs ITSELF `IntTools_FaceFace` run, parallel over faces (`BOPAlgo_FaceSelfIntersect : IntTools_FaceFace + BOPAlgo_ParallelAlgo`, `:42-92`). **Surface-type filter** (`:441-458`): Plane/Cylinder/Cone/Sphere skipped (cannot self-intersect); Torus skipped only when `MajorRadius > MinorRadius + Precision::Confusion()` (a spindle/self-crossing torus IS checked). Any resulting curve or point → pair (nF,nF) recorded.
- Solid stages — `BOPAlgo_CheckerSI_1.cxx`: `PerformVZ` (`:237-319`) classifies SD-resolved vertices against solids via cached `myContext->SolidClassifier`, state IN → `InterfVZ` (+`myDS->AddInterf`); pairs with existing sub-shape interference skipped (`HasInterfShapeSubShapes`, `:265-267`); `PerformSZ` (`:401-479`, shared by EZ/FZ) and `PerformZZ` (`:337-397`) mark interference only when SUB-shape interference records already exist between the entities — pure derived roll-ups, all parallel-vector based.
- `PostTreat` — `BOPAlgo_CheckerSI.cxx:213-409`: folds every typed interference array (VV,VE,EE,VF,EF,FF,VZ,EZ,FZ,ZZ) into `myDS->Interferences()` as normalized `BOPDS_Pair`s, SKIPPING pairs involving new (intersection-born) shapes for the vertex/edge types; EF with common part `TopAbs_SHAPE` skipped (`:286-288`); FF pair counts as self-intersection iff n1==n2, or tangent faces confirmed same-domain (`AreFacesSameDomain`, `:323-332`), or some section curve actually carries pave blocks (`:335-345` — raw curves without pave blocks are noise).

### 13. Interference candidate iterator — BOPDS_Iterator (BVH)
- Ctor — `BOPDS_Iterator.cxx:79-131`: one pair-list per interference type (`BOPDS_DS::NbInterfTypes()`), plus `NbExtInterfs()==4` extra lists (V/V, V/E, V/F; E/E "initialized but never filled" — `BOPDS_Iterator.hxx:104-107`).
- `Prepare` — `:247-265`: clears lists, calls virtual `Intersect`.
- `Intersect` — `:270-359` (the candidate engine):
  1. Build `BOPTools_BoxTree` (BVH) of the AABBs (`aSI.Box()`) of every source sub-shape with BRep (`:277-291`).
  2. `BOPTools_BoxPairSelector` with `SetBVHSets(&tree,&tree); SetSame(true); Select(); Sort();` — self-join of the tree, sorted pair output (`:294-298`).
  3. Walk pairs against DS argument RANGES (`myDS->NbRanges()`, each range = the index interval of one argument's sub-shapes): `if (!aRange.Contains(ID1)) break;` advances range (pairs sorted by ID1); **`if (aRange.Contains(ID2)) continue;` — a pair inside ONE argument is discarded** (`:311-324`). This is the load-bearing n-ary rule: GF only intersects across arguments, never within one (single-argument validity is the ArgumentAnalyzer/CheckerSI's job).
  4. Skip shape-vs-own-subshape (`aSI1.HasSubShape(ID2)` by type order, `:335-339`).
  5. Optional OBB rejection: `theCtx->OBB(shape, theFuzzyValue)` per shape, `anOBB1.IsOut(anOBB2)` → skip (`:342-352`).
  6. Classify by type pair `BOPDS_Tools::TypeToInteger(aType1,aType2)` and append normalized `BOPDS_Pair(min,max)` (`:354-356`).
- `Initialize(type1,type2)` — `:192-208`: selects list (ext list if `myUseExt` and applicable), `std::stable_sort(aPairs.begin(), aPairs.end())` — "sort interfering pairs for constant order of intersection" — determinism guarantee; `Value` — `:226-243` returns the pair with the HIGHER-type shape first (e.g. always (edge, vertex) not (vertex, edge)).
- `BlockLength` — `:174-188`: `0.5 * ExpectedLength()` heuristic for allocator increments.
- `IntersectExt(theIndices)` — `:363-463`: SECOND-pass candidate search for sub-shapes whose tolerance was INCREASED during the operation: rebuilds the tree using SD-resolved boxes for the increased set, parallel BVH selection via `BOPDS_TSR` selector tasks (`:35-73`, `BOPTools_Parallel::Perform`), then keeps only CROSS-RANK pairs (`myDS->Rank(i) != Rank(j)`, `:434-438`), sub-shape-filtered, fence-map deduped, appended to the ext lists (only types < 4); sets `myUseExt=true`.
- `BOPDS_IteratorSI::Intersect` — `BOPDS_IteratorSI.cxx:60-129`: SAME BVH engine but NO range filter (self-pairs wanted), filter `aSI.IsInterfering()`; `UpdateByLevelOfCheck` — `:47-56` clears lists above the requested level.

### 14. Parallel execution substrate — BOPTools_Parallel / BOPAlgo_ParallelAlgo
- `BOPTools_Parallel` — `BOPTools_Parallel.hxx:26-185`; three functors:
  - `Functor` (`:28-51`): plain `solvers[i].Perform()` over an index range via `OSD_Parallel::For(0, n, functor, !runParallel)` — the bool is "run serial".
  - `ContextFunctor` (`:54-109`): thread-id → `IntTools_Context` handle map under `std::mutex`; each task gets `SetContext(threadLocalCtx)` before `Perform()` — contexts (classifier/projector caches) are NEVER shared between threads; main thread's context is pre-bound so serial runs reuse the caller's caches.
  - `ContextFunctor2` (`:112-152`): OCCT thread-pool variant, per-thread-index context array; `UpperThreadIndex` reserved for the main thread (comment `:127-129`).
- `BOPAlgo_ParallelAlgo` — `BOPAlgo_Algo.hxx:87-105`: base for all vector tasks; carries a pre-assigned `Message_ProgressRange` (`SetProgressRange`) because progress scopes cannot be created concurrently; range-based `Perform(range)` is disabled.
- **Parallelization boundaries in this subsystem** (all fork-join, harvest serial): face SD pair checks (`Builder_2.cxx:805`), face splitting `BuilderFace` vector (`:521`), vertex-in-face classification (`:990`), solid splitting `BuilderSolid` vector (`Builder_3.cxx:532`), face-vs-solids classification (inside `BOPAlgo_Tools::ClassifyFaces`), CheckerSI face self-intersection (`CheckerSI.cxx:479`), V/S–S/S stages (`CheckerSI_1.cxx:299,372,438`), ext-box BVH selection (`BOPDS_Iterator.cxx:407`). Global switch: `BOPAlgo_Options::SetParallelMode` (static `myGlobalRunParallel`, `BOPAlgo_Options.cxx:26,91-101`) seeds per-instance `myRunParallel`.

### 15. Options / report / user-break substrate — BOPAlgo_Options
- `BOPAlgo_Options.hxx:36-148`: allocator, `Message_Report` (AddError=Message_Fail, AddWarning=Message_Warning, HasErrors/HasWarnings/typed queries, Dump), `myRunParallel`, `myFuzzyValue`, `myUseOBB`.
- `SetFuzzyValue` — `BOPAlgo_Options.cxx:105-108`: `myFuzzyValue = max(theFuzz, Precision::Confusion())` — fuzzy can never drop below 1e-7.
- `UserBreak(scope)` — `:110-118`: polls `Message_ProgressScope::UserBreak()`, converts to `BOPAlgo_AlertUserBreak` error. Called after nearly every loop iteration in all stages — cancellation points are pervasive and every stage exits cleanly.

---

## DATA STRUCTURES

- **`BOPAlgo_Builder` core maps** (`BOPAlgo_Builder.hxx:491-506`):
  - `myArguments : List<Shape>` — n-ary operands, order preserved; `myMapFence` enforces uniqueness.
  - `myImages : DataMap<Shape, List<Shape>>` — sub-shape → its splits (the single source of truth for Modified history, container rebuilds, draft solids, BuildBOP). States: UNBOUND = untouched (identity); EMPTY LIST = vanished (micro edge); list = splits, WITH SD substitution already applied for faces/solids.
  - `myShapesSD : DataMap<Shape, Shape>` — shape → same-domain representative (vertices, common-block edges, SD faces, equal-face-set solids). Consulted by `BuildDraftSolid` to know a split needs orientation checking.
  - `myOrigins : DataMap<Shape, List<Shape>>` — image → ALL source shapes it came from (list length >1 exactly when SD merged coincident geometry across arguments).
  - `myInParts : DataMap<Shape, List<Shape>>` — solid → own INTERNAL faces + faces of OTHER arguments classified IN it; computed once in `FillIn3DParts`, reused by `BuildSplitSolids` and `BuildBOP` (avoids reclassification per operation type).
  - `myPaveFiller/myDS/myContext` — borrowed intersection machinery; `myEntryPoint` controls filler ownership/deletion (`BOPAlgo_Builder.cxx:79-86`).
  - Options: `myNonDestructive`, `myGlue : BOPAlgo_GlueEnum`, `myCheckInverted`.
- **`BOPAlgo_BuilderShape`** (`BOPAlgo_BuilderShape.hxx:142-150`): `myShape` (result), `myHistShapes` (scratch return list), `myMapShape` (result membership map — filled in PrepareHistory, reused by LocGenerated), `myFillHistory` (default true), `myHistory : handle<BRepTools_History>`.
- **`BRepTools_History`** (`BRepTools_History.hxx:244-259`): `myShapeToModified`, `myShapeToGenerated` (both Shape→List, never bound to empty lists), `myRemoved : Map`. Supported types V/E/F/SOLID only. Class invariants in header `:27-68` (see INVARIANTS).
- **`BOPDS_Iterator`** (`BOPDS_Iterator.hxx:117-129`): `myLists : Array<Array<BOPDS_Pair>>` indexed by interference-type integer; `myExtLists` (4, tolerance-growth second pass); `myUseExt`; `myLength` (expected pair count of current type); iterator over the stable-sorted pair array. `BOPDS_Pair` = normalized (min,max) index pair with hash.
- **`BOPAlgo_CheckResult`** (used by ArgumentAnalyzer): shape1/shape2 anchors, faulty-subshape lists per side, `BOPAlgo_CheckStatus`, max distance/parameter for curve-on-surface faults.
- **`BOPAlgo_PISteps`** (`BOPAlgo_Algo.hxx:109-146`): per-operation progress weight array; constants normalized against variable steps in `analyzeProgress`.
- **Parallel task vectors**: `BOPAlgo_VectorOfPairOfShapeBoolean` (SD checks), `BOPAlgo_VectorOfBuilderFace`, `BOPAlgo_VectorOfBuilderSolid`, `BOPAlgo_VectorOfVFI`, `BOPAlgo_VectorOfFaceSelfIntersect`, `BOPAlgo_VectorOfVertexSolid/ShapeSolid/SolidSolid` — all `NCollection_DynamicArray` of value-type task objects; results read back serially by index.
- **`BOPTools_Set`** — order-independent hashed set of sub-shapes (edges of a face / faces of a solid); equality = same sub-shape set; THE key for SD face candidates and SD solid detection.
- **`BRepAlgoAPI_BuilderAlgo`** (`BRepAlgoAPI_BuilderAlgo.hxx:222-243`): `myArguments`, options (`myNonDestructive/myGlue/myCheckInverted/myFillHistory`), `myIsIntersectionNeeded` (external-filler reuse), `myDSFiller`, `myBuilder`, `myHistory`, `mySimplifierHistory` (kept separately so `SectionEdges` can re-map section edges through post-simplification — `BRepAlgoAPI_BuilderAlgo.cxx:290-319`).

---

## CONSTANTS & TOLERANCES

- `Precision::Confusion()` = 1e-7 — default fuzzy value (`BOPAlgo_Options.cxx:53,65`); hard floor of `SetFuzzyValue` (`:107`); torus self-intersection threshold `MajorRadius > MinorRadius + Precision::Confusion()` (`BOPAlgo_CheckerSI.cxx:454`); default fuzzy of `BOPDS_Iterator::Prepare/Intersect` signatures (`BOPDS_Iterator.hxx:84,115`).
- `BOPAlgo_FaceSelfIntersect::myTolF` default 1.e-7 (`BOPAlgo_CheckerSI.cxx:51`), overwritten by `BRep_Tool::Tolerance(aF)` (`:460-467`).
- PostTreat tolerance-correction cap: `0.05` passed as `theTolMax` to `CorrectTolerances` (`BOPAlgo_Builder.cxx:472`); declaration default is `0.0001` (`BOPTools_AlgoTools.hxx:448`) — Builder deliberately allows 500× larger correction.
- `FillInternalShapes` point-in-solid tolerance: `1.e-11` (`BOPAlgo_Builder_3.cxx:836`).
- Vertex coincidence in `TestMergeSubShapes`: `dist <= Tol(V1)+Tol(V2)` (`BOPAlgo_ArgumentAnalyzer.cxx:772`); small-edge-on-shape test accumulates support-shape tolerance (`:504-525`).
- Curve-on-surface validity: fault iff computed deviation `aD > BRep_Tool::Tolerance(edge)` (`:986-989`) — the edge tolerance IS the contract.
- Progress model: history 5%, PostTreat 3% of the whole (`BOPAlgo_Builder.cxx:284-288`); face step weight ×20, solid ×50 (`:301,303`); intersection:building split = 70:30 at API level (`BRepAlgoAPI_BuilderAlgo.cxx:89,100`), 9:1 inside `BOPAlgo_Builder::Perform` (`BOPAlgo_Builder.cxx:190-193`).
- `BOPDS_Iterator::BlockLength` prediction coefficient `aCfPredict = 0.5` (`BOPDS_Iterator.cxx:177`).
- `BOPDS_Iterator::NbExtInterfs() = 4` (`BOPDS_Iterator.hxx:107`).
- CheckerSI default level = `BOPDS_DS::NbInterfTypes()-1` = 9 (`BOPAlgo_CheckerSI.cxx:105`).
- `SimplifyResult` defaults: `theAngularTol = Precision::Angular()` (1e-12 rad), linear tolerance = operation fuzzy value (`BRepAlgoAPI_BuilderAlgo.cxx:183-189`).

---

## INVARIANTS

1. **Two-phase separation**: intersection results (DS) are immutable during building; the Builder never adds interferences. Any algorithm downstream may re-consume the same PaveFiller (`PerformWithFiller`) — enables computing intersection ONCE for cut+common+fuse+section.
2. **Bottom-up image completeness**: when `FillImages<T>` runs, images of ALL lower-dimensional sub-shapes are final. Face splitting may assume final edge images; solid splitting may assume final face images (incl. SD substitution).
3. **Image-map states are tri-valued** and downstream code relies on it: unbound = identity (shape reusable as-is), empty = deleted, non-empty = replaced by exactly those splits.
4. **SD substitution happens inside myImages**: after `FillSameDomainFaces`/`BuildSplitSolids`, no two distinct shapes in any image list are geometrically coincident; the result compound contains ONE representative per coincident group and `myOrigins` fans back out. Cross-argument only — same-solid pairs excluded by construction.
5. **Orientation contract**: every split placed into a container/shell/history is re-oriented to match its source (`IsSplitToReverse`); VERTEX/SOLID orientation copied verbatim (`Builder_4.cxx:217-224`). Failure is a WARNING (`BOPAlgo_AlertUnableToOrientTheShape`), not an error.
6. **Candidate iterator determinism**: pair lists stable-sorted before iteration (`BOPDS_Iterator.cxx:203`); SD face representative = minimal DS index among originals (`Builder_2.cxx:843-866`); face index vector sorted (`:687`) — identical inputs give identical results regardless of BVH traversal order.
7. **Cross-argument-only intersection**: `Intersect` discards same-range pairs; `IntersectExt` discards same-rank pairs. A single argument is ASSUMED self-intersection-free — that is exactly what ArgumentAnalyzer/CheckerSI verify beforehand.
8. **History axioms** (enforced by BRepTools_History, asserted via messages `myMsg*`): Generated∩Modified=∅ per source; Removed ⇒ no Modified and not itself an output; only V/E/F/SOLID tracked; history lists only contain shapes PRESENT IN THE RESULT (Builder filters through `myMapShape`).
9. **Error/warning discipline**: errors abort the ladder at the next stage boundary; sub-algorithm (BuilderSolid) errors are demoted to warnings with the offending solid attached; every catch-block converts crashes to alerts; report is merged upward at every layer (BuilderFace → Builder → API).
10. **Non-destructive contract**: with `myNonDestructive`, input V/E/F tolerances are excluded from PostTreat updates (MapToAvoid) and input solids are copied before receiving INTERNAL sub-shapes; result may share unmodified sub-shapes with inputs (identity images).
11. **Parallel safety**: task objects are value types in vectors, write only their own fields; `IntTools_Context` is per-thread; DS and maps are read-only during parallel sections; harvest and report merging are serial.
12. **API contract** (what industry code relies on): `Build()` then `IsDone()`; `Shape()`; `Modified/Generated/IsDeleted` non-throwing for ANY input shape (empty list/false for unknown); `SectionEdges()` = edges from FF interference curves; `SimplifyResult` merges its own history so Modified chains stay correct; setting a filler in the ctor skips re-intersection; fuzzy/glue/OBB/parallel/history toggles are pre-Build options.

---

## PITFALLS

(corner cases the source explicitly handles — each with the guarding code)

- **Small edges silently vanish**: an edge whose pave blocks were all removed gets an EMPTY image list on purpose so it disappears from the result but still reports as Deleted in history (`Builder_1.cxx:92-95`).
- **Seam/closed-edge splits losing the second pcurve**: `DoSplitSEAMOnFace` with a second fallback signature and a warning compound if both fail (`Builder_2.cxx:429-447`); iso-line + closed-surface detection precondition (`GeomLib::IsClosed` + `IsEdgeIsoline`, `:387-404`).
- **Draft-face fast path bail-outs**: INTERNAL edges (may split a face into halves), multi-connected vertices ("thin face split by a vertex", case `bug25245_1`, `:1068-1073`), unified edges appearing twice — all force the full BuilderFace (`:1105-1151`).
- **INTERNAL edges/faces are double-inserted FORWARD+REVERSED** everywhere a region-builder consumes them (`Builder_2.cxx:371-378, 420-427`; ArgumentAnalyzer TestRebuildFace `:606-613`; IN faces in `BuildSplitSolids` `:502-511`) — a membrane bounds regions on both sides.
- **Two faces of one solid must never be SD**: parent-solid exclusion incl. propagation to split pieces ("zero-thickness interior", `Builder_2.cxx:596-649`).
- **Planar-face SD shortcut requires a FINITE box**: open (infinite) planar faces with equal edge sets are NOT auto-SD (`:713-717`).
- **SD representative must be an original when possible**, with a self-image bound for originals ("Such face does not have any splits, but have an SD face. Consider it being split.", `:853-858`) — otherwise history would lose the Modified link.
- **BuilderSolid failure is not fatal**: errors → warnings wrapped with the solid (`Builder_3.cxx:544-576`); an argument that fails to split still flows through as draft.
- **Inverted solids (holes in space)**: `BuildBndBoxSolid(..., myCheckInverted)` (`Builder_3.cxx:182`); disabling documented as "most likely will lead to incorrect results" (`BOPAlgo_Builder.hxx:56-59`).
- **Untouched solid receiving internals is copied, not mutated** (`Builder_3.cxx:844-869`) — `aMSOr` tracks originals, removal from it after copy.
- **Generated vertices need SD resolution** before dedup/report: `myDS->HasShapeSD(nVNew, nVNew)` (`Builder_4.cxx:98`).
- **History for shapes untouched but absent**: Deleted = not-modified AND not-in-result — a shape passed through identity into the result is neither Modified nor Deleted.
- **`History()` after failure returns an EMPTY history, not NULL**; NULL only when filling was disabled (may be "partially filled for internal needs", `BOPAlgo_BuilderShape.hxx:96-109`).
- **CheckerSI FF noise suppression**: raw section curves without pave blocks do NOT count as self-intersection; tangent faces only count if geometrically same-domain; EF interference with common part `TopAbs_SHAPE` skipped (`CheckerSI.cxx:286-288, 304-346`).
- **Degenerate torus**: only `Major <= Minor + Confusion` tori are self-int checked (`:449-458`).
- **CheckerSI forces `NonDestructive=true`** in ctor — the check must never touch the model (`:106`).
- **Micro-edge relevance for SECTION**: a too-small edge is only reported if it actually touches the other shape (tolerance-summed distance test, `ArgumentAnalyzer.cxx:480-539`).
- **CUT dimension asymmetry**: `CUT` faults iff `maxDim(obj) > minDim(tool)`; `CUT21` mirrored; FUSE requires homogeneous dims (`:336-341`).
- **`TestTangent` and face-merge check are stubs** — do not port expectations onto them (`:676-679, 801-804`).
- **Iterator ranges assume pair list sorted by ID1** (`Select(); Sort();` before the range walk) — the `break`/`continue` range logic is wrong without the sort (`BOPDS_Iterator.cxx:296-323`).
- **IntersectExt uses the SD image's box, not the original's** (`:384-396`) and only refreshes V/V, V/E, V/F (E/E list exists but is never filled — header comment `BOPDS_Iterator.hxx:105-106`).
- **Filler options override Builder options** in `PerformWithFiller` (`BOPAlgo_Builder.cxx:198-208`) — option state must come from the object that actually intersected.
- **BuildBOP SD-face orientation logic** (`BOPAlgo_Builder.cxx:634-736`): duplicated faces belonging to both groups are kept/discarded by comparing orientation coincidence against `isSameOriNeeded = (theObjState == theToolsState)`; faces IN for both groups removed for CUT-like states (`bAvoidINforBoth`); ALL IN faces removed for FUSE (`bAvoidIN`); unused faces are re-shelled by connexity + `OrientFacesOnShell` into extra solids (`:795-833`) — nothing is dropped silently. History is NOT created for solids in this method (doc `BOPAlgo_Builder.hxx:167`).
- **BuildBOP validates that every input solid is a DS member** (`myDS->Index(aS) < 0` → `AlertUnknownShape`, `:517-529`) and accepts an optional SEPARATE report to avoid polluting the Builder report on repeated calls (doc `:169-173`).

---

## PORT MAP

Anchors: `session_cpp/src/brep_section.cpp` (`build_section_scaffold`), `session_cpp/src/brep.cpp`
(`split_with`, combine, classification), tolerances `tol3 = diag*2e-3`, `tol3_rep`.

| # | OCCT mechanism (exact) | Our anchor | Action | Design (for new builds) |
|---|---|---|---|---|
| 1 | Two-phase split: `BOPAlgo_PaveFiller` once + `BOPAlgo_Builder::PerformWithFiller` reuse (`Builder.cxx:198-208`; API ctor `BRepAlgoAPI_BuilderAlgo.cxx:38-47`) | `build_section_scaffold` recomputed per op call inside each boolean | **new build** | Cache `{operandsHash → scaffold+split arrangement}`; cut/common/fuse consume one shared intersection product like `PerformWithFiller` (also gives xor/split for free). |
| 2 | n-ary argument list + fence dedup: `BOPAlgo_Builder::AddArgument/SetArguments` (`Builder.cxx:103-125`), `CheckData` n≥2 | binary A/B entry in `brep.cpp` boolean ops | **new build** | Core takes `vector<Brep*>`; binary API = thin wrapper. First consumer: fuse of k solids without pairwise cascading. |
| 3 | BVH candidate iterator: `BOPDS_Iterator::Intersect` (BoxTree + `BOPTools_BoxPairSelector`, same-range skip, subshape skip, OBB opt, `BOPDS_Iterator.cxx:270-359`) | `build_section_scaffold` overlapping-surface-pair detection (currently O(nA·nB) box test) | **adopt** | Build one AABB tree over ALL operands' faces; self-join; discard same-operand pairs by index range; stable-sort pair list for determinism. |
| 4 | Tolerance-growth re-intersection: `BOPDS_Iterator::IntersectExt` (`:363-463`, SD-resolved boxes, cross-rank only, VV/VE/VF) | combine NK-RESCUE tolerant mate-pair pass (0.15*tol3) | **new build** | After any weld raises an entity's effective tolerance, re-run candidate selection with the inflated box for JUST those entities and re-attempt pave/mate creation — replaces the blanket 0.15*tol3 second pass with a targeted one. |
| 5 | `myImages`/`myOrigins` image bookkeeping with tri-state (unbound/empty/splits) (`Builder_1.cxx:40-168`) | `split_with` returns face pieces; combine consumes them anonymously | **new build** | Persistent `SplitMap: face_id → vector<result_face_id>` + back map, carried through combine; welds/renames update the map; identity faces stay unbound. Prerequisite for #8 and for SEGLOST-class debugging. |
| 6 | SD face detection: `FillSameDomainFaces` — `BOPTools_Set` edge-set hashing + same-solid exclusion + planar fast path + `AreFacesSameDomain` parallel check + min-index representative (`Builder_2.cxx:580-925`) | MISSING same-domain subsystem; nearest germ = whole-segment runs keyed by `seg_id` + combine tube merge | **new build** | Key candidate face pieces by sorted set of section/trim segment ids (our `seg_id`s = BOPTools_Set analog); exclude same-operand pairs; planar+bounded equal-set = SD直; else sample-based coincidence test at tol3_rep; elect min-id representative and substitute in the SplitMap. Unblocks tangent/coincident-face families (cyl-cyl tangency, box-on-box). |
| 7 | SD substitution into images + multi-origin fan-in (`Builder_2.cxx:886-921`) | combine common-block tube merge (vertex-pair-keyed) | **replace** | After #6, coincident pieces collapse BEFORE classification; classification then sees ONE membrane face with a both-sides role — removes the tube-merge special cases. |
| 8 | History: `PrepareHistory` + `LocModified` (=images) + `LocGenerated` (EE/EF new vertices; `FaceInfo.PaveBlocksSc/VerticesSc` section edges/vertices) + result-membership filter + Deleted rule (`Builder_4.cxx:27-252`) | MISSING history | **new build** | Post-combine pass: Modified = SplitMap entries surviving in result; Generated(faceA,faceB) = scaffold chains (we already store per-chain (p3,uvA,uvB) provenance — that IS `PaveBlocksSc`); Generated(edge) = chain-crossing paves on that edge; Deleted = source entity with no surviving split and no identity presence. Store as three id-maps on the result Brep. |
| 9 | History merge laws: `BRepTools_History::Merge` (G∘(G∪M)=G, M∘G=G, M∘M=M; `BRepTools_History.hxx:70-88`) | boolean_chain (oracle battery op) | **new build** | Implement the 3 composition rules over the id-maps of #8 so chained ops report against ORIGINAL inputs. |
| 10 | Section-edge API: `BRepAlgoAPI_BuilderAlgo::SectionEdges` from `InterfFF().Curves().PaveBlocks()` (`BRepAlgoAPI_BuilderAlgo.cxx:256-324`) | scaffold chains already exist internally | **adopt** | Expose `section_edges()` on the boolean result returning the kept chain intervals as polyline/curve edges — free once #8 stores chain provenance. |
| 11 | Input analysis battery: `BOPAlgo_ArgumentAnalyzer::Perform` — TestTypes dims table, TestSmallEdge (`IsMicroEdge`), TestContinuity (C0), TestCurveOnSurface (deviation > edge tol), TestMergeSubShapes (>1 coincidence partner both directions) (`ArgumentAnalyzer.cxx:130-1015`) | MISSING input validation (we currently discover bad operands mid-pipeline, e.g. trim-snapped slivers) | **new build** | `validate_operands()` preflight returning typed `CheckResult` list: micro-edge scan at tol3; pcurve-vs-3D deviation vs tol3_rep (we already compute this in reader diagnostics); cross-operand multi-coincidence scan of vertices (dist ≤ tolA+tolB) — direct early warning for the SEGWHOLE/alias-key families. |
| 12 | Self-intersection check: `BOPAlgo_CheckerSI` (level ladder 0-9; face-self `IntTools_FaceFace(F,F)` only for non-analytic + spindle torus; `BOPDS_IteratorSI` no-range-filter BVH) (`CheckerSI.cxx:102-508`) | MISSING; scaffold_eligible already narrows to deg≥3 surfaces | **new build** | Gated preflight: self-SSI march only for deg≥3 faces (same filter OCCT uses: skip plane/cyl/cone/sphere, check torus only if minor≥major); report pairs, do not attempt repair. Low priority; protects the marching corrector from self-crossing charts. |
| 13 | In-parts classification computed once, reused: `FillIn3DParts` → `myInParts` → `BuildSplitSolids`/`BuildBOP` (`Builder_3.cxx:97-263`, `Builder.cxx:552-556`) | per-op winding/flood/parity classification in combine | **replace** | Classify each candidate face once against the OPPOSITE operand, store `{face_piece → IN/OUT/ON}` table; each op (cut/common/fuse) is then a pure table lookup with the BuildBOP state map COMMON=IN/IN, FUSE=OUT/OUT, CUT=OUT/IN, CUT21=IN/OUT (`Builder.hxx:214-250`). Our flood/parity becomes the table filler, run once. |
| 14 | SD-face keep/drop by orientation vs `isSameOriNeeded` + `bAvoidIN`/`bAvoidINforBoth` filters + unused-face connexity reshelling (`Builder.cxx:634-833`) | combine tube-merge classification + face_outward_signs tiers | **adopt** | Import the exact decision table for double-membership faces (kept once if orientations agree per op-parity, dropped otherwise; both-IN dropped for CUT; any-IN dropped for FUSE); route rejected-but-closed leftovers through our connexity flood instead of discarding (matches OCCT's unused-face solids). |
| 15 | Draft fast paths: `BuildDraftFace` (skip arrangement when only boundary re-splits; bail on internal/multiconnected/unified) + `FillImagesContainer` reuse-if-unchanged + `FillIn3DParts` skip-unsplit-solid (`Builder_2.cxx:298-351,1052-1189`; `Builder_1.cxx:221-240`; `Builder_3.cxx:226-239`) | `split_by_uv_curves` always runs full UV arrangement | **adopt** | If a face receives no section runs and no trim edge was re-split → pass face through unchanged (identity image); this is the single biggest scale lever for many-face imported models. |
| 16 | Small-edge elision: no pave blocks → empty image list (`Builder_1.cxx:92-95`) | micro filter / micro-edge collapse in scaffold+combine | **already-equivalent** | Note only: record the elision as Deleted in #8 instead of forgetting it. |
| 17 | Tolerance post-treatment: `PostTreat` → `CorrectTolerances(shape, avoid, 0.05)` + `CorrectShapeTolerances` + non-destructive avoid-map (`Builder.cxx:450-475`) | no growth model; fixed tol3/tol3_rep bands | **new build** | Per-entity result tolerances: vertex tol = max over incident chain-end deviations; edge tol = max curve↔surface deviation of its two host faces (we compute this in the ON test already); cap growth at 0.05·(local size); never touch input entities. Gives downstream consumers (Rhino import) honest tolerances instead of implied diag bands. |
| 18 | Fuzzy option: `SetFuzzyValue` clamped ≥ `Precision::Confusion()`, fed to SSI, OBB inflation, SD checks (`Options.cxx:105-108`) | tol3 = diag*2e-3 hardwired | **new build** | Add user `fuzzy` parameter folded into pave/weld/ON tolerances with a floor; keep diag*2e-3 as the default derivation. |
| 19 | Parallel substrate: `BOPTools_Parallel::Perform` + per-thread `IntTools_Context` functors + `BOPAlgo_ParallelAlgo` task vectors (`BOPTools_Parallel.hxx:26-185`) | single-threaded pipeline | **new build** (deferred) | Structure per-pair SSI march and per-face UV arrangement as independent value-type tasks harvested serially (they already are pure per pair/face); per-thread corrector caches mirror ContextFunctor. Do NOT parallelize combine (map mutation). |
| 20 | Alert/report + UserBreak: `BOPAlgo_Options` report (errors abort, warnings carry, merged upward), demotion of sub-solver errors to warnings (`Builder_3.cxx:544-576`), crash→alert catch (`Builder.cxx:212-227`) | SESSION_* env prints + exit codes | **new build** | `BoolReport {errors, warnings}` threaded scaffold→split→combine; sub-stage failures (e.g. one face's arrangement) demote to warning + identity image instead of aborting the whole op — matches our observed partial-run failure mode. |
| 21 | Determinism: stable-sorted pair lists (`BOPDS_Iterator.cxx:203`), sorted SD candidates, min-index representative | chains canonicalized index-corresponded | **adopt** | Sort every pair/candidate list and elect min-id representatives at all weld/alias decision points (rotation-frontier nondeterminism guard). |
| 22 | `SimplifyResult` = `ShapeUpgrade_UnifySameDomain` post-op with history merge (`BRepAlgoAPI_BuilderAlgo.cxx:168-200`) | merge_coplanar_faces (STEP-dump only) | **already-equivalent** | Once #8 exists, merge its rename map into history like OCCT merges `mySimplifierHistory`. |
| 23 | Internal-shape settlement: `FillInternalVertices` (`ComputeVF`) + `FillInternalShapes` (1e-11 point-in-solid; copy-not-mutate originals) (`Builder_2.cxx:929-1008`, `Builder_3.cxx:622-887`) | not handled (we have no INTERNAL concept) | **new build** (low priority) | Only needed for GF parity on wire/vertex arguments; skip until n-ary fuse consumes non-solid operands. |
| 24 | Glue modes (`BOPAlgo_GlueEnum`: Off/Shift/Full — bypass intersection for known-coincident inputs) | same-domain imprint infrastructure (`extra_cuts`) | **new build** (deferred) | `glue=full`: skip SSI entirely, feed trim curves as section chains directly into split_with — our extra_cuts path is 80% of this. |
| 25 | API contract: `Build/IsDone/Shape/Modified/Generated/IsDeleted/HasErrors/SectionEdges/SetNonDestructive/SetToFillHistory` (`BRepAlgoAPI_BuilderAlgo.hxx:61-243`) | ad-hoc `boolean_*` functions returning Brep | **new build** | `BooleanOp` struct wrapping #1/#8/#13/#18/#20 with exactly these method names; binary convenience ctor mirrors `BRepAlgoAPI_BooleanOperation(S1,S2,op)`. |

Priority order for our kernel (impact × current pain):
**#6/#7 (same-domain subsystem)** → **#13 (classify-once table + BuildBOP state map)** → **#5+#8 (image maps + history)** → #15 (draft fast paths, scale) → #11 (preflight analyzer) → #3 (BVH iterator) → #17 (tolerance growth).
