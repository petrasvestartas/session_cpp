# brepalgoapi-cells — derived-operation layer (CellsBuilder / Splitter / MakerVolume / MakePeriodic / API options)

Implementation spec extracted from OCCT sources (read 2026-07-24) at
`C:/brg/compas_occt/external/occt/src/occt/src/ModelingAlgorithms/TKBO/{BRepAlgoAPI,BOPAlgo}`.
Files read verbatim: `BOPAlgo_CellsBuilder.cxx/.hxx`, `BOPAlgo_Splitter.cxx/.hxx`,
`BOPAlgo_MakerVolume.cxx/.hxx/.lxx`, `BOPAlgo_MakePeriodic.cxx/.hxx`, `BOPAlgo_ToolsProvider.cxx`,
`BOPAlgo_Options.cxx/.hxx`, `BOPAlgo_GlueEnum.hxx`, `BOPAlgo_Operation.hxx`,
`BRepAlgoAPI_Algo.hxx`, `BRepAlgoAPI_BuilderAlgo.cxx/.hxx`, `BRepAlgoAPI_BooleanOperation.cxx`,
`BRepAlgoAPI_Splitter.cxx`, `BRepAlgoAPI_Section.cxx`.

Scope: everything ABOVE the general-fuse core — the "derived operation" layer where an industry
kernel turns one shared split into many products (cells algebra, splitter, volume maker,
periodicity) and the public option contract (fuzzy, glue, safe mode, history switches).

**Related specs (referenced, not duplicated):**
- `kb/occt_history-gf-scale.md` — BRepAlgoAPI_BuilderAlgo/BooleanOperation internals, BRepTools_History
  semantics, ArgumentAnalyzer/CheckerSI. This spec only adds what that one omitted.
- `kb/occt_builder-assembly.md` — `BOPAlgo_Builder::PerformInternal1` image ladder, `BOPAlgo_BOP`
  cell-keep tables, `BuildBOP`, `BOPAlgo_BuilderSolid`. CellsBuilder delegates to all of these.
- `kb/occt_pavefiller-core.md` — the intersection phase every algorithm here reuses via one
  shared `BOPAlgo_PaveFiller`.

---

## STAGE PIPELINE

### A. CellsBuilder — the cell algebra (generalizes every boolean, including our xor/split)

Class relation: `BOPAlgo_CellsBuilder : BOPAlgo_Builder` (BOPAlgo_CellsBuilder.hxx:168).
Phase 1 is a plain General Fuse; phase 2 is a pure SELECTION algebra over the split parts;
phase 3 is optional boundary removal ("materials").

1. **`BOPAlgo_CellsBuilder::PerformInternal1`** (BOPAlgo_CellsBuilder.cxx:86-109)
   - Saves `HasHistory()`, calls `SetToFillHistory(false)` — history filling is deliberately
     SUPPRESSED during the GF phase because the GF result is about to be nullified.
   - Runs `BOPAlgo_Builder::PerformInternal1(theFiller, ...)` — full GF (see builder-assembly spec).
   - `IndexParts()`; then `RemoveAllFromResult()` (result := empty compound); restores history flag.
   - Contract: after `Perform()` the result `Shape()` is an EMPTY compound; `GetAllParts()`
     returns every split part.

2. **`BOPAlgo_CellsBuilder::IndexParts`** (BOPAlgo_CellsBuilder.cxx:113-238)
   - For every argument (compounds flattened via `BOPTools_AlgoTools::TreatCompound`), for every
     sub-shape of the argument's dimension type (`TypeToExplore(Dimension)`; dim→type table at
     :1180-1203: 0=VERTEX 1=EDGE 2=FACE 3=SOLID):
     * if the sub-shape has no image in `myImages` → the part IS the sub-shape itself;
     * else each image is a part.
     * `myIndex[part] += argumentShape` — **the origin set**: the list of top-level argument
       shapes whose splits contain this part. `myAllParts` compound collects each part once
       (fence map `aMFence`).
   - **Multi-dimensional second pass** (:181-237, only when args have ≥2 distinct dimensions):
     every LOWER-dimensional sub-shape of an indexed part inherits the part's origin list
     (unioned, uniqueness enforced by IsSame scan). Without this, cell queries mixing a SOLID
     argument and a FACE argument would never match.

3. **`BOPAlgo_CellsBuilder::FindParts`** (BOPAlgo_CellsBuilder.cxx:645-760) — the algebra kernel.
   Query = two shape lists (Take, Avoid). A part is selected iff it is **IN every shape of Take
   and OUT of every shape of Avoid**. Implementation is purely combinatorial — no geometry:
   - dedupe Take/Avoid into maps; `aNbS = aMSToTake.Extent()`;
   - pick the Take shape of MINIMAL dimension (`iDimMin` initialized to 10) — its splits are the
     candidate pool (a common cell must be a split of every take-shape, so the smallest-dim one
     has the fewest/smallest candidates);
   - for each split part of that shape (via `myImages`, falling back to the original):
     * reject if `!myIndex.Contains(part)`;
     * origin list `aLS = myIndex.FindFromKey(part)`; reject if `aLS.Extent() < aNbS`;
     * reject if any origin ∈ Avoid map; reject unless every Take shape ∈ origin set.
   - **Membership in the origin set IS the point-classification** — computed once by GF, reused
     by every subsequent query. No per-query winding/ray tests.

4. **`AddToResult(Take, Avoid, material=0, update=false)`** (:242-314)
   - `FindParts`; add each part to `myShape` compound with two fences: parts already in result
     (`aResParts` scan of current `myShape`), and parts already bound to a material
     (`myShapeMaterial.IsBound`) — a part can be added under only ONE material, silently skipped
     otherwise.
   - `material != 0`: bind part→material (`myShapeMaterial`) and material→parts (`myMaterials`).
   - `update=false` → `PrepareHistory()` only (if changed); `update=true` → immediately
     `RemoveInternalBoundaries()`.
   - `AddAllToResult(material, update)` (:318-348): result := `myAllParts` wholesale, all
     material maps rebuilt.

5. **`RemoveFromResult(Take, Avoid)`** (:352-445)
   - `FindParts`; unbind materials for removed parts (list surgery in `myMaterials` + UnBind in
     `myShapeMaterial`); rebuild the result compound EXCLUDING removed parts.
   - Containers (WIRE/SHELL/COMPSOLID) in the result that contained a removed part are rebuilt
     from surviving members via `MakeTypedContainers` (:1089-1147); empty containers vanish.
   - `RemoveAllFromResult()` (:449-461): empty compound + clear all material maps +
     `PrepareHistory`.

6. **`RemoveInternalBoundaries()`** (:465-641) — material-driven boundary dissolution.
   - No-op if `myMaterials` empty (material 0 is the "keep boundaries" sentinel and is never
     entered into the map).
   - Per material group: all shapes must be the SAME type, else warning
     `BOPAlgo_AlertRemovalOfIBForMDimShapes` and group kept as-is (removal of internal
     boundaries between multi-dimensional shapes is unsupported).
   - EDGE/FACE groups: DEFERRED — collected into `aLSUnify[0|1]` with a keep-map computed by
     `CollectMaterialBoundaries` (:1153-1176): map sub-shape→ancestors within the group; a
     sub-shape with exactly 1 ancestor is a BOUNDARY of the material area and is added to the
     keep-map so unification cannot dissolve the region outline. Then ONE
     `ShapeUpgrade_UnifySameDomain` run per type via `RemoveInternals` with
     `KeepShapes(theMapKeepBnd)`.
   - SOLID groups: immediate `RemoveInternals(aLS, aLSNew)` (:810-1036):
     * `BOPTools_AlgoTools::MakeConnexityBlocks(solids, FACE, SOLID)` — only face-connected
       solids can fuse (internal entities re-attachable per block);
     * per block, map face→owning solids (`aDMFS`); **faces with exactly 1 owner are boundary,
       faces with 2 owners are internal**; if every face is unique → nothing to remove, keep
       solids;
     * else `BOPAlgo_BuilderSolid` on the unique faces (see builder-assembly spec); MUST produce
       exactly 1 area, else warning `BOPAlgo_AlertRemovalOfIBForSolidsFailed` and originals kept;
     * INTERNAL-oriented sub-shapes of the old solids re-added into the new solid
       (`aSNew.Free(true)` … `Free(false)`);
     * every old solid → new solid recorded in `myMapModified`.
   - EDGE/FACE `RemoveInternals`: build WIRE/SHELL container, `ShapeUpgrade_UnifySameDomain
     anUnify(aShape, bEdges, bFaces)` + `KeepShapes`; empty output → warning
     (`...IBForEdgesFailed`/`...IBForFacesFailed`) and originals kept; unifier history
     (V/E/F `Modified`) recorded into `myMapModified`, materials propagated to unified shapes.
   - Finally shapes WITHOUT material are re-added; containers partially consumed by material
     binding are rebuilt via `MakeTypedContainers`.

7. **`MakeContainers()`** (:764-806): bucket result parts by dimension, wrap each connexity
   block into WIRE/SHELL/COMPSOLID (`MakeTypedContainers`; SHELL gets
   `BOPTools_AlgoTools::OrientFacesOnShell`).

8. **History overlay — `LocModified`** (:1040-1085): overrides Builder's hook. Chain:
   GF image list (`BOPAlgo_Builder::LocModified`) → per-image lookup in `myMapModified`
   (unification/solid-fusion results) with fence dedup. So `Modified(S)` transparently composes
   split-then-unify.

DRAW contract (hxx:146-167): `bfillds; bcbuild rx; bcadd res s1 1 s2 0 -m 1; bcremoveint res` —
each `si k` pair means Take (k=1) / Avoid (k=0).

### B. Splitter — GF minus tool parts

1. **`BOPAlgo_ToolsProvider`** (BOPAlgo_ToolsProvider.cxx:20-65): tiny base adding
   `myTools` + `myMapTools` fence (`AddTool` dedups by IsSame-hash). `BOPAlgo_Splitter :
   BOPAlgo_ToolsProvider : BOPAlgo_Builder`.
2. **`BOPAlgo_Splitter::CheckData`** (BOPAlgo_Splitter.cxx:40-50):
   `myArguments.IsEmpty() || (args+tools) < 2` → `BOPAlgo_AlertTooFewArguments`. Tools optional:
   no tools ⇒ result ≡ General Fuse.
3. **`BOPAlgo_Splitter::Perform`** (:54-93): concatenate Objects then Tools into ONE list →
   fresh `BOPAlgo_PaveFiller` with full option forwarding (`SetRunParallel, SetFuzzyValue,
   SetNonDestructive, SetGlue, SetUseOBB`) → `PerformInternal`. The trick: the Builder base
   class only iterates `myArguments` (= Objects) when building the result, so **tool splits are
   excluded from the result by construction** — no override of image building needed
   (hxx:44-49 documents this as the intended minimal implementation).
4. **`BOPAlgo_Splitter::BuildResult(COMPOUND)`** (:97-121): on the final call, if exactly 1
   argument produced exactly 1 shape in the result compound → unwrap (result := that shape, no
   gratuitous compound nesting).
5. API wrapper **`BRepAlgoAPI_Splitter::Build`** (BRepAlgoAPI_Splitter.cxx:35-76): same
   NotDone/Clear/arg-check ladder as BooleanOperation (see history-gf-scale spec), intersects
   args+tools together (70% progress), then `myBuilder = new BOPAlgo_Splitter` +
   `SetArguments/SetTools` + shared `BuildResult` (30%).

### C. MakerVolume — cells from a face soup (the "enclosing box" universe trick)

`BOPAlgo_MakerVolume : BOPAlgo_Builder`. Defaults (lxx:17-31): `myIntersect=true`,
`myAvoidInternalShapes=false`.

`PerformInternal1` (BOPAlgo_MakerVolume.cxx:102-194), numbered per the header doc (hxx:37-55):
1. `CheckData` (:33-42): args non-empty else `AlertTooFewArguments`.
2. `Prepare` + GF image ladder **only if `myIntersect`** (vertices→edges→wires→faces; no solids
   stage — this algorithm MAKES the solids). If `myIntersect=false`, `Perform` (:64-83) wraps
   all arguments into ONE compound argument so the PaveFiller sees a single argument and
   computes no interferences (user guarantees non-interference).
3. `CollectFaces` (:213-251): every DS FACE (or its images, fence-deduped) → `AddFace`
   (:407-414) appends the face **TWICE — FORWARD and REVERSED** — into `myFaces`; every face
   box accumulated into `myBBox`.
4. `MakeBox` (:255-276): `anExt = sqrt(myBBox.SquareExtent()) * 0.5; myBBox.Enlarge(anExt)`;
   axis box solid from corners; its 6 faces appended to `myFaces` AND remembered in
   `theBoxFaces`.
5. `BuildSolids` (:280-298): one `BOPAlgo_BuilderSolid` over all faces (both orientations + box)
   → `Areas()`; failure → `BOPAlgo_AlertSolidBuilderFailed`.
6. `RemoveBox` (:302-333): find the solid containing ANY box face → remove it; **breaks after
   the first hit** — the outer "universe" cell is the unique solid touching the box.
7. `FillInternalShapes` (:359-403): unless `myAvoidInternalShapes` — free vertices/edges (and
   wire members) of the arguments classified into the created solids via
   `BOPAlgo_Tools::FillInternals`.
8. `BuildShape` (:337-355): 1 solid → the solid itself; else compound of solids.
9. `PrepareHistory` + `PostTreat` (base Builder).

### D. MakePeriodic — Splitter-composed periodicity (twin association + glue tiers in anger)

`BOPAlgo_MakePeriodic : BOPAlgo_Options` (NOT a Builder — it composes API-level ops).
`Perform` = `CheckData` → `Trim` → `MakeIdentical` (BOPAlgo_MakePeriodic.cxx:40-73).

1. `CheckData` (:79-91): at least one direction periodic with period ≥ `Precision::Confusion()`,
   else `AlertNoPeriodicityRequired`.
2. `Trim` (:123-190): only for non-trimmed directions. Bounding box of the shape enlarged by
   `0.1 * sqrt(SquareExtent())`; per untrimmed direction the box min/max coords are overwritten
   with `[PeriodFirst(i), PeriodFirst(i)+Period(i)]`; **`BRepAlgoAPI_Common`** of shape × box =
   trimmed shape; errors → `AlertUnableToTrim` with an evidence compound. History into
   `mySplitHistory` (object side only).
3. `MakeIdentical` (:197-225) = `SplitNegative` then `SplitPositive`.
   - `SplitNegative` (:232-256): per periodic direction, translate the shape by
     `-Period(i)` and use the copy as a TOOL to split the original — one direction per
     operation "to avoid conflicts when copying geometries".
   - `SplitPositive` (:296-390): same with `+Period(i)` translations (all directions'
     tools this time), then associates each original sub-shape with the splits of its
     translated ghost — the **twins map** (`AddTwin` :262-289, symmetric entries).
4. **`SplitShape`** (:396-470) — the load-bearing composition:
   - `BOPAlgo_PaveFiller` fed **TOOLS FIRST, shape LAST** (`SetArguments(theTools);
     AddArgument(myShape)`) — comment :400-403: "coinciding parts use the geometry of the FIRST
     argument", i.e. argument ORDER is the geometry-ownership rule for same-domain welding.
   - `SetGlue(BOPAlgo_GlueShift)` (partial coincidence — the ghost overlaps but never truly
     intersects transversally) + `SetNonDestructive(true)` (the same geometry appears in two
     arguments — must not be mutated).
   - Then `BRepAlgoAPI_Splitter aSplitter(anIntersector)` — REUSES the filler (the
     prebuilt-PaveFiller ctor), `SetGlue(GlueShift)` again, `Build()`; errors →
     `AlertUnableToMakeIdentical`; histories merged per side.
5. `RepeatShape` (:476-581): translated copies (each recorded as `AddGenerated` in a
   translation history), then **`BOPAlgo_Builder` used as a GLUER**: `SetGlue(BOPAlgo_GlueFull)`
   — full coincidence, no intersection at all, gluing is pure identification. Failure is only a
   WARNING (`AlertUnableToRepeat`) — the un-glued repetition survives. Repeat period compounds:
   `myRepeatPeriod[id] += |times| * myRepeatPeriod[id]`. `UpdateTwins` (:587+) maps twins
   through translation+gluing histories.

### E. API option plumbing — the public contract

(BRepAlgoAPI_BuilderAlgo details in history-gf-scale spec; here the option semantics.)

- **Class ladder**: `BRepAlgoAPI_Algo : BRepBuilderAPI_MakeShape, protected BOPAlgo_Options`
  (BRepAlgoAPI_Algo.hxx:30) — options inherited protected, re-exported with `using` declarations
  (:41-54) so every API class exposes `SetFuzzyValue/SetRunParallel/SetUseOBB/HasErrors/
  GetReport/Dump*` uniformly.
- **`BOPAlgo_Options`** (BOPAlgo_Options.hxx:36-148) holds exactly: allocator, `Message_Report`,
  `myRunParallel`, `myFuzzyValue`, `myUseOBB`. Ctor defaults (cxx:49-69):
  `myRunParallel = myGlobalRunParallel` (process-wide static, settable via
  `SetParallelMode`), `myFuzzyValue = Precision::Confusion()`, `myUseOBB = false`.
- **Fuzzy clamp**: `SetFuzzyValue(f) { myFuzzyValue = std::max(f, Precision::Confusion()); }`
  (BOPAlgo_Options.cxx:105-108) — fuzzy can never go below 1e-7; "0" means "default", not
  "exact". The fuzzy value is consumed in the PaveFiller (half added to each vertex/edge
  tolerance — see pavefiller-core spec) and reused by `SimplifyResult` as the linear
  unification tolerance (BRepAlgoAPI_BuilderAlgo.cxx:185).
- **BuilderAlgo-level options** (BRepAlgoAPI_BuilderAlgo.hxx:222-231, defaults cxx:25-34):
  `myNonDestructive=false` (safe mode: inputs never mutated, modified sub-shapes copied;
  auto-forced ON when any input sub-shape is Locked — pavefiller-core), `myGlue=BOPAlgo_GlueOff`,
  `myCheckInverted=true` (inverted-solid = hole-in-space check), `myFillHistory=true`
  (`SetToFillHistory` is THE history kill-switch; when off, `Modified/Generated` return empty
  and `IsDeleted` false — cxx:204-252).
- **Glue tiers** (BOPAlgo_GlueEnum.hxx:57-62 + doc 18-55): `GlueOff` | `GlueShift` (partial
  coincidence: skips FF intersections; faces still split) | `GlueFull` (full coincidence:
  additionally skips VF/EF — nothing is split). Pure trust-me flags: "algorithms do not check
  this itself"; wrong tier ⇒ wrong result.
- **Option forwarding point**: every derived op copies the SAME five options into its private
  PaveFiller: `SetRunParallel, SetFuzzyValue, SetNonDestructive, SetGlue, SetUseOBB`
  (BRepAlgoAPI_BuilderAlgo.cxx:122-127; BOPAlgo_Splitter.cxx:81-86; BOPAlgo_MakerVolume.cxx:89-93)
  — options live on the API object, the filler is disposable. The `SetAttributes()` virtual
  hook (BuilderAlgo.hxx:208) lets subclasses add filler-only options: `BRepAlgoAPI_Section::
  SetAttributes` (BRepAlgoAPI_Section.cxx:209-213) passes `BOPAlgo_SectionAttribute(myApprox,
  myComputePCurve1, myComputePCurve2)`.
- **Section convenience ctors** (BRepAlgoAPI_Section.cxx:48-121): shape×shape, shape×gp_Pln,
  shape×Geom_Surface, surface×surface; `MakeShape` (:327-335): surface ≥ C2 → face
  (tol `Precision::Confusion()`), else shell. `HasAncestorFaceOn1/2` (:224-240 + 275-323)
  recovers the (F1,F2) pair that generated a section edge by scanning `InterfFF().Curves().
  PaveBlocks()` for the edge index.
- **Operation enum** (BOPAlgo_Operation.hxx:18-26): COMMON/FUSE/CUT/CUT21/SECTION/UNKNOWN.
  CUT21 = tools minus objects (arguments swapped at the classification table, not by list swap).
- **Debug hook**: env `CSF_DEBUG_BOP=<dir>` dumps args + DRAW script for invalid inputs/results
  (BRepAlgoAPI_BooleanOperation.cxx:36-79, 229-330).

---

## DATA STRUCTURES

- **`BOPAlgo_CellsBuilder` fields** (hxx:256-263):
  - `TopoDS_Shape myAllParts` — compound of every split part (each once).
  - `NCollection_IndexedDataMap<TopoDS_Shape, List<TopoDS_Shape>> myIndex` — **part → origin
    set** (argument shapes whose splits contain the part). THE cell-classification structure.
  - `NCollection_DataMap<int, List<TopoDS_Shape>> myMaterials` — material id → parts.
  - `NCollection_DataMap<TopoDS_Shape, int> myShapeMaterial` — part → material (single-valued).
  - `NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myMapModified` — local unification map
    (split → unified shape), overlaid on GF images in `LocModified`.
- **Cell query** = (Take list, Avoid list) → predicate `origin ⊇ Take ∧ origin ∩ Avoid = ∅`.
- **`BOPAlgo_ToolsProvider`**: `myTools` list + `myMapTools` fence map.
- **`BOPAlgo_MakerVolume`**: `myIntersect`, `Bnd_Box myBBox`, `TopoDS_Solid mySBox`,
  `List myFaces` (every candidate face twice, both orientations), `myAvoidInternalShapes`.
- **`BOPAlgo_MakePeriodic`**: `PeriodicityParams` POD (`bool myPeriodic[3]; double myPeriod[3];
  bool myIsTrimmed[3]; double myPeriodFirst[3]`, hxx:141-164); twins maps
  `myTwins`/`myRepeatedTwins` (shape → list of identical shapes on opposite side);
  `mySplitHistory` vs `myHistory` (split-only vs split+repeat); `myRepeatPeriod[3]`.
- **`BOPAlgo_Options`**: allocator, report, `myRunParallel`, `myFuzzyValue`, `myUseOBB`;
  static `myGlobalRunParallel`.
- **BuilderAlgo**: `myArguments`, `myNonDestructive`, `myGlue`, `myCheckInverted`,
  `myFillHistory`, `myIsIntersectionNeeded` (false when a prebuilt PaveFiller was injected —
  filler REUSE across derived ops), `myDSFiller`, `myBuilder`, `myHistory`,
  `mySimplifierHistory`.

---

## CONSTANTS & TOLERANCES

- `Precision::Confusion() = 1.e-7` (Precision.hxx:165) — default fuzzy value AND the lower
  clamp of `SetFuzzyValue` (BOPAlgo_Options.cxx:107); minimum meaningful period in
  MakePeriodic::CheckData; face tolerance in Section's `MakeShape`.
- `Precision::Angular() = 1.e-12` (Precision.hxx:123) — default angular tolerance of
  `SimplifyResult` (BRepAlgoAPI_BuilderAlgo.hxx:150).
- Material sentinel: `0` = "no material / keep boundaries" (never bound into `myMaterials`).
- `FindParts` min-dimension seed: `iDimMin = 10` (CellsBuilder.cxx:676).
- MakerVolume box enlargement: `anExt = sqrt(myBBox.SquareExtent()) * 0.5` (MakerVolume.cxx:261).
- MakePeriodic trim-box enlargement: `0.1 * sqrt(aBox.SquareExtent())` (MakePeriodic.cxx:137).
- Map preallocation: all CellsBuilder maps sized 100 (cxx:39-56); ToolsProvider fence 100.
- Progress budgets: API intersect/build = 70/30 (BuilderAlgo.cxx:89-100, Splitter, BooleanOperation);
  MakerVolume intersect/build = 9/1 when intersecting, 0.5/9.5 when not (MakerVolume.cxx:49-50);
  MakerVolume `fillPISteps`: TreatFaces and BuildSolids weighted `50 * NbFaces` (:198-209).
- Glue enum values: `GlueOff=0, GlueShift=1, GlueFull=2`.
- Operation enum: `COMMON=0, FUSE=1, CUT=2, CUT21=3, SECTION=4, UNKNOWN=5`.

---

## INVARIANTS

1. **One split, many products.** GF is executed ONCE; every derived operation (any boolean,
   xor, split-to-cells, arbitrary user unions) is a pure set-membership SELECTION over the
   indexed parts. The origin set (`myIndex`) fully classifies a part; no geometric test is
   ever repeated at query time.
2. **Origin-set semantics**: part P is a split of argument S ⟺ S ∈ `myIndex[P]`. "IN S" for a
   cell of the GF result means literally "P survived inside S's image", so the boolean tables
   (see `BOPAlgo_BOP` in builder-assembly) are the special cases Take={A},Avoid={B} (CUT),
   Take={A,B} (COMMON), etc.
3. **Uniqueness discipline**: a part appears in the result at most once (result-scan fence) and
   carries at most one material (`myShapeMaterial` checked before add). Materials never merge
   across dimension: same material + different shape type ⇒ warning, no removal.
4. **Boundary-of-area preservation**: when dissolving internal boundaries of EDGE/FACE material
   groups, sub-shapes with exactly ONE ancestor in the group are the area boundary and are
   pinned via `UnifySameDomain::KeepShapes` — dissolution must not leak past the material
   region's outline.
5. **Unique-face rule for solid fusion**: within a face-connected block, a face bounding
   exactly 1 kept solid is exterior; a face shared by 2 is interior and is dropped;
   the rebuilt solid must come back as exactly ONE area from BuilderSolid or the fusion is
   rolled back (warning, originals kept). Result is never silently wrong topology.
6. **Splitter result = images of Objects only**; tool splits participate in intersection but
   are structurally excluded because the Builder result loop iterates `myArguments`. No tools ⇒
   Splitter ≡ GF. Fewer than 2 total shapes ⇒ error.
7. **MakerVolume universe cell**: with every face inserted twice (both orientations) plus a
   strictly-enclosing box, BuilderSolid partitions space so that EXACTLY ONE solid touches box
   faces — removing it leaves precisely the finite cells of the arrangement.
8. **Argument order = geometry ownership** (MakePeriodic::SplitShape): when arguments coincide
   within tolerance, the FIRST argument's geometry wins in the shared result. Feeding tools
   first is how the algorithm guarantees the periodic ghost's geometry is copied onto the
   shape's split boundary (twin faces are geometrically IDENTICAL, not just close).
9. **History composes**: GF images → local modification overlay (`myMapModified`) →
   (API level) simplifier history merge. `Modified()` at the API always reports the FINAL
   shape, whatever chain of split/unify/simplify produced it. History filling is suspended
   whenever the intermediate result would be thrown away (CellsBuilder GF phase).
10. **Options outlive fillers**: options are state of the API object; each `Perform` forwards
    them into a fresh (or injected) PaveFiller. An injected filler (`myIsIntersectionNeeded =
    false`) makes intersection REUSABLE across multiple derived operations on the same
    arguments — the OCCT idiom for our "compute section network once, run all ops" goal.

---

## PITFALLS

- **Silent material conflict**: `AddToResult` with a material skips parts already bound to
  another material WITHOUT any warning (cxx:270, 285) — parts stay in result under the old
  material.
- **`RemoveInternalBoundaries` with material 0 is a no-op** — callers who forgot to assign a
  non-zero material get their boundaries kept, no diagnostics.
- **Recommended ordering for EDGE/FACE cells** (hxx:58-60): remove boundaries only at the END,
  after the result is complete — early unification of open regions can create
  self-intersections when more parts are added later.
- **Deferred vs immediate removal asymmetry**: solids dissolve inside `RemoveInternalBoundaries`
  per material; edges/faces are batched across ALL materials into two UnifySameDomain runs —
  a failure there (empty output) keeps originals with only a warning; result mixes unified and
  raw parts.
- **`RemoveBox` breaks on the first solid containing a box face** — if the input face soup
  leaks (open shells), several solids may touch the box and finite cells silently absorb
  box faces (garbage faces in output). The enclosure must be watertight for the trick to hold.
- **`SetIntersect(false)` trusts the caller**: arguments are wrapped into one compound so the
  PaveFiller computes NO interference; interfering inputs produce unpredictable results
  (hxx:58-63 explicit).
- **Glue is unchecked**: `GlueShift/GlueFull` on genuinely intersecting shapes = wrong result
  by design (GlueEnum.hxx:48-50). Full requires NO partial overlap at all.
- **Fuzzy floor**: you cannot request a fuzzy below 1e-7; passing 0 restores the default, it
  does not disable fuzz. Port note: our `tolerance = 0.0` API means "class default" — same
  convention, keep it.
- **CellsBuilder `FindParts` early-reject subtlety**: `aLS.Extent() < aNbS` compares the origin
  LIST length (unique by construction in `IndexParts`) with the deduped take-map size;
  duplicating a shape in the take list is harmless, but passing a compound that CONTAINS an
  argument (rather than the argument itself) never matches — matching is by `IsSame` on the
  top-level argument shape.
- **Multi-dim ancestor inheritance is one-way (downward)**: a solid part never inherits a
  face-argument origin unless the face's parts physically appear among the solid part's
  sub-shapes; querying Take={solidA}, Avoid={faceB} relies on the second IndexParts pass having
  run (only when dims differ).
- **CUT21 exists because argument/tool lists are not symmetric** in the classification tables;
  swapping lists instead changes rank → changes same-domain geometry ownership (see invariant 8).
- **BooleanOperation requires non-empty args AND tools** even though the core is n-ary — a
  binary-only API gate (`AlertTooFewArguments`), plus `AlertBOPNotSet` for UNKNOWN op.
- **MakePeriodic split failure = error, repeat glue failure = warning**: periodicity is
  mandatory, repetition is best-effort; callers must check both channels.

---

## PORT MAP

Our pipeline anchors: `brep_section.cpp build_section_scaffold` (SSI chains, paves,
keep-verdict, valence-1 bridge, welded vertices) → `brep.cpp split_with` (UV arrangement,
shared-chain run lifting, whole-seg keys) → `combine` (exact weld + tube merge + NK-RESCUE) →
winding+radial classification. Derived ops today: `brep.h boolean_xor` (:386, disjoint assembly
of two difference runs) and `boolean_split` (:397, three independent boolean runs).

| OCCT mechanism | Our anchor | Action |
|---|---|---|
| One shared PaveFiller reused by all derived ops (`BRepAlgoAPI_BuilderAlgo(const BOPAlgo_PaveFiller&)`, `myIsIntersectionNeeded=false`) | `split_with` re-runs SSI + arrangement per boolean; `boolean_xor/split` pay 2-3 full section networks | **new-build**: cache the section scaffold + both operands' split face sets once per (A,B,tol) and derive every op from it — the exact OCCT filler-injection pattern (we already march the network once for one op; lift it above the op switch). |
| CellsBuilder origin-set algebra (`IndexParts` part→origin map; `FindParts` Take/Avoid superset/disjoint predicate) | winding+radial classification produces a per-face in/out verdict per operand inside `boolean()` | **adopt**: store the verdict as a per-face origin/side bitmask (IN_A, IN_B) on the combined split instead of consuming it immediately; ops become keep-masks: CUT=IN_A&OUT_B, COMMON=IN_A&IN_B, XOR=(IN_A&OUT_B)\|(IN_B&OUT_A), SPLIT=partition by mask value. Kills the re-run instability our `boolean_xor` doc-comment records (union-on-results shattered same-domain machinery). |
| Splitter contract: tools participate in intersection, excluded from result; no tools ⇒ GF; 1-arg compound unwrap (`BOPAlgo_Splitter::BuildResult`) | `boolean_split` = 3 boolean runs | **replace**: implement split as the mask partition of the single shared split (previous row); keep the {A-B, A∩B, B-A} output order and empty-fragment omission as the API contract. |
| Materials + `RemoveInternalBoundaries` unique-face rule (face with 1 owner = boundary, 2 owners = internal; connexity-block scoping; rollback on BuilderSolid ≠ 1 area) | `combine` (exact weld + tube merge) + merge_coplanar_faces (STEP dump only) | **adopt** the owner-count rule as our cell-fusion primitive: fusing selected cells = drop section faces owned by 2 kept cells, re-close via combine; scope by face-connected blocks; roll back the block on closure failure instead of emitting naked edges (complements NK-RESCUE). |
| Boundary pinning during unification (`CollectMaterialBoundaries` → `UnifySameDomain::KeepShapes`) | our sew/weld keys (whole-seg alias keys, forced-node eps caps) | **adopt** the principle: when dissolving interior edges, explicitly pin the region-boundary key set first — never let a weld/merge step touch keys that are the outline of the merged region. |
| MakerVolume universe-box trick (all faces both orientations + enclosing box; remove the one solid touching box faces) | winding-number `contains_point` classification of scaffold pieces | **new-build (optional)**: for non-manifold/many-body cell extraction, add a synthetic outer box so the infinite cell is identified by marker faces, not by winding sign at infinity — robust when operands are open or marginally leaky. |
| Options plumbing: fuzzy clamp `max(f, 1e-7)`, glue tiers, non-destructive copy-on-write, `SetToFillHistory` switch, options-on-API/filler-disposable | our env-gate flags (SESSION_NO_MERGE, SESSION_NO_FLOOD, …) + per-call `tolerance` param | **adopt shape**: a `BooleanOptions{fuzzy, glue, safe, fill_history}` struct on `boolean()/split_with` replacing ad-hoc env gates for the stable knobs (env stays for experiments); clamp fuzzy at our TOLERANCE floor like OCCT clamps at 1e-7. |
| Argument order = same-domain geometry ownership (MakePeriodic::SplitShape feeds tools FIRST so coinciding parts take tool geometry) | `combine`'s exact weld + section-edge aliasing across operand copies | **adopt**: codify a deterministic ownership rank (operand A wins) when welding coincident geometry, instead of first-encountered order — OCCT proves rank-determinism is what makes twin geometry EXACTLY identical, which is what our exact-weld keys depend on. |
| Glue tiers as intersection skips (GlueShift: skip FF; GlueFull: skip VF/EF/FF) | chairs-family repeated cuts where operands share known-coincident planar walls | **new-build (later)**: a "glued" fast path in build_section_scaffold that trusts caller-declared coincident face pairs and skips their SSI — matches our zzzz-filter spirit; unchecked-by-design like OCCT. |
| `SectionEdges()` recovering section edges + `HasAncestorFaceOn1/2` (edge → generating face pair from InterfFF) | scaffold chains already know their (surfA, surfB) parents | **adopt**: expose (faceA, faceB) parentage per section edge in the public result — we have the data in the scaffold; OCCT shows it is part of the industry contract (Section API). |
| History switch semantics (empty answers when off; suspended while intermediate result will be discarded) | no history yet; MEMORY notes BOP2 pool referencing | **new-build (later)**: when we add Modified/Generated/IsDeleted, adopt the kill-switch + overlay-map composition (`LocModified` chain) rather than a monolithic history log. |
