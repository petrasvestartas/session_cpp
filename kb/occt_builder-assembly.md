# builder-assembly

Implementation spec extracted from OCCT sources at
`C:/brg/compas_occt/external/occt/src/occt/src/ModelingAlgorithms/TKBO/` (BOPAlgo + BOPTools).
Scope: face splitting (BuilderFace + WireSplitter), solid assembly (BuilderSolid + ShellSplitter),
solid-image pipeline (Builder_2/_3), BOP cell selection (BOP.cxx, Builder.cxx BuildBOP), Section.

---

## STAGE PIPELINE

Ordering is the literal call order of `BOPAlgo_BOP::PerformInternal1` (BOPAlgo_BOP.cxx:422-579),
which is the general-fuse ladder of `BOPAlgo_Builder::PerformInternal1` (BOPAlgo_Builder.cxx:310-446)
plus op-specific `BuildShape`. Every stage consumes/produces the four global maps
(`myImages`, `myShapesSD`, `myOrigins`, `myInParts` — see DATA STRUCTURES).

### 0. Entry + argument checking
- `BOPAlgo_BOP::Perform` (BOPAlgo_BOP.cxx:364-409): concatenates Arguments+Tools into one list,
  runs `BOPAlgo_PaveFiller` (interference stage, out of scope here), then `PerformInternal`.
- `BOPAlgo_BOP::CheckData` (BOPAlgo_BOP.cxx:106-210): op-dimension legality table:
  FUSE: every operand homogeneous and dimMax[0]==dimMax[1];
  CUT: max dim(args) <= min dim(tools); CUT21 mirrored; COMMON: anything.
  Records `myDims[2]` (min dims per group). Empty shapes → warning `AlertEmptyShape`.
- `BOPAlgo_BOP::TreatEmptyShape` (BOPAlgo_BOP.cxx:214-319): early-out table when a whole group
  is empty: FUSE → result = the single valid shape (if >1 shape, fall through so they get split);
  CUT → result = objects (only if exactly 1, else fall through); CUT21 → tools; COMMON → empty.
  Runs when: any AlertEmptyShape present. Inputs: raw args. Output: myShape, skips all stages.

### 1. FillImagesVertices — `BOPAlgo_Builder_1.cxx:40-67`
For each DS same-domain vertex pair (nV → nVSD from `myDS->ShapesSD()`):
`myImages[V] = [VSD]`, `myShapesSD[V]=VSD`, `myOrigins[VSD] += V`.
Then `BuildResult(VERTEX)` (Builder_1.cxx:130-168) appends images (or untouched originals) into
the growing compound `myShape` with a fence map.

### 2. FillImagesEdges — `BOPAlgo_Builder_1.cxx:71-126`
For each source edge with pave blocks: image list = the `RealPaveBlock` edge of each pave block
(common blocks collapse aliases to a single representative edge — this is the "shared section
edge" mechanism). If a pave block is a common block, also `myShapesSD[splitEdge] = realEdge`.
Small edges without pave blocks get an EMPTY image list → silently vanish from the result.
Then `BuildResult(EDGE)`.

### 3. FillImagesContainers(WIRE) — `BOPAlgo_Builder_1.cxx:172-193, 221-276`
Rebuild each wire from images of its edges only if some sub-shape actually changed
(`FillImagesContainer`: identity image `[self]` counts as unchanged). Split edges re-oriented
against parent with `IsSplitToReverseWithWarn`. Then `BuildResult(WIRE)`.

### 4. FillImagesFaces — `BOPAlgo_Builder_2.cxx:215-229`
Three sub-stages: `BuildSplitFaces` (90% of budget), `FillSameDomainFaces`, `FillInternalVertices`.

#### 4a. BuildSplitFaces — `BOPAlgo_Builder_2.cxx:233-555`
Per source face `i` with `myDS->HasFaceInfo(i)`:
- Gather from `BOPDS_FaceInfo`: `PaveBlocksIn()` (edges inside face), `PaveBlocksOn()`
  (split boundary), `PaveBlocksSc()` (section edges), and `AloneVertices` (vertices with no edge).
  If all four empty → face untouched, skip (no image).
- **Draft fast path** (lines 298-351): if no IN and no Sc pave blocks: if additionally no wire was
  modified and no INTERNAL wires → skip entirely; else `BuildDraftFace`
  (Builder_2.cxx:1052-1189) rebuilds the same wire structure substituting split edges:
  * INTERNAL edge met → return null face (must go to BuilderFace: internal edge may cut face in two);
  * split with multi-connected vertex (`HasMultiConnected`, Builder_2.cxx:1014-1045: a vertex
    referenced by >2 edges) → null face (case "bugs modalg_5 bug25245_1" — thin face split by vertex);
  * non-closed edge whose split repeats (edges *unified* during intersection) → null face;
  * closed (seam) parent whose split lost closedness → `DoSplitSEAMOnFace(aSp, theFace)` adds
    the second pcurve;
  * split reversed vs parent → `IsSplitToReverseWithWarn` → `Reverse()`.
  Non-null draft face becomes the single image; else falls to the full path.
- **Full path — edge set assembly** (lines 353-499), face forced FORWARD:
  * boundary edges without images: INTERNAL orientation → append TWICE (FORWARD + REVERSED),
    else append as-is;
  * boundary edges with images: degenerated → keep parent orientation; INTERNAL → twice F+R;
    **closed/seam edges** (surface closed in U or V, `BRep_Tool::IsClosed(aE,aF)`, and the edge is
    the matching iso — `IsEdgeIsoline`): each split deduped by fence map, second pcurve installed
    (`DoSplitSEAMOnFace(aSp,aF)`, retry `DoSplitSEAMOnFace(aE,aSp,aF)`, else warning
    `AlertUnableToMakeClosedEdgeOnFace`), then appended TWICE F+R;
    ordinary split → orient like parent, `IsSplitToReverseWithWarn` → maybe `Reverse()`, append once;
  * IN pave-block edges: append TWICE (FORWARD + REVERSED) — line 469-480;
  * Section pave-block edges: append TWICE (FORWARD + REVERSED) — line 484-494.
  This **both-orientation insertion** is the load-bearing convention: every interior edge offers
  one traversal per side, so the wire walk consumes each side exactly once and the face splits
  tile the face without overlap.
  * `BRepLib::BuildPCurveForEdgesOnPlane(aLE, aFF)` speedup for planar faces (non-destructive off).
- Run one `BOPAlgo_BuilderFace` per face (parallel vector `BOPAlgo_SplitFace`), collect
  `aBF.Areas()` into `myImages[face]`, restoring REVERSED on all images if the source face was
  REVERSED (lines 527-552).

#### 4b. FillSameDomainFaces — `BOPAlgo_Builder_2.cxx:580-925`
Only runs if `myDS->InterfFF()` non-empty (Face/Face interferences exist).
- Zero-thickness guard (lines 597-649): map every face (and its splits, propagated) to its parent
  SOLID; two faces with the same parent solid are NEVER same-domain ("that would imply a
  zero-thickness interior in a single operand").
- Candidate faces = both faces of every FF interference, sorted by DS index (deterministic SD
  representative selection).
- Group by **edge-set signature**: `AddEdgeSet` (lines 562-576) builds a `BOPTools_Set` of ALL
  edges of the face(split) → `IndexedDataMap<BOPTools_Set, list<face>>`. Only groups with >= 2
  faces continue.
- Per pair in a group: skip if same parent solid; if BOTH are planar with finite bbox →
  declared SD **without geometric check** (lines 707-718, 780-784); else queue
  `BOPTools_AlgoTools::AreFacesSameDomain(F1,F2,ctx,fuzzy)` (BOPTools_AlgoTools.cxx:1139+ —
  take interior point of F1, project on F2, valid ⇒ SD), run parallel.
- Confirmed pairs → union-find (`BOPAlgo_Tools::FillMap` + `MakeBlocks`); per block the SD
  representative = original face with minimal DS index (originals get a self-image bound at that
  moment, lines 850-866), else first face. `myShapesSD[each]=rep`; every image list entry is
  rewritten to the representative and `myOrigins[rep] += sourceFace` (lines 884-921).

#### 4c. FillInternalVertices — `BOPAlgo_Builder_2.cxx:929-1008`
For each source face's alone vertices × its image faces: `ComputeVF(V,F)` (parallel `BOPAlgo_VFI`);
flag==0 ⇒ vertex INTERNAL in that split → `BRep_Builder().Add(face, vertexINTERNAL)`.

### 5. BuilderFace — `BOPAlgo_BuilderFace.cxx` (invoked per face from 4a)
`Perform` (117-148): CheckData → PerformShapesToAvoid (1%) → PerformLoops (10%) →
PerformAreas (80%) → PerformInternalShapes (9%).

#### 5a. PerformShapesToAvoid — 152-235 (dangling-edge fixpoint)
Loop to fixpoint: build vertex→edges map over non-avoided edges;
- vertex with exactly 1 edge: edge → avoid (unless degenerated or the vertex is INTERNAL);
- vertex with exactly 2 entries which are the SAME edge (`IsSame`) and edge endpoints differ →
  both entries avoided (an edge doubled at a non-loop vertex is a stub).
Avoided edges are excluded from the wire walk and become candidates for internal wires.

#### 5b. PerformLoops — 239-383
- `BOPAlgo_WireEdgeSet aWES` (face + start edges = myShapes minus avoided; WireEdgeSet.hxx:32-60);
- `BOPAlgo_WireSplitter` on the WES (see stage 6); its output wires → `myLoops`;
- post: any input edge not consumed by a loop → added to `myShapesToAvoid`;
- internal wires (327-383): connect the avoided edges by shared vertices into wires
  (all edges, orientation preserved), `myLoopsInternal`.

### 6. WireSplitter — `BOPAlgo_WireSplitter.cxx` + `_1.cxx`
`Perform` (90-117): `MakeConnexityBlocks(startEdges, VERTEX, EDGE)`
(BOPTools_AlgoTools.cxx:187-256). A block is **regular** iff no start element was passed twice
(both-orientation duplicates make it irregular) AND every connection vertex has valence exactly 2.
`MakeWires` (163-219): regular block → `MakeWire` directly (single loop);
irregular block → `SplitBlock` (parallel).

#### SplitBlock — `BOPAlgo_WireSplitter_1.cxx:112-354`
1. Build `mySmartMap: vertex → list<BOPAlgo_EdgeInfo>` (WireSplitter.lxx:22-69; fields: edge,
   `myInFlag` = vertex orientation REVERSED ⇒ edge comes IN at this vertex, `myPassed`,
   `myIsInside`, `myAngle`). Edges without pcurve on the face are skipped. `aMS` set tracks
   single-occurrence edges: an edge added twice (i.e. fed F+R = section/inside edge) is removed
   from `aMS`, so `IsInside = !aMS.Contains(edge)` (line 309) — **boundary edges are the
   single-orientation ones**.
2. `bNothingToDo` shortcut (199-295): every vertex has exactly 1 in + 1 out AND no two distinct
   list entries share a TShape → emit single wire.
3. Angles (300-318): for every (vertex, edge-info): `Angle2D` (768-840) — tangent direction of the
   pcurve at the vertex parameter, sampled at `aTV ± dt` with
   `dt = max(Resolution(2*Tolerance2D), PConfusion)`, widened by curvature
   (`acos(R/(R+tol2d))`) and clamped by `aTX = 0.05*(range)` (case chl/927/r9, min branch 5e-5);
   IN edges use vector (P1→P), OUT edges (P→P1); angle measured CCW from +X in [0,2π).
4. `RefineAngles` (905-1029): at each vertex with EXACTLY 2 boundary infos (1 in aA2, 1 out aA1):
   sector `aDelta = ClockWiseAngle(aA2,aA1)`; every non-boundary OUT edge whose angle falls
   outside the sector is re-derived by intersecting its pcurve with the boundary direction lines
   (`RefineAngle2D` 1033-1125: `Geom2dInt_GInter`, `aTolInt=1e-10`, walk `aCf=0.01` of remaining
   param, reject intersections farther than `MaxDT=0.3*range`); if refine fails and there are
   exactly 2 interior edges, snap to `aA1 + Precision::Angular()` / `aA2 − Precision::Angular()`.
   IN edges get the refined angle + π. Purpose: p-curves convergent (tangential) at a node.
5. `Path` walk (358-617), started once per unpassed OUT edge-info:
   - push (edge, startVertex, 2D coord); mark Passed; advance to other vertex aVb;
   - **loop-closure scan** (426-524): walk the stack backward; a previous stack vertex `IsSame(aVb)`
     closes a loop, but on a *closed* (seam-touching) vertex the 2D coords must also agree:
     `d² < (2*Tolerance2D)²` AND `|du| <= 2*UTolerance2D` AND `|dv| <= 2*VTolerance2D`
     (Tolerance2D 859-881: max(UResolution(tolV), VResolution(tolV), tolV3D), ×1.1 on B-spline
     surfaces). Emitted loop must contain at least one non-degenerated edge; a 2-edge loop of the
     same edge twice is discarded (iPriz). Stack is truncated to the closure point and the walk
     resumes from there; empty stack → return.
   - **next-edge selection** (527-611): among not-passed OUT infos at aVb:
     `iCnt = NbWaysOut` == 0 → dead end, abandon; == 1 → forced;
     candidate == the edge we came on (same TShape) → angle := 2π (last resort);
     on a closed vertex, candidate 2D start (`Coord2dVf`) must be within aTol2D² of arrival point;
     otherwise `angle = ClockWiseAngle(angleIn, angleOut)` (621-659: dA = (AIn+π) − AOut mod 2π;
     `dA <= 0 → +2π`; `dA <= 1e-14 → 2π`), pick strict minimum with eps = `Epsilon(1.)`;
     **boundary override**: if the current edge is boundary (`!IsInside`) and EXACTLY ONE candidate
     is inside (`aNbWaysInside == 1`) → take that one regardless of angle (589-604). This is the
     rule that peels split faces off the outer boundary at tangential contacts.
   The min-clockwise-angle rule makes every emitted loop the tightest possible ⇒ output faces tile.

### 7. BuilderFace::PerformAreas — 387-614 (growth/hole classification, 2D)
- No loops: if face is infinite → natural-restriction face, else nothing.
- Per loop-wire: make candidate face (same surface `aS`, location, tolerance `BRep_Tool::Tolerance(myFace)`);
  growth test: fast `IsGrowthWire` (898-913: wire contains an edge already claimed by a hole)
  else `!myContext->FClass2d(aFace).IsHole()`. Growths → `aNewFaces`, holes → `aHoleFaces` +
  their edges → `aMHE`.
- Hole→growth assignment: 2D UV-bbox BVH (`BOPTools_Box2dTree`) over holes; for each growth face
  select intersecting holes; `IsInside(hole, growth)` (842-894: middle point of the 2D curve of the
  first non-degenerated hole edge NOT shared with the growth face, classified by `FClass2d`;
  shared edge ⇒ not inside); if a hole matches several growths, the **innermost growth wins**:
  `if (IsInside(aFace, *pFaceWas)) *pFaceWas = aFace` (524-535).
- Unassigned holes + face bbox open in any direction → they become holes of a new unbounded face
  (558-581). Hole wires are added to their growth face and the face's FClass2d re-inited (593-609).

### 8. BuilderFace::PerformInternalShapes — 618-778
Classify `myLoopsInternal` edges into the produced areas via 2D bbox BVH + `IsInside`;
inside edges grouped into INTERNAL-orientation wires (`MakeInternalWires` 782-838, vertex
connexity) and added to the face. Leftover edges → warning `AlertFaceBuilderUnusedEdges`.

### 9. FillImagesContainers(SHELL) then FillImagesSolids — `BOPAlgo_Builder_3.cxx:60-93`
`FillIn3DParts` → `BuildSplitSolids` → `FillInternalShapes`.

#### 9a. FillIn3DParts — Builder_3.cxx:97-263
- Collect classify-candidates: every face image (fence-deduped) or untouched face (with its DS box).
- Per solid: `BuildDraftSolid` (267-368): rebuild shells substituting face images
  (`IsSplitToReverseWithWarn` orientation fix); INTERNAL faces diverted to `aLIF` list;
  empty shells dropped. Solid box built by `BuildBndBoxSolid` (checks inverted).
- `BOPAlgo_Tools::ClassifyFaces(faces, draftSolids, …)` (BOPAlgo_Tools.cxx:1622+):
  BVH of face boxes; per solid (`BOPAlgo_FillIn3DParts`, perform ~1380-1519):
  * skip faces that are ON the solid (its own draft faces, `aMSF`);
  * build **connexity blocks that do not cross the solid's boundary edges**
    (`MakeConnexityBlock` 1555-1615, stop at edges of the solid `theMEAvoid`; a face touching a
    boundary edge is remembered as `aFaceToClassify` — the best representative);
  * fast reject: any block vertex box outside solid box → whole block OUT;
  * classify ONE face per block: `BOPTools_AlgoTools::IsInternalFace(face, solid, MEF, Precision::Confusion(), ctx)`
    (BOPTools_AlgoTools.cxx ~820-891): find an edge of `face` shared with 1 or 2 solid faces →
    `IsInternalFace(face, edge, F1, F2)` (939-990) → `GetFaceOff` radial test (see stage 10);
    result 2 (ambiguous) → fallback `ComputeState` point classification;
  * whole block inherits the verdict (`myInFaces`);
  * degenerate case: draft solid with NO faces (all-internal source) ⇒ ALL candidates IN (1405-1417).
- Post (211-262): a solid with no IN faces and unmodified shells is skipped (no split needed);
  else `theDraftSolids[solid]=draft` and `myInParts[solid] = IN faces + own INTERNAL faces`.

#### 9b. BuildSplitSolids — Builder_3.cxx:413-618
- Non-interfered solids: record `BOPTools_Set` face-signature into `aMST` (SD-solid detection pool).
- Per interfered solid: if `myInParts` empty ⇒ image = draft solid as-is.
  Else **SFS (shell face set)** = all draft-solid faces (orientation preserved) + every IN face
  TWICE (FORWARD + REVERSED) (491-511) → `BOPAlgo_BuilderSolid` (parallel `BOPAlgo_SplitSolid`).
- Report merge converts BuilderSolid errors to warnings, attaching (solid + alert shape) compound.
- Images bound; per result solid, `BOPTools_Set` signature looked up in `aMST`:
  already present ⇒ same-domain solid, reuse the previously-registered shape (`aMST.Added`),
  `myShapesSD[newSolid]=representative` (581-616). Origins updated.

### 10. BuilderSolid — `BOPAlgo_BuilderSolid.cxx`
`Perform` (76-125): PerformShapesToAvoid (1) → PerformLoops (10) → PerformAreas (80) →
PerformInternalShapes (9).

#### 10a. PerformShapesToAvoid — 129-219 (dangling-face fixpoint)
Fixpoint loop on edge→faces map (degenerated edges skipped):
- edge with 1 face: face → avoid (unless edge INTERNAL);
- edge with 2 entries of the SAME face: avoid both, unless the edge is closed/seam on that face
  (`BRep_Tool::IsClosed(aE,aF1)`) or INTERNAL.

#### 10b. PerformLoops — 223-393
- Infinite faces each get a singleton shell immediately (241-251).
- `BOPAlgo_ShellSplitter` over the rest; failure ⇒ warning AlertShellSplitterFailed + abort of loops.
- Unconsumed faces → `myShapesToAvoid`; those are then chained edge-wise into
  **internal shells** `myLoopsInternal` (338-392).

#### ShellSplitter — `BOPAlgo_ShellSplitter.cxx`
`Perform` (137-149): `MakeConnexityBlocks(faces, EDGE, FACE)`; regular blocks (every edge valence 2,
no doubled input) → `MakeShell` + `BOPTools_AlgoTools::OrientFacesOnShell` + `Closed(true)`
(683-698); irregular → `SplitBlock` (parallel).

`SplitBlock` — 153-421:
1. **Erosion loop** (185-222): repeatedly drop faces having a free edge (edge valence 1, not
   degenerated, not INTERNAL) until fixpoint — open flaps can never close a shell.
2. **Boundary-face marking** (229-245): input list may contain a face twice (F+R, from the doubled
   IN faces); the toggle `if (!aBoundaryFaces.Add(aF)) Remove(aF)` leaves exactly the
   single-occurrence faces marked as *boundary* (the doubled ones are interior/section walls).
3. **Shell growth** (251-369): seed face → for each edge of the growing shell that is still free
   inside the shell (`aMEFP` valence 1), gather candidate mates: faces containing the edge with
   **opposite orientation** (`GetEdgeOff`, BOPTools_AlgoTools.cxx:1107-1135) and not yet used;
   * boundary override: current face boundary + exactly one candidate interior ⇒ take it
     (316-337, 351 — mirrors the WireSplitter rule);
   * 1 candidate ⇒ take it; >1 ⇒ `GetFaceOff` radial minimal-angle selection (see below);
   selected face added to shell + `aMEFP` updated.
4. **RefineShell** (443-617): find *stop edges* `aMEStop`: edge with >2 faces; edge with 2 faces
   using it with the SAME orientation (non-manifold gluing); edge INTERNAL in enough faces that
   the doubled count exceeds 2. Flood the shell faces avoiding stop edges → sub-shells.
5. Sub-shells that are `BRep_Tool::IsClosed` → `myLoops` (result); open sub-shells give their
   faces BACK to the pool (`AddedFacesMap.Remove`) — "faces of one shell might be needed for
   building the other" (406-419); loop continues until all faces taken.

#### GetFaceOff — `BOPTools_AlgoTools.cxx:994-1103` (the radial engine)
Given base (edge E1 as used by face F1) and candidates (E-in-face, face):
- mid-parameter point `aPx`, 3D tangent `aDTgt`; reference plane through aPx normal to tangent;
- `aDt3D = MinStep3D` (2243+): `aDt = 2*(tolE+tolF)` maxed over candidates, floor `5e-6`
  (sphere/freeform floor `5e-4`; radius correction for R>100: `sqrt(d² + 2 d R)`, `d = 10*PConfusion()
  = 1e-8`); also detects faces too small for the step (`bSmallFaces` → skip circle search);
- `GetFaceDir` (2118-2160): face normal at edge (reversed for REVERSED face), bi-normal
  `aDB = N ^ T`, then **refined into the face interior** by `FindPointInFace` (2168-2239):
  project `P + 2*tolE*aDB` then iterate `P + aDt*aDB` → project on surface → project on section
  plane, up to 15 iterations, success when distance < `Precision::Angular()` (scaled
  `5e-16*|P|` when |P|>1000); on failure fall back to hatcher
  (`GetApproxNormalToFaceOnEdge`);
- candidate tangent flipped when candidate edge orientation != base (`aDTgt2`);
- angle = `AngleWithRef(aDBF, aDBF2, aDTF)` with `aDTF = N1 ^ B1`;
- special cases at |angle| < Precision::Angular(): candidate == F1 (same TShape, other wire use)
  ⇒ π; `IsSame(F1)` ⇒ 2π; different face with failed bi-normal ⇒ 2π (do not prefer unreliable);
- normalize negative to +2π; strict minimum wins; **ambiguity flag**: any candidate angle
  < `Precision::Confusion()` or within `Precision::Confusion()` of the current minimum ⇒
  return false (caller treats classification as unreliable → point-classification fallback).

### 11. BuilderSolid::PerformAreas — 397-598 (growth/hole classification, 3D)
- Per shell: `IsGrowthShell` fast check (contains a face of a known hole) else
  `!IsHole(shell)` — `IsHole` (823-831): `BRepClass3d_SolidClassifier::PerformInfinitePoint(RealSmall())`,
  state IN ⇒ hole (inward-oriented shell). Growth ⇒ new solid; hole ⇒ pool + faces to `aMHF`.
- Hole→solid assignment: 3D bbox BVH; `IsInside(hole, solid)` (835-860): `ComputeState` of the
  hole's first face vs solid with `Precision::Confusion()`, bounds = solid edges; innermost
  enclosing solid wins (`if (IsInside(aSolid, *pSolidWas)) *pSolidWas = aSolid`, 517-528);
  hole shells added into their solids, `SolidClassifier` reloaded.
- Orphan holes become standalone solids with a **Whole** bounding box (infinite void solids,
  579-597) — myBoxes bookkeeping for later ClassifyFaces calls.

### 12. BuilderSolid::PerformInternalShapes — 602-759
Internal-shell faces classified into result solids via `BOPAlgo_Tools::ClassifyFaces`
(reusing `myBoxes`); IN faces grouped into INTERNAL shells (`MakeInternalShells` 763-819, edge
connexity, faces re-oriented INTERNAL) and added into the solids. Unclassified faces → warning
**`BOPAlgo_AlertSolidBuilderUnusedFaces`** (consumed later by `CheckArgsForOpenSolid`). If there
were no growth solids at all: one solid made from all internal shells (633-651).

### 13. FillInternalShapes — Builder_3.cxx:622-887
Loose args (vertices/edges/wires) + INTERNAL vertices/edges owned by source solids
(`OwnInternalShapes` 891-905: direct non-shell children): classified into split solids via
`ComputeStateByOnePoint(shape, solid, 1.e-11, ctx)`; state IN ⇒ added INTERNAL. If the receiving
solid is an unmodified original, a NEW solid image is created for it at that moment (844-869).

### 14. BOP result assembly — `BOPAlgo_BOP::BuildShape` (BOPAlgo_BOP.cxx:871-1093)
- 3D×3D pre-check `CheckArgsForOpenSolid` (1382-1556): a source solid is "open" if its non-internal
  faces have a non-degenerated, non-seam, non-internal edge with face count < 2; combined with
  either a BuilderSolid unused-faces warning for that solid or new INTERNAL faces in the splits ⇒
  use the alternative **BuildBOP** path (Builder.cxx:479-885, below) which rebuilds from selected
  faces (loses solid modification history). On its failure, fall through to normal path.
- Normal path: `BuildRC` then (FUSE 3D) `BuildSolid`, else container reconstruction.

#### BuildRC — BOPAlgo_BOP.cxx:583-867 (cell selection per operation)
- **FUSE**: result = all `myShape` (GF result) sub-shapes of `TypeToExplore(myDims[0])`
  (0→VERTEX,1→EDGE,2→FACE,3→SOLID), fence-deduped. (SD merging already collapsed duplicates.)
- **COMMON / CUT / CUT21**: build `aMArgsIm` / `aMToolsIm` = the images (splits) of every
  building element (per-dimension `TypeToExplore(Dimension(subshape))`) of each side; untouched
  SOLIDS additionally keyed by `BOPTools_Set` face-signature (`aMSetArgs/aMSetTools`) so that a
  split solid equals an untouched solid with identical faces.
  * roles: `aMIt = bCut21 ? toolsIm : argsIm`; `aMCheck` = the other; membership test
    `bContains = aMCheckExp.Contains(aS)`, for solids fallback to set-signature.
  * COMMON expands BOTH maps down to `iDimMin = min(myDims)` (a shared edge of two faces counts);
    keeps aS iff contains; then a de-nesting filter keeps highest-dimension shapes first
    (786-809, fence map with sub-shape registration).
  * CUT/CUT21 expands only the check map; keeps aS iff !contains.
  * **degenerate-edge squats** (821-866): degenerated edges whose vertex is in the result, is not
    a NEW shape and has no interference are re-added (they vanish from splits otherwise).
- **FUSE 3D: BuildSolid** (1097-1378):
  * `aMFS`: face → owning result-solids via `MapFacesToBuildSolids` (1754-1784): INTERNAL faces
    skipped; a face key seen again with the SAME orientation does NOT append a solid (SD wall of
    stacked solids stays a boundary, count 1); seen with OPPOSITE orientation appends (interface
    wall between two fused solids, count 2 ⇒ dropped below);
  * solids untouched by the operation and sharing no face with others are moved to the result
    as-is (`aMUSols` + `BOPTools_Set` dedup `aDMSTS`, 1154-1206);
  * SFS = faces with exactly ONE solid reference (1217-1227) → `BOPAlgo_BuilderSolid` with
    `SetAvoidInternalShapes(true)`; errors ⇒ `AlertSolidBuilderFailed`;
  * COMPSOLID reconstruction from connexity blocks of new solids sharing splits of compsolid
    faces (1267-1377).
- **Containers (dim < 3 or non-fuse)** (914-1093): images of source WIRE/SHELL/COMPSOLID filtered
  by membership in `myRC`, reassembled per connexity block
  (WIRE: VERTEX/EDGE; SHELL: EDGE/FACE; else FACE/SOLID), re-oriented
  (`OrientEdgesOnWire` / `OrientFacesOnShell`), duplicates by identical content removed
  (`RemoveDuplicates` 1613-1719); then loose non-container splits appended.

### 15. BuildBOP — `BOPAlgo_Builder.cxx:479-885` (state-based cell assembly, same-domain op table)
Inputs: objects with state, tools with state (`TopAbs_IN` / `TopAbs_OUT` each; CUT = objects OUT +
tools IN; COMMON = IN/IN; FUSE = OUT/OUT). Requires GF (`myShape`, `myImages`, `myInParts`) done.
- Face harvesting (565-629): per group, per solid, per FORWARD/REVERSED face: its images oriented
  to match the source (`IsSplitToReverse` ⇒ reversed copy); collected in oriented + unoriented
  maps; solid's `myInParts` lists copied into `anINObjects` / `anINTools`.
- Selection loop (650-737) per group with `bTakeIN = (state == IN)`:
  * `bAvoidIN` (both OUT, i.e. FUSE): skip any face IN either group;
  * `bAvoidINforBoth` (states differ, i.e. CUT): skip faces IN both groups;
  * `isSameOriNeeded = (objState == toolsState)`;
  * face present in BOTH groups (same-domain wall): compare orientations via oriented fence
    `aMFenceOri`: second sighting same-oriented ⇒ `isSameOri`; keep one copy iff
    `isSameOriNeeded == isSameOri`, else discard both (`aMFToAvoid`) — **the SD face op table**:
    FUSE/COMMON keep equally-oriented coincident faces once, CUT keeps opposite-oriented pairs once;
  * face seen twice within ONE group: avoid iff `bTakeIN != isSameOriNeeded`;
  * ordinary face kept iff `bTakeIN == isINOpposite` (OUT-group keeps faces not inside the other
    operand; IN-group keeps faces inside the other operand); kept IN-own-group faces are added
    BOTH orientations; CUT tools-side (`bTakeIN && !isSameOriNeeded`) added REVERSED (719-735).
- `BOPAlgo_BuilderSolid` over the selected faces (754-759); result solids must contain at least
  one face from the harvested maps (773-788); unused faces → connexity blocks →
  `OrientFacesOnShell` → extra solids (796-833); `FillInternals` puts source INTERNAL parts into
  results unless bAvoidIN (835-871).

### 16. Section — `BOPAlgo_Section.cxx:100-414`
`PerformInternal1` runs only vertices+edges images then `BuildSection` (167-414):
result compound = per face: section vertices (`FaceInfo.VerticesSc`), IN vertices that are new or
interfered, section pave-block edges; plus **E/F common-block representative edges**
(244-269: pave blocks whose common block has faces); plus boundary images shared by >= 2 arguments
(occurrence counter `aMSI` over per-argument vertex/edge image closures, count > 1 ⇒ include,
283-376); vertices already covered by edges are absorbed (alone-vertex logic 380-411).

### 17. PostTreat — `BOPAlgo_Builder.cxx:450-475`
`BOPTools_AlgoTools::CorrectTolerances(myShape, aMA, 0.05, parallel)` +
`CorrectShapeTolerances(myShape, aMA)` — the tolerance-growth settlement pass over the result;
`aMA` (map-to-avoid) = all source V/E/F when NonDestructive, protecting inputs from tol updates.

---

## DATA STRUCTURES

- **`myImages`** : `DataMap<Shape, List<Shape>>` — source shape → its splits (post-SD substitution).
  Empty list = shape vanishes (micro edges). Identity list `[self]` = "split but unchanged"
  (SD originals). THE central artifact; every later stage reads it.
- **`myShapesSD`** : `DataMap<Shape, Shape>` — split → same-domain representative
  (vertices from DS, edges from common blocks, faces from FillSameDomainFaces, solids from
  BuildSplitSolids set-signatures). Guarantees coincident geometry has ONE identity in results —
  the mechanism our kernel lacks (same-domain subsystem).
- **`myOrigins`** : image → list of source shapes (inverse of myImages; feeds history).
- **`myInParts`** : `DataMap<Solid, List<Face>>` — per source solid: faces of OTHER arguments
  classified IN + its own INTERNAL faces. Computed ONCE in FillIn3DParts; reused verbatim by
  BuildSplitSolids and BuildBOP (cell selection never re-classifies).
- **`BOPDS_FaceInfo`** — per-face pave-block partition: `PaveBlocksIn` (edges interior to face),
  `PaveBlocksOn` (boundary splits), `PaveBlocksSc` (section edges), `VerticesIn/On/Sc`,
  alone vertices. Drives which faces need rebuilding and what goes into the WES.
- **`BOPAlgo_WireEdgeSet`** (WireEdgeSet.hxx:32-60): face + start edges + output wires. Trivial
  but load-bearing: keeps the face context for pcurve-based walking.
- **`BOPAlgo_EdgeInfo`** (WireSplitter.lxx:22-69): edge, `myInFlag` (vertex REVERSED = incoming),
  `myPassed` (consumed one traversal), `myIsInside` (fed twice = interior/section edge),
  `myAngle`. One instance per (vertex, edge-use): the both-orientation insertion materializes as
  two infos per interior edge per vertex.
- **`BOPTools_ConnexityBlock`** (BOPTools_ConnexityBlock.hxx): shapes + loops + `myRegular`
  (regular ⇔ all connections valence exactly 2 AND no doubled input element ⇒ can skip the
  angle walk entirely).
- **`BOPTools_Set`** — unordered sub-shape signature (edges of a face / faces of a solid); equal
  sets ⇒ same-domain candidates. Used in: FillSameDomainFaces grouping, BuildSplitSolids SD
  solids, BuildRC solid membership, BuildSolid untouched-solid dedup.
- **`BOPTools_CoupleOfShape`** — (edge-as-used-in-face, face) pair; candidate list for GetFaceOff
  so each candidate's tangent orientation is known relative to the base edge.
- **`aMFS`** (BuildSolid) — face → owner-solid list with the orientation trick: same-orientation
  duplicate keeps 1 owner (SD wall survives), opposite-orientation duplicate makes 2 owners
  (interface wall dies). Faces with exactly 1 owner form the fuse boundary.
- **`aFaceToParent`** (FillSameDomainFaces) — face/split → source solid; blocks SD-merge inside
  one operand (zero-thickness guard).
- **`myBoxes`** (BuilderSolid) — solid/shell → Bnd_Box; hole-solids get `SetWhole()`.

---

## CONSTANTS & TOLERANCES

| value | where | role |
|---|---|---|
| `Precision::Confusion()` = 1e-7 | GetFaceOff `anAngleCriteria` (AlgoTools.cxx:1042); IsInternalFace/ComputeState tol; ClassifyFaces; BuilderSolid IsInside | radial-angle ambiguity band; point-classification tolerance |
| `Precision::Angular()` = 1e-12 | GetFaceOff near-zero angle special cases (1065); FindPointInFace convergence `aDTol` (2182); RefineAngles snap offsets (998) | zero-angle detection |
| `5e-16 * |P|` | FindPointInFace (2186) | scale-aware convergence tol when |P| > 1000 |
| `dA <= 1e-14 → 2π` | ClockWiseAngle (WireSplitter_1.cxx:654) | going straight back = full turn (comment: was 1e-15) |
| `eps = Epsilon(1.)` ≈ 2.2e-16 | Path min-angle comparison (595) | strict-minimum guard |
| `aTol2D = 2 * Tolerance2D(V)`; `Tolerance2D = max(URes(tolV), VRes(tolV), tolV3D)`, ×1.1 for B-spline surface | WireSplitter_1.cxx:859-881, used 421, 577 | 2D same-vertex identification at seams |
| `2*UTolerance2D`, `2*VTolerance2D` (per-axis) | Path (457-464) | anisotropic seam check (U and V separately) |
| `dt = max(Resolution(2*tol2d), PConfusion())`, widen `acos(R/(R+tol2d))`, clamp `aTX = 0.05*range` (min branch `5e-5`) | Angle2D (786-820) | tangent sampling step; chl/927/r9 |
| `aCf = 0.01`, `aTolInt = 1e-10`, `MaxDT = 0.3*(t2−t1)` | RefineAngle2D (1053-1065) | tangential-node angle refinement |
| `MinStep3D: aDt = 2*(tolE+tolF)`, floor `5e-6`; sphere/freeform floor `5e-4`; R>100: `sqrt(d²+2dR)`, `d = 10*PConfusion() = 1e-8` | AlgoTools.cxx:2243-2316 | in-face walking step for bi-normal refinement |
| `FindPointInFace: 15 iters`, seed offset `2*tolE`, `anEps = Precision::SquareConfusion()` | 2168-2239 | bi-normal refinement loop |
| `RealSmall()` (≈ DBL_MIN) | IsHole PerformInfinitePoint (BuilderSolid.cxx:828) | infinite-point classification |
| `1e-11` | FillInternalShapes ComputeStateByOnePoint (Builder_3.cxx:836) | internal-shape settling |
| `0.05` | PostTreat CorrectTolerances (Builder.cxx:472) | max relative tolerance growth on result |
| face tol = `BRep_Tool::Tolerance(myFace)` verbatim | BuilderFace PerformAreas (396), BuildDraftFace (1063) | splits inherit parent face tolerance |
| planar + finite bbox ⇒ SD without check | FillSameDomainFaces (707-718) | fast SD for planar faces with equal edge sets |
| progress weights: faces ×20, solids ×50; Perform split 1/10/80/9 | Builder.cxx:301-303; BuilderFace/Solid | relative cost model (informative) |

---

## INVARIANTS

1. **Both-orientation feed**: every interior (IN/section/INTERNAL) edge enters the WES twice
   (F+R); every IN face enters the SFS twice (F+R). Downstream walkers may assume each side/use
   is available exactly once and never re-traverse (Passed flag / AddedFacesMap).
2. Every edge given to BuilderFace has a pcurve on the face (edges without one are skipped at
   SplitBlock:141); seam splits carry BOTH pcurves before insertion (DoSplitSEAMOnFace).
3. Split orientation matches its parent (IsSplitToReverse applied at every substitution point:
   faces 4a, wires 3, draft solids 9a, containers) — loop/shell ordering survives substitution.
4. BuilderFace outputs faces on the SAME surface object with the same location/tolerance,
   built from a FORWARD face; the caller restores REVERSED (4a end). Areas carry their holes
   already attached, innermost-nesting resolved.
5. After FillSameDomainFaces, `myImages` values contain exactly ONE representative per coincident
   face group, and two faces of one source solid are never merged.
6. `myInParts` is complete before any solid is rebuilt; BuildBOP and BuildSplitSolids never
   re-classify (all IN/OUT knowledge is frozen at FillIn3DParts).
7. Wire walk minimal-clockwise-angle + strict-min eps ⇒ emitted loops are the tightest ⇒ face
   areas tile the face without overlap; same for shells via GetFaceOff minimal radial angle.
8. Shells from the regular ShellSplitter path are closed by construction; irregular path emits
   only `BRep_Tool::IsClosed` shells — an open shell NEVER reaches PerformAreas.
9. GetFaceOff signals ambiguity (angle < 1e-7 or tie within 1e-7) instead of guessing; caller
   falls back to point-in-solid classification (`iRet == 2` path).
10. Every hole (wire or shell) is attached to its innermost enclosing growth; orphans are handled
    explicitly (unbounded face / whole-box void solid) — nothing is dropped silently; leftovers
    produce typed warnings (UnusedEdges / UnusedFaces) that later stages read as signals
    (CheckArgsForOpenSolid).
11. Result tolerances are settled once, at the end (PostTreat), with growth capped at 5% and
    source shapes protected under NonDestructive.

---

## PITFALLS (explicitly handled in source)

- **Thin face split by a vertex** (bug25245_1): a vertex used by >2 edges kills the draft-face
  fast path (HasMultiConnected) — the face may need to split even with no interior edges.
- **Edges unified during intersection**: two boundary edges mapping to the same split invalidate
  the draft wire (fence `aMEdges`); must re-run full BuilderFace.
- **Seam/closed edges**: exempt from valence-2-same-element stub removal
  (BuilderSolid PerformShapesToAvoid:196); split seams need a second pcurve, with a two-attempt
  fallback and a typed warning; wire walk needs per-axis 2D tolerance checks at seam vertices,
  and closure requires 2D agreement, not just 3D vertex identity.
- **Degenerated edges**: keep parent orientation; never counted as free/dangling; skipped in all
  valence logic; do not seed wires alone (Path `bHasEdge` check 437-445); "squats" (degenerated
  edges around kept vertices) re-added at BuildRC:821-866.
- **Angle ties**: returning along the arrival edge = 2π; `dA <= 1e-14` = 2π; strict-min with
  machine eps — prevents oscillation between coincident pcurves.
- **Tangential p-curves at a node**: RefineAngles/RefineAngle2D re-derives branch angles by
  2D intersection with the boundary sector when naive tangents lie outside it.
- **Boundary-vs-inside single-way override** (Path 589-604; ShellSplitter 316-337, 351): when
  standing on a boundary element and exactly ONE candidate is an interior (section) element, take
  it unconditionally — prevents the walk from sliding along the operand boundary at tangential
  contacts and guarantees section edges/faces are consumed.
- **GetFaceOff zero-angle special cases**: same TShape other-use = π (seam mate); IsSame face =
  2π; different face with failed bi-normal = 2π ("prevent incorrect selection of this unreliable
  face over a proper closing face").
- **Non-manifold branch edges**: RefineShell splits assembled shells at edges with >2 faces OR
  2 faces with same orientation OR internal-doubled count > 2.
- **Open-shell retry**: faces of non-closed candidate shells are returned to the pool because
  "faces of one shell might be needed for building the other".
- **Interface-wall suppression in fuse** (MapFacesToBuildSolids): duplicate face key with opposite
  orientation ⇒ 2 owners ⇒ excluded from SFS; with same orientation ⇒ 1 owner ⇒ kept (stacked SD
  wall remains a boundary).
- **Untouched-solid shortcut** with BOPTools_Set dedup — solids untouched by the operation are
  passed through without rebuild (identity + performance).
- **Open input solids** (CheckArgsForOpenSolid): detected via free-edge scan (ignoring
  degenerated/seam/internal edges) + BuilderSolid warnings + new INTERNAL faces; routed to
  BuildBOP face-selection assembly since BuilderSolid "cannot be expected to produce good splits".
- **Empty draft solid** (all faces INTERNAL): every candidate face classifies IN without
  computation (classifier would say IN for any point anyway).
- **Inverted solids**: bounding box set to Whole before classification.
- **Zero-thickness SD guard**: faces of the same source solid never same-domain.
- **Planar SD shortcut** only when the face bbox is finite in all 6 directions.
- **FillImagesEdges empty image list** = intentional deletion of micro edges (not a bug):
  "The small edges, having no pave blocks, will have the empty list of images".

---

## PORT MAP

Anchors: `session_cpp/src/brep_section.cpp` (`build_section_scaffold`),
`session_cpp/src/brep.cpp` (`split_with` / `split_by_uv_curves`, `combine`, classification tiers),
tolerances tol3 = diag*2e-3, exact weld 1e-7.

| # | OCCT mechanism | Our anchor | Action + design |
|---|---|---|---|
| 1 | **Both-orientation section-edge insertion** (BuildSplitFaces 469-494: IN/Sc edges fed F+R so each side is consumed once) | brep.cpp `split_with` — section chains fed into `split_by_uv_curves` | **already-equivalent (implicit)**: a planar UV arrangement gives both sides of an inserted curve by construction. **Adopt as audit**: post-split invariant "every kept section interval borders exactly 2 result faces of the operand (1 per side)" — a cheap detector for our SEGLOST/partial-run class. |
| 2 | **WireSplitter clockwise-angle walk** (SplitBlock/Path; EdgeInfo in/out + Passed; min ClockWiseAngle; 2π back-edge) | brep.cpp `split_by_uv_curves` face tracing | **already-equivalent** for the core (exact UV arrangement ordering ≡ angle sort). **Adopt** two rules verbatim: (a) back-edge = worst choice (2π), (b) strict-min with eps; both matter when trim-snapped section endpoints create near-duplicate branch directions. |
| 3 | **Boundary-vs-inside single-way override** (Path 589-604 `aNbWaysInside==1`; ShellSplitter 351) | split_by_uv_curves branch ordering at nodes where section chains meet trim loops; combine flood at common-block edges | **adopt (new rule)**: at a node on the trim boundary with exactly one section-curve branch, take the section branch regardless of angle. Directly targets our tangential-contact partial splits (the UV cut-node crossing snap fixed one instance; this is the general rule). |
| 4 | **RefineAngles / RefineAngle2D** (tangential pcurves at a node: re-derive angle by 2D intersection, aCf=0.01, tolInt=1e-10, MaxDT=0.3*range) | split_by_uv_curves node ordering for curved section chains | **new build**: when two branch tangents at a node differ by < angular band, re-sample each curve at 1% of its remaining parameter span (capped at 30%) and order by the resampled chords instead of tangents. One function at arrangement-node sort time. |
| 5 | **Dangling-element fixpoint** (BuilderFace PerformShapesToAvoid valence-1 edge pruning; BuilderSolid valence-1-face pruning; ShellSplitter erosion loop) | split_by_uv_curves input hygiene (today: zero-span collapse, SESSION_ZEROFILL); combine micro-edge collapse | **adopt**: iteratively prune fed section sub-curves with a free endpoint (valence-1 in the UV graph, trim edges exempt) BEFORE face tracing, to fixpoint. Converts "stalled march corrupts the split" into "stalled march is ignored" — our current interior-stall failure mode. |
| 6 | **Draft fast path** (BuildDraftFace + guards: multi-connected vertex, unified edges, INTERNAL edges) | split_with faces without runs (we already skip them) | **already-equivalent** for the skip; **adopt** the guards if we ever add a boundary-only rebuild path (vertex valence > 2 on the rebuilt loop ⇒ full split). |
| 7 | **Hole/growth loop nesting** (PerformAreas: signed classification via FClass2d, hole→innermost growth via 2-level IsInside, orphan holes → unbounded face) | split_with result-face assembly (loops currently taken as the arrangement emits them) | **new build**: per split face, classify each closed UV loop by signed area (growth vs hole), assign each hole to the smallest-area containing growth loop (point-in-loop test), attach; orphan hole = error signal. Needed the moment a cut produces a face with an island (chair leg through slab). |
| 8 | **Shell growth by mate-finding + radial GetFaceOff** (GetEdgeOff opposite-orientation mate; GetFaceOff: tangent-plane bi-normal, FindPointInFace walk step MinStep3D=2*(tolE+tolF) floor 5e-6/5e-4, min signed angle, π/2π specials, 1e-7 ambiguity ⇒ fallback) | combine — vertex-pair-keyed common-block tube merge + radial in-face-direction pre-pass | **replace/upgrade**: our radial pre-pass is the embryo of GetFaceOff. Build `get_face_off(edge, base_face, candidates)` exactly per spec (bi-normal refined by walking INTO the face, not just surface normal cross tangent — the walk is what makes it robust on curved faces) and use it as tier-0 mate pairing at every common-block edge before the connexity flood; ambiguity (< 1e-7) falls through to the existing volume-flux tier-3. |
| 9 | **RefineShell branch-edge barriers** (stop edges: >2 faces, 2 faces same orientation, internal-doubled) | combine connexity flood (symmetric-coverage 7/9 gate) | **adopt**: mark merged edges with face-count > 2 or same-orientation face pairs as flood barriers, then flood within barrier-bounded regions. This is the principled version of our asymmetric-span-crossing exclusion (rotation-robustness round 3) and should replace the heuristic gate long-term. |
| 10 | **Open-shell face recycling** ("faces of one shell might be needed for building the other") | combine NK-RESCUE tolerant mate-pair pass | **adopt the retry loop**: when a candidate closed region fails closure, release its faces back to the unassigned pool and re-run assembly with the remainder, instead of one-shot rescue. Cheap: loop until no new closed shell forms. |
| 11 | **Block classification** (FillIn3DParts: connexity blocks stopped at the solid's OWN boundary edges; ONE point classification per block; bbox pre-reject; empty-solid ⇒ all IN) | combine classification: connexity flood + winding-number contains + parity | **already-equivalent** in spirit (our flood = their block inheritance). **Adopt**: (a) stop block growth at edges lying ON the other operand (exactly our symmetric-coverage lesson, now principled), (b) classify one representative face per block and inherit — cuts winding-number calls by ~10x on chairs. |
| 12 | **Cell-selection tables** (BuildRC: CUT = own splits absent from other side's split set, COMMON = intersection with lower-dim expansion, solids compared by BOPTools_Set signature; degenerate squats) | brep.cpp boolean driver (face_outward_signs tiers → per-face keep/flip decisions) | **already-equivalent** for solid/solid cut/common/fuse via our IN/OUT flood; the *set-membership formulation* becomes relevant for mixed-dimension and multi-body operands (our xor/split matrix). **Adopt** the solid-by-face-signature identity for n-ary fuse dedup. |
| 13 | **Same-domain face table** (BuildBOP 681-713: `isSameOriNeeded = (objState==toolsState)`; coincident face in both operands kept ONCE iff orientation relation matches op, else dropped; single-group double kept iff `bTakeIN == isSameOriNeeded`; CUT keeps tool faces REVERSED) | **MISSING same-domain subsystem** (top gap) | **new build (highest value)**: after split, key every result face by canonical edge signature (sorted welded-vertex-pair ids); groups of 2+ across operands = coincident walls; apply verbatim: fuse/common keep one copy when normals agree, cut keeps one reversed copy when normals oppose, else drop both. This one table is what makes box-on-box / face-glued cases exact. |
| 14 | **AreFacesSameDomain + BOPTools_Set** (equal edge sets ⇒ geometric confirm by one interior point projection; planar+finite ⇒ no check) | MISSING same-domain subsystem | **new build** (companion to #13): `faces_same_domain(fa, fb)` = edge-signature equality, then midpoint-of-fa projected onto fb within max(tol_fa, tol_fb, fuzzy); planar shortcut included. Feeds #13's grouping. |
| 15 | **Zero-thickness guard** (two faces of one operand never SD) | same-domain subsystem (#13/#14) | **adopt**: signature groups must span operands; same-operand pairs are split artifacts (alias bugs), flag not merge. |
| 16 | **TreatEmptyShape table** | boolean driver entry (before scaffold/SSI) | **new build (trivial)**: empty-operand early-out per op (cut→A, common→∅, fuse→other), avoids feeding empty scaffolds into the pipeline. |
| 17 | **Per-entity tolerance settlement** (PostTreat CorrectTolerances cap 0.05; splits inherit parent face tol; MinStep3D derives steps from entity tols) | MISSING persistent per-entity tolerances; today flat tol3 = diag*2e-3 | **new build**: carry per-vertex/per-edge tol on combine output (init = max of merged children, then raise to measured 3D/pcurve deviation, cap 5% growth); replaces the global weld band at result-write and gives STEP output honest tolerances. |
| 18 | **Internal-shape settlement** (FillInternalVertices ComputeVF; FillInternalShapes 1e-11; PerformInternalShapes; MakeInternalWires/Shells) | not represented (kernel has no INTERNAL orientation) | **new build (low priority)**: keep alone vertices / floating section edges in a side-list on the result body instead of dropping; only matters for Section-op parity and exotic inputs. |
| 19 | **Section result assembly** (BuildSection: Sc vertices, new/interfered IN vertices, Sc pave-block edges, E/F common-block edges, boundaries shared by 2+ args) | brep_section `build_section_scaffold` output (chains) | **already-equivalent** for the edge core (our chains = Sc pave blocks); **adopt**: include E/F-tangency common-segment edges and section vertices as first-class result members when we expose a public `section()` op. |
| 20 | **Typed warnings as inter-stage signals** (AlertSolidBuilderUnusedFaces consumed by CheckArgsForOpenSolid to reroute strategy) | our env-gated debug prints ([SCAF-RUN] MATED etc.) | **adopt pattern**: make combine/classification emit machine-readable verdicts (unused-faces, open-shell, ambiguous-mate) that the driver can react to (e.g., auto-retry with scaffold off / mesh-CSG fallback) instead of log-only. |

### Priority (repo-actionable order)
1. #13+#14+#15 — same-domain subsystem (edge-signature SD faces + op table). Unlocks coincident-face
   cases wholesale; everything else assumes it.
2. #8 — GetFaceOff radial mate selection with FindPointInFace walk + 1e-7 ambiguity fallback,
   as tier-0 before volume-flux.
3. #5 + #3 — valence-1 fixpoint pruning of fed section curves + boundary/inside single-way rule:
   the two cheapest fixes for our stalled-march / tangential partial-split failure classes.
