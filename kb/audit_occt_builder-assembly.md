# audit — occt_builder-assembly.md vs real OCCT V8 source

Source tree: `/home/petras/code/code_cpp/OCCT` @ `37dd5686f2` (8.0.1.dev). All citations below are
relative to that root, with two path shorthands used throughout:
`BOPAlgo/` = `src/ModelingAlgorithms/TKBO/BOPAlgo/`, `BOPTools/` = `src/ModelingAlgorithms/TKBO/BOPTools/`.
Audit scope (session B): FillImagesFaces / FillSameDomainFaces, SD representative choice, BuildBOP
keep+reverse table incl. the ON-same / ON-opposite rows, CUT tool-face reversal, orientation bookkeeping.

---

## 1. VERDICT

**Faithful in structure, line numbers, and the top-level op table; three material errors and a set of
port-critical omissions in exactly the areas session B ports.**

Verified correct against source, line-for-line: stage order (`BOPAlgo/BOPAlgo_Builder.cxx:310-446`),
FillImagesVertices/Edges/Containers (`BOPAlgo/BOPAlgo_Builder_1.cxx:40-67,71-126,130-168,221-276`),
FillImagesFaces 9/0.5/0.5 split (`BOPAlgo/BOPAlgo_Builder_2.cxx:215-229`), BuildSplitFaces incl. the
F+R double-insertion of IN/Sc/INTERNAL edges (`Builder_2.cxx:469-494`, `420-427`), FillSameDomainFaces
zero-thickness guard / planar shortcut / min-index rep (`Builder_2.cxx:595-649,707-718,780-785,843-873`),
BuildBOP harvesting + selection loop (`BOPAlgo_Builder.cxx:565-629,650-737`), BuildRC cell selection
(`BOPAlgo/BOPAlgo_BOP.cxx:583-867`), CheckData/TreatEmptyShape tables (`BOP.cxx:106-210,214-319`),
MapFacesToBuildSolids orientation trick (`BOP.cxx:1754-1784`), PostTreat 0.05 cap
(`BOPAlgo_Builder.cxx:472`). Line numbers in the spec match this checkout exactly — it was written
against this version, not an older one.

Material errors:

* **E1 (wrong).** Spec §9a: "BuildDraftSolid … rebuild shells substituting face images
  (`IsSplitToReverseWithWarn` orientation fix)". The geometric reversal test is applied **only** to
  images that are same-domain (`if (myShapesSD.IsBound(aFx))`, `BOPAlgo/BOPAlgo_Builder_3.cxx:311`,
  test at `:321-326`). Ordinary splits take the parent's in-shell orientation verbatim —
  `aFx.Orientation(aOrF)` `Builder_3.cxx:334` — with **no** test at all. Spec §INVARIANTS-3
  ("IsSplitToReverse applied at every substitution point … draft solids 9a") is therefore false.
* **E2 (wrong).** Spec §15: "face seen twice within ONE group: avoid iff `bTakeIN != isSameOriNeeded`"
  reads as a terminal rule. In source that branch has **no `continue`** (`BOPAlgo_Builder.cxx:684-692`);
  control falls through into the `aMFenceOri` gate and the ordinary keep block at `:714-735`, so the
  face is *also* evaluated (and possibly added to `aMResFacesOri`) before being deleted by the
  `aMFToAvoid` filter at `:742-749`. Only the both-groups branch `continue`s (`:711`).
* **E3 (incomplete → wrong outcome).** Spec §15 SD row: "keep one copy iff `isSameOriNeeded == isSameOri`".
  Which copy survives is decided by `aMResFacesFence` (`BOPAlgo_Builder.cxx:700`): if the first group
  already emitted the face through the ordinary path, the second group's add is **suppressed**, so the
  surviving orientation is the *first* group's, not the SD-branch's `aFIm`. This changes the result
  normal for CUT vs CUT21 (worked example in §2.6).

Omissions that will silently break a port: hasher orientation-sensitivity (§2.1), `myShapesSD[rep]=rep`
self-binding (§2.4), the `isIN`-before-CUT-reversal precedence (§2.5), `AreFacesSameDomain` tolerance
formula (§2.9), face `myOrigins` only existing when `InterfFF` is non-empty (§2.10).

---

## 2. PORT-CRITICAL DETAILS THE SPEC MISSED

### 2.1 The whole SD/orientation mechanism is carried by *hasher choice*, not by code
`NCollection_...<TopoDS_Shape>` (default hasher) compares with `operator==` → `TopoDS_Shape::IsEqual`
= TShape **+ Location + Orientation** (`src/ModelingData/TKBRep/TopoDS/TopoDS_Shape.hxx:276,282`);
`NCollection_...<TopoDS_Shape, TopTools_ShapeMapHasher>` compares with `IsSame` = TShape + Location,
**orientation ignored** (`src/ModelingData/TKBRep/TopTools/TopTools_ShapeMapHasher.hxx:34-37`). The hash
function is orientation-blind in both cases, so only equality differs. In BuildBOP:

| map | decl | semantics |
|---|---|---|
| `aMObjFacesOri`, `aMToolFacesOri`, `aMResFacesOri`, `aMFenceOri` | `BOPAlgo_Builder.cxx:559,643,648` | **oriented** (IsEqual) |
| `aMObjFaces`, `aMToolFaces`, `anINObjects`, `anINTools`, `aMFence`, `aMFToAvoid`, `aMResFacesFence` | `:561,563,644,646` | **unoriented** (IsSame) |

A port that uses one identity for both collapses the ON-same/ON-opposite rows into a single row and
loses the CUT wall. `myImages`, `myShapesSD`, `myOrigins`, `myInParts` are all IsSame-keyed
(`BOPAlgo_Builder.hxx:501` and siblings) — image lookup is orientation-blind, so **orientation must be
re-derived at every use site**.

### 2.2 BuildSplitFaces: what actually reaches the image map
* Skip conditions, in order: not a FACE; `!myDS->HasFaceInfo(i)` (`Builder_2.cxx:275-279`);
  `!aNbPBIn && !aNbPBOn && !aNbPBSc && !aNbAV` (`:293-296`).
* Draft fast path is entered only when `!aNbPBIn && !aNbPBSc` (`:298`), and inside it: if `aNbAV==0`
  and no wire is in `myImages` and no first-edge-of-a-wire is INTERNAL → `continue` with **no image at
  all** (`:309-334`); `hasInternals` is computed from the **first edge of each wire only**
  (`itE.More() && itE.Value().Orientation()==INTERNAL`, `:320-321`) — not a full scan.
* Seam handling requires *both* `GeomLib::IsClosed(surface, BRep_Tool::Tolerance(aE), isU, isV)`
  (computed once per face, lazily, `:387-393`) **and** `BRep_Tool::IsClosed(aE,aF)` **and**
  `BOPTools_AlgoTools2D::IsEdgeIsoline` agreeing on the axis (`:397-404`). Only then are splits
  deduped by fence and fed F+R.
* `BuilderFace::SetFace` forces FORWARD internally (`BOPAlgo/BOPAlgo_BuilderFace.cxx:79-83`), so
  `aBF.Face()` is FORWARD and `myDS->Index(aBF.Face())` still resolves (Index is IsSame-keyed,
  `src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_DS.cxx:276-281`).
* Final orientation pass: images are stored carrying **the DS source face's orientation** — REVERSED is
  re-applied to every area iff `anOriF == TopAbs_REVERSED` (`Builder_2.cxx:535-552`). This is the
  baseline every later consumer re-derives from.

### 2.3 FillSameDomainFaces — candidate set and grouping (exact guards)
* Early `return` when `myDS->InterfFF()` is empty (`Builder_2.cxx:584-589`) — nothing else in the
  method runs, including the origins fill (see §2.10).
* Candidates: both indices of every FF interference, filtered by `myDS->HasFaceInfo(nF[j])`
  (`:672-675`) and an `NCollection_Map<int>` fence (`:677-680`), then `std::sort` on the **DS index**
  (`:687`). This sort is the only determinism anchor for the whole SD subsystem.
* Signature: `BOPTools_Set::Add(theS, TopAbs_EDGE)` (`Builder_2.cxx:567-568`) →
  `BOPTools/BOPTools_Set.cxx:124-180`: degenerated edges **excluded** (`:141-146`); INTERNAL sub-shapes
  appended **twice**, F and R (`:153-165`); hash = Σ of `TopTools_ShapeMapHasher{}(sub) % 432123 + 1`
  normalized by `NormalizedIds` (`:174-178`, `myUpper=432123` set at `:34`); equality = equal count +
  one-way inclusion under **IsSame** (`:96-119`). Consequence: two faces with the same edge set but
  opposite orientation have **equal** signatures — orientation is deliberately not part of SD identity.
* Planar shortcut needs `myContext->SurfaceAdaptor(F).GetType()==GeomAbs_Plane` **and** a bbox open in
  none of the 6 directions (`:711-717`); both faces of the pair must be flagged (`:768,780`); then
  `FillMap` directly, **no geometric check, no `AreFacesSameDomain` call**.
* Zero-thickness guard fires only when **both** faces have a known parent solid
  (`pParent1 && pParent2 && IsSame`, `:776-779`). `aFaceToParent` binds the **first** solid that
  contains the face (`:613-616`), so a face shared by two solids of one compsolid is attributed to one
  of them only; free faces (not in any solid) are never guarded. Split→parent inheritance is done in a
  separate collect-then-bind pass (`aPropagation`, `:619-648`) to avoid mutating the map under iteration.

### 2.4 SD representative: exact rule, plus the self-binding a port will forget
`Builder_2.cxx:843-873`. Per block: scan members, `nF = myDS->Index(aF)`; **only `nF >= 0` members are
candidates** — i.e. only *unsplit originals*, since splits are not in the DS. Each such original gets
`myImages.Bound(aF, {})->Append(aF)` **at that moment** (`:858`, an identity image marking
"split but unchanged"). Rep = min `nF`; if the block contains no original, rep = `aLSD.First()`
(`:872`), which is the block's BFS seed = the earliest-inserted key of `aDMSLS`
(`BOPAlgo/BOPAlgo_Tools.hxx:50-61`), i.e. determined by the `std::sort` at `:687` and the pair
enumeration order at `:764-792` — deterministic but *not* index-minimal.

Then `myShapesSD.Bind(aF, *pFSD)` for **every** member of the block **including the representative
itself** (`:876-881`) → `myShapesSD[rep] == rep`. This is load-bearing, not bookkeeping:
`BOPAlgo_Builder_3.cxx:311` branches on `myShapesSD.IsBound(aFx)`, so the representative takes the
geometric-reversal branch too. Omitting the self-binding silently switches that face to the
"copy parent orientation" branch.

### 2.5 The image rewrite pass (`Builder_2.cxx:886-921`)
Iterates **all** source faces in DS index order, not just interfered ones; rewrites each image in place
to its SD rep (`aFIm = *pFSD`, `:905-911`) and appends `myOrigins[aFIm] += sourceFace` for **every**
image (SD or not, `:913-919`). No dedup: if two splits of one face map to the same rep, the image list
holds the rep twice — OCCT tolerates this via fence maps downstream (`Builder_1.cxx:161`,
`Builder_3.cxx:139`), so a port must not assume image lists are duplicate-free.

### 2.6 BuildBOP — the complete keep/reverse table as actually coded
Operation → states (`BOPAlgo_Builder.hxx:214-250`): COMMON = IN/IN, FUSE = OUT/OUT, CUT = OUT/IN,
CUT21 = IN/OUT. Derived flags (`BOPAlgo_Builder.cxx:634-641`):
`bAvoidIN = (!objIN && !toolsIN)` (FUSE only), `bAvoidINforBoth = (objIN != toolsIN)` (CUT/CUT21 only),
`isSameOriNeeded = (objState == toolsState)` (true for FUSE and COMMON, false for CUT/CUT21).

Per group `i` (0 = objects, 1 = tools), per oriented face `aFIm`, in source order:

1. `isIN = anINMap.Contains(aFIm)` — face lies IN a solid of its **own** group;
   `isINOpposite = anOppositeINMap.Contains(aFIm)` — IN a solid of the **other** group (`:666-667`).
   Both maps are `myInParts` unions, IsSame-keyed, so orientation is irrelevant here.
2. `if (bAvoidIN && (isIN || isINOpposite)) continue;` — FUSE drops every interior wall (`:670-673`).
3. `if (bAvoidINforBoth && isIN && isINOpposite) continue;` — CUT/CUT21 drop doubly-interior walls (`:676-679`).
4. `if (!aMFence.Add(aFIm))` — second unoriented sighting (`:682`):
   * **not in `anOppositeMap`** → duplicate *within one group* (wall between two solids of the same
     operand): `if (bTakeIN != isSameOriNeeded) aMFToAvoid.Add(aFIm);` then **fall through** to 5
     (`:684-692`). FUSE (`bTakeIN=false`, needed=true) ⇒ avoided — two touching operand solids merge.
     COMMON (`true/true`) ⇒ kept, and each sighting adds its own orientation ⇒ two-sided wall survives.
   * **in `anOppositeMap`** → the ON / same-domain wall shared by both operands:
     `isSameOri = !aMFenceOri.Add(aFIm)` (`:696`) — true iff the *identical orientation* was already
     seen. Keep iff `isSameOriNeeded == isSameOri`, and only via
     `if (aMResFacesFence.Add(aFIm)) aMResFacesOri.Add(aFIm);` (`:697-704`) — i.e. **suppressed if the
     other group already emitted this face**. Else `aMFToAvoid.Add(aFIm)` (`:708`). `continue` (`:711`).
5. `if (!aMFenceOri.Add(aFIm)) continue;` (`:714`) — one traversal per oriented face.
6. Keep test `bTakeIN == isINOpposite` (`:719`), with reversal precedence (`:721-734`):
   * `isIN` → add **both** `aFIm` and `aFIm.Reversed()` (two-sided interior wall);
   * else if `bTakeIN && !isSameOriNeeded` → add **`aFIm.Reversed()` only** ← this is the CUT tool-face
     reversal (and the CUT21 object-face reversal);
   * else → add `aFIm` as-is. Then `aMResFacesFence.Add(aFIm)`.
7. Final: emit `aMResFacesOri` minus anything in `aMFToAvoid` (IsSame ⇒ **all** orientations of an
   avoided face die) (`:740-749`).

Resulting ON-face rows (F present in both operands; an ON face is never in either IN map, because
`ClassifyFaces` excludes the solid's own faces — `BOPAlgo/BOPAlgo_Tools.cxx:1389-1392`):

| op | states | isSameOriNeeded | ON-same (normals agree) | ON-opposite |
|---|---|---|---|---|
| FUSE | OUT/OUT | true | keep once, **objects' orientation** (objects passed step 6 first) | drop both |
| COMMON | IN/IN | true | keep once, objects' orientation | drop both |
| CUT (A−B) | OUT/IN | false | drop both | keep once, **objects' orientation** (objects kept it at step 6: `bTakeIN=false == isINOpposite=false`) |
| CUT21 (B−A) | IN/OUT | false | drop both | keep once, **tools' orientation** — objects fail step 6 (`bTakeIN=true != isINOpposite=false`), so `aMResFacesFence` is free and the SD branch emits the tool copy |

That asymmetry is the concrete consequence of E3 and is invisible in the spec.

### 2.7 Orientation bookkeeping at harvest time (`BOPAlgo_Builder.cxx:565-629`)
* Faces are taken via `TopExp_Explorer(solid, TopAbs_FACE)`, which **composes** shell→face orientation;
  faces whose composed orientation is neither FORWARD nor REVERSED (INTERNAL/EXTERNAL) are skipped
  (`:583-586`).
* Each image is reoriented per use: `IsSplitToReverse(aFImRef, aF, myContext)` → `aFIm.Reverse()`
  (`:594-605`). Note this is the **no-warn** variant here, unlike everywhere else in the Builder.
* `BOPTools_AlgoTools::IsSplitToReverse(Face,Face)` (`BOPTools/BOPTools_AlgoTools.cxx:1324-1436`) has a
  surface-identity fast path: `if (BRep_Tool::Surface(FSp) == BRep_Tool::Surface(FSr)) return
  FSp.Orientation() != FSr.Orientation();` (`:1336-1341`). For ordinary splits this makes the call a
  pure orientation copy — which is exactly what `Builder_3.cxx:334` does directly. For an SD rep coming
  from the *other* operand the surfaces differ and the full path runs: `PointInFace` (hatcher), fallback
  `PointNearEdge` on a non-degenerate non-seam edge, `GetNormalToSurface`, reverse per own orientation,
  project onto the source face, normal there, reverse per its orientation, `return (aDNFSp*aDNFOr) < 0`
  (`:1343-1435`); error codes 1..4 signal "could not decide" and yield `false` (= do not reverse).
* Both maps (`…Ori` oriented, `…Faces` unoriented) get the *same* reoriented copy; untouched faces are
  added as-is (`:610-611`). `myInParts[solid]` lists are unioned per group into `anINObjects/anINTools`
  (`:616-626`) — faces enter with whatever orientation `ClassifyFaces` stored, which is why the IN maps
  must be IsSame-keyed.

### 2.8 `myInParts` content and the "never re-classified" invariant
`BOPAlgo/BOPAlgo_Builder_3.cxx:210-262`: bound **only** if the solid has IN faces or own INTERNAL faces
(`:244`); a solid with no IN faces **and** no shell images is skipped entirely (`:225-239`), so
`myInParts.Seek()` returning null is normal and means "nothing inside". The list = classified IN faces
followed by the solid's own INTERNAL faces (`:250-260`). Classification itself: one representative face
per connexity block, blocks stopped at the solid's boundary edges, block-wide bbox pre-reject, and
`bIsEmpty` (draft solid with no faces) ⇒ **all candidates IN without any test**
(`BOPAlgo_Tools.cxx:1398-1418, 1443-1518`); tolerance is `Precision::Confusion()` (`:1508`).

### 2.9 `AreFacesSameDomain` — the tolerance formula the spec omits
`BOPTools/BOPTools_AlgoTools.cxx:1139-1205`: point inside F1 via
`BOPTools_AlgoTools3D::PointInFace` (hatcher; any error ⇒ **not** SD, `:1153-1159`); then
`aTolF1 = BRep_Tool::Tolerance(F1)`, `aTolF2 = Tolerance(F2)`, both **raised to the maximum tolerance of
the non-degenerate edges of F1** (`:1173-1196`), and
`aTol = aTolF1 + aTolF2 + max(theFuzz, Precision::Confusion())` (`:1199`), verdict
`myContext->IsValidPointForFace(aP1, F2, aTol)` (`:1202`). The check is **one-directional** (F1's point
against F2) and the fuzzy value is `myFuzzyValue` (`Builder_2.cxx:790`).

### 2.10 `myOrigins` for faces exists only when there were FF interferences
Face origins are written at exactly one place, `Builder_2.cxx:914-919`, inside `FillSameDomainFaces`,
which returns at `:586-589` when `myDS->InterfFF()` is empty. Edges/vertices get origins in
`Builder_1.cxx:60-65,107-112`, solids in `Builder_3.cxx:604-609`. A port that mirrors OCCT history must
reproduce this asymmetry or, better, fill face origins unconditionally in the split stage.

### 2.11 BuildRC / BuildSolid details the spec compressed
* `bCheckEdges` is set whenever **any** building element is an EDGE (`BOP.cxx:671-674`), not only a
  degenerate one; degenerate edges are then skipped from the image maps (`:675-678`) and re-added later
  only if their vertex is in the result, is not `IsNewShape`, and has no interference (`:848-861`).
* The COMMON expansion runs over `aMIt` (`:724-738`); the **check** map is expanded for *all*
  operations (`:744-755`). Solid membership falls back to `BOPTools_Set` face-signature only when the
  direct `Contains` failed (`:762-768`).
* `BuildSolid` first maps FACE→SOLID over the **arguments** to find shared faces, then **clears and
  reuses** `aMFS` for the RC solids (`:1124,1156`). Faces with exactly one owning solid form the SFS
  (`:1218-1227`); `SetAvoidInternalShapes(true)` is set here (`:1237`) but **not** in BuildBOP.

---

## 3. PORTING TRAPS

1. **One identity per map is wrong.** The SD table is decided by running the *same* face through an
   IsSame fence (`aMFence`) and an IsEqual fence (`aMFenceOri`) in that order
   (`BOPAlgo_Builder.cxx:682` then `:696`/`:714`). Collapse them and every ON row degenerates.
2. **The one-group-duplicate branch does not `continue`** (`:684-692`, E2). A port that returns early
   there loses the COMMON two-sided-wall case, where each sighting must add its own orientation at
   `:732`.
3. **`aMResFacesFence` decides which copy of an ON face survives** (`:700`, E3). Emit the SD copy
   unconditionally and CUT/CUT21 come out with mirrored wall normals; the CUT21 wall in particular is
   emitted by the *tools* sighting, not the objects one.
4. **`isIN` outranks the CUT reversal** (`:721-729`). A tool face that is IN the objects *and* IN
   another tool solid is added F **and** R un-reversed; only the `else if` branch reverses. Implementing
   "CUT ⇒ reverse all tool faces" produces inside-out cavities for multi-body tools.
5. **`aMFToAvoid` is unoriented** (`:646`, filter `:745`) — marking one orientation kills both. Marking
   only the oriented copy leaves an orphan wall that BuilderSolid will happily close into a ghost solid.
6. **Reversal test is SD-only in BuildDraftSolid but unconditional in BuildBOP** (`Builder_3.cxx:311-334`
   vs `BOPAlgo_Builder.cxx:594`). They agree *only* because of the surface-identity fast path in
   `IsSplitToReverse(Face,Face)` (`BOPTools_AlgoTools.cxx:1336-1341`). If your split faces do not share
   the parent's surface object (e.g. you rebuild geometry per split), the fast path never fires, the
   hatcher runs on every split, and `PointInFace` failures return `false` = "do not reverse" — silently
   inheriting wrong orientations. Keep the surface handle shared, or special-case parent-child pairs.
7. **`myShapesSD[rep] = rep`** (`Builder_2.cxx:876-881`). The representative is bound to itself and that
   binding is a *branch selector* downstream (`Builder_3.cxx:311`). Do not "optimize" it away.
8. **`myImages.Bound()` replaces.** `Builder_2.cxx:858` uses `Bound` (not `ChangeSeek`+append) to give
   originals a self-image; it is safe only because DS-resolvable block members provably have no splits.
   A port using an unconditional bind on a shape that already has images destroys them.
9. **Empty image list ≠ missing image list.** `Builder_1.cxx:95` binds an empty list for edges with no
   pave blocks (intentional deletion of micro edges), whereas an absent binding means "untouched, use
   the source". Three consumers branch on `Seek() == nullptr` (`Builder_1.cxx:145`, `Builder_2.cxx:720`,
   `Builder_3.cxx:131`); conflating the two states either resurrects micro edges or drops good faces.
10. **`aMObjFacesOri`/`aMToolFacesOri` use *composed* orientations** (TopExp_Explorer, `:575-582`) while
    `BuildDraftSolid` iterates shells with a plain `TopoDS_Iterator` and uses the **raw** stored
    orientation (`Builder_3.cxx:297-301`). For a REVERSED shell inside a solid the two conventions
    disagree; pick one convention and apply it everywhere, or reproduce both faithfully.
11. **BuilderSolid failure inside BuildBOP is reported as success.** `BOPAlgo_Builder.cxx:790-793`
    returns without adding any alert to the alternative report, and `BOP.cxx:890` treats an alert-free
    report as success and returns — leaving `myShape` as the *general-fuse* compound. Genuine OCCT
    defect; do not port. Add an explicit failure alert (or a return code) before falling back to BuildRC.
12. **Off-by-one in BuildSolid's shared-face scan:** `for (i = 1; i < aNb; ++i)` over `aMFS.Extent()`
    (`BOP.cxx:1139-1140`) never visits the last map entry, so one face can fail to mark its solids as
    "touched". Do not replicate; use `i <= aNb`.
13. **Solid-acceptance test after BuildBOP's BuilderSolid is orientation-sensitive**
    (`aMObjFacesOri.Contains(aF)` / `aMToolFacesOri.Contains(aF)`, `:774-782`). If your shell orienter
    flips faces, legitimate solids get discarded. OCCT survives because `OrientFacesOnShell` keeps at
    least one harvested orientation per solid — verify that property or use an IsSame test.
14. **Unused faces become solids unconditionally** (`:796-833`): connexity block → shell →
    `OrientFacesOnShell` → solid, with **no closedness check**. Porting this verbatim can emit open
    "solids"; gate on `BRep_Tool::IsClosed(shell)` unless you replicate OCCT bug-for-bug.
15. **`FillInternals` runs only when `!bAvoidIN`** (`:835-871`), i.e. never for FUSE — internal
    vertices/edges/shells of the operands are dropped by fuse and kept by CUT/COMMON.
16. **Alone vertices are injected into the SD representative face** — `FillInternalVertices`
    (`Builder_2.cxx:929-1008`) runs *after* SD substitution and does `BRep_Builder().Add(aF, aV)` on the
    shared face's TShape (`:1005`), mutating a face that both operands now reference. If your
    representative faces are shared immutable objects, this is an aliasing hazard.
17. **BOPTools_Set is not a set-of-unique-shapes.** Seam edges appear twice (explorer), INTERNAL
    sub-shapes are pushed twice deliberately (`BOPTools_Set.cxx:153-165`), equality is
    `count-equal + one-way IsSame inclusion` (`:96-119`), and the hash is a modular **sum** (`myUpper =
    432123`, `:34`) — order-independent but collision-prone by construction. Reimplement as
    (multiset-cardinality, sorted-id-vector) rather than a hash sum if you want reliability.
18. **Determinism has exactly one anchor:** `std::sort(aFIVec)` on DS indices (`Builder_2.cxx:687`).
    Everything downstream — pair enumeration order, `aDMSLS` insertion order, `MakeBlocks` seeds, and
    hence the "first face" fallback representative (`:872`) — inherits from it. If your face ordering is
    hash-map order, SD representatives (and therefore result face identity and orientation) become
    run-dependent.
