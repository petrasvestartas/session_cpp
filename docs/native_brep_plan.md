# Native session BRep translation

## Contract

Priority update — 2026-11-01: solid BRep Booleans are the highest priority.
Do not start or resume offset implementation until native C++ Boolean acceptance
is complete. Preserve existing offset work and evidence. Continue intersections,
pcurves, classification, topology and tessellation only as needed to qualify
union, intersection, both differences and XOR, including the original freeform
chairs. Complete their public API, topology, full-pcurve, volume/occupancy,
Viewer-evidence and clean Windows/Linux/macOS CI gates before offsets.


Implement the algorithms in `session_cpp` using session NURBS and indexed BRep
topology. Production operations must not convert to OCCT, call its libraries,
invoke OCP, or use mesh Booleans. OCCT is a separately executed reference only.
Retain the original surfaces and represent intersection boundaries by shared
3D curves with pcurves on both incident faces. Preserve attribution and license
notices for any source actually translated from OCCT.

User-required failure workflow (2026-10-26): for every implementation error or
poor result in these features, retain a reproducible failing fixture and log;
deeply inspect the corresponding OCCT implementation, its callers and required
invariants; record exact source references and the native divergence; only then
make the native correction and rerun that regression and relevant acceptance
tests. Do not replace this analysis with tolerance relaxation or repeated trial
patches. Parallel agents may examine the same failure from numerical and
topological angles, with explicit file ownership.

Detailed source audits:

- [Interference and reconstruction](native_occt_interference_audit.md)
- [Boolean selection and offsets](native_occt_boolean_selection_audit.md)
- [Surface intersections](native_occt_ssi_audit.md)
- [Curve intersections and assembly](native_occt_curve_audit.md)
- [Volume integration](native_occt_volume_audit.md)

The previous OCCT-backed results establish reference data, not native completion.
Keep that evidence separate. Do not silently fall back to OCCT on native failure.
Do not count skipped, unsupported, or inconclusive cases as passes.

## Checkpoint — 2026-11-12, Linux, branch `claude/clever-ramanujan-dkse3b`

Main at `3326d89b` passes `brep_boolean_test` 58/58 and `brep_test` 72/72 on Linux, the seven
Rhino families and all five freeform chair operations included. The 6 November range-foundation
packets (`codex/native-brep-recovery-20261112`) are not applied: main passes the tests they
were written for.

`examples/native_boolean_fuzz.cpp` (option `SESSION_BUILD_BOOLEAN_FUZZ`, sweep script beside
it) runs random pairs of the seven primitives through all five operations and checks validity,
volume identities and point membership. It found six failure classes the fixtures miss; each is
fixed with a test that was red before:

- collapsed triangle faces of a pyramid: exact rational pcurve, sections clipped to the wedge;
- section ends meeting on an edge of the other solid: joined whatever the pcurve weights;
- a marched seam piece between a knot line and the seam: period from its extent;
- world curve and pcurve knots a rounding apart: an exact trim names the cut on all three;
- a plane-section loop seeded on a seam: cut at its start;
- a plane-section end cut on a seam: polished along the seam only.

After them: `brep_boolean_test` 62/62, `brep_test` 72/72, `intersection_test` 115/115,
`brep_section_test` 25/25, `surface_section_test` 23/23, thicken 15/15, hollow 11/11,
offset 14/14, tessellation 7/7. Sweep of 49 ordered pairs, seeds 1 and 2: generic poses 96/98,
axis-parallel poses 98/98.

Open: torus against cone in one pose (marched trace starting a hair off the seam), sphere
against drilled block in one pose and one operation (paired fit budget), an axis-parallel plane
through a cone whose fitted hyperbola pcurve fails the self-intersection check, a cone against
a drilled block (tangent fan), and the exact volume budget on some torus results.

## Groundwork first — 2026-11-01

The user has paused individual Rhino-case repairs until the shared Boolean
foundations are established. Existing fixtures remain acceptance tests; passing
individual cases does not establish completeness of the algorithm.

Implement and qualify in this dependency order:

1. One oriented face-boundary view shared by classification and splitting:
   actual pcurve uses, raw intervals, seam aliases, singular edges and original
   endpoint identities. Structural validity, UV chart closure and whole-geometry
   agreement are separate obligations.
2. Complete intersection records: returned-curve agreement is distinct from
   discovery coverage. Preserve isolated contacts, overlaps and unresolved
   domains. An empty curve vector alone cannot establish no interference.
   Sampled analytic recognition cannot establish whole-support eligibility.
3. Operation-wide event ownership before either face partition. Reuse section
   indices and `(operand, original_edge)` ancestry; retain clipping events and
   propagate ordered split parameters to every incident face. Keep standalone
   assembly usable and retain its geometric and manifold checks.
4. Shared four-state classification of actual pcurve boundaries, deterministic
   half-edge ownership, and nonboundary witnesses for region containment.
   Unknown states and unowned cycles must remain explicit failures.
5. Material classification and assembly: complete ordered ray events, composed
   orientation, unique retained-face ownership, balanced edge uses, regular
   vertex links, and correctly nested cavity shells.
6. Public qualification: all five operations on independent synthetic geometry,
   verified Rhino references and the freeform chairs; original-geometry checks,
   independent material/volume evidence, actual Session Viewer captures, then
   clean Windows/Linux/macOS CI. Offsets and language ports remain deferred.

Each foundation needs independent positive, negative and ordering/reversal
tests before resuming individual acceptance-case repairs. OCCT's local source
at revision `d3056ef80c9668f395da40f5fd7be186cae4501f` supplies the algorithmic
comparison. In particular, `BOPAlgo_PaveFiller::PerformInternal` resolves
vertex/edge/face interferences and split blocks before face construction;
Session currently discovers some shared-edge subdivisions during assembly.

Current qualification (2026-11-01, native C++ only):

- Shared oriented boundary extraction and actual-pcurve classification are built.
  Classification passes 18/18 and parameter contracts 3/3.
- Shared Section/Boolean inventory v5 is built. It retains isolated contacts,
  original boundary incidences, degenerate UV uses and explicit coverage status.
  Computational success does not establish complete discovery. General support
  coverage and operation-wide singular UV splitting remain incomplete.
- Original-edge event splitting and its eight regressions are built. The original
  circle cutter now passes with certified event ownership. Splitting is 36/37:
  the expanded shifted/reversed/rational circle fixture exposes closed-edge seam
  aliases at the same original vertex. An explicit endpoint-identity repair is
  under review; no proximity-based alias merging is accepted.
- The meridian pole-endpoint repair is built and its original BRep regression
  passes. Expanded shifted-domain qualification exposes another raw-range issue.
  Section is 19/21; general Intersection is 112/113 on this checkpoint.
- A new ownership fixture incorrectly assumed separated parallel supports had a
  positive section. Exact geometry review corrected the premise explicitly:
  preserve the original as a point-only regression and add an actually intersecting
  separate branch for the all-incidence distance test. Both remain strict and red
  pending canonical world-event construction. Original failed inputs are retained.
- The full preceding nine-group run completed: 262 passed, 16 failed. Assembly
  passed 18/18; native Booleans 20/27; public Booleans 4/10. Those Boolean counts
  have not been rerun on later repairs. Rhino boxes/pyramids regressions and curved
  Rhino failures remain open; no new general Boolean or chair qualification claim.

Implemented after that run but not yet built: paired original support trim-event
provenance (including both raw parameters and exact owning occurrences), and
certified reuse of every selected original pcurve on a corresponding target
support. Mapping is only a proposal; independent whole source/target agreement
and finite target-domain containment remain mandatory.

Next execution order:

1. Review and qualify closed-edge endpoint aliases, exact raw trimming/knot
   insertion, shifted meridian construction and original pcurve reuse against
   captured failures. Preserve real tiny positive geometry and original inputs.
2. Complete canonical world-event construction from paired original trim records.
   A tolerance ball alone cannot identify two crossings: prove a shared original
   event, retain all raw incidences, and certify the whole original interval before
   representing it as one contact. Do not infer emptiness from an empty curve list.
3. Complete discovery coverage and singular UV ownership, then region witnesses,
   material classification and shell reconstruction. Rerun the full foundation
   suite before individual Rhino/chair acceptance repairs.
4. Qualify all five operations against Rhino and freeform chairs; save actual
   Session Viewer images and run clean Windows/Linux/macOS CI before publication.
   Offsets and Rust/Python remain deferred.

Two native builds and the full/targeted test runs completed under the unchanged
8 GiB generated / 20 GiB total budgets. Builds and CPU tests are coordinated with
the separate Wood/Gemini and Viewer work. Existing screenshots are older evidence;
no new result images or green CI are claimed for these groundwork changes.

## Earlier execution history — 2026-11-01

Full native C++ acceptance is not complete. OCCT is an external reference only;
production must remain independent of OCCT. Rust and Python remain gated on
qualified C++ behavior. Counts below belong to their recorded builds and must
not be combined into a claim that one final build passes everything.

### Verified status

- New build `wip-build-boolean-critical-events-20261101-v1` passes full
  **Intersection 109/109** and **BRep Public Boolean 3/3**. The opposite-saddle
  case now passes all five operations with strict material-lobe, topology,
  paired-curve and volume checks. Two new strict bilinear section qualification
  tests still return unresolved; their negative-control test passes. This build
  is not yet qualified. Native Boolean passes **27/27** on this same build.
- Preceding full Intersection run: **102/108 pass, 6 fail** on
  `wip-build-boolean-route-observers-20261101-v1`. The local seam-query extension
  fixes three endspan regressions and its new regression passes. Six actual
  failure snapshots remain: two constructor ranges, three near-cap seam events
  and one wrapped range. This build is not qualified.
  Independently reviewed event-owned seam regions, a finite wrapped-range guard
  and the existing local quintic fallback are now applied, with all original
  failing tests retained and one new tiny-span regression. They are unbuilt;
  their source receipt is `boolean-event-regions-root-promotion-20261101-v1`.
- On the preceding `wip-build-boolean-exact-events-20261101-v1` build, NURBS
  curves pass **42/42**, curve intersections **27/27**, curve/surface
  intersections **33/33**, and native Booleans **27/27**. Public Boolean
  tests pass **2/3**: the new opposite-saddle case fails complete SSI branch
  coverage at the first union's bottom-face pair. The exact depth-48 failure
  is retained under `opposite-saddle-critical-failure-20261101-v1`; increasing
  subdivision limits or accepting an unproved branch is not a correction.
  Assembly **18/18** and Measure **17/17** pass on that same preceding build,
  including signed and component XOR volumes. The strict four-variant sphere
  XOR assembly regression now passes after material-shell reconstruction. The
  assembly suite includes chair reconstruction. Full public Boolean
  qualification remains open; these groups have not been rerun on the newest
  diagnostic build.
  The reviewed exact bilinear critical-branch implementation is now applied,
  unbuilt, with three strict section tests. It proves both branches only for
  its exact common-projection/factorization contract; general freeform critical
  intersections remain on the existing solver. The opposite-saddle five-operation
  test remains mandatory. Receipt: `boolean-critical-branches-root-promotion-20261101-v1`.
- Published revision `f1f1c57` fails **8 tests on each of Windows, Linux and macOS**
  (1348/1356). `ef253d` is the last verified all-platform green revision.
- All five original Rhino chair operations pass public construction and
  independent saved-result validation on one `boolean-polish-plane` build.
  All five actual Viewer PNGs are saved, but visible faceting prevents final
  visual acceptance. Stronger input/protobuf checks and the final kernel
  composition still need their own chair matrix.
  A finer saved-union display export was interrupted after retaining a measured
  rejection in a 987,522-triangle proposal. No new accepted mesh/image was produced.
  Diagnose the failing triangle and local refinement before repeating global
  remeshing; the original Boolean evidence is unchanged.

### Remaining work, in execution order

1. **Repair Boolean prerequisite intersections.** Resolve the six captured seam/range failures and critical tangent-branch coverage;
   qualify exact cone-cap wrapping, near-tangent cap partitioning, short chart
   pieces, closed-chart endpoint ownership and endpoint derivative fitting.
   Use the corresponding OCCT source to explain each divergence. Run focused
   regressions and then the complete intersection and curve/surface suites.
2. **Finish solid Boolean reconstruction.** Qualify the reviewed one-pass XOR
   shell reconstruction while preserving effective face orientation and all
   geometry pools. Require all four orientation/order variants, unique topology
   ownership, solid-wide edge/vertex-link checks, component volumes and occupancy.
   Extend union, common, both cuts and XOR across curved/freeform, tangent,
   coincident, nested, disconnected, seam/pole and small-feature configurations.
3. **Complete Section and curve/NURBS intersection events.** Preserve isolated
   contact points and genuine short curves, paired UV curves, raw parameters and
   boundary ownership. Resolve corner contacts and degeneracies through public
   APIs; internal-module tests alone do not establish integration.
4. **Qualify the original chairs on one final C++ build.** Run public union,
   intersection, A-minus-B, B-minus-A and XOR on `solidbrep.stp`; require topology,
   complete pcurve certificates, component volumes, independent occupancy probes,
   serialization and input nonmutation. Keep external OCCT comparisons separate.
5. **Finish Boolean tessellation and Viewer evidence.** Resolve remaining tessellation
   gates, render actual Boolean and prerequisite intersection results in Session Viewer,
   inspect the saved images and bind each to its exact scene and tested build.
   Publish the image folder; coarse previews are not final-quality acceptance.
6. **Qualify and publish C++ Booleans.** Independently build the clean publish composition;
   run the complete relevant tests and Windows/Linux/macOS GitHub CI. Verify the
   exact pushed revision is green and production contains no OCCT dependency.
   Preserve the shared index and other agents' work; do not merge main.
7. **Only after all C++ Boolean gates pass: offsets and hollowing.** Qualify clean native public integration,
   then general freeform joins, support continuation, seam/pole behavior,
   collapse and self-intersection handling. Cover curve/wire/surface offsets,
   solid offsets and thick/hollow solids with positive and negative distances.
8. **Translate qualified behavior to Rust, then Python.** Reuse the same fixtures
   and acceptance matrix, verify public API parity and reproduce Viewer evidence
   where applicable. Do not begin by copying unqualified C++ behavior.

### Immediate next batch

The original-chair public API matrix passed all five operations on the frozen
`wip-build-boolean-polish-plane-20261101-v1` binary: intersection, union,
A-minus-B, B-minus-A and XOR. Reports are in
`C:/pc/tmp/session-migration-20261028/public-chair-matrix-20261101-v1`.
Independent saved-result validation also passed all five against the frozen
oracle, including component volumes and 80 classifications per operation;
reports are in `public-chair-independent-validation-20261101-v1` under the
same evidence root. Capture actual Session
Viewer evidence is saved as previews; final visual acceptance remains open.
The exact-knot trim and seam corrections have now been built and tested, with
the failures above retained. The local linear seam-query extension and
failure observations are built; the remaining six failures now have captured
raw event, constructor and wrapped-run data. The stronger operand/protobuf
chair harness is compiled but its matrix has not yet been rerun. Qualify the
repairs and stronger chair matrix before clean publication. Offset
implementation remains deferred.

Additional Boolean coverage now implemented: oppositely curved saddle
solids with identical footprints and crossing intersection branches at an
interior tangent point. Analytic union/intersection/difference/XOR volumes and
regular material-component counts provide an independent oracle. This case is
a retained runtime failure and must not be replaced by the transverse saddle tests.

Sol agents prepare and independently review isolated drafts; root coordinates,
reviews and promotes shared-source changes. One guarded heavy job at a time,
four compiler workers, 6 GiB process guard, 20 GiB workspace / 8 GiB generated
limit and 1 GiB launch reserve. No arbitrary wall timeout, tolerance relaxation,
mesh Boolean substitution or OCCT fallback.

Evidence root: `C:/pc/tmp/session-migration-20261028`.
Existing actual Viewer previews:
- `public-chair-viewer-20261101-v1/index.html`: all five public native chair
  results, each independently validated. All five PNGs were inspected and the
  folder opened. Visible faceting and renderer `source admission` limitation
  prevent final visual acceptance; `visual-review.json` records this explicitly.
- `chair-results-all-five-20261030/index.html`
- `concave-hollow-viewer-20261030/opening-results.html`

These remain previews; final-quality acceptance is pending. The compact checkpoint
is `resume-checkpoint-20261030-native-progress.json`; shared-job authority is
`docs/coordination/storage-and-floor-2026-10-28.md` in the parent repository.
No overall completion percentage is justified while geometric coverage remains
open. Earlier records below are historical evidence, not current acceptance.

## Sequence and gates

Current checkpoint, 2026-10-29: native Windows build `wip-build-20261029-1043`
passes and NURBS curves pass 37/37. The new unclamped-trim regression first
failed on build1038, then passed after correcting retained endpoint clamping.
An exact raw SSI replay proves that trimming, rather than the raw section,
introduced the previous affine-UV Boolean backtracking: the first world
endpoint moved by 0.021811. The new Boolean run passed its first 26 cases,
then was deliberately interrupted in the additional triple-event case after
ten sampled stacks proved expensive interval checks on unrelated source edges.
The run is incomplete. A reviewed source-identity short-circuit and phase
observer are promoted but unbuilt; no speedup or final-case pass is claimed.

The lower-endpoint span repair now passes the exact trimmed-support
same-parameter replay and its previously failing thickening case. The current
curve/surface group passes 25/26: the remaining negative test rejects bad
geometry as Unresolved, while our assertion incorrectly requires Mismatched.
A reviewed test correction independently checks an over-tolerance gap and
requires rejection; it remains unpromoted. No negative geometry was accepted.

Hollowing now passes 4/7 after context-boundary ancestry and affine support
extension repairs. Curved retained walls still fail curve-intersection
subdivision. Constant-weight rational mouth pcurves and an optional extension
of a concave opening also fail; reviewed repairs remain uncompiled. The
preceding full offset/thickening runs were 15/15 and 14/15 respectively;
the newly fixed thickening case has passed individually, not in a new full run.

The native tessellation group is incomplete. Its first case was interrupted
after discovering an invalid signed-volume assertion elsewhere in the group;
that assertion is corrected and phase instrumentation is ready for the next run.
No tessellation acceptance or performance improvement is claimed. Historical
build0810 passed NURBS curves36/36, volume11/11, assembly13/13 and Booleans26/26;
the new Boolean regression is additional and has not passed. The volume and
assembly groups include both supplied Rhino chairs.

Six actual Session Viewer previews are saved in
`build/native-geometry-tests/viewer/index.html`, with scene, report, renderer and
PNG hashes in `viewer_captures.json`. Some complex surfaces use coarse display
fallback; these images are not a numerical acceptance gate. Six intersection
exports and actual captures also pass in
`build/native-geometry-tests/intersections-20261029-0817/index.html`. Subsequent
Boolean and thickening reruns changed all six original scene files: their earlier
PNGs remain previews awaiting recapture. New captures retain immutable scene/report bytes.
All twelve PNGs are retained; redundant raw PPM files were retired only after
verifying identical decoded pixels. Ten fresh transparent Session Viewer
captures are saved externally at
`C:/pc/tmp/session-migration-20261028/viewer-transparent-20261029-1053`.
All scene/report bytes are immutable and bound to renderer and PNG hashes.
Visual inspection approves seven as previews; three small-scale intersection
images have oversized strokes. Three width-only recaptures at
`viewer-default-pens-20261029-1113-v2/gallery` also failed visual QA; the Viewer
owner has the source diagnosis: planar-file automatic sheet detection converts
screen pens to 1 mm world ink. An explicit display-mode correction is pending. No capture
is a substitute for topology, parameter and numerical acceptance.
Saved original chair common and union both pass independent revalidation at
`chair-revalidation-20261029-0838`: all 80 probes, component volumes, vertex
links, full pcurves and serialization round trips. These validate saved native
results; the two differences and XOR still require construction and complete
acceptance. Draft PR20 (`7a0d2ad`) is published; remote Linux passed 1320/1324
and macOS ARM64 and Windows each passed 1318/1324. Windows adds rotated
Platonic Boolean and dense Rhino chair face-cut failures, bringing the distinct
failures to eight. Commit e1b8201 repairs five: Linux and macOS each pass
1324/1325, and Windows passes 1322/1325. Exact surface-split witnesses show
nearby boundary nodes incorrectly merged across a crease's exact incidence.
Commit4725d85 preserves that incidence and checks constraint insertion. Its
1326-test run passes1324 on Linux,1325 on macOS and1322 on Windows: the strict
checks expose a further insertion defect in the saved exact-coordinate replay.
Two fixtures retain the actual Linux/macOS UV loops. Exact arithmetic proves
that a rounded orientation selected a triangle outside the requested point.
A reviewed filtered/exact predicate and direct coordinate-range regression
are now building locally; no runtime pass is claimed yet.
Commitd64fbd4 captures the failed Windows dodecahedron topology and dense-chair
cycle decisions. These diagnostics preserve all acceptance conditions.
Three Sol workers are repairing causes without weakening acceptance.
Published CI is not yet green. The expanded unqualified CI candidate retains all 1,278 original default
tests plus nine gated bodies, with 1,326 default tests in total. Rust/Python
translation remains gated on C++ completion. Older counts
below are historical checkpoints, not the current full-suite result.

| Stage | Implementation | Acceptance gate | Status |
| --- | --- | --- | --- |
| 0 | Inventory native primitives; pin reference revision; create OCCT-free build and dedicated native regression target | Compile and execute with `SESSION_WITH_OCCT=OFF`; retain reference reports separately | Complete locally |
| 1 | Geometric tolerance policy; conservative NURBS bounds; trim classification; solid IN/OUT/ON/UNKNOWN classification | Rational trims, holes, seams, poles, cavities, far translations, small scales; ambiguous cases never become OUT | Certified ray roots integrated;12 ray and11 classifier groups pass; ambiguous boundary/singular cases remain explicit |
| 2 | Vertex/vertex, vertex/edge, edge/edge, vertex/face, edge/face interference; sorted edge split parameters and common intervals | Tangencies, overlaps, multiple roots, tiny spans, endpoint contacts, positive rational weights; residual checks | Native CC/CS solvers integrated; complete persistent pre-FF graph unfinished |
| 3 | Native surface/surface intersections with paired pcurves, adaptive patch bounds and seeds, continuation, closed loops, tangent/coincident regions | Doubly curved pairs, internal loops, branch points, seam crossing; sampled and adaptive world/pcurve residual bounds | Intersection88/88 and Surface Section19 plus one new focused case pass in recorded checkpoints. Six current intersection exports pass, including rational tangency/overlap and sphere-cylinder raw seams. Singular/coincident completeness remains unfinished |
| 4 | Shared interference graph and edge splitting; trim intersections against both faces; parameter-space arrangement and face reconstruction | Holes, disconnected regions, coincident boundaries, periodic domains; adjacent faces share the same split edge | Historical Split19/19, Split Direction8/8 and BRep Section6/6 pass; remote Windows dense-chair split remains failing. Corrected affine section expectation now passes all three platforms. Consecutive retained source-curve intervals form one section; rejected hole intervals remain separate |
| 5 | Classify split regions; Boolean truth tables and orientation; sew shells; nest cavities; assemble disconnected solids | Union, common, A-B, B-A, XOR; closed oriented manifold output and no unused geometry; no mesh reconstruction | Historical26/26 pass at0810, including25 rotated Platonic operations and curved cavity reuse. Actual retained-endpoint trim defect corrected in1043; full Boolean run pending. Saved original chair common and union pass independent80-probe acceptance; both differences and XOR remain unaccepted |
| 6 | Native public APIs and acceptance corpus, with OCCT only in separate oracle programs | All chair operations and configured hard cases pass in OCCT-free executable; compare geometry/topology/volume, not volume alone | Planned |
| 7 | Native curve/surface offsets, approximation error control, BRep joins, intersections, trimming, self-intersection removal, collapse and hollowing | Signed offsets, topology changes, singularities, all Platonic solids, freeform chairs; failed outward chair cases remain visible until resolved | Historical NURBS offsets15/15 and thickening14/15; trimmed-support certificate and thickening case now pass individually on1043. Hollowing4/7; curved walls, rational mouth PCs and concave optional extension remain failing. Four mixed freeform variants, global cleanup and general hollowing remain unfinished |
| 8 | Review, reproducibility, complete C++ suite, saved native images and reports | Build without OCCT on supported platforms; render actual output, preserve failed cases, open evidence folder | All three platform builds pass at7a0d2ad; full tests fail. Current draft rerun and native Boolean/offset gates unfinished |
| 9 | Translate completed C++ algorithms into Rust and Python with the same API and tests | Same fixtures, classifications, tolerances and outputs; no OCCT/OCP runtime dependency | Waiting for C++ acceptance in stages0-8 |

Stages are dependency gates, not estimates or promises that the entire OCCT
kernel can be translated in one patch. Work through them in order, recording
actual completion and failures here. Keep small testable functions and reuse
correct native primitives instead of duplicating OCCT's framework.

## OCCT algorithm mapping

Reference checkout: `C:/pc/occt8`, revision
`d3056ef80c9668f395da40f5fd7be186cae4501f`.

| Reference responsibilities | Session destination |
| --- | --- |
| `BOPDS`, `BOPAlgo_PaveFiller` VV/VE/EE/VF/EF/FF and pave/common blocks | Native interference records, source IDs, split parameter lists and shared section edges |
| `IntTools_EdgeEdge`, `IntTools_EdgeFace`, `IntTools_FaceFace`, `IntPatch`, `IntWalk`, `IntSurf` | Native subdivision, root refinement, continuation, isolated contacts, overlap regions and paired pcurves |
| `BRepClass`, `BRepClass3d`, `IntCurvesFace` | Trim-aware ray intersections and explicit point/solid classification |
| `BOPAlgo_BuilderFace`, `BOPAlgo_WireSplitter` | Extend native `split.cpp` arrangement to consume existing paired pcurves and interference identity |
| `BOPAlgo_BOP`, `BOPAlgo_BuilderSolid`, `BOPTools_AlgoTools` | Boolean region selection, orientation, sewing and shell nesting |
| `BRepCheck`, `BRepLib` | Native geometric and topological postconditions with bounded tolerances |
| `Geom_OffsetCurve`, `Geom_OffsetSurface`, `BRepOffset`, `BRepOffsetAPI` | Native evaluated offsets, controlled NURBS approximation and topology reconstruction |

Sources: [Boolean specification](https://occt3d.com/dev/doc/overview/html/specification__boolean_operations.html),
[source tree](https://github.com/Open-Cascade-SAS/OCCT/tree/d3056ef80c9668f395da40f5fd7be186cae4501f/src).
This mapping is an implementation dependency map, not a claim that these source
files have already been translated.

## Native acceptance matrix

- Boxes: the 38 recorded configurations, all five operations; empty, identical,
  disjoint, containment, coplanar overlap and face/edge/vertex contact.
- All five Platonic solids: rotated overlap, containment, nearly tangent faces.
- Rational geometry: spheres, cones, cylinders, tori, thin rings and bores;
  tangency, poles, repeated seams, near-coincidence and tiny intersection loops.
- Freeform: saddle pairs, high curvature patches, multiple loops, trimmed holes,
  boundary tangency, narrow slivers, and the two supplied Rhino chair solids.
- Topology: disconnected outputs, cavities and nested shells, reused geometry,
  reversed parents, degenerate edges, shared-edge split propagation.
- Numerics: uniform scaling, translation far from the origin, non-unit domains,
  repeated knots, varying rational weights and tolerances. Check metamorphic
  identities and input immutability in addition to independent references.
- For every native result: validity, closedness, orientation, component count,
  edge/vertex incidence, pcurve residuals, source-surface preservation, volume,
  serialization round trip, actual geometry images and reproducible parameters.

The chair fixture is `tests/fixtures/solidbrep.stp`, SHA-256
`6cb6b9d1723a0570fd72e1eb8ecedf109a750a468f5f8e771a624ed4224aac19`.
Reference volumes and component counts already exist in
`build/geometry-tests/occt_step_reference.json`. Native results must be saved in
a separate `build/native-geometry-tests` directory.

## Next implementation slice

Current source-guided checkpoint:

- `native_interference_minitest`: the earlier source-guided run passed 134/134.
  The expanded run passes 138/140. A new tiny-domain parameter certificate exposed
  a degree-one endpoint bug, fixed but awaiting rerun. Chair reconstruction now
  fails its unchanged mesh-volume comparison after pcurve normalization; exact
  per-face diagnostics are being collected before a mesher correction.
- Public intersections: 88/88 after local-coordinate fitting and whole-interval
  pcurve certificates resolved the periodic sphere/cylinder case at unchanged
  requested tolerance. Two closed loops inside one knot cell are a new pending
  coverage regression; this test count does not establish branch completeness.
- Native solid Booleans: 9/10 groups. Transverse, partial-coplanar, identical,
  disjoint, containment/cavity, empty, face/edge/vertex contact and tangent-sphere
  cases exercise union/common/difference/XOR. Coincident curved support with
  different adjacent partitions now passes. The transverse saddle gets past
  collapsed intervals and now reports an unresolved edge/pcurve certificate.
- Both original chairs reassemble successfully. Direct native NURBS integration
  gives `2944.7043442359222` and `2944.7043442358918`, agreeing with the separate
  OCCT STEP oracle to approximately `2.1e-9` absolute. These are original-shape
  measurements, not Boolean result acceptance.
- Actual Session Viewer captures are supported by `examples/native_viewer_gallery.py`.
  Final capture/export review and the complete hard-case gallery remain pending.

The earlier dependency list below remains the architectural completion gate;
some individual primitives are now implemented, but none of the listed gaps may
be inferred complete from the narrower passing tests.

1. Extract the control-hull bounds and curve-pair subdivision shared by
   `brep_classify.cpp` and `split.cpp`. Keep positive-weight assumptions explicit;
   replace depth-limit acceptance with an unresolved result.
2. Add interference records carrying both source indices and parameter intervals.
   Record isolated roots, tangent contacts, and overlap intervals separately.
   Reuse them in curve sections and edge splitting; do not recover identity later
   by welding nearby points. Port VV/VE/EE first, then VF/EF.
3. Replace sampled surface-seed boxes with conservative patch subdivision and
   explicit unresolved/tangent/coincident results. Add simultaneous approximation
   refinement for world curves and both pcurves, including periodic seam pieces.
4. Extend `split_brep_face_by_curves` to accept the existing paired pcurves and
   shared interference IDs. Its current world-curve pullback loses that identity.
   Split each operand's affected edges once and propagate splits to every use.
5. Only then connect the native classifier to region selection and shell assembly.
   Run the original chair pair in all five operations before exposing success
   through the public Boolean APIs. Retain original surfaces and shared edges.

These are required changes, not completed work. General BRep offsets depend on
the same intersection and reconstruction infrastructure and remain gated on it.

## Initial audit

Reusable: NURBS evaluation and splitting, curve/plane Bernstein roots,
`split.cpp` face arrangements and shared-edge updates, native surface marching,
STEP import, indexed topology and serialization, convex planar miter offsets.

Not established: completeness of native surface intersection seeds; tangent and
coincident handling; a production trim-aware solid classifier; cross-operand
edge identity; general shell assembly and cavities. `surface_surface` now uses
the native implementation even in reference-enabled builds. Existing public solid Booleans,
general offsets, exact volume and controlled tessellation still dispatch to
OCCT. They must not be described as native while that remains true.

## Execution record â€” 2026-10-26

- Added `brep_classify.h/.cpp`: native rational-curve trim winding, control-hull
  ray/surface subdivision, Newton refinement, oriented crossing classification,
  cavity and active-face handling, boundary-distance checks, explicit UNKNOWN
  and diagnostic failures for unresolved subdivision or numeric resolution.
- Added `native_brep_minitest`, available only in an OCCT-disabled build. Its
  source also contains a compile-time rejection of `SESSION_WITH_OCCT`.
- The first run failed seven of eight groups. Investigation found that native
  curve trimming and knot insertion snapped small intervals away using
  `sqrt(epsilon)` scaled by parameter origin. Replaced this with machine-scale
  parameter comparisons and exact lookup of inserted trim knots; added narrow
  intervals and tiny reparameterized domains as regressions.
- Ten native regression groups now pass. They cover rational spheres and poles,
  thin tori, cone apex, cylindrical holes and trims, box boundary tolerance bands,
  cavities, unused face pools, scales 0.001â€“1000, far translations, invalid inputs,
  and both sides of every face in the supplied chairs.
- A separate OCP executable path reads the original STEP and agrees with all
  80 chair classifications (40 faces, two sides each). This is a point
  classification comparison, not a native Boolean result.
- The final OCCT-disabled C++ suite passed 1108/1108 tests. Dedicated executables
  passed 10/10 classification groups and 85/85 intersection groups. These counts
  include existing tests; OCCT-only Boolean/offset tests are excluded from this
  build and are not being counted as native passes.
- Fixed stage 3 dependencies discovered during the audit: world curves and both
  pcurves now use shared interpolation parameters; section samples receive tighter
  Newton correction. A small internal loop previously traced three laps now closes
  on its first return. Removed the OCCT branch from public `surface_surface`.
- Added explicit-parameter NURBS interpolation and replaced the periodic dense
  linear solve with a cyclic tridiagonal solve. Regressions exercise 2048 samples,
  nonuniform parameters, derivative continuity, and affine correspondence.
- The nonzero-domain regression exposed an existing hard-coded zero in
  `nurbsknot::build_interp_nurbsknots`; the initial endpoint now uses the supplied
  first parameter. The failing full-suite log is retained with the native evidence.
- These intersection fixes do not establish completeness. Grid-sampled seed bounds,
  tangent/coincident regions, silent marching limits, and adaptive approximation
  certificates still need replacement before Boolean acceptance.
- Saved sampled paired-curve gaps are below 1e-8 for the doubly curved pair and
  below 1e-7 for the small interior loop. The torus/plane section still has a
  sampled paired-curve gap around 1.4e-5 despite its separate cut-curve test passing.
  This remains an explicit approximation defect for stage 3, not a satisfied
  requested-tolerance guarantee. The gallery preserves the measured value.

Reproduce from the repository root:

```powershell
cmake -S session_cpp -B session_cpp/build -DSESSION_WITH_OCCT=OFF
cmake --build session_cpp/build --config Release --target native_brep_minitest native_intersection_minitest point_minitest --parallel 6
Set-Location session_cpp
./build/Release/native_brep_minitest.exe
./build/Release/native_intersection_minitest.exe
./build/Release/point_minitest.exe
python examples/occt_classification_reference.py
python examples/native_geometry_gallery.py
```

The gallery reads `build/native-suite-test.log`, so capture the full suite's output
there when reproducing it. The optional comparison script requires OCP; neither
native executable uses it. `native_brep_minitest` exports the original chair
geometry, native meshes, raw classification probes and assertions into
`build/native-geometry-tests`. The gallery labels this milestone explicitly;
stages 2â€“9 remain unfinished and native solid Boolean acceptance has not passed.

## Checkpoint 2026-10-26, native hard-case failures

The latest expanded focused run passed 142/146 groups. The four failures were
chair reconstruction mesh invariance, rounded-box offset parameter certification,
cylinder offset tube approximation, and two closed SSI loops inside one knot
cell. Partitioned sphere offsets of both signs and a 42-face doubly curved sharp
join offset passed. Source-derived corrections are being built; these are not
yet acceptance results. The public 88/88 intersection pass predates the new
coverage correction. Native Boolean groups remain 9/10 pending the saddle fix.

The original chair common operation failed at section interpolation. A scan of
all 400 face pairs preserved 9 such failures: (8,17), (9,17), (14,0), (15,18),
(16,2), (16,17), (19,0), (19,14), (19,19). The other 391 pairs satisfy the
existing sampled residual checks; that does not establish branch completeness.
Exact fit samples are being added to the failure diagnostics.

Actual Session Viewer rendering produced 74 captures, 19 empty-result records and
one renderer failure on a 789-span curve. A narrow display clamp correction
and regression are pending verification; this is not a Rust kernel port.

## Checkpoint 2026-10-26, boundary tracing and expanded solid matrix

Focused corrections now pass for analytic arc length, adjacent-patch parameter
certificates, rounded-box offsets, and exact circular cylinder offsets. The
initial Session Viewer gallery has 75 rendered images, 19 empty results and
zero remaining capture failures. These are component results, not full kernel
acceptance.

The expanded matrix executed 190 box operations and 25 rotated Platonic operations.
188 box assertions passed; two XOR shell-count expectations incorrectly counted
a bounded complement region between touching components as a separate cavity
shell. OCCT assigns already-built hole shells to containing solids. The independent
cell oracle is being corrected per material component, retaining the original
failure report. Tetrahedron and octahedron passed all ten operations; cube,
dodecahedron and icosahedron returned unresolved face classifications in all
fifteen operations. Exact operands and reports are preserved for diagnosis.

Independent chair reconstruction analysis proves the entire mesh-volume delta
comes from 22 valid triangle-diagonal replacements with the same oriented patch
boundaries. The test now checks the actual BRep surface integral, unchanged
support surfaces/orientations and watertight meshes; the old failure and mesh
connectivity audit remain saved. The revised test has not yet run.

Chair pair (19,0) stops at a stale predicted boundary before Newton correction
actually reaches the support boundary. A boundary-constrained correction, audited
against OCCT's stopping/caller sequence, is building. Strict SSI graph-coverage
failures on a cell boundary and an original-domain boundary have separate saved
fixtures and an independently reviewed chart/clipping correction building.
Seven new native face-thickening groups are also building. General solid offsets,
all chair Boolean operations, public API replacement and language ports remain
unfinished.


## Checkpoint 2026-10-26: expanded Boolean matrices pass

The native build without OCCT now passes all 190 box operations and all 25
rotated Platonic operations. The saved grazing-ray regression and rotated cube
common operation pass after original-support Newton polishing checks the full
world correction, including displacement along the ray. No tolerance or duplicate
merge distance changed. The corrected box cavity oracle retains the original two
shell-count failures as evidence.

All 400 original Rhino chair face pairs now pass completed intersection processing
and sampled paired-lift checks. The former nine interpolation/marching failures
are resolved. The scan took 240.136 seconds. This is not a proof that every
possible branch is found. The original chair common operation is running; no
successful chair Boolean is claimed yet.

The two-loop and narrow-span SSI regressions pass. Both original chair volumes
agree with the independent OCCT reference. Natural rectangular polynomial face
integration passes the dense preserved rational-thickening result in 28.8 ms,
where earlier generic integration exceeded its budget. This does not yet validate
the complete rational-thickening workflow.

Two hard regressions remain explicitly failing: extreme Mobius reparameterization
lacks a root-graph coverage certificate, and a tiny warped surface offset violates
its error bound. Source analysis identified an interval-derivative conditioning
opportunity for the former and a shared Vector norm snapping bug for the latter.
Their corrections and direct regressions are pending. General inward/mixed
offsets, hollowing, complete singular intersection handling, public native API
replacement, and language ports remain unfinished.


## Checkpoint 2026-10-27: certified rays and original-span splitting

All 25 Vector groups and ten surface-ray groups pass. The differential norm now
preserves small values, and normalization handles subnormal and very large finite
vectors. The ray module certifies isolated roots on original positive-rational
supports and preserves distinct parameter branches. Integrated classification
passes ten of eleven groups, including the Rhino chairs; the remaining bore-origin
trim failure is preserved and being compared with OCCT's caller sequence.

The saved dense chair face now splits in 1.17 seconds. Its original failure counted
the full Cartesian span product before any bounds test. The correction counts only
R-tree candidates bounded by original controls, retaining the same work limit.
The complete chair common operation is running again. No native chair Boolean
result has yet met acceptance.

Extreme Mobius SSI, two closed loops, and narrow-span regressions now pass after
an independently reviewed interval derivative correction. Runtime and seed identity
still require work. All four new adjacent-freeform inward offsets reach arrangement
selection and fail; the exact supports succeed in the independent OCCT oracle.
Full FaceSplit lineage diagnostics are ready before correcting the selection.
Native self-intersection regressions and an open cubic crossing fixture are added
for the previously unexamined interior of one knot span.

General offsets/hollowing, singular intersections, complete chair Boolean
acceptance, production OCCT removal, and Rust/Python ports remain unfinished.


## Checkpoint 2026-10-27: chair partitioning reaches assembly

The preserved trim sampler failure passes, and the saved 31-section replay partitions both original chairs successfully. The complete chair common operation now reaches assembly and fails after 418.760 seconds at the edge/pcurve SameParameter certificate. Its world and pcurve start parameters differ by approximately 4e-16. This is the same caller invariant exposed by small inward saddle offsets; no accepted chair Boolean is claimed. The exact first report is preserved, and the driver now retains typed correspondence failures for future replays.

The ordinary-knot/genuine-event regression passes. All four strict inward saddle offsets still fail: small cases lose shared parameter identity during independent inversion, large cases nest opposite cycles through sampled-area rounding. The exact reverse-cycle fixture now has a graph-edge identity correction and direct replay test. Shared parameter repair is staged pending an unchanged certificate diagnostic.

Self intersections pass six of seven groups; a second compressed-tail correction is building. The splitter still requires integration of the helper, nondegenerate same-vertex edges, whole-interval vertex-ball collapse proofs, and parameter-aware edge reuse. Both a cubic one-span crossing and a quartic with two loops at one vertex have explicit regressions. SSI branch provenance and continuation changes passed independent source review and are building before renewed tests.


## Checkpoint 2026-10-27: strict inward freeform offsets pass

All four six-support saddle inward offsets now pass the unchanged topology gate (six faces, twelve edges, eight vertices), paired boundary and normal residuals, occupancy probes and independent volume checks. Both offset distances and affine surface domains are covered. Immutable first_strict_pass records include geometry hashes. Native volumes are6.968826608086911 and4.662603061947375; the largest OCCT difference is9.78e-7 at unchanged modeling tolerance3e-5. Total test wall time was178.418 seconds. This restricted convex sharp-join path is not acceptance of general mixed or self-interfering offsets.

The rational freeform thickening workflow passes all nine interior/exterior probes in132.120 seconds after immutable ray-span caching, versus390.144 seconds before. All12 ray groups and11 classifier groups pass. Four fresh captures from the actual Session Viewer GPU renderer are saved with scene/renderer hashes in verified_freeform_viewer_captures.json. Final gallery and folder opening remain pending.

The stationary endpoint self-helper regression now passes in0.405ms after a600-second baseline timeout; all eight standalone self groups pass. The C0 one-sided trim direction regression passes. Expanded reversed/affine stationary fixtures expose a distinct interval dependency, and a tangent-sector permutation fails topology. The quartic two-loop trim failure is proven to be a false global-projection rejection: paired residual1.27e-14 and full certificate Matched, versus projected gap0.03675. Full certificate output validation is edited but not yet rebuilt.

SSI junction sharpening succeeds for all179 failed-join candidates in the two-loop fixture, then exposes a residual-limited midpoint at a one-ULP interval. The tighter midpoint correction is source-reviewed and ready to build; strict chart proofs and work limits are unchanged. Original chair Boolean acceptance, mixed/global offsets, hollowing, production OCCT removal and kernel language ports remain unfinished.


## Checkpoint 2026-10-27: preserved continuation and rotated-tangent witnesses

Ten self-intersection groups pass after raw-pole derivative evaluation and exact original-knot endpoint identities. All 24 curve/surface groups, 12 ray groups and 11 classifier groups pass. Full paired edge validation passes the quartic split and three direct parameter-contract tests. The certified tangent-sector construction passes all six existing direction groups, including input permutations; a newly rotated case fails explicitly with unresolved prefix order. Its exact geometry is preserved before further arithmetic changes.

The two-loop SSI still exceeds 600 seconds after tighter midpoint correction. A bounded diagnostic captures a depth-eight graph-inclusion failure despite small contraction and tiny endpoint residuals. Center-residual enclosure dependency is under investigation; the geometric proof and work limits are unchanged. The 30-second observation is not an acceptance result. The original chair common operation is rerunning after shared-parameter lineage repair; no successful native chair Boolean is claimed.

Four corrected-color images have been rendered by the actual Session Viewer. Re-export reports prove unchanged serialized geometry except color, and original white images remain preserved. The mixed L-prism source fixtures all validate, while the native small planar offset fails at its concave edge as expected from the current missing path. All twelve independent OCCT offset shapes validate, but two standard-domain inside probes return OUT despite signed closest-face gaps and three independent crossing lines supporting INSIDE. Expected states and tolerances remain unchanged while the oracle classifier is traced.

## Checkpoint 2026-10-28: native Boolean acceptance and retained failures

The original Rhino chair intersection passes independent OCCT-result classification at all 80 probes, total and per-component volume, connected shell ownership, complete edge/pcurve correspondence, and protobuf round trip. Its result contains 19 faces and two solids. The later C++ vertex-link gate still needs a saved-result replay; a separate combinatorial check found valid links in both components.

Original-chair union constructs 50 faces and one solid in 532.816 seconds. Connected shell ownership, referenced edges/vertices and the vertex-link gate pass. Acceptance then fails the volume work budget; its actual BRep, Session scene and executable hashes are retained in the external chair-union-first archive. Neither union acceptance nor fresh A-B, B-A and XOR acceptance is claimed.

The existing Boolean group passed 20/20, including affine UV identities/partitions. Two additional focused cases pass for bounded common-support agreement across degree elevation and knot insertion, including nonuniform rational representations. The complete Intersection group passes 88/88 after original-domain isolated-boundary seed ownership repair. These results do not establish completeness for arbitrary singular intersections or Boolean inputs.

Polynomial volume primitive caching exposes two exact constant-coordinate evaluation failures: an evaluated trim coordinate drifts one ulp outside its support domain. The exact witnesses are preserved. A reviewed native evaluator repair preserves proven nonrational constant coordinates and zero derivatives without a domain clamp. Tangent-cylinder reversal exposes a distinct seam occurrence-selection bug; small-solid and cavity tests expose finite-probe orientation overshoot. Composed pcurve selection and a certified first-transverse-crossing orientation method are reviewed and promoted, but their current runtime qualification is pending.

New planar and doubly curved cavity-reuse tests exercise an island in a void and exact cavity filling across all five operations. They require signed shell and per-solid volumes, vertex links, full pcurves, analytic occupancy, and removal of internal cavity faces. Their Session scenes and assertions will be captured after execution; source review alone is not a pass.

The isolated CI repair source retains all 1,278 original unconditional tests and contains 1,300 in total. It is materialized but not yet built or published. Existing GitHub CI remains red. Production OCCT removal, remaining general offsets/hollowing and Rust/Python ports remain unfinished. OCCT source and oracle builds stay outside this workspace; native acceptance builds disable OCCT.

Latest Hollow checkpoint (2026-10-30, 01:46): unchanged baseline 4/7; reviewed
v3 support-extension build passes, expanded group 8/10. Rational mouth charts
and exact extension rollback pass. Curved retained walls still fail at a
triple-junction CC budget; shifted rational-chart walls fail while clipping an
adjacent section. Exact captures and hashes are retained in
`C:/pc/tmp/session-migration-20261028/hollow-v3-failures-20261030-0146/manifest.json`.
The first failure has 961 x 684 = 657324 span pairs, while all pairs including
proven disjoint ones currently consume a 200000 narrow-phase budget. A native
outward-bound pruning repair is drafted after studying OCCT's disjoint-box
candidate rejection. Neither curved-wall case is accepted yet. Concave-opening
context half-cylinder implementation and strict physical oracle remain separate,
uncompiled drafts. Linux/macOS latest CI pass 1344 each; Windows tests active.

CI checkpoint (2026-10-30, 01:51): published head 28fa434 passes all 1344 tests
on Windows, Linux and macOS. This is the published registry; the local source
registry contains additional unfinished work. Two new local regressions and a
checked failure-only section-pair capture are now promoted for RED/GREEN
qualification; algorithm fixes have not yet been applied. The reviewed span
pruning and certified negative same-parameter patches preserve existing strict
assertions, tolerances and work budgets. No new offset success is claimed.

Native intersection checkpoint (2026-10-30, 02:00): regression-only build
reproduced CurveIntersection 25/26 and Curve Surface Native 25/27. After the
reviewed native corrections, the full groups pass 26/26 and 27/27 respectively.
The focal-support assertion still requires Mismatched; only certified lower
residual bounds can produce the new early rejection. Dense spans use certified
negative bounding boxes before consuming the unchanged detailed-work budget.
Build `wip-build-20261030-0159-intersection-repairs` and source freeze pass.
The ten-test Hollow rerun is in progress; affected Boolean/assembly/splitting
qualification remains pending on this build. These changes are not yet pushed.


Native qualification checkpoint (2026-10-30, 02:42): build
`wip-build-20261030-0228-hollow-repairs` passes CurveIntersection 27/27,
BRep Native Section 7/7 and BRep Native Measure 13/13. Exact homogeneous
chart translation preserves original parameters and unchanged tolerance.
Certified disjoint control hulls can resolve negative events before the
coordinate guard. Optional polynomial-cache preflight now falls back to the
existing integration path instead of rejecting an otherwise measurable solid.
All three corrections retain their original controls and independent reviews.

The saved curved-retained Hollow constructs and passes boundary validation,
but the unchanged volume gate still fails at 200001 work units: 197041 inner
evaluations, 297 outer evaluations, zero adaptive subdivisions. This identifies
repeated prefix integration, not difficult refinement. OCCT plane-flux and
interval reuse are being studied for a native bounded-cache correction.
The affine rational-mouth Hollow progresses past its original section clipping
failure, then fails in face arrangements at the coordinate-resolution guard.
It is not accepted. Exact caller capture and OCCT comparison are next.

Current unpublished repairs require the affected Boolean/assembly/split suites
and a clean publication review; the published 28fa434 CI remains separate.
Final chair quality images, same-build chair matrix, general offset coverage
and Rust/Python ports remain pending. Existing five chair previews are in
`C:/pc/tmp/session-migration-20261028/chair-results-all-five-20261030/`.


Native implementation checkpoint (2026-10-30): observer build
`wip-build-20261030-0305-cc-observer` passes Curve Surface Native 27/27,
BRep Native Boolean 27/27, BRepAssemble 15/15, Split 20/20 and Intersection
89/89. The affine-Hollow failure is captured exactly in
`hollow-affine-guard-capture-v1.jsonl`: two rational boundary lines share the
exact raw endpoint (2,2), but absolute UV magnitude 1040 triggers a guard
7.3896444519e-12 above requested UV tolerance 5.5047289972e-12. A scoped
parameter-only arrangement translation is source-reviewed; only its strict
regression and unchanged fixture are promoted for baseline RED qualification.

The reviewed volume implementation now uses one global plane-flux field,
conservative trimmed support cells and reuse of proven-identical C1 derivative
coefficients. Nonzero UV closure gaps receive outward error bounds reserved
from each face/pass tolerance before cache construction. The work limit and
tolerance are unchanged. Source and two analytic qualification tests are
promoted but unbuilt. No hard-Hollow volume success is claimed. An independent
full-NURBS OCCT8 JSON converter is prepared outside the repository for raw
geometry volume comparison; the older planar reference executable is unsuitable.

### 2026-10-30 public regression continuation

Fresh native Boolean group passes 27/27 after plane/split repairs. Six legacy
public Boolean tests are restored without OCCT guards; their new runtime gates
are pending. Concave context candidate72bd still fails the strict positive test
at the orthogonal/equal-radius guard on source edge15; preserve this failure and
analyze actual geometry against OCCT before modifying the construction.

### 2026-10-30 restored public matrix results

On native build `wip-build-public-legacy-six-v1` (mini SHA32ed0c18...):
38 box configurations, all Platonic solids, and solid/cavity configurations pass.
Scale/translation fails full same-parameter certification; curved seams/poles
fails paired pcurve domain consistency. Neither is waived. Failure-only witnesses
are prepared for the next build. The original Rhino all-five public test is
restored but not yet rerun on this build.

The concave offset failure is a model error: an enlarged working rectangle border
was mistaken for the original edge's physical offset image. OCCT's original-path
and generated-edge ancestry (`BRepOffset_MakeOffset`1935â€“1995,
`BRepOffset_Offset`1112â€“1195) separates these from working support extension.
Reviewed correction f792e5e derives from the original spine and normal, projects
both physical tube boundaries onto their working supports and retains full paired
pcurve certification. Source applied; strict positive test remains pending.

### 2026-10-30 qualified concave and translated-coordinate repairs

Native full Hollow group passes 10/10 on build3a10d4b0, now with the strict positive
concave prism test replacing the historical rejection. Both thicknesses .1/.2
pass signed volume, physical mouth, material probes, topology and complete pcurve
checks. Repairs preserve original-spine ancestry, include interior tangent cuts,
and directly construct exact bilinear raw knot endpoints. Existing curved-wall
results remain exact geometry/topology matches to the independently OCCT-validated
saved input. Full general concave/freeform hollow coverage is still incomplete.

Exact common homogeneous translation repairs the public million-coordinate box
case without changing tolerance or raw parameters: CS 31/31, public scale test and
native Boolean 27/27 pass. A source-reviewed single-pass wrapper avoids duplicated
normal-path validation; its fresh build/run is pending.

Sphere sections lose world parameters through chord-length pcurve construction.
The captured ordinary/wrapped witness fails the strict regression. Reviewed native
repair retains raw parameters and accepts adaptive pcurves only with full interval
same-parameter certificates. Both-order coverage and initial work caps are required.
Source applied, native acceptance pending; short curves/isolated contact events
remain separate open work.

### 2026-11-01 Rhino reference fixture audit

The user's `RhinoBooleans` folder contains seven input pairs and 21 output solids.
All 28 files are readable; every object is valid, closed, manifold and outward.
Names match geometry: the pyramids have pentagonal bases; Twisted has six
nonplanar bilinear faces per solid. Units are millimetres, model tolerance 0.001.
An independent headless Rhino audit checked 18,868 usable material probes,
including all four input membership regions in every family. Originals are unchanged.

Nineteen output references agree with their inputs. A and B mean the first and
second original file objects, with GUIDs retained. Difference is A minus B for
Boxes, Cones and Pyramids, and B minus A for Cylinders, Torus and Twisted.
Sphere intersection and difference are excluded: their second sphere is at Z=0,
whereas the current input and union put it at Z=3.490701324253359. Both excluded
outputs contradict 86 current-input material probes. Sphere union remains usable.

Next: export each operand separately through Rhino STEP, independently check
roundtrip geometry, then qualify native Session import before running Booleans.
Native acceptance requires whole pcurve agreement, closed topology, signed volume,
independent material probes and unchanged operands. Finite reference probes alone
are not a proof of general Boolean completeness. No new Rhino fixture native pass
is claimed yet. Rhino is an external reference/conversion tool, never a runtime
dependency or Boolean fallback.

Current native checkpoint passes Intersection 109/109, native Boolean 27/27 and
public Boolean 3/3, including all five opposite-saddle operations. Two new strict
critical-section tests still reject at the coordinate-resolution guard. Their
original failed geometry is captured; a directed whole-polynomial residual proof
is under independent review. Tolerances and work limits remain unchanged.
Offsets and the Rust/Python ports remain deferred until C++ Booleans qualify.

### 2026-11-01 native Rhino import failure capture

Build `cf1db858` passes Curve Surface Native 37/37, all three strict critical
section tests, Intersection 109/109, public Boolean 3/3 and native Boolean 27/27.
The directed residual correction resolves both recorded critical-section failures.

All 14 Rhino inputs pass independent STEP roundtrip checks. Six flat output
conversions pass; 13 curved output conversions change volume by 0.0068--0.0308
mm cubed and are excluded without relaxing the conversion gate. Original Rhino
oracle volumes and occupancy data remain authoritative.

Native import diagnostics on those 14 qualified inputs all fail whole pcurve
agreement, despite valid closed topology. Boxes, pyramids and twisted solids fail
at endpoints; curved primitives also fail inside the parameter span. Exact world
curves, pcurves, supports and original STEP bytes are retained for replay.
The affine projector loses original surface domains and assumes orthogonal axes;
the projected-surface path ignores supplied STEP pcurves and resamples boundaries.
These importer failures must be repaired before claiming these Boolean cases pass.
