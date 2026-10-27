# Native session BRep translation

## Contract

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

## Sequence and gates

| Stage | Implementation | Acceptance gate | Status |
| --- | --- | --- | --- |
| 0 | Inventory native primitives; pin reference revision; create OCCT-free build and dedicated native regression target | Compile and execute with `SESSION_WITH_OCCT=OFF`; retain reference reports separately | Complete locally |
| 1 | Geometric tolerance policy; conservative NURBS bounds; trim classification; solid IN/OUT/ON/UNKNOWN classification | Rational trims, holes, seams, poles, cavities, far translations, small scales; ambiguous cases never become OUT | Certified ray roots integrated;12 ray and11 classifier groups pass; ambiguous boundary/singular cases remain explicit |
| 2 | Vertex/vertex, vertex/edge, edge/edge, vertex/face, edge/face interference; sorted edge split parameters and common intervals | Tangencies, overlaps, multiple roots, tiny spans, endpoint contacts, positive rational weights; residual checks | Native CC/CS solvers integrated; complete persistent pre-FF graph unfinished |
| 3 | Native surface/surface intersections with paired pcurves, adaptive patch bounds and seeds, continuation, closed loops, tangent/coincident regions | Doubly curved pairs, internal loops, branch points, seam crossing; sampled and adaptive world/pcurve residual bounds | Two-loop continuation still times out; exact bounded chart witness retained. Mobius, narrow-span and duplicate-chair-endpoint regressions pass. Singular/coincident completeness unfinished |
| 4 | Shared interference graph and edge splitting; trim intersections against both faces; parameter-space arrangement and face reconstruction | Holes, disconnected regions, coincident boundaries, periodic domains; adjacent faces share the same split edge | Full parameter certificate and quartic loops pass. All six original stationary/tangent direction groups pass; new rotated tangent case explicitly unresolved |
| 5 | Classify split regions; Boolean truth tables and orientation; sew shells; nest cavities; assemble disconnected solids | Union, common, A-B, B-A, XOR; closed oriented manifold output and no unused geometry; no mesh reconstruction | Earlier190 box operations and25 rotated Platonic operations passed; rerun after current changes pending; chairs not accepted |
| 6 | Native public APIs and acceptance corpus, with OCCT only in separate oracle programs | All chair operations and configured hard cases pass in OCCT-free executable; compare geometry/topology/volume, not volume alone | Planned |
| 7 | Native curve/surface offsets, approximation error control, BRep joins, intersections, trimming, self-intersection removal, collapse and hollowing | Signed offsets, topology changes, singularities, all Platonic solids, freeform chairs; failed outward chair cases remain visible until resolved | Four strict six-support inward saddle cases and rational thickening pass. Mixed concave tube construction staged; global cleanup and hollowing unfinished |
| 8 | Review, reproducibility, complete C++ suite, saved native images and reports | Build without OCCT on supported platforms; render actual output, preserve failed cases, open evidence folder | Windows milestone verified; full native Boolean/offset gate unfinished |
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

## Execution record — 2026-10-26

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
  cavities, unused face pools, scales 0.001–1000, far translations, invalid inputs,
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
stages 2–9 remain unfinished and native solid Boolean acceptance has not passed.

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
