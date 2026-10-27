# OCCT interference and reconstruction audit

Reference: `C:/pc/occt8`, revision `d3056ef80c9668f395da40f5fd7be186cae4501f`.
Reviewed 2026-10-26. Paths below are relative to
`src/ModelingAlgorithms/TKBO/` in that checkout. This is a source comparison,
not a statement that these algorithms have already been translated.

## Execution order is part of the algorithm

`BOPAlgo/BOPAlgo_PaveFiller.cxx:234`, `PerformInternal`, performs VV, VE, EE,
VF, EF, then FF. Between stages it updates blocks with same-domain vertices.
At lines 290-307 it repeats intersections affected by enlarged vertex tolerances
and forces EE/EF reconsideration. At lines 318-350 it synchronizes shared
vertices, creates split edges, constructs section blocks, updates interferences,
removes micro edges, builds pcurves, and handles degenerate edges.

Our `brep_boolean::apply` starts with face/face curves and partitions each face.
`brep_assemble::synchronize` discovers edge contacts after face selection. This
omits the earlier interference graph on which OCCT's face builder relies.
The independently implemented CC and CS solvers are necessary primitives;
calling FF first does not reproduce OCCT's Boolean algorithm.

Required change: build typed VV/VE/EE/VF/EF records, carrying original operand,
edge, face and parameter identities, before FF reconstruction. Propagate each
edge event to every adjacent face. A contact point and a coincident interval
must remain different records. Preserve unresolved failures through every stage.

Evidence for the distinction: `BOPAlgo_PaveFiller_3.cxx:369,529` handles EE
`TopAbs_VERTEX` and `TopAbs_EDGE` separately; `_5.cxx:406,545` does the same for
EF. Neither is interchangeable with an arbitrary face sample being ON.

## Shared intervals must survive reconstruction

`BOPDS/BOPDS_PaveBlock.hxx:35` defines an interval by two adjacent paves.
`BOPDS_PaveBlock.cxx:253`, `Update`, sorts added paves and gives every resulting
interval the original edge identity. `BOPDS_CommonBlock.hxx:35` groups coincident
intervals and faces; its representative supplies one shared edge identity.

`BOPAlgo_PaveFiller_7.cxx:371`, `MakeSplitEdges`, processes each common block
once. Lines 439-454 can reuse an original edge and update its tolerance;
lines 539-546 assign the newly created edge to the entire common block.

Our split API already returns `EdgePiece {source_edge, section, edge, first,
last}` and the assembler accepts it. The Boolean caller discards that data by
taking only `.brep`, then leaves `FaceUse::pieces` and `operand` unset.
Consequently source identity is lost and assembly re-solves geometric overlap.

Concrete repair: retain a partition result with BRep plus persistent edge
lineage. `split.cpp` appends edges and replaces wire references without compacting
edge pools. For each later split, inherit original edge/section identity from its
parent edge; do not overwrite the original identity with an intermediate edge
index. Trimmed curves retain original parameters. Pass lineage and operand IDs
to assembly. Test repeated adjacent-face splitting and identical section edges
with different subdivision histories before relying on that identity.

This repairs one omission; it does not replace the complete interference graph.

## Face/face curves are processed with existing topology

`BOPAlgo_PaveFiller_6.cxx:649`, `MakeBlocks`, consults the existing ON/IN
vertices and intervals of both faces. Lines 773-791 process isolated FF points.
Lines 796-854 put known, boundary and closing vertices on section curves.
`FilterPavesOnCurves` rejects a vertex attached to the wrong nearby section.
The source explicitly references regression `bugs/modalg_6/bug26789_1`.

At lines 902-956 it checks each interval against both trimmed faces and verifies
that its interior is not entirely consumed by endpoint tolerance spheres.
At lines 963-1023 it reuses an existing interval and attaches it to the other
face when necessary. Lines 1024-1045 construct one edge with pcurves on both faces.

Our `brep_section::faces` clips paired pcurves at trim intersections and checks
interval midpoints. It does not retain isolated FF contacts or a shared vertex
graph. Our split vertex merge uses only distance, and assembly has no explicit
equivalent of the valid-range/tolerance-sphere treatment. Short intervals may
therefore be topologically significant even when a simple length test suggests
otherwise. Do not fix these cases by deleting short edges indiscriminately.

Required tests: two distinct nearby branches, a section endpoint within a source
vertex tolerance, an isolated tangent contact, an interval lying on an existing
edge, a closed section with a seam, and partial overlap shared by three faces.
The OCCT spike regression requires external `bug26789_All-Spikes.brep`; its test
script exists locally, but availability and native import of its data must be
checked before claiming it is covered.

## Pcurves require parameter correspondence

`BOPTools/BOPTools_AlgoTools.cxx:1649`, `MakePCurve`, prefers the section's
existing pcurves, adjusts them for each face and periodic range, and calls
`BRepLib::SameParameter` at line 1716. Geometry lying on the same edge is not
sufficient: the parameter used by both representations must identify the same
point. `BOPTools_AlgoTools2D_1.cxx:94-132` also adjusts ranges and rebuilds
same-parameter correspondence when attaching an existing pcurve.

Our new SSI fitter shares interpolation parameters. However,
`split::add_shared_pcurves` and `brep_assemble::piece_pcurve` locate the endpoints
of reused trims and preserve the interior parameterization. Split validation
checks closest-point distance, which can pass for an incorrect parameter map.
Assembly must validate the lifted pcurve against the world edge at matching
parameters, including a derivative/residual bound or controlled refinement.
Do not confuse nearest-curve proximity with this invariant.

Required regression: geometrically coincident rational curves with unequal
parameter speeds, reversed uses, unequal knot domains, and both pcurves of a seam.

## Implementation gates

1. Preserve lineage through repeated splits and test it.
2. Establish same-domain face groups from matched split boundaries/supports;
   the selection audit details this prerequisite.
3. Enforce same-parameter world/pcurve correspondence after edge sharing.
4. Add the missing typed pre-FF interference graph and propagate its events.
5. Preserve isolated/tangent FF events and tolerance-aware small intervals.
6. Run analytical fixtures, freeform fixtures and the original chair operations;
   compare topology, source surfaces, paired residuals, classifications and volume.

The current implementation has not passed these gates. Passing primitive or
solver tests alone does not establish native solid Boolean completion.

## XOR shell failure: first source-guided correction

The first transverse-box Boolean run passed union, common and A-B, then failed
XOR with `BRep assembly: open or nonmanifold edge 6`. Exact result geometry of
the three completed operations and the failing test log remain in the evidence
folder. The selected XOR boundary has four uses of a section edge belonging to
two separate material shells. A global two-face incidence restriction is wrong;
the restriction must hold within each shell.

`BOPAlgo/BOPAlgo_ShellSplitter.cxx:270-388` grows shells by choosing an opposite
oriented face use at each free edge, using `BOPTools_AlgoTools::GetFaceOff` when
there are multiple candidates. `BOPTools_AlgoTools.cxx:994-1102` compares inward
face directions in a plane normal to the edge, orders them by oriented angle,
and reports angular ambiguity. Its `GetFaceDir` helper at line 2110 also obtains
interior probes for curved/tangent cases rather than relying on distance alone.

Native correction: retain all balanced edge uses, pair opposite uses by oriented
inward-binormal angle, require reciprocal unambiguous pairing, grow shells only
through the chosen mates, and enforce two uses per edge within every resulting
shell. Regular transverse faces use their limiting surface normals. Tangent
angular ambiguity still fails explicitly; the OCCT interior-probe treatment is
needed before claiming support for ambiguous tangent branches. The original XOR
test and edge-contact boxes must pass before this change is accepted.

The source-guided transverse-box rerun passed all four operations and reversed
difference. An independent agent verified the oriented-angle sign and identified
an excessively small ambiguity threshold in the first patch. OCCT's
`GetFaceOff` uses `Precision::Confusion()` for that comparison; the native check
now uses the corresponding `1e-7` angular separation instead of machine epsilon.
This does not establish ordering through an interior tangent event: such events
must split the source edge before shell pairing. Full face-interference coverage
and near-face probes remain required for those cases.

## Freeform saddle failure: duplicate paves

The first doubly curved saddle Boolean failed while matching representative
pieces, rather than intersecting original source edges. Diagnostic domains were
both near `0.006866` and their control hulls had collapsed numerically. The CC
solver correctly rejected a constant regular curve. An independent agent traced
the cause: `synchronize` accumulates intersections from multiple edge pairs,
while `sorted_cuts` merged parameters only at machine precision. Each individual
CC result can be deduplicated and still introduce duplicate paves across pairs.

OCCT updates same-domain vertices before split-edge construction
(`BOPAlgo_PaveFiller.cxx:266-288,318-328`). `BRepLib_1.cxx:173-258`,
`FindValidRange`, checks for a meaningful interval outside the endpoint tolerance
spheres. `BOPAlgo_PaveFiller_6.cxx:4388`, `RemoveMicroEdges`, treats split intervals
with the same endpoint vertex and no valid shrunk range. It does not declare
every short or closed curve degenerate.

The native correction coalesces consecutive paves only when the entire trimmed
positive-rational control hull fits inside one quarter of the vertex tolerance,
including coordinate roundoff. This is a sufficient whole-interval bound, unlike
checking endpoint distance or a fixed parameter gap. The real source-domain end
is retained; a whole regular edge consumed by its vertex tolerance remains an
explicit failure. Original closed curves with nontrivial control hulls remain
untouched. This is the local repair for duplicate paves; a persistent shared
vertex/common-block graph is still needed for full tolerance propagation.

The original saddle Boolean and reconstruction/seam/partial-partition groups
must pass after this correction. It is not accepted merely because the constant
curve exception disappears.

The next Boolean run passed 9/10 groups. The saddle advanced past collapsed
intervals and stopped on unresolved SameParameter certification. The overlapping
trim-cut reconstruction now passes; the saddle is still an open acceptance item.

## Chair reconstruction mesh regression after SameRange

The expanded reconstruction test preserved the original assertion and writes
`build/native-geometry-tests/reassembled_chair_0.diagnostic.json` plus both input
and result BReps and meshes before checking it. The original mesh volume is
`2919.833584347105`; the rebuilt mesh volume is `2919.8411980425567`. Both meshes
have 1753 vertices and 3502 triangles. Surface control points, knot vectors,
orders and rational flags are identical. Face counts and orientations also agree.

Nearest-vertex comparison gives a maximum bidirectional gap of approximately
`1.0344e-7`. After matching vertices geometrically, 3484 of 3502 triangles match;
the other 18 use different diagonals. Most signed-volume change is on faces 14
and 12. Normalized pcurve evaluations agree to about `1e-14` UV. This isolates a
meshing/sampling sensitivity, rather than lost BRep surfaces or reversed faces.

Source comparison before a fix:

- `BRepMesh_CurveTessellator.cxx:157-193` refines the same canonical edge parameter
  intervals on every pcurve only when SameParameter and SameRange hold.
- `BRepMesh_EdgeParameterProvider.hxx:90-156` reuses established shared parameters;
  otherwise it scales parameter increments and projects monotonically, guarding
  against reversal caused by a projection onto a different periodic branch.
- `ModelingData/TKGeomBase/CPnts/CPnts_AbscissaPoint.cxx:42-57` integrates the
  exact adaptor D1 magnitude for both 3D and 2D curves.
- Session's `NurbsCurve::divide_by_count` instead calls `arc_length_gauss` and
  `find_t_at_s`, which estimate speed from finite differences with step
  `domain_length * 1e-8`. Those estimates can change after affine reparameterizing
  the same geometric pcurve.

Direct original/reparameterized pcurve arc-length sampling diagnostics have been
added to the preserved test. They must establish this causal link before replacing
the speed evaluation. The mesher's geometric tolerances and the original volume
assertion remain unchanged; no CDT tolerance adjustment has been made.


Direct sampling now confirms the causal link: face14, edge43, pcurve57 reproduces
exactly the mesh maximum world gap1.0344028993621736e-7, with normalized parameter
gap6.3897882540686624e-9. The pre-correction diagnostic is retained separately.
The correction integrates analytic D1 and adds every distinct knot to the length
table, following GCPnts_UniformAbscissa.cxx:54 and GCPnts_AbscissaPoint.cxx:97-135
which respect continuity intervals before integrating speed. Newton inversion
stays inside a single such interval. A preserved original chair pcurve and a
piecewise linear curve with nonuniform knots test affine-domain invariance and
known equal-length points. The original chair volume assertion is unchanged.

## Chair paired-surface fit duplicate endpoints

The400-pair scan initially failed to interpolate9pairs. After certified hull
coverage changes, two retain the exact fit error: (15,18) has36points with its
first chord increment zero, and (19,0) has313points with its last increment zero.
The full point/parameter arrays and both support surfaces are saved in the
per-pair viewer JSON. A separate OCCT oracle, fed these native surface control
points and knots, finds one support section for all9originally failing pairs.
Face trimming can remove a support section, so direct native support diagnostics
are required before treating the new empty face results as correct.

OCCT prerequisites reviewed before correction: IntPatch_Intersection.cxx:1335-1373
postprocesses walking lines only when the caller requests it and IsPurgingAllowed.
IntPatch_WLineTool.cxx:1468-1590 removes equal world/UV samples while updating
vertex indices; it separately guards against smoothing across C0 continuity.
ApproxInt_Approx.gxx:87-163 computes one chord parameter sequence from the combined
3D and 2D walking data. Session's paired-surface path refines UV samples but
then directly computes world chords, so duplicate endpoints create repeated
parameters rejected by create_interpolated. Paired UV diagnostics are being added
before deciding the exact native cleanup condition; world proximity alone must
not merge distinct surface branches.


The paired diagnostics now show exact equality in world and all four surface
coordinates for the adjacent samples causing both failures. The native correction
removes only adjacent exactly-equal four-parameter records before common fitting,
including after each refinement pass. It does not compare nonadjacent samples or
merge distinct preimages of the same world point. The closing record of a loop
remains because it is not adjacent to the start; too few distinct records remain
an explicit error. The two persisted source-surface fixtures require one section
per the independent OCCT oracle, common domains, dense residual checks and full
native world/pcurve correspondence bounds.

The other seven formerly failing chair pairs now produce one direct support
section each, matching OCCT's count; five have no section left after face trimming.
That distinction is now explicit in the diagnostic report. The complete400-pair
scan and original chair Boolean operation still need rerunning after cleanup.

Analytic speed plus knot-split integration passes both focused length regressions.
It reduces the chair's maximum sampling shift from1.03e-7 to3.36e-12, but the
unchanged mesh-volume comparison still fails (2919.838860056678 versus
2919.8328056972055). Small changes still choose different UV Delaunay diagonals.
This is preserved as a separate remaining meshing problem, not marked repaired.
OCCT CircleTool uses PConfusion and expands its radius by2*RealEpsilon; the
CircleInspector itself documents a tolerance issue and perf mesh bug27119. Those
choices must not be copied as an unexplained global CDT tolerance increase.

## Rotated Platonic classification failures: preserved caller prerequisites

The first complete rotated matrix passed the tetrahedron and octahedron but all
five operations on the cube, dodecahedron and icosahedron returned an unresolved
face classification. Inputs and operation reports are preserved under
`build/native-geometry-tests/viewer/native_platonic*`; the original log is
`build/native-platonic-matrix-first.log`. No classification tolerance or ray count
has been changed. A typed diagnostic exception now carries the exact classified
partition and query back to the caller; the test records all seven original ray
results, including completeness, trim states, crossings and parameters. This
diagnostic addition is not yet built and does not repair classification.

OCCT call chain reviewed before any behavioral correction:

- `TKBO/BOPTools/BOPTools_AlgoTools.cxx:660-714` first tries an edge that is not
  part of the reference boundary, then an interior face point if all edges are
  shared; the latter uses the hatcher and a near-edge fallback. It does not
  assume the support-surface midpoint is inside a trimmed face.
- `BOPTools_AlgoTools3D.cxx:906-933` gets trimmed UV bounds and tries a hatch
  line through an intermediate U parameter, then an alternate direction.
- `BOPTools_AlgoTools.cxx:790-803` delegates the actual world point to the cached
  solid classifier with the requested tolerance.
- `TKTopAlgo/BRepClass3d/BRepClass3d_SClassifier.cxx:207-310` checks tolerance
  regions of vertices and edges first, obtains a face-directed segment, and
  retries when a point lies on a support but outside its trimmed face.
- The same classifier at lines 413-509 checks intersection completion, computes
  a parallel-support projection when needed, and derives state from the nearest
  resolved transverse crossing. Boundary crossings force another segment.
- `TKTopAlgo/IntCurvesFace/IntCurvesFace_Intersector.cxx:232-358` classifies every
  intersection in the trimmed face, converts geometric tolerances using the
  surface adaptor resolutions, and flips crossing transitions for reversed faces.

Session also requires a trim-interior witness, checks boundary distance and uses
oriented crossings, but its exact failing stage is not established until the
saved query/ray diagnostic runs. In particular, increasing retries or treating
Unknown as Outside would conceal the defect and is not a correction.

## Confirmed grazing-ray duplicate cause and convergence correction

The saved rotated-cube query is `(-1,0,0.5000000000000002)`. All seven rays
reported complete; therefore incomplete AABB candidates were not the cause of
this particular failure. Ray `(-.217,1,.413)` accepted three entry hits on the
same face before one exit. Their lateral residuals (3.56e-9,1.89e-9,7.19e-9)
were all below the existing1.25e-8 threshold, while grazing incidence0.040268
amplified the along-ray errors. The independent70-digit calculation on the
original bilinear coefficients gives exactly one root at distance
0.5486470850147471. One accepted point was1.72e-7 from it. Adjacent duplicates
were farther apart than the unchanged1e-7 merge tolerance.

`IntImp_IntCS.gxx:113-152` requires the bounded three-variable root solver to
finish before checking geometric residual. Its tolerances include curve/ray
parameter resolution as well as U and V. `math_FunctionSetRoot.hxx:65-78`
requires every unknown's increment to converge; the caller fills these increments
at `math_FunctionSetRoot.cxx:1295-1300`. Candidate polishing uses the original
adaptors in `IntCurveSurface_InterUtils.pxx:1460-1500` before duplicate handling.

The native correction evaluates Newton on the original support and also requires
the full world displacement implied by its UV correction to be below the existing
tol/8 convergence allowance. This includes the previously unchecked along-ray
displacement. Iteration limits, geometric tolerance, merge tolerance and ray
agreement logic are unchanged. The exact failure is preserved in
`tests/fixtures/native_rotated_cube_classification_failure.json`; a focused test
requires one entry/exit and the independent root distance at three world scales.
The correction is not yet built/tested. A small local Newton step is not a
general root-count/coverage proof; interval isolation and parameter-branch
preservation remain separate requirements.


**Focused validation of the grazing-root correction.** The rebuilt native executable passes `Grazing Ray Has One Root Per Face` at all three scales, and `Rotated Cube Classification` passes the complete saved Boolean operation. Logs are `build/native-grazing-ray-fixed.log` and `build/native-rotated-cube-fixed.log`. These establish the observed duplicate-root correction only. A separate original-support interval ray module is being implemented to certify miss/root coverage and parameter-branch identity; the existing subdivision termination remains insufficient for that broader claim.


## Differential vector norms erased by an application tolerance

The near-planar offset cone exposed a shared numeric prerequisite violation: `Vector::compute_magnitude` returns zero when every component is below `Tolerance::ZERO_TOLERANCE=1e-12`. The actual cone residual has components near2.5e-19; its nonzero magnitude is essential because it is compared relative to another small differential, not to a world-distance tolerance. `gp_Vec.hxx:172` delegates magnitude to `gp_XYZ::Modulus` (`gp_XYZ.hxx:156`), which computes the Euclidean norm without applying a geometric tolerance. Admissibility for normalization is a separate caller decision in `gp_Vec.hxx:314-319`.

The first regression requires scale invariance for the exact3-4-12 direction, retains a representable subnormal axis norm, and reproduces the near-plane differential directly. The fixture is `tests/fixtures/native_vector_differential_scale.json`. No production Vector change accompanies this first regression build. The intended native correction is an overflow/underflow-resistant Euclidean norm, leaving `is_zero` as the explicit tolerance predicate; general normalization and downstream degeneracy behavior must be retested.


The independent review also found that a norm-only change would expose a normalization defect: for `(denorm_min,denorm_min,denorm_min)`, the rounded norm cannot support accurate direct division, while finite near-DBL_MAX components may have an overflowing norm. Existing normalization also accepts NaN/Inf because `d<=0` is false. A second regression therefore requires scale-normalized directions for subnormal and large finite components, and explicit rejection of nonfinite components without modifying them. Max-component scaling provides the native correction; OCCT's direct square-root expression is not used as a subnormal numeric oracle.


**Preserved first Vector failures and correction.** The unchanged implementation fails `Magnitude Preserves Differential Scale` at its nonzero-length assertion and `Normalize Extreme And Invalid Vectors` at subnormal normalization. Logs are `build/native-vector-scale-first.log` and `build/native-vector-normalization-first.log`. `compute_magnitude` now uses `std::hypot`; normalization first rejects nonfinite coordinates, divides by the largest absolute component, and normalizes the bounded ratios. The explicit tolerance predicate `is_zero` is unchanged. These changes are awaiting the next build and broad dependent tests.


## Chair Boolean face splitting: filtering candidates before counting work

After all 400 chair face pairs passed, the full common operation failed after
231.328 seconds with `Face split exceeds the bounded workload`. The immutable
report is `native_chairs_intersection.first_split_workload_failure.json` in the
viewer evidence folder. The three exact paired cuts on operand0 face0 are saved
in `tests/fixtures/native_chair_face_zero_cuts.json`; the direct regression
reproduces the unchanged failure in166.224 ms. The curves contain207,199,53 knot
spans, respectively. Their459 spans alone exceed the old `N*N<=200000` check,
before adding the original face boundary. This occurs before any geometry test.

OCCT `BOPDS_Iterator.cxx:276-301` first builds a BVH and selects overlapping shape
boxes. `BOPAlgo_PaveFiller_3.cxx:224-249` rejects disjoint pave-block bounds before
creating an edge-edge task. The prepared In/On/Section edges pass through
`BOPAlgo_Builder_2.cxx:473-513` into the face builder; `BOPAlgo_WireSplitter`
constructs connectivity rather than intersecting a knot-span Cartesian product.

The native correction bounds every original knot span using its active original
positive-rational basis controls. Euclidean control coordinates and tolerance
expansion are rounded outward. A two-dimensional existing SpatialRTree enumerates
only unordered overlapping candidates. The same200000 limit applies to actual
candidates, with a separate200000 span memory cap. Same-source adjacent and
nonadjacent candidates remain eligible; parameter and section lineage and the
intersection solver are unchanged. Bounds use original controls rather than
claiming ordinary trimmed coefficients enclose the original curve. A fixture-only
independent analysis finds1361 possible cut-span pairs under deliberately larger
UV padding, compared with105111 unfiltered unordered cut pairs. The correction
is awaiting the next build and unchanged splitter regressions.


**Focused post-correction results,2026-10-27.** All25 Vector groups pass after the norm/normalization correction. All10 Surface Ray groups pass, including certified crossing orientation for every dominant axis, both ray directions, reversed U and transposition. The preserved dense chair face splitter passes in1172.1 ms after the bounded R-tree candidate search. Logs: `native-vector-fixed.log`, `native-surface-ray-crossing.log`, and `native-chair-dense-split-broadphase.log` under build. No full chair Boolean pass is implied.

The classifier now consumes the independently reviewed original-support ray module instead of accepting one Newton root per tiny leaf. Parameter-distinct roots are retained. Crossing signs come from the interval chart certificate and face orientation. Trim ambiguity uses the outward sum of the root enclosure radius and the existing parameter tolerance; near-origin and crossing-separation decisions use distance enclosures. Unresolved support/trim cases still force another ray or Unknown. The pre-existing closest-boundary search and approximate pcurve winding remain independent limits, so this is not a claim of a fully certified solid classifier. Integrated classification regressions are pending.


## Supporting-surface root at the ray origin, outside a face trim

The first certified-classifier integration passes ten of eleven groups, including
all forty original chair face-side checks. `Bore Trims` fails at `(0,0,1)` on the
top supporting plane of a 4 by 4 by 2 block with a unit cylindrical bore. The
point lies in the hole, so the expected solid state is Outside. The first log is
`build/native-classification-certified.log`; exact shape and all seven original
ray diagnostics have been added to the unchanged failing assertion.

OCCT `IntCurvesFace_Intersector.cxx:254-297` classifies each support root in the
trimmed face before admitting its ray parameter. `BRepClass3d_SClassifier.cxx`
explicitly skips an ON-support/OUT-face witness (lines 298-304), extends the ray
segment slightly behind its origin (around lines 400-418), and applies the ON
state only to retained face intersections (around lines 463-478). The native
module currently insists that an isolated root's distance enclosure be strictly
positive or negative before returning it. A zero-distance root therefore cannot
reach the caller's trim rejection. This is the leading source-derived hypothesis;
the exact diagnostic run must confirm it before the implementation changes.


The bore diagnostic confirms all seven original rays fail on face 6 with root
isolation depth exhaustion. The immutable shape/query/rays fixture is
`tests/fixtures/native_bore_origin_classification_failure.json`. The reviewed
opt-in origin-candidate reporting retains the existing root certificate and lets
the caller reject an OUT-trim root before applying the origin-distance rule.
The correction and direct ray-origin regressions are awaiting a build.

## Chair trim sampling: geometric deflection versus projection roundoff

Full chair common advanced to `Trim sampling exceeds the bounded workload` after
318.353 seconds. The immutable first report is
`native_chairs_intersection.first_trim_sampling_failure.json`. Replaying the 31
saved paired sections reproduces it in 26.617 seconds at operand 0 face 15. The
exact partition, cutters, input curve and active leaf are preserved in
`tests/fixtures/native_chair_trim_sampling_failure.json`.

The requested world tolerance is 1e-6, UV tolerance 4.739172895351418e-8, and
flatness allowance 1.1847932238378545e-8. Ordinary basis evaluation at the clamped
endpoint yields 0.9999999999999999, moving the endpoint by 3.66e-15. On the
1.0655745848828772e-6 leaf chord this gives a first-CV projection of
-2.705068382704035e-9. The former flat predicate rejects it against the fixed
1e-12 longitudinal epsilon, despite a clamped segment distance below 4.5e-15.
The complete numeric audit is `build/native-geometry-tests/chair_trim_sampling_numeric_audit.json`.

OCCT GCPnts_QuasiUniformDeflection.cxx:100-145 separates geometric deflection
from tangent/chord formulas for short chords and warns that parameter midpoint
deflection is distorted by nonuniform parameterization. Lines 586-600 separately
check adaptor resolution against parameter ULP. Its face classifiers retain an
exact fallback for polygon ambiguity; coarser sampling cannot be copied without
that prerequisite.

The native correction tests every positive-rational control against the closed
chord capsule. Raw homogeneous control division, chord construction and squared
distance are rounded outward, and the distance upper bound is compared to a
lower bound on the same tolerance squared. Convexity makes this sufficient for
the represented leaf. The chosen projection may be approximate; it only chooses
a point on the chord. No geometric tolerance, subdivision limit or acceptance
assertion changed. Independent review confirms this local proof; ordinary trim
rounding still prevents claiming an exact-original-source sampling certificate.


**Post-correction validation.** All 12 Surface Ray groups and all 11 solid
classification groups pass, including the preserved bore and original chairs.
The exact saved face-15 trim regression passes in 2.63 seconds. Replaying all
31 saved chair sections through both complete face partitions passes in 198.66
seconds. This validates partitioning only; full Boolean selection/assembly and
result acceptance still need to run.

## Knot spans versus topological pave vertices

The inward selector now retains the unique region after verifying one-to-many
section ancestry covers each expected interval exactly once. It builds a closed
solid but the unchanged topology assertion finds 408 edges and 404 vertices
instead of 12 and 8. The first result and assertions are preserved under
`native_offset_inward_standard_small.first_topology_failure*` in the viewer folder.
The face arrangement has 147 pieces on four section curves solely because each
curve's knot spans were emitted as separate topological edges.

OCCT BRepAlgo_Loop.cxx:609-613 cuts edges at stored vertices. CutEdge at 790ff
orders those vertices and retains the original edge when the list is empty. A
spline knot does not itself create a topological vertex. The native correction
marks genuine cross-source hits and overlap endpoints in the arrangement before
arc canonicalization. Only an unmarked degree-two vertex may join consecutive
monotone runs of the same original source at an exactly shared parameter. Cyclic
first/last runs receive the same check. Polygon sampling and source parameter
ranges are retained. Independent review found the ancestry and pave safeguards
sound; within-span self intersections must also mark their two parameters once
the new helper is integrated. The change is awaiting build and unchanged inward
topology validation.


### Inward-offset reverse-cycle nesting, preserved failure

The four strict six-support saddle offset regressions failed after ordinary-knot contraction. The large standard and affine cases fail on source face 3: the positive section cycle is assigned its own reversed cycle as a hole, while the original rectangular boundary is left without its expected hole. The exact standard configuration is preserved in `tests/fixtures/native_offset_inward_large_cycle_nesting_failure.json`; the first full logs remain unchanged.

At OCCT d3056ef80c9668f395da40f5fd7be186cae4501f, `BOPAlgo_BuilderFace.cxx::IsInside` (842?891) rejects a candidate sharing a face edge before geometric containment. The actual offset caller chain is `BRepAlgo_Loop::WiresToFaces` ? `BRepAlgo_FaceRestrictor::Perform` ? `TopOpeBRepBuild_WireToFace::MakeFaces` ? `TopOpeBRepBuild_WireEdgeClassifier::CompareShapes`; its shared-edge check (271?275) likewise prevents a containment decision. Our `nest_cycles` only compared sampled signed areas and a boundary point, so reversal roundoff could make the positive cycle appear larger than its twin.

Native correction: retain sorted undirected graph-edge identities in each traced cycle and exclude shared-edge candidates before hole containment. Contraction of ordinary knot runs does not change these graph identities. This uses topology rather than a new area epsilon. `Preserved Inward Offset Cycle Nesting` replays the exact saved surface and four sections, requires two regions and exactly one hole, and rejects any edge reused in two wires of the same face. Build/run pending at this checkpoint.


### Self-loop integration validation checkpoint

The standalone self-intersection helper now passes all seven groups, including the rational compressed-tail regression (5.64 ms). The native splitter integration preserves both parameter occurrences, uses original-source outward interval vertex-ball bounds before collapsing excursions, keeps nondegenerate same-vertex half-edge pairs, and matches edge reuse by source parameter intervals. Source comparison used `BRepLib_MakeEdge.cxx:683?793` (closed versus degenerated, two vertex occurrences plus range) and `BOPAlgo_WireSplitter_1.cxx:155?180,438?487` (both incident occurrences and nondegenerate one-edge loops).

An initial integration rerun used a stale object: PowerShell `Copy-Item` retained the draft's 00:54:38 timestamp, earlier than the 00:56:06 object. Build16905 therefore omitted `split.cpp`. The failing `native-single-span-self-integrated.log` and `native-two-loops-self-integrated.log` describe the old binary and are retained as build-process evidence, not a new algorithmic regression. The source timestamp is refreshed; the next build must explicitly compile the file before drawing conclusions.

The separate nesting correction is actually compiled and passes its preserved fixture in70.88ms. All four inward shapes now reach a closed one-shell solid after the shared-parameter fix, but retain extra degree-two subdivisions; their strict6/12/8 assertions still fail. No topology threshold was changed.
