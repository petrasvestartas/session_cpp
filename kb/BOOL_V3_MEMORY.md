# BOOL V3 — persistent memory for the ground-up boolean rewrite

> Read this first in any session that works on the boolean engine. It is the
> standing brief: the original request, the machine constraints, the domain
> knowledge, and the v3 architecture decision. Update it when the design moves.

## 1. The standing request (from the user, 2026-08-02)

- We are developing a boolean solid B-rep algorithm like
  https://dev.opencascade.org/doc/overview/html/specification__boolean_operations.html
- Ground truth OCCT source: `/home/petras/code/code_cpp/OCCT/src` (read-only reference).
- The previous implementation (`src/brep.cpp`, `src/brep.h`, `src/brep_test.cpp`,
  plus the v2 attempt in `src/v2/`) is **extremely faulty — write it from ground
  up, state of the art, industry ready**.
- Must work on: box, cylinder, sphere, torus, cone, pyramid under **arbitrary
  random rotations** (not just axis-aligned), and their combinations:
  `serialization/boolean_steps/inmem/` (26 primitive-pair STEP triples),
  freeform `serialization/boolean_steps/chairs/`, and `serialization/step_import/`.
- **Verify operands actually intersect before/while running the algorithm.**
- **MEMORY SAFETY (critical): the machine crashed once from memory exhaustion.**
  30 GB RAM + 8 GB swap. Before every build/run: `free -h`; build with low
  parallelism (`-j2` to `-j4`, watch RSS); run sweep binaries one at a time;
  kill stale build/test processes (`ps aux | grep -E 'main_|cmake|ninja'`).
  Never launch parallel test sweeps. Keep per-process budgets in long loops.

## 2. Machine & project facts

- Working dir: `/home/petras/code/code_rust/session/session_cpp`. C++23, CMake,
  Release `-O3 -march=native`, unity build for `session_core`, PCH `src/pch.h`.
- Isolation convention (from v2): new kernel code lives in `src/v3/`, compiled as
  its own object library; v1 files are never touched by v3 work. Driver mains are
  numbered (`main_25` = v3).
- Existing substrate to REUSE (battle-tested, do not rewrite):
  `Point/Vector/Xform` (`src/point.h`), `NurbsCurve/NurbsSurface`, STEP IO
  (`src/file_step.cpp` — imports PLANE/CYL/CONE/SPHERE/TORUS/B-spline ADVANCED_FACE,
  converts quadrics to rational quarter-arc NURBS; has `detect_analytic_srf` for
  export), meshing, `BRep` container + JSON/pb serialization, recognizers
  (`fit_sphere/cylinder/cone/torus` in `src/intersection.cpp:2374+`).
- `BRep::boolean` already has a backend-registration hook
  (`BRep::register_boolean_backend`, `src/brep.cpp:8529`) — v3 plugs in there,
  same as v2 does today; v1 remains the fallback during bring-up.
- Ground-truth data:
  - `serialization/boolean_steps/inmem/` — 26 files, each 3 bodies (A red, B blue,
    result green); families: sphcyl tilts, sphcyl 20°(1,1,0), boxbox poses,
    boxcone/boxsph/conecone/cylcone/cylcyl/sphsph at rot25/rot55. Oracle-free:
    check `vol(cut)+vol(common)==vol(A)`, closedness, analytic vol(A).
  - `serialization/boolean_steps/chairs/` — chair0/chair1 (20 B-spline faces, no
    pcurves) + OCCT ground truths (`*_cut_*` 35f/46.7943, `*_common_*` 25f/33.5025,
    `*_fuse_*` 50f/127.0913) + `rot/` 10 rotated copies of chair1.
  - `serialization/step_import/truth.txt` — OCCT BRepGProp volumes per file
    (bottle 6962.4938/71f, Ball, prims box 64, cone 16.755, cyl 42.41, sph 65.45,
    torus 25.27). `validation/occt_cache.txt` — OCCT oracle cache for main_7 matrix.
  - `corpus/` — runner.py + baseline.json (132 cells, verdicts exact/open/
    closed-wrong) + invariants.py (partition, equivariance, idempotence).

## 3. Why v1/v2 fail (root causes, from kb audits — do not repeat)

1. **NURBS-only quadrics.** Every analytic surface is a rational quarter-arc
   NURBS chart. Intersections on them must be marched + re-fitted → tolerance
   inflation, non-SameParameter pcurves, seam/pole closure failures
   (cyl 42.9 vs 64.9 true; lune reads 2× truth). kb law: *analytic identity
   carried, never re-fitted* — v1/v2 violate it at construction.
2. Section stage never cuts section curves at periodic seams (`kb/V2_STATUS.md` N3);
   spurious arena vertices on cones (N1/N2); assembly wrong with byte-identical
   upstream (N4); 224× slower from display-quality triangulation in classification.
3. Tolerances in parameter space instead of model space; grow-only tolerance
   model (OCCT uses provisional growth + rollback + reducer).
4. STEP reader historically misread FACE_BOUND as inner loop — decide outer/inner
   geometrically from 2D loop areas.
5. Modules green in isolation, corpus red: gates must run on the production path.

## 4. Domain knowledge — how an industrial boolean works (OCCT BOPAlgo model)

Pipeline (BRepAlgoAPI → BOPAlgo):
1. **Prepare**: recognize geometry, bounding boxes per face/edge, BVH of boxes
   for O(n log n) pair filtering (`BOPDS_Iterator`).
2. **Interferences (PaveFiller)**: VV (coincident vertices), VE, EE (edge-edge
   3D intersection → paves), VF, EF (edge∩face → pave on edge + UV point on face),
   FF (face∩face → section curves with pcurves on BOTH faces). All intersection
   points become **paves** on edges; edges split into **pave blocks**; coincident
   blocks shared between faces become **common blocks** (one geometric entity).
3. **Face-face intersection is the heart** (`IntPatch`): dispatch on surface type
   pair — analytic via `IntAna` (plane×plane→line; plane×quadric→conic;
   quadric×quadric special cases→line/circle/ellipse; cylinder×cylinder
   perpendicular equal radii→Steinmetz ellipses; sphere×sphere→circle;
   cone/cylinder/sphere coaxial→circles; plane×torus special; torus×torus
   coaxial→circles), else **marching** (`IntWalk`): seed from triangulation
   interference (polyhedron BVH) or iso-curve grids, walk with Newton correction
   on F1(P)=0,F2(P)=0 (implicit×parametric) or closest-point conditions
   (parametric×parametric), deflection-controlled step, detect singular points
   (apex, tangency, self-intersection), join branches.
4. **Split faces** (`BOPAlgo_WireSplitter` + `BuilderFace`): in each face's UV
   domain build the arrangement of (a) original boundary pcurves split at paves,
   (b) section pcurves; chain into closed loops; discard dangling edges; decide
   loop containment; produce new faces (same surface, new wires). Periodic
   surfaces: work in the unwrapped chart; a section crossing the seam must be
   cut there; degenerate (pole) edges are typed and never marched.
5. **Classify**: each face part vs the other solid — take an interior point of
   the part (in UV, lifted to 3D), classify IN/OUT/ON with a solid classifier
   (`BRepClass3d`: ray-cast with edge/on-face treatment, or nearest-point-on-
   boundary + outward normal). Seed once, propagate across shared edges by
   topological adjacency (same classification across a non-section edge).
6. **Select + orient** per op (`BOPAlgo_BOP` op-table):
   COMMON: A parts IN B + B parts IN A; CUT: A parts OUT B (+B parts IN A reversed
   as boundary of the cavity — CUT keeps object orientation); FUSE: A OUT B +
   B OUT A (+ ON-same-domain parts once).
7. **Assemble** (`BuilderSolid`/`ShellSplitter`): glue face parts along common
   edges into shells, orient shell (material inside), classify shells for holes/
   growth, nest into solids.
8. **Tolerances**: fuzzy value adds to all comparisons; section-curve tolerance
   = max of pair tolerances, capped; vertex tolerance encloses all incident paves;
   SameParameter: pcurve and 3D curve must agree within edge tolerance measured
   in MODEL space (uv_tol = tol3d / |dS/du|).
Known truth from OCCT's own history: ~38% of its boolean bugs are silently-wrong
results — volume/closedness oracles and partition identities are mandatory, and
OCCT itself is not truth near tangency.

## 5. v3 architecture (the ground-up rewrite decision)

New module `src/v3/` (CMake target `session_v3`, driver `main_25.cpp`), plugged
into `BRep::boolean` via `register_boolean_backend`; v1 stays fallback until v3
is strictly greener on the corpus.

**The one decision that matters: analytic geometry is first-class.**

- `v3_geom`: `SrfPlane, SrfCylinder, SrfCone, SrfSphere, SrfTorus` (+`SrfNurbs`
  wrapper) as exact types: `eval(u,v)`, `normal(u,v)`, `implied F(P)=0` where it
  exists, UV of 3D point (exact inverse), axis/placement. Curves:
  `CurLine, CurCircle, CurEllipse, CurNurbs`. No quarter-arc NURBS re-fitting of
  quadrics anywhere inside the engine.
- `v3_recog`: NurbsSurface→analytic recognition at import (reuse fit_* logic);
  failure → SrfNurbs (freeform path).
- `v3_ssi`: surface×surface → section curves (3D curve + pcurve on each surface,
  exact pcurves for analytic cases). Exact cases first (all pairs among
  plane/cyl/cone/sphere that have closed forms, torus specials), general marching
  (Newton on implicit/point distance, deflection control, BVH-seeded) for
  torus-general and anything with SrfNurbs.
- `v3_topo`: Vertex/Edge(+pcurves)/Loop/Face/Shell/Solid, orientation flags,
  per-vertex/edge measured tolerances, seam edges with dual pcurves, typed
  degenerate edges.
- `v3_pave`: VV/EE/EF/FF interferences → paves → pave blocks → common blocks.
- `v3_split`: UV arrangement per face, seam-aware, loop chaining by angle sort.
- `v3_classify`: nearest-point solid classifier (exact, mesh-free) seeded once
  per connected part + topological propagation.
- `v3_build`: op-table selection, shell assembly, conversion back to `BRep`
  (quadrics re-emitted in the project's quarter-arc NURBS convention so STEP
  export/meshing/recognition keep working unchanged).
- Conversion at the boundary only: `BRep → v3::Solid` (recognize) and back.

**Acceptance gates** (all oracle-free unless noted):
G1 unit: geom eval/inverse roundtrip; SSI exact cases vs known curves.
G2 `inmem` families × 3 ops × random rotations: closed (0 naked, 0 nonmanifold),
`vol(cut)+vol(common)=vol(A)`, `vol(fuse)=vol(A)+vol(B)−vol(common)` ≤1e-6,
vol(A) analytic.
G3 step_import analytic files (occt_prim_*, bottle/Ball as freeform stretch):
volume vs truth.txt ≤ 0.5% after booleans with rotated copies.
G4 chairs pairs: closed results, volumes vs OCCT truth ≤ 1%.
G5 memory: peak RSS of any driver < 4 GB; sweeps sequential.

**Build/run discipline**: `free -h` before builds; `cmake --build build_v3 -j2`;
never run two sweep binaries concurrently; kill stale mains before new sweeps.

## 6. Progress log (append, newest last)

- 2026-08-02 G1 GREEN (main_25, 1063/1063): `v3_geom` analytic surfaces/curves
  (eval/inverse/implicit/normal ~1e-15) + residual-gated recognition of all
  primitive NURBS charts (box/cyl/sphere/cone/torus/pyramid) under random rigid
  motions. Build: `build_v3/` (configure with
  `-DFETCHCONTENT_SOURCE_DIR_PROTOBUF=$PWD/build/_deps/protobuf-src
  -DFETCHCONTENT_SOURCE_DIR_ABSL=$PWD/build/_deps/absl-src` to stay offline).
  Hard-won facts:
  - `NurbsSurface::evaluate` returns `[S, Sv, Svv, Su, Suv, Suu]` (d[1]=Sv,
    d[2]=Su); `normal_at` = normalize(**Su × Sv**) despite misleading locals.
  - Recognition ring averages need samples ON the quarter-arc knots → grid N=5
    (fractions 0/.25/.5/.75/1), 4-point ring average.
  - Degenerate rows (poles/apex) report unit garbage normals via `normal_at`;
    detect degeneracy by derivative cross magnitude instead.
  - `Xform::axis_rotation` does NOT normalize its axis — non-unit axis = shear.
  - Jacobi eigensolver angle: tan(2θ) = 2a_pq/(a_pp − a_qq) (sign-flipped
    denominator silently breaks every rotated fit).
  - Torus recognition: axis from ring-center PCA (normal covariance is
    near-isotropic); torus Kåsa circle fit RHS is +Σrow·b (no negation).
- Next: v3_topo (topology + BRep conversion), then the boolean pipeline.

- 2026-08-02 boolean pipeline ONLINE (main_25 G3): v3_bool driver + v3_split
  (paves + UV arrangement) work end-to-end. GREEN: box_cyl (5.99/2.01/12.02),
  sph_cyl (11.39/2.75/17.42), all solid, partition identities ~1-2%.
  Architecture that finally works:
  - ssi() once per face PAIR -> section chains with UVs on BOTH surfaces
    (shared section edges, SEC_BASE id space; exact Cur for analytic cases,
    POLY polyline for walked, never re-fitted).
  - uv_in_face is PERIOD-AWARE (wraps queries into the face band) — this
    killed 90% of the re-anchor gymnastics; runs on RAW chains.
  - prepare_secpc per face: whole-period shift into band; closed sections
    cut at band-cut into junction arcs (rotate at cut + continuity unwrap +
    band-center shift — every step needed, order matters).
  - attach: section endpoints -> paves on boundary edges; te from
    Cur::project of the endpoint 3D point (NOT pt<->UV lerp — that is
    frame-dependent and wrong); UV distance is the hard guard, 3D gate is
    loose (max(tol*100, 0.01), OCCT vertex-tolerance enlargement).
  - split_edges_at_paves dedupes by 3D distance (attach vs EF pierce
    produce ~1e-3-apart twins of the same physical point).
  - UV arrangement: nodes merged by CHART POSITION only (never by vertex id
    — a 3D vertex at u and u+2pi is TWO nodes or seam trims collapse to
    diagonals); half-edge next = CCW-PREDECESSOR of the twin at each node;
    angle_of for a BACKWARD half-edge is its reversed start direction
    (pc[size-2] - pc.back()), NOT the forward end direction (root bug that
    produced phantom/mixed cycles for weeks).
  - keep test: interior point on the LEFT of the walk, always.
  - Face assembly by containment tree: parent = smallest-area kept cycle
    containing the test point; own(C) = |C| - sum(|opposite-sign children|);
    faces = own != 0; holes = opposite-sign children. This correctly yields
    zones from horizontal sections AND islands+holes from closed sections.
  - classify_part: interior point (grid+uv_in_face) -> classify_point
    (closest-boundary + outward normal, dihedral fallback for edges).
  - Section-circle exactness for junctions: sample_exact closes with
    continuous unwrap; junction arcs start/end exactly at band cuts.
  - select table: FUSE = A-OUT + B-OUT (+A-ON); COMMON = A-IN + B-IN
    (+A-ON); CUT = A-not-IN + B-IN-reversed (reverse_face flips loops+rev).
  - Degenerate (pole) edges: zero-length point curves on output (is_solid
    skips by extent); singular trims (ty=3) are authoritative degenerate
    markers; loop alignment via align_loop_charts (4 phases, seam-pair
    bands exactly one period apart).
  REMAINING: box_sph over/under-count, box_box_rot partition, cone_cyl
  same-domain, box_torus no sections (plane-parallel-torus walker),
  pyr_box cut empty, box_cyl_rot* over-count.

- 2026-08-02 G2 GREEN (main_25, 1117/1117): `v3_topo` + `v3_ssi`(exact cases) +
  `v3_walk`(marcher, untested). BRep<->v3 roundtrip exact for all 6 primitives
  (vol exact, is_solid, random rotations). Key contracts:
  - to_brep emits quarter-arc NURBS charts with set_domain so chart UV == v3 UV
    affinely (u=angle rad); periodic charts are ANCHORED at the used-range min
    and the frame is pre-rotated by that anchor — pcurves transfer verbatim.
    NEVER emit a reversed domain (set_domain(t1,t0)) — downstream mesh/volume
    breaks. Degenerate edges: emit zero-length point curves (is_solid skips by
    extent), never curve_3d_index=-1. Singular trims (ty=3) are authoritative
    degenerate markers on import.
  - Periodic loop alignment (align_loop_charts, 4 phases): A snap junction
    samples at singular vertices to the coedge's own generator direction
    (uv_of is arbitrary at poles/apex; handle each junction side independently,
    never skip when the neighbor is degenerate); B whole-period shifts to
    connect; C seam-pair bands: an edge appearing twice with opposite fwd must
    sit EXACTLY one period apart in its seam coordinate (whole-period shifts
    only); D degenerate coedges rebuilt as straight UV spans between neighbors.
  - Curve conic fit: eigenvalues/K share sign (l*K>0), major radius from
    max(K/l), conic RHS = 1 normalization is sign-unstable — gate on signs.
  - v3 signed_volume = divergence over centroid-rule UV tessellation;
    grid 40 ≈ 1% for sphere/torus (coarse smoke check only).

## 2026-08-02 session: tangency + containment fixes (box_torus GREEN)

Machine: 30 GB RAM — check `free -h` before builds, build `-j2`, kill stale main_*/cmake.

box_torus = box(4,4,1) x torus(1.5,0.5): torus is INSIDE the box, touching top/bottom
faces along tangent ridge circles (z=+-0.5) and side walls at 4 tangent points (rho=2).
Three root fixes in v3_bool.cpp, all generally applicable:
1. **bbox gate was pruning tangent pairs**: face_bbox (tessellate_face n=6 + 9 edge
   samples) underestimated the torus z-extent (0.49881 vs 0.5, gap 1.2e-3 > tol*10).
   Fix: deflection-aware pad = tol*10 + 0.01*(diagA+diagB). Sampling never converges
   to exact extrema; pruning pads must exceed max tessellation deflection. Generous
   pad is safe (SSI returns empty for disjoint pairs), tight pad loses contacts.
2. **Tangent sections are multiplicity-2**: the implicit SSI solver returns TWO
   IDENTICAL circles for plane x torus tangency. Fix: dedupe in pass 3 -- skip a run
   if 32 sample points all lie within tol*100 of an already-registered section from
   the same face pair.
3. **Single-vertex containment fails on tangency**: first torus vertex (2,0,0) is
   exactly ON the box wall -> neither IN nor OUT -> fell to "disjoint" (wrong).
   Fix: no-section containment classifies MANY samples (verts + edge midpoints +
   tessellation centroids, n=4); X inside Y iff no sample OUT and >=1 strictly IN.
   Also sets *intersected = ca||cb (volumetric intersection without boundary curves).

Result: box_torus cut 8.5978(1) com 7.4022(1) fus 16.0000(1) -- EXACT. Pipeline
itself handled tangent island circles fine (torus split along ridge = 1 cylindrical
part; box top island disk + remainder both classify OUT of torus).

Known separate issue: v3 Solid::signed_volume(10) is tessellation-coarse for curved
faces (torus 6.478 vs exact 7.402, ~12%). Only used for orient sign + debug prints;
harness checks use BRep::volume (exact). Revisit if precision budget demands.

## 2026-08-02 (later): USER REQUIREMENTS (hard, from user) + cone_cyl diagnosis state

### New hard requirements from the user
1. **Exact curves, not simplified polylines**: intersection boundaries must be actual
   curves (line/circle/analytic or proper NURBS fits), not crude polylines. The
   pipeline already carries `has_exact` section curves (Cur) through to output edges;
   marched/walked sections must FIT curves (NURBS) rather than emit raw polylines.
   Audit `to_brep` output: section edges must carry exact/fitted curve geometry.
2. **Geometry-type-agnostic**: must work on ANY input solids, not just the 6 test
   primitives — freeform NURBS breps (chairs, step_import, inmem). Never assume
   analytic surface kinds; the walker/marching SSI path must be production-grade.

### CRITICAL: PtCls enum order is {IN=0, OUT=1, ON=2}
I read debug output INVERTED in earlier sessions (thought 1=IN). Correct reading of
cone_cyl `[cls]` lines (cone(1.5,3.0) x cyl(0.8,3.2), coplanar bases z=0, cyl pokes
through cone side at z=1.4):
- A0 cone UPPER band (z>1.4, inside cyl): cls=1=OUT  **WRONG (should be IN)**
- A1 cone LOWER band (outside cyl):       cls=0=IN   **WRONG (should be OUT)**
- A2 base annulus (0.8<r<1.5): OUT correct; A3 base inner disk: ON correct
- B parts (cyl side bands/base/top) ALL CORRECT.
=> Both cone-SIDE bands are IN/OUT-swapped; planes and cylinder faces classify fine.

### Diagnosis so far (next step identified)
- Standalone classify_point on the oriented cylinder is CORRECT for all probe
  points incl. the cone-band test point (0.0647,-0.047,2.84) -> IN (V3PROBE3 env
  in main_25.cpp replicates: cone+cyl, orient both, classify+closest_on_solid dump).
- The misclassification exists ONLY inside the boolean pipeline by classify time
  (it predates the dual-probe change: single-point classify showed the same swap).
  => Standing hypothesis: B (or A) topology is corrupted by the pipeline before
  classification (split_edges_at_paves remapping coedge.edge, or split_face's
  coedge appends invalidating uv_in_face for the ORIGINAL faces), so
  closest_on_solid/uv_in_face misbehave for the cone-side test points.
- NEXT STEP: in the probe, replay the pipeline stages on B one by one (or add a
  classify hook right at the classification site in v3_bool.cpp) and find which
  stage breaks classify_point(B, (0.0647,-0.047,2.84)).

### Fixes landed this session (all verified, don't regress)
1. split_face emission rule REWRITTEN (v3_split.cpp): zone-outer walks are CCW
   (positive area, interior-left rule) => emit ONLY positive kept cycles as faces;
   negative cycles are hole boundaries consumed by their nearest POSITIVE ancestor
   (walk up parents). The old `own!=0` rule emitted the CW copy of an island
   section as a duplicate face (cone base disk twice). Also fixed pyr_box (all
   solid) and box_box_rot common.
2. classify_part dual-probe (v3_bool.cpp): probes p±n*eps (eps=tol*100) along the
   part outward normal; both-IN=>IN, both-OUT=>OUT, differ=>ON. Fixes grazing/
   coplanar test points where single-point ray casting is degenerate.
3. closest_on_face/classify_point (v3_classify.{h,cpp}): ClosestInfo.on_boundary
   (+ winning edge) now reported; the face half-space test applies ONLY to
   interior winners; boundary winners route to the dihedral/parity path. (Landed,
   compiled; standalone probes correct. NOTE: turned out NOT to be the cone_cyl
   root cause — the swap predates it — but the fix is principled, keep it.)

### Battery state now
GREEN: box_cyl, sph_cyl, box_torus (exact), pyr_box (all solid).
RED: cone_cyl (A-side band swap, above), box_box_rot (fuse not solid), box_sph
(T-junction welding), box_cyl_rot* (partition over-count; rot1 false
"not intersecting"), G3 totals otherwise fine; G1+G2 all green (1143).

## 2026-08-02 (session 3): segfault fixed, battery runnable again

Battery died with SIGSEGV in closest_on_face at G3 start. TWO bugs, both fixed:
1. **use-after-free in split_edges_at_paves** (v3_split.cpp): `Edge& e` held across
   `S.edges.push_back(ne)` in the piece loop; `ne.c = e.c` for piece p>=2 read
   through a dangling reference after reallocation -> garbage Cur in piece edges.
   Fix: copy `Cur ec = e.c; double etol = e.tol;` BEFORE the push loop.
2. **OOB face index in classify_by_adjacent** (v3_classify.cpp): split_face's
   add_loop appends PART coedges to the shared S.coedges pool with
   `ce.face = S.faces.size() + out.size()` (a part-space index, NOT valid in
   S.faces). classify_by_adjacent resolved `s.faces[ce.face]` unguarded ->
   garbage Face& -> segfault. Exposed by the new boundary->dihedral routing.
   Fix: skip coedges with ce.face out of range (original faces' coedges for the
   same edge carry the same surface geometry).
   LESSON: any code resolving ce.face on a LIVE solid must bounds-check; part
   coedges are only valid after the output-build remap (v3_bool.cpp:103
   `nce.face += foff`).
Added env-gated validator: V3COFDBG=1 in closest_on_face bounds-checks
srf/cei/ce.edge/ce.face/e.v0 and aborts with context (keep until battery green).
Battery state (matches RED list below): GREEN box_cyl, sph_cyl, box_torus,
pyr_box. RED box_box_rot (fuse), box_sph (all), cone_cyl (all), box_cyl_rot0/2/3
(partition over-count: cut reads vol(A)+vol(common)), box_cyl_rot1 (false
"not intersecting"). G1+G2: 1178 passed / 25 failed (25 = the G3 FAILs).

## 2026-08-02 (session 3, part 2): battery 1198/1203 -- only cone_cyl + box_sph-fuse red

GREEN now: box_cyl, box_box_rot, sph_cyl, box_sph(solid), box_torus, pyr_box,
box_cyl_rot0-3 (random rotations!). RED: cone_cyl (all ops not solid -- needs
coincident-edge unification), box_sph fuse identity (-4.85, caps missing).
Root causes + fixes landed (all verified by full battery after each step):
1. v3_ssi.cpp plane_cylinder oblique: rmaj = r/|an| (was r/sin_th -- sin/cos
   swap, only near-correct at 45deg tilt). Killed rot1 "not intersecting".
2. v3_topo.cpp uv_in_face wrap guard: TWO_PI + 1e-12 (full-period band is
   2pi +/- 1 ULP; guard excluded it -> section runs truncated at the seam).
   MUST stay ULP-scale: 1e-9 broke box_torus (seam-pair bands span 2pi+slop).
3. Tangent (osculating) sections must NOT split faces: pass-3 skip when
   |nA.nB| > 1-1e-6 at ALL of 8 samples (box_torus ridge circles). Previously
   box_torus worked only by accidental run truncation; now principled.
4. prepare(): sequential re-unwrap of every chain first (refine_boundary
   leaves internal 2pi jumps from canonical uv_of -> garbage center rule);
   phantom wrap-segment append removed for explicitly-closed chains (cone_cyl
   retrace spur); tail arc image continuity-based (descending chains).
5. merge_sections() NEW (before prepare): joins open section pieces end-to-end
   (3D gate, whole-period shifts, boundary-junction guard at tol*100+0.005 --
   diag-scale gates block legit seam-neighborhood junctions), marks physically
   closed chains CLOSED (exact closure snap mod period) so prepare's band-cut
   machinery handles seam-crossing loops. Carries NEW SecPC.pc_edge
   (per-sample edge id, v3_split.h) through merge/rotate/cut; post-pass
   re-splits junction arcs at edge-id transitions (else the other operand's
   arcs see one physical curve under two ids -> naked).
6. classify_part: probe the MAX-CLEARANCE interior grid point (was first-hit:
   boundary-straddling probes flipped box_box_rot corner triangles to false
   ON). PartCls exposes cp/cm; keep table: CUT keeps A-ON iff cm==OUT,
   COMMON iff cm==IN (material-side disambiguation), FUSE unchanged.
7. Output stage: cyclically-adjacent same-edge coedge merge + coedge
   compaction; T-junction split of >2-coedge edges (split every coedge at
   sibling 3D endpoints via segment projection + insertion, regroup by welded
   endpoint pairs, compact edges) -- a section arc crossing a face seam is
   cut there on that face but used whole on the other: 3 trims, false nm.
8. snap_to_pierces: anchor period unwrap at the CURRENT endpoint (not the
   adjacent sample) -- preserves merge chart alignment at pierce junctions.
9. REMOVED align_junctions (superseded by merge_sections; its center-distance
   heuristic tore merged chains apart, even oscillated).
LESSON: debug order matters -- check [presplit] piece continuity first, then
arrangement cycles, then [out] face edge-uses; V3MRGDBG/V3OUTDBG added.

## 2026-08-02 (session 3 finale): BATTERY FULLY GREEN -- 1203/1203

main_25: ALL 12 cases x 3 ops solid, all partition/fuse identities pass.
Exact-volume cases: box_cyl (5.99/2.01/12.02), box_box_rot (4.66/3.34/8.04),
sph_cyl (11.39/2.75/17.42), cone_cyl (3.18/3.89/9.61 -- all three exact:
com = pi*0.64*1.4 + cone-cap integral = 3.887, fuse = A+B-com), box_torus
(8.60/7.40/16.00), pyr_box (1.69/2.31/15.19). box_sph (0.07/7.93/14.17 vs
Monte Carlo 0.10/7.90/14.24, within 0.5%). rot0-3 all solid, partition exact.

cone_cyl root causes (3, all fixed): (a) phantom wrap-segment spur in prepare
for explicitly-closed chains; (b) coincident-edge unification missing
(common blocks): full-circle coincident R.edges now merged geometrically
(conservative: CIRCLE kind + center/radius/axis gates + full period only);
(c) ON-rule op table: CUT keeps A-ON iff cm==OUT, COMMON iff cm==IN.
ALSO: cone apex probe classified ON by garbage singular normal ->
classify_by_adjacent nudges UV 2% toward band center when |du x dv| < 1e-8.

box_sph fuse odyssey (the last red cell): geometry was ALWAYS right (mesh
volume 13.93 vs truth 14.24). Chain of eliminations: exact rational arc
emission for partial circles (Plug A of the exact-curve requirement: split
span <= 2pi/3, Primitives::arc per segment, NurbsCurve::join; full ellipse
emission added) + to_brep canonicalizes periodic-u charts into [-pi,pi)
(whole-period shift of anchor AND pcurves) -- both principled, KEEP, but
neither was the cause. TRUE CAUSE (v1 bug): brep_massprops loop_green did
`if (budget_left <= 0) { capped = true; break; }` -- after the evaluation
budget blew (symmetric-moment trap on the full-sphere frame loop, moments
~0 -> relative convergence never fires), every subsequent loop got exactly
ONE trim integrated then broke, silently dropping the rest. Fix: break
removed; adaptive_gk always evaluates the full initial panel list, so later
trims stay initial-panel accurate. **THIS IS A V1 FILE EDIT
(src/brep_massprops.cpp) -- a deliberate exception to the v1-isolation
convention: the bug silently corrupts any budget-capped face. Corpus
verification (build/main_7) run separately.**

Debug tooling added (env-gated, keep): V3COFDBG (topology bounds validator),
V3MRGDBG (merge joins), V3OUTDBG (result face edge-uses with pc ranges),
V3DUMPB/V3MESHVOL (main_25 result dumps).
Convention discovery: project BRep stores trim pcurves in EDGE direction +
reversed flag; v3 coedges carry pc in TRAVERSAL direction internally;
to_brep emits stored pc verbatim + rev=!ce.fwd (massprops' chain resolver
is convention-free and copes; full-domain faces take the immune path).

### Latent issues (noticed in passing, not yet fixed, no battery failure)
- v3_split.cpp:132-135 split_edges_at_paves: `npieces = cur + 1` mutation inside
  the per-coedge loop can corrupt piece_edge mapping for the seam-mate coedge.
- Walker (v3_walk) is still not exercised by the battery; marched sections
  emit raw polylines (Plug B fit ladder from the audit is NOT implemented:
  recognize_curve core in v3_geom.cpp:815+ and NurbsCurve::create_fitted are
  the building blocks). Required for chairs/step_import (G3/G4) and the
  geometry-type-agnostic requirement.
- Cur::NURBS edges are re-sampled to degree-1 polylines in emit_curve_3d;
  pcurve trims are always degree-1 polylines (SameParameter looseness).
- signed_volume(10) is tessellation-coarse (debug prints only).

### Corpus check of the v1 massprops edit (2026-08-02 late)
Full corpus run (150 cells, build/main_7 rebuilt): 12 matrix cells flipped
exact -> closed-wrong with rel ~2e-4 (box_sph x3, box_tor x2, cyl_cyl x3,
sph_sph x3, tor_tor x2). The regression tickets show those cells use ~60k
evals -- NOWHERE near the 60M budget -- so the budget-exhaustion path my edit
touches is never exercised there; the flips must come from the pre-existing
uncommitted v1 working-tree state (src/brep.cpp et al., Aug 2 01:07) vs the
2026-07-25 baseline kernel, NOT from the massprops edit. A/B run (same tree
minus the edit) to confirm.
CONFIRMED by A/B (same tree minus the edit, matrix cells rerun): the same 12
cells regress identically (exact 32 / closed-wrong 13 both ways). The flips
are pre-existing working-tree drift, NOT the massprops edit. The edit is
corpus-neutral on non-capped cells and fixes the capped-truncation bug.

# ============================================================================
# NEXT SESSION — START HERE (written 2026-08-02, battery 1203/1203 green)
# ============================================================================

## 2026-08-03 (session 4b): seam-crossing walked loops FIXED — 1239/1239

New green regression cases (main_25 G3): `sph_tor_seam` (sphere(0.8)@
(1.5,0,0.35) x torus(1.5,0.5) rot 1.1 about z — walked loop crosses the
torus v-seam) and `tor_wall` (box(2.6,4.4,3) walls x=+-1.3 cut axis-parallel
plane-torus ovals wrapping the tube in v, crossing the v-seam twice each).
Fix chain (each step battery-verified):
1. prepare_dir: junction arcs are shifted to lie INSIDE the face band (arcs
   are cut at band lines, so a whole-period shift per arc is exact). An arc
   left outside the band lets the flat-chart traversal walk across the frame
   boundary -> peninsula zones whose holes mis-emit when sibling zones are
   selected apart.
2. Closed-cut splits fire only at GENUINE crossings (neighbors straddle the
   line): a walker seed can sit exactly ON an iso line and the chain then
   touches the cut line without crossing — cutting there spawns spur arcs.
3. T-junction split (output): spts = 3D endpoints of ALL coedges of the
   over-used edge (incl. full-range ones — a closed loop's arbitrary start
   is a real junction whenever a sibling is cut there), deduped, then
   REFINED onto the shared 3D curve (unclamped project; chart-lerped
   junction UVs sit ~1e-2 off the curve and every cut gate misses them).
   Cut a coedge unless the spt is within gate of ITS OWN endpoints (3D) or
   projects at the polyline's global ends. Regroup by endpoint welds + 3D
   PATH MIDPOINT (arcs above/below a lens share the endpoint pair). Piece
   t-ranges on closed curves: chained unwrap through the midpoint (a blind
   t0/t1 swap emits the complementary arc when the piece wraps the period).
4. Point-sliver coedges (zero 3D extent) dropped at output.
5. T-junction gate = max(100*tol, 0.01) (chart-lerp junction error scale).
Chairs agent bug 2: project_nurbs_seeded (v3_walk) now returns Newton
CONVERGENCE (not 1e-6 proximity); callers measure/gate the distance
themselves — NURBS x NURBS seeding fires.
Note: the from_brep winding flip must stay restricted to FULL-PERIOD edges
(the sphere's seam meridian is an open edge; flipping it breaks the chart).

## 2026-08-03 (session 4): walker ONLINE + Plug B landed, battery 1227/1227

main_25: `1227 passed, 0 failed` (1203 old + 24 new checks from 4 walker
cases). Verify: `flock /tmp/v3build.lock cmake --build build_v3 -j2 --target
main_25 && (cd build_v3 && ./main_25)`.

### Walker (v3_walk.cpp) — was completely broken, now production-usable
Root bugs fixed (the walker had never run before):
- **Predictor solved the corrector equation** `J d = T*h` (min-norm), which
  splits the step into OPPOSITE motions of the two surface points — the walk
  crawled in place at hmin to max_branch_pts. Fixed: null-space step = per
  surface 2-var least squares of [du dv] vs T*h (T = nA x nB lies in both
  tangent planes).
- **Backward walks always rejected**: the angle test dotted the raw new
  tangent against tan_prev (which carries `dir`) -> ang=pi at every step for
  dir=-1. Dot with `tan_new*dir` instead.
- **Deflection probe failure treated as defl=0** (accepted anything): now
  defl = HUGE when the midpoint corrector fails.
- **Closure**: walk_dir stops when the step chord passes within r_close of
  the branch start (chord-segment distance; r_close = max(20*tol, min(0.25*h,
  1000*tol)) — must cover per-lap drift ~1e-5; without it loops re-walk to
  the 20k point cap). On closure the backward walk is SKIPPED (it would
  double-cover the loop).
- **in_rect clamped periodic params**: walks stopped dead at canonical
  0/2pi seam lines, fragmenting every seam-crossing branch. Periodic
  directions whose rect spans the full period no longer stop the walk.
- **NURBS-B iso seeding never fired** (chairs agent report): raw iso samples
  sit ~spacing/2 (~1e-2) from crossings, gate was 2e-5. Now: local minima of
  the projected distance below 4x the local sample spacing get polished by
  the 4-var corrector.
- Seed skip radius was 4e-6 << walk spacing -> every seed re-walked loops
  (duplicates). Now seed_skip = max(8*tol, 0.005*diag); PtHash cell sized to
  cover it.
- Zero-length spur branches (< 20*tol arc length) dropped.

### Plug B (fit ladder, v3_bool.cpp pass 3, fit_section())
Marched runs: (i) analytic recognition from run points (NEW
`recognize_curve_pts(pts,n,tol,out)` in v3_geom — the recognize_curve core
refactored point-based; circle/ellipse frame orientation aligned to run
order by flipping y,z when the back-projection decreases); (ii) LSQ
create_fitted ncv 8->128 until verified; (iii) polyline fallback.
Verification gate: every run point projects within fit_tol=max(10*tol,1e-6)
AND dense curve samples stay within fit_tol of both surfaces via
`srf_residual = |F|/|gradF|` (FIRST-ORDER distance — raw |F| is
quadratic/quartic and not length-comparable; the literal |F|<=fit_tol gate
from the plan would fail everything on torus). Measured dev -> Edge.tol.
Closed runs fit periodic (is_periodic=true). One tor_cyl loop needs ncv=128
(dev 2.2e-7); 64 was 2% short of the gate.

### emit_curve_3d (v3_topo.cpp)
- Cur::NURBS: passthrough via `nrb.trim(t0,t1)` (no polyline re-sampling).
- Partial ELLIPSE arcs: exact rational quadratic segments (span split
  <= 2pi/3; shoulder = chord_mid + (P_mid - chord_mid)/w, w = cos(dspan/2) —
  exact for ellipses by affine invariance of the circular-arc formula),
  NurbsCurve::join. Full ellipses already emitted via Primitives::ellipse.

### Integration bugs fixed (each was a battery/walker blocker)
- from_brep: singular trims (ty=3) with edge_index == -1 (imported cone
  apex) crashed v3_classify — synthesize a degenerate point edge + vertex
  (3D point from the 2D trim through the source surface).
- from_brep circle/ellipse WINDING: recognize_curve's conic plane normal
  comes from a PCA eigenvector (arbitrary sign) -> anti-parallel frames
  reverse the parametrization and flip every pcurve slope built from the
  edge (rotated-cylinder side bands spanned two periods -> uv_in_face
  broken). Fix: compare the source NURBS tangent at the edge start with
  c.d1(t0); if opposite, flip the frame (negate y,z). **FULL-PERIOD edges
  ONLY** (v0==v1): flipping open edges (sphere meridian seam) breaks the
  face chart — open-edge unwrap handles them.
- prepare() is now per-coordinate (prepare_dir with member pointers; runs
  for u AND v — torus faces have both seams as boundary edges). Cut rule
  changed: cut a closed chain at a band line only when it genuinely SPANS
  the line (samples strictly on both sides); a chain merely touching it
  (closure point on the seam, sph_cyl latitude circle) must stay whole for
  the self-loop cycle. Zero-extent phantom junction arcs (duplicate samples
  on the cut line) are filtered.
- split_edges_at_paves: npieces mutation bug FIXED (local nce_pieces).
- snap_endpoints: (1) pave-t map lookup now falls back to nearest-3D (the
  3D pave dedupe keeps a twin t -> exact-double miss -> unsnapped
  endpoint); (2) the UV snap lerps over ALL split pieces of the parent
  edge (only the first piece carries the parent id) — unsnapped endpoints
  left unmerged arrangement nodes on periodic seams.
- Containment tree (v3_split): a zone's CW copy (fp-equal area) could be
  chosen as its parent -> holes orphaned / zones doubled. Skip opposite-sign
  near-tie candidates as containers; prefer positive cycles on near-ties.
- intersect_all: per-piece UV bbox rejection (walked polylines are ~2-5k
  samples; brute force was 1e8+ seg tests/round). seg_x: zero-length
  segment guard (chairs agent's "infinite loop" — actually the quadratic
  blowup + these).
- pass-3 dedupe: point-to-POLYLINE distance (point-to-point misses marched
  duplicates by sample phasing), gate 2000*tol (duplicate walks drift up to
  the closure radius). SIGFPE guard: skip dedupe for 1-point runs.

### Battery walker cases (main_25 G3, all green)
tor_box (small tilted box clipping one plane-torus loop off the tube top),
tor_cyl (cyl axis along x at z=-0.45 through the tube), sph_tor (sphere
r=0.6 at (1.5,0,0.55)), cone_sph_nc (cone spun 0.9 about its axis, sphere
r=0.7 at (0.9,0,1.2)). Poses are chosen so walked loops stay INSIDE one
periodic chart (seam-interior) — see known limitation below.

### Bug 3 (main_26 analytic pairs) — DIAGNOSIS: v1 massprops, not v3
box_x_box_p1: v3 cut result mesh volume 22.44 == MC oracle 22.20
(NEW V3MC=<case> env probe in main_25: replicates main_26 poses +
contains_point Monte Carlo), but BRep::volume() (v1 brep_massprops) reads
19.01. Zones/classification verified correct (zone areas sum exactly per
face). cone_x_box_p1 "hang" is also v1 loop_green quadrature grinding on a
v3 result, not a v3 hang. -> main_26 reds need a v1 brep_massprops
investigation on v3-style outputs (long polyline pcurve trims? hole-loop
Green forms on plane charts?). OUT OF v3 SCOPE — not fixed.
main_26 no longer crashes (stale-binary SIGSEGV was the from_brep -1 edge;
SIGFPE was the 1-point-run dedupe).

### seam-crossing walked loops — FIXED in session 4b (below)

### New debug env flags
V3WALKDBG (branch stats), V3WALKDBG2 (step trace), V3FITDBG (fit ladder),
V3PREPDBG (prepare pass decisions + raw/prepared SecPC dump), V3XDBG
(intersect_all rounds), V3FLIPDBG/V3NOFLIP (winding flip), V3FBDBG
(from_brep trims), V3MC=<main_26 case name> + V3MCMESH (MC volume oracle,
in main_25).

## 2026-08-04 (session 5, main_26 agent): v1 massprops cavity-depth fix — second sanctioned v1 edit

main_26 (G3 step_import driver) Bug 3 root-caused and FIXED in v1
`src/brep_massprops.cpp` (second v3-motivated v1 edit, after the :508
budget-cap fix):
- **Root cause** (NOT loop_green quadrature, NOT hole-loop Green forms —
  per-face quadrature was exact, mesh==mp to 1e-6 on all 18 faces): PASS 3/4
  cavity detection decided nesting by BBOX CONTAINMENT. A multi-lump boolean
  result (box_x_box_p1 cut = THREE disjoint lumps, 0.058+1.657+20.726 =
  22.4415 exact polyhedral) has the small lumps' bboxes inside the big
  lump's bbox -> they were read as cavities and SUBTRACTED (19.0108). Bbox
  nesting is necessary but NOT sufficient for a cavity.
- **Fix**: bbox nesting stays as the cheap filter; every candidate pair is
  verified geometrically by INTEGRATING the winding number (solid-angle
  kernel n.(S-p)/|S-p|^3) over shell b's trimmed faces with the shell's
  consistent orientation (sgn*traversal), reusing the PASS-1 Green machinery
  (FaceIntegrator gained a wind_p mode; resolved loops are saved per face as
  saved_loops, hole-heal sense applied to them). |w|>0.5 on a majority of up
  to 5 nudged boundary samples of shell a => cavity. Unverifiable (capped
  quadrature = sample essentially on b's surface, tangent contact) falls
  back to the historical bbox verdict. Single-shell breps pay nothing.
- **Rejected approaches (do not retry)**: (1) face_meshes()+contains_point
  winding — mesh_q's CDT grinds >600 s on boolean-output polyline pcurves
  (corpus matrix_rot coneRx_cyl/common: 12 s -> timeout; gdb:
  Delaunay2D::insert_constraint). (2) RemeshCDT UV soup with capped sampling
  — Clipper2 returns EMPTY on some v3 boolean loops (box_x_box_p1 face 12),
  and the bbox fallback then flipped the wrong shell.
- **Verification**: main_26 box_x_box_p1 cut 22.4415 == mesh oracle (was
  19.0108); main_25 battery 1239/1239; corpus A/B (150 cells, build/main_7,
  runner.py run --jobs 1): 0 verdict/volume flips, wall 726.5 -> 735.8 s
  (+1.3%), no cell slowdown. Ledgers: corpus/ledger/
  ledger_20260804-062510.json (A, without) vs ledger_20260804-073823.json
  (B2, with).
- main_26 box_x_box_p3 residual (partition -9.40, results not solid) is
  v3-side GEOMETRY, not massprops: mesh oracle agrees with v1 volume exactly
  (12.2520/42.3470/76.2520) while MC says com ~50. For the v3 agent.

## Where we are (2026-08-02 state, superseded by session-4 entry above)
main_25 (build_v3) is FULLY GREEN: G1 (geom/recognition), G2 (roundtrip +
classifier), G3 primitive battery — 12 pairs x 3 ops, all solid, partition +
fuse identities pass. Exact: box_cyl, box_box_rot, sph_cyl, cone_cyl,
box_torus, pyr_box. <=0.5% of Monte Carlo truth: box_sph, box_cyl_rot0-3.
Peak RSS 15 MB. Verify first: `cmake --build build_v3 -j2 --target main_25 &&
(cd build_v3 && ./main_25)` -> expect "1203 passed, 0 failed", exit 0.
v3 is NOT yet plugged into BRep::boolean as the backend (v1 still serves);
that switch is the last step, taken only when G3(step_import)/G4(chairs) pass.

## Remaining work, in priority order
1. **Walker production-grade + Plug B (NURBS fit of marched sections)** — the
   hard user requirement. v3_walk.cpp exists but is UNTESTED (battery never
   exercises it: all battery sections are analytic-exact). Tasks:
   a. Fit stage in v3_bool.cpp pass 3, replacing `se.c = cur_poly(poly, false)`
      (~line 290): ladder = (i) analytic recognition of run 3D points using the
      recognize_curve core (src/v3/v3_geom.cpp:815-961: line -> planar conic ->
      circle/ellipse, residual-gated), (ii) NurbsCurve::create_fitted
      (src/nurbscurve.cpp:353, LSQ, grow ncv 8->64 until max dev <= fit_tol),
      (iii) fallback cur_poly. fit_tol = max(tol*10, 1e-6); store measured dev
      in Edge.tol; verify every run point within fit_tol AND |F_a|,|F_b| of
      dense samples <= fit_tol, else fallback.
   b. Test the walker: torus x box (general plane-torus), torus x cylinder,
      sphere x torus, cone x sphere non-coaxial, then chairs.
   c. emit_curve_3d: Cur::NURBS passthrough (trim(t0,t1)) instead of polyline
      re-sampling (v3_topo.cpp:697+); partial ELLIPSE arcs (currently polyline;
      rot* sections are ellipses!).
2. **G3 gate: step_import.** occt_prim_* files + truth.txt volumes. Boolean
   each primitive against rotated copies of itself/others; volume vs truth
   <= 0.5%. Bottle/Ball as freeform stretch (needs #1).
3. **G4 gate: chairs.** serialization/boolean_steps/chairs/: chair0 x chair1
   cut 46.7943/35f, common 33.5025/25f, fuse 127.0913/50f (OCCT truth), plus
   rot/ 10 rotated chair1 copies. Needs the walker (#1) — chairs are 20
   B-spline faces, no pcurves.
4. **Backend switch.** BRep::register_boolean_backend (src/brep.cpp:8529):
   register v3 (same as v2 does), keep v1 fallback; run corpus runner
   (corpus/runner.py run --jobs 1) to compare verdicts.
5. Cleanup: `git add src/v3 main_25.cpp` etc. (untracked so far); decide
   whether main_25 debug env blocks (V3DUMPB/V3MESHVOL) stay.

## Known latent issues (fix when touched)
- v3_split.cpp:132-135: npieces mutation in split_edges_at_paves per-coedge
  loop can corrupt seam-mate piece_edge mapping.
- signed_volume(10) tessellation-coarse (debug only).
- pcurve trims always degree-1 polylines (SameParameter looseness on export).
- Walker never sets SecPoint.t (fine today; revisit if fits need it).
- Corpus has 12 pre-existing matrix regressions (exact->closed-wrong ~2e-4)
  from the Aug 2 01:07 v1 working-tree state — NOT from this work (A/B proven).

## Hard-won rules (do NOT relearn the hard way)
- PtCls order: {IN=0, OUT=1, ON=2}.
- Never hold Edge&/CoEdge&/Face& across vector push_back (two crash bugs).
- split_face part coedges have ce.face = part-space index: bounds-check before
  s.faces[ce.face] on a live solid.
- Tangent (osculating) sections must NOT split faces (|nA.nB| ~ 1 at ALL 8
  samples -> skip in pass 3). Closed loops that physically close must go
  through merge_sections -> prepare closed-cut (band cuts + junction arcs +
  pc_edge transition re-split), never chart-position merging of period-apart
  endpoints. Boundary junctions (dist to boundary < tol*100+0.005) must stay
  free ends for attach().
- BRep::volume (brep_massprops) has a 60M eval budget; budget-capped faces
  used to silently drop trims (fixed at src/brep_massprops.cpp:508).
  Multi-lump results were misread as nested cavities by the bbox-only depth
  test (fixed 2026-08-04, winding-number verification — see session-5 entry;
  corpus-neutral A/B-proven). These are the ONLY two v1 edits from v3 work,
  both in src/brep_massprops.cpp. Mesh volume
  (BRep::mesh().volume()) is the independent geometry oracle when in doubt.
- Debug env flags: V3BOOLDBG V3SPLITDBG V3CLSDBG V3COFDBG V3MRGDBG V3OUTDBG
  V3DUMPB V3MESHVOL V3RAW V3PROBE3 (see main_25.cpp / v3 sources).
- Machine: 30 GB RAM; free -h before builds; build -j2; one test binary at a
  time; kill stale main_*/cmake before sweeps.
