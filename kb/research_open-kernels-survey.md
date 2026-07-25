# Survey: Open-Source B-Rep / Solid Kernels Beyond OCCT — Boolean Implementation Ideas

Date: 2026-07-24. Scope: SMLib/NLib (docs), SolveSpace, FreeCAD (as OCCT power-user), truck, Fornjot, BRL-CAD (NMG + ray-CSG), OpenVSP, gmsh (OCC wrapper), libfive. Focus: novel robustness mechanisms; deep dives on how SolveSpace and BRL-CAD NMG handle coincident faces.

Companion KB files: `occt_pavefiller-core.md`, `occt_ff-posttreat-samedomain.md`, `occt_tolerance-model.md`, `reference_brep_boolean_kb.md` (memory).

---

## 1. SolveSpace — exact-curve NURBS booleans with explicit coincidence classes

SolveSpace has the only complete open-source *NURBS* (non-OCCT) boolean pipeline. Source: `src/srf/boolean.cpp`, `src/srf/surfinter.cpp` ([boolean.cpp](https://github.com/solvespace/solvespace/blob/master/src/srf/boolean.cpp), [repo](https://github.com/solvespace/solvespace)).

### Pipeline (`SShell::MakeFromBoolean`)
1. `CopyCurvesSplitAgainst()` — split all piecewise-linear (PWL) trim curves of each operand against the other shell's surfaces.
2. `MakeIntersectionCurvesAgainst()` — SSI for all surface pairs.
3. `CopySurfacesTrimAgainst()` — per-surface region classification and re-trim.
4. `RewriteSurfaceHandlesForCurves()` — fix curve→surface references.

### SSI design (`surfinter.cpp`)
- Case ladder: plane–plane analytic; plane–extrusion (2 subcases: extrusion ∥ plane → line-vs-curve in plane; else project extruded curve into plane); extrusion–extrusion same axis → parallel lines; general case → **numerical marching from boundary intersection points** with chord-tolerance step control.
- **Exact + PWL dual representation**: `AddExactIntersectionCurve` stores the exact rational-Bézier curve *and* a PWL approximation; crucially, "If there's already an identical curve in the shell, then follow that pwl exactly, otherwise calculate from scratch" — **intersection-curve reuse guarantees both operands see bitwise-identical section polylines** (their answer to our shared-closure/SEG-UNIFY problem, built in from day one).
- Fake-curve rejection: a candidate intersection curve is discarded if it "lies entirely outside one of the surfaces" (sampled PWL test) — a cheap global sanity filter after SSI.
- Refinement: chopped-curve endpoints are refined to lie on 3 surfaces simultaneously (`PointOnSurfaces`), "Finding the intersection to within EPS is important to match the ends of different chopped trim curves"; for exact curves a dedicated `PointOnCurve()` (curve-vs-surface Newton, approximating surface by plane per iteration) was added because 3-surface Newton **fails to converge where a curve joins two tangent patches** ([commit 68200d2](https://github.com/phkahler/solvespace/commit/68200d278c2db3767e683348bdc010211f7c634c)) — same failure mode as our tangential `newton_cc` stalls; their fix is the same family as our corrector-pinned crossings.

### Coincident-face handling (the requested detail)
- Detection: `CoincidentWith` / `CoincidentWithPlane` on surface pairs; coincident pairs are **excluded from SSI** (no intersection curves generated between coincident surfaces).
- Classification enum has **four states, not three**: `SURF_INSIDE`, `SURF_OUTSIDE`, `SURF_COINC_SAME`, `SURF_COINC_OPP` (coincident with same vs opposite normal).
- The keep/discard table (`KeepRegion`) resolves coincidence **asymmetrically by operand** so exactly one copy of an overlapping face survives:
  - UNION: opA keeps `outSide`; opB keeps `outSide || coincSame` (B contributes the shared face).
  - DIFFERENCE: opA keeps `outSide || coincOpp`; opB keeps `inShell`.
  - INTERSECTION: opA keeps `inShell`; opB keeps `inShell || coincSame`.
  - All gated by `inOrig` (region must be inside the surface's own original trim).
- Edge-level classification is a **2D UV BSP with 3D-mapped tolerance**: `ClassifyPoint` returns INSIDE/OUTSIDE/EDGE_PARALLEL/EDGE_ANTIPARALLEL; `ScaledSignedDistanceToLine` linearizes the surface so "a point is on-edge if its xyz distance to that edge is less than LENGTH_EPS" (tolerance defined in model space, applied in UV — same philosophy as OCCT's `Tolerance2D = Tol3D/Resolution`).
- **Tangent-curve orientation trick**: where surfaces are tangent along a curve, normals give no orientation, so SolveSpace adds the edge **in both directions** and lets shell classification "keep the correctly oriented one and discard the other" — generate-and-test instead of deciding orientation locally at a degenerate configuration.
- `TagByClassifiedEdge` converts `EDGE_PARALLEL`/`EDGE_ANTIPARALLEL` hits into coincident-region tags for `KeepRegion`.

### Known limits (their own docs)
"Incorrect classifications in surface intersection (numerical problems, or unhandled special cases of all the ways surfaces can be inside/outside/coincident/edge-on-surface/edge-on-edge/vertex-on-surface) typically account for failed Booleans"; chord tolerance changes can "fix" failures; recommended user workaround is modeling with fewer booleans ([issues doc](https://solvespace.readthedocs.io/en/latest/issues.html), [ref](https://solvespace.com/ref.pl)). NURBS booleans on curved surfaces were still being improved in v3.2 ([CHANGELOG](https://github.com/solvespace/solvespace/blob/master/CHANGELOG.md)).

**Takeaways for us**: (1) four-state region classification with per-operand asymmetric keep tables is a compact, provably single-copy way to do same-domain faces — directly relevant to P3; (2) both-directions edge emission at tangencies; (3) intersection-curve dedup/reuse as a *correctness* mechanism, not an optimization; (4) dedicated curve-surface Newton for tangent-junction refinement.

---

## 2. BRL-CAD NMG — pre-fuse everything, classify with explicit shared/anti-shared classes

BRL-CAD's NMG (non-manifold geometry, radial-edge derived) boolean is the most complete open-source implementation of the Weiler-style non-manifold boolean. Sources: [libnmg/bool.c](https://github.com/BRL-CAD/brlcad/blob/main/src/libnmg/bool.c), [libnmg/class.c](https://github.com/BRL-CAD/brlcad/blob/main/src/libnmg/class.c), [libnmg/eval.c](https://github.com/BRL-CAD/brlcad/blob/main/src/libnmg/eval.c), [libnmg/fuse.c](https://github.com/BRL-CAD/brlcad/blob/main/src/libnmg/fuse.c), [NMG docs](https://brl-cad.github.io/docs/), [doxygen](http://brlcad.sourceforge.net/doxygen/d4/db4/group__nmg.html).

### Pipeline (`nmg_bool`)
1. **Fast disjoint reject**: `V3RPP_DISJOINT_TOL(bboxA, bboxB, tol->dist)` — disjoint union = move faces, disjoint intersect = empty.
2. **Pre-fuse (the load-bearing step)**: `nmg_shell_coplanar_face_merge()` on each shell, then model-wide `nmg_model_fuse()` BEFORE any intersection: vertex fuse (spatial sort + `tol->dist`), `nmg_break_e_on_v` (split edges passing through vertices — "vertices and/or edges have been moved, may have created out-of-tolerance faces"), face-geometry fuse, edge fuse, edge-geometry fuse. Purpose per header: make geometric identity "explicit" as **shared topology** so the intersector and classifier never have to re-discover coincidence numerically.
3. `nmg_crackshells(sA, sB)` — shell/shell intersection (mutual imprint), then cleanup: `nmg_kill_anti_loops`, `nmg_s_split_touchingloops` ("make interior (touching) loop segments into true interior loops").
4. `nmg_classify_shared_edges_verts()` — "get all the easy shared edges and vertices marked as shared" **topologically first** (pointer equality after fusing), before any geometric classification.
5. `nmg_class_shells` both ways, with the AonBshared classlist **copied forward** so B-vs-A classification agrees with A-vs-B on shared elements by construction.
6. `nmg_evaluate_boolean` applies per-class RETAIN/KILL tables; subtraction is implemented as **global `nmg_invert_shell(sB)` + table**, no per-face flipping.
7. Post-checks: `nmg_has_dangling_faces` before/after, `nmg_s_radial_check`, re-fuse verification, closure warnings (`nmg_check_closed_shell`).

### Coincident-face handling (the requested detail)
- **Eight classes**: {AinB, AonBshared, AonBanti, AoutB} × both directions. "on" is split by relative orientation: `class_shared_lu()` compares edge "left vectors" (into-face directions) of a loop vs the radially shared reference loop — if `VDOT(left, left_ref) > 0` and edge directions agree → `AonBshared`; opposite direction → `AonBanti`; if left vectors oppose, radial face inspection decides IN vs OUT.
- **Coplanarity is topological at eval time**: `nmg_two_face_fuse` (bbox test, plane-D comparison `NEAR_ZERO(dist, tol->dist)`, all-vertices-within-`tol->dist`-of-other-plane via `nmg_ck_fg_verts`, then `VDOT(N1,N2)` for orientation) merges coplanar face *geometry* pre-boolean, so at classification time coincident faces literally share geometry structs and radial edges.
- **Per-op keep tables for shared/anti classes** (eval.c):
  - SUBTRACTION: kill AonBshared, **retain AonBanti** (A face coincident-but-opposed to inverted B becomes the cavity wall), kill B-side on-classes.
  - UNION: **retain AonBshared**, kill AonBanti, kill BonAshared/BonAanti (single copy survives, from A).
  - INTERSECTION: retain AinB/BinA and **AonBshared**; kill AonBanti ("generates non-manifold result" otherwise).
  - KILL is implemented as *demotion* (loop→edges→vertices) not deletion, preserving lower-dim structure for the non-manifold result.
- Tolerance model: single global `bn_tol` with `dist` (absolute distance, default 0.0005 model units) + `perp`/parallel dot thresholds — one coherent tolerance used identically by fuse, intersect, classify. Classification of points uses Jordan ray-count with **multiple retry directions** (`nmg_good_dirs`) when a ray hits degeneracy.

### Ray-partition CSG (the other BRL-CAD boolean)
BRL-CAD's primary evaluation never builds a B-rep at all: per ray, `rt_boolweave` weaves per-primitive hit segments into partitions and `rt_boolfinal` evaluates the CSG expression on each partition ([bool.c](https://brlcad.org/OLD/doxygen/d7/db1/bool_8c.html)). This is "100% reliable, numerically deterministic" and is why their own robust-facetization GSoC proposal suggests **voxelize-via-raytrace + evaluate in OpenVDB + repolygonize** as the always-works fallback when NMG fails ([GSoC issue #70](https://github.com/opencax/GSoC/issues/70)). NMG itself "works some of the time"; their candidate fixes are exhaustive SSI coverage + unit tests, or NMG re-implementation. Historical design papers: Muuss & Butler, *Combinatorial Solid Geometry, Boundary Representations, and Non-Manifold Geometry* ([ARL](https://ftp.arl.army.mil/~mike/papers/90nmg/joined.html), site intermittently down).

**Takeaways for us**: (1) *fuse-first* discipline — coincidence resolved once, structurally, before SSI; classification then reads topology (pointer identity), never re-derives coincidence from floats (our same-domain P3 should adopt exactly this: detect same-domain pairs → weld to shared geometry → classify by orientation flag); (2) shared/anti split with per-op tables matches SolveSpace's coincSame/coincOpp — two independent kernels converged on the same 4-way classification, strong evidence it is the right abstraction; (3) copy shared classifications between the two classification passes to force A/B symmetry (our symmetric-coverage flood is a rediscovery of this); (4) kill-by-demotion keeps the model consistent mid-surgery; (5) dangling-face + radial-consistency checks bracketing every boolean = built-in P6-style validation gates.

---

## 3. SMLib/NLib — the commercial reference point (docs only)

- NLib = pure NURBS math (Piegl/Tiller "The NURBS Book" code lineage), explicitly "does not have topological structures and does not perform intersections" ([NLib manual](https://docs.nvidia.com/smlib/manual/nlib/introduction/index.html)). SMLib layers **non-manifold topology** on top ([SMLib intro](https://docs.nvidia.com/smlib/manual/smlib/introduction/index.html), [manual root](https://docs.nvidia.com/smlib/manual/smlib/index.html)).
- Booleans via `SmMerge` class with a **rich op set beyond CSG**: `SM_BO_UNION/INTERSECTION/DIFFERENCE/EXCLUSIVE_OR/MERGE/PARTIAL_MERGE/IMPRINT/EXTRACT_SEPARATE/SLICE`; operators `SmMerge::ManifoldBoolean` or `SmMerge::NonManifoldBoolean`; plus `StitchFace`, `merge_breps` ([booleans page](https://docs.nvidia.com/smlib/manual/smlib/booleans/index.html)).
- Key architectural claim: with non-manifold topology "Boolean operation is now a closed operation… you can not do a Boolean that would produce something you can not represent topologically"; `SmBrep::MakeManifold` extracts a manifold solid afterwards by removing excess geometry.
- Now legacy-only licensing (NVIDIA acquisition); manual hosted by NVIDIA. ([Wikipedia](https://en.wikipedia.org/wiki/Solid_Modeling_Solutions))

**Takeaways**: the MERGE/PARTIAL_MERGE/IMPRINT/SLICE decomposition mirrors OCCT GF/Splitter/CellsBuilder — confirms our plan to expose imprint + cell extraction as first-class ops; "boolean closed under non-manifold + manifold extraction at the end" is the same shape as our scaffold→closure-weld→extract pipeline.

---

## 4. FreeCAD — OCCT usage patterns (what a production consumer bolts on)

FreeCAD is the largest OCCT boolean consumer and its workarounds are a map of OCCT's failure surface:

- **Auto-fuzzy by default**: fuzzy tolerance computed as `FuzzyHelper::getBooleanFuzzy() * sqrt(bbox.SquareExtent()) * Precision::Confusion()` (i.e., *scale-proportional* fuzzy, default multiplier 1.0) passed to `SetFuzzyValue()`; issue [#26119](https://github.com/FreeCAD/FreeCAD/issues/26119) shows why: arrays/rotations introduce ~1e-14 coordinate error, making exactly-coincident faces *nearly*-coincident, and "OCCT BOPAlgo cannot process nearly-coincident faces without fuzzy tolerance". Fix = enable auto-fuzzy in `makeElementBoolean()` even when the user passes tolerance 0.
- **Refine after boolean**: `removeSplitter()` (OCCT same-domain face unification) is routinely applied post-boolean to erase redundant splits ([Part API](https://freecad-python-stubs.readthedocs.io/en/latest/autoapi/Part/_TopoShape/)).
- **Check-then-heal loop**: `shape.check(runBopCheck=True)` (BOPAlgo_ArgumentAnalyzer) before booleans; `sew()` for gaps; documented user recipe for failed booleans is check-geometry → sew → retry ([XSim guide](https://www.xsim.info/articles/FreeCAD/en-US/HowTo/Fix-failed-boolean-operations.html), [TaskCheckGeometry.cpp](https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/Part/Gui/TaskCheckGeometry.cpp)).
- Fuzzy semantics (OCCT): "measure the gap… and slightly increase it to make shifted entities coincident in terms of their tolerance plus the additional one" ([OCCT boolean docs](https://dev.opencascade.org/doc/overview/html/specification__boolean_operations.html), [fuzzy forum](https://dev.opencascade.org/content/fuzzy-boolean-operations)).

**Takeaways**: our P5 tolerance model should include a **scale-proportional auto-fuzzy default** (bbox-diagonal × k), not opt-in — the single highest-value robustness knob a consumer ever found for OCCT; and an argument-analyzer-style precheck (self-intersection, tiny edges) as a formal gate.

---

## 5. truck (Rust) — transversal-only booleans, honest about it

- `truck-shapeops` provides only `and()` / `or()` on `Solid`; docs state faces must "intersect **transversally** — tangential face intersections remain unsupported"; BSP acceleration listed as future work ([docs.rs](https://docs.rs/truck-shapeops/latest/truck_shapeops/), [repo](https://github.com/ricosjp/truck), [lib.rs](https://lib.rs/crates/truck-shapeops)). Algorithm (from crate structure/history): tessellate → polyline intersection curves → divide faces → classify loops → integrate; `healing` module (`SplitClosedEdgesAndFaces` + `RobustSplitClosedEdgesAndFaces`) splits closed/periodic edges & faces on import — i.e., **periodic-seam decomposition as a precondition for booleans** (independent confirmation of our seam-decomposition groundwork).
- Fork `monstertruck` claims booleans + fillets + shape healing, "heavily fortified" ([monstertruck](https://github.com/virtualritz/monstertruck), [docs.rs](https://docs.rs/monstertruck)); community consensus: booleans remain the hard unfinished part ([HN](https://news.ycombinator.com/item?id=35071317)).

**Takeaways**: nothing algorithmically novel to borrow, but two confirmations: closed-entity splitting as a healing *pre-pass*, and "transversal-only" as an explicitly declared capability boundary (worth emulating in our capability matrix: declare tangential/same-domain support level per phase).

---

## 6. Fornjot (Rust) — booleans blocked on foundations

Booleans (union/difference/intersection, issues #42/#43/#44) are "the most important feature" but **blocked**; the kernel's "3D-mostly" coordinate handling caused problems and they concluded geometry-manipulation infrastructure (+ many intersection tests) must come first ([discussion #146](https://github.com/hannobraun/fornjot/discussions/146), [dev log](https://www.fornjot.app/blog/weekly-dev-log/2022-w19/), [interview](https://console.substack.com/p/console-116)). Their pivot to surface-local (UV-first) representations to make intersections tractable is the lesson.

**Takeaway**: negative result worth recording — a B-rep kernel that deferred a principled pcurve/UV story could not reach booleans at all; validates our pcurve-centric arrangement architecture.

---

## 7. OpenVSP — engineering answer: mesh CSG with sliver hygiene

OpenVSP never attempts NURBS booleans; `CompGeom` meshes, intersects, and trims components — "CSG-style intersection… exact wetted surface of the *faceted* representation", handling thick/thin combinations ([API docs](https://openvsp.org/api_docs/latest/group___computations.html)). Robustness mechanisms visible in the changelog: post-intersection **sliver elimination pass**; move from triangles to **NGON polygonal mesh** results; **dual CDT strategy — DBA as primary with automatic fallback to Triangle on detected failure**; duplicate-edge suppression for subsurface intersections ([CHANGELOG](https://github.com/OpenVSP/OpenVSP/blob/main/CHANGELOG.md), [3.46 release](https://openvsp.org/blogs/announcements/2025/09/30/openvsp-3-46-0-released)); negative-volume trimming workflow ([workshop pdf](https://openvsp.org/wiki/lib/exe/fetch.php?media=workshop21%3A2021_vspws_-_trimming_components_with_negative_volume.pdf)).

**Takeaways**: (1) primary-algorithm + detect-failure + fallback-algorithm pattern for triangulation (we can do the same for face CDT and even whole-boolean: exact path → scaffold path → mesh-CSG path, which we already prototyped in main_8); (2) explicit sliver-artifact cleanup as a named pipeline stage.

---

## 8. gmsh — OCC wrapper findings

- Exposes OCCT fuzzy directly as `Geometry.ToleranceBoolean` (default 0 → users routinely set e.g. 1e-3 to rescue failures) ([tutorial](https://bthierry.pages.math.cnrs.fr/tutorial/gmsh/occ/basics/), [gmsh manual](https://gmsh.info/doc/texinfo/gmsh.html), [GModelIO_OCC.cpp](https://github.com/jeromerobert/gmsh/blob/master/Geo/GModelIO_OCC.cpp)).
- Documented field experience: boolean failures are parameter-sensitive (sphere/cone cells fail, tiny perturbation passes) ([gmsh #923](https://gitlab.onelab.info/gmsh/gmsh/-/work_items/923), [pygmsh #93](https://github.com/nschloe/pygmsh/issues/93)); **fuse output tolerance can exceed input tolerances and accumulates over chained ops** ([OCC tolerance forum](https://dev.opencascade.org/content/tolerance-issues)); `BooleanFragments` (= OCCT GF) is the workhorse for conformal multi-domain models, "removes duplicated entities including common boundaries" ([pygmsh docs](https://pygmsh.readthedocs.io/en/latest/occ.html)).
- Import hygiene options mirror FreeCAD: `healShapes`, fix-degenerated/fix-small-edges/sew before booleans; gmsh deliberately operates on **native kernel representation** (no translation) to avoid tolerance loss ([manual](https://gmsh.info/doc/texinfo/gmsh.html)).

**Takeaways**: tolerance *growth accounting* across chained booleans is a real user-facing failure (P5 must track and report per-op tolerance inflation, and test chained-op corpora, not single ops); fragment/GF-style "common boundary dedup" is the API meshing users actually need.

---

## 9. libfive — the implicit counterpoint

F-rep kernel: shapes are `f(x,y,z)<0` fields; CSG is literally `min/max` — "robust boolean operations" by construction, no intersection curves, no classification, trivially parallel ([libfive](https://github.com/libfive/libfive), [deepwiki](https://deepwiki.com/libfive/libfive), [discussion #509](https://github.com/libfive/libfive/discussions/509)). Cost: no exact edges/faces (feature reconstruction at meshing time), no per-face attributes, offsets of NURBS not representable exactly. Same trade SDF-CAD critiques note ([sdf-thoughts](https://incoherency.co.uk/blog/stories/sdf-thoughts.html)).

**Takeaway**: as a *verification oracle* — an implicit/min-max evaluation of the same boolean (via our SDF marching-cubes path) gives an independent inside/outside field to cross-check B-rep classification verdicts (we already exploit this in main_8/volume-flux tier-3; worth formalizing in P6).

---

## Cross-kernel synthesis — mechanisms worth adopting

| # | Mechanism | Seen in | Maps to our phase |
|---|-----------|---------|-------------------|
| 1 | 4-way region classes (IN/OUT/COINC_SAME/COINC_OPP) + per-op per-operand keep tables | SolveSpace `KeepRegion`, NMG eval tables | P3 same-domain |
| 2 | Fuse-first: weld coincident verts/edges/faces into shared topology BEFORE SSI; classify by pointer identity + orientation | NMG `nmg_model_fuse` | P3, P4 |
| 3 | Copy shared classifications between A-vs-B and B-vs-A passes (forced symmetry) | NMG bool.c | P3 (our symmetric flood) |
| 4 | Emit tangent-curve edges in both directions; classification discards the wrong one | SolveSpace | P4 tangent EF |
| 5 | Intersection-curve reuse: identical curve in shell ⇒ reuse its exact PWL | SolveSpace | done (shared-closure) — keep as invariant |
| 6 | Dedicated curve×surface Newton at tangent junctions (3-surface Newton diverges) | SolveSpace 68200d2 | P4 EE/EF |
| 7 | Scale-proportional auto-fuzzy ON BY DEFAULT (bbox-based) | FreeCAD | P5 |
| 8 | Tolerance-growth tracking across chained ops + argument precheck gate | gmsh/OCC field data, FreeCAD BOPCheck | P5, P6 |
| 9 | Primary/fallback algorithm ladder with failure detection (DBA→Triangle; NMG→voxel/ray) | OpenVSP, BRL-CAD GSoC | P6 + our exact→scaffold→mesh ladder |
| 10 | Kill-by-demotion (face→edges→verts) instead of deletion during eval | NMG eval.c | P3/P4 surgery hygiene |
| 11 | Ray/implicit independent oracle for classification cross-check | BRL-CAD ray-CSG, libfive | P6 validation |
| 12 | Closed-edge/periodic-face split as healing pre-pass; transversal-only capability declaration | truck healing | seam decomposition (in progress) |
| 13 | Post-boolean structural checks: dangling faces, radial consistency, re-fuse verification | NMG | P6 gates |
| 14 | Non-manifold-closed boolean + manifold extraction step | SMLib `NonManifoldBoolean`+`MakeManifold`, NMG | architecture north star |

## Sources
- SolveSpace: https://github.com/solvespace/solvespace/blob/master/src/srf/boolean.cpp · https://solvespace.readthedocs.io/en/latest/issues.html · https://solvespace.com/ref.pl · https://github.com/phkahler/solvespace/commit/68200d278c2db3767e683348bdc010211f7c634c · https://github.com/solvespace/solvespace/blob/master/CHANGELOG.md
- BRL-CAD: https://github.com/BRL-CAD/brlcad/blob/main/src/libnmg/bool.c · .../class.c · .../eval.c · .../fuse.c · https://brl-cad.github.io/docs/ · http://brlcad.sourceforge.net/doxygen/d4/db4/group__nmg.html · https://brlcad.org/OLD/doxygen/d7/db1/bool_8c.html · https://github.com/opencax/GSoC/issues/70 · https://ftp.arl.army.mil/~mike/papers/90nmg/joined.html
- SMLib/NLib: https://docs.nvidia.com/smlib/manual/smlib/index.html · https://docs.nvidia.com/smlib/manual/smlib/booleans/index.html · https://docs.nvidia.com/smlib/manual/nlib/introduction/index.html · https://en.wikipedia.org/wiki/Solid_Modeling_Solutions
- FreeCAD: https://github.com/FreeCAD/FreeCAD/issues/26119 · https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/Part/Gui/TaskCheckGeometry.cpp · https://www.xsim.info/articles/FreeCAD/en-US/HowTo/Fix-failed-boolean-operations.html · https://freecad-python-stubs.readthedocs.io/en/latest/autoapi/Part/_TopoShape/
- OCCT fuzzy: https://dev.opencascade.org/doc/overview/html/specification__boolean_operations.html · https://dev.opencascade.org/content/fuzzy-boolean-operations · https://dev.opencascade.org/content/tolerance-issues
- truck: https://github.com/ricosjp/truck · https://docs.rs/truck-shapeops/latest/truck_shapeops/ · https://github.com/virtualritz/monstertruck · https://news.ycombinator.com/item?id=35071317
- Fornjot: https://github.com/hannobraun/fornjot/discussions/146 · https://www.fornjot.app/blog/weekly-dev-log/2022-w19/ · https://console.substack.com/p/console-116
- OpenVSP: https://github.com/OpenVSP/OpenVSP/blob/main/CHANGELOG.md · https://openvsp.org/api_docs/latest/group___computations.html · https://openvsp.org/blogs/announcements/2025/09/30/openvsp-3-46-0-released
- gmsh: https://gmsh.info/doc/texinfo/gmsh.html · https://bthierry.pages.math.cnrs.fr/tutorial/gmsh/occ/basics/ · https://gitlab.onelab.info/gmsh/gmsh/-/work_items/923 · https://github.com/nschloe/pygmsh/issues/93 · https://pygmsh.readthedocs.io/en/latest/occ.html
- libfive: https://github.com/libfive/libfive · https://deepwiki.com/libfive/libfive · https://github.com/libfive/libfive/discussions/509 · https://incoherency.co.uk/blog/stories/sdf-thoughts.html
