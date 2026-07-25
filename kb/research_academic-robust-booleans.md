# Academic Robust Boolean / Mesh-CSG Literature Survey (2010–2026)

Deep survey of academic robustness literature for boolean operations, evaluated against ONE question:
**what can a TOLERANCE-BASED NURBS B-Rep kernel (OCCT-aligned) actually adopt?**
Pre-2010 foundations included where they define the tolerance-based paradigm we live in.
Companion to `occt_tolerance-model.md`, `occt_pavefiller-core.md`, and the B-Rep boolean KB
(`reference_brep_boolean_kb.md` in memory).

Program context: P3 same-domain subsystem, P4 EE/EF interference, P5 tolerance model, P6 corpus
validation; current frontier = rotated-chair robustness (freeform SSI completeness, scaffold flood,
classification).

---

## 0. Taxonomy — five robustness strategies

| # | Strategy | Canonical exponents | Guarantee | Cost | Transfers to NURBS B-Rep? |
|---|----------|--------------------|-----------|------|---------------------------|
| 1 | **Pure exact arithmetic (EGC)** | CGAL Nef, ESOLID, 3D-EPUG, Lévy 2024 | absolute (input = algebraic) | 10–1000× slowdown, memory blowup | NO as a whole — NURBS SSI curves are non-algebraic-representable; YES for isolated *decisions* |
| 2 | **Hybrid float/exact (filtered predicates, lazy exact, implicit points)** | Zhou 2016, Cherchi 2020/2022, EMBER, Attene indirect predicates, CGAL corefinement | exact decisions, float speed on non-degenerate input | 1–3× | YES for linearized subproblems (UV arrangements, cell classification, tie-breaking) |
| 3 | **Tolerance-based topology** | Segal 1990, Jackson 1995 (Parasolid), Fang–Brüderlin 1993, ACIS tolerant modeling | consistency (not exactness) under stated separation invariants | ~1× | THIS IS US — the only strategy proven at industrial NURBS scale (Parasolid/ACIS/OCCT) |
| 4 | **Interval / certified numerics** | Hu–Patrikalakis 1996, IATA TOG 2023, Topology-Guaranteed SSI TOG 2023 | certified enclosure / certified topology | 2–10× on hot paths | YES — the modern answer to SSI topology failures (our tangential stalls, missed small loops) |
| 5 | **Perturbation / symbolic** | SoS (Edelsbrunner–Mücke), controlled perturbation (Halperin), snap rounding | removes degeneracies by construction | ~1× + logical complexity | YES for degeneracy tie-breaking in same-domain (P3) and EE/EF (P4) |

Meta-law extracted from the whole corpus (Hoffmann's principle, restated by every successful system):
**robustness = making all topological decisions mutually consistent, not making them all correct.**
Exact methods achieve consistency by correctness; tolerance methods must achieve it by
*deriving dependent decisions from a minimal set of independent ones* and never re-deriving the
same fact two different ways. Most of our historical bugs (asymmetric flood coverage, one-sided
whole-segment loss, alias-key misses) are exactly re-derivation inconsistencies.

---

## 1. Exact arithmetic approaches

### 1.1 CGAL Nef polyhedra — Granados/Hachenberger/Hert/Kettner/Mehlhorn/Seel (ESA 2003; CGAL Nef_3)
- <https://doc.cgal.org/latest/Nef_3/index.html>
- <https://www.researchgate.net/publication/222702927_Boolean_operations_on_3D_selective_Nef_complexes_Data_structure_algorithms_optimized_implementation_and_experiments>
- **Core idea**: solids = Nef polyhedra (finite boolean combinations of open halfspaces), closed
  under ALL set operations including non-regularized ones. Data structure = Selective Nef Complex:
  global 3D structure + a **sphere map** at every vertex (a 2D Nef structure on an infinitesimal
  sphere recording the exact local cone of the solid). All arithmetic exact (rational).
- **Robustness mechanism**: every local configuration — vertex on face, edge through edge, tangent
  contact — is representable, so there are no "special cases"; degeneracy is first-class.
- **Known weakness**: hundreds of seconds and GB-scale memory for modest models; linear geometry only.
- **Adopt (P4)**: the **sphere-map idea is the correct mental model for EE/EF interference**. When
  an edge of A passes through an edge/vertex of B, the complete answer is the arrangement of surface
  wedges on an infinitesimal sphere around the contact — i.e., classify by *local cone*, not by
  sampling nearby points. A tolerance-kernel version: at each interference vertex, sort incident
  faces by angle around the shared tangent and derive in/out wedges combinatorially once, then reuse
  that single decision everywhere (never re-classify per face). OCCT's BOPDS interference tables are
  a flattened, lossy version of this; the sphere map says what the lossless version stores.

### 1.2 CGAL Polygon Mesh Processing corefinement booleans — Loriot et al.
- <https://doc.cgal.org/latest/Polygon_mesh_processing/group__PMP__corefinement__grp.html>
- **Core idea**: co-refine both meshes against the SAME exactly-computed intersection polylines
  (lazy-exact kernel: floats with interval filter, exact rational fallback), then classify and stitch.
  Intersection edges are marked as constrained and survive as shared topology.
- **Adopt (P3, already partially ours)**: the **single-shared-section law** — one geometric
  section entity, referenced by both operands, never two independently computed copies. Our BOP2
  Phase-4 pool referencing (39-edge arena alias) is exactly this; the corefinement design confirms it
  must extend to *pcurves and split parameters too* (both operands must split at literally the same
  parameter values on the shared curve — our SEGLOST/one-sided-split class is a violation of this law).

### 1.3 ESOLID + MAPC — Keyser, Culver, Manocha, Krishnan (Solid Modeling 2002)
- <http://www.cs.unc.edu/~geom/ESOLID/> ; <https://dl.acm.org/doi/abs/10.1145/566282.566289>
- **Core idea**: exact boundary evaluation for *curved* (low-degree algebraic) CSG primitives.
  Exact algebraic numbers for curve-curve intersections in trim domains (MAPC library), exact SSI.
- **Reality check**: works, but restricted to low-degree implicit/parametric algebraic surfaces and
  runs orders of magnitude slower; nobody scaled it to NURBS. This is the strongest evidence that
  **full EGC does not transfer to a NURBS kernel** — SSI curves of bicubics have astronomical
  algebraic degree; exactness must be *localized to decisions*, not representations.
- **Adopt (design law)**: know the algebraic degree of every predicate you evaluate; where the
  degree is low (plane/quadric cases, our analytic recognizers), exact answers ARE affordable —
  which justifies our analytic-first architecture (recognize_solid kinds, IntAna-style paths) and
  argues for extending recognizer coverage (tilted cones, cone×cone) rather than accepting the
  freeform path for recognizable geometry.

### 1.4 Exact Weiler model for mesh CSG — Bruno Lévy (arXiv 2405.12949, TOG 2025; geogram)
- <https://arxiv.org/abs/2405.12949> ; <https://dl.acm.org/doi/10.1145/3744642>
- **Core idea**: first fully exact **arrangement (Weiler model)** of a triangle soup: co-refinement
  with exact intersection points, facet radial sort around non-manifold edges via specialized exact
  predicates, constrained Delaunay retriangulation with **symbolic perturbation for co-cyclic
  points** so the retriangulation is *uniquely defined* (deterministic). Two exact kernels compared:
  arithmetic expansions vs multi-precision floats.
- **Adopt (P3)**: **canonical, uniquely-defined re-triangulation/arrangement**. Our UV-arrangement
  splits must be a *function of the input set only* — no iteration-order or seed dependence — so
  both operands and repeated runs produce identical decompositions. Wherever we have "first hit
  wins" logic in the splitter, replace with a canonical ordering (lexicographic on exact/quantized
  keys). Determinism is a robustness feature: it converts Heisenbugs into reproducible ones.
- Also: radial sort of faces around a shared section edge is THE primitive for building the Weiler
  model; our scaffold flood approximates the same information — an explicit radial-sort structure
  around each section edge (faces of A and B interleaved by dihedral angle) would replace
  quorum-style flood heuristics with a combinatorial invariant.

### 1.5 3D-EPUG-Overlay — Magalhães, Franklin, Andrade (CAD 2020 etc.)
- <https://wrf.ecse.rpi.edu/nikola/pages/salles/> ;
  <https://www.sciencedirect.com/science/article/abs/pii/S0010448519305330>
- **Core idea**: exact parallel mesh intersection using rationals + **Simulation of Simplicity** for
  all degeneracies + uniform grid; key engineering result: the whole pipeline reduced to
  **orientation predicates only** (no in-circle, no constructions in decisions).
- **Adopt**: predicate minimalism. Audit our boolean decision points and reduce them to the smallest
  predicate basis possible (orientation/side-of-surface); every distinct predicate type is a distinct
  inconsistency channel. SoS reference implementation guide: <https://arxiv.org/abs/2212.08226>.

### 1.6 Sound solid modeling via exact real arithmetic — Sherman, Michel, Carbin (ICFP 2019)
- <https://dl.acm.org/doi/10.1145/3341703> ; <https://people.csail.mit.edu/sherman/papers/icfp19.pdf>
- **Core idea**: represent solids as continuous maps in an exact-real-arithmetic language
  (MarshallB/StoneWorks); all programs sound by construction; K-rep enables Minkowski sums,
  Hausdorff distance. Distinguished Paper.
- **Applicability**: conceptual north star only (no B-Rep output, performance far from interactive).
  Useful idea: **robustness via continuity** — predicates that are discontinuous in the input are
  the only places robustness can fail; catalogue ours (in/out at tangency, seam crossing, quorum
  thresholds) and treat each as requiring either certification (intervals) or a tolerance contract.

---

## 2. Hybrid floating/exact — the modern mesh-boolean wave

### 2.1 Mesh Arrangements for Solid Geometry — Zhou, Grinspun, Zorin, Jacobson (SIGGRAPH 2016; libigl, Blender)
- <https://www.cs.columbia.edu/cg/mesh-arrangements/> ; <https://dl.acm.org/doi/10.1145/2897824.2925901>
- **Core idea**: exact (rational) resolution of all triangle intersections → cell decomposition of
  space → **winding-number VECTOR per cell** (one integer per input operand) → any boolean (union,
  intersect, minus, XOR, arbitrary N-ary expressions) = a selection function over winding vectors.
  Robust even to self-intersecting inputs. Industrial adoption: Blender's exact boolean.
- **Adopt (classification doctrine, P3/P6)**:
  1. **Classify CELLS (volumetric regions), not faces.** A face's fate = derived from the winding
     vectors of its two adjacent cells. Our historical face-flip failures (all-faces-flip on z15/z30,
     fixed by tier-3 volume-flux) are the textbook symptom of classifying faces directly.
  2. **Winding vector (w_A, w_B) subsumes all ops** — cut = (w_A≥1 ∧ w_B=0), fuse = (w_A+w_B≥1),
     common = both, XOR = exactly one. One classification pass, four ops for free — this matches and
     generalizes our boolean_chain and disjoint-assembly XOR handling.
  3. N-ary evaluation without intermediate B-Reps (see also QuickCSG §2.7) avoids error accumulation.

### 2.2 Fast & Robust Mesh Arrangements using Floating-point — Cherchi, Livesu, Scateni, Attene (SIGGRAPH Asia 2020)
- <https://github.com/gcherchi/FastAndRobustMeshArrangements>
- **Core idea**: same arrangement as Zhou 2016 but 10–100× faster by replacing rational constructed
  points with **implicit points + indirect predicates** (§2.3); points that are intersections stay
  *unevaluated*; only predicates on them are computed, in floating point with exact filtered fallback.

### 2.3 Indirect predicates — Marco Attene (Computer-Aided Design 2020)
- <https://arxiv.org/abs/2105.09772> ; <https://github.com/MarcoAttene/Indirect_Predicates>
- **Core idea**: an intersection point is stored as its *defining entities* (Line-Plane LPI,
  Three-Planes TPI, LiNear-Combination LNC), never as coordinates. Predicates take the defining
  entities and answer exactly via floating-point expansions with static/dynamic filters. Exactness
  of decisions with ~float speed; constructions deferred to output time (then rounded once).
- **Adopt (HIGH VALUE, P3/P4)**: this is the exact-arithmetic idea that best fits a tolerance NURBS
  kernel because it is *local and incremental*. Concrete transfers:
  - **Never round-then-reason.** Our section-curve points snapped to tolerance and *then* used for
    alias-key lookups (the fb=47.99997 sliver dropping keys) is the anti-pattern. Keys must be
    derived from defining entities (face-pair id + curve id + branch index), not from rounded
    coordinates. BOP2's arena alias is the right structure; extend it so every derived point/param
    carries provenance (which SSI, which trim, which weld produced it).
  - In UV space our arrangement is piecewise-linear/low-degree: LPI/TPI-style implicit points +
    filtered orient2d are directly usable for the P3 same-domain splitter (exact 2D decisions at
    float cost, e.g. Shewchuk orient2d: <https://www.cs.cmu.edu/~quake/robust.html>).

### 2.4 Interactive & Robust Mesh Booleans — Cherchi, Pellacini, Attene, Livesu (SIGGRAPH Asia 2022)
- <https://dl.acm.org/doi/10.1145/3550454.3555460> ; <https://github.com/gcherchi/InteractiveAndRobustMeshBooleans>
- **Core idea**: first robustness-guaranteed boolean pipeline at interactive rates (200K tris):
  arrangement reuse across frames, parallelized indirect-predicate arrangement, inside/outside via
  localized ray casting with exact predicates.
- **Adopt (P6 / product)**: incremental reuse — when only one operand moves (our rotated-chair
  campaign literally re-runs the same pair under rotation), cache the operand's self-structures
  (face BVH, UV arrangements, recognizer results) and recompute only the cross terms. Also validates
  our "fast probe" methodology (SESSION_ROT_VOL) as an engineering pattern.

### 2.5 EMBER — Trettner, Nehring-Wirxel, Kobbelt (SIGGRAPH 2022)
- <https://dl.acm.org/doi/abs/10.1145/3528223.3530181> ;
  <https://www.graphics.rwth-aachen.de/media/papers/339/ember_exact_mesh_booleans_via_efficient_and_robust_local_arrangements.pdf>
- **Core idea**: meshes represented **plane-based** (each triangle = 3 planes with homogeneous
  integer coefficients; vertices = derived TPI, never stored). All predicates = integer determinant
  signs → unconditionally exact, branch-free, SIMD-able. Adaptive recursive bbox subdivision builds
  **local arrangements only where operands interact**; generalized winding numbers for classification;
  no global acceleration structure.
- **Adopt (architecture-level)**:
  1. Plane-based = "the SURFACE is the ground truth; points are derived" — the polyhedral twin of
     our analytic-STEP-reader doctrine (keep cylinder/cone/torus exact, derive everything). The
     lesson: push *derivedness* one level further — trim-curve endpoints and split params should be
     derived-on-demand from (surface, surface, surface/trim) triples, not stored rounded.
  2. **Locality**: build arrangements only inside interference cells (we already localize SSI; the
     paper shows classification can be localized too — winding evaluated per subdivision cell with
     early outs).

### 2.6 Octree-Embedded BSPs for iterated CSG — Nehring-Wirxel, Trettner, Kobbelt (CAD 2021)
- <https://arxiv.org/abs/2103.02486>
- **Core idea**: global octree, per-cell BSP with integer plane coefficients; booleans = BSP merge
  per cell; 2.5M mesh-plane cuts/s/core; unconditionally robust iterated CSG (error does not
  accumulate over long op chains).
- **Adopt (P6)**: iterated-boolean stress tests belong in the corpus — chains of 10–100 ops expose
  tolerance inflation (P5: does our tolerance-growth law stay bounded over chains?). Parasolid-class
  kernels are validated exactly this way.

### 2.7 QuickCSG — Douze, Franco, Raffin (Inria 2015/2017)
- <https://inria.hal.science/hal-01121419> ; <https://arxiv.org/abs/1706.01558>
- **Core idea**: N-ary boolean of many solids in one pass; vertex-centric formulation; single KD-tree
  geared to the *output*; not fully robust (relies on perturbation; acknowledged failure cases) but
  extremely fast. Useful as the cautionary tale: speed-first without an exactness or tolerance
  contract yields a long tail of failures — the same long tail we see in naked-edge counts.

### 2.8 Fast winding numbers — Barill, Dickson, Schmidt, Levin, Jacobson (SIGGRAPH 2018)
- <https://www.dgp.toronto.edu/projects/fast-winding-numbers/> ; <https://github.com/GavinBarill/fast-winding-number-soups>
- **Core idea**: generalized winding number = solid-angle integral, defect-tolerant in/out measure
  for soups/clouds; tree-based dipole/multipole approximation makes it O(log n) per query.
  Newer: Antipodal method (<https://arxiv.org/abs/2605.01536>) for accuracy/robustness improvements.
- **Adopt (validated already, extend)**: our winding-number contains_point (replacing 7-ray parity)
  and the tier-3 volume-flux classifier are this literature's recommendations verbatim. Extension
  worth taking: **hierarchical evaluation** (dipole tree over our tessellation) to make flux/winding
  classification cheap enough to run per-cell instead of per-op, enabling Zhou-style winding-vector
  classification (§2.1) at acceptable cost; and use winding as the *arbiter* when flood/quorum
  produces asymmetric verdicts (our 7/9-both-ways rule is a hand-made proxy for a continuous measure).

### 2.9 Manifold library — Emmett Lalish (open source, used by Blender/OpenSCAD ecosystem)
- <https://github.com/elalish/manifold>
- **Core idea**: guaranteed-manifold mesh booleans; reliability-first engineering (no exact-kernel
  opt-in, no precision crashes); parallel (Thrust/CUDA/OMP).
- **Adopt (P6 discipline)**: their invariant — *output is always a valid watertight oriented
  2-manifold, no caveats* — is the right acceptance bar phrasing for our gates: define the kernel's
  postcondition as a checkable structural invariant (closed shells, consistent orientation,
  naked=0, BRepCheck-valid) and refuse to ship results violating it, falling back (mesh-CSG path)
  instead. We already trend this way (naked-0 EXACT gates); make it the formal contract.

### 2.10 Robust implicit surface networks — Du, Zhou, Carr, Ju (SIGGRAPH 2022) + Boundary-Sampled Halfspaces (SIGGRAPH 2021)
- <https://dl.acm.org/doi/10.1145/3528223.3530176> ; <https://github.com/duxingyi-charles/Robust-Implicit-Surface-Networks>
- <https://dl.acm.org/doi/10.1145/3450626.3459870>
- **Core idea (2022)**: exact arrangement/material-interface extraction of piecewise-linear implicit
  functions on a tet grid; nested-region identification; look-up tables for tet splitting.
  **(2021)**: solids as halfspaces + boundary samples (CSG without boolean expressions).
- **Adopt**: the material-interface framing generalizes two-operand booleans to N materials — the
  right abstraction if the kernel ever does multi-body fuse with region labels (assembly booleans,
  our disjoint-assembly XOR). Otherwise low direct transfer.

### 2.11 Mandoline cut-cells — Tao, Batty, Fiume, Levin (SIGGRAPH Asia 2019)
- <https://www.dgp.toronto.edu/projects/mandoline/> ; <https://dl.acm.org/doi/10.1145/3355089.3356543>
- **Core idea**: cut-cell mesh from surface × grid: computes only segment-vs-axis-plane
  intersections (lowest-degree primitive available), then builds cut-edges → cut-faces → cut-cells
  **bottom-up with topological-correctness guarantees at each level**; handles open/non-manifold input.
- **Adopt (P3/scaffold doctrine)**: **dimension-ladder construction with per-level invariants**.
  Our scaffold pipeline (paves → section chains → loops → faces → shells) should assert a closure
  invariant at each level (every cut-edge bounds exactly 2 cut-face slots per surface sheet, every
  loop closes within tol, Euler check per face) and *repair at the level where the invariant broke*,
  never above it. The closure-weld cap bug (SEGWHOLE keep=0 holes) was an above-level repair.

### 2.12 TetWild / fTetWild ε-envelope — Hu, Zhou, Gao, Jacobson, Zorin, Panozzo (SIGGRAPH 2018/2020)
- <https://github.com/Yixin-Hu/TetWild> ; <https://dl.acm.org/doi/10.1145/3386569.3392385> ; <https://github.com/wildmeshing/fTetWild>
- **Core idea**: robustness by CONTRACT RELAXATION — output need only lie within an ε-envelope
  (default bbox-diag/1000) of the input; inside that freedom, the algorithm maintains a valid
  floating-point mesh at every stage (fTetWild) instead of exact rationals (TetWild), with equal
  robustness. Related: Diazzi–Attene convex polyhedral meshing (<https://arxiv.org/abs/2109.14434>).
- **Adopt (P5/P6 — conceptually the most important mesh-world paper for us)**: the ε-envelope IS
  the tolerance-based paradigm, formalized: **state the acceptance region explicitly, then design
  every stage to stay within it while preserving a validity invariant**. For P5: define, per entity,
  the envelope (vertex sphere, edge tube, face shell — Parasolid semantics) and prove/assert each
  algorithm stage keeps geometry inside its envelope. For P6: the corpus pass metric = structural
  validity + Hausdorff-within-envelope vs oracle (our vol-rel + naked gates are close; add explicit
  two-sided Hausdorff on section curves).

### 2.13 Boolean for CAD models via hybrid representation — Yang, Jia, Wang, Yang, Xin, Yan (SIGGRAPH/TOG 2025)
- <https://dl.acm.org/doi/10.1145/3730908>
- **Core idea**: **B-Rep booleans executed through a mesh proxy**: build a bijective mapping between
  the B-Rep and a triangle mesh with controllable approximation error; run a robust conservative
  mesh intersection to find ALL intersection-curve topology; map back to the B-Rep and compute the
  true curves there; guarantees watertight, correct-topology CAD output. Directly targets the
  OCCT/ACIS failure cases on near-critical freeform configurations.
- **Adopt (STRATEGIC — this is our lane)**: this legitimizes the architecture we've been converging
  on (mesh/scaffold as topological oracle + analytic/NURBS geometry as ground truth). Key deltas vs
  our current scaffold: (a) their mesh is *conservative* (guaranteed to find every intersection
  branch — solves our missed-tangent-circle class), (b) the mesh→B-Rep mapping is *bijective by
  construction*, so every mesh-level curve has a well-defined B-Rep home (our alias/key layer,
  made principled). Worth a full read + reimplementation notes; closest single paper to the
  rotated-chairs program.

### 2.14 Industrial floating-point + snapping — Landier (CAD 2017, Distene/MeshGems)
- <https://www.sciencedirect.com/science/article/abs/pii/S0010448516300847>
- Floating-point boolean on arbitrary polygonal/polyhedral meshes with snapping and conformal
  output; the honest industrial baseline: works broadly, no guarantee. Same family: Adaptive Mesh
  Booleans (Schmidt, <https://arxiv.org/abs/1605.01760>) — tolerance-driven mesh booleans with
  explicit handling of near-coincident geometry via welding, the mesh analog of our q6 weld.

---

## 3. B-Rep-specific robustness (the literature that talks our language)

### 3.1 Hoffmann's robustness corpus (1989–2001) — the problem statement
- Robustness in Geometric Computations, JCISE 2001: <https://cs.purdue.edu/cgvlab/www/publications/hoffmann2001robustness>
- Degeneracy & instability chapter: <https://link.springer.com/chapter/10.1007/978-0-387-35490-3_1>
- Robust set operations on polyhedral solids (Hoffmann–Hopcroft–Karasick 1989).
- **Content**: proves by example that naive ε-comparison cannot be globally consistent (ε-relations
  are not transitive); classifies remedies into exact computing vs fixed-precision-with-consistency;
  articulates the **minimal-independent-decisions principle**: choose a small set of primary
  topological decisions, derive everything else combinatorially, never decide the same fact twice
  numerically.
- **Adopt (P3/P4 audit tool)**: for each pipeline stage list which decisions are primary vs derived;
  every place two code paths can disagree about one fact (is this pave ON the trim? does this
  segment exist in operand B?) is a defect by this principle regardless of whether it has yet fired.

### 3.2 Segal 1990 — per-feature tolerances with consistency guarantees
- <https://dl.acm.org/doi/10.1145/97879.97891>
- **Core idea**: every vertex/edge/face carries its own tolerance; defines *topological consistency*
  for toleranced B-Reps; enforces **minimum-feature-separation invariants** (distinct features must
  be farther apart than the sum of their tolerances, else they are merged or the op is flagged);
  when data is ambiguous at the given tolerances the modeler *says so* rather than guessing.
- **Adopt (P5 core)**: the separation invariant is the missing formal backbone of our weld layer:
  after every weld/snap (q6, closure-weld, forced_node_eps, join_tol) assert no two surviving
  distinct entities are closer than their tolerance sum; if they are, merge deterministically or
  escalate tolerance — never leave the pair ambient. Most flood/quorum pathologies are downstream
  of ambient near-coincident pairs.

### 3.3 Jackson 1995 — Boundary representation modelling with local tolerances (Parasolid's tolerant modeling)
- <https://dl.acm.org/doi/10.1145/218013.218067> ; PDF: <https://ftp.cs.wisc.edu/pub/users/prem/jackson-SM-95.pdf>
- ACIS tolerant-modeling manual (same design): <http://www-isl.ece.arizona.edu/ACIS-docs/PDF/KERN/06TMOD.PDF>
- **Core idea**: THE industrial tolerance model. Edges = tubes of radius tol(edge), vertices =
  spheres of radius tol(vertex); coincidence/incidence tests are against these swept regions;
  tolerances attach to *topology* (edge/vertex), never to faces' surfaces (surface geometry stays
  authoritative); tolerances are introduced locally where data is imprecise (import, SSI approx)
  and **only ever grow**; downstream algorithms must accept toleranced entities everywhere.
- **Adopt (P5 blueprint — align with OCCT which copied this)**: (1) per-edge/per-vertex tolerance
  fields with grow-only updates and a recorded growth reason (import, SSI-approx, weld, sameparameter);
  (2) all incidence predicates take entity tolerance, not global tol — our current mix of diag-scaled
  constants (1e-2 alias keys, sew diag*5e-3, on_eps asymmetry) should collapse into per-entity
  tolerances consulted through one predicate layer; (3) pcurve-vs-3D-curve deviation (OCCT
  SameParameter) is the canonical tolerance-growth driver — measure it and store it, don't clamp it.

### 3.4 Fang & Brüderlin 1993 — intuitionistic tolerance-based robustness
- <https://www.sciencedirect.com/science/article/abs/pii/001044859390072V> ;
  survey chapter: <https://link.springer.com/chapter/10.1007/3-540-54891-2_7>
- **Core idea**: three-valued predicate logic (TRUE / FALSE / UNCERTAIN); UNCERTAIN answers are
  never silently coerced — they trigger either finer computation (symbolic/interval) or a
  consistency-preserving decision recorded in a dependency structure so later queries reuse it.
- **Adopt (P4)**: our predicates return bool today. A 3-state return (with the UNCERTAIN band =
  entity tolerance zone) at the few hot predicates (point-vs-trim ON test, pave-on-boundary,
  segment-in-face) plus a memoized decision table would turn silent flips into logged, reused,
  consistent decisions. This composes with indirect predicates: exact evaluation is the
  "finer computation" fallback for the UNCERTAIN band where geometry is linear/low-degree.

### 3.5 Interval solid modeling — Hu, Patrikalakis, Ye (CAD 1996 I/II) and descendants
- Part I representations: <https://www.sciencedirect.com/science/article/abs/pii/0010448596000139>
- Patrikalakis–Maekawa book bibliography: <https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node245.html>
- **Core idea**: replace floats with rounded interval arithmetic through the whole B-Rep (interval
  control points, interval curves/surfaces); incidence questions get certified answers; targets
  exactly the failure modes we log: topology violation (gaps/spurious intersections), incidence
  asymmetry, incidence intransitivity.
- **Verdict**: full-interval B-Reps never reached industry (intervals fatten under long dependency
  chains — wrapping effect), but the *certification of individual queries* survived and matured into:

### 3.6 Certified SSI topology — the 2023–2025 school (MOST DIRECTLY USEFUL CLUSTER)
- **IATA**: Topology-driven approximation of rational SSI via Interval Algebraic Topology Analysis —
  Cheng, Zhang, Xiao, Li (TOG 2023): <https://dl.acm.org/doi/10.1145/3592452>. 4D interval boxes in
  the two parameter domains; classifies SSI topology into 4 fundamental cases + mixtures; handles
  cusps, isolated tangent points, tangent curves, tiny loops, self-intersections — certified.
- **Topology-Guaranteed B-Spline SSI** — Yang, Jia, Yan (TOG/SIGGRAPH Asia 2023):
  <https://dl.acm.org/doi/10.1145/3618349>. Practical certified topology for B-spline SSI incl.
  multi-branch crossings, singular contacts, boundary-running intersections; **explicitly
  demonstrates OCCT and ACIS getting these wrong** (their failure gallery is a ready-made test set
  for us — slides: <https://yangjieyin.github.io/homepage/ACMTOG2023/SIGA23-slides.pdf>).
- **Watertight parametric SSI** — Wang, Jia, Yang, Wang, Bo, Liu (CGF 2025):
  <https://onlinelibrary.wiley.com/doi/10.1111/cgf.70298>. Makes the two pcurves and the 3D curve
  of an SSI *agree* (the trim-consistency problem; our pcurve/3D sync and sameparameter class).
- **Matrix-representation tracing** (Springer 2025): <https://link.springer.com/chapter/10.1007/978-981-96-5812-1_4> —
  robust SSI tracing via resultant/matrix reps.
- **Adopt (P-frontier: the rotated-chair SSI incompleteness IS this problem)**:
  1. Marching/newton SSI without topology certification will always have the tangential-stall and
     missed-loop classes; the fix is a **pre-pass that certifies the SSI curve-network topology**
     (count branches, loops, singular points within interval bounds) and *then* traces each certified
     component. Even a partial IATA (loop-existence test on subdivided interval boxes — a
     Krawczyk/Newton-exclusion test per box) would convert "missed tiny loop / stalled at tangency"
     from silent wrongness into a detected, subdividable event.
  2. Mine both TOG-2023 papers' failure galleries into session_tests as hard SSI cells (P6).

### 3.7 Watertight/gap-free trimmed-NURBS B-Reps (export-side robustness)
- Watertight trimmed NURBS — Sederberg et al. (SIGGRAPH 2008): <https://dl.acm.org/doi/10.1145/1360612.1360678>
- Watertight Boolean operations framework — Urick & Marussig et al. (CAD 2019):
  <https://www.sciencedirect.com/science/article/abs/pii/S0010448519302106>
- Reconstruction of trimmed NURBS for gap-free intersections — Urick, Crawford, Hughes, Cohen,
  Riesenfeld (JCISE 2020): <https://asmedigitalcollection.asme.org/computingengineering/article-abstract/20/5/051008/1084390>
- Making trimmed B-spline B-Reps watertight with a hybrid representation — Marussig et al. (2019):
  <https://www.researchgate.net/publication/337530530_Making_Trimmed_B-Spline_B-Reps_Watertight_With_a_Hybrid_Representation>
- Watertightization at intersection boundary (arXiv 2024): <https://arxiv.org/abs/2402.10216>
- **Core idea family**: after a boolean, *rework the surfaces* near intersection curves
  (reparameterize / refit patches) so the shared edge curve lies EXACTLY on both surfaces — trading
  surface fidelity within tolerance for genuine watertightness (vs tolerant edges which only declare it).
- **Adopt (STEP-export tier, not kernel core)**: our Rhino-acceptance battles (trim rejection,
  sliver segments, two-arc splits) are consumer-side symptoms; a post-boolean "watertightization"
  pass on exported faces (refit patch boundary to the section curve exactly, as in Urick 2020) is
  the literature-sanctioned fix and belongs next to the existing STEP compressor/merge_coplanar stage.

### 3.8 Surveys / meta
- **A survey of Boolean operations in 3D geometric modeling** (Computer-Aided Design, April 2026):
  <https://www.sciencedirect.com/science/article/abs/pii/S0010448526000515> — FIRST dedicated survey;
  mine its bibliography and taxonomy when accessible.
- Qiang Zou, A note on solid modeling: history, state of the art, future (arXiv 2023):
  <https://arxiv.org/abs/2302.14373> — positions robust B-Rep booleans as a still-open problem.
- Geometric interoperability via queries (CAD 2013): <https://dl.acm.org/doi/abs/10.1016/j.cad.2013.08.027> —
  tolerance-aware query semantics instead of translation; relevant to oracle-based validation.

---

## 4. Certified-numerics toolbox (cross-cutting)

| Tool | Source | Use in our kernel |
|------|--------|-------------------|
| Shewchuk adaptive-precision predicates (orient2d/3d, incircle) | <https://www.cs.cmu.edu/~quake/robust.html> | exact 2D decisions in UV arrangements (P3), radial sorts (P4) |
| Floating-point filters, modern | Bartels et al.: <https://arxiv.org/abs/2208.00497> | cheap certify-then-fallback wrappers for hot predicates |
| Indirect predicates lib | <https://github.com/MarcoAttene/Indirect_Predicates> | drop-in C++ header, MIT-friendly; LPI/TPI/LNC implicit points |
| Simulation of Simplicity | Edelsbrunner–Mücke 1990; impl guide <https://arxiv.org/abs/2212.08226> | deterministic tie-breaking for exactly-degenerate same-domain configs (P3) |
| Controlled perturbation | Halperin et al.: <https://link.springer.com/chapter/10.1007/978-3-642-15582-6_19> | last-resort input jitter for pathological corpus cells; document, don't default |
| Snap rounding / iterated snap rounding | Halperin–Packer 2002: <http://www.math.tau.ac.il/~danha/publications.html> | topology-preserving quantization; the principled version of our weld caps |
| Rounded interval arithmetic | Hu–Patrikalakis 1996 (§3.5) | SSI loop-exclusion tests, tolerance-growth certification (P5) |

---

## 5. Synthesis — ranked adoption list for our kernel

**Tier A (do; direct hits on current frontier)**
1. **Certified SSI topology pre-pass** (IATA + Topology-Guaranteed SSI, §3.6): interval
   subdivision + Newton-exclusion per UV-box to enumerate SSI branches/loops before marching.
   Kills the missed-tangent-circle and tangential-stall classes behind rotated-chair partial runs.
2. **Single-shared-section law completed to parameters** (CGAL corefinement §1.2, Lévy §1.4): both
   operands split at identical parameters on the shared curve, keys derived from provenance
   (face-pair, curve, branch), never from rounded coords (indirect-predicate doctrine §2.3).
   Directly targets the SEGLOST one-sided-split class.
3. **Cell-based winding-vector classification** (Zhou §2.1 + Barill §2.8): classify volumetric
   regions once with (w_A, w_B); all four ops derived; hierarchical winding as arbiter for
   flood/quorum ambiguity. Formalizes the already-validated tier-3 flux classifier.
4. **Per-entity grow-only tolerances with separation invariant** (Jackson §3.3 + Segal §3.2) as the
   P5 spec: tube/sphere semantics, one predicate layer consuming entity tolerance, recorded growth
   reasons, post-weld separation assertion. Replaces the scattered diag-scaled constants.

**Tier B (adopt as design law / audit)**
5. Dimension-ladder invariants with lowest-level repair (Mandoline §2.11) over the scaffold pipeline.
6. Canonical deterministic arrangements — unique CDT/lexicographic orderings (Lévy §1.4); no
   iteration-order dependence anywhere in the splitter.
7. Hoffmann minimal-independent-decisions audit (§3.1) + three-valued predicates at the ~5 hottest
   ON/incidence tests (Fang–Brüderlin §3.4).
8. Local-cone (sphere-map) classification at interference vertices for P4 EE/EF (Nef §1.1); radial
   face sort around section edges as combinatorial invariant.
9. ε-envelope acceptance contract for P6 (fTetWild §2.12): validity invariant + two-sided Hausdorff
   vs oracle per corpus cell; Manifold-style "no-caveat postcondition or fallback" (§2.9).

**Tier C (strategic / watch)**
10. Mesh-proxy B-Rep booleans with bijective mapping (TOG 2025 §2.13) — closest single paper to our
    scaffold architecture; read in full, harvest the conservative-intersection guarantee.
11. Export-side watertightization by surface refit at section curves (Urick/Marussig §3.7).
12. Iterated-CSG chain stress tests for tolerance-growth boundedness (§2.6).
13. Mine failure galleries: Topology-Guaranteed SSI (OCCT/ACIS failures), survey CAD-2026 refs (§3.8).

**Explicit non-adoptions (with reasons)**
- Full exact arithmetic B-Rep (Nef/ESOLID): NURBS SSI curves have no exact finite representation;
  cost 10–1000×; exactness stays predicate-local.
- Full interval B-Rep (Hu–Patrikalakis): interval fattening over op chains; intervals stay
  query-local (SSI certification, tolerance updates).
- Global controlled perturbation of inputs: incompatible with CAD ground-truth semantics
  (users' surfaces are contractual); allowed only as documented per-cell corpus workaround.

---
*Compiled 2026-07-24. All URLs verified via web search on that date.*
