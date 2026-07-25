# Theory of Tolerance-Based Geometric Computation — When Is a Tolerance Model Sound?

Companion to `occt_tolerance-model.md` (how OCCT implements it) and
`research_academic-robust-booleans.md` (strategy survey). This file answers ONE question from
first principles, with the primary literature: **under what conditions is a tolerance-based
topology model sound (free of contradiction cycles), what are its provable failure modes, and
what mitigation designs does the theory prescribe?**

Program context: P3 same-domain subsystem, P4 EE/EF interference, **P5 tolerance model**,
P6 corpus validation. Everything here feeds the P5 design spec.

---

## 0. The one-paragraph answer

A tolerance model replaces exact incidence with *zone incidence*: every topological entity
carries a region (sphere/tube/thickened patch) certifying "the true entity is in here." The
raw zone-coincidence relation is reflexive and symmetric but **not transitive** — that is the
root of every contradiction cycle. The theory (Segal 1990, Jackson 1995, Sakkalis–Patrikalakis
2000, Qi–Shapiro 2004/2006) converges on the same soundness condition, stated three ways:

1. **Combinatorial**: coincidence must be maintained as an *equivalence* (a partition built by
   explicit merging), never used as a raw pairwise predicate; a merge that grows a zone must
   trigger re-verification of previously-decided negatives (Segal's `near` values).
2. **Metric**: distinct entities must be separated by ≫ the sum of their tolerances (minimum
   feature separation); equivalently, tolerance must stay below the local feature size.
3. **Topological**: the fattened boundary must deformation-retract onto the true boundary —
   zone intersections contractible onto shared topology (Jackson), boundary cover confined to
   disjoint tubular neighborhoods (Sakkalis et al.), ε-regularity of the set interval
   (Qi–Shapiro).

When these hold, the model is sound in the *backward-error* sense (Fortune ε-stability): the
stored topology is the exact topology of SOME geometry within tolerance of the stored geometry.
When they fail, the theory says you must either **merge** (collapse features, growing
tolerance) or **refuse/re-verify** — there is no third option that preserves consistency.

---

## 1. Epsilon geometry — the predicate-level formalism (Guibas–Salesin–Stolfi 1989)

*Epsilon Geometry: Building Robust Algorithms from Imprecise Computations*, 5th ACM SoCG 1989.
- <https://dl.acm.org/doi/10.1145/73833.73857>
- PDF: <https://members.loria.fr/Sylvain.Lazard/BellairsWorkshops/Bellairs-2009/background/epsilon-geometry-guibas-salesin-stolfi.pdf>

**Definitions** (verified from the paper):
- For predicate P on metric space O: **ε-P(X)** ≡ "P(X′) is true for some X′ with ‖X,X′‖ ≤ ε"
  (X is at most ε away from satisfying P). Truth set of P "fattened" by ε. 0-P = P.
- **(−ε)-P(X)** ≡ "P(X′) true for ALL X′ within ε" — "extremely P" vs "nearly P".
- Monotone: ε-P(X) ⇒ ε′-P(X) for ε′ ≥ ε.
- An **epsilon-box** for P returns a partition of the ε-line into (False, Unknown, True)
  intervals, represented by a pair (e.lo, e.hi): definitely false if e.lo > 0, definitely true
  if e.hi ≤ 0, *don't know* between. The trivial always-Unknown box is correct — correctness
  and usefulness are decoupled.

**Composition laws** (the load-bearing results):
- ε-(P∨Q) ⇔ ε-P ∨ ε-Q, but ε-(P∧Q) **⇒ only** ε-P ∧ ε-Q — the converse FAILS: satisfying two
  constraints by separate ε-perturbations does not mean one perturbation satisfies both. This
  is the formal reason "each test passed individually" does not imply "the configuration is
  consistent" — the theoretical root of why per-pair tolerance tests can contradict globally.
- Chaining (Lemma 2): P ⇒ ε-Q and Q ⇒ δ-R gives P ⇒ (ε+δ)-R. **Uncertainty composes
  additively along inference chains** — this is tolerance creep at the logic level, before any
  geometry: an n-step derivation has an O(n·ε) warranty, not ε.

**Guarantee model**: algorithms compute an exact solution to a perturbed input and return a
bound ("warranty") on the perturbation — backward error analysis married to interval
arithmetic. This is exactly the guarantee a tolerant B-Rep kernel should advertise (§8, M8).

**Limits**: the paper works out only point/line primitives in the plane; already there,
maintaining tight epsilon-boxes through constructions is laborious. Yap's Handbook survey
(§6 below) classifies it "interval geometry" and notes nobody scaled it to full solid
modeling — the scaling substitute is the entity-tolerance model (§3–§5), which is epsilon
geometry with zones attached to *topology* instead of to every predicate evaluation.

---

## 2. Fortune's stability — the guarantee a tolerant kernel can actually promise

- S. Fortune, *Stable maintenance of point-set triangulations in two dimensions*, FOCS 1989.
- Fortune & Milenkovic, *Numerical stability of algorithms for 2D Delaunay triangulations /
  line arrangements* (stable arrangement algorithms), 1991–1995.
- Surveyed in Sharma & Yap, Handbook of DCG ch. 45: <https://www.csun.edu/~ctoth/Handbook/chap45.pdf>
- Shewchuk's adaptive exact predicates (the EGC counterpoint):
  <https://graphics.stanford.edu/courses/cs268-09-winter/manuals/robust-arithmetic.pdf>

**ε-stability** (Handbook ch. 45, following Fortune 1989): an algorithm computing a geometric
structure D = (G, λ, Φ, c) from input parameters c is **ε-stable** if it outputs a structure
that is *exactly consistent* for some parameter vector c′ with ‖c − c′‖ < ε. Strong stability:
ε = O(1) machine epsilons; linear stability: ε = O(n). Fortune/Milenkovic achieved provably
stable algorithms for triangulations and line arrangements — small domains, hard proofs.

**Parsimonious algorithms** (Fortune 1989, via ch. 45): evaluate a predicate ONLY if its value
is not already implied by previously evaluated predicates; derive, never re-decide. Yap notes
full parsimony "amounts to theorem proving" (deciding implication is reducible to the
existential theory of reals, NP-hard) — so real kernels enforce parsimony *structurally* (one
interference table, one shared section entity, derived classifications) rather than logically.
This is the academic name for our meta-law: **most historical chair bugs (asymmetric flood,
one-sided SEGLOST, alias-key misses) are parsimony violations — the same fact re-derived two
ways with two answers.**

**Doctrine**: a tolerance kernel cannot promise forward correctness ("the output equals the
true boolean"); it CAN promise ε-stability ("the output is the exact boolean of inputs moved
by ≤ ε, ε reported"). Segal's closure property and Jackson's tolerant booleans are exactly
claims of this shape.

---

## 3. Segal 1990 — the first complete tolerant-topology soundness design (read in full)

M. Segal, *Using Tolerances to Guarantee Valid Polyhedral Modeling Results*, SIGGRAPH 1990.
- <https://dl.acm.org/doi/10.1145/97879.97891>
- PDF: <https://ftp.cs.wisc.edu/pub/users/prem/segal-repair-siggraph-1990.pdf>
- Precursor: Segal & Séquin, *Consistent calculations for solids modeling*, SoCG 1985.

**Model**: each vertex/edge/face f has tolerance ε_f defining a region tol(f) around its flat
(point/line/plane), plus the regions of its boundary features. **Approximate intersection** of
two flats: distance ≤ ε₁ + ε₂ (SUM rule — same as OCCT/Parasolid, unlike ACIS §5). An object's
representation is **consistent** if every zone-intersection among features is explicitly in
the connectivity lists, and every connectivity is realizable from the metric data.

**Imposed restrictions** (§3.4 of the paper) — the soundness axioms:
1. Approximate alignment of equal-dimension features is DEMANDED to be symmetric and
   transitive (it is not automatically — Fig. 4 of the paper shows both failures). The model
   *enforces* it by merging.
2. Equal-dimension features never approximately coincide — coincidence is represented by a
   single merged feature (coincidence-as-partition, not as relation).
3. Feature tolerance regions may intersect in **at most one connected component** whose
   associated boundary regions also intersect (excludes the "two edges within tolerance at two
   separate places" ambiguity, Fig. 5).
4. Features near-aligned with a common higher-dimensional feature and mutually intersecting in
   projection must themselves be aligned (excludes ill-ordered vertices along an edge, Fig. 6).

**Merge rule**: merging f₁,f₂ → f requires tol(f₁) ∪ tol(f₂) ⊆ tol(f); coordinates chosen to
minimize the new tolerance subject to that containment (min enclosing sphere for vertices,
bounded cylinder for edges, Newell-averaged plane + max deviation for faces). Extent and
containing-feature tolerances updated upward transitively.

**The `near` value — the re-verification invention** (the paper's deepest mechanism): every
feature stores near(f) = the minimum distance to all features *tested and found disjoint* so
far. If any merge raises f's tolerance beyond near(f), a previously-made negative decision may
now be wrong → **backtrack: reset near values and rerun the whole algorithm**. Termination:
merges only reduce feature count, so recursion bottoms out (worst case: everything collapses
to one vertex). Cost: 5–10× over cursory checking even without backtracking; backtracking
worst case exponential but rare ("only when features approximately coincide AND other features
are near the coincident ones").

**Closure theorem (informal)**: given approximately-consistent inputs, the modeler always
terminates and outputs an approximately-consistent boundary — output can be fed back as input.
In most cases connectivities equal the infinite-precision result; in ambiguous cases small
complex regions are *collapsed into simpler ones* (accepted information loss instead of
contradiction).

**The canonical tolerance-creep experiment** (also cited by Yap ch. 45): iterated
rotate-and-union of two cubes. 8 iterations: tolerances imperceptible (no merging triggered).
9th: features merge, tolerances grow. 11th: **the object collapses to a single vertex**.
Zone growth is benign until first contact with the merge threshold, then compounds
catastrophically. Also documented: with face tolerances seeded at 10⁻⁶ and a high coplanarity
factor, computed vertex tolerances "engulf the original vertices" — creep is driven by the
*largest* seed tolerance in the pipeline, not the average.

**Segal's own mitigation list** (conclusions): report large tolerances to the user; a
post-processor that *decreases* tolerances while maintaining consistency (relaxation
minimizing a weighted sum of tolerances) — proposed 1990, still not standard in any kernel.

---

## 4. Jackson 1995 — Parasolid tolerant modelling formalism (read in full)

D. J. Jackson, *Boundary Representation Modelling with Local Tolerances*, ACM Solid Modeling 1995.
- <https://dl.acm.org/doi/10.1145/218013.218067>
- PDF: <https://ftp.cs.wisc.edu/pub/users/prem/jackson-SM-95.pdf>

**Model** (what Parasolid ships): tolerance attached to **topology, not geometry** — points,
curves, surfaces and derived objects (point-on-surface, trimmed curves) stay "exact" at
default tolerance. Vertex region = sphere; tolerant edge = represented by parameter-space
curves on each adjacent face (3-space curve dropped), region = tube of edge-tolerance radius
around ONE arbitrarily-chosen-but-fixed pcurve, all other pcurves must lie in the tube; face
region = surface thickened by face tolerance.

**Axioms**:
- **Nesting hierarchy**: vertex tolerance ≥ tolerance of every connected edge; edge tolerance
  ≥ tolerance of every connected face. (OCCT inherits exactly this; ends of pcurves must lie
  in the vertex sphere; face loops need not close point-wise, only within vertex tolerance.)
- **Test rule**: every intersection/coincidence test uses the SUM of the two topologies'
  tolerances.
- **Validity = contractibility** (the paper's key generalization of Segal's restriction 3):
  edges may intersect only at vertices — precisely, every connected component of the
  intersection of two edge tubes must intersect (be contractible onto) a common vertex of
  both; every connected component of two face-region intersections must be contractible onto
  a collection of common edges/vertices. Fig. 4: same geometric proximity is valid or invalid
  depending on whether the overlap retracts onto shared topology.
- **Resolution rule**: when a topological operation cannot be performed with existing
  tolerances "because entities collide, the local tolerances are increased, and redundant
  topology removed, until a consistent model can be created." Growth+merge is the ONLY escape
  hatch — same conclusion as Segal.

**Boolean architecture** (imprint/join/select — the direct ancestor of OCCT's
PaveFiller/Builder and of our BOP2):
- Imprint compares **low dimension first**: VV → VE → EE → VF → EF → FF; every split
  re-compares the pieces (smaller edges may now be coincident when the whole was not) — the
  paper's answer to what we rediscovered as SEGLOST/SEG-UNIFY.
- The FF intersector "need not consider curves already known to be common by virtue of
  edge-edge or edge-face comparison. **This can be important, since computation of
  near-tangent intersection curves can be very unstable**" — 1995 statement of our scaffold /
  shared-SSI-suppression doctrine, and of the P3 same-domain principle: *decide coincidence at
  the lowest dimension possible and propagate upward; never ask the SSI a question topology
  already answered.*
- Join: corresponding entities of different tolerances take the LARGER tolerance; topology
  compressed to 1–1 correspondence before joining.
- A vertex may correspond to an edge, a face, or a whole 3-space cell "if the tolerances so
  indicate" — dimension-collapsing correspondence is first-class.
- Short-edge lemma: with a single global tolerance you cannot split an edge shorter than
  **4× tolerance** without creating edges that are too short or fail to meet — the metric
  statement of why tolerance must stay below feature size.
- §7.3 *Preventing tolerance growth*: "Compression normally results in some growth of local
  tolerances. Ideally models defined to a certain tolerance should unite to create models of
  the same or similar tolerance. **Although this seems not to be achievable in all cases,
  tolerance growth should be contained where possible.**" — the vendor's own admission that
  closure-without-growth is an open problem, not an engineering detail.

**Current Parasolid doctrine** (Functional Description, *Session and Local Precision*):
- <http://www.q-solid.com/Parasolid_Docs_V35/chapters/fd_chap.017.html>
- Session precision 1.0e-8 (linear), 1.0e-11 rad (angular); size box 1000×1000×1000 (unit =
  meter ⇒ modeling range ±500 m; relative precision ~1e-11 at box edge).
- **Minimum-feature-separation guidance is normative**: vertices not intended coincident must
  be > 100× linear precision apart; non-parallel directions > 100× angular precision.
- **Transitivity is a validity condition**: "For a body to be valid, linear precision must be
  transitive" — three points with d(1,2), d(2,3) < eps but d(1,3) > eps ⇒ the BODY IS INVALID.
  Parasolid refuses the contradiction cycle rather than resolving it.
- Vertex precision automatically raised to ≥ max precision of adjacent edges (nesting enforced
  by construction). Setting local precision is classified as a *modeling operation* (changes
  topology: edges split at surface-parameterization discontinuities, SP-curves generated).
- Tolerance REDUCTION exists as a first-class workflow ("Improving local precision at tolerant
  vertices": closing gaps in periodic faces, extending/trimming SP-curves, reparameterizing
  the surface) — the productized version of Segal's relaxation post-processor.

---

## 5. ACIS tolerant modeling — the competing coincidence semantics

Kernel R10 ch. 6 *Tolerant Modeling*: <http://www-isl.ece.arizona.edu/ACIS-docs/PDF/KERN/06TMOD.PDF>
(mirrored ACIS docs; Spatial's current docs say the same).

- TEDGE/TVERTEX/TCOEDGE derived from exact classes; in a tolerant edge the **pcurves (TCOEDGE)
  hold the primary geometric definition** and the 3D curve is secondary/lazy — same inversion
  as Parasolid's SP-curves. Edge tolerance = max distance between equiparametric positions on
  its coedges; vertex tolerance = max distance from vertex point to adjacent edge ends.
- **Coincidence rule uses MAX, not SUM**: two tolerant entities are coincident if distance <
  max(tol₁, tol₂) (floored by SPAresabs). Contrast: Segal/Jackson/Parasolid/OCCT use
  tol₁+tol₂ (OCCT Boolean Ops guide: shapes interfere when distance ≤ sum of tolerances).
  Both are defensible (max ⇔ "one witness zone contains the other's nominal"; sum ⇔ "the
  zones overlap") — but a kernel MUST pick one and use it in every subsystem; a sum-rule
  intersector feeding a max-rule classifier manufactures contradiction cycles at entities
  whose tolerances differ by more than 2×. (P5 audit item: session_cpp currently mixes
  per-call epsilons; unify to one documented rule.)
- **Tolerances are system-maintained, never user-set** ("updated after each operation with no
  user input necessary; users may query, not set") — creep control by making growth an
  internal, audited event.
- Tested envelope: tolerances up to ~10% of model size still function — an explicit,
  published growth cap philosophy.
- Near-tangent doctrine: `res_near_tangent` angle; near-tangent edges are *deliberately made
  tolerant* so that "difficult intersections that are likely to be slow and/or fail are
  avoided between near coincident surfaces" — institutionalized avoidance of tangential SSI,
  identical in spirit to our scaffold gating of tangent circles.

---

## 6. Interval geometric computation — certified zones instead of assumed zones

**Hu–Maekawa–Patrikalakis (MIT), 1996–1997**:
- *Robust interval solid modelling Part I: representations*; *Part II: boundary evaluation*,
  CAD 28(10), 1996: <https://www.sciencedirect.com/science/article/abs/pii/0010448596000139>
- Robust interval algorithms for curve/surface intersections, CAD 1996/1997:
  <https://www.sciencedirect.com/science/article/abs/pii/S0010448596000991>
- Hyperbook ch. on interval methods: <https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node45.html>
- Explicit target failure modes (their words): **topology violation (gaps, inappropriate
  intersections), incidence asymmetry, incidence intransitivity** in floating-point B-Rep
  modelers — the same three failure modes Segal restricted away axiomatically; here they are
  attacked by *rounded interval arithmetic*: every curve/surface is an interval polynomial
  spline (coefficient boxes), so the zone is not a modeling assumption but a **certified
  enclosure**. Booleans on interval B-Reps for manifold + non-manifold objects.

**Sakkalis–Shen–Patrikalakis, *Topological and Geometric Properties of Interval Solid
Models*, Geometric Modeling / MIT memo 99-6 (2000)**:
- PDF: <https://ftp.cs.wisc.edu/pub/users/prem/for-prem/directly/Sakkalis-Shen-Patrikalakis-2001-draft-GM.pdf>
- THE soundness theorem of the zone approach. Setup: solid M, finite collection B of
  axis-aligned boxes covering ∂M; interval solid M^B = M ∪ B. Conditions (paraphrased):
  (A) pairwise box intersections are boxes (well-behaved cover); (C1–C3) each box meets
  exactly one boundary component, boxes of different components lie in **disjoint (tubular)
  neighborhoods**, and boxes are small relative to those neighborhoods.
  **Theorem 2.5 / Corollary 2.6: under these conditions M^B (and M−B) is a solid
  homeomorphic to M** — "approximate equality" of the interval solid and the true solid.
- Read as doctrine: a tolerance model is sound exactly while the fattened boundary is
  ambient-isotopic to the true boundary, and that holds iff **zone thickness < the reach
  (normal-injectivity radius) of the boundary and < half the separation between distinct
  boundary parts**. Tolerance creep is precisely the monotone consumption of this margin;
  the moment a zone bridges two boundary sheets (thin wall, near-tangency), homeomorphism —
  and with it every classification — is forfeit. This is the theorem behind the empirical
  OCCT advice "fuzzy must be well below minimum feature size."

---

## 7. ε-Solidity — validity of tolerant models as a theory (Qi–Shapiro–Stewart)

- Qi & Shapiro, *ε-Solidity in Geometric Data Translation*, SAL TR 2004:
  <https://spatial.engr.wisc.edu/wp-content/uploads/sites/715/2014/04/2004-2.pdf>
- Qi & Shapiro, *ε-topological formulation of tolerant solid modeling*, CAD 38(4), 2006:
  <https://dl.acm.org/doi/10.1016/j.cad.2005.10.010>
- Qi & Shapiro, *Geometric Interoperability with Epsilon Solidity*, JCISE 6(3), 2006:
  <https://asmedigitalcollection.asme.org/computingengineering/article-abstract/6/3/213/654785/>
- Related: Shapiro & Stewart single-set vs class-of-sets semantics; Qi–Shapiro
  *Epsilon-regular sets and intervals*:
  <https://www.semanticscholar.org/paper/Epsilon-regular-sets-and-intervals-Qi-Shapiro/43a2eb34c6cf9543f660fa0f31531a32b1947dcb>

**The formalism** (definitions verified from the 2004 paper):
- **ε-closure** k_ε(X): points x with B(x,r) ∩ X ≠ ∅ for every r > ε. **ε-interior** i_ε(X):
  points with some B(x,r) ⊆ X, r > ε. ε-boundary ∂_ε = k_ε(X) ∩ k_ε(complement). Dual:
  i_ε(X) = c k_ε c (X). Monotone in ε (Thm 3.7): larger ε shrinks interior, grows closure,
  thickens boundary; ε→0 recovers classical topology.
- **ε-regular set** (Def 3.8): i_ε k₀(X) ⊆ X ⊆ k_ε i₀(X) — Requicha's r-set condition
  X = ki(X) relaxed to "errors allowed within ε of the boundary."
- **ε-regular interval** (Def 3.9): [X⁻, X⁺] with i_ε(X⁺) ⊆ X⁻ ⊂ X⁺ ⊆ k_ε(X⁻) — the
  set-interval (inner/outer bound) formulation; subsumes 3.8 (Thm 3.10).
- **ε-solid** (Def 3.12): ε-regular interval with X⁻ nonempty and X⁺ bounded.
- **Subinterval theorem** (Thm 3.11/3.13, "of paramount practical significance"): any
  subinterval of an ε-regular interval is ε-regular — so you can certify an uncomputable
  model by certifying a computable containing interval [i_δ(X), k_δ(X)].
- **Validity redefined** (Def 4.1): a representation is valid **relative to the evaluation
  algorithm**: (R, PMC_δ) is valid if applying the point-membership classifier at precision δ
  induces an ε-solid. Validity is a property of the *pair* (data, interpreting algorithm) —
  not of the file. This is the formal reason a model can be "valid" in the sending kernel and
  garbage in the receiving one.
- **Translation theorems**: tightening precision by Δ preserves ε-solidity with
  ε′ = ε − 2Δ; loosening gives ε′ = ε + 2Δ — tolerance budgets compose linearly across
  systems, and there is a floor below which retightening is impossible without remodeling.
- **Punchline the authors emphasize**: current B-Rep validity checking is "neither necessary
  nor sufficient" for maintaining ε-solidity under numerical inaccuracy, and geometric
  healing "may be avoided in many common situations" — check ε-solidity (a semantic,
  PMC-level property), not syntactic gap thresholds.

**Fang–Brüderlin–Zhu** (*Robustness in solid modelling: a tolerance-based intuitionistic
approach*, CAD 25(9), 1993): <https://www.sciencedirect.com/science/article/abs/pii/001044859390072V>
and Springer chapter *Robustness in geometric modeling — Tolerance-based methods*:
<https://link.springer.com/chapter/10.1007/3-540-54891-2_7>. Intuitionistic (constructive)
three-valued logic over toleranced data: relations are computed from tolerances, only
*definitely-true* and *definitely-false* verdicts are asserted, and tolerances are
**dynamically updated so the asserted relations keep their theoretical properties**
(symmetry, transitivity) — "self-validation"; proved robust for booleans on planar/quadric
solids. The bridge between epsilon-boxes (§1) and kernel tolerance models: the tri-state
verdict is what a predicate returns; the tolerance update is what makes the two-valued
projection of it consistent.

---

## 8. Yap's synthesis — where tolerance models sit in the robustness taxonomy

Sharma & Yap, *Robust Geometric Computation*, Handbook of DCG ch. 45:
<https://www.csun.edu/~ctoth/Handbook/chap45.pdf>

- Four models of a "finite-precision line": interval geometry (fattened zones — Segal/Séquin),
  topological distortion (Greene–Yao polylines, Milenkovic), rounded geometry (Sugihara's
  bounded-coefficient lines), discretization. Entity-tolerance kernels are **interval
  geometry**; their verdict: "in order to obtain correct predicates, they enforce minimum
  feature separations. To do this, features that are too close must be merged (or pushed
  apart)." — the Handbook states our soundness condition as the definition of the approach.
- Zone-blowup warning, citing Segal's experiment: "In applications where zones expand rapidly,
  there is danger of the zone becoming catastrophically large."
- Epsilon-tweaking (ad-hoc per-call epsilons, no maintained invariant) is dismissed as
  unsound — it is NOT the same thing as a tolerance *model*; the difference is precisely the
  maintained invariants (S1–S6 below). Kettner et al., *Classroom examples of robustness
  problems in geometric computations* (CGTA 2008)
  <https://people.mpi-inf.mpg.de/~mehlhorn/ftp/classroom-examples.pdf> supplies the concrete
  disasters for naive epsilon use.
- Topology-oriented computing (Sugihara–Iri, *Topology-oriented implementation*, Algorithmica
  27:5–20, 2000): make the COMBINATORIAL invariant primary and force numerics to conform —
  never contradictory, possibly geometrically distorted. OCCT's "keep the shape valid, grow
  tolerance to cover the lie" is a half-adoption: it preserves local validity certificates
  but has no global invariant like planarity-of-graph to anchor to.
- ε-stability (§2) is "a metric form of topological distortion ... analogous to backward
  error analysis"; stability is STRONGER than topological consistency (Sugihara's Voronoi
  codes are consistent but not proved stable).

---

## 9. OCCT's tolerance model measured against the theory

Primary docs / community evidence:
- Boolean Operations user guide (fuzzy value, interference = distance ≤ sum of tolerances,
  growth formulas like Tol(V) := max(Tol(V), D + Tol(E))):
  <https://dev.opencascade.org/doc/occt-7.4.0/overview/html/occt_user_guides__boolean_operations.html>,
  <https://github.com/Open-Cascade-SAS/OCCT/wiki/boolean_operations>
- Shape Healing guide: <https://dev.opencascade.org/doc/overview/html/occt_user_guides__shape_healing.html>
- Forum: *Boolean operations: in search for a robust process*
  <https://dev.opencascade.org/content/boolean-operations-search-robust-process>;
  *Tolerance issues* <https://dev.opencascade.org/content/tolerance-issues>;
  *Shape tolerance* <https://dev.opencascade.org/content/shape-tolerance>;
  *Fixing tolerances inside a shape* <https://dev.opencascade.org/content/fixing-tolerances-inside-shape>;
  *how to guess the fuzzy value* <https://dev.opencascade.org/content/how-guess-fuzzy-value-boolean-operations>
- Analysis Situs tolerance checker notes: <https://www.analysissitus.org/features/features_check-toler.html>

What OCCT got right per the theory: Jackson nesting invariant (vertex covers curve ends, edge
tolerance covers 3D-curve/pcurve divergence — enforced by BRepCheck and ShapeAnalysis_Edge);
sum-rule interference used uniformly in BOPAlgo; measured-then-floored tolerance growth (grow
by what was actually observed, not by a preset); fuzzy value as **query-time additive
widening handed to one operation** rather than persisted growth — a clean one-shot version of
"increase tolerance until consistent."

Documented gaps (the community's standing critiques, all predicted by the theory):
- **Monotone growth, no reduction**: "the edge tolerance is never decreased" (ShapeFix docs);
  no productized analog of Parasolid's precision-improvement or Segal's relaxation. Users
  report post-fuse tolerances of 1.2 model units and vertices drifting 1.9 mm
  (*Tolerance issues* thread) — creep with no recovery path.
- **No re-verification**: nothing like Segal's `near` values; a tolerance raised in
  PaveFiller stage k never re-opens a disjointness verdict from stage k−2 within the same
  operation, and never across operations. Contradictions surface later as "self-intersecting"
  warnings or invalid results (forum: incremental fuse degrading at the 21st solid).
- **No separation enforcement**: OCCT accepts bodies whose feature separation is below the
  sum of tolerances; BRepCheck checks per-entity certificates (nesting) but not Segal
  restriction 3 / Jackson contractibility (no "two entities within tolerance at two disjoint
  places" test). Fuzzy guidance ("must be well below min edge length, group solids by feature
  size" — Eugeny on the robust-process thread) is folklore where Parasolid has a normative
  100× rule.
- Healing by tolerance increase (ShapeFix with ModifyGeometryMode=false) fixes certificates
  by consuming separation margin — exactly the trade the interval-solid theorem warns about.

---

## 10. Synthesis I — WHEN a tolerance model is sound

A tolerance model is a claim: *"there exists an exact configuration, within each entity's
zone, whose exact topology is the stored topology"* (S1, the backward-error semantic). The
literature yields six conditions; together they are sufficient, and each has a named
counterexample when dropped:

- **S1 — Containment (zone honesty)**: every zone contains its true entity; derived entities
  get zones covering the derivation error (Segal: intersection-line tolerance formula covers
  tol(a₁)∩tol(a₂); OCCT: measured D + representation tolerance). Drop it → zones lie,
  classification undecidable. [Segal §4; Hu-Patrikalakis certified version]
- **S2 — Nesting (boundary coherence)**: tol(vertex) covers ends of all adjacent edge
  curves; tol(edge) covers 3D-curve/pcurve divergence and lies within adjacent faces'
  thickened surfaces. Drop it → an edge's own endpoints disagree with it. [Jackson §3.1;
  Parasolid auto-raise; OCCT BRepCheck]
- **S3 — Separation (minimum feature separation)**: distinct-and-not-corresponding entities
  are farther apart than the coincidence threshold by a safety factor (Parasolid: 100×;
  Segal: enforced by merging violators; interval-solid theorem: zones inside disjoint tubular
  neighborhoods ⇒ ε < reach and ε < ½·gap). Drop it → transitivity failures are reachable and
  some pair must eventually be decided both ways. **This is the load-bearing condition: raw
  zone-coincidence is not transitive; separation is what makes its transitive closure stay
  within clusters.** [Segal Fig. 4b; Parasolid validity rule; Sakkalis Thm 2.5]
- **S4 — Contractibility (single-witness overlap)**: every connected component of a
  zone-zone intersection retracts onto shared topology; at most one component per
  non-adjacent pair. Drop it → "coincident here, distinct there" along one entity pair
  (Jackson Fig. 4; Segal Fig. 5/6 restrictions).
- **S5 — Parsimony (decision single-sourcing)**: each independent incidence decided exactly
  once; all dependent facts derived from the decision store; the same question never posed to
  two different numerical procedures. Drop it → contradiction without any tolerance event at
  all. [Fortune parsimonious algorithms; Jackson's "FF intersector must not recompute known
  coincidences"; our flood/SEGLOST history]
- **S6 — Closure (inductive soundness)**: the operation's output satisfies S1–S5 again, so
  chains of operations stay sound; growth events that would violate S3 must trigger merge or
  re-verification, not be ignored. [Segal's closure theorem + near-value backtracking]

Equivalent one-line criterion (the topological compression of S1–S4): **the union of all
zones, viewed as a thickened boundary complex, must be ambient-isotopic to the boundary of
some exact solid, and the stored topology must be that solid's topology.** (Sakkalis et al.
homeomorphism theorem; Qi–Shapiro ε-regularity is the same statement in point-set form.)

Soundness is therefore **conditional and perishable**: every operation re-spends the margin
between tolerance and local feature size. A sound kernel is one that (a) knows the margin,
(b) refuses or merges when it is exhausted, (c) never silently continues.

---

## 11. Synthesis II — the failure-mode catalog

- **F1 Tolerance creep (zone inflation)**: monotone growth compounding across operations;
  benign below the merge threshold, catastrophic after first contact (Segal's 8-then-collapse
  iterations; OCCT never-decrease rule + forum drift reports; Yap "catastrophically large").
  Logic-level version: warranties add along inference chains (GSS Lemma 2).
- **F2 Transitivity violation**: A≈B, B≈C, A≉C. The defining contradiction cycle. Surfaces
  as: vertex-cluster chains (Segal Fig. 9), coincidence chains along near-tangent seams,
  Parasolid invalid-body verdicts, Hu's "incidence intransitivity." Only cures: merge the
  chain's cluster (grow) or keep clusters separated (S3).
- **F3 Asymmetry**: approximate alignment/ON tests that are not symmetric (B within tol of A
  but not conversely — Segal Fig. 4a; Hu's "incidence asymmetry"; our historical on_eps
  asymmetry and asymmetric flood-coverage bugs). Any predicate whose answer depends on
  argument order is a latent contradiction generator; symmetrize by construction (test with
  ε₁+ε₂, or always canonical-order the pair).
- **F4 Retroactive invalidation**: a merge/growth event enlarges a zone past the distance of
  a previously-decided disjointness (Segal's near-value trigger). Kernels without the trigger
  (OCCT, us today) carry the stale negative silently to the output.
- **F5 Tolerance ≥ feature size**: zone engulfs features — unsplittable short edges
  (< 4× tol, Jackson), fuzzy > min edge length making "many solids invalid" (OCCT forum),
  zones bridging thin walls (loss of homeomorphism, Sakkalis), full collapse (Segal). The
  hard ceiling is the boundary's reach / local feature size.
- **F6 Interpretation mismatch (interoperability)**: validity is a property of
  (data, algorithm-precision) pairs (Qi–Shapiro Def 4.1); the same B-Rep classifies ON/OFF
  differently in sender and receiver; retightening costs 2Δ of ε-budget and has a floor.
  Also intra-kernel: sum-rule vs max-rule subsystems (§5) are two "receivers" inside one
  executable.
- **F7 Representation divergence**: multiple geometric representatives of one entity (3D
  curve vs pcurves; nominal vs SP-curves) drifting beyond the tolerance that is supposed to
  bind them (OCCT SameParameter/SameRange machinery and its failure modes; ACIS makes pcurves
  primary to kill this class).
- **F8 Zone conservatism / false merging**: overestimated tolerances trigger merges that
  destroy intended small features (Segal: "too conservative ... no access to global
  relationships"; exact-arithmetic critique in Jackson §2.1: creates features with no design
  intent — same failure, opposite sign).
- **F9 Ad-hoc epsilons (tweaking)**: per-call thresholds with no maintained invariant — not a
  tolerance model at all; the Handbook/Kettner failure catalog applies. Any raw `< 1e-7` in a
  kernel decision path is an F9 seed.

---

## 12. Synthesis III — mitigation designs (ranked for P5)

1. **M1 Tolerance hierarchy as enforced invariant** (Jackson/Parasolid/OCCT): vertex ⊇ edge
   ⊇ face containment, auto-raised on construction, audited by a checker that is part of the
   pipeline (BRepCheck-style), not an offline tool. Cheap, foundational, already
   half-standard. *P5: make the nesting audit a debug-build postcondition of every mutator.*
2. **M2 Growth caps tied to local feature size** (from Sakkalis theorem + ACIS 10% envelope +
   OCCT folklore): compute/maintain per-region minimum feature separation (min adjacent edge
   length, min wall thickness proxy); define budget β = κ·separation (κ ≈ 0.01–0.1); any
   growth event exceeding β is an ERROR (refuse/diagnose), not a warning. Creep becomes
   detectable the moment it starts, not at collapse. *P5: wire into the two growth mutators
   (we already mirror OCCT's "only two mutators" doctrine — see occt_tolerance-model.md §10).*
3. **M3 Negative-decision ledger + targeted re-verification** (Segal's `near` values,
   modernized): record every disjointness verdict with its measured margin; on any tolerance
   growth of entity f, re-open exactly the verdicts whose margin < new tolerance (spatial
   index makes this near-free); rerun affected local stages instead of Segal's full restart.
   This is THE missing feature in OCCT and the highest-leverage novel mechanism available to
   us. *P5 flagship.*
4. **M4 Tolerance reduction pass** (Segal's relaxation proposal + Parasolid's productized
   gap-closing): after an op, re-intersect exact geometry where possible (Jackson: extend and
   re-intersect non-tangent edges), re-fit pcurves, shrink certificates to measured values.
   Prevents budget exhaustion across chained operations — the difference between "survives
   one boolean" and "survives eleven" (Segal's experiment is the acceptance test: iterate
   rotate-union until collapse; count iterations). *P6 corpus should include exactly this
   iterated-CSG stress.*
5. **M5 Separation enforcement / merge-or-refuse discipline** (Segal restrictions,
   Parasolid transitive-precision validity): when zones of distinct entities overlap beyond
   S4-contractibility, either merge (deliberate regularization, tolerance grows, ledger
   fires) or reject with a diagnosis — never proceed with the raw non-transitive relation.
   Micro-feature collapse becomes an intentional, logged act.
6. **M6 Tri-state predicates on hot decisions** (epsilon-boxes; Fang intuitionistic;
   filtered predicates): predicates return definite-true/definite-false/uncertain with
   margins; uncertain routes to higher precision (interval/exact fallback — Fortune–van Wyk
   filters, Shewchuk) or to a topology-driven tie-break, never to a coin-flip comparison.
   Margins are what M3's ledger stores.
7. **M7 One coincidence algebra**: a single documented rule (recommend OCCT/Parasolid **sum**
   rule) used by every subsystem; all epsilons derived from entity tolerances, zero free
   per-call constants (kills F9); symmetric by construction (kills F3).
8. **M8 The stability contract** (Fortune/GSS): the kernel's public guarantee is ε-stability
   — "result = exact boolean of inputs perturbed ≤ ε_out, ε_out reported" where ε_out = max
   final tolerance. Report it; let callers gate on it (Segal: "it makes sense to report large
   tolerances so that a user or higher-level program can be alerted"). Makes creep observable
   at the API instead of a latent lie.
9. **M9 Semantic validity checking** (Qi–Shapiro): validity checks must use the kernel's own
   PMC at its own precision (does the model induce an ε-solid?), not syntactic gap
   thresholds; conversely don't "heal" models whose ε-solidity is intact (healing is neither
   necessary nor sufficient). Our oracle-differencing (OCCT/Rhino verdicts) is already an
   instance; add a self-PMC consistency probe (random rays vs stored topology) to P6.

---

## 13. Program mapping (P3–P6)

- **P3 same-domain**: Jackson's imprint doctrine is the spec — lowest dimension first,
  re-compare after every split, and the FF stage must consume (not recompute) EE/EF
  coincidences; near-tangent SSI avoidance is 30-year-old industry practice, our scaffold
  gating is the right instinct. Segal's dimension-collapsing correspondences (vertex ↔ edge ↔
  face) are the formal model for same-domain merge bookkeeping.
- **P4 EE/EF interference**: sum-rule uniformly; symmetric predicates (F3); every
  interference verdict written once to the decision store with margin (M3, S5).
- **P5 tolerance model**: adopt S1–S6 as the spec sheet; implement M1 (audit), M2 (feature-
  size-linked caps), M3 (ledger + re-verification — the novel piece), M7 (one algebra),
  M8 (reported ε-stability); M4 reduction pass as stretch goal — no shipping kernel has
  Segal relaxation done right, it is a differentiator.
- **P6 corpus validation**: add (a) Segal's iterated rotate-union creep benchmark (score =
  iterations to first merge / to collapse), (b) minimum-feature-separation sweep (shrink gap
  until failure; sound kernels fail by *refusal*, unsound by contradiction), (c) mixed-
  tolerance interop set (import at 1e-4, model at 1e-7 — Qi–Shapiro retightening floor),
  (d) self-PMC ε-solidity probe on every result.

---

## 14. Sources

Primary papers (read in full for this report):
- Segal, *Using Tolerances to Guarantee Valid Polyhedral Modeling Results*, SIGGRAPH 1990 — <https://dl.acm.org/doi/10.1145/97879.97891> / <https://ftp.cs.wisc.edu/pub/users/prem/segal-repair-siggraph-1990.pdf>
- Jackson, *Boundary Representation Modelling with Local Tolerances*, SM 1995 — <https://dl.acm.org/doi/10.1145/218013.218067> / <https://ftp.cs.wisc.edu/pub/users/prem/jackson-SM-95.pdf>
- Guibas–Salesin–Stolfi, *Epsilon Geometry*, SoCG 1989 — <https://dl.acm.org/doi/10.1145/73833.73857> / <https://members.loria.fr/Sylvain.Lazard/BellairsWorkshops/Bellairs-2009/background/epsilon-geometry-guibas-salesin-stolfi.pdf>
- Qi–Shapiro, *ε-Solidity in Geometric Data Translation*, 2004 — <https://spatial.engr.wisc.edu/wp-content/uploads/sites/715/2014/04/2004-2.pdf>
- Sakkalis–Shen–Patrikalakis, *Topological and Geometric Properties of Interval Solid Models*, 2000 — <https://ftp.cs.wisc.edu/pub/users/prem/for-prem/directly/Sakkalis-Shen-Patrikalakis-2001-draft-GM.pdf>
- Sharma–Yap, *Robust Geometric Computation*, Handbook DCG ch. 45 — <https://www.csun.edu/~ctoth/Handbook/chap45.pdf>
- ACIS Kernel R10 ch. 6, *Tolerant Modeling* — <http://www-isl.ece.arizona.edu/ACIS-docs/PDF/KERN/06TMOD.PDF>
- Parasolid Functional Description, *Session and Local Precision* — <http://www.q-solid.com/Parasolid_Docs_V35/chapters/fd_chap.017.html>

Secondary / abstracts / docs:
- Fang–Brüderlin–Zhu, *Robustness in solid modelling: a tolerance-based intuitionistic approach*, CAD 1993 — <https://www.sciencedirect.com/science/article/abs/pii/001044859390072V>; Springer chapter — <https://link.springer.com/chapter/10.1007/3-540-54891-2_7>
- Hu–Maekawa–Patrikalakis, *Robust interval solid modelling I/II*, CAD 1996 — <https://www.sciencedirect.com/science/article/abs/pii/0010448596000139>; surface intersections — <https://www.sciencedirect.com/science/article/abs/pii/S0010448596000991>; MIT hyperbook — <https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node45.html>
- Qi–Shapiro, *ε-topological formulation of tolerant solid modeling*, CAD 2006 — <https://dl.acm.org/doi/10.1016/j.cad.2005.10.010>; *Geometric Interoperability with Epsilon Solidity*, JCISE 2006 — <https://asmedigitalcollection.asme.org/computingengineering/article-abstract/6/3/213/654785/>
- Sugihara–Iri et al., *Topology-oriented implementation*, Algorithmica 2000 (via ch. 45 refs)
- Fortune, *Stable maintenance of point-set triangulations*, FOCS 1989; Fortune–van Wyk filters; Shewchuk adaptive predicates — <https://graphics.stanford.edu/courses/cs268-09-winter/manuals/robust-arithmetic.pdf>
- Kettner–Mehlhorn–Pion–Schirra–Yap, *Classroom examples of robustness problems*, CGTA 2008 — <https://people.mpi-inf.mpg.de/~mehlhorn/ftp/classroom-examples.pdf>
- Hoffmann, *The problems of accuracy and robustness in geometric computation*, IEEE Computer 1989; Hoffmann–Hopcroft–Karasick, *Robust set operations on polyhedral solids*, IEEE CG&A 1989 (via Segal/Jackson refs)
- OCCT: Boolean Operations guide — <https://dev.opencascade.org/doc/occt-7.4.0/overview/html/occt_user_guides__boolean_operations.html>; Shape Healing — <https://dev.opencascade.org/doc/overview/html/occt_user_guides__shape_healing.html>; forum threads — <https://dev.opencascade.org/content/boolean-operations-search-robust-process>, <https://dev.opencascade.org/content/tolerance-issues>, <https://dev.opencascade.org/content/shape-tolerance>, <https://dev.opencascade.org/content/fixing-tolerances-inside-shape>, <https://dev.opencascade.org/content/how-guess-fuzzy-value-boolean-operations>
- Analysis Situs tolerance checking — <https://www.analysissitus.org/features/features_check-toler.html>
- Schirra, *Robustness and Precision Issues in Geometric Computation* — <https://pure.mpg.de/rest/items/item_1819517_4/component/file_2599039/content>
