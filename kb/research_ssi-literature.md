# research_ssi-literature

Surface-surface intersection (SSI) literature survey — marching, subdivision, algebraic/resultant
methods, loop-detection guarantees, tangential/singular handling, seam/pole treatment — with
concrete algorithms mapped to our single-seed marcher (`build_section_scaffold` /
freeform marcher in `brep_section.cpp` / `brep.cpp`) and the master-plan phases
(P3 same-domain, P4 EE/EF, P5 tolerance model, P6 corpus validation).
Companion to `kb/occt_ssi-walking.md` (the OCCT industrial baseline, extracted from source).

---

## 0. OUR MARCHER'S WEAKNESS INVENTORY (what the literature must answer)

- **W1 — single-seed / no branch guarantee.** Seeds come from triangulated pre-intersection
  (IntPolyh-style mesh section polylines). Any loop or branch smaller than mesh resolution, or
  disconnected from a seeded polyline, is silently missed. No collinear-normal / Gauss-map / winding
  test exists to certify "all components seeded".
- **W2 — order sensitivity.** The freeform marcher can find sections one way round and not the other
  (`brep.cpp` ~L8016–8060 retries swapped operands).
- **W3 — graze/tangency stalls.** The Newton corrector is near-singular where normals are
  near-parallel; stalled marches are "completed" by minimization, not marching
  (`newton_cc` stalls → 0.008 spurious splits, partial runs); whole tangent branches can be lost
  (cone×cone tangent circle needed the scaffold-eligibility carve-out).
- **W4 — heuristic termination.** Endpoint gaps bridged with `tol3 = max(tol*50, diag*5e-4)`
  (~L3136) instead of guaranteed boundary landing; graze-distance paves and junction caps at
  1.3e-1 are compensations for missing exact border points.
- **W5 — seam/pole.** Closed-surface seams are handled by special cases (tangent circles "need
  seam handling"); no systematic seed duplication across the period, no pole-adjacent step policy.
- **W6 — no closure/topology certificate.** Nothing proves a traced network has the right number of
  loops/segments; downstream flood-fill quorums absorb the ambiguity.
- **W7 — pp-curve consistency / error bound.** uvB is re-seeded from normalized fractions then
  re-projected (F1 parameterization mismatch, ~L445–450); no verified bound linking the two
  parameter-space preimages to the 3D curve (watertightness gap).

---

## 1. TAXONOMY (surveys)

Four classical families — **lattice/grid**, **marching (tracing)**, **subdivision
(divide-and-conquer)**, **algebraic/analytic** — plus modern **hybrid topology-first** methods.
Marching dominates industrial kernels (ease of implementation, generality across offsets/blends);
its two canonical failure modes are (a) finding a start point on *every* component and
(b) stepping across closely spaced features ("component jumping" / straying).

- Patrikalakis–Maekawa–Cho, *Shape Interrogation for CAD/CAM* (free hyperbook), ch. 5 intersections,
  ch. 4 solvers, ch. 6 differential geometry of intersection curves:
  https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node115.html (ch. 6 intro),
  https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node109.html (5.8.2.3 marching).
- Dokken et al., "Intersection Algorithms and CAGD" (SISL lineage; recursive subdivision +
  Sinha-type loop elimination): https://link.springer.com/chapter/10.1007/978-3-540-68783-2_3
- "Challenges in Surface-Surface Intersections":
  https://link.springer.com/chapter/10.1007/3-540-27157-0_2
- Sederberg et al. survey, "A survey of intersection algorithms for curved surfaces":
  https://www.sciencedirect.com/science/article/abs/pii/009784939190037I
- CAD-journal overview "Surface to Surface Intersections" (2004):
  https://www.cad-journal.net/files/vol_1/CAD_1(1-4)_2004_449-457.pdf
- Wikipedia summary: https://en.wikipedia.org/wiki/Surface-to-surface_intersection_problem

---

## 2. MARCHING METHODS

### 2.1 Barnhill–Kersey 1990 — the canonical relaxation marcher
CAGD 7:257–280. https://www.sciencedirect.com/science/article/abs/pii/016783969090035P
- Pure evaluation-based (needs only positions + tangents, no surface equations). Predictor: step
  along t = N1×N2; corrector: **alternating relaxation** — project the predicted point onto S1
  (closest point), then onto S2, iterate until the pair converges under tolerance. This is the
  ancestor of our `newton_cc`-style corrector.
- **Boundary relaxation**: explicit procedure to relax a curve endpoint *onto the domain boundary*
  (solve for the boundary iso crossing), rather than stopping at the last interior point — the
  literature answer to W4.
- Handles triangular domains, tangent tracks and branch points (with special-case logic, not
  guarantees). Start points from a coarse lattice + subdivision-refined candidates.

### 2.2 Bajaj–Hoffmann–Lynch–Hopcroft 1988 — higher-order predictor + singular tracing
CAGD 5:285–307. https://www.sciencedirect.com/science/article/abs/pii/0167839688900106
- Predictor is a **third-order Taylor approximant** of the curve (from successive differentiation of
  the intersection system), giving variable step length far beyond tangent-line stepping; corrector
  is Newton on the full system. Step length chosen from the local Taylor error term — i.e.
  curvature-adaptive step control with an analytic basis (vs OCCT's empirical `TestDeflection`).
- **At singularities**: constructs local parametrizations (place/branch analysis); for implicit
  plane curves adds **desingularization** (blow-ups) to trace *through* all singularity types
  correctly — the strongest classical answer to "how do I march through a crossing/cusp instead of
  stalling on it". Practical takeaway: near a singular point, switch from (predict, correct) on the
  curve to a local model of the *singularity* (quadratic cone of tangents; see §6 Ye–Maekawa) and
  jump a finite distance along each branch before resuming normal marching.

### 2.3 Kriezis–Patrikalakis–Wolter 1992 — oriented distance function + critical points
CAD 24(1):41–55. https://www.sciencedirect.com/science/article/abs/pii/001044859290090W
- Represents the intersection implicitly as the zero set of the **oriented distance function** of one
  surface from the other, on one parameter domain. Loops/singularities correspond to **critical
  points of the gradient vector field** of that function; an adaptive search guided by topological
  conditions (field index/degree over subdomains) finds them; tracing is by tensorial ODEs on the
  implicit representation.
- This is the conceptual root of both Ma–Lee 1998 (§5.4) and the 2026 winding-number paper (§5.6):
  loop detection = detecting zeros of a vector field you can *count* with a degree/winding integral,
  which subdivision can localize without ever missing a zero.

### 2.4 Grandine–Klein 1997 — topology resolution THEN solve (Boeing production algorithm)
CAGD 14(2):111–134. https://www.sciencedirect.com/science/article/abs/pii/S0167839696000246
- Two-stage: (1) **topology resolution** — find all *turning points* (where the preimage curve is
  tangent to a parameter direction) and singular points by solving the associated polynomial
  systems with a global (all-roots) solver; cut the (u,v) domain into **parallel panels** whose
  boundary lines pass through every turning/singular point. Inside a panel every intersection branch
  is monotone in the sweep parameter, so counting branch crossings on panel walls determines the
  complete topology (loops included) *before* any tracing. (2) Solve each monotone segment as a
  **two-point boundary-value problem for a DAE** by spline collocation, parametrized (nearly) by
  arclength — no marching, no step-size failure, endpoints exactly on panel walls.
- Guarantee quality: complete **if** the turning-point solve finds all roots (they use a robust
  global solver on Bernstein forms). Critical/tangential cases where the criteria degenerate were
  classified and repaired by a **perturbation method**:
  "Classification and resolution of critical cases in Grandine and Klein's topology determination
  using a perturbation method", CAGD 2008,
  https://www.sciencedirect.com/science/article/abs/pii/S0167839608000162
- Related: Grandine, "Applications of Contouring", SIAM Review:
  https://epubs.siam.org/doi/abs/10.1137/S003614459936403X ; coupled topology resolution + domain
  decomposition for guaranteed trimmed-surface consistency:
  https://link.springer.com/article/10.1007/s10444-005-7539-5

### 2.5 Krishnan–Manocha 1997 — lower-dimensional (algebraic-curve) formulation
ACM TOG 16(1):74–106. https://dl.acm.org/doi/10.1145/237748.237751 ;
PDF: https://www.researchgate.net/publication/220184441
- Implicitize one surface (resultant/Dixon **matrix, kept as a matrix** — see §4.2); substitute the
  other parametrization → the intersection becomes an **algebraic plane curve F(u,v)=0 in one
  parameter domain**. Tracing, start points, singularities all become *curve* problems:
  - start point on **every open and closed component** via curve topology machinery (all
    F=Fu=Fv=0 singular points + turning points Fu=0 or Fv=0 by eigenvalue solves on the matrix
    representation — no missed loop by construction);
  - **singularity detection** = zeros of (F, Fu, Fv); all branches at the singularity recovered from
    the local cone;
  - **step size chosen to prevent component jumping**: bound the distance to the nearest other
    branch (matrix condition/eigenvalue gap), march in the higher-dimensional (u,v,s,t) space to
    keep the corrector well-posed.
  - Reported an order of magnitude faster than earlier robust algorithms.
- Loop detection companion: "Algebraic loop detection and evaluation algorithms for curve and
  surface interrogations", Graphics Interface '96. Overview page:
  http://gamma.cs.unc.edu/CSG/intersect.html

### 2.6 Mukundan–Ko–Maekawa–Sakkalis–Patrikalakis 2004 — validated ODE tracing
ACM SM '04. https://diglib.eg.org/items/7ad5e773-2606-40cd-b75a-62b8035e0870
- Traces transversal **and tangential** intersection segments with a **validated (interval) ODE
  solver**: each step returns a proven enclosure of the curve, eliminating **straying** (drifting to
  a nearby branch) and **looping** (revisiting a segment) *by construction*; yields a numerically
  verified upper bound on curve error in parameter space, mapped to a 3D model-space bound —
  a direct P5 (tolerance model) reference: the reported edge tolerance can be a *proof*, not an
  estimate.

### 2.7 Zhang–Cheng–Liu–Zhang 2025 — strongly monotone segment decomposition
CAGD 118 (2025) 102432. https://www.sciencedirect.com/science/article/abs/pii/S0167839625000214
- Decomposes the 4D intersection curve into segments **strongly monotone in parameter space whose
  3D images are also strongly monotone**; monotonicity ⇒ no straying/looping during tracing and
  the 3D topology between segments is maintained; error bound controlled by densifying the
  decomposition locally. Modern, practical restatement of Grandine–Klein + Bartoň–Elber–Hanniel
  monotonicity, tuned for tracing rather than collocation.

### 2.8 OCCT baseline (see `kb/occt_ssi-walking.md` for line-level detail)
- Seeding from polyhedral interference (multiple schedules per section line), boundary seeding via
  restriction-arc roots (`IntStart`), **seam duplication of seeds** on periodic bounds, periodic
  domain **widening** (KELARG=20 steps) so marches cross seams, pole mitigation by shrinking
  UVMaxStep 10× near parametric singularities, tangent-zone seed rejection (cos²>0.9998 + 8-probe),
  `TestDeflection` empirical step ladder, boundary framing + backtrack/anti-orbit tests,
  2D+3D closure tests, and post-walk purging. OCCT achieves robustness by *layered heuristics*, not
  guarantees — the same architecture class as ours, which is why the guarantee papers above matter.

---

## 3. SUBDIVISION METHODS

### 3.1 Houghton–Emnett–Factor–Sabharwal 1985 — divide-and-conquer with OBB culling
CAGD 2:173–183. https://www.sciencedirect.com/science/article/abs/pii/0167839685900226
- Subdivide by flatness (curvature + boundary linearity criteria); cull subpiece pairs with
  **oriented parallelepipeds**; approximate flat subpieces by two triangles each; intersect
  triangle pairs; merge segment soup into curves. This is exactly the ancestry of our mesh-section
  seeding — and inherits its defect: features below the flatness tolerance vanish. The literature
  fix is *not* finer subdivision but the certificates of §3.3/§5.

### 3.2 Interval Projected Polyhedron (IPP) solver + rounded interval arithmetic
Patrikalakis–Maekawa hyperbook ch. 4 (4.9): Bernstein-form polynomial systems, project control
points to 2D, intersect convex hulls, contract the box, subdivide when contraction stalls; rounded
interval arithmetic makes it numerically sound. Used to find **all roots** of the significant-point
systems (border, turning, collinear-normal, singular) — the global-solver backbone every
"guaranteed" SSI method assumes. https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/

### 3.3 Bartoň–Elber–Hanniel — No-Loop Test + Single-Component Test (deep-dive, PDF extracted)
"Topologically guaranteed solution of surface–surface intersections via no-loop and
single-component tests." PDF: https://gershon.cs.technion.ac.il/papers/TopolGuarSSI.pdf
(also CAD 43(8), 2011). SSI as 3 equations, 4 unknowns over D ⊂ R⁴; solution is a curve in 4-space.
- **NLT (no-loop test)**: for each constraint fᵢ build the **complementary (tangent) bounding
  hypercone** C^C_i = C^N_i(v_i, 90°−αᵢ) from the normal bounding cone (axis = average of gradient
  samples, αᵢ = max deviation). Let K = (∩ᵢ C^C_i) ∩ S³ bound all possible curve tangents. If a
  hyperplane α through the origin separates… i.e. α ∩ K = ∅, the curve's tangent image stays in one
  open hemisphere ⇒ ⟨a, φ′(t)⟩ > 0 ⇒ the curve is **monotone in direction a ⇒ no closed loop in D**
  (Lemma 3.5/Theorem 3.7). Test executed by intersecting α with the 2n bounding hyperplanes of the
  cones and checking the points stay inside S³ (Algorithm 1). Wedge products computed by
  Gram–Schmidt + random-vector completion (ε=1e-3 ⇒ retry probability ~1e-6).
  Limit case: an isolated (0-dim) root can only sit on the domain boundary — loops shrunk to points
  are still detected.
- **SCT (single-component test)**: with NLT true, solve the 2(n+1)=8 boundary systems
  F|_{xᵢ=αᵢ}=0, F|_{xᵢ=βᵢ}=0 (each a square system, solved by the all-roots solver). |S| boundary
  hits: 0 → discard box; 2 → exactly one monotone open segment: **trace it** (Newton
  predictor/corrector, monotone ⇒ cannot stray or close); ≥4 → subdivide (hit locations hint the
  split). Semi-loop jumping is impossible *because* NLT passed first.
- **Merge**: traced segments joined at shared box-boundary points; topology of the global curve is
  guaranteed. Tightening ∥PᵢO∥ trades subdivision depth for steeper monotonicity (more robust
  tracing).
- This is the cleanest drop-in certificate for a marcher like ours: run NLT+SCT per candidate UV×UV
  box *only where the mesh seeding found nothing* or where classification is ambiguous.

### 3.4 Bounding-volume refinements
- Sederberg–Zundel, "Pyramids that bound surface patches" (GMIP 1996): translatable pyramid such
  that patch lies outside it from any of its own points — sharper loop/self-intersection culling
  than cones. https://www.sciencedirect.com/science/article/abs/pii/S1077316996900052

---

## 4. ALGEBRAIC / RESULTANT METHODS

### 4.1 Implicitization background
Sederberg's implicitization + moving surfaces; modern overview of matrix implicit representations:
https://arxiv.org/pdf/1502.00890 , https://inria.hal.science/hal-00847802/document

### 4.2 Manocha–Canny 1991 — resultants as numeric eigenproblems
ACM Solid Modeling '91 / IJCGA. https://www.worldscientific.com/doi/abs/10.1142/S0218195991000311
- Express the resultant of the intersection equations as a **matrix determinant but never expand
  it**; the matrix (entries linear in x,y,z or in the remaining parameters) *is* the curve
  representation; evaluation/intersection questions become **eigenvalue/eigenvector computations**
  with well-understood numerical accuracy. Foundation of §2.5.

### 4.3 Yang–Jia–Yan 2023 — Topology-guaranteed B-spline SSI (TOG 42(6), SIGGRAPH Asia)
https://dl.acm.org/doi/10.1145/3618349
- Algebraic pipeline for B-spline×B-spline: **Dixon-matrix implicitization accelerated by moving
  planes** (works in the presence of base points); topology determination with "helping points"
  (turning/singular/boundary) that double as tracing start points; trace in the parameter domain,
  lift to 3D, then use the Dixon matrix to **clip redundant branches** (the implicit curve contains
  phantom components outside the real patch). Handles multi-branch crossings, isolated singular
  contacts, high-order contact along a curve, boundary-coincident intersections.

### 4.4 Cheng–Zhang–Xiao–Li 2023 — Interval Algebraic Topology Analysis (TOG 42(4))
https://dl.acm.org/doi/10.1145/3592452
- Hybrid symbolic–numeric: configures the SSI topology **inside a 4D interval box**; all topological
  situations classified/enumerated as mixtures of **four fundamental cases**; covers cusp points and
  cusp curves, isolated and non-isolated tangency, **tiny loops**, self-intersections. The strongest
  current answer to "certified topology under floating point" for rational surfaces.

### 4.5 Matrix-representation tracing (2025)
"An Efficient and Robust Tracing Method Based on Matrix Representation for Surface-Surface
Intersection": https://link.springer.com/chapter/10.1007/978-981-96-5812-1_4 — modern fusion:
march, but with the matrix representation supplying corrector Jacobians and branch separation.

---

## 5. LOOP DETECTION GUARANTEES (the W1 answers, ranked by strength)

### 5.1 Sederberg–Meyers 1988 — boundary-branch condition via tangent/normal cones
CAGD 5(2):161–171. https://www.sciencedirect.com/science/article/abs/pii/0167839688900295
- Condition guaranteeing **every branch of the intersection touches a patch boundary** — if the
  bounding cone of one surface's iso-curve tangents lies entirely inside the tangent cone of the
  other surface (checked both ways), no closed loop exists; then boundary seeds alone are complete.
  Subdivide until the condition holds. Cheap: two cone constructions + containment tests per box.

### 5.2 Sederberg–Christiansen–Katz 1989 — collinear-normal certificate
CAD 21(10):505–508. https://www.sciencedirect.com/science/article/abs/pii/0010448589900584
- Theorem: patches intersecting in a closed loop (with normal-variation hypotheses) admit a **line
  perpendicular to both patches**. Contrapositive: *no collinear-normal pair ⇒ no loop*. The
  collinear-normal points are roots of a 4-eq/4-unk polynomial system (hyperbook Eq. 5.100,
  https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node109.html): solve once globally (IPP),
  **subdivide the patches at the collinear-normal points, then boundary seeds of the sub-patches
  capture every component** — loops become open arcs of sub-patches. This is the classical
  "guaranteed branch capture" recipe used by the MIT school.

### 5.3 Hohmeyer 1991 — pseudo-Gauss maps + linear programming
ACM Solid Modeling '91; PhD Berkeley 1992. https://dl.acm.org/doi/10.1145/112515.112543
- Bounds Gauss maps with pseudo-normal patches; loop-exclusion reduces to a **separability LP**
  (does a great circle separate the two normal bound sets?). Notably: needs **no tolerances beyond
  machine arithmetic**; works for any surface with evaluable bounds. Sharper than cones (5.1) at
  similar cost; the natural upgrade path for an NLT-style test.

### 5.4 Topological/vector-field methods
- Ma–Lee 1998, "Detection of loops and singularities of surface intersections", CAD 30(14):
  https://www.sciencedirect.com/science/article/abs/pii/S0010448598000566 — oriented distance
  function between the surfaces; **singular points of its gradient field** (found by field-index
  counting over subdomains) flag potential loops; bisect until each cell has index 0 (no loop) or
  isolates one critical point.
- "Topological method for loop detection of surface intersection", CAD 1995:
  https://www.sciencedirect.com/science/article/abs/pii/001044859500002X
- Kriezis et al. 1992 (§2.3) is the same principle with tracing integrated.

### 5.5 Grandine–Klein turning-point completeness (§2.4)
All-roots solve of turning/singular systems + panel decomposition ⇒ loops counted by wall
crossings. Complete under the global-solver assumption; critical (tangential) cases handled by
perturbation (CAGD 2008).

### 5.6 Winding numbers + gradient-variation subdivision (TOG, publ. 2026) — state of the art
"A Robust and Efficient Intersection Algorithm for NURBS Surfaces: Handling Small Loops and Tangent
Intersections": https://dl.acm.org/doi/10.1145/3807948
- Defines a vector field on one surface's parameter domain (distance-gradient type, §2.3 lineage);
  **winding-number theory detects small loops and isolated singularities** (zeros of the field are
  counted exactly by the winding integral on cell boundaries — a zero cannot be missed no matter how
  small the loop); a **subdivision scheme driven by gradient-direction variation on the parameter
  grid** localizes **closest-point pairs with parallel normals**, giving certified starting points
  *on tangential intersection branches* — i.e. it solves W1 and the tangential half of W3 with one
  mechanism. Explicitly motivated by CAD-kernel topological correctness.
- Related winding-number machinery: https://dl.acm.org/doi/10.1145/3658228 (rational curves),
  https://arxiv.org/html/2504.11435 (trimmed NURBS containment via generalized winding numbers).

### 5.7 Practical hierarchy (what to implement, in order)
1. Boundary seeds: all restriction-arc × other-surface roots (OCCT IntStart model; already partial).
2. Collinear-normal solve (5.2) on coarse UV×UV boxes → interior seeds for loops + tangency
   candidates; subdivide at the roots.
3. Per-box certificate where 1–2 are ambiguous: cone/LP no-loop test (5.1/5.3) or NLT+SCT (§3.3).
4. Optional gold-plating: winding-number zero counting (5.6) for sub-tolerance loops/tangencies.

---

## 6. TANGENTIAL / SINGULAR INTERSECTIONS (the W3 answers)

### 6.1 Ye–Maekawa 1999 — differential geometry at tangential contact (the reference formulas)
"Differential geometry of intersection curves of two surfaces", CAGD 16(8):767–788:
https://www.sciencedirect.com/science/article/abs/pii/S0167839699000187 ;
worked form in hyperbook 6.4: https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node122.html ,
6.4.1: https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node123.html
- At a tangential point (N1 ∥ N2) the tangent t = N1×N2 degenerates. Write t in the common tangent
  plane basis; **equality of normal curvatures of the two surfaces in the unknown direction**
  (second fundamental forms II₁ = II₂ along t) yields a **quadratic equation** in the direction
  components (hyperbook eq. 6.64, coefficients from IIᵢ and basis dot products, eqs. 6.60–6.63).
  Discriminant Δ classifies the contact:
  - Δ > 0: **branch point** — two distinct real tangent directions, i.e. two crossing tangential
    branches (march BOTH, in both senses: 4 departures);
  - Δ = 0: **tangential intersection curve** — unique (double-root) direction: march it;
  - Δ < 0: **isolated tangential contact point** — emit a vertex, march nothing;
  - Δ ≡ 0 (coefficients vanish): higher-order/curvature-continuous contact → same-domain
    candidate (P3), needs higher-order analysis or overlap extraction (§8.1).
- Marching direction *along* a tangential curve: the determinant of the **Hessian of the oriented
  distance function is zero** on the tangential curve; its kernel direction is the march direction.
  Ye–Maekawa also give curvature and higher derivatives for both transversal and tangential curves
  (usable for Taylor predictors, §2.2).

### 6.2 Corrector design at tangency
- The 3-eq/4-unk Newton system loses rank at tangency. Remedies in the literature:
  (a) **augmented systems**: add the tangency condition (N1×N2 = 0 component or det II = 0) and
  solve the bordered system — restores quadratic convergence *on* the tangential curve;
  (b) **validated ODE tracing** (§2.6) which handles tangential segments uniformly;
  (c) Krishnan–Manocha: at singular points of the algebraic curve (F=Fu=Fv=0) compute all local
  branches from the quadratic cone and re-seed each (§2.5);
  (d) OCCT's pragmatic fallback: reject tangent-zone seeds, stop marches on tangency
  (`ArretSurPoint`), and cover tangential loci by dedicated Geom-Geom recognizers — the reason our
  analytic-pair carve-outs (cone×cone) exist. Literature consensus: (a)+(§6.1 classification) is
  the minimal correct core; (d) alone loses tangential branches, which is exactly our W3.
- Grandine–Klein critical-case perturbation (CAGD 2008) is the topology-stage analog: perturb the
  offset ε, resolve topology for S1 ∩ offset_ε(S2), take ε→0 limits.

### 6.3 Singular points on transversal curves (crossings/cusps)
- Bajaj et al. 1988: local parametrization/desingularization to continue through singularities
  (§2.2). At an X-crossing in UV, our arrangement-level "cut-node crossing snap" is the discrete
  version; the literature version computes the crossing point by solving the singular system
  (F=∇F=0) with a bordered Newton, then reconnects branch pairs by tangent-direction matching
  (Δ>0 case of §6.1).

---

## 7. SEAM / POLE TREATMENT

Sparse in the academic literature (papers assume simply-connected rectangular domains); the
industrial answers:
- **OCCT** (kb, line-level): duplicate every seed lying on a periodic bound to the opposite chart
  (`Precision::PConfusion` snap, ~L1866–1959 PrmPrm), **widen the walking domain past the seam**
  by min(20·pasuv, gap-to-period) so marches cross seams without stalling (KELARG, PWalking ctor),
  shrink UVMaxStep 10× when a parametric pole (D1-norm < confusion) sits within (1e-7, 1e-5) of the
  other surface (`CheckSingularPoints`/`DefineUVMaxStep`), and clamp/snap converged params to
  bounds before acceptance. Post-hoc, walked lines crossing the seam are **decomposed at seam
  jumps** (our brep_section seam decomposition mirrors this).
- **Rhino/OCCT practice** (validated in our repo, `reference_seam_split_cuts`): split periodic
  surfaces at the seam into two charts *before* booleans — sidesteps seam-crossing marches entirely;
  the cost is doubled face counts and seam-edge bookkeeping.
- **Pole (sphere/cone apex)**: march in a rotated or stereographic chart near poles, or clamp step
  by 3D arclength not UV (UV steps blow up as 1/sin φ); OCCT's singular-direction stop
  (~L1700–1749) kills marches whose 3D progress vanishes — completion must then come from the
  *other* operand's chart (mate-based completion), which our scaffold closure-weld already
  approximates.
- Grandine–Klein-style topology resolution works per chart; loops that cross the seam appear as
  two boundary-terminated branches whose wall crossings match across the seam — the panel/wall
  bookkeeping generalizes cleanly if seam walls are treated as interior walls with identification.

---

## 8. ADJACENT RESULTS FOR MASTER-PLAN PHASES

### 8.1 P3 — same-domain / overlap
- "Overlap Region Extraction of Two NURBS Surfaces", ACM TOG (2025):
  https://dl.acm.org/doi/10.1145/3763308 — computes the 2D overlap *regions* (not curves) of two
  NURBS surfaces: the correct primitive for same-domain face pairs (our SEG-UNIFY / SD imprint);
  boundary of the overlap region = the same-domain section curves.
- OCCT `IntStart` **Arcsol** whole-arc-solution detection (restriction arc entirely on the other
  surface ⇒ emit a segment/RLine, not point soup) — the boundary-level coincidence primitive.
- IATA (§4.4) enumerates tangential-curve topology cases needed to classify partial overlap rims.
- "Fast Determination and Computation of Self-intersections for NURBS Surfaces", TOG 2025:
  https://dl.acm.org/doi/10.1145/3727620 (same machinery, one surface against itself).

### 8.2 P4 — EE/EF interference
- The boundary-seed machinery (§5.7 step 1, OCCT IntStart model) *is* EF interference: restriction
  arc × surface roots with `math_FunctionAllRoots`-style certified interval rooting
  (EpsX=1e-10, maxdist=TolBoundary+TolTangency), graze-interval collapse to a single min-|F| vertex,
  and exact curve×quadric fast paths. Reuse it for both seeding and P4 vertices so section endpoints
  and EF interference points are the *same object* (OCCT's pave identity discipline).

### 8.3 P5 — tolerance model
- "A Precision Controlled Surface-Surface Intersection Algorithm for NURBS", ACM TOG (2026):
  https://dl.acm.org/doi/10.1145/3806045 — **a-priori Hausdorff control**: Lipschitz constants +
  convex hulls bound polyline-vs-true-curve distance; fitted-curve-vs-polyline bounded by a novel
  fitting method; OBB + normal-range analysis classify intersection existence/type first. Claims
  superiority over commercial and open-source kernels. Template for making our reported edge
  tolerance a certified bound (feeds SameParameter budget).
- "Improving the Watertightness of Parametric Surface/Surface Intersection", CGF (2025):
  https://onlinelibrary.wiley.com/doi/10.1111/cgf.70298 — the W7 problem by name: make the two
  pp-curves and the 3D curve mutually consistent under the two surface maps so trimming is
  watertight; directly relevant to our F1 parameterization-mismatch reseeding and mate deviation
  (dev A/B) diagnostics.
- Validated tracing (§2.6) supplies proven per-segment bounds.

### 8.4 P6 — corpus validation
- The TOG papers (§4.3, §4.4, §5.6, §8.3) all benchmark against OCCT and/or commercial kernels on
  loop/tangency corpora — mine their failure examples (tiny loops, tangent circles, cusp curves,
  high-order contact) as battery cells; IATA's four-fundamental-case enumeration is a coverage
  checklist for corpus design.

---

## 9. EXTRACTED ALGORITHMS MAPPED TO OUR WEAKNESSES

| # | Weakness | Literature fix | Concrete change |
|---|----------|----------------|-----------------|
| 1 | W1 missed loops/branches | Collinear-normal solve (§5.2) + no-loop certificate (§5.1/§5.3/§3.3); winding numbers (§5.6) | New `ssi_seed_certify(A_face,B_face)`: coarse UV×UV boxes; per box run cone-overlap no-loop test; failing boxes get a collinear-normal IPP solve; roots → extra seeds fed to the existing marcher; boxes passing NLT with 0 boundary hits are *proven empty* — kills the mesh-resolution dependency of scaffold seeding |
| 2 | W2 order sensitivity | Lower-dimensional formulation (§2.5); symmetric seed set | Seeds and certificates computed on the *pair* (4D boxes), marcher consumes one canonical seed list; drop the swapped-retry heuristic |
| 3 | W3 tangency stalls | Ye–Maekawa quadratic classification (§6.1) + bordered Newton (§6.2a) | At corrector rank-drop (normals cos² > threshold): classify via Δ of eq. 6.64 → isolated vertex / march double-root direction with augmented system / spawn 4 branch departures; replaces minimization-completion of stalled marches |
| 4 | W4 heuristic termination | Boundary relaxation (§2.1); framing on exact border points (§2.4, OCCT TestArret) | March ends = roots of restriction-arc×surface systems (P4 machinery), landed by 1D Newton on the boundary iso — retire `tol3` endpoint bridging where a certified border point exists |
| 5 | W5 seam/pole | OCCT seed duplication + domain widening + pole step shrink (§7) | Duplicate periodic-bound seeds to both charts; allow march window ± k·step past seam then decompose at seam jumps (already partial); clamp steps by 3D arclength near poles |
| 6 | W6 no topology certificate | SCT boundary-hit counting (§3.3); panel wall crossings (§2.4) | Per certified box: |boundary hits| = 2 ⇒ one segment, 0 ⇒ none; assert traced network matches; mismatch ⇒ subdivide and re-march, instead of flood-fill quorum absorbing the error |
| 7 | W7 pp-curve consistency | Watertightness CGF 2025 (§8.3); validated bounds (§2.6) | Track per-segment Hausdorff bound (Lipschitz + hull), store as edge tolerance; reconcile uvA/uvB preimages against the 3D curve before approximation |

Marching-engine upgrades independent of the above: third-order Taylor predictor with analytic step
bound (§2.2) to replace the empirical deflection ladder; strong-monotonicity segment decomposition
(§2.7) as the internal invariant ("a marched segment is monotone in some direction a" — a cheap
assert derived from NLT's plane normal).

---

## 10. BIBLIOGRAPHY (primary URLs)

Marching: Barnhill–Kersey https://www.sciencedirect.com/science/article/abs/pii/016783969090035P ·
Bajaj–Hoffmann–Lynch–Hopcroft https://www.sciencedirect.com/science/article/abs/pii/0167839688900106 ·
Kriezis–Patrikalakis–Wolter https://www.sciencedirect.com/science/article/abs/pii/001044859290090W ·
Grandine–Klein https://www.sciencedirect.com/science/article/abs/pii/S0167839696000246 ·
GK critical cases https://www.sciencedirect.com/science/article/abs/pii/S0167839608000162 ·
Krishnan–Manocha https://dl.acm.org/doi/10.1145/237748.237751 ·
Validated ODE https://diglib.eg.org/items/7ad5e773-2606-40cd-b75a-62b8035e0870 ·
Monotone tracing 2025 https://www.sciencedirect.com/science/article/abs/pii/S0167839625000214
Subdivision: Houghton et al. https://www.sciencedirect.com/science/article/abs/pii/0167839685900226 ·
Bartoň–Elber–Hanniel https://gershon.cs.technion.ac.il/papers/TopolGuarSSI.pdf ·
Sederberg–Zundel https://www.sciencedirect.com/science/article/abs/pii/S1077316996900052
Algebraic: Manocha–Canny https://www.worldscientific.com/doi/abs/10.1142/S0218195991000311 ·
Yang–Jia–Yan https://dl.acm.org/doi/10.1145/3618349 ·
IATA https://dl.acm.org/doi/10.1145/3592452 ·
Matrix-rep tracing https://link.springer.com/chapter/10.1007/978-981-96-5812-1_4
Loops: Sederberg–Meyers https://www.sciencedirect.com/science/article/abs/pii/0167839688900295 ·
Collinear normals https://www.sciencedirect.com/science/article/abs/pii/0010448589900584 ·
Hohmeyer https://dl.acm.org/doi/10.1145/112515.112543 ·
Ma–Lee https://www.sciencedirect.com/science/article/abs/pii/S0010448598000566 ·
Winding numbers https://dl.acm.org/doi/10.1145/3807948
Tangency: Ye–Maekawa https://www.sciencedirect.com/science/article/abs/pii/S0167839699000187 ·
Hyperbook 6.4 https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node122.html
Phases: Overlap extraction https://dl.acm.org/doi/10.1145/3763308 ·
Self-intersections https://dl.acm.org/doi/10.1145/3727620 ·
Precision control https://dl.acm.org/doi/10.1145/3806045 ·
Watertightness https://onlinelibrary.wiley.com/doi/10.1111/cgf.70298
Surveys: Hyperbook https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node109.html ·
Dokken https://link.springer.com/chapter/10.1007/978-3-540-68783-2_3 ·
Challenges https://link.springer.com/chapter/10.1007/3-540-27157-0_2 ·
Survey '91 https://www.sciencedirect.com/science/article/abs/pii/009784939190037I
