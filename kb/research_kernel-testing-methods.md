# How Production Geometry Kernels Are Validated — and a Test-Methodology Design for Ours

Research survey (2026-07-24) + concrete methodology for the session kernel (tolerance-based NURBS B-Rep boolean kernel, OCCT-aligned, phases P3 same-domain / P4 EE-EF interference / P5 tolerance model / P6 corpus validation).

---

## 1. What the commercial kernels publish about their QA

### 1.1 Parasolid (Siemens)

Siemens' published quality material is the most explicit of the three majors:

- **"Millions of overnight auto-tests with full functional coverage"** plus preemptive developer testing of code changes and in-application / customer-scenario tests. ([Siemens PLM Components blog](https://blogs.sw.siemens.com/plm-components/parasolid-delivers-30-years-of-quality-in-industrial-use/))
- **Zero tolerance for regressions** as an explicit process rule; every customer-reported issue is converted into a permanent regression test; coverage analysis drives suite growth; all new functionality ships with accompanying tests. (same source)
- Release cadence: major release every ~6 months + maintenance updates; the suite gates every release. 200+ ISVs / 350+ applications depend on XT compatibility, which is why round-trip stability is a first-class test axis. (same source)
- **The checker is a product API, not just an internal tool**: `PK_BODY_check` "performs a series of checks on a body and returns information about the faults found" — geometry checks (self-intersection = "lazy" vs "full" checks, continuity), topology checks, and data-structure checks, with options to select subsets; by default "all checks appropriate to the body will be made". Most modeling functions do *not* validate inputs — validity is a contract maintained by operations and audited by the checker. ([PK_BODY_check](http://www.q-solid.com/Parasolid_Docs/headers/pk_body_check.html), [Parasolid Checking chapter](http://www.q-solid.com/Parasolid_Docs/chapters/fd_chap.09.html), [PK_FACE_check](http://www.q-solid.com/Parasolid_Docs_V35/headers/pk_face_check.html), [PK_check_state_t](http://www.q-solid.com/Parasolid_Docs_V35/headers/pk_check_state_t.html))
- Notably, the docs admit the checker itself can fail on "difficult geometric configurations" (`PK_ERROR_check_fail`) — i.e. even the oracle is tolerance-limited. ([Error handling chapter](http://www.q-solid.com/Parasolid_Docs_V35/chapters/fd_chap.121.html))

**Takeaways for us:** (a) the validity checker *is* the primary oracle and must be a hardened, versioned component of the kernel itself; (b) customer/field failures become permanent named regression cases; (c) scale matters — they win by running the whole functional surface nightly, not by clever single tests.

### 1.2 ACIS (Spatial / Dassault)

Less published detail, but the architecture is visible:

- **Scheme AIDE** is a Scheme-language driver exposing "a large subset of ACIS functionality", used "to test ACIS functionality, prototype software, or communicate questions or suspected problems to Spatial" — i.e. the scriptable harness doubles as the bug-repro exchange format with customers, exactly like OCCT's DRAW. ([Scheme AIDE docs](http://www.q-solid.com/ACIS_Docs_R17/online/SPAacisgsTechArticles/SPAacisgs_usex.htm))
- **`api_check_entity`** "checks an entity's geometry, topology, and data structure for errors" — same three-level checker taxonomy as Parasolid. ([ACIS docs](http://www-isl.ece.arizona.edu/ACIS-docs/HTM/DATA/INTR/INTR/03FN/0003.HTM))
- Every API returns an **`outcome`** object; Spatial's guidance is systematic outcome-checking + logging around every modeling call. ([Spatial blog: outcome checking](https://blog.spatial.com/3d-acis/outcome-checking-logging-and-progress-reporting-3d-acis))
- Process: daily builds on all platforms with immediate automated test runs, and a **multi-level promotion system** where code must pass each level before advancing toward release. ([Spatial blog: integration team](https://blog.spatial.com/employee-spotlight-behind-the-scenes-how-integration-ensures-spatials-software-remains-reliable-and-secure))

**Takeaways:** scripting harness = test format = customer repro format (one artifact, three uses); promotion levels (fast smoke → full battery → release certification) rather than one monolithic suite.

### 1.3 OCCT (Open Cascade)

The only kernel whose test system is fully public — and the closest architecture to ours:

- **Structure: group → grid → case.** Groups: `blend, boolean, bugs, caf, chamfer, de (data exchange), demo, draft, feat, heal, mesh, mkface, nproject, offset, pipe, prism, sewing, thrusection, xcaf`, … Boolean grids are split per-op and per-difficulty: `bcommon_2d/_simple/_complex`, `bcut_*`, `bfuse_*`, `bop{common,cut,fuse,tuc}_*`, `bsection`, `bopsection`. ([OCCT Automated Testing System](https://dev.opencascade.org/doc/overview/html/occt_contribution__tests.html), [OCCT wiki: tests](https://github.com/Open-Cascade-SAS/OCCT/wiki/tests))
- **A test case is a tiny Tcl script** (build shapes → run op → check), with per-grid `begin`/`end` hooks (`pload`, `set cpulimit 300`, final `checkshape result`, `puts "TEST COMPLETED"`) and `parse.rules` regex classification of log output (`FAILED /\bFaulty\b/ bad shape`).
- **Five verdicts**: OK / FAILED (regression) / **BAD (known problem — recorded, tolerated, tracked)** / IMPROVEMENT (known problem now fixed — flags the change that fixed it) / SKIPPED (data missing). This "known-bad ledger" is the key device for driving a frontier forward without blocking CI — a real run posted publicly showed `208 BAD, 31 SKIPPED, 3 IMPROVEMENT, 1791 OK` for one battery. ([OCCT 8.0 RC discussion](https://github.com/Open-Cascade-SAS/OCCT/discussions/1097))
- **Standard check commands** (each one an oracle class):
  - `checkshape` — BRepCheck_Analyzer topological/geometric validity;
  - `checknbshapes` — exact sub-shape census (`-vertex 8 -edge 4 …`);
  - `checkprops` — mass properties vs pinned reference (`-s 6265.68`, with epsilon);
  - `checktrinfo` — mesh triangle/node counts + deflection;
  - `checkreal` — scalar vs reference with tolerance;
  - `checkview`/`vdump` — screenshot vs master image.
- **`testdiff` compares two runs on CPU time, memory, and images** — performance and leak regressions are first-class, not an afterthought. Tests run in parallel (one process per CPU).
- **Bug-driven tests**: every fixed issue gets `bug<ID>[_<n>]` in the `bugs` group — the mantis/GitHub tracker and the test suite are joined at the name.
- Some certification data is confidential to Open Cascade — the public suite is a subset; full certification ("non-regression on all existing test cases and supported platforms") is run by the company per release. ([DRAW Test Harness](https://dev.opencascade.org/doc/occt-7.6.0/overview/html/occt_user_guides__test_harness.html))
- Newer OCCT adds a **GoogleTest C++ unit layer** (`OpenCascadeGTest`) beside the Tcl battery — two-tier: unit tests for kernel math, scripted scenario tests for modeling ops.
- **Fuzzy booleans as a product feature**: `BRepAlgoAPI_BooleanOperation::SetFuzzyValue(theFuzz)` inflates the interference tolerance to "handle robustly cases of touching and near-coincident entities" — i.e. OCCT institutionalizes tolerance-perturbation as an *input parameter*, which also makes it a natural test axis (sweep fuzz, verdict must be stable/monotone). ([OCCT Boolean Operations guide](https://dev.opencascade.org/doc/occt-7.4.0/overview/html/occt_user_guides__boolean_operations.html), [BOPAlgo_Options](https://dev.opencascade.org/doc/occt-7.5.0/refman/html/class_b_o_p_algo___options.html))

### 1.4 Industry-level conformance testing (data exchange)

- **CAx-IF / MBx-IF**: since 1999, vendors (incl. all kernel-based CAD systems) run joint STEP **test rounds** — native models built to a Test Case Specification → STEP export → cross-import into every other system → statistics loaded into a shared database (CAESAR) → divergences become Recommended Practices. 41+ rounds completed in the CAD domain. ([MBx-IF](https://www.mbx-if.org/home/cax/), [who we are](https://www.mbx-if.org/who_we_are.php), [Datakit on CAx-IF](https://www.datakit.com/en/step_improving_the_use.php))
- **NIST MBE PMI test system**: curated CAD test models (CTC/FTC series) + verification & validation criteria measuring CAD-system conformance to ASME Y14.5/Y14.41 through STEP/JT derivatives; still the seed corpus for CAx-IF AP242 testing. ([NIST project](https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-validation), [results browser](https://pages.nist.gov/CAD-PMI-Testing/results.html))

**Takeaway:** for a kernel whose I/O is STEP, *cross-system round-trip* is a recognized industry oracle: write → re-read → same topology census, same mass properties, and (for us) same boolean verdicts; plus third-party import (Rhino, OCCT) as external judges — which we already do with the headless Rhino probe.

---

## 2. The oracle problem and the oracle hierarchy for geometry

Geometry code is the canonical "no test oracle" domain: for a nontrivial boolean there is no closed-form expected output. The scientific-software testing literature (systematic review: [Kanewala & Bieman 2014](https://arxiv.org/abs/1804.01954)) lists the standard workarounds — pseudo-oracles (differential testing), analytical special cases, expert judgment, and **metamorphic testing** — all of which map cleanly onto what Parasolid/ACIS/OCCT actually do:

| Oracle class | Industrial instance | Ours today |
|---|---|---|
| Validity checker | PK_BODY_check / api_check_entity / checkshape | naked-edge count, closed-shell check, `BRepCheck`-style audit (partial) |
| Census oracle | checknbshapes exact sub-shape counts | face/edge counts vs OCCT ground truth (chairs 35/25/50) |
| Mass-property oracle | checkprops area/volume vs pinned reference | volume rel-error vs OCCT (1e-4…1e-5) |
| Differential (pseudo-)oracle | CAx-IF cross-system rounds | validation/ OCCT + headless Rhino probes |
| Analytic oracle | primitives with closed-form results | analytic recognizer families (cyl/cone/torus exact) |
| Metamorphic oracle | GraphicsFuzz; OCCT fuzzy sweeps (implicitly) | **missing — the biggest gap** (§4) |
| Visual regression | checkview master images | viewer screenshots (manual) |

Ranking principle (used below): a single test should consult the *cheapest oracle that can fail it* — validity < census < mass properties < differential < analytic.

---

## 3. Robustness theory that motivates perturbation testing

- **Kettner, Mehlhorn, Pion, Schirra, Yap — "Classroom examples of robustness problems in geometric computations"** (CGTA 2008): systematically constructs float inputs that make textbook convex-hull/Delaunay implementations crash, loop, or return wildly wrong output, by exploring the sign-error geometry of the float grid near degeneracy. The paper's method — *enumerate inputs in a tiny neighborhood of a degenerate configuration and map the wrong-verdict region* — is directly a test-generation recipe. ([paper](https://www.sciencedirect.com/science/article/pii/S0925772107000697), [HAL](https://inria.hal.science/inria-00344310/))
- **Simulation of Simplicity** (Edelsbrunner & Mücke, ACM TOG 1990): symbolic infinitesimal perturbation removes degeneracies consistently — the theoretical statement that *degenerate inputs are the hard core* and that consistent tie-breaking (not per-site epsilon hacks) is what fixes them. ([ACM](https://dl.acm.org/doi/10.1145/77635.77639), [PDF](https://www.sandia.gov/files/samitch/unm_math_579/p66_edelsbrunner_simulation_of_simplicity.pdf))
- **Controlled perturbation** (Halperin et al.; general analysis by Mehlhorn et al.): numerically perturb the input by a bounded δ so all predicates become reliably evaluable in float — the practical dual: *if a kernel is correct, its verdicts must be stable under sub-tolerance jitter*. ([analysis](https://people.mpi-inf.mpg.de/~mehlhorn/ftp/ControlledPerturbationGeneralApproach.pdf), [MPI lecture](https://resources.mpi-inf.mpg.de/departments/d1/teaching/ws09_10/CGGC/Notes/Perturbation.pdf), [overview](https://en.wikipedia.org/wiki/Robust_geometric_computation))

For a *tolerance-based* (non-exact) kernel like ours and OCCT, these translate into two testable contracts:
1. **Jitter stability**: perturbing every input coordinate by δ ≪ tol must not change any discrete verdict (face counts, closedness); volume moves O(δ·area).
2. **Tolerance monotonicity**: sweeping the fuzzy/interference tolerance upward through a gap size must switch the verdict *once* (miss → hit), never oscillate. OCCT exposes exactly this knob (`SetFuzzyValue`).

Our rotated-chair campaign is precisely a rigid-motion instance of contract 1 — the literature says to systematize it, not treat each rotation as a one-off bug hunt.

---

## 4. Metamorphic testing: the main missing layer

**Theory.** Metamorphic testing (MT) checks *relations between outputs of multiple runs* instead of a per-run expected value — built for the oracle problem. Surveys: [Segura et al., IEEE TSE 2016](https://eprints.whiterose.ac.uk/id/eprint/110335/1/segura16-tse.pdf); [Chen et al., ACM Computing Surveys 2018](https://dl.acm.org/doi/10.1145/3143561); ML-assisted MR discovery ([Kanewala, STVR 2016](https://onlinelibrary.wiley.com/doi/10.1002/stvr.1594)).

**Production proof.** [GraphicsFuzz](https://dl.acm.org/doi/10.1145/3133917) (Donaldson et al., OOPSLA 2017; [short paper](https://www.doc.ic.ac.uk/~afd/homepages/papers/pdfs/2016/MET.pdf); [production report](https://www.doc.ic.ac.uk/~afd/papers/2020/ECOOP_GraphicsFuzz.pdf)) applies *semantics-preserving transformations* to shaders and flags image mismatches; acquired by Google, now a standing defense line for Android GPU drivers, and its reduced counterexamples were folded into the Khronos Vulkan CTS. The exact analogue for a boolean kernel: apply *solid-semantics-preserving transformations* to operands (rigid motions, operand swap, complement rewrites, tessellation changes, unit rescale) and flag verdict mismatches — then **auto-reduce** the failing transform to a minimal delta (their reducer is the part that made the tool usable; ours would bisect rotation angle / jitter magnitude).

### 4.1 Invariant catalog for a B-Rep boolean kernel

Notation: `∪ ∩ \ ⊕` regularized ops; `T` rigid motion; `s` uniform scale; `V()` volume; `A()` area; `#F/#E/#V` census; `tolV = c·diag³·1e-9` (calibrate c once). Class **E** = exact/discrete (must hold bit-for-bit or count-for-count), **N** = numeric (holds to stated tolerance), **S** = structural (validity predicates).

**Algebraic (multi-run, no oracle needed)**
| ID | Relation | Class | Catches |
|---|---|---|---|
| MR-A1 | `A∪B ≡ B∪A`, `A∩B ≡ B∩A` (census E, V/A N) | E/N | operand-order asymmetries in SSI walking, classification seeding |
| MR-A2 | `A\B ≠ B\A` but `V(A\B)−V(B\A) = V(A)−V(B)` | N | one-sided splitter loss (our SEGLOST class!) |
| MR-A3 | `A⊕B ≡ (A∪B)\(A∩B) ≡ (A\B)∪(B\A)` | N + census | consistency of the 4 ops sharing one intersection network |
| MR-A4 | `(A∪B)∪C ≡ A∪(B∪C)` on a 3-solid chain | N | pairwise-network reuse, tolerance accumulation across ops |
| MR-A5 | Idempotence: `A∪A ≡ A`, `A∩A ≡ A`, `A\A = ∅` | E | same-domain subsystem (P3) — the sharpest same-domain smoke test there is |
| MR-A6 | Absorption: `A∪(A∩B) ≡ A`, `A∩(A∪B) ≡ A` | N | classification of ON faces, coincident-face keep/drop rules |
| MR-A7 | Op on disjoint solids: `V(A∪B)=V(A)+V(B)`, `A∩B=∅`, `A\B≡A` | E | false-positive interference detection (P4 EE/EF) |
| MR-A8 | `B⊂A ⟹ A∪B≡A, A∩B≡B, V(A\B)=V(A)−V(B)` | N | containment classification, nested-shell handling |

**Measure identities (single run + mass properties)**
| ID | Relation | Class | Catches |
|---|---|---|---|
| MR-M1 | Inclusion–exclusion: `V(A∪B)+V(A∩B) = V(A)+V(B)` | N (tolV) | any face kept/lost/doubled anywhere across the two results |
| MR-M2 | Partition: `V(A\B)+V(A∩B) = V(A)`, and symmetric for B | N | per-operand splitter loss, one-sided (finer than MR-M1) |
| MR-M3 | `V(A∪B) = V(A\B)+V(B\A)+V(A∩B)` | N | cross-check of all four ops at once |
| MR-M4 | Flux consistency: divergence-theorem volume computed from result faces == volume from decomposition identity | N | orientation flips (our tier-3 volume-flux classifier is this invariant weaponized) |
| MR-M5 | Surface bound: `A(A∪B) ≤ A(A)+A(B)` (strict if interfering) | N | ghost faces / duplicated caps |

**Equivariance (transform-and-compare, GraphicsFuzz-style)**
| ID | Relation | Class | Catches |
|---|---|---|---|
| MR-T1 | Rigid: `T(A) op T(B) ≡ T(A op B)` — census E, V invariant, faces mapped within tol | E/N | **the rotated-chair failure class**, axis-aligned fast paths, seam placement |
| MR-T2 | Translation-only special case of T1 (cheap, run always) | E/N | absolute-coordinate leaks, tolerance vs position coupling |
| MR-T3 | Uniform scale: `V(s·A op s·B) = s³·V(A op B)`, census E, run s∈{1e-3, 1, 1e3} | E/N | absolute-epsilon leaks (mm vs m modeling) — P5 tolerance-model acceptance test |
| MR-T4 | Reflection/handedness: mirror both operands → mirrored result, census E | E | orientation-convention bugs, normal-sign assumptions |
| MR-T5 | Operand relabel (swap A,B and swap cut direction) | E | asymmetric code paths (complements MR-A1/A2) |

**Representation invariance**
| ID | Relation | Class | Catches |
|---|---|---|---|
| MR-R1 | STEP round-trip: write(op result) → read → census E, V within tolV, checker clean | E/N/S | writer/reader tolerance loss (the CAx-IF oracle, self-applied) |
| MR-R2 | Op(write∘read(A), write∘read(B)) ≡ Op(A,B) | E/N | reader-introduced pcurve/seam differences (our chairs pipeline!) |
| MR-R3 | Knot-insertion / degree-elevation of operand NURBS (same geometry, richer basis) → identical verdicts | E/N | parameterization-dependent SSI marching |
| MR-R4 | Seam relocation on periodic surfaces → identical result modulo seam edges | N | seam-handling (known industry-wide hard spot; Rhino/OCCT split at seams) |

**Perturbation / tolerance (P5's acceptance battery)**
| ID | Relation | Class | Catches |
|---|---|---|---|
| MR-P1 | Jitter stability: coords += U(−δ,δ), δ = 1e-{12,11,10,9}·diag → census E, V moves O(δ·A) | E/N | knife-edge predicates, non-robust quorums (Kettner et al. methodology) |
| MR-P2 | Near-coincidence ladder: place faces at gap g = k·tol, k∈{0.1,0.5,0.9,1.1,2,10} → verdict must be monotone in g, single switch | E | on/off ON-classification thresholds (our x20 naked class) |
| MR-P3 | Fuzzy sweep: rerun with interference tol × {0.5,1,2,4} → verdict stable or single monotone switch (OCCT SetFuzzyValue precedent) | E | tolerance cliffs |
| MR-P4 | Rotation grid: golden-angle set of ~30 (axis,angle) pairs applied per MR-T1 → pass-rate is a tracked scalar per corpus model | E/N | systematizes the current z15/z30/y30 frontier into one number |

**Structural gates (every result, always)**
| ID | Predicate | Catches |
|---|---|---|
| S-1 | naked/boundary edge count == 0 for closed inputs (watertight) | open shells |
| S-2 | every edge has exactly 2 face uses, opposite orientation (2-manifold, oriented) | non-manifold welds |
| S-3 | Euler–Poincaré: `V−E+F = 2(S−G)` per shell (with genus from known family or flux) | census corruption |
| S-4 | no zero-area faces / zero-length edges above tol; vertex tol ≥ containing-edge tol ≥ face tol consistency (Parasolid/OCCT tolerance-nesting rule) | sliver leaks, tolerance inversions |
| S-5 | self-intersection audit on result (sampled) | fold-overs from bad trims |

### 4.2 Which invariants would have caught our recent bug classes

- SEGLOST / one-sided whole-segment loss → MR-M2 (partition per operand) + MR-A2.
- All-faces-flip on z15/z30 → MR-M4 (flux vs identity volume) — already partially built as the tier-3 classifier; promote it from classifier to *test gate*.
- fb=47.99997 sliver key drop → MR-P2 ladder around the trim-snap tolerance.
- Cone×cone hijacked into scaffold → MR-A5/MR-A8 on analytic families + MR-T1 (recognizer must be rotation-equivariant).
- Rhino OPEN closed-edge loops → MR-R1 (external-reader clean import as census oracle).

---

## 5. Fuzzing geometry kernels

Two distinct fuzzing surfaces, with different payoffs:

1. **File-format fuzzing (crash/memory-safety oracle).** Precedent: Cisco Talos fuzzed CGAL's Nef polygon parsers and found **~19 code-execution vulnerabilities (CVE-2020-28601 … CVE-2020-35636)** — OOB reads and type confusion in `Nef_2/Nef_3/Nef_S2` — in a mature, widely-used geometry library. ([TALOS-2020-1225](https://talosintelligence.com/vulnerability_reports/TALOS-2020-1225), [CVE-2020-35628](https://nvd.nist.gov/vuln/detail/CVE-2020-35628)) Our STEP reader is the same attack surface. Recipe: libFuzzer/AFL++ harness around `step read → brep build → checker`, seeded with our existing STEP corpus, running under ASan/UBSan. Cheap to stand up; finds real crashes, not booleans-wrong bugs.
2. **Operation fuzzing (wrong-result oracle = the metamorphic layer).** Random CSG programs (bounded op-chains over corpus operands + random rigid motions) where the *only* oracles are §4's invariants + §6's differential judges. This is GraphicsFuzz's architecture transplanted: generator → invariant checker → **automatic reducer** (bisect chain length, then rotation angle, then jitter) → minimal repro saved as a named regression case, OCCT `bugNNNN`-style.

Property-based-testing frameworks supply the missing machinery for #2: generators + invariants + **shrinking** ([QuickCheck](https://github.com/BurntSushi/quickcheck) for Rust, [Hypothesis](https://getcode.substack.com/p/property-based-testing-5-shrinking) for the Python layer; note Hypothesis's warning that float shrinking needs special care). For C++, a thin in-house generator over our own Xform/corpus sampler is more practical than adopting rapidcheck.

---

## 6. Differential testing against reference kernels

- The industry does this at forum scale (CAx-IF rounds, §1.4). Academically, robust-geometry papers validate by running entire corpora through competing implementations and diffing outcomes (e.g. mesh-boolean papers all report Thingi10K success-rate tables — [Mesh Arrangements](https://www.researchgate.net/publication/305217551_Mesh_arrangements_for_solid_geometry), [EMBER](https://www.graphics.rwth-aachen.de/media/papers/339/ember_exact_mesh_booleans_via_efficient_and_robust_local_arrangements.pdf), [Interactive & Robust Mesh Booleans](https://arxiv.org/pdf/2205.14151)).
- **We already have the two best judges wired**: OCCT built from source and headless Rhino (`rhino_headless_probe.py`). Missing: a third, *independent-architecture* judge to break 1-1 ties — a mesh-CSG pseudo-oracle. Options: our own marching-cubes CSG (main_8.cpp, already exists), or the exact-arithmetic [Manifold library](https://github.com/elalish/manifold/wiki/Manifold-Library) on tessellated operands. With 3 judges, use **2-of-3 voting on {V within 1e-3·rel, census class, watertight}**; disagreement of all three = flag input as a research specimen, not a test failure.
- Comparison metrics, in order of strictness: crash/timeout < checker-invalid < not-watertight < census mismatch < V rel-error < face-mapped Hausdorff. Record the *ladder level reached*, not pass/fail — this is what turns a corpus run into a trackable frontier (like our chairs 30→8 naked progression).

---

## 7. Corpus validation (P6)

| Corpus | Size / nature | Use |
|---|---|---|
| In-house chairs + rotation grid | ~10 models × 30 poses × 3 ops | daily frontier metric (already the de-facto benchmark) |
| OCCT public test data | boolean grids `_simple/_complex` + `bugs/modalg_*` | free, adversarial, has 25 yrs of accumulated degeneracies; port cases as DRAW→our-CLI translations |
| [NIST PMI CTC/FTC](https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0) | ~20 curated STEP parts | reader conformance + round-trip (MR-R1) |
| [ABC dataset](https://openaccess.thecvf.com/content_CVPR_2019/papers/Koch_ABC_A_Big_CAD_Model_Dataset_for_Geometric_Deep_Learning_CVPR_2019_paper.pdf) | 1M Onshape B-Rep STEP files | the P6 endgame: sample 1k → 10k → 100k; boolean random pairs at random poses; ladder scoring |
| [Thingi10K](https://ten-thousand-models.appspot.com/) | 10k dirty meshes | *not* B-Rep — use only for the mesh-CSG judge and the reader's mesh path |

Corpus methodology (per the mesh-boolean literature): fixed random seed; publish the ladder table per release; every new failure auto-reduced and promoted to a named regression case; **zero-regression rule on the named set** (Parasolid's policy), BAD-ledger for known-open cases (OCCT's policy).

---

## 8. Concrete methodology design for the session kernel

Seven layers; each maps to existing infra and a master-plan phase.

**L0 — Unit (exists).** minitest ×3 languages; keep. Add GTest-style C++ micro-units only for predicate-level code (classifiers, quorums) where minitest granularity is too coarse — OCCT's two-tier precedent.

**L1 — Structural gates (mostly exists → formalize).** One function `kernel_check(result)` implementing S-1…S-5 (checker as product API, Parasolid-style). Every boolean in every test battery calls it. Output = fault list, not bool.

**L2 — Invariant engine (build now, P3/P4 lever).** New `session_invariants` runner: input = (A, B) pair + op; executes the MR catalog rows selected by a profile (`fast` = A1,A5,M1,M2,T2,S-all ≈ 9 extra ops; `full` = everything). CLI + env-gates like existing SESSION_*. *Highest-leverage single build item in this report* — MR-A5 alone is a continuous P3 acceptance test, MR-A7 for P4, MR-T3+P1-P3 for P5.

**L3 — Differential judges (exists → add voting).** OCCT + Rhino + mesh-CSG third judge; 2-of-3 vote; ladder scoring; wire into oracle scripts in validation/.

**L4 — Perturbation harness (generalize chairs-rotation).** MR-P1 jitter, MR-P2 gap ladder, MR-P3 tol sweep, MR-P4 golden-angle rotation grid — one harness, per-model pass-rate scalars, tracked over commits (testdiff-style trend, incl. CPU/mem like OCCT).

**L5 — Corpus campaigns (P6).** §7 ladder runs: nightly = chairs grid + OCCT-ported grids; weekly = NIST + ABC-1k sample; release = ABC-10k. Verdict ledger with OK/FAILED/BAD/IMPROVEMENT states — adopt OCCT's five verdicts verbatim.

**L6 — Fuzzing (background).** (a) ASan libFuzzer on the STEP reader, seeded with session_tests STEP corpus — start immediately, it is independent of kernel work; (b) CSG-program fuzzer with L2 as oracle + auto-reducer (bisect chain → angle → jitter) once L2 exists.

**Process rules (from the majors):** every field/bug failure becomes a permanently named regression case tied to its issue ID (OCCT `bugNNNN`); zero tolerance for regressions on the named set (Parasolid); known-open cases live in a BAD ledger so CI stays green while the frontier is tracked (OCCT); promotion levels fast→full→release (Spatial); perf+memory diffs on every battery (OCCT `testdiff`).

**Effort-ranked next actions:** 1) L2 invariant engine with `fast` profile in the chairs loop; 2) MR-M2/M4 as gates (they detect the two live bug classes); 3) reader libFuzzer+ASan; 4) third-judge voting; 5) rotation-grid pass-rate scalar in CI; 6) ABC-1k pilot.

---

## Sources

- Parasolid quality: https://blogs.sw.siemens.com/plm-components/parasolid-delivers-30-years-of-quality-in-industrial-use/ · checker: http://www.q-solid.com/Parasolid_Docs/headers/pk_body_check.html · http://www.q-solid.com/Parasolid_Docs/chapters/fd_chap.09.html · http://www.q-solid.com/Parasolid_Docs_V35/headers/pk_face_check.html · http://www.q-solid.com/Parasolid_Docs_V35/chapters/fd_chap.121.html
- ACIS: http://www.q-solid.com/ACIS_Docs_R17/online/SPAacisgsTechArticles/SPAacisgs_usex.htm · http://www-isl.ece.arizona.edu/ACIS-docs/HTM/DATA/INTR/INTR/03FN/0003.HTM · https://blog.spatial.com/3d-acis/outcome-checking-logging-and-progress-reporting-3d-acis · https://blog.spatial.com/employee-spotlight-behind-the-scenes-how-integration-ensures-spatials-software-remains-reliable-and-secure
- OCCT testing: https://dev.opencascade.org/doc/overview/html/occt_contribution__tests.html · https://github.com/Open-Cascade-SAS/OCCT/wiki/tests · https://dev.opencascade.org/doc/occt-7.6.0/overview/html/occt_user_guides__test_harness.html · https://github.com/Open-Cascade-SAS/OCCT/discussions/1097 · fuzzy booleans: https://dev.opencascade.org/doc/occt-7.4.0/overview/html/occt_user_guides__boolean_operations.html · https://dev.opencascade.org/doc/occt-7.5.0/refman/html/class_b_o_p_algo___options.html
- Conformance forums: https://www.mbx-if.org/home/cax/ · https://www.mbx-if.org/who_we_are.php · https://www.datakit.com/en/step_improving_the_use.php · NIST PMI: https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-validation · https://pages.nist.gov/CAD-PMI-Testing/results.html
- Metamorphic testing: https://eprints.whiterose.ac.uk/id/eprint/110335/1/segura16-tse.pdf · https://dl.acm.org/doi/10.1145/3143561 · https://www.doc.ic.ac.uk/~afd/homepages/papers/pdfs/2016/MET.pdf · GraphicsFuzz: https://dl.acm.org/doi/10.1145/3133917 · https://www.doc.ic.ac.uk/~afd/papers/2020/ECOOP_GraphicsFuzz.pdf
- Oracle problem in scientific software: https://arxiv.org/abs/1804.01954 · https://onlinelibrary.wiley.com/doi/10.1002/stvr.1594
- Robustness theory: https://www.sciencedirect.com/science/article/pii/S0925772107000697 · https://inria.hal.science/inria-00344310/ · https://dl.acm.org/doi/10.1145/77635.77639 · https://people.mpi-inf.mpg.de/~mehlhorn/ftp/ControlledPerturbationGeneralApproach.pdf · https://resources.mpi-inf.mpg.de/departments/d1/teaching/ws09_10/CGGC/Notes/Perturbation.pdf · https://en.wikipedia.org/wiki/Robust_geometric_computation
- Fuzzing precedent: https://talosintelligence.com/vulnerability_reports/TALOS-2020-1225 · https://nvd.nist.gov/vuln/detail/CVE-2020-35628
- Corpora & mesh-boolean evaluation: https://ten-thousand-models.appspot.com/ · https://www.researchgate.net/publication/305217551_Mesh_arrangements_for_solid_geometry · https://www.graphics.rwth-aachen.de/media/papers/339/ember_exact_mesh_booleans_via_efficient_and_robust_local_arrangements.pdf · https://arxiv.org/pdf/2205.14151 · https://openaccess.thecvf.com/content_CVPR_2019/papers/Koch_ABC_A_Big_CAD_Model_Dataset_for_Geometric_Deep_Learning_CVPR_2019_paper.pdf · https://github.com/elalish/manifold/wiki/Manifold-Library
- Property-based testing: https://github.com/BurntSushi/quickcheck · https://getcode.substack.com/p/property-based-testing-5-shrinking
