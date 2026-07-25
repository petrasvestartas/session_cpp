# MASTER-PLAN additions from OCCT round-2 extraction (occt2_*)

Synthesized 2026-07-24 from the 10 round-2 specs (`kb/occt2_*.md`) against the 7 round-1 specs
(`kb/occt_*.md`). Round-2 specs were written non-duplicating by construction ("related specs
referenced, not duplicated"), so every mechanism below is NEW beyond round 1. Per phase: concrete
mechanisms + exact OCCT anchors + the round-2 spec that carries the full detail.

---

## P1 — rotated-solid robustness

Round 1 gave the walker/pavefiller/tolerance core; round 2 adds the ANALYTIC branch, the branch
ENUMERATOR, and the Newton fixes that make rotation survivable.

1. **Axis/center canonicalization so coaxial branches survive rotation** — `AxeOperator`
   (IntAna_QuadQuadGeo.cxx:69-231: `Parallel/Distance/Coplanar/Intersect/Same/PtIntersect`, eps
   1e-14 dist / Precision::Angular()) + `RefineDir` (:2867, ULP snap to exact unit axes). OCCT's
   cyl×sphere / cone×sphere coaxial tests are `Pt.Distance(PtIntersect()) == 0.0` — bitwise; rotated
   imported frames miss them unless the recognizer snaps centers onto axes. Direct fix class for the
   z15/z30 rotated-recognition losses. [occt2_intana-analytic §Stage3, Pitfall 1, PortMap 4]
2. **Full quadric-quadric case table** — `IntAna_QuadQuadGeo::Perform` per pair (15 pairs: pln×pln
   :389 … tor×tor :2588; all 9 coaxial torus-family circle cases, plane×cone 5-conic split :752,
   cone×cone 5 branches :1433 incl. common-generatrix `myCommonGen`/`myPChar`), tolerances
   `InitTolerances` (:351: 1e-14, Angular, 1e-9 mini-circle, 1e-13 RELATIVE radius, Confusion).
   Widens our conic registry from 2 configurations to the whole table; conics are rotation-exact
   once axes are canonical. [occt2_intana-analytic PortMap 1]
3. **Analytic-eligibility numeric gates** — `IntPatch_Intersection::Perform` (~L1090-1338: cone
   semiangle <0.02 / >1.55 rad → biparametric; torus analytic ONLY coaxial; plane×cone axis-in-plane
   0.015), `IntTools_FaceFace::isTreatAnalityc` (L249: needle ellipse Major ≥ 1e5·Minor rejected),
   extreme-conic bails (EllipseLimit 1e9 / HyperbolaLimit 2e6, QuadQuadGeo :925). Adopt into
   `scaffold_eligible` so cone×cone / tilted-torus route by OCCT's criteria, not degree heuristics.
   Master rule: `done=false` anywhere → silent full fallback to marching, never mixed output.
   [occt2_intana-analytic §Stage0-1]
4. **Exact trig ALines for cyl/cone × quadric** — `IntAna_IntQuadQuad::Perform` (IntQuadQuad.cxx
   :375 cylinder / :841 cone; 18 trig coeffs, mirror parametrization, `TwoCurves`), apex split
   `ExploreCurve` (IntPatch_ImpImpIntersection.cxx:8608, tube 10·Tol), ALine→WLine sampling 200 pts
   (`GeomGeomPerfom`), chain links param 1e-7 / 3D 1e-10. Replaces marched chains with
   rotation-invariant exact curves for the cyl×cone family. [occt2_intana-analytic PortMap 3]
5. **Polyhedral branch enumerator (the missing-branch structural fix)** — tri-tri SAT soup
   `IntPolyh_MaillageAffinage::TriangleCompare` (:3141) + BVH `GetInterferingTriangles` (:185) +
   winged-edge chaining `StartPointsChain` (:3333): connected components of the contact graph ==
   section lines; every component seeds. Our single-seed-per-pair misses second components — the
   (18,5) mini-branch class. Rotation creates exactly these grazing secondary branches.
   [occt2_intpolyh-seeding PortMap 1-2]
6. **Tangential 4-shift pass** — `IntPolyh_Intersection::PerformAdv` (:266): four meshes offset
   ±1.5·deflTol along normals; trigger `IsAdvRequired` (0 couples, or ≤10 with |cos|>0.996);
   `MergeCouples` unordered-pair fence. Turns tangential contact into transversal crossings —
   rotated tangent circles become seedable. [occt2_intpolyh-seeding S5]
7. **Multi-seed schedule with UV confinement** — `IntPatch_PrmPrmIntersection::Perform` (:2679:
   nbp/2, 1, nbp-1, 3nbp/4, nbp/4, then EXHAUSTIVE while no line found; walk clamped to the
   polyhedral line's own UV bbox; longest-line-first sort so dedup kills fragments not parents).
   [occt2_intpolyh-seeding S7]
8. **Full second-order projection Newton** — `Extrema_FuncPSNorm::Values` (Extrema_FuncPSNorm.cxx
   :112: Jacobian includes `PPs·Suu/Suv/Svv`). Our `Closest::surface_point` is first-order-only —
   the tangential-stall root cause behind the cut-node crossing snap. Plus degenerate-iso densify
   `IsoIsDeg` (Extrema_ExtPS.cxx:32: |D1|≤1e-9 → 300 samples that direction) for apex/pole feet.
   [occt2_extrema-internals PortMap 1,3]

## P2 — random battery

Round 1 had no test-corpus spec at all; round 2 adds a complete mined suite + new gate invariants.

1. **51-cell ready-to-port grid** (17 pairs × 3 ops + reserves) from `tests/boolean/bop*_simple`
   (375/379/378/373 inline cases): coincidence (A5/B5/ZD4/ZD5/ZP1/ZP2), tangency (F5/Z5/S5/ZD8
   Steinmetz-common exactly 16/ZH5/obcA1/J1 internal line-tangency), grazing (C5/H5/ZP9 dirty
   decimals), thin-wall (K5/ZD2). Conversions solved (corner→centered box; trotate-about-point
   fold `t' = R(t−c)+c`). Target: `occt3_placements()/occt3_pairs()` in main_7.cpp, env
   SESSION_OCCT3. [occt2_occt-test-mining §cell-list]
2. **One intersection, four ops as harness law** — `bop` fills one static `BOPAlgo_PaveFiller`
   (BOPTest_BOPCommands.cxx:150), `bopsmt` (:195) reuses it for COMMON/FUSE/CUT/CUT21. Extends our
   landed shared-closure bridge to the full battery: compute scaffold once per PAIR.
3. **New free invariant gates**: area conservation `S(fuse)+S(common) = S(A)+S(B)` (add AREA to
   oracle.cpp via BRepGProp::SurfaceProperties); duality `tuc(A,B)==cut(B,A)` (BOPAlgo_CUT21);
   seam-twin diff (ZD4 vs ZD5, ZL2 vs ZL5 — operand rotated about own axis, results must be
   identical: a no-oracle seam-aliasing detector); `-s empty` = EXACT zero faces (sliver-filter
   regression harness); checkshape on every result promoted to hard per-cell verdict bit
   (`tests/boolean/end` + parse.rules `/\bFaulty\b/`). checkprops is only 1% relative
   (CheckCommands.tcl `checkprops` ~:520, depsilon 1e-2) — full precision from our oracle.
   [occt2_occt-test-mining PortMap 2,3,6,7]
4. **Unblocking builds**: arbitrary-axis `xf_of()` (use existing `Xform::rotation(Vector&,deg)`),
   `create_cone(r1,r2,h)` truncated cone ×3 langs + oracle `cone3` (unblocks ZF6/ZM5/ZP7);
   cavity reserve oqZP8 (cut → solid with inner shell, fuse-with-void == 600) = the multi-shell
   frontier acceptance cell; gdmlA1 denormal 6.27e-46-degree rotation, gdmlB1 body-diagonal-rotated
   flush fuse = minimal inline chairsROT-class chains. [occt2_occt-test-mining PortMap 4,5,8,11]

## P3 — same-domain

Round 1 (ff-posttreat-samedomain) covered face-face SD post-treatment; round 2 adds the op algebra
above it, the ownership rule, and the seam detectors.

1. **Cell/origin-set algebra** — `BOPAlgo_CellsBuilder::IndexParts` (BOPAlgo_CellsBuilder.cxx
   :113: part → origin-argument set, THE classification computed once by GF) + `FindParts` (:645:
   Take/Avoid = superset/disjoint predicate, purely combinatorial). Store our winding/radial verdict
   as a per-face (IN_A, IN_B) bitmask on the ONE combined split; every op (incl. xor/split) becomes
   a keep-mask — kills the boolean_xor re-run instability. [occt2_brepalgoapi-cells PortMap rows 2-3]
2. **Explicit keep matrix incl. ON row by orientation** — `TopOpeBRepBuild_GTopo` 3×3 IN/ON/OUT
   tables built per op × `TopOpeBRepDS_Config` (SAMEORIENTED/DIFFORIENTED) by `TopOpeBRepBuild_GTool
   ::GFusSame/GCutSame/GComSame/...`. The cleanest artifact of the deleted generation: make our
   cut/common/fuse keep tables explicit with the ON row split by relative orientation.
   [occt2_topopebrep-lessons PortMap 3]
3. **Same-domain detection from geometry, never event order** — anti-pattern `islFFsamdom`
   fill/unfill in `TopOpeBRep_DSFiller::InsertIntersection` (:443, scan-order dependent) vs the
   correct key: `Same` verdict from the QuadQuadGeo case table (pln/cyl/sph/cone/tor same-type
   pairs, exact criteria) feeding the SD path. [occt2_topopebrep-lessons Pitfall 5; occt2_intana-analytic PortMap 6]
4. **Coincidence-ownership rank** — `BOPAlgo_MakePeriodic::SplitShape` (BOPAlgo_MakePeriodic.cxx
   :396-470: tools fed FIRST because "coinciding parts use the geometry of the FIRST argument";
   + `SetGlue(BOPAlgo_GlueShift)` + NonDestructive). Codify deterministic ownership (operand A wins)
   in our exact weld — rank-determinism is what makes twin geometry EXACTLY identical.
   [occt2_brepalgoapi-cells §D, Invariant 8]
5. **Glue tiers as SD fast path** — `BOPAlgo_GlueEnum` (GlueOff/GlueShift skips FF; GlueFull skips
   VF/EF too; unchecked trust-me flags). A declared-coincident-walls fast path for chairs-family
   repeated cuts. [occt2_brepalgoapi-cells §E]
6. **Boundary pinning when dissolving internal boundaries** — `CollectMaterialBoundaries` (:1153:
   sub-shape with exactly 1 ancestor in group = area boundary) → `ShapeUpgrade_UnifySameDomain::
   KeepShapes`; unique-face rule (1 owner = exterior, 2 = internal) + rollback when BuilderSolid ≠ 1
   area. Our cell-fusion primitive for merge/fuse of selected regions. [occt2_brepalgoapi-cells §A6]
7. **Seam-needed detector** — `ShapeFix_Face::FixMissingSeam` static `CheckWire` (ShapeFix_Face.cxx
   :1652: wire 2d closure vector Σ(c2d(l)−c2d(f)) ≈ ±period within 10% ⟹ face needs a seam edge)
   + `ShapeFix_Wire::FixShifted` (:1661: AdjustByPeriod against previous edge / wire-box vs surface
   middle, 0.2·range dead-band, degeneracy-axis lock, copy-on-write pcurves). Period-normalize UV
   chain keys before hashing — the missing canonicalization behind the fb=47.99997 alias-key drop.
   [occt2_shapehealing PortMap 6,7]

## P4 — EE/EF interference

Round 1 (interference-vef) had the verdict tables; round 2 supplies the underlying projection /
extrema ENGINES and their exact budgets.

1. **Projection engine parity** — grid density 32/44 (+300 degenerate-iso), knot-aware seeds
   `fillParams` (Extrema_GenExtPS.cxx:321: max(degree,2) per knot span), border-inset grid, sentinel
   ring, three-level distance field `ComputeEdgeParameters` (:460), bounded quarter-domain-step
   Newton `math_FunctionSetRoot::Perform` (math_FunctionSetRoot.cxx:796). [occt2_extrema-internals §A]
2. **Multi-solution accumulation** — `Extrema_FuncPSNorm::GetStateNumber` (:139: every Newton
   launch appends into ONE store, UV dedup 1e-18 sq) → build `surface_points_all`; equidistant twin
   feet on periodic surfaces currently break tube merge / radial classification.
   [occt2_extrema-internals PortMap 4]
3. **Warm per-entity projector cache** — `IntTools_Context::ProjPS` (IntTools_Context.cxx:247: ONE
   projector per face, restriction-UV-bounds init, tol 1e-12, MIN flag) reused across thousands of
   queries; decoupled verdicts `ComputeVE` (:499: −1 fail / −2 distance > tolV+tolF+fuzz / −3 UV
   classified OUT) — distance and containment are separate questions. [occt2_extrema-internals §D]
4. **Cheap CC pre-verdicts** — `Extrema_ExtCC::Perform` endpoint-pair distances first
   (`TrimmedSquareDistances`); parallel verdict needs three independent confirmations; range accept
   at RealEpsilon (no slack — unlike PS/CS ±tol). [occt2_extrema-internals §C]
5. **Large-UV termination fix** — `Extrema_GenLocateExtPS::CorrectTol` (Extrema_GenLocateExtPS.cxx
   :29: escalate tol by 10^n when Epsilon(U0) > Epsilon(1)); `IsMinDist` 3×3 stencil (:63) rejects
   saddle feet. Imported STEP domains at 1e2-1e4 UV hit this. [occt2_extrema-internals PortMap 9,10]
6. **Micro-edge law, exact algorithm** — `BRepLib::FindValidRange` / `findNearestValidPoint`
   (BRepLib_1.cxx:31-309: endpoint-tolerance-sphere subtraction by march step Resolution(tol)·1.01 +
   bisection to anEps; whole-edge-inside-sphere ⟹ micro edge, weld don't emit). Round 1 only
   invoked it; round 2 gives the port. Kills pave-sphere slivers AND rescues grazing-but-valid
   blocks. [occt2_breplib-sameparameter §5, PortMap 5]
7. **Adjacent-pair intersection ladder** — `ShapeAnalysis_Wire::CheckIntersectingEdges`
   (ShapeAnalysis_Wire.cxx:1360: operand-order-asymmetric Geom2dInt matched with BRepCheck; test at
   tolt = min(tolVtx, max(tolEdge, prec)) — deliberately BELOW vertex tol) + fix ladder edge-tol →
   vertex-tol → cut (ShapeFix_Wire.cxx:2966). [occt2_shapehealing §7b]

## P5 — tolerance model

Round 1 (tolerance-model) gave the laws; round 2 gives the two machines that ENFORCE them.

1. **Same-parameter doctrine (whole spec new)** — `BRepLib::SameParameter` per-edge core
   (BRepLib.cxx:1251-1740): `ComputeTol` 23-sample deviation with out-of-chart parametric penalty
   (uv beyond 1% span → DSdu·overshoot) and <10% outlier vote, ×1.5 floor 1e-7 (:1070-1188);
   C0→C1 pcurve repair + bad-knot curvilinear re-approx; `Approx_SameParameter::Build`
   (Approx_SameParameter.cxx:318: cubic reparam function, accept < 250·besttol, accept-if-not-worse);
   epilogue = THE one sanctioned tolerance SHRINK (`TE->Tolerance(maxdist)` :1719-1737, every rep
   just measured; vertices never shrunk). [occt2_breplib-sameparameter §2-3]
2. **Same-parameter BY CONSTRUCTION for new section curves** — ApproxInt coupled fit: ONE
   least-squares system, shared knot vector, channels = C3d + pcurveA + pcurveB
   (`AppParCurves_MultiCurve`; params from 7-dim chords `ApproxInt_Approx::Parameters`); verified
   ladder accept iff MaxE3d≤tol3d AND MaxE2d≤tol2d (`Approx_ComputeLine::Compute`); curvature knot
   seeding `ApproxInt_KnotTools::BuildKnots` (aSinCoeff2=(3−√5)/8, ratio 3.0, 15× cap); ONE Hoschek
   projection step (BFGS disabled: NbIterMax=0 in production); TolReached feeds edge tolerance;
   degree-1 polyline fallback ALWAYS produced. Our chains are discretely same-parameter already —
   this is the compression/export contract (STEP writer, Rhino trim rejection).
   [occt2_approx-geomint §5-11, PortMap 1-4]
3. **Multi-representation edge record** — `BRep_TEdge` (one 3D curve + N pcurves + tol +
   SameParameter/SameRange flags; rep uniqueness per (surface,location); seam =
   `BRep_CurveOnClosedSurface` with TWO pcurves addressed by orientation). Build the shared
   `EdgeGeom` referenced by both faces — alias keys become pointers, not tolerance lookups; the
   seam model is the missing piece for scaffold-on-analytic with seam trims.
   [occt2_breplib-sameparameter PortMap 1,7]
4. **Vertex fusion + harmonization primitives** — `BRepLib::BoundingVertex` (:3013: minimal
   enclosing sphere pair formula, coordinate-sorted barycenter n>2 — deterministic welds);
   `InternalUpdateTolerances` (:1744: V≥E≥F bottom-up, vertex covers EVERY rep endpoint + 2·Epsilon).
   [occt2_breplib-sameparameter §6-7]
5. **Options contract** — `BOPAlgo_Options` fuzzy clamp `max(f, Precision::Confusion())`
   (BOPAlgo_Options.cxx:105); `BooleanOptions{fuzzy, glue, safe, fill_history}` struct replacing
   stable env gates. [occt2_brepalgoapi-cells §E]
6. **Healing tolerance grammar** — the exact growth factors (1.0001 CombineVertex, 1.001 growth,
   1.00001, 1.0000001 cover), canonical 2d tolerance `2·max(URes(tol),VRes(tol))`, 3-tier
   `CheckConnected` ladder (gp::Resolution / precision / maxtol), auto-precision clamp
   `min(preci, LeastEdgeSize/2)·1.00001`. Consumed by scaffold welds/bridges. [occt2_shapehealing]

## (proposed) P5.5 — validity gates + targeted healing — see verdict section below

## P6 — history / corpus / scale

1. **History composition machinery** — `SetToFillHistory` kill-switch semantics (off ⟹ empty
   answers, BRepAlgoAPI_BuilderAlgo.cxx:204-252); history SUSPENDED while the intermediate result
   will be discarded (`BOPAlgo_CellsBuilder::PerformInternal1` :86); overlay-map composition
   `CellsBuilder::LocModified` (:1040: GF images → local unification map, chained) rather than a
   monolithic log; `HasAncestorFaceOn1/2` (BRepAlgoAPI_Section.cxx:224: section edge → generating
   face pair is part of the public contract). Anti-pattern: `TopOpeBRepBuild_HBuilder` retrofit —
   history must be first-class from the start. [occt2_brepalgoapi-cells §A,E; occt2_topopebrep-lessons Pitfall 9]
2. **Derived-operation layer** (one split, many products) — `BOPAlgo_Splitter` (tools intersect but
   are excluded from result by construction, BOPAlgo_Splitter.cxx:54-121), `BOPAlgo_MakerVolume`
   universe-box trick (both orientations of every face + enclosing box; remove the ONE solid
   touching box faces, BOPAlgo_MakerVolume.cxx:102-333), `BOPAlgo_MakePeriodic` twin association.
   Filler injection (`myIsIntersectionNeeded=false`) = the OCCT idiom for our compute-network-once
   goal lifted ABOVE the op switch. [occt2_brepalgoapi-cells]
3. **Corpus scale-out** — grids.list 35 grids; cells_test (68) for future general fuse;
   volumemaker (74) at ±1e6 extents = large-extent arrangement stress; gdml chains; the mining
   pipeline (case → Place conversion → oracle cache → scorecard) is repeatable for all of them.
   [occt2_occt-test-mining]
4. **Scale robustness specifics** — CorrectTol epsilon escalation (P4.5 above) at 1e6 UV; healing
   2d intersector floor 1e-10 is ABSOLUTE (starves on unscaled models); quadric coefficient
   normalization before the 1e-8 trig-root residual test (else far-from-origin frames demote
   analytic pairs to marching). [occt2_extrema-internals; occt2_shapehealing Pitfall; occt2_intana-analytic Pitfall 8]

## P7 — deletion + ports

1. **The deletion case study** — TopOpeBRep* (384 files) is OCCT's own deleted boolean generation;
   the three failed bets: (a) repair-the-DS-afterwards (`CompleteDS` cascade of 12+ dated passes
   `FUN_ds_completeforSE1..SE9`), (b) single-point deferred classification
   (`ClassifyEdgeToFaceByOnePoint` at PAR_T=0.43213918), (c) capped per-pair scalar tolerance
   (`FTOL_FaceTolerances` sum capped at ABSOLUTE 1e-4). Codified rules for our tree: fix verdicts
   where computed, never append repair passes; quorum classification, never one probe; per-entity
   growing tolerances, never absolute caps; every special/fast path passes the general path's gates
   (KPart forest = our tor×tor hijack class); no mutable-global switches (`GLOBAL_USE_NEW_BUILDER`
   half-migration froze both paths); in-pipeline validation with hard failure (their `Checker` was
   an NYI stub). [occt2_topopebrep-lessons]
2. **What NOT to port (negative port list)** — BFGS gradient machinery (production runs
   NbIterMax=0); RatioTol=1.5 double-speak ("should be removed" per OCCT's own comment);
   transition-interference bookkeeping (the failure core of the old gen); pcurve-translation
   periodic repair (`CORRISO` — seam-split up front instead); math_PSO for CC extrema (interval
   products suffice); statuses declared-never-emitted (`Invalid3DCurve`, `InvalidToleranceValue`).
   [occt2_approx-geomint Pitfalls; occt2_topopebrep-lessons; occt2_brepcheck-validity]
3. **Port-order dependencies surfaced by round 2**: EdgeGeom record (P5.3) before identity-based
   combine; branch enumerator (P1.5) before multi-seed march; BRepCheck gates (P5.5) before corpus
   scale-out (checkshape is the battery verdict bit); create_cone/xf_of before reserve cells.

---

## ShapeFix/BRepCheck robustness layer — NEW phase between P5 and P6?

Question: do `occt2_shapehealing` + `occt2_brepcheck-validity` justify a standalone phase
("P5.5 validity gates + targeted healing"), or should their content dissolve into P1-P5?

### For a new phase (evidence)

1. **BRepCheck IS our acceptance oracle already.** Every "OCCT-VALID" verdict in validation/ is
   `BRepCheck_Analyzer::IsValid` — but we emit shapes without knowing its exact budgets. Round 2
   shows FOUR different tolerance budgets (vertex-on-curve max(tolV,tolE); wire 3D closure max of
   two VERTEX tols only; self-intersect escape 1.1·tolV; yawn wedge 2·tolE) and loose spots (1%-of-
   UV-span fast accept; 23-point sampling not proof). Emitting-to-pass requires a dedicated,
   coherent effort: measured vertex tol ×1.05, edge tol containing 23-sample deviation ×1.00001,
   `Closed(true)` flag semantics, aMEToAvoid + degenerated exemptions in naked counts
   (`BRepCheck_Shell::Closed`), opposite-use rule (`BRepCheck_Shell::Orientation`). Scattering
   these across phases loses the "one contract" coherence.
2. **The audit side is the piece the deleted generation never wrote.** `TopOpeBRep_DSFiller::
   Checker` = NYI stub is Pitfall 8 of occt2_topopebrep-lessons; BOPAlgo survived partly because
   validation moved in-pipeline. Our equivalents (naked-edge audit, closure gates) are diagnostics,
   not hard typed verdicts. A phase gives them: wire vertex-parity (`BRepCheck_Wire::Closed`
   even-valence), shell parity/orientation + reorientability flood, face imbrication
   (`ClassifyWires`: exactly one outer wire, holes reversed), solid single-growth probe
   (aPAR_T=0.43213918), same-param 23-point gate (`BRepLib_ValidateEdge`) — each mapping a known
   silent failure class (all-faces-flip, SEGLOST holes, partial lifts) to a typed status BEFORE the
   oracle sees it.
3. **Natural ordering.** Healing currency #1 is "grow a tolerance" — it REQUIRES the P5 per-entity
   tolerance model as substrate; and P6 corpus scale-out needs the checkshape-equivalent verdict
   bit as its per-cell gate (occt2_occt-test-mining PortMap 2). P5 → P5.5 → P6 is a real
   dependency chain, not an insertion of convenience.
4. **Imported-input reality.** The chairs campaign is dirty-STEP input; ShapeFix is exactly OCCT's
   pre-boolean answer to it (auto-precision clamp, FixShifted period normalization, singularity
   split, seam synthesis). Several chairs failure classes map 1:1 onto named fixes (fb=47.99997 →
   FixShifted; asymmetric span-crossing bridge unions → FixLacking's 2d/3d cross-check + 0.9π
   zigzag guard; 0.008 spurious splits → Middle-transition + vertex-tol-sphere escape).

### Against a new phase (evidence)

1. **Repair-cascade hazard.** occt2_topopebrep-lessons' #1 rule: fix at the source, never append
   repair passes. A standalone "healing phase" institutionalizes exactly the post-hoc-repair
   architecture that killed TopOpeBRep. Notably, the shapehealing spec's own port map does NOT
   propose a wire-fixer: 10 of its 12 rows say ADOPT a predicate/threshold INTO an existing anchor
   (closure-weld, valence-1 bridge, alias keys, NK-RESCUE acceptance) — i.e. the value dissolves
   into P1/P3/P4/P5 items.
2. **Scope asymmetry.** Full ShapeFix_Wire/Face is ~10k lines of order-dependent, mode-flagged,
   30-iteration-capped fixes tuned for arbitrary IGES/STEP garbage. We control both ends of our
   pipeline except imported chairs; our own writer output is already exact. Porting the machine
   (vs its predicates) competes for a phase-slot with the boolean frontier at poor ROI.
3. **The audit could be absorbed**: BRepCheck gates fit inside P2 (battery hard gate, already a
   port-map row) and P5 (measured-tolerance contract) without renumbering phases.

### Verdict

**Adopt a NARROW new phase P5.5 = "BRepCheck-parity gates + write-to-pass tolerance emission" —
audit-first, healing-only-by-predicate.** In scope: (a) the five in-pipeline typed gates (wire
parity, shell parity+orientation flood, face imbrication, solid growth, 23-pt same-param audit)
with hard failure; (b) emitting measured tolerances that pass BRepCheck by construction (×1.05 /
×1.00001 margins, flag semantics); (c) oracle-side exact-mode analyzer (theIsExact=true) for
frontier cells. Explicitly OUT of scope: a general ShapeFix_Wire port — every healing mechanism
stays where its port-map row put it (P1/P3/P4/P5 anchors), honoring the fix-at-source rule. This
keeps the P5→P6 dependency chain real (gates precede corpus scale-out) while refusing the
repair-cascade architecture the deletion case study warns against.
