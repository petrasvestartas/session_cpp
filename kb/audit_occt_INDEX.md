# audit_occt_INDEX — routing note for the 8 OCCT source audits

Ground truth for all eight: real OCCT tree at `/home/petras/code/code_cpp/OCCT` @ `37dd5686f2` (8.0.1.dev).
Each `audit_occt_*.md` falsifies the same-topic spec written earlier from memory; **where an audit and its
spec disagree, the audit wins**:

| audit | supersedes |
|---|---|
| `audit_occt_ssi-walking.md` | `occt_ssi-walking.md` |
| `audit_occt_intpolyh-seeding.md` | `occt2_intpolyh-seeding.md` |
| `audit_occt_pavefiller-core.md` | `occt_pavefiller-core.md` |
| `audit_occt_ff-samedomain.md` | `occt_ff-posttreat-samedomain.md` |
| `audit_occt_builder-assembly.md` | `occt_builder-assembly.md` §15 |
| `audit_occt_intana-analytic.md` | `occt2_intana-analytic.md` |
| `audit_occt_tolerance-model.md` | `occt_tolerance-model.md` |
| `audit_occt_blowup-guards.md` | (no spec — source-derived baseline) |

## The eight files

- **`audit_occt_ssi-walking.md`** — IntWalk_PWalking marching, PutToBoundary/SeekPointOnBoundary boundary
  landing, JoinWLines, purge/densify policy. **Top finding:** `DistanceMinimizeByGradient` is a documented
  no-op unless BOTH surfaces are Bezier/BSpline (PWalking:2413-2422) — for analytic operands the entire
  boundary landing is `HandleSingleSingularPoint`'s locked-iso Int2S corrector, run on a *pristine* copy that
  overwrites the minimizer result (:2750/2779/2782).
- **`audit_occt_intpolyh-seeding.md`** — IntPolyh triangulation, deflection refinement, contact detection,
  start-point extraction, section-line chaining, seed scheduling. **Top finding:** branch coverage is **not**
  guaranteed — a couple whose `StartingPointsResearch` returns 0 is marked analyzed and yields nothing
  (MaillageAffinage:3369-3370, 3602-3604), and chaining needs an un-analyzed ordered (T1,T2) match (:3197), so
  one connected component can split into several lines or none.
- **`audit_occt_pavefiller-core.md`** — BOPDS data structure, pave/pave-block lifecycle, VV/VE/EE/VF/EF passes,
  SplitPaveBlocks, UpdateVertex/RepeatIntersection, aTolFF plumbing. **Top finding:** the watertightness gate
  the spec omits — EE fuses into a CommonBlock only when `aNbCPrts==1 && HasSameBounds` (PF3:530-540) and EF
  only when `CheckFacePaves` succeeds for **both** pave vertices (PF5:423-439, 553-559); misaligned pairs are
  recovered later by ForceInterfEE/EF after vertex unification. That two-phase structure is the Law-1 core.
- **`audit_occt_ff-samedomain.md`** — AreFacesSameDomain, IsValidPointForFace, hatcher interior points,
  BOPTools_Set bucketing, FillSameDomainFaces, PostTreatFF. **Top finding:** the Builder never consults
  `TangentFaces`; SD candidates are HasFaceInfo-filtered FF faces bucketed by identical `BOPTools_Set` edge sets
  (Builder_2.cxx:653-748), so PostTreatFF's cross-pair edge fusion (shared TShape on both faces) is a hard
  precondition for SD detection ever firing.
- **`audit_occt_builder-assembly.md`** — BuildSplitFaces/BuildDraftSolid/BuilderSolid assembly and the
  `BuildBOP` selection op-table. **Top finding:** the hasher choice *is* the mechanism — `aMFence` (IsSame,
  orientation-blind) then `aMFenceOri` (IsEqual, orientation-sensitive) at Builder.cxx:682/696/714; collapsing
  the two identities destroys every ON-same/ON-opposite row, and `aMResFacesFence` (:700) is what makes CUT
  keep the objects' orientation while CUT21 keeps the tools'.
- **`audit_occt_intana-analytic.md`** — IntAna_QuadQuadGeo case table, IntAna_IntQuadQuad/IntAna_Curve ALines,
  ImpImp consumption, transition/tangency encoding. **Top finding:** tangency has **three** encodings, not one
  — `IntPatch_Point(isTangent)`, the Situation constructor (`tS1=tS2=IntSurf_Touch` + Inside/Outside from a
  curvature-centre test, IntPatch_Line.cxx:35-48), and transversal-with-unknown-sign = Undecided; the spec
  collapses the second into the third and loses OCCT's inside/outside verdict.
- **`audit_occt_tolerance-model.md`** — every tolerance writer/reader, fuzzy plumbing, ValidateEdge, E/F
  criteria, provisional growth. **Top finding:** tolerance is **not** monotone non-decreasing (five hard-set,
  shrink-capable writers: SameParameter :1734, UpdateEdgeTol :685, UpdShTol force :889-892, UpdateFace :604),
  and the missing layer is provisional growth + rollback: `aMVTol` (PF6:699) saves pre-growth vertex tolerances,
  the acceptance predicate reads the **saved** value (:2974), and growth is hard-reset per FF pair (:1073-1095).
- **`audit_occt_blowup-guards.md`** — every count/iteration/step bound OCCT imposes on intersection and
  splitting; feeds task #9. **Top finding:** OCCT has **no memory guard in the FF pass** —
  `BOPTools_Parallel::Perform` (PF6:529) dispatches all face pairs with no chunking and keeps every pair's
  `IntPatch_Intersection` + `mySeqOfCurve` alive until the serial loop ends at PF6:621; structurally the same
  balloon as SYMEMIT, so #9 has no knob to copy — it needs a streaming/chunked FF pass OCCT does not have.

---

## For session A

### PutToBoundary port (M2, `occt_ssi-walking.md` PORT MAP #1)
- `ssi-walking`: for our analytic chairs operands, **port the corrector, not the gradient minimizer** —
  `DistanceMinimizeByGradient` returns true without touching the point (PWalking:2413-2422) and
  `SeekAdditionalPoints` degenerates to a near no-op. The whole mechanism is `HandleSingleSingularPoint`:
  locked-iso Int2S, 3 free params hard-clamped to the **natural** surface box (ZerParFunc.gxx:248-311),
  acceptance `|P1-P2| <= TolTangency`, emitted point = midpoint (ZerParFunc.lxx:27-30).
- `ssi-walking`: it is not a sequential refinement — `aSingularPnt` is the **pristine** input snapshot
  (:2750) and a successful corrector run **overwrites** any minimized point (:2779/2782). Port that order.
- `ssi-walking`: **PutToBoundary can shrink the line below 3 points**; hairpin cleanup deletes END points (not
  the scanned index) and aborts insertion entirely when no usable pair exists (:2874/2861). Only one of OCCT's
  two call sites re-checks `NbPoints<3` — our call site must always re-check, or M2 trades junction gaps for
  dropped chains.
- `ssi-walking`: drop the "walk in a shrunk box then extend" design (spec error). `Perform(ParDep,u1min..v2max)`
  uses the UV box only in `ComputePasInit` (:783); the marching domain stays the constructor's widened bounds,
  and the 6-arg ctor every PrmPrm call site uses has non-periodic KELARG widening **commented out** with
  NEWRESO capped <10 (:337/469/517).
- `ssi-walking`: `aNbIterMAX=60` / `aNbIter=10` are failure/success counters, not loop bounds, and the extrema
  Hessian off-diagonal is inexact on purpose (:2574) — port literally, do not "fix".
- `ssi-walking`: densified lines get `EnablePurging(false)` permanently (PrmPrm:2882/3143) and
  `ExtendLineInCommonZone` appends its buffer even when it returns false (:2373-2384) — both change what M2
  leaves behind for M1/M3.

### SEED2 (polyhedral branch enumerator)
- `intpolyh-seeding`: **instrument per-couple "produced a start point?"** — component count is not a branch
  budget (components <= section lines, and some components contribute zero seeds). This is the single most
  useful SEED2 instrument and directly measures the class we keep losing.
- `intpolyh-seeding`: the **disproportion rule is inverted** in the spec. Real test is
  `diag_i = |box_i diag|^2/(NbU_i*NbV_i)`; if `diag1<diag2`, compare `FlecheCritique2` against `diag1` and
  refine surface **2** (MaillageAffinage:1341-1386). Our small-cutter x large-base pairs are exactly the case
  the spec's version gets wrong on every pair.
- `intpolyh-seeding`: the seed schedule double-increments after the 5 spread picks (PrmPrm:2706-2708) so the
  fallback scan visits indices 3,5,7,9,… — every other point. The spec's loop condition (`|| !lignetrouvee`)
  **hangs** when no line is found; real condition is `(k-3<nbp || lignetrouvee)`.
- `intpolyh-seeding`: **do not port PORT MAP #10** (enlarge-zone retry) — it is provably dead code
  (`SetIntersectionPossible(true)` exists nowhere; pass 2 starts fully culled and returns 0).
- `intpolyh-seeding`: reject `alpha=beta=RealLast()` start points — the terminal else of the 6-branch ladder
  emits ~1e308 UV and OCCT does **not** filter it. Also reset `CoupleAngle` per couple: it is declared once
  outside both loops (:3151) and `TriContact` only writes it when both `|n|^2 > 1e-24` (:1657-1667), so a
  degenerate contact inherits the previous couple's angle (or -2.0, whose |cos|>0.996 reads as near-parallel)
  straight into our `incidence` metric.
- `intpolyh-seeding`: dedup is bidirectional and loose (AddWLine deletes stored lines whose midpoint and every
  vertex lie on the new one, PrmPrm:4085-4144; `IsPointOnLine` compares signed perpendicular components and
  accepts within one segment length of an endpoint, :3993-4081) — `SeuildPointLigne = 15*Increment^2` is a
  sentinel flag, not a distance. Our pre-walk dedup must not use it as a threshold.
- `intpolyh-seeding` + `blowup-guards`: grid-density calibration — OCCT's real densities are plane 2x2,
  quadrics/revolutions 15x15, default 10x10, clamped >=6x6, BSpline capped 50/1001, and `IsUniformSampling()`
  is true for everything but BSpline so a mixed analytic x BSpline pair discards the knot arrays. Hard ceiling
  is `NBMAXUV=30` (31x31 = 1922 triangles, Polyhedron:33/43/52/82) — a 24x24 SEED2 grid is inside OCCT's
  ceiling but denser than what OCCT actually uses for every analytic pair we have.

### Task #9 blowup guards (SYMEMIT 4.7-5.5 GB)
- `blowup-guards`: stop looking for an OCCT memory cap — there is none in BOPAlgo. The fix is architectural
  (chunk/stream the FF pass; free each pair's intersector after its section curves are consumed).
- `blowup-guards`: the only hard ceilings are counts — `RejectIndexMAX=250000` appended points **per walking
  line**, initialised once at PW:835 and **never reset on walk reversal**, and **bypassed entirely** by the
  tangent-zone continuation (`ExtendLineInCommonZone` flushes via AddAPoint at PW:2381 without touching
  RejectIndex, bounded only by nbIterWithoutAppend>20 / nbEqualPoints>20). Near-tangential pairs are exactly
  the SYMEMIT class, so any point budget we add must cover the continuation path too. (This corrects
  `occt_ssi-walking.md:208`.)
- `blowup-guards`: the memory dial is `UVMaxStep` and **its 1e-3 floor is commented out**
  (IntPatch_Intersection.cxx:163); `DefineUVMaxStep` returns 1e-4 whenever a cone apex / sphere pole lies
  within (Confusion,1e-5) of the other surface (:2372-2406), and `pasMax = 0.2*Increment` — ~10x more points
  for the whole pair. Re-enabling that floor is a one-line guard for our worst cells.
- `blowup-guards`: approximation failure **moves** memory rather than freeing it — the reApprox retry is
  single-shot and, when the approximator gives up, `GeomInt_IntSS::MakeBSpline` (FF:1388-1407) turns a
  250k-point line into a 250k-pole curve. Cap before the approximator, not after.
- `blowup-guards`: failure is degradation, not abort — a failed pair yields `aFF.Init(0,0)` + warning
  (PF6:545-552), UserBreak is recorded as an ERROR so `HasErrors()` cannot distinguish cancel from geometric
  failure, and alerts-with-shape never merge into a report whose `myLimit=-1` is never set — one retained shape
  compound per failed pair is itself a leak vector. Our guard must both bound and *report* distinguishably.
- `blowup-guards`: re-entrancy is data-driven, not counted — the nested PostTreatFF PaveFiller
  (`SetIsPrimary(false)`, PF6:1195-1197) disables ForceInterfEF and NonDestructive but **not** ForceInterfEE
  (PF3:997), and the FF recheck loop grows `aNbFF` mid-iteration (:1070) bounded only by `i < aNbFFPrev`
  (:879). Any recheck queue we add (M1) needs an explicit round cap.
- `ssi-walking`: `EnablePurging(!hasBeenAdded)` means every densified line is permanently exempt from
  `ComputePurgedWLine` — a second retention path feeding the balloon.
- `tolerance-model`: per-FF-pair state is bounded in OCCT by the `aMVTol` hard reset + from-scratch box rebuild
  at end of pair (PF6:1073-1095); if we retain growth globally we retain the map too.

### Law-1 architecture gate (shared pave-block graph, if M1 doesn't reach 0)
- `pavefiller-core`: the imprint law is **two-phase**, not "match everything" — CommonBlocks form only where
  partitions already agree (EE `aNbCPrts==1 && HasSameBounds`, PF3:530-540; EF both pave vertices already in
  the face's VerticesOn/In, PF5:423-439/553-559), and everything else is recovered by a **second forced pass**
  (ForceInterfEE/EF) after vertex unification. Design the shared graph with that second pass built in.
- `pavefiller-core`: the CommonBlock construction subsystem is missing from every spec we have and is exactly
  M3's union-find — `BOPAlgo_Tools::PerformCommonBlocks` (PB->list<PB> :107-187, PB->list<face> :191-244) does
  connected components, skips groups <2, **reuses the first member's existing CB** and merges all members' face
  lists; `ComputeToleranceOfCB` (:248-356) samples 11 interior points at `dt=(t2-t1)/12` over the
  **representative's** range and takes `max(tol, Tolerance(mate)+LowerDistance)` over every mate edge and
  attached face. Port this instead of pairwise NK-RESCUE.
- `pavefiller-core`: the DS needs two distinct "gone" states — `PaveBlock::Update` emits nothing and clears the
  extras when `aNb<=1` (BOPDS_PaveBlock.cxx:263-268), which is the normal way an edge gets an **empty** PB list
  = "deleted", whereas `reference == -1` = "untouched, reuse the original edge" (BOPDS_DS.cxx:1503-1543).
- `pavefiller-core`: PORT MAP #10 ("no valid range => unify, never drop") is only 2/3 right — the real decision
  is three-way (PF2:464-508): `nV1==nV2` drops the sub-block with **no** weld; `!HasShrunkData` or
  (HasShrunkData && !IsSplittable && ComputeVV==0) welds via MakeSDVertices under a `BOPDS_Pair` fence and does
  **not** append the block; `InitPaveBlocksForVertex` is deferred to after the whole loop (:620-625).
- `pavefiller-core`: `UpdateVertex` returns a **different** index for original vertices (non-destructive
  copy-on-write, PF10:105-162) — every caller must use the return value, `myIncreasedSS` is keyed on the passed
  index, and `RepeatIntersection` therefore tests both `i` and its SD image (PF:373-389). `tolerance-model`
  adds: that copy-on-write fires **unconditionally**, with no `tolV<tolNew` guard, and
  `UpdateCommonBlocksWithSDVertices` abuses it (`UpdateVertex(nV, Confusion)`) purely to mint SD images.
- `pavefiller-core`: iterator ordering is inverted in the spec — `BOPDS_Iterator::Value` puts the
  **lower-dimension** shape first (swap when iT1<iT2 on raw TopAbs enum, :226-243), and the pair loop is a
  merge-walk over range-sorted pairs with a break (:307-357), correct only because `aPairSelector.Sort()`
  orders by ID1. Our pair enumeration must match or the flags below land on the wrong shape.
- `pavefiller-core`: `aTolFF` is three different numbers (SSI `max(seamShift, ToleranceFF)` with the 5e-6
  free-form floor, PF6:495/3939; MakeBlocks `max(tolF1,tolF2)` for vertex reuse, :750; per-block
  `max(curve.Tolerance, curve.TangentialTolerance)`, :886), with adoption cap
  `aMaxTolAdd = min(0.001, 10*(tolR3D+fuzz))`, x2 for common blocks and a tangency branch (|cos|>=0.9063,
  aCoeff=2) at :2106-2196 — do not collapse them into one band.
- `pavefiller-core`: indexing/flag semantics for the DS port — 0-based throughout, `myFlag` defaults -1 and
  `HasFlag()` is `myFlag>=0` so flag value 0 (face index 0) is valid, `prepareEdges` sets flag=edgeIndex then
  `prepareFaces` **overwrites** it with the owning face index (:1674 then :1745), `prepareSolids` is a no-op
  unless there is exactly one argument (:1789-1792).
- `tolerance-model`: M4's "growth-only, never shrink" invariant is **ours, not OCCT's** — only
  UpdateEdge/UpdateVertex are max-monotone. What OCCT actually does is provisional growth + rollback
  (`aMVTol`), plus a real reducer `FilterPavesOnCurves` (PF6:2437-2534) that removes paves only when far
  (>100x band) **and** obliquely projected (sin<0.5) and then shrinks vertex tolerance to
  `max(saved, sqrt(maxDistKept)+Confusion)`. Adopt the save/read-saved/commit-or-reset discipline before
  adopting any growth rule.
- `tolerance-model`: the E/F criterion is `1.5*tolE + tolF` for BSpline/Bezier edges (`max(tolE,tolF)` when the
  ratio exceeds 100 either way, plain sum only for analytic edges; IntTools_EdgeFace.cxx:532-547), both
  operands already padded by fuzz/2; AABB gaps get fuzz/2 per operand but OBBs get **full** fuzz
  (BOPDS_Iterator.cxx:345-346); fuzzy is dead in BuilderSolid (set, never read) and is **not** propagated to
  PostTreatFF's nested PaveFiller, which runs at default Confusion.
- `tolerance-model`: analytic Line/Circle and Line/Plane widened tolerances are **clustering radii only** —
  they raise the BVH gap and pair predicate in `BOPAlgo_Tools::IntersectVertices` (:1076-1108) and nothing
  else; the fused vertex tolerance comes from `BRepLib::BoundingVertex` over the members' **real** tolerances
  (exact 2-sphere minimal enclosing formula :3048-3068, lexicographically sorted barycenter for n>2 at :3090).
- `ff-samedomain`: PostTreatFF details that change M3 — `aLS` is assembled in **reverse** (k=aNbS..1) with
  existing edges appended last and mutated mid-read-back; the micro-PB gap/2 mutual inflation is confirmed
  (:1345-1358); pave-vertex SD is accepted unconditionally only on **exact** parameter equality (:1577); the
  final `aDMNewSD` closure is a **single pass**, not transitive (:1659-1668), so our fixpoint walk is a
  deliberate improvement — record it as such; a nested-filler error aborts the entire section stage
  (:1393-1397). **Do not port** the latent index bug: point couples record the loop counter `i` (PF6:787)
  while edges record `aCurInd` (:1039), and on a recheck iteration that index is out of range of `aFFs`.
- `intana-analytic`: analytic-path corrections that affect the AUTO ladder and tangent cells — cyl x cyl
  coplanarity epsilon is `Precision::Confusion()=1e-7`, **not** 1e-14 (QuadQuadGeo.cxx:1054-1057), so
  equal-radius cylinders whose axes miss by 1e-9 do get exact Steinmetz ellipses; transition dead-bands are
  non-uniform (bare `>0.0` for plane pairs, 1e-9, 1e-7, 1e-8 depending on site); cone contacts are purged by a
  nappe test (`LineParameter >= paramapex`, IntCoCo:8858, IntCoSp:9245); cyl x cone external tangency is
  analytically **unreachable** and yields nothing; `IntAna_Curve`'s discriminant snap is one-sided (:353) so
  ALine evaluation is total and silently returns the tangency vertex outside the true curve.

## For session B

Target: `src/brep_samedomain.h/.cpp` (`SDEdgeSig`, `SDFaceKey`, `SameDomain::detect`, `sd_select_face`,
`sd_classify_face`) and `kb/p3_integration_notes.md` §4.

### Same-domain detection (`audit_occt_ff-samedomain.md`, cross-checked by `audit_occt_builder-assembly.md`)
- `ff-samedomain`: `AreFacesSameDomain` (AlgoTools.cxx:1139-1205) is **one-directional and fail-closed** — the
  interior point comes from F1 only (:1153), `PointInFace` failure returns false with **no** PointNearEdge
  fallback (unlike IsSplitToReverse :1357-1369), `aTolEMax` is scanned over **F1's edges only** (:1174-1187)
  yet raises **both** tolerances, and `aTol = tolF1' + tolF2' + max(fuzz,1e-7)` (:1199).
  `faces_same_domain()` must either reproduce this asymmetry or document the symmetric divergence explicitly;
  the edge-tolerance lift is not optional.
- `ff-samedomain`: the verdict predicate carries **three** tolerances (`IsValidPointForFace`,
  IntTools_Context.cxx:647-673) — projection restricted to the face's UV box with projector tol 1e-12 and
  `Extrema_ExtFlag_MIN`, strict **unsquared** `dist > aTol` rejection, then a classifier that accepts
  `TopAbs_ON` as inside with band `BRep_Tool::Tolerance(F2)`. `sd_classify_face`'s probe loop must mirror the
  strict-reject / ON-accepts asymmetry, not use one band.
- `ff-samedomain`: OCCT's interior point is a hatcher probe at U = 43.213918% of the U span, **first hatch
  domain only**, V again at 43.2%, exactly two attempts (mirrored U); hatcher = Intersector(1e-10,1e-10) +
  Hatcher(1e-8,1e-8,true,false), edges without pcurve or with |u1-u2|<1e-9 skipped. Our `samples_in_face(16)`
  is strictly denser — fine, but the SD verdict must stay fail-closed when probing fails, not fall back to a
  looser test.
- `ff-samedomain`: **SD candidacy ignores `TangentFaces` entirely** in the Builder — candidates are
  HasFaceInfo-filtered FF faces bucketed by `BOPTools_Set` edge sets (Builder_2.cxx:653-748); only
  `BOPAlgo_CheckerSI.cxx:310-331` uses TangentFaces, on unsplit faces. Do not wire any tangency flag into
  candidacy. Corollary for HOOK 1: OCCT's bucketing works because PostTreatFF has already **fused** the
  cross-pair section edges into shared TShapes — we do not have shared entities at that point, so `key_tol`
  geometric bucketing is the substitute and its tolerance is load-bearing. Flag this precondition to A.
- `ff-samedomain`: `BOPTools_Set` semantics for `SDEdgeSig`/`SDFaceKey` (BOPTools_Set.cxx:120-172) — degenerate
  edges skipped, INTERNAL edges inserted **twice** (FWD+REV), equality = equal count + IsSame containment
  (orientation-**insensitive**, multiplicity-**sensitive**), hash = sum of normalized shape hashes mod 432123.
- `ff-samedomain`: the same-solid guard uses only `TopAbs_SOLID` sources with first-bind-wins and myImages
  propagation (Builder_2.cxx:596-648) — **shell-only operands are never guarded** in OCCT, so our mandatory
  `solid` key is stricter; document the divergence. The planar-bounded shortcut (:707-716, :780-785) declares
  SD with **no geometric test** from the surface type plus a non-open bbox of the **original** face inherited
  by all its splits — worth implementing for the all-planar chairs cells, but it is also the mechanism that can
  assert false SD, so gate it on the same-solid guard.
- `ff-samedomain` + `builder-assembly`: SD is recorded only in shape-level maps (myShapesSD / myImages /
  myOrigins); the consumer that matters, `BOPAlgo_BOP::BuildRC`, decides COMMON/CUT by **plain shape
  identity** — the image rewrite to a single representative *is* the tangential-contact boolean rule, which is
  exactly what HOOK 1 + HOOK 2 encode. `Sense()` has **zero callers**: delete any Sense-style API from the port.
- `ff-samedomain`: vertex fusion numerics if B extends into Law 3 — `MakeSDVertices` mutates an existing SD
  vertex's TShape in place (PF1:163-172) and geometry comes from `BRepLib::BoundingVertex` (exact 2-ball
  smallest enclosing sphere, or sorted-mean for >2).
- `pavefiller-core`: if Law 3 (common blocks) is B's next deliverable, the source is
  `BOPAlgo_Tools::PerformCommonBlocks` + `ComputeToleranceOfCB` (:107-356) — absent from every existing spec.

### BuildBOP op-table (`audit_occt_builder-assembly.md`)
- **Two fences, not one state enum.** `aMFence` (IsSame, orientation-blind) then `aMFenceOri` (IsEqual,
  TShape+Location+**Orientation**) at Builder.cxx:682/696/714. `sd_select_face` currently takes
  (op, operand, state); the ON-same vs ON-opposite distinction only exists because those two identities are
  kept separate, so the caller (HOOK 2) must carry both sightings, not just a boolean.
- **Full order of operations to encode** (Builder.cxx:650-737): (1) isIN/isINOpposite from myInParts;
  (2) FUSE skip if IN either group; (3) CUT/CUT21 skip if IN both; (4) second unoriented sighting → same-group
  branch (**falls through**, marks avoid iff `bTakeIN != isSameOriNeeded`) or both-groups SD branch
  (`isSameOri = !aMFenceOri.Add`, keep iff `isSameOriNeeded == isSameOri` **and** `aMResFacesFence` still free,
  else avoid, `continue`); (5) oriented fence; (6) keep iff `bTakeIN == isINOpposite`, with isIN adding **both**
  orientations, else if `(bTakeIN && !isSameOriNeeded)` add **only** `aFIm.Reversed()`; (7) filter by the
  unoriented avoid map. The spec's one-group-duplicate branch terminating is an error — it falls through.
- **CUT21 asymmetry confirmed** (p3 §4 rule 1, erratum E3): `aMResFacesFence` (:700) decides which copy of the
  SD wall survives — CUT keeps the objects' orientation, CUT21 the tools'. Keep the table as delivered.
- **isIN outranks the CUT reversal** (:721-729): a tool face IN the objects **and** IN another tool solid is
  emitted in both orientations **un-reversed**. When p3 §4's unmodelled multi-solid row lands, it must be
  evaluated *before* the `KeepReversed` rule, not after.
- **Orientation relation = surface-identity short-circuit first.** BuildDraftSolid applies
  `IsSplitToReverseWithWarn` **only to SD images** (Builder_3.cxx:311-326) while ordinary splits take
  `aFx.Orientation(aOrF)` verbatim (:334), and BuildBOP tests unconditionally (:594); the two agree only
  because `IsSplitToReverse(Face,Face)` short-circuits to a pure orientation compare when the faces share the
  `Geom_Surface` handle (AlgoTools.cxx:1336-1341). `sd_select_face`'s caller should compute
  `orientation_relation_same` the same way: shared-surface identity compare first, outward-normal geometric
  test only otherwise.
- **Representative rule for `groups()`**: only block members resolvable in the DS (unsplit originals) are
  candidates for the min-index rule, each self-bound on the spot (Builder_2.cxx:858); with no originals the rep
  is `aLSD.First()` = the MakeBlocks BFS seed; every member **including the rep** is bound in myShapesSD
  (:876-881), and `myShapesSD[rep]==rep` is a **branch selector** at Builder_3.cxx:311, not bookkeeping.
  Determinism has a single anchor — `std::sort` of DS indices at Builder_2.cxx:687 — so `detect()` must sort
  its inputs before component construction or reps drift between runs.
- **Origins are written at one site only** (Builder_2.cxx:914-919) inside FillSameDomainFaces, which returns
  early when `myDS->InterfFF()` is empty (:586-589): with no F/F interference, face images exist but face
  origins do not. Any consumer of face provenance must tolerate that.
- **Two OCCT defects not to port**: BuilderSolid failure inside BuildBOP returns without a Fail alert
  (Builder.cxx:790-793), so BOP.cxx:890 reads success and silently returns the general-fuse compound; and
  `BuildSolid`'s shared-face scan is `for (i=1; i<aNb; ++i)` (BOP.cxx:1140), skipping the last entry.
- `tolerance-model` cross-check for `faces_same_domain`'s tolerance: `tolF1`/`tolF2` are each raised to F1's
  max non-degenerate **edge** tolerance before summing, and self-interference fuzzy is **added**, not replaced.
- `intana-analytic` cross-check: `TangentFaces` is set at exactly one site (ImpImp:2865-2869) by
  `SameSurf || (all1 && all2)`, and clearing `slin`/`spnt` with it — another reason SD candidacy must not
  depend on it.
