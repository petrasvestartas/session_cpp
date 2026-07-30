# AUDIT: kb/occt2_intpolyh-seeding.md vs real OCCT source

Source root `/home/petras/code/code_cpp/OCCT`. Path shorthand used below:
`IP/` = `src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/`,
`IPA/` = `src/ModelingAlgorithms/TKGeomAlgo/IntPatch/`,
`AD/` = `src/ModelingData/TKG3d/Adaptor3d/`,
`ICS/` = `src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/`.

---

## 1. VERDICT

**Structurally faithful, quantitatively unreliable.** Every line number the spec cites resolves to the
right function in this OCCT tree (checked ~30 anchors), and the S0–S7 pipeline order, the 4-shift design,
the winged-edge chaining idea and the constants table are correct. But the spec was written from a
skim of the *intent* comments, not the branches: it inverts one selection rule, mis-transcribes a loop
condition into a hang, describes one retry that provably cannot fire, and asserts a coverage guarantee
that the code does not provide. Material errors, in order of porting damage:

- **E1 (coverage claim is false).** Spec INVARIANT 2 / "KEY PROPERTY": "every tri-tri contact couple is
  either consumed by a chain or OPENS A NEW section line … Branch enumeration is exhaustive by
  construction." A couple that opens a new line but yields `StartingPointsResearch == 0` is marked
  analyzed and produces **nothing** — the line stays empty and is silently recycled by the next couple
  (`IP/IntPolyh_MaillageAffinage.cxx:3369-3370, 3345-3354, 3602-3604`). `TriContact` accepts touching
  triangles, `StartingPointsResearch` uses 1e-11 absolute predicates — the two disagree often. Also,
  chaining requires the neighbour couple to be *un-analyzed* (`:3197`), so two chains meeting inside one
  connected component split it into several section lines. Components ≤ section lines, and some
  components contribute zero seeds. Session A must not treat "component count = branch budget" as an
  OCCT-proven invariant; it is a stronger property than OCCT has.
- **E2 (disproportion rule inverted).** Spec S3: "if the *smaller* surface's critical deflection >= the
  larger's weighted diag". Real: `diag_i = |box_i diagonal|² / (NbU_i·NbV_i)`; if `diag1 < diag2` the test
  is `FlecheCritique2 < diag1`, i.e. the **larger-cell** surface's critical deflection against the
  **smaller-cell** surface's weighted diag (`IP/IntPolyh_MaillageAffinage.cxx:1341-1386`). Porting the
  spec's version picks the wrong branch on every small-cutter × large-base pair — exactly the chairs case.
- **E3 (seed schedule not exhaustive; spec's loop hangs).** Real termination
  (`IPA/IntPatch_PrmPrmIntersection.cxx:2933-2936`) is
  `nbp>5 && (k<5 || !lignetrouvee) && (k-3<nbp || lignetrouvee)`; the spec wrote the third clause as
  `|| !lignetrouvee`, which never terminates when no line is found. Worse, the fallback branch
  double-increments the counter (`:2706-2708`), so the scan visits indices **3,5,7,9,…** — every *other*
  point, not "EVERY point of the line".
- **E4 (enlarge-zone retry is dead code in the common path).** `IP/IntPolyh_Intersection.cxx:373-390`
  re-samples points and re-runs `ComputeIntersection`, but `IntPolyh_Triangle::myIsIntersectionPossible`
  is **never** reset to true anywhere in the module (`SetIntersectionPossible(true)` does not exist), and
  `FillArrayOfTriangles` reuses the same triangle objects (`IntPolyh_Array::Init` does not clear,
  `IP/IntPolyh_Array.hxx:121-127`). The zero-couple condition that triggers the retry is produced by
  `TrianglesDeflectionsRefinement` demoting *all* triangles of both meshes
  (`IP/IntPolyh_MaillageAffinage.cxx:1150-1156, 1180-1189`), so pass 2 starts with every triangle culled
  and returns 0. Spec PORT MAP #10 ("ADOPT (cheap)") is adopting a no-op.
- **E5 (grid density).** Spec S0: "`SamplePnts(Deflection,10,10)` … min 10×10 … knot-aware". For every
  non-BSpline surface `SamplePnts` ignores both `theDefl` and the two minima and calls
  `ComputeSamplePoints()` (`AD/Adaptor3d_TopolTool.cxx:1086-1095`), which uses fixed per-type counts:
  plane 2×2, Bezier 3+NbPoles, cylinder/cone/sphere/torus/revolution/extrusion **15×15**, default 10×10,
  all clamped up to ≥6×6 (`AD/Adaptor3d_TopolTool.cxx:801-856`). And `IsUniformSampling()` is `true` for
  everything except BSpline (`AD/Adaptor3d_TopolTool.cxx:1649-1654`), so `D1->IsUniformSampling() ||
  D2->IsUniformSampling()` (note **OR**, spec says "unless D1->…") makes PrmPrm discard the knot-aware
  arrays and rebuild a **uniform** grid of the same counts whenever *either* surface is analytic
  (`IPA/IntPatch_PrmPrmIntersection.cxx:2534-2541`, `IP/IntPolyh_Tools.cxx:88-139`).
- **E6 (pre-walk dedup).** Spec: "threshold logic uses `SeuildPointLigne = 15*Increment²`". `SeuildPointLigne`
  is only a sentinel — `dminiPointLigne` is set to `2*Seuil` and slammed to `0.0` by a boolean
  (`IPA/IntPatch_PrmPrmIntersection.cxx:2725-2742`). The real predicate is `IsPointOnLine(..., Deflection)`
  (`IPA/IntPatch_PrmPrmIntersection.cxx:3993-4081`), which is far more permissive than the spec implies.
- **E7 (tangent-zone path differs).** Spec S7: TZ points are "tried the same way (dedup, walk,
  reject-by-endpoint+midpoint, AddWLine)". Real TZ loop (`IPA/IntPatch_PrmPrmIntersection.cxx:3004-3189`)
  has **no** `PutToBoundary`, **no** `NbPoints<3` recheck, **no** midpoint-to-segment probe, **no**
  `DublicateOfLinesProcessing`, **no** `SeveralWlinesProcessing`, and uses `> 0.` instead of `>= 0.` for the
  transition sign (`:3128` vs `:2867`). Its rejection is `IsPointOnLine(endpoint)` OR start/end within
  `TolTangency` of the *corresponding* endpoint of an existing line (`:3093-3111`).

Everything else the spec asserts I could confirm verbatim: the 17-axis SAT order, non-strict rejection,
the `.996` triggers, `1.5*deflTol` shifts, `FlecheMin*0.2+FlecheMax*0.8`, `2*NbT+1000`, the 1% enlarge
margin, BVH leaf 10, box gap `deflection+Precision::Confusion()`, the `Prepend` two-half chaining, the
vertex→TangentZone quarantine, `Limit=2500`, `aMinNbPoints=40`, and the IntPatch clamps.

---

## 2. PORT-CRITICAL DETAILS THE SPEC MISSED

### A. Sampling / grid density

- `Adaptor3d_TopolTool::ComputeSamplePoints` clamps an infinite domain to `[-1e5,1e5]` (or `±2e5` span
  from the finite end) *before* sampling — an unbounded surface silently becomes a 2e5-wide grid:
  `AD/Adaptor3d_TopolTool.cxx:771-797`.
- BSpline-only refinement of counts: `Analyse(poles,…)` runs only when `nbsu>8 || nbsv>8`, then an
  anisotropy doubling when the U/V arc-length ratio is `>=10` or `<=0.1`, capped at
  `aMaxNbSample = 50` per direction: `AD/Adaptor3d_TopolTool.cxx:752, 858-882`.
- `BSplSamplePnts` (the only deflection-driven path) caps at `aMaxPnts = 1001`:
  `AD/Adaptor3d_TopolTool.cxx:1134-1138`.
- Uniform rebuild forces the exact endpoints: last U/V sample is assigned `u1`/`v1` literally, not
  `u0 + i*dU`: `IP/IntPolyh_Tools.cxx:120-138`.
- Deflection = max over all net triangles of |dist(surface(UV-barycentre), triangle plane)|, ×1.2:
  `ICS/IntCurveSurface_PolyhedronUtils.pxx` (`ComputeMaxDeflection`) called at
  `ICS/IntCurveSurface_ThePolyhedronOfHInter.cxx:129-130`.

### B. Boxes, outcodes, triangle culling

- `CommonBox` has **no disjoint-box early-out**: if the two boxes miss, `XMin..ZMax` stay `0` and the whole
  pipeline proceeds against a degenerate box at the origin (`IP/IntPolyh_MaillageAffinage.cxx:531-540, 661-663`).
- The 10% inflation is **order-dependent and cross-contaminating**: X is fixed first (borrowing from the
  *original* Y then Z), then Y (borrowing from the *already scaled* X), then Z (from scaled X then Y):
  `IP/IntPolyh_MaillageAffinage.cxx:622-659`.
- Triangle rejection is **pairwise** outcode AND, not Cohen–Sutherland triple AND:
  `(poc_a & poc_b) && (poc_b & poc_c) && (poc_c & poc_a)` (`IP/IntPolyh_MaillageAffinage.cxx:983-991,
  1000-1006`). This is strictly *more* aggressive than `a&b&c`: e.g. `a=x⁻|y⁻, b=x⁻|z⁻, c=y⁻|z⁻` is rejected
  although the triangle may cut the common box. Over-rejection is possible; replicate exactly or you
  change which triangles survive.
- Per-surface box inflation happens **after** all points are added, `±(deflTol*1.2)` on each axis then
  `Enlarge(MyTolerance=1e-6)`: `IP/IntPolyh_MaillageAffinage.cxx:401-408`.
- `MyBox1/MyBox2` are member `Bnd_Box`es that are only ever `Add`-ed to — a second `FillArrayOfPnt` on the
  same maillage **unions** with the first pass's box: `IP/IntPolyh_MaillageAffinage.cxx:395, 465`.
- Degeneracy: `DegeneratedIndex` is checked in **V first, and in U only if V found nothing**
  (`IP/IntPolyh_MaillageAffinage.cxx:369-373, 427-431`). Only the 4 boundary isos are probed, 3 points each,
  `dist² <= (1e-6)²`, matched to a sample index by `|x - degX| < 1e-6`
  (`IP/IntPolyh_MaillageAffinage.cxx:3863-3916, 3920-3997`).
- A triangle is degenerate iff **≥2** of its points carry the degenerate flag **or** `|n|² < 1e-24`;
  `ComputeDeflection` returns 0 in both cases: `IP/IntPolyh_Triangle.cxx:63-83`.

### C. Refinement (single pass, destructive)

- `TrianglesDeflectionsRefinement` is **one pass** with the triangle counts captured *before* refinement
  (`FinTT1/FinTT2`, `IP/IntPolyh_MaillageAffinage.cxx:1138-1139`): newly appended triangles are neither
  visited nor demoted and enter `TriangleCompare` with `IsIntersectionPossible == true` by default
  (`IP/IntPolyh_Triangle.hxx:34-38`).
- Asymmetry between the two meshes: surface-1 triangles are refined per interfering pair; surface-2
  triangles are refined **once each** behind an `NCollection_Map` fence
  (`IP/IntPolyh_MaillageAffinage.cxx:1163-1176`). Which mesh is "1" here is decided by the diag test (E2).
- `MiddleRefinement` splits the **3D-longest** edge (`SquareDistance` on xyz, `IP/IntPolyh_Point.cxx:105-110`),
  ties falling through to edge 3 (`IP/IntPolyh_Triangle.cxx:278, 368, 457`). The new point's UV is the
  parametric midpoint, re-evaluated on the surface (`IP/IntPolyh_Point.cxx:23-35`) — so the split point is
  *not* the 3D midpoint.
- It creates **4 new triangles + 4 new edges** and **kills the adjacent triangle across the split edge**
  (`deflection=-1, IsIntersectionPossible=false`, `IP/IntPolyh_Triangle.cxx:340-343`) even if that neighbour
  was interfering and already deflection-classified. On a mesh boundary (`numTA < 0`) only 2 triangles and
  3 edges are made, with `-1` winged slots (`IP/IntPolyh_Triangle.cxx:344-365`).
- `MultipleMiddleRefinement` is **not recursive**: one `MiddleRefinement` then a linear sweep over the
  appended range `[FinTTInit, min(NbItems, 2*FinTTInit+1000))`, demoting out-of-box children and
  single-splitting the rest: `IP/IntPolyh_Triangle.cxx:564-583`.
- `BoundingBox` is **cached on first call** (`myBox.IsVoid()`) with gap `myDeflection + Precision::Confusion()`
  (1e-7): `IP/IntPolyh_Triangle.cxx:627-640`. A box computed before refinement is never recomputed.
- `LargeTrianglesDeflectionsRefinement` demotes every big-surface triangle whose box misses the opposite
  box *before* computing the criterion, and criterion = `0.5 × min(dx,dy,dz)` of the opposite box
  (`IP/IntPolyh_MaillageAffinage.cxx:1207-1253`); triangles *below* the criterion still get one
  `MiddleRefinement` (`:1271-1274`).

### D. Triangle–triangle soup

- `GetInterferingTriangles` is called **twice** — once inside `TrianglesDeflectionsRefinement` (`:1132`) and
  once at the top of `TriangleCompare` (`:3145`) — i.e. two full BVH builds per maillage. It bails out
  entirely (returns an empty map) if **either** tree is empty (`:219-222`).
- `TriContact` runs **6 AABB slab rejects first** with strict `<`/`>` (touching survives), then the 17
  SAT axes in this order: `n1, m1, e_i×f_j (9), g1..g3, h1..h3`
  (`IP/IntPolyh_MaillageAffinage.cxx:1480-1654`). All arithmetic is done in coordinates **translated by
  P1** (`:1523-1541`) — a deliberate conditioning choice worth replicating for far-from-origin models.
- **`Angle` is only written when both normals are non-degenerate** (`|n|² > 1e-24`,
  `IP/IntPolyh_MaillageAffinage.cxx:1657-1667`), and `TriangleCompare` declares `double CoupleAngle = -2.0`
  **once outside both loops** (`:3151`). A degenerate contact therefore inherits the *previous* couple's
  angle. That value feeds `IsAdvRequired`, the parallel guard, and `StartPoint::angle` → the consumer's
  `incidence`.
- Couple list order is deterministic: `BVH_PairTraverse` pairs are `std::sort`ed lexicographically by
  `(ID1, ID2)` (`IP/IntPolyh_MaillageAffinage.cxx:137-143, 175, 232`) and the indexed map preserves
  insertion order — so the couple list, and hence section-line discovery order, is sorted by `(T1,T2)`.
- Parallel guard counts `npara` against `GetArrayOfTriangles(i).NbItems()`, which **includes triangles
  appended by refinement and triangles already killed** (`IP/IntPolyh_Intersection.cxx:559-560`) — the
  threshold is inflated by however much refinement happened.
- `myIsParallel` is sticky: nothing ever sets it back to false (`IP/IntPolyh_Intersection.cxx:563`).
- `PerformAdv` computes in order **FR, RF, FF, RR** with short-circuit `&&`
  (`IP/IntPolyh_Intersection.cxx:284-327`) but `MergeCouples` fences in order **FF, FR, RF, RR**
  (`:440`), so the surviving copy of a shared couple lands in FF. `StartPointsChain` then runs
  FF, FR, RF, RR into the **same** `mySectionLines`/`myTangentZones` arrays (`:217-220`).

### E. Start points

- Coplanar/transversal split threshold is **absolute 1e-11 on a unit-normal dot product**, i.e. 1e-11 model
  units: `|n·PE1 − n·PT1| < 1e-11 && |n·PE2 − n·PT1| < 1e-11` (`IP/IntPolyh_MaillageAffinage.cxx:2606-2607`).
  Real-world meshes essentially never take the coplanar branch.
- Transversal acceptance is `α ∈ [-1e-11, 1+1e-11]` **and** `β ∈ [-1e-11, α+1e-11]` in the
  `PT1 + α·Cote12 + β·Cote23` frame (`:2877, 2989`) — i.e. `0 ≤ β ≤ α ≤ 1`, not two independent barycentrics.
- The α/β solve is a 6-branch hand-rolled equation-pair ladder keyed on which components of `Cote12`/`Cote23`
  exceed 1e-11 (`:2868-2980`). The terminal `else` sets `alpha = beta = RealLast()` (`:2985-2986`) and the
  guard `beta > alpha + eps` does **not** reject it — a fully degenerate triangle yields a start point with
  ~1e308 UV1.
- Vertex snapping happens *after* `NbPoints++`: α≈0, (β≈0,α≈1), (β≈1,α≈1) overwrite XYZ **and** UV with the
  exact triangle vertex and set `Edge = -1` (`:3003-3021`, mirrored at `:3065-3083`). Edge-interior cases set
  the mesh edge index and `lambda`, flipped to `1-lambda` when the triangle's stored edge orientation is
  negative (`:3022-3057`).
- Edge codes: `>=0` mesh edge, `-1` vertex, `-2` unset. Defaults from the ctor are
  `lambda1=lambda2=-1.0, angle=-2.0, t=-1, e=-2, chainlist=-1` (`IP/IntPolyh_StartPoint.cxx:24-41`) — the
  `-1.0` lambda default is load-bearing: `CheckSameSP` guards its lambda comparisons with
  `lambda > -1e-11`, which is precisely what stops two unrelated default-lambda points from comparing equal.
- `CheckSameSP` (`IP/IntPolyh_StartPoint.cxx:262-299`) is: *(matching e1 **or** matching e2, with `e >= -1`
  so vertex code `-1` participates)* **and** *(matching lambda1 **or** matching lambda2)*; failing that, if
  `e1==-1 || e2==-1`, compare **U1/V1 only** — UV2 is not consulted. Two points identical on surface 1 but
  distinct on surface 2 are declared the same point.
- `TestNbPoints` (`IP/IntPolyh_MaillageAffinage.cxx:1677-1759`) caps `NbPointsTotal` at 3; so
  `StartPointsChain`'s `(NbPoints > 2) && (NbPoints < 7)` branch (`:3594`) is only ever `== 3`.
- `NextStartingPointsResearch` excludes the incoming edge by comparing `SPInit.E1()/E2()` against
  `Tri.FirstEdge()/SecondEdge()/ThirdEdge()` (`:1962-2016`) and returns 0 if the single found point equals
  `SPInit` (`:2019-2039`); it always stamps `SPNext.SetCoupleValue(T1,T2)` even on failure (`:2045`).

### F. Chaining and section lines

- `IntPolyh_SectionLine` always carries one **trailing dummy** StartPoint: `Init(N)` ignores `N` and just
  guarantees length ≥1, `NbStartPoints() == Length()-1`, append writes into the dummy then appends a new one
  (`IP/IntPolyh_SectionLine.cxx:39-72`, `IP/IntPolyh_MaillageAffinage.cxx:3317-3318`). `Prepend` inserts at
  the front without a dummy (`IP/IntPolyh_SectionLine.cxx:135-138`). Backward-half ordering is therefore
  correct as the spec claims, but a naive port that stores `NbStartPoints` explicitly will be off by one.
- The empty-line recycle (`:3345-3354`) means `NbSectionLines()` can include a trailing empty line; the
  consumer skips it with `if (!nbp) continue` (`IPA/IntPatch_PrmPrmIntersection.cxx:2608-2612`).
- `CheckCoupleAndGetAngle` requires an **ordered** `(T1,T2)` match *and* `!IsAnalyzed`, and marks it analyzed
  on success (`IP/IntPolyh_MaillageAffinage.cxx:3188-3208`). A chain that reaches an already-consumed couple
  simply stops.
- Edge-edge (both `E1>=0 && E2>=0`) case: `CheckCoupleAndGetAngle2` marks only the primary couple analyzed
  and *saves iterators* to the two diagonals; the diagonals are marked analyzed **only if the recomputation
  yields exactly one point** (`:3740-3766`). Otherwise they survive and later open duplicate lines.
- `CheckNextStartPoint` deduplicates tangent-zone entries on **(U1,V1) and (U2,V2) within 1e-11**
  (`:3290-3300`) — the only cross-maillage dedup in the whole advanced path.
- Chains never test for `NextTriangle == -1`; it is absorbed because no couple has index `-1`
  (`IP/IntPolyh_Edge.hxx:32-38` default `-1`), so a boundary edge terminates the chain via a failed lookup.
- `IntPolyh_Array` grows by `myIncrement` (default 256) inside `IncrementNbItems`
  (`IP/IntPolyh_Array.hxx:132-142`); `Init(N)` only touches slot `N` and **preserves** slots `0..N-1`,
  which is exactly why the enlarge retry inherits stale triangle flags and stale cached boxes (E4).

### G. Consumer / seeding into PWalking

- Seed schedule indices and the double-increment: `k=1 → nbp/2` (and if `nbp<3`, `k` is forced to the limit
  5 so only one attempt happens), `k=2 → 1`, `k=3 → nbp-1`, `k=4 → 3nbp/4`, `k=5 → nbp/4`, then
  `nbps2 = k-3` with `k` incremented **again** → 3,5,7,9,…
  (`IPA/IntPatch_PrmPrmIntersection.cxx:2682-2709`). `nbp <= 5` ⇒ the do-while body executes exactly once.
- The walk box is the raw min/max of the polyhedral line's UV on both surfaces with **no inflation**, passed
  as `(u1min,v1min,u2min,v2min,u1max,v1max,u2max,v2max)`
  (`IPA/IntPatch_PrmPrmIntersection.cxx:2620-2670, 2744-2752`). For tangent zones the box is the union over
  **all** TZ points (`:2950-3002`), i.e. essentially unconfined.
- `IsPointOnLine` (`IPA/IntPatch_PrmPrmIntersection.cxx:3993-4081`) is much looser than a distance test:
  after 2D/3D box rejects, for each segment it either (a) projects the point when `AM·MB > 0` and compares
  the perpendicular **components signed against `Deflection`** — `HM.X() < Deflection` etc., so any negative
  component passes — or (b) when the projection falls outside, declares "on line" whenever
  `|P−Pa|² < |Pb−Pa|²` **or** `|P−Pb|² < |Pb−Pa|²`, i.e. within one segment length of an endpoint.
- Duplicate-line verdict for section lines is two-stage: endpoints matched both ways within `TolTangency`,
  then the existing line's **middle point** must be within `2*Epsilon` of some segment of the new polyline
  (`aLin12.Distance(aPx)`, an infinite-line distance, not a segment distance)
  (`IPA/IntPatch_PrmPrmIntersection.cxx:2806-2843`), then `DublicateOfLinesProcessing` (`:2848-2851`).
- **`AddWLine` deletes existing lines subsumed by the new one** — for each stored WLine, if its middle point
  *and every one of its vertices* lies on the new line, it is removed (`:4085-4144`). Dedup is bidirectional;
  the "longest first" bubble sort (`:2581-2601`) is a heuristic on top of this, not the sole protection.
- Zero-length safety in the midpoint probe: segments with `|P1P2|² < 1e-20` are skipped (`:2823`).
- `PutToBoundary` then `NbPoints<3 → continue` then `SeekAdditionalPoints(...,40)` then
  `EnablePurging(!hasBeenAdded)` — purging is disabled precisely on lines that were densified (`:2767-2782, 2882`).

---

## 3. PORTING TRAPS

1. **Do not port "components = branches" as a guarantee.** Add an explicit accounting pass: for every couple
   in the soup record whether it produced ≥1 start point; couples that contact but yield nothing are exactly
   the class that silently drops a branch (E1). This is the single most useful thing to instrument in SEED2.
2. **Two independent tolerance regimes.** Contact detection is *conservative* (SAT with touching accepted,
   boxes inflated by deflection+1e-7); point extraction is *brittle* (1e-11 absolute predicates everywhere).
   A reimplementation that uses one tolerance for both will either lose contacts or manufacture ghost points.
   In particular 1e-11 is not scale-relative: on a 1e3-sized model it is below double precision on the dot
   products being compared.
3. **The `Angle` leak.** `CoupleAngle` persists across the whole `TriangleCompare` double loop; degenerate
   contacts carry a neighbour's angle (or the initial `-2.0`, whose `|cos| > .996` makes it read as
   *near-parallel*). If you initialize per-couple instead, `IsAdvRequired` and the parallel guard will fire on
   different inputs than OCCT. Decide deliberately; do not inherit it by accident.
4. **`IsPointOnLine`'s signed-component test and its endpoint fallback.** Porting it as a proper
   point-to-polyline distance makes the pre-walk dedup *stricter* than OCCT and will admit re-seeds OCCT
   suppresses; porting it literally reproduces an asymmetric, orientation-dependent filter. Either way,
   remember `SeuildPointLigne` is not a geometric threshold (E6).
5. **`AddWLine` removes subsumed lines.** If you adopt PORT MAP #7 without #9's counterpart here, a long
   correct chain marched late can delete short *correct* sibling branches that happen to lie along it. The
   guard is that *all* vertices of the victim must lie on the new line — port the vertex loop, not just the
   midpoint test.
6. **The 4-shift pass fragments its own chains.** `MergeCouples` removes duplicated `(T1,T2)` pairs from
   lists 2–4, but chaining walks the *per-maillage* list; a couple deleted from FR because FF had it breaks
   FR's chain at that triangle. Expect the advanced path to emit many short section lines for one branch,
   geometrically offset by ±1.5·deflTol from the true intersection, and rely on `PerformFirstPoint`'s Newton
   plus `AddWLine` dedup to clean up. Do not "improve" this by merging chains across shifts before marching
   unless you also re-derive the couple adjacency.
7. **Refinement mutates shared state destructively.** `MiddleRefinement` kills the neighbour across the split
   edge, arrays are append-only with index-stable dead entries, and `Bnd_Box`es are cached on first touch. A
   port that rebuilds boxes lazily *after* refinement, or that compacts arrays, changes both which couples are
   found and which triangle indices the chain walks over.
8. **Diagonal orientation is fixed and asymmetric.** Cell (u,v) → T_even = (uv, u v+1, u+1 v+1),
   T_odd = (uv, u+1 v+1, u+1 v) (`IP/IntPolyh_MaillageAffinage.cxx:978-998`), with the edge table hand-built
   to match (`:770-951`, `NbEdges = 3·nbU·nbV − 2(nbU+nbV) + 1`). The chain's "flip across the winged edge"
   depends on this exact indexing; if you use a different diagonal or a generic half-edge structure, verify
   `T = 2·((u)·(nbV−1) + v) + {0,1}` still holds wherever the code does index arithmetic.
9. **Live OCCT bugs you will either inherit or diverge from** (all reachable, none guarded):
   - `IsDegenerated` probes `aU = i*dU` instead of `aU1 + i*dU` (`IP/IntPolyh_MaillageAffinage.cxx:3951, 3979`)
     — pole detection is wrong for any surface whose domain does not start at 0.
   - `TriangleEdgeContact`, `TriSurfID==2`, `alpha≈1` branch sets `SetLambda2(alpha)` where the mirrored
     `TriSurfID==1` branch sets `SetLambda1(beta)` (`:3113` vs `:3051`).
   - `CalculPtsInterTriEdgeCoplanaires` uses `Tri1.GetEdgeOrientation` inside the `TriSurfID==2`/SP2 branch
     (`:2257`), normalizes by *square* magnitude (`:2077, 2082`), and uses `if` instead of `else if` between the
     `alpha≈0` and `alpha≈1` cases (`:2148, 2216, 2246`) so both can execute.
   - `FillArrayOfPnt(SurfID, isShiftFwd, Upars, Vpars, tol)` forwards a hardcoded `1` as SurfID
     (`IP/IntPolyh_MaillageAffinage.cxx:498`) — harmless only because `IntPolyh_Intersection` never calls it.
   - `printf` to stdout on two reachable degenerate paths (`IP/IntPolyh_MaillageAffinage.cxx:2958-2960`,
     `IP/IntPolyh_StartPoint.cxx:292-294`).
10. **Walk confinement can truncate.** The UV box comes from a coarse polyhedral line; the true branch
    routinely extends past it and `PW.Perform` stops there, with `PutToBoundary` doing the recovery in 3D
    (`IPA/IntPatch_PrmPrmIntersection.cxx:2744-2767`). If your walker has no `PutToBoundary` analogue, clamping
    to the enumerated line's box will systematically shorten branches — inflate the box by at least one mesh
    cell, or re-extend after the march.
11. **For a 24×24 SEED2 grid**, note OCCT's own densities for comparison: 15×15 for all quadrics/revolutions,
    6×6 for planes, ≤50×50 for BSplines, hard ceiling 2500 nodes per surface before the whole IntPolyh path is
    abandoned for the legacy `PointDepart` grid (`IPA/IntPatch_PrmPrmIntersection.cxx:2498, 2524, 3200-3201`).
    24×24 = 576 is comfortably inside the ceiling but ~2.5× OCCT's quadric density — expect a larger, noisier
    couple soup and correspondingly more short components; the deflection-refinement stage (which OCCT uses to
    *earn* that density only where it matters) is what keeps OCCT's soup small.
