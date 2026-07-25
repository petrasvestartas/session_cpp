# audit_brep_section

Complete audit of `session_cpp/src/brep_section.cpp` (2205 lines, HEAD 4544770f) against the two
OCCT specs `kb/occt_ssi-walking.md` (IntPatch/IntWalk marching) and `kb/occt_pavefiller-core.md`
(BOPAlgo_PaveFiller/BOPDS). File exposes three functions: `build_section_scaffold` (L379–2041),
`refine_scaffold_at_breaks` (L2043–2193), `build_shared_edge_pool` (L2199–2222).

---

## 1. HELPERS (anonymous namespace, L27–377)

| Helper | Lines | Role | OCCT analog |
|---|---|---|---|
| `SEval` | 29–35 | surface eval closure; NOTE kernel derivative order: `d[2]=Su, d[1]=Sv` | Adaptor3d |
| `correct7` | 39–81 | min-norm 4-var Gauss-Newton on A(uA,vA)−B(uB,vB)=0, 8 iters, accept `<conv_tol`, fallback accept `<conv_tol*10` | IntWalk_TheInt2S corrector (unpinned) |
| `correct7_pinned` | 85–123 | 3-var Newton with one param frozen at a bound, 12 iters | IntImp_ConstIsoparametric / SeekPointOnBoundary locked-iso |
| `refine_triple_point` | 129–175 | 6-var Newton S0=S1, S0=S2 (3-surface corner), 14 iters | EF/FF interference vertex solve |
| `seg_seg_2d` | 177–187 | 2D segment crossing, den guard 1e-20, param slack 1e-12 | Geom2dInt (chordal proxy) |
| `face_loops_uv` | 191–257 | trim-loop UV polylines per face; adaptive sampling n=clamp(cv*4,16,256), sag refine to `max(1e-9, bbox*2e-4)`, 6 passes, cap 4096 | TopolTool domain polygon |
| `dist_to_poly_uv` / `point_in_poly_uv` / `in_faces_uv` | 259–299 | UV point-in-face with eps band (grazing counts inside outer; inner holes excluded only if `dist > eps`) | TopolTool::Classify |
| `srf_aabb` | 301–310 | 9x9 sample AABB per surface | Bnd_Box (coarser: no pole handling) |
| `Chain` struct | 313–324 | `{surfA,surfB, p3/uvA/uvB lockstep, closed, paves(i,f), pave_kinds[5], pave_fix map pos->{p3,uvA,uvB}}` | IntSurf_LineOn2S + IntPatch_Point set |
| `refine_trim_pave` | 330–371 | Newton on S_side(c(t))=S_other(u,v), t clamped [-0.25,1.25] | IntPatch_CurvIntSurf |
| `lerp` | 373–375 | 3D lerp | — |

`Chain` is the port of `IntSurf_PntOn2S` sequences: every sample is index-corresponded
(3D, uvA, uvB) — the load-bearing lockstep invariant used by every later stage.

---

## 2. STAGES OF build_section_scaffold (L379–2041)

### Stage 0 — tolerance setup (L380–400)
Joint AABB diag over BOTH operands' vertex arrays -> `weld_tol = diag*2e-3` (`scaf.tol3`),
`SESSION_TOL3_MULT` scales it; `tol3_rep = weld_tol` (`SESSION_REP_MULT` scales);
`conv_tol = max(tolerance, diag*1e-9)`; per-surface segment index lists sized.
- Provides: single global 3D weld band for the whole run.
- Mismatch: OCCT has NO single band — per-entity tolerances (`ToleranceFF` = max(tolF1,tolF2)
  floored 5e-6 freeform, curve tol = max(IC tol, TolFF), vertex tol >= real distance, growth
  recorded in `myIncreasedSS`). pavefiller invariant 7 ("tolerance honesty") has no analog here.

### Stage 1 — one fenced SSI per overlapping pair -> chains (L402–471)
AABB overlap prefilter with margin `maxExtent(A_i)*1e-3` (L413–418).
`Intersection::surface_surface(sa,sb,tol)`; if empty, retried SWAPPED (L423–426) — the marcher is
order-sensitive. Per returned pcurve pair: resample at `n = clamp(pca.cv_count()*4, 48, 1024)`
uniform params of A's pcurve (L437–441); uvB seeded from B's pcurve at the SAME normalized
fraction (L447); `correct7` converges each sample; on failure re-seed via global
`Closest::surface_point` and retry ONCE — **the retry's return value is discarded (L454)**, and the
sample is pushed regardless. All four params then clamped into domain (L457–460) AFTER
convergence, and `p3` re-evaluated from the clamped uvA (L461–462) — so `p3` always lies on A but
possibly off B, and `uvB` may no longer correspond to `p3`. `closed = front/back < weld_tol`.
- Assumes: pcurves valid on both surfaces; the two pcurve parameterizations are fraction-alignable.
- Provides: chains with lockstep triples; INTENDED all-converged (not enforced — see mismatch M1).
- Mismatches: no deflection-driven sampling (fixed count vs OCCT `TestDeflection` sag<=fleche,
  ssi-walking Stage 3 + invariant 4); no `SeekAdditionalPoints` min-40 densification; no pre-walk
  duplicate rejection (`IsPointOnLine`) or densest-wins arbitration (`DublicateOfLinesProcessing`)
  — swap-retry + adjacent-pair grazes can mint duplicates that only gated SESSION_SEG_UNIFY cures;
  no `IsTangentExtCheck` refusal (tangent-zone pair marches into fragments instead of routing to a
  coincidence handler); no transition (In/Out) stamping (`tgline.DotCross`).

### Stage 1a2 — seam decomposition (L472–574) [always on]
Per side: if surface `is_closed(d)`, detect UV jumps > period/2 between consecutive samples;
closed chains rotated so the walk starts right after a jump (L502–510); each jump replaced by a
corrector-pinned crossing (`correct7_pinned` with the seam coord frozen at the bound, L541–547)
emitted TWICE: end of piece at one seam bound, mirrored start of next piece at the other bound,
identical 3D (L548–563). Chairs (non-periodic) no-op.
- Provides: no chain interpolates across a period jump (ssi-walking invariant 9); crossing exists
  in both chart representations.
- Mismatch (minor): OCCT `IntPatch_SpecialPoints` also handles POLES (`AddSingularPole`) and
  carries a `PrePoint` state so the next corrector call is chart-pinned; we have no pole
  decomposition and no chart-pin carry — a pole-crossing chain is not split. jump_dir checks only
  first-found direction per step; a simultaneous UV corner jump (SeamUV) picks one direction only.

### Stage 1b — PutToBoundary (L578–859) [always on; EXT_TRIM parts gated]
For each open chain end:
1. Already ON a chart bound (abs test `|cur-bnd| < 1e-9`, L601–603): re-converge ALONG the border
   with `correct7_pinned` to kill along-border marcher drift; accept only if moved
   `<= max(step3*4, weld_tol*4)` (L621). Then optional `SESSION_EXT_TRIM` pinned variant
   (L634–720): march along the pinned border up to 6 steps until the OTHER operand's trim state
   flips, bisect 24 iters, append points (length gate `<= max(step3*6, weld_tol*6)`).
2. Not on a bound: extrapolate along end tangent, find first chart bound crossed within 4
   end-steps (`best_f < 4.0`, L727–734), pin + `correct7_pinned`, clamp, accept if landing
   `<= max(step3*4, weld_tol*4)` (L844), insert (L846–854).
3. No bound crossed = INTERIOR STALL: default `continue` (L746) — the end stays dangling unless
   `SESSION_EXT_TRIM` (L735–829): free-march (correct7) up to 6 doubling steps until the A/B trim
   state flips, forward-progress guard, 24-iter bisection to the flip, append (length gate 6x).
- Assumes: end tangent from last two samples meaningful; chart bounds are the only bound type
  needing exact landing (trim bounds optional).
- Provides: ssi-walking invariant 2 only PARTIALLY — chain ends on chart bounds are exact, but
  interior stalls (graze/tangency) are left dangling by default; the spec's invariant 2 says
  downstream may assume NO dangling interior end that isn't a tangency.
- Mismatches: OCCT completes stalled ends by CONSTRAINED MINIMIZATION, never by more marching
  (`PutToBoundary` band `1e-3*min(1,ranges)`, `SeekPointOnBoundary` gradient descent + 2-var
  extrema Newton to SqDist<1e-14, `HandleSingleSingularPoint` locked-iso snap, <=20 rounds); ours
  marches. No hairpin cleanup on insertion (OCCT deletes end points whose direction dot <= 0). No
  `IsParallel` guard (a boundary-parallel chain can be slid sideways by case 1's re-converge). All
  epsilons absolute (1e-9, 1e-30) instead of resolution-relative (ssi-walking invariant 10). No
  singular-direction / anti-orbit tests.

### Chain fidelity metrics (L861–869)
`max_devA/max_devB` sampled every n/16 points — feeds the external [SCAF] gate. No per-chain
stored tolerance (see M5).

### Stage 2 — Paves (L871–1084)
Trim-loop caches `loopsA/loopsB` per used surface (L873–877).
`pave_dedup = weld_tol*0.5`; `add_pave` rejects a new pave whose 3D chord-lerp position is within
pave_dedup of ANY existing pave on the chain (L884–894) — first-inserted wins.
- **(a) trim crossings** (L896–951): seg-seg of chain UV polyline vs every loop polyline segment,
  both sides; on hit, `refine_trim_pave` Newton (exact section x trim, pave stays ON the loop
  polyline segment); accepted fix stored in `ch.pave_fix[i+s]` only if refined point stays within
  `weld_tol*6` of the chord crossing (L939). kind 0/1 = trimA/trimB.
- **(a2) chart-border touch paves** (L952–975): every inside<->on-border transition of the UV
  polyline paved (border = domain bound within 1e-12) — catches clamped collinear runs seg_seg
  can't see (den~0). Counted as trimA/trimB.
- **(b) chain-chain crossings** (L976–1048): all pairs sharing a surface on a side, UV-bbox
  prefiltered, INCLUDING self-crossings (c2==c1, j from i+2); both chains paved (kind 2); if the
  two chains' other surfaces differ, `refine_triple_point` solves the TRUE 3-surface corner and
  pins both `pave_fix` entries to the identical 3D point (accept band `weld_tol*6`).
- **(c) operand vertex paves** (L1049–1074): every topology vertex of A and B projected onto every
  chain (closest chord point); pave if within `weld_tol*2` (kind 3).
- **(d) closing pave** (L1075–1084): closed chain with no paves gets one pave at the
  lexicographically-max sample (kind 4).
- Assumes: loop polylines faithful to `bbox*2e-4`; chains in-domain.
- Provides: breakpoint set per chain; Newton-refined positions for trim/corner paves (sag-free).
- Mismatches: no graze-cluster collapse — OCCT IntStart merges quasi-tangent root clusters to the
  single min-|F| point at pitch <=1e-3 (pave spam on grazing trims possible; pave_dedup only welds
  within weld_tol/2 and keeps the FIRST, not the best); no whole-arc coincidence detection
  (Arcsol -> RLine; our SEGWHOLE analog lives downstream, so a trim lying ON the section yields
  paves/fragments, not a run); vertex paves don't carry vertex IDENTITY (OCCT IntPatch_Point
  stores arc+vertex; pavefiller paves reference DS vertex indices — ours are bare (i,f) positions,
  so operand-vertex/section-vertex identity must be re-derived downstream); no
  `FilterPavesOnCurves` best-chain-wins when one vertex projects onto several chains (kind-3 paves
  are added to EVERY chain within band); no stick/EF pave battery.

### Stage 3 — intervals -> shared verdict -> micro filter -> segments (L1086–1316)
`weld_vertex` (L1087–1099): nearest existing scaffold vertex within `weld_tol` else new — greedy,
non-transitive (clusters never merge; see M5).
Per chain: paves sorted by pos=i+f (L1102); breakpoints `bps` = [0, paves..., n-1] for open,
wrap-around from first pave for closed (L1106–1110). `at(pos)` (L1120–1139) returns the lockstep
lerp of p3/uvA/uvB, OVERRIDDEN by `pave_fix` within 1e-9 of a stored key; closed chains wrap,
open chains clamp.
**Keep-verdict** (L1141–1233): per interval, 9 midpoint samples; sample is IN iff
`in_faces_uv(loopsA, uvA, epsA*mult) && in_faces_uv(loopsB, uvB, epsB*mult)` where
`epsX = min(domain spans)*1e-3` (UV-parametric band), `SESSION_VERDICT_EPS_MULT` scales.
- all-in -> keep; all-out -> drop (`n_dropped_verdict`); mixed -> per-run 40-iter bisection onto
  the crossing, keep inside runs ending ON the boundary (L1213–1229).
- `SESSION_ON_QUORUM=W` (L1167–1199): keep a whole grazing interval when >=2 strict-in and ALL 9
  within the W-wide band (pure boundary runs still drop).
- `SESSION_CONN_STUB` (L1234–1258): kept interval abutting a dropped one extends a short tail
  (default 2 chords) into it — a sacrificial dangler that forces the T-junction node downstream.
**Micro filter** (L1259–1285): interval 3D length < `weld_tol * SESSION_SCAF_MICRO(default 1)` ->
DROPPED (`n_dropped_micro`).
**Segment emit** (L1287–1315): endpoints at RAW chain-lerp pave positions (deliberate: trim paves
must stay ON the loop polyline — L1290–1295 comment; pave_fix applies via at() for the
override-keyed positions), interior samples at integer chain indices, `v_start/v_end` welded,
`closed = v_start==v_end && size>3`, registered in `segs_by_surfA/B`.
- Provides: every segment inside both operands' trims (within eps band); endpoints welded; lockstep
  preserved.
- Mismatches: verdict band is UV-parametric, not metric — OCCT `IsValidBlockForFaces` classifies
  middle+bound points at 3D tolerance and all its tests are resolution-scaled (ssi-walking inv 10,
  pavefiller Stage 12.5); dropped micro intervals do NOT weld their endpoints — OCCT's
  FindValidRange/shrunk-range gate NEVER drops silently, it welds the two end vertices (four
  layers of defense, pavefiller invariant 5 + port map 5); dropped verdict intervals likewise
  leave valence-1 ends (the code's own comments identify this as the y30/x20/z45 naked-rim class);
  no `IsExistingPaveBlock` adoption — a section run coinciding with an existing operand edge
  still mints a new segment (aliasing deferred to brep.cpp keys).

### Stage 4 — unify per-side UV at shared vertices (L1318–1344) [always on]
Canonical UV per (vertex, side, surface) = FIRST segment end seen; all later ends assigned
bit-identical doubles; 3D endpoints snapped to the welded vertex position.
- Provides: exact-double equality junction weld for the downstream arrangement's vertex pool.
- Mismatch (minor): OCCT `SeveralWlinesProcessing` welds with resolution-of-distance acceptance and
  period-aware `MakeNewPoint`, rewriting the POINT in the line; ours rewrites only endpoints and
  picks first-seen (not best) as canonical; not period-aware (relies on stage 1a2 having split).

### Stage 4a2 — SD-WELD (L1346–1425) [gate SESSION_SDWELD, default OFF]
Transitive union-find over VALENCE-1 endpoints only (valence>=2 junctions deliberately excluded —
re-projecting them corrupted the exact network, base 35->127 faces). Union when gap < weld_tol, or
gap < 0.5 AND the midpoint projects onto all four referenced surfaces within `max(weld_tol, gap/2)`.
Component representative = mean point (OCCT MakeVertex covering-point analog); segment ends
re-projected per own surface via `Closest::surface_point`.
- This is the closest thing to pavefiller's `MakeSDVertices`, but: valence-1-only, endpoint-only,
  OFF by default, and no SD map persists (weld is a one-shot geometry rewrite, not a re-keyable
  link — pavefiller invariant 2's chain-walk closure has no analog).

### Stage 4b — JUNCTION WELD / bridge (L1427–1774) [gate SESSION_BRIDGE, default OFF]
Dangle inventory (valence-1 ends with surface pair + UV footprints);
mutual-nearest pairing within max_gap=0.5.
- **EF-MARCH** (L1501–1650) [nested gate SESSION_EF_MARCH]: dangle sitting ON the other operand's
  edge = exact EF junction; find continuation face across that edge; march the (own surf, next
  surf) pair from the anchor with normal-cross tangent predictor + correct7, step `weld_tol`, max
  200, staying inside both trims; continuation-angle gate cos>=0.866 (pi/6, OCCT
  `myMaxConcatAngle`); length gate `>= weld_tol*2`. New segment closes the dangle's valence.
- **CASE A same-pair re-march** (L1668–1727): 12-step constant-UV predictor + correct7 between the
  two dangling ends, VERIFY every sample on both surfaces and inside both trims, else refuse
  (never midpoint-welded — the fictitious y30 e122 chord lesson). Skipped entirely when
  `SESSION_BRIDGE>=2` (measured z30/z37/y30 regression source).
- **CASE B cross-pair fuse** (L1729–1763): midpoint T of the two dangles; accept iff T projects
  onto ALL FOUR involved surfaces within `max(weld_tol, gap/2)`; both segment ends snapped to T.
- OCCT correspondences: CASE B ~ `ExtendTwoWLines` — but OCCT's junction is the
  CORRECTOR-CONVERGED midpoint (Int2S on 0.5(P1+P2)) with THREE pairwise pi/6 angle gates and
  period rejection; ours is the raw midpoint with a projection gate only, and only EF-MARCH has
  the angle gate. CASE A ~ the missing `ExtendLineInCommonZone`+`RepartirOuDiviser` (re-march
  with locked iso through the tangent zone) — ours is an unlocked constant-step re-march.

### Stage 4c — EF diagnostic (L1776–1863) [gate SESSION_EF_DIAG, default OFF]
Pure diagnostic: coarse 32-sample scan of every other-operand edge vs every surface,
Gauss-Newton polish E(t)=S(u,v), dedup weld_tol*0.1; prints dangle-to-EF distance histogram.
This is pavefiller's `PerformEF` FIRST INCREMENT (exact EF interference vertices) existing only as
a diagnostic — the paves/junctions it finds are never fed into the scaffold (pavefiller Stage 6:
EF vertices are mandatory SSI points via `GetEFPnts`/`SetList`; we have no such feed).

### Stage 4d — cross-pair segment unification (L1865–2000) [gate SESSION_SEG_UNIFY, default OFF]
Graze duplicates from ADJACENT pairs (one operand surface shared, other differs): shorter segment's
samples within `band = mult(default 5)*tol3` of the longer master's polyline, tangent-parallel
gate |dot|>=0.95, end-anchored run only, run length >= 4*tol3; snap samples onto master + reproject
own-pair UVs; record master break at the deep end's fractional index -> `refine_scaffold_at_breaks`;
scoped end-vertex weld (only unify-touched + refine-minted vertices may merge at tol3).
- OCCT analog: `DublicateOfLinesProcessing` (densest wins) + CommonBlock formation. Differences:
  OCCT rejects duplicates BEFORE walking (seed-in-tube) and keeps ONE; we keep both geometrically
  snapped copies relying on downstream identity keys; OCCT's closed-CB regroup re-verifies by
  midpoint projection — our tie-break is length/id only.

### Stage 5 — valence audit (L2002–2039) [SESSION_SPLIT_DBG only]
Prints segments, valence-1 vertices with nearest-v1 distance, full vertex table. The even-valence
expectation stated here (closed solids -> closed section loops) is exactly ssi-walking invariant 2
+ pavefiller invariant 6; the audit MEASURES the violation instead of preventing it.

---

## 3. refine_scaffold_at_breaks (L2043–2193)
Splits existing segments at externally discovered fractional chain indices (SEG_UNIFY deep ends;
brep.cpp callers). `eps_f = 1e-2` end-window MUST equal brep.cpp `whole_seg` alias tolerance
(fa < 1e-2 && fb > nCh-1-1e-2) — divergence opens a dead band where a segment can acquire identity
by NEITHER route (documented L2045–2053). Closed segments skipped (`n_closed_skip` — wrap needs a
seam choice). Breaks clustered to group MEANS (same rule as normalize_section_blocks); breaks that
would carve a piece shorter than tol3 in arc length rejected (both ends would weld to one vertex
-> misread as closed). Endpoint samples copied bit-for-bit (NOT lerp_at at t=1: one ULP of drift
breaks the stage-4 exact-double junction keying, documented L2145–2151). First part keeps seg_id
(per-surface lists stay valid); rest appended with fresh ids.
- OCCT analog: `SplitPaveBlocks`/`BOPDS_PaveBlock::Update`. Missing vs spec: OCCT's split
  guarantees contiguous, exactly-covering sub-blocks and re-keys through the SD map afterwards;
  ours has no SD re-key (no SD map exists).

## 4. build_shared_edge_pool (L2199–2222) [BOP2 M1]
One arena vertex per scaffold vertex; one degree-1 NurbsCurve 3D polyline edge per segment with
SHARED endpoint topology vertices; `seg_edge[seg_id]` + `block_edge[{seg_id,0}]`.
- pavefiller analog: MakeBlocks section-edge factory + CommonBlock `SetEdge` (all members share one
  image edge). Missing: pcurves on BOTH faces at edge creation (`MakePCurve` L1024–1032 of _6.cxx —
  ours keeps uvA/uvB on the segment, not the edge), edge/vertex tolerances, `IsExistingPaveBlock`
  adoption before minting, micro-edge defense at pool level.

---

## 5. SESSION_* GATE TABLE

| Gate | Lines | Effect | Default |
|---|---|---|---|
| `SESSION_TOL3_MULT` | 394 | scales weld_tol (=tol3) | unset = 1.0 |
| `SESSION_REP_MULT` | 398 | tol3_rep = weld_tol * v | unset: tol3_rep = weld_tol |
| `SESSION_EXT_TRIM` | 634, 746 | stage-1b trim-boundary completion: pinned border-march variant + interior-stall free-march variant | OFF (interior stalls dangle) |
| `SESSION_VERDICT_EPS_MULT` | 1158 | scales verdict UV band only (pave bands untouched) | unset = 1.0 |
| `SESSION_ON_QUORUM` | 1174 | W>1: keep grazing interval when all 9 samples within W-band and >=2 strict-in | OFF (0.0) |
| `SESSION_CONN_STUB` | 1239 | extend kept ends into abutting dropped intervals by v chords (<=0 -> 2.0) | OFF |
| `SESSION_SCAF_MICRO` | 1281 | scales micro-filter floor weld_tol*v (static, read once) | unset = 1.0 |
| `SESSION_SDWELD` | 1355 | transitive union-find weld of valence-1 endpoints | OFF |
| `SESSION_BRIDGE` | 1452, 1667 | dangle repair: 1 = march(CASE A)+fuse(CASE B); >=2 = fuse+EF-march only (CASE A skipped) | OFF |
| `SESSION_EF_MARCH` | 1507 | (inside BRIDGE) EF-junction continuation march | OFF |
| `SESSION_EF_DIAG` | 1781 | EF interference histogram, diagnostic only | OFF |
| `SESSION_SEG_UNIFY` | 1873 | graze-duplicate unification, band = v*tol3 (v<=1 -> 5) | OFF |
| `SESSION_SPLIT_DBG` | 15 sites | stderr diagnostics ([SCAF-*], [PVFIX], [EXTTRIM-DBG]) | OFF |
| `SESSION_NT_DBG` | 7 sites | stderr diagnostics ([SDWELD], [EFMARCH], [SEGUNIFY], [REFINE]) | OFF |

Default-path = stages 0, 1, 1a2, 1b(chart-bounds only), 2, 3(no quorum/stub, micro=1), 4, 5-silent.
Everything that repairs valence (SDWELD/BRIDGE/EF_MARCH/SEG_UNIFY) is opt-in.

---

## 6. DATA FLOW

```
A,B BReps
  └─ Stage 0: diag -> weld_tol(tol3), conv_tol
  └─ Stage 1: per overlapping (surfA,surfB): SSI -> pcurves -> resample+correct7
       -> Chain{p3,uvA,uvB lockstep, closed}
  └─ Stage 1a2: split at seam jumps (dual-chart pinned crossings)
  └─ Stage 1b: open ends -> chart bounds (pin+reconverge); [EXT_TRIM] trim bounds
  └─ Stage 2: paves on each chain: (a) trim x section (Newton pave_fix)
       (a2) border-touch  (b) chain x chain (+triple-point pave_fix)
       (c) operand vertices  (d) closing            [dedup weld_tol/2, first wins]
  └─ Stage 3: sort paves -> intervals -> 9-sample both-trims verdict
       (keep / drop / bisect mixed) -> micro filter -> SectionSegment
       {p3,uvA,uvB, v_start/v_end via weld_vertex, segs_by_surfA/B}
  └─ Stage 4: canonical endpoint UVs per (vertex,side,surf); p3 ends = vertex pos
  └─ [4a2 SDWELD] [4b BRIDGE/EF_MARCH] [4d SEG_UNIFY -> refine_scaffold_at_breaks]
  └─ Stage 5: valence audit (debug)
SectionScaffold{vertices, segments, tol3, tol3_rep, counters}
  └─ build_shared_edge_pool: arena vertices + one shared edge per segment
  └─ consumers: brep.cpp split_with (segs_by_surf*, whole_seg keys eps 1e-2), combine
```

## 7. INVARIANTS (assumed -> provided)

| Stage | Assumes | Provides |
|---|---|---|
| 1 | SSI pcurves valid both sides; fraction-seeding valid | lockstep (p3,uvA,uvB); samples MOSTLY converged (NOT guaranteed: L454 unchecked retry, L457–462 post-convergence clamp) |
| 1a2 | is_closed correct; jump > period/2 detectable | no cross-period interpolation; dual-chart seam points, identical 3D |
| 1b | end tangents meaningful; >=2 samples | chart-bound ends exact + reconverged; interior stalls UNRESOLVED by default (violates ssi-walking inv 2 downstream contract) |
| 2 | loop polylines faithful to bbox*2e-4; chains in-domain | breakpoints for every trim/chain/vertex event; pave_fix sag-free for trim/corner paves |
| 3 | paves complete (bisection rescues straddles); eps band adequate | segments inside both trims; welded endpoints; drops recorded but endpoints NOT welded (violates pavefiller inv 5) |
| 4 | stage-3 endpoints near welded vertices | bit-exact endpoint UV/3D equality at shared vertices (the downstream arrangement's weld currency; refine_scaffold_at_breaks must preserve it bit-for-bit) |
| pool | segments >= 2 pts, valid vertex ids | one shared edge per segment, shared topo vertices (BOP2 referencing) |

Cross-file coupling invariants: `eps_f = 1e-2` (refine) == brep.cpp `whole_seg` window;
`tol3`/`tol3_rep` consumed by split_with/combine; exact-double junction UVs keyed by std::map in
the splitter.

---

## 8. MISMATCHES vs OCCT SPECS (ranked)

**M1. Unconverged/inconsistent chain samples are emitted (L449–464).** The correct7 fallback's
second attempt is unchecked and the sample is stored regardless; domain clamping runs AFTER
convergence and p3 is re-evaluated from clamped uvA only. Violates ssi-walking invariant 1
("every emitted point is corrector-converged; index-corresponded") — a clamped sample's p3 is on
A but off B, and uvB no longer corresponds. Every downstream consumer (paves, verdict, weld,
pool) trusts the lockstep triple. OCCT's corrector snap-to-border (aTol=Epsilon(range)) clamps
INSIDE the Newton loop and re-converges; failures abort the point.

**M2. Fixed-count resampling with no deflection control (L437; no densification).** Chains are
uniform-parameter pcurve resamples, n=clamp(cv*4,48,1024). OCCT bounds consecutive-point sag by
`fleche` (osculating-circle TestDeflection, per-axis pasuv with hysteresis) and guarantees >=40
points (`SeekAdditionalPoints`) — ssi-walking Stage 3/5, invariant 4, port-map 5/7. The chord sag
this permits (~0.1 at corners) is the documented root cause of half the file's own patches
(pave_fix, weld_tol=diag*2e-3 sized to absorb sag, stage-4 canonicalization) — the tolerance is
inflated to hide a sampling defect OCCT prevents at the source.

**M3. Invalid intervals are DROPPED, not endpoint-welded (L1200–1210 verdict, L1259–1285 micro).**
OCCT's four-layer micro defense (SplitPaveBlocks no-shrunk-range, MakeBlocks FindValidRange,
UpdatePaveBlocks micro detection, RemoveMicroSectionEdges/Edges) NEVER silently discards a block:
a range swallowed by vertex tolerance spheres causes the two end VERTICES TO BE WELDED
(pavefiller invariant 5, port-map 5). Our `continue` leaves valence-1 dangles — the file's own
comments identify dropped intervals as the y30/x20/z45 naked-rim class, and all repairs
(ON_QUORUM, CONN_STUB, BRIDGE, SDWELD) are opt-in gates instead of the spec's mandatory weld.

**M4. Boundary completion marches instead of minimizing; interior stalls dangle by default
(Stage 1b, L578–859).** OCCT's architectural rule: "never ask the marcher to finish a stalled
end — snap it by constrained minimization" (PutToBoundary band 1e-3*min(1,ranges) +
SeekPointOnBoundary gradient/extrema to SqDist<1e-14 + HandleSingleSingularPoint + hairpin
cleanup + IsParallel guard; ssi-walking Stage 5, port-map 1 marked REPLACE). Ours: extrapolation
+ pinned Newton to CHART bounds only; trim-boundary completion exists only under SESSION_EXT_TRIM
(OFF) and is itself a march; no hairpin cleanup, no boundary-parallel skip, absolute epsilons
(1e-9) instead of resolution-relative tests (ssi-walking invariant 10).

**M5. Greedy non-transitive weld + single global tolerance; no SD map, no growth feedback
(L1087–1099, L393; absent subsystem).** OCCT welds connected COMPONENTS transitively
(MakeSDVertices + chain-walked myShapesSD, re-keyed after EVERY stage via
UpdatePaveBlocksWithSDVertices — pavefiller invariant 2) and keeps per-entity tolerances with
honesty + feedback (tol >= accepted distance, myIncreasedSS -> RepeatIntersection/ForceInterfEE/EF
— invariant 7, port-map 1/2/9/10). Ours: weld_vertex greedily binds to the nearest EXISTING
cluster (clusters never merge; order-dependent), one global tol3=diag*2e-3 for detection AND
acceptance, no per-segment/vertex tolerance stored, no re-discovery after any weld. SESSION_SDWELD
is the transitive fix but valence-1-only and OFF.

**Secondary mismatches.**
- M6 No duplicate-chain arbitration at source (OCCT IsPointOnLine pre-walk tube rejection +
  densest-wins); SEG_UNIFY is the gated after-the-fact cure (Stage 1 / 4d).
- M7 No whole-arc coincidence (Arcsol/RLine) and no IsTangentExtCheck tangent-zone refusal —
  grazing/coincident pairs are marched into fragments instead of routed to a coincidence handler
  (ssi-walking Stage 1-alt/port-map 11/15; pavefiller CommonBlock subsystem).
- M8 No pave graze-cluster collapse (IntStart min-|F| single point, pitch 1e-3) and no
  best-curve filter for vertex paves (FilterPavesOnCurves); pave_dedup keeps first, not best.
- M9 Verdict band is UV-parametric (min-span*1e-3), not metric/resolution-scaled
  (IsValidBlockForFaces classifies at 3D tol; ssi-walking invariant 10).
- M10 EF interference exists only as a diagnostic (4c) — OCCT feeds EF points into SSI as
  mandatory points (GetEFPnts/SetList) and EF vertices into pave placement; our dangles
  undershoot junctions EF paves would anchor.
- M11 No In/Out transition stamping at creation (tgline.DotCross; ssi-walking port-map 16).
- M12 No section-edge adoption of coincident existing operand edges before minting
  (IsExistingPaveBlock x2, growth cap 0.001, PutSEInOtherFaces) — deferred to brep.cpp alias keys.
- M13 Pool edges carry no pcurves and no tolerances (MakePCurve at creation; CorrectToleranceOfSE).
- M14 CASE B junction = raw midpoint + projection gate; OCCT converges the midpoint with the
  corrector and applies three pairwise pi/6 angle gates + period rejection (ExtendTwoWLines).
- M15 No pole decomposition / PrePoint chart-pin in seam splitting (IntPatch_SpecialPoints).
