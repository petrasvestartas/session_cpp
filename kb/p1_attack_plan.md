# P1 attack plan — chairsROT cut naked-edge elimination

Synthesized 2026-07-24 from 10 per-config defect censuses (`scratchpad/census_*.md`; control = z30x20, naked 0)
against the OCCT PORT MAPs in `kb/occt_ssi-walking.md`, `kb/occt_pavefiller-core.md`,
`kb/occt_interference-vef.md`, `kb/occt_ff-posttreat-samedomain.md`.

Defect classes: A = SEGLOST/one-sided seg coverage; B = divergent invented closure (seg=-1) whose mate is the
other side's true section path; C = junction undershoot at valence-1 scaffold pairs; D = micro/triangle hole;
E = verdict drop matching naked uv; F = unmerged identical mate pair (rescue missed); G = other.

## 1. Defect-class x config matrix

| Config | naked | A | B | C | D | E | F | G |
|--------|------:|--:|--:|--:|--:|--:|--:|--:|
| z30x20 (control) | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| x20    |  9 |  5 |  3 | 1 | 0 | 0 | 0 | 0 |
| z63    | 13 |  4 |  6 | 0 | 1 | 0 | 2 | 0 |
| z90    | 13 |  1 | 12 | 0 | 0 | 0 | 0 | 0 |
| y30    | 13 |  6 |  3 | 4 | 0 | 0 | 0 | 0 |
| z15    | 17 |  9 |  8 | 0 | 0 | 0 | 0 | 0 |
| x13y29 | 22 |  7 |  7 | 4 | 0 | 0 | 4 | 0 |
| z30    | 25 |  9 | 13 | 2 | 0 | 0 | 0 | 1 |
| z37    | 30 | 15 |  7 | 5 | 0 | 0 | 1 | 2 |
| z45    | 32 | 15 | 11 | 2 | 0 | 0 | 2 | 2 |
| **Total** | **174** | **71** | **70** | **18** | **1** | **0** | **9** | **5** |

Causal structure (consistent across every census):
- **B is downstream of A** in ~55/70 cases: the operand that lost a seg invents a seg=-1 closure chord along a
  different route (divergence 0.05–2.74, always >> weld tube ~0.021), so fixing A collapses most of B.
- **C is upstream of A** in y30/x13y29: an unclosed junction leaves the section chain open, the face split
  refuses open chains (`cuts=2 parts=1`) and whole segs go SEGLOST — closing C prevents those A losses.
- **E = 0 everywhere**: verdict drops never directly produce a naked edge (three censuses note ALL-OUT drops on
  the same surface *pair* as a lost seg — root-cause correlation only). No mechanism needed for E.
- z90 is the outlier: 10/13 edges are residual whole-boundary 1-trim edges (faces f0/f20 emitted their boundary
  as ONE closed edge after [JOINFAIL] gapmax 3.2e-2 and [SEGFALL] chords) that span 2–4 logical edges of the
  neighbor faces — an edge-granularity mismatch, not a seg loss.
- Scaffold bridge march never runs anywhere: `bridge(march=0 weld=0 resid=0)` in all 10 configs; valence-1
  pairs (gaps 0.038–0.466, all >> tol3 ~0.035) are simply left open.

## 2. Ranked mechanism plan

### M1 — One-sided seg adoption / SEG-ADOPT (kills class A + dependent B + G; ~121 of 174 edges)
The seg exists in the scaffold (marched, usually MATED) but the split/symmetric-coverage stage keeps it on one
operand only ([SEGAUDIT] keptX=0, [SPANSEG] asymmetry), or MICROFRAG/verdict drops kill all fragments on one side.
OCCT never loses one side of a shared curve because a section entity coinciding with anything existing is
*adopted*, never dropped, and pairs that produce nothing are re-run:
- `occt_pavefiller-core.md` PORT MAP **#15** (ForceInterfEE-style same-domain pass, marked TOP PRIORITY there):
  after vertex weld, bucket section/trim edges by welded (v1,v2) pair; 24-sample coincidence with band
  `tol3 + 2*max(tolV)` gated by tangent alignment `|cos| >= 0.9063`; positives alias to ONE seg_id. Directly
  rescues the 8 "soft-lost" z15 segs (present in A2 as 1-trim, pruned) and every keptX=0 whole seg.
- `occt_pavefiller-core.md` PORT MAP **#10** (`SplitPaveBlocks` no-valid-range => UNIFY endpoint vertices,
  never drop): replaces MICROFRAG's silent drop — the z90 seg-4 ladder (16 fragments dropped, tol 0.073) and
  z30 seg40 (13 A-pieces mostly dropped) become welds instead of holes.
- `occt_ff-posttreat-samedomain.md` PORT MAP **#3** (`IsExistingPaveBlock` A/B: BVH over existing edges, midpoint
  + endpoints projection within `max(tolE,tolV)+band`, 25-deg tangent test, growth cap `min(0.001,10*tol)`):
  before emitting a section run as new, adopt the coincident existing edge and record the other face as owner.
- `occt_pavefiller-core.md` PORT MAP **#11** / `occt_ff-posttreat-samedomain.md` **#12** (recheck queue
  `aFFToRecheck`): any pair whose chains yield zero kept intervals is re-run after all pairs contributed paves —
  direct antidote for the hard-SEGLOST segs (z15 seg37: A2 edges=0, no scaffold march at all).
- Symmetric-coverage flood change: when a seg is kept on exactly one operand, force-inject it into the losing
  operand's UV arrangement (it has exact uv from the scaffold chain) instead of letting closure-weld invent a
  chord. The B-side then welds 1:1 (both sides run the same path) and the A/B pair goes 2-trim.

### M2 — Scaffold junction closure / JUNCTION-CLOSE (kills class C + prevents C→A cascades; ~27 edges)
All 10 configs show `bridge(march=0 weld=0)`; 7 configs have valence-1 pairs with gaps 0.038–0.466 >> tol3.
Each operand then bridges the gap to a DIFFERENT improvised target (z37: A bridged v17→v13, B bridged v14→v37).
- `occt_ssi-walking.md` PORT MAP **#9** (`JoinWLines`/`ExtendTwoWLines`): junction = corrector-converged midpoint
  (correct7 on 0.5*(P1+P2)), accepted iff all three pairwise angles (tangent1, tangent2, gap vector) <= pi/6 and
  the point stays in both UV boxes; "Singular" (period landing) = extend both, don't merge.
- `occt_ssi-walking.md` PORT MAP **#1** (`PutToBoundary`/`SeekPointOnBoundary`): chain ends near a chart bound are
  completed by constrained minimization (4-var gradient + per-surface Newton extremum, <= 20 rounds), never by
  more marching — removes the trim-snap shortening that feeds partial runs (y30 seg2 fa 0→1.59).
- `occt_pavefiller-core.md` PORT MAP **#19** (`MakeNewVertex`): junction vertex = midpoint, tol = max(chain reps)
  + half separation — the closure becomes ONE shared vertex both operands route through.
- `occt_ssi-walking.md` PORT MAP **#6** (seed scheduling): if a pair's network still has valence-1 ends, re-seed
  the march from additional interference points (mid/1/last/3n4/n4), rejecting seeds inside existing-chain tubes.
Critical side effect: with junctions closed, y30's A si=1/si=13 face splits see CLOSED chains → `parts>1` →
segs 4,5,17,18 are no longer SEGLOST (6 class-A edges prevented beyond the 4 class-C edges).

### M3 — Global fuse: vertex imprint on 1-trim residuals + same-operand weld / POSTTREAT-FUSE (~22 edges)
Two rescue blind spots: (a) XWELD merges "section edges A<->B" only — geometrically identical same-operand
(A-A or B-B) 1-trim pairs are never candidates (x13y29 F1/F2 d=0.0002; z45 e88/e123 d=0.0000); (b) a residual
1-trim edge spanning 2–4 logical edges never mates because span granularity differs (z90 B1/B2 = 10 edges;
x13y29 e128 needs one vertex imprint at P7 to become two weldable pairs).
- `occt_ff-posttreat-samedomain.md` PORT MAP **#4** (PostTreatFF nested fuse): ONE global unification pass over
  all 1-trim edges regardless of operand/face; existing edges enter as a "compound" (never re-intersected
  among themselves); micro-PB rule: if two vertices' tol spheres don't touch, inflate both by gap/2 so the
  fuse MUST unite them (~L1344–1358 in the spec).
- `occt_ssi-walking.md` PORT MAP **#10** (`SeveralWlinesProcessing`): imprint every welded network vertex lying
  on a 1-trim edge (resolution-of-distance acceptance), splitting the edge there — fixes span mismatch, then
  the ordinary weld mates the pieces.
- `occt_pavefiller-core.md` PORT MAP **#1**+**#2** (persistent SD map walked to fixpoint + union-find
  connected-component weld): replaces order-dependent pairwise NK-RESCUE; all run endpoints re-key through the
  SD map after each pass.

### M4 — Per-entity tolerance growth + shrunk-range gate / TOL-GROWTH (~4 direct edges + robustness)
Flat bands cause the remainder: weld tube 0.021 vs gaps in the tube..tol3 window (z63 e128 stub 0.0276;
z90 B4 0.0203 unwelded on length mismatch), JOINFAIL drops chains at gap 6.2e-3 > join tol 4.4e-3 (y30 B1 lens),
MICROFRAG flat tol 0.07 eats legitimate fragments.
- `occt_pavefiller-core.md` PORT MAP **#17** (`UpdateVertex` growth-only tol + `myIncreasedSS` +
  `RepeatIntersection`): any weld/adoption at distance d sets entity tol >= d; grown entities trigger one
  re-discovery pass. `occt_interference-vef.md` INVARIANT 7: acceptance bands never silently exceed detection.
- `occt_pavefiller-core.md` PORT MAP **#3** (shrunk range): classify only the core outside endpoint tol spheres;
  never place a pave inside an end band.
- `occt_pavefiller-core.md` PORT MAP **#6** / `occt_ff-posttreat-samedomain.md` **#5**: common-block tolerance =
  max mate deviation, pushed into endpoint vertices; NK-RESCUE band becomes per-edge `max(chainTol, vertexTol)`
  instead of global 0.15*tol3. JOINFAIL: grow the chain tol to the measured gap instead of splitting the chain.

## 3. Predicted per-config naked reduction (edges removed by each mechanism)

| Config | naked | M1 SEG-ADOPT | M2 JUNCTION | M3 FUSE | M4 TOL | residual |
|--------|------:|---:|---:|---:|---:|---:|
| z15    | 17 | 17 |  0 |  0 | 0 | 0 |
| z30    | 25 | 22 |  2 |  0 | 1 | 0 |
| z37    | 30 | 24 |  5 |  1 | 0 | 0 |
| z45    | 32 | 28 |  2 |  2 | 0 | 0 |
| z63    | 13 | 10 |  0 |  2 | 1 | 0 |
| z90    | 13 |  3 |  0 | 10 | 0 | 0 |
| x20    |  9 |  8 |  1 |  0 | 0 | 0 |
| y30    | 13 |  0 | 11 |  0 | 2 | 0 |
| x13y29 | 22 |  9 |  6 |  7 | 0 | 0 |
| **Total** | **174** | **121** | **27** | **22** | **4** | **0** |

Notes: y30's 6 class-A edges are credited to M2 (junction gaps are their root cause — open chains refused by the
face split); z90's cluster 3 (3 edges) to M1 (MICROFRAG unify-instead-of-drop), B4 + clusters 1–2 (10) to M3;
z45's 2 class-G no-mate closures to M1 (they span A-lost seg corridors). Attributions are per-census root cause;
overlap means totals are upper bounds per mechanism but the union covers all 174.

## 4. Implementation order (risk-minimizing)

1. **M3 POSTTREAT-FUSE** — pure post-pass over 1-trim edges; control config has zero 1-trim edges post-combine
   ([NT] edges1=0), so it is a structural no-op on z30x20. No topology-creation risk; verify z63 13→10,
   x13y29 22→18, z90 13→~3.
2. **M2 JUNCTION-CLOSE** — scaffold-level, gated by the OCCT pi/6 triple + corrector convergence; control has
   valence1=0 of 33, so again a no-op on healthy runs. Verify y30 13→~2, x13y29, z37, x20; confirm the y30
   cascade (segs 4,5,17,18 no longer SEGLOST).
3. **M4 TOL-GROWTH (minimal)** — growth-only per-entity tol on weld/adoption distances + JOINFAIL gap absorption
   + shrunk-range unify. Infrastructure for M1's adoption bands; growth-only means the control's exact welds are
   unchanged. Verify z63 e128 stub and y30 B1 lens disappear.
4. **M1 SEG-ADOPT** — largest payoff, deepest change (operand split / symmetric-coverage flood). Land last, on
   top of M4's per-entity bands and M3's SD map; gate strictly on [SEGAUDIT] asymmetry (keptA=0 xor keptB=0) so
   the symmetric control path is untouched. Verify z15 17→0 first (single-mechanism config), then z45/z37/z30.

Regression gates per step: z30x20 cut must stay faces 34 / naked 0; base chairs cut/common/fuse exact
(35/46.8114, 25/33.4951, 50/127.0950); matrix 45/45.
