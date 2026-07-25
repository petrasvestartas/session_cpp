# Defect census — chairsROT z15 cut

Log: `census_z15.log` (same dir). Summary line:
`chairsROT z15     cut   : faces 38 solid 0 naked 17 vol -1.0000`

Flags: SESSION_CHAIRS_ROT=1 SESSION_ROT_ONLY=z15 SESSION_OP=cut SESSION_NO_MERGE=1 SESSION_NO_SEW=1 SESSION_FAST=1 SESSION_NT_DBG=1 SESSION_SPLIT_DBG=1

## Headline

All 17 naked edges chain into ONE closed 17-edge band-loop (every naked vertex has exactly 2 naked incidences).
It is the boundary of a single missing strip caused by ONE mechanism: the A operand lost the entire
9-segment section span {32,33,34,35,36,37,38,39,41} (SEGAUDIT keptA=0 for all nine; seg 37 absent from the
A2 arrangement entirely — [SEGLOST] A seg=37 surfA=18 surfB=14 n=10). A's trim loops were then closed with
8 invented seg=-1 chord edges, while B kept each lost segment one-sided as a whole-segment 1-trim edge.
The hole is the area between A's chord path and B's true section path (max path divergence ~1.73).

## Raw evidence

- `[SPANSEG] A@presym n=42: 0..36 38..42` (seg 37 missing) vs `B@presym n=43: 0..42`
- `[SEGLOST] A seg=37 surfA=18 surfB=14 n=10 closed=0`
- `[SEGAUDIT]` — segs 32,33,34,35,36,38,39,41: `keptA=0 keptB=1 | A2 edges=1 trims=1 B2 edges=1 trims=2`
  seg 37: `keptA=0 keptB=1 | A2 edges=0 trims=0 B2 edges=1 trims=2`
  (seg 9: keptA=1 keptB=2 — B split into 2 edges, benign, no naked edge on seg 9)
- `[SCAF-MERGE] section edges merged A<->B: 33 (A-side 34, B-side 44)`; A segs list = 0..31,40,42 (the nine lost segs never merged on A)
- `[SCAF-RUN]` — segs 32,33,34,35,36,38,39,41 all ran `whole=1` but NONE got `MATED` (only segs 40 -> e129 and 42 -> e130 mated). Seg 37 never even ran a scaffold march.
- `[NT] A2 edges1=20/151  B2 edges1=1/144` — A2 has 20 one-trim edges pre-prune
- `[MICROFRAG] A dropped=1 tol=0.06936`
- `[SCAF-VAL] valence1=0 of 43 vertices` — no valence-1 scaffold vertices
- `[SCAF] chains=42 segs=43 verts=43 paves(tA=62 tB=50 x=137 v=1 c=0) drop(verdict=47 micro=18) bridge(march=0 weld=0 resid=0) dev(A=0.000e+00 B=1.704e-08) tol3=3.468e-02`
- No `[SCAF-BRIDGE]` lines in the log.
- `[XWELD] welds=27 common-blocks=1 micro=1 rescue=0` (pass 1); `welds=0 ... rescue=0` (pass 2). NK-RESCUE fired 0 times — correctly, no identical mate pairs exist (class F empty).
- 47 `[SCAF-DROP]` lines, ALL of them `ALL-OUT` verdict drops (== drop(verdict=47)). Of note near the loss region: 3 drops on `sA=18 sB=14` (uvA(-2.158,2.722), (-0.199,1.663), (0.463,-0.978)) — the same surface pair as SEGLOST seg 37; plausibly the root cause of seg 37's total absence from A2 (counted under class A, not double-counted as E).
- `[SPLIT] si=18 nbnd=10 cuts=11 parts=6` — surface 18 (the SEGLOST surfA) split context.

## The 17-edge naked loop (chained, in order)

V-ids from [X1T]; coordinates world.

| # | edge | side | face | seg | len | from -> to |
|---|------|------|------|-----|-----|------------|
| 1 | e7   | A | f0  | -1 | 1.472 | V7 (11.4931,1.8213,0.1420) -> V8 (12.6622,1.6390,1.0159) |
| 2 | e14  | A | f2  | -1 | 2.935 | V8 -> V14 (12.2662,4.1988,-0.3540) |
| 3 | e73  | A | f20 | -1 | 1.995 | V14 -> V57 (10.4127,4.3967,-1.0539) |
| 4 | e104 | B | f31 | 35 | 1.715 | V57 -> V90 (8.7422,4.1252,-1.3236) |
| 5 | e103 | B | f31 | 36 | 0.075 | V90 -> V88 (8.6684,4.1181,-1.3317) |
| 6 | e100 | B | f30 | 34 | 0.370 | V88 -> V62 (8.6165,4.4825,-1.3649) |
| 7 | e80  | A | f21 | -1 | 1.450 | V62 -> V63 (7.1689,4.4984,-1.4359) |
| 8 | e96  | B | f28 | 32 | 0.541 | V63 -> V82 (7.2261,3.9602,-1.4252) |
| 9 | e98  | B | f29 | 33 | 0.504 | V82 -> V84 (6.7234,3.9924,-1.4364) |
| 10| e108 | B | f32 | 37 | 0.393 | V84 -> V30 (6.8855,3.6346,-1.4280) |
| 11| e33  | A | f7  | -1 | 1.856 | V30 -> V28 (8.7333,3.6114,-1.2746) |
| 12| e57  | A | f16 | -1 | 2.270 | V28 -> V48 (8.6744,1.3674,-0.9389) |
| 13| e107 | B | f32 | 38 | 0.839 | V48 -> V56 (9.4464,1.2581,-0.6499) |
| 14| e71  | A | f19 | -1 | 0.803 | V56 -> V11 (9.4736,2.0385,-0.8377) |
| 15| e11  | A | f1  | -1 | 1.557 | V11 -> V12 (10.8912,1.8996,-0.2135) |
| 16| e106 | B | f32 | 39 | 0.867 | V12 -> V93 (11.5399,2.4597,-0.0889) |
| 17| e113 | B | f36 | 41 | 0.681 | V93 -> V7 (close) |

A-runs and B-runs strictly alternate: A{e7,e14,e73} B{e104,e103,e100} A{e80} B{e96,e98,e108} A{e33,e57} B{e107} A{e71,e11} B{e106,e113}.
A's chord endpoints all lie ON section vertices — A takes shortcut chords between section vertices where its
seg trims are missing; B follows the true marched section.

## Classification (per edge)

### Class A — SEGLOST / one-sided segment (9 edges)
Seg kept on B only (`keptA=0 keptB=1`), edge is the WHOLE segment (f range == [SCAF-RUN] nCh-1):
- e96  seg=32 f[0,49] (SCAF-RUN nCh=50 whole=1, never MATED)
- e98  seg=33 f[0,49] (nCh=50)
- e100 seg=34 f[0,48] (nCh=49)
- e104 seg=35 f[0,46] (nCh=47)
- e103 seg=36 f[0,2]  (nCh=3)
- e108 seg=37 f[0,9]  — the hard [SEGLOST]: A2 edges=0 trims=0, no scaffold march at all; surfA=18 surfB=14
- e107 seg=38 f[0,17] (nCh=18)
- e106 seg=39 f[0,18] (nCh=19)
- e113 seg=41 f[0,8]  (nCh=9)

Sub-split: 1 hard-lost (seg 37, absent from A2) + 8 soft-lost (present in A2 with 1 trim, pruned:
`A2 edges=1 trims=1, keptA=0`). Their [SCAF-RUN] chains ran whole but none MATED to an A2 edge —
consistent with the whole span being rejected on the A side as one branch (9 contiguous-span segs;
seg 37's absence breaks A-way coverage of the branch, cf. the symmetric-coverage >=7/9-both-ways flood gate).

### Class B — divergent bridge / invented closure (8 edges)
seg=-1 chord edges on A faces, closing trim loops across the lost span. No 1:1 same-endpoint mate exists
(so NK-RESCUE correctly did not fire); the mate is the B section PATH, paired at shared loop vertices.
Path pairs (A chord run vs B section run it bypasses; d = nearest-naked distance from [NKPAIR]):

1. A run e7+e14+e73: V7(11.4931,1.8213,0.1420) -> V8 -> V14 -> V57(10.4127,4.3967,-1.0539)
   vs B run e113+e106 (V93 junction) and e104 beyond V57.
   Max divergence: e73 <-> e104 d=1.7320 (largest in the whole loop); e14 <-> e106 d=1.6295; e7 <-> e113 (adjacent at V7).
2. A run e80: V62(8.6165,4.4825,-1.3649) -> V63(7.1689,4.4984,-1.4359)
   vs B run e100 / e96 flanking it. Divergence: e80 <-> e33 d=0.8688 across the strip; e80 <-> e96/e100 adjacent at V62/V63.
3. A run e33+e57: V30(6.8855,3.6346,-1.4280) -> V28 -> V48(8.6744,1.3674,-0.9389)
   vs B run e108 (at V30) / e107 (at V48). Max divergence: e57 <-> e107 d=0.7252; e33 <-> e98 d=0.3662.
4. A run e71+e11: V56(9.4464,1.2581,-0.6499) -> V11 -> V12(10.8912,1.8996,-0.2135)
   vs B run e107 (at V56) / e106 (at V12). Divergence: e11 <-> e71 span, e71 <-> e11 d=0.6974; e11 <-> e106 adjacent.

Max divergence of A-closure path from B-section path: 1.7320 (e73 vs e104).

### Class C — junction undershoot: 0
[SCAF-VAL] valence1=0 of 43; no [SCAF-BRIDGE] lines; [SCAF] bridge(march=0 weld=0 resid=0).

### Class D — micro/triangle hole: 0
Single 17-edge loop; no 2-3-edge closed micro-loops. (e103 len=0.075 is short but part of the big loop.)

### Class E — verdict drop matching naked uv region: 0 counted
All 47 [SCAF-DROP] are ALL-OUT verdict drops. The 3 drops on sA=18 sB=14 coincide with the SEGLOST seg-37
surface pair and are the likely root cause of seg 37's A2 absence, but that edge (e108) is already counted
in class A — noted as causal overlap, not double-counted.

### Class F — unmerged identical mate pair: 0
No two 1-trim edges share endpoints+length (X1T list is 17 distinct loop edges). XWELD rescue=0 is correct.

### Class G — other/unknown: 0

## Cross-check
9 (A) + 8 (B) + 0 + 0 + 0 + 0 + 0 = 17 = naked total. OK.

## Mechanism summary (single dominant defect)
A-side branch loss of the contiguous section span segs {32..39,41}: seg 37 never entered the A2
arrangement (SEGLOST, surfA=18/surfB=14, 3 ALL-OUT verdict drops on that surface pair), the other 8 made it
into A2 as 1-trim edges but were pruned (keptA=0) and their whole-seg scaffold chains never MATED.
A then closed its loops with 8 chord edges (seg=-1), B kept the 9 true segments one-sided, and the strip
between the two paths (width up to 1.73) is the single 17-edge naked band-loop. Fix vector: rescue the
span on A (mate the 8 whole-seg SCAF-RUN chains / recover seg 37) OR make A's closure follow the B section
path instead of chording across it.
