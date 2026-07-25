# Defect census — chairsROT z37 cut

Run: `SESSION_CHAIRS=serialization/boolean_steps/chairs SESSION_CHAIRS_ROT=1 SESSION_ROT_ONLY=z37 SESSION_OP=cut SESSION_NO_MERGE=1 SESSION_NO_SEW=1 SESSION_FAST=1 SESSION_NT_DBG=1 SESSION_SPLIT_DBG=1 ./build/Release/main_7_gm.exe zzzz`
Log: `scratchpad/census_z37.log` (1003 lines)

## Summary line
```
chairsROT z37     cut   : faces 38 solid 0 naked 30 vol -1.0000
```

## Key pipeline stats
- `[SCAF] chains=45 segs=45 verts=49 paves(tA=67 tB=48 x=239 v=3 c=0) drop(verdict=35 micro=44) bridge(march=0 weld=0 resid=0) dev(A=0 B=1.7e-8) tol3=3.547e-02`
- `[SCAF-VAL] valence1=8 of 49 vertices` — 4 unclosed scaffold gaps:
  - (v3, v42) gap 0.2114 : (9.461,2.040,-0.735) / (9.467,1.836,-0.792)
  - (v5, v47) gap 0.3161 : (11.722,1.788,0.564) / (11.663,1.797,0.253)
  - (v14, v17) gap 0.1490 : (6.740,3.635,-1.371) / (6.682,3.636,-1.233)
  - (v28, v48) gap 0.1159 : (9.395,0.498,-0.342) / (9.413,0.534,-0.451)
- `[SPANSEG] A@presym n=43` (missing segs 3, 21); `[SPANSEG] B@presym n=32` (missing segs 2, 9, 10, 11, 16, 24, 27, 31, 36, 37, 38, 43, 44)
- `[SEGLOST] A seg=3 surfA=0 surfB=15 n=49 closed=0` ; `[SEGLOST] A seg=21 surfA=15 surfB=15 n=49 closed=0`
  (scaffold marched both fully — `SCAF-RUN seg=3 whole=1 MATED`, `seg=21 whole=1 MATED` — the 49-pt chains existed and were lost in the A split/coverage stage)
- `[XWELD] welds=28 common-blocks=5 micro=3 rescue=6 tube=0.02128` then post-imprint `[XWELD] welds=18 common-blocks=9 micro=6 rescue=5 tube=0.02128`
- No `[SCAF-BRIDGE]` lines; bridge(march=0 weld=0 resid=0)
- `[BLOCKS A] split=5 merged=3 cap=4 micro=0 projfail=2`; `[MICROFRAG] A dropped=17 tol=0.07093`; `[CLS-ISLAND] A flipped 2`

## Naked-edge topology
All 30 naked edges chain into:
- ONE giant 23-edge loop alternating A-runs and B-runs around the section band (edges: e6 e127 | e156 e152 e151 e150 e149 | e119 e99 e133 e90 e31 e67 e112 | e143 e145 | e105 e106 | e157 e146 e148 | e131 e132). The A/B transition points sit exactly at the 4 valence-1 scaffold pairs.
- One cluster of 7 short edges + 1 long dangler at the (v14,v17) junction: e33 e34 e66 e123 (A) + e140 e179 (B) + e182 (A, rescue-failed).

## Per-edge classification (30 edges)

### Class A — SEGLOST / one-sided seg coverage (15)
Seg exists on one operand only. Root: `[SPANSEG]` asymmetry — the symmetric-coverage/split stage kept the seg's section edge on one side only (`SEGAUDIT keptX=0`), even though the scaffold marched and MATED every one of these segs.

A-side kept, B lost (13):
| edge | face | seg | len | a | b |
|---|---|---|---|---|---|
| e31 | 7 A | 9 | 2.091 | (6.6792,3.6370,-1.1608) | (8.6672,3.6250,-0.5255) |
| e66 | 10 A | 10 | 0.248 | (6.6819,3.6361,-1.2333) | (6.6896,3.7805,-1.4346) |
| e67 | 10 A | 11 | 0.940 | (6.6823,4.5046,-0.7991) | (6.6792,3.6370,-1.1608) |
| e90 | 18 A | 16 | 1.798 | (8.6672,3.6250,-0.5255) | (8.6920,1.9019,-1.0378) |
| e105 | 21 A | 26 | 0.234 | (8.6700,4.5013,-0.3333) | (8.4396,4.5066,-0.3721) |
| e106 | 21 A | 25 | 2.436 | (11.0846,4.3331,-0.0727) | (8.6700,4.5013,-0.3333) |
| e112 | 22 A | 27 | 1.061 | (7.7057,4.5126,-0.5188) | (6.6823,4.5046,-0.7991) |
| e119 | 25 A | 44 | 0.499 | (9.4134,0.5343,-0.4506) | (8.9739,0.3349,-0.5785) |
| e123 | 26 A | 31 | 0.130 | (6.6896,3.7805,-1.4346) | (6.7490,3.6652,-1.4319) |
| e127 | 26 A | 43 | 0.311 | (11.9143,1.9632,0.3330) | (11.6634,1.7971,0.2534) |
| e131 | 26 A | 36 | 0.324 | (10.7874,2.5618,-0.4828) | (10.8485,2.2679,-0.3614) |
| e132 | 26 A | 38 | 0.742 | (10.1980,1.9757,-0.5536) | (10.8485,2.2679,-0.3614) |
| e133 | 26 A | 37 | 0.819 | (8.6920,1.9019,-1.0378) | (9.4672,1.8364,-0.7917) |

(SEGAUDIT: segs 9,10,11,16,27,31,36,37,38,43,44 have `B2 edges=0 trims=0`; segs 25,26 have `B2 edges=1 trims=1` — B produced an edge that never got a second trim and vanished from the kept set.)

B-side kept, A lost — the two `[SEGLOST]` segs (2):
| edge | face | seg | len | a | b |
|---|---|---|---|---|---|
| e150 | 34 B | 21 | 1.633 | (9.3322,2.0703,0.0927) | (9.3949,0.4979,-0.3420) |
| e151 | 34 B | 3 | 2.454 | (11.7223,1.7879,0.5639) | (9.3322,2.0703,0.0927) |

### Class B — divergent bridge: seg=-1 invented closure, mate exists on other operand (7)
The operand that lost a seg invented a closure chord to keep its face loop closed; the surviving real edge on the other operand has different geometry, so neither welds (divergence >> tube 0.0213).

| invented (seg=-1) | path | mate | mate path | max div |
|---|---|---|---|---|
| e6 A f0, len 1.685 | (10.1980,1.9757,-0.5536)->(11.6634,1.7971,0.2535) [scaffold v4->v47 chord] | e151 B seg3 | (11.7223,1.7879,0.5639)->(9.3322,2.0703,0.0927) | 0.92 (NKPAIR 0.7879/0.9213) |
| e99 A f20, len 1.347 | (9.4134,0.5343,-0.4506)->(9.4672,1.8364,-0.7917) [v48->v42 chord] | e150 B seg21 | (9.3949,0.4979,-0.3420)->(9.3322,2.0703,0.0927) | 0.83 (0.7297/0.8304) |
| e143 B f31, len 0.358 | (7.8788,4.2023,-0.5614)->(7.7057,4.5126,-0.5188) | e112 A seg27 | (7.7057,4.5126,-0.5188)->(6.6823,4.5046,-0.7991) | 0.31 |
| e145 B f32, len 0.666 | (8.4396,4.5066,-0.3721)->(7.8788,4.2023,-0.5614) | e105 A seg26 (+e104) | (8.6700,4.5013,-0.3333)->(8.4396,4.5066,-0.3721) | 0.58 |
| e148 B f33, len 0.165 | (10.7874,2.5618,-0.4828)->(10.7424,2.5132,-0.3322) | e131 A seg36 | (10.7874,2.5618,-0.4828)->(10.8485,2.2679,-0.3614) | 0.24 |
| e149 B f34, len 0.510 | (8.9739,0.3349,-0.5785)->(9.3949,0.4979,-0.3420) | e119 A seg44 | (9.4134,0.5343,-0.4506)->(8.9739,0.3349,-0.5785) | 0.11 |
| e156 B f37, len 0.293 | (11.8199,1.8541,0.5881)->(11.9143,1.9632,0.3330) | e127 A seg43 | (11.9143,1.9632,0.3330)->(11.6634,1.7971,0.2534) | 0.25 |

Note e6 and e99 also terminate at valence-1 vertices (v47, v42/v48): the invented chords replace lost segs 3/21 AND absorb the junction gaps.

### Class C — junction undershoot at valence-1 scaffold pairs (5)
| edge | len | path | vertex pair | gap |
|---|---|---|---|---|
| e33 A f7 | 0.199 | v17 (6.6820,3.6361,-1.2333)->(6.6879,3.6342,-1.4327) | (v17,v14) | 0.1490 |
| e34 A f7 | 0.028 | (6.6879,3.6342,-1.4327)->v13 (6.7162,3.6342,-1.4320) | (v17,v14) | 0.1490 |
| e140 B f30 | 0.025 | v14 (6.7399,3.6348,-1.3705)->(6.7467,3.6403,-1.3942) | (v14,v17) | 0.1490 |
| e179 B f30 | 0.044 | (6.7467,3.6403,-1.3942)->v37 (6.7583,3.6497,-1.4351) | (v14,v17) | 0.1490 |
| e152 B f34 | 0.120 | v5 (11.7223,1.7879,0.5639)->(11.8199,1.8541,0.5881) | (v5,v47) | 0.3161 |

Mechanism: the scaffold network never connected v17 to v14 (gap 0.149). A bridged v17 DOWN to v13 via invented e33+e34; B bridged v14 ACROSS to v37 via invented e140+e179 (face 30 is the micro triangle v13/v14/v37 — its original 3-edge naked triangle e140/e141/e142 partially welded, leaving these two). e152 is B's stub out of dangling v5 toward v46 (continues as e156). The other two pairs, (v3,v42) and (v28,v48), are consumed inside class-B chords e99/e149.

### Class D — micro/triangle hole: 0
Face 30's original 3-edge triangle (first XWELD X1T dump: e140 0.069 / e141 0.066 / e142 0.059 spanning v13,v14,v37) would have qualified, but e141/e142 welded in xweld2; survivors e140/e179 are counted in C. No closed 2-3-edge naked loop remains.

### Class E — verdict drop: 0
All ~35 `[SCAF-DROP] ALL-OUT` intervals (drop verdict=35) are on surface pairs (sA in {0,11,13,14,16,18,19} x sB in {14..19,0..3}) with uv far outside the naked regions (many with negative/out-of-domain uv). None matches a naked edge's uv region; the drops look genuinely outside.

### Class F — unmerged identical mate pair, rescue missed/failed (1)
| edge | detail |
|---|---|
| e182 A f7, len 1.976, seg=-1, (6.7646,3.6344,-1.4310)->(8.7332,3.6114,-1.2746) | Mate pair: e54 f7(A) len 1.9838 (8.7333,3.6114,-1.2746)->(6.7573,3.6343,-1.4312) and e120 f26(A) len 1.9765 (8.7333,3.6114,-1.2746)->(6.7646,3.6344,-1.4310). Same start vertex v26, far endpoints only 0.0073 apart — well inside weld tube 0.0213. xweld2's rescue merged them into e182 but the result kept only ONE trim (face 7); face 26's trim vanished, so the merged edge is still naked. This is the A-hull seam between faces 7 and 26, not a section edge. |

### Class G — divergent closure with NO naked mate (2)
| edge | detail |
|---|---|
| e146 B f33, len 1.132, seg=-1, (10.7424,2.5132,-0.3322)->(11.6676,3.1285,-0.1200) | Runs 0.17 from the WELDED 2-trim edge e130 (= seg35 v39->v40 (10.7874,2.5618,-0.4828)->(11.7021,3.1692,-0.2666)). B's cap face 33 boundary duplicates an already-mated seam with divergent geometry; the A side welded along the true path, so no naked mate exists. |
| e157 B f37, len 1.339, seg=-1, (11.6676,3.1285,-0.1200)->(11.0846,4.3331,-0.0727) | Connects e146's divergent end back to v35 (real seg25/seg30 junction). No scaffold seg v40->v35 exists; nearest naked edge e106 is 1.17 away, nearest welded e153 is 0.83 away. Pure invented B-cap excursion. |

## Count cross-check
A=15, B=7, C=5, D=0, E=0, F=1, G=2 -> total 30 = naked 30. OK.

## Dominant mechanisms
1. **Asymmetric seg coverage (24/30 edges = A + B + G).** The scaffold marched and MATED all 45 segs, but the split/symmetric-coverage stage kept 13 segs on A only and 2 segs ([SEGLOST] 3, 21) on B only (`SPANSEG A n=43 vs B n=32`). Every one-sided seg leaves its kept edge naked (class A), and the losing operand invents a divergent closure chord (classes B/G) that misses the weld tube by 5-40x (div 0.11-0.92 vs tube 0.0213).
2. **Scaffold valence-1 gaps (5 edges = C).** 4 unclosed vertex pairs (gaps 0.116-0.316, all >> tol3 0.0355); around (v14,v17) both operands bridged to WRONG targets (A: v17->v13, B: v14->v37) creating the face-7/face-30 micro cluster.
3. **Rescue-failed identical mate pair (1 edge = F).** e54+e120 (A faces 7/26 hull seam, endpoint gap 0.0073 < tube 0.0213) merged to e182 but with a single trim.
