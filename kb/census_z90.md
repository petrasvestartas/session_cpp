# Defect census — chairsROT z90 cut

Log: `census_z90.log` (992 lines). Summary:

```
chairsROT z90     cut   : faces 37 solid 0 naked 13 vol -1.0000
```

Run env: SESSION_CHAIRS_ROT=1 SESSION_ROT_ONLY=z90 SESSION_OP=cut SESSION_NO_MERGE=1 SESSION_NO_SEW=1 SESSION_FAST=1 SESSION_NT_DBG=1 SESSION_SPLIT_DBG=1

## Pipeline state snapshot

- `[SCAF] chains=47 segs=50 verts=52 paves(tA=67 tB=51 x=335 v=0 c=0) drop(verdict=42 micro=43) bridge(march=0 weld=0 resid=0) dev(A=0.000e+00 B=1.828e-08) tol3=3.661e-02`
- `[SCAF-VAL] valence1=5 of 52`: pairs v3/v4 (d=0.0366) and v5/v6 (d=0.0382), plus UNPAIRED v20 (nearest_v1 d=3.6309). No `[SCAF-BRIDGE]` lines anywhere in the log (bridge march=0 weld=0 resid=0).
- `[SEGLOST] A seg=12 surfA=7 surfB=14 n=5 closed=0` (tiny 0.046-len seg v20..v19 — the unpaired valence-1 end) and `[SEGLOST] A seg=25 surfA=15 surfB=15 n=35 closed=0` (v33..v7, 3D len ~0.258).
- `[SEGAUDIT]` asymmetries: seg=0,1,3,23,24 keptA=0 keptB=1; **seg=4 keptA=0 keptB=7** (A2 edges=6 trims=10, B2 edges=10 trims=16); seg=11 keptA=2 keptB=1; **seg=25 keptA=0 keptB=1, A2 edges=0 trims=0, B2 edges=2 trims=4**.
- `[MICROFRAG] A dropped=16 tol=0.07321`, `B dropped=6` — A dropped every fragment of segs 3/4 (uv cuts on A si=0 show the fragments incl. 4 closed micro-loops at uv (7.758,0.241), (7.697,0.090), (7.695,0.084), (7.691,0.075)).
- `[JOINFAIL]` on A si=0: pieces=16 joined=2 gapmax=3.167e-02 (2x), pieces=3 gapmax=1.471e-15; A si=15: gapmax=6.557e-03 (2x).
- `[SEGFALL]` straight-chord fallbacks on A: si=0 cidx=6 (ta 0.077, 0.048), cidx=8 (0.256), **cidx=1 ta=6.23776**; si=16 **cidx=1 ta=19.80224**, cidx=12 ta=2.11609; plus si=16 fi=3 cidx=10 keep=0 -> face emitted with segments=0.
- `[XWELD]` pass1: welds=25 common-blocks=3 micro=6 rescue=0 tube=0.02196; pass2 (xweld2): welds=20 common-blocks=6 micro=0 rescue=1. Pass1 1-trim list had CLOSED whole-boundary loops e0 (f0 A, len 18.11, v(0,0)) and e65 (f20 A, len 16.31, v(0,0)); pass2 shows them split with B quad edges e103/e104/e107/e108 (f27/f29 B, upper section quad segs 0/1/23/24) and B slivers e117/e123 consumed, leaving the residuals that are the final naked set.
- `[SCAF-DROP]` ALL-OUT: 41 lines; none on the sA=0/sB=15 pair that hosts the naked cluster 3 (checked every drop line — surf pairs are 7/14, 14/*, 16/*, 17/18, 18/*, 19/*). One drop, sA=7 sB=14 uvA(0.868,1.534), matches the closed=1 uv cut a(0.868,1.538) on A si=7, but no result naked edge lies in that region (nearest naked endpoint > 2 away): no class-E naked edge.

## Scaffold geometry needed for the pairing (sA=0 / sB=15 curve, ordered by x)

v8(9.9008,2.0036,-0.6730) — v3(9.7438,2.0174,-0.6601) — v4(9.7074,2.0206,-0.6571) — v5(9.5542,2.0335,-0.6445) — v6(9.5162,2.0366,-0.6414) — v7(9.4627,2.0409,-0.6371) — v33(9.4654,1.8245,-0.7771)

Segs on this curve OVERLAP: seg2=v3..v4, seg3=v5..v6, seg4=v7..v8 (spans the whole ladder, nCh=34, fragmented into >=10 whole=0 runs), seg25=v33..v7 (lost on A entirely). Valence-1 pairs v3/v4 and v5/v6 are the ladder breaks; the fragment drops (MICROFRAG A=16) killed all A-side pieces.

## The 13 naked edges and their classification

Notation: NKPAIR d = logged nearest-1-trim distance.

### Cluster 1 — pseudo-loop L1, 5 edges, faces 0/23/26/24 vs 20 (all side A) — class B

Path P7->P1->P4->P5->P8 (4 edges, total len 12.16) vs single residual boundary edge e136 (f20, len 11.80) covering the SAME path.
P1=(7.2644,3.2183,4.3511) P4=(6.6105,1.0610,4.9107) P5=(9.3558,-0.4190,-0.1505) P7=(8.0877,2.6835,3.3485) P8=v33=(9.4654,1.8245,-0.7771)

| edge | face | len | a | b | NKPAIR near1 |
|---|---|---|---|---|---|
| e0   | f0  | 1.407  | (7.2645,3.2183,4.3511) | (8.0877,2.6835,3.3485) | e136 d=0.2534 |
| e81  | f23 | 2.323  | (6.6105,1.0610,4.9107) | (7.2644,3.2183,4.3511) | e136 d=0.0145 |
| e97  | f26 | 6.110  | (9.3558,-0.4190,-0.1505) | (6.6105,1.0610,4.9107) | e136 d=0.2005 |
| e90  | f24 | 2.323  | (9.4664,1.8114,-0.7859) | (9.3558,-0.4190,-0.1505) | e136 d=0.1824 |
| e136 | f20 | 11.799 | (8.0877,2.6835,3.3485) | (9.4654,1.8245,-0.7771) | e97 d=2.0206 |

Class B pair **B2**: e136 <-> {e0+e81+e97+e90}. Both paths side A (inter-face mates, not A-B): f20's pass-1 boundary was ONE closed 1-trim edge (e65, 16.31) which after xweld2 left one 11.8-long residual spanning 4 logical edges. Max divergence 0.2534 (vs e0); endpoint mismatch e90.a<->e136.b = 0.0158 (= the A-lost seg25 stub at v33). Divergence >> weld tube 0.02196, and span mismatch 1-vs-4 defeats endpoint pairing.

### Cluster 2 — pseudo-loop L2, 3 edges, faces 1/25 vs 0 (all side A) — class B

Path P2->P3->P6' (2 edges, 7.04+3.26=10.30) vs residual boundary edge e131 (f0, 10.22) same path.
P2=(9.4330,3.1739,6.8805) P3=(12.6622,1.6390,1.0159) P6=v8=(9.9008,2.0036,-0.6730)

| edge | face | len | a | b | NKPAIR near1 |
|---|---|---|---|---|---|
| e4   | f1  | 7.043  | (9.4330,3.1739,6.8805) | (12.6622,1.6390,1.0159) | e131 d=0.0405 |
| e91  | f25 | 3.263  | (12.6622,1.6390,1.0159) | (9.9123,2.0026,-0.6740) | e131 d=0.1093 |
| e131 | f0  | 10.222 | (9.9008,2.0036,-0.6730) | (9.4330,3.1739,6.8805) | e4 d=2.6462 |

Class B pair **B1**: e131 <-> {e4+e91} (A-A inter-face). Max divergence 0.1093 (vs e91); endpoint mismatch e91.b<->e131.a = 0.0116. Same mechanism as L1: f0's pass-1 boundary was one closed 1-trim edge (18.11) that stayed a multi-span residual.

### Cluster 3 — the v3..v7 ladder region, 5 edges — classes B and A

| edge | face/side | len | seg | a | b | NKPAIR |
|---|---|---|---|---|---|---|
| e128 | f0 A  | 3.595 | -1 | (8.4782,2.4588,2.6732)=P9 | (9.5542,2.0335,-0.6445)=v5 | e133 d=0.0601 |
| e133 | f20 A | 3.499 | -1 | (9.4490,2.0400,-0.6372)~v7 (off 0.0137) | (8.4782,2.4588,2.6732)=P9 | e128 d=0.0054 |
| e130 | f0 A  | 0.221 | -1 | (9.7074,2.0206,-0.6571)=v4 | (9.9008,2.0036,-0.6730)=v8 | e125 d=0.0053 |
| e125 | f32 B | 0.174 | 4 f[19.89,30.96] | (9.7276,2.0188,-0.6587) | (9.9008,2.0036,-0.6730)=v8 | e130 d=0.0000 |
| e119 | f32 B | 0.067 | 4 f[2.66,7.08] | (9.4627,2.0409,-0.6371)=v7 | (9.5288,2.0355,-0.6425) | e128 d=0.0000 |

- Class B pair **B3**: e128 (f0) <-> e133 (f20), A-A divergent chords both anchored at P9; divergence 0.0054–0.0601; far-end gap v5 <-> e133.a = 0.1056 (they land on OPPOSITE sides of the lost seg3 span v5..v6 + gap v6-v7).
- Class B pair **B4**: e130 (A, invented chord v4->v8) <-> e125 (B, real seg-4 fragment): share v8 exactly, other ends differ by 0.0203 (vs weld tube 0.02196 — inside the tube but unwelded; lengths 0.221 vs 0.174, 27% mismatch). Max divergence over the shared span 0.0053. NOT class F: endpoints+length differ.
- Class **A**: e119 — one-sided B fragment of seg 4 (SEGAUDIT seg=4 keptA=0 keptB=7; the A-side counterparts were the 16 MICROFRAG-dropped fragments, tol 0.0732 > every fragment; e119 itself len 0.067 < tol survived on B because B dropped only 6). It rides at d=0.0000 under the long A chord e128, which is already the B3 mate of e133.

Root scaffold defect under all of cluster 3: overlapping segs 2/3/4/25 on the same sA=0/sB=15 curve + valence-1 ladder breaks v3/v4 (0.0366) and v5/v6 (0.0382). SEGLOST seg25 (A2 edges=0) also explains the 0.0158 stub between e136.b and e90.a in cluster 1.

## Class totals

| class | meaning | count | edges |
|---|---|---|---|
| A | SEGLOST/one-sided seg fragment | 1 | e119 |
| B | divergent mate pairs (seg=-1 invented/residual vs real path) | 12 | B1: e131 vs e4,e91; B2: e136 vs e0,e81,e97,e90; B3: e128 vs e133; B4: e130 vs e125 |
| C | junction undershoot at valence-1 pair | 0 | (v3/v4, v5/v6 pairs exist but their naked expression is via B3/B4/A above; unpaired v20/seg12 produced NO naked edge) |
| D | micro/triangle hole | 0 | (the 4 closed uv micro-loops on A si=0 were MICROFRAG-dropped, not emitted) |
| E | verdict drop matching naked uv | 0 | (no SCAF-DROP on sA=0/sB=15; sA=7/sB=14 drop region has no naked edge) |
| F | unmerged identical mate pair | 0 | (closest candidate e130/e125 differs 0.0203 in endpoint and 27% in length -> classed B4) |
| G | other | 0 | |
| **total** | | **13** | matches `naked 13` |

Cross-check: 1+12+0+0+0+0+0 = 13 = naked total. OK.

## Dominant mechanisms (ranked)

1. **Residual whole-boundary edges vs neighbor-face edge chains (10 of 13, pairs B1+B2).** A faces f0/f20 emitted their boundary as ONE closed 1-trim edge (pass-1 e0 len 18.11, e65 len 16.31) after `[JOINFAIL] gapmax=3.167e-02` and straight-chord `[SEGFALL]` (ta up to 6.24 and 19.80) degraded the loop geometry. xweld2 could only weld the sub-spans that matched B's section quad; the residuals span 2–4 logical edges each with divergence 0.0145–0.2534 >> tube 0.02196, so they never mate with f1/f23/f24/f25/f26 edges running the same path. Representative: e136 f20 len 11.799 (8.0877,2.6835,3.3485)->(9.4654,1.8245,-0.7771) vs the 4-edge chain e0+e81+e97+e90 through (7.2644,3.2183,4.3511), (6.6105,1.0610,4.9107), (9.3558,-0.4190,-0.1505).
2. **Overlapping scaffold segs + micro-fragment drop on the sA=0/sB=15 curve (4 of 13: B3, B4, A).** Segs 2/3/4/25 overlap on one physical curve (seg4 v7..v8 spans the whole v3–v7 valence-1 ladder); the arrangement fragmented A's copy into 16 micro pieces all dropped (`[MICROFRAG] A dropped=16 tol=0.07321`, `[SEGAUDIT] seg=4 keptA=0 keptB=7`, `seg=25 A2 edges=0`), so A's faces closed with divergent chords (e128/e133/e130) while B kept real slivers (e119/e125). Representative: e119 f32(B) len 0.067 (9.4627,2.0409,-0.6371)->(9.5288,2.0355,-0.6425) sitting at d=0.0000 under A chord e128.
3. **Sub-tube weld miss (subset of 2, pair B4).** e130 (A, v4->v8, 0.221) vs e125 (B, 0.174) share v8, other endpoints 0.0203 apart — inside tube 0.02196 — yet unwelded (length/span mismatch); NK-RESCUE (xweld2 rescue=1 fired elsewhere) missed it. Cheapest single-edge win in this config.
