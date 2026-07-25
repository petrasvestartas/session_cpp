# Defect census — chairsROT z30 cut

Run: `SESSION_CHAIRS=serialization/boolean_steps/chairs SESSION_CHAIRS_ROT=1 SESSION_ROT_ONLY=z30 SESSION_OP=cut SESSION_NO_MERGE=1 SESSION_NO_SEW=1 SESSION_FAST=1 SESSION_NT_DBG=1 SESSION_SPLIT_DBG=1 ./build/Release/main_7_gm.exe zzzz`
Log: `census_z30.log` (907 lines)

## Summary line
```
chairsROT z30     cut   : faces 38 solid 0 naked 25 vol -1.0000
```

## Key pipeline signals
- `[SCAF] chains=41 segs=45 verts=48 paves(tA=59 tB=52 x=133 v=1 c=0) drop(verdict=35 micro=34) bridge(march=0 weld=0 resid=0) dev(A=0 B=1.76e-08) tol3=3.525e-02`
- `[SCAF-VAL] valence1=6 of 48` — THREE valence-1 vertex pairs, each maps 1:1 onto one of the three naked clusters:
  - v19 (6.684,3.697,-1.257) ↔ v39 (6.689,3.723,-1.434) gap **0.1786** → leg cluster
  - v29 (9.373,0.391,-0.221) ↔ v47 (9.409,0.450,-0.426) gap **0.2167** → strut cluster
  - v41 (10.728,2.170,-0.384) ↔ v43 (10.739,2.055,-0.341) gap **0.1227** → seat-corner cluster
  - All gaps ≫ tol3=0.0353 and weld tube=0.0212; scaffold bridge march did nothing (march=0 weld=0 resid=0).
- `[SEGLOST]` (A operand produced 0 edges at split): seg=12 (surf 7/14, n=6), seg=20 (15/0, n=51), seg=21 (15/1, n=49), seg=22 (15/14, n=34), seg=23 (15/15, n=49), seg=37 (18/11, n=49), seg=39 (18/14, n=7). **A-surface 15 loses 4 large segments; A-surface 18 loses 2.**
- `[SEGAUDIT]` one-sided: keptA=0 → segs 12,20,21,22,23,37,39 (B kept 1 edge each); keptB=0 → segs 34,44 (A kept 1 edge each). Asymmetric: **seg=40 keptA=5 keptB=1 (A2 edges=13 vs B2 edges=1)**.
- `[XWELD]` pass1: welds=27 common-blocks=1 micro=4 rescue=1 tube=0.02115; pass2: welds=18 common-blocks=4 micro=1 rescue=0.
- Co-signals on A operand: `[BLOCKS A] split=4 merged=8 projfail=1`, `[MICROFRAG] A dropped=5 tol=0.07049`.
- `[SCAF-DROP]`: 35 ALL-OUT verdict drops. Two surface-pair coincidences with lost segs — sA=7 sB=14 (line 69, = seg12's pair) and sA=18 sB=14 ×4 (lines 95–98, = seg39's pair) — but SEGAUDIT proves the B-side edges of those segs SURVIVED, so the drops are other intervals on the same surface pairs; the loss happened at operand split, not at scaffold verdict. **No drop interval conclusively matches a naked edge → class E = 0.**

## The three naked clusters (holes)

### Cluster 1 — leg junction, faces 7/25 (A) + 29/33 (B), 6 edges
Two divergent routes sharing both cluster endpoints (6.6837,3.6355,-1.2841) and (6.8933,3.7673,-1.4304):
- A route (low road, z≈-1.43): e32 → e91 → e92
- B route (high road, z≈-1.18..-1.26): e129 → e130 → e117
Root: seg34 exists only on A (keptB=0), seg12 only on B (keptA=0); scaffold left the valence-1 pair v19/v39 (gap 0.1786) between the routes; each operand closure-welded along its own path. Max path divergence ≈ 0.18–0.25.

### Cluster 2 — strut, faces 19/24 (A) + 27/28/33/34 (B), 9 edges
Sub-cluster 2a (upper): A bridged with single chord e74 (8.0817,2.6871,3.3577)→(8.6807,2.3513,2.2448) where B has the true 2-seg path e111+e113 via (8.0279,1.7803,3.3759) (segs 20,21 lost on A). **Max divergence ≈ 0.89** (B interior vertex to A chord).
Sub-cluster 2b (lower): double route from (8.4004,0.1492,-0.7498) to ≈(9.461,1.6745,-0.75):
- Route A: e90 (seg44, keptB=0) → v47 (9.4090,0.4502,-0.4258) → e71 (seg=-1)
- Route B: e131 (seg=-1) → v29 (9.3725,0.3907,-0.2207) → e132 (seg23) → (9.3209,2.0737,0.1514) → [A e73 seg=-1] → (9.4527,2.0414,-0.6657) → e128 (seg22)
Valence-1 pair v29/v47 gap 0.2167 sits exactly at the route fork. Max divergence ≈ 1.0 (route-B apex (9.3209,2.0737,0.1514) to route A).

### Cluster 3 — seat corner NE, faces 25 (A) + 32/33 (B), 10 edges
One open naked chain (11.7516,1.7843,0.3133) —e105— (10.5169,1.9426,-0.4059) —e127—e126—e123—e122—e125—e142—e143—e144—e139— (11.6807,2.7270,-0.1168):
- e127 (seg39), e122 (seg37): lost on A (surfA=18) → B one-sided.
- e123+e126 (B, seg=-1): bridge the valence-1 pair v41/v43 (gap 0.1227) via detour vertex (10.7107,2.1546,-0.3322); A never bridged there (A went around via e105).
- e125 (seg40, naked residual f[0,7.08], final len 0.067): asymmetric split — B kept seg40 whole (nCh=10, f[0,9]) while A split it into 13 pieces of which only one MATED (e158, f[4.08,4.92]); rest lost (MICROFRAG A dropped=5 / verdict). XWELD rescued most of the corridor, stub (11.3927,2.4887,-0.1756)→(11.4443,2.5291,-0.1643) remains.
- e105 (A, seg=-1, len 1.439): A's closure chord (10.5169,1.9426,-0.4059)→(11.7516,1.7843,0.3133); B's corridor (e127…e125 high road) diverges ≈0.8 (NKPAIR e105↔e122 d=0.8105).
- e142+e143+e144+e139 (A, seg=-1): A closure chain (11.4443,2.5291,-0.1643)→(11.5299,2.5687,-0.1352)→(11.6644,2.6499,-0.0961)→(11.6878,2.6650,-0.0896)→(11.6807,2.7270,-0.1168). B mate = original whole e125 span (11.3927→11.6807, X1T pass1 len 0.3785). Max divergence ≈ 0.057 (A vertex (11.6644,2.6499,-0.0961) to B chord). Pre-weld, face 25 (A) had TWO alternative 1-trim paths over this corridor — e99 (len 0.3107, v72→v74) and e104 (len 0.4637, v72→v78, endpoints ≈0.011 apart but different geometry) — XWELD rescue=1 consumed one; the divergent leftovers are this chain.

## Per-edge classification (25 edges)

| e | face | side | len | seg | endpoints | class |
|---|------|------|-----|-----|-----------|-------|
| 90 | 24 | A | 1.101 | 44 | (9.4090,0.4502,-0.4258)–(8.4004,0.1492,-0.7498) | A |
| 92 | 25 | A | 0.209 | 34 | (6.8933,3.7673,-1.4304)–(6.6889,3.7234,-1.4339) | A |
| 111 | 27 | B | 1.427 | 20 | (8.0279,1.7803,3.3759)–(8.6807,2.3513,2.2448) | A |
| 113 | 28 | B | 0.909 | 21 | (8.0817,2.6871,3.3577)–(8.0279,1.7803,3.3759) | A |
| 122 | 32 | B | 0.766 | 37 | (10.7284,2.1697,-0.3841)–(11.3927,2.4887,-0.1756) | A |
| 127 | 33 | B | 0.258 | 39 | (10.5169,1.9426,-0.4059)–(10.7392,2.0552,-0.3411) | A |
| 128 | 33 | B | 0.377 | 22 | (9.4527,2.0414,-0.6657)–(9.4613,1.6745,-0.7495) | A |
| 129 | 33 | B | 0.067 | 12 | (6.6838,3.6971,-1.2572)–(6.6837,3.6355,-1.2841) | A |
| 132 | 34 | B | 1.725 | 23 | (9.3209,2.0737,0.1514)–(9.3725,0.3907,-0.2207) | A |
| 32 | 7 | A | 0.149 | -1 | (6.6837,3.6355,-1.2841)–(6.6879,3.6342,-1.4327) | B |
| 91 | 25 | A | 0.089 | -1 | (6.6890,3.7234,-1.4339)–(6.6879,3.6342,-1.4326) | B |
| 117 | 29 | B | 0.266 | -1 | (6.8125,3.7162,-1.1820)–(6.8933,3.7673,-1.4304) | B |
| 130 | 33 | B | 0.150 | -1 | (6.8125,3.7162,-1.1820)–(6.6838,3.6971,-1.2572) | B |
| 74 | 19 | A | 1.310 | -1 | (8.0817,2.6871,3.3577)–(8.6807,2.3513,2.2448) | B |
| 71 | 19 | A | 1.269 | -1 | (9.4090,0.4502,-0.4258)–(9.4618,1.6745,-0.7536) | B |
| 73 | 19 | A | 0.828 | -1 | (9.3209,2.0737,0.1514)–(9.4527,2.0414,-0.6657) | B |
| 131 | 34 | B | 1.134 | -1 | (8.4004,0.1492,-0.7498)–(9.3725,0.3907,-0.2207) | B |
| 105 | 25 | A | 1.439 | -1 | (10.5169,1.9426,-0.4059)–(11.7516,1.7843,0.3133) | B |
| 142 | 25 | A | 0.221 | -1 | (11.4443,2.5291,-0.1643)–(11.5299,2.5687,-0.1352) | B |
| 143 | 25 | A | 0.162 | -1 | (11.5299,2.5687,-0.1352)–(11.6644,2.6499,-0.0961) | B |
| 144 | 25 | A | 0.029 | -1 | (11.6644,2.6499,-0.0961)–(11.6878,2.6650,-0.0896) | B |
| 139 | 25 | A | 0.068 | -1 | (11.6878,2.6650,-0.0896)–(11.6807,2.7270,-0.1168) | B |
| 123 | 32 | B | 0.057 | -1 | (10.7284,2.1697,-0.3841)–(10.7107,2.1546,-0.3322) | C |
| 126 | 33 | B | 0.104 | -1 | (10.7107,2.1547,-0.3322)–(10.7392,2.0552,-0.3411) | C |
| 125 | 33 | B | 0.067 | 40 f[0,7.08] | (11.3927,2.4887,-0.1756)–(11.4443,2.5291,-0.1643) | G |

## Class-B pairs (both paths + max divergence)

| pair | A path | B path | shared endpoints | max div |
|------|--------|--------|------------------|---------|
| leg | e32+e91(+e92): (6.6837,3.6355,-1.2841)→(6.6879,3.6342,-1.4327)→(6.6890,3.7234,-1.4339)→(6.8933,3.7673,-1.4304) | e129+e130+e117: (6.6837,3.6355,-1.2841)→(6.6838,3.6971,-1.2572)→(6.8125,3.7162,-1.1820)→(6.8933,3.7673,-1.4304) | both | ≈0.18–0.25 |
| strut-upper | e74 chord: (8.0817,2.6871,3.3577)→(8.6807,2.3513,2.2448) | e113+e111: (8.0817,2.6871,3.3577)→(8.0279,1.7803,3.3759)→(8.6807,2.3513,2.2448) | both | ≈0.89 |
| strut-lower | e90+e71: (8.4004,0.1492,-0.7498)→(9.4090,0.4502,-0.4258)→(9.4618,1.6745,-0.7536) | e131+e132+e73+e128: (8.4004,0.1492,-0.7498)→(9.3725,0.3907,-0.2207)→(9.3209,2.0737,0.1514)→(9.4527,2.0414,-0.6657)→(9.4613,1.6745,-0.7495) | both (ends ≈0.004 apart) | ≈1.0 |
| seat-corner low | e105: (10.5169,1.9426,-0.4059)→(11.7516,1.7843,0.3133) | e127+e126+e123+e122+e125: (10.5169,1.9426,-0.4059)→…→(11.4443,2.5291,-0.1643) | start only | ≈0.8 |
| seat-corner high | e142+e143+e144+e139: (11.4443,2.5291,-0.1643)→(11.5299,2.5687,-0.1352)→(11.6644,2.6499,-0.0961)→(11.6878,2.6650,-0.0896)→(11.6807,2.7270,-0.1168) | original e125 whole span (11.3927,2.4887,-0.1756)→(11.6807,2.7270,-0.1168) (seg40, pass-1 len 0.3785; mostly weld-consumed) | end (11.6807,2.7270,-0.1168) | ≈0.057 |

## Class-C vertex pairs
- v41 (10.728,2.170,-0.384) ↔ v43 (10.739,2.055,-0.341), gap 0.1227 — bridged by B-only e123+e126 (total 0.161 via detour vertex (10.7107,2.1546,-0.3322)); A never bridged.
- (root-cause C signatures also under class-B clusters: v19/v39 gap 0.1786 at leg; v29/v47 gap 0.2167 at strut-lower.)

## Cross-check
A=9, B=13, C=2, D=0, E=0, F=0, G=1 → **25 = naked total 25** ✓
Clusters: leg 6 + strut 9 + seat-corner 10 = 25 ✓

## Causal chain (dominant)
1. Operand-split whole-seg loss (class A, task #7): A operand drops 7 segments (4 on surf 15, 2 on surf 18, 1 on surf 7); B drops 2 (segs 34, 44). 9 one-sided naked edges.
2. Each loss forces the losing operand's closure-weld to invent seg=-1 bridges along a DIFFERENT route than the surviving side's true section edges → 13 class-B divergent edges (divergence 0.06–1.0, all ≫ weld tube 0.0212).
3. Scaffold left 3 valence-1 vertex pairs (gaps 0.12–0.22, ≫ tol3 0.0353) exactly at the 3 holes; bridge march did nothing (march=0). e123/e126 are pure undershoot bridges (class C).
4. seg40 asymmetric split (13 A-pieces vs 1 whole B-edge; A pieces mostly lost via microfrag/verdict) leaves the 0.067 e125 stub (class G).
