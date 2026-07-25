# Defect census — chairsROT z63 cut

Log: `census_z63.log` (841 lines). Summary line (log:837):

```
chairsROT z63     cut   : faces 39 solid 0 naked 13 vol -1.0000
```

Pipeline state markers:
- `[SCAF]` (log:211): chains=50 segs=45 verts=45 paves(tA=64 tB=64 x=132 v=2 c=0) drop(verdict=40 micro=33) bridge(march=0 weld=0 resid=0) dev(A=0.000e+00 B=1.810e-08) tol3=3.633e-02
- `[SCAF-VAL]` (log:165): valence1=0 of 45 vertices → scaffold itself is closed, no junction undershoot anywhere.
- `[XWELD]` pass 1 (log:776): welds=33 common-blocks=6 micro=1 rescue=0 tube=0.0218
- `[XWELD]` pass 2 / xweld2 (log:795): welds=4 common-blocks=1 micro=0 rescue=1 tube=0.0218
- No `[SEGLOST]` lines, no `[SCAF-BRIDGE]` lines in this run.
- `[SPANSEG]` (log:764-765): A@presym n=45 has ALL segs 0..44; B@presym n=41 is missing segs **25, 34, 35, 36** — the exact segs that later show up naked on A.
- `[SEGAUDIT]` (log:770-773) — the one-sided segs:
  ```
  seg=25 keptA=1 keptB=0 | A2 edges=1 trims=2  B2 edges=0 trims=0
  seg=34 keptA=1 keptB=0 | A2 edges=1 trims=2  B2 edges=0 trims=0
  seg=35 keptA=1 keptB=0 | A2 edges=1 trims=2  B2 edges=0 trims=0
  seg=36 keptA=1 keptB=0 | A2 edges=1 trims=2  B2 edges=0 trims=0
  ```

## Naked-edge inventory (13 edges, log:811-836)

| e | face | side | len | seg | a | b |
|---|------|------|-----|-----|---|---|
| 74 | 18 | A | 0.458 | 36 | (11.6220,4.2731,-0.1562) | (11.8632,4.2517,-0.5447) |
| 75 | 18 | A | 0.970 | 35 | (11.0755,4.3442,0.6421) | (11.6220,4.2731,-0.1562) |
| 78 | 18 | A | 1.913 | 34 | (8.3518,5.1595,3.6686) | (9.6637,4.6641,2.3709) |
| 79 | 18 | A | 1.021 | 25 | (7.3701,5.2366,3.4002) | (8.3518,5.1595,3.6686) |
| 103 | 26 | B | 1.890 | -1 | (9.6637,4.6641,2.3709) | (9.3734,3.5882,3.8945) |
| 107 | 28 | B | 1.889 | -1 | (9.3734,3.5882,3.8945) | (8.3518,5.1595,3.6686) |
| 108 | 28 | B | 1.046 | -1 | (8.3518,5.1595,3.6686) | (7.7145,5.9709,3.4985) |
| 109 | 28 | B | 0.831 | -1 | (7.7145,5.9709,3.4985) | (7.3396,5.2378,3.3917) |
| 121 | 34 | B | 0.929 | -1 | (11.7367,3.6932,0.5881) | (11.0755,4.3442,0.6421) |
| 123 | 35 | B | 1.148 | -1 | (6.7854,4.2589,3.1641) | (7.3397,5.2377,3.3920) |
| 127 | 38 | B | 1.252 | -1 | (11.8790,4.2314,-0.5322) | (11.7367,3.6932,0.5881) |
| 128 | 18 | A | 0.028 | -1 | (11.8882,4.2485,-0.5335) | (11.8632,4.2517,-0.5447) |
| 131 | 28 | B | 1.148 | -1 | (7.3396,5.2378,3.3917) | (6.7854,4.2589,3.1640) |

## Classification

### Class A — SEGLOST one-sided: 4 edges (e74, e75, e78, e79)

All four are A-side (face 18) section edges whose seg exists in the scaffold but produced edges on operand A ONLY (SEGAUDIT keptA=1 keptB=0, B2 edges=0 trims=0; SPANSEG B@presym missing 25/34/35/36).

Seg identities (from `[SCAF-SEG]`, log:145,154-156):
- seg=25: sA=16 sB=1  uvB(0.0255,4.8884)->(0.8650,5.4697)  p3 (7.3701,5.2366,3.4002)->(8.3518,5.1595,3.6686) → e79
- seg=34: sA=16 sB=18 uvB(-1.4826,-3.1117)->(-0.0148,-1.8891) p3 (8.3518,5.1595,3.6686)->(9.6637,4.6641,2.3709) → e78
- seg=35: sA=16 sB=18 uvB(1.5093,-0.2338)->(2.0837,0.5412)  p3 (11.0755,4.3442,0.6421)->(11.6220,4.2731,-0.1562) → e75
- seg=36: sA=16 sB=18 uvB(2.0837,0.5412)->(2.3338,0.9213)   p3 (11.6220,4.2731,-0.1562)->(11.8632,4.2517,-0.5447) → e74

Pattern: the lost-on-B segs are exactly the sA=16 segs whose sB is 1 or 18 (B kept every other sA=16 seg: 24,26,27,28,29,30,31,32,33). Upstream correlation — `[SCAF-DROP] ALL-OUT` verdicts on the same surface pairs bracket these seg corridors:
- sA=16 sB=18 drops at uvB(-1.728,-3.285) (just past seg34 start) and uvB(0.772,-1.092) (between seg34 end and seg35 start) — log:96-97
- sB=1 drops at uvB(0.216,4.621) [sA=14, log:82] near seg25's uvB range (0.03..0.87, 4.89..5.47), plus sA=19 sB=1 drops (log:114-115)

These drops are on the same intersection-curve corridors but at uv points OUTSIDE the retained seg intervals; the segs themselves survived the scaffold (they are SCAF-SEG entries, valence-closed). The hard fact remains: B-side split never ran/kept them (no `[SCAF-RUN]` for segs 25/34/35/36 — only segs 37-44 got MATED). So the primary class is A (one-sided seg), with the SCAF-DROP correlation noted as probable root cause rather than class E proper.

### Class B — divergent bridges: 6 edges (e103, e107, e108, e109, e121, e127)

B-side seg=-1 invented closure edges that close B's loops where segs 25/34/35/36 are missing. Each B path shares endpoints (exact or near) with an A-side class-A edge but detours through off-curve geometry.

**Pair B1 — mate e78 (seg34), B path e103+e107, max divergence 1.42:**
- A: e78  (8.3518,5.1595,3.6686) -> (9.6637,4.6641,2.3709), len 1.913 (straight along seg34)
- B: e103 (9.6637,4.6641,2.3709) -> (9.3734,3.5882,3.8945), len 1.890 (face 26)
     e107 (9.3734,3.5882,3.8945) -> (8.3518,5.1595,3.6686), len 1.889 (face 28)
- Chain endpoints match e78 exactly; detour vertex (9.3734,3.5882,3.8945). NKPAIR: e103<->e78 d=1.3727/1.4217, e107<->e78 d=1.3800. Max divergence ~1.42.

**Pair B2 — mate e79 (seg25), B path e108+e109, max divergence 0.66:**
- A: e79  (7.3701,5.2366,3.4002) -> (8.3518,5.1595,3.6686), len 1.021
- B: e108 (8.3518,5.1595,3.6686) -> (7.7145,5.9709,3.4985), len 1.046 (face 28)
     e109 (7.7145,5.9709,3.4985) -> (7.3396,5.2378,3.3917), len 0.831 (face 28)
- One shared endpoint exact (e79.b); far end mismatched by 0.032 ((7.3396,5.2378,3.3917) vs e79.a (7.3701,5.2366,3.4002)) — gap > weld tube 0.0218, < tol3 0.0363. Detour vertex (7.7145,5.9709,3.4985). NKPAIR: e108 d=0.6594, e109 d=0.6555 vs e79. Max divergence ~0.66.

**Pair B3 — mates e74+e75 (segs 36+35), B path e127+e121, max divergence 0.74:**
- A: e74 (11.6220,4.2731,-0.1562) -> (11.8632,4.2517,-0.5447), len 0.458
     e75 (11.0755,4.3442,0.6421) -> (11.6220,4.2731,-0.1562), len 0.970
- B: e127 (11.8790,4.2314,-0.5322) -> (11.7367,3.6932,0.5881), len 1.252 (face 38)
     e121 (11.7367,3.6932,0.5881) -> (11.0755,4.3442,0.6421), len 0.929 (face 34)
- Chain end at e75.a exact; chain start (11.8790,4.2314,-0.5322) is 0.028 from e74.b (and 0.0195 from e128.a). Detour vertex (11.7367,3.6932,0.5881). NKPAIR: e121<->e75 d=0.6962/0.7381, e127<->e75 d=0.6915, e127<->e74 d=0.2610. Max divergence ~0.74.

### Class C — junction undershoot: 0 edges
`[SCAF-VAL] valence1=0 of 45 vertices`; no `[SCAF-BRIDGE]` lines. Scaffold fully valence-2.

### Class D — micro/triangle hole: 1 edge (e128)
- e128 f18(A) len 0.0276: (11.8882,4.2485,-0.5335) -> (11.8632,4.2517,-0.5447). NKPAIR near e74 d=0.0242, near e73 d=0.0242.
- Genesis: pass-1 X1T shows 1-trim e73 f18(A) v(10,57) b=(11.8632,4.2517,-0.5447) and e99 f24(A) v(10,73) b=(11.8882,4.2485,-0.5336). xweld2 rescue=1 merged e73/e99 (both vanish from the pass-2 X1T list) but their far endpoints v57/v73 differ by 0.0276 — ABOVE weld tube 0.0218, BELOW tol3 0.0363 — so a micro stub e128 was emitted to close the gap. It stays naked because its continuation e74 (class A) is naked.

### Class E — verdict drop matching a naked edge: 0 edges (as primary class)
40 verdict drops and 33 micro drops exist (`[SCAF]` census). The sA=16×sB=18 and sB=1 ALL-OUT drops (log:82,96,97,114,115) sit on the same intersection corridors as the class-A segs, but at uv points outside the naked edges' retained intervals, and the segs themselves survived into the scaffold. Counted as root-cause correlation for class A, not as a separate class-E defect.

### Class F — unmerged identical mate pair: 2 edges (e123, e131)
- e123 f35(B) v(18,115): (6.7854,4.2589,3.1641) -> (7.3397,5.2377,3.3920), len 1.1482
- e131 f28(B) v(130,88): (7.3396,5.2378,3.3917) -> (6.7854,4.2589,3.1640), len 1.1482
- Same endpoints (reversed), same length, NKPAIR d=0.0003 — far inside weld tube 0.0218. Both are 1-trim; welding them makes both 2-trim (2 naked recovered outright). NK-RESCUE missed them because their endpoint vertex ids differ (v18/v115 vs v130/v88; the 115-vs-130 positions differ by 3e-4 but were never unified). e131 was born late: in pass-1 e109 was len 1.9787 (6.7854..)->(7.7145..); co_refine split it at (7.3397,5.2377,3.3920) producing e131 (log:794 co_refine edges1 15->17), after which no weld pass revisited the pair (xweld2 rescue budget spent on e73/e99).

### Class G — other/unknown: 0 edges

## Cross-check
A(4) + B(6) + C(0) + D(1) + E(0) + F(2) + G(0) = 13 = naked total. OK.

## Dominant mechanisms (ranked)
1. **B-side loss of segs 25/34/35/36 (all sA=16, sB∈{1,18})** — produces 4 class-A naked on A face 18 AND forces B to invent the 6 divergent class-B closure edges around the same holes (10 of 13 naked). Fixing B's retention of these segs collapses both classes at once. Suspect: ALL-OUT verdict on the sA=16×sB=18 / ×sB=1 corridors starving B's runs (segs never appear in any B `[SCAF-RUN]`).
2. **Missed identical-mate weld e123/e131** (d=3e-4 << tube 0.0218) — 2 naked; a post-co_refine re-weld pass or vertex unification at 3e-4 rescues both.
3. **Micro stub e128** (0.0276 gap: above tube 0.0218, below tol3 0.0363) — 1 naked; a tube ceiling at tol3 for endpoint rescue welds absorbs it (and note pair-B2's 0.032 far-end gap is in the same tube..tol3 window).
