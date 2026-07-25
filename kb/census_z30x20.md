# Defect census: chairsROT z30x20 cut (SESSION_NO_MERGE=1 SESSION_NO_SEW=1 SESSION_FAST=1)

Log: census_z30x20.log (600 lines). Run exit 0.

## Summary line
```
chairsROT z30x20  cut   : faces 34 solid 1 naked 0 vol -1.0000
```
NAKED = 0. Closed solid. (vol -1.0000 = volume not evaluated under SESSION_FAST; "0/0 cells OK" because zzzz filter skips the gate.) No defects to classify — classes A-G all count 0. Below is the healthy-pipeline baseline for comparison against failing configs.

## Class counts
| Class | Description | Count |
|---|---|---|
| A | SEGLOST one-sided | 0 (no [SEGLOST]/[SEGAUDIT] lines emitted) |
| B | divergent bridge (seg=-1) | 0 |
| C | junction undershoot (valence-1) | 0 ([SCAF-VAL] valence1=0 of 33 vertices) |
| D | micro/triangle hole | 0 |
| E | verdict drop matching naked uv | 0 (25 [SCAF-DROP] ALL-OUT intervals exist but produce no holes) |
| F | unmerged identical mate pair | 0 (post-combine edges1=0) |
| G | other | 0 |
| **Total naked** | | **0** |

## Healthy-pipeline state (baseline metrics)

### Scaffold census (line 150)
```
[SCAF] chains=32 segs=34 verts=33 paves(tA=48 tB=40 x=24 v=2 c=0) drop(verdict=25 micro=18)
       bridge(march=0 weld=0 resid=0) dev(A=0.000e+00 B=1.778e-08) tol3=3.587e-02
```
- **Bridge residual = 0** (march=0 weld=0 resid=0): no invented closure edges, no seg=-1 bridges needed.
- Verdict drops: 25 ALL-OUT intervals + 18 micro drops — all legitimately outside; none adjacent to a hole.
- Section-curve deviation: A exact (0), B 1.8e-8 — both far under tol3=3.6e-2.

### Scaffold graph health
- [SCAF-VAL] valence1=0 of 33 vertices — fully closed section graph. 32 verts val=2, one junction vert (v10 at 6.6572,3.9904,-0.6027) val=4. No [SCAF-BRIDGE] lines needed.
- [SCAF-EXT] endpoint pins: 43 lines, max d3=0.0047 (sA=18 sB=15 end=1), 9 ONBND moved=0.0000; all pins well under tol3.
- [SCAF-SPLITV]: 3 mixed-verdict intervals resolved into runs (sA=7/sB=14, sA=14/sB=16, sA=16/sB=18) — verdict splitter working.

### Seg audit symmetry (lines 577-578)
```
[SPANSEG] A@presym n=34: 0..33   [SPANSEG] B@presym n=34: 0..33
```
Perfectly symmetric — every one of the 34 scaffold segs has edges on BOTH operands (this is exactly what [SEGLOST] would flag if broken; the tag never fires).

### SCAF-RUN mating
All 34 segs on both the B-side split (si=0..18) and A-side split report **MATED** (68 MATED lines, 0 unmated). Non-whole runs (seg=9 fa=0.7836, seg=10 fa=0.4902, seg=14 fb=48.0200, seg=29 fa=0.2188, seg=30 fb=32.7574) all still MATED — partial-span mating works.

### Weld counts
```
[SCAF-MERGE] section edges merged A<->B: 29 (A-side 34, B-side 34)
[XWELD] welds=29 common-blocks=5 micro=0 rescue=0 tube=0.02152   (main pass)
[XWELD] welds=0  common-blocks=0 micro=0 rescue=0 tube=0.02152   (xweld2: nothing left)
```
29 merges + 5 common blocks = 34 section pairings, matching segs=34. rescue=0 — NK-RESCUE never needed (class F empty by construction).

### Edge-manifold ledger ([NT])
- A2 edges1=1/138, B2 edges1=1/136 pre-combine (one boundary edge each, expected pre-merge)
- combine / blocks / imprint_edges / co_refine / xweld2 / sew: edges1=0 edges4+=0 at every stage — manifold throughout after combine, with sew disabled (SESSION_NO_SEW=1) and nothing for it to do.

### Splitter / classification
- [SCAF-SPLIT] side=B sec_edges=32 fallbacks=0; side=A sec_edges=31 fallbacks=0.
- One [SEGWHOLE] keep=0 -> [SEGFALL] straight chord at si=2 cidx=4 (ta=tb=0, degenerate whole-seg alias); harmless here. Second SEGWHOLE (si=7 cidx=9) keep=1.
- [BLOCKS A]/[BLOCKS B]/[BLOCKS R]: split=0 merged=0 cap=0 micro=0 projfail=0.
- [RADIAL] blocks=68 certA=25 certB=26 conflicts=0 — radial classification clean.
- [MICROFRAG] A dropped=1, B dropped=1 (tol=0.07174) — one micro fragment each side, absorbed without holes.

### Cross-check
Class sum 0 == naked total 0. Consistent.

Note: the trailing gate lines (`chairs [id1] cut+common-A rel 1.00e+00`, `[id2] fuse-(A+B-com) rel 2.00e+00`) are the identity checks running without volumes (SESSION_FAST) — rels are the degenerate -1-volume ratios, not geometry failures.
