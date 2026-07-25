# Defect census — chairsROT x13y29 cut

Summary line: `chairsROT x13y29  cut   : faces 39 solid 0 naked 22 vol -1.0000`
Log: `census_x13y29.log` (same directory). Scaffold: chains=43 segs=44 verts=45, drop(verdict=31 micro=56), bridge(march=0 weld=0 resid=0), tol3=3.419e-02, weld tube=0.02051, XWELD pass1 welds=27 rescue=0, pass2 welds=12 rescue=0.

## Key scaffold facts

- `[SEGLOST] A seg=2  surfA=0  surfB=15` — and SPANSEG shows seg 2 missing on **BOTH** operands (A@presym missing {2,17}; B@presym missing {2,16,21,39,40,41}). seg2 span v4(9.4755,2.0398,-0.6449) -> v5(9.7647,2.0154,-0.7274).
- `[SEGLOST] A seg=17 surfA=13 surfB=19` — A lost, B kept.
- `[SEGAUDIT]` one-sided kept segs: 16,21,39,40,41 (A-only; B2 edges=0) and 17 (B-only; A2 edges=0). Segs 0,1,3 (keptB=1, A2 trims=1) healed by xweld2 (their first-pass 1-trim edges e73/e101/e105/e118 are gone from the final naked list).
- `[SCAF-VAL]` valence-1 pairs: v4(9.476,2.040,-0.645) <-> v43(9.461,1.639,-0.745) gap **0.4137**; v20(6.001,0.820,-0.843) <-> v23(5.998,0.607,-0.896) gap **0.2201**. `[SCAF-BRIDGE]` lines: none (bridge march=0 weld=0).
- `[SCAF-DROP]` (31 verdict drops): all ALL-OUT paves on pairs sA∈{0,1,3,7,14,16,17,18}; none matches the uv region of a lost seg (seg2 = sA0/sB15, seg17 = sA13/sB19 — no drops on those pairs) → no class-E naked edges.

## The 22 naked edges, classified

Vertices used below:
P1=(9.4490,2.0420,-0.6377)≈v4(+0.028) P2=(9.4736,2.0385,-0.8376) P3=v0=(9.3371,2.0689,0.0671)
P4=v43=(9.4606,1.6387,-0.7450) P5=(9.7712,2.0148,-0.7293)≈v5(+0.0068) P6=(12.6622,1.6390,1.0159)
P7=(9.4330,3.1739,6.8805) P8=(7.2644,3.2183,4.3511) P9=v5=(9.7647,2.0154,-0.7274)
P10=v38=(9.9456,2.2361,-0.7233) P11=v24=(8.6170,2.3261,-0.3964) P12=v28=(8.6510,0.7665,-0.8129)
P13=v21=(8.5784,0.6960,-0.8208) P14=v20=(6.0008,0.8203,-0.8435) P15=(6.0604,0.7966,-1.3580)
P16=v22=(6.0588,0.7576,-1.3556) P17=v23=(5.9979,0.6066,-0.8960) P18=v44=(7.8338,0.0494,-0.9158)

### Class A — one-sided segment loss (7 edges)

| edge | face/side | seg | span | lost on |
|---|---|---|---|---|
| e45  | f12 A | 16 | P14->P13 len 2.595 | B (SEGAUDIT B2 edges=0) |
| e65  | f17 A | 21 | P12->P11 len 1.615 | B |
| e87  | f22 A | 41 | P13->P18 len 0.991 | B |
| e88  | f22 A | 40 | P4->P12 len 1.192 | B |
| e96  | f24 A | 39 | P10->P9 len 0.286 | B |
| e120 | f38 B | 17 | P16->P17 len 0.488 | A ([SEGLOST] A seg=17) |
| e126 | f0 A  | -1 | P1->P5 len 0.336 | BOTH — this is SEGLOST seg2 (v4->v5) recreated as closure on f0 only |

Five B-side losses (16,21,39,40,41) all belong to one arm region; A-side loss (17) and the both-sides loss (2) complete the picture. These A-kept edges are naked purely because the mate operand never produced the seg.

### Class B — divergent bridge pairs (7 edges)

**Pair B1 (3 edges): e128 vs chain e3+e84 — missing vertex split at P7.**
- Path 1: e128 f0(A) seg=-1, P6(12.6622,1.6390,1.0159) -> P8(7.2645,3.2183,4.3511), len 10.3816 (strongly curved; chord only 6.54).
- Path 2: e84 f21(A) P8 -> P7(9.4330,3.1739,6.8805) len 3.3492  +  e3 f2(A) P7 -> P6 len 7.0412; chain len 10.390.
- Same endpoints (P6,P8); e128 is the whole span un-split at P7. Max divergence 0.0449 (NKPAIR e3<->e128 d=0.0449; e84<->e128 d=0.0103) — **exceeds weld tube 0.02051**, so xweld cannot mate them; a vertex imprint at P7 on e128 would reduce it to two weldable pairs.

**Pair B2 (2 edges): e1 vs e70.**
- e1  f1(A) P1 -> P2, len 0.2335 (curved).
- e70 f18(A) P2 -> P1, len 0.2014 (straight = chord).
- Endpoints identical (NKPAIR d=0.0002); arc-vs-chord sagitta ~0.05 > tube 0.02051 → weld rejected.

**Pair B3 (1 edge): e99 vs the A-side detour chain.**
- e99 f26(B) seg=-1, P11(v24) -> P10(v38), len 1.3726 (straight shortcut).
- A-side path between the same endpoints: e65(seg21) + e88(seg40) + [v43~v4 gap 0.414] + e126(seg2) + e96(seg39), total ≈ 3.8, dipping to v28(8.651,0.767,-0.813).
- Max divergence ≈ 1.55 (v28 to the straight line v24-v38). B lost every seg on this span, so its closure drew a chord across the whole arm.

**Pair B4 (1 edge): e119 vs chain e87+e45.**
- e119 f38(B) seg=-1, P17(v23) -> P18(v44), len 1.9247 (straight).
- A-side path: e87(seg41) v44->v21 + e45(seg16) v21->v20, total 3.586, ending 0.2201 away (the v20~v23 valence-1 gap).
- Max divergence ≈ 0.99 (v21 lies past the straight line v23-v44). Endpoint mismatch 0.2201 at the gap end.

### Class C — junction undershoot at valence-1 scaffold pairs (4 edges)

**Gap 1: v4 <-> v43, d=0.4137** (`[SCAF-VAL] v=4 ... nearest_v1=43 d=0.4137`)
- e69  f18(A) seg=-1, P4(v43) -> P2, len 0.4106 — closure from the dead-end v43 to non-network point P2.
- e129 f1(A)  seg=-1, P2 -> P1(≈v4), len 0.6401 — f1's long-way closure of the same dead zone (forms a bigon with e1).

**Gap 2: v20 <-> v23, d=0.2201** (`[SCAF-VAL] v=20 ... nearest_v1=23 d=0.2201`)
- e51 f14(A) seg=-1, P15 -> P14(v20), len 0.5186 — closure from dead-end v20.
- e52 f14(A) seg=-1, P15 -> P16(v22), len 0.0391 — micro stub (P15 sits 0.039 off v22).

### Class D — micro/triangle holes: 0

The only closed short loop is the f1 bigon e1+e129, but its two edges have distinct mechanisms (B2 pair / v4-v43 closure) and are counted there. No 2-3-edge closed micro loop survives as an independent mechanism.

### Class E — verdict drops: 0

No [SCAF-DROP] interval lies on the surface pairs of any lost seg or naked-edge uv region (drops are on sA∈{0,1,3,7,14,16,17,18} ALL-OUT paves, all elsewhere).

### Class F — unmerged identical mate pairs, NK-RESCUE missed (4 edges)

| pair | edges | span | lens | endpoint d |
|---|---|---|---|---|
| F1 | e121 f18(A) + e125 f0(A) | P1 <-> P3 | 0.7143 / 0.7143 | 0.0002 |
| F2 | e95 f24(A) + e127 f0(A) | P6 <-> P9/P5 | 3.4148 / 3.4148 | 0.0012 (P5-P9 = 0.0068 < tube 0.0205) |

Both pairs are **A-side <-> A-side** (different result faces of operand A). XWELD reports `rescue=0` in both passes and its merge step is "section edges merged A<->B" — same-operand 1-trim duplicates are apparently not weld-eligible, so two geometrically identical in-tolerance edges stayed as two 1-trim edges each.

### Class G — other: 0

## Cross-check

A=7, B=7, C=4, D=0, E=0, F=4, G=0 → **22 = naked total 22** ✓

## Mechanism chain (one story)

Operand-split seg loss (A) is the root: B lost segs 16,21,39,40,41, A lost 17, both lost 2. The kept sides go naked (7×A), and the losing operand invents straight shortcut closures across the missing spans (e99, e119 → B). Independently, two valence-1 scaffold gaps (v4-v43 0.414, v20-v23 0.220; bridge march=0) force per-face improvised closures (4×C) and divergent arc/chord twins (e1/e70) plus an un-split whole-span closure (e128 vs e3+e84) whose 0.045 divergence exceeds the 0.0205 weld tube (5×B). Finally 4 edges are pure bookkeeping misses: two identical A-A pairs within weld tolerance that the A<->B-only rescue never considers (F).
