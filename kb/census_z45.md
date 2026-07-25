# Defect census — chairsROT z45 cut

Summary line: `chairsROT z45     cut   : faces 36 solid 0 naked 32 vol -1.0000`
Log: `census_z45.log` (same dir). Scaffold: `[SCAF] chains=47 segs=43 verts=45 paves(tA=62 tB=60 x=137 v=3 c=0) drop(verdict=39 micro=33) bridge(march=0 weld=0 resid=0) dev(A=0 B=1.8e-08) tol3=3.567e-02`
XWELD pass1: welds=25 common-blocks=4 micro=2 rescue=0; pass2 (xweld2): welds=10 common-blocks=2 micro=0 rescue=3.

## Seg bookkeeping (root data)

- `[SEGLOST] A` (whole-seg lost in A's operand split): segs 20,21,23,24,25,27,29,30,31,38,39 (11 segs, all open; n up to 133 pts).
- `SPANSEG A@presym` = A merged list (A lost nothing post-presym).
- `SPANSEG B@presym` contains 24,25,26 but `[SCAF-MERGE] B segs` does NOT → B dropped segs 24,25,26 AFTER presym (symmetric-coverage flood). B never had 4,6,8,9,14,16,29,36,37.
- One-sided kept segs ([SEGAUDIT]): A-only = 4,6,8,9,14,16,26,36,37; B-only = 20,21,23,27,30,31,38,39. Both = 7 only.
- Segs missing on BOTH operands: 24,25 (A split-lost + B flood-dropped), 26 (B flood-dropped; A kept → e71 one-sided), 29 (never on either side; spans valence-1 v35→v26).
- Valence-1 scaffold vertices ([SCAF-VAL]): v21(7.761,4.958,2.783)↔v35(7.789,4.984,2.789) gap 0.0385; v40(9.473,2.020,-0.833)↔v41(9.576,2.031,-0.802) gap 0.1083. No [SCAF-BRIDGE] lines (bridge march=0 weld=0 resid=0).
- [SCAF-DROP]: 39 lines, ALL are `ALL-OUT` interval culls; none of their uv points falls on a kept naked-edge span (checked surf pairs (16,14),(16,16),(18,15) — uvB coords do not lie inside seg 24/25/29/39 uv ranges). No naked edge is attributable to a verdict drop of a kept interval → class E empty. The drops on pairs (16,14)/(16,16)/(18,15) are however circumstantially near the both-lost segs.

## Topology of the naked set

All 32 naked edges form ONE giant ring plus one 3-edge open chain on face 22 (A):

Ring (28 edges), junctions at v58=(7.9811,4.4931,-1.4106) and the v21/v35 gap:
B/A section arc: e103(B,s23) → e101(B,-1) → e98(B,-1) → e71(A,s26) → e35(A,s9) → e28(A,s8) → e58(A,s14) → e88(A,s36) [dup e123] → e86(A,-1, v40↔v41) → e85(A,s37) → e107(B,-1) → e104(B,-1) → e117(B,-1) → e118(B,s31) → e110(B,s27) → e91(B,s20) → e119(B,s30) → e96(B,s21) → e97(B,-1) → e20(A,s6) → e13(A,s4) → e56(A,s16) → [gap v21↔v35 0.0385, stub e55] 
A closure arc (returns to v58): e16(A,-1) → e22(A,-1) → e75(A,-1) → e11(A,-1) → e79(A,-1), total len 21.6.

Face-22 chain (A): e123 (8.6962,2.0424,-1.0617)→(9.4730,2.0198,-0.8335) → e124 →(9.4182,0.6287,-0.4779) → e125 →(8.6460,0.6520,-0.7871). e123 duplicates e88 exactly; e125 duplicates e112's endpoints with different geometry.

## Per-edge classification (32 edges)

| e | face | side | len | seg | class | note |
|---|------|------|-----|-----|-------|------|
| e13 | 2 | A | 1.343 | 4 | A | seg4 A-only (SEGAUDIT keptB=0) |
| e20 | 3 | A | 0.490 | 6 | A | seg6 A-only |
| e28 | 6 | A | 2.059 | 8 | A | seg8 A-only |
| e35 | 8 | A | 0.941 | 9 | A | seg9 A-only |
| e56 | 15 | A | 0.041 | 16 | A | seg16 A-only; far end = valence-1 v21 |
| e58 | 16 | A | 1.672 | 14 | A | seg14 A-only |
| e71 | 19 | A | 0.754 | 26 | A | seg26: A kept, B flood-dropped post-presym |
| e85 | 23 | A | 1.546 | 37 | A | seg37 A-only |
| e91 | 25 | B | 0.655 | 20 | A | seg20 B-only (A SEGLOST) |
| e96 | 26 | B | 1.068 | 21 | A | seg21 B-only (A SEGLOST) |
| e103 | 28 | B | 1.016 | 23 | A | seg23 B-only (A SEGLOST) |
| e110 | 31 | B | 0.402 | 27 | A | seg27 B-only (A SEGLOST) |
| e112 | 31 | B | 0.834 | 38 | A | seg38 B-only (A SEGLOST); divergent A mate e125 |
| e118 | 33 | B | 0.850 | 31 | A | seg31 B-only (A SEGLOST) |
| e119 | 34 | B | 2.804 | 30 | A | seg30 B-only (A SEGLOST, n=133) |
| e79 | 23 | A | 4.459 | -1 | B | mega-closure (7.9811,4.4931,-1.4106)→(12.2662,4.1988,-0.3540); div 2.74 vs real B section e85 |
| e11 | 1 | A | 6.684 | -1 | B | mega-closure (12.2662,4.1988,-0.3540)→(9.8843,5.2797,5.6253); div 1.85–1.93 vs e119 |
| e75 | 21 | A | 6.358 | -1 | B | mega-closure (9.8843,5.2797,5.6253)→(4.2323,5.2308,3.3103); div 2.33 vs e97 |
| e22 | 4 | A | 1.077 | -1 | B | mega-closure (4.2323,5.2308,3.3103)→(4.7779,4.8443,2.4715); div 0.80 |
| e16 | 2 | A | 3.014 | -1 | B | mega-closure (4.7779,4.8443,2.4715)→(7.7517,4.9920,2.8046); div 1.09 |
| e97 | 26 | B | 1.456 | -1 | B | closure (8.0452,5.1209,3.3612)→(6.9830,4.1813,3.0344) shortcutting A chain e20+e13+e56 + both-lost seg29; div 0.47–0.55 |
| e98 | 27 | B | 0.399 | -1 | B | closure (7.6540,4.1983,-0.5614)→(7.4146,4.5137,-0.5140); mate e71 (seg26), div 0.35 |
| e101 | 28 | B | 0.556 | -1 | B | closure (8.0834,4.5129,-0.4001)→(7.6540,4.1983,-0.5614) replacing B-dropped seg25; div 0.45 vs e103 |
| e104 | 29 | B | 1.132 | -1 | B | closure (10.7249,2.9241,-0.3322)→(11.5555,3.6623,-0.1200) paralleling MATED seg34 edge (e83); div 0.38 |
| e107 | 30 | B | 0.501 | -1 | B | closure (10.9789,2.4945,-0.3766)→(10.7249,2.9242,-0.3322) paralleling MATED seg35 edge (e84); div 0.23 |
| e125 | 22 | A | 1.202 | -1 | B | closure (9.4182,0.6287,-0.4779)→(8.6460,0.6520,-0.7871); same endpoints as e112 (d=4e-4) but len 1.202 vs 0.834 (chord 0.832) → curved detour, max div ≈0.36 |
| e86 | 23 | A | 0.109 | -1 | C | bridge across valence-1 pair v40(9.473,2.020,-0.833)↔v41(9.576,2.031,-0.802), gap 0.1083; naked (B has no counterpart) |
| e55 | 15 | A | 0.052 | -1 | C | stub (7.7521,4.9920,2.8047)→(7.7761,4.9770,2.7607) at valence-1 pair v21↔v35, gap 0.0385 |
| e88 | 23 | A | 0.812 | 36 | F | identical to e123: same endpoints (8.6962,2.0424,-1.0617)/(9.4730,2.0198,-0.8335), same len 0.8117, NKPAIR d=0.0000 |
| e123 | 22 | A | 0.812 | -1 | F | dup of e88 — both 1-trim, BOTH on operand A (f22/f23); xweld2 rescue=3 missed this pair |
| e117 | 33 | B | 0.798 | -1 | G | closure (11.5555,3.6622,-0.1200)→(11.1190,4.3293,-0.0897); NO mate path — corner between seg34-end and seg31-end has no scaffold seg (segs 30/31 A-lost, nearest edge e105 d=0.69) |
| e124 | 22 | A | 1.437 | -1 | G | closure (9.4730,2.0198,-0.8335)→(9.4182,0.6287,-0.4779); NO mate — spans seg36-end→seg38-start where A's surf-15 coverage lives on other faces (nearest e88 d=1.26) |

## Class-B pair detail

1. e125 (A f22) vs e112 (B f31, seg38): endpoints match to 4e-4 at (9.4182,0.6287,-0.4779)/(8.6460,0.6520,-0.7871); e112 len 0.834 ≈ chord (straight), e125 len 1.202 (curved detour). Max divergence ≈0.36 (arc estimate from len/chord ratio 1.445).
2. A mega-chain {e79,e11,e75,e22,e16} (total 21.6) vs the entire B-side arc between junction v58=(7.9811,4.4931,-1.4106) and the v21/v35 gap: max divergence 2.74 (e79↔e85), 1.93 (e11↔e119), 2.33 (e75↔e97), 1.09 (e16↔e75), 0.80 (e22↔e75). A's faces 1,2,4,21,23 lost their section segs entirely and welded one giant free path.
3. e97 (B) (8.0452,5.1209,3.3612)→(6.9830,4.1813,3.0344) vs A path e20+e13+e56 (v9→v6→v7→v21) + both-lost seg29 (v35→v26=(8.0452,5.1209,3.3612)): div 0.47–0.55.
4. e98 vs e71 (share (7.4146,4.5137,-0.5140)) div 0.349; e101 vs e103 (share (8.0834,4.5129,-0.4001)) div 0.445.
5. e104 vs mated seg34 edge e83: endpoint offsets 0.30/0.40, NKPAIR d=0.3835. e107 vs mated seg35 edge e84: shares (10.9789,2.4945,-0.3766), NKPAIR d=0.2262. Both closures parallel PROPERLY MATED 2-trim edges — the closure welder re-routed instead of landing on them.

## Class-C detail

- Pair v40(9.473,2.020,-0.833)↔v41(9.576,2.031,-0.802), gap 0.1083: seg36 ends at v40, seg37 starts at v41 (A-only segs on sA=18/sB=14); bridge e86 fills the gap on A but B has nothing → e86 naked. tol3=0.0357 < 0.1083, so closure-weld cap could not weld it.
- Pair v21(7.761,4.958,2.783)↔v35(7.789,4.984,2.789), gap 0.0385 (≈ tol3 0.0357, just over): seg16 ends at v21, seg29 (both-lost) started at v35; stubs e55/e56 (len 0.052/0.041) are the residue.

## Cross-check

A=15, B=11, C=2, D=0, E=0, F=2, G=2 → 32 = naked total. ✓
Sides: A-side naked 18, B-side naked 14 (matches NK side tags).

## Dominant mechanisms

1. One-sided whole-seg coverage (15 A + root cause of all 11 B closures): A's operand split lost 11 whole segs ([SEGLOST] segs 20,21,23,24,25,27,29,30,31,38,39 — all with surfA=16 or 18, the two big freeform A surfaces), and B's symmetric-coverage flood then dropped segs 24,25,26. Every surviving section edge in the ring is 1-trim.
2. Divergent closure-weld bridges: each operand closed its face loops with free geometry instead of the mate's section (worst: A mega-chain, div up to 2.74; subtlest: e125 vs e112, same endpoints, div 0.36; e104/e107 parallel already-MATED edges at div 0.23–0.38).
3. Junction undershoot at the 2 valence-1 vertex pairs (gaps 0.1083 and 0.0385 vs tol3 0.0357) — 3 micro edges (e86, e55, + e56's dangling end).
4. Missed identical-pair rescue: e88/e123 (d=0.0000, both operand-A faces f23/f22) — xweld2 rescued 3 pairs but skipped this one.
