# Defect census — chairsROT x20 cut

Log: `census_x20.log` (same dir). Summary line:
`chairsROT x20 cut : faces 35 solid 0 naked 9 vol -1.0000`

Pipeline stage counts: `[NT] combine edges1=18` → `co_refine edges1=27` → `xweld2 edges1=9` (final naked).
XWELD pass 1: welds=20 common-blocks=2 rescue=0; pass 2: welds=14 common-blocks=5 rescue=4 (4 identical mate pairs rescued — none left).
SCAF census: chains=30 segs=37 verts=39, paves(tA=46 tB=38 x=24 v=1 c=0), drop(verdict=21 micro=15), **bridge(march=0 weld=0 resid=0)**, dev(A=0 B=1.8e-08), tol3=3.577e-02.

## Seg-coverage asymmetry (root data)

SPANSEG presym: A keeps 33/37 segs (missing 4, 5, 29, 35); B keeps 34/37 (missing 2, 21, 36).
[SEGLOST] A: seg=4 (surf 2/16, n=49), seg=5 (3/16, n=49), seg=29 (18/6, n=50), seg=35 (18/18, n=7).
[SEGAUDIT] one-sided after retention: seg 2, 21, 36 → keptA=1 keptB=0 (B2 edges=0); seg 4, 5, 29 → keptA=0 keptB=1 (A2 edges=0); seg 3, 27, 28 keptA=1 keptB=0 but B2 edges=1 trims=1 (these + seg-2/36 region got fixed by xweld2 rescue=4 — e.g. e2/e3/e92 no longer naked).

Valence-1 scaffold vertices ([SCAF-VAL], 4 of 39):
- v6 (5.573,4.148,2.427) ↔ v8 (5.729,4.056,2.506), gap **0.1968** — ends of segs 4 and 5, never bridged.
- v28 (9.137,4.467,-1.305) ↔ v36 (9.124,4.369,-1.297), gap **0.0996** — ends of segs 21 and 29, never bridged.
No [SCAF-BRIDGE] lines exist; bridge(march=0 weld=0 resid=0).

## Edge-by-edge classification (9 naked)

| edge | side | face | NK seg | scaffold identity | class | endpoints |
|---|---|---|---|---|---|---|
| e80 | A | 20 | 21 | = seg 21 (v28→v29) exact | **A** (B lost seg 21) | (9.1367,4.4672,-1.3053)→(8.9170,4.5153,0.1888), len 1.511 |
| e100 | B | 29 | 29 | = seg 29 (v35→v36) exact | **A** (A lost seg 29) | (11.7858,3.8406,-0.4514)→(9.1242,4.3688,-1.2973), len 2.853 |
| e111 | B | 34 | -1 | geometry = seg 35 (v34→v35) exact | **A** (A lost seg 35; result edge lost seg tag) | (11.7858,3.8406,-0.4514)→(11.8211,4.2569,-0.5633), len 0.433 |
| e107 | B | 33 | 5 | = seg 5 (v7→v8) exact | **A** (A lost seg 5) | (5.9387,4.0397,2.4668)→(5.7286,4.0564,2.5058), len 0.214 |
| e108 | B | 33 | 4 | = seg 4 (v6→v7) exact | **A** (A lost seg 4) | (5.5732,4.1478,2.4271)→(5.9387,4.0397,2.4668), len 0.383 |
| e99 | B | 29 | -1 | invented; B's substitute for lost seg 21, re-anchored to v36 | **B** (mate e80) | (9.1242,4.3688,-1.2973)→(8.8302,3.9266,-0.1086), len 1.303 |
| e102 | B | 30 | -1 | invented; 2nd leg of B's seg-21 substitute path (P→v29) | **B** (mate e80) | (8.8302,3.9266,-0.1086)→(8.9170,4.5153,0.1888), len 0.665 |
| e86 | A | 23 | -1 | invented; A's substitute for lost segs 35+29 chain, re-anchored v34→v28 | **B** (mate e111+e100) | (11.8211,4.2569,-0.5633)→(9.1367,4.4672,-1.3053), len 2.803 |
| e106 | B | 33 | -1 | invented; spans unbridged valence-1 pair v8→v6 | **C** (gap 0.1968) | (5.7286,4.0564,2.5058)→(5.5732,4.1478,2.4271), len 0.197 |

Counts: **A=5, B=3, C=1, D=0, E=0, F=0, G=0. Sum = 9 = naked total.** ✓

## Class-B pair details (both paths + divergence)

Pair 1 — lost seg 21 (B side lost it):
- True path (A, kept): e80 = seg 21, v28(9.1367,4.4672,-1.3053) → v29(8.9170,4.5153,0.1888), f[0,48], 49 samples.
- Divergent substitute (B, invented): e99 v36(9.1242,4.3688,-1.2973) → P(8.8302,3.9266,-0.1086), then e102 P → v29(8.9170,4.5153,0.1888).
- Divergence: NKPAIR e80↔e99 d = 0.5194–0.5838 (max ≈ **0.58**); endpoint offsets: start v28↔v36 = 0.0996, mid-anchor P is **0.665** off e80's b-end (= e102's whole length).

Pair 2 — lost segs 35+29 (A side lost both):
- True path (B, kept): e111 = seg 35, v34(11.8211,4.2569,-0.5633) → v35(11.7858,3.8406,-0.4514); then e100 = seg 29, v35 → v36(9.1242,4.3688,-1.2973). Total 3.286.
- Divergent substitute (A, invented): e86 v34(11.8211,4.2569,-0.5633) → v28(9.1367,4.4672,-1.3053), len 2.803 (single chord replacing the 2-seg chain, far end remapped v36→v28).
- Divergence: NKPAIR e86↔e100 d = 0.3978–0.4055 (max ≈ **0.41**); start offset e86↔e100 = 0.4326 (= e111's whole length), end offset v28↔v36 = 0.0996.

## Class-C detail

- e106 spans valence-1 pair v8(5.729,4.056,2.506) ↔ v6(5.573,4.148,2.427), gap **0.1968** (> tol3 3.577e-02, so no weld; bridge march=0 never attempted). B invented e106 to close its loop; A has nothing there (lost segs 4+5 entirely).
- Second undershoot v28↔v36 gap **0.0996** is embedded inside the two class-B complexes (it is the seam through which the 6-edge band closes); no separate edge spans it.

## Hole topology (manifestation view)

The 9 edges form exactly 2 naked loops:
1. **6-edge sliver band** across faces 20/23/29/30/34: v28 -e80→ v29 -e102ʳ→ P -e99ʳ→ v36 -e100ʳ→ v35 -e111ʳ→ v34 -e86→ v28. One long thin wedge between each side's version of the section near the seat-back junction.
2. **3-edge triangle hole** on face 33 (class-D manifestation of A+C mechanisms): e107 + e106 + e108, verts 14(5.9387,4.0397,2.4668) / 101=v8 / 102=v6, sides 0.214/0.197/0.383.

## Class E/F negative evidence

- E=0: the 21 [SCAF-DROP] ALL-OUT verdicts are on surface pairs {0,14,16,18}×{14,15,16,17,18}; naked segs live on pairs (16,5) seg21, (18,6) seg29, (2,16) seg4, (3,16) seg5, (18,18) seg35. Only (18,18) overlaps as a pair; its 3 drops sit at uvB(-0.929,0.951), (0.427,-0.039), (1.409,-0.689) — disjoint from seg 35's uvB interval (-1.837,1.627)→(-1.488,1.369). Drops happen at scaffold construction; every naked edge maps to a *kept* seg or an invented closure, so no naked region is a dropped interval.
- F=0: no two naked 1-trim edges share endpoints+length (closest: e80/e99 differ 0.0996 & 0.665; e86/e100 differ 0.4326 & 0.0996). xweld2 rescue=4 already caught all genuinely identical mate pairs (seg 3/27/28 + seg-2/36 region, edges e2/e3/e69/e70/e87/e92/e101/e103/e105/e109 cleared).

## Root cause chain

Symmetric-coverage connexity flood retained 7 scaffold segs one-sidedly (B lost {2,21,36}, A lost {4,5,29,35}); 2 of those regions self-healed via xweld2 rescue; segs {4,5,21,29,35} survived one-sided → the losing side's closure-weld invented divergent substitute paths (e99+e102, e86, e106) anchored at the 4 unbridged valence-1 scaffold vertices (v28/v36 gap 0.0996, v6/v8 gap 0.1968) instead of the true seg endpoints → one 6-edge sliver band + one triangle hole = 9 naked, solid 0.
