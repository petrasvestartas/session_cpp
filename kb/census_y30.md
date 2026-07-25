# Defect census — chairsROT y30 cut

Summary line: `chairsROT y30     cut   : faces 42 solid 0 naked 13 vol -1.0000`
Log: `census_y30.log` (this directory). Exe run with SESSION_NO_MERGE=1 SESSION_NO_SEW=1 SESSION_FAST=1 SESSION_NT_DBG=1 SESSION_SPLIT_DBG=1.

## Global state

- `[SCAF] chains=44 segs=48 verts=50 paves(tA=60 tB=53 x=209 v=1 c=0) drop(verdict=28 micro=48) bridge(march=0 weld=0 resid=0) dev(A=0 B=1.6e-08) tol3=3.332e-02`
- `[SCAF-VAL] valence1=4 of 50` — TWO open junction pairs, and `bridge(march=0 weld=0)`: no scaffold bridging ran. NO `[SCAF-BRIDGE]` lines exist in the log.
  - pair 1: v7 (11.935,4.196,1.629) <-> v8 (12.016,3.745,1.710), gap 0.4656
  - pair 2: v23 (6.043,0.803,-1.189) <-> v27 (6.034,0.552,-1.198), gap 0.2512
- `[SEGLOST] A seg=4 (sA1/sB16) seg=5 (sA1/sB18) seg=17 (sA13/sB15) seg=18 (sA13/sB19)` — A-side whole-seg losses.
- `[SEGAUDIT]` one-sided segs: 4,5,17,18 (keptA=0 keptB=1) and 15,41 (keptA=1 keptB=0). Six one-sided segs total; B-side losses 15/41 have no [SEGLOST] print (audit only).
- `[SCAF-MERGE] section edges merged A<->B: 40 (A-side 44, B-side 46)` — common segs = 42, so 2 common segs failed cross-merge (seg 2 is one: A ran it PARTIAL unmated, see class B1).
- `[XWELD] welds=27 common-blocks=2 micro=1 rescue=0` then `welds=4 rescue=0` — NK-RESCUE fired 0 times (correct: no identical 1-trim pair exists, see class F).
- `[SCAF-DROP]` 28 verdict ALL-OUT drops: none of their uv regions coincide with the naked-edge seg bands (e.g. seg15/41 band on sB14 is u 0.00–0.68, v 1.26–2.81; nearest drops sit at u 1.6–4.6). Class E = 0.
- One `[JOINFAIL]` in whole run: A-side si=0 fi=2 `pieces=6 joined=2 closed=-1 gapmax=6.194e-03 tol=4.419e-03` (feeds class B1).

## The 13 naked edges ([NK])

| e | face | side | len | seg | span f[] | a | b | class |
|---|------|------|-----|-----|----------|---|---|-------|
| 9 | 1 | A | 0.509 | -1 | — | (9.9280,2.0012,-0.6632) | (9.4408,2.0442,-0.5241) | B (pair B1) |
| 13 | 2 | A | 0.387 | -1 | — | (12.0776,4.2130,0.9348) | (11.9983,4.2311,1.3130) | B (pair B2) |
| 56 | 14 | A | 1.623 | 15 | [0,48] | (7.6593,0.7553,-1.0950) | (6.0433,0.8030,-1.1889) | A |
| 60 | 16 | A | 0.796 | -1 | — | (6.0433,0.8030,-1.1889) | (5.9299,0.8524,-0.4025) | C (v23) |
| 92 | 24 | A | 0.756 | 41 | [0.46,11] | (7.1979,0.1563,-1.1066) | (7.6593,0.7553,-1.0950) | A |
| 110 | 30 | B | 0.503 | -1 | — | (12.0160,3.7452,1.7101) | (12.1469,4.1988,1.5360) | C (v8) |
| 111 | 30 | B | 0.232 | -1 | — | (12.1469,4.1988,1.5360) | (11.9350,4.1965,1.6294) | C (v7) |
| 121 | 36 | B | 0.352 | 17 | [0,49] | (5.9095,0.5013,-0.4102) | (5.9299,0.8524,-0.4025) | A |
| 122 | 38 | B | 0.325 | 4 | [0,50] | (11.9983,4.2311,1.3130) | (11.9350,4.1965,1.6294) | A |
| 124 | 40 | B | 0.908 | 5 | [0,30] | (12.0160,3.7452,1.7101) | (12.0776,4.2130,0.9348) | A |
| 125 | 41 | B | 0.799 | 18 | [0,42] | (6.0345,0.5521,-1.1976) | (5.9095,0.5013,-0.4102) | A |
| 126 | 41 | B | 1.234 | -1 | — | (7.1979,0.1563,-1.1066) | (6.0345,0.5521,-1.1976) | C (v27) |
| 127 | 1 | A | 0.792 | -1 | — | (9.4408,2.0442,-0.5241) | (9.9280,2.0012,-0.6632) | B (pair B1) |

Counts: A=6, B=3, C=4, D=0, E=0, F=0, G=0. Sum = 13 = naked total. ✔

## Loop structure (all 13 edges form exactly 3 closed naked loops)

- **Loop 1** (2 edges): e9 + e127, both on result face A1, endpoints scafv4=(9.4408,2.0442,-0.5241) / scafv5=(9.9280,2.0012,-0.6632).
- **Loop 2** (5 edges): e13(A2) + e122(B38) + e111 + e110(B30) + e124(B40): v9→v6→v7→P5→v8→v9 around scaffold junction gap v7–v8.
- **Loop 3** (6 edges): e56(A14) + e60(A16) + e121(B36) + e125(B41) + e126(B41) + e92(A24): v22→v23→v24→v26→v27→v47→v22 around junction gap v23–v27.

## Class A — SEGLOST one-sided (6 edges)

Each is a healthy section edge on one operand whose mate seg was never created on the other operand:

- e56 = seg15 (sA11/sB14), A kept (MATED e69 on A si=11), B lost — B si=14 arrangement HAD the seg15 cut (`cut a(0.683,2.806) b(0.237,1.263)`) but no SCAF-RUN for seg 15 ran: the B14 lobe adjacent to segs 15/41 produced no kept piece needing them (open chain at v23/v27 leaves the cycle unclosed in that lobe).
- e92 = seg41 (sA18/sB14), A kept PARTIAL (`[SCAF-RUN] seg=41 fa=0.4583 fb=11.0000 whole=0 MATED e128`), B lost (same B14 lobe).
- e121 = seg17 (sA13/sB15), B kept (e101 on B si=15), A lost: A si=13 `[SPLIT] cuts=2 parts=1` — face NOT split because the seg17+seg18 chain is open at v23/v27, so no A section edges were created ⇒ `[SEGLOST] A seg=17, seg=18`.
- e125 = seg18 (sA13/sB19), B kept (e141 on B si=19), A lost (same A si=13 parts=1).
- e122 = seg4 (sA1/sB16), B kept (e121 on B si=16), A lost: A si=1 `[SPLIT] cuts=2 parts=1` — open chain at v7/v8 ⇒ `[SEGLOST] A seg=4, seg=5`.
- e124 = seg5 (sA1/sB18), B kept (e133 on B si=18), A lost (same A si=1 parts=1).

Note: 5 of these 6 (all but e56's seg15 pairing) are DOWNSTREAM of the two junction gaps — the segs themselves are fine, the operand face split refused them because the chain does not close.

## Class B — divergent bridge pairs (3 edges)

**Pair B1: e9 + e127 (closed lens on face A1) vs mated edge e5.**
- Path 1 (naked): e9 (9.9280,2.0012,-0.6632)→(9.4408,2.0442,-0.5241) len 0.509, then e127 back (9.4408,2.0442,-0.5241)→(9.9280,2.0012,-0.6632) len 0.792 — a closed 2-edge loop.
- Path 2 (mate, on other side): scaffold seg 2 (sA0/sB15, v4..v5, 58-sample chain). B ran it WHOLE on si=15 (`seg=2 fa=0 fb=57 MATED e107`) → final 2-trim edge e5. A ran it PARTIAL and unmated: `[SCAF-RUN] seg=2 fa=1.5893 fb=55.6137 nCh=58 whole=0` (no MATED), right after the run's only `[JOINFAIL] pieces=6 joined=2 gapmax=6.194e-03 tol=4.419e-03` on A si=0.
- Divergence: `[NKPAIR] e=9 near2(e5 d=0.0000)` — e9 lies EXACTLY on mated e5; `[NKPAIR] e=127 near2(e5 d=0.2385)` — max divergence 0.2385. First XWELD pass shows e9 was born as a CLOSED loop v(5,5) len 0.9741 (closure-weld of the partial chain), co_refine later split it into e9+e127.
- Mechanism: trim-boundary snap shortened seg 2 on A (fa 0→1.59, fb 57→55.61), the shortened piece failed the A↔B whole-chain merge (this is 1 of the 2 unmerged common segs: 42 common, 40 merged), then closure-weld sealed it into a divergent lens while B's whole edge mated elsewhere.

**Pair B2: e13 (A face 2) vs the B chain e124+e110+e111+e122.**
- Path 1 (A, naked): e13 straight bridge v9=(12.0776,4.2130,0.9348)→v6=(11.9983,4.2311,1.3130), len 0.387, seg=-1 — invented closure that shortcuts across the whole lost seg5-[gap]-seg4 span (A si=1 was parts=1, segs 4/5 lost).
- Path 2 (B, also naked): v9→v8 (e124 rev, 0.908) → P5=(12.1469,4.1988,1.5360) (e110 rev) → v7 (e111 rev) → v6 (e122 rev, 0.325); total len 1.968.
- Max divergence between the paths ≈ 0.63 (distance from v8 to the e13 chord); NKPAIR nearest-edge figures: d(e13,e124)=0.2021, d(e13,e88)=0.3051.

## Class C — junction undershoot (4 edges)

- **Gap v7–v8 = 0.4656** (`[SCAF-VAL] v=7 val=1 nearest_v1=8 d=0.4656`): e110 (v8→(12.1469,4.1988,1.5360), 0.503) + e111 ((12.1469,4.1988,1.5360)→v7, 0.232) = the B-side invented 2-edge detour bridging the open junction via an overshoot point P5 that exists on no A face.
- **Gap v23–v27 = 0.2512** (`[SCAF-VAL] v=23/27`): e60 (A: v23=(6.0433,0.8030,-1.1889)→v24=(5.9299,0.8524,-0.4025), 0.796, seg=-1, endpoint AT valence-1 v23; B closes the same span as v27→v26→v24 via e125+e121, NKPAIR d(e60,e125)=0.3366) and e126 (B: v47=(7.1979,0.1563,-1.1066)→v27=(6.0345,0.5521,-1.1976), 1.234, seg=-1, endpoint AT valence-1 v27; A closes the same span as v47→v22→v23 via e92+e56, NKPAIR d(e126,e56)=0.5839).

## Classes D, E, F, G — empty

- D: the only 2-edge closed one-face loop (e9+e127) is root-caused by the seg-2 partial/JOINFAIL, counted in B; lengths 0.51/0.79 are not micro.
- E: no [SCAF-DROP] uv region coincides with any naked-edge seg band (drops are interior x-pave ALL-OUT verdicts far from the defect bands).
- F: no two 1-trim edges share endpoints+length+geometry (e9 does lie d=0.0000 on e5, but e5 is already 2-trim — a merge-into-existing case, not a rescuable 1-trim pair; XWELD rescue=0 is correct).
- G: none.

## Dominant mechanisms (ranked)

1. **Unbridged scaffold junction gaps (10/13 edges: 6×A + 4×C).** Two valence-1 pairs (v7/v8 gap 0.4656; v23/v27 gap 0.2512) with `bridge(march=0 weld=0 resid=0)` — the scaffold never closed them. Consequences cascade: A faces si=1 and si=13 come out `cuts=2 parts=1` (open chain cannot split a face) → [SEGLOST] A segs 4,5,17,18; the corresponding B14 lobe drops segs 15,41; both operands then invent divergent closures (e13, e60 on A; e110+e111, e126 on B). Fix leverage: close the two gaps at scaffold level (march/weld the valence-1 pairs) and all 10 edges should collapse into mated section chains.
2. **Whole-seg partial run + JOINFAIL on A si=0 (3/13 edges: pair B1).** Seg 2 trim-snap shortened to [1.5893,55.6137]/[0,57] on A, whole=0, unmated (one of 2 failed A↔B common-seg merges; `[JOINFAIL] gapmax=6.194e-03 > tol=4.419e-03`), closure-welded into the e9+e127 lens on face A1 while B's whole-chain edge (final e5) is 2-trim; e9 sits at d=0.0000 on e5, e127 diverges 0.2385.
