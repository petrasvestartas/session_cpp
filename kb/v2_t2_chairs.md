# TIER 2 — the rotated chairs through v2, measured

Trimmed multi-face STEP solids (20 faces each), 10 stored rigid motions of B, 3 ops = 30 cells,
plus the un-rotated base cell and A-op-A. Both kernels on the same operands, scored by the one
shared harness.

## HEADLINE

**v2 and the current kernel produce BIT-IDENTICAL results on all 30 rotated cells, because v2's
front end refuses every one of them and delegates to `BRep::boolean`.** Verified field by field:
36 of 36 cells (30 rotated + 3 base + 3 A-op-A) match on F / shells / naked_real / nonmanifold /
seam / degenerate / orphan / closure_residual / volume. When v2 is forced to answer with its own
pipeline (`SESSION_V2_NODELEGATE=1`) it closes **0 of 30** against the current kernel's 3 of 30,
and it fails A-op-A on all three ops.

* current kernel, default env: **3 / 30** topologically closed (naked_real 0 AND nonmanifold 0);
  **2 / 30** also within 1e-3 of the OCCT reference volume.
* v2, default env (delegating): **3 / 30** and **2 / 30** — the same three cells, the same numbers.
* v2 forced onto its own pipeline: **0 / 30** and **0 / 30**.
* current kernel with `SESSION_AUTO=1` (its escalation ladder): **6 / 30** and **2 / 30**;
  v2+AUTO identical in all 30 cells (it delegates into the same ladder).
* harness `closed()` (which additionally demands closure_residual < 1e-9): **0 / 30 for every
  kernel and every env** — and that is a property of the CORPUS, not of any kernel: the
  un-booleaned operand chair0 itself sits at residual 6.37e-06. See "the harness floor" below.

## THE GATE

The plan's kill criterion is that v2 must be >= the current kernel on every pair. On the chairs
v2 **meets it with exact equality — and only by declining to run**. Its own pipeline is strictly
worse on 30 of 30 cells and on 3 of 3 A-op-A cells. Nothing in this tier can be counted as v2
capability on real CAD geometry; what was measured is that v2's refusal gate is correct and that
its fallback is honest. If the gate is meant to certify v2 as a REPLACEMENT, this tier does not
pass it — it is not attempted.

## THE 30 CELLS — current (v1) vs v2, default env

`F/sh/naked/nonman` = faces / shells / naked_real / nonmanifold, all from `v2v::v2_verdict`.
`resid` = closure_residual. `dv` = |vol - reference| / max(1,|reference|), shown only where the
result is topologically closed (a volume off an open boundary is not a volume).

| cfg | op | ref vol (OCCT) | v1 F/sh/naked/nonman | v1 resid | v1 vol | v2 F/sh/naked/nonman | v2 resid | v2 vol | dv vs ref |
|---|---|---|---|---|---|---|---|---|---|
| z15 | cut | 80.2973 | 38/2/17/0 | 7.0e-02 | 38.4302 | 38/2/17/0 | 7.0e-02 | 38.4302 | - |
| z15 | common | 0.0000 | 27/1/6/0 | 3.8e-03 | 28.6853 | 27/1/6/0 | 3.8e-03 | 28.6853 | - |
| z15 | fuse | 80.9522 | 54/1/17/0 | 4.4e-02 | 118.7841 | 54/1/17/0 | 4.4e-02 | 118.7841 | - |
| z30 | cut | 55.9355 | 38/2/27/0 | 1.0e-02 | 61.3936 | 38/2/27/0 | 1.0e-02 | 61.3936 | - |
| z30 | common | 24.3618 | 24/1/23/0 | 2.6e-02 | 18.7250 | 24/1/23/0 | 2.6e-02 | 18.7250 | - |
| z30 | fuse | 136.2329 | 55/1/28/0 | 2.4e-02 | 147.2432 | 55/1/28/0 | 2.4e-02 | 147.2432 | - |
| z45 | cut | 59.1969 | 35/2/32/0 | 9.4e-02 | 31.2097 | 35/2/32/0 | 9.4e-02 | 31.2097 | - |
| z45 | common | 21.1006 | 24/2/28/0 | 3.2e-01 | 5.5854 | 24/2/28/0 | 3.2e-01 | 5.5854 | - |
| z45 | fuse | 139.4943 | 51/1/32/0 | 4.8e-02 | 118.2917 | 51/1/32/0 | 4.8e-02 | 118.2917 | - |
| z90 | cut | 66.9937 | 36/1/25/0 | 1.3e-01 | 28.1400 | 36/1/25/0 | 1.3e-01 | 28.1400 | - |
| z90 | common | 13.3033 | 32/3/8/10 | 2.5e-01 | 52.1984 | 32/3/8/10 | 2.5e-01 | 52.1984 | - |
| z90 | fuse | 147.2909 | 52/1/20/0 | 7.8e-02 | 108.4425 | 52/1/20/0 | 7.8e-02 | 108.4425 | - |
| x20 | cut | 80.2967 | 35/2/9/0 | 5.2e-03 | 51.5395 | 35/2/9/0 | 5.2e-03 | 51.5395 | - |
| x20 | common | 0.0005 | 21/2/13/0 | 1.2e-02 | 26.1270 | 21/2/13/0 | 1.2e-02 | 26.1270 | - |
| x20 | fuse | 160.5934 | 50/1/6/0 | 3.3e-03 | 131.8345 | 50/1/6/0 | 3.3e-03 | 131.8345 | - |
| y30 | cut | 46.9596 | 42/3/11/0 | 4.4e-03 | 48.1636 | 42/3/11/0 | 4.4e-03 | 48.1636 | - |
| y30 | common | 33.3375 | 28/2/14/0 | 8.7e-03 | 32.1504 | 28/2/14/0 | 8.7e-03 | 32.1504 | - |
| y30 | fuse | 127.2567 | 56/2/11/0 | 2.7e-03 | 128.4608 | 56/2/11/0 | 2.7e-03 | 128.4608 | - |
| z30x20 | cut | 54.2580 | 34/1/0/0 | 9.1e-07 | 54.2300 | 34/1/0/0 | 9.1e-07 | 54.2300 | 5.2e-04 |
| z30x20 | common | 26.0394 | 23/1/0/0 | 2.5e-06 | 26.0667 | 23/1/0/0 | 2.5e-06 | 26.0667 | 1.0e-03 |
| z30x20 | fuse | 134.5549 | 50/1/0/0 | 5.3e-07 | 134.5272 | 50/1/0/0 | 5.3e-07 | 134.5272 | 2.1e-04 |
| z37 | cut | 57.5580 | 37/2/30/0 | 4.7e-02 | 66.2095 | 37/2/30/0 | 4.7e-02 | 66.2095 | - |
| z37 | common | 22.7393 | 25/2/28/0 | 1.3e-01 | 13.9414 | 25/2/28/0 | 1.3e-01 | 13.9414 | - |
| z37 | fuse | 137.8552 | 54/1/30/1 | 2.8e-02 | 146.7088 | 54/1/30/1 | 2.8e-02 | 146.7088 | - |
| x13y29 | cut | 48.4734 | 38/2/16/0 | 1.4e-02 | 49.6902 | 38/2/16/0 | 1.4e-02 | 49.6902 | - |
| x13y29 | common | 31.8239 | 28/3/11/0 | 2.9e-02 | 30.5573 | 28/3/11/0 | 2.9e-02 | 30.5573 | - |
| x13y29 | fuse | 128.7708 | 54/1/16/7 | 8.2e-03 | 129.9887 | 54/1/16/7 | 8.2e-03 | 129.9887 | - |
| z63 | cut | 62.6239 | 39/2/10/0 | 1.1e-02 | 69.4352 | 39/2/10/0 | 1.1e-02 | 69.4352 | - |
| z63 | common | 17.6731 | 30/2/10/0 | 3.0e-02 | 10.8279 | 30/2/10/0 | 3.0e-02 | 10.8279 | - |
| z63 | fuse | 142.9204 | 52/1/10/0 | 6.9e-03 | 149.7683 | 52/1/10/0 | 6.9e-03 | 149.7683 | - |

The v1 and v2 columns are not merely close, they are the same bytes. The three closed cells are
z30x20 cut / common / fuse. Two of the three are within 1e-3 of the reference (cut 5.2e-04, fuse
2.1e-04); z30x20 common is 1.05e-03 out — closed, self-consistent, and 0.1% away from OCCT.

## THE SAME 30 CELLS WITH v2 FORCED TO ANSWER (`SESSION_V2_NODELEGATE=1`)

| cfg | op | ref vol | v2-forced F/sh/naked/nonman | resid | vol |
|---|---|---|---|---|---|
| z15 | cut | 80.2973 | 43/4/58/0 | 7.9e-02 | 124.5066 |
| z15 | common | 0.0000 | 22/6/32/0 | 4.5e-01 | 56.3540 |
| z15 | fuse | 80.9522 | 64/11/65/0 | 1.1e-01 | 4.7242 |
| z30 | cut | 55.9355 | 45/9/55/0 | 8.1e-02 | 109.8952 |
| z30 | common | 24.3618 | 21/6/43/0 | 5.2e-01 | 56.3886 |
| z30 | fuse | 136.2329 | 66/12/62/0 | 1.8e-01 | 7.8170 |
| z45 | cut | 59.1969 | 51/7/58/0 | 6.7e-02 | 31.9179 |
| z45 | common | 21.1006 | 27/10/47/0 | 2.3e-01 | 44.3799 |
| z45 | fuse | 139.4943 | 70/14/62/0 | 9.2e-02 | 21.6266 |
| z90 | cut | 66.9937 | 42/9/51/0 | 7.4e-02 | 50.2044 |
| z90 | common | 13.3033 | 20/11/36/0 | 3.6e-01 | 25.1193 |
| z90 | fuse | 147.2909 | 63/9/61/0 | 1.9e-01 | -7.5560 |
| x20 | cut | 80.2967 | 40/6/49/0 | 1.9e-01 | 31.4509 |
| x20 | common | 0.0005 | 18/8/32/0 | 3.8e-01 | 52.1494 |
| x20 | fuse | 160.5934 | 72/12/82/0 | 2.2e-01 | -5.1041 |
| y30 | cut | 46.9596 | 41/5/59/0 | 2.5e-01 | 127.1645 |
| y30 | common | 33.3375 | 28/6/50/0 | 3.6e-01 | 38.5016 |
| y30 | fuse | 127.2567 | 66/8/82/0 | 1.4e-01 | 75.1107 |
| z30x20 | cut | 54.2580 | 37/5/42/0 | 2.9e-01 | 4.8603 |
| z30x20 | common | 26.0394 | 14/5/30/0 | 5.8e-01 | 51.5827 |
| z30x20 | fuse | 134.5549 | 65/15/60/0 | 1.7e-01 | 49.8247 |
| z37 | cut | 57.5580 | 47/15/55/0 | 7.9e-02 | 55.4995 |
| z37 | common | 22.7393 | 18/10/38/0 | 5.6e-01 | 50.3028 |
| z37 | fuse | 137.8552 | 67/12/59/0 | 9.4e-02 | 27.4391 |
| x13y29 | cut | 48.4734 | 37/7/51/0 | 2.9e-01 | 157.6341 |
| x13y29 | common | 31.8239 | 20/8/40/0 | 4.4e-01 | 42.5309 |
| x13y29 | fuse | 128.7708 | 62/11/67/0 | 7.0e-02 | 120.7443 |
| z63 | cut | 62.6239 | 41/10/54/0 | 1.2e-01 | 28.7712 |
| z63 | common | 17.6731 | 18/5/39/0 | 3.4e-01 | 36.7812 |
| z63 | fuse | 142.9204 | 70/13/70/0 | 1.3e-01 | 82.7269 |

TOTALS default env: topologically closed  v1 3/30  v2 3/30  v2-forced 0/30
TOTALS default env: closed AND volume within 1e-3 of the reference  v1 2/30  v2 2/30  v2-forced 0/30
TOTALS default env: harness closed() (resid<1e-9)  v1 0/30  v2 0/30  v2-forced 0/30

Every forced-v2 cell is open by 30-82 naked edges; several integrate to a negative volume
(z90 fuse -7.56, x20 fuse -5.10). No cell is closer to closed than the current kernel's answer,
so there is no cell on which forcing v2 would be an improvement. Two v2 gates were tried on the
worst and the best config (base, z30x20) to check whether the forced path is merely
mis-configured: `SESSION_V2_FUZZY=1` reproduces the default forced numbers exactly, and
`SESSION_V2_SCAF=1` is worse still — it adds non-manifold edges (base cut F=105 naked=51
nonman=13 vol=893.1; base fuse F=201 naked=59 nonman=49 vol=-846.1). 12/12 of those cells exited.

## WHY v2 REFUSES — the front-end census

The refusal is not a tolerance quibble; the v2 face split does not survive contact with this
geometry. `SESSION_V2_SFDBG=1`, cut op, one run per config (the split stage is op-independent),
12/12 exited. Each operand has 20 faces; "images" is how many face images the split produced,
"vol integrated" is the split operand's own volume, "vol wanted" the operand's volume. These two
columns are v2's OWN internal numbers, printed by its self-check at the harness DEFAULT budget
(hence 83.154 for chair0 rather than the 80.294 the raised budget gives); every other number in
this report uses the raised budget.

| cfg | A images (of 20 faces) | A naked | A vol integrated | A vol wanted | B images | B naked | B vol integrated | B vol wanted | EdgeNotConsumed |
|---|---|---|---|---|---|---|---|---|---|
| z15 | 17 | 18 | 4.151 | 83.154 | 1 | 11 | 9.320 | 81.973 | 237 |
| z30 | 18 | 20 | 6.343 | 83.154 | 2 | 12 | 6.323 | 80.813 | 231 |
| z45 | 20 | 22 | 9.570 | 83.154 | 2 | 21 | 21.418 | 79.697 | 231 |
| z90 | 18 | 46 | 65.825 | 83.154 | 2 | 24 | 55.432 | 77.324 | 250 |
| x20 | 17 | 28 | 64.785 | 83.154 | 1 | 13 | 21.445 | 82.964 | 210 |
| y30 | 17 | 42 | 60.194 | 83.154 | 1 | 15 | 29.214 | 83.237 | 262 |
| z30x20 | 19 | 28 | 65.662 | 83.154 | 1 | 10 | 4.726 | 80.881 | 200 |
| z37 | 17 | 23 | 6.923 | 83.154 | 3 | 16 | 15.399 | 80.282 | 227 |
| x13y29 | 18 | 32 | 63.442 | 83.154 | 2 | 14 | 22.596 | 83.177 | 228 |
| z63 | 18 | 36 | 64.416 | 83.154 | 2 | 24 | 37.777 | 78.521 | 262 |
| base | 17 | 28 | 65.825 | 83.154 | 17 | 36 | 31.611 | 83.098 | 116 |
| self | 147 | 309 | -757.591 | 83.154 | 154 | 282 | -878.966 | 83.154 | 1032 |

The rotated operand B collapses to 1-3 face images out of 20 in nine of the ten rotated configs,
and A loses 1-3 faces and comes back with 18-46 naked edges everywhere. Each cell raises 200-262
`EdgeNotConsumed` alerts — section blocks the wire walk could not place. The acceptance gate of
`v2sol_run_front` (`n_faces_lost == 0 && n_unused == 0 && split_faithful`) fires for the right
reason,
and the delegation is the honest outcome rather than a missed opportunity.

One structural point worth recording independently: `split_faithful` compares the split operands'
volumes against `v2v::v2_verdict(A).volume` **at the harness's default budget**, and demands
`closed()` on them. On this corpus the un-split operand fails both (see the harness floor below),
so `split_faithful` is UNREACHABLE on any imported trimmed solid no matter how good the split is.
That is a second, independent reason v2 can never accept a chair — but it is not the binding one
today, because the split is nowhere near.

## THE HARNESS FLOOR — this corpus cannot reach `closed()`, and that is the input's fault

Measured on the un-booleaned operand, before any boolean runs:

| operand / budget | closure_residual | volume | converged | naked_real |
|---|---|---|---|---|
| chair0, harness default budget (4e6 evals / 4e4 per face) | 9.117e-03 | 83.153899 | 0 | 0 |
| chair0, budget x100 (4e8 / 4e6) | 6.37e-06 | 80.294114 | 1 | 0 |
| OCCT reference vol(A) | - | 80.296862 | - | - |
| `BRep::volume()` (kernel's own tessellation) | - | 80.301057 | - | - |

Two separate facts fall out of that, and both were verified rather than assumed:

1. **The default budget is not enough for this corpus.** One face (index 16, area 31.32) hits its
   per-face cap; the operand then integrates 3.6% high (83.15 vs 80.30) and reports
   `converged=0`. Scoring a boolean result for volume correctness with a quadrature that is
   itself 3.6% wrong measures the quadrature. Every number in this report therefore uses the
   harness with the budget raised x100 — `v2v::v2_verdict(b, opt)` is a documented entry point of
   the shared harness and no topology rule is touched. At that budget the operand's volume is
   3.4e-05 relative to the OCCT truth, i.e. more accurate than `BRep::volume()`'s 5.2e-05.
2. **The residual floor is the chairs' own trim data, not the budget and not STEP.** At x100 the
   operand still sits at 6.37e-06, far above the 1e-9 gate, with `max_chain_gap = 1.298e-04`:
   the imported trim loops do not chain in UV, and Green's theorem is being applied to a boundary
   that does not quite close. A control settles the attribution — box, sphere and cylinder written
   to STEP and read back score residual 0.0e+00 / 5.95e-17 / 5.45e-18 with `max_chain_gap` 0 and
   exact volumes, i.e. the STEP round trip is clean and the defect is specific to the chair files.

Consequence: `V2Verdict::closed()` is 0/30 for every kernel in every configuration here, and that
number carries no information about either kernel. The comparative criterion used throughout this
report is therefore stated explicitly: **topologically closed = `naked_real == 0 && nonmanifold ==
0` from the harness** (the same criterion under which the historical "3 of 30" was recorded),
**plus** the harness volume against the OCCT reference. Both come from the shared harness; the
only thing dropped is the residual gate the input itself cannot pass.

## THE ORACLE-FREE CHECK — partition identity, cut + common == vol(A)

| cfg | v1/v2 cut | v1/v2 common | cut+common | vol(A) | rel |
|---|---|---|---|---|---|
| z15 | open | open | - | 80.2969 | not both closed |
| z30 | open | open | - | 80.2969 | not both closed |
| z45 | open | open | - | 80.2969 | not both closed |
| z90 | open | open | - | 80.2969 | not both closed |
| x20 | open | open | - | 80.2969 | not both closed |
| y30 | open | open | - | 80.2969 | not both closed |
| z30x20 | 54.2300 | 26.0667 | 80.2967 | 80.2969 | 1.5e-06 |
| z37 | open | open | - | 80.2969 | not both closed |
| x13y29 | open | open | - | 80.2969 | not both closed |
| z63 | open | open | - | 80.2969 | not both closed |

Only z30x20 has both a closed cut and a closed common, and there the pair is self-consistent to
2.0e-06 — better than its 5.2e-04 / 1.05e-03 agreement with OCCT. The two kernels' answers are
internally coherent and sit about 0.028 (3.4e-04 of vol(A)) away from OCCT's placement of the
same boundary; OCCT's own pair sums to 80.2974 (rel 6.8e-06). So this cell is a genuine 0.03
geometric disagreement, not an open shell dressed up as a volume.

The base (un-rotated) cell is the control and it is exact under all three envs (identical bytes
in default, `SESSION_SD=1` and `SESSION_AUTO=1`):

| base op | v1 = v2 | reference | rel |
|---|---|---|---|
| cut | 35 faces, 1 shell, naked 0, resid 7.83e-06, vol 46.794251 | 46.794114 | 2.93e-06 |
| common | 25 faces, 1 shell, naked 0, resid 1.55e-05, vol 33.502534 | 33.503027 | 1.47e-05 |
| fuse | 50 faces, 1 shell, naked 0, resid 4.77e-06, vol 127.091282 | 127.091467 | 1.46e-06 |
| partition | 46.794251 + 33.502534 = 80.296785 vs vol(A) 80.296862 | - | 9.6e-07 |

The control cell is bit-identical in all four sweeps that included it — default env,
`SESSION_SD=1`, `SESSION_AUTO=1`, and the post-rebuild control re-run — so when two sweeps differ
elsewhere, the difference is the kernel and not the harness.

## THE CURRENT KERNEL'S ESCALATION LADDER (`SESSION_AUTO=1`) — and v2 inside it

| cfg | op | ref vol | v1+AUTO F/sh/naked/nonman | resid | vol | v2+AUTO identical? |
|---|---|---|---|---|---|---|
| z15 | cut | 80.2973 | 38/2/17/0 | 7.0e-02 | 38.4302 | yes |
| z15 | common | 0.0000 | 27/1/6/0 | 3.8e-03 | 28.6853 | yes |
| z15 | fuse | 80.9522 | 54/1/17/0 | 4.4e-02 | 118.7841 | yes |
| z30 | cut | 55.9355 | 41/2/13/0 | 2.5e-04 | 55.5485 | yes |
| z30 | common | 24.3618 | 28/1/10/0 | 1.3e-02 | 21.4415 | yes |
| z30 | fuse | 136.2329 | 56/1/13/0 | 1.2e-04 | 136.0340 | yes |
| z45 | cut | 59.1969 | 37/2/20/0 | 2.7e-02 | 62.4922 | yes |
| z45 | common | 21.1006 | 26/1/16/0 | 1.4e-01 | 12.7608 | yes |
| z45 | fuse | 139.4943 | 52/1/17/0 | 1.9e-02 | 149.5750 | yes |
| z90 | cut | 66.9937 | 36/1/25/0 | 1.3e-01 | 28.1400 | yes |
| z90 | common | 13.3033 | 32/3/8/10 | 2.5e-01 | 52.1984 | yes |
| z90 | fuse | 147.2909 | 52/1/20/0 | 7.8e-02 | 108.4425 | yes |
| x20 | cut | 80.2967 | 34/1/7/0 | 5.2e-03 | 51.4889 | yes |
| x20 | common | 0.0005 | 20/1/10/0 | 1.2e-02 | 26.1773 | yes |
| x20 | fuse | 160.5934 | 50/1/6/0 | 3.3e-03 | 131.8345 | yes |
| y30 | cut | 46.9596 | 43/3/0/0 | 3.2e-04 | 47.6485 | yes |
| y30 | common | 33.3375 | 31/2/3/0 | 9.6e-04 | 32.6654 | yes |
| y30 | fuse | 127.2567 | 57/2/0/0 | 2.0e-04 | 127.9457 | yes |
| z30x20 | cut | 54.2580 | 34/1/0/0 | 9.1e-07 | 54.2300 | yes |
| z30x20 | common | 26.0394 | 23/1/0/0 | 2.5e-06 | 26.0667 | yes |
| z30x20 | fuse | 134.5549 | 50/1/0/0 | 5.3e-07 | 134.5272 | yes |
| z37 | cut | 57.5580 | 39/2/27/0 | 4.8e-02 | 67.7519 | yes |
| z37 | common | 22.7393 | 22/2/25/0 | 1.9e-01 | 10.3685 | yes |
| z37 | fuse | 137.8552 | 56/1/21/2 | 2.9e-03 | 139.4121 | yes |
| x13y29 | cut | 48.4734 | 40/2/3/0 | 3.4e-05 | 49.8638 | yes |
| x13y29 | common | 31.8239 | 32/3/0/0 | 3.5e-04 | 30.3844 | yes |
| x13y29 | fuse | 128.7708 | 54/1/3/7 | 1.9e-05 | 130.1623 | yes |
| z63 | cut | 62.6239 | 39/2/10/0 | 1.1e-02 | 69.4352 | yes |
| z63 | common | 17.6731 | 30/2/10/0 | 3.0e-02 | 10.8279 | yes |
| z63 | fuse | 142.9204 | 52/1/10/0 | 6.9e-03 | 149.7683 | yes |

SESSION_AUTO=1: cells with a result 30/30, topologically closed 6/30, closed and within 1e-3 of the reference 2/30

AUTO doubles the closed count (3 -> 6: it adds y30 cut, y30 fuse, x13y29 common) but adds no
correct-volume cell: the new closures are 1.4e-02 (y30 cut 47.6485 vs 46.9596), 5.4e-03 (y30 fuse
127.9457 vs 127.2567) and 4.5e-02 (x13y29 common 30.3844 vs 31.8239) away from the reference.
Two of those are exactly the cells the project has recorded a reference caveat on (y30, x13y29:
our solid decomposition independently verified correct where the reference omits a lump of A\B),
and the direction is consistent with that caveat — our cut is LARGER than the reference on both
(y30 +0.69, x13y29 cut +1.39). I did not re-derive the reference, so I record the direction and
leave the cell unclaimed.

`SESSION_SD=1` (the same-domain route) was also swept over all 30 cells: 3/30 closed, 2/30
correct — identical to default on every cell, as expected since none of the rotated pairs has a
coincident face. It matters only for A-op-A, below.

## A-op-A — chair0 against an independently re-read copy of chair0

`cut` must be EMPTY, `common` must be A, `fuse` must be A. Expected volume is A's own
raised-budget volume, 80.294113.

| run | op | kernel | F | shells | naked | nonman | resid | vol | expected |
|---|---|---|---|---|---|---|---|---|---|
| default env | cut | v1 | 66 | 64 | 24 | 0 | 8.8e-01 | 123.1834 | EMPTY |
| default env | cut | v2 | 66 | 64 | 24 | 0 | 8.8e-01 | 123.1834 | EMPTY |
| default env | cut | v2n | 60 | 37 | 52 | 0 | 4.7e-01 | 369.4392 | EMPTY |
| default env | common | v1 | 205 | 177 | 88 | 1 | 1.3e-01 | 2367.9275 | = A (F=20, vol 80.2941) |
| default env | common | v2 | 205 | 177 | 88 | 1 | 1.3e-01 | 2367.9275 | = A (F=20, vol 80.2941) |
| default env | common | v2n | 10 | 4 | 26 | 0 | 5.7e-01 | 93.9074 | = A (F=20, vol 80.2941) |
| default env | fuse | v1 | 242 | 217 | 91 | 0 | 1.1e-01 | 2121.4583 | = A (F=20, vol 80.2941) |
| default env | fuse | v2 | 242 | 217 | 91 | 0 | 1.1e-01 | 2121.4583 | = A (F=20, vol 80.2941) |
| default env | fuse | v2n | 122 | 36 | 68 | 1 | 2.3e-01 | -46.8577 | = A (F=20, vol 80.2941) |
| SESSION_SD=1 | cut | v1 | 0 | 0 | 0 | 0 | - | EMPTY | EMPTY |
| SESSION_SD=1 | common | v1 | 20 | 1 | 0 | 0 | 6.4e-06 | 80.2941 | = A (F=20, vol 80.2941) |
| SESSION_SD=1 | fuse | v1 | 20 | 1 | 0 | 0 | 6.4e-06 | 80.2941 | = A (F=20, vol 80.2941) |
| SESSION_SD=1 | cut | v2 | 0 | 0 | 0 | 0 | - | EMPTY | EMPTY |
| SESSION_SD=1 | cut | v2n | 60 | 37 | 52 | 0 | 4.7e-01 | 369.4392 | EMPTY |
| SESSION_SD=1 | common | v2 | 20 | 1 | 0 | 0 | 6.4e-06 | 80.2941 | = A (F=20, vol 80.2941) |
| SESSION_SD=1 | common | v2n | 10 | 4 | 26 | 0 | 5.7e-01 | 93.9074 | = A (F=20, vol 80.2941) |
| SESSION_SD=1 | fuse | v2 | 20 | 1 | 0 | 0 | 6.4e-06 | 80.2941 | = A (F=20, vol 80.2941) |
| SESSION_SD=1 | fuse | v2n | 122 | 36 | 68 | 1 | 2.3e-01 | -46.8577 | = A (F=20, vol 80.2941) |

Reading it:

* **Default env, both kernels FAIL A-op-A**: cut yields 66 faces / 64 shells / 24 naked instead of
  nothing; common yields 205 faces / 177 shells; fuse 242 faces / 217 shells. So the "three
  mechanisms" that make A-op-A pass are not on by default.
* **`SESSION_SD=1`, current kernel PASSES all three**: cut EMPTY, common = 20 faces / 1 shell /
  naked 0 / vol 80.294113, fuse identical — both equal to A to every digit the harness prints
  (A's own raised-budget volume is 80.294113 and its residual 6.366e-06; common and fuse report
  exactly 80.294113 and 6.366e-06). That is A returned unchanged, not a rebuilt approximation.
* **v2 with delegation PASSES too**, by handing the pair to that same route: identical bytes.
* **v2 forced onto its own pipeline FAILS all three under the same `SESSION_SD=1`**: cut returns
  60 faces / 52 naked (should be empty), common 10 faces / 26 naked, fuse 122 faces / 68 naked /
  1 non-manifold / volume -46.86. Its front end refuses A-op-A as well, and for the most extreme
  reason in the census above — splitting chair0 against its own copy explodes 20 faces into 147
  images with 309 naked edges and 1032 unconsumed section edges.

## METHOD AND PROVENANCE

* Driver: `session_cpp/main_21.cpp` (new, mine). One cell per process — `SESSION_T2_CFG`,
  `SESSION_T2_OP`, `SESSION_T2_KERNEL` — so a hang or a crash costs one cell, and every run
  records its own exit status. It prints one `T2CELL` line of harness fields and a `T2DONE`
  sentinel; a log without `T2DONE` is discarded rather than parsed.
* Operands: the STORED files, never a re-derived rotation.
  `serialization/boolean_steps/chairs/chair0.stp` (md5 5832e5471e2c761af3960d0d2efc011f),
  `chair1.stp` (md5 fd3ff5d2483f73f566bd7175a8decb66) and the ten `chairs/rot/B_<cfg>.step`.
  A-op-A re-reads chair0.stp a second time into an independent BRep.
* Scoring: `session_cpp::v2v::v2_verdict` from `src/v2/v2_verdict.h` ONLY, with
  `MassPropsOptions` at 100x the harness default budget for the reason measured above. No
  verdict code was written in this driver. `build_t2/main_17` (the harness's own validation gate)
  passes 5/5 in this build dir both before and after the concurrent rebuild.
* Reference: `validation/occt_truth.json` / `OCCT_TRUTH.md`, `volA` = 80.296861869.
* Kernels: `v1` = `BRep::boolean` (`boolean_difference` / `_intersection` / `_union`);
  `v2` = `v2sol::v2_cut` / `v2_common` / `v2_fuse`; `v2n` = the same with
  `SESSION_V2_NODELEGATE=1`.
* Build: my own `build_t2`, configured from the unmodified project `CMakeLists.txt`
  (`-DFETCHCONTENT_SOURCE_DIR_PROTOBUF/ABSL` pointed at an existing checkout so nothing is
  downloaded). `main_21` is NOT in the project's `foreach(MAIN_ID ...)` list and I did not add it:
  it is compiled with the exact flags CMake generated for `main_17` and linked with `main_17`'s
  own `link.txt` with the object swapped. Nothing outside `main_21.cpp` was edited.
* Binary that produced every number above: sha1 `102bed03b811b36ec6e90464ccb4dc601f8eaef9`
  (kept at `scratchpad/t2_main_21_measured`). Repo HEAD `5bb685a`, with the working-tree
  modifications of the concurrent v1 session in `src/brep.cpp`, `src/brep_section.cpp`,
  `src/intersection.cpp`, `src/nurbssurface_trimmed.cpp`, `src/file_step.cpp`.
* CONCURRENCY CHECK. `src/brep.cpp` was modified by the v1 session at 15:00, after my 13:56 link.
  I rebuilt (sha1 `52fb8ee19983930743aa01946e5210291a60bee2`) and re-ran the six control cells
  (base and z30x20, cut/common/fuse, v1 and v2): 12/12 bit-identical to the originals. The
  comparison stands on the newer tree for those cells; the other 24 were not re-run.
* Cells attempted vs completed, all with exit status 0 and a `T2DONE`:
  default sweep 108/108 (12 configs x 3 ops x 3 kernels), `SESSION_AUTO` sweep 72/72,
  `SESSION_SD` sweep 36/36, A-op-A under SD 3/3 + 6/6, front-end census 12/12,
  v2-gate probes 6/6 + 6/6, control re-run 12/12. Nothing timed out (per-cell limit 2400 s;
  slowest cell 746 s, y30 fuse under AUTO).
* Wall time is not comparable between kernels here: a v2 cell pays for the front-end attempt AND
  the delegated v1 boolean (base cut: v1 1.5-3.2 s, v2 13.6-20.9 s, same result).

## WHAT I CHANGED

`session_cpp/main_21.cpp` — new file, the only file written. No kernel source, no CMakeLists, no
kb file other than this one. Nothing committed or pushed.

## WHAT THIS DOES NOT SHOW

* No number here says v2's own boolean can or cannot do the chairs *in principle*; it says that
  today its face split loses 17-19 of B's 20 faces on this input and that its refusal gate catches
  that. Fixing it is a `src/v2/brep_v2_splitface.cpp` job and that file is not mine.
* The 1e-3 volume threshold for "correct" is a choice; the underlying `dv` is printed in every row
  so any other threshold can be applied to the same data. The OCCT truth table's own identity
  tolerance is 0.4015 (0.5%), which would call 3/30 correct for v1 and v2 and still 0/30 for
  forced v2.
* z15 is the recorded self-inconsistent reference cell (its cut implies a common of -0.0004 while
  its fuse implies 79.64). No kernel closes any z15 cell here, so nothing was gated on it; the
  partition identity is reported instead and is unavailable there for the same reason.
* `SESSION_AUTO=1` combined with `SESSION_SD=1` was not swept.
