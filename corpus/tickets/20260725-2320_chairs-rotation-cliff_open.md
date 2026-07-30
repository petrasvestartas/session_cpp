# Chairs cut: closure dies between 3 and 6 degrees of rotation — CLIFF, then RAMP

| field | value |
|---|---|
| battery | t0r sweep (`rotprim.py sweepchairs`) |
| operands | `corpus/work/chairs/{chair0.stp, chair1.stp}` (freeform NURBS — the STEP path that WORKS) |
| rotation | B about z through the joint centroid (8.0578, 3.2309, 0.1372), the same centre the kernel's own rotated battery uses |
| op | cut |
| kernel | `corpus/work/main_7_snapshot` sha1 3de30de13d70 |
| oracle | not needed — continuity and closure are oracle-free |

## Measured curve

| angle | our cut vol | d(vol) | solids | naked | non-manifold | open shells | closed |
|---|---|---|---|---|---|---|---|
| 0.0 | 46.7931 | — | 1 | 0 | 0 | 0 | **yes** |
| 3.0 | 47.2134 | +0.42 | 1 | 0 | 0 | 0 | **yes** |
| 6.0 | 47.9045 | +0.69 | 0 | 4 | 0 | 1 | no |
| 9.0 | 48.5923 | +0.69 | 0 | 7 | 0 | 1 | no |
| 12.0 | 52.5434 | **+3.95** | 0 | 17 | 0 | 1 | no |
| 15.0 | 51.0917 | −1.45 | 0 | 26 | 0 | 1 | no |
| 18.0 | 48.7166 | −2.38 | 0 | 41 | 0 | 1 | no |
| 21.0 | 38.6312 | **−10.09** | 0 | 30 | 0 | 1 | no |
| 24.0 | 46.8061 | **+8.17** | 0 | 20 | 0 | 1 | no |
| 27.0 | 46.8752 | +0.07 | 0 | 16 | 0 | 1 | no |

The true cut volume is smooth and monotone over this range (OCCT: 46.79 at 0 deg rising
to ~55.9 at 30 deg), so a step of ~0.7 per 3 deg is what continuity allows.

## Answer to the cliff-vs-ramp question: BOTH, in that order

1. **CLIFF in topology between 3.0 and 6.0 deg.** At 3 deg the result is a closed valid
   solid with ZERO naked edges; at 6 deg it has 4 naked edges and an open shell. Nothing
   degrades gradually across that step — closure is simply lost. A rotation of under 6
   degrees is far too small for accumulated numerical error to explain, so this points at
   a predicate with an angular tolerance, not at drift.
2. **RAMP in severity afterwards**: naked edges grow 4 -> 7 -> 17 -> 26 -> 41 as the angle
   increases, i.e. once the predicate stops firing, more and more of the section is lost.
3. **Volume DISCONTINUITIES on top**: +3.95 (9->12 deg), −10.09 (18->21 deg), +8.17
   (21->24 deg) — each far outside what the true curve can do in 3 degrees.

## REFINEMENT: the cliff is between 3.5 and 4.0 degrees

0.5-deg steps across the transition (`corpus/t0r/ledger/sweep_chairs_20260725-232013.json`):

| angle | our cut vol | faces | naked | closed |
|---|---|---|---|---|
| 3.0 | 47.2134 | — | 0 | **yes** |
| 3.5 | 47.3011 | 36 | 0 | **yes** |
| **4.0** | 47.5070 | 35 | **7** | **no** |
| 4.5 | 47.5940 | 36 | 4 | no |
| 5.0 | 47.6931 | 36 | 4 | no |
| 5.5 | 47.7993 | 35 | 4 | no |
| 6.0 | 47.9045 | — | 4 | no |

Two things to notice, and the second is the important one:

1. **Closure snaps between 3.5 and 4.0 deg** — naked 0 -> 7 in half a degree, on the same
   pair of solids.
2. **The VOLUME is perfectly continuous across that snap**: 47.30, 47.51, 47.59, 47.69,
   47.80, 47.90 — smooth increments of ~0.1-0.2 with no jump at all. So nothing is
   drifting numerically at the transition; a TOPOLOGICAL decision flips while the
   geometry stays put.

That combination (continuous volume, discontinuous topology, at a fixed small angle) is
the signature of a predicate with an angular tolerance — not accumulated error, and not a
tolerance on distance. **Look for a comparison against an angle threshold of roughly
3.5-4.0 degrees (~0.065 rad, or a cosine near 0.998) in the section/classification path.**
The later volume jumps (+3.95 at 9->12 deg, -10.09 at 18->21 deg) are downstream
consequences once enough section pieces are dropped.

## Repro (cwd session_cpp) — one angle

```bash
python corpus/rotprim.py sweepchairs --start 6 --end 6 --step 1 --ops cut --no-oracle
# or by hand:
python corpus/step_rigid.py corpus/work/chairs/chair1.stp /tmp/B6.step \
       --axis 0 0 1 --deg 6 --trans <centre-compensating translation>
```
The sweep command writes each angle's operands and result into
`corpus/t0r/cells/chairsweep_<angle>/`, so any angle can be re-measured with

```bash
../validation/step_probe/build/step_probe corpus/t0r/cells/chairsweep_006.000/chair0_cut_chair1.step -n
```

## Why this is the highest-value diagnostic in the tier

Fixed placements (the z15/z30/z45 battery) cannot tell a cliff from a ramp — they only
sample four widely separated angles, all already past the threshold. The sweep shows the
kernel is exact at 3 deg and broken at 6 deg on the SAME geometry, which turns "rotated
chairs fail" into "find the predicate whose angular tolerance sits between 3 and 6
degrees".
