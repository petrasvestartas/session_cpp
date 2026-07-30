# Primitive boolean matrix — independent re-verification (FreeCAD/OCCT reader)

Audit of the "45/45 cells OK" claim with metrics that can see what `is_solid()` cannot:
shell fragmentation, open boundaries, impossible volumes. Every number below was measured
in this session; nothing is carried over from the matrix's own scorecard except where the
row is explicitly labelled `our_mem_*` (that column IS the matrix's own scorecard).

## 1. VERDICT

**All 45 primitive cells are genuinely closed, valid, correctly-fragmented solids under
the independent reader. Zero fragmented results, zero open shells, zero naked edges, zero
wrong solid counts. The failure mode found in the rotated chairs (reported "solid 1 naked 0",
actually open shells with 9-58 naked edges) does NOT exist anywhere in the primitive matrix.**

That is the reassuring half. The other half:

**22 of the 45 shipped STEP results do NOT reproduce the OCCT reference volume to the
matrix's own 1e-6 tolerance when re-measured from the file — worst 4.35e-3 (box ∩ tor),
i.e. 4350x the gate threshold.** The matrix does not see this because it gates on the
in-memory `BRep::volume()` (which integrates the exact pcurves) while the geometry we write
out and hand to any other kernel is a different, coarser shape. All 45 in-memory volumes are
correct (max rel 8.52e-7); 23 of 45 exported volumes are.

| what was checked | result |
|---|---|
| cells produced / verified | 45 / 45 |
| solids == OCCT reference solids | 45 / 45 |
| faces == OCCT reference faces | 45 / 45 |
| every shell closed (`Shell.isClosed()`) | 45 / 45 |
| naked edges (seam/pole-corrected metric) | 0 in all 45 |
| OCCT `isValid()` | 45 / 45 YES |
| in-memory volume vs OCCT oracle < 1e-6 | 45 / 45 (max 8.52e-7) |
| **shipped-STEP volume vs OCCT oracle < 1e-6** | **23 / 45** |
| shipped-STEP volume < 1e-4 | 32 / 45 |
| shipped-STEP volume < 1e-2 | 45 / 45 |

Scope note: the run was `SESSION_NO_ROT=1`, which skips the 6 oriented (`*R`) pairs, so the
grid is 15 pairs x 3 ops = **45 cells, not 60**. `SESSION_XOR_CHECK` was not set, so the
xor/split identities were not exercised and are NOT covered by this audit.

## 2. How it was produced

```
cp session_cpp/build/main_7 session_cpp/build/main_7_corpus          # private, untouched binary
cd session_cpp
SESSION_NO_ROT=1 SESSION_STEP_DIR=/home/petras/fc_inspect/matrix_out \
SESSION_FREEFORM=1 timeout 2400 ./build/main_7_corpus                # exit 0, ~34 min, 8.4 MB
cd /home/petras/fc_inspect
FC_IN=... FC_OUT=... /snap/bin/freecad.cmd matrix_verify2.py         # FreeCAD 1.1.1 / OCCT
```

The binary's own last line was `45/46 cells OK` — 45 primitive cells OK plus the separate
SESSION_FREEFORM probe cell, which it correctly reported FAIL. Nothing in the repo was
written except this file and the gitignored `build/main_7_corpus` copy.

Artifacts: `/home/petras/fc_inspect/matrix_out/*.step` (45 matrix + 5 freeform),
`/home/petras/fc_inspect/matrix_report2/{FINAL_TABLE.txt,matrix_verify2.json,fidelity.json}`,
`/home/petras/fc_inspect/matrix_report2/REF_<cell>.step` (45 OCCT reference results, openable
next to ours), scripts `matrix_verify2.py`, `probe_fidelity.py`, `probe_metrics.py`,
`probe_refacc.py`, `probe_freeform.py`.

### 2a. Two reader bugs that had to be fixed first

**(i) The naked-edge test in `validation`-style inspectors is invalid on quadrics.**
`len(shape.ancestorsOfType(edge, Face)) == 1` flags SEAM edges (one face, two wire
occurrences) and POLE edges (degenerate). Measured on *pristine* OCCT primitives:

| pristine solid | naive naked | corrected naked | shell closed |
|---|---:|---:|:---:|
| box | 0 | 0 | yes |
| cylinder | 1 | 0 | yes |
| cone | 2 | 0 | yes |
| torus | 2 | 0 | yes |
| sphere | 3 | 0 | yes |
| box shell minus 1 face (control, open) | 4 | 4 | no |
| cylinder shell minus 1 cap (control, open) | 2 | 1 | no |

Corrected metric: an edge is free iff its total occurrence count across all face wires is
< 2, degenerate edges excluded. Across the 45 matrix cells the naive metric reports **126
"naked" edges in 37 cells**; the corrected metric reports **0 in all 45**. (This does not
retro-invalidate the chairs finding — those shapes also failed `Shell.isClosed()`, which is
seam-safe. It does mean the naive count must not be quoted for quadric results.)

**(ii) A multi-solid result occupies more than one top-level STEP shape.** The result is
`shapes[2:]`, not `shapes[2]`. Four cells write 4 root shapes: `cyl_cut_cyl2`,
`sph_cut_cone`, `tor_cut_tor2`, `tor_common_tor2`. Reading only `shapes[2]` makes them look
like half-volume single solids; all four are in fact correct 2-solid results and the OCCT
reference is also 2 solids.

### 2b. Reference used

Two independent OCCT references were computed and cross-checked:

* **oracle cache** `validation/occt_cache.txt` — self-consistent to **<= 4.3e-12** on
  `vol(A-B)+vol(A&B)==vol(A)` for all 15 pairs. Used as the volume reference.
* **FreeCAD-rebuilt** (`Part.makeBox/Sphere/Cylinder/Cone/Torus` + the same placement table)
  — agrees with the cache to <= 1e-8 on 14 pairs, but is itself self-inconsistent at
  **2.3e-5** on tor x tor2, so it is used only for topology (solids/shells/faces).

Both operands read back out of our STEP files with volume error <= 8e-16 versus closed form,
so operand export is exact and every discrepancy below is in the RESULT.

## 3. Per-cell table

`nkd` = corrected naked edges, `nkNv` = the naive (seam-blind) count for comparison,
`step_rel` = |shipped STEP volume - oracle| / oracle, `mem_rel` = the matrix's own in-memory
figure, `Etol` = max OCCT edge tolerance in the result, `poly` = largest degree-1 polyline
pole count among the result's edges (2 = an honest straight line, 0 = none).
PASS = naked 0 AND all shells closed AND solids == ref AND faces == ref AND step_rel < 1e-6.

| pair | op | F | sld | shl(open) | nkd | nkNv | valid | our STEP vol | our mem vol | OCCT ref vol | refF | refS | step_rel | mem_rel | Etol | poly | verdict |
|---|---|---:|---:|---:|---:|---:|:---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| box x box | common | 6 | 1 | 1(0) | 0 | 0 | Y | 4.000000 | 4.0000 | 4.000000000 | 6 | 1 | 4.4e-16 | 1.8e-15 | 1.0e-07 | 2 | PASS |
| box x box | cut | 11 | 1 | 1(0) | 0 | 0 | Y | 60.000000 | 60.0000 | 60.000000000 | 11 | 1 | 2.4e-16 | 2.3e-15 | 1.0e-07 | 2 | PASS |
| box x box | fuse | 11 | 1 | 1(0) | 0 | 0 | Y | 68.000000 | 68.0000 | 68.000000000 | 11 | 1 | 2.1e-16 | 1.7e-15 | 1.0e-07 | 2 | PASS |
| box x cone | common | 2 | 1 | 1(0) | 0 | 2 | Y | 16.756703 | 16.7552 | 16.755160819 | 2 | 1 | 9.2e-05 | 4.7e-15 | 1.4e-04 | 129 | FAIL |
| box x cone | cut | 10 | 1 | 1(0) | 0 | 2 | Y | 47.244839 | 47.2448 | 47.244839181 | 10 | 1 | 1.1e-11 | 1.7e-15 | 1.0e-07 | 2 | PASS |
| box x cone | fuse | 10 | 1 | 1(0) | 0 | 0 | Y | 64.000000 | 64.0000 | 64.000000000 | 10 | 1 | 8.1e-12 | 2.6e-15 | 1.0e-07 | 2 | PASS |
| box x cyl | common | 3 | 1 | 1(0) | 0 | 1 | Y | 28.276819 | 28.2743 | 28.274333882 | 3 | 1 | 8.8e-05 | 9.3e-15 | 1.2e-04 | 129 | FAIL |
| box x cyl | cut | 7 | 1 | 1(0) | 0 | 1 | Y | 35.725674 | 35.7257 | 35.725666118 | 7 | 1 | 2.3e-07 | 1.1e-14 | 4.9e-06 | 2 | PASS |
| box x cyl | fuse | 10 | 1 | 1(0) | 0 | 2 | Y | 78.137175 | 78.1372 | 78.137166941 | 10 | 1 | 1.0e-07 | 1.3e-15 | 4.9e-06 | 2 | PASS |
| box x sph | common | 7 | 1 | 1(0) | 0 | 2 | Y | 54.484208 | 54.4543 | 54.454272666 | 7 | 1 | 5.5e-04 | 1.5e-07 | 1.0e-03 | 33 | FAIL |
| box x sph | cut | 7 | 1 | 1(0) | 0 | 2 | Y | 9.514562 | 9.5457 | 9.545727334 | 7 | 1 | 3.3e-03 | 8.5e-07 | 1.0e-03 | 33 | FAIL |
| box x sph | fuse | 13 | 1 | 1(0) | 0 | 4 | Y | 74.985943 | 74.9956 | 74.995574284 | 13 | 1 | 1.3e-04 | 1.2e-07 | 1.0e-03 | 33 | FAIL |
| box x tor | common | 5 | 1 | 1(0) | 0 | 5 | Y | 15.548371 | 15.4811 | 15.481080126 | 5 | 1 | 4.4e-03 | 8.1e-07 | 5.6e-04 | 77 | FAIL |
| box x tor | cut | 7 | 1 | 1(0) | 0 | 5 | Y | 48.454334 | 48.5189 | 48.518919874 | 7 | 1 | 1.3e-03 | 2.6e-07 | 5.6e-04 | 77 | FAIL |
| box x tor | fuse | 16 | 1 | 1(0) | 0 | 0 | Y | 73.717436 | 73.7851 | 73.785107140 | 16 | 1 | 9.2e-04 | 1.4e-07 | 5.6e-04 | 77 | FAIL |
| cone x cone | common | 2 | 1 | 1(0) | 0 | 4 | Y | 4.188790 | 4.1888 | 4.188790205 | 2 | 1 | 1.5e-15 | 7.1e-14 | 1.0e-07 | 2 | PASS |
| cone x cone | cut | 3 | 1 | 1(0) | 0 | 3 | Y | 12.566371 | 12.5664 | 12.566370614 | 3 | 1 | 2.8e-16 | 5.6e-14 | 1.0e-07 | 2 | PASS |
| cone x cone | fuse | 4 | 1 | 1(0) | 0 | 2 | Y | 29.321531 | 29.3215 | 29.321531434 | 4 | 1 | 0.0e+00 | 3.7e-14 | 1.0e-07 | 2 | PASS |
| cone x cyl | common | 3 | 1 | 1(0) | 0 | 3 | Y | 14.137334 | 14.1372 | 14.137166941 | 3 | 1 | 1.2e-05 | 7.4e-15 | 1.2e-04 | 129 | FAIL |
| cone x cyl | cut | 3 | 1 | 1(0) | 0 | 2 | Y | 2.617936 | 2.6180 | 2.617993878 | 3 | 1 | 2.2e-05 | 9.1e-14 | 1.2e-04 | 129 | FAIL |
| cone x cyl | fuse | 6 | 1 | 1(0) | 0 | 3 | Y | 45.029499 | 45.0295 | 45.029494701 | 6 | 1 | 9.1e-08 | 6.3e-15 | 4.9e-06 | 2 | PASS |
| cone x tor3 | common | 2 | 1 | 1(0) | 0 | 2 | Y | 3.041027 | 3.0410 | 3.041025548 | 2 | 1 | 5.5e-07 | 2.2e-07 | 6.6e-06 | 2 | PASS |
| cone x tor3 | cut | 4 | 1 | 1(0) | 0 | 4 | Y | 13.714060 | 13.7141 | 13.714135271 | 4 | 1 | 5.5e-06 | 4.9e-08 | 6.6e-06 | 2 | FAIL |
| cone x tor3 | fuse | 5 | 1 | 1(0) | 0 | 5 | Y | 38.980247 | 38.9803 | 38.980322538 | 5 | 1 | 1.9e-06 | 1.7e-09 | 6.6e-06 | 2 | FAIL |
| cyl x cyl | common | 6 | 1 | 1(0) | 0 | 0 | Y | 17.996134 | 18.0000 | 17.999999977 | 6 | 1 | 2.2e-04 | 1.8e-07 | 4.9e-04 | 65 | FAIL |
| cyl x cyl | cut | 7 | 2 | 2(0) | 0 | 2 | Y | 24.407755 | 24.4115 | 24.411500847 | 7 | 2 | 1.5e-04 | 1.4e-07 | 4.9e-04 | 65 | FAIL |
| cyl x cyl | fuse | 8 | 1 | 1(0) | 0 | 4 | Y | 66.821923 | 66.8230 | 66.823001670 | 8 | 1 | 1.6e-05 | 3.2e-09 | 4.9e-04 | 65 | FAIL |
| cyl x tor | common | 2 | 1 | 1(0) | 0 | 2 | Y | 2.259291 | 2.2593 | 2.259315239 | 2 | 1 | 1.1e-05 | 2.1e-07 | 1.8e-06 | 2 | FAIL |
| cyl x tor | cut | 5 | 1 | 1(0) | 0 | 3 | Y | 40.152186 | 40.1522 | 40.152185584 | 5 | 1 | 1.8e-15 | 1.2e-08 | 1.0e-07 | 2 | PASS |
| cyl x tor | fuse | 6 | 1 | 1(0) | 0 | 4 | Y | 65.418381 | 65.4184 | 65.418372851 | 6 | 1 | 1.3e-07 | 1.2e-08 | 3.0e-06 | 2 | PASS |
| sph x cone | common | 3 | 1 | 1(0) | 0 | 3 | Y | 15.917541 | 15.9175 | 15.917540810 | 3 | 1 | 4.7e-15 | 1.9e-10 | 1.0e-07 | 2 | PASS |
| sph x cone | cut | 4 | 2 | 2(0) | 0 | 6 | Y | 49.532306 | 49.5323 | 49.532306140 | 4 | 2 | 2.2e-15 | 1.3e-07 | 1.0e-07 | 2 | PASS |
| sph x cone | fuse | 4 | 1 | 1(0) | 0 | 5 | Y | 66.287467 | 66.2875 | 66.287466959 | 4 | 1 | 2.1e-16 | 9.4e-08 | 1.0e-07 | 2 | PASS |
| sph x cyl | common | 3 | 1 | 1(0) | 0 | 5 | Y | 31.939525 | 31.9395 | 31.939525311 | 3 | 1 | 3.7e-15 | 3.7e-09 | 1.0e-07 | 2 | PASS |
| sph x cyl | cut | 2 | 1 | 1(0) | 0 | 2 | Y | 33.510322 | 33.5103 | 33.510321638 | 2 | 1 | 6.4e-15 | 1.1e-07 | 1.0e-07 | 2 | PASS |
| sph x cyl | fuse | 5 | 1 | 1(0) | 0 | 3 | Y | 75.921822 | 75.9218 | 75.921822462 | 5 | 1 | 3.0e-15 | 4.9e-08 | 1.0e-07 | 2 | PASS |
| sph x sph | common | 3 | 1 | 1(0) | 0 | 0 | Y | 17.380856 | 17.3851 | 17.385115595 | 3 | 1 | 2.5e-04 | 2.8e-08 | 8.0e-04 | 65 | FAIL |
| sph x sph | cut | 2 | 1 | 1(0) | 0 | 4 | Y | 48.035938 | 48.0647 | 48.064731355 | 2 | 1 | 6.0e-04 | 7.2e-10 | 8.0e-04 | 65 | FAIL |
| sph x sph | fuse | 2 | 1 | 1(0) | 0 | 7 | Y | 81.546855 | 81.5750 | 81.575052993 | 2 | 1 | 3.5e-04 | 8.5e-08 | 8.0e-04 | 65 | FAIL |
| sph x tor | common | 2 | 1 | 1(0) | 0 | 2 | Y | 20.370088 | 20.3701 | 20.370105337 | 2 | 1 | 8.7e-07 | 3.2e-07 | 9.7e-07 | 0 | PASS |
| sph x tor | cut | 3 | 1 | 1(0) | 0 | 5 | Y | 45.079759 | 45.0797 | 45.079741612 | 3 | 1 | 3.9e-07 | 1.9e-07 | 9.7e-07 | 0 | PASS |
| sph x tor | fuse | 4 | 1 | 1(0) | 0 | 6 | Y | 70.345929 | 70.3459 | 70.345928879 | 4 | 1 | 2.0e-15 | 3.0e-08 | 1.0e-07 | 0 | PASS |
| tor x tor | common | 12 | 2 | 2(0) | 0 | 0 | Y | 6.534937 | 6.5365 | 6.536539794 | 12 | 2 | 2.5e-04 | 4.0e-07 | 6.3e-05 | 0 | FAIL |
| tor x tor | cut | 9 | 2 | 2(0) | 0 | 3 | Y | 18.731645 | 18.7296 | 18.729647473 | 9 | 2 | 1.1e-04 | 5.0e-08 | 6.3e-05 | 0 | FAIL |
| tor x tor | fuse | 6 | 1 | 1(0) | 0 | 6 | Y | 43.996754 | 43.9958 | 43.995834739 | 6 | 1 | 2.1e-05 | 5.1e-08 | 6.3e-05 | 0 | FAIL |

## 4. Failure classes

**Class 0 — fragmented / open / wrong solid count: 0 cells.** Nothing to report. Every
result is `isValid()`, every shell passes `Shell.isClosed()`, every solid count equals the
OCCT reference. The four multi-solid cells (`cyl x cyl cut`, `sph x cone cut`,
`tor x tor cut`, `tor x tor common`) are *correctly* two solids — OCCT says two as well.

**Class 1 — section-curve fidelity, 22 cells.** The result's 3D edge geometry in the shipped
file disagrees with the exact section by 1e-6 .. 1e-3. Two sub-shapes:

* *1a, dense degree-1 polyline section edges (16 cells).* Result boundaries are written as
  degree-1 B-splines with up to **129 poles = a 128-chord polyline**; OCCT then assigns those
  edges tolerances of 1.2e-4 .. 1.05e-3. Direct example, `box_common_cone` (the answer is
  exactly the cone): the operand cone's base circle round-trips as a STEP `CIRCLE` of length
  6.28318531, while the *result's* copy of the same circle is a 129-pole polyline of length
  6.28302656 with edge tolerance 1.387e-4 — hence 16.756703 read back for an exact 16.755161.
  Cells: box x sph (3), box x tor (3), sph x sph (3), cyl x cyl (3), cone x cyl (2, common+cut),
  box x cone common, box x cyl common.
* *1b, fitted higher-degree section edges (6 cells).* No dense polyline, but the fitted
  section carries 6.6e-6 .. 6.3e-5 edge tolerance and still misses 1e-6:
  `tor x tor` (all 3), `cyl x tor common`, `cone x tor3 cut`, `cone x tor3 fuse`.

The in-memory `mem_rel` column shows the same 22 cells at 1.7e-09 .. 8.1e-07. Since
`BRep::volume()` integrates the **pcurves** and the file carries the **3D curves**, the two
must be inconsistent with each other by that amount inside the result — which side is
authoritative was not determined here, but they cannot both be right.

**Class 2 — reference-free confirmation.** The shipped volumes violate the boolean partition
identities on their own, with no reference at all:

| pair | vol(A-B)+vol(A&B) vs vol(A) | vol(AuB) vs vol(A)+vol(B)-vol(A&B) |
|---|---:|---:|
| box x box2 | 0.0e+00 | 0.0e+00 |
| box x sph | 1.9e-05 | 3.2e-04 |
| box x cone | 2.4e-05 | 2.4e-05 |
| box x cyl | 3.9e-05 | 3.9e-05 |
| box x tor | 4.2e-05 | 6.0e-06 |
| sph x sph2 | **5.1e-04** | **5.0e-04** |
| sph x cone | 0.0e+00 | 6.5e-16 |
| sph x cyl | 1.1e-15 | 1.1e-15 |
| sph x tor | 6.5e-16 | 2.7e-07 |
| cone x cone2 | 6.4e-16 | 4.2e-16 |
| cone x cyl | 6.5e-06 | 1.0e-05 |
| cone x tor3 | 4.4e-06 | 4.4e-06 |
| cyl x cyl2 | 1.8e-04 | 1.2e-04 |
| cyl x tor | 5.7e-07 | 3.7e-07 |
| tor x tor2 | 1.6e-05 | 2.7e-05 |

9 of 15 pairs break at least one identity by more than 1e-6. The same identities on the
oracle cache hold to <= 4.3e-12, so the inconsistency is ours.

**Class 3 — outside the 45, same run: the SESSION_FREEFORM probe is genuinely broken, and
the matrix already says so.** Independently read:

| file | result shape(s) | closed | naked | verdict |
|---|---|---|---:|---|
| freeform_cut_box | 1 Shell, 8 faces | NO | 2 | open |
| freeform_common_box | 1 Shell, 6 faces | NO | 4 | open |
| freeform_fuse_box | Shell 13 faces + Shell 1 face | 1 of 2 | 0 + 2 | fragmented + open |
| freeform_pillow | Solid, 6 faces, vol 41.871898 | yes | 0 | OK |

The scorecard reported these as `s0` with `[id1] cut+com-64 rel 3.01e-2` and
`freeform verdict: FAIL`, so no blindness here — the freeform cell is honestly red.
One extra datum: the blob operand itself reads back at 64.589785 against the 64.1292 the
binary printed (rel 7.2e-3), i.e. the rational-NURBS operand does *not* round-trip, unlike
every analytic operand in the matrix (<= 8e-16).

**Non-defects worth recording so they are not re-litigated.** 8 cells contain 2-4 edges for
which FreeCAD raises "undefined curve type" (`box_fuse_sph`, `sph_common_cyl`, `sph_cut_cone`,
`sph_cut_sph2`, `sph_cut_tor`, `sph_fuse_cone`, `sph_fuse_sph2`, `sph_fuse_tor`) — all are
sphere pole/degenerate edges; the shapes are still valid and closed. `inspect_results.py`
also reports `cone_common_cone2` as READ-FAIL; that is its own `Part.Compound(naked).exportStep`
of degenerate edges throwing, not a read failure — the file reads fine in `matrix_verify2.py`.

## 5. Would the matrix's own gate have caught any of this?

The gate is `vol rel < 1e-6 AND nf == occt_nf AND is_solid()` (main_7.cpp:413).

**Topology: nothing was hidden, but only by luck of the corpus.** `is_solid()` is
"every edge has 2 trims", which a result that has come apart into several closed shells
satisfies exactly. `main_7.cpp` even carries a `shell_count_of()` helper (line 41) that
counts connected components — but it is only called from the rotated-chairs debug prints
(lines 1160-1168), **never in the primitive-matrix verdict**, so that gate cannot
distinguish 1 shell from 4. The proof is in the corpus itself: the gate prints `solid=1` for
`cyl x cyl cut`, `sph x cone cut`, `tor x tor cut` and `tor x tor common`, whose correct
answers are **two** disjoint solids. A genuine fragmentation would print the identical `1`.
So the gate had no ability to catch Class 0; it simply had nothing to catch here.

**Geometry: the gate hid 3 to 3.5 orders of magnitude.** It measures `BRep::volume()` on the
in-memory BRep, which integrates the loops' pcurves — all 45 pass at <= 8.52e-7. Re-measuring
the *exported* result against the *same* OCCT oracle fails 22 of 45, up to 4.35e-3. Quantified:

| threshold | in-memory (what the gate sees) | shipped STEP (what anyone else sees) |
|---|---:|---:|
| < 1e-6 | 45 / 45 | 23 / 45 |
| < 1e-5 | 45 / 45 | 25 / 45 |
| < 1e-4 | 45 / 45 | 32 / 45 |
| < 1e-3 | 45 / 45 | 42 / 45 |
| < 1e-2 | 45 / 45 | 45 / 45 |

Face counts are not affected: the exported result matches the OCCT reference face count in
all 45 cells, so `nf == occt_nf` is an honest check.

**Recommended gate additions (not applied — this session only measured):**
1. `shell_count_of(r)` into the verdict, compared against an OCCT reference shell/solid count.
2. A seam-safe naked-edge certificate, not `trim_indices.size()==2` alone.
3. Round-trip the written STEP and re-check volume/faces against the oracle — that single
   step turns today's silent 22/45 into a visible one.
4. Assert the partition identities `cut+common==A` and `fuse==A+B-common` on the exported
   volumes; they need no oracle and already flag 9 of 15 pairs.
