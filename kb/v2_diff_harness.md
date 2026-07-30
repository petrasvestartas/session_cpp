# v2 ↔ OCCT differential harness — build, validation, and the measured divergence table

Localising a v2 defect used to mean bisecting our own code. This makes it a diff: the v2 pipeline
dumps its internal state in `validation/occt_trace`'s exact record format, and one tool names the
first stage at which the two kernels disagree.

Everything below is measured. Provenance is in §7; anything that did not run says so.

---

## 0. Deliverables

| file | what |
|---|---|
| `session_cpp/src/v2/v2_dump.{h,cpp}` | dumps the v2 pipeline in `occt_trace`'s record format |
| `session_cpp/main_20.cpp` | tracer driver; same CLI as `occt_trace` |
| `validation/v2_trace/v2_trace_diff.py` | names the FIRST STAGE at which two traces diverge |
| `validation/v2_trace/run_cases.sh` | traces the corpus, prints the divergence table |
| `validation/v2_trace/selftest.sh` | harness self-test — no false positives |
| `validation/v2_trace/mutation_test.sh` | harness negative control — no blind spots |
| `validation/v2_trace/build_main_20.sh` | builds `main_20` WITHOUT editing `CMakeLists.txt` |
| `validation/v2_trace/traces/*.trace` | 38 captured v2 traces |

All new. `CMakeLists.txt`, `brep.cpp`, `brep_section.*`, `intersection.cpp`,
`nurbssurface_trimmed.cpp`, `file_step.cpp`, `main_7.cpp`, `v2_verdict.*` are untouched by this
work (they show as modified in `git status` because other sessions edit them concurrently; neither
`main_20` nor `v2_dump` appears in any of those diffs). Nothing was committed or pushed.

```bash
validation/v2_trace/build_main_20.sh          # -> session_cpp/build_v2diff/main_20
validation/v2_trace/run_cases.sh              # 38 traces + the divergence table
validation/v2_trace/selftest.sh               # must print SELFTEST PASS
validation/v2_trace/mutation_test.sh          # must print MUTATION PASS
python3 validation/v2_trace/v2_trace_diff.py <occt.trace> <v2.trace> [-v] [--tol 1e-4] [--row] [--md]
```

`main_20` flags: `--no-interf` (skip the probe arena), `--no-result` (skip the boolean),
`--trail <n>` (override the 768 trail samples).

---

## 1. What was built

### 1.1 `v2_dump` — the same record schema, tag for tag

`occt_trace.cpp` writes `TAG key=value` lines, `%.9g`, `|x|<1e-12` snapped to 0, every list-valued
field sorted, every set of like records emitted sorted. `v2_dump` reproduces that formatter
(`v2dump_r` / `v2dump_p3` / `v2dump_ints` are line-for-line ports of `R` / `P3` / `IntListStr`) and
the same tags: `TRACE SPEC ARG AFACE AFEDGE AEDGE AVERT DS RANGE SI DSVERT SD PAVE PB CB FI FIPB
IVV IVE IVF IEE IEF IFF SEC SEC2D SECPB STAGE RES RESSOLID RESSHELL RESFACE RESFEDGE RESEDGE
RESVERT SUMMARY`, plus five tags OCCT has no analogue for: `CAP` (capability declaration), `TOL`
(the tolerance the case actually ran at), `SECSTAT`, `PROBE`, `TIME`.

The operand grammar is copied from `occt_trace` so one spec string drives both tracers:
`sphere,r=` `cylinder,r=,h=` `box,dx=,dy=,dz=` `cone,r1=,r2=,h=` `torus,r1=,r2=`, plus
`center rotx= roty= rotz= tx= ty= tz=` applied in that order about the origin.

**Byte-stability, measured.** Two runs of the same binary on the same input, `cmp`-identical:
`box_box_cut`, `cone_cone_p2_cut`, `sph_cyl_roty23578_cut` on binary `0f02f213`, and `box_box_cut`
again on the rebuilt `1536dbcf`. `TIME` records are **off unless `SESSION_V2_DUMP_TIME` is set**,
gated on `(p && p[0])`, precisely because a wall-clock number would destroy this — `getenv()`
returns a non-null EMPTY string for `FOO=`, and reading that as "set" is one of the measurement
errors this program exists to avoid.

### 1.2 It is a REPLICA of the front end, not an in-situ hook — stated, not implied

`v2sol::v2_boolean()` runs its front end inside the file-static `v2sol_run_front` and returns only a
`BRep` plus a census, so there is nowhere to hang a per-stage callback without editing
`brep_v2_boolean.cpp`, a file this work does not own. `v2_dump` therefore runs three phases:

* **Phase 1, arena replica.** Trustworthy only because every parameter was read out of
  `brep_v2_boolean.cpp` and matched line for line:

  | production (`brep_v2_boolean.cpp`) | replica (`v2_dump.cpp`) |
  |---|---|
  | `scale = max(1e-9, diag(v2sol_brep_box(A) ∪ box(B)))`, verts + 5×5 surface grid | identical body |
  | `tol = max(1e-9, scale·1e-6)` | identical |
  | `ds.init({&A,&B}, max(tol,1e-7))` | identical |
  | materialise every edge's pave-block pool | identical |
  | `trail_samples = 768`, or `SESSION_V2_TRAIL_N` when `p && p[0]` | identical |
  | post-section pave-attachment loop | copied verbatim, marked as a copy |

  Records: `DS RANGE SI DSVERT SD PAVE PB CB FI FIPB IFF SEC SEC2D SECPB`.

* **Phase 2, interference probe.** `v2int::V2Interf` (VV/VE/EE/VF/EF) is green on its own driver
  (main_15 29/29) but the production boolean **does not call it** — `brep_v2_boolean.cpp` says so in
  its own comment at stage 0/1/2. Running it into the phase-1 arena would change what `v2sec` then
  sees, so it runs into a **separate arena** and every record carries `src=probe`. The comparison
  tool prints those rows as `PROBE` and never counts them as a divergence.

* **Phase 3, result.** `v2sol::v2_boolean()` itself, scored through the shared harness
  `src/v2/v2_verdict.h` and `brep_massprops` only. No metric is computed in `v2_dump`. The call is
  `v2_boolean(A, B, op, V2BooleanOptions{}, &rep)` — verified identical to the ladder driver's,
  which goes through `v2_cut(A,B)` → `V2BooleanOptions o; o.tolerance = 0;`.

A `CAP` record at the top of every trace lists which stages this kernel populates, so the tool can
tell **diverged** from **not implemented**.

**The replica caught its own first bug, and that is the point.** The first version stopped after
`V2Section::perform_all()`. The harness immediately reported `ds.split_edges occt=4 v2=0` on
box × box — v2 apparently splitting no operand edge at all. That was not the kernel: the production
front attaches section paves to operand edges *after* the section stage, and the replica was missing
that loop. Porting it verbatim moved v2 to `split_edges 4`, `pblock_splits 8`, identical to OCCT.
Recorded because it is exactly the failure mode a replica invites.

### 1.3 `v2_trace_diff.py` — geometry, never indices

The two kernels share no index space, no surface parameterisation (our sphere's u domain is `[0,4]`,
not `[0,2π]`) and no curve representation. So **nothing in the comparator compares an index or a
parameter**: every check is a count or a 3D-geometry match at a stated tolerance (default 1e-6 model
units), with greedy nearest-neighbour matching and the unmatched points printed.

Four classes of row: `OK`/`FAIL`, `N/A` (the `CAP` record says the stage is not populated), `PROBE`
(v2 side is the probe arena), and `INFO` — printed, never voted on, because the quantity is a
representation policy rather than geometry. Four `INFO` classes, each with a measured reason:

1. **`ds.si.*` and `ds.pblocks` raw counts.** OCCT builds a pave-block pool only for an edge that
   interferes; the v2 front materialises every edge's pool up front, and OCCT's DS also carries
   SOLID/SHELL/WIRE rows. Measured on box × box: 12 vs 48 pave blocks, pools on 4 vs 24 edges,
   while the *derived* split counts are 4 vs 4 and 8 vs 8.
2. **`input.degen_edges` / `res.degen_edges`.** OCCT carries a DEGENERATED edge over every pole and
   apex (a sphere is 3 edges, 2 degenerate); our primitives carry no edge record at a pole at all.
   Comparing raw `nedge` made this the first divergence of every curved case and hid everything
   downstream, so `input.counts` and `res.edges` compare **non-degenerate** edges.
3. **`sec.types`.** OCCT names a Geom type outright; `v2_dump` must MEASURE one from samples and its
   recogniser resolves only Line / Circle / Degenerated exactly — an OCCT `Ellipse` comes back
   `BSpline` (measured on `box_cone_p2_cut`: Ellipse ×4 vs BSpline ×3, purely the recogniser).
   Reporting that as a divergence would be the harness inventing one.
4. **`ds.faceinfo` is `N/A`.** The production v2 front never populates `BdsFaceInfo` In/On/Sc
   (measured 0 on all 38 cases against OCCT's 2–10); face selection goes through classify-once +
   InParts instead. A real architectural difference, stated rather than scored.

Two derived quantities do the real work at the arena stage:

* **`ds.vertices` / `ds.newvertices`** — the SD-resolved, deduplicated canonical vertex set. Both
  kernels leave an SD-superseded vertex *in* the dump, so comparing raw `DSVERT` reported 40 vs 24
  on box × box where the two kernels in fact name the same 24 points. That was the harness's second
  self-inflicted error and is fixed by resolving the `sd=` chain first.
* **`ds.split_paves` / `ds.split_edges` / `ds.pblock_splits`** — paves at an intersection node **on
  an operand edge** (`SI rank` 0 or 1), and the split counts. A pave on a created edge is a section
  carrier: OCCT keeps those inside `BOPDS_Curve` where they surface as `SECPB`, the v2 arena keeps
  them as real edges. Counting them compares a representation choice; excluding them takes box × box
  from 8 vs 40 to 8 vs 8.

**The reference is OCCT, which is not truth.** `kb/occt_trace_findings.md` measured OCCT itself
3.2e-3 off in volume at sph_cyl 23°. A `res.volume` FAIL there can mean our side is closer — and
§4.3 shows it does, by 74×. The tool's docstring says this so no reader treats a FAIL as a verdict.

---

## 2. Validation — the step that is not optional

### 2.1 Positive control: box × box, where both kernels are right

```
box_box_cut     cut     FIRST DIVERGENCE: NONE     0 failing stages
box_box_common  common  FIRST DIVERGENCE: NONE     0 failing stages
```

Every stage `OK`, including `ds.vertices 24/24`, `ds.newvertices 8/8`, `ds.split_paves 8/8`,
`ds.split_edges 4/4`, `ds.pblock_splits 8/8`, `sec.curves 8/8`, `sec.endpoints 8/8`,
`res.faces 10/10`, `res.volume 48/48`, `res.area 120/120`. Both kernels report
`naked=0 valid=1 solids=1`. Clean in both sweeps, on both binaries.

### 2.2 Self-test — no false positives (`selftest.sh`)

Every trace diffed against **itself**. Both sides are run, so an asymmetry between the OCCT-shaped
and v2-shaped resolvers would show up.

```
SELFTEST attempted=76 clean=76     (38 OCCT self-diffs + 38 v2 self-diffs)
SELFTEST PASS
```

### 2.3 Negative control — no blind spots (`mutation_test.sh`)

A self-test alone certifies nothing: a tool that never reports anything also passes it. Twelve known
defects are injected one at a time into the v2 side of `box_box_cut` — the case the harness calls
clean — each naming the stage that **must** turn FAIL.

```
arg_nvert_changed   input.counts.arg0    CAUGHT     res_face_dropped    res.face_areas      CAUGHT
arg_vert_moved      input.vertices.arg0  CAUGHT     res_naked_changed   res.naked           CAUGHT
arg_vol_changed     input.volume.arg0    CAUGHT     res_nface_changed   res.faces           CAUGHT
ds_vert_moved       ds.vertices          CAUGHT     res_vert_moved      res.vert_positions  CAUGHT
sec_end_moved       sec.endpoints        CAUGHT     res_vol_changed     res.volume          CAUGHT
sec_len_changed     sec.lengths          CAUGHT     sec_pblock_dropped  sec.pblocks         CAUGHT
CONTROL unmutated=-       MUTATION attempted=12 caught=12       MUTATION PASS
```

**This found a real blind spot.** On the first run `res_naked_changed` was a MISS: the injected value
was written `1.0`, the integer field parser raised `ValueError` and silently returned its default
`0`, which equalled OCCT's `0`, so the stage passed. A default-on-garbage path is a divergence the
harness cannot see. `i()` now parses through `float` and rounds; the mutation is caught and the
self-test still passes. That is the only change made to the comparison logic as a result of
validation.

### 2.4 Cross-controls: it fires where it should

| control | expectation | measured first divergence |
|---|---|---|
| OCCT `box_box_cut` vs OCCT `box_box_common` (same PaveFiller, different op) | agree through the DS, diverge at the result | `res.faces occt=10 v2=6` |
| OCCT `box_box_cut` vs OCCT `sph_sph_p1_cut` (different operands) | diverge at the input | `input.counts.arg0 6/12/8 vs 1/3/2` |
| OCCT `sph_cyl_roty23_cut` vs OCCT `sph_cyl_roty24_cut` (same shapes, 1° apart) | same counts, different positions | `input.vertices.arg1 2/2, matched=0` |

**Verdict on the harness: validated.** It reports nothing on 76 identical-input diffs, it agrees on
box × box where both kernels are correct, and it fires on all 12 mutations and all 3 cross-controls.

---

## 3. THE DIVERGENCE TABLE — 38 cases, 38 attempted, 38 completed, all `exit=0`

`tol = 1e-6` model units. `N` = number of failing stages. The last column is **every** failing stage,
because "everything after the first divergence is downstream" is a hypothesis, not a fact, and the
full signature is what lets a reader test it.

| case | op | first divergence | occt | v2 | N |
|---|---|---|---|---|---|
| box_box_cut | cut | **none** | - | - | 0 |
| box_box_common | common | **none** | - | - | 0 |
| box_box_half_fuse | fuse | ds.vertices | 16 | 272 | 13 |
| box_box_touch_common | common | ds.vertices | 12 | 272 | 12 |
| box_box_touch_fuse | fuse | ds.vertices | 12 | 272 | 13 |
| box_cone_p1_cut | cut | ds.vertices | 12 | 13 | 9 |
| box_cone_p1_common | common | ds.vertices | 12 | 13 | 9 |
| box_cone_p2_cut | cut | ds.vertices | 16 | 24 | 17 |
| box_cone_p2_common | common | ds.vertices | 16 | 24 | 15 |
| cone_cone_p1_cut | cut | ds.vertices | 13 | 15 | 19 |
| cone_cone_p1_common | common | ds.vertices | 13 | 15 | 19 |
| cone_cone_p2_cut | cut | ds.vertices | 11 | 21 | 21 |
| cone_cone_p2_common | common | ds.vertices | 11 | 21 | 21 |
| cyl_cyl_cut | cut | ds.vertices | 6 | 8 | 15 |
| cyl_cyl_common | common | ds.vertices | 6 | 8 | 16 |
| sph_box_cut | cut | ds.vertices | 17 | 18 | 14 |
| sph_box_common | common | ds.vertices | 17 | 18 | 14 |
| sph_box_fuse | fuse | ds.vertices | 17 | 18 | 14 |
| sph_cyl_roty0_cut | cut | ds.vertices | 6 | 6 | 9 |
| sph_cyl_roty0_common | common | ds.vertices | 6 | 6 | 9 |
| sph_cyl_roty20_cut | cut | res.solids | 1 | 0 | 2 |
| sph_cyl_roty20_common | common | res.solids | 1 | 0 | 4 |
| sph_cyl_roty23_cut | cut | res.solids | 1 | 0 | 4 |
| sph_cyl_roty23_common | common | res.face_areas | 3 | 3 | 5 |
| sph_cyl_roty23578_cut | cut | ds.vertices | 5 | 7 | 17 |
| sph_cyl_roty23578_common | common | ds.vertices | 5 | 7 | 13 |
| sph_cyl_roty24_cut | cut | sec.curves | 3 | 2 | 6 |
| sph_cyl_roty24_common | common | sec.curves | 3 | 2 | 10 |
| sph_cyl_roty25_cut | cut | sec.curves | 3 | 2 | 5 |
| sph_cyl_roty25_common | common | sec.curves | 3 | 2 | 8 |
| sph_cyl_roty30_cut | cut | sec.curves | 3 | 2 | 5 |
| sph_cyl_roty30_common | common | sec.curves | 3 | 2 | 8 |
| sph_cyl_roty45_cut | cut | sec.curves | 3 | 2 | 3 |
| sph_cyl_roty45_common | common | sec.curves | 3 | 2 | 8 |
| sph_sph_p1_cut | cut | ds.vertices | 6 | 5 | 18 |
| sph_sph_p1_common | common | ds.vertices | 6 | 5 | 19 |
| sph_sph_p2_cut | cut | ds.vertices | 7 | 8 | 14 |
| sph_sph_p2_common | common | ds.vertices | 7 | 8 | 15 |

Stage failure frequency over the 38 cases:

```
sec.endpoints 32   ds.vertices 24      res.vert_positions 21   sec.pblocks 18    res.naked   7
sec.lengths   26   ds.newvertices 24   res.face_areas 21       res.edges   16    res.shells  6
res.valid     25   res.volume 22       res.area 21             ds.split_edges 13 ds.cblocks  3
sec.curves    24   ds.split_paves 22   sec.block_nodes 20      res.faces   12
res.solids    24   ds.pblock_splits 22 res.verts 20
```

Result-level census (v2 from `v2v::v2_verdict`, OCCT from `BRepCheck_Analyzer` + `BRepGProp`):

```
CASE                     CLOSURE_RESID  NAKED SHELL         V2_VOL       OCCT_VOL VALID
box_box_cut                          0     0     1             48             48     1
box_box_common                       0     0     1             16             16     1
box_box_half_fuse                    0     0     1             96             96     1
box_box_touch_fuse                   0     0     1            128            128     1
box_box_touch_common     (no result: BOPERR v2-front-refused(delegated-to-kernel))
box_cone_p1_cut                      0     0     1     48.7528037     48.7528037     1
box_cone_p1_common                   0     0     1     15.2471963     15.2471963     1
box_cone_p2_cut           0.0679835262     0     1     52.6295654     46.5479313     0
box_cone_p2_common         0.321512485     0     1     11.3704346     17.4520687     0
cone_cone_p1_cut         0.00765258849     4     3     21.1143873     15.7076198     0
cone_cone_p1_common        0.371390676     5    62     774.926188      5.2542877     0
cone_cone_p2_cut          0.0850991025    21    11     29.6098993     6.60028262     0
cone_cone_p2_common        0.307700349     6     5     9.95193579     14.3436692     0
cyl_cyl_cut              7.81778983e-09     2     1     53.1940706     47.6817989     0
cyl_cyl_common           3.67471898e-08     2     2     3.35459786     8.86713168     0
sph_box_cut                          0     0     6     10.9834957     10.9955738     1
sph_box_common                       0     0     1     54.4663513     54.4542756     1
sph_box_fuse                         0     0     1     74.9834957     74.9955738     1
sph_cyl_roty0_cut                    0     0     1     50.3880515     50.3880515     1
sph_cyl_roty0_common                 0     0     1     15.0617955     15.0617955     1
sph_cyl_roty20_cut       1.67812512e-09     0     1     50.3880522      50.388017     0
sph_cyl_roty20_common    4.78400647e-09     0     1     15.0617947     15.0618286     0
sph_cyl_roty23_cut       2.04263035e-07     0     1     50.3880965     50.3846998     0
sph_cyl_roty23_common    5.82315174e-07     0     1     15.0617505     15.0649961     0
sph_cyl_roty23578_cut     0.00060382779     3     2      69.519196     50.3880515     0
sph_cyl_roty23578_common  9.1828805e-12     0     1     15.0591515     15.0617954     1
sph_cyl_roty24_cut       6.42813908e-07     0     1     50.3879917     50.3880515     0
sph_cyl_roty24_common    1.83252868e-06     0     1     15.0618552     15.0617954     0
sph_cyl_roty25_cut       8.40965959e-08     0     1     50.3880438     50.3880515     0
sph_cyl_roty25_common    2.39752477e-07     0     1     15.0618031     15.0617954     0
sph_cyl_roty30_cut       5.52677103e-09     0     1      50.388051     50.3880515     0
sph_cyl_roty30_common    1.57575032e-08     0     1      15.061796     15.0617954     0
sph_cyl_roty45_cut       4.49671511e-10     0     1     50.3880514     50.3880515     1
sph_cyl_roty45_common    1.28626216e-09     0     1     15.0617955     15.0617955     0
sph_sph_p1_cut            0.0478878282     0     1     68.2805528     58.5285258     0
sph_sph_p1_common           0.28437093     0     1     2.83070587     6.92132131     0
sph_sph_p2_cut           8.04601808e-05     0     1     50.0194193     50.0149375     0
sph_sph_p2_common        0.000206468174     0     1     15.4304276     15.4349095     0
```

`CLOSURE_RESID` is `v2_verdict`'s `closure_residual` = |Σ outward face vector areas| / area, from
`brep_massprops`. `closed()` requires it below 1e-9. See §4.6.

---

## 4. What the table localises

### 4.1 The section splits closed curves that OCCT leaves whole

Counting, over all 38 traces, closed section curves (`p0 == p1`) and how many carry more than one
pave block:

```
OCCT:  0 closed curves are split, in all 38 traces.  Always npb=1.
v2:   17 of the 31 cases that have a closed section curve split at least one of them.
```

`box_cone_p1_cut`, the cleanest instance. The z = −2 circle, radius 1.8, arc length **11.3097336 in
both kernels**:

```
OCCT  SEC  p0=1.8,0,-2  p1=1.8,0,-2  len=11.3097336  npb=1
      SECPB k=0  t0=0 t1=6.28318531  v1=45 v2=45              <- ONE closed block, node on the cone seam
v2    SEC  p0=0,1.8,-2  p1=0,1.8,-2  len=11.3097336  npb=2  closed=1
      SECPB k=0  t0=0    t1=0.25  len=2.82743339  v1=34 v2=36  <- a quarter arc
      SECPB k=1  t0=0.25 t1=1     len=8.48230016  v1=36 v2=34  <- and three quarters
```

`2.82743339 = 11.3097336/4` exactly: v2 cuts the circle at **two** nodes — the seam node
`(1.8,0,-2)` that OCCT also names, **plus its own marcher origin** `(0,1.8,-2)` at u = π/2. The
result carries both: `RESVERT` has 12 entries to OCCT's 10, and the two extras are exactly
`(0,1.8,-2)` and `(0,0.2,2)`, the marcher origins on the two circles.

This is why almost every curved case's first divergence is `ds.vertices`. It is a **redundant node**,
not a misplaced one. An earlier draft of this harness was going to fold "the split node of a closed
curve is a parameterisation choice" into the matcher; that concession would have **hidden** this
defect, and was dropped for exactly that reason.

### 4.2 box × cone: the section was right, the assembly was wrong — and the arithmetic said so

`box_cone_p1` — 2 section curves both sides, arc lengths identical to 9 figures, `res.faces` 3 = 3,
and **every face area identical**: 10.1787602, 0.125663706, 27.0687907. But at the first sweep
(binary `0f02f213`) the common's volume was 1.50796447 against OCCT's 15.2471963, closure residual
0.538.

The per-shell contributions in the dump determine the cause with no inference left over:

```
v2 as measured:  -6.78584013  -0.0837758041  +8.37758041  = 1.50796448   (= v2's reported 1.50796447)
flip the two planar caps:
                 +6.78584013  +0.0837758041  +8.37758041  = 15.24719634  (= OCCT's 15.2471963)
```

The face set was correct and two cap orientations were inverted. **A concurrent kernel change
between the two sweeps fixed exactly this**: on binary `1536dbcf`, `box_cone_p1_common` is now
`vol=15.2471963 closure=0 valid=1`, matching OCCT to all nine printed digits, and its failing-stage
count fell 12 → 9. `sph_box_fuse` moved too (`nsolid` 3 → 1) and `sph_box_common` from 35.6167954 to
54.4663513. **I did not make any of those changes** — they came from another track; the harness
measured them, and the prediction it made from arithmetic before the fix landed matched the fixed
kernel's output digit for digit.

`box_cone_p1_cut` is and was bit-exact: 48.7528037 both sides, area 112.764367 both sides. **This
pair is not broken at this pose.**

`box_cone_p2` (`roty=30, tx=0.7`) is the failing pose and its signature is different: the arena
acquires **8 spurious new vertices** (24 canonical vs 16) at box-corner-on-cone locations
`(-0.835898, ±2, -2)` and six more, `ds.split_edges` grows, and the result reaches
`closure_residual = 6.80e-2` with volume +13%. `SECSTAT` corroborates over-production:
`blocks=12 kept=6 dropped=6`.

### 4.3 sphere × cylinder: v2 is more accurate than OCCT approaching tangency, and breaks at it

The true answer is tilt-independent by symmetry; converged values from
`kb/occt_trace_findings.md`: cut 50.3880515, common 15.0617954, sum 65.4498469.

| tilt | v2 cut | \|err\| | OCCT cut | \|err\| | v2 partition err | OCCT partition err |
|---|---|---|---|---|---|---|
| 0° | 50.3880515 | 0 | 50.3880515 | 0 | 1e-07 | 1e-07 |
| 20° | 50.3880522 | **7.0e-07** | 50.3880170 | 3.45e-05 | **0** | 1.3e-06 |
| 23° | 50.3880965 | **4.50e-05** | 50.3846998 | 3.35e-03 | **1e-07** | 1.51e-04 |
| **23.578° (exact tangency)** | 69.5191960 | **1.91e+01** | 50.3880515 | **0** | **1.91e+01** | 0 |
| 24° | 50.3879917 | 5.98e-05 | 50.3880515 | 0 | 0 | 0 |
| 25° | 50.3880438 | 7.7e-06 | 50.3880515 | 0 | 0 | 0 |
| 30° | 50.3880510 | 5.0e-07 | 50.3880515 | 0 | 1e-07 | 0 |
| 45° | 50.3880514 | 1.0e-07 | 50.3880515 | 0 | 0 | 0 |

* At 23° v2's error is **74× smaller than OCCT's** (4.50e-05 vs 3.35e-03); at 20°, **49×** smaller.
  v2's cut + common partition is exact to 1e-7 at seven of the eight tilts. **A `res.volume` FAIL at
  20° and 23° is a difference, not a defect** — v2 is the closer of the two to converged truth.
* At the **exact pole tangency** v2 fails outright: cut = 69.519196, `naked=3`, 2 shells, closure
  6.04e-04. OCCT is exact there, and `kb/occt_trace_findings.md` measured how: it absorbs the
  tangency by **inflating the existing pole vertices in place**, 1e-07 → 1.702e-05, the only case of
  38 needing a re-intersection pass. v2 creates no such enlargement — its `DSVERT` tolerances stay at
  1e-07 — and instead mints `(0,0,±2.5)` as **new** arena vertices, 7 canonical to OCCT's 5.
  **Localisation: the tangency is handled by creating geometry where OCCT widens a tolerance.**
* From 24° up the first divergence is `sec.curves` 3 vs 2 — and it is **benign grouping**:
  `sec.pblocks` is 3 = 3 and `sec.block_nodes` 6 = 6. OCCT's extra "curve" is a near-pole sliver
  starting at `(0.0184053, 0, 2.49993)`, 2.5 being the sphere radius. Reading the full stage list
  rather than the first line is what shows this: `sph_cyl_roty45_cut` fails **only**
  `sec.curves, sec.endpoints, sec.lengths` and is otherwise identical through `res.area`.

### 4.4 cone × cone: the section itself is incomplete

`cone_cone_p1_cut`. Both kernels emit 6 section curves over the same branch-point set. Total arc
length:

```
OCCT  0.493646307 + 0.493681712 + 0.532577637 + 1.97972734 + 2.69855696 + 9.00246751 = 15.2007
v2    0.548402149 + 0.644847961 + 0.686402241 + 2.73940854 + 2.76912328 + 6.62738289 = 14.0156
```

**v2's section is 1.185 shorter — 7.8% of the intersection curve is missing.** OCCT's
1.97972734-long branch running between the two branch points `(±0.790099, −0.344828, 0.344828)` has
no v2 counterpart. `sec.pblocks` is 12 vs 8. Downstream: 3 faces vs 4, 4 naked edges, 3 shells,
vol 21.1143873 vs 15.7076198.

*(Correction to the previous revision of this file, which reported these sums as 15.2307 and 12.0156
and concluded "21% short". Both were arithmetic slips; the summands were right. The correct figures
are 15.2007, 14.0156 and 7.8%.)*

The tangential apex-contact point is `(1,0,0)`; OCCT names it exactly, v2 names it
`(1.00000033, 8.34e-07, −8.34e-07)`, 1.2e-06 off, and enters it into `ds.split_paves` three times.

`cone_cone_p2` (`roty=35, tx=1`) is catastrophic with the same signature: 21 canonical vertices vs
11, 11 shells, 21 naked edges, volume 29.61 vs 6.60. `cone_cone_p1_common` is worse still —
774.926188 against 5.2542877, across 62 shells.

**So for cone × cone the defect is upstream in the section, whereas for box × cone (§4.2) it was
downstream in the assembly.** That distinction is the whole point of the harness: before it, both
were "the cone cases are broken".

### 4.5 Coincident faces: a 64× over-segmentation the result currently survives

| | ds shapes | new verts | pave blocks | common blocks | sec curves | sec blocks | result vol |
|---|---|---|---|---|---|---|---|
| box_box_touch_fuse OCCT | 84 | 4 | 16 | 4 | 2 | 0 | 128 |
| box_box_touch_fuse v2 | 1350 | 776 | 1552 | 256 | 8 | **512** | 128 |
| box_box_half_fuse OCCT | 72 | 0 | 10 | 4 | 3 | 0 | 96 |
| box_box_half_fuse v2 | 964 | 520 | 1174 | 128 | 6 | **384** | 96 |

Each 4-unit coincident edge line becomes **64 pave blocks** (`SEC ... len=4 ... npb=64`, × 8 curves
= 512). `SECSTAT` confirms it is the section stage and not the replica:
`blocks=512 kept=512 trim_paves=504 section_edges=256 common_blocks=256`.

`box_box_touch_fuse` and `box_box_half_fuse` still produce the exact answer (128 and 96, `valid=1`),
so today this is a robustness and cost liability rather than a wrong answer. `box_box_touch_common`
is not: it prints `RES null=1` with `BOPERR v2-front-refused(delegated-to-kernel)` and produces no
result at all. These are also the only three cases where `ds.cblocks` fails — the only configuration
in the corpus that produces common blocks at all.

### 4.6 The closure-residual band

`v2_verdict::closed()` requires `closure_residual < 1e-9` in addition to `naked_real == 0` and
`nonmanifold == 0`. Over the 38 cases, **nine results are topologically watertight (`naked=0`,
`nonmanifold=0`, one shell) with a volume within 1e-5 relative of OCCT, yet score `valid=0` on the
residual alone**:

| case | closure_residual | v2 vol | OCCT vol | rel |
|---|---|---|---|---|
| sph_cyl_roty45_common | **1.28626216e-09** | 15.0617955 | 15.0617955 | 0 |
| sph_cyl_roty20_cut | 1.67812512e-09 | 50.3880522 | 50.3880170 | 7.0e-07 |
| sph_cyl_roty20_common | 4.78400647e-09 | 15.0617947 | 15.0618286 | 2.25e-06 |
| sph_cyl_roty30_cut | 5.52677103e-09 | 50.3880510 | 50.3880515 | 9.9e-09 |
| sph_cyl_roty30_common | 1.57575032e-08 | 15.0617960 | 15.0617954 | 4.0e-08 |
| sph_cyl_roty25_cut | 8.40965959e-08 | 50.3880438 | 50.3880515 | 1.5e-07 |
| sph_cyl_roty25_common | 2.39752477e-07 | 15.0618031 | 15.0617954 | 5.1e-07 |
| sph_cyl_roty24_cut | 6.42813908e-07 | 50.3879917 | 50.3880515 | 1.2e-06 |
| sph_cyl_roty24_common | 1.83252868e-06 | 15.0618552 | 15.0617954 | 4.0e-06 |

`sph_cyl_roty45_common` misses the gate by a factor of **1.29** while reproducing OCCT's volume to
all nine printed digits. Four of the nine sit in `[1e-9, 1e-8)`.

**This is not a proposal to change the threshold.** `v2_verdict.h` is the single shared harness and
this work does not get to move it. It is the measurement, handed to whoever owns the closure track
(main_19): the gate is absolute, not scale-relative, and on this corpus it is what separates
`res.solids 1 vs 0` from a real failure. `res.solids` and `res.valid` are the 4th and 3rd most
frequent failing stages (24 and 25 of 38), and this band is part of why.

`cyl_cyl_cut` is the near-miss worth naming separately: `closure_residual = 7.82e-09` — within 8e-9
of closing — but with 2 genuinely naked edges and volume +12%. `ds.newvertices` is 6 vs 8, the two
extras at `(1, ±1.11803, −2.2e-06)`, i.e. a **duplicate pair 2.2e-06 from the correct nodes** while
the trace's own `TOL` line records an arena fusion tolerance of 1.17e-05 — five times the gap. Two
nodes the arena's own tolerance says are one were not fused.

### 4.7 An inconsistency between two shared modules, not chased

On `box_cone_p1_common` at binary `0f02f213`, `v2_verdict` reported `nshell=1` while
`brep_massprops`' own shell partition put the three faces in **three** separate shells
(`RESSHELL i=1..3`, one face each). Both numbers come from shared modules; the dump prints each from
its own source rather than picking one. The §4.2 fix made it moot for that case, but the two
partitions can disagree and **I did not investigate why**. Flagged for the owners.

---

## 5. Cost

Wall time per case, `SESSION_V2_DUMP_TIME=1`, phases separated so nothing is attributed by guess:

| case | interf probe | front (arena + section) | v2_boolean | verdict + massprops |
|---|---|---|---|---|
| box_box_cut | 26 ms | 111 ms | **17 978 ms** | 3 ms |
| box_box_touch_fuse | 1 224 ms | 3 276 ms | 3 610 ms | 2 ms |
| cone_cone_p1_cut | 46 ms | 2 756 ms | 5 903 ms | **8 911 ms** |
| sph_cyl_roty24_cut | 195 ms | 200 ms | 8 713 ms | 1 712 ms |

The tracer's own front is cheap (0.11 s on box × box). `v2_boolean` takes **18.0 s on a box × box
cut**; `brep_massprops` takes 8.9 s on the broken 3-shell cone × cone result. Performance is
main_24's lane — this is a byproduct, reported because it was measured, not analysed further.

---

## 6. Limits — what this harness does NOT measure

Stated so nobody reads more into a green stage than is there.

1. The DS/section side is a **replica** of the production front, not an observed hook. It is
   deterministic and reproduces the driver's arena (proved by box × box matching OCCT exactly), but
   **if `v2sol_run_front` changes, `attach_section_paves` in `v2_dump.cpp` must be mirrored.** A
   one-line hook in `brep_v2_boolean.cpp` would remove this risk; that file is not owned here.
2. `IVV/IVE/IVF/IEE/IEF` come from a **probe arena**. Reported, never scored.
3. `FaceInfo In/On/Sc` (`ds.faceinfo`) and the input→output face image map (`res.imgface`) are not
   produced by the v2 boolean at all, so those stages are `N/A`.
4. `SEC2D` 2D footprints are dumped on both sides but **not compared**: our surfaces carry their own
   parameterisation ([0,4] for a full turn, not [0,2π]), so the numbers are not commensurable. The
   *shape* of the footprint — does it wrap, does it leave the rectangle — is, and is left for a
   reader; no automatic check exists.
5. `RESFEDGE` is emitted with `uv0=- uv1=-`: pcurve endpoint parameters are not dumped, so OCCT's
   seam-detection-from-pcurve cannot be reproduced. Listed in the trace's `CAP missing=` field.
   `RESSHELL closed=` is taken from the verdict's **global** `naked_real`, not per shell.
6. The curve-type recogniser resolves Line / Circle / Degenerated only; everything else is
   `BSpline`. `sec.types` is therefore `INFO`.
7. Truncated cones (`r2 != 0`) cannot be built — `BRep::create_cone` makes an apex cone only. The
   corpus uses `r2=0` throughout so no case was lost, but such a spec exits 2 with `SPECERR`.
8. `torus` is supported by the dumper but **no torus case exists in the OCCT corpus**, so no torus
   row was produced.
9. The comparison is v2 against OCCT only. Nothing here scores v2 against the current kernel; the
   ladder (main_16) owns that.

---

## 7. Provenance

```
date                2026-07-26
v2 tracer           session_cpp/build_v2diff/main_20   sha1 1536dbcfc9f1dc2b843c3a75908dcc32986d2ab6
  (first sweep)                                        sha1 0f02f213838a9d2b9533ff01d8b04e4b5e99d37c
occt tracer         validation/occt_trace/build/occt_trace  sha1 57eeedd1ac0e37fbce865a75504026d09fafc0d4
OCCT version        8.0.0.rc2-a66b3fd6   (TRACE record of every occt trace)
build               Release, gcc, main_17's own flags reused verbatim
environment         no SESSION_* variable set  (env | grep '^SESSION_' -> empty)
                    => SESSION_V2_TRAIL_N unset, tracer used the driver default of 768 trail samples
cases               attempted 38, completed 38, exit=0 for all 38  (traces/_status.txt)
control cell        box_box_cut + box_box_common: 0 failing stages in BOTH sweeps, BOTH binaries
self-test           76/76 clean            negative control  12/12 caught
byte-stability      cmp-verified on 4 traces across both binaries
```

`main_20.cpp` is not in `CMakeLists.txt`'s `foreach(MAIN_ID …)` because that file is owned by
concurrent sessions. `build_main_20.sh` builds `main_17` normally — which compiles
`src/v2/v2_dump.cpp` into `session_v2` via the existing `CONFIGURE_DEPENDS` glob — then reuses
`main_17`'s generated compile flags and link line verbatim, substituting only the object file and
the output name. Anyone who wants `main_20` as a first-class target need only add `20` to that
`foreach`. `v2_dump.cpp` compiles with zero warnings and `main_17` still reports
`TOTAL pass=5 fail=0 -- harness VALIDATED` with it in `session_v2`; `main_7` and the v1 targets do
not link `session_v2` at all.

### What changed in this revision

* `run_cases.sh` extended from 12 cases to the **full 38-case OCCT corpus**, with per-case exit-code
  capture (`traces/_status.txt`) and attempted-vs-completed counts printed.
* `v2_trace_diff.py`: all-failing-stages column, `--md` mode, and the `i()` parser fix from §2.3.
* **New** `selftest.sh` (76/76) and `mutation_test.sh` (12/12) — the harness had no negative control
  before, and the mutation test immediately found one blind spot.
* `v2_dump.cpp`: env-gated phase timing (`TIME` records, off by default).
* Corrected two arithmetic errors in §4.4 carried by the previous revision (section-length sums).
* Re-measured everything on binary `1536dbcf` after a concurrent kernel fix landed (§4.2).

### Note on concurrency

The shared build was broken for part of this session by another agent's in-progress edit to
`src/intersection.cpp` (`invalid initialization of reference … PBNode`), which blocked rebuilding
`main_20` for roughly 40 minutes. It was fixed by its owner and the rebuild then succeeded. The
first sweep therefore ran on binary `0f02f213` and the final sweep on `1536dbcf`; §4.2 reports what
moved between them, which is a real kernel change made by another track, not by this one.

---

## 8. Highest-value next probes this harness makes cheap

Each is now a one-command re-measurement instead of a bisection.

| probe | command | what to look at |
|---|---|---|
| the redundant closed-curve node (§4.1) | `v2_trace_diff.py …/box_cone_p1_cut.trace traces/box_cone_p1_cut.trace -v` | `SEC npb=2 closed=1` — one node too many on every closed section circle, in 17 of 31 cases |
| cone × cone missing branch (§4.4) | `… cone_cone_p1_cut.trace -v` | 7.8% of section arclength absent; OCCT's 1.9797 branch has no counterpart |
| exact pole tangency (§4.3) | `… sph_cyl_roty23578_cut.trace -v` | OCCT inflates the pole vertex 1e-07 → 1.702e-05; v2 mints `(0,0,±2.5)` instead |
| unfused node pair (§4.6) | `… cyl_cyl_cut.trace -v` | two nodes 2.2e-06 apart, arena fusion tolerance 1.17e-05 |
| coincident-face blowup (§4.5) | `… box_box_touch_fuse.trace -v` | 512 section blocks vs 0; `box_box_touch_common` produces no result at all |
| closure gate (§4.6) | `grep '^RES ' traces/*.trace` | nine watertight results scored `valid=0` on `closure_residual` alone |
