# occt-test-mining

Systematic mining spec for the OCCT boolean test suite at
`C:/brg/compas_occt/external/occt/src/occt/tests/boolean/` (survey read 2026-07-24), plus the
DRAW command layer that executes it (`src/Draw/TKTopTest/BOPTest/*`) and the Tcl verification
layer (`resources/DrawResources/CheckCommands.tcl`).

Companion specs (referenced, not duplicated): `kb/occt_pavefiller-core.md` (intersection phase the
`bop` command drives), `kb/occt_tolerance-model.md` (why checkshape passes), `kb/occt_builder-assembly.md`,
`kb/occt_ff-posttreat-samedomain.md` (the same-domain machinery these cells stress),
`kb/occt_interference-vef.md`, `kb/occt_ssi-walking.md`, `kb/occt_history-gf-scale.md`.

Deliverable: category census, hard-class identification, and a **ready-to-port cell list of 51 cells
(17 pairs x 3 ops) + reserves** targeting coincidence / tangency / grazing / thin walls — the classes
our pipeline currently fails.

---

## CATEGORY CENSUS (tests/boolean/*, case counts, inline = no external .brep needed)

| category | total | inline | content | mining value |
|---|---|---|---|---|
| bopfuse_simple | 375 | 375 | primitive-pair grid, `bop`+`bopfuse` | PRIMARY — same grid as the 3 below |
| bopcut_simple | 379 | 379 | same grid, `bopcut` | PRIMARY |
| bopcommon_simple | 378 | 378 | same grid + 3 extra (ZP7 cone-face prism, ZP8 cavity, ZP9 grazing spheres) | PRIMARY |
| boptuc_simple | 373 | 373 | same grid, `boptuc` (= cut B-A, `BOPAlgo_CUT21`) | PRIMARY (swapped-cut duality) |
| bcommon/bcut/bfuse_simple | 83/110/102 | all | older one-shot `bcut/bfuse` API, DIFFERENT grid (sphere-corner-box family) | SECONDARY — source of obcA1 cell |
| bopfuse/cut/common/tuc_complex | 128/140/115/73 | 0 (2) | `restore locate_data_file *.brep` imported breps | NOT portable (maps to chairs campaign); only bfuse_complex/J1,J5 inline |
| bfuse/bcut/bcommon_complex | 165/153/25 | 2/0/0 | same, customer bugs | J1 = cyl internal line-tangency (MINED below) |
| *_2d (7 dirs) | 28–170 | 0 | face/wire booleans on restored data | out of scope (planar arrangement tests) |
| bsection / bopsection | 164/48 | 3/1 | solid x face/edge sections | DEFER — no section op in our gate |
| volumemaker | 74 | 74 | `mkvolume` from infinite-face soups (planes+cones, 1e6 extents) | REFERENCE for combine; not pair-cell portable |
| cells_test | 68 | 68 | `bcbuild`/`bcadd` cells builder (keep cells+boundaries) | REFERENCE for future general-fuse |
| gdml_public | 25 | 25 | GEANT detector chains: nested thin boxes, arbitrary-axis rotations, fuse/cut chains, NO numeric checks (checkshape+image only) | thin-wall + rotated-frame chains (2 reserves mined) |
| gdml_private | 326 | 0 | `source *.gdml.tcl` external | not portable |
| opensolid | 9 | 9 | booleans on OPEN shells (5-face boxes), expects `Faulty` | DEFER — our classifier assumes closed |
| periodicity / mkconnected | 6/5 | 6/3 | `makeperiodic`/`makeconnected` | out of scope |
| history / splitter / simplify / removefeatures | 9/12/5/58 | 9/10/4/1 | history API, `bsplit`, unify-same-domain | REFERENCE (history spec already in kb/occt_history-gf-scale.md) |
| bcutblend | 1 | 0 | cut+blend radius | skip |

Row semantics of the bop*_simple grid (case files named `[A-Z][1-9]` and `Z[A-P][1-9]`; the four op
dirs are IDENTICAL geometry per case name, only the final op differs — verified on A5/C5/F5/K5/Z5/ZD5/ZH5/ZM5):
rows A–B/E/K box-x-box axis-aligned **coincidence sweeps** (shared corner/planes); C–D/F–J/L–Q box-x-box
**rotated grazing** (slabs at 45 deg / atan(1/2), corner-piercing, inscribed diamonds); R–ZC cylinder-x-box
(**inscribed / corner-on-surface tangencies**); ZD–ZF cyl-x-cyl (**coaxial tubes, cap-on-cap stacks,
equal-radius orthogonal Steinmetz**); ZF–ZH box-x-cone (**cap-circle-on-plane coincidence**); ZH–ZJ
cyl-x-sphere (**equator==rim**); ZK–ZL cyl-x-cone, cyl-x-torus (seam-crossing); ZM–ZN cone-x-cone;
ZO–ZP box-x-cyl axis-on-wall/edge + multi-solid/halfspace/cavity specials. Row letters DRIFT by one
between digit columns in places — always read the case file, never infer from a neighbor.

OCCT's own known-failures in these grids: exactly 1 TODO in bopfuse_simple (ZP6, 3-tori Linux area),
0 in the other three — the grid is a RELIABLE oracle target.

---

## STAGE PIPELINE (ordered, exact file+function names)

How one OCCT boolean test executes, and where our mining flow hooks in:

1. **Grid selection** — `tests/boolean/grids.list`: 35 numbered grids (001 bcommon_2d … 035 mkconnected).
2. **Setup** — `tests/boolean/begin`: `pload TOPTEST`, `cpulimit 300` (hard 5-min per case),
   `dset SCALE 100` (used by some 2d cases).
3. **Case script** — geometry via DRAW primitives (`box x0 y0 z0 dx dy dz` = CORNER-placed;
   `pcylinder r h`, `pcone r1 r2 h` = base at z=0 axis +Z; `psphere r`, `ptorus R r` = centered;
   `ttranslate`, `trotate px py pz ax ay az deg` = rotate about axis through POINT p).
4. **Intersection once, ops many** — `src/Draw/TKTopTest/BOPTest/BOPTest_BOPCommands.cxx`:
   `bop` (registered line 81, impl fills static `BOPAlgo_PaveFiller* pPF`, line 150) runs the full
   pave-filler ONCE; `bopcommon`/`bopfuse`/`bopcut`/`boptuc` (lines 82–85) all call
   `bopsmt(di,n,a,BOPAlgo_COMMON|FUSE|CUT|CUT21)` (line 195) which builds `BOPAlgo_BOP` (line 217)
   REUSING `pPF`. One intersection, four extractions. Old-style `bfuse r s1 s2`/`bcut r s1 s2`
   (lines 89–90) are one-shot `BRepAlgoAPI` calls. `mkvolume` -> `BOPTest_VolumeMaker.cxx`
   (`BOPAlgo_MakerVolume`); `bcbuild`/`bcadd` -> `BOPTest_CellsCommands.cxx` line 52 (`BOPAlgo_CellsBuilder`).
5. **Numeric gate** — `resources/DrawResources/CheckCommands.tcl` `proc checkprops` (line ~520):
   runs `sprops result 1.0e-4`, regex-extracts `Mass`, compares to expected `-s` at RELATIVE
   `depsilon = 1e-2` (1 %). `-s empty` requires mass == 0 exactly. A zero mass when a number was
   expected is a separate hard error ("The command is not valid").
6. **Topology gate** — `tests/boolean/end`: `if {[isdraw result]} { checkshape result }` on EVERY case;
   `tests/boolean/parse.rules`: any line matching `/\bFaulty\b/` -> FAILED; the vprops
   "Relative error of mass computation" message is whitelisted OK.
7. **Our mining flow** (target of this spec) — enumerate case file -> convert coordinates
   (corner->centered box, trotate-about-point -> single (R, t') pair; formulas in PITFALLS) ->
   append to `occt3_placements()/occt3_pairs()` in `session_cpp/main_7.cpp` (new grid, env
   `SESSION_OCCT3`, modeled on `occt2_placements()` at line 131) -> `oracle.exe` (`validation/occt_oracle/oracle.cpp`,
   `OP boolean` grammar line 719) fills `validation/occt_cache.txt` full-precision vol+nfaces ->
   scorecard gate; OCCT `-s` areas kept as an independent 1 %-cross-check column.

---

## DATA STRUCTURES

- **Grid case file** = flat Tcl: 1–4 geometry commands, optional `dset` exact-value macros
  (`sqrt(2)`, `atan2(1,2)*180/pi` — angles are EXACT algebraic constructions, not round numbers),
  `bop a b` + `bop<op> result`, `checkprops result -s <area|empty>`, `checkview`.
- **Harness triple** per dir: `begin` (env), `end` (checkshape), `parse.rules` (verdict regexes).
- **Ours (main_7.cpp)**: `Place {kind, params, xf[7] = tx ty tz ax ay az deg}` — xf applies
  `t * r`, rotation about ORIGIN, currently principal-axis-only via `xf_of()` (line 194: picks
  `Xform::rotation_x/y/z` by first nonzero axis component); `Ref {vol, nf}` cache rows in
  `validation/occt_cache.txt` (`key \t vol \t nfaces`); `ChainCell {label, keys, modes}` for
  left-fold boolean chains (line 183).
- **Oracle grammar** (`validation/occt_oracle/oracle.cpp`): `OP boolean / MODE m / SHAPE kind p... XF tx ty tz ax ay az deg`
  — XF rotation is ALREADY arbitrary-axis (`gp_Ax1` at line 114); solid `cone` is 2-param full cone
  (line 298); truncated cone exists only in the SURF grammar (line 10).
- **Proposed cell row** (this spec's list): `id | class | occt-src | Place A | Place B | expected areas
  fuse/cut/common/tuc @1% | oracle vol/nf (cache)` — tuc recorded as `cut(B,A)` reference.

---

## CONSTANTS & TOLERANCES (exact values)

- `cpulimit 300` — 5-minute hard kill per case (`tests/boolean/begin`).
- `checkprops` computation epsilon `1.0e-4` (passed to `sprops`/`vprops`); comparison `depsilon = 1e-2`
  (1 % RELATIVE) — the published `-s` values are only trustworthy to 1 %; regenerate full precision
  via our oracle, never gate at better than 1 % against the printed number.
- `-s empty` => mass == 0.0 exactly (no epsilon).
- `dset SCALE 100` in `begin` (2d grids); gdml scales are mm with extents up to 1270.
- volumemaker face extents `-1000000..1000000` (infinite-face proxies) — 1e6 scale, tolerance stress.
- gdml_public/A1 noise rotation: `trotate ... 1 0 0 6.2725381128105878e-046` — a 6.3e-46 DEGREE
  rotation (denormal-adjacent near-identity transform) that OCCT must treat as identity-but-not-identity.
- Exact expected identities in the mined cells (checkprops values that are closed-form):
  A5 fuse `6` (== box area), F5 fuse `6`, Z5 fuse `18.8496` (= 6*pi), ZD4/ZD5 fuse `31.4159` (= 10*pi),
  ZD8 common `16` (Steinmetz 16 r^2, r=1), ZP1 cut `96` (== 4x4x4 box area), ZF6 fuse `96`, ZP5 fuse `600`.

---

## INVARIANTS

1. **checkshape on every result** (`end` script): no Faulty sub-shape — our equivalent hard gate:
   naked-edge count == 0 + winding-classified closed solid.
2. **Area conservation**: `S(fuse) + S(common) = S(A) + S(B)` for transversal boundaries — verified
   exactly on A5 (6+2.5 = 6+2.5) and C5 (6.41789+3.61764 = 6+4.03553). Free cross-check our gate can
   evaluate with no oracle call.
3. **Duality**: `tuc(A,B) == cut(B,A)` (`BOPAlgo_CUT21`); `common` symmetric. Every mined pair gives a
   4th expectation for free by swapping.
4. **Seam invariance**: ZD4 vs ZD5 (and ZL2 vs ZL5, torus) differ ONLY by rotating one operand about
   its own axis (seam moved 90/270 deg) and have IDENTICAL expected areas — any divergence between the
   twin cells is a seam-aliasing bug by construction.
5. **Contact-only identity results**: when B touches A without volume overlap (ZP1 common empty,
   ZD4 common empty, F5/Z5 tuc empty) the surviving ops must return the EXACT untouched operand area
   (ZP1 cut == 96 == S(box)); classification must not eat the contact face.
6. **One intersection, four ops** (`bop` then `bopfuse/cut/tuc/common`): all four results derive from
   one PaveFiller DS — internal consistency across ops on the same pair is assumed by the suite.
7. **Determinism**: every inline case is fully reproducible from the script text alone (dset exact
   algebra); our cache keys stay valid forever.

---

## PITFALLS

- **Corner vs centered box**: DRAW `box x0 y0 z0 dx dy dz` places the CORNER; our `create_box` is
  centered. Convert: pre-rotation center `c_box = (x0+dx/2, y0+dy/2, z0+dz/2)`. pcylinder/pcone
  (base z=0) and psphere/ptorus (centered) match our constructors 1:1.
- **trotate is about a POINT**: `trotate px py pz ax ay az deg` composes `W' = T_c R T_{-c} W`
  (c=(px,py,pz)). Fold to our single `(R, t)` form: start `R=I, t=c_box` (box) or `t=0`; per
  ttranslate `t += v`; per trotate `R' = R_rot R`, `t' = R_rot (t - c) + c`. Portable into today's
  `Place` iff final R is one principal rotation; otherwise needs the xf_of arbitrary-axis upgrade
  (PORT MAP row 4) — the oracle side already accepts arbitrary axes.
- **Argument order**: `bcut result s b` = s - b; `boptuc` = tool - object (B - A). Getting tuc
  backwards silently passes fuse/common and corrupts two cells.
- **checkprops is 1 % loose**: do not "fix" a cell to match a printed area to 5 digits — the suite
  itself only enforces 1e-2 relative; full-precision truth comes from our oracle cache.
- **`-s empty` is exact-zero**, not small: an epsilon-area sliver result FAILS empty cells — these are
  precisely our sliver-suppression regressions (micro-piece filter must fire).
- **bopcommon_simple has 3 extra cases** (ZP7–ZP9) absent from the other three dirs — scripted diffs
  across op dirs must key by case NAME, not index.
- **Row-letter drift**: family boundaries shift by one case between letter rows (ZF1 is cyl-x-cyl,
  ZF5 box-x-cone). Read each file; never batch-assume a row's geometry.
- **gdml_public has NO numeric checks** — checkshape + screenshot only; porting those cells REQUIRES
  oracle-generated references (a wrong-but-valid solid would pass OCCT's own harness there).
- **`psphere s pl 7.5` centers the sphere at a plane's origin point** (ZP9) — commas in DRAW args are
  tolerated; don't parse them as separate tokens.
- **volumemaker/opensolid are not pair-battery material**: infinite-face mkvolume soups and
  deliberately-Faulty open shells; mining them into the pair gate produces meaningless cells.
- **truncated cones**: `pcone r1 r2 h` with r2 != 0 (ZF6, ZM5, ZP7) has NO counterpart in our solid
  builder or the oracle's solid grammar (2-param full cone only) — blocked until create_cone(r1,r2,h).

---

## READY-TO-PORT CELL LIST (51 core cells = 17 pairs x 3 ops; + reserves)

Format: Place = `kind params | tx ty tz ax ay az deg` (our conventions, conversions already applied).
Expected `-s` areas are OCCT-published (gate at 1 %); vol/nfaces to be filled by `oracle.exe --refresh`
into `validation/occt_cache.txt`. tuc value = reference for `cut(B,A)`. All results must pass
checkshape (naked == 0). Cells 1–51 = rows below x {fuse, cut, common}.

### Class C — coincidence / same-domain (cells 1–18)

| pair | OCCT src | A | B | fuse | cut | common | tuc |
|---|---|---|---|---|---|---|---|
| oqA5 | bop*_simple/A5 | `box 1 1 1 \| 0.5 0.5 0.5 0 0 1 0` | `box 0.5 1 0.5 \| 0.25 0.5 0.25 0 0 1 0` | 6 | 5.5 | 2.5 | empty |
| oqB5 | bop*_simple/B5 | same A | `box 1.5 0.5 0.5 \| 0.75 0.25 0.25 0 0 1 0` | 7 | 5.5 | 2.5 | 1.5 |
| oqZD4 | bop*_simple/ZD4 | `cylinder 1 2 \| 0 0 0 0 0 1 0` | `cylinder 1 2 \| 0 0 2 0 0 1 0` | 31.4159 | 18.8496 | empty | 18.8496 |
| oqZD5 | bop*_simple/ZD5 (seam twin: + deg=90 about z; MUST equal oqZD4) | same A | `cylinder 1 2 \| 0 0 2 0 0 1 90` | 31.4159 | 18.8496 | empty | 18.8496 |
| oqZP1 | bop*_simple/ZP1 | `box 4 4 4 \| 2 2 2 0 0 1 0` | `cylinder 1 4 \| 1 1 4 0 0 1 0` | 121.133 | 96 | empty | 31.4159 |
| oqZP2 | bop*_simple/ZP2 | same A | `cylinder 2 6 \| 0 0 -2 0 0 1 0` (axis == box vertical edge) | 161.681 | 86.2832 | 34.8496 | 103.965 |

Notes: oqA5/oqB5 = 3 shared wall-planes (one full-face + partial coplanars); oqZD4/5 = cap-on-cap
annular coincidence with seam offset (invariant #4); oqZP1 = cap-circle-ON-plane, cut must return the
EXACT box (96); oqZP2 = cylinder axis lying IN two wall planes simultaneously.

### Class T — tangency (cells 19–36)

| pair | OCCT src | A | B | fuse | cut | common | tuc |
|---|---|---|---|---|---|---|---|
| oqF5 | bop*_simple/F5 | `box 1 1 1 \| 0.5 0.5 0.5 0 0 1 0` | `box 0.70710678 0.70710678 1 \| 0.5 0.5 0.5 0 0 1 -45` (inscribed diamond: 4 vertex-on-face lines + coplanar z) | 6 | 7.82843 | 3.82843 | empty |
| oqZ5 | bop*_simple/Z5 | `cylinder 1 2 \| 0 0 0 0 0 1 0` | `box 1.41421356 1.41421356 1 \| 0 0 0.5 0 0 1 0` (4 internal line tangencies) | 18.8496 | 24.5064 | 9.65685 | empty |
| oqS5 | bop*_simple/S5 | same A | `box 1.70710678 1.70710678 2 \| 0.14644661 0.14644661 1 0 0 1 0` (vertical box edge ON cylinder) | 21.2532 | 13.0816 | 17.0816 | 14.97 |
| oqZD8 | bop*_simple/ZD8 | `cylinder 1 4 \| 0 0 0 0 0 1 0` | `cylinder 1 4 \| 0 2 2 1 0 0 90` (EQUAL radii, orthogonal, axes meet at (0,0,2): Steinmetz with 2 tangent points) | 46.8319 | 31.4159 | **16** (exact) | 31.4159 |
| oqZH5 | bop*_simple/ZH5 | `cylinder 4 8 \| 0 0 0 0 0 1 0` | `sphere 4 \| 0 0 8 0 0 1 0` (equator circle == cap rim) | 351.858 | 351.858 | 150.796 | 150.796 |
| obcA1 | bcut/bfuse/bcommon_simple/A1 | `sphere 1 \| 0 0 0 0 0 1 0` | `box 1 1 1 \| 0.5 0.5 0.5 0 0 1 0` (box corner AT center; 3 box vertices ON sphere) | 14.6394 | 13.3518 | 3.92699 | oracle |
| oqJ1 | bfuse_complex/J1 | `cylinder 10 20 \| 0 0 0 0 0 1 0` | `cylinder 5 20 \| 5 0 10 0 0 1 0` (axis offset 5 = R1-R2: INTERNAL line tangency at (10,0,z)) | 2199.11 | oracle | oracle | oracle |

### Class G — grazing (cells 37–45)

| pair | OCCT src | A | B | fuse | cut | common | tuc |
|---|---|---|---|---|---|---|---|
| oqC5 | bop*_simple/C5 | `box 1 1 1 \| 0.5 0.5 0.5 0 0 1 0` | `box 1.41421356 0.25 1 \| 0.41161165 0.58838835 0.5 0 0 1 45` (slab side-plane passes through BOTH diagonal corners) | 6.41789 | 7.03921 | 3.61764 | 1.83211 |
| oqH5 | bop*_simple/H5 | same A | `box 1.5 0.44721360 1 \| 0.58695050 0.04347524 0.5 0 0 1 26.56505118` (atan(1/2) slab pivoting on corner (1,0,0)) | 8.11803 | 5.11803 | 3.11803 | 5.11803 |
| oqZP9 | bopcommon_simple/ZP9 | `box 100 100 100 \| 50 50 50 0 0 1 0` | `sphere 7.5 \| 98.946735014 46.491265177 17.092869659 0 0 1 0` (center 1.053 inside the x=100 wall — grazing cap, dirty decimals) | oracle | oracle | 576.293 | oracle |

### Class W — thin walls / cavities (cells 46–51)

| pair | OCCT src | A | B | fuse | cut | common | tuc |
|---|---|---|---|---|---|---|---|
| oqK5 | bop*_simple/K5 | `box 1 1 1 \| 0.5 0.5 0.5 0 0 1 0` | `box 0.5 1.5 1 \| 0.5 0.5 0.5 0 0 1 0` (pierces both y-walls, BOTH z-planes coplanar: cut = 0.25-wall tunnel) | 7.5 | 6 | 4 | 3.5 |
| oqZD2 | bop*_simple/ZD2 | `cylinder 1 2 \| 0 0 0 0 0 1 0` | `cylinder 0.5 2 \| 0 0 0 0 0 1 0` (coaxial, full height: cut = tube, BOTH caps annular-coplanar) | 18.8496 | 23.5619 | 7.85398 | empty |

### Reserves (blocked or needing harness work — port after PORT MAP rows 4–5 land)

- **oqZF6** (bop*_simple/ZF6): truncated cone `pcone 1 0.5 2` strictly inside `box 4 4 4`, TOP circle ON
  the top face — fuse 96 / cut 108.071 / common 13.6418 / tuc empty. Blocked: create_cone(r1,r2,h).
- **oqZM5** (ZM5): coaxial truncated cones `pcone 8 4 8` + `pcone 2 1 4 @ z=2` — fuse 588.519 /
  cut 643.086 / common 54.5673 / tuc empty. Blocked: same.
- **oqZC5** (ZC5): box(1.4^3) rotated -45 about axis (-1,1,0) through (0,0,-0.2) resting tilted on
  `cylinder 1 2` — fuse 19.7221 / cut 27.1316 / common 10.8874 / tuc 3.47797. Blocked: arbitrary-axis xf_of.
- **oqZL5** (ZL5, seam twin of existing oCYL48/oTOR41 cell): torus rotated 270 about z — identical
  expecteds to ZL2 (342.851 / 317.718 / 116.656 / 141.789); seam-invariance probe for the torus path.
- **oqZP8** (bop*_simple/ZP5+ZP8): `box 10 10 10` minus centered `box 4 4 4 @ (2,2,2)…(6,6,6)` -> cut is a
  CAVITY solid (12 faces, 2 shells); fuse of hull-with-void = 600 exactly. Blocked: our combine/classify
  has no inner-shell (void) support — this is the multi-shell frontier cell.
- **gdmlA1** (gdml_public/A1 core step): nested boxes `762x1270x889` / `635x1143x787.4` co-centered,
  inner rotated `6.2725381128105878e-46` deg about x — cut = thin shell (walls 50.8–63.5) under a
  denormal near-identity rotation. Oracle-only references. Blocked: needs 2-step chain (cut then fuse ladder).
- **gdmlB1** (gdml_public/B1): box rotated 120 deg about body diagonal (-1,-1,-1)/sqrt3 (axis permutation)
  then 90 about y, fused flush with a co-extent box — rotated-frame full-face coincidence, the exact
  class of our chairsROT failures. Blocked: two-rotation composition (fold to single axis-angle via
  the PITFALLS formula, then arbitrary-axis xf_of).
- **halfspace rows** (bopcut_simple/ZP3–ZP6): box vs `halfspace` — no half-space primitive in our
  kernel; DEFER (equivalent coverage via big-box substitution if ever needed).

---

## PORT MAP (OCCT mechanism -> our anchor -> action)

| # | OCCT mechanism | our anchor | action | design (1 line) |
|---|---|---|---|---|
| 1 | `bop` fills one `BOPAlgo_PaveFiller` (BOPTest_BOPCommands.cxx:150), 4 ops via `bopsmt`:195 reuse it | `brep_section.cpp build_section_scaffold` re-run per op | ADOPT | compute scaffold/SSI-chains once per PAIR, hand the same section network to fuse/cut/common (extends the landed shared-closure bridge, task #5) |
| 2 | `end`-script `checkshape result` on every case + parse.rules `/\bFaulty\b/` | naked-edge audit in main_7 scorecard | ADOPT | promote naked==0 + closed-winding to a HARD per-cell verdict bit for the whole SESSION_OCCT3 grid, not a printed diagnostic |
| 3 | `checkprops` sprops area gate, depsilon 1e-2 (CheckCommands.tcl:543-548) | oracle vol+nfaces gate (`occt(...)` main_7.cpp:245) | ADOPT+EXTEND | add `AREA` line to oracle.cpp output (BRepGProp::SurfaceProperties, ~3 lines) + area column; check invariant `S(fuse)+S(common)==S(A)+S(B)` for free |
| 4 | trotate about arbitrary point/axis (DRAW) ; oracle already general (`gp_Ax1` oracle.cpp:114) | `xf_of()` main_7.cpp:194 principal-axis-only | REPLACE | use existing `Xform::rotation(Vector&,deg,true)` (xform.h:85) for non-principal axes; fold rotate-about-point via `t' = R(t-c)+c` at cell-authoring time |
| 5 | `pcone r1 r2 h` truncated cone (solid) | `BRep::create_cone(r,h)` full cone only; oracle solid `cone` 2-param (oracle.cpp:298) | NEW-BUILD | add create_cone(r1,r2,h) x3 langs + oracle `SHAPE cone3 r1 r2 h`; unblocks ZF6/ZM5/ZP7 truncated-tangency reserves |
| 6 | seam-twin cases (ZD4/ZD5, ZL2/ZL5): operand rotated about own axis, expecteds identical | whole-seg alias keys + UV arrangement in `brep.cpp split_with` | ADOPT | run both twins in the grid and diff OUR results against each other — a self-referential seam-aliasing detector needing no oracle |
| 7 | `-s empty` exact-zero cells (A5/F5/Z5 tuc, ZD4 common) | micro-piece filter + winding contains in combine | ADOPT | gate: empty-expected cells must yield ZERO faces (not epsilon-slivers) — direct regression harness for the sliver filter |
| 8 | cavity results (ZP5/ZP8: cut -> solid with inner shell, fuse-with-void 600) | `combine` (exact weld + tube merge + NK-RESCUE) single-shell assumption | NEW-BUILD | inner-shell support: orient the second closed component as void (negative winding) instead of dropping/merging it — frontier item, reserve cell ready |
| 9 | `BOPAlgo_CellsBuilder` (`bcbuild`/`bcadd`, BOPTest_CellsCommands.cxx:52; cells_test 68 cases) | no anchor (our ops are pairwise) | REFERENCE | future general-fuse/selective-cells op; mine cells_test only after multi-operand splitting exists |
| 10 | `BOPAlgo_MakerVolume` (`mkvolume`, volumemaker 74 face-soup cases at 1e6 extents) | `split_with` UV arrangement + combine | REFERENCE | large-extent face-arrangement stress corpus for the UV-arrangement stage; not pair-cell material |
| 11 | gdml_public boolean CHAINS with arbitrary-axis rotated coincident operands | `occt2_chains()` main_7.cpp:184 | NEW-BUILD | add gdmlA1/gdmlB1 as ChainCells once PORT-MAP-4 lands; they are the minimal inline reproduction of the chairsROT rotated-coincidence class |

---

## EXECUTION ORDER FOR THE MINING PLAN

1. Land PORT MAP 4 (xf_of arbitrary axis — 3 lines) and 3 (oracle AREA — 3 lines).
2. Add `occt3_placements()/occt3_pairs()` with the 17 core pairs above; `SESSION_OCCT3=1 main_7.exe --refresh`
   to fill the cache; record area cross-check vs the OCCT `-s` column at 1 %.
3. Wire invariant gates: seam-twin diff (row 6), empty==zero-faces (row 7), area conservation (row 3).
4. Land PORT MAP 5 (truncated cone), promote ZF6/ZM5; then ZC5 + gdml chains (row 11).
5. Attack the cavity frontier (row 8) with oqZP8 as the acceptance cell.
