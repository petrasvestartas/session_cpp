# occt_suite.h — OCCT boolean test-suite harvest

Battery of `OcctSuiteCell` entries harvested from OCCT's own draw-harness boolean
regression scripts (`occt/tests/boolean/*` and `occt/tests/bugs/modalg_1..8`), scanned at
`C:\brg\compas_occt\external\occt\src\occt\tests` (identical copy under
`validation/occt_oracle/build/deps/occt/src/occt/tests`).

## Counts

| stage | count |
|---|---|
| files scanned (boolean/* incl. 2d + bugs/modalg_1..8) | 6651 |
| translatable files | 951 |
| harvested cells (one per boolean op; some files yield several) | 958 |
| unique geometry configs among cells | 476 |
| **selected into OCCT_SUITE** | **120** |

Skipped by reason (files): imported .brep/.rle/.gdml data 3290, 2d-only dirs 684,
no boolean op (section/splitter/volumemaker/mkconnected/…) 553, tcl scripting
(`$var`/`[expr]`/loops in build commands) 447, unsupported builders (prism, revol,
profile, wire, blend, wedge, compound operand, `bbuild` general fuse) 275,
truncated cone `pcone r1 r2 h` with both radii nonzero 214, boolean-of-boolean chains 89,
non-axis-aligned rotation composites 80, multi-argument `bbop` (N objects/tools) 31,
partial (angle-limited) primitives 29, plane-frame with non-axis normal 5, bool options 3.

Translatable files per dir: bop{common,cut,fuse,tuc}_simple 222 each (same geometry grid
under 4 ops), bcommon/bcut/bfuse_simple 9 each, bfuse_complex 3, gdml_public 2, history 2,
periodicity 1, modalg_4 2, modalg_5 5, modalg_6 2, modalg_7 17, modalg_8 2.

Selected 120 by pair: box-cyl 41, cyl-cyl 26, box-box 19, cyl-sph 13, cyl-tor 9,
box-sph 9, sph-sph 2, box-cone 1. By op: cut 64 (incl. boptuc swaps), fuse 34, common 22.
Emphasis: line/point tangencies, coincident faces (flush caps, base-on-face, face-to-face),
seam-crossing configs, containment/void results, same-domain (A==B), 1e-4 sliver gaps,
grazing rotated operands. Near-identical parameterizations deduped (see seam note below).

## Draw → Place conversion rules

Everything is done at the 4x4 matrix level: for each shape a matrix `M` accumulates
`M := M_cmd * M` per Tcl command (column-vector convention, same as `Xform`), then the
final `M` must decompose as `T(t) * R(single origin axis x|y|z, deg)` — exactly what
`main_7.cpp xf_of()` builds. If the linear part is not an exact (1e-8) single-axis
rotation the case is rejected (`non-axis-rotation`).

Canonical placements (verified against `brep.cpp`):

| OCCT | ours | compensation |
|---|---|---|
| `box dx dy dz` (corner at origin) | `create_box` centered | pre-fold `T(dx/2,dy/2,dz/2)` |
| `box x y z dx dy dz` (corner at x,y,z) | centered | pre-fold `T(x+dx/2, y+dy/2, z+dz/2)` |
| `pcylinder r h` base z=0..h, +Z | `create_cylinder` identical | none |
| `pcone r 0 h` base z=0, apex z=h | `create_cone` identical | none |
| `pcone 0 r h` apex z=0, base z=h | — | pre-fold `T(0,0,h) * R_x(180)` |
| `psphere r` centered | `create_sphere` identical | none |
| `ptorus R r` centered, axis +Z | `create_torus` identical | none |

- `ttranslate dx dy dz` → `M := T(v) * M`.
- `trotate x y z dx dy dz a` (axis through point p=(x,y,z), direction d, degrees, both
  conventions right-handed) → `M := T(p) * R(d,a) * T(-p) * M`, i.e. the equivalent
  origin form is `T(p - R p) * R`. Negative/unnormalized axes normalized; the folded
  translation comes straight out of the final matrix.
- Multiple rotations are allowed if the *product* is single-axis: e.g. gdml_public/B1
  composes 120 deg about the body diagonal with 90 deg about y — the product is exactly
  `R_x(-90)`; matrix-level decomposition catches such collapses automatically.
- Plane-framed primitives `p* name pln ...` (from `plane x0 y0 z0 nx ny nz ux uy uz`):
  accepted when the normal is axis-aligned (snap 1e-9); the frame folds to
  `T(origin) * R(z→normal)` with one of I, R_x(±90), R_x(180), R_y(±90). The plane's
  x-direction is ignored — legal for surfaces of revolution, it only moves the seam.
  `psphere` on a plane accepts *any* normal (only the origin matters). Trailing
  full-angle args (`... 360`) accepted; any other angle = partial primitive, rejected.
- Op mapping (validated numerically against `checkprops` areas in the sources):
  `bcut r a b` = a\b → `cut`; `bfuse`/`bcommon` literal; modern `bop a b` +
  `bopcut r` = a\b, `bopfuse`, `bopcommon`; `boptuc r` = **b\a** — stored with operands
  swapped so `op` is always A-relative; `bbop r n` (after `baddobjects a; baddtools b`):
  0=common, 1=fuse, 2=cut(a\b), 3=tuc(b\a, swapped), 4=section (ignored). Only
  single-object single-tool `bbop` accepted.
- Cosmetics: angles snapped to integers when within 1e-9 (atan2 noise), translations
  |t|<1e-12 zeroed, -0.0 normalized. Sliver offsets (e.g. bug25722's 15.0001) untouched.
- `bfuzzyvalue` ignored (tolerance-only, geometry unchanged) — but note bug25722
  (1e-4 face gap, run with fuzzy 1e-4 and 0) and bug25477_1 (1e-5 gap, fuzzy 2e-5)
  *depend* on it for their expected topology: with fuzzy on, the gap welds into one
  solid; without, the fuse keeps two solids. Treat both outcomes as defensible.
  `tscale`/`tmirror`/`nurbsconvert`/`settolerance` on an operand invalidate it.

## Pitfalls found

- The box center-vs-corner mismatch is the only canonical-placement compensation needed;
  all four quadric primitives match OCCT exactly.
- z-rotation of a cylinder/sphere/torus about its own axis produces *identical geometry
  with a moved seam* — the grids contain whole families of these (ZD5–ZD7, ZH6–ZH8,
  ZL3–ZL5, ZO7–ZO8). One seam variant per family kept (still a real test for us since
  our seams also move with the rotation); the rest deduped.
- A few source scripts post-process the boolean before `checkprops` (history/* blend
  edges, bug28828_* chamfer): do not compare against those scripts' area/volume numbers;
  re-derive references from OCCT (validation/ oracle) for the raw boolean instead.
- `boptuc` direction is easy to get backwards; confirmed b\a via boptuc_simple/ZL2 area.
- bop*_simple grid cells reuse one geometry across 4 dirs (4 ops) — dedup by geometry,
  then choose ops per config, otherwise the battery is 4x redundant.

## High-value NOT-translatable categories (worth supporting later)

1. **Truncated cone** (`pcone r1 r2 h`, 214 files): the single biggest primitive gap;
   the entire ZF5..ZG* grid region (cone x box/cyl tangencies) is locked behind it.
2. **Boolean-of-boolean chains** (89 files): washer/annulus and progressive-fuse
   stress cases; main_7 `occt2_chains()` already has the mechanism — cells could carry
   a chain list instead of a single pair.
3. **Multi-argument fuse/cut** (`bbop`/`bbuild` with N operands, 31 files): general-fuse
   semantics, includes bug27274-style same-radius crossed cylinders.
4. **Prism/revol/profile solids** (bulk of `unsupported-construction`): extruded
   polygons vs primitives — first non-quadric surface class OCCT tests heavily.
5. **Angle-limited primitives** (29 files): pie-wedge cylinders/spheres — exercises
   planar-face + seam interaction on otherwise supported quadrics.
6. **Non-axis rotation composites** (80 files): would need full 9-dof rotation in
   `Place.xf` (or a quaternion) — includes the gdml_private lattice cases.
7. **Imported .brep data** (3290 files): out of scope for primitive replay; reachable
   only via the STEP/BRep reader path.

## Regeneration

Scanner/emitter: scratchpad `occt_scan.py` (classify + JSON) and `occt_emit.py`
(curated pick list → header). Verified: MSVC `/W4 /std:c++17` compile-clean; runtime
check 120/120 cells well-formed (valid kinds/ops, one-hot axes, finite doubles).
