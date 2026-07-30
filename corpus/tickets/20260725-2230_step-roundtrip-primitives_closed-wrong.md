# STEP round-trip breaks booleans on ALL analytic primitives — blocks the T0R tier

| field | value |
|---|---|
| battery | t0r (rotated-primitive tier) |
| severity | **blocker** — every T0R cell fails on this before rotation is ever exercised |
| kernel | `corpus/work/main_7_snapshot` sha1 3de30de13d70 (build/main_7 @ 2026-07-25 21:14) |
| op | cut / common / fuse (all) |
| verdict | closed-wrong / open, depending on cell |

## The finding

A solid built **in memory** booleans exactly. The **same solid written to STEP and read
back** booleans to garbage. Rotation is not involved — a pure axis-aligned translation is
enough.

Control (in-memory, matrix battery, `main_7 "box  x box "`):

```
box  x box    cut  |  60.0000  60.0000  2.25e-15 | 11 11 | 1 | OK
box  x box    common |  4.0000   4.0000  1.78e-15 |  6  6 | 1 | OK
box  x box    fuse |  68.0000  68.0000  1.67e-15 | 11 11 | 1 | OK
```

Same geometry class through STEP (box 4³ vs itself translated +1 on x, no rotation):

```
chairs cut   : faces 2 solid 0 naked 8 vol 5.3333      (truth 32.0000, 6 faces)
chairs common: faces 6 solid 0 naked 8 vol 48.0000     (truth 32.0000)
chairs fuse  : faces 6 solid 0 naked 8 vol 69.3333     (truth 96.0000)
```

The operands are read correctly — the kernel prints
`A: faces 6 solid 1 vol 64.0000 | B: faces 6 solid 1 vol 64.0000`, so this is not a
reader volume/topology failure; it is the boolean on the reconstructed representation.

## It is not the writer, and not one surface type

Same +1 translation, cut, operands written by **both** writers (`ours` = kernel result,
`occt` = truth):

| operand source | box | sph | cyl | cone | tor |
|---|---|---|---|---|---|
| kernel's own STEP (`SESSION_STEP_PRIMS`) | 5.33 / 32.00 | 19.37 / 37.18 | 17.66 / 33.12 | 7.52 / 13.07 | 0.00 / 18.73 |
| OCCT-written STEP (`step_probe --prim`) | 5.33 / 32.00 | 0.00 / 37.18 | 0.00 / 33.12 | 7.08 / 13.07 | 0.00 / 18.73 |

Every primitive type fails, from both writers. Torus results are EMPTY. Naked-edge
counts on the results run 1–8 while the operands themselves are clean
(`step_probe <f> -n` reports NAKED 0 / NONMANIFOLD 0 / SHELLS_OPEN 0 for all operands).

Analytic exact checks (closed form, no oracle): **24/24 violations** on axis-aligned
box×box (4 offsets × 3 ops) and sphere×sphere lens (4 distances × 3 ops), including a
**negative** volume (−18.29) and results that invent volume where the truth is empty
(common 10.67 where the solids do not overlap).

## Why it matters beyond this tier

- Freeform NURBS operands read from STEP boolean *correctly* (base chairs cut/common/fuse
  are exact, `corpus/baseline.json`). So the defect is specific to the **analytic /
  planar** representation coming out of the reader, not to STEP import in general.
- It masks the entire rotated-primitive programme: T0R drives the kernel through
  `SESSION_CHAIRS=<dir>` (the only way to feed arbitrary rotated operands without a C++
  change), so every T0R cell hits this defect first and rotation effects cannot be seen.
- The in-memory oriented battery says rotation itself is fine for planar solids
  (`box x boxR` is exact to 7.5e-16), which makes this round-trip defect, not rotation,
  the top blocker for planar geometry.

## Minimal repro (cwd session_cpp, ~2 s)

```bash
# 1. write the kernel's own box to STEP
env SESSION_STEP_PRIMS=/tmp/kp ./build/main_7 zzzz

# 2. make a pair dir: A = that box, B = same box translated +1 on x
mkdir -p /tmp/rt && cp /tmp/kp/prim_box.step /tmp/rt/chair0.stp
python corpus/step_rigid.py /tmp/kp/prim_box.step /tmp/rt/chair1.stp --deg 0 --trans 1 0 0

# 3. boolean them through the STEP entry point
env SESSION_CHAIRS=/tmp/rt SESSION_OP=cut ./build/main_7 zzzz
#    -> chairs cut : faces 2 solid 0 naked 8 vol 5.3333     (truth 32.0)

# 4. oracle truth for the same pair
../validation/step_probe/build/step_probe --cut /tmp/rt/chair0.stp /tmp/rt/chair1.stp
#    -> OP_VOLUME 32.000000000  OP_FACES 6  OP_SOLIDS 1  OP_VALID 1
```

Compare against the in-memory control, which passes:
```bash
env SESSION_NO_ROT=1 ./build/main_7 "box  x box "
```

## Suggested first look

The two paths differ only in how the operand BRep is constructed: `BRep::create_box(...)`
versus `file_step::read_file_step_breps(...)`. Volume and face count agree after the
read, so surfaces and trims are geometrically right; what differs is likely the
*representation* the splitter depends on (analytic surface recognition, pcurve presence,
trim domains, or per-face tolerances). A diff of the two BReps for the same box —
surface types, trim curve types, domains, tolerances — should localise it quickly.

## Sharper diagnostic: the answer depends on SEAM PLACEMENT, not on geometry

Rotating a rotationally symmetric solid about its own symmetry axis is the identity on
the solid — only the periodic seam moves. Through the STEP path the kernel's answer
changes anyway (`rotprim.py analytic`, oracle-free, `common` against a fixed box):

| operand rotated about its own axis | 0 deg (ref) | 17 deg | 90 deg | 143 deg |
|---|---|---|---|---|
| cylinder | 14.3923 | **77.7658** | 14.3923 | **65.9344** |
| cone | 6.6525 | **-7.8576** | **6.5586** | **10.2561** |
| torus | -67.9329 | -67.9329 | -67.9329 | **-17.4005** |
| sphere (4 axes x 3 angles) | all identical — 0 violations | | | |

The solid is unchanged in every row; only the seam parameter moves. Cylinder and cone
swing by 5x and even change sign, while the sphere is stable. That points the
investigation at seam/parameterization handling in the reconstructed analytic
representation (and note the reference values are themselves wrong, so this is
instability on top of the wrongness, not a separate bug).
