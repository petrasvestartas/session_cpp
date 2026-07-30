# invariant/partition/chairs_rot/x20 — open (invariant violation)

| field | value |
|---|---|
| battery | chairs_rot |
| config | x20 |
| op | fuse |
| verdict | **open** (baseline: n/a (invariant)) |
| truth vol | 160.593388812 |
| truth solids | 1 (gated: True) |
| our vol | 126.15824491599999 |
| our solids / valid | 0 / 1 |
| rel vol err | 0.21442441778417046 |
| kernel faces / naked | 49 / 13 |
| runtime | 146.2 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_CHAIRS=serialization/boolean_steps/chairs SESSION_CHAIRS_ROT=1 SESSION_ROT_ONLY=x20 SESSION_OP=fuse SESSION_ROT_STEP=1 SESSION_FAST=1 ./build/main_7 zzzz
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Representative cell: fuse (largest residual). Partition identity broken on our own results: cut+common-A = -54.4416, fuse-(A+B-common) = -55.1124 (tol 0.8030). vol(A)=80.2969 vol(B)=80.2969; our cut=46.5322 common=-20.6770 fuse=126.1582. This is oracle-independent — at least one of the three ops on this config is wrong regardless of what OCCT says. NOTE: common volume is NEGATIVE — an inside-out shell (orientation/classification inversion), not a sewing gap; chase face orientation before naked-edge welding.
