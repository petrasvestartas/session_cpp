# invariant/partition/chairs_rot/z45 — open (invariant violation)

| field | value |
|---|---|
| battery | chairs_rot |
| config | z45 |
| op | fuse |
| verdict | **open** (baseline: n/a (invariant)) |
| truth vol | 139.494282465 |
| truth solids | 1 (gated: True) |
| our vol | 128.50384304 |
| our solids / valid | 0 / 0 |
| rel vol err | 0.07878774119475167 |
| kernel faces / naked | 52 / 31 |
| runtime | 20.7 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_CHAIRS=serialization/boolean_steps/chairs SESSION_CHAIRS_ROT=1 SESSION_ROT_ONLY=z45 SESSION_OP=fuse SESSION_ROT_STEP=1 SESSION_FAST=1 ./build/main_7 zzzz
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Representative cell: fuse (largest residual). Partition identity broken on our own results: cut+common-A = -39.0520, fuse-(A+B-common) = -44.0691 (tol 0.8030). vol(A)=80.2969 vol(B)=80.2969; our cut=53.2241 common=-11.9792 fuse=128.5038. This is oracle-independent — at least one of the three ops on this config is wrong regardless of what OCCT says. NOTE: common volume is NEGATIVE — an inside-out shell (orientation/classification inversion), not a sewing gap; chase face orientation before naked-edge welding.
