# invariant/partition/chairs_rot/z15 — open (invariant violation)

| field | value |
|---|---|
| battery | chairs_rot |
| config | z15 |
| op | cut |
| verdict | **open** (baseline: n/a (invariant)) |
| truth vol | 80.297280939 |
| truth solids | 2 (gated: True) |
| our vol | 43.241516828 |
| our solids / valid | 0 / 1 |
| rel vol err | 0.4614821781966741 |
| kernel faces / naked | 38 / 17 |
| runtime | 10.7 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_CHAIRS=serialization/boolean_steps/chairs SESSION_CHAIRS_ROT=1 SESSION_ROT_ONLY=z15 SESSION_OP=cut SESSION_ROT_STEP=1 SESSION_FAST=1 ./build/main_7 zzzz
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Representative cell: cut (largest residual). Partition identity broken on our own results: cut+common-A = -8.2175, fuse-(A+B-common) = -6.2651 (tol 0.8030). vol(A)=80.2969 vol(B)=80.2969; our cut=43.2415 common=28.8378 fuse=125.4908. This is oracle-independent — at least one of the three ops on this config is wrong regardless of what OCCT says.
