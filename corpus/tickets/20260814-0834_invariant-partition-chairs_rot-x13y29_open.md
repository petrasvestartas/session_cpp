# invariant/partition/chairs_rot/x13y29 — open (invariant violation)

| field | value |
|---|---|
| battery | chairs_rot |
| config | x13y29 |
| op | cut |
| verdict | **open** (baseline: n/a (invariant)) |
| truth vol | 48.473352771 |
| truth solids | 1 (gated: True) |
| our vol | 46.691924895000014 |
| our solids / valid | 0 / 1 |
| rel vol err | 0.03675066349166912 |
| kernel faces / naked | 38 / 16 |
| runtime | 15.7 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_CHAIRS=serialization/boolean_steps/chairs SESSION_CHAIRS_ROT=1 SESSION_ROT_ONLY=x13y29 SESSION_OP=cut SESSION_ROT_STEP=1 SESSION_FAST=1 ./build/main_7 zzzz
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Representative cell: cut (largest residual). Partition identity broken on our own results: cut+common-A = -3.0950, fuse-(A+B-common) = +0.1024 (tol 0.8030). vol(A)=80.2969 vol(B)=80.2969; our cut=46.6919 common=30.5099 fuse=130.1862. This is oracle-independent — at least one of the three ops on this config is wrong regardless of what OCCT says.
