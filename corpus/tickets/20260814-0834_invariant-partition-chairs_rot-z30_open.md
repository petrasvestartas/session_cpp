# invariant/partition/chairs_rot/z30 — open (invariant violation)

| field | value |
|---|---|
| battery | chairs_rot |
| config | z30 |
| op | cut |
| verdict | **open** (baseline: n/a (invariant)) |
| truth vol | 55.935476163 |
| truth solids | 2 (gated: True) |
| our vol | 60.37047628699999 |
| our solids / valid | 0 / 0 |
| rel vol err | 0.07928778707588154 |
| kernel faces / naked | 39 / 23 |
| runtime | 29.1 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_CHAIRS=serialization/boolean_steps/chairs SESSION_CHAIRS_ROT=1 SESSION_ROT_ONLY=z30 SESSION_OP=cut SESSION_ROT_STEP=1 SESSION_FAST=1 ./build/main_7 zzzz
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Representative cell: cut (largest residual). Partition identity broken on our own results: cut+common-A = +6.6530, fuse-(A+B-common) = -0.1647 (tol 0.8030). vol(A)=80.2969 vol(B)=80.2969; our cut=60.3705 common=26.5794 fuse=133.8496. This is oracle-independent — at least one of the three ops on this config is wrong regardless of what OCCT says.
