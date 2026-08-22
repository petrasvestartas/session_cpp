# chairs_rot/z30x20/cut — open (regression)

| field | value |
|---|---|
| battery | chairs_rot |
| config | z30x20 |
| op | cut |
| verdict | **open** (baseline: exact) |
| truth vol | 54.258043908 |
| truth solids | 1 (gated: True) |
| our vol | 54.237096811 |
| our solids / valid | 0 / 0 |
| rel vol err | 0.000386064360070092 |
| kernel faces / naked | 34 / 1 |
| runtime | 11.3 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_CHAIRS=serialization/boolean_steps/chairs SESSION_CHAIRS_ROT=1 SESSION_ROT_ONLY=z30x20 SESSION_OP=cut SESSION_ROT_STEP=1 SESSION_FAST=1 ./build/main_7 zzzz
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```
