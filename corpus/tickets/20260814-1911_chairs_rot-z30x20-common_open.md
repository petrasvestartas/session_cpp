# chairs_rot/z30x20/common — open (regression)

| field | value |
|---|---|
| battery | chairs_rot |
| config | z30x20 |
| op | common |
| verdict | **open** (baseline: exact) |
| truth vol | 26.039392788 |
| truth solids | 1 (gated: True) |
| our vol | 26.0602952 |
| our solids / valid | 0 / 0 |
| rel vol err | 0.0008027227120914573 |
| kernel faces / naked | 21 / 5 |
| runtime | 10.8 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_CHAIRS=serialization/boolean_steps/chairs SESSION_CHAIRS_ROT=1 SESSION_ROT_ONLY=z30x20 SESSION_OP=common SESSION_ROT_STEP=1 SESSION_FAST=1 ./build/main_7 zzzz
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```
