# matrix/box__x_tor/cut — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | box  x tor  |
| op | cut |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 48.51891987397517 |
| truth solids | None (gated: None) |
| our vol | 48.5193 |
| our solids / valid | 1 / None |
| rel vol err | 7.834593717602563e-06 |
| kernel faces / naked | 7 / None |
| runtime | 18.1 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=cut ./build/main_7 'box  x tor'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `box  x tor cut |     48.5193     48.5189  8.03e-06 |    7     7 | 1 |  5710332 | FAIL`
