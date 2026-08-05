# matrix/tor__x_tor/cut — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | tor  x tor  |
| op | cut |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 18.72964747315363 |
| truth solids | None (gated: None) |
| our vol | 18.7296 |
| our solids / valid | 1 / None |
| rel vol err | 2.5346528116279895e-06 |
| kernel faces / naked | 9 / None |
| runtime | 19.4 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=cut ./build/main_7 'tor  x tor'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `tor  x tor cut |     18.7296     18.7296  3.40e-06 |    9     9 | 1 |  5157560 | FAIL`
