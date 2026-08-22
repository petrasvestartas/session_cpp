# matrix/sph__x_tor/cut — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | sph  x tor  |
| op | cut |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 45.07974161233009 |
| truth solids | None (gated: None) |
| our vol | 0.0 |
| our solids / valid | 1 / None |
| rel vol err | 1.0 |
| kernel faces / naked | 3 / None |
| runtime | 0.3 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=cut ./build/main_7 'sph  x tor'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `sph  x tor cut |      0.0000     45.0797  1.00e+00 |    3     3 | 1 |    11853 | FAIL`
