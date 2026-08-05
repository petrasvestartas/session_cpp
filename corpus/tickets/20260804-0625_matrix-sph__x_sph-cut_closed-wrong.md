# matrix/sph__x_sph/cut — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | sph  x sph  |
| op | cut |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 48.064731354707 |
| truth solids | None (gated: None) |
| our vol | 48.0737 |
| our solids / valid | 1 / None |
| rel vol err | 0.0001865951403497057 |
| kernel faces / naked | 2 / None |
| runtime | 0.5 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=cut ./build/main_7 'sph  x sph'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `sph  x sph cut |     48.0737     48.0647  1.86e-04 |    2     2 | 1 |    29907 | FAIL`
