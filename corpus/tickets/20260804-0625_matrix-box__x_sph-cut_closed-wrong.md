# matrix/box__x_sph/cut — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | box  x sph  |
| op | cut |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 9.545727333968934 |
| truth solids | None (gated: None) |
| our vol | 9.5336 |
| our solids / valid | 1 / None |
| rel vol err | 0.0012704462996526345 |
| kernel faces / naked | 7 / None |
| runtime | 1.2 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=cut ./build/main_7 'box  x sph'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `box  x sph cut |      9.5336      9.5457  1.27e-03 |    7     7 | 1 |    69048 | FAIL`
