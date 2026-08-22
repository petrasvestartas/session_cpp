# matrix/sph__x_sph/common — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | sph  x sph  |
| op | common |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 17.3851155951788 |
| truth solids | None (gated: None) |
| our vol | 17.3762 |
| our solids / valid | 1 / None |
| rel vol err | 0.000512829214737643 |
| kernel faces / naked | 3 / None |
| runtime | 0.7 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=common ./build/main_7 'sph  x sph'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `sph  x sph common |     17.3762     17.3851  5.13e-04 |    3     3 | 1 |    33086 | FAIL`
