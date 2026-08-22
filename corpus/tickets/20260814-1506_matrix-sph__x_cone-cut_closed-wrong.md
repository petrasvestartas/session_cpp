# matrix/sph__x_cone/cut — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | sph  x cone |
| op | cut |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 49.53230613988762 |
| truth solids | None (gated: None) |
| our vol | 8.9545 |
| our solids / valid | 1 / None |
| rel vol err | 0.8192189966945821 |
| kernel faces / naked | 4 / None |
| runtime | 0.3 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=cut ./build/main_7 'sph  x cone'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `sph  x cone cut |      8.9545     49.5323  8.19e-01 |    4     4 | 1 |    28153 | FAIL`
