# matrix/sph__x_cone/fuse — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | sph  x cone |
| op | fuse |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 66.28746695903324 |
| truth solids | None (gated: None) |
| our vol | 11.4659 |
| our solids / valid | 1 / None |
| rel vol err | 0.8270276339404232 |
| kernel faces / naked | 4 / None |
| runtime | 0.2 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=fuse ./build/main_7 'sph  x cone'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `sph  x cone fuse |     11.4659     66.2875  8.27e-01 |    4     4 | 1 |    24694 | FAIL`
