# matrix/sph__x_sph/fuse — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | sph  x sph  |
| op | fuse |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 81.57505299294891 |
| truth solids | None (gated: None) |
| our vol | 81.584 |
| our solids / valid | 1 / None |
| rel vol err | 0.0001096782254234595 |
| kernel faces / naked | 2 / None |
| runtime | 0.7 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=fuse ./build/main_7 'sph  x sph'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `sph  x sph fuse |     81.5840     81.5751  1.09e-04 |    2     2 | 1 |    43014 | FAIL`
