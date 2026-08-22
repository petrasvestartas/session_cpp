# matrix/cyl__x_tor/fuse — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | cyl  x tor  |
| op | fuse |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 65.41837285093575 |
| truth solids | None (gated: None) |
| our vol | 36.5257 |
| our solids / valid | 1 / None |
| rel vol err | 0.4416599128316972 |
| kernel faces / naked | 6 / None |
| runtime | 0.3 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=fuse ./build/main_7 'cyl  x tor'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `cyl  x tor fuse |     36.5257     65.4184  4.42e-01 |    6     6 | 1 |    12604 | FAIL`
