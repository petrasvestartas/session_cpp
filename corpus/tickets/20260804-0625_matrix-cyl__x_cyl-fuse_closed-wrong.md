# matrix/cyl__x_cyl/fuse — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | cyl  x cyl  |
| op | fuse |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 66.82300167016635 |
| truth solids | None (gated: None) |
| our vol | 66.8268 |
| our solids / valid | 1 / None |
| rel vol err | 5.684165240593817e-05 |
| kernel faces / naked | 8 / None |
| runtime | 1.2 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=fuse ./build/main_7 'cyl  x cyl'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `cyl  x cyl fuse |     66.8268     66.8230  5.62e-05 |    8     8 | 1 |    57304 | FAIL`
