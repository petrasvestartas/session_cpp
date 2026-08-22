# matrix/cyl__x_tor/cut — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | cyl  x tor  |
| op | cut |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 40.152185584147 |
| truth solids | None (gated: None) |
| our vol | 36.5257 |
| our solids / valid | 1 / None |
| rel vol err | 0.09031851022273661 |
| kernel faces / naked | 5 / None |
| runtime | 0.2 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=cut ./build/main_7 'cyl  x tor'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `cyl  x tor cut |     36.5257     40.1522  9.03e-02 |    5     5 | 1 |    12739 | FAIL`
