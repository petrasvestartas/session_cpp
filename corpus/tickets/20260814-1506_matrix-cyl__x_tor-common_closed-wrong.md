# matrix/cyl__x_tor/common — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | cyl  x tor  |
| op | common |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 2.259315239315205 |
| truth solids | None (gated: None) |
| our vol | 5.8858 |
| our solids / valid | 1 / None |
| rel vol err | 1.6051256139820387 |
| kernel faces / naked | 2 / None |
| runtime | 0.2 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=common ./build/main_7 'cyl  x tor'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `cyl  x tor common |      5.8858      2.2593  1.61e+00 |    2     2 | 1 |    12192 | FAIL`
