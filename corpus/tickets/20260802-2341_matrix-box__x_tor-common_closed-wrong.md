# matrix/box__x_tor/common — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | box  x tor  |
| op | common |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 15.48108012629832 |
| truth solids | None (gated: None) |
| our vol | 15.4807 |
| our solids / valid | 1 / None |
| rel vol err | 2.4554249136237454e-05 |
| kernel faces / naked | 5 / None |
| runtime | 18.3 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=common ./build/main_7 'box  x tor'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `box  x tor common |     15.4807     15.4811  2.52e-05 |    5     5 | 1 |  5985881 | FAIL`
