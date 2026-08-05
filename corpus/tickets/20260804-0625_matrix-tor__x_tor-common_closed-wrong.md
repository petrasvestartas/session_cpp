# matrix/tor__x_tor/common — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | tor  x tor  |
| op | common |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 6.536539793669854 |
| truth solids | None (gated: None) |
| our vol | 6.5366 |
| our solids / valid | 1 / None |
| rel vol err | 9.210734126419004e-06 |
| kernel faces / naked | 12 / None |
| runtime | 14.1 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=common ./build/main_7 'tor  x tor'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `tor  x tor common |      6.5366      6.5365  1.50e-05 |   12    12 | 1 |  5148920 | FAIL`
