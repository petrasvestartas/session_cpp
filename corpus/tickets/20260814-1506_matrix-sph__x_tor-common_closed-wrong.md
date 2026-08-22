# matrix/sph__x_tor/common — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | sph  x tor  |
| op | common |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 20.37010533745728 |
| truth solids | None (gated: None) |
| our vol | 18.1 |
| our solids / valid | 1 / None |
| rel vol err | 0.11144298469988402 |
| kernel faces / naked | 2 / None |
| runtime | 0.3 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=common ./build/main_7 'sph  x tor'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `sph  x tor common |     18.1000     20.3701  1.11e-01 |    2     2 | 1 |    11536 | FAIL`
