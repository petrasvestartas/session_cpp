# matrix/cyl__x_cyl/common — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | cyl  x cyl  |
| op | common |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 17.99999997675719 |
| truth solids | None (gated: None) |
| our vol | 17.9962 |
| our solids / valid | 1 / None |
| rel vol err | 0.00021110982011647044 |
| kernel faces / naked | 6 / None |
| runtime | 1.4 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=common ./build/main_7 'cyl  x cyl'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `cyl  x cyl common |     17.9962     18.0000  2.09e-04 |    6     6 | 1 |    57309 | FAIL`
