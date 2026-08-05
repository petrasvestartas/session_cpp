# matrix/cyl__x_cyl/cut — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | cyl  x cyl  |
| op | cut |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 24.41150084670453 |
| truth solids | None (gated: None) |
| our vol | 24.4153 |
| our solids / valid | 1 / None |
| rel vol err | 0.00015562964847286862 |
| kernel faces / naked | 7 / None |
| runtime | 0.9 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=cut ./build/main_7 'cyl  x cyl'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `cyl  x cyl cut |     24.4153     24.4115  1.54e-04 |    7     7 | 1 |    64588 | FAIL`
