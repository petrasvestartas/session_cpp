# matrix/cone_x_tor3/common — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | cone x tor3 |
| op | common |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 3.041025547967639 |
| truth solids | None (gated: None) |
| our vol | 3.9763 |
| our solids / valid | 1 / None |
| rel vol err | 0.30755231657209137 |
| kernel faces / naked | 2 / None |
| runtime | 0.2 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=common ./build/main_7 'cone x tor3'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `cone x tor3 common |      3.9763      3.0410  3.08e-01 |    2     2 | 1 |    25448 | FAIL`
