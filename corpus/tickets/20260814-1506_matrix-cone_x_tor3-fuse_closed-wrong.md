# matrix/cone_x_tor3/fuse — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | cone x tor3 |
| op | fuse |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 38.98032253796666 |
| truth solids | None (gated: None) |
| our vol | 31.5969 |
| our solids / valid | 1 / None |
| rel vol err | 0.18941409555488506 |
| kernel faces / naked | 5 / None |
| runtime | 0.6 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=fuse ./build/main_7 'cone x tor3'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `cone x tor3 fuse |     31.5969     38.9803  1.89e-01 |    5     5 | 1 |    21468 | FAIL`
