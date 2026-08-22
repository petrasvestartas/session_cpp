# matrix/box__x_tor/fuse — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | box  x tor  |
| op | fuse |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 73.78510714049007 |
| truth solids | None (gated: None) |
| our vol | 63.3666 |
| our solids / valid | 1 / None |
| rel vol err | 0.1412006778095853 |
| kernel faces / naked | 16 / None |
| runtime | 59.3 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=fuse ./build/main_7 'box  x tor'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `box  x tor fuse |     63.3666     73.7851  1.41e-01 |   16    16 | 1 |  5656857 | FAIL`
