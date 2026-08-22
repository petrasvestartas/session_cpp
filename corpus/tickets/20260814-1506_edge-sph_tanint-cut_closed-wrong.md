# edge/sph_tanint/cut — closed-wrong (regression)

| field | value |
|---|---|
| battery | edge |
| config | sph tanint  |
| op | cut |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 61.26105674500096 |
| truth solids | None (gated: None) |
| our vol | 4.1888 |
| our solids / valid | 1 / None |
| rel vol err | 0.9316237717309404 |
| kernel faces / naked | 2 / None |
| runtime | 0.2 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_EDGE=1 SESSION_NO_ROT=1 SESSION_OP=cut ./build/main_7 'sph tanint'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `sph tanint cut |      4.1888     61.2611  9.32e-01 |    2     2 | 1 |     1678 | FAIL`
