# edge/sph_tanext/cut — closed-wrong (regression)

| field | value |
|---|---|
| battery | edge |
| config | sph tanext  |
| op | cut |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 65.44984694978737 |
| truth solids | None (gated: None) |
| our vol | 0.0 |
| our solids / valid | 1 / None |
| rel vol err | 1.0 |
| kernel faces / naked | 1 / None |
| runtime | 0.1 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_EDGE=1 SESSION_NO_ROT=1 SESSION_OP=cut ./build/main_7 'sph tanext'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `sph tanext cut |      0.0000     65.4498  1.00e+00 |    1     1 | 1 |     1256 | FAIL`
