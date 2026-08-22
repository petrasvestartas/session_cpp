# matrix/cone_x_tor3/cut — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | cone x tor3 |
| op | cut |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 13.71413527117791 |
| truth solids | None (gated: None) |
| our vol | 12.7789 |
| our solids / valid | 1 / None |
| rel vol err | 0.06819498660943145 |
| kernel faces / naked | 4 / None |
| runtime | 0.2 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=cut ./build/main_7 'cone x tor3'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `cone x tor3 cut |     12.7789     13.7141  6.82e-02 |    4     4 | 1 |    25834 | FAIL`
