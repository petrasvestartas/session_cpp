# matrix/tor__x_tor/fuse — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | tor  x tor  |
| op | fuse |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 43.99583473948817 |
| truth solids | None (gated: None) |
| our vol | 26.3352 |
| our solids / valid | 1 / None |
| rel vol err | 0.4014160623173035 |
| kernel faces / naked | 6 / None |
| runtime | 269.0 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=fuse ./build/main_7 'tor  x tor'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `tor  x tor fuse |     26.3352     43.9958  4.01e-01 |    6     6 | 1 |  5147661 | FAIL`
