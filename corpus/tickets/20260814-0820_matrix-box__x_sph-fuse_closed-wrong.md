# matrix/box__x_sph/fuse — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | box  x sph  |
| op | fuse |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 74.9955742837574 |
| truth solids | None (gated: None) |
| our vol | 74.9835 |
| our solids / valid | 1 / None |
| rel vol err | 0.00016099995063328685 |
| kernel faces / naked | 13 / None |
| runtime | 2.3 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=fuse ./build/main_7 'box  x sph'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `box  x sph fuse |     74.9835     74.9956  1.61e-04 |   13    13 | 1 |    67825 | FAIL`
