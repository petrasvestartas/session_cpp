# matrix/sph__x_tor/fuse — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | sph  x tor  |
| op | fuse |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 70.34592887911884 |
| truth solids | None (gated: None) |
| our vol | 22.9961 |
| our solids / valid | 1 / None |
| rel vol err | 0.6730997746932011 |
| kernel faces / naked | 4 / None |
| runtime | 0.3 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=fuse ./build/main_7 'sph  x tor'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `sph  x tor fuse |     22.9961     70.3459  6.73e-01 |    4     4 | 1 |    12097 | FAIL`
