# matrix/box__x_sph/common — closed-wrong (regression)

| field | value |
|---|---|
| battery | matrix |
| config | box  x sph  |
| op | common |
| verdict | **closed-wrong** (baseline: exact) |
| truth vol | 54.45427266606967 |
| truth solids | None (gated: None) |
| our vol | 54.4664 |
| our solids / valid | 1 / None |
| rel vol err | 0.00022270674708481722 |
| kernel faces / naked | 7 / None |
| runtime | 1.2 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_NO_ROT=1 SESSION_OP=common ./build/main_7 'box  x sph'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `box  x sph common |     54.4664     54.4543  2.22e-04 |    7     7 | 1 |    64798 | FAIL`
