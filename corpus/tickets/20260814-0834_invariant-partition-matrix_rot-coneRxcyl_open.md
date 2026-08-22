# invariant/partition/matrix_rot/coneRx cyl — open (invariant violation)

| field | value |
|---|---|
| battery | matrix_rot |
| config | coneRx cyl  |
| op | cut |
| verdict | **open** (baseline: n/a (invariant)) |
| truth vol | 12.30741331613601 |
| truth solids | None (gated: None) |
| our vol | 43.1452 |
| our solids / valid | 0 / None |
| rel vol err | 2.5056269657762424 |
| kernel faces / naked | 4 / None |
| runtime | 11.9 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_OP=cut ./build/main_7 'coneRx cyl'
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

Battery row: `coneRx cyl cut |     43.1452     12.3074  2.51e+00 |    4     3 | 0 |  8497909 | FAIL`

Representative cell: cut (largest residual). Partition identity broken on our own results: cut+common-A = +52.7801, fuse-(A+B-common) = -3.7685 (tol 0.1676). vol(A)=16.7552 vol(B)=42.4115; our cut=43.1452 common=26.3901 fuse=29.0081. This is oracle-independent — at least one of the three ops on this config is wrong regardless of what OCCT says.
