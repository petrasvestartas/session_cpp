# invariant/idempotence/common — open (invariant violation)

| field | value |
|---|---|
| battery | chairs_base |
| config | base |
| op | common |
| verdict | **open** (baseline: n/a (invariant)) |
| truth vol | 80.296861869 |
| truth solids | 1 (gated: False) |
| our vol | -0.010062693999998373 |
| our solids / valid | 0 / 0 |
| rel vol err | 1.000125318645907 |
| kernel faces / naked | 205 / 56 |
| runtime | 138.5 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_CHAIRS=/home/petras/code/code_rust/session/session_cpp/corpus/work/idem SESSION_OP=common ./build/main_7 zzzz
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

A-op-A idempotence broken: with B an exact copy of A, common must give vol 80.2969 with 20 faces but gave vol -0.0101 with 205 faces (vol tol 0.4015). A zero volume alone does not clear it: an open shell also measures zero. Repro dir /home/petras/code/code_rust/session/session_cpp/corpus/work/idem holds chair1.stp = chair0.stp; this is the same-domain (P3) regime — every face pair is exactly coincident.
