# invariant/idempotence/common — open (invariant violation)

| field | value |
|---|---|
| battery | chairs_base |
| config | base |
| op | common |
| verdict | **open** (baseline: n/a (invariant)) |
| truth vol | 80.296861869 |
| truth solids | 1 (gated: False) |
| our vol | 0.011102690000001303 |
| our solids / valid | 0 / 0 |
| rel vol err | 0.9998617294656158 |
| kernel faces / naked | 203 / 83 |
| runtime | 591.0 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_CHAIRS=corpus/work/idem SESSION_OP=common ./build/main_7 zzzz
# corpus/work/idem holds chair1.stp == chair0.stp (A op A)
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

A-op-A idempotence broken: with B an exact copy of A, common must give 80.2969 but gave 0.0111 (tol 0.4015). Repro dir /home/petras/code/code_rust/session/session_cpp/corpus/work/idem holds chair1.stp = chair0.stp; this is the same-domain (P3) regime — every face pair is exactly coincident.
