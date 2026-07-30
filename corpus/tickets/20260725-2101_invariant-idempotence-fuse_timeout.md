# invariant/idempotence/fuse — timeout (invariant violation)

| field | value |
|---|---|
| battery | chairs_base |
| config | base |
| op | fuse |
| verdict | **timeout** (baseline: n/a (invariant)) |
| truth vol | 80.296861869 |
| truth solids | 1 (gated: False) |
| our vol | None |
| our solids / valid | None / None |
| rel vol err | None |
| kernel faces / naked | None / None |
| runtime | 600.1 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_CHAIRS=corpus/work/idem SESSION_OP=fuse ./build/main_7 zzzz
# corpus/work/idem holds chair1.stp == chair0.stp (A op A)
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

A-op-A idempotence broken: with B an exact copy of A, fuse must give 80.2969 but gave 0.0000 (tol 0.4015). Repro dir /home/petras/code/code_rust/session/session_cpp/corpus/work/idem holds chair1.stp = chair0.stp; this is the same-domain (P3) regime — every face pair is exactly coincident.
