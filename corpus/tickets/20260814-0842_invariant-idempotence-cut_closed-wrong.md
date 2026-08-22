# invariant/idempotence/cut — closed-wrong (invariant violation)

| field | value |
|---|---|
| battery | chairs_base |
| config | base |
| op | cut |
| verdict | **closed-wrong** (baseline: n/a (invariant)) |
| truth vol | 0.0 |
| truth solids | 1 (gated: False) |
| our vol | 0.0005154190000098424 |
| our solids / valid | 0 / 0 |
| rel vol err | None |
| kernel faces / naked | 68 / 13 |
| runtime | 44.8 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_CHAIRS=/home/petras/code/code_rust/session/session_cpp/corpus/work/idem SESSION_OP=cut ./build/main_7 zzzz
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

A-op-A idempotence broken: with B an exact copy of A, cut must give vol 0.0000 with 0 faces but gave vol 0.0005 with 68 faces (vol tol 0.4015). A zero volume alone does not clear it: an open shell also measures zero. Repro dir /home/petras/code/code_rust/session/session_cpp/corpus/work/idem holds chair1.stp = chair0.stp; this is the same-domain (P3) regime — every face pair is exactly coincident.
