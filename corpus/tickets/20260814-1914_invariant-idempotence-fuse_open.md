# invariant/idempotence/fuse — open (invariant violation)

| field | value |
|---|---|
| battery | chairs_base |
| config | base |
| op | fuse |
| verdict | **open** (baseline: n/a (invariant)) |
| truth vol | 80.296861869 |
| truth solids | 1 (gated: False) |
| our vol | -22.092908988000005 |
| our solids / valid | 0 / 0 |
| rel vol err | 1.2751403787615436 |
| kernel faces / naked | 273 / 67 |
| runtime | 168.8 s |

## Minimal repro (cwd session_cpp)
```bash
env SESSION_CHAIRS=/home/petras/code/code_rust/session/session_cpp/corpus/work/idem SESSION_OP=fuse ./build/main_7 zzzz
```

Check the exported result:
```bash
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step>        # ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
/home/petras/code/code_rust/session/validation/step_probe/build/step_probe <result.step> -c     # per-subshape BRepCheck failures
```

A-op-A idempotence broken: with B an exact copy of A, fuse must give vol 80.2969 with 20 faces but gave vol -22.0929 with 273 faces (vol tol 0.4015). A zero volume alone does not clear it: an open shell also measures zero. Repro dir /home/petras/code/code_rust/session/session_cpp/corpus/work/idem holds chair1.stp = chair0.stp; this is the same-domain (P3) regime — every face pair is exactly coincident.
