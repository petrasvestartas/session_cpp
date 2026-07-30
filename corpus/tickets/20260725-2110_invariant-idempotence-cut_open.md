# invariant/idempotence/cut — open (invariant violation)

| field | value |
|---|---|
| battery | chairs_base (A-op-A) |
| config | B := exact copy of chair0.stp |
| op | cut |
| verdict | **open** (must be EMPTY) |
| truth vol / faces | 0.0000 / **0 faces** |
| our vol | 0.0000 (result contributes nothing: file volume 160.5942 == volA+volB) |
| our topology | **68 result faces in 65 open shells, 59 naked edges** |
| exported file | `corpus/work/idem/chair0_cut_chair1.step` (2 solids A+B + 65 shells) |
| runtime | ~530 s |

## What is wrong
`A cut A` must return the empty shape. The kernel instead returns a soup of 65 open
shells (68 faces, 59 naked edges) whose enclosed volume happens to be zero — so a
volume-only gate passes it. This is the pure same-domain (P3) case: every face of A is
exactly coincident with a face of B, every intersection is a whole-face overlap.

Zero volume is NOT the acceptance criterion here; face count 0 is.

## Minimal repro (cwd session_cpp)
```bash
# corpus/work/idem holds chair1.stp == chair0.stp (byte-identical copy)
env SESSION_CHAIRS=corpus/work/idem SESSION_OP=cut ./build/main_7 zzzz
```
Expected kernel line: `chairs cut : faces 0 ...`; actual: 68 faces of sliver shells.

## Inspect
```bash
../validation/step_probe/build/step_probe corpus/work/idem/chair0_cut_chair1.step
../validation/step_probe/build/step_probe corpus/work/idem/chair0_cut_chair1.step -n   # EDGES/NAKED/SEAM/SHARED
../validation/step_probe/build/step_probe corpus/work/idem/chair0_cut_chair1.step -c   # BRepCheck per subshape
```

## Companion cells (same repro dir, other ops)
- `common`: 203 faces / 83 naked / vol 0.0111 — must be A itself (20 faces, 80.2969).
- `fuse`: TIMEOUT at 600 s — must be A itself (20 faces, 80.2969).

All three together say the same thing: exact face-on-face coincidence is not recognised
as same-domain, so the splitter emits duplicate coincident patches instead of a shared
domain, and classification then keeps/discards them inconsistently.
