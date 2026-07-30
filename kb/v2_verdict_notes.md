# The shared verdict harness — how to call it, and what it is validated to do

INC 1 of `kb/PLAN_v2_execution.md`. 2026-07-26.

**Replace your own scoring code with this.** Five measurement errors in one session, three of
which inverted a headline conclusion, came from per-agent verdict metrics that disagreed with
the kernel's own validity rule. There is now one implementation and it has a validation gate.

---

## 1. Call it

```cpp
#include "src/v2/v2_verdict.h"          // from a main_*.cpp at the repo root
#include "v2_verdict.h"                 // from another file inside src/v2/

using session_cpp::v2v::v2_verdict;     // NOT `using namespace` -- v2sol::V2Verdict is a
                                        // different struct and the two would be ambiguous
const auto v = v2_verdict(result);
if (!v.closed()) std::printf("%s\n", v.str().c_str());
```

Fields:

| field | meaning |
|---|---|
| `faces`, `shells`, `solids` | `shells` = components of faces joined across 2-use edges; `solids` = positive-volume shells (0 unless `closed()`) |
| `naked_real` | edges with ONE face use, **excluding zero-length degenerates** |
| `nonmanifold` | edges with more than two face uses |
| `seam_edges` | used twice by ONE face's wires — manifold, **not** naked |
| `degenerate`, `orphan` | zero-length edges; edge records with no use at all (both excluded, as `is_solid()` excludes them) |
| `closure_residual` | `|sum of outward face vector areas| / area`, from `brep_massprops`. Exactly 0 for a boundary that closes geometrically |
| `volume`, `area`, `volume_valid` | `brep_massprops`; `volume_valid` = `closed() && converged` |
| `closed()` | `naked_real==0 && nonmanifold==0 && closure_residual < 1e-9` (plus: at least one face and one edge, which is what `is_solid()` requires) |

`v2_verdict_topology_only(b)` skips the quadrature for cheap inner-loop debugging. It leaves
`closure_residual` at its 1.0 sentinel, so `closed()` is always false — **never** report a
success from it.

Cost: one `brep_massprops` run per call, `rel_tolerance` 1e-10 under a 4M-evaluation budget
(`v2_verdict_options()`, overridable via the two-argument overload). All of `main_17` — 5
shapes x (agreement + volume + 10 motions + a negative control) — runs in **0.62 s**.

`closed()` is strictly stronger than `is_solid()`: `is_solid()` is topological only, so a shell
whose faces do not actually meet can satisfy it. That is deliberate and is exercised by the
gate below.

---

## 2. Validation results — `main_17`, real run, 5/5 PASS

`build_j/main_17`, exit 0, `[V17] TOTAL pass=5 fail=0 -- harness VALIDATED, may be used to score`.

**Agreement with `is_solid()` on five un-split solids** — all five AGREE, all `naked_real=0`,
`nonmanifold=0`:

| shape | faces | seam | degen | closure residual | volume |
|---|---|---|---|---|---|
| box 2x2x2 | 6 | 0 | 0 | 0.00e+00 | 8 |
| sphere r=1 | 1 | 1 | 0 | 6.64e-18 | 4.188790204786 |
| cone r=1 h=2 | 2 | 1 | 0 | 4.38e-17 | 2.094395102393 |
| cylinder r=1 h=2 | 3 | 1 | 0 | 2.40e-17 | 6.283185307180 |
| torus R=3 r=1 | 1 | 2 | 0 | 1.95e-18 | 59.217626406536 |

**Volume vs closed form**: worst relative error **2.4e-16** across the five (box 1.1e-16,
sphere 2.1e-16, cone 0, cylinder 0, torus 2.4e-16).

**Invariance under 10 arbitrary rigid motions each (50 cells)**: 10/10 for every shape. Every
count identical; worst volume drift **9.99e-16** relative; worst closure residual **8.74e-17**.

**Negative control** — one face removed from box / cone / cylinder: `naked_real` = 4 / 1 / 1,
`closed()` false, `volume_valid` false, residual 2.0e-1 / 4.5e-1 / 2.0e-1. Detected in all
three. (Sphere and torus are single-face; nothing to remove.)

**Degeneracy branch, forced**: the five primitives above carry **no zero-length edge record** —
their poles and apexes are pcurve singularities with no 3D edge, so they do *not* exercise the
degeneracy branch. Do not assume otherwise when reasoning about your own shapes. The gate
forces it separately: the open box's four naked edges are collapsed to points, after which
`naked_real` 4 -> 0, `degenerate` 0 -> 4, and `is_solid()` returns true — while `closed()`
stays **false**, because the closure certificate still sees the hole (residual 2.0e-1).

---

## 3. Build isolation — you are now free to break v2

`src/v2/` is compiled by its own `session_v2` OBJECT target. `CMakeLists.txt` globs `src/*.cpp`
**non-recursively**, so `session_core` never sees v2; two explicit `list(FILTER ... EXCLUDE)`
lines are a second fence in case a v2 source ever reappears under `src/`.

Proven, not asserted (a file with a syntax error placed at `src/v2/_break_probe.cpp`):

- `main_7` deleted and rebuilt **from scratch** -> exit **0**, links fine
- `point_minitest` (every v1 minitest) -> exit **0**
- `session_v2` -> exit **2**, error on `_break_probe.cpp` only
- `main_17` (a v2 driver) -> exit **2**, as it must

The probe has been deleted.

This already mattered once, during INC 1 itself: minutes after the move, another session's
editor wrote `brep_v2_boolean.{h,cpp}` and `brep_v2_solid.{h,cpp}` back to `src/` from a stale
buffer. The `list(FILTER ...)` fence kept them out of `session_core`, so nothing broke; the
newest content (which was the `src/v2/` copy — the `src/` write-backs were older content with
a newer mtime, so compare content, never timestamps) was kept and the strays deleted. If you
see the configure-time warning below, do exactly that: diff, keep the superset, delete the
stray. There are deliberately no symlinks at the old paths — a stale write-back through one
would silently overwrite newer work.

Practical notes:
- The v2 glob is `CONFIGURE_DEPENDS`, so a new `src/v2/*.cpp` is picked up without a manual
  re-run of cmake. It also filters `*_test.cpp` — do not name a real v2 source that way.
- If a v2 file is mid-rewrite and does not compile, add it to `V2_EXCLUDE` in `CMakeLists.txt`
  with a dated comment instead of leaving the target broken for the other three sessions.
- No `UNITY_BUILD` on `session_v2`: several sessions edit these files at once and unity
  batching turns one session's file-local collision into everyone else's compile error.
  Namespaces (`v2sec` / `v2sf` / `v2int` / `v2sol` / `v2v`) stay mandatory regardless.
- Include paths moved: `"src/brep_v2_x.h"` -> `"src/v2/brep_v2_x.h"` from a root `main_*.cpp`;
  unqualified `"brep_v2_x.h"` still works from inside `src/v2/`.
- `main_13`..`main_17` link `session_v2`; nothing else does.
