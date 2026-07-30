# HUNT — port parity of the boolean subsystem (Python / Rust vs C++)

**Measured:** 2026-07-25 22:30–22:50 CEST · working tree at `session_cpp` HEAD `5bb685a` + uncommitted
(`src/brep.cpp` +514, `src/brep_section.cpp` +105 vs HEAD; `brep_samedomain.*`, `brep_commonblock.*`,
`main_10.cpp` untracked). Files were being edited by other agents during the run — the boolean file sizes
below are a snapshot at 22:48, not a stable value. Every number here came from the real files or a real
test run; nothing is estimated unless labelled.

**Reproduce:** commands in §6.

---

## 0. EXECUTIVE ANSWER

1. **The ports were last mirrored on 2026-07-01.** Since then C++'s boolean subsystem grew by ~8,400 lines
   across 28 commits; Python's `brep.py` received **+4 lines** and Rust's `brep.rs` **+5**, both from one
   unrelated `refresh_guid` commit. The boolean subsystem is now **11,924 C++ lines vs 1,021 Python and
   1,121 Rust — 8.6% / 9.4% coverage.** Four C++ files of the subsystem (`brep_section`, `brep_samedomain`,
   `brep_commonblock`, and the `main_7`/`main_10` batteries) have **no counterpart of any kind** in either port.

2. **The "1,531 C++ tests vs 757" premise is a measurement artifact.** Measured: **C++ 760, Python 739,
   Rust 713**. 1,531 = 760 (the binary's own aggregate) + 771 (the sum of the per-class lines printed
   *after* it) — the aggregate was double-counted. Same arithmetic gives Rust 713+716=1,429. The real
   parity gap is **21 tests (cpp→py) and 47 (cpp→rust)**, and *most of it is not boolean*.

3. **Python's boolean tests exist, pass, and never run.** `session_py/src/session_py/brep_test.py` defines
   six boolean tests (lines 608–740) plus one volume test with **no `@MINI_TEST` decorator** — they are
   dead code the harness never sees. I ran all seven manually: **7/7 PASS** in 75 s. This is the rot
   signature: not broken code, *unobserved* code.

4. **The mirroring rule, as written in this repo, does not reach the splitter internals.** Every artifact
   that defines it (`CLAUDE.md`, `/new-class`, `/port`, `/sync`, `/test-rules`) is written in terms of a
   *class*: six files (`X.py`, `X.rs`, `X.h`+`X.cpp`, and three minitests), a public method list, JSON field
   order, and matching test names. `/sync`'s entire file list is those six paths. `brep_section.cpp`,
   `brep_samedomain.cpp` and the 108 `SESSION_*` gates are not classes and have never been in scope.
   **The obligation is the public API surface and the test suite. It always was.**

5. **Recommendation: option (iii) — declare the splitter a C++-only subsystem with a frozen public API and
   a *mirrored assertion set*.** Not because mirroring is expensive (it is: ~14,500 lines of new port code),
   but because **it is physically impossible in Python**: measured C++ box×cyl boolean 6.68 ms vs Python
   ~5.0 s — **~750×**. The freeform chairs case that this entire campaign targets costs C++ 401–600 s per
   op; Python would need **83–125 hours per op**. A mirrored Python splitter could never be run, therefore
   never be tested, therefore would be pure liability. See §4.

---

## 1. CURRENT DIVERGENCE — MEASURED

### 1.1 Timeline: when the ports stopped tracking

`brep.cpp` line count by commit (via `git show <sha>:src/brep.cpp | wc -l`) against the same for
`brep.py` / `brep.rs` in their submodules:

| date | `session_cpp/src/brep.cpp` | `session_py/.../brep.py` | `session_rust/src/brep.rs` |
|---|---|---|---|
| 2026-02-24 (first) | 1,355 | 1,046 | 1,358 |
| 2026-05-31 | 1,447 | 1,250 | 1,706 |
| 2026-06-16 | **1,806** | 1,571 | 2,087 |
| 2026-06-18 | 1,996 | 1,860 | — |
| **2026-07-01** | **3,706 – 3,742** | **3,423** | **3,777** | ← last real mirror
| 2026-07-16 | 6,854 – 7,212 | 3,427 (+4) | 3,782 (+5) |
| 2026-07-25 HEAD | 11,592 | 3,427 | 3,782 |
| 2026-07-25 working | **12,106** | 3,427 | 3,782 |

Commits touching the boolean subsystem since 2026-07-01:

| repo | commits total | commits touching the BRep file | net lines added to it |
|---|---|---|---|
| session_cpp (`brep.cpp` + `brep_section.cpp` + `brep_samedomain.cpp`) | — | **28** | **+8,400** |
| session_py (`brep.py`) | 29 | **1** (`b21c98b`, `refresh_guid`) | **+4** |
| session_rust (`brep.rs`) | 15 | **1** (`552b4df`, `refresh_guid`) | **+5** |

Parity held to within ~8% until 2026-07-01 and has been abandoned in fact since. `/sync`'s own tripwire —
*"line count divergence > 10% (may indicate missing logic)"* — has been at **253%** (py) / **220%** (rust)
on `brep` alone for 24 days without producing a signal, because nothing runs `/sync`.

### 1.2 File inventory

| file | C++ | Python | Rust |
|---|---|---|---|
| `brep.{cpp,h}` / `brep.py` / `brep.rs` | 12,106 + 508 | 3,427 | 3,782 |
| `brep_section.{cpp,h}` (scaffold + shared-edge pool) | 2,608 + 94 | **absent** | **absent** |
| `brep_samedomain.{cpp,h}` (SD detector + BuildBOP table) | 720 + 251 | **absent** | **absent** |
| `brep_commonblock.{cpp,h}` (untracked) | 293 + 142 | **absent** | **absent** |
| `mesh_boolean.cpp` / `.rs` | 237 | **absent** | 351 |
| `brep_test` | 851 | 744 | 809 |
| `main_7.cpp` (63+54+22+120-cell batteries) | 1,467 | **absent** | **absent** |
| `main_10.cpp` (same-domain harness, 59/59) | 677 | **absent** | **absent** |
| `corpus/` (T0 runner, invariants, ledger, tickets) | 2,240 py | n/a | n/a |

### 1.3 Function-level divergence inside the boolean pipeline

Line spans measured from definition line to next definition line.

| function | C++ | Python | Rust | ratio |
|---|---:|---:|---:|---|
| `boolean()` — imprint → classify → select → sew + AUTO ladder | **3,120** | 156 | 161 | **20×** |
| `split_with()` — the UV arrangement / splitter | **1,582** | 167 (`_split`) | 197¹ | **9×** |
| `co_refine_coincident_edges()` | **918** | 247 | 267² | 3.7× |
| `sew_coincident_edges()` | **406** | 99 | 103 | 4.1× |
| `merge_coplanar_faces()` | **367** | — | — | ∞ |
| `normalize_section_blocks()` | **349** | — | — | ∞ |
| `split_by_brep()` | 293 | 27 | 24 | 11× |
| `make_shared_section_edges()` | **263** | — | — | ∞ |
| `recognize_solid` + `inside_prim` + `srf_is_planar` (+`PrimSolid`) | 246 | 185 | 214 | 1.3× |
| `imprint_edges()` | 235 | 140 | 155 | 1.7× |
| `sameparameter_planar_pcurves()` | **90** | — | — | ∞ |
| `snap_section_edges()` | **79** | — | — | ∞ |
| `recover_section_spans()` | **77** | — | — | ∞ |
| **subtotal, in-file** | **8,025** | **1,021** | **1,121** | |
| `brep_section` + `brep_samedomain` + `brep_commonblock` | **+3,899** | 0 | 0 | |
| **TOTAL boolean subsystem** | **11,924** | **1,021** | **1,121** | |
| **port coverage** | 100% | **8.6%** | **9.4%** | |

¹ Rust: `split_with` 89 + `lift_loop` 48 + `find_or_add_vertex` 11 + `append_face` 49.
² Rust: `co_refine_coincident_edges` 257 + `coref_cand` 10.

### 1.4 Public methods present in C++, absent from the ports

`BRep::` methods in `brep.cpp` vs `def`/`fn` in the ports:

| method | py | rust | boolean-relevant? |
|---|---|---|---|
| `normalize_section_blocks` | ✗ | ✗ | **yes — scaffold block normalisation** |
| `recover_section_spans` | ✗ | ✗ | **yes — scaffold span recovery** |
| `make_shared_section_edges` | ✗ | ✗ | **yes — shared-edge minting** |
| `snap_section_edges` | ✗ | ✗ | **yes** |
| `sameparameter_planar_pcurves` | ✗ | ✗ | **yes — OCCT `BRepLib::SameParameter` analog** |
| `merge_coplanar_faces` | ✗ | ✗ | **yes — post-boolean face unification** |
| `contains_point_exact` | ✗ | ✗ | **yes — Law 5 exact classifier seed** |
| `face_outward_signs` | ✗ | ✗ | **yes — orientation/classification** |
| `loop_material_left` | ✗ | ✗ | **yes** |
| `check_trim_orientation` | ✗ | ✗ | **yes — validity diagnostic** |
| `topology_report` | ✗ | ✗ | **yes — the naked/nonman verdict accessor** |
| `append_brep` | ✗ | ✗ | yes (assembly) |
| `split_with` (public in C++, private in ports) | private | private | yes |
| `deep_copy_from` | ✗ | ✗ | no (idiom) |
| `jsondump` / `jsonload` / `str` / `repr` | ✗¹ | ✗¹ | no (naming idiom) |

¹ Python uses `__jsondump__`/`__str__`; Rust uses serde + `Display`. These are idiomatic, not gaps.

**Also absent from both ports:** `boolean_xor()` and `boolean_split()` (declared `brep.h:386,397`) —
2 public API methods, not internals. And the entire `SectionSegment` / `SectionScaffold` /
`SharedEdgePool` / `SDOp` / `SDState` / `SDVerdict` / `SDEdgeSig` / `SDFaceKey` / `SDFaceRec` /
`SDPairRec` / `SameDomain` type family.

### 1.5 Do the ports have *any* boolean implementation?

**Yes — a complete one for the primitive/legacy path, none for the freeform/scaffold path.**

- Both ports implement `boolean(op)`, `boolean_union`, `boolean_difference`, `boolean_intersection`,
  `split_by_brep`, `split_by_plane/surface/curves/line`, `split_by_plane_pieces`, `subset`,
  `imprint_edges`, `co_refine_coincident_edges`, `sew_coincident_edges`, `recognize_solid`, `inside_prim`.
- Both **lack** the branch that all 2026-07 work went into. C++ `brep.cpp:8400,8432`:
  ```cpp
  bool imported_freeform = (prA0.kind == 0 && prB0.kind == 0);
  bool scaffold_eligible =
      ((imported_freeform && (has_freeform(*this) || has_freeform(other))) || s_scaffold_all)
      && !s_scaffold_off;
  ```
  Neither port has `has_freeform`, `s_scaffold_all`, `build_section_scaffold`, or any scaffold branch.
- **108 `SESSION_*` environment gates** exist in the C++ boolean files. Python has 2 `SESSION_` mentions,
  Rust has 1. The experimental control surface is entirely C++.
- Python additionally cannot reach the target class at all: it has **no STEP importer** (`file_step` exists
  in C++ only; Rust has none either — see §2.3). The chairs corpus is STEP-only.

**Verdict on 1.5:** the ports are not stubs and they are not rotten. They are a faithful mirror of
`brep.cpp` as of 2026-07-01, and the 8,400 lines added since sit entirely inside a branch the ports'
inputs cannot enter.

---

## 2. MINITEST PARITY — MEASURED

All three suites ran to completion (exit 0) despite machine load. Logs retained in the session scratchpad.

### 2.1 Headline totals

| suite | command | binary's own aggregate | sum of per-class lines | **true total** | wall |
|---|---|---:|---:|---:|---|
| Python | `./bash/minitest.sh --py --no-web` | *(none printed)* | 739 | **739** | ok |
| Rust | `./bash/minitest.sh --rust --no-web` | 713/713 | 716 | **713** | ok |
| C++ | `./bash/minitest.sh --cpp --no-web` | 760/760 | 771 | **760** | ok |

**They do not match: 760 / 739 / 713.**

### 2.2 The 1,531 figure is a double-count — and the discrepancies are stale files

`bash/lib/common.sh:103` `print_class_summary()` **globs `*_test.json`** in the output directory and reports
every file it finds, whether or not a test produced it this run. `bash/test_cpp.sh:93` and
`bash/test_rust.sh:53` call it; `bash/test_py.sh` does not (Python iterates `CLASS_NAMES` instead). So:

- **C++:** 771 − 11 stale = **760**. Stale: `session_tests/session_cpp/elementfeature_test.json` (10 tests,
  mtime 16:19 vs 22:34 for everything else) and `reciprocal_test.json` (1). `CMakeLists.txt:312` says
  `# src/elementfeature_test.cpp  # removed: source file does not exist (stale reference)` — the source is
  gone, the JSON is not, and the summary still reports 10 passing tests that do not exist.
- **Rust:** 716 − 3 stale (`file_step_test.json`, mtime 16:19) = **713**. There is no `file_step.rs`
  anywhere in `session_rust/src/`.
- **Python:** 739, clean (driven by `CLASS_NAMES`, so `session_tests/session_py/file_step_test.json`'s
  4 stale entries are silently excluded).

`760 + 771 = 1531` exactly. `713 + 716 = 1429`. The reported "1,531 C++ tests" is the aggregate plus the
per-class sum plus 11 phantom tests.

**This is a live measurement-integrity bug, not just a reporting nit:** a deleted C++ test module keeps
reporting green forever.

### 2.3 Per-class divergence (canonical parity set)

`bash/lib/common.sh:5` defines the canonical parity contract: **45 class names**. Restricted to those 45:

| language | classes present | tests |
|---|---:|---:|
| C++ | 44 / 45 (missing `io`) | **751** |
| Python | 45 / 45 | **739** |
| Rust | 45 / 45 | **713** |

Diverging classes (canonical set only; `MISS` = no JSON produced):

| class | cpp | py | rust | note |
|---|---:|---:|---:|---|
| **mesh** | 52 | 48 | **22** | **largest hole in the project — 30 tests, not boolean** |
| **brep** | **33** | **24** | 30 | see 2.4 |
| primitives | 32 | 32 | 27 | |
| nurbscurve | 14 | 13 | 11 | |
| nurbssurface | 21 | 20 | 20 | |
| intersection | 49 | 49 | 50 | |
| io | MISS | 3 | 4 | cpp ships `io_xyz` (3, non-canonical) instead |
| remesh_cdt | 12 | 11 | 11 | |
| remesh_nurbssurface_adaptive | 10 | 10 | 9 | |
| vector | 21 | 22 | 22 | |

Non-canonical extras: C++ runs `file_step` (6) and `io_xyz` (3); Rust and Python run neither.

**Note the shape of this:** the single worst parity hole is `mesh` in Rust (21 registered vs C++ 52,
`session_rust/src/mesh_test.rs` 1,633 lines vs `mesh_test.cpp` 2,010). Parity erosion is *not* a
boolean-specific phenomenon — the boolean subsystem is just the loudest instance.

### 2.4 `BRep` test divergence — the actual boolean parity gap

C++ registers 33 `BRep` tests, Rust 32 (30 counted; `ZZ NxN Boolean Matrix` and one more are name-only),
Python 24.

| test | cpp | py | rust |
|---|:-:|:-:|:-:|
| `Boolean` (box×cyl, 3 ops, exact vol + OCCT timing bench) | ✔ | ✘ | ✘ |
| `Boolean Sphere Split` | ✔ | **dead** | ✔ |
| `Boolean Contained Sphere` | ✔ | **dead** | ✔ |
| `Boolean Example brep_booleans` | ✔ | **dead** | ✔ |
| `Boolean Off-Center Cyl` | ✔ | **dead** | ✔ |
| `Boolean Contained Box` | ✔ | **dead** | ✔ |
| `Boolean Box-Box` | ✔ | **dead** | ✔ |
| `Contains Point` | ✔ | ✘ | ✔ |
| `ZZ NxN Boolean Matrix` (env-gated `SESSION_MATRIX`) | ✔ | ✘ | ✘ |
| `Block With Hole Volume` | ✘ | **dead** | ✔ |

**"dead" = the function exists in `brep_test.py` but has no `@MINI_TEST` decorator, so the harness never
registers it.** Seven such functions, lines 595, 608, 631, 655, 674, 697, 717.

I executed all seven directly:

```
PASS  test_brep_block_with_hole_volume            ( 2.7s)
PASS  test_brep_boolean_example_brep_booleans     (15.4s)
PASS  test_brep_boolean_offcenter_cylinder        (14.9s)
PASS  test_brep_boolean_contained_box             ( 1.1s)
PASS  test_brep_boolean_contained_sphere          ( 5.4s)
PASS  test_brep_boolean_box_box                   ( 3.3s)
PASS  test_brep_boolean_sphere_split              (32.3s)
```

**7/7 pass, with the identical assertions and identical OCCT reference constants as C++** (face counts
7/3/10, volumes to 1e-6 relative, `is_solid()`). Rust's registered equivalents also pass — I diffed
`run_brep_boolean_box_box` and `run_brep_boolean_sphere_split` against the C++ bodies: same operands, same
constants, same tolerances.

So: **Python's boolean is correct and unobserved. Rust's is correct and observed. Neither has drifted
numerically.** The likely reason Python's are undecorated is cost — 75 s is ~1/3 of the whole Python suite.

### 2.5 Test *assertion* counts (a second, independent parity measure)

| | `MINI_CHECK` occurrences in `*_test` files |
|---|---:|
| C++ | 3,991 |
| Python | 3,825 |
| Rust | 3,740 |

Same ordering, ~6% spread — consistent with §2.3 and much tighter than the 253% spread on `brep` source.

---

## 3. THE REAL OBLIGATION — WHAT THE REPO ACTUALLY SAYS

I read every artifact that defines the mirroring rule. There are five.

| artifact | what it scopes |
|---|---|
| `CLAUDE.md` | *"C++ is ground truth — port to Rust and Python with identical APIs, variable names, test logic, line counts"*. Then eight "Minitest Rules" — **all about tests**: identical names/logic/line count, one test per **API method**, JSON field order, `file_json_dump`/`to_proto` per class, method order constructors→accessors→mutators→operators→utilities→serialization. |
| `.claude/commands/new-class.md` | *"Create a new geometry class in all 3 languages"*. Names exactly 4 impl files + 3 minitest files + 3 registration points. "Required API (all classes)": ctor, guid/name, `duplicate`, `clone`, `[]`, `==`, `!=`, `str`/`repr`, json, proto. |
| `.claude/commands/port.md` | *"Port a **class** implementation from Python to Rust and/or C++"*. Enforced rules: identical variable/method/test names, `p[0]` not `p.x`, explicit for-loops in tests, alphabetical JSON, *"Line count: match **Python** density"*. |
| `.claude/commands/sync.md` | *"Audit cross-language parity for **class** $ARGUMENTS. Read all **6 files**"* — `X.py`, `X_minitest.py`, `X.rs`, `X_minitest.rs`, `X.h`+`X.cpp`, `X_minitest.cpp`. Divergence checks: missing **methods**, test names, test **counts**, variable names, coordinate access, JSON order, import location, *"Line count divergence > 10%"*. |
| `.claude/commands/test-rules.md` | 9 core rules + 3 language templates. Every rule is about **test files**. |

**Three findings:**

1. **The unit of the rule is a class, and its surface is the public API + the tests.** Nothing in any of
   the five artifacts mentions free functions, anonymous namespaces, helper structs, algorithm internals,
   or environment gates. `/sync` — the only *parity audit* tool in the repo — cannot even see
   `brep_section.cpp` or `brep_samedomain.cpp`: they are not `<class>.{h,cpp}` for any class in
   `CLASS_NAMES`, so they fall outside its six-file world by construction.

2. **The tooling assumes Python-first, which is the structural cause of the drift.** `CLAUDE.md` says
   *"Dev order: Python → Rust → C++"* and `/port.md` says *"Port a class implementation **from Python**"*
   and *"match **Python** density"* — while `CLAUDE.md`'s style section says *"C++ is ground truth"*.
   The boolean kernel was developed C++-first, against an OCCT oracle, in a direction the repo has **no
   command for**. There is no `/port --from-cpp`. The rule was never violated so much as *unimplementable*.

3. **The distinction is already recognised.** `kb/DECISION_architecture.md` (2026-07-25), Step 0 item 5:
   > *"Ten minutes, do it now: add to CLAUDE.md that `brep_section`, `brep_samedomain`, `BRep::boolean`
   > internals and `file_step` are an **explicitly C++-only subsystem with a frozen public API and a
   > mirrored assertion set**. Python's boolean is 156 lines against C++'s ~3,370; the drift is a month old.
   > Make it a recorded decision instead of an accumulating rule violation."*

   My measurements confirm its arithmetic (Python `boolean()` = **156** lines exactly; C++ `boolean()` =
   **3,120** lines, 3,366 including `recognize_solid`/`inside_prim`/`srf_is_planar`/`PrimSolid`).

**Answer to Q3: the mirroring obligation covers `brep.boolean_difference(other)` and its tests. It does
not, and never did, cover the splitter internals.** Mirroring a 12,000-line splitter is a commitment
nobody in this repo ever made in writing.

---

## 4. RECOMMENDATION

### 4.1 The cost arithmetic, corrected

The panel's framing ("3× cost on a 5,000-delete / 1,750-write refactor") does not survive contact with the
measurements, because **the ports never received the 5,000 lines that are slated for deletion.** All of
them live in `split_with`'s scaffold branch, `boolean()`'s classification/AUTO architecture,
`brep_section.cpp`, and the repair stack — none of which exist in Python or Rust.

| policy | Python lines to write | Rust lines to write | total |
|---|---:|---:|---:|
| (ii) mirror continuously, starting now | back-fill 10,903 + mirror every experiment | back-fill 10,803 + same | **21,706 + churn** |
| mirror only the *post-refactor* end state (~8,400 C++ lines) | ~7,380 | ~7,280 | **~14,660** |
| (i) freeze now, mirror once when stable | 0 today; ~7,380 later | 0 today; ~7,280 later | deferred 14,660 |
| (iii) C++-only subsystem, frozen API + mirrored assertions | **~120** (see 4.4) | **~120** | **~240** |

### 4.2 The argument that settles it: Python physically cannot run this algorithm

Measured, same machine, same session:

| case | C++ | Python | ratio |
|---|---:|---:|---:|
| box(4) × cyl(r=1), one `boolean_difference` (`[bool-time]`, N=20 warm) | **6.68 ms** | ~5.0 s¹ | **~750×** |
| box(4) × sphere(2.5) cut | **50.4 ms** | ~11 s² | ~215× |
| chairs freeform fuse (the actual target class) | **401–600 s**³ | **83–125 hours** (extrapolated) | — |

¹ `test_brep_boolean_offcenter_cylinder` = 14.9 s for 3 ops. ² `test_brep_boolean_sphere_split` = 32.3 s
for split + 2 ops. ³ `kb/DECISION_architecture.md`: `chairs_base/fuse ∈ {401 s, 508 s, 600 s-timeout}`.

A mirrored Python splitter could not be executed on the corpus that defines correctness for this
subsystem. It could therefore never be validated, and an unvalidated 8,000-line numerical algorithm is
strictly worse than no algorithm — it looks like coverage and provides none. **Option (ii) is not
expensive; it is void.** Rust could physically run it, but Rust-alone mirroring breaks the symmetry the
rule exists to preserve and doubles the debugging surface of an *architecturally unstable* design
(108 live experiment gates, 28 commits in 24 days, an AUTO ladder the DECISION doc shows is optimising a
metric that cannot see the failure).

### 4.3 Verdict on the three options

**(i) Freeze the ports' boolean surface and mirror once C++ stabilises** — *rejected as stated, adopted in
part.* Correct instinct, wrong terminal state: it still promises a 14,660-line mirror later, which §4.2
shows Python can never validate. And "freeze" without a written boundary is what has been happening
since 2026-07-01 by default — an accumulating unrecorded violation whose only visible symptom so far has
been six silently-undecorated Python tests.

**(ii) Mirror continuously** — *rejected.* 21,706 lines of port code, tracking a design the repo's own
architecture review says will delete 5,000–5,500 of its C++ lines, in a language that cannot execute the
result. Every experiment (there were 28 in 24 days) would cost 3×.

**(iii) C++-only subsystem, frozen public API, mirrored assertion set** — **adopt.** This is also
`DECISION_architecture.md` Step 0 item 5, and my measurements support it on every axis: the ports already
implement the frozen API; they already pass its assertions; the divergence is entirely below the API line;
and Python cannot host the implementation at any price.

### 4.4 The policy, concretely

**Frozen public API** (ports must implement, keep name-identical, and test — the C++ signatures are the
contract; `brep.h` lines given):

```
boolean(other, op, tol)                       h:298      ← ports have it
boolean_union / _difference / _intersection   h:370-378  ← ports have it
boolean_xor / boolean_split                   h:386,397  ← MISSING in both ports (gap to close)
split_by_plane / _surface / _curves / _line   h:220-229  ← ports have it
split_by_brep / split_by_plane_pieces         h:240,283  ← ports have it
subset / append_brep                          h:276,280  ← append_brep MISSING in both
imprint_edges / sew_coincident_edges /
  co_refine_coincident_edges / merge_coplanar_faces      ← merge_coplanar_faces MISSING in both
is_solid / is_valid / volume / contains_point / face_count / edge_count / vertex_count
topology_report(bool* valid_manifold)         h:157      ← MISSING in both (highest-value gap, see §5)
```

**Explicitly C++-only, exempt from the mirroring rule** (record in `CLAUDE.md`):

```
src/brep_section.{h,cpp}       SectionSegment, SectionScaffold, SharedEdgePool,
                               build_section_scaffold, refine_scaffold_at_breaks,
                               build_shared_edge_pool
src/brep_samedomain.{h,cpp}    SDOp/SDState/SDVerdict/SDEdgeSig/SDFaceKey/SDFaceRec/
                               SDPairRec/SameDomain
src/brep_commonblock.{h,cpp}
BRep::split_with, normalize_section_blocks, recover_section_spans,
  make_shared_section_edges, snap_section_edges, sameparameter_planar_pcurves,
  face_outward_signs, loop_material_left, contains_point_exact, check_trim_orientation
all 108 SESSION_* gates
main_7.cpp, main_10.cpp, corpus/, validation/
src/file_step.* (C++-only until a port has a consumer)
```

**Mirrored assertion set** (the ports' real obligation — ~120 lines each):

1. Re-register Python's 7 dead tests (7 decorator lines; they pass today — §2.4).
2. Add `Boolean` (box×cyl 3-op exact) to Python and Rust — the one boolean test C++ has and neither port
   does. ~25 lines each.
3. Add `topology_report()` returning `(naked, nonmanifold, shells, valid_manifold)` to `brep.py` and
   `brep.rs` (~40 lines each) and assert `naked == 0 && nonman == 0` in every boolean test in all three
   languages. This is the single highest-value port task available: it converts every existing boolean
   test from "face count + volume" to "face count + volume + **watertightness verdict**", which is the
   metric `DECISION_architecture.md` Step 0 is replacing `metric()` with anyway.
4. Add `boolean_xor`, `boolean_split`, `append_brep`, `merge_coplanar_faces` to both ports (~50 lines
   each language) with one test apiece — closing the *public API* gap, which is the part of the rule that
   is genuinely binding.
5. Fix `print_class_summary` (`bash/lib/common.sh:103`) to iterate `CLASS_NAMES` instead of globbing, and
   delete the 4 stale JSONs. ~5 lines. Without this, every parity number this project quotes is wrong.

**The re-mirror trigger, written down now:** when the C++ boolean subsystem goes 30 consecutive days
without a change to `boolean()`, `split_with()`, or `brep_section.cpp`, **and** T0 is green under the
Step-0 metric, re-open the port question. Not before.

### 4.5 What (iii) costs and what it risks — honestly

**Costs.**
- The ports permanently lack imported-freeform booleans. Python and Rust users get exact primitive
  booleans (validated against OCCT to 1e-6, 45/45 matrix, 54/54 edge grid) and nothing else. That is a
  real, permanent capability gap, and it must be documented in the ports' READMEs, not discovered.
- Cross-language differential testing — three independent implementations disagreeing is a genuinely
  strong oracle — is given up for the boolean path. It is *already* given up in fact (the ports run the
  2026-07-01 algorithm), so this cost is recognition of a loss, not a new one. Mitigation: `corpus/` +
  `validation/step_probe` + OCCT oracle are a stronger oracle than a Python mirror would be, and they
  already exist.
- `CLAUDE.md`'s headline rule acquires an exception. Rules with exceptions erode. Mitigation: the
  exception is a *named file list*, not a judgement call, so a future `/sync` can enforce it mechanically.

**Risks.**
- **Silent rot is the real one, and it is already here.** Six Python boolean tests sat undecorated for
  weeks; `elementfeature`'s deleted C++ module still reports 10 green tests; a stale `file_step_test.json`
  inflates two suites. Declaring a subsystem C++-only *without* §4.4 item 5 makes this worse, because the
  boundary becomes unpoliced. **The measurement fix is the load-bearing part of this recommendation, not
  an afterthought.**
- **Re-mirroring later becomes a rewrite.** Quantified: today the ports are 1,021/1,121 lines behind a
  moving 11,924-line target. If C++ lands the pave-block design (net ≈ −3,500) the gap shrinks to
  ~7,300 lines — a rewrite either way. The honest statement is that **the boolean port is already a
  rewrite, not a merge, and (iii) does not make that worse; it just stops pretending otherwise.**
- **Scope creep of the exemption.** `brep.cpp` is one file holding both the exempt subsystem (lines
  ~3,158–11,310) and the mirrored data structure/serialization (1–3,157, 11,311–12,106). The exemption is
  currently expressible only as a line range, which will drift. Mitigation, and the one structural change
  worth making: **move the exempt region out of `brep.cpp` into `brep_boolean.cpp`**, so the exemption is a
  file list. This also makes `/sync brep` meaningful again (it would compare a ~3,900-line
  `brep.{h,cpp}` against a 3,427-line `brep.py` and a 3,782-line `brep.rs` — back inside the 10% band).

---

## 5. THE TEST-PARITY SAFETY NET

### 5.1 Confirmed: zero coverage

```
grep -l 'SectionScaffold\|SharedEdgePool\|SameDomain\|scaffold' \
     session_cpp/src/*test*.cpp session_py/src/session_py/*_test.py session_rust/src/*_test.rs
→ (no matches, all three languages)
```

`brep_section.h` declares 3 structs with **26 fields** including 11 self-diagnostic counters
(`n_chains`, `n_paves_trimA/B/xing/vertex/closing`, `n_dropped_verdict/micro`,
`n_bridge_marched/welded/residual`, `max_devA/max_devB`) and 3 entry points
(`build_section_scaffold` `brep_section.cpp:405`, `refine_scaffold_at_breaks` `:2427`,
`build_shared_edge_pool` `:2583`). **Not one is asserted anywhere.** The only signal between "I changed a
data structure" and "a 132-cell corpus verdict flipped" is a 15–18-minute T0 run with ~3,000 changed lines
to bisect — and 2/132 cells flip run-to-run anyway (`DECISION_architecture.md` Step 0.3), so the signal is
noisy at the level the bug lives.

CI (`\.github/workflows/session-minitest.yml`) runs `minitest.sh` per language on 4 OSes and **nothing
else** — no `main_7`, no `main_10`, no `corpus/`. Its only boolean cells are the registered `BRep` tests,
all of which take the legacy path (`recognize_solid` names every operand). **CI coverage of the 11,924-line
boolean subsystem's scaffold branch is 0%.**

### 5.2 Minimal tripwire set — seven C++ tests

Design rule: each asserts a **structural invariant the pave-block design is supposed to guarantee**, so it
fails at the data structure, before the corpus verdict. Run under `SESSION_SCAFFOLD_ALL=1` on one cheap
deterministic freeform pair (a bicubic pillow × box — already built in `main_7`'s `SESSION_FREEFORM`
battery) plus one primitive pair, so the whole set runs in seconds, not minutes.

| # | test | asserts | catches |
|---|---|---|---|
| **T1** | **Scaffold census** | golden ints for `segments.size()`, `vertices.size()`, `n_chains`, `n_paves_trimA/B/xing/vertex/closing`, `n_dropped_verdict/micro` | any change to SSI seeding, paving, or the keep-verdict — the class that produced "97% section suppression → zero outcome change" |
| **T2** | **Chain fidelity** | for every segment and every `i`: `\|A(uvA[i]) − p3[i]\| < tol3` and `\|B(uvB[i]) − p3[i]\| < tol3`; `max_devA/max_devB` below golden bound; `p3.size() == uvA.size() == uvB.size()` | the index-correspondence invariant the entire shared-topology design rests on. If this breaks, everything downstream is meaningless and currently nothing says so |
| **T3** | **Scaffold connectivity** | every `v_start`/`v_end` in range; `closed ⟺ v_start == v_end`; `n_bridge_residual == 0`; `segs_by_surfA`/`segs_by_surfB` each contain every seg id exactly once | dangling valence-1 vertices and index-corruption in the per-surface maps |
| **T4** | **Pool identity** | `vert_tv.size() == vertices.size()`; `seg_edge.size() == segments.size()`; all `seg_edge` distinct; each pool edge's endpoints are exactly `vert_tv[v_start]`, `vert_tv[v_end]`; no two arena vertices within `tol3` | **the one property the whole design exists to deliver** — two segments meeting at a scaffold vertex reference the *same* topology vertex. Untested today |
| **T5** | **Refine fixpoint** | `refine_scaffold_at_breaks(scaf, B)` then again with the same `B` returns **0**; segment count monotone non-decreasing; total chain arclength invariant (splits move no geometry); T2 re-asserted after refine; run count under the cap | the `SESSION_SYMEMIT` blow-up (459 runs / 19k edges / 8.3 GB → `bad_alloc` on z37) and the Step-2 requirement that refinement become an invariant rather than a 2-pass env-gated loop |
| **T6** | **Verdict bridge** | after a scaffold-mode boolean on the golden pair: golden triple `(edges with 1 trim, 2 trims, ≥3 trims)`; `[P4MISS] == 0`; `face_count`, `volume`, `is_solid()` | the actual defect class (`res_y30_cut`'s 13 non-manifold edges that `metric()` at `brep.cpp:8240` cannot see) |
| **T7** | **Run determinism** | same operands twice in one process → identical `face_count`, `edge_count`, and **bitwise** `volume` | the measured 2/132 run-to-run verdict flips, localized to a unit test instead of a 15-minute T0 |

Estimated cost: **~250–350 lines of C++**, one new `brep_section_test.cpp`, seconds of runtime. It is the
cheapest artifact in this entire program and `DECISION_architecture.md` already names it *"the single
highest-value missing artifact"*.

### 5.3 Mirrorability verdict — a clean split

| test | mirrorable to py/rust? | why |
|---|---|---|
| T1, T2, T3, T4, T5 | **No — inherently C++-internal** | They dereference `SectionScaffold` / `SharedEdgePool` fields. Mirroring them *requires porting those types*, which is precisely the commitment §4 rejects. Attempting it would be the tail wagging the dog: writing 3,900 lines of Python data structures so that a test can be identical. **These must be explicitly exempted alongside the sources they test.** |
| **T6, T7** | **Yes — mirror them** | Both are expressible entirely through the frozen public API: `face_count()`, `edge_count()`, `volume()`, `is_solid()`, `topology_report()`. T6 needs `topology_report` in the ports (§4.4 item 3, ~40 lines each) — after that it is ~30 lines per language. T7 is ~15 lines per language and needs nothing new. |

This is the **"mirrored assertion set"** in operational form: **golden-structure tests stay C++-only;
golden-verdict tests mirror.** The boundary is not a judgement call — it is "does the test name a type
that exists in `brep_section.h`?"

### 5.4 Two things to do regardless of the port decision

1. **T1–T7 in C++** — no port implications, blocks Step 2 of the architecture plan, costs ~300 lines.
2. **Fix `print_class_summary`** (`bash/lib/common.sh:103`) and delete the 4 stale JSONs. Until this is
   done, "the C++ suite has N tests" is not a measurable statement in this repo, and any future
   port-parity claim inherits the error.

---

## 6. REPRODUCTION

```bash
cd /home/petras/code/code_rust/session
source uvsession/bin/activate

# suites (§2)
./bash/minitest.sh --py   --no-web        # 45 modules, 739 tests
./bash/minitest.sh --rust --no-web        # aggregate 713; per-class sum 716 (3 stale)
./bash/minitest.sh --cpp  --no-web        # aggregate 760; per-class sum 771 (11 stale)

# stale-JSON proof (§2.2)
ls -la --time-style=+%m-%d_%H:%M session_tests/session_{cpp,py,rust}/*_test.json | sort -k6
grep -n 'elementfeature' session_cpp/CMakeLists.txt          # :312 removed, JSON remains

# divergence (§1)
wc -l session_cpp/src/brep*.{cpp,h} session_py/src/session_py/brep.py session_rust/src/brep.rs
grep -c 'SESSION_' session_py/src/session_py/brep.py session_rust/src/brep.rs   # 2, 1
grep -ohE 'SESSION_[A-Z0-9_]+' session_cpp/src/brep{,_section,_samedomain}.cpp | sort -u | wc -l  # 108

# python dead tests (§2.4) — all 7 pass
python3 -c "import session_py.brep_test as b; b.test_brep_boolean_box_box(); print('PASS')"

# zero scaffold coverage (§5.1)
grep -l 'SectionScaffold\|SharedEdgePool\|SameDomain\|scaffold' \
  session_cpp/src/*test*.cpp session_py/src/session_py/*_test.py session_rust/src/*_test.rs
```

---

## 7. ONE-PARAGRAPH ANSWER

The ports stopped tracking the boolean kernel on **2026-07-01**; C++ has since added ~8,400 lines across
28 commits while Python and Rust received 4 and 5 lines respectively, both from an unrelated commit. The
boolean subsystem is **11,924 C++ lines against 1,021 Python and 1,121 Rust (8.6% / 9.4%)**, and the
missing 91% is entirely inside a branch (`imported_freeform && has_freeform`) that the ports' inputs cannot
enter — Python has no STEP importer at all. Measured suite totals are **C++ 760, Python 739, Rust 713**;
the quoted 1,531 is the C++ aggregate double-counted with its own per-class sum plus 11 phantom tests from
a deleted module whose JSON was never cleaned up. The mirroring rule, as actually written in `CLAUDE.md`
and the four `.claude/commands/` files, is a *class-shaped* rule covering public API and tests — `/sync`'s
world is six files per class and cannot even see `brep_section.cpp`. Therefore the obligation is
`brep.boolean_difference(other)` and its assertions, never the splitter. Mirror continuously and you buy
21,706 lines of port code tracking an unstable design in a language measured at **~750× slower** (C++
box×cyl 6.68 ms vs Python ~5.0 s), which would put the chairs corpus that defines correctness at 83–125
hours per operation — an implementation that could never be validated. **Adopt option (iii)**: declare
`brep_section`, `brep_samedomain`, `brep_commonblock`, `BRep::split_with`, the section/classification
helpers and the 108 `SESSION_*` gates a C++-only subsystem behind a frozen public API; move the exempt
region of `brep.cpp` into `brep_boolean.cpp` so the exemption is a file list rather than a drifting line
range; and spend ~120 lines per port on the real obligation — re-register Python's 7 silently-dead tests
(they pass, verified), add `topology_report()` so every boolean test asserts watertightness, and close the
4 genuine public-API gaps (`boolean_xor`, `boolean_split`, `append_brep`, `merge_coplanar_faces`).
Independently, write the seven golden-structure tripwire tests (T1–T7, ~300 lines) on the scaffold and
shared-edge pool, which have **zero** test references in any language today: **T1–T5 are inherently
C++-internal and must be exempted with the code they test; T6 (naked/non-manifold/volume verdict) and T7
(run determinism) go through the frozen public API and are exactly the assertions the ports should carry.**
And fix `bash/lib/common.sh:103` first — until the per-class summary stops globbing stale JSON, every
parity number this project quotes, including the one that started this investigation, is wrong.
