# session_cpp Agent Guide

C++ geometry kernel and the ground truth of the project. One of three parallel implementations (`session_cpp`, `session_py`, `session_rust`) sharing protobuf schemas in `session_proto` and a Vue test viewer in `session_tests`.

## Goal

- Keep this the reference implementation: `session_py` and `session_rust` are ports of what lands here.
- Keep public headers documented with Doxygen comments.
- Keep every class covered by minitests.

## Scope

These instructions apply to the whole `session_cpp` repository. Preserve unrelated working-tree changes and keep patches focused on the requested task.

## Cross-Language Parity

- C++ is ground truth. A change to a public method here is not finished until the same change exists in `session_py` and `session_rust` with the same method name, parameter names, and parameter order. If the task is C++-only, say so explicitly in the summary.
- Serialization names are fixed across languages: `file_json_dump` / `file_json_load` / `file_json_dumps` / `file_json_loads`, `pb_dump` / `pb_load` / `pb_dumps` / `pb_loads`, `to_proto` / `from_proto`.
- JSON fields are written in alphabetical order in all three languages.
- Method order in a class: constructors → accessors → mutators (`*_self`) → operators → utilities → serialization → `str` / `repr`.
- Do not add a C++-only convenience overload to a shared class without deciding what Python and Rust do instead — an overload set that only exists here is divergence.

## Headers and Types

- Target C++23 (`CMAKE_CXX_STANDARD 23`). Every header starts with `#pragma once` and everything lives in `namespace session_cpp`.
- Pass and return objects by `const&`; return `const&` from trivial accessors so callers do not copy. Return by value only when the result is a new object.
- Mark accessors `const`. Only the `*_self` mutators and setters modify state.
- `guid` is a `mutable std::string _guid` minted lazily on first read, so a copy keeps identity semantics identical to Python and Rust.
- Prefer `double` for all coordinate and tolerance values — the kernel is double-precision throughout, and mixing `float` silently changes results.
- Use `std::vector<Point>` and friends directly at API boundaries; do not invent typedefs that the other two languages cannot mirror.
- Forward-declare (`class Polyline;`) instead of including a header when only a reference or pointer is needed — it keeps compile times down and breaks include cycles.
- Never `#include "tolerance.h"` in a **header**: it leaks the dependency into every translation unit that includes it. Implementation files include it where they genuinely use `TOLERANCE.format_number` or a `Tolerance::` constant (13 `.cpp` files do), and minitests include it freely. Never hardcode an epsilon literal instead.
- Use the streaming operators (`std::cout << point`) rather than printing coordinates by hand.

## Runtime-Behavior Preservation

- Refactors must not change valid-input behavior, return types, ordering, orientation, side effects, or error behavior.
- Before changing an implementation, compare old and new control flow and identify any equivalence that depends on data-structure invariants.
- When a change requires a small, non-obvious change to a function body, add one short comment explaining which contract or invariant it preserves.
- Keep diffs narrow. Do not fold speculative architecture changes into documentation or cleanup patches.
- Use names that reflect cardinality: a returned collection is `points`, not `point`.
- No debug printing in library code. Comment only what is non-obvious.

## Build and Layout

- `src/*.cpp` is globbed into `session_core` automatically — a new production source file needs no CMake edit.
- `MINITEST_SOURCES` in `CMakeLists.txt` is an explicit list: **every new `*_test.cpp` must be added there**, or its tests silently never run.
- `src/v2/` is built by the separate `session_v2` target and is deliberately fenced out of `session_core` by regex filters. Do not "fix" those filters by moving v2 sources into `src/`.
- Configure and build: `cmake -B build -DENABLE_PROTOBUF=ON -DCMAKE_BUILD_TYPE=Release && cmake --build build --parallel 4`.

## Tests and Validation

- Run the smallest relevant selection first, then broaden:
  - `../bash/quicktest.sh <class> --cpp` — one class
  - `../bash/minitest.sh --cpp --no-web` — all C++ minitests
  - `../bash/minitest.sh` — all three languages plus the viewer on `localhost:8769`
- CI configures with `-DENABLE_PROTOBUF=ON`, builds Release, and runs `build/point_minitest` (`build/Release/point_minitest.exe` on Windows) on Linux, macOS ARM64, and Windows. Keep tests platform-agnostic — no hardcoded path separators, no assumptions about locale or `size_t` width.
- Always run `git diff --check` on changed patches.
- Minitest conventions (identical test names and logic across languages, one test per API method, operators tested inside the constructor test, `file_json_*` and `to_proto`/`from_proto` tests for every class, one object per line in collections) are documented in the parent repo's `/test-rules` command — follow them exactly.
- The constructor test covers: default and parameterized construction, `operator[]`, `==`, `!=`, `str()`, `repr()`, in-place and copy operators, and `duplicate()` with a fresh GUID.
- Test file header order: `#include "mini_test.h"`, the class header, other geometry headers, `#include "tolerance.h"`, then standard library headers; `using namespace session_cpp::mini_test;` before the `namespace session_cpp {` block.
- Test artifacts write to `serialization/` and `session_tests/session_cpp/`; regenerated artifacts are untracked — do not commit them.
- Prefer behavioral assertions over assertions about implementation details.

## Documentation

- Public classes get a Doxygen block: `/** @class Name @brief one line */`.
- Public members and fields get a trailing `///< description`.
- Non-trivial free functions and methods get a `///` summary line; add `@param` / `@return` only where the meaning is not obvious from the signature.
- Section banners (`////////... // Constructors`) separate constructors, operators, utilities, and serialization in both header and implementation — keep the same section order in both, and the same order the other two languages use.
- Document non-obvious setter side effects — in particular when setting one axis of a `Plane` or `Xform` normalizes it or recomputes another axis to preserve orthonormality.
- Docs build from `session_docs/build_docs.sh`.

## Public API

- Do not expose implementation-only payload, encoder, or helper types without a clear public use case.
- Preserve existing convenience APIs during refactors unless the task explicitly includes a deprecation or breaking-change plan.
- Removing a parameter or method is part of a task only when explicitly requested; then update header, implementation, tests, and the Python/Rust counterparts together rather than leaving hidden aliases.

## Git

- Never add Claude or any AI as git author, contributor, or co-author.
- Push all submodules with `../bash/git_push.sh "message"`.
- Check CI with `gh run list --limit 5`; inspect failures with `gh run view <id> --log-failed`.
