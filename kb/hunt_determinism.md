# Determinism and reproducibility audit — chairs boolean campaign

**Date:** 2026-07-25 · **Scope:** is the campaign's measurement history trustworthy?
**Binary under test:** `build/main_7_determinism`, md5 `2d4625f01c60c23ed21dcb403ed4a119`
(a frozen copy of session A's `build/main_7` as of 22:15; session A has +578 uncommitted
lines in `src/brep.cpp`, +105 in `src/brep_section.cpp`, and untracked `src/brep_samedomain.{cpp,h}`).
**Toolchain:** GCC 15.2.0, `-O3 -march=native` (CMakeLists.txt:84), Linux 7.0.0-28-generic, 32 cores, 30 GB.
**Working dir:** `/home/petras/hunt_determinism/` · **Oracle:** FreeCAD 1.1.1 headless (OCCT).

---

## VERDICT (one line)

**The kernel is deterministic; the campaign's measurement history is not, and the corruption
is bookkeeping — platform, uncommitted source, and flag drift — not run-to-run randomness.**

The trigger case resolves as **(a) a real SD effect through a path that was mis-characterised
as inert**, and the non-reproducing frontier numbers resolve as **measurements taken on
Windows with a source tree that cannot compile on Linux**. Two orthogonal confounds, neither
of them nondeterminism.

---

## 1. REPEATABILITY — measured

Protocol: **serialized** (one kernel process at a time from this session), identical env,
`SESSION_CHAIRS=<own dir> SESSION_CHAIRS_ROT=1 SESSION_ROT_ONLY=<cfg> SESSION_OP=cut
SESSION_FAST=1 SESSION_M3=1`, `SKIPMATRIX`, `(ulimit -v 4194304)`, `timeout 900`.
Repeats interleaved round-robin across configs so each repeat sees a different machine load.
Comparison is **not** the summary line alone — it is the **md5 of the entire stdout+stderr**
(every `[SCAF]`, `[SEGAUDIT]`, `[XWELD]` diagnostic included), with the timing line stripped.

| cell | run | wall s | full-log md5 | faces | solid | shells | naked | nonmani |
|---|---|---:|---|---:|---:|---:|---:|---:|
| y30 cut | r1 | 130 | `a5ea49e042` | 42 | 0 | 3 | 11 | 0 |
| y30 cut | r2 | 139 | `a5ea49e042` | 42 | 0 | 3 | 11 | 0 |
| z37 cut | r1 | 147 | `2e011fdb94` | 38 | 0 | 2 | 27 | 0 |
| z37 cut | r2 | 204 | `2e011fdb94` | 38 | 0 | 2 | 27 | 0 |
| z90 cut | r1 | 141 | `f21b2cf911` | 37 | 1 | 1 | 0 | 0 |
| z90 cut | r2 | 222 | `f21b2cf911` | 37 | 1 | 1 | 0 | 0 |
| x20 cut | r1 | 120 | `383b5b1e06` | 34 | 0 | 2 | 16 | 0 |
| x20 cut | r2 | 182 | `383b5b1e06` | 34 | 0 | 2 | 16 | 0 |
| base cut | r1 | 121 | `70bfc02653` | 35 | 1 | — | 0 | — |

**Variance per cell: ZERO. Not one differing digit, not one differing diagnostic byte.**

This held while the 1-minute load average ranged **28.7 → 58.1** (32-core box; sessions A and
C were running 8-10 of their own kernel processes throughout), producing a **57 % wall-time
spread** on z90 (141 s vs 222 s) with a byte-identical log. Wall time is load-sensitive; the
answer is not.

**Honest limits.** N=2 completed per config at report time, not the N≥5 requested; repeats 3-5
and the base-config repeats were still running when this was written (`logs/phase1.progress`).
A byte-identical *full log* is a far stronger statement than an identical summary line — it
constrains every intermediate quantity the kernel prints — so N=2 here carries more evidence
than N=5 on the summary line would. But it is N=2, and it is stated as N=2.
`SESSION_FAST=1` suppresses the final `volume()` call and the STEP write only; it does not
enter the boolean. Volumes are therefore absent from this table by construction — and for
y30/z37/x20 the harness would refuse to print one anyway, because those results are open
shells and the divergence theorem does not apply (main_7.cpp:1157-1165).

### Why zero variance is the *expected* result

A read-only grep of first-party `src/` (excluding `fmt/ json/ clipper2/ guid/ *_test.cpp`)
for `"/proc/`, `getrusage`, `sysconf(`, `std::thread`, `#pragma omp`, `std::async`, `::rand(`,
`mt19937`, `random_device`, `steady_clock`, `system_clock`, `high_resolution_clock`,
`gettimeofday` returns **one** decision-relevant hit in the whole boolean path:

| site | what it does |
|---|---|
| `src/brep.cpp:44,50` | `brep_rss_mb()` — reads `/proc/self/statm` |
| `src/brep.cpp:170-176` | `guard_check_mem()` — **throws** when RSS > `g_mem_cap_mb` |
| `src/brep.cpp:3942` | check site: chain-lift, every 8th capped run |
| `src/brep.cpp:4780` | check site: per-face UV arrangement |
| `src/brep.cpp:8323-8325` | **the AUTO ladder arms it**: `g_mem_cap_mb = 3000`, `g_segrun_cap = 64` |
| `src/brep.cpp:8364-8371` | `catch` → "resource guard trip = discarded candidate" |

Both caps are `0` (inert) outside the ladder (`src/brep.cpp:55-56`). Everything else that
looks like a budget is an **iteration count**, which is deterministic: `g_segrun_cap`
(brep.cpp:3935-3941), `pair_budget = 20000` (intersection.cpp:4907), `max_iterations` 50/10
(intersection.cpp:934,964), the CDT sweep/flip budgets (remesh_cdt.cpp:708,888).
`high_resolution_clock` appears at brep.cpp:3481 and 8386 — printing only, behind
`SESSION_BOOL_PROFILE`. There are **zero** `unordered_map`/`unordered_set`, **zero** RNG, and
**zero** threads in `brep.cpp`, `brep_section.cpp`, `brep_samedomain.cpp`, `intersection.cpp`.
`mesh.cpp:201-213` and `mesh.cpp:2832-2843` do use `hardware_concurrency()`, but every worker
writes a pre-indexed slot (`results[i]`, `cache[t.out_idx]`), so the output is
scheduling-independent.

So: **without `SESSION_AUTO` the kernel is deterministic by construction, and it measures that
way. With `SESSION_AUTO` there is exactly one door open, and it is the RSS gate.**

### Two latent hazards found, both currently harmless — fix before they bite

1. **`src/brep.cpp:1391-1396`** — a `thread_local` one-entry mesh cache keyed on the raw
   `this` pointer plus face/vertex counts:
   ```cpp
   static thread_local const BRep* s_owner = nullptr;
   static thread_local size_t s_nf = 0, s_nv = 0;
   static thread_local Mesh s_mesh;
   if (s_owner != this || s_nf != m_faces.size() || s_nv != m_vertices.size()) { … }
   ```
   Classic ABA: free a BRep, allocate a new one at the same address with the same face and
   vertex counts, and the *previous* solid's mesh is silently reused for a point-in-solid
   test. That is address-dependent, hence ASLR- and allocator-dependent, hence genuinely
   nondeterministic. It is inert today only because the whole Tier-2 branch is gated behind
   `SESSION_PIP_GUARD` (brep.cpp:1345, 1378, 1387). **Do not promote `SESSION_PIP_GUARD` to
   default until this key is replaced by a content hash or a generation counter.**
2. **`src/brep.cpp:8199-8210` + `8311-8314`** — the ladder's env save/restore cannot
   distinguish *unset* from *set to empty string*: `saved[k] = v ? v : ""`, and `put_env`
   maps an empty value to `unsetenv`. A variable exported as `SESSION_M3=` is therefore
   **deleted** by the first restore, silently changing the pipeline mid-run.
   (`std::map<const BRep*,…>` at brep_samedomain.cpp:252-253 is lookup-only, never iterated —
   benign.)

---

## 2. CONCURRENCY SENSITIVITY — partially measured, mechanism identified

**Status: NOT COMPLETED.** The 6-concurrent wave (`phase2.sh`, 6 competing kernel processes
per config) was queued behind phases 1 and 3 and had not run when this was written. It is
reported as not run. What *was* measured is arguably the same question under a harsher
condition: every serialized run above shared the machine with **8-10 concurrent kernel
processes belonging to sessions A and C**, at load averages from 28.7 to 58.1, and produced
byte-identical logs. That is real contention, not a controlled one.

**The mechanism to look for, if a difference ever appears, is exactly one and it is now
located.** `brep_rss_mb()` (`src/brep.cpp:42-53`) reads *resident* pages. RSS is not a
function of the program alone — under system memory pressure the kernel reclaims resident
pages and RSS **drops**; under hugepage or arena changes it rises. So the throw at
`src/brep.cpp:173` is a coin whose bias depends on the rest of the machine, and the catch at
`src/brep.cpp:8364` converts it into "this ladder variant never existed". Concurrency changes
which variants survive → changes the ladder's argmin → changes the answer.

Two conditions must both hold for it to fire, and both are checkable:
- `SESSION_AUTO` must be set (otherwise `g_mem_cap_mb == 0` and the guard returns immediately);
- peak RSS must approach 3000 MB. **Measured peak RSS in the non-AUTO runs above: ~33 MB**
  — three orders of magnitude below the cap, which is why nothing moved. The ladder's heavy
  variants are the ones that balloon (the comment at brep.cpp:33-38 cites 8.3 GB on z37), so
  the exposure is real but confined to AUTO.

**A second, larger load-sensitivity channel is in the harness, not the kernel.** Session C's
`chairs_base/fuse` at 401 s / 508 s / **600 s-timeout** across three runs is the signature:
`timeout N` kills the process and the harness records a missing or truncated result. Under
load, wall time moved 57 % in my own runs — enough to cross a fixed timeout. A cell that
"flipped between open and crash" is overwhelmingly likely to be a cell that flipped between
*finishing* and *being killed*. That is a measurement-protocol defect, and it is the single
biggest threat to the campaign's A/B history.

---

## 3. THE y30 SD MYSTERY — resolved: **(a) a real SD effect through an unidentified path**

**Status: the empirical 5×5 serialized AUTO runs (`phase3.sh`) were still queued when this was
written and are NOT reported as run.** The question is nevertheless answered decisively at
source level, and the answer is checkable by anyone in thirty seconds.

`SESSION_SD` has a **fifth** effect that is absent from the "four SD code paths" list:

```
src/brep.cpp:8586   static const bool s_sd_mode = (std::getenv("SESSION_SD") != nullptr);
src/brep.cpp:8587   for (int fb = 0; !s_sd_mode && fb < (int)B2.m_faces.size(); ++fb) {
```

The `!s_sd_mode` term is in the **loop condition**. Setting `SESSION_SD` does not modify the
legacy ON-imprint pass — it deletes it. And the kernel's own comment, thirty lines below, at
`src/brep.cpp:8619-8621`, records the measurement:

> "on y30 — which has NO coincidence at all (SD groups=0) — **disabling it alone moved the cut
> from 23.9619 to 47.6964** against a 46.9596 reference."

Those are the two numbers in the trigger, to four decimals, attributed in the source to this
exact toggle. So:

- **It is not ladder nondeterminism (b).** It reproduced twice for session A because it is
  deterministic.
- **It is not load sensitivity (c).** Nothing on this path reads machine state.
- **It is (a)**, and the "unidentified path" is `brep.cpp:8587`.

**Why the path was mis-characterised as inert — and this is the transferable lesson.** The
counter that would have revealed it, `[ONIMP] same-domain imprints onto A: N`, lives at
`src/brep.cpp:8773-8774`, *inside* the loop that `s_sd_mode` skips, and it is furthermore
printed only `if (n_imprint && …)`. Measuring "ON-imprint candidates" **with `SESSION_SD` set**
therefore returns 0 in every case — not because nothing happened, but because the code that
counts is the code that was turned off. A zero read from a disabled probe is not a
measurement. (kb/PLAN_capability_ladder.md already states this rule under P0: *"a zero read
from a binary lacking the print is not a measurement — this actually happened today"*.)

**Also note that neither 23.9619 nor 47.6964 is a volume.** Both were printed for results the
current binary classifies as open (`vol UNDEFINED(open/nonmanifold)`, main_7.cpp:1163-1165):
`volume()` applies the divergence theorem, which requires a closed boundary. The correct
reading is not "SD improved the volume from 23.96 to 47.70" but "SD changed which wrong open
shell we got". The OCCT reference for y30 cut, measured independently here with FreeCAD 1.1.1
on `/home/petras/fc_inspect/REFERENCE/REFERENCE_y30_cut.step`, is **46.9589**, 32 faces, 1
solid, naked 0, OCCT-valid. (z90 cut reference, same tool: **66.9934**, 36 faces, naked 0.)

---

## 4. ORDER DEPENDENCE — NOT MEASURED

`phase4.sh` (4 configs × {common, fuse} × {A,B / B,A}, plus base cut as a positive control,
operands staged as `pairs/<cfg>_{AB,BA}/chair{0,1}.stp`) was queued and had not run. **No
order-dependence numbers are claimed.**

What is known from source: order dependence is **expected to be non-zero**, and the kernel says
so. `src/brep.cpp:8446-8452`: *"The freeform marcher is order-sensitive (seeds come from the
first argument's cells), so the legacy A-by-B / B-by-A calls can trace a grazing section in one
order and miss it in the other."* The shared-scaffold path (`build_section_scaffold(A, B)`,
brep_section.cpp:405) was introduced specifically to make the *imprint* symmetric, but the
scaffold is still built from `A` first. So for commutative ops (common, fuse) any A/B-vs-B/A
difference is a defect by definition, and the machinery to detect it is staged and ready in
`/home/petras/hunt_determinism/pairs/`. One caveat for whoever finishes it: the rotated
operands are STEP round-trips (`chairs/rot/B_<cfg>.step`), so P4 numbers are comparable
**within** a pair, not against the in-process `SESSION_CHAIRS_ROT` numbers.

---

## 5. WHY THE HANDOFF FRONTIER NUMBERS DO NOT REPRODUCE

They were measured on a different operating system, with a compiler that produces different
floating-point results, from source that no longer exists. Nondeterminism is not required to
explain any of it.

1. **The frontier numbers are Windows numbers.** Commit `aa49805` — the commit whose message
   carries `rot cut frontier 196->123: x20 7 y30 10 z30 10 z90 10 z63 11 x13y29 12 z45 17
   z15 17 z37 29` — calls `_putenv` **unconditionally** in the AUTO ladder (`git show 2a810bf
   -- src/brep.cpp` shows it being replaced). Verified on this machine:
   ```
   $ g++ -c putenv_probe.cpp
   error: '_putenv' was not declared in this scope
   ```
   A hard error, not a warning. **`src/brep.cpp` at `aa49805` cannot compile on Linux.** The
   Linux port (`2a810bf`, "fix linux build: portable put_env … replaces `_putenv`") is dated
   the same day as this audit. Corroborated by the run records: `kb/census_*.md` cite
   `./build/Release/main_7_gm.exe` and `main_7.exe` (13 Windows-path references against 1
   Linux one).
2. **The source has moved 683 lines since.** `git status` at audit time: `src/brep.cpp` +578,
   `src/brep_section.cpp` +105, plus untracked `src/brep_samedomain.{cpp,h}`. The binary under
   test is not the frontier algorithm.
3. **`-march=native` (CMakeLists.txt:84).** The binary is CPU-specific. This kernel branches on
   tolerance comparisons, so a different vectorisation is a different topology, not a different
   last digit.
4. **Flag drift.** `z15 measures 17` is not a failure to reproduce — it is an *exact* match to
   `kb/census_z15.md`, which records `faces 38 solid 0 naked 17` under
   `SESSION_NO_MERGE=1 SESSION_NO_SEW=1 SESSION_FAST=1` and **no `SESSION_AUTO`**. The
   frontier's `z15 4` is an AUTO-ladder number. Comparing a non-AUTO run to an AUTO frontier is
   comparing two different algorithms. Likewise "z90/y30 measure 0": z90 cut measures **naked
   0** on today's binary (a closed 37-face solid, table §1) — that is *better* than the
   frontier's 8, not a failure.

---

## 6. VERDICT AND PROTOCOL

### Can the campaign's A/B measurements be trusted?

**Within one binary, on one machine, with one recorded flag set, and with the run not killed
by a timeout: yes — and to the byte.** Zero variance across every repeated cell measured here.

**Across sessions, commits, platforms, or unrecorded flag sets: no.** Every unexplained
discrepancy examined in this audit resolved into one of five bookkeeping failures, and none
into randomness:

| failure | instance |
|---|---|
| platform | frontier measured on Windows; ladder could not compile on Linux |
| source drift | +683 uncommitted lines vs the frontier commit |
| flag drift | AUTO vs non-AUTO compared as if the same measurement (`z15 4` vs `17`) |
| disabled probe read as zero | `[ONIMP] … 0` measured with the counting loop switched off |
| harness timeout | 401 s / 508 s / 600 s-timeout under load = "flipped between open and crash" |

### The protocol that makes them trustworthy

This is the deliverable. Seven rules; the first four are non-negotiable.

1. **Serialize, and pin the binary.** One kernel process per measurement session. Copy the
   binary to a private name and record its **md5** in every ledger row. Never measure against
   a binary another session is rebuilding.
2. **Record the full `SESSION_*` set, the platform, and `ulimit -v` with every row.** All
   three change the answer. `ulimit -v` in particular: a `std::bad_alloc` inside a ladder
   variant is caught at `src/brep.cpp:8364` and silently becomes "variant discarded", so
   session A's 8 GB and my 4 GB are *different experiments*. Record `git rev-parse HEAD`
   **and** `git status --short src/` — an uncommitted tree is not a version.
3. **Compare full logs, not summary lines.** `md5(stdout+stderr)` with timing stripped. It
   costs nothing, it constrains every intermediate the kernel prints, and it is what let N=2
   settle the repeatability question here. A summary line can match while the pipeline
   diverged.
4. **Distinguish timeout from result.** Record the exit code. `rc=124` is *no measurement*,
   never a data point. Set the timeout from a measured baseline × 3, not from a guess, and
   re-run any row whose wall time exceeded half the timeout — wall time moved 57 % under load
   here.
5. **Repeat count.** N=2 byte-identical full logs is sufficient for a non-AUTO cell (the code
   path provably has no non-deterministic input). **N=5 is required for any `SESSION_AUTO`
   cell**, because the RSS gate at `src/brep.cpp:173` is the one real coin — and log peak RSS
   alongside, so a run that came within 20 % of `g_mem_cap_mb` is flagged rather than trusted.
6. **Tolerance band: none. Zero.** Do not adopt a "close enough" band for topology counts or
   log hashes. Any difference between two runs of the same pinned binary + flags + machine is
   a **finding**, not noise — it means the RSS gate fired, or a timeout truncated a run, or
   the flag record is wrong. For *volumes* compared against the OCCT oracle, keep the existing
   `rel < 1e-3` / identity tol 0.4015 from `validation/OCCT_TRUTH.md`; that band is about
   OCCT's own accuracy, not about our reproducibility.
7. **Never read a counter from inside the branch you disabled.** Before recording "mechanism X
   fired 0 times", verify the print is reachable under the flags in force. Add the flag set to
   the diagnostic line itself so this is self-evident in the log.

### Two concrete fixes worth making

- **Make the resource guard deterministic.** Replace the RSS ceiling with a deterministic
  proxy — allocated-object count, topology-edge count, or peak `std::vector` capacity — so the
  ladder's variant-discard decision stops depending on machine state. Alternatively keep RSS
  but log every trip loudly (it is currently visible only under `SESSION_NT_DBG`), so a
  load-induced discard can never be mistaken for an algorithmic result.
- **`kb/DECISION_architecture.md:167-168` proposes collapsing the AUTO ladder**, with the side
  note that "the Step-0 run-to-run nondeterminism disappears". This audit supports that for a
  sharper reason than performance: **the ladder is the only part of the kernel that can give
  two answers to the same question.** Collapsing it removes the sole non-deterministic input
  from the boolean path outright.

---

## Reproduction

All artefacts under `/home/petras/hunt_determinism/`: `run1.sh` / `runpair.sh` / `runauto.sh`
(single runs), `phase1.sh`…`phase6.sh` (batches), `cmp.sh` (full-log hash table), `ladder.sh`
(AUTO trace), `logs/` (every raw log), `pairs/` (staged A/B and B/A operand dirs),
`chairs/rot/B_<cfg>.step` (the ten rotated operands, generated with `SESSION_OP=nop` so no
boolean runs). Phases 2, 4, 5 and 6, and the empirical half of phase 3, were still queued when
this report was written and are marked NOT MEASURED above rather than inferred.
