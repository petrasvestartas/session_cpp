# DECISION — boolean kernel architecture

**Date:** 2026-07-25 · **Decider:** architecture review over 4 independent analyses + measured evidence
**Status:** ACTIONABLE. Steps 0–1 are unconditional and gate everything after them.

---

## VERDICT

**(b) LAW 1 — but scoped as *finishing the pave-block path that is already built*, not as a splitter rewrite.**

Not (a). Not "(a) first, then (b)". The one genuinely new piece of (a) — common-block synthesis wired into the
current per-operand splitter — is the piece that gets thrown away under (b), and it is the piece least likely to
work, because it needs a carrier the current data flow cannot provide.

Session B's same-domain detector + BuildBOP op-table are **kept** (≈500 of 693 lines are carrier-independent,
59/59 on `main_10`). They are an *input to* (b), scheduled last, not an alternative to it. `grep sd_select_face
src/brep.cpp` → **zero hits**: they are not wired anywhere, so nothing is lost by scheduling them at the end.

---

## THE DECISIVE FACT

**Turning the shared-edge pool on changes nothing on rotated configs, and makes the entire 3,729-line
reconciliation layer a measured no-op on the base case.** One A/B, both halves load-bearing:

```
base chairs cut, SESSION_BOP2=1:  pool 40v/40e, 156×[P4REF], 4×[P4MISS]
                                  [NT] A2 edges1=0/148  B2 edges1=0/140
                                  combine 0  blocks 0  imprint 0  co_refine 0  xweld 0  sew 0

z15 cut  BOP2 OFF: A2 edges1=21/152 B2 1/144 | sew 17 | naked 17 OPEN
z15 cut  BOP2 ON : pool 43e, [P4] merged 33 | A2 edges1=24/155 B2 1/145 | sew 17 | naked 17 OPEN
z30 cut  BOP2 OFF: A2 edges1=14/169 B2 16/146 | sew 23 | naked 23, nonman 2, OPEN
z30 cut  BOP2 ON : pool 45e, [P4] merged 34 | A2 edges1=18/185 B2 18/151 | sew 24 | naked 24, nonman 2, OPEN
```

Read it precisely, because it refutes both advocates' headline claims:

1. **Shared entities are the right model.** On base chairs, referencing pool edges drives one-trim edges to
   **0/148 and 0/140 before combine** and every one of the nine `count_nt` checkpoints reports zero changes.
   `co_refine` (918 L), `sew` (406 L), `xweld` (491 L), the fuzzy ladder — all executed, all no-ops. That is the
   3,729-line repair layer proving *by measurement* that it exists only to undo per-operand independence.
2. **The landed 60% of (b) is the cheap 60%, and it yields zero on the target class.** The residual naked edges
   are one-trim edges minted **inside a single operand's own split** (A2 24/155 on z15). Sharing an edge object
   cannot manufacture a run the other operand never emitted. `brep.cpp:8762` names the mechanism verbatim:
   *"the two operands' arrangements can disagree on which scaffold section runs survive the dangling-edge prune
   (measured z90 cut: A drops segs 0,1,2,20,21,22 that B keeps)."*

So the remaining value of (b) is **not** "make edges shared" (done, gated off, worth nothing on rotation). It is
**totality**: make the shared graph *authoritative over the arrangement* — every run spans a whole block, the
per-operand prune loses its veto, and the paves that make one-sided drops impossible actually exist. Nothing in
(a) touches any of that.

Supporting facts that fix the ranking (do not re-litigate):

- **97% section suppression → zero outcome change on A-op-A** (481→10 segs; cut still 72f/18 naked, common
  204f/40 naked, fuse still times out). A section-stage change with a 97% amplitude moved the result by 0. The
  residual defect is downstream of the section stage — exactly where (a) does not operate.
- **The optimiser is aimed at a metric that cannot see the failure.** `brep.cpp:8240` `metric()` counts only
  `trim_indices.size() == 1`. A 3-trim edge scores **0**. `res_y30_cut`'s 13 non-manifold edges are invisible;
  its own comment (*"With naked == 0 every edge already has 2 trims"*) is false. The AUTO ladder will actively
  **prefer a non-manifold shell**. Every number in 196→108→96 is an argmin over this. This is the mechanical
  reason the handoff numbers do not reproduce.
- **The 45/45 matrix is structurally out of the blast radius.** `scaffold_eligible = imported_freeform &&
  has_freeform(deg>=3)` (`brep.cpp:8389`). Every matrix pair has ≥1 recognised primitive → legacy path →
  byte-identical through steps 2–7 *by construction*, not by care. Same for all 762+746+724 minitests and both
  ports. **Base chairs are the entire regression surface, and they are three numbers.**
- **Growth vs yield:** `src/brep.cpp` 1,806 lines (2026-06-16) → **12,045 today**. +10,239 lines in 39 days.
  Verified yield on the target class: **1 of 30 rotated cells**, and that one (z30x20 54.2377 vs 54.2581) is
  inside tolerance, not exact. Continuing to patch is not the conservative choice; it is the choice with a
  measured zero slope on a blind metric.

---

## WHY NOT (a)

(a)'s own advocate concedes F1: `res_y30_cut` is a **transversal** pair with 13 non-manifold edges and no
coincident faces. No detector, no op-table, no common block, no CB tolerance touches it. (a) cannot move the 25
open rotated cells, and both advocates agree on this.

What (a) would legitimately close — A-op-A, flush stacks, shared walls, coplanar caps — is real and a credible
kernel must have it. But:

- `main_10`'s 59/59 builds coincident pairs directly from primitives via `place()`; **it never runs them through
  the splitter**. 59/59 → in-pipeline is precisely the untested gap.
- Synthesis needs a **carrier**. Bolted onto the current splitter, the synthesised coincident region must be
  numerically attached to two independently computed UV arrangements that disagree by the census's own
  0.038–0.466 junction gaps. That attachment *is* sewing — the thing being escaped. Under a shared graph the
  carrier already exists (`SharedEdgePool` + a common block). **(a)-first builds the carrier twice and discards
  the first.**
- It would add a 91st behavioural gate and a 9th AUTO rung, scored on the metric from `brep.cpp:8240`.

**A-op-A is therefore reassigned: it is not a feature (a) delivers, it is Law 1's own acceptance gate.** Every
face of A is coincident with a face of B — maximally degenerate. If the graph is genuinely shared, A-op-A is the
*easy* case. If Law 1 lands and A-op-A still fails, Law 1 did not land. `corpus/invariants.py idempotence`
already implements it and it runs in seconds, not the 15–18 min a T0 costs.

---

## FIRST THREE STEPS

### Step 0 — Make the numbers mean something (½–1 day). **Before any code decision.**

1. Replace AUTO's `metric()` (`brep.cpp:8240`) with `topology_report`'s verdict: `naked == 0 && nonman == 0`
   plus the existing volume-sanity gate. Add `nonman` + shell count as corpus verdict columns.
2. Record the exact `SESSION_*` env set in `baseline_meta.json` (today: `"flags": {}` against 119 gates) and
   refuse cross-flag ledger comparisons.
3. Run T0 **three times, serialized**, same binary, same flags. Prior evidence: 104/105/105 exact,
   `chairs_base/fuse` ∈ {401 s, 508 s, 600 s-timeout}, `chairs_rot/z90/common` ∈ {open, crash@0 s, open} —
   **2/132 cells flip verdict class run-to-run.** Declare the noise band.
4. Free measurement while rebuilding: `SESSION_PIP_GUARD=1` (written at `brep.cpp:1341-1398`, default-OFF, with
   a measured `dp = -1.9e-2` boundary-hit bias) across T0 + A-op-A. One rebuild, no development. Record the
   delta; commit to nothing.
5. Ten minutes, do it now: add to CLAUDE.md that `brep_section`, `brep_samedomain`, `BRep::boolean` internals
   and `file_step` are an **explicitly C++-only subsystem with a frozen public API and a mirrored assertion
   set**. Python's boolean is 156 lines against C++'s ~3,370; the drift is a month old. Make it a recorded
   decision instead of an accumulating rule violation.

**Gate:** the re-baselined ledger reclassifies `res_y30_cut` (13 non-manifold edges) out of "exact", and the
exact count **falls from 105**. If it does not fall, the metric change did not land. Losing that number is the
deliverable.

### Step 1 — Partition the 25 open cells: *discovery* failure vs *agreement* failure (1–2 days). **Highest-value work in the program.**

For each open cell, dump the scaffold's section branches and compare against `validation/step_probe` /
`occt_oracle`'s section. One question per cell: **does the scaffold contain every branch OCCT finds, yes/no?**

**Gate:** a 25-row table, every cell labelled.
**Decision rule:** ≥60% agreement-failures → proceed to Step 2. If half or more are **discovery** failures →
**stop; the project is Law 4 (branch enumeration/seeding — `SESSION_SEED2` is a partial), not Law 1.** A shared
graph of the branches you found is still missing the branch you never traced.

`brep.cpp` states this ceiling itself: *"the marcher is seed-limited and simply never traces branches it was not
seeded on, which no amount of repair downstream can invent."* Learning this after a rewrite instead of before is
the single most expensive outcome available. Two days removes it.

### Step 2 — Totality: BOP2 default-on inside the scaffold branch + `refine_scaffold_at_breaks` mandatory to fixpoint (1–2 weeks).

The substrate exists (`build_shared_edge_pool`, `brep_section.cpp:2578`; pool referencing at `brep.cpp:3969`).
What is missing is that only **whole-segment** runs reference it: `bool wseg2 = full_wrap || (fa < 1e-2 && fb >
nCh-1-1e-2);`. Everything else falls back to mint + `emap` + span. `refine_scaffold_at_breaks`
(`brep_section.h:77-88`) exists precisely to make that predicate always true — *"after refinement every run
spans a whole segment"* — and is today a two-pass env-gated loop, not an invariant.

Make it a loop to fixpoint with a **hard run-count cap that throws a diagnosable failure rather than
allocating** (`SESSION_SYMEMIT`'s own comment records 459 runs / 19k edges / 8.3 GB → `std::bad_alloc` on z37 at
exactly this site).

**Gate:** `[P4MISS] == 0` on base chairs (today 4) **and** on z15 + z30; base chairs still exactly
35/46.8114, 25/33.4951, 50/127.0950; matrix 45/45 asserted byte-identical (guaranteed by `scaffold_eligible`,
assert it anyway); T0 non-decreasing under the Step-0 metric; refine terminates within cap on all 132 cells.

---

## THE REST OF THE PATH (sketch, gates only)

3. **Block-level referencing** — partial runs reference `pool.block_edge`. *Gate:* "section edges created by the
   sew stage" instrumented to **0** on base chairs; T0 non-decreasing.
4. **Remove the arrangement's veto** — the dangling-edge prune becomes a graph-level decision, not per-operand.
   *Gate:* `[SEGLOST]` count → 0 on z90 (today: A drops segs 0,1,2,20,21,22 that B keeps).
5. **Delete repairs one per commit**, each with its own T0: NK-RESCUE → junction bridge weld → EF-march →
   SYMEMIT → SYMLIFT → third weld pass → `co_refine` → `sew`. *Gate:* exact count non-decreasing per deletion.
   **This step is what proves Law 1 landed.** A repair that deletes cleanly was repairing a defect now
   impossible by construction; a repair that cannot be deleted means the arrangements still diverge somewhere
   unfound — stop and find it, do not re-add the patch.
6. **Collapse the AUTO ladder.** *Gate:* T0 with `SESSION_AUTO` unset == T0 with it set. Side effect:
   `chairs_base/fuse` ~500 s → ~60 s, and the Step-0 run-to-run nondeterminism disappears.
7. **Law 5 topological classification.** Keep `contains_point_exact` as the per-region seed; drop the 763-line
   per-face voting/override/parity/repair architecture. *Gate:* idempotence invariant + T0.
   *(Note: the doctrine's claim that "the classifier survives" is wrong. Its kernel survives; its architecture
   does not. Budget ~900 lines.)*
8. **Coincidence lands here**, as common blocks in the pool — where OCCT puts it (`BOPDS_CommonBlock`). Session
   B's detector + op-table plug in unchanged. *Gate:* `main_10` 59/59 still green **and** A-op-A green (cut
   EMPTY, common = A at 20 faces/80.2969, fuse = A, no timeout) **and** T0.

**Do NOT wire `sd_boolean_coincident` into the current splitter at any point.** That is the only piece of work
discarded under either outcome.

### Missing safety net — required before Step 2, not optional

- Route **all 132** cells through `step_probe`. Today only the 33 chair cells get an external verdict; the 99
  matrix+edge cells are graded on our own `volume()` / `is_solid()` — self-grading on metrics this campaign has
  already been burned by. ~40 lines in `runner.py`.
- Add **A-op-A to the T0 manifest** so it produces tickets. It is currently only in a nightly script that is not
  in CI and needs OCCT `step_probe` built.
- Add **golden-structure minitests** on scaffold + pool under `SESSION_SCAFFOLD_ALL`: assert
  `segments.size()`, `vertices.size()`, `max_devA/max_devB`, `seg_keyed == segs`. Today
  `grep -l "scaffold\|SharedEdgePool" src/*_test.cpp` → **zero files**: there is no tripwire between "I changed
  a data structure" and "a 132-cell verdict flipped" with 3,000 changed lines to bisect. This is the single
  highest-value missing artifact and it does not exist in any form.
- CI covers **0%** of this code (its boolean minitest is box × cylinder → both recognised → legacy path).

---

## WHAT REVERSES THIS

| Trigger | Reversal |
|---|---|
| **Step 1 shows discovery-dominance** (≥half of the 25 open cells are missing section branches, not disagreeing about found ones) | Abandon (b). The project is Law 4 branch enumeration. Law 1 would faithfully share a graph that is missing the branch. |
| **`[P4MISS]` cannot be driven to 0** — `refine_scaffold_at_breaks` does not reach fixpoint, or hits the cap on grazing cells | Law 1 has no floor. Fall back to (a) for the coincidence class + Law 4 for discovery, and accept a repair-based kernel. |
| **Base chairs cannot be held exact through Steps 2–4** | Stop. The new data flow is losing something the old one had. Do not paper it over with a new tolerance. |
| **≥3 of the 6 repair mechanisms are undeletable at Step 5 without T0 regression** | The graph is not authoritative. Diagnose where the arrangements still diverge; do not re-add patches and declare victory. |
| **A-op-A still fails at Step 8** with the graph shared and the op-table wired | Law 1 did not land. This is the harshest, cheapest gate; treat a red here as decisive. |
| **45/45 matrix moves at all** | Impossible by `scaffold_eligible` — if it happens, something escaped the branch predicate. Halt immediately. |

---

## HONEST COST

- **Steps 0–1: 2–3 days, valuable under every outcome.** These are the only unconditional items.
- **Steps 2–7: 2–4 months of focused C++**, comparable to what has already been spent. ≈5,000–5,500 lines
  deleted or replaced in `brep.cpp` (of 12,045); ≈1,500–2,000 written. Step 5 alone is ~25–30 hours of pure gate
  time (15–18 min per T0 × ~90 behavioural gates' worth of deletions and re-validations). Every further week of
  patching adds to that bill — delay does not preserve optionality here, it consumes it.
- **The largest unknown is EF/VF interference paving.** The scaffold paves from SSI chains and trim-loop
  crossings only; there is no edge-vs-face or vertex-vs-face stage. OCCT spends `BOPAlgo_PaveFiller_4/5.cxx` on
  it. There is no anchor in this tree and no estimate worth quoting. x20 and x13y29 are attributed to it.
- **This will not produce a kernel with zero rotated failures.** `kb/corpus_occt_bug_cases.md`: 38% of OCCT's
  25-year bug history is silently-wrong output, 5–7% crashes. Write this down now so a non-zero residual after
  the rewrite reads as normal kernel engineering, not another failed month.
- **What (b) actually buys, stated conservatively:** SEGLOST, divergent duplicates, junction gaps and >2-trim
  non-manifold edges become *unrepresentable* rather than rarer. Closure becomes near-universal. **Correctness
  becomes the separable remaining problem** — and a closed-but-wrong result is diffable against the oracle,
  while an open shell is not. It does **not** fix wrong-branch SSI (the z90 apex slide) or the 2 closed-wrong
  T0 cells.
- **The one thing not in doubt:** the geometry is right. 45/45 matrix, cone × cone to 1e-14, rigid-motion
  equivariance at 4e-6, all produced by the code that survives untouched — `intersection.cpp` (6,821 L, zero
  BRep references), the Gauss-Newton corrector, `split_by_uv_curves`, `file_step.cpp`, JSON/proto. ~17,000 lines
  of geometry are not on the table. That is what makes this affordable: the part that works is the part you
  keep.
