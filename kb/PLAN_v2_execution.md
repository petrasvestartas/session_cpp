# v2 execution plan

2026-07-26. Written after a session that produced good components, five measurement
errors, and a broken shared build. This document fixes the *process*, not just the
targets. It supersedes ad-hoc redirection.

---

## 0. What went wrong, so the plan corrects it rather than repeating it

| failure | cause | what this plan changes |
|---|---|---|
| 5 measurement errors, 3 of which inverted a headline | every agent wrote its own verdict metric | ONE shared, validated verdict harness; no agent scores anything with its own code |
| shared build broken, session A stalled | 4 agents writing `src/*.cpp` into a GLOB + Unity build | v2 lives in `src/v2/` (glob is non-recursive), separate target; v2 cannot break `main_7` |
| constant re-targeting on each new report | no plan; reacting to the newest finding | fixed phase gates; redirection only when a gate fails or a premise is falsified |
| 17k lines of spec, 3.5k of code | specs commissioned in parallel with implementation | specs are DONE; no more spec work. Implementation only |
| "industry grade" unmeasurable | no definition of done | §2 defines it as specific cells at specific tolerances |

---

## 1. Structural fixes (Phase 0 — blocks everything else)

**0.1 Build isolation.** All v2 sources move to `src/v2/`. `CMakeLists.txt:221` globs
`src/*.cpp` non-recursively, so `src/v2/*.cpp` is invisible to `session_core` — exactly
how `src/yaml/` is handled. Add one explicit target `session_v2` linking `session_core`.
Consequence: a broken v2 file breaks only v2. Four sessions stop being coupled.
Namespaces stay mandatory (`v2sec`, `v2sf`, `v2int`, `v2sol`) because Unity batching still
applies within the v2 target.

**0.2 One verdict harness.** `src/v2/v2_verdict.h/.cpp`, written once, used by every v2
test driver and every sweep. It returns:

```
struct V2Verdict {
  int  faces, shells, solids;
  int  naked_real;      // 1-trim edges EXCLUDING zero-length degenerate (poles, apexes)
  int  nonmanifold;     // >2 face uses
  bool seam_ok;         // an edge used twice by one face is NOT naked
  double closure_residual;   // |sum of outward vector areas| / area   (brep_massprops)
  double volume; bool volume_valid;
  bool closed() const;  // naked_real==0 && nonmanifold==0 && closure_residual<1e-9
};
```

**Validation gate, run before it may score anything**: agree with `is_solid()` on an
un-split sphere (poles), cone (apex), cylinder (seam), torus (two seams) and box. This is
non-negotiable — every one of the five errors came from a metric that disagreed with the
kernel's own validity rule on exactly these shapes.

**0.3 Frozen baseline.** Re-score the 222-cell in-memory sweep with 0.2 and publish it as
`kb/BASELINE_v1.md`. Current best knowledge (to be replaced by the harness numbers):
36 genuine exact / 38 trivially exact / 27 closed-wrong / 121 open. Trivially-exact
(empty common) cells are reported separately and never counted as successes.

---

## 2. Definition of done

Three tiers. A tier is done when every cell passes under the 0.2 harness, judged
oracle-free wherever possible.

**T1 — primitives, arbitrary pose.** 15 pair types x 20 seeded rigid motions x 3 ops.
Pass = `closed()` AND (volume matches the analytic value to 1e-9 where derivable, else
partition identity `cut+common = volA` to 1e-9). No hangs. **This is the tier that
defines "works for curved solids".**

**T2 — trimmed multi-face.** The 10 rotated chair configs x 3 ops, same rule, plus
`A-op-A` idempotence under rotation.

**T3 — foreign and freeform.** OCCT/FreeCAD-authored operands, the freeform ladder
(L0 boxes → L5 freeform x freeform), and chained booleans to depth 3.

Explicitly NOT the bar: matching OCCT cell-for-cell. OCCT is self-inconsistent on z15,
fails BRepCheck on 3 of our 30 reference cells, returns NaN on 34 of 80 cross-pairs, and
omits a legitimate lump of A\B in two cells. Match the mathematics.

---

## 3. Implementation sequence

Each increment: one owner, one gate, one kill criterion. No increment starts before its
predecessor's gate is green — except where §4 marks it parallel-safe.

### INC 1 — Phase 0 (owner: session A) — no cell movement expected
Build isolation (0.1), verdict harness (0.2) with its validation gate, frozen baseline (0.3).
**Gate:** harness agrees with `is_solid()` on all five shapes; `main_7` builds while a
deliberately broken file sits in `src/v2/`; baseline published.
**Kill:** none — this is unconditional.

### INC 2 — v2 interference stages (owner: interf agent) — parallel-safe
VV, VE, EE, VF, EF into the arena.
**Gate:** on sphere x cylinder through-centre at 0–45°, every true edge-face piercing has
an arena vertex and the pierced edge is paved at it; interference COUNT invariant under a
rigid motion applied to both operands.
**Kill:** if piercing counts vary with pose on a configuration whose answer is
pose-invariant, the intersector is wrong, not the paving — stop and report.

### INC 3 — v2 section stage (owner: section agent) — parallel-safe
FF section, restriction to the *trimmed* faces, arena entities, cross-pair fusion,
seam splitting.
**Gate:** sphere x cylinder through-centre produces the SAME section topology at every
tilt 0–45° (it must, the configuration is tilt-invariant); box x sphere's six coplanar
circles resolve to one shared node set.
**Kill:** if section topology varies with tilt, the defect is in restriction/seam
handling — localise before proceeding.

### INC 4 — v2 face splitter (owner: splitface agent) — parallel-safe
Wires from arena entities; angular ordering in UV corrected by the surface metric;
outer/inner classification; seam and pole handling.
**Gate:** THE DECISIVE ONE — split one operand alone, canonical `[0,1]` domain and padded
`[-0.04,4.04]` domain, both give identical topology and **0 naked**. v1 gives 0/20 and
32/36. Also: sphere face split by a section circle at 20 poses, all shared, 0 naked.
**Kill:** if padded and canonical still differ, entity identity is not actually being used
— inspect the lift, do not widen a tolerance.

### INC 5 — v2 assembly + selection + driver (owner: solid agent)
Shells by shared-edge connexity; cavity nesting; op-table selection; the end-to-end driver
behind `SESSION_V2`.
**Gate:** **FIRST CURVED SUCCESS** — sphere r2.5 x cylinder r1.0 through the centre, tilted
about Y, `common = 15.061795` and `cut = 50.388051` at **every** tilt 0–45° in 1° steps, to
1e-9, cut never open. v1 is exact 0–22°, drifts at 23°, opens at 24°, is 5.6e-2 wrong at
25°, and the cut stays open through 44°.
**Kill:** if v2 reproduces a band failure at the same tilts, the defect is upstream of
assembly and INC 3's gate was not strict enough.

### INC 6 — T1 sweep
All 15 pairs x 20 poses x 3 ops through v2, scored by the 0.2 harness, reported
side-by-side against `BASELINE_v1.md`.
**Gate:** v2 >= v1 on every pair; cone pairs (today 18–19/20 open) move materially.
**Kill:** if v2 is worse on any pair v1 handles (box x box 15/20, sphere x sphere 16/20),
stop and fix before extending.

### INC 7 — coincidence integration
Wire `brep_samedomain` + `brep_commonblock` (both green, 73/73) into the v2 section stage.
**Gate:** A-op-A exact under rotation (cut empty, common = A, fuse = A); partial-coincidence
cells from the stress design.

### INC 8 — T2 chairs; INC 9 — T3 foreign/freeform
Gates as in §2.

---

## 4. Parallelism

Safe in parallel once INC 1 lands, because all four write to `src/v2/` in separate
namespaces and communicate only through the arena's stable API:
INC 2, INC 3, INC 4 concurrently; INC 5 integrates them.
Serial: INC 1 → {2,3,4} → 5 → 6 → 7 → 8 → 9.

Session A additionally owns v1 instrumentation *only* when a v2 gate needs a v1 comparison
number. **v1 is frozen otherwise** — no more v1 patching. Sessions B and C continue on
coincidence and corpus respectively, unchanged.

---

## 5. Reporting discipline

- Every number carries: binary sha1, full `SESSION_*` env, cell count attempted vs
  completed, and whether the run exited.
- No claim from a partial log, a binary lacking the probe, or a metric that has not passed
  the 0.2 validation gate. These three rules cost five errors today.
- Trivially-exact (empty common) cells reported in a separate column, never summed into
  successes.
- A control cell with an independently known answer in every sweep; if the control moves,
  the harness moved.

---

## 6. Honest risk

**What could make this fail.** The port surface is large — section restriction, angular
ordering on curved surfaces at multi-edge vertices, and cavity-aware shell assembly are
each subtle, and OCCT spends far more code on them than we will. A subtly wrong piece
produces plausible-but-wrong results, which is the failure mode that has already cost this
project months.

**Mitigation.** Every gate above is oracle-free where possible (tilt-invariance, partition
identity, padded-vs-canonical equality, rotation invariance of counts). Those cannot be
satisfied by a plausible-looking wrong answer.

**Fallback if v2 stalls at INC 5.** v1 is exact 0–22° on the same case and 36/222 overall;
its failures are now bracketed to narrow bands with named suspects (seam entering the
section). Targeted v1 fixes remain available and v1 stays frozen-but-working throughout —
this plan never leaves the project without a functioning kernel.

**What this plan does not promise.** Zero failures. 38% of OCCT's own 25-year bug history
is silently-wrong boolean output. The bar in §2 is specific cells at specific tolerances,
which is a bar that can actually be met and measured.
