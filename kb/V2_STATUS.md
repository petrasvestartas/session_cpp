# V2 STATUS — consolidated, against `kb/PLAN_v2_execution.md`

2026-07-26, after the seven-track wave. Scored against the plan's gates only.

**Provenance of this document.** Four of seven track reports are in `kb/`
(`v2_diff_harness.md`, `v2_t2_chairs.md`, `v2_t3_interop.md`, `v2_performance.md`). The other
three are not, at the time of writing. The T1 ladder totals below (`104/44/127`, `104/83/132`)
come from the orchestrator's summary line, **not** from any file in `kb/`; the per-pair table
needed to audit the INC 6 kill criterion does not exist in the repo. That is itself a §5
reporting-discipline failure and is item N0 below. Every other number here is traceable to one
of the four reports.

---

## 1. Gate table

| # | Plan gate | Verdict | Deciding number |
|---|---|---|---|
| INC 1a | verdict harness agrees with `is_solid()` on box/sphere/cone/cylinder/torus | **MET** | `main_17` 5/5 PASS, reproduced independently in three build dirs this wave |
| INC 1b | `main_7` builds with a broken file in `src/v2/` | **MET (structural, not re-tested)** | `session_v2` is a separate OBJECT target (`CMakeLists.txt:295`), `src/*.cpp` glob non-recursive (`:225`). Indirect: `src/brep.cpp` rewritten 14:30 and `src/v2/v2_dump.cpp` added 14:21 by concurrent sessions with no cross-break; T2 rebuild after that edit reproduced 12/12 control cells bit-identical |
| INC 1c | frozen baseline published as `kb/BASELINE_v1.md` | **NOT MET** | File absent. The baseline exists only as a prompt line with no per-pair table, no binary sha1, no env — the exact artifact §5 requires |
| INC 2 | every true EF piercing paved on sph×cyl 0–45°; interference counts invariant under rigid motion | **MET in module / VOID in the shipped path** | `main_15` 29/29 PASS, but the production v2 boolean **never calls `v2int::V2Interf`**; the harness's IVV/IVE/IVF/IEE/IEF rows come from a separate probe arena and are reported, never scored. Production contradicts invariance: sph×cyl roty23.578 `ds.vertices` 5 vs 7; at exact tangency v2 mints `(0,0,±2.5)` as 3 NEW arena vertices where OCCT inflates the existing poles (1 new) |
| INC 3a | section topology identical at every tilt 0–45° | **NOT MET — kill criterion FIRED** | roty23 `sec.curves` 3/3 (all upstream identical); roty24 `sec.curves` 3 vs 2 with `sec.pblocks` still 3/3. Topology varies with tilt |
| INC 3b | box×sphere's six coplanar circles resolve to one shared node set | **UNMEASURED** | Nearest evidence: sph_box `ds.vertices` 17 vs 18, result closed, rel vol 1.1e-3 |
| INC 4 | padded `[-0.04,4.04]` vs canonical `[0,1]` split identical, 0 naked | **MET on analytic faces / FALSIFIED on trimmed imports** | `main_14` 62/62 PASS. Production per-face split of a 20-face chair: 17–20 A images with **18–46 naked**, 1–3 B images, volume 4.2–65.8 against 83.15 |
| INC 5 | FIRST CURVED SUCCESS — sph r2.5 × cyl r1.0, `cut=50.388051` / `common=15.061795` at every tilt 0–45° to **1e-9**, cut never open | **NOT MET — kill criterion FIRED** | Best cell (23°): v2 cut 50.3880965, err 4.5e-5 = **4.5e4× the gate**. Worst in-band (23.578°): 69.51920 with naked 3, 38% high. Note 23° is v2's *win* on accuracy: OCCT 50.3846998 (err 3.4e-3) vs converged 50.3880515 |
| INC 6 | **v2 ≥ v1 on every pair**; cone pairs move materially | **NOT MET — kill criterion FIRED** | Two cone pairs regress. "Move materially" met only in the volume column (44→83) |
| INC 7 | A-op-A exact under rotation (cut empty, common = A, fuse = A) | **NOT MET for v2** | v2-forced fails all three: cut 60 faces/52 naked, common 10 faces/26 naked, fuse 122 faces/68 naked, vol −46.86. v1+`SESSION_SD=1` passes exactly (cut EMPTY; common = fuse = A, 20 faces, naked 0, vol 80.294113); v2 "passes" only by delegating. Default env: both fail. `brep_samedomain` is not wired into the v2 section stage — INC 7 has not started |
| INC 8 | T2 chairs: `closed()` + 1e-9 on 10 configs × 3 ops + A-op-A | **NOT MET, and UNREACHABLE AS SPECIFIED** | 0/30 at resid<1e-9 for every kernel and env. The un-split operand `chair0` is itself resid 9.117e-03 (default budget) / 6.37e-06 (100× budget). STEP round-trip control on box/sphere/cylinder: resid 0 / 5.95e-17 / 5.45e-18 — so the floor is the chair trim data, not STEP and not the budget. Topology-only score: v1 3/30, v2 3/30 (same cells, by delegation), v2-forced **0/30** |
| INC 9 | T3 foreign/freeform | **NOT MET, and partly UNMEASURABLE** | Round-trip identity (the precondition) fails first: 3 of 5 OCCT primitives survive import+export. 0/40 imported OCCT boolean results `closed()`. Foreign SUCCESS: cur 3/30, **v2 0/30**; own-geometry control: cur 3/28, **v2 6/28**. 3 FALSE_PASS cells per kernel |
| — (no plan gate) | liveness | **MET** | 194/194 cells exited; zero timeouts, zero `bad_alloc`, zero non-termination in either kernel. coneR×cyl — previously "no output in 600 s twice", `bad_alloc` under 4 GiB — completes all three ops in 5.4–10.4 s and under `ulimit -v 4194304` |
| — (no plan gate) | memory | **MET** | Peak RSS 417 MB worst synthetic (130+130 faces), 77 MB worst realistic cell, against the 8.4 GB prior regression |
| — (**gate missing from the plan**) | speed | **FAIL** | v2 ladder median **3072 ms** vs current **13.7 ms** = 224×; worst 18.4 s; chairs 9.1–12.2× current and 2.4–13× outside the OCCT band; longest operation anywhere 469.8 s |

---

## 2. v2 vs current, the three ladders now on record

**T1 ladder — 8 pairs × 20 rigid motions × 3 ops (of 160):**

| metric | current | v2 | delta |
|---|---|---|---|
| closed | 104 | 104 | 0 |
| volume | 44 | **83** | **+39** |
| partition identity | 127 | **132** | **+5** |
| pairs where v2 < current | — | **2 (cone family)** | **kill criterion violated** |

**T3 foreign / translation-only poses (different pose family, not a subset of the above):**

| cell family | current | v2 |
|---|---|---|
| foreign operands, SUCCESS = `closed()` ∧ rel vol < 1e-6 | 3/30 | **0/30** |
| own-geometry control, same scoring | 3/28 | **6/28** |
| OCCT-authored box × box (cut/common/fuse) | 43 / 21 / 107, all closed, exact | **0.333333 / 21.6667 / 64.3333, all open** |
| FALSE_PASS (closed, solid, and wrong) | 3 | 3 |

v2's wins are real and large where they land — sphere×sphere control rel err 4.2e-9 / 5.1e-9 /
1.7e-9 against cur's 1.4e-4 / 5.3e-4 / 6.1e-5 — but `cur == v2` byte-identical in 25/28 control
and 24/30 foreign cells, and in 36/36 chair cells, because the v2 front end refuses and delegates
to `BRep::boolean` (`brep_v2_boolean.cpp:1255`).

**Speed ladder — 8 pairs × 3 poses × 3 ops, warm medians (ms):**
current 3.2 / 13.7 / 350.2 (min/med/max); v2 53 / 3072 / 18377. Sum of medians 4.03 s vs 370.2 s.

---

## 3. THE DECISION

**No. v2 does not meet the kill criterion "v2 ≥ current on every pair", and it is now violated on
two independent axes, not one.**

What remains, exactly:

**R1 — the two cone regressions (T1 ladder, the literal criterion).** Result-level today:
box×cone p2 rel vol 1.3e-1 with closure residual 6.80e-2; cone×cone p1 rel 3.4e-1 with 4 naked;
cone×cone p2 rel 3.5e0 with 21 naked. Both carry the same first-divergence signature —
`ds.vertices` inflated (16 vs 24, 13 vs 15, 11 vs 21) — and the diff harness proves it is **not**
the section marcher on box×cone (`sec.block_nodes` 12/12, worst 6e-8, yet the arena then acquires
8 spurious new vertices). Must return to ≥ current per pair.

**R2 — OCCT-authored box × box (T3 pose family).** cur is exact and closed on all three ops; v2 is
open on all three, on the easiest pair in the matrix, while being exact on the identical
own-authored geometry. Independently reproduced by chain C4 (46 / 30.6667 / 15, open from depth
1). This is a second, separate criterion violation: **fixing the cones does not clear it.**

**R3 — the criterion is currently un-auditable.** The per-pair 8×20×3 table for either kernel is
not in the repo, so the verdict above rests on a summary line rather than on a re-derivable
artifact. Until N0 lands, "v2 ≥ current on every pair" cannot be checked by anyone but the agent
that ran it — the failure mode §0 of the plan was written to eliminate.

Not blocking the criterion but worth stating alongside it: **v2 is not shippable regardless**, on
the speed gate the plan never wrote (224× median, 469.8 s worst case), and its slowness is already
corrupting other tracks' measurements — 7 of 14 T3 chain runs and 4 T1 sph×cyl cells were lost to
`rc=124` timeouts caused by v2 runtime, not by defects.

---

## 4. Ranked next increments

| # | Increment | Gate | Deciding number today |
|---|---|---|---|
| **N0** | Publish `kb/BASELINE_v1.md` with the per-pair 8×20×3 table for **both** kernels, binary sha1 + full env per row; add `20 21 23 24` to `CMakeLists.txt:311 foreach(MAIN_ID …)` | The INC 6 kill criterion is re-derivable from `kb/` alone; every reported binary builds via `cmake --build` | Baseline file absent; four of this wave's four drivers were hand-compiled outside CMake |
| **N1** | Arena node fusion — the cyl×cyl weld | cyl×cyl vol 47.68180 (rel < 1e-6), `ds.vertices` 6/6, naked 0, **and** box×box cut/common still report `FIRST DIVERGENCE: none` | v2 mints two nodes **2.2–2.4e-6 apart** where OCCT has one, under the trace's own recorded fusion tolerance of **1.17e-5** (5× the gap); closure residual already 7.82e-09. Cheapest correct-answer conversion in the whole table — one weld |
| **N2** | Spurious arena vertices, cone family — **the kill-criterion blocker** | box×cone p2 `ds.vertices` 24→16 and rel vol 1.3e-1 → <1e-6; cone×cone p1 naked 4→0; per-pair ladder ≥ current on both cone pairs | **Scope this first:** box×cone's section is exact (12/12 nodes, 6e-8) but cone×cone's section is **21% short** (arclength sum 12.0156 vs 15.2307). Same downstream signature, at least one has an upstream component — decide one defect or two before assigning an owner |
| **N3** | Seam splitting in the **production** section stage | INC 3's original gate restored: identical section topology at every tilt 0–45° in 1° steps; sph×sph `ds.split_edges` 0→1 and rel vol 1.7e-1 → <1e-6 | sph×sph `SECSTAT trim_paves=0 seam_paves=0 blocks=1` — the section circle is never cut at either sphere's seam and no operand edge is split. Same defect as roty24 `sec.curves` 3 vs 2. This is the localisation INC 3's and INC 5's kill criteria both demanded |
| **N4** | The assembly-side defect isolated by roty23 | sph×cyl roty23 `res.solids` 1; cut within 1e-9 of the **converged 50.3880515** — explicitly not of OCCT's 50.3846998 | roty23 has every upstream stage byte-identical to OCCT and still returns `res.solids` 1 vs 0. INC 5's kill criterion assumed the band failure was purely upstream; roty23 says assembly, roty24 says upstream. Both are real |
| **N5** | Speed: replace `v2_shell_signed_volume` / `v2_shell_is_hole` display triangulation (`src/v2/brep_v2_solid.cpp:487,512,531`) with `brep_massprops`' bounded-budget signed volume | v2 ladder median < 100 ms (today 3072); box×box cut result-mesh < 50 ms (today 4146 ms / 9408 tris / 2412 trim CVs); **all 144 ladder verdicts bit-identical before/after** | 40/40 gdb samples land in that one stack; it runs 5× per boolean at display quality. Sub-item, needs its own owner and a full v1 regression: hoist `is_valid()` out of `NurbsCurve::point_at` (`src/nurbscurve.cpp:2104`, scan at `:824`) — a ~10^4 lever on trimmed-face meshing, shared v1 code. **Treat the 30× trim-loop density (84 vs 2412 CVs for an identical 9-face result) as a correctness signal in the N1/N2 family, not only a perf one** |
| **N6** | STEP reader `VERTEX_LOOP` branch (`src/file_step.cpp:1203` and `:1516`) + writer seam-alias fix (both pcurves emitted at u=2π, u=0 alias dropped) | 5/5 OCCT and FreeCAD primitives round-trip to rel dvol < 1e-12; torus re-read at 19.34442463 (today 9.6722123, resid 0.36); FALSE_PASS 0/30 per kernel | Converts 3 silently-wrong FALSE_PASS cells and 6 empty/zero-volume cells **per kernel** into real measurements. Without it INC 9 is unmeasurable rather than failing. **Needs an owner** — `file_step.cpp` is on the do-not-touch list of all three tracks that located the defects |
| **N7** | Restate the T2/T3 definition of done and the v2 front-end acceptance predicate | Plan change, needs sign-off. Success measure: v2 attempts ≥1 of 12 chair configs without delegating (today **0/12**) | `split_faithful` requires `v2_verdict(...).closed()` on split operands at the default budget, and the **un-split** chair fails that same test (9.117e-03 vs a 1e-9 gate). The acceptance condition is unreachable on any imported trimmed solid — no amount of splitter work can satisfy it |
| **N8** | Wire `v2int::V2Interf` into the production boolean; replace `v2_dump.cpp`'s verbatim copy of `v2sol_run_front`/`attach_section_paves` with a one-line in-situ hook | Replica deleted; box×box `FIRST DIVERGENCE: none` reproduced through the hook; the five interference rows scored instead of reported | The production path calls none of INC 2's code, so INC 2's green gate says nothing about the shipped kernel. The replica has already drifted once (missing pave-attachment loop, caught as `ds.split_edges` 4 vs 0) |
| **N9** | Broad-phase reject (kernel-agnostic, unclaimed, benefits v1 too) | Disjoint-prism n=68 row drops > 2×; all ladder verdicts unchanged | 5.783 s of the current kernel's 16.107 s on a 68-face pair is spent on face pairs that cannot interfere — 0.97 ms (cur) / 1.40 ms (v2) per candidate pair, 36% / 15% of total runtime |

---

## 5. Premises this wave invalidated

1. **"The cone regression is in the section marcher" — FALSE for box×cone.** Section exact
   (`sec.block_nodes` 12/12, worst 6e-8); the arena then acquires 8 spurious new vertices. But
   cone×cone's section *is* 21% short. The two cone regressions share a downstream signature and
   do **not** demonstrably share a root. Anything scoped as "one cone fix" is mis-scoped (→ N2).

2. **OCCT is not truth near tangency, quantified.** At 23°: converged 50.3880515, OCCT
   50.3846998 (err 3.4e-3), v2 50.3880965 (err 4.5e-5) — **v2 is 75× closer than the column it is
   being diffed against**. Every OCCT-referenced `res.volume` FAIL near tangency is a difference,
   not a defect. §2 already said "not the bar"; it now has a number. Consequence: the T1 volume
   column must be scored against analytic values and the partition identity, never against OCCT.

3. **`closed()` at resid < 1e-9 is unreachable on imported trimmed data.** `chair0` alone is
   9.117e-03 at default budget and 6.37e-06 at 100×, with `max_chain_gap` 1.298e-04; a STEP
   round-trip control (0 / 5.95e-17 / 5.45e-18) eliminates STEP and budget as causes. This
   invalidates §2's T2 bar **and** the v2 front-end's acceptance predicate at once (→ N7), and
   makes `V2Verdict::closed()` unusable as the comparative metric on that corpus for either kernel.

4. **"v2 has been measured on multi-face and foreign corpora" — FALSE.** v2's front end refuses
   every chair config and every difficult foreign cell and delegates to `BRep::boolean`
   (`brep_v2_boolean.cpp:1255`): 36/36 chair cells, 25/28 interop control, 24/30 interop foreign
   are byte-identical to v1. Every "v2" number previously recorded on those corpora is a v1 number.
   The one figure that is v2's own — `SESSION_V2_NODELEGATE=1` — is 0/30 chairs against v1's 3/30.

5. **The "no pcurves ⇒ closure loss" hypothesis is RETRACTED, with the experiment.** Re-exporting
   four REFERENCE files through OCCT with 228–519 pcurves each gives identical verdicts to every
   printed digit; FreeCAD primitives carry 0 pcurves and import exactly; OCCT solids through
   `BRepBuilderAPI_NurbsConvert` import at vol 64 / 43 with resid 0. Surface type is eliminated
   too. The live suspect is trimmed-face construction in `file_step.cpp`: topology reads faithfully
   (27/29 files match OCCT's face/edge/naked/nonmanifold counts exactly) but trimmed **area** is
   1.15–6.76× too large (median 1.77×), and imported faces occupy space outside the true solid
   (re-export bbox z from −4.7239 where the source file's whole bbox starts at −3.74642).

6. **Volume is not translation-invariant on unsound imports** — 78.32326 at origin vs 2000.36581
   translated by (100,0,0), area unchanged at 327.238. Divergence-theorem volume over a
   non-closing shell is meaningless, so the volume column must be conditioned on `closed()` in
   every sweep; a volume delta on an open cell ranks nothing.

7. **The hang/blow-up premise is dead.** coneR×cyl completes all three ops; peak RSS 417 MB worst
   case against the 8.4 GB regression. The blow-up guard work item can be closed. What replaced it
   is a measurement hazard: the longest legitimate operation is 469.8 s, which any harness with a
   300 s budget reports as a hang — and T3 lost 11 cells to exactly that. Raise budgets; do not
   chase those as defects.

8. **INC 2's gate does not certify the shipped kernel** (see N8) and **INC 4's gate does not
   generalise past analytic faces** (62/62 on primitives, 18–46 naked edges on a chair). Both
   modules are green and neither result transfers to the corpus the plan uses them on.

**Process.** T3 self-caught and corrected two measurement errors mid-run — block-buffered `printf`
discarded by `SIGTERM` (which had inverted "v2 never reached depth 1"; it had completed in 15.8 s),
and probing only `breps[0]` of files holding up to 3 solids (which had faked a 4-face reader loss).
That is §5 working as designed. The two remaining process holes are the hand-built binaries outside
CMake and the missing baseline artifact — both in N0.
