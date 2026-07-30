# B-Rep boolean kernel v2 — the architecture, ground up

2026-07-26. This replaces incremental repair of the current splitter. It is the OCCT
architecture (BOPAlgo/BOPDS), adapted to this codebase, written as an implementation
plan with data structures, stage order, invariants and gates.

**Why v1 cannot be repaired into v2**: v1 splits each operand independently and then
reconciles the two results numerically. Every defect class measured in this campaign is
downstream of that choice — one-sided segment loss, divergent duplicate curves, junction
gaps, non-manifold edges, detached shells. v2 makes those states *unrepresentable*.

**What v1 got right and v2 keeps**: analytic surface intersection (45/45 axis-aligned,
1e-14), the STEP reader's analytic surface recognition, the point classifier's radial
test (0/42 vs oracle), the same-domain subsystem (`brep_samedomain`, A-op-A green), the
common-block subsystem (`brep_commonblock`, 73/73), and the entire validation corpus.

---

## 0. The five laws v2 enforces structurally

These are not style guidance; each maps to a data-structure decision below.

1. **One entity per geometric feature.** An intersection edge is created ONCE and
   referenced by every face that touches it. Duplicate-curve and one-sided-loss defects
   become unrepresentable rather than rare.
2. **Tolerances live in model space.** Never in parameter space. A UV domain is an
   arbitrary exporter choice; `min(range)*2e-3` measures nothing physical. Every
   tolerance is a 3D distance, converted to UV through the surface metric at the point
   of use.
3. **Analytic identity survives I/O and transforms.** A cylinder read from STEP is a
   cylinder with exact parameters; a rotated cylinder is a cylinder with axis `R·a`.
   Never re-derive by fitting what is already known exactly.
4. **Degeneracy is typed, not survived.** Coincidence, tangency, seams and poles produce
   named outcomes, never lucky near-zeros.
5. **Classification is seeded once and propagated topologically.** Crossing a section
   edge flips in/out; crossing an ordinary edge preserves it. N independent per-face
   verdicts is N chances to be wrong.

---

## 1. The data structure (this is the whole design)

New file `src/brep_bds.h/.cpp` — the shared intersection state. Modelled on OCCT's
`BOPDS_DS` (see `kb/audit_occt_pavefiller-core.md`; source at
`/home/petras/code/code_cpp/OCCT/src/ModelingAlgorithms/TKBO/BOPDS/`).

```
BdsVertex   { Point p; double tol; int origin; }        // fused; one per location
BdsPave     { int edge; double t; int vertex; }         // a vertex sitting ON an edge
BdsPaveBlock{ int edge; Pave t0, t1; int curve;         // one INTERVAL of an edge
              int common_block; }                       // -1 if not shared
BdsCommonBlock { vector<int> pave_blocks;               // pieces that coincide
                 vector<int> faces; double tol; }       // ONE geometric reality
BdsFaceInfo { set<int> pave_blocks_in, _on, _out;       // the face's own boundary state
              set<int> vertices_in, _on; }
BdsInterf   { enum {VV,VE,EE,VF,EF,FF} type; int a,b;   // typed interference record
              int new_vertex; vector<int> new_curves; }
BdsShape    { int index; ShapeType t; vector<int> subs; // the flat arena of both operands
              int operand;  /*0=A 1=B*/ }
```

Two rules make the laws structural:
- **Nothing is split.** `BdsPaveBlock` describes an interval; the original edge is never
  mutated. Splitting is a *view* materialised at the end.
- **Coincidence is one object.** Two coincident edge pieces are one `BdsCommonBlock`
  referencing both `BdsPaveBlock`s. There is no second copy to diverge.

`SharedEdgePool` from the BOP2 work is the partial ancestor of `BdsPaveBlock` and should
be absorbed, not paralleled — session B's audit found it lacks per-block tolerance, a
faces list, coincidence paves, orientation relation and member provenance, and that it
is only ever populated from a section scaffold (so it is empty for pure coincidence,
which is the case it is most needed for).

---

## 2. The pipeline — fixed stage order, each stage total before the next begins

OCCT's `BOPAlgo_PaveFiller` runs interference types in a strict order because each
creates entities the next must see. v1 has no equivalent of stages 4–5 at all
(`kb/hunt_efvf_gap.md`).

| # | stage | creates | our status |
|---|---|---|---|
| 0 | **Prepare** | flat shape arena, per-entity tolerances, bounding structures | new |
| 1 | **VV** vertex/vertex | fused vertices (one per coincident cluster) | new |
| 2 | **VE** vertex/edge | paves on edges where a vertex lies on them | new |
| 3 | **EE** edge/edge | new vertices at crossings; **common blocks** for overlaps | partial |
| 4 | **VF** vertex/face | paves where a vertex lies on a face | **MISSING** |
| 5 | **EF** edge/face | **new vertices where an edge pierces a face** | **MISSING** |
| 6 | **FF** face/face | section curves (analytic first, marching only if needed) | have (v1 SSI) |
| 7 | **PostTreatFF** | section curves become real edges, **fused across pairs** | partial |
| 8 | **Common blocks** | coincident pieces unified, tolerances computed | have (B) |
| 9 | **Split images** | each face/edge → its list of pieces, built FROM the paves | new |
| 10 | **Build** | select pieces per op; assemble shells; orient | rewrite |

**Stage 5 is the one whose absence is measurable.** When an edge of A pierces a face of
B, there must be a vertex there. Without it the arrangement has no node to route through,
which is consistent with the "lost regions" family (z15 missing 18 reference faces).

**Stage 6 keeps v1's analytic SSI** — it is the part that works. It needs the coverage
gaps filled (`kb/spec_analytic_ssi_canonical.md`): cyl×cyl skew/unequal-radius,
cyl×cone non-coaxial, cyl×sphere off-axis, cone×cone (no dispatcher arm at all). OCCT
never marches a plane/cyl/cone/sphere pair; neither should we.

---

## 3. Tolerance model

Per-entity, model-space, growth recorded at merge sites (`kb/audit_occt_tolerance-model.md`
— note the audited correction: monotone growth is design intent, not an OCCT invariant;
five shrink-capable writers exist, so port per-site).

```
tol(vertex) >= tol(edge) >= tol(face)   wherever incident        // containment
merge at distance d  =>  tol := max(tol, d)                      // record what you absorbed
compare against tol(a) + tol(b) + fuzz                           // never a global epsilon
uv_tol = tol3d / |dS/du|                                         // metric conversion, per use
```

The last line is the fix for the defect found on 2026-07-25: a STEP round-trip returns a
box face on `u[-0.04, 4.04]` instead of `u[0,1]`, and every domain-relative constant
(`samp_tol = max(range)*2e-5`, `eps_border = min(range)*2e-3`, `snap_uv`,
`scaf_forced_eps`) silently inflates 4×. Model-space tolerances are invariant to that by
construction; normalising incoming domains would only hide it.

---

## 4. Build (stage 10) — selection, not re-classification

For each face image piece, its state relative to the other solid is determined ONCE:
- pieces created by a section edge inherit their side from the section's orientation;
- pieces with no section boundary take one seed classification (the radial test, which
  already matches the oracle 0/42) and propagate to neighbours across ordinary edges.

Then the op-table (session B, verified against `BOPAlgo_Builder.cxx:641-737` and
independently against the audit) selects: keep-once for same-domain walls per the op's
OUT-state side; CUT reverses tool faces; CUT21 keeps the tools' orientation.

Assembly is by shared entity: two pieces that reference the same `BdsPaveBlock` are
adjacent by construction. **No sewing. No tolerance-based matching. No fuzzy joins.**

---

## 5. Migration — v2 grows beside v1, gated, corpus-judged at every step

`SESSION_V2` selects the new path. v1 remains default until v2 beats it on the corpus.

| step | deliverable | gate (all oracle-free unless noted) |
|---|---|---|
| M0 | model-space tolerance conversion in v1 | STEP round-trip box cell: 6 faces, naked 0 (currently 2 faces, 8 naked) |
| M1 | `brep_bds` structures + stage 0/1/2 | unit tests: vertex fusion idempotent, paves sorted/unique |
| M2 | stage 3 EE + absorb `SharedEdgePool` | A-op-A stays green; base chairs exact |
| M3 | **stages 4/5 VF/EF** | every true edge-face piercing has a vertex (computable independently) |
| M4 | stage 6 FF reusing v1 SSI + coverage gaps | oriented battery 18/18 at 1e-6 (today 3 OK / 12 wrong / 3 hang) |
| M5 | stages 7/8 PostTreatFF + common blocks (B's) | partial-coincidence cells from the stress design |
| M6 | stage 9 split images | base chairs exact via v2; matrix 45/45 via v2 |
| M7 | stage 10 build + op-table | rotated chairs; partition identity at every angle |
| M8 | v2 default, v1 deleted | full corpus non-decreasing; 3 primitive tiers green |

Each step is independently revertable and independently measured. **No step lands
without the guards battery**: base 3 ops exact, matrix (all 63 cells, oriented battery
UNMASKED), edge 54/54, minitests C++ 760, A-op-A, and partition identity on the
in-memory primitive sweep where operand volumes are analytically exact.

---

## 6. What is NOT in scope, deliberately

- Porting to Python/Rust. Measured: C++ boolean 6.68 ms vs Python ~5.0 s (750×); the
  chairs case would be 83–125 hours per op in Python, so a mirrored splitter could never
  be run against the corpus that defines its correctness. The splitter is a C++-only
  subsystem with a frozen public API (`kb/hunt_port_parity.md`).
- Matching OCCT's blemishes. OCCT fails BRepCheck on 3 of our 30 reference cells, is
  self-inconsistent on z15, produced NaN on 34 of 80 cross-pairs, and omits a legitimate
  lump of A\B in x13y29/y30. Match the mathematics, not the oracle.
- Zero failures as an acceptance bar. 38% of OCCT's own 25-year bug history is
  silently-wrong boolean output. The bar is: closure structural, correctness checkable
  without an oracle, accuracy stated, degeneracy typed, foreign geometry consumable,
  regression impossible to hide.
