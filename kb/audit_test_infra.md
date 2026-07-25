# Boolean test infrastructure audit (2026-07-24)

Inventory of every boolean battery/harness/oracle in session_cpp + validation/, then a
defect-class coverage matrix. Line refs are against main_7.cpp @ 4544770f.

## 1. main_7.cpp batteries (build: `cmake --build build --config Release --target main_7 --parallel 8`)

All batteries share one binary: `./build/Release/main_7.exe ["label filter"] [--refresh]`.
OCCT references come from `validation/occt_oracle/build/Release/oracle.exe` and are cached
full-precision in `validation/occt_cache.txt` (`--refresh` re-shells, ~20 min; cached ~30 s).
Cell gate: vol rel < 1e-6 AND exact OCCT face count AND `is_solid()` (both-empty passes).

| battery | invocation | cells | covers |
|---|---|---|---|
| Base matrix | `main_7.exe` (default) | 21 pairs x 3 ops = **63** (15 base + 6 oriented `*R`; `SESSION_NO_ROT` -> 45) | box/sph/cyl/cone/tor deep transversal penetrations; tilted cone (all-ellipse sections); rotated/skew operands off the iso-aligned comfort zone; periodic seams implicitly crossed |
| XOR/split check | `SESSION_XOR_CHECK=1 main_7.exe` | +1 per pair (xor + split identities vs cached fuse/common) | composite ops: vol(xor)=fuse-common, sum(split)=fuse |
| Edge grid | `SESSION_EDGE=1 main_7.exe` | 18 pairs x 3 ops = **54** | same-domain (A op A x4: box/sph/cyl/tor), coincident faces (full/partial/coplanar/coplanar-3-axis), edge contact, vertex contact, containment, disjoint, point tangency ext/int, inscribed sphere (6 tangencies), cyl line tangency, flush caps, linked-but-disjoint (cyl through torus hole) |
| OCCT2 hard cells | `SESSION_OCCT2=1 main_7.exe` | 6 pairs x 3 ops + 4 chains = **22** | cone base-on-face coincidence, torus wrapping coaxial cyl (R==r_cyl, seam tangency), off-axis cyl piercing sphere seam, grazing rotated box x sphere (bug32470), equal cylinders crossed 45 (bug27274), twin tori offset by major radius (bug29910); chains: 3-orthogonal-tori fuse, Steinmetz tricylinder common, washer annulus cut+common, 8-sphere progressive fuse (boolean-of-boolean via oracle `boolean_chain`) |
| OCCT suite | `SESSION_OCCT_SUITE=1 main_7.exe` | **120** (one op per cell; occt_suite.h, OCCT_SUITE_NOTES.md) | harvest of OCCT's own tests/boolean + bugs/modalg_1..8: line/point tangencies, coincident faces (flush caps, base-on-face, face-to-face), seam-crossing configs, containment/void results, same-domain, 1e-4 sliver-gap fuses (bug25722/bug25477_1, fuzzy-dependent), grazing rotated operands. By pair: box-cyl 41, cyl-cyl 26, box-box 19, cyl-sph 13, cyl-tor 9, box-sph 9, sph-sph 2, box-cone 1. By op: cut 64, fuse 34, common 22 |
| Freeform blob | `SESSION_FREEFORM=1 main_7.exe` | **1** verdict (3 ops + 2 identities + verbatim-NURBS oracle cmp) + pillow print | perturbed-sphere closed NURBS (quadric recognition rejected -> general marcher) x box; seam C0 kink; multi-face bicubic "pillow" box (chair-cushion class, watertight, exact box edges) |
| Cavity | `SESSION_CAVITY=1 main_7.exe` | **1** | full containment cut (box minus strictly-interior sphere -> internal void shell) |
| Chairs base | `SESSION_CHAIRS=<dir> main_7.exe` (dir = serialization/boolean_steps/chairs) | 3 ops + 2 volume identities | real-world imported STEP freeform x freeform; audits: `SESSION_CORNER_AUDIT`, `SESSION_WIREGAP`, `SESSION_CHAIRS_DUMP`, `SESSION_FAST` (skip volume/STEP), `SESSION_OP=<op>` single op |
| Chairs ROT | `+ SESSION_CHAIRS_ROT=1` | 10 deterministic configs x 3 ops = **30** (z15 z30 z45 z90 x20 y30 z30x20 z37 x13y29 z63; `SESSION_ROT_ONLY=<cfg>` one) | rotation robustness about joint centroid; 3 odd angles hit no symmetry; writes rot/B_<cfg>.step for oracle; `SESSION_ROT_STEP=1` exports results, `SESSION_TOPO_CHECK=1` topology_report |
| Chairs RND | `+ SESSION_ROT_RANDOM=N [SESSION_ROT_SEED=s]` | N x 3 ops, deterministic PRNG (alternating axis-angle / Euler) | randomized-orientation sweep; writes rnd/B_rNNN.step for oracle |
| Z90 probe | `+ SESSION_Z90_PROBE=1` | 2 known-bad points, 5 classifiers | point-in-solid classifier shootout (winding/ray-parity/oriented-omega/EXACT-trimmed-closest) vs OCCT verdicts, seconds not minutes |
| ROT_VOL probe | `+ SESSION_ROT_VOL=1` | 11 configs, no boolean | rotation-robust orientation classifier: per-face-sign flux volume must stay +80.30, flips=0 |
| Naked trichotomy | `SESSION_ANALYZE_NAKED=<dir> main_7.exe` | offline, per naked edge | classification drop vs SSI incompleteness vs sew miss (needs chair_cut.pb + split_A2/B2.pb dumps) |
| Diagnostics | `SESSION_PAIR_SSI`, `SESSION_SSI_DBG`, `SESSION_SOLID_DBG`, `SESSION_ORIENT_DBG`, `SESSION_DUMP_PB`, `SESSION_PB_DUMP`, `SESSION_STEP_DIR`, `SESSION_MERGE` (opt-in display merge), `SESSION_STEP_PRIMS`, `SESSION_BOOL_SHARED_EDGES` | — | per-cell SSI curve dump, unmated-edge listing, trim-orientation check, pb/STEP export (A red / B blue / result green) |

## 2. main_9.cpp — imported-STEP pair harness

`main_9 <A.step[:i]> <B.step[:j]> <cut|common|fuse|all> [out_dir]` (`:i` picks solid i, so a
colored triple file can supply both operands: `f.step:0 f.step:1`). Prints machine-parseable
`RES op=<op> faces=N solid=0|1 naked=N vol=V` + topology_report; writes colored triple STEP per
op; exit 1 if any op is open/naked/bad-topology. Cells: 1-3 per invocation, arbitrary imported
geometry (step_crash_test etc.). No built-in oracle — pair with step_probe for truth.

## 3. validation/ oracle tools

- **oracle.exe** `<req.txt> <out.txt>` (occt_oracle/, OCCT V8_0_0_rc2 static, ModelingAlgorithms
  only — no STEP). Request ops: `OP ssi` (bounded-face Section), `OP interpolate`,
  `OP boolean` (`MODE cut|common|fuse` + 2x `SHAPE kind params XF tx ty tz ax ay az deg`;
  `SHAPE nurbs` accepts a verbatim surface), `OP boolean_chain` (`NOPS n` + `MODES ...` + n+1
  shapes). Returns VOLUME/NFACES (booleans). Backs occt_cache.txt for all main_7 batteries.
- **step_probe.exe** (step_probe/, DataExchange OCCT) — 4 usage forms:
  1. `step_probe file.step` — strict-importer probe: ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID
     (Rhino-proxy gate); `-w` per-face wire/pcurve ranges; `-c` per-subshape BRepCheck failures.
  2. `step_probe --make-ref out.step [bsphere|tortor]` — OCCT-authored reference STEP to diff
     our writer against.
  3. `step_probe --inside file.step x y z [x y z ...]` — exact solid classifier IN/OUT/ON
     (truth for winding debates).
  4. `step_probe --cut|--common|--fuse A.step B.step` — boolean oracle on IMPORTED files:
     OP_SOLIDS/OP_FACES/OP_VOLUME/OP_VALID (the chairs-campaign authority).
- **OCCT_TRUTH.md** (serialization/boolean_steps/chairs/) — authoritative rotated-chair table:
  10 cfgs x 3 ops, vol / BRepCheck-valid / solid COUNT (cut is legitimately 2 solids on
  z15/z30/z45/z37/z63; common up to 3 on x13y29; z15-common empty and x20-common 5e-4 graze are
  detected-degenerate, not gated). Plus per-config blocker map from `step_probe -c`.
- **compare_boolean.py** — Python-side 15-pair x 3-op matrix vs oracle (same placements).
- **rhino_headless_probe.py / rhino_step_probe.py / rhino_closed_probe.py** — real Rhino
  (pythonnet3+coreclr, RhinoDoc.CreateHeadless) STEP acceptance verdicts; manual, not gated.

## 4. Bash scripts (run from session_cpp/)

- **validate_oracle.sh** `<exe> "<SESSION_ flags>" "<cfgs>" <op>` — the oracle gate for chairs
  ROT: phase 1 runs all configs in parallel (`SESSION_ROT_ONLY` per job, `SESSION_FAST=1`,
  private exe recommended); phase 2 gates each `res_<cfg>_<op>.step` on: OCCT-VALID + solids ==
  OCCT OP_SOLIDS + vol within 1% of OCCT's own boolean. Degeneracy DETECTED (OP_VALID=0 or
  |vol|<0.01), never hardcoded. Prints ORACLE-VALID n/10.
- **validate_rot.sh** `<exe> det|rnd <op> [N|cfgs...]` — v1 table (7-cfg default), interleaved.
- **validate_rot2.sh** `<exe> <op> det|rnd [cfgs|N]` — v2: per-config logs (/tmp/vr2_logs),
  truth cached in `validation/rot_truth_<op>.txt`, topology_report column, BRepCheck failure
  count, verdict SOLID/OPEN+OCCTVALID.
- **bash/minitest.sh --cpp** — brep_test.cpp minitests include fixed boolean cells
  (box x cyl / box x sph counts+volumes, brep_booleans example, boolean bench vs OCCT times).

## 5. COVERAGE MATRIX — defect classes x batteries

Legend: X = gated cells, o = present but not gated / incidental, blank = absent.
Batteries: MTX=base matrix(63), EDGE(54), OC2=OCCT2(22), SUITE(120), FF=freeform(1),
CAV=cavity(1), CH=chairs base(3), ROT(30), RND(3N), M9=main_9.

| defect class | MTX | EDGE | OC2 | SUITE | FF | CAV | CH | ROT | RND | M9 |
|---|---|---|---|---|---|---|---|---|---|---|
| transversal (general position) | X | o | X | X | X | | X | X | X | o |
| tangency point (ext/int/inscribed) | | X | X | X | | | | | | |
| tangency line (cyl-cyl, wall, seam-wrap) | | X | X | X | | | | | | |
| coincidence / same-domain (A==A, flush faces, base-on-face) | | X | X | X | | | | | | |
| graze / near-tangency (sub-tol sliver sections) | | | o | o | | | | o | o | |
| containment (full A-in-B, void/cavity) | | X | | X | | X | | | | |
| disjoint (incl. linked-but-disjoint, void common) | | X | | X | | | | | | |
| seam/pole (periodic seam crossing; pole crossing) | o | o | X | X | o | | o | o | o | |
| thin-wall (sub-tol wall / 1e-4 gap weld) | | | | o | | | | | | |
| self-intersection (self-int input; result self-int wire) | | | | | | | | o | | |

Row notes:
- graze: oBOXG (OC2) and SUITE grazing cells ARE gated, but only as single fixed placements;
  x20-common (5e-4) is explicitly excluded as OCCT-degenerate — no epsilon-sweep family exists.
- seam/pole: seams well covered (explicit OC2/SUITE cells); POLE crossing only incidental
  (any box x sph cut near poles) — no cell isolates a section through a sphere/torus pole.
- thin-wall: SUITE bug25722/bug25477_1 are fuzzy-tolerance gap-fuses whose "both outcomes
  defensible" note means they are effectively ungated; no sub-tolerance-wall result cell.
- self-int: SelfIntersectingWire appears only as an OCCT diagnosis of OUR z45/x13y29 failures
  (OCCT_TRUTH blocker map), never as a constructed input or a gated cell.

## 6. GAPS (ranked)

1. **Freeform/imported tangency+coincidence**: every tangency, same-domain, coincident-face,
   containment and disjoint cell is quadric-primitive only. Chairs/blob exercise ONLY
   transversal. No freeform A op A, no freeform flush-face, no freeform containment cell
   (same-domain extra_cuts infra exists in the kernel but no battery drives it).
2. **No near-tangency epsilon sweep**: graze coverage is 2-3 fixed placements; no family that
   walks a tangency offset through 1e-2..1e-9 to find the tolerance cliff (the known z45/x13y29
   marcher failure class is exactly this and has no primitive-battery reproduction).
3. **Thin-wall absent**: no cell produces a result wall thinner than sew/weld tolerance
   (e.g. box cut box offset by 1e-3), the classic over-merge killer; the two SUITE gap cells
   are declared both-outcomes-defensible.
4. **Pole-crossing not isolated**: no cell forces a section curve through a sphere/torus pole
   or a cone apex (apex-through-face is also absent; upright-cone hyperbola sections are
   deliberately kept OUT of the gate — a parked known gap).
5. **Self-intersection class untested**: no self-intersecting input operand, and result-wire
   self-intersection is only observed post-hoc via step_probe -c, not asserted by any battery.
6. **xor/split gated only on the 15 base pairs** (SESSION_XOR_CHECK); no xor for EDGE
   (tangency/coincidence xor is the hardest composite), OCCT2, chairs.
7. **Dead cell**: `ecylO` (parallel-axis PROPER-overlap cylinders) is defined in
   edge_placements() but absent from edge_pairs() — written, never run.
8. **common op second-class in harnesses**: validate_oracle.sh/validate_rot*.sh default to cut;
   OCCT_TRUTH marks common hardest (multi-solid, degenerate cells) and it is validated last.
9. **main_9 has no oracle hookup**: RES lines gate on internal is_solid/naked only — the exact
   false-positive validate_oracle.sh was built to kill; needs a --probe mode or wrapper.
10. **Rhino acceptance not a gate**: rhino_*_probe.py are manual; no battery fails when Rhino
    drops trims/wires (known step_probe-VALID-but-Rhino-open class).
11. **OCCT-suite untranslatable categories** (SUITE notes): truncated cone (214 files — biggest
    locked region: cone x box/cyl tangency grid), prism/revol solids, angle-limited primitives
    (seam+planar-face interaction), multi-operand bbop/general fuse, non-axis rotation
    composites — all zero coverage.
12. **Base matrix rotated cells excluded under SESSION_NO_ROT** in the STEP-regen workflow
    (hang avoidance) — the oriented battery silently drops out of that gate.
