# STEP import corpus — reader/boolean test sources

Corpus of STEP files to read BReps from, ordered by difficulty. Goal: `read_file_step_breps` →
`is_solid`/`volume()` correct → booleans on imported breps (chairs campaign).

## Tier 1 — our own writer (round-trip, 109 files)
`serialization/boolean_steps/*.step` — every matrix/battery/freeform/prim result, probe-verified
volumes in its README.md. Gauntlet: read back, compare volume/faces/is_solid against README values.

## Tier 2 — OCCT-authored exports of known solids
Generate via `step_probe --make-ref` pattern (extend to all 5 primitives + 2-3 boolean results):
OCCT's own STEP of solids whose exact volumes we know. Cross-vendor read with known truth.

## Tier 3 — real-world CAD exports (in this dir / chairs)
- `../boolean_steps/chairs/chair0.stp`, `chair1.stp` — ST-Developer exports, 20 faces each,
  OCCT truth vol(chair0)=80.2969. KNOWN FAILING: our volume()=0, cut not solid.
- `screw.step` (88 KB) — OCCT test data (helical/blend faces).
- `linkrods.step` (1.8 MB) — OCCT test data (assembly, multiple products).

## Tier 4 — to generate: OCCT .brep → STEP conversions
Source: `validation/occt_oracle/build/deps/occt/src/occt/data/occ/` (BRepTools format).
Build a small brep2step tool against the oracle's OCCT build (BRepTools::Read + STEPControl_Writer):
`bottle.brep` (tutorial bottle: fillets, shell, threads), `hammer.brep`, `CrankArm.brep`,
`Ball.brep`, `fuse.brep`, `Axis_of_bearing.brep`. Skip `.rle` (compressed).

## Tier 5 — external (manual/user)
- Rhino-exported STEP of our own primitives (user has Rhino; Rhino is the stricter oracle).
- NIST STEP file repository (AP203/AP214 conformance models) — download when needed.

## Most meaningful MISSING tests (kernel + reader)
Boolean kernel cells (main_7):
1. Scale-disparate operands 1000:1 (on_eps asymmetry bug).
2. Thin plate cut, thickness < 4e-3·diag (on_eps + sew diag*5e-3 ceiling).
3. Two features closer than 0.5%·diag, e.g. twin near holes (sew false-merge).
4. Whole matrix at 1e-3 and 1e+3 model scale (q6 absolute weld quantization).
5. Plane tangent to cylinder along a line (marcher tangency abort).
6. Tilted plane × torus (general spiric, marching-only path).
7. Non-coaxial cyl×sph / cyl×cone / cone×sph (analytic holes → marcher accuracy).
8. Boolean-of-boolean-result chains: (A−B)−C, fuse(cut,cut) (closure; phantom-cut class).
9. All-operands-rotated matrix (nothing axis-aligned; tilted-cone recognition).
10. Freeform × freeform (blob × pillow).
11. Near-coincident fuzzy pair: same box offset by 1e-7 (fuzzy tolerance semantics).
Reader (SESSION_IMPORT gauntlet):
12. Tier-1 round-trip scoreboard (109 files, volume/faces/solid vs README truth).
13. Tier-2/3/4 import scoreboard vs step_probe volumes (OCCT as oracle on same file).
14. Imported × kernel primitive boolean (chair0 − box), imported × imported (chair0 − chair1).
