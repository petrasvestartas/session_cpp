# T2 harvest plan — ABC-dataset boolean pairs

Scope of this document: how the T2 tier is produced, what a harvested case looks like,
and what is deliberately NOT done yet. Implementation: `corpus/t2_abc.py`
(`plan` / `index` / `sample` / `stats`). **No bulk download is performed by any tool in
this repo** — the sampler works only against a local mirror that a human populated.

## Tier map

| tier | content | size | oracle | where |
|---|---|---|---|---|
| T0 | chairs base + 10 rotated configs + matrix + edge batteries | 132 cells | re-derived OCCT truth + metamorphic invariants | `corpus/runner.py`, `corpus/t0_manifest.json` |
| T0R | rotated primitives + platonics: 45 pairs x N seeded rotations, plus angle sweeps | 2700 cells at N=20 | oracle-free invariants (continuity, equivariance, A-op-A, closed-form) + OCCT | `corpus/rotprim.py`, `corpus/t0r/` |
| T1 | OCCT test-suite ports (`SESSION_OCCT2=1`) + random-rotation battery (`SESSION_ROT_RANDOM`) | ~120 + N cells | kernel's OCCT cache | already in `main_7.cpp`; fold into the runner by adding a battery entry |
| T2 | ABC-dataset synthetic pairs (this document) | 10^3–10^4 pairs | OCCT per pair, identity-checked; invariants | `corpus/t2_abc.py`, `corpus/t2/manifest.jsonl` |
| T3 | Fusion 360 Gallery assembly joints (real designer contacts, 0.1 mm coincidence labels) | 32k joints | OCCT + contact labels | not started; **non-commercial license — internal diagnostics only** |

## Source

- ABC: ~1M CAD models from Onshape public documents, 100 chunks × 10k models, per-format
  7z archives (`step`, `para`, `stl2`, `meta`). Landing page
  <https://deep-geometry.github.io/abc-dataset/>, archives on NYU FDA (chunk 0000 =
  <https://archive.nyu.edu/handle/2451/44309>). Some chunks have been flaky
  (<https://github.com/deep-geometry/abc-dataset/issues/15>) — mirror once, keep locally.
- Licensing: dataset tooling MIT; model copyright stays with creators under Onshape ToU
  1.g.ii (public documents carry a royalty-free use/copy/modify/distribute license).
  Safe for internal testing. **Every manifest record keeps `provenance.model_a/model_b`
  so any derived case can be traced back.**
- Quality: expect duplicates and non-solids; OCC meshing fails on ~1.56 % of ABC
  (Better STEP, <https://arxiv.org/html/2506.05417v1>). The index stage rejects anything
  that is not an importable valid closed solid, and dedups by fingerprint.

## Fetch recipe (manual, one chunk at a time — NOT run by any tool here)

```bash
# 1 chunk of STEP files ~= 8-12 GB compressed; the whole STEP mirror is 100-200 GB.
mkdir -p /data/abc/step && cd /data/abc/step
curl -L -O <chunk-url-from-NYU-FDA>/abc_0000_step_v00.7z
7z x abc_0000_step_v00.7z          # -> abc_0000_step_v00/<model_id>/<model>.step
```
Start with ONE chunk (10k models). That is already ~3–5k usable unique solids after
filtering, i.e. far more pairs than the nightly budget.

## Stage 1 — index (`t2_abc.py index --mirror /data/abc/step`)

Per model: `step_probe <file>` → ROOTS/SOLIDS/SHELLS/FACES/VOLUME/VALID.
Keep only `SOLIDS>=1 AND VALID==1 AND |VOLUME|>0`; reject buckets are counted and
reported (`import-fail`, `not-valid-solid`, `duplicate`, `probe-error`).
Fingerprint = `(solids, shells, faces, round(volume,6))` — the AutoMate-style
topology+inertia recipe, cheap and effective for exact duplicates.
bbox comes from the 3D `CARTESIAN_POINT` hull (control points bound the NURBS geometry),
so no extra probe flag is needed.
Output: `corpus/t2/index.jsonl`, one record per surviving model.

## Stage 2 — pair sampling (`t2_abc.py sample --pairs N --seed S`)

Deterministic (`random.Random(seed)`), so a nightly run is reproducible:
1. draw two distinct models A, B;
2. draw a rigid motion for B: uniform random axis, angle in [0,360), translation chosen
   so B's bbox centre lands at a uniform point in the middle half of A's bbox
   (guarantees bbox overlap without forcing containment);
3. write the pair dir `corpus/t2/pairs/<id>/{chair0.stp,chair1.stp}` — B is re-emitted
   through `corpus/step_rigid.py` (rewrites 3D CARTESIAN_POINT/DIRECTION only; 2D pcurve
   data and all scalars are invariant under a rigid motion). Round-trip cost measured on
   chair0.stp: 4.3e-7 relative volume drift;
4. interference prefilter: classify ≤120 of B's control points against solid A with
   `step_probe --inside`; keep the pose only when at least one point is IN and one OUT
   (`class = partial`). `contained` / `disjoint-or-outside` poses are dropped unless
   `--keep-all` (they are cheap but low-value; containment is already covered by the
   edge battery);
5. confirm the interference with the oracle itself (`interference.occt_common`): the
   control-point test is loose because poles need not lie on the solid, so poses whose
   OCCT common volume is 0 are relabelled `empty-common` and dropped by default;
6. record OCCT truth for cut/common/fuse **plus the inclusion–exclusion verdict of that
   truth** (same test as `validation/rederive_truth.py`): each record carries
   `truth.identity.verdict = CONSISTENT | SELF-INCONSISTENT` and `odd_cell`. A
   SELF-INCONSISTENT pair is still a valid corpus case — it just may not be gated on
   OCCT's numbers, only on the invariants.

Smoke-tested end to end on a 4-model synthetic mirror (OCCT's own `--make-ref` shapes +
chair0): 4/4 sampled pairs came out `partial` with CONSISTENT oracle identity.

The pair dir layout is deliberately the chairs layout, so the existing kernel entry point
runs a T2 case with no C++ change:

```bash
env SESSION_CHAIRS=corpus/t2/pairs/<id> SESSION_OP=cut ./build/main_7 zzzz
```

which is exactly what `corpus/runner.py:run_chairs_base` drives. Wiring T2 into the
nightly ledger is therefore a manifest-loader change in `runner.py`
(battery `"t2"` → `run_chairs_base` with `wd = record["dir"]`), not new plumbing.

## Manifest schema

One JSON object per line in `corpus/t2/manifest.jsonl`:

```json
{"id": "00001234__00007777__p03",
 "dir": "corpus/t2/pairs/00001234__00007777__p03",
 "a": {"src": "...", "sha1": "...", "solids": 1, "faces": 44, "volume": 12.5,
       "bbox": [[x0,y0,z0],[x1,y1,z1]]},
 "b": {"...": "same shape as a"},
 "pose": {"axis": [0.31,-0.62,0.72], "deg": 214.7, "trans": [1.2,-0.4,3.9]},
 "interference": {"pts_in": 37, "pts_out": 83, "class": "partial"},
 "truth": {"cut":    {"vol": 9.1, "solids": 1, "faces": 51, "valid": 1},
           "common": {"vol": 3.4, "solids": 1, "faces": 22, "valid": 1},
           "fuse":   {"vol": 21.8, "solids": 1, "faces": 63, "valid": 1},
           "identity": {"implied_common_from_cut": 3.4,
                        "implied_common_from_fuse": 3.4,
                        "verdict": "CONSISTENT", "odd_cell": "",
                        "volA": 12.5, "volB": 12.7}},
 "provenance": {"dataset": "ABC", "model_a": "...", "model_b": "...",
                "license": "ABC/Onshape public-document terms; keep provenance"},
 "created": "2026-07-25T20:10:00", "sampler_seed": 1}
```

Stable id = `<modelA>__<modelB>__p<NN>`; a re-run with the same seed reproduces the same
ids, poses and dirs, so ledger diffs across nights are meaningful.

## Nightly budget

Chairs-class pairs cost ~80 s/op on this machine; ABC models are usually far smaller.
Budget 3 ops × 500 pairs at 8-way parallelism ≈ 2–4 h wall for a nightly T2 slice; sample
a fresh slice per night by advancing `--seed` and keep the T0 tier (132 cells, ~15 min)
as the fast gate on every kernel change.

## Deliberately not done yet

- No download, no mirror, no chunk stored in this repo (`corpus/t2/` is created on
  demand and holds only generated artifacts).
- No Fusion 360 (T3) harvesting: license is non-commercial research, so it must stay out
  of any redistributed suite; revisit when someone confirms the legal position.
- No mesh-boolean cross-oracle yet (would be the second opinion that replaces the now
  unavailable Rhino oracle on Linux).
