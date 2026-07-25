# Validation Corpora + Methodology for B-Rep Boolean Kernel Testing at Scale

Research survey (2026-07-24) for the session kernel program. Context: industry-grade tolerance-based NURBS B-Rep boolean kernel in C++, OCCT-aligned architecture. Master plan: P3 same-domain subsystem, P4 EE/EF interference, P5 tolerance model, P6 corpus validation. Goal here: catalog every usable CAD model corpus, how to mine boolean test pairs from each, and a concrete 10k-case nightly corpus design with an OCCT oracle differ.

---

## 1. Corpus catalog

### 1.1 ABC Dataset — 1M STEP models (primary volume source)

- **Size**: ~1,000,000 CAD models scraped from Onshape public documents. Distributed as 100 chunks x 10k models, 7z archives per format. Formats always available: `meta`, `step`, `para` (Parasolid), `stl2`; higher-quality `obj`/`feat`/`stat` for subsets.
- **Hosting**: https://deep-geometry.github.io/abc-dataset/ with chunk archives on NYU's Faculty Digital Archive (e.g. chunk 0000 = https://archive.nyu.edu/handle/2451/44309). Availability of some chunks has been flaky — see https://github.com/deep-geometry/abc-dataset/issues/15 . Mirror strategy: fetch once, archive locally (a full STEP-only mirror is roughly 100-200 GB compressed).
- **License**: dataset tooling MIT (https://github.com/deep-geometry/abc-dataset/blob/master/LICENSE); model copyright stays with creators under Onshape ToU 1.g.ii — public documents carry a worldwide royalty-free license to use/copy/modify/distribute/sell (https://www.onshape.com/en/legal/terms-of-use). Safe for internal testing; redistribution of derived test cases is defensible but keep provenance metadata per file.
- **Quality reality**: contains duplicates and models that are not valid solids; OpenCascade meshing fails on ~1.56% of ABC (measured by the Better STEP project, https://arxiv.org/html/2506.05417v1). Expect a similar or larger fraction to fail `BRepCheck_Analyzer` as closed solids. Dedup via B-Rep fingerprint (topology entity counts + inertia tensor + volume — the AutoMate recipe) is mandatory; expect the usable unique closed-solid pool to be a few hundred thousand.
- **Boolean pair harvesting**:
  1. **Synthetic pairing** (main): sample two valid closed solids, normalize each to unit bbox diagonal, generate relative poses that guarantee interference (bbox overlap test + fast mesh-boolean prefilter on the `stl2` meshes). Run cut/common/fuse.
  2. **Self-boolean stress**: `A op R(A)` with R in {z15, z30, z45, z30x20, random SO(3)} — exactly our rotated-chair protocol, generalized. This is the classic robustness protocol from the mesh-boolean literature (Zhou et al., Mesh Arrangements, https://dl.acm.org/doi/10.1145/2897824.2925901).
  3. **Multi-solid STEP files**: many ABC files contain assemblies/multiple bodies — these give *designer-placed* coincident/tangent contacts, i.e. natural P3 same-domain cases.
- **Derived resource**: **Better STEP** (2025, https://arxiv.org/html/2506.05417v1 , https://github.com/better-step/abs) — ABC + Fusion 360 + ~1M Onshape models re-extracted through OCC into a clean HDF5 geometry/topology format, pip-installable, hosted on FRDR (https://www.frdr-dfdr.ca/repo/dataset/d54b95e0-bc14-4236-b50b-922e5bf4ba7d). Useful as a pre-validated index of which ABC models are healthy, even if we consume the raw STEP ourselves.

### 1.2 Fusion 360 Gallery — B-Rep + assemblies with real contacts

- **Size/subsets** (https://github.com/AutodeskAILab/Fusion360GalleryDataset):
  - Segmentation: 35,858 bodies (~390k faces); Extended STEP: 42,912 STEP files.
  - Reconstruction: ~8.6k designs with full sketch+extrude construction sequences.
  - **Assembly: 8,251 assemblies, 154,468 bodies, 32,148 parametric joints**, with contact labels marking B-Rep face pairs coincident or within **0.1 mm** in the assembled state (https://github.com/AutodeskAILab/Fusion360GalleryDataset/blob/master/docs/assembly.md).
- **Formats**: native `.smt` (Autodesk Shape Manager — ground truth, minimal conversion error) + `.step` + `.obj` + JSON metadata.
- **License**: **non-commercial research license for the data** (custom Autodesk license in repo; code MIT). CAUTION for an industry-grade kernel program: use for internal research/diagnostics only, never in redistributed test suites or marketing benchmarks. AutoMate (below) is the commercial-safe substitute.
- **Boolean pair harvesting**: the Assembly/Joint data is the highest-value part — pairs of parts **in assembled pose with known coincident/tangent face contacts**. That is precisely the P3 same-domain + P4 EE/EF interference regime (faces touching within tolerance, edge-on-face slides, tangential contacts). Harvest: load joint pairs, place per joint pose, run cut/common/fuse; the 0.1 mm contact annotation tells you *a priori* which cases are same-domain-heavy. Also: joint axes give physically meaningful rotation sweeps (rotate about the actual hinge axis in 5-degree steps → tolerance-continuity curves).

### 1.3 MFCAD / MFCAD++ — machining features = mass-produced subtraction results

- **MFCAD**: 15,488 synthetic STEP models, per-face machining-feature labels, 16 planar feature classes (https://github.com/hducg/MFCAD, models under `dataset/step`). License not explicit in repo — verify before redistribution.
- **MFCAD++**: **59,655 STEP models, 24 feature classes incl. non-planar**, 3-10 intersecting features per model; labels embedded directly in `ADVANCED_FACE` names inside the STEP. Hosted at Queen's University Belfast Pure (https://pure.qub.ac.uk/en/datasets/mfcad-dataset-dataset-for-paper-hierarchical-cadnet-learning-from). Paper: Hierarchical CADNet (https://dl.acm.org/doi/10.1016/j.cad.2022.103226).
- **Boolean pair harvesting**: every model is *stock minus features* — i.e. already the answer to a boolean program. Two uses:
  1. **Reconstruct operands**: from the per-face labels, rebuild the feature tool solids (slot/pocket/hole primitives are parametric) and the stock block; then `stock - tool_i` must reproduce the labeled model region — an oracle-free consistency check.
  2. **Same-domain torture at volume**: features share planar/cylindrical faces with the stock; pairing any model with a translated copy of its own stock, or subtracting one model from another on a shared datum plane, generates thousands of exactly-coplanar and exactly-coaxial interactions — the P3 regime. Intersecting features (MFCAD++'s design goal) stress the multi-tool interaction logic.

### 1.4 DeepCAD — 178k boolean construction programs

- **Size**: 178,238 sketch+extrude construction sequences as JSON, parsed from Onshape public documents via ABC links (https://github.com/ChrisWu1997/DeepCAD, https://arxiv.org/abs/2105.09492). **MIT license** on the repo; underlying geometry under Onshape public-document terms (same as ABC).
- **STEP availability**: not shipped as STEP, but each JSON replays deterministically through pythonOCC/OCCT into a B-Rep → export STEP at any intermediate step.
- **Boolean pair harvesting** (unique property): each extrude carries a boolean flag — *new body / join / cut / intersect*. Replaying a sequence yields, at every step, an ordered operand pair (accumulated body, new extrusion) plus the designer-intended op. That is ~500k+ natural boolean cases with a strong "a production kernel accepted this" prior, and they are structurally biased toward **sketch-on-face coplanarity** (profiles drawn on existing faces) — the same-domain P3 subsystem's bread and butter. Harvest plan: replay with OCCT, dump `(A.step, B.step, op, occt_result_props)` per step.

### 1.5 Thingi10K — mesh-domain robustness protocol (not STEP)

- **Size**: 10,000 real 3D-printing meshes from Thingiverse 2009-2015 (https://github.com/Thingi10K/Thingi10K , https://huggingface.co/datasets/Thingi10K/Thingi10K , `pip install thingi10k`). Per-model licenses (various CC; 3,680 CC BY-SA). 4,523 of 10k meshes self-intersect; many non-manifold — intentionally dirty.
- **STEP**: none — mesh only. Not a NURBS operand source.
- **Value to us**:
  1. **Methodology import**: the mesh-boolean papers (Mesh Arrangements https://dl.acm.org/doi/10.1145/2897824.2925901 ; EMBER https://www.graphics.rwth-aachen.de/media/papers/339/ember_exact_mesh_booleans_via_efficient_and_robust_local_arrangements.pdf ; Interactive & Robust Mesh Booleans https://dl.acm.org/doi/10.1145/3550454.3555460 ; exact mesh CSG https://arxiv.org/abs/2405.12949) standardized "run all 10k + rotated-copy stress unions" as the robustness bar. Our per-corpus pass-rate reporting should copy this format (pass %, crash %, timeout %, invalid-output %).
  2. **Second-oracle fuel**: meshes feed the mesh-boolean cross-oracle (Section 2.3) used when OCCT itself is untrustworthy.
  3. Planar-B-Rep conversion (triangles → planar faces) gives an extreme planar same-domain arrangement stress if ever needed; low priority.

### 1.6 NIST MBE PMI test cases — small, surgical, public-domain

- **Size**: ~11 core models — 5 CTC + 6 FTC (421 PMI annotations) plus STC simplified variants; STEP AP242 and AP203 for each (https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0 , browser: https://pages.nist.gov/CAD-PMI-Testing/models.html). US-government work → effectively public domain; also mirrored by MBx-IF (https://www.mbx-if.org/home/cax/resources/).
- **Boolean harvesting**: not a pair source. Role: **STEP reader/writer conformance floor** (P5/P6) — our importer must round-trip all CTC/FTC/STC exactly (they contain production-grade fillets, drafted walls, patterned holes from CATIA/Creo/NX exports); then boolean each FTC against primitive tools (plane cuts, cylinder drills) as curated hard freeform cases. Also the reference material for a future tolerance/PMI story.

### 1.7 OCCT test suite + bug tracker — the densest adversarial mine

- **Test suite** (https://github.com/Open-Cascade-SAS/OCCT/wiki/tests): `tests/boolean/` has **39 grids** (verified via GitHub API): bcommon/bcut/bfuse/bopcommon/bopcut/bopfuse/bopsection/boptuc x {2d, simple, complex}, plus `cells_test`, `history`, `volumemaker`, `splitter`, `gdml_public/private`, `opensolid`, `periodicity`, `removefeatures`, `simplify`, `mkconnected`, `bcutblend`. The regression group `tests/bugs/modalg_1..8` holds boolean-related Mantis fallout — **modalg_7 alone = 577 cases** (verified); total boolean-relevant cases across boolean/ + modalg is in the several-thousand range. Each `.tcl` case embeds its oracle: `checkshape`, `checkprops -s/-v <expected>` with tolerance, `checknbshapes` counts.
- **Test data**: public dataset of **2,500+ shapes** (BREP/IGES/STEP) downloadable from dev.opencascade.org (announcement: https://dev.opencascade.org/content/open-cascade-technology-testing-dataset-published); raised OCCT's own public test coverage to ~60%. Some referenced data files remain OCCT-private — those cases are unusable.
- **Bug tracker**: Mantis is decommissioned into a read-only archive; live tracker is GitHub Issues (https://github.com/Open-Cascade-SAS/OCCT/issues , migration notes: https://github.com/Open-Cascade-SAS/OCCT/discussions/36). New boolean issues frequently attach `.brep`/`.step` repro files — a slow but continuing drip of fresh adversarial pairs.
- **License**: LGPL-2.1-with-exception covers code, scripts, and data. Fine for an internal test corpus; keep the harvested cases in a clearly LGPL-attributed subdirectory.
- **Harvest plan** (highest ROI per case in this whole document): write a tcl-script scraper that, per case: `restore`s the operand `.brep` files, converts to STEP via OCCT, records op + expected `checkprops`/`checknbshapes` values as the oracle row. Every one of these was once a real kernel bug — they are pre-minimized, pre-triaged, and come with numeric expectations.

### 1.8 FreeCAD bug tracker — human-confirmed boolean failures with attachments

- **Tracker**: GitHub https://github.com/FreeCAD/FreeCAD/issues (search `boolean` + labels; e.g. #5619 "Boolean Difference leaves an extra hole (upstream OCC bug)", #5782 "Boolean fails depending on placement", #18047 offset-solid boolean failure); legacy Mantis archive at https://tracker.freecad.org/ .
- **Gold detail**: attachments are `.FCStd` files = **zip containers holding raw OCCT `.brp` (BRep) blobs** per object — operands extractable without running FreeCAD: unzip, pull `*.brp`, `BRepTools::Read`, replay the failing op. Placement-dependent failures (a recurring FreeCAD theme) are exactly our rotated-chair robustness class.
- **Harvest plan**: `gh api search/issues 'repo:FreeCAD/FreeCAD boolean in:title'` → download issue attachments → unzip → extract `.brp` pairs + reported op → tag with issue URL. Expect low hundreds of usable curated cases. License: treat as user-contributed repro data, internal-use; keep issue-URL provenance.

### 1.9 AutoMate — commercial-safe assembly contacts at scale

- **Size**: **451,967 unique parts, 255,211 assemblies, 1,292,016 mates** from Onshape public documents; parts as **STEP + Parasolid**, assemblies/mates as JSON + parquet (https://arxiv.org/abs/2105.12238 , https://degravity.github.io/automate/).
- **License**: hosted on Dryad (https://datadryad.org/dataset/doi:10.5061/dryad.2547d7wvw) — Dryad publishes data under **CC0**, which removes the Fusion-360 non-commercial problem. This is the scale + license winner for contact-pose boolean pairs.
- **Harvest plan**: mates carry mating coordinate frames → place part pairs in mated pose (guaranteed realistic contact: coaxial pins, coplanar flanges, tangent seats) → boolean. Deduplicate with their own B-Rep fingerprint scheme (topology counts + inertia tensor + center of mass — reuse it for ABC too). Also useful: their preprocessing already removed corrupted/duplicate parts.

### 1.10 STEP file zoos (reader-robustness, not pair volume)

- **CAx-IF / MBx-IF library** — vendor-exported AP242/AP203/AP214 test-round files from CATIA/NX/Creo/SolidWorks etc. (https://www.mbx-if.org/home/cax/resources/ , https://www.cax-if.org/cax/cax_introduction.php). Best source of *cross-vendor STEP dialect* stress for the importer; free to download.
- **NIST STEP File Analyzer** sample files + syntax checker (https://www.nist.gov/services-resources/software/step-file-analyzer-and-viewer) — run SFA over our *written* STEP output as an independent conformance lint.
- **IDA-STEP examples** (https://www.ida-step.net/example/step_file) — small, historic, marginal.
- **GrabCAD** (https://grabcad.com/library/tag/step) — millions of models but ToS forbids bulk scraping; manual cherry-picking only. Do not build automation on it. Same warning for fresh Onshape scraping: Onshape ToU prohibits robots/data-mining (https://www.onshape.com/en/legal/terms-of-use) — use the already-published academic snapshots (ABC, DeepCAD, AutoMate) instead of re-scraping.

---

## 2. Validation methodology

### 2.1 OCCT oracle differ (primary oracle)

Per case `(A, B, op, pose)` run our kernel and OCCT `BRepAlgoAPI_{Cut,Common,Fuse,Section}` (pin one OCCT version, currently 7.8.x; record version in every result row). Compare, in escalating cost order:

1. **Self-validity of our result** (no oracle needed): closed-shell check, naked/free edge count = 0, `BRepCheck_Analyzer`-equivalent, orientation consistency, tolerance bounds. OCCT's own argument checks (`bopargcheck`) applied to inputs decide whether a case is *eligible* — invalid inputs go to a separate "garbage-in" track, never the pass/fail scoreboard.
2. **Mass properties**: volume, area, centroid vs OCCT `GProp` with relative tolerance ladder (1e-6 exact-tier, 1e-4 freeform-tier — matches our current chairs rels 1e-4..1e-5).
3. **Topology counts**: faces/edges/vertices/shells/solids vs `checknbshapes`; allow a mapped-equivalence band (merged coplanar faces etc.) rather than strict equality — OCCT face counts are convention-dependent.
4. **Point-membership differ**: sample ~1k points stratified near the section curves + uniform in bbox; classify in/out against both results. Catches orientation/flip bugs that volume comparison misses (a flipped thin sliver changes volume by ~0, membership by 100%). This generalizes our radial-classification 0/42-mismatch harness.
5. **Mesh Hausdorff** between the two results (both meshed at same deflection) — the expensive last resort, gated to disagreement triage.

### 2.2 Oracle fallibility — N-version voting

OCCT itself fails or errs on real inputs (its own bugs group proves it; FreeCAD routinely tags "upstream OCC bug"). So: never treat OCCT disagreement as automatic failure. Add a **second independent oracle**: mesh-boolean on tessellated operands (manifold/libigl mesh-arrangements/CGAL — the Thingi10k-hardened family, e.g. https://dl.acm.org/doi/10.1145/3550454.3555460). Verdict logic: ours-vs-OCCT agree → pass; disagree → mesh oracle votes; 2-vs-1 against us → confirmed fail; three-way disagreement → human triage queue (these are frequently *OCCT* bugs — publishable/reportable upstream, and exactly the KB material the boolean campaign charter wants). This is standard differential-testing practice (https://www.emergentmind.com/topics/differential-testing).

### 2.3 Metamorphic invariants (oracle-free, catches tolerance bugs)

Cheap checks that need no reference kernel — run on 100% of cases:

- **Rigid-motion invariance**: `vol(op(A,B)) == vol(op(R(A),R(B)))` within tier tolerance, for random R. Directly generalizes the rotated-chair campaign; any drift is a tolerance-model bug (P5).
- **Scale invariance**: rerun at x1e-3 and x1e3 (mm vs m units); rel-props must be stable. Flushes absolute-epsilon leaks.
- **Inclusion–exclusion**: `vol(A) + vol(B) == vol(A∪B) + vol(A∩B)`.
- **Partition**: `vol(A\B) + vol(A∩B) == vol(A)`.
- **Commutativity** of ∩, ∪; **idempotence** `A∪A = A`; **complement consistency** `A\B == A ∩ (Box\B)`.
- **Continuity probes**: sweep a pose parameter across a tangency (offset -10t..+10t in t steps); result volume must be monotone/continuous except at the contact event — spikes = sliver/classification instability (P4 EE/EF).

### 2.4 Process harness

- Every case in a **subprocess** with timeout (30 s default) + RSS cap; verdicts: PASS / FAIL(diff) / INVALID(self-check) / CRASH / HANG / OCCT_FAIL. (We already learned this the hard way: merge hangs box x tor fuse.)
- **Auto-minimization** on new failures: drop faces/sub-shapes and bisect pose parameters until the differ still fires with the smallest operands; store minimized repro as a named regression (the OCCT `bugs/` model).
- **Baseline snapshots**: nightly report is *delta vs last green baseline* (new fails, new passes, flaky set), not raw pass count — mirrors OCCT's own grid workflow (https://github.com/Open-Cascade-SAS/OCCT/wiki/tests).

---

## 3. Concrete plan: 10k-case nightly corpus with OCCT oracle differ

### 3.1 Composition (10,000 cases)

| Tier | n | Source | What it stresses | License bucket |
|---|---|---|---|---|
| T0 curated | 500 | hand-built primitives matrix (existing 60/60 + battery), chairs + rotation frontier, cone/torus analytic pairs | exactness, analytic recognizers, known frontier | ours |
| T1 OCCT mine | 1,500 | tests/boolean + bugs/modalg harvest (public-data cases only), expected-props preserved | pre-triaged adversarial pairs, each a former bug | LGPL-attributed |
| T2 ABC pairs | 3,000 | 600 deduped valid ABC solids x 5 poses: aligned, z15, z30x20, tangent-contact offset 0, offset ±10·tol | freeform SSI, rotation robustness, near-tangency (P4/P5) | Onshape-ToU |
| T3 assembly contacts | 2,000 | AutoMate mated pairs in mating pose (CC0); Fusion 360 Assembly joint pairs as internal-only extra | real coincident/coaxial contact → same-domain P3, EE/EF P4 | CC0 / (NC internal) |
| T4 DeepCAD replay | 2,000 | sampled extrude-step operand pairs with designer op (join/cut/intersect) | sketch-on-face coplanarity at volume (P3) | MIT/Onshape-ToU |
| T5 synthetic adversarial | 1,000 | generator: near-tangent cyl/cone/sphere lattices, seam-crossing rotations, unit-scale sweeps 1e-3..1e3, ±k·tol jitter ladders | tolerance model (P5), EE/EF events, seam logic | ours |

Weekly extended sweep: T2 expanded to the full deduped ABC pool (~100k+ pairs) on a schedule, nightly stays at 10k for <2 h wall time (10k x ~0.5 s avg x 2 kernels, parallel over ~16 workers).

### 3.2 Build pipeline (one-time, ~1-2 weeks of scripting)

1. **Mirror + index**: download ABC STEP chunks (start with 5 chunks = 50k models), AutoMate parts+mates parquet, DeepCAD JSON, OCCT public test dataset, MFCAD++ (held for P3-targeted grids), NIST CTC/FTC/STC.
2. **Sanitize**: per model — OCCT import, require single closed valid solid (`BRepCheck` + free-edge scan), normalize to unit bbox, compute B-Rep fingerprint (topo counts + inertia tensor) → dedupe; write parquet manifest: `case_id, source, source_url, license, sha256, n_faces, surf-type histogram, fingerprint`.
3. **Pair + pose generator**: stratified sampling to hit quotas per surface-type mix (plane-only / analytic / freeform / mixed) and per pose class (generic, rotated, tangent, coincident). Guarantee interference via bbox + stl2 mesh-boolean prefilter.
4. **Oracle precompute**: run OCCT once per case, store `vol/area/centroid/nbshapes/verdict + occt_version`; store mesh-oracle volume for the triage path. Cases where OCCT itself crashes/hangs go to an `oracle-unstable` shelf (still run our kernel: self-validity + metamorphic checks only).
5. **Runner + report**: nightly CI job; per-case subprocess; verdict rows appended to parquet; HTML dashboard: pass % by tier/surface-class/pose-class, vol rel-err percentiles, naked-edge histogram, new-fail list with one-click minimized repro download; auto-file a regression stub (T0 append) for every confirmed new fail.
6. **Storage/licensing**: `corpus/{ours,lgpl-occt,cc0-automate,onshape-tou,nc-internal}/...`; only `ours` + `cc0` + `lgpl` buckets are ever redistributable; manifest carries the license column so any future public benchmark cut is a filter, not a re-audit.

### 3.3 Phase mapping

- **P3 same-domain**: T3 contact pairs + T4 DeepCAD coplanar replays + MFCAD++ stock/feature coplanar grid are the acceptance corpora; target metric: same-domain-tagged subset pass % tracked separately.
- **P4 EE/EF interference**: T5 tangency jitter ladders + T3 coaxial/tangent mates; continuity probes (2.3) are the acceptance test.
- **P5 tolerance model**: scale-sweep metamorphic battery over T0-T2; NIST/CAx-IF round-trip conformance as the I/O floor.
- **P6 corpus validation**: the nightly itself; definition of done = stable green baseline + differ-triage loop feeding T0.

---

## 4. Sources

- ABC: https://deep-geometry.github.io/abc-dataset/ , https://openaccess.thecvf.com/content_CVPR_2019/papers/Koch_ABC_A_Big_CAD_Model_Dataset_for_Geometric_Deep_Learning_CVPR_2019_paper.pdf , https://github.com/deep-geometry/abc-dataset/blob/master/LICENSE , https://archive.nyu.edu/handle/2451/44309 , https://github.com/deep-geometry/abc-dataset/issues/15
- Better STEP: https://arxiv.org/html/2506.05417v1 , https://github.com/better-step/abs
- Fusion 360 Gallery: https://github.com/AutodeskAILab/Fusion360GalleryDataset , assembly docs https://github.com/AutodeskAILab/Fusion360GalleryDataset/blob/master/docs/assembly.md , paper https://arxiv.org/pdf/2010.02392
- MFCAD: https://github.com/hducg/MFCAD ; MFCAD++/Hierarchical CADNet: https://pure.qub.ac.uk/en/datasets/mfcad-dataset-dataset-for-paper-hierarchical-cadnet-learning-from , https://dl.acm.org/doi/10.1016/j.cad.2022.103226
- DeepCAD: https://github.com/ChrisWu1997/DeepCAD , https://arxiv.org/abs/2105.09492
- Thingi10K: https://github.com/Thingi10K/Thingi10K , https://huggingface.co/datasets/Thingi10K/Thingi10K , https://arxiv.org/pdf/1605.04797
- Mesh-boolean robustness protocol: https://dl.acm.org/doi/10.1145/2897824.2925901 , https://dl.acm.org/doi/10.1145/3550454.3555460 , https://www.graphics.rwth-aachen.de/media/papers/339/ember_exact_mesh_booleans_via_efficient_and_robust_local_arrangements.pdf , https://arxiv.org/abs/2405.12949
- NIST MBE PMI: https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0 , https://pages.nist.gov/CAD-PMI-Testing/models.html
- OCCT tests + data + tracker: https://github.com/Open-Cascade-SAS/OCCT/wiki/tests , https://github.com/Open-Cascade-SAS/OCCT/wiki/boolean_operations , https://dev.opencascade.org/content/open-cascade-technology-testing-dataset-published , https://github.com/Open-Cascade-SAS/OCCT/issues , https://github.com/Open-Cascade-SAS/OCCT/discussions/36
- FreeCAD tracker: https://github.com/FreeCAD/FreeCAD/issues/5619 , https://github.com/FreeCAD/FreeCAD/issues/5782 , https://github.com/FreeCAD/FreeCAD/issues/18047 , https://tracker.freecad.org/
- AutoMate: https://arxiv.org/abs/2105.12238 , https://datadryad.org/dataset/doi:10.5061/dryad.2547d7wvw , https://degravity.github.io/automate/
- CAx-IF/MBx-IF: https://www.mbx-if.org/home/cax/resources/ , https://www.cax-if.org/cax/cax_introduction.php ; NIST SFA: https://www.nist.gov/services-resources/software/step-file-analyzer-and-viewer
- Onshape ToU (harvesting limits + public-doc license): https://www.onshape.com/en/legal/terms-of-use
- Differential testing background: https://www.emergentmind.com/topics/differential-testing
