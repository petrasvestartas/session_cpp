# Public Datasets & Benchmark Suites for B-Rep Boolean Stress Testing — Survey + Acquisition Plan

Research date **2026-07-25**. Companion to `kb/research_validation-corpora.md` (which covers the *methodology* / 10k-case nightly design). This file is the **source dossier**: what each public corpus actually contains, what it costs, what you are legally allowed to do with it, and — the question that decides everything — **whether it ships expected boolean results**.

## How to read the numbers

Two labels are used throughout and they are not interchangeable:

- **[MEASURED]** — I fetched it and counted/HEAD-requested it during this session. Reproducible.
- **[STATED]** — taken from the publisher's page/paper. Not independently verified.

Anything I could not measure or find is written as "unknown", not estimated.

---

## 0. The distinction that eliminates half the field

A boolean kernel operates on **trimmed NURBS/analytic faces with topology and tolerances**. A triangle soup cannot exercise SSI walking, same-domain face merging, EE/EF interference, seam handling, or the tolerance model. It exercises a *different algorithm* (mesh arrangements).

| Corpus | Native representation | Can it be a **B-Rep boolean operand**? |
|---|---|---|
| ABC | STEP AP203/AP214 + Parasolid | **Yes** — primary |
| Fusion 360 Gallery | SMT (ASM) + STEP | **Yes** |
| AutoMate | STEP + Parasolid | **Yes** |
| MFCAD / MFCAD++ | STEP | **Yes** |
| CC3D-Ops | B-Rep (SolidWorks-extracted) | **Yes**, but gated |
| DeepCAD | JSON construction programs | **Yes, after replay** through OCCT |
| BenchCAD | CadQuery programs + STEP | **Yes, after replay** |
| OCCT test data | `.brep` / `.stp` / `.igs` / `.rle` | **Yes** — and the only one with oracles |
| FreeCAD repo data | `.FCStd` (zip of `.brp`), `.stp`, `.brep` | **Yes**, tiny |
| CC3D (base) | triangular meshes (CAD + scan pairs) | **No** |
| Thingi10K | STL/OBJ/PLY/OFF meshes | **No** |
| ModelNet | OFF meshes | **No** |
| ShapeNet | OBJ meshes (+ voxels) | **No** |

**Say it plainly: Thingi10K, ModelNet, ShapeNet, and base CC3D are mesh corpora. They cannot test a B-Rep boolean.** Their value to us is (a) importing the *robustness-reporting protocol* the mesh-boolean literature standardised, and (b) fuelling a second, independent mesh-boolean oracle when OCCT itself is the suspect. Nothing else. Do not spend acquisition budget on them as operand sources.

---

## 1. ABC Dataset (Koch et al., CVPR 2019)

**Contains.** ~1,000,000 CAD models scraped from Onshape public documents, published as 100 chunks × 10k models, one 7z archive per format per chunk. Formats: `step` (B-Rep, ISO 10303-21), `para` (Parasolid XT), `stl2`, `obj` (with GT normals/curvature), `feat` (per-patch surface/curve type ground truth), `stat`, `meta` (YAML), `ofs` (FeatureScript), `img`.

**Size — [MEASURED]** by parsing the publisher's own `size.yml` (`https://deep-geometry.github.io/abc-dataset/data/size.yml`, 25,366 bytes, 800 entries):

| format | chunks | total compressed |
|---|---|---|
| `stl2` | 100 | 614.1 GiB |
| `obj` | 100 | 592.1 GiB |
| `feat` | 100 | 336.8 GiB |
| `para` | 100 | 183.9 GiB |
| **`step`** | **100** | **99.3 GiB** |
| `ofs` | 100 | 13.8 GiB |
| `stat` | 100 | 0.1 GiB |
| `meta` | 100 | 0.1 GiB |
| **all formats** | | **1840.3 GiB (~1.8 TiB)** |

Chunk 0000 STEP archive = 1,594,129,754 bytes [MEASURED]. So a **STEP-only mirror is ~99 GiB of download**; a single chunk (10k models) is ~1.5 GiB. Uncompressed footprint of STEP is **not measured** — 7z on ASCII STEP typically expands severalfold, budget generously and measure on chunk 0 before committing.

**Download mechanism — [MEASURED].** `https://deep-geometry.github.io/abc-dataset/data/step_v00.txt` is a 100-line file of `<url> <filename>` pairs pointing at NYU Faculty Digital Archive bitstreams (`https://archive.nyu.edu/rest/bitstreams/<id>/retrieve`). Publisher's own recipe: `cat step_v00.txt | xargs -n 2 -P 8 sh -c 'wget --no-check-certificate $0 -O step/$1'`. Chunk 0000 (`bitstreams/88598`) and chunk 0050 (`bitstreams/88830`) both returned **HTTP 200 on 2026-07-25** [MEASURED] — the archive is up right now, but historical flakiness is documented (`deep-geometry/abc-dataset` issue #15), so **mirror once, locally, and checksum against the published `md5.yml`**.

**License / redistribution.** Repo tooling MIT. NYU record for chunk 0000 shows rights statement "MIT License" [STATED]. The dataset page itself says **"The copyright of the CAD models is owned by their creators"** and points at Onshape ToU 1.g.ii. Onshape ToU [STATED, quoted]: for Free-Plan public documents created after 2018-08-07, a **"worldwide, royalty-free and non-exclusive license to any End User or third party ... to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies"**. Same ToU **forbids** "any robot, spider, scraper or other automated means to access the Service, or use any data mining, data gathering or extraction method" — so consume the published academic snapshot, **never re-scrape Onshape**.

**GROUND TRUTH FOR BOOLEANS: NONE.** ABC supplies models only. Its `feat` files give per-face/per-curve *surface type* ground truth (useful for stratifying pairs by geometry class), and `stat` gives topology counts (useful for dedup/filtering) — but there are **zero expected boolean outputs**. Every boolean verdict on ABC must come from our own oracle.

**Known dirt.** Contains duplicates and non-solid models; a published re-extraction effort ("Better STEP", FRDR `d54b95e0-...`) reports OCC meshing failures on a small percentage of ABC. Dedup + closed-solid filtering is mandatory before pairing.

---

## 2. Fusion 360 Gallery (Autodesk)

**Contains — [STATED]** from the repo README/docs:

| subset | scale | download |
|---|---|---|
| Assembly | 8,251 assemblies / 154,468 parts | size not stated |
| Assembly-Joint | 32,148 joints / 23,029 parts | **2.8 GB** (j1.0.0) |
| Reconstruction | 8,625 construction sequences | **2.0 GB** (r1.0.1) |
| Segmentation | 35,680 parts | **3.1 GB** (s2.0.1) |
| Segmentation "Extended STEP" | 42,912 STEP files | **483 MB** |

Formats: `.smt` (native Autodesk Shape Manager — least conversion loss), `.step`, `.obj`, JSON metadata, PNG thumbnails. Units cm, angles radians.

**The high-value part.** Assembly JSON records **contacts: "which B-Rep faces are coincident or within a tolerance of 0.1 mm, when a joint is in an assembled state"** [STATED, verbatim], with both face references, body references, **surface types**, bounding boxes and face indices. Joint types: rigid, revolute, slider, cylindrical, pin-slot, planar, ball. This is a curated index of *designer-placed tangency and coincidence* — exactly the P3 same-domain / P4 EE-EF regime — with the contacting faces already named.

**License — [MEASURED, quoted from `LICENSE.md`].** Non-commercial only: **"You may access, use, reproduce and modify the Dataset, in each case, only for non-commercial research purposes."** Redistribution of the whole set is barred: **"You may not redistribute or make available to others the Dataset in its entirety; however you may direct others to https://github.com/AutodeskAILab/Fusion360GalleryDataset to obtain the Dataset."** Portions/modifications may be shared only under the same non-commercial restriction. Repo license field on GitHub = `NOASSERTION` [MEASURED]. **For a kernel intended to be commercial, this is internal-diagnostics-only. It can never appear in a shipped or published test suite.** AutoMate is the license-clean substitute for the same contact-pose idea.

**GROUND TRUTH FOR BOOLEANS: NONE.** No expected cut/common/fuse results anywhere. What it supplies is *where the hard contact is*, not *what the answer is*.

---

## 3. AutoMate (Onshape assemblies + mates)

**Contains — [STATED]** from the Dryad record: 8 files, **34.93 GB** total — `parasolid.zip` 20.49 GB, `step.zip` 13.21 GB, `parts.parquet` 211.75 MB, `mates.parquet` 91.51 MB, `assemblies.zip` 911.44 MB, `assemblies.parquet` 13.57 MB, `config_encodings.json`, README. Parts as **both STEP and Parasolid**; assembly/mate metadata as JSON + Apache parquet. Paper scale: ~452k unique parts, ~255k assemblies, ~1.29M mates.

**License.** Hosted on Dryad, whose site-wide policy applies CC0 to data submissions; **the dataset page displays no explicit per-dataset license badge** [MEASURED — I looked]. Treat CC0 as likely but **verify in writing before redistributing anything derived from it.** Underlying models are Onshape public documents (same ToU 1.g.ii commercial-use grant as ABC).

**GROUND TRUTH FOR BOOLEANS: NONE.** Mates give *poses*, not results. But mates are the single richest public source of *guaranteed-contact relative placement* between two real B-Rep solids — coaxial pins in holes, coplanar flange faces, tangent seats — which is precisely what a random-pose pair generator cannot manufacture at scale.

---

## 4. DeepCAD

**Contains.** 178,238 sketch-and-extrude **construction sequences** as JSON (derived from ABC/Onshape), plus vectorised form for ML.

**Size — [MEASURED].** `http://www.cs.columbia.edu/cg/deepcad/data.tar` = **208,074,561 bytes (208 MB)**, HTTP 200. That is the entire corpus — it is *programs*, not geometry, hence the tiny footprint.

**License — [MEASURED].** Repo `rundiwu/DeepCAD` (formerly `ChrisWu1997/DeepCAD`) carries **MIT**. Underlying geometry inherits Onshape public-document terms.

**Format caveat.** **No STEP is shipped.** Each JSON must be replayed through pythonOCC/OCCT to become a B-Rep. That replay is the feature, not the bug (below).

**GROUND TRUTH FOR BOOLEANS: NONE — but the closest thing to boolean *intent* in any public corpus.** Every extrude in a sequence carries an operation flag (new-body / join / cut / intersect). Replaying a sequence yields, at each step, an ordered operand pair `(accumulated body, new extrusion)` **plus the operation a production kernel actually performed and a human designer accepted**. That is not an expected *result*, but it is a very strong prior that the case is well-posed, and the profiles are drawn *on existing faces*, which biases the corpus hard toward exact coplanarity — the same-domain regime. ~178k sequences × several extrudes each is a large supply of natural pairs.

---

## 5. BenchCAD (2026) — newer, worth knowing

**Contains — [STATED].** 17,900 execution-verified parametric **CadQuery** programs across 106 part families, shipped with **STEP files**, multi-view renders, parameter JSON and operation lists. Explicitly covers boolean composition (`cut`, `union`, `cutThruAll`) with per-operation labels.

**License / hosting — [MEASURED].** HuggingFace `BenchCAD/BenchCAD`, `gated: false`, **`license: cc-by-4.0`**, 23 files in the repo, 1,505 downloads. Eval code MIT on GitHub.

**GROUND TRUTH FOR BOOLEANS: NONE in the boolean-result sense.** It is an LLM benchmark; "ground truth" means the reference program and its rendered geometry, scored by IoU/Chamfer. For us its value is identical in kind to DeepCAD (replayable boolean programs) but with a **cleanly permissive CC-BY licence** and pre-exported STEP. Small, cheap, license-clean.

---

## 6. CC3D and CC3D-Ops (University of Luxembourg, CVI²)

**CC3D — [STATED].** 50k+ **pairs of 3D scans and CAD models, both provided as triangular meshes**. It is a scan-to-CAD reconstruction dataset. **It is not a B-Rep corpus** despite the "CAD" in the name.

**CC3D-Ops — [STATED].** 37k+ **B-Reps** with per-face **CAD operation type** and **operation step** annotations, extracted via the SolidWorks API (CADOps-Net, 3DV 2022). This *is* B-Rep, and the per-face op-type/op-step labels are the richest public "which faces came from which modelling operation" signal that exists.

**Acquisition friction — [STATED, and it is the deciding factor].** Both require a **formal licence agreement signed by the recipient and by the research-administration office director of the recipient's institution**, requested by email (`Shapify3D (at) uni (dot) lu`). No anonymous download. Size, formats, commercial-use and redistribution terms are **not published** — they are settled during licence negotiation. Attribution to the CADOps-Net paper required.

**GROUND TRUTH FOR BOOLEANS: NONE.** Op-type/op-step labels describe how the *single* model was built, not the result of an A-op-B.

**Verdict:** high intrinsic value, blocked by institutional paperwork and unpublished commercial terms. For a commercially-intended kernel this is a **do-not-pursue-first** item.

---

## 7. MFCAD and MFCAD++

**MFCAD — [MEASURED].** GitHub `hducg/MFCAD`, repo size **189,930 KB (~190 MB)**, **MIT licence**. ~15.5k synthetic STEP models with per-face machining-feature labels, 16 planar feature classes.

**MFCAD++ — [STATED].** 59,655 (page says "59665 samples") STEP models, **24 feature classes including non-planar**, 3–10 intersecting features per model, labels embedded in `ADVANCED_FACE` names. Hosted at QUB Pure: `MFCAD_dataset.zip`, **1.5 GB**, **licence CC BY**, DOI `10.17034/d1fec5a0-8c10-4630-b02e-b92dc81df823`, direct URL `https://pure.qub.ac.uk/files/278385243/MFCAD_dataset.zip`. My HEAD request to that URL returned **HTTP 403** [MEASURED] — the host blocks non-browser agents, so the 1.5 GB figure is page-stated, not measured; plan on a browser-style download (correct User-Agent / manual fetch). Generation code at GitLab `qub_femg/machine-learning/mfcad2-dataset`, MIT, no tagged releases [MEASURED].

**GROUND TRUTH FOR BOOLEANS: NONE directly — but a uniquely reconstructible near-oracle.** Every MFCAD model *is* `stock − Σ features`, and the per-face labels tell you which faces belong to which feature. Because slots/pockets/holes/chamfers are parametric primitives, you can **rebuild the tool solids and the stock block from the labels**, then assert `stock − tool` reproduces the shipped model (compare mass properties + face-label partition). That is a genuine self-consistency oracle derived from the dataset's own annotations — no reference kernel needed. It is the only corpus in this survey where that trick is available, and it is *free ground truth* for the exactly-coplanar / exactly-coaxial regime, which is our P3 pain point.

CC-BY on MFCAD++ and MIT on MFCAD mean derived test cases are **redistributable with attribution**. That is rare and valuable.

---

## 8. OCCT test suite + OCCT public test data — the only real boolean oracle in public

This is the one source that ships **expected numeric results**. I measured it end to end.

### 8.1 The test scripts

Sparse-checkout of `Open-Cascade-SAS/OCCT@master:tests/` = **75 MB on disk** [MEASURED].

- `tests/boolean/` = 34 grids, **4,093 case scripts** [MEASURED].
- `tests/bugs/` = 4,244 case scripts, of which `modalg_1..8` = **2,558** [MEASURED: modalg_1 220, _2 498, _3 26, _4 169, _5 533, _6 466, _7 576, _8 70].
- modalg cases that invoke a boolean command (`bcut|bfuse|bcommon|bopcut|bopfuse|bopcommon|bopsection|bsection|bapibuild|bbuild|bfillds|bopargcheck`): **1,107**; of those with a `checkprops` assertion: **557** [MEASURED].

Per-grid boolean case counts [MEASURED, top of list]: `bopcut_simple` 379, `bopcommon_simple` 378, `bopfuse_simple` 375, `boptuc_simple` 373, `gdml_private` 326, `bcut_2d` 170, `bfuse_complex` 165, `bsection` 164, `bcut_complex` 153, `bcommon_2d` 143, `bopcommon_2d` 143, … down to `bcutblend` 1.

### 8.2 The oracle is inside the script

Ground-truth assertion coverage in `tests/boolean/` [MEASURED]:

- **3,754 / 4,093 cases contain `checkprops`** (numeric mass-property assertion).
- 822 contain `checknbshapes` (topology counts).
- 102 contain `checkshape` (validity).
- `checkprops` flag histogram across the grid: **`-s` 2,936** (surface area), **`-l` 917** (total length — used for section curves), **`-v` 154** (volume), `-deps` 66 (relative epsilon), `-equal` 10, `-skip` 3, `-eps` 1.

A whole self-contained case, verbatim from `tests/boolean/bcut_simple/A1` [MEASURED]:

```
psphere s 1
box b 1 1 1
bcut result s b
checkprops result -s 13.3518
checkview -display result -2d -s -otherwise { s b } -path ${imagedir}/${test_image}.png
```

and a data-driven one, `tests/boolean/bcut_complex/A1`:

```
restore [locate_data_file CIN900_cts20hlh.rle] a
restore [locate_data_file CIN900_cts20hli.rle] b
restore [locate_data_file CIN900_cts20hlj.rle] c
bcut rab a b
bcut result rab c
checkprops result -s 272.503
```

**That `checkprops result -s 272.503` is the ground truth.** It is a value blessed by OCCT's maintainers, regression-locked for years, per case. Nothing else in this survey has that.

### 8.3 Reproducibility without OCCT-private data

Cases reference external shapes via `locate_data_file`. I cross-checked every reference against the public dataset [MEASURED]:

| `tests/boolean` | count |
|---|---|
| total case scripts | 4,093 |
| **self-contained** (built from Draw primitives, zero external data) | **2,015** — of which 1,980 carry `checkprops` |
| **runnable with the public dataset** | **1,291** — all 1,291 carry `checkprops` |
| blocked (references OCCT-private data) | 787 |
| **reproducible from public downloads** | **3,306 = 80.8%** |
| distinct data files referenced | 1,304 (788 present publicly, 516 missing) |
| disk cost of those 788 operand files | **11.4 MB** |

Same analysis for `tests/bugs`: 966 self-contained, 1,035 public-runnable, 2,243 blocked; 3,313 distinct refs, 1,186 present, **114.9 MB** of operand data.

**The 2,015 self-contained boolean cases are the single best thing in this document: 1,980 pre-blessed numeric oracles, zero download, zero licence friction, replayable from a text script.**

### 8.4 The dataset itself — and a correction

**There is no public GitHub repository named `occt-test-data`.** I enumerated the entire `Open-Cascade-SAS` org (24 repos: `OCCT`, `OCCT-Archive`, `OCCT-Light`, `OCCT-Components`, `opennurbs`, `IfcOpenShell`, `Inspector`, `ExpToCas`, `pmuc`, samples, archives …) and `Open-Cascade-SAS/OCCT-test-data` returns **404** [MEASURED]. The OCCT dev guide's phrase "a data files repository" refers to internal infrastructure. What is *actually* public is a versioned tarball:

| artefact | size | notes |
|---|---|---|
| `https://dev.opencascade.org/sites/default/files/free/shapes_7.5.0.tgz` | **65,580,939 B (65.6 MB)** [MEASURED] | the original "2500+ shapes" release; extracts to **207 MB / 3,223 files** [MEASURED] |
| `https://github.com/Open-Cascade-SAS/OCCT/releases/download/V7_9_0_beta1/opencascade-dataset-7.9.0.tar.xz` | **98,739,184 B (98.7 MB)** [MEASURED], 10,184 downloads | extracts to **347 MB / 3,388 files** [MEASURED] |
| same, `.zip` | 124,064,355 B (124.1 MB) [MEASURED] | 6,333 downloads |

`CSF_TestDataPath` must point at the extracted tree. Contents of the 7.5.0 tree [MEASURED]: `brep/` 2,562 files / 93 MB, `step/` 320 files / 88 MB, `geom/` 173, `iges/` 57, `others/` 106, `msv/` 5. By extension: 1,378 `.brep`, 875 `.rle` (old OCC binary shape format — `restore` reads it), 312 `.stp`, 199 `.draw`, 57 `.igs`, 29 `.stl`.

Notably, **the 7.9.0 dataset does not improve boolean coverage over 7.5.0**: both yield exactly 788/1,304 referenced boolean data files and the same 3,306 reproducible cases [MEASURED]. Take 7.9.0 (it is a superset overall) but expect no boolean-specific gain.

**Licence.** The OCCT distribution page states LGPL-2.1 over the release. **No LICENSE/COPYING file exists at the root of the extracted dataset tree** [MEASURED — I looked]. So: fine for an internal corpus, keep it in a clearly-attributed LGPL bucket, and get written clarification before redistributing shapes in any public benchmark cut.

**GROUND TRUTH FOR BOOLEANS: YES — the only "yes" in this survey.** Per-case `checkprops` (area/length/volume, some with explicit `-deps` relative epsilon) plus `checknbshapes` topology counts.

---

## 9. FreeCAD's own test suite

**What is actually there — [MEASURED]** against `FreeCAD/FreeCAD@main` (13,809 blobs in tree):

- `src/Mod/Part/TestPartApp.py`: 1,292 lines, **74 test functions**, of which exactly **4 are boolean-named** (`testMakeCut`, `testMakeFuse`, `testMakeCommon`, `testCutHoles`).
- `src/Mod/Part/parttests/`: `TopoShapeTest.py` 979 lines, `regression_tests.py` 271 lines, plus face-maker/mirror/colour tests.
- Mass-property assertions across those three files: **2 lines matching `.Volume|.Area` in `TestPartApp.py`, 1 in `TopoShapeTest.py`, 0 in `regression_tests.py`** [MEASURED].
- Geometry data shipped in-repo: **86 files, 16.6 MB** (54 `.FCStd`, 28 `.brep`, 4 `.stp`) [MEASURED]; the interesting cluster is `data/tests/ModelRefineTests/` — **24 `.brep` files, 3.68 MB**, heavy on `cylinderSeam*` names, i.e. seam/coplanar-merge torture shapes.

**Verdict, bluntly:** FreeCAD's *test suite* is a weak boolean oracle — a handful of smoke tests with almost no numeric assertions. **The value in the FreeCAD project is the issue tracker, not the test suite**: `.FCStd` attachments are zip containers holding raw OCCT `.brp` blobs, so a failing-boolean bug report yields extractable operands plus a human-confirmed verdict without running FreeCAD. Licence: LGPL project; issue attachments are user-contributed — internal use, keep issue-URL provenance.

**GROUND TRUTH FOR BOOLEANS: essentially NONE** (3 mass-property assertion lines total across the Part test modules).

---

## 10. CadQuery / OCP test suite

**What is there — [MEASURED]** against `CadQuery/cadquery@master:tests/`:

- 24 files. `test_cadquery.py` = 194.5 KB / **5,931 lines / 202 test functions**; `test_free_functions.py` 35.1 KB / 1,591 lines; `test_shapes.py`, `test_sketch.py`, `test_selectors.py`, `test_importers.py`, `testdata/`.
- Test functions whose names touch boolean/split semantics: **22** — `testCut`, `testCutBlind`, `testCutBlindUntilFace`, `testCutEach`, `test_cutFromBase`, `testCutThroughAll`, `testIntersect`, `testUnions`, `testUnionCompound`, `testUnionNoArgs`, `test_bool_operators`, **`testFuzzyBoolOp`**, `testSplitShape`, `testSplitKeepingBoth/Bottom/Half`, `testSplitError`, `testRevolveCut`, `testTaperedExtrudeCutBlind`, `testTwistExtrudeCombineCut`, `testFaceIntersectedByLine`, `testCutToFaceOffsetNOTIMPLEMENTEDYET`.
- 38 boolean-operation call sites in `test_cadquery.py`.

**Licence.** CadQuery Apache-2.0; OCP wraps OCCT (LGPL-2.1-with-exception).

**GROUND TRUTH FOR BOOLEANS: thin but real, and *derivative*.** The assertions are hand-written expected volumes/face-counts on small procedurally-built solids — genuinely blessed numbers, but the "blessing" ultimately comes from OCCT, since CadQuery *is* OCCT. Using them as an oracle for our kernel is therefore **not independent** of the OCCT oracle; it is the same oracle in a different wrapper. Value is as a **API-shape/behaviour reference and a source of ~20 curated small cases**, especially `testFuzzyBoolOp` (fuzzy-tolerance semantics) which maps directly onto our P5 tolerance work.

---

## 11. Mesh corpora — Thingi10K, ModelNet, ShapeNet, CC3D-base

Grouped because their verdict is identical: **not B-Rep, cannot serve as boolean operands.**

**Thingi10K** — 10,000 Thingiverse 3D-printing models, 2009–2015. Formats: 9,956 STL, 42 OBJ, 1 PLY, 1 OFF [STATED]. Deliberately dirty (thousands self-intersect; non-manifold cases abundant). Per-model licences vary (3,680 CC-BY-SA) — there is **no single dataset licence**; the HuggingFace mirror `Thingi10K/Thingi10K` has **20,013 files and no `license` field in its card metadata** [MEASURED]. `pip install thingi10k` gives programmatic access with filtering.

**ModelNet (Princeton)** — OFF meshes. ModelNet40: 12,311 shapes / 40 categories; ModelNet10: 4,899 shapes. **`ModelNet40.zip` = 2,039,180,837 B (2.04 GB), HTTP 200** [MEASURED]; the analogous `ModelNet10.zip` at the same path returned **404** [MEASURED] — get it from the site's own link. Terms [STATED]: models downloaded from the internet, original authors hold copyright, "provided for the convenience of academic research only".

**ShapeNet** — OBJ meshes + voxelisations. ShapeNetCore v2 ~51,300 models / 55 categories, ~30.3 GB [STATED]. HuggingFace `ShapeNet/ShapeNetCore` is **`gated: manual`, license `other`** [MEASURED]. Terms: **non-commercial research and educational purposes only**, with indemnification clauses. Disqualifying for a commercial kernel programme even as a mesh oracle input.

**CC3D base** — see §6: triangular meshes, gated.

**GROUND TRUTH FOR BOOLEANS: NONE, in any of them.**

**What we actually take from this family:** the *reporting protocol*. The mesh-boolean literature (Zhou/Grinspun/Zorin mesh arrangements SIGGRAPH 2016; Cherchi et al. floating-point mesh arrangements SIGGRAPH Asia 2020; Cherchi/Pellacini/Attene/Livesu interactive & robust mesh booleans SIGGRAPH Asia 2022; EMBER SIGGRAPH 2022) standardised: run the whole corpus, report pass / fail / crash / timeout / invalid-output percentages, and self-union / rotated-copy stress. Cherchi et al. 2022 [STATED] extracted **7,628 clean meshes from Thingi10K, split into two halves of 3,814, randomly paired**, and ran booleans on the pairs — that is the canonical "how to make pairs from a single-model corpus" precedent, and it is exactly what we must do with ABC.

---

## 12. Academic boolean-robustness benchmark suites — the honest answer

I searched for a public benchmark that ships **inputs + expected boolean outputs**. **It does not exist.** Every candidate is inputs-plus-methodology:

- **Mesh-arrangement / exact-boolean papers** (Zhou 2016, Cherchi 2020/2022, EMBER 2022, OpenMeshCraft) publish code and cite Thingi10K, but do **not** publish an expected-output corpus. Their correctness argument is: *all exact methods must produce identical output topology, so cross-check connected-component counts and Euler characteristic across implementations* [STATED]. In other words, the literature's own ground truth is **N-version agreement plus topological invariants** — which is precisely the oracle design we independently arrived at.
- **`gcherchi/InteractiveAndRobustMeshBooleans`** — code only; **the repo publishes no benchmark dataset, no input list, and no validation criteria** [MEASURED — I read it]. Inputs must be "manifold, watertight, self-intersection free, well-oriented".
- **`BrunoLevy/geogram.data/Intersections`** — **14 `.obj` files** [MEASURED]: `three_cylinders.obj` (coplanar triangles), `cube_with_8_spheres.obj`, `rot_seven_cubes.obj`, `cucubes.obj`, `two_cylinders.obj`, `classif_1/2.obj`, … A tiny, hand-picked *degeneracy* set — coplanarity, rotated-copy unions, classification torture. Repo carries **no licence file** [MEASURED]. Small but conceptually well-aimed; worth reading for case *ideas* even though it is mesh-domain.
- **`mangoleaves/OpenMeshCraft`** — GPL-3.0 [MEASURED]; exact-arithmetic mesh library with benchmarking harness. GPL makes it unusable inside our product; usable as an external cross-oracle process.
- **MeshLib's public "3D Boolean Libraries Comparison & Benchmark 2025"** — compares MeshLib, MCut, Manifold, CinoLib+RobustMesh, CGAL, Rhino, libigl, trimesh-Manifold, trimesh-Blender on **three** cases (Nefertiti 2M-tri, dental ~500k-tri with coincident surfaces, simple objects with holes/self-intersections). Input and result meshes are downloadable from a vendor SharePoint link; **no repo, no documented reproducible methodology**, correctness graded as Yes/No/Fail plus a manual "result is incorrect" flag [STATED]. Vendor marketing artefact — read it for the degeneracy taxonomy, do not rely on it.
- **CGAL Polygon Mesh Processing corefinement** — GPL-licensed reference implementation with its own test tree; usable as an external cross-oracle process, not as a corpus.
- **NIST MBE PMI test cases** (CTC/FTC/STC, AP242+AP203) — ~11 core models, US-government work, effectively public domain. Not a pair source; it is the **STEP import/export conformance floor** and a supply of production-grade fillets/drafts to cut against primitive tools.

**Conclusion to state plainly in any plan review: there is no public B-Rep boolean benchmark with expected results other than the OCCT test scripts. Anyone claiming otherwise has not checked.**

---

## 13. Ground-truth summary table

| Source | B-Rep? | Ships expected boolean results? | What it does give |
|---|---|---|---|
| **OCCT `tests/boolean` + `tests/bugs`** | yes | **YES** — 3,754/4,093 cases carry `checkprops`; 822 carry `checknbshapes` | per-case blessed area/length/volume + topology counts |
| MFCAD / MFCAD++ | yes | no — **but reconstructible** self-consistency oracle from per-face feature labels | `stock − tool` must reproduce the shipped model |
| CadQuery tests | yes | partial (~20 cases), **not independent of OCCT** | API-behaviour reference, fuzzy-boolean semantics |
| DeepCAD / BenchCAD | after replay | no | designer-intended op per step (join/cut/intersect) |
| Fusion 360 Gallery Assembly | yes | no | contacting face pairs within 0.1 mm, joint types/axes |
| AutoMate | yes | no | 1.29M mates = guaranteed-contact relative poses |
| ABC | yes | no | 1M solids + per-face surface-type GT (`feat`) for stratification |
| CC3D-Ops | yes (gated) | no | per-face op type + op step |
| FreeCAD tests | yes | ~none (3 assertion lines) | tracker attachments are the real asset |
| Thingi10K / ModelNet / ShapeNet / CC3D-base | **no** | no | robustness *protocol*; mesh cross-oracle fuel |
| Academic boolean benchmarks | mixed | **no — none exist** | degeneracy taxonomy; N-version + Euler-characteristic methodology |

---

## 14. RANKED ACQUISITION PLAN (value per unit effort)

Ranking criterion: *(oracle strength × case hardness × licence cleanliness) ÷ (bytes + engineering hours)*.

### Rank 1 — OCCT `tests/boolean` self-contained cases. **Do this first, today.**
- **Cost:** one sparse checkout, **75 MB**, ~0 bandwidth concerns. No dataset download at all for the self-contained tier.
- **Yield:** **2,015 cases, 1,980 with numeric oracles**, every one a script that builds its own operands from Draw primitives (`psphere`, `box`, `pcylinder`, `ptorus`, …) — trivially portable to our API.
- **Why first:** it is the only public source of *blessed expected values*, it needs no licence negotiation, and porting the Draw primitive vocabulary is a bounded, mechanical job.
- **Effort:** ~2–3 days for a tcl→case-row transpiler covering the primitive subset + `bcut/bfuse/bcommon/bsection` + `checkprops`/`checknbshapes` parsing.

### Rank 2 — OCCT public dataset + the data-driven boolean cases.
- **Cost:** **98.7 MB** download (`opencascade-dataset-7.9.0.tar.xz`) → 347 MB on disk; only **11.4 MB** of it is actually referenced by boolean cases.
- **Yield:** **+1,291 cases, all with `checkprops`**, on real industrial shapes (`.brep`/`.rle`/`.stp`). Combined with Rank 1 → **3,306 oracle-bearing cases = 80.8% of OCCT's boolean grid**.
- **Effort:** ~1 day more (add `restore` of `.brep`/`.rle`, path resolution via `CSF_TestDataPath`).
- **Note:** accept that 787 cases are unrecoverable (OCCT-private data). Do not burn time on them.

### Rank 3 — OCCT `tests/bugs/modalg_*` boolean subset.
- **Cost:** already in the same 75 MB checkout; +114.9 MB of `tests/bugs` operand data from the same dataset tarball.
- **Yield:** **1,107 boolean-invoking cases, 557 with `checkprops`**. Every one is a former *shipped kernel bug*, pre-minimised and pre-triaged. Highest adversarial density per case in existence.
- **Effort:** ~1 day (same transpiler, messier scripts).

### Rank 4 — MFCAD++ (CC BY) + MFCAD (MIT).
- **Cost:** 1.5 GB + ~190 MB. Redistributable with attribution — **the only large STEP corpus in this survey we could legally ship in a public benchmark cut.**
- **Yield:** 59,655 + 15,488 STEP models saturated with *exactly coplanar and exactly coaxial* feature/stock interactions, plus a **reconstructible oracle** (§7) that needs no reference kernel.
- **Effort:** ~1 week to build the feature→tool-solid reconstructor. Defer until P3 same-domain work needs volume.

### Rank 5 — ABC, STEP chunks only, **partial mirror**.
- **Cost:** **1.5 GB per chunk**; 5 chunks (50k models) ≈ 8 GB, full STEP mirror 99.3 GiB. Onshape ToU 1.g.ii grants commercial use.
- **Yield:** unlimited operand volume and freeform/NURBS diversity — the only corpus that can supply *hundreds of thousands* of distinct solids.
- **Why not higher:** **zero ground truth**, plus mandatory sanitation (dedup, closed-solid filtering) before a single pair is usable. It is a *volume* play, not a *truth* play. Start at 5 chunks; expand only once the harness is proven.

### Rank 6 — AutoMate (contact poses at scale, likely CC0).
- **Cost:** `step.zip` 13.21 GB (skip `parasolid.zip` 20.49 GB) + `mates.parquet` 91.51 MB.
- **Yield:** ~1.29M mates → realistic coaxial/coplanar/tangent placements between real parts. This is the *only* scalable source of contact poses under a plausibly-CC0 licence.
- **Blocker:** confirm the licence in writing first (page shows no explicit badge).

### Rank 7 — DeepCAD (208 MB, MIT) + BenchCAD (CC-BY-4.0).
- **Cost:** trivial bytes; the cost is a **replay harness** (pythonOCC/OCCT sequence executor).
- **Yield:** ~500k+ natural operand pairs with designer-intended ops, structurally biased toward sketch-on-face coplanarity.
- **Why not higher:** replay engineering is real work and the resulting pairs still have no expected results.

### Rank 8 — NIST MBE PMI (CTC/FTC/STC).
- ~11 models, public domain. Not a pair source. Acquire anyway — it is a one-hour download and it is the **STEP round-trip conformance floor**.

### Rank 9 — FreeCAD issue-tracker mining.
- Low hundreds of curated, human-confirmed failures with extractable `.brp` operands. Slow, manual, unbounded. Do it as a background drip, not a sprint.

### Rank 10 — CadQuery test port (~20 cases, incl. `testFuzzyBoolOp`).
- Cheap, small, and **not oracle-independent**. Port for API-semantics parity, not for truth.

### Rank 11 — geogram.data/Intersections (14 OBJ degeneracy cases).
- Read for case *ideas* (coplanar triangles, rotated-copy unions). Mesh domain; no licence file. Do not ingest.

### Do NOT acquire (with reasons)
- **Fusion 360 Gallery** — non-commercial licence, cannot redistribute in entirety. Internal diagnostics only, if at all; AutoMate covers the same idea cleanly.
- **ShapeNet / ModelNet / Thingi10K / CC3D-base** — **meshes; cannot test a B-Rep boolean.** ShapeNet additionally non-commercial+gated.
- **CC3D-Ops** — institutional signed licence, undisclosed commercial terms. Revisit only if a specific research need justifies the paperwork.
- **GrabCAD / fresh Onshape scraping** — ToS forbids automated harvesting (Onshape verbatim: "Use any robot, spider, scraper or other automated means…"). Use the published snapshots.

---

## 15. Top choice in detail: sampling hard pairs from a single-model corpus

Rank 1–3 give oracle-bearing cases but a bounded number of them (~4,400). For *volume*, the top single-model corpus is **ABC** (Rank 5), and the whole game is **how you place two solids so the intersection is hard**. Random placement produces generic transversal intersections, which our kernel already handles; it wastes compute proving what we know.

### 15.1 Sanitation gate (before any pairing)
1. Import STEP with OCCT; require exactly one **closed, valid** solid (`BRepCheck_Analyzer` clean, zero free/naked edges).
2. Reject shells, wires, compounds, and multi-body files → route multi-body files to a **separate** track (they are free *designer-placed* contact pairs, see 15.3d).
3. Fingerprint = (face count, edge count, vertex count, surface-type histogram, volume, area, inertia tensor eigenvalues) → dedup.
4. Record surface-type class: `plane-only` / `analytic` (plane+cyl+cone+sph+tor) / `freeform` (any BSpline) / `mixed`.
5. Normalise: translate centroid to origin, scale so bbox diagonal = 1. **Keep the original scale factor in the manifest** — unit sweeps depend on it.

### 15.2 Pose parameterisation
For a pair (A, B), a case is `(A, B, op, T)` where `T = Translate(t) ∘ Rotate(R) ∘ Scale(s)` applied to B. Define **`d`** = A's bbox diagonal (= 1 after normalisation) and **`τ`** = the kernel's working linear tolerance (default 1e-7 in normalised units unless the case overrides it).

### 15.3 Placement families — each forces a specific degeneracy

**(a) Generic transversal — the control group, 15% of budget.**
Random `R ∈ SO(3)` (uniform quaternion), `t` sampled so the bbox overlap ratio lands in [0.25, 0.75]. Guarantee interference by bbox test then a fast mesh-boolean prefilter on the tessellations. These should all pass; they are the regression floor and the timing baseline.

**(b) Face-tangency ladder — 25% of budget. The P4 EE/EF workhorse.**
Pick a planar or cylindrical face `f_A` of A and a compatible face `f_B` of B. Compute the translation `t*` that brings them into **exact contact** (plane-to-plane coincidence; cylinder-to-cylinder external tangency; sphere-to-plane tangency). Then emit a **ladder** of cases at offsets
`δ ∈ {−10τ, −3τ, −τ, −τ/3, 0, +τ/3, +τ, +3τ, +10τ}` along the contact normal.
This single family produces: deep intersection, near-tangency-from-inside, exact tangency, near-tangency-from-outside, and clean separation — in one continuous parameter. The **acceptance criterion is not per-case correctness but monotone continuity of the result volume across the ladder**, with the only permitted discontinuity at the contact event. A spike anywhere else is a sliver/classification bug. (This is the metamorphic "continuity probe" made concrete.)

**(c) Coplanarity / same-domain forcing — 25% of budget. The P3 workhorse.**
Choose a planar face of A with normal `n` and plane offset `p`. Rotate B so one of *its* planar faces has normal `±n`, then translate so the two planes are **exactly coincident**. Sub-variants, all valuable:
- **same-domain, same orientation** (both faces outward `+n`) → face merge required.
- **same-domain, opposite orientation** → face annihilation required.
- **partial overlap**: slide B tangentially so the two coplanar face regions overlap by fraction `α ∈ {0.1, 0.5, 0.9, 1.0}` — `α = 1.0` (exact face-on-face congruence) is the nastiest.
- **coaxial cylinders**: align cylinder axes exactly, radii equal (`r_B = r_A`) and near-equal (`r_B = r_A(1 ± kτ)`, k ∈ {1,3,10}). Equal-radius coaxial cylinders are the canonical same-domain-on-a-curved-surface case.
- **jitter ladder** on top: perturb the plane offset and the normal by `kτ` to walk the case across the same-domain acceptance threshold.

**(d) Self-boolean under rigid motion — 20% of budget. The strongest oracle-free test we have.**
`A op R(A)` for `R` in a fixed rotation battery — reuse the existing frontier set `{z15, z30, z45, z63, z90, x20, y30, x13y29, z30x20}` plus random `SO(3)` — and `A op T(A)` for small translations `t = kτ·u`. Properties that must hold *without any reference kernel*:
- `A ∪ A = A` (idempotence), `A ∩ A = A`, `A \ A = ∅` — for `R = I`.
- For `R ≠ I`: `vol(A ∪ R(A)) + vol(A ∩ R(A)) = 2·vol(A)` (inclusion–exclusion).
- `vol(A \ R(A)) + vol(A ∩ R(A)) = vol(A)` (partition identity).
- **Rigid-motion equivariance**: `vol(op(A,B))` computed in world frame must equal `vol(op(Q(A),Q(B)))` for random `Q` — any drift is a tolerance-model defect, not a geometry defect.
This family is where our existing rotated-chairs campaign generalises to 100k models. It is also the family that needs **zero ground truth**, which is why it gets a fifth of the budget on a corpus that has none.

**(e) Grazing / near-degenerate contact — 10% of budget.**
- **Edge-on-face graze**: translate B so one of its edges lies within `kτ` of a face of A, `k ∈ {0, 1, 3, 10}`.
- **Vertex-on-face / vertex-on-edge**: same with a vertex.
- **Seam-crossing rotation**: rotate a cylindrical/spherical/toroidal B so the intersection curve crosses B's periodic seam. Sample the rotation angle densely around the seam-crossing value.
- **Sliver forcing**: scale B by `1 ± 1e-6` about a shared face so the boolean produces faces of area ~`τ²`.

**(f) Unit/scale sweep — 5% of budget, applied as a multiplier on families (b)(c)(d).**
Re-emit the same case with global scale `s ∈ {1e-3, 1, 1e3}`. All relative mass-property errors must be scale-invariant. Any absolute-epsilon leak in the kernel shows up here and nowhere else.

**(g) Multi-body STEP files** (from the sanitation reject pile): bodies already sit in **designer-chosen relative poses**, which is free family-(c) data with real engineering semantics. Boolean each body pair in situ.

### 15.4 Budget shaping
Stratify so each `(surface-class × placement-family × op)` cell gets a quota, rather than sampling uniformly — otherwise `plane-only × generic` swamps everything. Ops: `cut(A,B)`, `cut(B,A)`, `common`, `fuse`, `section`. `cut` in both directions is not redundant: asymmetric failures are common.

### 15.5 Verdict logic on a corpus with no ground truth
See §16 — but note the structural point: families (b), (d) and (f) are **self-validating**. They generate their own expectations from invariants. Prefer them when the oracle is untrustworthy, which on ABC-scale freeform inputs it frequently is.

---

## 16. Oracle doctrine for corpora without ground truth — stated explicitly

**For every corpus in this survey except the OCCT test scripts, the dataset does not tell us what the answer is. Our oracle is OCCT plus metamorphic invariants. The dataset is not the oracle.** Writing this down because it is the most common way a corpus programme goes wrong: teams import a million models, diff against a reference kernel, and quietly redefine "reference kernel disagreement" as "our bug".

**Layer 1 — self-validity (no oracle needed, runs on 100% of cases).**
Result is a closed, oriented, valid solid; zero naked/free edges; consistent face orientation; tolerances within bounds; no null/degenerate faces. Input pre-check (`bopargcheck`-equivalent) decides *eligibility*: invalid inputs go to a `garbage-in` shelf and never touch the pass/fail scoreboard.

**Layer 2 — metamorphic invariants (no oracle needed, runs on 100% of cases).** These are the load-bearing checks on ground-truth-free corpora:
- **Partition identity**: `vol(A \ B) + vol(A ∩ B) = vol(A)`.
- **Inclusion–exclusion**: `vol(A) + vol(B) = vol(A ∪ B) + vol(A ∩ B)`.
- **Rigid-motion equivariance**: `vol(op(A,B)) = vol(op(Q(A),Q(B)))` for random `Q ∈ SE(3)`; likewise area and (transformed) centroid.
- **A-op-A idempotence**: `A ∪ A = A`, `A ∩ A = A`, `A \ A = ∅`.
- **Commutativity** of `∪` and `∩`; **complement consistency** `A \ B ≡ A ∩ (Box \ B)`.
- **Scale invariance** across `1e-3 / 1 / 1e3`.
- **Continuity across a tangency ladder** (§15.3b): result volume monotone except at the contact event.

**Layer 3 — OCCT differ (reference, *fallible*).** Pin one OCCT version and record it in every result row. Compare mass properties (relative tiers: 1e-6 exact/analytic, 1e-4 freeform), then topology counts as a *banded* comparison (face counts are convention-dependent — merged coplanar faces are legitimate), then point-membership classification on ~1k stratified samples near the section curves (this catches orientation flips that volume comparison misses).

**Layer 4 — N-version vote on disagreement.** OCCT has boolean bugs; FreeCAD routinely tags failures "upstream OCC bug", and OCCT's own `modalg_*` groups are 2,558 monuments to that fact. So a disagreement with OCCT is **not** automatically our failure. Escalate to an independent mesh-boolean oracle (CGAL / mesh-arrangement family, run as an external process to avoid GPL contamination) on tessellated operands. Two-against-one loses; three-way disagreement goes to a human triage queue — and those are frequently *OCCT* defects worth reporting upstream.

**The ordering matters.** Layers 1–2 are cheap, independent, and correct-by-construction; they are what makes a ground-truth-free corpus scientifically usable at all. Layer 3 is convenient but is a *peer*, not an authority.

---

## 17. Manifest schema

One row per **case** (an operand pair + pose + op). Store as parquet; the `license_bucket` column is what makes any future public benchmark cut a `WHERE` clause instead of a re-audit.

```
# ---- identity -------------------------------------------------------------
case_id                 string   # stable uuid5(namespace, source|a_id|b_id|op|pose_hash)
created_utc             string   # ISO-8601
generator_version       string   # semver of the pair/pose generator
# ---- provenance: operand A ------------------------------------------------
a_source                string   # occt_tests | occt_data | abc | mfcadpp | automate | deepcad | benchcad | nist | freecad_issue | ours
a_source_id             string   # e.g. "00000050_80d90bfd..._step_000" | "tests/boolean/bcut_simple/A1"
a_source_url            string   # canonical, resolvable
a_sha256                string
a_path                  string   # repo-relative, under corpus/<license_bucket>/
a_format                string   # step | brep | rle | smt | replayed
# ---- provenance: operand B ------------------------------------------------
b_source                string
b_source_id             string
b_source_url            string
b_sha256                string
b_path                  string
b_format                string
# ---- licensing (drives redistribution) ------------------------------------
license_bucket          string   # ours | public_domain | cc0 | ccby | mit | lgpl_occt | onshape_tou | noncommercial
license_note            string   # e.g. "CC BY 4.0, attribute MFCAD++ DOI 10.17034/d1fec5a0-..."
redistributable         bool     # precomputed from license_bucket
# ---- geometry fingerprints (dedup + stratification) -----------------------
a_nfaces,a_nedges,a_nverts        int
b_nfaces,b_nedges,b_nverts        int
a_surf_hist             map<string,int>   # {plane:12, cylinder:4, bspline:2, ...}
b_surf_hist             map<string,int>
a_volume,a_area         double   # in normalised units
b_volume,b_area         double
a_inertia_eig           list<double>[3]
b_inertia_eig           list<double>[3]
a_fingerprint           string   # hash(topo counts | inertia eig | vol | area)
b_fingerprint           string
surface_class           string   # plane_only | analytic | freeform | mixed
# ---- the case ------------------------------------------------------------
op                      string   # cut_ab | cut_ba | common | fuse | section | splitter
placement_family        string   # generic | tangency | coplanar | self_rigid | grazing | multibody | seam_cross
placement_detail        string   # e.g. "planeplane_coincident_overlap0.5" | "coaxial_cyl_dr=+3tau"
pose_R                  list<double>[9]   # row-major rotation applied to B
pose_t                  list<double>[3]
pose_scale              double
ladder_index            int      # position on a tangency/jitter ladder, else null
ladder_delta_over_tau   double   # signed offset in units of tau, else null
global_scale            double   # 1e-3 | 1 | 1e3  (unit-sweep multiplier)
tolerance_linear        double   # tau used for this case
tolerance_angular       double
seed                    int64    # RNG seed that produced this pose (full reproducibility)
# ---- oracle (null where the corpus supplies none — most rows) ------------
gt_source               string   # occt_checkprops | mfcad_reconstruction | none
gt_area                 double   # from `checkprops -s`, else null
gt_length               double   # from `checkprops -l`, else null
gt_volume               double   # from `checkprops -v`, else null
gt_rel_eps              double   # from `checkprops -deps`, else default tier
gt_nbshapes             map<string,int>   # from `checknbshapes`, else null
gt_script_path          string   # original .tcl for traceability, else null
# ---- reference-kernel precompute (a peer, not an authority) --------------
occt_version            string   # e.g. "7.9.0"
occt_verdict            string   # ok | invalid | crash | hang | refused
occt_volume,occt_area   double
occt_centroid           list<double>[3]
occt_nbshapes           map<string,int>
mesh_oracle_volume      double   # second-opinion oracle, null unless triage needed
oracle_trust            string   # trusted | unstable   (unstable => layers 1-2 only)
# ---- invariants applicable to this row ------------------------------------
invariants              list<string>  # ["partition","incl_excl","rigid_equivariance","idempotence","scale_inv","continuity"]
# ---- run result (appended per nightly) ------------------------------------
run_id                  string
kernel_version          string
verdict                 string   # PASS | FAIL_DIFF | FAIL_INVARIANT | INVALID | CRASH | HANG | ORACLE_FAIL
vol_rel_err             double
area_rel_err            double
naked_edge_count        int
wall_ms                 int
peak_rss_mb             int
repro_path              string   # minimised repro if this row ever failed
```

**Layout on disk**, so licence never has to be re-derived:

```
corpus/
  ours/                 # hand-built primitives, chairs, generated adversarial
  public_domain/        # NIST CTC/FTC/STC
  lgpl_occt/            # OCCT test scripts + public dataset shapes
  ccby/                 # MFCAD++ (attribution required)
  mit/                  # MFCAD, DeepCAD-derived
  cc0/                  # AutoMate (pending written confirmation)
  onshape_tou/          # ABC-derived
  noncommercial/        # Fusion 360 Gallery — NEVER ships
  manifest/*.parquet
```

---

## 18. Sources

**ABC** — https://deep-geometry.github.io/abc-dataset/ · size manifest https://deep-geometry.github.io/abc-dataset/data/size.yml · chunk URL list https://deep-geometry.github.io/abc-dataset/data/step_v00.txt · NYU chunk 0000 https://archive.nyu.edu/handle/2451/44309 · paper https://openaccess.thecvf.com/content_CVPR_2019/papers/Koch_ABC_A_Big_CAD_Model_Dataset_for_Geometric_Deep_Learning_CVPR_2019_paper.pdf · repo/licence https://github.com/deep-geometry/abc-dataset/blob/master/LICENSE · availability issue https://github.com/deep-geometry/abc-dataset/issues/15
**Better STEP (ABC re-extraction)** — https://openreview.net/forum?id=RlKUK83N1L · https://www.frdr-dfdr.ca/repo/dataset/d54b95e0-bc14-4236-b50b-922e5bf4ba7d · https://github.com/better-step/abs
**Fusion 360 Gallery** — https://github.com/AutodeskAILab/Fusion360GalleryDataset · licence https://github.com/AutodeskAILab/Fusion360GalleryDataset/blob/master/LICENSE.md · assembly/contacts https://github.com/AutodeskAILab/Fusion360GalleryDataset/blob/master/docs/assembly.md · segmentation https://github.com/AutodeskAILab/Fusion360GalleryDataset/blob/master/docs/segmentation.md
**AutoMate** — https://arxiv.org/abs/2105.12238 · https://datadryad.org/dataset/doi:10.5061/dryad.2547d7wvw · https://degravity.github.io/automate/
**DeepCAD** — https://github.com/rundiwu/DeepCAD · https://arxiv.org/abs/2105.09492 · data http://www.cs.columbia.edu/cg/deepcad/data.tar
**BenchCAD** — https://arxiv.org/html/2605.10865v1 · https://huggingface.co/datasets/BenchCAD/BenchCAD
**CC3D / CC3D-Ops** — https://cvi2.uni.lu/cc3d-dataset/ · https://cvi2.uni.lu/cc3d-ops/ · CADOps-Net https://arxiv.org/abs/2208.10555
**MFCAD / MFCAD++** — https://github.com/hducg/MFCAD · https://pure.qub.ac.uk/en/datasets/mfcad-dataset-dataset-for-paper-hierarchical-cadnet-learning-from · https://gitlab.com/qub_femg/machine-learning/mfcad2-dataset · Hierarchical CADNet https://dl.acm.org/doi/10.1016/j.cad.2022.103226
**OCCT tests + data** — https://github.com/Open-Cascade-SAS/OCCT/wiki/tests · dataset announcement https://dev.opencascade.org/content/open-cascade-technology-testing-dataset-published · 7.5.0 tarball https://dev.opencascade.org/sites/default/files/free/shapes_7.5.0.tgz · 7.9.0 dataset https://github.com/Open-Cascade-SAS/OCCT/releases/download/V7_9_0_beta1/opencascade-dataset-7.9.0.tar.xz · downloads page https://dev.opencascade.org/release · testing-system doc https://dev.opencascade.org/doc/overview/html/occt_contribution__tests.html
**FreeCAD** — https://github.com/FreeCAD/FreeCAD (tests under `src/Mod/Part/`, data under `data/tests/`) · tracker https://github.com/FreeCAD/FreeCAD/issues
**CadQuery / OCP** — https://github.com/CadQuery/cadquery (`tests/`) · https://github.com/CadQuery/OCP
**Thingi10K** — https://github.com/Thingi10K/Thingi10K · https://huggingface.co/datasets/Thingi10K/Thingi10K · https://arxiv.org/abs/1605.04797
**ModelNet** — https://modelnet.cs.princeton.edu/ (`ModelNet40.zip`)
**ShapeNet** — https://shapenet.org/ · terms https://shapenet.org/terms · https://huggingface.co/datasets/ShapeNet/ShapeNetCore
**Mesh-boolean robustness literature** — Mesh Arrangements (Zhou/Grinspun/Zorin, SIGGRAPH 2016) https://dl.acm.org/doi/10.1145/2897824.2925901 · Fast & Robust Mesh Arrangements (Cherchi et al., SIGGRAPH Asia 2020) https://github.com/gcherchi/FastAndRobustMeshArrangements · Interactive & Robust Mesh Booleans (SIGGRAPH Asia 2022) https://arxiv.org/pdf/2205.14151 , https://github.com/gcherchi/InteractiveAndRobustMeshBooleans · EMBER (SIGGRAPH 2022) https://www.graphics.rwth-aachen.de/media/papers/339/ember_exact_mesh_booleans_via_efficient_and_robust_local_arrangements.pdf · OpenMeshCraft https://github.com/mangoleaves/OpenMeshCraft · geogram BooleanOps https://github.com/BrunoLevy/geogram/wiki/BooleanOps , data https://github.com/BrunoLevy/geogram.data/tree/main/Intersections · CGAL corefinement https://doc.cgal.org/latest/Polygon_mesh_processing/group__PMP__corefinement__grp.html · MeshLib comparison https://meshlib.io/blog/comparing-3d-boolean-libraries/
**NIST MBE PMI** — https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0 · https://pages.nist.gov/CAD-PMI-Testing/models.html
**Onshape ToU** (public-document licence grant + anti-scraping clause) — https://www.onshape.com/en/legal/terms-of-use
