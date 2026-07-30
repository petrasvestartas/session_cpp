# corpus_occt_primitive_cases

Machine-readable corpus of **self-contained primitive boolean cases** mined from OCCT's own test
suite. Source tree read: `/home/petras/code/code_cpp/OCCT/tests/boolean/` (35 grids, 4093 case
files, listing date 2026-07-25). DRAW semantics cross-read from
`/home/petras/code/code_cpp/OCCT/src/Draw/TKTopTest/BRepTest/BRepTest_PrimitiveCommands.cxx`,
`.../BRepTest_BasicCommands.cxx`, and
`/home/petras/code/code_cpp/OCCT/resources/DrawResources/CheckCommands.tcl`.

**Definitions used throughout (all counts below are machine-computed over every case file, not sampled):**

- *external-data* — the case body matches any of `locate_data_file`, `restore`, `stepread`,
  `igesread`, `testreadshape`, `binrestore`, `source`, `ReadStep`, `xload`. NOT portable.
- *inline (self-contained)* — no such token; every operand is built procedurally.
- *primitive-only (PORTABLE)* — inline, uses at least one of `box psphere pcylinder pcone ptorus`,
  and uses **none** of the constructive/topological commands
  (`vertex edge plane mkface prism mkplane wire profile circle nurbsconvert mkedge cylinder cone
  sphere polyline mkcurve revol blend halfspace compound explode sewing mksurface line torus
  nexplode emptycopy treverse invert orientation add shape normals bounds cvalue uiso viso tcopy`).
  Only placement transforms (`ttranslate trotate tscale tmirror`), `dset`/`set`, boolean commands and
  `check*` are permitted.
- *prim+other* — inline, uses primitives **and** at least one constructive command (partially portable).
- *inline no-primitive* — inline but built entirely from faces/curves/planes (e.g. all of `volumemaker`).
- *asserts numeric* — has a `checkprops ... -s/-l/-v <number>` or any `checknbshapes`.
- *asserts empty* — has a `checkprops ... -s/-l/-v empty` (result must be void). This is an
  assertion too, and one of the most valuable ones: 228 case files assert an empty result.

---

## 1. CENSUS

Every one of the 35 grids in `tests/boolean/`. Columns are file counts (the per-grid `begin`/`end`
helper files are excluded; `grids.list` and `parse.rules` excluded).

| grid | total | external-data (not portable) | inline (self-contained) | **primitive-only (PORTABLE)** | prim+other geom | inline no-primitive | asserts numeric | asserts empty | files containing TODO |
|---|---|---|---|---|---|---|---|---|---|
| bcommon_2d | 143 | 143 | 0 | 0 | 0 | 0 | 143 | 18 | 0 |
| bcommon_complex | 25 | 25 | 0 | 0 | 0 | 0 | 25 | 0 | 0 |
| bcommon_simple | 83 | 0 | 83 | 13 | 29 | 41 | 72 | 11 | 0 |
| bcut_2d | 170 | 170 | 0 | 0 | 0 | 0 | 170 | 19 | 0 |
| bcut_complex | 153 | 153 | 0 | 0 | 0 | 0 | 146 | 7 | 3 |
| bcut_simple | 110 | 0 | 110 | 15 | 56 | 39 | 102 | 8 | 0 |
| bcutblend | 1 | 1 | 0 | 0 | 0 | 0 | 1 | 0 | 0 |
| bfuse_2d | 28 | 28 | 0 | 0 | 0 | 0 | 28 | 0 | 0 |
| bfuse_complex | 165 | 163 | 2 | 2 | 0 | 0 | 165 | 0 | 0 |
| bfuse_simple | 102 | 0 | 102 | 14 | 56 | 32 | 102 | 0 | 0 |
| bopcommon_2d | 143 | 143 | 0 | 0 | 0 | 0 | 143 | 18 | 0 |
| bopcommon_complex | 115 | 115 | 0 | 0 | 0 | 0 | 102 | 13 | 0 |
| bopcommon_simple | 378 | 0 | 378 | 371 | 7 | 0 | 314 | 64 | 0 |
| bopcut_2d | 32 | 32 | 0 | 0 | 0 | 0 | 32 | 0 | 0 |
| bopcut_complex | 140 | 140 | 0 | 0 | 0 | 0 | 137 | 3 | 1 |
| bopcut_simple | 379 | 0 | 379 | 371 | 6 | 2 | 373 | 6 | 0 |
| bopfuse_2d | 28 | 28 | 0 | 0 | 0 | 0 | 28 | 0 | 0 |
| bopfuse_complex | 128 | 128 | 0 | 0 | 0 | 0 | 128 | 0 | 0 |
| bopfuse_simple | 375 | 0 | 375 | 371 | 4 | 0 | 375 | 0 | 1 |
| bopsection | 48 | 47 | 1 | 0 | 1 | 0 | 48 | 0 | 0 |
| boptuc_2d | 140 | 140 | 0 | 0 | 0 | 0 | 140 | 19 | 0 |
| boptuc_complex | 73 | 73 | 0 | 0 | 0 | 0 | 66 | 7 | 0 |
| boptuc_simple | 373 | 0 | 373 | 371 | 2 | 0 | 345 | 28 | 0 |
| bsection | 164 | 161 | 3 | 2 | 0 | 1 | 164 | 0 | 2 |
| cells_test | 68 | 0 | 68 | 41 | 10 | 17 | 68 | 0 | 0 |
| gdml_private | 326 | 326 | 0 | 0 | 0 | 0 | 21 | 1 | 8 |
| gdml_public | 25 | 0 | 25 | 0 | 25 | 0 | 1 | 0 | 0 |
| history | 9 | 0 | 9 | 0 | 4 | 5 | 9 | 0 | 0 |
| mkconnected | 5 | 2 | 3 | 0 | 3 | 0 | 5 | 0 | 0 |
| opensolid | 9 | 0 | 9 | 4 | 5 | 0 | 9 | 2 | 0 |
| periodicity | 6 | 0 | 6 | 3 | 2 | 1 | 6 | 0 | 0 |
| removefeatures | 58 | 57 | 1 | 0 | 1 | 0 | 58 | 0 | 1 |
| simplify | 5 | 1 | 4 | 1 | 2 | 1 | 5 | 0 | 0 |
| splitter | 12 | 2 | 10 | 2 | 6 | 2 | 12 | 0 | 0 |
| volumemaker | 74 | 0 | 74 | 0 | 0 | 74 | 66 | 4 | 1 |
| **TOTAL** | **4093** | **2078** | **2015** | **1581** | **219** | **215** | **3609** | **228** | **17** |
### 1.1 Census readout

- **4093** case files total. **2078** need external data (50.8%) — every `*_2d`, every `*_complex`
  (except 2), all of `gdml_private`, and most of `bsection`/`bopsection`. Not portable, period.
- **2015** are self-contained. Of those, **1581 are primitive-only and directly portable**,
  **219** mix primitives with constructive geometry, **215** use no primitive at all.
- The portable mass is overwhelmingly concentrated in four grids:
  `bopcommon_simple` / `bopcut_simple` / `bopfuse_simple` / `boptuc_simple` = **371 portable cases each = 1484**
  (93.9% of the whole portable corpus).
- **Measured cross-grid fact:** those four grids contain **373 case names present in all four**;
  for **371** of them the operand construction is **byte-identical across all four grids** (verified by
  string comparison of the geometry block); only `ZP3` and `ZP4` share a name but not geometry.
  Those 371 names are exactly the 371 primitive-only cases in each grid. **Therefore the 1484 core
  cases are 371 distinct geometry pairs x 4 operations**, and Table 2.1 below encodes all 1484 of
  them with one row per pair and four expected-value columns.
- **3609** case files assert a numeric result; **228** assert an empty result.
- **16** case files carry an active `puts "TODO ...` (OCCT's own known-bad marker); 2 more carry a
  commented-out one. Full list in section 5.

---

## 2. THE PORTABLE CORPUS

### 2.1 CORE GRID — 371 geometry pairs x 4 ops = 1484 portable cases

**How to read a row.** Row `A5` means these four OCCT cases exist and all four use the same two
operands:

| OCCT case id | operation | DRAW call | expected |
|---|---|---|---|
| `boolean/bopcommon_simple/A5` | COMMON (A ∩ B) | `bop b1 b2` then `bopcommon result` | `checkprops result -s 2.5` |
| `boolean/bopcut_simple/A5` | CUT (A − B) | `bop b1 b2` then `bopcut result` | `checkprops result -s 5.5` |
| `boolean/bopfuse_simple/A5` | FUSE (A ∪ B) | `bop b1 b2` then `bopfuse result` | `checkprops result -s 6` |
| `boolean/boptuc_simple/A5` | TUC (B − A) | `bop b1 b2` then `boptuc result` | `checkprops result -s empty` |

Every one of the 1484 cases asserts exactly **one** `checkprops result -s <value>` line and nothing
else (verified: 1484/1484). `-s` = **surface area** of the result (not volume). `empty` means the
result must be a void shape. The `prelude` column holds the `dset` lines that must be evaluated
before the operand line (Tcl `expr` semantics; `sqrt`, `atan2`, `pi` are Tcl math). The
`class` column is machine-derived (see section 4 for how).

| case | prelude (dset) | operand A | operand B | COMMON `-s` | CUT A−B `-s` | FUSE `-s` | TUC B−A `-s` | pair | class |
|---|---|---|---|---|---|---|---|---|---|
| A1 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 1 1 1` | 6 | empty | 6 | empty | boxxbox | IDENTICAL-OPERANDS, COINCIDENT-SOLIDS, NESTED, coplanar-faces=6 |
| A2 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 1 1.5 1` | 6 | empty | 8 | 4 | boxxbox | NESTED, coplanar-faces=5 |
| A3 | - | `box b1 0 0 0 1 1 1` | `box b2 0 1 0 1 0.5 1` | empty | 6 | 8 | 4 | boxxbox | FACE-TOUCH, coplanar-faces=5, COMMON-EMPTY |
| A4 | - | `box b1 0 0 0 1 1 1` | `box b2 1 1 0 1 1 1` | empty | 6 | 12 | 6 | boxxbox | EDGE-TOUCH, coplanar-faces=4, COMMON-EMPTY |
| A5 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 0.5 1 0.5` | 2.5 | 5.5 | 6 | empty | boxxbox | NESTED, coplanar-faces=4 |
| A6 | - | `box b1 0 0 0 1 1 1` | `box b2 0 -0.5 0 0.5 0.5 1` | empty | 6 | 7.5 | 2.5 | boxxbox | FACE-TOUCH, coplanar-faces=4, COMMON-EMPTY |
| A7 | - | `box b1 0 0 0 1 1 1` | `box b2 0 -0.5 0 0.5 1.5 1` | 4 | 4 | 7.5 | 2.5 | boxxbox | OVERLAP, coplanar-faces=4 |
| A8 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.5 0 1 1 1` | 4 | 4 | 8 | 4 | boxxbox | OVERLAP, coplanar-faces=4 |
| A9 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 1 0.5 1` | 4 | 6 | 6 | empty | boxxbox | NESTED, coplanar-faces=4 |
| B1 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 0.5 0.5 0.5` | 1.5 | 6 | 6 | empty | boxxbox | NESTED, coplanar-faces=3 |
| B2 | - | `box b1 0 0 0 1 1 1` | `box b2 0 -0.5 0 0.5 0.5 0.5` | empty | 6 | 7 | 1.5 | boxxbox | FACE-TOUCH, coplanar-faces=3, COMMON-EMPTY |
| B3 | - | `box b1 0 0 0 1 1 1` | `box b2 0 -0.5 -0.5 0.5 0.5 0.5` | empty | 6 | 7.5 | 1.5 | boxxbox | EDGE-TOUCH, coplanar-faces=3, COMMON-EMPTY |
| B4 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.5 -0.5 -0.5 0.5 0.5 0.5` | empty | 6 | 7.5 | 1.5 | boxxbox | VERTEX-TOUCH, coplanar-faces=3, COMMON-EMPTY |
| B5 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 1.5 0.5 0.5` | 2.5 | 5.5 | 7 | 1.5 | boxxbox | OVERLAP, coplanar-faces=3 |
| B6 | - | `box b1 0 0 0 1 1 1` | `box b2 0 -0.5 0 1.5 0.5 0.5` | empty | 6 | 8.5 | 3.5 | boxxbox | FACE-TOUCH, coplanar-faces=3, COMMON-EMPTY |
| B7 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0 0 0.5 0.5 1` | 2.5 | 6.5 | 6 | empty | boxxbox | NESTED, coplanar-faces=3 |
| B8 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 -0.5 0 0.5 0.5 1` | empty | 6 | 7.5 | 2.5 | boxxbox | FACE-TOUCH, coplanar-faces=3, COMMON-EMPTY |
| B9 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0 0 0.5 1.5 1` | 4 | 6 | 7.5 | 2.5 | boxxbox | OVERLAP, coplanar-faces=3 |
| C1 | - | `box b1 0 0 0 1 1 1` | `box b2 0.5 0 0 1 1 0.5` | 2.5 | 5.5 | 7.5 | 2.5 | boxxbox | OVERLAP, coplanar-faces=3 |
| C2 | - | `box b1 0 0 0 1 1 1` | `box b2 0.5 0 -0.5 1 1 0.5` | empty | 6 | 9 | 4 | boxxbox | FACE-TOUCH, coplanar-faces=3, COMMON-EMPTY |
| C3 | dset r sqrt(2) | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 r r/2 1 ; trotate b2 0 0 0 0 0 1 45` | 4.41421 | 4.41421 | 7.82843 | 5.82843 | boxxbox | OBLIQUE |
| C4 | dset r sqrt(2)/2 | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 r r 1 ; trotate b2 0 0 0 0 0 1 45` | 2.91421 | 5.91421 | 6.91421 | 2.91421 | boxxbox | OBLIQUE |
| C5 | dset r sqrt(2) | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 r 0.25 1 ; trotate b2 0 0 0 0 0 1 45` | 3.61764 | 7.03921 | 6.41789 | 1.83211 | boxxbox | OBLIQUE |
| C6 | dset r sqrt(31) | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 r/4 0.25 1 ; trotate b2 0 0 0 0 0 1 34.73` | 3.65032 | 7.21677 | 6.32953 | 1.54631 | boxxbox | OBLIQUE |
| C7 | dset r sqrt(2) | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 1.5 r/2 1 ; trotate b2 0 0 0 0 0 1 45` | 4.41421 | 4.41421 | 8.12132 | 6.12132 | boxxbox | OBLIQUE |
| C8 | dset a30 atan2(1,2)*180/pi; dset r sqrt(5) | `box b1 0 0 0 1 1 1` | `box b2 0 0.5 0 r/2 1/r 1 ; trotate b2 0 0.5 0 0 0 1 a30` | 3.11803 | 5.11803 | 7.01246 | 4.01246 | boxxbox | OBLIQUE |
| C9 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 0.5 0.5 1 ; trotate b2 0 0 0 0 0 1 45` | 1.95711 | 6.04289 | 6.54289 | 1.95711 | boxxbox | OBLIQUE |
| D1 | dset a30 atan2(1,2)*180/pi; dset r sqrt(5) | `box b1 0 0 0 1 1 1` | `box b2 0 0.5 0 1/r r/2 1 ; trotate b2 0 0.5 0 0 0 1 a30-90` | 3.85967 | 7.05967 | 6.27082 | 1.27082 | boxxbox | OBLIQUE |
| D2 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.5 0 0 0.5 1.7 1 ; trotate b2 0 0 0 0 0 1 -30` | 3.3094 | 5 | 8.7906 | 5.9453 | boxxbox | OBLIQUE |
| D3 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 1.5 0.25 1 ; trotate b2 0 0 0 0 0 1 45` | 3.61764 | 7.03921 | 6.63236 | 2.04657 | boxxbox | OBLIQUE |
| D4 | dset r sqrt(31) | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 r/4 0.5 1 ; trotate b2 0 0 0 0 0 1 34.73` | 4.33997 | 5.46836 | 6.83585 | 3.53635 | boxxbox | OBLIQUE |
| D5 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 0.5 0.5 1 ; trotate b2 0 0 0 0 0 1 30` | 2.14434 | 6.27831 | 6.35566 | 1.51036 | boxxbox | OBLIQUE |
| D6 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 0.5 0.5 1 ; trotate b2 0 0 0 0 0 1 60` | 1.51036 | 6.06699 | 6.98964 | 2.14434 | boxxbox | OBLIQUE |
| D7 | - | `box b1 0 0 0 1 1 1` | `box b2 1 1 0 1 1 1 ; trotate b2 1 1 1 0 0 1 30` | empty | 6 | 12 | 6 | boxxbox | OBLIQUE, COMMON-EMPTY |
| D8 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 0 0 0.25 1.7 1 ; trotate b2 0 0 0 0 0 1 -30` | 2.95374 | 6.51036 | 7.79626 | 3.37361 | boxxbox | OBLIQUE |
| D9 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.5 0 0 0.5 1.7 1 ; trotate b2 0 0 0 0 0 1 -50` | 4.26257 | 5.42883 | 7.83743 | 4.63685 | boxxbox | OBLIQUE |
| E1 | dset r sqrt(30) | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 r/4 0.25 1 ; trotate b2 0 0 0 0 0 1 40` | 3.62194 | 7.24544 | 6.30133 | 1.44401 | boxxbox | OBLIQUE |
| E2 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0 0 0.5 0.5 0.5` | 1.5 | 6.5 | 6 | empty | boxxbox | NESTED, coplanar-faces=2 |
| E3 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 -0.5 0 0.5 0.5 0.5` | empty | 6 | 7 | 1.5 | boxxbox | FACE-TOUCH, coplanar-faces=2, COMMON-EMPTY |
| E4 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 -0.5 -0.5 0.5 0.5 0.5` | empty | 6 | 7.5 | 1.5 | boxxbox | EDGE-TOUCH, coplanar-faces=2, COMMON-EMPTY |
| E5 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0 0 0.5 1.5 0.5` | 2.5 | 6.5 | 7 | 1.5 | boxxbox | OVERLAP, coplanar-faces=2 |
| E6 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0 0 1.5 0.5 0.5` | 2 | 6 | 7.5 | 2 | boxxbox | OVERLAP, coplanar-faces=2 |
| E7 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 -0.5 0 1.5 0.5 0.5` | empty | 6 | 8.75 | 3.5 | boxxbox | FACE-TOUCH, coplanar-faces=2, COMMON-EMPTY |
| E8 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 -0.5 -0.5 1.5 0.5 0.5` | empty | 6 | 9.5 | 3.5 | boxxbox | EDGE-TOUCH, coplanar-faces=2, COMMON-EMPTY |
| E9 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0 0 1.5 1.5 0.5` | 3.25 | 5.25 | 10.25 | 6 | boxxbox | OVERLAP, coplanar-faces=2 |
| F1 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 0 0 1.5 0.5 0.5` | 2.5 | 5.5 | 7 | 2 | boxxbox | OVERLAP, coplanar-faces=2 |
| F2 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 -0.5 0 1.5 0.5 0.5` | empty | 6 | 8.5 | 3.5 | boxxbox | FACE-TOUCH, coplanar-faces=2, COMMON-EMPTY |
| F3 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 -0.5 -0.5 1.5 0.5 0.5` | empty | 6 | 9.5 | 3.5 | boxxbox | EDGE-TOUCH, coplanar-faces=2, COMMON-EMPTY |
| F4 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0.25 0 1.5 0.5 0.5` | 2 | 6.75 | 7.5 | 2 | boxxbox | OVERLAP, coplanar-faces=1 |
| F5 | dset r sqrt(2)/2 | `box b1 0 0 0 1 1 1` | `box b2 0 0.5 0 r r 1 ; trotate b2 0 0.5 0 0 0 1 -45` | 3.82843 | 7.82843 | 6 | empty | boxxbox | OBLIQUE |
| F6 | dset r sqrt(2)*0.75 | `box b1 0 0 0 1 1 1` | `box b2 0 0.5 0 r r 1 ; trotate b2 0 0.5 0 0 0 1 -45` | 4.91421 | 3.91421 | 7.57843 | 5.57843 | boxxbox | OBLIQUE |
| F7 | dset a30 atan2(1,2)*180/pi; dset r sqrt(5) | `box b1 0 0 0 1 1 1` | `box b2 0 0.5 0 1.25 1/r 1 ; trotate b2 0 0.5 0 0 0 1 a30` | 3.11803 | 5.11803 | 7.39443 | 4.39443 | boxxbox | OBLIQUE |
| F8 | dset r sqrt(2) | `box b1 0 0 0 1 1 1` | `box b2 -0.25 -0.25 0 2 r/2 1 ; trotate b2 -0.25 -0.25 0 0 0 1 45` | 4.41421 | 4.41421 | 9.82843 | 7.82843 | boxxbox | OBLIQUE |
| F9 | dset r sqrt(2)/2 | `box b1 0 0 0 1 1 1` | `box b2 0 0.5 0 r r/2 1 ; trotate b2 0 0.5 0 0 0 1 45` | 1.95711 | 5.45711 | 6.66421 | 2.66421 | boxxbox | OBLIQUE |
| G1 | dset a30 atan2(1,2)*180/pi; dset r sqrt(5) | `box b1 0 0 0 1 1 1` | `box b2 0 0.5 0 2/r 1.25 1 ; trotate b2 0 0.5 0 0 0 1 a30-90` | 4.80205 | 4.55205 | 7.72287 | 5.22287 | boxxbox | OBLIQUE |
| G2 | dset a30 atan2(1,2)*180/pi; dset r sqrt(5)/4 | `box b1 0 0 0 1 1 1` | `box b2 0.25 0 0 1.5*r r 1 ; trotate b2 0.25 0 0 0 0 1 a30` | 3.73258 | 7.85758 | 6 | empty | boxxbox | OBLIQUE |
| G3 | dset r sqrt(2)/2 | `box b1 0 0 0 1 1 1` | `box b2 1-r 0 0 2*r 1.5*r 1 ; trotate b2 1 0 0 0 0 1 45` | 5.45711 | 1.95711 | 8.49264 | 8.49264 | boxxbox | OBLIQUE |
| G4 | dset r sqrt(2) | `box b1 0 0 0 1 1 1` | `box b2 -0.25 -0.25 0 2 r/4 1 ; trotate b2 -0.25 -0.25 0 0 0 1 45` | 3.87132 | 6.37132 | 8.25 | 4.25 | boxxbox | OBLIQUE |
| G5 | dset a30 atan2(1,2)*180/pi; dset r sqrt(5) | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 1.3 3/r/2 1 ; trotate b2 0 0.25 0 0 0 1 a30` | 4.11803 | 4.11803 | 7.56774 | 5.56774 | boxxbox | OBLIQUE |
| G6 | dset a30 atan2(1,2)*180/pi; dset r sqrt(5) | `box b1 0 0 0 1 1 1` | `box b2 0 0.45 0 1/r r/2 1 ; trotate b2 0 0.45 0 0 0 1 a30-90` | 3.85967 | 7.05967 | 6.27082 | 1.27082 | boxxbox | OBLIQUE |
| G7 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 0.5 0.5 1 ; trotate b2 0 .25 0 0 0 1 45` | 1.95711 | 6.04289 | 6.54289 | 1.95711 | boxxbox | OBLIQUE |
| G8 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 0.5 0.5 1 ; trotate b2 0 .25 0 0 0 1 -30` | 2.5 | 7.5 | 6 | empty | boxxbox | OBLIQUE |
| G9 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 0.5 0.5 1 ; trotate b2 0 .25 0 0 0 1 240` | 1.29127 | 5.70873 | 7.20873 | 2.57476 | boxxbox | OBLIQUE |
| H1 | dset r sqrt(2)*0.875 | `box b1 0 0 0 1 1 1` | `box b2 0 0.75 0 r r 1 ; trotate b2 0 0.75 0 0 0 1 -45` | 4.78921 | 4.03921 | 9.22303 | 7.22303 | boxxbox | OBLIQUE |
| H2 | dset a30 atan2(1,2)*180/pi; dset r sqrt(5)*0.5 | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 r r 1 ; trotate b2 0 0.25 0 0 0 1 a30` | 3.99129 | 4.80379 | 8.98084 | 6.23084 | boxxbox | OBLIQUE |
| H3 | dset r sqrt(2)*0.5 | `box b1 0 0 0 1 1 1` | `box b2 0.5 0 0 1 r 1 ; trotate b2 0.5 0 0 0 0 1 45` | 4.31371 | 6.17157 | 6.51472 | 2.17157 | boxxbox | OBLIQUE |
| H4 | dset r sqrt(2)*0.5 | `box b1 0 0 0 1 1 1` | `box b2 0.5 0 0 1.5 r 1 ; trotate b2 0.5 0 0 0 0 1 45` | 4.37132 | 5.87132 | 8.16421 | 4.16421 | boxxbox | OBLIQUE |
| H5 | dset a30 atan2(1,2)*180/pi; dset r sqrt(5) | `box b1 0 0 0 1 1 1` | `box b2 -0.1 0 0 1.5 1/r 1 ; trotate b2 1 0 0 0 0 1 a30` | 3.11803 | 5.11803 | 8.11803 | 5.11803 | boxxbox | OBLIQUE |
| H6 | dset r sqrt(2) | `box b1 0 0 0 1 1 1` | `box b2 0 -0.25 0 r 0.5 1 ; trotate b2 0 0 0 0 0 1 45` | 4.40685 | 5.25 | 6.83579 | 3.66421 | boxxbox | OBLIQUE |
| H7 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 0.1 0.5 1 ; trotate b2 0 .25 0 0 0 1 240` | 1.16144 | 6.37668 | 6.13856 | 0.769504 | boxxbox | OBLIQUE |
| H8 | dset a30 atan2(1,2)*180/pi; dset r sqrt(5)*0.5 | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 r 0.5 1 ; trotate b2 0 0.25 0 0 0 1 a30` | 3.77245 | 5.87674 | 6.58166 | 2.68576 | boxxbox | OBLIQUE |
| H9 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0.25 0 0.5 0.5 1 ; trotate b2 .25 .25 0 0 0 1 30` | 2.5 | 7.5 | 6 | empty | boxxbox | OBLIQUE |
| I1 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0.25 0 0.5 0.5 1 ; trotate b2 .25 .25 0 0 0 1 120` | 1.51036 | 6.06699 | 6.98964 | 2.14434 | boxxbox | OBLIQUE |
| I2 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 0.5 0.5 1 ; trotate b2 0 .25 0 0 0 1 35` | 2.08523 | 6.21456 | 6.41477 | 1.63554 | boxxbox | OBLIQUE |
| I3 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 0.5 0.5 1 ; trotate b2 0 .25 0 0 0 1 115` | empty | 6 | 8.5 | 2.5 | boxxbox | OBLIQUE, COMMON-EMPTY |
| I4 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 0.5 0.5 1 ; trotate b2 0 .25 0 0 0 1 230` | 1.01135 | 5.76651 | 7.48865 | 2.58452 | boxxbox | OBLIQUE |
| I5 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 0.5 0.5 1 ; trotate b2 0 .25 0 0 0 1 250` | 1.42746 | 5.74066 | 7.07254 | 2.45473 | boxxbox | OBLIQUE |
| I6 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.5 0 1.5 1.5 1 ; trotate b2 0 0.5 0 0 0 1 -45` | 4.91421 | 3.91421 | 11.5858 | 9.58579 | boxxbox | OBLIQUE |
| I7 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.5 0 0.8 0.8 1 ; trotate b2 0 0.5 0 0 0 1 -45` | 4.21056 | 6.70315 | 6.26944 | 1.3204 | boxxbox | OBLIQUE |
| I8 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 0.8 0.8 1 ; trotate b2 0 0.25 0 0 0 1 -45` | 3.87582 | 5.99525 | 6.60418 | 2.39241 | boxxbox | OBLIQUE |
| I9 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 0.5 0.5 1 ; trotate b2 0 0.25 0 0 0 1 -45` | 2.39277 | 7.02145 | 6.10723 | 0.521447 | boxxbox | OBLIQUE |
| J1 | dset r sqrt(2)*0.75 | `box b1 0 0 0 1 1 1` | `box b2 0 0.625 0 r r 1 ; trotate b2 0 0.625 0 0 0 1 -45` | 4.79412 | 4.38787 | 7.69853 | 5.19853 | boxxbox | OBLIQUE |
| J2 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 0 0 0.5 1.7 1 ; trotate b2 0 0 0 0 0 1 -45` | 4.40685 | 5.25 | 7.69315 | 4.52157 | boxxbox | OBLIQUE |
| J3 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 0 0 0.5 1.7 1 ; trotate b2 0 0 0 0 0 1 -30` | 4.10844 | 5.35566 | 7.99156 | 4.72361 | boxxbox | OBLIQUE |
| J4 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 -0.25 0 0.25 2 1 ; trotate b2 0 0 0 0 0 1 -30` | 2.95374 | 6.51036 | 8.54626 | 4.12361 | boxxbox | OBLIQUE |
| J5 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 0 0 0.5 1.3 1 ; trotate b2 0 0 0 0 0 1 -45` | 4.28615 | 5.82756 | 6.61385 | 2.79619 | boxxbox | OBLIQUE |
| J6 | - | `box b1 0 0 0 1 1 1` | `box b2 0 -0.25 0 0.5 1.55 1 ; trotate b2 0 0 0 0 0 1 -35` | 4.24099 | 5.95381 | 7.40901 | 3.56678 | boxxbox | OBLIQUE |
| J7 | - | `box b1 0 0 0 1 1 1` | `box b2 0.1 0.25 0 0.5 0.5 1 ; trotate b2 0 .25 0 0 0 1 230` | 0.464678 | 5.90701 | 8.03532 | 2.55897 | boxxbox | OBLIQUE |
| J8 | - | `box b1 0 0 0 1 1 1` | `box b2 0.1 0.4 0 0.5 0.5 1 ; trotate b2 0.1 .4 0 0 0 1 245` | 1.90425 | 5.98538 | 6.59575 | 1.89718 | boxxbox | OBLIQUE |
| J9 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 -0.25 0 0.5 0.5 1` | 1.125 | 5.875 | 7.375 | 2.375 | boxxbox | OVERLAP, coplanar-faces=2 |
| K1 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0.25 0 0.5 0.5 1` | 2.5 | 7.5 | 6 | empty | boxxbox | NESTED, coplanar-faces=2 |
| K2 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0.25 0 0.5 0.5 1 ; trotate b2 .25 .25 0 0 0 1 45` | 2.39277 | 7.02145 | 6.10723 | 0.521447 | boxxbox | OBLIQUE |
| K3 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0.25 0 0.5 0.5 1 ; trotate b2 .25 .25 0 0 0 1 90` | 1.75 | 6.25 | 6.75 | 1.75 | boxxbox | OVERLAP, coplanar-faces=2 |
| K4 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0.25 0 0.5 0.5 1 ; trotate b2 .25 .25 0 0 0 1 125` | 1.40617 | 6.07594 | 7.09383 | 2.15801 | boxxbox | OBLIQUE |
| K5 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 -0.25 0 0.5 1.5 1` | 4 | 6 | 7.5 | 3.5 | boxxbox | OVERLAP, coplanar-faces=2 |
| K6 | - | `box b1 0 0 0 1 1 1` | `box b2 0.9 -0.25 0 0.5 1.7 1 ; trotate b2 0.9 -0.25 0 0 0 1 45` | 4.40683 | 5.25003 | 7.69317 | 4.5216 | boxxbox | OBLIQUE |
| K7 | - | `box b1 0 0 0 1 1 1` | `box b2 1.25 0 0 0.25 1.7 1 ; trotate b2 1.1 0 0 0 0 1 45` | 2.51409 | 6.37708 | 8.23591 | 3.65013 | boxxbox | OBLIQUE |
| K8 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 -0.25 0 0.5 1.7 1 ; trotate b2 0 0 0 0 0 1 -30` | 4.10844 | 5.35566 | 7.99156 | 4.72361 | boxxbox | OBLIQUE |
| K9 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 -0.25 0 0.5 1.55 1 ; trotate b2 0 0 0 0 0 1 -45` | 4.28615 | 5.82756 | 7.36385 | 3.54619 | boxxbox | OBLIQUE |
| L1 | - | `box b1 0 0 0 1 1 1` | `box b2 0.5 -0.25 0 1 1 1 ; trotate b2 0.5 -0.25 0 0 0 1 45` | 4.9632 | 4.35051 | 7.0368 | 4.35051 | boxxbox | OBLIQUE |
| L2 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.15 0 0.8 0.9 1 ; trotate b2 0 0.25 0 0 0 1 -45` | 4.00937 | 5.4617 | 6.83063 | 3.18454 | boxxbox | OBLIQUE |
| L3 | - | `box b1 0 0 0 1 1 1` | `box b2 0.5 -0.25 0 0.5 1 1 ; trotate b2 0.5 -0.25 0 0 0 1 45` | 3.41053 | 6.00368 | 6.58947 | 2.41789 | boxxbox | OBLIQUE |
| L4 | dset r sqrt(2) | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 r r/2 0.5 ; trotate b2 0 0 0 0 0 1 45` | 2.70711 | 5.70711 | 7.41421 | 3.41421 | boxxbox | OBLIQUE |
| L5 | dset r sqrt(2)/2 | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 r r 0.5 ; trotate b2 0 0 0 0 0 1 45` | 1.70711 | 6.20711 | 6.70711 | 1.70711 | boxxbox | OBLIQUE |
| L6 | dset r sqrt(2) | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 r 0.25 0.5 ; trotate b2 0 0 0 0 0 1 45` | 2.09987 | 6.81066 | 6.27145 | 0.978553 | boxxbox | OBLIQUE |
| L7 | dset r sqrt(31) | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 r/4 0.25 0.5 ; trotate b2 0 0 0 0 0 1 34.73` | 2.12935 | 6.91258 | 6.20856 | 0.816945 | boxxbox | OBLIQUE |
| L8 | dset r sqrt(2) | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 1.5 r/2 0.5 ; trotate b2 0 0 0 0 0 1 45` | 2.70711 | 5.70711 | 7.62132 | 3.62132 | boxxbox | OBLIQUE |
| L9 | dset a30 atan2(1,2)*180/pi; dset r sqrt(5) | `box b1 0 0 0 1 1 1` | `box b2 0 0.5 0 r/2 1/r 0.5 ; trotate b2 0 0.5 0 0 0 1 a30` | 1.80902 | 5.80902 | 6.75623 | 2.25623 | boxxbox | OBLIQUE |
| M1 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 0.5 0.5 0.5 ; trotate b2 0 0 0 0 0 1 45` | 1.10355 | 6.14645 | 6.39645 | 1.10355 | boxxbox | OBLIQUE |
| M2 | dset a30 atan2(1,2)*180/pi; dset r sqrt(5) | `box b1 0 0 0 1 1 1` | `box b2 0 0.5 0 1/r r/2 0.5 ; trotate b2 0 0.5 0 0 0 1 a30-90` | 2.37984 | 6.97984 | 6.18541 | 0.68541 | boxxbox | OBLIQUE |
| M3 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.5 0 0 0.5 1.7 0.5 ; trotate b2 0 0 0 0 0 1 -30` | 1.94338 | 5.78868 | 7.95662 | 3.53397 | boxxbox | OBLIQUE |
| M4 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 1.5 0.25 0.5 ; trotate b2 0 0 0 0 0 1 45` | 2.09987 | 6.81066 | 6.40013 | 1.10723 | boxxbox | OBLIQUE |
| M5 | dset r sqrt(31) | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 r/4 0.5 0.5 ; trotate b2 0 0 0 0 0 1 34.73` | 2.71277 | 6.27696 | 6.57112 | 1.92137 | boxxbox | OBLIQUE |
| M6 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 0.5 0.5 0.5 ; trotate b2 0 0 0 0 0 1 30` | 1.25 | 6.31699 | 6.25 | 0.82735 | boxxbox | OBLIQUE |
| M7 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 0.5 0.5 0.5 ; trotate b2 0 0 0 0 0 1 60` | 0.82735 | 6.10566 | 6.67265 | 1.25 | boxxbox | OBLIQUE |
| M8 | - | `box b1 0 0 0 1 1 1` | `box b2 1 1 0 1 1 0.5 ; trotate b2 1 1 1 0 0 1 30` | empty | 6 | 10 | 4 | boxxbox | OBLIQUE, COMMON-EMPTY |
| M9 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 0 0 0.25 1.7 0.5 ; trotate b2 0 0 0 0 0 1 -30` | 1.69338 | 6.47169 | 7.10662 | 1.8953 | boxxbox | OBLIQUE |
| N1 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.5 0 0 0.5 1.7 0.5 ; trotate b2 0 0 0 0 0 1 -50` | 2.63987 | 6.223 | 7.26013 | 2.65985 | boxxbox | OBLIQUE |
| N2 | dset r sqrt(30) | `box b1 0 0 0 1 1 1` | `box b2 0 0 0 r/4 0.25 0.5 ; trotate b2 0 0 0 0 0 1 40` | 2.11942 | 6.93117 | 6.18454 | 0.755878 | boxxbox | OBLIQUE |
| N3 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0.25 0 0.5 0.5 -1 ; trotate b2 .25 .25 0 0 0 1 30` | empty | 6 | 8 | 2.5 | boxxbox | OBLIQUE, COMMON-EMPTY |
| N4 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0.25 0 0.5 0.5 -1 ; trotate b2 .25 .25 0 0 0 1 120` | empty | 6 | 8.35566 | 2.5 | boxxbox | OBLIQUE, COMMON-EMPTY |
| N5 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 0.5 0.5 -1 ; trotate b2 0 .25 0 0 0 1 35` | empty | 6 | 8.17505 | 2.5 | boxxbox | OBLIQUE, COMMON-EMPTY |
| N6 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 0.5 0.5 -1 ; trotate b2 0 .25 0 0 0 1 115` | empty | 6 | 8.5 | 2.5 | boxxbox | OBLIQUE, COMMON-EMPTY |
| N7 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 0.5 0.5 -1 ; trotate b2 0 .25 0 0 0 1 230` | empty | 6 | 8.42552 | 2.5 | boxxbox | OBLIQUE, COMMON-EMPTY |
| N8 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 0.5 0.5 -1 ; trotate b2 0 .25 0 0 0 1 250` | empty | 6 | 8.3477 | 2.5 | boxxbox | OBLIQUE, COMMON-EMPTY |
| N9 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.5 0 1.5 1.5 -1 ; trotate b2 0 0.5 0 0 0 1 -45` | empty | 6 | 15 | 10.5 | boxxbox | OBLIQUE, COMMON-EMPTY |
| O1 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.5 0 0.8 0.8 -1 ; trotate b2 0 0.5 0 0 0 1 -45` | empty | 6 | 9.25177 | 4.48 | boxxbox | OBLIQUE, COMMON-EMPTY |
| O2 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 0.8 0.8 -1 ; trotate b2 0 0.25 0 0 0 1 -45` | empty | 6 | 9.43383 | 4.48 | boxxbox | OBLIQUE, COMMON-EMPTY |
| O3 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0 0.5 0.5 -1 ; trotate b2 0 0.25 0 0 0 1 -45` | empty | 6 | 8.02145 | 2.5 | boxxbox | OBLIQUE, COMMON-EMPTY |
| O4 | dset r sqrt(2)*0.75 | `box b1 0 0 0 1 1 1` | `box b2 0 0.625 0 r r -1 ; trotate b2 0 0.625 0 0 0 1 -45` | empty | 6 | 11.0395 | 6.49264 | boxxbox | OBLIQUE, COMMON-EMPTY |
| O5 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 0 0 0.5 1.7 -1 ; trotate b2 0 0 0 0 0 1 -45` | empty | 6 | 10.9358 | 6.1 | boxxbox | OBLIQUE, COMMON-EMPTY |
| O6 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 0 0 0.5 1.7 -1 ; trotate b2 0 0 0 0 0 1 -30` | empty | 6 | 11.0896 | 6.1 | boxxbox | OBLIQUE, COMMON-EMPTY |
| O7 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 -0.25 0 0.25 2 -1 ; trotate b2 0 0 0 0 0 1 -30` | empty | 6 | 11.067 | 5.5 | boxxbox | OBLIQUE, COMMON-EMPTY |
| O8 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 0 0 0.5 1.3 -1 ; trotate b2 0 0 0 0 0 1 -45` | empty | 6 | 9.76188 | 4.9 | boxxbox | OBLIQUE, COMMON-EMPTY |
| O9 | - | `box b1 0 0 0 1 1 1` | `box b2 0 -0.25 0 0.5 1.55 -1 ; trotate b2 0 0 0 0 0 1 -35` | empty | 6 | 10.5853 | 5.65 | boxxbox | OBLIQUE, COMMON-EMPTY |
| P1 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.25 0.5 0.5 0.5 1 ; trotate b2 0 .25 0 0 0 1 230` | 0.542919 | 5.9205 | 7.95708 | 2.5795 | boxxbox | OBLIQUE |
| P2 | - | `box b1 0 0 0 1 1 1` | `box b2 0.1 0.4 0.5 0.5 0.5 1 ; trotate b2 0.1 .4 0 0 0 1 245` | 1.10649 | 6.14705 | 7.39351 | 2.35295 | boxxbox | OBLIQUE |
| P3 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 -0.25 0.5 0.5 0.5 1` | 0.625 | 6 | 7.875 | 2.5 | boxxbox | OVERLAP |
| P4 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0.5 0.5 0.5 0.5 1` | 1.5 | 6.5 | 7 | 1.5 | boxxbox | OVERLAP, coplanar-faces=1 |
| P5 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0.5 0.5 0.5 0.5 1 ; trotate b2 .25 .25 0 0 0 1 45` | 1.12132 | 6.10355 | 7.37868 | 2.39645 | boxxbox | OBLIQUE |
| P6 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0.5 0.5 0.5 0.5 1 ; trotate b2 .25 .25 0 0 0 1 90` | empty | 6 | 8 | 2.5 | boxxbox | FACE-TOUCH, coplanar-faces=1, COMMON-EMPTY |
| P7 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 0.25 0.5 0.5 0.5 1 ; trotate b2 .25 .25 0 0 0 1 125` | 0.769594 | 6.10448 | 7.73041 | 2.39552 | boxxbox | OBLIQUE |
| P8 | - | `box b1 0 0 0 1 1 1` | `box b2 0.25 -0.25 0.5 0.5 1.5 1` | 2.5 | 6.5 | 9 | 5 | boxxbox | OVERLAP |
| P9 | - | `box b1 0 0 0 1 1 1` | `box b2 0.9 -0.25 0.5 0.5 1.7 1 ; trotate b2 0.9 -0.25 0 0 0 1 45` | 2.78551 | 6.20711 | 9.31449 | 5.89289 | boxxbox | OBLIQUE |
| Q1 | - | `box b1 0 0 0 1 1 1` | `box b2 1.25 0 0.5 0.25 1.7 1 ; trotate b2 1.1 0 0 0 0 1 45` | 1.43774 | 6.36924 | 9.31226 | 4.38076 | boxxbox | OBLIQUE |
| Q2 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 -0.25 0.5 0.5 1.7 1 ; trotate b2 0 0 0 0 0 1 -30` | 2.5594 | 6.18301 | 9.5406 | 5.91699 | boxxbox | OBLIQUE |
| Q3 | - | `box b1 0 0 0 1 1 1` | `box b2 -0.25 -0.25 0.5 0.5 1.55 1 ; trotate b2 0 0 0 0 0 1 -45` | 2.71214 | 6.48284 | 8.93786 | 5.16716 | boxxbox | OBLIQUE |
| Q4 | - | `box b1 0 0 0 1 1 1` | `box b2 0.5 -0.25 0.5 1 1 1 ; trotate b2 0.5 -0.25 0 0 0 1 45` | 3.30635 | 6 | 8.69365 | 6 | boxxbox | OBLIQUE |
| Q5 | - | `box b1 0 0 0 1 1 1` | `box b2 0 0.15 0.5 0.8 0.9 1 ; trotate b2 0 0.25 0 0 0 1 -45` | 2.55312 | 6.27929 | 8.28688 | 4.56071 | boxxbox | OBLIQUE |
| Q6 | - | `box b1 0 0 0 1 1 1` | `box b2 0.5 -0.25 0.5 0.5 1 1 ; trotate b2 0.5 -0.25 0 0 0 1 45` | 2.09987 | 6.39645 | 7.90013 | 3.60355 | boxxbox | OBLIQUE |
| Q7 | - | `box b1 1 1 1` | `box b2 -0.25 -0.25 0.25 0.25 1 0.5 ; trotate b2 0 0 0 0 0 1 -30` | 1.17524 | 6.67524 | 6.57476 | 1.07476 | boxxbox | OBLIQUE |
| Q8 | - | `box b1 0 0 0 1 1 1` | `box b2 0 -0.25 0.25 0.5 1.7 0.5 ; trotate b2 0 0 0 0 0 1 -35` | 2.7099 | 7.35615 | 7.1901 | 2.54385 | boxxbox | OBLIQUE |
| Q9 | - | `box b1 3 3 3` | `box b2 1 1 1 ; trotate b2 0 0 0 1 0 0 -30 ; ttranslate b2 1 1 3` | 3.3094 | 55 | 56.6906 | 5 | boxxbox | OBLIQUE |
| R1 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 2 2 2` | 18.8496 | empty | 24 | 30.2832 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| R2 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 2 2 2 ; trotate b2 0 0 0 0 0 1 45` | 18.8496 | empty | 24 | 30.2832 | pcylinderxbox | OBLIQUE |
| R3 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 2 1.5 2` | 16.8965 | 8.88126 | 21.9531 | 19.8587 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| R4 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 -1 -1 0 1+r 2 2 ; trotate b2 0 0 0 0 0 1 60` | 18.574 | 4.27557 | 23.2038 | 25.2982 | pcylinderxbox | OBLIQUE |
| R5 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 1.5 2 2` | 16.8965 | 8.88126 | 21.9531 | 19.8587 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| R6 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 2 2.5 2` | 18.8496 | empty | 28 | 34.2832 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| R7 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 2.5 2 2` | 18.8496 | empty | 28 | 34.2832 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| R8 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 1 2 2` | 13.4248 | 13.4248 | 21.4248 | 15.1416 | pcylinderxbox | axis-on-face@axis0, face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| R9 | - | `pcylinder b1 1 2` | `box b2 0 -1 0 1 2 2` | 13.4248 | 13.4248 | 21.4248 | 15.1416 | pcylinderxbox | axis-on-face@axis0, face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| S1 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 2 1 2` | 13.4248 | 13.4248 | 21.4248 | 15.1416 | pcylinderxbox | axis-on-face@axis1, face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| S2 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -1 -1 0 1+r 1+r 2 ; trotate b2 0 0 0 0 0 1 45` | 17.0816 | 13.0816 | 21.2532 | 14.97 | pcylinderxbox | OBLIQUE |
| S3 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -1 -1 0 1+r 1+r 2 ; trotate b2 0 0 0 0 0 1 -45` | 17.0816 | 13.0816 | 21.2532 | 14.97 | pcylinderxbox | OBLIQUE |
| S4 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -1 -1 0 1+r 1+r 2` | 17.0816 | 13.0816 | 21.2532 | 14.97 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| S5 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 1+r 1+r 2` | 17.0816 | 13.0816 | 21.2532 | 14.97 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| S6 | - | `pcylinder b1 1 2` | `box b2 -0.8 -0.8 0 1.8 1.8 2` | 17.8475 | 10.602 | 21.882 | 17.8692 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| S7 | - | `pcylinder b1 1 2` | `box b2 -1 -0.8 0 1.8 1.8 2` | 17.8475 | 10.602 | 21.882 | 17.8692 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| S8 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 -1 -r 0 1+r 1+r 2 ; trotate b2 0 0 0 0 0 1 30` | 18.2984 | 8.55113 | 22.4434 | 20.349 | pcylinderxbox | OBLIQUE |
| S9 | - | `pcylinder b1 1 2` | `box b2 -1 -0.8 0 2 1.6 2` | 17.8475 | 10.602 | 21.802 | 17.7892 | pcylinderxbox | face-tangent-r@axis0, BOX-x-QUADRIC |
| T1 | - | `pcylinder b1 1 2` | `box b2 -0.8 -1 0 1.6 2 2` | 17.8475 | 10.602 | 21.802 | 17.7892 | pcylinderxbox | face-tangent-r@axis1, BOX-x-QUADRIC |
| T2 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 -r -1 0 2*r 2 2 ; trotate b2 0 0 0 0 0 1 30` | 18.2984 | 8.55113 | 22.4075 | 20.3131 | pcylinderxbox | OBLIQUE |
| T3 | - | `pcylinder b1 1 2` | `box b2 -0.5 -1 0 1.5 1.5 2` | 14.6841 | 15.0937 | 20.6654 | 12.2879 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| T4 | - | `pcylinder b1 1 2` | `box b2 -0.5 -1 0 1.5 1.5 2 ; trotate b2 0 0 0 0 0 1 30` | 14.6841 | 15.0937 | 20.6654 | 12.2879 | pcylinderxbox | OBLIQUE |
| T5 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 1.5 1.5 2` | 14.6841 | 15.0937 | 20.6654 | 12.2879 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| T6 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 1+r 2*r 2` | 16.1977 | 19.6224 | 19.9656 | 7.39922 | pcylinderxbox | face-tangent-r@axis0, BOX-x-QUADRIC |
| T7 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 1+r 2*r 2 ; trotate b2 0 0 0 0 0 1 135` | 16.1977 | 19.6224 | 19.9656 | 7.39922 | pcylinderxbox | OBLIQUE |
| T8 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 -0.5 -r 0 1.5 2*r 2 ; trotate b2 0 0 0 0 0 1 60` | 16.3454 | 17.4324 | 20.6285 | 10.1566 | pcylinderxbox | OBLIQUE |
| T9 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 2*r r+1 2` | 16.1977 | 19.6224 | 19.9656 | 7.39922 | pcylinderxbox | face-tangent-r@axis1, BOX-x-QUADRIC |
| U1 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 -r -1 0 2*r 1+r 2 ; trotate b2 0 0 0 0 0 1 30` | 18.0229 | 12.8267 | 21.6831 | 15.3999 | pcylinderxbox | OBLIQUE |
| U2 | - | `pcylinder b1 1 2` | `box b2 -0.8 -1 0 1.6 1.8 2 ; trotate b2 0 0 0 0 0 1 90` | 17.3465 | 15.903 | 20.863 | 11.7022 | pcylinderxbox | face-tangent-r@axis0, BOX-x-QUADRIC |
| U3 | - | `pcylinder b1 1 2` | `box b2 -0.8 -1 0 1.6 1.8 2` | 17.3465 | 15.903 | 20.863 | 11.7022 | pcylinderxbox | face-tangent-r@axis1, BOX-x-QUADRIC |
| U4 | - | `pcylinder b1 1 2` | `box b2 -1 -1.1 0 1 2.2 2` | 13.4248 | 13.4248 | 22.6248 | 16.3416 | pcylinderxbox | axis-on-face@axis0, face-tangent-r@axis0, BOX-x-QUADRIC |
| U5 | - | `pcylinder b1 1 2` | `box b2 0 -1.1 0 1 2.2 2` | 13.4248 | 13.4248 | 22.6248 | 16.3416 | pcylinderxbox | axis-on-face@axis0, face-tangent-r@axis0, BOX-x-QUADRIC |
| U6 | - | `pcylinder b1 1 2` | `box b2 -1.1 -1 0 2.2 1 2` | 13.4248 | 13.4248 | 22.6248 | 16.3416 | pcylinderxbox | axis-on-face@axis1, face-tangent-r@axis1, BOX-x-QUADRIC |
| U7 | - | `pcylinder b1 1 2` | `box b2 -1 1 0 2 2 2` | empty | 18.8496 | 42.8496 | 24 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC, COMMON-EMPTY |
| U8 | - | `pcylinder b1 1 2` | `box b2 1 -1 0 2 2 2` | empty | 18.8496 | 42.8496 | 24 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC, COMMON-EMPTY |
| U9 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 2*r 2*r 2 ; trotate b2 0 0 0 0 0 1 135` | 15.3137 | 26.1633 | 18.8496 | empty | pcylinderxbox | OBLIQUE |
| V1 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 2*r 2*r 2` | 15.3137 | 26.1633 | 18.8496 | empty | pcylinderxbox | BOX-x-QUADRIC |
| V2 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 1.6 2*r 2 ; trotate b2 0 0 0 0 0 1 135` | 16.0004 | 23.4218 | 19.4315 | 3.12856 | pcylinderxbox | OBLIQUE |
| V3 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 -0.5 -r 0 1.4 2*r 2 ; trotate b2 0 0 0 0 0 1 60` | 16.1674 | 21.0975 | 20.0601 | 5.97995 | pcylinderxbox | OBLIQUE |
| V4 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 1.6 2*r 2` | 16.0004 | 23.4218 | 19.4315 | 3.12856 | pcylinderxbox | BOX-x-QUADRIC |
| V5 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 2 2 1` | 12.5664 | 12.5664 | 22.2832 | 16 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| V6 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 2 2 1 ; trotate b2 0 0 0 0 0 1 45` | 12.5664 | 12.5664 | 22.2832 | 16 | pcylinderxbox | OBLIQUE |
| V7 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 2 1.5 1` | 10.9757 | 16.3928 | 20.8739 | 10.4019 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| V8 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 -1 -1 0 1+r 2 1 ; trotate b2 0 0 0 0 0 1 60` | 12.338 | 14.6136 | 21.7077 | 13.3301 | pcylinderxbox | OBLIQUE |
| V9 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 1.5 2 1` | 10.9757 | 16.3928 | 20.8739 | 10.4019 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| W1 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 2 2.5 1` | 12.5664 | 12.5664 | 25.2832 | 19 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| W2 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 2.5 2 1` | 12.5664 | 12.5664 | 25.2832 | 19 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| W3 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 1 2 1` | 8.28319 | 17.708 | 20.5664 | 8 | pcylinderxbox | axis-on-face@axis0, face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| W4 | - | `pcylinder b1 1 2` | `box b2 0 -1 0 1 2 1` | 8.28319 | 17.708 | 20.5664 | 8 | pcylinderxbox | axis-on-face@axis0, face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| W5 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 2 1 1` | 8.28319 | 17.708 | 20.5664 | 8 | pcylinderxbox | axis-on-face@axis1, face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| W6 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -1 -1 0 1+r 1+r 1 ; trotate b2 0 0 0 0 0 1 45` | 11.1116 | 18.5364 | 20.3948 | 7.82843 | pcylinderxbox | OBLIQUE |
| W7 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -1 -1 0 1+r 1+r 1 ; trotate b2 0 0 0 0 0 1 -45` | 11.1116 | 18.5364 | 20.3948 | 7.82843 | pcylinderxbox | OBLIQUE |
| W8 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -1 -1 0 1+r 1+r 1` | 11.1116 | 18.5364 | 20.3948 | 7.82843 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| W9 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 1+r 1+r 1` | 11.1116 | 18.5364 | 20.3948 | 7.82843 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| X1 | - | `pcylinder b1 1 2` | `box b2 -0.8 -0.8 0 1.8 1.8 1` | 11.7384 | 17.5404 | 20.7912 | 9.36 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| X2 | - | `pcylinder b1 1 2` | `box b2 -1 -0.8 0 1.8 1.8 1` | 11.7384 | 17.5404 | 20.7912 | 9.36 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| X3 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 -1 -r 0 1+r 1+r 1 ; trotate b2 0 0 0 0 0 1 30` | 12.1096 | 16.6608 | 21.1681 | 10.6962 | pcylinderxbox | OBLIQUE |
| X4 | - | `pcylinder b1 1 2` | `box b2 -1 -0.8 0 2 1.6 1` | 11.7384 | 17.5404 | 20.7112 | 9.28 | pcylinderxbox | face-tangent-r@axis0, BOX-x-QUADRIC |
| X5 | - | `pcylinder b1 1 2` | `box b2 -0.8 -1 0 1.6 2 1` | 11.7384 | 17.5404 | 20.7112 | 9.28 | pcylinderxbox | face-tangent-r@axis1, BOX-x-QUADRIC |
| X6 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 -r -1 0 2*r 2 1 ; trotate b2 0 0 0 0 0 1 30` | 12.1096 | 16.6608 | 21.1322 | 10.6603 | pcylinderxbox | OBLIQUE |
| X7 | - | `pcylinder b1 1 2` | `box b2 -0.5 -1 0 1.5 1.5 1` | 9.33406 | 18.9636 | 20.0155 | 6.40192 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| X8 | - | `pcylinder b1 1 2` | `box b2 -0.5 -1 0 1.5 1.5 1 ; trotate b2 0 0 0 0 0 1 30` | 9.33406 | 18.9636 | 20.0155 | 6.40192 | pcylinderxbox | OBLIQUE |
| X9 | - | `pcylinder b1 1 2` | `box b2 -1 -1 0 1.5 1.5 1` | 9.33406 | 18.9636 | 20.0155 | 6.40192 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| Y1 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 1+r 2*r 1` | 10.3842 | 21.5214 | 19.5364 | 3.82843 | pcylinderxbox | face-tangent-r@axis0, BOX-x-QUADRIC |
| Y2 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 1+r 2*r 1 ; trotate b2 0 0 0 0 0 1 135` | 10.3842 | 21.5214 | 19.5364 | 3.82843 | pcylinderxbox | OBLIQUE |
| Y3 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 -0.5 -r 0 1.5 2*r 1 ; trotate b2 0 0 0 0 0 1 60` | 10.5189 | 20.4872 | 19.9909 | 5.33013 | pcylinderxbox | OBLIQUE |
| Y4 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 2*r r+1 1` | 10.3842 | 21.5214 | 19.5364 | 3.82843 | pcylinderxbox | face-tangent-r@axis1, BOX-x-QUADRIC |
| Y5 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 -r -1 0 2*r 1+r 1 ; trotate b2 0 0 0 0 0 1 30` | 11.8813 | 18.708 | 20.6285 | 8.06218 | pcylinderxbox | OBLIQUE |
| Y6 | - | `pcylinder b1 1 2` | `box b2 -0.8 -1 0 1.6 1.8 1 ; trotate b2 0 0 0 0 0 1 90` | 11.3244 | 20.0274 | 20.0852 | 6.08 | pcylinderxbox | face-tangent-r@axis0, BOX-x-QUADRIC |
| Y7 | - | `pcylinder b1 1 2` | `box b2 -0.8 -1 0 1.6 1.8 1` | 11.3244 | 20.0274 | 20.0852 | 6.08 | pcylinderxbox | face-tangent-r@axis1, BOX-x-QUADRIC |
| Y8 | - | `pcylinder b1 1 2` | `box b2 -1 -1.1 0 1 2.2 1` | 8.28319 | 17.708 | 21.3664 | 8.8 | pcylinderxbox | axis-on-face@axis0, face-tangent-r@axis0, BOX-x-QUADRIC |
| Y9 | - | `pcylinder b1 1 2` | `box b2 0 -1.1 0 1 2.2 1` | 8.28319 | 17.708 | 21.3664 | 8.8 | pcylinderxbox | axis-on-face@axis0, face-tangent-r@axis0, BOX-x-QUADRIC |
| Z1 | - | `pcylinder b1 1 2` | `box b2 -1.1 -1 0 2.2 1 1` | 8.28319 | 17.708 | 21.3664 | 8.8 | pcylinderxbox | axis-on-face@axis1, face-tangent-r@axis1, BOX-x-QUADRIC |
| Z2 | - | `pcylinder b1 1 2` | `box b2 -1 1 0 2 2 1` | empty | 18.8496 | 34.8496 | 16 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC, COMMON-EMPTY |
| Z3 | - | `pcylinder b1 1 2` | `box b2 1 -1 0 2 2 1` | empty | 18.8496 | 34.8496 | 16 | pcylinderxbox | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC, COMMON-EMPTY |
| Z4 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 2*r 2*r 1 ; trotate b2 0 0 0 0 0 1 135` | 9.65685 | 24.5064 | 18.8496 | empty | pcylinderxbox | OBLIQUE |
| Z5 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 2*r 2*r 1` | 9.65685 | 24.5064 | 18.8496 | empty | pcylinderxbox | BOX-x-QUADRIC |
| Z6 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 1.6 2*r 1 ; trotate b2 0 0 0 0 0 1 135` | 10.2206 | 23.3561 | 19.1829 | 1.60665 | pcylinderxbox | OBLIQUE |
| Z7 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 -0.5 -r 0 1.4 2*r 1 ; trotate b2 0 0 0 0 0 1 60` | 10.3712 | 22.261 | 19.5922 | 3.12734 | pcylinderxbox | OBLIQUE |
| Z8 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 1.6 2*r 1` | 10.2206 | 23.3561 | 19.1829 | 1.60665 | pcylinderxbox | BOX-x-QUADRIC |
| Z9 | - | `box b1 4 4 4` | `pcylinder b2 2 6 ; ttranslate b2 0 0 -2` | 34.8496 | 86.2832 | 161.681 | 103.965 | boxxpcylinder | axis-on-face@axis0, axis-on-face@axis1, BOX-x-QUADRIC |
| ZA1 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 1.6 1.6 2` | 16.6872 | 20.6804 | 20.0824 | 6.32616 | pcylinderxbox | BOX-x-QUADRIC |
| ZA2 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 -0.5 -r 0 1.45 1.8 2 ; trotate b2 0 0 0 0 0 1 60` | 16.4622 | 18.6723 | 20.6074 | 8.86036 | pcylinderxbox | OBLIQUE |
| ZA3 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 1.6 1.6 2 ; trotate b2 0 0 0 0 0 1 135` | 16.6872 | 20.6804 | 20.0824 | 6.32616 | pcylinderxbox | OBLIQUE |
| ZA4 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 2*r 2*r 2 ; trotate b2 0 0 0 0 0 1 60` | 17.7473 | 17.1023 | 20.9587 | 10.4867 | pcylinderxbox | OBLIQUE |
| ZA5 | - | `pcylinder b1 1 2` | `box b2 -0.8 -0.8 0 1.6 1.6 2` | 16.8455 | 21.204 | 19.924 | 5.61518 | pcylinderxbox | BOX-x-QUADRIC |
| ZA6 | - | `pcylinder b1 1 2` | `box b2 0 -0.5 0 2 1 2` | 9.47172 | 20.306 | 25.3778 | 10.7171 | pcylinderxbox | axis-on-face@axis0, BOX-x-QUADRIC |
| ZA7 | - | `pcylinder b1 1 2` | `box b2 0 -0.5 0 2 1 2 ; trotate b2 0 0 0 0 0 1 30` | 9.47172 | 20.306 | 25.3778 | 10.7171 | pcylinderxbox | OBLIQUE |
| ZA8 | - | `pcylinder b1 1 2` | `box b2 0 0 0 2 2 2` | 8.71239 | 18.1372 | 34.1372 | 21.5708 | pcylinderxbox | axis-on-face@axis0, axis-on-face@axis1, BOX-x-QUADRIC |
| ZA9 | - | `pcylinder b1 1 2` | `box b2 -2 -2 0 2 2 2` | 8.71239 | 18.1372 | 34.1372 | 21.5708 | pcylinderxbox | axis-on-face@axis0, axis-on-face@axis1, BOX-x-QUADRIC |
| ZB1 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 0 0 0 r r 2` | 7.90092 | 21.8768 | 19.3768 | 2.62167 | pcylinderxbox | axis-on-face@axis0, axis-on-face@axis1, BOX-x-QUADRIC |
| ZB2 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 0 0 0 r r 2 ; trotate b2 0 0 0 0 0 1 -30` | 7.90092 | 21.8768 | 19.3768 | 2.62167 | pcylinderxbox | OBLIQUE |
| ZB3 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 1.6 1.6 1` | 10.7843 | 22.2057 | 19.5852 | 3.28233 | pcylinderxbox | BOX-x-QUADRIC |
| ZB4 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 -0.5 -r 0 1.45 1.8 1 ; trotate b2 0 0 0 0 0 1 60` | 10.6153 | 21.1451 | 19.9542 | 4.65595 | pcylinderxbox | OBLIQUE |
| ZB5 | dset r sqrt(2)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 1.6 1.6 1 ; trotate b2 0 0 0 0 0 1 135` | 10.7843 | 22.2057 | 19.5852 | 3.28233 | pcylinderxbox | OBLIQUE |
| ZB6 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 -r -r 0 2*r 2*r 1 ; trotate b2 0 0 0 0 0 1 60` | 11.6529 | 20.7552 | 20.1249 | 5.4641 | pcylinderxbox | OBLIQUE |
| ZB7 | - | `pcylinder b1 1 2` | `box b2 -0.8 -0.8 0 1.6 1.6 1` | 10.9104 | 22.5144 | 19.4592 | 2.88 | pcylinderxbox | BOX-x-QUADRIC |
| ZB8 | - | `pcylinder b1 1 2` | `box b2 0 -0.5 0 2 1 1` | 5.69247 | 20.5344 | 23.1571 | 6.40192 | pcylinderxbox | axis-on-face@axis0, BOX-x-QUADRIC |
| ZB9 | - | `pcylinder b1 1 2` | `box b2 0 -0.5 0 2 1 1 ; trotate b2 0 0 0 0 0 1 30` | 5.69247 | 20.5344 | 23.1571 | 6.40192 | pcylinderxbox | OBLIQUE |
| ZC1 | - | `pcylinder b1 1 2` | `box b2 0 0 0 2 2 1` | 5.14159 | 19.2788 | 29.708 | 14 | pcylinderxbox | axis-on-face@axis0, axis-on-face@axis1, BOX-x-QUADRIC |
| ZC2 | - | `pcylinder b1 1 2` | `box b2 -2 -2 0 2 2 1` | 5.14159 | 19.2788 | 29.708 | 14 | pcylinderxbox | axis-on-face@axis0, axis-on-face@axis1, BOX-x-QUADRIC |
| ZC3 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 0 0 0 r r 1` | 4.64527 | 21.058 | 19.1684 | 1.36603 | pcylinderxbox | axis-on-face@axis0, axis-on-face@axis1, BOX-x-QUADRIC |
| ZC4 | dset r sqrt(3)/2 | `pcylinder b1 1 2` | `box b2 0 0 0 r r 1 ; trotate b2 0 0 0 0 0 1 -30` | 4.64527 | 21.058 | 19.1684 | 1.36603 | pcylinderxbox | OBLIQUE |
| ZC5 | - | `pcylinder b1 1 2` | `box b2 0 0 -0.2 1.4 1.4 1.4 ; trotate b2 0 0 -0.2 -1 1 0 -45` | 10.8874 | 27.1316 | 19.7221 | 3.47797 | pcylinderxbox | OBLIQUE |
| ZC6 | - | `pcylinder b1 1 2` | `box b2 -1 -2 0.5 2 2 1` | 8.28319 | 20.8496 | 26.5664 | 14 | pcylinderxbox | axis-on-face@axis1, face-tangent-r@axis0, BOX-x-QUADRIC |
| ZC7 | - | `box b1 -1 0 1 2 2 2` | `pcylinder b2 1 4` | 13.4248 | 23.1416 | 41.9911 | 32.2743 | boxxpcylinder | axis-on-face@axis1, face-tangent-r@axis0, BOX-x-QUADRIC |
| ZC8 | - | `pcylinder b1 1 2` | `pcylinder b2 0.5 1 ; ttranslate b2 0 0 2` | empty | 18.8496 | 21.9911 | 4.71239 | pcylinderxpcylinder | PARALLEL-AXES, COAXIAL, COMMON-EMPTY |
| ZC9 | - | `pcylinder b1 1 2` | `pcylinder b2 0.5 1 ; ttranslate b2 0 0 1` | 4.71239 | 21.9911 | 18.8496 | empty | pcylinderxpcylinder | PARALLEL-AXES, COAXIAL |
| ZD1 | - | `pcylinder b1 1 2` | `pcylinder b2 0.5 1 ; ttranslate b2 0 0 0.5` | 4.71239 | 23.5619 | 18.8496 | empty | pcylinderxpcylinder | PARALLEL-AXES, COAXIAL |
| ZD2 | - | `pcylinder b1 1 2` | `pcylinder b2 0.5 2` | 7.85398 | 23.5619 | 18.8496 | empty | pcylinderxpcylinder | PARALLEL-AXES, COAXIAL |
| ZD3 | - | `pcylinder b1 1 2` | `pcylinder b2 0.5 3 ; ttranslate b2 0 0 -1` | 7.85398 | 23.5619 | 21.9911 | 4.71239 | pcylinderxpcylinder | PARALLEL-AXES, COAXIAL |
| ZD4 | - | `pcylinder b1 1 2` | `pcylinder b2 1 2 ; ttranslate b2 0 0 2` | empty | 18.8496 | 31.4159 | 18.8496 | pcylinderxpcylinder | PARALLEL-AXES, COAXIAL, equal-radius(r=r=1), COMMON-EMPTY |
| ZD5 | - | `pcylinder b1 1 2` | `pcylinder b2 1 2 ; ttranslate b2 0 0 2 ; trotate b2 0 0 0 0 0 1 90` | empty | 18.8496 | 31.4159 | 18.8496 | pcylinderxpcylinder | PARALLEL-AXES, COAXIAL, equal-radius(r=r=1), COMMON-EMPTY |
| ZD6 | - | `pcylinder b1 1 2` | `pcylinder b2 1 2 ; ttranslate b2 0 0 2 ; trotate b2 0 0 0 0 0 1 180` | empty | 18.8496 | 31.4159 | 18.8496 | pcylinderxpcylinder | PARALLEL-AXES, COAXIAL, equal-radius(r=r=1), COMMON-EMPTY |
| ZD7 | - | `pcylinder b1 1 2` | `pcylinder b2 1 2 ; ttranslate b2 0 0 2 ; trotate b2 0 0 0 0 0 1 270` | empty | 18.8496 | 31.4159 | 18.8496 | pcylinderxpcylinder | PARALLEL-AXES, COAXIAL, equal-radius(r=r=1), COMMON-EMPTY |
| ZD8 | - | `pcylinder b1 1 4` | `pcylinder b2 1 4 ; trotate b2 0 0 2 1 0 0 90` | 16 | 31.4159 | 46.8319 | 31.4159 | pcylinderxpcylinder | PERPENDICULAR-AXES |
| ZD9 | - | `pcylinder b1 1 4` | `pcylinder b2 1 4 ; trotate b2 0 0 2 1 0 0 90 ; trotate b2 0 0 2 0 1 0 60` | 16 | 31.4159 | 46.8319 | 31.4159 | pcylinderxpcylinder | OBLIQUE |
| ZE1 | - | `pcylinder b1 1 4` | `pcylinder b2 1 4 ; trotate b2 0 0 2 1 0 0 90 ; trotate b2 0 0 2 0 1 0 180` | 16 | 31.4159 | 46.8319 | 31.4159 | pcylinderxpcylinder | PERPENDICULAR-AXES |
| ZE2 | - | `pcylinder b1 1 4` | `pcylinder b2 1 4 ; trotate b2 0 0 2 1 0 0 90 ; trotate b2 0 0 2 0 1 0 60 ; trotate b2 0 0 2 0 0 1 120` | 16 | 31.4159 | 46.8319 | 31.4159 | pcylinderxpcylinder | OBLIQUE |
| ZE3 | - | `pcylinder b1 1 4` | `pcylinder b2 0.5 4 ; trotate b2 0 0 2 1 0 0 90` | 7.49478 | 35.6608 | 38.0586 | 9.89259 | pcylinderxpcylinder | PERPENDICULAR-AXES |
| ZE4 | - | `pcylinder b1 1 4` | `pcylinder b2 0.5 4 ; trotate b2 0 0 2 1 0 0 90 ; trotate b2 0 0 2 0 1 0 90` | 7.49533 | 35.6613 | 38.0584 | 9.89236 | pcylinderxpcylinder | PERPENDICULAR-AXES |
| ZE5 | - | `pcylinder b1 1 4` | `pcylinder b2 0.5 4 ; trotate b2 0 0 2 1 0 0 90 ; trotate b2 0 0 2 0 1 0 180` | 7.49478 | 35.6602 | 38.058 | 9.89259 | pcylinderxpcylinder | PERPENDICULAR-AXES |
| ZE6 | - | `pcylinder b1 1 4` | `pcylinder b2 0.5 4 ; trotate b2 0 0 2 1 0 0 90 ; trotate b2 0 0 2 0 1 0 270` | 7.49533 | 35.6601 | 38.0572 | 9.89236 | pcylinderxpcylinder | PERPENDICULAR-AXES |
| ZE7 | - | `pcylinder b1 1 4` | `pcylinder b2 0.5 4 ; trotate b2 0 0 2 1 0 0 90 ; ttranslate b2 0.5 0 0` | 6.72264 | 33.8759 | 38.8306 | 11.6773 | pcylinderxpcylinder | PERPENDICULAR-AXES |
| ZE8 | - | `pcylinder b1 1 4` | `pcylinder b2 0.5 4 ; trotate b2 0 0 2 1 0 0 90 ; trotate b2 0 0 2 0 1 0 90 ; ttranslate b2 0.5 0 0` | 6.72254 | 33.8756 | 38.8304 | 11.6773 | pcylinderxpcylinder | PERPENDICULAR-AXES |
| ZE9 | - | `pcylinder b1 1 4` | `pcylinder b2 0.5 4 ; trotate b2 0 0 2 1 0 0 90 ; trotate b2 0 0 2 0 1 0 180 ; ttranslate b2 0.5 0 0` | 6.72239 | 33.8757 | 38.8309 | 11.6776 | pcylinderxpcylinder | PERPENDICULAR-AXES |
| ZF1 | - | `pcylinder b1 1 4` | `pcylinder b2 0.5 4 ; trotate b2 0 0 2 1 0 0 90 ; trotate b2 0 0 2 0 1 0 270 ; ttranslate b2 0.5 0 0` | 6.72254 | 33.8758 | 38.8307 | 11.6774 | pcylinderxpcylinder | PERPENDICULAR-AXES |
| ZF2 | - | `pcylinder b1 1 4` | `pcylinder b2 1 4 ; ttranslate b2 1 0 0 ; trotate b2 1 0 0 0 0 1 -120 ; trotate b2 0 0 0 0 0 1 60` | 19.2119 | 28.9592 | 43.62 | 28.9592 | pcylinderxpcylinder | OBLIQUE |
| ZF3 | - | `pcylinder b1 1 4` | `pcylinder b2 1 4 ; ttranslate b2 1 0 0 ; trotate b2 1 0 0 0 0 1 120 ; trotate b2 0 0 0 0 0 1 60` | 19.2119 | 28.9592 | 43.62 | 28.9592 | pcylinderxpcylinder | OBLIQUE |
| ZF4 | - | `pcylinder b1 1 4` | `pcylinder b2 1 4 ; ttranslate b2 1 0 0` | 19.2119 | 28.9592 | 43.62 | 28.9592 | pcylinderxpcylinder | PARALLEL-AXES, equal-radius(r=r=1) |
| ZF5 | - | `box b1 4 4 4` | `pcone b2 1 0.5 2 ; ttranslate b2 2 2 4` | empty | 96 | 103.359 | 13.6418 | boxxpcone | BOX-x-QUADRIC, COMMON-EMPTY |
| ZF6 | - | `box b1 4 4 4` | `pcone b2 1 0.5 2 ; ttranslate b2 2 2 2` | 13.6418 | 108.071 | 96 | empty | boxxpcone | BOX-x-QUADRIC |
| ZF7 | - | `box b1 4 4 4` | `pcone b2 1 0.5 4 ; ttranslate b2 2 2 0` | 22.9232 | 111.069 | 96 | empty | boxxpcone | BOX-x-QUADRIC |
| ZF8 | - | `box b1 4 4 4` | `pcone b2 1 0.5 5 ; ttranslate b2 2 2 -1` | 21.0108 | 110.351 | 102.596 | 11.6851 | boxxpcone | BOX-x-QUADRIC |
| ZF9 | - | `box b1 4 4 4` | `pcone b2 1 0.5 2 ; ttranslate b2 2 2 1` | 13.6418 | 109.642 | 96 | empty | boxxpcone | BOX-x-QUADRIC |
| ZG1 | - | `box b1 4 4 4` | `pcone b2 3 2 4 ; ttranslate b2 4 2 0` | 64.3685 | 70.8546 | 137.238 | 96.1059 | boxxpcone | axis-on-face@axis0, BOX-x-QUADRIC |
| ZG2 | - | `box b1 4 4 4` | `pcone b2 3 2 4 ; ttranslate b2 2 0 0` | 64.3685 | 70.8546 | 137.238 | 96.1059 | boxxpcone | axis-on-face@axis1, BOX-x-QUADRIC |
| ZG3 | - | `box b1 4 4 4` | `pcone b2 3 2 4 ; ttranslate b2 0 2 0` | 64.3685 | 70.8546 | 137.238 | 96.1059 | boxxpcone | axis-on-face@axis0, BOX-x-QUADRIC |
| ZG4 | - | `box b1 4 4 4` | `pcone b2 3 2 4 ; ttranslate b2 2 0 0 ; trotate b2 2 0 0 0 0 1 30` | 64.3685 | 70.8546 | 137.238 | 96.1059 | boxxpcone | OBLIQUE |
| ZG5 | - | `box b1 4 4 4` | `pcone b2 2 1 4 ; ttranslate b2 4 2 0` | 39.2837 | 95.5757 | 111.284 | 39.2837 | boxxpcone | axis-on-face@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| ZG6 | - | `box b1 4 4 4` | `pcone b2 2 1 4 ; ttranslate b2 2 0 0` | 39.2837 | 95.5757 | 111.284 | 39.2837 | boxxpcone | axis-on-face@axis1, face-tangent-r@axis0, BOX-x-QUADRIC |
| ZG7 | - | `box b1 4 4 4` | `pcone b2 2 1 4 ; ttranslate b2 0 2 0` | 39.2837 | 95.5757 | 111.284 | 39.2837 | boxxpcone | axis-on-face@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| ZG8 | - | `box b1 4 4 4` | `pcone b2 1 0.5 2 ; ttranslate b2 3 1 4` | empty | 96 | 103.359 | 13.6418 | boxxpcone | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC, COMMON-EMPTY |
| ZG9 | - | `box b1 4 4 4` | `pcone b2 1 0.5 2 ; ttranslate b2 1 1 4` | empty | 96 | 103.359 | 13.6418 | boxxpcone | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC, COMMON-EMPTY |
| ZH1 | - | `box b1 4 4 4` | `pcone b2 5 4 4 ; ttranslate b2 6 2 0` | 68.087 | 62.1124 | 273.296 | 241.753 | boxxpcone | BOX-x-QUADRIC |
| ZH2 | - | `box b1 4 4 4` | `pcone b2 5 4 4 ; ttranslate b2 -2 2 0` | 68.087 | 62.1124 | 273.296 | 241.753 | boxxpcone | BOX-x-QUADRIC |
| ZH3 | - | `box b1 4 4 4` | `pcone b2 5 4 4 ; ttranslate b2 -2 2 0 ; trotate b2 -2 2 0 0 0 1 30` | 68.087 | 62.1124 | 273.296 | 241.753 | boxxpcone | OBLIQUE |
| ZH4 | - | `box b1 4 4 4` | `pcone b2 5 3.5 4 ; ttranslate b2 -2 2 0 ; trotate b2 -2 2 0 0 0 1 30` | 64.4112 | 67.2316 | 262.691 | 226.573 | boxxpcone | OBLIQUE |
| ZH5 | - | `pcylinder b1 4 8` | `psphere b2 4 ; ttranslate b2 0 0 8` | 150.796 | 351.858 | 351.858 | 150.796 | pcylinderxpsphere | PARALLEL-AXES, COAXIAL, equal-radius(r=r=4) |
| ZH6 | - | `pcylinder b1 4 8` | `psphere b2 4 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 0 0 1 90` | 150.796 | 351.858 | 351.858 | 150.796 | pcylinderxpsphere | PARALLEL-AXES, COAXIAL, equal-radius(r=r=4) |
| ZH7 | - | `pcylinder b1 4 8` | `psphere b2 4 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 0 0 1 180` | 150.796 | 351.858 | 351.858 | 150.796 | pcylinderxpsphere | PARALLEL-AXES, COAXIAL, equal-radius(r=r=4) |
| ZH8 | - | `pcylinder b1 4 8` | `psphere b2 4 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 0 0 1 270` | 150.796 | 351.858 | 351.858 | 150.796 | pcylinderxpsphere | PARALLEL-AXES, COAXIAL, equal-radius(r=r=4) |
| ZH9 | - | `pcylinder b1 4 8` | `psphere b2 4 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 1 0 0 90` | 150.796 | 351.858 | 351.858 | 150.796 | pcylinderxpsphere | PERPENDICULAR-AXES |
| ZI1 | - | `pcylinder b1 4 8` | `psphere b2 4 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 1 0 0 90 ; trotate b2 0 0 8 0 0 1 90` | 150.796 | 351.858 | 351.858 | 150.796 | pcylinderxpsphere | PERPENDICULAR-AXES |
| ZI2 | - | `pcylinder b1 4 8` | `psphere b2 4 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 1 0 0 90 ; trotate b2 0 0 8 0 0 1 180` | 150.796 | 351.858 | 351.858 | 150.796 | pcylinderxpsphere | PERPENDICULAR-AXES |
| ZI3 | - | `pcylinder b1 4 8` | `psphere b2 4 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 1 0 0 90 ; trotate b2 0 0 8 0 0 1 270` | 150.796 | 351.858 | 351.858 | 150.796 | pcylinderxpsphere | PERPENDICULAR-AXES |
| ZI4 | - | `pcylinder b1 4 8` | `psphere b2 2 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 1 0 0 90` | 37.6989 | 314.159 | 314.159 | 37.6989 | pcylinderxpsphere | PERPENDICULAR-AXES |
| ZI5 | - | `pcylinder b1 4 8` | `psphere b2 2 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 1 0 0 90 ; trotate b2 0 0 8 0 1 0 90` | 37.6989 | 314.159 | 314.159 | 37.6989 | pcylinderxpsphere | PERPENDICULAR-AXES |
| ZI6 | - | `pcylinder b1 4 8` | `psphere b2 2 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 1 0 0 90 ; trotate b2 0 0 8 0 1 0 180` | 37.6989 | 314.159 | 314.159 | 37.6989 | pcylinderxpsphere | PERPENDICULAR-AXES |
| ZI7 | - | `pcylinder b1 4 8` | `psphere b2 2 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 1 0 0 90 ; trotate b2 0 0 8 0 1 0 270` | 37.6989 | 314.159 | 314.159 | 37.6989 | pcylinderxpsphere | PERPENDICULAR-AXES |
| ZI8 | - | `pcylinder b1 4 8` | `psphere b2 6 ; ttranslate b2 0 0 8` | 220.262 | 196.529 | 533.72 | 557.452 | pcylinderxpsphere | PARALLEL-AXES, COAXIAL |
| ZI9 | - | `pcylinder b1 4 8` | `psphere b2 6 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 0 0 1 90` | 220.262 | 196.529 | 533.72 | 557.452 | pcylinderxpsphere | PARALLEL-AXES, COAXIAL |
| ZJ1 | - | `pcylinder b1 4 8` | `psphere b2 6 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 0 0 1 180` | 220.262 | 196.529 | 533.72 | 557.452 | pcylinderxpsphere | PARALLEL-AXES, COAXIAL |
| ZJ2 | - | `pcylinder b1 4 8` | `psphere b2 6 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 0 0 1 270` | 220.262 | 196.529 | 533.72 | 557.452 | pcylinderxpsphere | PARALLEL-AXES, COAXIAL |
| ZJ3 | - | `pcylinder b1 4 8` | `psphere b2 6 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 0 1 0 90` | 220.262 | 196.529 | 533.721 | 557.453 | pcylinderxpsphere | PERPENDICULAR-AXES |
| ZJ4 | - | `pcylinder b1 4 8` | `pcone b2 2 1 4 ; ttranslate b2 0 0 8` | empty | 301.593 | 331.027 | 54.5673 | pcylinderxpcone | PARALLEL-AXES, COAXIAL, COMMON-EMPTY |
| ZJ5 | - | `pcylinder b1 4 8` | `pcone b2 1 2 4 ; ttranslate b2 0 0 4` | 54.5673 | 331.027 | 301.593 | empty | pcylinderxpcone | PARALLEL-AXES, COAXIAL |
| ZJ6 | - | `pcylinder b1 4 8` | `pcone b2 2 1 8` | 91.693 | 361.87 | 301.593 | empty | pcylinderxpcone | PARALLEL-AXES, COAXIAL |
| ZJ7 | - | `pcylinder b1 4 8` | `pcone b2 2 1 9 ; ttranslate b2 0 0 -1` | 87.403 | 360.295 | 315.243 | 36.0678 | pcylinderxpcone | PARALLEL-AXES, COAXIAL |
| ZJ8 | - | `pcylinder b1 4 8` | `pcone b2 2 1 4 ; ttranslate b2 0 0 2` | 54.5673 | 356.16 | 301.593 | empty | pcylinderxpcone | PARALLEL-AXES, COAXIAL |
| ZJ9 | - | `pcylinder b1 4 8` | `pcone b2 2 1 10 ; ttranslate b2 0 0 -1` | 90.9168 | 362.225 | 321.102 | 49.794 | pcylinderxpcone | PARALLEL-AXES, COAXIAL |
| ZK1 | - | `pcylinder b1 4 8` | `pcone b2 4 2 4 ; ttranslate b2 0 0 8` | empty | 301.593 | 348.192 | 147.13 | pcylinderxpcone | PARALLEL-AXES, COAXIAL, equal-radius(r=r1=4), COMMON-EMPTY |
| ZK2 | - | `pcylinder b1 4 8` | `pcone b2 4 2 4 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 0 0 1 90` | empty | 301.593 | 348.192 | 147.13 | pcylinderxpcone | PARALLEL-AXES, COAXIAL, equal-radius(r=r1=4), COMMON-EMPTY |
| ZK3 | - | `pcylinder b1 4 8` | `pcone b2 4 2 4 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 0 0 1 180` | empty | 301.593 | 348.192 | 147.13 | pcylinderxpcone | PARALLEL-AXES, COAXIAL, equal-radius(r=r1=4), COMMON-EMPTY |
| ZK4 | - | `pcylinder b1 4 8` | `pcone b2 4 2 4 ; ttranslate b2 0 0 8 ; trotate b2 0 0 8 0 0 1 270` | empty | 301.593 | 348.192 | 147.13 | pcylinderxpcone | PARALLEL-AXES, COAXIAL, equal-radius(r=r1=4), COMMON-EMPTY |
| ZK5 | - | `pcylinder b1 4 8` | `pcone b2 2 1 10 ; ttranslate b2 0 0 -1 ; trotate b2 0 0 4 0 1 0 90` | 87.8407 | 358.727 | 324.106 | 53.2199 | pcylinderxpcone | PERPENDICULAR-AXES |
| ZK6 | - | `pcylinder b1 4 8` | `pcone b2 2 1 10 ; ttranslate b2 0 0 -1 ; trotate b2 0 0 4 1 0 0 90` | 87.8112 | 358.826 | 324.207 | 53.1921 | pcylinderxpcone | PERPENDICULAR-AXES |
| ZK7 | - | `pcylinder b1 4 8` | `pcone b2 6 1 10 ; ttranslate b2 0 0 -1 ; trotate b2 0 0 4 0 1 0 90` | 216.554 | 308.057 | 447.132 | 355.629 | pcylinderxpcone | PERPENDICULAR-AXES |
| ZK8 | - | `pcylinder b1 4 8` | `pcone b2 6 1 10 ; ttranslate b2 0 0 -1 ; trotate b2 0 0 4 1 0 0 90` | 216.563 | 308.068 | 447.125 | 355.619 | pcylinderxpcone | PERPENDICULAR-AXES |
| ZK9 | - | `pcylinder b1 4 8` | `pcone b2 1 6 8 ; ttranslate b2 0 0 -1 ; trotate b2 0 0 4 0 1 0 90` | 228.558 | 355.69 | 396.65 | 269.518 | pcylinderxpcone | PERPENDICULAR-AXES |
| ZL1 | - | `pcylinder b1 4 8` | `pcone b2 1 6 8 ; ttranslate b2 0 0 -1 ; trotate b2 0 0 4 0 1 0 -90` | 228.47 | 355.82 | 396.823 | 269.473 | pcylinderxpcone | PERPENDICULAR-AXES |
| ZL2 | - | `pcylinder b1 4 8` | `ptorus b2 4 1 ; ttranslate b2 0 0 4` | 116.656 | 317.718 | 342.851 | 141.789 | pcylinderxptorus | PARALLEL-AXES, COAXIAL, equal-radius(r=r1=4) |
| ZL3 | - | `pcylinder b1 4 8` | `ptorus b2 4 1 ; ttranslate b2 0 0 4 ; trotate b2 0 0 4 0 0 1 90` | 116.656 | 317.718 | 342.851 | 141.789 | pcylinderxptorus | PARALLEL-AXES, COAXIAL, equal-radius(r=r1=4) |
| ZL4 | - | `pcylinder b1 4 8` | `ptorus b2 4 1 ; ttranslate b2 0 0 4 ; trotate b2 0 0 4 0 0 1 180` | 116.656 | 317.718 | 342.851 | 141.789 | pcylinderxptorus | PARALLEL-AXES, COAXIAL, equal-radius(r=r1=4) |
| ZL5 | - | `pcylinder b1 4 8` | `ptorus b2 4 1 ; ttranslate b2 0 0 4 ; trotate b2 0 0 4 0 0 1 270` | 116.656 | 317.718 | 342.851 | 141.789 | pcylinderxptorus | PARALLEL-AXES, COAXIAL, equal-radius(r=r1=4) |
| ZL6 | - | `pcone b1 8 4 8` | `pcone b2 4 2 4 ; ttranslate b2 0 0 8` | empty | 588.519 | 635.117 | 147.13 | pconexpcone | PARALLEL-AXES, COAXIAL, equal-radius(r2=r1=4), COMMON-EMPTY |
| ZL7 | - | `pcone b1 8 4 8` | `pcone b2 4 2 4 ; ttranslate b2 0 0 8 ; trotate b2 0 0 2 0 0 1 90` | empty | 588.519 | 635.117 | 147.13 | pconexpcone | PARALLEL-AXES, COAXIAL, equal-radius(r2=r1=4), COMMON-EMPTY |
| ZL8 | - | `pcone b1 8 4 8` | `pcone b2 4 2 4 ; ttranslate b2 0 0 8 ; trotate b2 0 0 2 0 0 1 180` | empty | 588.519 | 635.117 | 147.13 | pconexpcone | PARALLEL-AXES, COAXIAL, equal-radius(r2=r1=4), COMMON-EMPTY |
| ZL9 | - | `pcone b1 8 4 8` | `pcone b2 4 2 4 ; ttranslate b2 0 0 8 ; trotate b2 0 0 2 0 0 1 270` | empty | 588.519 | 635.117 | 147.13 | pconexpcone | PARALLEL-AXES, COAXIAL, equal-radius(r2=r1=4), COMMON-EMPTY |
| ZM1 | - | `pcone b1 8 4 8` | `pcone b2 2 1 2 ; ttranslate b2 0 0 8` | empty | 588.519 | 600.168 | 36.7824 | pconexpcone | PARALLEL-AXES, COAXIAL, COMMON-EMPTY |
| ZM2 | - | `pcone b1 8 4 8` | `pcone b2 2 1 2 ; ttranslate b2 0 0 6` | 36.7824 | 619.018 | 588.519 | empty | pconexpcone | PARALLEL-AXES, COAXIAL |
| ZM3 | - | `pcone b1 8 4 8` | `pcone b2 2 1 8` | 91.693 | 648.796 | 588.519 | empty | pconexpcone | PARALLEL-AXES, COAXIAL |
| ZM4 | - | `pcone b1 8 4 8` | `pcone b2 2 1 9 ; ttranslate b2 0 0 -1` | 87.403 | 647.221 | 602.168 | 36.0678 | pconexpcone | PARALLEL-AXES, COAXIAL |
| ZM5 | - | `pcone b1 8 4 8` | `pcone b2 2 1 4 ; ttranslate b2 0 0 2` | 54.5673 | 643.086 | 588.519 | empty | pconexpcone | PARALLEL-AXES, COAXIAL |
| ZM6 | - | `pcone b1 8 4 8` | `pcone b2 2 1 10 ; ttranslate b2 0 0 -1` | 90.9168 | 649.15 | 608.028 | 49.794 | pconexpcone | PARALLEL-AXES, COAXIAL |
| ZM7 | - | `pcone b1 8 4 8` | `pcone b2 4 8 4 ; ttranslate b2 0 0 -4` | empty | 588.519 | 650.98 | 464.586 | pconexpcone | PARALLEL-AXES, COAXIAL, equal-radius(r1=r2=8), equal-radius(r2=r1=4), COMMON-EMPTY |
| ZM8 | - | `pcone b1 8 4 8` | `pcone b2 4 8 4 ; ttranslate b2 0 0 -4 ; trotate b2 0 0 -4 0 0 1 90` | empty | 588.519 | 650.98 | 464.586 | pconexpcone | PARALLEL-AXES, COAXIAL, equal-radius(r1=r2=8), equal-radius(r2=r1=4), COMMON-EMPTY |
| ZM9 | - | `pcone b1 8 4 8` | `pcone b2 4 8 4 ; ttranslate b2 0 0 -4 ; trotate b2 0 0 -4 0 0 1 180` | empty | 588.519 | 650.98 | 464.586 | pconexpcone | PARALLEL-AXES, COAXIAL, equal-radius(r1=r2=8), equal-radius(r2=r1=4), COMMON-EMPTY |
| ZN1 | - | `pcone b1 8 4 8` | `pcone b2 4 8 4 ; ttranslate b2 0 0 -4 ; trotate b2 0 0 -4 0 0 1 270` | empty | 588.519 | 650.98 | 464.586 | pconexpcone | PARALLEL-AXES, COAXIAL, equal-radius(r1=r2=8), equal-radius(r2=r1=4), COMMON-EMPTY |
| ZN2 | - | `pcone b1 8 4 8` | `pcone b2 8 4 4 ; ttranslate b2 4 0 0` | 340.979 | 523.976 | 712.125 | 253.658 | pconexpcone | PARALLEL-AXES, equal-radius(r1=r1=8), equal-radius(r2=r2=4) |
| ZN3 | - | `box b1 4 4 4` | `pcylinder b2 1 6 ; ttranslate b2 2 2 -2` | 31.4159 | 114.85 | 108.566 | 18.8496 | boxxpcylinder | BOX-x-QUADRIC |
| ZN4 | - | `box b1 4 4 4` | `pcylinder b2 1 2 ; ttranslate b2 2 2 1` | 18.8496 | 114.85 | 96 | empty | boxxpcylinder | BOX-x-QUADRIC |
| ZN5 | - | `box b1 4 4 4` | `pcylinder b2 2 4 ; ttranslate b2 4 2 0` | 53.6991 | 92.5664 | 117.699 | 53.6991 | boxxpcylinder | axis-on-face@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| ZN6 | - | `box b1 4 4 4` | `pcylinder b2 2 4 ; ttranslate b2 2 0 0` | 53.6991 | 92.5664 | 117.699 | 53.6991 | boxxpcylinder | axis-on-face@axis1, face-tangent-r@axis0, BOX-x-QUADRIC |
| ZN7 | - | `box b1 4 4 4` | `pcylinder b2 2 4 ; ttranslate b2 0 2 0` | 53.6991 | 92.5664 | 117.699 | 53.6991 | boxxpcylinder | axis-on-face@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| ZN8 | - | `box b1 4 4 4` | `pcylinder b2 1 4 ; ttranslate b2 4 2 0` | 23.708 | 97.4248 | 103.708 | 23.708 | boxxpcylinder | axis-on-face@axis0, BOX-x-QUADRIC |
| ZN9 | - | `box b1 4 4 4` | `pcylinder b2 1 4 ; ttranslate b2 2 0 0` | 23.708 | 97.4248 | 103.708 | 23.708 | boxxpcylinder | axis-on-face@axis1, BOX-x-QUADRIC |
| ZO1 | - | `box b1 4 4 4` | `pcylinder b2 1 4 ; ttranslate b2 0 2 0` | 23.708 | 97.4248 | 103.708 | 23.708 | boxxpcylinder | axis-on-face@axis0, BOX-x-QUADRIC |
| ZO2 | - | `box b1 4 4 4` | `pcylinder b2 1 4 ; ttranslate b2 4 3 0` | 23.708 | 97.4248 | 103.708 | 23.708 | boxxpcylinder | axis-on-face@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| ZO3 | - | `box b1 4 4 4` | `pcylinder b2 1 4 ; ttranslate b2 3 0 0` | 23.708 | 97.4248 | 103.708 | 23.708 | boxxpcylinder | axis-on-face@axis1, face-tangent-r@axis0, BOX-x-QUADRIC |
| ZO4 | - | `box b1 4 4 4` | `pcylinder b2 1 4 ; ttranslate b2 0 3 0` | 23.708 | 97.4248 | 103.708 | 23.708 | boxxpcylinder | axis-on-face@axis0, face-tangent-r@axis1, BOX-x-QUADRIC |
| ZO5 | - | `box b1 4 4 4` | `pcylinder b2 1 4 ; ttranslate b2 1 0 0` | 23.708 | 97.4248 | 103.708 | 23.708 | boxxpcylinder | axis-on-face@axis1, face-tangent-r@axis0, BOX-x-QUADRIC |
| ZO6 | - | `box b1 4 4 4` | `pcylinder b2 4 4 ; ttranslate b2 -2 2 0` | 59.0795 | 70.4308 | 237.982 | 197.408 | boxxpcylinder | BOX-x-QUADRIC |
| ZO7 | - | `box b1 4 4 4` | `pcylinder b2 4 4 ; ttranslate b2 -2 2 0 ; trotate b2 -2 2 0 0 0 1 30` | 59.0795 | 70.4308 | 237.982 | 197.408 | boxxpcylinder | OBLIQUE |
| ZO8 | - | `box b1 4 4 4` | `pcylinder b2 4 4 ; ttranslate b2 -2 2 0 ; trotate b2 -2 2 0 0 0 1 90` | 59.0795 | 70.4308 | 237.982 | 197.408 | boxxpcylinder | BOX-x-QUADRIC |
| ZO9 | - | `box b1 4 4 4` | `pcylinder b2 1 4 ; ttranslate b2 3 1 4` | empty | 96 | 121.133 | 31.4159 | boxxpcylinder | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC, COMMON-EMPTY |
| ZP1 | - | `box b1 4 4 4` | `pcylinder b2 1 4 ; ttranslate b2 1 1 4` | empty | 96 | 121.133 | 31.4159 | boxxpcylinder | face-tangent-r@axis0, face-tangent-r@axis1, BOX-x-QUADRIC, COMMON-EMPTY |
| ZP2 | - | `box b1 4 4 4` | `pcylinder b2 2 6 ; ttranslate b2 0 0 -2` | 34.8496 | 86.2832 | 161.681 | 103.965 | boxxpcylinder | axis-on-face@axis0, axis-on-face@axis1, BOX-x-QUADRIC |
### 2.2 bop*_simple TAIL — the 21 cases outside the shared 371

These case names exist in some but not all four grids and are **not** primitive-only (they use
`halfspace`, `prism`, `revol`, `explode`, `emptycopy`, `plane`, `tcopy`). They are still fully
self-contained (no external file). Listed complete, with the exact construction and the exact
assertion, so they can be ported if/when the missing constructor exists.

| case id | construction (exact DRAW lines, `\|`-separated, comments/puts stripped) | operation | expected assertion | portable? |
|---|---|---|---|---|
| bopcommon_simple/ZP3 | `vertex v1 250 250 0` \| `vertex v2 -250 250 0` \| `vertex v3 -250 -250 0` \| `vertex v4 250 -250 0` \| `edge e1 v1 v2` \| `edge e2 v2 v3` \| `edge e3 v3 v4` \| `edge e4 v4 v1` \| `wire w1 e1 e2 e3 e4` \| `mkplane f w1` \| `halfspace hs f 0 0 -100` \| `box b 0 0 -30 150 200 200` | `bop b hs; bopcommon` | `checkprops result -s 81000` | needs halfspace |
| bopcommon_simple/ZP4 | same as ZP3 but `halfspace hs f 0 0 100` (Zpoint=+100), `box b 0 0 -30 150 200 200` | `bop b hs; bopcommon` | `checkprops result -s 179000` | needs halfspace |
| bopcommon_simple/ZP5 | same as ZP3 but `halfspace hs f 0 0 -100`, `box b 0 0 -80 150 200 200` | `bop b hs; bopcommon` | `checkprops result -s 116000` | needs halfspace |
| bopcommon_simple/ZP6 | same as ZP3 but `halfspace hs f 0 0 100`, `box b 0 0 -80 150 200 200` | `bop b hs; bopcommon` | `checkprops result -s 144000` | needs halfspace |
| bopcommon_simple/ZP7 | `pcone pc 10 0 20` \| `explode pc f` \| `prism pcy pc_2 0 0 10` | `bop pc pcy; bopcommon` | `checkprops result -s 919.56` | needs explode+prism |
| bopcommon_simple/ZP8 | `box b 10 10 10` \| `box c 4 4 4` \| `ttranslate c 2 2 2` \| `bop b c` \| `bopcut r` \| `explode r sh` \| `orientation r_2 R` \| `emptycopy b1 b` \| `add r_2 b1` \| `explode r so` | `bop r_1 b1; bopcommon` | `checkprops result -s empty` | needs shell surgery (cavity solid) |
| bopcommon_simple/ZP9 | `box bb 100 100 100` \| `plane pl1 98.946735014, 46.491265177, 17.092869659` \| `psphere s1 pl1 7.5` | `bop bb s1; bopcommon` | `checkprops result -s 576.293` | needs `psphere <plane> R` placement |
| bopcut_simple/ZP3 | same geometry as bopcommon_simple/ZP3 | `bop b hs; bopcut` | `checkprops result -s 179000` | needs halfspace |
| bopcut_simple/ZP4 | same geometry as bopcommon_simple/ZP4 | `bop b hs; bopcut` | `checkprops result -s 81000` | needs halfspace |
| bopcut_simple/ZP5 | same geometry as bopcommon_simple/ZP5 | `bop b hs; bopcut` | `checkprops result -s 144000` | needs halfspace |
| bopcut_simple/ZP6 | same geometry as bopcommon_simple/ZP6 | `bop b hs; bopcut` | `checkprops result -s 116000` | needs halfspace |
| bopcut_simple/ZP7 | `vertex v1 -1 -1 0` \| `vertex v2 2 -1 0` \| `edge ea v1 v2` \| `prism fa ea 0 3.5 0` \| `prism ba fa 0 0 2` \| `vertex v3 0 0 0` \| `vertex v4 1 0 0` \| `vertex v5 1 0 2` \| `vertex v6 0 0 2` \| `edge eb v3 v4` \| `edge ec v4 v5` \| `edge ed v5 v6` \| `wire wb eb ec ed` \| `revol bb wb 0 0 0 0 0 1 360` \| `emptycopy s ba` \| `add bb s` | `bop ba s; bopcut` | `checkprops result -s 53.2832` | needs prism+revol (box with an internal cylinder as one shape) |
| bopcut_simple/ZP8 | `pcone pc 10 0 20` \| `explode pc f` \| `prism pcy pc_2 0 0 10` | `bop pc pcy; bopcut` | `checkprops result -s 254.16` | needs explode+prism |
| bopcut_simple/ZP9 | same as bopcommon_simple/ZP8 construction | `bop r_1 b1; bopcut` | `checkprops result -s 696` | needs shell surgery |
| bopcut_simple/ZQ1 | `plane pl1` \| `mkface face_1 pl1 -100 100 -100 100` \| `circle c1 0 0 0 20` \| `circle c2 0 0 0 30` \| `mkedge e1 c1` \| `wire w1 e1` \| `mkface d1 pl1 w1` \| `mkedge e2 c2` \| `wire w2 e2` \| `mkface d2 pl1 w2` \| `bop d2 d1` \| `bopcut D_coupe_1` \| `bop D_coupe_1 d1` \| `bopfuse Union_1` | `bop face_1 Union_1; bopcut` | `checkprops result -s 37172.6` | 2D face booleans, not solids |
| bopfuse_simple/ZP3 | `pcylinder cyl 1 4` \| `vertex va 0 -1 0` \| `vertex vb 0 1 0` \| `vertex vc 0 1 2` \| `vertex vd 0 -1 2` \| `edge e1 va vb` \| `edge e2 vb vc` \| `edge e3 vc vd` \| `edge e4 vd va` \| `wire w e1 e2 e3 e4` \| `mkplane f w 1` \| `prism priz f 1 0 0` | `bop priz cyl; bopfuse` | `checkprops result -s 33.9911` | needs prism (box 1x2x2 tangent-inside a r=1 cylinder) |
| bopfuse_simple/ZP4 | `pcone pc 10 0 20` \| `explode pc f` \| `prism pcy pc_2 0 0 10` | `bop pc pcy; bopfuse` | `checkprops result -s 1353.72` | needs explode+prism |
| bopfuse_simple/ZP5 | same as bopcommon_simple/ZP8 construction | `bop r_1 b1; bopfuse` | `checkprops result -s 600` | needs shell surgery |
| bopfuse_simple/ZP6 | `ptorus a0 100 20` \| `tcopy a0 a1` \| `tcopy a0 a2` \| `trotate a1 0 0 0 1 0 0 90` \| `trotate a2 0 0 0 0 1 0 90` \| `bop a0 a1` \| `bopfuse x1` | `bop x1 a2; bopfuse` | `checkprops result -s 197700` | **PORTABLE** (3 mutually orthogonal tori, chained fuse) — **but OCCT marks it TODO on Linux, see section 5** |
| boptuc_simple/ZP3 | `pcone pc 10 0 20` \| `explode pc f` \| `prism pcy pc_2 0 0 10` | `bop pc pcy; boptuc` | `checkprops result -s 1390.8` | needs explode+prism |
| boptuc_simple/ZP4 | same as bopcommon_simple/ZP8 construction | `bop r_1 b1; boptuc` | `checkprops result -s 96` | needs shell surgery |

### 2.3 b{common,cut,fuse}_simple + bfuse_complex + bsection — analytic primitive pairs (44 cases)

Older one-shot API: `bcut result A B` / `bfuse result A B` / `bcommon result A B` / `bsection result A B`
(no separate `bop` step). All primitive-only, all self-contained.

| case id | prelude (dset) | operand A | operand B | operation | expected assertions (verbatim) |
|---|---|---|---|---|---|
| bcommon_simple/A1 | - | `psphere s 1` | `box b 1 1 1` | bcommon | checkprops result -s 3.92699 |
| bcommon_simple/A2 | - | `psphere s 1 ; trotate s 0 0 0 0 0 1 -90 ; trotate s 0 0 0 0 1 0 -45` | `box b 1 1 1` | bcommon | checkprops result -s 3.92699 |
| bcommon_simple/A3 | - | `box b 1 1 1` | `psphere s 1 ; trotate s 0 0 0 0 0 1 -90 ; trotate s 0 0 0 0 1 0 -45` | bcommon | checkprops result -s 3.92699 |
| bcommon_simple/A4 | - | `psphere s 1` | `box b 1 1 1 ; trotate b 0 0 1 0 1 0 90` | bcommon | checkprops result -s 3.92699 |
| bcommon_simple/A5 | - | `box b 1 1 1 ; trotate b 0 0 1 0 1 0 90` | `psphere s 1` | bcommon | checkprops result -s 3.92699 |
| bcommon_simple/D3 | - | `psphere s 1` | `box b 1 1 1` | bcommon | checkprops result -s 3.92699 |
| bcommon_simple/D4 | - | `box b 1 1 1` | `psphere s 1` | bcommon | checkprops result -s 3.92699 |
| bcommon_simple/D5 | - | `psphere s 1 ; trotate s 0 0 0 0 0 1 -90 ; trotate s 0 0 0 0 1 0 -45` | `box b 1 1 1` | bcommon | checkprops result -s 3.92699 |
| bcommon_simple/D6 | - | `box b 1 1 1` | `psphere s 1 ; trotate s 0 0 0 0 0 1 -90 ; trotate s 0 0 0 0 1 0 -45` | bcommon | checkprops result -s 3.92699 |
| bcommon_simple/D7 | - | `psphere s 1` | `box b 1 1 1 ; trotate b 0 0 1 0 1 0 90` | bcommon | checkprops result -s 3.92699 |
| bcommon_simple/D8 | - | `box b 1 1 1 ; trotate b 0 0 1 0 1 0 90` | `psphere s 1` | bcommon | checkprops result -s 3.92699 |
| bcommon_simple/I5 | - | `box b1 3 3 3` | `box b2 1 1 1 ; ttranslate b2 0 3 0` | bcommon | checkprops result -s empty |
| bcommon_simple/J1 | - | `pcylinder c1 20 100` | `pcylinder c2 20 100 ; ttranslate c2 0 0 50` | bcommon | checkprops result -s 8796.46 |
| bcut_simple/A1 | - | `psphere s 1` | `box b 1 1 1` | bcut | checkprops result -s 13.3518 |
| bcut_simple/A2 | - | `psphere s 1 ; trotate s 0 0 0 0 0 1 -90 ; trotate s 0 0 0 0 1 0 -45` | `box b 1 1 1` | bcut | checkprops result -s 13.3517 |
| bcut_simple/A3 | - | `box b 1 1 1` | `psphere s 1 ; trotate s 0 0 0 0 0 1 -90 ; trotate s 0 0 0 0 1 0 -45` | bcut | checkprops result -s 5.2146 |
| bcut_simple/A4 | - | `psphere s 1` | `box b 1 1 1 ; trotate b 0 0 1 0 1 0 90` | bcut | checkprops result -s 13.3517 |
| bcut_simple/A5 | - | `box b 1 1 1 ; trotate b 0 0 1 0 1 0 90` | `psphere s 1` | bcut | checkprops result -s 5.2146 |
| bcut_simple/F9 | - | `psphere s 1` | `box b 1 1 1` | bcut | checkprops result -s 13.3518 |
| bcut_simple/G1 | - | `box b 1 1 1` | `psphere s 1` | bcut | checkprops result -s 5.2146 |
| bcut_simple/G2 | - | `psphere s 1 ; trotate s 0 0 0 0 0 1 -90 ; trotate s 0 0 0 0 1 0 -45` | `box b 1 1 1` | bcut | checkprops result -s 13.3517 |
| bcut_simple/G3 | - | `box b 1 1 1` | `psphere s 1 ; trotate s 0 0 0 0 0 1 -90 ; trotate s 0 0 0 0 1 0 -45` | bcut | checkprops result -s 5.2146 |
| bcut_simple/G4 | - | `psphere s 1` | `box b 1 1 1 ; trotate b 0 0 1 0 1 0 90` | bcut | checkprops result -s 13.3517 |
| bcut_simple/G5 | - | `box b 1 1 1 ; trotate b 0 0 1 0 1 0 90` | `psphere s 1` | bcut | checkprops result -s 5.2146 |
| bcut_simple/G7 | - | `box b1 3 3 3` | `box b2 1 1 1 ; ttranslate b2 0 3 0` | bcut | checkprops result -s 54 |
| bcut_simple/L8 | - | `pcylinder c1 20 100` | `pcylinder c2 20 100 ; ttranslate c2 0 0 50` | bcut | checkprops result -s 8796.46 |
| bfuse_complex/J1 | - | `pcylinder c1 10 20` | `pcylinder c2 5 20 ; ttranslate c2 5 0 10` | bfuse | checkprops result -s 2199.11 |
| bfuse_complex/J5 | - | `pcylinder a 50 150` | `pcylinder b 50 150 ; trotate b 0 0 75 0 1 0 90` | bfuse | checkprops result -s 85663.7 |
| bfuse_simple/A1 | - | `psphere s 1` | `box b 1 1 1` | bfuse | checkprops result -s 14.6394 |
| bfuse_simple/A2 | - | `psphere s 1 ; trotate s 0 0 0 0 0 1 -90 ; trotate s 0 0 0 0 1 0 -45` | `box b 1 1 1` | bfuse | checkprops result -s 14.6393 |
| bfuse_simple/A3 | - | `box b 1 1 1` | `psphere s 1 ; trotate s 0 0 0 0 0 1 -90 ; trotate s 0 0 0 0 1 0 -45` | bfuse | checkprops result -s 14.6393 |
| bfuse_simple/A4 | - | `psphere s 1` | `box b 1 1 1 ; trotate b 0 0 1 0 1 0 90` | bfuse | checkprops result -s 14.6393 |
| bfuse_simple/A5 | - | `box b 1 1 1 ; trotate b 0 0 1 0 1 0 90` | `psphere s 1` | bfuse | checkprops result -s 14.6393 |
| bfuse_simple/D3 | - | `psphere s 1` | `box b 1 1 1` | bfuse | checkprops result -s 14.6394 |
| bfuse_simple/D4 | - | `box b 1 1 1` | `psphere s 1` | bfuse | checkprops result -s 14.6394 |
| bfuse_simple/D5 | - | `psphere s 1 ; trotate s 0 0 0 0 0 1 -90 ; trotate s 0 0 0 0 1 0 -45` | `box b 1 1 1` | bfuse | checkprops result -s 14.6393 |
| bfuse_simple/D6 | - | `box b 1 1 1` | `psphere s 1 ; trotate s 0 0 0 0 0 1 -90 ; trotate s 0 0 0 0 1 0 -45` | bfuse | checkprops result -s 14.6393 |
| bfuse_simple/D7 | - | `psphere s 1` | `box b 1 1 1 ; trotate b 0 0 1 0 1 0 90` | bfuse | checkprops result -s 14.6393 |
| bfuse_simple/D8 | - | `box b 1 1 1 ; trotate b 0 0 1 0 1 0 90` | `psphere s 1` | bfuse | checkprops result -s 14.6393 |
| bfuse_simple/E2 | - | `box b1 3 3 3` | `box b2 1 1 1 ; ttranslate b2 0 3 0` | bfuse | checkprops result -s 58 |
| bfuse_simple/L1 | set MinFaceTolerance [ lindex $tolerance 14 ]; set MinEdgeTolerance [ lindex $tolerance 18 ]; set MinVertexTolerance [ lindex $tolerance 24 ] | `box b1 10 10 10` | `box b2 5 5 5 10 10 10` | bfuse | checkprops result -s 1050 |
| bfuse_simple/L2 | - | `pcylinder c1 20 100` | `pcylinder c2 20 100 ; ttranslate c2 0 0 50` | bfuse | checkprops result -s 21362.8 |
| bsection/Q3 | - | `pcylinder c1 10 20` | `pcylinder c2 5 20 ; ttranslate c2 5 0 10` | bsection | checkprops result -l 41.4159 ; checksection result |
| bsection/S1 | - | `pcylinder c1 20 100` | `pcylinder c2 20 100 ; ttranslate c2 0 0 50` | bsection | checkprops result -l 301.327 ; checksection result |
Caveats measured on three rows above:
- `bfuse_simple/L1` also runs `updatetolerance b1 1` and `updatetolerance b2 1` **before** the fuse
  (both operands inflated to tolerance 1.0 on a 10-unit box). Its `-s 1050` is only the ground truth
  under that tolerance inflation. Do not port it as a plain box-fuse cell.
- `bcut_simple/G9` and `bcut_simple/H3` are 3-primitive chains, not pairs, and are listed in 2.4.
- `bsection/Q3` and `bsection/S1` assert `-l` (**curve length** of the section wire), not area.

### 2.4 Chained-primitive portable cases (2 cases)

| case id | step 1 | step 2 | expected |
|---|---|---|---|
| bcut_simple/G9 | `pcylinder cyl 9 3` + `pcone kone 7 6 4` → `bfuse body cyl kone` | `pcylinder pcyl 1 9` ; `ttranslate pcyl 5 0 -2` ; `bcut result body pcyl` | `checkprops result -s 727.481` |
| bcut_simple/H3 | identical to G9 (duplicate case; `pcylinder cyl 9 3`, `pcone kone 7 6 4`, `bfuse body cyl kone`) | `pcylinder pcyl 1 9` ; `ttranslate pcyl 5 0 -2` ; `bcut result body pcyl` | `checkprops result -s 727.481` |

Both are OCCT bug `pro13307` — the cylinder's seam edge meets the cone's seam. Highest-value
seam-vs-seam stress in the whole self-contained corpus.

### 2.5 NURBS-converted variants of the same primitive pairs (96 cases)

Identical construction to 2.3 but with `nurbsconvert <name> <name>` applied to one or both operands
before the boolean — the analytic surface is replaced by an equivalent BSpline. Same geometry,
different surface representation, and OCCT's expected areas sometimes differ in the last digit
(e.g. `bcut_simple/A1` = 13.3518 analytic vs `bcut_simple/A2` = 13.3517 after rotation). Portable
only if the kernel can build a NURBS-form primitive; otherwise these are a second oracle pass over
the same 40-odd geometry pairs.

| case id | prelude (dset) | operand A (incl. nurbsconvert) | operand B | operation | expected assertions (verbatim) |
|---|---|---|---|---|---|
| bcommon_simple/A6 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 1 1 1` | bcommon | checkprops result -s 6 |
| bcommon_simple/A7 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 1 1.5 1` | bcommon | checkprops result -s 6 |
| bcommon_simple/A8 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 1 0 1 0.5 1` | bcommon | checkprops result -s empty |
| bcommon_simple/A9 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 1 1 0 1 1 1` | bcommon | checkprops result -s empty |
| bcommon_simple/B1 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 0.5 1 0.5` | bcommon | checkprops result -s 2.5 |
| bcommon_simple/B2 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 -0.5 0 0.5 0.5 1` | bcommon | checkprops result -s empty |
| bcommon_simple/B3 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 -0.5 0 0.5 1.5 1` | bcommon | checkprops result -s 4 |
| bcommon_simple/B4 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0.5 0 1 1 1` | bcommon | checkprops result -s 4 |
| bcommon_simple/B5 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0.25 0 1 0.5 1` | bcommon | checkprops result -s 4 |
| bcommon_simple/B6 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 0.5 0.5 0.5` | bcommon | checkprops result -s 1.5 |
| bcommon_simple/B7 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 -0.5 0 0.5 0.5 0.5` | bcommon | checkprops result -s empty |
| bcommon_simple/B8 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 -0.5 -0.5 0.5 0.5 0.5` | bcommon | checkprops result -s empty |
| bcommon_simple/B9 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 -0.5 -0.5 -0.5 0.5 0.5 0.5` | bcommon | checkprops result -s empty |
| bcommon_simple/C1 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 1.5 0.5 0.5` | bcommon | checkprops result -s 2.5 |
| bcommon_simple/C2 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 -0.5 0 1.5 0.5 0.5` | bcommon | checkprops result -s empty |
| bcommon_simple/C3 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0.25 0 0 0.5 0.5 1` | bcommon | checkprops result -s 2.5 |
| bcommon_simple/C4 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0.25 -0.5 0 0.5 0.5 1` | bcommon | checkprops result -s empty |
| bcommon_simple/C5 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0.25 0 0 0.5 1.5 1` | bcommon | checkprops result -s 4 |
| bcommon_simple/C6 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0.5 0 0 1 1 0.5` | bcommon | checkprops result -s 2.5 |
| bcommon_simple/C7 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0.5 0 -0.5 1 1 0.5` | bcommon | checkprops result -s empty |
| bcommon_simple/C8 | dset r sqrt(2) | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 r r/2 1 ; trotate b2 0 0 0 0 0 1 45` | bcommon | checkprops result -s 4.41421 |
| bcommon_simple/C9 | dset r sqrt(2)/2 | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 r r 1 ; trotate b2 0 0 0 0 0 1 45` | bcommon | checkprops result -s 2.91421 |
| bcommon_simple/D1 | dset r sqrt(2) | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 r 0.25 1 ; trotate b2 0 0 0 0 0 1 45` | bcommon | checkprops result -s 3.61764 |
| bcommon_simple/D2 | dset r sqrt(31) | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 r/4 0.25 1 ; trotate b2 0 0 0 0 0 1 34.73` | bcommon | checkprops result -s 3.65032 |
| bcut_simple/A6 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 1 1 1` | bcut | checkprops result -s empty |
| bcut_simple/A7 | - | `box b2 0 0 0 1 1 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s empty |
| bcut_simple/A8 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 1 1.5 1` | bcut | checkprops result -s empty |
| bcut_simple/A9 | - | `box b2 0 0 0 1 1.5 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 4 |
| bcut_simple/B1 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 1 0 1 0.5 1` | bcut | checkprops result -s 6 |
| bcut_simple/B2 | - | `box b2 0 1 0 1 0.5 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 4 |
| bcut_simple/B3 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 1 1 0 1 1 1` | bcut | checkprops result -s 6 |
| bcut_simple/B4 | - | `box b2 1 1 0 1 1 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 6 |
| bcut_simple/B5 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 0.5 1 0.5` | bcut | checkprops result -s 5.5 |
| bcut_simple/B6 | - | `box b2 0 0 0 0.5 1 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s empty |
| bcut_simple/B7 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 -0.5 0 0.5 0.5 1` | bcut | checkprops result -s 6 |
| bcut_simple/B8 | - | `box b2 0 -0.5 0 0.5 0.5 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 2.5 |
| bcut_simple/B9 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 -0.5 0 0.5 1.5 1` | bcut | checkprops result -s 4 |
| bcut_simple/C1 | - | `box b2 0 -0.5 0 0.5 1.5 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 2.5 |
| bcut_simple/C2 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0.5 0 1 1 1` | bcut | checkprops result -s 4 |
| bcut_simple/C3 | - | `box b2 0 0.5 0 1 1 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 4 |
| bcut_simple/C4 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0.25 0 1 0.5 1` | bcut | checkprops result -s 6 |
| bcut_simple/C5 | - | `box b2 0 0.25 0 1 0.5 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s empty |
| bcut_simple/C6 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 0.5 0.5 0.5` | bcut | checkprops result -s 6 |
| bcut_simple/C7 | - | `box b2 0 0 0 0.5 0.5 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s empty |
| bcut_simple/C8 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 -0.5 0 0.5 0.5 0.5` | bcut | checkprops result -s 6 |
| bcut_simple/C9 | - | `box b2 0 -0.5 0 0.5 0.5 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 1.5 |
| bcut_simple/D1 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 -0.5 -0.5 0.5 0.5 0.5` | bcut | checkprops result -s 6 |
| bcut_simple/D2 | - | `box b2 0 -0.5 -0.5 0.5 0.5 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 1.5 |
| bcut_simple/D3 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 -0.5 -0.5 -0.5 0.5 0.5 0.5` | bcut | checkprops result -s 6 |
| bcut_simple/D4 | - | `box b2 -0.5 -0.5 -0.5 0.5 0.5 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 1.5 |
| bcut_simple/D5 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 1.5 0.5 0.5` | bcut | checkprops result -s 5.5 |
| bcut_simple/D6 | - | `box b2 0 0 0 1.5 0.5 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 1.5 |
| bcut_simple/D7 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 -0.5 0 1.5 0.5 0.5` | bcut | checkprops result -s 6 |
| bcut_simple/D8 | - | `box b2 0 -0.5 0 1.5 0.5 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 3.5 |
| bcut_simple/D9 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0.25 0 0 0.5 0.5 1` | bcut | checkprops result -s 6.5 |
| bcut_simple/E1 | - | `box b2 0.25 0 0 0.5 0.5 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s empty |
| bcut_simple/E2 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0.25 -0.5 0 0.5 0.5 1` | bcut | checkprops result -s 6 |
| bcut_simple/E3 | - | `box b2 0.25 -0.5 0 0.5 0.5 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 2.5 |
| bcut_simple/E4 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0.25 0 0 0.5 1.5 1` | bcut | checkprops result -s 6 |
| bcut_simple/E5 | - | `box b2 0.25 0 0 0.5 1.5 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 2.5 |
| bcut_simple/E6 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0.5 0 0 1 1 0.5` | bcut | checkprops result -s 5.5 |
| bcut_simple/E7 | - | `box b2 0.5 0 0 1 1 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 2.5 |
| bcut_simple/E8 | - | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0.5 0 -0.5 1 1 0.5` | bcut | checkprops result -s 6 |
| bcut_simple/E9 | - | `box b2 0.5 0 -0.5 1 1 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 4 |
| bcut_simple/F1 | dset r sqrt(2) | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 r r/2 1 ; trotate b2 0 0 0 0 0 1 45` | bcut | checkprops result -s 4.41421 |
| bcut_simple/F2 | dset r sqrt(2) | `box b2 0 0 0 r r/2 1 ; trotate b2 0 0 0 0 0 1 45` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 5.82843 |
| bcut_simple/F3 | dset r sqrt(2)/2 | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 r r 1 ; trotate b2 0 0 0 0 0 1 45` | bcut | checkprops result -s 5.91421 |
| bcut_simple/F4 | dset r sqrt(2)/2 | `box b2 0 0 0 r r 1 ; trotate b2 0 0 0 0 0 1 45` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 2.91421 |
| bcut_simple/F5 | dset r sqrt(2) | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 r 0.25 1 ; trotate b2 0 0 0 0 0 1 45` | bcut | checkprops result -s 7.03921 |
| bcut_simple/F6 | dset r sqrt(2) | `box b2 0 0 0 r 0.25 1 ; trotate b2 0 0 0 0 0 1 45` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 1.83211 |
| bcut_simple/F7 | dset r sqrt(31) | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | `box b2 0 0 0 r/4 0.25 1 ; trotate b2 0 0 0 0 0 1 34.73` | bcut | checkprops result -s 7.21677 |
| bcut_simple/F8 | dset r sqrt(31) | `box b2 0 0 0 r/4 0.25 1 ; trotate b2 0 0 0 0 0 1 34.73` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bcut | checkprops result -s 1.54631 |
| bfuse_simple/A6 | - | `box b2 0 0 0 1 1 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 6 |
| bfuse_simple/A7 | - | `box b2 0 0 0 1 1.5 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 8 |
| bfuse_simple/A8 | - | `box b2 0 1 0 1 0.5 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 8 |
| bfuse_simple/A9 | - | `box b2 1 1 0 1 1 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 12 |
| bfuse_simple/B1 | - | `box b2 0 0 0 0.5 1 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 6 |
| bfuse_simple/B2 | - | `box b2 0 -0.5 0 0.5 0.5 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 7.5 |
| bfuse_simple/B3 | - | `box b2 0 -0.5 0 0.5 1.5 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 7.5 |
| bfuse_simple/B4 | - | `box b2 0 0.5 0 1 1 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 8 |
| bfuse_simple/B5 | - | `box b2 0 0.25 0 1 0.5 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 6 |
| bfuse_simple/B6 | - | `box b2 0 0 0 0.5 0.5 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 6 |
| bfuse_simple/B7 | - | `box b2 0 -0.5 0 0.5 0.5 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 7 |
| bfuse_simple/B8 | - | `box b2 0 -0.5 -0.5 0.5 0.5 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 7.5 |
| bfuse_simple/B9 | - | `box b2 -0.5 -0.5 -0.5 0.5 0.5 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 7.5 |
| bfuse_simple/C1 | - | `box b2 0 0 0 1.5 0.5 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 7 |
| bfuse_simple/C2 | - | `box b2 0 -0.5 0 1.5 0.5 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 8.5 |
| bfuse_simple/C3 | - | `box b2 0.25 0 0 0.5 0.5 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 6 |
| bfuse_simple/C4 | - | `box b2 0.25 -0.5 0 0.5 0.5 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 7.5 |
| bfuse_simple/C5 | - | `box b2 0.25 0 0 0.5 1.5 1` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 7.5 |
| bfuse_simple/C6 | - | `box b2 0.5 0 0 1 1 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 7.5 |
| bfuse_simple/C7 | - | `box b2 0.5 0 -0.5 1 1 0.5` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 9 |
| bfuse_simple/C8 | dset r sqrt(2) | `box b2 0 0 0 r r/2 1 ; trotate b2 0 0 0 0 0 1 45` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 7.82843 |
| bfuse_simple/C9 | dset r sqrt(2)/2 | `box b2 0 0 0 r r 1 ; trotate b2 0 0 0 0 0 1 45` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 6.91421 |
| bfuse_simple/D1 | dset r sqrt(2) | `box b2 0 0 0 r 0.25 1 ; trotate b2 0 0 0 0 0 1 45` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 6.41789 |
| bfuse_simple/D2 | dset r sqrt(31) | `box b2 0 0 0 r/4 0.25 1 ; trotate b2 0 0 0 0 0 1 34.73` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1` | bfuse | checkprops result -s 6.32953 |
### 2.6 Multi-operand / builder-API portable cases (51 cases)

All primitive-only and self-contained, but they drive the **General Fuse / Cells builder / Splitter /
Periodicity** APIs rather than a 2-operand boolean. Included complete because the operand
constructions and expected areas/volumes are directly reusable as a general-fuse oracle.

| case id | operand constructions (exact DRAW) | builder call sequence | expected assertions (verbatim) |
|---|---|---|---|
| cells_test/A1 | `psphere s1 15` \| `psphere s2 15` \| `psphere s3 15` \| `ttranslate s1 0 0 10` \| `ttranslate s2 20 0 10` \| `ttranslate s3 10 0 0` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 1 s2 0 s3 1` | checkprops result -s 1345.27 |
| cells_test/A2 | `psphere s1 15` \| `psphere s2 15` \| `psphere s3 15` \| `ttranslate s1 0 0 10` \| `ttranslate s2 20 0 10` \| `ttranslate s3 10 0 0` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 0 s2 1 s3 1` | checkprops result -s 1345.27 |
| cells_test/A3 | `psphere s1 15` \| `psphere s2 15` \| `psphere s3 15` \| `ttranslate s1 0 0 10` \| `ttranslate s2 20 0 10` \| `ttranslate s3 10 0 0` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 1 s2 0 s3 1` \| `bcadd result s1 0 s2 1 s3 1` | checkprops result -s 2690.54 |
| cells_test/A4 | `psphere s1 15` \| `psphere s2 15` \| `psphere s3 15` \| `ttranslate s1 0 0 10` \| `ttranslate s2 20 0 10` \| `ttranslate s3 10 0 0` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 1 s2 1 s3 1` | checkprops result -s 769.835 |
| cells_test/A5 | `psphere s1 15` \| `psphere s2 15` \| `psphere s3 15` \| `ttranslate s1 0 0 10` \| `ttranslate s2 20 0 10` \| `ttranslate s3 10 0 0` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 1 s3 1` | checkprops result -s 2115.11 |
| cells_test/A6 | `psphere s1 15` \| `psphere s2 15` \| `psphere s3 15` \| `ttranslate s1 0 0 10` \| `ttranslate s2 20 0 10` \| `ttranslate s3 10 0 0` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s2 1 s3 1` | checkprops result -s 2115.11 |
| cells_test/A7 | `psphere s1 15` \| `psphere s2 15` \| `psphere s3 15` \| `ttranslate s1 0 0 10` \| `ttranslate s2 20 0 10` \| `ttranslate s3 10 0 0` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 1 s3 1` \| `bcadd result s2 1 s3 2` | checkprops result -s 3460.38 |
| cells_test/A8 | `psphere s1 15` \| `psphere s2 15` \| `psphere s3 15` \| `ttranslate s1 0 0 10` \| `ttranslate s2 20 0 10` \| `ttranslate s3 10 0 0` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 0 s2 0 s3 1` | checkprops result -s 2356.19 |
| cells_test/A9 | `psphere s1 15` \| `psphere s2 15` \| `psphere s3 15` \| `ttranslate s1 0 0 10` \| `ttranslate s2 20 0 10` \| `ttranslate s3 10 0 0` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s2 0 s3 1` | checkprops result -s 3701.46 |
| cells_test/B1 | `psphere s1 15` \| `psphere s2 15` \| `psphere s3 15` \| `ttranslate s1 0 0 10` \| `ttranslate s2 20 0 10` \| `ttranslate s3 10 0 0` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 0 s3 1` | checkprops result -s 3701.46 |
| cells_test/B2 | `psphere s1 15` \| `psphere s2 15` \| `psphere s3 15` \| `ttranslate s1 0 0 10` \| `ttranslate s2 20 0 10` \| `ttranslate s3 10 0 0` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s2 0 s3 1` \| `bcadd result s1 0 s3 1` | checkprops result -s 5046.74 |
| cells_test/B3 | `psphere s1 15` \| `psphere s2 15` \| `psphere s3 15` \| `ttranslate s1 0 0 10` \| `ttranslate s2 20 0 10` \| `ttranslate s3 10 0 0` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s3 1` | checkprops result -s 5816.57 |
| cells_test/B4 | `psphere s1 15` \| `psphere s2 15` \| `psphere s3 15` \| `ttranslate s1 0 0 10` \| `ttranslate s2 20 0 10` \| `ttranslate s3 10 0 0` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3` \| `bfillds` \| `bcbuild r` \| `bcaddall result` \| `bcremove result s1 1 s2 1 s3 1` \| `bcremove result s2 0 s3 1` | checkprops result -s 7172.78 |
| cells_test/B5 | `psphere s1 15` \| `psphere s2 15` \| `psphere s3 15` \| `ttranslate s1 0 0 10` \| `ttranslate s2 20 0 10` \| `ttranslate s3 10 0 0` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 1 s2 0 s3 0 -m 1` \| `bcadd result s1 1 s2 0 s3 1 -m 1` \| `bcadd result s1 1 s2 1 s3 0 -m 2` \| `bcadd result s1 1 s2 1 s3 1 -m 2` \| `bcadd result s1 0 s2 1 s3 1 -m 2` \| `bcremoveint result` | checkprops result -s 4494.64 |
| cells_test/B6 | `psphere s1 15` \| `psphere s2 15` \| `psphere s3 15` \| `ttranslate s1 0 0 10` \| `ttranslate s2 20 0 10` \| `ttranslate s3 10 0 0` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 1 s2 0 s3 0 -m 1` \| `bcadd result s1 1 s2 0 s3 1 -m 1 -u` \| `bcadd result s1 1 s2 1 s3 0 -m 2` \| `bcadd result s1 1 s2 1 s3 1 -m 2` \| `bcadd result s1 0 s2 1 s3 1 -m 2 -u` \| `bcremoveint result` | checkprops result -s 4494.64 |
| cells_test/E1 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcaddall result` | checkprops result -s 7729.72 |
| cells_test/E2 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcaddall result -m 1 -u` | checkprops result -s 2689.14 |
| cells_test/E3 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 1 s2 1 s3 1 s4 1` | checkprops result -s 417.967 |
| cells_test/E4 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 1` | checkprops result -s 1416.31 |
| cells_test/E5 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s2 1` | checkprops result -s 4993.56 |
| cells_test/E6 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s3 1` | checkprops result -s 5017.47 |
| cells_test/E7 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s4 1` | checkprops result -s 2086.49 |
| cells_test/E8 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcaddall result` \| `bcremove result s1 1 s2 1 s3 1 s4 1` | checkprops result -s 7311.75 |
| cells_test/E9 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 1 s2 1 -m 1 -u` | checkprops result -s 502.043 |
| cells_test/F1 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 1 s3 1 -m 1 -u` | checkprops result -s 514.159 |
| cells_test/F2 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 1 s4 1 -m 1 -u` | checkprops result -s 418.093 |
| cells_test/F3 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s2 1 s3 1 -m 1 -u` | checkprops result -s 1135.58 |
| cells_test/F4 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s2 1 s4 1 -m 1 -u` | checkprops result -s 1017.77 |
| cells_test/F5 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s3 1 s4 1 -m 1 -u` | checkprops result -s 1232.14 |
| cells_test/F6 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 1 s2 0 s3 0 s4 0` | checkprops result -s 259.774 |
| cells_test/F7 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 0 s2 1 s3 0 s4 0` | checkprops result -s 2067.69 |
| cells_test/F8 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 0 s2 0 s3 1 s4 0` | checkprops result -s 1640.54 |
| cells_test/F9 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 0 s2 0 s3 0 s4 1` | checkprops result -s 145.291 |
| cells_test/G1 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 1 s2 0 s3 0 s4 0` \| `bcadd result s1 0 s2 1 s3 0 s4 0` \| `bcadd result s1 0 s2 0 s3 1 s4 0` \| `bcadd result s1 0 s2 0 s3 0 s4 1` | checkprops result -s 4113.3 |
| cells_test/G2 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 1 -m 1` \| `bcremove result s1 1 s2 0 s3 0 s4 0` \| `bcremoveint result` | checkprops result -s 545.266 |
| cells_test/G3 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s2 1 -m 1` \| `bcremove result s1 0 s2 1 s3 0 s4 0` \| `bcremoveint result` | checkprops result -s 1166.69 |
| cells_test/G4 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s3 1 -m 1` \| `bcremove result s1 0 s2 0 s3 1 s4 0` \| `bcremoveint result` | checkprops result -s 1393.05 |
| cells_test/G5 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s4 1 -m 1` \| `bcremove result s1 0 s2 0 s3 0 s4 1` \| `bcremoveint result` | checkprops result -s 1232.14 |
| cells_test/G6 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcaddall result -m 1` \| `bcremove result s1 1 s2 0 s3 0 s4 0` \| `bcremove result s1 0 s2 1 s3 0 s4 0` \| `bcremove result s1 0 s2 0 s3 1 s4 0` \| `bcremove result s1 0 s2 0 s3 0 s4 1` \| `bcremoveint result` | checkprops result -s 1424.16 |
| cells_test/G7 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcremoveall` \| `bcadd result s1 1 -m 1` \| `bcadd result s2 1 -m 2` \| `bcadd result s3 1 -m 3` \| `bcadd result s4 1 -m 4` \| `bcremoveint result` | checkprops result -s 4326.81 |
| cells_test/G8 | `box s1 10 10 10` \| `psphere s2 12` \| `pcylinder s3 10 20` \| `pcone s4 10 0 30` | `bclearobjects` \| `bcleartools` \| `baddobjects s1 s2 s3 s4` \| `bfillds` \| `bcbuild r` \| `bcaddall result` \| `bcmakecontainers result` \| `bcremove result s1 1` \| `bcadd result s1 1 s2 1` | checkprops result -s 7299.17 |
| opensolid/A4 | `box b1 40 10 10` \| `box b2 0 0 30 40 10 10` \| `box b3 10 10 40` \| `box b4 30 0 0 10 10 40` | `bclearobjects` \| `bcleartools` \| `baddobjects b1 b2 b3 b4` \| `bfillds` \| `bbuild r` \| `buildbop r1 -o b1 b2 b3 b4 -op fuse` \| `buildbop r2 -o b1 b2 b3 -t b4 -op fuse` \| `buildbop r3 -o b1 b2 -t b3 b4 -op fuse` \| `buildbop r4 -o b1 -t b2 b3 b4 -op fuse` \| `buildbop r5 -t b1 b2 b3 b4 -op fuse` \| `buildbop r1 -o b1 b2 -t b3 b4 -op cut` \| `buildbop r2 -o b1 b2 -t b3 b4 -op tuc` \| `buildbop r1 -o b1 -t b3 b4 -op cut` \| `buildbop r2 -o b2 -t b3 b4 -op cut` \| `buildbop r1 -o b1 b2 -t b3 b4 -op common` \| `buildbop r1 -o b1 -t b3 b4 -op common` \| `buildbop r2 -o b2 -t b3 b4 -op common` | checkshape $r ; checknbshapes $r -vertex 32 -edge 64 -wire 32 -face 32 -shell 1 -solid 1 ; checkprops $r -s 4800 -v 12000 ; checkshape $r ; checknbshapes $r -vertex 16 -edge 24 -wire 12 -face 12 -shell 2 -solid 2 ; checkprops $r -s 2000 -v 4000 ; checkshape $r ; checknbshapes $r -vertex 8 -edge 12 -wire 6 -face 6 -shell 1 -solid 1 ; checkprops $r -s 1000 -v 2000 ; checkshape r1 ; checknbshapes r1 -vertex 32 -edge 48 -wire 24 -face 24 -shell 4 -solid 4 ; checkprops r1 -s 2400 -v 4000 ; checkshape r1 ; checkshape r2 ; checkshape $r ; checknbshapes $r -vertex 16 -edge 24 -wire 12 -face 12 -shell 2 -solid 2 ; checkprops $r -s 1200 -v 2000 |
| opensolid/A6 | `box b1 10 2 2` \| `box b2 5 0 0 5 5 5` \| `box b3 6 0 0 1 1 1` | `bclearobjects` \| `bcleartools` \| `baddobjects b1` \| `baddtools b2 b3` \| `bfillds` \| `bbuild r` \| `buildbop r0 -o b1 -t b2 b3 -op common` \| `buildbop r1 -o b1 -t b2 b3 -op fuse` \| `buildbop r2 -o b1 -t b2 b3 -op cut` \| `buildbop r3 -o b1 -t b2 b3 -op tuc` | checkshape $r ; checkprops r0 -s 56 -v 20 ; checknbshapes r0 -vertex 16 -edge 25 -wire 12 -face 12 -shell 2 -solid 2 ; checkprops r1 -s 190 -v 145 ; checknbshapes r1 -vertex 24 -edge 38 -wire 16 -face 16 -shell 1 -solid 1 ; checkprops r2 -s 48 -v 20 ; checknbshapes r2 -vertex 8 -edge 12 -wire 6 -face 6 -shell 1 -solid 1 ; checkprops r3 -s 142 -v 105 ; checknbshapes r3 -vertex 12 -edge 18 -wire 8 -face 8 -shell 1 -solid 1 |
| opensolid/A7 | `box b1 10 10 10` \| `box b2 1 1 1 8 8 8` \| `box b3 -2 3 3 14 4 4` | `bclearobjects` \| `bcleartools` \| `baddobjects b1 b2 b3` \| `bfillds` \| `bbuild r` \| `buildbop r0 -o b3 -t b1 b2 -op common` \| `buildbop r1 -o b3 -t b1 b2 -op fuse` \| `buildbop r2 -o b3 -t b1 b2 -op cut` \| `buildbop r3 -o b3 -t b1 b2 -op tuc` \| `buildbop r0 -o b2 -t b1 b3 -op common` \| `buildbop r1 -o b2 -t b1 b3 -op fuse` \| `buildbop r2 -o b2 -t b1 b3 -op cut` \| `buildbop r3 -o b2 -t b1 b3 -op tuc` | checkshape $r ; checkprops r0 -s 256 -v 160 ; checknbshapes r0 -vertex 16 -edge 28 -wire 16 -face 16 -shell 3 -solid 3 ; checkprops r1 -s 664 -v 1064 ; checknbshapes r1 -vertex 24 -edge 36 -wire 18 -face 16 -shell 1 -solid 1 ; checkprops r2 -s 128 -v 64 ; checknbshapes r2 -vertex 16 -edge 24 -wire 12 -face 12 -shell 2 -solid 2 ; checkprops r3 -s 1432 -v 840 ; checknbshapes r3 -vertex 32 -edge 52 -wire 28 -face 24 -shell 2 -solid 2 ; checkshape $r ; checkprops r0 -s 640 -v 512 ; checknbshapes r0 -vertex 16 -edge 24 -wire 14 -face 12 -shell 2 -solid 2 ; checkprops r1 -s 664 -v 1064 ; checknbshapes r1 -vertex 24 -edge 36 -wire 18 -face 16 -shell 1 -solid 1 ; checkprops r2 -s empty -v empty ; checknbshapes r2 -vertex 0 -edge 0 -wire 0 -face 0 -shell 0 -solid 0 ; checkprops r3 -s 1176 -v 552 ; checknbshapes r3 -vertex 40 -edge 64 -wire 38 -face 34 -shell 5 -solid 5 |
| opensolid/A8 | `box b1 10 10 10` \| `box b2 -2 -2 2 7 14 6` \| `box b3 5 -2 2 7 14 6` | `bclearobjects` \| `bcleartools` \| `baddobjects b1` \| `baddtools b2 b3` \| `bfillds` \| `bbuild r` \| `buildbop r0 -o b1 -t b3 b2 -op common` \| `buildbop r1 -o b1 -t b3 b2 -op fuse` \| `buildbop r2 -o b1 -t b3 b2 -op cut` \| `buildbop r3 -o b1 -t b3 b2 -op tuc` | checkshape $r ; checkprops r0 -s 560 -v 600 ; checknbshapes r0 -vertex 12 -edge 20 -wire 11 -face 11 -shell 2 -solid 2 ; checkprops r1 -s 888 -v 1576 ; checknbshapes r1 -vertex 32 -edge 50 -wire 20 -face 20 -shell 1 -solid 1 ; checkprops r2 -s 560 -v 400 ; checknbshapes r2 -vertex 20 -edge 30 -wire 14 -face 14 -shell 2 -solid 2 ; checkprops r3 -s 816 -v 576 ; checknbshapes r3 -vertex 24 -edge 40 -wire 18 -face 18 -shell 2 -solid 2 |
| periodicity/A1 | `box b 10 10 10` | `makeperiodic bp b -x 5 -trim 2 -y 8 -trim 1 -z 12 -trim -1` \| `repeatshape result -x 5 -y 5 -z 5` | checkshape bp ; checknbshapes bp -vertex 8 -edge 12 -wire 6 -face 6 -shell 1 -solid 1 -t ; checkprops bp -s 340 -v 400 ; checkshape result ; checknbshapes result -vertex 588 -edge 1302 -wire 936 -face 936 -shell 216 -solid 216 -t ; checkprops result -s 73440 -v 86400 |
| periodicity/A3 | `box b 100 100 1` \| `pcylinder c 2 2` \| `ttranslate c 2.5 2.5 0` | `makeperiodic p c -x 5 -trim 0 -y 5 -trim 0` \| `repeatshape r -x 19 -y 19` \| `bcut result b r` | checkshape result ; checknbshapes result -vertex 808 -edge 1212 -wire 1206 -face 406 -shell 1 -solid 1 -t ; checkprops result -s 15373.5 -v 4973.45 |
| periodicity/A4 | `box b 5 5 1` \| `pcylinder c 2 2` \| `ttranslate c 2.5 2.5 0` | `bcut p b c` \| `makeperiodic p p -x 5 -y 5` \| `repeatshape result -x -9 -x 1 -y -9 -y 1` | checkshape result ; checknbshapes result -vertex 1682 -edge 3321 -wire 2840 -face 2040 -shell 400 -solid 400 -t ; checkprops result -s  22973.5 -v 4973.45 |
| simplify/A1 | `box b1 10 10 15` \| `box b2 3 7 0 10 10 15` | `boptions -default` \| `bclearobjects` \| `bcleartools` \| `baddobjects b1` \| `baddtools b2` \| `bfillds` \| `bapibop res 1` \| `bsimplify -f 1` \| `bapibop res_simple 1` \| `boptions -default` | checkshape res ; checknbshapes res -vertex 20 -edge 32 -wire 14 -face 14 ; checkshape res_simple ; checknbshapes res_simple -vertex 16 -edge 24 -wire 10 -face 10 |
| splitter/A1 | `box b1 10 10 10` \| `box b2 4 -2 -2 2 14 14` \| `box b3 -2 4 -2 14 2 14` | `bclearobjects` \| `bcleartools` \| `baddobjects b1` \| `baddtools b2 b3` \| `bfillds` \| `bsplit result` | checkshape result ; checknbshapes result -solid 9 ; checkprops result -v 1000 |
| splitter/B1 | `box b1 10 10 10` \| `box b2 4 -2 -2 2 14 14` \| `box b3 -2 4 -2 14 2 14` | `bclearobjects` \| `bcleartools` \| `baddobjects b1` \| `baddtools b2 b3` \| `bfillds` \| `bapisplit result` | checkshape result ; checknbshapes result -solid 9 ; checkprops result -v 1000 |

Notes measured on this block:
- `cells_test` A1–B6 = three r=15 spheres at (0,0,10) (20,0,10) (10,0,0); E1–G8 = the four-solid soup
  `box 10 10 10` + `psphere 12` + `pcylinder 10 20` + `pcone 10 0 30`, all at the origin — i.e. a
  cone whose apex is on the cylinder axis and a sphere that cuts the box corners. 41 different cell
  selections over those two soups, each with its own expected area.
- `opensolid/A4/A6/A7/A8` are the only grid cases that assert **both** area and volume **and**
  `checknbshapes` counts for all four ops (fuse/cut/tuc/common) on the same operand set — the
  richest single-file oracles in the corpus.
- `periodicity/A3` and `A4` assert on the *repeated* shape, not on the boolean result, so their
  `checkprops` is not a usable boolean oracle; `periodicity/A1` contains no boolean at all.
- `volumemaker` (74 cases, all self-contained, 66 with numeric asserts) is **excluded from the
  portable corpus** because 0 of its cases use a solid primitive: every operand is an infinite
  `plane`/`cone`/`sphere`/`cylinder` surface turned into a face with `mkface ... -1000000 1000000
  -1000000 1000000` and fed to `mkvolume`. Reusable only if a face-soup volume-maker exists.
- `gdml_public` (25 cases, all self-contained) is likewise excluded: all 25 mix primitives with
  `compound`/`add`/`plane`, and only 1 of the 25 asserts any number — the rest assert only
  `checkshape` + an image. Low oracle value.

---

## 3. TRANSLATION NOTES — exact DRAW semantics

Read from `src/Draw/TKTopTest/BRepTest/BRepTest_PrimitiveCommands.cxx` (function `BRepTest::PrimitiveCommands`)
and `.../BRepTest_BasicCommands.cxx` (function `transform`). Angles in DRAW are **degrees**;
every constructor multiplies by `M_PI/180`.

### 3.1 Solid primitives

| DRAW | OCCT call | placement / meaning |
|---|---|---|
| `box name dx dy dz` | `BRepPrimAPI_MakeBox(gp_Pnt(0,0,0), dx, dy, dz)` | **minimum corner at the ORIGIN**, extends `+X +Y +Z`. Not centered. |
| `box name x y z dx dy dz` | `BRepPrimAPI_MakeBox(gp_Pnt(x,y,z), dx, dy, dz)` | minimum corner at `(x,y,z)`, extends `+dx +dy +dz`. |
| `box name -min x y z -max X Y Z` | `BRepPrimAPI_MakeBox(gp_Pnt, gp_Pnt)` | corner-to-corner form. **Not used by any case in this corpus.** |
| `pcylinder name R H` | `BRepPrimAPI_MakeCylinder(R, H)` | axis `+Z`, **base circle at z=0**, top cap at `z=H`, centre `(0,0,·)`. Full 360°. |
| `pcylinder name R H angle` | `...MakeCylinder(R, H, angle·π/180)` | wedge of a cylinder. **0 occurrences in the portable corpus.** |
| `pcone name R1 R2 H` | `BRepPrimAPI_MakeCone(R1, R2, H)` | axis `+Z`, **bottom radius R1 at z=0, top radius R2 at z=H**. `R2=0` ⇒ apex cone. |
| `pcone name R1 R2 H angle` | `...MakeCone(R1,R2,H,angle·π/180)` | wedge. **0 occurrences in the portable corpus.** |
| `psphere name R` | `BRepPrimAPI_MakeSphere(R)` | **centred at the ORIGIN**, poles on `±Z`, seam meridian at `u=0` (the `+X` half-plane). |
| `psphere name R angle` | `...MakeSphere(R, angle·π/180)` | spherical wedge. **0 occurrences in the portable corpus.** |
| `psphere name R a1 a2` | `...MakeSphere(R, a1·π/180, a2·π/180)` | latitude segment, `a1,a2 ∈ [-90,90]`. **0 occurrences.** |
| `ptorus name R1 R2` | `BRepPrimAPI_MakeTorus(R1, R2)` | **centred at the ORIGIN, axis `+Z`**; `R1` = major (origin→tube-centre), `R2` = minor (tube). |
| `ptorus name R1 R2 angle…` | segments | **0 occurrences.** |
| `<prim> name <planeVar> …` | the plane's `Pln().Position().Ax2()` is used as the placement frame | only 1 occurrence in the whole corpus: `bopcommon_simple/ZP9` (`psphere s1 pl1 7.5`). |

**Measured:** across the 371 core geometry pairs there are **448 `box`** (408 in the 6-arg
corner+size form, 40 in the 3-arg size-only form), **209 `pcylinder`**, **64 `pcone`**, **17 `psphere`**,
**4 `ptorus`** — and **zero partial-angle primitives**. Everything is a full revolution.

**Measured, and important:** all 64 `pcone` instances in the 371 core pairs have `R2 ≠ 0`, i.e. they
are **truncated cones (frusta)**, never apex cones. Across the four bop grids `R2` takes the values
`{0.5×28, 1×72, 2×52, 3.5×4, 4×76, 6×8, 8×16}` plus `0` exactly 4 times — and those 4 are the
non-core tail cases `ZP7/ZP8` (`pcone pc 10 0 20`).

### 3.2 Transforms

| DRAW | semantics |
|---|---|
| `ttranslate name dx dy dz` | `gp_Trsf::SetTranslation(dx,dy,dz)` |
| `trotate name px py pz dx dy dz ang` | `gp_Trsf::SetRotation(gp_Ax1(gp_Pnt(px,py,pz), gp_Vec(dx,dy,dz)), ang·π/180)` — right-handed about the axis through `(px,py,pz)`. |
| `tmirror`, `tscale` | mirror about `gp_Ax2`, scale about a point. **0 occurrences in the portable corpus.** |

The command name's **first letter selects how the transform is applied** (function `transform`):
`b*` (e.g. `btranslate`) and `f*` set a `TopLoc_Location` only; **`t*` (the prefix used everywhere in
this corpus) goes through `BRepBuilderAPI_Transform`, which BAKES the transform into the geometry**
(new surfaces/curves, `isCopy=false` so the shape is replaced in place). So `ttranslate`/`trotate`
can be ported as a genuine geometric transform of the built solid; there is no location/instancing
subtlety to reproduce.

`trotate` angle histogram over the 371 core pairs (227 rotations total):
`90×42, 45×34, -45×24, 30×20, 60×16, -30×15, 180×12, 270×11, a30×8, 135×8, 34.73×4, a30-90×4,
120×4, 230×4, -35×3, -50×2, 40×2, 240×2, 35×2, 115×2, 250×2, 245×2, 125×2, -120×1, -90×1`
where `a30` is the Tcl variable from `dset a30 atan2(1,2)*180/pi` ≈ 26.5651°.

### 3.3 `dset` prelude expressions actually used (371 core pairs)

`dset` evaluates a Tcl `expr`; the variable is then substituted into a later primitive line.
Exact multiset measured:
`sqrt(2)/2 ×31`, `sqrt(3)/2 ×20`, `atan2(1,2)*180/pi ×12`, `sqrt(2) ×9`, `sqrt(5) ×9`,
`sqrt(31) ×4`, `sqrt(2)*0.75 ×3`, `sqrt(30) ×2`, `sqrt(5)*0.5 ×2`, `sqrt(2)*0.5 ×2`,
`sqrt(5)/4 ×1`, `sqrt(2)*0.875 ×1` — **96 `dset` lines spread over 84 of the 371 core pairs**
(= 336 of the 1484 core cases carry a prelude; some pairs carry two `dset` lines). In the
non-core tables, 16 of the 96 nurbsconvert variants carry a prelude.

### 3.4 Assertion semantics (`resources/DrawResources/CheckCommands.tcl`)

- `checkprops <shape> -s <A>` runs `sprops shape 1.0e-4` and compares `Mass` to `A`.
  **`-s` is AREA, `-l` is LENGTH, `-v` is VOLUME.**
- **Tolerance is RELATIVE and is 1% by default** (`set depsilon 1e-2`; failure iff
  `abs((expected-actual)/expected) > depsilon`), overridable per case with `-deps`. No case in the
  portable corpus overrides it. The numeric integration epsilon is `1.0e-4` (`-eps`).
- `checkprops <shape> -s empty` asserts `Mass == 0` exactly — the result must be a void shape.
  If the DRAW variable does not exist at all, `checkprops` prints `Error: The command cannot be built`.
- `checknbshapes <shape> -vertex N -edge N -wire N -face N -shell N -solid N [-t]` compares the
  `nbshapes` histogram; `-t` counts with locations.
- `checkshape result` runs `BRepCheck_Analyzer`; the grid-level `end` file runs it automatically on
  `result` for **every** case, so every listed expected value is additionally an implicit
  "result is a valid shape" assertion.
- `checksection result` asserts the section result has no free/dangling vertices.
- Global `begin` sets `cpulimit 300` (5 min per case) and `dset SCALE 100`.

### 3.5 Mapping onto `session_cpp` `BRep::create_*`

Read from `session_cpp/src/brep.cpp` (measured, not assumed):

| our API | actual placement | DRAW equivalent | conversion needed |
|---|---|---|---|
| `BRep::create_box(sx,sy,sz)` | box **CENTRED ON THE ORIGIN**; internally `hx = sx*0.5` so `sx,sy,sz` are **full side lengths** (the header comment "half-extents" is wrong) | `box b dx dy dz` | build `create_box(dx,dy,dz)` then **translate by `(dx/2, dy/2, dz/2)`**; for `box b x y z dx dy dz` translate by `(x+dx/2, y+dy/2, z+dz/2)` |
| `BRep::create_cylinder(r,h)` | base circle at `z=0`, top at `z=h`, axis `+Z` | `pcylinder name r h` | **exact 1:1, no offset** |
| `BRep::create_sphere(r)` | centred at origin, poles `±Z` | `psphere name r` | **exact 1:1** |
| `BRep::create_cone(r,h)` | base circle radius `r` at `z=0`, **singular apex at `z=h`** | `pcone name r 0 h` only | **cannot express a frustum.** 49 of the 371 core pairs (64 primitive instances) need `pcone R1 R2 H` with `R2>0`; a `create_cone(r1,r2,h)` overload (or cone∩halfspace) is required before those cells can be ported. |
| `BRep::create_torus(R1,R2)` | centred at origin, axis `+Z` | `ptorus name R1 R2` | **exact 1:1** |

Coverage consequence, measured over the 371 core geometry pairs: **322 pairs (1288 of the 1484 core
cases) are expressible today** with `create_box` + `create_cylinder` + `create_sphere` +
`create_torus` + rigid transforms; **49 pairs (196 cases) additionally require a truncated-cone
constructor**.

---

## 4. HARD CELLS — degenerate / tangent / coincident by construction

Everything in this section is **machine-derived**, not eyeballed. Method: each of the 371 core
operand pairs was reconstructed numerically (primitive params evaluated through the `dset` prelude,
then `ttranslate`/`trotate` applied). Pairs whose rotations are all exact multiples of 90° about a
coordinate axis were placed exactly and classified; the rest are tagged `OBLIQUE` and left
unclassified rather than guessed. The `class` column of Table 2.1 carries the per-case tags.

Tag totals over the 371 core pairs:

| tag | count | meaning |
|---|---|---|
| `OBLIQUE` | 158 | at least one rotation is not a multiple of 90° about a coordinate axis (grazing/slanted slabs, corner-piercing wedges) |
| `BOX-x-QUADRIC` | 101 | box against cylinder/cone/sphere/torus, exactly placeable |
| `COMMON-EMPTY` | 63 | OCCT asserts `bopcommon` yields a **void** result |
| `face-tangent-r@axis0` | 52 | a box face plane is **exactly tangent** to the quadric (distance from axis to plane == radius), X direction |
| `face-tangent-r@axis1` | 50 | same, Y direction |
| `PARALLEL-AXES` | 47 | two quadrics with parallel axes |
| `COAXIAL` | 45 | two quadrics sharing the same axis line |
| `coplanar-faces=N` | 38 | axis-aligned box pair with N coincident face planes |
| `equal-radius(...)` | 26 | a radius of A exactly equals a radius of B |
| `axis-on-face@axis0/1` | 28 / 20 | the quadric's **axis lies exactly in a box face plane** |
| `PERPENDICULAR-AXES` | 25 | two quadrics with orthogonal axes (Steinmetz family) |
| `OVERLAP` / `NESTED` | 16 / 8 | proper box-box overlap / containment |
| `FACE-TOUCH` / `EDGE-TOUCH` / `VERTEX-TOUCH` | 10 / 5 / 1 | box-box **zero-volume** contact |
| `COINCIDENT-SOLIDS` | 1 | A and B occupy exactly the same region (`A1`) |

### 4.1 The highest-value stress cells, by class

**(a) Exactly coincident solids — 1 pair (4 cases).**
`A1` = `box b1 0 0 0 1 1 1` vs `box b2 0 0 0 1 1 1`. Expected: COMMON 6, CUT **empty**, FUSE 6,
TUC **empty**. This is the canonical same-domain test: every face of A is same-domain with a face of
B, and the correct answer for CUT/TUC is nothing at all.

**(b) Box-box with ≥4 coincident face planes — 9 pairs (36 cases):**
`A1, A2, A3, A4, A5, A6, A7, A8, A9`.
Whole rows A/B/E/K of the grid are deliberate coincidence sweeps: one, two, three shared planes,
shared corner, shared edge.

**(c) Box-box pure contact — zero-volume intersection — 16 pairs (64 cases):**
`A3, A4, A6, B2, B3, B4, B6, B8, C2, E3, E4, E7, E8, F2, F3, P6`
(10 face-contact, 5 edge-contact, 1 vertex-contact = `B4`). These are the cases where a naive
`intersect → classify` pipeline invents a zero-thickness sliver solid instead of returning empty.

**(d) Coaxial quadrics with an exactly equal radius (surface-coincident or cap-on-cap) — 24 pairs (96 cases):**

| family | A | B | why degenerate |
|---|---|---|---|
| `ZD4 ZD5 ZD6 ZD7` | `pcylinder b1 1 2` | `pcylinder b2 1 2` at `z=+2` (+ 0/90/180/270° seam spin) | equal-radius coaxial cylinders **stacked cap-on-cap**: the shared plane circle is the entire intersection. COMMON = **empty**, FUSE = 31.4159 |
| `ZH5 ZH6 ZH7 ZH8` | `pcylinder b1 4 8` | `psphere b2 4` at `z=+8` | **sphere equator == cylinder top rim** (tangent circle, not a transversal curve). COMMON = 150.796 |
| `ZK1 ZK2 ZK3 ZK4` | `pcylinder b1 4 8` | `pcone b2 4 2 4` at `z=+8` | frustum base circle == cylinder top rim, **G1-tangent cap stack**. COMMON = **empty** |
| `ZL2 ZL3 ZL4 ZL5` | `pcylinder b1 4 8` | `ptorus b2 4 1` at `z=+4` | torus **centre-line circle lies exactly on the cylinder wall**; the tube straddles the surface, and the intersection crosses the torus seam. COMMON = 116.656 |
| `ZL6 ZL7 ZL8 ZL9` | `pcone b1 8 4 8` | `pcone b2 4 2 4` at `z=+8` | cone-on-cone cap stack, radii match at the shared plane. COMMON = **empty** |
| `ZM7 ZM8 ZM9 ZN1` | `pcone b1 8 4 8` | `pcone b2 4 8 4` at `z=-4` | two frusta meeting base-to-base at a shared circle (**bow-tie**). COMMON = **empty** |

**Seam-invariance oracle.** Within each of the six families above, the four members are the *same*
geometry with B spun about its own axis by 0/90/180/270° — i.e. only the **periodic seam moves**.
Verified: OCCT expects **identical** areas for all four members of every family (e.g. `ZD4..ZD7` all
`common=empty, cut=18.8496, fuse=31.4159, tuc=18.8496`; `ZL6..ZL9` all
`common=empty, cut=588.519, fuse=635.117, tuc=147.13`). Any seam-position dependence in our result
is a bug that these 24 pairs will catch for free.

**(e) Box exactly inscribed on / circumscribed about a quadric — tangent on both transverse axes — 36 pairs (144 cases):**
`R1, R3, R5, R6, R7, R8, R9, S1, S4, S5, S6, S7, T3, T5, U7, U8, V5, V7, V9, W1, W2, W3, W4, W5, W8, W9, X1, X2, X7, X9, Z2, Z3, ZG8, ZG9, ZO9, ZP1`.
Tangency produces a single-point (or single-line) contact between a plane and a cylinder/cone —
the classic SSI degeneracy where the intersection curve degenerates to a point.

**(f) Quadric axis lying exactly IN a box face plane — 40 pairs (160 cases):**
`R8, R9, S1, U4, U5, U6, W3, W4, W5, Y8, Y9, Z1, Z9, ZA6, ZA8, ZA9, ZB1, ZB8, ZC1, ZC2, ZC3, ZC6, ZC7, ZG1, ZG2, ZG3, ZG5, ZG6, ZG7, ZN5, ZN6, ZN7, ZN8, ZN9, ZO1, ZO2, ZO3, ZO4, ZO5, ZP2`.
Half the cylinder is inside the box and half outside, with the split plane passing through the axis —
which is also where the cylinder's seam usually sits.

**(g) Perpendicular-axis quadric pairs (Steinmetz solids) — 25 pairs (100 cases):**
`ZD8, ZE1, ZE3, ZE4, ZE5, ZE6, ZE7, ZE8, ZE9, ZF1, ZH9, ZI1, ZI2, ZI3, ZI4, ZI5, ZI6, ZI7, ZJ3, ZK5, ZK6, ZK7, ZK8, ZK9, ZL1`.
Equal-radius orthogonal cylinders give a **self-intersecting** (figure-eight) intersection curve
through two singular points.

**(h) COMMON asserted empty — 63 pairs (63 cases in `bopcommon_simple`, plus 6 in `bopcut_simple`
and 28 in `boptuc_simple` asserting empty for their own op):**
`A3, A4, A6, B2, B3, B4, B6, B8, C2, D7, E3, E4, E7, E8, F2, F3, I3, M8, N3, N4, N5, N6, N7, N8,
N9, O1, O2, O3, O4, O5, O6, O7, O8, O9, P6, U7, U8, Z2, Z3, ZC8, ZD4, ZD5, ZD6, ZD7, ZF5, ZG8, ZG9,
ZJ4, ZK1, ZK2, ZK3, ZK4, ZL6, ZL7, ZL8, ZL9, ZM1, ZM7, ZM8, ZM9, ZN1, ZO9, ZP1`.
Per-grid empty-assert counts across the whole suite: `bopcommon_simple` 64, `boptuc_simple` 28,
`bopcut_simple` 6, `bopfuse_simple` 0, `bcommon_simple` 11, `bcut_simple` 8, `bfuse_simple` 0.

**(i) Oblique / grazing — 158 pairs (632 cases), tagged `OBLIQUE` in Table 2.1.**
Rows C–D, F–Q are boxes rotated by 45°, ±30°, 60°, 135°, `atan2(1,2)` ≈ 26.5651°, 34.73°, 230°, 245°… about
arbitrary in-plane points. These are near-tangent rather than exactly tangent: the intersection
exists but is ill-conditioned. They are not reconstructible by an exact-arithmetic classifier, which
is precisely why they are good stress cells.

**(j) Seam-vs-seam, outside the core grid.** `bcut_simple/G9` and `bcut_simple/H3` (OCCT bug
`pro13307`): `bfuse` a `pcylinder 9 3` with a `pcone 7 6 4`, then cut a `pcylinder 1 9` translated
`(5,0,-2)` — **in the X direction specifically, so the small cylinder's seam edge meets the cone's
seam**. The case file itself records that translating in Y instead "has no problem". Expected
`-s 727.481`. Highest-value single self-contained seam cell in the suite.

---

## 5. OCCT's OWN KNOWN-BAD CASES (`TODO`) — do NOT use as ground truth

OCCT's test harness treats a line `puts "TODO <bug-id> <platform>: <expected error text>"` as
"this case is *expected to fail* with that message". The stated `checkprops` value in such a case is
what OCCT *wishes* were true, not what OCCT produces.

`grep -rn '^[[:space:]]*puts[[:space:]]*"TODO' tests/boolean/` — **16 files**, complete list:

| file | TODO line (verbatim) | in portable corpus? |
|---|---|---|
| `bopfuse_simple/ZP6` | `puts "TODO CR33225 Linux: Error : The area of result shape is 138625, expected 197700"` | **YES — the 3-orthogonal-tori chained fuse (section 2.2). OCCT produces 138625 on Linux where it asserts 197700. EXCLUDE from ground truth (or treat 138625 as the Linux-actual).** |
| `volumemaker/E5` | `puts "TODO OCC26020 ALL: Error: bopcheck failed"` | no (volumemaker excluded) |
| `bcut_complex/O7` | `puts "TODO #25937 ALL: Faulty shapes in variables faulty_1"` | no (external data) |
| `bcut_complex/O8` | `puts "TODO #25319 ALL: Error : The area of result shape is"` | no |
| `bcut_complex/P6` | `puts "TODO #22911 ALL: Faulty shapes in variables faulty_1 to faulty_"` + `puts "TODO #22911 ALL: Error : The area of result shape is"` | no |
| `bopcut_complex/E2` | `puts "TODO #22911 ALL: Error : The command is not valid. The area is"` | no |
| `bsection/B6` | `puts "TODO #22911 ALL: Error : The command is not valid. The length is"` + `puts "TODO #22911 ALL: Error : The length of result shape is"` | no |
| `removefeatures/G4` | `puts "TODO OCC30099 ALL: Error : The area of result shape is"` + `... volume ...` | no |
| `gdml_private/B5` | `puts "TODO OCC26018 ALL: Tcl Exception:"` + `... is not a valid shape` + `puts "TODO ALL: TEST INCOMPLETE"` | no |
| `gdml_private/I8`, `/W1`, `/X2`, `/ZF4`, `/ZF5`, `/ZG5`, `/ZG7` | `puts "TODO OCC26018 ALL: Error : The area of result shape is"` (one each) | no |

Two further files carry a **commented-out** TODO (historically failing, now fixed — safe to use):
`bcut_complex/O7:2` (`#puts "TODO #22911 ..."`) and `bsection/N4:1` (`#puts "TODO #23749 ..."`).

**Bottom line for the portable corpus: exactly ONE of the 1581 portable cases is marked known-bad by
OCCT — `bopfuse_simple/ZP6`.** The other 1580 are cases OCCT itself passes on every platform, so
their stated `checkprops` values are usable as ground truth (within the 1% relative tolerance and
the `checkshape`-validity requirement documented in 3.4).

---

## 6. RECONSTRUCTION RECIPE (how to consume Table 2.1)

For a core row `<case> | <prelude> | <A> | <B> | c | x | f | t`:

1. Evaluate the `prelude` `dset` expressions (Tcl `expr`: `sqrt`, `atan2`, `pi`).
2. Build A: first token of `<A>` is the primitive (see 3.1 for placement), then apply each
   `;`-separated `ttranslate`/`trotate` **in order**, baking into geometry (see 3.2).
3. Build B the same way.
4. Run the four ops. `bop A B` + `bopcommon` = A ∩ B; `bopcut` = A − B; `bopfuse` = A ∪ B;
   `boptuc` = **B − A** (OCCT `BOPAlgo_CUT21`).
5. Assert `area(result)` against `c/x/f/t` with **1% relative** tolerance; `empty` means the result
   must contain nothing (`Mass == 0`).
6. Additionally assert the result is a valid shape (the grid's `end` file does `checkshape result`
   unconditionally).

Case ids for a row `<case>` are exactly
`boolean/bopcommon_simple/<case>`, `boolean/bopcut_simple/<case>`,
`boolean/bopfuse_simple/<case>`, `boolean/boptuc_simple/<case>`.
