# OCCT regression suite mined for BOOLEAN failure cases

Source tree: `/home/petras/code/code_cpp/OCCT`, git `37dd5686f216ede6bef5b33d1d5d6650bde9ec93`
(`V8_0_0_p1-5-g37dd5686f2`, 2026-07-13, "Coding - Bump version to 8.0.1.dev").
Mined directory: `tests/bugs/` (4270 files total; the eight `modalg_*` grids hold 2565 of them).

**Method (so every number below is reproducible).** A file counts as a *boolean case* if it
contains, as a command word, any of
`bcut bcommon bfuse bopcut bopcommon bopfuse bopcheck bfillds bbuild bsection bopsection`
(word-boundary regex `(^|[^a-zA-Z0-9_])CMD([^a-zA-Z0-9_]|$)`). A boolean case is *external-data*
if it also contains any of `restore stepread igesread brestore locate_data_file ReadStep ReadIges
testreadstep testreadiges readstl binrestore bmrestore XLoad meshfromstl stlread vrmlread readbrep`;
otherwise it is *self-contained* (built only from DRAW procedural primitives). Counts were produced
with `grep -rlE` + `comm` under `LC_ALL=C`; taxonomy assignment (section 3) is a manual one-class-per-case
reading of all 237 self-contained files, published in the `Tax` column of section 2 so it is auditable.

---

## 1. CENSUS

### 1.1 Boolean cases in `tests/bugs`

| | count |
|---|---|
| Files in `tests/bugs` | 4270 |
| **Boolean cases (any of the 11 commands)** | **1252** |
| — self-contained (procedural primitives only) | **237** |
| — requiring an external data file | **1015** |

Alternative split using only the four loaders the task named (`restore` / `stepread` /
`igesread` / `brestore`): **993 external, 259 self-contained**. The 22-case difference is made of
files that reach for data through `locate_data_file` + a different loader (`binrestore`, `ReadStep`,
`ReadIges`, `testreadstep`): `iges/bug26047`, `modalg_5/bug24140`, `modalg_5/bug24746`,
`modalg_5/bug25477_2`, `modalg_6/bug26576_2`, `modalg_6/bug27888`, `modalg_6/bug28002_1`,
`modalg_7/bug27973`, `modalg_7/bug28892_1..3`, `modalg_7/bug28893_1..4`, `modalg_7/bug29182`,
`modalg_7/bug29322_1`, `modalg_7/bug29711`, `modalg_7/bug30590_1..3`, `step/bug32556`.
The wider definition (1015 / 237) is used everywhere below.

Verification of purity: `grep -lniE 'locate_data_file|\.brep|\.stp|\.step|\.igs|\.iges|\.rle|\.bin|CSF_TestDataPath'`
over the 237 self-contained files returns **zero** matches.

### 1.2 By grid

| grid | boolean cases | self-contained | external-data |
|---|---|---|---|
| modalg_7 | 257 | 61 | 196 |
| modalg_5 | 223 | 55 | 168 |
| modalg_6 | 222 | 40 | 182 |
| modalg_2 | 160 | 19 | 141 |
| modalg_1 | 141 | 13 | 128 |
| modalg_4 | 100 | 4 | 96 |
| moddata_2 | 42 | 7 | 35 |
| moddata_1 | 31 | 2 | 29 |
| modalg_8 | 22 | 9 | 13 |
| heal | 21 | 9 | 12 |
| moddata_3 | 15 | 7 | 8 |
| mesh | 5 | 5 | 0 |
| fclasses | 5 | 5 | 0 |
| modalg_3 | 4 | 0 | 4 |
| step | 2 | 0 | 2 |
| iges | 1 | 0 | 1 |
| caf | 1 | 1 | 0 |
| **total** | **1252** | **237** | **1015** |

### 1.3 Boolean cases elsewhere in `tests/` (same regex, for scale)

`boolean` 3059 · `bugs` 1252 · `draft` 72 · `perf` 38 · `mkface` 38 · `lowalgos` 35 · `feat` 24 ·
`offset` 19 · `blend` 13 · `heal` 11 · `pipe` 6 · `v3d` 4 · `opengl` 3 · `hlr` 2 · `evolved` 2 ·
`mesh` 1 · `de_mesh` 1 · `cr` 1 · `caf` 1.
The dedicated `tests/boolean` grid is 2.4x larger than the whole bug corpus, but `tests/bugs` is
the *historical-defect* corpus — every entry is a shipped bug someone reported.

### 1.4 Command mix across the 1252 boolean bug cases (files containing the command)

`bop` 377 · `bfillds` 339 · `bbuild` 183 · `bfuse` 174 · `bcut` 164 · `bbop` 160 ·
`bopsection` 139 · `bsection` 135 · `bopcheck` 120 · `bopfuse` 98 · `bopcommon` 89 ·
`bcommon` 79 · `bopcut` 79 · `bopargcheck` 48 · `bfuzzyvalue` 33 · `bsplit` 23 ·
`brunparallel` 15 · `bnondestructive` 14 · `bglue` 10 · `bcbuild` 8 · `boptuc` 5 · `btuc` 2.

Assertion mix across the same 1252: `checkview` 1060 · `checkshape` 926 · `checkprops` 920 ·
`checknbshapes` 724 · `checksection` 218 · `bopcheck` 120 · `checkmaxtol` 62 · `bopargcheck` 48 ·
`checkreal` 19 · `tricheck` 14 · `checklength` 2.

### 1.5 What the 237 self-contained cases are made of

Primitive/constructor frequency (cases using the command):
`box` 126 · `explode` 90 · `ttranslate` 63 · `vertex` 51 · `pcylinder` 46 · `mkface` 45 ·
`mkedge` 42 · `trotate` 40 · `plane` 37 · `wire` 36 · `mkplane` 35 · `psphere` 32 · `edge` 27 ·
`circle` 24 · `prism` 16 · `polyline` 16 · `revol` 16 · `compound` 16 · `pcone` 15 · `line` 13 ·
`cylinder` 12 · `trim` 11 · `ellipse` 11 · `unifysamedom` 11 · `halfspace` 10 · `nurbsconvert` 9 ·
`settolerance` 8 · `ptorus` 7 · `trimv` 7 · `mksurface` 7 · `sewing` 5.

Boolean-command frequency (cases): `baddobjects` 75 · `bfillds` 72 · `bop` 62 · `baddtools` 47 ·
`bcut` 41 · `bfuse` 41 · `bbuild` 40 · `bbop` 27 · `bopsection` 21 · `bcommon` 21 · `bopfuse` 17 ·
`bsection` 15 · `bopcut` 14 · `bopcommon` 13 · `bsplit` 11 · `baddcompound` 9 · `bfuzzyvalue` 7 ·
`brunparallel` 6 · `bnondestructive` 5 · `bcbuild` 4 · `bopcheck` 3 · `bcadd` 3 · `boptuc` 2 ·
`bglue` 2 · `bcremoveall` 2 · `bopargcheck` 1 · `bcaddall` 1 · `bcremoveint` 1 · `bapibop` 1 · `btuc` 1.

Assertion frequency (cases): `checkview` 148 · `checknbshapes` 135 · `checkshape` 132 ·
`checkprops` 106 · `checkmaxtol` 27 · `bopcheck` 21 · `checksection` 19 · `checktrinfo` 8 ·
`tricheck` 8 · `bopargcheck` 6 · `checktrend` 5 · `checkreal` 3 · `checklength` 2.
Of the 106 `checkprops` users: 68 pin an exact **area** (`-s`), 34 an exact **volume** (`-v`),
27 an exact **length** (`-l`).

**Read of 1.5**: the whole self-contained corpus is boxes, cylinders, spheres, cones, tori,
planes, prisms and revolutions — exactly the primitive vocabulary we already have. `box` alone
appears in 126 of the 237 cases; `pcylinder` 46, `psphere` 32, `pcone` 15, `ptorus` 7. No file
I/O, no importer, no `.brep` reader is needed to run any of it.

---

## 2. THE 237 SELF-CONTAINED BOOLEAN BUG CASES

Columns: exact DRAW construction commands (truncated at ~300 chars with a `...[N cmds]` marker
when the case is a long build script), the boolean call(s), the assertion(s), the one-line defect
description taken verbatim from a comment or `puts` header in the file, and the taxonomy code used
in section 3. `**[TODO]**` marks a case that carries a `TODO` line (OCCT itself still fails it —
see section 4).

| # | Case (tests/bugs/…) | Geometry built (exact DRAW commands) | Boolean operation | Assertion | Defect class stated in the file | Tax |
|---|---|---|---|---|---|---|
| 1 | `caf/bug31918_2` | `box b1 0 0 0 [expr $nx + .5] [expr $ny + .5] 1 ; box b2 0.5 0.5 0 [expr $nx - .5] [expr $nx - .5] 0.4 ; pcylinder c${x}_$y 0.25 1.01 ; ttranslate c${x}_$y [expr $x+.75] [expr $y+.75] 0.39 ; ttranslate part $dx $dy $dz ; store_part 16 16 0 0 0 0:1:1 ; store_part 4 4 $n $n $n 0:2:$n ; ...[12 cmds]` | `bcut base b1 b2 ; bop base cc ; bopfuse part` | `checkshape opened_top2 ; checkshape s ; checkshape s ; checkshape top3` | Application Framework - New binary format for fast reading part of OCAF document | DOWNSTREAM |
| 2 | `fclasses/bug7287_1` **[TODO]** | `box b1 100 100 100 ; box b2 50 50 50` | `bop b1 b2 ; bopcut r ; bop b1 b2 ; bopcut r` | `checktrend $listmem 0 10 "Memory leak detected"` | Problem of Memory Leak | LEAK |
| 3 | `fclasses/bug7287_2` **[TODO]** | `box b1 10 10 10 100 100 100 ; box b2 50 50 50` | `bop b1 b2 ; bopcut r` | `checktrend $listmem 0 100 "Memory leak detected"` | Problem of Memory Leak | LEAK |
| 4 | `fclasses/bug7287_4` **[TODO]** | `box b1 10 10 10 100 100 100 ; box b2 50 50 50` | `bop b1 b2 ; bopcommon r` | `checktrend $listmem 0 100 "Memory leak detected"` | Problem of Memory Leak | LEAK |
| 5 | `fclasses/bug7287_5` **[TODO]** | `box b1 10 10 10 100 100 100 ; box b2 50 50 50` | `bop b1 b2 ; bopfuse r` | `checktrend $listmem 0 100 "Memory leak detected"` | Problem of Memory Leak | LEAK |
| 6 | `fclasses/bug7287_6` **[TODO]** | `box b1 10 10 10 100 100 100 ; box b2 50 50 50` | `bop b1 b2 ; bopsection r` | `checktrend $listmem 0 100 "Memory leak detected"` | Problem of Memory Leak | LEAK |
| 7 | `heal/bug26219_gehause_rohteil` | `point p1  0  0 ; point p2  35 0 ; point p3  39 4 ; point p4  39 10 ; point p5  35 13 ; point p6  27 13 ; point p7  27 10 ; point p8  22 10 ; point p9  20 8 ; point p10 20 4 ; point p11 11 4 ; point p12 11 3 ; point p13 5  3 ; point p14 5  13 ; point p15 0  13 ; point p16 0  9 ; point p17 4  9 ; ...[223 cmds]` | `bcut f f f1 ; bfuse p p ptop ; bcut p p tool ; bcut ptube ptube_outer ptube_inner ; bfuse p p ptube ; bfuse p p t1 ; bfuse p p t2 ; bfuse p p t1 ; bfuse p p t2 ; bcut p p blend_box ; ...[13 cmds]` | `checknbshapes p -ref ${nbshapes_before_simplify} -t -m "result before attempt to simplify the model" ; ...[2 cmds]` | The following example constructs (however, not trying to follow the drawings | SAMEDOMAIN |
| 8 | `heal/bug26244` | `box _platform $aPlatformLength $aPlatformWidth $aPlatformHeight ; pcylinder _screwhole $aScrewHoleRadius $aPlatformHeight ; ttranslate _screwhole $aScrewHoleOffset $aScrewHoleOffset 0 ; ttranslate _screwhole [expr $aPlatformLength - $aScrewHoleOffset] $aScrewHoleOffset 0 ; ...[123 cmds]` | `bcut _platform _platform _screwhole ; bcut _platform _platform _screwhole ; bcut _platform _platform _screwhole ; bcut _platform _platform _screwhole ; bfuse _backcave t_s1 t_s2 ; ...[23 cmds]` | `checkshape res ; checktrinfo r -defl 0.049761016978299343 -tol_abs_defl 0.01 -tol_rel_defl 0.01` | Destructive results of simplification with DRAW command 'unifysamedom' after intersection of two complex models | SAMEDOMAIN |
| 9 | `heal/bug29382_3` | `beziercurve a 5  0 0 0  1 0 0  2 2 0  0 0.5 0  0 0 0 ; mkedge a a ; wire a a ; mkplane a a ; prism a a 0 0 1 ; box b -0.3 -0.2 0  1 0.4 1 ; unifysamedom result shape +b` | `bcommon shape a b` | `checkshape result ; checknbshapes result -solid 1 -shell 1 -face 5 -wire 5 -edge 9 -vertex 5` | ShapeUpgrade_UnifySameDomain algorithm incorrectly processes the shape | SAMEDOMAIN |
| 10 | `heal/bug29502` | `cylinder c 0 0 0 0 0 1 10 ; mkface f c 0 2*pi -10 10 ; vertex v 10 0 0 ; unifysamedom result r` | `bclearobjects ; bcleartools ; baddobjects f ; baddtools v ; bfillds ; bsplit r` | `checknbshapes r -vertex 3 -edge 4 -wire 1 -face 1 ; checkshape result ; checkprops result -equal f ; checknbshapes result -ref [nbshapes f]` | Improve performance of the ShapeUpgrade_UnifySameDomain::UnifyEdges() method | SEAM |
| 11 | `heal/bug29544_2` | `line l 0 0 0 1 0 0 ; mkedge e l 0 10 ; vertex v 5 2.e-7 0 ; unifysamedom result sp` | `bclearobjects ; bcleartools ; baddobjects e ; baddtools v ; bfillds ; bsplit sp` | `checkshape result ; checkprops result -l 10 ; checknbshapes result -vertex 2 -edge 1` | Regression vs 7.2.0: ShapeUpgrade_UnifySameDomain fails to merge linear edges | TOL |
| 12 | `heal/bug30927` | `box b1 10 10 10 ; box b2 5 0 -5 10 10 20 ; unifysamedom result s` | `bfuse s b1 b2` | `-` | Modeling Algorithms - UnifySameDom looses the Closed flag for wires | CONTAINER |
| 13 | `heal/bug33171_1` | `polyline p 0 0 0  10 0 0  10 10 0  0 10 0  0 0 0 ; mkplane f p ; prism s f 0 0 5 ; polyline p1 3 10 0  3 7 0  6 7 0  6 3 0  10 3 0 ; polyline p2 6 7 0  10 7 0 ; polyline p3 8 7 0  8 10 0 ; polyline p4 0 5 0  10 5 0 ; prism sh1 p1 0 0 5 ; prism sh2 p2 0 0 5 ; prism sh3 p3 0 0 5 ; ...[15 cmds]` | `bclearobjects ; bcleartools ; baddobjects s ; baddtools sh1 sh2 sh3 sh4 ; bfillds ; bsplit r` | `checkshape r ; checkshape ru1 ; checkshape ru2 ; checknbshapes ru1 -ref [nbshapes r -t] -t ; checknbshapes ru2 -ref [nbshapes r -t] -t ; checkshape shsu1 ; checkshape shsu2 ; ...[9 cmds]` |  	0033171: Modeling Algorithms - Invalid result of faces unification | SAMEDOMAIN |
| 14 | `heal/bug33171_2` | `box b1 10 10 5 ; box b2 10 0 0 5 5 5 ; unifysamedom ru1 r ; unifysamedom ru2 r +i ; unifysamedom shsu1 r ; unifysamedom shsu2 r +i` | `bclearobjects ; bcleartools ; baddobjects b1 b2 ; bfillds ; bbuild r` | `checkshape r ; checkshape ru1 ; checkshape ru2 ; checknbshapes ru1 -ref [nbshapes r -t] -t ; checknbshapes ru2 -ref [nbshapes r -t] -t ; checkshape shsu1 ; checkshape shsu2 ; ...[9 cmds]` |  	0033171: Modeling Algorithms - Invalid result of faces unification | SAMEDOMAIN |
| 15 | `heal/bug33421` **[TODO]** | `pcylinder c1 10 10 ; tmirror c2 0 0 10 0 0 1 ; unifysamedom result c3` | `bop c1 c2 ; bopfuse c3` | `checknbshapes result -t -solid 1 -shell 1 -face 3 -wire 3 -edge 3 -vertex 3` | Modeling Algorithms - ShapeUpgrade_UnifySameDomain fails | SAMEDOMAIN |
| 16 | `mesh/bug25142` | `box b1 20 10 10 ; box b2 10 10 10 ; explode r ; incmesh r_1 0.1 ; incmesh r_2 0.1 ; tclean r` | `bclear ; baddobjects b1 b2 ; bfillds ; bbuild r ; bopargcheck r #F ; bopargcheck r_1 r_2 -F #F` | `checkshape r ; tricheck r_1 ; tricheck r_2 ; tricheck r_1 ; tricheck r_1` | Visualization breaks triangulation on shared solids in composite solid model | DOWNSTREAM |
| 17 | `mesh/bug25157` | `pcone Cone 6 0 10 180 ; pcylinder Cylinder 1 6 360 ; ttranslate Cylinder 0 3 1 ; explode Cut F ; tclean Cut_1 ; incmesh Cut_1 0.1` | `bcut Cut Cone Cylinder` | `checktrinfo Cut_1 -tri -nod` | Face missing depending the angle of revolution | DOWNSTREAM |
| 18 | `mesh/bug29751` | `psphere sp 10 ; pcylinder b1 2 10 ; ttranslate b1 0 0 -10 ; trotate result 0 0 0 1 0 0 45 ; incmesh result 0.1` | `bcut result sp b1` | `checktrinfo result -tri 1013 -nod 578 -defl 0.1164052220738387 -tol_abs_defl 1e-6` | Incremental mesh produces different meshes for windows and linux (debian 8) | DOWNSTREAM |
| 19 | `mesh/bug30008_2` | `pcylinder cy 100 500 ; ...[9 cmds]` | `bcut rc cy hs` | `tricheck result ; checktrinfo result -tri 193 -nod 147 -defl 0.052300780129031083 -tol_abs_defl 1.0e-6` | BRepMesh does not respect angular deflection in internal area of bspline surface | DOWNSTREAM |
| 20 | `mesh/bug31926` | `vertex v11 0 1 0 ; vertex v12 1 0 0 ; vertex v13 0 0 0 ; edge e11 v11 v12 ; edge e12 v12 v13 ; edge e13 v13 v11 ; wire w1 e11 e12 e13 ; mkplane f1 w1 ; vertex v21 0 0 2 ; vertex v22 1 0 0 ; vertex v23 0 0 0 ; edge e21 v21 v22 ; edge e22 v22 v23 ; edge e23 v23 v21 ; wire w2 e21 e22 e23 ; ...[36 cmds]` | `bcut result s1 s2` | `-` | Shape Healing - ShapeAnalysis::OuterWire() considers next iteration element always to be a wire causing skipping of primal one | DOWNSTREAM |
| 21 | `modalg_1/buc60668` | `box b 100 0 0 10 10 10 ; box b1 10 10 10 ; box b2 50 50 50 10 10 10 ; box a -10 -10 -10 200 200 200 ; compound b b1 b2 c` | `bcommon result c a` | `checkprops result -s 1800 ; checkshape result` | Three boxes must appear on picture | WRONGRESULT |
| 22 | `modalg_1/buc60801` | `box b -10 -10 -10 30 40 40 ; box a1 -20 0 -20 20 20 100 ; box a2 0 20 -20 50 20 100 ; box c -20 -20 20 100 100 100` | `bcut r1 b a1 ; bcut r2 r1 a2 ; bcut r3 r2 c` | `checkshape r3 ; checkprops result -s 6400 ; checkshape result` | (no comment in file) | WRONGRESULT |
| 23 | `modalg_1/buc60899_1` | `circle c $xc 0 $zc $r ; mkedge c c ; prism cyl c 0 0 -$zc*2 inf ; prism crg w 0 -$y*2 0 inf` | `bsection result crg cyl` | `checkprops result -l 110.093 ; checkshape result ; checksection result` | Execution of the script prism_sec.tcl gives  exception "NumericError". | CRASH |
| 24 | `modalg_1/buc60899_2` | `polyline c $xc-$r -$r*2 $zc $xc+$r -$r*2 $zc $xc+$r $r*2 $zc $xc-$r $r*2 $zc $xc-$r -$r*2 $zc ; prism cyl c 0 0 -$zc*2 inf ; prism crg w 0 -$y*2 0 inf` | `bsection result crg cyl` | `checkprops result -l 162.299 ; checkshape result ; checksection result` | Execution of the script prism_sec.tcl gives  exception "NumericError". | EMPTY |
| 25 | `modalg_1/buc60901` | `circle c $xc $yc $zc $nxc $nyc $nzc $r ; mkedge c c ; prism cyl c 0 0 $lcyl ; prism crg w 0 $lcrg 0` | `bsection result crg cyl` | `checkprops result -l 350.404 ; checkshape result ; checksection result` | We should get the edges forming a closed intersection line but we see breaks. | OPENSECTION |
| 26 | `modalg_1/bug12507` | `pcylinder c 3 3 ; explode c e ; tcopy c_3 e3 ; ttranslate e3 0 6 0` | `bop c_3 e3 ; bopfuse result` | `checknbshapes result -vertex 3 -edge 4 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 8` | Wrong result of fuse operation | WRONGRESULT |
| 27 | `modalg_1/bug13211_2` | `vertex v1 136.06, 187.51, 0.0 ; vertex v2 136.06, 143.01, 0.0 ; edge e1 v1 v2 ; vertex v3 136.06, 143.01, 0.0 ; vertex v4 128.08, 143.01, 0.0 ; edge e2 v3 v4 ; vertex v5 128.08, 143.01, 0.0 ; vertex v6 127.88, 164.75, 0.0 ; edge e3 v5 v6 ; vertex v7 127.88, 164.75, 0.0 ; ...[19 cmds]` | `bop r1 r2 ; bopfuse result` | `checkprops result -s 17308.3 ; checkshape result ; checknbshapes result -vertex 13 -edge 25 -wire 13 -face 12 -shell 1 -solid 1 -compsolid 0 -compound 1 -shape 66` | Wrong treatment of conical faces in BOP algorithm | TANGENT |
| 28 | `modalg_1/bug13211_3` | `vertex v1 136.06, 187.51, 0.0 ; vertex v2 136.06, 143.01, 0.0 ; edge e1 v1 v2 ; vertex v3 136.06, 143.01, 0.0 ; vertex v4 128.08, 143.01, 0.0 ; edge e2 v3 v4 ; vertex v5 128.08, 143.01, 0.0 ; vertex v6 127.88, 164.75, 0.0 ; edge e3 v5 v6 ; vertex v7 127.88, 164.75, 0.0 ; ...[19 cmds]` | `bop r1 r2 ; bopcommon result` | `checkprops result -s 2990.42 ; checkshape result ; checknbshapes result -vertex 13 -edge 23 -wire 10 -face 10 -shell 1 -solid 1 -compsolid 0 -compound 1 -shape 59` | Wrong treatment of conical faces in BOP algorithm | TANGENT |
| 29 | `modalg_1/bug13211_4` | `vertex v1 136.06, 187.51, 0.0 ; vertex v2 136.06, 143.01, 0.0 ; edge e1 v1 v2 ; vertex v3 136.06, 143.01, 0.0 ; vertex v4 128.08, 143.01, 0.0 ; edge e2 v3 v4 ; vertex v5 128.08, 143.01, 0.0 ; vertex v6 127.88, 164.75, 0.0 ; edge e3 v5 v6 ; vertex v7 127.88, 164.75, 0.0 ; ...[19 cmds]` | `bop r1 r2 ; bopcut result` | `checkprops result -s 17292.5 ; checkshape result ; checknbshapes result -vertex 13 -edge 26 -wire 12 -face 11 -shell 1 -solid 1 -compsolid 0 -compound 1 -shape 65` | Wrong treatment of conical faces in BOP algorithm | TANGENT |
| 30 | `modalg_1/bug13211_6` | `vertex v1 136.06, 187.51, 0.0 ; vertex v2 136.06, 143.01, 0.0 ; edge e1 v1 v2 ; vertex v3 136.06, 143.01, 0.0 ; vertex v4 128.08, 143.01, 0.0 ; edge e2 v3 v4 ; vertex v5 128.08, 143.01, 0.0 ; vertex v6 127.88, 164.75, 0.0 ; edge e3 v5 v6 ; vertex v7 127.88, 164.75, 0.0 ; ...[19 cmds]` | `bop r1 r2 ; bopsection result` | `checkprops result -l 228.699 ; checkshape result ; checksection result ; checknbshapes result -vertex 13 -edge 15 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 29` | Wrong treatment of conical faces in BOP algorithm | TANGENT |
| 31 | `modalg_1/bug13538` | `box a 100 100 100 ; box b 100 100 100 ; ttranslate b 0 0 100 ; explode a Sh ; explode b Sh` | `bop a_1 b_1 ; bopcommon result` | `checkprops result -s 10000 ; checkshape result ; checknbshapes result -vertex 4 -edge 4 -wire 1 -face 1 -shell 1 -solid 0 -compsolid 0 -compound 1 -shape 12` | Problem with Boolean operation on Shells | SAMEDOMAIN |
| 32 | `modalg_1/bug17194_1` | `circle f 0 0 0 0 0 1 1 0 0 20 ; mkedge f f -pi/2 pi/2 ; prism f f 50 0 30 ; plane pl 10 0 0 1 0 4 ; mkface pl pl` | `bsection result f pl` | `checknbshapes result -vertex 4 -edge 2 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 7 ; checkprops result -l 15.8064 ; checkshape result ; checksection result` | Incorrect section of a face got by extrusion of an arc of circle | WRONGRESULT |
| 33 | `modalg_1/bug17194_2` | `circle f 0 0 0 0 0 1 1 0 0 20 ; mkedge f f -pi/2 pi/2 ; prism f f 50 0 30 ; plane pl 10 0 10 1 0 4 ; mkface pl pl` | `bsection result f pl` | `checknbshapes result -vertex 2 -edge 1 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 4 ; checkprops result -l 54.6122 ; checkshape result ; checksection result` | Incorrect section of a face got by extrusion of an arc of circle | WRONGRESULT |
| 34 | `modalg_2/bug22990` | `psphere s 10 ; plane p 2 0 0 1 0 0 ; mkface f p -100 100 -100 100 ; explode s f ; explode e e ; splitshape result s s_1 e_1 e_2` | `bsection e s f -na` | `checknbshapes result -face 3 ; checkprops result -s 1256.64 ; checkshape result` | Regression vs 6.5.2: splitting sphere across the seam is incomplete | SEAM |
| 35 | `modalg_2/bug2785_1` | `pcylinder c1 2 10 ; pcylinder c2 2 2 ; explode c1 f ; explode c2 f ; mksurface gs c2_1 ; trim trgs gs 1 2 2 4 ; mkface ff trgs ; trim trgs gs 0 1 2 4 ; mkface ff2 trgs` | `bcut rcut1 c1_1 ff ; bop ff2 rcut1 ; boptuc rcut2 ; bfuse fuse1 rcut2 ff2 ; bfuse result fuse1 ff` | `checkshape rcut1 ; checkshape rcut2 ; checkshape fuse1 ; checkprops result -s 125.664 ; checkshape result` | Cut and fuse operations for two faces that share same domain give invalid result | SAMEDOMAIN |
| 36 | `modalg_2/bug2785_2` | `pcylinder c1 2 10 ; pcylinder c2 2 2 ; explode c1 f ; explode c2 f ; mksurface gs c2_1 ; trim trgs gs 1 2 2 4 ; mkface ff trgs ; trim trgs gs 0 1 2 4 ; mkface ff2 trgs` | `bcut rcut1 c1_1 ff ; bcut rcut2 rcut1 ff2 ; bfuse fuse1 rcut2 ff2 ; bfuse result fuse1 ff` | `checkshape rcut1 ; checkshape rcut2 ; checkshape fuse1 ; checkprops result -s 125.664 ; checkshape result` | Cut and fuse operations for two faces that share same domain give invalid result | SAMEDOMAIN |
| 37 | `modalg_2/bug297_1` | `vertex v1  250  250 0 ; vertex v2 -250  250 0 ; vertex v3 -250 -250 0 ; vertex v4  250 -250 0 ; edge e1 v1 v2 ; edge e2 v2 v3 ; edge e3 v3 v4 ; edge e4 v4 v1 ; wire w1 e1 e2 e3 e4 ; mkplane f w1 ; halfspace hs f $x $y $Zpoint ; box b 0 0 $Zbox 150 200 200` | `bcut result b hs` | `checkprops result -s 179000 ; checkshape result` | (no comment in file) | HALFSPACE |
| 38 | `modalg_2/bug297_2` | `vertex v1  250  250 0 ; vertex v2 -250  250 0 ; vertex v3 -250 -250 0 ; vertex v4  250 -250 0 ; edge e1 v1 v2 ; edge e2 v2 v3 ; edge e3 v3 v4 ; edge e4 v4 v1 ; wire w1 e1 e2 e3 e4 ; mkplane f w1 ; halfspace hs f $x $y $Zpoint ; box b 0 0 $Zbox 150 200 200` | `bcut result b hs` | `checkprops result -s 81000 ; checkshape result` | (no comment in file) | HALFSPACE |
| 39 | `modalg_2/bug297_3` | `vertex v1  250  250 0 ; vertex v2 -250  250 0 ; vertex v3 -250 -250 0 ; vertex v4  250 -250 0 ; edge e1 v1 v2 ; edge e2 v2 v3 ; edge e3 v3 v4 ; edge e4 v4 v1 ; wire w1 e1 e2 e3 e4 ; mkplane f w1 ; halfspace hs f $x $y $Zpoint ; box b 0 0 $Zbox 150 200 200` | `bcut result b hs` | `checkprops result -s 144000 ; checkshape result` | (no comment in file) | HALFSPACE |
| 40 | `modalg_2/bug297_4` | `vertex v1  250  250 0 ; vertex v2 -250  250 0 ; vertex v3 -250 -250 0 ; vertex v4  250 -250 0 ; edge e1 v1 v2 ; edge e2 v2 v3 ; edge e3 v3 v4 ; edge e4 v4 v1 ; wire w1 e1 e2 e3 e4 ; mkplane f w1 ; halfspace hs f $x $y $Zpoint ; box b 0 0 $Zbox 150 200 200` | `bcut result b hs` | `checkprops result -s 116000 ; checkshape result` | (no comment in file) | HALFSPACE |
| 41 | `modalg_2/bug2986_1` | `ptorus p1 10 4 ; explode p1 f ; mksurface gs p1_1 ; trim trgs gs 1 2 1 2 ; mkface ff trgs` | `bop p1_1 ff ; bopcut rcut1 ; bfuse result rcut1 ff` | `checkshape rcut1 ; checkprops result -s 1579.81 ; checkshape result` | Cut&fuse oprs. for 2 toroidal faces, sharing the same domain give invalid result | SAMEDOMAIN |
| 42 | `modalg_2/bug2986_2` | `ptorus p1 10 4 ; explode p1 f ; mksurface gs p1_1 ; trim trgs gs 1 2 1 2 ; mkface ff trgs` | `bcut rcut1 p1_1 ff ; bfuse result rcut1 ff` | `checkshape rcut1 ; checkprops result -s 1579.81 ; checkshape result` | Cut&fuse oprs. for 2 toroidal faces, sharing the same domain give invalid result | SAMEDOMAIN |
| 43 | `modalg_2/bug407_1` | `pcylinder b1 1 4 ; pcylinder b2 1 4 ; trotate b2 0 0 2 1 0 0 90 ; trotate b2 0 0 2 0 1 0 60` | `bfuse result  b2 b1` | `checkshape -top result ; checkprops result -s 46.8319 ; checkshape result` | (no comment in file) | WRONGRESULT |
| 44 | `modalg_2/bug407_2` | `pcylinder b1 1 4 ; pcylinder b2 1 4 ; trotate b2 0 0 2 1 0 0 90 ; trotate b2 0 0 2 0 1 0 60` | `bcut result b2 b1` | `checkshape -top result ; checkprops result -s 31.4159 ; checkshape result` | (no comment in file) | WRONGRESULT |
| 45 | `modalg_2/bug422_1` | `plane p 0 0 0 0 0 1 -1 0 0 ; psphere p p 20 0 90 ; profile pf o -4 -4 1 l 8 c 2 90 l 8 d -1 0 l 8 ; prism p2 pf 0 0 40 ; nexplode f f ; depouille result f 0 0 1 f_7 5 0 0 40 0 0 1` | `bfuse f p p2` | `checkprops result -s 4630.78 ; checkshape result` | profile pf o -4 -4 1 l 8 d 0 1 l 8 d -1 0 l 8 | DOWNSTREAM |
| 46 | `modalg_2/bug422_2` | `plane ps 10 -3 0  1 0 0  0 .2 1 ; psphere ps ps 20 ; profile pf o 5 1 5 l 10 c 2 90 l 5 d -1 0 \ ; l 14 d 0 -1 l 5 c 2 90 ; prism pr pf 0 0 30 ; nexplode f f ; depouille result f 0 0 1 f_4 4 0 0 30 0 0 1` | `bfuse f ps pr` | `checkprops result -s 5719.09 ; checkshape result` | (no comment in file) | DOWNSTREAM |
| 47 | `modalg_2/bug4717_1` **[TODO]** | `box b -20 -20 -20 40 40 20 ; ellipse w1 0 0 0 15 10 ; mkedge w1 w1 -pi/2 pi/2 ; trotate w1 0 0 0 1 0 0 90 ; wire w w1 ; revol r w 0 0 0 0 0 1 360 ; shape s So ; trotate s b 0 0 0 1 1 0 -40 ; trotate s b 0 0 0 0 0 1 -10 ; tscale b 0 0 0 SCALE1 ; tscale s 0 0 0 SCALE1` | `bop s b ; bopfuse result` | `checkprops result -s 680565 ; checkshape result ; checknbshapes result -vertex 11 -edge 16 -wire 8 -face 7 -shell 1 -solid 1 -compsolid 0 -compound 1 -shape 45` | A regression of new Boolean Operations | WRONGRESULT |
| 48 | `modalg_2/bug4717_2` **[TODO]** | `box b -20 -20 -20 40 40 20 ; ellipse w1 0 0 0 15 10 ; mkedge w1 w1 -pi/2 pi/2 ; trotate w1 0 0 0 1 0 0 90 ; wire w w1 ; revol r w 0 0 0 0 0 1 360 ; shape s So ; trotate s b 0 0 0 1 1 0 -40 ; trotate s b 0 0 0 0 0 1 -10 ; tscale b 0 0 0 SCALE1 ; tscale s 0 0 0 SCALE1` | `bop b s ; bopfuse result` | `checkprops result -s 680565 ; checkshape result ; checknbshapes result -vertex 11 -edge 16 -wire 8 -face 7 -shell 1 -solid 1 -compsolid 0 -compound 1 -shape 45` | A regression of new Boolean Operations | WRONGRESULT |
| 49 | `modalg_2/bug4717_3` **[TODO]** | `box b -20 -20 -20 40 40 20 ; ellipse w1 0 0 0 15 10 ; mkedge w1 w1 -pi/2 pi/2 ; trotate w1 0 0 0 1 0 0 90 ; wire w w1 ; revol r w 0 0 0 0 0 1 360 ; shape s So ; trotate s b 0 0 0 1 1 0 -40 ; trotate s b 0 0 0 0 0 1 -10 ; tscale b 0 0 0 SCALE1 ; tscale s 0 0 0 SCALE1` | `bop s b ; bopcut result` | `checkprops result -s 181937 ; checkshape result ; checknbshapes result -vertex 3 -edge 4 -wire 2 -face 2 -shell 1 -solid 1 -compsolid 0 -compound 1 -shape 14` | A regression of new Boolean Operations | WRONGRESULT |
| 50 | `modalg_2/bug4717_4` **[TODO]** | `box b -20 -20 -20 40 40 20 ; ellipse w1 0 0 0 15 10 ; mkedge w1 w1 -pi/2 pi/2 ; trotate w1 0 0 0 1 0 0 90 ; wire w w1 ; revol r w 0 0 0 0 0 1 360 ; shape s So ; trotate s b 0 0 0 1 1 0 -40 ; trotate s b 0 0 0 0 0 1 -10 ; tscale b 0 0 0 SCALE1 ; tscale s 0 0 0 SCALE1` | `bop b s ; bopcut result` | `checkprops result -s 680565 ; checkshape result ; checknbshapes result -vertex 11 -edge 16 -wire 8 -face 7 -shell 1 -solid 1 -compsolid 0 -compound 1 -shape 45` | A regression of new Boolean Operations | WRONGRESULT |
| 51 | `modalg_2/bug4717_7` **[TODO]** | `box b -20 -20 -20 40 40 20 ; ellipse w1 0 0 0 15 10 ; mkedge w1 w1 -pi/2 pi/2 ; trotate w1 0 0 0 1 0 0 90 ; wire w w1 ; revol r w 0 0 0 0 0 1 360 ; shape s So ; trotate s b 0 0 0 1 1 0 -40 ; trotate s b 0 0 0 0 0 1 -10 ; tscale b 0 0 0 SCALE1 ; tscale s 0 0 0 SCALE1` | `bop s b ; bopcommon result` | `checkprops result -s 181937 ; checkshape result ; checknbshapes result -vertex 3 -edge 4 -wire 2 -face 2 -shell 1 -solid 1 -compsolid 0 -compound 1 -shape 14` | A regression of new Boolean Operations | WRONGRESULT |
| 52 | `modalg_2/bug4717_8` **[TODO]** | `box b -20 -20 -20 40 40 20 ; ellipse w1 0 0 0 15 10 ; mkedge w1 w1 -pi/2 pi/2 ; trotate w1 0 0 0 1 0 0 90 ; wire w w1 ; revol r w 0 0 0 0 0 1 360 ; shape s So ; trotate s b 0 0 0 1 1 0 -40 ; trotate s b 0 0 0 0 0 1 -10 ; tscale b 0 0 0 SCALE1 ; tscale s 0 0 0 SCALE1` | `bop b s ; bopcommon result` | `checkprops result -s 181937 ; checkshape result ; checknbshapes result -vertex 3 -edge 4 -wire 2 -face 2 -shell 1 -solid 1 -compsolid 0 -compound 1 -shape 14` | A regression of new Boolean Operations | WRONGRESULT |
| 53 | `modalg_4/bug693` | `box h1 234 52 0 41 98 40 ; box v1 241 136 20 31 31 15 ; box h3 215 120 20 78 15 15 ; compound h1 c1 ; compound v1 c2 ; tcopy c1 cc1 ; tcopy c2 cc2 ; tcopy c1 cc1 ; tcopy c2 cc2 ; tcopy c1 cc1 ; tcopy c2 cc2 ; compound int1 cut1 cut2 comp3 ; tcopy comp3 ccomp3 ; tcopy h3 ch3 ; tcopy comp3 ccomp3 ; ...[19 cmds]` | `bcommon int1 cc1 cc2 ; bcut cut1 cc1 cc2 ; bcut cut2 cc2 cc1 ; bcommon int2 ccomp3 ch3 ; bcut cut3 ccomp3 ch3 ; bcut cut4 ch3 ccomp3` | `checkshape -top int1 ; checkshape -top cut1 ; checkshape -top cut2 ; checkshape -top comp3 ; checkshape -top int2 ; checkshape -top cut3 ; checkshape -top cut4 ; checkshape -top result ; ...[10 cmds]` | The boolean operations give incorrect result if one of shapes is compound | WRONGRESULT |
| 54 | `modalg_4/bug693_1` | `box h1 234 52 0 41 98 40 ; box v1 241 136 20 31 31 15 ; box h3 215 120 20 78 15 15 ; compound h1 c1 ; compound v1 c2 ; tcopy c1 cc1 ; tcopy c2 cc2 ; tcopy c1 cc1 ; tcopy c2 cc2 ; tcopy c1 cc1 ; tcopy c2 cc2 ; compound int1 cut1 cut2 comp3 ; tcopy comp3 ccomp3 ; tcopy h3 ch3 ; tcopy comp3 ccomp3 ; ...[19 cmds]` | `bop cc1 cc2 ; bopcommon int1 ; bop cc1 cc2 ; bopcut cut1 ; bop cc2 cc1 ; bopcut cut2 ; bop ccomp3 ch3 ; bopcommon int2 ; bop ccomp3 ch3 ; bopcut cut3 ; bop ch3 ccomp3 ; bopcut cut4` | `checkshape -top int1 ; checkshape -top cut1 ; checkshape -top cut2 ; checkshape -top comp3 ; checkshape -top int2 ; checkshape -top cut3 ; checkshape -top cut4 ; checkshape -top result ; ...[10 cmds]` | The boolean operations give incorrect result if one of shapes is compound | WRONGRESULT |
| 55 | `modalg_4/bug7626_1` | `psphere s 10 ; box b 10 0 0 20 20 20 ; trotate b 10 0 0 0 0 1 45` | `bcut result s b` | `checkprops result -s 1271.7 ; checkshape result ; checknbshapes result -vertex 4 -edge 7 -wire 4 -face 3 -shell 1 -solid 1 -compsolid 0 -compound 1 -shape 21` | Bad result of bcut operation | WRONGRESULT |
| 56 | `modalg_4/bug7626_2` | `psphere s 10 ; box b 10 0 0 20 20 20 ; trotate b 10 0 0 0 0 1 45` | `bcommon result s b` | `checkprops result -s 199.095 ; checkshape result ; checknbshapes result -vertex 2 -edge 3 -wire 3 -face 3 -shell 1 -solid 1 -compsolid 0 -compound 1 -shape 14` | Bad result of bcut operation | WRONGRESULT |
| 57 | `modalg_5/bug21564` | `plane p1 0 0 0 0 0 1 ; trim p1t p1 -10 10 -10 10 ; convert bs1 p1t ; mkface f1 bs1 ; plane p2 0 0 0 1 0 0 ; mkface f2 p2 -10 10 -10 10 ; plane p2 0 0 0 1 2 0 ; mkface f2 p2 -15 15 -15 15` | `bsection sec f1 f2 ; bsection sec f1 f2 -a ; bsection sec f1 f2 ; bsection sec f1 f2 -a` | `-` | Intersection of two planar faces produces curve with too many poles | WRONGRESULT |
| 58 | `modalg_5/bug22027` | `circle c 0 0 0 25 ; ellipse e 0 0 0 25 10 ; mkedge c c ; mkedge e e ; wire w1 c ; wire w2 e ; mkplane f1 w1 ; mkplane f2 w2` | `bop f1 f2 ; bopcut result` | `checkprops result -s 1178.1 ; checkshape result ; checknbshapes result -vertex 2 -edge 4 -wire 2 -face 2` | Bop command between two faces raises exception | CRASH |
| 59 | `modalg_5/bug23402` | `polyline l 0 0 0 0 5 0 0 5 5 0 3 5 0 3 3 0 2 3 0 2 5 0 0 5 0 0 0 ; mkplane sf l ; prism s sf 5 0 0 ; polyline w1 -2 2.5 2 7 2.5 2 7 7 2 -2 7 2 -2 2.5 2 ; mkplane f1 w1 ; ttranslate c1 0 0 5 ; polyline w2 -2 2.5 3 7 2.5 3 7 7 3 -2 7 3 -2 2.5 3 ; mkplane f2 w2 ; ttranslate c2 0 0 5 ; ...[12 cmds]` | `bop s f1 ; bopcommon c1 ; bop s f2 ; bopcommon c2 ; bop s f3 ; bopcommon c3` | `checkprops c1 -s 12.5 ; checkprops c2 -s 12.5 ; checkprops c3 -s 10` | Issue in BRepAlgoAPI_Common - SOLID and FACE | WRONGRESULT |
| 60 | `modalg_5/bug23855` | `psphere s1 10 ; psphere s2 10 ; explode result so` | `bcommon result s1 s2` | `checkshape result ; checknbshapes result_1 -ref [nbshapes s1] ; checknbshapes result_1 -ref [nbshapes s2] ; checkprops result_1 -equal s1 ; checkprops result_1 -equal s2` | Old BOPs fail on Win7 64bit when using TBB | SAMEDOMAIN |
| 61 | `modalg_5/bug23876` | `vertex v1 10 0 0 ; vertex v2 20 0 0 ; vertex v3 10 0 50 ; vertex v4 20 0 50 ; edge e1 v1 v3 ; edge e2 v3 v4 ; edge e3 v2 v4 ; edge e4 v2 v1 ; wire w e1 e2 e3 e4 ; mkplane f w ; revol b1 f 0 0 0 0 0 1 360 ; ptorus b2 15 5 ; ttranslate b2 0 0 50` | `bop b1 b2 ; bopcommon result` | `checkprops result -s 2422.92 ; checkshape result` | New Boolean Operation algorithm works incorrect with cylinder made by revolution and torus | TANGENT |
| 62 | `modalg_5/bug23881` | `polyline w1 0 0 0 1 0 0 1 1 0 0 1 0 0 0 0 ; polyline w2 0 1 0 1 1 0 1 2 0 0 2 0 0 1 0 ; mkplane f1 w1 ; mkplane f2 w2 ; sewing s1 0.0001 f1 f2 ; plane pl 0 1 0 0 1 0 ; mkface f3 pl ; compound r s1 c ; ! [regexp {EDGE\s*:\s*([0-9]+)} [nbshapes c ] str nbedges_c] } { ; ...[10 cmds]` | `bsection r s1 f3` | `-` | BRepAlgoAPI_Section HasAnsectorFaceOn1 returned False on the boundary | WRONGRESULT |
| 63 | `modalg_5/bug23932_1` | `cylinder s1 538.57646417050069, 347.77316708315834, 183.37500000000000 0 -1 0 1 0 0 44.194173824159222 ; trimu s1t s1 0 $PI ; mkface b1 s1t ; cylinder s2 859.00000000000011 463.75990629028450 80.865093709715211 -1 0 0 0 -1 0 186.78835506149036 ; trimu s2t s2 0 $PI ; mkface b2 s2t` | `bop b1 b2 ; bopsection result` | `checkprops result -l 326.401 ; checkshape result ; checksection result` | Standard_NoSuchObject thrown computing section between two cylinders - new to 6.6.0 | CRASH |
| 64 | `modalg_5/bug23932_2` | `cylinder s1 538.57646417050069 347.77316708315834 183.37500000000000 0 -1 0 1 0 0 44.819173824159222 ; trimu s1t s1 0 $PI ; mkface b1 s1t ; cylinder s2 859.00000000000011 463.75990629028450 80.865093709715211 -1 0 0 0 -1 0 187.41335506149036 ; trimu s2t s2 0 $PI ; mkface b2 s2t` | `bop b1 b2 ; bopsection result` | `checkprops result -l 330.903 ; checkshape result ; checksection result` | Standard_NoSuchObject thrown computing section between two cylinders - new to 6.6.0 | CRASH |
| 65 | `modalg_5/bug24029` | `box b1 10 10 10 ; box b2 5 5 5 10 10 10 ; compound b1 b2 c` | `-` | `-` | Add a flag to bopcheck command to provide possibility to disable Face/Face intersection | API |
| 66 | `modalg_5/bug24033` | `polyline f1 0 0 0 0 1 0 1 1 0 1 0 0 0 0 0 ; mkplane f1 f1 ; polyline f2 0 1 0 0 2 0 1 2 0 1 1 0 0 1 0 ; mkplane f2 f2 ; polyline f3 0 2 0 0 3 0 1 3 0 1 2 0 0 2 0 ; mkplane f3 f3 ; polyline f4 1 0 0 1 1 0 2 1 0 2 0 0 1 0 0 ; mkplane f4 f4 ; polyline f5 1 1 0 1 2 0 2 2 0 2 1 0 1 1 0 ; mkplane f5 f5 ; ...[24 cmds]` | `bop s b ; bopcommon r` | `-` | All the orientation as a result of BRepAlgoAPI_Common is set to INTERNAL | CONTAINER |
| 67 | `modalg_5/bug24060` | `polyline p 0 0 0 0 0 1 1 0 1 1 0 0 0 0 0 ; mkplane b1 p ; vertex b2 0 0 1` | `bclearobjects ; bcleartools ; baddobjects b1 b2 ; bfillds ; bbuild result` | `checknbshapes result -vertex 4` | Wrong result done by general fuse algorithm | DEGEN |
| 68 | `modalg_5/bug24157_10` | `box b0 10 10 10 ; box b 12 12 12 ; explode b f ; ttranslate f 0 -1 -1 ; tcopy f fx_$i ; ttranslate fx_$i [expr ($i*2)] 0. 0. ; ttranslate f -1 0 -1 ; tcopy f fy_$i ; ttranslate fy_$i 0. [expr ($i*2)] 0. ; ttranslate f -1 -1 0 ; tcopy f fz_$i ; ttranslate fz_$i 0. 0. [expr ($i*2)] ; ...[16 cmds]` | `bclearobjects ; bcleartools ; baddobjects b0 ; baddcompound b1 ; baddcompound b2 ; baddcompound b3 ; baddcompound b4 ; bfillds` | `-` | Parallelization of assembly part of BO | HANG_PERF |
| 69 | `modalg_5/bug24157_4` | `vertex v1_${i} ${aX} ${aY} ${aZ} ; vertex v2_${i} ${aX} ${aY} ${aZ} ; edge e1_${i} v1_${i} v2_${i} ; edge e2_${i} v2_${i} v1_${j} ; mkplane bs w ; box b 700 820 1 ; explode b f ; tcopy bs bs_${i}_{$j} ; ttranslate bs_${i}_{$j} [expr $i * 17.] [expr $j * 20.] 0.` | `bclearobjects ; bcleartools ; baddobjects b1 ; baddcompound b2 ; bfillds` | `-` | Parallelization of assembly part of BO | HANG_PERF |
| 70 | `modalg_5/bug24157_5` | `box b $L $L 20 ; explode b f ; vertex v1 ${dS} -${dL} 0 ; vertex v2 ${dS} [expr ${L}+${dL}] 0 ; edge ex v1 v2 ; vertex v1 -${dL} $dS 0 ; vertex v2 [expr ${L}+${dL}] ${dS} 0 ; edge ey v1 v2 ; tcopy ex ex$i ; ttranslate ex$i [expr $i*${dS}] 0 0 ; tcopy ey ey$i ; ttranslate ey$i 0 [expr $i*${dS}] 0 ; ...[18 cmds]` | `bclearobjects ; bcleartools ; baddobjects b1 ; baddcompound b2 ; bfillds` | `-` | Parallelization of assembly part of BO | HANG_PERF |
| 71 | `modalg_5/bug24157_6` | `box b1 100 100 100 ; psphere sp 1. ; explode sp f ; tcopy sp   sp_${N}_${K}_${M}_${n}_${k}_${m} ; ttranslate sp_${N}_${K}_${M}_${n}_${k}_${m} 2.5 2.5 2.5 ; ttranslate sp_${N}_${K}_${M}_${n}_${k}_${m} [expr $k*${ds}] [expr $m*${ds}] [expr $n*${ds}] ; ...[7 cmds]` | `bclearobjects ; bcleartools ; baddobjects b1 ; baddcompound b2 ; bfillds` | `-` | Parallelization of assembly part of BO | HANG_PERF |
| 72 | `modalg_5/bug24157_7` | `box b1 100 100 100 ; psphere sp 1. ; explode sp f ; tcopy sp   sp_${N}_${K}_${M}_${n}_${k}_${m} ; ttranslate sp_${N}_${K}_${M}_${n}_${k}_${m} 2.5 2.5 2.5 ; ttranslate sp_${N}_${K}_${M}_${n}_${k}_${m} [expr $k*${ds}] [expr $m*${ds}] [expr $n*${ds}] ; ...[7 cmds]` | `bclearobjects ; bcleartools ; baddobjects b1 ; baddcompound b2 ; bfillds` | `-` | Parallelization of assembly part of BO | HANG_PERF |
| 73 | `modalg_5/bug24157_8` | `box b 100 100 100 ; explode b f ; tcopy fx fx_${j} ; ttranslate fx_${j} [expr $j*$dX] 0. 0. ; tcopy fy fy_${j} ; ttranslate fy_${j} 0. [expr $j*$dX] 0. ; tcopy fz fz_${j} ; ttranslate fz_${j} 0. 0. [expr $j*$dX]` | `bclearobjects ; bcleartools ; baddcompound b1 ; bfillds -t` | `-` | Parallelization of assembly part of BO | HANG_PERF |
| 74 | `modalg_5/bug24187` | `box b1 10 10 10 ; circle c1 -3 5 2 5 ; circle c2 -3 5 8 5 ; mkedge e1 c1 ; mkedge e2 c2 ; compound e1 e2 b2` | `bop b1 b2 ; bopcommon result` | `checkprops result -l 18.5459 ; checkshape result ; checksection result ; checknbshapes result -vertex 6 -edge 4 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 11` | Wrong result of COMMON operation | WRONGRESULT |
| 75 | `modalg_5/bug24328` | `vertex v1 23.500000 11.123000 0 ; vertex v2 22.000000 11.140900 0 ; vertex v3 20.500000 11.180700 0 ; edge e1 v1 v2 ; edge e2 v2 v3 ; wire w e1 e2 ; revol r w 0 0 0 1 0 0 360` | `-` | `-` | Revolution of a wire generates two interfered faces | INVALID |
| 76 | `modalg_5/bug24359` | `psphere s01 2 ; ttranslate s01 1 1 1 ; psphere s02 2 ; ttranslate s02 1 1 2 ; psphere s03 2 ; ttranslate s03 1 1 3 ; psphere s04 2 ; ttranslate s04 1 2 1 ; psphere s05 2 ; ttranslate s05 1 2 2 ; psphere s06 2 ; ttranslate s06 1 2 3 ; psphere s07 2 ; ttranslate s07 1 3 1 ; psphere s08 2 ; ...[54 cmds]` | `bop s01 s02 ; bopfuse s ; bop s s03 ; bopfuse s ; bop s s04 ; bopfuse s ; bop s s05 ; bopfuse s ; bop s s06 ; bopfuse s ; bop s s07 ; bopfuse s ; bop s s08 ; bopfuse s ; bop s s09 ; ...[52 cmds]` | `checknbshapes result -vertex 60 -edge 105 -wire 29 -face 29 -shell 1 -solid 1 -compsolid 0 -compound 1 -shape 226 ; checkprops result -s 150.187 ; checkshape result` | Crash on BRepAlgoAPI_Fuse | CRASH |
| 77 | `modalg_5/bug24404` | `box b1 1500 1500 1500 ; explode b1 f ; circle c1 1361.60462531413 1500 275.105307765905 0 1 0 182.781239888725 ; mkedge e c1 ; wire w e ; mkplane b2 w ; explode b1 f ; circle c2 398.623694869499 1500 5.77182937332096 0 1 0 181.948898616306 ; mkedge e c2 ; wire w e ; mkplane b2 w ; explode b1 f ; ...[22 cmds]` | `bcut b1 b1 b2 ; bcut b1 b1 b2 ; bcut b1 b1 b2 ; bcut b1 b1 b2` | `-` | The function BRepTools::UVBounds gives wrong result for the face | PCURVE |
| 78 | `modalg_5/bug24597` | `cylinder c1 20 ; trimv c1 c1 0 50 ; mkface b1 c1 ; vertex v1 0 20 25 ; orientation v1 I ; box b2 -30 -30 5 60 60 40` | `bclearobjects ; bcleartools ; baddobjects b1 b2 ; bfillds ; bbuild result` | `checkprops result -s 33136.3 ; checkshape result ; checknbshapes result -vertex 13 -edge 19 -wire 13 -face 11 -shell 2 -solid 2 -compsolid 0 -compound 1 -shape 61` | Missing internal vertex in the result of General Fuse Operation | DEGEN |
| 79 | `modalg_5/bug24618_1` | `cylinder c1 20 ; trimv c1 c1 0 50 ; mkface c1 c1 ; vertex v1 0 20 25 ; explode r` | `bclearobjects ; bcleartools ; baddobjects c1 v1 ; bfillds ; bbuild r` | `checknbshapes result -vertex 1 -edge 0 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 0 -shape 1` | Embedding vertex in BOP depends on the order of arguments | ORDER |
| 80 | `modalg_5/bug24618_2` | `cylinder c1 20 ; trimv c1 c1 0 50 ; mkface c1 c1 ; vertex v1 0 20 25 ; explode r` | `bclearobjects ; bcleartools ; baddobjects c1 v1 ; bfillds ; bbuild r` | `checknbshapes result -vertex 3 -edge 3 -wire 1 -face 1 -shell 0 -solid 0 -compsolid 0 -compound 0 -shape 8` | Embedding vertex in BOP depends on the order of arguments | ORDER |
| 81 | `modalg_5/bug24618_3` | `cylinder c1 20 ; trimv c1 c1 0 50 ; mkface c1 c1 ; vertex v1 0 20 25 ; explode r` | `bclearobjects ; bcleartools ; baddobjects v1 c1 ; bfillds ; bbuild r` | `checknbshapes result -vertex 1 -edge 0 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 0 -shape 1` | Embedding vertex in BOP depends on the order of arguments | ORDER |
| 82 | `modalg_5/bug24618_4` | `cylinder c1 20 ; trimv c1 c1 0 50 ; mkface c1 c1 ; vertex v1 0 20 25 ; explode r` | `bclearobjects ; bcleartools ; baddobjects v1 c1 ; bfillds ; bbuild r` | `checknbshapes result -vertex 3 -edge 3 -wire 1 -face 1 -shell 0 -solid 0 -compsolid 0 -compound 0 -shape 8` | Embedding vertex in BOP depends on the order of arguments | ORDER |
| 83 | `modalg_5/bug24620` | `vertex v1 0 0 0 ; vertex v2 10 0 0 ; vertex v3 4 -5 0 ; vertex v4 4 5 0 ; edge e1 v1 v2 ; edge e2 v3 v4 ; compound e1 e2 b1 ; compound x0 x1 b1 result` | `bopcheck b1` | `checknbshapes result -vertex 4 -edge 2 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 2 -shape 8` | BOPAlgo_CheckerSI returns interferences that are not sub-shapes of the source shape | INVALID |
| 84 | `modalg_5/bug24639` | `vertex v1 0 0 0 ; vertex v2 0 0 $aC ; edge ez v1 v2 ; tcopy ez ez_${i}_{$j} ; ttranslate ez_${i}_{$j} [expr $i + ${dC}] [expr $j + $dC] 0. ; vertex v1 0. 0. 0. ; vertex v2 $aC  0. 0. ; edge ex v1 v2 ; tcopy ex ex_${i}_{$j} ; ttranslate ex_${i}_{$j} 0. [expr $i + $dC] [expr $j + $dC] ; ...[18 cmds]` | `bclearobjects ; bcleartools ; baddcompound bx ; baddcompound by ; baddcompound bz ; brunparallel 0 ; brunparallel 1` | `-` | Parallelization FillDS part of BO | HANG_PERF |
| 85 | `modalg_5/bug24706` | `box b 10 10 10 ; psphere s 2 ; explode r sh` | `bcut r b s` | `-` | Solids produced by BOP do not have flag Closed set in shells | CONTAINER |
| 86 | `modalg_5/bug24758_1` | `circle outer 0 0 0 0 0 1 R ; circle inner 0 0 0 0 0 1 R-2 ; circle round 3 6 0 0 0 1 5 ; polyline cut -2 0 0  -2 R+1 0  0.5 R+1 0  -1 0 0  -2 0 0 ; plane p0 ; _curvetoface outer ; _curvetoface inner ; _curvetoface round ; mkface cut p0 cut ; tcopy inner profile ; ...[41 cmds]` | `bcommon teeth cut round ; bcommon teeth teeth outer ; bfuse profile profile teeth ; bfuse shank shank sh2 ; bfuse cutter base shank` | `checkshape cutter ; checkprops result -s 1403.85 ; checkshape result` | Sample Draw scripts for demonstrating sweeping algorithm | DOWNSTREAM |
| 87 | `modalg_5/bug24758_2` | `polyline rectangle1 d -R 0  R -R 0  -d R 0  -R R 0  d -R 0 ; circle circle1 0 0 0  0 0 1 R ; mkedge circle1 circle1 ; wire circle1 circle1 ; circle circle2 0 0 0  0 0 1 Rr ; mkedge circle2 circle2 ; wire circle2 circle2 ; plane p0 ; mkface rectangle1 p0 rectangle1 ; mkface circle1 p0 circle1 ; ...[59 cmds]` | `bcommon sec rectangle1 circle1 ; bfuse sec sec circle2 ; bsection sflute spiral f0 ; bcut sec sec flute ; bcut sec sec flute ; bcommon qq sh1 sh2 ; bcut sharpener sh qq ; ...[11 cmds]` | `checkshape drill ; checkprops result -s 3277.87 ; checkshape result` | Sample Draw scripts for demonstrating sweeping algorithm | DOWNSTREAM |
| 88 | `modalg_5/bug25242` | `box a 0 0 0 10 10 10 ; box b 20 0 0 10 10 10` | `bfuse r0 a b ; bcut result r0 a` | `checknbshapes result -vertex 8 -edge 12 -wire 6 -face 6 -shell 1 -solid 1 -compsolid 0 -compound 1 -shape 35` | Wrong result of cut operation | SAMEDOMAIN |
| 89 | `modalg_5/bug25337_1` | `circle c1 -50 0 0 10 ; circle c2 -35 0 0 10 ; mkedge e1 c1 ; mkedge e2 c2 ; wire w1 e1 ; wire w2 e2 ; mkplane f1 w1 ; mkplane f2 w2` | `bop f1 f2 ; bopfuse result` | `checkprops result -s 582.987 ; checkshape result ; checknbshapes result -vertex 4 -edge 6 -wire 3 -face 3 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 17` | Regression to version 6.7.1 : boolean operations fail on two planar circular faces lying in the same plane | SAMEDOMAIN |
| 90 | `modalg_5/bug25337_2` | `circle c1 -50 0 0 10 ; circle c2 -35 0 0 10 ; mkedge e1 c1 ; mkedge e2 c2 ; wire w1 e1 ; wire w2 e2 ; mkplane f1 w1 ; mkplane f2 w2` | `bop f1 f2 ; bopcommon result` | `checkprops result -s 45.3312 ; checkshape result ; checknbshapes result -vertex 3 -edge 3 -wire 1 -face 1 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 9` | Regression to version 6.7.1 : boolean operations fail on two planar circular faces lying in the same plane | SAMEDOMAIN |
| 91 | `modalg_5/bug25354_32` | `box b1 10 10 10 ; box b2 10 10 10 ; ttranslate b2 -10 -10 -10` | `bop b1 b2 ; bopsection result` | `checknbshapes result -vertex 1 -edge 0 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 2` | Intersection operation | DEGEN |
| 92 | `modalg_5/bug25354_33` | `box b1 10 10 10 ; box b2 10 10 10 ; ttranslate b2 5 -10 -10 ; trotate b2 5 0 0 0 0 1 -45` | `bop b1 b2 ; bopsection result` | `checknbshapes result -vertex 1 -edge 0 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 2` | Intersection operation | DEGEN |
| 93 | `modalg_5/bug25354_34` | `box b1 10 10 10 ; box b2 10 10 10 ; ttranslate b2 5 -10 -4 ; trotate b2 5 0 0  0 0 1 -45 ; trotate b2 5 0 0  1 0 0 45` | `bop b1 b2 ; bopsection result` | `checknbshapes result -vertex 1 -edge 0 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 2` | Intersection operation | DEGEN |
| 94 | `modalg_5/bug25354_35` | `box b1 10 10 10 ; box b2 10 10 10 ; ttranslate b2 -10 -10 -4` | `bop b1 b2 ; bopsection result` | `checknbshapes result -vertex 2 -edge 1 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 4` | Intersection operation | DEGEN |
| 95 | `modalg_5/bug25354_36` | `box b1 10 10 10 ; box b2 10 10 10 ; ttranslate b2 5 -10 -4 ; trotate b2 5 0 0  0 0 1 -45` | `bop b1 b2 ; bopsection result` | `checknbshapes result -vertex 2 -edge 1 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 4` | Intersection operation | DEGEN |
| 96 | `modalg_5/bug25354_37` | `box b1 10 10 10 ; box b2 10 10 10 ; ttranslate b2 10 3 4` | `bop b1 b2 ; bopsection result` | `checknbshapes result -vertex 4 -edge 4 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 9` | Intersection operation | DEGEN |
| 97 | `modalg_5/bug25354_38` | `box b1 10 10 10 ; box b2 10 10 10 ; ttranslate b2 5 3 4` | `bop b1 b2 ; bopsection result` | `checknbshapes result -vertex 6 -edge 6 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 13` | Intersection operation | DEGEN |
| 98 | `modalg_5/bug25354_39` | `vertex b1 0 0 0 ; vertex b2 0 0 0` | `bop b1 b2 ; bopsection result` | `checknbshapes result -vertex 1 -edge 0 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 2` | Intersection operation | DEGEN |
| 99 | `modalg_5/bug25354_40` | `vertex b1 4 0 0 ; vertex ba 0 0 0 ; vertex bb 10 0 0 ; edge b2 ba bb` | `bop b1 b2 ; bopsection result` | `checknbshapes result -vertex 1 -edge 0 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 2` | Intersection operation | DEGEN |
| 100 | `modalg_5/bug25354_41` | `vertex b1 0 0 0 ; vertex ba 0 0 0 ; vertex bb 10 0 0 ; edge b2 ba bb` | `bop b1 b2 ; bopsection result` | `checknbshapes result -vertex 1 -edge 0 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 2` | Intersection operation | DEGEN |
| 101 | `modalg_5/bug25354_45` | `vertex b1 0 0 0 ; box b2 10 10 10` | `bop b1 b2 ; bopsection result` | `checknbshapes result -vertex 1 -edge 0 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 2` | Intersection operation | DEGEN |
| 102 | `modalg_5/bug25354_46` | `vertex b1 4 0 0 ; box b2 10 10 10` | `bop b1 b2 ; bopsection result` | `checknbshapes result -vertex 1 -edge 0 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 2` | Intersection operation | DEGEN |
| 103 | `modalg_5/bug25354_47` | `vertex b1 4 3 0 ; box b2 10 10 10` | `bop b1 b2 ; bopsection result` | `checknbshapes result -vertex 1 -edge 0 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 2` | Intersection operation | DEGEN |
| 104 | `modalg_5/bug25354_48` | `vertex b1 4 3 2 ; box b2 10 10 10` | `bop b1 b2 ; bopsection result` | `checknbshapes result -vertex 0 -edge 0 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 1` | Intersection operation | DEGEN |
| 105 | `modalg_5/bug25354_49` | `box b 100 100 100 ; explode b f ; tcopy b_1 f1_${i} ; ttranslate f1_${i} [expr $i * 10]  0.  0. ; tcopy b_3 f3_${i} ; ttranslate f3_${i} 0. [expr $i * 10]  0. ; tcopy b_5 f5_${i} ; ttranslate f5_${i} 0. 0. [expr $i * 10] ; psphere s_${i} [expr $i * 5] ; ttranslate s_${i} 50 50 50` | `bclearobjects ; bcleartools ; baddcompound q ; bfillds -t ; bbop result 4 -t` | `checknbshapes result -vertex 2781 -edge 7332 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 10114` | Intersection operation | HANG_PERF |
| 106 | `modalg_5/bug25354_50` | `box b 10 10 10 ; ttranslate b -5 -5 -5 ; psphere b{$i} [expr $i * 0.1  + 5.] ; explode b{$i} f` | `bclearobjects ; bcleartools ; baddobjects b ; baddtools q ; bfillds -t ; bbop result 4 -t` | `checknbshapes result -vertex 530 -edge 524 -wire 0 -face 0 -shell 0 -solid 0 -compsolid 0 -compound 1 -shape 1055` | Intersection operation | HANG_PERF |
| 107 | `modalg_5/bug25477_1` | `box b1 10 10 10 ; box b2 10.00001 0 0 10 10 10` | `bfuzzyvalue 0.00002 ; bop b1 b2 ; bopfuse result` | `checknbshapes result -vertex 12 -edge 20 -wire 10 -face 10 -shell 1 -solid 1 -compsolid 0 -compound 1 -shape 55` | Boolean Operations with additional tolerance - Fuzzy Boolean operations | TOL |
| 108 | `modalg_5/bug25657_1` | `pcylinder a1 140 220 ; pcylinder a2 206 120 ; ttranslate a2 100 100 120 ; explode a e ; blend result a 20 a_9` | `bcut a a1 a2` | `-` | Bad result of Fillet operation | DOWNSTREAM |
| 109 | `modalg_5/bug25715_1` | `pcylinder s1 0.069 0.6 ; pcylinder s2 0.024, 0.4 ; trotate s1 0 0 0 0 0 1 90 ; trotate s2 0 0 0 0 1 0 90 ; trotate s2 0 0 0 1 0 0 -45 ; ttranslate s2 -0.2 0 0.48` | `bcut result s1 s2` | `checknbshapes result -vertex 4 -edge 6 -wire 6 -face 4 -shell 1 -solid 1 -compsolid 0 -compound 1 -shape 23` | Intersection between cylinders produces excess vertices | WRONGRESULT |
| 110 | `modalg_5/bug25722` | `box b1 10 10 10 ; box b2 10.0001 0 0 10 10 10 ; box b1 10 10 10 ; box b2 10.0001 0 0 10 10 10` | `bfuzzyvalue 0.0001 ; bclearobjects ; bcleartools ; baddobjects b1 ; baddtools b2 ; bfillds ; bbop r 1 ; bfuzzyvalue 0. ; bclearobjects ; bcleartools ; baddobjects b1 ; baddtools b2 ; ...[14 cmds]` | `checknbshapes r -solid 1 ; checknbshapes result -solid 2` | Wrong implementation of SetFuzzyValue method | TOL |
| 111 | `modalg_5/bug25801` | `box b1 10 10 10` | `-` | `-` | Some of the commands in BOPTest packages show the execution time without -t key | API |
| 112 | `modalg_6/bug14531` | `pcylinder c 10 20 ; pcone x 20 0 20 ; trotate x 0 0 20 0 1 0 45 ; ttranslate x 5 0 0 ; ttranslate x 0 0 1 ; ttranslate c -5 0 0 ; explode c f ; explode x f ; Number of shapes in result ; VERTEX    : 7 ; EDGE      : 12 ; WIRE      : 5 ; FACE      : 4 ; SHELL     : 0 ; SOLID     : 0 ; COMPSOLID : 0 ; ...[19 cmds]` | `bop a1 a2 ; bopfuse result` | `checkshape result ; checkprops result -s 3033.79 ; checknbshapes result -ref ${nbshapes_expected} -t -m "Result obtained by Boolean cut operation"` | Boolean Operation Algorithm fails | TANGENT |
| 113 | `modalg_6/bug24803` | `vertex v1 $x1 $y1 $z1 ; settolerance v1 $tolerance1 ; vertex v2 $x2 $y2 $z2 ; settolerance v2 $tolerance2 ; circle c1 $x1 $y1 $z1 $tolerance1 ; circle c2 $x2 $y2 $z2 $tolerance2 ; circle c_res $x_res $y_res $z_res $tolerance_res` | `bop v1 v2 ; bopcommon res` | `checkreal "MaxTolerance" ${MaxTolerance} ${expected_MaxTolerance} ${tol_abs_MaxTolerance} ${tol_rel_MaxTolerance} ; ...[5 cmds]` | improve the result of v/v interference for two vertices case | TOL |
| 114 | `modalg_6/bug25880` | `box Box018 13.550000190735 50.200000762939 3.299999952316 ; ttranslate Box018 -19.1 -0.1 2.35 ; psphere Sphere002 1.600000023842 0 90 180 ; ttranslate Sphere002 -15 50.1 5.5 ; pcylinder Cylinder006 1.600000023842 3.200000047684 180 ; ttranslate Cylinder006 -15 50.1 2.35 ; ...[26 cmds]` | `bfuzzyvalue 0.00001 ; bclearobjects ; bcleartools ; baddobjects Box018 ; baddtools Sphere002 Cylinder006 Sphere001 Cylinder005 Cylinder004 ; bfillds ; bbop result 1` | `checkprops result -s 1893.17 ; checkshape result ; checknbshapes result -ref ${nbshapes_expected} -t -m "fuzzy booleans with multiple tools"` | fuzzy booleans with multiple tools | TOL |
| 115 | `modalg_6/bug25937_1` | `plane h 0 0 0 1 0 0 ; mkface h h ; halfspace h h 10 0 0 ; polyline a -10 -10 0 10 -10 0 10 10 0 -10 10 0 -10 -10 0 ; Number of shapes in shape ; VERTEX    : 4 ; EDGE      : 3 ; WIRE      : 1 ; FACE      : 0 ; SHELL     : 0 ; SOLID     : 0 ; COMPSOLID : 0 ; COMPOUND  : 1 ; SHAPE     : 9 ; "` | `bop a h ; bopcut result` | `checkprops result -l 40. ; checkshape result ; checksection result ; checknbshapes result -ref ${nbshapes_expected} -t -m "Result obtained by Cut a wire by halfspace"` | Failed Cut Edge or Face by HalfSpace. | HALFSPACE |
| 116 | `modalg_6/bug25937_2` | `plane h 0 0 0 1 0 0 ; mkface h h ; halfspace h h 10 0 0 ; plane a 0 0 0 0 0 1 ; mkface a a -10 10 -10 10 ; Number of shapes in shape ; VERTEX    : 4 ; EDGE      : 4 ; WIRE      : 1 ; FACE      : 1 ; SHELL     : 0 ; SOLID     : 0 ; COMPSOLID : 0 ; COMPOUND  : 1 ; SHAPE     : 11 ; "` | `bop a h ; bopcut result` | `checkprops result -l 60. -s 200. ; checksection result ; checkshape result ; checknbshapes result -ref ${nbshapes_expected} -t -m "Result obtained by Cut a wire by halfspace"` | Failed Cut Edge or Face by HalfSpace. | HALFSPACE |
| 117 | `modalg_6/bug25937_3` | `plane h 0 0 0 1 0 0 ; mkface h h ; halfspace h h 10 0 0 ; box a -10 -10 0 20 20 10 ; Number of shapes in shape ; VERTEX    : 8 ; EDGE      : 12 ; WIRE      : 6 ; FACE      : 6 ; SHELL     : 1 ; SOLID     : 1 ; COMPSOLID : 0 ; COMPOUND  : 1 ; SHAPE     : 35 ; "` | `bop a h ; bopcut result` | `checkprops result -s 1000. ; checkshape result ; checknbshapes result -ref ${nbshapes_expected} -t -m "Result obtained by Cut a wire by halfspace"` | Failed Cut Edge or Face by HalfSpace. | HALFSPACE |
| 118 | `modalg_6/bug26420_1` | `polyline pp 0 4 0 7 4 0 7 6 0 0 6 0 0 4 0 ; vertex v0 5 4 0 ; vertex v1 4 6 0 ; explode r w` | `bclearobjects ; bcleartools ; baddobjects v0 v1 pp ; bfillds ; bbuild r` | `-` | BOPAlgo_Builder resets "Closed" flag in the result | CONTAINER |
| 119 | `modalg_6/bug26420_2` | `polyline pp 0 4 0 7 4 0 7 6 0 0 6 0 0 4 0 ; mkplane ff pp ; vertex v0 5 4 0 ; vertex v1 4 6 0 ; explode r w` | `bclearobjects ; bcleartools ; baddobjects v0 v1 ff ; bfillds ; bbuild r` | `-` | BOPAlgo_Builder resets "Closed" flag in the result | CONTAINER |
| 120 | `modalg_6/bug26420_3` | `box bb 0 0 0 5 5 5 ; explode bb sh ; polyline pp -1 -1 2 -1 6 2 6 6 2 6 -1 2 -1 -1 2 ; mkplane pl pp ; explode r sh` | `baddobjects pl bb_1 ; bfillds ; bbuild r` | `-` | BOPAlgo_Builder resets "Closed" flag in the result | CONTAINER |
| 121 | `modalg_6/bug26565_1` | `box b1 10 10 10 ; explode b1 f ; explode b1 e ; box b2 10 5 5 ; explode r` | `bclearobjects ; bcleartools ; baddobjects f e ; baddtools b2 ; bfillds ; bbop r 0 ; bbop r 2` | `-` | Compsolid after cut becomes compound of solids | CONTAINER |
| 122 | `modalg_6/bug26565_2` | `box b1 10 10 10 ; box b2 5 0 0 10 10 10 ; explode b2 sh ; box b3 5 0 5 10 10 10 ; explode r` | `bclearobjects ; bcleartools ; baddobjects b1 b2 ; baddtools b3 ; bfillds ; bbop r 0 ; bbop r 2` | `-` | Compsolid after cut becomes compound of solids | CONTAINER |
| 123 | `modalg_6/bug26565_3` | `box b1 10 10 10 ; explode b1 f ; explode b1 e ; box b2 10 5 5 ; shape sh sh ; shape w w ; explode r` | `bclearobjects ; bcleartools ; baddobjects f e ; baddtools b2 ; bfillds ; bbop r 0 ; bbop r 2` | `-` | Compsolid after cut becomes compound of solids | CONTAINER |
| 124 | `modalg_6/bug26565_4` | `box b1 10 10 10 ; box b2 10 0 0 10 5 5 ; box b3 10 5 0 10 5 5 ; box b4 10 0 5 10 10 5 ; explode r so ; shape b1 CS ; box b2 5 0 2 10 10 10 ; explode r` | `bclearobjects ; bcleartools ; baddobjects b1 b2 b3 b4 ; bfillds ; bbuild r ; bclearobjects ; bcleartools ; baddobjects b1 ; baddtools b2 ; bfillds ; bbop r 2` | `-` | Compsolid after cut becomes compound of solids | CONTAINER |
| 125 | `modalg_6/bug26565_5` | `box b1 10 10 10 ; box b2 10 0 0 10 5 5 ; box b3 10 5 0 10 5 5 ; box b4 10 0 5 10 10 5 ; explode r so ; shape b1 CS ; box b2 5 0 2 10 10 5 ; box b3 5 0 7 10 10 5 ; explode r so ; shape b2 CS ; box b3 -5 2 3 30 6 4 ; explode r` | `bclearobjects ; bcleartools ; baddobjects b1 b2 b3 b4 ; bfillds ; bbuild r ; bclearobjects ; bcleartools ; baddobjects b2 b3 ; bfillds ; bbuild r ; bclearobjects ; bcleartools ; ...[16 cmds]` | `-` | Compsolid after cut becomes compound of solids | CONTAINER |
| 126 | `modalg_6/bug26565_6` | `box b1 10 10 10 ; box b2 10 0 0 10 5 5 ; box b3 10 5 0 10 5 5 ; box b4 10 0 5 10 10 5 ; explode r so ; shape b1 CS ; box b2 5 0 2 10 10 5 ; box b3 5 0 7 10 10 5 ; explode r so ; shape b2 CS ; box b3 5 -2 -2 10 14 20 ; explode r` | `bclearobjects ; bcleartools ; baddobjects b1 b2 b3 b4 ; bfillds ; bbuild r ; bclearobjects ; bcleartools ; baddobjects b2 b3 ; bfillds ; bbuild r ; bclearobjects ; bcleartools ; ...[16 cmds]` | `-` | Compsolid after cut becomes compound of solids | CONTAINER |
| 127 | `modalg_6/bug26565_7` | `box b1 10 10 10 ; box b2 10 0 0 10 5 5 ; box b3 10 5 0 10 5 5 ; box b4 10 0 5 10 10 5 ; explode r so ; shape b1 CS ; box b2 5 0 2 10 10 10 ; box b3 -5 2 3 30 6 4 ; explode r` | `bclearobjects ; bcleartools ; baddobjects b1 b2 b3 b4 ; bfillds ; bbuild r ; bclearobjects ; bcleartools ; baddobjects b1 b2 ; baddtools b3 ; bfillds ; bbop r 2` | `-` | Compsolid after cut becomes compound of solids | CONTAINER |
| 128 | `modalg_6/bug26565_8` | `box b 10 10 10 ; explode b f ; shape profile Sh ; polyline spine 0 0 0 10 10 0 ; pipe sh spine profile ; plane pl 10 10 0 -1 -1 0 1 0 0 ; mkface f pl ; shape sl So ; shape sl_sh Sh ; explode res` | `bcut res sh sl` | `-` | Compsolid after cut becomes compound of solids | CONTAINER |
| 129 | `modalg_6/bug26796` | `box b1 10 10 10 ; box b2 10.00000018 0 0 10 10 10 ; explode b2 f ; trotate f 0 5 5 0 1 0 10 ; mksurface s f ; mkface f s -100 100 -100 100` | `bclearobjects ; bcleartools ; baddobjects b1 b2 f ; bfillds ; bbuild result ; bopcheck result` | `checkshape result ; checknbshapes result -solid 4 ; checkprops result -v 2852.9 -s 41403.1` | General Fuse operation error | TOL |
| 130 | `modalg_6/bug27270` | `box b1 200 200 200 ; explode b1 w ; compound b1_1 b1_2 b1_3 b1_4 b1_5 b1_6 c1 ; tcopy c1 c2 ; ttranslate c2 100 0 300` | `bfuse result c1 c2` | `checkprops result -l 9600 ; checkshape result` | Boolean operations: incorrect assembling of sub-shapes in container shapes (wires, shells, compsolids) | CONTAINER |
| 131 | `modalg_6/bug27274` | `pcylinder b1 50 145 ; tcopy b1 b2 ; trotate b2 0 0 0 1 0 0 45 ; VERTEX    : 5 ; EDGE      : 11 ; WIRE      : 10 ; FACE      : 10 ; SHELL     : 3 ; SOLID     : 3 ; COMPSOLID : 0 ; COMPOUND  : 1 ; SHAPE     : 43 ; "` | `bclearobjects ; bcleartools ; baddobjects b1 ; baddtools b2 ; bfillds ; bbuild result` | `checkshape result ; checknbshapes result -ref ${nbshapes_expected} -t ; checkprops result -s 154518` | Wrong result of General Fuse operation on two cylinders | WRONGRESULT |
| 132 | `modalg_6/bug27383_1` | `box b1 10 10 10 ; box b2 5 5 0 10 10 10 ; explode r f ; explode r_7 e ; encoderegularity r` | `bfuse r b1 b2` | `-` | Modeling - improve handling of regularity on edges | TANGENT |
| 133 | `modalg_6/bug27383_7` | `pcylinder p1 2 10 180 ; pcylinder p2 2 10 180 ; tmirror p2 0 0 0 0 1 0 ; nurbsconvert q p2 ; psphere s1 2 ; ttranslate s1 0 0 10 ; psphere s2 2 ; trotate s2 0 0 0 0 1 0 -90 ; encoderegularity r` | `bclearobjects ; bcleartools ; baddobjects p1 p2 ; baddtools s1 s2 ; bfillds ; bbop r 1` | `-` | Modeling - improve handling of regularity on edges | TANGENT |
| 134 | `modalg_6/bug27773` | `circle c 1 0 0 20 ; offset c1 c 10 0 0 1 ; mkedge e1 c1 ; vertex v1 0 35 0 ; vertex v2 0 -35 0 ; edge e2 v1 v2` | `bsection result e1 e2` | `checknbshapes result -vertex 2` | Empty result of section operation between line and offset of a circle | EMPTY |
| 135 | `modalg_6/bug27878_6` | `compound sh ; box b 1 1 1 ; ttranslate b $i $j 0 ; nurbsconvert b1 sh ; ttranslate b2 0.5 0.5 1` | `bglue 2 ; bclearobjects ; bcleartools ; baddobjects {*}[explode b1 so] ; bfillds -t ; bcbuild rx ; bcaddall res1 -m 1 -u ; bglue 1 ; bop res1 b2 ; bopfuse result` | `checkshape result ; checknbshapes result -face 78 -solid 1 ; checkprops result -s 71.5 -v 32` | Development of the Gluing operations based on the new Boolean component | SAMEDOMAIN |
| 136 | `modalg_6/bug28094` | `psphere a 10 ; plane p2 2 0 0 1 0 0 ; mkface f2 p2 -100 100 -100 100 ; explode a f ; explode s2 e ; splitshape res a_1 a_1 s2_1 a_1 s2_2` | `bsection s2 a f2` | `checkreal "MaxTolerance" ${MaxTolerance} ${expected_MaxTolerance} ${tol_abs_MaxTolerance} ${tol_rel_MaxTolerance}` | Shape obtained after DRAW command "splitshape" has unnecessary high tolerance. | TOL |
| 137 | `modalg_6/bug28189_2` | `box b1 10 10 10 ; box b2 10 10 10` | `bcommon result b1 b2` | `checknbshapes result -wire 6` | Result of Boolean operation is non-manifold wire | CONTAINER |
| 138 | `modalg_6/bug28189_3` | `box b1 10 10 10 ; box b2 10 10 10` | `bfuse result b1 b2` | `checknbshapes result -wire 6` | Result of Boolean operation is non-manifold wire | CONTAINER |
| 139 | `modalg_6/bug28189_4` | `box b1 10 10 10 ; box b2 10 10 10 ; ttranslate b2 5 0 0` | `bcommon result b1 b2` | `checknbshapes result -wire 4` | Result of Boolean operation is non-manifold wire | CONTAINER |
| 140 | `modalg_6/bug28189_5` | `box b1 10 10 10 ; box b2 10 10 10 ; ttranslate b2 5 0 0` | `bfuse result b1 b2` | `checknbshapes result -wire 12` | Result of Boolean operation is non-manifold wire | CONTAINER |
| 141 | `modalg_6/bug28189_7` | `polyline p1 0 0 0 1 0 0 ; polyline p2 0 0 0 1 0 0 ; orientation p1 R ; orientation p2 F ; explode result1 ; explode result2` | `bfuse result1 p1 p2 ; bcommon result2 p2 p1` | `-` | Result of Boolean operation is non-manifold wire | CONTAINER |
| 142 | `modalg_6/bug28189_8` | `box b1 10 10 10 ; box b2 5 5 5 10 10 10 ; shape c1 Cs ; shape c2 Cs` | `bfuse result c1 c2` | `checknbshapes result -compsolid 1 -solid 1` | Result of Boolean operation is non-manifold wire | CONTAINER |
| 143 | `modalg_6/bug28189_9` | `box b1 10 10 10 ; box b2 5 5 5 10 10 10 ; box b3 20 0 0 10 10 10 ; box b4 25 5 5 10 10 10 ; shape c1 Cs ; shape c2 Cs ; explode result` | `bclearobjects ; bcleartools ; baddobjects c1 b3 ; baddtools c2 b4 ; bfillds ; bbop result 1` | `checknbshapes result -solid 2 -compsolid 1` | Result of Boolean operation is non-manifold wire | CONTAINER |
| 144 | `modalg_6/bug28284_3` | `box b1 10 10 10 ; explode b1 sh ; shape b2 So` | `bop b1 b2 ; bopcommon rcom ; bopfuse rfuse ; bopcut rcut ; boptuc rtuc` | `checknbshapes rcom -solid 1 ; checknbshapes rfuse -solid 1 ; checknbshapes rcut -solid 0 ; checknbshapes rcut -solid 0` | Avoid classification of sub-shapes of arguments of BOPs relatively solids during Intersection phase | DEGEN |
| 145 | `modalg_6/bug28486_4` | `plane p 0 0 0 0 0 1 ; mkface f1 p -10 10 -10 10 ; trotate f2 0 0 0 1 0 0 20 ; line l 0 0 0 0 1 0 ; mkedge e l -8 8 ; trotate e 0 0 0 1 0 0 10 ; settolerance e 1.5` | `bclearobjects ; bcleartools ; baddobjects f1 f2 ; baddtools e ; bfillds ; bbuild result` | `checkshape result ; checknbshapes result -face 4 -wire 8 ; checkprops result -s 800` | Fuse of several solids fails due to presence of common zones between faces | TOL |
| 146 | `modalg_6/bug28626_2` | `cone c1 0 0 0  0 -1 0  45  0 ; cone c2 -23 -20 10  1 0 0  45  0 ; trimv tc1 c1 0 42.4264068711929 ; trimv tc2 c2 0 42.4264068711929 ; mkface f1 tc1 ; mkface f2 tc2` | `bop f1 f2 ; bopsection result` | `checkshape result ; checknbshapes result -edge 3 -vertex 4 ; checkmaxtol result -ref 5.21731e-007 ; checkprops result -l 88.9692` | Boolean CUT operation fails due to exception while intersecting two conical faces | CRASH |
| 147 | `modalg_6/bug28626_3` | `cone c1 0 0 0  0 -1 0  45  0 ; cone c2 -22 -20 10  1 0 0  45  0 ; trimv tc1 c1 0 42.4264068711929 ; trimv tc2 c2 0 42.4264068711929 ; mkface f1 tc1 ; mkface f2 tc2` | `bop f1 f2 ; bopsection result` | `checkshape result ; checknbshapes result -edge 4 -vertex 5 ; checkmaxtol result -ref 6.02982e-007 ; checkprops result -l 94.3164` | Boolean CUT operation fails due to exception while intersecting two conical faces | CRASH |
| 148 | `modalg_6/bug28775` | `vertex v1 1 0 0 ; polyline p1 0 0 0 2 0 0 ; polyline p2 1 0 -1 1 0 1` | `bnondestructive 1 ; bclearobjects ; bcleartools ; baddobjects v1 p1 p2 ; bfillds ; bbuild result` | `checkshape result ; checknbshapes result -edge 4` | Code duplication removal across the BOPAlgo_PaveFiller algorithm | DEGEN |
| 149 | `modalg_6/bug28776` | `box b1 10 10 10 ; box b2 10 10 10 ; box b3 10 10 10 ; explode r v ; explode r e ; explode r f ; explode r so` | `bclearobjects ; bcleartools ; baddobjects b1 b2 b3 ; bfillds ; bbuild r` | `checknbshapes r_1_or -vertex 3 ; checknbshapes r_1_or -edge 3 ; checknbshapes r_1_or -face 3 ; checknbshapes r_1_or -solid 3` | Extend the field BOPAlgo_Builder::myOrigins so that the shape could have multiple origins | API |
| 150 | `modalg_6/bug28795` | `box mb -0.5 -0.5 -0.5 1 1 1 ; explode mb F ; prism pryz mb_1 1 0 0 SemiInf ; box ab 0 -1 -1 2 2 2 ; explode ab f ; explode pryz f` | `bsection rs ab_2 pryz_1` | `checkreal "[lindex $expected $i 0]" [lindex $bounds $i] [lindex $expected $i 1] 0.0 1.0e-7 ; checkreal "[lindex $expected $i 0]" [lindex $bounds $i] [lindex $expected $i 1] 0.0 1.0e-7` | Boolean operations corrupt the p-curve of the source planar face if "non-destructive" option is switched off | PCURVE |
| 151 | `modalg_6/bug8040` | `circle c 1 0 0 20 ; offset c1 c 10 0 0 1 ; offset c2 c -10 0 0 1 ; mkedge e c ; mkedge e1 c1 ; mkedge e2 c2 ; vertex v1 0 35 0 ; vertex v2 0 -35 0 ; edge e3 v1 v2` | `bop $s e3 ; bopsection result$index` | `checklength c -l 125.66370614359172 ; checklength c1 -l 188.49555921538757 ; checklength c2 -l 62.831853071795862` | Offset direction is wrong for a closed circle | WRONGRESULT |
| 152 | `modalg_7/bug21264` | `box b1 10 10 10 ; box b2 5 5 5 10 10 10 ; isTracked $log ; isTracked $log ; isTracked $log ; isTracked $log ; isTracked $log ; isTracked $log ; isTracked $log ; isTracked $log ; isTracked $log ; isTracked $log ; isTracked $log ; isTracked $log ; isTracked $log ; isTracked $log ; isTracked $log ; ...[29 cmds]` | `bclearobjects ; bcleartools ; baddobjects b1 ; baddtools b2` | `-` | Modeling Algorithms - Progress indicator for Boolean operations | API |
| 153 | `modalg_7/bug22750` | `ellipse e1 0 0 30 20 ; ellipse e2 0 0  20 10 ; ellipse e3  0 0 16 8 ; to3d ee1 e1 ; to3d ee2 e2 ; translate ee2 0 0 100 ; to3d ee3 e3 ; translate ee3 0 0 200 ; rotate ee2 0 0 100 0 1 0 10 ; rotate ee3 0 0 200 0 1 0 20 ; appro c1 16 ee1 ; appro c2 16 ee2 ; appro c3 16 ee3 ; appsurf s1 c1 c2 c3 ; ...[41 cmds]` | `bop so1 so2 ; bopfuse rrr ; bop rrr so3 ; bopfuse result` | `checkshape so1 ; checkshape so2 ; checkshape rrr ; checknbshapes rrr -face 5 -wire 5 -shell 1 -solid 1 ; checkprops rrr -s 57448.9 -v 664969 ; checkshape result ; ...[8 cmds]` | Boolean operation: Bug in Fuse | WRONGRESULT |
| 154 | `modalg_7/bug23902` **[TODO]** | `bsplinecurve r 4 3 1 5 2 2 3 5 0 8 0 1 2 8 2 1 4 8 3 1 4 8 3 1 4 8 3 1 6 8 4 1 10 8 10 1 ; mkedge e1 r ; vertex v1 10 8 10 ; vertex v2 5 8 8 ; edge e2 v1 v2 ; vertex v3 0 8 0 ; edge e3 v2 v3 ; wire w e1 e2 e3 ; mkplane f w ; prism p f 0 5 0 ; explode p e` | `-` | `-` | Cannot build fillet | DOWNSTREAM |
| 155 | `modalg_7/bug24632_3` | `box b1 10 10 10 ; box b2 10 0 0 10 10 10 ; plane p -10 5 5 1 0 0 ; pcylinder cut p 2.5 40` | `bclearobjects ; bcleartools ; baddobjects b1 b2 ; bfillds ; bbuild c ; bop c cut ; bopcut result` | `checkshape cut ; checkshape c ; checkshape result ; checknbshapes result -solid 2 -shell 2 -face 13 -wire 16 ; checkprops result -v 1607.3` | Artifacts during Boolean cut operation on neighboring parts | SAMEDOMAIN |
| 156 | `modalg_7/bug24692` **[TODO]** | `pcylinder cyl 10 40 ; box abox 10 40 25 ; ttranslate abox 0 -20 5 ; psphere asphere 7 ; ttranslate asphere 0 0 3 ; box abox2 5 30 25 ; ttranslate abox2 -2 -15 -11 ; fuseedge result` | `bop cyl abox ; bopfuse fuse1 ; bop abox2 asphere ; bopfuse fuse2 ; bop fuse1 fuse2 ; bopcut result` | `checkshape result_1` | fuseedge corruption of spherical edge | DOWNSTREAM |
| 157 | `modalg_7/bug24905` | `circle c1 0 0 0 0 -1 0 75 ; circle c2 0 0 0 0 -1 0 65 ; mkedge c1 c1 ; mkedge c2 c2 ; wire c1 c1 ; wire c2 c2 ; orientation c2 R ; mkplane f1 c1 1 ; prism p1 f1 0 1000 0 ; trotate p1 0 0 0 0 0 1 180 ; circle c3 50 500 -2000 0 0 1 50 ; mkedge c3 c3 ; wire c3 c3 ; mkplane f2 c3 1 ; ...[17 cmds]` | `bclearobjects ; bcleartools ; baddobjects p1 ; baddtools p2 ; bfillds ; bbop r_0 0 ; bbop r_1 1 ; bbop r_2 2 ; bbop r_3 3 ; bbop r_4 4` | `checkshape r_$i ; checknbshapes r_0 -solid 1 -shell 1 -face 5 -wire 6 ; checkprops r_0 -s 196255 -v 857818 ; checknbshapes r_1 -solid 1 -shell 1 -face 9 -wire 13 ; ...[13 cmds]` | Boolean cut produced invalid result | INVALID |
| 158 | `modalg_7/bug25395_2` **[TODO]** | `ellipse Ellipse-curve 0 0 0 100.87 22 ; mkedge Ellipse-edge Ellipse-curve 0 6.28318530718 ; wire Ellipse-wire Ellipse-edge ; mkplane Ellipse Ellipse-wire ; ttranslate Ellipse 0 -27 184.5 ; explode Ellipse E ; wire Sweep-0-spine Ellipse_1 ; mksweep Sweep-0-spine ; setsweep -CF ; ...[24 cmds]` | `bcut Cut031 Sweep Box001 ; bcut Cut032 Cut031 Box002` | `-` | SIGSEGV in BRepOffsetAPI_MakeThickSolid | DOWNSTREAM |
| 159 | `modalg_7/bug25478_2` **[TODO]** | `box Box001 10 10 10 ; box Box002 10 10 12 ; ttranslate Box002 5 5 -1 ; explode Cut E` | `bcut Cut Box001 Box002` | `-` | Fillets can not touch | DOWNSTREAM |
| 160 | `modalg_7/bug25879` | `box Box 10 10 10 ; explode Box E ; blend Fillet Box 1 Box_1 1 Box_2 1 Box_3 1 Box_4 1 Box_5 1 Box_6 1 Box_7 1 Box_8 1 Box_9 1 Box_10 1 Box_11 1 Box_12` | `-` | `-` | result of blend fails the bopcheck | DOWNSTREAM |
| 161 | `modalg_7/bug25939` | `psphere Sphere 5 ; pcylinder Cylinder 83 100 ; ttranslate Cylinder -83 0 -50 ; explode Cut F ; offsetparameter 1e-7 p a ; offsetload Cut -2 Cut_4` | `bcut Cut Sphere Cylinder` | `-` | S I G S E G V in MakeThickSolid | DOWNSTREAM |
| 162 | `modalg_7/bug25968` | `pcylinder Cylinder004 50 1000 ; pcylinder Cylinder005 45 1000 ; pcylinder Cylinder006 45 102 ; trotate Cylinder006 0 0 0 1 0 0 90 ; ttranslate Cylinder006 0 51 1000 ; pcylinder Cylinder007 45 102 ; trotate Cylinder007 0 0 0 1 0 0 90 ; ttranslate Cylinder007 0 51 0 ; ttranslate Cut005 0 110 0 ; ...[57 cmds]` | `bcut Cut003 Cylinder004 Cylinder005 ; bcut Cut004 Cut003 Cylinder006 ; bcut Cut005 Cut004 Cylinder007 ; bcut Cut032 Cylinder044 Cylinder045 ; bcut Cut033 Cylinder046 Cylinder047 ; ...[22 cmds]` | `checkshape Fusion001 ; checkprops Fusion001 -s 1.52134e+006 ; checknbshapes Fusion001 -vertex 73 -edge 111 -wire 42 -face 32 -shell 1 -solid 1` | boolean returning an invalid shape | INVALID |
| 163 | `modalg_7/bug27378` **[TODO]** | `cylinder cyl 10 ; mkface fcyl cyl pi 3*pi -10 10 ; sphere sph 10 ; trimu spht sph pi 3*pi ; mkface fsph spht ; cone con 30 0 ; mkface fcon con pi 3*pi 0 20 ; torus tor 20 5 ; mkface ftor1 tor pi 3*pi 0 2*pi ; mkface ftor2 tor 0 2*pi pi 3*pi` | `-` | `checkshape fcyl ; checkshape fsph ; checkshape fcon ; checkshape ftor1 ; checkshape ftor2` | BRepLib_MakeFace produces invalid faces on periodic surfaces in case the given parametrization does not match the default | SEAM |
| 164 | `modalg_7/bug28245` | `circle c1 0 0 0 0 0 1 10 ; circle c2 10 0 0 0 0 1 10 ; circle c3 5 9 0 0 0 1 10 ; mkedge e1 c1 ; mkedge e2 c2 ; mkedge e3 c3 ; wire w1 e1 ; wire w2 e2 ; wire w3 e3 ; mkplane f1 w1 ; mkplane f2 w2 ; mkplane f3 w3` | `bclearobjects ; bcleartools ; baddobjects f1 f2 f3 ; bfillds ; bcbuild rx ; bcadd res f1 1 -m 1 ; bcadd res f2 1 f3 0 -m 1 ; bcadd res f3 1 f1 0 -m 2 ; bopcheck res ; bcremoveint res` | `-` | Result of Cells Builder algorithm becomes invalid after removal of internal boundaries on faces | SAMEDOMAIN |
| 165 | `modalg_7/bug28485_1` | `box b1 10 10 10 ; box b2 10 2 2 6 6 6 ; nurbsconvert b1 b1 ; nurbsconvert b2 b2` | `bnondestructive 1 ; brunparallel 1 ; bfuzzyvalue 0 ; bglue 1 ; bclearobjects ; bcleartools ; baddobjects b1 ; baddtools b2 ; bfillds ; bbop result 1` | `checkshape result ; checknbshapes result -vertex 16 -edge 24 -wire 12 -face 11 -shell 1 -solid 1 ; checkprops result -s 744 -v 1216` | Fuse of two shapes using gluing and non-destructive options gives invalid result | SAMEDOMAIN |
| 166 | `modalg_7/bug28786_2` | `plane p1 0 0 0 1 0 0 ; mkface f1 p1 -10 10 -10 10 ; plane p2 0 0 0 1 0 1 ; mkface f2 p2 -10 10 -10 10 ; compound f1 f2 a ; line l 0 0 0 0 1 0 ; mkedge b l -11 11` | `bclearobjects ; bcleartools ; baddobjects a ; baddtools b ; bbuild result` | `checkshape result ; checknbshapes result -face 4 -edge 15` | Refactoring of the Warning/Error reporting system of Boolean Operations Algorithm | DEGEN |
| 167 | `modalg_7/bug28786_3` | `vertex v 0 0 0 ; vertex v1 10 0 0 ; vertex v2 0 0 -3 ; vertex v3 5 0 0 ; edge e1 v v1 ; edge e2 v2 v3 ; compound e1 e2 a ; vertex v4 5 0 5 ; vertex v5 5 0 -5 ; edge b v4 v5` | `bclearobjects ; bcleartools ; baddobjects a ; baddtools b ; bbuild result` | `checkshape result ; checknbshapes result -edge 5` | Refactoring of the Warning/Error reporting system of Boolean Operations Algorithm | DEGEN |
| 168 | `modalg_7/bug28786_4` | `vertex v 0 0 0 ; vertex v1 1.e-5 0 0 ; settolerance v1 1.e-5 ; vertex v2 0 1.e-5 0 ; edge a v v1 ; edge b v v2` | `bclearobjects ; bcleartools ; baddobjects a ; baddtools b ; bbuild result` | `checkshape result ; checknbshapes result -edge 2` | Refactoring of the Warning/Error reporting system of Boolean Operations Algorithm | DEGEN |
| 169 | `modalg_7/bug28786_5` | `vertex v 0 0 0 ; vertex v1 10 0 0 ; edge e1 v v1 ; edge e2 v v1 ; compound e1 e2 a ; vertex v3 -5 0 0 ; vertex v4 15 0 0 ; edge b v3 v4` | `bclearobjects ; bcleartools ; baddobjects a ; baddtools b ; bbuild result` | `checkshape result ; checknbshapes result -edge 3` | Refactoring of the Warning/Error reporting system of Boolean Operations Algorithm | DEGEN |
| 170 | `modalg_7/bug28828_1` | `pcylinder a1 10 50 ; box a2 -20 -20 -20  40 40 40 ; nurbsconvert ba2 a2 ; explode a1 f ; explode ee e ; explode a1 e ; chamf_throat result b  a1_1 1.  ee_1 2.` | `bfuse b a1 ba2 ; savehistory hh` | `checkshape result ; checknbshapes result -solid 1 -shell 1 -face 10 -wire 11 -edge 19 -vertex 12 -shape 55 ; checkprops result -v 73639.9 -deps 1.e-7` | New functionalities of BRepFilletAPI_MakeChamfer algorithm | DOWNSTREAM |
| 171 | `modalg_7/bug28828_15` | `pcylinder a1 10 50 ; box a2 -20 -20 -20  25 40 40 ; explode a1 f ; explode ee ; chamf_throat result b ee_3 2.` | `bfuse b a1 a2 ; savehistory hh` | `checkshape result ; checknbshapes result -solid 1 -shell 1 -face 10 -wire 10 -edge 26 -vertex 18 -shape 67 ; checkprops result -v 50822.7 -deps 1.e-7` | New functionalities of BRepFilletAPI_MakeChamfer algorithm | DOWNSTREAM |
| 172 | `modalg_7/bug28828_16` | `pcylinder a1 10 50 ; box a2 -20 -20 -20  25 40 40 ; explode a1 f ; explode ee ; chamf_throat_with_penetration result b  ee_3 ff 1. 1.2  ee_4 ff 1. 1.2` | `bfuse b a1 a2 ; savehistory hh` | `checkshape result ; checknbshapes result -solid 1 -shell 1 -face 11 -wire 11 -edge 31 -vertex 22 -shape 78 ; checkprops result -v 50702.4 -deps 1.e-7` | New functionalities of BRepFilletAPI_MakeChamfer algorithm | DOWNSTREAM |
| 173 | `modalg_7/bug28828_17` | `pcone a1 10 30 50 ; box a2 -25 -25 -20  50 50 40 ; explode a1 f ; chamf_throat result b ee 3.` | `bfuse b a1 a2 ; savehistory hh` | `checkshape result ; checknbshapes result -solid 1 -shell 1 -face 9 -wire 10 -edge 17 -vertex 11 -shape 50 ; checkprops result -v 156170 -deps 1.e-7` | New functionalities of BRepFilletAPI_MakeChamfer algorithm | DOWNSTREAM |
| 174 | `modalg_7/bug28828_18` | `pcone a1 10 30 50 ; box a2 -25 -25 -20  50 50 40 ; explode a1 f ; chamf_throat_with_penetration result b ee ff 3. 6.` | `bfuse b a1 a2 ; savehistory hh` | `checkshape result ; checknbshapes result -solid 1 -shell 1 -face 9 -wire 10 -edge 17 -vertex 11 -shape 50 ; checkprops result -v 156442 -deps 1.e-7` | New functionalities of BRepFilletAPI_MakeChamfer algorithm | DOWNSTREAM |
| 175 | `modalg_7/bug28828_19` | `pcone a1 40 0 70 ; box a2 -50 -50 -20  100 100 40 ; explode a1 f ; chamf_throat result b ee 3.` | `bfuse b a1 a2 ; savehistory hh` | `checkshape result ; checknbshapes result -solid 1 -shell 1 -face 8 -wire 9 -edge 17 -vertex 11 -shape 48 ; checkprops result -v 445625 -deps 1.e-7` | New functionalities of BRepFilletAPI_MakeChamfer algorithm | DOWNSTREAM |
| 176 | `modalg_7/bug28828_2` | `pcylinder a1 10 50 ; ttranslate a1 0 -5 0 ; box a2 -20 -20 -20  40 40 40 ; trotate a2 0 0 0  1 0 0  20 ; explode a1 f ; chamf_throat result b ee 2.` | `bfuse b a1 a2 ; savehistory hh` | `checkshape result ; checknbshapes result -solid 1 -shell 1 -face 9 -wire 10 -edge 17 -vertex 11 -shape 50 ; checkprops result -v 73885.9 -deps 1.e-7` | New functionalities of BRepFilletAPI_MakeChamfer algorithm | DOWNSTREAM |
| 177 | `modalg_7/bug28828_20` | `pcone a1 40 0 70 ; box a2 -50 -50 -20  100 100 40 ; explode a1 f ; chamf_throat_with_penetration result b ee ff 2. 3.` | `bfuse b a1 a2 ; savehistory hh` | `checkshape result ; checknbshapes result -solid 1 -shell 1 -face 8 -wire 9 -edge 17 -vertex 11 -shape 48 ; checkprops result -v 446299 -deps 1.e-7` | New functionalities of BRepFilletAPI_MakeChamfer algorithm | DOWNSTREAM |
| 178 | `modalg_7/bug28828_21` | `pcone a1 40 0 70 ; box a2 -50 -50 -20  100 100 40 ; nurbsconvert b b ; explode b f ; explode b_1 e ; chamf_throat_with_penetration result b b_1_2 b_1 2. 3.` | `bfuse b a1 a2` | `checkshape result ; checknbshapes result -solid 1 -shell 1 -face 8 -wire 9 -edge 17 -vertex 11 -shape 48 ; checkprops result -v 446298 -deps 1.e-7` | New functionalities of BRepFilletAPI_MakeChamfer algorithm | DOWNSTREAM |
| 179 | `modalg_7/bug28828_4` | `pcylinder a1 10 50 ; plane pp 25 0 0  -1 0 0 ; pcylinder a2 pp 15 50 ; explode a1 f ; chamf_throat result b ee 1.` | `bfuse b a1 a2 ; savehistory hh` | `checkshape result ; checknbshapes result -solid 1 -shell 1 -face 6 -wire 7 -edge 8 -vertex 5 -shape 29 ; checkprops result -v 46724.8 -deps 1.e-7` | New functionalities of BRepFilletAPI_MakeChamfer algorithm | DOWNSTREAM |
| 180 | `modalg_7/bug28828_6` | `pcylinder a1 10 50 ; box a2 -20 -20 -20  40 40 40 ; nurbsconvert b a ; explode b f ; explode b_1 e ; chamf_throat_with_penetration result b b_1_2 b_1 1. 2.` | `bfuse a a1 a2` | `checkshape result ; checknbshapes result -solid 1 -shell 1 -face 9 -wire 10 -edge 17 -vertex 11 -shape 50 ; checkprops result -v 73604.1 -deps 1.e-7` | New functionalities of BRepFilletAPI_MakeChamfer algorithm | DOWNSTREAM |
| 181 | `modalg_7/bug28828_7` | `pcylinder a1 10 50 ; ttranslate a1 0 -5 0 ; box a2 -20 -20 -20  40 40 40 ; trotate a2 0 0 0  1 0 0  20 ; explode a1 f ; chamf_throat_with_penetration result b ee ff 1. 2.` | `bfuse b a1 a2 ; savehistory hh` | `checkshape result ; checknbshapes result -solid 1 -shell 1 -face 9 -wire 10 -edge 17 -vertex 11 -shape 50 ; checkprops result -v 73900.4 -deps 1.e-7` | New functionalities of BRepFilletAPI_MakeChamfer algorithm | DOWNSTREAM |
| 182 | `modalg_7/bug28828_8` | `pcylinder a1 10 50 ; box a2 -20 -20 -20  40 40 40 ; explode a1 f ; chamf_throat result b ee 2.` | `bfuse b a1 a2 ; savehistory hh` | `checkshape result ; checknbshapes result -solid 1 -shell 1 -face 9 -wire 10 -edge 17 -vertex 11 -shape 50 ; checkprops result -v 73699.8 -deps 1.e-7` | New functionalities of BRepFilletAPI_MakeChamfer algorithm | DOWNSTREAM |
| 183 | `modalg_7/bug28828_9` | `pcylinder a1 10 50 ; box a2 -20 -20 -20  40 40 40 ; explode a1 f ; chamf_throat_with_penetration result b ee ff 1. 2.` | `bfuse b a1 a2 ; savehistory hh` | `checkshape result ; checknbshapes result -solid 1 -shell 1 -face 9 -wire 10 -edge 17 -vertex 11 -shape 50 ; checkprops result -v 73604.3 -deps 1.e-7` | New functionalities of BRepFilletAPI_MakeChamfer algorithm | DOWNSTREAM |
| 184 | `modalg_7/bug29182_1` | `cylinder c 0 10 0 1 0 0 10 ; mkface fc c pi/3 2*pi/3 -10 10 ; shape a C ; trotate f1 0 0 0 1 0 0 $i*5` | `brunparallel 1 ; bclearobjects ; bcleartools ; baddcompound a` | `-` | BOPAlgo_PaveFiller sometimes raises exception in parallel mode | CRASH |
| 185 | `modalg_7/bug29322_2` | `line line1 0 0 0 0 0 1 ; mkedge e1 line1 0 10 ; line line2 0 0.5 0 0 0 1 ; mkedge e2 line2 0 10 ; vertex v1 0 0.25 0 ; settolerance v1 0.25 ; vertex v2 0 0.25 10 ; settolerance v2 0.25` | `bclearobjects ; bcleartools ; baddobjects e1 e2 v1 v2 ; bfillds ; bbuild result` | `checkshape result ; checknbshapes result -vertex 2 -edge 1 ; checkprops result -l 10` | Unify faces classification procedures in Boolean Operations | TOL |
| 186 | `modalg_7/bug29322_3` | `line line1 0 0 0 0 0 1 ; mkedge e1 line1 0 10 ; line line2 0 0.5 0 0 0 1 ; mkedge e2 line2 0 10 ; vertex v1 0 0.25 0 ; settolerance v1 0.25 ; vertex v2 0 0.25 10 ; settolerance v2 0.25 ; vertex v3 0 0.25 5 ; settolerance v3 0.25` | `bclearobjects ; bcleartools ; baddobjects e1 e2 v1 v2 v3 ; bfillds ; bbuild result` | `checkshape result ; checknbshapes result -vertex 3 -edge 2 ; checkprops result -l 10` | Unify faces classification procedures in Boolean Operations | TOL |
| 187 | `modalg_7/bug29322_4` | `line line1 0 0 0 0 0 1 ; mkedge e1 line1 0 10 ; line line2 0 0.5 0 0 0 1 ; mkedge e2 line2 0 10 ; mkedge e3 line2 -5 15 ; vertex v1 0 0.25 0 ; settolerance v1 0.25 ; vertex v2 0 0.25 10 ; settolerance v2 0.25 ; vertex v3 0 0.25 5 ; settolerance v3 0.25` | `bclearobjects ; bcleartools ; baddobjects e1 e2 e3 v1 v2 v3 ; bfillds ; bbuild result` | `checkshape result ; checknbshapes result -vertex 5 -edge 4 ; checkprops result -l 20` | Unify faces classification procedures in Boolean Operations | TOL |
| 188 | `modalg_7/bug29322_5` | `line line1 0 0 0 0 0 1 ; mkedge e1 line1 0 10 ; line line2 0 0.5 0 0 0 1 ; mkedge e2 line2 0 10 ; mkedge e3 line2 -5 15 ; vertex v1 0 0.25 0 ; settolerance v1 0.25 ; vertex v2 0 0.25 10 ; settolerance v2 0.25 ; vertex v3 0 0.25 5 ; settolerance v3 0.25 ; plane p 0 0 0 0 1 0 ; mkface f p -5 15 -5 5` | `bclearobjects ; bcleartools ; baddobjects f e1 e2 e3 v1 v2 v3 ; bfillds ; bbuild result` | `checkshape result ; checknbshapes result -vertex 9 -edge 8 -wire 2 -face 1 ; checkprops result -l 90 -s 200` | Unify faces classification procedures in Boolean Operations | TOL |
| 189 | `modalg_7/bug29333_1` | `plane p 0 0 0 0 0 1 ; mkface f1 p -10 10 -10 10 ; ttranslate f2 20 0 0 ; explode s f ; line l 0 0 0 1 0 0 ; mkedge e l ; compound result s1_sp c ; compound result s1_sp c` | `bfuse s f1 f2 ; bclearobjects ; bcleartools ; baddobjects s_1 ; baddtools e ; bfillds ; bsplit s1_sp ; bclearobjects ; bcleartools ; baddobjects s1_sp ; baddtools s_2 ; bfillds ; ...[19 cmds]` | `checkshape result ; checkprops result -s 800 ; checknbshapes result -vertex 8 -edge 10 -wire 3 -face 3 ; checknbshapes c -vertex 8 -edge 10 -wire 3 -face 3 ; checkshape result ; ...[8 cmds]` | Boolean Operations - Prevent modification of the input shapes in case their sub-shapes have not been modified | API |
| 190 | `modalg_7/bug29333_2` | `box b1 10 10 10 ; box b2 10 0 0 10 10 10 ; mkvolume s b1 b2 ; explode s so ; plane p 0 0 5 0 0 1 ; mkface f p ; compound result s1_sp c ; compound result s1_sp c` | `bclearobjects ; bcleartools ; baddobjects s_1 ; baddtools f ; bfillds ; bsplit s1_sp ; bclearobjects ; bcleartools ; baddobjects s1_sp ; baddtools s_2 ; bfillds ; bbuild result ; ...[18 cmds]` | `checkshape result ; checkprops result -s 1400 -v 2000 ; checknbshapes result -vertex 16 -edge 28 -wire 16 -face 16 -shell 3 -solid 3 ; ...[8 cmds]` | Boolean Operations - Prevent modification of the input shapes in case their sub-shapes have not been modified | API |
| 191 | `modalg_7/bug29580_2` | `circle c1 0 -5 0 1 0 0 10 ; circle c2 0 5 0 1 0 0 10 ; mkedge e1 c1 ; mkedge e2 c2 ; wire w1 e1 ; wire w2 e2 ; mkplane f1 w1 ; mkplane f2 w2 ; revol b1 f12 0 0 0 0 0 1 180 ; ttranslate b1 0 31.358955689999998 0 ; plane pln 0 0 0 0 0 1 1.1102230246251565e-016 -1 0 ; ...[13 cmds]` | `bcut f12 f1 f2 ; bclearobjects ; bcleartools ; baddobjects b1 ; baddtools b2 ; bfillds ; bbuild result` | `checkshape result ; checknbshapes result -vertex 4 -edge 16 -wire 10 -face 10 -shell 3 -solid 3 -t ; checkprops result -s 3879.55 -v 6295.15` | Regression: invalid result of BOP Fuse | INVALID |
| 192 | `modalg_7/bug29586` | `circle c1 0 -5 0 1 0 0 10 ; circle c2 0 5 0 1 0 0 10 ; mkedge e1 c1 ; mkedge e2 c2 ; wire w1 e1 ; wire w2 e2 ; mkplane f1 w1 ; mkplane f2 w2 ; revol r f12 0 0 0 0 0 1 180 ; explode r f ; explode r_1 e ; explode r_2 e ; explode r_3 e ; explode f12 v` | `bcut f12 f1 f2 ; savehistory h` | `checknbshapes g1 -edge 2 ; checknbshapes g2 -edge 1 ; checknbshapes g3 -edge 2` | Revolution creates solid with degenerated edges shared among faces | DEGEN |
| 193 | `modalg_7/bug29734` | `upvar ${shape} ${shape} ; "lprops"    { set mass "length" } ; "sprops"    { set mass "area" } ; "vprops"    { set mass "volume" } ; compmass $mass $props1 $props2 ; compmoms $props1 $props2 ; box b1 1 2 3 ; box b2 3 2 1 ; ttranslate b2 .5 .5 .5 ; trotate b2 0 0 0 1 1 1 30 ; incmesh ff .01 ; ...[12 cmds]` | `bfuse ff b1 b2` | `-` | Compute global properties of tessellated shape | DOWNSTREAM |
| 194 | `modalg_7/bug29807_svm01` | `cone con_f2 0 518.47000000000003 0 0 -1 1.1102230246251565e-016 14.999999999912038 0 ; mkface f2 con_f2 0 6.2831853071795862 0 1000000 ; cone con_f6 0 -440.74363604000001 0 0 -1 1.1102230246251565e-016 45.110284878807235 0 ; mkface f6 con_f6 0 6.2831853071795862 0 1000000` | `bsection result f2 f6` | `checksection result -r 0 ; checkmaxtol result -ref 7.3189259943803184e-007 ; checkprops result -l 2202.91 ; checknbshapes result -vertex 1 -edge 1 ; checkshape result` | Impossible to cut cone from prism | TANGENT |
| 195 | `modalg_7/bug29807_svm02` | `cone con_f1 0 -60.919306349999999 0 0 -1 1.1102230246251565e-016 28.800000000062262 0 ; mkface f1 con_f1 0 6.2831853071795862 0 1000000 ; cone con_f5 0 -309.47272469000001 0 0 -1 1.1102230246251565e-016 43.999999999485127 0 ; mkface f5 con_f5 0 6.2831853071795862 0 1000000` | `bsection result f1 f5` | `checksection result -r 0 ; checkmaxtol result -ref 6.6226289034767669e-007 ; checkprops result -l 1993.34 ; checknbshapes result -vertex 1 -edge 1 ; checkshape result` | Impossible to cut cone from prism | TANGENT |
| 196 | `modalg_7/bug29910_1` **[TODO]** | `ptorus t1 100 10 ; ptorus t2 100 10 ; ttranslate t2 100 0 0` | `bclearobjects ; bcleartools ; baddobjects t1 ; baddtools t2 ; bfillds ; bbop rs 4 ; bbuild result` | `checksection rs -r 0 ; checkshape rs ; checkmaxtol rs -ref 7.5e-6 ; checknbshapes rs -edge 14 ; checkprops rs -l 330.096 ; checknbshapes result -shell 6 -solid 6 ; checkshape result ; ...[8 cmds]` | Porting to Debian80-64 : Regressions in Modeling Algorithms | ORDER |
| 197 | `modalg_7/bug29973_2` | `box b1 10 10 10 ; box b2 5 5 5 10 10 10 ; compound c` | `bclearobjects ; bcleartools ; baddobjects b1 b2 ; baddtools c ; bfillds ; bbop r_0 0 ; bbop r_1 1 ; bbop r_2 2 ; bbop r_3 3 ; bbop r_4 4 ; bclearobjects ; bcleartools ; baddobjects c ; ...[20 cmds]` | `checkshape r_0 ; checknbshapes r_0 -shape 1 ; checkprops r_0 -s empty -v empty ; checkshape r_1 ; checknbshapes r_1 -wire 12 -face 12 -solid 1 -shell 1 ; checkprops r_1 -s 1050 -v 1875 ; ...[30 cmds]` | Modeling Algorithms - access violation within BRepAlgoAPI_Cut on empty input shape | DEGEN |
| 198 | `modalg_7/bug30150` | `box b1 10 10 10 ; explode b1 f ; shape sx Sh ; box b2 -50 -50 30 100 100 10 ; explode b2 f ; draft r1 sx 0 0 1 0.2 b2 ; draft r2 sx 0 0 1 0.2 b2_6 ; draft r3 sx 0 0 1 0.2 b2 -OUT` | `-` | `checkshape $r ; checkprops r1 -s 25319.3 -v 105060 ; checknbshapes r1 -vertex 20 -edge 32 -wire 16 -face 15 -shell 1 -solid 1 ; checkprops r2 -s 11977.8 ; ...[7 cmds]` | Modeling Algorithms - Removal of BRepAlgo_BooleanOperations and BRepAlgo_DSAccess classes | DOWNSTREAM |
| 199 | `modalg_7/bug30154_1` | `plane p 165424660 597500 42834196 -1 0 0 ; mkface f p ; box b -94190864 -46229000 -17178478.4 519231048 93653000 120025348.8` | `bcut result f b` | `checkshape result ; checkprops result -s 1.12407e+16 ; checknbshapes result -wire 1 -face 1` | Modeling Algorithms - Boolean Operation on planar geometry hangs inside BRepLib::FindValidRange() | HANG_PERF |
| 200 | `modalg_7/bug30206` | `box b1 10 10 10 ; box b2 5 5 5 10 10 10` | `bclearobjects ; bcleartools ; baddobjects b1 ; baddtools b2 ; bfillds ; bcommon r0 b1 b2 ; bbop r0_1 0 ; bbop r0_2 common ; bapibop r0_3 0 ; bapibop r0_4 common ; bfuse r1 b1 b2 ; ...[32 cmds]` | `checknbshapes r0_1 -ref [nbshapes r0] ; checknbshapes r0_2 -ref [nbshapes r0] ; checknbshapes r0_3 -ref [nbshapes r0] ; checknbshapes r0_4 -ref [nbshapes r0] ; ...[22 cmds]` | Improve API of commands bbop and bapibop | API |
| 201 | `modalg_7/bug30386_4` | `vertex v 0 0 2 ; line l 5 5 0 0 0 1 ; mkedge e l -10 10 ; plane p 0 0 1 0 0 1 ; mkface f p -15 15 -15 15 ; box b 8 8 8 5 5 5 ; box s2 10 10 10 ; explode r0 so ; explode r2 so` | `bclearobjects ; bcleartools ; baddobjects v e f b ; bfillds ; bbuild s1 ; bclearobjects ; bcleartools ; baddobjects s1 ; baddtools s2 ; bfillds ; bbop r0 0 ; bbop r2 2` | `checkshape $r ; checknbshapes r0 -vertex 16 -edge 18 -wire 7 -face 7 -shell 1 -solid 1 -t ; checkprops r0 -s 124 ; checkprops r0_1 -v 8 ; ...[7 cmds]` | Modeling Algorithms - Unable to perform Cut operation | DEGEN |
| 202 | `modalg_7/bug30386_5` | `vertex v 0 0 2 ; line l 5 5 0 0 0 1 ; mkedge e l -10 10 ; plane p 0 0 1 0 0 1 ; mkface f p -15 15 -15 15 ; box b 8 8 8 5 5 5 ; box b2 10 10 10` | `bclearobjects ; bcleartools ; baddobjects v e f b ; bfillds ; bbuild s1 ; bclearobjects ; bcleartools ; baddobjects f ; baddtools b2 ; bfillds ; bbuild s2 ; bclearobjects ; bcleartools ; ...[16 cmds]` | `-` | REQUIRED All: Error: Boolean operation of the given type is not allowed on the given inputs | DEGEN |
| 203 | `modalg_7/bug31404` | `psphere sph1 25 ; plane pln2 32, 68, -27 ; psphere sph2 pln2 75 ; explode sph1 f` | `bfuse f1 sph1 sph2 ; savehistory h1 ; bfuse f2 sph2 sph1 ; savehistory h2` | `checkshape $r ; checknbshapes f1 -wire 3 -face 2 -edge 9 -vertex 6 ; checknbshapes f2 -wire 3 -face 2 -edge 9 -vertex 6 ; checkprops f1 -s 74192.6 ; checkprops f2 -s 74192.6 ; ...[7 cmds]` | Modeling Algorithms - BOP Fuse produces a self-interfering or a good shape depending on the order of arguments | ORDER |
| 204 | `modalg_7/bug31460` | `circle cir 3 2.9999999999989 0 0 0 -1 -1 0 0 3 ; trim cir cir 4.71238898038469 6.28318530717959 ; mkedge e cir ; revol r10 e 0 0 0 1 0 0 10 1 ; revol r360 e 0 0 0 1 0 0 360 1` | `-` | `-` | Modeling Algorithms - Regression: Revolution not done. | TANGENT |
| 205 | `modalg_7/bug31470` | `plane pln1 51.899912462 99.996691888 62.33204004 ; psphere s1 pln1 15 ; psphere s2 pln1 10 ; box bb 100 100 100 ; explode ss f ; trim pc pc` | `bcut ss s1 s2 ; bcommon result bb ss ; savehistory hh` | `checkshape result ; checknbshapes result -solid 1 -shell 1 -face 3 -wire 6 -edge 8 -vertex 6 ; checkprops result -v 4975.49` | BOP common produces empty result (box and holed sphere) | EMPTY |
| 206 | `modalg_7/bug31835_1` | `plane p1 -200 2.22044604925033e-14 12.6935294289015 1 -1.11022302462516e-16 0 ; pcylinder c1 p1 100 400 ; plane p2 0 0 -187.306470571099 0 0 1 ; pcylinder c2 p2 100 400 ; plane p3 0 200 12.6935294289015 0 -1 1.11022302462516e-16 ; pcylinder c3 p3 100 400` | `bcommon r c1 c2 ; bcommon result r c3 ; bclearobjects ; bcleartools ; baddobjects c1 c2 c3 ; bfillds ; bcbuild r ; bcremoveall ; bcadd result1 c1 1 c2 1 c3 1` | `checkshape r ; checkmaxtol r -ref 2.e-7 ; checknbshapes r -wire 5 -face 5 -shell 1 -solid 1 ; checkprops r -s 160000 -v 5.33333e+06 ; checkshape result ; checkmaxtol result -ref 2.e-7 ; ...[11 cmds]` | Modeling Algorithms - step by step Boolean common produces bad shape on given three cylinders | INVALID |
| 207 | `modalg_7/bug31835_2` | `plane p1 -200 2.22044604925033e-14 12.6935294289015 1 -1.11022302462516e-16 0 ; pcylinder c1 p1 100 400 ; plane p2 0 0 -187.306470571099 0 0 1 ; pcylinder c2 p2 100 400 ; plane p3 0 200 12.6935294289015 0 -1 1.11022302462516e-16 ; pcylinder c3 p3 100 400` | `bfuzzyvalue 1.e-6 ; bnondestructive 1 ; brunparallel 1 ; setfillhistory 0 ; bcommon r c1 c2 ; bcommon result r c3 ; bclearobjects ; bcleartools ; baddobjects c1 c2 c3 ; bfillds ; ...[14 cmds]` | `checkshape r ; checkmaxtol r -ref 2.e-7 ; checknbshapes r -wire 5 -face 5 -shell 1 -solid 1 ; checkprops r -s 160000 -v 5.33333e+06 ; checkshape result ; checkmaxtol result -ref 2.e-7 ; ...[11 cmds]` | Modeling Algorithms - step by step Boolean common produces bad shape on given three cylinders | INVALID |
| 208 | `modalg_7/bug31836` | `pcone c 282.842712474619 100 300 ; ttranslate c 100 0 0 ; ptorus t 80.2535587508467 19.7464412491533 ; ttranslate t 100 0 300 ; explode s` | `bfuzzyvalue 1.e-6 ; brunparallel 1 ; bnondestructive 1 ; setfillhistory 0 ; bclearobjects ; bcleartools ; baddobjects c ; baddtools t ; bfillds ; bbop r_0 0 ; bbop r_1 1 ; bbop r_2 2 ; ...[16 cmds]` | `checkshape r_$i ; checkprops r_0 -s 51195.4 -v 308846 ; checknbshapes r_0 -wire 3 -face 2 -shell 1 -solid 1 -t ; checkprops r_1 -s 716665 -v 3.74689e+07 ; ...[14 cmds]` | regression issue - boolean cut between a cone and a Torus is not done | TANGENT |
| 209 | `modalg_7/bug31858_1` | `pcylinder c 500 200 ; plane p -2.31895142368858e-15 -4.49296838339662e-15 200 0 0 1 ; ptorus t p 449.367136080235 50.6328639197654` | `bclearobjects ; bcleartools ; baddobjects c ; baddtools t ; bfillds ; bbop r0 0 ; bbop r1 1 ; bbop r2 2 ; bbop r3 3 ; bbop r4 4 ; bbuild rgf` | `checkshape $r ; checknbshapes r0 -wire 3 -face 2 -shell 1 -solid 1 -t ; checkprops r0 -s 735041 -v 1.13701e+07 ; checknbshapes r1 -wire 4 -face 4 -shell 1 -solid 1 -t ; ...[13 cmds]` | Modeling Algorithms - boolean operation crash between the given cylinder and torus. | TANGENT |
| 210 | `modalg_7/bug31858_2` | `pcylinder c 500 200 ; plane p -2.31895142368858e-15 -4.49296838339662e-15 200 0 0 1 ; ptorus t p 449.367136080235 50.6328639197654` | `bfuzzyvalue 1.e-6 ; bnondestructive 1 ; brunparallel 1 ; setfillhistory 0 ; bclearobjects ; bcleartools ; baddobjects c ; baddtools t ; bfillds ; bbop r0 0 ; bbop r1 1 ; bbop r2 2 ; ...[16 cmds]` | `checkshape $r ; checknbshapes r0 -wire 3 -face 2 -shell 1 -solid 1 -t ; checkprops r0 -s 735041 -v 1.13701e+07 ; checknbshapes r1 -wire 4 -face 4 -shell 1 -solid 1 -t ; ...[13 cmds]` | Modeling Algorithms - boolean operation crash between the given cylinder and torus. | TANGENT |
| 211 | `modalg_7/bug32470` | `psphere sph 10 ; box b 20 100 100 ; ttranslate b 0 -50 -50 ; trotate b 0 0 10 0 -1 0 45` | `bcut result sph b` | `checkshape result ; checknbshapes result -t -vertex 3 -edge 5 -wire 2 -face 2 -shell 1 -solid 1 ; checkprops result -s 1229.69 -v 3945.6` | BOP wrong result on sphere and box | WRONGRESULT |
| 212 | `modalg_7/bug32502` | `psphere sph 10 ; plane pln 0 0 7 0 1 0 0 0 1 ; pcylinder cyl pln 3 40 ; ttranslate cyl 0 -20 0` | `bop sph cyl ; bopfuse result` | `checkshape result ; checknbshapes result -t -vertex 4 -edge 11 -wire 5 -face 5 -shell 1 -solid 1 ; checkprops result -s 1750.1 -v 4947.2` | BOP wrong result on sphere and cylinder | WRONGRESULT |
| 213 | `modalg_8/bug32716_1` | `psphere s1 10 ; pcylinder s2 8 20 ; pcone s3 10 8 5 ; ttranslate s1 0 0 25 ; ttranslate s3 0 0 -5 ; incmesh s 0.1 ; tscale s 0 0 0  2 -copymesh ; tmirror s 1 0 0  1 1 1 -copymesh ; ttranslate s 0 0 10 ; trotate s 0 0 0  0 0 1  45` | `baddobjects s1 ; baddtools s2 s3 ; bfillds ; bbop s fuse` | `checkshape s ; checktrinfo s -face $nbFaces -nod $nbNodes -tri $nbTri ; checkshape s ; checktrinfo s -face $nbFaces -nod $nbNodes -tri $nbTri ; checkshape s ; ...[8 cmds]` | Modeling Algorithms - BRepBuilderAPI_Transform discards triangulation | DOWNSTREAM |
| 214 | `modalg_8/bug32716_2` | `psphere s1 10 ; pcylinder s2 8 20 ; pcone s3 10 8 5 ; ttranslate s1 0 0 25 ; ttranslate s3 0 0 -5 ; incmesh s 0.1 ; tscale s 0 0 0  2 -copy -copymesh ; tmirror s 1 0 0  1 1 1 -copy -copymesh ; ttranslate s 0 0 10 -copy -copymesh ; trotate s 0 0 0  0 0 1  45  -copy -copymesh` | `baddobjects s1 ; baddtools s2 s3 ; bfillds ; bbop s fuse` | `checkshape s ; checktrinfo s -face $nbFaces -nod $nbNodes -tri $nbTri ; checkshape s ; checktrinfo s -face $nbFaces -nod $nbNodes -tri $nbTri ; checkshape s ; ...[8 cmds]` | Modeling Algorithms - BRepBuilderAPI_Transform discards triangulation | DOWNSTREAM |
| 215 | `modalg_8/bug32716_3` | `psphere s1 10 ; pcylinder s2 8 20 ; pcone s3 10 8 5 ; ttranslate s1 0 0 25 ; ttranslate s3 0 0 -5 ; incmesh s 0.1 ; tclean -geom s ; tscale s 0 0 0  2 -copymesh ; tmirror s 1 0 0  1 1 1 -copymesh ; ttranslate s 0 0 10 ; trotate s 0 0 0  0 0 1  45` | `baddobjects s1 ; baddtools s2 s3 ; bfillds ; bbop s fuse` | `checktrinfo s -face $nbFaces -nod $nbNodes -tri $nbTri ; checktrinfo s -face $nbFaces -nod $nbNodes -tri $nbTri ; checktrinfo s -face $nbFaces -nod $nbNodes -tri $nbTri ; ...[4 cmds]` | Modeling Algorithms - BRepBuilderAPI_Transform discards triangulation | DOWNSTREAM |
| 216 | `modalg_8/bug32716_4` | `psphere s1 10 ; pcylinder s2 8 20 ; pcone s3 10 8 5 ; ttranslate s1 0 0 25 ; ttranslate s3 0 0 -5 ; incmesh s 0.1 ; tclean -geom s ; tscale s 0 0 0  2 -copy -copymesh ; tmirror s 1 0 0  1 1 1 -copy -copymesh ; ttranslate s 0 0 10 -copy -copymesh ; trotate s 0 0 0  0 0 1  45  -copy -copymesh` | `baddobjects s1 ; baddtools s2 s3 ; bfillds ; bbop s fuse` | `checktrinfo s -face $nbFaces -nod $nbNodes -tri $nbTri ; checktrinfo s -face $nbFaces -nod $nbNodes -tri $nbTri ; checktrinfo s -face $nbFaces -nod $nbNodes -tri $nbTri ; ...[4 cmds]` | Modeling Algorithms - BRepBuilderAPI_Transform discards triangulation | DOWNSTREAM |
| 217 | `modalg_8/bug33361` | `vertex v1 -13.1319809553115 86.7822037750006 -0.00566973476865229 ; vertex v2 -128.558636709231 86.5345485791916 -0.00570258843333704 ; vertex v3 -100.69915263562 -4.39311321186025 -0.00566084285433277 ; vertex v4 -79.6175928285065 -4.39310160958954 -0.0056548256445943 ; ...[136 cmds]` | `bfuse result s1 s2 ; bfuse result result s3 ; bfuse result result s4` | `checkshape result` | Modeling Algorithm - Fuse operation generates incomplete result | WRONGRESULT |
| 218 | `modalg_8/bug33541` | `pcylinder c 1 1 ; psphere s 1 ; ttranslate s 1 1 1` | `bcut result c s` | `checknbshapes result -vertex 3 -edge 5 -wire 4 -face 4` | Modeling Algorithms - Simple sphere cut from cylinder fails | TANGENT |
| 219 | `modalg_8/bug33570` | `psphere s1 4 ; psphere s2 1 ; ttranslate s1 0 5.5 18 ; ttranslate s2 0 4.5 14` | `bcut result s1 s2` | `checknbshapes result -vertex 4 -edge 7 -wire 2 -face 2` | Modeling Algorithms - Incorrect result of cutting spheres | WRONGRESULT |
| 220 | `modalg_8/bug33615` | `plane plane -5 0 4 -1 0 0 ; pcone cone plane 3 1 10 ; pcylinder cylinder 10 20 ; explode cylinder f ; explode cone f` | `bclearobjects ; bcleartools ; baddobjects cone_1 ; baddtools cylinder_1 ; bfillds ; bbuild result` | `checknbshapes result -vertex 5 -edge 8 -wire 5 -face 4` | Modeling Algorithms - Partition algorithm creates unexpected vertices | WRONGRESULT |
| 221 | `modalg_8/bug_gh501` **[TODO]** | `vertex vEdgeBackRight 0 0 0 ; vertex vEdgeBackLeft xEdgeBackLeft 0 0 ; edge edgeBack vEdgeBackLeft vEdgeBackRight ; circle circleBack xCircleBack 0 0 radiusCircleBack ; mkedge edgeCircleBack circleBack angleCircleBackStart 0 ; settolerance edgeCircleBack tolerance ; ...[31 cmds]` | `bop solidBottom solidTop ; bopfuse fuse` | `-` | This test case is to reproduce the crash during blend operation | DOWNSTREAM |
| 222 | `moddata_1/bug17046` | `ellipse ell 0 0 300 1 0 0 150 100 ; mkedge ell_e ell ; wire ell_w ell_e ; mkplane ell_f ell_w 1 ; box ell_t -10 -110 140 20 110 460 ; revol ell_z ell_d 0 0 0 0 0 1 360 ; explode ell_z F ; mksurface su1 ell_z_1` | `bcut ell_d ell_f ell_t` | `-` | Exception in Extrema_ExtPS on Mandriva2006 32-bits | DOWNSTREAM |
| 223 | `moddata_1/bug217` | `box b -5 -5 -5 10 10 10 ; psphere s 6` | `bfuse result s b` | `checkshape result ; checkprops result -s 618.85` | nurbsconvert problem | WRONGRESULT |
| 224 | `moddata_2/bug259` | `pcylinder b1 1 2 ; box b2 -r -r 0 2*r 2*r 2 ; trotate b2 0 0 0 0 0 1 135` | `bfuse result b2 b1` | `checkshape -top result ; checkprops result -s 18.8496` | Inequality behaviour of checkshape -top on SunOS and IRIX. | TANGENT |
| 225 | `moddata_2/bug2784_1` | `box b1 10 20 10 ; box b2 1 1 1 ; ttranslate b2 5 5 0 ; explode b1 f ; explode b2 f` | `bcut rcut1 b1_5 b2_5 ; bcommon result rcut1 b2_5` | `checkshape rcut1` | Common and cut operations for two faces gives invalid result | SAMEDOMAIN |
| 226 | `moddata_2/bug2784_2` | `box b1 10 20 10 ; box b2 1 1 1 ; ttranslate b2 5 5 0 ; explode b1 f ; explode b2 f` | `bcut rcut1 b1_5 b2_5 ; bcommon result rcut1 b2_5` | `checkshape rcut1` | Common and cut operations for two faces gives invalid result | SAMEDOMAIN |
| 227 | `moddata_2/bug408` | `box b1 0 0 0 1 1 1 ; nurbsconvert b1 b1 ; box b2  0    -0.5   0     1.5   0.5   0.5` | `bcommon result b1 b2` | `checkshape -top result` | (no comment in file) | SAMEDOMAIN |
| 228 | `moddata_2/bug6450_1` | `plane pl 100 0 0 1 0 0 ; mkface f pl ; halfspace so1 f 0 0 0 ; ttranslate f -200 0 0 ; halfspace so2 f 0 0 0` | `bcommon result so1 so2` | `-` | Cannot get common part of two parallel halfspaces | HALFSPACE |
| 229 | `moddata_2/bug6450_2` | `plane pl 100 0 0 1 0 0 ; mkface f pl ; halfspace so1 f 0 0 0 ; ttranslate f -200 0 0 ; trotate f 0 0 0 0 0 1 45 ; halfspace so2 f 0 0 0` | `bcommon result so1 so2` | `-` | Cannot get common part of two parallel halfspaces | HALFSPACE |
| 230 | `moddata_2/bug6862_7` | `box b1 10 20 10 ; box b2 20 10 10 ; explode b1 f ; explode b2 f ; explode r1 w ; explode r2 w ; mksurface gs b1_6 ; mkface ff gs` | `bcut r1 b1_6 b2_6 ; bcommon r2 b1_6 b2_6` | `-` | Tool for checking arguments of Boolean Operations | API |
| 231 | `moddata_3/bug31587_1` | `box b 10 10 10 ; box b1 2 2 2 3 2 1 ; explode b1 f ; removeinternals r` | `bclearobjects ; bcleartools ; baddobjects b ; baddtools b1_1 ; bfillds ; bsplit r` | `checkshape r ; checknbshapes r -vertex 8 -edge 12 -wire 6 -face 6 -shell 1 -solid 1` | Modeling Data - Remove internal sub-shapes from the shape | CONTAINER |
| 232 | `moddata_3/bug31587_2` | `box b 10 10 10 ; box b1 2 2 0 3 2 1 ; explode b1 f ; removeinternals r` | `bclearobjects ; bcleartools ; baddobjects b ; baddtools b1_1 ; bfillds ; bsplit r` | `checkshape r ; checknbshapes r -vertex 8 -edge 12 -wire 6 -face 6 -shell 1 -solid 1` | Modeling Data - Remove internal sub-shapes from the shape | CONTAINER |
| 233 | `moddata_3/bug31587_3` | `box b 10 10 10 ; box b1 2 2 -1 3 2 2 ; explode b1 f ; removeinternals r ; removeinternals r1 0 ; removeinternals r1 1 ; explode r1 so` | `bclearobjects ; bcleartools ; baddobjects b ; baddtools b1_1 b1_2 ; bfillds ; bsplit r ; bbuild r ; bbuild r1` | `checkshape r ; checknbshapes r -vertex 8 -edge 12 -wire 6 -face 6 -shell 1 -solid 1 ; checkshape r1 ; checknbshapes r1 -ref [nbshapes r] ; checkshape r1 ; ...[7 cmds]` | Modeling Data - Remove internal sub-shapes from the shape | CONTAINER |
| 234 | `moddata_3/bug31587_4` | `box b 10 10 10 ; box b1 2 2 0 5 5 5 ; explode b1 f ; removeinternals r` | `bclearobjects ; bcleartools ; baddobjects b ; baddtools b1_2 b1_3 ; bfillds ; bsplit r` | `checkshape r ; checknbshapes r -vertex 8 -edge 12 -wire 6 -face 6 -shell 1 -solid 1` | Modeling Data - Remove internal sub-shapes from the shape | CONTAINER |
| 235 | `moddata_3/bug31587_5` | `box b 10 10 10 ; box b1 2 2 0 5 5 5 ; box b2 3 2 2 4 5 2 ; explode b1 f ; explode b2 f ; removeinternals r` | `bclearobjects ; bcleartools ; baddobjects b ; baddtools b1_2 b1_3 ; baddtools b2_5 b2_6 ; bfillds ; bsplit r` | `checkshape r ; checknbshapes r -vertex 8 -edge 12 -wire 6 -face 6 -shell 1 -solid 1` | Modeling Data - Remove internal sub-shapes from the shape | CONTAINER |
| 236 | `moddata_3/bug31587_6` | `box b 10 10 10 ; polyline e1 2 2 0 8 2 0 ; polyline e2 3 2 -5 3 2 0 ; compound r e2 c ; removeinternals c` | `bclearobjects ; bcleartools ; baddobjects b ; baddtools e1 e2 ; bfillds ; bsplit r` | `checkshape c ; checknbshapes c -vertex 10 -edge 13 -wire 7 -face 6 -shell 1 -solid 1` | Modeling Data - Remove internal sub-shapes from the shape | CONTAINER |
| 237 | `moddata_3/bug33049` | `psphere s1 10 ; pcylinder s2 8 20 ; pcone s3 10 8 5 ; ttranslate s1 0 0 25 ; ttranslate s3 0 0 -5 ; incmesh s 0.1 ; tclean -geom s` | `baddobjects s1 ; baddtools s2 s3 ; bfillds ; bbop s fuse` | `-` | Command 'tclean -geom' does not remove all geometric objects from the shape | API |

---

## 3. DEFECT TAXONOMY — what a production kernel regression-tests

Every one of the 237 self-contained cases is assigned exactly **one** primary class
(the `Tax` column above; sum = 237). Where the file states its own defect line, that line drove the
assignment; 9 files carry no descriptive comment at all
(`modalg_1/buc60801`, `modalg_2/bug297_1..4`, `modalg_2/bug407_1`, `modalg_2/bug407_2`,
`modalg_2/bug422_2`, `moddata_2/bug408`) and were classed from their geometry + assertion.

| # | Class | count | share | what it means |
|---|---|---:|---:|---|
| 1 | `DOWNSTREAM` | 39 | 16.5% | boolean is the *input generator*; the assertion targets fillet/chamfer/draft/offset/thick-solid/mesh/OCAF/transform |
| 2 | `WRONGRESULT` | 31 | 13.1% | operation completes, result has wrong area/volume/length or wrong shape counts |
| 3 | `CONTAINER` | 28 | 11.8% | topology *metadata* wrong: `Closed` flag, compsolid→compound demotion, wire/shell assembly, orientation, internal sub-shapes, non-manifold wire |
| 4 | `DEGEN` | 26 | 11.0% | degenerate / lower-dimensional / touching-only arguments (vertex∩vertex, vertex∩edge, corner-touch, empty compound, mixed-dimension compound, degenerated edges) |
| 5 | `SAMEDOMAIN` | 22 | 9.3% | coincident / coplanar / same-domain faces, gluing, face unification after BOP |
| 6 | `TANGENT` | 16 | 6.8% | tangential or coincident-surface intersection (cone∩cone, cyl∩cone, cyl∩torus, cyl∩cyl, sphere∩box) |
| 7 | `TOL` | 12 | 5.1% | tolerance / fuzzy value / near-coincident-within-tolerance |
| 8 | `HANG_PERF` | 10 | 4.2% | hang, or wall-clock/scale budget on huge argument sets |
| 9 | `HALFSPACE` | 9 | 3.8% | semi-infinite (halfspace) operands |
| 10 | `API` | 9 | 3.8% | API/infrastructure: progress, warning system, history, non-destructive mode, checker commands |
| 11 | `CRASH` | 8 | 3.4% | exception / SIGSEGV / `Standard_NoSuchObject` / `NumericError` |
| 12 | `INVALID` | 7 | 3.0% | operation completes but result is self-intersecting / fails `checkshape` or `bopcheck` |
| 13 | `ORDER` | 6 | 2.5% | result depends on argument order, or differs between platforms |
| 14 | `LEAK` | 5 | 2.1% | memory leak across repeated BOPs |
| 15 | `SEAM` | 3 | 1.3% | seam / periodic-surface parametrization |
| 16 | `EMPTY` | 3 | 1.3% | operation returns nothing where a result exists |
| 17 | `PCURVE` | 2 | 0.8% | p-curve / UV-bounds of the *input* corrupted by the operation |
| 18 | `OPENSECTION` | 1 | 0.4% | section that should close does not |
| | **total** | **237** | | |

Excluding the three classes where the boolean is not the thing under test
(`DOWNSTREAM` 39, `API` 9, `LEAK` 5), the **core-BOP defect corpus is 184 cases**.
Re-normalised over those 184: `WRONGRESULT` 16.8%, `CONTAINER` 15.2%, `DEGEN` 14.1%,
`SAMEDOMAIN` 12.0%, `TANGENT` 8.7%, `TOL` 6.5%, `HANG_PERF` 5.4%, `HALFSPACE` 4.9%,
`CRASH` 4.3%, `INVALID` 3.8%, `ORDER` 3.3%, `SEAM`/`EMPTY` 1.6% each, `PCURVE` 1.1%,
`OPENSECTION` 0.5%.

### 3.1 Per-class case lists (auditable)

**DOWNSTREAM (39)** — `caf/bug31918_2`, `mesh/bug25142`, `mesh/bug25157`, `mesh/bug29751`,
`mesh/bug30008_2`, `mesh/bug31926`, `modalg_2/bug422_1`, `modalg_2/bug422_2`,
`modalg_5/bug24758_1`, `modalg_5/bug24758_2`, `modalg_5/bug25657_1`, `modalg_7/bug23902`,
`modalg_7/bug24692`, `modalg_7/bug25395_2`, `modalg_7/bug25478_2`, `modalg_7/bug25879`,
`modalg_7/bug25939`, `modalg_7/bug28828_{1,2,4,6,7,8,9,15,16,17,18,19,20,21}` (14 chamfer cases),
`modalg_7/bug29734`, `modalg_7/bug30150`, `modalg_8/bug32716_{1,2,3,4}`, `modalg_8/bug_gh501`,
`moddata_1/bug17046`.

**WRONGRESULT (31)** — `modalg_1/buc60668`, `modalg_1/buc60801`, `modalg_1/bug12507`,
`modalg_1/bug17194_1`, `modalg_1/bug17194_2`, `modalg_2/bug407_1`, `modalg_2/bug407_2`,
`modalg_2/bug4717_{1,2,3,4,7,8}`, `modalg_4/bug693`, `modalg_4/bug693_1`, `modalg_4/bug7626_1`,
`modalg_4/bug7626_2`, `modalg_5/bug21564`, `modalg_5/bug23402`, `modalg_5/bug23881`,
`modalg_5/bug24187`, `modalg_5/bug25715_1`, `modalg_6/bug27274`, `modalg_6/bug8040`,
`modalg_7/bug22750`, `modalg_7/bug32470`, `modalg_7/bug32502`, `modalg_8/bug33361`,
`modalg_8/bug33570`, `modalg_8/bug33615`, `moddata_1/bug217`.

**CONTAINER (28)** — `heal/bug30927`, `modalg_5/bug24033`, `modalg_5/bug24706`,
`modalg_6/bug26420_{1,2,3}`, `modalg_6/bug26565_{1..8}`, `modalg_6/bug27270`,
`modalg_6/bug28189_{2,3,4,5,7,8,9}`, `moddata_3/bug31587_{1..6}`.

**DEGEN (26)** — `modalg_5/bug24060`, `modalg_5/bug24597`,
`modalg_5/bug25354_{32,33,34,35,36,37,38,39,40,41,45,46,47,48}` (14 touching-contact section cases),
`modalg_6/bug28284_3`, `modalg_6/bug28775`, `modalg_7/bug28786_{2,3,4,5}`, `modalg_7/bug29586`,
`modalg_7/bug29973_2`, `modalg_7/bug30386_4`, `modalg_7/bug30386_5`.

**SAMEDOMAIN (22)** — `heal/bug26219_gehause_rohteil`, `heal/bug26244`, `heal/bug29382_3`,
`heal/bug33171_1`, `heal/bug33171_2`, `heal/bug33421`, `modalg_1/bug13538`,
`modalg_2/bug2785_1`, `modalg_2/bug2785_2`, `modalg_2/bug2986_1`, `modalg_2/bug2986_2`,
`modalg_5/bug23855`, `modalg_5/bug25242`, `modalg_5/bug25337_1`, `modalg_5/bug25337_2`,
`modalg_6/bug27878_6`, `modalg_7/bug24632_3`, `modalg_7/bug28245`, `modalg_7/bug28485_1`,
`moddata_2/bug2784_1`, `moddata_2/bug2784_2`, `moddata_2/bug408`.

**TANGENT (16)** — `modalg_1/bug13211_{2,3,4,6}`, `modalg_5/bug23876`, `modalg_6/bug14531`,
`modalg_6/bug27383_1`, `modalg_6/bug27383_7`, `modalg_7/bug29807_svm01`,
`modalg_7/bug29807_svm02`, `modalg_7/bug31460`, `modalg_7/bug31836`, `modalg_7/bug31858_1`,
`modalg_7/bug31858_2`, `modalg_8/bug33541`, `moddata_2/bug259`.

**TOL (12)** — `heal/bug29544_2`, `modalg_5/bug25477_1`, `modalg_5/bug25722`,
`modalg_6/bug24803`, `modalg_6/bug25880`, `modalg_6/bug26796`, `modalg_6/bug28094`,
`modalg_6/bug28486_4`, `modalg_7/bug29322_{2,3,4,5}`.

**HANG_PERF (10)** — `modalg_5/bug24157_{4,5,6,7,8,10}`, `modalg_5/bug24639`,
`modalg_5/bug25354_49`, `modalg_5/bug25354_50`, `modalg_7/bug30154_1`.

**HALFSPACE (9)** — `modalg_2/bug297_{1,2,3,4}`, `modalg_6/bug25937_{1,2,3}`,
`moddata_2/bug6450_1`, `moddata_2/bug6450_2`.

**API (9)** — `modalg_5/bug24029`, `modalg_5/bug25801`, `modalg_6/bug28776`,
`modalg_7/bug21264`, `modalg_7/bug29333_1`, `modalg_7/bug29333_2`, `modalg_7/bug30206`,
`moddata_2/bug6862_7`, `moddata_3/bug33049`.

**CRASH (8)** — `modalg_1/buc60899_1`, `modalg_5/bug22027`, `modalg_5/bug23932_1`,
`modalg_5/bug23932_2`, `modalg_5/bug24359`, `modalg_6/bug28626_2`, `modalg_6/bug28626_3`,
`modalg_7/bug29182_1`.

**INVALID (7)** — `modalg_5/bug24328`, `modalg_5/bug24620`, `modalg_7/bug24905`,
`modalg_7/bug25968`, `modalg_7/bug29580_2`, `modalg_7/bug31835_1`, `modalg_7/bug31835_2`.

**ORDER (6)** — `modalg_5/bug24618_{1,2,3,4}`, `modalg_7/bug29910_1`, `modalg_7/bug31404`.

**LEAK (5)** — `fclasses/bug7287_{1,2,4,5,6}`.

**SEAM (3)** — `heal/bug29502`, `modalg_2/bug22990`, `modalg_7/bug27378`.
**EMPTY (3)** — `modalg_1/buc60899_2`, `modalg_6/bug27773`, `modalg_7/bug31470`.
**PCURVE (2)** — `modalg_5/bug24404`, `modalg_6/bug28795`.
**OPENSECTION (1)** — `modalg_1/buc60901`.

### 3.2 Independent keyword census (description text only, all three sets)

Regexes run over comment lines and `puts` header strings only (not over commands, so
`settolerance`/`orientation`/`tolerance` command names do not inflate the counts). A case can
match more than one keyword, so columns do not sum to the case count.

| keyword group | self-contained (237) | external-data (1015) | all boolean (1252) |
|---|---:|---:|---:|
| wrong / incorrect / bad / invalid / faulty result | 89 (38%) | 384 (38%) | 473 (38%) |
| crash / exception / SIGSEGV | 13 (5%) | 66 (7%) | 79 (6%) |
| self-intersecting / self-interfering | 8 (3%) | 60 (6%) | 68 (5%) |
| tolerance / fuzzy | 24 (10%) | 53 (5%) | 77 (6%) |
| same-domain / coincident / coplanar / glue | 12 (5%) | 45 (4%) | 57 (5%) |
| regression | 14 (6%) | 46 (5%) | 60 (5%) |
| empty / missing / not done | 8 (3%) | 35 (3%) | 43 (3%) |
| hang / performance / slow | 10 (4%) | 32 (3%) | 42 (3%) |
| closed flag / compsolid / non-manifold | 24 (10%) | 5 (0%) | 29 (2%) |
| unifysamedomain / face unification | 12 (5%) | 13 (1%) | 25 (2%) |
| tangent / tangency | 1 (0%) | 7 (1%) | 8 (1%) |
| seam / periodic | 3 (1%) | 1 (0%) | 4 (0%) |
| degenerated edge / vertex | 1 (0%) | 2 (0%) | 3 (0%) |

The self-contained and external-data corpora have the **same** headline profile (38% "wrong
result" in both, crash 5–7% in both, regression 5–6% in both). That matters: it means the
self-contained subset is not a soft skew of the corpus — the 237 free cases are representative of
the 1252, so running them is a real proxy for the whole bug history.

### 3.3 What the taxonomy actually says

1. **"Wrong result" dominates, not "crash".** 38% of every boolean bug case, self-contained or
   not, is *silently wrong output*, versus 5–7% crashes. A production kernel spends the bulk of
   its regression budget pinning exact area/volume/shape counts, not on not-falling-over. Of the
   106 self-contained `checkprops` users, 68 pin an exact area, 34 an exact volume, 27 an exact
   length; 135 of 237 pin exact `nbshapes`.
2. **Topology *metadata* is a first-class defect class** (`CONTAINER`, 28 cases, 15% of the
   core-BOP corpus, and only 5 of 1015 external cases mention it — it is almost entirely a
   procedural-case concern). Losing the `Closed` flag on a shell/wire, demoting a compsolid to a
   compound, mis-assembling wires/shells, leaving stray `INTERNAL` orientations, failing to strip
   internal sub-shapes: these get eight (`bug26565_1..8`), seven (`bug28189_*`), six
   (`bug31587_1..6`) and three (`bug26420_1..3`) dedicated cases respectively. The boolean can
   produce geometrically correct faces and still be a bug.
3. **Degenerate/touching contact is over-represented relative to intuition** (26 cases, 14% of
   core). `bug25354_32..48` alone is 14 cases whose whole content is: two boxes touching at one
   corner, at one edge, offset by exactly the box size; a vertex sectioned against a coincident
   vertex; a vertex against an edge that passes through it; a vertex inside/outside a box. All of
   them assert the exact `nbshapes` of a section that is a single vertex or a single edge. OCCT
   treats "the answer is one point" as a first-class correctness requirement.
4. **Same-domain / coincident faces is the single largest *geometric* class** (22 cases, 12% of
   core) and it recurs across two decades: `OCC2784`/`OCC2785`/`OCC2986` (2 faces sharing a
   domain, one with a hole touching the other's boundary), `OCC25337` (two coplanar circular
   faces), `OCC23855` (two identical spheres), `OCC25242` (`bfuse a b` then `bcut` by `a`),
   `OCC27878`/`OCC28485` (glue mode), and six `unifysamedom` follow-ups in `heal/`.
5. **Tangency is a small case count but a high-severity one** (16 cases). The instances are
   exactly the pairs whose SSI degenerates: cone∩cone with equal half-angle (`bug13211_*`,
   `bug28626_*`, `bug29807_*`), cylinder∩cone (`bug14531`), cylinder∩torus tuned to touch
   (`bug31858_*`: `pcylinder c 500 200` vs `ptorus t 449.367136080235 50.6328639197654`, whose
   radii sum to 500.0000000000004 — tangency to 4e-13), cone∩torus (`bug31836`), a box whose four
   vertical edges lie exactly on a cylinder (`moddata_2/bug259`: half-size sqrt(2)/2 rotated 135°,
   corner distance from the axis = 1.0 = the cylinder radius), a revolution whose profile circle
   is tangent to the axis to 1.1e-12 (`bug31460`: `circle cir 3 2.9999999999989 0 … 3` revolved
   about X), and a sphere tangent to a cylinder's base plane (`bug33541`: `pcylinder c 1 1`,
   `psphere s 1` at (1,1,1) — touches the z=0 plane at a single point and is centred exactly in
   the z=1 top-face plane). Note the shape of the evidence: these are not "roughly tangent"
   configurations, they are tangency **pinned to the last representable digit**. Whoever filed
   them was cornering the SSI deliberately.
6. **Tolerance work is explicit and parameterised**, not implicit: `bfuzzyvalue` appears in 33 of
   the 1252 boolean cases and 7 of the 237 self-contained ones, always with a gap deliberately set
   just below the fuzzy value (`bug25477_1`: gap 1e-5, fuzzy 2e-5; `bug25722`: gap 1e-4, fuzzy
   1e-4; `bug26796`: gap 1.8e-7 with no fuzzy at all).
7. **Seam/periodic is only 3 self-contained cases and 1 external** — surprisingly few for how much
   trouble it causes. Note `modalg_7/bug27378` (still TODO) is a seam case: OCCT cannot even build
   a *valid face* on a periodic surface when the given U-range is `pi..3*pi` instead of
   `0..2*pi`, for cylinder, sphere, cone and torus alike.
8. **Halfspace / semi-infinite operands get a dedicated 9 cases.** Cutting an edge, a face and a
   solid by a halfspace (`bug25937_1/2/3`), intersecting two parallel halfspaces
   (`bug6450_1/2`) — infinite operands are treated as an ordinary supported input, not an edge case.
9. **The failure-mode distribution is a build order.** Ranked by core-BOP weight the classes are:
   wrong metric/count → container metadata → degenerate contact → same-domain → tangency →
   tolerance → scale → halfspace → crash → self-intersection → ordering. That is close to the
   inverse of the order in which a from-scratch kernel usually gets them right.

---

## 4. CASES MARKED `TODO` — what a mature kernel still cannot do

`TODO` in an OCCT test declares an *expected failure*: the listed message is tolerated so CI stays
green. `TODO ?…` means "expected on some platforms/intermittently"; plain `TODO … ALL:` means
"always fails, everywhere".

* **92 of the 1252 boolean bug cases (7.3%) carry a `TODO`.**
* **19 of the 237 self-contained** (8.0%) and **73 of the 1015 external-data** (7.2%).

### 4.1 The 19 self-contained TODO cases, verbatim

| case | TODO line(s) | class |
|---|---|---|
| `fclasses/bug7287_1` | `TODO ?СК33225 Linux: Checking trend failed`; `TODO ?OCC7287 Linux/MacOS: Tcl Exception: Memory leak detected`; `TEST INCOMPLETE` | LEAK |
| `fclasses/bug7287_2` | `TODO ?OCC7287 Linux/MacOS: Tcl Exception: Memory leak detected`; `TEST INCOMPLETE` | LEAK |
| `fclasses/bug7287_4` | `TODO ?OCC7287 Linux: Checking trend failed`; `Memory leak detected`; `TEST INCOMPLETE` | LEAK |
| `fclasses/bug7287_5` | `TODO ?CR332257287 Linux: Checking trend failed: mean delta per step = 9432.0, sigma = 4220.01…, expected delta = 0` | LEAK |
| `fclasses/bug7287_6` | `TODO ?CR33225 Linux: Checking trend failed: mean delta per step =`; `Memory leak detected` | LEAK |
| `heal/bug33421` | `TODO CR33439 ALL: Error : is WRONG because number of EDGE / WIRE / FACE entities in shape` | SAMEDOMAIN |
| `modalg_2/bug4717_1` | `TODO OCC12345 ALL: Error : is WRONG because number of` | WRONGRESULT |
| `modalg_2/bug4717_2` | idem | WRONGRESULT |
| `modalg_2/bug4717_3` | idem | WRONGRESULT |
| `modalg_2/bug4717_4` | idem | WRONGRESULT |
| `modalg_2/bug4717_7` | idem | WRONGRESULT |
| `modalg_2/bug4717_8` | idem | WRONGRESULT |
| `modalg_7/bug23902` | `TODO OCC23902 ALL: Error: Cannot build fillet` | DOWNSTREAM |
| `modalg_7/bug24692` | `TODO OCC24692 ALL: Faulty shapes in variables faulty_1 to faulty_` | DOWNSTREAM |
| `modalg_7/bug25395_2` | `TODO OCC25395 ALL: Standard_ConstructionError`; `TEST INCOMPLETE` | DOWNSTREAM |
| `modalg_7/bug25478_2` | `TODO OCC25478 ALL: Error: Fillets can not touch` | DOWNSTREAM |
| `modalg_7/bug27378` | `TODO OCC27378 ALL: Error: BRepLib_MakeFace produces invalid faces on periodic surfaces` | SEAM |
| `modalg_7/bug29910_1` | `TODO ?OCC29910 Linux:` wrong EDGE count in `rs` (16), wrong SHELL/SOLID count in `result` (5), wrong area, wrong volume | ORDER |
| `modalg_8/bug_gh501` | `TODO ALL: Tcl Exception: tolerance ang`; `TEST INCOMPLETE` | DOWNSTREAM |

### 4.2 Clustered TODO messages across all 92 boolean TODO cases

The 92 cases carry **156 `TODO` lines** in total. Measured breakdown of those 156:

| pattern | lines |
|---|---:|
| `Error : … is WRONG because number of <ENTITY> entities …` | **50** |
| `Error : The area / length / volume of result shape is …` | **29** |
| `TEST INCOMPLETE` | 12 |
| `Tcl Exception: Memory leak detected` | 10 |
| `Faulty shapes in variables faulty_1 to faulty_` | 6 |
| `ERROR: OCC<nnnnn> is reproduced.` | 4 |
| `Error : The command is not valid` (± length/area) | 6 |
| everything else (fillet/offset/tolerance/section one-offs) | ~39 |

Scope markers: **122** of the 156 lines are `… ALL:` (fails on every platform); **30** are
`TODO ?…` (platform-conditional / intermittent).

Notable one-offs in the "everything else" bucket:
`Error : the resulting shape is unclosed!!!` ·
`Error: Filleting destroys shape if the filleted edge touches a seam edge` ·
`Error: Erroneous fillet` · `Error: BRepOffset_MakeOffsetShape algorithm does not return valid result` ·
`Error: BRepOffsetAPI_MakeOffset build invalid wire` (×2) · `Error : There are alone Vertices` ·
`Error : bopsection is WRONG` · `Error: Vertex tolerance` / `Error: Edge tolerance` ·
`Error : Nfold tori is bad` · `Standard_ConstructionError` · `Faulty : the distance is` (×2).

### 4.3 What the residual TODOs say

* **The residual is "wrong counts / wrong measure", not "crash".** 50 of the 156 TODO lines are
  "number of X entities is wrong" and another 29 are "area/length/volume of result is wrong" —
  together **51%** of all tolerated failures. After 25 years, the thing OCCT still cannot guarantee
  is the *exact topology and measure* of a boolean result, not that it terminates.
* **Most of it is unconditional.** 122 of 156 TODO lines are `ALL:` — not a flaky-platform
  allowance, a permanent known-wrong.
* **A trivially simple case is still open.** `heal/bug33421` is `pcylinder c1 10 10` + its mirror
  about `z=10`, `bopfuse`, `unifysamedom` — two identical cylinders meeting on a coincident
  circular face. Expected 3 faces / 3 edges / 3 vertices; OCCT gets it wrong and the test is
  TODO-ed on ALL platforms. Coincident-face fuse + same-domain merge is *still* not solved.
* **Periodic faces are still broken at the face-construction level** (`modalg_7/bug27378`, TODO
  ALL: `Error: BRepLib_MakeFace produces invalid faces on periodic surfaces`). The case builds
  cylinder, sphere, cone and two torus faces on a U- or V-range of `pi..3*pi` instead of
  `0..2*pi` and runs `checkshape` + `bopcheck` on each. Seam handling is not merely hard inside
  the BOP — it is unresolved one level below it, in face construction.
* **Fillet/offset on boolean output is the largest TODO cluster** (`bug23902`, `bug24692`,
  `bug25395_2`, `bug25478_2`, `bug_gh501` self-contained, plus most of the offset/fillet TODOs in
  the external set). Whenever a fillet must touch another fillet, or a seam edge, or the boolean
  produced a slightly-off tolerance, OCCT still fails.
* **All six `OCC4717` variants are TODO on ALL platforms** — an ellipsoid of revolution
  fused/cut/commoned with a box, uniformly scaled ×10, with both argument orders; every one is
  pinned to a shape count OCCT does not deliver. Their TODO tag is the placeholder `OCC12345`,
  i.e. nobody ever filed the follow-up issue.
* **Platform non-determinism is normalised.** 10 TODO lines are memory-leak trend failures on
  Linux/macOS only; `modalg_7/bug29910_1` is `?OCC29910 Linux:` — Debian 8 gives a different edge
  count, shell count, area and volume for a fuse of two tangent tori than Windows does. A mature
  kernel accepts that identical inputs produce different topology per platform.

---

## 5. THE EXTERNAL-DATA CASES — what an acquisition would cost

### 5.1 Volume

* 1015 boolean bug cases need data files.
* **1739** `locate_data_file` call sites across them.
* **1163 distinct file basenames** required.
* By extension: **1083 `.brep`**, 19 `.rle`, 16 `.bin`, 8 `.stp`, 8 `.draw`, 1 `.cbf`,
  28 names with no extension (e.g. `bug24157_x_512_solids_glued`, `bug25721_qf1`,
  `bug23884_fz124`, `bug24089_qf1`).
* Loader command used (files containing it, out of the 1015): `restore` 974, `binrestore` 16,
  `brestore` 15, `stepread` 4, `ReadStep` 2, `testreadstep` 2, `ReadIges` 1, `igesread` 0.
  So the corpus is **overwhelmingly native OCCT `.brep`**, not neutral formats.
* Reuse is thin: **918 of the 1163 files are used by exactly one case**. The most-reused are
  `bug25354_f1.brep` (24 call sites), `OCC6272_bridge2.brep` (22), `OCC951_1.brep` /
  `OCC951_2.brep` (16 each), `bug25354_e1.brep`, `OCC20285-tool.brep`, `OCC20285-main.brep`
  (12 each), `bug29807-tool.brep` / `bug29807-obj.brep`, `bug25354_e2.brep`, `OCC697_2.brep`,
  `OCC536.brep`, `OCC485a.brep`, `OCC10604-1.brep` (8 each).

Sample of the required names (first 40, alphabetical): `bug33414_1.brep`, `bug33414_2.brep`,
`BUC60839-1.brep`, `BUC60839-2.brep`, `BUC60861_gap.brep`, `BUC60861_gap1.brep`,
`BUC60865_sh6.brep`, `BUC60875_profile.brep`, `BUC60875_wire.brep`, `BUC60877_lh.brep`,
`BUC60880_sec_error2.brep`, `BUC60881.brep`, `BUC60907_px1.rle`, `BUC60912_sec_slow.brep`,
`BUC60926-2.brep`, `BUC60927.brep`, `BUC60928.brep`, `BUC60929.brep`, `BUC60931.brep`,
`BUC60937.brep`, `BUC60939.brep`, `BUC60946_prism.brep`, `BUC60974.brep`, `BUC60975.brep`,
`Bug26567_c0..c3.brep`, `CTO900_cts16184a.rle`, `CTO900_cts19305-part.rle`,
`CTO900_cts19305-tool.rle`, `CTO902_pro14235a.rle`, `CTO902_pro14235b.rle`,
`CTO904_cts20370-part.rle`, `CTO904_fra50047a.rle`, `CTO909_objects.brep`,
`CTO909_tool_4.brep`, `OCC100_cyl.brep`, …
(full list: `grep -rho 'locate_data_file [^]}]*' tests/bugs/<boolean cases>`).

### 5.2 Where OCCT expects them

* Environment variable **`CSF_TestDataPath`** — a `;`-separated (Windows) or `:`-separated
  (Linux/macOS) list of directories. DRAW resolves `locate_data_file <name>` by searching it.
* It defaults at DRAW start to **`$CASROOT/data`** (documented in
  `dox/contribution/tests/tests.md:64`); extra directories are appended, conventionally via a
  `DrawAppliInit` file in `$CASROOT`.
* The data is **not in the source tree** — `dox/.../tests.md:718` refers to "the test data archive
  … unpacked to `d:/test-data`", and `contribution_workflow.md:96` says new test data files
  "should be attached to the GitHub Issue or PR".

### 5.3 Does the OCCT checkout ship any of it? — **No.**

`/home/petras/code/code_cpp/OCCT/data` exists and contains **77 files**:
`occ/` 45, `images/` 17, `stl/` 10, `iges/` 2, `step/` 2, `vrml/` 1. They are DRAW *demo/sample*
models — `bottle.brep`, `hammer.brep`, `CrankArm.rle`, `Motor-c.brep`, `MODERN_*` furniture,
`propeller.stl`, `screw.step`, `bearing.iges`, plus GIF/PNG textures.

**Intersection of those 77 shipped names with the 1163 names the boolean bug cases require:
zero.** Not one external-data boolean case can run from the repository as checked out.

### 5.4 What an acquisition looks like

OCCT's own CI downloads the dataset as a single archive
(`.github/actions/run-tests/action.yml:42-57`):

```
cd data
wget -q https://github.com/Open-Cascade-SAS/OCCT/releases/download/V7_9_0_beta1/opencascade-dataset-7.9.0.tar.xz
tar -xf opencascade-dataset-7.9.0.tar.xz
# Windows: opencascade-dataset-7.9.0.zip, Expand-Archive
```

and then sets `CSF_TestDataPath: ${{ github.workspace }}/data`. So acquisition is **one release
asset**, unpacked into `$CASROOT/data`, no per-file hunting. Note the archive is tagged 7.9.0
while the checked-out sources are 8.0.1.dev — a version skew to expect if the 1015 external cases
are ever wanted.

### 5.5 Recommendation implied by the numbers

The 237 self-contained cases are free (no download, no parser, no `.brep` reader) and are
statistically indistinguishable from the 1015 paid ones on every keyword axis measured in §3.2.
They are also built from exactly the primitive set we already have (126/237 use only `box`).
The external 1015 buy mostly *imported freeform B-rep* stress — worth having eventually, but
strictly second in line behind running the free 237.
