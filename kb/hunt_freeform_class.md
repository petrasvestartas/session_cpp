# The freeform defect class — root cause, ladder dataset, and the decisive answer

Session: hunt_freeform. Working dir `/home/petras/hunt_freeform/`. Binary used:
`build/main_7_freeform` (a copy of `build/main_7` taken at 2026-07-25 22:19; `src/` was never
touched). Every number below was measured; nothing is inferred. Independent verification is
FreeCAD 1.1.1 / OCCT 7.8 headless.

---

## 1. THE DECISIVE ANSWER

> **It breaks at L1 — freeform per se.** Not the seam, not the poles, not single-face
> periodicity. A closed, valid freeform solid with **zero seams and zero poles** already
> produces a geometrically wrong boolean.

And separately, **seam and pole topology never reach the boolean at all**: they are destroyed
by `src/file_step.cpp` on read. Those are two independent defects and they need two different
fixes.

### 1a. The proof: `L1b_pillow`

A "pillow" — a 4x4x4-ish cube whose 6 planar faces are replaced by bulged bicubic B-spline
patches. The bulge is `0.95*sin(pi u)*sin(pi v)`, which is zero on every boundary, so all 12
edges stay exactly straight and shared. Census: **6 BSplineSurface faces, 12 shared edges,
seam 0, pole 0, free 0, non-manifold 0, closed TRUE, solid.isValid TRUE, volume 76.575864.**
This is the simplest possible genuinely-curved closed NURBS solid.

Our STEP reader handles it **exactly** — OCCT measures our own round-tripped operand at
**76.5759** against the true **76.575864**, closed and valid. So the operand that reaches the
boolean is right.

The boolean is still wrong:

| op | OCCT truth (faces/solids/vol) | ours (faces/solid/naked/vol) |
|---|---|---|
| cut | 12 / 1 / **4.761896** | 12 / 1 / 0 / **3.5412** |
| common | 12 / 1 / **59.238119** | 9 / 1 / 0 / **67.5412** |
| fuse | 12 / 1 / **81.337693** | 9 / 1 / 0 / **72.6602** |

Note the results are **watertight** (`solid 1, naked 0`, and OCCT independently confirms our
written STEP is `closed=True valid=True`). They are simply the wrong solids.

(Section 4b shows OCCT is *not* trustworthy on the single-face periodic blob. It **is**
trustworthy here: the pillow has no seam and no pole, and OCCT's own three answers satisfy the
partition identity to `1.5e-05`. In any case the two proofs below need no reference at all.)

Two oracle-free impossibilities, needing no reference at all:

* `vol(common) = 67.84 > 64.0 = vol(box operand)` — measured by OCCT on our own written
  result (67.8401). An intersection cannot exceed either operand.
* `vol(cut) + vol(common) = 3.8389 + 67.8401 = 71.68`, but `vol(A) = 64`. The partition
  identity is violated by 7.68 (12%).

### 1b. Exactly what goes wrong, face by face

Our `common` result vs the OCCT reference, face by face (areas, centroids):

```
OURS  9 faces, area 85.7642                 OCCT 12 faces, area 81.7621
 Plane  8.5596 at (+2, 0, 0)                 Plane 8.5790 at (+2,0,0)   <- ours has it
 Plane  8.5690 at ( 0,+2, 0)                 Plane 8.5790 at (0,+2,0)   <- ours has it
 Plane  8.5690 at ( 0, 0,+2)                 Plane 8.5790 at (0,0,+2)   <- ours has it
                                             Plane 8.5789 at (-2,0,0)   <- MISSING in ours
                                             Plane 8.5789 at (0,-2,0)   <- MISSING in ours
                                             Plane 8.5789 at (0,0,-2)   <- MISSING in ours
 BSpline 5.0166 at (+1.887, 0, 0)            BSpline 5.0480 at (+1.889,0,0)   correct
 BSpline 5.0384 at (0, +1.887, 0)            BSpline 5.0480 at (0,+1.889,0)   correct
 BSpline 5.0384 at (0, 0, +1.887)            BSpline 5.0480 at (0,0,+1.889)   correct
 BSpline 14.9911 at (-2.179, 0, 0)           BSpline 5.0481 at (-1.889,0,0)   <- UNTRIMMED
 BSpline 14.9911 at (0, -2.179, 0)           BSpline 5.0481 at (0,-1.889,0)   <- UNTRIMMED
 BSpline 14.9911 at (0, 0, -2.179)           BSpline 5.0481 at (0,0,-1.889)   <- UNTRIMMED
```

**14.9911 is exactly the whole patch**: the pillow's shell area is 89.946641, i.e.
89.946641 / 6 = **14.9911068** per face. Those three faces have `wires=1, edges=4` — the
patch's own four original straight boundary edges. They were **never cut**.

So the answer to the question the brief poses — *"is a face classified in when it is out, or
never trimmed at all?"* — is, for the clean freeform case, unambiguously:

> **Never trimmed at all.** For exactly 3 of the 6 plane x freeform-patch pairs the SSI
> produced no intersection curve. With no curve there is nothing to trim with, so the patch
> passes through whole and its opposing box plane is then classified wholly-out and dropped.
> The other 3 pairs work and agree with OCCT to 0.2% (5.0166/5.0384/5.0384 vs
> 5.0480/5.0480/5.0481).

The failing three are the −x, −y, −z pairs and the working three are +x, +y, +z. The pillow is
geometrically symmetric under sign flip, so this is an algorithmic asymmetry (seeding /
marching direction), not a property of the data. The same three pairs fail in all three ops:
tessellated tight bboxes are `cut` and `common` = `(-2.75,-2.75,-2.75)-(2.00,2.00,2.00)` while
OCCT's are exactly `+-2.00`; our `fuse` = `(-2.00,-2.00,-2.00)-(2.75,2.75,2.75)` while OCCT's
is `+-2.75` — the negative-side bulges are missing from the fuse and wrongly present in the
cut/common. 1506 of 11907 mesh points of our `common` lie outside the box operand, worst
excursion **+0.75**.

### 1c. The control that proves the harness and the boolean are otherwise sound

Same FreeCAD/OCCT STEP pipeline, same box, same binary, `A0_box_cyl_analytic`
(box 4^3 x analytic cylinder r=1.6 h=8, which has a **seam** and 3 faces):

| op | OCCT | ours | |
|---|---|---|---|
| cut | 7 / 1 / 31.830091 | 7 / 1 / naked 0 / 31.8301 | **exact** |
| common | 3 / 1 / 32.169909 | 3 / 1 / naked 0 / 32.1699 | **exact** |
| fuse | 10 / 1 / 96.169909 | 10 / 1 / naked 0 / 96.1699 | **exact** |

Analytic surfaces, seam included: exact to the printed digits. Freeform: wrong. That is the
whole story in two rows.

---

## 2. THE SECOND DEFECT — the STEP reader destroys seam and pole topology

This is what stops the ladder from being run cleanly above L1, and it is why the
`SESSION_CHAIRS` route cannot currently be used to test seams/poles at all.

Reader fidelity, measured by feeding each operand in and reading our kernel's own
`B: ... vol` print (in-memory, before any boolean). Truth is OCCT on the same file.

| operand | topology | true vol | our in-memory vol | verdict |
|---|---|---|---|---|
| `cyl_analytic` (CYLINDRICAL_SURFACE) | seam 1, pole 0 | 64.3398 | 64.3398 | **exact** |
| `tor_analytic` (TOROIDAL_SURFACE) | seam 2, pole 0 | 59.2176 | 59.2176 | **exact** |
| `halfcyl_nurbs` (B_SPLINE) | seam 0, pole 0 | 37.3586 | 37.2675 | ok (0.24%) |
| `pillow` (B_SPLINE) | seam 0, pole 0 | 76.5759 | 76.2019 | shape exact*, volume() 0.49% low |
| `cyl_nurbs` (B_SPLINE) | **seam 1**, pole 0 | 64.9289 | **42.8985** | **BROKEN** |
| `tor_nurbs` (B_SPLINE) | **seam 2**, pole 0 | 59.3611 | **177.6529** | **BROKEN (3x)** |
| L2 tube (B_SPLINE) | **seam 1**, pole 0 | 109.1609 | **2.0404** | **BROKEN** |
| `lunes_analytic` (SPHERICAL x2) | seam 0, **pole 4** | 65.4498 | **130.8815** | **BROKEN (2x)** |
| `lunes_nurbs` (B_SPLINE x2) | seam 0, **pole 4** | 65.4305 | **130.8270** | **BROKEN (2x)** |
| `iso_f8` (B_SPLINE x8) | seam 0, **pole 8** | 65.2526 | **47.0531** | **BROKEN** |
| `sph_analytic` (SPHERICAL, 1 face) | **seam 1, pole 2** | 65.4498 | **0.0000, is_solid 0** | **BROKEN** |
| `sph_nurbs` (B_SPLINE, 1 face) | **seam 1, pole 2** | 65.4212 | **0.0000, is_solid 0** | **BROKEN** |
| L4 blob (B_SPLINE, 1 face) | **seam 1, pole 2** | 65.1885 | 65.2561 but **is_solid 0** | **BROKEN** |

\* for the pillow, OCCT measured our *written* round-trip at 76.5759 = the truth, so the read
geometry is exact and only `BRep::volume()` is 0.49% low. Distinguish the two: reader fidelity
must be judged on the round-tripped STEP, not on our own `volume()`.

Read the pattern:

* **Analytic surfaces survive any topology** — a cylinder's seam and a torus' two seams are
  read exactly. So the seam concept itself is handled somewhere.
* **NURBS + seam is broken.** OCCT writes periodic surfaces to STEP as a *clamped* B-spline
  with duplicated wrap poles and the `u_closed = .T.` flag (verified: the L2 lateral face is
  `B_SPLINE_SURFACE_WITH_KNOTS('',3,2,...,.UNSPECIFIED.,.T.,.F.,.F.,(4,1,...,1,4),(3,3),...)`
  — 105 x 3 poles, ordinary clamped knots). Nothing exotic is in the file; we mishandle it.
* **Poles (degenerate edges) are broken independently of the seam.** Two lune faces with no
  in-face seam and 4 degenerate pole edges read as *exactly twice* the true volume
  (130.8815 vs 65.4498) — which is what you get if each lune face is taken over the full
  periodic domain instead of its trimmed half.
* **Single face + seam + poles fails `is_solid()`** outright (sphere -> `faces 1 solid 0`).
  A torus, which is a single periodic face with **two** seams but **no poles**, reads as
  `solid 1`. So the thing that breaks `is_solid()` is the **degenerate pole edge**, not
  periodicity: a pole edge has one trim, and a "every edge has two trims" solidity test
  rejects it.

Consequence for `SESSION_CHAIRS` cells: whenever B fails `is_solid()`, the boolean
short-circuits — `cut` returns A unchanged, `common` returns 0 faces, `fuse` returns A and B
as two disjoint bodies. That is exactly what every L4 cell shows
(`cut 6/1/0/64.0000`, `common 0/0/0/0.0000`, `fuse 7/1/0/129.2561 = 64.0 + 65.2561`).
Those rows are **not** boolean failures; they are the guard firing on a bad operand.

---

## 3. THE THIRD DEFECT — `FACE_BOUND` is read as a hole

`src/file_step.cpp:1191-1193` and `:1473-1476`:

```cpp
bool is_outer = bent->second.has("FACE_OUTER_BOUND");
if (!is_outer && !bent->second.has("FACE_BOUND")) continue;
```

so a `FACE_BOUND` entity yields `is_outer = false` and the loop becomes an inner/hole loop.
**FreeCAD/OCCT writes every outer bound as plain `FACE_BOUND`** (legal AP214 — `FACE_OUTER_BOUND`
is an optional subtype and the receiver is expected to work out which bound is outer).
Rhino writes `FACE_OUTER_BOUND`, which is the only reason the chairs corpus escapes this.

Measured, on `box 4^3 x cylinder(1.6, 8)` from FreeCAD:

| | cut | common | fuse |
|---|---|---|---|
| as written (`FACE_BOUND`) | 3 / solid 0 / naked 5 / 21.5032 | 7 / 0 / 5 / 21.1634 | 2 / 0 / 5 / 21.3900 |
| after `FACE_BOUND -> FACE_OUTER_BOUND` | 7 / **1** / **0** / **31.8301** | 3 / **1** / **0** / **32.1699** | 10 / **1** / **0** / **96.1699** |
| OCCT truth | 7 / 1 / 31.830091 | 3 / 1 / 32.169909 | 10 / 1 / 96.169909 |

A one-token textual rewrite turns garbage into an exact match. It also fixes the operand read
(that cylinder read as `vol 21.4523` before and `64.3398` after — the truth is 64.339818).

**This is the cheapest high-value fix in this report** and it unblocks every OCCT-authored
STEP file in the world. The whole dataset below is shipped in both forms so it can be used as
a regression test for exactly this.

---

## 4. ROOT CAUSE OF THE SHIPPED L4 CASE (`freeform_common_box.step`)

The shipped cell does **not** go through the STEP reader — `main_7.cpp:516-620` builds the
blob in code from `BRep::create_sphere(2.5)` with interior control points scaled by
`1 + 0.12*sin(2.1 i + 1.3 j)`, and the box from `BRep::create_box(4,4,4)`. Reproduced exactly
with the current binary (`SESSION_FREEFORM=1`): `cut 15.0872 f8 s0`, `common 46.9840 f6 s0`,
`fuse 79.2160 f14 s0`, identical to the shipped file.

Operands (OCCT, on the bodies inside the shipped file): box 6 faces closed valid vol 64.000000;
blob **1 face, 3 edges — one seam of length 7.85398 (the meridian at y = 0, x > 0, running pole
to pole) and two zero-length degenerate pole edges at (0,0,-2.5) and (0,0,2.5)** — closed,
valid, vol 64.589785.

OCCT's answer: `cut 7/1/5.487351`, `common 5/1/58.525194`, `fuse 11/1/70.003399` — **but see
4b, that answer is itself wrong.** Independent sampling gives `common = 53.7600 +- 0.3519`.
Ours: `cut 8 faces open, 2 naked`, `common 6 faces open, 4 naked, vol 47.1447`,
`fuse 13 faces closed vol 72.7665 plus a stray 1-face open shell of vol 1.3518`.

### 4a. The naked edges are ON THE SEAM

Every free-edge endpoint of every op, listed with its `|y|`:

```
cut     (-1.3687,-0.0015,-1.9992) |y|=0.00150   (1.5000, 0.0000,-2.0000) |y|=0.00000
cut     ( 1.5000, 0.0000,-2.0000) |y|=0.00000   (-1.3687,-0.0015,-1.9992) |y|=0.00150
common  ( 2.0000,-0.0000, 1.5000) |y|=0.00000   ( 2.0000,-0.0111,-1.4985) |y|=0.01110
common  ( 1.5000, 0.0000,-2.0000) |y|=0.00000   (-1.3687,-0.0015,-1.9992) |y|=0.00150
common  (-1.3687,-0.0015,-1.9992) |y|=0.00150   ( 1.5000, 0.0000,-2.0000) |y|=0.00000
common  ( 2.0000, 0.0000,-1.5000) |y|=0.00000   ( 2.0000, 0.0000, 1.5000) |y|=0.00000
fuse    ( 1.5000, 0.0000,-2.0000) |y|=0.00000   (-1.3687,-0.0015,-1.9992) |y|=0.00150
fuse    (-1.3687,-0.0015,-1.9992) |y|=0.00150   ( 1.5000, 0.0000,-2.0000) |y|=0.00000
```

**Every single endpoint lies on the plane y = 0 — the blob's seam plane** (max |y| = 0.0111).
None of them is near a pole: the nearest, (1.5, 0, -2.0), is 1.58 away from (0,0,-2.5).

So for the shipped case the wires fail to close **exactly where an intersection curve crosses
the seam**: the curve is split at the seam and the two halves are never re-joined across the
periodic wrap, leaving the loop open. The result: the blob contributes **one face carrying
four separate wires and 15 edges**, and the shell cannot close.

### 4b. The OCCT reference for this cell is ITSELF WRONG — and we can prove it

Before blaming our box faces I checked the reference by direct sampling, with no boolean
involved: for each of the 6 box faces, grid-sample it (60x60 over the 4x4 face) and count the
samples that lie inside the blob solid. A box face with a non-empty inside-blob region **must**
appear in `common(box, blob)`.

| box face | sampled area inside the blob | OCCT reference face | OUR face |
|---|---|---|---|
| x = +2 | **5.8178** | 5.8199 (match) | **2.9260** = 0.503 x |
| x = -2 | **6.0178** | 6.0107 (match) | 6.0085 (match) |
| y = +2 | **7.3200** | **ABSENT** | 7.3371 (match) |
| y = -2 | **7.3111** | **ABSENT** | 7.3064 (match) |
| z = +2 | **6.9778** | 6.9606 (match) | 6.9371 (match) |
| z = -2 | **6.9778** | 6.9458 (match) | **ABSENT** |

**OCCT/FreeCAD 1.1.1 omits both `y = +-2` cuts on this cell** — 14.63 of face area — which is
why its reference solid tessellates out to `y = +-2.65`, beyond the box operand (11200 of 21929
mesh points outside). It is not a loose-bbox artefact for the reference; it is a real OCCT
failure on this single-face periodic freeform. Monte-Carlo confirms it: 40000 uniform points in
the box, `blob.isInside`, gives

```
true common(box, blob) = 53.7600  +-0.3519 (3 sigma)
OCCT reference         = 58.5252   (+4.77, 13 sigma out -- WRONG, too big by the un-cut y caps)
OURS                   = 47.1447   (-6.62 -- WRONG, too small)
```

and the two errors have the expected structure: OCCT's `cut` (5.487351) is short by the same
4.77 that its `common` is long, which is what you get when the same two cuts are missed in both.

**So the reference numbers quoted in the brief (`common = 5 faces / 1 solid / 58.5252`) are not
truth.** Our two `y = +-2` faces, which look like fabrications against that reference, are
actually **correct** (7.3371 / 7.3064 against the sampled 7.3200 / 7.3111, agreeing to 0.5%).

### 4c. What IS wrong with our L4 result — the BOX faces at the seam crossings, not the blob

The blob's own contribution is **correct**. Jacobian-weighted sampling of the blob surface
(`ref_sanity.py`, no boolean involved) integrates `|Su x Sv| du dv` over `{S(u,v) inside the box}`:

```
blob surface total area  (sampled) = 77.9763   (OCCT Face.Area 78.0689 -- sampling good to 0.12%)
blob surface INSIDE the box        = 32.5443   <- the correct blob contribution
OUR retained blob face area        = 32.7684   <- 100.7% of correct   CORRECT
OCCT reference blob face area      = 49.9662   <- 154% of correct     WRONG (the un-cut y caps)
```

So the earlier reading that our blob face was "under-trimmed by a third" was an artefact of
grading against OCCT. **Our blob face is right to 0.7%.** What is wrong is confined to the box
side, and precisely to the two faces the seam interacts with:

* **`x = +2`: we keep 2.9260 where the truth is 5.8178 — a ratio of 0.5029, exactly one half.**
  `x = +2` is the one box face the seam crosses: the seam meridian runs along `y = 0, x > 0`
  and pierces the plane `x = 2` at `(2, 0, z)`. The seam divides that face's inside-blob region
  in two and **we retain one half and discard the other.**
* **`z = -2`: dropped entirely (6.9778 missing).** That is the face carrying the two naked edges
  with endpoints `(1.5, 0, -2.0)` and `(-1.3687, -0.0015, -1.9992)` — both on `y = 0`. Its loop
  is broken at the seam, so the face is lost.

The volume deficit follows: 47.1447 against the sampled truth 53.7600, i.e. **-6.62**, which is
the un-recovered material behind those two faces. And the blob's contribution arrives as **one
face carrying four separate wires and 15 edges** — the loop fragments produced by the seam
splits, never merged.

So for L4 the failure mode is *different* from the pillow. The curves **are** produced, the
freeform face **is** trimmed correctly, and the defect is that **section curves crossing the
seam are split into fragments that are never re-joined across the periodic wrap**, so the
adjacent planar faces' loops cannot close: one keeps half its region, the other is dropped.

Answering the brief's question for this cell: it is **neither** "classified in when it is out"
**nor** "never trimmed at all" — it is a *loop-closure* failure at the seam that costs the
adjacent faces half their region or all of it.

---

## 5. CORRECTIONS TO THE BRIEF — please propagate these

**(a) The "oracle-free proof of wrongness" in the brief is an artefact. Retract it.**
The brief states our common's bbox is `(-2.24,-2.64,-2.18)-(2.49,2.64,2.18)` against a box
operand of `+-2`, i.e. the intersection escapes an operand. Those numbers are FreeCAD's
`Shape.BoundBox`, which for a B-spline face bounds the **control net**, not the trimmed face.
Measured properly by tessellation, our shipped `common` result's tight bbox is

```
(-2.0000, -2.0040, -2.0020) - (2.0001, 2.0036, 2.0025)     worst excursion +0.0040
```

i.e. it does **not** escape the box beyond tolerance, and a face-vs-solid intersection puts
only **0.0072** of its 32.7684 area outside. The same loose-bbox artefact makes OCCT's *own*
correct reference report `y` out to `+-2.65`. Do not chase this.

**(b) The real oracle-free proofs, which do hold**, are the pillow's
`vol(common) = 67.84 > 64 = vol(A)` and its partition residual of 7.68 on 64, plus the
tessellated `+-0.75` excursion of the pillow result.

**(b2) The OCCT reference numbers quoted in the brief for the shipped L4 cell are wrong.**
`common = 5 faces / 58.5252` omits both `y = +-2` cuts (see 4b). Independent sampling gives
`53.7600 +- 0.3519`. Do not grade the L4 cell against 58.5252, and do not treat our `y = +-2`
faces as spurious — they are right. Grade against the per-box-face sampled areas in 4b, which
need no boolean at all. The same caution applies to every `L4*` and `I_iso_*` reference in the
shipped dataset: they are OCCT's answer, not verified truth.

**(c) `naked` in the driver print counts topology edges with one trim**, which counts a seam as
naked. Everything reported here as "free"/"naked" uses **wire-occurrence counting** instead:
an edge is properly used when it appears exactly twice across all face wires
(`Face.Wires[i].OrderedEdges`, where a seam legitimately appears twice inside one face); a
degenerate pole edge appears once and is exempt. Note `validate_operands.py` in
`/home/petras/fc_inspect/` uses `Face.Edges`, which de-duplicates, so it reports every seam and
pole as "TRULY FREE" — that is why it calls the shipped blob 3-free while also calling it
closed and valid. My `dataset/` gate uses the corrected metric.

**(d) L4 cells run through `SESSION_CHAIRS` are not testing the boolean.** They test the STEP
reader, which rejects the operand, after which the boolean guard returns A / empty / disjoint.
Only the in-code cell exercises the L4 boolean today.

---

## 6. DELIVERABLE 1 — the dataset

`/home/petras/hunt_freeform/dataset/<cell>/` — `chair0.stp` (A), `chair1.stp` (B),
`REF_cut.step`, `REF_common.step`, `REF_fuse.step`, plus `MANIFEST.json` and `operands.json`.
`/home/petras/hunt_freeform/dataset_ob/<cell>/` — the identical pairs with
`FACE_BOUND` rewritten to `FACE_OUTER_BOUND` so they are runnable **today** (defect 3 above).
Every face in every operand has exactly one bound, so the rewrite is loss-free.

**Every operand passes the gate before shipping**: `Shell.isClosed() == True`,
`Part.Solid(shell).isValid() == True`, 0 truly-free edges (wire-occurrence metric),
0 non-manifold edges, volume > 0. Built by `build_dataset.py` / `build_iso.py` /
`ref_probe4.py`; gate + references regenerated by `finalize_dataset.py`.

| cell | B faces | seam | pole | B vol | purpose |
|---|---|---|---|---|---|
| `A0_box_cyl_analytic` | 3 | 1 | 0 | 64.339818 | analytic control — must stay exact |
| `L0_box_box` | 6 | 0 | 0 | 64.000000 | planar control (axis-aligned, parallel faces) |
| `L1b_pillow` | 6 | 0 | 0 | 76.575864 | **freeform, no seam, no pole — the key cell** |
| `L1_noseam_nopole` | 6 | 0 | 0 | 107.429554 | freeform lofted tube, 4 lateral patches + 2 caps |
| `L1_seam_thru_face` | 6 | 0 | 0 | 107.429554 | same, box face plane at y = 0 |
| `L2_seam_nopole` | 3 | **1** | 0 | 109.160926 | periodic lateral face: **seam, no pole** |
| `L2_seam_thru_face` | 3 | **1** | 0 | 109.160926 | box face plane y = 0 **contains the seam** |
| `L2_caps_only` | 3 | **1** | 0 | 109.160926 | wide box: only the caps cut the tube |
| `L3_poles_noseam` | 2 | 0 | **4** | 65.385530 | two lune faces: **poles, no in-face seam** |
| `L3_pole_inside` | 2 | 0 | **4** | 65.385530 | same, box 4x4x6 so both poles are inside |
| `L4_seam_poles` | 1 | **1** | **2** | 65.188461 | **the shipped case's topology** |
| `L4_seam_thru_face` | 1 | **1** | **2** | 65.188461 | box face plane y = 0 contains the seam meridian |
| `L4_pole_inside` | 1 | **1** | **2** | 65.188461 | box 4x4x6, both poles strictly inside |
| `L4_pole_on_face` | 1 | **1** | **2** | 65.188461 | box 4x4x5, poles exactly on the z faces |
| `L4x_exact_sphere` | 1 | **1** | **2** | 65.368889 | unperturbed NURBS sphere (recogniser bait) |
| `L5_ff_ff` | 1 | **1** | **2** | 44.424111 | freeform x freeform |
| `I_iso_f1` | 1 | 1 | 2 | 65.188461 | ISO ladder: one blob, 4 decompositions |
| `I_iso_f2` | 2 | 0 | 4 | 65.243061 | same blob split at 2 meridians |
| `I_iso_f2z` | 2 | 2 | 2 | 65.343815 | same blob split at the equator |
| `I_iso_f8` | 8 | 0 | 8 | 65.252576 | same blob split into 8 octants |

The `I_iso_*` group was intended as the perfect controlled experiment — one geometry, four face
decompositions. It only partly worked: splitting a periodic sphere-like surface leaves a
degenerate pole edge on **every** sub-face, so `iso_f8` still has 8 pole edges and no variant is
pole-free. `L1b_pillow` is the genuinely pole-free, seam-free control, and it is the cell that
carries the conclusion. Also note OCCT's own answers for `iso_f1` (cut 8.1098) differ from
`iso_f2/f2z/f8` (cut ~10.00) by 23%, because OCCT re-approximates the surface when splitting —
so treat `iso_f*` cross-variant comparisons with care and prefer `L1b_pillow`.

### Running a cell

```bash
cd /home/petras/code/code_rust/session/session_cpp
SESSION_NO_ROT=1 SESSION_CHAIRS=/home/petras/hunt_freeform/dataset_ob/L1b_pillow \
  timeout 900 ./build/main_7_freeform SKIPMATRIX
```

Verify independently (never trust our own numbers):

```bash
cd /home/petras/hunt_freeform
FC_FACES=1 FC_FILE=<dir>/chair0_common_chair1.step /snap/bin/freecad.cmd inspect_one.py
```

Helper scripts, all in `/home/petras/hunt_freeform/`: `inspect_one.py` (wire-occurrence census
of every body in a STEP file), `tight_bbox2.py` (tessellated bbox + containment audit),
`face_detail.py` (face-by-face ours-vs-OCCT), `run_ladder.sh`, `make_table.py`.

---

## 7. DELIVERABLE 2 — the full ladder result

`OURS` is `faces/solid/naked/vol`; `naked` is the driver's own count. `PASS` requires exact
face count, `solid 1`, `naked 0`, and volume within 1e-3 relative.

| cell | B read | op | OCCT f/s/vol | OURS | |
|---|---|---|---|---|---|
| `A0_box_cyl_analytic` | **exact** | cut | 7/1/31.830091 | 7/1/0/31.8301 | **PASS** |
| | | common | 3/1/32.169909 | 3/1/0/32.1699 | **PASS** |
| | | fuse | 10/1/96.169909 | 10/1/0/96.1699 | **PASS** |
| `L0_box_box` | exact | cut | 9/1/36.379 | 8/0/6/31.2923 | FAIL |
| | | common | 6/1/27.621 | 7/0/6/32.7077 | FAIL |
| | | fuse | 12/1/100.379 | 12/1/0/100.3790 | **PASS** |
| `L1b_pillow` | exact* | cut | 12/1/4.761896 | 12/1/0/3.5412 | FAIL (vol) |
| | | common | 12/1/59.238119 | 9/1/0/67.5412 | FAIL |
| | | fuse | 12/1/81.337693 | 9/1/0/72.6602 | FAIL |
| `L1_noseam_nopole` | exact | cut | 18/4/1.063837 | 12/0/10/34.3869 | FAIL |
| | | common | 10/1/62.936172 | 4/0/10/29.6130 | FAIL |
| | | fuse | 20/1/108.49275 | 17/0/10/141.8166 | FAIL |
| `L1_seam_thru_face` | exact | cut | 7/1/25.318175 | 6/0/7/52.9257 | FAIL |
| | | common | 7/1/38.682143 | 2/0/7/11.0743 | FAIL |
| | | fuse | 11/1/132.750407 | 12/0/7/160.3541 | FAIL |
| `L2_seam_nopole` | **BROKEN** 2.0404 | all | — | — | operand destroyed on read |
| `L2_seam_thru_face` | **BROKEN** 2.0404 | all | — | — | operand destroyed on read |
| `L2_caps_only` | **BROKEN** 2.0404 | all | — | — | operand destroyed on read |
| `L3_poles_noseam` | 65.3687 (2x bug absent here) | cut | 8/1/10.228673 | 6/0/5/50.4484 | FAIL |
| | | common | 8/1/53.828724 | 3/0/5/13.5516 | FAIL |
| | | fuse | 16/1/75.473279 | 9/0/12/92.1740 | FAIL |
| `L3_pole_inside` | 65.3687 | cut | 8/1/38.711941 | 6/0/3/87.0852 | FAIL |
| | | common | 6/1/57.462483 | 2/0/4/8.9148 | FAIL |
| | | fuse | 12/1/103.954998 | 9/0/6/128.8110 | FAIL |
| `L4_seam_poles` | **is_solid 0** | cut | 7/1/8.109725 | 6/1/0/64.0000 = A | guard fired |
| | | common | 6/1/55.914906 | 0/0/0/0.0000 | guard fired |
| | | fuse | 12/1/73.430722 | 7/1/0/129.2561 = A+B | guard fired |
| `L4_seam_thru_face` | **is_solid 0** | — | — | identical to above | guard fired |
| `L4_pole_inside` | **is_solid 0** | — | — | 96.0 / 0 / 161.2561 | guard fired |
| `L4_pole_on_face` | **is_solid 0** | — | — | 80.0 / 0 / 145.2561 | guard fired |
| `L4x_exact_sphere` | **BROKEN** 0.0 | cut | 7/1/9.480074 | 6/0/6/35.7257 | FAIL |
| `L5_ff_ff` | **BROKEN** both | cut | 2/1/48.60889 | 1/0/0/65.2561 | FAIL |
| `I_iso_f1` | **is_solid 0** | — | — | guard fired | |
| `I_iso_f2` | **BROKEN** 0.0 | cut | 8/1/10.006631 | 4/0/**169**/1.5425 | FAIL |
| | | common | 8/1/53.991969 | `std::bad_alloc` | THREW |
| `I_iso_f2z` | **BROKEN** 0.4914 | cut | 8/1/9.953708 | 2/0/6/0.1385 | FAIL |
| `I_iso_f8` | **BROKEN** 47.05 | cut | 14/1/10.009418 | 13/0/3/63.9022 | FAIL |
| | | common | 14/1/53.990093 | 12/0/2/0.0146 | FAIL |

\* pillow read is exact in geometry (OCCT measures our round-trip at 76.5759 = truth);
only `BRep::volume()` reports 76.2019.

Also observed: `tor_analytic` and `tor_nurbs` (box x torus) **throw `std::bad_alloc`** in
`cut`, and `I_iso_f2 common` throws `std::bad_alloc`. Those are separate blow-ups worth a
guard.

`L0_box_box` fails cut/common while its `fuse` is exact. It is the axis-aligned box pair, all
face pairs parallel or perpendicular — that smells like the coincidence class rather than this
one; `A0_box_cyl_analytic` (also axis-aligned) is exact, and rotating the box pair changes the
failure signature, so I did not pursue it here. It is *not* the reason the freeform cells fail:
`A0` passes with the same harness, same reader, same day.

---

## 8. WHAT TO FIX, IN ORDER

1. **`file_step.cpp`: treat `FACE_BOUND` as the outer bound when a face has exactly one bound**
   (or, better, decide outer-vs-inner geometrically from the 2D loop areas, which is what OCCT
   and every other reader does). One-line-scale change, unblocks every OCCT/FreeCAD STEP file.
   Regression cells: `dataset/A0_box_cyl_analytic` (must go exact), all of `dataset/`.
2. **SSI plane x bicubic NURBS patch is dropping 3 of 6 intersections in
   `dataset_ob/L1b_pillow`.** Sign-correlated (the −x/−y/−z pairs), so look at seeding /
   marching-direction bias, not at tolerance. This is the freeform defect class proper and it
   is the one the user asked about. The untrimmed-face signature (patch area 14.9911 =
   89.946641/6, `wires=1 edges=4`) is an easy assertion to add: no face of a boolean result may
   equal an input face when the input faces provably intersect.
3. **Degenerate (pole) edges: `is_solid()` must exempt them** (a pole edge has one trim by
   construction), and the volume integration must not double-count a face that carries them —
   the lunes read as exactly 2x (130.8815 vs 65.4498), i.e. each face is being integrated over
   the full periodic domain rather than its trimmed half.
4. **Periodic NURBS surfaces from STEP** (clamped poles + `u_closed = .T.`) are being
   reconstructed wrongly — `cyl_nurbs` 42.90 vs 64.93, `tor_nurbs` 177.65 vs 59.36, L2 tube
   2.04 vs 109.16 — while the *analytic* equivalents are exact. Compare the two paths.
5. **Seam-crossing loop closure in the splitter**, which is the shipped L4 failure: all eight
   naked endpoints sit on y = 0, the seam plane; the one box face the seam crosses keeps
   **exactly half** its region (2.9260 of 5.8178) and the face whose loop breaks at the seam is
   dropped whole (z = -2, 6.9778). Prior art: memory topic `reference_seam_split_cuts` and the
   seam decomposition already in `src/brep_section.cpp`. Note this is item 5, not item 1 —
   fixing it will not fix `L1b_pillow`.
6. `std::bad_alloc` guards for box x torus and `I_iso_f2 common`.

### A regression suite that needs no oracle

Three checks catch every failure in this report without any reference solid, and two of them
catch failures that OCCT itself does not:

* `vol(A cut B) + vol(A common B) == vol(A)` — caught `L1b_pillow` (residual 7.68 on 64).
* `vol(A common B) <= min(vol(A), vol(B))` — caught `L1b_pillow` (67.84 > 64).
* no face of a boolean result may be identical to an input face when the two inputs provably
  intersect — caught `L1b_pillow`'s three untrimmed 14.9911 patches directly.

Per-box-face sampled areas (4b) are a cheap oracle-free reference for any box x anything cell
and are strictly more reliable than OCCT on the freeform cells.

---

## 9. WHAT I DID NOT MEASURE

* I never ran a freeform x freeform boolean with two valid operands — `L5_ff_ff` and every
  `iso` variant have B rejected by the reader, so L5 is untested.
* `L2` (seam, no pole) was never tested at the boolean level for the same reason. The ladder
  therefore **cannot** rank seam-vs-pole severity inside the boolean; it can only say that
  the boolean is already wrong below both of them (L1) and that both break the reader.
* The `iso_f1`-vs-`iso_f8` 23% discrepancy in OCCT's own answers (cut 8.1098 vs 10.0094 for
  what should be the same solid) was **not** resolved — `mc_verify.py` timed out at 900 s
  because `isInside` on a periodic B-spline solid is very slow. Given 4b, my working assumption
  is that OCCT is losing cuts on the single-face variant the same way it does on the shipped
  cell, but I did not confirm it. Treat every `L4*` and `I_iso_*` reference volume as OCCT's
  answer, not as truth. `A0_box_cyl_analytic`, `L0_box_box` and `L1b_pillow` references are
  sound (exact analytic values; `L1b_pillow` partition residual 1.5e-05).
* `L0_box_box` cut/common was not root-caused.
* Everything in section 4 rests on sampling oracles (per-box-face grid sampling, Jacobian-
  weighted surface integration, 40k-point Monte-Carlo), not on OCCT, because OCCT is wrong on
  this cell. The sampling accuracy is 0.12% on a known quantity (blob total area 77.9763 vs
  78.0689) and +-0.35 at 3 sigma on the MC volume.
