# Hunt: the "missing EF/VF interference paving stage"

**Panel claim under test:** *"there is no EF/VF interference paving stage at all (OCCT spends
BOPAlgo_PaveFiller_4/5.cxx on it; x20 and x13y29 are attributed to it)."*

**Verdict: the claim is WRONG as stated, and the attribution of x20 / x13y29 to it is wrong.**
We have no *named* EF stage and no edge x face solver in the production call path, but the
section scaffold produces the EF piercing points by another route, and it produces essentially
all of them, to essentially full accuracy. Measured: **33 of 34** true EF points for x20 and
**42 of 42** for x13y29 are present as vertices *in our shipped result*, worst-case error
3.2e-4 and 3.8e-3 respectively. VF is a non-issue: **zero** VF interferences exist in either
config. The two configs' defects sit at those very junctions but are *branch/continuation and
face-emission* failures, not paving failures.

Everything below is measured, not inferred. Reproduction commands at the end.
Metric caveats are stated inline; naked counts are flagged with which metric produced them.

---

## 1. What OCCT PaveFiller_4 / _5 actually compute and add

Line counts (measured, `wc -l`, `/home/petras/code/code_cpp/OCCT/src/ModelingAlgorithms/TKBO/`):

| file | lines |
|---|---:|
| `BOPAlgo/BOPAlgo_PaveFiller_4.cxx` (VF) | 390 |
| `BOPAlgo/BOPAlgo_PaveFiller_5.cxx` (EF) | 1199 |
| `IntTools/IntTools_EdgeFace.cxx` (the actual E x F solver) | 875 |
| `IntTools/IntTools_BeanFaceIntersector.cxx` (its engine) | 2639 |
| `IntTools/IntTools_Context.cxx` (`ComputeVF`, `ProjPS`, `IsPointInFace` caches) | 1039 |
| `IntTools/IntTools_Tools.cxx` | 804 |
| `BOPTools/BOPTools_AlgoTools.cxx` (`MakeNewVertex`, `ComputeVV`, `CorrectRange`) | 2416 |
| `BOPTools/BOPTools_AlgoTools2D.cxx` | 700 |
| `BOPDS/BOPDS_PaveBlock.cxx` | 358 |

### PaveFiller_4 — VF (vertex x face)

`PerformVF` at `BOPAlgo_PaveFiller_4.cxx:139`. Iterates VERTEX x FACE pairs, skipping
sub-shape pairs (`:189`), already-recorded interferences (`:194`), and same-SD-vertex repeats
(`:206-220`). The work is `IntTools_Context::ComputeVF(V, F, T1, T2, tolVNew, fuzzy)`
(`:113`, run in parallel at `:242`). On success it **adds**:

- a `BOPDS_InterfVF` record carrying the (u,v) parameters of the vertex on the face (`:279-281`);
- `myDS->AddInterf(nV, nF)` (`:283`);
- `UpdateVertex(nV, aTolVNew)` (`:286`) — **grows the vertex tolerance, minting a NEW vertex
  shape** if needed, recorded as `SetIndexNew` (`:289-292`);
- the vertex into that face's `BOPDS_FaceInfo::ChangeVerticesIn()` (`:295-297`) — the *in* set
  the face splitter later consumes.

`TreatVerticesEE` (`:305`) then takes vertices **newly created by EE interferences**
(`:321-332`) and re-tests them against every face (`:363-389`), adding the hits to `VerticesIn`
as well. VF adds **no paves on edges** — only vertex-in-face incidence and tolerance growth.

### PaveFiller_5 — EF (edge x face)

`PerformEF` at `BOPAlgo_PaveFiller_5.cxx:165`.

1. `FillShrunkData(EDGE, FACE)` (`:167`) shrinks each pave block away from its end vertices'
   tolerance spheres, so an intersection near an end is not confused with the end.
2. Works **per pave block**, not per edge (`:246-306`) — an edge already split by VV/EE is
   handled piecewise. Bbox reject at `:268`; `BOPTools_AlgoTools::CorrectRange` at `:291,:296`.
3. `IntTools_EdgeFace::Perform` in parallel (`:317`); the worker (`:102-141`) also translates
   edge+face to the origin for conditioning.
4. Each result is an `IntTools_CommonPrt` of type VERTEX or EDGE:
   - **VERTEX** (`:406`): parameter `t` on the edge -> `BOPTools_AlgoTools::MakeNewVertex(aE, aT, aF, aVnew)`
     (`:413`) = **a new vertex**. If it lands in the pave block's end ranges the new vertex is
     *not* created; instead the existing end vertex is forced into a VF interference
     (`ForceInterfVF`, `:454` / `:631`) or **its tolerance is grown to swallow the point**
     (`:482-501`). Otherwise: reject if coincident with a face pave (`:505`), grow tolerance to
     `max(tolE, tolF)` (`:510-512`), reject if outside the face (`IsPointInFace`, `:523`), then
     record `BOPDS_InterfEF` (`:530-533`), `AddInterf` (`:535`), and queue the vertex **together
     with the pave block it must split** (`aMVCPB`, `:537-542`).
   - **EDGE** (`:545`): the edge lies *on* the face -> an **edge/face common block**
     (`BOPAlgo_Tools::FillMap`, `:564`), i.e. the edge becomes shared with the face instead of
     being split.
5. Post-treatment (`:576-585`): `PerformCommonBlocks`, `UpdateVerticesOfCB`,
   **`PerformNewVertices`** (this is what actually *inserts the new paves* and re-splits the
   pave blocks), `UpdateFaceInfoIn`.
6. `ReduceIntersectionRange` (`:685`) shrinks the search range using EE results.
   `ForceInterfEF` (`:772` / `:831`) runs a **second pass** after vertices have been grown and
   unified, over a BVH of pave blocks, to catch edge/face *common blocks* that only became
   coincident after tolerance growth.

So EF **adds**: (a) new vertices at piercings, (b) **new paves on the pierced edge**, (c) EF
interference records + `FaceInfo` *In* sets, (d) edge/face common blocks, (e) tolerance growth
on nearby existing vertices.

---

## 2. What our source has and does not have (read-only inspection)

### Does not exist

- **There is no edge/curve x face/surface intersection entry point in the kernel API at all.**
  `src/intersection.h` exposes `curve_plane` (`:364-419`, curve vs *infinite plane*),
  `surface_plane` (`:453`), `surface_plane_uv` (`:471`), `surface_surface` (`:489`),
  `cut_curves_on_surface` (`:500`). There is no `curve_surface` / `edge_face`. Confirmed by
  listing every `static` in the header.
- No stage named EF or VF runs by default anywhere in `brep.cpp` / `brep_section.cpp`.

### Exists but is diagnostic-only or opt-in

- **`src/brep_section.cpp:2160-2247` — `SESSION_EF_DIAG`.** This *is* a real edge x surface
  solver: coarse 32-sample scan of every edge of one operand against every surface of the other,
  then a 3x3 Gauss-Newton on `C(t) = S(u,v)` (`:2186-2213`) with the Jacobian
  `[dC/dt, -dS/du, -dS/dv]`. It converges to `conv_tol*100`. **It prints a histogram and injects
  nothing** — `efpts` is local, used only for `[EFDIAG] dangle vN dEF=...` lines (`:2242-2246`).
- **`src/brep.cpp:3560-3609` — `SESSION_EF_PAVES`.** Opt-in, and only on the non-scaffold
  (quadric) path (`else if` after `if (scaf)` at `:3557`). It is **not a solver**: it takes the
  *endpoints of the already-computed cut curves* and keeps those that lie within
  `diag*2e-3` of one of this operand's edges (`:3591-3602`). Comment at `:3562` records it
  regressed the quadric path (cone x cyl 2->8 naked), so it is off.
- **`src/brep_section.cpp:1832-1990` — `SESSION_EF_MARCH`.** EF-*anchored* continuation march:
  for a dangling chain end, find an other-operand edge through it (`find_cont`, `:1861-1873`) and
  march the section onto the next face across that edge, gated by a pi/6 continuation-angle test
  (`:1906-1917`). It consumes EF junctions, it does not discover them, and it only fires on ends
  that are already dangling. Off by default; enabled by the `SESSION_AUTO` ladder
  (`brep.cpp:8341-8355`), which is itself opt-in (`brep.cpp:8213`).

### Exists and does the job by another route (the reason the claim fails)

- **`src/brep_section.cpp:1227-1282` — trim crossings.** For every section chain (surfA x surfB),
  every crossing of the chain's UV image with the *face trim loops* of both operands is a pave
  (`seg_seg_2d` at `:1244`), Newton-refined onto the exact section-∩-trim point by
  `refine_trim_pave` (`:330`, 3 unknowns t,u,v).
  **A crossing of the (FA1 x FB) section with FA1's trim boundary is, by definition, the point
  where the edge bounding FA1 pierces face FB — i.e. an EF interference.** These become scaffold
  vertices (`:1423-1429`), which become the shared pave set for both operands' splits
  (`brep.cpp:3557-3559`) and are minted as result topology vertices (`brep.cpp:3676-3686`).
- Supporting paves: chart-border touch transitions (`:1289-1306`), chain x chain crossings on a
  shared surface with exact 3-surface corner solve (`:1316-1345`).
- VV-equivalent: SD-weld / `MakeSDVertices` port at `brep_section.cpp:1677-1690`; seed
  unification of original corners onto paves at `brep.cpp:3657-3673`.
- Scaffold is on for these configs: `brep.cpp:8432-8435` — imported freeform with a deg>=3
  surface. Both chairs qualify.

---

## 3. Measured EF/VF census (independent, FreeCAD 1.1.1 = OCCT)

Method: `/home/petras/hunt_efvf/ef_probe.py`. For every edge of A x every face of B and every
edge of B x every face of A (bbox prefiltered), `edge.section(face)`; each resulting vertex is
kept as a **true EF** point only if it is >1e-4 from the face's own wires *and* >1e-4 from the
edge's own endpoints (otherwise it is EE/VV territory). Dedup at 1e-5.

Operands: `chair0.stp` and `/home/petras/fc_inspect/operands/B_<cfg>.step`, each 20 faces /
54 edges / 36 vertices, OCCT volume A 80.2969, B 80.2968.

| config | raw hits | unique | **true EF** | **VF** (vertex on face, <1e-6) | min vertex->other-face distance |
|---|---:|---:|---:|---:|---:|
| x20    | 34 | 34 | **34** | **0** | 0.042859 |
| x13y29 | 42 | 42 | **42** | **0** | 0.007938 |

**VF is entirely absent from both configs.** Nothing is even close: the nearest any operand
vertex comes to the other operand's surface is 0.043 (x20) and 0.0079 (x13y29). Attributing any
part of these two cells to `BOPAlgo_PaveFiller_4.cxx` is unsupportable.

### Completeness proof of the census

`/home/petras/hunt_efvf/accounting.py` classifies every vertex of every OCCT result as
*original operand vertex* or *EF point* (1e-4):

| result | verts | original | EF | **unexplained** |
|---|---:|---:|---:|---:|
| x20 cut | 70 | 36 | 34 | **0** |
| x20 common | 4 | 0 | 4 | **0** |
| x20 fuse | 106 | 72 | 34 | **0** |
| x13y29 cut | 60 | 29 | 31 | **0** |
| x13y29 common | 51 | 9 | 42 | **0** |
| x13y29 fuse | 105 | 63 | 42 | **0** |

Zero unexplained vertices in six independent results. The census is complete, and **EF is the
only source of new vertices in these two booleans** (no EE-born vertices, no VF).

### OCCT references, recomputed here (they match the brief)

| config | op | faces | solids | shells | verts | valid | volume |
|---|---|---:|---:|---:|---:|:--:|---:|
| x20 | cut | 38 | 1 | 1 | 70 | Y | 80.2966 |
| x20 | common | 4 | 1 | 1 | 4 | Y | 0.0005 |
| x20 | fuse | 68 | 1 | 1 | 106 | Y | 160.5934 |
| x13y29 | cut | 29 | 1 | 1 | 60 | Y | 48.4734 |
| x13y29 | common | 31 | 3 | 3 | 51 | Y | 31.8239 |
| x13y29 | fuse | 50 | 1 | 1 | 105 | Y | 128.7708 |

x20 is a **graze with a huge section**: only 0.0005 of volume is removed, yet A's 20 faces
become 38 and 34 new vertices appear. That is the config's real character.

---

## 4. Does our kernel have vertices at those points? — YES, almost all of them

Runs: `build/main_7_efvf` (my copy), base path, `SESSION_AUTO` **not** set, operands re-imported
from the same STEP files the OCCT reference used.

```
x20    [SCAF] chains=30 segs=37 verts=39 paves(tA=46 tB=38 x=24 v=1 c=0) drop(verdict=21 micro=19) bridge(march=0 weld=0 resid=0) tol3=3.577e-02
x13y29 [SCAF] chains=43 segs=45 verts=46 paves(tA=52 tB=60 x=331 v=2 c=0) drop(verdict=33 micro=60) bridge(march=0 weld=0 resid=0) tol3=3.419e-02
```

### 4a. Scaffold vertices vs the OCCT EF points (`scaf_match2.py` on `[SCAF-V]` dumps)

| config | scaffold verts | EF <1e-4 | <1e-3 | <1e-2 | <tol3 | **absent (>=tol3)** |
|---|---:|---:|---:|---:|---:|---:|
| x20 (tol3=0.03577) | 39 | 30 | 3 | 0 | 0 | **1** |
| x13y29 (tol3=0.03419) | 46 | 37 | 2 | 1 | 2 | **0** |

- x20: the one absent point is `Aedge e8 x Bface f1` at `(5.59791, 4.02297, 2.42683)`,
  nearest scaffold vertex 0.12725 away.
- x13y29: nothing absent. Three points are *displaced*: 0.01627, 0.01390, 0.00582 (0.48x, 0.41x,
  0.17x of tol3). One EF point — `Bedge e5 x Aface f1` at `(12.01973, 3.42214, 1.97867)` — is
  represented **twice**, by V7 (d=0.0058, valence 3) and V6 (d=0.0283, valence 1). That is a
  junction derived independently from two chains and never unified.

### 4b. Our shipped result vs the OCCT EF points (`ref_and_match.py`, FC_RES = our exported result)

| config / op | our result | EF verts <1e-4 | <1e-2 | <0.1 | <1 | worst |
|---|---|---:|---:|---:|---:|---:|
| x20 cut | 2 shells, both OPEN, 35 faces, 75 verts, 9 naked | 30 | 3 | 0 | 1 | **0.12721** |
| x20 cut, `SESSION_AUTO=1` | 1 shell, OPEN, 34 faces, 73 verts, 7 naked | 26 | 5 | 0 | 3 | **0.38321** |
| x20 common | 2 shells, both OPEN, 21 faces, 48 verts, 13 naked | 29 | 3 | 1 | 1 | **0.12721** |
| x13y29 cut | 2 shells (1 open + 1 closed 10-face), 38 faces, 82 verts, 18 naked | 39 | 3 | 0 | 0 | **0.00380** |

Two things worth naming in that table:

- **x20 common is the clearest disproof of the EF attribution.** OCCT's answer is a 4-face,
  4-vertex solid of volume 0.0005. Ours is 21 faces / 48 vertices / 13 naked / open, kernel-
  reported `vol 26.3426`. Yet **32 of the 34 EF points are present in it**. We are not missing
  interference geometry; we are keeping the wrong side of it. That is a keep-verdict /
  classification failure.
- **The `SESSION_AUTO` ladder improves the naked count by deleting the problem, not fixing it.**
  It reaches the quoted frontier (`faces 34 solid 0 naked 7`) by dropping the whole graze
  fragment: the stray 1-face shell disappears (2 shells -> 1) and three EF vertices go with it
  (worst miss rises from 0.127 to 0.383). Across all nine ladder passes,
  `bridge(march=0 weld=0|2)` — **`SESSION_EF_MARCH` never fires once on x20**, even when
  enabled: `find_cont` (`brep_section.cpp:1861`) or the pi/6 gate (`:1906`) rejects all four
  dangles.

(kernel's own metric on the same runs: x20 `faces 35 solid 0 naked 9`, x13y29 `faces 38 solid 0
naked 16`; FreeCAD's per-shell one-adjacent-face count gives 9 and 18. The metric-independent
statement is `Shell.isClosed() == False`. Calibration for the naked metric: OCCT's own x20 cut —
1 valid closed solid — reports naked=3 under the global one-adjacent-face count, so ~3 of any
count here are periodic-seam false positives.)

**33 of 34 EF points for x20 and 42 of 42 for x13y29 exist as vertices in our result.** Reverse
accounting (`ours_accounting.py`, tol 0.005): x20 75 verts = 35 original + 34 EF + 6 extra;
x13y29 82 verts = 35 original + 42 EF + 5 extra.

### 4c. The pierced edge IS split at the piercing (`valence.py`)

An EF point that has been properly paved splits the pierced edge, so the node is 3- or 4-valent,
never a 2-valent point sitting on a curve.

```
x20      [OCCT cut] val1=1 val2=12 val3=5  val4=16   [OURS cut] val2=4 val3=28 val4=1  (1 point absent)
x13y29   [OCCT cut] val3=31 (11 pts not in cut)      [OURS cut] val2=1 val3=39 val4=1 val5=1
```

Valence >= 3 everywhere in ours except 4 (x20) / 1 (x13y29) nodes. **The paves on the pierced
edges exist.**

### 4d. Our own EF solver, when asked, finds them

`SESSION_EF_DIAG=1` on x20 reports `ef_points=39 dangles=4`, and **all four dangling scaffold
vertices are within 2e-5 of an EF point**:

```
[EFDIAG] dangle v6 dEF=0.00000   [EFDIAG] dangle v8 dEF=0.00002
[EFDIAG] dangle v28 dEF=0.00001  [EFDIAG] dangle v36 dEF=0.00000
[EFDIAG] ef_points=39 dangles=4 weld_tol=0.0358
```

The dangles are *at* the EF junctions, exactly. They are not dangling because the anchor is
missing; they are dangling because the section branch that should continue through the anchor
was never marched.

---

## 5. Where the defect actually is (`naked_where.py`)

### x20 — 9 naked edges, 2 open shells

- **Shell 1 = a single stray face with 3 naked edges**, midpoints `(5.7566,4.0949,2.4436)`,
  `(5.8336,4.0481,2.4860)`, `(5.6505,4.1016,2.4667)` — all within 0.11 of the **one missing EF
  point** `(5.59791,4.02297,2.42683)` and of the two dangling scaffold vertices V6/V8. This
  3-naked-edge fragment **is** the EF gap, and it is 3 of 9 naked edges.
- **Shell 0 = 34 faces, open, 6 naked edges.** **8 of their 12 endpoints sit at distance
  0.0000 from an EF point**; 3 of the 6 edges have *both* endpoints there, 5 of 6 have at least
  one. The anchors are present; the second face using the edge is not. This is a face-emission /
  keep-verdict failure, not a paving failure.
- Valence deficit: OCCT's cut has 16 four-valent EF nodes, ours has 1. A 4-valent EF node means
  both section branches (onto FA1 and onto FA2) pass through. Ours are 3-valent -> **one section
  branch is missing at most junctions**. That is exactly the gap `SESSION_EF_MARCH` targets, and
  `bridge(march=0)` on the default path confirms it never fires.

### x13y29 — 18 naked edges, 1 open + 1 closed shell

- **8 of 18 have BOTH endpoints exactly on EF points (0.0000); 12 of 18 have at least one.**
  Anchors present, mate missing.
- **2 are micro-slivers**, lengths 0.0065 and 0.0030, at `(12.0175,3.4313,1.9794)` — precisely
  the doubly-represented EF point (V6/V7, 0.0283 apart). **This is the only defect in either
  config genuinely caused by EF-point handling**, and it is caused by *duplication + displacement*,
  not absence.
- **3 are long naked edges of 10.39, 7.04 and 3.35 units** at z ~ 4-7, **2.1 to 4.1 away from any
  EF point**. A whole face is missing far outside the intersection zone. Nothing to do with EF.

### The extra vertices we mint (`spurious_probe.py`)

x20's 6 extras and x13y29's 3 extras lie **on both operands' faces** (d_face <= 5e-5) but
**0.10-1.13 from any operand edge**. They are interior points of the section curve where two
section chains cross *at the surface level* (`n_paves_xing` = 24 for x20, 331 for x13y29,
`brep_section.cpp:1316-1345`). OCCT never creates a vertex there. So the scaffold's error on
these configs is a small **surplus** of junctions, not a deficit.

---

## 6. Answering the four questions

1. **Absence verified?** Partly. There is no EF/VF *stage*, no edge x face solver in the
   production API (`intersection.h` has none), and the only real solver is diagnostic-only
   (`brep_section.cpp:2165`). But the *function* is covered: section-∩-trim paving
   (`brep_section.cpp:1227-1282`) produces the EF piercing points. VF is covered vacuously —
   there are none.

2. **Cost quantified?** Measured, not estimated. x20 cut: 1 of 34 EF points missing (d=0.127),
   costing a 1-face stray shell = 3 of 9 naked edges. x13y29 cut: 0 of 42 missing; 1 duplicated
   at 0.0283 separation, costing 2 micro-sliver naked edges of 18. **Total defect attributable to
   EF paving: 3/9 naked edges in x20 cut, 2/18 in x13y29 cut, and 0 of x20 common's 13.** The
   remainder sit at, or far from, EF points that are correctly paved.

3. **What a minimal EF stage would still buy, and its size.** Not the points — those we have.
   What OCCT gets from computing EF *independently of the FF section* is:
   - **Unconditional coverage.** Our paves are conditional on the SSI chain existing and being
     marched through the crossing. x20's missing point is in the graze cluster where the marcher
     fragmented (2 dangles, one lost crossing). An edge x face solve is a 3-unknown Newton on a
     *transversal* system; it does not care that the surfaces graze.
   - **Single-source identity.** OCCT computes each (edge, face) piercing **once** and mints one
     vertex (`MakeNewVertex`, `_5.cxx:413`), then unifies (`UpdateVertex`). We derive the same
     point once per chain and rely on 3D welding at tol3 to reconcile — which failed once in
     x13y29 (0.0283 < tol3 0.0342 yet unmerged).
   - **Exactness.** Our worst pave error is 0.01627 (x13y29) against a Newton-attainable 1e-9;
     the diagnostic already converges to `conv_tol*100`.
   - **Branch seeds.** An EF point with only one incident section branch is a *proof* that a
     branch is missing (the val3-where-OCCT-has-val4 signature, 16 nodes in x20). Independent EF
     turns that into a hard, checkable invariant and gives `SESSION_EF_MARCH` its anchors
     unconditionally.
   - **Common blocks.** `_5.cxx:545-565` handles the EDGE-type common part (edge lying *in* a
     face). We have no analog at all. Not exercised by these two configs, but it is the
     mated-face case, which is exactly the *unrotated* chair pair.

   **Size.** OCCT spends 390 + 1199 = 1589 lines in PaveFiller_4/5, on top of 875 + 2639 + 1039 =
   4553 lines of `IntTools` solver plus `BOPTools_AlgoTools` (2416). We would not need most of
   it, because we already have: the 3x3 Newton on `C(t)=S(u,v)` (already written,
   `brep_section.cpp:2186-2213`, 28 lines), `Closest::surface_point` for seeding,
   `face_loops_uv` / `in_faces_uv` for the in-face test, AABB/BVH prefilters
   (`aabb.cpp`, `spatial_bvh.cpp`), and the pave-capture, weld and seed-unification machinery
   (`brep.cpp:3611-3700`). Missing pieces are: bbox-driven pair enumeration, multi-root scanning
   per (edge, surface) span (the current 32-sample local-min scan takes one root per span and
   would miss a double crossing), the on-face (not on-surface) acceptance test, dedup/weld into
   `scaf.vertices`, and splitting the affected section segments at the new anchor.
   **Realistic size: 250-400 lines in `brep_section.cpp`**, plus a `curve_surface` entry point in
   `intersection.h`. The EDGE-type common-block half would roughly double that.

4. **Refutation stated plainly.** The panel asserted the stage was absent and blamed x20 and
   x13y29 on it. Measured: our result contains **33/34** and **42/42** of the true EF points,
   with the pierced edges split at them (valence >= 3), and **zero** VF interferences exist in
   either config. The defects are dominated by *missing section branches through junctions that
   are correctly paved* (x20) and *a missing face far outside the intersection zone* (x13y29),
   neither of which an EF stage addresses. An EF stage is worth building — for unconditional
   coverage, single-source identity, exactness, branch-completeness invariants, and common
   blocks — but it is **not** the explanation for these two cells, and sizing the campaign around
   it would misallocate the effort.

---

## 7. Reproduction

Working dir `/home/petras/hunt_efvf/`. Binary: my own copy `build/main_7_efvf`
(`src/` untouched, nothing rebuilt).

```bash
# operand pairs
mkdir -p /home/petras/hunt_efvf/x20 /home/petras/hunt_efvf/x13y29
cp /home/petras/fc_inspect/operands/chair0.stp      <dir>/chair0.stp
cp /home/petras/fc_inspect/operands/B_<cfg>.step    <dir>/chair1.stp

# kernel (base path, no AUTO)
cd /home/petras/code/code_rust/session/session_cpp
(ulimit -v 4194304; SESSION_NO_ROT=1 SESSION_CHAIRS=<dir> SESSION_OP=cut \
   timeout 900 ./build/main_7_efvf SKIPMATRIX)
# scaffold vertex dump:  add SESSION_FAST=1 SESSION_SPLIT_DBG=1   -> [SCAF-V] lines
# our EF solver:         add SESSION_FAST=1 SESSION_EF_DIAG=1     -> [EFDIAG] lines

# OCCT-side measurement (all paths under /home/petras, args via env only)
cd /home/petras/fc_inspect
FC_A=<dir>/chair0.stp FC_B=<dir>/chair1.stp FC_JSON=/home/petras/hunt_efvf/ef_<cfg>.json \
  /snap/bin/freecad.cmd /home/petras/hunt_efvf/ef_probe.py           # EF/VF census
FC_A=.. FC_B=.. FC_JSON=.. FC_RES=<dir>/chair0_cut_chair1.step \
  /snap/bin/freecad.cmd /home/petras/hunt_efvf/ref_and_match.py      # reference + match
  /snap/bin/freecad.cmd /home/petras/hunt_efvf/accounting.py         # vertex accounting + VF margin
  /snap/bin/freecad.cmd /home/petras/hunt_efvf/ours_accounting.py    # reverse accounting
  /snap/bin/freecad.cmd /home/petras/hunt_efvf/valence.py            # EF-node valence
FC_RES=.. FC_JSON=.. /snap/bin/freecad.cmd /home/petras/hunt_efvf/naked_where.py
python3 /home/petras/hunt_efvf/scaf_match2.py <scafdbg.log> <ef.json> <tol3>
```

Artifacts: `ef_x20.json`, `ef_x13y29.json`, `x20_cut.log`, `x13y29_cut.log`,
`x20_scafdbg.log`, `x13_scafdbg.log`, `x20_efdiag.log`, and the exported results
`x20/chair0_cut_chair1.step`, `x13y29/chair0_cut_chair1.step`.

### Not run / not measured

- `SESSION_AUTO=1` was run only for **x20 cut** (completed: `faces 34 solid 0 naked 7 vol
  51.5635`, matching the quoted frontier). x13y29 was measured on the **base path** only
  (`faces 38 solid 0 naked 16`); its ladder result of naked 12 is quoted from the commit log,
  not observed here.
- Our kernel's **fuse** was not run for either config; **common** was run for x20 only
  (`faces 21 solid 0 naked 13 vol 26.3426`) and not for x13y29.
- Our STEP reader gives A vol 80.3011 vs OCCT 80.2969 (5e-5 relative). All EF matching above is
  across that fidelity gap, which makes the 1e-4 agreement stronger, not weaker.
- The EF census uses OCCT's own `BRepAlgoAPI_Section` as the oracle, so it shares OCCT's
  tangency handling. Its completeness is established independently by the zero-unexplained-vertex
  accounting in section 3, not by trusting the section call.
