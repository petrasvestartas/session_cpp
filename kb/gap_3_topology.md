# gap_3_topology.md — THE TOPOLOGY LAYER: identity, the shared data structure, interferences

**Scope.** What must be implemented so that a boolean's *entities* — vertices, edges, pave blocks —
are the same object when they are the same thing. Covers (a) identity / cross-pair fusion,
(b) the missing VF and EF stages, (c) common blocks. Ranked by measured impact on the 63-cell
primitive matrix.

---

## 0. HEADLINE — the ranking, with the number that earns each rank

| # | Gap | Measured impact | Where |
|---|---|---|---|
| **T1** | **Our result carries 2–3 distinct vertices where OCCT carries 1**, at every node born from an edge/face piercing. Split 1.5e-4 … 4.3e-3. | **9 of the 12 failing cells I could measure** (15 fail in total; `coneR×cyl`'s 3 are not measurable, §0.1). vol rel 3.4e-5 … 2.6e-4 on 8 of them; `is_solid=0` + 5.6e-2 on `cyl×cylR fuse`; partition identity `cut+common−vol(A)` violated by **+1.196e-3** on `boxR×sph`. **0 of 9 passing control cells has a single duplicated node.** | §1.2, §2.1, §4.1 |
| **T2** | **v2 has the fix and does not run it.** `v2int::V2Interf` (VV/VE/EE/VF/EF, `main_15` 29/29) is never instantiated by the v2 pipeline. The stated blocker — a `V2State` name collision — **does not exist**: the two enums are in different nested namespaces and `src/v2/v2_dump.h` already includes both headers in one TU. | v2 `cyl×cyl`: 2 node pairs **2.2e-6** apart left unfused → **+12 % volume** (53.194 vs 47.682), 2 naked, `closure_residual=7.8e-9` ("one weld away"). Cone family: +8 / +2 / +10 spurious arena vertices, up to **+349 %** volume, 21 naked. | §2.4, §3.T2, §4.2 |
| **T3** | **v2's section stage mints a new arena vertex per pave and reconciles by distance** with band `tol_i+tol_k+fuzzy ≈ 3e-7`; it never looks up the vertex the EF stage would have created, nor the pave already on the boundary edge. | The 2.2e-6 gap above is **7× the fusion band**, and the arena's own recorded tolerance (1.17e-5) is **5× the gap** — i.e. the band is not even self-consistent. | §2.4, §3.T3 |
| **T4** | v1 has **no VF and no EF stage**, and on the quadric (primitive) path its *entire* vertex-identity mechanism is a 1e-6 coordinate grid. | **Zero measured impact as missing points**: every EF-born OCCT vertex is present in our result in **12 of 12** measured failing cells (worst position error 7.2e-4). VF > 0 in only **2 of the 31 configurations traced**, neither of them a failing matrix cell. The cost is entirely T1 (identity), not coverage. | §1.3, §2.2, §4.3 |
| **T5** | **Common blocks** (both flavours) exist in `BdsArena` and in `V2Interf`, are unreachable in v1 production, and are unreachable in the v2 pipeline for the same reason as T2. | **No measured impact on the primitive matrix.** The only cell that exercises them (`box×boxR`: OCCT `cb=10`, `VF=6`, 2 same-domain FF pairs) **passes to 1e-15 in all three ops**. Impact is on the chair mated-face configuration and on OCCT-authored coincident inputs (v2 T3: OCCT-authored `box×box` → 0.333/21.67/64.33, all open). | §1.4, §2.3, §4.4 |
| **—** | `box×torR`'s 4 absent result vertices | **Not a topology-layer gap.** The missing nodes are *not* EF-born; they are FF section nodes (OCCT tol 1.000001e-06, i.e. marched). Assign to the section/marcher layer or effort is misallocated. | §4.5 |

**One-sentence delta.** OCCT computes each node once, in a stage that owns it, stores it as an
integer index in one arena, and every later consumer *refers* to it; we compute each node once per
consumer and try to reconcile the copies afterwards by coordinate. Everything in §3 is that
sentence made executable.

---

## 0.1 Measurement provenance

All new numbers below were produced during this task and are reproducible.

* **OCCT ground truth.** `validation/occt_trace/build/occt_trace` (prebuilt, OCCT 8.0.0.rc2-a66b3fd6,
  `SetRunParallel(false)`, `SetUseOBB(false)`). 38 existing trace files (19 operand configurations)
  plus **36 new ones I generated** (12 configurations × 3 ops) for the exact placements of
  `main_7.cpp:206-248` — 74 files / **31 configurations** in total. New traces in
  `<scratch>/tr/{coneR_cyl,cyl_cylR,boxR_sph,box_torR,boxR_cyl}_{cut,common,fuse}.trace` and
  `m_{box_boxR,box_sph,box_cyl,sph_cyl,cyl_cyl2,cone_cyl,box_box2}_{cut,common,fuse}.trace`.
  Every new trace reproduces the shipped OCCT cache to ≥8 digits (e.g. `coneR×cyl cut`
  `res_vol=12.307399` vs cache `12.307413`/FreeCAD `12.3073990`), so the mapping from our
  `Place` records to tracer operand strings is verified, not assumed:

  ```
  cyl   -> cylinder,r=1.5,h=6,tz=-3            cylR  -> cylinder,r=1.5,h=6,roty=45,tx=-1,tz=-1.8
  box   -> box,dx=4,dy=4,dz=4,center           boxR  -> box,...,center,rotz=30,tx=2,ty=1
  sph   -> sphere,r=2.5                        torR  -> torus,r1=2,r2=0.8,rotx=45
  cone  -> cone,r1=2,r2=0,h=4,tz=-2            coneR -> cone,r1=2,r2=0,h=4,rotx=-90,ty=-2.8
  ```

* **Our side.** `session_cpp/build/main_7`, **built 2026-07-26 08:21**, run with
  `SESSION_STEP_DIR`; result vertices extracted from the third `MANIFOLD_SOLID_BREP` of each
  written STEP (`file_step.cpp:1757` writes `%.15g`, so vertex coordinates are lossless — the
  3.3e-3 STEP loss recorded in `kb/hunt_oriented_primitives.md §4` is a *surface/pcurve* loss and
  does not touch this measurement; all separations reported here are ≥1.5e-4, ten decades above
  the format's resolution).
* **Staleness caveat, stated rather than hidden.** That binary predates today's cubic-pcurve work.
  On it I measure **42 of the 45 non-oriented cells OK** (the remaining 3, `tor×tor`, exceeded my
  110 s budget and are unmeasured) and **3 of the 18 oriented cells OK** (`box×boxR` only), i.e.
  ~45 OK / 15 FAIL / 3 unmeasured, against the brief's 34 OK / 29 FAIL for the current source. I did not reconcile that difference
  and no claim below depends on it: every impact number is tied to a cell I measured myself, with
  both sides of the comparison taken from the same run.
* **`coneR×cyl` was not measurable on our side**: the 08:21 binary produces no output in 120 s for
  `cut` (consistent with `kb/hunt_oriented_primitives.md §1`, ">600 s twice"). Its OCCT reference
  is now traced (§4.1) and is *trivially small*: **1 section curve, 1 EF interference, 1 new
  vertex, 2 result vertices, 3 result faces**. The brief's 43.1452-vs-12.3074 figure is quoted,
  not re-measured.

---

## 1. WHAT OCCT DOES — from the traces

### 1.1 Identity is an integer index into one arena

Every entity in `BOPDS_DS` is an index. The trace's `SI` records show it directly
(`cyl_cylR_cut`): `RANGE i=0 first=0 last=12`, `RANGE i=1 first=13 last=25`, `nbsource=26` — the
two operands' sub-shapes occupy disjoint index ranges of one array, and *rank* is a table lookup,
not a geometric test. `SI i=2 type=FACE rank=0 ... nsub=4,5,6,7,8` — a face names its sub-edges by
index.

Consequences visible in the dumps:

* A pave names a vertex **by index**: `PAVE`/`PB` records carry `v1=`/`v2=` integers, never points.
* A pave block names its materialised edge by index: `PB ... edge=72`.
* Two pave blocks that are the same edge carry the **same `edge=`** (see §1.4).
* Same-domain vertices are a **map**, not a merge: `SD tag=final i=38 sd=55` — index 38 *resolves
  to* 55; both records survive, and every consumer resolves through the map
  (`BOPAlgo_PaveFiller_1.cxx:136 MakeSDVertices`).

### 1.2 Cross-pair fusion — how a section edge computed for (F1,F2) becomes the same entity as the one for (F1,F3)

Three distinct mechanisms, all measured, in the order they fire.

**(i) The node is created BEFORE any section curve exists — by the EF stage — and both face pairs
pave at that index.** This is the load-bearing one and it is fully visible in `cyl_cylR_cut`:

```
IEF  tag=final i1=21 i2=2  ctype=VERTEX r1=1.16959499:1.16959559 new=27
IEF  tag=final i1=21 i2=2  ctype=VERTEX r1=5.11358972:5.11359032 new=28
```

Edge 21 (the boundary circle of operand B's cap face 24) pierces face 2 (operand A's lateral
cylinder) at two parameters; stage 5 mints DS vertices 27 and 28 and splits edge 21 into three
pave blocks *at exactly those parameters*:

```
FIPB tag=final f=24 set=On k=0 orig=21 t0=0          t1=1.16959529 edge=35
FIPB tag=final f=24 set=On k=1 orig=21 t0=1.16959529 t1=5.11359002 edge=36
FIPB tag=final f=24 set=On k=2 orig=21 t0=5.11359002 t1=6.28318531 edge=37
```

Only then does `PerformFF` run. The section of the pair **(2,24)** is bounded by those same
indices, and so are two arcs of the *different* pair **(2,15)**:

```
SECPB tag=final f1=2 f2=24 c=0 t0=5.11359002 t1=7.45278059 v1=27 v2=28 edge=48
SECPB tag=final f1=2 f2=15 c=5 t0=1.57079633 t1=1.97199737 v1=52 v2=27 edge=53
SECPB tag=final f1=2 f2=15 c=6 t0=4.31118794 t1=4.71238898 v1=28 v2=49 edge=51
```

Vertex 27 is shared by pair (2,15) curve 5, pair (2,24) curve 0, and the split of original edge 21.
**No distance test was involved anywhere.** `BOPAlgo_PaveFiller_5.cxx:413`
(`BOPTools_AlgoTools::MakeNewVertex(aE, aT, aF, aVnew)`) mints it; `:537-542` queues it *with the
pave block it must split*; `:578 PerformNewVertices` inserts the pave; `:585 UpdateFaceInfoIn`
files it.

**(ii) One pave block object appears in the `Sc` set of BOTH faces of its pair, and a face that
belongs to many pairs accumulates all of them in ONE set.** `sph_box_cut`, sphere face 2:

```
FI   tag=final f=2  nIn=0 nOn=5 nSc=7
FIPB tag=final f=2  set=Sc orig=-1 edge=55 / 56 / 57 / 58 / 59 / 60 / 61
FI   tag=final f=11 nSc=1  -> edge=61      FI f=21 nSc=2 -> edge=60, 59
FI   tag=final f=31 nSc=1  -> edge=58      FI f=35 nSc=1 -> edge=57
FI   tag=final f=39 nSc=1  -> edge=56      FI f=41 nSc=1 -> edge=55
```

Six different box faces; seven section pave blocks; each block's integer appears in exactly two
`Sc` sets. The face splitter for face 2 consumes `On ∪ Sc` as one index list. `orig=-1` marks
"section, no original edge". Symmetry is exact (`kb/occt_trace_findings.md` Q2 quotes the same
structure for `sph_cyl_roty45`: `f=2` and `f=11` both `nSc=3` with edges 31/32/33).

**(iii) Whatever duplicates still arise are collapsed by the SD map in `PostTreatFF`, and every
reference is rewritten through it.** `cyl_cylR_cut` (`sd=10` in the summary):

```
DSVERT i=39,40,46,47 p=0,-1.5,-0.8               -> SD sd=49   (4 copies -> 1)
DSVERT i=41,42,44,45 p=0,1.5,-0.8                -> SD sd=52   (4 copies -> 1)
DSVERT i=38,43       p=-1.5,0,-0.178679656       -> SD sd=55   (2 copies -> 1)
```

and the `SECPB` records above already name 49/52/55, never 38–47.
`BOPAlgo_PaveFiller_6.cxx:1113 MakeSDVerticesFF` → `:1152 MakeSDVertices`. Note the copies are
**bit-identical points** (`0,-1.5,-0.8` printed identically four times): they arise from four arcs
meeting at one node, not from four different computations of it. The fusion is a bookkeeping step
over exact duplicates, **not** a tolerance rescue.

**Negative result that constrains the port:** `BOPDS_CommonBlock` is **not** the cross-pair
mechanism for transverse sections. `cb=0` in every one of the 28 transverse configurations
(19 corpus + 12 new, minus the 3 coincident ones). Cross-pair identity for transverse booleans is realised entirely at
the **node** level (i + iii) plus shared `Sc` membership (ii). Common blocks appear only for
coincident geometry (§1.4).

### 1.3 What the EF and VF stages actually add — a census over 31 configurations / 74 trace files

Measured stage order (all 38 corpus cases): `Init → VV → VE → EE → VF → EF → FF`
(`kb/occt_trace_findings.md §0`). So **every interference that can create a vertex has already run
before the first section curve exists.**

| stage | configurations with count > 0 (of 31) | max count | what it adds |
|---|---|---|---|
| VV | 1 (`box_box_touch`) | 4 | SD map over coincident source vertices; 8 sources → 4 **new** DS vertices, indices ≥ `NbSourceShapes` |
| VE | 2 (`box_box_half_fuse`, `sph_cyl_roty23578`) | 4 | paves on an edge naming an existing vertex by index |
| EE | 10 | 4 | new vertex + a pave on each edge; overlap → CommonBlock |
| **VF** | **2** (`sph_cyl_roty23578` = 1, `box×boxR` = 6) | 6 | vertex → `FaceInfo::VerticesIn`, tolerance growth; **no paves** (`BOPAlgo_PaveFiller_4.cxx:139,283-297`) |
| **EF** | **22** | 10 | new vertex **+ pave on the pierced edge** + `InterfEF` + `FaceInfo` In set (`BOPAlgo_PaveFiller_5.cxx:406-542`) |

EF counts on the six oriented (failing) pairs: `coneR×cyl` 1, `cyl×cylR` 3, `boxR×cyl` 6,
`box×torR` 6, `boxR×sph` 10, `box×boxR` 10. So the EF stage on the primitive matrix produces
**1–10 interferences per cell** — small, and every one that mints a vertex is load-bearing for the
section's paving (§1.2(i)). Not every EF interference mints one: in `box×boxR` all 10 have
`new=-1` (the piercing landed on an existing vertex or inside a pave's end range, so OCCT grows a
tolerance or forces a VF instead — `BOPAlgo_PaveFiller_5.cxx:454,482-501,631`).

### 1.4 Common blocks — both flavours, when they are created, and what they are for

A `BOPDS_CommonBlock` is *a set of pave blocks that are one edge*, plus *a set of faces the edge
lies on*. Two flavours are exercised in the corpus.

**Flavour A — edge-on-edge (`npb ≥ 2`, `nfaces=0`).** `box_box_touch_fuse`:

```
CB tag=final id=0 tol=1e-07 edge=72 npb=2 pbs=14:0:4|38:0:4 nfaces=0 faces=-
PB tag=final e=14 k=0 orig=14 t0=0 t1=4 v1=69 v2=68 edge=72 etol=1e-07 cb=0
PB tag=final e=38 k=0 orig=38 t0=0 t1=4 v1=69 v2=68 edge=72 etol=1e-07 cb=0
```

Edge 14 of operand A and edge 38 of operand B are geometrically the same line; both pave blocks
carry `edge=72`, i.e. **one materialised result edge for two source edges**, and both name the same
end vertices 69/68 (which are themselves the VV-fused new vertices). 4 such blocks in that case.

**Flavour B — edge-on-face (`npb = 1`, `nfaces = 1`).** `box_box_half_fuse` id=2/3 and, more
richly, my new `m_box_boxR_cut` (a *primitive-matrix* cell) with **10 of them, all flavour B**:

```
CB id=4 tol=1e-07 edge=41 npb=1 pbs=41:0:4          nfaces=1 faces=32
CB id=6 tol=1e-07 edge=82 npb=1 pbs=58:0:0.845299   nfaces=1 faces=30
CB id=0 tol=1e-07 edge=77 npb=1 pbs=17:0.690599:4   nfaces=1 faces=66
```

Meaning: this pave block **lies in** that face. The edge is not split by the face and the face does
not section it; the face's splitter simply uses the edge as a boundary. Note `pbs=17:0.690599:4`
— a *partial* pave block: only part of edge 17 lies in face 66, and the coincident part was
isolated by EE/EF first.

Creation sites: EE overlap → `BOPAlgo_Tools::PerformCommonBlocks` (`BOPAlgo_Tools.cxx:107,191`);
EF `ctype=EDGE` → `BOPAlgo_PaveFiller_5.cxx:545-565` (`FillMap`) then `:576 PerformCommonBlocks`.

`m_box_boxR` also carries `VF=6` (six box corners lying on the other box's coplanar face,
`IVF ... new=-1` — no new vertex, the existing one is filed into `VerticesIn`) and
`fftangent=2` (`IFF i1=30 i2=64 tangent=1 ncurves=0`, `IFF i1=32 i2=66 tangent=1 ncurves=0`) —
the two coplanar face pairs. `kb/occt_trace_findings.md` Q1 already established that
`TangentFaces()` means *same-domain*, not point/curve tangency; this cell confirms it on a
primitive-matrix configuration.

### 1.5 Negative results (things a port must NOT do)

1. **No CommonBlock for transverse sections** — `cb=0` in **28 of the 31 configurations**; the
   three exceptions (`box_box_touch`, `box_box_half_fuse`, `box×boxR`) all have coincident faces
   (§1.2, §1.4).
2. **No typed tangency record** for a point/curve tangency — the exact pole-tangency case has
   `fftangent=0`; the tangency is absorbed as **tolerance growth on two existing vertices**
   (1e-07 → 2.149e-06 and 1.702e-05, both keeping `sd=-1`), never by new geometry
   (`kb/occt_trace_findings.md` Q1/Q5).
3. **OCCT never lowers a tolerance and never collapses a micro-edge**: `cone_cone_p1_cut` ships
   result edges of length 1.5e-05 / 2.7e-05 / 3.6e-05 and still reports `valid=1 naked=0`.
4. **Vertex enlargement is exactly `curve_tol + 1.0e-12`** (measured twice).

---

## 2. WHAT WE DO

### 2.1 v1 — identity is a coordinate, and the reconciliation pass cannot merge vertices

**Mint-time identity.** `src/brep.cpp:3846-3860`:

```cpp
long long kx = q6(p[0]), ky = q6(p[1]), kz = q6(p[2]);   // q6 = llround(x*1e6), brep.cpp:2996
for (dx,dy,dz in -1..1) { auto it = vmap.find({kx+dx,ky+dy,kz+dz}); ...
    if (d0*d0+d1*d1+d2*d2 <= 1e-12) return it->second; }               // i.e. 1e-6 absolute
int idx = result.add_vertex(p); vmap[{kx,ky,kz}] = idx;
```

Edge identity is the analogous coordinate key `(lo_vertex, hi_vertex, q6(arc-mid))`
(`brep.cpp:4111` and `:4290-4295`, `emap`).

**The two snap layers that are supposed to make that key succeed are DEAD on the primitive path.**
`brep.cpp:3654-3676`:

```cpp
std::vector<Point> pave_pts;  double pave_cap_tol = 0.0;
if (scaf) { pave_pts = scaf->vertices; pave_cap_tol = ...; }
else if (std::getenv("SESSION_EF_PAVES")) { ... }        // OPT-IN, comment at :3667 records a regression
```

On the quadric path `scaf` is false and `SESSION_EF_PAVES` is unset, so `pave_pts` is **empty** and
every guard of the form `if (!pave_pts.empty() && !s_no_pavesnap)` short-circuits: the pave capture
(`:3722`), the original-vertex capture (`:3742`), and the **within-operand common-block pass**
(`:5016`). On the whole primitive matrix the *only* vertex-identity mechanism that runs is the
1e-6 grid at `:3846`.

**The post-hoc reconciliation merges EDGES only.** `BRep::sew_coincident_edges`
(`brep.cpp:7190`):

* tolerance `tol = diag * 5e-3` (`:7208`) — for `boxR×sph` that is **0.0435**;
* candidates are **under-mated edges only**, `trim_indices.size() < 2` (`:7221`);
* merging is capped at 2 trims per representative (`:7318-7333`);
* the rebuild (`:7568-7594`) rewrites `m_trims[ti].edge_index` and recomputes
  `m_topology_vertices[*].edge_indices`, but **`BRepEdge::start_vertex` / `end_vertex` are copied
  through unchanged**. There is no vertex-merge pass anywhere in the function, and none elsewhere
  in the boolean.

So two topology vertices 1.5e-3 apart — 29× *inside* the sew tolerance — survive as two vertices,
because the sew never looks at vertices and the mint-time grid is three decades too tight. That is
the mechanism behind every number in §4.1.

`brep.cpp:7596 sameparameter_planar_pcurves` states the downstream consequence in the source
itself: after sew "its two trims still integrate their OWN pcurve copies, which differ at the
section-fit tolerance (~2e-4): the volume error is FIRST order in that mismatch".

### 2.2 v1 — no VF stage, no EF stage

Established by `kb/hunt_efvf_gap.md §2` and re-verified: `src/intersection.h` has no
`curve_surface` / `edge_face` entry point at all. The three EF-adjacent code paths are
`SESSION_EF_DIAG` (`brep_section.cpp:2160-2247`, a real 3×3 Gauss-Newton on `C(t)=S(u,v)` that
**injects nothing**), `SESSION_EF_PAVES` (`brep.cpp:3664`, opt-in, not a solver — it filters
already-computed cut-curve endpoints), and `SESSION_EF_MARCH` (`brep_section.cpp:1832-1990`, which
*consumes* EF junctions). The function is covered by another route: section-∩-trim crossings at
`brep_section.cpp:1227-1282`, Newton-refined by `refine_trim_pave` (`:330`).

### 2.3 v1 — no common blocks in production

`src/brep_commonblock.{h,cpp}` implements a per-region chain splitter with a `ComputeToleranceOfCB`
port and a `PerformCommonBlocks` port. **Its only callers are `main_10.cpp`** (lines 488-662) —
it is not reachable from `BRep::boolean`. The one production-side analogue, the "WITHIN-OPERAND
COMMON-BLOCK PASS" at `brep.cpp:5010-5016`, is gated on `!pave_pts.empty()` and is therefore dead
on the primitive path (§2.1).

### 2.4 v2 — the arena is right; two wires are missing

**What is right.** `BdsArena` (`src/brep_bds.h`) is an index arena with exactly the OCCT shape:
`rank(i)` (`:404`), `index_of_vertex/edge/face(operand, local)` (`:413-415`),
`fuse_vertices(cluster)` (`:442`), `resolve_sd(i)` (`:450`), `add_pave(e, t, vertex, fuzz)`
(`:461`), `BdsCommonBlock` with `members` + `faces` + `set_edge()` (`:273-295`), and
`make_common_block` / `make_common_block_on_faces` (`:483,486`). `main_11` 75/75.
`v2sf::sf_split_face` mints nothing and resolves adjacency through
`sf_pave_block_id(arena, pb) = (arena edge index, pool position)` — `main_14` 62/62, canonical
`[0,1]` and padded `[-0.04,4.04]` domains give **index-for-index identical topology, 0 naked both**
(`kb/v2_splitface_notes.md`, "THE DECISIVE MEASUREMENT"), against v1's 32 naked of 36 on the
same input — the measurement `kb/port_01_bds_arena.md:1767` attributes to exactly the coordinate
identity of §2.1.

**Missing wire 1 — the interference stages are never run.** `src/v2/brep_v2_boolean.cpp:531
v2sol_run_front` does `F.ds.init(...)`, materialises pave-block pools, then goes straight to
`v2sec::V2Section S(F.ds, ops, prm)` at `:551`. `V2Interf` is never constructed. The reason given
at `:22-25` is:

> "brep_v2_interf.h and brep_v2_section.h each declare a different `V2State` in the same namespace,
> so a translation unit that pulls in both does not compile."

**That is false as stated.** `brep_v2_interf.h:76,97` declares `session_cpp::v2int::V2State`;
`brep_v2_section.h:53,68` declares `session_cpp::v2sec::V2State`. Different nested namespaces, no
collision — and `src/v2/v2_dump.h:44-45` **already includes both headers in one translation unit**
and compiles (`v2_dump.cpp`, 48 kB, in the `session_v2` target). The switchover really is the
one-line include the comment claims it would be once the (non-existent) conflict is resolved.

**Missing wire 2 — the section stage re-derives every node instead of referring to one.**
`V2Section::make_blocks` (`src/v2/brep_v2_section.cpp:1122`; that file is being edited
concurrently, so anchor on the function name, not the line):

```cpp
const int va = m_ds.append_vertex(c.c3d->point_at(d.first), c.tol);      // always NEW
...
for (interior paves) { const int v = m_ds.append_vertex(c.c3d->point_at(p.t), c.tol); ... }
```

Every pave gets a **brand-new** arena vertex, even where `V2Interf::perform_ef`
(`brep_v2_interf.cpp:1183`, stage entry points at `:945/1003/1040/1143/1183`,
driver `perform_all` at `:1334`) would already have produced one and put a pave on the pierced edge.
Reconciliation is deferred to `V2Section::post_treat_ff` (`:1295`), whose band is

```cpp
const double band = m_ds.tolerance(nodes[i]) + m_ds.tolerance(nodes[k]) + m_prm.fuzzy;   // :1313
```

≈ 3e-7 at default tolerances. Attachment of a section node to an operand edge then happens
*afterwards*, geometrically, in `brep_v2_boolean.cpp:600-690`: a 257-sample scan plus 60 bisection
steps per (pave, arena edge) pair, accepted when the distance is below
`max(tol*100, sec_lim) + egap[q]` where `lim = max(tol,1e-7)*100` (`:613`) and
`sec_lim = max(c.tol, dev)*8 + trail_chord + lim` (`:621`). Identity downstream *is* an index (`F.ds.add_pave(q, t, pv.vertex)` reuses
`pv.vertex`), but **discovery is a distance test whose band is derived from the trail's sampling
chord** — precisely the OCCT dependency that §1.2(i) removes.

Both common-block flavours are implemented in the unreached stage:
`brep_v2_interf.cpp:1068` (EE overlap → `make_common_block`) and `:1225`
(EF `Coincident` → `make_common_block_on_faces`).

---

## 3. THE GAP — the concrete implementable delta

**T1 — one arena vertex per geometric node, created by the stage that owns it.**
Order: `VV → VE → EE → VF → EF` must all complete, into the arena, *before* the first section
curve is computed. The EF stage mints the piercing vertex and inserts the pave on the pierced
edge; the FF stage then **looks the vertex up by index** (via the pave already on the boundary
edge that the section curve crosses) instead of minting one.

**T2 — wire `v2int::V2Interf` into `v2sol_run_front`.** Concretely, in
`src/v2/brep_v2_boolean.cpp`, between `F.ds.init(...)` (`:532`) and
`v2sec::V2Section S(...)` (`:551`):

```cpp
#include "brep_v2_interf.h"                 // no conflict: v2_dump.h already does this
...
v2int::V2Interf I(F.ds, ops);
I.perform_all();                            // VV VE EE VF EF, arena-mutating
```

Nothing else changes: `V2Interf` writes only into `BdsArena` (its header's contract, `:11-12`), and
`V2Section` already reads paves out of the arena.

**T3 — `V2Section::collect_paves` / `make_blocks` must consult the arena before minting**
(`brep_v2_section.cpp:980` / `:1122`).
For each candidate pave at parameter `t` on the section curve with 3D point `P`:
1. resolve the named operand edge to an arena index (`BdsArena::index_of_edge(rank, local)` — the
   call already exists at `brep_v2_boolean.cpp:660`);
2. if that edge already carries a pave whose vertex is within `tol(v) + tol(E) + fuzz` of `P`,
   **reuse that vertex index**; only otherwise `append_vertex`.
   The lookup is `O(paves on that edge)`, not a scan over all edges, because the *edge* is named by
   the trim that produced the crossing (`V2Pave::edge` / `V2Pave::face` are already populated,
   `brep_v2_section.h:194-195`).
This deletes the 257-sample projection loop at `brep_v2_boolean.cpp:623-690` for every pave that
came from a named trim, and with it the trail-chord-dependent band.

**T4 — v1, if v1 must survive at all: a vertex-level SD map.** Two sub-deltas, both small:
1. In `sew_coincident_edges`, after the edge groups are elected, run a union-find over
   `m_topology_vertices` joining any pair within `min(tol, 0.1 * shortest_incident_edge)`, then
   rewrite `start_vertex`/`end_vertex` and `m_trims`' endpoints through it. Today the function
   provably never touches vertices (`brep.cpp:7568-7594`).
2. Raise the mint-time weld at `brep.cpp:3846-3860` from its fixed 1e-6 to
   `max(1e-6, model_diag * 1e-5)` **and** key it on the *entity that produced the point* where one
   exists. The fixed grid is the reason two copies of one physical node 1.5e-3 apart are minted at
   all.
   *(This is a mitigation, not the fix. The fix is T1/T2/T3.)*

**T5 — common blocks, both flavours, in the v2 pipeline.** Already implemented
(`brep_v2_interf.cpp:1068,1225`); they arrive with T2 at zero extra cost. What is *additionally*
required for the flavour-B case to be usable is that the face splitter treat a `BdsCommonBlock`
with `faces = {F}` as a boundary contribution to `F` rather than as a section — i.e. the block's
edge must enter `F`'s arrangement input list without being intersected against `F`.

**T6 — the SD map must be walked, not merged.** `BdsArena::resolve_sd` exists (`brep_bds.h:450`);
`V2Section::post_treat_ff` already rewrites through it (`brep_v2_section.cpp:1326-1339`). What is missing is that
*every* consumer does so — `brep_v2_splitface.cpp` resolves at `:307-308,396-397,1057` but the
arena has no invariant that forbids reading `pb->pave1.vertex` unresolved. Make `BdsPave::vertex`
private behind a resolving accessor, or assert in `check_invariants` that no unresolved index
reaches an emitter.

**T7 — no measured impact yet, but it is the one OCCT behaviour we have no analogue for at all:**
tangency absorbed as tolerance growth on an existing vertex plus a re-intersection pass
(`RepeatIntersection`). v2 mints `(0,0,±2.5)` as new vertices where OCCT widens the two pole
vertices in place; measured cost 3 naked edges and +38 % volume on `sph_cyl_roty23578`, which is
*not* a matrix cell. Listed so it is not re-discovered.

---

## 4. MEASURED IMPACT

### 4.1 Node multiplicity vs OCCT — the primary table

Our result vertices vs OCCT's result vertices, matched at 5e-3. `dupnodes` = OCCT vertices that
have **more than one** of our vertices within 5e-3. `maxsplit` = largest separation inside such a
cluster. `maxposerr` = worst distance from an OCCT result vertex to our nearest copy. `orphan` =
our vertices with no OCCT counterpart within 5e-3. `EFmiss` = EF-born OCCT vertices with **no**
counterpart of ours.

**FAILING cells (all measured this task):**

| cell | our vol | OCCT vol | rel | ours/OCCT verts | dupnodes | maxsplit | maxposerr | EFmiss | orphan |
|---|---:|---:|---:|---|---:|---:|---:|---:|---:|
| `boxR × sph` cut | 38.6509 | 38.6472 | 9.45e-05 | 28 / 18 | **8** | **1.49e-03** | 7.20e-04 | 0/10 | 0 |
| `boxR × sph` common | 25.3503 | 25.3528 | 9.69e-05 | 24 / 10 | **8** | **2.74e-03** | 7.20e-04 | 0/10 | 0 |
| `boxR × sph` fuse | 104.1007 | 104.0971 | 3.51e-05 | 30 / 20 | **8** | 1.49e-03 | 7.20e-04 | 0/10 | 0 |
| `boxR × cyl` cut | 52.6395 | 52.6363 | 6.14e-05 | 18 / 14 | **4** | 8.93e-04 | 3.44e-09 | 0/6 | 0 |
| `boxR × cyl` common | 11.3608 | 11.3637 | 2.55e-04 | 10 / 6 | **4** | 8.93e-04 | 1.98e-09 | 0/6 | 0 |
| `boxR × cyl` fuse | 95.0510 | 95.0478 | 3.40e-05 | 22 / 16 | **4** | 8.93e-04 | 3.44e-09 | 0/6 | 2 |
| `cyl × cylR` cut | 20.9660 | 20.9641 | 9.39e-05 | 17 / 10 | **3** | **4.33e-03** | 3.16e-05 | 0/3 | 4 |
| `cyl × cylR` common | 21.4455 | 21.4474 | 9.18e-05 | 12 / 8 | **3** | **4.33e-03** | 3.16e-05 | 0/3 | 1 |
| `cyl × cylR` fuse **(is_solid=0)** | 59.8514 | 63.3756 | **5.56e-02** | 18 / 10 | 2 | 5.86e-05 | 3.16e-05 | 0/3 | 6 |
| `box × torR` cut | 43.6264 | 44.5656 | 2.11e-02 | 15 / 18 | 1 | 5.05e-06 | 3.61e-06 | 0/6 | 0 |
| `box × torR` common (3 vs 7 faces) | 20.3736 | 19.4344 | **4.83e-02** | 8 / 10 | 1 | 5.05e-06 | 3.61e-06 | 0/6 | 1 |
| `box × torR` fuse (12 vs 16 faces) | 68.8926 | 69.8317 | 1.34e-02 | 16 / 19 | 1 | 5.05e-06 | 3.61e-06 | 0/6 | 0 |

**PASSING controls (same instrument, same run):**

| cell | rel | ours/OCCT verts | dupnodes | maxsplit | maxposerr | orphan |
|---|---:|---|---:|---:|---:|---:|
| `box × boxR` cut / common / fuse | 7e-16 / 4e-15 / 1e-15 | 14/14, 10/10, 20/20 | **0** | 0 | 3.44e-09 | 0 |
| `cyl × cyl2` cut | 1.37e-07 | 11 / 8 | **0** | 0 | 0.00e+00 | 3 |
| `box × sph` cut | 8.52e-07 | 20 / 15 | **0** | 0 | 6.12e-16 | 5 |
| `sph × cyl` cut | 1.12e-07 | 4 / 2 | **0** | 0 | 0.00e+00 | 2 |
| `box × cyl` cut | 1.13e-14 | 12 / 10 | **0** | 0 | 0.00e+00 | 2 |
| `cone × cyl` cut | 9.11e-14 | 6 / 3 | **0** | 0 | 0.00e+00 | 3 |
| `box × box2` cut | 2.25e-15 | 17 / 16 | **0** | 0 | 0.00e+00 | 1 |

**The separation is clean: `dupnodes` is materially > 0 in 9 of the 12 failing cells I could
measure, and is 0 in all 9 passing control cells.** (`box × torR`'s `dupnodes = 1` at a 5.05e-06
split is immaterial — that cell fails for a different reason, §4.5.)

**The causal chain, spelled out on `boxR × sph`.** OCCT's 18 result vertices include 8 EF-born ones
whose tolerance is 1e-07 (two at 1.433e-07). We carry:

```
occt v2  (1.267949,-1.732051,-1.281524)  copies=2  at d=1.48e-04, 1.49e-03   EF-born
occt v3  (0.9419329,-1.167374,-2)        copies=3  at d=7.20e-04, 8.93e-04, 9.41e-04
occt v11 (-0.5400092,1.399425,-2)        copies=2  at d=3.78e-04, 7.20e-04
...
```

and in the **common** run the same nodes carry a *third* copy at 2.74e-03 / 1.25e-03 that the
**cut** run does not have. The two operations are therefore bounded by **different geometry at the
same physical nodes**, which is exactly what the partition-identity residual measures:

```
cut + common - vol(boxR):   ours 38.6509 + 25.3503 - 64 = +1.196e-03      (violated)
                            OCCT 38.6472 + 25.3528 - 64 =  0.000e+00      (holds)
boxR x cyl:                 ours 52.6395 + 11.3608 - 64 = +3.0e-04        (violated)
cyl  x cylR:                ours 20.9660 + 21.4455 - 42.4115 = 0.0000     (holds; same dup set both ops)
box  x torR:                ours 43.6264 + 20.3736 - 64 = 0.0000          (holds; shared displaced boundary)
```

`kb/hunt_oriented_primitives.md §4` reached the same conclusion from the volumes alone
("`boxR × sph` … partition identity **violated by +1.196e-03** … an inconsistent trim, i.e. a
genuine construction error"). This document supplies the entity-level cause: **8 nodes represented
2–3 times, with a different multiplicity per operation.**

**`boxR × cyl` is the counter-example that stops over-claiming.** Its `maxposerr` is **3.4e-9** and
every one of its 6 EF nodes is exact, yet all three ops fail (up to 2.55e-04). Duplication is
present (4 nodes, split 8.93e-04) but the *positions* are exact — so a material part of that cell's
error is **not** a topology-layer defect. Attribute the remainder to the pcurve layer (the
degree-1 boundary pcurve finding in the brief), not here.

**`coneR × cyl` — OCCT reference now on record, our side not measurable on the 08:21 binary:**

```
coneR_cyl_cut     EF=1 VF=0 EE=0 FF=1 seccurves=1 secpb=1  res_face=3 res_edge=3 res_vert=2  vol=12.307399
coneR_cyl_common  same DS                                   res_face=2 res_edge=3 res_vert=2  vol=4.44774457
coneR_cyl_fuse    same DS                                   res_face=5 res_edge=6 res_vert=4  vol=54.7193869
RESVERT i=1 p=1.10641839,-1.01283678,0  tol=4.00817023e-07
RESVERT i=2 p=2,-2.8,0                  tol=1e-07
```

**Two result vertices.** Whatever produces our 43.1452-vs-12.3074, it is not a node-count problem
of the kind in the table above; it is the marcher (the cell has no analytic dispatcher arm) losing
the section entirely. Topology layer: **no measured impact on this cell.**

### 4.2 v2's identity defect, measured by another agent's harness

`kb/v2_diff_harness.md §3.1`, reproduced here because it is the *same* defect three decades
smaller and it is the only place where a v2 identity failure has a number attached:

| v2 case | signature | consequence |
|---|---|---|
| `cyl_cyl_cut` | `ds.newvertices 2 vs 4`: OCCT names the seam crossings at `(1,±1.11803,0)`; v2 names those **and** a second pair at `(1,±1.11803,-2.35e-06)`, **2.2–2.4e-6 apart**, while the trace's own `TOL … tol=1.17046999e-05` line puts the arena's fusion tolerance at **5× the gap** | 3 faces vs 4, **2 naked**, volume 53.194 vs 47.682 (**+12 %**), `closure_residual = 7.82e-09` |
| `box_cone_p2_cut` | `sec.block_nodes 12/12` exact (worst 6e-8) — the section is right — then **8 spurious new arena vertices** (14 vs 6), `ds.split_edges 2→6` | 12 faces vs 8, 28 verts vs 14, `closure_residual = 6.8e-2`, +13 % |
| `cone_cone_p2_cut` | 17 new vertices vs 7, `ds.split_paves 15 vs 8` | 11 shells, **21 naked**, +349 % |
| `sph_cyl_roty23578` | v2 mints `(0,0,±2.5)` as **new** vertices where OCCT inflates the two existing pole vertices in place | 3 naked, 2 shells, +38 % |

Across the cone family v2 ends with **2.3× / 1.2× / 2.4×** OCCT's new-vertex count *while the
section blocks are exact or close*. This is T1/T3 with the section stage exonerated.

### 4.3 The missing VF and EF stages, quantified honestly

* **EF as a source of missing points: zero measured impact.** `EFmiss = 0` in **12 of 12**
  measured failing cells (25 EF-born vertices in total across them); every EF-born OCCT vertex has
  one of ours within 7.2e-4 (and within 3.4e-9 on `boxR×cyl`). Consistent with
  `kb/hunt_efvf_gap.md`'s 33/34 and 42/42 on the two chair configs; this task extends that result
  to the primitive failures the brief points at.
* **VF: no measured impact anywhere in the matrix.** `VF > 0` in 2 of the 31 traced
  configurations — `sph_cyl_roty23578` (VF=1, not a matrix cell) and `box×boxR` (VF=6, and that
  cell passes to 1e-15 in all three ops). Any campaign sized around a VF stage would be
  misallocated.
* **What EF is actually worth is T1, not coverage:** it is the stage that makes the node exist
  *once, before the sections*, so that both face pairs pave at the same index (§1.2(i)). The 9
  duplicated-node cells in §4.1 are its bill.

### 4.4 Common blocks

**No measured impact on the 63-cell primitive matrix.** The single cell that exercises them is
`box × boxR`, which OCCT solves with `cb=10` (all flavour B), `VF=6`, `EE=4`, `EF=10` and two
same-domain FF pairs — and which **our kernel already solves exactly (7.5e-16 / 4.0e-15 /
9.8e-16, faces 9/7/13 matching, `is_solid=1`)** with none of that machinery, because every surface
involved is planar and our planar path is exact.

Where it *will* be measured: the mated-face chair pair (`kb/hunt_efvf_gap.md §6`: "the EDGE-type
common part … we have no analog at all … it is the mated-face case, which is exactly the
*unrotated* chair pair"), and OCCT-authored coincident inputs — `kb/V2_STATUS.md §2`:
OCCT-authored `box × box` under v2 gives **0.333333 / 21.6667 / 64.3333, all open**, against
43 / 21 / 107 exact for v1.

### 4.5 What is NOT a topology-layer gap (stated so effort is not misallocated)

`box × torR` (3 cells, 1.34e-02 … 4.83e-02 — with `cyl×cylR fuse` at 5.56e-02, the largest errors
among the cells I could measure)
is missing **4 of OCCT's 18 result vertices entirely**, at distances 1.02–1.33 from anything we
have:

```
MISSING (-1.090167, -2,        -1.541914)   dmin=1.019e+00
MISSING ( 1.090167,  1.541914,  2       )   dmin=1.019e+00
MISSING (-0.9165664,-0.9340764,-2       )   dmin=1.325e+00
MISSING ( 0.9165664, 2,         0.9340764)  dmin=1.325e+00
```

They carry OCCT tolerance `1.000001e-06` (marched, not analytic) and **they are not in the EF
vertex set** (`EFmiss = 0` for this cell — the 6 EF nodes are all present, worst 3.6e-06). They are
FF section nodes on the plane×torus sections. `common` also emits 3 faces against OCCT's 7. This is
a section/marcher coverage failure, not an identity failure, and belongs to that layer's gap
document.

---

## 5. IMPLEMENTATION ORDER — smallest shippable increment first

Each step is independently landable and has an acceptance test that fails today.

### Step 1 — wire `V2Interf` into `v2sol_run_front` (≈ 5 lines)

`#include "brep_v2_interf.h"` in `src/v2/brep_v2_boolean.cpp`; construct `v2int::V2Interf I(F.ds, ops)`
and call `I.perform_all()` between `:532` and `:551`. Delete the false comment at `:22-25`.

*Acceptance:* `main_20 --op cut --a cylinder,r=1.5,h=6,tz=-3 --b cylinder,r=1.5,h=6,roty=90,tx=-3`
reports `ds.newvertices 2` (today 4) and no vertex pair closer than 1e-5 in the arena dump; the
`cyl_cyl_cut` row of the v2 divergence table moves from `FIRST_DIVERGENCE ds.vertices 6/8` to
`NONE` at that stage. Regression gate: `main_11` 75/75, `main_13` 76/76, `main_15` 29/29 unchanged.

*Risk:* the stage is arena-mutating and previously ran only against a probe arena. Gate it behind
`SESSION_V2_INTERF` for one cycle so both ladders are measurable side by side.

### Step 2 — `V2Section::collect_paves` reuses an existing pave's vertex (T3)

Before `append_vertex` in `V2Section::make_blocks` (`brep_v2_section.cpp:1122`), resolve `V2Pave::edge`
through `BdsArena::index_of_edge(rank(face), edge)` and search that edge's existing paves for one
within `tol(v)+tol(E)+fuzz` of the 3D point; reuse its vertex index on a hit.

*Acceptance:* on `cyl × cylR`, the arena contains **one** vertex within 1e-6 of
`(-0.585786438, ±1.38088893, -2.21421356)` and one within 1e-6 of `(1.5,0,2.82132034)` — i.e. the
three EF nodes of that cell are single-sourced. Oracle-free companion: rerun with
`SESSION_V2_TRAIL_N=192` and `=768`; the arena's vertex **count** must be identical (today the
attachment band contains the trail chord, so it is not).

### Step 3 — make the SD map non-bypassable (T6)

Add to `V2Interf::check_invariants` / `BdsArena` a check that no `BdsPave::vertex` reaching an
emitter has `resolve_sd(v) != v`. Prefer making the raw field inaccessible.

*Acceptance:* a new `main_11` cell that fuses two vertices and then asserts every pave block's
`pave1/pave2.vertex` is already canonical; it must fail before the change.

### Step 4 — flavour-B common blocks consumed by the splitter (T5)

With Step 1 landed, `make_common_block_on_faces` (`brep_v2_interf.cpp:1225`) will start producing
blocks. Teach `v2sf::SfEmitter` / the per-face input assembly in `brep_v2_boolean.cpp:697+` to add
a common block whose `faces` contains `F` to `F`'s arrangement input as a boundary, without
sectioning it.

*Acceptance:* `main_20 --op cut --a box,dx=4,dy=4,dz=4,center --b box,dx=4,dy=4,dz=4,center,rotz=30,tx=2,ty=1`
produces 10 arena common blocks, matching the trace's `cb=10`, and the result stays exact
(37.939310229206 /
26.060689770794 / 101.939310229206) — this cell is a **no-regression** gate today, so the step is
provably safe before it is provably useful.

### Step 5 — v1 vertex-level SD map (T4), only if v1 must ship

Union-find over `m_topology_vertices` at the end of `sew_coincident_edges`, joining pairs within
`min(tol, 0.1 × shortest incident edge)`, rewriting `start_vertex`/`end_vertex` and trim endpoints.

*Acceptance:* `boxR × sph` cut/common go from 28/24 result vertices to 18/10, `dupnodes` to 0, and
the partition residual `cut+common−64` from **+1.196e-03** to below 1e-6. Regression gate: the 9
passing control cells of §4.1 keep `dupnodes = 0` and their volumes to the digits shown; chairs
`z30x20` cut stays faces 34 / naked 0.

### Step 6 — tolerance-growth tangency + one re-intersection pass (T7)

Only after 1–5. `UpdateVertex`-style growth-only tolerance on any weld/adoption at distance `d`,
plus a single `RepeatIntersection` pass when a vertex grew.

*Acceptance:* `sph r=2.5 × cyl r=1 h=8 centre, roty=23.578178478201835` yields 3 section curves,
`res_degen=3`, `naked=0`, `common=15.0617954` to 1e-7 — and the two pole vertices are **widened in
place**, not replaced (`sd=-1`), matching the trace exactly.

---

## 6. Reproduction

```bash
# OCCT ground truth for a matrix cell (tracer already built)
cd validation/occt_trace
./build/occt_trace --op cut \
  --a box,dx=4,dy=4,dz=4,center,rotz=30,tx=2,ty=1 --b sphere,r=2.5 \
  --name boxR_sph_cut --out boxR_sph_cut.trace
grep -E '^(SUMMARY|RESVERT|IEF|SD |FI |FIPB|SECPB)' boxR_sph_cut.trace

# our side
cd session_cpp
SESSION_STEP_DIR=/tmp/steps ./build/main_7 "boxR x sph"

# node-multiplicity comparison (scripts written for this task, scratchpad only)
python3 table.py /tmp/steps/boxR_cut_sph.step boxR_sph_cut.trace
```

The three scripts (`vx.py` result-vertex extraction from the third `MANIFOLD_SOLID_BREP`,
`cmp.py` OCCT `RESVERT`/`IEF`→`DSVERT` extraction, `table.py` the matcher) are ~90 lines total and
are the whole instrument; nothing under `src/`, `main_*.cpp`, `validation/` or `corpus/` was
modified by this task.
