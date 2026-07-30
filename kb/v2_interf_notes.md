# v2 interference stages — integration notes for consumers

`src/v2/brep_v2_interf.h` / `.cpp` implement OCCT PaveFiller stages **VV, VE, EE, VF, EF**
(`kb/port_02_vv_ve_ee.md`, `kb/port_03_vf_ef.md`). Everything lives in **`session_cpp::v2int`**.
Test driver `main_15.cpp`, **29/29 PASS** on an actual run; output byte-identical across runs.

```bash
cmake -S session_cpp -B session_cpp/build_h -DCMAKE_BUILD_TYPE=Release
cmake --build session_cpp/build_h -j 12 && ./session_cpp/build_h/main_15
```

Only shared-file edit: `15` appended to `foreach(MAIN_ID ...)` in `CMakeLists.txt`. The module
reads BReps, writes only into a `BdsArena` (`src/brep_bds.h`, 75/75, unmodified) and mutates no
BRep. `src/brep_massprops.*` and `src/brep_bds.*` are used, never touched.

---

## 0. The verdict metric, validated BEFORE anything else was measured

`main_15` cell 0 runs first and refuses to print another number if it fails. Measured:

| shape | `v2_naked_edges` | non-manifold | degenerate | `is_solid()` | naive `!=2 trims` |
|---|---|---|---|---|---|
| sphere (2 poles) | 0 | 0 | 0 | true | 0 |
| cone (apex) | 0 | 0 | 0 | true | 0 |
| cylinder (seam) | 0 | 0 | 0 | true | 0 |
| sphere + planted zero-length 1-trim edge | **0** | 0 | **1** | true | **1** |
| box with one trim removed | **1** | 0 | 0 | false | 1 |

Row 4 is the trap that produced five measurement errors this session: a zero-length degenerate
edge carrying one trim. `BRep::is_solid()` excludes it (`brep.cpp:1085-1102`), the naive metric
calls it naked, `v2_naked_edges()` agrees with `is_solid()`. Row 5 proves the metric still catches
a real hole. In this kernel the primitives' poles/apexes are *singular trims with
`edge_index == -1`*, so they contribute no edge record at all — which is why the trap had to be
planted explicitly rather than found on a stock primitive.

`src/v2/v2_verdict.h` (`v2v::v2_verdict`, validated by `main_17`) is the project-wide harness and
is what a *boolean result* should be scored with; `v2_naked_edges` is the local topology-only
counter this driver uses for its own cell-0 self-check and agrees with it on these shapes.

---

## 1. What exists

| entity | role | OCCT origin |
|---|---|---|
| `V2FaceView` | surface + adaptor UV rectangle + sampled trim loops + cached eval grid | `BRepAdaptor_Surface` + `IntTools_FClass2d` |
| `v2_proj_pc` | point→curve projection, **interior stationary points only, never clamped** | `GeomAPI_ProjectPointOnCurve` / `Extrema_GGExtPC` |
| `v2_proj_ps` | point→surface projection, UV-restricted, grid-seeded Newton, seam retry | `IntTools_Context::ProjPS` |
| `v2_compute_vv/ve/vf` | the three stage predicates, verbatim including the return codes | `BOPTools_AlgoTools::ComputeVV`, `IntTools_Context::ComputeVE/ComputeVF` |
| `v2_edge_edge` | EE: crossings → `V2EEPart(Vertex)`, overlaps → `V2EEPart(Edge)` | `IntTools_EdgeEdge` |
| `v2_edge_face` | EF: `Transversal` / `Tangential` / `Coincident` typed parts | `IntTools_EdgeFace` + `IntTools_BeanFaceIntersector` |
| `V2Interf` | the five-stage driver over a `BdsArena` | `BOPAlgo_PaveFiller` stages 1–5 |
| `v2_naked_edges` / `v2_closed` | the local verdict counter | `BRepCheck` |

Stage order is enforced by `perform_all()`; each `perform_xx()` is separately callable so a test
can run a prefix. Between stages the arena's pave blocks are rebuilt
(`BdsArena::update_pave_blocks()`), which is what makes stage N+1 see stage N's paves.

---

## 2. What each stage writes into the arena

| stage | writes |
|---|---|
| VV | `fuse_vertices(cluster)` once per connected component; every member redirected through the SD map; `BdsInterfType::VV` records; every edge's paves re-resolved through SD |
| VE | `add_pave(edge, t, vertexSD, fuzz)`; `absorb_tolerance(v, dist + tol(E))`; `VE` records carrying `ta` |
| EE | new arena vertices (fused first, `bds_bounding_vertex`), a pave on **both** edges, `make_common_block({pb1,pb2})` for overlaps, `EE` records carrying `new_vertex`, `ta`, `tb` |
| VF | `face_info(f).v_in` entries, `absorb_tolerance(v, dist + tol(F))`, `VF` records carrying `(u,v)` |
| EF | new arena vertices at piercings (fused across faces first), a pave on the edge, `face_info(f).v_in` entries, `make_common_block_on_faces(pb,{f})` + `pb_in` for coincidence, `EF` records |

Consumer-facing records: `V2Interf::ef_records()`, `vf_records()`, `ee_records()`.
`V2EFRecord` carries `edge`, `face`, `kind`, `type`, `t`, `t0/t1`, `u/v`, `new_vertex`,
`on_pave`. **`on_pave == true` means no vertex was minted**: the block's existing end vertex was
made known to the face instead (OCCT `ForceInterfVF`, `PF5:447-457`). A splitter must not look for
a new node there.

`V2Interf::check_invariants()` is the cheap oracle-free sweep: pave tiling law, exact pave-block
cover, pave containment, and "every EF vertex has a pave on its edge". Call it at the end of a
corpus cell.

---

## 3. Measured results (`./build_h/main_15`, 29/29)

**EF completeness against INDEPENDENT oracles** — the oracle never calls the production
projector:

| pair | oracle | poses | true piercings | vertices found | edges split | missing |
|---|---|---|---|---|---|---|
| box edge × sphere face | analytic line/sphere quadratic | 20 | **297** | **297** | **297** | **0** |
| sphere seam × box face | plane equations in the box's own frame | 20 | **22** | **22** | — | **0** |
| cone × cone lateral | implicit `sqrt(x²+y²) − r(1−z/h)` sign change | 20 | **75** | **75** | **75** | **0** |
| cylinder × cylinder lateral | implicit `sqrt(x²+y²) − r` sign change | 20 | **98** | **98** | **98** | **0** |

`exactly_one_vertex_per_piercing`: 297/297 box×sphere piercings have **exactly one** arena vertex
within 1e-6 — the A-side and B-side of a piercing fuse (G2), they never become two vertices a hair
apart.

**Rigid-motion invariance of the interference count** (same motion applied to both operands,
20 random motions each):

| pair | invariant | base interferences | EF parts |
|---|---|---|---|
| box × sphere | 20/20 | 12 | 24 |
| cone × cone (tip-to-base) | 20/20 | 5 | 6 |
| cylinder × cylinder (equal radii, perpendicular intersecting axes) | 20/20 | 13 | 15 |

The third row is an **exact double tangency** at `(0,±1,1)` — the hardest cylinder configuration
there is. It needed the tangency-recovery rule of §4.3 to become invariant; before it the EF part
count oscillated 14/15/16 across motions.

**Other cells**: 120-permutation bitwise order-independence of `bds_bounding_vertex` on
mixed-magnitude input; exact two-ball smallest enclosing sphere (both balls internally tangent to
1e-16); `fuse_vertices` idempotent under permutation *and* duplicate members; VV stage idempotent
(second run: 0 new fusions, byte-identical arena signature); 16 edge crossings → 16 vertices and
32 paves on the 45°-rotated box pair; 4 vertex-on-face incidences into the face IN set; 4 common
blocks on a flush face-to-face contact; tangential/transversal/miss typing on a cylinder.

---

## 4. Three things a consumer must know

### 4.1 Trim pcurves are stored in loop-traversal order; `BRepTrim::reversed` is about the EDGE

`brep.cpp:388-397` stores the cylinder's top-circle pcurve as `(u1,v1) → (u0,v1)` **and** sets
`reversed = true`. Honouring the flag when sampling the loop turns the trim rectangle into a
bow-tie and **inverts the even-odd parity over part of the domain**. Measured cost of getting this
wrong: 17 of 297 box-edge/sphere piercings silently classified `Out` and dropped, plus the whole
cylinder tangency cell returning zero parts. `v2_make_face_view` therefore chains trims
**by geometry** — each pcurve is appended in whichever direction continues from the current chain
end — and never reads `reversed`. Any other code that samples trim loops in UV should do the same.

### 4.2 A closed direction needs a seam retry in the projector

On a surface closed in `u`, the rectangle's two `u`-edges are the same 3D curve. A Newton step
that wants to leave through one of them is clamped, and the returned distance is not the distance.
Measured: 1 of 297 piercings lost, at a foot 0.5° on the far side of the sphere's seam
(`d` came back `1.35e-2` instead of `0`). `v2_proj_ps` re-runs Newton from the opposite edge
whenever the converged foot sits on a boundary of a closed direction. `V2FaceView` caches
`closed_u` / `closed_v`.

### 4.3 The EF typing rules, and where they are deliberately not OCCT

`v2_edge_face` decides the type from the **signed normal residual**
`s(t) = (C(t) − S(u*,v*)) · N(u*,v*)`, which is exactly ±dist at a genuine projection foot:

- **Transversal** — `s` changes sign. Bracketable and pose-independent; a distance threshold is
  not. (Divergence D2 from `IntTools_BeanFaceIntersector`'s marched tolerance tube.)
- **Tangential** — an interior minimum of `d` below `criteria` with no sign change, refined by
  golden section. The refinement matters: a line tangent to a unit cylinder sampled at 96 points
  has a minimum sampled distance of ~5e-5, four orders above `criteria`, so a "sample below
  criteria" test finds nothing.
- **Coincident** (`type == Edge`) — a run of `d ≤ criteria` covering **essentially the whole
  working range** (OCCT `isWholeRange`, `IEF:326-347`) whose 3D extent exceeds `2·criteria`. This
  is why OCCT works per pave block: a coincident piece is a whole block once VE/EE have split the
  edge at the ends of the contact. Dropping `isWholeRange` and promoting any long-enough run turns
  a tangency widened by a large fuzz into a false coincidence — at `fuzz = 1e-3` a line tangent to
  a unit cylinder is within criteria over a 0.09-long run that is not coincidence at all
  (`kb/port_03` acceptance test I3d).
- **End contact** — the block's own end sitting on the face is decided **by distance, never by a
  sign change**. At a block end the crossing coincides with the sample, so whether `s` flips is
  settled at the 1e-16 level; measured, cone × cone lost 2 of its 5 interferences under 2 of 8
  rigid motions before this branch existed. Its type comes from the angle between the curve
  tangent and the surface normal (5° from perpendicular). The driver routes it to `ForceInterfVF`:
  the existing end vertex becomes known to the face, `on_pave` is set, **no vertex is minted**.
- **Tangency recovery** — a tangency is a double root of `s`; any perturbation splits it into two
  simple roots a few 1e-8 apart, and the same contact then reports as two piercings. Two crossings
  within two sample steps merge back into one `Tangential` part when the curve never leaves the
  tolerance band between them (a real chord reaches a real depth between its ends; a tangency does
  not). This is what makes the equal-radius cross-cylinder invariant.

`vertex_for_tangential` (default true) keeps OCCT's behaviour of minting a vertex for a tangential
touch — the arrangement still needs the node — but the record carries `kind == Tangential`, so no
consumer can mistake it for a piercing.

---

## 5. Known gaps

1. **`RepeatIntersection` / `ForceInterfEE` / `ForceInterfEF` are not implemented**
   (`kb/port_02` §2.7 last row, `kb/port_03` §2.6.5). Contacts that only become reachable *after*
   tolerances have grown are not recovered. Nothing in the current corpus needs them; a coincident
   pair whose bounding vertices are not yet shared is recorded as an interference but produces no
   common block, exactly as OCCT does before the second pass.
2. **No shrunk range / splittability oracle** (`IntTools_ShrunkRange`). Micro-edge suppression
   (`kb/port_02` T10) is therefore not enforced: a pave block shorter than its vertices' tolerance
   spheres is still intersected. `BdsPaveBlock` already carries `ts1/ts2/splittable`, so adding it
   is not a structural change.
3. **`TreatVerticesEE`** (the second VF pass over EE-born vertices, `PF4:305-390`) is absent.
   `candidates()` only pairs *source* shapes, so a vertex created by EE is never tested against a
   face. In practice such a vertex lies on an edge of both operands, so the strict-IN rule would
   reject it anyway, and the EF end-contact branch covers the case that matters — but a face whose
   interior is pierced only by an EE vertex would be missed.
4. **Candidate generation is O(n²)** over arena boxes, not a BVH. Correct and deterministic
   (stable-sorted), but it will not scale past a few thousand entities.
5. **EE overlaps only become common blocks when the two pave blocks already share both bounding
   vertices** (`has_same_bounds`, OCCT `PF_3.cxx:535-540`). That is the documented OCCT gate and
   the reason VE must be total before EE.
6. **Per-entity tolerances are assumed, not measured** — inherited from `BdsArena`
   (`kb/bds_integration_notes.md` §4.1). Everything starts at `V2_CONFUSION` and grows through
   `absorb_tolerance`, which records the growth.

---

## 6. Wiring example

```cpp
#include "src/v2/brep_v2_interf.h"
using namespace session_cpp;
using namespace session_cpp::v2int;

BdsArena ds;
ds.init({&A, &B}, fuzz);
V2Interf iv(ds, {&A, &B});
iv.perform_all();                       // VV -> VE -> EE -> VF -> EF, in that order

std::string why;
if (!iv.check_invariants(&why)) { /* report why */ }

for (const V2EFRecord& r : iv.ef_records()) {
    if (r.type == V2PartType::Edge) {           // the edge LIES ON the face
        // ds.face_info(r.face).pb_in already holds the pave block
        continue;
    }
    if (r.on_pave) continue;                    // no new node: an existing end vertex was used
    if (r.kind == V2EFKind::Tangential) { /* contact, not a crossing */ }
    // r.new_vertex is an arena index; ds.paves(r.edge, ...) already contains it
}
```

Split images are **not** materialised here: `BdsPaveBlock::edge` stays `-1`. The splitter calls
`BdsArena::update_pave_blocks()` (already done at the end of `perform_all()`) and then names one
edge per block / per common block via `BdsCommonBlock::set_edge()`.
