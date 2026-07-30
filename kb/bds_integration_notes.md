# BDS arena — integration notes for session A

`src/brep_bds.h` / `src/brep_bds.cpp` are NEW and self-contained: nothing in the existing kernel
calls them, and they mutate nothing. Test driver `main_11.cpp`, **75/75 PASS** on an actual run
(`cmake -S session_cpp -B session_cpp/build_d -DCMAKE_BUILD_TYPE=Release && ./build_d/main_11`);
output byte-identical across 20 consecutive runs.

The only shared-file edit was appending `11` to `foreach(MAIN_ID ...)` at `CMakeLists.txt:258`.
`src/*.cpp` is already GLOBbed into `session_core` (`CMakeLists.txt:221`), so **re-run `cmake -S . -B <builddir>`
once** after pulling — the GLOB is evaluated at configure time and a stale cache will fail to link
`BdsArena::append_vertex`.

Spec anchors: `kb/ARCHITECTURE_v2.md` §1 (the design), `kb/port_01_bds_arena.md` (the full port with
`file:line` anchors, guarantees G1–G12 and acceptance tests T1–T14),
`kb/audit_occt_pavefiller-core.md` (audited corrections E1–E6, O1).

---

## 1. What exists now

| structure | role | OCCT source |
|---|---|---|
| `BdsShape` | one arena slot: type, operand, source index, subs, model-space tol, box | `BOPDS_ShapeInfo` |
| `BdsVertex` | fused vertex geometry: point, tol, provenance | `TopoDS_Vertex` + `BRepLib::BoundingVertex` |
| `BdsPave` | a vertex sitting at a parameter on an edge | `BOPDS_Pave` |
| `BdsPaveBlock` | one INTERVAL of an edge; ext-paves; shrunk range | `BOPDS_PaveBlock` |
| `BdsCommonBlock` | coincident pieces as ONE object; faces; measured tol | `BOPDS_CommonBlock` |
| `BdsFaceInfo` | per-face In / On / Sc (there is **no** Out set) | `BOPDS_FaceInfo` |
| `BdsInterf` | typed interference record VV/VE/EE/VF/EF/FF | `BOPDS_Interf` |
| `BdsArena` | the flat arena, pools and maps | `BOPDS_DS` |

Free functions: `bds_bounding_vertex`, `bds_uv_tolerance`, `bds_t_tolerance`,
`bds_curve_fingerprint`, `bds_pair_type`, `bds_split_interval`, `bds_check_coincidence`.

**Not implemented here** (deliberately out of this deliverable, see `port_01` §6 steps 3/3b):
`BdsIterator` / `BdsSubIterator` (the BVH candidate-pair generator), `BdsFFCurve` / `BdsFFPoint`
(section-curve slots), `release_pave_blocks`, `update_common_block`, shrunk-data computation,
`sub_shapes_on_in` / `shared_edges` / `alone_vertices`, and `face_info_in` (which only becomes
meaningful once VF/EF exist). `BdsPaveBlock` already carries the `ts1/ts2/shrunk_box/splittable`
fields so adding shrunk data later is not a structural change.

---

## 2. Call sites session A should use

### 2.1 Replace `build_shared_edge_pool` (`brep_section.cpp:2583`, declared `brep_section.h:66`)

`SharedEdgePool` is `BdsArena` restricted to (created vertices, created section edges) with the
pave-block layer flattened away. The absorption is mechanical:

| `SharedEdgePool` | `BdsArena` |
|---|---|
| `arena.m_vertices` + `m_topology_vertices` | `int v = bds.append_vertex(p, tol)` — the returned index IS the identity |
| `arena.m_curves_3d` + `m_topology_edges` | `int e = bds.append_edge(c3d, v0, v1, tol)` |
| `vert_tv[scaffold_vertex]` | store the `append_vertex` result directly on the scaffold vertex |
| `seg_edge[seg]` | `bds.pave_blocks(e)[0]->edge` after materialisation |
| `block_edge[{seg,k}]` | `bds.pave_blocks(e)[k]` — the key disappears, a pave block *is* the block |

`append_edge` inserts the two bounding paves itself, so an edge always has a valid pave-block pool
the moment it exists. Do not construct `BdsPaveBlock`s by hand for source edges.

### 2.2 Replace `vmap` / `emap` in `split_with` (`brep.cpp:3478-3479`, lookups `:3745-3761`, `:4013-4024`)

Those two maps are the coordinate-coincidence identity path (quantised to 1e-6, preceded by the
`pave_tv` and `orig_tv` snaps). Their replacement is:

```cpp
int v = bds.index_of_vertex(operand, local_topology_vertex);   // table read, not a search
int e = bds.index_of_edge  (operand, local_topology_edge);
int f = bds.index_of_face  (operand, local_face);
```

There is intentionally **no** `find_vertex_near(Point)` and no coordinate key anywhere in
`brep_bds.h`. If a call site needs one, the identity was lost upstream and the fix belongs there.
`bds_check_coincidence(ds, pbA, pbB, fuzz)` answers "are these two blocks, which you already
named, one reality?" — it returns `bool`, never an entity, so it cannot be used to mint identity.

Do not attempt this before 2.1 lands; removing the coordinate weld without a shared arena behind
it would take box×box from 15/20 to 0/20 (`port_01` §6, step 9 dependency note).

### 2.3 Replace the domain-relative tolerances (`brep.cpp:4350` `eps_border`, `:4280` `scaf_forced_eps`, `:4271` `ov`)

Each of those multiplies a UV **domain extent** by a constant, so a STEP round-trip that returns a
box face on `u[-0.04, 4.04]` instead of `u[0,1]` inflates all three 4×. The replacement pattern is:

```cpp
double du, dv;
if (!bds_uv_tolerance(surface, u, v, tol3d, du, dv)) { /* pole: handle by name */ }
// use du/dv HERE, at the point of use; never store them
```

`main_11` asserts the invariance directly: the same geometric plane presented on `[0,1]²` and on
`[0,2]²` yields UV tolerances differing by exactly 2×, and `du * |dS/du| == tol3d` in both.

### 2.4 Where the current welds go

`run_xweld` (`brep.cpp:10364`), `sew_coincident_edges` (`:10951`), `imprint_edges` (`:5038`),
`co_refine_coincident_edges` (`:5388`) are repair work for identity destroyed in the splitter.
Once 2.1 + 2.2 land, the acceptance test is `SESSION_NO_SEW=1` producing a byte-identical result
to the default run on box×box. Instrument the four with counters first so "zero weld calls" is a
measured claim rather than an assumption.

---

## 3. Typical stage wiring

```cpp
BdsArena bds;
bds.init({&A, &B}, fuzz);            // stage 0: flatten; ranges; boxes; sub-shape lists
// stage 1 VV: cluster coincident vertices, then ONE call per cluster:
int nv = bds.fuse_vertices(cluster); // idempotent + order independent; members redirect via SD
// stage 2 VE: a vertex found on an edge becomes a pave
auto st = bds.add_pave(edge, t, nv, fuzz);
// stage 3 EE: coincident intervals become ONE object
BdsCB cb = bds.make_common_block({pbA, pbB});   // tolerance is computed inside
// stage 5 EF: a block lying on faces
bds.make_common_block_on_faces(pb, {f1, f2});
// stage 9: materialise the interval views exactly once
bds.update_pave_blocks();
cb->set_edge(new_edge_index);        // ONE edge index for every member, in one call
```

Rules that matter at the call sites:

- `add_pave` returns a status. `CoincidentVertex` means two DISTINCT vertices occupy the same
  model-space location: fuse them first, then re-add. Silently accepting both is what produces
  zero-length blocks and divergent copies downstream.
- `update_pave_blocks()` is idempotent — safe to call after every stage. It rebuilds the interval
  view; it never touches edge geometry (`bds_curve_fingerprint` before/after is the tripwire).
- `make_common_block` refuses groups smaller than two (OCCT does the same); use
  `make_common_block_on_faces` for the block-lies-on-face case.
- The common-block representative is the minimal original-edge index **unless**
  `set_representative` was called; that call sets `representative_forced`, which is the audited
  E3 quirk made visible instead of silent.
- `check_invariants(&why)` is cheap; call it at the end of a corpus cell under the v2 gate.
- `signature()` is a canonical, run-to-run reproducible dump. Use it for golden regression files.

---

## 4. Known gaps to close before this can carry a boolean

1. **Per-entity tolerances are assumed, not measured.** `BRep` has no tolerance field
   (`brep.h:27-58`), so `init()` seeds every shape with `BDS_CONFUSION` and reports that through
   `tolerances_are_default()`. Real tolerances must come from the reader/builder.
2. **No candidate-pair iterator.** Stages currently have to supply their own pairs.
3. **Edge boxes are control-hull bounds**, not tight curve bounds — conservative, so no missed
   pairs, but looser than `BRepBndLib`.
4. **`face_info_in` is not derived yet**; `pb_in` / `v_in` are writable but nothing fills them
   until VF/EF exist. `refine_face_info_in` already works on hand-filled sets.
5. **Shells are not modelled**; each operand contributes one `Solid` slot whose subs are its faces.
6. **`BdsCommonBlock::orientation` is a recorded slot, never written here.** The same/opposite
   detector is session B's (`brep_samedomain`); whoever builds the block should set it.
