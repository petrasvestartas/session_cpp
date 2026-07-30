# hunt_trimmed_splitter — why the kernel is exact on untrimmed primitives and fails on trimmed multi-face solids

Audit of the TRIMMED-FACE SPLITTER. Read-only on source; no code changed, nothing committed.

**Snapshot audited:** `5bb685aac3ff8f6a4922aaa2c34931702fac0004` (HEAD at audit time). All `file:line`
references below are line numbers in `git show HEAD:src/<file>`, copied to
`/home/petras/split_audit/{brep_HEAD.cpp, brep_section_HEAD.cpp, nst_HEAD.cpp, intersection_HEAD.cpp,
brep_HEAD.h, brep_section_HEAD.h}`. The working tree is being edited concurrently (session A) and its
line numbers drift by ~200 in `brep.cpp`; use the snapshot, not the working copy.

**Independent measurements** in this document were taken from the exported STEP results in
`/home/petras/fc_inspect/verify_now/`, the canonical operands in `/home/petras/fc_inspect/operands/`,
and the OCCT reference answers in `/home/petras/fc_inspect/REFERENCE/`. Two routes were used: direct
STEP entity-graph parsing (no kernel), and FreeCAD 1.1.1 / OCCT 7.8.0 via
`/snap/bin/freecad.cmd <script>`. Scripts: `/home/petras/split_audit/{verify_ref, chunk_connect,
verify_y30_face, patch_area}.py`.

> **Provenance, and two retractions.** An earlier revision of this document carried two numbers that
> came from a delegated measurement and that I published without re-deriving. **Both were wrong and
> both are retracted where they appear:**
>
> | retracted claim | truth, re-measured directly |
> |---|---|
> | x13y29 `A.cut(B)` recomputes to 79.732960, so "the reference is not a fixed point" | **48.473392** vs stored 48.472832 — the references are stable to four decimals (Appendix A) |
> | the doubled y30 face has "flux exactly 0.000000, so it bounds nothing" | flux **73.257183** vs the reference counterpart's 73.257742 — nearly identical. The duplication shows in **area**, not flux (§2.4, §4.4) |
>
> Conclusions built on them have been re-derived rather than merely patched: §2.2 now rests on
> operand-only booleans and point membership, and §4.4's gate design is now backed by a measured
> false-positive rate against the reference. Every quantitative claim about y30 and x13y29 in §2.2,
> §2.4 and §4.4 has been re-derived by me. Numbers elsewhere that remain delegated are marked
> *(delegated)* at their point of use.

---

## 0. Headline

Three findings change the framing of the hunt:

1. **The recurring "4-face detached chunk" is not a defect. It is the correct answer.** For
   z15/z30/z37/z45/z63 the OCCT reference cut result is *also* two solids, and its second solid is
   *also* 4 faces with a **bit-identical bounding box** to ours. The kernel's own gate already knows
   this — `validate_oracle.sh:20-22` says verbatim: *"The correct solid count is OCCT's OP_SOLIDS
   (z15/z30/z45 cut = 2!), never a hardcoded 1."* Question 2's premise ("one mechanism, not five
   bugs") is right about there being one mechanism; the mechanism is *the boolean disconnecting the
   body, correctly*. §2.1.

2. **The other detached piece — the 10-face chunk of y30 / x13y29 — is also correct, and here the
   reference is the one that is wrong.** Proven from the operands alone (item 5 below). Neither
   detached component in this corpus is a defect: our kernel's *solid decomposition* is right in
   every cell examined. §2.2.

3. **12 of y30's 13 non-manifold edges live on ONE face, and that face is covered exactly twice.**
   Result face `#11271` has an outer wire of 30 oriented edges in which **12 distinct edges appear
   twice**, its wire is **not closed**, its area is **2.0009×** the reference counterpart's, and it
   carries **11 two-edge inner loops in mirrored pairs, all inside one 0.08-wide box**. This is a
   *lasso*: a section chain that enters the face from the boundary, loops, and closes on itself; the
   leftmost-turn walk traverses its stem in both directions inside one cycle. The 13th non-manifold
   edge is on `#6386`, a one-face "solid" whose entire wire is one edge traversed three times; the
   body also contains a **zero-area face**. The non-manifold count is not a diffuse tolerance problem
   — it is a handful of degenerate arrangement outputs that nothing downstream rejects. §2.3, §3.

4. **y30 is not missing a region at all — the defect is one face, duplicated.** *(Re-verified
   directly.)* Our 32-face large component maps **1:1 onto the reference's 32 faces**, max centroid
   distance **0.000904**, 32/32 matched. Exactly one face is grossly wrong: **ours 35.379663 vs the
   reference's 17.682530, ratio 2.0008**, `isValid() = False`, **12 wires / 78 edges** where the
   reference has 1 wire / 8 edges, and its outer wire (24 edges, length 33.580661) is **not closed**.
   Total surface area 175.998851 vs the reference's 158.301420 — an excess of **17.697431**, which is
   the duplicated face's area to within 0.09 %. So the closed-but-wrong-volume hypothesis is
   **refuted for y30**: there is no mis-classified region, there is one face covered twice by a wire
   that never closed. §2.4.

5. **The 10-face chunk is correct too — and the reference is wrong on that cell.** Verified for
   x13y29 from the canonical operands, with no reference involved: `chunk.common(A)` = 1.41383024
   (its whole volume), `chunk.cut(A)` = 0, `chunk.common(B)` = 0, `chunk.cut(B)` = 1.41383024, and
   40/40 uniformly sampled interior points are inside A and outside B. So `chunk ⊆ A \ B` and the cut
   *must* contain it. A freshly recomputed `A.cut(B)` does not: `chunk.common(refcut)` = 0 and 40/40
   of those points fall outside it. It is also **not** a mis-assembled part of our big component —
   disjoint from it by **1.37 units**, sharing no edge, no face and no volume — so it is not a
   connectivity defect either. The reference is *stable* (reproduces to four decimals) but
   *incorrect* here; its partition identity still closes only because the lump is double-booked into
   `common`. §2.2.

6. **The failures split into two disjoint families, not one.** y30 / x13y29: correct partition, one
   degenerate face (R1). z15 / z45 / z37 / z63 / x20: whole regions never produced — z15's large
   shell is missing **18 reference faces totalling 53.351 of area** (R2). Ranking a single fix across
   both was the previous campaign's error. §5.

The structural answer to "why exact on primitives, broken on trimmed multi-face solids" is:
**an untrimmed analytic face's boundary is the chart rectangle, which the arrangement synthesises
itself and can never disagree about; a trimmed face's boundary is imported topology that enters the
arrangement as an ordinary peer pcurve, distinguished only by an index threshold `n_boundary`, and
every stage downstream (weld, snap, prune, walk, join, lift, key) treats boundary and section
symmetrically.** There is no point in the pipeline where the face's existing trim wire is
authoritative. §1.6 lists the eleven places that costs.

---

## 1. Q1 — where a face's trim boundary enters the split

### 1.1 Stage A: the shared scaffold consults *both* operands' trims (brep_section.cpp)

`build_section_scaffold` (`brep_section.cpp:379`) marches SSI chains, then clips them to the trimmed
regions of both operands:

- `face_loops_uv(X, si)` (`brep_section.cpp:191-257`) samples every loop of every face on surface
  `si` into a UV polyline, adaptively refined to chord sag `max(bbox)*2e-4`
  (`:221`, `:232-243`), with outer loops placed first and inner loops after (`:250-251`).
- `point_in_poly_uv` / `in_faces_uv` (`:275-299`) answer "is this UV sample inside the trimmed
  face", tolerant by `eps` on the loops (`:291`, `:295`).
- The **keep verdict** for a section interval is taken once, for both operands, at
  `brep_section.cpp:1367-1490`: `keep()` at `:1389-1390` is
  `in_faces_uv(loopsA[surfA], a) && in_faces_uv(loopsB[surfB], b)`; intervals are bisected at
  verdict changes (`:1401-1458`) and emitted as `keep_iv` → `SectionSegment`s (`:1469-1490`).
- Trim crossings are refined by `refine_trim_pave` (`:330`, called `:1149-1150`) into pave points,
  welded in 3D — these become `scaf->vertices`.

This is the *only* stage that treats the trim boundary as shared, authoritative data. Everything
after it is per-operand.

### 1.2 Stage B: the face's own trim wires are harvested (brep.cpp `split_with`)

`BRep::split_with` — `brep.cpp:3268`.

```
brep.cpp:4024-4044   for (int li : face.loop_indices) {
                         ... pcs.push_back(m_curves_2d[c2]);          // the pcurve
                             pcs_eids.push_back(m_trims[ti].edge_index);  // its ORIGINAL edge
                         if (bloop.type == Inner) { has_inner = true; inner_loops.push_back(pcs); }
                         else { outer_pcs = pcs; scaf_bnd_edge_ids = pcs_eids; }
                     }
```

Two facts decided here and never revisited:

- **`scaf_bnd_edge_ids[i]` is the pave-block identity of boundary pcurve `i`** — the only link back
  to the operand's original edge. It is consumed at `brep.cpp:3869-3874` to look up `oe` and at
  `:3880` to fetch that edge's pave subdivision.
- **Inner loops are dropped from the arrangement entirely.** `all_pcs` is built from `outer_pcs` +
  `cut_pcs` only (`:4558-4560`); `inner_loops` never reaches `split_by_uv_curves`. To stay correct
  the code therefore bails out: `if (cut_pcs.empty() || has_inner) { ... append_face(...); continue; }`
  (`brep.cpp:4350-4358`). **A face with a hole is never split, whatever the section does to it.**
  (Measured: the chair operands have zero inner loops — 20 faces / 20 outer bounds / 0 inner bounds
  each — so this is a *latent* defect for chairs, not an active one. It is active for any operand
  with a through-hole, i.e. `create_block_with_hole` and every real imported part.)

### 1.3 Stage C: section segments become cut pcurves, gated against the trim boundary

`brep.cpp:4055-4263`, scaffold path:

| line | what |
|---|---|
| 4061-4084 | `ov` (stub overshoot) and `scaf_forced_eps` from the UV-image of `scaf->tol3`, capped at `min_range*1.3e-1` — the comment at `:4077-4080` records that the previous cap of `6e-2` made y30 seg4's forced node MISS at 0.0514, so "cut dangled, face never split" |
| 4090-4120 | this face's boundary re-sampled to sag `min_range*5e-4` → `bnd_polys_sc` |
| 4151 | `eps_border = min(range_u, range_v) * 2e-3` |
| 4160-4167 | junction endpoints counted so shared ends never get stubs |
| **4176-4181** | **`on_border` drop**: if 5 probes along the segment are all within `eps_border` of this face's own trim boundary, the segment is **discarded** (unless `SESSION_KEEP_BORDER`) |
| 4185-4188 | segment endpoints pushed to `scaf_forced_nodes` (the OCCT `FaceInfo.On` analog) |
| 4202-4251 | overshoot stubs, only for ends in the band `[0.9*forced_eps, max(5*ov, 2*forced_eps))` |
| 4254-4262 | `scaf_cut_seg_ids[i]`, `scaf_cut_spans[i] = {n_pre*step, (n_pre+|uv|-1)*step}` — the run-to-segment identity map |
| 4272-4332 | `SESSION_SE_OTHER` (PutSEInOtherFaces) — **opt-in, off** |
| 4337-4343 | same-domain imprint cuts (`extra_cuts`), tagged `seg_id = -1` |

The non-scaffold (legacy/analytic) path at `:4364-4500` does the same job with three more drops
(out-of-domain clip `:4364-4437`, chart-border drop `:4438-4459`, own-boundary-coincidence drop
`:4460-4499`) and the opt-in `SESSION_TRIM_SNAP` bridges at `:4509-4557`.

### 1.4 Stage D: the arrangement (`nurbssurface_trimmed.cpp:536`)

```
brep.cpp:4558-4560   int n_boundary = (int)outer_pcs.size();
                     std::vector<NurbsCurve> all_pcs = outer_pcs;
                     all_pcs.insert(all_pcs.end(), cut_pcs.begin(), cut_pcs.end());
brep.cpp:4584-4589   parts = NurbsSurfaceTrimmed::split_by_uv_curves(
                         srf, all_pcs, tolerance, /*use_domain_border=*/false, n_boundary,
                         snap_bnd, &scaf_forced_nodes, scaf_forced_eps);
```

`snap_bnd = (imported_freeform && !scaf) ? 0.05 : 0.0` (`:4584`) — **zero on the scaffold path**, so
the 1b/1b2 endpoint-snap and junction-weld blocks (`nst:734-808`) are inert for chairs.

Inside `split_by_uv_curves`:

| nst line | stage | note |
|---|---|---|
| 539-541 | `is_boundary(cidx) = cidx < n_boundary` | the *entire* distinction between trim and section |
| 565-569 | `snap_uv = tolerance / uv_to_3d` | the vertex-pool weld radius |
| 589-661 | sample every pcurve to `samp_tol = max(range)*2e-5` | boundary and cuts identically |
| 663-732 | **forced boundary nodes**: insert each pave as an exact sample on the nearest *boundary* polyline, then snap only bit-equal cut ends to it (`:719 eps_exact = max(1e-12, snap_uv*4)`) | a MISS is silent (`:688-693`) |
| 734-774 | `bnd_snap` endpoint projection | inert on scaffold path |
| 776-808 | `jweld` interior-endpoint weld | inert on scaffold path |
| 818-840 | **boundary-loop endpoint weld** at `bweld = max(snap_uv*8, min(range)*5e-4)` | closes imported wires whose corners disagree |
| 842-857 | drop degenerate cut polylines | |
| 859-1050 | seg-seg crossings + `newton_cc` refinement; `SESSION_NODESNAP` crossing snap **default OFF** (`:918-920`) | see §2.3 |
| 1053-1075 | `vert_id` — the UV vertex pool, 3×3 cell scan at `snap_uv` | |
| 1077-1100 | polylines → `SplitEdge{a,b,cidx,ta,tb}` | |
| **1115-1147** | **dangling prune**: iteratively kill any edge with a valence-1 endpoint. `SESSION_SECPROTECT` (which would exempt section cuts) is **default OFF** (`:1122`) | this is what makes `parts=1` |
| 1177-1222 | half-edge build, `atan2` angular sort, `next_he` leftmost-turn, cycle extraction | **no self-intersection or repeated-edge check** |
| 1224-1281 | signed areas; `pos_faces` (area > snap_uv²) become fragments, `neg_faces` not touching the border become holes | sign of a sliver's area is numerical noise |
| 1292-1318 | `holes_of[best]` — smallest containing positive cycle | |
| 1325-1402 | `cycle_to_segments`: consecutive same-`cidx` half-edges collapse into one run, trimmed from the *original* pcurve; on failure a **straight chord** with `srcs = {-1,0,0}` (`SEGFALL`, `:1373-1399`) unless `SESSION_SEGKEEP` |
| 1407-1520 | `cycle_to_loop`: join at `join_tol = max(snap_uv*4, min(range)*1.5e-3)` on the scaffold path (`:1424-1426`), close-weld cap `max(join_tol, forced_node_eps)` (`:1463-1473`); on failure → **polyline fallback loop and `seg_valid = false`** (`:1498-1519`) |
| 1548-1585 | emit one `NurbsSurfaceTrimmed` per positive cycle with `m_outer_segments` / `m_outer_segment_srcs` |

### 1.5 Stage E: runs are lifted to 3D and keyed into edges (`append_face`, brep.cpp:3669)

Back in `split_with`:

```
brep.cpp:4666-4681   loops.push_back({Outer, part.m_outer_segments or {part.m_outer_loop}});
                     loop_srcs.push_back(part.m_outer_segments.empty() ? nullptr
                                                                       : &part.m_outer_segment_srcs);
                     append_face(part.m_surface, loops, scaf ? &loop_srcs : nullptr);
```

`append_face` then routes each run three ways:

- **Section run** (`srcs[pidx][0] >= n_boundary`, `brep.cpp:3704-3863`): remap the run's clip params
  onto the *shared* chain `s.p3` (`chain_pos`, `:3743-3749`), extract the sub-polyline (`:3752-3767`),
  and key it. Whole-segment runs (`fa < 1e-2 && fb > nCh-1-1e-2`, `:3834`) get `sec_edges_out` +
  `sec_emap` so combine can alias them cross-operand; partial runs get only `sec_spans_out`
  (`:3849-3850`). `SESSION_BOP2` short-circuits to a pooled edge at `:3776-3796`.
- **Boundary run** (`oe = scaf_bnd_edge_ids[bidx] >= 0`, `:3869-3874`): subdivide at `edge_paves[oe]`
  (`:3878-3947`, the `MakeSplitEdges` analog), lift, then look up `bemap[{oe,lo,hi}]` with a tolerant
  arc-midpoint compare, tolerance `1.5*(devtol + pr.dev) + bemap_tol*0.1` (`:3977`).
- **Legacy run** (`oe < 0`, `:3989-4001`): exact-quantized key `emap[{lo,hi,q6(pm)}]`.

All three end in `result.add_trim(ci2d, ei, li, false, ttype)` (`:3787`, `:3845`, `:4003`).

Vertex identity comes from `find_or_add_vertex` (`:3503-3574`): original-corner capture
(`orig_tol2`, radius `diag*5e-4` capped at ⅓ of the closest corner pair, `:3454-3472`) → pave capture
(`pave_tol2 = (0.7*tol3)²`, `:3434-3446`) → a tolerant `q6` grid weld (`:3553-3573`).

### 1.6 What actually differs between unrotated and rotated chairs

The base (unrotated) chairs are exact through this same code. Eleven divergences, all of which are
*conditional on the section meeting the trim boundary transversally and at a well-conditioned
angle*, and all of which are **one-sided** (they fire on one operand's face and not on the mating
face of the other operand) — which is why they produce naked/non-manifold rather than a
symmetrically wrong answer:

| # | divergence | site | fires when |
|---|---|---|---|
| D1 | segment discarded as `on_border` | brep.cpp:4176-4181 | section grazes *along* a trim edge (rotation makes coplanar/near-coplanar contact generic) |
| D2 | forced node MISS → the cut's end is never welded to the boundary | nst:688-693, cap at brep.cpp:4081-4082 | the junction's 3D point sits a graze-distance off the surface, so its closest-point UV drifts into the face (comment names y30 seg4 at 0.0514) |
| D3 | stub band miss: `d_f` outside `[0.9·eps, max(5·ov, 2·eps))` → no stub, no crossing | brep.cpp:4209, 4234 | `SESSION_STUB_REACH` (the boundary-*reaching* stub) is opt-in |
| D4 | valence-1 cut deleted wholesale by the dangling prune → `parts=1`, face emitted uncut | nst:1128-1147 | consequence of D1/D2/D3; `SESSION_SECPROTECT` off |
| D5 | near-tangential crossings leave a spurious node ~1e-2 away (`newton_cc` stalls on a singular Jacobian) → the wire self-intersects | nst:904-928 (comment names z90 WIRE 38 / z45 / x13y29); `SESSION_NODESNAP` **off** | grazing sections, i.e. rotation |
| D6 | exact angular tie in the leftmost-turn walk when a snapped stub tip leaves a chord collinear with the boundary → the cut is traversed as a slit | nst:715-718 (comment), walk at nst:1188-1206 | same |
| D7 | `cycle_to_loop` JOINFAIL → polyline fallback, `seg_valid=false` → `loop_srcs = nullptr` → `append_face` gets no `srcs` → **no chain lift, no `seg_id`, no `sec_edges` entry** | nst:1498-1519 → brep.cpp:4671, 3705 | census_y30.md:17 records `gapmax=6.194e-03 > tol=4.419e-03` on A si=0 |
| D8 | `SEGFALL` → straight chord with `src = -1` (identity erased) | nst:1373-1399 | `SESSION_SEGKEEP` off |
| D9 | zero-span collapse: the operand's arrangement clips a shared segment to a point → the copy is **dropped** (`SESSION_ZEROFILL` off) → one-sided imprint | brep.cpp:3721-3733 | section runs along this operand's trim |
| D10 | boundary pave rejected as too far (`> 1.7*pave_cap_tol`) or too close to an end (`< span*1e-3`) → the two flanking faces' copies of one physical sub-edge cover **different** intervals → T-junction | brep.cpp:3909, 3916 | grazing paves |
| D11 | `MICROFRAG` drops a whole sliver fragment (and its perimeter edges) | brep.cpp:8836-8882 | slivers only exist under grazing |

The consequence chain is documented independently in the census. `kb/census_y30.md:51-53`:

> e121 = seg17 … A lost: A si=13 `[SPLIT] cuts=2 parts=1` — face NOT split because the seg17+seg18
> chain is open at v23/v27, so no A section edges were created ⇒ `[SEGLOST] A seg=17, seg=18`.

and `:85`: two valence-1 pairs with gaps 0.4656 and 0.2512 that `bridge(march=0 weld=0 resid=0)`
never closed, cascading into `parts=1` on two A faces and divergent invented closures on both sides.

For **untrimmed axis-aligned primitives** none of D1–D11 can fire: `use_domain_border=true`, the
"boundary" is the four synthetic chart sides created at `nst:811-816`, sections meet them
transversally at analytic parameters, `parts` is always ≥2, and the two operands' arrangements are
*forced* to agree because both are trivially exact. That is the whole of the contrast.

---

## 2. Q2 — the recurring small detached component

### 2.1 The 4-face chunk is correct. Measured.

Parsing the shell structure of every exported result against the OCCT reference
(`/home/petras/split_audit` scripts; entity-graph walk of `CLOSED_SHELL`/`OPEN_SHELL`):

| cfg (cut) | OURS (result components) | REFERENCE |
|---|---|---|
| x20 | OPEN 33 | CLOSED 38 |
| **y30** | **CLOSED 1, CLOSED 33, CLOSED 10** | **CLOSED 32** |
| z15 | OPEN 4, OPEN 34 | CLOSED 40, **CLOSED 4** |
| z30 | OPEN 4, OPEN 37, OPEN 1 | CLOSED 38, **CLOSED 4** |
| z30x20 | CLOSED 34 | CLOSED 32 |
| z37 | OPEN 4, OPEN 38 | CLOSED 38, **CLOSED 4** |
| z45 | OPEN 4, OPEN 34 | CLOSED 37, **CLOSED 4** |
| z63 | OPEN 4, OPEN 35 | CLOSED 37, **CLOSED 4** |
| z90 | CLOSED 37 | CLOSED 36 |
| **x13y29** | **CLOSED 30, CLOSED 10** | **CLOSED 29** |

Bounding boxes of the 4-face components, ours vs reference:

```
z45  ours [3.770, 2.367, -4.535, 12.266, 6.812, 5.625]
     ref  [3.770, 2.367, -4.535, 12.266, 6.812, 5.625]      identical
z15  ours [4.196, 3.634, -4.535, 12.266, 5.488, 5.625]
     ref  [4.196, 3.634, -4.535, 12.266, 5.488, 5.625]      identical
```

Confirmed exactly by an independent OCCT measurement of the chunk itself:

```
z15 chunk: exact volume 0.027732   REFERENCE z15 cut solid1 volume 0.027734
           OCCT isInside on 30 interior points: A=30  B=0  REF=30
           distance(chunk, ref solid1) = 0.000000
z45 chunk: exact volume 0.087782 ; distance(chunk, ref solid1) = 0.000000 ; A=30 B=0 REF=30
```

**Verdict: for z15/z30/z37/z45/z63 the cut genuinely disconnects a 4-face lump, the reference
produces it, and we produce it in the right place with the right volume.** It is not a defect and no
mechanism needs to be found for it. The kernel's own `validate_oracle.sh:20-22` already records this
("z15/z30/z45 cut = 2!"). The real defect in those five cells is that **both** components fail to
close and the large one is short of faces (z15: 34 vs 40; z45: 34 vs 37; z63: 35 vs 37).

### 2.2 The 10-face chunk (y30, x13y29) is CORRECT — the reference omits it

y30 reference = one 32-face solid. Ours = **three** components: 33 faces + 10 faces + **1 face**.
x13y29 reference = one 29-face solid; ours = 30 + 10.

**What the chunk is.** A rectangular foot/leg block sitting on chair A's own z-minimum
(A zmin = −1.4389). y30: exact volume **0.73320328**, area 6.420197, tight bbox
x[6.6686, 9.2275] y[2.5517, 4.5051] z[−1.4381, −0.8404], centroid (7.63970, 4.03474, −1.15884),
0 naked edges, 0 non-manifold edges. x13y29: volume **1.41383024**, area 9.884393. Four near-planar
walls (y ≈ 3.63, x ≈ 6.68, x ≈ 8.72, y ≈ 4.49), a curved bottom and top, four small fillet strips.
No two of its faces share a control net, so it is **not** a coincident duplicate pair — it is a
genuine solid lump, correctly built and correctly sewn.

**Two independent measurements disagree about which side it belongs on.**

*Direct membership tests — every one says it is in `A \ B`, i.e. it belongs in the cut:*

```
all 10 faces lie ON operand A's boundary       max deviation <= 6.7e-4
none lies on operand B                         closest approach to B 0.10 .. 0.52
voxel, 3-axis majority, h = 0.0426             99.63 % in A,  0.77 % in B,  98.86 % in A\B
OCCT isInside, 30 interior points              A = 30/30,     B = 0/30
ball r = 0.15 at the centroid                  100 % in A,    0 % in B
+x ray from the centroid                       does not enter B until t = 0.798 (y30) / 1.063
centroid-to-B-boundary distance                0.262661 (y30) / 0.384795 (x13y29)
```

*Reference-relative tests — say the reference files it under `common`:*

```
OCCT chunk ∩ REFERENCE_y30_cut       = 0.00000000
OCCT chunk ∩ REFERENCE_y30_common    = 0.73320328   (the chunk's entire volume)
OCCT chunk ∩ REFERENCE_x13y29_cut    = 0.00000000
OCCT chunk ∩ REFERENCE_x13y29_common = 1.41383024
```

> **RETRACTION.** An earlier revision of this document claimed the reference "is demonstrably not a
> fixed point on this corpus", citing a recomputed x13y29 `A.cut(B) = 79.732960` against the stored
> 48.472832. **That number is wrong and is withdrawn.** It came from a delegated measurement that I
> passed through without re-deriving it. Recomputing from the canonical operands
> (`operands/chair0.stp`, `operands/B_x13y29.step`) under FreeCAD 1.1.1 / OCCT 7.8.0:
>
> ```
> x13y29  A.cut(B) = 48.473392 (1 solid, 29 faces)   stored 48.472832   delta 0.000560
> y30     A.cut(B) = 46.959623 (1 solid, 32 faces)   stored 46.958863   delta 0.000759
> z15     A.cut(B) = 80.269517 + 0.027734            stored 80.269405 + 0.027734
> ```
>
> **The stored references are stable and reproduce to four decimals.** The 79.732960 figure is
> ≈ vol(A) minus a sliver — the signature of cutting with a barely-overlapping or wrongly-posed B.
> Standing rule reaffirmed: verify the operand before trusting a number that contradicts a stable
> reference.

**Re-derived from the verified reference — and the answer is sharper, not weaker.** With
`refcut = A.cut(B)` computed fresh from the canonical operands, and the chunk extracted from our own
result as a closed solid:

```
x13y29   our BIG component   30 faces, CLOSED solid, vol 48.517728
         our CHUNK           10 faces, CLOSED solid, vol  1.41383024
         verified REF cut     1 solid, 29 faces,     vol 48.473392

   operands only, no reference involved:
      chunk.common(A) = 1.41383024   chunk.cut(A) = 0.00000000   -> chunk ⊆ A
      chunk.common(B) = 0.00000000   chunk.cut(B) = 1.41383024   -> chunk ∩ B = ∅
      40 uniformly sampled chunk-interior points:  inA = 40/40,  inB = 0/40
   therefore  chunk ⊆ A \ B  — the chunk is real material that the cut must contain.

   against the verified reference:
      chunk.common(refcut) = 0.00000000
      40/40 chunk-interior points are OUTSIDE the reference cut
      refcut.cut(chunk) = 48.473392  (unchanged — the reference contains none of it)

   is it a duplicate of our own big component?
      chunk.common(BIG) = 0.00000000              -> no overlap
      min midpoint distance chunk-edge -> big-edge = 1.37 ; nearest 6 are 1.37 .. 1.74
      0 of 25 chunk edges lie within 0.05 of any big-component edge
      0 of 10 chunk faces share a centre of mass with any big-component face
```

**This resolves the question, and it rules out the coordinator's connectivity reading.** The chunk is
not a mis-assembled piece of the single solid: it is **spatially disjoint from the big component by
1.37 units**, shares no edge, no face and no volume with it, and could not have been sewn to it by
any tolerance. Nor is it a duplicate. It is a separate lump of `A \ B` — proven by operand-only tests
that never touch the reference — which OCCT's `A.cut(B)` omits entirely.

So the two facts the coordinator identified as jointly impossible are in fact jointly true, and the
resolution is the third option neither of us listed: **the reference is stable but incorrect on this
cell.** Stable ≠ correct. OCCT's partition identity still closes
(48.473392 + 31.823906 = 80.297298 vs vol A 80.296907, residual 0.000391) because the missing lump is
double-booked into `common` — an error that is invisible to the very identity we were relying on as a
cross-check.

**Verdict: our kernel is right here and the reference is wrong.** The 10-face chunk is a *correct*
second solid, like the 4-face chunk of §2.1. Strike it from the defect list. What remains genuinely
wrong in x13y29 is the big component's volume: 48.517728 against 48.473392, **+0.044336**.

**Naming the detachment mechanism anyway**, since it is what *would* produce a spurious detached
component and is worth having on record. There is no shell concept inside `BRep` at all — the class has faces,
loops, trims, edges, vertices and nothing else. Components are formed for the first time at STEP
write, `file_step.cpp:3856-3857`: *"Group written faces into edge-connected components — one shell
(and one MANIFOLD_SOLID_BREP) per component."* So a detached component is **purely** an artefact of
which trims ended up on which edge record. The three candidate mechanisms in the brief resolve as:

- *not* a face group separated in an assembly/connectivity step — there is no such step;
- *not* a sub-loop getting its own shell — holes are attached to faces at `nst:1292-1318` and never
  promoted;
- **it is duplicated-edge mating**: `subset()` (`brep.cpp:2821-2896`) rebuilds one edge record per
  original edge for the kept faces, and the cross-operand alias at `brep.cpp:9761-9788` only unifies
  A and B copies whose **quantized chain span** `(seg, round(fa*32), round(fb*32))` matches. Any
  section block whose two operands disagree on the span (D7/D8/D9/D10 above) stays as two separate
  edge records; if each side's copy happens to acquire two trims from its own operand's fragments,
  the result splits into two watertight shells.

### 2.3 The dominant *measured* defect: degenerate arrangement wires

**Read the edge counts at the right layer.** Two measurements of `res_y30_cut.step` look
contradictory and are not:

| layer | how measured | histogram |
|---|---|---|
| **as our kernel wrote it** | count `ORIENTED_EDGE → EDGE_CURVE` references in the raw STEP text | 268 edges: **0 used once, 255 twice, 13 three times** |
| **as a consumer sees it** | `Part.Shape().read()`, then OCCT's own topology | 314 edges: **58 used once, 256 twice, 0 three times** |

OCCT's importer **splits** each of the 13 non-manifold `EDGE_CURVE`s, which is where the 58 free
boundaries come from (+46 edges). Both numbers are correct for their layer; quoting the post-import
one alone hides the defect, quoting the pre-import one alone hides the damage. Geometric clustering
of coincident curves in the imported shape finds the same object from the other side — 10 clusters at
1e-3 rounding (11 @1e-2, 8 @1e-4), with reuse counts of 3, 4, 5, 8 and even 10, **all inside one
0.09-wide blob at (11.96, 4.03, 1.66)**, all degree-3 B-splines, two of them **exactly degenerate**
(zero length, used 10× each).

Those edges are **section (intersection) edges, not operand boundary edges**: each lies within
2.1e-12 of operand A *and* within 3.1e-6 of operand B. Over all 162 distinct result edges the split
is **125 section / 37 A-boundary-only / 0 B-only / 0 off-both**.

Pre-import distribution of the 13:

```
e#6380  used=3   faces=[6386]                      <- ONE face uses it three times
e#6237  used=3   faces=[6320, 11271]
e#6441  used=3   faces=[11271, 19343]
e#6844  used=3   faces=[11271, 16526]
e#6900  used=3   faces=[11271, 19882]
e#6924  used=3   faces=[11271, 19603]
e#7043  used=3   faces=[11271, 19603]
e#7197  used=3   faces=[11271, 19603]
e#7221  used=3   faces=[11271, 19603]
e#7271  used=3   faces=[11271, 19603]
e#7390  used=3   faces=[11271, 19603]
e#7881  used=3   faces=[11271, 19866]
e#8284  used=3   faces=[11271, 17950]
```

Face `#11271`'s wire structure:

```
FACE_OUTER_BOUND  30 oriented edges:
  6412, 6237, 6441, 6844, 6900, 6924, 7043, 7162, 7197, 7221, 7245, 7271, 7390, 7471, 7881, 8284,
  8401, 6237, 6441, 6844, 6900, 6924, 7043, 7197, 7221, 8526, 7271, 7390, 7881, 8284
        ^^^^ 12 edges repeated inside the SAME wire
FACE_BOUND  [8963, 8750]      FACE_BOUND  [8750, 8963]
FACE_BOUND  [9401, 9188]      FACE_BOUND  [9188, 9401]
FACE_BOUND  [9839, 9626]      FACE_BOUND  [9626, 9839]
FACE_BOUND  [9976, 10103]
FACE_BOUND  [10826, 10469]    FACE_BOUND  [10469, 10826]
FACE_BOUND  [11262, 11049]    FACE_BOUND  [11262, 11049]
```

Face `#6386` (the 1-face "solid"): `FACE_OUTER_BOUND` = `[6380, 6380, 6380]`.

Reading: `#11271` is a **lasso**. A section chain enters from the trim boundary, runs into the
interior, and closes on itself (or on a near-tangential spurious crossing, D5). The stem has valence
2 everywhere so the dangling prune (`nst:1128-1147`) cannot remove it; the leftmost-turn walk
therefore traverses the stem **out and back inside one cycle**. `cycle_to_segments`
(`nst:1325-1340`) emits both traversals as separate runs; both lift to the same 3D curve; the
`emap`/`bemap` lookup in `append_face` returns the *same* edge for the second traversal and marks it
`Mated` — so one face contributes **two** trims to each stem edge. The neighbouring fragment
(`#19603`, `#19343`, …) then contributes the third. The eleven 2-edge inner bounds are the lasso
heads / sliver lenses, several emitted twice with opposite traversal — degenerate cycles whose
signed area sign is numerical noise at the `snap_uv²` threshold (`nst:1266-1281`).

An independent OCCT read of the same face (after the importer decomposes the non-manifold edges)
quantifies the damage exactly:

```
our f03  (= STEP #11271)  area 35.379663   reference counterpart area 17.682249   ratio 2.0009
   wires = 12,  edges = 78,  vertices = 43        reference counterpart: 1 wire, 8 edges
   wire0 : 24 edges, closed = FALSE, len 33.5807   (reference outer wire: 8 edges, closed, len 20.4704)
   wire1..11 : eleven micro-loops, len 0.0306-0.0654, ALL inside the 0.08-wide box
               x[11.9436,11.9876] y[3.9029,4.1484] z[1.6380,1.6817]
               and they pair up: w1≡w2, w3≡w4, w5≡w6, w8≡w9, w10≡w11
```

Four things pin the diagnosis:

- `isValid()` returns **False**, and the outer wire is **not closed** — it has four free endpoints:
  `(12.66218,1.63897,1.01594)`/`(12.66218,1.63899,1.01593)`, a 2e-5 numerical gap, and
  `(11.96673,4.01922,1.66095)`/`(11.94356,4.14843,1.63796)`, the real gap, at the micro-tangle.
- The **untrimmed** area of the underlying patch is **17.862850**, so the reference's 17.682249 is
  99 % of the whole patch and our 35.379663 is a bogus double count. The ratio is **2.0009 — the
  face is covered exactly twice**, which is what a slit/lasso wire integrates to.
- **RETRACTED:** an earlier revision claimed this face's divergence-theorem flux is "exactly
  0.000000, so it bounds nothing". Re-measured directly, it is **73.257183** against the reference
  counterpart's **73.257742** — essentially identical, not zero. The double cover shows up in
  **area**, not in flux. That retraction matters, because a conclusion was built on it (§4.4).
- The eleven micro-holes are one cluster of near-tangential crossing slivers, each emitted with both
  traversal senses. These are the section fragments that should have been assembled *into* the outer
  wire and instead became free-floating loops.

The result body also carries a **zero-area face** (`area = 0.000000`, centroid
`(11.9462, 4.1170, 1.6489)` — inside that same 0.08 box) as its own one-face component, and a
0.0759-area stray face as another. All 58 of y30's post-import naked edges are micro
(**total length 0.9976**) and every one lies inside `x[11.94,11.99] y[3.90,4.15] z[1.638,1.682]` —
i.e. **the entire y30 failure is confined to one 0.09-wide blob.**

**Nothing between the walk and the STEP writer rejects any of this.** There is no wire-validity check
(closed, repeated edge, self-intersection, zero area, area-vs-loop consistency), `is_solid()`
(`brep.cpp:910-965`) is never called inside `boolean`, and the AUTO metric is blind to it (§4.4).

The kernel's own comment already identified the trigger and shipped the fix **disabled**:

> `nst:911-917` — "near-tangential section cuts that share a paved node still leave a spurious ~1e-2
> crossing (newton_cc stalls on the singular Jacobian) that **self-intersects the wire** (chairsROT
> z90 cut WIRE 38 / z45 / x13y29). Snapping the CROSSING POINT onto the shared node … collapses the
> sliver, so no phantom split forms."

`SESSION_NODESNAP` is opt-in (`nst:918-920`) and `bnd_snap` is 0 on the scaffold path
(`brep.cpp:4584`), so on chairs this block is completely inert.

---

### 2.4 y30 is missing NOTHING — one broken face accounts for the whole volume error

Face-by-face matching of our large component against the **freshly recomputed** reference solid
(`A.cut(B)` from the canonical operands, vol 46.959623, area 158.301420) — all of this re-derived
directly, not delegated:

```
matched 32/32 reference faces      our-unmatched 0      max centroid distance 0.000904
total area  ours 175.998851   reference 158.301420   excess +17.697431

only three faces differ in area by more than 1e-3, and two of them are noise:
   our f3  <-> ref f2    35.379663 vs 17.682530   ratio 2.0008    <- the defect
   our f12 <-> ref f29    0.002337 vs  0.002346   ratio 0.9962
   our f27 <-> ref f27    0.020096 vs  0.020123   ratio 0.9987
```

The excess area **17.697431** is the reference face's area **17.682530** to within 0.09 %: the face
is present twice over. Its wire structure against the reference's:

```
ours:  area 35.379663  isValid=False  12 wires  78 edges
       wire0  24 edges  closed=False  len 33.580661     <- outer wire never closed
       wire1..11  2..9 edges each, closed, len 0.0306 .. 0.0654, in mirrored pairs
ref :  area 17.682530  isValid=True    1 wire   8 edges
       wire0   8 edges  closed=True   len 20.470548
```

**No contiguous sub-region of the reference cut is absent from our result.** The entire y30 error is
one face covered twice by a wire that never closed, caused by a degenerate SSI tangle in a 0.09-wide
blob at (11.96, 4.03, 1.66). That refutes the closed-but-wrong-region hypothesis for this cell.

> **Retracted from an earlier revision:** "the broken face's flux is exactly 0.000000, so it bounds
> nothing" and "substituting the reference's face gives 47.073033". Re-measured: the broken face's
> flux is **73.257183** vs the reference counterpart's **73.257742** — nearly identical, not zero.
> Both retracted numbers were delegated and not re-derived. The duplication is visible in **area**,
> not in flux. (My own uv-quadrature summed 131.57 over the 32 faces against a true 46.96, so that
> integrator is not fit for volume either and no volume figure is asserted here — the shell is open,
> so its volume is undefined regardless.)

Two caveats on the brief's numbers, both worth recording:

- **The 23.9619 figure does not reproduce on the current artefacts.** The result in
  `verify_now/res_y30_cut.step` has no closable large shell at all: `Part.Solid` fails, and
  `sewShape()` / `fix()` at 1e-3 … 0.2 all fail to close it. The only closed piece is the 0.7332
  chunk. (46.958863 for the reference does reproduce exactly.) So "y30 closes with volume 23.9619"
  describes a different build or a different run, and any conclusion resting on it needs re-taking.
- **z15 and z45 are the opposite case** and are where genuinely missing regions live:

  ```
  z15  ours 34 faces / area 164.1823   reference main solid 40 faces / area 176.8216
       18 reference faces UNMATCHED, total area 53.351;  12 of ours unmatched
       largest absent: 14.2918 @ (9.0334,4.5255,0.5372)   11.7583 @ (10.2794,3.1043,-0.6872)
                        8.6920 @ (8.1654,2.8852,1.2544)    4.7663 @ (7.4886,3.7045,0.1749)
       17 naked edges, total length 20.3221 -- LONG real boundaries (2.93, 2.27, 1.99, 1.86, ...)
  z45  ours 34 / 167.3432   reference 37 / 178.5381
       4 reference faces unmatched, total area 11.1864 (7.5957 @ (9.3424,3.4828,-0.4680) + ...)
       20 naked edges, total length 23.4810;  two of our faces isValid()==False with OPEN outer wires
                                             (areas 13.474912 and 9.641171)
  ```

  Long naked edges and unmatched reference faces = **regions never produced** (R2). Micro naked edges
  in one blob = **one degenerate wire** (R1). The two families are cleanly separable by that single
  statistic — naked-edge length distribution — and no previous census made that split.

## 3. Q3 — every path that can attach a third trim to an edge

Enumerated by grepping every `add_trim(` / `t.edge_index = ` / `trim_indices.push_back` in the
snapshot and reading each site's guard.

| # | path | site | guard | does the guard exclude a 3rd trim? |
|---|---|---|---|---|
| P1 | `bemap` boundary-run mate | brep.cpp:3969-3988, then `add_trim` :4003 | tolerant arc-midpoint match inside bucket `(oe, lo, hi)`; `mt = 1.5*(devtol+pr.dev)+bemap_tol*0.1` | **NO.** No trim count check, no "different face/loop" check, no cap. Any number of runs whose arc-mid falls in the tube bind to `pr.ei`. This is the path that turns a lasso's two same-face traversals into a 2-trim edge, then a neighbour's run into a 3rd. |
| P2 | `emap` exact-key mate (section + legacy) | brep.cpp:3813-3822 (`ttype = Mated`), :3990-4000; `add_trim` :3845, :4003 | key `(lo, hi, q6(pm))` | **NO.** No count check, no face check. |
| P3 | `SESSION_BOP2` pool referencing | brep.cpp:3776-3796 | `ttype = trim_indices.empty() ? Boundary : Mated` | **NO** — the ternary *reports* the state, it does not gate. Gated only by `SESSION_BOP2` being set. |
| P4 | within-operand common block (`CBLOCK`) | brep.cpp:4699-4788 | candidates require `uses[ei]==1` (:4705); `acc_trims < 2` cap (:4735-4736); `gone[]` | Caps at 2 **but has no different-face check** — it can merge two 1-trim edges of the *same* face (creating a slit), which then takes a 3rd trim from the neighbour via P1/P2. |
| **P5** | **cross-operand alias at combine** | **brep.cpp:9761-9788** | `b_edge_new[j] = span_to_res[{seg, qspan(fa), qspan(fb)}]` — then every subB trim on edge `j` is re-pointed (`:9802-9806`) | **NO GUARD AT ALL.** Neither side's trim count is examined. If subA's edge already has 2 trims (both A flanks kept — a classification error, §4) the alias makes it 3 or 4. `qspan` quantizes to 1/32, so distinct spans collide and several subB edges can alias onto one subA edge. |
| P6 | BOP2 arena alias by index | brep.cpp:9761-9773 | index equality only | **NO.** Same as P5, gated on `SESSION_BOP2`. |
| P7 | xweld common-block merge | brep.cpp:10019-10100 | `uses[ei]==1` (:10025); `gone[a2]/gone[b2]` ⇒ each edge merges at most once | Caps at 2. **No different-face check** — can create a same-face slit. |
| P8 | NK-RESCUE tolerant mate-pair | brep.cpp:10101-10189 | `uses3[ei]==1` recomputed after P7 (:10108-10110); `goneR[]`; endpoint radius `0.15*tol3` (0.5 under M3) | Caps at 2. **No different-face check.** Also welds vertices globally (`:10182-10188`), which can retro-actively merge *other* edges' endpoints. |
| P9 | M3 network-vertex imprint | brep.cpp:9855-10017 | `usesM[ei]==1` (:9909), `has_mate` veto (:9917) | Creates new edges with 1 trim each; safe by itself, but multiplies the candidate pool for P7/P8. Gated `SESSION_M3`. |
| P10 | `imprint_edges` | brep.cpp:4837-5071, attach at :5045-5050 | `if (!mated_too && trim_indices.size() >= 2) continue;` (:4877) | With `mated_too=false` (the `result` call, :10379) only 1-trim edges are split, so each piece gets 1 trim. With **`mated_too=true`** (`A2/B2.imprint_edges(scaf.tol3*0.7, true)`, :8578-8579, gated `SESSION_BOP2`) already-mated edges are split and **the attach loop has no cap** — pieces can accumulate 3+. |
| P11 | `co_refine_coincident_edges` | brep.cpp:5187-6104, attach at :5738-5741 | `cand(e)` requires `trim_indices.size() < 2` (:5219) for an edge to be *split* | The attach loop pushes onto `edge_for[k]` with **no cap**. Three faces imprinting onto the same sub-arc set ⇒ 3 trims. Called unconditionally at :10402 (and inside the fuzzy ladder :10443, :10491). |
| P12 | `normalize_section_blocks` | brep.cpp:6182-6530, attach at :6416-6417, :6433-6438, :6472 | explicit cap `if ((int)rt.size() >= 2) { ++n_cap; break; }` at :6469 and `while (... rt.size() < 2)` at :6471 | **YES** — this one is correctly capped. |
| P13 | `sew_coincident_edges` | brep.cpp:6873-7278 | `is_candidate` = `<2 trims` (:6904); `rep_trims[r] + own <= 2` (:7013); passes 2 and 3 require `uses==1` (:7030, :7033) | Caps at 2. **No different-face check.** Comment at :7001-7004 shows the cap was added deliberately for the 4-copy non-manifold contact case. |
| P14 | `make_shared_section_edges` | brep.cpp:6531-6793 | gated `SESSION_BOOL_SHARED_EDGES` (:10394) | off by default |
| P15 | `CAPFILL` synthesised mating cap | brep.cpp:10618 | gated `SESSION_CAPFILL` | off by default |

**Which one made y30's 13?** The measured face incidence (§2.3) says P1/P2 inside a *single*
operand's `split_with`: 12 of 13 edges have exactly two distinct faces but three usages, with the
*same* face `#11271` appearing on all twelve, and `#6386` using one edge three times by itself. That
is a same-face double-traversal, which only P1, P2, P4, P7, P8 can produce — and only P1/P2 can then
admit the third trim without a cap. P5 remains the most dangerous *unguarded* path structurally (it
has no cap at all) and is the one to instrument next, but it is not what produced these thirteen.

**No pass anywhere in `boolean` detects or repairs a >2-trim edge.** `topology_report` counts them
(`brep.cpp:994-1001`) and `is_solid` rejects them (`:922-963`), but neither is called from `boolean`,
and the AUTO metric ignores them (§4.4).

---

## 4. Q4 — how a closed result gets the wrong volume

> **Measurement update — read this before acting on this section.** The question's premise was that
> y30 is a closed shell with a coherently mis-classified region. §2.4 refutes that: y30's large shell
> does not close at all in the current artefacts, its 32 faces match the reference's 32 one-to-one,
> and exactly one face is doubled (35.379663 vs 17.682530, ratio 2.0008). **There
> is no mis-classified region in y30.** What follows is therefore the *architectural* answer — how
> the code makes a coherent whole-region flip possible, and why nothing would catch it — not a
> diagnosis of y30. It stays in the audit because the mechanism is real, unguarded, and will produce
> exactly the described symptom the moment a seed goes wrong; §4.4 (no validity gate) is confirmed
> independent of any of this.

### 4.1 The classification architecture

`classify` is a 763-line lambda, `brep.cpp:8782-9544`, invoked twice (`:9670-9677`). It is **neither**
purely per-face **nor** purely propagated — it is a seven-stage cascade in which later stages can
overwrite earlier ones:

| stage | lines | nature |
|---|---|---|
| S0 micro-fragment drop | 8836-8882 | per-face, `md = 2*tol3` |
| S1 ON detection + sampling | 8883-8931 | **per-face, independent**. `in_other` = `contains_point_exact` (`:8799-8803`, definition `:1120-1218`) or `inside_prim` for recognized primitives. `K = 5` samples for freeform (`:8820`), majority via `vote_inside` (`:8821-8834`) |
| S2 angle method | 8932-9085 | **per-section-edge, local & analytic**. Transversality-weighted evidence `ang_sum/ang_abs`; overrides S1 when `conf >= 0.2` (`:9071-9084`) |
| S3 section-pair repair | 9086-9120 | pairwise: two flanks of one section edge must differ; re-vote at K=9, else flip the weaker |
| S4 connexity flood | 9121-9229 | union-find blocks: unite across every **non**-section edge (`:9162-9179`) plus cross-face border pieces gated on original adjacency and symmetric 7/9 coverage (`:9202-9229`) |
| **S5 parity** | **9265-9414** | **propagation.** See below |
| S6 block-majority fallback | 9415-9426 | for fragments S5 did not settle |
| S7 island repair | 9495-9541 | flips a fragment whose keep-state disagrees with **every** non-section neighbour |
| — | 9486-9494 | the op table: Union keeps `!inside`; Intersection keeps `inside`; Cut keeps `!inside` on A and `inside`+reversed on B |

Radial certification (`brep.cpp:9545-9669`) runs *before* `classify` and seeds it with `certified[]`
verdicts that S2/S3/S5/S6/S7 must not override (`:9080`, `:9104`, `:9406`, `:9529`).

### 4.2 The coherent-flip mechanism

S5 is exactly the mechanism the question asks for. It 2-colours the block graph (blocks = union-find
components; edges = **keyed** section edges, `:9278-9293`), then for each connected component `cid`
chooses **one phase bit** `P` and assigns

```
brep.cpp:9392-9393    pv[b] = (color[b] == 0) ? P : 1 - P;
brep.cpp:9401-9409    for every fragment fi in the component:
                          if (!certified[fi] && inside_v[fi] != pv[b]) inside_v[fi] = pv[b];
                          score[fi] = pv[b] ? 1.0 : 0.0;      // evidence overwritten
```

**If `P` is wrong, every fragment in the component flips together, the section-edge separation
invariant still holds everywhere (parity is preserved by construction), and the shell stays
watertight — it is simply the complementary region.** That is precisely "an entire REGION on the
wrong side yet still sewn consistently".

How `P` is chosen (`:9333-9389`):

- `SESSION_ANG_PRIMARY` (**off by default**, `:9343`) would seed from analytic angle evidence with
  `aconf >= 0.5`.
- Default: `ev_tot`/`ev_p1` are accumulated from **sampling-derived** `score[fi]` with
  `|score - 0.5| >= 0.25` (`:9357-9367`) — i.e. from `contains_point_exact`.
- If no evidence at all: one seed fragment is hardened with a K=9 winding re-vote (`:9373-9389`).
- Refusal gates: `agree*5 < ev_tot*4` ⇒ fall back (`:9371`); a block whose weighted majority is
  ≥80 % against `pv[b]` ⇒ fall back (`:9390-9400`); a component with an odd cycle or an
  `internal_bad` block ⇒ fall back (`:9317-9323`).

So the phase is decided by `contains_point_exact` (`brep.cpp:1120-1218`), a **closest-point-on-trimmed-
boundary + outward-normal-sign** test that samples each trim pcurve at only `min(max(cv_count,8),24)`
points in the boundary-fallback branch (`:1173`). On a thin-walled, deeply concave chair this is
exactly the classifier that is least reliable — and the code says so itself at `:8791-8796`
("chairsROT z90: f2 read inside 0.8 OUTSIDE B, f40 read outside 0.1 INSIDE B") and at `:9336-9342`
("documented 67–86 % A-side accuracy; z90 winding inversion").

`kb/commercial_kernel_doctrine_bridge.md` Law 5 predicts this exact outcome: *"a wrong seed shows up
as a globally wrong (volume-sanity-catchable) selection instead of one naked face."* y30 is that
sentence, measured. What Law 5 assumes and this kernel lacks is a volume-sanity gate that can catch
it (§4.4).

Two secondary coherent-flip routes exist:

- **S4 mis-partition.** If the flood unites two blocks *across* a section (a partial/unkeyed section
  piece collected as a non-section border edge at `:9162-9179`), the section constraint is dropped
  and one block spans both sides — its majority then paints a whole region uniformly. The code has a
  detector for this, `SESSION_FLOOD_AUDIT` → `[FLOOD-BRIDGE]` / `[FLOOD-AUDIT]` (`:9230-9255`), and
  it is **off by default**.
- **S6 block majority** (`:9415-9426`) paints every unsettled fragment of a block with one bit, with
  no section constraint at all.

### 4.3 Instrumentation that would prove or refute a coherent flip (cheapest first)

*Originally scoped to y30; §2.4 removed that target. Use this on the analytic reproducer of R3, or on
whatever cell R0 promotes — the procedure is unchanged, only the subject is.*

1. **Run the existing gates.** `SESSION_CLS_DBG=1` prints `[CLSF] <side> f<i> sp(x,y,z) on= ins= sc=`
   after every override (`brep.cpp:9479-9485`) and `[CLS-PARITY] comps= fallback= flipped= done=`
   (`:9411-9413`). `SESSION_FLOOD_AUDIT=1` prints `[FLOOD-BRIDGE]`/`[FLOOD-AUDIT]` (`:9235-9255`).
   `SESSION_NT_DBG=1` prints `[RADIAL] blocks= certA= certB= conflicts=` (`:9666-9668`) and
   `[SEGAUDIT]` per-segment kept-coverage asymmetry (`:9714-9741`).
2. **Oracle-diff the sample points.** Feed every `[CLSF]` sample point through
   `validation/occt_oracle` (`BRepClass3d` point classification against the same operand) and label
   each fragment agree/disagree. *Decisive test:* if the disagreeing fragments are exactly the
   fragments of one union-find component (one `cid`), the volume error is one flipped parity phase —
   not distributed sampling noise. Print `uf_find(fi)` and `compid[uf_find(fi)]` alongside `[CLSF]`;
   they are already computed at `:9152-9158` and `:9297`.
3. **Bisect the cascade with the existing switches** (each is a one-line env change, no rebuild):
   `SESSION_NO_PARITY=1`, `SESSION_ANG_PRIMARY=1`, `SESSION_NO_FLOOD=1`, `SESSION_NO_ISLAND=1`,
   `SESSION_NO_RADIAL=1`, `SESSION_NO_ANG=1`. Record result volume for each. If
   `SESSION_ANG_PRIMARY=1` alone moves y30 from 23.96 toward 46.96, the phase seed is the defect and
   the fix is a one-line default change.
4. **The one new probe worth adding** (kept out of scope here, recorded as a request): after
   `classify`, print per component `cid` the block list, `ev_tot`, `ev_p1`, the chosen `P`, and the
   summed |signed volume| of the fragments it keeps. Then re-run with `P` inverted for that component
   only. If inverting one component's phase reproduces 46.9596, the defect is proven and localised to
   one bit.

### 4.4 Why no invariant catches it

`brep.cpp:8027-8043`, the AUTO ladder's selection metric:

```cpp
auto metric = [&](BRep& r) -> int {
    int nk = 0;
    for (const auto& e : r.m_topology_edges)
        if ((int)e.trim_indices.size() == 1) ++nk;          // ONLY 1-trim counted
    if (nk == 0 && volAB > 0) {
        double v = r.volume();
        if (!std::isfinite(v) || v <= 0 || v > volAB * 1.001)
            nk = 1;
    }
    return nk;
};
```

- A 3-trim edge scores **0**. y30's thirteen are invisible to the optimiser.
- The volume gate is one-sided: it rejects `v > volA+volB` but accepts any positive `v` below it.
  Any under-count whatsoever scores a perfect 0 and wins the ladder.
- **A volume gate cannot catch y30 — but not for the reason first claimed here.** The retracted
  version said the doubled face has zero flux; it does not (73.257183 vs the reference's 73.257742).
  The real reason is simpler and firmer: **y30's large component is an open shell**, so its volume is
  undefined, and `r.volume()` returns a number for it anyway. The duplication registers in **area**
  (175.998851 vs 158.301420) and in **per-face validity**, never in a volume integral computed by
  point-masked quadrature, which cannot count a doubly-covered point twice.

**The gate that does work, with its false-positive rate measured on the verified reference:**

| check | catches in y30 | false positives on the reference solid |
|---|---|---|
| `face.isValid()` | **2 / 32 faces** (f3 area 35.379663; f24 area 9.729845) | **0 / 32** |
| outer wire is closed | **1 / 32** (f3, 24 edges, len 33.580661) | **0 / 32** |
| trimmed area ≤ untrimmed patch area | f3 at ratio **1.9806** (35.379663 vs 17.862850 — impossible) | 2 / 32, both at ratio 1.0000 (rounding; needs a relative tolerance) |
| Σ face area vs expected | +11.2 % | — |

Per-face validity is the standout: **zero false positives on the reference and it fires on both
defective faces.** Note this also reveals a second broken face in y30 (f24, `isValid()==False`,
area 9.729845) that the wire-repeat analysis alone did not surface. The kernel has no equivalent
check anywhere — `is_solid()` (`brep.cpp:910-965`) inspects trim counts only and never looks at a
wire.
- `is_solid()` (which *does* reject >2-trim, `:922-963`) is never called in `boolean`.

So the ladder at `:8060-8078` is an argmin over a metric that is blind to both defect classes present
in y30. This corroborates `kb/DECISION_architecture.md`'s Step 0 recommendation
(`naked == 0 && nonman == 0` + volume sanity) — with the added measurement that the volume half of
that gate must be a *cross-op partition identity* (`vol(cut) + vol(common) == vol(A)`), because a
single-body bound cannot separate 23.96 from 46.96.

---

## 5. Q5 — ranked defect list

Ranked by how many of the 26 open/wrong cells each mechanism plausibly explains. "Cells" = the 30
rotated chair cells (10 configs × 3 ops).

**The measurements split the failures into two disjoint families**, and the cheapest discriminator is
the *naked-edge length distribution*, already computable from any exported result:

| family | signature | cells | defect |
|---|---|---|---|
| **degenerate wire** | few naked edges, all micro, all in one small blob; faces with `isValid()==False`, an unclosed outer wire, and trimmed area exceeding the untrimmed patch | y30 (58 naked, total len 0.9976, one 0.09 blob; 2 invalid faces), z30 (10 / 1.4181) | R1 |
| **lost regions** | many naked edges, individually long; reference faces with no counterpart at all | z15 (17 / 20.3221, 18 ref faces / 53.351 area missing), z37 (24 / 24.4871), z45 (20 / 23.4810, 4 ref faces / 11.1864), z63 (10 / 12.2245), x20 | R2 |

Fixing R1 does nothing for the second family and vice versa. Previous campaigns ranked one remedy
across both, which is why the frontier moved in fits.

---

### R0 — The scoreboard has two wrong targets; fix them before ranking anything
**Status: measured, not speculative.** The stored references are stable — they recompute to four
decimals (Appendix A) — but stability is not correctness. For x13y29 the reference `A.cut(B)` omits a
1.41383024 lump that is provably in `A \ B` (`chunk.common(A)` = full, `chunk.common(B)` = 0, 40/40
interior points in A and outside B), and OCCT's partition identity still closes because the lump is
double-booked into `common`. Our kernel produces that lump correctly. **So two of ten cut cells are
currently scored against a target that is wrong in solid count, and every metric built on
`REFERENCE_*` inherits it.**
**Consequence for the census:** neither detached component in this corpus is a defect — the 4-face
chunk (§2.1) matches the reference exactly, and the 10-face chunk (§2.2) is right where the reference
is wrong. Any census counting either as a failure is mis-ranking remediation.
**Cheapest confirmation (one afternoon, no kernel involvement):** classify ~10³ stratified random
points in the chunk's bbox against A and B with a **second, independent kernel** — Rhino headless is
already wired up (`validation/rhino_headless_probe.py`). OCCT has now answered this question twice by
two different code paths (solid booleans and `BRepClass3d` point membership) and agreed with itself
both times, so the remaining risk is a systematic OCCT error, which only a different kernel can
exclude. If Rhino agrees, mark `REFERENCE_x13y29_cut` (and y30, once measured — see below)
**untrusted for solid count** and record the expected answer as two solids.
**y30 now verified too**, by the same operand-only chain:

```
y30  chunk 10 faces, closed solid, vol 0.73320328
     chunk.common(A)=0.73320328  chunk.cut(A)=0.00000000   -> chunk ⊆ A
     chunk.common(B)=0.00000000  chunk.cut(B)=0.73320328   -> chunk ∩ B = ∅
     40/40 interior points: inA=40, inB=0, in verified REF cut=0
     chunk.common(refcut)=0.00000000   refcut.cut(chunk)=46.959623 (unchanged)
     chunk.common(BIG)=0.00000000 ; min chunk-edge -> big-edge distance 1.55 ; 0/24 edges within 0.05
```

Both cells now rest on measurements I re-derived myself. Neither is a connectivity defect: in y30 the
chunk stands **1.55 units** clear of the big component, in x13y29 **1.37**.

---

### R1 — Degenerate arrangement wires (lasso / slit) emitted without validation
**Explains: the whole of y30 — not part of it.** Measured: 32 of 32 faces match the reference 1:1,
one face is covered **2.0008×** (35.379663 vs 17.682530, exceeding even its untrimmed patch area of
17.862850) with an unclosed outer wire, `isValid()==False`, and 11 duplicated micro-holes; a **second**
face (area 9.729845) is also `isValid()==False`. Total area excess +17.697431 ≈ the doubled face. All 13
pre-import non-manifold edges (58 post-import naked edges, total length **0.9976**) sit in the single
0.09-wide blob around it, and all are section edges. Same signature drives z30 (10 naked, len 1.4181)
and produces the zero-area face and the 1-face stray components in y30/z30. Two of z45's faces are
also `isValid()==False` with open outer wires (areas 13.4749, 9.6412), so R1 is *co-present* there.
The comment at `nst:911-917` names z90, z45, x13y29 for the same trigger.
**Why rank it first:** it is the only defect with a fully closed causal chain from source line to
final volume, and it is upstream of R2 wherever both are present (a doubled face's edges cannot mate
with the other operand).
**Sites:** walk `nst:1177-1222` (no repeated-edge / self-intersection rejection); crossing
conditioning `nst:880-902` + `nst:904-928` (`SESSION_NODESNAP` off); mating without a face check
`brep.cpp:3969-4003`.
**Cheapest experiment (minutes, no rebuild):** re-run y30/z45/x13y29 cut with `SESSION_NODESNAP=0.02`
and re-parse the exported STEP for the edge-usage histogram plus the area of the face at centroid
≈ `(11.95, 4.09, 1.65)`. *Confirms* if the >2-usage count drops to 0, that face's area halves from
35.3797 to ≈17.6822, and its 11 micro-holes vanish; *refutes* if the histogram is unchanged. The
area-doubling is the sharpest single scalar in the whole hunt — it is a 2.0009 ratio against a known
reference number, so it cannot be argued with.
**Second experiment (one probe, no behaviour change):** after `cycle_to_loop`, count how many cycles
contain a repeated `eidx`, print `[WIRE-SLIT] si=.. cyc=.. rep=..`. A non-zero count on rotated and
zero on base chairs isolates the class in one run.

---

### R2 — One-sided section-run loss (SEGLOST / class A + dependent B)
**Explains:** the entire second failure family, now with hard face-level numbers — **z15 is missing
18 reference faces totalling 53.351 of area** (largest 14.2918 @ (9.0334,4.5255,0.5372); 11.7583 @
(10.2794,3.1043,−0.6872); 8.6920 @ (8.1654,2.8852,1.2544)), **z45 is missing 4 totalling 11.1864**,
and both carry long naked edges (z15: 17 edges, total 20.3221, longest 2.9349; z45: 20 edges, total
23.4810, longest 3.5570) rather than micro ones. Also z37 (24 / 24.4871), z63 (10 / 12.2245), x20.
It is the largest single census class — 71 A + ~55 dependent B of 174 edges
(`kb/p1_attack_plan.md:8-9`). It does **not** explain y30 (§2.4).
**Sites:** D1 `brep.cpp:4176-4181`; D2 `nst:688-693` + cap `brep.cpp:4081-4082`; D4 prune
`nst:1128-1147`; D7 JOINFAIL `nst:1498-1519` → `brep.cpp:4671`; D9 zero-span drop
`brep.cpp:3721-3733`.
**Cheapest experiment:** `SESSION_NT_DBG=1 SESSION_SPLIT_DBG=1` on y30 cut; count `[SEGLOST]`
(`brep.cpp:8413`), `[SCAF-FALL]` (`:3857-3859`), `[JOINFAIL]` (`nst:1507`), `[SEGFALL]`
(`nst:1374-1376`), and `[SEGAUDIT]` asymmetric segments (`:9738`). Then flip the four opt-in
mitigations one at a time — `SESSION_SECPROTECT=1`, `SESSION_ZEROFILL=1`, `SESSION_SEGKEEP=1`,
`SESSION_STUB_REACH=1` — and record `[SEGAUDIT]` asymmetry count + final face count vs the reference's
40/37/38. Any single flag that drives `[SEGAUDIT]` to zero identifies the dominant sub-mechanism.

---

### R3 — Whole-region selection inversion (parity phase / block majority) — **DEMOTED, no witness**
**Status:** this was ranked 3rd on the strength of the 10-face chunk. That evidence has been
withdrawn: §2.4 shows y30's 32 faces match the reference's 32 one-to-one with no missing region, and
§2.2 proves the chunk is correct material of `A \ B` that the reference omits. **There is currently
no measured instance of a coherent whole-region inversion anywhere in
the corpus.** The mechanism (`brep.cpp:9392-9409`, one phase bit per block component, unguarded, seeded
from a sampler the code itself documents at 67–86 % accuracy) is real and unguarded, so it stays on
the list as a latent risk — but it explains **0** of the 26 cells on present evidence, and effort
spent on it now is speculative.
**Explains:** nothing measured. The 10-face chunk, which was its only candidate witness, has since
been verified *correct* (§2.2) — so R3 has lost that evidence outright rather than merely having it
questioned.
**Sites:** `brep.cpp:9333-9409` (phase seed and propagation); seed oracle
`contains_point_exact` `brep.cpp:1120-1218`; `SESSION_ANG_PRIMARY` off at `:9343`.
**Cheapest experiment — a regression guard, not a hunt.** Do not chase this on chairs. Instead prove
the mechanism can misfire at all, on a case where truth is free: two nested boxes, or a box cut by a
box sharing a face, where the correct verdict of every fragment is known analytically. Run with
`SESSION_CLS_DBG=1` and check that `[CLS-PARITY]` never reports `fallback` and that every `[CLSF]`
verdict matches the analytic answer. If it does misfire there, R3 becomes a real defect with a
minimal reproducer; if it does not, R3 can be closed as a theoretical concern and the audit's
attention stays on R1/R2.
**Only if R0 reinstates it:** the wrong region is already localised — chunk centre of mass
`(7.63970, 4.03474, -1.15884)` for y30, `(7.78283, 4.01846, -1.01672)` for x13y29. Find the `[CLSF]`
lines whose sample point falls in the chunk's tight bbox (`x[6.6686,9.2275] y[2.5517,4.5051]
z[-1.4381,-0.8404]`), then check whether `[CLS-PARITY]` reports their component as settled rather
than `fallback` — if settled, one phase bit explains the whole chunk. The per-component dump in
§4.3 item 4 proves it exactly.

---

### R4 — Cross-operand alias with no manifold guard
**Explains:** structurally, every case where a classification error keeps both flanks; it is the
single most dangerous unguarded write in the pipeline even though it is *not* what produced y30's 13.
Likely contributor in configs where the non-manifold edge's two faces come from different operands.
**Site:** `brep.cpp:9761-9788` — `b_edge_new[j] = itA->second` then `:9802-9806` re-points every subB
trim, with **zero** inspection of either edge's trim count; `qspan` (`:9694`) quantizes spans to 1/32
so distinct blocks collide.
**Cheapest experiment:** with `SESSION_NT_DBG=1`, `[XNM]` already prints every ≥3-trim edge with the
operand of each incident face (`brep.cpp:10301-10316`). If any `[XNM]` line mixes `A:frag` and
`B:frag`, R4 fired; if all are same-operand, R4 is refuted for that cell. Zero code changes.

---

### R5 — Merge passes with no "different face" predicate
**Explains:** a slit can be *created* (not just propagated) by any of the five merge passes, turning
a correct pair of 1-trim edges on one face into a same-face 2-trim edge that then takes a third trim.
Suspected secondary contributor wherever R1's arrangement output was actually fine.
**Sites:** P4 `brep.cpp:4727-4777`, P7 `:10059-10099`, P8 `:10132-10181`, P13 `:7008-7016` — all group
by vertex pair + geometric tube, none checks that the two edges belong to different faces (or even
different loops).
**Cheapest experiment:** with `SESSION_BEMAP_DBG=1` / `SESSION_NT_DBG=1`, `[CBLOCK]` and `[XWELD]`
print merge counts. Cross-check: for each merged pair, are both edges' single trims in the same
`loop_index`? A one-line diagnostic print (no behaviour change) answers it; a non-zero same-face merge
count on rotated and zero on base chairs confirms.

---

### R6 — Coincidence-drop rules that assume transversality
**Explains:** the specific cells where a section runs along a trim (the `on_border` class). Bounded:
at most the configs where the graze is a whole segment.
**Sites:** `brep.cpp:4176-4181` (scaffold), `:4438-4459` and `:4460-4499` (legacy), `nst:647-659`.
**Cheapest experiment:** `SESSION_KEEP_BORDER=1` on y30 + z45; count `[SEGLOST]` before/after and the
final face count. The comment at `brep.cpp:4173-4175` predicts the trade ("at a graze along the
boundary the section must still carve the thin strip … else the other operand's rim meets an UNSPLIT
2-trim edge = nonmanifold (y30)") — this measures whether the prediction holds.

---

### R7 — A face with an inner loop can never be split
**Explains:** 0 of the 26 chair cells (measured: both chair operands have 0 inner bounds). Listed
because it is a hard, unconditional correctness hole for any operand with a through-hole, and because
the boolean *creates* inner loops (11 of them on y30's face `#11271`), so a second boolean on a
boolean result hits it immediately — i.e. it blocks the A-op-A acceptance gate that
`kb/DECISION_architecture.md` assigns to Law 1.
**Site:** `brep.cpp:4350-4358`; `all_pcs` excludes `inner_loops` at `:4558-4560`.
**Cheapest experiment:** `create_block_with_hole(...)` (`brep.cpp:458`) cut by a box that crosses the
hole; assert the hole face splits. Two lines in an existing test harness, no chair data needed.

---

### R8 — No validity gate anywhere in `boolean`, and an AUTO metric blind to both defect classes
**Explains:** why R1–R5 survive to the output. Not a cause, but it is why the campaign's 196→108
numbers are an argmin over a metric that scores a slit-faced, non-manifold, half-volume shell as
perfect.
**Sites:** metric `brep.cpp:8027-8043`; `is_solid` `:910-965` never called from `boolean`;
`topology_report` `:968` diagnostic only.
**Cheapest experiment:** none needed — it is a read of the code. The change is the one
`kb/DECISION_architecture.md` already schedules as Step 0, with one addition this audit forces:
the volume half of the gate must be the **cross-op partition identity** `vol(cut) + vol(common) ==
vol(A)` summed over **all** solids of each result, because no single-body volume bound can separate a
correct answer from an under-count.
**Two additions this audit forces beyond that**, both measured above:
1. **Per-face validity is the highest-value check and the cheapest** — 0 false positives on the
   reference, fires on both of y30's broken faces. Concretely: outer wire closed, no edge traversed
   twice within a wire, trimmed area ≤ untrimmed patch area (relative tolerance), face `isValid()`.
2. **The partition identity is necessary but not sufficient, and it is not a correctness oracle.**
   OCCT's own x13y29 partition closes to 0.000391 while omitting a 1.41383024 lump from `cut` and
   double-booking it into `common` (§2.2). A consistent mis-assignment is invisible to it. Pair it
   with direct point-membership spot checks against the operands.

---

## 6. Corrections this audit forces on existing kb material

1. **`kb/DECISION_architecture.md`** quotes the AUTO metric as counting only `trim_indices.size()==1`.
   Still true at HEAD, but the metric has since gained a volume-sanity clause
   (`brep.cpp:8034-8039`). That clause does **not** catch y30: it only rejects `v > volA+volB`.
2. **The "4-face detached chunk" must be struck from the defect list.** It is the correct second
   solid — symmetric difference against the reference's second solid is **0.00000000 both directions
   in all five configs** (z15 0.02773248 vs 0.02773424; z30 0.09102301 vs 0.09102214; z37 0.10024918
   vs 0.10024042; z45 0.08778172 vs 0.08777193; z63 0.01637244 vs 0.01637083) — and the kernel's own
   `validate_oracle.sh:20-22` already documents `OP_SOLIDS = 2` for z15/z30/z45 cut. Any census that
   counts it as a failure will mis-rank its remediation.
2b. **The "26 of 30 are open shells / closed-but-wrong" framing needs re-taking.** On the current
   artefacts y30's large shell does not close at all (`Part.Solid` fails; `sewShape`/`fix` at
   1e-3…0.2 all fail), so the 23.9619 figure comes from a different build. Re-measure before
   ranking anything against it.
2c. **The 10-face chunk must be struck too, and two reference cells marked untrusted for solid
   count.** The stored references are stable (they recompute to four decimals, Appendix A), but the
   x13y29 reference cut reproducibly omits a 1.41383024 lump that operand-only tests place in
   `A \ B`; our kernel produces it correctly. Conclusion: **neither** detached component in this
   corpus is a defect. y30's chunk has the same signature but is not yet re-verified by the same
   chain — do not assert it until it is (R0).
3. **The correct per-cell face targets** (cut) and our deltas on the *large* component:

   | cfg | reference | ours | large-component delta | extra artefacts |
   |---|---|---|---|---|
   | x20 | 38 | 33 | **−5** | — |
   | y30 | 32 (+ chunk, see R0) | 33 + 10 + 1 | +1 | 1-face orphan (spurious); 10-face chunk likely correct |
   | z15 | 40 + 4 | 34 + 4 | **−6** | — (4-face chunk correct) |
   | z30 | 38 + 4 | 37 + 4 + 1 | −1 | 1-face orphan |
   | z30x20 | 32 | 34 | +2 | — |
   | z37 | 38 + 4 | 38 + 4 | 0 | — (face counts exact; only closure fails) |
   | z45 | 37 + 4 | 34 + 4 | **−3** | — |
   | z63 | 37 + 4 | 35 + 4 | **−2** | — |
   | z90 | 36 | 37 | +1 | — |
   | x13y29 | 29 **(reference is wrong: omits the 10-face lump)** | 30 + 10 | +1 | none — chunk verified correct |

   Two distinct signatures, confirmed by the naked-edge length distribution (§5): x20/z15/z45/z63
   (and z30 marginally) are **short** of faces in the large component with long naked edges — regions
   the split never produced (R2); y30/x13y29 and z90/z30x20 are **over** by one or two faces with
   micro naked edges in a single blob — degenerate wires (R1). Any remediation ranking must separate
   the two. z37 is the cleanest target of all: correct face counts, correct solid count, only closure
   failing — though note its naked-edge budget (24 edges, 24.4871 total) is R2-shaped, so its
   "correct face count" is a coincidence of two errors, not a near-miss.

---

## Appendix A — reference truth measured during this audit

OCCT reference results (`/home/petras/fc_inspect/REFERENCE/`), solids / volumes / face counts:

```
y30      cut    1 solid   46.958863                        faces [32]
         common 2 solids  33.121509 + 0.216298             faces [25, 6]
         fuse   1 solid  127.255797                        faces [54]
x13y29   cut    1 solid   48.472832                        faces [29]
         common 3 solids  31.260179 + 0.243149 + 0.320731  faces [21, 5, 5]
         fuse   1 solid  128.769892                        faces [50]
z15      cut    2 solids  80.269405 + 0.027734             faces [40, 4]
         common 0 solids  (empty -- oracle-untrusted cell)
         fuse   1 solid   80.952059                        faces [43]
```

**These stored files ARE reproducible.** Recomputed from the canonical operands
(`operands/chair0.stp` + `operands/B_<cfg>.step`, each 20 faces / 1 closed valid solid /
vol 80.2968–80.2969) under FreeCAD 1.1.1 / OCCT 7.8.0:

```
cfg      A.cut(B) recomputed now              stored reference            delta
x13y29   48.473392  (1 solid, 29 faces)       48.472832                   0.000560
y30      46.959623  (1 solid, 32 faces)       46.958863                   0.000759
z15      80.269517 + 0.027734 (2 solids)      80.269405 + 0.027734        0.000111
```

An earlier revision of this document asserted the opposite for x13y29 (a recomputed 79.732960),
sourced from a delegated measurement I did not re-derive. **That is retracted** — it is ≈ vol(A)
minus a sliver, the signature of a wrongly-posed operand B. The rule that failed here: verify the
operand before trusting a number that contradicts a stable reference.

**Stable is not the same as correct**, though — see §2.2 and R0: the x13y29 reference cut reproducibly
omits a 1.41383024 lump that operand-only tests place squarely in `A \ B`, and its partition identity
`cut + common = vol(A)` (48.473392 + 31.823906 = 80.297298 vs 80.296907, residual 0.000391) closes
anyway because the lump is double-booked into `common`. A partition identity cannot detect a
consistent mis-assignment; only a direct membership test can.

Note also that **every surface in the entire corpus is a degree-3 `BSplineSurface`** — 20 per
chair operand, zero planes/cylinders/cones/spheres/tori — so every analytic fast path in the kernel
(`recognize_solid`, `inside_prim`, the whole primitive ladder) is bypassed on this corpus, and the
45/45 primitive matrix exercises none of the code that fails here.

Two consequences for how results are scored:

- `cut + common == A` holds on the reference as a *multi-solid* identity
  (y30: 46.958863 + 33.337808 = 80.296671 = vol(A)). Any partition-identity gate must sum **all**
  solids of each result, not the largest.
- z15 `common` is empty, so a "volume matches" gate must special-case it — `validate_oracle.sh:17-22`
  already detects this ("a config is oracle-untrusted iff OCCT's OWN boolean is not valid").

## Appendix B — reproduction

```bash
# git-stable snapshot (what this audit read)
cd /home/petras/code/code_rust/session/session_cpp
git show HEAD:src/brep.cpp                 > /home/petras/split_audit/brep_HEAD.cpp
git show HEAD:src/nurbssurface_trimmed.cpp > /home/petras/split_audit/nst_HEAD.cpp
git show HEAD:src/brep_section.cpp         > /home/petras/split_audit/brep_section_HEAD.cpp
# sha 5bb685aac3ff8f6a4922aaa2c34931702fac0004

# shell / component structure of any result vs its reference (pure python, no FreeCAD)
#   parse #id = ENTITY(args); build the entity graph; walk CLOSED_SHELL/OPEN_SHELL -> faces
#   -> bounds -> edge loops -> ORIENTED_EDGE -> EDGE_CURVE; histogram the EDGE_CURVE usage.
# Inputs: /home/petras/fc_inspect/verify_now/res_<cfg>_<op>.step   (3 bodies: A, B, result)
#         /home/petras/fc_inspect/REFERENCE/REFERENCE_<cfg>_<op>.step
```

Debug switches used or recommended above, all read via `std::getenv` in the snapshot:
`SESSION_CLS_DBG`, `SESSION_NT_DBG`, `SESSION_NT_DBG2`, `SESSION_SPLIT_DBG`, `SESSION_ARR_DBG`,
`SESSION_ARR_DUMP`, `SESSION_BEMAP_DBG`, `SESSION_RUN_DBG`, `SESSION_TRIM_DBG`, `SESSION_SEW_DBG`,
`SESSION_SOLID_DBG`, `SESSION_FLOOD_AUDIT`, `SESSION_BOOL_PROFILE`, `SESSION_DUMP_SPLITS`.

Behaviour switches referenced (all default OFF unless noted): `SESSION_NODESNAP`, `SESSION_SECPROTECT`,
`SESSION_ZEROFILL`, `SESSION_SEGKEEP`, `SESSION_STUB_REACH`, `SESSION_KEEP_BORDER`, `SESSION_SE_OTHER`,
`SESSION_ANG_PRIMARY`, `SESSION_CLS_FIX2`, `SESSION_CUTDEDUP`, `SESSION_BOP2`, `SESSION_M3`,
`SESSION_SYMEMIT`, `SESSION_SYMLIFT`, `SESSION_CAPFILL`, `SESSION_WIRE_REPAIR`, `SESSION_FUZZLADDER`,
`SESSION_WIRESPLIT`, `SESSION_TRIM_SNAP`, `SESSION_BND_SNAP`, `SESSION_BOOL_SHARED_EDGES`;
and the "disable" switches `SESSION_NO_{PARITY,FLOOD,ANG,ISLAND,RADIAL,EXACT_PIP,EXACT_PIP2,ON,
MICROFRAG,PAVESNAP,XWELD,NKRESCUE,COREFINE,SEW,BLOCKS,SCAFFOLD,ISLAND,P4SPLIT,EFGATE}`.
