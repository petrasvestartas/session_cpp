# brep.cpp subsystem audit (11,285 lines, audited 2026-07-24 @ HEAD 4544770f)

Scope: `src/brep.cpp` only. The scaffold builder (`build_section_scaffold`,
`refine_scaffold_at_breaks`, `build_shared_edge_pool`) lives in `brep_section.cpp`
(41 further `getenv` sites, not inventoried here). `SESSION_BND_SNAP`'s actual read
is in `nurbssurface_trimmed.cpp:744`; brep.cpp only mentions it.

---

## 1. Subsystem inventory (line ranges)

### A. Construction / factories (26–883)
| Block | Lines |
|---|---|
| ctors, copy, ==/!=, deep_copy_from | 26–70 |
| create_box | 76–183 |
| create_cylinder | 185–271 |
| create_sphere (on-surface seam meridian) | 273–333 |
| create_cone (singular apex) | 335–401 |
| create_torus (2 seams, no poles) | 403–456 |
| create_block_with_hole | 458–642 |
| from_polylines | 644–744 |
| from_nurbscurves (+holes) | 746–883 |

### B. Queries / validity / classification primitives (885–1375)
| Block | Lines | Notes |
|---|---|---|
| face/edge/vertex_count, is_valid | 889–908 | |
| is_solid (degenerate + orphan-edge scoping) | 910–966 | SESSION_SOLID_DBG |
| topology_report (naked/nonman/shells/chi/dupV) | 968–1083 | SESSION_ORPHAN_DBG |
| contains_point (generalized winding number, mesh) | 1085–1118 | replaced 7-ray parity (comment 1088–1091) |
| contains_point_exact (trimmed closest-point + outward sign) | 1120–1218 | SESSION_NO_EXACT_PIP2 boundary fallback default-on |
| check_trim_orientation (chain gaps + loop sense) | 1222–1294 | |
| loop_material_left (UV left/right probe vote) | 1297–1375 | |

### C. face_outward_signs — shell orientation engine (1377–1987)
| Sub-block | Lines |
|---|---|
| face_interior CDT probe (largest in-material triangle) | 1394–1473 |
| PASS 0 orientation propagation: trim_dir (anchored material-left) | 1564–1628 |
| adjacency rel (geometric anti-parallel-mates; SESSION_TOPO_REL alt) | 1629–1661 |
| BFS relsgn propagation per component | 1662–1676 |
| SESSION_ORIENT_REPAIR neighbour-majority relaxation (gated OFF) | 1677–1700 |
| SESSION_FRUST_DBG frustrated-edge diagnostic | 1701–1719 |
| per-component probe clouds + cavity detection (per-comp meshes) | 1720–1789 |
| evidence TIER 1 comp-local supporting plane (w=5) | 1797–1817 |
| evidence TIER 2 global supporting plane (w=3) | 1818–1833 |
| evidence TIER 3 signed volume flux (rotation-invariant; replaced parity double-probe as primary) | 1834–1876 |
| evidence TIER 4 parity double probe (w=1/0.5, last resort) | 1877–1887 |
| component score flip + SESSION_SHELL_ORIENT per-comp re-vote (gated OFF) | 1890–1981 |

### D. volume() — divergence-theorem flux (1989–2633)
| Sub-block | Lines |
|---|---|
| loop_vector_area (knot-aligned per-span Gauss-5) | 2016–2098 |
| SESSION_VOL_GUARD open-shell warning | 2101–2135 |
| quadric recognition hoist (sphere/cyl/torus) | 2147–2222 |
| planar flux (loop vector areas) | 2226–2255 |
| masked Gauss fallback (NU 24 rect / 384 masked; skipped when analytic) | 2256–2350 |
| analytic quadric boundary-integral flux (flag-driven, pole runs, v-locks) | 2351–2621 |
| final sum, SESSION_VOL_DBG | 2622–2633 |

### E. Builders + split entry points (2635–2973)
| Block | Lines |
|---|---|
| add_surface/curve/vertex/edge/trim/loop/face | 2639–2701 |
| anon helpers: q6 quantizer, aabb_from_surface/curve, uv_in_polys | 2707–2774 |
| split_by_plane / _surface / _curves / _line | 2776–2819 |
| subset (face extraction w/ edge_remap) | 2821–2896 |
| split_by_plane_pieces | 2898–2930 |
| append_brep (disjoint assembly — xor path) | 2932–2973 |

### F. split_by_brep (2975–3266)
| Sub-block | Lines | Notes |
|---|---|---|
| trim-aware cutting (s_trimcut) — SSI cache, clip_to_face, fragment chaining | 2988–3242 | on only via SESSION_TRIM_CUT or SESSION_BOOL_SHARED_EDGES |
| pre_cuts consumption (shared pair-SSI, legacy freeform) | 3243–3255 | |
| legacy per-cutter-surface SSI + UV dedup | 3027–3067, 3256–3264 | |

### G. split_with — the operand splitter (3268–4835)
| Sub-block | Lines | Notes |
|---|---|---|
| BOP2 arena pre-seed (pool verts/edges into result) | 3277–3288 | pool != null |
| lift_loop (adaptive 3D lift, arc-length midpoint) | 3300–3364 | |
| pave collection: scaffold verts / SESSION_EF_PAVES endpoints | 3366–3426 | EF_PAVES opt-in, regressed cone×cyl |
| pave capture + original-corner capture + seed unification (MakeSDVertices) | 3427–3502 | SESSION_NO_PAVESNAP kill |
| find_or_add_vertex (capture ladder + 3×3×3 tolerant weld) | 3503–3574 | |
| boundary-run identity: edge_paves (MakeSplitEdges), bemap, edge_dev tubes | 3576–3657 | |
| **append_face**: chain-lift section runs (zero-span 3722–39, P4REF pool ref 3772–99, whole-seg alias key 3800–53) | 3696–3863 | SESSION_ZEROFILL |
| append_face: boundary-run pave subdivision + keying (B/L kinds) | 3864–4004 | |
| face loop: scaffold cut prep — overshoot stubs, forced nodes, border drop | 4046–4263 | SESSION_KEEP_BORDER, SESSION_STUB_REACH |
| PutSEInOtherFaces (SESSION_SE_OTHER, opt-in, measured regression) | 4264–4332 | |
| same-domain extra_cuts (seg id −1 → legacy lift) | 4333–4343 | |
| legacy path: clip-to-chart, border-coincident drops | 4360–4499 | non-scaffold only |
| SESSION_TRIM_SNAP boundary bridges (legacy only; `scaf ? nullptr`) | 4500–4557 | superseded by scaffold stubs |
| arrangement call: split_by_uv_curves (default) / SESSION_WIRESPLIT | 4558–4589 | snap_bnd=0.05 freeform legacy |
| pass-through faces via pave-block path; per-part lift | 4631–4683 | |
| within-operand common-block pass (PerformEE, tube tolerance, S-survivor) | 4692–4788 | |
| NK1 debug + vertex-edge adjacency rebuild | 4789–4834 | SESSION_BEMAP_DBG |

### H. Repair / identity passes (4837–7367)
| Block | Lines | Notes |
|---|---|---|
| imprint_edges (T-junction split at on-edge vertices; mated_too = BOP2 mode) | 4837–5070 | |
| anon: exact_arc_3d, sphere/cylinder_of_surface, circle_through_points | 5072–5185 | |
| **co_refine_coincident_edges**: split-point discovery | 5244–5277 | |
| — exact conic fit: vertex circle / sphere-sphere radical / Steinmetz (gated) | 5279–5421 | SESSION_BOOL_ELLIPSE gated OFF |
| — pave dedup, split, wrap-join, exact-arc rebuild + propagation | 5423–5541 | |
| — per-trim pcurve split (branch-free param) + affine planar rebuild | 5556–5746 | |
| — FINAL PASS: unsplit closed circle → exact rational circle | 5748–5778 | |
| — FINAL PASS 2: conic registry + SameParameter-guard relift + upgrade sweep | 5779–6096 | registry derives Steinmetz UNGATED (5932–68) — partially supersedes the 5354 gated block |
| recover_section_spans (opt-in SESSION_RECOVER) | 6105–6180 | "residual lacks mateable copies" |
| normalize_section_blocks (pave/common-block on shared chains, shared_centers) | 6182–6529 | |
| make_shared_section_edges (P0/P1 pave engine: PerformFF/PostTreatFF port) | 6531–6792 | opt-in via SESSION_BOOL_SHARED_EDGES; early-returns the boolean |
| snap_section_edges (legacy freeform: snap under-mated copies to pair-SSI c3d) | 6794–6871 | |
| **sew_coincident_edges**: micro-edge collapse | 6952–7000 | |
| — P1 Hausdorff merge (2-trim cap) | 7001–7016 | |
| — P2 endpoint-anchored 2.5× relax | 7017–7049 | |
| — P3 mutual-best 8× with separation | 7050–7095 | |
| — post-merge orphan collapse (FixSmall) + loop splice | 7129–7222 | SESSION_NO_ORPHAN |
| — geometry rep upgrade (densest/rational wins) + compaction | 7223–7276 | |
| sameparameter_planar_pcurves (OFF at callsite, SESSION_SAMEPARAM) | 7279–7367 | |
| merge_coplanar_faces (display-only merge; 5000-trim planner cap) | 7369–7734 | |

### I. Recognition (7736–7993)
PrimSolid; recognize_solid: convex polyhedron 7796–7810, sphere 7814–7840,
torus (kind 4, fixed tor×tor hijack) 7841–7907, cylinder 7909–7965; inside_prim 7969–7992.

### J. boolean() — the pipeline (7995–10484)
| Stage | Lines | Notes |
|---|---|---|
| recognize operands; imported_freeform | 8006–8019 | SESSION_REC_DBG |
| legacy shared pair-SSI (S1; imported_freeform && !scaffold_eligible, e.g. cone×cone) | 8047–8076 | |
| scaffold build + eligibility (deg≥3 freeform present; SCAFFOLD_ALL; NO_SCAFFOLD) | 8023–8097 | |
| BOP2 pool mint (SESSION_BOP2) | 8098–8113 | |
| run_splits: B first → ON-imprint of B2 coincident loops onto A → A split | 8114–8253 | re-entrant |
| P1c SYMEMIT re-imprint+resplit (opt-in; superseded) | 8254–8333 | |
| P1c-DIRECT SYMLIFT fragment-split (opt-in; "CORRECT replacement for SYMEMIT") | 8334–8441 | |
| preserve-identity REFINE loop (opt-in / forced by BOP2; measured no-op on base) | 8442–8489 | |
| P4 MakeSplitEdges on operand results (BOP2; NO_P4SPLIT) | 8490–8498 | |
| SHAREDPAVE shared centers + per-operand normalize_section_blocks | 8499–8547 | NO_BLOCKS kill |
| **classify lambda**: face_sample/face_samples CDT probes | 8557–8677 | |
| — ON detection (scale-invariant ε/16 recheck) + OCCT pair rules | 8679–8848 | NO_ON, NO_EXACT_PIP |
| — MICROFRAG collapse | 8753–8799 | NO_MICROFRAG |
| — angle method (GetFaceOff; transversality-weighted) | 8849–9002 | NO_ANG |
| — section-edge consistency repair (K=9 re-vote, weakest flip) | 9003–9037 | |
| — connexity flood (symmetric ≥7/9 coverage unions; orig-adjacency gate) | 9038–9172 | NO_FLOOD, FLOOD_AUDIT |
| — block votes + seeded BFS parity (ANG_PRIMARY seeding opt-in) | 9173–9331 | NO_PARITY |
| — fallback block majority; CLS_FIX2 (measured harmful, OFF) | 9332–9390 | |
| — keep rules per op; island/hole repair (monotone) | 9403–9458 | NO_ISLAND |
| radial pre-pass (Phase-5 certification at section blocks) | 9462–9586 | NO_RADIAL |
| classify(A2), classify(B2) | 9587–9594 | |
| subset selection + revB; span alias tables (qspan 1/32) | 9596–9626 | |
| SEGAUDIT | 9628–9658 | NT_DBG |
| combine: BOP2 index alias → span alias → append | 9659–9739 | |
| xweld: exact 1e-7 weld + cross-operand common-block (0.6·tol3 tube) | 9740–9841 | NO_XWELD |
| NK-RESCUE tolerant mate-pair (0.15·tol3 endpoints) | 9842–9911 | NO_NKRESCUE |
| micro-edge orphan collapse (pin-hole diameter rule) + rebuild | 9913–10043 | |
| cross-operand normalize_section_blocks("R") (+opt-in RECOVER) | 10055–10068 | |
| imprint_edges @ pave tol | 10070–10077 | |
| snap_section_edges (legacy freeform only) | 10078–10084 | NO_SECSNAP |
| P0 make_shared_section_edges branch (early return) | 10085–10093 | BOOL_SHARED_EDGES |
| co_refine → xweld2 → Hausdorff sew | 10094–10115 | NO_COREFINE, NO_SEW |
| FUZZY manual fallback | 10116–10151 | |
| FUZZLADDER auto-escalation (opt-in, cap 16) | 10152–10198 | |
| CAPFILL synthesize absent mating cap (gated OFF) | 10199–10324 | |
| WIRE_REPAIR loop-sense enforcement (gated OFF) | 10325–10380 | |
| NT_DBG2 / NK lineage dumps | 10381–10477 | |
| SAMEPARAM (gated OFF) | 10478–10482 | |

### K. Meshing / transforms / serialization (10486–11284)
mesh() 4-phase 10490–10681; face_meshes 10683–10689; face_meshes_q 10691–10874;
point_at/normal_at 10876–10894; transform/transformed 10896–10922;
jsondump 10949 / jsonload 11012 / file_json_* 11073–11092;
pb_dumps 11094 / pb_loads 11169 / pb_dump 11252 / pb_load 11258; str/repr/<< 11269–11284.

---

## 2. SESSION_* gate inventory (77 unique names, 168 getenv sites)

### 2a. Debug/print-only (26) — no behavior change beyond output
| Gate | Sites | Purpose |
|---|---|---|
| SESSION_SOLID_DBG | 930,950 | is_solid naked/orphan edge report |
| SESSION_ORPHAN_DBG | 1048 | single-face shell forensics |
| SESSION_SIGN_DBG | 1620–1894 | outward-sign propagation trace |
| SESSION_FRUST_DBG | 1704,1977 | frustrated-edge (odd-cycle) report |
| SESSION_VOL_DBG / VOL_DBG2 | 2627 / 2236 | per-face flux / loop point dump |
| SESSION_VOL_GUARD | 2108 | open-shell volume warning |
| SESSION_TRIM_DBG | 3129,4653 | trim-cut chaining + LIFTPART |
| SESSION_MARCH_DBG | 3249,3259 | pair-SSI cut counts |
| SESSION_BOOL_PROFILE | 3295,7997 | per-phase microseconds |
| SESSION_BEMAP_DBG | 3627–4824 | pave/boundary-edge-map forensics |
| SESSION_SPLIT_DBG | 3857–4691 | arrangement inputs/outputs |
| SESSION_RUN_DBG | 3953 | boundary run dump |
| SESSION_NT_DBG / NT_DBG2 | ~40 sites / 10381 | boolean pipeline naked-trim forensics |
| SESSION_SEW_DBG | 5267–7218 | co_refine/sew merge decisions |
| SESSION_COREF_DBG | 5624,5649 | pcurve split junction dump |
| SESSION_XCHK | 6075 | PAIRPASS/POLY curve audit |
| SESSION_MERGE_DBG | 7624 | coplanar-merge group plans |
| SESSION_REC_DBG | 8013 | operand recognition kinds |
| SESSION_ONDET_DBG | 8200 | same-domain coincidence bands |
| SESSION_CLS_DBG | 8844–9450 | classification verdict trace ([CLS]/[CLSF]) |
| SESSION_FLOOD_AUDIT | 9152 | flood-bridges-section audit |
| SESSION_DUMP_SPLITS | 8553 | pb_dump A2/B2 to dir |

### 2b. Kill switches — behavior ON by default (20)
| Gate | Site | Disables |
|---|---|---|
| SESSION_NO_EXACT_PIP2 | 1183 | contains_point_exact trim-boundary fallback |
| SESSION_NO_PAVESNAP | 3433 | pave/corner capture + common-block pass in split_with |
| SESSION_NO_ORPHAN | 7138 | sew post-merge orphan collapse |
| SESSION_NO_SCAFFOLD | 8028 | scaffold → legacy per-operand imprint+sew |
| SESSION_NO_BLOCKS | 8117 | all normalize_section_blocks calls |
| SESSION_NO_P4SPLIT | 8494 | BOP2 operand imprint (only under BOP2) |
| SESSION_NO_EXACT_PIP | 8714 | exact classifier → winding mesh contains |
| SESSION_NO_ON | 8814 | ON-fragment handling (experiment) |
| SESSION_NO_MICROFRAG | 8759 | micro-fragment collapse |
| SESSION_NO_ANG | 8857 | angle method |
| SESSION_NO_FLOOD | 9044 | connexity flood |
| SESSION_NO_PARITY | 9190 | BFS parity propagation |
| SESSION_NO_ISLAND | 9425 | island/hole repair |
| SESSION_NO_RADIAL | 9469 | radial certification pre-pass |
| SESSION_NO_XWELD | 9746 | cross-operand weld/common-block/NK/micro |
| SESSION_NO_NKRESCUE | 9848 | NK tolerant mate-pair rescue |
| SESSION_NO_SECSNAP | 10081 | legacy snap_section_edges |
| SESSION_NO_COREFINE | 10099 | co_refine at combine |
| SESSION_NO_SEW | 10114 | Hausdorff sew (BOP2 measurement gate) |
| SESSION_NO_FUZZLADDER | 10160 | **COMMENT-ONLY — never read.** Code gate at 10161 is positive `SESSION_FUZZLADDER`; the ladder is opt-IN, comment is stale. |

### 2c. Opt-in experiments / alternates — OFF by default (24)
| Gate | Site | Purpose / status |
|---|---|---|
| SESSION_TOPO_REL | 1643 | reversed-flag rel instead of geometric probe |
| SESSION_ORIENT_REPAIR | 1683 | relsgn neighbour-majority relaxation |
| SESSION_SHELL_ORIENT | 1909 | per-component outward re-vote on closed shells |
| SESSION_FORCE_GAUSS | 2328 | force masked Gauss over analytic quadric flux |
| SESSION_TRIM_CUT | 2991 | trim-aware cutting (also on via BOOL_SHARED_EDGES) |
| SESSION_EF_PAVES | 3376 | EF paves on quadric path — regressed cone×cyl 2→8 naked |
| SESSION_ZEROFILL | 3730 | fill zero-span section runs with whole segment (Law 5 experiment) |
| SESSION_KEEP_BORDER | 4181 | keep border-coincident scaffold segments (y30 graze) |
| SESSION_STUB_REACH | 4208 | boundary-reaching stubs — "mixed on rotated cfgs" |
| SESSION_SE_OTHER | 4272 | PutSEInOtherFaces — one-sided, y30 naked 13→31, "next design pass" |
| SESSION_TRIM_SNAP / _OV | 4509/4536 | legacy boundary-snap bridges (scaffold path excludes) |
| SESSION_WIRESPLIT | 4571 | seam-aware WireSplitter arrangement |
| SESSION_BOOL_ELLIPSE | 5354 | Steinmetz fit in co_refine split loop — "GATED OFF pending P8; flipped one fuse edge non-solid" |
| SESSION_SHAREDPAVE | 8507 | one shared pave set per segment for both normalizes (P1b) |
| SESSION_BOOL_SHARED_EDGES | 2991,10089 | P0 pave-engine branch (make_shared_section_edges, early return) |
| SESSION_BOP2 | 8102 | shared-edge pool + P4 referencing/alias (construction identity) |
| SESSION_SCAFFOLD_ALL | 8033 | scaffold for any pair (battery experiment) |
| SESSION_SYMEMIT | 8260 | re-imprint missing segs + resplit — superseded by SYMLIFT |
| SESSION_SYMLIFT | 8342 | direct chain lift + fragment split (replacement) |
| SESSION_REFINE | 8462 | preserve-identity loop; forced under BOP2; measured no-op on base, +2 naked z90 |
| SESSION_ANG_PRIMARY | 9260 | angle-evidence-first parity seeding (KB law 7) |
| SESSION_CLS_FIX2 | 9357 | post-flood section re-enforcement — MEASURED HARMFUL (z30 23→35) |
| SESSION_RECOVER | 10062 | recover spans for legacy-lifted edges — "residual lacks mateable copies" |
| SESSION_FUZZY / FUZZLADDER / _CAP | 10125/10161/10179 | manual / auto tolerance escalation fallback |
| SESSION_CAPFILL | 10206 | synthesize absent mating cap face |
| SESSION_WIRE_REPAIR | 10330 | loop-sense (CCW/CW) enforcement |
| SESSION_SAMEPARAM | 10482 | planar pcurve rebuild — "OFF: moves volume AWAY with mixed-quality copies" |

### 2d. Tuning knobs (value-carrying)
SESSION_TRIM_TOL3 (3137), SESSION_TRIM_COS (3140), SESSION_TRIM_SNAP_OV (4536),
SESSION_FUZZY (mult), SESSION_FUZZLADDER_CAP (10179), SESSION_BND_SNAP (read in
nurbssurface_trimmed.cpp:744; brep.cpp comments 4573/4580 only).

---

## 3. Dead / superseded code candidates (evidence cited)

1. **SYMEMIT block (8254–8333) + p1c_onA/p1c_onB channels (8126–8142, 8236–8241)** —
   comment 8334: "P1c-DIRECT (SESSION_SYMLIFT): the CORRECT replacement for SYMEMIT …
   the SSI corrector … slid to the WRONG BRANCH". Both opt-in; SYMEMIT is explicitly
   superseded. DELETE (and the run_splits re-entrancy it required, if SYMLIFT stays).
2. **SESSION_CLS_FIX2 (9344–9390)** — "MEASURED HARMFUL, OFF by default … chairsROT z30
   cut went 23 -> 35 naked … Kept, gated, as a diagnostic." DELETE (the finding is
   recorded in comments/KB; the code adds nothing).
3. **SESSION_EF_PAVES (3376–3426)** — "OPT-IN: the full identity bundle regressed the
   proven quadric path (reimport cone x cyl 2->8 naked)". Superseded by scaffold paves.
   DELETE unless the quadric-identity plan is revived.
4. **SESSION_TRIM_SNAP bridges (4500–4557)** — legacy path only (`scaf ? nullptr`);
   scaffold overshoot stubs at 4055–4263 are "the proven TRIM_SNAP-bridge pattern"
   internalized. DELETE with the legacy freeform path.
5. **make_shared_section_edges (6531–6792) + SESSION_BOOL_SHARED_EDGES branch (10085–10093)**
   — P0/P1 pave engine; opt-in, early-returns, never default. Superseded by scaffold S2
   chain-lift + BOP2 pool (same OCCT concepts, done at construction). Its only side
   effect on default paths is enabling s_trimcut (2991). DELETE once BOP2 default-on;
   decouple s_trimcut first.
6. **snap_section_edges (6794–6871) + call (10078–10084) + sec_c3ds plumbing** — active
   only on `imported_freeform && !use_scaffold`, i.e. the legacy freeform lane the
   scaffold replaced (comment 8025: "Scaffold is DEFAULT-ON for imported freeform").
   Remaining consumer: unrecognized-but-analytic pairs (cone×cone) which take the S1
   pre_cuts lane — that lane feeds sec_c3ds. Keep until scaffold gains seam handling
   (comment 8034–8038), then DELETE.
7. **SESSION_BOOL_ELLIPSE block in co_refine (5354–5409)** — "GATED OFF pending P8";
   FINAL PASS 2's conic registry derives the same Steinmetz ellipses UNGATED
   (5932–5968) and upgrades arcs through their own endpoints. The gated block is the
   older, endpoint-forcing variant. DELETE the gated block; keep the registry.
8. **SESSION_SE_OTHER (4264–4332)** — "one-sided imports create MORE unmated copies
   (y30 13->31) until the import is made symmetric … next design pass". Dead-end
   as-is; the symmetric successor is SYMLIFT / P4. DELETE or rewrite symmetric.
9. **SESSION_STUB_REACH (4209–4241)** — "opt-in (mixed on rotated cfgs)". Candidate for
   deletion after rotation campaign settles.
10. **SESSION_RECOVER / recover_section_spans (6105–6180, 10062–10064)** — "opt-in:
    recovers spans but residual lacks mateable copies" — measured ineffective. DELETE.
11. **SESSION_CAPFILL (10199–10324)** — cap synthesis for absent SSI caps; superseded in
    direction by symmetric emission/lift + scaffold closure-weld. Diagnostic value only.
12. **SESSION_ORIENT_REPAIR (1677–1700) / SESSION_SHELL_ORIENT (1899–1981)** — both
    gated OFF; SHELL_ORIENT's rationale (fragmented rel-graph) is being addressed
    upstream (symmetric coverage flood, radial certification). Keep SHELL_ORIENT while
    rotation frontier open; ORIENT_REPAIR is a weaker duplicate of it. DELETE ORIENT_REPAIR.
13. **SESSION_NO_FUZZLADDER** — name exists only in a comment (10160); code gate is
    positive SESSION_FUZZLADDER (10161). Fix the comment or the polarity.
14. **SESSION_NO_ON (8814)** — labeled "experiment"; ON handling is load-bearing.
    The experiment is concluded. DELETE gate.
15. **Hausdorff sew escalation passes P2/P3 (7017–7095) + FUZZY/FUZZLADDER (10116–10198)**
    — the construction-identity thesis: comment 9740 "CROSS-OPERAND WELD + COMMON-BLOCK
    (construction-time; replaces sew/fuzzy residue)", comment 10102 "with the Hausdorff
    sew retired, THIS is where those refined pairs merge", and SESSION_NO_SEW exists
    precisely "to expose the exact residual the construction-time shared-edge path must
    close by itself" (10112). P7 removal targets once residual is 0 under NO_SEW.
16. **SYMLIFT (8334–8441)** — still opt-in; if BOP2 P4 referencing makes both operands
    emit every segment by construction, the missing-segment lift becomes unreachable.
    Re-evaluate at P7.

---

## 4. P7 deletion plan (post construction-identity)

Precondition: BOP2 (pool referencing + P4 split + index alias at combine) default-on and
`SESSION_NO_SEW` runs show naked=0 across base/matrix/edge/rot batteries — i.e. section
mating holds by construction, not by distance.

**Phase 1 — no-risk deletions (independent of BOP2):**
- SYMEMIT block + p1c channels; SESSION_CLS_FIX2; SESSION_EF_PAVES; SESSION_RECOVER +
  recover_section_spans; SESSION_SE_OTHER; SESSION_ORIENT_REPAIR; SESSION_NO_ON;
  co_refine's gated SESSION_BOOL_ELLIPSE block; stale SESSION_NO_FUZZLADDER comment.
- Consolidate the 26 debug gates behind one SESSION_BREP_DBG=<topic list> dispatcher
  (biggest single line-count win: debug printing is ≈15% of boolean()).

**Phase 2 — legacy-freeform lane retirement (needs scaffold seam handling for
unrecognized analytic pairs, cone×cone):**
- pre_cuts pair-SSI (8047–8076), cutsA/cutsB, sec_c3ds, snap_section_edges + NO_SECSNAP,
  TRIM_SNAP/_OV bridges, legacy `snap_bnd=0.05` branch, imported_freeform && !use_scaffold
  conditionals throughout.

**Phase 3 — distance-based mating retirement (the construction-identity payoff):**
- sew_coincident_edges P2 + P3 (keep P1 exact-Hausdorff merge + micro/orphan collapse as
  the final safety net, or move collapses into xweld).
- FUZZY + FUZZLADDER (+_CAP): a construction-identity result must never need them.
- make_shared_section_edges + SESSION_BOOL_SHARED_EDGES (fold s_trimcut onto its own gate
  or delete trim-cut path if scaffold covers it).
- normalize_section_blocks A/B/R calls + SHAREDPAVE: P4 "one-edge-per-segment IS
  one-edge-per-block" (8482–8488) makes them redundant; keep only the R-pass initially,
  measure, then drop.
- NK-RESCUE and span-alias (qspan) fallback at combine: BOP2 index alias supersedes both.
- CAPFILL, WIRE_REPAIR: delete or promote to unconditional ShapeFix-style validators.

**Must NOT be deleted (proven load-bearing):**
- co_refine circle/arc exact rebuild (volume exactness for the quadric matrix),
  sew P1 for the non-scaffold matrix path until BOP2 covers primitives too,
  winding contains_point, contains_point_exact, angle method, connexity flood
  (symmetric coverage), parity, radial certification, island repair, micro-fragment
  collapse, xweld, imprint_edges, face_outward_signs tiers 1–3.

Estimated reduction at full P7: ~2,500–3,000 lines of brep.cpp (SYMEMIT ~80, CLS_FIX2 ~45,
EF_PAVES ~50, TRIM_SNAP ~57, SE_OTHER ~70, make_shared_section_edges ~260,
snap_section_edges ~80, recover ~75, CAPFILL ~125, sew P2/P3 ~80, FUZZY+LADDER ~85,
legacy pre_cuts ~30, normalize+SHAREDPAVE ~400, debug consolidation ~600–900).
