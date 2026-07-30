# P3 same-domain subsystem — integration notes (session B → session A)

2026-07-25. Delivered by session B as NEW files; **session B did not integrate**. This document
is the call-site design for the P1/kernel session to apply. Nothing here has been applied to
`brep.cpp` / `brep_section.cpp`.

Ground truth used: the real OCCT V8 source at `/home/petras/code/code_cpp/OCCT`
(`BOPAlgo_Builder.cxx:565-749`, `BOPAlgo_Builder_2.cxx:580-925`,
`BOPTools_AlgoTools.cxx:1139-1205`), reconciled against **both** source audits:
`kb/audit_occt_builder-assembly.md` (op-table; supersedes `kb/occt_builder-assembly.md` §15) and
`kb/audit_occt_ff-samedomain.md` (SD verdict; found 7 material errors in
`kb/occt_ff-posttreat-samedomain.md`, the spec this module was ported from).

**Reconciliation pass (post-audit) changed three things and confirmed the rest:**

| audited fact | status |
|---|---|
| `BOPTools_Set` is multiplicity-SENSITIVE (seam counts twice) | **FIXED** — the key deduplicated edges; now a multiset. Locked by cell K6 |
| verdict carries THREE tolerances; ON band is `tolF2`, not the summed band | **FIXED** — `faces_same_domain` now takes `band` + `class_band` separately |
| edge-tolerance lift is asymmetric (F1's edges raise BOTH terms) | **FIXED structurally** — `sd_band`; numerically a no-op until P5 (§7) |
| ON survivor needs BOTH sightings, not one state enum | **ADDRESSED** — `sd_select_sd_face` pair overload + §3 guidance |
| one-directional, fail-closed, no `PointNearEdge` fallback | confirmed already correct |
| SD candidacy ignores `TangentFaces` entirely | confirmed already correct (no tangency flag anywhere) |
| `Sense()` is dead code (zero callers) | confirmed — never ported |
| planar shortcut must be gated on the same-solid guard | confirmed already correct |
| op-table incl. the CUT21 asymmetry | confirmed already correct (verified twice) |

---

## 1. What landed (files owned by session B)

| file | contents |
|---|---|
| `src/brep_samedomain.h/.cpp` | same-domain detection: `SDEdgeSig`, `SDFaceKey`, `SameDomain` detector, `sd_select_face` (BuildBOP op-table), `sd_classify_face` (ON-classification), `sd_boolean_coincident` (reference driver) |
| `src/brep_commonblock.h/.cpp` | PARTIAL coincidence (§10): `CommonBlock`, `cb_split_chain` (per-region splitting predicate), `cb_tolerance` (ComputeToleranceOfCB), `cb_perform_common_blocks`, `cb_partition_face` / `cb_select_region` (region op-table) |
| `main_10.cpp` | 73-cell standalone test driver (CMake `MAIN_ID 10`), 73/73 green |
| `kb/p3_integration_notes.md` | this file |

Only shared-file edit made: `10` appended to `foreach(MAIN_ID 3 6 7 8 9)` in `CMakeLists.txt:258`.
`src/*.cpp` is GLOBbed into `session_core` (`CMakeLists.txt:221`), so the module builds with no
further CMake change.

Build/run:
```
cmake -S session_cpp -B session_cpp/build_b -DCMAKE_BUILD_TYPE=Release
cmake --build session_cpp/build_b --target main_10 -j 8
./session_cpp/build_b/main_10                 # [SD] TOTAL 73/73, exit 0
SESSION_SD_DUMP=1  ./build_b/main_10          # per-face state dump for the ZD4 pair
SESSION_SD_TRACE=1 ./build_b/main_10          # per-probe nearest-hit trace (d, dp)
```

## 2. API surface

```cpp
enum class SDOp     { Fuse, Common, Cut, Cut21 };            // Cut = A-B, Cut21 = B-A
enum class SDState  { In, Out, OnSame, OnOpposite };
enum class SDVerdict{ Drop, Keep, KeepReversed };

class SameDomain {
    SameDomain(double key_tol, double fuzz = 0.0);
    int  add_face(const BRep&, int face, int operand, int solid = -1, double tol = 0.0);
    void add_solid(const BRep&, int operand, int solid = -1);
    void detect();
    int  rep(int i) const;  bool same_domain(int i, int j) const;
    const std::vector<SDPairRec>& pairs() const;
    std::vector<std::vector<int>> groups() const;             // classes >= 2, sorted, [0] = rep
    // one-directional + fail-closed; TWO bands (strict reject / ON classifier) -- see §7
    static bool      faces_same_domain(const BRep&,int, const BRep&,int,
                                       double band, double class_band, int* orient = nullptr);
    static double    sd_band(const BRep& A, int fa, double tolF1, double tolF2, double fuzz,
                             double* class_band = nullptr);
    static double    max_edge_tolerance(const BRep&, int face);   // 0 today; the P5 hook
    static SDFaceKey face_key(const BRep&, int face, double key_tol);
    static bool      edge_sig(const BRep&, int edge, double key_tol, SDEdgeSig&);
    static bool      point_in_face(const BRep&, int face, Point&, double& u, double& v);
    static std::vector<std::array<double,2>> samples_in_face(const BRep&, int face, int want = 16);
};

SDVerdict sd_select_face(SDOp op, int operand, SDState state);          // ordinary In/Out fragments
SDVerdict sd_select_sd_face(SDOp op, int operand, bool orient_same);   // SD wall: PAIR property
SDState   sd_classify_face(const BRep& S, int face, const BRep& other, double tol,
                           const std::vector<double>& osign_S = {},
                           const std::vector<double>& osign_other = {}, int probes = 16);
BRep      sd_boolean_coincident(const BRep& A, const BRep& B, SDOp op, double tol);

// per-REGION classification primitives (used by the common-block module)
SDState   sd_classify_samples(const BRep& S, int face, const std::vector<std::array<double,2>>& uvs,
                              const BRep& other, double tol, osign_S = {}, osign_other = {});
double    sd_distance_to_boundary(const BRep& b, const Point& p, int* face = nullptr);
```

`src/brep_commonblock.h` — PARTIAL coincidence (§10):

```cpp
enum class CBOn { Face, Edge };                 // membership predicate
struct CBPave   { double t; int kind; };        // kind 1 = synthesised coincidence boundary
struct CBMember { const BRep* brep; int operand, face, edge; double dist, tol; };
struct CommonBlock {                            // one interval of CONSTANT coincidence state
    const NurbsCurve* curve; double t0, t1;     // range on the source curve -- geometry NEVER refit
    bool coincident; int operands_mask;
    std::vector<CBMember> members; std::vector<CBPave> paves;
    double tolerance; int orient;
};
struct CBSplit  { std::vector<CommonBlock> blocks; int n_coincident, n_transitions; };
struct CBParams { double band; CBOn on; int samples, bisect; double micro_frac; };

bool     cb_on_operand(const BRep&, const Point&, double band, CBOn, int* face, int* edge, double* d);
CBSplit  cb_split_chain(const NurbsCurve&, const BRep& A, const BRep& B, const CBParams& = {});
double   cb_tolerance(CommonBlock&, int samples = 11);
std::vector<int> cb_perform_common_blocks(std::vector<CommonBlock>&);

struct CBRegion { const BRep* brep; int operand, face;
                  std::vector<std::array<double,2>> uvs; SDState state; int orient; };
SDState   cb_classify_region(CBRegion&, const BRep& other, double tol, osign_S, osign_other);
SDVerdict cb_select_region(SDOp op, const CBRegion& reg);
std::vector<CBRegion> cb_partition_face(const BRep& S, int face, int operand, const BRep& other,
                                        double tol, int probes = 64, osign_S = {}, osign_other = {});
```

## 3. The two hooks A must apply

### HOOK 1 PRECONDITION — read this before wiring anything (audit trap #8)

**OCCT's edge-set bucketing works only because `PostTreatFF` has ALREADY fused the cross-pair
coincident section edges into shared `TShape`s.** By the time `FillSameDomainFaces` runs, the two
operands' copies of one section edge *are the same object*, so `BOPTools_Set` equality (`IsSame`
containment) matches them for free. The audit states the consequence bluntly:

> "A port that keeps per-face copies of coincident section edges will produce zero SD faces and
> then wrong COMMON/CUT results."

This kernel has **no shared section entities** at that point (BOOLEAN_MASTER_PLAN §1.2 R2 — a
section edge exists as two private copies reconciled post-hoc). So:

- `SameDomain` substitutes **geometric bucketing at `key_tol`** for OCCT's `IsSame` identity.
  That is the only reason HOOK 1 can run *before* the imprint exists.
- **`key_tol` is therefore load-bearing, not a nicety.** Too tight ⇒ the two copies of a
  coincident face land in different buckets ⇒ zero SD pairs ⇒ silently wrong COMMON/CUT (the
  exact failure the audit names). Too loose ⇒ distinct rims collide ⇒ false SD.
- The tolerant fallback (edge count + bbox + bipartite signature match) exists specifically to
  absorb quantization straddles at the bucket boundary; it is not redundant.
- **Recommended `key_tol` = `tol3_rep`** (the per-pair representative band), never the global
  `tol3`. If the imprint/shared-edge work (Law 1) later gives us real shared section edges, the
  geometric bucketing can be replaced by identity comparison and `key_tol` stops mattering.

Sanity check A should run when wiring this: on a known-coincident cell, assert
`sd.group_count() > 0` before trusting any downstream selection. Zero groups on a cell that
visibly has a shared wall means the bucketing tolerance is wrong, **not** that there is no
coincidence.

### HOOK 1 — SD face detection: run it **PRE-split** (slot corrected by session A)

**SLOT CORRECTION (2026-07-25, from session A's integration behind `SESSION_SD`).** This note
originally specified the post-split slot, mirroring OCCT's `FillSameDomainFaces` (which runs after
`BuildSplitFaces`). That is **wrong for this kernel**, and A measured why: post-split, A is already
shattered into fragments, so the edge-set buckets cannot match. Run detection on the **UNSPLIT**
operands. Validated: pre-split on the A-op-A cell the detector reports `groups=20 pairs=20`.

The reason is the HOOK 1 PRECONDITION above, seen from the other side: OCCT can afford the
post-split slot because `PostTreatFF` has already made the fragments' shared edges *identical
objects*, so splitting does not destroy bucket equality. Without shared entities, splitting is
exactly what destroys it. Detect first, then split.

Call site: `BRep::boolean`, **before** `split_with` on either operand.

```cpp
// A, B = the ORIGINAL unsplit operands.
SameDomain sd(tol3_rep, fuzz);
sd.add_solid(A, /*operand*/0, /*solid*/0);
sd.add_solid(B, /*operand*/1, /*solid*/1);
sd.detect();
for (const auto& g : sd.groups()) {
    // g[0] = representative (min index). Every other member is a coincident duplicate:
    // keep ONE geometric face, and route the op-table verdict through it (HOOK 2).
}
```

Data flow: the detector needs only `m_faces/m_loops/m_trims/m_curves_2d/m_curves_3d/m_surfaces`
plus `face_outward_signs()` (called once per distinct `BRep*`, cached internally). It **does not
mutate** either operand — which is what makes the pre-split slot free of side effects.

Session A's landed mechanism order for FULL coincidence (A-op-A now passes all three ops: cut
EMPTY, common = A at 20 faces / vol 80.3011, fuse = A — previously a >600 s timeout): (1) skip SSI
on identical surface pairs [20/20], (2) suppress common-block chains within `weld_tol` of an
existing edge of BOTH operands [141/149; scaffold 149 chains/481 segs → 8/10], (3) stand down the
legacy ON-imprint path. **Mechanisms 2 and 3 are whole-chain and whole-path** — §10 is what makes
them per-region, which is required the moment coincidence is partial.

`solid` is the zero-thickness guard key: two faces with the same `solid` are never SD. Pass the
operand id for a single-solid operand. For **multi-solid operands** (n-ary fuse, cavity results)
pass a per-source-solid id, or coincident walls inside one operand will be wrongly merged.

### HOOK 2 — replace the per-fragment keep/flip decision with the op-table

Call site: the fragment selection in `BRep::boolean` (today: per-fragment point classification →
per-op keep rules). Replace the *state → keep* decision, not the classifier:

```cpp
SDState st = /* In/Out from the existing classifier */;
if (sd_is_coincident_fragment) st = (orientation_relation_same ? SDState::OnSame
                                                              : SDState::OnOpposite);
switch (sd_select_face(op, operand, st)) {
    case SDVerdict::Drop:         break;                  // not in the result
    case SDVerdict::Keep:         emit(face);      break;
    case SDVerdict::KeepReversed: emit(face.reversed()); break;   // CUT tool faces
}
```

The orientation relation must be measured on **OUTWARD** normals
(`face_outward_signs()` sign × surface normal), not stored `trim.reversed` flags — the flags encode
pcurve direction, not manifold traversal (BOOLEAN_MASTER_PLAN §1.2 R2).

**The ON state is a property of the PAIR, not of one face.** OCCT has no "ON" state at all: the
survivor is decided by **two fences** (`BOPAlgo_Builder.cxx:682/696/714`) — `aMFence`,
orientation-**blind** (`IsSame`), then `aMFenceOri`, orientation-**sensitive** (`IsEqual` =
TShape + Location + Orientation). `isSameOri` is literally "was this exact *oriented* face already
seen". The ON-same / ON-opposite distinction exists **only** because those two identities are kept
separate, so HOOK 2 must carry **both sightings of the wall**, not one face's measurement. Use the
pair-explicit overload and feed it the detector's pair record:

```cpp
for (const auto& pr : sd.pairs()) {                 // pr.orient: 0 = same, 1 = opposite
    bool orient_same = (pr.orient == 0);
    auto vi = sd_select_sd_face(op, sd.face(pr.i).operand, orient_same);
    auto vj = sd_select_sd_face(op, sd.face(pr.j).operand, orient_same);
    // exactly one of vi/vj is Keep (the op's OUT-state side); the other is Drop
}
```
Do **not** derive `orient_same` per-face from `sd_classify_face`'s OnSame/OnOpposite and then call
`sd_select_face` twice independently — that reads a nearest-boundary normal, which agrees with the
pair relation for a genuine SD mate but is a *different measurement* and can disagree at a
three-face junction. `sd_select_face(op, operand, state)` remains available for ordinary In/Out
fragments, where the state genuinely is a per-face property.

## 4. The op-table (verified against source AND `kb/audit_occt_builder-assembly.md` §2.6)

`isSameOriNeeded = (objState == toolsState)`: true for FUSE (OUT/OUT) and COMMON (IN/IN),
false for CUT (OUT/IN) and CUT21 (IN/OUT).

| op | operand | In | Out | OnSame | OnOpposite |
|---|---|---|---|---|---|
| FUSE | obj (0) | Drop | **Keep** | **Keep** | Drop |
| FUSE | tool (1) | Drop | **Keep** | Drop | Drop |
| COMMON | obj | **Keep** | Drop | **Keep** | Drop |
| COMMON | tool | **Keep** | Drop | Drop | Drop |
| CUT | obj | Drop | **Keep** | Drop | **Keep** |
| CUT | tool | **KeepReversed** | Drop | Drop | Drop |
| CUT21 | obj | **KeepReversed** | Drop | Drop | Drop |
| CUT21 | tool | Drop | **Keep** | Drop | **Keep** |

Two rules encoded here that a naive port gets wrong:

1. **The SD wall is kept exactly ONCE**, from the op's OUT-state side — objects for FUSE/CUT,
   tools for CUT21 (COMMON has no OUT side; objects by convention). The CUT21 asymmetry is real:
   the audit (§2.6, erratum E3) shows it arises from `aMResFacesFence` suppressing the second
   group's copy, and it is *invisible* in the older spec `kb/occt_builder-assembly.md` §15.
2. **CUT reverses tool faces** (`bTakeIN && !isSameOriNeeded → aFIm.Reversed()`), and CUT21
   likewise reverses object faces.

Not modelled (single-solid-per-operand assumption): OCCT's `isIN` row — a face IN a solid of its
**own** group is emitted in *both* orientations. That case only arises for multi-solid operands;
add it to `sd_select_face` when n-ary operands land.

## 5. ON-classification: coincidence is AREAL (Law 3), never a lucky near-zero

`sd_classify_face` probes up to 16 in-material points and declares ON **only if every probe** is
within `tol` of the other solid's trimmed boundary. This is load-bearing, not a refinement — two
main_10 cells fail with a single centroid probe:

- **C5** (ZN4, flush cap inside a bigger wall): the box wall's centroid lands exactly on the
  cylinder cap → whole 4×4 wall reads `OnSame` though it only partially overlaps.
- **C7** (external line tangency): the cylinder's lateral face probes at u-domain centre =
  exactly the tangency line → reads `OnOpposite` for measure-zero contact.

A partially-coincident face reports its **majority In/Out**; exact per-region states require the
face to be split first. In the integrated pipeline fragments are already split, so whole-face
states are exact there — this matters for the *unsplit* fast paths.

### 5.1 Two point-classification defects found (they affect `brep.cpp` too)

While building the ON test, two robustness defects surfaced in the "signed side of the nearest
boundary" rule that `BRep::contains_point_exact` (`brep.cpp:1120-1218`) also implements. Session B
fixed them inside its own module; **`contains_point_exact` still has both** — recommend A ports
the same guards.

1. **Boundary-hit bias (systematic, not noise).** When the nearest point is found by
   `closest_on_trim` (a *sampled* trim boundary), `p` and `q` can both lie at radius R but at
   slightly different angles, so `(p−q)·n̂(q) = cos Δθ − 1 < 0` **always** — a false "inside"
   of order the sampling sagitta (measured `dp = −1.9e-2` on stacked unit cylinders, ~24 rim
   samples). Fix: a nearest point on a trim boundary means the probe is nearest to an **edge**,
   where the face normal is not the separating direction — the sign must not be used.
2. **Silhouette/rim degeneracy.** When the probe direction is tangential to the boundary
   (`|dp|/d → 0`; e.g. a point radially aligned with a coaxial cylinder), `dp` is pure round-off
   and its sign is decided by noise (measured `dp ~ ±1e-16…1e-18`, flipping In/Out per sample).

Guards now in `nearest_on_solid` / `sd_classify_face`:
```cpp
// prefer an INTERIOR hit over an equidistant BOUNDARY hit; among equals, largest |dp| wins
bool reliable = !hit.on_boundary && std::abs(hit.dp) > 1e-6 * hit.d;
bool inside   = reliable ? (hit.dp < 0) : other.contains_point(mesh_of_other, p);  // ray parity
```
The parity fallback tessellates lazily — most faces never trigger it.

## 6. Canonical keys (BOPTools_Set analog)

Face key = **sorted** edge signatures; equal keys are the SD candidate gate. Signatures are
quantized on a `key_tol` grid and built to be invariant under the two things that legitimately
differ between two copies of one geometric edge:

- **orientation** — open edges sort their endpoints and use the *domain-midpoint* (curve reversal
  maps the midpoint to itself);
- **parameterization phase** — closed edges (rim circles) use `{sample centroid, r_mean, r_spread}`,
  because seam twins (ZD5 = ZD4 with one operand rotated 90° about its own axis) have the *same*
  circle with a different parameter origin. A param-midpoint key would differ and miss the pair.
  main_10 cell **D10** is exactly this regression guard.

`BOPTools_Set` semantics reproduced exactly (`BOPTools_Set.cxx:120-172`):

- **degenerate edges SKIPPED** (span below the grid — poles/apexes), matching `BRep_Tool::Degenerated`;
- **orientation-INSENSITIVE** — signatures are built to be reversal-invariant, and equality is
  multiset equality (OCCT: equal count + one-way `IsSame` containment);
- **multiplicity-SENSITIVE** — the key is a MULTISET: an edge used twice by the wires (a seam)
  contributes **2**. The first cut of this module deduplicated edge indices, which made the key
  multiplicity-*insensitive* and would collide faces OCCT keeps distinct; fixed, and locked by
  cell **K6** (sphere face 0 = 2 identical seam signatures, poles excluded).
- OCCT additionally inserts an `INTERNAL`-oriented edge twice; this kernel has no INTERNAL
  orientation (`BRepTrimType` is Boundary/Mated/Seam/Singular), so that rule is vacuous here.

A tolerant fallback (edge count + bbox + bipartite signature match within
`tolA+tolB+max(fuzz,1e-7)`) catches quantization straddles that the exact-key bucket misses.

## 7. Tolerances — the verdict carries THREE bands, not one

Reconciled against `kb/audit_occt_ff-samedomain.md` (which found 7 material errors in the spec
this was ported from). `SameDomain::sd_band` + `faces_same_domain` now reproduce:

| # | band | value | role |
|---|---|---|---|
| 1 | projector tolerance | 1e-12, `Extrema_ExtFlag_MIN`, onto the face's **UV patch** (not the infinite surface) | finds the projection |
| 2 | `band` | `tolF1' + tolF2' + max(fuzz, 1e-7)` | **STRICT** unsquared rejection: `dist > band` fails, `dist == band` **passes** |
| 3 | `class_band` | `BRep_Tool::Tolerance(F2)` = **tolF2' alone** | classifier band; `TopAbs_ON` counts as **inside** |

Collapsing 2 and 3 into one band (the first cut of this module did) makes ON acceptance far looser
than OCCT's, since `band >= class_band` always. Both are now separate parameters of
`faces_same_domain`.

The **edge-tolerance lift is asymmetric** (audit E3): `aTolEMax` is scanned over **F1's edges
only** and raises **both** `tolF1` and `tolF2` — F2's own edge tolerances never enter the band.
`sd_band` implements exactly that asymmetry.

**Known gap (structural, not numerical):** this kernel has no per-entity tolerances — `BRepEdge`
carries no tolerance field — so `SameDomain::max_edge_tolerance` returns 0 today and the lift is a
no-op. The OCCT *structure* is in place, so P5 wires in by making that one function return
`max(tolE)`. Audit trap #5 warns this term is normally *dominant* once section edges inflate their
vertices/edges, so SD will under-fire on exactly the geometry that needs it until P5 lands.

`SDFaceRec::tol` is the per-face input (defaults to `key_tol`). Recommended `key_tol` for the
integrated path: `tol3_rep`, not the global `tol3`.

### 7.1 One-directional and fail-closed (audit E2, traps 1–2)

`faces_same_domain(A,fa,B,fb,…)` takes its interior point from **A.fa only** and is *not*
symmetric. If no interior point can be found it returns **false** — "cannot decide" is reported as
"not same domain", with **no** `PointNearEdge` fallback (that fallback exists only in
`IsSplitToReverse`). `detect()` calls it once per candidate pair with F1 = the lower-indexed face,
so the choice is deterministic. Do not "improve" this into a symmetric or more-robust test: it
changes which pairs enter the connexity blocks, and therefore boolean results.

The 16-probe areal rule in `sd_classify_face` is *denser* than OCCT's 2-attempt hatcher probe (at
43.2 % of the U span, `PAR_T = 0.43213918`), which is a deliberate improvement — but it keeps the
fail-closed policy: probing failure yields `Out` (i.e. "not ON"), never a looser fallback test.

## 7.2 Candidacy, guards, and deliberate divergences (audit E1, E4, §C)

- **No tangency flag is consulted anywhere.** SD candidacy is purely "identical edge set", exactly
  as in the Builder. OCCT's `TangentFaces` is read *only* by `BOPAlgo_CheckerSI`, on the original
  unsplit faces; `FillSameDomainFaces` never sees it (audit E4). Consequence worth knowing: if
  imprinting fails to give two coincident faces identical edge sets, a tangential pair is
  **silently dropped** — which is the same failure mode as the HOOK 1 precondition above.
- **`Sense()` is dead code** — zero callers in the whole OCCT tree (audit E1). Not ported, and it
  should not be added. Orientation is resolved per consumer via the `IsSplitToReverse` path.
- **Same-solid guard is STRICTER here.** OCCT fills its face→solid map only from `TopAbs_SOLID`
  sources with first-bind-wins, so faces of **shell-only operands are never guarded at all**.
  Every face registered here carries a `solid` key and is always guarded. For closed-solid
  operands the two agree; for open/shell operands this module refuses SD pairs OCCT would allow.
- **Planar shortcut is gated on the same-solid guard** (matching `Builder_2.cxx:776-785`, where the
  guard runs first). This matters because the shortcut asserts SD from surface type + bbox with
  **no geometric test at all** — it is the one path that can declare a false SD, so it must never
  be reachable for two faces of one operand. Verified: `detect()` `continue`s on
  `fi.solid == fj.solid` before the shortcut is considered.
- **Bounded-ness:** OCCT computes `bCheckPlanar` on the **original** face (planar surface + bbox
  not open in any of 6 directions) and lets all splits inherit it; a port computing it per split
  diverges (audit trap #9). This kernel has no unbounded/infinite faces, so `bounded` is
  unconditionally true and the test is vacuous — but if HOOK 1 registers *fragments*, keep
  inheriting the parent face's planarity rather than recomputing per fragment.

## 8. Determinism and representative election

Representative = **minimum registered index** in the class (union-find, path-compressed), so
registration order fixes the result. Register A's faces before B's, in face-index order, and the
representative is deterministic and reproducible.

OCCT's rule is min **DS index among unsplit originals**, with a fallback (block BFS seed) when a
class contains no original — deterministic but *not* index-minimal (audit §2.4). Since every face
registered here is an "original" from the detector's perspective, the fallback never fires.

**Do not forget** (audit §2.4): OCCT binds `myShapesSD[rep] = rep` — the representative is in its
own SD map, so it also takes the *geometric* reversal branch downstream. The analogue here:
`rep(i) == i` for a representative, but `groups()` still lists it as a member. Orientation must be
**re-derived at every use site** (audit §2.1: image lookup is orientation-blind), never inherited
from the map.

## 9. Test coverage delivered (main_10, 73/73)

| group | cells | what is asserted |
|---|---|---|
| K1–K6 | 6 | key self-equality, per-face distinctness, twin match, translation sensitivity, degenerate-edge exclusion, **multiplicity-sensitivity** (BOPTools_Set) |
| P1–P14 | 14 | **partial coincidence** (§10): FC03 half-face 3-block split + paves at the exact region boundary + not-whole-chain-decidable, FC02 inner region, FC21 cylinder partial lateral strip, box-face-on-cyl-cap, full-coincidence single block, no-coincidence single block, `ComputeToleranceOfCB` = measured gap, `PerformCommonBlocks` grouping + singleton skip, per-region face partition, **region op-table giving different verdicts within one face**, CUT keeps ON-opposite |
| D1–D14 | 14 | identical box/cyl/sphere/torus pairs, planar shortcut, min-index rep, groups, **same-solid guard**, stacked-wall & flush-cap orientation, **ZD5 seam twin**, partial overlap → no pair, line contact → no pair, disjoint |
| T1–T4 | 4 | the full op-table for FUSE/COMMON/CUT/CUT21 (all 4 states × 2 operands) |
| C1–C7 | 7 | ON-classification: all-OnSame, stacked OnOpposite, nested In, ZN4 flush caps, partial-overlap wall, ZP1 base cap, **line contact all-Out** |
| E | 28 | A-op-A idempotence for box/cyl/sphere/cone/torus (common=fuse=A, cut=∅), stacked/side boxes, ZD4 + ZD5 seam twin, nested cavity cut |

E-cells assert `faces / naked==0 / is_solid / volume` exactly (e.g. `torus_AopA_common` vol
9.869604401 = π²·... exact to 1e-9; `nested_cut_cavity` 12 faces vol 7.0).

## 10. PARTIAL coincidence — the common-block subsystem (`src/brep_commonblock.h/.cpp`)

Full coincidence can be solved by **suppression** (drop the duplicate chain) — that is what
session A's mechanism 2 does, and it fires 141/149 on A-op-A. Partial coincidence **cannot**: along
part of a face a section legitimately must exist, so the shared boundary has to be **synthesised**
as a common entity and the chain **split at the region boundary**. Every mechanism in this module
is per-region; nothing accepts or rejects a whole chain.

Port source: `BOPAlgo_Tools::PerformCommonBlocks` + `ComputeToleranceOfCB`
(`BOPAlgo_Tools.cxx:107-356`), data model `BOPDS_CommonBlock` / `BOPDS_PaveBlock`, as documented in
`kb/audit_occt_pavefiller-core.md` §2.8.

### 10.1 What `SharedEdgePool` lacks for this purpose

`SharedEdgePool` (`src/brep_section.h:57`, built by `build_shared_edge_pool`) is the *partial*
analog — it is a `BOPDS_PaveBlock` arena, not a `BOPDS_CommonBlock`. Precisely what is missing:

| `BOPDS_CommonBlock` needs | `SharedEdgePool` has | gap |
|---|---|---|
| exists for **any** coincident region | built **only** from a `SectionScaffold` | for pure coincidence SSI returns **no curve at all** (the FC class's defining property), so the pool is never populated — the case it is most needed for is the one it cannot represent |
| per-block **tolerance** (`ComputeToleranceOfCB`) | none — a single global `tol3`/`tol3_rep` | no measured per-entity coincidence band; nothing to grow at a weld or hand to STEP |
| **`Faces()`** — which faces the block lies IN/ON | `seg_edge` / `block_edge` index only | cannot answer "which faces must treat this as a boundary", which is what the classification flood needs |
| paves at the **coincidence** boundary | blocks keyed by `(seg_id, block)` from **scaffold** paves | its paves are section-curve paves; the parameters where coincidence *begins and ends* are a different set and are never computed |
| **orientation relation** of the coincident pair | none | the op-table's ON-same / ON-opposite rows cannot be evaluated |
| **members** (which operand entity each block replaces) | implicit, one arena edge per segment | no provenance, so no `PerformCommonBlocks` merge and no min-index representative |

Conclusion: the pool is reusable as the *storage* of a shared edge once a block is decided, but the
deciding — region boundaries, tolerance, faces, orientation — is new and lives in this module.

### 10.2 The splitting predicate (the per-region core)

```cpp
CBSplit cb_split_chain(const NurbsCurve& curve, const BRep& A, const BRep& B, const CBParams&);
```

For parameter `t` define `coincident(t) = cb_on_operand(A, C(t)) && cb_on_operand(B, C(t))`, where
membership is either **`CBOn::Face`** — within band of the operand's *trimmed boundary*, the
2-dimensional shared-region test — or **`CBOn::Edge`** — within band of an existing *edge*, which is
exactly the per-parameter form of session A's whole-chain mechanism-2 predicate.

Algorithm: uniform sample (64) → detect state changes between consecutive samples → **bisect** each
transition (40 steps) to land the cut on the true region edge rather than on a sample → absorb
micro intervals (`micro_frac`, OCCT's micro-edge policy) → emit contiguous blocks, each with its
members, paves (`kind=1` marks a *synthesised* coincidence boundary), and tolerance.

Measured on the FC03 half-face cell: cut at `0.333333` of the chain and region ends at
`y = -0.000001 / 2.000001` against exact `0 / 2` — the bisection resolves the boundary to ~1e-6
without any surface-surface section being computed.

**A chain that is coincident over only part of its span yields ≥ 2 blocks instead of one verdict.**
That is the whole deliverable: `P3` asserts `0 < coincident_frac < 1`, i.e. the case is provably not
whole-chain decidable. Full coincidence still collapses to a single block (`P7`), and no
coincidence to a single non-coincident block (`P8`) — so this generalises A's mechanisms rather
than replacing them.

### 10.3 Tolerance (`cb_tolerance`)

`ComputeToleranceOfCB` port: seed with the representative entity's tolerance, then **11 interior
samples at `dt = (t1-t0)/12` over the block's OWN range** (OCCT samples the representative pave
block's range, never the whole curve — audit trap #9), taking
`max(tol, tol(mate) + distance)` over every member edge and attached face. Verified by `P9`: a
chain offset by exactly 1e-3 from the shared wall yields `tolerance = 0.001000000`.

**P5 gap (same as §7):** every `CBMember::tol` is 0 because this kernel stores no per-entity
tolerances, so the result is the pure max deviation. The OCCT structure is preserved — P5 wires in
by filling `CBMember::tol`.

### 10.4 Grouping (`cb_perform_common_blocks`)

`PerformCommonBlocks` port: connected components over "shares a member entity", member/face lists
merged and deduplicated, min-index representative, tolerance recomputed for the group. **Groups of
fewer than 2 blocks are skipped**, exactly as OCCT does (`BOPAlgo_Tools.cxx:134-137`) — asserted by
`P11`, which checks a lone block reports group `-1` rather than being promoted to a common block.

### 10.5 Region op-table (extends `sd_select_sd_face`)

```cpp
std::vector<CBRegion> cb_partition_face(S, face, operand, other, tol, probes, osignS, osignO);
SDState              cb_classify_region(CBRegion&, other, tol, osignS, osignO);
SDVerdict            cb_select_region(SDOp op, const CBRegion& reg);
```

`cb_partition_face` classifies each in-material sample **individually** (a one-sample region is
exact under the areal rule) and groups by state. A face that is wholly In/Out/ON yields one region;
a **partially coincident face yields two or more — that split is the detection signal** a
whole-face verdict cannot produce. `cb_select_region` then applies the ON-same / ON-opposite rows
per region via `sd_select_sd_face`, and the ordinary In/Out rows otherwise.

`P13` is the load-bearing assertion: on ONE face of A, the coincident region returns `Drop` and the
non-coincident region returns `Keep` for FUSE — **different verdicts within a single face**, which
is impossible in whole-face logic. `P14` confirms CUT keeps the ON-opposite region from the
objects' side, matching §4.

### 10.6 Call sites for session A

1. **After** `SESSION_SD` detection identifies a coincident face pair that is only *partial*
   (`cb_partition_face` returns ≥ 2 regions), do **not** suppress the chain.
2. Run `cb_split_chain` on each candidate chain of that face pair with
   `band = tolA + tolB + fuzz` and `CBOn::Face`. Blocks with `coincident == false` keep their
   normal section treatment; blocks with `coincident == true` become common blocks.
3. Feed all coincident blocks of the pair through `cb_perform_common_blocks` to merge blocks that
   share an entity and to get one representative + one tolerance per shared region.
4. Mint ONE edge per coincident group (the `SharedEdgePool` arena is the right storage — see
   §10.1), referenced by both operands' faces, with `CommonBlock::tolerance` as its band.
5. Select faces per region with `cb_select_region` instead of per whole face.

Ordering note: this runs **after** the same-domain detection of HOOK 1 and **before** face
selection, i.e. in the same slot where mechanism 2 currently suppresses whole chains.

## 11. What is NOT covered (open items for A)

1. ~~**Partial-face coincidence**~~ — **DELIVERED** in §10 (`src/brep_commonblock.h/.cpp`). The
   *detector* still requires equal edge sets (OCCT does too), but partial coincidence no longer
   depends on it: `cb_split_chain` finds the region boundary directly and `cb_partition_face`
   detects partiality without any edge-set match.
2. ~~**SD edges / common blocks**~~ — **DELIVERED** in §10. Still faces+chains only; a full
   `BOPDS_CommonBlock` also carries the *split edge* identity written back into both operands,
   which is the minting step (§10.6 step 4) that belongs to A's arena.
2b. **2D region boundary curves.** `cb_split_chain` splits a chain *given* to it. It does not yet
   *generate* the region outline for the FC02/FC11 inner-loop cells (a face whose coincident region
   is a closed island needs that outline imprinted as an inner loop). The outline is the other
   face's trim boundary pulled into this face's UV; `cb_partition_face` already proves the island
   exists. This is the next increment.
3. **`sd_boolean_coincident` is a reference driver, not the kernel path.** It is exact only when
   every face classifies whole (A-op-A, flush stacks, full shared walls). Real integration goes
   through HOOK 1 + HOOK 2 on split fragments.
4. **Cost.** `nearest_on_solid` is O(faces of `other`) per probe, ×16 probes per face. Fine at test
   scale; the integrated path wants the existing BVH prefilter around it.
5. **n-ary / multi-solid operands** — pass real per-solid ids (§3) and add the `isIN` two-orientation
   row (§4).
