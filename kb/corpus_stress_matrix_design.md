# PRIMITIVE STRESS MATRIX — constructible cell list for degenerate primitive booleans

Written 2026-07-25. Companion to `kb/occt2_occt-test-mining.md` (OCCT grid ports),
`kb/audit_occt_intana-analytic.md` (what the analytic tangency ladder actually does),
`kb/commercial_kernel_doctrine_bridge.md` (Law 3 coincidence-as-outcome, Law 4 discovery≠refinement),
`kb/p1_attack_plan.md` (our defect classes A–G and mechanisms M1–M4).

**Why this file exists.** Today's gate is `main_7.cpp` `pairs()` — 21 pairs (the header comment calls
the first 15 the "15-pair x 3-op primitive matrix" = the 45 cells) of *general-position* operand pairs,
plus `edge_pairs()` (18 pairs = 54 op-cells, SESSION_EDGE) and `occt2_pairs()` (6 pairs + 4 chains).
General position is the easy case: two transversally intersecting quadrics have a clean 1-D section
everywhere. **Every residual failure in this kernel is at degeneracy** (chairsROT frontier 108; census
classes A/B/C in `p1_attack_plan.md`). This file enumerates the degenerate configurations an
industry-grade kernel must survive, each fully constructible, each with an analytic expected result
and an acceptance rule.

**101 distinct operand pairs (83 new; all 18 of today's `edge_pairs()` are absorbed) = 303 op-cells,
plus 18 epsilon-sweep families × 17 offsets = 306 configurations = 918 op-cells.
407 configurations / 1221 op-cells total.** (The class tables below hold 111 rows; 10 are deliberate
cross-listings of the same pair under a second degeneracy class, marked `=` in the row.)

---

## 0. CONVENTIONS (all verified against source, 2026-07-25)

**Place literal** — copied verbatim from `main_7.cpp:66`:

```cpp
struct Place { std::string kind; std::vector<double> p; std::array<double,7> xf; }; // xf: tx ty tz ax ay az deg
// xf_of(): Xform = translation(tx,ty,tz) * rotation(axis, deg)   -- rotation about the ORIGIN, applied FIRST
```

`xf_of()` (main_7.cpp:194) picks `rotation_x` if `ax != 0`, else `rotation_y` if `ay != 0`, else
`rotation_z`. **Cells marked [AXIS] need the arbitrary-axis upgrade** (PORT MAP row 4 of
`occt2_occt-test-mining.md`; `Xform::rotation(Vector&,deg,true)` already exists, the oracle
(`oracle.cpp:114` `gp_Ax1`) is already general). Rotation about a point `c` folds to this form as
`t' = c - R c` (composed with any subsequent translation).

**Primitive conventions** (`src/brep.cpp`, `src/primitives.cpp`, face counts from `src/brep_test.cpp`):

| kind | params | placement | faces | seam / pole |
|---|---|---|---|---|
| `box` | sx,sy,sz | **centered on origin** | 6 | — |
| `cylinder` | r,h | base disk at z=0, axis +Z, top at z=h | 3 | seam line at (r,0,z), u=0 = +X |
| `cone` | r,h | base disk at z=0, apex at (0,0,h) | 2 | seam (r,0,0)→apex; apex is a SINGULAR pole (v=1) |
| `sphere` | r | centered on origin | 1 | seam meridian in the **+X half of the XZ plane**; poles (0,0,±r) singular |
| `torus` | R,r | centered on origin, axis +Z | 1 | u-seam = minor circle at +X (u=0); v-seam = **outer equator, radius R+r at z=0**; they meet at the corner vertex (R+r,0,0) |

Analytic volumes used throughout: box `sx·sy·sz`; sphere `4πr³/3`; cylinder `πr²h`; cone `πr²h/3`;
torus `2π²Rr²`. Operand values for the standard vocabulary:
box 4³ = **64**, sph 2.5 = **65.449847**, cyl 1.5×6 = **42.411501**, cone 2×4 = **16.755161**,
tor 2/0.8 = **25.266187**, sph 2.0 = **33.510322**, sph 1.5 = **14.137167**, sph 1.0 = **4.188790**,
cyl 1×2 = **6.283185**, cyl 1.5×4 = **28.274334**, cyl 2×6 = **75.398224**, cyl 1.2×6 = **27.143361**,
cyl 2.8×6 = **147.780518**, cyl 3×6 = **169.646003**, cone 1.5×3 = **7.068583**, cone 1×2 = **2.094395**.

**Kernel constants that these cells are aimed at** (read from source, not assumed):
- `brep.cpp:3318` `tol3 = max(tolerance*50, diag*5e-4)` where `diag` = the **target surface's bbox
  diagonal**. With the default `tolerance` path this yields, for our vocabulary:
  4×4 planar face (diag 5.6569) → **2.83e-3**; box 4³ (6.9282) → 3.46e-3; cyl 1.5×6 (7.3485) →
  3.67e-3; sph 2.5 (8.6603) → 4.33e-3; tor 2/0.8 (8.0796) → 4.04e-3.
  *Prediction (not yet measured): every eps-sweep family below has its first stability cliff at
  |e| ≈ 3e-3, because a true section separation below tol3 is inside the chain-join band.*
- `Tolerance::ABSOLUTE = 1e-9`, `ANGULAR = 1e-6`, `ZERO_TOLERANCE = 1e-12` (`src/tolerance.h:31-39`).
  The `tolerance*50` floor = 5e-8 **overtakes** `diag*5e-4` once `diag < 1e-4` — see class SC.
- OCCT comparison points (from `audit_occt_intana-analytic.md`): cyl×cyl coplanarity /
  "Same" epsilon = `Precision::Confusion()` = **1e-7**; plane×sphere tangency band = `Epsilon(R)`
  ≈ 2.2e-16·R; transition dead bands **0.0 / 1e-9 / 1e-8 / 1e-7** depending on the pair;
  root-finder absolute epsilons 1.5e-12 / 1e-8 / 1e-10 (unnormalized → scale-dependent).

**Result measurements** the harness must produce per op-cell (all already exist):
`volume`, `shell_count_of()` (`main_7.cpp:40` — connected components over shared edges),
`naked edge count`, `face_count()`, and the operand-relative partition residual
(`corpus/invariants.py`). Reference face counts for IDENTITY cells: box 6, cylinder 3, cone 2,
sphere 1, torus 1.

---

## 1. ACCEPTANCE RULES (per op-cell verdict vector)

Every op-cell yields a 6-bit verdict. A cell PASSES only if all applicable bits are set.

| bit | rule | threshold |
|---|---|---|
| **CLOSED** | naked edges == 0 **and** every edge carries exactly 2 trims with opposite orientation | exact (Law 6: check after every stage, not only at the end) |
| **SOLIDS** | `shell_count_of(result) == expected_solids` from the cell row | exact integer. **Multi-solid results are VALID** — reference data shows OCCT itself returns 2 solids for chairs z30/z45/z37/z63 cut and 3 for x13y29 common (`validation/OCCT_TRUTH.md`). A harness that asserts 1 is wrong. |
| **VOLUME** | `\|V − V*\| / S ≤ τ`, `S = max(V*, 1e-9·vol(A))` | τ = **1e-6** for all-planar cells (exact rational truth), **1e-3** for cells with a curved section (kernel trim resampling costs ~1e-3 relative — `corpus/invariants.py:41`) |
| **EMPTY** | cells whose expected result is empty must return **0 faces, 0 solids, volume exactly 0.0** | exact — an epsilon sliver is a FAIL (OCCT `-s empty` semantics) |
| **IDENTITY** | cells whose expected result is an untouched operand: volume matches that operand to 1e-9 relative, solids == 1, and the face count is either the operand's count or the operand's count + imprint faces (record and freeze on the first correct run) | 1e-9 rel on volume |
| **INVARIANT** | `vol(cut)+vol(common) = vol(A)`; `vol(fuse) = vol(A)+vol(B)−vol(common)`; `common(A,B) == common(B,A)`; `tuc(A,B) == cut(B,A)`; **twin cells** (seam twins, mirrored twins) must agree on volume and solids exactly and on face count up to the declared seam difference | 1e-3 rel (`PART_TOL` is 0.01 today; tighten to 1e-3 for analytic primitive cells) |

**Doctrine allowance for non-manifold results.** For contact-only fuses (edge or point contact) two
answers are legitimate: (a) one solid with a non-manifold edge/vertex, (b) two solids in a compound.
Either is accepted, but the choice must be **deterministic and identical across every cell of the
class and across its mirrored/rotated twins**. Flip-flopping between (a) and (b) inside a class is a
FAIL even when both individual answers are defensible.

**eps-sweep acceptance is a family property, not a per-point one.** For a family with reference
`V*(e) = C·|e|^p` (constants in §6):
1. **Sign discipline** — on the non-overlapping side of 0, `common` must be exactly empty and `cut`
   must be IDENTITY (bit-for-bit the same verdict at e = +1e-9 as at e = +1e-2).
2. **Power law** — for every `e` with `V*(e) ≥ 1e-9·vol(A)`: `|V/V* − 1| ≤ 0.05`.
3. **Monotonicity** — `V(e)` non-increasing as `|e|` decreases; no non-monotone excursion anywhere.
4. **Topological stability** — CLOSED and SOLIDS constant across the whole sweep except at the single
   declared transition (see the GZ table's "topology step" column).
5. **CLIFF metric** — `cliff(family) = max |e|` at which any of 1–4 fails. This number, per family,
   is the deliverable; target `cliff ≤ 1e-9` (i.e. no failure above 1e-9). Report it, don't pass/fail it.

Below `V*(e) < 1e-9·vol(A)` only rules 1, 3, 4 apply (rule 2 is meaningless: the reference volume is
below the resolution of an accumulated volume integral). Per family the volumetric floor sits at the offset
`|e| = (1e-9·vol(A)/C)^(1/p)`: **GZ-F 4.0e-9, GZ-A 1.7e-4, GZ-B 9.1e-5, GZ-C 2.8e-6, GZ-M 4.0e-3,
GZ-O 4.0e-3** — note the vertex- and apex-contact families (p=3) can only be gated volumetrically
down to 4e-3; below that they are pure topology tests.

---

## 2. CLASS FC — FACE-COINCIDENT (same-domain)

Targets Law 3 (coincidence is a first-class outcome) and the whole `occt_ff-posttreat-samedomain.md`
port. The defining property: **the intersection of the two surfaces is 2-dimensional, so SSI returns
no curve at all**. Everything must come from same-domain detection + boundary imprinting.

Operands:

```
fbox   {"box",      {4,4,4},   {0,0,0, 0,0,1, 0}}
fboxT  {"box",      {4,4,4},   {4,0,0, 0,0,1, 0}}          // whole-face flush at x=2
fboxP  {"box",      {2,2,2},   {3,0,0, 0,0,1, 0}}          // B's face strictly inside A's face
fboxH  {"box",      {4,4,4},   {4,2,0, 0,0,1, 0}}          // coincident plane, half overlap
fboxC  {"box",      {4,4,4},   {2,2,0, 0,0,1, 0}}          // two coincident z-planes + volume overlap
fboxC3 {"box",      {4,4,4},   {2,2,2, 0,0,1, 0}}          // CONTROL: 3-axis shift, no coincidence
fboxB  {"box",      {8,8,4},   {0,0,4, 0,0,1, 0}}          // B larger: A's whole top face inside B's bottom
fboxK  {"box",      {2,2,2},   {0,0,1, 0,0,1, 0}}          // inside A, top face flush -> blind pocket
fboxN  {"box",      {2,2,2},   {1,1,1, 0,0,1, 0}}          // inside A, 3 faces flush -> corner notch
fcyA   {"cylinder", {1,2},     {0,0,0, 0,0,1, 0}}
fcyB   {"cylinder", {1,2},     {0,0,2, 0,0,1, 0}}          // cap-on-cap, whole disk
fcyB90 {"cylinder", {1,2},     {0,0,2, 0,0,1, 90}}         // seam twin of fcyB
fcyS   {"cylinder", {0.5,2},   {0,0,2, 0,0,1, 0}}          // cap-on-cap, disk inside disk
fcyO   {"cylinder", {1,2},     {1,0,2, 0,0,1, 0}}          // cap-on-cap, lens overlap (two arcs)
fcyF   {"cylinder", {1.5,4},   {0,0,-2, 0,0,1, 0}}         // both caps flush with box z=±2
fcyL   {"cylinder", {1.5,6},   {0,0,-3, 0,0,1, 0}}
fcyU   {"cylinder", {1.5,6},   {0,0,0, 0,0,1, 0}}          // coaxial, half-overlapping: curved same-domain
fcyR90 {"cylinder", {1.5,6},   {0,0,-3, 0,0,1, 90}}        // identical solid, seam moved 90 deg
fcnA   {"cone",     {2,4},     {0,0,0, 0,0,1, 0}}
fcnB   {"cone",     {2,4},     {0,0,0, 1,0,0, 180}}        // base-on-base bicone (shared whole base disk)
fcnS   {"cone",     {1,2},     {0,0,2, 0,0,1, 0}}          // SAME apex + SAME half-angle: lateral subset
fcnT   {"cone",     {1.5,3},   {0,0,2, 0,0,1, 0}}          // base circle on box top face
fcnR90 {"cone",     {2,4},     {0,0,0, 0,0,1, 90}}
fsp    {"sphere",   {2.5},     {0,0,0, 0,0,1, 0}}
fsp37  {"sphere",   {2.5},     {0,0,0, 0,0,1, 37}}         // identical solid, seam meridian moved
ftor   {"torus",    {2,0.8},   {0,0,0, 0,0,1, 0}}
ftor37 {"torus",    {2,0.8},   {0,0,0, 0,0,1, 37}}         // u-seam moved
ftorX  {"torus",    {2,0.8},   {0,0,0, 1,0,0, 180}}        // identical solid, v-seam orientation flipped
```

| id | A | B | cut vol/solids | common | fuse | degeneracy targeted | naive-kernel failure |
|---|---|---|---|---|---|---|---|
| FC01 | fbox | fboxT | 64 / 1 (IDENTITY) | EMPTY | 128 / 1 | whole-face coincidence, opposite normals | no SSI curve → both faces survive into the fuse (one edge with 4 trims) or both are dropped (naked square hole) |
| FC02 | fbox | fboxP | 64 / 1 | EMPTY | 72 / 1 | B's face strictly inside A's face | A's 4×4 face needs an **inner loop** imprinted from B's outline; a section-only splitter never creates it |
| FC03 | fbox | fboxH | 64 / 1 | EMPTY | 128 / 1 | coincident plane, half overlap (straight boundary crossing the face) | partial same-domain: A's face must split into kept+consumed halves; naive keeps it whole → overlapping coincident faces |
| FC04 | fbox | fboxC | 48 / 1 | 16 / 1 | 112 / 1 | 2 coincident planes **plus** genuine volume overlap | mixed transversal+coincident FF on one pair; the coincident pair usually silently loses to the transversal one |
| FC05 | fbox | fboxC3 | 56 / 1 | 8 / 1 | 120 / 1 | CONTROL (no coincidence, 3-axis shift) | must stay green; regression anchor for the class |
| FC06 | fbox | fbox | **EMPTY** | 64 / 1 (IDENTITY) | 64 / 1 (IDENTITY) | A-op-A, planar | cut(A,A) returning slivers instead of nothing is the canonical same-domain failure |
| FC07 | fsp | fsp | EMPTY | 65.449847 / 1 | 65.449847 / 1 | A-op-A, sphere (seam + 2 poles) | — |
| FC08 | fcyL | fcyL | EMPTY | 42.411501 / 1 | 42.411501 / 1 | A-op-A, cylinder (seam + 2 cap circles) | — |
| FC09 | fcnA | fcnA | EMPTY | 16.755161 / 1 | 16.755161 / 1 | A-op-A, cone (seam + singular apex) | apex is a singular UV row; arrangement degenerates |
| FC10 | ftor | ftor | EMPTY | 25.266187 / 1 | 25.266187 / 1 | A-op-A, torus (both seams, corner vertex) | — |
| FC11 | fbox | fboxB | 64 / 1 | EMPTY | 320 / 1 | whole face of A inside a larger face of B | B's bottom face needs a 4×4 **hole**; same imprint gap as FC02, reversed roles |
| FC12 | fcyA | fcyB | 6.283185 / 1 | EMPTY | 12.566371 / 1 | annular **cap-on-cap** coincidence (OCCT ZD4) | the shared boundary is a circle that exists as a TRIM on both operands — an EE/VE coincidence, not an FF section; SSI-only kernels see nothing |
| FC13 | fcyA | fcyB90 | 6.283185 / 1 | EMPTY | 12.566371 / 1 | **seam twin** of FC12 (OCCT ZD5) | any divergence from FC12 is a seam-aliasing bug by construction (needs no oracle) |
| FC14 | fcyA | fcyS | 6.283185 / 1 | EMPTY | 7.853982 / 1 | cap disk strictly inside cap disk | circular inner loop imprint on a planar face |
| FC15 | fcyA | fcyO | 6.283185 / 1 | EMPTY | 12.566371 / 1 | cap-on-cap **lens** overlap (boundary = 2 circular arcs, area 1.228370) | curved partial same-domain boundary; arc-arc intersection must be found without any surface-surface section |
| FC16 | fbox | fcyF | 35.725666 / 1 | 28.274334 / 1 | 64 / 1 (IDENTITY) | **two** annular cap coincidences + a transversal lateral face | fuse volume must be exactly 64: any leak shows as vol > 64 |
| FC17 | fbox | fcnT | 64 / 1 | EMPTY | 71.068583 / 1 | cone base circle lying **on** a box face | cap-circle-on-plane (OCCT ZP1/ZF6 class) |
| FC18 | fcnA | fcnB | 16.755161 / 1 | EMPTY | 33.510322 / 1 | whole base disk shared by two cones (bicone) | the shared disk must annihilate; two apexes survive |
| FC19 | fbox | fboxK | 56 / 1 | 8 / 1 | 64 / 1 (IDENTITY) | containment + one flush face → **blind pocket** (1 shell, not a void) | kernels that create a void shell here produce an unopened cavity |
| FC20 | fbox | fboxN | 56 / 1 | 8 / 1 | 64 / 1 (IDENTITY) | containment + **three** flush faces → corner notch | triple coincidence at one vertex (VV/VE/EE all fire) |
| FC21 | fcyL | fcyU | 21.205750 / 1 | 21.205750 / 1 | 63.617251 / 1 | **curved** same-domain: identical lateral surface, overlapping trims, same orientation | requires surface-identity detection; SSI of a cylinder with itself is degenerate everywhere |
| FC22 | fcnA | fcnS | 14.660766 / 1 | 2.094395 / 1 | 16.755161 / 1 (IDENTITY) | curved same-domain on a **cone** (shared apex, shared half-angle, B's lateral ⊂ A's lateral) | same as FC21 plus a shared singular apex |
| FC23 | fsp | fsp37 | EMPTY | 65.449847 / 1 | 65.449847 / 1 | geometrically identical spheres, **different seam meridian** | any weld keyed on trim parameterisation (rather than 3-D points) fails; this is the minimal reproduction of the rotated-frame coincidence class (chairsROT) |
| FC24 | fcyL | fcyR90 | EMPTY | 42.411501 / 1 | 42.411501 / 1 | identical cylinders, seam 90° apart | — |
| FC25 | fcnA | fcnR90 | EMPTY | 16.755161 / 1 | 16.755161 / 1 | identical cones, seam 90° apart | — |
| FC26 | ftor | ftor37 | EMPTY | 25.266187 / 1 | 25.266187 / 1 | identical tori, u-seam 37° apart | — |
| FC27 | ftor | ftorX | EMPTY | 25.266187 / 1 | 25.266187 / 1 | identical tori, v-seam side flipped (180° about x) | mirrored seam parity: surface orientation matches but the (u,v) frames are reflected |

Already present in `edge_placements()` and only re-labelled here: FC01 (`face full`), FC02
(`face part`), FC04 (`face copl`), FC05 (`face copl3d`), FC06/07/08/10 (`eq boxbox/sphsph/cylcyl/tortor`),
FC16 (`cyl flush`). **18 of the 27 are new**, and the new ones carry the entire curved-same-domain and
seam-mismatch load.

---

## 3. CLASS EC — EDGE / LINE CONTACT (a straight edge or a rectilinear tangency)

```
ebox   {"box",      {4,4,4},   {0,0,0, 0,0,1, 0}}
eboxX  {"box",      {4,4,4},   {3.5,0,0, 0,0,1, 0}}                    // face plane x=1.5 tangent to ecyl
eboxI  {"cylinder", {1.5,6},   {0.5,0,-3, 0,0,1, 0}}                   // tangent to A's x=2 wall from INSIDE
eboxD  {"box",      {4,4,4},   {4.8284271247,0,0, 0,0,1, 45}}          // vertical edge lying in the plane x=2
eboxDC {"box",      {4,4,4},   {4.3284271247,0,0, 0,0,1, 45}}          // vertical edge tangent to cyl r=1.5
eboxE  {"box",      {4,4,4},   {4,4,0, 0,0,1, 0}}                      // vertical edge coincident with A's edge
eboxO  {"box",      {8,4,4},   {0,0,4.8284271247, 1,0,0, 45}}          // bottom edge crossing A's whole top face
ecyl   {"cylinder", {1.5,6},   {0,0,-3, 0,0,1, 0}}
econe  {"cone",     {2,4},     {0,0,0, 0,0,1, 0}}
eboxG  {"box",      {4,4,4},   {3.3888543820,0,1.6944271910, 0,1,0, -26.5650511771}}   // face plane 2x+z=4
```

`eboxG`'s face plane is `2x + z = 4`, verified to contain the cone generatrix from (2,0,0) to
(0,0,4) exactly (residual 0.0 at three sampled points) with the whole cone on the `2x+z < 4` side;
the box centre is at distance exactly 2.0 from that plane. Contact segment = the generatrix from
(2,0,0) to (0.705573,0,2.588854), length 2.894427.

| id | A | B | cut | common | fuse | degeneracy | naive failure |
|---|---|---|---|---|---|---|---|
| EC01 | eboxX | ecyl | 64 / 1 (IDENTITY) | EMPTY | 106.411501 | cylinder **tangent to a plane** along the line (1.5, 0, z), z∈[−2,2] | plane×cylinder with `sin(angle)` in the relaxation band (`IntAna_QuadQuadGeo:571-598`): OCCT emits 1 tangent line via NbSol==1; a marcher started on a double root walks off or produces two nearly-coincident lines |
| EC02 | ebox | eboxI | 35.725666 / 1 | 28.274334 / 1 | 78.137167 / 1 | same tangency but **from inside**, combined with a real volume overlap | the tangent line and the transversal cap sections meet at 2 points; junction closure (defect class C) |
| EC03 | ebox | eboxD | 64 / 1 (IDENTITY) | EMPTY | 128 | box **edge on box face** | edge-in-plane is a VE/EF interference, never an FF section |
| EC04 | ebox | eboxO | 64 / 1 (IDENTITY) | EMPTY | 192 | box edge crossing the **whole** face and overhanging both ends | the imprinted line reaches the face boundary at both ends: boundary-crossing chain, not a closed loop |
| EC05 | ebox | eboxE | 64 / 1 (IDENTITY) | EMPTY | 128 | two box **edges collinear** (edge-on-edge over the full 4-length) | EE same-domain; existing in `edge_pairs()` as `edge touch` |
| EC06 | ecyl | eboxDC | 42.411501 / 1 (IDENTITY) | EMPTY | 106.411501 | box edge **tangent to a cylinder** along a line | edge-on-curved-surface: the contact is a line on the box and a line on the cylinder, and its UV image on the cylinder is the seam-parallel iso at u=0 (aliases with the seam!) |
| EC07 | econe | eboxG | 16.755161 / 1 (IDENTITY) | EMPTY | 80.755161 | plane **tangent to a cone along a generatrix** | plane×cone NbSol==1 → OCCT splits the tangent line **at the apex into 2 GLines** (audit E6); a port emitting one line gets the apex vertex wrong |

---

## 4. CLASS PT — POINT CONTACT

```
psp    {"sphere",   {2.5},     {0,0,0, 0,0,1, 0}}
psp37  {"sphere",   {2.5},     {0,0,0, 0,0,1, 37}}                 // same solid, contact point off-seam
pboxS  {"box",      {4,4,4},   {4.5,0,0, 0,0,1, 0}}                // tangent at (2.5,0,0) = ON the seam
pboxN  {"box",      {4,4,4},   {0,0,4.5, 0,0,1, 0}}                // tangent at (0,0,2.5) = ON the north pole
pspX   {"sphere",   {1.0},     {3.5,0,0, 0,0,1, 0}}                // external tangency at (2.5,0,0)
pspY   {"sphere",   {1.0},     {1.5,0,0, 0,0,1, 0}}                // internal tangency at (2.5,0,0)
pbox   {"box",      {4,4,4},   {0,0,0, 0,0,1, 0}}
pboxV  {"box",      {4,4,4},   {4,4,4, 0,0,1, 0}}                  // vertex-on-vertex
pboxC  {"box",      {4,4,4},   {0,0,5.4641016151, 1,-1,0, 54.7356103172}}   // [AXIS] corner-down: vertex on face
pboxR  {"box",      {4,4,4},   {0,0,0, 0,1,0, 45}}                 // top edge runs along y at z=2.8284271
pboxR2 {"box",      {4,4,4},   {0,0,5.6568542495, 1,0,0, 45}}      // bottom edge runs along x at z=2.8284271
pcnD   {"cone",     {2,4},     {0,0,6, 1,0,0, 180}}                // apex down at z=2
pcnD2  {"cone",     {2,4},     {0,0,6.5, 1,0,0, 180}}              // apex at z=2.5 = sphere north pole
pcnD3  {"cone",     {2,4},     {0,0,8, 1,0,0, 180}}                // apex at z=4 = apex of pcnA
pcnA   {"cone",     {2,4},     {0,0,0, 0,0,1, 0}}                  // apex at (0,0,4)
pcnX   {"cone",     {2,4},     {5.5,0,0, 0,1,0, -90}}              // axis -x, apex at (1.5,0,0) = cyl seam, opening AWAY
pcyl   {"cylinder", {1.5,6},   {0,0,-3, 0,0,1, 0}}
pcylY  {"cylinder", {1.5,6},   {-3,3,0, 0,1,0, 90}}                // axis along x at y=3: point tangency at (0,1.5,0)
pspC   {"sphere",   {1.0},     {2.5,0,0, 0,0,1, 0}}                // tangent to pcyl at (1.5,0,0) = seam
ptor   {"torus",    {2,0.8},   {0,0,0, 0,0,1, 0}}
pspT   {"sphere",   {1.0},     {3.8,0,0, 0,0,1, 0}}                // tangent at (2.8,0,0) = torus seam CORNER
ptorT  {"torus",    {2,0.8},   {5.6,0,0, 0,0,1, 0}}                // outer equators touch at (2.8,0,0)
```

| id | A | B | cut | common | fuse | degeneracy | naive failure |
|---|---|---|---|---|---|---|---|
| PT01 | psp | pboxS | 65.449847 (IDENTITY) | EMPTY | 129.449847 | sphere tangent to a plane **at its seam meridian** | plane×sphere tangency band is `Epsilon(R)` ≈ 5.6e-16 here — anything looser returns a **circle of radius √(2R·δ)** instead of a point (audit §2.3: a 1e-9 error gives r≈4.5e-4) |
| PT02 | psp | pboxN | 65.449847 (IDENTITY) | EMPTY | 129.449847 | contact **at the singular pole** | pole + tangency: the UV image of the contact is a whole degenerate row |
| PT03 | psp37 | pboxS | 65.449847 (IDENTITY) | EMPTY | 129.449847 | **twin of PT01** with the contact off-seam | must agree with PT01 exactly; divergence = seam aliasing |
| PT04 | psp | pspX | 65.449847 (IDENTITY) | EMPTY | 69.638637 | sphere-sphere **external** point tangency | `IntSpSp` dead band 1e-8 (audit E3): a 1e-9 overlap gives a real circle, a 1e-9 gap gives nothing — the classifier must not invent either |
| PT05 | psp | pspY | 61.261057 / 1 | 4.188790 / 1 (IDENTITY of B) | 65.449847 (IDENTITY of A) | sphere-sphere **internal** point tangency (B inside A, touching) | common must be exactly B; cut must be A with an internal void that pinches to a point |
| PT06 | pbox | pboxV | 64 (IDENTITY) | EMPTY | 128 | vertex-on-vertex | VV interference only; both solids' corner vertices must weld into one |
| PT07 | pbox | pboxC | 64 (IDENTITY) | EMPTY | 128 | vertex-on-face **[AXIS]** (corner-down box, lowest vertex at z=−3.4641016151) | a vertex interior to a face: VF interference, the face gains an isolated interior vertex (a 0-D imprint most arrangements cannot represent) |
| PT08 | pbox | pcnD | 64 (IDENTITY) | EMPTY | 80.755161 | **cone apex** touching a plane | the apex is singular in UV *and* a tangency point; OCCT's `IntPCo` NbSol==1 path (`:3515-3596`) |
| PT09 | psp | pcnD2 | 65.449847 (IDENTITY) | EMPTY | 82.205008 | cone apex on the sphere's **north pole** | two singular points coincide: the nappe purge (`param >= paramapex`, audit §2.1) must keep this contact and flag `isTangent = false` |
| PT10 | pcnA | pcnD3 | 16.755161 (IDENTITY) | EMPTY | 33.510322 | apex-to-apex | both operands singular at the same point |
| PT11 | pcyl | pcnX | 42.411501 (IDENTITY) | EMPTY | 59.166662 | cone apex on the cylinder **seam** | apex + seam + tangency; cyl×cone analytic path cannot produce isolated tangent points at all (`IntAna_IntQuadQuad::NbPnt()` is always 0, audit §1) |
| PT12 | pcyl | pspC | 42.411501 (IDENTITY) | EMPTY | 46.600291 | sphere tangent to a cylinder at its **seam** | `IntCySp` dead band 1e-7 (audit E3) — the loosest of all the point-tangency bands |
| PT13 | ptor | pspT | 25.266187 (IDENTITY) | EMPTY | 29.454977 | sphere tangent to the torus at the **u-seam ∩ v-seam corner vertex** | the contact point is already a 4-trim corner of the torus's single face |
| PT14 | ptor | ptorT | 25.266187 (IDENTITY) | EMPTY | 50.532375 | torus-torus outer-equator point tangency (both seam corners coincide) | torus×torus has no analytic case beyond `Same` (audit §2.3) → falls to the walker, which is unstable at a tangency by design (Law 4) |
| PT15 | pboxR | pboxR2 | 64 (IDENTITY) | EMPTY | 128 | two box **edges crossing at a single point** (0,0,2.8284271) | EE interference whose result is a point, not a segment |
| PT16 | pcyl | pcylY | 42.411501 (IDENTITY) | EMPTY | 84.823002 | perpendicular equal cylinders tangent externally at (0,1.5,0) | cyl×cyl crossing at 90° with `Dist = R1+R2`: the audit's `RealSmall()` branch (`:1154-1168`) is effectively unreachable, so external tangency is caught only by the chord-collapse test inside the 2-line branch — a different code path from what a naive port expects |

Already present: PT04 (`sph tanext`), PT05 (`sph tanint`), PT06 (`vert touch`). **13 new.**

---

## 5. CLASS TG — TANGENT SURFACES (contact along a curve)

```
tcyA   {"cylinder", {1.5,6},   {0,0,-3, 0,0,1, 0}}
tcyP   {"cylinder", {1.5,6},   {3,0,-3, 0,0,1, 0}}              // parallel, EXTERNAL line tangency at x=1.5
tcyBig {"cylinder", {3,6},     {0,0,-3, 0,0,1, 0}}
tcyIn  {"cylinder", {1.5,6},   {1.5,0,-3, 0,0,1, 0}}            // INTERNAL line tangency at x=3
tcyU   {"cylinder", {0.5,6},   {2,0,-3, 0,0,1, 0}}              // unequal radii, external tangency at x=1.5
tsp    {"sphere",   {1.5},     {0,0,0, 0,0,1, 0}}               // equator circle == cylinder surface
ttor   {"torus",    {2,0.8},   {0,0,0, 0,0,1, 0}}
tboxT  {"box",      {8,8,4},   {0,0,2.8, 0,0,1, 0}}             // bottom face z=0.8 tangent along circle r=2
tcyI   {"cylinder", {1.2,6},   {0,0,-3, 0,0,1, 0}}              // tangent along the INNER equator (r = R−r)
tcyO   {"cylinder", {2.8,6},   {0,0,-3, 0,0,1, 0}}              // tangent along the OUTER equator (r = R+r)
ttorZ  {"torus",    {2,0.8},   {0,0,1.6, 0,0,1, 0}}             // tube-to-tube tangency along the circle r=2, z=0.8
tcnA   {"cone",     {2,4},     {0,0,0, 0,0,1, 0}}
tcnG   {"cone",     {2,4},     {3.2,0,1.6, 0,1,0, -53.1301023542}}   // shares apex + one generatrix
tcyG   {"cylinder", {1,6},     {3.7888543820,0,-1.3416407865, 0,1,0, -26.5650511771}}  // tangent along a generatrix
tspC   {"sphere",   {1.2360679775}, {0,0,1.2360679775, 0,0,1, 0}}    // inscribed in tcnA
tbox   {"box",      {4,4,4},   {0,0,0, 0,0,1, 0}}
tcyIns {"cylinder", {2,6},     {0,0,-3, 0,0,1, 0}}              // inscribed in tbox: 4 line tangencies
tspIns {"sphere",   {2.0},     {0,0,0, 0,0,1, 0}}               // inscribed in tbox: 6 point tangencies
```

`tcnG` verified: after the transform the apex returns to (0,0,4) (residual 2e-12) with axis
(−0.8, 0, 0.6); the generatrix (2,0,0)→(0,0,4) lies on B's surface to 1.3e-10 deg and a probe
interior to A is 2.34° outside B — interiors are disjoint, contact is exactly the shared generatrix.
`tcyG` verified: every sampled generatrix point is at distance 1.000000000000 from B's axis, and cone
points 0.05–0.30 rad off the generatrix are at 1.0024–1.0811 (external tangency, contact segment
(2,0,0)→(0.211146,0,3.577709), length 4).

| id | A | B | cut | common | fuse | degeneracy | naive failure |
|---|---|---|---|---|---|---|---|
| TG01 | tcyA | tcyP | 42.411501 (IDENTITY) | EMPTY | 84.823002 | equal-radius parallel cylinders, **external** line tangency | `Dist > \|R1−R2\|` 2-line branch with chord collapse `4R1²(1−cos²) < Tol²` → 1 line; a port using the documented `RealSmall()` outer-tangency branch gets **nothing** |
| TG02 | tcyBig | tcyIn | 127.234502 / 1 | 42.411501 / 1 (IDENTITY of B) | 169.646003 (IDENTITY of A) | unequal radii, **internal** line tangency (`d = R1 − R2`; OCCT bfuse_complex/J1 class) | `Dist > \|R1−R2\| − Tol` internal-tangent branch, ratio negated when R1 < R2 — sign errors here produce the tangent line on the wrong side |
| TG03 | tcyA | tcyU | 42.411501 (IDENTITY) | EMPTY | 47.123890 | unequal-radius parallel cylinders, **external** line tangency (fuse = 42.411501 + π·0.5²·6) | same branch as TG01 with asymmetric radii |
| TG04 | tcyA | tsp | 28.274334 / **2** | 14.137167 / 1 (IDENTITY of B) | 42.411501 (IDENTITY of A) | sphere tangent **inside** a cylinder along the equator circle | the cut is legitimately 2 solids touching along a circle; a kernel that welds the contact circle produces one wrong solid, one that drops a half produces vol 14.137167 |
| TG05 | ttor | tboxT | 25.266187 (IDENTITY) | EMPTY | 281.266187 | torus tangent to a **plane** along the circle of radius **R = 2** at z = 0.8 | a plane can only be tangent to a torus along the r-radius circles at z=±r — *not* at the inner/outer equators (those admit tangent **cylinders**, TG06/TG07). plane×torus parallel branch carries the **torus** axis as the circle normal (audit §2.3) |
| TG06 | ttor | tcyI | 25.266187 (IDENTITY) | EMPTY | 52.409548 | cylinder tangent along the **inner** equator (radius R−r = 1.2, z=0) | contact circle sits exactly where the torus's UV v-parameter is π: the "far seam" of the tube |
| TG07 | tcyO | ttor | 122.514331 / 1 | 25.266187 / 1 (IDENTITY of B) | 147.780518 (IDENTITY of A) | cylinder tangent along the **outer** equator (radius R+r = 2.8, z=0) — which is also the torus **v-seam** | tangency ON a seam: the section curve and the seam edge are the same locus (needs existing-edge adoption, PORT MAP #3) |
| TG08 | ttor | ttorZ | 25.266187 (IDENTITY) | EMPTY | 50.532375 | torus-torus tangency along a full circle (tube touching tube) | torus×torus has no analytic branch except `Same`; the walker must trace a curve it is unstable on |
| TG09 | tcnA | tcnG | 16.755161 (IDENTITY) | EMPTY | 33.510322 | cone-cone **common generatrix** (shared apex, axes 53.1301023542° apart) | `IntAna_QuadQuadGeo:1760-1884` requires apex-on-other-cone round-trips and emits `PChar` with **parameter 0 regardless of the true conic parameter** (audit trap 14) |
| TG10 | tcnA | tcyG | 16.755161 (IDENTITY) | EMPTY | 35.604717 | **cylinder tangent to a cone along a generatrix** | THE analytic dead zone: `Perform(Cyl,Con)` returns only Circle/NoGeometricSolution and `IntAna_IntQuadQuad` drops isolated tangent points, so OCCT itself yields **nothing** here (audit §2.1). Any port inheriting that ladder must add its own path |
| TG11 | tcnA | tspC | 8.844447 / 1 | 7.910714 / 1 (IDENTITY of B) | 16.755161 (IDENTITY of A) | sphere inscribed in a cone: tangent **along a circle** (radius 1.105573 at z=1.788854) **and at a point** (0,0,0 on the base plane) | two different tangency types on one pair; `PointAndCircle` (audit §2.3 `:1971-1992`) returns a zero-radius circle if both roots degenerate |
| TG12 | tbox | tcyIns | 13.734518 / **4** | 50.265482 / 1 | 89.132741 / 1 | cylinder inscribed in a box: **4 simultaneous line tangencies**, cut = 4 corner solids | 4 tangencies on one pair; the cut result is legitimately 4 solids meeting pairwise at… nothing (they are disjoint) |
| TG13 | tbox | tspIns | 30.489678 / 1 | 33.510322 / 1 | 64 (IDENTITY) | sphere inscribed in a box: **6 point tangencies**; the cut is ONE solid whose boundary self-touches at 6 points | see MS class: the result is non-manifold at 6 vertices and must not be split or welded away |

Already present: TG01 (`cyl tanline`), TG13 (`sph inscr`). **11 new.**

---

## 6. CLASS GZ — GRAZING / NEAR-TANGENT EPSILON SWEEPS

The point of this class is **not** any single value; it is the shape of `V(e)` and the location of the
stability cliff. Each family is one parametric operand pair; sweep
`e ∈ {±1e-2, ±1e-3, ±1e-4, ±1e-5, ±1e-6, ±1e-7, ±1e-8, ±1e-9, 0}` = 17 configurations × 3 ops.
Sign convention: `e > 0` = separated / non-overlapping side; `e < 0` = overlapping side (except
GZ-G/GZ-H/GZ-L/GZ-P/GZ-Q where only one side is geometrically meaningful, noted below).

All constants below are **computed**, and the closed forms were verified against the exact formulas
at 8 decades (script in the session scratchpad):

| fam | A | B(e) | overlap V*(e) closed form | p | C | topology step at e=0 |
|---|---|---|---|---|---|---|
| GZ-A | `sphere 2.5 @0` | `sphere 1.0 @(3.5+e,0,0)` | lens: `π(R+r−d)²(d²+2dr−3r²+2dR+6rR−3R²)/(12d)`, d = 3.5+e | 2 | `πRr/(R+r)` = **2.243995** | fuse solids 2→1 |
| GZ-B | `sphere 2.5 @0` | `box 4,4,4 @(4.5+e,0,0)` | cap: `πh²(3R−h)/3`, h = −e | 2 | `πR` = **7.853982** | fuse solids 2→1 |
| GZ-C | `cylinder 1.5,6 @(0,0,-3)` | `box 4,4,4 @(3.5+e,0,0)` | segment prism: `L·r²(θ−sinθcosθ)`, θ=acos((r+e)/r), L=4 | 1.5 | `L·(4/3)√(2r)` = **9.237604** | fuse solids 2→1 |
| GZ-D | `cylinder 1.5,6 @(0,0,-3)` | `cylinder 1.5,6 @(3+e,0,-3)` | `h·lens2(r,r,3+e)` | 1.5 | `h·(4/3)√r` = **9.797959** | fuse solids 2→1 |
| GZ-E | `cylinder 1,2 @0` | `cylinder 1,2 @(0,0,2+e)` | `πr²·\|e\|` (thin disk) | 1 | `πr²` = **3.141593** | fuse solids 2→1; at e=0 the caps are same-domain (FC12) |
| GZ-F | `box 4,4,4 @0` | `box 4,4,4 @(4+e,0,0)` | `16·\|e\|` (thin slab) | 1 | **16** | fuse solids 2→1; at e=0 the faces are same-domain (FC01) |
| GZ-G | `cylinder 1.5,6 @(0,0,-3)` | `cylinder 1.5+e,6 @(0,0,-3)` (e>0 only) | `cut(B,A)` = `6π((1.5+e)²−1.5²)` (thin tube) | 1 | `12πr` = **56.548668** | at e=0 the pair is `Same`; OCCT's cyl×cyl `Same` epsilon is **1e-7** — the cliff is *designed in* at 1e-7 |
| GZ-H | `cylinder 1.5,6 @(0,0,-3)` | `cylinder 1.5,6 @(e,0,-3)` (e>0 only) | `cut` = `πr²h − h·lens2(r,r,e)` (thin crescent) | 1 | `2rh` = **18** | `Dist ≤ Tol` → `Same` at 1e-7 (audit E5: axes missing by 1e-9 still get exact Steinmetz ellipses) |
| GZ-I | `cylinder 1.5,6 @(0,0,-3)` | `cylinder 1.5,6 @(0,0,-3, 0,1,0, e_deg)` (e in degrees) | no closed form | — | — | monotone + topology gate only; `A==0 \|\| B==0 → Same` (`:1239-1243`) |
| GZ-J | `box 4,4,4 @0` | `box 4,4,4 @(4,0,0, 0,0,1, φ_deg)` | wedge, `V = 8·φ_rad` (verified 7.999996–8.000138 over φ = 1e-9…1e-5 deg) | 1 (in rad) | **8** (= 0.139626340 per degree) | **near-coplanar face pair** — the direct primitive analogue of the chairsROT z15/x20 residue: at φ=0 the faces are exactly coincident (FC01), at φ≠0 they are two distinct planes meeting at an angle |
| GZ-K | `torus 2,0.8 @0` | `box 8,8,4 @(0,0,2.8+e)` | `4πR∫√(r²−z²)dz` from z=0.8+e to 0.8 | 1.5 | `(8/3)πR√(2r)` = **21.193788** | fuse solids 2→1 |
| GZ-L | `cylinder 1.5,6 @(0,0,-3)` | `sphere 1.5+e @0` (e>0 only) | protruding belt `(4π/3)((1.5+e)²−1.5²)^{3/2}` | 1.5 | `(4π/3)(2r)^{3/2}` = **21.765592** | at e≤0 the sphere is inside and tangent (TG04); at e>0 it pierces the lateral surface |
| GZ-M | `box 4,4,4 @0` | `box 4,4,4 @(4+e,4+e,4+e)` | `\|e\|³` | 3 | **1** | vertex contact; volumetric gate dies at \|e\| < 4e-3 |
| GZ-N | `box 4,4,4 @0` | `box 4,4,4 @(4+e,4+e,0)` | `4e²` | 2 | **4** | edge contact |
| GZ-O | `cone 2,4 @0` | `box 10,10,10 @(0,0,9−e)` | tip cone `π(e/2)²e/3` | 3 | `π/12` = **0.261799** | plane grazing **through the apex**: the removed piece is a similar cone of radius e/2 |
| GZ-P | `cone 2,4 @0` | `eboxG translated by −e along (2,0,1)/√5` (e>0 = cutting) | no closed form; quadrature gives V/e^1.5 = 8.4066 (stable over e = 1e-2…1e-5) | 1.5 | **8.4066** | plane near-tangent along a **generatrix**: the section is a near-degenerate parabola |
| GZ-Q | `cone 2,4 @0` | `tcyG translated by e along (2,0,1)/√5` | no closed form | — | — | cyl/cone near-generatrix-tangency = the IntCyCo dead zone under perturbation; monotone + topology gate only |
| GZ-R | `sphere 2.5 @0` | `sphere 1.0 @(1.5+e,0,0)` (e>0 = protruding) | protrusion = `vol(B) − lens(2.5,1,1.5+e)` | 2 | `πrR/(R−r)` = **5.235988** | internal tangency from inside; at e<0 B is strictly inside (CN class) |

Reference values (from the table, exact): at |e| = 1e-3 / 1e-5 / 1e-7 / 1e-9 the overlap volumes are
GZ-A 2.2440e-06 / 2.2440e-10 / 2.2440e-14 / 2.2440e-18; GZ-C 2.9212e-04 / 2.9212e-07 / 2.9212e-10 /
2.9212e-13; GZ-F 1.6000e-02 / 1.6000e-04 / 1.6000e-06 / 1.6000e-08; GZ-J 8.0000e-03 / 8.0000e-05 /
8.0000e-07 / 8.0000e-09; GZ-M 1.0000e-09 / 1.0000e-15 / 1.0000e-21 / 1.0000e-27.

**Measured caveat on the reference formulas themselves.** The closed forms for GZ-D, GZ-H and GZ-K
lose relative accuracy below |e| ≈ 1e-6 through catastrophic cancellation (evaluated in double,
GZ-D at e = −1e-8 returns 3.427e-11 against the asymptote 9.238e-12, GZ-K at e = −1e-9 returns a
*negative* −3.365e-12). **Below |e| = 1e-6 the harness must use the asymptote `C|e|^p`, not the
closed form.** This is a property of the reference, not of the kernel.

**Vanishing-order ladder** (the reason these families are the right sweep set): as the contact
degenerates, the overlap volume vanishes as `|e|^p` with p = 1 for face contact (GZ-E/F/G/H/J),
p = 1.5 for curve tangency against a curved operand (GZ-C/D/K/L/P), p = 2 for point tangency and edge
contact (GZ-A/B/N/R), p = 3 for vertex and apex contact (GZ-M/O). **The higher p is, the earlier the
volumetric signal drops below the noise and the more purely topological the test becomes** — which is
exactly the order in which a stitch-based kernel loses the geometry.

---

## 7. CLASS SP — SEAM AND POLE CROSSINGS

```
ssp    {"sphere",   {2.5},     {0,0,0, 0,0,1, 0}}
ssp37  {"sphere",   {2.5},     {0,0,0, 0,0,1, 37}}         // seam rotated off the cut region
sboxP  {"box",      {8,8,4},   {0,0,3.5, 0,0,1, 0}}        // half-space z>=1.5: cap over the NORTH POLE
sboxS  {"box",      {1,10,10}, {0,0,0, 0,0,1, 0}}          // slab |x|<=0.5 through BOTH poles
sboxY  {"box",      {10,10,10},{0,5,0, 0,0,1, 0}}          // half-space y>=0: the cut plane CONTAINS the seam
sboxX  {"box",      {6,6,6},   {4.5,0,0, 0,0,1, 0}}        // half-space x>=1.5: cap crossing the seam
sboxZ  {"box",      {10,10,10},{0,0,5, 0,0,1, 0}}          // half-space z>=0
sboxW  {"box",      {8,8,10},  {4,0,0, 0,0,1, 0}}          // half-space x>=1.0
sboxE  {"box",      {10,10,10},{7.3,0,0, 0,0,1, 0}}        // half-space x>=2.3
sdrill {"cylinder", {0.5,10},  {0,0,-5, 0,0,1, 0}}         // drilled through both poles / the apex
scyl   {"cylinder", {1.5,6},   {0,0,-3, 0,0,1, 0}}
scylI  {"cylinder", {0.75,6},  {0,0,-3, 0,0,1, 0}}         // coaxial: seam ON seam
scylI9 {"cylinder", {0.75,6},  {0,0,-3, 0,0,1, 90}}        // twin: seams 90 deg apart
scone  {"cone",     {2,4},     {0,0,0, 0,0,1, 0}}
stor   {"torus",    {2,0.8},   {0,0,0, 0,0,1, 0}}
```

| id | A | B | cut | common | fuse | degeneracy | naive failure |
|---|---|---|---|---|---|---|---|
| SP01 | ssp | sboxP | 58.643063 / 1 | 6.806784 / 1 | 314.643063 / 1 | cut removing the **north pole** cap (h = 1.0) | the section circle is fine, but the kept piece contains a singular UV row; the arrangement's degenerate row is usually pruned as a zero-length edge |
| SP02 | ssp | sboxS | 46.076692 / **2** | 19.373155 / 1 | 146.076692 / 1 | slab removing **both poles**, cutting the seam twice → 2 solids | multi-solid + 2 poles + 2 seam crossings on one cell |
| SP03 | ssp | sboxY | 32.724923 / 1 | 32.724923 / 1 | 1032.724923 / 1 | cut plane **contains** the seam meridian **and both poles** | the section curve lies exactly ON the existing seam edge: an EE coincidence. Without existing-edge adoption the kernel emits a duplicate edge on the same locus → 4 trims |
| SP04 | ssp | sdrill | 61.562392 / 1 (genus 1) | 3.887455 / 1 | 69.416373 / 1 | drilled through **both poles** (napkin ring, `(4π/3)(R²−a²)^{3/2}`) | both poles removed; the result is genus-1 and the surviving sphere face is an annulus in UV |
| SP05 | scyl | sboxW | 37.765278 / 1 | 4.646223 / 1 | 677.765278 / 1 | plane cut **crossing** the cylinder seam (segment prism, θ = acos(2/3)) | the section on the cylinder is one curve in 3-D but **two** in UV (it wraps past u=0); a per-face chain closer invents a chord across the parameter jump (defect class B) |
| SP06 | scyl | sboxY | 21.205750 / 1 | 21.205750 / 1 | 1021.205750 / 1 | cut plane **contains** the cylinder seam and the axis | seam-coincident section, as SP03 |
| SP07 | scone | sboxY | 8.377580 / 1 | 8.377580 / 1 | 1008.377580 / 1 | cut plane contains **the apex and the seam** | plane through the apex → OCCT splits into **2 or 4 GLines** (audit E6); a port emitting 1 or 2 gets the apex vertex count wrong |
| SP08 | scone | sdrill | 14.137167 / 1 (genus 1) | 2.617994 / 1 (= π·0.25·3 + π·0.25·1/3) | 21.991149 / 1 | drilling out the **apex** (the tip above z=3 is entirely inside the drill) | the singular point is *removed*: the result's cone face is an annulus with no pole |
| SP09 | stor | sboxZ | 12.633094 / 1 | 12.633094 / 1 | 1012.633094 / 1 | cut plane **coincident with the v-seam** (the outer equator at z=0 lies exactly in it) | the section curve is the v-seam itself, twice (inner and outer equators): the worst seam-aliasing configuration in the vocabulary |
| SP10 | stor | sboxY | 12.633094 / 1 | 12.633094 / 1 | 1012.633094 / 1 | cut plane contains **both u-seams** (u=0 and u=π) | same as SP09 in the other parameter |
| SP11 | stor | sboxE | 24.227284 / 1 | 1.038903 / 1 | 1024.227284 / 1 | half-space x ≥ 2.3 removing a lens of tube that **crosses both seams and the corner vertex** | the removed piece straddles the (u,v) origin — the one place both periodic directions wrap |
| SP12 | ssp | sboxX | 58.643063 / 1 | 6.806784 / 1 | 274.643063 / 1 | cap cut whose boundary circle **crosses the seam** at 2 points | — |
| SP13 | ssp37 | sboxX | 58.643063 / 1 | 6.806784 / 1 | 274.643063 / 1 | **twin of SP12** with the seam rotated 37° away | volumes and solids must match SP12 exactly; face counts may differ only by seam placement |
| SP14 | scyl | scylI | 31.808626 / 1 (tube) | 10.602875 / 1 | 42.411501 (IDENTITY) | coaxial tube: the two **seams lie in the same half-plane** (seam-on-seam) | two seam edges at different radii but the same u: alias keys that quantise u collapse them |
| SP14b | scyl | scylI9 | 31.808626 / 1 | 10.602875 / 1 | 42.411501 | **twin** of SP14 with the inner seam 90° away | must match SP14 exactly |

---

## 8. CLASS MS — MULTI-SOLID AND SELF-TOUCHING RESULTS

**Reference fact (`validation/OCCT_TRUTH.md`, independent OCCT front end): chairs cut is 2 solids for
z15/z30/z45/z37/z63, common is 2 solids for z90/y30/z37/z63 and 3 for x13y29.** Multi-solid results
are correct answers; a harness asserting `solids == 1` fails valid geometry.

```
mbox   {"box",      {4,4,4},   {0,0,0, 0,0,1, 0}}
mdia   {"box",      {2.8284271247,2.8284271247,6}, {0,0,0, 0,0,1, 45}}   // inscribed diamond: vertices ON the 4 walls
mrot   {"box",      {4,4,4},   {0,0,0, 0,0,1, 45}}                       // same size, rotated 45: octagon common
mslab  {"box",      {2,8,8},   {0,0,0, 0,0,1, 0}}
mspIns {"sphere",   {2.0},     {0,0,0, 0,0,1, 0}}
mcyl   {"cylinder", {1.5,6},   {0,0,-3, 0,0,1, 0}}
msp    {"sphere",   {1.5},     {0,0,0, 0,0,1, 0}}
mtor   {"torus",    {2,0.8},   {0,0,0, 0,0,1, 0}}
mtsl   {"box",      {1,10,10}, {0,0,0, 0,0,1, 0}}
mbig   {"box",      {10,10,10},{0,0,0, 0,0,1, 0}}
mvoid  {"box",      {4,4,4},   {0,0,0, 0,0,1, 0}}
mvsp   {"sphere",   {2.5},     {0,0,0, 0,0,1, 0}}
mdis   {"box",      {2,2,2},   {6,0,0, 0,0,1, 0}}
mdrill {"cylinder", {1,10},    {0,0,-5, 0,0,1, 0}}
mspA   {"sphere",   {2.5},     {0,0,0, 0,0,1, 0}}
mspB   {"sphere",   {1.0},     {3.5,0,0, 0,0,1, 0}}
```

| id | A | B | cut | common | fuse | what makes it hard |
|---|---|---|---|---|---|---|
| MS01 | mbox | mdia | 32 / **4** (8 each) | 32 / 1 | 80 / 1 | the 4 pieces touch **pairwise along 4 vertical lines** (the diamond's vertices sit on the wall midpoints): a legitimately non-manifold cut. Accept 4 solids **or** 1 solid with 4 non-manifold edges; not a weld |
| MS02 | mbox | mslab | 32 / **2** (16 each) | 32 / 1 | 160 / 1 | trivial 2-solid control: if this fails, the multi-solid path is absent entirely |
| MS03 | mbox | mspIns | 30.489678 / **1** | 33.510322 / 1 | 64 (IDENTITY) | inscribed sphere: the cut is ONE solid whose boundary **pinches at 6 points**. Splitting it into 8 corner pieces is wrong (they are connected along the box edges) |
| MS04 | mcyl | msp | 28.274334 / **2** (14.137167 each) | 14.137167 / 1 | 42.411501 (IDENTITY) | the 2 pieces touch **along a full circle** — the highest-dimensional self-contact in the set |
| MS05 | mspA | mspB | 65.449847 / 1 | EMPTY | 69.638637 / 1 or 2 | fuse of externally tangent spheres: 1 pinched solid or 2 solids — declare and be consistent (PT04 duplicate, listed here for the doctrine bit) |
| MS06 | mbox | mrot | 10.980664 / **4** (2.745166 each) | 53.019336 / 1 | 74.980664 / 1 | the 4 corner triangles are **disjoint** (unlike MS01) — same-looking configuration, different correct answer. Octagon area 13.254834 = 16 − 2(4−2√2)² |
| MS07 | mtor | mtsl | 21.195316 / **2** | 4.070871 / **2** | 121.195316 / 1 | a torus cut by a slab: **both** cut and common are 2 solids, and the pieces are curved |
| MS08 | mbig | mvoid | 936 / 1 solid **2 shells** | 64 / 1 | 1000 (IDENTITY) | inner void. `kb/occt2_occt-test-mining.md` PORT MAP #8 records that our combine has **no inner-shell support** — this is the declared frontier cell |
| MS09 | mbig | mvsp | 934.550153 / 1 solid **2 shells** | 65.449847 / 1 | 1000 (IDENTITY) | curved inner void |
| MS10 | mbox | mdis | 64 (IDENTITY) | EMPTY | 72 / **2** | disjoint fuse must be 2 solids, not a merged hull or a dropped operand |
| MS11 | mbox | mdrill | 51.433629 / 1 (**genus 1**) | 12.566371 / 1 | 82.849556 / 1 | genus change: Euler characteristic of the result differs from both operands |
| MS12 | mtor | mdrill | 25.266187 (IDENTITY, no contact) | EMPTY | 56.682114 / **2** | operands **linked but disjoint** (drill through the torus hole, radius 1 < R−r = 1.2, clearance 0.2): zero section, and the fuse is two topologically **linked** solids. Present today as `tor linked` |
| MS13 | mbox | (TG12 `tcyIns`) | 13.734518 / **4** | 50.265482 / 1 | 89.132741 / 1 | tangency **and** multi-solid on one cell (cross-listed from TG12) |

---

## 9. CLASS CN — CONTAINMENT EXTREMES

```
cbox   {"box",      {4,4,4},   {0,0,0, 0,0,1, 0}}
csml   {"box",      {2,2,2},   {0,0,0, 0,0,1, 0}}
cbig   {"box",      {10,10,10},{0,0,0, 0,0,1, 0}}
cshl   {"box",      {3.999998,3.999998,3.999998}, {0,0,0, 0,0,1, 0}}   // 1e-6 wall
cspA   {"sphere",   {2.5},     {0,0,0, 0,0,1, 0}}
cspS   {"sphere",   {2.499999},{0,0,0, 0,0,1, 0}}                      // 1e-6 wall
ccyA   {"cylinder", {1.5,6},   {0,0,-3, 0,0,1, 0}}
ccyS   {"cylinder", {1.499999,6}, {0,0,-3, 0,0,1, 0}}                  // 1e-6 wall, caps COINCIDENT
```

| id | A | B | cut | common | fuse | note |
|---|---|---|---|---|---|---|
| CN01 | cbox | csml | 56 / 1 solid **2 shells** | 8 / 1 | 64 (IDENTITY) | B strictly inside A (present as `contain`) |
| CN02 | csml | cbox | **EMPTY** | 8 / 1 (IDENTITY of A) | 64 (IDENTITY of B) | A strictly inside B: cut must return **nothing at all** |
| CN03 | cbox | cbox | EMPTY | 64 (IDENTITY) | 64 (IDENTITY) | identical (= FC06; the idempotence triple) |
| CN04 | cbox | `box 2,2,2 @(6,0,0)` | 64 (IDENTITY) | EMPTY | 72 / 2 | disjoint (= MS10) |
| CN05 | cbox | `box 2,2,2 @(0,0,1)` | 56 / 1 | 8 / 1 | 64 (IDENTITY) | inside, touching on one face → blind pocket, **1 shell** (= FC19) |
| CN06 | cbox | `box 2,2,2 @(1,1,1)` | 56 / 1 | 8 / 1 | 64 (IDENTITY) | inside, touching on three faces → corner notch (= FC20) |
| CN07 | cspA | `sphere 1.0 @(1.5,0,0)` | 61.261057 / 1 | 4.188790 / 1 | 65.449847 (IDENTITY) | inside, touching at **one point** (= PT05): the void pinches to a vertex |
| CN08 | ccyA | `sphere 1.5 @0` | 28.274334 / 2 | 14.137167 / 1 | 42.411501 (IDENTITY) | inside, touching along **a circle** (= TG04) |
| CN09 | cbox | cshl | **9.599995e-05** / 1 solid 2 shells | 63.999904000 / 1 | 64 (IDENTITY) | 1e-6 **wall thickness** — the void surface is 1e-6 from the outer surface everywhere (tol3 for this face is 2.83e-3, i.e. **2800× the wall**: predicted total collapse; this cell measures how bad) |
| CN10 | cspA | cspS | **7.853978e-05** / 2 shells | 65.449768 / 1 | 65.449847 (IDENTITY) | 1e-6 curved wall |
| CN11 | ccyA | ccyS | **5.654864e-05** / 2 shells (π(1.5²−1.499999²)·6) | 42.411444 / 1 | 42.411501 (IDENTITY) | 1e-6 wall **with coincident cap planes** — thin wall + same-domain together (the cut is a tube, not a void: the caps are flush) |
| CN12 | cbig | `box 4,4,4 @(0,0,3)` | 936 / 1 (open pocket, 1 shell) | 64 / 1 | 1000 (IDENTITY) | containment vs. **partial protrusion** control: B half inside → the cut is a pocket, not a void. Distinguishes a real void from a pocket |

---

## 10. CLASS SC — SCALE EXTREMES

Tolerance behaviour in this kernel is **partly relative and partly absolute**: `tol3 = max(tolerance*50,
diag*5e-4)` (`brep.cpp:3318`). With `tolerance = Tolerance::ABSOLUTE = 1e-9` the absolute floor
`5e-8` overtakes the relative term once `diag < 1e-4`. **Prediction: every cell whose bbox diagonal
is below ~1e-4 units enters a regime where the chain-join band exceeds a fixed fraction of the model,
and below diag ≈ 5e-5 the band exceeds the model itself.** The SC family measures where that starts.

| id | construction | expected | what it measures |
|---|---|---|---|
| SC01 | **FC01 scaled**: `box 4s,4s,4s @0` vs `box 4s,4s,4s @(4s,0,0)`, s ∈ {1e-6, 1e-4, 1e-2, 1, 1e2, 1e4, 1e6} | cut 64s³, common EMPTY, fuse 128s³; **face and solid counts identical at every s** | scale-equivariance of whole-face coincidence. Any s where counts change is a scale defect |
| SC02 | **TG01 scaled**: `cylinder 1.5s,6s @(0,0,-3s)` vs `cylinder 1.5s,6s @(3s,0,-3s)`, same s set | cut 42.411501s³, common EMPTY, fuse 84.823002s³ | scale-equivariance of **line tangency** (the analytic branch thresholds `RealSmall()`, `Tol²` chord collapse are absolute) |
| SC03 | **PT04 scaled**: `sphere 2.5s` vs `sphere 1.0s @(3.5s,0,0)` | cut 65.449847s³, common EMPTY, fuse 69.638637s³ | scale-equivariance of **point tangency** (plane×sphere band is `Epsilon(R)`, i.e. relative; sphere×sphere is 1e-8, i.e. absolute — these must diverge at some s, and this cell finds it) |
| SC04 | **GZ-F at fixed RELATIVE eps**: `box 4s,4s,4s @0` vs `box 4s,4s,4s @(4s(1−1e-6),0,0)` over the same s set | overlap = `(4s)²·(4s·1e-6)` = **6.4e-5·s³**; cut `64s³ − 6.4e-5 s³`, fuse `128s³ − 6.4e-5 s³` | separates relative from absolute tolerance behaviour: if the kernel were purely relative this cell would be s-invariant after dividing out s³ |
| SC05 | ratio 1e6, **coincident**: `box 1000,1000,1000 @0` vs `sphere 0.0005 @(0,0,500)` (half-embedded in the top face) | cut 1e9 − 2.617994e-10, common **2.617994e-10** / 1, fuse 1e9 + 2.617994e-10 | operand ratio 2e6. **The volume gate must be relative to the SMALL operand** (2.6e-10/1e9 = 2.6e-19 is below double resolution on the large one) |
| SC06 | ratio 1e6, **tangent**: `sphere 1000 @0` vs `sphere 0.001 @(1000.001,0,0)` | cut 4.188790e9 (IDENTITY), common EMPTY, fuse 4.188790e9 + 4.188790e-9 | curvature ratio 1e6 at a tangency point |
| SC07 | **thin plate**: `box 1000,1000,0.001 @0` vs `cylinder 100,10 @(0,0,-5)` | cut 968.584073 / 1, common 31.415927 / 1, fuse 315127.849432 / 1 | aspect ratio 1e6 within one operand; the plate's faces have diag ≈ 1414 → tol3 ≈ 0.707, i.e. **707× the plate thickness** (predicted collapse) |
| SC08 | **needle**: `box 4,4,4 @0` vs `cylinder 0.001,20 @(0,0,-10)` | cut 63.999987434 / 1, common 1.256637e-05 / 1, fuse 64.000050265 / 1 | needle through a normal-scale solid: section circles of radius 1e-3 against a face whose tol3 is 2.83e-3 (**the section is smaller than the join band**) |

---

## 11. WHAT EACH CLASS PROVES, MAPPED TO OUR OWN DEFECT TAXONOMY

| class | `p1_attack_plan.md` defect classes it should provoke | mechanism it gates |
|---|---|---|
| FC (coincidence) | A (one-sided seg coverage), F (unmerged identical mate pair) | M1 SEG-ADOPT; Law 3 (typed ON verdicts, common blocks) |
| EC / TG (tangency) | B (divergent invented closure), G | Law 4 (discovery ≠ refinement): tangent branches must be *found*, not marched into |
| PT (point contact) | E-adjacent (verdict drops), G | VV/VF interference passes; `IntPatch_Point(isTangent)` equivalent |
| GZ (grazing) | A + B together; the z15/x20/z37 signature | M4 TOL-GROWTH; the tol3 cliff is the measurable output |
| SP (seam/pole) | B (per-face chain closure across a parameter jump), C (junction undershoot) | M2 JUNCTION-CLOSE; seam decomposition in `brep_section` |
| MS (multi-solid) | — (harness defect class: results silently reduced to 1 solid) | combine/classify: inner shells (PORT MAP #8), non-manifold contact preservation |
| CN (containment) | D (micro/triangle hole) at 1e-6 walls | combine void support + per-entity tolerance |
| SC (scale) | all of them, re-run at other magnitudes | tolerance model: which constants are relative and which are absolute |

**The single highest-value cell in the file** is **GZ-J** (two boxes flush at angle φ). It is the
minimal, fully analytic reproduction of the chairsROT residue: at φ = 0 the faces are exactly
coincident and the kernel's control config (z30x20 analogue) passes; at φ = 1e-3…1° they are two
distinct nearly-coplanar planes and every rotated chairs config lives. `V(φ) = 8·φ_rad` exactly, with
no curved geometry, no imported STEP, no freeform surfaces — if GZ-J's cliff is at 3e-3 while the
chairs frontier is 108, the two numbers are the same defect and GZ-J is a 40-line repro.

---

## 12. CENSUS

| class | rows | cross-listed | distinct | new | already in `edge_pairs()` |
|---|---:|---:|---:|---:|---:|
| FC face-coincident | 27 | 0 | 27 | 18 | 9 (`face full/part/copl/copl3d`, `eq boxbox/sphsph/cylcyl/tortor`, `cyl flush`) |
| EC edge/line contact | 7 | 0 | 7 | 6 | 1 (`edge touch`) |
| TG tangent surfaces | 13 | 0 | 13 | 11 | 2 (`cyl tanline`, `sph inscr`) |
| PT point contact | 16 | 0 | 16 | 13 | 3 (`sph tanext`, `sph tanint`, `vert touch`) |
| SP seam/pole | 15 | 0 | 15 | 15 | 0 |
| MS multi-solid / non-manifold | 13 | 3 (MS03=TG13, MS05=PT04, MS13=TG12) | 10 | 7 | 3 (`contain`, `disjoint`, `tor linked`) |
| CN containment | 12 | 7 (CN01=MS08, CN03=FC06, CN04=MS10, CN05=FC19, CN06=FC20, CN07=PT05, CN08=TG04) | 5 | 5 | 0 |
| SC scale | 8 | 0 | 8 | 8 | 0 |
| **subtotal** | **111** | **10** | **101** | **83** | **18** |
| GZ grazing sweeps | 18 families × 17 offsets | — | **306 configs** | 306 | 0 |
| **total** | | | **407 configs** | | **1221 op-cells** |

The 18 pairs of today's `edge_pairs()` grid are exactly the subset marked "already in
`edge_pairs()`" — every one of them survives with identical parameters, so this design is a strict
superset of the existing SESSION_EDGE grid and can replace it in place.

Suggested execution order (risk-ordered, mirrors `p1_attack_plan.md` §4):
1. **FC + CN** — coincidence is the largest class and needs no new SSI code, only same-domain typing.
2. **GZ-F, GZ-J, GZ-E** — the three p=1 families; they measure the tol3 cliff directly and cheaply.
3. **SP** — seam/pole, gated by the seam decomposition already landed in `brep_section`.
4. **EC + TG + PT** — tangency, hardest; TG10 (cyl/cone generatrix) is knowingly unreachable through
   the OCCT analytic ladder and needs its own path.
5. **MS + SC** — MS08/MS09 depend on inner-shell support (declared frontier); SC after everything else,
   because it multiplies every earlier failure.

---

## 13. HONESTY APPENDIX

- **Computed here** (python, full double precision, formulas cross-checked against their asymptotes at
  8 decades): every volume, area, lens/cap/segment/napkin/torus-slice value; all GZ power-law constants
  `C` and exponents `p`; the placement transforms for TG09/TG10/EC07/PT07 (verified by sampling: apex
  residual 2e-12, generatrix-on-surface residual 1.3e-10 deg, tangent distance 1.000000000000, plane
  residual 0.0); GZ-J's `V = 8·φ_rad` (numeric polygon clipping, ratio 7.999996–8.000138 over
  φ = 1e-9…1e-5 deg); GZ-P's `C = 8.4066` (quadrature, stable over e = 1e-2…1e-5); the tol3 values for
  the standard operands.
- **Read from source, not assumed**: Place/xf semantics (`main_7.cpp:66,194`), primitive centring/seam/
  pole conventions (`src/brep.cpp:228,337,425,487,555`, `src/primitives.cpp:227,251,275,305`), face
  counts (`src/brep_test.cpp:214,227,241,252`), `tol3` formula (`brep.cpp:3318`), tolerance constants
  (`src/tolerance.h:31-39`), the existing grids (`main_7.cpp:69,95,124,141,157,201`), the multi-solid
  reference data (`validation/OCCT_TRUTH.md`), acceptance tolerances (`corpus/invariants.py:40-43`).
- **Predictions, explicitly not measurements**: every "naive failure" column entry; the tol3-driven
  cliff at |e| ≈ 3e-3; the diag < 1e-4 scale breakdown; the claim that GZ-J and chairsROT are the same
  defect. None of these cells has been run — this file is the design, not a result.
- **Not derivable in closed form** (must come from the oracle or from quadrature at authoring time):
  GZ-I, GZ-Q, SP11's exact common (1.038903 here is quadrature at 20000 intervals, not exact),
  MS07's 4.070871 (same), and every `fuse` value that inherits one of those.
- **Blocked cells not included**: truncated cones (`pcone r1 r2 h`) have no constructor in this kernel
  (PORT MAP #5) — the OCCT reserves ZF6/ZM5/ZP7 stay blocked; half-spaces likewise (substituted with
  10-unit boxes throughout, which is exact for these operand sizes but not identical to OCCT's cells).
