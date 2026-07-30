# Commercial-kernel doctrine → this pipeline (the bridge document)

2026-07-25. Written for session A's architecture decision gate (queue step 4/5) and for
the user directly. Sources: the public record on Parasolid/ACIS/CGM (Jackson's tolerant
modelling paper, the Inside ACIS books, Braid's thesis, Requicha/Voelcker regularized
booleans, Hoffmann's robustness chapters, ESOLID/Keyser exact curved booleans, kernel
interface docs and patents) + direct reading of OCCT V8 source at
`/home/petras/code/code_cpp/OCCT` (paths below verified on disk). NO proprietary source
was available or used — Parasolid/ACIS/CGM/Rhino boolean internals are closed; anything
claiming to quote them verbatim is fabrication. What follows is doctrine every serious
kernel converges on, stated as laws with the *why*, and mapped to the 108-naked residue.

The one-sentence version: **your residue classes are not tolerance bugs; they are the
signature of a stitch-based architecture, and every commercial kernel eliminated them
by making watertightness a topological invariant instead of a numerical achievement.**

---

## Law 1 — IMPRINT, DON'T STITCH (kills ~2/3 of the residue by construction)

Every industrial kernel (Parasolid imprinting, ACIS cellular topology, CGM watertight
operators, OCCT pave blocks) builds the boolean like this:

1. Compute the section as ONE global graph of intersection edges (pave blocks), owned
   by neither operand.
2. IMPRINT that graph into BOTH operands' topologies: each intersection edge becomes a
   single shared edge ENTITY, referenced by up to 4 faces (2 of A, 2 of B). Faces are
   split against edges that already exist in the data structure.
3. Selection (cut/common/fuse) then only PICKS faces. Closure is guaranteed because
   both sides of every seam reference the same edge object — there is nothing to weld.

A stitch-based pipeline (split each operand's faces independently, then weld the two
boundaries numerically) can be driven arbitrarily far down — that is what the 196→108
campaign has done — but it has a structural floor: any per-face decision (drop a sliver,
snap a trim, cull a micro-seg) taken on ONE side creates a naked edge that must then be
re-closed by tolerance. The census taxonomy says it directly:
- SEGLOST / one-sided whole-seg drops (largest class, M1's target) = a segment kept in
  one face's UV arrangement and lost in its mate's. Impossible under imprinting: there
  is one segment entity; "lost on one side" is not a representable state.
- Divergent duplicates (both copies valence-2, no scaffold dangles) = TWO numerical
  copies of one section curve. Impossible: one pave block, one edge entity.
- Junction gaps (0.038–0.466 ≫ tol3) at valence-1 pairs = chains built per-face meeting
  at a junction that was never a shared vertex entity. Under imprinting, junction paves
  are global vertices first; chains terminate AT them by construction.

M1 (SEG-ADOPT: force-inject the one-sidedly-kept seg into the losing arrangement) is
the patch form of Law 1. The architecture form is: the UV arrangements of the two mated
faces are DERIVED VIEWS of one shared pave-block chain, never independent computations.
BOP2's "construction identity + pool referencing" was already moving there — Law 1 is
that idea taken to its logical end.

OCCT reading list for the mechanism (local paths, verified):
- `src/ModelingAlgorithms/TKBO/BOPDS/` — `BOPDS_DS` (the shared data structure:
  paves, pave blocks, common blocks, face info), `BOPDS_PaveBlock`, `BOPDS_CommonBlock`
  (ONE object representing coincident edge pieces of BOTH operands — Law 3's carrier).
- `src/ModelingAlgorithms/TKBO/BOPAlgo/` — `BOPAlgo_PaveFiller_*.cxx` (the intersection
  passes, split by interference type: 1=VV, 2=VE, 3=EE, 4=VF, 5=EF, 6=FF, 7=post,
  8=common-block glue), `BOPAlgo_Builder_*` (imprint + assembly),
  `BOPAlgo_BOP.cxx` (BuildBOP selection op-table).
- `src/ModelingAlgorithms/TKBO/BOPTools/` — `BOPTools_AlgoTools*.cxx` (classification,
  MakeSplitEdge, orientation tools).

## Law 2 — TOLERANCE IS PER-ENTITY, MONOTONE, AND HIERARCHICAL

Parasolid's tolerant modelling (Jackson, SMA '95 — the public description of what
became tolerant vertices/edges) and ACIS's tedge/tvertex are the same doctrine:
- Every vertex/edge carries its OWN tolerance; tol(vertex) ≥ tol(edge) ≥ tol(face)
  wherever they are incident (the containment invariant).
- Tolerances GROW at merge decisions: every merge/snap/weld that succeeds at distance d
  bumps the entity tolerance to ≥ d. The entity then *remembers* the fuzz it absorbed;
  later decisions test against per-entity sums (tolA + tolB + fuzz), never a global
  epsilon.
- A boolean's output tolerances are part of the result, not a shame to hide: a vertex
  that closed a 0.05 gap IS a tolerance-0.05 vertex. Downstream code respects it.

CORRECTION (from `audit_occt_tolerance-model.md`, verified against the real source —
this supersedes the stronger claim originally written here): **"tolerance is globally
monotone non-decreasing" is FALSE as an implementation invariant.** OCCT contains five
hard-set, shrink-capable tolerance writers, and same-parameter **overwrites** tolE
rather than raising it. Growth-only is the *design intent* at merge sites, not a global
guarantee, and porting it as an unconditional invariant will diverge from OCCT. Port
per-site: grow where merges absorb fuzz, and reproduce the explicit set-points where
OCCT sets rather than raises. Related audited corrections: the E/F criterion is
`1.5*tolE + tolF` for splines (not `tolE + tolF`), self-interference fuzzy is **added**
not replaced, and "geometry is never moved" has an FF seam-shift exception.

This is M4 (TOL-GROWTH) — and it is infrastructure for M1, because force-injected segs
land at mate-pair distances that per-entity tolerances absorb and record, while a global
tol3 either swallows real geometry or rejects the weld. OCCT's version:
`BOPAlgo_PaveFiller` fuzzy value; `BRepLib::SameParameter` (in
`src/ModelingAlgorithms/TKTopAlgo/BRepLib/`) — read the audit file before porting it.

## Law 3 — COINCIDENCE IS A FIRST-CLASS OUTCOME, NOT A NUMERICAL ACCIDENT

Roughly half of a commercial boolean's code handles the degenerate lattice: same-domain
faces, tangent contacts, edge-on-face, vertex-on-edge. The kernel enumerates ALL
interference types in a fixed order (OCCT: VV VE EE VF EF FF — see the PaveFiller pass
files) and every "ON" verdict becomes a typed object (common block, same-domain pair),
never a lucky near-zero. The op-table then has explicit rows for ON-same / ON-opposite
per operation (CUT reverses tool faces). This is session B's whole mandate (P3) — it is
not an optimization, it is the missing half of the taxonomy. z15/x20 grazing configs sit
exactly on this boundary: a kernel without a coincidence subsystem experiences tangency
as an unstable numerical knife-edge (which is also why the OCCT oracle itself is
self-inconsistent on z15 — even industrial kernels wobble here; ground truth must come
from partition-identity invariants, as session C is establishing).

## Law 4 — FIND ALL SECTION BRANCHES BEFORE REFINING ANY (discovery ≠ refinement)

Industrial SSI is two-phase: (a) DISCOVERY guarantees every branch/loop of the
intersection has at least one start point — polyhedral seeding (OCCT `IntPolyh` at
`src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/` — exactly what SESSION_SEED2 is),
characteristic points (extrema, tangency points, boundary crossings), analytic special
cases (`IntPatch_ImpImpIntersection` for quadric pairs; `IntAna`); (b) REFINEMENT
(marching) only traces and polishes what discovery found — `IntWalk_PWalking` at
`src/ModelingAlgorithms/TKGeomAlgo/IntWalk/`, whose `PutToBoundary` (minimization at
domain boundaries, queue item #3) exists because marching UP TO a boundary always
stalls short by a step. Marching must never be responsible for discovery: a missed
0.128 mini-branch (x20) is a discovery failure, unfixable by better marching. Grazing
arcs (z37) are tangential zones where walking is unstable BY DESIGN — industrial
kernels detect the tangential condition and switch representation (tangent-zone
handling, perturbation, or analytic treatment), they do not push the marcher harder.

## Law 5 — CLASSIFY ONCE, PROPAGATE TOPOLOGICALLY

No industrial kernel classifies every face independently against the other body (N
independent chances to be wrong at grazing angles). They classify a FEW faces robustly,
then PROPAGATE the verdict across the imprinted topology: crossing a section edge flips
in/out; crossing an ordinary edge preserves it. With Law 1 in place, propagation is
free and classification errors can no longer create local leaks — a wrong seed shows up
as a globally wrong (volume-sanity-catchable) selection instead of one naked face.
OCCT: `BOPTools_AlgoTools::IsInternalFace` + BuildBOP's state maps. The radial
classifier already matches the oracle 0/42 — keep it as the SEED, stop using it per-face.

## Law 6 — VALIDITY IS A PIPELINE INVARIANT, NOT A FINAL CHECK

Every-edge-has-exactly-2-face-uses-with-opposite-orientation is checked (cheaply) after
EVERY stage; the first stage where it breaks is the defect site. The census discipline
already does this by hand — make it a structural assert with SESSION gate. Commercial
kernels additionally run Euler-operator bookkeeping so illegal topology is unmakeable.

## Law 7 — NEVER HEAL THE RESULT

Sewing/healing is for imported geometry, never for your own boolean output. If output
needs sewing, the pipeline upstream is wrong (locked method already says this — it is
also Parasolid/ACIS doctrine; their booleans have no "fix the result" stage).

---

## What Rhino does (public knowledge) — the anti-pattern

openNURBS ships no booleans. Rhino's implementation is the classic stitch design:
SSI → split faces at document tolerance → join. Its failure modes are famous and
PERMANENT (tangent cylinders, coplanar caps, "naked edges after boolean" as a standing
support topic) because they are the structural floor of stitching, not bugs. Rhino 8
rewrote chunks of SSI and the failures moved, they did not vanish. Matching Rhino is
aiming BELOW where this campaign already is on its solved cells; the target is the
Parasolid class, and the delta is exactly Laws 1/2/3/5.

## The decision gate for session A

The M-ladder (M3→M2→M4→M1) is the patch-form of the laws and is still the right
risk-ordered way to drain 108 → small. But set the gate now: **if after M1 lands the
frontier is not 0 (or z37/z15-class grazing cells keep oscillating), stop patching and
execute the architecture form of Law 1**: one shared pave-block graph, face
arrangements as derived views, junction paves as global vertices, common blocks for
coincidence (Law 3, arriving from session B), classification by propagation (Law 5).
That is a rewrite of the splitter's data flow, not of the geometry code — SSI,
corrector, classifier, STEP I/O all survive. BOP2 construction identity was the first
step of exactly this; the user has explicitly authorized going the rest of the way
("whatever it takes, ground-up if needed").

## Honesty appendix

- Parasolid/ACIS/CGM/Rhino sources are closed. This document is doctrine reconstructed
  from the public record + OCCT's implementation of the same doctrine, which IS fully
  readable at `/home/petras/code/code_cpp/OCCT`. Where a specific mechanism matters
  (pave blocks, common blocks, PutToBoundary, IntPolyh seeding, SameParameter), read
  the OCCT file — it is the only industrial-grade boolean whose source you can consult,
  and its architecture is the same family as Parasolid's.
- The kb/ already contains deep dives (research_parasolid-architecture.md,
  research_acis-cgm-architecture.md, research_tolerance-doctrine-theory.md,
  occt_pavefiller-core.md, occt_ssi-walking.md …). This document is the bridge: laws →
  your residue classes → the decision gate. It intentionally repeats nothing at spec
  level.
