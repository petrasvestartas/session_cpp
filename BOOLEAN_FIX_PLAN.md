# Boolean Pipeline — Remaining-Work Plan
Working from BOOLEAN_PIPELINE_REPORT.md. Final validated tree = fixes A–E + scaffold
bisection (base exact, matrix 45/45, edge 54/54, minitests 757/757, z30x20 solid on all 3
ops). One open bug (grazing rotated configs → open shells) with three distinct causes,
one fundamental limit, three architectural investments.

Discipline for EVERY fix: gate behind `SESSION_*`, build a private exe, test on chairs
configs, then re-gate base+matrix+edge before considering default-on. Never ship a change
that regresses a watertight cell.

---

## PHASE 1 — close the known configurations (scoped, implementable now)

### Fix 2 — SYMMETRY GATE  [lowest risk, do FIRST]
Cause: a section segment imprinted on only ONE operand (z15: seg on B not A → an A face
never splits → one oversized fragment takes a single verdict, whole region lost).
Plan: at combine in `BRep::boolean`, the two per-side whole-segment `seg_id` sets are in
`span_to_res` (A) and `b_span_of` (B). For every `seg_id` present on exactly one side,
either force the cut onto the other operand (re-split that face with the missing segment)
or drop it from the side that has it. Simplest safe v1: DETECT + report + drop-from-one-side
for section edges that have no cross-operand partner AND both endpoints on scaffold vertices.
Gate `SESSION_NO_SYMGATE`. Validate: z15 face count vs OCCT, base unchanged.

### Fix 3 — RIM-LOCAL RE-CLASSIFICATION  [safe, cheap]
Cause: residual sampling error at the rim (z90: A f2, A f40 mis-dropped — the last ~2 of
the 9% sampling errors, sitting on the rim).
Plan: after the shell is assembled, if naked edges remain, take the fragments adjacent to
each naked rim and re-classify them with an EXACT point query (contains_point at the true
sample, K=1 exact rather than sampled majority) — the rim localizes exactly where sampling
failed. Only flips a fragment if the exact query disagrees with high confidence. Gate
`SESSION_NO_RIMFIX`. Validate: z90 naked 8→?, base+matrix+edge unchanged.

### Fix 1 — VALENCE-1 BRIDGE / local SSI re-march  [THE real work, highest risk]
Cause: SSI marcher terminates mid-face → dangling chain end (y30, x20, z45 dominant).
Plan: turn `[SCAF-VAL]` audit into a REPAIR in build_section_scaffold:
  1. after building segments, compute vertex valence; collect valence-1 vertices.
  2. pair each with its nearest valence-1 partner within a max-gap (~0.5).
  3. bridge: seed a local SSI march between the two endpoints on the shared (surfA,surfB)
     pair (correct7 each intermediate sample); if the two ends are on DIFFERENT pairs it is
     a triple point — solve the 3-surface intersection (refine_triple_point exists).
  4. VERIFY every bridge sample lies on both surfaces within conv_tol; append as a new
     SectionSegment welding to the two existing vertices (valence → 2).
  5. if the network STILL dangles after bridging, REFUSE the scaffold for that surface pair
     and fall back to the legacy imprint path — never ship a half-network.
Gate `SESSION_NO_BRIDGE`. Adversarial code review BEFORE build. Validate every config +
full re-gate. This is SSI-completeness work.

---

## PHASE 2 — the fundamental limit (different contract, not an exact fix)

### Fix 4 — FUZZY FALLBACK for grazing (z15, x20)
These are ambiguous at sub-tolerance grazing; OCCT itself fails them. No exact method
closes them. Plan: when the parity classifier reports a CONTRADICTORY component (the trigger
already exists), or naked edges survive all repairs, re-run the operation with an inflated
fuzzy tolerance that snaps near-coincident geometry together, and return the fuzzed solid
behind an explicit `fuzzy_value` option (approximate BY DESIGN, never silent). Gate
`SESSION_FUZZY`.

---

## PHASE 3 — architectural investments (prevent recurrence; larger, later)

### Fix 5 — PostTreatFF nested self-fuse
Re-fuse the section soup over just the new section vertices/edges so the network is closed
BY CONSTRUCTION → makes Fix 1 a construction guarantee instead of a repair.

### Fix 6 — per-entity tolerances
Monotone vertex⊇edge⊇face tolerances, growth reopens earlier negative interference
verdicts, post-pass reduction. Removes the fixed-threshold brittleness at grazing.

### Fix 7 — shared topology store (OCCT BOPDS analog)
Section edges/paves/vertices as the SAME objects both operands reference. The real cure for
the whole dangling-network / staggered-copy family; FIX A fakes it post-hoc. Weeks;
incremental (make the store authoritative for section edges first, then face splitting,
then classification).

---

## STATUS
- [x] Fix 1 — JUNCTION WELD (done, validated, gated SESSION_BRIDGE). The dangling ends turned
      out to be CROSS-surface-pair junction undershoots (the marcher stops short of a face-border
      junction from two sides, leaving two vertices ~0.1-0.2 apart that should be one). Repair =
      weld the pair to one vertex at their common junction (midpoint, verified to project onto
      every involved surface) and reproject each segment's endpoint onto its own surfaces →
      valence 2 → arrangement nodes instead of prunes. RESULT: x20 naked 9→6, z45 32→31, base
      EXACT, adversarial-reviewed, no-op where no dangles (y30/z90). The SSI-march bridge and the
      3-surface-refine T were both tried and reverted (march: dangles are cross-pair not same-pair;
      refine: over-rejects without closing anything more).
- [~] Fix 2 symmetry gate (z15) — DIAGNOSED, DEFERRED (grazing-limited, high-risk fix).
      Root cause (agent-traced): seg 37 collapses to a ZERO-SPAN slit (ta==tb=0.51057) on A's
      surface-18 arrangement because B pokes through A almost exactly along A's internal 18/7 seam,
      making the kept strip near-zero-width (4.4e-5 sliver). cycle_to_segments' SEGFALL path
      (nurbssurface_trimmed.cpp ~1287-1316) replaces it with a src=-1 straight chord → loses the
      seg-id → invisible to the combine alias merge → seg 37 naked on A (B-only). The 17 naked =
      9 B-side (segs 32-39,41 rim) + 8 A-side (surface-18 neighbours). This is ANOTHER GRAZING
      pathology (z15 is one of the 2 OCCT-unsound configs). The agent's fix (a: post-combine
      imprint seg 37's chain onto A face 18 + re-split + re-classify, brep.cpp ~7929-7943) is
      strictly additive but COMPLEX and touches face splitting post-classification; fix (b: preserve
      seg-id through SEGFALL) touches the shared arrangement with base-chairs exposure. Neither is
      the clean provable-no-op kind. DECISION: document precisely, do NOT ship a risky fix for one
      OCCT-unsound grazing config; the real path is Fix 4-v2 (early fuzzy) or Fix 7 (shared store).
- [x] Fix 3b EXACT POINT CLASSIFIER (z90 winding) — DONE, validated. NOT a fundamental limit.
      Fast probe (SESSION_Z90_PROBE, no boolean) PROVED the winding on B.mesh() is wrong because the
      rotated freeform mesh SELF-OVERLAPS: f2 (0.8 OUTSIDE B) gives omega 4.33pi (says inside), f40
      (inside) gives 1.32pi (says outside) -- a closed mesh gives EXACTLY 0 or 4pi. ray-parity and
      oriented-winding fail identically (mesh GEOMETRY is bad, not orientation). BRep::contains_point_exact
      = closest point on the TRIMMED boundary + outward-normal sign, MESH-FREE, gets both right.
      Wired into classify in_other/in_own for freeform (primitives keep inside_prim -> matrix/edge
      untouched). Gate SESSION_NO_EXACT_PIP. RESULT: base chairs cut EXACT (35/46.8114), z90 A-side
      classification 91%->96% (audit vs OCCT point oracle). z90 residual now = 2 fragments (f19/f22)
      that are a near-TANGENT region (A grazes B's boundary within 0.08) flagged ON -- a grazing/ON
      interaction, NOT a winding error. z90 8 naked -> still 8 (the winding fix changed verdicts
      40->43 faces but the grazing rim persists). The exact classifier is a genuine safe improvement;
      z90's residual is now the same grazing class as x20/z15.
- [~] Fix 3 island/hole repair — IMPLEMENTED (provably-safe no-op) but does NOT close z90.
      Idea: a fragment whose keep-state disagrees with EVERY non-section 2-trim neighbour is an
      isolated island/hole → flip. Validated no-op: base EXACT, edge 54/54, matrix 0 non-R fails.
      BUT z90 unchanged (8 naked). DEBUG ([CLS-ISLAND-DBG]) showed why: z90's wrongly-kept face is
      bounded almost ENTIRELY by SECTION edges (nb_tot=1, its one non-section neighbour AGREES).
      Keep-state legitimately flips across section edges, so they MUST be excluded → the isolation
      is section-edge-mediated and invisible to this test. z90's f2/f40 are a genuine WINDING
      classification error (the 90-deg rotation self-intersects the freeform mesh, inverting the
      winding over a pocket; all K samples unanimously wrong; the isolation only appears in the
      RESULT after the section edges fail to mate). A correct z90 fix needs a BETTER POINT
      CLASSIFIER (ray-cast against the real trimmed BRep faces, not the tessellated mesh) — a
      shared-path change with regression risk — or the architectural store. The island repair is
      KEPT (gated SESSION_NO_ISLAND default-on) because it is harmless and protects genuine
      non-section islands on other inputs; it just isn't z90's fix.
- [~] Fix 4 fuzzy fallback — v1 DONE (partial), gated SESSION_FUZZY=<mult>. A post-sew pass that,
      when naked edges remain, re-runs co_refine+sew at tol=diag*5e-3*mult to snap near-coincident
      staggered boundaries. MEASURED on x20 (after the weld's 9->6): mult 2 (ftol .14) 6->6, mult 4
      (.28) 6->6, mult 8 (.57) **6->3**. So it closes the small-gap slivers but needs a very
      aggressive tolerance and still leaves 3 (the wider ~0.7 missing-region slivers). Honest
      verdict: inflated-sew fuzzy is a real but PARTIAL, approximate fallback. The COMPLETE grazing
      fix needs OCCT-style EARLY fuzzy -- inflate tolerances at the SSI/same-domain stage so the
      grazing region is handled as ONE shared contact patch instead of two diverging trims (v2,
      deeper: hook the scale-invariant ON test at brep.cpp ~7280 to accept proximity within a
      fuzzy_value as coincidence). v1 kept as a documented gated tool; v2 is the real feature.
- [ ] Fix 5 PostTreatFF self-fuse
- [ ] Fix 6 per-entity tolerances
- [ ] Fix 7 shared topology store

## HONEST DECOMPOSITION of the remaining open bug (evidence-backed)
Each open config's naked edges resolve to one of four classes:
1. JUNCTION DANGLES → Fix 1 weld (DONE). x20 -3, z45 -1.
2. GRAZING SLIVERS → Fix 4 fuzzy (fundamental; OCCT fails too). The MAJORITY of the residual
   (x20's remaining 6, most of y30/z45/z15). No exact fix exists; needs the fuzzy contract.
3. CLASSIFICATION → Fix 3 (z90: 8 naked, no dangles).
4. UNSPLIT FACE → Fix 2 (z15 one-sided imprint).
Conclusion: the scaffold-level fixes (weld, bisection, pave-blocks) are at their useful limit.
Closing the grazing-dominated configs requires Fix 4 (fuzzy) — a different, approximate contract —
or the architectural Fix 7 (shared topology store). Both are large, standalone efforts.
