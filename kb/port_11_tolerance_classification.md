# port_11 — THE TOLERANCE MODEL AND POINT CLASSIFICATION

**Port specification. Implementable without further reference to OCCT.**

Subsystem: the numerical foundation — what a tolerance *means* per entity, who writes it,
how a 3D tolerance becomes a UV tolerance, how a point is classified against a curved
trimmed face and against a solid, and what a boolean result must satisfy to be accepted.

Everything below is grounded in real OCCT V8 source at
`/home/petras/code/code_cpp/OCCT`. All `file:line` citations are **relative to that root**
unless the path starts with `/home/petras/code/code_rust` (our code). Every line was read
in this session unless explicitly marked `[audit]`, in which case it is taken from
`kb/audit_occt_tolerance-model.md`, whose citations were themselves verified against the
same tree (I re-verified the load-bearing ones — `BOPAlgo_PaveFiller_6.cxx:699 / 761 /
2974 / 3029-3060 / 1047-1048 / 1073-1095 / 629-645` — and they are correct).

Trust order, per the mission brief: this document > `kb/audit_occt_*.md` > older
`kb/occt_*.md` specs (several of which are wrong).

---

## 1. WHAT THIS SUBSYSTEM MUST GUARANTEE

Each guarantee is written so it can be turned into an assertion. `tol(X)` is the stored
model-space tolerance of entity `X`; `Tol(X)` is the *effective* tolerance = `max(tol(X),
CONFUSION)`.

**G1 — Tolerance is a 3D distance, always.**
No tolerance, threshold, epsilon or band anywhere in the kernel is expressed in parameter
space, in units of a UV domain extent, or as a fraction of a knot range. Parameter-space
quantities exist only as *derived*, per-evaluation-point conversions of a 3D tolerance
(G5). Testable: grep the splitter for any literal multiplied by `(u1-u0)`, `(v1-v0)`,
`range_u`, `range_v`, `max(range)`, `min(range)`, or a knot span — the count must be zero
outside the metric-conversion function itself.

**G2 — Domain-padding invariance.**
Let `F` be a trimmed face and `F'` the same face whose surface has been re-parameterised
by an affine map of the UV domain (the STEP round-trip case: `u∈[0,1]` becomes
`u∈[-0.04, 4.04]`, all pcurves mapped accordingly). Then every predicate in this
subsystem returns bit-identical answers on `F` and `F'`, and the split of `F` by any
cutter is combinatorially identical (same face count, same edge count, same naked count).
This is the single most important guarantee: it is the one our kernel violates today with
no boolean involved.

**G3 — The containment invariant.**
For every incidence, `Tol(vertex) >= Tol(edge) >= Tol(face)`. The invariant is enforced by
a *harmonisation pass* (§2.1.6) that runs at defined points, not by every writer. It is a
post-condition of the pipeline, not a per-write precondition — because OCCT has
shrink-capable writers and so must we (G4).

**G4 — Every tolerance write is one of three declared kinds, and the kind is part of the
call.**
`RAISE(x, t)` = `tol(x) = max(tol(x), t)`; `SET(x, t)` = `tol(x) = t` (may shrink);
`PROVISIONAL(x, t)` = raise, recorded in an undo log, committed or rolled back at a
declared scope boundary. There is no fourth kind and no untyped write. Testable: the
tolerance field is `private` with no non-const accessor; the only mutators are these
three.

**G5 — Metric conversion is per-point and per-direction.**
`uv_tol_u(P) = tol3d / |∂S/∂u|(P)`, `uv_tol_v(P) = tol3d / |∂S/∂v|(P)`, each clamped by the
surface-type-exact closed forms of §2.3. Never one scalar for both directions; never
evaluated once at the domain centre and reused across the face. Testable: on a
cylinder of radius `R = 10`, `uv_tol_u(tol=1e-6) ≈ 1e-7` while `uv_tol_v(tol=1e-6) = 1e-6`
— a factor of `R` apart; a scalar implementation cannot produce both.

**G6 — `IN` and `ON` are distinct outcomes and both are first-class.**
`classify_point_face` returns `{IN, ON, OUT}`; `classify_point_solid` returns
`{IN, ON, OUT}`. No caller collapses `ON` into `IN` or `OUT` implicitly; each caller
declares which of the two "ON-accepting" or "ON-rejecting" predicates it wants
(§2.4.4). Testable: the enum has three values and the two convenience wrappers are
named `is_in_face` (ON→false) and `is_in_or_on_face` (ON→true), matching OCCT
`IntTools_Context::IsPointInFace` / `IsPointInOnFace`
(`src/ModelingAlgorithms/TKBO/IntTools/IntTools_Context.cxx:604-608`, `:639-643`).

**G7 — No probe is placed at a symmetric parameter.**
Every interior-point probe, every intermediate parameter, every ray origin uses an
asymmetric constant from the declared table (§2.6). Midpoints, domain centres and
equal-weight centroids are forbidden as probe locations. Testable: a probe-location audit
function asserts `|t - 0.5| > 0.05` for every 1-parameter probe and asserts barycentric
weights are pairwise distinct.

**G8 — Classification is equivariant under rigid motion.**
For any rigid motion `M`, `classify_point_solid(M·p, M·S) == classify_point_solid(p, S)`
for all `p` at distance `> 10·Tol` from `∂S`. Testable directly, no oracle.

**G9 — Classification is deterministic and order-independent.**
The same query twice returns the same answer; the answer does not depend on face
iteration order, on which operand is `A`, or on thread scheduling. Testable: shuffle the
face array, re-query.

**G10 — The fuzzy value is ADDED, never substituted.**
User fuzzy `f` enters every pair predicate as an *additional* term on top of the
entities' own tolerances, in the exact weighting OCCT uses (half per operand for
narrow-phase box/pair tests, whole once for point tests). `f` is clamped from below at
`CONFUSION`. Setting `f = CONFUSION` (the default) must reproduce the no-fuzzy behaviour
exactly. Testable: `result(f=CONFUSION) == result(no fuzzy)` bit-for-bit.

**G11 — Acceptance of a boolean result is structural first, numeric second.**
A result is accepted iff: naked edge count == 0, non-manifold edge count == 0, every
shell is closed and correctly oriented, every face passes per-face validity, and the four
oracle-free invariants of §5.6 hold. No oracle, no reference file, no "close enough on our
corpus".

**G12 — Failure is typed, never silent.**
Every predicate that can fail returns a status, not a bare `bool` that defaults to a
guess. `IntTools_Context::ComputePE` returns `0 / -2 / -3 / -4`
(`IntTools_Context.cxx:437-495`); `BOPTools_AlgoTools3D::PointInFace` returns `0..5`
(`BOPTools_AlgoTools3D.cxx:1010-1065`). Our port keeps the same discipline: an
un-classifiable point is `UNKNOWN`, and callers must handle it.

---

## 2. OCCT'S ALGORITHM, IN FULL

### 2.0 The universal constants

`src/FoundationClasses/TKernel/Precision/Precision.hxx`:

| name | value | line | meaning |
|---|---|---|---|
| `Precision::Confusion()` | `1.e-7` | :165 | the floor for every 3D geometric tolerance |
| `Precision::SquareConfusion()` | `1e-14` | :169 | |
| `Precision::Intersection()` | `Confusion*0.01 = 1.e-9` | :220 | "two points are the *same* intersection point" |
| `Precision::Approximation()` | `Confusion*10 = 1.e-6` | :235 | approximation targets |
| `Precision::Angular()` | `1.e-12` | :123 | parallelism / orthogonality of unit vectors |
| `Precision::PConfusion()` | `Confusion*0.01 = 1.e-9` | :334 | **default parametric confusion, unit-free** |
| `Precision::PIntersection()` | `1.e-11` | :340 | |
| `Precision::PApproximation()` | `1.e-8` | :346 | |
| `Precision::Parametric(P,T)` | `P/T` | :242 | **the metric conversion, spelled out** |
| `Precision::Parametric(P)` | `P*0.01` | :328 | the `T=100` default "typical derivative magnitude" |
| `Precision::Infinite()` | `2.e+100` | :371 | |
| `Precision::IsInfinite(R)` | `|R| >= 1.e+100` | :350-353 | |

Read `Precision::Parametric(P, T) = P / T` as: *T is the magnitude of the derivative of
the geometry with respect to its parameter.* The scalar `PConfusion()` is nothing but
`Parametric(Confusion(), 100)` — an admission that a bare parametric constant is only
valid when the derivative is ~100. **This is the whole of the model-space-vs-parameter-space
doctrine in one function.** Everywhere OCCT can compute the real derivative it does
(§2.3); `PConfusion()` is the fallback when it cannot.

### 2.1 The per-entity tolerance model

#### 2.1.1 Geometric meaning

- **Vertex**: `tol(V)` is the radius of a sphere centred at `BRep_Tool::Pnt(V)`. The true
  point of the vertex is *somewhere* in that ball. Every curve and surface incident on
  `V` must pass through the ball.
- **Edge**: `tol(E)` is the radius of a pipe around the 3D curve `C3D(t)`. Every pcurve
  representation `S_i(PC_i(t))` of the edge must lie inside the pipe, and the pipe must
  contain the true edge. This is what `BRepLib_ValidateEdge` measures
  (`src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_ValidateEdge.cxx:103-230`) and what
  `BRepLib::UpdateInnerTolerances` enforces (`BRepLib.cxx:1983-2083`).
- **Face**: `tol(F)` is the offset band half-thickness around the surface within which the
  face's true geometry lies. In the boolean pipeline `tol(F)` is **an input-side
  constant** — `BOPTools_AlgoTools_1.cxx:1066-1089` (`UpdateShape`) handles only `EDGE`
  and `VERTEX`; no boolean stage ever writes a face tolerance.

#### 2.1.2 Storage, and the *effective* tolerance floor

The accessor clamps. All three of

- `BRep_Tool::Tolerance(const TopoDS_Face&)` — `src/ModelingData/TKBRep/BRep/BRep_Tool.cxx:137-150`
- `BRep_Tool::Tolerance(const TopoDS_Edge&)` — `BRep_Tool.cxx:881-894`
- `BRep_Tool::Tolerance(const TopoDS_Vertex&)` — `BRep_Tool.cxx:1314-1330`

do exactly `p = T->Tolerance(); return p > Precision::Confusion() ? p : Precision::Confusion();`

**Port consequence**: a stored tolerance below `1e-7` is unobservable. Any "shrink" below
`CONFUSION` is a no-op at read time. Implement the clamp in the accessor, not in the
writers — otherwise the shrink-capable writers (below) become lossy.

Raw storage mutators:

- `BRep_TVertex::UpdateTolerance(T)` — `src/ModelingData/TKBRep/BRep/BRep_TVertex.lxx:33-37`:
  `if (T > myTolerance) myTolerance = T;` — **max-monotone**.
- `BRep_TEdge::UpdateTolerance(T)` — `src/ModelingData/TKBRep/BRep/BRep_TEdge.lxx:33-37`:
  identical — **max-monotone**.
- `BRep_TFace` has **no** `UpdateTolerance` — only the raw setter `Tolerance(T)`.
  Confirmed: `grep UpdateTolerance BRep_TFace.hxx` is empty.

Builder-level:

- `BRep_Builder::UpdateVertex(V, Tol)` — `BRep_Builder.cxx:1442-1454`: `TV->UpdateTolerance(Tol)`
  (**RAISE**), throws `TopoDS_LockedShape` if `TV->Locked()`.
- `BRep_Builder::UpdateEdge(E, Tol)` — `BRep_Builder.cxx:999-1009`: `TE->UpdateTolerance(Tol)`
  (**RAISE**), same lock throw.
- `BRep_Builder::UpdateFace(F, Tol)` — `BRep_Builder.cxx:597-607`: `TF->Tolerance(Tol)`
  (**SET** — can shrink), same lock throw.

#### 2.1.3 Complete writer census

Every site in OCCT that writes a tolerance, with its kind. This is the table your port
must reproduce site-for-site; a single global `RAISE` policy reproduces none of OCCT's
acceptance boundaries.

| # | site | file:line | kind | what it writes |
|---|---|---|---|---|
| W1 | `BRep_Builder::UpdateVertex(V,tol)` | `BRep_Builder.cxx:1442-1454` | RAISE | caller's value |
| W2 | `BRep_Builder::UpdateEdge(E,tol)` | `BRep_Builder.cxx:999-1009` | RAISE | caller's value |
| W3 | `BRep_Builder::UpdateFace(F,tol)` | `BRep_Builder.cxx:597-607` | **SET** | caller's value |
| W4 | `BRepLib::SameParameter(edge)` final write | `BRepLib.cxx:1721-1737` | **SET** | `max(maxdist, Confusion)`; only if the edge had ≥1 pcurve (`YaPCu`) |
| W5 | `BRepLib::UpdateEdgeTol` | `BRepLib.cxx:685` | **SET** | `max_i(1.4 · dist_i)` over representations |
| W6 | `UpdShTol(..., theVForceUpdate=true)` vertex branch | `BRepLib.cxx:887-899` | **SET** | reached from `InternalUpdateTolerances` (`:1961`) |
| W7 | `UpdShTol(..., false)` vertex branch | `BRepLib.cxx:895` | RAISE | reached from `InternalSameParameter` (`:1005`) |
| W8 | `UpdShTol` edge branch | `BRepLib.cxx:884` | RAISE | |
| W9 | `UpdShTol` face branch | `BRepLib.cxx:880` | **SET** | |
| W10 | FF per-pair **rollback** | `BOPAlgo_PaveFiller_6.cxx:1078-1082` | **SET** | restores the pre-growth tolerance of every uncommitted vertex |
| W11 | `FilterPavesOnCurves` reduction | `BOPAlgo_PaveFiller_6.cxx:2525-2532` `[audit]` | **SET** | `max(savedTol, sqrt(maxDistKept)+Confusion)` |
| W12 | `CorrectToleranceOfSE` edge reduction | `BOPAlgo_PaveFiller_6.cxx:4126-4131` | **SET** | `aTolC` (the curve's own tol) when `aTolC < aTolTang && aTolC < tolE` |
| W13 | `CorrectToleranceOfSE` vertex reduction | `BOPAlgo_PaveFiller_6.cxx:4268-4271` | **SET** | `aMaxTol` = max over incident pave-block ends of `dist(V, C(t)) + tolE`, only when `< tolV` |
| W14 | `PutPaveOnCurve` provisional growth | `BOPAlgo_PaveFiller_6.cxx:3052-3059` | PROVISIONAL | `UpdateVertex(aV, aDist + 1e-12)`, previous value saved into `aMVTol` |
| W15 | `BOPTools_AlgoTools::UpdateShape` | `BOPTools_AlgoTools_1.cxx:1066-1089` | RAISE | EDGE and VERTEX only; skips anything in `aMapToAvoid` |
| W16 | `BRepLib::UpdateInnerTolerances` | `BRepLib.cxx:2055,2061,2065,2073,2080` | RAISE | five separate writes |

Notes that matter for the port:

- **W4 is the trap.** A section edge built with `tolE = aTolR3D = 3e-4` can leave
  `SameParameter` with `tolE = 1e-7`, because the write is a SET floored only at
  `Confusion` (`BRepLib.cxx:1731-1734`). OCCT survives this only because `PostTreat`
  re-raises `tolE ≥ tolF` and `tolV ≥ tolE` afterwards (§2.1.6). **Port both halves or
  neither.**
- **W4 does not fire at all** if the edge already carries the `SameParameter` flag —
  `BRepLib.cxx` guards the whole routine on that flag. If your port marks every
  constructed section edge same-parameter (the natural thing for an exactly marched
  curve), the entire re-approximation and tolerance recomputation is skipped.
- **W12/W13 (`CorrectToleranceOfSE`) is a shrink pass the older specs omit entirely.**
  Purpose (comment at `BOPAlgo_PaveFiller_6.cxx:4112-4119`): when two faces meet at a
  small angle the *tangential* tolerance of the section curve is huge, and that huge value
  would otherwise be baked into the edge. The pass reduces the edge to the curve's own
  (small) tolerance and then reduces the touched vertices to the largest distance actually
  required, guarded by "don't bother unless we can shrink by ≥0.1%"
  (`if (aTolV - aMaxTol < 0.001*aTolV) continue;`, `:4225-4228`).

#### 2.1.4 Provisional growth and rollback — the layer you must not skip

OCCT treats pave-placement growth as a **bet**, unwound if the pave is not consumed.
Scope = one face-face pair.

1. `NCollection_DataMap<int,double> aMVTol` declared per FF loop
   (`BOPAlgo_PaveFiller_6.cxx:699`), `aMVTol.Clear()` at the top of each FF iteration
   (`:761`).
2. `PutPaveOnCurve` binds the vertex's **pre-growth** tolerance into `aMVTol` *before*
   `BRep_Builder().UpdateVertex(aV, aDist + aDTol)`: the growth is at `:3054`, the save at
   `:3056-3059`, gated by `if (aTolV < aDist + aDTol)` at `:3052`; the "vertex already in
   the list" branch saves at `:3034-3038`. `aDTol = BOPTools_AlgoTools::DTolerance()`
   (`:2999`), and `DTolerance()` is a literal `1.e-12`
   (`src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools.hxx:70`).
3. **The acceptance predicate reads the SAVED tolerance, not the grown one**:
   `double aTolV = (aMVTol.IsBound(nV) ? aMVTol(nV) : BRep_Tool::Tolerance(aV));`
   (`:2974`), then `IsVertexOnLine(aV, aTolV, aIC, aTolR3D + myFuzzyValue, aT)` (`:2976`).
   Growth caused by curve *k* therefore does **not** widen acceptance for curve *k+1*.
   Without this the pipeline self-amplifies.
4. **Commit**: when a section edge is actually built from the pave block, its two vertices
   are `aMVTol.UnBind(nV1); aMVTol.UnBind(nV2);` (`:1047-1048`).
5. **Rollback**: at the end of the FF pair, every vertex still in `aMVTol` is restored —
   `TV->Tolerance(aTol)` (**hard SET**, `:1081`), its DS bounding box is **rebuilt from
   scratch** (`aBoxDS = Bnd_Box(); BRepBndLib::Add(aV, aBoxDS); aBoxDS.SetGap(GetGap()+Confusion);`
   `:1085-1089` — a plain `BRepBndLib::Add` only unions and can never shrink), and its
   same-domain group is dropped (`aDMVLV.UnBind(nV1)`, `:1091-1094`).
6. `UpdateSavedTolerance` (`:629-645`): when an **existing** edge is grown instead of
   building a section edge, the saved tolerances of that edge's vertices are raised too
   (`if (pTolSaved && *pTolSaved < theTolNew) *pTolSaved = theTolNew;`), so the rollback
   cannot undo growth that a committed edge depends on.

#### 2.1.5 Non-destructive mode and copy-on-write

`BOPAlgo_PaveFiller_10.cxx` `[audit]`: in non-destructive mode, for an **old** vertex
there is **no `aTolV < aTolNew` test before the copy** — a new vertex at the same point
with `max(tolV, aTolNew)` is created unconditionally, registered as same-domain and fenced
in `myVertsToAvoidExtension` (`:130-154`), even when `aTolNew` is smaller.
`UpdateEdgeTolerance` refuses early if the edge is an original **or any of its vertices is
an original without an SD image** — the refusal is per-edge, all-or-nothing (`:69-85`).
The underlying `BRep_Builder` calls throw `TopoDS_LockedShape` on locked TShapes
(`BRep_Builder.cxx:1446-1449`, `:1002-1005`, `:600-603`), and `Locked()` is exactly what
`SetNonDestructive` detects (`BOPAlgo_PaveFiller_10.cxx:51-58`).

**Port decision**: our kernel owns its data and has no shared-TShape aliasing problem. Do
**not** port copy-on-write. Do port the *effect* it protects: an input operand's entities
must never be mutated by a boolean; the DS holds tolerance overrides in its own arrays.
This is cleaner and removes an entire class of OCCT bugs.

#### 2.1.6 Where the containment invariant is actually enforced

Three distinct passes; they are **not** interchangeable.

**(a) `BRepLib::InternalUpdateTolerances` (`BRepLib.cxx:1744-1962`)** — the reference
implementation. Never called from TKBO directly, but reachable via
`SameParameter(shape)` (`:1007`).

- Optional face minima (only when `verifyFaceTolerance`, `:1753-1823`): base by surface
  type — Plane/Cylinder/Cone `= Confusion`, Sphere/Torus `= 2·Confusion`, everything else
  `= 4·Confusion` (`:1776-1789`) — then **scaled by the face bounding-box extent**
  `tol = tol * dMax` (`:1814`) and clamped to `0.99` if `> 1.` (`:1816-1819`).
  *This is the only place in all of OCCT where a minimum tolerance is made model-scale
  relative.* The boolean pipeline never does it.
- Edges (`:1826-1856`): `tol := max over incident faces of tolF`, recorded only
  `if (tol > BRep_Tool::Tolerance(EK))` (`:1852`). Comment at `:1849-1850`: *"Update can
  only increase tolerance"*.
- Vertices (`:1858-1959`): `tol` starts at `max` of incident-edge tolerances; edges with
  `tol > BigTol = 1.e10` (`:1859`, `:1877-1880`) or `!SameRange` (`:1881-1884`) are skipped
  for the geometric part; for every curve representation of every incident edge the vertex
  point is compared against `C3D(par)` (`:1895-1908`) **and** `S(PC(par))` **and**
  `S(PC2(par))` for closed surfaces (`:1909-1937`); then `tol = max(tol, sqrt(aMaxDist))`
  and **`tol += 2.*Epsilon(tol)`** (`:1941-1942`).
- The write goes through `UpdShTol(..., theVForceUpdate = true)` (`:1961`) → vertices are
  **hard-SET** (`:889-892`), i.e. this pass **can shrink** a vertex tolerance; edges RAISE,
  faces SET.

**(b) The boolean's own `PostTreat` (`BOPAlgo_Builder.cxx:450-474`)** — this is what
actually runs on a boolean result:

```
BOPTools_AlgoTools::CorrectTolerances(myShape, aMA, 0.05, myRunParallel);   // :472
BOPTools_AlgoTools::CorrectShapeTolerances(myShape, aMA, myRunParallel);    // :474
```

`aMA` (map-to-avoid) is non-empty only in non-destructive mode (`:455-469`).

- `CorrectTolerances` (`BOPTools_AlgoTools_1.cxx:309-317`) = `CorrectPointOnCurve` +
  `CorrectCurveOnSurface`.
  - `CorrectPointOnCurve` → per edge, `CheckEdge(E, aMaxTol=0.05, aMA)`
    (`BOPTools_AlgoTools_1.cxx:430-513`). For each vertex of the edge:
    `aTol = max(tolV, tolE)`, `dd = 0.1*aTol`, compare `aPV` to the curve at the stored
    point-representation parameter and at the edge's `First()`/`Last()`; if
    `d² > aTol²` then `aNewTolerance = sqrt(d²) + dd`, and **it is written only if
    `aNewTolerance < aMaxTol`** (`:479-482`, `:505-509`). `aMaxTol = 0.05` is
    **absolute** — a 0.06 deviation leaves the shape invalid rather than tolerant.
  - `CorrectCurveOnSurface` → per face `CorrectWires` (`:665-756`) and per (face,edge)
    `CorrectEdgeTolerance` (`:761-1000`). `CorrectWires`: for every vertex of the face,
    `d2max` = max over incident edges of `|P(V) - S(PC(t_V))|²`, plus the 2D
    self-intersection distances from `IntersectCurves2d`; if `d2max > tolV²` then
    `tolV := 1.01*sqrt(d2max)` (`:749-753`).
- `CorrectShapeTolerances` (`BOPTools_AlgoTools_1.cxx:389-424`) = per edge
  `CorrectVertexTolerance` + per face `UpdateEdges`.
  - `CorrectVertexTolerance` (`:1007-1024`): `if (tolV < tolE) UpdateShape(V, tolE)` —
    **this is `tolV ≥ tolE`**.
  - `UpdateEdges` (`:1027-1062`): `if (tolE < tolF) UpdateShape(E, tolF)` — **this is
    `tolE ≥ tolF`**. Note the OCCT quirk to *not* replicate: in the `else` branch
    (`:1054-1060`) an internal vertex of the face is compared against `aTolE`, which
    carries over from the **last edge processed in the same loop** (initialised to `aTolF`
    at `:1033`) — order-dependent — and then `aTolF` is written anyway.

**(c) `BRepLib::UpdateInnerTolerances` (`BRepLib.cxx:1983-2083`)** — the per-edge audit:
`NbSamples = SameParameter(edge) ? 23 : 2` (`:2028`); when not same-parameter only the two
representation endpoints are compared (`:2037-2040`) — parameters are **not** assumed
shared; per sample `aDist += 2.*Epsilon(aDist)` (`:2044`); edge raised to `MaxDist`
(`:2065`); vertices raised twice — to the first/last sample of every representation
(`:2051-2062`) and then to `max(dist(V, C(fpar)), TolEdge)` / `max(dist(V, C(lpar)), TolEdge)`
(`:2067-2081`). **The second write is what enforces `tolV ≥ tolE` locally.**

**Port rule**: run harmonisation `(b)` at exactly one place — immediately after result
assembly, before validity checking — and never inside the interference stages. Inside the
stages, tolerances are what the stages computed; harmonising early destroys the
provisional/rollback logic of §2.1.4.

#### 2.1.7 The fuzzy value

Definition and clamping — three independent clamp sites, all `max(theFuzz, Precision::Confusion())`:

- `BOPAlgo_Options::SetFuzzyValue` — `BOPAlgo_Options.cxx:105-108`.
- `IntTools_EdgeEdge` — `IntTools_EdgeEdge.lxx:163` `[audit]`.
- `IntTools_FaceFace` — `IntTools_FaceFace.cxx:232` `[audit]`.

Defaults: `myFuzzyValue(Precision::Confusion())` at `BOPAlgo_Options.cxx:53` and `:65`;
same default in `IntTools_EdgeFace.cxx:53`, `IntTools_FaceFace.cxx:152`,
`IntTools_EdgeEdge.lxx:24,48,76` `[audit]`.

**Consumption census** — the weighting is deliberately *inconsistent* and must be copied
site-for-site. Sites I read directly are unmarked; the rest are `[audit]` (§2.9 of
`kb/audit_occt_tolerance-model.md`).

| consumer | site | form |
|---|---|---|
| DS AABB inflation | `BOPDS_DS.cxx:312` `[audit]` | `max(fuzz,Confusion)*0.5` per operand; vertex `SetGap(tol+add)` (**SET**, `:1605`), edge/face `SetGap(GetGap()+add)` (**accumulate**, `:1688,:1775`) |
| broad-phase OBB | `BOPDS_Iterator.cxx:345-346` `[audit]` | `theCtx->OBB(shape, fuzz)` — **full** fuzz, not half |
| V/V | `BOPTools_AlgoTools.cxx:1783` `[audit]` | `tolV1+tolV2+max(fuzz,Confusion)` |
| V/E | `IntTools_Context::ComputeVE` — **read**: `IntTools_Context.cxx:532` | `aTolSum = tolV + tolE + max(theFuzz, Confusion)`; **`theTol = aDist + aTolE` is written at `:534` BEFORE the accept test at `:536`** |
| V/F | `IntTools_Context::ComputeVF` — **read**: `IntTools_Context.cxx:573` | `aTolSum = tolV + tolF + max(theFuzz, Confusion)`; `theTol = aDist + aTolF` (`:574`) written before the test (`:577`) |
| P/E | `IntTools_Context::ComputePE` — **read**: `IntTools_Context.cxx:460`, `:480` | `tolP + tolE + Confusion` — **no fuzz inside**; the out-of-curve fallback tests distance to the *vertices* with `tolP + tolV + Confusion` |
| E/E | `IntTools_EdgeEdge.cxx:149-152` `[audit]` | `tol_i = curveTol_i + fuzz/2`, criterion `myTol1+myTol2` |
| E/F | `IntTools_EdgeFace.cxx:527-548` — **read** | see below |
| F/F | `IntTools_FaceFace.cxx:381-387` `[audit]` | `myTolF_i = tolF_i + fuzz/2`; `TolArc = TolTang = myTolF1+myTolF2` |
| result simplification | `BRepAlgoAPI_BuilderAlgo.cxx:185` `[audit]` | fuzzy doubles as `ShapeUpgrade_UnifySameDomain::SetLinearTolerance` |
| **dead** | `BOPAlgo_Builder.cxx:758` `[audit]` | `BuilderSolid` receives a fuzzy value and contains **no read of it** — solid assembly is fuzzy-blind |
| **not propagated** | `BOPAlgo_PaveFiller_6.cxx:1196-1197` `[audit]` | `PostTreatFF`'s nested PaveFiller never receives a fuzzy value → section-edge self-intersection always runs at `Confusion` |

**E/F is asymmetric** (`IntTools_EdgeFace.cxx:527-548`, read this session):

```
aFuzz = myFuzzyValue / 2.;
aTolF = BRep_Tool::Tolerance(myFace) + aFuzz;
aTolE = BRep_Tool::Tolerance(myEdge) + aFuzz;
if (curveType == BSpline || curveType == Bezier) {
    if (aTolE/aTolF > 100 || aTolF/aTolE > 100)  myCriteria = max(aTolE, aTolF);
    else                                          myCriteria = 1.5*aTolE + aTolF;   // :542
} else {
    myCriteria = aTolE + aTolF;                                                      // :547
}
```

A symmetric `tolE+tolF+fuzz` port both over-accepts (ratio case) and under-accepts
(spline case) relative to OCCT.

#### 2.1.8 The pad zoo — measured deviations are inflated by different factors

Do **not** unify these; each is an acceptance boundary.

| pad | site | applies to |
|---|---|---|
| `1.00001` | `BRepLib_ValidateEdge::UpdateTolerance` / `GetMaxDistance`, `BRepLib_ValidateEdge.cxx:51,59` | edge-vs-pcurve deviation |
| `1.5` | `ComputeTol` final, `BRepLib.cxx:1185` | sampled pcurve deviation in `SameParameter` |
| `1.4` | `safe_factor`, `BRepLib.cxx:503`, applied `:677` | `UpdateEdgeTol` |
| `1.05` | `GetEdgeTol`, `BRepLib.cxx:791` | planar-face edge tolerance |
| `1.01` | `CorrectWires`, `BOPTools_AlgoTools_1.cxx:751` | vertex-on-wire |
| `+0.1·tol` | `CheckEdge`, `BOPTools_AlgoTools_1.cxx:452`, used `:479`,`:504` | vertex-on-curve |
| `+1e-12` | `BOPTools_AlgoTools::DTolerance()`, `BOPTools_AlgoTools.hxx:70` | provisional pave growth (`BOPAlgo_PaveFiller_6.cxx:2999`, `:3054`); also `PutClosingPaveOnCurve` `:3573` |
| `+2·Epsilon(tol)` | `BRepLib.cxx:1942`; `:2044,2054,2060,2072,2079` | harmonisation, inner audit |
| `+Confusion` | `IntTools_Context::ComputePE`, `IntTools_Context.cxx:460`,`:480` | point-on-edge |

**`CheckTolerance` pads the threshold; `UpdateTolerance` pads the distance.** Writing
`if (dist > tol) tol = dist*1.00001;` is wrong. The real gate is
`correctTolerance(t) > dist` where
`correctTolerance(t) = t + max(BRepCheck::PrecCurve, BRepCheck::PrecSurface)`
(`BRepLib_ValidateEdge.cxx:42-45`, `:68-77`).
`BRepCheck::PrecCurve` = `RealEpsilon()` except for **ellipses**, where it is
`max_i Epsilon(x_i)` over `{|centre.x|,|centre.y|,|centre.z|, Rmaj, Rmin}`
(`src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck.cxx:70-98`);
`BRepCheck::PrecSurface` likewise except for **cones**
(`{|apex.x|,|apex.y|,|apex.z|, RefRadius}`, `BRepCheck.cxx:102-129`). ULP-scale but
**coordinate-scaled** — at 1e6 mm coordinates it is ~1e-10, not 2e-16.
`SetExitIfToleranceExceeded` (`:81-85`) makes `myCalculatedDistance` a **lower bound
only**: never feed `GetMaxDistance()` from an early-exited run into a tolerance.

### 2.2 `ComputeTol` — the sampled deviation, and the one place OCCT converts 2D→3D by hand

`BRepLib.cxx:1070-1188`. This is the function that decides whether a pcurve is acceptable,
and it contains the cleanest statement of the metric conversion in the codebase:

```
double du = 0.01*(ul-uf), dv = 0.01*(vl-vf);                                    // :1083
double DSdu = 1./surf->UResolution(1.), DSdv = 1./surf->VResolution(1.);        // :1085
...
if (!isUPeriodic) {
  if (Puv.X() < uf - du) { dapp = max(dapp, DSdu*(uf - Puv.X())); continue; }   // :1098-1102
  else if (Puv.X() > ul + du) { dapp = max(dapp, DSdu*(Puv.X() - ul)); continue; }
}
```

`1/UResolution(1.)` **is** `|∂S/∂u|` (the surface's own upper bound, §2.3). A 2D overshoot
is converted to a 3D distance by multiplying by it, and only then compared with 3D
distances. Nothing anywhere compares a 2D number with a 3D number.

Rest of `ComputeTol`, needed for a faithful port:

- `nbp` samples uniform in the 3D curve's parameter, `nbp = 22` from the caller
  (`NCONTROL = 22`, `BRepLib.cxx:1294` `[audit]`), array sized `nbp+10` (`:1077`).
- Infinite `Pcons` ⇒ `d2 = Precision::Infinite()` and immediate return (`:1123-1128`,
  `:1136-1139`).
- `dapp > d2` short-circuits the return (`:1142-1145`).
- **Outlier suppression** (`:1147-1182`): distances bucketed `<1.0` (N1) and `>=1.0` (N2);
  if `N1 > N2 && N2 != 0` then `N3 = 100*N2/(N1+N2)`; if `N3 < 10 && N3 != 0` the `>=1.0`
  samples are treated as spurious and only the `<1.0` maximum `D2` is used.
- Final: `d2 = (!ana) ? 1.5*d2 : 1.5*sqrt(D2)`, floored at `1.e-7` (`:1185-1186`).
- `EvalTol` (`:1034-1066`), used only in the C0-BSpline fallback: 5 interior samples at
  `i/6`, `i = 1..5`; succeeds if ≥3 projections converge (`return ok > 2`, `:1065`).

### 2.3 MODEL SPACE VS PARAMETER SPACE — the conversion, exactly

This is the decisive section for our kernel.

#### 2.3.1 The interface

Every OCCT surface adaptor exposes

```
double UResolution(double R3d) const;   // a Δu such that a Δu step moves ≤ R3d in 3D
double VResolution(double R3d) const;
```

`src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_Surface.cxx:1818-1896` (U) and
`:1900-1958` (V). Closed forms, **exact, not sampled**:

| surface | `UResolution(R)` | line | `VResolution(R)` | line |
|---|---|---|---|---|
| Plane | `R` | :1869-1871 | `R` | :1931-1933 |
| Cylinder radius `r` | `2·asin(R/(2r))` if `R/(2r) ≤ 1` else `2π` | :1846-1854, :1890-1895 | `R` | :1929-1933 |
| Cone | `R / max(radius at Vfirst, radius at Vlast)`; if the V-range exceeds `1e10`, falls back to `Precision::Parametric(R)` | :1855-1868 | `R` | :1930-1933 |
| Sphere radius `r` | `2·asin(R/(2r))` clamp `2π` | :1837-1845 | `2·asin(R/(2r))` clamp `2π` | :1919-1927 |
| Torus `Rmaj,Rmin` | `2·asin(R/(2(Rmaj+Rmin)))` clamp `2π` | :1828-1836 | `2·asin(R/(2·Rmin))` clamp `2π` | :1910-1918 |
| Bezier | `Geom_BezierSurface::Resolution(R, Ures, Vres)` | :1872-1876 | same | :1934-1938 |
| BSpline | `Geom_BSplineSurface::Resolution(R, Ures, Vres)` | :1877-1883 | same | :1939-1943 |
| Extrusion | basis **curve**'s `Resolution(R)` | :1824-1827 | `R` | :1928-1933 |
| Revolution | `Precision::Parametric(R)` (default arm) | :1886-1888 | basis curve's `Resolution(R)` | :1906-1909 |
| Offset | basis adaptor's `UResolution(R)` | :1880-1885 | basis adaptor's `VResolution(R)` | :1944-1947 |
| anything else | `Precision::Parametric(R) = R*0.01` | :1886-1888 | same | :1948-1950 |

The common tail (`:1890-1895`, `:1952-1957`):
```
if (Res <= 1.) return 2.*std::asin(Res);
return 2.*M_PI;
```
where `Res = R3d/(2·r)`. `2·asin(d/(2r))` is the **exact** central angle subtended by a
chord of length `d` on a circle of radius `r` — not the small-angle approximation `d/r`.
Use the exact form; on a sphere of radius 1 with `R3d = 1.9` the linearised form is 12%
wrong.

#### 2.3.2 The BSpline case — why it is *not* domain-relative

`Geom_BSplineSurface::Resolution` (`src/ModelingData/TKG3d/Geom/Geom_BSplineSurface_1.cxx:2197-2222`):

```
BSplSLib::Resolution(myPoles, Weights(), myUKnots, myVKnots, myUMults, myVMults,
                     myUDeg, myVDeg, ..., 1., myUMaxDerivInv, myVMaxDerivInv);
UTolerance = Tolerance3D * myUMaxDerivInv;
VTolerance = Tolerance3D * myVMaxDerivInv;
```

`BSplCLib::Resolution` (`src/FoundationClasses/TKMath/BSplCLib/BSplCLib.cxx:4316-4820`)
computes, over the control net,

```
max_derivative = max over spans i, over poles j in the span's support of
                 | D_i·(C_i - C_j) - D_{i-1}·(C_{i-1} - C_j) | / (t_{i+deg} - t_i)
max_derivative /= min_weights                       // rational only
max_derivative *= Degree;                                                    // :4811
UTolerance = (max_derivative > RealSmall()) ? Tolerance3D/max_derivative
                                            : Tolerance3D/RealSmall();       // :4812-4819
```

i.e. an **upper bound on `|∂S/∂u|` derived from the control polygon and the knot spans**.
It is a property of the *geometry*, not of the numeric extent of the domain. Reparameterise
`u → 4u + c`: the knot spans quadruple, `max_derivative` quarters, `UTolerance` quadruples
— exactly compensating, so **`UResolution` returns a Δu that still corresponds to the same
3D distance**. That is the whole point, and it is why a padded STEP domain costs OCCT
nothing.

**Port**: implement `NurbsSurface::u_resolution(double r3d)` / `v_resolution` with:
1. an analytic arm per recognised surface type (the table above), used whenever the
   surface carries an analytic identity (per ARCHITECTURE_v2 law 3);
2. the `BSplCLib` control-net bound for general NURBS;
3. `Precision::Parametric(r3d) = r3d*0.01` only as a documented last resort, and log when
   it is hit.

#### 2.3.3 The inverse direction, and the local metric

Where OCCT needs `|∂S/∂u|` (a 2D→3D conversion) it uses `1/UResolution(1.)`
(`BRepLib.cxx:1085`). Where it needs a genuinely *local* metric it evaluates `D1` and uses
the first fundamental form directly — e.g. `BOPTools_AlgoTools3D::GetNormalToSurface`
(`BOPTools_AlgoTools3D.cxx:406-439`) computes `aS->D1(U,V,aP,aD1U,aD1V)` and rejects when
`|D1U|² < gp::Resolution()` or `|D1V|² < gp::Resolution()` — the pole/degeneracy guard.

**Port rule (G5), stated as code**:

```
// 3D tolerance -> UV box half-extents at a specific point.
struct UVTol { double du, dv; };

UVTol uv_tol_at(const NurbsSurface& S, double u, double v, double tol3d) {
    Vector Su, Sv;  S.derivatives(u, v, Su, Sv);         // first fundamental form
    double su = Su.magnitude(), sv = Sv.magnitude();
    // Global upper bounds keep the answer sane at poles / near-degenerate rows,
    // where the local derivative vanishes and tol/|Su| explodes.
    double gu = S.u_resolution(tol3d);                    // analytic or control-net bound
    double gv = S.v_resolution(tol3d);
    double du = (su > POLE_EPS) ? std::min(tol3d/su, gu) : gu;
    double dv = (sv > POLE_EPS) ? std::min(tol3d/sv, gv) : gv;
    return {du, dv};
}
```

`POLE_EPS` is a *derivative magnitude*, not a distance: use `gp::Resolution()`-scale,
matching `BOPTools_AlgoTools3D.cxx:417,423`. Never `min(du,dv)` collapsed to one scalar
(that is what our code does today, §4.2).

#### 2.3.4 Where OCCT deliberately works in 3D to avoid the whole problem

A census, because each is a place our splitter currently works in UV:

1. **All pair tolerance sums** (`ComputeVV/VE/VF/PE`, EE/EF/FF criteria) are 3D distances
   compared to sums of 3D tolerances. `IntTools_Context.cxx:437-590`.
2. **`IsValidPointForFace`** projects the 3D point onto the surface and compares
   `LowerDistance()` — a 3D number — with `aTol` (3D), and only *then* looks at the UV
   classification. `IntTools_Context.cxx:647-673`.
3. **Shrunk range / micro-edge detection** measures **arc length** via
   `GCPnts_AbscissaPoint::Length` and compares with `Precision::Confusion()`, not a
   parameter span (`IntTools_ShrunkRange.cxx:107-190` `[audit]`; micro-edge if
   `myLength < Confusion`, splittable iff `myLength > 2·tolE + 2·Confusion`).
4. **`FindValidRange`** bisects to `anEps = max(max(Resolution(tolE)*0.1, Epsilon(maxAbsPar)),
   PConfusion)` (`BRepLib_1.cxx:201-202` `[audit]`) — `Resolution(tolE)` is again the
   metric conversion, applied to a *curve*.
5. **`ComputeToleranceOfCB`** samples 11 strictly-interior points and takes
   `max(tol_member + LowerDistance)` — 3D projections
   (`BOPAlgo_Tools.cxx:248-355` `[audit]`).
6. **`BRepLib_ValidateEdge`** compares 3D points at matched parameters, or runs a two-way
   3D `Extrema_LocateExtPC` when parameters do not match
   (`BRepLib_ValidateEdge.cxx:119-228`).
7. **`IntTools_FClass2d::Perform`** — even the *2D* classifier converts: when the polygon
   test is inconclusive it computes
   `aURes = surf->UResolution(Toluv); aVRes = surf->VResolution(Toluv);` and picks
   `aFCTol = (bUIn == bVIn) ? min(aURes,aVRes) : (!bUIn ? aURes : aVRes)`
   (`IntTools_FClass2d.cxx:729-745`), where `bUIn = (u >= Umin && u <= Umax)`,
   `bVIn` likewise. That is: *use the resolution of the direction in which the point is
   out of range.*

**Honest note on an OCCT inconsistency you should NOT copy.** `IntTools_Context::FClass2d`
constructs `IntTools_FClass2d(aFF, aTolF)` with `aTolF = BRep_Tool::Tolerance(face)`, a
**3D** value (`IntTools_Context.cxx:230-241`), and `IntTools_FClass2d::Init` then uses that
3D value directly as a **UV** floor for the polygon deflections
(`if (FlecheU < Toluv) FlecheU = Toluv;`, `IntTools_FClass2d.cxx:536-544`). Likewise
`BRepTopAdaptor_FClass2d` is constructed with `Precision::Confusion()` as `TolUV`
(`BRepClass3d_SolidExplorer.cxx:163`, `:533`; `IntCurvesFace_Intersector` construction at
`:924`, `:541`) and then clamps it with the magic `m_Toluv = (Toluv > 4.0) ? 4.0 : Toluv`
(`BRepTopAdaptor_FClass2d.cxx:625-627`) — a guard that only makes sense if someone passes
a 3D value into a UV slot. **In our port, `Toluv` is a genuine UV tolerance, produced by
`uv_tol_at()`.** Passing a 3D number where a UV number is expected is exactly the class of
bug this port exists to eliminate.

### 2.4 POINT-IN-FACE CLASSIFICATION for curved trimmed faces

#### 2.4.1 The cached context

`IntTools_Context` (`IntTools_Context.cxx:52-193`) caches, keyed by shape:

| cache | member | what it holds | init line |
|---|---|---|---|
| face classifier | `myFClass2dMap` | `IntTools_FClass2d(F_forward, tol3d(F))` | :225-243 |
| point→surface projector | `myProjPSMap` | `GeomAPI_ProjectPointOnSurf` on `(S, Umin,Usup,Vmin,Vsup, myPOnSTolerance)` with `SetExtremaFlag(Extrema_ExtFlag_MIN)` | :247-265 |
| point→edge projector | `myProjPCMap` | `GeomAPI_ProjectPointOnCurve(C3D, f, l)` | :269-286 |
| point→curve projector | `myProjPTMap` | keyed by the curve handle, full range | :290-308 |
| solid classifier | `mySClassMap` | `BRepClass3d_SolidClassifier(solid)` | :312-323 |
| surface adaptor | `mySurfAdaptorMap` | `BRepAdaptor_Surface(F, true)` (restricted) | :327-339 |
| 2D hatcher | `myHatcherMap` | see §2.6.2 | :343-395 |
| AABB | `myBndBoxDataMap` | `BRepBndLib::Add` | :197-212 |
| OBB | `myOBBMap` | `BRepBndLib::AddOBB` then `Enlarge(theGap)` | :399-414 |

**`myPOnSTolerance = 1.e-12`** — set in both constructors, `IntTools_Context.cxx:65` and
`:84`. It is the *projector's convergence* tolerance (the Newton stop criterion in
`GeomAPI_ProjectPointOnSurf::Init`), **not** an acceptance tolerance. Changing it clears
the projector cache (`SetPOnSProjectionTolerance`, `:1004-1008`).

`UVBounds` (`:1028-1039`) reads the **restricted** adaptor's parameter range
(`BRepAdaptor_Surface(F, true)`), i.e. the UV bounds of the *face's trims*, not the
surface's natural domain.

#### 2.4.2 `IsValidPointForFace` — the three tolerances

`IntTools_Context.cxx:647-673`, verbatim structure:

```
GeomAPI_ProjectPointOnSurf& aProjector = ProjPS(aF);   // projector tol = myPOnSTolerance = 1e-12
aProjector.Perform(aP);
bFlag = aProjector.IsDone();
if (bFlag) {
    Umin = aProjector.LowerDistance();                 // UNSQUARED 3D distance
    if (Umin > aTol) return !bFlag;                    // strict >, reject          :663-666
    aProjector.LowerDistanceParameters(U, V);
    bFlag = IsPointInOnFace(aF, gp_Pnt2d(U,V));        // ON accepted as inside      :670
}
return bFlag;
```

The three tolerances, distinct and non-interchangeable:

1. **Projector tolerance `1e-12`** — how precisely the foot of the perpendicular is found.
   Must be far below any acceptance tolerance so that the *measured* distance is exact.
2. **`aTol`, the strict unsquared distance reject** — `if (Umin > aTol) return false`.
   Note: unsquared (`LowerDistance()`, not `LowerDistanceSquared`), and **strict `>`**, so
   a distance exactly equal to `aTol` is accepted. Callers pass their own 3D criterion:
   `IntTools_EdgeFace::IsProjectable` passes `myCriteria` (§2.1.7 E/F formula,
   `IntTools_EdgeFace.cxx:187`); `PutBoundPaveOnCurve` passes `aTolR3D`
   (`BOPAlgo_PaveFiller_6.cxx:2340`); `AreFacesSameDomain` passes its own `aTol`
   (`BOPTools_AlgoTools.cxx:1202`).
3. **The classifier band = the face's own tolerance**, entered when the projector's UV
   point is fed to `IsPointInOnFace` → `StatePointFace` → `FClass2d(aF)` →
   `IntTools_FClass2d` constructed with `aTolF = BRep_Tool::Tolerance(aFF)`
   (`IntTools_Context.cxx:235-238`). **`ON` is accepted as inside**, because
   `IsPointInOnFace` is `aState != TopAbs_OUT` (`:639-643`).

Contrast `IsPointInFace(F, P2d)` (`:604-608`): `aState != TopAbs_OUT && aState != TopAbs_ON`
— **ON rejected**. And `IsPointInFace(P3d, F, aTol)` (`:612-635`): projects, requires
`aDist < aTol` (**strict `<`** here, unlike `IsValidPointForFace`'s `>` reject — the
boundary case differs by design), then the **ON-rejecting** `IsPointInFace`.

Summary of the four face predicates you must port with these exact boundary conventions:

| predicate | distance gate | ON verdict | line |
|---|---|---|---|
| `IsPointInFace(F, P2d)` | — | OUT | :604-608 |
| `IsPointInOnFace(F, P2d)` | — | IN | :639-643 |
| `IsPointInFace(P3d, F, tol)` | `dist < tol` | OUT | :612-635 |
| `IsValidPointForFace(P3d, F, tol)` | `dist <= tol` (rejects `>`) | IN | :647-673 |

And the two composites: `IsValidPointForFaces` = `IsValidPointForFace` on both, short-circuit
on the first failure (`:677-691`); `IsValidBlockForFaces` (`:717-755`) evaluates the block's
**intermediate** parameter (§2.6, `IntTools_Tools::IntermediatePoint`), and — crucially —
if a **pcurve exists** for a face it uses the pcurve's own 2D point with `IsPointInOnFace`,
falling back to the 3D projection only when the pcurve is null (`:743-751`). Using the
pcurve avoids a projection entirely; that is the fast and *correct* path.

#### 2.4.3 `IntTools_FClass2d` — the actual curved-face classifier

`IntTools_FClass2d.cxx`. Two phases: build a UV polygon per wire (`Init`, `:77-621`), then
classify against the polygons with an exact-classifier fallback (`Perform`, `:637-804`).

**Init, per wire** (`:120-575`):

- Walk edges with `BRepTools_WireExplorer` (ordered), skip orientations other than
  FORWARD/REVERSED (`:152-155`). A null pcurve **aborts the whole Init** (`:157-161`) —
  the classifier then has `nbtabclass == 0` and `Perform` returns `TopAbs_IN`
  unconditionally (`:639-643`). *This is a real OCCT failure mode: a face with one missing
  pcurve classifies everything as inside.* Our port must return `UNKNOWN`, not `IN`.
- Degeneracy detection (`:199-220`): even if the edge is not flagged degenerate, sample
  `NBSTEPS = 10` points; if all lie within `sqrt(0.25·Confusion²) = Confusion/2` of the
  midpoint value, treat as degenerate.
- Sample count: `nbs = Geom2dInt_Geom2dCurveTool::NbSamples(C)`, and **`if (nbs > 2) nbs *= 4`**
  (`:229-233`). For `nbs == 2` the three parameters are `uFirst`, `uFirst + 0.0025·(uLast-uFirst)`,
  `uLast` (`:254-260`) — the `0.0025` nudge exists so a straight edge contributes a
  non-degenerate direction.
- Consecutive-sample suppression (`:305-343`): a sample whose **3D** point is within
  `Precision::Confusion()` of the previous *and* within `Confusion` of the segment
  midpoint's 3D point is dropped (`aPrCf2 = Confusion²`). Note this is a **3D** test used
  to filter a **2D** polygon — deliberate.
- Deflection `FlecheU/FlecheV` (`:345-364`): for each interior sample, the perpendicular
  offset of `P[i-1]` from the chord `P[i-2]→P[i]`, per axis, maxed.
- **Adaptive re-discretisation** (`:454-534`): `Poly::PolygonProperties(SeqPnt2d, aS, aPer)`
  gives signed area and perimeter; `anExpThick = max(2·|aS|/aPer, 1e-7)` is the polygon's
  "expected thickness"; while `aDefl > anExpThick && aDiscrDefl > 1e-7`, re-discretise
  every edge with `GCPnts_QuasiUniformDeflection(C, aDiscrDefl)` starting from
  `aDiscrDefl = min(aDefl·0.1, anExpThick·10)` and shrinking by `·0.1` each pass. *Purpose:
  a coarse polygon of a thin face self-intersects and classifies wrongly.*
- Floors: `if (FlecheU < Toluv) FlecheU = Toluv;` likewise V (`:536-544`).
- Orientation: `aS > 0` → outer (`TabOrien = 1`, `myIsHole = false`); `aS < 0` → hole
  (`TabOrien = 0`); `|aS| < Precision::SquareConfusion()` → **bad wire**, `TabOrien = -1`
  (`:548-565`). A single bad wire sets `TabOrien(1) = -1` for the whole face (`:583-586`),
  which forces the exact classifier for every query.
- Periodic bookkeeping (`:588-619`): for Cone/Cylinder/Torus/Sphere/Revolution,
  `U1 = Umin - max(0, 2π - (Umax-Umin))/2`, `U2 = U1 + 2π`; for Torus the same in V.

**Perform** (`:637-804`):

```
for (;;) {
  dedans = 1;
  bUseClassifier = (TabOrien(1) == -1);
  if (!bUseClassifier) {
     for each wire n:
        cur = TabClass(n).SiDans(Puv);
        if (cur == 1  && TabOrien(n) == 0) { dedans = -1; break; }   // inside a hole
        if (cur == -1 && TabOrien(n) == 1) { dedans = -1; break; }   // outside the outer
        if (cur == 0) { dedans = 0; break; }                          // ON -> exact classifier
     if (dedans == 0) bUseClassifier = true;
     else aStatus = (dedans == 1) ? IN : OUT;
  }
  if (bUseClassifier) {
     aURes = surf->UResolution(Toluv);  aVRes = surf->VResolution(Toluv);
     bUIn = (u >= Umin && u <= Umax);   bVIn = (v >= Vmin && v <= Vmax);
     aFCTol = (bUIn == bVIn) ? min(aURes,aVRes) : (!bUIn ? aURes : aVRes);
     BRepClass_FClassifier aClassifier;
     aClassifier.Perform(*myFExplorer, Puv, aFCTol);
     aStatus = aClassifier.State();
  }
  if (!RecadreOnPeriodic || (!IsUPer && !IsVPer)) return aStatus;
  if (aStatus == IN || aStatus == ON) return aStatus;
  // else shift by one period in U, then in V, and retry
}
```

The periodic retry ladder is `:768-802`: first replace `u` by the period-adjusted `uu`
(computed with `GeomInt::AdjustPeriodic(uu, Umin, Umax, uperiod, uu, du)`, `:671`), then
keep adding `uperiod` until `u > Umax`, then advance `v` the same way, then give up.
**A point on a periodic surface is IN if it is IN for any period shift.**

`TestOnRestriction(P2d, Tol, RecadreOnPeriodic)` (`:808-943`) is the same walk with
`SiDans_OnMode(Puv, Tol)` instead of `SiDans`, and `dedans == 0` maps to `TopAbs_ON`
rather than to the exact classifier — this is the "is the point on the boundary within
`Tol`" query.

#### 2.4.4 `CSLib_Class2d::SiDans` — the band, exactly

`src/FoundationClasses/TKMath/CSLib/CSLib_Class2d.cxx`. This is the polygon classifier; the
"band" the mission asks about lives here.

**Normalisation** (`init`, `:43-96`): the polygon points are mapped to `[0,1]²` by
`transformToNormalized(x, min, range) = (x-min)/range` when `range > 1e-10`, else identity
(`:30-38`); **the tolerances are normalised the same way**: `myTolU /= aDu; myTolV /= aDv;`
(`:88-95`). Degenerate input (`UMax <= UMin`, `VMax <= VMin`, fewer than 3 points) sets
`myPointsCount = 0` (`:57-62`) and every query then returns `Uncertain` (`:143-146`).

**`SiDans(P)`** (`:141-187`):

1. De-normalised box reject: `aTolU = myTolU*(myUMax-myUMin)`, `aTolV = myTolV*(myVMax-myVMin)`;
   if `P` is outside the polygon bbox inflated by those, return `Outside` (`:152-160`).
2. Normalise `P`.
3. `aResult = internalSiDansOuOn(x, y)`; if `Uncertain` (i.e. **ON**), return it (`:167-171`).
4. **The band**: if `myTolU > 0 || myTolV > 0`, classify the four corners
   `(x±myTolU, y±myTolV)` with the plain `internalSiDans`; **if any disagrees with the
   point's own verdict, return `Uncertain` (= ON)** (`:174-184`).

**`internalSiDansOuOn(x,y)`** (`:278-340`) — ray casting to `+x` with ON detection:

- For each polygon vertex, `if (|Vx-x| < myTolU && |Vy-y| < myTolV) return Uncertain;`
  — **ON at a vertex** (`:293-297`).
- For each edge spanning `x` (`(Vprev.x - x)*(Vcurr.x - x) < 0`) and **not near-vertical**
  (`|edgeDx| > Precision::PConfusion()`), interpolate the edge's `y` at `x`; if
  `|interpY - y| <= myTolV` return `Uncertain` — **ON on an edge** (`:301-316`). Vertical
  edges are deliberately skipped here; their ON case is covered by the vertex test.
- Crossing count with the standard sign-change rule, with the `aPrevDx > 0 && aCurrDx > 0`
  fast path and the explicit x-intersection otherwise (`:318-336`).
- Odd crossings → `Inside`, else `Outside` (`:339`).

`SiDans_OnMode(P, Tol)` (`:191-230`) is the same but takes `Tol` **already in normalised
units** for the corner probes and in raw units for the bbox reject — a genuine unit
inconsistency in OCCT; our port takes a single `UVTol{du,dv}` and applies it consistently.

**Port note**: the band being *four corner probes that must all agree* — rather than a
distance-to-polygon computation — is what makes ON detection cheap and, more importantly,
**consistent with the IN/OUT rule itself**: a point is ON exactly when perturbing it by the
tolerance can change the answer. That is the correct definition and we should adopt it
verbatim. It also means the band automatically widens where the polygon is dense and
narrows where it is smooth, with no extra logic.

#### 2.4.5 The exact fallback classifier

When the polygon says ON, OCCT re-classifies against the **real pcurves**:
`BRepClass_FClassifier::Perform` → `TopClass_FaceClassifier::Perform`
(`src/ModelingAlgorithms/TKGeomAlgo/TopClass/TopClass_FaceClassifier.pxx:31-140`):

1. `theFexp.CheckPoint(aPoint)` in a loop until it returns true
   (`BRepClass_FaceExplorer.cxx:67-102`): if `Epsilon(dist(P, uv-centre))` exceeds the
   domain extent, the point is pulled toward the centre — a guard against classifying a
   point so far away that `Epsilon` swamps the domain.
2. `theFexp.Segment(P, L, Par)` picks a 2D ray from `P` toward a point on some edge;
   `OtherSegment` produces successive alternatives. **The probe parameters are asymmetric
   constants**: `Probing_Start = 0.123`, `Probing_End = 0.7`, `Probing_Step = 0.2111`
   (`BRepClass_FaceExplorer.cxx:30-32`), `myMaxTolerance = 0.1` (`:40`).
3. For each wire/edge not rejected, `theClassifier.Compare(anEdge, anEdgeOri)` intersects
   the ray with the real pcurve; the closest intersection decides. **If the state comes
   back `ON`, return immediately** (`:120-124`). If a wire comes back `OUT`, return
   immediately (`:127-131`).
4. If the classifier ended at a "head or end" (the ray hit an edge endpoint) and the state
   is not resolved, take **another** segment and repeat (`:135-138`).
5. `State()` (`:145-156`): `rejected → OUT`; `noWires → IN`; else the classifier's state.

**`noWires → IN`** is the natural-boundary case (a face with no trims is the whole
surface). Port it explicitly.

### 2.5 POINT-IN-SOLID CLASSIFICATION

#### 2.5.1 Entry point and the two tolerances actually used

`BOPTools_AlgoTools::ComputeState(const gp_Pnt&, const TopoDS_Solid&, double theTol, ctx)`
(`BOPTools_AlgoTools.cxx:790-803`) → `ctx->SolidClassifier(theRef).Perform(theP, theTol)`.

Call sites and their `theTol`:

- `BOPAlgo_BuilderSolid.cxx:857` — `Precision::Confusion()` (`1e-7`), classifying a face of
  one shell against another shell during solid assembly.
- `BOPAlgo_Tools.cxx:1841` — `Precision::Confusion()`.
- `BOPAlgo_Builder_3.cxx:836` — **`1.e-11`**, classifying a standalone sub-shape against a
  solid. Deliberately far tighter.

Shape-level dispatch `ComputeStateByOnePoint` (`BOPTools_AlgoTools.cxx:623-655`):
VERTEX → its point; EDGE → the point at `IntTools_Tools::IntermediatePoint(t1,t2)`
(`:734-785`, with `±10` offsets for semi-infinite ranges and `0.` for bi-infinite);
FACE → §2.6.3; anything else → recurse into the first sub-shape.

#### 2.5.2 `BRepClass3d_SClassifier::Perform` — the algorithm

`src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SClassifier.cxx:203-523`.

State encoding (`:525-542`): `myState` 1 = faulty/rejected, 2 = ON, 3 = IN, 4 = OUT;
`State()` maps anything else to **OUT** (`:540-541` — "return OUT state when there is an
error during execution"). Port this: an unresolved classification is OUT, and that is
recorded, not silently equal to a computed OUT.

```
1. if (SolidExplorer.Reject(P)) { myState = 3 /*IN*/; return; }        // solid without faces
                                                                       // = the whole space
2. // TOLERANCE-BAND ON TEST, before any ray work:
   BRepClass3d_BndBoxTreeSelectorPoint aSelectorPoint(mapEV);
   aSelectorPoint.SetCurrentPoint(P);
   if (aTree.Select(aSelectorPoint) > 0) { myState = 2 /*ON*/; return; }   // :219-230
3. loop (isFaultyLine):
   a. iFlag = Segment(P,L,Par) or OtherSegment(P,L,Par)                    // :259-266
      - the face index must strictly increase, else myState=1 (Faulty), return  :268-278
      - iFlag==1 -> ON (infinite face);  iFlag==2 -> OUT;  iFlag==3 -> skip this face
   b. line-vs-edge/vertex interference via the same UB-tree, selector Line  :303-361
      - a vertex hit records NearFaultPar
      - an edge hit uses GetTransi(f1,f2,e,param,L,tran) to decide IN/OUT from the two
        adjacent face normals; Tst==1 and |Lpar| < |parmin| -> Trans(parmin,tran,myState)
   c. for every face: prolong the segment and intersect
        addW = max(10*Tol, 0.01*Par); grow to the face's bbox if finite       :380-393
        minW = -AddW;  maxW = min(Par*10, Par + addW);                         :395-396
        Intersector3d.Perform(L, minW, maxW);
      - NbPnt()==0 && IsParallel(): project P onto the face
        (Extrema_ExtPS with PConfusion,PConfusion, MIN flag); if d <= Tol*Tol and the
        UV point classifies IN or ON -> myState = 2 (ON)                       :400-443
      - for each intersection with |W| < |parmin| - PConfusion:
          * |parmin| <= Tol                 -> ON                              :451-456
          * state IN  && tran == Tangent    -> ignore this point               :463-470
          * state IN  && tran != Tangent    -> Trans(parmin,tran,myState)      :472-473
          * state ON                        -> isFaultyLine = true, restart    :477-481
   d. if (NearFaultPar != RealLast() && |parmin| >= |NearFaultPar| - PConfusion)
         isFaultyLine = true;                                                  :511-515
```

`Trans` (`:728-746`): if `parmin < 0` the transition is reversed; `Out → IN(3)`, else
`OUT(4)`. (Reads backwards but is right: the ray leaves the solid at its first hit ⟹ the
origin was inside.)

`GetTransi` (`:654-724`): needs both adjacent faces' normals at the edge parameter; returns
`-1` (faulty) if either normal is unavailable, if the line is orthogonal to either normal
within `Precision::Angular()`, or if the normals are parallel and the line is tangent;
returns `0` (skip) if the projected line direction has mixed signs against the two normals.

#### 2.5.3 The ON band is edge/vertex tolerance, evaluated exactly

`BRepClass3d_BndBoxTreeSelectorPoint::Accept`
(`src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_BndBoxTree.cxx:25-71`):

- EDGE: `EdgeTSq = Tolerance(E)²`; run `Extrema_ExtPC(myP, BRepAdaptor_Curve(E), f, l)`;
  if any `SquareDistance(i) < EdgeTSq` → accept, set `myStop = true` (`:34-56`).
- VERTEX: `VertTSq = Tolerance(V)²`; `VPnt.SquareDistance(myP) < VertTSq` → accept (`:57-69`).

The tree is built in `BRepClass3d_SolidExplorer::InitShape` (`:936-981`) from **all
non-INTERNAL/EXTERNAL, non-degenerate edges and their vertices**, boxed by `BRepBndLib::Add`.

`BRepClass3d_BndBoxTreeSelectorLine::Accept` (`BRepClass3d_BndBoxTree.cxx:75-156`) is the
line version: `Extrema_ExtCC` for edges (with `IsParallel()` ⟹ `myIsValid = false`,
"tangent case is invalid for classification"), `Extrema_ExtPElC` for vertices, both against
the *squared* entity tolerance.

**This is the correct shape of a point-in-solid ON test**: exact extrema against the real
curves, radius = the entity's own stored tolerance, spatially indexed. It is not a mesh, not
a ray count, and not a global epsilon.

#### 2.5.4 The ray (`Segment` / `OtherSegment`) and its retry ladder

`BRepClass3d_SolidExplorer::OtherSegment` (`:493-787`). This is the part everyone
underestimates.

- `TolU = TolV = Precision::PConfusion()` (`:495-496`).
- Per face: compute the restricted UV bounds; **reject the whole solid as OUT (`return 2`)
  if the face's UV extent is degenerate**: `epsU = max(PConfusion·max(|U2|,|U1|), PConfusion)`,
  and `|U2-U1| < epsU || |V2-V1| < epsV` (`:559-565`). Note `epsU` is *parameter-magnitude
  scaled* — the only correct way to compare parameters without knowing the metric.
- `Extrema_ExtPS(P, GA, TolU, TolV, Extrema_ExtFlag_MIN)` on the **unrestricted** surface
  (`:575-576`); pick the nearest extremum whose UV lies inside the face bounds (`:587-600`).
- **`aDist2Tresh = 1.e-24`** (`:602`): if the squared distance is below it, the point is
  *on the surface*; then either `return 1` (ON, infinite face) or run
  `BRepClass_FaceClassifier::Perform(face, aPuv, Precision::PConfusion())` and return `1`
  for IN/ON, **`3` (skip this face)** for OUT (`:604-633`).
- Otherwise find an interior point of the face (`PointInTheFace`, §2.6.4), form
  `L = gp_Lin(P, V)` with `V = P→APoint`, and score it by
  `tt = |Norm·V| / (|Norm|·|V|)` — the cosine between the face normal at the sampled point
  and the ray. **Keep the best; accept immediately once `maxscal > 0.2`** (`:670-691`).
  *The ray is chosen to hit a face as perpendicularly as possible.* That is the entire
  robustness trick.
- Up to `IndexPoint < 200 && NbPointsOK < 16` candidate points per face (`:693`).
- If no face produced a good ray, the **`myParamOnEdge` ladder** fires
  (`:738-784`): `0.512345` (initial, `InitShape:905`) → `0.4` → `0.6` → `0.3` → `0.7` →
  `0.2` → `0.8` → `0.1` → `0.9` → then repeatedly `*= 0.5`, giving up below `0.0001` with a
  degenerate `+X` ray (`:774-783`). Every one of those is an **asymmetric** parameter.
- `aTestInvert` (`:531-549`) re-checks each face with
  `BRepTopAdaptor_FClass2d(face, Precision::Confusion()).PerformInfinitePoint() == TopAbs_IN`
  and, if so, rebuilds the intersector unrestricted — the "inverted trim" repair.

#### 2.5.5 `PerformInfinitePoint` — is the shell a hole in space?

`BRepClass3d_SClassifier.cxx:82-199`. Take a random interior point of a face, shoot the
**reversed normal**, find the minimum-parameter intersection; if it is a genuine (non-tangent)
crossing of a face interior, the transition gives IN/OUT. Randomisation:
`math_BullardGenerator`, `aParam = 0.1 + 0.8·NextReal()` — **uniform in `[0.1, 0.9]`**
(`:134`), up to `NB_MAX_POINTS_PER_FACE = 10` tries per face (`:124-125`), over all faces.

**Port note**: a *seeded* generator is required for G9 (determinism). Use a fixed-seed PRNG
per call, seeded from a hash of the shape's content, never from wall time.

### 2.6 INTERIOR-POINT SELECTION — the asymmetric constants

#### 2.6.1 `IntermediatePoint`

Two identical copies:

```
// define parameter division number as 10*e^(-PI) = 0.43213918
const double PAR_T = 0.43213918;
return (1. - PAR_T)*aFirst + PAR_T*aLast;
```

- `BOPTools_AlgoTools2D::IntermediatePoint(first,last)` —
  `src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools2D.cxx:404-411`.
- `IntTools_Tools::IntermediatePoint(first,last)` —
  `src/ModelingAlgorithms/TKBO/IntTools/IntTools_Tools.cxx:254-259`.

Also `BRepCheck_Solid.cxx:129`, `BRepCheck_Wire.cxx:1622`, `BRepSweep_Rotation.cxx:664`
(as `aTx`), `TopOpeBRepBuild_Tools.cxx:55-56`, `TopOpeBRepBuild_Builder1_1.cxx:44-45`.

`10·e^{-π} = 0.4321391826...`. The number is irrational-looking on purpose: it is never
`1/2`, never `1/3`, never a dyadic rational, so it cannot land on a symmetry plane, a seam,
a knot, or a shared vertex of symmetric geometry.

Users of `IntermediatePoint` in this subsystem:
`IsValidBlockForFace` (`IntTools_Context.cxx:705`), `IsValidBlockForFaces` (`:732`),
`ComputeState(edge)` (`BOPTools_AlgoTools.cxx:777`), `DoSplitSEAMOnFace`
(`BOPTools_AlgoTools3D.cxx:152`, `:269`), `GetNormalToFaceOnEdge`
(`BOPTools_AlgoTools3D.cxx:339`), `PointNearEdge` (`BOPTools_AlgoTools3D.cxx:708`),
`PointInFace` U-line placement (`BOPTools_AlgoTools3D.cxx:917`) and V-parameter pick
(`:1054-1055`).

#### 2.6.2 `BOPTools_AlgoTools3D::PointInFace` — the hatcher probe

`BOPTools_AlgoTools3D.cxx:906-1066`. Three overloads; the workhorse is the `Geom2d_Curve`
one (`:992-1066`).

**Overload A — no reference edge** (`:906-938`):

```
theContext->UVBounds(theF, aUMin, aUMax, aVMin, aVMax);
aUx = IntTools_Tools::IntermediatePoint(aUMin, aUMax);       // 0.43213918 of the U span
for (i = 0; i < 2; ++i) {
    aL2D = new Geom2d_Line(gp_Pnt2d(aUx, 0.), gp_Dir2d(0,1));   // vertical line at aUx
    iErr = PointInFace(theF, aL2D, theP, theP2D, theContext);
    if (iErr == 0) break;
    aUx = aUMax - (aUx - aUMin);                                 // mirror and retry
}
```

**This is the constant the mission names**: OCCT probes a face's interior by hatching it
with a vertical line at `u = uMin + 0.43213918·(uMax-uMin)`, **not** at the midpoint, and
the single retry mirrors to `uMax - 0.43213918·(uMax-uMin)`.

**Overload B — near a reference edge** (`:942-988`): 2D line through the edge point at
parameter `theT`, direction = the **left normal** of the pcurve tangent
(`aD2D = (-tangent.Y(), tangent.X())`), reversed once if the edge is REVERSED and again if
the face is REVERSED, trimmed to `[0, Precision::Infinite()]`.

**The hatching core** (`:992-1066`), with the cached `Geom2dHatch_Hatcher`
(`IntTools_Context.cxx:343-395`, constants `aTolHatch2D = aTolHatch3D = 1.e-8`,
`aTolArcIntr = aTolTangfIntr = 1.e-10`, `aEpsT = Precision::PConfusion()`; edges whose
pcurve range satisfies `|aU1-aU2| < aEpsT` are skipped, `:382-385`):

```
aHatcher.ClrHatchings(); aIH = aHatcher.AddHatching(aHCur);
aHatcher.Trim();              if (!TrimDone(aIH))    iErr = 1;
aHatcher.ComputeDomains(aIH); if (!IsDone(aIH))      iErr = 2;
if (NbDomains(aIH) == 0)                             iErr = 2;
aDomain = Domain(aIH, 1);
if (!aDomain.HasFirstPoint())                        iErr = 3;
if (!aDomain.HasSecondPoint())                       iErr = 4;
aV1 = FirstPoint().Parameter(); aV2 = SecondPoint().Parameter();
aVx = (theDt2D > 0. && (aV2-aV1) > theDt2D) ? (aV1 + theDt2D)
                                            : IntTools_Tools::IntermediatePoint(aV1, aV2);
theL2D->D0(aVx, theP2D);  aS->D0(theP2D.X(), theP2D.Y(), theP);
```

So the point is taken at `0.43213918` **along the first hatch domain** as well — asymmetric
in both directions — unless a caller-supplied offset `theDt2D` applies. `iErr == 5` is the
null-pcurve case of overload B (`:955-960`).

`PointNearEdge` (`:616-663`) computes the default offset:
`dT2D = 10·MinStepIn2d() = 1.e-4` (`MinStepIn2d()` is `1.e-5`, `:724-728`), `×10` again for
Cylinder/Sphere (`:631-634`), then `dTx = 2·(tolE + tolF)` and `dT2D = max(dT2D, dTx)`
(`:636-642`). If the offset point is not in-or-on the face, fall back to `PointInFace`
(`:645-660`), and if that fails too, `iErr = 2` ("point is out of the face").

The low-level offset (`:525-612`) has a real-geometry correction: for
`tolE > 1e-5 || tolF > 1e-5`, and surface type ≠ Sphere, `transVal = aDt2D + tolE + tolF`,
and **for a Cylinder** the linear offset is converted to an angle:
`dT = 1 - transVal/R; if (-1 <= dT <= 1) transVal = acos(dT)` (`:584-595`). That is a
metric conversion done by hand, for exactly the reason of §2.3.

#### 2.6.3 Face-vs-solid state

`BOPTools_AlgoTools::ComputeState(face, solid, tol, bounds, ctx)`
(`BOPTools_AlgoTools.cxx:660-712`):

1. Prefer an edge of the face that is **not** in the solid's edge map, and classify the
   `IntermediatePoint` of that edge (`:670-686`). Cheapest and most reliable.
2. Otherwise `BOPTools_AlgoTools3D::PointInFace(theF, aP3D, aP2D, ctx)` (§2.6.2).
3. If the hatcher fails, walk the face's non-degenerate edges calling `PointNearEdge`
   until one succeeds (`:690-704`).
4. Only if a point was obtained do we classify it; otherwise the state stays `UNKNOWN`
   (`:706-711`).

#### 2.6.4 `PointInTheFace` / `FindAPointInTheFace` — the UV probe grid

`BRepClass3d_SolidExplorer.cxx`.

`PointInTheFace` (`:241-425`): `du = (U2-U1)/6`, `dv = (V2-V1)/6`, floored at `1e-12`
(`:255-264`); walk four quadrant grids **outward from the centre** (`u = ±du + (U1+U2)/2`,
etc., `:299-367`); then a finer sweep `du = (U2-U1)/37`, `dv = (V2-V1)/37` over the whole
domain (`:369-396`); and **only as the last candidate** the exact centre
`u = (U1+U2)/2, v = (V1+V2)/2` (`:397-409`). Every candidate is accepted only if
`ClassifyUVPoint(...) == TopAbs_IN`, where `ClassifyUVPoint` (`:221-237`) **first** tests the
3D point against the edge/vertex tolerance tree (returning `ON` if it is inside any entity's
tolerance) and only then asks the face intersector. `IndexPoint` lets successive calls
continue where the last stopped.

`FindAPointInTheFace` (`:74-187`): for each edge, evaluate the pcurve at
`(last-first)·param + first` with the caller's `param`, take the inward 2D normal, offset by
`TolInit = 0.00001` (`:110`, `:116`), shoot a `BRepClass_FacePassiveClassifier` ray with
`RealEpsilon()` tolerance (`:118`), find the closest other-edge intersection `ParamInit`,
then **`ParamInit *= 0.41234`** (`:158`) — again an asymmetric fraction — and validate the
resulting UV with `BRepTopAdaptor_FClass2d(face, Precision::Confusion()).Perform(...) == IN`
(`:163-169`), plus a non-degenerate-normal check `|Su × Sv| > gp::Resolution()` (`:175-178`),
looping (halving `ParamInit` each time) until `ParamInit < Precision::PConfusion()` (`:180-183`).

**The complete asymmetric-constant table to port:**

| constant | value | site | role |
|---|---|---|---|
| `PAR_T` | `0.43213918` | `IntTools_Tools.cxx:257`, `BOPTools_AlgoTools2D.cxx:407` | intermediate parameter on any interval |
| mirror of `PAR_T` | `1 - 0.43213918` | `BOPTools_AlgoTools3D.cxx:933` | second hatch attempt |
| `0.41234` | | `BRepClass3d_SolidExplorer.cxx:158` | shrink toward the boundary point |
| `myParamOnEdge` seed | `0.512345` | `BRepClass3d_SolidExplorer.cxx:905` | first edge probe parameter |
| ladder | `0.4, 0.6, 0.3, 0.7, 0.2, 0.8, 0.1, 0.9`, then `×0.5` | `BRepClass3d_SolidExplorer.cxx:739-784` | retry parameters |
| `Probing_Start / End / Step` | `0.123 / 0.7 / 0.2111` | `BRepClass_FaceExplorer.cxx:30-32` | 2D classifier ray targets |
| random interior | `0.1 + 0.8·U(0,1)` | `BRepClass3d_SClassifier.cxx:134` | `PerformInfinitePoint` |
| `TolInit` | `0.00001` | `BRepClass3d_SolidExplorer.cxx:110` | UV inward offset |
| `nbs == 2` nudge | `0.0025` | `IntTools_FClass2d.cxx:256` | second polygon sample |
| grid divisors | `6`, then `37` | `BRepClass3d_SolidExplorer.cxx:255-256`, `:369-370` | interior-point search; note **37 is prime** |

The pattern is uniform and deliberate: **no probe location is a simple fraction, and the
fallback ladders are odd/prime-flavoured so successive attempts do not land on the same
symmetry.** Our corpus (origin-centred boxes, coaxial cylinders, spheres at the origin,
A-op-A) is maximally symmetric, so this is not a stylistic detail for us — it is the
difference between working and not.

---

## 3. DATA STRUCTURES AND C++ DECLARATIONS FOR OUR PORT

New files: `src/brep_tol.h` / `src/brep_tol.cpp` (tolerance model + metric),
`src/brep_classify.h` / `src/brep_classify.cpp` (point classification + context cache).
Both are consumed by `brep_bds` (ARCHITECTURE_v2 §1) and by the splitter.

```cpp
#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>
#include "point.h"
#include "vector.h"
#include "nurbssurface.h"
#include "nurbscurve.h"

namespace session_cpp {

// ---------------------------------------------------------------------------
// 3.1 Universal constants. Mirror of Precision.hxx; single definition site.
// ---------------------------------------------------------------------------
namespace prec {
inline constexpr double CONFUSION        = 1e-7;   // Precision.hxx:165
inline constexpr double SQ_CONFUSION     = 1e-14;  // Precision.hxx:169
inline constexpr double INTERSECTION     = 1e-9;   // Precision.hxx:220
inline constexpr double APPROXIMATION    = 1e-6;   // Precision.hxx:235
inline constexpr double ANGULAR          = 1e-12;  // Precision.hxx:123
inline constexpr double PCONFUSION       = 1e-9;   // Precision.hxx:334 (fallback only)
inline constexpr double INFINITE         = 2e100;  // Precision.hxx:371
inline constexpr double DTOLERANCE       = 1e-12;  // BOPTools_AlgoTools.hxx:70
inline constexpr double PROJ_TOL         = 1e-12;  // IntTools_Context.cxx:65 (projector only)
inline constexpr double MAX_CORRECT_TOL  = 0.05;   // BOPAlgo_Builder.cxx:472 (absolute reject)
inline constexpr double BIG_TOL          = 1e10;   // BRepLib.cxx:1859
// Asymmetric probe constants (§2.6). NEVER replace with 0.5.
inline constexpr double PAR_T            = 0.43213918;  // 10*e^-pi
inline constexpr double PAR_SHRINK       = 0.41234;     // BRepClass3d_SolidExplorer.cxx:158
inline constexpr double PROBE_START      = 0.123;       // BRepClass_FaceExplorer.cxx:30
inline constexpr double PROBE_END        = 0.7;         // :31
inline constexpr double PROBE_STEP       = 0.2111;      // :32
inline constexpr double EDGE_PARAM_SEED  = 0.512345;    // BRepClass3d_SolidExplorer.cxx:905
// Pads (§2.1.8): each is an acceptance boundary; do not unify.
inline constexpr double PAD_VALIDATE     = 1.00001;
inline constexpr double PAD_SAMPLED      = 1.5;
inline constexpr double PAD_EDGETOL      = 1.4;
inline constexpr double PAD_PLANAR_EDGE  = 1.05;
inline constexpr double PAD_WIRE         = 1.01;
inline constexpr double PAD_CHECKEDGE_F  = 0.1;   // additive: + 0.1*tol
inline double eps_of(double v) { return std::nextafter(std::abs(v), 1e300) - std::abs(v); }
}

// ---------------------------------------------------------------------------
// 3.2 The tolerance store. Owned by the DS, never by the input operands.
//     G4: three write kinds, no fourth.
// ---------------------------------------------------------------------------
enum class TolKind { Raise, Set, Provisional };

class TolStore {
public:
    // Effective read: always clamped at CONFUSION (BRep_Tool.cxx:137/881/1314).
    double vertex(int nv) const { return std::max(v_[nv], prec::CONFUSION); }
    double edge  (int ne) const { return std::max(e_[ne], prec::CONFUSION); }
    double face  (int nf) const { return std::max(f_[nf], prec::CONFUSION); }
    // Raw read: needed by the rollback bookkeeping only.
    double vertex_raw(int nv) const { return v_[nv]; }

    void raise_vertex(int nv, double t) { v_[nv] = std::max(v_[nv], t); }
    void raise_edge  (int ne, double t) { e_[ne] = std::max(e_[ne], t); }
    void set_vertex  (int nv, double t) { v_[nv] = t; }   // shrink-capable, §2.1.3 W10-W13
    void set_edge    (int ne, double t) { e_[ne] = t; }
    void set_face    (int nf, double t) { f_[nf] = t; }

    // --- provisional layer (§2.1.4), scope = one face-face pair -------------
    void scope_begin()  { saved_.clear(); }
    void grow_provisional(int nv, double t) {           // PutPaveOnCurve analogue
        if (!saved_.count(nv)) saved_[nv] = v_[nv];     // save PRE-growth value
        v_[nv] = std::max(v_[nv], t);
    }
    // The acceptance predicate must read THIS, not vertex(): PaveFiller_6.cxx:2974
    double vertex_for_acceptance(int nv) const {
        auto it = saved_.find(nv);
        return std::max(it != saved_.end() ? it->second : v_[nv], prec::CONFUSION);
    }
    void commit(int nv) { saved_.erase(nv); }                       // :1047-1048
    void raise_saved(int nv, double t) {                            // UpdateSavedTolerance :629
        auto it = saved_.find(nv);
        if (it != saved_.end() && it->second < t) it->second = t;
    }
    // Returns the vertices whose boxes the caller must REBUILD (not union): :1085-1089
    std::vector<int> scope_rollback() {
        std::vector<int> touched;
        for (auto& [nv, t] : saved_) { v_[nv] = t; touched.push_back(nv); }
        saved_.clear();
        return touched;
    }
private:
    std::vector<double> v_, e_, f_;
    std::unordered_map<int, double> saved_;
};

// ---------------------------------------------------------------------------
// 3.3 Metric: 3D tolerance <-> UV tolerance (§2.3). THE fix for G1/G2/G5.
// ---------------------------------------------------------------------------
struct UVTol { double du = 0, dv = 0; };

// Global upper bounds, analytic per surface type; GeomAdaptor_Surface.cxx:1818/1900.
double u_resolution(const NurbsSurface& S, double r3d);
double v_resolution(const NurbsSurface& S, double r3d);
// Control-net bound for general NURBS; BSplCLib.cxx:4316-4820.
void   nurbs_resolution(const NurbsSurface& S, double r3d, double& ures, double& vres);
// Local, per-point; combines the first fundamental form with the global bound.
UVTol  uv_tol_at(const NurbsSurface& S, double u, double v, double tol3d);
// Inverse: a UV displacement's worst-case 3D length at (u,v).
double uv_to_3d(const NurbsSurface& S, double u, double v, double du, double dv);
// Curve analogue: BRepLib_1.cxx:201 uses Resolution(tol) on curves the same way.
double curve_resolution(const NurbsCurve& C, double t, double r3d);

// ---------------------------------------------------------------------------
// 3.4 Classification results. G6: three states, always.
// ---------------------------------------------------------------------------
enum class State { In, On, Out, Unknown };

// Status codes, mirroring IntTools_Context::ComputePE (:437) and
// BOPTools_AlgoTools3D::PointInFace (:1010-1065). G12.
enum class ProbeErr : int {
    Ok = 0, TrimFailed = 1, NoDomain = 2, NoFirstPoint = 3, NoSecondPoint = 4, NoPCurve = 5,
    NotGeometric = -2, NoProjection = -3, TooFar = -4, Degenerate = -1
};

// ---------------------------------------------------------------------------
// 3.5 The cached classification context (analogue of IntTools_Context).
//     One per boolean operation; NOT thread-shared (OCCT's is not either).
// ---------------------------------------------------------------------------
class ClassifyContext {
public:
    explicit ClassifyContext(const BRep& shape);

    // ---- face-level -------------------------------------------------------
    // The 2D polygon classifier with the four-corner band (§2.4.4).
    State state_uv_in_face(int face, double u, double v) const;
    bool  is_in_face      (int face, double u, double v) const;  // ON -> false  (:604-608)
    bool  is_in_or_on_face(int face, double u, double v) const;  // ON -> true   (:639-643)
    // 3D entry points (§2.4.2). Note the differing boundary conventions.
    bool  is_point_in_face      (const Point& p, int face, double tol3d) const; // dist <  tol, ON->false
    bool  is_valid_point_for_face(const Point& p, int face, double tol3d) const;// dist <= tol, ON->true
    bool  is_valid_point_for_faces(const Point& p, int fA, int fB, double tol3d) const;
    // Block validity at the asymmetric intermediate parameter (§2.4.2 / :717-755).
    bool  is_valid_block_for_faces(double t1, double t2, const SectionCurve& c,
                                   int fA, int fB, double tol3d) const;

    // ---- solid-level ------------------------------------------------------
    State classify_point_solid(const Point& p, double tol3d) const;   // §2.5

    // ---- interior probes (§2.6) -------------------------------------------
    // Hatcher-equivalent: probe line at u = uMin + PAR_T*(uMax-uMin), retry mirrored.
    ProbeErr point_in_face(int face, Point& p3, double& u, double& v) const;
    ProbeErr point_near_edge(int face, int edge, double t, double dt_uv,
                             Point& p3, double& u, double& v) const;

    // ---- pair predicates (§2.1.7). fuzz is ADDED (G10). -------------------
    int compute_vv(const Point& a, double tolA, const Point& b, double tolB, double fuzz) const;
    int compute_ve(int nv, int ne, double& t, double& tol_out, double fuzz) const;
    int compute_vf(int nv, int nf, double& u, double& v, double& tol_out, double fuzz) const;
    int compute_pe(const Point& p, double tolP, int ne, double& t, double& dist) const;

private:
    struct FaceClass2d {                 // == IntTools_FClass2d
        struct Wire { std::vector<std::array<double,2>> poly; int orient; }; // 1 outer, 0 hole, -1 bad
        std::vector<Wire> wires;
        double umin, umax, vmin, vmax;
        UVTol  band;                     // FlecheU/FlecheV floored by uv_tol_at (§2.4.3)
        bool   any_bad = false;          // forces the exact classifier
        bool   usable  = true;           // false when a pcurve was missing -> UNKNOWN, not IN
        bool   u_periodic = false, v_periodic = false;
        double u_period = 0, v_period = 0;
    };
    const BRep& shape_;
    mutable std::unordered_map<int, FaceClass2d> fclass_;
    mutable std::unordered_map<int, /*AABB*/ std::array<double,6>> box_;
    // Edge/vertex tolerance tree for the solid ON band (§2.5.3).
    struct EVTree { /* BVH over BRepBndLib-equivalent boxes of edges + vertices */ };
    mutable EVTree ev_;
};

} // namespace session_cpp
```

Notes on the declarations:

- `TolStore` holds tolerances in **DS index space**, not on the `BRep` structs. This
  matches the ARCHITECTURE_v2 decision that nothing is mutated and satisfies §2.1.5's
  intent without porting copy-on-write. `BRepVertex`/`BRepEdge`/`BRepFace`
  (`/home/petras/code/code_rust/session/session_cpp/src/brep.h:27-58`) get **no** new
  fields; persisted tolerances (if ever needed) go through a separate serialisation step.
- `vertex_for_acceptance` is deliberately ugly-named. It is the single most easily-lost
  detail of §2.1.4 and the name must make its use conspicuous.
- `State::Unknown` exists so the "missing pcurve ⇒ everything is IN" OCCT failure
  (`IntTools_FClass2d.cxx:157-161` + `:639-643`) cannot happen here.

---

## 4. WHAT OUR CODE DOES TODAY, AND WHERE IT DIVERGES

All paths absolute, under `/home/petras/code/code_rust/session/session_cpp/src/`.

### 4.1 There is no per-entity tolerance at all

`brep.h:27-58` — `BRepVertex`, `BRepEdge`, `BRepTrim`, `BRepLoop`, `BRepFace` carry **no
tolerance field**. Every tolerance in the splitter is a *local variable*, derived on the
spot, and thrown away.

Consequences, each of which G1/G3/G4 exist to prevent:

- There is nothing to raise when a merge absorbs a deviation, so the deviation is
  forgotten and the next stage re-derives a different number.
- The containment invariant `tol(V) ≥ tol(E) ≥ tol(F)` cannot even be *stated*, let alone
  checked.
- There is no provisional/rollback layer, so any growth we do is permanent and
  self-amplifying.

**Divergence D1** — highest severity. Fix: `TolStore` (§3.2), populated in stage 0.

### 4.2 Every splitter tolerance is domain-relative

Confirmed sites (this is the exhaustive list for the splitter proper):

| our site | expression | OCCT equivalent |
|---|---|---|
| `nurbssurface_trimmed.cpp:574` | `samp_tol = max(range_u, range_v) * 2e-5` | a 3D sag tolerance → `uv_tol_at` |
| `nurbssurface_trimmed.cpp:1653` | `samp_tol = max(range_u, range_v) * 1e-3` | same |
| `nurbssurface_trimmed.cpp:569` | `snap_uv = min(range_u, range_v) * 1e-7` (the `tolerance == 0` branch) | `uv_tol_at(S,u,v,CONFUSION)` |
| `nurbssurface_trimmed.cpp:1645` | `snap_uv = min(range_u, range_v) * 1e-7` | same |
| `nurbssurface_trimmed.cpp:828` | `bweld = max(snap_uv*8, min(range)*5e-4)` | 3D weld radius |
| `nurbssurface_trimmed.cpp:848` | `min_ext = max(snap_uv*8, min(range)*1e-5)` | 3D minimum extension |
| `nurbssurface_trimmed.cpp:1425` | `max(snap_uv*4, min(range)*1.5e-3)` | join tolerance |
| `nurbssurface_trimmed.cpp:1650` | `seam_tol = max(snap_uv*8, min(range)*1e-6)` | seam separation |
| `brep.cpp:4262` | `ov = min_rangeF * 1e-2` | scaffold overshoot |
| `brep.cpp:4280-4281` | `scaf_forced_eps = clamp(min_rangeF*1e-2 … min_rangeF*1.3e-1)` | scaffold forced node |
| `brep.cpp:4297` | `sag_tol = min_rangeF * 5e-4` | sag |
| `brep.cpp:4350` | `eps_border = min(duF.range, dvF.range) * 2e-3` | on-border test |
| `brep.cpp:11373`, `brep.cpp:11574` | `tol = max(u1-u0, v1-v0) * 0.01` | |
| `brep_section.cpp:221` | `samp_tol_lp = max(1e-9, max(bx1-bx0, by1-by0) * 2e-4)` | |
| `brep_section.cpp:1448-1449`, `:1889-1890`, `:2016-2017` | `epsA/epsB = min(du.range, dv.range) * 1e-3` | |

**Divergence D2.** A STEP round trip that returns a box face on `u ∈ [-0.04, 4.04]` instead
of `u ∈ [0,1]` multiplies `range_u` by 4.08 and therefore multiplies *every one of these*
by 4.08 (or, for the `min` ones, by whatever the padded minimum becomes). The measured
symptom is in the mission brief: splitting one operand alone on a padded domain yields
**32 naked edges of 36** while the UV arrangement is verified identical. OCCT has no
analogue of any of these numbers; the closest thing it has —
`Geom_BSplineSurface::Resolution` — is *constructed to be invariant* under exactly this
reparameterisation (§2.3.2).

Fix: every row becomes `uv_tol_at(S, u, v, <the 3D tolerance this actually means>)`.
Deriving "the 3D tolerance this actually means" per row is the M0 work item and must be
done by reading each use, not by a blanket substitution.

### 4.3 We do have a metric estimate, and it is wrong in four ways

`nurbssurface_trimmed.cpp:548-572` (and the duplicate at `:1633-1647`):

```cpp
int nu = max(spans_u.size()-1, 1) * 4;      // :552
double du = range_u / nu;                    // :554
double mu = (u0+u1)*0.5, mv = (v0+v1)*0.5;   // :556-557  <-- domain CENTRE
Point pmid = srf.point_at(mu, mv);
double uv_to_3d_u = pmid.distance(srf.point_at(min(mu+du,u1), mv)) / du;
double uv_to_3d_v = pmid.distance(srf.point_at(mu, min(mv+dv,v1))) / dv;
double uv_to_3d   = max(uv_to_3d_u, uv_to_3d_v);   // :561  <-- ONE scalar
if (uv_to_3d < 1e-10) uv_to_3d = 1.0;              // :562-563
snap_uv = (tolerance > 0) ? max(1e-9, tolerance/uv_to_3d)
                          : min(range_u, range_v) * 1e-7;   // :567-569
```

**Divergence D3**, four distinct defects:

1. **Evaluated once, at the domain centre** — `(u0+u1)*0.5`. That is both a symmetric probe
   (G7 violation: on a sphere the centre of the natural domain is on the equator/seam
   crossing) and a single-point estimate used across the whole face. A cone's `|∂S/∂u|`
   varies from `0` at the apex to `R` at the base; one centre sample is wrong everywhere
   else.
2. **Collapsed to one scalar via `max`** (`:561`). For a cylinder of radius 10 the true
   ratio between the U and V metrics is 10; taking the max makes the V tolerance 10× too
   tight and, since it is used as a *snap* radius, silently refuses to weld points that
   should weld. G5 violation.
3. **Finite-differenced over a span-derived step** rather than evaluating `D1`. The step
   `du = range_u/(4·nspans)` is itself domain-relative, so on a padded domain the finite
   difference is taken over a different arc — the estimate is *not* padding-invariant
   either.
4. **The fallback is domain-relative** (`:569`, `:1645`). When `tolerance == 0` — which is
   the default of every public entry point (`brep.h:220-240`) — the metric is bypassed
   entirely and we are back to D2.

Fix: replace both copies with `uv_tol_at` (§3.3), which uses the analytic closed forms for
recognised surfaces and the control-net bound otherwise, and evaluate it at the point of
use.

### 4.4 Default-tolerance resolution is bbox-relative, in five different flavours

`brep.cpp:5049` `tol = diag*1e-6`; `:5399`, `:6766`, `:7007`, `:7092` `tol = diag*5e-3`;
`:6316` `diag*3e-3`; `:7582` `diag*1e-6`; `:7988` `diag*1e-6`;
`nurbssurface_trimmed.cpp:2411`, `:2546` `weld_tol = bbox_diag*1e-5`.

**Divergence D4.** Model-scale-relative is *far* better than domain-relative — it satisfies
G1 and G2 — but the spread from `1e-6·diag` to `5e-3·diag` is four orders of magnitude, and
none of the five is documented as "the tolerance of X". OCCT has exactly one model-relative
tolerance in the entire codebase (`BRepLib.cxx:1814`, and only in the optional
face-minimum pass); everything else is absolute or entity-derived.

Fix: one resolution function. `tol3d = (user > 0) ? user : max(CONFUSION, diag * 1e-7)`,
and every consumer then adds the entities' own tolerances per §2.1.7 instead of inventing
a scale. Where a *sag* (tessellation deflection) is wanted, name it `sag`, keep it
model-relative, and never route it through the same variable as a coincidence tolerance.

### 4.5 Symmetric probes: fixed, but off by default

`brep.cpp:9159-9231` (`face_sample`) contains a correct implementation of the doctrine —
`PAR_T = 0.43213918`, a second constant `PAR_T2 = 0.61803399` so that a square face's
*diagonal* symmetry is also broken, and asymmetric barycentric weights
`(0.43213918, 0.34589803, 0.22196279)` — with a comment that names the OCCT source. But:

```cpp
static const bool s_probe_fix = (std::getenv("SESSION_PROBE_FIX") != nullptr);  // brep.cpp:9188
double fu = s_probe_fix ? PAR_T  : 0.5;                                          // :9192
double w0 = s_probe_fix ? 0.43213918 : (1.0/3.0);                                // :9225
```

**It is gated off by default.** And the *other* interior-point routine,
`face_interior` at `brep.cpp:1576-1655`, has no such fix at all:

```cpp
double cu = 0.5*(u0+u1), cv = 0.5*(v0+v1);                       // brep.cpp:1601
...
double tcu = (a[0]+b[0]+c[0])/3.0, tcv = (a[1]+b[1]+c[1])/3.0;   // :1637  equal-weight centroid
...
double su = u0 + (u1-u0)*iu/12.0, sv = v0 + (v1-v0)*iv/12.0;     // :1648  12x12 grid, includes 0.5
```

`face_interior` feeds the *outward-sign* determination that `volume()` and the shell
orientation propagation depend on. `is_planar` at `brep.cpp:2189-2196` probes at
`0.5` and then `{0.25,0.5,0.75}×{0.3,0.6,0.8}` — the `0.5` again.

**Divergence D5.** Fix: delete the env gate, make the asymmetric path unconditional,
apply it to `face_interior`, `is_planar` and the `12×12` grid (use a prime divisor as OCCT
does — `37` at `BRepClass3d_SolidExplorer.cxx:369-370`), and add the G7 audit assertion.

### 4.6 Point-in-face uses sampled polygons with no band and no exact fallback

`brep.cpp:1273-1305` (`contains_point_exact`'s `uv_in_trims` lambda) and
`nurbssurface_trimmed.cpp:2330-2334` (`inside_trim`) and
`nurbssurface_trimmed.cpp:1242-1251` / `:1998-2007` (`point_in_cycle`):

- The trim loops are sampled into polygons at `min(max(cv_count*2,16),128)` points
  (`brep.cpp:1288`) or `min(max(cv_count*3,6),1024)` (`brep.cpp:1592`, `:9172`) — a *count*,
  not a deflection, so the polygon's fidelity is unknown and varies per call site.
- The crossing test is a bare even-odd ray cast with a `+1e-30` division guard
  (`brep.cpp:1300-1303`): **no ON state, no tolerance band, no exact-pcurve fallback**.
- `point_in_cycle` (`nurbssurface_trimmed.cpp:1242-1251`) has no guard at all.
- Face/hole areas are thresholded against `snap_uv*snap_uv`
  (`nurbssurface_trimmed.cpp:1258`, `:1268-1270`, `:2013-2015`) — a **UV** area compared
  against a **UV** tolerance derived from a 3D one, which is dimensionally right but
  inherits D3's single-scalar error.

**Divergence D6.** Missing, relative to §2.4: the four-corner band (`CSLib_Class2d.cxx:174-184`),
the vertex/edge ON detection (`:293-316`), the adaptive re-discretisation driven by
area/perimeter (`IntTools_FClass2d.cxx:454-534`), the exact pcurve classifier fallback
(`TopClass_FaceClassifier.pxx:31-140`), the periodic retry ladder
(`IntTools_FClass2d.cxx:758-802`), and the `UNKNOWN` outcome.

### 4.7 Point-in-solid: two implementations, both unlike OCCT's

`brep.cpp:1238-1267` — `contains_point(mesh, p)`: **generalised winding number** over the
tessellation; `|Ω| > 2π` ⇒ inside; any vertex within `1e-15` ⇒ "ON the boundary" and
returns `true` (`:1259`).

`brep.cpp:1273-1400` — `contains_point_exact(p, osign)`: closest point over all faces
(`Closest::surface_point`), in-trim test, boundary fallback for off-trim faces
(`closest_on_trim`, `:1311-1334`, sampling the pcurve at `min(max(cv_count,8),24)` points),
then the sign of `(p - bpt)·n·s`. Two guard tiers exist but are **env-gated off**:
`s_pip_guard = getenv("SESSION_PIP_GUARD")` (`brep.cpp:1345`), which is the only thing that
handles "the nearest point is on a trim boundary, where the face normal is not the
separating direction" — a defect the comment itself measures at `-1.9e-2` on stacked unit
cylinders (`brep.cpp:1338-1344`).

`brep.cpp:1906-1913` — `parity_in`: 3 fixed skew rays against the self-tessellation,
`Intersection::ray_mesh(ray, bmesh, 1e-9, true)`, majority of 3.

**Divergence D7**, itemised against §2.5:

1. **No tolerance-band ON test.** OCCT's *first* action is to test the point against every
   edge's and vertex's own tolerance via a spatial tree
   (`BRepClass3d_SClassifier.cxx:219-230`, `BRepClass3d_BndBoxTree.cxx:25-71`) and return
   `ON` before doing any ray work. We have no `ON` state at all — every predicate returns
   `bool`. This is the direct cause of "a fragment lying exactly on the other solid's
   surface answers IN on one probe and OUT on the next".
2. **The ray is arbitrary, not chosen.** OCCT picks the ray that hits some face as
   perpendicularly as possible (`maxscal > 0.2`,
   `BRepClass3d_SolidExplorer.cxx:670-691`) and retries with an 8-step asymmetric ladder
   when it cannot. We use 3 hard-coded directions and vote.
3. **No faulty-line detection.** OCCT restarts the whole classification when the ray hits a
   face boundary (`aState == TopAbs_ON ⇒ isFaultyLine = true`,
   `BRepClass3d_SClassifier.cxx:477-481`) or passes closer to an edge/vertex than to the
   nearest face hit (`:511-515`). We have no equivalent, so a grazing ray silently returns
   a wrong parity.
4. **Tangency is not excluded.** OCCT ignores intersection points whose transition is
   `Tangent` (`:463-470`) and marks parallel edge/line extrema invalid
   (`BRepClass3d_BndBoxTree.cxx:97-101`). We count every mesh hit.
5. **Classification is against a tessellation** in two of the three paths, so its accuracy
   is the tessellation's, not the geometry's — the exact failure mode the
   `contains_point_exact` comment documents for rotated chairs (`brep.cpp:9326-9331`).
6. **Guards are env-gated off**, so the shipped default is the known-bad path.

### 4.8 `volume()` cannot be used as an acceptance oracle in its current state

`brep.cpp:2172-2260+`. The mission reports it wrong by exact constant factors (cylinder and
cone `1/3` of truth, torus `3×`, sphere `0` with `is_solid = 0`) and non-terminating on
NURBS solids. Reading the code, the structural reasons are visible: the curved-face branch
integrates `S·(S_u×S_v)` over the **UV bounding box of the trim** ("exact for
axis-aligned/rectangular trims", `brep.cpp:2181-2182`) — i.e. it ignores the trim for
non-rectangular regions — and the planar branch relies on a greedy nearest-endpoint chaining
of pcurves (`:2286-2301`) that can mis-order a loop.

**Divergence D8.** This is not my subsystem to fix, but §5 depends on it: **the acceptance
harness must not use `volume()` until it is independently validated.** §5.0 specifies what
it must use instead.

---

## 5. ACCEPTANCE TESTS

Every test below is oracle-free or analytically exact. No test compares against OCCT
output, and no test uses a stored reference file.

### 5.0 Harness prerequisite: a trustworthy volume

The invariant tests need `vol(·)`. Until `volume()` (§4.8) is validated, the harness uses
`measure_volume_ref(B)`: divergence-theorem flux over a tessellation refined until two
successive refinements agree to `1e-9` relative, **plus** a cross-check against
`measure_volume_ref` of the same solid under a random rigid motion (must agree to `1e-12`
relative — rigid motion cannot change a volume, and a bug that survives that is not a
tessellation bug). Analytic values are used wherever the operand is a primitive.

### 5.1 Metric conversion (G5) — closed-form, no geometry pipeline

| # | operand | assertion |
|---|---|---|
| T1.1 | plane | `u_resolution(P, r) == r` and `v_resolution(P, r) == r` exactly, for `r ∈ {1e-9, 1e-3, 1}` |
| T1.2 | cylinder `R = 10` | `u_resolution(C, 1e-6) == 2·asin(1e-6/20)` to 1 ulp; `v_resolution(C, 1e-6) == 1e-6` exactly. Ratio ≈ `20` |
| T1.3 | sphere `R = 1` | `u_resolution == v_resolution == 2·asin(r/2)`; at `r = 1.9`, assert the value differs from the linearised `r/1` by `> 10%` (proves the exact form is used) |
| T1.4 | sphere `R = 1`, `r = 3` | `Res = 1.5 > 1` ⇒ result is exactly `2π` (the clamp, `GeomAdaptor_Surface.cxx:1895`) |
| T1.5 | torus `Rmaj=3, Rmin=1` | `u_resolution(r) = 2·asin(r/8)`, `v_resolution(r) = 2·asin(r/2)` |
| T1.6 | cone, base radius `5`, apex radius `0` | `u_resolution(r) = r/5`; `v_resolution(r) = r` |
| T1.7 | NURBS surface `S`, and `S'` = `S` with knots affinely mapped `u → 4u - 0.04` | `u_resolution(S,r)·4 == u_resolution(S',r)` to `1e-12` relative. **This is the padding-invariance proof at the metric level.** |
| T1.8 | any surface, pole (`\|S_u\| → 0`) | `uv_tol_at` returns the finite global bound, never `inf` or `NaN` |

### 5.2 Domain-padding invariance (G2) — the defect that motivated this document

| # | setup | assertion |
|---|---|---|
| T2.1 | unit box, face `f` on `u,v ∈ [0,1]`; `f'` = same face with the surface reparameterised to `u,v ∈ [-0.04, 4.04]` and pcurves mapped | `split_by_surface(f, cutter)` and `split_by_surface(f', cutter)` produce equal face counts, equal edge counts, **naked = 0 in both** |
| T2.2 | same, the full mission scenario: split ONE operand alone on the padded domain | naked edges `== 0` (measured today: **32 of 36**) |
| T2.3 | STEP round trip of a unit box → boolean cut with a rotated box | 6-face cell, naked `== 0` (ARCHITECTURE_v2 gate M0: "6 faces, naked 0 (currently 2 faces, 8 naked)") |
| T2.4 | sphere with domain `v ∈ [-π/2, π/2]` vs `v ∈ [-π/2+0, π/2]` scaled by 3 | identical classification of 1000 random points |
| T2.5 | grep-style static test | zero occurrences of `range_u`, `range_v`, `(u1-u0)`, `(v1-v0)` multiplied by a literal, outside `brep_tol.cpp` |

### 5.3 Tolerance-model invariants (G3, G4)

| # | setup | assertion |
|---|---|---|
| T3.1 | any assembled result | for every incidence `V∈E∈F`: `Tol(V) >= Tol(E) >= Tol(F)` — run after harmonisation, must hold with zero exceptions |
| T3.2 | provisional layer, synthetic | `scope_begin(); grow_provisional(v, 1e-3); assert(vertex_for_acceptance(v) == old); scope_rollback(); assert(vertex(v) == old)` |
| T3.3 | same, with commit | `scope_begin(); grow_provisional(v,1e-3); commit(v); scope_rollback(); assert(vertex(v) == 1e-3)` |
| T3.4 | rollback box rebuild | after `scope_rollback`, the vertex's DS box must have **shrunk** back — assert `box_diagonal_after < box_diagonal_during` (catches the union-instead-of-rebuild bug, `BOPAlgo_PaveFiller_6.cxx:1085-1089`) |
| T3.5 | effective floor | `set_vertex(v, 1e-12); assert(vertex(v) == 1e-7)` (`BRep_Tool.cxx:1314-1330`) |
| T3.6 | fuzzy additivity (G10) | run a full boolean with `fuzz = CONFUSION` and with the fuzzy path disabled; results must be **bit-identical** |
| T3.7 | fuzzy monotonicity | `fuzz1 < fuzz2` ⇒ the interference set at `fuzz1` is a subset of that at `fuzz2` |
| T3.8 | absolute reject | a vertex needing `0.06` of growth must leave the shape *reported invalid*, not silently grown (`aMaxTol = 0.05`, `BOPAlgo_Builder.cxx:472`) |

### 5.4 Point-in-face on CURVED trimmed faces (G6)

Operands with analytically known answers:

| # | face | probe | expected |
|---|---|---|---|
| T4.1 | sphere `R=1`, trimmed to the octant `u∈[0,π/2], v∈[0,π/2]` | `S(π/4, π/4)` | `IN` |
| T4.2 | same | `S(0, π/4)` (exactly on the `u=0` trim) | `ON` |
| T4.3 | same | `S(-1e-4, π/4)` | `OUT` |
| T4.4 | same | `S(-δ, π/4)` where `δ = uv_tol_at(...).du/2` | `ON` (the band, `CSLib_Class2d.cxx:174-184`) |
| T4.5 | cylinder `R=10`, full `u∈[0,2π]`, `v∈[0,5]`, with a circular hole of radius 1 centred at `(π, 2.5)` in 3D-arc-length terms | probe at the hole centre → `OUT`; at 1.5 arc-length from the centre → `IN`; **the hole must be circular in 3D, hence elliptical in UV with axis ratio 10** — this test fails outright with a single-scalar metric (§4.3 defect 2) |
| T4.6 | torus `Rmaj=3,Rmin=1`, trimmed across the `u=0` seam (`u∈[3π/2, 2π]∪[0, π/2]` expressed as `u∈[3π/2, 5π/2]`) | probes at `u = 0` and `u = 2π` must both classify `IN` (periodic retry ladder, `IntTools_FClass2d.cxx:758-802`) |
| T4.7 | cone, trimmed to a band away from the apex | probe near the apex-side trim; assert no `NaN`, verdict stable under `uv_tol_at`'s pole clamp |
| T4.8 | any face with one pcurve deliberately removed | verdict is `UNKNOWN`, **never** `IN` (our fix to `IntTools_FClass2d.cxx:157-161`) |
| T4.9 | ON/OUT convention | `is_point_in_face` (ON→false) and `is_in_or_on_face` (ON→true) disagree on exactly the T4.2 probe and agree everywhere else |
| T4.10 | equivariance (G8) | apply 100 random rigid motions to T4.1–T4.7; every verdict unchanged |

### 5.5 Point-in-solid (G6, G8, G9)

| # | solid | probe | expected |
|---|---|---|---|
| T5.1 | sphere `R=1` at origin | `(1-1e-9, 0, 0)` | `IN` (`1e-9 < CONFUSION`? no — `1e-9 < 1e-7`, so this is inside the ON band ⇒ **`ON`**; assert `ON`) |
| T5.2 | same | `(0.99, 0, 0)` | `IN` |
| T5.3 | same | `(1.01, 0, 0)` | `OUT` |
| T5.4 | same | exactly `(1,0,0)` | `ON` |
| T5.5 | unit box centred at origin | the 8 corners, the 12 edge midpoints, the 6 face centres | all `ON`. **This is the symmetric-probe torture test**: every one of those 26 points is on a symmetry plane |
| T5.6 | torus `Rmaj=3,Rmin=1` | `(0,0,0)` (inside the hole) | `OUT` |
| T5.7 | same | `(3,0,0)` (the core circle) | `IN` |
| T5.8 | hollow sphere (shell `R=2` with inner void `R=1`) | `(0,0,0)` | `OUT` (cavity handling) |
| T5.9 | determinism (G9) | shuffle the face array 20×, re-run T5.1–T5.8 | identical verdicts |
| T5.10 | equivariance (G8) | 100 random rigid motions | identical verdicts for all probes at distance `> 10·Tol` from the boundary |
| T5.11 | grazing ray | cylinder `R=1`, probe at `(0,0,0)` with a ray direction exactly along the axis (forced) | must detect the faulty line and retry, not return a parity from a tangential hit |

### 5.6 The oracle-free result invariants (G11)

Run on every cell of the 224-cell primitive-pair sweep, at every pose.

**I1 — Closure.** `naked_edge_count(R) == 0` **and** `nonmanifold_edge_count(R) == 0` **and**
every shell of `R` is closed and consistently oriented. No tolerance involved; pure
combinatorics on shared entities.

**I2 — Per-face validity.** For every face of `R`: its outer loop is closed in UV within
`uv_tol_at`; loop orientation is positive-area for outer, negative for inner; every trim's
3D curve and its pcurve agree to within `Tol(edge)` at 23 samples
(`BRepLib::UpdateInnerTolerances`, `BRepLib.cxx:2028`); no face has zero area.

**I3 — Partition identity.**
`vol(A∖B) + vol(A∩B) + vol(B∖A) == vol(A∪B)` and `vol(A) + vol(B) == vol(A∪B) + vol(A∩B)`,
each to `1e-12` relative. Requires §5.0's `measure_volume_ref`. Reported today at `1e-15`
residual for the 15 genuine box×box passes — that is the bar for curved pairs too.

**I4 — Rigid-motion equivariance.**
`vol(M·A op M·B) == vol(A op B)` to `1e-12` relative, and the face/edge/vertex counts are
equal, for 20 random `(axis, angle)` per cell. This is the invariant the whole campaign is
about: it is what "random axis, random angle" measures, with no oracle.

**I5 — Idempotence.** `A ∪ A == A`, `A ∩ A == A`, `A ∖ A == ∅`, up to entity counts and
volume. Note the maximal-symmetry hazard: `A op A` is precisely where a symmetric probe
lands on a shared face (G7).

**I6 — bbox containment.** `bbox(A∩B) ⊆ bbox(A) ∩ bbox(B)`; `bbox(A∪B) == bbox(A) ∪ bbox(B)`;
`bbox(A∖B) ⊆ bbox(A)`. Each inflated by the result's max tolerance. Cheap, and catches
runaway fragments immediately.

**I7 — Tolerance sanity.** `max over entities of Tol` in the result must be
`<= max(input max Tol, MAX_CORRECT_TOL = 0.05)`. A result whose tolerances exploded is a
failure even if it passes I1.

### 5.7 The cells that currently return zero, restated as tests

Each of these is `naked == 0 && I3 && I4`, over 20 random poses:

- `sphere × sphere` — today 0/20, cut produces 2 faces / 2 naked.
- `box × sphere` — today 0/20, 20 open shells.
- `box × cone`, `cone × cone`, `cyl × cone`, `sphere × cone` — today 0.
- `box × torus`, `torus × torus` — today **hang**; add a hard wall-clock budget per cell
  and record a timeout as a distinct failure class (G12: typed failure).
- `cyl × cyl`, `box × cyl` — today "pass" only on non-intersecting operands; the test must
  assert `intersection_is_nonempty` first, so a trivial pass cannot be recorded.

---

## 6. IMPLEMENTATION ORDER — smallest shippable increment first

Each increment is independently revertable, independently measured, and lands only with the
existing guards battery green (base 3 ops exact, matrix 45/45, edge 54/54, C++ minitests,
A-op-A).

**S1 — `brep_tol.{h,cpp}`: constants + metric. No behaviour change.**
Ship `prec::` constants, `u_resolution` / `v_resolution` (analytic arms + `BSplCLib`
control-net bound), `uv_tol_at`, `uv_to_3d`, `curve_resolution`. Nothing calls them yet.
*Gate*: T1.1–T1.8 green. Zero diff in any existing test.

**S2 — Replace the two `uv_to_3d` estimators.**
`nurbssurface_trimmed.cpp:548-572` and `:1633-1647` call `uv_tol_at` instead of the
centre-sampled finite difference; `snap_uv` becomes per-direction `UVTol`. Keep the
`tolerance == 0` fallback wired to `max(CONFUSION, diag*1e-7)` rather than
`min(range)*1e-7`.
*Gate*: guards battery unchanged; T2.4 green.

**S3 — Retire the domain-relative constants, one call site at a time.**
Work the §4.2 table top to bottom. For each row: read the use, decide what 3D quantity it
means (coincidence radius / sag / seam separation / border band), replace with
`uv_tol_at(S,u,v,that)`. Do **not** batch — each row is a separate commit with the guards
battery.
*Gate*: T2.1, T2.2, T2.3 green — this is ARCHITECTURE_v2's M0 milestone ("STEP round-trip
box cell: 6 faces, naked 0").

**S4 — Un-gate and generalise the asymmetric probes.**
Delete `SESSION_PROBE_FIX` (`brep.cpp:9188`); make `PAR_T`/`PAR_T2`/asymmetric barycentrics
unconditional; apply the same to `face_interior` (`brep.cpp:1601`, `:1637`, `:1648`) and
`is_planar` (`brep.cpp:2189-2196`); change the `12×12` grid to a prime divisor. Add the G7
audit assertion in debug builds.
*Gate*: T5.5 (26 symmetric probes on a centred box) green; I5 (A-op-A) green.

**S5 — `TolStore`, populated but read-only.**
Add per-entity tolerances to the DS, initialise from the input (or `CONFUSION`), implement
the harmonisation pass `(b)` of §2.1.6, and run **I-assertions only** — no predicate reads
it yet.
*Gate*: T3.1, T3.5 green; no behaviour change.

**S6 — Three-state face classification.**
Implement `FaceClass2d` with the four-corner band, vertex/edge ON detection, the
area/perimeter adaptive re-discretisation, and the `UNKNOWN` outcome. Route
`contains_point_exact`'s `uv_in_trims` (`brep.cpp:1275-1305`) and
`nurbssurface_trimmed.cpp:2330-2334` through it. Add the exact-pcurve fallback last (it is
only reached on `ON`).
*Gate*: T4.1–T4.10 green.

**S7 — Three-state solid classification.**
Implement the edge/vertex tolerance BVH and the ON pre-test; then the perpendicularity-scored
ray with the asymmetric retry ladder and faulty-line detection. Keep the winding-number path
as a *tie-breaker of last resort*, reached only when the ray ladder exhausts, and log it.
Un-gate `SESSION_PIP_GUARD` (`brep.cpp:1345`) as part of this — its Tier-2 fallback becomes
the `ON` verdict instead of a parity guess.
*Gate*: T5.1–T5.11 green; the classifier's oracle comparison (currently 0/42 wrong)
non-decreasing.

**S8 — Provisional/rollback layer.**
`scope_begin` / `grow_provisional` / `vertex_for_acceptance` / `commit` / `scope_rollback`,
wired into the FF stage, with the box **rebuild** on rollback.
*Gate*: T3.2–T3.4 green; tolerance-explosion invariant I7 green on the full corpus.

**S9 — Fuzzy plumbing.**
Add the fuzzy term at every site of the §2.1.7 table, with OCCT's exact half/whole
weighting. Default `CONFUSION`.
*Gate*: T3.6 (bit-identical at default), T3.7 (monotone) green.

**S10 — The acceptance harness.**
`measure_volume_ref` (§5.0), then I1–I7 as a runnable battery over the 224-cell sweep with a
per-cell wall-clock budget and typed failure classes.
*Gate*: the battery runs to completion on every cell — including recording the torus hangs
as `TIMEOUT` rather than hanging the run. This is the instrument every later increment is
measured with, so it must exist before the curved-pair work is judged.

---

## 7. WHERE OCCT ITSELF GIVES UP — documented fallbacks, not inventions

Port these as *typed* outcomes; do not paper over them.

1. **`Precision::Parametric(P) = P*0.01`** (`Precision.hxx:328`) — the "assume the
   derivative is 100" fallback, used by `GeomAdaptor_Surface::UResolution/VResolution` for
   `SurfaceOfRevolution` in U, and for every unrecognised surface type
   (`GeomAdaptor_Surface.cxx:1886-1888`, `:1948-1950`). It is a guess. Log it.
2. **Cone with an unbounded V range** falls back to the same guess
   (`GeomAdaptor_Surface.cxx:1856-1860`).
3. **`IntTools_FClass2d` with a missing pcurve** aborts `Init` and then classifies
   *everything* as `IN` (`IntTools_FClass2d.cxx:157-161` + `:639-643`). OCCT does not
   detect this. We return `UNKNOWN`.
4. **`BRepClass3d_SClassifier::State()` maps every unresolved state to `OUT`**
   (`BRepClass3d_SClassifier.cxx:540-541`, with the source comment "return OUT state when
   there is an error during execution"). A rejected or faulty classification is
   indistinguishable from a computed OUT at the API boundary.
5. **`OtherSegment` gives up with a `+X` ray** when the parameter ladder falls below
   `0.0001` (`BRepClass3d_SolidExplorer.cxx:710-720`, `:774-783`) — an arbitrary direction,
   accepted as a last resort.
6. **`FindAPointInTheFace` returns `false`** when the resulting UV does not classify `IN`
   (`BRepClass3d_SolidExplorer.cxx:166-169`) — no repair is attempted.
7. **`BRepLib_ValidateEdge` aborts with a partial distance** when a single `Extrema_LocateExtPC`
   fails to converge (`BRepLib_ValidateEdge.cxx:201-206`, `:221-226`), and
   `SetExitIfToleranceExceeded` makes `myCalculatedDistance` a lower bound
   (`:81-85`, `:134-138`). Both leave the reported tolerance too small, silently.
8. **`ComputeTol` returns `Precision::Infinite()`** for an infinite `Pcons`
   (`BRepLib.cxx:1123-1128`), which `SameParameter` turns into an abort at `> 1e10`
   (`BRepLib.cxx:1398-1402` `[audit]`) — the edge keeps whatever tolerance it had.
9. **`CorrectTolerances` refuses to grow past `0.05`** (`BOPAlgo_Builder.cxx:472`,
   `BOPTools_AlgoTools_1.cxx:481`, `:507`): beyond that the shape is simply left invalid.
   That is an *intentional* refusal and the right behaviour — port it, and surface it.
10. **`BuilderSolid` ignores the fuzzy value entirely** (`[audit]` §2.9): solid assembly is
    fuzzy-blind, and `PostTreatFF`'s nested PaveFiller never receives one
    (`BOPAlgo_PaveFiller_6.cxx:1196-1197`). Under a large user fuzzy, OCCT's own pipeline is
    internally inconsistent.
11. **`BRepClass3d_SClassifier::PerformInfinitePoint` is randomised**
    (`math_BullardGenerator`, `BRepClass3d_SClassifier.cxx:109`, `:134`) and bounded at 10
    tries per face. It can fail to decide, leaving `myState = 2` (ON) from its initialiser
    (`:111`). Our port must seed deterministically (G9) and report failure.

---

## 8. THE THREE THINGS THAT MATTER MOST

If only three items from this document are implemented, make them these:

1. **`uv_tol_at`, and the deletion of every domain-relative constant** (§2.3, §4.2, §4.3;
   increments S1–S3). It is the difference between a kernel that survives a STEP round trip
   and one that does not, and it costs no algorithmic change anywhere else.
2. **A three-state classifier with a real ON band** — the four-corner band for faces
   (`CSLib_Class2d.cxx:174-184`) and the edge/vertex tolerance tree for solids
   (`BRepClass3d_BndBoxTree.cxx:25-71`) — replacing every `bool` predicate (§2.4, §2.5, §4.6,
   §4.7; increments S6–S7). Curved booleans fail today largely because "on the boundary" is
   not representable.
3. **Asymmetric probes, unconditionally** (§2.6, §4.5; increment S4). Our corpus is
   maximally symmetric and the fix is already written — it is just switched off.
