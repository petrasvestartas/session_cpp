# Analytic SSI Coverage Matrix — our kernel vs OCCT, per quadric pair × relative configuration

Status: research + specification. READ-ONLY study of `session_cpp/src/**` and
`/home/petras/code/code_cpp/OCCT/src/**`. No code was changed by this document.

Scope note (title changed from "canonicalisation"): the original brief hypothesised that our
analytic recognisers test for *world* axis-alignment and fall back to marching when an operand is
rotated. **That hypothesis is refuted** (§0). The real axis is **relative pose**, and the
deliverable is the coverage matrix of §3–§4.

OCCT paths cited below are relative to `/home/petras/code/code_cpp/OCCT/src/`:

- `ModelingData/TKGeomBase/IntAna/IntAna_QuadQuadGeo.cxx`      → **QQG**
- `ModelingData/TKGeomBase/IntAna/IntAna_IntQuadQuad.cxx`      → **IQQ**
- `ModelingData/TKGeomBase/IntAna/IntAna_Quadric.cxx`          → **QUAD**
- `ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.cxx` → **III**
- `ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Intersection.cxx`       → **IPI**

Ours are relative to `/home/petras/code/code_rust/session/session_cpp/`.

---

## 0. The refuted hypothesis, and what replaced it

### 0.1 Refuted: "recognisers test for axis alignment"

Evidence, three independent lines:

1. **No such predicate exists.** An exhaustive sweep of `src/intersection.cpp` (6821 lines),
   `src/brep_section.cpp` (2608), `src/brep.cpp` (12106) found **zero** comparisons of a surface
   axis against a world unit vector, and zero `dir.z > 1 - tol`-style tests, in the SSI decision
   path. Every recogniser derives its frame from sampled geometry:
   - `fit_cylinder` — `src/intersection.cpp:2381-2440`, axis = smallest-eigenvalue eigenvector of
     the normal covariance (`jacobi_eig3`, 2400-2407)
   - `fit_cone` — `src/intersection.cpp:2442-2514`, apex from a normal-plane least-squares solve
     (2462-2474), axis = largest-eigenvalue eigenvector of the generator covariance (2483-2497)
   - `fit_torus` — `src/intersection.cpp:2555-2615`, axis = smallest-variance eigenvector (2569-2582)
   - `fit_sphere` — `src/intersection.cpp:2516-2550`, 4×4 least squares, no axis at all
   - `recognize_surface` — `src/intersection.cpp:2617-2662`

   The only component-vs-component test anywhere near the path is `ortho_basis`
   (`src/intersection.cpp:2281-2294`), which picks the smallest-magnitude world component of `n`
   as a seed for a perpendicular. That is a *tie-breaker*, not an alignment gate; its only
   consequence is that the emitted circle's CV phase rotates with the operand, which is
   geometrically exact. OCCT does the identical thing in `DirToAx2` (**QQG:237-257**).

2. **It is structurally impossible.** Under a *shared* rigid motion of both operands, every
   relative angle and distance is invariant, so a relative-pose predicate cannot flip. Session A's
   equivariance sweep confirms it numerically: `box × box2` 8.77e-15, `cyl × cyl2` 8.30e-15 at 45°,
   `sph × cyl` 5.07e-14.

3. **The recogniser demonstrably keeps firing** in the broken `box × tor` cells
   (`[REC] A kind=1 B kind=4` at every angle including the failing ones).

### 0.2 What is actually true

Our kernel has **no canonicalisation at all** — confirmed by inclusion analysis: neither
`src/intersection.cpp` nor `src/brep_section.cpp` includes `xform.h`, and no world→local
transform of operand B into operand A's frame exists anywhere in the SSI path. Every `ssi_*`
handler works directly in world coordinates with dot/cross products against the stored
`RecogSurface::p1/p2` (`src/intersection.cpp:2373-2379`).

This is *not itself* the bug — OCCT's `IntAna_QuadQuadGeo` is likewise written in coordinate-free
vector algebra and is pose-invariant by construction. It matters only for the **second tier**
(§2), which we do not have at all.

The real defect has **two distinct causes**, and it is important that session A treats them
separately because they need different fixes:

| Cause | What breaks | Where | Rank |
|---|---|---|---|
| **(A) Missing 3D branches** — relative configurations for which OCCT has a closed form and we have none, so the pair drops to the generic marcher | the 3D curve itself is approximate | §3–§4 | see §4 |
| **(B) Pcurve degradation** — the 3D curve is *exact* but `analytic_pcurve` declines and a **fitted cubic** replaces the exact rational pcurve | 3D exact, UV approximate; every downstream trim/mask/flux inherits the fit error | §5 | **highest** |

**Cause (B) explains both measured 9.4e-05 cells directly** and should be attacked first; see §5.

---

## 1. OCCT's three-tier architecture (the shape our kernel should take)

Dispatcher: **III:2517-2769** (`IntPatch_ImpImpIntersection::Perform`). Surface types are encoded
`iT1`, `iT2` ∈ {1=Plane, 2=Cylinder, 3=Cone, 4=Sphere, 5=Torus} by `SetQuad` (**III:2488**), and
the pair key is `iTT = iT1*10 + iT2` (**III:2565**). `bReverse = iT1 > iT2` (**III:2564**) — the
canonical ordering is by type index, exactly as our dispatcher orders its `else if` chain.

```
Tier 1  IntAna_QuadQuadGeo                       exact conic  → IntPatch_GLine
        QQG:389-2666.  Pure vector algebra, pose-invariant.
        Covers only SPECIAL relative configurations (coaxial, parallel,
        equal-radius-and-intersecting, apex-on-surface, ...).
        Returns IntAna_NoGeometricSolution when the configuration is general.

Tier 2  IntAna_IntQuadQuad                       exact quartic → IntPatch_ALine
        IQQ:375 (cylinder × quadric), IQQ:841 (cone × quadric).
        THE GENERAL CASE for any relative pose, provided one operand is a
        cylinder or a cone and the other is a plane/cylinder/cone/sphere.
        This is where canonicalisation lives (§2). NOT available for torus.

Tier 2' cyl × cyl only: ComputationMethods       exact points → IntPatch_WLine
        III:3949-4250 (stCoeffsValue), III:6573 (CyCyNoGeometric),
        III:7881 (IntCyCy).  Closed-form U2(U1), V1(U1,U2), V2(U1,U2);
        each emitted point is exact, only the polyline between them is not.

Tier 3  IntPatch_PrmPrmIntersection (marching)
        IPI:1792-1797 — reached only when Tier 1/2 report not-done.
        For quadrics this happens ONLY for torus pairs outside their
        coaxial/parallel branches.
```

**The single most important structural fact for us:** OCCT never marches a plane/cylinder/cone/
sphere pair. Tier 2 catches every relative pose. Torus is the sole quadric that falls to Tier 3,
and only outside its coaxial branches.

Our kernel has Tier 1 (partially, see §3) and **no Tier 2 or Tier 2' at all**
(`src/intersection.cpp:4141-4232`). Every Tier-1 miss becomes a Tier-3 march.

---

## 2. The canonicalisation primitive (needed only for Tier 2)

Tier 2 is where a transform genuinely happens, and it is worth porting verbatim.

**`IntAna_Quadric`** (**QUAD:28-248**) stores a general implicit quadric

```
f(x,y,z) = CXX·x² + CYY·y² + CZZ·z²
         + 2(CXY·xy + CXZ·xz + CYZ·yz)
         + 2(CX·x + CY·y + CZ·z)
         + CCte
```

Built from `gp_*::Coefficients()`:
- plane   — **QUAD:58-65** (`P.Coefficients(CX,CY,CZ,CCte)` then halve CX,CY,CZ; quadratic part 0)
- cylinder — **QUAD:79-87**; the source form is `gp_Cylinder::Coefficients`
  (`FoundationClasses/TKMath/gp/gp_Cylinder.cxx`), which is literally "X²+Y²−R²=0 in the local
  frame, pushed out through `gp_Trsf T; T.SetTransformation(pos)`":
  `A1=T11²+T21²`, `A2=T12²+T22²`, `A3=T13²+T23²`, `B1=T11T12+T21T22`, `B2=T11T13+T21T23`,
  `B3=T12T13+T22T23`, `C1=T11T14+T21T24`, `C2=T12T14+T22T24`, `C3=T13T14+T23T24`,
  `D=T14²+T24²−R²`
- cone    — **QUAD:97-102**, plus one *special point* appended: the apex,
  `ElSLib::Value(0, -RefRadius/sin(SemiAngle), Cone)`
- sphere  — **QUAD:107-112**, plus two special points: the two poles
  `ElSLib::Value(0, ∓π/2, Sph)`

**`IntAna_Quadric::NewCoefficients(..., const gp_Ax3& Axis)`** — **QUAD:148-248** — rewrites those
ten coefficients in the frame `Axis`. Construction (**QUAD:168-182**):

```cpp
gp_Trsf Trans;
Trans.SetTransformation(Axis);   // world -> Axis frame
Trans.Invert();                  // t_ij = Axis -> world, so x = t11 X + t12 Y + t13 Z + t14
```

then the ten closed-form pushforwards at **QUAD:203-236**. There is no iteration and no sampling;
it is a pure congruence transform of the 4×4 symmetric matrix, `Q' = Mᵀ Q M`, written out
by hand.

**Which frame is chosen** — always **surface 1's own frame**, never a neutral one:

- cylinder × quadric — **IQQ:413**: `Quad.NewCoefficients(..., Cyl.Position())`
- cone × quadric — **IQQ:868-870**: `gp_Ax3 tAx3(Cone.Position()); tAx3.SetLocation(Cone.Apex());`
  then `NewCoefficients(..., tAx3)`. **Note the apex re-location** — the cone frame used for the
  algebra is *not* the cone's stored `Position()`; its origin is moved to the apex so the
  parameterisation `x = z·tanβ·cos t` has no constant term.

**Mapping back** is free: the solution is produced *as a parameterisation of surface 1*
(`(θ, z)` on the cylinder, `(t, z)` on the cone), so the 3D point is recovered with the
surface's own `ElSLib::Value` — no inverse transform is ever applied to a curve.

Also present, same pattern, for the conic × quadric sub-problem:
`IntAna_IntConicQuad.cxx:156` (circle), `:225` (ellipse), `:293` (parabola), `:355` (hyperbola).

### 2.1 The Tier-2 equations, ready to implement

**Cylinder × quadric** (**IQQ:375-826**). After `NewCoefficients(..., Cyl.Position())` the
cylinder is `x = R cos θ, y = R sin θ, z = z`, and `f = 0` becomes a **quadratic in z** whose
coefficients are trigonometric polynomials in θ (**IQQ:426-430**):

```
CoeffZ2(θ) = Qzz
CoeffZ1(θ) = 2( Qz + R(Qxz cos θ + Qyz sin θ) )
CoeffZ0(θ) = Q1 + 2 Qx R cos θ + Qxx R² cos²θ
                + 2 R sin θ (Qy + Qxy R cos θ) + Qyy R² sin²θ
```

Guard: `if (|Qzz| < myEpsilonCoeffPolyNull) { done = false; return; }` (**IQQ:440-444**),
`myEpsilonCoeffPolyNull = 1e-8` (**IQQ:352**). *(This is the degenerate "linear in z" case —
OCCT declines it rather than special-casing it.)*

Discriminant (**IQQ:452-457**), which is what selects the branch topology:

```
DIS(θ) = C_1 + C_SS sin²θ + C_CC cos²θ + 2 C_SC sinθ cosθ + 2 C_S sinθ + 2 C_C cosθ

C_1  = Qz²   − Qzz·Q1
C_SS = R²( Qyz² − Qyy·Qzz )
C_CC = R²( Qxz² − Qxx·Qzz )
C_S  = R ( Qyz·Qz − Qy·Qzz )
C_C  = R ( Qxz·Qz − Qx·Qzz )
C_SC = R²( Qxz·Qyz − Qxy·Qzz )
```

Its roots are found by `TrigonometricRoots(C_CC − C_SS, C_SC, 2C_C, 2C_S, C_1 + C_SS, 0, 2π)`
(**IQQ:460**) — a quartic in `tan(θ/2)`. Branch selection (**IQQ:462-826**):

| root count of DIS | meaning | emitted |
|---|---|---|
| `InfiniteRoots()` | DIS ≡ 0 | 2 curves over `[0, 2π]`, `Z_POSITIF` / `Z_NEGATIF` (**IQQ:468-503**) |
| 0, and `MTF.Value(π) ≥ −RealEpsilon()` | DIS > 0 everywhere | 2 curves over `[0, 2π]` (**IQQ:511-556**) |
| 0, and DIS < 0 | no real intersection | `NbCurves = 0` (**IQQ:557-563**) |
| 1 | tangency at that θ | one tangent point + branches (**IQQ:571+**) |
| 2, 3, 4 | DIS changes sign | one ± pair of curves per positive-DIS θ-interval |

Then `z = ( −CoeffZ1 ± √DIS ) / (2·CoeffZ2)`; each `(θ ↦ z)` branch becomes an `IntAna_Curve`
(`SetCylinderQuadValues`), evaluated **exactly** on demand.

**Cone × quadric** (**IQQ:841-1400**). After `NewCoefficients(..., tAx3-at-apex)`, with
`TgAngle = 1/tan(SemiAngle)` and `x = z tanβ cos t, y = z tanβ sin t` (**IQQ:875-885**):

```
A(t)·z² + B(t)·z + C(t) = 0

A(t):  Z2CC = Qxx,  Z2SS = Qyy,  Z2Cte = Qzz·TgAngle²,
       Z2SC = Qxy,  Z2C  = TgAngle·Qxz,  Z2S = TgAngle·Qyz
```

`A(t) = 0` is solved first via `TrigonometricRoots(Z2CC−Z2SS, Z2SC, 2·Z2C, 2·Z2S, Z2Cte+Z2SS, 0, 2π)`
(**IQQ:900**) to find the t-values where the equation degenerates to linear (those are the
directions along which the intersection runs to infinity / through the apex); the remaining
intervals get the usual `z = (−B ± √(B²−4AC))/(2A)` pair.

---

## 3. THE COVERAGE MATRIX

Legend: **✅** we have a branch · **❌** OCCT analytic, we march · **≡** both march (parity) ·
**⚠** we have a branch but it differs materially from OCCT.

Our dispatcher is `analytic_ssi`, `src/intersection.cpp:4141-4232`; the pair chain is
`4157-4214`; the fall-through to the marcher is `4213` (pair absent) and `4216` (`handled == false`).

---

### 3.1 plane × plane

| Relative configuration | Discriminant | Curve | OCCT | Ours |
|---|---|---|---|---|
| non-parallel | `\|n1 × n2\| > TolAng` | LINE | **QQG:418-510** | ✅ `ssi_plane_plane` **3085-3157**, gate `vl < 1e-9` at **3092** |
| parallel, both distances ≤ Tol | — | SAME | **QQG:416** `IntAna_Same` | ❌ **3092** returns false → marcher |
| parallel, distinct | — | EMPTY | **QQG:416** `IntAna_Empty` | ❌ **3092** returns false → marcher |
| near-parallel (`\|n1×n2\| < 2e-6`) and residual `> 1e-12` | **QQG:460-461** | LINE, origin **refined** | **QQG:463-509** — two `IntAna_IntConicQuad` solves re-anchor `pt1` | ❌ no refinement |

OCCT's near-parallel note is worth reading (**QQG:448-457**): at small angles the naive origin
`pt1` carries ~1e-4 error. Their fix is (1) intersect the normal line through `pt1` with `P1` to
get `aPnt1`, (2) intersect the line through `aPnt1` along `dir1 × n1` with `P2`.

---

### 3.2 plane × cylinder  ← *chairs carry these*

| Relative configuration | Discriminant | Curve | OCCT | Ours |
|---|---|---|---|---|
| axis **not** parallel to plane, `sinθ ≥ Tol/R` | `sint = \|n × w\|` | **ELLIPSE**, major `R/\|cos\|`, minor `R` | **QQG:705-718** | ✅ `ssi_plane_cylinder` **2697-2714**, gate `\|wn\| < 1e-7` at **2701** |
| axis not parallel, `sinθ < Tol/R` | **QQG:695** | CIRCLE radius `R` | **QQG:695-704** | ✅ **2705-2709** (gate `\|w×n\| < 1e-9`) |
| axis parallel to plane, `\|\|d\|−R\| < Tol` | — | 1 tangent LINE | **QQG:611-636** | ✅ `ssi_plane_cylinder_lines` **2720+**, tangency at `tt = R·1e-9 + 1e-12` (**2728**) |
| axis parallel, `\|d\| < R` | — | 2 LINES | **QQG:637-674** | ✅ **2720+** |
| axis parallel, `\|d\| > R` | — | EMPTY | **QQG:680-683** | ✅ **2729** |
| **near-parallel with known face height H** | `sinda·H < 2·Tol` | force the LINE branch | **QQG:567-598** — relaxes `tolang = 2·sinda`, `toltang = max(Tol, sinda·H·1.01)`, and re-derives the line directions from a point 100 units along the axis (**QQG:616-631, 646-668**) | ❌ no `H`-aware relaxation |

**Assessment:** the 3D case table for plane × cylinder is essentially complete. The two real
problems are (a) the missing near-parallel `H` relaxation, which makes a nearly-tangent plane emit
a wildly eccentric ellipse instead of two lines, and (b) **the ELLIPSE's pcurve degrades** — §5.

---

### 3.3 plane × sphere

| Relative configuration | Discriminant | Curve | OCCT | Ours |
|---|---|---|---|---|
| `\|d\| < R` | — | CIRCLE radius `√(R²−d²)` | **QQG:1005-1018** | ✅ `ssi_plane_sphere` **2685-2695** |
| `\|\|d\|−R\| < Epsilon(R)` | ULP-scaled | **POINT** (tangency) | **QQG:997-1004** | ⚠ **2689** `if (\|d\| ≥ r) return false` → no point emitted; `handled` stays true so it reads as no-hit |
| `\|d\| > R` | — | EMPTY | implicit | ✅ **2689** |

Sphere tangency being dropped is defensible for face/face booleans (a point section contributes
nothing) and matches our deliberate `drop_point_sections` policy (`src/intersection.cpp:4680-4686`).
**Low priority.**

---

### 3.4 plane × cone  ← *chairs may carry these*

Common quantities (OCCT **QQG:774-797**; ours **src/intersection.cpp:2992-2998**):
`cost = |w·n|`, `sint = |n × w|`, `cosa = cos α`, `sina = |sin α|`,
`costa = cost·cosa − sint·sina`, `dist` = signed apex-to-plane distance.

| Relative configuration | Discriminant | Curve | OCCT | Ours |
|---|---|---|---|---|
| apex on plane, `\|costa\| < TolAng` | — | 1 LINE (tangent along a generator) | **QQG:805-816** | ✅ **3002-3009** |
| apex on plane, `cost < sina` | — | 2 LINES, `dh = √(sina²−cost²)/cosa` | **QQG:817-826** | ✅ **3010-3023** |
| apex on plane, else | — | POINT (apex) | **QQG:827-832** | ⚠ **3024** `return true` with nothing emitted |
| apex off plane, `cost < TolAng` | plane contains axis dir | HYPERBOLA, `a = \|dist/tan α\|`, `b = \|dist\|` | **QQG:840-852** | ✅ **3027** |
| apex off, `\|costa\| < TolAng` | — | PARABOLA, focal `= (dist/2/cosa)·sina²` | **QQG:872-882** | ✅ **3028** |
| apex off, `sint < TolAng` | plane ⟂ axis | CIRCLE, `r = \|apex→centre\|·\|tan α\|` | **QQG:883-891** | ✅ **3029** |
| apex off, `cost < sina` | — | HYPERBOLA, `a = cost·sina·cosa·D/(sina²−cost²)`, `b = cost·sina·D/√(sina²−cost²)` | **QQG:892-905** | ✅ **3030** |
| apex off, `cost > sina` | — | ELLIPSE, `a = cost·sina·cosa·D/(cost²−sina²)`, `b = cost·sina·D/√(cost²−sina²)`, centre offset `δ = sint·sina²·D/(cost²−sina²)` | **QQG:906-918** | ✅ **3031, 3042-3046** |
| **extreme conic** | `\|param\| > 1e9` (ellipse) or `> 2e6` (hyperbola) | **decline** → biparametric | **QQG:925-950** | ⚠ no limit; falls to `sample_plane_cone_arcs` (N=720) **3054-3056** |

**⚠ Constant mismatch (real, actionable).** OCCT passes `Tolang = 1.e-8` into every plane × cone
call (**III:2551**). We use `const double ang = 1e-6` (**src/intersection.cpp:2999**) — **100×
looser**. Near a branch boundary this selects PARABOLA where OCCT selects ELLIPSE or HYPERBOLA,
and the parabola/hyperbola path (**3047-3053**) is a *sampled* arc builder, so the mis-selection
is doubly costly. Also `distTol = 1e-6·max(1,H)` (**3000**) vs OCCT's `Tol` (the boolean's 3D
tolerance).

---

### 3.5 plane × torus

Guard first, in both: ring torus only. OCCT `if (aRMin >= aRMaj) → NoGeometricSolution`
(**QQG:2171-2175**).

| Relative configuration | Discriminant | Curve | OCCT | Ours |
|---|---|---|---|---|
| plane normal ∥ torus axis, `\|d\| ≤ rMin` | `aDR = \|d\| − rMin` vs `1e-13` | 1 or 2 **concentric CIRCLES**, radii `rMaj ± √(rMin²−d²)`, centred at the axis foot | **QQG:2193-2229** | ✅ `ssi_plane_torus` **3064-3078**, gate `\|\|wn\|−1\| > 1e-7` at **3068** |
| plane normal ∥ axis, `\|d\| > rMin` | `aDR > 1e-13` | EMPTY | **QQG:2204-2208** | ✅ **3070** |
| **plane normal ⟂ torus axis AND plane contains the torus centre** (`dist(centre, plane) ≤ 1e-14`) | **QQG:2233-2238** | **2 CIRCLES of radius rMin**, centres at `C ± rMaj·(w × n)`, axis `= n` | **QQG:2231-2248** | ❌ **3068** rejects → marcher |
| plane ⟂ axis but **not** through centre | — | NoGeometricSolution | **QQG:2236** | ≡ both march |
| any oblique plane | — | NoGeometricSolution (**quartic**) | **QQG:2184-2188** | ≡ both march |

**Note on Villarceau circles.** They are the famous trap and **OCCT does not compute them.** The
bitangent plane that yields the two Villarceau circles is oblique to the torus axis
(`sin θ = rMin/rMaj`), so it fails **QQG:2184** and returns `NoGeometricSolution` → `IntPTo`
returns false (**III:3850-3853**) → `ImpImp` not-done → marching (**IPI:1792-1797**). **Our target
here is parity, not superiority.** Do not attempt Villarceau closed forms; match the marcher.

---

### 3.6 cylinder × cylinder  ← ★ *the highest-value pair for chairs*

OCCT setup (**QQG:1054-1057**):
```cpp
AxeOperator A1A2(Cyl1.Axis(), Cyl2.Axis(),
                 myEPSILON_CYLINDER_DELTA_DISTANCE,   // = Precision::Confusion() = 1e-7
                 myEPSILON_AXES_PARA);                // = Precision::Angular()   = 1e-12
```
**This confirms the audited correction:** the cyl × cyl coplanarity epsilon is **1e-7**, *not*
1e-14. The 1e-14 default in the `AxeOperator` signature (**QQG:74**) is overridden here.
`AxeOperator::Coplanar()` is `distance < eps && |det3(V1,V2,P1−P2)| ≤ eps` (**QQG:159-174**), and
`Intersect() = coplanar && !parallel` (**QQG:87**).

| Relative configuration | Discriminant | Curve | OCCT | Ours |
|---|---|---|---|---|
| ∥ axes, `d ≤ Tol`, `\|R1−R2\| ≤ Tol` | — | SAME | **QQG:1074-1077** | ❌ **3926** returns false → marcher (should route to same-domain) |
| ∥ axes, `d ≤ Tol`, `R1 ≠ R2` | — | EMPTY | **QQG:1078-1081** | ✅ **3926** |
| ∥ axes, `d > R1+R2+Tol` | — | EMPTY | **QQG:1099-1103** | ✅ **3930** |
| ∥ axes, `R1+R2−d ≤ RealSmall()` | — | 1 tangent LINE at `P1 + R1/(R1+R2)·(P2−P1)` | **QQG:1104-1115** | ⚠ **3947** uses `h ≤ 1e-6` (a sagitta test) rather than the radius-sum test; looser but same answer |
| ∥ axes, `d > \|R1−R2\|` | `4R1²sin²(∠AP1P2) < Tol²` ⇒ tangent | 2 LINES (or 1 if tangent) | **QQG:1116-1197** | ✅ **3932-3949** |
| ∥ axes, `\|R1−R2\| − Tol < d ≤ \|R1−R2\|` | — | 1 internally-tangent LINE | **QQG:1198-1214** | ⚠ approximately, via `h → 0` at **3947** |
| **non-∥, `\|R1−R2\|/Rmax ≤ 1e-13` AND axes coplanar-and-intersecting** | **QQG:1224** | **2 ELLIPSES**: centre = axis intersection, minor `= R1`, majors `R1/\|sin(A/2)\|` and `R1/\|sin((π−A)/2)\|`, dirs `w1+w2` and `w1−w2` | **QQG:1224-1266** | ✅ `ssi_cylinder_cylinder` **3952-3964** — **but** with `1e-6` relative radius tolerance (**3953**) and `1e-6` coplanarity (**3955**), vs OCCT's 1e-13 / 1e-7 |
| non-∥, `\|d − R1 − R2\| < Tol` | — | 1 POINT (skew external tangency) | **QQG:1269-1290** | ❌ not handled → marcher |
| **non-∥, unequal radii — ANY configuration** | — | **general quartic** | ✅ **III:7909-8102** → `CyCyNoGeometric` (Tier 2′) | ❌❌ **3953** `if (\|R1−R2\|/Rmax > 1e-6) return false;` → **marcher** |
| **non-∥, equal radii but SKEW axes (not coplanar)** | — | **general quartic** | ✅ same Tier 2′ | ❌❌ **3955** `lines_closest_point(...)` fails → **marcher** |

**★★ This is the single largest hole in the matrix.** Two whole configuration classes —
*skew cylinders* and *unequal-radius crossed cylinders* — have no analytic branch at all. Both are
completely ordinary in real assemblies (any two pipes that are neither parallel nor perfectly
crossed at equal diameter).

---

### 3.7 cylinder × cone

| Relative configuration | Discriminant | Curve | OCCT | Ours |
|---|---|---|---|---|
| coaxial (`A1A2.Same()`) | `parallel && dist < 1e-14` (**QQG:83**) | **2 CIRCLES** of radius `R`, at `apex ± (R/tan β)·dir` | **QQG:1328-1339** — note `pt1` **and** `pt2`, both nappes | ⚠ `ssi_cylinder_cone` **3876-3888** emits **only one** circle (the `+s` nappe). For a double-napped cone the `−s` circle is lost. (Our `create_cone` is single-nap, so low priority — but a STEP-imported cone is not.) |
| **everything else** | — | **general quartic** | ✅ **III:8465-8530** → `IntAna_IntQuadQuad(Cy, Co, Tol)`, then `ExploreCurve(Co, aC, 10·Tol, aLC)` (**III:8512**) to split each ALine at the apex | ❌❌ **3880** `if (!axes_coaxial(...)) return false;` → marcher |

---

### 3.8 cylinder × sphere

| Relative configuration | Discriminant | Curve | OCCT | Ours |
|---|---|---|---|---|
| sphere centre **on** cylinder axis, `R_sph ≥ R_cyl` | `dist = √(R_sph²−R_cyl²)` | 1 or 2 CIRCLES of radius `R_cyl` at `C ± dist·w` | **QQG:1378-1399** | ✅ `ssi_cylinder_sphere` **3861-3875** |
| centre on axis, `R_sph < R_cyl` | — | EMPTY | **QQG:1380-1383** | ✅ **3866** |
| **centre off axis** | — | **general quartic** | ✅ **III:8263-8362** → `IntAna_IntQuadQuad(Cy, Sp, Tol)` → ALines | ❌❌ **3865** `if (point_axis_dist(P,w,C) > 1e-6) return false;` → marcher |

---

### 3.9 cylinder × torus

| Relative configuration | Discriminant | Curve | OCCT | Ours |
|---|---|---|---|---|
| coaxial, `rMaj−rMin ≤ R_cyl ≤ rMaj+rMin` | `aDist = √\|rMin² − (R_cyl−rMaj)²\|` | 1 or 2 CIRCLES radius `R_cyl` at `C ± aDist·w` | **QQG:2298-2329** | ✅ `ssi_cylinder_torus` **3971-3985** |
| coaxial, outside that band | — | EMPTY | **QQG:2308-2312** | ✅ **3978** |
| non-coaxial, or `rMin ≥ rMaj` | — | NoGeometricSolution | **QQG:2286-2303** → `IntCyTo` false (**III:9564-9578**) | ≡ **both march** |

---

### 3.10 cone × cone  ← ★ *entirely absent from our dispatcher*

| Relative configuration | Discriminant | Curve | OCCT | Ours |
|---|---|---|---|---|
| coaxial, `\|tg1−tg2\| > 1e-12` | `d = axis·(apex1→apex2)`, `\|d\| ≥ 1e-10` | **2 CIRCLES** at `x = d·tg2/(tg1±tg2)`, radii `\|x·tg1\|` | **QQG:1478-1505** | ❌ |
| coaxial, `\|d\| < 1e-10` (`TOL_APEX_CONF`) | — | POINT (shared apex) | **QQG:1488-1494** | ❌ |
| coaxial, `\|tg1−tg2\| ≤ 1e-12`, `\|d\| < 1e-10` | — | SAME | **QQG:1508-1511** | ❌ |
| coaxial, equal semi-angle, `\|d\| ≥ 1e-10` | — | 1 CIRCLE at `d/2` | **QQG:1512-1520** | ❌ |
| **∥ axes, `\|tg1−tg2\| < aTolAng`** | adaptive `aTolAng`, **QQG:1458-1475** | build the **bisector plane** through `P1` with normal `(DA1×DB1)×P1MO1O2`, then reduce to **plane × cone** → ELLIPSE / CIRCLE / HYPERBOLA / LINE | **QQG:1524-1606** | ❌ |
| **coincident apices** (`d² < Tol²`) | 2D construction **QQG:1616-1677**: `aRD2` vs `aR2 ± Tol` | EMPTY / 1 LINE (touch) / 2 LINES | **QQG:1608-1758** | ❌ |
| **intersecting axes with a common generatrix** | both apices lie on the other cone (`ElSLib` round-trip within `Tol²`), **QQG:1762-1778** | build the plane through `PChar` with the computed normal → plane × cone conic; sets `myCommonGen = true` | **QQG:1760-1884** | ❌ |
| everything else | — | **general quartic** | ✅ **III:9022+** → `IntAna_IntQuadQuad(Co1, Co2, Tol)` | ❌ |

`aTolAng` adaptation is worth copying verbatim (**QQG:1458-1475**): when the cones are parallel and
`DistA1A2 > 100·Tol`, OCCT estimates the minimum apex-to-solution distance with `EstimDist`
(**QQG:263-325** — projects both cones' generator lines into the apex-apex plane and intersects
them in 2D) and sets `aTolAng = clamp(Tol/aMinSolDist, Precision::Angular(), Tol)`. Without it,
a fixed angular tolerance either misses near-parallel cones or over-triggers on distant ones.

**Our dispatcher has no CONE × CONE arm at all** — the chain at **src/intersection.cpp:4157-4211**
skips it, so the pair reaches `else { return res; }` at **4213**.

---

### 3.11 cone × sphere

| Relative configuration | Discriminant | Curve | OCCT | Ours |
|---|---|---|---|---|
| sphere centre **on** cone axis | roots of `(1+t²)x² + 2t²·d·x + (t²d² − R²) = 0`, `t = tan α`, `d = \|apex→centre\|` | 1 or 2 CIRCLES, radius `\|t·(d+x)\|`, centre `apex + (d+x)·dir` | **QQG:1926-1999** | ✅ `ssi_cone_sphere` **3890-3916** — the *same* quadratic, **3899** |
| centre on axis, root radius `≤ 1e-9` | `myEPSILON_MINI_CIRCLE_RADIUS = 0.01·Confusion()` | **`IntAna_PointAndCircle`** | **QQG:1971-1975, 1988-1992** | ⚠ **3911** `if (rr < 1e-6) continue;` — drops it entirely, and the threshold is 1000× looser |
| no real root | — | EMPTY | **QQG:1953-1956** | ✅ **3901** |
| **centre off axis** | — | **general quartic** | ✅ **III:9349+** → `IntAna_IntQuadQuad(Co, Sp, Tol)` | ❌❌ **3894** `if (point_axis_dist(apex,a,C) > 1e-6) return false;` → marcher |

---

### 3.12 cone × torus

| Relative configuration | Curve | OCCT | Ours |
|---|---|---|---|
| coaxial (`torus axis ∥ cone axis` **and** apex on the torus axis within 1e-14) | up to **4 CIRCLES** — the cone's generator line is rotated into the tube plane and intersected with each of the two tube circles (`±XDir`), each giving up to 2 solutions | **QQG:2377-2468** | ✅ `ssi_cone_torus` **3986-4010** (same 2×2 structure via `solve_emit(±R)`) |
| anything else | NoGeometricSolution | **QQG:2377-2382** → `IntCoTo` false (**III:9582-9596**) | ≡ both march |

---

### 3.13 sphere × sphere

| Relative configuration | Discriminant | Curve | OCCT | Ours |
|---|---|---|---|---|
| `d ≤ Tol` and `\|R1−R2\| ≤ Tol` | — | SAME | **QQG:2057-2060** | ❌ not detected (empty result reads as no-hit) — should route to same-domain |
| `t = Rmax − d − Rmin ∈ [0, Tol]` | — | POINT (internal tangency) | **QQG:2079-2094** | ⚠ excluded by `tan_tol = (R1+R2)·1e-9` at **4185** — deliberate, see the comment at **4180-4183** |
| `d > R1+R2+Tol` or `Rmax > d+Rmin+Tol` | — | EMPTY | **QQG:2105-2108** | ✅ **4185** |
| otherwise | `β = √(R1²−α²)`, `α = (R1²−R2²+d²)/(2d)` | CIRCLE radius `β` at `O1 + α·dir` | **QQG:2115-2132** | ✅ **4187-4193**, identical formula |
| `β ≤ 1e-9` | `myEPSILON_MINI_CIRCLE_RADIUS` | POINT | **QQG:2119-2123** | ⚠ via `rr2 > 0.0` at **4189** |

**sphere × sphere is the one pair where OCCT never needs Tier 2** — the closed form is total.

---

### 3.14 sphere × torus

| Relative configuration | Curve | OCCT | Ours |
|---|---|---|---|
| sphere centre on the torus axis (within 1e-14) | 1 or 2 CIRCLES — circle/circle intersection in the (ρ, z) half-plane between the tube circle at `C + rMaj·XDir` and the sphere | **QQG:2510-2560** | ✅ `ssi_sphere_torus` **4011-4032** |
| otherwise | NoGeometricSolution | **QQG:2514-2518** | ≡ both march |

---

### 3.15 torus × torus

| Relative configuration | Curve | OCCT | Ours |
|---|---|---|---|
| coaxial, coincident, equal `rMin`/`rMaj` | SAME | **QQG:2612-2617** | ❌ **4126** `if (d < 1e-6) return false;` → marcher |
| coaxial, distinct | 1 or 2 CIRCLES — circle/circle in the (ρ, z) half-plane | **QQG:2625-2665** | ✅ `ssi_torus_torus` **4118-4139** |
| coaxial, tube circles apart | EMPTY | **QQG:2634-2638** | ✅ **4127** |
| **∥ axes, non-coaxial, equal `R` and `r`, zero axial offset** | NoGeometricSolution → marching | **QQG:2606-2610** | ⭐ `ssi_torus_torus_spiric` **4043-4117** — **we exceed OCCT** with the exact spiric factorisation. *Caveat:* it emits a **512-point interpolated loop** (`const int N = 512`, **4064**), not an exact rational curve, so it carries interpolation error of its own. |
| anything else | NoGeometricSolution | **QQG:2606-2610** | ≡ both march |

---

## 4. RANKED GAP LIST — what session A should write, in order

Ranking weighs (a) how often the configuration occurs in real geometry, (b) whether our chairs
carry the surface types, (c) implementation cost.

| # | Gap | Our site | OCCT reference | Why it ranks here |
|---|---|---|---|---|
| **1** | **Pcurve degradation on oblique conics** (§5) — 3D exact, UV fitted | `intersection.cpp:3214` (cyl), `:3242` (sph), `:3270` (cone), `:3309` (tor); wiring hole at `:4223-4226` | `ProjLib` analogues; OCCT never fits | Explains **both** measured 9.4e-05 cells. Two of the three fixes are ~2 lines. |
| **2** | **cyl × cyl, non-parallel + unequal radii** | `intersection.cpp:3953` | **III:7909-8102**, **III:6573**, **III:3949-4250** | Chairs are planar + cylindrical. Any two non-parallel pipes of different diameter. Whole class missing. |
| **3** | **cyl × cyl, equal radii but SKEW axes** | `intersection.cpp:3955` | same Tier 2′ | Sibling of #2; same code path once #2 exists. |
| **4** | **cyl × sphere, centre off axis** | `intersection.cpp:3865` | **III:8263-8362** → **IQQ:375** | Very common (a boss/fillet meeting a spherical cap). Tier 2 cylinder×quadric. |
| **5** | **cyl × cone, non-coaxial** | `intersection.cpp:3880` | **III:8465-8530** → **IQQ:375** | Same Tier-2 machine as #4 — one implementation covers both. |
| **6** | **cone × cone — arm entirely absent** | `intersection.cpp:4213` (no branch) | **QQG:1433-1890**, **III:9022+** | 7 distinct Tier-1 sub-branches plus Tier 2. Large but self-contained. |
| **7** | **cone × sphere, centre off axis** | `intersection.cpp:3894` | **III:9349+** → **IQQ:841** | Falls out of the Tier-2 cone×quadric implementation for free once #6 exists. |
| **8** | **plane × torus, plane through the torus axis** | `intersection.cpp:3068` | **QQG:2231-2248** | Small, exact, ~15 lines: 2 circles of radius `rMin` at `C ± rMaj·(w×n)`. |
| **9** | **SAME detection routed to the marcher** — plane∥plane coincident, coaxial equal cylinders, identical tori | `intersection.cpp:3092`, `:3926`, `:4126` | **QQG:416**, **:1076**, **:2615** | These should reach the same-domain path (`src/brep_samedomain.cpp`), never the marcher. Cheap and prevents spurious sections. |
| **10** | **plane × cone angular tolerance 1e-6 vs OCCT 1e-8** | `intersection.cpp:2999` | **III:2551** | One-constant change; mis-selects the conic branch near boundaries and the parabola/hyperbola branch is sampled. |
| **11** | **plane × cylinder near-parallel `H` relaxation** | absent | **QQG:567-598** | Prevents a nearly-tangent plane emitting a degenerate ellipse instead of two rulings. |
| **12** | **cyl × cone coaxial: second nappe circle** | `intersection.cpp:3885` | **QQG:1333-1334** | Only bites on double-napped (STEP-imported) cones. |
| **13** | **skew cylinder external tangency POINT** | absent | **QQG:1269-1290** | A point section; low value for booleans. |
| **14** | **`IntAna_PointAndCircle` degenerate radius** | `intersection.cpp:3911` (`< 1e-6`, drops) | **QQG:1971-1975** (`≤ 1e-9`, emits point+circle) | Threshold is 1000× too coarse; a genuine small circle can be silently dropped. |

**Gaps #2–#7 all collapse to two pieces of machinery:** Tier 2 (`IntAna_Quadric::NewCoefficients`
+ `IntAna_IntQuadQuad`, §2) and Tier 2′ (`stCoeffsValue`, §6.2). Building those two is the whole
job; each individual pair then becomes a thin dispatcher arm.

---

## 5. ★ THE PCURVE GAP — most likely source of the measured 9.4e-05 cluster

This is **not** a case-table gap and deserves its own section because it is cheap to fix and
almost certainly dominant.

### 5.1 The mechanism

`analytic_ssi` (`src/intersection.cpp:4141`) does two things: produce the exact 3D curve, then
pull it back to UV on both surfaces (**4218-4229**):

```cpp
NurbsCurve pa = analytic_pcurve(a, ra, cc3);
NurbsCurve pb = analytic_pcurve(b, rb, cc3);
if (!pa.is_valid() && ra.kind == K::TORUS) { ... analytic_torus_pullback ... }
if (!pb.is_valid() && rb.kind == K::TORUS) { ... analytic_torus_pullback ... }
if (!pa.is_valid()) { auto v = Closest::surface_curve(a, cc3); if (!v.empty()) pa = v[0]; }
if (!pb.is_valid()) { auto v = Closest::surface_curve(b, cc3); if (!v.empty()) pb = v[0]; }
```

`analytic_pcurve` (`src/intersection.cpp:3159-3340`) is exact **only for a plane** (affine inverse
of the bilinear map, **3164-3189**). For every curved surface it demands that the conic be a
**chart-aligned circle** and declines otherwise:

| surface | gate | line |
|---|---|---|
| CYLINDER | `hmax − hmin > 1e-5·\|h1−h0\|` → decline (must be constant height) | **3214** |
| CYLINDER | start/end must coincide (full wrap) | **3216-3217** |
| SPHERE | `hmax − hmin > R·1e-4` → decline (must be constant latitude) | **3242** |
| SPHERE | `dist(start,end) > R·1e-3` → decline (must be a full parallel) | **3243** |
| CONE | `hmax − hmin > hscale·1e-4` → decline | **3270** |
| TORUS | `amax − amin > 1e-4` → decline (must be constant tube angle) | **3309** |

When it declines, `Closest::surface_curve` fits a **cubic B-spline** through sampled projections.
Its CV budget is a fixed formula, `src/closest.cpp:690`:

```cpp
int target_cvs = std::max(8, (int)(total_turning / 0.5) + 6);
```

For a closed loop `total_turning = 2π`, so `target_cvs = 18` — a cubic with ~15 spans. The radial
deviation of a cubic B-spline approximating a circle of radius `R` over span angle `θ = 2π/15` is
`≈ R·θ⁴/384 ≈ 8e-5·R`, i.e. **relative error ≈ 1e-4**, independent of the geometry. That is a
fixed constant that reproduces 9.39e-05 / 9.18e-05 / 9.45e-05 / 9.69e-05 across four different
operand pairs — exactly the "agreeing to two significant figures across different geometry"
signature in the brief. The acceptance tolerance is also loose: `fit_tol_uv = step`
(`src/closest.cpp:661`) is the **whole marching step**.

The error then enters the volume directly, because the closed-form flux integrand is gated on the
curve being an exact rational: `brep.cpp:2657` `if (c3 && c3->is_rational()) { ... }` and
`brep.cpp:2809` `if (have_analytic && !curved_rect && !pole_face) flux_nat = flux_analytic;`.
A fitted non-rational cubic falls to sampled Green's-theorem integration.

### 5.2 Why this explains the two measured cells exactly

- **`cyl × cylR` (45° crossed, equal radii) → 9.39e-05.** The 3D result *is* exact: two ellipses
  from `ssi_cylinder_cylinder` **3963-3964**. But an ellipse on a cylinder is **not** constant
  height, so the CYLINDER gate at **3214** rejects it on *both* operands → two fitted pcurves.
- **`boxR × sph` (rotated box vs sphere) → 9.45e-05.** The 3D result is an exact circle from
  `ssi_plane_sphere` **2685-2695**. The plane side is exact (**3164**). But the circle's plane is
  oblique to the sphere's chart axis, so the SPHERE gate at **3242** rejects → one fitted pcurve.

In both cases the *case table was never the problem* — the exact conic was found and then thrown
away at the UV boundary.

### 5.3 The fix, in order of payoff

1. **Wire in the general pullbacks that already exist.** `analytic_sphere_pullback`
   (`src/intersection.cpp:3348`) and `analytic_cone_pullback` (`src/intersection.cpp:3509`) handle
   *oblique* curves — they invert the longitude map by table + bisection rather than assuming a
   chart-aligned circle. They are currently reachable **only** from `cut_curves_on_surface`
   (`src/intersection.cpp:5602`, `:5609`, `:5615`). Mirror the TORUS wiring at **4223-4224**:

   ```
   if (!pa.is_valid() && ra.kind == K::SPHERE)   { analytic_sphere_pullback(a, ra, cc3) }
   if (!pa.is_valid() && ra.kind == K::CONE)     { analytic_cone_pullback(a, ra, cc3) }
   if (!pa.is_valid() && ra.kind == K::CYLINDER) { analytic_cone_pullback(a, ra, cc3) }   // cylinder is a surface of revolution too — see the comment at :3503
   ```
   …and the same four for `pb`. **This is the single highest-value change in this document.**

2. **Add an exact ellipse pcurve for the cylinder.** A plane section of a cylinder pulls back to
   `v = h₀ + (R/tan φ)·cos(u − u₀)` in `(u = θ, v = height)` — a pure cosine, exactly
   representable to any required accuracy and, more importantly, **analytically differentiable**
   so the flux integrand stays closed-form. This retires gate **3214** for the ELLIPSE case.

3. **If a fit is genuinely unavoidable, raise the budget and tighten the acceptance.**
   `src/closest.cpp:690` `total_turning / 0.5` → `/ 0.1` moves the deflection by ~5⁴ ≈ 625×
   (1e-4 → 1.6e-7); `src/closest.cpp:661` `fit_tol_uv = step` → `step * 1e-3`. Use this as the
   *diagnostic* first: if bumping 690 moves the 9.4e-05 numbers by ~600×, the diagnosis in §5.1 is
   confirmed and fixes 1–2 are the right permanent answer.

---

## 6. Constants, tolerances and encodings — the audited table

### 6.1 `IntAna_QuadQuadGeo::InitTolerances` (**QQG:351-359**)

| member | value | used for |
|---|---|---|
| `myEPSILON_DISTANCE` | `1.0e-14` | axis coplanarity default; torus coaxiality (**QQG:2234, 2299, 2378, 2514, 2606**) |
| `myEPSILON_ANGLE_CONE` | `Precision::Angular()` = **1e-12** | cone semi-angle equality (**QQG:1486**) |
| `myEPSILON_MINI_CIRCLE_RADIUS` | `0.01 · Precision::Confusion()` = **1e-9** | degenerate circle → `PointAndCircle` (**QQG:1971, 1988, 2119**) |
| `myEPSILON_CYLINDER_DELTA_RADIUS` | `1.0e-13` | **relative** `\|R1−R2\|/Rmax` for crossed cylinders (**QQG:1224**); also the plane×torus numeric tol (**QQG:2197**) |
| `myEPSILON_CYLINDER_DELTA_DISTANCE` | `Precision::Confusion()` = **1e-7** | **the cyl × cyl coplanarity epsilon** (**QQG:1056**) |
| `myEPSILON_AXES_PARA` | `Precision::Angular()` = **1e-12** | axis parallelism everywhere |

OCCT `Precision` values (`FoundationClasses/TKernel/Precision/Precision.hxx`):
`Angular() = 1e-12` (:123), `Confusion() = 1e-7` (:165), `PConfusion() = Confusion()·0.01 = 1e-9`
(:334), `Intersection() = Confusion()·0.01 = 1e-9` (:220), `Approximation() = 1e-6` (:235).

**Audit confirmation:** the brief's note that the cyl × cyl coplanarity epsilon is **1e-7
(Precision::Confusion), not 1e-14**, is correct and is visible at **QQG:1054-1057** — the
`AxeOperator` default of 1e-14 (**QQG:74**) is explicitly overridden for this call.

### 6.2 Tier 2′ (cyl × cyl) constants

| constant | value | site |
|---|---|---|
| determinant floor for `V1,V2` solve | `Precision::Angular()` = 1e-12, throws on failure | **III:4123-4126** |
| 2D tolerance | `min(1e-4, min(S1->UResolution(TolTang), S2->UResolution(TolTang)))` | **III:2654-2655** |
| infinite-curve reject | `aRange.Delta() > 1e5 · Radius` → `IntStatus_InfiniteSectionCurve` | **III:6603-6608** |
| "good intersection" heuristic | `R_max > 3·R_min`, axes normal within `π/18` (10°), `axisdist ≤ R_max/2` | **III:6618-6652** |
| WLine point budget | `aNbMaxPoints = 1000`, `aNbMinPoints = 200`; if "good": `200 / 50` | **III:6675-6683** |
| default step | `du = 2π / aNbMaxPoints` | **III:6686** |
| "good" step | relative deflection `0.001`, `aNbP = 2π/(2·acos(1−0.001)) + 1` | **III:6656-6664** |
| step bounds | `aStepMin = max(aTol2D, Precision::PConfusion())`, `aStepMax = (U1l−U1f)/aNbPoints` | **III:6691-6694** |
| critical-point array | `aNbCritPointsMax = 12` | **III:6724** |
| zero threshold | `aNulValue = 1.0e-11` | **III:3930** |

### 6.3 Transition dead-bands — **non-uniform, as audited**

| value | site | pair |
|---|---|---|
| **0.0** | **III:3833** (`> 0.0`, no dead-band at all) | plane × torus (`IntPTo`) |
| **1e-9** (`myTolOpenDomain`) | `IntPatch_ALineToWLine.cxx:151` | ALine domain trimming |
| **1e-8** (`myTolTransition`) | `IntPatch_ALineToWLine.cxx:152`, used at `:924, :929` | ALine → WLine transitions |
| **1e-8** (`0.00000001`) | **III:4997/5002** (cyl×cyl lines), **:5052/5057, :5128/5133** (cyl×cyl ellipses), **:8322/8327** (cyl×sph ALine), **:9686/9691** (torus results), **:3731/3736, :3761/3766** | the common case |
| **1e-7** (`0.0000001`) | **III:8243/8248** | cylinder × sphere, **second** circle only |

All are applied to `qwe = Tgt.DotCross(Quad2.Normale(P), Quad1.Normale(P))`; outside the band
`trans1/trans2` become `Out/In` or `In/Out`, inside it both become `IntSurf_Undecided`.

### 6.4 Tangency encoding — **`IntSurf_Touch` + `Situation`, NOT `Undecided`** (audit confirmed)

`IntPatch_Line` has two three-argument constructors (`IntPatch_Line.cxx:21` and `:36`). The
**Situation** one is what tangency uses:

```cpp
IntPatch_Line::IntPatch_Line(const bool Tang,
                             const IntSurf_Situation Situ1,
                             const IntSurf_Situation Situ2)
    : tg(Tang),
      tS1(IntSurf_Touch),      // <-- forced
      tS2(IntSurf_Touch),      // <-- forced
      sit1(Situ1), sit2(Situ2), ...
```
`IntPatch_Line.cxx:36-43`.

Enums: `IntSurf_TypeTrans { In, Out, Touch, Undecided }`
(`IntSurf/IntSurf_TypeTrans.hxx`); `IntSurf_Situation { Inside, Outside, Unknown }`
(`IntSurf/IntSurf_Situation.hxx`).

Call sites that build tangency lines with Situations:
- cyl × cyl tangent line — **III:4898-4985**: `crb1·crb2 < 0` distinguishes "opposed curvature
  centres"; note the deliberate cross-assignment (**III:4917-4920**: *"Normal and Radius-vector of
  the 1st cylinder is used for judging what the situation of the 2nd cylinder is"*).
  Then `new IntPatch_GLine(linsol, true, situcyl1, situcyl2)` (**III:4984**).
- cyl × sphere single circle — **III:8172-8223**, `new IntPatch_GLine(cirsol, true, situcyl, situsp)`.
- plane × cone — **III:3534-3535** (`situco`, `situco_otherside`, `situp`, `situp_otherside`).
- plane × cylinder — **III:3218-3219**.

**Implication for us:** a tangency must be recorded as `Touch` + which side each surface is on, not
as an undecided transition and not as a dropped section. Our grazing/tangency residue lives
precisely here: we currently either drop the section (plane × sphere tangency, `intersection.cpp:2689`;
sphere × sphere tangency, `:4185`) or emit a phantom (the comment at `:4180-4183` documents the
one-ULP phantom circle that motivated the current gates). The right target is a **Touch section
with Inside/Outside recorded**, which the classifier can then use instead of guessing.

### 6.5 Conic extent limits (**QQG:925-950**)

```cpp
static double EllipseLimit   = 1.0E+9;  // OCC513(apo), was 1000000
static double HyperbolaLimit = 2.0E+6;  // OCC537(apo), was 50000
```
Exceeding either sets `done = false`, which propagates to `ImpImp` not-done and hence to marching.
We have no equivalent guard.

---

## 7. Torus — revised assessment

### 7.1 What OCCT actually does

**OCCT's torus coverage is thin, and deliberately so.** Every torus pair requires the *other*
surface to be **coaxial or axis-aligned** with the torus:

| pair | required relative configuration | else |
|---|---|---|
| plane × torus | plane normal ∥ torus axis, **or** normal ⟂ axis and the plane passes through the centre (≤1e-14) | NoGeometricSolution (**QQG:2184-2188**) |
| cylinder × torus | axes parallel **and** cylinder location on the torus axis (≤1e-14) | NoGeometricSolution (**QQG:2298-2303**) |
| cone × torus | axes parallel **and** apex on the torus axis (≤1e-14) | NoGeometricSolution (**QQG:2377-2382**) |
| sphere × torus | sphere centre on the torus axis (≤1e-14) | NoGeometricSolution (**QQG:2514-2518**) |
| torus × torus | axes parallel **and** coaxial (≤1e-14) | NoGeometricSolution (**QQG:2606-2610**) |

All five additionally require a **ring torus**: `rMin < rMaj`, else NoGeometricSolution
(**QQG:2171, 2286, 2365, 2504, 2619**).

`NoGeometricSolution` → `TreatResultTorus` returns `false` (**III:9707-9710**) → `IntPTo`/`IntCyTo`/
`IntCoTo`/`IntSpTo`/`IntToTo` return `false` → `ImpImp::Perform` returns with
`myDone = IntStatus_Fail` → `GeomGeomPerfom` calls `ParamParamPerfom` (**IPI:1792-1797**) →
**full numerical marching**. There is no Tier 2 for torus — `IntAna_IntQuadQuad` only has
`Perform(gp_Cylinder, IntAna_Quadric)` and `Perform(gp_Cone, IntAna_Quadric)`, and a torus is
quartic so it cannot be an `IntAna_Quadric` at all.

**Conclusion:** our torus coverage (§3.5, §3.9, §3.12, §3.14, §3.15) is **at parity with OCCT and
in one cell ahead of it** (the spiric factorisation, `intersection.cpp:4043-4117`). The only real
torus case-table gap is §3.5's plane-through-axis branch (rank #8) and the missing SAME detection
(rank #9).

### 7.2 The `box × tor` defect is NOT a case-table gap — flag for session A

Session A's measurement:

```
box × tor   cut   ~5e-11  stable at every angle
box × tor   fuse  ~5e-11  stable at every angle
box × tor   common 2.38e-02 @ 1e-4 deg
                   2.78e-08 @ 1e-2 deg
                   4.12e-10 @ 0.1 deg     <- non-monotonic
face counts        7 / 5 / 16              stable throughout
```

Topology stable, two of three operations stable, one operation wrong and **non-monotonic in the
perturbation angle**. That is not the signature of a missing analytic branch — a missing branch
degrades *all* operations equally and *monotonically*, because they share the same section curves.
A single op misbehaving with stable face counts points at **selection/assembly on the `common`
path**: shell orientation, face-state classification, or the same-domain/keep decision.

**Recommendation:** do not chase this in the case table. Open it as a separate defect against
`brep.cpp`'s common-path classification, and reproduce it with `SESSION_VOL_DBG=1`
(`brep.cpp:2810`, prints `gauss=` vs `an=` per face) to see whether a *single* face's flux flips
sign or whether the face-keep set differs between the working and failing angles.

---

## 8. The fallback contract — what marching must guarantee

When no closed form exists (torus outside its branches; genuine freeform), OCCT still holds a
contract that our marcher currently does not.

### 8.1 Exact points, approximate connectivity

Tier 2′ is the model: `CyCyNoGeometric` (**III:6573**) does *not* Newton-iterate onto the surfaces.
It steps `U1` and computes everything else in **closed form**:

- `U2 = FI2 ± acos(B·cos(U1 − FI1) + C)` — **III:3982** (the derivation is at **III:3951-3989**)
- `V1 = K1·cos(U1−FIV1) + L1·cos(U2−PSIV1) + M1`, `V2` likewise — **III:3977-3979**

so **every emitted point lies exactly on both surfaces to machine precision**. The approximation is
confined to the polyline *between* points. Our marcher instead Newton-corrects
(`intersection.cpp:4856-4900`, 8 iterations) to `conv_tol = max(tolerance, h_init·1e-7)`
(`:4846`) — good, but then the *fit* through those points (§5.1) throws the accuracy away.

**Contract item 1:** the point set must be exact on both surfaces; the curve through them must be
fitted to a **stated deflection**, and that deflection must be recorded.

### 8.2 Error must be recorded, not discarded

OCCT attaches a tolerance to every intersection point (`IntPatch_Point::SetValue(P, Tol, tang)`)
and to every vertex, and `BRepAlgoAPI` propagates it into the edge tolerance. We currently fit to
`fit_tol_uv = step` (`closest.cpp:661`) and record nothing.

**Contract item 2:** the fitted curve's measured maximum deviation must be written into the section
edge's tolerance so downstream classification can widen its tests, exactly as OCCT does.

### 8.3 Known marcher hazards (from the existing audits, re-verified)

- **`myFleche` floor is 1e-3** — `IPI:159-162`:
  ```cpp
  if (myFleche < 1.0e-3) { myFleche = 1e-3; }
  ```
  and the default when unset is `0.01` (**IPI:184-187**). So OCCT's *own* marching deflection is
  never finer than 1e-3 — another reason not to try to beat it on torus.
- **`myUVMaxStep`'s 1e-3 floor is commented out** — `IPI:163`:
  ```cpp
  // if(myUVMaxStep<1.0e-3) myUVMaxStep=1e-3;
  ```
  with only a `> 0.5` ceiling (**IPI:168-171**) and a `== 0.0 → 0.01` default (**IPI:188-191**).
  `DefineUVMaxStep` returns `0.001`, tightened to `0.0001` only when a singular point is between
  `Precision::Confusion()` and `1e-5` away (**IPI:2372-2408**). Confirms the audit note.
- **`RejectIndexMAX = 250000` is never reset on walk reversal** and is bypassed by the tangent-zone
  continuation — per `kb/audit_occt_blowup-guards.md`; unchanged in this source.
- **`PutToBoundary` is `HandleSingleSingularPoint`**, not the gradient minimiser (which is a no-op
  for non-BSpline surfaces) — per `kb/audit_occt_ssi-walking.md`; unchanged.
- **ALine → WLine discretisation** uses a fixed `aNbPointsInALine = 200` (**IPI:1807**,
  `IntPatch_ALineToWLine AToW(theS1, theS2, aNbPointsInALine)` at **IPI:1816**). If we port Tier 2,
  match this budget rather than inventing one.

---

## 9. PORT ORDER, with an acceptance test for each step

Each step is independently shippable and independently testable. Volumes below are stated for the
canonical operand shapes; substitute the actual corpus primitives when wiring the tests.

---

**STEP 0 — Confirm the §5 diagnosis before writing any new branch.**
Change `src/closest.cpp:690` `total_turning / 0.5` → `/ 0.1` temporarily.
*Acceptance:* the four 9.4e-05 cells drop by roughly 5⁴ ≈ 600× (to ~1.5e-07). If they do, §5 is
confirmed and STEP 1 is the permanent fix. If they do **not** move, the error is in the 3D curve
and STEP 2 leads instead. **Revert the change either way.**

---

**STEP 1 — Wire the general pullbacks into `analytic_ssi`.** (`src/intersection.cpp:4223-4226`)
Add SPHERE, CONE and CYLINDER arms mirroring the existing TORUS arms, calling
`analytic_sphere_pullback` (`:3348`) and `analytic_cone_pullback` (`:3509`).
*Acceptance:*
- `boxR × sph` (box rotated 30° about (1,1,1), sphere r=1 centred in it): `cut`, `common`, `fuse`
  all ≤ **1e-10** relative, against the axis-aligned `box × sph` answer. Currently 9.45e-05 /
  9.69e-05 / 3.51e-05.
- `cyl × cylR` (equal radii, axes crossing at 45°): ≤ **1e-10**. Currently 9.39e-05 / 9.18e-05 /
  5.56e-02, and `fuse` must additionally report `is_solid = 1`.
- Regression: the full 45-cell matrix stays at 45/45.

---

**STEP 2 — Exact ellipse pcurve on the cylinder.** Retire gate `src/intersection.cpp:3214` for the
plane-section ellipse: pull back to `v = h₀ + (R/tan φ)·cos(u − u₀)`.
*Acceptance:* `box × cyl` with the box rotated 0.01° (session A measured a break to 1.89e-09 there)
returns to ≤ **1e-12**; and `plane × cylinder` at 45° gives an ellipse whose area matches
`π·R²/|cos φ|` to **1e-13** relative.

---

**STEP 3 — Tier 2 machinery: `Quadric` + `NewCoefficients`.** (§2) New internal type carrying the
10 coefficients, built from each `RecogSurface`, plus the congruence transform. No dispatcher change
yet.
*Acceptance:* unit test — for each of {plane, cylinder, cone, sphere} build the quadric, evaluate
`f` at 100 points sampled on the surface, require `|f| ≤ 1e-12·scale²`; then transform into a
random `gp_Ax3` and re-evaluate the transformed points, requiring the same bound. This is a pure
algebra test with no geometry dependency.

---

**STEP 4 — Tier 2: cylinder × quadric.** (`IQQ:375-826`, equations in §2.1) Unlocks gaps #4 and #5.
*Acceptance:*
- **cyl × sphere, off-axis.** Cylinder r=1 along Z through the origin; sphere r=2 centred at
  (0.5, 0, 0). The section is a single closed quartic. Test invariants rather than a closed-form
  volume: every emitted point satisfies `|x²+y²−1| ≤ 1e-12` **and** `|(x−0.5)²+y²+z²−4| ≤ 1e-12`;
  the curve is closed to 1e-12; and `common` volume is stable to **1e-9** under a shared 37°
  rotation of both operands.
- **cyl × cone, non-coaxial.** Cylinder r=1 along Z at the origin; cone apex (2,0,0), axis +X,
  semi-angle 30°. Same two-residual invariant, plus: the ALine must be **split at the apex**
  (mirror `ExploreCurve`, **III:8608**, tolerance `10·Tol`) — assert the emitted curve count
  matches OCCT's for this configuration.

---

**STEP 5 — Tier 2′: cyl × cyl general.** (`stCoeffsValue`, **III:4063-4250**; walker **III:6573**)
Unlocks gaps #2 and #3 — **the largest frontier mover for chairs.**
*Acceptance:*
- **Unequal radii, crossed.** r=1 along Z at origin; r=0.6 along X at origin. Two branches;
  `U2 = FI2 ± acos(B·cos(U1−FI1)+C)`. Assert both residuals ≤ 1e-12 at 1000 sampled `U1`, and
  `common` volume stable to **1e-9** under a shared rotation sweep 0→45°.
- **Skew, equal radii.** Both r=1; axis 1 = Z through origin, axis 2 = X through (0, 0, 0.5) —
  distance 0.5, not coplanar. Same invariants. This is the cell that currently has *no* branch.
- **Determinant guard.** Two nearly-parallel cylinders (0.5e-12 rad apart) must throw/decline
  cleanly at the `|aDetV1V2| < Precision::Angular()` check (**III:4123-4126**) and fall to the
  parallel branch, not divide by zero.
- Regression: the previously-working equal-radius crossed case (`3952-3964`) must still take the
  **Tier 1 ellipse** path, not the new Tier 2′ path — Tier 1 is checked first (**III:7909**).

---

**STEP 6 — cone × cone.** (`QQG:1433-1890` for Tier 1's seven branches; **III:9022** for Tier 2.)
Requires STEP 3 for the Tier-2 half; the Tier-1 half is independent.
*Acceptance:*
- Coaxial, different semi-angles (20° and 35°, apices 3 apart on the shared axis): exactly
  **2 circles**, radii matching `|x·tg1|` at `x = d·tg2/(tg1±tg2)` to **1e-13**.
- Coincident apices, axes at 40°, semi-angles 25° and 25°: exactly **2 lines** through the apex
  (the `iRet = 2` branch, **QQG:1742-1757**).
- Parallel axes, equal semi-angle 30°, axes 2 apart: the bisector-plane reduction must yield an
  **ellipse** whose major/minor match a direct plane × cone solve on the same plane to **1e-13**.
- Common generatrix: two cones sharing one ruling — `HasCommonGen()` true and `PChar` on both
  surfaces to 1e-12.

---

**STEP 7 — the small exact wins.** In one pass:
- plane × torus through the axis (rank #8) — `src/intersection.cpp:3068`; *acceptance:* torus
  R=3, r=1 about Z at origin; plane `y = 0`. Expect **exactly 2 circles of radius 1** centred at
  (±3, 0, 0) with normal +Y, to **1e-14**.
- plane × cone `ang` 1e-6 → 1e-8 (rank #10) — `src/intersection.cpp:2999`; *acceptance:* the
  45-cell matrix stays 45/45 and the parabola branch is entered only when
  `|costa| < 1e-8`.
- SAME routing (rank #9) — `:3092`, `:3926`, `:4126`; *acceptance:* `A op A` for a
  cylinder and for a torus produces **zero** section curves and reaches the same-domain path, not
  the marcher (guard already exists at `brep_section.cpp:459-465`).
- cyl × cone second nappe (rank #12) — `:3885`.
- `PointAndCircle` threshold 1e-6 → 1e-9 (rank #14) — `:3911`.

---

**STEP 8 — the fallback contract.** (§8) Record the fitted deviation into the section edge
tolerance; raise `closest.cpp:690`'s budget or replace the fit with an exact form wherever §5's
work leaves a fit in place.
*Acceptance:* every section edge produced by the marcher carries a tolerance ≥ its measured
deviation, verified by re-evaluating 100 points per edge against both surfaces.

---

## 10. Summary for the implementer

1. **Do STEP 0 first.** It is a one-line experiment that decides whether the next two weeks go into
   pcurves (§5) or into the case table (§3). The evidence strongly favours pcurves.
2. **The case table is in better shape than expected.** Of 15 pairs, 9 are at or above OCCT parity.
   The real holes are concentrated in three cells: **cyl × cyl non-parallel-unequal**,
   **cyl × sphere off-axis / cyl × cone non-coaxial**, and **cone × cone (absent entirely)**.
3. **Those three holes collapse to two pieces of machinery**: `IntAna_Quadric::NewCoefficients` +
   `IntAna_IntQuadQuad` (Tier 2, §2) and `stCoeffsValue` (Tier 2′, §6.2). Build those and the
   dispatcher arms are thin.
4. **Torus needs no case-table work.** We are at parity or ahead. The `box × tor common` defect is
   an assembly/selection bug on the common path and belongs in a separate ticket (§7.2).
5. **Tangency must become `Touch` + `Situation`**, not a dropped section (§6.4). That is where the
   grazing residue lives.
