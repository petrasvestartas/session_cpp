# Native volume failure and OCCT comparison

Reference: OCCT `d3056ef80c9668f395da40f5fd7be186cae4501f`, local `C:/pc/occt8`.

The first integrated native run passed analytical primitives and both original
chair volumes. It failed `Translation Scale And Inactive Geometry` immediately
on a sphere of radius 0.002 translated to `(1e5,-2e5,3e5)`: quadrature reported
nonconvergence. The input and assertion remain in `brep_measure_test.cpp`;
`build/native-interference-integrated.log` preserves the first failure.

Source trace:

- `ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp.cxx:419` obtains a rough
  barycenter as the integration reference point.
- `BRepGProp_Gauss.cxx:298-322` evaluates the divergence-theorem mass integrand
  using coordinates relative to that point and the oriented surface normal.
- `BRepGProp_Gauss.cxx:600-680` partitions integration at surface/trim knots and
  selects paired quadrature orders; later loops refine estimated integration error.
- `BRepGProp_Face.cxx:201-209` obtains the normal from `D1U.Crossed(D1V)` and
  reverses it for a reversed face.
- `ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_TransformedSurface.cxx:248-259`
  evaluates derivatives on the underlying surface first, then transforms point
  and derivative vectors separately. A placement translation does not enter
  the derivative arithmetic.

Native divergence: the session transform bakes translation into homogeneous
control points. Our first volume implementation subtracted the integration
origin only after evaluating the rational surface derivatives in world
coordinates. The quotient derivative calculation therefore subtracted large
translated terms to recover a tiny derivative. Repeated quadrature subdivision
cannot remove this evaluation noise.

Correction: evaluate a private surface copy in the integration frame. Subtract
`origin * weight` from homogeneous coordinates with fused multiply-add before
evaluation, then divide by the characteristic length. This preserves original
weights and parameters, keeps source geometry immutable, and scales the final
volume by the length cubed. The mathematical integrand remains the same; the
derivative computation no longer carries the placement translation.

This is a numerical adaptation to session's baked-coordinate representation,
not a claim that OCCT normalizes its control nets in this way. The original
translation/scale regression must pass unchanged. Existing primitive and chair
comparisons must also pass after the correction. Stored input coordinates still
limit attainable accuracy; normalization cannot recover geometry already lost
to floating-point representation.

An empty result is accepted as volume zero before checking `BRep::is_valid`,
whose session contract deliberately rejects faceless shapes. This matches the
empty integration domain and preserves the existing BRep validity API.


## 2026-10-30: optional polynomial cache refusal

After the native dense-span intersection repair, Curved Retained Walls builds
and passes topology, vertex-link and complete pcurve checks. Its volume call
fails on face 6 with total_work=2663 and polynomial_coefficients=120, before
adaptive quadrature begins. The error is the polynomial Primitive constructor's
preflight: a 67 x 67 cubic support has 64 x 64 knot cells and an estimated
2064384 coefficient operations. This exceeds the remaining shared budget; it
is not evidence that actual quadrature exhausted 200000 operations.

OCCT BRepGProp's Gauss integration separates surface U knot intervals, boundary
parameter intervals, and high/low quadrature rules. This supports keeping
optional acceleration separate from the adaptive integration contract; it does
not justify copying sampled error estimates as certified native bounds.
The reviewed draft selects the existing adaptive path when optional full-net
precomputation is unaffordable. It preserves the Primitive constructor's direct
budget-refusal contract, actual integration budget, tolerance and depth limits.
A dense trimmed planar box provides an independent signed-volume oracle of
+24/-24. Neither this draft nor the actual Hollow volume is qualified yet.

The next required evidence is a checked complete result capture before volume
measurement, followed by direct replay. Dense boundary curves may legitimately
exhaust the unchanged adaptive budget even with this policy correction. If so,
measure actual visited spans before implementing lazy, trim-local primitive
caching; omitted cells must never be treated as zero contributions.
Drafts and source audit: `C:/pc/tmp/session-migration-20261028/hollow-volume-cache-preflight-v1`.


### 2026-10-30: actual dense hollow after optional-cache fallback

The saved native result `hollow-result-before-volume-20261030-0226/curved-retained.brep.json`
(SHA256 58e666592881a7f9319b01f99c832f4b708c7dcb2d0226126034a0f96ea5636e)
now reaches adaptive integration instead of rejecting optional cache allocation.
The unchanged 1e-7 volume replay fails at face 6 with total_work=200001,
polynomial_coefficients=120, face_inner=197041 and face_outer=297. Both
subdivision counters and maximum depths are zero. The runtime is 7.6201664s.
Evidence: `hollow-measure-fallback-20261030-0234-report.json`, external archive.

This is repeated initial prefix integration, not nonconvergence requiring a
looser accuracy target. OCCT revision d3056ef80c9668f395da40f5fd7be186cae4501f,
BRepGProp_Gauss.cxx:298-348 explicitly supports both point flux and plane flux.
Its plane contribution is signed distance times the normal projected on one
fixed plane normal. A native plane-flux implementation must change every
face path consistently, including polynomial rectangles and rational adaptive
supports, and preserve oriented shells/cavities. OCCT adaptive integration
retains unaffected interval contributions; the proposed Bernstein coefficient
reuse is an independent native derivation, not a claim that OCCT implements it.

Two independent source audits prove adjacent derivative coefficients can be
reused only across the same continuous derivative trace and identical Bernstein
basis interval. Similar floating-point values are insufficient. Trim footprint
restriction and a changed integration anchor also need a valid closed-wire
gauge argument, including endpoint gaps. These are implementation review
conditions, not accepted results. The unchanged measure group passes 13/13
on build wip-build-20261030-0228-hollow-repairs, including the new signed
dense-support cache-fallback regression. The hard hollow remains unaccepted.


### Dense rational lens: inner quadrature order, 2026-10-30

The exact reassembled unit-sphere lens exhausted the unchanged 200,000-work
budget: face 2 had 95,367 inner visits, 890 outer visits, 43,066 inner
subdivisions and depth only one. Dense cubic boundary pcurves retain 480 spans.
OCCT BRepGProp_VinertGK.cxx:379 and BRepGProp_TFunction.cxx:133 select
integration order per span and separate inner integration from outer integration.
The native repair tries 32-point Gauss integration after the existing 16/8
estimates disagree, charging the extra pass before evaluation. It retains the
same tolerance, finite checks, 200,000 budget and bisection/depth fallback.
This is an adaptive error estimator, not a rigorous interval error certificate.

`lens-volume-higher-order-v1` passes all 16 Measure tests. The original
`orientation-proposal-volume-green-v1` regression passes all four rebuilt
lens variants, signed analytic volume, occupancy, topology and paired pcurves.
Both use build b3c381328b9bb821d31c4a34a76d132478c5ecfcc401990f06a9e9624f1456fc.
Reported minitest times exclude successful assertion evaluation; use wrapper
wall_seconds for timing comparisons. Nonlinear pcurve crossings of surface
knots remain a separate general-integration issue requiring qualification.


### Sphere XOR: exact repeated primitive requests, 2026-10-30

The retained five-face XOR now constructs two solids. Both its strict assembly
regression and the public curved-seams test fail volume at work 200001 on face 3,
with 23597 face-inner visits and no inner subdivisions. The captured fixture
contains four byte-identical pairs of 480-span pcurves on shared supports.

OCCT BRepGProp_TFunction.cxx:55-166 integrates the support prefix at each boundary
sample and propagates the inner error through the absolute boundary derivative.
No cross-boundary cache was found there. The native exact-key memo is therefore
an independent optimization, not a translated OCCT cache. Each sum_volume pass
owns a fresh cache keyed by support index and the exact bits of u, v and the
actual inner tolerance; its immutable normalization frame is fixed for that pass.
All oriented outer contributions and their error allowances remain unchanged.
Only finite completed requests with charged inner work are stored, bounding
storage by the existing work budget. Tolerances and work limits are unchanged.

The candidate and physical signed/component-volume regression were independently
source-reviewed and applied. Runtime qualification is pending. Source proof,
fixture identity audit and promotion hashes are preserved under the external
volume-exact-primitive-memo-v1 and xor-volume-reuse-audit-v1 directories.
