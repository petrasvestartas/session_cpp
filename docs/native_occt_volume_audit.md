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
