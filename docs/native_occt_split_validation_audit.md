# Native split edge validation and OCCT correspondence prerequisites

Reference: local OCCT `d3056ef80c9668f395da40f5fd7be186cae4501f` at `C:/pc/occt8`. This is a read-only production audit; the parent owns `split.cpp` and will preserve the exact quartic-loop failure triple before correction.

## Native divergence

`src/split.cpp::validate:1414-1437` samples each pcurve, lifts it through its support and always calls `closest(world,p)`. It does not test the known corresponding world parameter first. `closest:239-266` starts with one global sampled minimum, then performs a local Gauss-Newton iteration using `C'(t)?(C(t)-p)/|C'(t)|?`; it has no global-extremum certificate, stationary-point completeness check or original pcurve-parameter constraint. At a loop or stationary endpoint it can settle on another local minimum. A distance from that candidate is an upper bound on the true minimum and therefore cannot prove that the pcurve has left the world curve. Conversely, zero geometric nearest distance does not prove correct parameter correspondence or branch identity.

The construction code already recognizes the relevant mapping: `world_parameter:1132-1144` first tests the affine source-domain parameter before projection. `add_run_edge:1224-1271` trims the world/pcurve pair and reverses the pcurve when required to match edge direction. `add_shared_pcurves:1178-1220` resolves the adjacent pcurve endpoints separately; without a previously established common parameterization, those endpoint matches alone do not prove the interior affine correspondence. Native `BRepEdge` currently has no SameParameter/SameRange flag, so the validator must establish the prerequisite from the actual curves rather than assume it for imported boundaries.

## Exact OCCT caller contract

* `ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Edge.cxx:313-330` rejects an edge whose SameParameter or SameRange flag is false before checking its geometry in a face. Lines356-366 also verify its represented ranges.
* `ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib.cxx:1347-1396` reads SameRange and calls `GeomLib::SameRange` before loading the pcurve over the 3D range. Lines1627-1640 invoke `Approx_SameParameter` when the original representation needs repair. This is a construction/repair prerequisite, not permission to ignore validation failure.
* `BRepLib/BRepLib_ValidateEdge.cxx:115-145` uses direct `C(t)` versus lifted `S(pcurve(t))` comparisons when SameParameter and ranges agree. The alternative path at176-224 initializes both projection adaptors, seeds each query from the corresponding range parameter, checks both directions and marks the operation unresolved when an extremum solver fails.
* `BRepLib_ValidateEdge.cxx:243-251` delegates its exact SameParameter check to `GeomLib_CheckCurveOnSurface`. `ModelingData/TKGeomBase/GeomLib/GeomLib_CheckCurveOnSurface.cxx:78-86` defines its target from the two points at the same parameter; `Derive:119-142` differentiates that same-parameter gap. `Perform:335-420` partitions the comparison and computes its maximum deviation. It does not replace this target with global nearest distance.

## Safe native correction strategy

1. Preserve the exact failed world curve, pcurve, surface, parameter, tolerance and nearest candidate. Compare the expected affine parameter and the projected parameter on that retained input. This distinguishes a projector failure from a construction/mapping error.
2. For every candidate paired edge, invoke the existing native `curve_surface_intersection::same_parameter` with the validator's unchanged residual limit. `Matched` certifies the full normalized parameter interval and safely discharges the edge without repeated projection. The interval certificate is stronger than the previous33 samples.
3. `Mismatched` means that this parameter mapping fails. On an imported edge without established SameParameter, it does not prove geometrical separation. `Unresolved` is also not success. Either preserve an explicit failure or repair/establish the parameter mapping using the original source relation; do not silently accept a nearest-world match as branch identity.
4. If keeping the old approximate validation path temporarily, use the known affine parameter as a candidate before local projection and retain the smaller observed residual. This prevents a bad projector from overturning a directly observed good correspondence, but it remains sample-based and must not be presented as full-edge certification.

No tolerance increase, sample-count increase or projection budget change is justified by this failure. Exact runtime diagnosis and the parent-owned correction remain pending.

## Exact quartic diagnosis and correction

The compiled diagnostic reproduced the failure at pcurve parameter0.25781250000000006. The global-projector candidate is0.03675240611537944 away, while the affine corresponding world point is only1.2684298056342413e-14 away. The existing whole-interval SameParameter certificate returns Matched at the unchanged8e-7 validation limit. The full triple is retained in tests/fixtures/two_loops_split_correspondence_failure.json. This establishes a false rejection by the approximate projector, not an off-surface edge.

The native validator now requires the full normalized-parameter certificate for each nondegenerate edge/pcurve pair. Mismatched and Unresolved remain explicit failures with the preserved geometry and certificate explanation. The33 direct paired samples retained only on failure produce a diagnostic witness; they do not grant acceptance. No tolerance, curve geometry, knot or topology acceptance threshold changes.
