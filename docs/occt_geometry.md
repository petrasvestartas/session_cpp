# C++ geometry operations

The requested implementation is now tracked in [the native translation plan](native_brep_plan.md).
The OCCT integration below is retained as reference infrastructure. Its passing
Boolean tests do not count toward native session Boolean acceptance.

The optional OCCT backend converts the session topology and NURBS geometry into
OCCT, computes the operation, and converts the result back into ordinary session
vertices, edges, pcurves, wires, faces, shells, and solids. Results do not store
opaque OCCT handles and can use the existing JSON and protobuf serialization.
This is a dependency integration, not a native port of OCCT's implementation.

## Build and reproduce

For an installed OCCT package:

```powershell
cmake -S session_cpp -B session_cpp/build -DSESSION_WITH_OCCT=ON -DOpenCASCADE_DIR=<package-directory>
cmake --build session_cpp/build --config Release --target point_minitest geometry_evidence --parallel 6
Set-Location session_cpp
./build/Release/point_minitest.exe
./build/Release/geometry_evidence.exe
python examples/geometry_gallery.py
```

For an existing static OCCT build, provide `SESSION_OCCT_INCLUDE_DIR` and
`SESSION_OCCT_LIBRARY_DIR` instead of `OpenCASCADE_DIR`. The local verification
uses OCCT source revision `d3056ef80c9668f395da40f5fd7be186cae4501f`, headers at
`C:/pc/occt8/build/inc`, and libraries at `C:/pc/occt8/build/win64/vc14/lib`.
The gallery script requires NumPy and Matplotlib.
The optional `python examples/occt_step_reference.py` comparison additionally
uses the installed OCP binding. The local reference version is 7.9.3.1; it reads
the STEP file directly and does not call the session importer or adapter.
`geometry_evidence` exports separate XOR components and records outward-offset
probes. It returns nonzero while those probes fail; their outcomes are reported
separately from passing unit tests.

`SESSION_WITH_OCCT` defaults to `OFF`. The added OCCT-dependent APIs throw a
clear exception in that configuration; their conditional tests run only with
the option enabled. Native topology, curve-plane, and convex-offset regressions
also run without OCCT. Rust and Python have not been ported while C++ validation
is still in progress.

## Operations

| API | Behavior |
| --- | --- |
| `BRep::boolean_union/intersection/difference/xor` | Solid Booleans with shared topology and NURBS results |
| `BRep::offset_convex` | Native planar convex miter offset; empty after complete erosion |
| `BRep::offset` | OCCT shape offset with rounded joins |
| `BRep::hollow` | Signed wall thickness and explicit face-index openings |
| `NurbsCurve::offset` | Signed offset along tangent cross reference normal, approximated as NURBS |
| `NurbsSurface::offset` | Normal offset approximated as NURBS |
| `Intersection::curve_curve/curve_surface/curve_brep/brep_brep` | Isolated points and overlap/section curves |
| `Intersection::surface_surface` | Analytic paths where recognized; OCCT freeform intersections when enabled |
| `BRep::volume_exact` | Numerical surface integration on the BRep geometry, independent of tessellation |
| `BRep::tessellate` | Trim-aware OCCT tessellation with chord deflection and angular deflection in radians |

Solid Boolean results are regularized: touching faces, edges, or vertices alone
produce an empty solid intersection. Use section APIs to retrieve boundary-only
intersections. XOR computes both directional differences and their union.

OCCT tolerances are geometric tolerances, including input topology tolerances
and the Boolean fuzzy tolerance. They are not a guarantee that every output
edge has exactly the requested tolerance. The chair regression records output
edge tolerances and independently samples edge/pcurve agreement.

Curve and surface offset conversion checks OCCT's reported approximation error.
General offsets can fail at singularities, self-intersections, or topology
changes; failed or invalid results raise an exception. Arbitrary self-intersection
removal and the entire OCCT regression corpus are not claimed as supported.
Planar wire offsets and sweep construction are not implemented by this change.

Both supplied chairs have inward-offset regressions at distance -0.01. The
outward offset at +0.01 remains unsupported: the C++ OCCT 8 backend returns no
shape, and the independent OCCT 7.9.3.1 STEP path produces an invalid result.
These outcomes are failures, not passing offset cases.

## Evidence

`tests/fixtures/solidbrep.stp` is the unmodified user-supplied Rhino file containing
two closed, doubly curved chair solids. Its SHA-256 is
`6cb6b9d1723a0570fd72e1eb8ecedf109a750a468f5f8e771a624ed4224aac19`.

The gallery is written to `build/geometry-tests/index.html`. It includes original
geometry, images, assertion lists, and a coverage manifest. The chair comparison
uses a shared scale and viewpoint, and its report records volumes, topology
counts, curved-face counts, and edge/surface gaps. Controlled-deflection meshes
and native tessellations are saved separately.

`occt_boolean_boxes.json` contains 38 distinct input configurations extracted
from 152 OCCT `*_simple` cases. The C++ test checks all five solid operations
against analytic box-overlap volumes. The source-case names are retained;
this does not claim to reproduce OCCT's lower-dimensional area assertions or
the complete upstream test corpus. Rotated pairs of all five Platonic solids
have separate Boolean regressions.

`occt_step_reference.json` compares all five chair Boolean volumes and solid
counts with a separate OCCT STEP import, and records both inward and outward
offset attempts. This complements the volume identities checked by C++.

The optional standalone `examples/occt_reference` executable compares the native
convex offsets with independently built OCCT solids. The disappearing-bevel case
currently records an invalid OCCT reference result and a volume discrepancy;
that failure is retained in the comparison report. The native result has a
separate analytic unit-cube check. A reference discrepancy is not counted as a
passing comparison.

## References

- [OCCT source](https://github.com/Open-Cascade-SAS/OCCT)
- [Boolean operation specification](https://occt3d.com/dev/doc/overview/html/specification__boolean_operations.html)
- [Offset limitations](https://dev.opencascade.org/doc/refman/html/class_b_rep_offset_a_p_i___make_offset_shape.html)
- [OCCT licensing and linking exception](https://github.com/Open-Cascade-SAS/OCCT/blob/master/OCCT_LGPL_EXCEPTION.txt)

Distributing binaries linked with OCCT also requires satisfying the license of
the OCCT build being distributed; this repository does not vendor OCCT sources
or libraries.
