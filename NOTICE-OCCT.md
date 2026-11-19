# Open CASCADE Technology derivatives

These files follow the structure and algorithms of Open CASCADE Technology, src/ModelingAlgorithms/TKBO.

Copyright (c) 1999-2026 OPEN CASCADE SAS. Licensed under the GNU Lesser General Public License version 2.1 with the Open CASCADE exception.

- src/bop_ds.h, src/bop_ds.cpp: BOPDS_DS, BOPDS_Pave, BOPDS_PaveBlock, BOPDS_CommonBlock, BOPDS_FaceInfo, BOPDS_Curve; BOPTools_AlgoTools::MakeVertex
- src/bop_filler.cpp: BOPAlgo_PaveFiller, BOPAlgo_PaveFiller_1 to _6 (PerformVV, PerformVE, PerformEE, PerformVF, PerformEF, PerformFF, MakeBlocks), IntTools_EdgeEdge, IntTools_EdgeFace
- src/bop_ff.h, src/bop_ff.cpp: IntTools_FaceFace (Perform, PerformPlanes, ClassifyLin2d, CorrectSurfaceBoundaries, restriction lines in MakeCurve, ComputeTolReached3d, TangentFaces), IntTools_Tools::ComputeTolerance, BRepTools::UVBounds, GeomLib_CheckCurveOnSurface, IntPatch_Intersection and IntPatch_PrmPrmIntersection (walking lines from seeds, tangent faces), IntWalk_PWalking (the march and its step control), IntPolyh (seeds from two grids), GeomInt_WLApprox (curve and pcurves on one parameter)
- src/bop_builder_face.h, src/bop_builder_face.cpp: BOPAlgo_BuilderFace, BOPAlgo_WireSplitter
- src/bop_builder_solid.h, src/bop_builder_solid.cpp: BOPAlgo_BuilderSolid, BOPAlgo_ShellSplitter, BOPTools_AlgoTools::GetFaceOff
- src/bop.h, src/bop.cpp: BOPAlgo_Builder (FillImagesFaces, FillSameDomainFaces), BOPAlgo_BOP (BuildRC, BuildShape), BOPTools_AlgoTools::ComputeState
src/intersection_analytic.h, src/intersection_analytic.cpp: IntAna_QuadQuadGeo, IntAna_IntConicQuad, IntAna_Quadric, ElSLib, ElCLib
- src/curve.h, src/curve.cpp (parabola, hyperbola): Geom_Parabola, Geom_Hyperbola, ElCLib (ParabolaValue, HyperbolaValue, their derivatives and parameters)
- src/surface.h, src/surface.cpp, src/surface_sweep.cpp (revolution, extrusion): Geom_SurfaceOfRevolution, Geom_SurfaceOfLinearExtrusion (parametrisation, derivatives, periods)
- src/intersection_sweep.h, src/intersection_sweep.cpp: IntAna2d_AnaIntersection (its cases _1 to _8: a parametrised line or conic in the equation of the other), IntAna2d_Conic, math_DirectPolynomialRoots
