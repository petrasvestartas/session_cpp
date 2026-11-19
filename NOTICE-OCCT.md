# Open CASCADE Technology derivatives

These files follow the structure and algorithms of Open CASCADE Technology, src/ModelingAlgorithms/TKBO.

Copyright (c) 1999-2026 OPEN CASCADE SAS. Licensed under the GNU Lesser General Public License version 2.1 with the Open CASCADE exception.

- src/bop_ds.h, src/bop_ds.cpp: BOPDS_DS, BOPDS_Pave, BOPDS_PaveBlock, BOPDS_CommonBlock, BOPDS_FaceInfo, BOPDS_Curve; BOPTools_AlgoTools::MakeVertex
- src/bop_filler.cpp: BOPAlgo_PaveFiller, BOPAlgo_PaveFiller_1 to _6 (PerformVV, PerformVE, PerformEE, PerformVF, PerformEF, PerformFF, MakeBlocks), IntTools_EdgeEdge, IntTools_EdgeFace
- src/bop_ff.h, src/bop_ff.cpp: IntTools_FaceFace
- src/bop_builder_face.h, src/bop_builder_face.cpp: BOPAlgo_BuilderFace, BOPAlgo_WireSplitter
- src/bop_builder_solid.h, src/bop_builder_solid.cpp: BOPAlgo_BuilderSolid, BOPAlgo_ShellSplitter, BOPTools_AlgoTools::GetFaceOff
- src/bop.h, src/bop.cpp: BOPAlgo_Builder (FillImagesFaces, FillSameDomainFaces), BOPAlgo_BOP (BuildRC, BuildShape), BOPTools_AlgoTools::ComputeState
