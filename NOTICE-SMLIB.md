# SMLib derivatives

These files follow algorithms of SMLib as published in NVIDIA's usd-brep (https://github.com/NVIDIA-Omniverse/usd-brep, source/SMLib), written anew in session_cpp's own types; no SMLib source, header or library is copied, included or linked.

Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. Licensed under the Apache License, Version 2.0.

- src/bop_ff.cpp (`branches`: crossing, tangent and coincident contact told apart by the second fundamental forms): source/SMLib/src/SmSurfaceIntersector.cpp (sm_ComputeCurveType; the point types SM_IP_TANGENT_POINT, SM_IP_TANGENT_CURVE, SM_IP_COINCIDENCE)
