#pragma once

#include "nurbssurface.h"
#include "mesh.h"

namespace session_cpp {

struct RemeshNurbsSurfaceGrid {
    static Mesh from_u_v(const NurbsSurface& s, int max_u, int max_v);
};

} // namespace session_cpp
