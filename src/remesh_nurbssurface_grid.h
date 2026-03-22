#pragma once

#include "nurbssurface.h"
#include "mesh.h"

namespace session_cpp {

Mesh remesh_nurbssurface_grid(const NurbsSurface& s, int max_u, int max_v);

struct RemeshNurbsSurfaceGrid {
    static Mesh from_u_v(const NurbsSurface& s, int max_u, int max_v);
};

} // namespace session_cpp
