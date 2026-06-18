#pragma once

#include "nurbssurface.h"
#include "mesh.h"

namespace session_cpp {

struct RemeshNurbsSurfaceGrid {
    static Mesh from_u_v(const NurbsSurface& s, int max_u, int max_v);
    static Mesh from_u_v_q(const NurbsSurface& s, int max_u, int max_v, double max_angle_deg, double chord_factor);
};

} // namespace session_cpp
