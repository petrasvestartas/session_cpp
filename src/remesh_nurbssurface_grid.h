#pragma once

#include "nurbssurface.h"
#include "mesh.h"

namespace session_cpp {

struct RemeshNurbsSurfaceGrid {
    static Mesh from_u_v(const NurbsSurface& s, int max_u, int max_v);
    /// Grid tessellation: angular tolerance in degrees, chord tolerance relative to the bbox.
    /// Finite unit surface normals follow winding; poles use area-weighted adjacent normals
    /// in face-key order. Zero/non-finite estimates fall back to +Z, without a world-unit gate.
    static Mesh from_u_v_q(const NurbsSurface& s, int max_u, int max_v, double max_angle_deg, double chord_factor);
private:
    friend class NurbsSurfaceTrimmed;
    static void split_crease_normals(const NurbsSurface& s, Mesh& mesh);
};

} // namespace session_cpp
