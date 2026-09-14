#pragma once

#include "nurbssurface.h"
#include "mesh.h"

namespace session_cpp {

/// Grid mesh of a NURBS surface: spans split by normal turn and chord height, poles fanned, seams closed
struct RemeshNurbsSurfaceGrid {
    /// Grid at 20 degrees and 0.5 percent of the bbox diagonal; max_u and max_v fix the parameter counts when positive
    static Mesh from_u_v(const NurbsSurface& s, int max_u, int max_v);

    /// Grid with the normal turn per subdivision capped at max_angle_deg and the chord height at chord_factor of the bbox diagonal; vertex normals are unit surface normals on the fan side, fan normals at poles
    static Mesh from_u_v_q(const NurbsSurface& s, int max_u, int max_v, double max_angle_deg, double chord_factor);

private:
    friend class NurbsSurfaceTrimmed;

    /// Split shading vertices at internal C0 knots whose one-sided normals disagree
    static void split_crease_normals(const NurbsSurface& s, Mesh& mesh);
};

} // namespace session_cpp
