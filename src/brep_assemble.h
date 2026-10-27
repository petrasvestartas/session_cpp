#pragma once

#include "brep.h"
#include "curve_surface_intersection.h"
#include <stdexcept>
#include <vector>

namespace session_cpp::split {
struct EdgePiece;
}

namespace session_cpp::brep_assemble {

struct FaceUse {
    const BRep* brep = nullptr;
    int face = -1;
    bool reversed = false;
    const std::vector<split::EdgePiece>* pieces = nullptr;
    int operand = -1;
};

/// Retain the exact failed correspondence for caller diagnostics without library filesystem side effects.
struct ParameterError : std::runtime_error {
    NurbsCurve curve;
    NurbsCurve pcurve;
    NurbsSurface surface;
    double tolerance;
    curve_surface_intersection::ParameterMatch match;

    ParameterError(
        const std::string& reason,
        const NurbsCurve& world,
        const NurbsCurve& uv,
        const NurbsSurface& support,
        double distance,
        curve_surface_intersection::ParameterMatch status
    );
};

/// Sew selected faces into nested closed solids, synchronizing coincident NURBS edge partitions.
BRep assemble(const std::vector<FaceUse>& faces, double tolerance = 1e-7);

}
