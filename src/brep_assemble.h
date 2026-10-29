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

struct TopologyImage {
    int input = -1;
    int local_edge = -1;
    int original_edge = -1;
    int section = -1;
    int output_edge = -1;
    double first = 0;
    double last = 0;
    bool reversed = false;
};

/// Preserve a failed shell's exact geometry and canonical edge ancestry for caller diagnostics.
struct TopologyError : std::runtime_error {
    BRep partial;
    std::vector<BRepOrientation> face_orientations;
    std::vector<TopologyImage> images;

    explicit TopologyError(const std::string& reason) : std::runtime_error(reason) {}
};

/// Sew selected faces into nested closed solids, synchronizing coincident NURBS edge partitions.
BRep assemble(const std::vector<FaceUse>& faces, double tolerance = 1e-7);

}
