#pragma once

#include "nurbscurve.h"
#include "nurbssurface.h"
#include <string>
#include <vector>

namespace session_cpp::curve_surface_intersection {

struct Hit {
    Point point;
    double t;
    double u;
    double v;
    bool tangent;
};

struct Overlap {
    double t0;
    double t1;
    NurbsCurve pcurve;
};

struct Result {
    std::vector<Hit> points;
    std::vector<Overlap> overlaps;
    bool complete = true;
    std::string failure;
};

enum class ParameterMatch { Matched, Mismatched, Unresolved };

/// Certify normalized-parameter correspondence; mismatched denotes a witnessed gap, unresolved an unsupported proof.
ParameterMatch same_parameter(
    const NurbsCurve& curve,
    const NurbsCurve& pcurve,
    const NurbsSurface& surface,
    double tolerance = 1e-7,
    std::string* diagnostic = nullptr
);

/// Intersect bounded positive-weight NURBS geometry, retaining original parameters and unresolved status.
Result intersect(const NurbsCurve& curve, const NurbsSurface& surface, double tolerance = 1e-7);

}
