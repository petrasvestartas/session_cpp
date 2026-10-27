#pragma once

#include "nurbscurve.h"
#include "nurbssurface.h"

namespace session_cpp::nurbs_offset {

/// Approximate a signed curve offset, preserving parameters and checking interior residuals adaptively.
NurbsCurve curve(
    const NurbsCurve& source,
    double distance,
    const Vector& normal,
    double tolerance
);

/// Approximate a signed surface offset, rejecting ambiguous normals and unresolved sampled residuals.
NurbsSurface surface(const NurbsSurface& source, double distance, double tolerance);

}
