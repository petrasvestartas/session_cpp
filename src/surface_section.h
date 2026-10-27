#pragma once

#include "nurbssurface.h"
#include "plane.h"
#include <optional>
#include <utility>
#include <vector>

namespace session_cpp::surface_section {

/// Return exact paired extrusion/plane sections; nullopt denotes unsupported geometry or numerical conditioning.
std::optional<std::vector<std::pair<NurbsCurve, NurbsCurve>>> extrusion_plane(
    const NurbsSurface& surface,
    const Plane& plane,
    double tolerance = 1e-7
);

}
