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

// A refinement proposal location, never an agreement certificate. Raw locations
// are mapped from the worker's normalized parameters and may be rounded.
struct ParameterFailure {
    enum class Kind { None, EndpointResidual, Partition, PointResidual, Bound, RefinementLimit, WorkLimit };
    Kind kind = Kind::None;
    bool has_interval = false, has_point = false;
    double normalized_lo = 0, normalized_hi = 0, normalized_point = 0;
    std::pair<double,double> curve_interval{}, pcurve_interval{};
    double curve_point = 0, pcurve_point = 0;
};

// Temporary opt-in observation of EXISTING partition work, no new evaluations.
namespace parameter_diagnostics {
struct Partition {
    size_t calls = 0, cuts = 0, world_spans = 0, pcurve_spans = 0;
    bool normalized = false;
};
inline thread_local Partition* active = nullptr;
}

/// Certify normalized-parameter correspondence; mismatched denotes a witnessed gap, unresolved an unsupported proof.
ParameterMatch same_parameter(
    const NurbsCurve& curve,
    const NurbsCurve& pcurve,
    const NurbsSurface& surface,
    double tolerance = 1e-7,
    std::string* diagnostic = nullptr,
    ParameterFailure* failure = nullptr
);

/// Intersect bounded positive-weight NURBS geometry, retaining original parameters and unresolved status.
Result intersect(const NurbsCurve& curve, const NurbsSurface& surface, double tolerance = 1e-7);

}
