#pragma once

#include "brep.h"

namespace session_cpp { struct SectionResult; }

namespace session_cpp::occt {

enum class BooleanOperation { Union, Intersection, Difference, Xor };

BRep boolean_operation(
    const BRep& first,
    const BRep& second,
    BooleanOperation operation,
    double tolerance
);

BRep offset(const BRep& shape, double distance, double tolerance);

BRep hollow(const BRep& shape, const std::vector<int>& openings, double thickness, double tolerance);

NurbsCurve offset(const NurbsCurve& curve, double distance, const Vector& normal, double tolerance);

NurbsSurface offset(const NurbsSurface& surface, double distance, double tolerance);

double volume(const BRep& shape, double tolerance);

Mesh tessellate(const BRep& shape, double deflection, double angle);

SectionResult section(const NurbsCurve& a, const NurbsCurve& b, double tolerance);

SectionResult section(const NurbsCurve& curve, const NurbsSurface& surface, double tolerance);

SectionResult section(const NurbsCurve& curve, const BRep& brep, double tolerance);

SectionResult section(const BRep& a, const BRep& b, double tolerance);

std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>> surface_surface(
    const NurbsSurface& a,
    const NurbsSurface& b,
    double tolerance
);

}
