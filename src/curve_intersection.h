#pragma once

#include "nurbscurve.h"
#include <string>
#include <vector>

namespace session_cpp::curve_intersection {

struct Hit {
    Point point;
    double a;
    double b;
    bool tangent;
};

struct Overlap {
    double a0;
    double a1;
    double b0;
    double b1;
};

struct Result {
    std::vector<Hit> points;
    std::vector<Overlap> overlaps;
    bool complete = true;
    std::string failure;
};

/// Intersect bounded positive rational curves, retaining both parameters and oriented overlap intervals.
Result intersect(const NurbsCurve& a, const NurbsCurve& b, double tolerance = 1e-7);

}
