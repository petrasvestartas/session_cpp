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

/// The piece of a curve between two parameters exactly as NurbsCurve::trim leaves it, cut from the spans around them and not from the whole curve; false where the whole curve has to be trimmed.
bool trim_piece(const NurbsCurve& curve, double first, double last, NurbsCurve& piece);

}
