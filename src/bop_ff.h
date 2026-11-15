#pragma once

#include "brep.h"
#include <vector>

namespace session_cpp::bop_ff {

struct Curve {
    NurbsCurve curve;
    NurbsCurve pcurve_a, pcurve_b; // same parameter as curve up to an affine map of the domain
    double tolerance = 0.0;
    bool closed = false;
};

struct Point {
    session_cpp::Point point;
    session_cpp::Point uv_a, uv_b;
    double tolerance = 0.0;
};

struct Result {
    bool same_domain = false;
    std::vector<Curve> curves;
    std::vector<Point> points;
};

/// IntTools_FaceFace: sections of two faces' surfaces within fuzzy plus the faces' tolerances
Result intersect(const BRep& a, int face_a, const BRep& b, int face_b, double fuzzy);

}
