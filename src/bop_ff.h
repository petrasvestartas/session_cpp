#pragma once

#include "brep.h"
#include <array>
#include <vector>

namespace session_cpp::bop_ff {

/// IntTools_Curve: a section curve and its pcurves on both faces over one parameter range; tolerance bounds its distance to both surfaces.
struct Curve {
    NurbsCurve curve;
    NurbsCurve pcurve_a;
    NurbsCurve pcurve_b;
    double tolerance = 0.0;
    bool closed = false;
    std::array<bool, 2> bound{}; // its start, its end: a vertex of the section whatever else meets there (the origin of an ellipse two cylinders share)
};

/// IntTools_PntOn2Faces: an isolated tangent point, its (u, v, 0) on both faces and the gap between the surfaces there.
struct Point {
    session_cpp::Point point;
    session_cpp::Point uv_a;
    session_cpp::Point uv_b;
    double tolerance = 0.0;
};

/// IntTools_FaceFace: same_domain (TangentFaces) leaves curves and points empty; certified where the curves are exact by construction (two planes).
struct Result {
    bool same_domain = false;
    bool certified = false;
    std::vector<Curve> curves;
    std::vector<Point> points;
};

/// IntTools_FaceFace::Perform: sections of the two supporting surfaces over the boxes of the faces' pcurves, within the face tolerances plus fuzzy: coincident faces are same_domain, tangent ones give points or curves, every curve carries the tolerance that bounds its distance to both surfaces. Never throws.
Result intersect(const BRep& a, int face_a, const BRep& b, int face_b, double fuzzy);

}
