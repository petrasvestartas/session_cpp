#pragma once

#include "brep.h"
#include <string>
#include <vector>

namespace session_cpp::brep_section {

struct CurveHit {
    Point point;
    int face;
    double t;
    double u;
    double v;
    bool tangent;
};

struct CurveSpan {
    int face;
    double first;
    double last;
    NurbsCurve pcurve;
};

struct CurveResult {
    std::vector<CurveHit> points;
    std::vector<CurveSpan> overlaps;
    bool complete = true;
    std::string failure;
};

struct FaceCurve {
    int first_face;
    int second_face;
    NurbsCurve world;
    NurbsCurve first_pcurve;
    NurbsCurve second_pcurve;
};

/// Established support coincidence; trim-area overlap still requires partitioning and classification.
struct FaceOverlap {
    int first_face;
    int second_face;
};

struct FaceResult {
    std::vector<FaceCurve> curves;
    std::vector<FaceOverlap> overlaps;
    bool complete = true;
    std::string failure;
};

CurveResult curve(const NurbsCurve& curve, const BRep& brep, double tolerance);

FaceResult faces(const BRep& first, const BRep& second, double tolerance);

}
