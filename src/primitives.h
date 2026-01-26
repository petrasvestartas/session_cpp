#pragma once

#include "nurbscurve.h"
#include "point.h"

namespace session_cpp {

class Primitives {
public:
    /// Create a circle as a rational NURBS curve (9 control points)
    static NurbsCurve circle(double cx, double cy, double cz, double radius);

    /// Create an ellipse as a rational NURBS curve
    static NurbsCurve ellipse(double cx, double cy, double cz, double major_radius, double minor_radius);

    /// Create an arc through three points as a rational NURBS curve
    static NurbsCurve arc(const Point& start, const Point& mid, const Point& end);

    /// Create a parabola through three points
    static NurbsCurve parabola(const Point& p0, const Point& p1, const Point& p2);

    /// Create a hyperbola segment
    static NurbsCurve hyperbola(const Point& center, double a, double b, double extent);

    /// Create a spiral (helix with varying radius)
    static NurbsCurve spiral(double start_radius, double end_radius, double pitch, double turns);
};

} // namespace session_cpp
