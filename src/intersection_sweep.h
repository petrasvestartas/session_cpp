#pragma once

#include "intersection_analytic.h"
#include <vector>

namespace session_cpp::intersection_sweep {

/// Where two curves of one plane meet: the point, the parameter of each there, tangent where they touch without crossing.
struct Hit2d {
    Point point;
    double a = 0.0;
    double b = 0.0;
    bool tangent = false;
};

/// done is false where the numeric route gave up; coincident where the curves run along each other.
struct Result2d {
    bool done = false;
    bool coincident = false;
    std::vector<Hit2d> points;
};

/// IntAna2d_AnaIntersection on the whole curves: lines, circles, ellipses, parabolas and hyperbola branches against each other in closed form (a line against a conic by a quadratic, two conics by the quartic of one's parametrisation in the other's equation), parameters of a circle or ellipse in the first turn from the domain start. A NURBS curve is intersected numerically over its domain, with an open analytic partner over the stretch beside it.
Result2d curve_curve(const Curve2d& a, const Curve2d& b, double tolerance);

/// The closed forms of a revolution or extrusion, as intersection_analytic::surface_surface, which hands such pairs here:
/// a revolution against a plane, cylinder, cone, sphere, torus or revolution about its axis, in circles where the meridians meet (the profile in a plane of the axis; any profile against a plane across the axis); against a plane through its axis, in the profile and its mirror image;
/// an extrusion against a plane across its direction, in the profile's image along the direction; against a plane, cylinder or extrusion along its direction, in lines where the profiles seen along it meet.
/// Whole analytic profiles are intersected, a NURBS profile over its domain. Pcurves are 2D lines for circles about an axis, lines along a direction and profile copies, the conic or NURBS image on a plane, an exact arc of degree 2 under a parabola's image on its extrusion, and otherwise fitted, with the measured deviation. done is false for every other pair.
intersection_analytic::Result surface_surface(const Surface& a, const Surface& b, double tolerance);

}
