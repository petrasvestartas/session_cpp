#pragma once

#include "curve.h"
#include "surface.h"
#include <utility>
#include <vector>

namespace session_cpp::intersection_analytic {

/// A section curve and its pcurves in the true parameters of both surfaces, all three on the curve's parameter: a line, circle or ellipse, a parabola or hyperbola arc as an exact NURBS curve of degree two. A pcurve is a 2D line where the section is a ruling or a circle about the axis, a 2D conic on a plane, the exact image of a NURBS section on a plane, and otherwise interpolated to a hundredth of the tolerance; it runs on across a seam. tangent where the surfaces touch along the curve.
struct Section {
    Curve curve;
    Curve2d on_a;
    Curve2d on_b;
    bool tangent = false;
};

/// An isolated point: where surfaces touch (tangent) or meet in a point (a cone's apex), or where a curve meets a surface at its parameter t (tangent where it only touches).
struct Hit {
    Point point;
    double t = 0.0;
    std::pair<double, double> uv_a;
    std::pair<double, double> uv_b;
    bool tangent = false;
};

/// done is false where no closed form applies and the pair is left to the marcher; coincident as Surface::coincident (1, -1) where the two are one surface, 1 where a curve lies on the surface.
struct Result {
    bool done = false;
    int coincident = 0;
    std::vector<Section> curves;
    std::vector<Hit> points;
};

/// IntAna_QuadQuadGeo on the whole surfaces: plane against plane, cylinder, cone, sphere, and torus (planes across or through its axis); sphere pairs; parallel cylinders; equal cylinders with crossing axes; every pair of surfaces of revolution about one axis. Lines, parabolas and hyperbolas are returned over the parameters where they pass the boxes of both surfaces' domains.
Result surface_surface(const Surface& a, const Surface& b, double tolerance);

/// IntAna_IntConicQuad on the whole curve and surface: a line, circle or ellipse against a plane, cylinder, cone, sphere or torus; t of a conic lies in its first turn from the domain start, uv_a holds the surface parameters.
Result curve_surface(const Curve& curve, const Surface& surface, double tolerance);

}
