#pragma once

#include "curve.h"
#include "surface.h"
#include <utility>
#include <vector>

namespace session_cpp::intersection_analytic {

/// A section curve and its pcurves in the true parameters of both surfaces, all three on the curve's parameter. The curve is a line, circle or ellipse, an exact NURBS arc of degree two for a parabola or hyperbola, or, for the quartic sections (two quadrics in general position, a torus cut by a general plane), a NURBS curve fitted to a tenth of the tolerance, `fit` its measured distance from the two surfaces. A pcurve is a 2D line where the section is a ruling or a circle about the axis, a 2D conic or the exact image of the NURBS on a plane, and otherwise a fitted NURBS; deviation_a and deviation_b are the measured largest distances of the surface point under each pcurve from the curve, to set the edge tolerance from. Pcurves run on across a seam; a circle or ellipse over a pole of a sphere is cut there. tangent where the surfaces touch along the curve.
struct Section {
    Curve curve;
    Curve2d on_a;
    Curve2d on_b;
    bool tangent = false;
    double fit = 0.0;
    double deviation_a = 0.0;
    double deviation_b = 0.0;
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
