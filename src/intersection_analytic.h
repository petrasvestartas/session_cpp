#pragma once

#include "curve.h"
#include "surface.h"
#include <utility>
#include <vector>

namespace session_cpp::intersection_analytic {

/// A section curve and its pcurves in the true parameters of both surfaces, all three on the curve's parameter. The curve is a line, circle, ellipse, parabola or hyperbola, or, for the quartic sections (two quadrics in general position, a torus cut by a general plane), a NURBS curve of degree seven with simple knots through true points of the section, by its length, `fit` its measured distance from the two surfaces. A pcurve is a 2D line where the section is a ruling or a circle about the axis, a 2D conic or the exact image of the NURBS on a plane, and otherwise a NURBS curve of degree seven through the true parameters, on the knots of a fitted section; deviation_a and deviation_b are the measured largest distances of the surface point under each pcurve from the curve, to set the edge tolerance from. Fit and deviations are held to a tenth of the tolerance. Pcurves run on across a seam; a section over a pole of a sphere or the apex of a cone is cut there, and its pcurve ends at the pole or apex with the angle it arrives at. tangent where the surfaces touch along the curve.
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

/// done is false where no closed form applies, or where a section measures off its surfaces or pcurves by more than the tolerance, and the pair is left to the marcher; coincident as Surface::coincident (1, -1) where the two are one surface, 1 where a curve lies on the surface.
struct Result {
    bool done = false;
    int coincident = 0;
    std::vector<Section> curves;
    std::vector<Hit> points;
};

/// A pair with a revolution or extrusion is handed to intersection_sweep::surface_surface. IntAna_QuadQuadGeo on the whole surfaces: plane against plane, cylinder, cone, sphere, and torus (planes across or through its axis, and the two Villarceau circles of a plane through the centre that touches the tube twice); sphere pairs; parallel cylinders; equal cylinders with crossing axes; every pair of surfaces of revolution about one axis. Beyond it, IntAna_IntQuadQuad: two of sphere, cylinder and cone in general position, and a torus cut by any plane, swept over one surface's angle; a planar branch comes back as the conic it is, a ruling of both surfaces as a line, the rest fitted. Lines, parabolas and hyperbolas are returned over the parameters where they pass the boxes of both surfaces' domains.
Result surface_surface(const Surface& a, const Surface& b, double tolerance);

/// IntAna_IntConicQuad on the whole curve and surface: a line, circle or ellipse against a plane, cylinder, cone, sphere or torus (done is false for any other kind); t of a conic lies in its first turn from the domain start, uv_a holds the surface parameters.
Result curve_surface(const Curve& curve, const Surface& surface, double tolerance);

}
