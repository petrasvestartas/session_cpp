#include "intersection_sweep.h"
#include "mini_test.h"
#include <cmath>

using namespace session_cpp;
using namespace session_cpp::intersection_analytic;
using namespace session_cpp::mini_test;

namespace {

using Kind = Surface::Kind;
using Shape = Curve::Kind;
constexpr double pi = 3.14159265358979323846;
const Point O(0, 0, 0);
const Vector X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);

bool kinds(const Result& r, std::initializer_list<Shape> expected, size_t points = 0) {
    if (!r.done || r.coincident || r.curves.size() != expected.size() || r.points.size() != points)
        return false;
    size_t i = 0;
    for (Shape k : expected)
        if (r.curves[i++].curve.kind() != k)
            return false;
    return true;
}

// The largest distance from a section to the surface points under its pcurves and of a point from its two surface points.
double worst(const Surface& a, const Surface& b, const Result& r) {
    double w = 0.0;
    for (const Section& s : r.curves) {
        for (int i = 0; i <= 24; ++i) {
            const double t = s.curve.domain().first + (s.curve.domain().second - s.curve.domain().first) * i / 24;
            const Point p = s.curve.point_at(t), ua = s.on_a.point_at(t), ub = s.on_b.point_at(t);
            w = std::max({w, a.point_at(ua[0], ua[1]).distance(p), b.point_at(ub[0], ub[1]).distance(p)});
        }
        w = std::max({w, s.deviation_a, s.deviation_b});
    }
    for (const Hit& h : r.points)
        w = std::max({w, a.point_at(h.uv_a.first, h.uv_a.second).distance(h.point), b.point_at(h.uv_b.first, h.uv_b.second).distance(h.point)});
    return w;
}

// Both orders, here and moved rigidly: the same kinds in the same number, every pcurve on its section.
bool both(const Surface& a, const Surface& b, std::initializer_list<Shape> expected, size_t points = 0, double within = 1e-9) {
    const Xform move = Xform::translation(1.0, -4.0, 2.0) * Xform::rotation_z(0.8) * Xform::rotation_x(-0.3);
    const Surface ma = a.transformed(move), mb = b.transformed(move);
    const Result r = surface_surface(a, b, 1e-7), s = surface_surface(b, a, 1e-7), mr = surface_surface(ma, mb, 1e-7), ms = surface_surface(mb, ma, 1e-7);
    return kinds(r, expected, points) && kinds(s, expected, points) && kinds(mr, expected, points) && kinds(ms, expected, points) &&
        worst(a, b, r) < within && worst(b, a, s) < within && worst(ma, mb, mr) < within && worst(mb, ma, ms) < within;
}

bool meets(const Curve2d& a, const Curve2d& b, size_t count, size_t tangent = 0) {
    for (int order = 0; order < 2; ++order) {
        const Curve2d &first = order ? b : a, &second = order ? a : b;
        const intersection_sweep::Result2d r = intersection_sweep::curve_curve(first, second, 1e-7);
        size_t touching = 0;
        for (const intersection_sweep::Hit2d& h : r.points) {
            touching += h.tangent;
            if (first.point_at(h.a).distance(h.point) > (h.tangent ? 1e-6 : 1e-11) || second.point_at(h.b).distance(h.point) > (h.tangent ? 1e-6 : 1e-11))
                return false;
        }
        if (!r.done || r.coincident || r.points.size() != count || touching != tangent)
            return false;
    }
    return true;
}

const Curve2d line = Curve::line(Point(-3, 0.5, 0), Vector(1, 0.2, 0), -5.0, 5.0), circle = Curve::circle(Point(1, 0, 0), X, Y, 2.0), ellipse = Curve::ellipse(Point(0, 0.5, 0), Vector(1, 1, 0), Vector(-1, 1, 0), 3.0, 1.0);
const Curve2d parabola = Curve::parabola(Point(-1, -1, 0), Y, X, 0.5, -3.0, 3.0), hyperbola = Curve::hyperbola(Point(-2, 0, 0), X, Y, 1.0, 0.8, -2.0, 2.0);
const Curve2d freeform(NurbsCurve::create(false, 3, {Point(-3, -2, 0), Point(-1, 2.5, 0), Point(1, -2.5, 0), Point(3, 2, 0)}));

// A paraboloid, a cone of a line, a barrel of a NURBS profile and a hyperboloid of a line across the axis.
const Surface bowl = Surface::revolution(O, Z, Curve::parabola(O, Z, X, 0.5, 0.0, 3.0)), funnel = Surface::revolution(O, Z, Curve::line(Point(2, 0, 0), Point(1, 0, 3)), 1.0, 5.0);
const Surface barrel = Surface::revolution(O, Z, Curve(NurbsCurve::create(false, 3, {Point(1, 0, 0), Point(2.5, 0, 1), Point(1.5, 0, 2), Point(2, 0, 3)})));
const Surface waist = Surface::revolution(O, Z, Curve::line(Point(1, -2, -2), Point(1, 2, 2)));
const Surface wall = Surface::extrusion(Curve::ellipse(O, X, Y, 3.0, 2.0), Z, -2.0, 4.0), trough = Surface::extrusion(Curve::parabola(Point(0, -1, 0), Y, X, 0.5, -2.0, 2.0), Vector(0.3, 0, 1), -3.0, 3.0);
const Surface sheet = Surface::extrusion(Curve::line(Point(-4, 0.5, 0), Point(4, 1.5, 0)), Z, -2.0, 4.0), ribbon = Surface::extrusion(freeform, Z, -1.0, 1.0);

Surface plane(const Point& o, const Vector& z) {
    return Surface::plane(o, z, std::abs(z[0]) < 0.9 ? X.cross(z) : Y.cross(z), {-10, 10, -10, 10});
}

}

MINI_TEST("Intersection Sweep", "Curve Curve") {
    MINI_CHECK(meets(line, Curve2d::line(0, -3, 1, 4), 1) && meets(line, circle, 2) && meets(line, ellipse, 2) && meets(line, parabola, 2) && meets(line, hyperbola, 1));
    MINI_CHECK(meets(circle, ellipse, 2) && meets(circle, Curve::ellipse(Point(1, 0.2, 0), Vector(1, 1, 0), Vector(-1, 1, 0), 3.0, 1.0), 4) && meets(circle, parabola, 2) && meets(ellipse, parabola, 2) && meets(ellipse, hyperbola, 2) && meets(parabola, hyperbola, 2));
    // The circle crosses the hyperbola twice and touches its vertex.
    MINI_CHECK(meets(circle, hyperbola, 3, 1));
    MINI_CHECK(meets(circle, Curve::circle(Point(6, 0, 0), X, Y, 1.0), 0) && meets(line, Curve::line(Point(0, 3, 0), Vector(1, 0.2, 0), 0.0, 1.0), 0));
    // Touching: a line on a circle and on a parabola's vertex, circles from outside and from inside, an ellipse inside a circle at the ends of its long axis.
    MINI_CHECK(meets(Curve::line(Point(3, -4, 0), Y, 0.0, 1.0), circle, 1, 1) && meets(Curve::line(Point(-5, -1, 0), X, 0.0, 1.0), parabola, 1, 1));
    MINI_CHECK(meets(circle, Curve::circle(Point(4, 0, 0), Y, X, 1.0), 1, 1) && meets(circle, Curve::circle(Point(2, 0, 0), X, Y, 1.0), 1, 1));
    MINI_CHECK(meets(circle, Curve::ellipse(Point(1, 0, 0), Y, X, 2.0, 1.0), 2, 2));
    const intersection_sweep::Result2d same = intersection_sweep::curve_curve(circle, Curve::circle(Point(1, 0, 0), Y, X, 2.0, 1.0, 2.0), 1e-7);
    MINI_CHECK(same.done && same.coincident && same.points.empty());
    // The numeric route: a NURBS curve against the whole of an analytic one, parameters in each curve's own.
    MINI_CHECK(meets(freeform, Curve::line(Point(-9, 0, 0), X, 0.0, 1.0), 3) && meets(freeform, circle, 2) && meets(freeform, parabola, 2) && meets(freeform, Curve2d(freeform.transformed(Xform::scale_xyz(-1.0, 1.0, 1.0))), 3));
    const intersection_sweep::Result2d around = intersection_sweep::curve_curve(Curve::line(Point(0, 0, 0), X, 0.0, 1.0), Curve::circle(O, X, Y, 1.0, 3.0, 4.0), 1e-7);
    MINI_CHECK(around.points.size() == 2 && around.points[0].b >= 3.0 && around.points[1].b < 3.0 + 2 * pi && std::abs(std::abs(around.points[0].a) - 1.0) < 1e-12);
}

MINI_TEST("Intersection Sweep", "Revolution Plane") {
    // Across the axis: a circle through each point of the profile at the plane's height, a line along u on the revolution.
    const Result ring = surface_surface(bowl, plane(Point(0, 0, 2), Z), 1e-7);
    MINI_CHECK(kinds(ring, {Shape::Circle}) && std::abs(ring.curves[0].curve.radius() - 2.0) < 1e-12 && ring.curves[0].on_a.kind() == Shape::Line && ring.curves[0].on_b.kind() == Shape::Circle);
    MINI_CHECK(both(bowl, plane(Point(0, 0, 2), Z), {Shape::Circle}) && both(funnel, plane(Point(3, 1, 1.5), Z * -1.0), {Shape::Circle}) && both(barrel, plane(Point(0, 0, 1.2), Z), {Shape::Circle}));
    MINI_CHECK(both(bowl, plane(Point(0, 0, -1), Z), {}) && both(bowl, plane(O, Z), {}, 1));
    const Result vertex = surface_surface(bowl, plane(O, Z), 1e-7);
    MINI_CHECK(vertex.points[0].tangent && vertex.points[0].point.distance(O) < 1e-9);
    // A profile across the axis: the hyperboloid's waist is its narrowest circle.
    const Result neck = surface_surface(plane(O, Z), waist, 1e-7);
    MINI_CHECK(kinds(neck, {Shape::Circle}) && std::abs(neck.curves[0].curve.radius() - 1.0) < 1e-12 && neck.curves[0].on_b.kind() == Shape::Line && both(waist, plane(Point(0, 0, 1), Z), {Shape::Circle}));
    // Through the axis: the profile at the plane's two angles, lines along v on the revolution, the profile's kind on the plane.
    const Result halves = surface_surface(bowl, plane(O, Vector(1, 2, 0)), 1e-7);
    MINI_CHECK(kinds(halves, {Shape::Parabola, Shape::Parabola}) && halves.curves[0].on_a.kind() == Shape::Line && halves.curves[0].on_b.kind() == Shape::Parabola && halves.curves[0].curve.domain() == bowl.domain(1));
    MINI_CHECK(both(bowl, plane(O, Vector(1, 2, 0)), {Shape::Parabola, Shape::Parabola}) && both(funnel, plane(Point(0, 0, 5), Y), {Shape::Line, Shape::Line}) && both(barrel, plane(O, X), {Shape::Nurbs, Shape::Nurbs}));
    const Surface ball = Surface::revolution(O, Z, Curve::circle(O, X, Z, 2.0));
    MINI_CHECK(both(ball, plane(O, Y), {Shape::Circle}) && both(Surface::revolution(O, Z, Curve::circle(O, X, Z, 2.0, -0.5 * pi, 0.5 * pi)), plane(O, Y), {Shape::Circle, Shape::Circle}));
    MINI_CHECK(!surface_surface(bowl, plane(Point(0, 0, 2), Vector(0.2, 0, 1)), 1e-7).done && !surface_surface(waist, plane(O, X), 1e-7).done);
}

MINI_TEST("Intersection Sweep", "Revolution Coaxial") {
    const Surface cylinder = Surface::cylinder(Point(0, 0, 7), Z * -1.0, Y, 1.5, {0, 2 * pi, -10, 10}), cone = Surface::cone(Point(0, 0, 1), Z, X, 1.0, 0.3, {0, 2 * pi, -10, 10});
    const Surface sphere = Surface::sphere(Point(0, 0, 1), Z, X, 2.0), tilted = Surface::sphere(Point(0, 0, 1), Vector(1, 2, 2), X, 2.0), torus = Surface::torus(Point(0, 0, 2), Z, X, 2.0, 0.5);
    const Result ring = surface_surface(bowl, cylinder, 1e-7);
    MINI_CHECK(kinds(ring, {Shape::Circle}) && std::abs(ring.curves[0].curve.origin()[2] - 1.125) < 1e-12 && ring.curves[0].on_a.kind() == Shape::Line && ring.curves[0].on_b.kind() == Shape::Line);
    MINI_CHECK(both(bowl, cylinder, {Shape::Circle}) && both(bowl, cone, {Shape::Circle, Shape::Circle}) && both(bowl, sphere, {Shape::Circle}) && both(bowl, torus, {Shape::Circle, Shape::Circle}));
    MINI_CHECK(both(funnel, sphere, {Shape::Circle, Shape::Circle}) && both(barrel, cylinder, {Shape::Circle}) && both(barrel, funnel, {Shape::Circle}));
    // The whole line turns into both nappes of its cone: the cylinder meets each, the bowl the upper one twice.
    MINI_CHECK(both(funnel, cylinder, {Shape::Circle, Shape::Circle}) && both(bowl, funnel, {Shape::Circle, Shape::Circle}));
    // A sphere about another axis of its own: the circle is no line in its parameters, the pcurve is fitted.
    const Result fitted = surface_surface(bowl, tilted, 1e-7);
    MINI_CHECK(kinds(fitted, {Shape::Circle}) && fitted.curves[0].on_b.kind() == Shape::Nurbs && fitted.curves[0].deviation_b < 1e-8 && worst(bowl, tilted, fitted) < 1e-8);
    // Touching along a circle: the sphere inside the bowl, the cylinder round the barrel's widest circle.
    const Result inside = surface_surface(bowl, Surface::sphere(Point(0, 0, 3), Z, X, std::sqrt(5.0)), 1e-7);
    MINI_CHECK(kinds(inside, {Shape::Circle}) && inside.curves[0].tangent && std::abs(inside.curves[0].curve.radius() - 2.0) < 1e-6);
    MINI_CHECK(!surface_surface(bowl, cylinder, 1e-7).curves[0].tangent && !surface_surface(bowl, Surface::cylinder(Point(1, 0, 0), Z, X, 1.5), 1e-7).done && !surface_surface(bowl, wall, 1e-7).done);
    // One surface under two descriptions is coincident, not intersected.
    const Result one = surface_surface(Surface::revolution(O, Z, Curve::line(Point(1.5, 0, -1), Point(1.5, 0, 4))), cylinder, 1e-7);
    MINI_CHECK(one.done && one.coincident == 1 && one.curves.empty());
}

MINI_TEST("Intersection Sweep", "Extrusion Plane") {
    // Across the direction: the profile's image, its own kind; level with the profile the pcurve on the extrusion is a line.
    const Result level = surface_surface(wall, plane(Point(0, 0, 1), Z), 1e-7);
    MINI_CHECK(kinds(level, {Shape::Ellipse}) && level.curves[0].on_a.kind() == Shape::Line && level.curves[0].on_b.kind() == Shape::Ellipse && level.curves[0].curve.is_closed());
    const Result tilted = surface_surface(wall, plane(Point(0, 0, 1), Vector(0.2, 0.1, 1)), 1e-7);
    MINI_CHECK(kinds(tilted, {Shape::Ellipse}) && tilted.curves[0].on_a.kind() == Shape::Nurbs && tilted.curves[0].on_b.kind() == Shape::Ellipse && tilted.curves[0].deviation_a < 1e-8);
    MINI_CHECK(both(wall, plane(Point(0, 0, 1), Z), {Shape::Ellipse}) && both(wall, plane(Point(0, 0, 1), Vector(0.2, 0.1, 1)), {Shape::Ellipse}, 0, 1e-8));
    const Result arc = surface_surface(trough, plane(Point(0, 0, 1), Vector(0.1, 0.3, 1)), 1e-7);
    MINI_CHECK(kinds(arc, {Shape::Parabola}) && arc.curves[0].on_a.kind() == Shape::Nurbs && arc.curves[0].on_a.nurbs().cv_count() == 3 && arc.curves[0].deviation_a < 1e-12 && arc.curves[0].on_b.kind() == Shape::Parabola);
    MINI_CHECK(both(trough, plane(Point(0, 0, 1), Vector(0.1, 0.3, 1)), {Shape::Parabola}) && both(sheet, plane(Point(0, 0, 1), Vector(0.1, 0.3, 1)), {Shape::Line}) && both(ribbon, plane(O, Vector(0.1, 0.3, 1)), {Shape::Nurbs}, 0, 1e-8));
    MINI_CHECK(both(Surface::extrusion(Curve::circle(O, X, Y, 2.0), Vector(0.3, 0, 1)), plane(O, Z), {Shape::Circle}) && both(Surface::extrusion(hyperbola, Z), plane(O, Vector(0.3, 0, 1)), {Shape::Hyperbola}, 0, 1e-8));
    // Along the direction: a line through each point where the plane's trace meets the profile.
    const Result lines = surface_surface(wall, plane(Point(1, 0, 0), X), 1e-7);
    MINI_CHECK(kinds(lines, {Shape::Line, Shape::Line}) && lines.curves[0].on_a.kind() == Shape::Line && lines.curves[0].on_b.kind() == Shape::Line && lines.curves[0].curve.domain() == wall.domain(1));
    MINI_CHECK(both(wall, plane(Point(1, 0, 0), X), {Shape::Line, Shape::Line}) && both(trough, plane(Point(0, 0.5, 0), Y), {Shape::Line, Shape::Line}) && both(ribbon, plane(O, Y), {Shape::Line, Shape::Line, Shape::Line}));
    const Result touch = surface_surface(wall, plane(Point(3, 0, 0), X), 1e-7);
    MINI_CHECK(kinds(touch, {Shape::Line}) && touch.curves[0].tangent && both(wall, plane(Point(3.5, 0, 0), X), {}));
}

MINI_TEST("Intersection Sweep", "Extrusion Parallel") {
    const Surface cylinder = Surface::cylinder(Point(2, 0, 9), Z * -1.0, Y, 2.0, {0, 2 * pi, 0, 10});
    const Result lines = surface_surface(wall, cylinder, 1e-7);
    MINI_CHECK(kinds(lines, {Shape::Line, Shape::Line}) && lines.curves[0].on_a.kind() == Shape::Line && lines.curves[0].on_b.kind() == Shape::Line && !lines.curves[0].tangent);
    MINI_CHECK(both(wall, cylinder, {Shape::Line, Shape::Line}) && both(wall, Surface::cylinder(Point(0, 0, 0), Z, X, 2.5, {0, 2 * pi, -1, 1}), {Shape::Line, Shape::Line, Shape::Line, Shape::Line}));
    MINI_CHECK(both(wall, sheet, {Shape::Line, Shape::Line}) && both(sheet, ribbon, {Shape::Line}) && both(wall, ribbon, {Shape::Line, Shape::Line}));
    MINI_CHECK(both(trough, Surface::extrusion(Curve::circle(Point(0, -0.5, 0), X, Y, 1.0), Vector(-0.3, 0, -1), -3.0, 3.0), {Shape::Line, Shape::Line}));
    const Result touch = surface_surface(wall, Surface::cylinder(Point(4, 0, 0), Z, X, 1.0), 1e-7);
    MINI_CHECK(kinds(touch, {Shape::Line}) && touch.curves[0].tangent);
    MINI_CHECK(!surface_surface(wall, Surface::cylinder(O, Vector(0.1, 0, 1), X, 1.0), 1e-7).done && !surface_surface(wall, trough, 1e-7).done && !surface_surface(wall, Surface::sphere(O, Z, X, 2.5), 1e-7).done);
    const Result one = surface_surface(Surface::extrusion(Curve::circle(Point(2, 0, 0), X, Y, 2.0), Z), cylinder, 1e-7);
    MINI_CHECK(one.done && one.coincident == 1 && one.curves.empty());
}
