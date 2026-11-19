#include "intersection_analytic.h"
#include "mini_test.h"
#include <cmath>

using namespace session_cpp;
using namespace session_cpp::intersection_analytic;
using namespace session_cpp::mini_test;

namespace {

using Kind = Surface::Kind;
using Shape = Curve::Kind;
constexpr double pi = 3.14159265358979323846;

Vector across(const Vector& z) {
    Vector x = std::abs(z[0]) < 0.9 ? Vector(1, 0, 0).cross(z) : Vector(0, 1, 0).cross(z);
    x.normalize_self();
    return x;
}

Surface surface(Kind kind, const Point& o, const Point& axis, double radius = 0.0, double minor = 0.0) {
    Vector z(axis[0], axis[1], axis[2]);
    z.normalize_self();
    const Vector x = across(z);
    switch (kind) {
    case Kind::Plane: return Surface::plane(o, z, x, {-10, 10, -10, 10});
    case Kind::Cylinder: return Surface::cylinder(o, z, x, radius, {0, 2 * pi, -10, 10});
    case Kind::Cone: return Surface::cone(o, z, x, radius, minor, {0, 2 * pi, -10, 10});
    case Kind::Sphere: return Surface::sphere(o, z, x, radius);
    default: return Surface::torus(o, z, x, radius, minor);
    }
}

// The largest distance from the section to the surface points under its pcurves, over its domain.
double fit(const Surface& a, const Surface& b, const Section& s) {
    double worst = 0.0;
    for (double f : {0.0, 0.13, 0.4, 0.77, 1.0}) {
        const double t = s.curve.domain().first + f * (s.curve.domain().second - s.curve.domain().first);
        const Point p = s.curve.point_at(t), ua = s.on_a.point_at(t), ub = s.on_b.point_at(t);
        worst = std::max({worst, a.point_at(ua[0], ua[1]).distance(p), b.point_at(ub[0], ub[1]).distance(p)});
    }
    return worst;
}

bool kinds(const Result& r, std::initializer_list<Shape> expected, size_t points = 0) {
    if (!r.done || r.coincident || r.curves.size() != expected.size() || r.points.size() != points)
        return false;
    size_t i = 0;
    for (Shape k : expected)
        if (r.curves[i++].curve.kind() != k)
            return false;
    return true;
}

double worst(const Surface& a, const Surface& b, const Result& r) {
    double w = 0.0;
    for (const Section& s : r.curves)
        w = std::max(w, fit(a, b, s));
    return w;
}

bool straight(const Curve2d& on) {
    return on.kind() == Shape::Line;
}

const Point Z(0, 0, 1);
const Point X(1, 0, 0);
const Vector Xv(1, 0, 0);
const Vector Yv(0, 1, 0);
const Vector Zv(0, 0, 1);

}

MINI_TEST("Intersection Analytic", "Plane Plane") {
    const Surface a = surface(Kind::Plane, Point(0, 0, 0), Z), b = surface(Kind::Plane, Point(1, 2, 3), Point(1, 1, 0));
    const Result r = surface_surface(a, b, 1e-7);
    MINI_CHECK(kinds(r, {Shape::Line}) && worst(a, b, r) < 1e-12 && straight(r.curves[0].on_a) && straight(r.curves[0].on_b));
    MINI_CHECK(surface_surface(a, surface(Kind::Plane, Point(4, 5, 0), Point(0, 0, -1)), 1e-7).coincident != 0);
    MINI_CHECK(kinds(surface_surface(a, surface(Kind::Plane, Point(4, 5, 1), Z), 1e-7), {}));
}

MINI_TEST("Intersection Analytic", "Plane Sphere") {
    const Surface s = surface(Kind::Sphere, Point(0, 0, 0), Z, 2);
    const Surface cut = surface(Kind::Plane, Point(0, 0, 1), Z);
    const Result r = surface_surface(cut, s, 1e-7);
    MINI_CHECK(kinds(r, {Shape::Circle}) && std::abs(r.curves[0].curve.radius() - std::sqrt(3.0)) < 1e-12 && worst(cut, s, r) < 1e-12 && straight(r.curves[0].on_b));
    const Result touch = surface_surface(surface(Kind::Plane, Point(0, 0, 2), Z), s, 1e-7);
    MINI_CHECK(kinds(touch, {}, 1) && touch.points[0].tangent && touch.points[0].point.distance(Point(0, 0, 2)) < 1e-9);
    MINI_CHECK(kinds(surface_surface(surface(Kind::Plane, Point(0, 0, 2.1), Z), s, 1e-7), {}));
}

MINI_TEST("Intersection Analytic", "Plane Cylinder") {
    const Surface c = surface(Kind::Cylinder, Point(0, 0, 0), Z, 1);
    const Surface tilted = surface(Kind::Plane, Point(0, 0, 1), Point(1, 0, 1));
    const Result ellipse = surface_surface(tilted, c, 1e-7);
    MINI_CHECK(kinds(ellipse, {Shape::Ellipse}) && std::abs(ellipse.curves[0].curve.radius() - std::sqrt(2.0)) < 1e-12 && std::abs(ellipse.curves[0].curve.minor() - 1) < 1e-12);
    MINI_CHECK(worst(tilted, c, ellipse) < 1e-8 && !straight(ellipse.curves[0].on_b));
    const Result circle = surface_surface(c, surface(Kind::Plane, Point(0, 0, 1), Z), 1e-7);
    MINI_CHECK(kinds(circle, {Shape::Circle}) && straight(circle.curves[0].on_a));
    MINI_CHECK(kinds(surface_surface(surface(Kind::Plane, Point(0.5, 0, 0), X), c, 1e-7), {Shape::Line, Shape::Line}));
    const Result touch = surface_surface(surface(Kind::Plane, Point(1, 0, 0), X), c, 1e-7);
    MINI_CHECK(kinds(touch, {Shape::Line}) && touch.curves[0].tangent && straight(touch.curves[0].on_b));
    MINI_CHECK(kinds(surface_surface(surface(Kind::Plane, Point(1.1, 0, 0), X), c, 1e-7), {}));
}

MINI_TEST("Intersection Analytic", "Plane Cone") {
    const double a = pi / 6;
    const Surface c = surface(Kind::Cone, Point(0, 0, 0), Z, 1, a);
    const Point apex(0, 0, -1 / std::tan(a));
    for (const auto& [normal, kind] : {std::pair{Point(0.2, 0, 1), Shape::Ellipse}, {Z, Shape::Circle}, {Point(std::cos(a), 0, -std::sin(a)), Shape::Parabola}}) {
        const Surface p = surface(Kind::Plane, Point(0.3, 0, 1), normal);
        const Result r = surface_surface(p, c, 1e-7);
        MINI_CHECK(kinds(r, {kind}) && worst(p, c, r) < 1e-8 && r.curves[0].on_a.kind() == kind);
    }
    const Surface along = surface(Kind::Plane, Point(0.5, 0, 0), X);
    const Result hyperbola = surface_surface(along, c, 1e-7);
    MINI_CHECK(kinds(hyperbola, {Shape::Hyperbola, Shape::Hyperbola}) && worst(along, c, hyperbola) < 1e-8 && hyperbola.curves[1].on_a.kind() == Shape::Hyperbola);
    const Result point = surface_surface(surface(Kind::Plane, apex, Z), c, 1e-7);
    MINI_CHECK(kinds(point, {}, 1) && !point.points[0].tangent && point.points[0].point.distance(apex) < 1e-9);
    MINI_CHECK(kinds(surface_surface(surface(Kind::Plane, apex, X), c, 1e-7), {Shape::Line, Shape::Line, Shape::Line, Shape::Line}));
    const Result ruling = surface_surface(surface(Kind::Plane, apex, Point(std::cos(a), 0, -std::sin(a))), c, 1e-7);
    MINI_CHECK(kinds(ruling, {Shape::Line, Shape::Line}) && ruling.curves[0].tangent && straight(ruling.curves[0].on_b) && ruling.curves[0].deviation_b < 1e-12);
}

MINI_TEST("Intersection Analytic", "Plane Torus") {
    const Surface t = surface(Kind::Torus, Point(0, 0, 0), Z, 3, 1);
    const Surface across = surface(Kind::Plane, Point(0, 0, 0.6), Z);
    const Result two = surface_surface(across, t, 1e-7);
    MINI_CHECK(kinds(two, {Shape::Circle, Shape::Circle}) && worst(across, t, two) < 1e-12);
    const Result top = surface_surface(surface(Kind::Plane, Point(0, 0, 1), Z), t, 1e-7);
    MINI_CHECK(kinds(top, {Shape::Circle}) && top.curves[0].tangent && std::abs(top.curves[0].curve.radius() - 3) < 1e-9);
    const Surface through = surface(Kind::Plane, Point(0, 0, 0), X);
    const Result tubes = surface_surface(through, t, 1e-7);
    MINI_CHECK(kinds(tubes, {Shape::Circle, Shape::Circle}) && worst(through, t, tubes) < 1e-12);
    const Surface general = surface(Kind::Plane, Point(0, 0, 0.2), Point(1, 0, 1));
    const Result spiric = surface_surface(general, t, 1e-7);
    MINI_CHECK(kinds(spiric, {Shape::Nurbs, Shape::Nurbs}) && worst(general, t, spiric) < 1e-7 && spiric.curves[0].fit < 1e-7 && spiric.curves[0].deviation_b < 1e-7);
    const Surface villarceau = surface(Kind::Plane, Point(0, 0, 0), Point(std::sin(std::asin(1.0 / 3)), 0, std::cos(std::asin(1.0 / 3))));
    const Result circles = surface_surface(villarceau, t, 1e-7);
    MINI_CHECK(circles.done && circles.curves.size() == 2 && worst(villarceau, t, circles) < 1e-7);
}

MINI_TEST("Intersection Analytic", "Sphere Sphere") {
    const Surface a = surface(Kind::Sphere, Point(0, 0, 0), Z, 2), b = surface(Kind::Sphere, Point(3, 0, 0), X, 2);
    const Result r = surface_surface(a, b, 1e-7);
    MINI_CHECK(kinds(r, {Shape::Circle}) && std::abs(r.curves[0].curve.radius() - std::sqrt(1.75)) < 1e-12 && worst(a, b, r) < 1e-8);
    const Result touch = surface_surface(a, surface(Kind::Sphere, Point(4, 0, 0), Z, 2), 1e-7);
    MINI_CHECK(kinds(touch, {}, 1) && touch.points[0].tangent && touch.points[0].point.distance(Point(2, 0, 0)) < 1e-9);
    MINI_CHECK(surface_surface(a, surface(Kind::Sphere, Point(0, 0, 0), X, 2), 1e-7).coincident != 0);
    MINI_CHECK(kinds(surface_surface(a, surface(Kind::Sphere, Point(5, 0, 0), Z, 2), 1e-7), {}));
}

MINI_TEST("Intersection Analytic", "Cylinder Cylinder") {
    const Surface a = surface(Kind::Cylinder, Point(0, 0, 0), Z, 1);
    const Surface beside = surface(Kind::Cylinder, Point(1.5, 0, 0), Z, 1);
    const Result lines = surface_surface(a, beside, 1e-7);
    MINI_CHECK(kinds(lines, {Shape::Line, Shape::Line}) && worst(a, beside, lines) < 1e-12 && straight(lines.curves[0].on_a));
    const Result touch = surface_surface(a, surface(Kind::Cylinder, Point(2, 0, 0), Z, 1), 1e-7);
    MINI_CHECK(kinds(touch, {Shape::Line}) && touch.curves[0].tangent);
    MINI_CHECK(surface_surface(a, surface(Kind::Cylinder, Point(0, 0, 5), Point(0, 0, -1), 1), 1e-7).coincident != 0);
    const Surface turned = surface(Kind::Cylinder, Point(0, 0, 0), Point(0, std::sin(5 * pi / 180), std::cos(5 * pi / 180)), 1);
    const Result ellipses = surface_surface(a, turned, 1e-7);
    MINI_CHECK(kinds(ellipses, {Shape::Ellipse, Shape::Ellipse}) && worst(a, turned, ellipses) < 1e-8);
    const Surface through = surface(Kind::Cylinder, Point(0, 0, 0), X, 0.5);
    const Result loops = surface_surface(a, through, 1e-7);
    MINI_CHECK(kinds(loops, {Shape::Nurbs, Shape::Nurbs}) && worst(a, through, loops) < 1e-7 && loops.curves[0].fit < 1e-7);
    MINI_CHECK(loops.curves[0].deviation_a < 1e-7 && loops.curves[0].deviation_b < 1e-7 && loops.curves[0].curve.is_closed());
    const Surface grazing = surface(Kind::Cylinder, Point(1.5, 0, 0), Point(0, 1, 0), 0.5);
    const Result touch_point = surface_surface(a, grazing, 1e-7);
    MINI_CHECK(kinds(touch_point, {}, 1) && touch_point.points[0].tangent && touch_point.points[0].point.distance(Point(1, 0, 0)) < 1e-6);
    MINI_CHECK(kinds(surface_surface(a, surface(Kind::Cylinder, Point(1.6, 0, 0), Point(0, 1, 0), 0.5), 1e-7), {}));
}

MINI_TEST("Intersection Analytic", "Coaxial") {
    const Surface cylinder = surface(Kind::Cylinder, Point(0, 0, 0), Z, 1);
    const Surface cone = surface(Kind::Cone, Point(0, 0, 0), Z, 0.5, pi / 4);
    const Surface sphere = surface(Kind::Sphere, Point(0, 0, 0.5), X, 2);
    const Surface torus = surface(Kind::Torus, Point(0, 0, 0), Z, 3, 1.5);
    const Result one = surface_surface(cylinder, cone, 1e-7);
    MINI_CHECK(kinds(one, {Shape::Circle, Shape::Circle}) && worst(cylinder, cone, one) < 1e-12 && straight(one.curves[0].on_a) && straight(one.curves[0].on_b));
    const Result two = surface_surface(cylinder, sphere, 1e-7);
    MINI_CHECK(kinds(two, {Shape::Circle, Shape::Circle}) && worst(cylinder, sphere, two) < 1e-8);
    const Result three = surface_surface(sphere, cone, 1e-7);
    MINI_CHECK(three.done && three.curves.size() == 2 && worst(sphere, cone, three) < 1e-8);
    const Result four = surface_surface(sphere, torus, 1e-7);
    MINI_CHECK(four.done && four.curves.size() == 2 && worst(sphere, torus, four) < 1e-8);
    const Result rim = surface_surface(surface(Kind::Cylinder, Point(0, 0, 0), Z, 1.5), torus, 1e-7);
    MINI_CHECK(kinds(rim, {Shape::Circle}) && rim.curves[0].tangent);
}

MINI_TEST("Intersection Analytic", "Quadric Pairs") {
    const Surface sphere = surface(Kind::Sphere, Point(0, 0, 0), Z, 2), cylinder = surface(Kind::Cylinder, Point(1.2, 0, 0), Z, 0.5);
    const Result loops = surface_surface(sphere, cylinder, 1e-7);
    MINI_CHECK(kinds(loops, {Shape::Nurbs, Shape::Nurbs}) && loops.curves[0].curve.is_closed() && loops.curves[0].fit < 1e-7);
    MINI_CHECK(worst(sphere, cylinder, loops) < 1e-7 && loops.curves[0].deviation_a < 1e-7 && loops.curves[1].deviation_b < 1e-7);
    const Surface wide = surface(Kind::Cylinder, Point(1.8, 0, 0), Z, 0.5);
    const Result one = surface_surface(wide, sphere, 1e-7);
    MINI_CHECK(kinds(one, {Shape::Nurbs}) && one.curves[0].curve.is_closed() && worst(wide, sphere, one) < 1e-7);
    const Surface cone = surface(Kind::Cone, Point(0, 0, 0), Z, 1, pi / 6), across = surface(Kind::Cylinder, Point(0.2, 0, 1), X, 0.4);
    const Result pierced = surface_surface(cone, across, 1e-7);
    MINI_CHECK(kinds(pierced, {Shape::Nurbs, Shape::Nurbs}) && worst(cone, across, pierced) < 1e-7 && pierced.curves[0].fit < 1e-7);
    const Result touching = surface_surface(sphere, surface(Kind::Cylinder, Point(2.5, 0, 0), Z, 0.5), 1e-7);
    MINI_CHECK(kinds(touching, {}, 1) && touching.points[0].tangent && touching.points[0].point.distance(Point(2, 0, 0)) < 1e-6);
    // A great circle through the poles of the sphere is cut at them: two meridians, straight in its parameters.
    const Surface upright = surface(Kind::Plane, Point(0, 0, 0), X);
    const Result meridians = surface_surface(upright, sphere, 1e-7);
    MINI_CHECK(kinds(meridians, {Shape::Circle, Shape::Circle}) && straight(meridians.curves[0].on_b) && straight(meridians.curves[1].on_b));
    MINI_CHECK(meridians.curves[0].deviation_b < 1e-6 && meridians.curves[1].deviation_b < 1e-6);
}

MINI_TEST("Intersection Analytic", "Curve Surface") {
    const Surface sphere = surface(Kind::Sphere, Point(0, 0, 0), Z, 2), cylinder = surface(Kind::Cylinder, Point(0, 0, 0), Z, 1);
    const Surface plane = surface(Kind::Plane, Point(0, 0, 0), Z), torus = surface(Kind::Torus, Point(0, 0, 0), Z, 3, 1);
    const Curve line = Curve::line(Point(-5, 0, 0), Xv, 0, 10);
    const Result through = curve_surface(line, sphere, 1e-7);
    MINI_CHECK(through.points.size() == 2 && std::abs(through.points[0].t - 3) < 1e-12 && std::abs(through.points[1].t - 7) < 1e-12);
    MINI_CHECK(sphere.point_at(through.points[0].uv_a.first, through.points[0].uv_a.second).distance(Point(-2, 0, 0)) < 1e-12);
    const Result graze = curve_surface(Curve::line(Point(-5, 2, 0), Xv, 0, 10), sphere, 1e-7);
    MINI_CHECK(graze.points.size() == 1 && graze.points[0].tangent);
    MINI_CHECK(curve_surface(Curve::line(Point(1, 0, 0), Zv, 0, 1), cylinder, 1e-7).coincident == 1);
    MINI_CHECK(curve_surface(line, torus, 1e-7).points.size() == 4);
    const Curve circle = Curve::circle(Point(0, 0, 0), Xv, Yv, 1.5), upright = Curve::circle(Point(0, 0, 0), Xv, Zv, 1.5);
    MINI_CHECK(curve_surface(circle, plane, 1e-7).coincident == 1 && curve_surface(upright, plane, 1e-7).points.size() == 2);
    const Result on_cylinder = curve_surface(upright, cylinder, 1e-7);
    MINI_CHECK(on_cylinder.points.size() == 4 && std::abs(cylinder.distance(on_cylinder.points[0].point)) < 1e-12);
    const Curve ellipse = Curve::ellipse(Point(0, 0, 0), Xv, Yv, 3, 1);
    MINI_CHECK(curve_surface(ellipse, sphere, 1e-7).points.size() == 4 && curve_surface(ellipse, cylinder, 1e-7).points.size() == 2);
    MINI_CHECK(curve_surface(ellipse, cylinder, 1e-7).points[0].tangent);
    MINI_CHECK(!curve_surface(Curve(NurbsCurve::create(false, 1, {Point(0, 0, 0), Point(1, 1, 1)})), plane, 1e-7).done);
}
