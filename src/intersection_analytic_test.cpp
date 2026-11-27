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

// Whether every section is within `limit` of both surfaces and of the surface points under its pcurves all along, by the measures it carries.
bool within(const Result& r, double limit) {
    for (const Section& s : r.curves)
        if (std::max({s.fit, s.deviation_a, s.deviation_b}) > limit)
            return false;
    return r.done;
}

int heaviest(const Result& r) {
    int count = 0;
    for (const Section& s : r.curves)
        for (const Curve* c : {&s.curve, static_cast<const Curve*>(&s.on_a), static_cast<const Curve*>(&s.on_b)})
            count = std::max(count, c->kind() == Shape::Nurbs ? c->nurbs().cv_count() : 0);
    return count;
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
    MINI_CHECK(kinds(circles, {Shape::Circle, Shape::Circle}) && worst(villarceau, t, circles) < 1e-8);
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

MINI_TEST("Intersection Analytic", "Villarceau") {
    // A plane through the centre of a torus that touches the tube on both sides, wherever the two lie: two exact circles of the major radius.
    const Surface torus = surface(Kind::Torus, Point(0, 0, 0), Z, 3, 1), plane = surface(Kind::Plane, Point(0, 0, 0), Point(1.0 / 3, 0, std::sqrt(8.0) / 3));
    for (double turn : {0.0, 0.7, 1.9, 2.6, 4.1, 5.3}) {
        const Xform motion = Xform::translation(1.3 * turn, -2.1, 0.7) * Xform::rotation(Vector(0.3, -0.5 + turn, 0.8), turn);
        const Surface p = plane.transformed(motion), t = torus.transformed(motion);
        const Result r = surface_surface(p, t, 1e-7), swapped = surface_surface(t, p, 1e-7);
        MINI_CHECK(kinds(r, {Shape::Circle, Shape::Circle}) && kinds(swapped, {Shape::Circle, Shape::Circle}) && within(r, 1e-8) && within(swapped, 1e-8) && worst(p, t, r) < 1e-8);
        MINI_CHECK(std::abs(r.curves[0].curve.radius() - 3) < 1e-12 && std::abs(r.curves[0].curve.origin().distance(r.curves[1].curve.origin()) - 2) < 1e-9);
    }
    // Off the centre by more than the tolerance the circles open where they crossed into one curve that does not meet itself.
    const Result near = surface_surface(surface(Kind::Plane, Point(0, 0, 1e-4), Point(1.0 / 3, 0, std::sqrt(8.0) / 3)), torus, 1e-7);
    MINI_CHECK(kinds(near, {Shape::Nurbs}) && near.curves[0].curve.is_closed() && within(near, 1e-8));
}

MINI_TEST("Intersection Analytic", "Apex") {
    const Surface cone = surface(Kind::Cone, Point(0, 0, 0), Z, 1, pi / 6);
    const Point apex(0, 0, -std::sqrt(3.0));
    // Two cones whose section winds about one axis near its apex: the pcurve keeps to the section all the way.
    const Surface first = surface(Kind::Cone, Point(-4.135657, -0.347133, 1.967461), Point(-0.103541, -0.077841, 0.991575), 0.920141, 0.538084);
    const Surface second = surface(Kind::Cone, Point(-4.432875, -0.056893, 2.160909), Point(-0.460488, 0.722611, 0.515543), 0.984918, 0.444715);
    const Result winding = surface_surface(first, second, 1e-7);
    MINI_CHECK(!winding.curves.empty() && within(winding, 1e-8) && worst(first, second, winding) < 1e-8 && heaviest(winding) <= 300);
    // A sphere through the apex across the cone: a loop on each side of it, cut there, with pcurves regular through it.
    const Surface across = surface(Kind::Sphere, Point(1, 0, apex[2]), Z, 1);
    const Result eight = surface_surface(across, cone, 1e-7);
    MINI_CHECK(kinds(eight, {Shape::Nurbs, Shape::Nurbs}) && within(eight, 1e-8) && heaviest(eight) <= 60);
    for (const Section& s : eight.curves)
        MINI_CHECK(s.curve.point_at(s.curve.domain().first).distance(apex) < 1e-9 && s.curve.point_at(s.curve.domain().second).distance(apex) < 1e-9);
    // A sphere that only reaches the apex: the apex alone, and a loop elsewhere.
    const Result alone = surface_surface(surface(Kind::Sphere, Point(0.6, 0, apex[2] + 0.8), Z, 1), cone, 1e-7);
    MINI_CHECK(kinds(alone, {Shape::Nurbs}, 1) && within(alone, 1e-8) && !alone.points[0].tangent && alone.points[0].point.distance(apex) < 1e-9);
    // A cylinder that passes the apex by a hundred tolerances: the two branches stay apart, each turning sharply past it.
    const Surface by = surface(Kind::Cylinder, Point(0, 0.4 + 1e-5, apex[2]), X, 0.4);
    const Result past = surface_surface(by, cone, 1e-7);
    MINI_CHECK(past.curves.size() >= 2 && within(past, 1e-8) && worst(by, cone, past) < 1e-8);
    for (const Section& s : past.curves)
        MINI_CHECK(s.curve.closest_point(apex).distance(apex) > 5e-6);
}

MINI_TEST("Intersection Analytic", "Pole") {
    // A cylinder through the north pole of a sphere: the sections end exactly at the pole, with the angle they arrive at.
    const Surface sphere = surface(Kind::Sphere, Point(0, 0, 0), Z, 2), cylinder = surface(Kind::Cylinder, Point(0, 0, 1.3), Point(1, 0.3, 0), 0.7);
    for (double turn : {0.0, 1.3, 2.9}) {
        const Xform motion = Xform::translation(0.4, turn, -1.1) * Xform::rotation(Vector(0.6, 0.2, -0.3 + turn), turn);
        const Surface s = sphere.transformed(motion), c = cylinder.transformed(motion);
        const Result r = surface_surface(s, c, 1e-7);
        MINI_CHECK(kinds(r, {Shape::Nurbs, Shape::Nurbs}) && within(r, 1e-8) && worst(s, c, r) < 1e-8 && heaviest(r) <= 60);
        int ends = 0;
        for (const Section& section : r.curves)
            for (double t : {section.curve.domain().first, section.curve.domain().second})
                ends += std::abs(section.on_a.point_at(t)[1] - 0.5 * pi) < 1e-12 && s.point_at(section.on_a.point_at(t)[0], 0.5 * pi).distance(section.curve.point_at(t)) < 1e-12;
        MINI_CHECK(ends == 4);
    }
    // Viviani's curve: through both poles and across itself on the equator, four arcs.
    const Surface half = surface(Kind::Cylinder, Point(1, 0, 0), Z, 1);
    const Result viviani = surface_surface(sphere, half, 1e-7);
    MINI_CHECK(viviani.curves.size() == 4 && within(viviani, 1e-8) && heaviest(viviani) <= 40);
}

MINI_TEST("Intersection Analytic", "Light Sections") {
    // Ordinary loops take a few dozen control points, the pcurves the same, all within a tenth of the tolerance.
    const Surface sphere = surface(Kind::Sphere, Point(0.1, 0, 0), Z, 2), thin = surface(Kind::Cylinder, Point(0, 0, 0), Z, 0.7);
    const Result loops = surface_surface(sphere, thin, 1e-7);
    MINI_CHECK(kinds(loops, {Shape::Nurbs, Shape::Nurbs}) && within(loops, 1e-8) && heaviest(loops) <= 40);
    const Surface wide = surface(Kind::Cylinder, Point(0, 0, 0), Z, 1.5), through = surface(Kind::Cylinder, Point(0.1, 0, 0.3), Point(1, 0.2, 0.1), 0.6);
    const Result pierced = surface_surface(wide, through, 1e-7);
    MINI_CHECK(kinds(pierced, {Shape::Nurbs, Shape::Nurbs}) && within(pierced, 1e-8) && heaviest(pierced) <= 60);
    const Surface torus = surface(Kind::Torus, Point(0, 0, 0), Z, 3, 1), slanted = surface(Kind::Plane, Point(0, 0, 0.2), Point(1, 0, 1));
    const Result spiric = surface_surface(slanted, torus, 1e-7);
    MINI_CHECK(kinds(spiric, {Shape::Nurbs, Shape::Nurbs}) && within(spiric, 1e-8) && heaviest(spiric) <= 80);
    // An ellipse on a cylinder and a hyperbola on a cone: exact sections, light pcurves.
    const Result ellipse = surface_surface(surface(Kind::Plane, Point(0, 0, 0.3), Point(1, 0, 1)), wide, 1e-7);
    MINI_CHECK(kinds(ellipse, {Shape::Ellipse}) && within(ellipse, 1e-8) && heaviest(ellipse) <= 40);
}

MINI_TEST("Intersection Analytic", "Exact Sections") {
    // A cylinder and a cone about one sphere meet in two ellipses.
    const double a = pi / 6;
    const Surface cylinder = surface(Kind::Cylinder, Point(0, 0, 0), Z, 1);
    const Vector tilt(std::sin(0.8), 0, std::cos(0.8));
    const Surface around = Surface::cone(Point(0, 0, 0) + tilt * (-1 / std::sin(a) + 0.5 / std::tan(a)), tilt, Yv, 0.5, a, {0, 2 * pi, -10, 10});
    const Result ellipses = surface_surface(cylinder, around, 1e-7);
    MINI_CHECK(kinds(ellipses, {Shape::Ellipse, Shape::Ellipse}) && within(ellipses, 1e-8) && worst(cylinder, around, ellipses) < 1e-8);
    // A cone that touches the cylinder along a ruling through its apex: the ruling, cut at the apex and tangent, and an ellipse.
    const Vector lean(-std::sin(a), 0, std::cos(a));
    const Surface touching = Surface::cone(Point(1, 0, -2) + lean * (0.5 / std::tan(a)), lean, Yv, 0.5, a, {0, 2 * pi, -10, 10});
    const Result ruling = surface_surface(cylinder.transformed(Xform::rotation_z(0.3)), touching.transformed(Xform::rotation_z(0.3)), 1e-7);
    MINI_CHECK(kinds(ruling, {Shape::Line, Shape::Line, Shape::Ellipse}) && ruling.curves[0].tangent && ruling.curves[1].tangent && !ruling.curves[2].tangent && within(ruling, 1e-8));
    // Cones from one apex share two rulings, each cut at the apex.
    const Surface upright = surface(Kind::Cone, Point(0, 0, 0), Z, 1, 0.4);
    const Vector slant(std::sin(0.5), 0, std::cos(0.5));
    const Surface leaning = Surface::cone(Point(0, 0, -1 / std::tan(0.4)) + slant * (0.5 / std::tan(0.3)), slant, Yv, 0.5, 0.3, {0, 2 * pi, -10, 10});
    const Result rulings = surface_surface(upright, leaning, 1e-7);
    MINI_CHECK(kinds(rulings, {Shape::Line, Shape::Line, Shape::Line, Shape::Line}) && within(rulings, 1e-12) && straight(rulings.curves[0].on_a) && straight(rulings.curves[0].on_b));
}

MINI_TEST("Intersection Analytic", "Cones Along Their Rulings") {
    // Half cones of one semi-angle with axes 5 degrees apart (zip_ea0000006992revit against its turned copy): the section passes the boxes of both within a quarter of a degree of the swept angle, 10.9 long, on either side; too straight there to tell its plane.
    const Vector axis(-std::sin(pi / 36), std::cos(pi / 36), 0);
    const Surface a = Surface::cone(Point(0, -61.722, 0), Yv, Zv, 46.99, pi / 4, {-pi, 0, -5.388153673, 5.388153673});
    const Surface b = Surface::cone(Point(-1.616042, -61.792558, 0), axis, Zv, 46.99, pi / 4, {0, pi, -5.388153673, 5.388153673});
    const Result r = surface_surface(a, b, 1e-7);
    MINI_CHECK(r.done && kinds(r, {Shape::Nurbs, Shape::Nurbs}) && within(r, 1e-8) && worst(a, b, r) < 1e-8);
    MINI_CHECK(std::abs(r.curves[0].curve.domain().second - 10.886) < 1e-3 && std::abs(r.curves[1].curve.domain().second - 10.886) < 1e-3);
}

MINI_TEST("Intersection Analytic", "Curve Torus Touching") {
    const Surface torus = surface(Kind::Torus, Point(0, 0, 0), Z, 3, 1);
    for (double turn : {0.0, 0.9, 2.2}) {
        const Xform motion = Xform::translation(turn, 0.5, -0.2) * Xform::rotation(Vector(0.1 + turn, 0.4, 0.7), turn);
        const Surface t = torus.transformed(motion);
        // A circle, an ellipse and a line that touch the outer equator at (4, 0, 0), the conics where their parameter starts.
        for (const Curve& c : {Curve::circle(Point(5.5, 0, 0), Xv * -1.0, Yv, 1.5), Curve::ellipse(Point(6.5, 0, 0), Xv * -1.0, Yv, 2.5, 1.1), Curve::line(Point(4, -3, -1.5), Vector(0, 2, 1), 0, 8)}) {
            const Result touch = curve_surface(c.transformed(motion), t, 1e-7);
            MINI_CHECK(touch.done && touch.points.size() == 1 && touch.points[0].tangent && touch.points[0].point.distance(Point(4, 0, 0).transformed(motion)) < 1e-6);
        }
        // Pushed in by more than the tolerance the circle crosses twice, close together.
        const Result crossing = curve_surface(Curve::circle(Point(5.5 - 1e-5, 0, 0), Xv * -1.0, Yv, 1.5).transformed(motion), t, 1e-7);
        MINI_CHECK(crossing.points.size() == 2 && !crossing.points[0].tangent && !crossing.points[1].tangent && crossing.points[0].point.distance(crossing.points[1].point) < 0.02);
    }
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
