#include "curve.h"
#include "mini_test.h"
#include "primitives.h"
#include <cmath>
#include <numbers>

using namespace session_cpp::mini_test;

namespace session_cpp {

namespace {

const double PI = std::numbers::pi;
const Point O(3.0, -2.0, 5.0);
const Vector X(2.0, -1.0, 0.0), Y(1.0, 2.0, 2.0);

std::vector<Curve> analytic_curves() {
    return {Curve::line(O, Y, -2.0, 7.0), Curve::line(O, O + X), Curve::circle(O, X, Y, 2.5), Curve::circle(O, X, Y, 2.5, 5.0, 8.5),
        Curve::ellipse(O, X, Y, 4.0, 1.5), Curve::ellipse(O, X, Y, 4.0, 1.5, -1.0, 0.4)};
}

}

MINI_TEST("Curve", "Nurbs") {
    for (const Curve& c : analytic_curves()) {
        const NurbsCurve& n = c.nurbs();
        MINI_CHECK(c.is_valid() && n.is_valid() && n.domain() == c.domain() && &c.nurbs() == &n);
        const auto [t0, t1] = c.domain();
        double worst = 0.0;
        for (int i = 0; i <= 64; ++i) {
            const Point p = n.point_at(t0 + (t1 - t0) * i / 64);
            worst = std::max(worst, c.closest_point(p).distance(p));
        }
        MINI_CHECK(worst <= 1e-12 * 10.0);
        MINI_CHECK(n.point_at(t0).distance(c.point_at(t0)) <= 1e-12 * 10.0 && n.point_at(t1).distance(c.point_at(t1)) <= 1e-12 * 10.0);
        if (c.kind() == Curve::Kind::Line)
            MINI_CHECK(n.point_at(0.3 * t0 + 0.7 * t1).distance(c.point_at(0.3 * t0 + 0.7 * t1)) <= 1e-12 * 10.0);
        MINI_CHECK(c.is_closed() == (c.kind() != Curve::Kind::Line && t1 - t0 > 6.0));
        MINI_CHECK(std::abs(c.aabb().hx - AABB::from_nurbscurve(n, 0.0, true).hx) < 1e-12);
    }
}

MINI_TEST("Curve", "Evaluate") {
    for (const Curve& c : analytic_curves()) {
        const double t = 0.5 * (c.domain().first + c.domain().second) + 0.1, h = 1e-4;
        const std::vector<Vector> d = c.evaluate(t, 2);
        MINI_CHECK(d.size() == 3 && c.evaluate(t, 1).size() == 2 && c.evaluate(t).size() == 1);
        const auto at = [&](double a) { const Point p = c.point_at(t + a * h); return Vector(p[0], p[1], p[2]); };
        MINI_CHECK(((at(1) - at(-1)) * (0.5 / h) - d[1]).magnitude() < 1e-6);
        MINI_CHECK(((at(1) + at(-1) - at(0) * 2.0) * (1.0 / (h * h)) - d[2]).magnitude() < 1e-5);
        MINI_CHECK((c.tangent_at(t) * d[1].magnitude() - d[1]).magnitude() < 1e-12);
        MINI_CHECK(c.kind() != Curve::Kind::Line || std::abs(d[1].magnitude() - 1.0) < 1e-15);
    }
}

MINI_TEST("Curve", "Closest Parameter") {
    for (const Curve& c : analytic_curves()) {
        const auto [t0, t1] = c.domain();
        const Vector normal = c.kind() == Curve::Kind::Line ? c.x().cross(Vector(0.3, 0.1, 1.0)) : c.z();
        for (int i = 1; i < 16; ++i) {
            const double t = t0 + (t1 - t0) * i / 16.3;
            MINI_CHECK(std::abs(c.closest_parameter(c.point_at(t)) - t) < 1e-12);
            MINI_CHECK(std::abs(c.closest_parameter(c.point_at(t) + normal * 0.5) - t) < 1e-12);
        }
        if (c.kind() == Curve::Kind::Line)
            continue;
        // Off an ellipse along its own normal the foot is the point left.
        const double t = t0 + 0.37 * (t1 - t0);
        const Vector out = c.evaluate(t, 1)[1].cross(c.z());
        MINI_CHECK(std::abs(c.closest_parameter(c.point_at(t) + out * (0.2 / out.magnitude())) - t) < 1e-12);
    }
}

MINI_TEST("Curve", "Period") {
    for (const Curve& c : analytic_curves()) {
        const bool round = c.kind() != Curve::Kind::Line;
        MINI_CHECK(c.period() == (round ? 2.0 * PI : 0.0));
        MINI_CHECK(c.point_at(0.7 + c.period()).distance(c.point_at(0.7)) < 1e-12 * 10.0);
        const double start = c.domain().first, t = c.closest_parameter(c.point_at(start - 0.25));
        MINI_CHECK(std::abs(t - (start - 0.25 + c.period())) < 1e-12);
    }
    MINI_CHECK(Curve::wrapped(7.0, 0.0, 2.0 * PI) == 7.0 - 2.0 * PI && Curve::wrapped(-1.0, 5.0, 2.0 * PI) == -1.0 + 2.0 * PI);
    MINI_CHECK(Curve::wrapped(5.0, 5.0, 2.0 * PI) == 5.0 && Curve::wrapped(7.0, 0.0, 0.0) == 7.0);
    MINI_CHECK(Curve::wrapped(5.0 - 1e-17, 5.0, 2.0 * PI) >= 5.0 && Curve::wrapped(5.0 + 2.0 * PI, 5.0, 2.0 * PI) < 5.0 + 2.0 * PI);
}

MINI_TEST("Curve", "Transform") {
    const Xform rigid = Xform::translation(1.0, -4.0, 2.0) * Xform::rotation_z(0.8) * Xform::rotation_x(-0.3);
    const Xform mirror = Xform::scale_xyz(1.0, -1.0, 1.0), stretch = Xform::scale_xyz(1.0, 2.0, 1.0);
    for (const Curve& c : analytic_curves()) {
        const double t = 0.5 * (c.domain().first + c.domain().second);
        for (const Xform& x : {rigid, mirror}) {
            const Curve moved = c.transformed(x);
            MINI_CHECK(moved.kind() == c.kind() && moved.point_at(t).distance(x.transform_point(c.point_at(t))) < 1e-12 * 10.0);
            MINI_CHECK(moved.nurbs().point_at(moved.domain().first).distance(x.transform_point(c.point_at(c.domain().first))) < 1e-12 * 10.0);
        }
        const Curve stretched = c.transformed(stretch);
        MINI_CHECK(stretched.kind() == (c.kind() == Curve::Kind::Line ? Curve::Kind::Line : Curve::Kind::Nurbs));
        MINI_CHECK(stretched.point_at(stretched.domain().second).distance(stretch.transform_point(c.point_at(c.domain().second))) < 1e-12 * 10.0);
        Curve back = c;
        back.reverse();
        MINI_CHECK(back.point_at(-t).distance(c.point_at(t)) < 1e-12 * 10.0 && back.tangent_at(-t).dot(c.tangent_at(t)) < -1.0 + 1e-12);
        MINI_CHECK(back.domain().first == -c.domain().second && back.domain().second == -c.domain().first);
        MINI_CHECK(back.nurbs().point_at_start().distance(c.nurbs().point_at_end()) < 1e-12 * 10.0);
        Curve part = c;
        MINI_CHECK(part.set_domain(t, c.domain().second) && part.nurbs().point_at_start().distance(c.point_at(t)) < 1e-12 * 10.0 && !part.set_domain(1.0, 1.0));
    }
}

MINI_TEST("Curve", "Recognized") {
    const Curve circle = Curve::recognized(Primitives::circle(1, 2, 3, 2.0));
    MINI_CHECK(circle.kind() == Curve::Kind::Circle && std::abs(circle.radius() - 2.0) < 1e-12 && circle.origin().distance(Point(1, 2, 3)) < 1e-12);
    MINI_CHECK(circle.z().dot(Vector(0, 0, 1)) > 1.0 - 1e-12 && std::abs(circle.domain().second - circle.domain().first - 2.0 * PI) < 1e-12);
    const Curve ellipse = Curve::recognized(Primitives::ellipse(1, 2, 3, 2.0, 5.0));
    MINI_CHECK(ellipse.kind() == Curve::Kind::Ellipse && std::abs(ellipse.radius() - 5.0) < 1e-9 && std::abs(ellipse.minor() - 2.0) < 1e-9);
    MINI_CHECK(std::abs(ellipse.x().dot(Vector(0, 1, 0))) > 1.0 - 1e-9);
    MINI_CHECK(Curve::recognized(NurbsCurve::create(false, 1, {Point(0, 0, 0), Point(1, 2, 3)})).kind() == Curve::Kind::Line);
    MINI_CHECK(Curve::recognized(NurbsCurve::create(false, 3, {Point(0, 0, 0), Point(1, 2, 0), Point(2, -1, 0), Point(3, 0, 1)})).kind() == Curve::Kind::Nurbs);
    for (const Curve& c : analytic_curves()) {
        const Curve again = Curve::recognized(c.nurbs());
        MINI_CHECK(again.kind() == c.kind());
        MINI_CHECK(std::abs((again.domain().second - again.domain().first) - (c.domain().second - c.domain().first)) < 1e-8);
        // The same extent in the same direction, whatever the origin of the parameter.
        for (double f : {0.0, 0.3, 1.0}) {
            const double a = again.domain().first + f * (again.domain().second - again.domain().first), b = c.domain().first + f * (c.domain().second - c.domain().first);
            MINI_CHECK(again.point_at(a).distance(c.point_at(b)) < 1e-8);
        }
    }
}

MINI_TEST("Curve", "Json") {
    std::vector<Curve> all = analytic_curves();
    all.push_back(Curve(Primitives::circle(0, 0, 0, 1)));
    for (const Curve& c : all) {
        const Curve loaded = Curve::jsonload(nlohmann::json::parse(c.jsondump().dump()));
        MINI_CHECK(loaded.kind() == c.kind() && loaded.domain() == c.domain());
        MINI_CHECK(loaded.point_at(c.domain().second).distance(c.point_at(c.domain().second)) < 1e-12);
    }
    MINI_CHECK(!Curve().is_valid() && !Curve::jsonload(Curve().jsondump()).is_valid());
}

MINI_TEST("Curve2d", "Line And Arc") {
    // A straight pcurve across the seam of a cylinder: u from 5 to 8 radians, past 2 pi.
    const Curve2d line = Curve2d::line(5.0, 1.0, 8.0, 2.0);
    MINI_CHECK(line.kind() == Curve::Kind::Line && line.point_at(0.0).distance(Point(5, 1, 0)) < 1e-15 && line.point_at(line.domain().second).distance(Point(8, 2, 0)) < 1e-15);
    const Curve2d back = line.shifted(-2.0 * PI, 0.0);
    MINI_CHECK(back.kind() == Curve::Kind::Line && std::abs(back.domain().second - line.domain().second) < 1e-15 && std::abs(back.point_at(0.0)[0] - (5.0 - 2.0 * PI)) < 1e-15 && back.point_at(0.0)[1] == 1.0);
    const Curve2d arc = Curve::circle(Point(1, 2, 0), Vector(1, 0, 0), Vector(0, -1, 0), 3.0, 0.0, PI);
    MINI_CHECK(arc.kind() == Curve::Kind::Circle && arc.point_at(0.5 * PI).distance(Point(1, -1, 0)) < 1e-15 && arc.z()[2] == -1.0);
    MINI_CHECK(arc.nurbs().is_valid() && arc.nurbs().point_at(PI).distance(Point(-2, 2, 0)) < 1e-12);
    const Curve2d freeform(NurbsCurve::create(false, 1, {Point(0, 0, 0), Point(1, 1, 0)}));
    MINI_CHECK(freeform.kind() == Curve::Kind::Nurbs && freeform.shifted(1.0, 0.0).point_at(freeform.domain().first).distance(Point(1, 0, 0)) < 1e-15);
}

namespace {

std::vector<Curve> open_conics() {
    return {Curve::parabola(O, X, Y, 1.5, -2.0, 3.0), Curve::parabola(O, X, Y, 0.4, 1.0, 2.5), Curve::hyperbola(O, X, Y, 2.0, 1.2, -1.0, 1.5), Curve::hyperbola(O, X, Y, 0.7, 3.0, 0.5, 4.2)};
}

}

MINI_TEST("Curve", "Parabola And Hyperbola") {
    for (const Curve& c : open_conics()) {
        const bool parabola = c.kind() == Curve::Kind::Parabola;
        const auto [t0, t1] = c.domain();
        const NurbsCurve& n = c.nurbs();
        MINI_CHECK(c.is_valid() && n.is_valid() && n.domain() == c.domain() && &c.nurbs() == &n && c.period() == 0.0 && !c.is_closed());
        MINI_CHECK(n.degree() == 2 && n.is_rational() == !parabola);
        double size = 0.0, worst = 0.0, same = 0.0, implicit = 0.0;
        for (int i = 0; i <= 64; ++i)
            size = std::max(size, c.point_at(t0 + (t1 - t0) * i / 64).distance(O));
        for (int i = 0; i <= 64; ++i) {
            const double t = t0 + (t1 - t0) * i / 64;
            const Point p = n.point_at(t);
            worst = std::max(worst, c.closest_point(p).distance(p));
            same = std::max(same, p.distance(c.point_at(t)));
            const double a = (p - O).dot(c.x()), b = (p - O).dot(c.y());
            implicit = std::max(implicit, parabola ? std::abs(b * b - 4.0 * c.radius() * a) : std::abs(a * a / (c.radius() * c.radius()) - b * b / (c.minor() * c.minor()) - 1.0));
            MINI_CHECK(std::abs((p - O).dot(c.z())) <= 1e-12 * size && (parabola || a > 0.0));
        }
        MINI_CHECK(worst <= 1e-12 * size && implicit <= 1e-12 * size * size);
        // The NURBS of a parabola runs in the parabola's own parameter.
        MINI_CHECK(!parabola || same <= 1e-12 * size);
        MINI_CHECK(n.point_at(t0).distance(c.point_at(t0)) <= 1e-12 * size && n.point_at(t1).distance(c.point_at(t1)) <= 1e-12 * size);
        const double t = 0.5 * (t0 + t1) + 0.1, h = 1e-4;
        const std::vector<Vector> d = c.evaluate(t, 2);
        const auto at = [&](double a) { const Point p = c.point_at(t + a * h); return Vector(p[0], p[1], p[2]); };
        MINI_CHECK(d.size() == 3 && ((at(1) - at(-1)) * (0.5 / h) - d[1]).magnitude() < 1e-6 * size && ((at(1) + at(-1) - at(0) * 2.0) * (1.0 / (h * h)) - d[2]).magnitude() < 1e-5 * size);
        MINI_CHECK((c.tangent_at(t) * d[1].magnitude() - d[1]).magnitude() < 1e-12 * size);
        for (int i = 0; i <= 16; ++i) {
            const double at = t0 + (t1 - t0) * i / 16.3;
            const Vector out = c.evaluate(at, 1)[1].cross(c.z());
            MINI_CHECK(std::abs(c.closest_parameter(c.point_at(at)) - at) < 1e-12 * size);
            MINI_CHECK(std::abs(c.closest_parameter(c.point_at(at) + c.z() * 0.5) - at) < 1e-12 * size);
            for (double side : {-0.2, 0.2})
                MINI_CHECK(std::abs(c.closest_parameter(c.point_at(at) + out * (side / out.magnitude())) - at) < 1e-10 * size);
        }
        const Xform rigid = Xform::translation(1.0, -4.0, 2.0) * Xform::rotation_z(0.8) * Xform::rotation_x(-0.3);
        for (const Xform& x : {rigid, Xform::scale_xyz(1.0, -1.0, 1.0)}) {
            const Curve moved = c.transformed(x);
            MINI_CHECK(moved.kind() == c.kind() && std::abs(moved.domain().second - t1) < 1e-12 && moved.point_at(t).distance(x.transform_point(c.point_at(t))) < 1e-12 * size);
        }
        // A parabola's parameter is a length and grows with the curve; a hyperbola's is not.
        const Xform grow = Xform::scale_xyz(2.0, 2.0, 2.0), stretch = Xform::scale_xyz(1.0, 2.0, 1.0);
        const Curve grown = c.transformed(grow);
        MINI_CHECK(grown.kind() == c.kind() && grown.radius() == 2.0 * c.radius() && grown.domain().second == (parabola ? 2.0 : 1.0) * t1);
        MINI_CHECK(grown.point_at(parabola ? 2.0 * t : t).distance(grow.transform_point(c.point_at(t))) < 1e-12 * size);
        const Curve stretched = c.transformed(stretch);
        MINI_CHECK(stretched.kind() == Curve::Kind::Nurbs && stretched.point_at(t1).distance(stretch.transform_point(c.point_at(t1))) < 1e-12 * size);
        Curve back = c;
        back.reverse();
        MINI_CHECK(back.point_at(-t).distance(c.point_at(t)) < 1e-12 * size && back.tangent_at(-t).dot(c.tangent_at(t)) < -1.0 + 1e-12);
        MINI_CHECK(back.domain().first == -t1 && back.domain().second == -t0 && back.nurbs().point_at_start().distance(c.nurbs().point_at_end()) < 1e-12 * size);
        Curve part = c;
        MINI_CHECK(part.set_domain(t, t1) && part.nurbs().point_at_start().distance(c.point_at(t)) < 1e-12 * size);
        const Curve loaded = Curve::jsonload(nlohmann::json::parse(c.jsondump().dump()));
        MINI_CHECK(loaded.kind() == c.kind() && loaded.domain() == c.domain() && loaded.point_at(t1).distance(c.point_at(t1)) < 1e-12);
        MINI_CHECK(std::abs(c.aabb().hx - AABB::from_nurbscurve(n, 0.0, true).hx) < 1e-12);
    }
    // A long hyperbola arc is cut into spans; inside the evolute of a parabola the nearest of three feet is taken.
    const Curve wide = Curve::hyperbola(O, X, Y, 1.0, 1.0, -3.2, 3.2);
    MINI_CHECK(wide.nurbs().cv_count() == 15 && wide.closest_point(wide.nurbs().point_at(2.9)).distance(wide.nurbs().point_at(2.9)) < 1e-11);
    const Curve bowl = Curve::parabola(Point(0, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0), 0.25, -3.0, 3.0);
    MINI_CHECK(std::abs(bowl.closest_parameter(Point(4.0, 0.5, 0)) - 1.9056) < 1e-3 && std::abs(bowl.closest_parameter(Point(4.0, -0.5, 0)) + 1.9056) < 1e-3);
}

MINI_TEST("Curve2d", "Parabola And Hyperbola") {
    const Curve2d parabola = Curve::parabola(Point(1, 2, 0), Vector(0, 1, 0), Vector(1, 0, 0), 0.5, -1.0, 2.0);
    MINI_CHECK(parabola.kind() == Curve::Kind::Parabola && parabola.point_at(2.0).distance(Point(3, 4, 0)) < 1e-15 && parabola.z()[2] == -1.0);
    MINI_CHECK(parabola.shifted(2.0 * PI, 0.0).point_at(2.0).distance(Point(3 + 2.0 * PI, 4, 0)) < 1e-14 && parabola.nurbs().point_at(0.5).distance(parabola.point_at(0.5)) < 1e-14);
    const Curve2d hyperbola = Curve::hyperbola(Point(1, 2, 0), Vector(1, 0, 0), Vector(0, 1, 0), 3.0, 2.0, -1.0, 1.0);
    MINI_CHECK(hyperbola.kind() == Curve::Kind::Hyperbola && hyperbola.point_at(0.0).distance(Point(4, 2, 0)) < 1e-15 && std::abs(hyperbola.closest_parameter(hyperbola.point_at(0.7)) - 0.7) < 1e-13);
}

}
