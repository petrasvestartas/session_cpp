#include "mini_test.h"
#include "primitives.h"
#include "surface.h"
#include <cmath>
#include <numbers>

using namespace session_cpp::mini_test;

namespace session_cpp {

namespace {

const double PI = std::numbers::pi;
const Point O(3.0, -2.0, 5.0);
const Vector Z(1.0, 2.0, 2.0), X(2.0, -1.0, 0.0);

std::vector<Surface> analytic_surfaces() {
    return {Surface::plane(O, Z, X, {-2.0, 3.0, 1.0, 4.0}), Surface::cylinder(O, Z, X, 2.5, {1.0, 1.0 + 2.0 * PI, -1.0, 3.0}),
        Surface::cone(O, Z, X, 1.5, 0.4, {-0.5, 4.0, 0.5, 3.0}), Surface::sphere(O, Z, X, 2.0), Surface::sphere(O, Z, X, 2.0, {0.3, 2.0, -1.0, 0.7}),
        Surface::torus(O, Z, X, 5.0, 1.5), Surface::torus(O, Z, X, 5.0, 1.5, {5.0, 9.0, -2.0, 1.0})};
}

double size(const Surface& s) {
    return 1.0 + s.radius() + (s.kind() == Surface::Kind::Torus ? s.minor() : 0.0) + 10.0;
}

}

MINI_TEST("Surface", "Nurbs") {
    for (const Surface& s : analytic_surfaces()) {
        const NurbsSurface& n = s.nurbs();
        MINI_CHECK(s.is_valid() && n.is_valid());
        const auto [u0, u1] = s.domain(0);
        const auto [v0, v1] = s.domain(1);
        MINI_CHECK(n.domain(0) == s.domain(0) && n.domain(1) == s.domain(1));
        double worst = 0.0, corners = 0.0;
        for (int i = 0; i <= 12; ++i)
            for (int j = 0; j <= 12; ++j) {
                const Point p = n.point_at(u0 + (u1 - u0) * i / 12, v0 + (v1 - v0) * j / 12);
                worst = std::max(worst, s.distance(p));
            }
        for (double u : {u0, u1})
            for (double v : {v0, v1})
                corners = std::max(corners, n.point_at(u, v).distance(s.point_at(u, v)));
        MINI_CHECK(worst <= 1e-12 * size(s));
        MINI_CHECK(corners <= 1e-12 * size(s));
        MINI_CHECK(&s.nurbs() == &n);
    }
}

MINI_TEST("Surface", "Evaluate") {
    for (const Surface& s : analytic_surfaces()) {
        const double u = 0.5 * (s.domain(0).first + s.domain(0).second) + 0.1, v = 0.5 * (s.domain(1).first + s.domain(1).second) + 0.1, h = 1e-4;
        const std::vector<Vector> d = s.evaluate(u, v, 2);
        MINI_CHECK(d.size() == 6 && s.evaluate(u, v, 1).size() == 3 && s.evaluate(u, v).size() == 1);
        const auto at = [&](double a, double b) { const Point p = s.point_at(u + a * h, v + b * h); return Vector(p[0], p[1], p[2]); };
        MINI_CHECK(((at(0, 1) - at(0, -1)) * (0.5 / h) - d[1]).magnitude() < 1e-6);
        MINI_CHECK(((at(0, 1) + at(0, -1) - at(0, 0) * 2.0) * (1.0 / (h * h)) - d[2]).magnitude() < 1e-5);
        MINI_CHECK(((at(1, 0) - at(-1, 0)) * (0.5 / h) - d[3]).magnitude() < 1e-6);
        MINI_CHECK(((at(1, 1) - at(1, -1) - at(-1, 1) + at(-1, -1)) * (0.25 / (h * h)) - d[4]).magnitude() < 1e-5);
        MINI_CHECK(((at(1, 0) + at(-1, 0) - at(0, 0) * 2.0) * (1.0 / (h * h)) - d[5]).magnitude() < 1e-5);
        const Vector n = s.normal_at(u, v);
        MINI_CHECK(std::abs(n.magnitude() - 1.0) < 1e-12 && std::abs(n.dot(d[1])) < 1e-12 * size(s) && std::abs(n.dot(d[3])) < 1e-12 * size(s));
        MINI_CHECK(n.dot(d[3].cross(d[1])) > 0.0);
        const Vector out = s.point_at(u, v) - s.origin();
        MINI_CHECK(s.right_handed());
        MINI_CHECK(s.kind() == Surface::Kind::Plane ? n.dot(s.z()) > 1.0 - 1e-12 : s.kind() == Surface::Kind::Torus || n.dot(out - s.z() * out.dot(s.z())) > 0.0);
        const std::pair<double, double> uv = s.closest_parameters(s.nurbs().point_at(u, v));
        MINI_CHECK(s.normal_at(uv.first, uv.second).dot(s.nurbs().normal_at(u, v)) > 1.0 - 1e-9);
    }
    const Surface sphere = Surface::sphere(O, Z, X, 2.0);
    MINI_CHECK((sphere.normal_at(1.0, 0.5 * PI) - Z * (1.0 / 3.0)).magnitude() < 1e-12);
}

MINI_TEST("Surface", "Closest Parameters") {
    for (const Surface& s : analytic_surfaces()) {
        const auto [u0, u1] = s.domain(0);
        const auto [v0, v1] = s.domain(1);
        for (int i = 1; i < 8; ++i)
            for (int j = 1; j < 8; ++j) {
                const double u = u0 + (u1 - u0) * i / 8.1, v = v0 + (v1 - v0) * j / 8.3;
                const Point p = s.point_at(u, v);
                const std::pair<double, double> uv = s.closest_parameters(p);
                MINI_CHECK(std::abs(uv.first - u) < 1e-12 * size(s) && std::abs(uv.second - v) < 1e-12 * size(s));
                const Point lifted = p + s.normal_at(u, v) * 0.25;
                MINI_CHECK(s.closest_point(lifted).distance(p) < 1e-12 * size(s) && std::abs(s.distance(lifted) - 0.25) < 1e-12 * size(s));
            }
    }
}

MINI_TEST("Surface", "Period") {
    for (const Surface& s : analytic_surfaces()) {
        const bool round = s.kind() != Surface::Kind::Plane;
        MINI_CHECK(s.period(0) == (round ? 2.0 * PI : 0.0) && s.period(1) == (s.kind() == Surface::Kind::Torus ? 2.0 * PI : 0.0));
        const double u = 0.7, v = 0.3;
        MINI_CHECK(s.point_at(u + s.period(0), v + s.period(1)).distance(s.point_at(u, v)) < 1e-12 * size(s));
        const double start = s.domain(0).first;
        const std::pair<double, double> uv = s.closest_parameters(s.point_at(start - 0.25, v));
        MINI_CHECK(round ? std::abs(uv.first - (start - 0.25 + 2.0 * PI)) < 1e-12 : std::abs(uv.first - (start - 0.25)) < 1e-12);
    }
    const Surface cylinder = Surface::cylinder(O, Z, X, 2.5), torus = Surface::torus(O, Z, X, 5.0, 1.5);
    // A face's box past the period: 5 to 8 radians holds the angle 0.5 as 0.5 + 2 pi.
    MINI_CHECK(std::abs(cylinder.adjusted(0.5, 7.0, {5.0, 8.0, 0.0, 1.0}).first - (0.5 + 2.0 * PI)) < 1e-15);
    MINI_CHECK(cylinder.adjusted(0.5, 7.0, {5.0, 8.0, 0.0, 1.0}).second == 7.0);
    MINI_CHECK(std::abs(cylinder.adjusted(4.9, 0.0, {5.0, 8.0, 0.0, 1.0}).first - 4.9) < 1e-15);
    MINI_CHECK(std::abs(cylinder.adjusted(2.0, 0.0, {5.0, 8.0, 0.0, 1.0}).first - (2.0 + 2.0 * PI)) < 1e-15);
    MINI_CHECK(std::abs(torus.adjusted(0.0, 6.0, {-1.0, 1.0, -1.0, 1.0}).second - (6.0 - 2.0 * PI)) < 1e-15);
}

MINI_TEST("Surface", "Transform") {
    const Xform rigid = Xform::translation(1.0, -4.0, 2.0) * Xform::rotation_z(0.8) * Xform::rotation_x(-0.3);
    const Xform mirror = Xform::scale_xyz(1.0, -1.0, 1.0), stretch = Xform::scale_xyz(1.0, 2.0, 1.0);
    for (const Surface& s : analytic_surfaces()) {
        const double u = 0.5 * (s.domain(0).first + s.domain(0).second), v = 0.5 * (s.domain(1).first + s.domain(1).second);
        for (int k = 0; k < 2; ++k) {
            const Xform& x = k == 0 ? rigid : mirror;
            const Surface moved = s.transformed(x);
            MINI_CHECK(moved.kind() == s.kind() && moved.right_handed() == (k == 0));
            MINI_CHECK(moved.point_at(u, v).distance(x.transform_point(s.point_at(u, v))) < 1e-12 * size(s));
            MINI_CHECK(moved.distance(moved.nurbs().point_at(u, v)) < 1e-12 * size(s));
        }
        const Surface scaled = s.transformed(Xform::scale_xyz(2.0, 2.0, 2.0));
        MINI_CHECK(scaled.kind() == s.kind() && scaled.radius() == 2.0 * s.radius());
        MINI_CHECK(s.kind() == Surface::Kind::Plane || s.transformed(stretch).kind() == Surface::Kind::Nurbs);
        const Surface freeform = s.transformed(stretch);
        MINI_CHECK(freeform.is_valid() && freeform.nurbs().point_at(u, v).distance(stretch.transform_point(s.nurbs().point_at(u, v))) < 1e-12 * size(s));
        for (int dir = 0; dir < 2; ++dir) {
            Surface back = s;
            back.reverse(dir);
            MINI_CHECK(back.point_at(dir == 0 ? -u : u, dir == 1 ? -v : v).distance(s.point_at(u, v)) < 1e-12 * size(s));
            MINI_CHECK(back.normal_at(dir == 0 ? -u : u, dir == 1 ? -v : v).dot(s.normal_at(u, v)) < -1.0 + 1e-12);
            MINI_CHECK(back.domain(dir).first == -s.domain(dir).second && back.domain(dir).second == -s.domain(dir).first && !back.right_handed());
            MINI_CHECK(back.nurbs().is_valid() && s.coincident(back, 1e-9) == -1);
        }
    }
}

MINI_TEST("Surface", "Coincident") {
    const Vector y = Z.cross(X);
    const Surface cylinder = Surface::cylinder(O, Z, X, 2.5), cone = Surface::cone(O, Z, X, 1.5, 0.4);
    MINI_CHECK(cylinder.coincident(Surface::cylinder(O + Z * 3.0, Z, y, 2.5), 1e-9) == 1);
    MINI_CHECK(cylinder.coincident(Surface::cylinder(O, Z * -1.0, X, 2.5), 1e-9) == 1);
    MINI_CHECK(cylinder.coincident(Surface::cylinder(O, Z, X, 2.5 + 1e-6), 1e-9) == 0);
    MINI_CHECK(cylinder.coincident(Surface::cylinder(O + X * 1e-6, Z, X, 2.5), 1e-9) == 0);
    MINI_CHECK(cylinder.coincident(Surface::cylinder(O + X * 1e-6, Z, X, 2.5), 1e-5) == 1);
    // One cone from another point of its axis, where its radius differs.
    const double shift = 2.0;
    MINI_CHECK(cone.coincident(Surface::cone(O + Z * (shift / 3.0), Z, y, 1.5 + shift * std::tan(0.4), 0.4), 1e-9) == 1);
    MINI_CHECK(cone.coincident(Surface::cone(O, Z, X, 1.5, 0.41), 1e-9) == 0);
    MINI_CHECK(cone.coincident(cylinder, 1e-9) == 0);
    const Surface plane = Surface::plane(O, Z, X);
    MINI_CHECK(plane.coincident(Surface::plane(O + X + y, Z * -1.0, y), 1e-9) == -1);
    MINI_CHECK(plane.coincident(Surface::plane(O + Z * 1e-3, Z, X), 1e-9) == 0);
    MINI_CHECK(Surface::sphere(O, Z, X, 2.0).coincident(Surface::sphere(O, X, Z, 2.0), 1e-9) == 1);
    MINI_CHECK(Surface::torus(O, Z, X, 5.0, 1.5).coincident(Surface::torus(O, Z * -1.0, y, 5.0, 1.5), 1e-9) == 1);
    const Surface freeform(Primitives::cylinder_surface(0, 0, 0, 1, 2));
    MINI_CHECK(freeform.coincident(freeform, 1e-9) == 1 && freeform.coincident(Surface(Primitives::cylinder_surface(0, 0, 0, 1, 2)), 1e-9) == 0);
}

MINI_TEST("Surface", "Recognized") {
    const std::vector<std::pair<NurbsSurface, Surface::Kind>> legacy = {
        {Primitives::cylinder_surface(1, 2, 3, 2.0, 5.0), Surface::Kind::Cylinder}, {Primitives::cone_surface(1, 2, 3, 2.0, 5.0), Surface::Kind::Cone},
        {Primitives::frustum_surface(1, 2, 3, 2.0, 1.0, 5.0), Surface::Kind::Cone}, {Primitives::sphere_surface(1, 2, 3, 2.0), Surface::Kind::Sphere},
        {Primitives::torus_surface(1, 2, 3, 5.0, 1.0), Surface::Kind::Torus}, {Primitives::wave_surface(5, 1.0), Surface::Kind::Nurbs}};
    for (const auto& [nurbs, kind] : legacy) {
        const Surface s = Surface::recognized(nurbs);
        MINI_CHECK(s.kind() == kind && s.is_valid());
        if (kind == Surface::Kind::Nurbs)
            continue;
        const auto [u0, u1] = nurbs.domain(0);
        const auto [v0, v1] = nurbs.domain(1);
        double worst = 0.0;
        for (int i = 0; i <= 8; ++i)
            for (int j = 0; j <= 8; ++j)
                worst = std::max(worst, s.distance(nurbs.point_at(u0 + (u1 - u0) * i / 8, v0 + (v1 - v0) * j / 8)));
        MINI_CHECK(worst < 1e-9);
        MINI_CHECK(std::abs(s.domain(0).second - s.domain(0).first - 2.0 * PI) < 1e-9);
        MINI_CHECK(std::abs(AABB::from_nurbssurface(nurbs).hz - s.aabb().hz) < 1e-6);
    }
    int recognised = 0;
    for (const Surface& s : analytic_surfaces()) {
        // The recogniser of legacy input does not find every partial cone or torus: those stay freeform.
        const Surface again = Surface::recognized(s.nurbs());
        recognised += again.kind() != Surface::Kind::Nurbs;
        if (again.kind() == Surface::Kind::Nurbs)
            continue;
        MINI_CHECK(again.kind() == s.kind() && s.coincident(again, 1e-7) != 0);
        for (int dir = 0; dir < 2 && s.kind() != Surface::Kind::Sphere; ++dir)
            MINI_CHECK(std::abs((again.domain(dir).second - again.domain(dir).first) - (s.domain(dir).second - s.domain(dir).first)) < 1e-7 * size(s));
    }
    MINI_CHECK(recognised >= 5);
}

MINI_TEST("Surface", "Json") {
    std::vector<Surface> all = analytic_surfaces();
    all.push_back(Surface(Primitives::cylinder_surface(0, 0, 0, 1, 2)));
    for (const Surface& s : all) {
        const Surface loaded = Surface::jsonload(nlohmann::json::parse(s.jsondump().dump()));
        MINI_CHECK(loaded.kind() == s.kind() && loaded.domain(0) == s.domain(0) && loaded.domain(1) == s.domain(1));
        MINI_CHECK(loaded.point_at(s.domain(0).first, s.domain(1).second).distance(s.point_at(s.domain(0).first, s.domain(1).second)) < 1e-12);
    }
    MINI_CHECK(!Surface().is_valid() && !Surface::jsonload(Surface().jsondump()).is_valid());
}

namespace {

// Profiles in the plane of the axis Z and X, one across it and one freeform through space.
std::vector<Curve> profiles() {
    const Vector x = X * (1.0 / X.magnitude()), z = Z * (1.0 / Z.magnitude()), y = z.cross(x);
    return {Curve::line(O + x * 2.0, x * 0.3 + z, 0.0, 3.0), Curve::circle(O + x * 5.0, x, z, 1.5), Curve::circle(O + x * 5.0, x, z, 1.5, 0.5, 2.0), Curve::ellipse(O + x * 4.0 + z, z, x, 2.0, 1.0, -1.0, 1.0),
        Curve::parabola(O, z, x, 0.6, 0.5, 3.0), Curve::hyperbola(O, x, z, 2.0, 1.5, -1.0, 1.0), Curve::line(O + x * 2.0, y + z * 0.5, -2.0, 2.0),
        Curve(NurbsCurve::create(false, 3, {O + x * 1.0, O + x * 2.5 + z * 1.0 + y * 0.4, O + x * 1.5 + z * 2.0 - y * 0.3, O + x * 3.0 + z * 3.0, O + x * 2.0 + z * 4.5 + y * 0.2}))};
}

std::vector<Surface> sweeps() {
    std::vector<Surface> all;
    for (const Curve& c : profiles()) {
        all.push_back(Surface::revolution(O, Z, c));
        all.push_back(Surface::revolution(O, Z, c, 1.0, 2.5));
        all.push_back(Surface::extrusion(c, Vector(0.3, -1.0, 0.8), -1.0, 2.0));
    }
    return all;
}

}

MINI_TEST("Surface", "Sweep Nurbs") {
    for (const Surface& s : sweeps()) {
        const NurbsSurface& n = s.nurbs();
        const auto [u0, u1] = s.domain(0);
        const auto [v0, v1] = s.domain(1);
        const bool turned = s.kind() == Surface::Kind::Revolution;
        MINI_CHECK(s.is_valid() && n.is_valid() && &s.nurbs() == &n && n.domain(0) == s.domain(0) && n.domain(1) == s.domain(1));
        MINI_CHECK(s.profile().is_valid() && s.profile().domain() == s.domain(turned ? 1 : 0) && n.degree(turned ? 0 : 1) == (turned ? 2 : 1));
        double worst = 0.0, corners = 0.0;
        for (int i = 0; i <= 12; ++i)
            for (int j = 0; j <= 12; ++j)
                worst = std::max(worst, s.distance(n.point_at(u0 + (u1 - u0) * i / 12, v0 + (v1 - v0) * j / 12)));
        for (double u : {u0, u1})
            for (double v : {v0, v1})
                corners = std::max(corners, n.point_at(u, v).distance(s.point_at(u, v)));
        MINI_CHECK(worst <= 1e-12 * 20.0 && corners <= 1e-12 * 20.0);
        MINI_CHECK(std::abs(s.aabb().hx - AABB::from_nurbssurface(n).hx) < 1e-12);
        // Along a profile whose NURBS runs in its own parameter, and along the length, the two agree point for point.
        const Curve::Kind kind = s.profile().kind();
        if (!turned && kind != Curve::Kind::Circle && kind != Curve::Kind::Ellipse && kind != Curve::Kind::Hyperbola)
            MINI_CHECK(n.point_at(0.3 * u0 + 0.7 * u1, 0.6 * v0 + 0.4 * v1).distance(s.point_at(0.3 * u0 + 0.7 * u1, 0.6 * v0 + 0.4 * v1)) <= 1e-12 * 20.0);
    }
}

MINI_TEST("Surface", "Sweep Evaluate") {
    for (const Surface& s : sweeps()) {
        const double u = 0.5 * (s.domain(0).first + s.domain(0).second) + 0.1, v = 0.5 * (s.domain(1).first + s.domain(1).second) + 0.1, h = 1e-4;
        const std::vector<Vector> d = s.evaluate(u, v, 2);
        MINI_CHECK(d.size() == 6 && s.evaluate(u, v, 1).size() == 3 && s.evaluate(u, v).size() == 1);
        const auto at = [&](double a, double b) { const Point p = s.point_at(u + a * h, v + b * h); return Vector(p[0], p[1], p[2]); };
        MINI_CHECK(((at(0, 1) - at(0, -1)) * (0.5 / h) - d[1]).magnitude() < 1e-6 * 20.0);
        MINI_CHECK(((at(0, 1) + at(0, -1) - at(0, 0) * 2.0) * (1.0 / (h * h)) - d[2]).magnitude() < 1e-5 * 20.0);
        MINI_CHECK(((at(1, 0) - at(-1, 0)) * (0.5 / h) - d[3]).magnitude() < 1e-6 * 20.0);
        MINI_CHECK(((at(1, 1) - at(1, -1) - at(-1, 1) + at(-1, -1)) * (0.25 / (h * h)) - d[4]).magnitude() < 1e-5 * 20.0);
        MINI_CHECK(((at(1, 0) + at(-1, 0) - at(0, 0) * 2.0) * (1.0 / (h * h)) - d[5]).magnitude() < 1e-5 * 20.0);
        const Vector n = s.normal_at(u, v);
        MINI_CHECK(std::abs(n.magnitude() - 1.0) < 1e-12 && std::abs(n.dot(d[1])) < 1e-12 * 20.0 && std::abs(n.dot(d[3])) < 1e-12 * 20.0 && n.dot(d[3].cross(d[1])) > 0.0);
        const std::pair<double, double> uv = s.closest_parameters(s.nurbs().point_at(u, v));
        MINI_CHECK(s.normal_at(uv.first, uv.second).dot(s.nurbs().normal_at(u, v)) > 1.0 - 1e-9);
    }
    // Geom_SurfaceOfRevolution: the profile itself at u = 0, turned from x toward y of a right-handed frame about the axis.
    const Surface turned = Surface::revolution(Point(0, 0, 0), Vector(0, 0, 1), Curve::line(Point(2, 0, 0), Point(2, 0, 1)));
    MINI_CHECK(turned.point_at(0.0, 0.5).distance(Point(2, 0, 0.5)) < 1e-15 && turned.point_at(0.5 * PI, 0.5).distance(Point(0, 2, 0.5)) < 1e-15 && turned.right_handed());
    const Surface moved = Surface::extrusion(Curve::line(Point(2, 0, 0), Point(2, 1, 0)), Vector(0, 0, 3));
    MINI_CHECK(moved.point_at(0.5, 2.0).distance(Point(2, 0.5, 2)) < 1e-15 && moved.z()[2] == 1.0);
}

MINI_TEST("Surface", "Sweep Closest Parameters") {
    for (const Surface& s : sweeps()) {
        const auto [u0, u1] = s.domain(0);
        const auto [v0, v1] = s.domain(1);
        for (int i = 1; i < 8; ++i)
            for (int j = 1; j < 8; ++j) {
                const double u = u0 + (u1 - u0) * i / 8.1, v = v0 + (v1 - v0) * j / 8.3;
                const Point p = s.point_at(u, v);
                const std::pair<double, double> uv = s.closest_parameters(p);
                MINI_CHECK(std::abs(uv.first - u) < 1e-12 * 20.0 && std::abs(uv.second - v) < 1e-12 * 20.0);
                const Point lifted = p + s.normal_at(u, v) * 0.05;
                MINI_CHECK(s.closest_point(lifted).distance(p) < 1e-12 * 20.0 && std::abs(s.distance(lifted) - 0.05) < 1e-12 * 20.0);
            }
    }
}

MINI_TEST("Surface", "Sweep Period") {
    for (const Surface& s : sweeps()) {
        const bool turned = s.kind() == Surface::Kind::Revolution;
        const double along = s.profile().period();
        MINI_CHECK(s.period(0) == (turned ? 2.0 * PI : along) && s.period(1) == (turned ? along : 0.0));
        const double u = s.domain(0).first + 0.2, v = s.domain(1).first + 0.3;
        MINI_CHECK(s.point_at(u + s.period(0), v + s.period(1)).distance(s.point_at(u, v)) < 1e-12 * 20.0);
        if (turned)
            MINI_CHECK(std::abs(s.closest_parameters(s.point_at(s.domain(0).first - 0.25, v)).first - (s.domain(0).first - 0.25 + 2.0 * PI)) < 1e-12 * 20.0);
        MINI_CHECK(std::abs(s.adjusted(u - s.period(0), v, {s.domain(0).first, s.domain(0).second, s.domain(1).first, s.domain(1).second}).first - u) < 1e-12);
    }
}

MINI_TEST("Surface", "Sweep Transform") {
    const Xform rigid = Xform::translation(1.0, -4.0, 2.0) * Xform::rotation_z(0.8) * Xform::rotation_x(-0.3);
    const Xform mirror = Xform::scale_xyz(1.0, -1.0, 1.0), stretch = Xform::scale_xyz(1.0, 2.0, 1.0), grow = Xform::scale_xyz(2.0, 2.0, 2.0);
    for (const Surface& s : sweeps()) {
        const bool turned = s.kind() == Surface::Kind::Revolution, freeform = s.profile().kind() == Curve::Kind::Nurbs;
        const double u = 0.4 * s.domain(0).first + 0.6 * s.domain(0).second, v = 0.7 * s.domain(1).first + 0.3 * s.domain(1).second;
        for (const Xform& x : {rigid, mirror}) {
            const Surface moved = s.transformed(x);
            MINI_CHECK(moved.kind() == s.kind() && std::abs(moved.domain(0).second - s.domain(0).second) < 1e-12 && std::abs(moved.domain(1).first - s.domain(1).first) < 1e-12);
            MINI_CHECK(moved.point_at(u, v).distance(x.transform_point(s.point_at(u, v))) < 1e-12 * 20.0);
            MINI_CHECK(moved.distance(moved.nurbs().point_at(u, v)) < 1e-12 * 20.0);
        }
        // Lengths grow with the surface: the length of an extrusion, the parameter of a line or parabola.
        const Surface grown = s.transformed(grow);
        const Curve::Kind kind = s.profile().kind();
        const double along = kind == Curve::Kind::Line || kind == Curve::Kind::Parabola ? 2.0 : 1.0;
        MINI_CHECK(grown.kind() == s.kind() && grown.point_at(turned ? u : along * u, turned ? along * v : 2.0 * v).distance(grow.transform_point(s.point_at(u, v))) < 1e-12 * 40.0);
        const Surface stretched = s.transformed(stretch);
        MINI_CHECK(stretched.kind() == (turned ? Surface::Kind::Nurbs : Surface::Kind::Extrusion) && stretched.is_valid());
        MINI_CHECK(stretched.nurbs().point_at(s.domain(0).first, s.domain(1).second).distance(stretch.transform_point(s.point_at(s.domain(0).first, s.domain(1).second))) < 1e-12 * 40.0 || !turned);
        MINI_CHECK(turned || stretched.distance(stretch.transform_point(s.point_at(u, v))) < 1e-12 * 40.0);
        for (int dir = 0; dir < 2; ++dir) {
            Surface back = s;
            back.reverse(dir);
            // A freeform profile turns round inside its domain.
            const bool kept = freeform && dir == (turned ? 1 : 0);
            const double bu = dir == 0 ? (kept ? s.domain(0).first + s.domain(0).second - u : -u) : u, bv = dir == 1 ? (kept ? s.domain(1).first + s.domain(1).second - v : -v) : v;
            MINI_CHECK(back.point_at(bu, bv).distance(s.point_at(u, v)) < 1e-12 * 20.0 && back.normal_at(bu, bv).dot(s.normal_at(u, v)) < -1.0 + 1e-12);
            MINI_CHECK(kept ? back.domain(dir) == s.domain(dir) : back.domain(dir).first == -s.domain(dir).second && back.domain(dir).second == -s.domain(dir).first);
            MINI_CHECK(back.nurbs().is_valid() && s.coincident(back, 1e-9) == -1);
        }
    }
}

MINI_TEST("Surface", "Sweep Coincident") {
    const Vector x = X * (1.0 / X.magnitude()), z = Z * (1.0 / Z.magnitude()), y = z.cross(x);
    const Surface tube = Surface::revolution(O, Z, Curve::line(O + x * 2.5, z, -1.0, 3.0)), cylinder = Surface::cylinder(O, Z, X, 2.5, {0.0, 2.0 * PI, -1.0, 3.0});
    MINI_CHECK(tube.coincident(cylinder, 1e-9) == 1 && cylinder.coincident(tube, 1e-9) == 1 && tube.coincident(Surface::cylinder(O, Z, X, 2.5 + 1e-6), 1e-9) == 0);
    MINI_CHECK(tube.coincident(Surface::revolution(O + z, Z * -1.0, Curve::line(O - y * 2.5, z, 0.0, 1.0)), 1e-9) == -1);
    MINI_CHECK(Surface::extrusion(Curve::circle(O, x, y, 2.5), Z, -1.0, 3.0).coincident(cylinder, 1e-9) == 1);
    MINI_CHECK(Surface::extrusion(Curve::circle(O, x, y, 2.5), Z + X * 1e-3, -1.0, 3.0).coincident(cylinder, 1e-9) == 0);
    MINI_CHECK(Surface::revolution(O, Z, Curve::circle(O + x * 5.0, x, z, 1.5)).coincident(Surface::torus(O, Z, X, 5.0, 1.5), 1e-9) == 1);
    MINI_CHECK(Surface::revolution(O, Z, Curve::circle(O, x, z, 2.0, -0.5 * PI, 0.5 * PI)).coincident(Surface::sphere(O, Z, X, 2.0), 1e-9) == 1);
    MINI_CHECK(Surface::revolution(O, Z, Curve::line(O + x * 1.5, x * std::sin(0.4) + z * std::cos(0.4), 0.0, 2.0)).coincident(Surface::cone(O, Z, X, 1.5, 0.4, {0.0, 2.0 * PI, 0.0, 2.0}), 1e-9) == 1);
    MINI_CHECK(std::abs(Surface::extrusion(Curve::line(O, O + x), y, -1.0, 1.0).coincident(Surface::plane(O, Z, X), 1e-9)) == 1);
    // A line square to the axis turns into its plane.
    MINI_CHECK(Surface::revolution(O, Z, Curve::line(O + x, O + x * 3.0)).coincident(Surface::plane(O, Z, X, {-3.0, 3.0, -3.0, 3.0}), 1e-9) != 0);
    MINI_CHECK(Surface::revolution(O, Z, Curve::line(O + x, O + x * 3.0)).coincident(Surface::plane(O + z * 1e-3, Z, X, {-3.0, 3.0, -3.0, 3.0}), 1e-9) == 0);
    MINI_CHECK(tube.coincident(Surface(Primitives::cylinder_surface(0, 0, 0, 1, 2)), 1e-9) == 0);
}

MINI_TEST("Surface", "Sweep Json") {
    for (const Surface& s : sweeps()) {
        const Surface loaded = Surface::jsonload(nlohmann::json::parse(s.jsondump().dump()));
        MINI_CHECK(loaded.kind() == s.kind() && loaded.is_valid() && loaded.domain(0) == s.domain(0) && loaded.domain(1) == s.domain(1) && loaded.profile().kind() == s.profile().kind());
        MINI_CHECK(loaded.point_at(s.domain(0).first, s.domain(1).second).distance(s.point_at(s.domain(0).first, s.domain(1).second)) < 1e-12);
    }
    nlohmann::json cut = Surface::revolution(O, Z, Curve::line(O + X, O + X + Z)).jsondump();
    cut.erase("profile");
    MINI_CHECK(!Surface::jsonload(cut).is_valid());
}

}
