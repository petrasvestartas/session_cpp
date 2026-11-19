#include "surface.h"
#include "nurbs_implicit.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace session_cpp {

namespace {

const double TWO_PI = 2.0 * std::numbers::pi;
const char* const NAMES[] = {"nurbs", "plane", "cylinder", "cone", "sphere", "torus"};

Vector unit(const Vector& v) {
    const double length = v.magnitude();
    return length > 0.0 ? v * (1.0 / length) : v;
}

Vector as_vector(const Point& p) {
    return Vector(p[0], p[1], p[2]);
}

}

Surface::Surface(const NurbsSurface& nurbs)
    : m_domain{nurbs.domain(0).first, nurbs.domain(0).second, nurbs.domain(1).first, nurbs.domain(1).second}, m_nurbs(std::make_shared<const NurbsSurface>(nurbs)) {}

Surface Surface::analytic(Kind kind, const Point& origin, const Vector& z, const Vector& x, double radius, double minor, const std::array<double, 4>& domain) {
    Surface s;
    s.m_kind = kind;
    s.m_origin = origin;
    s.m_z = unit(z);
    s.m_x = unit(x - s.m_z * x.dot(s.m_z));
    s.m_y = s.m_z.cross(s.m_x);
    s.m_radius = radius;
    s.m_minor = minor;
    s.m_domain = domain;
    return s;
}

Surface Surface::plane(const Point& origin, const Vector& z, const Vector& x, const std::array<double, 4>& domain) {
    return analytic(Kind::Plane, origin, z, x, 0.0, 0.0, domain);
}

Surface Surface::cylinder(const Point& origin, const Vector& z, const Vector& x, double radius, const std::array<double, 4>& domain) {
    return analytic(Kind::Cylinder, origin, z, x, radius, 0.0, domain);
}

Surface Surface::cone(const Point& origin, const Vector& z, const Vector& x, double radius, double semi_angle, const std::array<double, 4>& domain) {
    return analytic(Kind::Cone, origin, z, x, radius, semi_angle, domain);
}

Surface Surface::sphere(const Point& origin, const Vector& z, const Vector& x, double radius, const std::array<double, 4>& domain) {
    return analytic(Kind::Sphere, origin, z, x, radius, 0.0, domain);
}

Surface Surface::torus(const Point& origin, const Vector& z, const Vector& x, double radius, double minor, const std::array<double, 4>& domain) {
    return analytic(Kind::Torus, origin, z, x, radius, minor, domain);
}

Surface Surface::recognized(const NurbsSurface& nurbs, double tolerance) {
    if (!nurbs.is_valid())
        return Surface(nurbs);
    const nurbs_implicit::Implicit f = nurbs_implicit::recognize(nurbs, tolerance);
    const auto [s0, s1] = nurbs.domain(0);
    const auto [t0, t1] = nurbs.domain(1);
    const Point corner = nurbs.point_at(s0, t0), point(f.point[0], f.point[1], f.point[2]);
    Vector axis(f.axis[0], f.axis[1], f.axis[2]);
    Surface s;
    if (f.kind == 1) {
        if (axis.dot(nurbs.normal_at(0.5 * (s0 + s1), 0.5 * (t0 + t1))) < 0.0)
            axis *= -1.0;
        s = plane(corner, axis, nurbs.point_at(s1, t0) - corner);
    } else if (f.kind == 2) {
        // A sphere has no axis of its own: the poles of the patch where it has them, else the normal of its first corner's meridian.
        const Vector pole = nurbs.point_at(s0, t1) - nurbs.point_at(s0, t0);
        axis = nurbs.point_at(s0, t0).distance(nurbs.point_at(0.5 * (s0 + s1), t0)) <= tolerance && pole.magnitude() > f.radius ? pole : Vector(0, 0, 1);
        const Vector out = nurbs.point_at(s0, 0.5 * (t0 + t1)) - point;
        s = sphere(point, axis, out.cross(axis).magnitude() > 0.5 * f.radius ? out : Vector(axis[1], axis[2], axis[0]), f.radius);
    } else if (f.kind >= 3 && f.kind <= 5) {
        const Point mid = nurbs.point_at(0.5 * (s0 + s1), 0.5 * (t0 + t1));
        if (f.kind == 4 && (mid - point).dot(axis) < 0.0)
            axis *= -1.0;
        Vector out = corner - point;
        if (!((out - axis * out.dot(axis)).magnitude() > tolerance))
            out = mid - point;
        s = f.kind == 3 ? cylinder(point, axis, out, f.radius) : f.kind == 4 ? cone(point, axis, out, 0.0, f.radius) : torus(point, axis, out, f.radius, f.minor);
    } else
        return Surface(nurbs);
    // The extent: the parameters of a grid of the patch, each following its neighbour by less than half a period.
    std::array<double, 4> box = {1e300, -1e300, 1e300, -1e300};
    std::array<std::pair<double, double>, 9> row;
    std::array<bool, 9> known{};
    for (int i = 0; i <= 8; ++i)
        for (int j = 0; j <= 8; ++j) {
            const Point p = nurbs.point_at(s0 + (s1 - s0) * i / 8, t0 + (t1 - t0) * j / 8);
            if (s.distance(p) > 10.0 * tolerance)
                return Surface(nurbs);
            std::pair<double, double> uv = s.closest_parameters(p);
            // At a pole or apex the angle is the neighbour's.
            const bool pole = (f.kind == 2 && std::abs(std::abs(uv.second) - 0.5 * std::numbers::pi) < 1e-9) || (f.kind == 4 && (p - point).magnitude() <= tolerance);
            const int near = j > 0 && known[j - 1] ? j - 1 : j;
            if (known[near])
                uv = s.adjusted(pole ? row[near].first : uv.first, uv.second, {row[near].first, row[near].first, row[near].second, row[near].second});
            if (!pole || known[near]) {
                row[j] = uv;
                known[j] = true;
                box = {std::min(box[0], uv.first), std::max(box[1], uv.first), box[2], box[3]};
            }
            box[2] = std::min(box[2], uv.second);
            box[3] = std::max(box[3], uv.second);
        }
    for (int d = 0; d < 2; ++d)
        if (s.period(d) > 0.0 && box[2 * d + 1] - box[2 * d] > TWO_PI - 1e-9)
            box[2 * d + 1] = box[2 * d] + TWO_PI;
    s.m_domain = box;
    return box[0] < box[1] && box[2] < box[3] ? s : Surface(nurbs);
}

bool Surface::right_handed() const {
    return m_x.cross(m_y).dot(m_z) > 0.0;
}

bool Surface::is_valid() const {
    if (m_kind == Kind::Nurbs)
        return m_nurbs && m_nurbs->is_valid();
    switch (m_kind) {
    case Kind::Plane: return true;
    case Kind::Cone: return m_radius >= 0.0 && std::abs(m_minor) < 0.5 * std::numbers::pi;
    case Kind::Torus: return m_radius > 0.0 && m_minor > 0.0;
    default: return m_radius > 0.0;
    }
}

std::pair<double, double> Surface::domain(int dir) const {
    return {m_domain[2 * dir], m_domain[2 * dir + 1]};
}

bool Surface::set_domain(int dir, double t0, double t1) {
    if (!(t0 < t1) || dir < 0 || dir > 1)
        return false;
    if (m_kind == Kind::Nurbs) {
        NurbsSurface trimmed = *m_nurbs;
        if (!trimmed.trim(dir, {t0, t1}))
            return false;
        m_nurbs = std::make_shared<const NurbsSurface>(std::move(trimmed));
    } else
        m_nurbs.reset();
    m_domain[2 * dir] = t0;
    m_domain[2 * dir + 1] = t1;
    return true;
}

double Surface::period(int dir) const {
    return (dir == 0 && m_kind != Kind::Nurbs && m_kind != Kind::Plane) || (dir == 1 && m_kind == Kind::Torus) ? TWO_PI : 0.0;
}

std::pair<double, double> Surface::adjusted(double u, double v, const std::array<double, 4>& box) const {
    const auto into = [](double t, double lo, double hi, double period) {
        if (!(period > 0.0))
            return t;
        t = Curve::wrapped(t, lo, period);
        // Past the box: the turn nearer to it.
        return t > hi && t - hi > lo - (t - period) ? t - period : t;
    };
    return {into(u, box[0], box[1], period(0)), into(v, box[2], box[3], period(1))};
}

int Surface::coincident(const Surface& other, double tolerance) const {
    if (m_kind == Kind::Nurbs || other.m_kind == Kind::Nurbs)
        return m_kind == other.m_kind && m_nurbs == other.m_nurbs && m_nurbs ? 1 : 0;
    if (m_kind != other.m_kind || std::abs(m_radius - other.m_radius) > tolerance * (m_kind == Kind::Cone ? 1e300 : 1.0) ||
        std::abs(std::abs(m_minor) - std::abs(other.m_minor)) > (m_kind == Kind::Cone ? tolerance / (1.0 + m_radius + other.m_radius) : tolerance))
        return 0;
    const double reach = 1.0 + m_radius;
    double alike = 0.0;
    for (const std::array<double, 2>& uv : {std::array<double, 2>{0.3, 0.2 * reach}, {1.7, -0.4 * reach}, {3.1, 0.9 * reach}, {4.9, -1.1 * reach}}) {
        // A latitude of a sphere; on a cone the side of its origin where the radius grows.
        const double v = m_kind == Kind::Sphere ? 0.5 * std::sin(uv[1]) : m_kind == Kind::Cone ? std::copysign(uv[1], std::sin(other.m_minor)) : uv[1];
        const Point p = other.point_at(uv[0], v);
        if (distance(p) > tolerance)
            return 0;
        const std::pair<double, double> own = closest_parameters(p);
        alike += normal_at(own.first, own.second).dot(other.normal_at(uv[0], v));
    }
    return alike > 0.0 ? 1 : -1;
}

std::array<double, 6> Surface::profile(double v) const {
    switch (m_kind) {
    case Kind::Cylinder: return {m_radius, 0, 0, v, 1, 0};
    case Kind::Cone: return {m_radius + v * std::sin(m_minor), std::sin(m_minor), 0, v * std::cos(m_minor), std::cos(m_minor), 0};
    case Kind::Sphere: return {m_radius * std::cos(v), -m_radius * std::sin(v), -m_radius * std::cos(v), m_radius * std::sin(v), m_radius * std::cos(v), -m_radius * std::sin(v)};
    default: return {m_radius + m_minor * std::cos(v), -m_minor * std::sin(v), -m_minor * std::cos(v), m_minor * std::sin(v), m_minor * std::cos(v), -m_minor * std::sin(v)};
    }
}

Point Surface::point_at(double u, double v) const {
    if (m_kind == Kind::Nurbs)
        return m_nurbs->point_at(u, v);
    if (m_kind == Kind::Plane)
        return m_origin + m_x * u + m_y * v;
    const std::array<double, 6> f = profile(v);
    return m_origin + m_x * (f[0] * std::cos(u)) + m_y * (f[0] * std::sin(u)) + m_z * f[3];
}

std::vector<Vector> Surface::evaluate(double u, double v, int num_derivs) const {
    if (m_kind == Kind::Nurbs)
        return m_nurbs->evaluate(u, v, num_derivs);
    const Vector zero(0, 0, 0);
    std::vector<Vector> result = {as_vector(point_at(u, v))};
    if (m_kind == Kind::Plane) {
        if (num_derivs > 0)
            result.insert(result.end(), num_derivs > 1 ? std::initializer_list<Vector>{m_y, zero, m_x, zero, zero} : std::initializer_list<Vector>{m_y, m_x});
        return result;
    }
    const std::array<double, 6> f = profile(v);
    const Vector e = m_x * std::cos(u) + m_y * std::sin(u), t = m_x * -std::sin(u) + m_y * std::cos(u);
    if (num_derivs > 0)
        result.push_back(e * f[1] + m_z * f[4]);
    if (num_derivs > 1)
        result.push_back(e * f[2] + m_z * f[5]);
    if (num_derivs > 0)
        result.push_back(t * f[0]);
    if (num_derivs > 1)
        result.insert(result.end(), {t * f[1], e * -f[0]});
    return result;
}

Vector Surface::normal_at(double u, double v) const {
    if (m_kind == Kind::Nurbs)
        return m_nurbs->normal_at(u, v);
    if (m_kind == Kind::Plane)
        return m_x.cross(m_y);
    if (m_kind == Kind::Sphere)
        return unit(point_at(u, v) - m_origin) * (right_handed() ? 1.0 : -1.0);
    const std::vector<Vector> d = evaluate(u, v, 1);
    return unit(d[2].cross(d[1]));
}

std::pair<double, double> Surface::closest_parameters(const Point& point) const {
    if (m_kind == Kind::Nurbs)
        return m_nurbs->closest_parameters(point);
    const Vector d = point - m_origin;
    const double a = d.dot(m_x), b = d.dot(m_y), h = d.dot(m_z), rho = std::hypot(a, b);
    if (m_kind == Kind::Plane)
        return {a, b};
    const double u = Curve::wrapped(std::atan2(b, a), m_domain[0], TWO_PI);
    switch (m_kind) {
    case Kind::Cylinder: return {u, h};
    case Kind::Cone: {
        // The nearer nappe (ElSLib::ConeParameters): beyond the apex the point lies at the opposite angle, at -rho.
        const double c = std::cos(m_minor), s = std::sin(m_minor);
        const bool far = std::abs((-rho - m_radius) * c - h * s) < std::abs((rho - m_radius) * c - h * s);
        return {far ? Curve::wrapped(u + 0.5 * TWO_PI, m_domain[0], TWO_PI) : u, ((far ? -rho : rho) - m_radius) * s + h * c};
    }
    case Kind::Sphere: return {u, std::atan2(h, rho)};
    default: return {u, Curve::wrapped(std::atan2(h, rho - m_radius), m_domain[2], TWO_PI)};
    }
}

Point Surface::closest_point(const Point& point) const {
    const std::pair<double, double> uv = closest_parameters(point);
    return point_at(uv.first, uv.second);
}

double Surface::distance(const Point& point) const {
    return closest_point(point).distance(point);
}

AABB Surface::aabb() const {
    return AABB::from_nurbssurface(nurbs());
}

void Surface::transform(const Xform& xform) {
    const Vector x = xform.transform_vector(m_x), y = xform.transform_vector(m_y), z = xform.transform_vector(m_z);
    double s = Curve::similarity(xform);
    const bool flat = m_kind == Kind::Plane && std::abs(x.dot(y)) <= 1e-12 * x.magnitude() * y.magnitude() && x.magnitude() > 0.0 && y.magnitude() > 0.0;
    if (m_kind == Kind::Nurbs || (!(s > 0.0) && !flat)) {
        NurbsSurface moved = nurbs();
        moved.transform(xform);
        *this = Surface(moved);
        return;
    }
    m_origin = xform.transform_point(m_origin);
    if (flat) {
        m_domain = {m_domain[0] * x.magnitude(), m_domain[1] * x.magnitude(), m_domain[2] * y.magnitude(), m_domain[3] * y.magnitude()};
        m_z = unit(x.cross(y)) * (z.dot(x.cross(y)) < 0.0 ? -1.0 : 1.0);
    } else {
        m_z = unit(z);
        m_radius *= s;
        if (m_kind == Kind::Torus)
            m_minor *= s;
        if (m_kind == Kind::Cylinder || m_kind == Kind::Cone)
            m_domain = {m_domain[0], m_domain[1], m_domain[2] * s, m_domain[3] * s};
    }
    m_x = unit(x);
    m_y = unit(y);
    m_nurbs.reset();
}

Surface Surface::transformed(const Xform& xform) const {
    Surface s = *this;
    s.transform(xform);
    return s;
}

void Surface::reverse(int dir) {
    if (m_kind == Kind::Nurbs) {
        NurbsSurface reversed = *m_nurbs;
        reversed.reverse(dir);
        m_nurbs = std::make_shared<const NurbsSurface>(std::move(reversed));
        return;
    }
    if (m_kind == Kind::Plane)
        (dir == 0 ? m_x : m_y) *= -1.0;
    else if (dir == 0)
        m_y *= -1.0;
    else {
        m_z *= -1.0;
        if (m_kind == Kind::Cone)
            m_minor = -m_minor;
    }
    m_domain[2 * dir] = -std::exchange(m_domain[2 * dir + 1], -m_domain[2 * dir]);
    m_nurbs.reset();
}

const NurbsSurface& Surface::nurbs() const {
    static const NurbsSurface invalid;
    if (m_nurbs || !is_valid() || !(m_domain[0] < m_domain[1]) || !(m_domain[2] < m_domain[3]))
        return m_nurbs ? *m_nurbs : invalid;
    // Along u and along v: controls (a, b, weight) and knots. A control is O + a_u x + b_u y on a plane, O + rho (a_u x + b_u y) + h z about an axis with (rho, h) from the control along v.
    std::vector<std::array<double, 3>> cu, cv;
    std::vector<double> ku, kv;
    const bool round = m_kind == Kind::Sphere || m_kind == Kind::Torus;
    if (m_kind == Kind::Plane) {
        cu = {{m_domain[0], 0, 1}, {m_domain[1], 0, 1}};
        ku = {m_domain[0], m_domain[1]};
    } else
        Curve::arc(m_domain[0], m_domain[1], cu, ku);
    if (round) {
        Curve::arc(m_domain[2], m_domain[3], cv, kv);
        const double r = m_kind == Kind::Sphere ? m_radius : m_minor, offset = m_kind == Kind::Sphere ? 0.0 : m_radius;
        for (std::array<double, 3>& c : cv)
            c = {offset + r * c[0], r * c[1], c[2]};
    } else {
        for (double v : {m_domain[2], m_domain[3]})
            cv.push_back(m_kind == Kind::Plane ? std::array<double, 3>{0, v, 1} : std::array<double, 3>{profile(v)[0], profile(v)[3], 1});
        kv = {m_domain[2], m_domain[3]};
    }
    NurbsSurface s(3, m_kind != Kind::Plane, m_kind == Kind::Plane ? 2 : 3, round ? 3 : 2, static_cast<int>(cu.size()), static_cast<int>(cv.size()));
    for (size_t i = 0; i < ku.size(); ++i)
        s.set_nurbsknot(0, static_cast<int>(i), ku[i]);
    for (size_t j = 0; j < kv.size(); ++j)
        s.set_nurbsknot(1, static_cast<int>(j), kv[j]);
    for (size_t i = 0; i < cu.size(); ++i)
        for (size_t j = 0; j < cv.size(); ++j) {
            const Point p = m_kind == Kind::Plane ? m_origin + m_x * cu[i][0] + m_y * cv[j][1]
                : m_origin + m_x * (cv[j][0] * cu[i][0]) + m_y * (cv[j][0] * cu[i][1]) + m_z * cv[j][1];
            const double w = cu[i][2] * cv[j][2];
            if (m_kind == Kind::Plane)
                s.set_cv(static_cast<int>(i), static_cast<int>(j), p);
            else
                s.set_cv_4d(static_cast<int>(i), static_cast<int>(j), p[0] * w, p[1] * w, p[2] * w, w);
        }
    m_nurbs = std::make_shared<const NurbsSurface>(std::move(s));
    return *m_nurbs;
}

nlohmann::ordered_json Surface::jsondump() const {
    const auto triple = [](const auto& p) { return nlohmann::ordered_json{p[0], p[1], p[2]}; };
    nlohmann::ordered_json j;
    j["domain"] = m_domain;
    j["kind"] = NAMES[static_cast<int>(m_kind)];
    if (m_kind == Kind::Nurbs)
        j["nurbs"] = m_nurbs ? m_nurbs->jsondump() : nlohmann::ordered_json();
    else {
        j["minor"] = m_minor;
        j["origin"] = triple(m_origin);
        j["radius"] = m_radius;
    }
    j["type"] = "Surface";
    if (m_kind != Kind::Nurbs) {
        j["x"] = triple(m_x);
        j["y"] = triple(m_y);
        j["z"] = triple(m_z);
    }
    return j;
}

Surface Surface::jsonload(const nlohmann::json& data) {
    const std::string kind = data.value("kind", "nurbs");
    if (kind == "nurbs")
        return data.contains("nurbs") && data["nurbs"].is_object() ? Surface(NurbsSurface::jsonload(data["nurbs"])) : Surface();
    const auto vector = [&](const char* key) { return Vector(data.at(key).at(0), data.at(key).at(1), data.at(key).at(2)); };
    const Vector o = vector("origin");
    Surface s;
    s.m_kind = static_cast<Kind>(std::find_if(std::begin(NAMES), std::end(NAMES), [&](const char* name) { return kind == name; }) - std::begin(NAMES));
    s.m_origin = Point(o[0], o[1], o[2]);
    s.m_x = vector("x");
    s.m_y = vector("y");
    s.m_z = vector("z");
    s.m_radius = data.at("radius");
    s.m_minor = data.at("minor");
    s.m_domain = data.at("domain").get<std::array<double, 4>>();
    return s.m_kind > Kind::Torus ? Surface() : s;
}

} // namespace session_cpp
