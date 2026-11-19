#include "curve.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace session_cpp {

namespace {

const double TWO_PI = 2.0 * std::numbers::pi;
const char* const NAMES[] = {"nurbs", "line", "circle", "ellipse"};

Vector unit(const Vector& v) {
    const double length = v.magnitude();
    return length > 0.0 ? v * (1.0 / length) : v;
}

nlohmann::ordered_json triple(const Point& p) {
    return {p[0], p[1], p[2]};
}

}

Curve::Curve(const NurbsCurve& nurbs) : m_domain{nurbs.domain().first, nurbs.domain().second}, m_nurbs(std::make_shared<const NurbsCurve>(nurbs)) {}

Curve Curve::line(const Point& origin, const Vector& direction, double t0, double t1) {
    Curve c;
    c.m_kind = Kind::Line;
    c.m_origin = origin;
    c.m_x = unit(direction);
    c.m_domain = {t0, t1};
    return c;
}

Curve Curve::line(const Point& a, const Point& b) {
    return line(a, b - a, 0.0, a.distance(b));
}

Curve Curve::circle(const Point& center, const Vector& x, const Vector& y, double radius, double t0, double t1) {
    Curve c = ellipse(center, x, y, radius, radius, t0, t1);
    c.m_kind = Kind::Circle;
    return c;
}

Curve Curve::ellipse(const Point& center, const Vector& x, const Vector& y, double radius, double minor, double t0, double t1) {
    Curve c;
    c.m_kind = Kind::Ellipse;
    c.m_origin = center;
    c.m_x = unit(x);
    c.m_y = unit(y - c.m_x * y.dot(c.m_x));
    c.m_radius = radius;
    c.m_minor = minor;
    c.m_domain = {t0, t1};
    return c;
}

Curve Curve::recognized(const NurbsCurve& nurbs, double tolerance) {
    if (!nurbs.is_valid())
        return Curve(nurbs);
    std::vector<Point> p;
    const auto [s0, s1] = nurbs.domain();
    for (int i = 0; i <= 16; ++i)
        p.push_back(nurbs.point_at(s0 + (s1 - s0) * i / 16));
    double size = 0.0;
    for (const Point& q : p)
        size = std::max(size, q.distance(p[0]));
    const double slack = tolerance * size;
    const auto on = [&](const Curve& c) { return std::all_of(p.begin(), p.end(), [&](const Point& q) { return c.closest_point(q).distance(q) <= slack; }); };
    if (!(size > 0.0))
        return Curve(nurbs);
    if (const Curve c = line(p[0], p.back()); p.back().distance(p[0]) > 0.5 * size && on(c))
        return c;
    // The plane of three spread samples, then the conic through five in it: A x^2 + B x y + C y^2 + D x + E y = 1 about their mean.
    Point mean(0, 0, 0);
    for (int i : {0, 4, 8, 12, 15})
        mean += (p[i] - Point(0, 0, 0)) * 0.2;
    const Vector e1 = unit(p[5] - p[0]), normal = unit(e1.cross(p[10] - p[0]));
    if (!(normal.magnitude() > 0.5))
        return Curve(nurbs);
    const Vector e2 = normal.cross(e1);
    std::array<std::array<double, 6>, 5> m;
    int row = 0;
    for (int i : {0, 4, 8, 12, 15}) {
        const double x = (p[i] - mean).dot(e1) / size, y = (p[i] - mean).dot(e2) / size;
        m[row++] = {x * x, x * y, y * y, x, y, 1.0};
    }
    for (int k = 0; k < 5; ++k) {
        int pivot = k;
        for (int i = k + 1; i < 5; ++i)
            if (std::abs(m[i][k]) > std::abs(m[pivot][k]))
                pivot = i;
        std::swap(m[k], m[pivot]);
        if (!(std::abs(m[k][k]) > 1e-14))
            return Curve(nurbs);
        for (int i = 0; i < 5; ++i)
            if (i != k)
                for (int j = 5; j >= k; --j)
                    m[i][j] -= m[k][j] * m[i][k] / m[k][k];
    }
    const double A = m[0][5] / m[0][0], B = m[1][5] / m[1][1], C = m[2][5] / m[2][2], D = m[3][5] / m[3][3], E = m[4][5] / m[4][4];
    const double det = 4.0 * A * C - B * B;
    if (!(det > 0.0))
        return Curve(nurbs);
    const double cx = (B * E - 2.0 * C * D) / det, cy = (B * D - 2.0 * A * E) / det;
    const double level = 1.0 - (A * cx * cx + B * cx * cy + C * cy * cy + D * cx + E * cy);
    const double mid = 0.5 * (A + C), half = std::hypot(0.5 * (A - C), 0.5 * B);
    if (!(level / (mid - half) > 0.0) || !(level / (mid + half) > 0.0))
        return Curve(nurbs);
    const double major = std::sqrt(level / (mid - half)) * size, minor = std::sqrt(level / (mid + half)) * size;
    const double turn = 0.5 * std::atan2(B, A - C) + std::numbers::pi / 2;
    const Point center = mean + e1 * (cx * size) + e2 * (cy * size);
    const bool round = major - minor <= slack;
    const Vector x = round ? unit(p[0] - center) : e1 * std::cos(turn) + e2 * std::sin(turn);
    // y on the side the curve leaves toward, so that it runs with the angle.
    const Vector z = (p[0] - center).cross(p[1] - center).dot(normal) < 0.0 ? normal * -1.0 : normal;
    Curve c = round ? circle(center, x, z.cross(x), major) : ellipse(center, x, z.cross(x), major, minor);
    if (!on(c))
        return Curve(nurbs);
    // The extent: each sample's angle follows the one before it by less than a turn.
    double t0 = c.closest_parameter(p[0]), t = t0;
    for (size_t i = 1; i < p.size(); ++i)
        t = wrapped(c.closest_parameter(p[i]), t - 1e-9, TWO_PI);
    c.m_domain = {t0, p.back().distance(p[0]) <= slack ? t0 + TWO_PI : t};
    return c;
}

bool Curve::is_valid() const {
    return m_kind == Kind::Nurbs ? m_nurbs && m_nurbs->is_valid() : m_kind == Kind::Line || (m_radius > 0.0 && m_minor > 0.0);
}

std::pair<double, double> Curve::domain() const {
    return {m_domain[0], m_domain[1]};
}

bool Curve::set_domain(double t0, double t1) {
    if (!(t0 < t1))
        return false;
    if (m_kind == Kind::Nurbs) {
        NurbsCurve trimmed = *m_nurbs;
        if (!trimmed.trim(t0, t1))
            return false;
        m_nurbs = std::make_shared<const NurbsCurve>(std::move(trimmed));
    } else
        m_nurbs.reset();
    m_domain = {t0, t1};
    return true;
}

double Curve::period() const {
    return m_kind == Kind::Circle || m_kind == Kind::Ellipse ? TWO_PI : 0.0;
}

bool Curve::is_closed() const {
    return m_kind == Kind::Nurbs ? m_nurbs && m_nurbs->is_closed() : m_kind != Kind::Line && m_domain[1] - m_domain[0] >= TWO_PI * (1.0 - 1e-15);
}

double Curve::wrapped(double t, double start, double period) {
    if (!(period > 0.0))
        return t;
    t -= period * std::floor((t - start) / period);
    return t < start ? start : t >= start + period ? t - period : t;
}

Point Curve::point_at(double t) const {
    switch (m_kind) {
    case Kind::Nurbs: return m_nurbs->point_at(t);
    case Kind::Line: return m_origin + m_x * t;
    default: return m_origin + m_x * (m_radius * std::cos(t)) + m_y * (m_minor * std::sin(t));
    }
}

std::vector<Vector> Curve::evaluate(double t, int derivative_count) const {
    if (m_kind == Kind::Nurbs)
        return m_nurbs->evaluate(t, derivative_count);
    const Point p = point_at(t);
    std::vector<Vector> result = {Vector(p[0], p[1], p[2])};
    const double c = m_radius * std::cos(t), s = m_minor * std::sin(t);
    if (derivative_count > 0)
        result.push_back(m_kind == Kind::Line ? m_x : m_x * (-m_radius * std::sin(t)) + m_y * (m_minor * std::cos(t)));
    if (derivative_count > 1)
        result.push_back(m_kind == Kind::Line ? Vector(0, 0, 0) : m_x * -c + m_y * -s);
    return result;
}

Vector Curve::tangent_at(double t) const {
    return m_kind == Kind::Nurbs ? m_nurbs->tangent_at(t) : unit(evaluate(t, 1)[1]);
}

double Curve::closest_parameter(const Point& point) const {
    if (m_kind == Kind::Nurbs)
        return m_nurbs->closest_parameter(point);
    const Vector d = point - m_origin;
    if (m_kind == Kind::Line)
        return d.dot(m_x);
    const double u = d.dot(m_x), v = d.dot(m_y);
    double t = std::atan2(v / m_minor, u / m_radius);
    // On an ellipse the foot of the normal: Newton on (C(t) - point) . C'(t) from the angle of the scaled point.
    for (int i = 0; i < 50 && m_kind == Kind::Ellipse; ++i) {
        const double c = std::cos(t), s = std::sin(t), dx = m_radius * c - u, dy = m_minor * s - v;
        const double f = -dx * m_radius * s + dy * m_minor * c;
        const double slope = m_radius * m_radius * s * s + m_minor * m_minor * c * c - dx * m_radius * c - dy * m_minor * s;
        if (!(slope > 0.0) || std::abs(f / slope) <= 1e-16)
            break;
        t -= f / slope;
    }
    return wrapped(t, m_domain[0], TWO_PI);
}

Point Curve::closest_point(const Point& point) const {
    return point_at(closest_parameter(point));
}

AABB Curve::aabb() const {
    return AABB::from_nurbscurve(nurbs(), 0.0, true);
}

double Curve::similarity(const Xform& xform) {
    const Vector a = xform.transform_vector(Vector(1, 0, 0)), b = xform.transform_vector(Vector(0, 1, 0)), c = xform.transform_vector(Vector(0, 0, 1));
    const double s = a.magnitude(), slack = 1e-12 * s * s;
    return s > 0.0 && std::abs(a.dot(b)) <= slack && std::abs(a.dot(c)) <= slack && std::abs(b.dot(c)) <= slack &&
        std::abs(b.dot(b) - s * s) <= slack && std::abs(c.dot(c) - s * s) <= slack ? s : 0.0;
}

void Curve::transform(const Xform& xform) {
    const double s = m_kind == Kind::Line ? xform.transform_vector(m_x).magnitude() : similarity(xform);
    if (m_kind == Kind::Nurbs || !(s > 0.0)) {
        NurbsCurve moved = nurbs();
        moved.transform(xform);
        *this = Curve(moved);
        return;
    }
    m_origin = xform.transform_point(m_origin);
    m_x = unit(xform.transform_vector(m_x));
    m_y = unit(xform.transform_vector(m_y));
    m_radius *= s;
    m_minor *= s;
    if (m_kind == Kind::Line)
        m_domain = {m_domain[0] * s, m_domain[1] * s};
    m_nurbs.reset();
}

Curve Curve::transformed(const Xform& xform) const {
    Curve c = *this;
    c.transform(xform);
    return c;
}

void Curve::reverse() {
    if (m_kind == Kind::Nurbs) {
        NurbsCurve reversed = *m_nurbs;
        reversed.reverse();
        m_nurbs = std::make_shared<const NurbsCurve>(std::move(reversed));
        return;
    }
    (m_kind == Kind::Line ? m_x : m_y) *= -1.0;
    m_domain = {-m_domain[1], -m_domain[0]};
    m_nurbs.reset();
}

void Curve::arc(double a0, double a1, std::vector<std::array<double, 3>>& controls, std::vector<double>& knots) {
    const int spans = std::max(1, static_cast<int>(std::ceil((a1 - a0) / (std::numbers::pi / 2) - 1e-9)));
    const double step = (a1 - a0) / spans, weight = std::cos(0.5 * step);
    controls.clear();
    knots.clear();
    for (int k = 0; k <= spans; ++k) {
        const double a = k == spans ? a1 : a0 + k * step;
        if (k > 0)
            controls.push_back({std::cos(a - 0.5 * step) / weight, std::sin(a - 0.5 * step) / weight, weight});
        controls.push_back({std::cos(a), std::sin(a), 1.0});
        knots.insert(knots.end(), 2, a);
    }
}

const NurbsCurve& Curve::nurbs() const {
    static const NurbsCurve invalid;
    if (m_nurbs || !is_valid() || !(m_domain[0] < m_domain[1]))
        return m_nurbs ? *m_nurbs : invalid;
    NurbsCurve c;
    if (m_kind == Kind::Line) {
        c = NurbsCurve::create(false, 1, {point_at(m_domain[0]), point_at(m_domain[1])});
        c.set_domain(m_domain[0], m_domain[1]);
    } else {
        std::vector<std::array<double, 3>> controls;
        std::vector<double> knots;
        arc(m_domain[0], m_domain[1], controls, knots);
        c = NurbsCurve(3, true, 3, static_cast<int>(controls.size()));
        for (size_t i = 0; i < knots.size(); ++i)
            c.set_nurbsknot(static_cast<int>(i), knots[i]);
        for (size_t i = 0; i < controls.size(); ++i) {
            const Point p = m_origin + m_x * (m_radius * controls[i][0]) + m_y * (m_minor * controls[i][1]);
            const double w = controls[i][2];
            c.set_cv_4d(static_cast<int>(i), p[0] * w, p[1] * w, p[2] * w, w);
        }
    }
    m_nurbs = std::make_shared<const NurbsCurve>(std::move(c));
    return *m_nurbs;
}

nlohmann::ordered_json Curve::jsondump() const {
    nlohmann::ordered_json j;
    j["domain"] = {m_domain[0], m_domain[1]};
    j["kind"] = NAMES[static_cast<int>(m_kind)];
    if (m_kind == Kind::Nurbs)
        j["nurbs"] = m_nurbs ? m_nurbs->jsondump() : nlohmann::ordered_json();
    else {
        j["minor"] = m_minor;
        j["origin"] = triple(m_origin);
        j["radius"] = m_radius;
    }
    j["type"] = "Curve";
    if (m_kind != Kind::Nurbs) {
        j["x"] = triple(Point(m_x[0], m_x[1], m_x[2]));
        j["y"] = triple(Point(m_y[0], m_y[1], m_y[2]));
    }
    return j;
}

Curve Curve::jsonload(const nlohmann::json& data) {
    const std::string kind = data.value("kind", "nurbs");
    if (kind == "nurbs")
        return data.contains("nurbs") && data["nurbs"].is_object() ? Curve(NurbsCurve::jsonload(data["nurbs"])) : Curve();
    const auto vector = [&](const char* key) { return Vector(data.at(key).at(0), data.at(key).at(1), data.at(key).at(2)); };
    const Vector o = vector("origin");
    Curve c;
    c.m_kind = static_cast<Kind>(std::find_if(std::begin(NAMES), std::end(NAMES), [&](const char* name) { return kind == name; }) - std::begin(NAMES));
    c.m_origin = Point(o[0], o[1], o[2]);
    c.m_x = vector("x");
    c.m_y = vector("y");
    c.m_radius = data.at("radius");
    c.m_minor = data.at("minor");
    c.m_domain = {data.at("domain").at(0), data.at("domain").at(1)};
    return c.m_kind > Kind::Ellipse ? Curve() : c;
}

Curve2d Curve2d::line(double u0, double v0, double u1, double v1) {
    return Curve::line(Point(u0, v0, 0), Point(u1, v1, 0));
}

Curve2d Curve2d::shifted(double du, double dv) const {
    return transformed(Xform::translation(du, dv, 0));
}

} // namespace session_cpp
