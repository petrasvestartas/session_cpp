#include "surface.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace session_cpp {

Surface Surface::revolution(const Point& origin, const Vector& z, const Curve& profile, double u0, double u1) {
    Surface s = analytic(Kind::Revolution, origin, z, std::abs(z[0]) < 0.9 * z.magnitude() ? Vector(1, 0, 0) : Vector(0, 1, 0), 0.0, 0.0, {u0, u1, profile.domain().first, profile.domain().second});
    s.m_profile = std::make_shared<const Curve>(profile);
    return s;
}

Surface Surface::extrusion(const Curve& profile, const Vector& direction, double v0, double v1) {
    Surface s = revolution(Point(0, 0, 0), direction, profile, profile.domain().first, profile.domain().second);
    s.m_kind = Kind::Extrusion;
    s.m_domain[2] = v0;
    s.m_domain[3] = v1;
    return s;
}

Curve Surface::swept() const {
    Curve c = *m_profile;
    const int dir = m_kind == Kind::Extrusion ? 0 : 1;
    if (c.domain() != domain(dir))
        c.set_domain(m_domain[2 * dir], m_domain[2 * dir + 1]);
    return c;
}

std::vector<Vector> Surface::sweep_evaluate(double u, double v, int num_derivs) const {
    const bool turned = m_kind == Kind::Revolution;
    const Vector zero(0, 0, 0), o(m_origin[0], m_origin[1], m_origin[2]);
    std::vector<Vector> c = m_profile->evaluate(turned ? v : u, std::clamp(num_derivs, 0, 2));
    // A vector turned about the axis by u, or its first or second derivative by u.
    const double cs = std::cos(u), sn = std::sin(u);
    const auto turn = [&](const Vector& w, int order) {
        const double a = w.dot(m_x), b = w.dot(m_y), ca = order == 0 ? cs : order == 1 ? -sn : -cs, sa = order == 0 ? sn : order == 1 ? cs : -sn;
        return m_x * (a * ca - b * sa) + m_y * (a * sa + b * ca) + m_z * (order == 0 ? w.dot(m_z) : 0.0);
    };
    const Vector q = c[0] - o;
    std::vector<Vector> result = {turned ? o + turn(q, 0) : c[0] + m_z * v};
    if (num_derivs > 0)
        result.push_back(turned ? turn(c[1], 0) : m_z);
    if (num_derivs > 1)
        result.push_back(turned ? turn(c[2], 0) : zero);
    if (num_derivs > 0)
        result.push_back(turned ? turn(q, 1) : c[1]);
    if (num_derivs > 1)
        result.insert(result.end(), {turned ? turn(c[1], 1) : zero, turned ? turn(q, 2) : c[2]});
    return result;
}

std::pair<double, double> Surface::sweep_parameters(const Point& point) const {
    const bool turned = m_kind == Kind::Revolution, bounded = m_profile->kind() == Curve::Kind::Nurbs;
    const int dir = turned ? 1 : 0;
    const double t0 = m_domain[2 * dir], t1 = m_domain[2 * dir + 1];
    const Vector d = point - m_origin, o(m_origin[0], m_origin[1], m_origin[2]);
    const double rho = std::hypot(d.dot(m_x), d.dot(m_y)), h = d.dot(m_z);
    // At a parameter of the profile: the distance of its sweep from the point, half the derivative of its square and the slope of that.
    const auto gap = [&](double t, int derivs) -> std::array<double, 3> {
        const std::vector<Vector> c = m_profile->evaluate(t, derivs);
        const Vector q = c[0] - o;
        if (!turned) {
            const Vector across = q - d - m_z * (q - d).dot(m_z);
            if (derivs < 2)
                return {across.magnitude(), 0.0, 0.0};
            return {across.magnitude(), across.dot(c[1]), c[1].dot(c[1]) - c[1].dot(m_z) * c[1].dot(m_z) + across.dot(c[2])};
        }
        const double a = q.dot(m_x), b = q.dot(m_y), r = std::hypot(a, b), k = q.dot(m_z) - h;
        if (derivs < 2)
            return {std::hypot(r - rho, k), 0.0, 0.0};
        // The distance from the axis and its derivatives; a profile on the axis leaves it at its speed across.
        const double a1 = c[1].dot(m_x), b1 = c[1].dot(m_y), a2 = c[2].dot(m_x), b2 = c[2].dot(m_y), k1 = c[1].dot(m_z), k2 = c[2].dot(m_z);
        const double r1 = r > 0.0 ? (a * a1 + b * b1) / r : std::hypot(a1, b1), r2 = r > 0.0 ? (a1 * a1 + b1 * b1 + a * a2 + b * b2 - r1 * r1) / r : 0.0;
        return {std::hypot(r - rho, k), (r - rho) * r1 + k * k1, r1 * r1 + (r - rho) * r2 + k1 * k1 + k * k2};
    };
    const int n = std::max(32, bounded ? 4 * m_profile->nurbs().cv_count() : 0);
    double t = t0, least = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= n; ++i) {
        const double at = t0 + (t1 - t0) * i / n, g = gap(at, 0)[0];
        if (g < least)
            least = g, t = at;
    }
    const double reach = 2.0 * (t1 - t0) / n;
    for (int i = 0; i < 60; ++i) {
        const std::array<double, 3> g = gap(t, 2);
        const double step = std::clamp(g[2] > 0.0 ? -g[1] / g[2] : g[1] > 0.0 ? -reach : reach, -reach, reach), next = bounded ? std::clamp(t + step, t0, t1) : t + step;
        const bool still = std::abs(next - t) <= 4e-16 * (1.0 + std::abs(t));
        t = next;
        if (still)
            break;
    }
    t = Curve::wrapped(t, t0, m_profile->period());
    const Vector q = m_profile->evaluate(t, 0)[0] - o;
    if (!turned)
        return {t, (d - q).dot(m_z)};
    const bool axial = !(rho > 0.0) || !(std::hypot(q.dot(m_x), q.dot(m_y)) > 0.0);
    return {axial ? m_domain[0] : Curve::wrapped(std::atan2(d.dot(m_y), d.dot(m_x)) - std::atan2(q.dot(m_y), q.dot(m_x)), m_domain[0], 2.0 * std::numbers::pi), t};
}

int Surface::sweep_coincident(const Surface& other, double tolerance) const {
    double alike = 0.0;
    for (const Surface* s : {this, &other}) {
        const Surface& on = s == this ? other : *this;
        for (double fu : {0.13, 0.37, 0.61, 0.89})
            for (double fv : {0.11, 0.43, 0.67, 0.93}) {
                const double u = s->m_domain[0] + fu * (s->m_domain[1] - s->m_domain[0]), v = s->m_domain[2] + fv * (s->m_domain[3] - s->m_domain[2]);
                const Point p = s->point_at(u, v);
                const std::pair<double, double> uv = on.closest_parameters(p);
                if (on.point_at(uv.first, uv.second).distance(p) > tolerance)
                    return 0;
                alike += s->normal_at(u, v).dot(on.normal_at(uv.first, uv.second));
            }
    }
    return alike > 0.0 ? 1 : -1;
}

bool Surface::sweep_transform(const Xform& xform) {
    const bool turned = m_kind == Kind::Revolution;
    const Vector z = xform.transform_vector(m_z);
    const double s = turned ? Curve::similarity(xform) : z.magnitude();
    if (!(s > 0.0))
        return false;
    Curve moved = swept();
    moved.transform(xform);
    const int dir = turned ? 1 : 0;
    m_domain[2 * dir] = moved.domain().first;
    m_domain[2 * dir + 1] = moved.domain().second;
    m_profile = std::make_shared<const Curve>(std::move(moved));
    if (turned) {
        m_origin = xform.transform_point(m_origin);
        m_x = xform.transform_vector(m_x) * (1.0 / s);
        m_y = xform.transform_vector(m_y) * (1.0 / s);
    } else {
        m_x = (std::abs(z[0]) < 0.9 * s ? Vector(1, 0, 0) : Vector(0, 1, 0)).cross(z).normalized();
        m_y = z.cross(m_x) * (1.0 / s);
        m_domain[2] *= s;
        m_domain[3] *= s;
    }
    m_z = z * (1.0 / s);
    m_nurbs.reset();
    return true;
}

void Surface::sweep_reverse(int dir) {
    if (dir == (m_kind == Kind::Extrusion ? 0 : 1)) {
        Curve reversed = swept();
        reversed.reverse();
        m_domain[2 * dir] = reversed.domain().first;
        m_domain[2 * dir + 1] = reversed.domain().second;
        m_profile = std::make_shared<const Curve>(std::move(reversed));
    } else {
        (dir == 0 ? m_y : m_z) *= -1.0;
        m_domain[2 * dir] = -std::exchange(m_domain[2 * dir + 1], -m_domain[2 * dir]);
    }
    m_nurbs.reset();
}

const NurbsSurface& Surface::sweep_nurbs() const {
    static const NurbsSurface invalid;
    const Curve profile = swept();
    const NurbsCurve& c = profile.nurbs();
    if (!c.is_valid())
        return invalid;
    const bool turned = m_kind == Kind::Revolution;
    // Across the profile: the arc of the angle as (cos, sin, weight), or the two ends of the length.
    std::vector<std::array<double, 3>> across = {{m_domain[2], 0, 1}, {m_domain[3], 0, 1}};
    std::vector<double> knots = {m_domain[2], m_domain[3]};
    if (turned)
        Curve::arc(m_domain[0], m_domain[1], across, knots);
    const int count = static_cast<int>(across.size()), dir = turned ? 1 : 0;
    NurbsSurface s(3, turned || c.is_rational(), turned ? 3 : c.order(), turned ? c.order() : 2, turned ? count : c.cv_count(), turned ? c.cv_count() : count);
    for (size_t i = 0; i < knots.size(); ++i)
        s.set_nurbsknot(1 - dir, static_cast<int>(i), knots[i]);
    for (int j = 0; j < c.nurbsknot_count(); ++j)
        s.set_nurbsknot(dir, j, c.nurbsknot(j));
    for (int i = 0; i < count; ++i)
        for (int j = 0; j < c.cv_count(); ++j) {
            const Vector q = c.get_cv(j) - m_origin;
            const double a = q.dot(m_x), b = q.dot(m_y), w = across[i][2] * (c.is_rational() ? c.weight(j) : 1.0);
            const Point p = turned ? m_origin + m_x * (a * across[i][0] - b * across[i][1]) + m_y * (a * across[i][1] + b * across[i][0]) + m_z * q.dot(m_z) : c.get_cv(j) + m_z * across[i][0];
            s.set_cv_4d(turned ? i : j, turned ? j : i, p[0] * w, p[1] * w, p[2] * w, w);
        }
    m_nurbs = std::make_shared<const NurbsSurface>(std::move(s));
    return *m_nurbs;
}

} // namespace session_cpp
