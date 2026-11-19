#include "intersection_analytic.h"
#include "intersection_sweep.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace session_cpp::intersection_analytic {
namespace {

using P2 = std::array<double, 2>;
constexpr double two_pi = 2.0 * M_PI;
constexpr double angular = 1e-12;
constexpr double infinity = std::numeric_limits<double>::infinity();
const double golden = 0.5 * (std::sqrt(5.0) - 1.0);

xyz unit(const xyz& v) {
    const double l = v.length();
    return l > 0 ? v / l : v;
}

double turn(double angle) {
    const double a = std::fmod(angle, two_pi);
    return a < 0 ? a + two_pi : a;
}

// gp_Ax3 in plain triples.
struct Axes {
    xyz o{0, 0, 0}, x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1};

    static Axes about(const xyz& origin, const xyz& axis) {
        const xyz z = unit(axis), x = unit(std::abs(z.x) < 0.9 ? xyz{1, 0, 0}.cross(z) : xyz{0, 1, 0}.cross(z));
        return {origin, x, z.cross(x), z};
    }
};

enum class FormKind { Plane, Cylinder, Cone, Sphere, Torus };

// The numbers of an analytic Surface: a cone has `radius` at its origin and the semi-angle `angle`.
struct Form {
    FormKind kind = FormKind::Plane;
    Axes frame;
    double radius = 0.0, minor = 0.0, angle = 0.0;

    // The signed distance from p to the surface (of a cone, to its nearer nappe).
    double distance(const xyz& p) const {
        const xyz d = p - frame.o;
        const double h = d.dot(frame.z), rho = (d - frame.z * h).length();
        switch (kind) {
        case FormKind::Plane: return h;
        case FormKind::Cylinder: return rho - radius;
        case FormKind::Cone: {
            const double c = std::cos(angle), s = std::sin(angle), near = (rho - radius) * c - h * s, far = (-rho - radius) * c - h * s;
            return std::abs(far) < std::abs(near) ? -far : near;
        }
        case FormKind::Sphere: return d.length() - radius;
        default: return std::hypot(rho - radius, h) - minor;
        }
    }
};

Form form(const Surface& s) {
    const bool cone = s.kind() == Surface::Kind::Cone;
    return {FormKind(static_cast<int>(s.kind()) - 1), {xyz(s.origin()), xyz(s.x()), xyz(s.y()), xyz(s.z())}, s.radius(), cone ? 0.0 : s.minor(), cone ? s.minor() : 0.0};
}

enum class ConicKind { Line, Circle, Ellipse, Parabola, Hyperbola };

// A section before it becomes a Curve, in ElCLib's parameters: a parabola o + t t / (4 radius) x + t y, a hyperbola branch
// o + radius cosh t x + minor sinh t y.
struct Conic {
    ConicKind kind = ConicKind::Line;
    Axes frame;
    double radius = 0.0, minor = 0.0;

    xyz point(double t) const {
        switch (kind) {
        case ConicKind::Line: return frame.o + frame.x * t;
        case ConicKind::Circle: return frame.o + (frame.x * std::cos(t) + frame.y * std::sin(t)) * radius;
        case ConicKind::Ellipse: return frame.o + frame.x * (radius * std::cos(t)) + frame.y * (minor * std::sin(t));
        case ConicKind::Parabola: return frame.o + frame.x * (t * t / (4 * radius)) + frame.y * t;
        default: return frame.o + frame.x * (radius * std::cosh(t)) + frame.y * (minor * std::sinh(t));
        }
    }

    xyz tangent(double t) const {
        switch (kind) {
        case ConicKind::Line: return frame.x;
        case ConicKind::Circle: return (frame.y * std::cos(t) - frame.x * std::sin(t)) * radius;
        case ConicKind::Ellipse: return frame.y * (minor * std::cos(t)) - frame.x * (radius * std::sin(t));
        case ConicKind::Parabola: return frame.x * (t / (2 * radius)) + frame.y;
        default: return frame.x * (radius * std::sinh(t)) + frame.y * (minor * std::cosh(t));
        }
    }

    double parameter(const xyz& p) const {
        const xyz d = p - frame.o;
        switch (kind) {
        case ConicKind::Line: return d.dot(frame.x);
        case ConicKind::Parabola: return d.dot(frame.y);
        case ConicKind::Hyperbola: return std::asinh(d.dot(frame.y) / minor);
        default: return turn(std::atan2(d.dot(frame.y) / (kind == ConicKind::Circle ? radius : minor), d.dot(frame.x) / radius));
        }
    }

    bool closed() const { return kind == ConicKind::Circle || kind == ConicKind::Ellipse; }
};

Conic make(ConicKind kind, const xyz& o, const xyz& x, const xyz& z, double radius = 0.0, double minor = 0.0) {
    return {kind, {o, x, z.cross(x), z}, radius, minor};
}

// A sphere, cylinder or cone as (p - c)^T (I - k z z^T) (p - c) = r2: IntAna_Quadric in the one frame these three share.
struct Quadric {
    xyz c, z;
    double k, r2;

    xyz apply(const xyz& v) const { return v - z * (k * v.dot(z)); }
    double value(const xyz& p) const { return (p - c).dot(apply(p - c)) - r2; }
};

Quadric quadric(const Form& s) {
    if (s.kind == FormKind::Sphere)
        return {s.frame.o, s.frame.z, 0.0, s.radius * s.radius};
    if (s.kind == FormKind::Cylinder)
        return {s.frame.o, s.frame.z, 1.0, s.radius * s.radius};
    return {s.frame.o - s.frame.z * (s.radius / std::tan(s.angle)), s.frame.z, 1.0 / (std::cos(s.angle) * std::cos(s.angle)), 0.0};
}

// ElSLib::Parameters; on a cone the nearer nappe, with the opposite angle beyond the apex.
std::pair<double, double> parameters(const Surface& surface, const xyz& p) {
    if (surface.kind() != Surface::Kind::Cone)
        return surface.closest_parameters(Point(p));
    const xyz d = p - xyz(surface.origin());
    const double x = d.dot(xyz(surface.x())), y = d.dot(xyz(surface.y())), h = d.dot(xyz(surface.z())), rho = std::hypot(x, y);
    const double c = std::cos(surface.minor()), s = std::sin(surface.minor()), u = rho > 0 ? turn(std::atan2(y, x)) : 0.0;
    const bool far = std::abs((-rho - surface.radius()) * c - h * s) < std::abs((rho - surface.radius()) * c - h * s);
    return {Curve::wrapped(far ? u + M_PI : u, surface.domain(0).first, two_pi), ((far ? -rho : rho) - surface.radius()) * s + h * c};
}

// The points of a surface without an angle of their own: the poles of a sphere, the apex of a cone.
std::vector<xyz> singular(const Surface& s) {
    const xyz o(s.origin()), z(s.z());
    if (s.kind() == Surface::Kind::Sphere)
        return {o + z * s.radius(), o - z * s.radius()};
    if (s.kind() == Surface::Kind::Cone)
        return {o - z * (s.radius() / std::tan(s.minor()))};
    return {};
}

// A function of one parameter as its Taylor coefficients up to a fixed order; products, quotients and roots are cut there.
struct Series {
    std::vector<double> c;

    explicit Series(size_t n = 0, double value = 0.0) : c(n, 0.0) {
        if (n)
            c[0] = value;
    }

    Series operator+(const Series& o) const {
        Series r = *this;
        for (size_t i = 0; i < c.size(); ++i)
            r.c[i] += o.c[i];
        return r;
    }

    Series operator*(double f) const {
        Series r = *this;
        for (double& v : r.c)
            v *= f;
        return r;
    }

    Series operator-(const Series& o) const { return *this + o * -1.0; }

    Series operator*(const Series& o) const {
        Series r(c.size());
        for (size_t i = 0; i < c.size(); ++i)
            for (size_t j = 0; i + j < c.size(); ++j)
                r.c[i + j] += c[i] * o.c[j];
        return r;
    }

    Series operator/(const Series& o) const {
        Series r(c.size());
        for (size_t i = 0; i < c.size(); ++i) {
            r.c[i] = c[i];
            for (size_t j = 1; j <= i; ++j)
                r.c[i] -= o.c[j] * r.c[i - j];
            r.c[i] /= o.c[0];
        }
        return r;
    }

    Series root() const {
        Series r(c.size(), std::sqrt(c[0]));
        for (size_t i = 1; i < c.size(); ++i) {
            r.c[i] = c[i];
            for (size_t j = 1; j < i; ++j)
                r.c[i] -= r.c[j] * r.c[i - j];
            r.c[i] /= 2 * r.c[0];
        }
        return r;
    }

    Series rate() const {
        Series r(c.size());
        for (size_t i = 1; i < c.size(); ++i)
            r.c[i - 1] = i * c[i];
        return r;
    }

    // Times the parameter (by 1) or over it, the constant term dropped (by -1).
    Series moved(int by) const {
        Series r(c.size());
        for (size_t i = by < 0 ? 1 : 0; i < c.size() - (by > 0 ? 1 : 0); ++i)
            r.c[i + by] = c[i];
        return r;
    }
};

// A curve about one of its parameters as Taylor coefficients: the point, the first derivative, half the second, ...
using Jet = std::vector<xyz>;

Series along(const Jet& p, const xyz& v) {
    Series r(p.size());
    for (size_t i = 0; i < p.size(); ++i)
        r.c[i] = p[i].dot(v);
    return r;
}

Series dot(const Jet& p, const Jet& q) {
    Series r(p.size());
    for (size_t i = 0; i < p.size(); ++i)
        for (size_t j = 0; i + j < p.size(); ++j)
            r.c[i + j] += p[i].dot(q[j]);
    return r;
}

// The angle of (x, y), from its rate (x y' - y x') / (x x + y y).
Series angle(const Series& y, const Series& x) {
    const Series rate = (x * y.rate() - y * x.rate()) / (x * x + y * y);
    Series a(x.c.size(), std::atan2(y.c[0], x.c[0]));
    for (size_t k = 1; k < a.c.size(); ++k)
        a.c[k] = rate.c[k - 1] / k;
    return a;
}

// The same curve about the parameter further on by `by`.
Jet shifted(Jet p, double by) {
    for (size_t i = 0; i + 1 < p.size(); ++i)
        for (size_t k = p.size() - 1; k > i; --k)
            p[k - 1] += p[k] * by;
    return p;
}

// The equation of a surface (zero on it) along a curve.
Series implicit(const Form& s, Jet p) {
    p[0] -= s.kind == FormKind::Cone ? s.frame.o - s.frame.z * (s.radius / std::tan(s.angle)) : s.frame.o;
    const Series h = along(p, s.frame.z), dd = dot(p, p), one(p.size(), 1.0);
    switch (s.kind) {
    case FormKind::Plane: return h;
    case FormKind::Cylinder: return dd - h * h - one * (s.radius * s.radius);
    case FormKind::Cone: return dd - h * h * (1.0 / (std::cos(s.angle) * std::cos(s.angle)));
    case FormKind::Sphere: return dd - one * (s.radius * s.radius);
    default: {
        const Series w = dd + one * (s.radius * s.radius - s.minor * s.minor);
        return w * w - (dd - h * h) * (4 * s.radius * s.radius);
    }
    }
}

xyz gradient(const Form& s, const xyz& p) {
    xyz g{0, 0, 0};
    for (int i = 0; i < 3; ++i) {
        Jet line = {p, xyz{0, 0, 0}};
        line[1][i] = 1.0;
        g[i] = implicit(s, line).c[1];
    }
    return g;
}

// The section of two surfaces about a point of both, by its length run in the sense of `toward`: the tangent is across both
// normals, and each further coefficient is the one unknown, linear, in that order of the two equations and of unit speed. The
// sine of the angle between the normals is returned; where they are parallel the jet stops at the direction given.
double section(const Form& a, const Form& b, const xyz& p, const xyz& toward, int order, Jet& jet) {
    jet.assign(order + 1, xyz{0, 0, 0});
    jet[0] = p;
    const xyz ga = gradient(a, p), gb = gradient(b, p), n = ga.cross(gb);
    const double sine = n.length() / (ga.length() * gb.length());
    jet[1] = unit(sine > 0 ? n * (n.dot(toward) < 0 ? -1.0 : 1.0) : toward);
    if (!(sine > 0))
        return 0.0;
    const double det = ga.dot(gb.cross(jet[1]));
    for (int k = 2; k <= order; ++k) {
        Jet rate(jet.size(), xyz{0, 0, 0});
        for (int i = 1; i < k; ++i)
            rate[i - 1] = jet[i] * i;
        jet[k] = (gb.cross(jet[1]) * -implicit(a, jet).c[k] + jet[1].cross(ga) * -implicit(b, jet).c[k] + n * (-dot(rate, rate).c[k - 1] / (2 * k))) / det;
    }
    return sine;
}

// An exact curve about t, n coefficients.
Jet taylor(const Curve& curve, double t, size_t n) {
    Jet p(n, xyz{0, 0, 0});
    const xyz x(curve.x()), y(curve.y());
    double f = 1.0;
    if (curve.kind() == Curve::Kind::Line)
        p[0] = xyz(curve.origin()) + x * t, p[1] = x;
    else if (curve.kind() == Curve::Kind::Nurbs) {
        const std::vector<Vector> d = curve.nurbs().evaluate(t, static_cast<int>(n) - 1);
        for (size_t k = 0; k < n; f /= ++k)
            p[k] = xyz(d[k]) * f;
    } else if (curve.kind() == Curve::Kind::Parabola) {
        p[0] = xyz(curve.origin()) + x * (t * t / (4 * curve.radius())) + y * t, p[1] = x * (t / (2 * curve.radius())) + y, p[2] = x * (1 / (4 * curve.radius()));
    } else if (curve.kind() == Curve::Kind::Hyperbola) {
        for (size_t k = 0; k < n; f /= ++k)
            p[k] = (x * (curve.radius() * (k % 2 ? std::sinh(t) : std::cosh(t))) + y * (curve.minor() * (k % 2 ? std::cosh(t) : std::sinh(t)))) * f;
        p[0] += xyz(curve.origin());
    } else {
        for (size_t k = 0; k < n; f /= ++k)
            p[k] = (x * (curve.radius() * std::cos(t + 0.5 * M_PI * k)) + y * ((curve.kind() == Curve::Kind::Circle ? curve.radius() : curve.minor()) * std::sin(t + 0.5 * M_PI * k))) * f;
        p[0] += xyz(curve.origin());
    }
    return p;
}

// (u, v) of a surface along a curve on it. Within tol of a pole of a sphere or the apex of a cone the curve is taken through it:
// (u, v) are regular in the curve's parameter there, the angle that of the curve's rate, on the side (1 after, -1 before) the
// piece lies; beyond the apex of a cone the angle is the opposite one, as its signed radius has it.
Jet mapped(const Surface& surface, const Form& s, Jet p, int side, double tol) {
    const bool cone = s.kind == FormKind::Cone, sphere = s.kind == FormKind::Sphere;
    p[0] -= cone ? s.frame.o - s.frame.z * (s.radius / std::tan(s.angle)) : s.frame.o;
    Series x = along(p, s.frame.x), y = along(p, s.frame.y), h = along(p, s.frame.z), u = x, v = y;
    if (s.kind != FormKind::Plane) {
        const bool pole = (cone || sphere) && std::hypot(x.c[0], y.c[0]) <= tol;
        if (pole)
            x = x.moved(-1), y = y.moved(-1);
        const double flip = cone ? (h.c[pole ? 1 : 0] * std::tan(s.angle) < 0 ? -1.0 : 1.0) : pole ? side : 1.0;
        u = angle(y * flip, x * flip);
        Series rho = (x * x + y * y).root() * flip;
        if (pole)
            rho = rho.moved(1);
        rho.c[0] -= s.kind == FormKind::Torus ? s.radius : 0.0;
        v = cone ? h * (1.0 / std::cos(s.angle)) : s.kind == FormKind::Cylinder ? h : angle(h, rho);
        v.c[0] -= cone ? s.radius / std::sin(s.angle) : 0.0;
    }
    Jet uv(p.size(), xyz{0, 0, 0});
    for (size_t k = 0; k < p.size(); ++k)
        uv[k] = {u.c[k], v.c[k], 0.0};
    for (int d = 0; d < 2; ++d)
        uv[0][d] = Curve::wrapped(uv[0][d], surface.domain(d).first, surface.period(d));
    return uv;
}

// The later parameters moved by whole periods next to where the earlier ones and the rates at both ends put them.
void follow(const Surface& surface, const Jet& from, Jet& to, double h) {
    for (int d = 0; d < 2; ++d)
        if (const double period = surface.period(d); period > 0)
            to[0][d] += period * std::round((from[0][d] + 0.5 * h * (from[1][d] + to[1][d]) + h * h / 6 * (from[2][d] - to[2][d]) - to[0][d]) / period);
}

// Derivatives matched at each knot: position, tangent, curvature and one more, on pieces of degree seven.
constexpr int contact = 4;

// Control points of the curve of degree 2 contact - 1 with the given Taylor coefficients at each parameter: its knots have
// multiplicity `contact`, and the control points about a knot are blossoms of the jet there alone.
std::vector<xyz> controls(const std::vector<double>& t, const std::vector<Jet>& jets) {
    const int n = static_cast<int>(t.size()), degree = 2 * contact - 1;
    std::vector<xyz> points;
    for (int i = 0; i < n; ++i) {
        const double before = t[std::max(i - 1, 0)] - t[i], after = t[std::min(i + 1, n - 1)] - t[i];
        for (int j = 0; j < contact; ++j) {
            // Elementary symmetric functions of the other arguments: contact - 1 - j at the knot before, j at the one after.
            std::array<double, contact> e{1.0};
            for (int q = 0; q < contact - 1; ++q)
                for (int k = contact - 1; k > 0; --k)
                    e[k] += (q < j ? after : before) * e[k - 1];
            xyz p{0, 0, 0};
            double f = 1.0;
            for (int k = 0; k < contact; f *= (k + 1.0) / (degree - k), ++k)
                p += jets[i][k] * (e[k] * f);
            points.push_back(p);
        }
    }
    return points;
}

NurbsCurve spline(const std::vector<double>& t, const std::vector<Jet>& jets) {
    std::vector<Point> points;
    for (const xyz& p : controls(t, jets))
        points.push_back(Point(p));
    std::vector<int> mults(t.size(), contact);
    mults.front() = mults.back() = 2 * contact;
    return NurbsCurve::create_from_parameters(points, std::vector<double>(points.size(), 1.0), t, mults, 2 * contact - 1);
}

xyz bezier(std::vector<xyz> c, double f) {
    for (size_t n = c.size(); n > 1; --n)
        for (size_t i = 0; i + 1 < n; ++i)
            c[i] = c[i] * (1 - f) + c[i + 1] * f;
    return c[0];
}

// A knot of a guide: the curve about it and the parameter span of the piece it starts.
struct Node {
    Jet jet;
    double span = 0.0;
};

// Keys respaced until every piece between them measures within the target: each piece asks for as many pieces as its error
// calls for at the order of the fit, aiming at half the target, and the new keys go where those run evenly. The lightest
// spacing that passed is returned; without one, pieces over the target are halved.
std::vector<double> refine(std::vector<double> keys, const std::function<std::vector<double>(const std::vector<double>&)>& errors, double target, int order) {
    std::vector<double> best;
    for (int round = 0; round < 10; ++round) {
        const std::vector<double> e = errors(keys);
        std::vector<double> want = {0.0};
        for (double v : e) {
            const double pieces = std::pow(v / (0.5 * target), 1.0 / order);
            want.push_back(want.back() + (pieces < 4.0 ? std::max(pieces, 0.2) : 4.0));
        }
        const bool within = *std::max_element(e.begin(), e.end()) <= target;
        const size_t count = std::max<size_t>(2, static_cast<size_t>(std::ceil(want.back())));
        if (within && (best.empty() || keys.size() < best.size()))
            best = keys;
        if ((within && count + 1 >= 0.85 * keys.size()) || count > 4000)
            break;
        std::vector<double> next = {keys.front()};
        for (size_t j = 1; j < count; ++j) {
            const double w = want.back() * j / count;
            const size_t i = std::min<size_t>(std::upper_bound(want.begin(), want.end(), w) - want.begin(), want.size() - 1) - 1;
            next.push_back(keys[i] + (keys[i + 1] - keys[i]) * (w - want[i]) / (want[i + 1] - want[i]));
        }
        next.push_back(keys.back());
        keys = std::move(next);
    }
    if (!best.empty())
        return best;
    for (int round = 0; round < 30 && keys.size() < 8000; ++round) {
        const std::vector<double> e = errors(keys);
        std::vector<double> next = {keys.front()};
        for (size_t i = 0; i + 1 < keys.size(); ++i) {
            if (e[i] > target && keys[i + 1] - keys[i] > 1e-9 * (keys.back() - keys.front()))
                next.push_back(0.5 * (keys[i] + keys[i + 1]));
            next.push_back(keys[i + 1]);
        }
        if (next.size() == keys.size())
            break;
        keys = std::move(next);
    }
    return keys;
}

// The pcurve piece between two nodes as Bezier control points over the first node's span.
std::vector<xyz> between(const Surface& surface, const Form& s, const Node& from, const Node& to, double tol) {
    const Jet a = mapped(surface, s, from.jet, 1, tol);
    Jet b = mapped(surface, s, to.jet, -1, tol);
    follow(surface, a, b, from.span);
    return controls({0.0, from.span}, {a, b});
}

// The pcurve through the nodes at the parameters t.
NurbsCurve laid(const std::vector<Node>& nodes, const std::vector<double>& t, const Surface& surface, const Form& s, double tol) {
    std::vector<Jet> jets;
    for (size_t i = 0; i < nodes.size(); ++i) {
        jets.push_back(mapped(surface, s, nodes[i].jet, i ? -1 : 1, tol));
        if (i)
            follow(surface, jets[i - 1], jets[i], t[i] - t[i - 1]);
    }
    return spline(t, jets);
}

// (u, v) of a point of the surface, next to the guide's by whole periods; the guide's own at a pole or an apex.
xyz beside(const Surface& surface, const Form& s, const xyz& p, const xyz& guide, double tol) {
    for (const xyz& q : singular(surface))
        if ((p - q).length() <= tol)
            return guide;
    xyz uv = mapped(surface, s, {p}, 1, -1.0)[0];
    for (int d = 0; d < 2; ++d)
        if (const double period = surface.period(d); period > 0)
            uv[d] += period * std::round((guide[d] - uv[d]) / period);
    return uv;
}

// The point of both surfaces nearest p; p itself where they are too nearly tangent to say.
xyz polished(const Form& a, const Form& b, xyz p) {
    for (int it = 0; it < 3; ++it) {
        const xyz ga = gradient(a, p), gb = gradient(b, p);
        const double fa = implicit(a, {p}).c[0], fb = implicit(b, {p}).c[0], aa = ga.dot(ga), ab = ga.dot(gb), bb = gb.dot(gb), det = aa * bb - ab * ab;
        if (!(det > 1e-10 * aa * bb))
            break;
        p -= ga * ((fa * bb - fb * ab) / det) + gb * ((fb * aa - fa * ab) / det);
    }
    return p;
}

// A section point and its (u, v) on both surfaces.
using Row = std::array<double, 7>;
constexpr int degree = 7;

// Control rows of the spline of degree seven with simple knots at the breaks that takes the rows at(t) at its Greville sites:
// the collocation matrix is banded and totally positive, eliminated without pivoting.
std::vector<Row> interpolated(const std::vector<double>& breaks, const std::function<Row(double)>& at) {
    std::vector<double> u(degree, breaks.front());
    u.insert(u.end(), breaks.begin(), breaks.end());
    u.insert(u.end(), degree, breaks.back());
    const int n = static_cast<int>(breaks.size()) + degree - 1;
    std::vector<std::array<double, 2 * degree + 1>> band(n, std::array<double, 2 * degree + 1>{});
    std::vector<Row> rows(n);
    for (int i = 0; i < n; ++i) {
        double t = 0.0;
        for (int k = 1; k <= degree; ++k)
            t += u[i + k] / degree;
        rows[i] = at(t);
        const int span = std::clamp(static_cast<int>(std::upper_bound(u.begin(), u.end(), t) - u.begin()) - 1, degree, n - 1);
        // The B-splines that are not zero at t (The NURBS Book, A2.2).
        std::array<double, degree + 1> b{1.0}, left{}, right{};
        for (int j = 1; j <= degree; ++j) {
            left[j] = t - u[span + 1 - j], right[j] = u[span + j] - t;
            double saved = 0.0;
            for (int r = 0; r < j; ++r) {
                const double part = b[r] / (right[r + 1] + left[j - r]);
                b[r] = saved + right[r + 1] * part;
                saved = left[j - r] * part;
            }
            b[j] = saved;
        }
        for (int j = 0; j <= degree; ++j)
            band[i][span + j - i] = b[j];
    }
    for (int i = 0; i < n; ++i)
        for (int r = i + 1; r < std::min(n, i + degree + 1); ++r) {
            const double f = band[r][degree + i - r] / band[i][degree];
            for (int c = i; c < std::min(n, i + degree + 1); ++c)
                band[r][degree + c - r] -= f * band[i][degree + c - i];
            for (int k = 0; k < 7; ++k)
                rows[r][k] -= f * rows[i][k];
        }
    for (int i = n - 1; i >= 0; --i)
        for (int k = 0; k < 7; ++k) {
            for (int c = i + 1; c < std::min(n, i + degree + 1); ++c)
                rows[i][k] -= band[i][degree + c - i] * rows[c][k];
            rows[i][k] /= band[i][degree];
        }
    return rows;
}

struct Fit {
    NurbsCurve curve;
    std::array<NurbsCurve, 2> on;
};

// The light curves: a section and its pcurves of degree seven on one simple knot vector through the rows at(t), the breaks
// respaced until error(fit, t) is within the target all along.
Fit lightened(const std::vector<double>& breaks, const std::function<Row(double)>& at, const std::function<double(const Fit&, double)>& error, double target) {
    Fit fit;
    const auto errors = [&](const std::vector<double>& keys) {
        std::array<std::vector<Point>, 3> points;
        for (const Row& row : interpolated(keys, at))
            for (int k = 0; k < 3; ++k)
                points[k].push_back(k ? Point(row[1 + 2 * k], row[2 + 2 * k], 0) : Point(row[0], row[1], row[2]));
        std::vector<int> mults(keys.size(), 1);
        mults.front() = mults.back() = degree + 1;
        const auto curve = [&](int k) { return NurbsCurve::create_from_parameters(points[k], std::vector<double>(points[k].size(), 1.0), keys, mults, degree); };
        fit = {curve(0), {curve(1), curve(2)}};
        std::vector<double> e(keys.size() - 1, 0.0);
        for (size_t i = 0; i + 1 < keys.size(); ++i)
            for (int j = 0; j <= 8; ++j)
                e[i] = std::max(e[i], error(fit, keys[i] + (keys[i + 1] - keys[i]) * j / 8));
        return e;
    };
    errors(refine(breaks, errors, target, degree + 1));
    return fit;
}

// The pcurve of an exact section on a surface, on the section's parameter.
Curve2d pcurve(const Curve& curve, const Conic* conic, const Surface& surface, double tol) {
    const auto [t0, t1] = curve.domain();
    const Form s = form(surface);
    const auto flat = [&](const xyz& v) { return Vector(v.dot(s.frame.x), v.dot(s.frame.y), 0); };
    if (surface.kind() == Surface::Kind::Plane && curve.kind() == Curve::Kind::Nurbs) {
        // The image of its control points, with their weights.
        NurbsCurve image = curve.nurbs();
        for (int i = 0; i < image.cv_count(); ++i) {
            const Vector uv = flat(xyz(curve.nurbs().get_cv(i)) - s.frame.o);
            const double w = image.is_rational() ? image.weight(i) : 1.0;
            image.set_cv_4d(i, uv[0] * w, uv[1] * w, 0.0, w);
        }
        return Curve2d(image);
    }
    const auto node = [&](double t) { return Node{taylor(curve, t, contact + 1)}; };
    // A ruling or a circle about the axis: a 2D line run at unit speed.
    const Jet start = mapped(surface, s, node(t0).jet, 1, tol);
    bool straight = std::abs(start[1].length() - 1.0) <= 1e-10;
    for (double f : {0.08, 0.41, 0.5, 0.93, 1.0}) {
        const double t = t0 + f * (t1 - t0);
        const Jet uv = mapped(surface, s, node(t).jet, f < 1 ? 1 : -1, tol);
        straight = straight && (uv[1] - start[1]).length() <= 1e-10 && uv[2].length() <= 1e-10;
        for (int d = 0; d < 2; ++d) {
            const double gap = uv[0][d] - start[0][d] - (t - t0) * start[1][d], period = surface.period(d);
            straight = straight && std::abs(period > 0 ? std::remainder(gap, period) : gap) <= 1e-10 * (1.0 + std::abs(start[0][d]) + std::abs(t - t0));
        }
    }
    if (straight)
        return Curve2d(Curve::line(Point(start[0] - start[1] * t0), Vector(start[1]), t0, t1));
    if (surface.kind() == Surface::Kind::Plane) {
        const Vector centre = flat(conic->frame.o - s.frame.o);
        const Point o(centre[0], centre[1], 0);
        const Vector x = flat(conic->frame.x), y = flat(conic->frame.y);
        switch (conic->kind) {
        case ConicKind::Circle: return Curve2d(Curve::circle(o, x, y, conic->radius, t0, t1));
        case ConicKind::Ellipse: return Curve2d(Curve::ellipse(o, x, y, conic->radius, conic->minor, t0, t1));
        case ConicKind::Parabola: return Curve2d(Curve::parabola(o, x, y, conic->radius, t0, t1));
        default: return Curve2d(Curve::hyperbola(o, x, y, conic->radius, conic->minor, t0, t1));
        }
    }
    // Anything else (an ellipse on a cylinder, a conic on a cone, a tilted circle on a sphere): a guide whose pieces take the
    // true parameters and their derivatives at the knots, then the light curve through the true parameters, each until the
    // surface point under it is on the section.
    std::vector<Node> nodes;
    std::vector<double> t;
    for (int i = 0; i <= 8; ++i)
        t.push_back(i < 8 ? t0 + (t1 - t0) * i / 8 : t1);
    const auto pieces = [&](const std::vector<double>& keys) {
        nodes.clear();
        for (double key : keys)
            nodes.push_back(node(key));
        std::vector<double> e(keys.size() - 1, 0.0);
        for (size_t i = 0; i + 1 < keys.size(); ++i) {
            nodes[i].span = keys[i + 1] - keys[i];
            const std::vector<xyz> c = between(surface, s, nodes[i], nodes[i + 1], tol);
            for (int j = 1; j < 16; ++j) {
                const xyz uv = bezier(c, j / 16.0);
                e[i] = std::max(e[i], surface.point_at(uv.x, uv.y).distance(curve.point_at(keys[i] + nodes[i].span * j / 16)));
            }
        }
        return e;
    };
    t = refine(t, pieces, 0.07 * tol, 2 * contact);
    pieces(t);
    const NurbsCurve guide = laid(nodes, t, surface, s, tol);
    return Curve2d(lightened(t, [&](double v) {
        const xyz uv = beside(surface, s, xyz(curve.point_at(v)), xyz(guide.point_at(v)), tol);
        return Row{0, 0, 0, uv.x, uv.y, 0, 0};
    }, [&](const Fit& fit, double v) {
        const Point uv = fit.on[0].point_at(v);
        return surface.point_at(uv[0], uv[1]).distance(curve.point_at(v));
    }, 0.07 * tol).on[0]);
}

// IntAna_IntQuadQuad's parametrisation: one surface swept by an angle (the rulings of a cylinder or cone, the tube circles of
// a torus), on each of which the other surface marks at most two points, the roots of a quadratic that are real where its
// discriminant is positive.
struct Pencil {
    std::function<double(double)> discriminant;
    std::function<xyz(double, int)> point; // at an angle on the branch -1 or +1; on branch 0, or with a discriminant below zero, the turning point between them
    std::function<Conic(double)> ruling = nullptr; // the swept line at an angle, where the sweep is of lines
};

Pencil ruled(const Form& swept, const Form& other) {
    const Quadric q = quadric(other);
    const bool cone = swept.kind == FormKind::Cone;
    const xyz base = cone ? swept.frame.o - swept.frame.z * (swept.radius / std::tan(swept.angle)) : swept.frame.o;
    const auto ruling = [=](double angle, xyz& from, xyz& along) {
        const xyz across = swept.frame.x * std::cos(angle) + swept.frame.y * std::sin(angle);
        from = cone ? base : base + across * swept.radius;
        along = cone ? across * std::sin(swept.angle) + swept.frame.z * std::cos(swept.angle) : swept.frame.z;
    };
    const auto terms = [=](double angle, xyz& from, xyz& along) {
        ruling(angle, from, along);
        return std::array<double, 3>{along.dot(q.apply(along)), 2 * along.dot(q.apply(from - q.c)), q.value(from)};
    };
    return {[=](double angle) {
                xyz from, along;
                const std::array<double, 3> k = terms(angle, from, along);
                return k[1] * k[1] - 4 * k[0] * k[2];
            },
            [=](double angle, int branch) {
                xyz from, along;
                std::array<double, 3> k = terms(angle, from, along);
                // On a ruling that the other surface has too the quadratic is nothing: the points next to it are the roots of
                // its rate over the angle.
                if (std::max({std::abs(k[0]), std::abs(k[1]), std::abs(k[2])}) <= 1e-8 * (1.0 + (from - q.c).dot(from - q.c) + std::abs(q.r2))) {
                    xyz other;
                    const std::array<double, 3> before = terms(angle - 1e-4, other, other), after = terms(angle + 1e-4, other, other);
                    k = {after[0] - before[0], after[1] - before[1], after[2] - before[2]};
                }
                const double root = std::sqrt(std::max(k[1] * k[1] - 4 * k[0] * k[2], 0.0));
                // The branches by the sign of the root over the leading term: they stay continuous where it changes sign.
                return from + along * (std::abs(k[0]) > 1e-14 ? (-k[1] + branch * root) / (2 * k[0]) : branch > 0 ? -k[2] / k[1] : infinity);
            },
            [=](double angle) {
                xyz from, along;
                ruling(angle, from, along);
                const Axes about = Axes::about(from, along);
                return Conic{ConicKind::Line, {from, along, about.x, about.y}};
            }};
}

// A torus cut by a plane: on the tube circle at u the height over the plane is alpha cos v + beta sin v + gamma.
Pencil tubes(const Form& torus, const Form& plane) {
    const xyz n = plane.frame.z;
    const auto terms = [=](double u) {
        const double across = n.dot(torus.frame.x * std::cos(u) + torus.frame.y * std::sin(u));
        return std::array<double, 3>{torus.minor * across, torus.minor * n.dot(torus.frame.z), torus.radius * across + (torus.frame.o - plane.frame.o).dot(n)};
    };
    return {[=](double u) {
                const std::array<double, 3> k = terms(u);
                return k[0] * k[0] + k[1] * k[1] - k[2] * k[2];
            },
            [=](double u, int branch) {
                const std::array<double, 3> k = terms(u);
                const double half = std::acos(std::clamp(-k[2] / std::hypot(k[0], k[1]), -1.0, 1.0)), v = std::atan2(k[1], k[0]) + (branch ? branch * half : half < 0.5 * M_PI ? 0.0 : M_PI);
                return torus.frame.o + (torus.frame.x * std::cos(u) + torus.frame.y * std::sin(u)) * (torus.radius + torus.minor * std::cos(v)) + torus.frame.z * (torus.minor * std::sin(v));
            }};
}

// GeomLib_CheckCurveOnSurface: the largest distance of the surface point under the pcurve from the section, sampled.
double deviation(const Curve& curve, const Curve2d& on, const Surface& surface) {
    const auto [t0, t1] = curve.domain();
    const int n = 16 * std::max({8, curve.kind() == Curve::Kind::Nurbs ? curve.nurbs().cv_count() : 0, on.kind() == Curve::Kind::Nurbs ? on.nurbs().cv_count() : 0});
    double worst = 0.0;
    for (int i = 0; i <= n; ++i) {
        const double t = t0 + (t1 - t0) * i / n;
        const Point uv = on.point_at(t);
        worst = std::max(worst, surface.point_at(uv[0], uv[1]).distance(curve.point_at(t)));
    }
    return worst;
}

struct Builder;
void plane_quadric(const Form& plane, const Form& other, double tol, Builder& out);

struct Builder {
    const Surface& sa;
    const Surface& sb;
    Form a, b;
    double tol;
    Result result;
    std::vector<Conic> conics;
    std::vector<Conic>* collect = nullptr; // where curve() leaves its conics instead of making sections
    std::vector<xyz> specks; // points a sweep met that may stand alone

    // The parameters where an open section passes the boxes of both surfaces' domains.
    std::pair<double, double> reach(const Conic& c) const {
        std::pair<double, double> range = {-infinity, infinity};
        for (const Surface* s : {&sa, &sb}) {
            const AABB box = s->aabb();
            std::pair<double, double> own = {infinity, -infinity};
            for (int i = 0; i < 8; ++i) {
                const double t = c.parameter(xyz{box.cx + (i & 1 ? box.hx : -box.hx), box.cy + (i & 2 ? box.hy : -box.hy), box.cz + (i & 4 ? box.hz : -box.hz)});
                own = {std::min(own.first, t), std::max(own.second, t)};
            }
            range = {std::max(range.first, own.first), std::min(range.second, own.second)};
        }
        return range;
    }

    void curve(const Conic& c, bool tangent = false) {
        if (collect)
            return collect->push_back(c);
        std::pair<double, double> range = c.closed() ? std::pair{0.0, two_pi} : reach(c);
        if (!(range.second > range.first))
            return;
        if (c.kind == ConicKind::Parabola || c.kind == ConicKind::Hyperbola) {
            // A parabola or hyperbola runs out of the boxes faster than its parameter says: kept between its first and last
            // points inside them.
            const std::array<AABB, 2> boxes = {sa.aabb(), sb.aabb()};
            const int n = 4000;
            int first = n + 1, last = -1;
            for (int i = 0; i <= n; ++i) {
                const xyz p = c.point(range.first + (range.second - range.first) * i / n);
                bool in = true;
                for (const AABB& box : boxes)
                    in = in && std::abs(p.x - box.cx) <= 1.01 * box.hx + tol && std::abs(p.y - box.cy) <= 1.01 * box.hy + tol && std::abs(p.z - box.cz) <= 1.01 * box.hz + tol;
                if (in)
                    first = std::min(first, i), last = i;
            }
            if (last <= first)
                return;
            range = {range.first + (range.second - range.first) * std::max(first - 1, 0) / n, range.first + (range.second - range.first) * std::min(last + 1, n) / n};
        }
        const Point o(c.frame.o);
        const Vector x(c.frame.x), y(c.frame.y);
        Curve section;
        if (c.kind == ConicKind::Line) {
            section = Curve::line(o, x, range.first, range.second);
        } else if (c.kind == ConicKind::Circle) {
            section = Curve::circle(o, x, y, c.radius);
        } else if (c.kind == ConicKind::Ellipse) {
            section = Curve::ellipse(o, x, y, c.radius, c.minor);
        } else if (c.kind == ConicKind::Parabola) {
            section = Curve::parabola(o, x, y, c.radius, range.first, range.second);
        } else {
            section = Curve::hyperbola(o, x, y, c.radius, c.minor, range.first, range.second);
        }
        add(section, &c, tangent);
    }

    // A section over a pole of a sphere or the apex of a cone is cut there: the angle about the axis jumps.
    void add(const Curve& whole, const Conic* c, bool tangent) {
        const auto [t0, t1] = whole.domain();
        std::vector<double> cuts;
        for (const Surface* s : {&sa, &sb})
            for (const xyz& q : singular(*s)) {
                const double t = whole.closest_parameter(Point(q));
                if ((xyz(whole.point_at(t)) - q).length() <= tol)
                    cuts.push_back(t);
            }
        std::sort(cuts.begin(), cuts.end());
        cuts.erase(std::unique(cuts.begin(), cuts.end(), [&](double l, double r) { return r - l < 1e-9 * (t1 - t0); }), cuts.end());
        const double period = whole.period(), slack = 1e-9 * (t1 - t0);
        std::vector<std::pair<double, double>> pieces;
        if (cuts.empty())
            pieces.push_back({t0, t1});
        else if (period > 0 && whole.is_closed())
            for (size_t i = 0; i < cuts.size(); ++i)
                pieces.push_back({cuts[i], i + 1 < cuts.size() ? cuts[i + 1] : cuts[0] + period});
        else {
            cuts.insert(cuts.begin(), t0);
            cuts.push_back(t1);
            for (size_t i = 0; i + 1 < cuts.size(); ++i)
                if (cuts[i + 1] - cuts[i] > slack)
                    pieces.push_back({cuts[i], cuts[i + 1]});
        }
        for (const auto& [from, to] : pieces) {
            Curve section = whole;
            if (pieces.size() > 1 || from != t0 || to != t1)
                section.set_domain(from, to);
            piece(section, c, tangent);
        }
    }

    void piece(const Curve& section, const Conic* c, bool tangent) {
        Section made{section, pcurve(section, c, sa, tol), pcurve(section, c, sb, tol), tangent, 0.0, 0.0, 0.0};
        made.deviation_a = deviation(section, made.on_a, sa), made.deviation_b = deviation(section, made.on_b, sb);
        result.curves.push_back(std::move(made));
    }

    // A section through the points at(key) of both surfaces. A planar one is the conic its plane cuts from the first quadric,
    // returned as such, once. Anything else is cut where it passes a pole or an apex and fitted between.
    void fitted(const std::function<xyz(double)>& at, double k0, double k1) {
        const int m = 512;
        std::vector<xyz> samples(m + 1);
        for (int i = 0; i <= m; ++i)
            samples[i] = at(k0 + (k1 - k0) * i / m);
        if (std::all_of(samples.begin(), samples.end(), [&](const xyz& p) { return !((p - samples[0]).length() > tol); }))
            return specks.push_back(samples[0]);
        if (!std::all_of(samples.begin(), samples.end(), [](const xyz& p) { return std::isfinite(p.x + p.y + p.z); }))
            return;
        const xyz p0 = samples[m / 6], normal = unit((samples[m / 2] - p0).cross(samples[5 * m / 6] - p0));
        bool planar = normal.length() > 0 && a.kind != FormKind::Plane && a.kind != FormKind::Torus && b.kind != FormKind::Plane && b.kind != FormKind::Torus;
        for (int i = 0; i <= m && planar; i += 8)
            planar = std::abs((samples[i] - p0).dot(normal)) <= tol;
        if (planar) {
            std::vector<Conic> found;
            Builder probe{sa, sb, a, b, tol, {}, {}, &found, {}};
            plane_quadric({FormKind::Plane, Axes::about(p0, normal)}, a, tol, probe);
            for (const Conic& c : found) {
                bool on = true;
                for (int i = 0; i <= m && on; i += 8) {
                    const xyz q = c.point(c.closed() ? two_pi * i / m : c.parameter(samples[i]));
                    on = std::abs(b.distance(q)) <= 0.1 * tol && (c.point(c.parameter(samples[i])) - samples[i]).length() <= tol;
                }
                if (!on)
                    continue;
                const bool known = std::any_of(conics.begin(), conics.end(), [&](const Conic& k) {
                    const xyz d = c.frame.o - k.frame.o, turned = k.frame.x.cross(c.frame.x);
                    const bool line = c.kind == ConicKind::Line, round = c.kind == ConicKind::Circle, open = !c.closed();
                    return k.kind == c.kind && std::abs(k.radius - c.radius) <= tol && std::abs(k.minor - c.minor) <= tol && (line ? d.cross(k.frame.x) : d).length() <= tol
                        && (line || k.frame.z.cross(c.frame.z).length() <= 1e-9) && (round || (turned.length() <= 1e-9 && (!open || line || k.frame.x.dot(c.frame.x) > 0)));
                });
                if (!known)
                    conics.push_back(c), curve(c);
                return;
            }
        }
        std::vector<std::pair<double, xyz>> stops = {{k0, samples[0]}, {k1, samples[m]}};
        for (const Surface* s : {&sa, &sb})
            for (const xyz& q : singular(*s)) {
                const auto gap = [&](double key) { return (at(key) - q).length(); };
                for (int i = 0; i <= m; ++i) {
                    const double here = (samples[i] - q).length();
                    if (i == 0 || i == m) {
                        if (here <= tol)
                            stops[i ? 1 : 0].second = q;
                        continue;
                    }
                    if (here > (samples[i - 1] - q).length() || here >= (samples[i + 1] - q).length() || here > 4 * (samples[i + 1] - samples[i - 1]).length())
                        continue;
                    double lo = k0 + (k1 - k0) * (i - 1) / m, hi = k0 + (k1 - k0) * (i + 1) / m;
                    for (int it = 0; it < 80; ++it) {
                        const double m1 = hi - golden * (hi - lo), m2 = lo + golden * (hi - lo);
                        const bool left = gap(m1) < gap(m2);
                        (left ? hi : lo) = left ? m2 : m1;
                    }
                    if (gap(0.5 * (lo + hi)) <= tol && 0.5 * (lo + hi) > k0 && 0.5 * (lo + hi) < k1)
                        stops.push_back({0.5 * (lo + hi), q});
                }
            }
        std::sort(stops.begin(), stops.end(), [](const auto& l, const auto& r) { return l.first < r.first; });
        for (size_t i = 0; i + 1 < stops.size(); ++i)
            stretch(at, stops[i].first, stops[i].second, stops[i + 1].first, stops[i + 1].second);
    }

    // A section from p0 at the key k0 to p1 at k1 as pieces of degree seven that take the true section's position, tangent,
    // curvature and third derivative at their ends, by its length; split until curve and pcurves measure within 0.07 tol of the
    // surfaces and of each other. Where the surfaces touch at an end (a crossing of branches) the derivatives are carried there
    // from a point further in.
    void stretch(const std::function<xyz(double)>& at, double k0, const xyz& p0, double k1, const xyz& p1) {
        const auto off = [&](const xyz& p) { return std::max(std::abs(a.distance(p)), std::abs(b.distance(p))); };
        const int m = 256;
        std::vector<double> run(m + 1, 0.0);
        xyz before = p0;
        for (int i = 1; i <= m; ++i) {
            const xyz p = i < m ? at(k0 + (k1 - k0) * i / m) : p1;
            run[i] = run[i - 1] + (p - before).length();
            before = p;
        }
        const double length = run[m], step = 1e-6 * (k1 - k0);
        if (!(length > tol))
            return specks.push_back(p0);
        const auto key = [&](double l) {
            const int i = std::clamp(static_cast<int>(std::upper_bound(run.begin(), run.end(), l) - run.begin()) - 1, 0, m - 1);
            return k0 + (k1 - k0) * (i + (run[i + 1] > run[i] ? (l - run[i]) / (run[i + 1] - run[i]) : 0.0)) / m;
        };
        const auto jet = [&](double l, int order, Jet& made) {
            const double k = key(l);
            return section(a, b, l <= 0 ? p0 : l >= length ? p1 : at(k), at(std::min(k + step, k1)) - at(std::max(k - step, k0)), order, made);
        };
        const auto node = [&](double l) {
            Node made;
            // At an end where the surfaces touch, or at an apex, the section has no normals to go by: its derivatives are
            // carried there from a point further in.
            if ((jet(l, contact, made.jet) >= 0.03 && !apex(made.jet[0])) || (l > 0 && l < length))
                return made;
            const xyz tangent = made.jet[1];
            made.jet.assign(contact + 1, xyz{0, 0, 0});
            made.jet[0] = l <= 0 ? p0 : p1, made.jet[1] = tangent;
            for (double f = 1e-3; f < 0.2; f *= 2) {
                Jet inner;
                if (jet(l <= 0 ? f * length : length - f * length, contact + 6, inner) < 0.03)
                    continue;
                const xyz p = made.jet[0];
                double by = (p - inner[0]).length() * (l <= 0 ? -1.0 : 1.0);
                for (int it = 0; it < 6; ++it)
                    by += (p - shifted(inner, by)[0]).dot(shifted(inner, by)[1]);
                made.jet = shifted(inner, by);
                made.jet.resize(contact + 1);
                made.jet[0] = p;
                break;
            }
            return made;
        };
        const std::array<const Surface*, 2> surfaces = {&sa, &sb};
        const std::array<const Form*, 2> forms = {&a, &b};
        std::vector<Node> nodes;
        const auto pieces = [&](const std::vector<double>& keys) {
            nodes.clear();
            for (double l : keys)
                nodes.push_back(node(l));
            std::vector<double> e(keys.size() - 1, 0.0);
            for (size_t n = 0; n + 1 < keys.size(); ++n) {
                // The span of a piece is its own length.
                Node &from = nodes[n], &to = nodes[n + 1];
                const double chord = (to.jet[0] - from.jet[0]).length();
                from.span = chord;
                std::vector<xyz> c;
                for (int it = 0; it < 5; ++it) {
                    c = controls({0.0, from.span}, {from.jet, to.jet});
                    if (it == 4)
                        break;
                    double run = 0.0;
                    for (int i = 0; i < 8; ++i)
                        for (double g : {-1.0, 1.0}) {
                            const double f = (i + 0.5 + g * 0.5 / std::sqrt(3.0)) / 8;
                            run += (bezier({c.begin() + 1, c.end()}, f) - bezier({c.begin(), c.end() - 1}, f)).length() * (c.size() - 1.0) / 16;
                        }
                    from.span = std::clamp(run, chord, 2 * chord);
                }
                std::array<std::vector<xyz>, 2> on;
                for (int k = 0; k < 2; ++k)
                    if (forms[k]->kind != FormKind::Plane)
                        on[k] = between(*surfaces[k], *forms[k], from, to, tol);
                for (int i = 1; i < 16; ++i) {
                    const xyz p = bezier(c, i / 16.0);
                    e[n] = std::max(e[n], off(p));
                    for (int k = 0; k < 2; ++k)
                        if (!on[k].empty()) {
                            const xyz uv = bezier(on[k], i / 16.0);
                            e[n] = std::max(e[n], (xyz(surfaces[k]->point_at(uv.x, uv.y)) - p).length());
                        }
                }
            }
            return e;
        };
        std::vector<double> keys;
        for (int i = 0; i <= 8; ++i)
            keys.push_back(length * i / 8);
        pieces(refine(keys, pieces, 0.07 * tol, 2 * contact));
        std::vector<double> t = {0.0};
        std::vector<Jet> jets;
        for (size_t i = 0; i < nodes.size(); ++i) {
            jets.push_back(nodes[i].jet);
            if (i)
                t.push_back(t.back() + nodes[i - 1].span);
        }
        const NurbsCurve guide = spline(t, jets);
        std::array<NurbsCurve, 2> guides;
        for (int k = 0; k < 2; ++k)
            if (forms[k]->kind != FormKind::Plane)
                guides[k] = laid(nodes, t, *surfaces[k], *forms[k], tol);
        // The light curves through true points of the section at the guide's parameters.
        const Fit fit = lightened(t, [&](double v) {
            const xyz p = v <= 0 || v >= t.back() ? xyz(guide.point_at(v)) : polished(a, b, xyz(guide.point_at(v)));
            Row row{p.x, p.y, p.z};
            for (int k = 0; k < 2; ++k)
                if (forms[k]->kind != FormKind::Plane) {
                    const xyz uv = beside(*surfaces[k], *forms[k], p, xyz(guides[k].point_at(v)), tol);
                    row[3 + 2 * k] = uv.x, row[4 + 2 * k] = uv.y;
                }
            return row;
        }, [&](const Fit& f, double v) {
            const Point p = f.curve.point_at(v);
            double worst = off(xyz(p));
            for (int k = 0; k < 2; ++k)
                if (forms[k]->kind != FormKind::Plane) {
                    const Point uv = f.on[k].point_at(v);
                    worst = std::max(worst, surfaces[k]->point_at(uv[0], uv[1]).distance(p));
                }
            return worst;
        }, 0.07 * tol);
        Section made{Curve(fit.curve), {}, {}, false, 0.0, 0.0, 0.0};
        const int count = 16 * fit.curve.cv_count();
        for (int i = 0; i <= count; ++i)
            made.fit = std::max(made.fit, off(xyz(fit.curve.point_at(t.back() * i / count))));
        if (!std::isfinite(t.back() + made.fit))
            made.fit = infinity;
        made.on_a = a.kind == FormKind::Plane ? pcurve(made.curve, nullptr, sa, tol) : Curve2d(fit.on[0]);
        made.on_b = b.kind == FormKind::Plane ? pcurve(made.curve, nullptr, sb, tol) : Curve2d(fit.on[1]);
        made.deviation_a = deviation(made.curve, made.on_a, sa), made.deviation_b = deviation(made.curve, made.on_b, sb);
        result.curves.push_back(std::move(made));
    }

    // The sections of a pencil: over each stretch of angles with two real points, the two branches run out and back as one
    // closed curve; with two real points all round, each branch is a closed curve of its own; a discriminant that only
    // reaches zero is a touching point. Parts outside the boxes of the domains are left out.
    void sweep(const Pencil& pencil) {
        swept(pencil);
        // A point met alone, where a branch stays in one place or the discriminant only reaches zero: touching, but for an apex.
        for (const xyz& p : specks) {
            bool alone = std::isfinite(p.x) && std::max(std::abs(a.distance(p)), std::abs(b.distance(p))) <= tol;
            for (const Section& s : result.curves)
                alone = alone && (xyz(s.curve.point_at(s.curve.domain().first)) - p).length() > tol && (xyz(s.curve.point_at(s.curve.domain().second)) - p).length() > tol;
            for (const Hit& h : result.points)
                alone = alone && (xyz(h.point) - p).length() > tol;
            if (alone)
                point(p, !apex(p));
        }
    }

    void swept(const Pencil& pencil) {
        const int n = 1440;
        std::vector<double> d(n + 1);
        for (int i = 0; i <= n; ++i)
            d[i] = pencil.discriminant(two_pi * i / n);
        const std::array<AABB, 2> boxes = {sa.aabb(), sb.aabb()};
        const auto inside = [&](const xyz& p) {
            for (const AABB& box : boxes)
                if (!(std::abs(p.x - box.cx) <= 1.01 * box.hx + tol && std::abs(p.y - box.cy) <= 1.01 * box.hy + tol && std::abs(p.z - box.cz) <= 1.01 * box.hz + tol))
                    return false;
            return true;
        };
        // A closed chain cut to the keys inside the boxes, each end where it passes out of them.
        const auto chain = [&](const std::function<xyz(double)>& at, double k0, double k1) {
            const int m = 720;
            const auto key = [&](int i) { return k0 + (k1 - k0) * i / m; };
            std::vector<char> in(m + 1);
            for (int i = 0; i <= m; ++i)
                in[i] = inside(at(key(i)));
            if (std::all_of(in.begin(), in.end(), [](char v) { return v; }))
                return fitted(at, k0, k1);
            const auto edge = [&](double within, double out) {
                for (int it = 0; it < 50; ++it)
                    (inside(at(0.5 * (within + out))) ? within : out) = 0.5 * (within + out);
                return within;
            };
            for (int i = 0; i <= m; ++i)
                if (in[i] && (i == 0 || !in[i - 1])) {
                    int j = i;
                    while (j < m && in[j + 1])
                        ++j;
                    if (j > i)
                        fitted(at, i > 0 ? edge(key(i), key(i - 1)) : k0, j < m ? edge(key(j), key(j + 1)) : k1);
                    i = j;
                }
        };
        const auto zero = [&](double lo, double hi) {
            for (int it = 0; it < 100; ++it) {
                const double mid = 0.5 * (lo + hi);
                ((pencil.discriminant(mid) < 0) == (pencil.discriminant(lo) < 0) ? lo : hi) = mid;
            }
            return 0.5 * (lo + hi);
        };
        // Where the discriminant touches zero its slope changes sign: found there closer than its lowest value can be.
        const auto turning = [&](double lo, double hi, double sign) {
            const auto slope = [&](double angle) { return sign * (pencil.discriminant(angle + 1e-5) - pencil.discriminant(angle - 1e-5)); };
            for (int it = 0; it < 60; ++it)
                (slope(0.5 * (lo + hi)) < 0 ? lo : hi) = 0.5 * (lo + hi);
            return 0.5 * (lo + hi);
        };
        const double top = *std::max_element(d.begin(), d.end()), size = std::max(top, -*std::min_element(d.begin(), d.end()));
        // A double root is zero only up to the rounding of the discriminant, which shows in how it scatters about the angle and is no less
        // than a few last places of its largest value.
        const auto touching = [&](double angle) {
            const double least = pencil.discriminant(angle);
            double noise = 0.0;
            for (int k = -4; k <= 4; ++k)
                noise = std::max(noise, std::abs(pencil.discriminant(angle + k * 1e-11) - least));
            return std::abs(least) <= 8 * noise + 64 * std::numeric_limits<double>::epsilon() * size;
        };
        // A swept line that lies on the other surface is a line of the section, tangent where the surfaces touch along it.
        // The discriminant is zero there without the branches turning or crossing.
        std::vector<double> shared;
        const auto along = [&](double angle) { return std::any_of(shared.begin(), shared.end(), [&](double v) { return std::abs(std::remainder(angle - v, two_pi)) < 1e-6; }); };
        for (int i = 0; i < n && pencil.ruling && a.kind != FormKind::Sphere && b.kind != FormKind::Sphere; ++i) {
            const double here = std::abs(d[i]);
            if (here > std::abs(d[i > 0 ? i - 1 : n - 1]) || here >= std::abs(d[i + 1]) || here > 1e-6 * size)
                continue;
            const double angle = turning(two_pi * (i - 1) / n, two_pi * (i + 1) / n, d[i + 1] < 0 ? -1.0 : 1.0);
            const Conic line = pencil.ruling(angle);
            const std::pair<double, double> range = reach(line);
            bool on = range.second - range.first > std::sqrt(tol);
            for (int k = 0; k <= 8 && on; ++k)
                on = std::max(std::abs(a.distance(line.point(range.first + (range.second - range.first) * k / 8))), std::abs(b.distance(line.point(range.first + (range.second - range.first) * k / 8)))) <= 0.1 * tol;
            Jet jet;
            if (on)
                shared.push_back(angle), curve(line, std::max(section(a, b, line.point(range.first + 0.3 * (range.second - range.first)), line.frame.x, 1, jet), section(a, b, line.point(range.first + 0.6 * (range.second - range.first)), line.frame.x, 1, jet)) <= 1e-6);
        }
        // Where it touches zero from above two branches cross: the section is cut there into simple arcs. Where it passes
        // zero and comes back between two samples, a narrow gap or a small loop, the two zeros are taken.
        std::vector<double> crossings, rise, fall;
        for (int i = 0; i < n; ++i) {
            const double before = d[i > 0 ? i - 1 : n - 1], after = d[i + 1], lo = two_pi * (i - 1) / n, hi = two_pi * (i + 1) / n;
            if (d[i] >= 0 && before >= d[i] && after > d[i] && before >= 0 && after >= 0 && d[i] <= 1e-4 * top) {
                const double angle = turning(lo, hi, 1.0), least = pencil.discriminant(angle), gap = (pencil.point(angle, 1) - pencil.point(angle, -1)).length();
                const xyz p = pencil.point(angle, 0);
                if (along(angle))
                    continue;
                if (((touching(angle) && gap <= std::sqrt(tol)) || (least >= 0 && gap <= tol)) && std::max(std::abs(a.distance(p)), std::abs(b.distance(p))) <= tol)
                    crossings.push_back(turn(angle));
                else if (least < 0)
                    fall.push_back(zero(lo, angle)), rise.push_back(zero(angle, hi));
            } else if (d[i] < 0 && before <= d[i] && after < d[i]) {
                // Where it only comes up to zero the surfaces may meet in a point.
                const double angle = turning(lo, hi, -1.0);
                const xyz p = pencil.point(angle, 0);
                if (pencil.discriminant(angle) > 0)
                    rise.push_back(zero(lo, angle)), fall.push_back(zero(angle, hi));
                else if (std::isfinite(p.x) && inside(p))
                    specks.push_back(p);
            }
        }
        std::sort(crossings.begin(), crossings.end());
        // At a crossing both branches take the one point between them.
        const auto crossed = [&](double angle) { return std::any_of(crossings.begin(), crossings.end(), [&](double c) { return std::abs(std::remainder(angle - c, two_pi)) < 1e-12; }); };
        // A closed chain over a turn of its key, cut at the given keys.
        const auto arcs = [&](const std::function<xyz(double)>& at, std::vector<double> cuts) {
            std::sort(cuts.begin(), cuts.end());
            if (cuts.empty())
                return chain(at, 0.0, two_pi);
            for (size_t i = 0; i < cuts.size(); ++i)
                chain(at, cuts[i], i + 1 < cuts.size() ? cuts[i + 1] : cuts[0] + two_pi);
        };
        for (int i = 0; i < n; ++i) {
            if (d[i] < 0 && d[i + 1] >= 0)
                rise.push_back(zero(two_pi * i / n, two_pi * (i + 1) / n));
            if (d[i] >= 0 && d[i + 1] < 0)
                fall.push_back(zero(two_pi * i / n, two_pi * (i + 1) / n));
        }
        // A dip of the discriminant below zero no deeper than its rounding, or that closes within the tolerance, is a crossing
        // met with rounding, not a turn.
        for (size_t i = 0; i < rise.size() && rise.size() == fall.size();) {
            size_t last = 0;
            const auto before = [&](size_t k) { return fall[k] < rise[i] ? fall[k] : fall[k] - two_pi; };
            for (size_t k = 1; k < fall.size(); ++k)
                last = before(k) > before(last) ? k : last;
            const double from = before(last), angle = rise[i] - from < 0.1 ? turning(from, rise[i], 1.0) : 0.0;
            const double gap = (pencil.point(from, 0) - pencil.point(rise[i], 0)).length();
            if (rise[i] - from < 0.1 && (along(angle) || gap <= tol || (touching(angle) && gap <= std::sqrt(tol)))) {
                if (!along(angle))
                    crossings.push_back(turn(angle));
                rise.erase(rise.begin() + i);
                fall.erase(fall.begin() + last);
            } else {
                ++i;
            }
        }
        std::sort(crossings.begin(), crossings.end());
        if (rise.empty()) {
            if (top >= 0)
                for (int branch : {-1, 1})
                    arcs([&, branch](double angle) { return pencil.point(angle, crossed(angle) ? 0 : branch); }, crossings);
            return;
        }
        for (double from : rise) {
            double to = infinity;
            for (double f : fall)
                to = std::min(to, f > from ? f : f + two_pi);
            // Out on one branch and back on the other, evenly in the angle of a half turn so that the ends, where the root
            // goes as a square root, are sampled finely; a loop no larger than the tolerance is a touching point.
            const std::function<xyz(double)> loop = [&, from, to](double key) {
                const double angle = 0.5 * (from + to) - 0.5 * (to - from) * std::cos(key);
                return pencil.point(angle, std::abs(std::sin(key)) < 1e-12 || crossed(angle) ? 0 : std::sin(key) > 0 ? 1 : -1);
            };
            // A crossing inside the stretch is met once on the way out and once on the way back.
            std::vector<double> cuts;
            for (double crossing : crossings) {
                const double angle = crossing < from ? crossing + two_pi : crossing, cosine = (0.5 * (from + to) - angle) / (0.5 * (to - from));
                if (std::abs(cosine) < 1.0)
                    cuts.insert(cuts.end(), {std::acos(cosine), two_pi - std::acos(cosine)});
            }
            if ((loop(0.0) - loop(M_PI)).length() <= tol && (loop(0.5 * M_PI) - loop(1.5 * M_PI)).length() <= tol) {
                if (inside(loop(0.0)))
                    point(loop(0.0), true);
            } else {
                arcs(loop, cuts);
            }
        }
    }

    bool apex(const xyz& p) const {
        return (sa.kind() == Surface::Kind::Cone && (singular(sa)[0] - p).length() <= tol) || (sb.kind() == Surface::Kind::Cone && (singular(sb)[0] - p).length() <= tol);
    }

    void point(const xyz& p, bool tangent) { result.points.push_back({Point(p), 0.0, parameters(sa, p), parameters(sb, p), tangent}); }
};

// A plane against a sphere, cylinder or cone: the quadric along the plane is a conic A s s + B s t + C t t + D s + E t + F = 0,
// brought to its axes and read as IntAna_QuadQuadGeo reads it: ellipse, circle, parabola, hyperbola, lines, a point or nothing.
void plane_quadric(const Form& plane, const Form& other, double tol, Builder& out) {
    const Quadric q = quadric(other);
    const xyz &e1 = plane.frame.x, &e2 = plane.frame.y, &n = plane.frame.z, w = plane.frame.o - q.c;
    const double A = e1.dot(q.apply(e1)), B = 2 * e1.dot(q.apply(e2)), C = e2.dot(q.apply(e2));
    const double D = 2 * e1.dot(q.apply(w)), E = 2 * e2.dot(q.apply(w)), F = q.value(plane.frame.o);
    const double angle = 0.5 * std::atan2(B, A - C), cs = std::cos(angle), sn = std::sin(angle);
    xyz v1 = e1 * cs + e2 * sn, v2 = e2 * cs - e1 * sn;
    double l1 = A * cs * cs + B * cs * sn + C * sn * sn, l2 = A * sn * sn - B * cs * sn + C * cs * cs;
    double d1 = D * cs + E * sn, d2 = E * cs - D * sn;
    const auto near = [&](const xyz& p) { return std::abs(other.distance(p)) <= tol; };
    if (std::abs(l1) < std::abs(l2))
        std::swap(l1, l2), std::swap(d1, d2), std::swap(v1, v2);
    if (std::abs(l2) <= 1e-10 * std::abs(l1)) {
        const double x0 = -d1 / (2 * l1);
        const xyz middle = plane.frame.o + v1 * x0;
        if (std::abs(d2) <= 1e-10 * (1.0 + w.length())) {
            const double h2 = -(F - l1 * x0 * x0) / l1;
            if (near(middle))
                out.curve(make(ConicKind::Line, middle, v2, n), true);
            else if (h2 > 0)
                for (double side : {-1.0, 1.0})
                    out.curve(make(ConicKind::Line, middle + v1 * (side * std::sqrt(h2)), v2, n));
            return;
        }
        const double y0 = (l1 * x0 * x0 - F) / d2;
        const xyz open = v2 * (d2 / l1 < 0 ? 1.0 : -1.0);
        out.curve(make(ConicKind::Parabola, middle + v2 * y0, open, open.cross(v1).dot(n) > 0 ? n : -n, std::abs(d2 / l1) / 4));
        return;
    }
    const double x0 = -d1 / (2 * l1), y0 = -d2 / (2 * l2), f0 = F - l1 * x0 * x0 - l2 * y0 * y0;
    const xyz centre = plane.frame.o + v1 * x0 + v2 * y0;
    if (l1 * l2 > 0) {
        const double r1 = -f0 / l1, r2 = -f0 / l2;
        if (near(centre) && !(std::sqrt(std::max(r1, 0.0)) > tol && std::sqrt(std::max(r2, 0.0)) > tol))
            out.point(centre, q.apply(centre - q.c).length() > tol);
        else if (r1 > 0 && r2 > 0) {
            const bool first = r1 >= r2;
            const double major = std::sqrt(first ? r1 : r2), minor = std::sqrt(first ? r2 : r1);
            if (major - minor <= 1e-12 * major)
                out.curve(make(ConicKind::Circle, centre, e1, n, major));
            else
                out.curve(make(ConicKind::Ellipse, centre, first ? v1 : v2, n, major, minor));
        }
        return;
    }
    if (near(centre)) {
        for (double side : {-1.0, 1.0})
            out.curve(make(ConicKind::Line, centre, unit(v1 * std::sqrt(std::abs(l2)) + v2 * (side * std::sqrt(std::abs(l1)))), n));
        return;
    }
    const bool first = -f0 / l1 > 0;
    const xyz across = first ? v1 : v2;
    const double major = std::sqrt(first ? -f0 / l1 : -f0 / l2), minor = std::sqrt(first ? f0 / l2 : f0 / l1);
    for (double side : {-1.0, 1.0})
        out.curve(make(ConicKind::Hyperbola, centre, across * side, n, major, minor));
}

// The meridian of a surface of revolution about the axis (origin, z) in (rho, h), mirrored in the axis: lines and circles.
struct Meridian {
    bool circle;
    P2 p, d;
    double r;
};

bool meridians(const Form& s, const xyz& origin, const xyz& z, double tol, std::vector<Meridian>& out) {
    const xyz d = s.frame.o - origin;
    const double h = d.dot(z), off = (d - z * h).length(), sense = s.frame.z.dot(z) < 0 ? -1.0 : 1.0;
    if ((off > tol && s.kind != FormKind::Plane) || (s.kind != FormKind::Sphere && s.frame.z.cross(z).length() > angular))
        return false;
    for (double side : {-1.0, 1.0})
        switch (s.kind) {
        case FormKind::Plane: out.push_back({false, {0, h}, {1, 0}, 0}); break;
        case FormKind::Cylinder: out.push_back({false, {side * s.radius, h}, {0, 1}, 0}); break;
        case FormKind::Cone: out.push_back({false, {side * s.radius, h}, {side * std::sin(s.angle), sense * std::cos(s.angle)}, 0}); break;
        case FormKind::Sphere: out.push_back({true, {0, h}, {}, s.radius}); break;
        case FormKind::Torus: out.push_back({true, {side * s.radius, h}, {}, s.minor}); break;
        }
    return true;
}

// The points two meridians share, tangent where they touch.
void meet(const Meridian& m, const Meridian& k, double tol, std::vector<std::pair<P2, bool>>& out) {
    if (!m.circle && !k.circle) {
        const double det = m.d[0] * k.d[1] - m.d[1] * k.d[0];
        if (std::abs(det) <= angular)
            return;
        const double f = ((k.p[0] - m.p[0]) * k.d[1] - (k.p[1] - m.p[1]) * k.d[0]) / det;
        out.push_back({{m.p[0] + f * m.d[0], m.p[1] + f * m.d[1]}, false});
    } else if (m.circle != k.circle) {
        const Meridian &line = m.circle ? k : m, &round = m.circle ? m : k;
        const double f = (round.p[0] - line.p[0]) * line.d[0] + (round.p[1] - line.p[1]) * line.d[1];
        const P2 foot = {line.p[0] + f * line.d[0], line.p[1] + f * line.d[1]};
        const double gap = std::hypot(foot[0] - round.p[0], foot[1] - round.p[1]);
        if (std::abs(gap - round.r) <= tol)
            out.push_back({foot, true});
        else if (gap < round.r)
            for (double side : {-1.0, 1.0}) {
                const double half = side * std::sqrt(round.r * round.r - gap * gap);
                out.push_back({{foot[0] + half * line.d[0], foot[1] + half * line.d[1]}, false});
            }
    } else {
        const double dx = k.p[0] - m.p[0], dy = k.p[1] - m.p[1], apart = std::hypot(dx, dy);
        if (apart <= tol || apart > m.r + k.r + tol || apart < std::abs(m.r - k.r) - tol)
            return;
        const bool touch = std::abs(apart - m.r - k.r) <= tol || std::abs(apart - std::abs(m.r - k.r)) <= tol;
        const double f = touch ? (apart > std::max(m.r, k.r) ? m.r : (m.r > k.r ? m.r : -m.r)) : (apart * apart + m.r * m.r - k.r * k.r) / (2 * apart);
        const double half = touch ? 0.0 : std::sqrt(std::max(m.r * m.r - f * f, 0.0));
        for (double side : {-1.0, 1.0}) {
            out.push_back({{m.p[0] + (f * dx - side * half * dy) / apart, m.p[1] + (f * dy + side * half * dx) / apart}, touch});
            if (touch)
                break;
        }
    }
}

// Two surfaces of revolution about one axis meet in circles across it, where their meridians meet.
bool coaxial(const xyz& origin, const xyz& z, double tol, Builder& out) {
    std::vector<Meridian> ma, mb;
    if (!meridians(out.a, origin, z, tol, ma) || !meridians(out.b, origin, z, tol, mb))
        return false;
    std::vector<std::pair<P2, bool>> found, kept;
    for (const Meridian& m : ma)
        for (const Meridian& k : mb)
            meet(m, k, tol, found);
    for (const auto& f : found)
        if (f.first[0] >= -tol && std::none_of(kept.begin(), kept.end(), [&](const auto& g) { return std::hypot(g.first[0] - f.first[0], g.first[1] - f.first[1]) <= tol; }))
            kept.push_back(f);
    std::sort(kept.begin(), kept.end(), [](const auto& l, const auto& r) { return l.first[1] < r.first[1]; });
    const xyz across = std::abs(out.a.frame.x.dot(z)) < 0.9 ? unit(out.a.frame.x - z * out.a.frame.x.dot(z)) : Axes::about(origin, z).x;
    for (const auto& [p, tangent] : kept)
        if (p[0] <= tol)
            out.point(origin + z * p[1], tangent);
        else
            out.curve(make(ConicKind::Circle, origin + z * p[1], across, z, p[0]), tangent);
    return true;
}

double horner(const std::vector<double>& c, double x) {
    double v = 0.0;
    for (size_t i = c.size(); i-- > 0;)
        v = v * x + c[i];
    return v;
}

// math_DirectPolynomialRoots in effect: the real roots of c0 + c1 x + ... (degree four at most here), each between two roots of
// the derivative by bisection; `turns` gets the derivative's roots, where the polynomial may touch zero without crossing it.
std::vector<double> real_roots(std::vector<double> c, std::vector<double>* turns = nullptr) {
    const double top = std::abs(*std::max_element(c.begin(), c.end(), [](double l, double r) { return std::abs(l) < std::abs(r); }));
    while (c.size() > 1 && std::abs(c.back()) <= 1e-13 * top)
        c.pop_back();
    if (c.size() < 2)
        return {};
    if (c.size() == 2)
        return {-c[0] / c[1]};
    std::vector<double> d;
    for (size_t i = 1; i < c.size(); ++i)
        d.push_back(i * c[i]);
    std::vector<double> stops = real_roots(d), found;
    std::sort(stops.begin(), stops.end());
    if (turns)
        *turns = stops;
    double reach = 0.0;
    for (size_t i = 0; i + 1 < c.size(); ++i)
        reach = std::max(reach, std::abs(c[i] / c.back()));
    stops.insert(stops.begin(), -1.0 - reach);
    stops.push_back(1.0 + reach);
    for (size_t i = 0; i + 1 < stops.size(); ++i) {
        double lo = stops[i], hi = stops[i + 1], flo = horner(c, lo);
        if (flo * horner(c, hi) > 0)
            continue;
        for (int it = 0; it < 200 && hi - lo > 1e-16 * (1.0 + std::abs(lo)); ++it) {
            const double mid = 0.5 * (lo + hi), fm = horner(c, mid);
            (fm * flo > 0 ? lo : hi) = mid;
            flo = fm * flo > 0 ? fm : flo;
        }
        if (found.empty() || 0.5 * (lo + hi) - found.back() > 1e-14 * (1.0 + std::abs(lo)))
            found.push_back(0.5 * (lo + hi));
    }
    return found;
}

// Roots of a distance along a curve: sign changes by bisection; where it turns toward zero between samples, two crossings if it
// passes zero by more than tol, a touching point if it comes within tol.
std::vector<std::pair<double, bool>> roots(const std::function<double(double)>& f, double t0, double t1, int n, bool periodic, double tol, bool& all) {
    std::vector<double> v(n + 1);
    all = true;
    const auto at = [&](int i) { return t0 + (t1 - t0) * i / n; };
    for (int i = 0; i <= n; ++i)
        v[i] = f(at(i)), all = all && std::abs(v[i]) <= tol;
    std::vector<std::pair<double, bool>> found;
    const double step = (t1 - t0) / n;
    // Between lo, where the distance is flo, and hi, where it has the other sign.
    const auto crossing = [&](double lo, double hi, double flo) {
        for (int it = 0; it < 80 && flo != 0.0; ++it) {
            const double mid = 0.5 * (lo + hi), fm = f(mid);
            (fm * flo > 0 ? lo : hi) = mid;
            flo = fm * flo > 0 ? fm : flo;
        }
        return flo == 0.0 ? lo : 0.5 * (lo + hi);
    };
    for (int i = 0; i < n && !all; ++i) {
        if (v[i] == 0.0 || v[i] * v[i + 1] < 0) {
            found.push_back({crossing(at(i), at(i + 1), v[i]), false});
            continue;
        }
        const double before = i > 0 ? v[i - 1] : periodic ? v[n - 1] : v[i], sign = v[i] < 0 ? -1.0 : 1.0;
        if (std::abs(v[i]) > std::abs(before) || std::abs(v[i]) > std::abs(v[i + 1]) || v[i] * before < 0)
            continue;
        double lo = at(i - 1), hi = at(i + 1);
        for (int it = 0; it < 80; ++it) {
            const double m1 = hi - golden * (hi - lo), m2 = lo + golden * (hi - lo);
            const bool left = sign * f(m1) < sign * f(m2);
            (left ? hi : lo) = left ? m2 : m1;
        }
        const double t = 0.5 * (lo + hi);
        if (sign * f(t) < -tol)
            found.insert(found.end(), {{crossing(at(i - 1), t, before), false}, {crossing(t, at(i + 1), f(t)), false}});
        else if (sign * f(t) <= tol && (found.empty() || std::abs(found.back().first - t) > 4 * step))
            found.push_back({t, true});
    }
    return found;
}

Result sections(const Surface& first, const Surface& second, double tolerance) {
    Result none;
    if (first.kind() == Surface::Kind::Nurbs || second.kind() == Surface::Kind::Nurbs)
        return none;
    Builder out{first, second, form(first), form(second), tolerance, {}, {}, nullptr, {}};
    const Form &a = out.a, &b = out.b;
    Result& r = out.result;
    r.done = true;
    r.coincident = first.coincident(second, tolerance);
    if (r.coincident)
        return r;
    const bool plane_a = a.kind == FormKind::Plane, plane_b = b.kind == FormKind::Plane;
    const Form &plane = plane_a ? a : b, &other = plane_a ? b : a;
    if (plane_a && plane_b) {
        const xyz dir = a.frame.z.cross(b.frame.z);
        if (dir.length() > angular) {
            // The point of the line nearest a's origin, in the span of the two normals.
            const double k = a.frame.z.dot(b.frame.z), h = (b.frame.o - a.frame.o).dot(b.frame.z);
            out.curve(make(ConicKind::Line, a.frame.o + (b.frame.z - a.frame.z * k) * (h / (1 - k * k)), unit(dir), a.frame.z));
        }
    } else if ((plane_a || plane_b) && other.kind != FormKind::Torus) {
        plane_quadric(plane, other, tolerance, out);
    } else if (plane_a || plane_b) {
        const xyz across = other.frame.z.cross(plane.frame.z);
        if (coaxial(other.frame.o, other.frame.z, tolerance, out))
            return r;
        const bool centred = std::abs((other.frame.o - plane.frame.o).dot(plane.frame.z)) <= tolerance;
        if (centred && std::abs(across.length() * other.radius - other.minor) <= tolerance)
            // A plane through the centre that touches the tube on both sides: the two Villarceau circles.
            for (double side : {-1.0, 1.0})
                out.curve(make(ConicKind::Circle, other.frame.o + unit(across) * (side * other.minor), unit(across), plane.frame.z, other.radius));
        else if (std::abs(other.frame.z.dot(plane.frame.z)) > angular || !centred)
            out.sweep(tubes(other, plane));
        else
            for (double side : {-1.0, 1.0})
                out.curve(make(ConicKind::Circle, other.frame.o + unit(across) * (side * other.radius), unit(across), plane.frame.z, other.minor));
    } else {
        const xyz d = b.frame.o - a.frame.o;
        const bool sphere_a = a.kind == FormKind::Sphere, sphere_b = b.kind == FormKind::Sphere;
        if (sphere_a && sphere_b && d.length() <= tolerance)
            return r;
        if (coaxial(a.frame.o, sphere_a ? (sphere_b ? unit(d) : b.frame.z) : a.frame.z, tolerance, out))
            return r;
        const bool ruled_a = a.kind == FormKind::Cylinder || a.kind == FormKind::Cone;
        r.done = a.kind != FormKind::Torus && b.kind != FormKind::Torus;
        if (!r.done)
            return r;
        const xyz cross = a.frame.z.cross(b.frame.z);
        const bool cylinders = a.kind == FormKind::Cylinder && b.kind == FormKind::Cylinder;
        const xyz apex = singular(first).empty() ? a.frame.o : singular(first)[0];
        if (a.kind == FormKind::Cone && b.kind == FormKind::Cone && (singular(second)[0] - apex).length() <= tolerance) {
            // Cones from one apex meet in the rulings they share: directions at the two semi-angles from the two axes, on either
            // side of the plane of the axes; one where the cones touch.
            const double c = a.frame.z.dot(b.frame.z), s2 = cross.dot(cross);
            for (double side : {1.0, -1.0}) {
                const double ca = std::cos(a.angle), cb = side * std::cos(b.angle), alpha = (ca - cb * c) / s2, beta = (cb - ca * c) / s2;
                const double out2 = 1.0 - (alpha * alpha + beta * beta + 2 * alpha * beta * c);
                for (double up : {1.0, -1.0})
                    if (out2 > -1e-14 && (up > 0 || out2 > 1e-14)) {
                        const xyz along = unit(a.frame.z * alpha + b.frame.z * beta + unit(cross) * (up * std::sqrt(std::max(out2, 0.0))));
                        const Axes about = Axes::about(apex, along);
                        out.curve(Conic{ConicKind::Line, {apex, along, about.x, about.y}}, out2 <= 1e-14);
                    }
            }
        } else if (!cylinders || (cross.length() > angular && !(std::abs(a.radius - b.radius) <= tolerance && std::abs(d.dot(unit(cross))) <= tolerance))) {
            // Two quadrics in general position: swept over the angle of a cylinder, else of a cone.
            const bool first = a.kind == FormKind::Cylinder || (ruled_a && b.kind != FormKind::Cylinder);
            out.sweep(first ? ruled(a, b) : ruled(b, a));
        } else if (cross.length() <= angular) {
            // Parallel cylinders: two circles in the plane across the axes.
            const xyz offset = d - a.frame.z * d.dot(a.frame.z);
            std::vector<std::pair<P2, bool>> found;
            meet({true, {0, 0}, {}, a.radius}, {true, {offset.length(), 0}, {}, b.radius}, tolerance, found);
            for (const auto& [p, tangent] : found)
                out.curve(make(ConicKind::Line, a.frame.o + unit(offset) * p[0] + a.frame.z.cross(unit(offset)) * p[1], a.frame.z, unit(offset)), tangent);
            r.done = true;
        } else if (std::abs(a.radius - b.radius) <= tolerance && std::abs(d.dot(unit(cross))) <= tolerance) {
            // Equal cylinders whose axes cross: an ellipse in each plane that halves the angle between the axes.
            const double k = a.frame.z.dot(b.frame.z), f = (d.dot(a.frame.z) - d.dot(b.frame.z) * k) / (1 - k * k);
            const Form half{FormKind::Plane, Axes::about(a.frame.o + a.frame.z * f, a.frame.z + b.frame.z)}, other_half{FormKind::Plane, Axes::about(half.frame.o, a.frame.z - b.frame.z)};
            plane_quadric(half, a, tolerance, out);
            plane_quadric(other_half, a, tolerance, out);
            r.done = true;
        }
    }
    return r;
}

}

Result surface_surface(const Surface& a, const Surface& b, double tolerance) {
    if (a.kind() > Surface::Kind::Torus || b.kind() > Surface::Kind::Torus)
        return intersection_sweep::surface_surface(a, b, tolerance);
    Result r = sections(a, b, tolerance);
    // A section that measures off its surfaces or its pcurves by more than the tolerance is not an answer.
    for (const Section& s : r.curves)
        r.done = r.done && std::max({s.fit, s.deviation_a, s.deviation_b}) <= tolerance;
    return r;
}

Result curve_surface(const Curve& given, const Surface& on, double tolerance) {
    Result r;
    if (given.kind() == Curve::Kind::Nurbs || on.kind() == Surface::Kind::Nurbs || given.kind() > Curve::Kind::Ellipse || on.kind() > Surface::Kind::Torus)
        return r;
    const Form surface = form(on);
    const xyz cx(given.x()), cy(given.y());
    const Conic curve{ConicKind(static_cast<int>(given.kind()) - 1), {xyz(given.origin()), cx, cy, cx.cross(cy)}, given.radius(), given.minor()};
    r.done = true;
    bool inside = false;
    std::vector<std::pair<double, bool>> found;
    const auto distance = [&](double t) { return surface.distance(curve.point(t)); };
    const bool line = curve.kind == ConicKind::Line, quadric_kind = surface.kind != FormKind::Plane && surface.kind != FormKind::Torus;
    if (line && surface.kind == FormKind::Plane) {
        const double rate = curve.frame.x.dot(surface.frame.z), h = (curve.frame.o - surface.frame.o).dot(surface.frame.z);
        if (std::abs(rate) > angular)
            found.push_back({-h / rate, false});
        else
            inside = std::abs(h) <= tolerance;
    } else if (line && quadric_kind) {
        // IntAna_IntConicQuad: the quadric along the line is A t t + B t + C = 0.
        const Quadric q = quadric(surface);
        const xyz w = curve.frame.o - q.c;
        const double A = curve.frame.x.dot(q.apply(curve.frame.x)), B = 2 * curve.frame.x.dot(q.apply(w)), C = q.value(curve.frame.o);
        if (std::abs(A) <= 1e-14) {
            if (std::abs(B) > 1e-14 * (1.0 + w.length()))
                found.push_back({-C / B, false});
            else
                inside = std::abs(distance(0.0)) <= tolerance;
        } else {
            const double middle = -B / (2 * A), disc = B * B - 4 * A * C;
            if (std::abs(distance(middle)) <= tolerance)
                found.push_back({middle, q.apply(curve.point(middle) - q.c).length() > tolerance});
            else if (disc > 0)
                found.insert(found.end(), {{middle - std::sqrt(disc) / (2 * std::abs(A)), false}, {middle + std::sqrt(disc) / (2 * std::abs(A)), false}});
        }
    } else if (!line && surface.kind == FormKind::Plane) {
        // The height of a circle or ellipse over the plane is alpha cos t + beta sin t + gamma.
        const double minor = curve.kind == ConicKind::Circle ? curve.radius : curve.minor;
        const double alpha = curve.radius * curve.frame.x.dot(surface.frame.z), beta = minor * curve.frame.y.dot(surface.frame.z);
        const double gamma = (curve.frame.o - surface.frame.o).dot(surface.frame.z), reach = std::hypot(alpha, beta), phase = std::atan2(beta, alpha);
        if (reach <= tolerance)
            inside = std::abs(gamma) <= tolerance;
        else if (std::abs(std::abs(gamma) - reach) <= tolerance)
            found.push_back({turn(gamma < 0 ? phase : phase + M_PI), true});
        else if (std::abs(gamma) < reach)
            found.insert(found.end(), {{turn(phase - std::acos(-gamma / reach)), false}, {turn(phase + std::acos(-gamma / reach)), false}});
    } else if (!line && quadric_kind) {
        // IntAna_IntConicQuad: the quadric along the conic is A cos cos + B cos sin + C sin sin + D cos + E sin + F, a quartic in
        // w = tan(t / 2); where it turns within tolerance of zero the conic touches the surface.
        const Quadric q = quadric(surface);
        const xyz w0 = curve.frame.o - q.c, ax = curve.frame.x * curve.radius, by = curve.frame.y * (curve.kind == ConicKind::Circle ? curve.radius : curve.minor);
        const double A = ax.dot(q.apply(ax)), B = 2 * ax.dot(q.apply(by)), C = by.dot(q.apply(by)), D = 2 * ax.dot(q.apply(w0)), E = 2 * by.dot(q.apply(w0)), F = q.value(curve.frame.o);
        std::vector<double> turns;
        for (double w : real_roots({A + D + F, 2 * B + 2 * E, -2 * A + 4 * C + 2 * F, 2 * E - 2 * B, A - D + F}, &turns))
            found.push_back({turn(2 * std::atan(w)), false});
        turns.push_back(infinity);
        for (double w : turns) {
            const double t = std::isfinite(w) ? turn(2 * std::atan(w)) : M_PI;
            if (std::abs(distance(t)) <= tolerance && std::none_of(found.begin(), found.end(), [&](const auto& f) { return (curve.point(f.first) - curve.point(t)).length() <= tolerance; }))
                found.push_back({t, true});
        }
        inside = true;
        for (int i = 0; i < 7; ++i)
            inside = inside && std::abs(distance(0.9 * i)) <= tolerance;
        if (inside)
            found.clear();
    } else {
        // A torus is of degree four: the roots of the distance along the curve.
        const double reach = surface.radius + surface.minor + tolerance, at = line ? (surface.frame.o - curve.frame.o).dot(curve.frame.x) : 0.0;
        found = roots(distance, line ? at - reach : 0.0, line ? at + reach : two_pi, 2880, !line, tolerance, inside);
    }
    r.coincident = inside ? 1 : 0;
    std::sort(found.begin(), found.end());
    // Two crossings with the curve within tolerance of the surface between them are one touching point.
    for (size_t i = 0; i < found.size() && found.size() > 1;) {
        // On a closed curve the last and the first are neighbours too.
        const size_t j = (i + 1) % found.size();
        const double next = found[j].first + (j ? 0.0 : two_pi), middle = 0.5 * (found[i].first + next);
        if ((j || !line) && std::abs(distance(middle)) <= tolerance && (curve.point(found[i].first) - curve.point(next)).length() <= std::sqrt(tolerance)) {
            found[i] = {line ? middle : turn(middle), true};
            found.erase(found.begin() + j);
        } else {
            ++i;
        }
    }
    // A hit where the curve runs along the surface is a touching one, however the roots fell.
    for (const auto& [t, tangent] : found) {
        const double slope = std::abs(distance(t + 1e-6) - distance(t - 1e-6)) / (2e-6 * curve.tangent(t).length());
        const double own = line ? t : Curve::wrapped(t, given.domain().first, two_pi);
        r.points.push_back({Point(curve.point(t)), own, parameters(on, curve.point(t)), {}, tangent || slope <= 1e-5});
    }
    return r;
}

}
