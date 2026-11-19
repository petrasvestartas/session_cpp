#include "intersection_analytic.h"
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

// (u, v) of the curve's point at t on the surface, followed from the start of its domain: each step takes the nearest turn of
// an angle and is halved while it is long or the angle moves by more than half a radian, so that the turn taken is the continuous one.
P2 lift(const Curve& curve, const Surface& surface, double t, double inset = 0.0) {
    // An end on a pole of a sphere has no angle of its own: it takes the one just inside.
    const double first = curve.domain().first + inset, last = curve.domain().second - inset;
    const auto at = [&](double v) { return parameters(surface, xyz(curve.point_at(std::clamp(v, first, last)))); };
    const double limit = curve.period() > 0 ? 0.5 : (curve.domain().second - curve.domain().first) / 16;
    const std::function<void(double, std::pair<double, double>&, double, int)> follow = [&](double from, std::pair<double, double>& uv, double to, int depth) {
        std::pair<double, double> next = at(to);
        bool far = false;
        for (int d = 0; d < 2; ++d) {
            double& now = d ? next.second : next.first;
            const double before = d ? uv.second : uv.first, period = surface.period(d);
            if (period > 0)
                now += period * std::round((before - now) / period), far = far || std::abs(now - before) > 0.5;
        }
        if ((far || std::abs(to - from) > limit) && depth < 40) {
            follow(from, uv, 0.5 * (from + to), depth + 1);
            follow(0.5 * (from + to), uv, to, depth + 1);
        } else {
            uv = next;
        }
    };
    std::pair<double, double> uv = at(curve.domain().first);
    follow(curve.domain().first, uv, t, 0);
    return {uv.first, uv.second};
}

// The pcurve of a section on a surface, on the section's parameter.
Curve2d pcurve(const Curve& curve, const Conic* conic, const Surface& surface, double tol) {
    const auto [t0, t1] = curve.domain();
    const auto pole = [&](double t) {
        const std::vector<xyz> points = singular(surface);
        return std::any_of(points.begin(), points.end(), [&](const xyz& q) { return (xyz(curve.point_at(t)) - q).length() <= tol; });
    };
    const double inset = pole(t0) || pole(t1) ? 1e-11 * (t1 - t0) : 0.0;
    const auto lift = [&](const Curve& c, const Surface& on, double t) { return intersection_analytic::lift(c, on, t, inset); };
    // Straight at unit speed, judged between two points inside the range.
    const double ta = t0 + 0.25 * (t1 - t0), tb = t0 + 0.75 * (t1 - t0);
    const P2 pa = lift(curve, surface, ta), pb = lift(curve, surface, tb);
    const P2 d = {(pb[0] - pa[0]) / (tb - ta), (pb[1] - pa[1]) / (tb - ta)}, p0 = {pa[0] - (ta - t0) * d[0], pa[1] - (ta - t0) * d[1]};
    bool straight = std::abs(std::hypot(d[0], d[1]) - 1.0) <= 1e-10;
    for (double f : {0.08, 0.41, 0.5, 0.93}) {
        const double t = t0 + f * (t1 - t0);
        const P2 p = lift(curve, surface, t);
        for (int k = 0; k < 2; ++k)
            straight = straight && std::abs(p[k] - (p0[k] + (t - t0) * d[k])) <= 1e-10 * (1.0 + std::abs(p0[k]) + std::abs(t - t0));
    }
    // A ruling or a circle about the axis: a 2D line run at unit speed.
    if (straight)
        return Curve2d(Curve::line(Point(p0[0] - t0 * d[0], p0[1] - t0 * d[1], 0), Vector(d[0], d[1], 0), t0, t1));
    if (surface.kind() == Surface::Kind::Plane) {
        const auto flat = [&](const xyz& v) { return Vector(v.dot(xyz(surface.x())), v.dot(xyz(surface.y())), 0); };
        const Vector centre = conic ? flat(conic->frame.o - xyz(surface.origin())) : Vector(0, 0, 0);
        if (conic && conic->kind == ConicKind::Circle)
            return Curve2d(Curve::circle(Point(centre[0], centre[1], 0), flat(conic->frame.x), flat(conic->frame.y), conic->radius, t0, t1));
        if (conic && conic->kind == ConicKind::Ellipse)
            return Curve2d(Curve::ellipse(Point(centre[0], centre[1], 0), flat(conic->frame.x), flat(conic->frame.y), conic->radius, conic->minor, t0, t1));
        // A NURBS section: the image of its control points, with their weights.
        NurbsCurve image = curve.nurbs();
        for (int i = 0; i < image.cv_count(); ++i) {
            const Vector uv = flat(xyz(curve.nurbs().get_cv(i)) - xyz(surface.origin()));
            const double w = image.is_rational() ? image.weight(i) : 1.0;
            image.set_cv_4d(i, uv[0] * w, uv[1] * w, 0.0, w);
        }
        return Curve2d(image);
    }
    // Anything else (an ellipse on a cylinder, a conic on a cone, a tilted circle on a sphere): interpolated at the section's
    // parameters, refined until the surface point under it is on the section.
    std::vector<double> t;
    const int first = std::max(32, curve.kind() == Curve::Kind::Nurbs ? 2 * curve.nurbs().cv_count() : 0);
    for (int i = 0; i <= first; ++i)
        t.push_back(t0 + (t1 - t0) * i / first);
    NurbsCurve fitted;
    for (int round = 0; round < 30; ++round) {
        std::vector<Point> points;
        for (double v : t) {
            const P2 p = lift(curve, surface, v);
            points.push_back(Point(p[0], p[1], 0));
        }
        fitted = NurbsCurve::create_interpolated(points, t);
        std::vector<double> more = {t.front()};
        for (size_t i = 1; i < t.size(); ++i) {
            const double middle = 0.5 * (t[i - 1] + t[i]);
            const Point uv = fitted.point_at(middle);
            if (surface.point_at(uv[0], uv[1]).distance(curve.point_at(middle)) > 0.01 * tol)
                more.push_back(middle);
            more.push_back(t[i]);
        }
        if (more.size() == t.size() || more.size() > 8000)
            break;
        t = std::move(more);
    }
    return Curve2d(fitted);
}

// IntAna_IntQuadQuad's parametrisation: one surface swept by an angle (the rulings of a cylinder or cone, the tube circles of
// a torus), on each of which the other surface marks at most two points, the roots of a quadratic that are real where its
// discriminant is positive.
struct Pencil {
    std::function<double(double)> discriminant;
    std::function<xyz(double, int)> point; // at an angle on the branch -1 or +1; on branch 0, or with a discriminant below zero, the turning point between them
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
                const std::array<double, 3> k = terms(angle, from, along);
                const double root = std::sqrt(std::max(k[1] * k[1] - 4 * k[0] * k[2], 0.0));
                // The branches by the sign of the root over the leading term: they stay continuous where it changes sign.
                return from + along * (std::abs(k[0]) > 1e-14 ? (-k[1] + branch * root) / (2 * k[0]) : branch > 0 ? -k[2] / k[1] : infinity);
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
                const double v = std::atan2(k[1], k[0]) + (branch < 0 ? -1 : 1) * std::acos(std::clamp(-k[2] / std::hypot(k[0], k[1]), -1.0, 1.0));
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

struct Builder {
    const Surface& sa;
    const Surface& sb;
    Form a, b;
    double tol;
    Result result;
    std::vector<Conic> circles;

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
        } else {
            // One arc of degree two: its end tangents meet in the middle control point, the point at the middle parameter gives its weight.
            const xyz p0 = c.point(range.first), p2 = c.point(range.second), d0 = c.tangent(range.first), d2 = c.tangent(range.second), w = c.point(0.5 * (range.first + range.second));
            const xyz n = d0.cross(d2);
            const xyz p1 = p0 + d0 * ((p2 - p0).cross(d2).dot(n) / n.dot(n));
            const xyz area = (p1 - p0).cross(p2 - p0);
            const double la = (p1 - w).cross(p2 - w).dot(area) / area.dot(area), lc = (p0 - w).cross(p1 - w).dot(area) / area.dot(area);
            NurbsCurve arc = NurbsCurve::create_from_parameters({Point(p0), Point(p1), Point(p2)}, {1.0, (1.0 - la - lc) / (2.0 * std::sqrt(la * lc)), 1.0}, {range.first, range.second}, {3, 3}, 2);
            section = Curve(arc);
        }
        add(section, &c, tangent, 0.0);
    }

    // A section over a pole of a sphere or the apex of a cone is cut there: the angle about the axis jumps.
    void add(const Curve& whole, const Conic* c, bool tangent, double fit) {
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
            piece(section, c, tangent, fit);
        }
    }

    void piece(const Curve& section, const Conic* c, bool tangent, double fit) {
        Section made{section, pcurve(section, c, sa, tol), pcurve(section, c, sb, tol), tangent, fit, 0.0, 0.0};
        made.deviation_a = deviation(section, made.on_a, sa), made.deviation_b = deviation(section, made.on_b, sb);
        result.curves.push_back(std::move(made));
    }

    // A section given by its points along a key. A circle (the Villarceau circles of a torus) is returned as one, once; anything
    // else as a least-squares NURBS curve of degree five with as few control points as keep it within a tenth of the tolerance
    // of both surfaces, its measured distance from them going with it.
    void fitted(const std::function<xyz(double)>& at, double k0, double k1) {
        const auto off = [&](const xyz& p) { return std::max(std::abs(a.distance(p)), std::abs(b.distance(p))); };
        // Points evenly along the curve, not along the key: the keys are respaced by the length run so far, twice.
        const int m = 1024;
        std::vector<double> keys(m + 1);
        for (int i = 0; i <= m; ++i)
            keys[i] = k0 + (k1 - k0) * i / m;
        std::vector<Point> samples(m + 1);
        for (int pass = 0; pass < 3; ++pass) {
            std::vector<double> run(m + 1, 0.0);
            for (int i = 0; i <= m; ++i) {
                samples[i] = Point(at(keys[i]));
                if (i)
                    run[i] = run[i - 1] + samples[i].distance(samples[i - 1]);
            }
            if (pass == 2 || !(run[m] > 0))
                break;
            std::vector<double> even(m + 1);
            for (int i = 0, j = 0; i <= m; ++i) {
                const double want = run[m] * i / m;
                while (j + 1 < m && run[j + 1] < want)
                    ++j;
                even[i] = keys[j] + (keys[j + 1] - keys[j]) * (run[j + 1] > run[j] ? (want - run[j]) / (run[j + 1] - run[j]) : 0.0);
            }
            even[0] = k0, even[m] = k1;
            keys = even;
        }
        const xyz p0(samples[m / 6]), p1(samples[m / 2]), p2(samples[5 * m / 6]), normal = unit((p1 - p0).cross(p2 - p0));
        if (normal.length() > 0) {
            // The circle through three points inside the arc (its ends may be crossings, known less well): its centre is where the
            // two chords' middle planes meet its plane.
            const xyz u = p1 - p0, v = p2 - p0;
            const double uu = u.dot(u), vv = v.dot(v), uv = u.dot(v), det = 2 * (uu * vv - uv * uv);
            const xyz centre = p0 + u * (vv * (uu - uv) / det) + v * (uu * (vv - uv) / det);
            const double radius = (p0 - centre).length();
            const Conic circle = make(ConicKind::Circle, centre, unit(p0 - centre), normal, radius);
            bool round = true;
            for (int i = 0; i <= m && round; i += 16)
                round = std::abs((xyz(samples[i]) - centre).length() - radius) <= 10 * tol && std::abs((xyz(samples[i]) - centre).dot(normal)) <= 10 * tol;
            for (int i = 0; i < 64 && round; ++i)
                round = off(circle.point(two_pi * i / 64)) <= 0.1 * tol;
            if (round) {
                const bool known = std::any_of(circles.begin(), circles.end(), [&](const Conic& c) { return (c.frame.o - centre).length() <= tol && std::abs(c.radius - radius) <= tol && c.frame.z.cross(normal).length() <= 1e-9; });
                if (!known)
                    circles.push_back(circle), curve(circle);
                return;
            }
        }
        NurbsCurve fit;
        double reached = infinity;
        for (int count = 12; count <= 96; count += std::max(4, count / 3)) {
            const NurbsCurve trial = NurbsCurve::create_fitted(samples, count, 5);
            if (!trial.is_valid())
                continue;
            double worst = 0.0;
            for (int i = 0; i <= 16 * count; ++i)
                worst = std::max(worst, off(xyz(trial.point_at(trial.domain_start() + (trial.domain_end() - trial.domain_start()) * i / (16 * count)))));
            if (worst < reached)
                fit = trial, reached = worst;
            if (worst <= 0.1 * tol)
                break;
        }
        if (reached <= 0.1 * tol)
            return add(Curve(fit), nullptr, false, reached);
        // Too sharp for a light fit: interpolated through as many points as it takes.
        interpolated(at, k0, k1);
    }

    void interpolated(const std::function<xyz(double)>& at, double k0, double k1) {
        const auto off = [&](const Point& p) { return std::max(std::abs(a.distance(xyz(p))), std::abs(b.distance(xyz(p)))); };
        std::vector<double> keys;
        for (int i = 0; i <= 48; ++i)
            keys.push_back(k0 + (k1 - k0) * i / 48);
        NurbsCurve curve;
        for (int round = 0; round < 10; ++round) {
            std::vector<Point> points;
            for (double k : keys)
                points.push_back(Point(at(k)));
            curve = NurbsCurve::create_interpolated(points);
            const std::vector<double> spans = curve.get_span_vector();
            std::vector<double> more = {keys.front()};
            for (size_t i = 1; i < keys.size(); ++i) {
                if (i < spans.size() && off(curve.point_at(0.5 * (spans[i - 1] + spans[i]))) > 0.1 * tol)
                    more.push_back(0.5 * (keys[i - 1] + keys[i]));
                more.push_back(keys[i]);
            }
            if (more.size() == keys.size() || more.size() > 4000)
                break;
            keys = std::move(more);
        }
        double fit = 0.0;
        for (int i = 0; i <= 8 * curve.cv_count(); ++i)
            fit = std::max(fit, off(curve.point_at(curve.domain_start() + (curve.domain_end() - curve.domain_start()) * i / (8 * curve.cv_count()))));
        add(Curve(curve), nullptr, false, fit);
    }

    // The sections of a pencil: over each stretch of angles with two real points, the two branches run out and back as one
    // closed curve; with two real points all round, each branch is a closed curve of its own; a discriminant that only
    // reaches zero is a touching point. Parts outside the boxes of the domains are left out.
    void sweep(const Pencil& pencil) {
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
        // A closed chain cut to the keys inside the boxes, one key past them on each side.
        const auto chain = [&](const std::function<xyz(double)>& at, double k0, double k1) {
            const int m = 720;
            std::vector<char> in(m + 1);
            for (int i = 0; i <= m; ++i)
                in[i] = inside(at(k0 + (k1 - k0) * i / m));
            if (std::all_of(in.begin(), in.end(), [](char v) { return v; }))
                return fitted(at, k0, k1);
            for (int i = 0; i <= m; ++i)
                if (in[i] && (i == 0 || !in[i - 1])) {
                    int j = i;
                    while (j < m && in[j + 1])
                        ++j;
                    const auto finite = [&](int k) { return std::isfinite(at(k0 + (k1 - k0) * k / m).length()); };
                    const int first = i > 0 && finite(i - 1) ? i - 1 : i, last = j < m && finite(j + 1) ? j + 1 : j;
                    if (last - first >= 2)
                        fitted(at, k0 + (k1 - k0) * first / m, k0 + (k1 - k0) * last / m);
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
        // Where the discriminant touches zero from above two branches cross: the section is cut there into simple arcs.
        const double top = *std::max_element(d.begin(), d.end()), golden = 0.5 * (std::sqrt(5.0) - 1.0);
        std::vector<double> crossings;
        for (int i = 0; i < n; ++i) {
            const double before = d[i > 0 ? i - 1 : n - 1], after = d[i + 1];
            if (!(d[i] >= 0 && before >= d[i] && after > d[i] && before >= 0 && after >= 0 && d[i] <= 1e-4 * top))
                continue;
            double lo = two_pi * (i - 1) / n, hi = two_pi * (i + 1) / n;
            for (int it = 0; it < 80; ++it) {
                const double m1 = hi - golden * (hi - lo), m2 = lo + golden * (hi - lo);
                const bool left = pencil.discriminant(m1) < pencil.discriminant(m2);
                (left ? hi : lo) = left ? m2 : m1;
            }
            const double angle = 0.5 * (lo + hi);
            if ((pencil.point(angle, 1) - pencil.point(angle, -1)).length() <= tol)
                crossings.push_back(turn(angle));
        }
        std::sort(crossings.begin(), crossings.end());
        // A closed chain over a turn of its key, cut at the given keys.
        const auto arcs = [&](const std::function<xyz(double)>& at, std::vector<double> cuts) {
            std::sort(cuts.begin(), cuts.end());
            if (cuts.empty())
                return chain(at, 0.0, two_pi);
            for (size_t i = 0; i < cuts.size(); ++i)
                chain(at, cuts[i], i + 1 < cuts.size() ? cuts[i + 1] : cuts[0] + two_pi);
        };
        std::vector<double> rise, fall;
        for (int i = 0; i < n; ++i) {
            if (d[i] < 0 && d[i + 1] >= 0)
                rise.push_back(zero(two_pi * i / n, two_pi * (i + 1) / n));
            if (d[i] >= 0 && d[i + 1] < 0)
                fall.push_back(zero(two_pi * i / n, two_pi * (i + 1) / n));
        }
        // A dip of the discriminant below zero that closes within the tolerance is a crossing met with rounding, not a turn.
        bool dips = !rise.empty() && rise.size() == fall.size();
        std::vector<double> met;
        for (size_t i = 0; i < rise.size() && dips; ++i) {
            double from = -infinity;
            for (double f : fall)
                from = std::max(from, f < rise[i] ? f : f - two_pi);
            dips = (pencil.point(from, 0) - pencil.point(rise[i], 0)).length() <= tol && rise[i] - from < 0.1;
            met.push_back(turn(0.5 * (from + rise[i])));
        }
        if (dips) {
            crossings.insert(crossings.end(), met.begin(), met.end());
            std::sort(crossings.begin(), crossings.end());
            rise.clear();
        }
        if (rise.empty()) {
            if (d[0] >= 0 || dips) {
                for (int branch : {-1, 1})
                    arcs([&, branch](double angle) { return pencil.point(angle, branch); }, crossings);
                return;
            }
            // Never two points: where the discriminant comes nearest to zero the surfaces may touch.
            const int top = static_cast<int>(std::max_element(d.begin(), d.end()) - d.begin());
            double lo = two_pi * (top - 1) / n, hi = two_pi * (top + 1) / n;
            for (int it = 0; it < 80; ++it) {
                const double m1 = hi - golden * (hi - lo), m2 = lo + golden * (hi - lo);
                const bool left = pencil.discriminant(m1) > pencil.discriminant(m2);
                (left ? hi : lo) = left ? m2 : m1;
            }
            const xyz p = pencil.point(0.5 * (lo + hi), 0);
            if (std::isfinite(p.x) && std::abs(a.distance(p)) <= tol && std::abs(b.distance(p)) <= tol && inside(p))
                point(p, true);
            return;
        }
        for (double from : rise) {
            double to = infinity;
            for (double f : fall)
                to = std::min(to, f > from ? f : f + two_pi);
            // Out on one branch and back on the other, evenly in the angle of a half turn so that the ends, where the root
            // goes as a square root, are sampled finely; a loop no larger than the tolerance is a touching point.
            const std::function<xyz(double)> loop = [&, from, to](double key) {
                return pencil.point(0.5 * (from + to) - 0.5 * (to - from) * std::cos(key), std::abs(std::sin(key)) < 1e-12 ? 0 : std::sin(key) > 0 ? 1 : -1);
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

// Roots of a distance along a curve: sign changes by bisection, and minima within tol where it only touches zero.
std::vector<std::pair<double, bool>> roots(const std::function<double(double)>& f, double t0, double t1, int n, bool periodic, double tol, bool& all) {
    std::vector<double> v(n + 1);
    all = true;
    for (int i = 0; i <= n; ++i)
        v[i] = f(t0 + (t1 - t0) * i / n), all = all && std::abs(v[i]) <= tol;
    std::vector<std::pair<double, bool>> found;
    const double step = (t1 - t0) / n, golden = 0.5 * (std::sqrt(5.0) - 1.0);
    for (int i = 0; i < n && !all; ++i) {
        if (v[i] == 0.0 || v[i] * v[i + 1] < 0) {
            double lo = t0 + i * step, hi = lo + step, flo = v[i];
            for (int it = 0; it < 80 && flo != 0.0; ++it) {
                const double mid = 0.5 * (lo + hi), fm = f(mid);
                (fm * flo > 0 ? lo : hi) = mid;
                flo = fm * flo > 0 ? fm : flo;
            }
            found.push_back({flo == 0.0 ? lo : 0.5 * (lo + hi), false});
            continue;
        }
        const double before = i > 0 ? v[i - 1] : periodic ? v[n - 1] : v[i];
        if (std::abs(v[i]) > std::abs(before) || std::abs(v[i]) > std::abs(v[i + 1]) || v[i] * before < 0)
            continue;
        double lo = t0 + (i - 1) * step, hi = t0 + (i + 1) * step;
        for (int it = 0; it < 80; ++it) {
            const double m1 = hi - golden * (hi - lo), m2 = lo + golden * (hi - lo);
            const bool left = std::abs(f(m1)) < std::abs(f(m2));
            (left ? hi : lo) = left ? m2 : m1;
        }
        const double t = 0.5 * (lo + hi);
        if (std::abs(f(t)) <= tol && (found.empty() || std::abs(found.back().first - t) > 4 * step))
            found.push_back({t, true});
    }
    return found;
}

}

Result surface_surface(const Surface& first, const Surface& second, double tolerance) {
    Result none;
    if (first.kind() == Surface::Kind::Nurbs || second.kind() == Surface::Kind::Nurbs)
        return none;
    Builder out{first, second, form(first), form(second), tolerance, {}, {}};
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
        if (std::abs(other.frame.z.dot(plane.frame.z)) > angular || std::abs((other.frame.o - plane.frame.o).dot(plane.frame.z)) > tolerance)
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
        if (!cylinders || (cross.length() > angular && !(std::abs(a.radius - b.radius) <= tolerance && std::abs(d.dot(unit(cross))) <= tolerance))) {
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

Result curve_surface(const Curve& given, const Surface& on, double tolerance) {
    Result r;
    if (given.kind() == Curve::Kind::Nurbs || on.kind() == Surface::Kind::Nurbs)
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
    for (size_t i = 0; i + 1 < found.size();)
        if (std::abs(distance(0.5 * (found[i].first + found[i + 1].first))) <= tolerance && (curve.point(found[i].first) - curve.point(found[i + 1].first)).length() <= std::sqrt(tolerance)) {
            found[i] = {0.5 * (found[i].first + found[i + 1].first), true};
            found.erase(found.begin() + i + 1);
        } else {
            ++i;
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
