#include "intersection_sweep.h"
#include "curve_intersection.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numbers>

namespace session_cpp::intersection_sweep {
namespace {

using intersection_analytic::Result;
using intersection_analytic::Section;
using Kind = Curve::Kind;
using P2 = std::array<double, 2>;
using Poly = std::vector<double>;
constexpr double two_pi = 2.0 * std::numbers::pi;
constexpr double angular = 1e-12;
constexpr double infinity = std::numeric_limits<double>::infinity();

xyz unit(const xyz& v) {
    const double l = v.length();
    return l > 0 ? v / l : v;
}

Poly times(const Poly& a, const Poly& b) {
    Poly c(a.size() + b.size() - 1, 0.0);
    for (size_t i = 0; i < a.size(); ++i)
        for (size_t j = 0; j < b.size(); ++j)
            c[i + j] += a[i] * b[j];
    return c;
}

Poly plus(Poly a, const Poly& b, double k = 1.0) {
    a.resize(std::max(a.size(), b.size()), 0.0);
    for (size_t i = 0; i < b.size(); ++i)
        a[i] += k * b[i];
    return a;
}

double horner(const Poly& c, double x) {
    double v = 0.0;
    for (size_t i = c.size(); i-- > 0;)
        v = v * x + c[i];
    return v;
}

// math_DirectPolynomialRoots in effect: the real roots of c0 + c1 x + ... in [lo, hi], each between two roots of the derivative
// by bisection; `turns` gets the derivative's roots, where the polynomial may touch zero without crossing it.
std::vector<double> real_roots(Poly c, double lo, double hi, std::vector<double>* turns = nullptr) {
    const double top = std::abs(*std::max_element(c.begin(), c.end(), [](double l, double r) { return std::abs(l) < std::abs(r); }));
    while (c.size() > 1 && std::abs(c.back()) <= 1e-13 * top)
        c.pop_back();
    if (c.size() < 2)
        return {};
    double reach = 0.0;
    for (size_t i = 0; i + 1 < c.size(); ++i)
        reach = std::max(reach, std::abs(c[i] / c.back()));
    lo = std::max(lo, -1.0 - reach), hi = std::min(hi, 1.0 + reach);
    Poly d;
    for (size_t i = 1; i < c.size(); ++i)
        d.push_back(i * c[i]);
    std::vector<double> stops = real_roots(d, lo, hi), found;
    if (turns)
        *turns = stops;
    stops.insert(stops.begin(), lo);
    stops.push_back(hi);
    for (size_t i = 0; i + 1 < stops.size(); ++i) {
        double from = stops[i], to = stops[i + 1], f = horner(c, from);
        if (f * horner(c, to) > 0 || !(to > from))
            continue;
        for (int it = 0; it < 200 && f != 0.0 && to - from > 1e-16 * (1.0 + std::abs(from)); ++it) {
            const double mid = 0.5 * (from + to), fm = horner(c, mid);
            (fm * f > 0 ? from : to) = mid;
            f = fm * f > 0 ? fm : f;
        }
        const double root = f == 0.0 ? from : 0.5 * (from + to);
        if (found.empty() || root - found.back() > 1e-14 * (1.0 + std::abs(root)))
            found.push_back(root);
    }
    return found;
}

double cross(const Vector& a, const Vector& b) {
    return a[0] * b[1] - a[1] * b[0];
}

// A meeting point brought onto both curves by Newton on their parameters, and judged: tangent where their directions agree.
Hit2d settled(const Curve2d& a, const Curve2d& b, double ta, double tb, bool tangent) {
    for (int i = 0; i < 4 && !tangent; ++i) {
        const std::vector<Vector> pa = a.evaluate(ta, 1), pb = b.evaluate(tb, 1);
        const Vector gap = pb[0] - pa[0];
        const double det = cross(pa[1], pb[1]);
        if (!(std::abs(det) > 1e-9 * pa[1].magnitude() * pb[1].magnitude()))
            break;
        ta += cross(gap, pb[1]) / det;
        tb += cross(gap, pa[1]) / det;
    }
    const Vector da = a.evaluate(ta, 1)[1], db = b.evaluate(tb, 1)[1];
    const Point pa = a.point_at(ta), pb = b.point_at(tb);
    return {Point(0.5 * (pa[0] + pb[0]), 0.5 * (pa[1] + pb[1]), 0), Curve::wrapped(ta, a.domain().first, a.period()), Curve::wrapped(tb, b.domain().first, b.period()),
        tangent || std::abs(cross(da, db)) <= 1e-6 * da.magnitude() * db.magnitude()};
}

// The stretch of an analytic curve that can meet a curve with the given control points: a whole turn, or the open curve beside them.
NurbsCurve beside(const Curve2d& c, const NurbsCurve& other) {
    if (c.kind() == Kind::Nurbs)
        return c.nurbs();
    Curve piece = c;
    double lo = infinity, hi = -infinity;
    for (int i = 0; i < other.cv_count() && !(c.period() > 0); ++i) {
        const Vector d = other.get_cv(i) - c.origin();
        const double t = c.kind() == Kind::Line ? d.dot(c.x()) : c.kind() == Kind::Parabola ? d.dot(c.y()) : std::asinh(d.dot(c.y()) / c.minor());
        lo = std::min(lo, t), hi = std::max(hi, t);
    }
    if (c.period() > 0)
        piece.set_domain(c.domain().first, c.domain().first + two_pi);
    else
        piece.set_domain(lo - 1e-3 * (hi - lo) - 1e-9, hi + 1e-3 * (hi - lo) + 1e-9);
    return piece.nurbs();
}

Result2d numeric(const Curve2d& a, const Curve2d& b, double tol) {
    Result2d r;
    if (!a.nurbs().is_valid() || !b.nurbs().is_valid())
        return r;
    const NurbsCurve na = beside(a, b.nurbs()), nb = beside(b, a.nurbs());
    const curve_intersection::Result met = curve_intersection::intersect(na, nb, tol);
    r.done = met.complete;
    r.coincident = !met.overlaps.empty();
    for (const curve_intersection::Hit& hit : met.points)
        r.points.push_back(settled(a, b, a.kind() == Kind::Nurbs ? hit.a : a.closest_parameter(hit.point), b.kind() == Kind::Nurbs ? hit.b : b.closest_parameter(hit.point), hit.tangent));
    return r;
}

// IntAna2d_AnaIntersection: b's parametrisation, rational in w (t itself on a line or parabola, tan of half the angle on a circle or
// ellipse, exp t on a hyperbola), put into a's equation in a's own frame; the roots are the meeting points, the turning points
// within tolerance of a the touching ones.
Result2d analytic(const Curve2d& a, const Curve2d& b, double tol) {
    Result2d r;
    r.done = true;
    const auto gap = [&](double t) { const Point p = b.point_at(t); return a.closest_point(p).distance(p); };
    const auto [b0, b1] = b.domain();
    r.coincident = true;
    for (int i = 0; i <= 8 && r.coincident; ++i)
        r.coincident = gap(b0 + (b1 - b0) * i / 8) <= tol;
    if (r.coincident)
        return r;
    const bool round = b.period() > 0;
    std::vector<std::pair<double, bool>> found;
    for (double chart : {0.0, std::numbers::pi}) {
        if (chart > 0 && !round)
            break;
        const double flip = chart > 0 ? -1.0 : 1.0;
        Vector A = b.x(), B = b.y();
        Poly p1 = {0, 1}, p2 = {0}, q = {1};
        double lo = -infinity, hi = infinity;
        if (b.kind() == Kind::Parabola)
            A *= 1.0 / (4.0 * b.radius()), p1 = {0, 0, 1}, p2 = {0, 1};
        else if (b.kind() == Kind::Hyperbola)
            A *= b.radius(), B *= b.minor(), p1 = {0.5, 0, 0.5}, p2 = {-0.5, 0, 0.5}, q = {0, 1}, lo = 0.0;
        else if (round)
            A *= flip * b.radius(), B *= flip * b.minor(), p1 = {1, 0, -1}, p2 = {0, 2}, q = {1, 0, 1}, lo = -1.0 - 1e-9, hi = 1.0 + 1e-9;
        const Vector d = b.origin() - a.origin(), ax = a.x(), ay = a.kind() == Kind::Line ? Vector(-ax[1], ax[0], 0) : a.y();
        const Poly xi = plus(plus(times(q, {d.dot(ax)}), p1, A.dot(ax)), p2, B.dot(ax)), eta = plus(plus(times(q, {d.dot(ay)}), p1, A.dot(ay)), p2, B.dot(ay));
        const double ra = a.radius() * a.radius(), rb = a.minor() * a.minor();
        Poly f = eta;
        if (a.kind() == Kind::Parabola)
            f = plus(times(eta, eta), times(xi, q), -4.0 * a.radius());
        else if (a.kind() == Kind::Hyperbola)
            f = plus(plus(times(xi, xi), times(eta, eta), -ra / rb), times(q, q), -ra);
        else if (a.kind() != Kind::Line)
            f = plus(plus(times(xi, xi), times(eta, eta), ra / rb), times(q, q), -ra);
        const auto parameter = [&](double w) { return round ? 2.0 * std::atan(w) + chart : b.kind() == Kind::Hyperbola ? std::log(w) : w; };
        std::vector<double> turns;
        for (double w : real_roots(f, lo, hi, &turns))
            if ((w > 0 || b.kind() != Kind::Hyperbola) && gap(parameter(w)) <= tol)
                found.push_back({parameter(w), false});
        for (double w : turns)
            if ((w > 0 || b.kind() != Kind::Hyperbola) && gap(parameter(w)) <= tol)
                found.push_back({parameter(w), true});
    }
    std::sort(found.begin(), found.end());
    const auto apart = [&](double s, double t) { return b.point_at(s).distance(b.point_at(t)); };
    // Two crossings with b within tolerance of a between them are one touching point; a turning point beside a crossing is that crossing.
    for (size_t i = 0; i + 1 < found.size();)
        if (!found[i].second && !found[i + 1].second && gap(0.5 * (found[i].first + found[i + 1].first)) <= tol && apart(found[i].first, found[i + 1].first) <= std::sqrt(tol)) {
            found[i] = {0.5 * (found[i].first + found[i + 1].first), true};
            found.erase(found.begin() + i + 1);
        } else {
            ++i;
        }
    std::vector<std::pair<double, bool>> kept;
    for (int pass = 0; pass < 2; ++pass)
        for (const auto& f : found)
            if (f.second == (pass == 1) && std::none_of(kept.begin(), kept.end(), [&](const auto& k) { return apart(k.first, f.first) <= (pass == 1 || k.second ? std::sqrt(tol) : tol); }))
                kept.push_back(f);
    std::sort(kept.begin(), kept.end());
    for (const auto& [t, tangent] : kept)
        r.points.push_back(settled(a, b, a.closest_parameter(b.point_at(t)), t, tangent));
    return r;
}

// An affine map of space onto a plane's coordinates: a point slides along `along` into the plane of the unit e1 and e2 through o.
struct Map {
    xyz o, e1, e2, along;

    xyz vector(const xyz& v) const {
        const xyz n = e1.cross(e2), w = v - along * (v.dot(n) / along.dot(n));
        return {w.dot(e1), w.dot(e2), 0.0};
    }
};

// The image of a curve under an affine map: a curve of its own kind, in its own parameter scale * (the given curve's - shift).
struct Image {
    Curve2d curve;
    double scale = 1.0, shift = 0.0;

    double source(double t) const { return shift + t / scale; }
};

bool image(const Curve& c, const Map& map, Image& out) {
    const auto [t0, t1] = c.domain();
    if (c.kind() == Kind::Nurbs) {
        NurbsCurve n = c.nurbs();
        if (!n.is_valid())
            return false;
        for (int i = 0; i < n.cv_count(); ++i) {
            const xyz p = map.vector(xyz(c.nurbs().get_cv(i)) - map.o);
            const double w = n.is_rational() ? n.weight(i) : 1.0;
            n.set_cv_4d(i, p.x * w, p.y * w, 0.0, w);
        }
        out = {Curve2d(n), 1.0, 0.0};
        return true;
    }
    const Point o(map.vector(xyz(c.origin()) - map.o));
    xyz A = map.vector(xyz(c.x())), B = map.vector(xyz(c.y()));
    if (c.kind() == Kind::Line) {
        const double s = A.length();
        out = {Curve2d(Curve::line(o, Vector(A), s * t0, s * t1)), s, 0.0};
        return s > angular;
    }
    if (c.kind() == Kind::Parabola) {
        // o + A t t + B t about the parameter where its speed is square to A.
        A = A / (4.0 * c.radius());
        const double shift = -A.dot(B) / (2.0 * A.dot(A));
        const xyz along = B + A * (2.0 * shift);
        const double s = along.length();
        if (!(A.length() > angular) || !(s > angular))
            return false;
        out = {Curve2d(Curve::parabola(o + Vector(A * (shift * shift) + B * shift), Vector(A), Vector(along), s * s / (4.0 * A.length()), s * (t0 - shift), s * (t1 - shift))), s, shift};
        return true;
    }
    A = A * c.radius(), B = B * c.minor();
    if (!(std::abs(A.cross(B).z) > angular * (A.dot(A) + B.dot(B))))
        return false;
    if (c.kind() == Kind::Hyperbola) {
        // A cosh t + B sinh t turned by the hyperbolic angle that sets its two vectors square.
        const double shift = 0.5 * std::atanh(-2.0 * A.dot(B) / (A.dot(A) + B.dot(B)));
        const xyz a = A * std::cosh(shift) + B * std::sinh(shift), b = A * std::sinh(shift) + B * std::cosh(shift);
        out = {Curve2d(Curve::hyperbola(o, Vector(a), Vector(b), a.length(), b.length(), t0 - shift, t1 - shift)), 1.0, shift};
        return true;
    }
    if (std::abs(A.dot(B)) <= 1e-14 * A.dot(A) && std::abs(A.dot(A) - B.dot(B)) <= 1e-14 * A.dot(A)) {
        out = {Curve2d(Curve::circle(o, Vector(A), Vector(B), A.length(), t0, t1)), 1.0, 0.0};
        return true;
    }
    // A cos t + B sin t turned to its axes, the longer first.
    const double shift = 0.5 * std::atan2(2.0 * A.dot(B), A.dot(A) - B.dot(B));
    const xyz a = A * std::cos(shift) + B * std::sin(shift), b = B * std::cos(shift) - A * std::sin(shift);
    out = {Curve2d(Curve::ellipse(o, Vector(a), Vector(b), a.length(), b.length(), t0 - shift, t1 - shift)), 1.0, shift};
    return true;
}

// A curve of a plane's coordinates in space, the plane through o along the unit e1 and e2: the same kind in the same parameter.
Curve lifted(const Curve2d& c, const xyz& o, const xyz& e1, const xyz& e2) {
    const auto up = [&](const xyz& v) { return e1 * v.x + e2 * v.y; };
    const auto [t0, t1] = c.domain();
    const Point origin(o + up(xyz(c.origin())));
    const Vector x(up(xyz(c.x()))), y(up(xyz(c.y())));
    switch (c.kind()) {
    case Kind::Line: return Curve::line(origin, x, t0, t1);
    case Kind::Circle: return Curve::circle(origin, x, y, c.radius(), t0, t1);
    case Kind::Ellipse: return Curve::ellipse(origin, x, y, c.radius(), c.minor(), t0, t1);
    case Kind::Parabola: return Curve::parabola(origin, x, y, c.radius(), t0, t1);
    case Kind::Hyperbola: return Curve::hyperbola(origin, x, y, c.radius(), c.minor(), t0, t1);
    default: break;
    }
    NurbsCurve n = c.nurbs();
    for (int i = 0; i < n.cv_count(); ++i) {
        const xyz p = o + up(xyz(c.nurbs().get_cv(i)));
        const double w = n.is_rational() ? n.weight(i) : 1.0;
        n.set_cv_4d(i, p.x * w, p.y * w, p.z * w, w);
    }
    return Curve(n);
}

Map onto(const Surface& plane) {
    return {xyz(plane.origin()), xyz(plane.x()), xyz(plane.y()), xyz(plane.x()).cross(xyz(plane.y()))};
}

// GeomLib_CheckCurveOnSurface: the largest distance of the surface point under the pcurve from the section, sampled.
double deviation(const Curve& curve, const Curve2d& on, const Surface& surface) {
    const auto [t0, t1] = curve.domain();
    const int n = 16 * std::max({8, curve.kind() == Kind::Nurbs ? curve.nurbs().cv_count() : 0, on.kind() == Kind::Nurbs ? on.nurbs().cv_count() : 0});
    double worst = 0.0;
    for (int i = 0; i <= n; ++i) {
        const double t = t0 + (t1 - t0) * i / n;
        const Point uv = on.point_at(t);
        worst = std::max(worst, surface.point_at(uv[0], uv[1]).distance(curve.point_at(t)));
    }
    return worst;
}

// A straight pcurve from p0 at t0 to p1 at t1 of the section's parameter: a 2D line where it runs at unit speed, else of degree 1.
Curve2d straight(const P2& p0, const P2& p1, double t0, double t1) {
    const P2 d = {(p1[0] - p0[0]) / (t1 - t0), (p1[1] - p0[1]) / (t1 - t0)};
    if (std::abs(std::hypot(d[0], d[1]) - 1.0) <= 1e-12)
        return Curve2d(Curve::line(Point(p0[0] - t0 * d[0], p0[1] - t0 * d[1], 0), Vector(d[0], d[1], 0), t0, t1));
    NurbsCurve n = NurbsCurve::create(false, 1, {Point(p0[0], p0[1], 0), Point(p1[0], p1[1], 0)});
    n.set_domain(t0, t1);
    return Curve2d(n);
}

// A pcurve that is no line or conic: interpolated at the section's parameters, refined until the surface point under it is on the section.
Curve2d fitted(const Curve& curve, const Surface& surface, const std::function<P2(double)>& uv, double tol) {
    const auto [t0, t1] = curve.domain();
    std::vector<double> t;
    const int first = std::max(32, curve.kind() == Kind::Nurbs ? 2 * curve.nurbs().cv_count() : 0);
    for (int i = 0; i <= first; ++i)
        t.push_back(t0 + (t1 - t0) * i / first);
    NurbsCurve fit;
    for (int round = 0; round < 30; ++round) {
        std::vector<Point> points;
        for (double v : t) {
            const P2 p = uv(v);
            points.push_back(Point(p[0], p[1], 0));
        }
        fit = NurbsCurve::create_interpolated(points, t);
        std::vector<double> more = {t.front()};
        for (size_t i = 1; i < t.size(); ++i) {
            const double middle = 0.5 * (t[i - 1] + t[i]);
            const Point p = fit.point_at(middle);
            if (surface.point_at(p[0], p[1]).distance(curve.point_at(middle)) > 0.01 * tol)
                more.push_back(middle);
            more.push_back(t[i]);
        }
        if (more.size() == t.size() || more.size() > 8000)
            break;
        t = std::move(more);
    }
    return Curve2d(fit);
}

void add(Result& r, const Surface& a, const Surface& b, const Curve& curve, const Curve2d& on_a, const Curve2d& on_b, bool tangent) {
    r.curves.push_back({curve, on_a, on_b, tangent, 0.0, deviation(curve, on_a, a), deviation(curve, on_b, b)});
}

// The meridian of a surface about the axis (o, z) in (distance across, height), on the surface's own parameter along it where
// `aligned`: the profile of a revolution in a plane of the axis (toward `toward` at its angle 0), the line or circle of a plane across
// the axis, a cylinder, cone or torus about it, a sphere centred on it.
bool meridian(const Surface& s, const xyz& o, const xyz& z, double tol, Image& out, bool& aligned, xyz* toward = nullptr) {
    const xyz so(s.origin()), sz(s.z()), d = so - o;
    const double h = d.dot(z), sense = sz.dot(z) < 0 ? -1.0 : 1.0, v0 = s.domain(1).first, v1 = s.domain(1).second;
    const bool parallel = sz.cross(z).length() <= angular, centred = (d - z * h).length() <= tol;
    const Vector x(1, 0, 0), y(0, sense, 0);
    aligned = parallel && s.kind() != Surface::Kind::Plane;
    if ((!parallel && s.kind() != Surface::Kind::Sphere) || (!centred && s.kind() != Surface::Kind::Plane))
        return false;
    switch (s.kind()) {
    case Surface::Kind::Plane: out = {Curve2d(Curve::line(Point(0, h, 0), x, -1.0, 1.0)), 1.0, 0.0}; return true;
    case Surface::Kind::Cylinder: out = {Curve2d(Curve::line(Point(s.radius(), h, 0), y, v0, v1)), 1.0, 0.0}; return true;
    case Surface::Kind::Cone: out = {Curve2d(Curve::line(Point(s.radius(), h, 0), x * std::sin(s.minor()) + y * std::cos(s.minor()), v0, v1)), 1.0, 0.0}; return true;
    case Surface::Kind::Sphere: out = {Curve2d(Curve::circle(Point(0, h, 0), x, y, s.radius(), v0, v1)), 1.0, 0.0}; return true;
    case Surface::Kind::Torus: out = {Curve2d(Curve::circle(Point(s.radius(), h, 0), x, y, s.minor(), v0, v1)), 1.0, 0.0}; return true;
    case Surface::Kind::Revolution: break;
    default: return false;
    }
    const Curve profile = s.profile();
    const NurbsCurve& n = profile.nurbs();
    xyz far{0, 0, 0};
    for (int i = 0; i < n.cv_count(); ++i) {
        const xyz q = xyz(n.get_cv(i)) - o, across = q - z * q.dot(z);
        if (across.length() > far.length())
            far = across;
    }
    if (!(far.length() > tol))
        return false;
    const xyz e = unit(far), normal = z.cross(e);
    for (int i = 0; i < n.cv_count(); ++i)
        if (std::abs((xyz(n.get_cv(i)) - o).dot(normal)) > tol)
            return false;
    if (toward)
        *toward = e;
    return image(profile, {o, e, z, normal}, out);
}

// The pcurve of a circle about the axis of a surface, at the surface's parameter v along its meridian: a circle on a plane, a line
// along u on a surface about that axis, fitted on a sphere whose own axis is another.
bool ring_pcurve(const Surface& s, const Curve& circle, double v, bool aligned, double tol, Curve2d& out) {
    Image flat;
    if (s.kind() == Surface::Kind::Plane) {
        image(circle, onto(s), flat);
        out = flat.curve;
        return true;
    }
    const xyz so(s.origin()), sx(s.x()), sy(s.y());
    const auto angle = [&](const Point& p) { return std::atan2((xyz(p) - so).dot(sy), (xyz(p) - so).dot(sx)); };
    if (aligned) {
        const double u = Curve::wrapped(angle(circle.point_at(0.0)) - angle(s.point_at(0.0, v)), s.domain(0).first, two_pi);
        out = Curve2d(Curve::line(Point(u, v, 0), Vector(circle.evaluate(0.0, 1)[1].dot(s.evaluate(u, v, 1)[2]) < 0 ? -1.0 : 1.0, 0, 0), 0.0, two_pi));
        return true;
    }
    // The angle followed round from node to node; a circle over a pole has none there.
    const int n = 256;
    std::vector<double> nodes(n + 1);
    for (int i = 0; i <= n; ++i) {
        const Point p = circle.point_at(two_pi * i / n);
        if (std::abs(std::abs((xyz(p) - so).dot(xyz(s.z()))) - s.radius()) <= tol)
            return false;
        nodes[i] = i ? angle(p) + two_pi * std::round((nodes[i - 1] - angle(p)) / two_pi) : Curve::wrapped(angle(p), s.domain(0).first, two_pi);
    }
    out = fitted(circle, s, [&](double t) {
        const Point p = circle.point_at(t);
        const double near = nodes[std::clamp(static_cast<int>(std::round(t / two_pi * n)), 0, n)];
        return P2{angle(p) + two_pi * std::round((near - angle(p)) / two_pi), s.closest_parameters(p).second};
    }, tol);
    return true;
}

bool ring(Result& r, const Surface& a, const Surface& b, const xyz& centre, const xyz& z, double radius, double va, bool aligned_a, double vb, bool aligned_b, bool tangent, double tol) {
    if (radius <= tol) {
        r.points.push_back({Point(centre), 0.0, a.closest_parameters(Point(centre)), b.closest_parameters(Point(centre)), tangent});
        return true;
    }
    const xyz ax(a.x()), across = unit(ax - z * ax.dot(z));
    const Curve circle = Curve::circle(Point(centre), Vector(across), Vector(z.cross(across)), radius);
    Curve2d on_a, on_b;
    if (!ring_pcurve(a, circle, va, aligned_a, tol, on_a) || !ring_pcurve(b, circle, vb, aligned_b, tol, on_b))
        return false;
    add(r, a, b, circle, on_a, on_b, tangent);
    return true;
}

// The latitude of a sphere's meridian circle taken past a pole comes back to the near side.
double latitude(const Surface& s, double v) {
    return s.kind() == Surface::Kind::Sphere ? std::atan2(std::sin(v), std::abs(std::cos(v))) : v;
}

// Two surfaces of revolution about one axis meet in circles across it, where one's meridian meets the other's or its mirror image.
bool coaxial(const Surface& a, const Surface& b, const xyz& o, const xyz& z, double tol, Result& r) {
    Image ma, mb;
    bool aligned_a = false, aligned_b = false;
    if (!meridian(a, o, z, tol, ma, aligned_a) || !meridian(b, o, z, tol, mb, aligned_b))
        return false;
    struct Met {
        double rho, h, va, vb;
        bool tangent;
    };
    std::vector<Met> all;
    for (int side = 0; side < 2; ++side) {
        const Curve2d other = side ? Curve2d(mb.curve.transformed(Xform::scale_xyz(-1.0, 1.0, 1.0))) : mb.curve;
        const Result2d met = curve_curve(ma.curve, other, tol);
        if (!met.done || met.coincident)
            return false;
        // A circle met from both sides of the axis is kept once, at the parameters inside the domains.
        const auto inside = [](const Surface& s, double v) { return v >= s.domain(1).first && v <= s.domain(1).second; };
        for (const Hit2d& hit : met.points) {
            const Met now = {std::abs(hit.point[0]), hit.point[1], latitude(a, ma.source(hit.a)), latitude(b, mb.source(hit.b)), hit.tangent};
            // Where the meridians touch the point is known to the root of the tolerance only.
            const auto same = std::find_if(all.begin(), all.end(), [&](const Met& m) { return std::hypot(m.rho - now.rho, m.h - now.h) <= (m.tangent || now.tangent ? std::sqrt(tol) : tol); });
            if (same == all.end())
                all.push_back(now);
            else if (inside(a, now.va) + inside(b, now.vb) > inside(a, same->va) + inside(b, same->vb))
                *same = now;
        }
    }
    std::sort(all.begin(), all.end(), [](const Met& l, const Met& m) { return l.h < m.h; });
    for (const Met& m : all)
        if (!ring(r, a, b, o + z * m.h, z, m.rho, m.va, aligned_a, m.vb, aligned_b, m.tangent, tol))
            return false;
    return true;
}

// A revolution of any profile cut by a plane across its axis: a circle through each point of the profile at the plane's height,
// found on the profile seen from the side.
bool across_axis(const Surface& a, const Surface& plane, const xyz& o, const xyz& z, double tol, Result& r) {
    const Curve profile = a.profile();
    const double h = (xyz(plane.origin()) - o).dot(z);
    Image side;
    if (!image(profile, {o, xyz(a.x()), z, xyz(a.y())}, side) && !image(profile, {o, xyz(a.y()), z, xyz(a.x())}, side))
        return false;
    const Result2d met = curve_curve(side.curve, Curve2d(Curve::line(Point(0, h, 0), Vector(1, 0, 0), -1.0, 1.0)), tol);
    if (!met.done || met.coincident)
        return false;
    for (const Hit2d& hit : met.points) {
        const double v = side.source(hit.a);
        const xyz q = xyz(profile.point_at(v)) - o;
        if (!ring(r, a, plane, o + z * h, z, (q - z * q.dot(z)).length(), v, true, 0.0, false, hit.tangent, tol))
            return false;
    }
    return true;
}

// A revolution cut by a plane through its axis: the profile at the plane's two angles, each a line along v on the revolution.
bool through_axis(const Surface& a, const Surface& plane, const xyz& o, const xyz& z, double tol, Result& r) {
    Image m;
    bool aligned = false;
    xyz toward;
    if (!meridian(a, o, z, tol, m, aligned, &toward))
        return false;
    const xyz e = unit(z.cross(xyz(plane.z()))), ax(a.x()), ay(a.y());
    const auto [t0, t1] = m.curve.domain();
    // A profile that is its own mirror image in the axis gives one curve.
    bool own = true;
    for (int i = 0; i <= 8 && own; ++i) {
        const Point p = m.curve.point_at(t0 + (t1 - t0) * i / 8), q(-p[0], p[1], 0);
        const double t = m.curve.closest_parameter(q);
        own = m.curve.point_at(t).distance(q) <= tol && t >= t0 - 1e-9 && t <= t1 + 1e-9;
    }
    for (double side : {1.0, -1.0}) {
        const Curve section = lifted(m.curve, o, e * side, z);
        const double u = Curve::wrapped(std::atan2(e.dot(ay) * side, e.dot(ax) * side) - std::atan2(toward.dot(ay), toward.dot(ax)), a.domain(0).first, two_pi);
        Image flat;
        if (!image(section, onto(plane), flat))
            return false;
        add(r, a, plane, section, straight({u, m.source(t0)}, {u, m.source(t1)}, t0, t1), flat.curve, false);
        if (own)
            break;
    }
    return true;
}

bool revolution(const Surface& a, const Surface& b, double tol, Result& r) {
    const xyz o(a.origin()), z(a.z());
    if (b.kind() == Surface::Kind::Plane) {
        const xyz n(b.z());
        Image m;
        bool aligned = false;
        if (std::abs(n.dot(z)) <= angular && std::abs((o - xyz(b.origin())).dot(n)) <= tol)
            return through_axis(a, b, o, z, tol, r);
        if (n.cross(z).length() <= angular && !meridian(a, o, z, tol, m, aligned))
            return across_axis(a, b, o, z, tol, r);
    }
    return coaxial(a, b, o, z, tol, r);
}

// The parameters where a line along d through q passes the boxes of both surfaces' domains.
std::pair<double, double> reach(const Surface& a, const Surface& b, const xyz& d) {
    std::pair<double, double> range = {-infinity, infinity};
    for (const Surface* s : {&a, &b}) {
        const AABB box = s->aabb();
        std::pair<double, double> own = {infinity, -infinity};
        for (int i = 0; i < 8; ++i) {
            const double t = xyz{box.cx + (i & 1 ? box.hx : -box.hx), box.cy + (i & 2 ? box.hy : -box.hy), box.cz + (i & 4 ? box.hz : -box.hz)}.dot(d);
            own = {std::min(own.first, t), std::max(own.second, t)};
        }
        range = {std::max(range.first, own.first), std::min(range.second, own.second)};
    }
    return range;
}

// An extrusion cut by a plane across its direction: the profile's image along the direction, a curve of the profile's kind.
bool oblique(const Surface& a, const Surface& plane, double tol, Result& r) {
    const Curve profile = a.profile();
    const xyz d(a.z());
    const Map map = {xyz(plane.origin()), xyz(plane.x()), xyz(plane.y()), d};
    Image img;
    if (!image(profile, map, img))
        return false;
    const Curve section = lifted(img.curve, map.o, map.e1, map.e2);
    const auto uv = [&](double t) {
        const double u = img.source(t);
        return P2{u, (xyz(section.point_at(t)) - xyz(profile.point_at(u))).dot(d)};
    };
    const auto [t0, t1] = section.domain();
    const P2 p0 = uv(t0), p1 = uv(t1), middle = uv(0.5 * (t0 + t1));
    bool level = true;
    for (double f : {0.21, 0.5, 0.83})
        level = level && std::abs(uv(t0 + f * (t1 - t0))[1] - (p0[1] + f * (p1[1] - p0[1]))) <= 1e-12 * (1.0 + std::abs(p0[1]) + std::abs(p1[1]));
    Curve2d on_a;
    if (level)
        on_a = straight(p0, p1, t0, t1);
    else if (profile.kind() == Kind::Parabola)
        // Both parameters are of degree 2 at most in the section's: one arc through the ends and the middle.
        on_a = Curve2d(NurbsCurve::create_from_parameters({Point(p0[0], p0[1], 0), Point(2.0 * middle[0] - 0.5 * (p0[0] + p1[0]), 2.0 * middle[1] - 0.5 * (p0[1] + p1[1]), 0), Point(p1[0], p1[1], 0)}, {1.0, 1.0, 1.0}, {t0, t1}, {3, 3}, 2));
    else
        on_a = fitted(section, a, uv, tol);
    add(r, a, plane, section, on_a, img.curve, false);
    return true;
}

// An extrusion against a plane, cylinder or extrusion along its direction: seen along it all are curves of one plane, and the
// surfaces meet in lines through the points where those meet.
bool extrusion(const Surface& a, const Surface& b, double tol, Result& r) {
    const xyz d(a.z()), bo(b.origin()), bz(b.z());
    const Curve profile = a.profile(), other = b.kind() == Surface::Kind::Extrusion ? b.profile() : Curve();
    const bool plane = b.kind() == Surface::Kind::Plane;
    if (plane && std::abs(bz.dot(d)) > angular)
        return oblique(a, b, tol, r);
    const Map map = {{0, 0, 0}, xyz(a.x()), xyz(a.y()), d};
    Image ia, ib;
    if ((!plane && bz.cross(d).length() > angular) || !image(profile, map, ia))
        return false;
    if (plane ? !image(Curve::line(b.origin(), Vector(d.cross(bz)), -1.0, 1.0), map, ib)
        : b.kind() == Surface::Kind::Cylinder ? !image(Curve::circle(b.origin(), b.x(), b.y(), b.radius(), b.domain(0).first, b.domain(0).second), map, ib)
        : b.kind() != Surface::Kind::Extrusion || !image(other, map, ib))
        return false;
    const Result2d met = curve_curve(ia.curve, ib.curve, tol);
    if (!met.done || met.coincident)
        return false;
    const std::pair<double, double> range = reach(a, b, d);
    const double sense = bz.dot(d) < 0 ? -1.0 : 1.0;
    for (const Hit2d& hit : met.points) {
        if (!(range.second > range.first))
            break;
        const xyz q = map.e1 * hit.point[0] + map.e2 * hit.point[1];
        const Curve line = Curve::line(Point(q), Vector(d), range.first, range.second);
        const double ua = ia.source(hit.a), ub = ib.source(hit.b);
        const Curve2d on_a = Curve::line(Point(ua, -xyz(profile.point_at(ua)).dot(d), 0), Vector(0, 1, 0), range.first, range.second);
        Image flat;
        if (plane)
            image(line, onto(b), flat);
        else if (b.kind() == Surface::Kind::Cylinder)
            flat.curve = Curve::line(Point(Curve::wrapped(ub, b.domain(0).first, two_pi), (q - bo).dot(bz), 0), Vector(0, sense, 0), range.first, range.second);
        else
            flat.curve = Curve::line(Point(ub, (q - xyz(other.point_at(ub))).dot(bz), 0), Vector(0, sense, 0), range.first, range.second);
        add(r, a, b, line, on_a, flat.curve, hit.tangent);
    }
    return true;
}

}

Result2d curve_curve(const Curve2d& a, const Curve2d& b, double tolerance) {
    if (!a.is_valid() || !b.is_valid())
        return {};
    return a.kind() == Kind::Nurbs || b.kind() == Kind::Nurbs ? numeric(a, b, tolerance) : analytic(a, b, tolerance);
}

Result surface_surface(const Surface& first, const Surface& second, double tolerance) {
    Result r;
    if (first.kind() == Surface::Kind::Nurbs || second.kind() == Surface::Kind::Nurbs || !first.is_valid() || !second.is_valid())
        return r;
    r.coincident = first.coincident(second, tolerance);
    r.done = r.coincident != 0;
    if (r.done)
        return r;
    // The sweep first, a revolution before an extrusion.
    const bool swapped = second.kind() > first.kind() && first.kind() != Surface::Kind::Revolution;
    const Surface &a = swapped ? second : first, &b = swapped ? first : second;
    r.done = a.kind() == Surface::Kind::Revolution ? revolution(a, b, tolerance, r) : a.kind() == Surface::Kind::Extrusion && extrusion(a, b, tolerance, r);
    if (!r.done)
        r.curves.clear(), r.points.clear();
    for (Section& s : r.curves)
        if (swapped)
            std::swap(s.on_a, s.on_b), std::swap(s.deviation_a, s.deviation_b);
    for (intersection_analytic::Hit& h : r.points)
        if (swapped)
            std::swap(h.uv_a, h.uv_b);
    return r;
}

}
