#include "curve_intersection.h"
#include "closest.h"
#include "nurbs_interval.h"
#include "nurbs_memo.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace session_cpp::curve_intersection {
namespace {

constexpr double epsilon = std::numeric_limits<double>::epsilon();
constexpr size_t work_limit = 200000;

bool finite(const Vector& value) {
    return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

bool finite_derivatives(const std::vector<Vector>& values) {
    for (const Vector& value : values)
        if (!finite(value))
            return false;

    return true;
}

std::pair<double, double> unresolved_parameters() {
    const double value = std::numeric_limits<double>::quiet_NaN();
    return {value, value};
}

struct Bounds {
    Point lo;
    Point hi;

    explicit Bounds(const NurbsCurve& curve) : Bounds(curve, 0, curve.cv_count()) {}

    Bounds(const NurbsCurve& curve, int first, int count) : lo(curve.get_cv(first)), hi(lo) {
        for (int i = first + 1; i < first + count; ++i) {
            const Point p = curve.get_cv(i);
            for (int d = 0; d < 3; ++d) {
                lo[d] = std::min(lo[d], p[d]);
                hi[d] = std::max(hi[d], p[d]);
            }
        }
    }

    double size() const {
        return lo.distance(hi);
    }

    double magnitude() const {
        double result = 0.0;
        for (int d = 0; d < 3; ++d)
            result = std::max({result, std::abs(lo[d]), std::abs(hi[d])});

        return result;
    }

    bool overlaps(const Bounds& box, double tolerance) const {
        for (int d = 0; d < 3; ++d)
            if (hi[d] + tolerance < box.lo[d] || box.hi[d] + tolerance < lo[d])
                return false;

        return true;
    }
};

// A negative whole-input proof needs no parameter resolution or root refinement.
bool certified_disjoint(const NurbsCurve& a, const NurbsCurve& b, double tolerance) {
    const auto hull = [](const NurbsCurve& curve, std::array<nurbs_interval::Interval, 3>& box) {
        for (int i = 0; i < curve.cv_count(); ++i) {
            const double* cv = curve.cv(i);
            const double weight = curve.is_rational() ? cv[curve.dimension()] : 1.0;
            if (!(weight > 0) || !std::isfinite(weight))
                return false;
            for (int d = 0; d < 3; ++d) {
                const auto coordinate = nurbs_interval::Interval(d < curve.dimension() ? cv[d] : 0.0)
                    / nurbs_interval::Interval(weight);
                if (!nurbs_interval::finite(coordinate))
                    return false;
                box[d] = i == 0 ? coordinate : nurbs_interval::interval_hull(box[d], coordinate);
            }
        }
        return true;
    };
    std::array<nurbs_interval::Interval, 3> first, second;
    if (!hull(a, first) || !hull(b, second))
        return false;
    for (int d = 0; d < 3; ++d) {
        const auto gap = first[d] - second[d];
        if (nurbs_interval::finite(gap) && (gap.lo > tolerance || gap.hi < -tolerance))
            return true;
    }
    return false;
}

struct Pair {
    NurbsCurve a;
    NurbsCurve b;
    int depth = 0;
};

bool finite_positive(const NurbsCurve& curve) {
    for (int i = 0; i < curve.cv_count(); ++i) {
        const Point p = curve.get_cv(i);
        const double w = curve.weight(i);
        if (!std::isfinite(w) || w <= 0.0)
            return false;

        for (int d = 0; d < 3; ++d)
            if (!std::isfinite(p[d]))
                return false;
    }

    return true;
}

NurbsCurve interval(const NurbsCurve& curve, double a, double b) {
    NurbsCurve result;
    if (trim_piece(curve, a, b, result))
        return result;
    result = curve;
    if ((a > curve.domain_start() || b < curve.domain_end()) && !result.trim(a, b))
        throw std::runtime_error("Curve intersection could not trim a parameter interval");

    return result;
}

double flatness(const NurbsCurve& curve) {
    const Point a = curve.point_at_start();
    const Vector chord = curve.point_at_end() - a;
    const double length2 = chord.dot(chord);
    double distance = 0.0;
    double previous = 0.0;
    for (int i = 0; i < curve.cv_count(); ++i) {
        const Point p = curve.get_cv(i);
        const double t = length2 > 0.0 ? std::clamp((p - a).dot(chord) / length2, 0.0, 1.0) : 0.0;
        if (t + epsilon * 64.0 < previous)
            return Bounds(curve).size();

        previous = t;
        distance = std::max(distance, p.distance(a + chord * t));
    }

    return distance;
}

double closest_parameter(const NurbsCurve& curve, const Point& point) {
    double t = Closest::curve_point(curve, point).first;
    const double lo = curve.domain_start();
    const double hi = curve.domain_end();
    for (int i = 0; i < 32; ++i) {
        std::array<xyz, 3> d{};
        curve.jet(t, 2, d);
        const xyz r = curve.point(t) - point;
        const double den = d[1].dot(d[1]) + r.dot(d[2]);
        if (den <= 0.0)
            break;

        const double next = std::clamp(t - r.dot(d[1]) / den, lo, hi);
        if (next == t)
            break;

        t = next;
    }

    return t;
}

double binomial(int n, int k) {
    double result = 1.0;
    for (int i = 1; i <= std::min(k, n - k); ++i)
        result *= static_cast<double>(n + 1 - i) / i;

    return result;
}

bool same_parameterization(const NurbsCurve& a, const NurbsCurve& b, double tolerance) {
    const int n = a.degree();
    const int m = b.degree();
    if (a.cv_count() != n + 1 || b.cv_count() != m + 1)
        return false;

    double wa = 0.0;
    double wb = 0.0;
    for (int i = 0; i <= n; ++i)
        wa = std::max(wa, a.weight(i));
    for (int j = 0; j <= m; ++j)
        wb = std::max(wb, b.weight(j));

    double minimum = 1.0;
    for (int i = 0; i <= n; ++i)
        for (int j = 0; j <= m; ++j)
            minimum = std::min(minimum, (a.weight(i) / wa) * (b.weight(j) / wb));

    for (int k = 0; k <= n + m; ++k) {
        Vector coefficient(0, 0, 0);
        for (int i = std::max(0, k - m); i <= std::min(n, k); ++i) {
            const int j = k - i;
            const double factor = binomial(n, i) * binomial(m, j) / binomial(n + m, k);
            const double weight = (a.weight(i) / wa) * (b.weight(j) / wb);
            coefficient += (a.get_cv(i) - b.get_cv(j)) * (factor * weight);
        }
        const double distance = coefficient.magnitude();
        if (!std::isfinite(distance) || distance > tolerance * minimum)
            return false;
    }

    return true;
}

bool same_geometry(const NurbsCurve& a, const NurbsCurve& b, double tolerance) {
    if (same_parameterization(a, b, tolerance))
        return true;

    NurbsCurve first = a;
    NurbsCurve second = b;
    const int degree = std::max(a.degree(), b.degree());
    if ((a.degree() < degree && !first.increase_degree(degree)) ||
        (b.degree() < degree && !second.increase_degree(degree)))
        return false;

    const double start = std::log(second.weight(0)) - std::log(first.weight(0));
    const double end = std::log(second.weight(degree)) - std::log(first.weight(degree));
    const double ratio = (end - start) / degree;
    double maximum = -std::numeric_limits<double>::infinity();
    for (int i = 0; i <= degree; ++i)
        maximum = std::max(maximum, std::log(first.weight(i)) + i * ratio);

    for (int i = 0; i <= degree; ++i) {
        const Point point = first.get_cv(i);
        const double weight = std::exp(std::log(first.weight(i)) + i * ratio - maximum);
        if (weight <= 0.0 || !first.set_cv_4d(
            i,
            point[0] * weight,
            point[1] * weight,
            point[2] * weight,
            weight
        ))
            return false;
    }

    return same_parameterization(first, second, tolerance);
}

/// Farther than the tolerance and the rounding of a point on the curve from the control box that holds it.
bool beyond(const Bounds& box, const Point& p, double reach) {
    for (int d = 0; d < 3; ++d)
        if (p[d] < box.lo[d] - reach || p[d] > box.hi[d] + reach)
            return true;
    return false;
}

std::vector<std::pair<double, double>> endpoint_matches(const Pair& pair, double tolerance) {
    std::vector<std::pair<double, double>> matches;
    const Bounds first(pair.a), second(pair.b);
    const double reach = tolerance + epsilon * 1024.0 * std::max(first.magnitude(), second.magnitude());
    for (const double ta : {pair.a.domain_start(), pair.a.domain_end()}) {
        const Point p = pair.a.point_at(ta);
        if (beyond(second, p, reach))
            continue;
        const double tb = closest_parameter(pair.b, p);
        if (xyz(p).distance(pair.b.point(tb)) <= tolerance)
            matches.emplace_back(ta, tb);
    }
    for (const double tb : {pair.b.domain_start(), pair.b.domain_end()}) {
        const Point p = pair.b.point_at(tb);
        if (beyond(first, p, reach))
            continue;
        const double ta = closest_parameter(pair.a, p);
        if (xyz(p).distance(pair.a.point(ta)) <= tolerance)
            matches.emplace_back(ta, tb);
    }
    std::sort(matches.begin(), matches.end());

    return matches;
}

bool overlap(const Pair& pair, double tolerance, Overlap& result) {
    // The pieces end to end first: a closest point search can miss an exact end by more than this proof.
    if (pair.a.point_at_start().distance(pair.b.point_at_start()) <= tolerance &&
        pair.a.point_at_end().distance(pair.b.point_at_end()) <= tolerance && same_geometry(pair.a, pair.b, tolerance)) {
        result = {pair.a.domain_start(), pair.a.domain_end(), pair.b.domain_start(), pair.b.domain_end()};
        return true;
    }

    const std::vector<std::pair<double, double>> matches = endpoint_matches(pair, tolerance);
    for (size_t i = 0; i < matches.size(); ++i)
        for (size_t j = matches.size(); j-- > i + 1;) {
            const double a0 = matches[i].first;
            const double a1 = matches[j].first;
            const double b0 = matches[i].second;
            const double b1 = matches[j].second;
            if (a1 - a0 <= epsilon * 32.0 || std::abs(b1 - b0) <= epsilon * 32.0)
                continue;

            const NurbsCurve a = interval(pair.a, a0, a1);
            NurbsCurve b = interval(pair.b, std::min(b0, b1), std::max(b0, b1));
            if (b1 < b0 && !b.reverse())
                continue;

            const bool linear = flatness(a) <= tolerance && flatness(b) <= tolerance;
            if (linear || same_geometry(a, b, tolerance)) {
                result = {a0, a1, b0, b1};
                return true;
            }
        }

    return false;
}

std::vector<double> split_parameters(const NurbsCurve& curve, double a, double b) {
    const double lo = curve.domain_start();
    const double hi = curve.domain_end();
    std::vector<double> result{lo, hi};
    for (const double t : {a, b})
        if (t > lo + epsilon * 32.0 && t < hi - epsilon * 32.0)
            result.push_back(t);

    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

void remaining_pairs(const Pair& pair, const Overlap& shared, std::vector<Pair>& work) {
    const std::vector<double> a = split_parameters(pair.a, shared.a0, shared.a1);
    const std::vector<double> b = split_parameters(pair.b, shared.b0, shared.b1);
    const double b0 = std::min(shared.b0, shared.b1);
    const double b1 = std::max(shared.b0, shared.b1);
    for (size_t i = 1; i < a.size(); ++i)
        for (size_t j = 1; j < b.size(); ++j) {
            if (a[i - 1] >= shared.a0 && a[i] <= shared.a1 && b[j - 1] >= b0 && b[j] <= b1)
                continue;

            work.push_back({interval(pair.a, a[i - 1], a[i]), interval(pair.b, b[j - 1], b[j])});
        }
}

std::pair<double, double> refine(const Pair& pair, double ta, double tb) {
    for (int i = 0; i < 80; ++i) {
        const std::vector<Vector> a = pair.a.evaluate(ta, 2);
        const std::vector<Vector> b = pair.b.evaluate(tb, 2);
        if (!finite_derivatives(a) || !finite_derivatives(b))
            return unresolved_parameters();

        const Vector r = a[0] - b[0];
        const double uu = a[1].dot(a[1]);
        const double vv = b[1].dot(b[1]);
        const double ru = r.dot(a[2]);
        const double rv = r.dot(b[2]);
        const double aa = uu + ru;
        const double ab = -a[1].dot(b[1]);
        const double bb = vv - rv;
        const double ar = -a[1].dot(r);
        const double br = b[1].dot(r);
        const double det = a[1].cross(b[1]).magnitude_squared() + ru * vv - rv * uu - ru * rv;
        if (!std::isfinite(det) || !std::isfinite(ar) || !std::isfinite(br))
            return unresolved_parameters();
        if (aa <= 0.0 || bb <= 0.0 || det <= 0.0)
            break;

        const double da = (bb * ar - ab * br) / det;
        const double db = (aa * br - ab * ar) / det;
        if (!std::isfinite(da) || !std::isfinite(db))
            return unresolved_parameters();
        double step = 1.0;
        bool accepted = false;
        for (int k = 0; k < 16; ++k) {
            const double na = std::clamp(ta + da * step, pair.a.domain_start(), pair.a.domain_end());
            const double nb = std::clamp(tb + db * step, pair.b.domain_start(), pair.b.domain_end());
            if (pair.a.point_at(na).distance(pair.b.point_at(nb)) <= r.magnitude()) {
                accepted = na != ta || nb != tb;
                ta = na;
                tb = nb;
                break;
            }
            step *= 0.5;
        }
        if (!accepted)
            break;
    }

    return {ta, tb};
}

std::pair<double, double> refine_tangent(const Pair& pair, double ta, double tb) {
    const double scale = Bounds(pair.a).size() * Bounds(pair.b).size();
    for (int i = 0; i < 16; ++i) {
        const std::vector<Vector> a = pair.a.evaluate(ta, 2);
        const std::vector<Vector> b = pair.b.evaluate(tb, 2);
        if (!finite_derivatives(a) || !finite_derivatives(b))
            return unresolved_parameters();

        const double uu = a[1].dot(a[1]);
        const double vv = b[1].dot(b[1]);
        const double divisor = std::sqrt(uu * vv);
        if (!std::isfinite(divisor))
            return unresolved_parameters();
        if (divisor == 0.0)
            break;

        const Vector cross = a[1].cross(b[1]) / divisor;
        if (cross.magnitude() > 1e-4)
            break;

        const Vector ca = a[2].cross(b[1]) / divisor - cross * (a[1].dot(a[2]) / uu);
        const Vector cb = a[1].cross(b[2]) / divisor - cross * (b[1].dot(b[2]) / vv);
        const Vector r = a[0] - b[0];
        const double aa = uu + scale * ca.dot(ca);
        const double ab = -a[1].dot(b[1]) + scale * ca.dot(cb);
        const double bb = vv + scale * cb.dot(cb);
        const double ar = -a[1].dot(r) - scale * ca.dot(cross);
        const double br = b[1].dot(r) - scale * cb.dot(cross);
        const double det = aa * bb - ab * ab;
        if (!std::isfinite(det) || !std::isfinite(ar) || !std::isfinite(br))
            return unresolved_parameters();
        if (det <= epsilon * aa * bb)
            break;

        const double na = std::clamp(ta + (bb * ar - ab * br) / det, pair.a.domain_start(), pair.a.domain_end());
        const double nb = std::clamp(tb + (aa * br - ab * ar) / det, pair.b.domain_start(), pair.b.domain_end());
        if (na == ta && nb == tb)
            break;

        ta = na;
        tb = nb;
    }

    return {ta, tb};
}

double parameter_tolerance(const NurbsCurve& curve, double t, double tolerance) {
    const std::vector<Vector> d = curve.evaluate(t, 1);
    const double speed = d[1].magnitude();
    return std::max(epsilon * 128.0, tolerance / std::max(speed, tolerance));
}

bool add_hit(const Pair& pair, const Pair& leaf, double tolerance, Result& result) {
    double ta = 0.0;
    double tb = 0.0;
    double distance = 0.0;
    std::tie(ta, tb, distance) = Closest::curve_curve(leaf.a, leaf.b);
    if (!std::isfinite(ta) || !std::isfinite(tb) || !std::isfinite(distance))
        return false;

    std::tie(ta, tb) = refine(pair, ta, tb);
    if (!std::isfinite(ta) || !std::isfinite(tb))
        return false;
    const std::pair<double, double> tangent = refine_tangent(pair, ta, tb);
    if (!std::isfinite(tangent.first) || !std::isfinite(tangent.second))
        return false;
    const double roundoff = epsilon * 64.0 * std::max(Bounds(pair.a).magnitude(), Bounds(pair.b).magnitude());
    if (pair.a.point_at(tangent.first).distance(pair.b.point_at(tangent.second)) <= std::min(tolerance, roundoff)) {
        ta = tangent.first;
        tb = tangent.second;
    }
    // Curves that run on through a shared end stay within tolerance a little before it: the hit is that end.
    for (const double ea : {pair.a.domain_start(), pair.a.domain_end()})
        for (const double eb : {pair.b.domain_start(), pair.b.domain_end()})
            if (pair.a.point_at(ea).distance(pair.b.point_at(eb)) <= tolerance &&
                pair.a.point_at(ta).distance(pair.a.point_at(ea)) <= tolerance * 4.0 &&
                pair.b.point_at(tb).distance(pair.b.point_at(eb)) <= tolerance * 4.0) {
                ta = ea;
                tb = eb;
            }
    // A hit at a curve's own end to roundoff is that end: vertices are told apart by exact parameter.
    for (const double end : {pair.a.domain_start(), pair.a.domain_end()})
        if (pair.a.point_at(ta).distance(pair.a.point_at(end)) <= roundoff)
            ta = end;
    for (const double end : {pair.b.domain_start(), pair.b.domain_end()})
        if (pair.b.point_at(tb).distance(pair.b.point_at(end)) <= roundoff)
            tb = end;
    const Point a = pair.a.point_at(ta);
    const Point b = pair.b.point_at(tb);
    const double gap = a.distance(b);
    if (!std::isfinite(gap))
        return false;
    if (gap > tolerance)
        return true;

    const double ea = parameter_tolerance(pair.a, ta, tolerance * 2.0);
    const double eb = parameter_tolerance(pair.b, tb, tolerance * 2.0);
    for (const Hit& hit : result.points)
        if (std::abs(ta - hit.a) <= ea && std::abs(tb - hit.b) <= eb)
            return true;

    const Vector u = pair.a.evaluate(ta, 1)[1];
    const Vector v = pair.b.evaluate(tb, 1)[1];
    if (!finite(u) || !finite(v))
        return false;
    const bool touching = u.cross(v).magnitude() <= 1e-6 * u.magnitude() * v.magnitude();
    result.points.push_back({a + (b - a) * 0.5, ta, tb, touching});
    return true;
}

bool subdivide(const Pair& pair, bool first, std::vector<Pair>& work) {
    const NurbsCurve& curve = first ? pair.a : pair.b;
    const double mid = curve.domain_middle();
    if (mid <= curve.domain_start() || mid >= curve.domain_end())
        return false;

    NurbsCurve left;
    NurbsCurve right;
    if (!curve.split(mid, left, right))
        return false;

    if (first) {
        work.push_back({left, pair.b, pair.depth + 1});
        work.push_back({right, pair.b, pair.depth + 1});
    } else {
        work.push_back({pair.a, left, pair.depth + 1});
        work.push_back({pair.a, right, pair.depth + 1});
    }
    return true;
}

bool isolated(const Pair& pair, double tolerance, size_t& budget, Result& result) {
    std::vector<Pair> work{pair};
    while (!work.empty()) {
        if (budget++ >= work_limit)
            return false;

        const Pair leaf = std::move(work.back());
        work.pop_back();
        const Bounds a(leaf.a);
        const Bounds b(leaf.b);
        if (!a.overlaps(b, tolerance))
            continue;

        const double fa = flatness(leaf.a);
        const double fb = flatness(leaf.b);
        if (fa <= tolerance * 0.05 && fb <= tolerance * 0.05) {
            if (!add_hit(pair, leaf, tolerance, result)) {
                result.failure = "Curve intersection encountered a nonfinite numerical result";
                return false;
            }
            continue;
        }
        // The longer piece: a flat line left whole keeps a box that holds every curved piece under it.
        if (leaf.depth >= 96 || !subdivide(leaf, a.size() >= b.size(), work))
            return false;
    }

    return true;
}

nurbs_interval::Interval binomial_bound(int n, int k) {
    nurbs_interval::Interval result(1.0);
    for (int i = 1; i <= std::min(k, n - k); ++i)
        result = result * nurbs_interval::Interval(n + 1 - i) / nurbs_interval::Interval(i);
    return result;
}

/// Bound the whole normalized correspondence, retaining a common range rather than one point.
bool certified_parameter_range(const Pair& pair, double tolerance, bool reverse) {
    using nurbs_interval::Interval;
    const Point point = pair.a.get_cv(0);
    const std::array<double, 3> origin = {point[0], point[1], point[2]};
    const auto a = nurbs_interval::curve_controls(pair.a,
        {pair.a.domain_start(), pair.a.domain_end()}, origin);
    auto b = nurbs_interval::curve_controls(pair.b,
        {pair.b.domain_start(), pair.b.domain_end()}, origin);
    if (a.empty() || b.empty())
        return false;
    if (reverse)
        std::reverse(b.begin(), b.end());
    double denominator = std::numeric_limits<double>::infinity();
    for (const auto& first : a)
        for (const auto& second : b) {
            if (!(first[3].lo > 0.0 && second[3].lo > 0.0))
                return false;
            const Interval product = first[3] * second[3];
            if (!nurbs_interval::finite(product) || !(product.lo > 0.0))
                return false;
            denominator = std::min(denominator, product.lo);
        }
    const int n = static_cast<int>(a.size()) - 1;
    const int m = static_cast<int>(b.size()) - 1;
    std::array<Interval, 3> bounds;
    for (int k = 0; k <= n + m; ++k) {
        std::array<Interval, 3> coefficient;
        for (int i = std::max(0, k - m); i <= std::min(n, k); ++i) {
            const int j = k - i;
            const Interval factor = binomial_bound(n, i) * binomial_bound(m, j) / binomial_bound(n + m, k);
            for (int d = 0; d < 3; ++d)
                coefficient[d] = coefficient[d] + factor * (a[i][d] * b[j][3] - b[j][d] * a[i][3]);
        }
        for (int d = 0; d < 3; ++d) {
            if (!nurbs_interval::finite(coefficient[d]))
                return false;
            bounds[d] = k == 0 ? coefficient[d] : nurbs_interval::interval_hull(bounds[d], coefficient[d]);
        }
    }
    Interval squared;
    for (const auto coordinate : bounds) {
        const Interval distance(nurbs_interval::magnitude(coordinate));
        squared = squared + distance * distance;
    }
    const Interval limit = Interval(tolerance) * Interval(tolerance) * Interval(denominator) * Interval(denominator);
    return nurbs_interval::finite(squared) && nurbs_interval::finite(limit) && squared.hi <= limit.lo;
}

bool certified_clipped_range(const Pair& pair, double tolerance, Overlap& shared) {
    const auto matches = endpoint_matches(pair, tolerance);
    for (size_t i = 0; i < matches.size(); ++i)
        for (size_t j = matches.size(); j-- > i + 1;) {
            const double a0 = matches[i].first, a1 = matches[j].first;
            const double b0 = matches[i].second, b1 = matches[j].second;
            if (!std::isfinite(a0) || !std::isfinite(a1) || !std::isfinite(b0) || !std::isfinite(b1)
                || a0 < pair.a.domain_start() || a1 > pair.a.domain_end()
                || std::min(b0, b1) < pair.b.domain_start() || std::max(b0, b1) > pair.b.domain_end()
                || a1 - a0 <= epsilon * 32.0 || std::abs(b1 - b0) <= epsilon * 32.0)
                continue;
            const Pair clipped{interval(pair.a, a0, a1), interval(pair.b, std::min(b0, b1), std::max(b0, b1))};
            if (certified_parameter_range(clipped, tolerance, b1 < b0)) {
                shared = {clipped.a.domain_start(), clipped.a.domain_end(),
                    b1 < b0 ? clipped.b.domain_end() : clipped.b.domain_start(),
                    b1 < b0 ? clipped.b.domain_start() : clipped.b.domain_end()};
                return true;
            }
        }
    return false;
}

bool process(const Pair& pair, double tolerance, size_t& budget, Result& result) {
    const Bounds a(pair.a);
    const Bounds b(pair.b);
    if (!a.overlaps(b, tolerance))
        return true;

    const double proof = std::max(tolerance * 1e-5, epsilon * 32.0 * std::max(a.magnitude(), b.magnitude()));
    Overlap shared{};
    if (!overlap(pair, proof, shared)) {
        // Certified within tolerance along their whole length, two curves overlap however curved they are.
        // Ends of the two curves apart by more than the tolerance fail the proof at its first or last coefficient.
        const double reach = tolerance + epsilon * 1024.0 * std::max(a.magnitude(), b.magnitude());
        const auto ends_meet = [&](bool reverse) {
            return !pair.a.is_clamped() || !pair.b.is_clamped() ||
                (pair.a.get_cv(0).distance(pair.b.get_cv(reverse ? pair.b.cv_count() - 1 : 0)) <= reach &&
                 pair.a.get_cv(pair.a.cv_count() - 1).distance(pair.b.get_cv(reverse ? 0 : pair.b.cv_count() - 1)) <= reach);
        };
        for (bool reverse : {false, true})
            if (ends_meet(reverse) && certified_parameter_range(pair, tolerance, reverse)) {
                result.overlaps.push_back({pair.a.domain_start(), pair.a.domain_end(),
                    reverse ? pair.b.domain_end() : pair.b.domain_start(),
                    reverse ? pair.b.domain_start() : pair.b.domain_end()});
                return true;
            }
        const bool flat = flatness(pair.a) <= tolerance * 0.05 && flatness(pair.b) <= tolerance * 0.05;
        if (!flat || !certified_clipped_range(pair, tolerance, shared))
            return isolated(pair, tolerance, budget, result);
    }

    result.overlaps.push_back(shared);
    std::vector<Pair> work;
    remaining_pairs(pair, shared, work);
    for (const Pair& piece : work)
        if (!isolated(piece, tolerance, budget, result))
            return false;

    return true;
}

struct SpanBounds {
    nurbs_interval::CurveBounds box;
    bool valid;
    bool known;
};

/// The box of the controls active on each span, which holds the span.
std::vector<Bounds> span_hulls(const NurbsCurve& curve, const std::vector<double>& knots) {
    std::vector<Bounds> result;
    for (size_t i = 1; i < knots.size(); ++i)
        result.emplace_back(curve, nurbsknot::find_span(curve.order(), curve.cv_count(), curve.m_nurbsknot,
            std::midpoint(knots[i - 1], knots[i])), curve.order());
    return result;
}

/// The outward bounds of one span, taken when first asked for.
const SpanBounds& span_bounds(std::vector<SpanBounds>& spans, const NurbsCurve& curve, const std::vector<double>& knots, size_t i) {
    SpanBounds& span = spans[i - 1];
    if (!span.known)
        span = {{}, false, true}, span.valid = nurbs_interval::curve_bounds(curve, {knots[i - 1], knots[i]}, {0, 0, 0}, span.box);
    return span;
}

bool separated(const SpanBounds& a, const SpanBounds& b, double tolerance) {
    if (!a.valid || !b.valid)
        return false;
    for (int d = 0; d < 3; ++d) {
        const auto gap = a.box.point[d] - b.box.point[d];
        if (gap.lo > tolerance || gap.hi < -tolerance)
            return true;
    }
    return false;
}

bool hit_less(const Hit& a, const Hit& b) {
    return a.a < b.a || (a.a == b.a && a.b < b.b);
}

bool overlap_less(const Overlap& a, const Overlap& b) {
    return a.a0 < b.a0 || (a.a0 == b.a0 && a.b0 < b.b0);
}

void merge_overlaps(Result& result) {
    std::sort(result.overlaps.begin(), result.overlaps.end(), overlap_less);
    std::vector<Overlap> merged;
    for (const Overlap& overlap : result.overlaps) {
        if (!merged.empty()) {
            Overlap& last = merged.back();
            const bool aligned = (last.b1 - last.b0) * (overlap.b1 - overlap.b0) > 0.0;
            if (aligned && std::abs(last.a1 - overlap.a0) <= epsilon * 256.0 &&
                std::abs(last.b1 - overlap.b0) <= epsilon * 256.0) {
                last.a1 = overlap.a1;
                last.b1 = overlap.b1;
                continue;
            }
        }
        merged.push_back(overlap);
    }
    result.overlaps = std::move(merged);
}

void map_result(const NurbsCurve& a, const NurbsCurve& b, Result& result) {
    const double a0 = a.domain_start();
    const double b0 = b.domain_start();
    const double a1 = a.domain_end();
    const double b1 = b.domain_end();
    const double da = a1 - a0;
    const double db = b1 - b0;
    std::sort(result.points.begin(), result.points.end(), hit_less);
    merge_overlaps(result);
    std::vector<Hit> points;
    for (Hit hit : result.points) {
        bool covered = false;
        for (const Overlap& shared : result.overlaps)
            if (hit.a >= shared.a0 - epsilon * 256.0 && hit.a <= shared.a1 + epsilon * 256.0 &&
                hit.b >= std::min(shared.b0, shared.b1) - epsilon * 256.0 &&
                hit.b <= std::max(shared.b0, shared.b1) + epsilon * 256.0)
                covered = true;

        if (covered)
            continue;

        hit.a = std::clamp(a0 + hit.a * da, a0, a1);
        hit.b = std::clamp(b0 + hit.b * db, b0, b1);
        points.push_back(hit);
    }
    result.points = std::move(points);
    for (Overlap& shared : result.overlaps) {
        shared.a0 = std::clamp(a0 + shared.a0 * da, a0, a1);
        shared.a1 = std::clamp(a0 + shared.a1 * da, a0, a1);
        shared.b0 = std::clamp(b0 + shared.b0 * db, b0, b1);
        shared.b1 = std::clamp(b0 + shared.b1 * db, b0, b1);
    }
}

Result intersect_once(const NurbsCurve& a, const NurbsCurve& b, double tolerance) {
    if (!std::isfinite(tolerance) || tolerance <= 0.0 || !a.is_valid() || !b.is_valid())
        throw std::invalid_argument("Curve intersection requires valid curves and a positive finite tolerance");

    Result result;
    if (!finite_positive(a) || !finite_positive(b)) {
        result.complete = false;
        result.failure = "Curve intersection requires finite controls and positive rational weights";
        return result;
    }
    const Bounds bounds_a(a);
    const Bounds bounds_b(b);
    if (bounds_a.size() == 0.0 || bounds_b.size() == 0.0 ||
        epsilon * 32.0 * std::max(bounds_a.magnitude(), bounds_b.magnitude()) > tolerance) {
        if (bounds_a.size() > 0.0 && bounds_b.size() > 0.0 && certified_disjoint(a, b, tolerance))
            return result;
        result.complete = false;
        result.failure = "Curve intersection has a constant input or tolerance below coordinate resolution";
        if (const char* path = std::getenv("SESSION_CC_GUARD_FAILURE"); path && *path) {
            try {
                std::ofstream output(path, std::ios::app);
                if (!output)
                    throw std::runtime_error("cannot open capture file");
                const nlohmann::ordered_json saved = {{"stage", "curve_intersection_input_guard"},
                    {"failure", result.failure}, {"uv_tolerance", tolerance},
                    {"section_pcurve", a.jsondump()}, {"boundary_pcurve", b.jsondump()},
                    {"first_size", bounds_a.size()}, {"second_size", bounds_b.size()},
                    {"first_magnitude", bounds_a.magnitude()}, {"second_magnitude", bounds_b.magnitude()},
                    {"coordinate_resolution", epsilon * 32.0 * std::max(bounds_a.magnitude(), bounds_b.magnitude())}};
                output << saved.dump() << '\n';
                if (!output)
                    throw std::runtime_error("cannot write capture record");
                output.flush();
                if (!output)
                    throw std::runtime_error("cannot flush capture file");
                output.close();
                if (output.fail())
                    throw std::runtime_error("cannot close capture file");
            } catch (const std::exception& error) {
                std::cerr << "Curve intersection guard capture failed: " << error.what() << '\n';
            } catch (...) {
                std::cerr << "Curve intersection guard capture failed: unknown error\n";
            }
        }
        return result;
    }
    NurbsCurve first = a;
    NurbsCurve second = b;
    if (!first.set_domain(0.0, 1.0) || !second.set_domain(0.0, 1.0))
        throw std::runtime_error("Curve intersection could not normalize parameter domains");

    const std::vector<double> av = first.get_span_vector();
    const std::vector<double> bv = second.get_span_vector();
    // Two spans whose control boxes are apart by the tolerance and more than the rounding of the outward bounds have separated outward bounds: those are left untaken.
    const auto ha = span_hulls(first, av), hb = span_hulls(second, bv);
    const double reach = tolerance + epsilon * 1048576.0 * std::max(bounds_a.magnitude(), bounds_b.magnitude());
    std::vector<SpanBounds> ab(ha.size(), SpanBounds{}), bb(hb.size(), SpanBounds{});
    size_t budget = 0;
    for (size_t i = 1; i < av.size() && result.complete; ++i)
        for (size_t j = 1; j < bv.size() && result.complete; ++j) {
            if (!ha[i - 1].overlaps(hb[j - 1], reach) ||
                separated(span_bounds(ab, first, av, i), span_bounds(bb, second, bv, j), tolerance))
                continue;
            const Pair pair{interval(first, av[i - 1], av[i]), interval(second, bv[j - 1], bv[j])};
            result.complete = budget++ < work_limit && process(pair, tolerance, budget, result);
        }

    if (!result.complete && result.failure.empty())
        result.failure = "Curve intersection exceeded its subdivision or floating-point resolution limit";

    map_result(a, b, result);
    return result;
}

}

namespace {

using Triple = std::array<double, 3>;

struct Segment {
    Triple p0;
    Triple p1;
    double t0;
    double t1;
};

Triple difference(const Triple& a, const Triple& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

double product(const Triple& a, const Triple& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Triple normal(const Triple& a, const Triple& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

Triple along(const Segment& s, double t) {
    return {std::lerp(s.p0[0], s.p1[0], t), std::lerp(s.p0[1], s.p1[1], t), std::lerp(s.p0[2], s.p1[2], t)};
}

double apart(const Triple& a, const Triple& b) {
    const Triple d = difference(a, b);
    return std::sqrt(product(d, d));
}

// A valid curve of two control points on one span, without weights or with equal ones.
bool straight(const NurbsCurve& curve, Segment& s) {
    if (curve.m_order != 2 || curve.m_cv_count != 2 || curve.m_dim < 2 || curve.m_dim > 3 ||
        curve.m_nurbsknot.size() != 2 || curve.m_cv_stride < curve.cv_size() ||
        static_cast<int>(curve.m_cv.size()) < curve.m_cv_stride + curve.cv_size())
        return false;
    s.t0 = curve.m_nurbsknot[0];
    s.t1 = curve.m_nurbsknot[1];
    if (!std::isfinite(s.t0) || !std::isfinite(s.t1) || !(s.t0 < s.t1))
        return false;
    for (const double value : curve.m_cv)
        if (!std::isfinite(value))
            return false;
    const double* a = &curve.m_cv[0];
    const double* b = &curve.m_cv[curve.m_cv_stride];
    const double weight = curve.m_is_rat ? a[curve.m_dim] : 1.0;
    if (curve.m_is_rat && (!(weight > 0.0) || b[curve.m_dim] != weight))
        return false;
    for (int d = 0; d < 3; ++d) {
        s.p0[d] = d < curve.m_dim ? a[d] / weight : 0.0;
        s.p1[d] = d < curve.m_dim ? b[d] / weight : 0.0;
    }
    return true;
}

// Two straight pieces in closed form: the answer of the general routine wherever each of its decisions is clear of its threshold by more than rounding, false to leave the pair to it.
bool straight_pair(const NurbsCurve& first, const NurbsCurve& second, double tolerance, Result& result) {
    Segment a, b;
    if (!std::isfinite(tolerance) || tolerance <= 0.0 || !straight(first, a) || !straight(second, b))
        return false;
    double magnitude = 0.0;
    for (const Triple* point : {&a.p0, &a.p1, &b.p0, &b.p1})
        for (const double value : *point)
            magnitude = std::max(magnitude, std::abs(value));
    const double band = epsilon * 16.0 * magnitude;
    const Triple u = difference(a.p1, a.p0), v = difference(b.p1, b.p0);
    const double uu = product(u, u), vv = product(v, v);
    const double la = std::sqrt(uu), lb = std::sqrt(vv);
    if (tolerance < band * 16.0 || !(std::min(la, lb) >= tolerance * 64.0))
        return false;
    const double limit = tolerance + band;
    const auto parameter = [](const Segment& s, double t) { return std::clamp(s.t0 + t * (s.t1 - s.t0), s.t0, s.t1); };
    const auto foot = [](const Segment& s, const Triple& d, double dd, const Triple& p, double& t) {
        t = std::clamp(product(difference(p, s.p0), d) / dd, 0.0, 1.0);
        return apart(p, along(s, t));
    };
    // The four ends on the other piece, as (parameter on a, parameter on b) and the distance.
    std::array<std::pair<double, double>, 4> ends = {{{0.0, 0.0}, {1.0, 0.0}, {0.0, 0.0}, {0.0, 1.0}}};
    const std::array<double, 4> off = {foot(b, v, vv, a.p0, ends[0].second), foot(b, v, vv, a.p1, ends[1].second),
        foot(a, u, uu, b.p0, ends[2].first), foot(a, u, uu, b.p1, ends[3].first)};
    const double nearest = *std::min_element(off.begin(), off.end());
    const Triple c = normal(u, v), w = difference(b.p0, a.p0);
    const double cc = product(c, c);
    const double sine = std::sqrt(cc / (uu * vv));
    const auto hit = [&](double ta, double tb, bool tangent) {
        const Triple pa = along(a, ta), pb = along(b, tb);
        result.points.push_back({Point(pa[0] + (pb[0] - pa[0]) * 0.5, pa[1] + (pb[1] - pa[1]) * 0.5,
            pa[2] + (pb[2] - pa[2]) * 0.5), parameter(a, ta), parameter(b, tb), tangent});
    };
    if (sine >= 1e-3) {
        const double s = product(normal(w, v), c) / cc, t = product(normal(w, u), c) / cc;
        const bool inside = s >= 0.0 && s <= 1.0 && t >= 0.0 && t <= 1.0;
        if ((inside ? std::min(nearest, std::abs(product(w, c)) / std::sqrt(cc)) : nearest) > limit)
            return true;
        // Two ends on the other piece more than a rounding apart on both start the overlap search.
        for (int i = 0; i < 4; ++i)
            for (int j = i + 1; j < 4; ++j)
                if (off[i] <= limit && off[j] <= limit && std::abs(ends[i].first - ends[j].first) > epsilon * 16.0 &&
                    std::abs(ends[i].second - ends[j].second) > epsilon * 16.0)
                    return false;
        // Past an end the general search stops at each parameter clamped on its own, when its Newton step is taken at all.
        if (!inside && cc < 1e-13)
            return false;
        double ta = std::clamp(s, 0.0, 1.0), tb = std::clamp(t, 0.0, 1.0);
        bool clear = true;
        const auto within = [&clear](double value, double threshold, double slack) {
            clear = clear && std::abs(value - threshold) > slack;
            return value <= threshold;
        };
        for (const double ea : {0.0, 1.0})
            for (const double eb : {0.0, 1.0})
                if (within(apart(along(a, ea), along(b, eb)), tolerance, band) &&
                    within(apart(along(a, ta), along(a, ea)), tolerance * 4.0, band) &&
                    within(apart(along(b, tb), along(b, eb)), tolerance * 4.0, band)) {
                    ta = ea;
                    tb = eb;
                }
        for (const double end : {0.0, 1.0})
            if (within(apart(along(a, ta), along(a, end)), band * 4.0, band * 0.25))
                ta = end;
        for (const double end : {0.0, 1.0})
            if (within(apart(along(b, tb), along(b, end)), band * 4.0, band * 0.25))
                tb = end;
        const bool met = within(apart(along(a, ta), along(b, tb)), tolerance, band);
        if (!clear)
            return false;
        if (met)
            hit(ta, tb, false);
        return true;
    }
    if (nearest - std::min(la, lb) * sine > limit)
        return true;
    if (sine * std::max(la, lb) > band)
        return false;
    // Parallel to rounding: an end lies on the other piece to rounding or clear of it.
    int count = 0;
    for (int i = 0; i < 4; ++i) {
        if (off[i] > band && off[i] <= limit)
            return false;
        if (off[i] <= band)
            ends[count++] = ends[i];
    }
    if (count == 0)
        return false;
    std::fill(ends.begin() + count, ends.end(), std::pair<double, double>{2.0, 2.0});
    std::sort(ends.begin(), ends.end());
    for (int i = 0; i < count; ++i)
        for (int j = count; j-- > i + 1;) {
            const double a0 = ends[i].first, a1 = ends[j].first, b0 = ends[i].second, b1 = ends[j].second;
            if (a1 - a0 <= epsilon * 16.0 || std::abs(b1 - b0) <= epsilon * 16.0)
                continue;
            for (const double t : {a0, a1, b0, b1})
                if (t != 0.0 && t != 1.0 && (t < 1e-9 || t > 1.0 - 1e-9))
                    return false;
            if (a1 - a0 < 1e-9 || std::abs(b1 - b0) < 1e-9 || (a1 - a0) * la < tolerance * 64.0)
                return false;
            result.overlaps.push_back({parameter(a, a0), parameter(a, a1), parameter(b, b0), parameter(b, b1)});
            return true;
        }
    // Two pieces in a row share one end.
    const double ea = std::round(ends[0].first), eb = std::round(ends[0].second);
    for (int i = 0; i < count; ++i)
        if (std::abs(ends[i].first - ea) > epsilon * 16.0 || std::abs(ends[i].second - eb) > epsilon * 16.0)
            return false;
    hit(ea, eb, true);
    return true;
}

}

// NurbsCurve::trim inserts each end as a knot and keeps the range between: every step reads the controls and knots of the span of that end and of no other. On a copy of those spans, with one more span either side so that neither end is an end of the copy, it takes the same steps on the same numbers.
bool trim_piece(const NurbsCurve& curve, double first, double last, NurbsCurve& piece) {
    const int p = curve.degree(), n = curve.cv_count(), size = curve.cv_size();
    const std::vector<double>& knots = curve.m_nurbsknot;
    if (n <= curve.order() || !curve.is_valid() || !(first < last) || knots.front() != knots[p - 1] || knots[n - 1] != knots.back() ||
        first < knots.front() || last > knots.back())
        return false;
    // Trim takes a parameter this near a knot for the knot, and counts knots this near each other as one: so the two ends, once knots.
    const double near = 4.0 * epsilon * std::max({std::abs(knots.front()), std::abs(knots.back()), knots.back() - knots.front()});
    if (!(last - first > near))
        return false;
    for (const double t : {first, last}) {
        const auto above = std::lower_bound(knots.begin(), knots.end(), t);
        if (*above != t && (*above - t <= near || t - *(above - 1) <= near))
            return false;
    }
    int lo = static_cast<int>(std::upper_bound(knots.begin(), knots.end(), first) - knots.begin());
    int hi = static_cast<int>(std::lower_bound(knots.begin(), knots.end(), last) - knots.begin());
    if (lo < p || hi > n - 1 || lo > hi)
        return false;
    if (lo > p)
        do --lo; while (lo > p && !(knots[lo - 1] < knots[lo]));
    if (hi < n - 1)
        do ++hi; while (hi < n - 1 && !(knots[hi - 1] < knots[hi]));
    const int count = hi - lo + p + 1;
    if (count == n)
        return false;
    for (int i = lo - p + 1; i < hi + p; ++i)
        if (knots[i] > knots[i - 1] && knots[i] - knots[i - 1] <= near)
            return false;
    // Neither the curve nor the copy is one whose last controls repeat its first: trim treats that as a closed loop of knots.
    const auto open = [&](int a, int b) {
        for (int d = 0; d < size; ++d)
            if (std::abs(curve.cv(a)[d] - curve.cv(b)[d]) > 1e-9)
                return true;
        return false;
    };
    if (p > 1 && (!open(0, n - p) || (count >= 2 * p && !open(lo - p, hi - p + 1))))
        return false;
    piece = curve;
    piece.m_cv_count = count;
    piece.m_nurbsknot.assign(knots.begin() + lo - p, knots.begin() + hi + p);
    piece.m_cv.assign(curve.m_cv.begin() + (lo - p) * curve.m_cv_stride,
        curve.m_cv.begin() + std::min<size_t>((hi + 1) * curve.m_cv_stride, curve.m_cv.size()));
    return piece.trim(first, last);
}

Result intersect(const NurbsCurve& a, const NurbsCurve& b, double tolerance) {
    Result straight_result;
    if (straight_pair(a, b, tolerance, straight_result))
        return straight_result;
    nurbs_memo::Scope* memo = nurbs_memo::Scope::active;
    if (!memo)
        return intersect_once(a, b, tolerance);
    std::string key;
    nurbs_memo::add(key, a);
    nurbs_memo::add(key, b);
    nurbs_memo::add(key, &tolerance, 1);
    const auto found = memo->curve_pairs.find(key);
    if (found != memo->curve_pairs.end())
        return found->second;
    Result result = intersect_once(a, b, tolerance);
    if (result.complete)
        memo->curve_pairs.emplace(std::move(key), result);
    return result;
}

}
