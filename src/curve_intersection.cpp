#include "curve_intersection.h"
#include "closest.h"
#include <algorithm>
#include <array>
#include <cmath>
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

    explicit Bounds(const NurbsCurve& curve) : lo(curve.get_cv(0)), hi(lo) {
        for (int i = 1; i < curve.cv_count(); ++i) {
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
    NurbsCurve result = curve;
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
        const std::vector<Vector> d = curve.evaluate(t, 2);
        const Vector r = curve.point_at(t) - point;
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

std::vector<std::pair<double, double>> endpoint_matches(const Pair& pair, double tolerance) {
    std::vector<std::pair<double, double>> matches;
    for (const double ta : {pair.a.domain_start(), pair.a.domain_end()}) {
        const Point p = pair.a.point_at(ta);
        const double tb = closest_parameter(pair.b, p);
        if (p.distance(pair.b.point_at(tb)) <= tolerance)
            matches.emplace_back(ta, tb);
    }
    for (const double tb : {pair.b.domain_start(), pair.b.domain_end()}) {
        const Point p = pair.b.point_at(tb);
        const double ta = closest_parameter(pair.a, p);
        if (p.distance(pair.a.point_at(ta)) <= tolerance)
            matches.emplace_back(ta, tb);
    }
    std::sort(matches.begin(), matches.end());

    return matches;
}

bool overlap(const Pair& pair, double tolerance, Overlap& result) {
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
    if (distance > tolerance * 1.2)
        return true;

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
        if (leaf.depth >= 96 || !subdivide(leaf, fa >= fb, work))
            return false;
    }

    return true;
}

bool process(const Pair& pair, double tolerance, size_t& budget, Result& result) {
    const Bounds a(pair.a);
    const Bounds b(pair.b);
    if (!a.overlaps(b, tolerance))
        return true;

    const double proof = std::max(tolerance * 1e-5, epsilon * 32.0 * std::max(a.magnitude(), b.magnitude()));
    Overlap shared{};
    if (!overlap(pair, proof, shared))
        return isolated(pair, tolerance, budget, result);

    result.overlaps.push_back(shared);
    std::vector<Pair> work;
    remaining_pairs(pair, shared, work);
    for (const Pair& piece : work)
        if (!isolated(piece, tolerance, budget, result))
            return false;

    return true;
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
    const double da = a.domain_end() - a0;
    const double db = b.domain_end() - b0;
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

        hit.a = a0 + hit.a * da;
        hit.b = b0 + hit.b * db;
        points.push_back(hit);
    }
    result.points = std::move(points);
    for (Overlap& shared : result.overlaps) {
        shared.a0 = a0 + shared.a0 * da;
        shared.a1 = a0 + shared.a1 * da;
        shared.b0 = b0 + shared.b0 * db;
        shared.b1 = b0 + shared.b1 * db;
    }
}

}

Result intersect(const NurbsCurve& a, const NurbsCurve& b, double tolerance) {
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
        result.complete = false;
        result.failure = "Curve intersection has a constant input or tolerance below coordinate resolution";
        return result;
    }
    NurbsCurve first = a;
    NurbsCurve second = b;
    if (!first.set_domain(0.0, 1.0) || !second.set_domain(0.0, 1.0))
        throw std::runtime_error("Curve intersection could not normalize parameter domains");

    const std::vector<double> av = first.get_span_vector();
    const std::vector<double> bv = second.get_span_vector();
    size_t budget = 0;
    for (size_t i = 1; i < av.size() && result.complete; ++i)
        for (size_t j = 1; j < bv.size() && result.complete; ++j) {
            const Pair pair{interval(first, av[i - 1], av[i]), interval(second, bv[j - 1], bv[j])};
            result.complete = budget++ < work_limit && process(pair, tolerance, budget, result);
        }

    if (!result.complete && result.failure.empty())
        result.failure = "Curve intersection exceeded its subdivision or floating-point resolution limit";

    map_result(a, b, result);
    return result;
}

}
