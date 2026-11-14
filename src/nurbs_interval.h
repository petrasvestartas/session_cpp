#pragma once

#include "nurbssurface.h"
#include "nurbscurve.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <span>
#include <vector>

namespace session_cpp::nurbs_interval {

struct Interval {
    double lo = 0.0;
    double hi = 0.0;
    Interval() = default;
    Interval(double value) : lo(value), hi(value) {}
    Interval(double first, double last) : lo(first), hi(last) {}
};

/// The next double above value, as std::nextafter(value, +infinity) without the library call.
inline double next_up(double value) {
    if (!(value < std::numeric_limits<double>::infinity()))
        return value;
    if (value == 0.0)
        return std::numeric_limits<double>::denorm_min();
    const std::uint64_t bits = std::bit_cast<std::uint64_t>(value);
    return std::bit_cast<double>(value > 0.0 ? bits + 1 : bits - 1);
}

/// The next double below value, as std::nextafter(value, -infinity) without the library call.
inline double next_down(double value) {
    return -next_up(-value);
}

inline Interval outward(double lo, double hi) {
    if (!std::isfinite(lo) || !std::isfinite(hi) || lo > hi)
        return {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    return {next_down(lo), next_up(hi)};
}

inline bool finite(Interval value) {
    return std::isfinite(value.lo) && std::isfinite(value.hi) && value.lo <= value.hi;
}

inline Interval operator+(Interval a, Interval b) {
    if (a.lo == 0.0 && a.hi == 0.0)
        return b;
    if (b.lo == 0.0 && b.hi == 0.0)
        return a;
    if (finite(a) && finite(b) && a.lo == a.hi && b.lo == b.hi && a.lo == -b.lo)
        return {};
    return outward(a.lo + b.lo, a.hi + b.hi);
}

inline Interval operator-(Interval a, Interval b) {
    if (b.lo == 0.0 && b.hi == 0.0)
        return a;
    if (a.lo == a.hi && b.lo == b.hi && a.lo == b.lo)
        return {};
    return outward(a.lo - b.hi, a.hi - b.lo);
}

/// A factor classified once for many products.
struct Factor {
    enum class Kind : unsigned char { general, zero, one, minus_one, unbounded, positive };
    Interval value;
    Kind kind = Kind::zero;

    Factor() = default;
    explicit Factor(Interval a)
        : value(a), kind(a.lo == 0.0 && a.hi == 0.0 ? Kind::zero : a.lo == 1.0 && a.hi == 1.0 ? Kind::one :
            !finite(a) ? Kind::unbounded : a.lo == -1.0 && a.hi == -1.0 ? Kind::minus_one :
            a.lo >= 0.0 ? Kind::positive : Kind::general) {}
};

inline Interval operator*(const Factor& a, Interval b) {
    if (a.kind == Factor::Kind::zero || (b.lo == 0.0 && b.hi == 0.0))
        return {};
    if (a.kind == Factor::Kind::one)
        return b;
    if (b.lo == 1.0 && b.hi == 1.0)
        return a.value;
    if (a.kind == Factor::Kind::unbounded || !finite(b))
        return {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    if (a.kind == Factor::Kind::minus_one)
        return {-b.hi, -b.lo};
    if (b.lo == -1.0 && b.hi == -1.0)
        return {-a.value.hi, -a.value.lo};
    // The four products are finite or infinite, never NaN, and outward takes either zero alike: any smallest and largest give the same interval. Rounding keeps order, so those of a factor that is nowhere negative are known by the signs of b.
    double lo, hi;
    if (a.kind == Factor::Kind::positive) {
        lo = (b.lo >= 0.0 ? a.value.lo : a.value.hi) * b.lo;
        hi = (b.hi >= 0.0 ? a.value.hi : a.value.lo) * b.hi;
    } else {
        const double first = a.value.lo * b.lo, second = a.value.lo * b.hi, third = a.value.hi * b.lo, fourth = a.value.hi * b.hi;
        lo = std::min(std::min(first, second), std::min(third, fourth));
        hi = std::max(std::max(first, second), std::max(third, fourth));
    }
    return outward(lo, hi);
}

inline Interval operator*(Interval a, Interval b) {
    return Factor(a) * b;
}

/// A divisor prepared once for many dividends: its reciprocal is rounded once, and dividing by it gives the interval dividing by the interval gives.
struct Divisor {
    bool zero = false;
    bool one = false;
    Interval reciprocal;

    Divisor() = default;
    explicit Divisor(Interval b) : zero(b.lo <= 0.0 && b.hi >= 0.0), one(b.lo == 1.0 && b.hi == 1.0) {
        if (!zero && !one)
            reciprocal = outward(1.0 / b.hi, 1.0 / b.lo);
    }
};

inline Interval operator/(Interval a, const Divisor& b) {
    if (b.zero)
        return {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    if (b.one)
        return a;
    return a * b.reciprocal;
}

inline Interval operator/(Interval a, Interval b) {
    return a / Divisor(b);
}

inline Interval interval_hull(Interval a, Interval b) {
    if (!finite(a) || !finite(b))
        return {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    return {std::min(a.lo, b.lo), std::max(a.hi, b.hi)};
}

inline double magnitude(Interval a) {
    return std::max(std::abs(a.lo), std::abs(a.hi));
}

using Homogeneous = std::array<Interval, 4>;

/// Knot i of a knot vector padded by one repeated knot at each end, read from the original vector: a blossom on a span of the curve or surface never reads the two added ends.
struct PaddedKnots {
    const double* knots;
    double operator[](int index) const {
        return knots[index - 1];
    }
};

/// Index of the first knot above value in the padded knot vector, by the halving steps of std::upper_bound on that vector: the same index on any input, sorted or not.
inline int padded_upper_bound(const std::vector<double>& knots, double value) {
    const int count = static_cast<int>(knots.size());
    int first = 0;
    for (int length = count + 2; length > 0;) {
        const int half = length >> 1;
        const int middle = first + half;
        if (value < knots[std::clamp(middle - 1, 0, count - 1)])
            length = half;
        else {
            first = middle + 1;
            length -= half + 1;
        }
    }
    return first;
}

/// Affine weights of every blossom step on one original NURBS span, at the arguments lo and hi: one table serves every pole and every row of controls on that span.
struct BlossomSteps {
    int degree;
    bool valid = true;
    bool point; // lo and hi are one double: both ends share the table of lo.
    nurbsknot::Scratch<std::array<Factor, 2>, 112> weights;

    template <typename Knots>
    BlossomSteps(const Knots& knots, int span, int degree, double lo, double hi)
        : degree(degree), point(std::bit_cast<std::uint64_t>(lo) == std::bit_cast<std::uint64_t>(hi)),
          weights(2 * degree * (degree + 1)) {
        for (int level = 1; level <= degree && valid; ++level)
            for (int j = degree; j >= level && valid; --j) {
                const int index = span - degree + j;
                const double left = knots[index];
                const double right = knots[index + degree - level + 1];
                const Interval width = Interval(right) - Interval(left);
                valid = !(width.lo <= 0.0);
                const Divisor divisor(width);
                for (int end = 0; end < (point ? 1 : 2) && valid; ++end) {
                    const double t = end ? hi : lo;
                    const Interval alpha = t == left ? Interval(0.0) : t == right ? Interval(1.0) :
                        (Interval(t) - Interval(left)) / divisor;
                    at(end, level, j) = {Factor(Interval(1.0) - alpha), Factor(alpha)};
                }
                if (point && valid)
                    at(1, level, j) = at(0, level, j);
            }
    }

    std::array<Factor, 2>& at(int end, int level, int j) {
        return weights.data[((end * degree) + level - 1) * (degree + 1) + j];
    }

    void step(Homogeneous* work, int end, int level) const {
        for (int j = degree; j >= level; --j) {
            const std::array<Factor, 2>& weight = weights.data[((end * degree) + level - 1) * (degree + 1) + j];
            for (int d = 0; d < 4; ++d)
                if (!(work[j - 1][d].lo == work[j - 1][d].hi && work[j][d].lo == work[j][d].hi &&
                    work[j - 1][d].lo == work[j][d].lo))
                    work[j][d] = weight[0] * work[j - 1][d] + weight[1] * work[j][d];
        }
    }
};

/// Outward-rounded blossom on one original NURBS span, including a point interval: result takes the degree + 1 controls and may be controls itself, scratch holds 2 * (degree + 1).
inline void blossom(const Homogeneous* controls, const BlossomSteps& steps, Homogeneous* result, Homogeneous* scratch) {
    const int degree = steps.degree;
    Homogeneous* low = scratch;
    Homogeneous* work = scratch + degree + 1;
    std::copy_n(controls, degree + 1, low);
    // Every pole of a point takes the same steps: they are taken once.
    if (steps.point) {
        for (int level = 1; level <= degree; ++level)
            steps.step(low, 0, level);
        std::fill_n(result, degree + 1, low[degree]);
        return;
    }
    // A pole takes its first steps at lo and the rest at hi; the steps at lo are the same for every pole after it and are taken once.
    for (int pole = degree; pole >= 0; --pole) {
        std::copy_n(low, degree + 1, work);
        for (int level = degree - pole + 1; level <= degree; ++level)
            steps.step(work, 1, level);
        result[pole] = work[degree];
        if (pole > 0)
            steps.step(low, 0, degree - pole + 1);
    }
}

inline std::vector<Homogeneous> blossom(const std::vector<Homogeneous>& controls, const BlossomSteps& steps) {
    if (!steps.valid)
        return {};
    std::vector<Homogeneous> result(steps.degree + 1);
    nurbsknot::Scratch<Homogeneous, 16> scratch(2 * (steps.degree + 1));
    blossom(controls.data(), steps, result.data(), scratch.data);
    return result;
}

inline std::vector<Homogeneous> blossom(const std::vector<Homogeneous>& controls,
    const std::vector<double>& knots, int span, int degree, double lo, double hi) {
    return blossom(controls, BlossomSteps(knots, span, degree, lo, hi));
}

struct CurveBounds {
    std::array<Interval, 3> point;
};

/// The run of knots [first, last) strictly between lo and hi among those at begin up to end, found by halving as on a sorted vector: on one that is not sorted the run is whatever the steps give, and its curve or surface is invalid.
inline std::array<int, 2> knots_inside(const std::vector<double>& knots, int begin, int end, double lo, double hi) {
    int first = begin;
    for (int length = end - begin; length > 0;) {
        const int half = length >> 1;
        if (knots[first + half] > lo)
            length = half;
        else {
            first += half + 1;
            length -= half + 1;
        }
    }
    int last = first;
    while (last < end && knots[last] < hi)
        ++last;
    return {first, last};
}

/// Write the degree + 1 outward-rounded Bezier controls of the curve over interval, relative to origin; false when the interval leaves one knot span or the input is unset.
inline bool curve_controls(const NurbsCurve& curve, const std::array<double, 2>& interval,
    const std::array<double, 3>& origin, Homogeneous* controls) {
    if (curve.order() < 2 || curve.cv_count() < curve.order() || curve.dimension() < 1 || curve.dimension() > 3 ||
        curve.m_nurbsknot.size() != static_cast<std::size_t>(curve.order() + curve.cv_count() - 2) ||
        !std::isfinite(interval[0]) || !std::isfinite(interval[1]) ||
        !std::isfinite(origin[0]) || !std::isfinite(origin[1]) || !std::isfinite(origin[2]) ||
        interval[0] < curve.domain_start() || interval[1] > curve.domain_end() || interval[0] > interval[1])
        return false;
    const std::vector<double>& knots = curve.m_nurbsknot;
    const int degree = curve.degree();
    const int index = std::clamp(padded_upper_bound(knots, interval[0]) - 1, degree, curve.cv_count() - 1);
    if (interval[0] < knots[index - 1] || interval[1] > knots[index])
        return false;
    for (int i = 0; i <= degree; ++i) {
        const double* cv = curve.cv(index - degree + i);
        const double weight = curve.is_rational() ? cv[curve.dimension()] : 1.0;
        if (!(weight > 0.0) || !std::isfinite(weight))
            return false;
        controls[i][3] = weight;
        for (int d = 0; d < 3; ++d)
            controls[i][d] = Interval(d < curve.dimension() ? cv[d] : 0.0) - Interval(origin[d]) * Interval(weight);
    }
    const BlossomSteps steps(PaddedKnots{knots.data()}, index, degree, interval[0], interval[1]);
    if (!steps.valid)
        return false;
    nurbsknot::Scratch<Homogeneous, 16> scratch(2 * (degree + 1));
    blossom(controls, steps, controls, scratch.data);
    return true;
}

inline std::vector<Homogeneous> curve_controls(
    const NurbsCurve& curve,
    const std::array<double, 2>& interval,
    const std::array<double, 3>& origin
) {
    std::vector<Homogeneous> controls(std::max(curve.order(), 0));
    if (!curve_controls(curve, interval, origin, controls.data()))
        controls.clear();
    return controls;
}

inline bool curve_bounds(
    const NurbsCurve& curve,
    const std::array<double, 2>& interval,
    const std::array<double, 3>& origin,
    CurveBounds& result
) {
    if (curve.order() < 2 || curve.cv_count() < curve.order() ||
        curve.m_nurbsknot.size() != static_cast<std::size_t>(curve.order() + curve.cv_count() - 2) ||
        !std::isfinite(interval[0]) || !std::isfinite(interval[1]) || interval[0] > interval[1] ||
        interval[0] < curve.domain_start() || interval[1] > curve.domain_end())
        return false;
    // The cuts are the interval ends and the distinct knots between them, the list of get_span_vector, which is empty on an invalid curve: the curve is checked only where a knot lies between.
    const std::vector<double>& knots = curve.m_nurbsknot;
    const std::array<int, 2> inside = knots_inside(knots, curve.order() - 2, curve.cv_count(), interval[0], interval[1]);
    const bool valid = inside[0] < inside[1] && curve.is_valid();
    nurbsknot::Scratch<double, 32> cuts(valid ? inside[1] - inside[0] + 2 : 2);
    std::size_t count = 0;
    cuts.data[count++] = interval[0];
    for (int i = inside[0]; valid && i < inside[1]; ++i)
        if (i == curve.order() - 2 || knots[i] > knots[i - 1])
            cuts.data[count++] = knots[i];
    cuts.data[count++] = interval[1];
    const auto same = [](double a, double b) { return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b); };
    // A point is its own two ends: one evaluation stands for the three.
    if (same(interval[0], interval[1]))
        count = 1;
    nurbsknot::Scratch<Homogeneous, 16> data(curve.order());
    bool first = true;
    for (std::size_t i = 0; i < count; ++i) {
        for (int endpoint = 0; endpoint < (i == 0 ? 1 : 2); ++endpoint) {
            const double lo = endpoint == 0 ? cuts.data[i] : cuts.data[i - 1];
            if (!curve_controls(curve, {lo, cuts.data[i]}, origin, data.data))
                return false;
            // The controls of a point are one control.
            for (int k = 0; k < (same(lo, cuts.data[i]) ? 1 : curve.order()); ++k) {
                const Homogeneous& cv = data.data[k];
                if (!(cv[3].lo > 0.0))
                    return false;
                const Divisor weight(cv[3]);
                for (int d = 0; d < 3; ++d) {
                    const Interval value = cv[d] / weight;
                    if (!finite(value))
                        return false;
                    result.point[d] = first ? value : interval_hull(result.point[d], value);
                }
                first = false;
            }
        }
    }
    return !first;
}

/// Prove that every point of an original-curve interval lies inside one world-space ball.
inline bool curve_within_ball(const NurbsCurve& curve, const std::array<double, 2>& interval,
    const std::array<double, 3>& center, double radius) {
    CurveBounds bounds;
    if (!(radius >= 0.0) || !std::isfinite(radius) || !curve_bounds(curve, interval, center, bounds))
        return false;
    Interval squared;
    for (const auto coordinate : bounds.point) {
        const Interval distance(magnitude(coordinate));
        squared = squared + distance * distance;
    }
    const Interval allowed = Interval(radius) * Interval(radius);
    return finite(squared) && finite(allowed) && squared.hi <= allowed.lo;
}

struct SurfaceBounds {
    std::array<Interval, 3> point;
    std::array<std::array<Interval, 3>, 2> derivative;
};

/// Write the (p + 1) * (q + 1) outward-rounded Bezier controls of the surface over rectangle, relative to origin; false when the rectangle leaves one knot cell or a control is not finite with a positive weight.
inline bool surface_controls(const NurbsSurface& surface, const std::array<double, 4>& rectangle,
    const std::array<double, 3>& origin, bool extend, Homogeneous* controls) {
    if (surface.dimension() != 3)
        return false;
    std::array<int, 2> span;
    const int p = surface.degree(0);
    const int q = surface.degree(1);
    for (int d = 0; d < 2; ++d) {
        const std::vector<double>& knots = surface.m_nurbsknot[d];
        span[d] = std::clamp(padded_upper_bound(knots, rectangle[d * 2]) - 1, surface.degree(d), surface.cv_count(d) - 1);
        if ((rectangle[d * 2] < knots[span[d] - 1] && !(extend && span[d] == surface.degree(d))) ||
            (rectangle[d * 2 + 1] > knots[span[d]] && !(extend && span[d] == surface.cv_count(d) - 1)))
            return false;
    }
    const BlossomSteps along_u(PaddedKnots{surface.m_nurbsknot[0].data()}, span[0], p, rectangle[0], rectangle[1]);
    const BlossomSteps along_v(PaddedKnots{surface.m_nurbsknot[1].data()}, span[1], q, rectangle[2], rectangle[3]);
    if (!along_u.valid)
        return false;
    const int widest = std::max(p, q) + 1;
    nurbsknot::Scratch<Homogeneous, 32> scratch(4 * widest);
    Homogeneous* column = scratch.data;
    Homogeneous* extracted = column + widest;
    Homogeneous* work = extracted + widest;
    for (int j = 0; j <= q; ++j) {
        for (int i = 0; i <= p; ++i) {
            const double* cv = surface.cv(span[0] - p + i, span[1] - q + j);
            const double weight = surface.is_rational() ? cv[surface.dimension()] : 1.0;
            column[i][3] = weight;
            for (int d = 0; d < 3; ++d)
                column[i][d] = Interval(cv[d]) - Interval(origin[d]) * Interval(weight);
        }
        blossom(column, along_u, extracted, work);
        for (int i = 0; i <= p; ++i)
            controls[i * (q + 1) + j] = extracted[i];
    }
    if (!along_v.valid)
        return false;
    // At a point in u every row holds the same controls.
    for (int i = 0; i <= (along_u.point ? 0 : p); ++i)
        blossom(controls + i * (q + 1), along_v, controls + i * (q + 1), work);
    for (int i = 1; along_u.point && i <= p; ++i)
        std::copy_n(controls, q + 1, controls + i * (q + 1));
    for (int k = 0; k < (p + 1) * (q + 1); ++k) {
        for (Interval value : controls[k])
            if (!finite(value))
                return false;
        if (!(controls[k][3].lo > 0.0))
            return false;
    }
    return true;
}

inline std::vector<Homogeneous> surface_controls(const NurbsSurface& surface,
    const std::array<double, 4>& rectangle, const std::array<double, 3>& origin, bool extend = false) {
    if (surface.dimension() != 3)
        return {};
    std::vector<Homogeneous> controls((surface.degree(0) + 1) * (surface.degree(1) + 1));
    if (!surface_controls(surface, rectangle, origin, extend, controls.data()))
        controls.clear();
    return controls;
}

inline bool bezier_bounds(const NurbsSurface& surface, const std::array<double, 4>& rectangle,
    const std::array<double, 3>& origin, SurfaceBounds& bounds, bool derivatives, bool extend = false) {
    if (surface.dimension() != 3)
        return false;
    const int p = surface.degree(0);
    const int q = surface.degree(1);
    nurbsknot::Scratch<Homogeneous, 36> storage((p + 1) * (q + 1));
    if (!surface_controls(surface, rectangle, origin, extend, storage.data))
        return false;
    const std::span<const Homogeneous> controls(storage.data, (p + 1) * (q + 1));
    // Along a side that is one point the controls repeat: the first of them stands for all, a point is one control and a line one row of them.
    const int rows = std::bit_cast<std::uint64_t>(rectangle[0]) == std::bit_cast<std::uint64_t>(rectangle[1]) ? 1 : p + 1;
    const int columns = std::bit_cast<std::uint64_t>(rectangle[2]) == std::bit_cast<std::uint64_t>(rectangle[3]) ? 1 : q + 1;
    Homogeneous hull = controls.front();
    nurbsknot::Scratch<Divisor, 36> weights(controls.size());
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < columns; ++j) {
            const Homogeneous& cv = controls[i * (q + 1) + j];
            for (int d = 0; d < 4; ++d)
                hull[d] = interval_hull(hull[d], cv[d]);
            weights.data[i * (q + 1) + j] = Divisor(cv[3]);
        }
    for (int d = 0; d < 3; ++d) {
        bounds.point[d] = controls.front()[d] / weights.data[0];
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < columns; ++j)
                bounds.point[d] = interval_hull(bounds.point[d], controls[i * (q + 1) + j][d] / weights.data[i * (q + 1) + j]);
        if (!finite(bounds.point[d]))
            return false;
    }
    if (!derivatives)
        return true;
    for (int direction = 0; direction < 2; ++direction) {
        const Interval width = Interval(rectangle[direction * 2 + 1]) - Interval(rectangle[direction * 2]);
        if (!(width.lo > 0.0))
            return false;
        const Divisor divisor(width);
        Homogeneous derivative;
        bool first = true;
        for (int i = 0; i <= p - (direction == 0); ++i)
            for (int j = 0; j <= q - (direction == 1); ++j) {
                const auto& a = controls[i * (q + 1) + j];
                const auto& b = controls[(i + (direction == 0)) * (q + 1) + j + (direction == 1)];
                for (int d = 0; d < 4; ++d) {
                    const Interval value = Interval(surface.degree(direction)) * (b[d] - a[d]) / divisor;
                    if (!finite(value))
                        return false;
                    derivative[d] = first ? value : interval_hull(derivative[d], value);
                }
                first = false;
            }
        const Divisor weight(hull[3]), squared(hull[3] * hull[3]);
        for (int d = 0; d < 3; ++d) {
            const Interval quotient = (derivative[d] * hull[3] - hull[d] * derivative[3]) / squared;
            const Interval recurrence = (derivative[d] - bounds.point[d] * derivative[3]) / weight;
            bounds.derivative[direction][d] = {std::max(quotient.lo, recurrence.lo), std::min(quotient.hi, recurrence.hi)};
            if (!finite(bounds.derivative[direction][d]))
                return false;
        }
    }
    return true;
}

inline bool surface_bounds(const NurbsSurface& surface, const std::array<double, 4>& rectangle,
    const std::array<double, 3>& origin, SurfaceBounds& result, bool derivatives = true, bool extend = false) {
    // The cuts are the rectangle sides and the distinct knots between them, the list of get_span_vector, which is empty on an invalid surface: the surface is checked only where a knot lies between.
    std::array<std::array<int, 2>, 2> inside{};
    for (int d = 0; d < 2; ++d)
        if (surface.order(d) >= 2 && surface.cv_count(d) >= surface.order(d) &&
            surface.m_nurbsknot[d].size() == static_cast<std::size_t>(surface.order(d) + surface.cv_count(d) - 2))
            inside[d] = knots_inside(surface.m_nurbsknot[d], surface.order(d) - 2, surface.cv_count(d), rectangle[d * 2], rectangle[d * 2 + 1]);
    const bool valid = (inside[0][0] < inside[0][1] || inside[1][0] < inside[1][1]) && surface.is_valid();
    nurbsknot::Scratch<double, 64> storage(valid ? inside[0][1] - inside[0][0] + inside[1][1] - inside[1][0] + 4 : 4);
    std::array<std::span<const double>, 2> cuts;
    double* next = storage.data;
    for (int d = 0; d < 2; ++d) {
        double* first_cut = next;
        const std::vector<double>& knots = surface.m_nurbsknot[d];
        *next++ = rectangle[d * 2];
        for (int i = inside[d][0]; valid && i < inside[d][1]; ++i)
            if (i == surface.order(d) - 2 || knots[i] > knots[i - 1])
                *next++ = knots[i];
        *next++ = rectangle[d * 2 + 1];
        cuts[d] = {first_cut, next};
    }
    bool first = true;
    for (size_t i = 1; i < cuts[0].size(); ++i)
        for (size_t j = 1; j < cuts[1].size(); ++j) {
            SurfaceBounds part;
            if (!bezier_bounds(surface, {cuts[0][i - 1], cuts[0][i], cuts[1][j - 1], cuts[1][j]},
                origin, part, derivatives, extend))
                return false;
            if (first)
                result = part;
            else
                for (int d = 0; d < 3; ++d) {
                    result.point[d] = interval_hull(result.point[d], part.point[d]);
                    if (derivatives)
                        for (int k = 0; k < 2; ++k)
                            result.derivative[k][d] = interval_hull(result.derivative[k][d], part.derivative[k][d]);
                }
            first = false;
        }
    return !first;
}

/// The enclosure surface_bounds gives the rectangle that is the point (u, v) alone, a point past the domain taken on its end span: one blossom along each direction.
inline bool surface_point_bounds(const NurbsSurface& surface, double u, double v, const std::array<double, 3>& origin, SurfaceBounds& bounds) {
    return surface_bounds(surface, {u, u, v, v}, origin, bounds, false, true);
}

}
