#pragma once

#include "nurbssurface.h"
#include "nurbscurve.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace session_cpp::nurbs_interval {

struct Interval {
    double lo = 0.0;
    double hi = 0.0;
    Interval() = default;
    Interval(double value) : lo(value), hi(value) {}
    Interval(double first, double last) : lo(first), hi(last) {}
};

inline Interval outward(double lo, double hi) {
    if (!std::isfinite(lo) || !std::isfinite(hi) || lo > hi)
        return {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    return {std::nextafter(lo, -std::numeric_limits<double>::infinity()),
        std::nextafter(hi, std::numeric_limits<double>::infinity())};
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

inline Interval operator*(Interval a, Interval b) {
    if ((a.lo == 0.0 && a.hi == 0.0) || (b.lo == 0.0 && b.hi == 0.0))
        return {};
    if (a.lo == 1.0 && a.hi == 1.0)
        return b;
    if (b.lo == 1.0 && b.hi == 1.0)
        return a;
    if (!finite(a) || !finite(b))
        return {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    if (a.lo == -1.0 && a.hi == -1.0)
        return {-b.hi, -b.lo};
    if (b.lo == -1.0 && b.hi == -1.0)
        return {-a.hi, -a.lo};
    const std::array<double, 4> products = {a.lo * b.lo, a.lo * b.hi, a.hi * b.lo, a.hi * b.hi};
    return outward(*std::min_element(products.begin(), products.end()),
        *std::max_element(products.begin(), products.end()));
}

inline Interval operator/(Interval a, Interval b) {
    if (b.lo <= 0.0 && b.hi >= 0.0)
        return {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    if (b.lo == 1.0 && b.hi == 1.0)
        return a;
    return a * outward(1.0 / b.hi, 1.0 / b.lo);
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

/// Outward-rounded blossom on one original NURBS span, including a point interval.
inline std::vector<Homogeneous> blossom(const std::vector<Homogeneous>& controls,
    const std::vector<double>& knots, int span, int degree, double lo, double hi) {
    std::vector<Homogeneous> result(degree + 1);
    for (int pole = 0; pole <= degree; ++pole) {
        std::vector<Homogeneous> work = controls;
        for (int level = 1; level <= degree; ++level) {
            const double t = level > degree - pole ? hi : lo;
            for (int j = degree; j >= level; --j) {
                const int index = span - degree + j;
                const double left = knots[index];
                const double right = knots[index + degree - level + 1];
                const Interval width = Interval(right) - Interval(left);
                if (width.lo <= 0.0)
                    return {};
                const Interval alpha = t == left ? Interval(0.0) : t == right ? Interval(1.0) :
                    (Interval(t) - Interval(left)) / width;
                for (int d = 0; d < 4; ++d)
                    if (!(work[j - 1][d].lo == work[j - 1][d].hi && work[j][d].lo == work[j][d].hi &&
                        work[j - 1][d].lo == work[j][d].lo))
                        work[j][d] = (Interval(1.0) - alpha) * work[j - 1][d] + alpha * work[j][d];
            }
        }
        result[pole] = work[degree];
    }
    return result;
}

struct CurveBounds {
    std::array<Interval, 3> point;
};

inline std::vector<Homogeneous> curve_controls(
    const NurbsCurve& curve,
    const std::array<double, 2>& interval,
    const std::array<double, 3>& origin
) {
    if (curve.order() < 2 || curve.cv_count() < curve.order() || curve.dimension() < 1 || curve.dimension() > 3 ||
        curve.m_nurbsknot.size() != static_cast<std::size_t>(curve.order() + curve.cv_count() - 2) ||
        !std::isfinite(interval[0]) || !std::isfinite(interval[1]) ||
        !std::isfinite(origin[0]) || !std::isfinite(origin[1]) || !std::isfinite(origin[2]) ||
        interval[0] < curve.domain_start() || interval[1] > curve.domain_end() || interval[0] > interval[1])
        return {};
    std::vector<double> knots = curve.m_nurbsknot;
    knots.insert(knots.begin(), knots.front());
    knots.push_back(knots.back());
    const int degree = curve.degree();
    const int index = std::clamp(static_cast<int>(std::upper_bound(knots.begin(), knots.end(), interval[0]) -
        knots.begin()) - 1, degree, curve.cv_count() - 1);
    if (interval[0] < knots[index] || interval[1] > knots[index + 1])
        return {};
    std::vector<Homogeneous> data(degree + 1);
    for (int i = 0; i <= degree; ++i) {
        const double* cv = curve.cv(index - degree + i);
        const double weight = curve.is_rational() ? cv[curve.dimension()] : 1.0;
        if (!(weight > 0.0) || !std::isfinite(weight))
            return {};
        data[i][3] = weight;
        for (int d = 0; d < 3; ++d)
            data[i][d] = Interval(d < curve.dimension() ? cv[d] : 0.0) - Interval(origin[d]) * Interval(weight);
    }
    return blossom(data, knots, index, degree, interval[0], interval[1]);
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
    std::vector<double> cuts = {interval[0]};
    for (double knot : curve.get_span_vector())
        if (knot > interval[0] && knot < interval[1])
            cuts.push_back(knot);
    cuts.push_back(interval[1]);
    bool first = true;
    for (std::size_t i = 0; i < cuts.size(); ++i) {
        for (int endpoint = 0; endpoint < (i == 0 ? 1 : 2); ++endpoint) {
            const double lo = endpoint == 0 ? cuts[i] : cuts[i - 1];
            const auto data = curve_controls(curve, {lo, cuts[i]}, origin);
            if (data.empty())
                return false;
            for (const auto& cv : data) {
                if (!(cv[3].lo > 0.0))
                    return false;
                for (int d = 0; d < 3; ++d) {
                    const Interval value = cv[d] / cv[3];
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

inline std::vector<Homogeneous> surface_controls(const NurbsSurface& surface,
    const std::array<double, 4>& rectangle, const std::array<double, 3>& origin, bool extend = false) {
    if (surface.dimension() != 3)
        return {};
    std::array<std::vector<double>, 2> knots;
    std::array<int, 2> span;
    const int p = surface.degree(0);
    const int q = surface.degree(1);
    for (int d = 0; d < 2; ++d) {
        knots[d] = surface.m_nurbsknot[d];
        knots[d].insert(knots[d].begin(), knots[d].front());
        knots[d].push_back(knots[d].back());
        const double middle = std::midpoint(rectangle[d * 2], rectangle[d * 2 + 1]);
        span[d] = std::clamp(static_cast<int>(std::upper_bound(knots[d].begin(), knots[d].end(), middle) -
            knots[d].begin()) - 1, surface.degree(d), surface.cv_count(d) - 1);
        if ((rectangle[d * 2] < knots[d][span[d]] && !(extend && span[d] == surface.degree(d))) ||
            (rectangle[d * 2 + 1] > knots[d][span[d] + 1] && !(extend && span[d] == surface.cv_count(d) - 1)))
            return {};
    }
    std::vector<Homogeneous> controls((p + 1) * (q + 1));
    for (int j = 0; j <= q; ++j) {
        std::vector<Homogeneous> column(p + 1);
        for (int i = 0; i <= p; ++i) {
            const double* cv = surface.cv(span[0] - p + i, span[1] - q + j);
            const double weight = surface.is_rational() ? cv[surface.dimension()] : 1.0;
            column[i][3] = weight;
            for (int d = 0; d < 3; ++d)
                column[i][d] = Interval(cv[d]) - Interval(origin[d]) * Interval(weight);
        }
        const auto extracted = blossom(column, knots[0], span[0], p, rectangle[0], rectangle[1]);
        if (extracted.empty())
            return {};
        for (int i = 0; i <= p; ++i)
            controls[i * (q + 1) + j] = extracted[i];
    }
    for (int i = 0; i <= p; ++i) {
        std::vector<Homogeneous> row(controls.begin() + i * (q + 1), controls.begin() + (i + 1) * (q + 1));
        const auto extracted = blossom(row, knots[1], span[1], q, rectangle[2], rectangle[3]);
        if (extracted.empty())
            return {};
        std::copy(extracted.begin(), extracted.end(), controls.begin() + i * (q + 1));
    }
    for (const auto& cv : controls) {
        for (Interval value : cv)
            if (!finite(value))
                return {};
        if (!(cv[3].lo > 0.0))
            return {};
    }
    return controls;
}

inline bool bezier_bounds(const NurbsSurface& surface, const std::array<double, 4>& rectangle,
    const std::array<double, 3>& origin, SurfaceBounds& bounds, bool derivatives, bool extend = false) {
    const auto controls = surface_controls(surface, rectangle, origin, extend);
    if (controls.empty())
        return false;
    const int p = surface.degree(0);
    const int q = surface.degree(1);
    Homogeneous hull = controls.front();
    for (const auto& cv : controls)
        for (int d = 0; d < 4; ++d)
            hull[d] = interval_hull(hull[d], cv[d]);
    for (int d = 0; d < 3; ++d) {
        bounds.point[d] = controls.front()[d] / controls.front()[3];
        for (const auto& cv : controls)
            bounds.point[d] = interval_hull(bounds.point[d], cv[d] / cv[3]);
        if (!finite(bounds.point[d]))
            return false;
    }
    if (!derivatives)
        return true;
    for (int direction = 0; direction < 2; ++direction) {
        const Interval width = Interval(rectangle[direction * 2 + 1]) - Interval(rectangle[direction * 2]);
        if (!(width.lo > 0.0))
            return false;
        Homogeneous derivative;
        bool first = true;
        for (int i = 0; i <= p - (direction == 0); ++i)
            for (int j = 0; j <= q - (direction == 1); ++j) {
                const auto& a = controls[i * (q + 1) + j];
                const auto& b = controls[(i + (direction == 0)) * (q + 1) + j + (direction == 1)];
                for (int d = 0; d < 4; ++d) {
                    const Interval value = Interval(surface.degree(direction)) * (b[d] - a[d]) / width;
                    if (!finite(value))
                        return false;
                    derivative[d] = first ? value : interval_hull(derivative[d], value);
                }
                first = false;
            }
        for (int d = 0; d < 3; ++d) {
            const Interval quotient = (derivative[d] * hull[3] - hull[d] * derivative[3]) /
                (hull[3] * hull[3]);
            const Interval recurrence = (derivative[d] - bounds.point[d] * derivative[3]) / hull[3];
            bounds.derivative[direction][d] = {std::max(quotient.lo, recurrence.lo), std::min(quotient.hi, recurrence.hi)};
            if (!finite(bounds.derivative[direction][d]))
                return false;
        }
    }
    return true;
}

inline bool surface_bounds(const NurbsSurface& surface, const std::array<double, 4>& rectangle,
    const std::array<double, 3>& origin, SurfaceBounds& result, bool derivatives = true, bool extend = false) {
    std::array<std::vector<double>, 2> cuts;
    for (int d = 0; d < 2; ++d) {
        cuts[d] = {rectangle[d * 2]};
        for (double knot : surface.get_span_vector(d))
            if (knot > rectangle[d * 2] && knot < rectangle[d * 2 + 1])
                cuts[d].push_back(knot);
        cuts[d].push_back(rectangle[d * 2 + 1]);
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

}
