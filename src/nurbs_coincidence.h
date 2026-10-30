#pragma once

#include "nurbssurface.h"
#include "nurbscurve.h"
#include "nurbs_interval.h"
#include <algorithm>
#include <array>
#include <limits>
#include <vector>
#include <cmath>

namespace session_cpp::nurbs_coincidence {

inline bool exact_products_equal(double a, double b, double c, double d);

/// Bound same-parameter surface separation using a shared positive rational basis and control points.
inline bool same_parameters(const NurbsSurface& a, const NurbsSurface& b, double tolerance) {
    for (int d = 0; d < 2; ++d)
        if (a.degree(d) != b.degree(d) || a.cv_count(d) != b.cv_count(d)
            || a.m_nurbsknot[d] != b.m_nurbsknot[d])
            return false;
    const double wa = a.weight(0, 0);
    const double wb = b.weight(0, 0);
    if (!(wa > 0.0) || !(wb > 0.0))
        return false;
    for (int i = 0; i < a.cv_count(0); ++i)
        for (int j = 0; j < a.cv_count(1); ++j) {
            const double x = a.weight(i, j), y = b.weight(i, j);
            const double gap = a.get_cv(i, j).distance(b.get_cv(i, j));
            if (!(x > 0) || !(y > 0) || !exact_products_equal(x, wb, y, wa)
                || !std::isfinite(gap) || gap > tolerance * 0.25)
                return false;
        }
    return true;
}

struct ParameterMap {
    int axis[2] = {0, 1};
    double scale[2] = {1, 1};
    double offset[2] = {0, 0};

    Point apply(const Point& uv) const {
        return {scale[0] * uv[axis[0]] + offset[0], scale[1] * uv[axis[1]] + offset[1], 0};
    }
};

inline bool exact_products_equal(double a, double b, double c, double d) {
    if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c) || !std::isfinite(d))
        return false;
    if (a == 0 || b == 0 || c == 0 || d == 0)
        return (a == 0 || b == 0) && (c == 0 || d == 0);
    int ea, eb, ec, ed;
    const double ma = std::frexp(a, &ea), mb = std::frexp(b, &eb);
    const double mc = std::frexp(c, &ec), md = std::frexp(d, &ed);
    const int exponent = std::max(ea + eb, ec + ed);
    if (std::abs((ea + eb) - (ec + ed)) > 2)
        return false;
    const double first = ma * mb, second = mc * md;
    const double first_error = std::fma(ma, mb, -first), second_error = std::fma(mc, md, -second);
    return std::ldexp(first, ea + eb - exponent) == std::ldexp(second, ec + ed - exponent)
        && std::ldexp(first_error, ea + eb - exponent) == std::ldexp(second_error, ec + ed - exponent);
}

inline bool exact_difference(double a, double b, double& difference) {
    difference = a - b;
    const double part = a - difference;
    const double error = (a - (difference + part)) + (part - b);
    return std::isfinite(difference) && error == 0;
}

inline bool affine_knot(double x, double a0, double a1, double y, double b0, double b1) {
    double dx, da, dy, db;
    if (!exact_difference(x, a0, dx) || !exact_difference(a1, a0, da)
        || !exact_difference(y, b0, dy) || !exact_difference(b1, b0, db))
        return false;
    if (dx == 0 || dy == 0)
        return dx == 0 && dy == 0;
    return exact_products_equal(dx, db, dy, da);
}

inline bool mapped_knots(const NurbsSurface& a, const NurbsSurface& b, int variant) {
    for (int d = 0; d < 2; ++d) {
        const int target = (variant & 4) ? 1 - d : d;
        const bool reverse = (variant & (1 << d)) != 0;
        if (a.degree(d) != b.degree(target) || a.cv_count(d) != b.cv_count(target)
            || a.m_nurbsknot[d].size() != b.m_nurbsknot[target].size())
            return false;
        const std::pair<double, double> from = a.domain(d), to = b.domain(target);
        for (size_t i = 0; i < a.m_nurbsknot[d].size(); ++i) {
            const size_t j = reverse ? a.m_nurbsknot[d].size() - 1 - i : i;
            if (!affine_knot(a.m_nurbsknot[d][i], from.first, from.second, b.m_nurbsknot[target][j],
                    reverse ? to.second : to.first, reverse ? to.first : to.second))
                return false;
        }
    }
    return true;
}

inline bool mapped_basis(const NurbsSurface& a, const NurbsSurface& candidate, double tolerance) {
    for (int d = 0; d < 2; ++d)
        if (a.degree(d) != candidate.degree(d) || a.cv_count(d) != candidate.cv_count(d))
            return false;
    const double wa = a.weight(0, 0), wb = candidate.weight(0, 0);
    if (!(wa > 0) || !(wb > 0) || !std::isfinite(wa) || !std::isfinite(wb))
        return false;
    for (int i = 0; i < a.cv_count(0); ++i)
        for (int j = 0; j < a.cv_count(1); ++j) {
            const double x = a.weight(i, j), y = candidate.weight(i, j);
            const double gap = a.get_cv(i, j).distance(candidate.get_cv(i, j));
            if (!(x > 0) || !(y > 0) || !std::isfinite(x) || !std::isfinite(y)
                || !exact_products_equal(x, wb, y, wa) || !std::isfinite(gap) || gap > tolerance * 0.25)
                return false;
        }
    return true;
}

// Exact shared knot/control basis and proportional positive weights bound support separation.
inline bool parameter_mapping(const NurbsSurface& a, const NurbsSurface& b, double tolerance, ParameterMap& map) {
    if (!a.is_valid() || !b.is_valid() || !(tolerance > 0) || !std::isfinite(tolerance))
        return false;
    for (int variant = 0; variant < 8; ++variant) {
        if (!mapped_knots(a, b, variant))
            continue;
        NurbsSurface candidate = b;
        if ((variant & 4) && !candidate.transpose())
            continue;
        if ((variant & 1) && !candidate.reverse(0))
            continue;
        if ((variant & 2) && !candidate.reverse(1))
            continue;
        ParameterMap proposed;
        bool valid = true;
        for (int d = 0; d < 2; ++d) {
            const int target = (variant & 4) ? 1 - d : d;
            const std::pair<double, double> from = a.domain(d), to = b.domain(target);
            const bool reverse = (variant & (1 << d)) != 0;
            proposed.axis[target] = d;
            double from_span = 0, to_span = 0;
            valid = valid && exact_difference(from.second, from.first, from_span)
                && exact_difference(reverse ? to.first : to.second, reverse ? to.second : to.first, to_span);
            if (!valid)
                break;
            proposed.scale[target] = to_span / from_span;
            const double origin = proposed.scale[target] * from.first;
            valid = std::isfinite(proposed.scale[target]) && proposed.scale[target] != 0
                && exact_products_equal(proposed.scale[target], from_span, to_span, 1)
                && std::isfinite(origin) && exact_products_equal(proposed.scale[target], from.first, origin, 1)
                && exact_difference(reverse ? to.second : to.first, origin, proposed.offset[target]);
        }
        if (valid && mapped_basis(a, candidate, tolerance)) {
            map = proposed;
            return true;
        }
    }
    return false;
}

inline bool map_pcurve(const ParameterMap& map, NurbsCurve& curve) {
    for (int i = 0; i < curve.cv_count(); ++i) {
        double x, y, z, w;
        if (!curve.get_cv_4d(i, x, y, z, w) || !(w > 0) || !std::isfinite(w))
            return false;
        const double source[2] = {x, y};
        const double u = map.scale[0] * source[map.axis[0]] + map.offset[0] * w;
        const double v = map.scale[1] * source[map.axis[1]] + map.offset[1] * w;
        if (!std::isfinite(u) || !std::isfinite(v) || !curve.set_cv_4d(i, u, v, z, w))
            return false;
    }
    return true;
}

inline std::vector<nurbs_interval::Interval> binomial_intervals(int degree) {
    using nurbs_interval::Interval;
    if (degree < 0 || degree > 42)
        return {};
    std::vector<Interval> result(degree + 1, Interval(1));
    for (int i = 1; i < degree; ++i) {
        result[i] = result[i - 1] * Interval(degree - i + 1) / Interval(i);
        if (!nurbs_interval::finite(result[i]) || result[i].lo <= 0)
            return {};
    }
    return result;
}

inline std::vector<nurbs_interval::Interval> bernstein_product_factors(int first, int second) {
    using nurbs_interval::Interval;
    if (first < 0 || second < 0 || first > 21 || second > 21)
        return {};
    const std::vector<Interval> a = binomial_intervals(first), b = binomial_intervals(second);
    const std::vector<Interval> total = binomial_intervals(first + second);
    if (a.empty() || b.empty() || total.empty())
        return {};
    std::vector<Interval> factors((first + 1) * (second + 1));
    for (int i = 0; i <= first; ++i)
        for (int j = 0; j <= second; ++j) {
            const Interval value = a[i] * b[j] / total[i + j];
            if (!nurbs_interval::finite(value) || value.lo <= 0)
                return {};
            factors[i * (second + 1) + j] = value;
        }
    return factors;
}

inline bool homogeneous_cell_agreement(const std::vector<nurbs_interval::Homogeneous>& a,
    const std::vector<nurbs_interval::Homogeneous>& b, const std::array<int, 2>& p,
    const std::array<int, 2>& q, double tolerance) {
    using nurbs_interval::Interval;
    const std::vector<Interval> u = bernstein_product_factors(p[0], q[0]);
    const std::vector<Interval> v = bernstein_product_factors(p[1], q[1]);
    if (a.size() != static_cast<size_t>((p[0] + 1) * (p[1] + 1)) ||
        b.size() != static_cast<size_t>((q[0] + 1) * (q[1] + 1)) || u.empty() || v.empty())
        return false;
    std::vector<std::array<Interval, 3>> difference((p[0] + q[0] + 1) * (p[1] + q[1] + 1));
    double wa = std::numeric_limits<double>::infinity(), wb = wa;
    for (const nurbs_interval::Homogeneous& point : a)
        wa = std::min(wa, point[3].lo);
    for (const nurbs_interval::Homogeneous& point : b)
        wb = std::min(wb, point[3].lo);
    const Interval denominator = Interval(wa) * Interval(wb);
    if (!nurbs_interval::finite(denominator) || denominator.lo <= 0)
        return false;
    for (int i = 0; i <= p[0]; ++i)
        for (int j = 0; j <= p[1]; ++j)
            for (int k = 0; k <= q[0]; ++k)
                for (int l = 0; l <= q[1]; ++l) {
                    const nurbs_interval::Homogeneous& x = a[i * (p[1] + 1) + j];
                    const nurbs_interval::Homogeneous& y = b[k * (q[1] + 1) + l];
                    const Interval factor = u[i * (q[0] + 1) + k] * v[j * (q[1] + 1) + l];
                    std::array<Interval, 3>& coefficient = difference[(i + k) * (p[1] + q[1] + 1) + j + l];
                    for (int c = 0; c < 3; ++c)
                        coefficient[c] = coefficient[c] + factor * (x[c] * y[3] - y[c] * x[3]);
                }
    Interval squared;
    for (int c = 0; c < 3; ++c) {
        double numerator = 0;
        for (const std::array<Interval, 3>& coefficient : difference) {
            if (!nurbs_interval::finite(coefficient[c]))
                return false;
            numerator = std::max(numerator, nurbs_interval::magnitude(coefficient[c]));
        }
        const Interval bound = Interval(numerator) / denominator;
        squared = squared + bound * bound;
    }
    const Interval allowance = Interval(tolerance) * Interval(.25);
    return nurbs_interval::finite(squared) && squared.hi <= (allowance * allowance).lo;
}

// Bounded same-parameter support agreement for different bases; this is not exact identity.
inline bool common_basis_agreement(const NurbsSurface& a, const NurbsSurface& b, double tolerance) {
    if (!a.is_valid() || !b.is_valid() || a.dimension() != 3 || b.dimension() != 3 ||
        !(tolerance > 0) || !std::isfinite(tolerance))
        return false;
    for (const NurbsSurface* surface : {&a, &b})
        for (int i = 0; i < surface->cv_count(0); ++i)
            for (int j = 0; j < surface->cv_count(1); ++j)
                if (!(surface->weight(i, j) > 0) || !std::isfinite(surface->weight(i, j)))
                    return false;
    bool different = false;
    std::array<std::vector<double>, 2> knots;
    const std::array<int, 2> p = {a.degree(0), a.degree(1)}, q = {b.degree(0), b.degree(1)};
    for (int dir = 0; dir < 2; ++dir) {
        if (a.domain(dir) != b.domain(dir))
            return false;
        different = different || p[dir] != q[dir] || a.cv_count(dir) != b.cv_count(dir) ||
            a.m_nurbsknot[dir] != b.m_nurbsknot[dir];
        knots[dir] = a.get_span_vector(dir);
        const std::vector<double> other = b.get_span_vector(dir);
        knots[dir].insert(knots[dir].end(), other.begin(), other.end());
        std::sort(knots[dir].begin(), knots[dir].end());
        knots[dir].erase(std::unique(knots[dir].begin(), knots[dir].end()), knots[dir].end());
        if (p[dir] < 0 || q[dir] < 0 || p[dir] > 21 || q[dir] > 21 || knots[dir].size() < 2)
            return false;
    }
    if (!different)
        return false;
    const Point first = a.get_cv(0, 0);
    const std::array<double, 3> origin = {first[0], first[1], first[2]};
    for (size_t i = 1; i < knots[0].size(); ++i)
        for (size_t j = 1; j < knots[1].size(); ++j) {
            const std::array<double, 4> cell = {knots[0][i - 1], knots[0][i], knots[1][j - 1], knots[1][j]};
            const std::vector<nurbs_interval::Homogeneous> ca = nurbs_interval::surface_controls(a, cell, origin);
            const std::vector<nurbs_interval::Homogeneous> cb = nurbs_interval::surface_controls(b, cell, origin);
            if (!homogeneous_cell_agreement(ca, cb, p, q, tolerance))
                return false;
        }
    return true;
}

/// Bound both positive-weight control hulls against one nondegenerate plane.
inline bool coplanar(const NurbsSurface& a, const NurbsSurface& b, double tolerance) {
    const Point origin = a.get_cv(0, 0);
    Vector axis;
    for (int i = 0; i < a.cv_count(0); ++i)
        for (int j = 0; j < a.cv_count(1); ++j) {
            const Vector candidate = a.get_cv(i, j) - origin;
            if (candidate.magnitude() > axis.magnitude())
                axis = candidate;
        }
    Vector normal;
    for (int i = 0; i < a.cv_count(0); ++i)
        for (int j = 0; j < a.cv_count(1); ++j) {
            const Vector candidate = axis.cross(a.get_cv(i, j) - origin);
            if (candidate.magnitude() > normal.magnitude())
                normal = candidate;
        }
    if (!normal.normalize_self())
        return false;
    for (const NurbsSurface* surface : {&a, &b})
        for (int i = 0; i < surface->cv_count(0); ++i)
            for (int j = 0; j < surface->cv_count(1); ++j) {
                const double distance = std::abs((surface->get_cv(i, j) - origin).dot(normal));
                if (!(surface->weight(i, j) > 0.0)
                    || !std::isfinite(distance) || distance > tolerance * 0.125)
                    return false;
            }
    return true;
}

}
