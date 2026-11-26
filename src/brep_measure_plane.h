#pragma once
#include "brep_measure_polynomial.h"
#include <map>
#include <tuple>

namespace session_cpp::brep_measure::plane {
using polynomial::Grid;
using polynomial::Interval;

// C1 guarantees equal derivative Bernstein edge coefficients on both sides.
inline bool c1(const NurbsSurface& surface) {
    for (int d = 0; d < 2; ++d) {
        const auto domain = surface.domain(d);
        const auto& knots = surface.m_nurbsknot[d];
        for (size_t i = 0; i < knots.size();) {
            size_t j = i + 1;
            while (j < knots.size() && knots[j] == knots[i])
                ++j;
            if (knots[i] > domain.first && knots[i] < domain.second &&
                j - i > static_cast<size_t>(surface.degree(d) - 1))
                return false;
            i = j;
        }
    }
    return true;
}

/// A cell's coefficients for evaluation in doubles: their midpoints, the largest midpoint magnitude and the largest radius.
struct Midpoints {
    std::vector<double> values;
    double size = 0.0;
    double spread = 0.0;

    explicit Midpoints(const Grid& grid) {
        for (const Interval& value : grid.values) {
            values.push_back(std::midpoint(value.lo, value.hi));
            size = std::max(size, std::abs(values.back()));
            spread = std::max(spread, std::max(value.hi - values.back(), values.back() - value.lo));
        }
    }
};

struct Primitive {
    std::vector<double> u, v;
    std::vector<Grid> cells;
    std::vector<Midpoints> middles;
    using Key = std::tuple<int, int, size_t, size_t>;
    std::map<Key, Interval> derivatives;
    bool shared = false;

    Grid derivative(const Grid& point, int component, int direction, Interval width, size_t cell_u,
                    size_t cell_v, const std::function<void()>& charge) {
        if (!shared)
            return polynomial::derivative(point, direction, width, charge);
        Grid result(point.u - (direction == 0), point.v - (direction == 1));
        for (int i = 0; i <= result.u; ++i)
            for (int j = 0; j <= result.v; ++j) {
                const Key key(component, direction, cell_u * result.u + i, cell_v * result.v + j);
                const auto found = derivatives.find(key);
                if (found != derivatives.end()) {
                    result.at(i, j) = found->second;
                    continue;
                }
                charge();
                const Interval value =
                    Interval(direction ? point.v : point.u) *
                    (point.at(i + (direction == 0), j + (direction == 1)) - point.at(i, j)) / width;
                derivatives.emplace(key, value);
                result.at(i, j) = value;
            }
        return result;
    }

    Primitive(const NurbsSurface& surface, const std::array<double, 4>& trim, size_t remaining,
              const std::function<void()>& charge, const std::function<bool(double, size_t)>& permit = {}) {
        const int p = surface.degree(0), q = surface.degree(1);
        if (surface.is_rational() || p < 1 || q < 1 || p > 3 || q > 3)
            return;
        const auto crop = [&](int d, double lo, double hi) {
            const auto all = surface.get_span_vector(d);
            if (all.size() < 2 || !std::isfinite(lo) || !std::isfinite(hi) || lo < all.front() ||
                hi > all.back() || lo > hi)
                return std::vector<double>();
            size_t first = 0, last = all.size() - 2;
            while (first < last && all[first + 1] < lo)
                ++first;
            while (last > first && all[last] > hi)
                --last;
            return std::vector<double>(all.begin() + first, all.begin() + last + 2);
        };
        u = crop(0, trim[0], trim[1]);
        v = crop(1, trim[2], trim[3]);
        if (u.size() < 2 || v.size() < 2)
            return;
        shared = c1(surface) && p > 1 && q > 1;
        const size_t nu = u.size() - 1, nv = v.size() - 1;
        // Degrees are at most three. Check products before allocating or charging.
        if (nu > remaining || nv > remaining || nu > remaining / nv)
            return;
        const size_t count = nu * nv;
        const size_t derivative_count =
            shared ? 3 * ((nu * (p - 1) + 1) * (nv * q + 1) + (nu * p + 1) * (nv * (q - 1) + 1))
                   : count * (6 * p * q + 3 * p + 3 * q);
        const size_t fixed = count * 48 * p * q;
        if (fixed > remaining || derivative_count > remaining - fixed)
            return;
        if (permit && !permit(u.front(), fixed + derivative_count))
            return;
        for (size_t j = 1; j < v.size(); ++j) {
            std::vector<Interval> prefix(3 * q);
            for (size_t i = 1; i < u.size(); ++i) {
                const std::array<double, 4> cell = {u[i - 1], u[i], v[j - 1], v[j]};
                const auto controls = nurbs_interval::surface_controls(surface, cell, {0, 0, 0});
                if (controls.empty())
                    throw std::runtime_error("Native plane volume could not enclose a support knot cell");
                std::vector<Grid> point, du, dv;
                for (int d = 0; d < 3; ++d) {
                    point.emplace_back(p, q);
                    for (size_t k = 0; k < controls.size(); ++k)
                        point.back().values[k] = controls[k][d];
                    du.push_back(derivative(point.back(), d, 0, Interval(u[i]) - Interval(u[i - 1]), i - 1, j - 1, charge));
                    dv.push_back(derivative(point.back(), d, 1, Interval(v[j]) - Interval(v[j - 1]), i - 1, j - 1, charge));
                }
                Grid value(3 * p - 1, 3 * q - 1);
                for (int d = 0; d < 3; ++d) {
                    const int a = (d + 1) % 3, b = (d + 2) % 3;
                    const Grid term = polynomial::product(point[d], polynomial::cross_component(du[a], dv[b], du[b], dv[a], charge), charge);
                    for (size_t k = 0; k < value.values.size(); ++k)
                        value.values[k] = value.values[k] + term.values[k] / Interval(3);
                }
                cells.push_back(
                    polynomial::antiderivative(value, Interval(u[i]) - Interval(u[i - 1]), prefix, charge));
                middles.emplace_back(cells.back());
            }
        }
    }

    Interval at(double first, double second) const {
        // A boundary curve can end a rounding outside the grid it bounds: that point is the grid's edge.
        const double slack_u = (u.back() - u.front()) * 1e-12;
        const double slack_v = (v.back() - v.front()) * 1e-12;
        if (cells.empty() || !std::isfinite(first) || !std::isfinite(second) || first < u.front() - slack_u ||
            first > u.back() + slack_u || second < v.front() - slack_v || second > v.back() + slack_v)
            return {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
        first = std::clamp(first, u.front(), u.back());
        second = std::clamp(second, v.front(), v.back());
        const size_t i =
            std::min(static_cast<size_t>(std::upper_bound(u.begin(), u.end(), first) - u.begin()),
                     u.size() - 1) -
            1;
        const size_t j =
            std::min(static_cast<size_t>(std::upper_bound(v.begin(), v.end(), second) - v.begin()),
                     v.size() - 1) -
            1;
        const Interval x = (Interval(first) - Interval(u[i])) / (Interval(u[i + 1]) - Interval(u[i]));
        const Interval y = (Interval(second) - Interval(v[j])) / (Interval(v[j + 1]) - Interval(v[j]));
        const Grid& grid = cells[j * (u.size() - 1) + i];
        if (grid.u > 31 || grid.v > 31)
            return polynomial::evaluate(grid, x, y);
        const Midpoints& middle = middles[j * (u.size() - 1) + i];
        // De Casteljau in doubles at the midpoints of x and y: every step a convex combination, so the result lies within the largest
        // coefficient radius, three roundings of the largest coefficient per step, and the partials' bound times the widths of x and y.
        const double a = std::clamp(std::midpoint(x.lo, x.hi), 0.0, 1.0), b = std::clamp(std::midpoint(y.lo, y.hi), 0.0, 1.0);
        double row[32], column[32];
        for (int l = 0; l <= grid.v; ++l) {
            for (int k = 0; k <= grid.u; ++k)
                column[k] = middle.values[k * (grid.v + 1) + l];
            for (int count = grid.u; count > 0; --count)
                for (int k = 0; k < count; ++k)
                    column[k] = (1.0 - a) * column[k] + a * column[k + 1];
            row[l] = column[0];
        }
        for (int count = grid.v; count > 0; --count)
            for (int l = 0; l < count; ++l)
                row[l] = (1.0 - b) * row[l] + b * row[l + 1];
        const double bound = middle.size + middle.spread;
        const double reach = (middle.spread + 3.0 * (grid.u + grid.v + 1) * std::numeric_limits<double>::epsilon() * middle.size
            + 2.0 * grid.u * bound * std::max(x.hi - a, a - x.lo) + 2.0 * grid.v * bound * std::max(y.hi - b, b - y.lo)) * (1.0 + 1e-9);
        return nurbs_interval::outward(row[0] - reach, row[0] + reach);
    }
};
} // namespace session_cpp::brep_measure::plane
