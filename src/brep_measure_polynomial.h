#pragma once
#include "nurbs_interval.h"
#include <functional>
#include <stdexcept>
#include <numeric>

namespace session_cpp::brep_measure::polynomial {

using nurbs_interval::Interval;

inline bool resolved(Interval fine, Interval coarse, double tolerance) {
    return nurbs_interval::finite(fine) && nurbs_interval::finite(coarse)
        && std::abs(std::midpoint(fine.lo, fine.hi) - std::midpoint(coarse.lo, coarse.hi)) <= tolerance
        && nurbs_interval::magnitude(fine - coarse) <= tolerance;
}

struct Grid {
    int u;
    int v;
    std::vector<Interval> values;

    Grid(int first, int second) : u(first), v(second), values((first + 1) * (second + 1)) {}
    Interval& at(int i, int j) {
        return values[i * (v + 1) + j];
    }
    const Interval& at(int i, int j) const {
        return values[i * (v + 1) + j];
    }
};

inline double binomial(int n, int k) {
    double result = 1;
    for (int i = 1; i <= k; ++i)
        result = result * (n - k + i) / i;
    return result;
}

inline Grid product(const Grid& a, const Grid& b, const std::function<void()>& charge) {
    Grid result(a.u + b.u, a.v + b.v);
    for (int i = 0; i <= result.u; ++i)
        for (int j = 0; j <= result.v; ++j) {
            charge();
            for (int k = std::max(0, i - b.u); k <= std::min(a.u, i); ++k)
                for (int l = std::max(0, j - b.v); l <= std::min(a.v, j); ++l) {
                    const Interval weight = Interval(binomial(a.u, k)) * Interval(binomial(b.u, i - k))
                        / Interval(binomial(result.u, i)) * Interval(binomial(a.v, l))
                        * Interval(binomial(b.v, j - l)) / Interval(binomial(result.v, j));
                    result.at(i, j) = result.at(i, j) + weight * a.at(k, l) * b.at(i - k, j - l);
                }
        }
    return result;
}

inline Grid difference(Grid a, const Grid& b) {
    for (size_t i = 0; i < a.values.size(); ++i)
        a.values[i] = a.values[i] - b.values[i];
    return a;
}

inline Grid derivative(const Grid& source, int direction, Interval width,
                       const std::function<void()>& charge) {
    Grid result(source.u - (direction == 0), source.v - (direction == 1));
    for (int i = 0; i <= result.u; ++i)
        for (int j = 0; j <= result.v; ++j) {
            charge();
            result.at(i, j) = Interval(direction ? source.v : source.u)
                * (source.at(i + (direction == 0), j + (direction == 1)) - source.at(i, j)) / width;
        }
    return result;
}

inline Grid integrand(const NurbsSurface& surface, const std::array<double, 4>& cell,
                      const std::function<void()>& charge) {
    const std::vector<nurbs_interval::Homogeneous> controls =
        nurbs_interval::surface_controls(surface, cell, {0, 0, 0});
    if (controls.empty())
        throw std::runtime_error("Native polynomial volume could not enclose a support knot cell");
    std::vector<Grid> point, du, dv;
    for (int d = 0; d < 3; ++d) {
        point.emplace_back(surface.degree(0), surface.degree(1));
        for (size_t i = 0; i < controls.size(); ++i)
            point.back().values[i] = controls[i][d];
        du.push_back(derivative(point.back(), 0, Interval(cell[1]) - Interval(cell[0]), charge));
        dv.push_back(derivative(point.back(), 1, Interval(cell[3]) - Interval(cell[2]), charge));
    }
    Grid result(3 * surface.degree(0) - 1, 3 * surface.degree(1) - 1);
    for (int d = 0; d < 3; ++d) {
        const int a = (d + 1) % 3, b = (d + 2) % 3;
        const Grid cross = difference(product(du[a], dv[b], charge), product(du[b], dv[a], charge));
        const Grid term = product(point[d], cross, charge);
        for (size_t i = 0; i < result.values.size(); ++i)
            result.values[i] = result.values[i] + term.values[i] / Interval(3);
    }
    return result;
}

inline Grid antiderivative(const Grid& source, Interval width, std::vector<Interval>& prefix,
                           const std::function<void()>& charge) {
    Grid result(source.u + 1, source.v);
    for (int j = 0; j <= source.v; ++j) {
        result.at(0, j) = prefix[j];
        for (int i = 0; i <= source.u; ++i) {
            charge();
            result.at(i + 1, j) = result.at(i, j) + width * source.at(i, j) / Interval(source.u + 1);
        }
        prefix[j] = result.at(result.u, j);
    }
    return result;
}

inline Interval evaluate(const Grid& grid, Interval u, Interval v) {
    std::vector<Interval> row(grid.v + 1);
    for (int j = 0; j <= grid.v; ++j) {
        std::vector<Interval> column(grid.u + 1);
        for (int i = 0; i <= grid.u; ++i)
            column[i] = grid.at(i, j);
        for (int count = grid.u; count > 0; --count)
            for (int i = 0; i < count; ++i)
                column[i] = (Interval(1) - u) * column[i] + u * column[i + 1];
        row[j] = column[0];
    }
    for (int count = grid.v; count > 0; --count)
        for (int j = 0; j < count; ++j)
            row[j] = (Interval(1) - v) * row[j] + v * row[j + 1];
    return row[0];
}

struct Primitive {
    std::vector<double> u;
    std::vector<double> v;
    std::vector<Grid> cells;

    Primitive(const NurbsSurface& surface, size_t remaining, const std::function<void()>& charge)
        : u(surface.get_span_vector(0)), v(surface.get_span_vector(1)) {
        if (surface.is_rational() || surface.degree(0) < 1 || surface.degree(1) < 1
            || surface.degree(0) > 3 || surface.degree(1) > 3)
            return;
        if (u.size() < 2 || v.size() < 2)
            throw std::runtime_error("Native polynomial volume has no support knot cells");
        const size_t per_cell = 66 * surface.degree(0) * surface.degree(1)
            + 3 * surface.degree(0) + 3 * surface.degree(1);
        if (u.size() - 1 > (remaining / per_cell) / (v.size() - 1))
            throw std::runtime_error("Native BRep volume exceeded its quadrature budget");
        for (size_t j = 1; j < v.size(); ++j) {
            std::vector<Interval> prefix(3 * surface.degree(1));
            for (size_t i = 1; i < u.size(); ++i) {
                const Grid value = integrand(surface, {u[i - 1], u[i], v[j - 1], v[j]}, charge);
                cells.push_back(antiderivative(value, Interval(u[i]) - Interval(u[i - 1]), prefix, charge));
            }
        }
    }

    Interval at(double first, double second) const {
        if (cells.empty() || !std::isfinite(first) || !std::isfinite(second) || first < u.front() || first > u.back() || second < v.front() || second > v.back())
            return {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
        const size_t i = std::min(static_cast<size_t>(std::upper_bound(u.begin(), u.end(), first) - u.begin()), u.size() - 1) - 1;
        const size_t j = std::min(static_cast<size_t>(std::upper_bound(v.begin(), v.end(), second) - v.begin()), v.size() - 1) - 1;
        const Interval x = (Interval(first) - Interval(u[i])) / (Interval(u[i + 1]) - Interval(u[i]));
        const Interval y = (Interval(second) - Interval(v[j])) / (Interval(v[j + 1]) - Interval(v[j]));
        return evaluate(cells[j * (u.size() - 1) + i], x, y);
    }
};

}
