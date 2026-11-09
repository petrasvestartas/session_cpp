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

struct Primitive {
    std::vector<double> u, v;
    std::array<double,4> original_domain{};
    std::vector<Grid> cells;
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
        original_domain = {surface.domain(0).first,surface.domain(0).second,
                           surface.domain(1).first,surface.domain(1).second};
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
            shared ? 2 * ((nu * (p - 1) + 1) * (nv * q + 1) + (nu * p + 1) * (nv * (q - 1) + 1))
                   : count * (4 * p * q + 2 * p + 2 * q);
        const size_t fixed = count * 22 * p * q;
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
                    if (d < 2) {
                        du.push_back(derivative(point.back(), d, 0, Interval(u[i]) - Interval(u[i - 1]),
                                                i - 1, j - 1, charge));
                        dv.push_back(derivative(point.back(), d, 1, Interval(v[j]) - Interval(v[j - 1]),
                                                i - 1, j - 1, charge));
                    }
                }
                const Grid cross = polynomial::cross_component(du[0], dv[1], du[1], dv[0], charge);
                const Grid value = polynomial::product(point[2], cross, charge);
                cells.push_back(
                    polynomial::antiderivative(value, Interval(u[i]) - Interval(u[i - 1]), prefix, charge));
            }
        }
    }

    Interval at(double first, double second) const {
        // The original endspan polynomial remains the same law outside its raw
        // support boundary. A cropped interior boundary cannot cross omitted cells.
        if (cells.empty() || !std::isfinite(first) || !std::isfinite(second) ||
            (first < u.front() && u.front() != original_domain[0]) ||
            (first > u.back() && u.back() != original_domain[1]) ||
            (second < v.front() && v.front() != original_domain[2]) ||
            (second > v.back() && v.back() != original_domain[3]))
            return {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
        const auto cell = [](const std::vector<double>& knots, double parameter) {
            const size_t index = static_cast<size_t>(std::upper_bound(knots.begin(),knots.end(),parameter)-knots.begin());
            return std::min(std::max(size_t(1),index),knots.size()-1)-1;
        };
        const size_t i = cell(u,first), j = cell(v,second);
        return polynomial::evaluate(
            cells[j * (u.size() - 1) + i],
            (Interval(first) - Interval(u[i])) / (Interval(u[i + 1]) - Interval(u[i])),
            (Interval(second) - Interval(v[j])) / (Interval(v[j + 1]) - Interval(v[j])));
    }
};
} // namespace session_cpp::brep_measure::plane
