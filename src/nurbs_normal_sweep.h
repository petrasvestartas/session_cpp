#pragma once
#include "nurbs_interval.h"
#include <functional>
#include <limits>
#include <vector>

namespace session_cpp::normal_sweep {
using nurbs_interval::Interval;
using Box = std::array<Interval, 3>;
using Homogeneous = nurbs_interval::Homogeneous;
enum class State { Regular, Fold, Unknown };
struct Result {
    State state = State::Unknown;
    size_t cells = 0;
    std::array<double, 2> witness{0, 0};
};
struct Net {
    int u, v;
    std::vector<Homogeneous> controls;
};
inline Interval square(Interval x) {
    const double magnitude = nurbs_interval::magnitude(x);
    const double minimum = x.lo <= 0 && x.hi >= 0 ? 0 : std::min(std::abs(x.lo), std::abs(x.hi));
    return {std::max(0.0, (Interval(minimum) * Interval(minimum)).lo),
        (Interval(magnitude) * Interval(magnitude)).hi};
}
inline Interval dot(const Box& a, const Box& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline Interval norm_squared(const Box& a) {
    return square(a[0]) + square(a[1]) + square(a[2]);
}
inline Box cross(const Box& a, const Box& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]};
}
inline Net derivative(const Net& source, int axis, Interval width) {
    const int degree = axis == 0 ? source.u : source.v;
    if (degree == 0)
        return {0, 0, std::vector<Homogeneous>(1)};
    Net result{source.u - (axis == 0), source.v - (axis == 1), {}};
    result.controls.resize((result.u + 1) * (result.v + 1));
    const Interval factor = Interval(degree) / width;
    for (int i = 0; i <= result.u; ++i)
        for (int j = 0; j <= result.v; ++j)
            for (int d = 0; d < 4; ++d)
                result.controls[i * (result.v + 1) + j][d] = factor *
                    (source.controls[(i + (axis == 0)) * (source.v + 1) + j + (axis == 1)][d]
                        - source.controls[i * (source.v + 1) + j][d]);
    return result;
}
inline Homogeneous hull(const Net& net) {
    Homogeneous result = net.controls.front();
    for (const Homogeneous& point : net.controls)
        for (int d = 0; d < 4; ++d)
            result[d] = nurbs_interval::interval_hull(result[d], point[d]);
    return result;
}
struct Jet {
    Box u, v, uu, uv, vv;
};
inline bool jet(const NurbsSurface& surface, const std::array<double, 4>& cell, Jet& result) {
    const Point first = surface.get_cv(0, 0);
    Net net{surface.degree(0), surface.degree(1), nurbs_interval::surface_controls(surface,
        cell, {first[0], first[1], first[2]})};
    if (net.controls.empty())
        return false;
    const Interval wu = Interval(cell[1]) - Interval(cell[0]);
    const Interval wv = Interval(cell[3]) - Interval(cell[2]);
    if (!(nurbs_interval::finite(wu) && nurbs_interval::finite(wv) && wu.lo > 0 && wv.lo > 0))
        return false;
    const Net u = derivative(net, 0, wu), v = derivative(net, 1, wv);
    const Homogeneous h = hull(net), hu = hull(u), hv = hull(v);
    const Homogeneous huu = hull(derivative(u, 0, wu));
    const Homogeneous huv = hull(derivative(u, 1, wv));
    const Homogeneous hvv = hull(derivative(v, 1, wv));
    if (!nurbs_interval::finite(h[3]) || !(h[3].lo > 0))
        return false;
    for (int d = 0; d < 3; ++d) {
        const Interval point = h[d] / h[3];
        result.u[d] = (hu[d] - point * hu[3]) / h[3];
        result.v[d] = (hv[d] - point * hv[3]) / h[3];
        result.uu[d] = (huu[d] - Interval(2) * result.u[d] * hu[3] - point * huu[3]) / h[3];
        result.uv[d] = (huv[d] - result.u[d] * hv[3] - result.v[d] * hu[3] - point * huv[3]) / h[3];
        result.vv[d] = (hvv[d] - Interval(2) * result.v[d] * hv[3] - point * hvv[3]) / h[3];
        for (Interval value : {result.u[d], result.v[d], result.uu[d], result.uv[d], result.vv[d]})
            if (!nurbs_interval::finite(value))
                return false;
    }
    return true;
}
inline State cell_state(const Jet& jet, double distance) {
    const Box normal = cross(jet.u, jet.v);
    const Interval squared = norm_squared(normal);
    if (!nurbs_interval::finite(squared) || !(squared.lo > 0))
        return State::Unknown;
    const double infinity = std::numeric_limits<double>::infinity();
    const Interval length(std::nextafter(std::sqrt(squared.lo), -infinity),
        std::nextafter(std::sqrt(squared.hi), infinity));
    const Interval a = length * norm_squared(jet.u) - Interval(distance) * dot(normal, jet.uu);
    const Interval b = length * dot(jet.u, jet.v) - Interval(distance) * dot(normal, jet.uv);
    const Interval c = length * norm_squared(jet.v) - Interval(distance) * dot(normal, jet.vv);
    const Interval determinant = a * c - square(b);
    if (!nurbs_interval::finite(a) || !nurbs_interval::finite(b)
        || !nurbs_interval::finite(c) || !nurbs_interval::finite(determinant))
        return State::Unknown;
    if (a.lo > 0 && determinant.lo > 0)
        return State::Regular;
    if (a.hi < 0 || c.hi < 0 || determinant.hi < 0)
        return State::Fold;
    return State::Unknown;
}
inline Result check(const NurbsSurface& surface, const std::array<double, 4>& rectangle,
    double distance, size_t max_cells, const std::function<bool(double, double)>& original_trim_inside) {
    Result result;
    if (!surface.is_valid() || surface.dimension() != 3 || !std::isfinite(distance) || !max_cells)
        return result;
    std::array<std::vector<double>, 2> knots;
    bool continuous = true;
    for (int axis = 0; axis < 2; ++axis) {
        const std::pair<double, double> domain = surface.domain(axis);
        if (surface.degree(axis) < 1 || surface.degree(axis) > 21
            || !std::isfinite(rectangle[2 * axis]) || !std::isfinite(rectangle[2 * axis + 1])
            || rectangle[2 * axis] < domain.first || rectangle[2 * axis + 1] > domain.second
            || !(rectangle[2 * axis] < rectangle[2 * axis + 1]))
            return result;
        const std::vector<double>& stored = surface.m_nurbsknot[axis];
        for (size_t first = 0; first < stored.size();) {
            size_t last = first + 1;
            while (last < stored.size() && stored[last] == stored[first])
                ++last;
            if (stored[first] > rectangle[2 * axis] && stored[first] < rectangle[2 * axis + 1]
                && last - first >= static_cast<size_t>(surface.degree(axis)))
                continuous = false;
            first = last;
        }
        knots[axis] = {rectangle[2 * axis]};
        for (double knot : surface.get_span_vector(axis))
            if (knot > rectangle[2 * axis] && knot < rectangle[2 * axis + 1])
                knots[axis].push_back(knot);
        knots[axis].push_back(rectangle[2 * axis + 1]);
    }
    if (knots[0].size() - 1 > max_cells / (knots[1].size() - 1))
        return result;
    std::vector<std::array<double, 4>> pending;
    for (size_t i = 1; i < knots[0].size(); ++i)
        for (size_t j = 1; j < knots[1].size(); ++j)
            pending.push_back({knots[0][i - 1], knots[0][i], knots[1][j - 1], knots[1][j]});
    while (!pending.empty()) {
        if (result.cells == max_cells)
            return result;
        const std::array<double, 4> cell = pending.back();
        pending.pop_back();
        ++result.cells;
        Jet bounds;
        const State state = jet(surface, cell, bounds) ? cell_state(bounds, distance) : State::Unknown;
        if (state == State::Regular)
            continue;
        const double u = cell[0] * .5 + cell[1] * .5, v = cell[2] * .5 + cell[3] * .5;
        if (state == State::Fold && u > cell[0] && u < cell[1] && v > cell[2] && v < cell[3]
            && original_trim_inside && original_trim_inside(u, v)) {
            result.state = State::Fold;
            result.witness = {u, v};
            return result;
        }
        const int axis = (cell[1] - cell[0]) / (rectangle[1] - rectangle[0])
            >= (cell[3] - cell[2]) / (rectangle[3] - rectangle[2]) ? 0 : 1;
        const double middle = axis == 0 ? u : v;
        if (!(middle > cell[2 * axis] && middle < cell[2 * axis + 1]))
            return result;
        std::array<double, 4> first = cell, second = cell;
        first[2 * axis + 1] = middle;
        second[2 * axis] = middle;
        pending.push_back(second);
        pending.push_back(first);
    }
    result.state = continuous ? State::Regular : State::Unknown;
    return result;
}
}
