#pragma once

#include "nurbs_interval.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace session_cpp::nurbs_implicit {

inline constexpr int degree_limit = 32;
inline constexpr double rounding = std::numeric_limits<double>::epsilon();

/// Bernstein coefficients of a polynomial on the unit square, (m + 1) rows of (n + 1), each within `error` of its exact value.
struct Bernstein {
    int m = 0, n = 0;
    std::vector<double> c;
    double error = 0.0;

    Bernstein() = default;
    Bernstein(int m, int n) : m(m), n(n), c(static_cast<size_t>(m + 1) * (n + 1)) {}

    double& at(int i, int j) { return c[i * (n + 1) + j]; }
    double at(int i, int j) const { return c[i * (n + 1) + j]; }

    double peak() const {
        double value = 0.0;
        for (double x : c)
            value = std::max(value, std::abs(x));
        return value;
    }
};

inline double binomial(int n, int k) {
    static const std::array<std::array<double, degree_limit + 1>, degree_limit + 1> table = [] {
        std::array<std::array<double, degree_limit + 1>, degree_limit + 1> t{};
        for (int i = 0; i <= degree_limit; ++i) {
            t[i][0] = 1.0;
            for (int j = 1; j <= i; ++j)
                t[i][j] = t[i - 1][j - 1] + (j < i ? t[i - 1][j] : 0.0);
        }
        return t;
    }();
    return table[n][k];
}

/// Weighted sum of polynomials of one degree.
inline Bernstein sum(std::initializer_list<std::pair<double, const Bernstein*>> terms) {
    Bernstein result(terms.begin()->second->m, terms.begin()->second->n);
    double scale = 0.0;
    for (const auto& [weight, term] : terms) {
        for (size_t i = 0; i < result.c.size(); ++i)
            result.c[i] += weight * term->c[i];
        result.error += std::abs(weight) * term->error;
        scale += std::abs(weight) * term->peak();
    }
    result.error += (terms.size() + 2) * rounding * scale;
    return result;
}

/// Product of two polynomials, of the summed degrees: every coefficient is a convex combination of products of theirs.
inline Bernstein product(const Bernstein& a, const Bernstein& b) {
    Bernstein result(a.m + b.m, a.n + b.n);
    std::vector<double> scaled(b.c.size());
    for (int k = 0; k <= b.m; ++k)
        for (int l = 0; l <= b.n; ++l)
            scaled[k * (b.n + 1) + l] = binomial(b.m, k) * binomial(b.n, l) * b.at(k, l);
    for (int i = 0; i <= a.m; ++i)
        for (int j = 0; j <= a.n; ++j) {
            const double x = binomial(a.m, i) * binomial(a.n, j) * a.at(i, j);
            for (int k = 0; k <= b.m; ++k)
                for (int l = 0; l <= b.n; ++l)
                    result.at(i + k, j + l) += x * scaled[k * (b.n + 1) + l];
        }
    for (int i = 0; i <= result.m; ++i)
        for (int j = 0; j <= result.n; ++j)
            result.at(i, j) /= binomial(result.m, i) * binomial(result.n, j);
    const double first = a.peak(), second = b.peak();
    const int terms = (std::min(a.m, b.m) + 1) * (std::min(a.n, b.n) + 1);
    result.error = a.error * (second + b.error) + b.error * first + (terms + 8) * rounding * first * second;
    return result;
}

/// A plane, sphere, cylinder, cone or torus as the zero set of a polynomial: kind 1 to 5, a point (on the plane, the centre, on the axis, the apex), the unit normal or axis, the radius (half angle of a cone, major radius of a torus) and the minor radius.
struct Implicit {
    int kind = 0;
    std::array<double, 3> point{}, axis{};
    double radius = 0.0, minor = 0.0;

    int degree() const { return kind == 1 ? 1 : kind == 5 ? 4 : 2; }
};

/// The plane, sphere, cylinder, cone or torus samples of the surface fit within tolerance, kind 0 where none does: a candidate, confirmed only by a bound of its form on the surface.
Implicit recognize(const NurbsSurface& surface, double tolerance);

/// The implicit polynomial of homogeneous coordinates h = (x w, y w, z w, w) about `origin`, times a power of w: zero exactly where the patch meets the plane, sphere, cylinder, double cone or torus.
inline Bernstein compose(const std::array<Bernstein, 4>& h, const Implicit& f, const std::array<double, 3>& origin) {
    std::array<Bernstein, 3> d;
    for (int k = 0; k < 3; ++k)
        d[k] = sum({{1.0, &h[k]}, {origin[k] - f.point[k], &h[3]}});
    const Bernstein along = sum({{f.axis[0], &d[0]}, {f.axis[1], &d[1]}, {f.axis[2], &d[2]}});
    if (f.kind == 1)
        return along;
    const Bernstein x = product(d[0], d[0]), y = product(d[1], d[1]), z = product(d[2], d[2]);
    const Bernstein square = sum({{1.0, &x}, {1.0, &y}, {1.0, &z}}), weight = product(h[3], h[3]);
    if (f.kind == 2)
        return sum({{1.0, &square}, {-f.radius * f.radius, &weight}});
    const Bernstein height = product(along, along);
    if (f.kind == 3)
        return sum({{1.0, &square}, {-1.0, &height}, {-f.radius * f.radius, &weight}});
    if (f.kind == 4) {
        const double cosine = std::cos(f.radius);
        return sum({{1.0, &height}, {-cosine * cosine, &square}});
    }
    const double major = f.radius * f.radius;
    const Bernstein shell = sum({{1.0, &square}, {major - f.minor * f.minor, &weight}});
    const Bernstein ring = sum({{1.0, &square}, {-1.0, &height}});
    const Bernstein first = product(shell, shell), second = product(weight, ring);
    return sum({{1.0, &first}, {-4.0 * major, &second}});
}

/// The two halves of a polynomial cut at the middle of its first (direction 0) or second parameter.
inline void halve(const Bernstein& f, int direction, Bernstein& low, Bernstein& high) {
    low = high = f;
    const int count = direction ? f.n : f.m, lines = direction ? f.m : f.n;
    const int stride = direction ? 1 : f.n + 1, step = direction ? f.n + 1 : 1;
    std::array<double, degree_limit + 1> t;
    for (int line = 0; line <= lines; ++line) {
        for (int i = 0; i <= count; ++i)
            t[i] = f.c[line * step + i * stride];
        for (int k = 0; k <= count; ++k) {
            low.c[line * step + k * stride] = t[0];
            high.c[line * step + (count - k) * stride] = t[count - k];
            for (int i = 0; i < count - k; ++i)
                t[i] = 0.5 * (t[i] + t[i + 1]);
        }
    }
    low.error = high.error = f.error + count * rounding * f.peak();
}

/// The polynomial on [lo, lo + share (1 - lo)] of one parameter, again on [0, 1]: the part past lo, then the first share of what is left.
inline Bernstein part(const Bernstein& f, int direction, double lo, double share) {
    Bernstein result = f;
    const int count = direction ? f.n : f.m, lines = direction ? f.m : f.n;
    const int stride = direction ? 1 : f.n + 1, step = direction ? f.n + 1 : 1;
    std::array<double, degree_limit + 1> t{};
    for (int line = 0; line <= lines; ++line) {
        for (int i = 0; i <= count; ++i)
            t[i] = f.c[line * step + i * stride];
        for (int k = 1; k <= count; ++k)
            for (int i = 0; i + k <= count; ++i)
                t[i] += lo * (t[i + 1] - t[i]);
        result.c[line * step] = t[0];
        for (int k = 1; k <= count; ++k) {
            for (int i = 0; i + k <= count; ++i)
                t[i] += share * (t[i + 1] - t[i]);
            result.c[line * step + k * stride] = t[0];
        }
    }
    result.error = f.error + 6 * count * rounding * f.peak();
    return result;
}

/// An enclosure of the polynomial where its parameters lie in s and t, taken within the square: its value at their middles, widened by the rounding and by its slopes across them.
inline nurbs_interval::Interval value(const Bernstein& f, nurbs_interval::Interval s, nurbs_interval::Interval t) {
    const std::array<double, 4> box = {std::clamp(s.lo, 0.0, 1.0), std::clamp(s.hi, 0.0, 1.0), std::clamp(t.lo, 0.0, 1.0), std::clamp(t.hi, 0.0, 1.0)};
    const double x = std::midpoint(box[0], box[1]), y = std::midpoint(box[2], box[3]);
    std::array<double, degree_limit + 1> row{}, column{};
    std::array<double, 2> rise{};
    for (int i = 0; i <= f.m; ++i) {
        for (int j = 0; j <= f.n; ++j) {
            row[j] = f.at(i, j);
            if (j > 0)
                rise[1] = std::max(rise[1], std::abs(row[j] - row[j - 1]));
            if (i > 0)
                rise[0] = std::max(rise[0], std::abs(row[j] - f.at(i - 1, j)));
        }
        for (int k = f.n; k > 0; --k)
            for (int j = 0; j < k; ++j)
                row[j] += y * (row[j + 1] - row[j]);
        column[i] = row[0];
    }
    for (int k = f.m; k > 0; --k)
        for (int i = 0; i < k; ++i)
            column[i] += x * (column[i + 1] - column[i]);
    // Every term is positive and rounded a few times: a part in 1e14 more covers that.
    const double spread = (f.error + 3.0 * (f.m + f.n) * rounding * f.peak() + f.m * (rise[0] + 2.0 * f.error) * std::max(x - box[0], box[1] - x) +
        f.n * (rise[1] + 2.0 * f.error) * std::max(y - box[2], box[3] - y)) * (1.0 + 64.0 * rounding);
    return nurbs_interval::outward(column[0] - spread, column[0] + spread);
}

/// Enclosures, about the origin, of the points of a patch in homogeneous coordinates and of their derivatives along each parameter of the square: false where a weight is not proved positive.
inline bool bounds(const std::array<Bernstein, 4>& h, std::array<nurbs_interval::Interval, 3>& point,
    std::array<std::array<nurbs_interval::Interval, 3>, 2>& derivative) {
    using nurbs_interval::Interval;
    const Bernstein& w = h[3];
    double light = std::numeric_limits<double>::infinity(), heavy = 0.0, slack = 0.0;
    std::array<double, 3> low, high;
    low.fill(std::numeric_limits<double>::infinity());
    high.fill(-std::numeric_limits<double>::infinity());
    for (size_t k = 0; k < w.c.size(); ++k) {
        const double least = nurbs_interval::next_down(w.c[k] - w.error);
        if (!(least > 0.0))
            return false;
        light = std::min(light, least);
        heavy = std::max(heavy, nurbs_interval::next_up(w.c[k] + w.error));
        for (int d = 0; d < 3; ++d) {
            const double ratio = h[d].c[k] / w.c[k];
            low[d] = std::min(low[d], ratio);
            high[d] = std::max(high[d], ratio);
            slack = std::max(slack, std::abs(ratio) * rounding + (h[d].error + std::abs(ratio) * w.error) / least);
        }
    }
    slack *= 1.0 + 8.0 * rounding;
    for (int d = 0; d < 3; ++d) {
        point[d] = nurbs_interval::outward(low[d] - slack, high[d] + slack);
        if (!nurbs_interval::finite(point[d]))
            return false;
    }
    const Interval weight(light, heavy);
    for (int direction = 0; direction < 2; ++direction) {
        const int count = direction ? w.n : w.m;
        Interval rate;
        for (int d = 3; d >= 0; --d) {
            double least = 0.0, most = 0.0, size = 0.0;
            for (int i = 0; i + (direction == 0) <= w.m; ++i)
                for (int j = 0; j + (direction == 1) <= w.n; ++j) {
                    const double step = h[d].at(i + (direction == 0), j + (direction == 1)) - h[d].at(i, j);
                    least = i + j == 0 ? step : std::min(least, step);
                    most = i + j == 0 ? step : std::max(most, step);
                    size = std::max(size, std::abs(step));
                }
            const double margin = 2.0 * h[d].error + rounding * size;
            const Interval change = Interval(count) * nurbs_interval::outward(least - margin, most + margin);
            if (d == 3)
                rate = change;
            else
                derivative[direction][d] = (change - point[d] * rate) / weight;
        }
    }
    return true;
}

/// The polynomial along one side of the square: direction 0 runs along the first parameter, at the start (end 0) or the end of the second.
inline Bernstein side(const Bernstein& f, int direction, int end) {
    Bernstein edge(direction ? f.n : f.m, 0);
    for (int i = 0; i <= edge.m; ++i)
        edge.c[i] = direction ? f.at(end ? f.m : 0, i) : f.at(i, end ? f.n : 0);
    edge.error = f.error;
    return edge;
}

/// Whether the polynomial is proved to rise, or to fall, along the direction over the whole square.
inline bool monotone(const Bernstein& f, int direction) {
    if ((direction ? f.n : f.m) == 0)
        return false;
    const double bound = 3.0 * f.error;
    bool rising = true, falling = true;
    for (int i = 0; i <= f.m - (direction == 0); ++i)
        for (int j = 0; j <= f.n - (direction == 1); ++j) {
            const double change = f.at(i + (direction == 0), j + (direction == 1)) - f.at(i, j);
            rising = rising && change > bound;
            falling = falling && change < -bound;
        }
    return rising || falling;
}

/// Where a polynomial of one parameter that only rises or only falls meets zero: its crossing, and either end the bound can not tell from zero. False when both ends are that close.
inline bool crossing(const Bernstein& edge, double bound, std::vector<double>& roots) {
    const double first = edge.c.front(), last = edge.c.back();
    if ((first > bound && last > bound) || (first < -bound && last < -bound))
        return true;
    if (std::abs(first) <= bound && std::abs(last) <= bound)
        return false;
    if (std::abs(first) <= bound)
        roots.push_back(0.0);
    if (std::abs(last) <= bound)
        roots.push_back(1.0);
    if ((first < 0.0) == (last < 0.0))
        return true;
    double lo = 0.0, hi = 1.0;
    for (int iteration = 0; iteration < 52; ++iteration) {
        const double t = std::midpoint(lo, hi);
        std::array<double, degree_limit + 1> w{};
        std::copy(edge.c.begin(), edge.c.end(), w.begin());
        for (int k = edge.m; k > 0; --k)
            for (int i = 0; i < k; ++i)
                w[i] += t * (w[i + 1] - w[i]);
        ((w[0] < 0.0) == (first < 0.0) ? lo : hi) = t;
    }
    roots.push_back(std::midpoint(lo, hi));
    return true;
}

/// Every zero on [0, 1] of a polynomial of one parameter that a band about zero can hold. False where neither a sign nor a slope can be proved: a touching or an edge lying in the zero set.
inline bool edge_roots(const Bernstein& edge, double band, size_t& budget, std::vector<double>& roots) {
    struct Part {
        Bernstein f;
        double lo, hi;
        int depth;
    };
    std::vector<Part> work;
    work.push_back({edge, 0.0, 1.0, 0});
    while (!work.empty()) {
        if (budget == 0)
            return false;
        --budget;
        const Part part = std::move(work.back());
        work.pop_back();
        const double bound = part.f.error + band;
        const auto [low, high] = std::minmax_element(part.f.c.begin(), part.f.c.end());
        if (*low > bound || *high < -bound)
            continue;
        if (monotone(part.f, 0)) {
            std::vector<double> found;
            if (!crossing(part.f, bound, found))
                return false;
            for (double t : found)
                roots.push_back(std::lerp(part.lo, part.hi, t));
            continue;
        }
        if (part.depth >= 48)
            return false;
        Part first{{}, part.lo, std::midpoint(part.lo, part.hi), part.depth + 1};
        Part second{{}, first.hi, part.hi, part.depth + 1};
        halve(part.f, 0, first.f, second.f);
        work.push_back(std::move(second));
        work.push_back(std::move(first));
    }
    return true;
}

/// Points of the zero set on the edges of cells the polynomial is monotone along. Where it rises or falls along one parameter the zero set is a graph over the other, so a closed curve of the zero set inside the square passes from a cell of one kind to a cell of the other, across an edge that one of them is monotone along: the edge's only zero is found. False where a cell can neither be cleared nor proved monotone: a touching, a crossing of two branches, or a patch lying in the zero set.
inline bool loop_roots(const Bernstein& f, double band, size_t& budget, std::vector<std::array<double, 2>>& roots) {
    struct Cell {
        Bernstein f;
        std::array<double, 4> box;
        int depth;
    };
    std::vector<Cell> work;
    work.push_back({f, {0.0, 1.0, 0.0, 1.0}, 0});
    while (!work.empty()) {
        if (budget == 0)
            return false;
        --budget;
        const Cell cell = std::move(work.back());
        work.pop_back();
        const double bound = cell.f.error + band;
        const auto [low, high] = std::minmax_element(cell.f.c.begin(), cell.f.c.end());
        if (*low > bound || *high < -bound)
            continue;
        if (*low >= -bound && *high <= bound)
            return false;
        const int along = monotone(cell.f, 0) ? 0 : monotone(cell.f, 1) ? 1 : -1;
        if (along >= 0) {
            for (int end = 0; end < 2; ++end) {
                std::vector<double> found;
                if (!crossing(side(cell.f, along, end), bound, found))
                    return false;
                for (double t : found) {
                    std::array<double, 2> point;
                    point[along] = std::lerp(cell.box[2 * along], cell.box[2 * along + 1], t);
                    point[1 - along] = cell.box[2 * (1 - along) + end];
                    roots.push_back(point);
                }
            }
            continue;
        }
        if (cell.depth >= 64)
            return false;
        std::array<double, 2> spread{};
        for (int i = 0; i <= cell.f.m; ++i)
            for (int j = 0; j <= cell.f.n; ++j) {
                if (i < cell.f.m)
                    spread[0] = std::max(spread[0], cell.f.m * std::abs(cell.f.at(i + 1, j) - cell.f.at(i, j)));
                if (j < cell.f.n)
                    spread[1] = std::max(spread[1], cell.f.n * std::abs(cell.f.at(i, j + 1) - cell.f.at(i, j)));
            }
        const int direction = spread[0] >= spread[1] ? 0 : 1;
        Cell first{{}, cell.box, cell.depth + 1}, second{{}, cell.box, cell.depth + 1};
        first.box[2 * direction + 1] = second.box[2 * direction] = std::midpoint(cell.box[2 * direction], cell.box[2 * direction + 1]);
        halve(cell.f, direction, first.f, second.f);
        work.push_back(std::move(second));
        work.push_back(std::move(first));
    }
    return true;
}

/// The Bezier patches of a surface as homogeneous Bernstein coordinates (x w, y w, z w, w) about one origin, taken as they are asked for.
struct Patches {
    const NurbsSurface& surface;
    std::array<double, 3> origin;
    std::array<std::vector<double>, 2> spans;
    std::array<bool, 2> closed;
    mutable std::vector<std::array<Bernstein, 4>> store;
    mutable std::vector<char> taken;

    Patches(const NurbsSurface& surface, const std::array<double, 3>& origin)
        : surface(surface), origin(origin), spans{surface.get_span_vector(0), surface.get_span_vector(1)},
          closed{surface.is_closed(0), surface.is_closed(1)} {
        store.resize(count(0) * count(1));
        taken.resize(store.size());
    }

    int count(int direction) const { return static_cast<int>(spans[direction].size()) - 1; }

    /// Null where the span has no outward-rounded Bezier form.
    const std::array<Bernstein, 4>* at(int i, int j) const {
        const size_t index = static_cast<size_t>(i) * count(1) + j;
        if (taken[index] == 0) {
            taken[index] = 2;
            const std::vector<nurbs_interval::Homogeneous> controls = nurbs_interval::surface_controls(surface,
                {spans[0][i], spans[0][i + 1], spans[1][j], spans[1][j + 1]}, origin);
            if (!controls.empty()) {
                taken[index] = 1;
                for (int d = 0; d < 4; ++d) {
                    Bernstein& h = store[index][d] = Bernstein(surface.degree(0), surface.degree(1));
                    for (size_t k = 0; k < controls.size(); ++k) {
                        h.c[k] = std::midpoint(controls[k][d].lo, controls[k][d].hi);
                        h.error = std::max(h.error, std::nextafter(std::max(controls[k][d].hi - h.c[k], h.c[k] - controls[k][d].lo),
                            std::numeric_limits<double>::infinity()));
                    }
                }
            }
        }
        return taken[index] == 1 ? &store[index] : nullptr;
    }

    std::array<double, 2> parameters(int i, int j, double s, double t) const {
        return {std::lerp(spans[0][i], spans[0][i + 1], s), std::lerp(spans[1][j], spans[1][j + 1], t)};
    }
};

/// Outer box about the origin of a patch: the lowest and highest of each coordinate over its control points.
inline std::array<double, 6> hull(const std::array<Bernstein, 4>& h) {
    const double infinity = std::numeric_limits<double>::infinity();
    std::array<double, 6> box = {infinity, infinity, infinity, -infinity, -infinity, -infinity};
    for (size_t k = 0; k < h[3].c.size(); ++k) {
        const double light = h[3].c[k] - h[3].error, heavy = h[3].c[k] + h[3].error;
        if (!(light > 0.0))
            return {-infinity, -infinity, -infinity, infinity, infinity, infinity};
        for (int d = 0; d < 3; ++d) {
            const double lo = h[d].c[k] - h[d].error, hi = h[d].c[k] + h[d].error;
            box[d] = std::min({box[d], lo / light, lo / heavy});
            box[d + 3] = std::max({box[d + 3], hi / light, hi / heavy});
        }
    }
    for (int d = 0; d < 3; ++d) {
        box[d] -= 4.0 * rounding * std::abs(box[d]);
        box[d + 3] += 4.0 * rounding * std::abs(box[d + 3]);
    }
    return box;
}

/// Gauss-Newton from uv to the surface point nearest `point`, kept in the domain; true when it ends within `reach` in every coordinate.
inline bool nearest(const Patches& patches, const std::array<double, 3>& point, double reach, std::array<double, 2>& uv) {
    const NurbsSurface& surface = patches.surface;
    for (int iteration = 0; iteration < 16; ++iteration) {
        const std::vector<Vector> e = surface.evaluate(uv[0], uv[1], 1);
        const Vector gap(e[0][0] - point[0], e[0][1] - point[1], e[0][2] - point[2]);
        const double uu = e[2].dot(e[2]), mixed = e[2].dot(e[1]), vv = e[1].dot(e[1]);
        const double determinant = uu * vv - mixed * mixed;
        if (!(determinant > 1e-24 * uu * vv))
            break;
        const std::array<double, 2> step = {(e[2].dot(gap) * vv - e[1].dot(gap) * mixed) / determinant,
            (e[1].dot(gap) * uu - e[2].dot(gap) * mixed) / determinant};
        std::array<double, 2> next;
        for (int d = 0; d < 2; ++d) {
            const double lo = patches.spans[d].front(), hi = patches.spans[d].back();
            next[d] = uv[d] - step[d];
            if (patches.closed[d] && (next[d] < lo || next[d] > hi))
                next[d] -= std::floor((next[d] - lo) / (hi - lo)) * (hi - lo);
            next[d] = std::clamp(next[d], lo, hi);
        }
        if (!std::isfinite(next[0]) || !std::isfinite(next[1]))
            return false;
        if (next == uv)
            break;
        uv = next;
    }
    const Point found = surface.point_at(uv[0], uv[1]);
    for (int d = 0; d < 3; ++d)
        if (!(std::abs(found[d] - point[d]) <= reach))
            return false;
    return true;
}

/// Parameters at which the surface passes within `reach` of a point in every coordinate: 1 found, 0 proved to pass nowhere that close, -1 undecided. Only patches whose box, about the origin, holds the point are taken apart.
inline int invert(const Patches& patches, const std::vector<std::array<double, 6>>& boxes, const std::array<double, 3>& point, double reach,
    std::array<double, 2>& uv) {
    struct Cell {
        std::array<Bernstein, 3> g;
        std::array<double, 4> box;
        int depth;
    };
    bool undecided = false;
    size_t budget = 4096;
    for (int i = 0; i < patches.count(0); ++i)
        for (int j = 0; j < patches.count(1); ++j) {
            const std::array<double, 6>& box = boxes[i * patches.count(1) + j];
            bool outside = false;
            for (int d = 0; d < 3; ++d)
                outside = outside || point[d] - patches.origin[d] < box[d] - 2.0 * reach || point[d] - patches.origin[d] > box[d + 3] + 2.0 * reach;
            if (outside)
                continue;
            const std::array<Bernstein, 4>* h = patches.at(i, j);
            if (!h)
                return -1;
            const double weight = (*h)[3].peak() + (*h)[3].error;
            std::vector<Cell> work(1);
            work[0].box = {0.0, 1.0, 0.0, 1.0};
            work[0].depth = 0;
            for (int d = 0; d < 3; ++d)
                work[0].g[d] = sum({{1.0, &(*h)[d]}, {patches.origin[d] - point[d], &(*h)[3]}});
            while (!work.empty()) {
                const Cell cell = std::move(work.back());
                work.pop_back();
                bool away = false;
                for (int d = 0; d < 3 && !away; ++d) {
                    const double bound = cell.g[d].error + reach * weight;
                    const auto [low, high] = std::minmax_element(cell.g[d].c.begin(), cell.g[d].c.end());
                    away = *low > bound || *high < -bound;
                }
                if (away)
                    continue;
                if (cell.depth >= 2 && cell.depth % 2 == 0) {
                    uv = patches.parameters(i, j, std::midpoint(cell.box[0], cell.box[1]), std::midpoint(cell.box[2], cell.box[3]));
                    if (nearest(patches, point, reach, uv))
                        return 1;
                }
                if (cell.depth >= 48 || budget == 0) {
                    undecided = true;
                    continue;
                }
                --budget;
                const int direction = cell.depth % 2;
                Cell first{{}, cell.box, cell.depth + 1}, second{{}, cell.box, cell.depth + 1};
                first.box[2 * direction + 1] = second.box[2 * direction] = std::midpoint(cell.box[2 * direction], cell.box[2 * direction + 1]);
                for (int d = 0; d < 3; ++d)
                    halve(cell.g[d], direction, first.g[d], second.g[d]);
                work.push_back(std::move(second));
                work.push_back(std::move(first));
            }
        }
    return undecided ? -1 : 0;
}

}
