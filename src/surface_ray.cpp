#include "surface_ray.h"
#include "nurbs_interval.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>

namespace session_cpp::surface_ray {
namespace {

using I = nurbs_interval::Interval;
using Pair = std::array<I, 2>;
using Matrix = std::array<Pair, 2>;
using Spans = std::array<std::vector<double>, 2>;

struct Frame {
    Point origin;
    Vector direction;
    std::array<double, 3> origin_coordinates;
    std::array<I, 3> coefficients;
    std::array<int, 2> transverse;
    int dominant;
    double sign;
    I distance_scale;
    const Spans& spans;
};

struct Bounds {
    Pair value;
    Matrix derivative;
    I forward;
    std::vector<Pair> controls;
};

struct Krawczyk {
    bool valid = false;
    bool empty = false;
    bool unique = false;
    Rectangle enclosure{};
    int crossing = 0;
};

struct Cell {
    Rectangle rectangle;
    int depth;
};

struct Chart {
    Root root;
    int result_index;
};

bool finite(const Vector& value) {
    return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

bool finite(const Point& value) {
    return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

bool contains(const Rectangle& outer, const Rectangle& inner) {
    return outer[0] <= inner[0] && outer[1] >= inner[1] &&
        outer[2] <= inner[2] && outer[3] >= inner[3];
}

bool disjoint(const Rectangle& a, const Rectangle& b) {
    return a[1] < b[0] || b[1] < a[0] || a[3] < b[2] || b[3] < a[2];
}

I square(I value) {
    const double maximum = nurbs_interval::magnitude(value);
    const double minimum = value.lo <= 0.0 && value.hi >= 0.0 ? 0.0 :
        std::min(std::abs(value.lo), std::abs(value.hi));
    I result = nurbs_interval::outward(minimum * minimum, maximum * maximum);
    result.lo = std::max(0.0, result.lo);
    return result;
}

I square_root(I value) {
    if (!nurbs_interval::finite(value) || value.hi < 0.0)
        return {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    return nurbs_interval::outward(std::sqrt(std::max(0.0, value.lo)), std::sqrt(value.hi));
}

Frame make_frame(const Point& origin, const Vector& direction, const Spans& spans) {
    int k = 0;
    for (int d = 1; d < 3; ++d)
        if (std::abs(direction[d]) > std::abs(direction[k]))
            k = d;
    const double maximum = std::abs(direction[k]);
    Vector unit = direction / maximum;
    unit.normalize_self();
    Frame result{origin, unit, {origin[0], origin[1], origin[2]}, {}, {}, k,
        direction[k] > 0.0 ? 1.0 : -1.0, {}, spans};
    I squared;
    int index = 0;
    for (int d = 0; d < 3; ++d) {
        result.coefficients[d] = I(direction[d]) / I(maximum);
        squared = squared + square(result.coefficients[d]);
        if (d != k)
            result.transverse[index++] = d;
    }
    result.distance_scale = square_root(squared);
    return result;
}

Pair transverse(const nurbs_interval::Homogeneous& cv, const Frame& frame) {
    Pair result;
    for (int d = 0; d < 2; ++d) {
        const int axis = frame.transverse[d];
        result[d] = frame.coefficients[frame.dominant] * cv[axis] - frame.coefficients[axis] * cv[frame.dominant];
    }
    return result;
}

bool bounds(const NurbsSurface& surface, const Frame& frame, const Rectangle& rectangle,
    Bounds& result, bool derivatives = true) {
    std::array<std::vector<double>, 2> cuts;
    for (int d = 0; d < 2; ++d) {
        cuts[d] = {rectangle[2 * d]};
        for (double knot : frame.spans[d])
            if (knot > rectangle[2 * d] && knot < rectangle[2 * d + 1])
                cuts[d].push_back(knot);
        cuts[d].push_back(rectangle[2 * d + 1]);
    }
    bool first = true;
    const int p = surface.degree(0);
    const int q = surface.degree(1);
    for (std::size_t a = 1; a < cuts[0].size(); ++a)
        for (std::size_t b = 1; b < cuts[1].size(); ++b) {
            const Rectangle part = {cuts[0][a - 1], cuts[0][a], cuts[1][b - 1], cuts[1][b]};
            const auto controls = nurbs_interval::surface_controls(surface, part, frame.origin_coordinates);
            if (controls.empty())
                return false;
            std::vector<Pair> values;
            for (const auto& cv : controls) {
                const Pair value = transverse(cv, frame);
                const I forward = I(frame.sign) * cv[frame.dominant] / cv[3];
                if (!nurbs_interval::finite(value[0]) || !nurbs_interval::finite(value[1]) ||
                    !nurbs_interval::finite(forward))
                    return false;
                values.push_back(value);
                result.controls.push_back(value);
                for (int d = 0; d < 2; ++d)
                    result.value[d] = first ? value[d] : nurbs_interval::interval_hull(result.value[d], value[d]);
                result.forward = first ? forward : nurbs_interval::interval_hull(result.forward, forward);
                first = false;
            }
            if (!derivatives)
                continue;
            for (int direction = 0; direction < 2; ++direction) {
                const I width = I(part[2 * direction + 1]) - I(part[2 * direction]);
                if (!(width.lo > 0.0))
                    return false;
                const I factor = I(surface.degree(direction)) / width;
                bool initial = a == 1 && b == 1;
                for (int i = 0; i <= p - (direction == 0); ++i)
                    for (int j = 0; j <= q - (direction == 1); ++j) {
                        const Pair& low = values[i * (q + 1) + j];
                        const Pair& high = values[(i + (direction == 0)) * (q + 1) + j + (direction == 1)];
                        for (int d = 0; d < 2; ++d) {
                            const I value = factor * (high[d] - low[d]);
                            if (!nurbs_interval::finite(value))
                                return false;
                            result.derivative[d][direction] = initial ? value :
                                nurbs_interval::interval_hull(result.derivative[d][direction], value);
                        }
                        initial = false;
                    }
            }
        }
    return !first;
}

/// Whether the controls alone clear the ray, as the search clears a cell: all behind the origin, or all a tolerance to one side of the ray.
bool cleared(const NurbsSurface& surface, const Frame& frame, double tolerance) {
    bool behind = true;
    std::array<bool, 4> side = {true, true, true, true};
    for (int i = 0; i < surface.cv_count(0); ++i)
        for (int j = 0; j < surface.cv_count(1); ++j) {
            const double* cv = surface.cv(i, j);
            const double weight = surface.is_rational() ? cv[3] : 1.0;
            nurbs_interval::Homogeneous h;
            for (int d = 0; d < 3; ++d)
                h[d] = I(cv[d]) - I(frame.origin_coordinates[d]) * I(weight);
            h[3] = I(weight);
            const Pair value = transverse(h, frame);
            const double margin = tolerance * weight;
            behind = behind && (I(frame.sign) * h[frame.dominant]).hi < -margin;
            side = {side[0] && value[0].lo > margin, side[1] && value[0].hi < -margin, side[2] && value[1].lo > margin, side[3] && value[1].hi < -margin};
        }
    return behind || side[0] || side[1] || side[2] || side[3];
}

bool separated(const Bounds& value) {
    if (value.forward.hi < 0.0)
        return true;
    for (I coordinate : value.value)
        if (coordinate.lo > 0.0 || coordinate.hi < 0.0)
            return true;
    // Any finite axis is admissible; only its outward separating inequality decides.
    for (std::size_t i = 1; i < value.controls.size(); ++i) {
        const Pair& a = value.controls[i - 1];
        const Pair& b = value.controls[i];
        std::array<double, 2> axis = {
            std::midpoint(a[1].lo, a[1].hi) - std::midpoint(b[1].lo, b[1].hi),
            std::midpoint(b[0].lo, b[0].hi) - std::midpoint(a[0].lo, a[0].hi)};
        const double length = std::max(std::abs(axis[0]), std::abs(axis[1]));
        if (!std::isfinite(length) || length == 0.0)
            continue;
        axis[0] /= length;
        axis[1] /= length;
        bool positive = true;
        bool negative = true;
        for (const Pair& control : value.controls) {
            const I projection = I(axis[0]) * control[0] + I(axis[1]) * control[1];
            positive = positive && projection.lo > 0.0;
            negative = negative && projection.hi < 0.0;
        }
        if (positive || negative)
            return true;
    }
    return false;
}

Krawczyk krawczyk(const NurbsSurface& surface, const Frame& frame, const Rectangle& rectangle,
    const Bounds* known = nullptr) {
    Krawczyk result;
    Bounds range;
    if (known)
        range = *known;
    else if (!bounds(surface, frame, rectangle, range))
        return result;
    std::array<double, 2> center;
    std::array<double, 2> scale;
    Pair normalized;
    Pair lower;
    Pair upper;
    Rectangle at_center;
    for (int d = 0; d < 2; ++d) {
        center[d] = std::midpoint(rectangle[2 * d], rectangle[2 * d + 1]);
        scale[d] = rectangle[2 * d + 1] - rectangle[2 * d];
        if (!(scale[d] > 0.0) || !std::isfinite(scale[d]))
            return result;
        lower[d] = (I(rectangle[2 * d]) - I(center[d])) / I(scale[d]);
        upper[d] = (I(rectangle[2 * d + 1]) - I(center[d])) / I(scale[d]);
        normalized[d] = {lower[d].lo, upper[d].hi};
        at_center[2 * d] = at_center[2 * d + 1] = center[d];
    }
    Bounds point;
    if (!bounds(surface, frame, at_center, point, false))
        return result;
    Matrix jacobian;
    std::array<std::array<double, 2>, 2> middle;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            jacobian[i][j] = range.derivative[i][j] * I(scale[j]);
            middle[i][j] = std::midpoint(jacobian[i][j].lo, jacobian[i][j].hi);
        }
    const double determinant = middle[0][0] * middle[1][1] - middle[0][1] * middle[1][0];
    if (!std::isfinite(determinant) || determinant == 0.0)
        return result;
    const std::array<std::array<double, 2>, 2> inverse = {{
        {middle[1][1] / determinant, -middle[0][1] / determinant},
        {-middle[1][0] / determinant, middle[0][0] / determinant}}};
    bool inclusion = true;
    double contraction = 0.0;
    for (int i = 0; i < 2; ++i) {
        I image;
        I row_norm;
        for (int j = 0; j < 2; ++j) {
            if (!std::isfinite(inverse[i][j]))
                return result;
            image = image - I(inverse[i][j]) * point.value[j];
            I coefficient(i == j ? 1.0 : 0.0);
            for (int k = 0; k < 2; ++k)
                coefficient = coefficient - I(inverse[i][k]) * jacobian[k][j];
            row_norm = row_norm + I(nurbs_interval::magnitude(coefficient));
            image = image + coefficient * normalized[j];
        }
        if (!nurbs_interval::finite(image) || !nurbs_interval::finite(row_norm))
            return result;
        contraction = std::max(contraction, row_norm.hi);
        inclusion = inclusion && image.lo > lower[i].hi && image.hi < upper[i].lo;
        const I absolute = I(center[i]) + I(scale[i]) * image;
        result.enclosure[2 * i] = std::max(rectangle[2 * i], absolute.lo);
        result.enclosure[2 * i + 1] = std::min(rectangle[2 * i + 1], absolute.hi);
        if (result.enclosure[2 * i] > result.enclosure[2 * i + 1])
            result.empty = true;
    }
    result.valid = true;
    result.unique = !result.empty && inclusion && contraction < 1.0;
    if (result.unique) {
        // ||I-RJ|| < 1 implies det(RJ)>0 along I-t(I-RJ), so sign(det J)=sign(det R).
        const I determinant_r = I(inverse[0][0]) * I(inverse[1][1]) - I(inverse[0][1]) * I(inverse[1][0]);
        const int sign = determinant_r.lo > 0.0 ? 1 : (determinant_r.hi < 0.0 ? -1 : 0);
        result.crossing = sign * (frame.dominant % 2 == 0 ? 1 : -1) * static_cast<int>(frame.sign);
        result.unique = result.crossing != 0;
    }
    return result;
}

bool polish(const NurbsSurface& surface, const Frame& frame, const Rectangle& chart,
    double tolerance, double& u, double& v) {
    for (int iteration = 0; iteration < 32; ++iteration) {
        const auto values = surface.evaluate(u, v, 1);
        if (values.size() != 3 || !finite(values[0]) || !finite(values[1]) || !finite(values[2]))
            return false;
        const Vector residual = Point(values[0][0], values[0][1], values[0][2]) - frame.origin;
        std::array<double, 2> f;
        std::array<std::array<double, 2>, 2> j;
        for (int d = 0; d < 2; ++d) {
            const int axis = frame.transverse[d];
            f[d] = frame.direction[frame.dominant] * residual[axis] - frame.direction[axis] * residual[frame.dominant];
            for (int parameter = 0; parameter < 2; ++parameter) {
                const Vector& derivative = values[2 - parameter];
                j[d][parameter] = frame.direction[frame.dominant] * derivative[axis] -
                    frame.direction[axis] * derivative[frame.dominant];
            }
        }
        const double determinant = j[0][0] * j[1][1] - j[0][1] * j[1][0];
        if (!std::isfinite(determinant) || determinant == 0.0)
            return false;
        const double a = (j[1][1] * f[0] - j[0][1] * f[1]) / determinant;
        const double b = (j[0][0] * f[1] - j[1][0] * f[0]) / determinant;
        const double displacement = (values[2] * a + values[1] * b).magnitude();
        const double lateral = residual.cross(frame.direction).magnitude();
        if (!std::isfinite(displacement) || !std::isfinite(lateral))
            return false;
        if (displacement <= tolerance * 0.125 && lateral <= tolerance * 0.125)
            return true;
        const double next_u = std::clamp(u - a, chart[0], chart[1]);
        const double next_v = std::clamp(v - b, chart[2], chart[3]);
        if (next_u == u && next_v == v)
            return false;
        u = next_u;
        v = next_v;
    }
    return false;
}

bool root_value(const NurbsSurface& surface, const Frame& frame, const Rectangle& chart,
    Rectangle enclosure, int crossing, double tolerance, bool include_origin, Root& root) {
    for (int iteration = 0; iteration < 24; ++iteration) {
        double u = std::midpoint(enclosure[0], enclosure[1]);
        double v = std::midpoint(enclosure[2], enclosure[3]);
        if (polish(surface, frame, chart, tolerance, u, v)) {
            const Point point = surface.point_at(u, v);
            nurbs_interval::SurfaceBounds world;
            if (finite(point) && nurbs_interval::surface_bounds(surface, enclosure, frame.origin_coordinates, world, false)) {
                I squared;
                for (int d = 0; d < 3; ++d)
                    squared = squared + square(world.point[d] - (I(point[d]) - I(frame.origin[d])));
                const I error = square_root(squared);
                const I distance = I(frame.sign) * world.point[frame.dominant] * frame.distance_scale;
                if (nurbs_interval::finite(error) && error.hi <= tolerance && nurbs_interval::finite(distance) &&
                    (I(distance.hi) - I(distance.lo)).hi <= tolerance &&
                    (include_origin || distance.lo > 0.0 || distance.hi < 0.0)) {
                    root = {point, u, v, std::midpoint(distance.lo, distance.hi), enclosure, chart, {distance.lo, distance.hi}, crossing};
                    return true;
                }
            }
        }
        const Krawczyk next = krawczyk(surface, frame, enclosure);
        if (!next.valid || next.empty || next.enclosure == enclosure)
            return false;
        enclosure = next.enclosure;
    }
    return false;
}

Rectangle expand(const Rectangle& rectangle, const Rectangle& domain) {
    Rectangle result;
    for (int d = 0; d < 2; ++d) {
        const double width = rectangle[2 * d + 1] - rectangle[2 * d];
        result[2 * d] = std::max(domain[2 * d], rectangle[2 * d] - width);
        result[2 * d + 1] = std::min(domain[2 * d + 1], rectangle[2 * d + 1] + width);
    }
    return result;
}

bool append(Result& result, std::vector<Chart>& charts, Root root, bool include_origin) {
    int index = -1;
    for (const Chart& chart : charts)
        if (contains(chart.root.chart, root.enclosure) || contains(root.chart, chart.root.enclosure)) {
            index = chart.result_index;
            charts.push_back({std::move(root), index});
            return true;
        }
    for (const Root& previous : result.roots)
        if (!disjoint(previous.enclosure, root.enclosure))
            return false;
    if (root.distance_enclosure[0] > 0.0 || (include_origin && root.distance_enclosure[1] >= 0.0)) {
        index = static_cast<int>(result.roots.size());
        result.roots.push_back(root);
    }
    charts.push_back({std::move(root), index});
    return true;
}

void fail(Result& result, const std::string& reason) {
    result.complete = false;
    result.failure = reason;
}

// A bilinear patch over a box of the unit square, from its values at (0,0), (0,1), (1,0) and (1,1).
I bilinear(I c00, I c01, I c10, I c11, I s, I t) {
    return (I(1.0) - s) * ((I(1.0) - t) * c00 + t * c01) + s * ((I(1.0) - t) * c10 + t * c11);
}

// A patch of four corners without a search: a triangle (one side collapsed to a point) by the barycentric coordinates of the ray in it, any other by one Krawczyk step from the root of its affine part, which settles a parallelogram. A root at the rim, a twisted patch and a trapezoid are left to the search.
bool bilinear_roots(const NurbsSurface& surface, const Point& origin, const Vector& direction, double tolerance,
    const Options& options, Result& result) {
    if (surface.m_is_rat || surface.m_order[0] != 2 || surface.m_order[1] != 2 || surface.m_cv_count[0] != 2 ||
        surface.m_cv_count[1] != 2)
        return false;
    static const Spans none;
    const Frame frame = make_frame(origin, direction, none);
    std::array<std::array<I, 3>, 4> corners;
    Bounds cell;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            const double* cv = surface.cv(i, j);
            nurbs_interval::Homogeneous q;
            for (int d = 0; d < 3; ++d)
                q[d] = corners[2 * i + j][d] = I(cv[d]) - I(frame.origin_coordinates[d]);
            q[3] = I(1.0);
            const Pair value = transverse(q, frame);
            const I forward = I(frame.sign) * q[frame.dominant];
            if (!nurbs_interval::finite(value[0]) || !nurbs_interval::finite(value[1]) || !nurbs_interval::finite(forward))
                return false;
            for (int d = 0; d < 2; ++d)
                cell.value[d] = cell.controls.empty() ? value[d] : nurbs_interval::interval_hull(cell.value[d], value[d]);
            cell.forward = cell.controls.empty() ? forward : nurbs_interval::interval_hull(cell.forward, forward);
            cell.controls.push_back(value);
        }
    if (separated(cell))
        return true;
    for (int d = 0; d < 2; ++d)
        if (cell.value[d].lo == 0.0 && cell.value[d].hi == 0.0) {
            fail(result, "Surface ray is tangent, coincident or rank deficient");
            return true;
        }
    const std::vector<Pair>& f = cell.controls;
    const auto same = [&surface](int a, int b) {
        return std::equal(surface.cv(a / 2, a % 2), surface.cv(a / 2, a % 2) + 3, surface.cv(b / 2, b % 2));
    };
    // The box of the unit square holding the root, the sign of the Jacobian there, and whether the root lies on the rim. A value that holds zero or one puts it on that side of the patch; it is there exactly, out of reach of every enclosure, when one coordinate across the ray is zero to the bit at both corners of the side and of one strict sign at the other corners.
    Pair box;
    int sign = 0;
    bool rim = false, exact = true;
    const auto side = [&f](int a, int b, int c, int d) {
        for (int k = 0; k < 2; ++k)
            if (f[a][k].lo == 0.0 && f[a][k].hi == 0.0 && f[b][k].lo == 0.0 && f[b][k].hi == 0.0 &&
                ((f[c][k].lo > 0.0 && f[d][k].lo > 0.0) || (f[c][k].hi < 0.0 && f[d][k].hi < 0.0)))
                return true;
        return false;
    };
    // The corners are those of the side at zero and the two beyond it; with a top the side at one is the other pair.
    const auto on = [&](I value, double margin, const std::array<int, 4>& corners, bool top) {
        const bool low = value.lo <= 0.0 && value.hi >= 0.0;
        if (low || (top && value.lo <= 1.0 && value.hi >= 1.0)) {
            rim = true;
            exact = exact && (low ? side(corners[0], corners[1], corners[2], corners[3]) : side(corners[2], corners[3], corners[0], corners[1]));
            return true;
        }
        return value.lo > margin && (!top || value.hi < 1.0 - margin);
    };
    // Collapsed side, then base start, base end and apex among the corners, the parameter along the base, and whether the apex is at its parameter's start.
    const std::array<std::array<int, 7>, 4> triangles = {{{1, 3, 0, 2, 1, 0, 0}, {0, 2, 1, 3, 0, 0, 1}, {2, 3, 0, 1, 2, 1, 0}, {0, 1, 2, 3, 0, 1, 1}}};
    const auto triangle = std::find_if(triangles.begin(), triangles.end(), [&same](const auto& t) { return same(t[0], t[1]); });
    if (triangle != triangles.end()) {
        const auto cross = [](const Pair& a, const Pair& b) { return a[0] * b[1] - a[1] * b[0]; };
        const Pair& start = f[(*triangle)[2]];
        const Pair base = {f[(*triangle)[3]][0] - start[0], f[(*triangle)[3]][1] - start[1]};
        const Pair apex = {f[(*triangle)[4]][0] - start[0], f[(*triangle)[4]][1] - start[1]};
        const Pair back = {I(0.0) - start[0], I(0.0) - start[1]};
        const I area = cross(base, apex);
        if (!nurbs_interval::finite(area) || (area.lo <= 0.0 && area.hi >= 0.0))
            return false;
        const I along = cross(back, apex) / area, up = cross(base, back) / area, rest = I(1.0) - along - up;
        if (!nurbs_interval::finite(along) || !nurbs_interval::finite(up) || !nurbs_interval::finite(rest))
            return false;
        const double margin = 16.0 * std::max({along.hi - along.lo, up.hi - up.lo, rest.hi - rest.lo}) + 1e-9;
        if (along.hi < -margin || up.hi < -margin || rest.hi < -margin)
            return true;
        // Towards the apex the search meets a singular Jacobian: the last hundredth is left to it.
        const int b0 = (*triangle)[2], b1 = (*triangle)[3], top = (*triangle)[4];
        if (!(up.hi < 0.99) || !on(along, margin, {b0, top, b1, b1}, false) || !on(up, margin, {b0, b1, top, top}, false) ||
            !on(rest, margin, {b1, top, b0, b0}, false))
            return false;
        const I height = I(1.0) - up;
        const bool flipped = (*triangle)[6] != 0;
        box[(*triangle)[5]] = along / height;
        box[1 - (*triangle)[5]] = flipped ? height : up;
        sign = (area.lo > 0.0 ? 1 : -1) * (((*triangle)[5] != 0) != flipped ? -1 : 1);
    } else {
        Matrix jacobian;
        std::array<std::array<double, 2>, 2> middle;
        for (int d = 0; d < 2; ++d) {
            jacobian[d][0] = nurbs_interval::interval_hull(f[2][d] - f[0][d], f[3][d] - f[1][d]);
            jacobian[d][1] = nurbs_interval::interval_hull(f[1][d] - f[0][d], f[3][d] - f[2][d]);
            for (int k = 0; k < 2; ++k)
                middle[d][k] = std::midpoint(jacobian[d][k].lo, jacobian[d][k].hi);
        }
        const double determinant = middle[0][0] * middle[1][1] - middle[0][1] * middle[1][0];
        if (!std::isfinite(determinant) || determinant == 0.0)
            return false;
        const std::array<std::array<double, 2>, 2> inverse = {{
            {middle[1][1] / determinant, -middle[0][1] / determinant},
            {-middle[1][0] / determinant, middle[0][0] / determinant}}};
        std::array<double, 2> center;
        for (int i = 0; i < 2; ++i) {
            center[i] = std::clamp(-inverse[i][0] * std::midpoint(f[0][0].lo, f[0][0].hi)
                - inverse[i][1] * std::midpoint(f[0][1].lo, f[0][1].hi), 0.0, 1.0);
            if (!std::isfinite(center[i]))
                return false;
        }
        const Pair at_center = {bilinear(f[0][0], f[1][0], f[2][0], f[3][0], I(center[0]), I(center[1])),
            bilinear(f[0][1], f[1][1], f[2][1], f[3][1], I(center[0]), I(center[1]))};
        double contraction = 0.0;
        bool outside = false, settled = true;
        for (int i = 0; i < 2; ++i) {
            I image(center[i]);
            I row_norm;
            for (int j = 0; j < 2; ++j) {
                image = image - I(inverse[i][j]) * at_center[j];
                I coefficient(i == j ? 1.0 : 0.0);
                for (int k = 0; k < 2; ++k)
                    coefficient = coefficient - I(inverse[i][k]) * jacobian[k][j];
                row_norm = row_norm + I(nurbs_interval::magnitude(coefficient));
                image = image + coefficient * I((I(0.0) - I(center[j])).lo, (I(1.0) - I(center[j])).hi);
            }
            if (!nurbs_interval::finite(image) || !nurbs_interval::finite(row_norm))
                return false;
            // The search takes the middle of the cell for its centre: its image is wider by a few roundings.
            const double margin = 16.0 * (image.hi - image.lo) + 1e-12;
            contraction = std::max(contraction, row_norm.hi);
            outside = outside || image.hi < -margin || image.lo > 1.0 + margin;
            settled = on(image, margin, i == 0 ? std::array<int, 4>{0, 1, 2, 3} : std::array<int, 4>{0, 2, 1, 3}, true) && settled;
            box[i] = image;
        }
        if (outside)
            return true;
        const I determinant_r = I(inverse[0][0]) * I(inverse[1][1]) - I(inverse[0][1]) * I(inverse[1][0]);
        sign = determinant_r.lo > 0.0 ? 1 : (determinant_r.hi < 0.0 ? -1 : 0);
        if (!settled || !(contraction < 1.0) || sign == 0)
            return false;
    }
    Root root;
    root.chart = {surface.m_nurbsknot[0][0], surface.m_nurbsknot[0][1], surface.m_nurbsknot[1][0], surface.m_nurbsknot[1][1]};
    for (int i = 0; i < 2; ++i) {
        box[i] = {std::max(box[i].lo, 0.0), std::min(box[i].hi, 1.0)};
        const I parameter = I(root.chart[2 * i]) + (I(root.chart[2 * i + 1]) - I(root.chart[2 * i])) * box[i];
        root.enclosure[2 * i] = std::max(root.chart[2 * i], parameter.lo);
        root.enclosure[2 * i + 1] = std::min(root.chart[2 * i + 1], parameter.hi);
    }
    root.u = std::midpoint(root.enclosure[0], root.enclosure[1]);
    root.v = std::midpoint(root.enclosure[2], root.enclosure[3]);
    root.point = surface.point_at(root.u, root.v);
    I squared, forward;
    for (int d = 0; d < 3; ++d) {
        const I world = bilinear(corners[0][d], corners[1][d], corners[2][d], corners[3][d], box[0], box[1]);
        squared = squared + square(world - (I(root.point[d]) - I(frame.origin[d])));
        if (d == frame.dominant)
            forward = world;
    }
    const I error = square_root(squared);
    const I distance = I(frame.sign) * forward * frame.distance_scale;
    // A sixteenth of the tolerance keeps the bounds of the search, a few roundings wider, inside it.
    if (!finite(root.point) || !nurbs_interval::finite(error) || !(error.hi <= tolerance * 0.0625) ||
        !nurbs_interval::finite(distance) || !((I(distance.hi) - I(distance.lo)).hi <= tolerance * 0.0625))
        return false;
    if (rim) {
        // Behind the origin the search drops the cells around the root; ahead of it it halves them to its depth limit, which here comes before the limits of the parameters and of the work.
        if (distance.hi < -tolerance)
            return true;
        for (int d = 0; d < 2; ++d)
            if (!(std::ldexp(root.chart[2 * d + 1] - root.chart[2 * d], -(options.max_depth + 1) / 2) > 4.0 *
                std::numeric_limits<double>::epsilon() * std::max(std::abs(root.chart[2 * d]), std::abs(root.chart[2 * d + 1]))))
                return false;
        if (!exact || !(distance.lo > tolerance) || options.max_cells < 65536)
            return false;
        fail(result, "Surface ray root isolation depth exceeded");
        return true;
    }
    if (!(options.include_origin || distance.lo > 0.0 || distance.hi < 0.0))
        return false;
    root.distance = std::midpoint(distance.lo, distance.hi);
    root.distance_enclosure = {distance.lo, distance.hi};
    root.crossing = sign * (frame.dominant % 2 == 0 ? 1 : -1) * static_cast<int>(frame.sign);
    if (distance.lo > 0.0 || (options.include_origin && distance.hi >= 0.0))
        result.roots.push_back(std::move(root));
    return true;
}

using Polynomial = std::array<I, 5>; // Coefficients of 1, s, s squared, ...

Polynomial plus(const Polynomial& a, double weight, const Polynomial& b) {
    Polynomial result;
    for (int k = 0; k < 5; ++k)
        result[k] = a[k] + I(weight) * b[k];
    return result;
}

Polynomial times(const Polynomial& a, const Polynomial& b) {
    Polynomial result;
    for (int i = 0; i < 5; ++i)
        for (int j = 0; i + j < 5; ++j)
            result[i + j] = result[i + j] + a[i] * b[j];
    return result;
}

I value(const Polynomial& g, I s) {
    I result = g[4];
    for (int k = 3; k >= 0; --k)
        result = result * s + g[k];
    return result;
}

/// The implicit form at start + s direction about the origin of the patches, built of the double constants nurbs_implicit::compose takes.
Polynomial along_line(const nurbs_implicit::Implicit& f, const std::array<double, 3>& origin, const std::array<I, 3>& start, const Vector& direction) {
    Polynomial along, square, constant;
    for (int k = 0; k < 3; ++k) {
        const Polynomial e = {start[k] + I(origin[k] - f.point[k]), I(direction[k])};
        along = plus(along, f.axis[k], e);
        square = plus(square, 1.0, times(e, e));
    }
    constant[0] = I(1.0);
    const Polynomial height = times(along, along);
    if (f.kind == 1)
        return along;
    if (f.kind == 2)
        return plus(square, -f.radius * f.radius, constant);
    if (f.kind == 3)
        return plus(plus(square, -1.0, height), -f.radius * f.radius, constant);
    if (f.kind == 4) {
        const double cosine = std::cos(f.radius);
        return plus(height, -cosine * cosine, square);
    }
    const double major = f.radius * f.radius;
    const Polynomial shell = plus(square, major - f.minor * f.minor, constant);
    return plus(times(shell, shell), -4.0 * major, plus(square, -1.0, height));
}

/// The stretches of [near, far] where the polynomial can lie within band of zero, each on a piece it is monotone on and no longer than the band over its least slope there allows. False where a piece is neither cleared nor monotone: a double root.
bool stretches(const Polynomial& g, double band, double near, double far, std::vector<std::array<double, 2>>& found) {
    Polynomial slope;
    for (int k = 1; k < 5; ++k)
        slope[k - 1] = I(k) * g[k];
    std::vector<std::array<double, 3>> work = {{near, far, 0.0}};
    for (int budget = 256; !work.empty(); --budget) {
        if (budget == 0)
            return false;
        const auto [lo, hi, depth] = work.back();
        work.pop_back();
        const I rate = value(slope, I(lo, hi)), middle = value(g, I(std::midpoint(lo, hi))) + rate * (I(lo, hi) - I(std::midpoint(lo, hi)));
        const I range = value(g, I(lo, hi));
        if (std::max(range.lo, middle.lo) > band || std::min(range.hi, middle.hi) < -band)
            continue;
        if (rate.lo > 0.0 || rate.hi < 0.0) {
            // Monotone: its least and largest values are at the ends.
            const I first = value(g, I(rate.lo > 0.0 ? lo : hi)), last = value(g, I(rate.lo > 0.0 ? hi : lo));
            if (first.lo > band || last.hi < -band)
                continue;
            double s = std::midpoint(lo, hi);
            for (int iteration = 0; iteration < 16; ++iteration) {
                const I at = value(g, I(s)), steep = value(slope, I(s));
                const double next = std::clamp(s - std::midpoint(at.lo, at.hi) / std::midpoint(steep.lo, steep.hi), lo, hi);
                if (!std::isfinite(next) || next == s)
                    break;
                s = next;
            }
            const I reach = (I(band) + I(nurbs_interval::magnitude(value(g, I(s))))) / I(std::min(std::abs(rate.lo), std::abs(rate.hi)));
            const I low = I(s) - reach, high = I(s) + reach;
            if (!nurbs_interval::finite(low) || !nurbs_interval::finite(high))
                return false;
            found.push_back({std::max(lo, low.lo), std::min(hi, high.hi)});
            continue;
        }
        if (depth >= 64 || !(std::midpoint(lo, hi) > lo && std::midpoint(lo, hi) < hi))
            return false;
        work.push_back({std::midpoint(lo, hi), hi, depth + 1});
        work.push_back({lo, std::midpoint(lo, hi), depth + 1});
    }
    return true;
}

bool meets(const std::array<double, 6>& box, const std::vector<std::array<I, 3>>& stretches) {
    return std::any_of(stretches.begin(), stretches.end(), [&box](const std::array<I, 3>& stretch) {
        for (int k = 0; k < 3; ++k)
            if (box[k] > stretch[k].hi || stretch[k].lo > box[k + 3])
                return false;
        return true;
    });
}

/// Roots on a surface lying on a recognised form. Every root lies where the form along the ray is within its band of zero: a few short stretches of the ray. A root on a stretch ahead of the origin is found by Newton and held in a Krawczyk chart, and every Bezier cell outside the charts is proved off those stretches. False, leaving the search to decide, where the form is not confirmed, a stretch is not short or lies at the origin, a root is not certified, or a cell is not cleared.
bool form_roots(const NurbsSurface& surface, std::optional<Form>& slot, const Point& origin, const Vector& direction,
    double tolerance, const Options& options, Result& result) {
    if (!slot)
        slot.emplace(surface, nurbs_implicit::recognize(surface, tolerance));
    const Form& form = *slot;
    const nurbs_implicit::Patches& patches = form.patches;
    if (!(form.band <= form.gauge * tolerance))
        return false;
    const double infinity = std::numeric_limits<double>::infinity();
    std::array<I, 3> start;
    std::array<double, 6> box = {infinity, infinity, infinity, -infinity, -infinity, -infinity};
    for (int k = 0; k < 3; ++k) {
        start[k] = I(origin[k]) - I(patches.origin[k]);
        for (const std::array<double, 6>& part : form.boxes) {
            box[k] = std::min(box[k], part[k]);
            box[k + 3] = std::max(box[k + 3], part[k + 3]);
        }
    }
    double near = -infinity, far = infinity;
    for (int k = 0; k < 3; ++k) {
        if (direction[k] == 0.0) {
            if (start[k].hi < box[k] || start[k].lo > box[k + 3])
                return true;
            continue;
        }
        const I a = (I(box[k]) - start[k]) / I(direction[k]), b = (I(box[k + 3]) - start[k]) / I(direction[k]);
        near = std::max(near, std::min(a.lo, b.lo));
        far = std::min(far, std::max(a.hi, b.hi));
    }
    std::vector<std::array<double, 2>> zones;
    if (near > far)
        return true;
    if (!stretches(along_line(form.implicit, patches.origin, start, direction), form.band, near, far, zones))
        return false;
    I squared;
    for (int k = 0; k < 3; ++k)
        squared = squared + square(I(direction[k]));
    const I scale = square_root(squared);
    std::vector<std::array<I, 3>> ahead;
    for (const std::array<double, 2>& zone : zones) {
        const I distance = I(zone[0], zone[1]) * scale;
        if (distance.hi < -2.0 * tolerance)
            continue;
        if (!(distance.lo > 2.0 * tolerance))
            return false;
        ahead.push_back({});
        for (int k = 0; k < 3; ++k)
            ahead.back()[k] = start[k] + I(zone[0], zone[1]) * I(direction[k]);
    }
    if (ahead.empty())
        return true;
    const Frame frame = make_frame(origin, direction, patches.spans);
    const Rectangle domain = {patches.spans[0].front(), patches.spans[0].back(), patches.spans[1].front(), patches.spans[1].back()};
    std::vector<Chart> charts;
    for (size_t z = 0; z < ahead.size(); ++z) {
        std::array<double, 3> point;
        double reach = 16.0 * tolerance;
        for (int k = 0; k < 3; ++k) {
            point[k] = patches.origin[k] + std::midpoint(ahead[z][k].lo, ahead[z][k].hi);
            reach = std::max(reach, 2.0 * (ahead[z][k].hi - ahead[z][k].lo));
        }
        for (int i = 0; i < patches.count(0); ++i)
            for (int j = 0; j < patches.count(1); ++j) {
                if (!meets(form.boxes[i * patches.count(1) + j], {ahead[z]}))
                    continue;
                std::array<double, 2> uv = patches.parameters(i, j, 0.5, 0.5);
                if (!nurbs_implicit::nearest(patches, point, reach, uv) || !polish(surface, frame, domain, tolerance, uv[0], uv[1]) || std::any_of(charts.begin(), charts.end(),
                        [&uv](const Chart& chart) { return contains(chart.root.chart, {uv[0], uv[0], uv[1], uv[1]}); }))
                    continue;
                Rectangle chart;
                Krawczyk proof;
                for (int d = 0, cell; d < 2; ++d) {
                    cell = std::clamp(static_cast<int>(std::upper_bound(patches.spans[d].begin(), patches.spans[d].end(), uv[d]) - patches.spans[d].begin()) - 1, 0, patches.count(d) - 1);
                    const double half = (patches.spans[d][cell + 1] - patches.spans[d][cell]) * 0.0625;
                    chart[2 * d] = std::max(domain[2 * d], uv[d] - half);
                    chart[2 * d + 1] = std::min(domain[2 * d + 1], uv[d] + half);
                }
                for (int attempt = 0; attempt < 6 && !proof.unique; ++attempt) {
                    if (attempt > 0)
                        for (int d = 0; d < 2; ++d) {
                            const double half = (chart[2 * d + 1] - chart[2 * d]) * 0.25;
                            chart[2 * d] = std::max(domain[2 * d], uv[d] - half);
                            chart[2 * d + 1] = std::min(domain[2 * d + 1], uv[d] + half);
                        }
                    proof = krawczyk(surface, frame, chart);
                }
                if (!proof.unique)
                    return false;
                // About the polished root a box a tolerance across on the surface: its enclosure is narrow enough for root_value at once.
                const std::vector<Vector> jet = surface.evaluate(uv[0], uv[1], 1);
                Rectangle box = chart;
                for (int d = 0; d < 2 && jet.size() == 3; ++d) {
                    const double reach = tolerance / jet[2 - d].magnitude();
                    if (std::isfinite(reach)) {
                        box[2 * d] = std::max(chart[2 * d], uv[d] - reach);
                        box[2 * d + 1] = std::min(chart[2 * d + 1], uv[d] + reach);
                    }
                }
                const Krawczyk tight = krawczyk(surface, frame, box);
                Root root;
                if (!root_value(surface, frame, chart, tight.unique ? tight.enclosure : proof.enclosure, proof.crossing, tolerance, options.include_origin, root))
                    return false;
                for (const std::array<double, 2>& zone : zones) {
                    const I distance = I(zone[0], zone[1]) * scale;
                    if (root.distance_enclosure[0] <= distance.hi && distance.lo <= root.distance_enclosure[1] && !(distance.lo > 2.0 * tolerance))
                        return false;
                }
                if (!(root.distance_enclosure[0] > 0.0) || !append(result, charts, std::move(root), options.include_origin))
                    return false;
            }
    }
    struct Cell {
        std::array<nurbs_implicit::Bernstein, 4> h;
        std::array<double, 4> box;
        int i, j, depth;
    };
    std::vector<Cell> work;
    for (int i = 0; i < patches.count(0); ++i)
        for (int j = 0; j < patches.count(1); ++j)
            if (meets(form.boxes[i * patches.count(1) + j], ahead))
                work.push_back({*patches.at(i, j), {0.0, 1.0, 0.0, 1.0}, i, j, 0});
    while (!work.empty()) {
        Cell cell = std::move(work.back());
        work.pop_back();
        ++result.cells;
        Rectangle rectangle;
        for (int d = 0; d < 2; ++d) {
            const int index = d ? cell.j : cell.i;
            const I first(patches.spans[d][index]), width = I(patches.spans[d][index + 1]) - first;
            rectangle[2 * d] = (first + width * I(cell.box[2 * d])).lo;
            rectangle[2 * d + 1] = (first + width * I(cell.box[2 * d + 1])).hi;
        }
        if (std::any_of(charts.begin(), charts.end(), [&rectangle](const Chart& chart) { return contains(chart.root.chart, rectangle); }) ||
            (cell.depth > 0 && !meets(nurbs_implicit::hull(cell.h), ahead)))
            continue;
        if (cell.depth >= 24 || result.cells >= 512)
            return false;
        const int d = cell.depth % 2;
        Cell low{{}, cell.box, cell.i, cell.j, cell.depth + 1}, high = low;
        low.box[2 * d + 1] = high.box[2 * d] = std::midpoint(cell.box[2 * d], cell.box[2 * d + 1]);
        for (int k = 0; k < 4; ++k)
            nurbs_implicit::halve(cell.h[k], d, low.h[k], high.h[k]);
        work.push_back(std::move(high));
        work.push_back(std::move(low));
    }
    std::sort(result.roots.begin(), result.roots.end(), [](const Root& a, const Root& b) { return a.distance < b.distance; });
    return true;
}

std::array<double, 3> centre(const NurbsSurface& surface) {
    std::array<double, 3> lo = {surface.get_cv(0, 0)[0], surface.get_cv(0, 0)[1], surface.get_cv(0, 0)[2]}, hi = lo;
    for (int i = 0; i < surface.cv_count(0); ++i)
        for (int j = 0; j < surface.cv_count(1); ++j)
            for (int k = 0; k < 3; ++k) {
                lo[k] = std::min(lo[k], surface.get_cv(i, j)[k]);
                hi[k] = std::max(hi[k], surface.get_cv(i, j)[k]);
            }
    return {std::midpoint(lo[0], hi[0]), std::midpoint(lo[1], hi[1]), std::midpoint(lo[2], hi[2])};
}

}

Form::Form(const NurbsSurface& surface, const nurbs_implicit::Implicit& recognised) : implicit(recognised), patches(surface, centre(surface)) {
    nurbs_implicit::Implicit& f = implicit;
    const double length = std::hypot(f.axis[0], f.axis[1], f.axis[2]);
    if (f.kind < 1 || f.kind > 5 || (f.kind != 2 && !(length > 0.0)) || (f.kind != 1 && !(f.radius > 0.0)) ||
        (f.kind == 5 && !(f.minor > 0.0 && f.minor < f.radius)) || surface.dimension() != 3 ||
        f.degree() * std::max(surface.degree(0), surface.degree(1)) > nurbs_implicit::degree_limit)
        return;
    if (f.kind != 2)
        for (double& value : f.axis)
            value /= length;
    double slack = 0.0, weight = std::numeric_limits<double>::infinity();
    for (int i = 0; i < patches.count(0); ++i)
        for (int j = 0; j < patches.count(1); ++j) {
            const std::array<nurbs_implicit::Bernstein, 4>* h = patches.at(i, j);
            if (!h)
                return;
            boxes.push_back(nurbs_implicit::hull(*h));
            const nurbs_implicit::Bernstein form = nurbs_implicit::compose(*h, f, patches.origin);
            slack = std::max(slack, form.peak() + form.error);
            weight = std::min(weight, *std::min_element((*h)[3].c.begin(), (*h)[3].c.end()) - (*h)[3].error);
        }
    if (!(weight > 0.0))
        return;
    I power(1.0);
    for (int k = 0; k < f.degree(); ++k)
        power = power * I(weight);
    band = (I(slack) / power).hi;
    gauge = f.kind == 1 ? 1.0 : f.kind == 5 ? 8.0 * f.radius * (f.radius - f.minor) * f.minor : 2.0 * f.radius;
    if (f.kind == 4) {
        double reach = 0.0;
        for (int k = 0; k < 3; ++k) {
            double extent = 0.0;
            for (const std::array<double, 6>& part : boxes)
                extent = std::max({extent, std::abs(part[k] + patches.origin[k] - f.point[k]), std::abs(part[k + 3] + patches.origin[k] - f.point[k])});
            reach = std::hypot(reach, extent);
        }
        gauge = reach * std::sin(2.0 * f.radius);
    }
}

Result intersect(const NurbsSurface& surface, const Point& origin, const Vector& direction,
    double tolerance, const Options& options) {
    if (!surface.is_valid() || surface.dimension() != 3 || !finite(origin) || !finite(direction) ||
        std::max({std::abs(direction[0]), std::abs(direction[1]), std::abs(direction[2])}) == 0.0 ||
        !std::isfinite(tolerance) || tolerance <= 0.0 || options.max_cells == 0 || options.max_depth < 0)
        throw std::invalid_argument("Surface ray requires finite valid geometry, positive tolerance and a work budget");
    Result result;
    if (bilinear_roots(surface, origin, direction, tolerance, options, result)) {
        result.cells = 1;
        return result;
    }
    for (int i = 0; i < surface.cv_count(0); ++i)
        for (int j = 0; j < surface.cv_count(1); ++j)
            if (!(surface.weight(i, j) > 0.0) || !std::isfinite(surface.weight(i, j)) || !finite(surface.get_cv(i, j))) {
                fail(result, "Surface ray requires finite positive rational weights");
                return result;
            }
    const Spans spans = {surface.get_span_vector(0), surface.get_span_vector(1)};
    if (spans[0].size() < 2 || spans[1].size() < 2 ||
        spans[0].size() - 1 > options.max_cells / (spans[1].size() - 1)) {
        fail(result, "Surface ray initial knot cells exceed the work limit");
        return result;
    }
    for (int d = 0; d < 2; ++d)
        for (std::size_t i = 1; i + 1 < spans[d].size(); ++i)
            if (std::count(surface.m_nurbsknot[d].begin(), surface.m_nurbsknot[d].end(), spans[d][i]) > surface.degree(d)) {
                fail(result, "Surface ray does not certify discontinuous supports");
                return result;
            }
    const Rectangle domain = {spans[0].front(), spans[0].back(), spans[1].front(), spans[1].back()};
    const Frame frame = make_frame(origin, direction, spans);
    if (cleared(surface, frame, tolerance))
        return result;
    if (options.form) {
        Result found;
        if (form_roots(surface, *options.form, origin, direction, tolerance, options, found))
            return found;
    }
    std::vector<Cell> work;
    for (std::size_t i = 1; i < spans[0].size(); ++i)
        for (std::size_t j = 1; j < spans[1].size(); ++j)
            work.push_back({{spans[0][i - 1], spans[0][i], spans[1][j - 1], spans[1][j]}, 0});
    std::vector<Chart> charts;
    while (!work.empty()) {
        if (result.cells >= options.max_cells) {
            fail(result, "Surface ray subdivision work limit exceeded");
            break;
        }
        ++result.cells;
        const Cell cell = work.back();
        work.pop_back();
        if (std::any_of(charts.begin(), charts.end(), [&cell](const Chart& chart) { return contains(chart.root.chart, cell.rectangle); }))
            continue;
        Bounds value;
        if (!bounds(surface, frame, cell.rectangle, value)) {
            fail(result, "Surface ray original-source interval bounds failed");
            break;
        }
        if (separated(value))
            continue;
        Krawczyk proof = krawczyk(surface, frame, cell.rectangle, &value);
        if (proof.valid && proof.empty)
            continue;
        Rectangle chart = cell.rectangle;
        if (!proof.unique) {
            chart = expand(cell.rectangle, domain);
            proof = krawczyk(surface, frame, chart);
            if (proof.valid && proof.empty)
                continue;
        }
        if (proof.unique) {
            Root root;
            if (root_value(surface, frame, chart, proof.enclosure, proof.crossing, tolerance, options.include_origin, root)) {
                if (!append(result, charts, std::move(root), options.include_origin)) {
                    fail(result, "Surface ray root identity remains unresolved");
                    break;
                }
                continue;
            }
        }
        for (int d = 0; d < 2; ++d)
            if (value.value[d].lo == 0.0 && value.value[d].hi == 0.0) {
                fail(result, "Surface ray is tangent, coincident or rank deficient");
                return result;
            }
        if (cell.depth >= options.max_depth) {
            fail(result, "Surface ray root isolation depth exceeded");
            break;
        }
        const double u_width = (cell.rectangle[1] - cell.rectangle[0]) / (domain[1] - domain[0]);
        const double v_width = (cell.rectangle[3] - cell.rectangle[2]) / (domain[3] - domain[2]);
        const int d = u_width >= v_width ? 0 : 1;
        const double middle = std::midpoint(cell.rectangle[2 * d], cell.rectangle[2 * d + 1]);
        if (!(middle > cell.rectangle[2 * d] && middle < cell.rectangle[2 * d + 1])) {
            fail(result, "Surface ray subdivision reached parameter resolution");
            break;
        }
        Cell low{cell.rectangle, cell.depth + 1};
        Cell high = low;
        low.rectangle[2 * d + 1] = middle;
        high.rectangle[2 * d] = middle;
        work.push_back(high);
        work.push_back(low);
    }
    std::sort(result.roots.begin(), result.roots.end(), [](const Root& a, const Root& b) { return a.distance < b.distance; });
    return result;
}

}
