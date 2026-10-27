#include "curve_self_intersection.h"
#include "nurbs_interval.h"
#include "spatial_rtree.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace session_cpp::curve_self_intersection {
namespace {

struct Span {
    double lo;
    double hi;
};

struct Pair {
    Span a;
    Span b;
    int depth = 0;
};

struct Bounds {
    std::array<double, 3> lo;
    std::array<double, 3> hi;
};

struct EquationBounds {
    std::array<nurbs_interval::Interval, 2> value;
    std::array<std::array<nurbs_interval::Interval, 2>, 2> derivative;
};

struct RootProof {
    bool valid = false;
    bool unique = false;
    Pair enclosure{};
};

enum class AppendState {
    Complete,
    Subdivide,
    Failed
};

std::vector<nurbs_interval::Homogeneous> controls(const NurbsCurve& curve, Span span) {
    std::vector<double> knots = curve.m_nurbsknot;
    knots.insert(knots.begin(), knots.front());
    knots.push_back(knots.back());
    const int p = curve.degree();
    const int index = std::clamp(static_cast<int>(std::upper_bound(knots.begin(), knots.end(),
        std::midpoint(span.lo, span.hi)) - knots.begin()) - 1, p, curve.cv_count() - 1);
    if (span.lo < knots[index] || span.hi > knots[index + 1])
        return {};
    const Point origin = curve.get_cv(0);
    std::vector<nurbs_interval::Homogeneous> data(p + 1);
    for (int i = 0; i <= p; ++i) {
        const double* cv = curve.cv(index - p + i);
        const double weight = curve.is_rational() ? cv[curve.dimension()] : 1.0;
        data[i][3] = weight;
        for (int d = 0; d < 3; ++d)
            data[i][d] = nurbs_interval::Interval(d < curve.dimension() ? cv[d] : 0.0) -
                nurbs_interval::Interval(origin[d]) * nurbs_interval::Interval(weight);
    }
    return nurbs_interval::blossom(data, knots, index, p, span.lo, span.hi);
}

bool span_bounds(const NurbsCurve& curve, Span span, Bounds& box) {
    const auto data = controls(curve, span);
    if (data.empty())
        return false;
    box.lo.fill(std::numeric_limits<double>::infinity());
    box.hi.fill(-std::numeric_limits<double>::infinity());
    for (const auto& cv : data) {
        if (!(cv[3].lo > 0.0))
            return false;
        for (int d = 0; d < 3; ++d) {
            const auto value = cv[d] / cv[3];
            if (!nurbs_interval::finite(value))
                return false;
            box.lo[d] = std::min(box.lo[d], value.lo);
            box.hi[d] = std::max(box.hi[d], value.hi);
        }
    }
    return true;
}

bool bounds(const NurbsCurve& curve, Span span, Bounds& box) {
    if (span.lo == span.hi)
        return span_bounds(curve, span, box);
    const auto knots = curve.get_span_vector();
    bool first = true;
    for (std::size_t i = 1; i < knots.size(); ++i) {
        const Span part{std::max(span.lo, knots[i - 1]), std::min(span.hi, knots[i])};
        if (!(part.hi > part.lo))
            continue;
        Bounds range;
        if (!span_bounds(curve, part, range))
            return false;
        for (int d = 0; d < 3; ++d) {
            box.lo[d] = first ? range.lo[d] : std::min(box.lo[d], range.lo[d]);
            box.hi[d] = first ? range.hi[d] : std::max(box.hi[d], range.hi[d]);
        }
        first = false;
    }
    return !first;
}

nurbs_interval::Interval binomial(int n, int k) {
    nurbs_interval::Interval value(1.0);
    for (int i = 1; i <= std::min(k, n - k); ++i)
        value = value * nurbs_interval::Interval(n + 1 - i) / nurbs_interval::Interval(i);
    return value;
}

bool constant_weight(const NurbsCurve& curve) {
    if (!curve.is_rational())
        return true;
    const double weight = curve.cv(0)[curve.dimension()];
    for (int i = 1; i < curve.cv_count(); ++i)
        if (curve.cv(i)[curve.dimension()] != weight)
            return false;
    return true;
}

std::vector<std::array<nurbs_interval::Interval, 3>> polynomial_derivatives(const NurbsCurve& curve, Span span) {
    const int degree = curve.degree() - 1;
    const auto& knots = curve.m_nurbsknot;
    const int index = std::clamp(static_cast<int>(std::upper_bound(knots.begin(), knots.end(), span.lo) -
        knots.begin()) - 1, degree, curve.cv_count() - 2);
    if (span.lo < knots[index] || span.hi > knots[index + 1])
        return {};
    std::vector<nurbs_interval::Homogeneous> data(degree + 1);
    for (int j = 0; j <= degree; ++j) {
        const int i = index - degree + j;
        const auto width = nurbs_interval::Interval(knots[i + degree + 1]) - nurbs_interval::Interval(knots[i]);
        if (!(width.lo > 0.0))
            return {};
        const auto factor = nurbs_interval::Interval(degree + 1) / width;
        const double* a = curve.cv(i);
        const double* b = curve.cv(i + 1);
        data[j][3] = 1.0;
        for (int d = 0; d < curve.dimension(); ++d)
            data[j][d] = factor * (nurbs_interval::Interval(b[d]) - nurbs_interval::Interval(a[d]));
    }
    data = nurbs_interval::blossom(data, knots, index, degree, span.lo, span.hi);
    std::vector<std::array<nurbs_interval::Interval, 3>> result;
    result.reserve(data.size());
    for (const auto& cv : data)
        result.push_back({cv[0], cv[1], cv[2]});
    return result;
}

std::vector<std::array<nurbs_interval::Interval, 3>> derivatives(const NurbsCurve& curve, Span span) {
    if (constant_weight(curve))
        return polynomial_derivatives(curve, span);
    const auto data = controls(curve, span);
    const int p = curve.degree();
    if (data.empty())
        return {};
    std::vector<std::array<nurbs_interval::Interval, 3>> result(p * 2);
    for (int k = 0; k < p * 2; ++k)
        for (int i = std::max(0, k - p); i <= std::min(p - 1, k); ++i) {
            const int j = k - i;
            const auto factor = binomial(p - 1, i) * binomial(p, j) / binomial(p * 2 - 1, k);
            const auto weight = data[i + 1][3] - data[i][3];
            for (int d = 0; d < 3; ++d)
                result[k][d] = result[k][d] + factor *
                    ((data[i + 1][d] - data[i][d]) * data[j][3] - weight * data[j][d]);
        }
    return result;
}

bool injective(const NurbsCurve& curve, Span span, const std::vector<double>& knots) {
    std::vector<Vector> axes = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const auto tangent = curve.evaluate(std::midpoint(span.lo, span.hi), 1);
    if (tangent.size() > 1 && std::isfinite(tangent[1].magnitude()))
        axes.push_back(tangent[1]);
    const auto left = curve.evaluate(std::nextafter(span.lo, span.hi), 1);
    const auto right = curve.evaluate(std::nextafter(span.hi, span.lo), 1);
    if (left.size() > 1 && right.size() > 1) {
        const double a = left[1].magnitude();
        const double b = right[1].magnitude();
        if (a > 0.0 && b > 0.0 && std::isfinite(a) && std::isfinite(b))
            axes.push_back(left[1] / a + right[1] / b);
    }
    std::vector<std::array<bool, 2>> signs(axes.size(), {true, true});
    bool covered = false;
    for (std::size_t i = 1; i < knots.size(); ++i) {
        const Span part{std::max(span.lo, knots[i - 1]), std::min(span.hi, knots[i])};
        if (!(part.hi > part.lo))
            continue;
        const auto data = derivatives(curve, part);
        if (data.empty())
            return false;
        std::vector<std::array<bool, 2>> strict(axes.size(), {false, false});
        for (const auto& derivative : data) {
            for (std::size_t a = 0; a < axes.size(); ++a) {
                nurbs_interval::Interval value;
                for (int d = 0; d < 3; ++d)
                    value = value + derivative[d] * nurbs_interval::Interval(axes[a][d]);
                signs[a][0] = signs[a][0] && nurbs_interval::finite(value) && value.lo >= 0.0;
                signs[a][1] = signs[a][1] && nurbs_interval::finite(value) && value.hi <= 0.0;
                strict[a][0] = strict[a][0] || value.lo > 0.0;
                strict[a][1] = strict[a][1] || value.hi < 0.0;
            }
        }
        for (size_t a = 0; a < axes.size(); ++a)
            for (int sign = 0; sign < 2; ++sign)
                signs[a][sign] = signs[a][sign] && strict[a][sign];
        covered = true;
    }
    for (const auto& sign : signs)
        if (covered && (sign[0] || sign[1]))
            return true;
    return false;
}

bool equation_part(
    const NurbsCurve& curve,
    const Pair& pair,
    const std::array<int, 2>& axes,
    EquationBounds& result,
    bool derivative
) {
    const auto a = controls(curve, pair.a);
    const auto b = controls(curve, pair.b);
    if (a.empty() || b.empty())
        return false;
    for (int axis = 0; axis < 2; ++axis) {
        const int d = axes[axis];
        for (std::size_t i = 0; i < a.size(); ++i)
            for (std::size_t j = 0; j < b.size(); ++j) {
                const auto value = a[i][d] * b[j][3] - a[i][3] * b[j][d];
                result.value[axis] = i == 0 && j == 0 ? value :
                    nurbs_interval::interval_hull(result.value[axis], value);
            }
        if (!derivative)
            continue;
        for (int direction = 0; direction < 2; ++direction) {
            const Span span = direction == 0 ? pair.a : pair.b;
            const auto width = nurbs_interval::Interval(span.hi) - nurbs_interval::Interval(span.lo);
            if (!(width.lo > 0.0))
                return false;
            const auto factor = nurbs_interval::Interval(curve.degree()) / width;
            for (std::size_t i = 0; i < a.size() - (direction == 0); ++i)
                for (std::size_t j = 0; j < b.size() - (direction == 1); ++j) {
                    const auto value = factor * (direction == 0 ?
                        (a[i + 1][d] - a[i][d]) * b[j][3] - (a[i + 1][3] - a[i][3]) * b[j][d] :
                        a[i][d] * (b[j + 1][3] - b[j][3]) - a[i][3] * (b[j + 1][d] - b[j][d]));
                    result.derivative[axis][direction] = i == 0 && j == 0 ? value :
                        nurbs_interval::interval_hull(result.derivative[axis][direction], value);
                }
        }
    }
    return true;
}

bool equations(
    const NurbsCurve& curve,
    const Pair& pair,
    const std::array<int, 2>& axes,
    EquationBounds& result,
    bool derivative = true
) {
    std::array<std::vector<double>, 2> cuts;
    for (int d = 0; d < 2; ++d) {
        const Span span = d == 0 ? pair.a : pair.b;
        cuts[d].push_back(span.lo);
        for (double knot : curve.get_span_vector())
            if (knot > span.lo && knot < span.hi)
                cuts[d].push_back(knot);
        cuts[d].push_back(span.hi);
    }
    bool first = true;
    for (std::size_t i = 1; i < cuts[0].size(); ++i)
        for (std::size_t j = 1; j < cuts[1].size(); ++j) {
            EquationBounds part;
            if (!equation_part(curve, {{cuts[0][i - 1], cuts[0][i]}, {cuts[1][j - 1], cuts[1][j]}},
                axes, part, derivative))
                return false;
            for (int axis = 0; axis < 2; ++axis) {
                result.value[axis] = first ? part.value[axis] :
                    nurbs_interval::interval_hull(result.value[axis], part.value[axis]);
                for (int direction = 0; direction < 2; ++direction)
                    result.derivative[axis][direction] = first ? part.derivative[axis][direction] :
                        nurbs_interval::interval_hull(result.derivative[axis][direction], part.derivative[axis][direction]);
            }
            first = false;
        }
    return !first;
}

RootProof root_proof(const NurbsCurve& curve, const Pair& pair, const std::array<int, 2>& axes) {
    RootProof result;
    EquationBounds range;
    if (!equations(curve, pair, axes, range))
        return result;
    const std::array<Span, 2> spans = {pair.a, pair.b};
    std::array<double, 2> center;
    std::array<double, 2> scale;
    std::array<nurbs_interval::Interval, 2> lower;
    std::array<nurbs_interval::Interval, 2> upper;
    for (int d = 0; d < 2; ++d) {
        center[d] = std::midpoint(spans[d].lo, spans[d].hi);
        scale[d] = spans[d].hi - spans[d].lo;
        if (!(scale[d] > 0.0) || !std::isfinite(scale[d]))
            return result;
        lower[d] = (nurbs_interval::Interval(spans[d].lo) - nurbs_interval::Interval(center[d])) /
            nurbs_interval::Interval(scale[d]);
        upper[d] = (nurbs_interval::Interval(spans[d].hi) - nurbs_interval::Interval(center[d])) /
            nurbs_interval::Interval(scale[d]);
    }
    EquationBounds point;
    if (!equations(curve, {{center[0], center[0]}, {center[1], center[1]}}, axes, point, false))
        return result;
    std::array<std::array<double, 2>, 2> middle;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            range.derivative[i][j] = range.derivative[i][j] * nurbs_interval::Interval(scale[j]);
            middle[i][j] = std::midpoint(range.derivative[i][j].lo, range.derivative[i][j].hi);
        }
    const double determinant = middle[0][0] * middle[1][1] - middle[0][1] * middle[1][0];
    if (!std::isfinite(determinant) || determinant == 0.0)
        return result;
    const std::array<std::array<double, 2>, 2> inverse = {{{middle[1][1] / determinant, -middle[0][1] / determinant},
        {-middle[1][0] / determinant, middle[0][0] / determinant}}};
    std::array<Span, 2> enclosure;
    result.unique = true;
    for (int i = 0; i < 2; ++i) {
        nurbs_interval::Interval image;
        nurbs_interval::Interval norm;
        for (int j = 0; j < 2; ++j) {
            if (!std::isfinite(inverse[i][j]))
                return {};
            image = image - nurbs_interval::Interval(inverse[i][j]) * point.value[j];
            nurbs_interval::Interval coefficient(i == j ? 1.0 : 0.0);
            for (int k = 0; k < 2; ++k)
                coefficient = coefficient - nurbs_interval::Interval(inverse[i][k]) * range.derivative[k][j];
            norm = norm + nurbs_interval::Interval(nurbs_interval::magnitude(coefficient));
            image = image + coefficient * nurbs_interval::Interval(lower[j].lo, upper[j].hi);
        }
        if (!nurbs_interval::finite(image) || !nurbs_interval::finite(norm))
            return {};
        result.unique = result.unique && norm.hi < 1.0 && image.lo > lower[i].hi && image.hi < upper[i].lo;
        const auto absolute = nurbs_interval::Interval(center[i]) + nurbs_interval::Interval(scale[i]) * image;
        enclosure[i] = {std::max(spans[i].lo, absolute.lo), std::min(spans[i].hi, absolute.hi)};
        if (enclosure[i].lo > enclosure[i].hi)
            return {};
    }
    result.valid = true;
    result.enclosure = {enclosure[0], enclosure[1]};
    return result;
}

bool associated(
    const NurbsCurve& curve,
    const Pair& enclosure,
    const curve_intersection::Hit& a,
    const curve_intersection::Hit& b,
    double tolerance
) {
    const std::array<Span, 2> spans = {enclosure.a, enclosure.b};
    std::array<Bounds, 2> ranges;
    if (!bounds(curve, spans[0], ranges[0]) || !bounds(curve, spans[1], ranges[1]))
        return false;
    const auto limit = nurbs_interval::Interval(tolerance) * nurbs_interval::Interval(tolerance);
    nurbs_interval::Interval gap;
    for (int d = 0; d < 3; ++d) {
        const auto residual = nurbs_interval::Interval(ranges[0].lo[d], ranges[0].hi[d]) -
            nurbs_interval::Interval(ranges[1].lo[d], ranges[1].hi[d]);
        const double distance = nurbs_interval::magnitude(residual);
        gap = gap + nurbs_interval::Interval(distance) * nurbs_interval::Interval(distance);
    }
    if (!nurbs_interval::finite(gap) || gap.hi > limit.lo)
        return false;
    for (int side = 0; side < 2; ++side) {
        const Bounds& range = ranges[side];
        for (double parameter : {side == 0 ? a.a : a.b, side == 0 ? b.a : b.b}) {
            Bounds point;
            if (!bounds(curve, {parameter, parameter}, point))
                return false;
            nurbs_interval::Interval squared;
            for (int d = 0; d < 3; ++d) {
                const auto residual = nurbs_interval::Interval(range.lo[d], range.hi[d]) -
                    nurbs_interval::Interval(point.lo[d], point.hi[d]);
                const double distance = nurbs_interval::magnitude(residual);
                squared = squared + nurbs_interval::Interval(distance) * nurbs_interval::Interval(distance);
            }
            if (!nurbs_interval::finite(squared) || squared.hi > limit.lo)
                return false;
        }
    }
    return true;
}

bool same_root(
    const NurbsCurve& curve,
    const curve_intersection::Hit& a,
    const curve_intersection::Hit& b,
    double tolerance
) {
    if (a.a == b.a && a.b == b.b)
        return true;
    const double left = std::min(a.a, b.a);
    const double right = std::max(a.b, b.b);
    const double radius = (right - left) / 1024.0;
    Pair chart{{std::max(curve.domain_start(), left - radius), std::max(a.a, b.a) + radius},
        {std::min(a.b, b.b) - radius, std::min(curve.domain_end(), right + radius)}};
    if (!(chart.a.hi < chart.b.lo))
        return false;
    for (int omitted = 0; omitted < 3; ++omitted) {
        const std::array<int, 2> axes = {(omitted + 1) % 3, (omitted + 2) % 3};
        RootProof proof = root_proof(curve, chart, axes);
        if (!proof.unique)
            continue;
        for (int iteration = 0; iteration < 8; ++iteration) {
            if (associated(curve, proof.enclosure, a, b, tolerance))
                return true;
            const RootProof next = root_proof(curve, proof.enclosure, axes);
            if (!next.valid)
                break;
            proof = next;
        }
    }
    return false;
}

bool initial(
    const NurbsCurve& curve,
    const std::vector<double>& knots,
    double tolerance,
    std::size_t limit,
    std::vector<Pair>& work
) {
    if (knots.size() - 1 > limit)
        return false;
    SpatialRTree<std::size_t, double, 3> tree;
    const double infinity = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < knots.size(); ++i) {
        const Span span{knots[i - 1], knots[i]};
        Bounds box;
        if (!bounds(curve, span, box))
            return false;
        std::array<double, 3> lo;
        std::array<double, 3> hi;
        for (int d = 0; d < 3; ++d) {
            lo[d] = std::nextafter(box.lo[d] - tolerance, -infinity);
            hi[d] = std::nextafter(box.hi[d] + tolerance, infinity);
        }
        bool within = true;
        tree.search(lo.data(), hi.data(), [&](std::size_t j) {
            within = work.size() < limit;
            if (within)
                work.push_back({{knots[j - 1], knots[j]}, span});
            return within;
        });
        if (!within || work.size() >= limit)
            return false;
        work.push_back({span, span});
        tree.insert(box.lo.data(), box.hi.data(), i);
    }
    return true;
}

bool split(const Pair& pair, std::vector<Pair>& work) {
    const bool diagonal = pair.a.lo == pair.b.lo && pair.a.hi == pair.b.hi;
    const bool first = pair.a.hi - pair.a.lo >= pair.b.hi - pair.b.lo;
    const Span part = first ? pair.a : pair.b;
    const double middle = std::midpoint(part.lo, part.hi);
    if (!(middle > part.lo && middle < part.hi))
        return false;
    const Span left{part.lo, middle};
    const Span right{middle, part.hi};
    const int depth = pair.depth + 1;
    if (diagonal) {
        work.push_back({left, left, depth});
        work.push_back({left, right, depth});
        work.push_back({right, right, depth});
    } else if (first) {
        work.push_back({left, pair.b, depth});
        work.push_back({right, pair.b, depth});
    } else {
        work.push_back({pair.a, left, depth});
        work.push_back({pair.a, right, depth});
    }
    return true;
}

AppendState append(
    const NurbsCurve& curve,
    const Pair& pair,
    const std::vector<double>& knots,
    double tolerance,
    Result& result
) {
    NurbsCurve a = curve;
    NurbsCurve b = curve;
    if (!a.trim(pair.a.lo, pair.a.hi) || !b.trim(pair.b.lo, pair.b.hi)) {
        result.failure = "Self intersection could not restrict original curve intervals";
        return AppendState::Failed;
    }
    const auto events = curve_intersection::intersect(a, b, tolerance);
    if (events.complete && events.overlaps.empty())
        for (const auto& hit : events.points)
            if (hit.a < hit.b && injective(curve, {hit.a, hit.b}, knots))
                return AppendState::Subdivide;
    for (const auto& hit : events.points) {
        if (!(hit.a < hit.b)) {
            result.failure = "Self intersection could not separate the parameter diagonal";
            return AppendState::Failed;
        }
        const double distance = curve.point_at(hit.a).distance(curve.point_at(hit.b));
        if (!std::isfinite(distance) || distance > tolerance) {
            result.failure = "Self intersection failed original-curve correspondence";
            return AppendState::Failed;
        }
        auto& destination = hit.a == curve.domain_start() && hit.b == curve.domain_end() ?
            result.endpoints : result.points;
        const auto same = [&](const auto& previous) {
            return same_root(curve, previous, hit, tolerance);
        };
        if (std::none_of(destination.begin(), destination.end(), same))
            destination.push_back(hit);
    }
    result.overlaps.insert(result.overlaps.end(), events.overlaps.begin(), events.overlaps.end());
    if (!events.complete || !events.overlaps.empty()) {
        result.failure = events.complete ? "Self intersection found a nonisolated overlap" : events.failure;
        return AppendState::Failed;
    }
    return AppendState::Complete;
}

bool separated(const NurbsCurve& curve, const Pair& pair, double tolerance) {
    Bounds a;
    Bounds b;
    if (!bounds(curve, pair.a, a) || !bounds(curve, pair.b, b))
        return false;
    for (int d = 0; d < 3; ++d)
        if ((nurbs_interval::Interval(a.hi[d]) + nurbs_interval::Interval(tolerance)).hi < b.lo[d] ||
            (nurbs_interval::Interval(b.hi[d]) + nurbs_interval::Interval(tolerance)).hi < a.lo[d])
            return true;
    return false;
}

std::string unsupported(const NurbsCurve& curve) {
    if (curve.dimension() < 2 || curve.dimension() > 3)
        return "Self intersection supports two or three geometric coordinates";
    for (int i = 0; i < curve.cv_count(); ++i)
        if (!(curve.weight(i) > 0.0) || !std::isfinite(curve.weight(i)))
            return "Self intersection requires finite positive rational weights";
    for (std::size_t i = 0; i < curve.m_nurbsknot.size();) {
        std::size_t end = i + 1;
        while (end < curve.m_nurbsknot.size() && curve.m_nurbsknot[end] == curve.m_nurbsknot[i])
            ++end;
        if (curve.m_nurbsknot[i] > curve.domain_start() && curve.m_nurbsknot[i] < curve.domain_end() &&
            end - i > static_cast<std::size_t>(curve.degree()))
            return "Self intersection requires continuity at internal knots";
        i = end;
    }
    return {};
}

}

Result intersect(const NurbsCurve& curve, double tolerance, const Options& options) {
    if (!curve.is_valid() || !(tolerance > 0.0) || !std::isfinite(tolerance) ||
        options.max_cells == 0 || options.max_depth < 0)
        throw std::invalid_argument("Self intersection requires valid geometry, tolerance and limits");
    Result result;
    result.failure = unsupported(curve);
    if (!result.failure.empty()) {
        result.complete = false;
        return result;
    }
    const auto knots = curve.get_span_vector();
    std::vector<Pair> work;
    result.complete = initial(curve, knots, tolerance, options.max_cells, work);
    while (result.complete && !work.empty()) {
        if (result.cells++ >= options.max_cells) {
            result.complete = false;
            break;
        }
        const Pair pair = work.back();
        work.pop_back();
        if (injective(curve, {pair.a.lo, pair.b.hi}, knots))
            continue;
        if (pair.a.hi < pair.b.lo) {
            if (separated(curve, pair, tolerance))
                continue;
            const auto state = append(curve, pair, knots, tolerance, result);
            if (state == AppendState::Complete)
                continue;
            if (state == AppendState::Failed) {
                result.complete = false;
                break;
            }
        }
        result.complete = pair.depth < options.max_depth && work.size() + 3 <= options.max_cells && split(pair, work);
    }
    if (!result.complete && result.failure.empty())
        result.failure = "Self intersection has unresolved singular geometry, bounds or subdivision limits";
    const auto less = [](const auto& a, const auto& b) {
        return a.a < b.a || (a.a == b.a && a.b < b.b);
    };
    std::sort(result.points.begin(), result.points.end(), less);
    std::sort(result.endpoints.begin(), result.endpoints.end(), less);
    return result;
}

}
