#include "curve_surface_intersection.h"
#include "closest.h"
#include "intersection.h"
#include "nurbs_interval.h"
#include "nurbs_coincidence.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace session_cpp::curve_surface_intersection {
namespace {

struct Bounds {
    Point lo;
    Point hi;
    std::array<double, 2> width = {};
    double roundoff = 0.0;
};

struct Part {
    NurbsCurve curve;
    NurbsSurface surface;
    int depth = 0;
};

struct ParameterPart {
    NurbsCurve curve;
    NurbsCurve pcurve;
    int depth = 0;
};

struct AffinePatch {
    Point origin;
    Vector x;
    Vector y;
    Vector normal;
    double xx;
    double xy;
    double yy;
    double determinant;
};

struct GraphProjection {
    Point origin;
    Vector u;
    Vector v;
};

bool finite(const Point& point) {
    return std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2]);
}

void fail(Result& result, const std::string& message) {
    result.complete = false;
    if (result.failure.empty())
        result.failure = message;
}

bool grow(Bounds& box, const Point& point, double weight) {
    if (!(weight > 0.0) || !std::isfinite(weight))
        return false;
    for (int d = 0; d < 3; ++d) {
        if (!std::isfinite(point[d]))
            return false;
        box.lo[d] = std::min(box.lo[d], point[d]);
        box.hi[d] = std::max(box.hi[d], point[d]);
        box.roundoff = std::max(box.roundoff, std::abs(point[d]) * 32.0 * std::numeric_limits<double>::epsilon());
    }
    return true;
}

bool bounds(const NurbsCurve& curve, Bounds& box) {
    box.lo = box.hi = curve.get_cv(0);
    for (int i = 0; i < curve.cv_count(); ++i)
        if (!grow(box, curve.get_cv(i), curve.weight(i)))
            return false;
    return true;
}

bool bounds(const NurbsSurface& surface, Bounds& box) {
    box.lo = box.hi = surface.get_cv(0, 0);
    for (int i = 0; i < surface.cv_count(0); ++i)
        for (int j = 0; j < surface.cv_count(1); ++j) {
            const Point point = surface.get_cv(i, j);
            if (!grow(box, point, surface.weight(i, j)))
                return false;
            if (i > 0)
                box.width[0] = std::max(box.width[0], point.distance(surface.get_cv(i - 1, j)));
            if (j > 0)
                box.width[1] = std::max(box.width[1], point.distance(surface.get_cv(i, j - 1)));
        }
    return true;
}

bool separated(const Bounds& a, const Bounds& b) {
    const double error = a.roundoff + b.roundoff;
    for (int d = 0; d < 3; ++d)
        if (a.lo[d] > b.hi[d] + error || b.lo[d] > a.hi[d] + error)
            return true;
    return false;
}

bool separated_axis(const Part& part, Vector axis, double error) {
    if (!axis.normalize_self())
        return false;

    const Point origin = part.surface.get_cv(0, 0);
    double lo = std::numeric_limits<double>::infinity();
    double hi = -lo;
    for (int i = 0; i < part.curve.cv_count(); ++i) {
        const double value = (part.curve.get_cv(i) - origin).dot(axis);
        if (!std::isfinite(value))
            return false;

        lo = std::min(lo, value);
        hi = std::max(hi, value);
    }
    double low = std::numeric_limits<double>::infinity();
    double high = -low;
    for (int i = 0; i < part.surface.cv_count(0); ++i)
        for (int j = 0; j < part.surface.cv_count(1); ++j) {
            const double value = (part.surface.get_cv(i, j) - origin).dot(axis);
            if (!std::isfinite(value))
                return false;

            low = std::min(low, value);
            high = std::max(high, value);
        }
    return lo > high + error || low > hi + error;
}

bool separated_hulls(const Part& part, double error) {
    const std::pair<double, double> u = part.surface.domain(0);
    const std::pair<double, double> v = part.surface.domain(1);
    const std::vector<Vector> surface = part.surface.evaluate(
        std::midpoint(u.first, u.second), std::midpoint(v.first, v.second), 1);
    const Vector normal = surface[2].cross(surface[1]);
    if (separated_axis(part, normal, error))
        return true;

    const Vector tangent = part.curve.evaluate(part.curve.domain_middle(), 1)[1];
    return separated_axis(part, tangent.cross(surface[1]), error) ||
        separated_axis(part, tangent.cross(surface[2]), error);
}

bool affine_patch(const NurbsSurface& surface, double error, AffinePatch& patch) {
    if (surface.cv_count(0) != 2 || surface.cv_count(1) != 2)
        return false;
    const double weight = surface.weight(0, 0);
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            if (surface.weight(i, j) != weight)
                return false;
    patch.origin = surface.get_cv(0, 0);
    patch.x = surface.get_cv(1, 0) - patch.origin;
    patch.y = surface.get_cv(0, 1) - patch.origin;
    if (surface.get_cv(1, 1).distance(patch.origin + patch.x + patch.y) > error)
        return false;
    patch.normal = patch.x.cross(patch.y);
    if (!patch.normal.normalize_self())
        return false;
    patch.xx = patch.x.dot(patch.x);
    patch.xy = patch.x.dot(patch.y);
    patch.yy = patch.y.dot(patch.y);
    patch.determinant = patch.xx * patch.yy - patch.xy * patch.xy;
    return patch.determinant > 1e-24 * patch.xx * patch.yy;
}

Point patch_uv(const AffinePatch& patch, const Point& point) {
    const Vector delta = point - patch.origin;
    const double x = delta.dot(patch.x);
    const double y = delta.dot(patch.y);
    return {(patch.yy * x - patch.xy * y) / patch.determinant,
        (patch.xx * y - patch.xy * x) / patch.determinant, 0.0};
}

Point actual_uv(const NurbsSurface& surface, const Point& normalized) {
    const std::pair<double, double> u = surface.domain(0);
    const std::pair<double, double> v = surface.domain(1);
    return {u.first + (u.second - u.first) * normalized[0],
        v.first + (v.second - v.first) * normalized[1], 0.0};
}

bool within_patch(const Point& uv) {
    const double error = 64.0 * std::numeric_limits<double>::epsilon();
    return uv[0] >= -error && uv[0] <= 1.0 + error && uv[1] >= -error && uv[1] <= 1.0 + error;
}

NurbsCurve projected_curve(const NurbsCurve& curve, const AffinePatch& patch) {
    NurbsCurve projected = curve;
    for (int i = 0; i < projected.cv_count(); ++i) {
        const Point uv = patch_uv(patch, curve.get_cv(i));
        const double weight = curve.weight(i);
        if (!projected.set_cv_4d(
            i,
            uv[0] * weight,
            uv[1] * weight,
            0.0,
            weight
        ))
            throw std::runtime_error("Curve/surface pcurve construction failed");
    }
    return projected;
}

NurbsCurve actual_pcurve(const NurbsCurve& normalized, const NurbsSurface& surface) {
    NurbsCurve pcurve = normalized;
    for (int i = 0; i < pcurve.cv_count(); ++i) {
        const Point uv = actual_uv(surface, normalized.get_cv(i));
        const double weight = normalized.weight(i);
        if (!pcurve.set_cv_4d(
            i,
            uv[0] * weight,
            uv[1] * weight,
            0.0,
            weight
        ))
            throw std::runtime_error("Curve/surface pcurve domain mapping failed");
    }
    return pcurve;
}

void append_overlap(Result& result, const NurbsCurve& pcurve) {
    const double a = pcurve.domain_start();
    const double b = pcurve.domain_end();
    result.overlaps.push_back({a, b, pcurve});
}

bool in_overlap(const Result& result, double parameter) {
    for (const Overlap& interval : result.overlaps)
        if (parameter >= interval.t0 && parameter <= interval.t1)
            return true;
    return false;
}

void clip_planar_overlap(
    Result& result,
    const NurbsCurve& curve,
    const NurbsSurface& surface,
    const AffinePatch& patch
) {
    const NurbsCurve normalized = projected_curve(curve, patch);
    std::vector<double> cuts = {curve.domain_start(), curve.domain_end()};
    for (int axis = 0; axis < 2; ++axis)
        for (double side : {0.0, 1.0}) {
            Point origin;
            Vector normal;
            origin[axis] = side;
            normal[axis] = 1.0;
            const Plane plane = Plane::from_point_normal(origin, normal);
            const std::vector<double> roots = Intersection::curve_plane(normalized, plane, 1e-12);
            cuts.insert(cuts.end(), roots.begin(), roots.end());
        }
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
    for (size_t i = 1; i < cuts.size(); ++i) {
        if (!within_patch(normalized.point_at(std::midpoint(cuts[i - 1], cuts[i]))))
            continue;
        NurbsCurve pcurve = normalized;
        if (!pcurve.trim(cuts[i - 1], cuts[i])) {
            fail(result, "Planar overlap trim failed");
            continue;
        }
        append_overlap(result, actual_pcurve(pcurve, surface));
    }
    for (double t : cuts) {
        const Point normalized_uv = normalized.point_at(t);
        if (in_overlap(result, t) || !within_patch(normalized_uv))
            continue;
        const Point uv = actual_uv(surface, normalized_uv);
        result.points.push_back({curve.point_at(t), t, uv[0], uv[1], true});
    }
}

bool regular_hull(const NurbsSurface& surface, double error) {
    const Point origin = surface.get_cv(0, 0);
    Vector direction;
    for (int i = 0; i < surface.cv_count(0); ++i)
        for (int j = 0; j < surface.cv_count(1); ++j) {
            const Vector candidate = surface.get_cv(i, j) - origin;
            if (candidate.magnitude() > direction.magnitude())
                direction = candidate;
        }
    if (!(direction.magnitude() > error))
        return false;
    for (int i = 0; i < surface.cv_count(0); ++i)
        for (int j = 0; j < surface.cv_count(1); ++j) {
            const Vector candidate = surface.get_cv(i, j) - origin;
            if (direction.cross(candidate).magnitude() > error * direction.magnitude())
                return true;
        }
    return false;
}

Point project(const NurbsSurface& surface, const Point& point) {
    const std::tuple<double, double, double> nearest = Closest::surface_point(surface, point);
    Point uv(std::get<0>(nearest), std::get<1>(nearest), 0.0);
    const std::pair<double, double> du = surface.domain(0);
    const std::pair<double, double> dv = surface.domain(1);
    for (int iteration = 0; iteration < 32; ++iteration) {
        const std::vector<Vector> values = surface.evaluate(uv[0], uv[1], 1);
        const Vector residual = Point(values[0][0], values[0][1], values[0][2]) - point;
        const Vector u = values[2] * (du.second - du.first);
        const Vector v = values[1] * (dv.second - dv.first);
        const double a = u.dot(u);
        const double b = u.dot(v);
        const double c = v.dot(v);
        const double determinant = a * c - b * b;
        if (!(determinant > 1e-24 * a * c))
            break;
        const double x = u.dot(residual);
        const double y = v.dot(residual);
        const double next_u = std::clamp(uv[0] - (c * x - b * y) / determinant * (du.second - du.first), du.first, du.second);
        const double next_v = std::clamp(uv[1] - (a * y - b * x) / determinant * (dv.second - dv.first), dv.first, dv.second);
        if (next_u == uv[0] && next_v == uv[1])
            break;
        uv = {next_u, next_v, 0.0};
    }
    return uv;
}

void intersect_planar(
    Result& result,
    const NurbsCurve& curve,
    const NurbsSurface& surface,
    const AffinePatch& patch,
    double error,
    double tolerance
) {
    const std::vector<double> spans = curve.get_span_vector();
    for (size_t i = 1; i < spans.size(); ++i) {
        NurbsCurve part = curve;
        if (!part.trim(spans[i - 1], spans[i])) {
            fail(result, "Planar curve span extraction failed");
            return;
        }
        bool coplanar = true;
        for (int j = 0; j < part.cv_count(); ++j)
            coplanar = coplanar && std::abs((part.get_cv(j) - patch.origin).dot(patch.normal)) <= error;
        if (coplanar)
            clip_planar_overlap(result, part, surface, patch);
    }
    const Plane plane = Plane::from_point_normal(patch.origin, patch.normal);
    for (double t : Intersection::curve_plane(curve, plane, tolerance)) {
        const Point point = curve.point_at(t);
        const Point normalized = patch_uv(patch, point);
        if (in_overlap(result, t) || !within_patch(normalized))
            continue;
        const Point uv = actual_uv(surface, normalized);
        const Vector tangent = curve.evaluate(t, 1)[1];
        const bool contact = std::abs(tangent.dot(patch.normal)) <= tangent.magnitude() * 1e-7;
        result.points.push_back({point, t, uv[0], uv[1], contact});
    }
    result.points.erase(std::remove_if(result.points.begin(), result.points.end(), [&result](const Hit& hit) {
        return in_overlap(result, hit.t);
    }), result.points.end());
}

bool identical_parameterization(const NurbsCurve& curve, const NurbsCurve& other, double error) {
    if (curve.order() != other.order() || curve.cv_count() != other.cv_count())
        return false;
    const double factor = curve.weight(0) / other.weight(0);
    for (int i = 0; i < curve.cv_count(); ++i) {
        const double distance = curve.get_cv(i).distance(other.get_cv(i));
        if (!std::isfinite(distance) || distance > error)
            return false;
        const double a = curve.weight(i);
        const double b = other.weight(i) * factor;
        if (std::abs(a - b) > 32.0 * std::numeric_limits<double>::epsilon() * std::max(a, b))
            return false;
    }
    const double a0 = curve.domain_start();
    const double da = curve.domain_end() - a0;
    const double b0 = other.domain_start();
    const double db = other.domain_end() - b0;
    for (int i = 0; i < curve.nurbsknot_count(); ++i)
        if (std::abs((curve.nurbsknot(i) - a0) / da - (other.nurbsknot(i) - b0) / db) > 32.0 * std::numeric_limits<double>::epsilon())
            return false;
    return true;
}

bool append_iso_overlap(
    Result& result,
    const NurbsCurve& curve,
    const NurbsSurface& surface,
    const Point& first,
    const Point& last,
    int direction,
    double error
) {
    if (!(std::abs(first[direction] - last[direction]) > 0.0))
        return false;
    NurbsCurve iso = surface.iso_curve(direction, first[1 - direction]);
    if (!iso.trim(std::min(first[direction], last[direction]), std::max(first[direction], last[direction])))
        return false;
    if (first[direction] > last[direction] && !iso.reverse())
        return false;
    if (!identical_parameterization(curve, iso, error))
        return false;
    Point end = last;
    end[1 - direction] = first[1 - direction];
    NurbsCurve pcurve = NurbsCurve::create(false, 1, {first, end});
    if (!pcurve.set_domain(curve.domain_start(), curve.domain_end())) {
        fail(result, "Isoparametric overlap domain mapping failed");
        return true;
    }
    append_overlap(result, pcurve);
    return true;
}

bool iso_overlap(
    Result& result,
    const NurbsCurve& curve,
    const NurbsSurface& surface,
    double error
) {
    const Point middle = curve.point_at(std::midpoint(curve.domain_start(), curve.domain_end()));
    const Point nearest = project(surface, middle);
    const std::array<double, 2> parameters = {nearest[0], nearest[1]};
    if (!finite(nearest) || middle.distance(surface.point_at(parameters[0], parameters[1])) > error)
        return false;
    for (int direction = 0; direction < 2; ++direction) {
        NurbsCurve iso = surface.iso_curve(direction, parameters[1 - direction]);
        for (int reversed = 0; reversed < 2; ++reversed) {
            if (reversed && !iso.reverse())
                continue;
            if (!identical_parameterization(curve, iso, error))
                continue;
            const std::pair<double, double> domain = surface.domain(direction);
            Point first(parameters[0], parameters[1], 0.0);
            Point last = first;
            first[direction] = reversed ? domain.second : domain.first;
            last[direction] = reversed ? domain.first : domain.second;
            NurbsCurve pcurve = NurbsCurve::create(false, 1, {first, last});
            if (!pcurve.set_domain(curve.domain_start(), curve.domain_end())) {
                fail(result, "Isoparametric overlap domain mapping failed");
                return true;
            }
            append_overlap(result, pcurve);
            return true;
        }
    }
    const Point first = project(surface, curve.point_at_start());
    const Point last = project(surface, curve.point_at_end());
    if (!finite(first) || !finite(last))
        return false;
    for (int direction = 0; direction < 2; ++direction)
        if (append_iso_overlap(result, curve, surface, first, last, direction, error))
            return true;
    return false;
}

bool unresolved_overlap(const NurbsCurve& curve, const NurbsSurface& surface, double error) {
    for (int i = 0; i < 7; ++i) {
        const double t = curve.domain_start() + (curve.domain_end() - curve.domain_start()) * i / 6.0;
        const Point point = curve.point_at(t);
        const Point uv = project(surface, point);
        const double distance = point.distance(surface.point_at(uv[0], uv[1]));
        if (!finite(uv) || !std::isfinite(distance) || distance > error)
            return false;
    }
    return true;
}

double greville(const NurbsSurface& surface, int direction, int index) {
    double value = 0.0;
    for (int i = 0; i < surface.degree(direction); ++i)
        value += surface.nurbsknot(direction, index + i);
    const std::pair<double, double> domain = surface.domain(direction);
    return (value / surface.degree(direction) - domain.first) / (domain.second - domain.first);
}

bool graph_projection(const NurbsSurface& surface, double error, GraphProjection& projection) {
    const int last_u = surface.cv_count(0) - 1;
    const int last_v = surface.cv_count(1) - 1;
    const double u0 = greville(surface, 0, 0);
    const double v0 = greville(surface, 1, 0);
    const Vector x = (surface.get_cv(last_u, 0) - surface.get_cv(0, 0)) / (greville(surface, 0, last_u) - u0);
    const Vector y = (surface.get_cv(0, last_v) - surface.get_cv(0, 0)) / (greville(surface, 1, last_v) - v0);
    projection.origin = surface.get_cv(0, 0) - x * u0 - y * v0;
    double determinant = 0.0;
    int first = 0;
    int second = 1;
    for (int i = 0; i < 3; ++i)
        for (int j = i + 1; j < 3; ++j)
            if (std::abs(x[i] * y[j] - x[j] * y[i]) > std::abs(determinant)) {
                determinant = x[i] * y[j] - x[j] * y[i];
                first = i;
                second = j;
            }
    if (!(std::abs(determinant) > 1e-14 * x.magnitude() * y.magnitude()))
        return false;
    projection.u[first] = y[second] / determinant;
    projection.u[second] = -y[first] / determinant;
    projection.v[first] = -x[second] / determinant;
    projection.v[second] = x[first] / determinant;
    for (int i = 0; i <= last_u; ++i)
        for (int j = 0; j <= last_v; ++j) {
            if (surface.weight(i, j) != surface.weight(0, 0))
                return false;
            const Vector delta = surface.get_cv(i, j) - projection.origin;
            if (std::abs(delta.dot(projection.u) - greville(surface, 0, i)) > error * projection.u.magnitude() ||
                std::abs(delta.dot(projection.v) - greville(surface, 1, j)) > error * projection.v.magnitude())
                return false;
        }
    return true;
}

NurbsCurve graph_pcurve(const NurbsCurve& curve, const NurbsSurface& surface, const GraphProjection& projection) {
    NurbsCurve result = curve;
    for (int i = 0; i < result.cv_count(); ++i) {
        const Vector delta = curve.get_cv(i) - projection.origin;
        const Point normalized(delta.dot(projection.u), delta.dot(projection.v), 0.0);
        const Point uv = actual_uv(surface, normalized);
        const double weight = curve.weight(i);
        if (!result.set_cv_4d(i, uv[0] * weight, uv[1] * weight, 0.0, weight))
            throw std::runtime_error("Graph overlap pcurve construction failed");
    }
    return result;
}

double binomial(int n, int k) {
    double value = 1.0;
    for (int i = 1; i <= std::min(k, n - k); ++i)
        value *= static_cast<double>(n - i + 1) / i;
    return value;
}

std::vector<double> product(const std::vector<double>& a, const std::vector<double>& b) {
    const int n = static_cast<int>(a.size()) - 1;
    const int m = static_cast<int>(b.size()) - 1;
    if (n < 0 || m < 0 || n + m > 96)
        return {};
    std::vector<double> result(n + m + 1, 0.0);
    for (int i = 0; i <= n; ++i)
        for (int j = 0; j <= m; ++j)
            result[i + j] += a[i] * b[j] * binomial(n, i) * binomial(m, j) / binomial(n + m, i + j);
    for (double coefficient : result)
        if (!std::isfinite(coefficient))
            return {};
    return result;
}

std::vector<double> power(const std::vector<double>& coefficients, int exponent) {
    std::vector<double> result = {1.0};
    for (int i = 0; i < exponent; ++i) {
        result = product(result, coefficients);
        if (result.empty())
            break;
    }
    return result;
}

std::vector<std::vector<double>> composed_basis(const NurbsCurve& pcurve, const NurbsSurface& patch, int direction) {
    const std::pair<double, double> domain = patch.domain(direction);
    std::vector<double> value;
    std::vector<double> complement;
    double maximum = 0.0;
    for (int i = 0; i < pcurve.cv_count(); ++i)
        maximum = std::max(maximum, pcurve.weight(i));
    for (int i = 0; i < pcurve.cv_count(); ++i) {
        const double parameter = (pcurve.get_cv(i)[direction] - domain.first) / (domain.second - domain.first);
        const double weight = pcurve.weight(i) / maximum;
        value.push_back(parameter * weight);
        complement.push_back((1.0 - parameter) * weight);
    }
    const int degree = patch.degree(direction);
    std::vector<std::vector<double>> result;
    for (int i = 0; i <= degree; ++i) {
        std::vector<double> coefficients = product(power(value, i), power(complement, degree - i));
        if (coefficients.empty())
            return {};
        for (double& coefficient : coefficients)
            coefficient *= binomial(degree, i);
        result.push_back(std::move(coefficients));
    }
    return result;
}

std::array<std::vector<double>, 4> composed_surface(const NurbsCurve& pcurve, const NurbsSurface& patch) {
    const std::vector<std::vector<double>> u = composed_basis(pcurve, patch, 0);
    const std::vector<std::vector<double>> v = composed_basis(pcurve, patch, 1);
    if (u.empty() || v.empty())
        return {};
    const int degree = pcurve.degree() * (patch.degree(0) + patch.degree(1));
    std::array<std::vector<double>, 4> result;
    for (std::vector<double>& coordinates : result)
        coordinates.resize(degree + 1, 0.0);
    double maximum = 0.0;
    for (int i = 0; i < patch.cv_count(0); ++i)
        for (int j = 0; j < patch.cv_count(1); ++j)
            maximum = std::max(maximum, patch.weight(i, j));
    for (int i = 0; i < patch.cv_count(0); ++i)
        for (int j = 0; j < patch.cv_count(1); ++j) {
            const std::vector<double> basis = product(u[i], v[j]);
            if (basis.empty())
                return {};
            const Point point = patch.get_cv(i, j);
            const double weight = patch.weight(i, j) / maximum;
            for (int d = 0; d < 4; ++d)
                for (size_t k = 0; k < basis.size(); ++k)
                    result[d][k] += basis[k] * weight * (d == 3 ? 1.0 : point[d]);
        }
    return result;
}

bool certified_composition(
    const NurbsCurve& curve,
    const NurbsCurve& pcurve,
    const NurbsSurface& patch,
    double tolerance
) {
    if (curve.cv_count() != curve.order() || pcurve.cv_count() != pcurve.order() ||
        patch.cv_count(0) != patch.order(0) || patch.cv_count(1) != patch.order(1))
        return false;
    const std::array<std::vector<double>, 4> composed = composed_surface(pcurve, patch);
    if (composed[0].empty())
        return false;
    double maximum = 0.0;
    double minimum = std::numeric_limits<double>::infinity();
    for (int i = 0; i < curve.cv_count(); ++i) {
        maximum = std::max(maximum, curve.weight(i));
        minimum = std::min(minimum, curve.weight(i));
    }
    const double denominator = minimum / maximum * *std::min_element(composed[3].begin(), composed[3].end());
    if (!(denominator > 0.0))
        return false;
    std::vector<double> weights;
    for (int i = 0; i < curve.cv_count(); ++i)
        weights.push_back(curve.weight(i) / maximum);
    double squared = 0.0;
    for (int d = 0; d < 3; ++d) {
        std::vector<double> coordinates;
        for (int i = 0; i < curve.cv_count(); ++i)
            coordinates.push_back(curve.get_cv(i)[d] * weights[i]);
        const std::vector<double> a = product(coordinates, composed[3]);
        const std::vector<double> b = product(weights, composed[d]);
        if (a.empty() || b.empty())
            return false;
        double error = 0.0;
        for (size_t i = 0; i < a.size(); ++i)
            error = std::max(error, std::abs(a[i] - b[i]));
        squared += error * error;
    }
    return std::isfinite(squared) && std::sqrt(squared) <= tolerance * denominator;
}

std::vector<double> graph_cuts(const NurbsCurve& curve, const NurbsSurface& surface, bool internal_only = false) {
    std::vector<double> cuts = curve.get_span_vector();
    const size_t margin = internal_only ? 1 : 0;
    for (int d = 0; d < 2; ++d) {
        const std::vector<double> spans = surface.get_span_vector(d);
        for (size_t i = margin; i + margin < spans.size(); ++i) {
            const double parameter = spans[i];
            Point origin;
            Vector normal;
            origin[d] = parameter;
            normal[d] = 1.0;
            const Plane plane = Plane::from_point_normal(origin, normal);
            const double error = 64.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, std::abs(parameter));
            const std::vector<double> roots = Intersection::curve_plane(curve, plane, error);
            cuts.insert(cuts.end(), roots.begin(), roots.end());
        }
    }
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
    return cuts;
}

bool parameter_image_bound(
    const NurbsCurve& curve,
    const NurbsCurve& pcurve,
    const NurbsSurface& surface,
    const std::array<double, 2>& interval,
    double tolerance
) {
    for (int d = 0; d < 2; ++d) {
        const auto domain = surface.domain(d);
        const auto& knots = surface.m_nurbsknot[d];
        for (size_t i = 0; i < knots.size();) {
            size_t end = i + 1;
            while (end < knots.size() && knots[end] == knots[i])
                ++end;
            if (knots[i] > domain.first && knots[i] < domain.second &&
                end - i > static_cast<size_t>(surface.degree(d)))
                return false;
            i = end;
        }
    }
    const Point point = curve.get_cv(0);
    const std::array<double, 3> origin = {point[0], point[1], point[2]};
    nurbs_interval::CurveBounds world;
    nurbs_interval::CurveBounds uv;
    nurbs_interval::SurfaceBounds lifted;
    if (!nurbs_interval::curve_bounds(curve, interval, origin, world) ||
        !nurbs_interval::curve_bounds(pcurve, interval, {0, 0, 0}, uv) ||
        !nurbs_interval::surface_bounds(surface, {uv.point[0].lo, uv.point[0].hi, uv.point[1].lo, uv.point[1].hi},
            origin, lifted, false, true))
        return false;
    nurbs_interval::Interval squared;
    for (int d = 0; d < 3; ++d) {
        const double distance = nurbs_interval::magnitude(world.point[d] - lifted.point[d]);
        squared = squared + nurbs_interval::Interval(distance) * nurbs_interval::Interval(distance);
    }
    const auto limit = nurbs_interval::Interval(tolerance) * nurbs_interval::Interval(tolerance);
    return nurbs_interval::finite(squared) && squared.hi <= limit.lo;
}

bool parameter_point_mismatch(const NurbsCurve& curve, double t,
    const NurbsCurve& pcurve, double uv_t, const NurbsSurface& surface, double tolerance) {
    const Point anchor = curve.get_cv(0);
    const std::array<double, 3> origin = {anchor[0], anchor[1], anchor[2]};
    nurbs_interval::CurveBounds world, uv;
    nurbs_interval::SurfaceBounds lifted;
    if (!nurbs_interval::curve_bounds(curve, {t, t}, origin, world) ||
        !nurbs_interval::curve_bounds(pcurve, {uv_t, uv_t}, {0, 0, 0}, uv) ||
        !nurbs_interval::surface_bounds(surface,
            {uv.point[0].lo, uv.point[0].hi, uv.point[1].lo, uv.point[1].hi}, origin, lifted, false, true))
        return false;
    nurbs_interval::Interval squared;
    for (int d = 0; d < 3; ++d) {
        const auto residual = world.point[d] - lifted.point[d];
        if (!nurbs_interval::finite(residual))
            return false;
        const double lower = residual.lo > 0 ? residual.lo : residual.hi < 0 ? -residual.hi : 0;
        squared = squared + nurbs_interval::Interval(lower) * nurbs_interval::Interval(lower);
    }
    const auto allowed = nurbs_interval::Interval(tolerance) * nurbs_interval::Interval(tolerance);
    return nurbs_interval::finite(squared) && nurbs_interval::finite(allowed) && squared.lo > allowed.hi;
}

bool parameter_parts(
    NurbsCurve curve,
    NurbsCurve pcurve,
    const NurbsSurface& surface,
    double tolerance,
    std::vector<ParameterPart>& work,
    std::string* diagnostic
) {
    if (!curve.set_domain(0.0, 1.0) || !pcurve.set_domain(0.0, 1.0))
        return false;

    std::vector<double> cuts = graph_cuts(pcurve, surface, true);
    const std::vector<double> spans = curve.get_span_vector();
    cuts.insert(cuts.end(), spans.begin(), spans.end());
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
    if (cuts.size() < 2 || cuts.size() > 4096)
        return false;

    for (size_t i = 1; i < cuts.size(); ++i) {
        NurbsCurve part = curve;
        NurbsCurve uv = pcurve;
        if (!part.trim(cuts[i - 1], cuts[i]) || !uv.trim(cuts[i - 1], cuts[i])) {
            if (parameter_image_bound(curve, pcurve, surface, {cuts[i - 1], cuts[i]}, tolerance))
                continue;
            if (diagnostic) {
                std::ostringstream text;
                text.precision(17);
                text << "Common interval trim failed [" << cuts[i - 1] << ',' << cuts[i] << ']';
                *diagnostic = text.str();
            }
            return false;
        }

        work.push_back({std::move(part), std::move(uv)});
    }
    return true;
}

std::vector<std::array<double, 4>> parameter_rectangles(const NurbsCurve& pcurve, const NurbsSurface& surface) {
    Bounds uv;
    if (!bounds(pcurve, uv))
        return {};
    const std::array<std::vector<double>, 2> spans = {surface.get_span_vector(0), surface.get_span_vector(1)};
    std::array<size_t, 2> first;
    std::array<size_t, 2> last;
    for (int d = 0; d < 2; ++d) {
        first[d] = std::clamp(static_cast<size_t>(std::upper_bound(spans[d].begin(), spans[d].end(), uv.lo[d]) -
            spans[d].begin()), size_t{1}, spans[d].size() - 1);
        last[d] = std::clamp(static_cast<size_t>(std::upper_bound(spans[d].begin(), spans[d].end(), uv.hi[d]) -
            spans[d].begin()), size_t{1}, spans[d].size() - 1);
    }
    if ((last[0] - first[0] + 1) * (last[1] - first[1] + 1) > 64)
        return {};
    std::vector<std::array<double, 4>> rectangles;
    for (size_t i = first[0]; i <= last[0]; ++i)
        for (size_t j = first[1]; j <= last[1]; ++j)
            rectangles.push_back({spans[0][i - 1], spans[0][i], spans[1][j - 1], spans[1][j]});
    return rectangles;
}

void parameter_diagnostic(
    const std::string& stage,
    const ParameterPart& part,
    const NurbsSurface& patch,
    std::string* diagnostic
) {
    if (!diagnostic)
        return;
    Bounds uv;
    bounds(part.pcurve, uv);
    std::ostringstream text;
    text.precision(17);
    text << stage << "; interval=[" << part.curve.domain_start() << ',' << part.curve.domain_end()
        << "]; depth=" << part.depth << "; CVs=" << part.curve.cv_count() << '/' << part.pcurve.cv_count()
        << '/' << patch.cv_count(0) << 'x' << patch.cv_count(1)
        << "; patchU=[" << patch.domain(0).first << ',' << patch.domain(0).second
        << "]; patchV=[" << patch.domain(1).first << ',' << patch.domain(1).second
        << "]; pcurveU=[" << uv.lo[0] << ',' << uv.hi[0] << "]; pcurveV=[" << uv.lo[1] << ',' << uv.hi[1] << ']';
    *diagnostic = text.str();
}

ParameterMatch parameter_match(
    const ParameterPart& part,
    const NurbsSurface& surface,
    double tolerance,
    std::string* diagnostic
) {
    for (const double t : {part.curve.domain_start(), part.curve.domain_middle(), part.curve.domain_end()}) {
        if (parameter_point_mismatch(part.curve, t, part.pcurve, t, surface, tolerance))
            return ParameterMatch::Mismatched;
    }
    const std::vector<std::array<double, 4>> rectangles = parameter_rectangles(part.pcurve, surface);
    if (rectangles.empty()) {
        parameter_diagnostic("Patch enclosure exceeds support budget", part, surface, diagnostic);
        return ParameterMatch::Unresolved;
    }
    // Every possible evaluation branch must satisfy the bound, including either side of a rounded knot crossing.
    for (const std::array<double, 4>& rectangle : rectangles) {
        NurbsSurface patch = surface;
        if (!patch.trim(0, {rectangle[0], rectangle[1]}) || !patch.trim(1, {rectangle[2], rectangle[3]})) {
            parameter_diagnostic("Patch extraction failed", part, patch, diagnostic);
            return ParameterMatch::Unresolved;
        }
        if (!certified_composition(part.curve, part.pcurve, patch, tolerance)) {
            parameter_diagnostic("Composition bound unresolved", part, patch, diagnostic);
            return ParameterMatch::Unresolved;
        }
    }
    return ParameterMatch::Matched;
}

bool split_parameters(const ParameterPart& part, std::vector<ParameterPart>& work) {
    ParameterPart a;
    ParameterPart b;
    const double middle = part.curve.domain_middle();
    if (!part.curve.split(middle, a.curve, b.curve) || !part.pcurve.split(middle, a.pcurve, b.pcurve))
        return false;

    a.depth = b.depth = part.depth + 1;
    work.push_back(std::move(a));
    work.push_back(std::move(b));
    return true;
}

bool graph_piece(
    Result& result,
    const NurbsCurve& curve,
    const NurbsCurve& pcurve,
    const NurbsSurface& surface,
    double tolerance
) {
    const Point uv = pcurve.point_at(std::midpoint(pcurve.domain_start(), pcurve.domain_end()));
    NurbsSurface patch = surface;
    for (int d = 0; d < 2; ++d) {
        const std::vector<double> spans = surface.get_span_vector(d);
        if (uv[d] < spans.front() || uv[d] > spans.back())
            return true;
        for (size_t i = 1; i < spans.size(); ++i)
            if (uv[d] >= spans[i - 1] && uv[d] <= spans[i]) {
                if (!patch.trim(d, {spans[i - 1], spans[i]}))
                    return false;
                break;
            }
    }
    if (!certified_composition(curve, pcurve, patch, tolerance * 0.125))
        return false;
    append_overlap(result, pcurve);
    return true;
}

bool graph_overlap(
    Result& result,
    const NurbsCurve& curve,
    const NurbsSurface& surface,
    double error,
    double tolerance
) {
    GraphProjection projection;
    if (!graph_projection(surface, error, projection))
        return false;
    const NurbsCurve pcurve = graph_pcurve(curve, surface, projection);
    const std::vector<double> cuts = graph_cuts(pcurve, surface);
    Result candidate;
    for (size_t i = 1; i < cuts.size(); ++i) {
        NurbsCurve part = curve;
        NurbsCurve uv = pcurve;
        if (!part.trim(cuts[i - 1], cuts[i]) || !uv.trim(cuts[i - 1], cuts[i]))
            return false;
        if (!graph_piece(candidate, part, uv, surface, tolerance))
            return false;
    }
    for (double t : cuts) {
        const Point uv = pcurve.point_at(t);
        if (in_overlap(candidate, t) || curve.point_at(t).distance(surface.point_at(uv[0], uv[1])) > tolerance)
            continue;
        if (uv[0] < surface.domain(0).first || uv[0] > surface.domain(0).second ||
            uv[1] < surface.domain(1).first || uv[1] > surface.domain(1).second)
            continue;
        candidate.points.push_back({curve.point_at(t), t, uv[0], uv[1], true});
    }
    result = std::move(candidate);
    return true;
}

bool solve(const std::array<Vector, 3>& columns, const Vector& rhs, Vector& solution) {
    const Vector cross = columns[1].cross(columns[2]);
    const double determinant = columns[0].dot(cross);
    const double scale = columns[0].magnitude() * columns[1].magnitude() * columns[2].magnitude();
    if (!std::isfinite(determinant) || !std::isfinite(scale) || std::abs(determinant) <= 1e-14 * scale || !(scale > 0.0))
        return false;
    solution = {rhs.dot(cross) / determinant,
        columns[0].dot(rhs.cross(columns[2])) / determinant,
        columns[0].dot(columns[1].cross(rhs)) / determinant};
    return true;
}

bool least_squares(const std::array<Vector, 3>& columns, const Vector& rhs, Vector& solution) {
    std::array<Vector, 3> normal;
    Vector projected;
    double scale = 0.0;
    for (int i = 0; i < 3; ++i) {
        projected[i] = columns[i].dot(rhs);
        for (int j = 0; j < 3; ++j)
            normal[i][j] = columns[i].dot(columns[j]);
        scale = std::max(scale, normal[i][i]);
    }
    for (int i = 0; i < 3; ++i)
        normal[i][i] += scale * 1e-8;
    return solve(normal, projected, solution);
}

Vector bounded_step(const Part& part, const Vector& parameters, Vector step) {
    const std::array<std::pair<double, double>, 3> domains = {
        std::pair(part.curve.domain_start(), part.curve.domain_end()), part.surface.domain(0), part.surface.domain(1)
    };
    const double scale = std::max({1.0, 4.0 * std::abs(step[0]), 4.0 * std::abs(step[1]), 4.0 * std::abs(step[2])});
    step = step / scale;
    const double gap = part.curve.point_at(parameters[0]).distance(part.surface.point_at(parameters[1], parameters[2]));
    for (int iteration = 0; iteration < 20; ++iteration) {
        Vector next;
        for (int d = 0; d < 3; ++d)
            next[d] = std::clamp(parameters[d] - step[d] * (domains[d].second - domains[d].first),
                domains[d].first, domains[d].second);
        const double distance = part.curve.point_at(next[0]).distance(part.surface.point_at(next[1], next[2]));
        if (std::isfinite(distance) && distance < gap)
            return next;

        step = step * 0.5;
    }
    return parameters;
}

bool refine(
    const Part& part,
    double tolerance,
    Hit& hit,
    bool seeded
) {
    const std::array<std::pair<double, double>, 3> domains = {
        std::pair(part.curve.domain_start(), part.curve.domain_end()), part.surface.domain(0), part.surface.domain(1)
    };
    Vector parameters;
    Vector widths;
    for (int d = 0; d < 3; ++d) {
        parameters[d] = seeded ? (d == 0 ? hit.t : d == 1 ? hit.u : hit.v) :
            std::midpoint(domains[d].first, domains[d].second);
        widths[d] = domains[d].second - domains[d].first;
    }
    for (int iteration = 0; iteration < 40; ++iteration) {
        const std::vector<Vector> c = part.curve.evaluate(parameters[0], 1);
        const std::vector<Vector> s = part.surface.evaluate(parameters[1], parameters[2], 1);
        const Vector residual = c[0] - s[0];
        const std::array<Vector, 3> columns = {c[1] * widths[0], s[2] * -widths[1], s[1] * -widths[2]};
        Vector step;
        if (!solve(columns, residual, step) && !least_squares(columns, residual, step))
            break;
        if (!std::isfinite(step[0]) || !std::isfinite(step[1]) || !std::isfinite(step[2]))
            return false;
        const Vector next = bounded_step(part, parameters, step);
        if (next[0] == parameters[0] && next[1] == parameters[1] && next[2] == parameters[2])
            break;
        parameters = next;
    }
    hit.t = parameters[0];
    hit.u = parameters[1];
    hit.v = parameters[2];
    hit.point = part.curve.point_at(parameters[0]);
    const double distance = hit.point.distance(part.surface.point_at(parameters[1], parameters[2]));
    if (!finite(hit.point) || !std::isfinite(distance) || distance > tolerance * 0.25)
        return false;
    const std::vector<Vector> c = part.curve.evaluate(parameters[0], 1);
    const std::vector<Vector> s = part.surface.evaluate(parameters[1], parameters[2], 1);
    const Vector normal = s[2].cross(s[1]);
    hit.tangent = std::abs(normal.dot(c[1])) <= 1e-7 * normal.magnitude() * c[1].magnitude();
    return true;
}

bool contact_gradient(
    const NurbsCurve& curve,
    const NurbsSurface& surface,
    const Vector& parameters,
    Vector& gradient,
    double& value
) {
    const std::vector<Vector> c = curve.evaluate(parameters[0], 2);
    const std::vector<Vector> s = surface.evaluate(parameters[1], parameters[2], 2);
    const Vector normal = s[3].cross(s[1]);
    const double nn = normal.dot(normal);
    const double tt = c[1].dot(c[1]);
    if (!(nn > 0.0) || !(tt > 0.0))
        return false;

    const double divisor = std::sqrt(nn * tt);
    const Vector u = s[5].cross(s[1]) + s[3].cross(s[4]);
    const Vector v = s[4].cross(s[1]) + s[3].cross(s[2]);
    value = normal.dot(c[1]) / divisor;
    gradient = {normal.dot(c[2]) / divisor - value * c[1].dot(c[2]) / tt,
        u.dot(c[1]) / divisor - value * normal.dot(u) / nn,
        v.dot(c[1]) / divisor - value * normal.dot(v) / nn};
    return std::isfinite(value) && std::isfinite(gradient[0]) &&
        std::isfinite(gradient[1]) && std::isfinite(gradient[2]);
}

bool contact_step(const Part& part, const Vector& parameters, double scale, Vector& step) {
    Vector gradient;
    double value = 0.0;
    if (!contact_gradient(part.curve, part.surface, parameters, gradient, value) || std::abs(value) > 1e-4)
        return false;

    const std::vector<Vector> c = part.curve.evaluate(parameters[0], 1);
    const std::vector<Vector> s = part.surface.evaluate(parameters[1], parameters[2], 1);
    const Vector residual = c[0] - s[0];
    const std::pair<double, double> u = part.surface.domain(0);
    const std::pair<double, double> v = part.surface.domain(1);
    const Vector widths(part.curve.domain_end() - part.curve.domain_start(), u.second - u.first, v.second - v.first);
    const std::array<Vector, 3> columns = {c[1] * widths[0], s[2] * -widths[1], s[1] * -widths[2]};
    for (int i = 0; i < 3; ++i)
        gradient[i] *= widths[i];
    std::array<Vector, 3> matrix;
    Vector rhs;
    for (int i = 0; i < 3; ++i) {
        rhs[i] = columns[i].dot(residual) + scale * gradient[i] * value;
        for (int j = 0; j < 3; ++j)
            matrix[i][j] = columns[i].dot(columns[j]) + scale * gradient[i] * gradient[j];
    }
    if (!solve(matrix, rhs, step))
        return false;
    for (int i = 0; i < 3; ++i)
        step[i] *= widths[i];

    return std::isfinite(step[0]) && std::isfinite(step[1]) && std::isfinite(step[2]);
}

bool polish_contact(const Part& part, double tolerance, Hit& hit) {
    Bounds a;
    Bounds b;
    if (!bounds(part.curve, a) || !bounds(part.surface, b))
        return false;

    const double length = std::max(a.lo.distance(a.hi), b.lo.distance(b.hi));
    const std::array<std::pair<double, double>, 3> domains = {
        std::pair(part.curve.domain_start(), part.curve.domain_end()), part.surface.domain(0), part.surface.domain(1)
    };
    Vector parameters(hit.t, hit.u, hit.v);
    for (int iteration = 0; iteration < 24; ++iteration) {
        Vector step;
        if (!contact_step(part, parameters, length * length, step))
            break;

        Vector next;
        for (int d = 0; d < 3; ++d)
            next[d] = std::clamp(parameters[d] - step[d], domains[d].first, domains[d].second);
        if (next[0] == parameters[0] && next[1] == parameters[1] && next[2] == parameters[2])
            break;

        parameters = next;
    }
    Vector gradient;
    double value = 0.0;
    if (!contact_gradient(part.curve, part.surface, parameters, gradient, value) || std::abs(value) > 1e-9)
        return false;

    const Point point = part.curve.point_at(parameters[0]);
    const double gap = point.distance(part.surface.point_at(parameters[1], parameters[2]));
    const double error = std::min(tolerance * 0.25, (a.roundoff + b.roundoff) * 4.0);
    if (!finite(point) || !std::isfinite(gap) || gap > error)
        return false;

    hit = {point, parameters[0], parameters[1], parameters[2], true};
    return true;
}

bool same_surface_parameters(const NurbsSurface& surface, const Hit& a, const Hit& b, double tolerance) {
    const std::vector<Vector> derivatives = surface.evaluate(a.u, a.v, 1);
    const std::array<double, 2> differences = {std::abs(a.u - b.u), std::abs(a.v - b.v)};
    for (int d = 0; d < 2; ++d) {
        const std::pair<double, double> domain = surface.domain(d);
        const double speed = derivatives[2 - d].magnitude();
        const double range = domain.second - domain.first;
        const double error = 64.0 * std::numeric_limits<double>::epsilon() * std::max(std::abs(domain.first), std::abs(domain.second));
        if (differences[d] > error + (speed > 0.0 ? tolerance / speed : range * 1e-12))
            return false;
    }
    return true;
}

void append_hit(
    Result& result,
    const NurbsCurve& curve,
    const NurbsSurface& surface,
    const Hit& hit,
    double tolerance
) {
    const double speed = curve.evaluate(hit.t, 1)[1].magnitude();
    const double range = curve.domain_end() - curve.domain_start();
    const double error = 64.0 * std::numeric_limits<double>::epsilon() * std::max(std::abs(hit.t), range);
    const double parameter_tolerance = error + (speed > 0.0 ? tolerance / speed : 0.0);
    for (Hit& previous : result.points)
        if (std::abs(previous.t - hit.t) <= parameter_tolerance && previous.point.distance(hit.point) <= tolerance &&
            same_surface_parameters(surface, previous, hit, tolerance)) {
            previous.tangent = previous.tangent || hit.tangent;
            return;
        }
    result.points.push_back(hit);
}

bool subdivide(const Part& part, int direction, std::vector<Part>& work) {
    if (direction == 0) {
        NurbsCurve left;
        NurbsCurve right;
        const double middle = std::midpoint(part.curve.domain_start(), part.curve.domain_end());
        if (!part.curve.split(middle, left, right))
            return false;
        work.push_back({std::move(right), part.surface, part.depth + 1});
        work.push_back({std::move(left), part.surface, part.depth + 1});
    } else {
        const std::pair<double, double> domain = part.surface.domain(direction - 1);
        std::pair<NurbsSurface, NurbsSurface> halves = part.surface.split(direction - 1, std::midpoint(domain.first, domain.second));
        if (!halves.first.is_valid() || !halves.second.is_valid())
            return false;
        work.push_back({part.curve, std::move(halves.second), part.depth + 1});
        work.push_back({part.curve, std::move(halves.first), part.depth + 1});
    }
    return true;
}

void intersect_general(Result& result, const NurbsCurve& curve, const NurbsSurface& surface, double tolerance) {
    const Part original{curve, surface, 0};
    Bounds original_curve;
    Bounds original_surface;
    if (!bounds(curve, original_curve) || !bounds(surface, original_surface)) {
        fail(result, "Invalid curve/surface root bounds");
        return;
    }
    const double precision = std::min(tolerance, (original_curve.roundoff + original_surface.roundoff) * 32.0);
    std::vector<Part> work = {{curve, surface, 0}};
    for (size_t iteration = 0; !work.empty() && iteration < 200000; ++iteration) {
        Part part = std::move(work.back());
        work.pop_back();
        Bounds a;
        Bounds b;
        if (!bounds(part.curve, a) || !bounds(part.surface, b)) {
            fail(result, "Invalid rational control hull during curve/surface subdivision");
            return;
        }
        if (separated(a, b) || separated_hulls(part, (a.roundoff + b.roundoff) * 4.0))
            continue;
        const double ca = a.lo.distance(a.hi);
        const double sb = b.lo.distance(b.hi);
        if (std::max(ca, sb) <= tolerance * 0.125) {
            Hit hit;
            if (!refine(part, tolerance, hit, false)) {
                fail(result, "Unresolved local curve/surface root enclosure");
                continue;
            }
            const bool refined = refine(original, precision, hit, true);
            const bool contact = polish_contact(original, precision, hit);
            if (refined || contact)
                append_hit(result, curve, surface, hit, tolerance);
            else
                fail(result, "Unresolved curve/surface root enclosure");
            continue;
        }
        if (part.depth >= 160) {
            fail(result, "Curve/surface subdivision depth exceeded");
            return;
        }
        const int direction = ca > sb ? 0 : (b.width[0] >= b.width[1] ? 1 : 2);
        if (!subdivide(part, direction, work)) {
            fail(result, "Curve/surface subdivision failed");
            return;
        }
        if (result.points.size() > 1024) {
            fail(result, "Unresolved curve/surface coincidence or excessive root count");
            return;
        }
    }
    if (!work.empty())
        fail(result, "Curve/surface subdivision work limit exceeded");
}

}

static ParameterMatch same_parameter_impl(
    const NurbsCurve& curve,
    const NurbsCurve& pcurve,
    const NurbsSurface& surface,
    double tolerance,
    std::string* diagnostic,
    bool* coordinate_retry = nullptr
) {
    if (coordinate_retry)
        *coordinate_retry = false;
    if (diagnostic)
        *diagnostic = "Input or coordinate-resolution validation failed";
    if (!std::isfinite(tolerance) || tolerance <= 0.0 || !curve.is_valid() || !pcurve.is_valid() || !surface.is_valid())
        return ParameterMatch::Unresolved;

    Bounds a;
    Bounds b;
    Bounds uv;
    if (!bounds(curve, a) || !bounds(surface, b) || !bounds(pcurve, uv))
        return ParameterMatch::Unresolved;
    if (a.roundoff + b.roundoff > tolerance * 0.125) {
        if (coordinate_retry)
            *coordinate_retry = true;
        return ParameterMatch::Unresolved;
    }

    // A single certified negative witness is decisive even if common partitioning is unresolved.
    for (int end = 0; end < 2; ++end)
        if (parameter_point_mismatch(curve, end ? curve.domain_end() : curve.domain_start(),
                pcurve, end ? pcurve.domain_end() : pcurve.domain_start(), surface, tolerance)) {
            if (diagnostic)
                *diagnostic = "Certified endpoint residual exceeds tolerance";
            return ParameterMatch::Mismatched;
        }

    std::vector<ParameterPart> work;
    if (diagnostic)
        *diagnostic = "Common parameter partition failed";
    if (!parameter_parts(curve, pcurve, surface, tolerance, work, diagnostic))
        return ParameterMatch::Unresolved;

    for (size_t iteration = 0; !work.empty() && iteration < 4096; ++iteration) {
        const ParameterPart part = std::move(work.back());
        work.pop_back();
        const ParameterMatch match = parameter_match(part, surface, tolerance, diagnostic);
        if (match == ParameterMatch::Matched)
            continue;
        if (match == ParameterMatch::Mismatched)
            return match;
        if (part.depth >= 20 || !split_parameters(part, work))
            return ParameterMatch::Unresolved;
    }
    if (work.empty() && diagnostic)
        diagnostic->clear();
    return work.empty() ? ParameterMatch::Matched : ParameterMatch::Unresolved;
}

// Translate both physical geometries only when EVERY homogeneous operation is exact.
// The pcurve and raw parameter metadata are untouched. Failed candidates never escape.
static bool exact_world_chart(NurbsCurve& world, NurbsSurface& surface) {
    if (world.dimension() != 3 || surface.dimension() != 3)
        return false;
    const Point origin = world.get_cv(0);
    for (int d = 0; d < 3; ++d)
        if (!std::isfinite(origin[d]))
            return false;
    const auto shift = [&](double* cv, bool rational) {
        const double weight = rational ? cv[3] : 1.0;
        if (!(weight > 0) || !std::isfinite(weight))
            return false;
        for (int d = 0; d < 3; ++d) {
            const double product = origin[d] * weight;
            double local;
            if (!nurbs_coincidence::exact_products_equal(origin[d], weight, product, 1.0)
                || !nurbs_coincidence::exact_difference(cv[d], product, local))
                return false;
            cv[d] = local;
        }
        return true;
    };
    for (int i = 0; i < world.cv_count(); ++i)
        if (!shift(world.cv(i), world.is_rational()))
            return false;
    for (int i = 0; i < surface.cv_count(0); ++i)
        for (int j = 0; j < surface.cv_count(1); ++j)
            if (!shift(surface.cv(i, j), surface.is_rational()))
                return false;
    return true;
}

ParameterMatch same_parameter(const NurbsCurve& curve, const NurbsCurve& pcurve,
    const NurbsSurface& surface, double tolerance, std::string* diagnostic) {
    // The ordinary worker runs exactly once. Only its explicit roundoff-only
    // failure enables an exact common chart; invalid input never enables retry.
    bool coordinate_retry = false;
    const ParameterMatch original = same_parameter_impl(curve, pcurve, surface,
        tolerance, diagnostic, &coordinate_retry);
    if (!coordinate_retry)
        return original;
    NurbsCurve local_world = curve;
    NurbsSurface local_surface = surface;
    if (!exact_world_chart(local_world, local_surface))
        return original; // Preserve the original coordinate-guard diagnostic.
    return same_parameter_impl(local_world, pcurve, local_surface, tolerance, diagnostic);
}

Result intersect(const NurbsCurve& curve, const NurbsSurface& surface, double tolerance) {
    if (!std::isfinite(tolerance) || tolerance <= 0.0 || !curve.is_valid() || !surface.is_valid())
        throw std::invalid_argument("Curve/surface intersection requires valid geometry and a finite positive tolerance");
    Result result;
    Bounds a;
    Bounds b;
    if (!bounds(curve, a) || !bounds(surface, b)) {
        fail(result, "Curve/surface intersection requires finite control points and positive finite weights");
        return result;
    }
    const double error = std::max(a.roundoff + b.roundoff, std::numeric_limits<double>::min());
    if (error > tolerance * 0.25) {
        fail(result, "Curve/surface tolerance is below floating-point resolution");
        return result;
    }
    if (separated(a, b))
        return result;
    if (a.lo.distance(a.hi) <= error || !regular_hull(surface, error)) {
        fail(result, "Degenerate curve or surface requires a lower-dimensional intersection");
        return result;
    }
    AffinePatch patch;
    if (affine_patch(surface, error, patch))
        intersect_planar(result, curve, surface, patch, error, tolerance);
    else if (!iso_overlap(result, curve, surface, error) && !graph_overlap(result, curve, surface, error, tolerance)) {
        if (unresolved_overlap(curve, surface, error))
            fail(result, "On-surface overlap needs a certified parameter mapping");
        else
            intersect_general(result, curve, surface, tolerance);
    }
    std::sort(result.points.begin(), result.points.end(), [](const Hit& a, const Hit& b) {
        return a.t < b.t;
    });
    result.points.erase(std::unique(result.points.begin(), result.points.end(), [](const Hit& a, const Hit& b) {
        return a.t == b.t && a.u == b.u && a.v == b.v;
    }), result.points.end());
    return result;
}

}
