#include "surface_section.h"
#include "intersection.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace session_cpp::surface_section {
namespace {

struct Extrusion {
    int direction;
    Vector displacement;
    Vector normal;
    Point origin;
    double denominator;
    double start;
    double length;
};

bool finite(const Point& point) {
    return std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2]);
}

Point control_point(const NurbsSurface& surface, int direction, int row, int column) {
    return direction == 0 ? surface.get_cv(row, column) : surface.get_cv(column, row);
}

double control_weight(const NurbsSurface& surface, int direction, int row, int column) {
    return direction == 0 ? surface.weight(row, column) : surface.weight(column, row);
}

bool extrusion(const NurbsSurface& surface, const Plane& plane, double tolerance, Extrusion& result) {
    for (int direction = 0; direction < 2; ++direction) {
        if (surface.degree(direction) != 1 || surface.cv_count(direction) != 2)
            continue;
        const Vector displacement = control_point(surface, direction, 1, 0) - control_point(surface, direction, 0, 0);
        bool constant = displacement.magnitude() > 0.0;
        for (int i = 0; i < surface.cv_count(1 - direction); ++i) {
            const Point first = control_point(surface, direction, 0, i);
            const Point last = control_point(surface, direction, 1, i);
            const double weight = control_weight(surface, direction, 0, i);
            constant = constant && finite(first) && finite(last) && std::isfinite(weight) && weight > 0.0;
            constant = constant && weight == control_weight(surface, direction, 1, i);
            constant = constant && (last - first - displacement).magnitude() <= tolerance * 0.0625;
        }
        if (!constant)
            continue;
        Vector normal = plane.z_axis();
        if (!normal.normalize_self())
            return false;
        const double denominator = displacement.dot(normal);
        if (!std::isfinite(denominator) || std::abs(denominator) <= 64.0 * std::numeric_limits<double>::epsilon() * displacement.magnitude())
            continue;
        const std::pair<double, double> domain = surface.domain(direction);
        result = {direction, displacement, normal, plane.origin(), denominator, domain.first, domain.second - domain.first};
        return true;
    }
    return false;
}

double position(const Extrusion& extrusion, const Point& point) {
    return (extrusion.origin - point).dot(extrusion.normal) / extrusion.denominator;
}

std::optional<NurbsCurve> projected_world(const NurbsCurve& base, const Extrusion& extrusion, double tolerance) {
    NurbsCurve result = base;
    for (int i = 0; i < base.cv_count(); ++i) {
        const Point point = base.get_cv(i);
        const Point projected = point + extrusion.displacement * position(extrusion, point);
        const double weight = base.weight(i);
        if (!finite(projected) || std::abs((projected - extrusion.origin).dot(extrusion.normal)) > tolerance * 0.125)
            return std::nullopt;
        if (!result.set_cv_4d(
            i,
            projected[0] * weight,
            projected[1] * weight,
            projected[2] * weight,
            weight
        ))
            return std::nullopt;
    }
    return result;
}

std::optional<NurbsCurve> section_pcurve(const NurbsCurve& span, const Extrusion& extrusion) {
    const int degree = span.degree();
    if (span.cv_count() != degree + 1)
        return std::nullopt;
    NurbsCurve result(3, true, degree + 2, degree + 2);
    const double start = span.domain_start();
    const double length = span.domain_end() - start;
    for (int i = 0; i < result.nurbsknot_count(); ++i)
        if (!result.set_nurbsknot(i, i <= degree ? start : start + length))
            return std::nullopt;
    for (int i = 0; i <= degree + 1; ++i) {
        const double alpha = static_cast<double>(i) / (degree + 1);
        const double left = i > 0 ? span.weight(i - 1) * alpha : 0.0;
        const double right = i <= degree ? span.weight(i) * (1.0 - alpha) : 0.0;
        const double weight = left + right;
        double height = 0.0;
        if (i > 0)
            height += left * position(extrusion, span.get_cv(i - 1));
        if (i <= degree)
            height += right * position(extrusion, span.get_cv(i));
        Point uv;
        uv[1 - extrusion.direction] = start * weight + length * left;
        uv[extrusion.direction] = extrusion.start * weight + extrusion.length * height;
        if (!result.set_cv_4d(i, uv[0], uv[1], 0.0, weight))
            return std::nullopt;
    }
    return result;
}

std::vector<double> clip_parameters(const NurbsCurve& pcurve, const Extrusion& extrusion) {
    std::vector<double> cuts = {pcurve.domain_start(), pcurve.domain_end()};
    for (double bound : {extrusion.start, extrusion.start + extrusion.length}) {
        Point origin;
        Vector normal;
        origin[extrusion.direction] = bound;
        normal[extrusion.direction] = 1.0;
        const Plane plane = Plane::from_point_normal(origin, normal);
        const double error = 128.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, std::abs(bound));
        const std::vector<double> roots = Intersection::curve_plane(pcurve, plane, error);
        cuts.insert(cuts.end(), roots.begin(), roots.end());
    }
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
    return cuts;
}

bool append_section(
    std::vector<std::pair<NurbsCurve, NurbsCurve>>& sections,
    NurbsCurve world,
    NurbsCurve pcurve,
    double tolerance
) {
    if (!sections.empty()) {
        std::pair<NurbsCurve, NurbsCurve>& last = sections.back();
        if (last.first.domain_end() == world.domain_start()) {
            const std::vector<NurbsCurve> joined = NurbsCurve::join({last.first, world}, tolerance);
            const double scale = std::max(1.0, pcurve.point_at_start().distance({0, 0, 0}));
            const std::vector<NurbsCurve> joined_uv = NurbsCurve::join({last.second, pcurve}, scale * 128.0 * std::numeric_limits<double>::epsilon());
            if (joined.size() != 1 || joined_uv.size() != 1)
                return false;
            last = {joined[0], joined_uv[0]};
            return true;
        }
    }
    sections.push_back({std::move(world), std::move(pcurve)});
    return true;
}

bool clip_span(
    std::vector<std::pair<NurbsCurve, NurbsCurve>>& sections,
    const NurbsCurve& span,
    const Extrusion& extrusion,
    double tolerance
) {
    const std::optional<NurbsCurve> world = projected_world(span, extrusion, tolerance);
    const std::optional<NurbsCurve> pcurve = section_pcurve(span, extrusion);
    if (!world || !pcurve)
        return false;
    const std::vector<double> cuts = clip_parameters(*pcurve, extrusion);
    for (size_t i = 1; i < cuts.size(); ++i) {
        const Point uv = pcurve->point_at(std::midpoint(cuts[i - 1], cuts[i]));
        const double fraction = (uv[extrusion.direction] - extrusion.start) / extrusion.length;
        if (fraction < 0.0 || fraction > 1.0)
            continue;
        NurbsCurve section = *world;
        NurbsCurve parameters = *pcurve;
        if (!section.trim(cuts[i - 1], cuts[i]) || !parameters.trim(cuts[i - 1], cuts[i]))
            return false;
        if (!append_section(sections, std::move(section), std::move(parameters), tolerance))
            return false;
    }
    return true;
}

}

std::optional<std::vector<std::pair<NurbsCurve, NurbsCurve>>> extrusion_plane(
    const NurbsSurface& surface,
    const Plane& plane,
    double tolerance
) {
    if (!surface.is_valid() || !std::isfinite(tolerance) || tolerance <= 0.0 || !finite(plane.origin()))
        throw std::invalid_argument("Extrusion section requires valid geometry and a finite positive tolerance");
    Extrusion detected;
    if (!extrusion(surface, plane, tolerance, detected))
        return std::nullopt;
    const NurbsCurve base = surface.iso_curve(1 - detected.direction, detected.start);
    const std::vector<double> spans = base.get_span_vector();
    std::vector<std::pair<NurbsCurve, NurbsCurve>> result;
    for (size_t i = 1; i < spans.size(); ++i) {
        NurbsCurve span = base;
        if (!span.trim(spans[i - 1], spans[i]) || !clip_span(result, span, detected, tolerance))
            return std::nullopt;
    }
    for (std::pair<NurbsCurve, NurbsCurve>& section : result)
        if (!section.first.set_domain(0.0, 1.0) || !section.second.set_domain(0.0, 1.0))
            return std::nullopt;
    return result;
}

}
