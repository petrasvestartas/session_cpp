#include "surface_section.h"
#include "intersection.h"
#include "curve_surface_intersection.h"
#include "nurbs_coincidence.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
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

namespace {

bool polynomial_bilinear(const NurbsSurface& surface, std::array<Point, 4>& points) {
    if (!surface.is_valid() || surface.dimension() != 3 || surface.degree(0) != 1 || surface.degree(1) != 1 ||
        surface.cv_count(0) != 2 || surface.cv_count(1) != 2)
        return false;
    const double weight = surface.weight(0, 0);
    if (!(weight > 0) || !std::isfinite(weight))
        return false;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            if (surface.weight(i, j) != weight)
                return false;
            const Point point = surface.get_cv(i, j);
            for (int d = 0; d < 3; ++d)
                if (!nurbs_coincidence::exact_products_equal(point[d], weight, surface.cv(i, j)[d], 1))
                    return false;
            points[i + 2 * j] = point;
        }
    return true;
}

bool exact_vector(const Point& a, const Point& b, Vector& result) {
    for (int d = 0; d < 3; ++d)
        if (!nurbs_coincidence::exact_difference(a[d], b[d], result[d]))
            return false;
    return true;
}

bool parallel(const Vector& value, const Vector& normal, int axis) {
    for (int d = 0; d < 3; ++d)
        if (!nurbs_coincidence::exact_products_equal(value[d], normal[axis], value[axis], normal[d]))
            return false;
    return true;
}

bool projection_rank(const Vector& u, const Vector& v, const Vector& normal) {
    const nurbs_interval::Interval determinant =
        nurbs_interval::Interval(u[0]) *
            (nurbs_interval::Interval(v[1]) * nurbs_interval::Interval(normal[2]) -
             nurbs_interval::Interval(v[2]) * nurbs_interval::Interval(normal[1])) -
        nurbs_interval::Interval(u[1]) *
            (nurbs_interval::Interval(v[0]) * nurbs_interval::Interval(normal[2]) -
             nurbs_interval::Interval(v[2]) * nurbs_interval::Interval(normal[0])) +
        nurbs_interval::Interval(u[2]) *
            (nurbs_interval::Interval(v[0]) * nurbs_interval::Interval(normal[1]) -
             nurbs_interval::Interval(v[1]) * nurbs_interval::Interval(normal[0]));
    return nurbs_interval::finite(determinant) && (determinant.lo > 0 || determinant.hi < 0);
}

bool root_inside(double numerator, double denominator) {
    return denominator > 0 ? numerator > 0 && numerator < denominator
                           : numerator < 0 && numerator > denominator;
}

bool root_coverage(const std::array<Point, 4>& points, int fixed, nurbs_interval::Interval root, double raw,
                   const std::pair<double, double>& domain, double accuracy) {
    const nurbs_interval::Interval represented =
        (nurbs_interval::Interval(raw) - nurbs_interval::Interval(domain.first)) /
        (nurbs_interval::Interval(domain.second) - nurbs_interval::Interval(domain.first));
    const nurbs_interval::Interval gap = root - represented;
    if (!nurbs_interval::finite(gap))
        return false;
    const double delta = nurbs_interval::magnitude(gap);
    nurbs_interval::Interval squared;
    for (int d = 0; d < 3; ++d) {
        const nurbs_interval::Interval first =
            nurbs_interval::Interval(points[fixed == 0 ? 1 : 2][d]) - nurbs_interval::Interval(points[0][d]);
        const nurbs_interval::Interval last =
            nurbs_interval::Interval(points[3][d]) - nurbs_interval::Interval(points[fixed == 0 ? 2 : 1][d]);
        const double speed = std::max(nurbs_interval::magnitude(first), nurbs_interval::magnitude(last));
        const nurbs_interval::Interval error =
            nurbs_interval::Interval(delta) * nurbs_interval::Interval(speed);
        squared = squared + error * error;
    }
    const nurbs_interval::Interval limit =
        nurbs_interval::Interval(accuracy) * nurbs_interval::Interval(accuracy);
    return nurbs_interval::finite(squared) && squared.hi <= limit.lo;
}

NurbsCurve raw_line(const Point& start, const Point& end, const std::pair<double, double>& domain) {
    NurbsCurve curve(3, false, 2, 2);
    curve.m_nurbsknot = {domain.first, domain.second};
    if (!curve.set_cv(0, start) || !curve.set_cv(1, end) || !curve.is_valid())
        return {};
    return curve;
}

struct BilinearFactor {
    double numerators[2];
    double denominator;
};

bool bilinear_differences(const std::array<Point, 4>& a, const std::array<Point, 4>& b, int variant,
                          std::array<Vector, 4>& difference, Vector& normal) {
    const int axes[2] = {(variant & 4) ? 1 : 0, (variant & 4) ? 0 : 1};
    double largest = 0;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            int target[2];
            target[axes[0]] = (variant & 1) ? 1 - i : i;
            target[axes[1]] = (variant & 2) ? 1 - j : j;
            if (!exact_vector(a[i + 2 * j], b[target[0] + 2 * target[1]], difference[i + 2 * j]))
                return false;
            for (int d = 0; d < 3; ++d)
                if (std::abs(difference[i + 2 * j][d]) > largest) {
                    largest = std::abs(difference[i + 2 * j][d]);
                    normal = difference[i + 2 * j];
                }
        }
    return largest > 0;
}

std::optional<BilinearFactor> crossing_factor(const std::array<Point, 4>& a, const std::array<Point, 4>& b,
                                              const Vector& u, const Vector& v, const Vector& mixed,
                                              int variant) {
    std::array<Vector, 4> difference;
    Vector normal;
    if (!bilinear_differences(a, b, variant, difference, normal))
        return std::nullopt;
    int component = 0;
    for (int d = 1; d < 3; ++d)
        if (std::abs(normal[d]) > std::abs(normal[component]))
            component = d;
    for (const auto& value : difference)
        if (!parallel(value, normal, component))
            return std::nullopt;
    if (!parallel(mixed, normal, component) || !projection_rank(u, v, normal))
        return std::nullopt;
    const double constant = difference[0][component];
    double linear_u, linear_v, product, edge;
    if (!nurbs_coincidence::exact_difference(difference[1][component], constant, linear_u) ||
        !nurbs_coincidence::exact_difference(difference[2][component], constant, linear_v) ||
        !nurbs_coincidence::exact_difference(difference[3][component], difference[1][component], edge) ||
        !nurbs_coincidence::exact_difference(edge, linear_v, product) || product == 0 ||
        !nurbs_coincidence::exact_products_equal(constant, product, linear_u, linear_v))
        return std::nullopt;
    // Exact comparisons locate the rational roots; a near-factorable hyperbola
    // must fall through even when its perturbation is below modeling tolerance.
    BilinearFactor factor{{-linear_v, -linear_u}, product};
    if (!root_inside(factor.numerators[0], product) || !root_inside(factor.numerators[1], product))
        return std::nullopt;
    return factor;
}

bool mapped_raw_endpoints(const NurbsSurface& second, int variant, int fixed, double parameter, Point& start,
                          Point& end) {
    const int axes[2] = {(variant & 4) ? 1 : 0, (variant & 4) ? 0 : 1};
    for (int axis = 0; axis < 2; ++axis) {
        const auto domain = second.domain(axes[axis]);
        const double low = axis == fixed ? parameter : 0., high = axis == fixed ? parameter : 1.;
        const double p = (variant & (1 << axis)) ? 1 - low : low;
        const double q = (variant & (1 << axis)) ? 1 - high : high;
        start[axes[axis]] = std::lerp(domain.first, domain.second, p);
        end[axes[axis]] = std::lerp(domain.first, domain.second, q);
        if (axis == fixed && !(start[axes[axis]] > domain.first && start[axes[axis]] < domain.second))
            return false;
    }
    return true;
}

void observe_critical_failure(const NurbsSurface& first, const NurbsSurface& second, const char* stage,
                              nlohmann::ordered_json details, const NurbsCurve* world = nullptr,
                              const NurbsCurve* pa = nullptr, const NurbsCurve* pb = nullptr) {
    const char* path = std::getenv("SESSION_BILINEAR_CRITICAL_FAILURE");
    if (!path || !*path)
        return;
    try {
        details["stage"] = stage;
        details["first"] = first.jsondump();
        details["second"] = second.jsondump();
        if (world)
            details["world"] = world->jsondump();
        if (pa)
            details["first_pcurve"] = pa->jsondump();
        if (pb)
            details["second_pcurve"] = pb->jsondump();
        std::ofstream output(path, std::ios::binary | std::ios::app);
        if (!output || !(output << details.dump() << '\n'))
            throw std::runtime_error("Cannot append critical proof observation");
        output.flush();
        if (!output)
            throw std::runtime_error("Cannot flush critical proof observation");
        output.close();
        if (!output)
            throw std::runtime_error("Cannot close critical proof observation");
    } catch (const std::exception& error) {
        std::cerr << "Critical proof observation failed: " << error.what() << std::endl;
    } catch (...) {
        std::cerr << "Critical proof observation failed with unknown error" << std::endl;
    }
}

std::optional<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>>
crossing_branch(const NurbsSurface& first, const NurbsSurface& second, const std::array<Point, 4>& points,
                int variant, int fixed, const BilinearFactor& factor, double tolerance) {
    const int varying = 1 - fixed;
    const nurbs_interval::Interval root =
        nurbs_interval::Interval(factor.numerators[fixed]) / nurbs_interval::Interval(factor.denominator);
    const double parameter = factor.numerators[fixed] / factor.denominator;
    const auto fixed_domain = first.domain(fixed), varying_domain = first.domain(varying);
    const double raw = std::lerp(fixed_domain.first, fixed_domain.second, parameter);
    const double accuracy = tolerance * .25;
    if (!std::isfinite(parameter) || !(raw > fixed_domain.first && raw < fixed_domain.second) ||
        !(accuracy > 0) || !root_coverage(points, fixed, root, raw, fixed_domain, accuracy)) {
        observe_critical_failure(first, second, "root_or_range",
                                 {{"variant", variant},
                                  {"fixed", fixed},
                                  {"parameter", parameter},
                                  {"raw", raw},
                                  {"root", {root.lo, root.hi}},
                                  {"accuracy", accuracy}});
        return std::nullopt;
    }
    Point start, end, target_start, target_end;
    start[fixed] = end[fixed] = raw;
    start[varying] = varying_domain.first;
    end[varying] = varying_domain.second;
    if (!mapped_raw_endpoints(second, variant, fixed, parameter, target_start, target_end)) {
        observe_critical_failure(
            first, second, "mapped_raw_endpoints",
            {{"variant", variant}, {"fixed", fixed}, {"parameter", parameter}, {"accuracy", accuracy}});
        return std::nullopt;
    }
    NurbsCurve world = first.iso_curve(varying, raw);
    NurbsCurve pa = raw_line(start, end, varying_domain),
               pb = raw_line(target_start, target_end, varying_domain);
    if (!world.is_valid() || world.domain() != varying_domain || !pa.is_valid() || !pb.is_valid()) {
        observe_critical_failure(
            first, second, "raw_curves",
            {{"variant", variant}, {"fixed", fixed}, {"parameter", parameter}, {"accuracy", accuracy}},
            &world, &pa, &pb);
        return std::nullopt;
    }
    const char* capture = std::getenv("SESSION_BILINEAR_CRITICAL_FAILURE");
    const bool observing = capture && *capture;
    std::string diagnostic;
    const auto match_a = curve_surface_intersection::same_parameter(world, pa, first, accuracy,
                                                                    observing ? &diagnostic : nullptr);
    if (match_a != curve_surface_intersection::ParameterMatch::Matched) {
        observe_critical_failure(first, second, "first_certificate",
                                 {{"variant", variant},
                                  {"fixed", fixed},
                                  {"parameter", parameter},
                                  {"accuracy", accuracy},
                                  {"match", static_cast<int>(match_a)},
                                  {"diagnostic", diagnostic}},
                                 &world, &pa, &pb);
        return std::nullopt;
    }
    const auto match_b = curve_surface_intersection::same_parameter(world, pb, second, accuracy,
                                                                    observing ? &diagnostic : nullptr);
    if (match_b != curve_surface_intersection::ParameterMatch::Matched) {
        observe_critical_failure(first, second, "second_certificate",
                                 {{"variant", variant},
                                  {"fixed", fixed},
                                  {"parameter", parameter},
                                  {"accuracy", accuracy},
                                  {"match", static_cast<int>(match_b)},
                                  {"diagnostic", diagnostic}},
                                 &world, &pa, &pb);
        return std::nullopt;
    }
    return std::make_tuple(std::move(world), std::move(pa), std::move(pb));
}

struct CoveragePlaneLaw {
    Point origin;
    Vector u,v;
    std::array<std::pair<double,double>,2> domain;
    std::array<double,2> width;
};
bool coverage_plane_law(const NurbsSurface& surface,CoveragePlaneLaw& law) {
    std::array<Point,4> corners;
    Vector opposite;
    if (!polynomial_bilinear(surface,corners) || !exact_vector(corners[1],corners[0],law.u) ||
        !exact_vector(corners[2],corners[0],law.v) || !exact_vector(corners[3],corners[1],opposite))
        return false;
    for (int d=0;d<3;++d) if (opposite[d]!=law.v[d]) return false;
    law.origin=corners[0];
    for (int d=0;d<2;++d) {
        law.domain[d]=surface.domain(d);
        if (!nurbs_coincidence::exact_difference(law.domain[d].second,law.domain[d].first,law.width[d]) ||
            !(law.width[d]>0)) return false;
    }
    return true;
}
std::array<nurbs_interval::Interval,3> coverage_cross(
    const std::array<nurbs_interval::Interval,3>& a,const std::array<nurbs_interval::Interval,3>& b) {
    std::array<nurbs_interval::Interval,3> result;
    for (int d=0;d<3;++d) result[d]=a[(d+1)%3]*b[(d+2)%3]-a[(d+2)%3]*b[(d+1)%3];
    return result;
}
bool coverage_transverse(const CoveragePlaneLaw& a,const CoveragePlaneLaw& b) {
    const auto vector=[](const Vector& value) {
        return std::array<nurbs_interval::Interval,3>{nurbs_interval::Interval(value[0]),
            nurbs_interval::Interval(value[1]),nurbs_interval::Interval(value[2])};
    };
    const auto cross=coverage_cross(coverage_cross(vector(a.u),vector(a.v)),
                                    coverage_cross(vector(b.u),vector(b.v)));
    bool nonzero=false;
    for (const auto coordinate:cross) {
        if (!nurbs_interval::finite(coordinate)) return false;
        nonzero=nonzero || coordinate.lo>0 || coordinate.hi<0;
    }
    return nonzero; // Two injective original affine charts meet in one infinite line.
}
bool coverage_linear_law(const NurbsCurve& curve,int dimension,std::array<Point,2>& endpoints) {
    if (!curve.is_valid() || curve.dimension()<dimension || curve.dimension()>3 ||
        curve.degree()!=1 || curve.cv_count()!=2 || !(curve.domain_start()<curve.domain_end()) ||
        !std::isfinite(curve.domain_end()-curve.domain_start())) return false;
    const double weight=curve.weight(0);
    if (!std::isfinite(weight) || !(weight>0) || curve.weight(1)!=weight) return false;
    for (int i=0;i<2;++i) for (int d=0;d<dimension;++d) {
        const double value=curve.cv(i)[d]/weight;
        if (!std::isfinite(value) || !nurbs_coincidence::exact_products_equal(value,weight,curve.cv(i)[d],1))
            return false;
        endpoints[i][d]=value;
    }
    return true;
}
bool coverage_exact_point(const CoveragePlaneLaw& law,const Point& uv,const Point& world) {
    std::array<double,2> fraction;
    for (int d=0;d<2;++d) {
        double numerator;
        if (uv[d]<law.domain[d].first || uv[d]>law.domain[d].second ||
            !nurbs_coincidence::exact_difference(uv[d],law.domain[d].first,numerator)) return false;
        fraction[d]=numerator/law.width[d];
        if (!std::isfinite(fraction[d]) ||
            !nurbs_coincidence::exact_products_equal(fraction[d],law.width[d],numerator,1)) return false;
    }
    for (int d=0;d<3;++d) {
        const double u=law.u[d]*fraction[0],v=law.v[d]*fraction[1];
        double sum,value;
        if (!std::isfinite(u) || !std::isfinite(v) ||
            !nurbs_coincidence::exact_products_equal(law.u[d],fraction[0],u,1) ||
            !nurbs_coincidence::exact_products_equal(law.v[d],fraction[1],v,1) ||
            !nurbs_coincidence::exact_difference(law.origin[d],-u,sum) ||
            !nurbs_coincidence::exact_difference(sum,-v,value) || value!=world[d]) return false;
    }
    return true;
}
void coverage_domain_ends(const CoveragePlaneLaw& law,const std::array<Point,2>& uv,
                          bool& first,bool& last) {
    for (int d=0;d<2;++d) {
        const bool increasing=uv[1][d]>uv[0][d],decreasing=uv[1][d]<uv[0][d];
        first=first || (increasing && uv[0][d]==law.domain[d].first) ||
                       (decreasing && uv[0][d]==law.domain[d].second);
        last=last || (increasing && uv[1][d]==law.domain[d].second) ||
                     (decreasing && uv[1][d]==law.domain[d].first);
    }
}

} // namespace

std::optional<std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>>>
bilinear_crossing(const NurbsSurface& first, const NurbsSurface& second, double tolerance) {
    if (!std::isfinite(tolerance) || !(tolerance > 0))
        return std::nullopt;
    std::array<Point, 4> a, b;
    if (!polynomial_bilinear(first, a) || !polynomial_bilinear(second, b)) {
        observe_critical_failure(first, second, "polynomial_bilinear", {{"tolerance", tolerance}});
        return std::nullopt;
    }
    Vector u, v, mixed;
    if (!exact_vector(a[1], a[0], u) || !exact_vector(a[2], a[0], v)) {
        observe_critical_failure(first, second, "affine_frame", {{"tolerance", tolerance}});
        return std::nullopt;
    }
    for (int d = 0; d < 3; ++d) {
        double edge;
        if (!nurbs_coincidence::exact_difference(a[3][d], a[1][d], edge) ||
            !nurbs_coincidence::exact_difference(edge, v[d], mixed[d])) {
            observe_critical_failure(first, second, "mixed_exact",
                                     {{"tolerance", tolerance}, {"coordinate", d}});
            return std::nullopt;
        }
    }
    for (int variant = 0; variant < 8; ++variant) {
        const auto factor = crossing_factor(a, b, u, v, mixed, variant);
        if (!factor)
            continue;
        auto first_branch = crossing_branch(first, second, a, variant, 0, *factor, tolerance);
        if (!first_branch)
            continue;
        auto second_branch = crossing_branch(first, second, a, variant, 1, *factor, tolerance);
        if (second_branch)
            return std::vector<std::tuple<NurbsCurve, NurbsCurve, NurbsCurve>>{std::move(*first_branch),
                                                                               std::move(*second_branch)};
    }
    observe_critical_failure(first, second, "no_proved_correspondence", {{"tolerance", tolerance}});
    return std::nullopt;
}

bool affine_plane_curve_coverage(const NurbsSurface& first,const NurbsSurface& second,
    const std::vector<std::tuple<NurbsCurve,NurbsCurve,NurbsCurve>>& sections) {
    if (sections.size()!=1) return false;
    CoveragePlaneLaw a,b;
    std::array<Point,2> world,ua,ub;
    const auto& [curve,pa,pb]=sections.front();
    if (!coverage_plane_law(first,a) || !coverage_plane_law(second,b) || !coverage_transverse(a,b) ||
        curve.domain()!=pa.domain() || curve.domain()!=pb.domain() ||
        !coverage_linear_law(curve,3,world) || !coverage_linear_law(pa,2,ua) ||
        !coverage_linear_law(pb,2,ub)) return false;
    if (world[0][0]==world[1][0] && world[0][1]==world[1][1] && world[0][2]==world[1][2]) return false;
    for (int i=0;i<2;++i)
        if (!coverage_exact_point(a,ua[i],world[i]) || !coverage_exact_point(b,ub[i],world[i])) return false;
    bool start=false,end=false;
    coverage_domain_ends(a,ua,start,end);coverage_domain_ends(b,ub,start,end);
    return start && end; // Convex rectangle clips the unique line to exactly this segment.
}

} // namespace session_cpp::surface_section
