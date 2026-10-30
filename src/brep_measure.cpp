#include "brep_measure.h"
#include "brep_measure_polynomial.h"
#include "nurbsknot.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

namespace session_cpp::brep_measure {
namespace {

constexpr std::array<double, 8> nodes = {0.09501250983763744, 0.2816035507792589,
    0.4580167776572274, 0.6178762444026438, 0.7554044083550030, 0.8656312023878318,
    0.9445750230732326, 0.9894009349916499};
constexpr std::array<double, 8> weights = {0.1894506104550685, 0.1826034150449236,
    0.1691565193950025, 0.1495959888165767, 0.1246289712555339, 0.0951585116824928,
    0.06225352393864789, 0.02715245941175409};
constexpr std::array<double, 4> coarse_nodes = {0.1834346424956498, 0.5255324099163290,
    0.7966664774136267, 0.9602898564975363};
constexpr std::array<double, 4> coarse_weights = {0.3626837833783620, 0.3137066458778873,
    0.2223810344533745, 0.1012285362903763};

template<size_t N>
double gauss(const std::function<double(double)>& function, double first, double last,
             const std::array<double, N>& points, const std::array<double, N>& coefficients) {
    const double middle = (first + last) * 0.5, half = (last - first) * 0.5;
    double sum = 0.0;
    for (size_t i = 0; i < N; ++i)
        sum += coefficients[i] * (function(middle - half * points[i]) + function(middle + half * points[i]));
    return half * sum;
}

struct Work {
    size_t total = 0;
    size_t inner = 0;
    size_t outer = 0;
    size_t polynomial_coefficients = 0;
    size_t inner_subdivisions = 0;
    size_t outer_subdivisions = 0;
    int inner_depth = 0;
    int outer_depth = 0;
};

double integrate(const std::function<double(double)>& function, double first, double last,
                 double tolerance, Work& work, int depth = 0, bool inner = false) {
    if (first == last)
        return 0.0;
    ++(inner ? work.inner : work.outer);
    int& maximum_depth = inner ? work.inner_depth : work.outer_depth;
    maximum_depth = std::max(maximum_depth, depth);
    if (++work.total > 200000)
        throw std::runtime_error("Native BRep volume exceeded its quadrature budget");
    const double fine = gauss(function, first, last, nodes, weights);
    const double coarse = gauss(function, first, last, coarse_nodes, coarse_weights);
    if (!std::isfinite(fine) || !std::isfinite(coarse))
        throw std::runtime_error("Native BRep volume encountered a nonfinite integrand");
    if (std::abs(fine - coarse) <= tolerance)
        return fine;
    ++(inner ? work.inner_subdivisions : work.outer_subdivisions);
    const double middle = (first + last) * 0.5;
    if (depth >= 22 || middle == first || middle == last)
        throw std::runtime_error("Native BRep volume did not converge at the requested tolerance");
    return integrate(function, first, middle, tolerance * 0.5, work, depth + 1, inner) +
        integrate(function, middle, last, tolerance * 0.5, work, depth + 1, inner);
}


template<size_t N>
nurbs_interval::Interval enclosed_gauss(
    const std::function<nurbs_interval::Interval(double)>& function, double first, double last,
    const std::array<double, N>& points, const std::array<double, N>& coefficients) {
    const double middle = (first + last) * 0.5, half = (last - first) * 0.5;
    nurbs_interval::Interval sum;
    for (size_t i = 0; i < N; ++i)
        sum = sum + nurbs_interval::Interval(coefficients[i]) *
            (function(middle - half * points[i]) + function(middle + half * points[i]));
    return nurbs_interval::Interval(half) * sum;
}

double integrate_polynomial(const std::function<nurbs_interval::Interval(double)>& function,
    double first, double last, double tolerance, Work& work, int depth = 0) {
    if (first == last)
        return 0.0;
    ++work.outer;
    work.outer_depth = std::max(work.outer_depth, depth);
    if (++work.total > 200000)
        throw std::runtime_error("Native BRep volume exceeded its quadrature budget");
    const nurbs_interval::Interval fine_box = enclosed_gauss(function, first, last, nodes, weights);
    const nurbs_interval::Interval coarse_box = enclosed_gauss(function, first, last, coarse_nodes, coarse_weights);
    if (!nurbs_interval::finite(fine_box) || !nurbs_interval::finite(coarse_box))
        throw std::runtime_error("Native BRep volume encountered a nonfinite integrand");
    const double fine = std::midpoint(fine_box.lo, fine_box.hi);
    if (polynomial::resolved(fine_box, coarse_box, tolerance))
        return fine;
    ++work.outer_subdivisions;
    const double middle = (first + last) * 0.5;
    if (depth >= 22 || middle == first || middle == last)
        throw std::runtime_error("Native BRep volume did not converge at the requested tolerance");
    return integrate_polynomial(function, first, middle, tolerance * 0.5, work, depth + 1) +
        integrate_polynomial(function, middle, last, tolerance * 0.5, work, depth + 1);
}

struct Frame {
    Point origin;
    double scale;
};

Frame frame(const BRep& brep) {
    Point lo(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    Point hi(-lo[0], -lo[1], -lo[2]);
    for (const BRepSolid& solid : brep.m_solids)
        for (const BRepRef& shell : solid.shells)
            for (const BRepRef& face : brep.m_shells[shell.index].faces) {
                const NurbsSurface& surface = brep.m_surfaces[brep.m_faces[face.index].surface_index];
                for (int i = 0; i < surface.cv_count(0); ++i)
                    for (int j = 0; j < surface.cv_count(1); ++j) {
                        if (!(surface.weight(i, j) > 0.0))
                            throw std::invalid_argument("Native BRep volume requires positive rational weights");
                        const Point point = surface.get_cv(i, j);
                        for (int d = 0; d < 3; ++d) {
                            lo[d] = std::min(lo[d], point[d]);
                            hi[d] = std::max(hi[d], point[d]);
                        }
                    }
            }
    const double scale = lo.distance(hi);
    if (!std::isfinite(scale) || scale <= 0.0)
        throw std::runtime_error("Native BRep volume has degenerate bounds");
    return {lo + (hi - lo) * 0.5, scale};
}

NurbsSurface local_surface(const NurbsSurface& surface, const Frame& frame) {
    NurbsSurface result = surface;
    for (int i = 0; i < surface.cv_count(0); ++i)
        for (int j = 0; j < surface.cv_count(1); ++j) {
            double x, y, z, w;
            surface.get_cv_4d(i, j, x, y, z, w);
            result.set_cv_4d(i, j, std::fma(-frame.origin[0], w, x) / frame.scale,
                std::fma(-frame.origin[1], w, y) / frame.scale,
                std::fma(-frame.origin[2], w, z) / frame.scale, w);
        }
    return result;
}

struct SurfaceEvaluator {
    const NurbsSurface& surface;
    NurbsSurface local;
    std::array<int, 2> active{-1, -1};
    std::array<std::vector<double>, 2> knots;

    explicit SurfaceEvaluator(const NurbsSurface& source)
        : surface(source), local(3, source.is_rational(), source.order(0), source.order(1), source.order(0), source.order(1)),
          knots{source.get_span_vector(0), source.get_span_vector(1)} {
        if (!source.is_valid() || source.dimension() != 3)
            throw std::runtime_error("Native BRep volume requires a valid three-dimensional support");
    }

    double integrand(double u, double v) {
        const std::array<int, 2> span = {
            nurbsknot::find_span(surface.order(0), surface.cv_count(0), surface.m_nurbsknot[0], u),
            nurbsknot::find_span(surface.order(1), surface.cv_count(1), surface.m_nurbsknot[1], v)
        };
        if (span != active) {
            for (int dir = 0; dir < 2; ++dir)
                std::copy_n(surface.m_nurbsknot[dir].begin() + span[dir], local.m_nurbsknot[dir].size(), local.m_nurbsknot[dir].begin());
            for (int i = 0; i < local.cv_count(0); ++i)
                for (int j = 0; j < local.cv_count(1); ++j)
                    std::copy_n(surface.cv(span[0] + i, span[1] + j), surface.cv_size(), local.cv(i, j));
            active = span;
        }
        const std::vector<Vector> derivatives = local.evaluate(u, v, 1);
        if (derivatives.size() != 3)
            throw std::runtime_error("Native BRep volume could not evaluate a local surface span");
        return derivatives[0].dot(derivatives[2].cross(derivatives[1])) / 3.0;
    }
};

double primitive(SurfaceEvaluator& surface, double u, double v, double tolerance, Work& work) {
    const std::vector<double>& knots = surface.knots[0];
    double value = 0.0;
    for (size_t i = 1; i < knots.size() && knots[i - 1] < u; ++i)
        value += integrate([&](double parameter) { return surface.integrand(parameter, v); },
            knots[i - 1], std::min(u, knots[i]), tolerance / knots.size(), work, 0, true);
    return value;
}

std::vector<double> boundary_parameters(const NurbsCurve& curve, const std::vector<double>& vknots) {
    std::vector<double> parameters = curve.get_span_vector();
    if (curve.degree() != 1 || curve.cv_count() != 2)
        return parameters;
    const double scale = std::max(curve.weight(0), curve.weight(1));
    const double first_weight = curve.weight(0) / scale;
    const double last_weight = curve.weight(1) / scale;
    if (!(first_weight > 0) || !(last_weight > 0) || !std::isfinite(scale))
        throw std::invalid_argument("Native BRep volume requires positive finite pcurve weights");
    const double first = curve.get_cv(0)[1];
    const double last = curve.get_cv(1)[1];
    for (double knot : vknots) {
        if (knot <= std::min(first, last) || knot >= std::max(first, last))
            continue;
        const double a = first_weight * (first - knot);
        const double b = last_weight * (last - knot);
        const double fraction = -a / (b - a);
        const double parameter = curve.domain_start() + fraction * (curve.domain_end() - curve.domain_start());
        if (parameter > curve.domain_start() && parameter < curve.domain_end())
            parameters.push_back(parameter);
    }
    std::sort(parameters.begin(), parameters.end());
    parameters.erase(std::unique(parameters.begin(), parameters.end()), parameters.end());
    return parameters;
}

int rectangle_corner(const Point& point, const NurbsSurface& surface) {
    const std::pair<double, double> u = surface.domain(0);
    const std::pair<double, double> v = surface.domain(1);
    if (point[0] == u.first && point[1] == v.first)
        return 0;
    if (point[0] == u.second && point[1] == v.first)
        return 1;
    if (point[0] == u.second && point[1] == v.second)
        return 2;
    if (point[0] == u.first && point[1] == v.second)
        return 3;
    return -1;
}

int rectangle_orientation(const BRep& brep, int face, const NurbsSurface& surface) {
    const BRepFace& boundary = brep.m_faces[face];
    if (boundary.wires.size() != 1)
        return 0;
    const std::vector<BRepRef> edges = brep.wire_edges(boundary.wires[0]);
    if (edges.size() != 4)
        return 0;
    int orientation = 0;
    unsigned int corners = 0;
    for (const BRepRef& edge : edges) {
        if (edge.orientation != BRepOrientation::Forward && edge.orientation != BRepOrientation::Reversed)
            return 0;
        const int index = brep.pcurve_index(edge.index, face, edge.orientation);
        if (index < 0)
            return 0;
        const NurbsCurve& curve = brep.m_curves_2d[index];
        if (curve.degree() != 1 || curve.cv_count() != 2 || !(curve.weight(0) > 0) || !(curve.weight(1) > 0))
            return 0;
        const bool reverse = edge.orientation == BRepOrientation::Reversed;
        const int first = rectangle_corner(curve.get_cv(reverse ? 1 : 0), surface);
        const int last = rectangle_corner(curve.get_cv(reverse ? 0 : 1), surface);
        if (first < 0 || last < 0 || (corners & (1u << first)))
            return 0;
        const int sign = last == (first + 1) % 4 ? 1 : (last == (first + 3) % 4 ? -1 : 0);
        if (!sign || (orientation && sign != orientation))
            return 0;
        orientation = sign;
        corners |= 1u << first;
    }
    return corners == 15 ? orientation : 0;
}

std::pair<double, double> legendre(int degree, double parameter) {
    double previous = 1;
    double value = parameter;
    for (int i = 2; i <= degree; ++i) {
        const double next = ((2 * i - 1) * parameter * value - (i - 1) * previous) / i;
        previous = value;
        value = next;
    }
    return {value, degree * (parameter * value - previous) / (parameter * parameter - 1)};
}

std::vector<std::pair<double, double>> polynomial_rule(int degree) {
    const int count = (3 * degree + 1) / 2;
    std::vector<std::pair<double, double>> result(count);
    for (int i = 0; i < (count + 1) / 2; ++i) {
        double point = std::cos(Tolerance::PI * (i + 0.75) / (count + 0.5));
        bool converged = false;
        for (int step = 0; step < 24; ++step) {
            const std::pair<double, double> value = legendre(count, point);
            const double change = value.first / value.second;
            point -= change;
            if (std::abs(change) <= 4 * std::numeric_limits<double>::epsilon()) {
                converged = true;
                break;
            }
        }
        if (!converged)
            throw std::runtime_error("Native BRep polynomial quadrature rule did not converge");
        const double derivative = legendre(count, point).second;
        const double weight = 2 / ((1 - point * point) * derivative * derivative);
        result[i] = {-point, weight};
        result[count - 1 - i] = {point, weight};
    }
    return result;
}

double polynomial_volume(SurfaceEvaluator& surface, int orientation, Work& work) {
    const std::vector<std::pair<double, double>> u_rule = polynomial_rule(surface.surface.degree(0));
    const std::vector<std::pair<double, double>> v_rule = polynomial_rule(surface.surface.degree(1));
    const std::vector<double>& u = surface.knots[0];
    const std::vector<double>& v = surface.knots[1];
    double value = 0;
    double compensation = 0;
    for (size_t i = 1; i < u.size(); ++i)
        for (size_t j = 1; j < v.size(); ++j) {
            if (++work.total > 200000)
                throw std::runtime_error("Native BRep volume exceeded its quadrature budget");
            const double du = (u[i] - u[i - 1]) * 0.5;
            const double dv = (v[j] - v[j - 1]) * 0.5;
            const double um = u[i - 1] + du;
            const double vm = v[j - 1] + dv;
            double cell = 0;
            for (const std::pair<double, double>& a : u_rule)
                for (const std::pair<double, double>& b : v_rule)
                    cell += a.second * b.second * surface.integrand(um + du * a.first, vm + dv * b.first);
            const double contribution = du * dv * cell - compensation;
            const double next = value + contribution;
            compensation = (next - value) - contribution;
            value = next;
        }
    return orientation * value;
}

double face_volume(const BRep& brep, int face, const Frame& frame, double tolerance, Work& work) {
    const BRepFace& boundary = brep.m_faces[face];
    const NurbsSurface surface = local_surface(brep.m_surfaces[boundary.surface_index], frame);
    SurfaceEvaluator evaluator(surface);
    const int rectangle = rectangle_orientation(brep, face, surface);
    if (!surface.is_rational() && rectangle && surface.degree(0) <= 21 && surface.degree(1) <= 21)
        return polynomial_volume(evaluator, rectangle, work);
    // Dense polynomial antiderivatives are an optional acceleration. If their
    // allocation/work estimate cannot fit, use the existing adaptive path.
    std::optional<polynomial::Primitive> cache;
    if (!surface.is_rational() && surface.degree(0) >= 1 && surface.degree(1) >= 1
        && surface.degree(0) <= 3 && surface.degree(1) <= 3) {
        const size_t per_cell = 54 * surface.degree(0) * surface.degree(1)
            + 3 * surface.degree(0) + 3 * surface.degree(1);
        const size_t nu = evaluator.knots[0].size() - 1;
        const size_t nv = evaluator.knots[1].size() - 1;
        if (nu && nv && nu <= ((200000 - work.total) / per_cell) / nv)
            cache.emplace(surface, 200000 - work.total, [&]() {
                ++work.polynomial_coefficients;
                if (++work.total > 200000)
                    throw std::runtime_error("Native BRep volume exceeded its quadrature budget");
            });
    }
    double value = 0.0;
    for (const BRepRef& wire : boundary.wires)
        for (const BRepRef& edge : brep.wire_edges(wire)) {
            const int index = brep.pcurve_index(edge.index, face, edge.orientation);
            if (index < 0)
                throw std::runtime_error("Native BRep volume requires every face pcurve");
            const NurbsCurve& curve = brep.m_curves_2d[index];
            const std::vector<double> knots = boundary_parameters(curve, evaluator.knots[1]);
            const double sign = edge.orientation == BRepOrientation::Reversed ? -1.0 : 1.0;
            for (size_t i = 1; i < knots.size(); ++i) {
                const double allowed = tolerance / (knots.size() * boundary.wires.size());
                if (cache && !cache->cells.empty())
                    value += sign * integrate_polynomial([&](double parameter) {
                        const auto d = curve.evaluate(parameter, 1);
                        if (d[1][1] == 0.0)
                            return nurbs_interval::Interval();
                        const nurbs_interval::Interval primitive = cache->at(d[0][0], d[0][1]);
                        const nurbs_interval::Interval integrand = primitive * nurbs_interval::Interval(d[1][1]);
                        if (!nurbs_interval::finite(integrand))
                            throw std::runtime_error("Native polynomial volume has a nonfinite boundary integrand");
                        return integrand;
                    }, knots[i - 1], knots[i], allowed, work);
                else
                    value += sign * integrate([&](double parameter) {
                        const auto d = curve.evaluate(parameter, 1);
                        if (d[1][1] == 0.0)
                            return 0.0;
                        return primitive(evaluator, d[0][0], d[0][1], tolerance * 0.05, work) * d[1][1];
                    }, knots[i - 1], knots[i], allowed, work);
            }
        }
    return value;
}

double sum_volume(const BRep& brep, const Frame& frame, double tolerance) {
    Work work;
    double volume = 0.0;
    for (const BRepSolid& solid : brep.m_solids)
        for (const BRepRef& shell : solid.shells)
            for (const BRepRef& face : brep.m_shells[shell.index].faces) {
                const auto orientation = brep_compose(shell.orientation, face.orientation);
                if (orientation == BRepOrientation::Internal || orientation == BRepOrientation::External)
                    continue;
                const double sign = orientation == BRepOrientation::Reversed ? -1.0 : 1.0;
                const Work before = work;
                try {
                    volume += sign * face_volume(brep, face.index, frame, tolerance / brep.face_count(), work);
                } catch (const std::runtime_error& error) {
                    throw std::runtime_error(std::string(error.what()) + "; face=" + std::to_string(face.index)
                        + ", total_work=" + std::to_string(work.total)
                        + ", polynomial_coefficients=" + std::to_string(work.polynomial_coefficients)
                        + ", face_inner=" + std::to_string(work.inner - before.inner)
                        + ", face_outer=" + std::to_string(work.outer - before.outer)
                        + ", inner_subdivisions=" + std::to_string(work.inner_subdivisions)
                        + ", outer_subdivisions=" + std::to_string(work.outer_subdivisions)
                        + ", inner_max_depth=" + std::to_string(work.inner_depth)
                        + ", outer_max_depth=" + std::to_string(work.outer_depth));
                }
            }
    return volume;
}

}

double volume(const BRep& brep, double tolerance) {
    if (!std::isfinite(tolerance) || tolerance <= 0.0)
        throw std::invalid_argument("Native BRep volume requires positive relative tolerance");
    if (brep.face_count() == 0)
        return 0.0;
    if (!brep.is_valid() || !brep.is_solid())
        throw std::invalid_argument("Native BRep volume requires a closed oriented solid");
    const Frame bounds = frame(brep);
    const double estimate = sum_volume(brep, bounds, tolerance * 0.01);
    const double requested = tolerance * std::abs(estimate) * 0.1;
    if (requested < 64 * std::numeric_limits<double>::epsilon() * std::abs(estimate))
        throw std::runtime_error("Native BRep volume tolerance is below numerical resolution");
    const double refined = sum_volume(brep, bounds, requested);
    return refined * bounds.scale * bounds.scale * bounds.scale;
}

}
