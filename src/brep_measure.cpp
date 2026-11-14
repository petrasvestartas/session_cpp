#include "brep_measure.h"
#include "brep_measure_polynomial.h"
#include "brep_measure_plane.h"
#include "nurbsknot.h"
#include <algorithm>
#include <tuple>
#include <map>
#include <cstdint>
#include <bit>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
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

// Higher-order inner retry avoids bisecting smooth rational span integrals.
constexpr std::array<double, 16> higher_nodes = {0.04830766568773831, 0.14447196158279649, 0.23928736225213706,
    0.33186860228212767, 0.42135127613063533, 0.50689990893222936,
    0.5877157572407623, 0.66304426693021523, 0.73218211874028971,
    0.79448379596794239, 0.84936761373256997, 0.89632115576605209,
    0.93490607593773967, 0.96476225558750639, 0.98561151154526838,
    0.99726386184948157};
constexpr std::array<double, 16> higher_weights = {0.096540088514727854, 0.095638720079274847, 0.093844399080804539,
    0.091173878695763905, 0.087652093004403742, 0.083311924226946721,
    0.078193895787070436, 0.072345794108848616, 0.065822222776361947,
    0.058684093478535579, 0.050998059262376154, 0.042835898022226704,
    0.034273862913021411, 0.025392065309262139, 0.016274394730905709,
    0.0070186100094701363};

template<typename Function, size_t N>
double gauss(const Function& function, double first, double last,
             const std::array<double, N>& points, const std::array<double, N>& coefficients) {
    const double middle = (first + last) * 0.5, half = (last - first) * 0.5;
    double sum = 0.0;
    for (size_t i = 0; i < N; ++i) {
        const double below = function(middle - half * points[i]);
        const double above = function(middle + half * points[i]);
        sum += coefficients[i] * (below + above);
    }
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

template<typename Function>
double integrate(const Function& function, double first, double last,
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
    if (inner) {
        // This is an additional quadrature pass, not free work or a larger budget.
        ++work.inner;
        if (++work.total > 200000)
            throw std::runtime_error("Native BRep volume exceeded its quadrature budget");
        const double higher = gauss(function, first, last, higher_nodes, higher_weights);
        if (!std::isfinite(higher))
            throw std::runtime_error("Native BRep volume encountered a nonfinite integrand");
        if (std::abs(higher - fine) <= tolerance)
            return higher;
    }
    ++(inner ? work.inner_subdivisions : work.outer_subdivisions);
    const double middle = (first + last) * 0.5;
    if (depth >= 22 || middle == first || middle == last)
        throw std::runtime_error("Native BRep volume did not converge at the requested tolerance");
    return integrate(function, first, middle, tolerance * 0.5, work, depth + 1, inner) +
        integrate(function, middle, last, tolerance * 0.5, work, depth + 1, inner);
}


template<typename Function, size_t N>
nurbs_interval::Interval enclosed_gauss(
    const Function& function, double first, double last,
    const std::array<double, N>& points, const std::array<double, N>& coefficients) {
    const double middle = (first + last) * 0.5, half = (last - first) * 0.5;
    nurbs_interval::Interval sum;
    for (size_t i = 0; i < N; ++i) {
        const nurbs_interval::Interval above = function(middle + half * points[i]);
        const nurbs_interval::Interval below = function(middle - half * points[i]);
        sum = sum + nurbs_interval::Interval(coefficients[i]) * (below + above);
    }
    return nurbs_interval::Interval(half) * sum;
}

template<typename Function>
double integrate_polynomial(const Function& function,
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

constexpr int patch_order = 16;

/// Bernstein values and derivatives of the degree at s.
inline void bernstein(int degree, double s, double* value, double* derivative) {
    double lower[patch_order];
    lower[0] = 1.0;
    for (int k = 1; k < degree; ++k) {
        lower[k] = s * lower[k - 1];
        for (int i = k - 1; i > 0; --i)
            lower[i] = (1.0 - s) * lower[i] + s * lower[i - 1];
        lower[0] = (1.0 - s) * lower[0];
    }
    for (int i = 0; i <= degree; ++i) {
        const double before = i > 0 ? lower[i - 1] : 0.0;
        const double here = i < degree ? lower[i] : 0.0;
        value[i] = (1.0 - s) * here + s * before;
        derivative[i] = degree * (before - here);
    }
}

/// z times the z of the normal of a Bezier patch of controls x y z w over cell at (u, v); P and Q above zero are the degrees known at compile time. False where the weight vanishes.
template <int P, int Q>
inline bool patch_flux(const double* control, int degree_u, int degree_v, bool rational, const std::array<double, 4>& cell,
    double u, double v, double& flux) {
    const int p = P ? P : degree_u, q = Q ? Q : degree_v;
    const double du = cell[1] - cell[0], dv = cell[3] - cell[2];
    double bu[patch_order], gu[patch_order], bv[patch_order], gv[patch_order];
    bernstein(p, (u - cell[0]) / du, bu, gu);
    bernstein(q, (v - cell[2]) / dv, bv, gv);
    // The homogeneous point and its partials, rows along v first; x y w is all the normal's z takes.
    double point[4] = {0.0, 0.0, 0.0, 0.0}, along_u[4] = {0.0, 0.0, 0.0, 0.0}, along_v[4] = {0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i <= p; ++i) {
        double row[4] = {0.0, 0.0, 0.0, 0.0}, slope[4] = {0.0, 0.0, 0.0, 0.0};
        for (int j = 0; j <= q; ++j, control += 4) {
            for (int d = 0; d < 4; ++d)
                row[d] += bv[j] * control[d];
            for (int d : {0, 1, 3})
                slope[d] += gv[j] * control[d];
        }
        for (int d = 0; d < 4; ++d)
            point[d] += bu[i] * row[d];
        for (int d : {0, 1, 3}) {
            along_u[d] += gu[i] * row[d];
            along_v[d] += bu[i] * slope[d];
        }
    }
    if (!rational) {
        point[3] = 1.0;
        along_u[3] = along_v[3] = 0.0;
    }
    const double w = point[3];
    if (w == 0.0)
        return false;
    const double x = point[0] / w, y = point[1] / w;
    const double xu = (along_u[0] - x * along_u[3]) / w, yu = (along_u[1] - y * along_u[3]) / w;
    const double xv = (along_v[0] - x * along_v[3]) / w, yv = (along_v[1] - y * along_v[3]) / w;
    flux = point[2] / w * ((xu * yv - yu * xv) / (du * dv));
    return true;
}

struct SurfaceEvaluator {
    /// One knot cell: its sides {u0, u1, v0, v1} and, where the orders allow, its Bezier controls x y z w.
    struct Cell {
        bool entered = false;
        bool usable = false;
        std::array<double, 4> sides{};
        std::vector<double> patch;
    };

    const NurbsSurface& surface;
    NurbsSurface local;
    std::array<int, 2> active{-1, -1};
    std::array<std::vector<double>, 2> knots;
    std::vector<Cell> cells;
    const Cell* cell = nullptr;

    explicit SurfaceEvaluator(const NurbsSurface& source)
        : surface(source), local(3, source.is_rational(), source.order(0), source.order(1), source.order(0), source.order(1)),
          knots{source.get_span_vector(0), source.get_span_vector(1)} {
        if (!source.is_valid() || source.dimension() != 3)
            throw std::runtime_error("Native BRep volume requires a valid three-dimensional support");
        cells.resize((source.cv_count(0) - source.order(0) + 1) * (source.cv_count(1) - source.order(1) + 1));
    }

    /// Copy the active span into local, the surface jet evaluates.
    void load() {
        for (int dir = 0; dir < 2; ++dir)
            std::copy_n(surface.m_nurbsknot[dir].begin() + active[dir], local.m_nurbsknot[dir].size(), local.m_nurbsknot[dir].begin());
        for (int i = 0; i < local.cv_count(0); ++i)
            for (int j = 0; j < local.cv_count(1); ++j)
                std::copy_n(surface.cv(active[0] + i, active[1] + j), surface.cv_size(), local.cv(i, j));
    }

    void enter(const std::array<int, 2>& span) {
        active = span;
        Cell& entered = cells[span[0] * (surface.cv_count(1) - surface.order(1) + 1) + span[1]];
        cell = &entered;
        if (!entered.patch.empty())
            return;
        load();
        if (entered.entered)
            return;
        // Checked where the span is first copied in; nothing else writes local.
        entered.entered = true;
        entered.usable = local.is_valid();
        if (!entered.usable)
            return;
        for (int dir = 0; dir < 2; ++dir) {
            entered.sides[2 * dir] = local.m_nurbsknot[dir][local.order(dir) - 2];
            entered.sides[2 * dir + 1] = local.m_nurbsknot[dir][local.order(dir) - 1];
        }
        if (local.order(0) > patch_order || local.order(1) > patch_order)
            return;
        // The controls come as enclosures a few roundings wide, or as the controls themselves where the span is a Bezier patch already.
        for (const nurbs_interval::Homogeneous& control : nurbs_interval::surface_controls(local, entered.sides, {0, 0, 0}))
            for (const nurbs_interval::Interval& value : control)
                entered.patch.push_back(std::midpoint(value.lo, value.hi));
    }

    double integrand(double u, double v) {
        // Strictly inside the active cell the span search would return it again.
        if (!(cell && cell->usable && u > cell->sides[0] && u < cell->sides[1] && v > cell->sides[2] && v < cell->sides[3])) {
            const std::array<int, 2> span = {
                nurbsknot::find_span(surface.order(0), surface.cv_count(0), surface.m_nurbsknot[0], u),
                nurbsknot::find_span(surface.order(1), surface.cv_count(1), surface.m_nurbsknot[1], v)
            };
            if (span != active)
                enter(span);
        }
        // One global divergence-one field, F=(0,0,z), for EVERY face/path.
        if (!cell->patch.empty()) {
            const int p = surface.m_order[0] - 1, q = surface.m_order[1] - 1;
            const bool rational = surface.m_is_rat;
            const double* control = cell->patch.data();
            double flux;
            if (p == 2 && q == 2 ? patch_flux<2, 2>(control, p, q, rational, cell->sides, u, v, flux)
                : p == 2 && q == 1 ? patch_flux<2, 1>(control, p, q, rational, cell->sides, u, v, flux)
                : p == 1 && q == 2 ? patch_flux<1, 2>(control, p, q, rational, cell->sides, u, v, flux)
                : patch_flux<0, 0>(control, p, q, rational, cell->sides, u, v, flux))
                return flux;
            load();
        }
        std::array<xyz, 6> derivatives;
        if (!cell->usable || !local.jet(u, v, 1, derivatives))
            throw std::runtime_error("Native BRep volume could not evaluate a local surface span");
        return derivatives[0].z * derivatives[2].cross(derivatives[1]).z;
    }
};

// One immutable BRep/frame and one sum_volume pass own this memo. Raw support
// identity and exact argument bits avoid approximate geometry/key equivalence.
using PrimitiveKey = std::tuple<int, std::uint64_t, std::uint64_t, std::uint64_t>;
using PrimitiveMemo = std::map<PrimitiveKey, double>;

double primitive(SurfaceEvaluator& surface, double u, double v, double tolerance, Work& work,
    int support, PrimitiveMemo& memo) {
    const bool cacheable = support >= 0 && std::isfinite(u) && std::isfinite(v)
        && std::isfinite(tolerance) && tolerance > 0;
    PrimitiveKey key{};
    if (cacheable) {
        key = {support, std::bit_cast<std::uint64_t>(u), std::bit_cast<std::uint64_t>(v),
            std::bit_cast<std::uint64_t>(tolerance)};
        const auto found = memo.find(key);
        if (found != memo.end())
            return found->second;
    }
    const size_t before = work.inner;
    const std::vector<double>& knots = surface.knots[0];
    double value = 0.0;
    for (size_t i = 1; i < knots.size() && knots[i - 1] < u; ++i)
        value += integrate([&](double parameter) { return surface.integrand(parameter, v); },
            knots[i - 1], std::min(u, knots[i]), tolerance / knots.size(), work, 0, true);
    // Every stored miss performed charged inner work. Thus memo size is bounded
    // by the unchanged work budget; zero-width/no-work requests are not stored.
    if (cacheable && work.inner != before && std::isfinite(value))
        memo.emplace(key, value);
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

// Whole-gap double-integral bound for changing the primitive's U gauge.
// The boundary may be only tolerance-closed; no cancellation is assumed exact.
double gauge_error(const BRep& brep, int face, const NurbsSurface& surface, double anchor, Work& work) {
    const double original = surface.domain(0).first;
    if (anchor == original)
        return 0;
    nurbs_interval::Interval error;
    for (const auto& wire : brep.m_faces[face].wires) {
        const auto edges = brep.wire_edges(wire);
        for (size_t i = 0; i < edges.size(); ++i) {
            const auto& previous = edges[i];
            const auto& next = edges[(i + 1) % edges.size()];
            const int pi = brep.pcurve_index(previous.index, face, previous.orientation);
            const int ni = brep.pcurve_index(next.index, face, next.orientation);
            if (pi < 0 || ni < 0)
                return INFINITY;
            const auto& pc = brep.m_curves_2d[pi];
            const auto& nc = brep.m_curves_2d[ni];
            const double pt =
                previous.orientation == BRepOrientation::Reversed ? pc.domain_start() : pc.domain_end();
            const double nt =
                next.orientation == BRepOrientation::Reversed ? nc.domain_end() : nc.domain_start();
            nurbs_interval::CurveBounds a, b;
            if (!nurbs_interval::curve_bounds(pc, {pt, pt}, {0, 0, 0}, a) ||
                !nurbs_interval::curve_bounds(nc, {nt, nt}, {0, 0, 0}, b))
                return INFINITY;
            const double lo = std::min(a.point[1].lo, b.point[1].lo);
            const double hi = std::max(a.point[1].hi, b.point[1].hi);
            if (lo == hi)
                continue;
            const auto vd = surface.domain(1);
            if (lo < vd.first || hi > vd.second || !std::isfinite(lo) || !std::isfinite(hi))
                return INFINITY;
            std::vector<double> u = {original}, v = {lo};
            for (double knot : surface.get_span_vector(0))
                if (knot > original && knot < anchor)
                    u.push_back(knot);
            u.push_back(anchor);
            for (double knot : surface.get_span_vector(1))
                if (knot > lo && knot < hi)
                    v.push_back(knot);
            v.push_back(hi);
            for (size_t x = 1; x < u.size(); ++x)
                for (size_t y = 1; y < v.size(); ++y) {
                    // Charge every certified integration rectangle, as with an
                    // adaptive integration interval; never hide new work.
                    ++work.inner;
                    if (++work.total > 200000)
                        throw std::runtime_error("Native BRep volume exceeded its quadrature budget");
                    // Bound the flux on a FULL original V knot cell. Dividing
                    // restricted control differences by an ULP-sized gap would
                    // unnecessarily widen an otherwise sound certificate.
                    const auto full_v = surface.get_span_vector(1);
                    const size_t row = std::min(static_cast<size_t>(
                                                    std::upper_bound(full_v.begin(), full_v.end(), v[y - 1]) -
                                                    full_v.begin()),
                                                full_v.size() - 1) -
                                       1;
                    nurbs_interval::SurfaceBounds bounds;
                    if (!nurbs_interval::bezier_bounds(surface,
                                                       {u[x - 1], u[x], full_v[row], full_v[row + 1]},
                                                       {0, 0, 0}, bounds, true, false))
                        return INFINITY;
                    const auto flux = bounds.point[2] * (bounds.derivative[0][0] * bounds.derivative[1][1] -
                                                         bounds.derivative[0][1] * bounds.derivative[1][0]);
                    const auto box = flux *
                                     (nurbs_interval::Interval(u[x]) - nurbs_interval::Interval(u[x - 1])) *
                                     (nurbs_interval::Interval(v[y]) - nurbs_interval::Interval(v[y - 1]));
                    if (!nurbs_interval::finite(box))
                        return INFINITY;
                    error = error + nurbs_interval::Interval(nurbs_interval::magnitude(box));
                }
        }
    }
    return nurbs_interval::finite(error) ? error.hi : INFINITY;
}

std::array<double, 4> trim_hull(const BRep& brep, int face, const NurbsSurface& surface) {
    const auto u = surface.domain(0), v = surface.domain(1);
    const std::array<double, 4> entire = {u.first, u.second, v.first, v.second};
    std::array<double, 4> result = {INFINITY, -INFINITY, INFINITY, -INFINITY};
    for (const auto& wire : brep.m_faces[face].wires)
        for (const auto& edge : brep.wire_edges(wire)) {
            const int index = brep.pcurve_index(edge.index, face, edge.orientation);
            if (index < 0)
                return entire;
            const auto& curve = brep.m_curves_2d[index];
            for (int i = 0; i < curve.cv_count(); ++i) {
                const double w = curve.weight(i);
                if (!std::isfinite(w) || w <= 0)
                    return entire;
                for (int d = 0; d < 2; ++d) {
                    const auto point = curve.is_rational() ? nurbs_interval::Interval(curve.cv(i)[d]) /
                                                                 nurbs_interval::Interval(w)
                                                           : nurbs_interval::Interval(curve.cv(i)[d]);
                    if (!nurbs_interval::finite(point))
                        return entire;
                    result[2 * d] = std::min(result[2 * d], point.lo);
                    result[2 * d + 1] = std::max(result[2 * d + 1], point.hi);
                }
            }
        }
    if (result[0] < u.first || result[1] > u.second || result[2] < v.first || result[3] > v.second)
        return entire;
    return result;
}

double face_volume(const BRep& brep, int face, const Frame& frame, double tolerance, Work& work, PrimitiveMemo& memo) {
    const BRepFace& boundary = brep.m_faces[face];
    const NurbsSurface surface = local_surface(brep.m_surfaces[boundary.surface_index], frame);
    SurfaceEvaluator evaluator(surface);
    const int rectangle = rectangle_orientation(brep, face, surface);
    if (!surface.is_rational() && rectangle && surface.degree(0) <= 21 && surface.degree(1) <= 21)
        return polynomial_volume(evaluator, rectangle, work);
    const plane::Primitive cache(
        surface, trim_hull(brep, face, surface), 200000 - work.total,
        [&]() {
            ++work.polynomial_coefficients;
            if (++work.total > 200000)
                throw std::runtime_error("Native BRep volume exceeded its quadrature budget");
        },
        [&](double anchor, size_t required) {
            // Check the gauge BEFORE spending the coefficient budget. Every
            // certificate rectangle is charged, even if this cache is declined.
            const double residual = gauge_error(brep, face, surface, anchor, work);
            if (!std::isfinite(residual) || residual > tolerance * .125 || required > 200000 - work.total)
                return false;
            const auto remaining = nurbs_interval::Interval(tolerance) - nurbs_interval::Interval(residual);
            if (!nurbs_interval::finite(remaining) || remaining.lo <= 0)
                return false;
            tolerance = remaining.lo; // Directed reservation of the full gap bound.
            return true;
        });
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
                if (!cache.cells.empty())
                    value += sign * integrate_polynomial([&](double parameter) {
                        std::array<xyz, 3> d;
                        curve.jet(parameter, 1, d);
                        if (d[1][1] == 0.0)
                            return nurbs_interval::Interval();
                        const nurbs_interval::Interval primitive = cache.at(d[0][0], d[0][1]);
                        const nurbs_interval::Interval integrand = primitive * nurbs_interval::Interval(d[1][1]);
                        if (!nurbs_interval::finite(integrand)) {
                            const nlohmann::ordered_json witness = {
                                {"face", face},
                                {"edge", edge.index},
                                {"pcurve_index", index},
                                {"parameter", parameter},
                                {"integration_span", {knots[i - 1], knots[i]}},
                                {"uv", {d[0][0], d[0][1]}},
                                {"dv_dt", d[1][1]},
                                {"cache_domain",
                                 {cache.u.front(), cache.u.back(), cache.v.front(), cache.v.back()}},
                                {"primitive_finite", nurbs_interval::finite(primitive)},
                                {"primitive", {primitive.lo, primitive.hi}},
                                {"curve", curve.jsondump()},
                                {"surface", surface.jsondump()}};
                            throw std::runtime_error("Native polynomial volume has a nonfinite boundary integrand; uv_witness: " + witness.dump());
                        }
                        return integrand;
                    }, knots[i - 1], knots[i], allowed, work);
                else
                    value += sign * integrate([&](double parameter) {
                        std::array<xyz, 3> d;
                        curve.jet(parameter, 1, d);
                        if (d[1][1] == 0.0)
                            return 0.0;
                        return primitive(evaluator, d[0][0], d[0][1], tolerance * 0.05, work, boundary.surface_index, memo) * d[1][1];
                    }, knots[i - 1], knots[i], allowed, work);
            }
        }
    return value;
}

double sum_volume(const BRep& brep, const Frame& frame, double tolerance) {
    Work work;
    PrimitiveMemo memo;
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
                    volume += sign * face_volume(brep, face.index, frame, tolerance / brep.face_count(), work, memo);
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
