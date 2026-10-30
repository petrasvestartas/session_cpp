#include "brep_section.h"
#include "brep_classify.h"
#include "curve_intersection.h"
#include "curve_surface_intersection.h"
#include "intersection.h"
#include "nurbs_bounds.h"
#include "nurbs_coincidence.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace session_cpp::brep_section {
namespace {

struct Bounds {
    Point lo;
    Point hi;

    explicit Bounds(const NurbsSurface& surface) : lo(surface.get_cv(0, 0)), hi(lo) {
        for (int i = 0; i < surface.cv_count(0); ++i)
            for (int j = 0; j < surface.cv_count(1); ++j) {
                const Point p = surface.get_cv(i, j);
                if (!(surface.weight(i, j) > 0.0))
                    throw std::invalid_argument("Native BRep sections require positive rational weights");
                for (int d = 0; d < 3; ++d) {
                    lo[d] = std::min(lo[d], p[d]);
                    hi[d] = std::max(hi[d], p[d]);
                }
            }
    }

    bool overlaps(const Bounds& other, double tolerance) const {
        for (int d = 0; d < 3; ++d)
            if (hi[d] + tolerance < other.lo[d] || other.hi[d] + tolerance < lo[d])
                return false;
        return true;
    }
};

double uv_tolerance(const NurbsSurface& surface, double tolerance) {
    return tolerance / nurbs_bounds::surface_metric(surface);
}

std::vector<int> active_faces(const BRep& brep) {
    std::vector<int> faces;
    if (brep.m_solids.empty()) {
        for (int i = 0; i < brep.face_count(); ++i)
            faces.push_back(i);
        return faces;
    }
    for (const BRepSolid& solid : brep.m_solids)
        for (const BRepRef& shell : solid.shells)
            for (const BRepRef& face : brep.m_shells[shell.index].faces)
                if (const BRepOrientation orientation = brep_compose(shell.orientation, face.orientation);
                    orientation == BRepOrientation::Forward || orientation == BRepOrientation::Reversed)
                    faces.push_back(face.index);
    std::sort(faces.begin(), faces.end());
    faces.erase(std::unique(faces.begin(), faces.end()), faces.end());
    return faces;
}

// A shared exact UV translation changes neither metric nor raw curve parameters.
bool translate_parameters_exact(NurbsCurve& curve, const Point& origin) {
    if (curve.dimension() < 1 || curve.dimension() > 3)
        return false;
    for (int i = 0; i < curve.cv_count(); ++i) {
        double* cv = curve.cv(i);
        const double weight = curve.is_rational() ? cv[curve.dimension()] : 1.0;
        for (int d = 0; d < curve.dimension(); ++d) {
            const double product = origin[d] * weight;
            double local;
            if (!nurbs_coincidence::exact_products_equal(origin[d], weight, product, 1.0)
                || !nurbs_coincidence::exact_difference(cv[d], product, local))
                return false;
            cv[d] = local;
        }
    }
    return true;
}

void trim_parameters(
    const BRep& brep,
    int face,
    const NurbsCurve& pcurve,
    double tolerance,
    std::vector<double>& parameters
) {
    for (const BRepRef& wire : brep.m_faces[face].wires)
        for (const BRepRef& edge : brep.wire_edges(wire)) {
            const int index = brep.pcurve_index(edge.index, face, edge.orientation);
            if (index < 0)
                throw std::runtime_error("Native section encountered a missing face pcurve");
            const NurbsCurve& boundary = brep.m_curves_2d[index];
            NurbsCurve local_section = pcurve, local_boundary = boundary;
            const Point origin = boundary.get_cv(0);
            const bool exact = translate_parameters_exact(local_section, origin)
                && translate_parameters_exact(local_boundary, origin);
            // Only a/b parameters escape this call; all original UV curves stay unchanged.
            const curve_intersection::Result events = exact
                ? curve_intersection::intersect(local_section, local_boundary, tolerance)
                : curve_intersection::intersect(pcurve, boundary, tolerance);
            if (!events.complete)
                throw std::runtime_error(events.failure);
            for (const curve_intersection::Hit& point : events.points)
                parameters.push_back(point.a);
            for (const curve_intersection::Overlap& overlap : events.overlaps) {
                parameters.push_back(overlap.a0);
                parameters.push_back(overlap.a1);
            }
        }
}

void sort_parameters(std::vector<double>& parameters) {
    std::sort(parameters.begin(), parameters.end());
    const double scale = std::max({std::abs(parameters.front()), std::abs(parameters.back()),
                                  parameters.back() - parameters.front()});
    const double epsilon = 32.0 * std::numeric_limits<double>::epsilon() * scale;
    parameters.erase(std::unique(parameters.begin(), parameters.end(), [epsilon](double a, double b) {
        return b - a <= epsilon;
    }), parameters.end());
}

bool retained(const BRep& brep, int face, const NurbsCurve& pcurve, double t, double tolerance) {
    const brep_classify::State state = brep_classify::face_uv(brep, face, pcurve.point_at(t), tolerance);
    if (state == brep_classify::State::Unknown)
        throw std::runtime_error("Native section could not classify a trimming interval");
    return state != brep_classify::State::Outside;
}

NurbsCurve trimmed(const NurbsCurve& curve, double first, double last) {
    NurbsCurve result = curve;
    if (!result.trim(first, last))
        throw std::runtime_error("Native section could not trim a curve interval");
    return result;
}

void append_overlap(
    CurveResult& result,
    const BRep& brep,
    int face,
    const curve_surface_intersection::Overlap& overlap,
    double tolerance
) {
    if (!overlap.pcurve.is_valid())
        throw std::runtime_error("Native curve/face overlap has no parameter-space curve");
    std::vector<double> parameters = {overlap.t0, overlap.t1};
    trim_parameters(brep, face, overlap.pcurve, tolerance, parameters);
    sort_parameters(parameters);
    for (size_t i = 1; i < parameters.size(); ++i)
        if (retained(brep, face, overlap.pcurve, std::midpoint(parameters[i - 1], parameters[i]), tolerance))
            result.overlaps.push_back({face, parameters[i - 1], parameters[i],
                trimmed(overlap.pcurve, parameters[i - 1], parameters[i])});
}

void check_input(const BRep& brep, double tolerance) {
    if (!std::isfinite(tolerance) || tolerance <= 0.0 || !brep.is_valid())
        throw std::invalid_argument("Native BRep sections require valid geometry and finite positive tolerance");
}

bool unsupported_parameterization(const NurbsSurface& a, const NurbsSurface& b, double tolerance) {
    for (int variant = 0; variant < 8; ++variant) {
        NurbsSurface candidate = b;
        if ((variant & 4) && !candidate.transpose())
            continue;
        if ((variant & 1) && !candidate.reverse(0))
            continue;
        if ((variant & 2) && !candidate.reverse(1))
            continue;
        bool valid = true;
        for (int d = 0; d < 2; ++d) {
            const std::pair<double, double> domain = a.domain(d);
            valid = candidate.set_domain(d, domain.first, domain.second) && valid;
        }
        if (valid && nurbs_coincidence::same_parameters(a, candidate, tolerance))
            return true;
    }
    return false;
}

}

CurveResult curve(const NurbsCurve& input, const BRep& brep, double tolerance) {
    check_input(brep, tolerance);
    CurveResult result;
    try {
        for (int face : active_faces(brep)) {
            const NurbsSurface& surface = brep.m_surfaces[brep.m_faces[face].surface_index];
            const double uv_tol = uv_tolerance(surface, tolerance);
            const curve_surface_intersection::Result events = curve_surface_intersection::intersect(input, surface, tolerance);
            if (!events.complete)
                throw std::runtime_error(events.failure);
            for (const curve_surface_intersection::Hit& hit : events.points) {
                const brep_classify::State state = brep_classify::face_uv(brep, face, {hit.u, hit.v, 0}, uv_tol);
                if (state == brep_classify::State::Unknown)
                    throw std::runtime_error("Native curve/face hit has unresolved trim classification");
                if (state != brep_classify::State::Outside)
                    result.points.push_back({hit.point, face, hit.t, hit.u, hit.v, hit.tangent});
            }
            for (const curve_surface_intersection::Overlap& overlap : events.overlaps)
                append_overlap(result, brep, face, overlap, uv_tol);
        }
        std::sort(result.points.begin(), result.points.end(), [](const CurveHit& a, const CurveHit& b) { return a.t < b.t; });
    } catch (const std::runtime_error& error) {
        result.complete = false;
        result.failure = error.what();
    }
    return result;
}

FaceResult faces(const BRep& first, const BRep& second, double tolerance) {
    check_input(first, tolerance);
    check_input(second, tolerance);
    FaceResult result;
    int first_face = -1;
    int second_face = -1;
    try {
        for (int a : active_faces(first)) {
            first_face = a;
            const NurbsSurface& sa = first.m_surfaces[first.m_faces[a].surface_index];
            const Bounds box(sa);
            const double ta = uv_tolerance(sa, tolerance);
            for (int b : active_faces(second)) {
                second_face = b;
                const NurbsSurface& sb = second.m_surfaces[second.m_faces[b].surface_index];
                if (!box.overlaps(Bounds(sb), tolerance))
                    continue;
                nurbs_coincidence::ParameterMap mapping;
                if (nurbs_coincidence::same_parameters(sa, sb, tolerance)
                    || nurbs_coincidence::parameter_mapping(sa, sb, tolerance, mapping)
                    || nurbs_coincidence::common_basis_agreement(sa, sb, tolerance)
                    || nurbs_coincidence::coplanar(sa, sb, tolerance)) {
                    result.overlaps.push_back({a, b});
                    continue;
                }
                if (unsupported_parameterization(sa, sb, tolerance))
                    throw std::runtime_error("Native section has coincident supports with unsupported parameter mapping");
                const double tb = uv_tolerance(sb, tolerance);
                for (const auto& section : Intersection::surface_surface(sa, sb, tolerance)) {
                    const NurbsCurve& world = std::get<0>(section);
                    const NurbsCurve& ua = std::get<1>(section);
                    const NurbsCurve& ub = std::get<2>(section);
                    if (world.domain() != ua.domain() || world.domain() != ub.domain())
                        throw std::runtime_error("Native section pcurves have mismatched parameter domains");
                    std::vector<double> parameters = {world.domain_start(), world.domain_end()};
                    trim_parameters(first, a, ua, ta, parameters);
                    trim_parameters(second, b, ub, tb, parameters);
                    sort_parameters(parameters);
                    size_t run_begin = 0;
                    bool retaining = false;
                    for (size_t i = 1; i < parameters.size(); ++i) {
                        const double middle = std::midpoint(parameters[i - 1], parameters[i]);
                        const bool keep = retained(first, a, ua, middle, ta) && retained(second, b, ub, middle, tb);
                        if (keep && !retaining) {
                            run_begin = i - 1;
                            retaining = true;
                        }
                        if (retaining && (!keep || i + 1 == parameters.size())) {
                            const size_t run_end = keep ? i : i - 1;
                            result.curves.push_back({a, b, trimmed(world, parameters[run_begin], parameters[run_end]),
                                trimmed(ua, parameters[run_begin], parameters[run_end]),
                                trimmed(ub, parameters[run_begin], parameters[run_end])});
                            retaining = false;
                        }
                    }
                }
            }
        }
    } catch (const std::runtime_error& error) {
        result.complete = false;
        result.failure = std::string(error.what()) + "; face pair " + std::to_string(first_face)
            + "," + std::to_string(second_face);
    }
    return result;
}

}
