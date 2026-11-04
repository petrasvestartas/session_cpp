#pragma once
#include "brep_hollow_affine.h"
#include "nurbs_normal_sweep.h"

namespace session_cpp::brep_hollow::source_working {
struct UnsupportedExtension : std::runtime_error {
    explicit UnsupportedExtension(const char* message) : std::runtime_error(message) {}
};
inline void representable(bool valid, const char* message) {
    if (!valid)
        throw UnsupportedExtension(message);
}
inline double product(double a, double b) {
    const double value = a * b;
    representable(std::isfinite(value) && nurbs_coincidence::exact_products_equal(a, b, value, 1),
        "Hollow polynomial source extension has an unrepresentable product");
    return value;
}
inline double sum(double a, double b) {
    const double value = a + b;
    double recovered = 0;
    representable(std::isfinite(value) && nurbs_coincidence::exact_difference(value, a, recovered)
        && recovered == b, "Hollow polynomial source extension has an unrepresentable sum");
    return value;
}
inline bool graph_axes(const NurbsSurface& surface, std::array<double, 3>& first,
    std::array<double, 3>& second, std::array<double, 3>& mixed, std::array<int, 2>& axes) {
    if (surface.dimension() != 3 || surface.is_rational() || surface.degree(0) != 1 || surface.degree(1) != 1
        || surface.cv_count(0) != 2 || surface.cv_count(1) != 2)
        return false;
    axes = {-1, -1};
    for (int d = 0; d < 3; ++d) {
        double opposite = 0;
        if (!nurbs_coincidence::exact_difference(surface.get_cv(1, 0)[d], surface.get_cv(0, 0)[d], first[d])
            || !nurbs_coincidence::exact_difference(surface.get_cv(0, 1)[d], surface.get_cv(0, 0)[d], second[d])
            || !nurbs_coincidence::exact_difference(surface.get_cv(1, 1)[d], surface.get_cv(1, 0)[d], opposite)
            || !nurbs_coincidence::exact_difference(opposite, second[d], mixed[d]))
            return false;
        if (mixed[d] == 0 && second[d] == 0 && first[d] != 0)
            axes[0] = d;
        if (mixed[d] == 0 && first[d] == 0 && second[d] != 0)
            axes[1] = d;
    }
    return axes[0] >= 0 && axes[1] >= 0;
}
inline NurbsSurface extend(const NurbsSurface& source, const std::array<double, 3>& first,
    const std::array<double, 3>& second, const std::array<double, 3>& mixed,
    std::array<double, 4>& rectangle) {
    for (int d = 0; d < 2; ++d) {
        const std::pair<double, double> domain = source.domain(d);
        double width = 0;
        representable(nurbs_coincidence::exact_difference(domain.second, domain.first, width) && width > 0,
            "Hollow polynomial source chart width is not exact");
        rectangle[2*d] = sum(domain.first, -width);
        rectangle[2*d+1] = sum(domain.second, width);
    }
    std::vector<Point> controls;
    for (double u : {-1.0, 2.0})
        for (double v : {-1.0, 2.0}) {
            Point point;
            for (int d = 0; d < 3; ++d)
                point[d] = sum(sum(sum(source.get_cv(0, 0)[d], product(u, first[d])),
                    product(v, second[d])), product(u * v, mixed[d]));
            controls.push_back(point);
        }
    NurbsSurface result = NurbsSurface::create(false, false, 1, 1, 2, 2, controls);
    representable(result.set_domain(0, rectangle[0], rectangle[1])
        && result.set_domain(1, rectangle[2], rectangle[3]), "Hollow polynomial source chart extension failed");
    representable(result.domain(0) == std::pair<double, double>(rectangle[0], rectangle[1])
        && result.domain(1) == std::pair<double, double>(rectangle[2], rectangle[3]),
        "Hollow polynomial source chart endpoint remapping was inexact");
    return result;
}
inline bool covers_extent(const BRep& original, const NurbsSurface& extended,
    const std::array<int, 2>& axes, double distance) {
    for (int axis : axes) {
        nurbs_interval::Interval required;
        bool first = true;
        for (const NurbsSurface& surface : original.m_surfaces)
            for (int i = 0; i < surface.cv_count(0); ++i)
                for (int j = 0; j < surface.cv_count(1); ++j) {
                    const double weight = surface.weight(i, j);
                    if (surface.dimension() < 2 || surface.dimension() > 3 || !std::isfinite(weight) || weight <= 0)
                        return false;
                    const nurbs_interval::Interval value = axis < surface.dimension()
                        ? nurbs_interval::Interval(surface.cv(i, j)[axis]) / nurbs_interval::Interval(weight)
                        : nurbs_interval::Interval(0);
                    if (!nurbs_interval::finite(value))
                        return false;
                    required = first ? value : nurbs_interval::Interval(std::min(required.lo, value.lo), std::max(required.hi, value.hi));
                    first = false;
                }
        required = required + nurbs_interval::Interval(-2 * std::abs(distance), 2 * std::abs(distance));
        const double a = extended.get_cv(0, 0)[axis], b = extended.get_cv(1, 1)[axis];
        if (first || !nurbs_interval::finite(required)
            || !(std::min(a, b) < required.lo && std::max(a, b) > required.hi))
            return false;
    }
    return true;
}
inline bool prepare(const BRep& original, brep_offset::OffsetFace& working,
    double signed_distance, double tolerance) {
    affine_working::require(working.brep.is_valid() && working.brep.face_count() == 1,
        "Hollow polynomial source working face is invalid or ambiguous");
    const NurbsSurface old = working.brep.m_surfaces[working.brep.m_faces[0].surface_index];
    std::array<double, 3> first{}, second{}, mixed{};
    std::array<int, 2> axes{};
    if (!graph_axes(old, first, second, mixed, axes)
        || (mixed[0] == 0 && mixed[1] == 0 && mixed[2] == 0))
        return false;
    if (working.brep.m_faces[0].wires.size() != 1)
        return false;
    const std::vector<BRepRef> edges = working.brep.wire_edges(working.brep.m_faces[0].wires[0]);
    if (edges.size() != 4 || !affine_working::rectangular_wire(working.brep, old, edges))
        return false;
    std::array<double, 4> rectangle{};
    NurbsSurface extended;
    try {
        extended = extend(old, first, second, mixed, rectangle);
    } catch (const UnsupportedExtension&) {
        return false;
    }
    if (!covers_extent(original, extended, axes, signed_distance))
        return false;
    const normal_sweep::Result regularity = normal_sweep::check(extended, rectangle, signed_distance,
        4096, [](double, double) { return true; });
    if (regularity.state != normal_sweep::State::Regular)
        return false;
    BRep candidate = working.brep;
    candidate.m_surfaces[candidate.m_faces[0].surface_index] = std::move(extended);
    std::map<int, Point> vertices;
    std::set<int> worlds, pcurves;
    for (const BRepRef& use : edges) {
        const BRepEdge& edge = candidate.m_edges[use.index];
        affine_working::require(edge.pcurves.size() == 1 && worlds.insert(edge.curve_3d_index).second
            && pcurves.insert(edge.pcurves[0].curve_2d_index).second,
            "Hollow polynomial source rectangle aliases curve storage");
        affine_working::enlarge_border(candidate, use.index, old, rectangle, tolerance, vertices);
    }
    for (const std::pair<const int, Point>& vertex : vertices)
        candidate.m_vertices[vertex.first].point = vertex.second;
    affine_working::require(candidate.is_valid(), "Hollow polynomial source working face is invalid");
    working.brep = std::move(candidate);
    return true;
}
}
