#pragma once
#include "brep_offset_join.h"
#include "nurbs_coincidence.h"
#include "nurbs_interval.h"
#include "curve_surface_intersection.h"
#include <array>
#include <map>
#include <set>
#include <stdexcept>
#include <numeric>
#include <cmath>
#include <algorithm>

namespace session_cpp::brep_hollow::affine_working {
inline void require(bool valid, const char* message) {
    if (!valid)
        throw std::runtime_error(message);
}
inline nurbs_interval::Interval dot(const std::array<nurbs_interval::Interval, 3>& a, const std::array<nurbs_interval::Interval, 3>& b) {
    nurbs_interval::Interval result(0);
    for (int d = 0; d < 3; ++d)
        result = result + a[d] * b[d];
    return result;
}
struct Frame {
    Point origin;
    std::array<nurbs_interval::Interval, 3> first, second;
};
inline bool exact_frame(const NurbsSurface& surface, Frame& frame) {
    if (surface.dimension() != 3 || surface.is_rational() || surface.degree(0) != 1 || surface.degree(1) != 1
        || surface.cv_count(0) != 2 || surface.cv_count(1) != 2)
        return false;
    frame.origin = surface.get_cv(0, 0);
    for (int d = 0; d < 3; ++d) {
        double a = 0, b = 0, c = 0;
        if (!nurbs_coincidence::exact_difference(surface.get_cv(1, 0)[d], frame.origin[d], a)
            || !nurbs_coincidence::exact_difference(surface.get_cv(0, 1)[d], frame.origin[d], b)
            || !nurbs_coincidence::exact_difference(surface.get_cv(1, 1)[d], surface.get_cv(1, 0)[d], c)
            || b != c)
            return false;
        frame.first[d] = nurbs_interval::Interval(a);
        frame.second[d] = nurbs_interval::Interval(b);
    }
    return true;
}
inline std::array<double, 4> extent(const BRep& source, const NurbsSurface& surface,
    const Frame& frame, double distance) {
    std::array<nurbs_interval::Interval, 3> world;
    bool first = true;
    for (const NurbsSurface& support : source.m_surfaces) {
        require(support.dimension() >= 2 && support.dimension() <= 3,
            "Hollow working extent requires two or three dimensional source supports");
        for (int i = 0; i < support.cv_count(0); ++i)
            for (int j = 0; j < support.cv_count(1); ++j) {
                const double weight = support.weight(i, j);
                require(std::isfinite(weight) && weight > 0, "Hollow working extent requires finite positive source weights");
                for (int d = 0; d < 3; ++d) {
                    const nurbs_interval::Interval point = d < support.dimension()
                        ? nurbs_interval::Interval(support.cv(i, j)[d]) / nurbs_interval::Interval(weight)
                        : nurbs_interval::Interval(0);
                    require(nurbs_interval::finite(point), "Hollow working extent has nonfinite source controls");
                    world[d] = first ? point : nurbs_interval::Interval(std::min(world[d].lo, point.lo), std::max(world[d].hi, point.hi));
                }
                first = false;
            }
    }
    require(!first, "Hollow working extent has no source controls");
    for (int d = 0; d < 3; ++d)
        world[d] = world[d] + nurbs_interval::Interval(-std::abs(distance), std::abs(distance)) - nurbs_interval::Interval(frame.origin[d]);
    const nurbs_interval::Interval aa = dot(frame.first, frame.first), ab = dot(frame.first, frame.second);
    const nurbs_interval::Interval bb = dot(frame.second, frame.second), det = aa * bb - ab * ab;
    require(nurbs_interval::finite(det) && det.lo > 0, "Hollow working plane has unresolved rank");
    const nurbs_interval::Interval x = dot(frame.first, world), y = dot(frame.second, world);
    const std::array<nurbs_interval::Interval, 2> projected = {(bb * x - ab * y) / det, (aa * y - ab * x) / det};
    std::array<double, 4> result{};
    for (int d = 0; d < 2; ++d) {
        const std::pair<double, double> domain = surface.domain(d);
        const nurbs_interval::Interval parameter = nurbs_interval::Interval(domain.first) + projected[d] * (nurbs_interval::Interval(domain.second) - nurbs_interval::Interval(domain.first));
        require(nurbs_interval::finite(parameter), "Hollow working plane extent is nonfinite");
        result[2*d] = std::min(domain.first, parameter.lo);
        result[2*d+1] = std::max(domain.second, parameter.hi);
    }
    return result;
}
inline NurbsSurface enlarged_surface(const NurbsSurface& surface, const Frame& frame,
    const std::array<double, 4>& rectangle, double tolerance) {
    std::vector<Point> controls;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            const nurbs_interval::Interval u = (nurbs_interval::Interval(rectangle[i]) - nurbs_interval::Interval(surface.domain(0).first))
                / (nurbs_interval::Interval(surface.domain(0).second) - nurbs_interval::Interval(surface.domain(0).first));
            const nurbs_interval::Interval v = (nurbs_interval::Interval(rectangle[2+j]) - nurbs_interval::Interval(surface.domain(1).first))
                / (nurbs_interval::Interval(surface.domain(1).second) - nurbs_interval::Interval(surface.domain(1).first));
            Point point;
            nurbs_interval::Interval squared(0);
            for (int d = 0; d < 3; ++d) {
                const nurbs_interval::Interval exact = nurbs_interval::Interval(frame.origin[d]) + u * frame.first[d] + v * frame.second[d];
                require(nurbs_interval::finite(exact), "Hollow working plane has nonfinite corners");
                point[d] = std::midpoint(exact.lo, exact.hi);
                const double error = nurbs_interval::magnitude(exact - nurbs_interval::Interval(point[d]));
                squared = squared + nurbs_interval::Interval(error) * nurbs_interval::Interval(error);
            }
            const nurbs_interval::Interval limit = nurbs_interval::Interval(tolerance * .0625) * nurbs_interval::Interval(tolerance * .0625);
            require(nurbs_interval::finite(squared) && squared.hi <= limit.lo,
                "Hollow working plane extension exceeds its roundoff allowance");
            controls.push_back(point);
        }
    NurbsSurface result = NurbsSurface::create(false, false, 1, 1, 2, 2, controls);
    // This fresh bilinear patch has exactly two clamped knots per direction.
    // Install the prescribed raw endpoints, rather than rounding a+(b-a) again.
    result.m_nurbsknot[0] = {rectangle[0], rectangle[1]};
    result.m_nurbsknot[1] = {rectangle[2], rectangle[3]};
    require(result.is_valid(), "Hollow working plane domain construction failed");
    require(result.domain(0) == std::make_pair(rectangle[0], rectangle[1])
        && result.domain(1) == std::make_pair(rectangle[2], rectangle[3]),
        "Hollow working plane changed the requested raw chart");
    return result;
}
inline bool straight_pcurve(const NurbsCurve& curve) {
    if (curve.degree() != 1 || curve.cv_count() != 2 || curve.dimension() < 2 || curve.dimension() > 3
        || !std::isfinite(curve.weight(0)) || curve.weight(0) <= 0 || curve.weight(0) != curve.weight(1))
        return false;
    for (int i = 0; i < 2; ++i)
        for (int d = 0; d < curve.dimension(); ++d)
            if (!nurbs_coincidence::exact_products_equal(curve.get_cv(i)[d], curve.weight(i), curve.cv(i)[d], 1))
                return false;
    return true;
}
inline void enlarge_border(BRep& working, int edge_index, const NurbsSurface& old,
    const std::array<double, 4>& rectangle, double tolerance, std::map<int, Point>& vertices) {
    BRepEdge& edge = working.m_edges[edge_index];
    require(!edge.degenerated && edge.pcurves.size() == 1
        && edge.pcurves[0].curve_2d_index_2 < 0
        && edge.pcurves[0].surface_index == working.m_faces[0].surface_index, "Hollow affine working border does not support seams or poles");
    NurbsCurve& pc = working.m_curves_2d[edge.pcurves[0].curve_2d_index];
    require(straight_pcurve(pc), "Hollow affine working border requires an exactly affine pcurve");
    require(edge.curve_3d_index >= 0 && pc.domain() == working.m_curves_3d[edge.curve_3d_index].domain(),
        "Hollow affine working border requires identical raw domains");
    std::array<Point, 2> uv;
    for (int end = 0; end < 2; ++end) {
        uv[end] = pc.get_cv(end);
        for (int d = 0; d < 2; ++d) {
            const std::pair<double, double> domain = old.domain(d);
            require(uv[end][d] == domain.first || uv[end][d] == domain.second,
                "Hollow affine working border is not an original rectangle side");
            uv[end][d] = rectangle[2*d + (uv[end][d] == domain.second)];
        }
    }
    const int direction = uv[0][0] == uv[1][0] ? 1 : (uv[0][1] == uv[1][1] ? 0 : -1);
    require(direction >= 0 && uv[0][direction] != uv[1][direction], "Hollow affine working border is not a side");
    const NurbsSurface& surface = working.m_surfaces[working.m_faces[0].surface_index];
    NurbsCurve world = surface.iso_curve(direction, uv[0][1-direction]);
    if (uv[0][direction] > uv[1][direction])
        require(world.reverse(), "Hollow affine working border reversal failed");
    require(world.set_domain(pc.domain_start(), pc.domain_end()), "Hollow affine working border domain failed");
    for (int end = 0; end < 2; ++end) {
        require(pc.set_cv(end, uv[end]), "Hollow affine working pcurve construction failed");
        const int vertex = end ? edge.end_vertex : edge.start_vertex;
        const Point point = end ? world.point_at_end() : world.point_at_start();
        const auto found = vertices.find(vertex);
        require(found == vertices.end() || found->second.distance(point) <= tolerance * .125,
            "Hollow affine working border lost shared corners");
        if (found == vertices.end())
            require(vertices.emplace(vertex, point).second, "Hollow affine working corner insertion failed");
    }
    require(curve_surface_intersection::same_parameter(world, pc, surface, tolerance * .25)
        == curve_surface_intersection::ParameterMatch::Matched, "Hollow affine working border lacks a full parameter certificate");
    working.m_curves_3d[edge.curve_3d_index] = std::move(world);
}
inline bool rectangular_wire(const BRep& working, const NurbsSurface& surface,
    const std::vector<BRepRef>& edges) {
    std::set<std::pair<int, int>> sides;
    std::set<std::array<double, 2>> corners;
    std::array<double, 2> first{}, previous{};
    for (size_t i = 0; i < edges.size(); ++i) {
        const BRepRef& use = edges[i];
        const BRepEdge& edge = working.m_edges[use.index];
        if (edge.pcurves.size() != 1 || edge.pcurves[0].curve_2d_index_2 >= 0)
            return false;
        const NurbsCurve& pc = working.m_curves_2d[edge.pcurves[0].curve_2d_index];
        if (!straight_pcurve(pc))
            return false;
        const Point a = pc.get_cv(0), b = pc.get_cv(1);
        const int direction = a[0] == b[0] ? 1 : (a[1] == b[1] ? 0 : -1);
        if (direction < 0 || a[direction] == b[direction])
            return false;
        for (int d = 0; d < 2; ++d) {
            const std::pair<double, double> domain = surface.domain(d);
            if ((a[d] != domain.first && a[d] != domain.second)
                || (b[d] != domain.first && b[d] != domain.second))
                return false;
        }
        if (!sides.emplace(direction, a[1-direction] == surface.domain(1-direction).second).second)
            return false;
        if (use.orientation != BRepOrientation::Forward && use.orientation != BRepOrientation::Reversed)
            return false;
        const Point start = use.orientation == BRepOrientation::Forward ? a : b;
        const Point end = use.orientation == BRepOrientation::Forward ? b : a;
        const std::array<double, 2> uv = {start[0], start[1]};
        if (!corners.insert(uv).second || (i && previous != uv))
            return false;
        if (!i)
            first = uv;
        previous = {end[0], end[1]};
    }
    return sides.size() == 4 && corners.size() == 4 && previous == first;
}
inline void rectangle_wire(const BRep& working, const NurbsSurface& surface,
    const std::vector<BRepRef>& edges) {
    require(rectangular_wire(working, surface, edges), "Hollow working rectangle is not one complete affine parameter loop");
}
inline void enlarge(const BRep& source, brep_offset::OffsetFace& displaced, double distance, double tolerance) {
    BRep working = displaced.brep;
    require(working.face_count() == 1, "Hollow affine working face is ambiguous");
    const int surface_index = working.m_faces[0].surface_index;
    const NurbsSurface old = working.m_surfaces[surface_index];
    Frame frame;
    if (!exact_frame(old, frame))
        return;
    if (working.m_faces[0].wires.size() != 1)
        return;
    const std::vector<BRepRef> edges = working.wire_edges(working.m_faces[0].wires[0]);
    if (edges.size() != 4 || !rectangular_wire(working, old, edges))
        return;
    const std::array<double, 4> rectangle = extent(source, old, frame, distance);
    working.m_surfaces[surface_index] = enlarged_surface(old, frame, rectangle, tolerance);
    std::map<int, Point> vertices;
    std::set<int> visited, worlds, pcurves;
    for (const BRepRef& edge : edges) {
        require(visited.insert(edge.index).second, "Hollow affine working rectangle repeats an edge");
        const BRepEdge& border = working.m_edges[edge.index];
        require(border.pcurves.size() == 1 && worlds.insert(border.curve_3d_index).second
            && pcurves.insert(border.pcurves[0].curve_2d_index).second,
            "Hollow affine working rectangle aliases boundary curve storage");
        enlarge_border(working, edge.index, old, rectangle, tolerance, vertices);
    }
    require(vertices.size() == 4, "Hollow affine working rectangle has invalid corner incidence");
    for (const auto& vertex : vertices)
        working.m_vertices[vertex.first].point = vertex.second;
    require(working.is_valid(), "Hollow affine working rectangle is invalid");
    displaced.brep = std::move(working);
}
}
