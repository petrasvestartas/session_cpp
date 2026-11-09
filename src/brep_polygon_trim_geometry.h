#pragma once
#include "brep_section.h"
#include "brep_face_boundary.h"
#include "brep_classify.h"
#include "curve_surface_intersection.h"
#include "section_exact_linear_event.h"
#include "exact_orientation.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <vector>

namespace session_cpp::brep_section::detail {
struct PolygonTrimEdge {
    OriginalTrimEvent use;
    const NurbsCurve* uv = nullptr;
    const NurbsCurve* world = nullptr;
    std::array<Point, 2> ends;
};
struct PolygonTrimProof {
    std::vector<std::vector<PolygonTrimEdge>> wires;
    std::vector<OriginalTrimEvent> events;
};
inline bool polygon_same_point(const Point& a, const Point& b) {
    return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}
inline bool polygon_linear(const NurbsCurve& curve, std::array<Point, 2>& ends) {
    std::array<double, 3> start, direction;
    if (!event_line(curve, start, direction) || curve.weight(0) != curve.weight(1) ||
        !std::isfinite(curve.domain_end() - curve.domain_start()))
        return false;
    for (int i = 0; i < 2; ++i)
        for (int d = 0; d < 3; ++d)
            ends[i][d] = curve.cv(i)[d] / curve.weight(i);
    return true;
}
inline bool polygon_line_point(const NurbsCurve& curve, double t, Point& point) {
    std::array<Point, 2> ends;
    double width, left, right;
    if (!polygon_linear(curve, ends) || !std::isfinite(t) ||
        !event_difference(curve.domain_end(), curve.domain_start(), width) ||
        !event_difference(curve.domain_end(), t, left) || !event_difference(t, curve.domain_start(), right))
        return false;
    for (int d = 0; d < 3; ++d) {
        double numerator;
        if (!event_sum_products(left, ends[0][d], right, ends[1][d], numerator))
            return false;
        point[d] = numerator / width;
        if (!std::isfinite(point[d]) ||
            !nurbs_coincidence::exact_products_equal(point[d], width, numerator, 1))
            return false;
    }
    return true;
}
inline bool polygon_line_parameter(const NurbsCurve& curve, const Point& point, double& parameter) {
    std::array<double, 3> start, direction;
    if (!event_line(curve, start, direction) || curve.weight(0) != curve.weight(1))
        return false;
    int pivot = 0;
    while (pivot < 3 && direction[pivot] == 0)
        ++pivot;
    if (pivot == 3)
        return false;
    double delta, width, numerator;
    if (!event_difference(point[pivot], start[pivot], delta) ||
        !event_difference(curve.domain_end(), curve.domain_start(), width) ||
        !event_sum_products(curve.domain_start(), direction[pivot], width, delta, numerator))
        return false;
    parameter = numerator / direction[pivot];
    Point represented;
    return std::isfinite(parameter) &&
           nurbs_coincidence::exact_products_equal(parameter, direction[pivot], numerator, 1) &&
           polygon_line_point(curve, parameter, represented) && polygon_same_point(represented, point);
}
inline std::optional<PolygonTrimProof> polygon_boundary(const BRep& owner, int face, int operand,
                                                        double tolerance) {
    const auto original = brep_face_boundary::extract(owner, face);
    if (!original.complete || original.wires.empty())
        return std::nullopt;
    PolygonTrimProof result;
    for (const auto& wire : original.wires) {
        if (!brep_face_boundary::detail::active(wire.reference.orientation) || wire.uses.empty())
            return std::nullopt;
        std::vector<PolygonTrimEdge> loop;
        for (size_t i = 0; i < wire.uses.size(); ++i) {
            const auto& use = wire.uses[i];
            if (use.degenerated || !brep_face_boundary::detail::active(use.edge.orientation) ||
                use.pcurve < 0)
                return std::nullopt;
            const auto& edge = owner.m_edges[use.edge.index];
            PolygonTrimEdge item;
            item.use = {wire.reference.index, static_cast<int>(i), operand, face, use.edge.index,
                        use.pcurve,           use.edge.orientation};
            item.uv = &owner.m_curves_2d[use.pcurve];
            item.world = &owner.m_curves_3d[edge.curve_3d_index];
            std::array<Point, 2> world;
            if (!polygon_linear(*item.uv, item.ends) || !polygon_linear(*item.world, world) ||
                item.ends[0][2] != 0 || item.ends[1][2] != 0)
                return std::nullopt;
            const auto& surface = owner.m_surfaces[owner.m_faces[face].surface_index];
            for (int endpoint = 0; endpoint < 2; ++endpoint)
                for (int d = 0; d < 2; ++d) {
                    const auto domain = surface.domain(d);
                    if (item.ends[endpoint][d] < domain.first || item.ends[endpoint][d] > domain.second)
                        return std::nullopt;
                }
            if (curve_surface_intersection::same_parameter(*item.world, *item.uv, surface, tolerance) !=
                curve_surface_intersection::ParameterMatch::Matched)
                return std::nullopt;
            loop.push_back(std::move(item));
        }
        for (size_t i = 0; i < loop.size(); ++i) {
            const auto& a = loop[i];
            const auto& b = loop[(i + 1) % loop.size()];
            const int end = a.use.orientation == BRepOrientation::Reversed ? 0 : 1;
            const int start = b.use.orientation == BRepOrientation::Reversed ? 1 : 0;
            if (!polygon_same_point(a.ends[end], b.ends[start]))
                return std::nullopt;
        }
        result.wires.push_back(std::move(loop));
    }
    return result;
}
inline double polygon_side(const Point& a, const Point& b, const Point& point) {
    return exact_orientation::orient2d(a[0], a[1], b[0], b[1], point[0], point[1]);
}
inline bool polygon_strict_same_side(double a, double b) { return (a > 0 && b > 0) || (a < 0 && b < 0); }
inline bool polygon_add_event(PolygonTrimProof& proof, const PolygonTrimEdge& edge, const NurbsCurve& line,
                              double parameter, bool overlap) {
    Point point;
    double boundary;
    if (!polygon_line_point(line, parameter, point) || !polygon_line_parameter(*edge.uv, point, boundary) ||
        boundary < edge.uv->domain_start() || boundary > edge.uv->domain_end())
        return false;
    auto event = edge.use;
    event.section_parameter = parameter;
    event.boundary_parameter = boundary;
    event.overlap_endpoint = overlap;
    proof.events.push_back(std::move(event));
    return true;
}
inline bool polygon_edge_events(PolygonTrimProof& proof, const PolygonTrimEdge& edge,
                                const NurbsCurve& line) {
    std::array<Point, 2> points;
    if (!polygon_linear(line, points) || points[0][2] != 0 || points[1][2] != 0)
        return false;
    const double a = polygon_side(edge.ends[0], edge.ends[1], points[0]);
    const double b = polygon_side(edge.ends[0], edge.ends[1], points[1]);
    const double c = polygon_side(points[0], points[1], edge.ends[0]);
    const double d = polygon_side(points[0], points[1], edge.ends[1]);
    if (polygon_strict_same_side(a, b) || polygon_strict_same_side(c, d))
        return true;
    if (a == 0 && b == 0 && c == 0 && d == 0) {
        double first, last;
        if (!polygon_line_parameter(line, edge.ends[0], first) ||
            !polygon_line_parameter(line, edge.ends[1], last))
            return false;
        if (first > last)
            std::swap(first, last);
        first = std::max(first, line.domain_start());
        last = std::min(last, line.domain_end());
        if (first > last)
            return true;
        if (!polygon_add_event(proof, edge, line, first, first < last))
            return false;
        return first == last || polygon_add_event(proof, edge, line, last, true);
    }
    const auto crossing = exact_linear_crossing(line, *edge.uv);
    if (!crossing)
        return false; // A possible root without exact arithmetic is unqualified, never empty.
    for (int k = 0; k < 3; ++k)
        if (!nurbs_coincidence::exact_products_equal(crossing->center[k], crossing->weight,
                                                     crossing->homogeneous[k], 1))
            return false;
    double parameter;
    return polygon_line_parameter(line, crossing->center, parameter) && parameter >= line.domain_start() &&
           parameter <= line.domain_end() && polygon_add_event(proof, edge, line, parameter, false);
}
inline bool polygon_all_events(PolygonTrimProof& proof, const NurbsCurve& line) {
    for (const auto& wire : proof.wires)
        for (const auto& edge : wire)
            if (!polygon_edge_events(proof, edge, line))
                return false;
    return true;
}
inline brep_classify::State polygon_wire_state(const std::vector<PolygonTrimEdge>& wire, const Point& point) {
    int winding = 0;
    for (const auto& edge : wire) {
        const auto& a = edge.ends[0];
        const auto& b = edge.ends[1];
        const double side = polygon_side(a, b, point);
        if (side == 0 && point[0] >= std::min(a[0], b[0]) && point[0] <= std::max(a[0], b[0]) &&
            point[1] >= std::min(a[1], b[1]) && point[1] <= std::max(a[1], b[1]))
            return brep_classify::State::Boundary;
        int contribution = 0;
        if (a[1] <= point[1] && point[1] < b[1] && side > 0)
            contribution = 1;
        if (b[1] <= point[1] && point[1] < a[1] && side < 0)
            contribution = -1;
        winding += edge.use.orientation == BRepOrientation::Reversed ? -contribution : contribution;
    }
    return winding == 0 ? brep_classify::State::Outside : brep_classify::State::Inside;
}
inline std::optional<brep_classify::State> polygon_state(const PolygonTrimProof& proof,
                                                         const NurbsCurve& line, double t) {
    Point point;
    if (!polygon_line_point(line, t, point) || proof.wires.empty())
        return std::nullopt;
    const auto outer = polygon_wire_state(proof.wires[0], point);
    if (outer != brep_classify::State::Inside)
        return outer;
    for (size_t i = 1; i < proof.wires.size(); ++i) {
        const auto hole = polygon_wire_state(proof.wires[i], point);
        if (hole == brep_classify::State::Boundary)
            return hole;
        if (hole == brep_classify::State::Inside)
            return brep_classify::State::Outside;
    }
    return brep_classify::State::Inside; // Matches native first-outer, all-later-hole contract, not even/odd
                                         // fill.
}
} // namespace session_cpp::brep_section::detail
