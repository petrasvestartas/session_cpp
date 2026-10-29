#include "brep_boolean.h"
#include "brep_assemble.h"
#include "brep_classify.h"
#include "brep_section.h"
#include "curve_intersection.h"
#include "curve_surface_intersection.h"
#include "split.h"
#include "nurbs_bounds.h"
#include "nurbs_coincidence.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace session_cpp::brep_boolean {
namespace {

using State = brep_classify::State;

struct Partition {
    BRep brep;
    std::vector<split::EdgePiece> pieces;
    int operand;
};

bool occupied(bool a, bool b, Operation operation) {
    switch (operation) {
        case Operation::Union: return a || b;
        case Operation::Intersection: return a && b;
        case Operation::Difference: return a && !b;
        case Operation::Xor: return a != b;
    }
    throw std::invalid_argument("Unknown native BRep Boolean operation");
}

std::vector<int> active_faces(const BRep& brep) {
    std::vector<int> result;
    for (const BRepSolid& solid : brep.m_solids)
        for (const BRepRef& shell : solid.shells)
            for (const BRepRef& face : brep.m_shells[shell.index].faces) {
                const BRepOrientation orientation = brep_compose(shell.orientation, face.orientation);
                if (orientation == BRepOrientation::Forward || orientation == BRepOrientation::Reversed)
                    result.push_back(face.index);
            }
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

double uv_tolerance(const NurbsSurface& surface, double tolerance) {
    return tolerance / nurbs_bounds::surface_metric(surface);
}

bool covers(const NurbsCurve& candidate, const NurbsCurve& boundary, double tolerance) {
    const curve_intersection::Result events = curve_intersection::intersect(candidate, boundary, tolerance);
    if (!events.complete)
        throw std::runtime_error(events.failure);
    double last = candidate.domain_start();
    const double epsilon = 64 * std::numeric_limits<double>::epsilon() *
        std::max({std::abs(last), std::abs(candidate.domain_end()), candidate.domain_end() - last});
    for (const curve_intersection::Overlap& overlap : events.overlaps) {
        if (overlap.a0 > last + epsilon)
            return false;
        last = std::max(last, overlap.a1);
    }
    return last >= candidate.domain_end() - epsilon;
}

void append_cut(std::vector<split::FaceCut>& cuts, const BRep& brep, int face, split::FaceCut cut, double tolerance) {
    const double uv_tol = uv_tolerance(brep.m_surfaces[brep.m_faces[face].surface_index], tolerance);
    for (const BRepRef& wire : brep.m_faces[face].wires)
        for (const BRepRef& edge : brep.wire_edges(wire)) {
            const int index = brep.pcurve_index(edge.index, face, edge.orientation);
            if (index < 0)
                throw std::runtime_error("Native Boolean encountered a missing trim curve");
            if (covers(cut.pcurve, brep.m_curves_2d[index], uv_tol))
                return;
        }
    for (const split::FaceCut& previous : cuts)
        if (covers(cut.pcurve, previous.pcurve, uv_tol))
            return;
    cuts.push_back(std::move(cut));
}

void mapped_parameter_cuts(
    std::vector<split::FaceCut>& cuts,
    const BRep& target,
    int face,
    const BRep& other,
    int other_face,
    const nurbs_coincidence::ParameterMap& mapping,
    double tolerance
) {
    for (const BRepRef& wire : other.m_faces[other_face].wires)
        for (const BRepRef& use : other.wire_edges(wire)) {
            const BRepEdge& edge = other.m_edges[use.index];
            if (edge.degenerated)
                continue;
            const int index = other.pcurve_index(use.index, other_face, use.orientation);
            if (index < 0)
                throw std::runtime_error("Native Boolean encountered a missing coincident pcurve");
            NurbsCurve pcurve = other.m_curves_2d[index];
            if (!nurbs_coincidence::map_pcurve(mapping, pcurve))
                throw std::runtime_error("Native Boolean could not map a coincident boundary pcurve");
            NurbsCurve world = other.m_curves_3d[edge.curve_3d_index];
            if (!world.set_domain(pcurve.domain_start(), pcurve.domain_end()))
                throw std::runtime_error("Native Boolean could not synchronize a coincident boundary");
            if (curve_surface_intersection::same_parameter(world, pcurve,
                    target.m_surfaces[target.m_faces[face].surface_index], tolerance)
                != curve_surface_intersection::ParameterMatch::Matched)
                throw std::runtime_error("Native Boolean mapped boundary lacks a same-parameter certificate");
            append_cut(cuts, target, face, {world, pcurve, -1}, tolerance);
        }
}

void coincident_cuts(
    std::vector<split::FaceCut>& cuts,
    const BRep& target,
    int face,
    const BRep& other,
    const brep_section::FaceResult& sections,
    bool second,
    double tolerance
) {
    const NurbsSurface& surface = target.m_surfaces[target.m_faces[face].surface_index];
    std::vector<int> edges;
    for (const brep_section::FaceOverlap& overlap : sections.overlaps) {
        if ((second ? overlap.second_face : overlap.first_face) != face)
            continue;
        const int other_face = second ? overlap.first_face : overlap.second_face;
        const NurbsSurface& support = other.m_surfaces[other.m_faces[other_face].surface_index];
        nurbs_coincidence::ParameterMap mapping;
        if (nurbs_coincidence::same_parameters(surface, support, tolerance)
            || nurbs_coincidence::parameter_mapping(support, surface, tolerance, mapping)
            || nurbs_coincidence::common_basis_agreement(support, surface, tolerance)) {
            mapped_parameter_cuts(cuts, target, face, other, other_face, mapping, tolerance);
            continue;
        }
        if (!nurbs_coincidence::coplanar(surface, support, tolerance))
            throw std::runtime_error("Native Boolean has unsupported coincident surface parameterizations");
        for (const BRepRef& wire : other.m_faces[other_face].wires)
            for (const BRepRef& edge : other.wire_edges(wire))
                if (std::find(edges.begin(), edges.end(), edge.index) == edges.end())
                    edges.push_back(edge.index);
    }
    for (int edge : edges) {
        if (other.m_edges[edge].degenerated)
            continue;
        const NurbsCurve& world = other.m_curves_3d[other.m_edges[edge].curve_3d_index];
        const curve_surface_intersection::Result events = curve_surface_intersection::intersect(world, surface, tolerance);
        if (!events.complete)
            throw std::runtime_error(events.failure);
        for (const curve_surface_intersection::Overlap& overlap : events.overlaps) {
            NurbsCurve part = world;
            if (!part.trim(overlap.t0, overlap.t1))
                throw std::runtime_error("Native Boolean could not trim a coincident boundary");
            append_cut(cuts, target, face, {std::move(part), overlap.pcurve, -1}, tolerance);
        }
    }
}

void retain_lineage(Partition& result, split::FaceSplit next) {
    const size_t old_size = result.pieces.size();
    result.pieces.resize(next.brep.m_edges.size());
    for (size_t i = old_size; i < result.pieces.size(); ++i)
        result.pieces[i].edge = static_cast<int>(i);
    for (split::EdgePiece piece : next.edges) {
        if (piece.source_edge >= 0) {
            if (piece.source_edge >= static_cast<int>(old_size))
                throw std::runtime_error("Native Boolean split has an unresolved parent edge");
            const split::EdgePiece& parent = result.pieces[piece.source_edge];
            piece.source_edge = parent.source_edge;
            piece.section = parent.section;
        }
        result.pieces[piece.edge] = piece;
    }
    result.brep = std::move(next.brep);
}

Partition partition(
    const BRep& source,
    const BRep& other,
    const brep_section::FaceResult& sections,
    bool second,
    double tolerance
) {
    Partition result{source, {}, second ? 1 : 0};
    for (int i = 0; i < source.edge_count(); ++i) {
        const BRepEdge& edge = source.m_edges[i];
        const std::pair<double, double> domain = edge.degenerated ? std::pair<double, double>{0, 1}
            : source.m_curves_3d[edge.curve_3d_index].domain();
        result.pieces.push_back({i, -1, i, domain.first, domain.second});
    }
    for (int face : active_faces(source)) {
        std::vector<split::FaceCut> cuts;
        for (size_t i = 0; i < sections.curves.size(); ++i) {
            const brep_section::FaceCurve& section = sections.curves[i];
            if ((second ? section.second_face : section.first_face) == face)
                append_cut(cuts, result.brep, face, {section.world,
                    second ? section.second_pcurve : section.first_pcurve, static_cast<int>(i)}, tolerance);
        }
        coincident_cuts(cuts, result.brep, face, other, sections, second, tolerance);
        if (!cuts.empty())
            retain_lineage(result, split::split_brep_face_by_pcurves(result.brep, face, cuts, tolerance));
    }
    return result;
}

bool interior_candidate(
    const BRep& brep,
    int face,
    const Point& uv,
    double tolerance,
    const BRep* avoid
) {
    const NurbsSurface& surface = brep.m_surfaces[brep.m_faces[face].surface_index];
    if (brep_classify::face_uv(brep, face, uv, uv_tolerance(surface, tolerance)) != State::Inside)
        return false;
    if (!avoid)
        return true;
    const State state = brep_classify::point(*avoid, surface.point_at(uv[0], uv[1]), tolerance);
    if (state == State::Unknown)
        throw std::runtime_error("Native Boolean interior witness classification is unresolved");
    return state != State::Boundary;
}

Point interior(const BRep& brep, int face, double tolerance, const BRep* avoid = nullptr) {
    Point lo(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 0);
    Point hi(-lo[0], -lo[1], 0);
    std::vector<const NurbsCurve*> boundaries;
    for (const BRepRef& wire : brep.m_faces[face].wires)
        for (const BRepRef& edge : brep.wire_edges(wire)) {
            const int index = brep.pcurve_index(edge.index, face, edge.orientation);
            if (index < 0)
                throw std::runtime_error("Native Boolean encountered an untrimmed face");
            const NurbsCurve& curve = brep.m_curves_2d[index];
            boundaries.push_back(&curve);
            for (int i = 0; i < curve.cv_count(); ++i)
                for (int d = 0; d < 2; ++d) {
                    lo[d] = std::min(lo[d], curve.get_cv(i)[d]);
                    hi[d] = std::max(hi[d], curve.get_cv(i)[d]);
                }
        }
    for (int n : {2, 4, 8, 16})
        for (int i = 1; i < n; ++i)
            for (int j = 1; j < n; ++j) {
                const Point uv(lo[0] + (hi[0] - lo[0]) * i / n, lo[1] + (hi[1] - lo[1]) * j / n, 0);
                if (interior_candidate(brep, face, uv, tolerance, avoid))
                    return uv;
            }
    for (const NurbsCurve* boundary : boundaries) {
        const double t = (boundary->domain_start() + boundary->domain_end()) * 0.5;
        const std::vector<Vector> derivatives = boundary->evaluate(t, 1);
        Vector normal(-derivatives[1][1], derivatives[1][0], 0);
        if (!normal.normalize_self())
            continue;
        for (double fraction : {1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 0.1})
            for (double sign : {-1.0, 1.0}) {
                const Point uv = boundary->point_at(t) + normal * (sign * fraction * lo.distance(hi));
                if (interior_candidate(brep, face, uv, tolerance, avoid))
                    return uv;
            }
    }
    throw std::runtime_error("Native Boolean has no resolved interior witness or has unsupported coincidence");
}

std::vector<const NurbsCurve*> boundary_curves(const BRep& brep, int face) {
    std::vector<const NurbsCurve*> result;
    for (const BRepRef& wire : brep.m_faces[face].wires)
        for (const BRepRef& use : brep.wire_edges(wire)) {
            const BRepEdge& edge = brep.m_edges[use.index];
            if (!edge.degenerated)
                result.push_back(&brep.m_curves_3d[edge.curve_3d_index]);
        }
    return result;
}

bool boundary_covers(
    const std::vector<const NurbsCurve*>& target,
    const std::vector<const NurbsCurve*>& source,
    double tolerance
) {
    for (const NurbsCurve* curve : source) {
        std::vector<std::pair<double, double>> intervals;
        for (const NurbsCurve* candidate : target) {
            const curve_intersection::Result events = curve_intersection::intersect(*curve, *candidate, tolerance);
            if (!events.complete)
                throw std::runtime_error(events.failure);
            for (const curve_intersection::Overlap& overlap : events.overlaps)
                intervals.push_back({overlap.a0, overlap.a1});
        }
        std::sort(intervals.begin(), intervals.end());
        double last = curve->domain_start();
        const double epsilon = 128.0 * std::numeric_limits<double>::epsilon()
            * std::max({std::abs(last), std::abs(curve->domain_end()), curve->domain_end() - last});
        for (const std::pair<double, double>& interval : intervals) {
            if (interval.first > last + epsilon)
                return false;
            last = std::max(last, interval.second);
        }
        if (last < curve->domain_end() - epsilon)
            return false;
    }
    return true;
}

int coincident_region(
    const BRep& source,
    int face,
    const BRep& other,
    const Point& uv,
    Point& other_uv,
    double tolerance
) {
    const NurbsSurface& a = source.m_surfaces[source.m_faces[face].surface_index];
    const Point point = a.point_at(uv[0], uv[1]);
    for (int candidate : active_faces(other)) {
        const NurbsSurface& b = other.m_surfaces[other.m_faces[candidate].surface_index];
        nurbs_coincidence::ParameterMap mapping;
        const bool mapped = nurbs_coincidence::same_parameters(a, b, tolerance)
            || nurbs_coincidence::parameter_mapping(a, b, tolerance, mapping)
            || nurbs_coincidence::common_basis_agreement(a, b, tolerance);
        if (!mapped && !nurbs_coincidence::coplanar(a, b, tolerance))
            continue;
        const Point mapped_uv = mapping.apply(uv);
        const std::pair<double, double> parameters = mapped ? std::pair<double, double>{mapped_uv[0], mapped_uv[1]}
            : b.closest_parameters(point);
        if (b.point_at(parameters.first, parameters.second).distance(point) > tolerance
            || brep_classify::face_uv(other, candidate, {parameters.first, parameters.second, 0},
                uv_tolerance(b, tolerance)) != State::Inside)
            continue;
        const std::vector<const NurbsCurve*> first = boundary_curves(source, face);
        const std::vector<const NurbsCurve*> second = boundary_curves(other, candidate);
        if (!boundary_covers(first, second, tolerance) || !boundary_covers(second, first, tolerance))
            throw std::runtime_error("Native Boolean same-domain regions have unmatched partition boundaries");
        other_uv = {parameters.first, parameters.second, 0};
        return candidate;
    }
    return -1;
}

Vector oriented_normal(const BRep& brep, int face, const Point& uv) {
    const NurbsSurface& surface = brep.m_surfaces[brep.m_faces[face].surface_index];
    const std::vector<Vector> derivatives = surface.evaluate(uv[0], uv[1], 1);
    Vector normal = derivatives[2].cross(derivatives[1]);
    if (!normal.normalize_self())
        throw std::runtime_error("Native Boolean coincident region has a singular normal");
    if (brep.face_orientation(face) == BRepOrientation::Reversed)
        normal *= -1.0;
    return normal;
}

void select(
    std::vector<brep_assemble::FaceUse>& uses,
    const Partition& partition,
    const Partition& opposite,
    bool second,
    Operation operation,
    double tolerance
) {
    const BRep& source = partition.brep;
    const BRep& other = opposite.brep;
    for (int face : active_faces(source)) {
        Point uv = interior(source, face, tolerance);
        const NurbsSurface& surface = source.m_surfaces[source.m_faces[face].surface_index];
        Point p = surface.point_at(uv[0], uv[1]);
        State state = brep_classify::point(other, p, tolerance);
        if (state == State::Unknown)
            throw ClassificationError(other, p, face, second, tolerance);
        Point other_uv;
        const int coincident = state == State::Boundary ? coincident_region(source, face, other, uv, other_uv, tolerance) : -1;
        if (state == State::Boundary && coincident < 0) {
            uv = interior(source, face, tolerance, &other);
            p = surface.point_at(uv[0], uv[1]);
            state = brep_classify::point(other, p, tolerance);
            if (state != State::Inside && state != State::Outside)
                throw std::runtime_error("Native Boolean interior witness remained unresolved");
        }
        bool minus = false, plus = false;
        if (state != State::Boundary) {
            const bool inside = state == State::Inside;
            minus = second ? occupied(inside, true, operation) : occupied(true, inside, operation);
            plus = second ? occupied(inside, false, operation) : occupied(false, inside, operation);
        } else {
            if (second)
                continue;
            const double sense = oriented_normal(source, face, uv).dot(oriented_normal(other, coincident, other_uv));
            if (!std::isfinite(sense) || std::abs(sense) < 1.0 - 1e-8)
                throw std::runtime_error("Native Boolean could not resolve coincident face orientation");
            minus = occupied(true, sense > 0.0, operation);
            plus = occupied(false, sense < 0.0, operation);
        }
        if (minus != plus)
            uses.push_back({&source, face, plus, &partition.pieces, partition.operand});
    }
}

bool empty(const BRep& brep) {
    return brep.m_faces.empty() && brep.m_vertices.empty() && brep.m_edges.empty()
        && brep.m_wires.empty() && brep.m_shells.empty() && brep.m_solids.empty();
}

BRep empty_operation(const BRep& first, const BRep& second, Operation operation) {
    switch (operation) {
        case Operation::Union:
        case Operation::Xor:
            return empty(first) ? second : first;
        case Operation::Intersection:
            return {};
        case Operation::Difference:
            return empty(first) ? BRep() : first;
    }
    throw std::invalid_argument("Unknown native BRep Boolean operation");
}

}

BRep apply(const BRep& first, const BRep& second, Operation operation, double tolerance) {
    if (!std::isfinite(tolerance) || tolerance <= 0.0)
        throw std::invalid_argument("Native BRep Boolean tolerance must be finite and positive");
    for (const BRep* input : {&first, &second})
        if (!empty(*input) && (!input->is_valid() || !input->is_solid()))
            throw std::invalid_argument("Native BRep Booleans require closed oriented solid operands");
    if (empty(first) || empty(second))
        return empty_operation(first, second, operation);
    const brep_section::FaceResult sections = brep_section::faces(first, second, tolerance);
    if (!sections.complete)
        throw std::runtime_error(sections.failure);
    const Partition a = partition(first, second, sections, false, tolerance);
    const Partition b = partition(second, first, sections, true, tolerance);
    std::vector<brep_assemble::FaceUse> selected;
    select(selected, a, b, false, operation, tolerance);
    select(selected, b, a, true, operation, tolerance);
    return brep_assemble::assemble(selected, tolerance);
}

}
